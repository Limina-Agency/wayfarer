//! `wayfarer compile`: a routing package from an OpenStreetMap extract, with
//! human progress on stderr and the result as JSON on stdout, or the host
//! protocol's JSON events (`--protocol`, see `protocol.rs`).

use std::io::Read;
use std::num::NonZeroUsize;
use std::path::{Path, PathBuf};
use std::sync::Once;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use clap::Args;
use wayfarer_compiler::{
    CancelToken, CompileError, CompileProgress, CompileRequest, CompileStage, MemoryProfile,
    compile,
};

use crate::{Failure, print_json, protocol};

/// The exit status of a cancelled compile (as for SIGINT).
pub(crate) const CANCELLED_STATUS: i32 = 130;

/// Compile an OpenStreetMap extract (.osm.pbf) into a routing package.
///
/// Progress goes to stderr and the result (path, size, dataset hash, format
/// and costing versions, counts, build report) to stdout as JSON. Ctrl-C
/// cancels: the compile removes its scratch and partial files and exits with
/// status 130.
///
/// Instead of paths and flags, `--request` takes the compile as JSON
/// ({"input", "output", "scratchDir", "graphId", "country", "memory":
/// "standard"|"low", "threads"}), the same JSON the C ABI takes. With
/// `--protocol` the command speaks the host protocol (version 1): JSON events
/// on stdout, for applications that run the compiler as a child process; see
/// the wayfarer-cli README.
#[derive(Debug, Args)]
pub(crate) struct CompileArgs {
    /// The OpenStreetMap extract.
    #[arg(
        value_name = "input.osm.pbf",
        required_unless_present = "request",
        conflicts_with = "request"
    )]
    input: Option<PathBuf>,
    /// Where to write the package (replaced only by a complete one).
    #[arg(
        short,
        long,
        value_name = "routing.mrt",
        required_unless_present = "request",
        conflicts_with = "request"
    )]
    output: Option<PathBuf>,
    /// The ISO country code recorded in the package.
    #[arg(long, value_name = "CC", conflicts_with = "request")]
    country: Option<String>,
    /// The package's graph id [default: the input's name without .osm.pbf].
    #[arg(long, value_name = "ID", conflicts_with = "request")]
    graph_id: Option<String>,
    /// Where to keep scratch files; the compile works in a fresh directory
    /// inside it and removes that [default: the output's directory].
    #[arg(long, value_name = "DIR", conflicts_with = "request")]
    scratch_dir: Option<PathBuf>,
    /// Use less memory, more slowly (smaller ingest batches and queues). The
    /// package's bytes are the same.
    #[arg(long, conflicts_with = "request")]
    low_memory: bool,
    /// Threads for the compile [default: all cores, less two for the ingest].
    /// The package's bytes are the same for any count.
    #[arg(long, value_name = "N", conflicts_with = "request")]
    threads: Option<NonZeroUsize>,
    /// Take the compile as JSON from FILE, or from stdin with `-`.
    #[arg(long, value_name = "FILE")]
    request: Option<PathBuf>,
    /// Speak the host protocol: JSON events on stdout (needs --request).
    #[arg(long, requires = "request", conflicts_with_all = ["input", "output"])]
    protocol: bool,
    /// Host protocol: the parent holds stdin open; EOF cancels the compile.
    #[arg(long, requires = "protocol")]
    parent_stdin: bool,
}

impl CompileArgs {
    /// The request the paths and flags describe (`None` with `--request`).
    pub(crate) fn request_from_flags(&self) -> Option<CompileRequest> {
        let (input, output) = (self.input.clone()?, self.output.clone()?);
        Some(CompileRequest {
            input,
            output,
            scratch_dir: self.scratch_dir.clone(),
            graph_id: self.graph_id.clone(),
            country: self.country.clone(),
            memory: if self.low_memory {
                MemoryProfile::Low
            } else {
                MemoryProfile::Standard
            },
            threads: self.threads.map(NonZeroUsize::get),
        })
    }
}

pub(crate) fn run(args: CompileArgs) -> Result<(), Failure> {
    crate::purge_freed_memory_at_once();
    let cancel = CancelToken::new();
    cancel_on_signals(&cancel);
    if args.protocol {
        let source = args.request.as_deref().unwrap_or(Path::new("-"));
        return protocol::run(source, args.parent_stdin, &cancel);
    }
    let request = match (args.request_from_flags(), &args.request) {
        (Some(request), _) => request,
        (None, Some(source)) => read_request(source)?,
        (None, None) => return Err(Failure::new("compile needs an input and -o, or --request")),
    };
    let started = Instant::now();
    let mut printer = ProgressPrinter::new(started);
    match compile(&request, &cancel, |progress| printer.print(progress)) {
        Ok(result) => {
            eprintln!("compiled in {:.1} s", started.elapsed().as_secs_f64());
            print_json(&result)
        }
        Err(error) => Err(failure_of(error)),
    }
}

pub(crate) fn failure_of(error: CompileError) -> Failure {
    let status = if error.is_cancelled() {
        CANCELLED_STATUS
    } else {
        1
    };
    Failure {
        message: error.to_string(),
        status,
    }
}

/// Reads a request from `source` (`-`: stdin): a `CompileRequest`, or the host
/// protocol's request.
pub(crate) fn read_request(source: &Path) -> Result<CompileRequest, Failure> {
    let bytes = if source == Path::new("-") {
        let mut bytes = Vec::new();
        std::io::stdin()
            .read_to_end(&mut bytes)
            .map_err(|error| format!("Failed to read the compile request from stdin: {error}"))?;
        bytes
    } else {
        std::fs::read(source).map_err(|error| {
            format!(
                "Failed to read compile request {}: {error}",
                source.display()
            )
        })?
    };
    protocol::parse_request(&bytes).map_err(|error| {
        Failure::new(format!(
            "Invalid compile request {}: {error}",
            source.display()
        ))
    })
}

/// SIGINT and SIGTERM cancel the compile (it then removes its scratch and
/// partial files); a second signal, or a cancel that takes too long, exits at
/// once.
fn cancel_on_signals(cancel: &CancelToken) {
    let cancel = cancel.clone();
    let signals = AtomicUsize::new(0);
    // Without a handler (one is already set) the default behaviour stays.
    let _ = ctrlc::set_handler(move || {
        if signals.fetch_add(1, Ordering::Relaxed) == 0 {
            stop_soon(&cancel);
        } else {
            std::process::exit(CANCELLED_STATUS);
        }
    });
}

/// Cancels the compile, and exits if it has not stopped within 10 s (it polls
/// at least every second or so; cleanup is quick).
pub(crate) fn stop_soon(cancel: &CancelToken) {
    static WATCHDOG: Once = Once::new();
    cancel.cancel();
    WATCHDOG.call_once(|| {
        std::thread::spawn(|| {
            std::thread::sleep(Duration::from_secs(10));
            std::process::exit(CANCELLED_STATUS);
        });
    });
}

/// One stderr line per stage change, stage end, or quarter second.
struct ProgressPrinter {
    started: Instant,
    last_print: Option<Instant>,
    last_stage: Option<CompileStage>,
}

impl ProgressPrinter {
    fn new(started: Instant) -> Self {
        Self {
            started,
            last_print: None,
            last_stage: None,
        }
    }

    fn print(&mut self, progress: &CompileProgress) {
        let now = Instant::now();
        let due = self.last_stage != Some(progress.stage)
            || progress.stage_fraction == Some(1.0)
            || self
                .last_print
                .is_none_or(|last| now - last >= Duration::from_millis(250));
        if !due {
            return;
        }
        self.last_print = Some(now);
        self.last_stage = Some(progress.stage);
        eprintln!(
            "{:7.1} s {:3.0}% {} {}: {}",
            (now - self.started).as_secs_f64(),
            progress.overall * 100.0,
            progress.stage.as_str(),
            progress
                .stage_fraction
                .map(|fraction| format!("{:.0}%", fraction * 100.0))
                .unwrap_or_else(|| "-".into()),
            progress.message.as_deref().unwrap_or(""),
        );
    }
}

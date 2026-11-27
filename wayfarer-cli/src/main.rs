//! `wayfarer`: compiles OpenStreetMap extracts into routing packages,
//! inspects them, and routes on them. Each subcommand lives in its own module; `main` only
//! parses the command line and dispatches.

use clap::{Parser, Subcommand};

mod compile;
mod pace;
mod package;
mod protocol;
mod query;
mod terrain;

// mimalloc returns freed pages to the system between phases; the default
// allocator keeps much of a phase's peak resident. The profiling build
// installs its own counting allocator instead.
#[cfg(all(feature = "mimalloc", not(feature = "benchmark-profiling")))]
#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

/// Makes mimalloc return freed pages to the system at once rather than after
/// its default 10 ms: a compile frees each phase's working set in bursts on
/// several threads, and with the delay much of it stayed resident under the
/// next phase (Małopolskie at 4 threads: peak RSS 1,796 -> 1,284 MiB, the
/// live heap's peak, for no measurable time).
#[cfg(all(feature = "mimalloc", not(feature = "benchmark-profiling")))]
pub(crate) fn purge_freed_memory_at_once() {
    // `mi_option_purge_delay` in mimalloc v2 and v3's `mi_option_e`; the
    // bindings do not name it.
    const MI_OPTION_PURGE_DELAY: libmimalloc_sys::mi_option_t = 15;
    // SAFETY: setting an option is thread safe and takes any value.
    unsafe { libmimalloc_sys::mi_option_set(MI_OPTION_PURGE_DELAY, 0) };
}

#[cfg(not(all(feature = "mimalloc", not(feature = "benchmark-profiling"))))]
pub(crate) fn purge_freed_memory_at_once() {}

/// Offline routing packages from OpenStreetMap data.
#[derive(Debug, Parser)]
#[command(name = "wayfarer", version, about, propagate_version = true)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    Compile(compile::CompileArgs),
    Info(package::InfoArgs),
    Verify(package::VerifyArgs),
    Route(query::RouteArgs),
    Matrix(query::MatrixArgs),
    Nearest(query::NearestArgs),
    Isochrone(query::IsochroneArgs),
    #[command(name = "match")]
    Match(query::MatchArgs),
    Snap(query::SnapArgs),
    Pace(pace::PaceArgs),
    Terrain(terrain::TerrainArgs),
}

/// How a command failed: the message for stderr and the exit status.
pub(crate) struct Failure {
    pub(crate) message: String,
    pub(crate) status: i32,
}

impl Failure {
    pub(crate) fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            status: 1,
        }
    }
}

impl From<String> for Failure {
    fn from(message: String) -> Self {
        Self::new(message)
    }
}

/// Prints `value` as pretty JSON on stdout.
pub(crate) fn print_json(value: &impl serde::Serialize) -> Result<(), Failure> {
    let text = serde_json::to_string_pretty(value)
        .map_err(|error| Failure::new(format!("Failed to encode output: {error}")))?;
    println!("{text}");
    Ok(())
}

fn main() {
    let cli = Cli::parse();
    let result = match cli.command {
        Command::Compile(args) => compile::run(args),
        Command::Info(args) => package::info(args),
        Command::Verify(args) => package::verify(args),
        Command::Route(args) => query::route(args),
        Command::Matrix(args) => query::matrix(args),
        Command::Nearest(args) => query::nearest(args),
        Command::Isochrone(args) => query::isochrone(args),
        Command::Match(args) => query::map_match(args),
        Command::Snap(args) => query::snap(args),
        Command::Pace(args) => pace::run(args),
        Command::Terrain(args) => terrain::run(args),
    };
    if let Err(failure) = result {
        if !failure.message.is_empty() {
            eprintln!("error: {}", failure.message);
        }
        std::process::exit(failure.status);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;

    #[test]
    fn the_command_line_is_well_formed() {
        Cli::command().debug_assert();
    }

    fn parse(args: &[&str]) -> Result<Cli, clap::Error> {
        Cli::try_parse_from(std::iter::once("wayfarer").chain(args.iter().copied()))
    }

    #[test]
    fn queries_take_points_or_a_request() {
        assert!(parse(&["route", "a.mrt", "--from", "50.1,19.9", "--to", "50.0,19.8"]).is_ok());
        assert!(
            parse(&[
                "route",
                "a.mrt",
                "--from",
                "50.1,19.9",
                "--via",
                "50.05,19.85",
                "--via",
                "50.02,19.83",
                "--to",
                "-50.0,-19.8",
                "--profile",
                "on_foot",
                "--geojson"
            ])
            .is_ok()
        );
        assert!(parse(&["route", "a.mrt", "--request", "-"]).is_ok());
        assert!(parse(&["route", "a.mrt", "--from", "50.1,19.9"]).is_err());
        assert!(parse(&["route", "a.mrt", "--from", "50.1", "--to", "50.0,19.8"]).is_err());
        assert!(
            parse(&[
                "route",
                "a.mrt",
                "--from",
                "1,1",
                "--to",
                "2,2",
                "--profile",
                "car"
            ])
            .is_err()
        );
        assert!(
            parse(&[
                "matrix", "a.mrt", "--source", "1,1", "--target", "2,2", "--target", "3,3"
            ])
            .is_ok()
        );
        assert!(parse(&["isochrone", "a.mrt", "--from", "1,1", "--minutes", "5"]).is_ok());
        assert!(
            parse(&[
                "isochrone",
                "a.mrt",
                "--from",
                "1,1",
                "--distance-m",
                "500",
                "--distance-m",
                "1000"
            ])
            .is_ok()
        );
        assert!(
            parse(&[
                "isochrone",
                "a.mrt",
                "--from",
                "1,1",
                "--minutes",
                "5",
                "--distance-m",
                "500"
            ])
            .is_ok()
        );
        assert!(parse(&["isochrone", "a.mrt", "--from", "1,1"]).is_err());
        assert!(parse(&["isochrone", "a.mrt", "--request", "-"]).is_ok());
        assert!(
            parse(&[
                "nearest", "a.mrt", "--source", "1,1", "--target", "2,2", "--target", "3,3", "-k",
                "2"
            ])
            .is_ok()
        );
        assert!(
            parse(&[
                "nearest",
                "a.mrt",
                "--source",
                "1,1",
                "--target",
                "2,2",
                "--direction",
                "to_source"
            ])
            .is_ok()
        );
        assert!(parse(&["nearest", "a.mrt", "--request", "nearest.json"]).is_ok());
        assert!(parse(&["nearest", "a.mrt", "--source", "1,1"]).is_err());
        assert!(
            parse(&[
                "nearest", "a.mrt", "--source", "1,1", "--target", "2,2", "-k", "0"
            ])
            .is_err()
        );
        assert!(parse(&["snap", "a.mrt", "--at", "1,1"]).is_ok());
        assert!(parse(&["match", "a.mrt", "--request", "trace.json"]).is_ok());
        assert!(parse(&["match", "a.mrt"]).is_err());
    }

    #[test]
    fn compile_takes_paths_and_options_or_a_request() {
        let Command::Compile(args) = parse(&[
            "compile",
            "monaco.osm.pbf",
            "-o",
            "routing.mrt",
            "--country",
            "MC",
            "--graph-id",
            "mc",
            "--scratch-dir",
            "work",
            "--low-memory",
            "--threads",
            "2",
        ])
        .unwrap()
        .command
        else {
            panic!("not compile");
        };
        let request = args.request_from_flags().unwrap();
        assert_eq!(
            request,
            wayfarer_compiler::CompileRequest::new("monaco.osm.pbf", "routing.mrt")
                .country("MC")
                .graph_id("mc")
                .scratch_dir("work")
                .memory(wayfarer_compiler::MemoryProfile::Low)
                .threads(2)
        );

        assert!(parse(&["compile", "--request", "request.json"]).is_ok());
        assert!(
            parse(&[
                "compile",
                "--protocol",
                "--request",
                "r.json",
                "--parent-stdin"
            ])
            .is_ok()
        );
        // A request excludes the flags, the protocol needs a request, and
        // paths need an output.
        assert!(parse(&["compile", "--request", "r.json", "--country", "PL"]).is_err());
        assert!(parse(&["compile", "a.osm.pbf", "-o", "b.mrt", "--request", "r.json"]).is_err());
        assert!(parse(&["compile", "--protocol", "a.osm.pbf", "-o", "b.mrt"]).is_err());
        assert!(parse(&["compile", "a.osm.pbf"]).is_err());
        assert!(parse(&["compile", "a.osm.pbf", "-o", "b.mrt", "--threads", "0"]).is_err());
        assert!(parse(&["compile", "a.osm.pbf", "-o", "b.mrt", "--mobile"]).is_err());
    }

    #[test]
    fn info_and_verify_take_one_package() {
        assert!(parse(&["info", "routing.mrt"]).is_ok());
        assert!(parse(&["verify", "routing.mrt"]).is_ok());
        assert!(parse(&["info"]).is_err());
        assert!(parse(&["verify", "a.mrt", "b.mrt"]).is_err());
    }
}

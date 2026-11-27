//! `wayfarer-terrain`: compiles terrain files from the GEM-15, GLO-30 and WorldCover tiles a
//! host holds, and inspects them. It joins the `wayfarer` binary once the terrain crates
//! publish.

use clap::{Parser, Subcommand};
use std::alloc::{GlobalAlloc, Layout, System};
use std::io::IsTerminal;
use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::atomic::{AtomicUsize, Ordering};
use wayfarer_terrain::lattice::Spacing;
use wayfarer_terrain::terrain::{DEM_NONE, STEEP_NONE, Terrain};
use wayfarer_terrain_compiler::{
    CancelToken, Error, Options, Progress, Region, Sources, compile_to_file,
};

/// Counts heap bytes, for the report's peak heap. Two relaxed atomics an allocation; the
/// compile allocates a few buffers a block.
struct Counting;
static NOW: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);

// SAFETY: forwards to the system allocator unchanged and only counts.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        let p = unsafe { System.alloc(l) };
        if !p.is_null() {
            let now = NOW.fetch_add(l.size(), Ordering::Relaxed) + l.size();
            PEAK.fetch_max(now, Ordering::Relaxed);
        }
        p
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        unsafe { System.dealloc(p, l) };
        NOW.fetch_sub(l.size(), Ordering::Relaxed);
    }
}

#[global_allocator]
static A: Counting = Counting;

/// Off-road terrain files for Wayfarer.
#[derive(Debug, Parser)]
#[command(name = "wayfarer-terrain", version, about)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Compiles a terrain file from DEM and land-cover COG tiles and prints its report as JSON.
    ///
    /// Each `--heights` is one tier of height sources, best first: the posts GEM-15 covers
    /// take GEM-15, the rest GLO-30. Paths are `.tif` files or directories of them; a tier of
    /// several joins them as PATH does (`:` here, `;` on Windows).
    #[command(
        verbatim_doc_comment,
        after_long_help = "Example:\n\n    wayfarer-terrain compile --heights gem15/ --heights glo30/ \\\n        --land-cover worldcover/ --bbox 49.2,19.84,49.38,20.12 -o tatra.wtr"
    )]
    Compile {
        /// A height tier, repeated for each: files or directories of tiles (GEM-15 first,
        /// then GLO-30).
        #[arg(long, value_name = "PATHS", required = true, value_parser = parse_tier)]
        heights: Vec<Vec<PathBuf>>,
        /// ESA WorldCover tiles: files or directories.
        #[arg(long, value_name = "PATH", num_args = 1..)]
        land_cover: Vec<PathBuf>,
        /// The region, in degrees: south,west,north,east.
        #[arg(long, value_name = "S,W,N,E", value_parser = parse_bbox)]
        bbox: Region,
        /// The output terrain file.
        #[arg(short, long, value_name = "FILE")]
        output: PathBuf,
        /// Post spacing in ticks of 1/120″ (a multiple of 15; 120 is 1″).
        #[arg(long, default_value_t = 120)]
        spacing: u32,
        /// Worker threads (all cores by default). Never changes the file.
        #[arg(long)]
        threads: Option<usize>,
        /// Most MiB of decoded tiles held at once. Never changes the file.
        #[arg(long, default_value_t = 64)]
        cache_mib: usize,
        /// Replace the default attribution in the file's sources text with this file's text.
        #[arg(long, value_name = "FILE")]
        attribution: Option<PathBuf>,
        /// No progress on stderr.
        #[arg(long, short)]
        quiet: bool,
    },
    /// Prints a terrain file's plane statistics as JSON.
    Inspect {
        /// The terrain file.
        file: PathBuf,
    },
}

fn parse_tier(s: &str) -> Result<Vec<PathBuf>, String> {
    let v: Vec<PathBuf> = std::env::split_paths(s)
        .filter(|p| !p.as_os_str().is_empty())
        .collect();
    if v.is_empty() {
        return Err("no path".into());
    }
    Ok(v)
}

fn parse_bbox(s: &str) -> Result<Region, String> {
    let v: Vec<f64> = s
        .split(',')
        .map(|x| x.trim().parse::<f64>().map_err(|e| format!("{x}: {e}")))
        .collect::<Result<_, _>>()?;
    let [south, west, north, east] = v[..] else {
        return Err("four numbers: south,west,north,east".into());
    };
    Ok(Region::Bbox {
        south,
        west,
        north,
        east,
    })
}

fn main() -> ExitCode {
    match run(Cli::parse().command) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("wayfarer-terrain: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run(command: Command) -> Result<(), Box<dyn std::error::Error>> {
    match command {
        Command::Compile {
            heights,
            land_cover,
            bbox,
            output,
            spacing,
            threads,
            cache_mib,
            attribution,
            quiet,
        } => {
            let mut sources = Sources::from_paths(&heights, &land_cover)?;
            if let Some(a) = attribution {
                sources.attribution =
                    std::fs::read_to_string(&a).map_err(|e| format!("{}: {e}", a.display()))?;
            }
            let options = Options {
                spacing: Spacing::new(spacing as i64)?,
                cache_bytes: cache_mib << 20,
                threads,
            };
            let cancel = CancelToken::new();
            let token = cancel.clone();
            ctrlc::set_handler(move || token.cancel())?;
            let tty = std::io::stderr().is_terminal();
            let mut last = -1i64;
            let report = compile_to_file(sources, &bbox, &options, &output, &cancel, |p| {
                if !quiet {
                    show(p, tty, &mut last)
                }
            });
            if !quiet && tty {
                eprintln!();
            }
            let r = match report {
                Err(Error::Cancelled) => return Err("cancelled; no file written".into()),
                r => r?,
            };
            println!(
                "{{\"file\": {}, \"blocks\": {}, \"posts\": {}, \"heights_by_tier\": {:?}, \"posts_without_height\": {}, \"steep_from_later_tier\": {}, \"posts_with_cover\": {}, \"posts_with_confident_cover\": {}, \"tiles_decoded\": {}, \"cache_peak_bytes\": {}, \"peak_heap_bytes\": {}, \"file_bytes\": {}, \"terrain_hash\": \"{}\", \"seconds\": {:.2}}}",
                json_str(&output.display().to_string()),
                r.blocks,
                r.posts,
                r.heights_by_tier,
                r.posts_without_height,
                r.steep_from_later_tier,
                r.posts_with_cover,
                r.posts_with_confident_cover,
                r.tiles_decoded,
                r.cache_peak_bytes,
                PEAK.load(Ordering::Relaxed),
                r.file_bytes,
                wayfarer_terrain::hex(&r.terrain_hash),
                r.seconds
            );
            Ok(())
        }
        Command::Inspect { file } => {
            let bytes = std::fs::read(&file).map_err(|e| format!("{}: {e}", file.display()))?;
            inspect(&Terrain::read(&bytes)?);
            Ok(())
        }
    }
}

/// Progress on stderr: a line redrawn in place on a terminal, every 10 % otherwise.
fn show(p: Progress, tty: bool, last: &mut i64) {
    let pct = (p.fraction() * 100.0) as i64;
    if tty {
        eprint!("\r{pct:3}%  {} of {} blocks", p.blocks_done, p.blocks_total);
    } else if pct / 10 != *last / 10 {
        eprintln!("{pct}%  {} of {} blocks", p.blocks_done, p.blocks_total);
    }
    *last = pct;
}

fn inspect(t: &Terrain) {
    let (mut steep, mut cover) = ([0u64; 16], [0u64; 16]);
    let (mut lo, mut hi) = (i32::MAX, i32::MIN);
    let (mut none, mut no_steep, mut conf, mut lines) = (0u64, 0u64, 0u64, 0u64);
    for k in 0..t.blocks().len() {
        for (i, &d) in t.dm(k).iter().enumerate() {
            if d == DEM_NONE {
                none += 1;
            } else {
                lo = lo.min(d);
                hi = hi.max(d);
            }
            match t.steep(k)[i] {
                STEEP_NONE => no_steep += 1,
                s => steep[s as usize] += 1,
            }
            cover[t.cover(k)[i] as usize] += 1;
            conf += t.conf(k)[i] as u64;
            lines += (t.lines(k)[i] != 0) as u64;
        }
    }
    let posts = t.blocks().len() as u64 * wayfarer_terrain::lattice::BLOCK_POSTS as u64;
    let range = if lo <= hi {
        format!("[{lo}, {hi}]")
    } else {
        "null".into()
    };
    println!(
        "{{\"spacing_ticks\": {}, \"blocks\": {}, \"posts\": {posts}, \"dm_min_max\": {range}, \"posts_without_height\": {none}, \"posts_without_steep\": {no_steep}, \"steep_hist\": {steep:?}, \"cover_hist\": {cover:?}, \"posts_with_confident_cover\": {conf}, \"posts_with_lines\": {lines}, \"terrain_hash\": \"{}\", \"sources\": {}}}",
        t.spacing().ticks(),
        t.blocks().len(),
        wayfarer_terrain::hex(&t.hash()),
        json_str(t.sources())
    );
}

fn json_str(s: &str) -> String {
    let mut o = String::with_capacity(s.len() + 2);
    o.push('"');
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            '\n' => o.push_str("\\n"),
            c if (c as u32) < 0x20 => o.push_str(&format!("\\u{:04x}", c as u32)),
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

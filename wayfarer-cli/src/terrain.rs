//! `wayfarer terrain`: off-road terrain files. `compile` builds one from the DEM, land-cover
//! and OSM sources a host holds (`wayfarer-terrain-compiler`); `inspect` prints a file's plane
//! statistics.

use std::io::IsTerminal;
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};

use clap::{Args, Subcommand};
use serde_json::json;
use wayfarer_terrain::lattice::{BLOCK_POSTS, Spacing};
use wayfarer_terrain::lines::BIT_NAMES;
use wayfarer_terrain::terrain::{DEM_NONE, STEEP_NONE, Terrain};
use wayfarer_terrain_compiler::{
    CancelToken, Error, Options, Progress, Region, Sources, Stage, compile_to_file,
};

use crate::compile::CANCELLED_STATUS;
use crate::{Failure, print_json};

/// Off-road terrain files: compile one from DEM, land-cover and OSM sources, or inspect one.
#[derive(Debug, Args)]
pub(crate) struct TerrainArgs {
    #[command(subcommand)]
    command: TerrainCommand,
}

#[derive(Debug, Subcommand)]
enum TerrainCommand {
    Compile(CompileArgs),
    /// Print a terrain file's plane statistics as JSON.
    Inspect {
        /// The terrain file.
        file: PathBuf,
    },
}

/// Compile a terrain file from DEM and land-cover COG tiles and an OSM extract; print its
/// report as JSON.
///
/// Each `--heights` is one tier of height sources, best first: the posts GEM-15 covers take
/// GEM-15, the rest GLO-30. Paths are `.tif` files or directories of them; a tier of
/// several joins them as PATH does (`:` here, `;` on Windows). The output is written
/// whole or not at all: a failure or Ctrl-C leaves no file.
#[derive(Debug, Args)]
#[command(
    verbatim_doc_comment,
    after_long_help = "Example:\n\n    wayfarer terrain compile --heights gem15/ --heights glo30/ \\\n        --land-cover worldcover/ --osm malopolskie.osm.pbf \\\n        --bbox 49.2,19.84,49.38,20.12 -o tatra.wtr"
)]
pub(crate) struct CompileArgs {
    /// A height tier, repeated for each: files or directories of tiles (GEM-15 first, then
    /// GLO-30).
    #[arg(long, value_name = "PATHS", required = true, value_parser = parse_tier)]
    heights: Vec<Vec<PathBuf>>,
    /// ESA WorldCover tiles: files or directories.
    #[arg(long, value_name = "PATH", num_args = 1..)]
    land_cover: Vec<PathBuf>,
    /// An OSM extract (.osm.pbf) for the land-cover override and the linear features.
    #[arg(long, value_name = "FILE")]
    osm: Option<PathBuf>,
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
    /// Where the OSM node store goes while the extract is read (the system's temporary
    /// directory by default).
    #[arg(long, value_name = "DIR")]
    scratch_dir: Option<PathBuf>,
    /// Replace the default attribution in the file's sources text with this file's text.
    #[arg(long, value_name = "FILE")]
    attribution: Option<PathBuf>,
    /// No progress on stderr.
    #[arg(long, short)]
    quiet: bool,
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

pub(crate) fn run(args: TerrainArgs) -> Result<(), Failure> {
    match args.command {
        TerrainCommand::Compile(args) => compile(args),
        TerrainCommand::Inspect { file } => {
            let bytes = std::fs::read(&file)
                .map_err(|e| Failure::new(format!("{}: {e}", file.display())))?;
            let t = Terrain::read(&bytes)
                .map_err(|e| Failure::new(format!("{}: {e}", file.display())))?;
            print_json(&inspect(&t))
        }
    }
}

fn compile(a: CompileArgs) -> Result<(), Failure> {
    let fail = |e: Error| Failure::new(e.to_string());
    let mut sources = Sources::from_paths(&a.heights, &a.land_cover).map_err(fail)?;
    if let Some(pbf) = a.osm {
        sources = sources.with_osm(pbf);
    }
    if let Some(f) = a.attribution {
        sources.attribution = std::fs::read_to_string(&f)
            .map_err(|e| Failure::new(format!("{}: {e}", f.display())))?;
    }
    let options = Options {
        spacing: Spacing::new(a.spacing as i64).map_err(|e| Failure::new(e.to_string()))?,
        cache_bytes: a.cache_mib << 20,
        threads: a.threads,
        scratch_dir: a.scratch_dir,
    };
    let cancel = CancelToken::new();
    let token = cancel.clone();
    let signals = AtomicUsize::new(0);
    // A second signal exits at once; without a handler (one is set) the default stays.
    let _ = ctrlc::set_handler(move || {
        if signals.fetch_add(1, Ordering::Relaxed) == 0 {
            token.cancel();
        } else {
            std::process::exit(CANCELLED_STATUS);
        }
    });
    let tty = std::io::stderr().is_terminal();
    let mut last = -1i64;
    let report = compile_to_file(sources, &a.bbox, &options, &a.output, &cancel, |p| {
        if !a.quiet {
            show(p, tty, &mut last)
        }
    });
    if !a.quiet && tty {
        eprintln!();
    }
    let r = match report {
        Err(Error::Cancelled) => {
            return Err(Failure {
                message: "cancelled; no file written".into(),
                status: CANCELLED_STATUS,
            });
        }
        r => r.map_err(fail)?,
    };
    print_json(&json!({
        "file": a.output.display().to_string(),
        "blocks": r.blocks,
        "posts": r.posts,
        "heights_by_tier": r.heights_by_tier,
        "posts_without_height": r.posts_without_height,
        "steep_from_later_tier": r.steep_from_later_tier,
        "posts_with_cover": r.posts_with_cover,
        "posts_with_confident_cover": r.posts_with_confident_cover,
        "posts_with_osm_cover": r.posts_with_osm_cover,
        "posts_with_lines": r.posts_with_lines,
        "osm": r.osm.as_ref().map(|o| json!({
            "areas": o.areas,
            "multipolygons": o.multipolygons,
            "rings_joined": o.rings_joined,
            "rings_dropped": o.rings_dropped,
            "lines": o.lines,
        })),
        "tiles_decoded": r.tiles_decoded,
        "cache_peak_bytes": r.cache_peak_bytes,
        "file_bytes": r.file_bytes,
        "terrain_hash": wayfarer_terrain::hex(&r.terrain_hash),
        "seconds": (r.seconds * 100.0).round() / 100.0,
    }))
}

/// Progress on stderr: a line redrawn in place on a terminal, every 10 % otherwise.
fn show(p: Progress, tty: bool, last: &mut i64) {
    if p.stage == Stage::ReadingOsm {
        if tty {
            eprint!("\rreading the OSM extract");
        } else {
            eprintln!("reading the OSM extract");
        }
        return;
    }
    let pct = (p.fraction() * 100.0) as i64;
    if tty {
        eprint!(
            "\r{pct:3}%  {} of {} blocks        ",
            p.blocks_done, p.blocks_total
        );
    } else if pct / 10 != *last / 10 {
        eprintln!("{pct}%  {} of {} blocks", p.blocks_done, p.blocks_total);
    }
    *last = pct;
}

fn inspect(t: &Terrain) -> serde_json::Value {
    let (mut steep, mut cover, mut by_bit) = ([0u64; 16], [0u64; 16], [0u64; 16]);
    let (mut lo, mut hi) = (i32::MAX, i32::MIN);
    let (mut none, mut no_steep, mut conf, mut lines) = (0u64, 0u64, 0u64, 0u64);
    for k in 0..t.blocks().len() {
        let (dm, st, cv, cf, ln) = (t.dm(k), t.steep(k), t.cover(k), t.conf(k), t.lines(k));
        for i in 0..BLOCK_POSTS {
            if dm[i] == DEM_NONE {
                none += 1;
            } else {
                lo = lo.min(dm[i]);
                hi = hi.max(dm[i]);
            }
            match st[i] {
                STEEP_NONE => no_steep += 1,
                s => steep[s as usize] += 1,
            }
            cover[cv[i] as usize] += 1;
            conf += cf[i] as u64;
            lines += (ln[i] != 0) as u64;
            for (b, n) in by_bit.iter_mut().enumerate() {
                *n += (ln[i] >> b & 1) as u64;
            }
        }
    }
    let lines_by_kind: serde_json::Map<String, serde_json::Value> = BIT_NAMES
        .iter()
        .zip(by_bit)
        .filter(|(_, n)| *n > 0)
        .map(|(name, n)| (name.to_string(), json!(n)))
        .collect();
    json!({
        "spacing_ticks": t.spacing().ticks(),
        "blocks": t.blocks().len(),
        "posts": t.blocks().len() * BLOCK_POSTS,
        "dm_min_max": (lo <= hi).then_some([lo, hi]),
        "posts_without_height": none,
        "posts_without_steep": no_steep,
        "steep_hist": steep,
        "cover_hist": cover,
        "posts_with_confident_cover": conf,
        "posts_with_lines": lines,
        "lines_by_kind": lines_by_kind,
        "terrain_hash": wayfarer_terrain::hex(&t.hash()),
        "sources": t.sources(),
    })
}

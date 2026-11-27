//! Compiles a terrain file from directories of tiles, for measurements on real data.
//!
//! ```text
//! cargo run --release -p wayfarer-terrain-compiler --example compile_terrain -- \
//!     --heights gem15/ --heights glo30/ --land-cover worldcover/ \
//!     --bbox 49.2,19.84,49.38,20.12 [--spacing 120] [--threads N] [--cache-mib 64] -o out.wtr
//! ```
//!
//! Each `--heights` directory is one tier, best first. Prints the report as JSON.

use std::alloc::{GlobalAlloc, Layout, System};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};
use wayfarer_terrain_compiler::{Options, Region, SourceFile, Sources, attribution, compile};

/// Counts heap bytes, to report the compile's peak heap.
struct Counting;
static NOW: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        let p = unsafe { System.alloc(l) };
        if !p.is_null() {
            PEAK.fetch_max(
                NOW.fetch_add(l.size(), Ordering::Relaxed) + l.size(),
                Ordering::Relaxed,
            );
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

fn tifs(dir: &Path) -> Vec<PathBuf> {
    let mut v: Vec<PathBuf> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("{}: {e}", dir.display()))
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "tif" || x == "tiff"))
        .collect();
    v.sort();
    v
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let (mut heights, mut cover, mut bbox, mut out) = (Vec::new(), Vec::new(), None, None);
    let mut opts = Options::default();
    while let Some(a) = args.next() {
        let mut val = || args.next().ok_or(format!("{a} needs a value"));
        match a.as_str() {
            "--heights" => heights.push(PathBuf::from(val()?)),
            "--land-cover" => cover.push(PathBuf::from(val()?)),
            "--bbox" => {
                let v: Vec<f64> = val()?
                    .split(',')
                    .map(|x| x.parse())
                    .collect::<Result<_, _>>()?;
                bbox = Some(Region::Bbox {
                    south: v[0],
                    west: v[1],
                    north: v[2],
                    east: v[3],
                });
            }
            "--spacing" => opts.spacing = wayfarer_terrain::lattice::Spacing::new(val()?.parse()?)?,
            "--threads" => opts.threads = Some(val()?.parse()?),
            "--cache-mib" => opts.cache_bytes = val()?.parse::<usize>()? << 20,
            "-o" => out = Some(PathBuf::from(val()?)),
            _ => return Err(format!("unknown argument {a}").into()),
        }
    }
    let open = |dirs: &[PathBuf]| -> Result<Vec<SourceFile>, Box<dyn std::error::Error>> {
        let mut v = Vec::new();
        for d in dirs {
            for p in tifs(d) {
                v.push(SourceFile::open(p)?);
            }
        }
        Ok(v)
    };
    let mut tiers = Vec::new();
    for d in &heights {
        tiers.push(open(std::slice::from_ref(d))?);
    }
    let sources = Sources {
        heights: tiers,
        land_cover: open(&cover)?,
        attribution: attribution::ALL.join("\n"),
    };
    let out = out.ok_or("-o is required")?;
    let file = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(true)
        .open(&out)?;
    let (r, _) = compile(
        sources,
        &bbox.ok_or("--bbox is required")?,
        &opts,
        std::io::BufWriter::new(file),
    )?;
    let peak = PEAK.load(Ordering::Relaxed);
    println!(
        "{{\"peak_heap_bytes\": {peak}, \"blocks\": {}, \"posts\": {}, \"heights_by_tier\": {:?}, \"posts_without_height\": {}, \"steep_from_later_tier\": {}, \"posts_with_cover\": {}, \"posts_with_confident_cover\": {}, \"tiles_decoded\": {}, \"cache_peak_bytes\": {}, \"file_bytes\": {}, \"terrain_hash\": \"{}\", \"seconds\": {:.2}}}",
        r.blocks,
        r.posts,
        r.heights_by_tier,
        r.posts_without_height,
        r.steep_from_later_tier,
        r.posts_with_cover,
        r.posts_with_confident_cover,
        r.tiles_decoded,
        r.cache_peak_bytes,
        r.file_bytes,
        wayfarer_terrain::hex(&r.terrain_hash),
        r.seconds
    );
    Ok(())
}

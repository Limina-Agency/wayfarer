//! Stage 1 of Wayfarer's off-road terrain: compiles DEM and land-cover COG tiles and an OSM
//! extract into a terrain file (`wayfarer_terrain::terrain`) on the global lattice.
//!
//! The heights come from tiers of DEM sources, best first: GEM-15 (Google's 15 m DTM, the
//! squares an app may already hold and draw), then Copernicus GLO-30 for its gaps. The
//! steepness-max class comes from the same full-resolution pixels, and the land cover from
//! ESA WorldCover, overridden by OSM areas (multipolygons included) where they say more. OSM
//! ways give the linear features: roads and paths, rivers and streams, railways, fences,
//! cliffs. [`compile()`] reads only the tiles it needs, holds decoded tiles in a cache of
//! bounded size, works one block row (64 posts, about 2 km at 1″) at a time and streams the
//! file out. The rules of every step are in the [`compile`](mod@compile) and [`osm`] module
//! docs; they are integer arithmetic but for the UTM projection, which uses `libm`, so the
//! same inputs give the same file on every target and thread count.
//!
//! A host compiles the terrain file from the tiles it already holds:
//!
//! ```no_run
//! use wayfarer_terrain_compiler::{CancelToken, Options, Region, Sources, compile_to_file};
//!
//! // Files or directories of `.tif` tiles; each inner list is one height tier, best first.
//! let sources = Sources::from_paths(&[vec!["tiles/gem15"], vec!["tiles/glo30"]], &["tiles/worldcover"])?
//!     .with_osm("malopolskie.osm.pbf");
//! let region = Region::Bbox { south: 49.2, west: 19.84, north: 49.38, east: 20.12 };
//! let cancel = CancelToken::new(); // `cancel.cancel()` from any thread stops it
//! let report = compile_to_file(sources, &region, &Options::default(), "tatra.wtr", &cancel, |p| {
//!     eprintln!("{:.0}%", p.fraction() * 100.0)
//! })?;
//! println!("{} blocks, {} bytes", report.blocks, report.file_bytes);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! [`compile()`] writes to any seekable sink and takes sources from any reader
//! ([`SourceFile::new`]); a host that polls instead of taking callbacks starts a
//! [`TerrainJob`]. The command line is `wayfarer terrain compile` and `wayfarer terrain
//! inspect` (crate `wayfarer-cli`).

pub mod compile;
pub mod geo;
mod job;
pub mod osm;
pub mod source;
pub mod tiff;

pub use job::TerrainJob;
pub use source::{ReadSeek, SourceFile};
pub use wayfarer_package::CancelToken;

use rayon::prelude::*;
use std::io::{Seek, Write};
use std::path::{Path, PathBuf};
use std::time::Instant;
use wayfarer_terrain::lattice::{BLOCK, BLOCK_POSTS, BlockId, Spacing, TICKS_PER_DEG};
use wayfarer_terrain::terrain::{DEM_NONE, MAX_SOURCES};
use wayfarer_terrain::terrain_writer::TerrainWriter;

/// Why a compile failed.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// A source file could not be read or is of a kind this reader refuses.
    #[error("source: {0}")]
    Source(String),
    /// Options or a region the compile cannot use.
    #[error("options: {0}")]
    Options(String),
    /// The terrain file writer refused the output (or could not write it).
    #[error("terrain file: {0}")]
    Terrain(#[from] wayfarer_terrain::Error),
    /// The output file could not be created, moved into place or removed.
    #[error("output: {0}")]
    Output(String),
    /// The compile's [`CancelToken`] was cancelled.
    #[error("cancelled")]
    Cancelled,
}

pub type Result<T, E = Error> = std::result::Result<T, E>;

/// Attribution lines for the sources' licences, for [`Sources::attribution`].
pub mod attribution {
    pub const GEM15: &str = "Heights: GEM-15 DTM, CC BY 4.0. This dataset is produced by Google.";
    pub const GLO30: &str = "Heights where GEM-15 has none: Copernicus DEM GLO-30, produced using Copernicus WorldDEM-30 (c) DLR e.V. 2010-2014 and (c) Airbus Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA; all rights reserved.";
    pub const WORLDCOVER: &str = "Land cover: ESA WorldCover 10 m 2021 v200, CC BY 4.0. (c) ESA WorldCover project 2021 / Contains modified Copernicus Sentinel data (2021) processed by ESA WorldCover consortium.";
    /// Added by [`Sources::with_osm`](crate::Sources::with_osm).
    pub const OSM: &str = "Land-cover override and linear features: (c) OpenStreetMap contributors, ODbL 1.0, openstreetmap.org/copyright.";
    pub const ALL: [&str; 3] = [GEM15, GLO30, WORLDCOVER];
}

/// The inputs of a compile.
pub struct Sources {
    /// Height sources in tiers, best first: a post takes the first tier that covers it.
    /// Files may come in any order and from any coordinate system the reader knows
    /// (EPSG:4326 and UTM).
    pub heights: Vec<Vec<SourceFile>>,
    /// ESA WorldCover tiles (geographic, class codes 10-100); empty for no land cover.
    pub land_cover: Vec<SourceFile>,
    /// An OSM extract (`.osm.pbf`) for the land-cover override and the linear features; `None`
    /// for WorldCover only and an empty lines plane.
    pub osm: Option<PathBuf>,
    /// Licences and attribution, written into the file's sources text with the file names
    /// (sorted, so the order files come in changes no byte).
    pub attribution: String,
}

impl Sources {
    /// Opens the tiles at these paths, each a `.tif`/`.tiff` file or a directory of them (not
    /// searched recursively): `height_tiers` in tiers, best first (say GEM-15, then GLO-30),
    /// and WorldCover in `land_cover`. The attribution is [`attribution::ALL`].
    pub fn from_paths<P: AsRef<Path>>(
        height_tiers: &[Vec<P>],
        land_cover: &[P],
    ) -> Result<Sources> {
        let open = |paths: &[P]| -> Result<Vec<SourceFile>> {
            let mut files = Vec::new();
            for p in paths {
                for f in tiles(p.as_ref())? {
                    files.push(SourceFile::open(f)?);
                }
            }
            Ok(files)
        };
        Ok(Sources {
            heights: height_tiers
                .iter()
                .map(|t| open(t))
                .collect::<Result<_>>()?,
            land_cover: open(land_cover)?,
            osm: None,
            attribution: attribution::ALL.join("\n"),
        })
    }

    /// Adds an OSM extract and its attribution ([`attribution::OSM`]).
    pub fn with_osm(mut self, pbf: impl Into<PathBuf>) -> Sources {
        self.osm = Some(pbf.into());
        self.attribution.push('\n');
        self.attribution.push_str(attribution::OSM);
        self
    }
}

/// The path itself if it is a file, or the `.tif`/`.tiff` files in a directory, sorted.
fn tiles(path: &Path) -> Result<Vec<PathBuf>> {
    let err = |e: std::io::Error| Error::Source(format!("{}: {e}", path.display()));
    if !path.metadata().map_err(err)?.is_dir() {
        return Ok(vec![path.to_path_buf()]);
    }
    let mut v = Vec::new();
    for e in std::fs::read_dir(path).map_err(err)? {
        let p = e.map_err(err)?.path();
        let ext = p.extension().map(|x| x.to_ascii_lowercase());
        if p.is_file() && ext.is_some_and(|x| x == "tif" || x == "tiff") {
            v.push(p);
        }
    }
    if v.is_empty() {
        return Err(Error::Source(format!("{}: no .tif tiles", path.display())));
    }
    v.sort();
    Ok(v)
}

/// A directory of the compile's own, removed when dropped.
struct Scratch(PathBuf);

impl Scratch {
    fn new(base: Option<&Path>) -> Result<Scratch> {
        let base = base
            .map(Path::to_path_buf)
            .unwrap_or_else(std::env::temp_dir);
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos())
            .unwrap_or(0);
        let dir = base.join(format!("wayfarer-terrain-{}-{nanos}", std::process::id()));
        std::fs::create_dir_all(&dir)
            .map_err(|e| Error::Output(format!("scratch {}: {e}", dir.display())))?;
        Ok(Scratch(dir))
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// Where to compile.
#[derive(Clone, Debug, PartialEq)]
pub enum Region {
    /// Every block with a post whose cell meets this box (degrees).
    Bbox {
        south: f64,
        west: f64,
        north: f64,
        east: f64,
    },
    /// These blocks (sorted and deduplicated by the compile).
    Blocks(Vec<BlockId>),
}

impl Region {
    /// The region's blocks at a spacing, sorted.
    pub fn blocks(&self, s: Spacing) -> Result<Vec<BlockId>> {
        match self {
            Region::Blocks(b) => {
                let mut b = b.clone();
                b.sort();
                b.dedup();
                Ok(b)
            }
            &Region::Bbox {
                south,
                west,
                north,
                east,
            } => {
                if !(south < north && west < east && south >= -90.0 && north <= 90.0)
                    || !(west >= -180.0 && east <= 180.0)
                {
                    return Err(Error::Options(format!(
                        "bad box {south},{west},{north},{east}"
                    )));
                }
                let t = |d: f64| libm::round(d * TICKS_PER_DEG as f64) as i64;
                let b = BLOCK as i64;
                let (i0, i1) = (s.post_of(t(south)), s.post_of(t(north) - 1));
                let (j0, j1) = (s.post_of(t(west)), s.post_of(t(east) - 1));
                let n = (i1.div_euclid(b) - i0.div_euclid(b) + 1)
                    * (j1.div_euclid(b) - j0.div_euclid(b) + 1);
                if n > 50_000_000 {
                    return Err(Error::Options(format!("{n} blocks is too large a region")));
                }
                let mut out = Vec::with_capacity(n as usize);
                for bi in i0.div_euclid(b)..=i1.div_euclid(b) {
                    for bj in j0.div_euclid(b)..=j1.div_euclid(b) {
                        out.push(BlockId {
                            bi: bi as i32,
                            bj: bj as i32,
                        });
                    }
                }
                Ok(out)
            }
        }
    }
}

/// How to compile.
#[derive(Clone, Debug)]
pub struct Options {
    /// Post spacing; 1″ (120 ticks) by default. Must be a multiple of 15 ticks (1/8″).
    pub spacing: Spacing,
    /// Most bytes of decoded source tiles held at once (64 MiB by default). Smaller saves
    /// memory and decodes tiles more often; it never changes the output.
    pub cache_bytes: usize,
    /// Worker threads; `None` uses the current rayon pool. Never changes the output.
    pub threads: Option<usize>,
    /// Where the OSM stage keeps its node store while it reads an extract (a directory of
    /// its own is made in it and removed after); the system's temporary directory by
    /// default. A small extract's store stays in memory.
    pub scratch_dir: Option<PathBuf>,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            spacing: Spacing::new(120).unwrap(),
            cache_bytes: 64 << 20,
            threads: None,
            scratch_dir: None,
        }
    }
}

/// How far a compile has come, reported after each block row.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Progress {
    pub stage: Stage,
    pub blocks_done: u64,
    pub blocks_total: u64,
}

/// What a compile is doing.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Stage {
    /// Reading the OSM extract (two passes), before any block.
    ReadingOsm,
    /// Compiling and writing block rows.
    #[default]
    Blocks,
}

impl Progress {
    /// The share of blocks done, 0 to 1.
    pub fn fraction(&self) -> f64 {
        if self.blocks_total == 0 {
            1.0
        } else {
            self.blocks_done as f64 / self.blocks_total as f64
        }
    }
}

/// What a compile did.
#[derive(Clone, Debug, Default)]
pub struct Report {
    pub blocks: u64,
    pub posts: u64,
    /// Posts that took their height from each tier.
    pub heights_by_tier: Vec<u64>,
    pub posts_without_height: u64,
    /// Posts whose steepness class came from a later tier than their height.
    pub steep_from_later_tier: u64,
    pub posts_with_cover: u64,
    pub posts_with_confident_cover: u64,
    /// Posts whose land cover OSM set, and posts with a linear feature.
    pub posts_with_osm_cover: u64,
    pub posts_with_lines: u64,
    /// What the OSM read found, with an extract.
    pub osm: Option<osm::OsmStats>,
    /// Source tiles decoded, counting repeats after eviction from the cache.
    pub tiles_decoded: u64,
    pub cache_peak_bytes: u64,
    pub file_bytes: u64,
    pub terrain_hash: [u8; 32],
    pub seconds: f64,
}

/// Compiles a terrain file for a region into a file at `path`. It writes `<path>.partial` and
/// moves it into place once complete, so `path` never holds part of a file; on an error or a
/// cancel the partial file is removed. See [`compile()`] for the rest.
pub fn compile_to_file(
    sources: Sources,
    region: &Region,
    options: &Options,
    path: impl AsRef<Path>,
    cancel: &CancelToken,
    on_progress: impl FnMut(Progress) + Send,
) -> Result<Report> {
    let path = path.as_ref();
    let mut partial = path.as_os_str().to_owned();
    partial.push(".partial");
    let partial = PathBuf::from(partial);
    let out = |e: std::io::Error| Error::Output(format!("{}: {e}", path.display()));
    let file = std::fs::File::create(&partial).map_err(out)?;
    let done = compile(
        sources,
        region,
        options,
        std::io::BufWriter::new(file),
        cancel,
        on_progress,
    )
    .and_then(|(report, w)| {
        let file = w.into_inner().map_err(|e| out(e.into_error()))?;
        file.sync_all().map_err(out)?;
        drop(file);
        std::fs::rename(&partial, path).map_err(out)?;
        Ok(report)
    });
    if done.is_err() {
        let _ = std::fs::remove_file(&partial);
    }
    done
}

/// Compiles a terrain file for a region into `out` (a file, or any seekable sink), one block
/// row at a time, and returns the report and the sink.
///
/// `on_progress` is called after each block row, on the thread that called `compile`.
/// Cancelling `cancel` stops the compile within a block and returns [`Error::Cancelled`],
/// leaving part of a file in `out`.
pub fn compile<W: Write + Seek + Send>(
    sources: Sources,
    region: &Region,
    options: &Options,
    out: W,
    cancel: &CancelToken,
    mut on_progress: impl FnMut(Progress) + Send,
) -> Result<(Report, W)> {
    let started = Instant::now();
    let s = options.spacing;
    let k = compile::samples_per_side(s)?;
    let blocks = region.blocks(s)?;
    let mut text = sources.attribution.clone();
    for (t, tier) in sources.heights.iter().enumerate() {
        let mut names: Vec<&str> = tier.iter().map(|f| f.name.as_str()).collect();
        names.sort_unstable();
        text.push_str(&format!("\nheights tier {}: {}", t + 1, names.join(", ")));
    }
    if !sources.land_cover.is_empty() {
        let mut names: Vec<&str> = sources.land_cover.iter().map(|f| f.name.as_str()).collect();
        names.sort_unstable();
        text.push_str(&format!("\nland cover: {}", names.join(", ")));
    }
    if let Some(pbf) = &sources.osm {
        let name = pbf
            .file_name()
            .map(|n| n.to_string_lossy())
            .unwrap_or_default();
        text.push_str(&format!("\nosm: {name}"));
    }
    if text.len() > MAX_SOURCES {
        // Long file lists are cut at a character boundary; the attribution comes first.
        let mut cut = MAX_SOURCES;
        while !text.is_char_boundary(cut) {
            cut -= 1;
        }
        text.truncate(cut);
    }
    let mut id = 0u32;
    let mut tiers = Vec::new();
    for tier in sources.heights {
        let m = source::mosaics(tier, source::Kind::Heights, id)?;
        id += m.len() as u32;
        tiers.push(m);
    }
    let cover = source::mosaics(sources.land_cover, source::Kind::Classes, id)?;
    let cache = source::ChunkCache::new(options.cache_bytes);
    let ctx = compile::Ctx {
        tiers: &tiers,
        cover: &cover,
        cache: &cache,
        spacing: s,
        k,
        t2: compile::tan2_bounds(),
    };
    let mut report = Report {
        blocks: blocks.len() as u64,
        posts: (blocks.len() * BLOCK_POSTS) as u64,
        heights_by_tier: vec![0; tiers.len()],
        ..Report::default()
    };
    let mut writer = TerrainWriter::new(out, s, blocks.clone(), &text)?;
    let pbf = sources.osm;
    let mut run = |writer: &mut TerrainWriter<W>, report: &mut Report| -> Result<()> {
        let osm = match &pbf {
            Some(pbf) if !blocks.is_empty() => {
                on_progress(Progress {
                    stage: Stage::ReadingOsm,
                    blocks_done: 0,
                    blocks_total: blocks.len() as u64,
                });
                let scratch = Scratch::new(options.scratch_dir.as_deref())?;
                let o = osm::Osm::read(
                    pbf,
                    osm::Rect::of_blocks(&blocks, s),
                    options.threads,
                    &scratch.0,
                    cancel,
                );
                drop(scratch);
                let o = o?;
                report.osm = Some(o.stats.clone());
                Some(o)
            }
            _ => None,
        };
        let mut at = 0;
        while at < blocks.len() {
            let bi = blocks[at].bi;
            let end = at + blocks[at..].iter().take_while(|b| b.bi == bi).count();
            if cancel.is_cancelled() {
                return Err(Error::Cancelled);
            }
            let row = &blocks[at..end];
            let bands = osm.as_ref().map(|o| {
                let bjs: Vec<i32> = row.iter().map(|b| b.bj).collect();
                o.band(s, bi, &bjs)
            });
            let planes: Vec<compile::Planes> = row
                .par_iter()
                .enumerate()
                .map(|(n, &b)| {
                    if cancel.is_cancelled() {
                        return Err(Error::Cancelled);
                    }
                    compile::block(&ctx, b, bands.as_ref().map(|o| &o[n]))
                })
                .collect::<Result<_>>()?;
            drop(bands);
            for p in planes {
                for (t, n) in p.by_tier.iter().enumerate() {
                    report.heights_by_tier[t] += n;
                }
                report.posts_without_height +=
                    p.dm.iter().filter(|&&v| v == DEM_NONE).count() as u64;
                report.steep_from_later_tier += p.steep_fallback;
                report.posts_with_cover += p.cover.iter().filter(|&&c| c != 0).count() as u64;
                report.posts_with_confident_cover +=
                    p.conf.iter().filter(|&&c| c != 0).count() as u64;
                report.posts_with_osm_cover += p.osm_cover;
                report.posts_with_lines += p.lines.iter().filter(|&&l| l != 0).count() as u64;
                writer.push(&p.dm, &p.steep, &p.cover, &p.conf, &p.lines)?;
            }
            at = end;
            on_progress(Progress {
                stage: Stage::Blocks,
                blocks_done: at as u64,
                blocks_total: blocks.len() as u64,
            });
        }
        Ok(())
    };
    match options.threads {
        Some(n) => {
            let pool = rayon::ThreadPoolBuilder::new()
                .num_threads(n.max(1))
                .build()
                .map_err(|e| Error::Options(format!("thread pool: {e}")))?;
            pool.install(|| run(&mut writer, &mut report))?;
        }
        None => run(&mut writer, &mut report)?,
    }
    let (hash, bytes, out) = writer.finish()?;
    report.terrain_hash = hash;
    report.file_bytes = bytes;
    report.tiles_decoded = cache.decoded.load(std::sync::atomic::Ordering::Relaxed);
    report.cache_peak_bytes = cache.peak.load(std::sync::atomic::Ordering::Relaxed);
    report.seconds = started.elapsed().as_secs_f64();
    Ok((report, out))
}

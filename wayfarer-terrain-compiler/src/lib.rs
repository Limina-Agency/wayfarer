//! Stage 1 of Wayfarer's off-road terrain: compiles DEM and land-cover COG tiles into a
//! terrain file (`wayfarer_terrain::terrain`) on the global lattice.
//!
//! The heights come from tiers of DEM sources, best first: GEM-15 (Google's 15 m DTM, the
//! squares an app may already hold and draw), then Copernicus GLO-30 for its gaps. The
//! steepness-max class comes from the same full-resolution pixels, and the land cover from
//! ESA WorldCover. [`compile()`] reads only the tiles it needs, holds decoded tiles in a cache
//! of bounded size, works one block row (64 posts, about 2 km at 1″) at a time and streams
//! the file out, so its memory does not grow with the region. The rules of every step are in
//! [`compile`](mod@compile)'s module docs; they are integer arithmetic but for the UTM
//! projection, which uses `libm`, so the same inputs give the same file on every target and
//! thread count.
//!
//! ```no_run
//! use wayfarer_terrain_compiler::{Options, Region, SourceFile, Sources, compile};
//!
//! let sources = Sources {
//!     heights: vec![
//!         vec![SourceFile::open("gem15/dtm_w19s49.tif")?],
//!         vec![SourceFile::open("glo30/Copernicus_DSM_COG_10_N49_00_E019_00_DEM.tif")?],
//!     ],
//!     land_cover: vec![SourceFile::open("worldcover/ESA_WorldCover_10m_2021_v200_N48E018_Map.tif")?],
//!     attribution: wayfarer_terrain_compiler::attribution::ALL.join("\n"),
//! };
//! let region = Region::Bbox { south: 49.2, west: 19.84, north: 49.38, east: 20.12 };
//! let out = std::fs::File::create("tatra.wtr")?;
//! let (report, _) = compile(sources, &region, &Options::default(), out)?;
//! println!("{} blocks, {} bytes", report.blocks, report.file_bytes);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! Not yet here: the OSM stage (land-cover override and its confidence, linear features),
//! which fills the `cover`, `conf` and `lines` planes from an OSM extract.

pub mod compile;
pub mod geo;
pub mod source;
pub mod tiff;

pub use source::{ReadSeek, SourceFile};

use rayon::prelude::*;
use std::io::{Seek, Write};
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
}

pub type Result<T, E = Error> = std::result::Result<T, E>;

/// Attribution lines for the sources' licences, for [`Sources::attribution`].
pub mod attribution {
    pub const GEM15: &str = "Heights: GEM-15 DTM, CC BY 4.0. This dataset is produced by Google.";
    pub const GLO30: &str = "Heights where GEM-15 has none: Copernicus DEM GLO-30, produced using Copernicus WorldDEM-30 (c) DLR e.V. 2010-2014 and (c) Airbus Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA; all rights reserved.";
    pub const WORLDCOVER: &str = "Land cover: ESA WorldCover 10 m 2021 v200, CC BY 4.0. (c) ESA WorldCover project 2021 / Contains modified Copernicus Sentinel data (2021) processed by ESA WorldCover consortium.";
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
    /// Licences and attribution, written into the file's sources text with the file names
    /// (sorted, so the order files come in changes no byte).
    pub attribution: String,
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
}

impl Default for Options {
    fn default() -> Self {
        Options {
            spacing: Spacing::new(120).unwrap(),
            cache_bytes: 64 << 20,
            threads: None,
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
    /// Source tiles decoded, counting repeats after eviction from the cache.
    pub tiles_decoded: u64,
    pub cache_peak_bytes: u64,
    pub file_bytes: u64,
    pub terrain_hash: [u8; 32],
    pub seconds: f64,
}

/// Compiles a terrain file for a region into `out` (a file, or any seekable sink), one block
/// row at a time. Returns the report and the sink.
pub fn compile<W: Write + Seek + Send>(
    sources: Sources,
    region: &Region,
    options: &Options,
    out: W,
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
    let run = |writer: &mut TerrainWriter<W>, report: &mut Report| -> Result<()> {
        let mut at = 0;
        while at < blocks.len() {
            let bi = blocks[at].bi;
            let end = at + blocks[at..].iter().take_while(|b| b.bi == bi).count();
            let planes: Vec<compile::Planes> = blocks[at..end]
                .par_iter()
                .map(|&b| compile::block(&ctx, b))
                .collect::<Result<_>>()?;
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
                writer.push(&p.dm, &p.steep, &p.cover, &p.conf, &p.lines)?;
            }
            at = end;
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

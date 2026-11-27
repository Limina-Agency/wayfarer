//! The phone memory budget: a compile's peak heap stays within the tile cache plus a fixed
//! allowance per worker, whatever the region's size.
//!
//! Peak heap is measured with a counting allocator. The sources are virtual files whose tiles
//! are computed as they are read, so the test holds no source bytes and the measurement is the
//! compile's own: the tile cache, each worker's windows and decode buffers, one block row of
//! planes and the file's index.

mod common;

use common::{Place, Sample, Spec, Virtual, hills};
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};
use wayfarer_terrain_compiler::geo::Utm;
use wayfarer_terrain_compiler::{CancelToken, Options, Region, SourceFile, Sources, compile};

struct Counting;
static NOW: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        let p = unsafe { System.alloc(l) };
        if !p.is_null() {
            let n = NOW.fetch_add(l.size(), Ordering::Relaxed) + l.size();
            PEAK.fetch_max(n, Ordering::Relaxed);
        }
        p
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        unsafe { System.dealloc(p, l) };
        NOW.fetch_sub(l.size(), Ordering::Relaxed);
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, new: usize) -> *mut u8 {
        let q = unsafe { System.realloc(p, l, new) };
        if !q.is_null() {
            if new > l.size() {
                let n = NOW.fetch_add(new - l.size(), Ordering::Relaxed) + new - l.size();
                PEAK.fetch_max(n, Ordering::Relaxed);
            } else {
                NOW.fetch_sub(l.size() - new, Ordering::Relaxed);
            }
        }
        q
    }
}

#[global_allocator]
static A: Counting = Counting;

/// A sink that keeps only the length written (the output file is on disk, not in memory).
#[derive(Default)]
struct Sink {
    pos: u64,
    len: u64,
}

impl std::io::Write for Sink {
    fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
        self.pos += b.len() as u64;
        self.len = self.len.max(self.pos);
        Ok(b.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

impl std::io::Seek for Sink {
    fn seek(&mut self, p: std::io::SeekFrom) -> std::io::Result<u64> {
        self.pos = match p {
            std::io::SeekFrom::Start(x) => x,
            std::io::SeekFrom::End(x) => (self.len as i64 + x) as u64,
            std::io::SeekFrom::Current(x) => (self.pos as i64 + x) as u64,
        };
        Ok(self.pos)
    }
}

/// GEM-15-like UTM squares (512-pixel tiles, 1 MiB decoded) with a gap GLO-30-like tiles
/// (1,024-pixel tiles, 4 MiB decoded) fill, and WorldCover-like land cover over it all.
fn sources() -> Sources {
    let gem = |x: f64, gap: bool| {
        let spec = Spec {
            width: 4096,
            height: 2048,
            tile: 512,
            sample: Sample::F32,
            compression: 1,
            predictor: 1,
            big_endian: false,
            bigtiff: false,
            place: Place::Utm {
                zone: 34,
                x,
                y: 5_500_000.0,
                px: 15.0,
            },
            nodata: Some("nan"),
        };
        Virtual::new(spec, move |r, c| {
            let (e, n) = (
                x + 15.0 * (c as f64 + 0.5),
                5_500_000.0 - 15.0 * (r as f64 + 0.5),
            );
            if gap && (e - 400_000.0).abs() < 3000.0 {
                f64::NAN
            } else {
                hills(e, n)
            }
        })
    };
    let utm = Utm::new(34, true);
    let glo = Spec {
        width: 3600,
        height: 1800,
        tile: 1024,
        sample: Sample::F32,
        compression: 1,
        predictor: 1,
        big_endian: false,
        bigtiff: false,
        place: Place::Geo {
            lon: 19.0,
            lat: 49.75,
            dx: 1.0 / 3600.0,
            dy: 1.0 / 3600.0,
            point: true,
        },
        nodata: None,
    };
    let glo = Virtual::new(glo, move |r, c| {
        let (e, n) = utm.forward(49.75 - r as f64 / 3600.0, 19.0 + c as f64 / 3600.0);
        hills(e, n)
    });
    let wc = Spec {
        width: 12000,
        height: 6000,
        tile: 1024,
        sample: Sample::U8,
        compression: 1,
        predictor: 1,
        big_endian: false,
        bigtiff: false,
        place: Place::Geo {
            lon: 19.0,
            lat: 49.75,
            dx: 1.0 / 12000.0,
            dy: 1.0 / 12000.0,
            point: false,
        },
        nodata: Some("0"),
    };
    let wc = Virtual::new(wc, |r, c| {
        [10.0, 20.0, 30.0, 40.0, 50.0, 80.0][((r / 37 + c / 53) % 6) as usize]
    });
    Sources {
        heights: vec![
            vec![
                SourceFile::new("gem_w.tif", gem(360_000.0, true)),
                SourceFile::new("gem_e.tif", gem(410_000.0, false)),
            ],
            vec![SourceFile::new("glo.tif", glo)],
        ],
        land_cover: vec![SourceFile::new("wc.tif", wc)],
        osm: None,
        attribution: String::new(),
    }
}

fn peak(region: &Region, opts: &Options) -> (usize, wayfarer_terrain_compiler::Report) {
    let s = sources();
    let base = NOW.load(Ordering::Relaxed);
    PEAK.store(base, Ordering::Relaxed);
    let (r, _) = compile(
        s,
        region,
        opts,
        Sink::default(),
        &CancelToken::new(),
        |_| {},
    )
    .unwrap();
    (PEAK.load(Ordering::Relaxed) - base, r)
}

/// The budget the compile keeps to: the tile cache, 12 MiB per worker (a decoded 4 MiB tile
/// with its stored and raw bytes, the block's windows) and 8 MiB for the rest (a block row's
/// planes, the index, the thread pool).
fn budget(opts: &Options) -> usize {
    opts.cache_bytes + opts.threads.unwrap_or(1) * (12 << 20) + (8 << 20)
}

#[test]
fn peak_heap_stays_within_budget_and_flat_in_region_size() {
    let opts = Options {
        cache_bytes: 16 << 20,
        threads: Some(2),
        ..Options::default()
    };
    // 49 blocks a row (about 60 km at 49.5 N), 3 rows and then 9.
    let small = Region::Bbox {
        south: 49.40,
        west: 19.30,
        north: 49.43,
        east: 20.15,
    };
    let large = Region::Bbox {
        south: 49.40,
        west: 19.30,
        north: 49.54,
        east: 20.15,
    };
    let (ps, rs) = peak(&small, &opts);
    let (pl, rl) = peak(&large, &opts);
    eprintln!(
        "peak heap: {:.1} MiB for {} blocks, {:.1} MiB for {} blocks; budget {:.1} MiB",
        ps as f64 / 1048576.0,
        rs.blocks,
        pl as f64 / 1048576.0,
        rl.blocks,
        budget(&opts) as f64 / 1048576.0
    );
    assert!(rl.blocks >= 3 * rs.blocks, "{} {}", rs.blocks, rl.blocks);
    assert!(
        rl.heights_by_tier.iter().all(|&n| n > 0),
        "both tiers are read: {:?}",
        rl.heights_by_tier
    );
    assert!(rl.posts_with_cover > 0);
    assert!(
        pl <= budget(&opts),
        "peak heap {pl} over the budget {}",
        budget(&opts)
    );
    // Three times the rows, about the same peak: memory follows a block row, not the region.
    assert!(
        pl <= ps + ps / 5 + (2 << 20),
        "peak grew with the region: {ps} -> {pl}"
    );
}

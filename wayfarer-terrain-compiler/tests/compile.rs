//! End-to-end compiles of synthetic GEM-15-, GLO-30- and WorldCover-like COGs.

mod common;

use common::{Place, Sample, Spec, hills, tiff};
use std::io::Cursor;
use std::sync::OnceLock;
use wayfarer_terrain::lattice::{BLOCK, Spacing, TICKS_PER_DEG};
use wayfarer_terrain::terrain::{DEM_NONE, STEEP_NONE, Terrain};
use wayfarer_terrain_compiler::geo::Utm;
use wayfarer_terrain_compiler::{Options, Region, SourceFile, Sources, compile};

/// The terrain hash of `sources()` over `REGION`. It must be the same on every target,
/// thread count, cache size and input order; CI runs this on x86-64, i686, aarch64 macOS and
/// Windows.
const GOLDEN: &str = "269a83a35a4394f5e088500f380f45d784585c34e8ef9d1c7f7caee1473b0b14";

const REGION: Region = Region::Bbox {
    south: 49.40,
    west: 19.92,
    north: 49.47,
    east: 20.05,
};

struct Files {
    gem_a: Vec<u8>,
    gem_b: Vec<u8>,
    glo: Vec<u8>,
    wc: Vec<u8>,
}

fn files() -> &'static Files {
    static F: OnceLock<Files> = OnceLock::new();
    F.get_or_init(|| {
        let gem = |x: f64, w: u32, off: f64| {
            let spec = Spec {
                width: w,
                height: 2000,
                tile: 256,
                sample: Sample::F32,
                compression: 8,
                predictor: 1,
                big_endian: false,
                bigtiff: false,
                place: Place::Utm {
                    zone: 34,
                    x,
                    y: 5_490_000.0,
                    px: 15.0,
                },
                nodata: Some("nan"),
            };
            tiff(&spec, &move |r, c| {
                let (e, n) = (
                    x + 15.0 * (c as f64 + 0.5),
                    5_490_000.0 - 15.0 * (r as f64 + 0.5),
                );
                // The first file has no data east of 429,500 m: GLO-30 fills it.
                if off == 0.0 && e > 429_500.0 {
                    f64::NAN
                } else {
                    hills(e, n) + off
                }
            })
        };
        let glo = Spec {
            width: 2880,
            height: 1440,
            tile: 512,
            sample: Sample::F32,
            compression: 8,
            predictor: 3,
            big_endian: false,
            bigtiff: false,
            place: Place::Geo {
                lon: 19.5,
                lat: 49.7,
                dx: 1.0 / 3600.0,
                dy: 1.0 / 3600.0,
                point: true,
            },
            nodata: None,
        };
        let utm = Utm::new(34, true);
        let glo = tiff(&glo, &move |r, c| {
            let (e, n) = utm.forward(49.7 - r as f64 / 3600.0, 19.5 + c as f64 / 3600.0);
            hills(e, n) + 2.0
        });
        let wc = Spec {
            width: 1200,
            height: 1200,
            tile: 256,
            sample: Sample::U8,
            compression: 8,
            predictor: 1,
            big_endian: false,
            bigtiff: false,
            place: Place::Geo {
                lon: 19.9,
                lat: 49.5,
                dx: 1.0 / 12000.0,
                dy: 1.0 / 12000.0,
                point: false,
            },
            nodata: Some("0"),
        };
        let wc = tiff(&wc, &|r, c| {
            [10.0, 20.0, 30.0, 50.0, 80.0][((r / 40 + c / 60) % 5) as usize]
        });
        Files {
            gem_a: gem(410_000.0, 2000, 0.0),
            gem_b: gem(425_000.0, 300, 0.5),
            glo,
            wc,
        }
    })
}

fn sources(reversed: bool) -> Sources {
    let f = files();
    let file = |n: &str, b: &Vec<u8>| SourceFile::new(n, Cursor::new(b.clone()));
    let mut gem = vec![file("gem_a.tif", &f.gem_a), file("gem_b.tif", &f.gem_b)];
    if reversed {
        gem.reverse();
    }
    Sources {
        heights: vec![gem, vec![file("glo.tif", &f.glo)]],
        land_cover: vec![file("wc.tif", &f.wc)],
        attribution: "synthetic".into(),
    }
}

fn run(
    reversed: bool,
    threads: usize,
    cache: usize,
) -> (wayfarer_terrain_compiler::Report, Vec<u8>) {
    let opts = Options {
        threads: Some(threads),
        cache_bytes: cache,
        ..Options::default()
    };
    let (r, out) = compile(sources(reversed), &REGION, &opts, Cursor::new(Vec::new())).unwrap();
    (r, out.into_inner())
}

#[test]
fn same_bytes_whatever_the_order_threads_and_cache() {
    let (ra, a) = run(false, 1, 1 << 20);
    let (rb, b) = run(true, 3, 64 << 20);
    assert_eq!(a, b);
    assert_eq!(ra.terrain_hash, rb.terrain_hash);
    assert!(
        ra.tiles_decoded > rb.tiles_decoded,
        "a small cache decodes tiles again"
    );
    let t = Terrain::read(&a).unwrap();
    assert_eq!(t.hash(), ra.terrain_hash);
    assert_eq!(
        t.sources(),
        "synthetic\nheights tier 1: gem_a.tif, gem_b.tif\nheights tier 2: glo.tif\nland cover: wc.tif"
    );
    assert_eq!(
        wayfarer_terrain::hex(&ra.terrain_hash),
        GOLDEN,
        "terrain hash changed"
    );
}

/// Latitude and longitude (degrees) of post `q` of block `k`.
fn post_deg(t: &Terrain, k: usize, q: usize) -> (f64, f64) {
    let b = t.blocks()[k];
    let s = t.spacing().ticks() as f64;
    let i = b.i0() + (q / BLOCK) as i64;
    let j = b.j0() + (q % BLOCK) as i64;
    (
        i as f64 * s / TICKS_PER_DEG as f64,
        j as f64 * s / TICKS_PER_DEG as f64,
    )
}

#[test]
fn heights_steepness_and_cover() {
    let (r, bytes) = run(false, 2, 8 << 20);
    let t = Terrain::read(&bytes).unwrap();
    assert_eq!(t.spacing(), Spacing::new(120).unwrap());
    assert_eq!(r.posts_without_height, 0);
    assert!(
        r.heights_by_tier[0] > 0 && r.heights_by_tier[1] > 0,
        "{:?}",
        r.heights_by_tier
    );
    let utm = Utm::new(34, true);
    let (mut gem, mut overlap, mut glo, mut cliff) = (0, 0, 0, 0);
    for k in 0..t.blocks().len() {
        for q in 0..BLOCK * BLOCK {
            let (lat, lon) = post_deg(&t, k, q);
            let (e, n) = utm.forward(lat, lon);
            let dm = t.dm(k)[q];
            assert_ne!(dm, DEM_NONE);
            let got = dm as f64 / 10.0;
            let want = hills(e, n);
            // Away from the cliff and the edges of the first file's data.
            if (e - 424_000.0).abs() > 100.0
                && (e - 429_500.0).abs() > 60.0
                && (e - 425_000.0).abs() > 60.0
            {
                let off = if e < 425_000.0 {
                    gem += 1;
                    0.0
                } else if e < 429_500.0 {
                    overlap += 1;
                    0.25
                } else {
                    glo += 1;
                    2.0
                };
                assert!(
                    (got - want - off).abs() < 0.6,
                    "{lat} {lon}: {got} against {}",
                    want + off
                );
            }
            let s = t.steep(k)[q];
            assert_ne!(s, STEEP_NONE);
            // The plateau's edges at 423,960 and 424,040 m: a 30 m step between 15 m pixels.
            if (e - 423_960.0).abs() < 6.0 || (e - 424_040.0).abs() < 6.0 {
                cliff += 1;
                assert!(s >= 9, "the 30 m cliff is steep: class {s}");
            } else if (e - 424_000.0).abs() > 120.0 && e < 429_400.0 {
                assert!(
                    s <= 6,
                    "smooth hills stay below 25 degrees: class {s} at {lat} {lon}"
                );
            }
        }
    }
    assert!(
        gem > 1000 && overlap > 1000 && glo > 1000 && cliff > 50,
        "{gem} {overlap} {glo} {cliff}"
    );
    // Land cover inside the WorldCover square (49.4-49.5 N, 19.9-20.0 E): the pattern's
    // majority, confident away from its stripe edges.
    let mut seen = 0;
    for k in 0..t.blocks().len() {
        for q in 0..BLOCK * BLOCK {
            let (lat, lon) = post_deg(&t, k, q);
            let (cv, cf) = (t.cover(k)[q], t.conf(k)[q]);
            if lat > 49.401 && lon < 19.999 {
                let (r, c) = ((49.5 - lat) * 12000.0, (lon - 19.9) * 12000.0);
                let near = |v: f64, p: f64| (v.rem_euclid(p)).min(p - v.rem_euclid(p)) < 3.0;
                if !near(r, 40.0) && !near(c, 60.0) {
                    let want = [1, 2, 3, 5, 8][(((r as u32) / 40 + (c as u32) / 60) % 5) as usize];
                    assert_eq!((cv, cf), (want, 1), "{lat} {lon}");
                    seen += 1;
                }
            } else if lon > 20.001 {
                assert_eq!((cv, cf), (0, 0));
            }
        }
    }
    assert!(seen > 1000);
}

#[test]
fn half_second_spacing_and_given_blocks() {
    let opts = Options {
        spacing: Spacing::new(60).unwrap(),
        threads: Some(2),
        ..Options::default()
    };
    let s = opts.spacing;
    let blocks = REGION.blocks(s).unwrap();
    let pick = vec![blocks[blocks.len() / 2], blocks[0], blocks[0]];
    let (r, out) = compile(
        sources(false),
        &Region::Blocks(pick),
        &opts,
        Cursor::new(Vec::new()),
    )
    .unwrap();
    assert_eq!(r.blocks, 2);
    let t = Terrain::read(&out.into_inner()).unwrap();
    assert_eq!(t.blocks(), &[blocks[0], blocks[blocks.len() / 2]]);
    assert!(t.dm(1).iter().all(|&v| v != DEM_NONE));
}

#[test]
fn refuses_what_it_cannot_read() {
    let mut s = sources(false);
    s.heights[0][0] = SourceFile::new("junk.tif", Cursor::new(b"II*\0junk".to_vec()));
    assert!(compile(s, &REGION, &Options::default(), Cursor::new(Vec::new())).is_err());
    let opts = Options {
        spacing: Spacing::new(36).unwrap(),
        ..Options::default()
    };
    assert!(compile(sources(false), &REGION, &opts, Cursor::new(Vec::new())).is_err());
    let bad = Region::Bbox {
        south: 50.0,
        west: 19.0,
        north: 49.0,
        east: 20.0,
    };
    assert!(
        compile(
            sources(false),
            &bad,
            &Options::default(),
            Cursor::new(Vec::new())
        )
        .is_err()
    );
}

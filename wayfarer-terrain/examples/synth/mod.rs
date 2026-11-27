//! A synthetic incident area of a given size, for the bench and the memory test: a terrain
//! with relief, steep bands, land cover, roads, paths and rivers, six layers of the kinds an
//! incident collects, and a team overlay. Deterministic, so every machine builds the same one.

use wayfarer_terrain::fuse::{Effect, HashedLayer, OverlayItem};
use wayfarer_terrain::lattice::{BLOCK, BLOCK_POSTS, BlockId, Spacing};
use wayfarer_terrain::layer::{
    FEAT_FALLEN, FEAT_WASHOUT, Feature, GridBlock, Kind, Layer, Manifest, NCLASS,
};
use wayfarer_terrain::lines::*;
use wayfarer_terrain::terrain::Terrain;

pub const SP: i64 = 120;
pub const T0: i64 = 1_795_510_800; // 2026-11-24 09:00 UTC
const H: i64 = 3600;
/// South-west block: about 49.24° N, 20.0° E.
const BI0: i32 = 2770;
const BJ0: i32 = 1125;
/// One 1″ block at 49° N: 64 × 30.87 m by 64 × 30.87 m × cos 49°.
pub const BLOCK_KM2: f64 = 1.9757 * 1.2962;

fn mix(x: u64) -> u64 {
    let mut z = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// Rows and columns of blocks for a square area of about `km2`.
pub fn shape(km2: f64) -> (usize, usize) {
    let side = km2.sqrt();
    let nbi = (side / 1.9757).round().max(1.0) as usize;
    let nbj = (side / 1.2962).round().max(1.0) as usize;
    (nbi, nbj)
}

pub struct Area {
    pub terrain: Terrain,
    pub layers: Vec<HashedLayer>,
    pub overlay: Vec<OverlayItem>,
    /// Dense rows and columns of posts.
    pub rows: i64,
    pub cols: i64,
}

fn rc(b: BlockId, q: usize) -> (i64, i64) {
    (
        (b.bi - BI0) as i64 * BLOCK as i64 + (q / BLOCK) as i64,
        (b.bj - BJ0) as i64 * BLOCK as i64 + (q % BLOCK) as i64,
    )
}

pub fn area(km2: f64) -> Area {
    let (nbi, nbj) = shape(km2);
    let (rows, cols) = ((nbi * BLOCK) as i64, (nbj * BLOCK) as i64);
    let mut blocks = Vec::new();
    for bi in 0..nbi as i32 {
        for bj in 0..nbj as i32 {
            blocks.push(BlockId {
                bi: BI0 + bi,
                bj: BJ0 + bj,
            });
        }
    }
    let n = blocks.len() * BLOCK_POSTS;
    let (mut dm, mut steep, mut cover, mut conf, mut lines) =
        (vec![0; n], vec![0; n], vec![0; n], vec![0; n], vec![0; n]);
    for (k, &b) in blocks.iter().enumerate() {
        for q in 0..BLOCK_POSTS {
            let (r, c) = rc(b, q);
            let i = k * BLOCK_POSTS + q;
            // Ridges every 200 rows and a valley every 300 columns, with noise.
            let ridge = 4_000 - 40 * ((r % 200) - 100).abs();
            let valley = 3 * ((c % 300) - 150).abs();
            let noise = (mix((r * 100_003 + c) as u64) % 40) as i64;
            dm[i] = (8_000 + ridge + valley + noise) as i32;
            steep[i] = if (r % 200 - 100).abs() < 4 {
                11
            } else {
                ((r + c) % 6) as u8
            };
            let z = mix((r / 16 * 7_919 + c / 16) as u64);
            cover[i] = [3, 1, 2, 6, 12, 1, 4, 3][(z % 8) as usize];
            conf[i] = (z >> 8 & 1) as u8;
            let mut l = 0u16;
            if r % 150 == 0 {
                l |= TRACK;
            }
            if c % 220 == 0 {
                l |= PATH;
            }
            if (c % 300 - 150).abs() == 0 {
                l |= RIVER;
            }
            if (r + c) % 400 == 0 {
                l |= STREAM;
            }
            if c % 900 == 450 {
                l |= MAJOR;
            }
            lines[i] = l;
        }
    }
    let terrain = Terrain::new(
        Spacing::new(SP).unwrap(),
        blocks,
        dm,
        steep,
        cover,
        conf,
        lines,
    )
    .unwrap()
    .with_sources("synthetic")
    .unwrap();

    let year = T0 + 365 * 24 * H;
    let man = |kind, channels, model: &str, observed: i64, until: i64, fade: i64| Manifest {
        kind,
        licence: 0,
        s_lat: SP as u32,
        s_lon: SP as u32,
        observed_at: observed,
        valid_from: observed,
        valid_until: until,
        fade_from: fade,
        channels,
        model_id: model.into(),
        model_version: "1".into(),
        weights: *blake3::hash(model.as_bytes()).as_bytes(),
        dataset_manifest: [0; 32],
        eval_set: [0; 32],
        ece_permille: 0xFFFF,
        backend: "synthetic".into(),
        attribution: String::new(),
        inputs: vec![],
        parents: vec![],
    };
    let grid = |m: Manifest, f: &dyn Fn(i64, i64) -> Option<(u8, Vec<u16>)>| {
        let (slots, _) = m.slots();
        let mut bs = Vec::new();
        for &b in terrain.blocks() {
            let mut g = GridBlock::new(b, slots);
            for q in 0..BLOCK_POSTS {
                let (r, c) = rc(b, q);
                if let Some((w, v)) = f(r, c) {
                    g.set_present(q);
                    g.conf[q] = w;
                    for (s, x) in v.into_iter().enumerate() {
                        g.vals[s][q] = x;
                    }
                }
            }
            bs.push(g);
        }
        HashedLayer::new(Layer {
            m,
            blocks: bs,
            features: vec![],
        })
        .unwrap()
    };
    let river = |c: i64| (c % 300 - 150).abs();
    let mut layers = vec![
        // A dense land-cover refresh (the satellite model), everywhere.
        grid(
            man(Kind::Class, vec![], "landcover", T0 - 48 * H, year, year),
            &|r, c| {
                let z = mix((r * 7_919 + c) as u64);
                let mut v = vec![0u16; NCLASS];
                let top = [3usize, 1, 2, 6, 9][(z % 5) as usize];
                v[top] = 150 + (z >> 8) as u16 % 60;
                v[0] = 30;
                v[1] += 255 - v[top] - 30;
                Some((200, v))
            },
        ),
        // Flood probability and depth along every river, fading after two days.
        grid(
            man(
                Kind::Hazard,
                vec![1],
                "flood",
                T0 - 4 * H,
                T0 + 120 * H,
                T0 + 48 * H,
            ),
            &|_, c| (river(c) <= 6).then(|| (230, vec![(250 - 30 * river(c)) as u16])),
        ),
        grid(
            man(
                Kind::Level,
                vec![1],
                "depth",
                T0 - 4 * H,
                T0 + 120 * H,
                T0 + 48 * H,
            ),
            &|_, c| (river(c) <= 3).then(|| (200, vec![(90 - 15 * river(c)) as u16])),
        ),
        // Rubble patches.
        grid(
            man(Kind::Hazard, vec![2], "rubble", T0 - 9 * 24 * H, year, year),
            &|r, c| {
                ((r % 256 - 128).pow(2) + (c % 256 - 128).pow(2) < 150).then(|| (200, vec![200]))
            },
        ),
        // A second land-cover refresh from a drone, over a tenth of the area.
        grid(
            man(Kind::Class, vec![], "drone-landcover", T0 - H, year, year),
            &|r, c| {
                (r < rows / 3 && c < cols / 3).then(|| {
                    let mut v = vec![0u16; NCLASS];
                    v[2] = 255;
                    (220, v)
                })
            },
        ),
    ];
    // Drone features: washouts across the tracks, fallen trees on the paths.
    let e7 = |r: i64, c: i64| {
        let (la, lo) = ((BI0 as i64 * 64 + r) * SP, (BJ0 as i64 * 64 + c) * SP);
        ((la * 625 / 27) as i32, (lo * 625 / 27) as i32)
    };
    let mut features = Vec::new();
    for r in (150..rows).step_by(150) {
        for c in (40..cols).step_by(500) {
            features.push(Feature {
                class: FEAT_WASHOUT,
                conf: 220,
                pts: vec![e7(r - 3, c), e7(r + 3, c)],
            });
        }
    }
    for c in (220..cols).step_by(220) {
        for r in (70..rows).step_by(400) {
            features.push(Feature {
                class: FEAT_FALLEN,
                conf: 200,
                pts: vec![e7(r, c - 2), e7(r + 4, c + 2)],
            });
        }
    }
    let mut f = Layer {
        m: man(Kind::Feature, vec![], "drone", T0 + 1_800, year, year),
        blocks: vec![],
        features,
    };
    f.canonicalise().unwrap();
    layers.push(HashedLayer::new(f).unwrap());

    let tick = |r: i64, c: i64| ((BI0 as i64 * 64 + r) * SP, (BJ0 as i64 * 64 + c) * SP);
    let mut overlay = Vec::new();
    for k in 0..8 {
        let (la, lo) = tick(rows * k / 8 + 10, cols * (7 - k) / 8 + 10);
        overlay.push(OverlayItem {
            lat0: la,
            lat1: la + 6 * SP,
            lon0: lo,
            lon1: lo + 6 * SP,
            observed_at: T0 + k,
            effect: if k % 2 == 0 {
                Effect::Block
            } else {
                Effect::Clear
            },
        });
    }
    Area {
        terrain,
        layers,
        overlay,
        rows,
        cols,
    }
}

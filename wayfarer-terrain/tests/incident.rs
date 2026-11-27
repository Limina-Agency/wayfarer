//! An incident timeline on a small synthetic base: every layer kind, a team overlay, fading
//! and expiry. Checks the contract's invariants and pins the hashes, so a change to any result
//! on any CI target (x86_64, i686, aarch64 macOS, Windows) fails here.
//!
//! After an intended change, run with `UPDATE_GOLDEN=1` to print the new table.

use wayfarer_terrain::fuse::{Effect, HashedLayer, OverlayItem, Prior};
use wayfarer_terrain::lattice::{BLOCK, BLOCK_POSTS, BlockId, Spacing};
use wayfarer_terrain::layer::{
    FEAT_DEBRIS, FEAT_FALLEN, FEAT_PASSAGE, FEAT_WASHOUT, Feature, GridBlock, Kind,
    LICENCE_RESEARCH, Layer, Manifest, NCLASS,
};
use wayfarer_terrain::lines::*;
use wayfarer_terrain::profile::Rules;
use wayfarer_terrain::search::{Graph, Grid, Search, path, route};
use wayfarer_terrain::surface::{Incident, Surface, apply_delta, delta, epoch_of};
use wayfarer_terrain::terrain::{STEEP_NONE, Terrain};
use wayfarer_terrain::{cost, hex};

const SP: i64 = 120;
const E: i64 = 900;
const T0: i64 = 1_795_510_800; // 2026-11-24 09:00 UTC
const H: i64 = 3600;
const BI0: i32 = 2770; // about 49.24 N
const BJ0: i32 = 1125; // 20.0 E
const NBI: usize = 3;
const NBJ: usize = 4;

/// A deterministic mixer for synthetic values.
fn mix(x: u64) -> u64 {
    let mut z = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// Rows and columns of the dense rectangle, from the south-west post.
const ROWS: i64 = (NBI * BLOCK) as i64;
const COLS: i64 = (NBJ * BLOCK) as i64;

fn base() -> Terrain {
    let mut blocks = Vec::new();
    for bi in 0..NBI as i32 {
        for bj in 0..NBJ as i32 {
            blocks.push(BlockId {
                bi: BI0 + bi,
                bj: BJ0 + bj,
            });
        }
    }
    let n = blocks.len() * BLOCK_POSTS;
    let (mut dm, mut steep, mut cover, mut conf, mut lines) =
        (vec![0; n], vec![0; n], vec![0; n], vec![0; n], vec![0; n]);
    for (k, b) in blocks.iter().enumerate() {
        for q in 0..BLOCK_POSTS {
            let r = (b.bi - BI0) as i64 * BLOCK as i64 + (q / BLOCK) as i64;
            let c = (b.bj - BJ0) as i64 * BLOCK as i64 + (q % BLOCK) as i64;
            let i = k * BLOCK_POSTS + q;
            // a slope up to the north-east, a ridge at row 120, small bumps
            let ridge = 600 - 12 * (r - 120).abs().min(50);
            dm[i] =
                (9_000 + 25 * r + 10 * c + ridge + (mix((r * 1000 + c) as u64) % 7) as i64) as i32;
            steep[i] = if (r - 120).abs() < 3 && c % 50 > 8 {
                12
            } else if (r * 7 + c) % 211 == 0 {
                STEEP_NONE
            } else {
                ((r + c) % 7) as u8
            };
            let lake = (r - 40).pow(2) + (c - 180).pow(2) < 144;
            cover[i] = if lake {
                8
            } else {
                [3, 1, 2, 6, 12, 1, 4][((r / 24 + c / 30) % 7) as usize]
            };
            conf[i] = ((r + c) % 3 == 0) as u8;
            let mut l = 0u16;
            if r == 60 {
                l |= PATH;
            }
            if c == 100 && r < 150 {
                l |= STREAM;
            }
            if c == 200 {
                l |= RIVER;
            }
            if r == 20 && c > 30 {
                l |= TRACK;
            }
            if c == 30 && r > 100 {
                l |= MOTORWAY;
            }
            if r == 150 && (40..90).contains(&c) {
                l |= FENCE;
            }
            if (r - 2 * c + 120).abs() < 1 && r > 150 {
                l |= PATH_MOUNTAIN;
            }
            lines[i] = l;
        }
    }
    Terrain::new(
        Spacing::new(SP).unwrap(),
        blocks,
        dm,
        steep,
        cover,
        conf,
        lines,
    )
    .unwrap()
}

fn manifest(
    kind: Kind,
    channels: Vec<u16>,
    model: &str,
    observed: i64,
    valid_until: i64,
    fade_from: i64,
) -> Manifest {
    Manifest {
        kind,
        licence: 0,
        s_lat: SP as u32,
        s_lon: SP as u32,
        observed_at: observed,
        valid_from: observed,
        valid_until,
        fade_from,
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
    }
}

/// A grid layer from a per-post function of (row, col) giving (confidence, values) or None.
fn grid(m: Manifest, ter: &Terrain, f: impl Fn(i64, i64) -> Option<(u8, Vec<u16>)>) -> HashedLayer {
    let (slots, _) = m.slots();
    let mut blocks = Vec::new();
    for &b in ter.blocks() {
        let mut g = GridBlock::new(b, slots);
        for q in 0..BLOCK_POSTS {
            let r = (b.bi - BI0) as i64 * BLOCK as i64 + (q / BLOCK) as i64;
            let c = (b.bj - BJ0) as i64 * BLOCK as i64 + (q % BLOCK) as i64;
            if let Some((w, v)) = f(r, c) {
                g.set_present(q);
                g.conf[q] = w;
                for (s, x) in v.into_iter().enumerate() {
                    g.vals[s][q] = x;
                }
            }
        }
        blocks.push(g);
    }
    HashedLayer::new(Layer {
        m,
        blocks,
        features: vec![],
    })
    .unwrap()
}

/// E7 degrees of a dense (row, col) post.
fn e7(r: i64, c: i64) -> (i32, i32) {
    let lat = (BI0 as i64 * BLOCK as i64 + r) * SP; // ticks
    let lon = (BJ0 as i64 * BLOCK as i64 + c) * SP;
    // ticks x 625 = E7 x 27
    ((lat * 625 / 27) as i32, (lon * 625 / 27) as i32)
}

fn tick_box(r0: i64, r1: i64, c0: i64, c1: i64, effect: Effect) -> OverlayItem {
    let (i0, j0) = (BI0 as i64 * BLOCK as i64, BJ0 as i64 * BLOCK as i64);
    OverlayItem {
        lat0: (i0 + r0) * SP,
        lat1: (i0 + r1) * SP,
        lon0: (j0 + c0) * SP,
        lon1: (j0 + c1) * SP,
        effect,
    }
}

struct Fixture {
    ter: Terrain,
    layers: Vec<HashedLayer>,
    overlay: Vec<OverlayItem>,
}

fn fixture() -> Fixture {
    let ter = base();
    let year = T0 + 365 * 24 * H;
    let mut layers = Vec::new();
    // 0: a soft land-cover refresh with a cloud hole (confidence 0) and KEEP mass.
    layers.push(grid(
        manifest(
            Kind::Class,
            vec![],
            "landcover",
            T0 - 4 * 24 * H,
            year,
            year,
        ),
        &ter,
        |r, c| {
            if (r - 100).pow(2) + (c - 60).pow(2) < 400 {
                return Some((0, {
                    let mut v = vec![0u16; NCLASS];
                    v[0] = 255;
                    v
                }));
            }
            let z = mix((r * 7919 + c) as u64);
            let mut v = vec![0u16; NCLASS];
            let top = [3usize, 1, 2, 6, 9][(z % 5) as usize];
            v[top] = 150 + (z >> 8) as u16 % 60;
            v[0] = 30;
            v[1] += 255 - v[top] - 30;
            Some((180 + (z >> 20) as u8 % 60, v))
        },
    ));
    // 1, 2: flood probability and water depth along the river, fading after two days.
    let fl = |r: i64, c: i64| (c - 200).abs() <= 6 && r < 140;
    layers.push(grid(
        manifest(
            Kind::Hazard,
            vec![1],
            "flood",
            T0 - 4 * H,
            T0 + 5 * 24 * H,
            T0 + 2 * 24 * H,
        ),
        &ter,
        |r, c| fl(r, c).then(|| (230, vec![(250 - 30 * (c - 200).abs()) as u16])),
    ));
    layers.push(grid(
        manifest(
            Kind::Level,
            vec![1],
            "depth",
            T0 - 4 * H,
            T0 + 5 * 24 * H,
            T0 + 2 * 24 * H,
        ),
        &ter,
        |r, c| (fl(r, c) && r < 70).then(|| (200, vec![(90 - 15 * (c - 200).abs()) as u16])),
    ));
    // 3: a drone survey: washout across the track, fallen trees on the path, a passage, and a
    // debris line below the confidence threshold.
    let mut f = Layer {
        m: manifest(Kind::Feature, vec![], "drone", T0 + 30 * 60, year, year),
        blocks: vec![],
        features: vec![
            Feature {
                class: FEAT_WASHOUT,
                conf: 220,
                pts: vec![e7(17, 80), e7(23, 80)],
            },
            Feature {
                class: FEAT_FALLEN,
                conf: 200,
                pts: vec![e7(57, 130), e7(63, 132)],
            },
            Feature {
                class: FEAT_PASSAGE,
                conf: 200,
                pts: vec![e7(100, 100), e7(100, 140), e7(130, 160)],
            },
            Feature {
                class: FEAT_DEBRIS,
                conf: 60,
                pts: vec![e7(80, 10), e7(80, 60)],
            },
        ],
    };
    f.canonicalise().unwrap();
    layers.push(HashedLayer::new(f).unwrap());
    // 4: an older rubble hazard delivered late.
    layers.push(grid(
        manifest(
            Kind::Hazard,
            vec![2, 7],
            "rubble",
            T0 - 9 * 24 * H,
            year,
            year,
        ),
        &ter,
        |r, c| ((r - 30).pow(2) + (c - 140).pow(2) < 100).then(|| (200, vec![200, 90])),
    ));
    // 5-7: left out: expired, research-only, a spacing mismatch.
    layers.push(grid(
        manifest(
            Kind::Hazard,
            vec![5],
            "snow",
            T0 - 30 * 24 * H,
            T0 - 20 * 24 * H,
            T0 - 20 * 24 * H,
        ),
        &ter,
        |_, _| Some((255, vec![255])),
    ));
    let mut research = manifest(Kind::Hazard, vec![4], "fire", T0 - H, year, year);
    research.licence = LICENCE_RESEARCH;
    layers.push(grid(research, &ter, |r, _| {
        (r < 10).then(|| (255, vec![255]))
    }));
    let mut coarse = manifest(Kind::Hazard, vec![8], "closure", T0 - H, year, year);
    (coarse.s_lat, coarse.s_lon) = (360, 360);
    layers.push(grid(coarse, &ter, |r, _| {
        (r < 10).then(|| (255, vec![255]))
    }));
    let overlay = vec![
        tick_box(58, 61, 160, 163, Effect::Block),
        tick_box(54, 66, 194, 206, Effect::Clear),
    ];
    Fixture {
        ter,
        layers,
        overlay,
    }
}

struct Step {
    name: &'static str,
    t: i64,
    layers: Vec<usize>,
    overlay: bool,
}

fn steps() -> Vec<Step> {
    let all = vec![0, 5, 6, 7, 1, 2, 3, 4];
    vec![
        Step {
            name: "base",
            t: T0,
            layers: vec![],
            overlay: false,
        },
        Step {
            name: "landcover",
            t: T0,
            layers: vec![0, 5, 6, 7],
            overlay: false,
        },
        Step {
            name: "flood",
            t: T0,
            layers: vec![0, 5, 6, 7, 1, 2],
            overlay: false,
        },
        Step {
            name: "drone",
            t: T0 + 3 * E,
            layers: vec![0, 5, 6, 7, 1, 2, 3],
            overlay: false,
        },
        Step {
            name: "late",
            t: T0 + 4 * E,
            layers: all.clone(),
            overlay: false,
        },
        Step {
            name: "team",
            t: T0 + 5 * E,
            layers: all.clone(),
            overlay: true,
        },
        Step {
            name: "fading",
            t: T0 + 3 * 24 * H,
            layers: all.clone(),
            overlay: true,
        },
        Step {
            name: "expired",
            t: T0 + 6 * 24 * H,
            layers: all,
            overlay: true,
        },
    ]
}

/// (step, content hash, recipe hash), and the walker route path hashes, pinned.
const GOLDEN: &[(&str, &str)] = &[
    (
        "content_base",
        "727c82bfec56382cc67550757f756a88a0ed6eb60cec3c8d15e9a00d0cf7f298",
    ),
    (
        "recipe_base",
        "5070f26ad01d674d1a536fa0426cea6a62122c1bc0851e08e18e9dab83e966e1",
    ),
    (
        "content_landcover",
        "f96281e307abd987e8697331166180ded003995dd2c911d87bc936b131aab336",
    ),
    (
        "recipe_landcover",
        "f9941ecd11473999a7dbf371e2e02a53d033b827eee17aa7725777269ac1238b",
    ),
    (
        "content_flood",
        "cab0e5d900757d30c3e93f66963b059ba6fc2e3631973188f4586e932d373a3e",
    ),
    (
        "recipe_flood",
        "969cdb550c0d2c91ff8e848df64c093277ff77b6e72622e5775e90982abc3917",
    ),
    (
        "content_drone",
        "b4db890abf535ca50445152fda9ae2a217154b8383f644aecbcdce3e7ac8df76",
    ),
    (
        "recipe_drone",
        "5b759f39e54a52f58a28c644f857f8ea16a88fca972e521fe661b4bcb0f1ea6d",
    ),
    (
        "content_late",
        "3e4d8145d72f1cb040d3b92bac4d7828239935e8dfa630d777a982945771c9c9",
    ),
    (
        "recipe_late",
        "cbec9fd76231dea3f2a8ad5da0a251db4a85d667383a5c344d0b2d20bf1d09ef",
    ),
    (
        "content_team",
        "962fffb468e68434ce8d06c901c00e1e7f62ba551e7d5f335ff45159cb533a59",
    ),
    (
        "recipe_team",
        "b5a1e2542605d094f81e27493b68fe9f5cc4116dba39f9af77edc8b6c3516a90",
    ),
    (
        "content_fading",
        "5f02f4ea0c2a6327e4c48ba42f83228ee7415b0a21e973e10f5154bfdb02eeab",
    ),
    (
        "recipe_fading",
        "40a1079cb18bca2dcc4c467a2eb75106dd5a00bbf37658a19972f864f197546d",
    ),
    (
        "content_expired",
        "30623ab0a2018aabe50d58e9bab01f4585624231fa82ce886164f8beb4edfb2e",
    ),
    (
        "recipe_expired",
        "08a1213ff1f31ae66b3477afb90f0b1750c3ab7dd832e72b4f507d21b26316b6",
    ),
    (
        "route_base_0",
        "2cd95cadb3ef4236 11856619656 7496630 255 255 0 0 0 [] false",
    ),
    (
        "route_base_1",
        "b61f0f1c811f6645 4895926248 4533833 255 255 0 0 0 [] false",
    ),
    (
        "route_base_2",
        "76904fbb6766eb20 13847367961 4964974 255 255 0 0 0 [] false",
    ),
    (
        "route_base_3",
        "859673f4acd41017 5189638581 4067989 255 255 0 0 0 [] false",
    ),
    ("route_drone_0", "unreachable"),
    ("route_drone_1", "unreachable"),
    (
        "route_drone_2",
        "38a5d048dd89705f 10381129646 5120519 168 109 1000 0 0 [1] true",
    ),
    (
        "route_drone_3",
        "c493ff5b4b25424e 5024965713 3913748 161 111 1000 0 0 [1] false",
    ),
    (
        "route_team_0",
        "2ec9810e3e2d919b 11389349366 7089905 167 110 953 47 0 [2, 4] false",
    ),
    (
        "route_team_1",
        "95917234d7de055b 8163853204 5935973 167 111 945 55 0 [2, 4] false",
    ),
    (
        "route_team_2",
        "38a5d048dd89705f 10381129646 5120519 168 109 1000 0 0 [2] true",
    ),
    (
        "route_team_3",
        "52471f6371fd37f9 5146847653 3940943 161 111 1000 0 0 [2] false",
    ),
];

#[test]
fn timeline() {
    let fx = fixture();
    let rules = [Rules::walker_default(), Rules::vehicle()];
    let prior = Prior::default();
    let inc = |st: &Step| Incident {
        id: *b"wayfarer-test\0\0\0",
        terrain: &fx.ter,
        prior: &prior,
        layers: st.layers.iter().map(|&k| &fx.layers[k]).collect(),
        overlay: if st.overlay {
            fx.overlay.clone()
        } else {
            vec![]
        },
        rules: &rules,
        epoch_s: E,
        allow_research: false,
        feature_min_conf: 128,
        level_min_w: 128,
    };
    let steps = steps();
    let mut got: Vec<(String, String)> = Vec::new();
    let mut surfaces: Vec<Surface> = Vec::new();
    for (sn, st) in steps.iter().enumerate() {
        let i = inc(st);
        let full = i.surface(st.t).unwrap();
        // one thread gives the same surface
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(1)
            .build()
            .unwrap();
        assert_eq!(
            pool.install(|| i.surface(st.t).unwrap()),
            full,
            "{}: thread count",
            st.name
        );
        // the file round trip is exact
        let file = full.write();
        let back = Surface::read(&file).unwrap();
        assert_eq!(back, full);
        assert_eq!(back.write(), file);
        // the surface is the same until same_until
        let su = full.h.same_until;
        if su != i64::MAX {
            assert_eq!(su, epoch_of(su, E));
            let later = i.surface(su - E).unwrap();
            assert_eq!(
                later.content_hash(),
                full.content_hash(),
                "{}: same content until same_until",
                st.name
            );
        }
        if let Some(prev) = surfaces.last() {
            let before = inc(&steps[sn - 1]);
            let up = i.update(st.t, prev, &before).unwrap();
            assert_eq!(up, full, "{}: incremental equals full", st.name);
            let d = delta(prev, &full);
            assert_eq!(apply_delta(prev, &d).unwrap(), full);
            assert!(apply_delta(&full, &d).is_err() || full.content_hash() == prev.content_hash());
        }
        got.push((format!("content_{}", st.name), hex(&full.content_hash())));
        got.push((format!("recipe_{}", st.name), hex(&full.recipe_hash())));
        surfaces.push(full);
    }

    // What each step did.
    let walker_closed = |s: &Surface, why: u8| {
        s.blocks
            .values()
            .map(|(_, b)| b.prof[0].why.iter().filter(|&&w| w >> 4 == why).count())
            .sum::<usize>()
    };
    assert_eq!(surfaces[0].h.applied.len(), 0);
    assert_eq!(surfaces[1].h.applied.len(), 1);
    assert_eq!(
        surfaces[1]
            .h
            .left_out
            .iter()
            .map(|l| l.1)
            .collect::<Vec<_>>()
            .len(),
        3
    );
    assert!(
        walker_closed(&surfaces[2], cost::WHY_HAZARD) > 0,
        "the flood closes cells"
    );
    assert!(
        walker_closed(&surfaces[2], cost::WHY_LEVEL) > 0,
        "deep water closes cells"
    );
    assert!(
        walker_closed(&surfaces[3], cost::WHY_WASHOUT) > 0,
        "the washout cuts the track"
    );
    assert!(
        walker_closed(&surfaces[5], cost::WHY_TEAM) == 9,
        "the team block covers 3 x 3 posts"
    );
    assert_eq!(
        walker_closed(&surfaces[7], cost::WHY_HAZARD),
        0,
        "the flood has expired"
    );
    // after expiry, the content equals fusion without the flood layers; the recipes differ
    let mut no_flood = steps[7].layers.clone();
    no_flood.retain(|&k| k != 1 && k != 2);
    let nf = inc(&Step {
        name: "nf",
        t: steps[7].t,
        layers: no_flood,
        overlay: true,
    })
    .surface(steps[7].t)
    .unwrap();
    assert_eq!(nf.content_hash(), surfaces[7].content_hash());
    assert_ne!(nf.recipe_hash(), surfaces[7].recipe_hash());
    // layer input order does not matter
    let mut rev = steps[5].layers.clone();
    rev.reverse();
    let r = inc(&Step {
        name: "rev",
        t: steps[5].t,
        layers: rev,
        overlay: true,
    })
    .surface(steps[5].t)
    .unwrap();
    assert_eq!(r, surfaces[5]);
    // the late layer changes only its own blocks
    let touched = wayfarer_terrain::surface::touched_blocks(fx.layers[4].layer(), SP);
    assert!(
        surfaces[4]
            .blocks
            .iter()
            .all(|(k, (id, b))| surfaces[3].blocks[k].1.hash() == b.hash() || touched.contains(id))
    );

    // Routes from the south-west quarter to four cells, on the base, drone and team surfaces.
    let targets = [(150i64, 220i64), (10, 250), (180, 20), (65, 190)];
    for sn in [0usize, 3, 5] {
        let grid = Grid::new(&fx.ter, &surfaces[sn]).unwrap();
        let (i0, j0) = (BI0 as i64 * BLOCK as i64, BJ0 as i64 * BLOCK as i64);
        let src = grid.cell_of(i0 + 30, j0 + 40).unwrap();
        let g = Graph::new(&grid, 0);
        let mut se = Search::new(grid.h * grid.w);
        for (ti, &(r, c)) in targets.iter().enumerate() {
            let dst = grid.cell_of(i0 + r, j0 + c).unwrap();
            let name = format!("route_{}_{ti}", steps[sn].name);
            match route(&grid, 0, src, dst) {
                None => got.push((name, "unreachable".into())),
                Some(rt) => {
                    let a = se.run(&g, src, Some(dst), true);
                    assert_eq!(a, Some(rt.conf.cost_us), "{name}: A* equals Dijkstra");
                    se.run(&g, src, None, false);
                    assert_eq!(
                        path(&g, &se, src, dst).unwrap(),
                        rt.steps,
                        "{name}: same canonical path from a full field"
                    );
                    let bytes: Vec<u8> = rt
                        .steps
                        .iter()
                        .flat_map(|&(u, m)| [(u as u64).to_le_bytes(), (m as u64).to_le_bytes()])
                        .flatten()
                        .collect();
                    let c = &rt.conf;
                    got.push((
                        name,
                        format!(
                            "{} {} {} {} {} {} {} {} {:?} {}",
                            &blake3::hash(&bytes).to_hex()[..16],
                            c.cost_us,
                            c.len_mm,
                            c.mean_cert,
                            c.min_cert,
                            c.layer_permille,
                            c.team_permille,
                            c.max_risk,
                            c.layers,
                            c.at_edge
                        ),
                    ));
                }
            }
        }
    }

    if std::env::var("UPDATE_GOLDEN").is_ok() || GOLDEN.is_empty() {
        println!("const GOLDEN: &[(&str, &str)] = &[");
        for (k, v) in &got {
            println!("    (\"{k}\", \"{v}\"),");
        }
        println!("];");
    }
    assert!(
        !GOLDEN.is_empty(),
        "no golden table yet: paste the printed one"
    );
    assert_eq!(got.len(), GOLDEN.len(), "golden table size");
    for ((k, v), (gk, gv)) in got.iter().zip(GOLDEN) {
        assert_eq!((k.as_str(), v.as_str()), (*gk, *gv));
    }
}

#[test]
fn terrain_file_on_the_fixture() {
    let ter = base();
    let f = ter.write();
    assert_eq!(Terrain::read(&f).unwrap(), ter);
    assert_eq!(ROWS * COLS, (ter.blocks().len() * BLOCK_POSTS) as i64);
}

//! Replays experiment E14 (lane TERRX) on its real Tatra base and checks that this crate
//! reproduces the experiment's results byte for byte: every block of the eight incident
//! surfaces, the block ops of the seven deltas, and the walker routes' paths.
//!
//! E14 wrote contract v0 files. Contract v1 changed the header and the hashes on purpose (the
//! fuse version, the canonical overlay, new hash domains), so the files' headers and hashes
//! differ; their block sections, which hold everything a route reads, must not.
//!
//! The data is not in the repository (the E5 base is 21 MB). Set `WAYFARER_E14_PREP` to the
//! E5 prep directory (`blocks.txt`, `dem_dm.i32`, `steep.u8`, `lc4.u8`, `conf.u8`,
//! `lines.u16`) and `WAYFARER_E14_OUT` to the E14 output directory (its `.wtl`, `.wis` and
//! `.wisd` files and `e14.json`), then run `cargo test -p wayfarer-terrain --test e14_replay
//! --release`. Without them the test passes without checking anything.

use std::collections::BTreeMap;
use wayfarer_terrain::fuse::{Effect, HashedLayer, OverlayItem, Prior};
use wayfarer_terrain::lattice::{BlockId, Spacing};
use wayfarer_terrain::profile::Rules;
use wayfarer_terrain::search::{Graph, Grid, Search, path};
use wayfarer_terrain::surface::{Incident, Surface, apply_delta, delta, delta_ops};
use wayfarer_terrain::terrain::Terrain;

const SP: i64 = 120;
const E: i64 = 900;
const H: i64 = 3600;
const DAY: i64 = 86_400;
const T0: i64 = 1_795_510_800; // 2026-11-24 09:00 UTC

fn read_vec<T>(path: &str, size: usize, f: impl Fn(&[u8]) -> T) -> Vec<T> {
    std::fs::read(path)
        .unwrap_or_else(|e| panic!("{path}: {e}"))
        .chunks_exact(size)
        .map(f)
        .collect()
}

/// A field of `e14.json` holding an array of integers, by name.
fn json_ints(text: &str, key: &str) -> Vec<i64> {
    let at = text
        .find(&format!("\"{key}\": ["))
        .unwrap_or_else(|| panic!("{key} in e14.json"));
    let rest = &text[at + key.len() + 5..];
    rest[..rest.find(']').unwrap()]
        .split(',')
        .map(|s| s.trim().parse().unwrap())
        .collect()
}

fn json_str<'a>(text: &'a str, key: &str) -> &'a str {
    let at = text
        .find(&format!("\"{key}\": \""))
        .unwrap_or_else(|| panic!("{key} in e14.json"));
    let rest = &text[at + key.len() + 5..];
    &rest[..rest.find('"').unwrap()]
}

#[test]
fn e14_replay() {
    let (Ok(prep), Ok(out)) = (
        std::env::var("WAYFARER_E14_PREP"),
        std::env::var("WAYFARER_E14_OUT"),
    ) else {
        eprintln!("WAYFARER_E14_PREP / WAYFARER_E14_OUT not set: skipping the E14 replay");
        return;
    };
    let text = std::fs::read_to_string(format!("{out}/e14.json")).unwrap();
    let mut blocks: Vec<BlockId> = std::fs::read_to_string(format!("{prep}/blocks.txt"))
        .unwrap()
        .lines()
        .filter_map(|l| {
            let mut it = l.split_whitespace();
            Some(BlockId {
                bi: it.next()?.parse().ok()?,
                bj: it.next()?.parse().ok()?,
            })
        })
        .collect();
    let sorted = blocks.windows(2).all(|w| w[0] < w[1]);
    assert!(sorted, "blocks.txt is in key order");
    blocks.dedup();
    let ter = Terrain::new(
        Spacing::new(SP).unwrap(),
        blocks,
        read_vec(&format!("{prep}/dem_dm.i32"), 4, |b| {
            i32::from_le_bytes(b.try_into().unwrap())
        }),
        std::fs::read(format!("{prep}/steep.u8")).unwrap(),
        std::fs::read(format!("{prep}/lc4.u8")).unwrap(),
        std::fs::read(format!("{prep}/conf.u8")).unwrap(),
        read_vec(&format!("{prep}/lines.u16"), 2, |b| {
            u16::from_le_bytes(b.try_into().unwrap())
        }),
    )
    .unwrap();
    // Terrain file v1 folds the terrain hash per block, so it is no longer E14's
    // terrain_hash; the planes are E14's inputs as loaded above.
    // The terrain file round trip on real data.
    assert_eq!(Terrain::read(&ter.write()).unwrap(), ter);

    let rules = [Rules::walker_default(), Rules::vehicle()];
    let ph = &text[text.find("\"profile_hashes\"").unwrap()..];
    for r in &rules {
        assert!(
            ph[..200].contains(&wayfarer_terrain::hex(&r.hash())),
            "profile {} matches E14",
            r.p.name
        );
    }
    let prior = Prior::default();

    let names = [
        "s2_landcover",
        "s1_flood",
        "s1_depth",
        "drone_landcover",
        "drone_rubble",
        "drone_features",
        "late_landcover",
        "expired_snow",
        "research_landcover",
        "zero_confidence",
    ];
    let ls: Vec<HashedLayer> = names
        .iter()
        .map(|n| {
            let b = std::fs::read(format!("{out}/{n}.wtl")).unwrap();
            let l = HashedLayer::read(&b).unwrap();
            assert_eq!(l.layer().write(), b, "{n}.wtl rewrites to the same bytes");
            l
        })
        .collect();

    let tb = json_ints(&text, "team_block");
    let tc = json_ints(&text, "team_clear");
    let block_item = OverlayItem {
        lat0: (tb[0] - 1) * SP,
        lat1: (tb[0] + 2) * SP,
        lon0: (tb[1] - 1) * SP,
        lon1: (tb[1] + 2) * SP,
        // E14 applied the block before the clear box: observed first.
        observed_at: T0 + 5 * E - 120,
        effect: Effect::Block,
    };
    let clear_item = OverlayItem {
        lat0: (tc[0] - 6) * SP,
        lat1: (tc[0] + 6) * SP,
        lon0: (tc[1] - 6) * SP,
        lon1: (tc[1] + 6) * SP,
        observed_at: T0 + 5 * E - 60,
        effect: Effect::Clear,
    };
    let all: Vec<usize> = vec![0, 7, 8, 9, 1, 2, 3, 4, 5, 6];
    let team = vec![block_item, clear_item];
    let steps: Vec<(&str, i64, Vec<usize>, Vec<OverlayItem>)> = vec![
        ("base", T0, vec![], vec![]),
        ("s2", T0, vec![0, 7, 8, 9], vec![]),
        ("flood", T0, vec![0, 7, 8, 9, 1, 2], vec![]),
        ("drone", T0 + 3 * E, vec![0, 7, 8, 9, 1, 2, 3, 4, 5], vec![]),
        ("late", T0 + 4 * E, all.clone(), vec![]),
        ("team", T0 + 5 * E, all.clone(), team.clone()),
        ("fading", T0 + 2 * DAY + 3 * H, all.clone(), team.clone()),
        ("expired", T0 + 6 * DAY - 3 * H, all, team),
    ];
    let incident = |layers: &[usize], overlay: &[OverlayItem]| Incident {
        id: *b"E14-tatra-test\0\0",
        terrain: &ter,
        prior: &prior,
        layers: layers.iter().map(|&k| &ls[k]).collect(),
        overlay: overlay.to_vec(),
        rules: &rules,
        epoch_s: E,
        allow_research: false,
        feature_min_conf: 128,
        level_min_w: 128,
    };

    let mut surfaces: Vec<Surface> = Vec::new();
    for (sn, (name, t, layers, overlay)) in steps.iter().enumerate() {
        let inc = incident(layers, overlay);
        let full = inc.surface(*t).unwrap();
        let file = full.write();
        let want = std::fs::read(format!("{out}/step{sn}_{name}.wis")).unwrap();
        assert!(
            want.ends_with(&full.block_section()),
            "step {name}: every block is E14's, byte for byte"
        );
        assert_eq!(Surface::read(&file).unwrap(), full);
        if let Some(prev) = surfaces.last() {
            let (_, pt, pl, po) = &steps[sn - 1];
            let before = incident(pl, po);
            let inc_s = inc.update(*t, prev, &before).unwrap();
            assert_eq!(
                inc_s, full,
                "step {name}: incremental equals full (previous T {pt})"
            );
            let d = delta(prev, &full);
            assert!(
                std::fs::read(format!("{out}/step{sn}_{name}.wisd"))
                    .unwrap()
                    .ends_with(&delta_ops(prev, &full)),
                "step {name}: the delta's block ops are E14's"
            );
            assert_eq!(apply_delta(prev, &d).unwrap(), full);
        }
        surfaces.push(full);
    }

    // Walker routes from the incident point on the base, drone and team surfaces.
    let ipp = json_ints(&text, "ipp");
    let mut want: BTreeMap<String, String> = BTreeMap::new();
    for (sn, ti) in (0..6).flat_map(|t| [(0, t), (3, t), (5, t)]) {
        let k = format!("h_route_{sn}_{ti}");
        if text.contains(&format!("\"{k}\"")) {
            want.insert(k.clone(), json_str(&text, &k).to_string());
        }
    }
    let targets: Vec<usize> = {
        let r = &text[text.find("\"routes\"").unwrap()..];
        (0..6)
            .map(|t| {
                let key = format!("\"target\": {t}, \"cell\": ");
                r[r.find(&key).unwrap() + key.len()..]
                    .split(',')
                    .next()
                    .unwrap()
                    .trim()
                    .parse()
                    .unwrap()
            })
            .collect()
    };
    let mut checked = 0;
    for sn in [0usize, 3, 5] {
        let grid = Grid::new(&ter, &surfaces[sn]).unwrap();
        let src = grid.cell_of(ipp[0], ipp[1]).unwrap();
        let g = Graph::new(&grid, 0);
        let mut se = Search::new(grid.h * grid.w);
        for (ti, &tg) in targets.iter().enumerate() {
            let k = format!("h_route_{sn}_{ti}");
            match se.run(&g, src, Some(tg), false) {
                None => assert!(!want.contains_key(&k), "{k}: E14 reached it"),
                Some(cost) => {
                    let steps = path(&g, &se, src, tg).unwrap();
                    let bytes: Vec<u8> = steps
                        .iter()
                        .flat_map(|&(u, m)| [(u as u64).to_le_bytes(), (m as u64).to_le_bytes()])
                        .flatten()
                        .collect();
                    assert_eq!(blake3::hash(&bytes).to_hex().as_str(), want[&k], "{k}");
                    let mut se2 = Search::new(grid.h * grid.w);
                    assert_eq!(
                        se2.run(&g, src, Some(tg), true),
                        Some(cost),
                        "{k}: A* equals Dijkstra"
                    );
                    checked += 1;
                }
            }
        }
    }
    assert_eq!(checked, want.len());
}

//! E10 driver: synthetic model layers on the real Tatra/Podhale base, written as layer files,
//! read back, fused, and checked for determinism (input order, threads, incremental re-fusion,
//! expiry, zero-confidence layers). Prints one JSON object; `h_*` lines are the identity hashes.
//!
//!   terrexp e10 <base-dir> <dem-dir> <out-dir>
//!
//! base-dir holds blocks.txt, base_class.u8 and base_conf.u8 (scripts/e10/base_e3.py);
//! dem-dir holds E1's posts_dm.i32 for the same blocks (used only to place the synthetic flood).

use crate::fuse::{self, Base, Effect, FusedBlock, OverlayItem, Params, S};
use crate::lattice::{self, BLOCK, BLOCK_POSTS, BlockId};
use crate::layers::{GridBlock, Kind, Layer, Manifest, NCLASS, hex};
use crate::voxel::{self, Edge, Graph, Node};
use std::collections::BTreeSet;
use std::time::Instant;

const T: i64 = 1_795_521_600; // 2026-11-24 12:00 UTC, the incident time
const SP: i64 = 120; // 1"

fn mix(mut z: u64) -> u64 {
    z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}
fn rnd(i: i64, j: i64, seed: u64) -> u64 {
    mix(mix(i as u64 ^ seed.rotate_left(17)) ^ (j as u64).wrapping_mul(0xD6E8_FEB8_6659_FD93))
}

fn manifest(kind: Kind, channels: Vec<u16>, model: &str, ver: &str, obs: i64, from: i64, until: i64, fade: i64) -> Manifest {
    Manifest {
        kind, licence: 0, s_lat: SP as u32, s_lon: SP as u32, observed_at: obs, valid_from: from, valid_until: until,
        fade_from: fade, channels, model_id: model.into(), model_version: ver.into(),
        weights: *blake3::hash(format!("weights {model} {ver}").as_bytes()).as_bytes(),
        dataset_manifest: *blake3::hash(format!("datasets {model}").as_bytes()).as_bytes(),
        eval_set: *blake3::hash(format!("eval {model}").as_bytes()).as_bytes(), ece_permille: 31,
        backend: "burn-ndarray (synthetic)".into(), attribution: "synthetic E10 test layer".into(), inputs: vec![],
        parents: vec![],
    }
}

fn post_ij(b: BlockId, q: usize) -> (i64, i64) {
    (b.i0() + (q / BLOCK) as i64, b.j0() + (q % BLOCK) as i64)
}

/// E7 degrees of post (i, j) at 1" (exact only to the E7 unit; used for synthetic geometry).
fn e7_of(i: i64) -> i32 {
    ((i * SP * 10_000_000) / lattice::TICKS_PER_DEG) as i32
}

struct World<'a> {
    blocks: &'a [BlockId],
    class: &'a [u8],
    dem_m: Vec<i32>,
}

impl World<'_> {
    fn grid(&self, slots: usize, mut f: impl FnMut(i64, i64, u8, i32, &mut [u16], &mut u8) -> bool) -> Vec<GridBlock> {
        let mut out = Vec::new();
        for (k, &b) in self.blocks.iter().enumerate() {
            let mut g = GridBlock::new(b, slots);
            let mut vals = vec![0u16; slots];
            for q in 0..BLOCK_POSTS {
                let (i, j) = post_ij(b, q);
                vals.iter_mut().for_each(|v| *v = 0);
                let mut w = 0u8;
                if f(i, j, self.class[k * BLOCK_POSTS + q], self.dem_m[k * BLOCK_POSTS + q], &mut vals, &mut w) {
                    g.set_present(q);
                    g.conf[q] = w;
                    for s in 0..slots {
                        g.vals[s][q] = vals[s];
                    }
                }
            }
            if g.count() > 0 {
                out.push(g);
            }
        }
        out
    }
}

fn finish(mut l: Layer) -> Layer {
    l.canonicalise().expect("synthetic layer is canonical");
    l
}

/// Placeholder walker friction in ms/m (E5 owns the real tables): expected pace over the class
/// distribution, then hazard multipliers, level and mask rules. 0 = impassable.
fn friction(fb: &FusedBlock, q: usize) -> u16 {
    const PACE: [u64; NCLASS] = [0, 1190, 2041, 893, 1020, 1429, 1190, 2381, 0, 2857, 3571, 1020, 4762, 1786, 2381, 2857];
    let p = &fb.p[q];
    let blocked = p[0] as u64 + p[8] as u64;
    if 2 * blocked >= S as u64 {
        return 0;
    }
    let mut num = 0u64;
    for k in 0..NCLASS {
        num += p[k] as u64 * PACE[k];
    }
    let den = S as u64 - blocked;
    let mut pace = (2 * num + den) / (2 * den);
    let h = &fb.haz[q];
    if h[0] >= 128 || h[7] >= 128 {
        return 0; // flood or closure more likely than not
    }
    const K: [u64; 8] = [512, 768, 512, 1024, 256, 512, 768, 1024];
    for s in 0..8 {
        let d = 255 * 256;
        pace = (pace * (d + h[s] as u64 * K[s]) + d / 2) / d;
    }
    if fb.lev[q][0].0 > 80 {
        return 0; // water deeper than 80 cm
    }
    let m = fb.mask[q];
    if m & (fuse::M_TEAM_BLOCK | fuse::M_WASHOUT) != 0 {
        return 0;
    }
    if m & (fuse::M_DEBRIS | fuse::M_FALLEN) != 0 {
        pace *= 2;
    }
    if m & fuse::M_PASSAGE != 0 {
        pace = pace.min(1000);
    }
    pace.clamp(1, 65535) as u16
}

fn runs_len_u16(v: &[u16]) -> usize {
    let mut n = 0;
    let mut i = 0;
    while i < v.len() {
        let mut j = i + 1;
        while j < v.len() && v[j] == v[i] {
            j += 1;
        }
        let mut x = (j - i - 1) as u64;
        n += 2;
        loop {
            n += 1;
            x >>= 7;
            if x == 0 {
                break;
            }
        }
        i = j;
    }
    n.min(2 * v.len()) + 1
}

fn touched_blocks(l: &Layer) -> BTreeSet<BlockId> {
    let mut s: BTreeSet<BlockId> = l.blocks.iter().map(|b| b.id).collect();
    for f in &l.features {
        for seg in f.pts.windows(2) {
            fuse::supercover(seg[0], seg[1], SP, &mut |i, j| {
                s.insert(BlockId { bi: i.div_euclid(64) as i32, bj: j.div_euclid(64) as i32 });
            });
        }
    }
    s
}

fn in_pool<R: Send>(threads: usize, f: impl FnOnce() -> R + Send) -> R {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
}

pub fn run(args: &[String]) {
    let (bdir, ddir, out) = (&args[0], &args[1], &args[2]);
    std::fs::create_dir_all(out).unwrap();
    let blocks = lattice::read_blocks(&format!("{bdir}/blocks.txt")).unwrap();
    // read_blocks sorts; the base files are in blocks.txt order, which E1/E3 write sorted too.
    let raw_order = std::fs::read_to_string(format!("{bdir}/blocks.txt")).unwrap();
    let listed: Vec<BlockId> = raw_order
        .lines()
        .filter_map(|l| {
            let mut it = l.split_whitespace();
            Some(BlockId { bi: it.next()?.parse().ok()?, bj: it.next()?.parse().ok()? })
        })
        .collect();
    assert_eq!(listed, blocks, "base blocks must be listed in key order");
    let class = std::fs::read(format!("{bdir}/base_class.u8")).unwrap();
    let conf = std::fs::read(format!("{bdir}/base_conf.u8")).unwrap();
    let dem_dm: Vec<i32> = std::fs::read(format!("{ddir}/posts_dm.i32"))
        .unwrap()
        .chunks(4)
        .map(|c| i32::from_le_bytes(c.try_into().unwrap()))
        .collect();
    let dem_m: Vec<i32> = dem_dm.iter().map(|&v| lattice::div_round(v as i128, 10) as i32).collect();
    let base = Base::new(blocks.clone(), class.clone(), conf);
    let world = World { blocks: &blocks, class: &class, dem_m };

    // ---- synthetic layers -------------------------------------------------------------------
    let year = 1_818_201_600; // 2027-08-14
    // An old land-cover refresh on every fifth block: grass, full confidence (to be outvoted by
    // the newer S2 layer where they overlap, and to show fusion order matters and is fixed).
    let old = finish(Layer {
        m: manifest(Kind::Class, vec![], "m1-s2-landcover", "0.9", 1_780_308_000, 1_780_308_000, year, year),
        blocks: world
            .grid(NCLASS, |_, _, c, _, v, w| {
                v[3] = 255;
                *w = 255;
                c != 0
            })
            .into_iter()
            .filter(|g| (g.id.bi + g.id.bj).rem_euclid(5) == 0)
            .collect(),
        features: vec![],
    });
    // The S2 refresh: soft distributions around the base class, clouds as holes, haze as low w.
    let clouds: Vec<(i64, i64, i64, bool)> = (0..6)
        .map(|k| {
            let r = mix(1000 + k);
            let b = blocks[(r % blocks.len() as u64) as usize];
            (b.i0() + 32, b.j0() + 32, 40 + (r >> 20) as i64 % 60, k % 2 == 0)
        })
        .collect();
    let s2 = finish(Layer {
        m: manifest(Kind::Class, vec![], "m1-s2-landcover", "1.0", 1_786_701_600, 1_786_701_600, year, year),
        blocks: world.grid(NCLASS, |i, j, c, _, v, w| {
            if c == 0 {
                return false;
            }
            let mut haze = false;
            for &(ci, cj, r, hole) in &clouds {
                if (i - ci).pow(2) + (j - cj).pow(2) <= r * r {
                    if hole {
                        return false;
                    }
                    haze = true;
                }
            }
            let r = rnd(i, j, 7);
            let alt = 1 + ((r >> 8) % 15) as usize;
            let c = c as usize;
            match r % 100 {
                0..70 => {
                    v[c] += 230;
                    v[alt] += 25;
                }
                70..90 => {
                    v[c] += 150;
                    v[alt] += 80;
                    v[0] += 25;
                }
                _ => {
                    v[alt] += 200;
                    v[c] += 55;
                }
            }
            *w = if haze { 60 } else { 180 };
            true
        }),
        features: vec![],
    });
    // A Sentinel-1 flood model: probability from height below 700 m; valid a week, fading after 3 days.
    let (f_obs, f_until, f_fade) = (1_795_154_400, 1_795_759_200, 1_795_413_600);
    let flood = finish(Layer {
        m: manifest(Kind::Hazard, vec![1], "m2-s1-flood", "1.0", f_obs, f_obs, f_until, f_fade),
        blocks: world.grid(1, |_, _, _, h, v, w| {
            if h >= 760 {
                return false;
            }
            v[0] = ((700 - h).clamp(0, 60) * 255 / 60) as u16;
            *w = 200;
            true
        }),
        features: vec![],
    });
    let depth = finish(Layer {
        m: manifest(Kind::Level, vec![1], "m2-s1-flood-depth", "1.0", f_obs, f_obs, f_until, f_fade),
        blocks: world.grid(1, |_, _, _, h, v, w| {
            if h >= 690 {
                return false;
            }
            v[0] = ((700 - h) * 10).clamp(0, 300) as u16;
            *w = 200;
            true
        }),
        features: vec![],
    });
    // Drone survey near Dolina Kościeliska (49.27 N, 19.87 E): bare ground from a landslide.
    let (di, dj) = (49_27 * lattice::TICKS_PER_DEG / 100 / SP, 19_87 * lattice::TICKS_PER_DEG / 100 / SP);
    let d_obs = 1_795_435_200;
    let drone_lc = finish(Layer {
        m: manifest(Kind::Class, vec![], "m4-drone-landcover", "0.3", d_obs, d_obs, d_obs + 30 * 86_400, d_obs + 30 * 86_400),
        blocks: world.grid(NCLASS, |i, j, c, _, v, w| {
            if c == 0 || (i - di).pow(2) + (j - dj).pow(2) > 24 * 24 {
                return false;
            }
            v[6] = 180;
            v[0] = 75;
            *w = 240;
            true
        }),
        features: vec![],
    });
    let expired = finish(Layer {
        m: manifest(Kind::Hazard, vec![5], "snow-cover", "1.0", 1_756_684_800, 1_756_684_800, 1_759_276_800, 1_759_276_800),
        blocks: world.grid(1, |_, _, _, _, v, w| {
            v[0] = 255;
            *w = 255;
            true
        })[..4]
            .to_vec(),
        features: vec![],
    });
    let mut research = finish(Layer {
        m: manifest(Kind::Class, vec![], "landcover-ai-research", "0.1", d_obs, d_obs, year, year),
        blocks: world.grid(NCLASS, |i, j, c, _, v, w| {
            if c == 0 || (i - di).abs() > 40 || (j - dj).abs() > 40 {
                return false;
            }
            v[13] = 255;
            *w = 255;
            true
        }),
        features: vec![],
    });
    research.m.licence = 1;
    research.canonicalise().unwrap();
    let zero = finish(Layer {
        m: manifest(Kind::Class, vec![], "m1-s2-landcover", "1.0-zero", 1_786_701_600, 1_786_701_600, year, year),
        blocks: world.grid(NCLASS, |_, _, c, _, v, w| {
            if c == 0 {
                return false;
            }
            v[1] = 255;
            *w = 0;
            true
        }),
        features: vec![],
    });

    // A tiny voxel graph from the drone (5 m voxels, 150 m square): a rubble disc, a fallen trunk
    // line, a washout pair and a passage walked through the rubble.
    let (clat, clon) = (e7_of(di), e7_of(dj));
    let (dlat, dlon) = (450i32, 690i32); // ~5 m at 49.27 N
    let mut nodes = Vec::new();
    for a in -15..15i32 {
        for b in -15..15i32 {
            let id = ((a + 15) * 30 + (b + 15)) as u64 + 1;
            let r2 = a * a + b * b;
            let class = if a == b && a > -12 && a < 12 {
                voxel::V_TRUNK
            } else if (a, b) == (8, -6) || (a, b) == (8, -5) {
                voxel::V_DROP
            } else if r2 <= 64 {
                if mix(id) % 4 == 0 { voxel::V_DEBRIS } else { voxel::V_RUBBLE }
            } else if r2 <= 100 {
                voxel::V_VEG_LOW
            } else {
                voxel::V_GROUND
            };
            let conf = 150 + (mix(id ^ 77) % 100) as u8;
            nodes.push(Node { id, lat: clat + a * dlat, lon: clon + b * dlon, h_dm: 9_500 + a * 3, class, conf });
        }
    }
    let nid = |a: i32, b: i32| ((a + 15) * 30 + (b + 15)) as u64 + 1;
    let mut edges = Vec::new();
    for a in -11..10 {
        edges.push(Edge { a: nid(a, a), b: nid(a + 1, a + 1), kind: voxel::E_BLOCKED, conf: 220 });
    }
    edges.push(Edge { a: nid(8, -6), b: nid(8, -5), kind: voxel::E_BLOCKED, conf: 200 });
    for b in -14..14 {
        edges.push(Edge { a: nid(-3, b), b: nid(-3, b + 1), kind: voxel::E_TRAVERSABLE, conf: 180 });
    }
    let mut graph = Graph { nodes, edges, observed_at: 1_795_442_400 };
    graph.canonicalise().unwrap();
    let tmpl = manifest(Kind::Hazard, vec![1], "m4-voxel-projection", "proj-v0 n_full=8", 0, d_obs, d_obs + 30 * 86_400, d_obs + 30 * 86_400);
    let (vox_haz, vox_feat) = voxel::project(&graph, SP, &tmpl);

    // Team overlay: they walked part of the flood (clear), and closed a box by the drone site.
    let first_flood = flood
        .blocks
        .iter()
        .flat_map(|b| (0..BLOCK_POSTS).filter(|&q| b.has(q) && b.vals[0][q] >= 200).map(move |q| post_ij(b.id, q)))
        .next()
        .unwrap();
    let overlay = vec![
        OverlayItem {
            lat0: first_flood.0 * SP,
            lat1: (first_flood.0 + 10) * SP,
            lon0: first_flood.1 * SP,
            lon1: (first_flood.1 + 10) * SP,
            effect: Effect::Clear,
        },
        OverlayItem { lat0: (di + 2) * SP, lat1: (di + 6) * SP, lon0: (dj + 10) * SP, lon1: (dj + 14) * SP, effect: Effect::Block },
    ];

    // ---- files -----------------------------------------------------------------------------
    let named: Vec<(&str, &Layer)> = vec![
        ("old_landcover", &old),
        ("s2_landcover", &s2),
        ("s1_flood", &flood),
        ("s1_flood_depth", &depth),
        ("drone_landcover", &drone_lc),
        ("voxel_hazard", &vox_haz),
        ("voxel_features", &vox_feat),
        ("expired_snow", &expired),
        ("research_landcover", &research),
        ("zero_confidence", &zero),
    ];
    let mut file_json = Vec::new();
    let mut loaded: Vec<Layer> = Vec::new();
    let tw = Instant::now();
    for (name, l) in &named {
        let bytes = l.write(None);
        let raw = l.write(Some(0));
        std::fs::write(format!("{out}/{name}.wtl"), &bytes).unwrap();
        let back = Layer::read(&bytes).unwrap();
        assert_eq!(back.content_hash(), l.content_hash());
        assert_eq!(Layer::read(&raw).unwrap().content_hash(), l.content_hash(), "hash must not depend on the coding");
        let posts: usize = l.blocks.iter().map(|b| b.count()).sum();
        file_json.push(format!(
            "    {{\"name\": \"{name}\", \"kind\": \"{:?}\", \"blocks\": {}, \"posts\": {posts}, \"features\": {}, \"bytes\": {}, \"bytes_raw_planes\": {}, \"h_{name}\": \"{}\"}}",
            l.m.kind,
            l.blocks.len(),
            l.features.len(),
            bytes.len(),
            raw.len(),
            hex(&l.content_hash())
        ));
        loaded.push(back);
    }
    let t_files = tw.elapsed().as_secs_f64();
    let all: Vec<(&Layer, [u8; 32])> = loaded.iter().map(|l| (l, l.content_hash())).collect();
    let prm = |t: i64, research: bool| Params { t, allow_research: research, feature_min_conf: 128, level_min_w: 128, spacing: SP };

    // ---- fusion and checks -----------------------------------------------------------------
    let t0 = Instant::now();
    let (fused, rec) = fuse::fuse(&base, &all, &overlay, &prm(T, false), None);
    let t_fuse = t0.elapsed().as_secs_f64();
    let t1 = Instant::now();
    let (_, rec1) = in_pool(1, || fuse::fuse(&base, &all, &overlay, &prm(T, false), None));
    let t_fuse1 = t1.elapsed().as_secs_f64();
    let mut rev = all.clone();
    rev.reverse();
    let mut rot = all.clone();
    rot.rotate_left(3);
    let (_, rec_rev) = in_pool(3, || fuse::fuse(&base, &rev, &overlay, &prm(T, false), None));
    let (_, rec_rot) = fuse::fuse(&base, &rot, &overlay, &prm(T, false), None);
    let order_ok = rec1.fused == rec.fused && rec_rev.fused == rec.fused && rec_rot.fused == rec.fused;

    // incremental: fuse without the drone layers, then re-fuse only the blocks they touch
    let drone: Vec<&Layer> = vec![&loaded[4], &loaded[5], &loaded[6]];
    let without: Vec<(&Layer, [u8; 32])> = all.iter().copied().filter(|l| !drone.iter().any(|d| std::ptr::eq(*d, l.0))).collect();
    let (mut inc, _) = fuse::fuse(&base, &without, &overlay, &prm(T, false), None);
    let mut touched: BTreeSet<BlockId> = BTreeSet::new();
    for d in &drone {
        touched.extend(touched_blocks(d));
    }
    let tset: Vec<BlockId> = touched.iter().copied().collect();
    let ti = Instant::now();
    let (part, _) = fuse::fuse(&base, &all, &overlay, &prm(T, false), Some(&tset));
    let t_inc = ti.elapsed().as_secs_f64();
    for (id, fb) in part {
        let k = inc.iter().position(|x| x.0 == id).unwrap();
        inc[k].1 = fb;
    }
    let incremental_ok = fuse::hash_blocks(&inc) == rec.fused;

    // zero-confidence layer is a no-op on the fused state
    let no_zero: Vec<(&Layer, [u8; 32])> = all.iter().copied().filter(|l| !std::ptr::eq(l.0, &loaded[9])).collect();
    let (_, rec_nz) = fuse::fuse(&base, &no_zero, &overlay, &prm(T, false), None);
    let zero_ok = rec_nz.fused == rec.fused;

    // after the flood window closes, the flood layers change nothing
    let t_late = 1_796_601_600; // 2026-12-07
    let (_, rec_late) = fuse::fuse(&base, &all, &overlay, &prm(t_late, false), None);
    let no_flood: Vec<(&Layer, [u8; 32])> = all.iter().copied().filter(|l| l.0.m.model_id != "m2-s1-flood" && l.0.m.model_id != "m2-s1-flood-depth").collect();
    let (_, rec_late_nf) = fuse::fuse(&base, &no_flood, &overlay, &prm(t_late, false), None);
    let expiry_ok = rec_late.fused == rec_late_nf.fused && rec_late.fused != rec.fused;
    let t_fade = 1_795_629_600; // 2026-11-25 18:00, inside the flood's fade
    let (_, rec_fade) = fuse::fuse(&base, &all, &overlay, &prm(t_fade, false), None);
    let (_, rec_res) = fuse::fuse(&base, &all, &overlay, &prm(T, true), None);

    // ---- a sample for the independent Python check (scripts/e10/check_fusion.py) ---------
    {
        let mut sample: BTreeSet<BlockId> = tset.iter().copied().collect();
        for (k, it) in overlay.iter().enumerate() {
            let (b, _) = (BlockId { bi: (it.lat0 / SP).div_euclid(64) as i32, bj: (it.lon0 / SP).div_euclid(64) as i32 }, k);
            sample.insert(b);
        }
        for k in [0usize, 97, 288, 401, 575] {
            sample.insert(blocks[k.min(blocks.len() - 1)]);
        }
        for b in &loaded[2].blocks[..3] {
            sample.insert(b.id);
        }
        let mut bin = Vec::new();
        for (id, fb) in &fused {
            if sample.contains(id) {
                bin.extend_from_slice(&fb.bytes(*id));
            }
        }
        std::fs::write(format!("{out}/fused_sample.bin"), &bin).unwrap();
        let items: Vec<String> = overlay
            .iter()
            .map(|it| {
                let e = match it.effect {
                    Effect::Block => "[\"block\"]".to_string(),
                    Effect::Clear => "[\"clear\"]".to_string(),
                    Effect::SetHazard(c, v) => format!("[\"hazard\", {c}, {v}]"),
                };
                format!("[{}, {}, {}, {}, {e}]", it.lat0, it.lat1, it.lon0, it.lon1)
            })
            .collect();
        let files: Vec<String> = named.iter().map(|(n, _)| format!("\"{n}.wtl\"")).collect();
        std::fs::write(
            format!("{out}/fusion_inputs.json"),
            format!(
                "{{\"t\": {T}, \"spacing\": {SP}, \"allow_research\": false, \"feature_min_conf\": 128, \"level_min_w\": 128, \"layers\": [{}], \"overlay\": [{}], \"sample_blocks\": {}, \"fused_hash\": \"{}\"}}\n",
                files.join(", "),
                items.join(", "),
                sample.len(),
                hex(&rec.fused)
            ),
        )
        .unwrap();
    }

    // ---- statistics and the placeholder friction surface ---------------------------------
    let (mut changed, mut cert_sum, mut imp, mut masked, mut team) = (0u64, 0u64, 0u64, 0u64, 0u64);
    let mut fr_h = blake3::Hasher::new();
    let mut fr_bytes = 0usize;
    let (mut imp_base, mut fr_changed) = (0u64, 0u64);
    let (base_fused, _) = fuse::fuse(&base, &[], &[], &prm(T, false), None);
    for ((id, fb), (_, bb)) in fused.iter().zip(base_fused.iter()) {
        let fr: Vec<u16> = (0..BLOCK_POSTS).map(|q| friction(fb, q)).collect();
        let fr0: Vec<u16> = (0..BLOCK_POSTS).map(|q| friction(bb, q)).collect();
        fr_h.update(&id.key().to_le_bytes());
        for v in &fr {
            fr_h.update(&v.to_le_bytes());
        }
        fr_bytes += runs_len_u16(&fr) + 16;
        for q in 0..BLOCK_POSTS {
            changed += (fb.src[q] != 0) as u64;
            team += (fb.src[q] == 255) as u64;
            cert_sum += fb.cert[q] as u64;
            imp += (fr[q] == 0) as u64;
            imp_base += (fr0[q] == 0) as u64;
            masked += (fb.mask[q] != 0) as u64;
            fr_changed += (fr[q] != fr0[q]) as u64;
        }
    }
    let n = (blocks.len() * BLOCK_POSTS) as u64;
    let rec_json = |r: &fuse::Record| -> String {
        let ap: Vec<String> = r.applied.iter().map(|a| format!("\"{:?} {} {}\"", a.kind, a.observed_at, &hex(&a.hash)[..16])).collect();
        let ig: Vec<String> = r.ignored.iter().map(|(h, w)| format!("\"{} {w}\"", &hex(h)[..16])).collect();
        format!(
            "{{\"t\": {}, \"base\": \"{}\", \"prior\": \"{}\", \"overlay\": \"{}\", \"applied\": [{}], \"ignored\": [{}], \"fused\": \"{}\"}}",
            r.t,
            &hex(&r.base)[..16],
            &hex(&r.prior)[..16],
            &hex(&r.overlay)[..16],
            ap.join(", "),
            ig.join(", "),
            hex(&r.fused)
        )
    };
    println!("{{");
    println!("  \"cmd\": \"e10\", \"blocks\": {}, \"posts\": {n}, \"threads\": {},", blocks.len(), rayon::current_num_threads());
    println!("  \"voxel_graph\": {{\"nodes\": {}, \"edges\": {}, \"h_graph\": \"{}\"}},", graph.nodes.len(), graph.edges.len(), hex(&graph.hash()));
    println!("  \"files\": [\n{}\n  ],", file_json.join(",\n"));
    println!("  \"record\": {},", rec_json(&rec));
    println!("  \"checks\": {{\"order_and_threads\": {order_ok}, \"incremental\": {incremental_ok}, \"zero_confidence_noop\": {zero_ok}, \"expiry\": {expiry_ok}, \"incremental_blocks\": {}}},", tset.len());
    println!("  \"h_fused_T\": \"{}\",", hex(&rec.fused));
    println!("  \"h_fused_fade\": \"{}\",", hex(&rec_fade.fused));
    println!("  \"h_fused_late\": \"{}\",", hex(&rec_late.fused));
    println!("  \"h_fused_research\": \"{}\",", hex(&rec_res.fused));
    println!("  \"h_friction_T\": \"{}\",", fr_h.finalize().to_hex());
    println!(
        "  \"stats\": {{\"posts_changed_by_layers\": {changed}, \"posts_set_by_team\": {team}, \"mean_certainty\": {:.4}, \"posts_with_mask_bits\": {masked}, \"impassable_base\": {imp_base}, \"impassable_fused\": {imp}, \"friction_changed\": {fr_changed}, \"friction_runs_bytes\": {fr_bytes}}},",
        cert_sum as f64 / n as f64 / 255.0
    );
    println!("  \"t_write_read_s\": {t_files:.3}, \"t_fuse_s\": {t_fuse:.3}, \"t_fuse_1thread_s\": {t_fuse1:.3}, \"t_incremental_s\": {t_inc:.3}");
    println!("}}");
    assert!(order_ok && incremental_ok && zero_ok && expiry_ok, "a determinism check failed");
}

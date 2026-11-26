//! E14 driver: an incident timeline on E5's real Tatra base (20 x 20 km at 1"). Layers arrive one
//! by one, a team overlay set changes, the flood model fades and expires; at every step the
//! surface is built in full and incrementally, written, read back, and sent as a delta, and
//! routes from the incident point report their confidence. Prints one JSON object; `h_*` lines
//! are the identity hashes.
//!
//!   terrexp e14 <prep-dir> <pace-csv> <out-dir>
//!
//! prep-dir is E5's stage-1 prep (blocks.txt, dem_dm.i32, steep.u8, lc4.u8, conf.u8, lines.u16).

use crate::e10::{World, e7_of, finish, manifest, mix};
use crate::fuse::{self, Base, Effect, OverlayItem};
use crate::incident::{self, Inputs, Rules, Surface};
use crate::lattice::{self, BLOCK, BLOCK_POSTS, BlockId, Spacing};
use crate::layers::{Feature, Kind, Layer, NCLASS, hex};
use crate::lines::*;
use crate::surface;
use std::collections::BTreeSet;
use std::time::Instant;

const SP: i64 = 120;
const E: i64 = 900; // epoch: 15 minutes
const H: i64 = 3600;
const T0: i64 = 1_795_510_800; // 2026-11-24 09:00 UTC

fn read_vec<T>(path: &str, size: usize, from: fn(&[u8]) -> T) -> Vec<T> {
    std::fs::read(path).unwrap_or_else(|e| panic!("{path}: {e}")).chunks_exact(size).map(from).collect()
}

fn in_pool<R: Send>(threads: usize, f: impl FnOnce() -> R + Send) -> R {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
}

fn ms(t: Instant) -> f64 {
    (t.elapsed().as_secs_f64() * 1e4).round() / 10.0
}

struct Step {
    name: &'static str,
    t: i64,
    layers: Vec<usize>,
    overlay: Vec<OverlayItem>,
}

pub fn run(args: &[String]) {
    let (prep, csv, out) = (&args[0], &args[1], &args[2]);
    std::fs::create_dir_all(out).unwrap();
    let s = Spacing::new(SP).unwrap();
    let blocks = lattice::read_blocks(&format!("{prep}/blocks.txt")).unwrap();
    let dm = read_vec(&format!("{prep}/dem_dm.i32"), 4, |b| i32::from_le_bytes(b.try_into().unwrap()));
    let steep = std::fs::read(format!("{prep}/steep.u8")).unwrap();
    let lc = std::fs::read(format!("{prep}/lc4.u8")).unwrap();
    let conf = std::fs::read(format!("{prep}/conf.u8")).unwrap();
    let ln = read_vec(&format!("{prep}/lines.u16"), 2, |b| u16::from_le_bytes(b.try_into().unwrap()));
    let text = std::fs::read_to_string(csv).unwrap();
    let hdr: Vec<&str> = text.lines().next().unwrap().split(',').collect();
    let col = hdr.iter().position(|&h| h == "mean_pace_permille").unwrap();
    let grade_rel: Vec<u32> = text.lines().skip(1).map(|l| l.split(',').nth(col).unwrap().parse().unwrap()).collect();
    let rules = [Rules { p: surface::walker(grade_rel), x: incident::walker_ext() }, Rules { p: surface::vehicle(), x: incident::vehicle_ext() }];

    let eb = surface::Base::from_blocks(s, &blocks, &dm, &steep, &lc, &ln);
    let ter = incident::Terrain::new(blocks.clone(), &dm, steep.clone(), &lc, &conf, ln.clone());
    let base = Base::new(blocks.clone(), lc.clone(), conf.clone());
    let dem_m: Vec<i32> = dm.iter().map(|&v| lattice::div_round(v as i128, 10) as i32).collect();
    let world = World { blocks: &blocks, class: &lc, dem_m };
    let (h, w) = (eb.h as i64, eb.w as i64);
    let (i_org, j_org) = (eb.bi0 * BLOCK as i64, eb.bj0 * BLOCK as i64);
    let at = |i: i64, j: i64| -> Option<usize> {
        let (r, c) = (i - i_org, j - j_org);
        (r >= 0 && c >= 0 && r < h && c < w).then(|| (r * w + c) as usize)
    };
    let ij = |k: usize| (k as i64 / w + i_org, k as i64 % w + j_org);

    // E5's incident point: the walker-passable cell nearest the centre (on the base-only surface).
    let f0 = surface::build(&rules[0].p, &eb);
    let (cr, cc) = (h / 2, w / 2);
    let ipp = (|| {
        for rad in 0..h / 2 {
            for dr in -rad..=rad {
                for dc in -rad..=rad {
                    let k = ((cr + dr) * w + cc + dc) as usize;
                    if f0[k] > 0 {
                        return k;
                    }
                }
            }
        }
        panic!("no passable cell")
    })();
    let (ipp_i, ipp_j) = ij(ipp);
    let nearest = |i: i64, j: i64, pred: &dyn Fn(usize) -> bool| -> usize {
        (0..(h * w) as usize)
            .filter(|&k| pred(k))
            .min_by_key(|&k| {
                let (a, b) = ij(k);
                ((a - i).pow(2) + (b - j).pow(2), k)
            })
            .unwrap()
    };

    // River distance (Chebyshev, cells) for the synthetic flood, below 880 m.
    let mut rd = vec![u8::MAX; (h * w) as usize];
    for k in 0..(h * w) as usize {
        if eb.lines[k] & (RIVER | STREAM) != 0 && eb.dm[k] < 8800 {
            let (r, c) = (k as i64 / w, k as i64 % w);
            for dr in -5i64..=5 {
                for dc in -5i64..=5 {
                    let (nr, nc) = (r + dr, c + dc);
                    if nr >= 0 && nc >= 0 && nr < h && nc < w {
                        let d = dr.abs().max(dc.abs()) as u8;
                        let x = &mut rd[(nr * w + nc) as usize];
                        *x = (*x).min(d);
                    }
                }
            }
        }
    }

    // ---- layers -----------------------------------------------------------------------------
    let year = 1_818_201_600;
    let day = 86_400;
    let mut ls: Vec<(&str, Layer)> = Vec::new();
    // 0: Sentinel-2 land-cover refresh, 2026-11-20 10:00, soft around the base, two cloud holes.
    let s2_obs = 1_795_168_800;
    let clouds: Vec<(i64, i64, i64)> = (0..2)
        .map(|k| {
            let r = mix(2000 + k);
            let b = blocks[(r % blocks.len() as u64) as usize];
            (b.i0() + 32, b.j0() + 32, 30 + (r >> 20) as i64 % 40)
        })
        .collect();
    ls.push((
        "s2_landcover",
        finish(Layer {
            m: manifest(Kind::Class, vec![], "m1-dynamicworld", "1.0", s2_obs, s2_obs, year, year),
            blocks: world.grid(NCLASS, |i, j, c, _, v, wt| {
                if c == 0 || clouds.iter().any(|&(ci, cj, r)| (i - ci).pow(2) + (j - cj).pow(2) <= r * r) {
                    return false;
                }
                let r = crate::e10::rnd(i, j, 11);
                let alt = 1 + ((r >> 8) % 15) as usize;
                let c = c as usize;
                match r % 100 {
                    0..75 => {
                        v[c] += 230;
                        v[alt] += 25;
                    }
                    75..93 => {
                        v[c] += 150;
                        v[alt] += 80;
                        v[0] += 25;
                    }
                    _ => {
                        v[alt] += 200;
                        v[c] += 55;
                    }
                }
                *wt = 180;
                true
            }),
            features: vec![],
        }),
    ));
    // 1, 2: Sentinel-1 flood probability and depth, 2026-11-23 05:00, a week, fading after 3 days.
    let (f_obs, f_until, f_fade) = (1_795_410_000, 1_795_410_000 + 7 * day, 1_795_410_000 + 3 * day);
    let rdist = |i: i64, j: i64| at(i, j).map_or(u8::MAX, |k| rd[k]);
    ls.push((
        "s1_flood",
        finish(Layer {
            m: manifest(Kind::Hazard, vec![1], "m2-ai4g-flood", "1.0", f_obs, f_obs, f_until, f_fade),
            blocks: world.grid(1, |i, j, _, _, v, wt| {
                let d = rdist(i, j) as u16;
                if d > 5 {
                    return false;
                }
                v[0] = 255 - 40 * d;
                *wt = 220;
                true
            }),
            features: vec![],
        }),
    ));
    ls.push((
        "s1_depth",
        finish(Layer {
            m: manifest(Kind::Level, vec![1], "m2-ai4g-flood-depth", "1.0", f_obs, f_obs, f_until, f_fade),
            blocks: world.grid(1, |i, j, _, _, v, wt| {
                let d = rdist(i, j) as u16;
                if d > 4 {
                    return false;
                }
                v[0] = 120 - 25 * d;
                *wt = 200;
                true
            }),
            features: vec![],
        }),
    ));
    // 3-5: drone survey 2026-11-24 09:30 near the incident point: a landslide (bare ground and
    // rubble), fallen trees across the nearest trails, a washout on the nearest track, a passage.
    let d_obs = T0 + 1800;
    let lref = &eb.lines;
    let on = |bits: u16| move |k: usize| lref[k] & bits != 0;
    let dc = nearest(ipp_i + 20, ipp_j, &on(PATH | PATH_MOUNTAIN | PATH_DEMANDING));
    let (di, dj) = ij(dc);
    ls.push((
        "drone_landcover",
        finish(Layer {
            m: manifest(Kind::Class, vec![], "m4-drone-landcover", "0.3", d_obs, d_obs, d_obs + 30 * day, d_obs + 30 * day),
            blocks: world.grid(NCLASS, |i, j, c, _, v, wt| {
                if c == 0 || (i - di).pow(2) + (j - dj).pow(2) > 12 * 12 {
                    return false;
                }
                v[6] = 180;
                v[0] = 75;
                *wt = 240;
                true
            }),
            features: vec![],
        }),
    ));
    ls.push((
        "drone_rubble",
        finish(Layer {
            m: manifest(Kind::Hazard, vec![2], "m4-drone-voxel", "0.3", d_obs, d_obs, d_obs + 30 * day, d_obs + 30 * day),
            blocks: world.grid(1, |i, j, _, _, v, wt| {
                if (i - di).pow(2) + (j - dj).pow(2) > 8 * 8 {
                    return false;
                }
                v[0] = 200;
                *wt = 230;
                true
            }),
            features: vec![],
        }),
    ));
    let pt = |i: i64, j: i64| (e7_of(i), e7_of(j));
    let mut trail_cells: Vec<usize> = Vec::new();
    {
        let mut seen = BTreeSet::new();
        let mut cand: Vec<usize> = (0..(h * w) as usize).filter(|&k| on(PATH | PATH_MOUNTAIN | PATH_DEMANDING)(k)).collect();
        cand.sort_by_key(|&k| {
            let (a, b) = ij(k);
            ((a - di).pow(2) + (b - dj).pow(2), k)
        });
        for k in cand {
            let (a, b) = ij(k);
            if (a - di).pow(2) + (b - dj).pow(2) < 15 * 15 || seen.iter().any(|&(x, y): &(i64, i64)| (x - a).abs() < 30 && (y - b).abs() < 30) {
                continue;
            }
            seen.insert((a, b));
            trail_cells.push(k);
            if trail_cells.len() == 3 {
                break;
            }
        }
    }
    let wo = nearest(di, dj, &on(TRACK));
    let (wi, wj) = ij(wo);
    let mut feats: Vec<Feature> = trail_cells
        .iter()
        .map(|&k| {
            let (a, b) = ij(k);
            Feature { class: crate::layers::FEAT_FALLEN, conf: 210, pts: vec![pt(a - 1, b - 1), pt(a + 1, b + 1)] }
        })
        .collect();
    feats.push(Feature { class: crate::layers::FEAT_WASHOUT, conf: 230, pts: vec![pt(wi - 1, wj), pt(wi + 1, wj)] });
    feats.push(Feature { class: crate::layers::FEAT_WASHOUT, conf: 230, pts: vec![pt(wi, wj - 1), pt(wi, wj + 1)] });
    feats.push(Feature { class: crate::layers::FEAT_PASSAGE, conf: 200, pts: vec![pt(di, dj), pt(di + 4, dj + 15)] });
    feats.push(Feature { class: crate::layers::FEAT_DEBRIS, conf: 90, pts: vec![pt(di - 8, dj - 8), pt(di - 8, dj + 8)] }); // below the threshold
    ls.push((
        "drone_features",
        finish(Layer {
            m: manifest(Kind::Feature, vec![], "m4-drone-voxel", "0.3", d_obs, d_obs, d_obs + 30 * day, d_obs + 30 * day),
            blocks: vec![],
            features: feats,
        }),
    ));
    // 6: an older land-cover refresh (2026-11-15) that arrives late, on the 2 x 3 blocks around
    // the incident point: fused before the S2 refresh although it arrives after it.
    let late_obs = 1_794_744_000;
    let ib = BlockId { bi: ipp_i.div_euclid(64) as i32, bj: ipp_j.div_euclid(64) as i32 };
    ls.push((
        "late_landcover",
        finish(Layer {
            m: manifest(Kind::Class, vec![], "m1-dynamicworld", "0.9", late_obs, late_obs, year, year),
            blocks: world
                .grid(NCLASS, |_, _, c, _, v, wt| {
                    v[3] = 255;
                    *wt = 255;
                    c != 0
                })
                .into_iter()
                .filter(|g| (g.id.bi == ib.bi || g.id.bi == ib.bi + 1) && (g.id.bj - ib.bj).abs() <= 1)
                .collect(),
            features: vec![],
        }),
    ));
    // 7-9: left out: expired snow cover, a research-only layer, a zero-confidence layer.
    ls.push((
        "expired_snow",
        finish(Layer {
            m: manifest(Kind::Hazard, vec![5], "snow-cover", "1.0", 1_756_684_800, 1_756_684_800, 1_759_276_800, 1_759_276_800),
            blocks: world.grid(1, |_, _, _, _, v, wt| {
                v[0] = 255;
                *wt = 255;
                true
            })[..4]
                .to_vec(),
            features: vec![],
        }),
    ));
    let mut research = Layer {
        m: manifest(Kind::Class, vec![], "landcover-ai-research", "0.1", d_obs, d_obs, year, year),
        blocks: world.grid(NCLASS, |i, j, c, _, v, wt| {
            if c == 0 || (i - di).abs() > 40 || (j - dj).abs() > 40 {
                return false;
            }
            v[13] = 255;
            *wt = 255;
            true
        }),
        features: vec![],
    };
    research.m.licence = 1;
    ls.push(("research_landcover", finish(research)));
    ls.push((
        "zero_confidence",
        finish(Layer {
            m: manifest(Kind::Class, vec![], "m1-dynamicworld", "1.0-zero", s2_obs, s2_obs, year, year),
            blocks: world.grid(NCLASS, |_, _, c, _, v, wt| {
                v[1] = 255;
                *wt = 0;
                c != 0
            }),
            features: vec![],
        }),
    ));
    // Write, read back, hash once.
    let mut hashes = Vec::new();
    let mut layer_json = Vec::new();
    for (name, l) in &ls {
        let buf = l.write(None);
        std::fs::write(format!("{out}/{name}.wtl"), &buf).unwrap();
        let back = Layer::read(&buf).unwrap();
        let hh = back.content_hash();
        assert_eq!(hh, l.content_hash());
        hashes.push(hh);
        layer_json.push(format!("\"{name}\": {{\"bytes\": {}, \"hash\": \"{}\"}}", buf.len(), &hex(&hh)[..16]));
    }

    // Team overlay: a block on the trail 40 posts east of the incident point, and a "walked, dry"
    // clear box on the flooded river nearest the incident point.
    let tb = nearest(ipp_i, ipp_j + 40, &on(PATH | PATH_MOUNTAIN | PATH_DEMANDING));
    let (ti, tj) = ij(tb);
    let fl = nearest(ipp_i, ipp_j, &|k| rd[k] == 0);
    let (fi, fj) = ij(fl);
    let block_item = OverlayItem { lat0: (ti - 1) * SP, lat1: (ti + 2) * SP, lon0: (tj - 1) * SP, lon1: (tj + 2) * SP, effect: Effect::Block };
    let clear_item = OverlayItem { lat0: (fi - 6) * SP, lat1: (fi + 6) * SP, lon0: (fj - 6) * SP, lon1: (fj + 6) * SP, effect: Effect::Clear };

    // ---- the timeline -------------------------------------------------------------------------
    let steps = vec![
        Step { name: "base", t: T0, layers: vec![], overlay: vec![] },
        Step { name: "s2", t: T0, layers: vec![0, 7, 8, 9], overlay: vec![] },
        Step { name: "flood", t: T0, layers: vec![0, 7, 8, 9, 1, 2], overlay: vec![] },
        Step { name: "drone", t: T0 + 3 * E, layers: vec![0, 7, 8, 9, 1, 2, 3, 4, 5], overlay: vec![] },
        Step { name: "late", t: T0 + 4 * E, layers: vec![0, 7, 8, 9, 1, 2, 3, 4, 5, 6], overlay: vec![] },
        Step { name: "team", t: T0 + 5 * E, layers: vec![0, 7, 8, 9, 1, 2, 3, 4, 5, 6], overlay: vec![block_item, clear_item] },
        Step { name: "fading", t: T0 + 2 * day + 3 * H, layers: vec![0, 7, 8, 9, 1, 2, 3, 4, 5, 6], overlay: vec![block_item, clear_item] },
        Step { name: "expired", t: T0 + 6 * day - 3 * H, layers: vec![0, 7, 8, 9, 1, 2, 3, 4, 5, 6], overlay: vec![block_item, clear_item] },
    ];
    let mk = |st: &Step, layers: &[usize]| Inputs {
        incident: *b"E14-tatra-test\0\0",
        base: &base,
        ter: &ter,
        layers: layers.iter().map(|&k| (&ls[k].1, hashes[k])).collect(),
        overlay: st.overlay.clone(),
        rules: &rules,
        epoch_s: E,
        spacing: SP,
        allow_research: false,
        feature_min_conf: 128,
        level_min_w: 128,
    };

    let mut checks: Vec<String> = Vec::new();
    let mut hl: Vec<String> = Vec::new();
    let mut step_json = Vec::new();
    let mut prev: Option<(Surface, Vec<u8>)> = None; // surface as read from its file, and the file
    let mut prev_step: Option<&Step> = None;
    let mut surfaces: Vec<Surface> = Vec::new();
    for (sn, st) in steps.iter().enumerate() {
        let inp = mk(st, &st.layers);
        let t1 = Instant::now();
        let full = inp.surface(st.t, None, None);
        let t_full = ms(t1);
        let t1t = Instant::now();
        let full1 = in_pool(1, || inp.surface(st.t, None, None));
        let t_full_1 = ms(t1t);
        assert_eq!(full1.content_hash(), full.content_hash());
        // What changed since the previous step: new or removed layers' blocks, overlay items'
        // blocks, and the blocks of layers whose activity or weight can differ between the two T.
        let mut changed = BTreeSet::new();
        let mut inc_ms = None;
        if let (Some(ps), Some((psurf, _))) = (prev_step, &prev) {
            let a: BTreeSet<usize> = ps.layers.iter().copied().collect();
            let b: BTreeSet<usize> = st.layers.iter().copied().collect();
            for &k in a.symmetric_difference(&b) {
                changed.extend(incident::touched_blocks(&ls[k].1, SP));
            }
            if st.t >= psurf.h.same_until {
                for &k in a.intersection(&b) {
                    let m = &ls[k].1.m;
                    let act = |t: i64| m.valid_from <= t && t < m.valid_until;
                    let fading = |t: i64| m.fade_from < t && t < m.valid_until;
                    if act(ps.t) != act(st.t) || fading(ps.t) || fading(st.t) {
                        changed.extend(incident::touched_blocks(&ls[k].1, SP));
                    }
                }
            }
            if ps.overlay != st.overlay {
                for it in ps.overlay.iter().chain(&st.overlay) {
                    changed.extend(incident::overlay_blocks(it, SP));
                }
            }
            let only: Vec<BlockId> = changed.iter().copied().filter(|b| blocks.binary_search(b).is_ok()).collect();
            let t2 = Instant::now();
            let inc = in_pool(1, || inp.surface(st.t, Some(psurf), Some(&only)));
            inc_ms = Some(ms(t2));
            checks.push(format!("\"incremental_equals_full_{}\": {}", st.name, inc.content_hash() == full.content_hash() && inc.recipe_hash() == full.recipe_hash()));
        }
        // File round trip and delta.
        let t3 = Instant::now();
        let file = full.write();
        let t_write = ms(t3);
        let t4 = Instant::now();
        let back = Surface::read(&file).unwrap();
        let t_read = ms(t4);
        checks.push(format!("\"file_roundtrip_{}\": {}", st.name, back.content_hash() == full.content_hash() && back.write() == file));
        std::fs::write(format!("{out}/step{sn}_{}.wis", st.name), &file).unwrap();
        let mut dj = String::new();
        if let Some((psurf, _)) = &prev {
            let d = incident::delta(psurf, &full);
            std::fs::write(format!("{out}/step{sn}_{}.wisd", st.name), &d).unwrap();
            let t5 = Instant::now();
            let applied = incident::apply_delta(psurf, &d).unwrap();
            let t_apply = ms(t5);
            checks.push(format!("\"delta_applies_{}\": {}", st.name, applied.content_hash() == full.content_hash() && applied.write() == file));
            let nchg = full.blocks.iter().filter(|(k, (_, b))| psurf.blocks.get(*k).map(|x| x.1.hash) != Some(b.hash)).count();
            dj = format!(
                ", \"delta_bytes\": {}, \"delta_blocks\": {nchg}, \"blocks_refused\": {}, \"apply_delta_ms\": {t_apply}, \"incremental_ms_1t\": {}",
                d.len(),
                changed.len(),
                inc_ms.unwrap()
            );
            if nchg > 0 {
                checks.push(format!("\"changed_within_named_{}\": {}", st.name, full.blocks.iter().all(|(k, (id, b))| psurf.blocks.get(k).map(|x| x.1.hash) == Some(b.hash) || changed.contains(id))));
            }
        }
        // Plane sizes.
        let mut pl = [0usize; 9];
        let mut tables = 0usize;
        let mut zl = 0usize;
        for (_, b) in full.blocks.values() {
            let p = b.payload();
            tables += 1 + 32 * b.table.len();
            let mut pos = 1 + 32 * b.table.len();
            for x in pl.iter_mut() {
                let n = crate::encode::get_varint(&p, &mut pos).unwrap() as usize;
                *x += n + 1;
                zl += (p[pos] == 1) as usize;
                pos += n;
            }
        }
        let names = ["top", "cert", "src", "walker_t", "walker_risk", "walker_why", "vehicle_t", "vehicle_risk", "vehicle_why"];
        let pj: Vec<String> = names.iter().zip(pl).map(|(n, b)| format!("\"{n}\": {b}")).collect();
        let passable = |p: usize| full.blocks.values().map(|(_, b)| b.prof[p].t.iter().filter(|&&t| t > 0).count()).sum::<usize>();
        let header_bytes = file.len() - full.blocks.values().map(|(_, b)| b.payload().len()).sum::<usize>() - 44 * full.blocks.len();
        step_json.push(format!(
            "    {{\"step\": \"{}\", \"t\": {}, \"same_until\": {}, \"applied\": {}, \"left_out\": {:?}, \"file_bytes\": {}, \"header_bytes\": {header_bytes}, \"index_bytes\": {}, \"table_bytes\": {tables}, \"zlib_planes\": {zl}, \"planes\": {{{}}}, \"passable_walker\": {}, \"passable_vehicle\": {}, \"full_ms_4t\": {t_full}, \"full_ms_1t\": {t_full_1}, \"write_ms\": {t_write}, \"read_verify_ms\": {t_read}{dj}}}",
            st.name,
            st.t,
            full.h.same_until,
            full.h.applied.len(),
            full.h.left_out.iter().map(|x| x.1).collect::<Vec<_>>(),
            file.len(),
            44 * full.blocks.len(),
            pj.join(", "),
            passable(0),
            passable(1)
        ));
        hl.push(format!("\"h_content_{}\": \"{}\"", st.name, hex(&full.content_hash())));
        hl.push(format!("\"h_recipe_{}\": \"{}\"", st.name, hex(&full.recipe_hash())));
        if sn == 0 {
            // Without layers the surface is E5's, cell for cell, for both profiles.
            let mut same = true;
            for (p, r) in rules.iter().enumerate() {
                let f = surface::build(&r.p, &eb);
                let d = incident::dense(&full, &eb);
                same &= d.t[p] == f;
            }
            checks.push(format!("\"no_layers_equals_e5\": {same}"));
        }
        prev = Some((back, file));
        prev_step = Some(st);
        surfaces.push(full);
    }

    // Epochs: within [T, same_until) the content is the same; T itself is in the recipe.
    {
        let st = &steps[5];
        let su = surfaces[5].h.same_until;
        let later = mk(st, &st.layers).surface(epoch_of_before(su), None, None);
        checks.push(format!("\"same_until_holds\": {}", later.content_hash() == surfaces[5].content_hash() && later.recipe_hash() != surfaces[5].recipe_hash()));
        let st6 = &steps[6];
        checks.push(format!("\"fading_same_until_is_next_epoch\": {}", surfaces[6].h.same_until == st6.t + E));
        let next = mk(st6, &st6.layers).surface(st6.t + 8 * E, None, None);
        checks.push(format!("\"fading_changes_content\": {}", next.content_hash() != surfaces[6].content_hash()));
        // After expiry the result equals fusion without the flood layers (recipes differ).
        let st7 = &steps[7];
        let no_flood: Vec<usize> = st7.layers.iter().copied().filter(|&k| k != 1 && k != 2).collect();
        let nf = mk(st7, &no_flood).surface(st7.t, None, None);
        checks.push(format!("\"expired_equals_without\": {}", nf.content_hash() == surfaces[7].content_hash() && nf.recipe_hash() != surfaces[7].recipe_hash()));
        // Layer input order does not matter.
        let mut rev = st.layers.clone();
        rev.reverse();
        let r = mk(st, &rev).surface(st.t, None, None);
        checks.push(format!("\"layer_order_free\": {}", r.content_hash() == surfaces[5].content_hash() && r.recipe_hash() == surfaces[5].recipe_hash()));
        // The late layer changes only its own blocks (block-local sources).
        let lb = incident::touched_blocks(&ls[6].1, SP);
        let only_late = surfaces[4].blocks.iter().all(|(k, (id, b))| surfaces[3].blocks[k].1.hash == b.hash || lb.contains(id));
        checks.push(format!("\"late_layer_local\": {only_late}"));
    }

    // ---- routes ---------------------------------------------------------------------------------
    // Targets: four random walker-passable cells 1.5-4 km from the incident point, and two just
    // outside the flood (river distance 5), the nearest such cells to the incident point.
    let mut targets = Vec::new();
    let mut z = 0u64;
    while targets.len() < 4 {
        z += 1;
        let k = (mix(5000 + z) % (h * w) as u64) as usize;
        let (a, b) = ij(k);
        let d2 = ((a - ipp_i) * 31).pow(2) + ((b - ipp_j) * 20).pow(2); // ~ metres at 1" here
        if f0[k] > 0 && (1500 * 1500..4000 * 4000).contains(&d2) {
            targets.push(k);
        }
    }
    let mut fl5: Vec<usize> = (0..(h * w) as usize).filter(|&k| rd[k] == 5 && f0[k] > 0).collect();
    fl5.sort_by_key(|&k| {
        let (a, b) = ij(k);
        ((a - ipp_i).pow(2) + (b - ipp_j).pow(2), k)
    });
    targets.extend(fl5.iter().take(1));
    targets.extend(fl5.iter().rev().take(1));
    let mut route_json = Vec::new();
    let mut astar_same = true;
    for (sn, sf) in surfaces.iter().enumerate() {
        if ![0usize, 3, 5].contains(&sn) {
            continue;
        }
        let dn = incident::dense(sf, &eb);
        let rr = sf.rules(); // the tables embedded in the surface are enough to query it
        assert!(rr.iter().zip(&rules).all(|(a, b)| a.bytes() == b.bytes()));
        let g = surface::Graph::new(&eb, &rr[0].p, &dn.t[0]);
        let mut se = surface::Search::new((h * w) as usize);
        for (ti, &tg) in targets.iter().enumerate() {
            let t6 = Instant::now();
            let c = se.run(&g, ipp, Some(tg), false);
            let q_ms = ms(t6);
            let rj = match c {
                None => "null".to_string(),
                Some(cost) => {
                    let steps = incident::path(&g, &se, ipp, tg);
                    let rc = incident::route_conf(&g, &dn, 0, &steps, cost);
                    let mut se2 = surface::Search::new((h * w) as usize);
                    astar_same &= se2.run(&g, ipp, Some(tg), true) == Some(cost);
                    let ph = blake3::hash(&steps.iter().flat_map(|&(u, m)| [(u as u64).to_le_bytes(), (m as u64).to_le_bytes()]).flatten().collect::<Vec<u8>>());
                    hl.push(format!("\"h_route_{sn}_{ti}\": \"{}\"", ph.to_hex()));
                    format!(
                        "{{\"cost_s\": {:.1}, \"len_m\": {:.0}, \"cost_us\": {}, \"moves\": {}, \"mean_cert\": {}, \"min_cert\": {}, \"layer_permille\": {}, \"team_permille\": {}, \"max_risk\": {}, \"risky_permille\": {}, \"layers\": {:?}, \"at_edge\": {}, \"query_ms\": {q_ms}}}",
                        cost as f64 / 1e6,
                        rc.len_mm as f64 / 1e3,
                        cost,
                        steps.len(),
                        rc.mean_cert,
                        rc.min_cert,
                        rc.layer_permille,
                        rc.team_permille,
                        rc.max_risk,
                        rc.risky_permille,
                        rc.layers,
                        rc.at_edge
                    )
                }
            };
            let (a, b) = ij(tg);
            route_json.push(format!("    {{\"step\": \"{}\", \"target\": {ti}, \"cell\": {tg}, \"i\": {a}, \"j\": {b}, \"walker\": {rj}}}", steps[sn].name));
        }
    }
    checks.push(format!("\"astar_cost_equals_dijkstra\": {astar_same}"));

    // Inputs for the Python checks: the overlay and steps, the E10-format fused sample of the
    // team step, and base_class.u8 for E10's own check.
    let st = &steps[5];
    let inp = mk(st, &st.layers);
    let sample: Vec<BlockId> = {
        let mut v: BTreeSet<BlockId> = BTreeSet::new();
        for k in [3usize, 4, 5, 6] {
            v.extend(incident::touched_blocks(&ls[k].1, SP).into_iter().take(3));
        }
        v.extend(incident::overlay_blocks(&block_item, SP));
        v.extend(incident::overlay_blocks(&clear_item, SP));
        v.extend(blocks.iter().step_by(37).copied());
        v.into_iter().filter(|b| blocks.binary_search(b).is_ok()).collect()
    };
    let prm = fuse::Params { t: st.t, allow_research: false, feature_min_conf: 128, level_min_w: 128, spacing: SP };
    let (fused, rec) = fuse::fuse(&base, &inp.layers, &st.overlay, &prm, Some(&sample));
    let mut bin = Vec::new();
    for (id, fb) in &fused {
        bin.extend_from_slice(&fb.bytes(*id));
    }
    std::fs::write(format!("{out}/fused_sample.bin"), &bin).unwrap();
    std::fs::write(format!("{out}/base_class.u8"), &lc).unwrap();
    std::fs::copy(format!("{prep}/blocks.txt"), format!("{out}/blocks.txt")).unwrap();
    let items: Vec<String> = st
        .overlay
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
    let files: Vec<String> = st.layers.iter().map(|&k| format!("\"{}.wtl\"", ls[k].0)).collect();
    std::fs::write(
        format!("{out}/fusion_inputs.json"),
        format!(
            "{{\"t\": {}, \"spacing\": {SP}, \"allow_research\": false, \"feature_min_conf\": 128, \"level_min_w\": 128, \"layers\": [{}], \"overlay\": [{}], \"sample_blocks\": {}, \"fused_hash\": \"{}\", \"surface\": \"step5_team.wis\", \"ipp\": {ipp}, \"targets\": {:?}}}\n",
            st.t,
            files.join(", "),
            items.join(", "),
            sample.len(),
            hex(&rec.fused),
            targets
        ),
    )
    .unwrap();

    // DEM, if a surface embedded it: E1's Rice-coded MED residuals per block.
    let dem_bytes: usize = (0..blocks.len())
        .map(|k| {
            let v = crate::encode::to_units(&dm[k * BLOCK_POSTS..(k + 1) * BLOCK_POSTS], crate::encode::Units::Decimetre);
            crate::encode::encode_dem_rice(&v, crate::encode::Pred::Med).map_or(0, |b| b.len())
        })
        .sum();

    println!("{{");
    println!("  \"cmd\": \"e14\", \"blocks\": {}, \"posts\": {}, \"threads\": {}, \"epoch_s\": {E},", blocks.len(), blocks.len() * BLOCK_POSTS, rayon::current_num_threads());
    println!("  \"ipp\": [{ipp_i}, {ipp_j}], \"drone_centre\": [{di}, {dj}], \"team_block\": [{ti}, {tj}], \"team_clear\": [{fi}, {fj}], \"dem_rice_bytes\": {dem_bytes},");
    println!("  \"terrain_hash\": \"{}\", \"profile_hashes\": [\"{}\", \"{}\"],", hex(&ter.hash), hex(&rules[0].hash()), hex(&rules[1].hash()));
    println!("  \"layers\": {{{}}},", layer_json.join(", "));
    println!("  \"steps\": [\n{}\n  ],", step_json.join(",\n"));
    println!("  \"routes\": [\n{}\n  ],", route_json.join(",\n"));
    println!("  \"checks\": {{{}}},", checks.join(", "));
    println!("  {}", hl.join(",\n  "));
    println!("}}");
    assert!(checks.iter().all(|c| c.ends_with("true")), "a check failed");
}

/// The last epoch start before `su` (or 4 epochs on, if there is no end).
fn epoch_of_before(su: i64) -> i64 {
    if su == i64::MAX { T0 + 9 * E } else { su - E }
}

//! `terrexp`: lane TERRX experiment E1 driver.
//!
//!   terrexp dem  <dem-dir> <blocks> <spacing-ticks> <out-dir> [--point] [--steep] [--threads N]
//!   terrexp lc   <class-dir> <blocks> <spacing-ticks> <out-dir> [--threads N]
//!   terrexp lines <pbf> <blocks> <spacing-ticks> <out-dir>
//!   terrexp surface <prep-dir> <spacing-ticks> <pace-csv> <out-dir> [--queries N] [--threads N]
//!   terrexp dump <dem-file> <row> <col> <n>
//!   terrexp e10  <base-dir> <dem-dir> <out-dir>   (E10 layer contract, see src/e10.rs)
//!
//! Spacing in ticks of 1/120": 60 = 1/2", 120 = 1", 240 = 2", 360 = 3". Prints one JSON object.

mod dem;
mod e10;
mod e14;
mod encode;
mod fuse;
mod incident;
mod landcover;
mod lattice;
mod layers;
mod mosaic;
mod lines;
mod osm;
mod raster;
mod surface;
mod voxel;

use encode::{Pred, Units};
use lattice::{BLOCK, BLOCK_POSTS, BlockId, Spacing};
use rayon::prelude::*;
use std::io::Write;
use std::time::Instant;

fn window_of(blocks: &[BlockId], s: Spacing) -> raster::Window {
    let bi0 = blocks.iter().map(|b| b.bi).min().unwrap() as i64;
    let bi1 = blocks.iter().map(|b| b.bi).max().unwrap() as i64;
    let bj0 = blocks.iter().map(|b| b.bj).min().unwrap() as i64;
    let bj1 = blocks.iter().map(|b| b.bj).max().unwrap() as i64;
    let m = s.0 + 2 * lattice::TICKS_PER_ARCSEC;
    raster::Window {
        lat_min: s.pos(bi0 * BLOCK as i64) - m,
        lat_max: s.pos((bi1 + 1) * BLOCK as i64) + m,
        lon_min: s.pos(bj0 * BLOCK as i64) - m,
        lon_max: s.pos((bj1 + 1) * BLOCK as i64) + m,
    }
}

/// Container: "WTE1", kind, encoding, spacing u32, block count u32, then (key u64, offset u64)
/// per stored block in key order, then the payloads. Little-endian.
fn write_container(path: &str, kind: u8, enc: u8, s: Spacing, blocks: &[(BlockId, Vec<u8>)]) -> (usize, String) {
    let mut out = Vec::new();
    out.extend_from_slice(b"WTE1");
    out.push(kind);
    out.push(enc);
    out.extend_from_slice(&(s.0 as u32).to_le_bytes());
    out.extend_from_slice(&(blocks.len() as u32).to_le_bytes());
    let mut off = 0u64;
    for (b, p) in blocks {
        out.extend_from_slice(&b.key().to_le_bytes());
        out.extend_from_slice(&off.to_le_bytes());
        off += p.len() as u64;
    }
    for (_, p) in blocks {
        out.extend_from_slice(p);
    }
    std::fs::write(path, &out).unwrap();
    (out.len(), blake3::hash(&out).to_hex().to_string())
}

fn arg_flag(args: &[String], f: &str) -> bool {
    args.iter().any(|a| a == f)
}
fn arg_val(args: &[String], f: &str) -> Option<String> {
    args.iter().position(|a| a == f).and_then(|i| args.get(i + 1).cloned())
}

#[derive(Default)]
struct DemStats {
    posts: u64,
    none: u64,
    stored_blocks: u64,
    partial_blocks: u64,
    min: i64,
    max: i64,
    sz: [u64; 4], // m-delta, m-med, dm-delta, dm-med
    zlib_m_med: u64,
    rice_m: u64,
    rice_dm: u64,
    steep_packed: u64,
    steep_runs: u64,
    steep_zlib: u64,
    steep_hist: [u64; 16],
    steep_none: u64,
    // steepness-max class minus lattice-slope class, interior posts
    diff_hist: [u64; 16],
    hidden30: u64, // steep-max > 30 deg while the lattice slope says <= 30 deg
    max30: u64,
    lat30: u64,
    compared: u64,
}

fn cmd_dem(args: &[String]) {
    let (dir, bfile, st, out) = (&args[0], &args[1], args[2].parse::<i64>().unwrap(), &args[3]);
    let s = Spacing::new(st).unwrap();
    let mode = if arg_flag(args, "--point") { dem::Mode::Point } else { dem::Mode::Area };
    let steep = arg_flag(args, "--steep");
    let blocks = lattice::read_blocks(bfile).unwrap();
    std::fs::create_dir_all(out).unwrap();
    let t0 = Instant::now();
    let rasters = raster::read_dem_dir(dir, Some(window_of(&blocks, s))).unwrap();
    let names: Vec<String> = rasters.iter().map(|r| r.name.clone()).collect();
    let m = mosaic::Mosaic::new(rasters);
    let (shared, shared_bad) = m.shared_edge_mismatches();
    let t_read = t0.elapsed().as_secs_f64();
    let t1 = Instant::now();
    let cl = dem::SlopeClassifier::new();
    struct Res {
        b: BlockId,
        dm: Vec<i32>,
        enc: [Option<Vec<u8>>; 4],
        rice: [Option<Vec<u8>>; 2],
        steep: Option<Vec<u8>>,
        lslope: Option<Vec<u8>>,
    }
    let res: Vec<Res> = blocks
        .par_iter()
        .map(|&b| {
            let dm = dem::dem_block(&m, s, b, mode);
            let vm = encode::to_units(&dm, Units::Metre);
            let vd = encode::to_units(&dm, Units::Decimetre);
            let enc = [
                encode::encode_dem(&vm, Pred::Delta),
                encode::encode_dem(&vm, Pred::Med),
                encode::encode_dem(&vd, Pred::Delta),
                encode::encode_dem(&vd, Pred::Med),
            ];
            if let Some(e) = &enc[1] {
                assert_eq!(encode::decode_dem(e, Pred::Med).as_deref(), Some(&vm[..]), "roundtrip {b:?}");
            }
            let rice = [encode::encode_dem_rice(&vm, Pred::Med), encode::encode_dem_rice(&vd, Pred::Med)];
            if let Some(e) = &rice[0] {
                assert_eq!(encode::decode_dem_rice(e, Pred::Med).as_deref(), Some(&vm[..]), "rice roundtrip {b:?}");
            }
            let (steep, lslope) = if steep {
                (Some(dem::steep_block(&m, s, b, &cl)), Some(dem::lattice_slope_block(&dm, s, b, &cl)))
            } else {
                (None, None)
            };
            Res { b, dm, enc, rice, steep, lslope }
        })
        .collect();
    let t_resample = t1.elapsed().as_secs_f64();
    let mut st_ = DemStats { min: i64::MAX, max: i64::MIN, ..Default::default() };
    let mut med_blocks = Vec::new();
    let mut rice_blocks = Vec::new();
    let mut steep_blocks = Vec::new();
    let mut raw = std::io::BufWriter::new(std::fs::File::create(format!("{out}/posts_dm.i32")).unwrap());
    let mut bl = std::io::BufWriter::new(std::fs::File::create(format!("{out}/blocks.txt")).unwrap());
    let mut steep_raw = steep.then(|| std::io::BufWriter::new(std::fs::File::create(format!("{out}/posts_steep.u8")).unwrap()));
    for r in &res {
        st_.posts += BLOCK_POSTS as u64;
        let nn = r.dm.iter().filter(|&&v| v == raster::DEM_NONE).count() as u64;
        st_.none += nn;
        for &v in &r.dm {
            if v != raster::DEM_NONE {
                st_.min = st_.min.min(v as i64);
                st_.max = st_.max.max(v as i64);
            }
        }
        if let Some(e) = &r.enc[1] {
            st_.stored_blocks += 1;
            if nn > 0 {
                st_.partial_blocks += 1;
            }
            st_.zlib_m_med += encode::zlib_len(e) as u64;
            med_blocks.push((r.b, e.clone()));
        }
        st_.rice_m += r.rice[0].as_ref().map_or(0, |e| e.len() as u64);
        st_.rice_dm += r.rice[1].as_ref().map_or(0, |e| e.len() as u64);
        if let Some(e) = &r.rice[0] {
            rice_blocks.push((r.b, e.clone()));
        }
        for k in 0..4 {
            st_.sz[k] += r.enc[k].as_ref().map_or(0, |e| e.len() as u64);
        }
        writeln!(bl, "{} {}", r.b.bi, r.b.bj).unwrap();
        for &v in &r.dm {
            raw.write_all(&v.to_le_bytes()).unwrap();
        }
        if let (Some(sv), Some(ls)) = (&r.steep, &r.lslope) {
            steep_raw.as_mut().unwrap().write_all(sv).unwrap(); // 0xFF = no valid source pixel
            let mut v = sv.clone();
            for x in v.iter_mut() {
                if *x == 0xFF {
                    st_.steep_none += 1;
                    *x = 0;
                } else {
                    st_.steep_hist[*x as usize] += 1;
                }
            }
            let packed = encode::packed_nibbles(&v);
            let runs = encode::runs_nibbles(&v);
            assert_eq!(encode::decode_runs(&runs).as_deref(), Some(&v[..]));
            st_.steep_packed += packed.len() as u64;
            st_.steep_runs += runs.len() as u64;
            st_.steep_zlib += encode::zlib_len(&packed) as u64;
            steep_blocks.push((r.b, runs));
            for p in 0..BLOCK_POSTS {
                if sv[p] != 0xFF && ls[p] != 0xFF {
                    st_.compared += 1;
                    let d = sv[p] as i32 - ls[p] as i32;
                    st_.diff_hist[d.clamp(0, 15) as usize] += 1;
                    if sv[p] >= 8 {
                        st_.max30 += 1;
                    }
                    if ls[p] >= 8 {
                        st_.lat30 += 1;
                    }
                    if sv[p] >= 8 && ls[p] < 8 {
                        st_.hidden30 += 1;
                    }
                }
            }
        }
    }
    raw.flush().unwrap();
    let (fsize, fhash) = write_container(&format!("{out}/dem_m_med.wte"), 1, 1, s, &med_blocks);
    let (rsize, rhash) = write_container(&format!("{out}/dem_m_rice.wte"), 1, 3, s, &rice_blocks);
    let steep_file = if steep { Some(write_container(&format!("{out}/steep_runs.wte"), 2, 2, s, &steep_blocks)) } else { None };
    let mis = m.misaligned.load(std::sync::atomic::Ordering::Relaxed);
    let total = t0.elapsed().as_secs_f64();
    println!("{{");
    println!("  \"cmd\": \"dem\", \"dir\": \"{dir}\", \"blocks_file\": \"{bfile}\", \"spacing_ticks\": {st}, \"mode\": \"{mode:?}\",");
    println!("  \"tiles\": {names:?},");
    println!("  \"shared_edge_posts\": {shared}, \"shared_edge_mismatch\": {shared_bad}, \"missing_corners\": {mis},");
    println!("  \"blocks\": {}, \"stored_blocks\": {}, \"partial_blocks\": {}, \"posts\": {}, \"none_posts\": {},", blocks.len(), st_.stored_blocks, st_.partial_blocks, st_.posts, st_.none);
    println!("  \"min_dm\": {}, \"max_dm\": {},", st_.min, st_.max);
    println!("  \"raw_i16_bytes\": {},", st_.stored_blocks * 8192);
    println!("  \"index_bytes\": {},", st_.stored_blocks * 16);
    println!("  \"m_delta_bytes\": {}, \"m_med_bytes\": {}, \"dm_delta_bytes\": {}, \"dm_med_bytes\": {}, \"m_med_zlib_bytes\": {},", st_.sz[0], st_.sz[1], st_.sz[2], st_.sz[3], st_.zlib_m_med);
    println!("  \"file_m_med\": {{\"bytes\": {fsize}, \"blake3\": \"{fhash}\"}},");
    println!("  \"m_rice_bytes\": {}, \"dm_rice_bytes\": {}, \"file_m_rice\": {{\"bytes\": {rsize}, \"blake3\": \"{rhash}\"}},", st_.rice_m, st_.rice_dm);
    if let Some((ssz, sh)) = steep_file {
        println!("  \"steep_packed_bytes\": {}, \"steep_runs_bytes\": {}, \"steep_zlib_bytes\": {}, \"steep_none\": {},", st_.steep_packed, st_.steep_runs, st_.steep_zlib, st_.steep_none);
        println!("  \"steep_hist\": {:?},", st_.steep_hist);
        println!("  \"steep_minus_lattice_hist\": {:?}, \"compared\": {}, \"max_over30\": {}, \"lattice_over30\": {}, \"hidden_over30\": {},", st_.diff_hist, st_.compared, st_.max30, st_.lat30, st_.hidden30);
        println!("  \"file_steep_runs\": {{\"bytes\": {ssz}, \"blake3\": \"{sh}\"}},");
    }
    println!("  \"threads\": {}, \"t_read_s\": {t_read:.2}, \"t_resample_s\": {t_resample:.2}, \"t_total_s\": {total:.2}", rayon::current_num_threads());
    println!("}}");
}

fn cmd_lc(args: &[String]) {
    let (dir, bfile, st, out) = (&args[0], &args[1], args[2].parse::<i64>().unwrap(), &args[3]);
    let s = Spacing::new(st).unwrap();
    let blocks = lattice::read_blocks(bfile).unwrap();
    std::fs::create_dir_all(out).unwrap();
    let t0 = Instant::now();
    let rasters = raster::read_class_dir(dir, Some(window_of(&blocks, s))).unwrap();
    let names: Vec<String> = rasters.iter().map(|r| r.name.clone()).collect();
    let m = mosaic::Mosaic::new(rasters);
    let t_read = t0.elapsed().as_secs_f64();
    let t1 = Instant::now();
    let res: Vec<(BlockId, landcover::LcBlock)> = blocks.par_iter().map(|&b| (b, landcover::lc_block(&m, s, b))).collect();
    let t_res = t1.elapsed().as_secs_f64();
    let (mut packed, mut runs, mut zp, mut zr) = (0u64, 0u64, 0u64, 0u64);
    let mut hist = [0u64; 12];
    let mut purity = [0u64; 4]; // 1.0, [0.75,1), [0.5,0.75), <0.5
    let mut pix = 0u64;
    let mut stored = Vec::new();
    let mut raw = std::io::BufWriter::new(std::fs::File::create(format!("{out}/posts_class.u8")).unwrap());
    // E3: purity per post as two u8 (majority count, valid pixels); at most 9 x 9 pixels at 3".
    let mut pur = std::io::BufWriter::new(std::fs::File::create(format!("{out}/posts_purity.u8")).unwrap());
    let mut bl = std::io::BufWriter::new(std::fs::File::create(format!("{out}/blocks.txt")).unwrap());
    for (b, r) in &res {
        writeln!(bl, "{} {}", b.bi, b.bj).unwrap();
        raw.write_all(&r.class).unwrap();
        for p in 0..BLOCK_POSTS {
            pur.write_all(&[r.maj[p].min(255) as u8, r.total[p].min(255) as u8]).unwrap();
        }
        for p in 0..BLOCK_POSTS {
            hist[r.class[p] as usize] += 1;
            pix += r.total[p] as u64;
            if r.total[p] > 0 {
                let (mj, t) = (r.maj[p] as u64, r.total[p] as u64);
                purity[if mj == t { 0 } else if 4 * mj >= 3 * t { 1 } else if 2 * mj >= t { 2 } else { 3 }] += 1;
            }
        }
        if r.class.iter().all(|&c| c == 0) {
            continue;
        }
        let pk = encode::packed_nibbles(&r.class);
        let rn = encode::runs_nibbles(&r.class);
        assert_eq!(encode::decode_runs(&rn).as_deref(), Some(&r.class[..]));
        packed += pk.len() as u64;
        runs += rn.len() as u64;
        zp += encode::zlib_len(&pk) as u64;
        zr += encode::zlib_len(&rn) as u64;
        stored.push((*b, rn));
    }
    raw.flush().unwrap();
    pur.flush().unwrap();
    let (fsz, fh) = write_container(&format!("{out}/lc_runs.wte"), 3, 2, s, &stored);
    println!("{{");
    println!("  \"cmd\": \"lc\", \"dir\": \"{dir}\", \"blocks_file\": \"{bfile}\", \"spacing_ticks\": {st}, \"tiles\": {names:?},");
    println!("  \"blocks\": {}, \"stored_blocks\": {}, \"source_pixels\": {pix},", blocks.len(), stored.len());
    println!("  \"class_hist\": {{{}}},", landcover::CLASS_NAMES.iter().zip(hist.iter()).map(|(n, c)| format!("\"{n}\": {c}")).collect::<Vec<_>>().join(", "));
    println!("  \"purity_hist\": {{\"pure\": {}, \"ge75\": {}, \"ge50\": {}, \"lt50\": {}}},", purity[0], purity[1], purity[2], purity[3]);
    println!("  \"packed_bytes\": {packed}, \"runs_bytes\": {runs}, \"packed_zlib_bytes\": {zp}, \"runs_zlib_bytes\": {zr}, \"index_bytes\": {},", stored.len() * 16);
    println!("  \"file_runs\": {{\"bytes\": {fsz}, \"blake3\": \"{fh}\"}},");
    println!("  \"threads\": {}, \"t_read_s\": {t_read:.2}, \"t_resample_s\": {t_res:.2}, \"t_total_s\": {:.2}", rayon::current_num_threads(), t0.elapsed().as_secs_f64());
    println!("}}");
}

/// E3: OSM land-cover areas onto the lattice. Writes `posts_fine.u8` (fine OSM class per
/// post), `posts_cov.u8` (covered samples 0-16) and `blocks.txt`, in block-list order.
fn cmd_osm(args: &[String]) {
    let (pbf, bfile, st, out) = (&args[0], &args[1], args[2].parse::<i64>().unwrap(), &args[3]);
    let s = Spacing::new(st).unwrap();
    let blocks = lattice::read_blocks(bfile).unwrap();
    std::fs::create_dir_all(out).unwrap();
    let t0 = Instant::now();
    let (res, stats) = osm::rasterize(pbf, s, &blocks);
    let mut fine = std::io::BufWriter::new(std::fs::File::create(format!("{out}/posts_fine.u8")).unwrap());
    let mut cov = std::io::BufWriter::new(std::fs::File::create(format!("{out}/posts_cov.u8")).unwrap());
    let mut bl = std::io::BufWriter::new(std::fs::File::create(format!("{out}/blocks.txt")).unwrap());
    let mut hist = [0u64; 32];
    let mut covh = [0u64; 17];
    let mut h = blake3::Hasher::new();
    for (b, r) in &res {
        writeln!(bl, "{} {}", b.bi, b.bj).unwrap();
        fine.write_all(&r.class).unwrap();
        cov.write_all(&r.cov).unwrap();
        h.update(&r.class);
        h.update(&r.cov);
        for p in 0..BLOCK_POSTS {
            hist[r.class[p] as usize] += 1;
            covh[r.cov[p] as usize] += 1;
        }
    }
    println!("{{");
    println!("  \"cmd\": \"osm\", \"pbf\": \"{pbf}\", \"blocks_file\": \"{bfile}\", \"spacing_ticks\": {st}, \"blocks\": {},", res.len());
    println!("  \"stats\": \"{stats:?}\",");
    println!("  \"fine_hist\": {{{}}},", osm::FINE_NAMES.iter().zip(hist.iter()).map(|(n, c)| format!("\"{n}\": {c}")).collect::<Vec<_>>().join(", "));
    println!("  \"cov_hist\": {covh:?},");
    println!("  \"blake3\": \"{}\",", h.finalize().to_hex());
    println!("  \"threads\": {}, \"t_total_s\": {:.2}", rayon::current_num_threads(), t0.elapsed().as_secs_f64());
    println!("}}");
}

fn cmd_lines(args: &[String]) {
    let (pbf, bfile, st, out) = (&args[0], &args[1], args[2].parse::<i64>().unwrap(), &args[3]);
    let s = Spacing::new(st).unwrap();
    let blocks = lattice::read_blocks(bfile).unwrap();
    std::fs::create_dir_all(out).unwrap();
    let t0 = Instant::now();
    let (res, stats) = lines::rasterize(pbf, s, &blocks);
    let mut f = std::io::BufWriter::new(std::fs::File::create(format!("{out}/posts_lines.u16")).unwrap());
    let mut h = blake3::Hasher::new();
    let mut bits = [0u64; 13];
    for (_, v) in &res {
        for &x in v {
            f.write_all(&x.to_le_bytes()).unwrap();
            h.update(&x.to_le_bytes());
            for (k, c) in bits.iter_mut().enumerate() {
                *c += (x >> k & 1) as u64;
            }
        }
    }
    println!("{{");
    println!("  \"cmd\": \"lines\", \"pbf\": \"{pbf}\", \"blocks_file\": \"{bfile}\", \"spacing_ticks\": {st}, \"blocks\": {},", res.len());
    println!("  \"stats\": \"{stats:?}\",");
    println!("  \"cells_per_bit\": {{{}}},", lines::BIT_NAMES.iter().zip(bits.iter()).map(|(n, c)| format!("\"{n}\": {c}")).collect::<Vec<_>>().join(", "));
    println!("  \"blake3\": \"{}\",", h.finalize().to_hex());
    println!("  \"threads\": {}, \"t_total_s\": {:.2}", rayon::current_num_threads(), t0.elapsed().as_secs_f64());
    println!("}}");
}

fn peak_rss_kb() -> u64 {
    std::fs::read_to_string("/proc/self/status")
        .ok()
        .and_then(|s| s.lines().find(|l| l.starts_with("VmHWM:")).and_then(|l| l.split_whitespace().nth(1)?.parse().ok()))
        .unwrap_or(0)
}

fn read_vec<T: Copy>(path: &str, from: fn(&[u8]) -> T, size: usize) -> Vec<T> {
    let b = std::fs::read(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    b.chunks_exact(size).map(from).collect()
}

/// E5: build the incident surface for both profiles from a prep dir (blocks.txt, dem_dm.i32,
/// steep.u8, lc4.u8, lines.u16, all in block order), store it, and time searches on it.
fn cmd_surface(args: &[String]) {
    let (prep, st, csv, out) = (&args[0], args[1].parse::<i64>().unwrap(), &args[2], &args[3]);
    let nq: usize = arg_val(args, "--queries").map_or(200, |v| v.parse().unwrap());
    let s = Spacing::new(st).unwrap();
    std::fs::create_dir_all(out).unwrap();
    let blocks = lattice::read_blocks(&format!("{prep}/blocks.txt")).unwrap();
    let dm = read_vec(&format!("{prep}/dem_dm.i32"), |b| i32::from_le_bytes(b.try_into().unwrap()), 4);
    let steep = std::fs::read(format!("{prep}/steep.u8")).unwrap();
    let lc = std::fs::read(format!("{prep}/lc4.u8")).unwrap();
    let ln = read_vec(&format!("{prep}/lines.u16"), |b| u16::from_le_bytes(b.try_into().unwrap()), 2);
    let n_in = blocks.len() * BLOCK_POSTS;
    assert!(dm.len() == n_in && steep.len() == n_in && lc.len() == n_in && ln.len() == n_in);
    let mut in_h = blake3::Hasher::new();
    for f in ["dem_dm.i32", "steep.u8", "lc4.u8", "lines.u16"] {
        in_h.update(&std::fs::read(format!("{prep}/{f}")).unwrap());
    }
    let base = surface::Base::from_blocks(s, &blocks, &dm, &steep, &lc, &ln);
    drop((dm, steep, lc, ln));
    // E4 expected-pace curve: speed per mille of flat by whole grade percent
    let text = std::fs::read_to_string(csv).unwrap();
    let hdr: Vec<&str> = text.lines().next().unwrap().split(',').collect();
    let col = hdr.iter().position(|&h| h == "mean_pace_permille").unwrap();
    let grade_rel: Vec<u32> = text.lines().skip(1).map(|l| l.split(',').nth(col).unwrap().parse().unwrap()).collect();
    assert_eq!(grade_rel.len(), 201);
    let profiles = [surface::walker(grade_rel), surface::vehicle()];
    let n = base.h * base.w;
    println!("{{");
    println!("  \"cmd\": \"surface\", \"prep\": \"{prep}\", \"spacing_ticks\": {st}, \"rows\": {}, \"cols\": {}, \"cells\": {n}, \"blocks\": {},", base.h, base.w, blocks.len());
    println!("  \"inputs_blake3\": \"{}\", \"threads\": {},", in_h.finalize().to_hex(), rayon::current_num_threads());
    // IPP: the passable cell nearest the centre, for both profiles
    let mut first = true;
    for p in &profiles {
        let t0 = Instant::now();
        let f = surface::build(p, &base);
        let t_build = t0.elapsed().as_secs_f64();
        let t1 = Instant::now();
        let enc = surface::encode_palette_runs(&base, &f);
        let t_enc = t1.elapsed().as_secs_f64();
        let (bytes, hash) = write_container(&format!("{out}/surface_{}.wte", p.name), 6, 7, s, &enc);
        let t2 = Instant::now();
        let mut dec = vec![0u16; n];
        for (b, buf) in &enc {
            let v = surface::decode_palette_runs(buf).unwrap();
            let (r0, c0) = ((b.bi as i64 - base.bi0) as usize * BLOCK, (b.bj as i64 - base.bj0) as usize * BLOCK);
            for r in 0..BLOCK {
                dec[(r0 + r) * base.w + c0..(r0 + r) * base.w + c0 + BLOCK].copy_from_slice(&v[r * BLOCK..(r + 1) * BLOCK]);
            }
        }
        let t_dec = t2.elapsed().as_secs_f64();
        assert!(dec == f, "surface decode mismatch");
        let raw_le: Vec<u8> = f.iter().flat_map(|v| v.to_le_bytes()).collect();
        let zl = encode::zlib_len(&raw_le);
        let passable = f.iter().filter(|&&v| v > 0).count();
        let mut tv: Vec<u16> = f.iter().copied().filter(|&v| v > 0).collect();
        tv.sort_unstable();

        let g = surface::Graph::new(&base, p, &f);
        // IPP: the cell nearest the centre that is passable (walker) or on a usable way (vehicle,
        // which starts from where it parked)
        let on_way = |i: usize| (0..8).any(|k| base.lines[i] & (1 << k) != 0 && p.way_t[k] > 0);
        let (cr, cc) = (base.h / 2, base.w / 2);
        let mut ipp = cr * base.w + cc;
        'find: for rad in 0..base.h / 2 {
            for dr in -(rad as i64)..=rad as i64 {
                for dc in -(rad as i64)..=rad as i64 {
                    let i = (cr as i64 + dr) as usize * base.w + (cc as i64 + dc) as usize;
                    if f[i] > 0 && (p.name == "walker" || on_way(i)) {
                        ipp = i;
                        break 'find;
                    }
                }
            }
        }
        let mut se = surface::Search::new(n);
        let mut tf = Vec::new();
        for _ in 0..3 {
            let t3 = Instant::now();
            se.run(&g, ipp, None, false);
            tf.push(t3.elapsed().as_secs_f64());
        }
        tf.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let (t_field, t_field_max) = (tf[0], tf[2]);
        let (field_settled, field_pushes) = (se.settled, se.pushes);
        let field = se.field_ds(n);
        let reach = field.iter().filter(|&&v| v != u32::MAX).count();
        let mut fr: Vec<u32> = field.iter().copied().filter(|&v| v != u32::MAX).collect();
        fr.sort_unstable();
        let field_bytes: Vec<u8> = field.iter().flat_map(|v| v.to_le_bytes()).collect();
        std::fs::write(format!("{out}/field_{}.u32", p.name), &field_bytes).unwrap();
        let field_hash = blake3::hash(&field_bytes).to_hex().to_string();
        let within = |secs: u32| field.iter().filter(|&&v| v <= secs * 10).count();

        // access-leg queries: random passable pairs 1-3 km apart
        let mut rng = 0x9E37_79B9_7F4A_7C15u64 ^ st as u64;
        let mut next = || {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            rng
        };
        let dy = g.dy_mm as f64 / 1000.0;
        let dx = g.dx_min_mm as f64 / 1000.0;
        let mut q = Vec::new();
        while q.len() < nq {
            let a = (next() % n as u64) as usize;
            if f[a] == 0 {
                continue;
            }
            let d = 1000.0 + (next() % 2001) as f64;
            let th = (next() % 3600) as f64 / 3600.0 * 2.0 * std::f64::consts::PI;
            let (r, c) = ((a / base.w) as f64 + d * th.cos() / dy, (a % base.w) as f64 + d * th.sin() / dx);
            if r < 0.0 || c < 0.0 || r >= base.h as f64 || c >= base.w as f64 {
                continue;
            }
            let z = r as usize * base.w + c as usize;
            if f[z] > 0 {
                q.push((a, z));
            }
        }
        let (mut td, mut ta, mut sd, mut sa) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        let (mut mismatch, mut unreachable) = (0, 0);
        let mut qh = blake3::Hasher::new();
        for &(a, z) in &q {
            let t = Instant::now();
            let d1 = se.run(&g, a, Some(z), false);
            td.push(t.elapsed().as_secs_f64() * 1e3);
            sd.push(se.settled);
            let t = Instant::now();
            let d2 = se.run(&g, a, Some(z), true);
            ta.push(t.elapsed().as_secs_f64() * 1e3);
            sa.push(se.settled);
            if d1 != d2 {
                mismatch += 1;
            }
            if d1.is_none() {
                unreachable += 1;
            }
            qh.update(&d1.unwrap_or(u64::MAX).to_le_bytes());
        }
        let pct = |v: &mut Vec<f64>, p: f64| {
            v.sort_by(|a, b| a.partial_cmp(b).unwrap());
            v[((v.len() - 1) as f64 * p).round() as usize]
        };
        let pctu = |v: &mut Vec<u64>, p: f64| {
            v.sort_unstable();
            v[((v.len() - 1) as f64 * p).round() as usize]
        };
        if !first {
            println!(",");
        }
        first = false;
        println!("  \"{}\": {{", p.name);
        println!("    \"t_build_s\": {t_build:.4}, \"t_encode_s\": {t_enc:.4}, \"t_decode_s\": {t_dec:.4},");
        println!("    \"surface_bytes\": {bytes}, \"surface_blake3\": \"{hash}\", \"raw_u16_bytes\": {}, \"zlib_raw_bytes\": {zl},", raw_le.len());
        println!("    \"passable_share\": {:.4}, \"t_unit_us\": {}, \"t_median\": {}, \"t_p10\": {}, \"t_p90\": {}, \"distinct_t\": {},",
            passable as f64 / n as f64, p.unit_us, tv[tv.len() / 2], tv[tv.len() / 10], tv[tv.len() * 9 / 10],
            { let mut d = tv.clone(); d.dedup(); d.len() });
        println!("    \"ipp\": [{}, {}], \"t_field_s\": {t_field:.4}, \"t_field_max_s\": {t_field_max:.4}, \"field_settled\": {field_settled}, \"field_pushes\": {field_pushes}, \"field_reach_share\": {:.4},",
            ipp / base.w, ipp % base.w, reach as f64 / n as f64);
        println!("    \"field_median_s\": {:.0}, \"field_max_s\": {:.0}, \"cells_within_1h\": {}, \"cells_within_3h\": {}, \"field_blake3\": \"{field_hash}\",",
            fr[fr.len() / 2] as f64 / 10.0, *fr.last().unwrap() as f64 / 10.0, within(3600), within(3 * 3600));
        println!("    \"queries\": {nq}, \"unreachable\": {unreachable}, \"astar_mismatch\": {mismatch}, \"queries_blake3\": \"{}\",", qh.finalize().to_hex());
        println!("    \"dijkstra_ms_p50\": {:.2}, \"dijkstra_ms_p90\": {:.2}, \"dijkstra_settled_p50\": {}, \"dijkstra_settled_p90\": {},",
            pct(&mut td, 0.5), pct(&mut td, 0.9), pctu(&mut sd, 0.5), pctu(&mut sd, 0.9));
        println!("    \"astar_ms_p50\": {:.2}, \"astar_ms_p90\": {:.2}, \"astar_settled_p50\": {}, \"astar_settled_p90\": {}",
            pct(&mut ta, 0.5), pct(&mut ta, 0.9), pctu(&mut sa, 0.5), pctu(&mut sa, 0.9));
        print!("  }}");
    }
    println!(",");
    println!("  \"peak_rss_kb\": {}", peak_rss_kb());
    println!("}}");
}

fn cmd_dump(args: &[String]) {
    let path = &args[0];
    let (r0, c0, n): (usize, usize, usize) = (args[1].parse().unwrap(), args[2].parse().unwrap(), args[3].parse().unwrap());
    let ras = if path.ends_with(".tif") { raster::read_dem_geotiff(path, None).unwrap().unwrap() } else { raster::read_hgt(path).unwrap() };
    println!("{:?}", ras.grid);
    for r in r0..r0 + n {
        let row: Vec<String> = (c0..c0 + n).map(|c| ras.at(r, c).to_string()).collect();
        println!("{}", row.join(" "));
    }
    let h = blake3::hash(&ras.data.iter().flat_map(|v| v.to_le_bytes()).collect::<Vec<u8>>());
    println!("decoded_blake3 {}", h.to_hex());
}

/// Decode every block of a WTE DEM container (enc 1 = varint MED, 3 = Rice MED) and time it.
fn cmd_bench(args: &[String]) {
    let buf = std::fs::read(&args[0]).unwrap();
    assert_eq!(&buf[0..4], b"WTE1");
    let enc = buf[5];
    let n = u32::from_le_bytes(buf[10..14].try_into().unwrap()) as usize;
    let idx = 14;
    let base = idx + 16 * n;
    let off = |k: usize| u64::from_le_bytes(buf[idx + 16 * k + 8..idx + 16 * k + 16].try_into().unwrap()) as usize;
    let t = Instant::now();
    let mut sum = 0i64;
    for k in 0..n {
        let end = if k + 1 < n { off(k + 1) } else { buf.len() - base };
        let b = &buf[base + off(k)..base + end];
        let v = if enc == 3 { encode::decode_dem_rice(b, Pred::Med) } else { encode::decode_dem(b, Pred::Med) }.unwrap();
        sum += v.iter().flatten().sum::<i64>();
    }
    let dt = t.elapsed().as_secs_f64();
    println!("{{\"blocks\": {n}, \"enc\": {enc}, \"decode_s\": {dt:.3}, \"us_per_block\": {:.1}, \"checksum\": {sum}}}", dt * 1e6 / n as f64);
}

fn main() {
    let mut args: Vec<String> = std::env::args().collect();
    if let Some(t) = arg_val(&args, "--threads") {
        rayon::ThreadPoolBuilder::new().num_threads(t.parse().unwrap()).build_global().unwrap();
        let i = args.iter().position(|a| a == "--threads").unwrap();
        args.drain(i..i + 2);
    }
    match args.get(1).map(String::as_str) {
        Some("dem") => cmd_dem(&args[2..]),
        Some("lc") => cmd_lc(&args[2..]),
        Some("osm") => cmd_osm(&args[2..]),
        Some("lines") => cmd_lines(&args[2..]),
        Some("surface") => cmd_surface(&args[2..]),
        Some("dump") => cmd_dump(&args[2..]),
        Some("bench") => cmd_bench(&args[2..]),
        Some("e10") => e10::run(&args[2..]),
        Some("e14") => e14::run(&args[2..]),
        _ => eprintln!("usage: terrexp dem|lc|dump ..."),
    }
}

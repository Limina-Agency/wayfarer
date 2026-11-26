//! E14: the incident surface contract (docs/design/incident-surface.md, draft v0).
//!
//! The incident surface is the stored result of stage 2: E10's fusion of the base with the layers
//! and team overlay active at an incident epoch T, costed per profile by E5's tables plus the
//! hazard, level and effect rules of the layer contract's section 9. A query (stage 3) reads only
//! this surface and the terrain file's DEM.
//!
//! Per block it stores: the fused top class, its certainty, the post's source (block-local index
//! into a table of layer hashes, so a block depends only on its own inputs), and per profile the
//! time per metre (u16, 0 = impassable), the risk that the cell is impassable (u8, 1/255) and why
//! an impassable cell is closed. Every plane is coded with E5's palette-runs.
//!
//! Identity: a block hash over each block's canonical decoded bytes; the content hash over the
//! surface's query-relevant identity (spacing, terrain, profile tables) and the block hashes; the
//! recipe hash over every input (terrain, prior, layers applied and left out, overlay, profiles,
//! params, T). Same content hash = same routes. A delta carries only the changed blocks and is
//! checked by the resulting content hash.

use crate::encode::{get_varint, put_varint};
use crate::fuse::{self, FusedBlock, M_BARRIER, M_DEBRIS, M_FALLEN, M_PASSAGE, M_TEAM_BLOCK, M_WASHOUT, Record, S};
use crate::lattice::{BLOCK, BLOCK_POSTS, BlockId, div_round};
use crate::layers::{Kind, Layer, Manifest, NCLASS, NHAZ, NLEV};
use crate::lines::*;
use crate::surface::{self, Profile};
use rayon::prelude::*;
use std::collections::{BTreeMap, BTreeSet};

pub const MAJOR: u16 = 0;
pub const MINOR: u16 = 0;
pub const MAGIC: &[u8; 4] = b"WIS0";
pub const DELTA_MAGIC: &[u8; 4] = b"WISD";

// Line-mask bits the fused effect bits add to E5's mask (E5 uses bits 0-12).
pub const OBSTRUCTION: u16 = 1 << 13;
pub const WASHOUT: u16 = 1 << 14;
pub const TEAM_BLOCK: u16 = 1 << 15;

/// Why a cell is impassable: reason << 4 | channel.
pub const WHY_CLASS: u8 = 1;
pub const WHY_STEEP: u8 = 2;
pub const WHY_BARRIER: u8 = 3;
pub const WHY_MOTORWAY: u8 = 4;
pub const WHY_HAZARD: u8 = 5;
pub const WHY_LEVEL: u8 = 6;
pub const WHY_OBSTRUCTION: u8 = 7;
pub const WHY_WASHOUT: u8 = 8;
pub const WHY_TEAM: u8 = 9;

/// A blocking hazard is superseded by a measured level: (hazard slot, level slot).
/// Flood by water depth, snow cover by snow depth.
pub const SUPERSEDE: [(usize, usize); 2] = [(0, 0), (4, 1)];

/// Reasons a layer is left out of fusion (stored as codes; E10 records them as text).
pub fn reason_code(why: &str) -> u8 {
    match why {
        "outside its validity window at T" => 1,
        "research-only licence" => 2,
        "spacing differs from the surface (resample at import)" => 3,
        "no confidence anywhere at T" => 4,
        _ => 255,
    }
}

pub fn effect_lines(mask: u16) -> u16 {
    let mut l = 0;
    if mask & M_BARRIER != 0 {
        l |= FENCE;
    }
    if mask & (M_DEBRIS | M_FALLEN) != 0 {
        l |= OBSTRUCTION;
    }
    if mask & M_WASHOUT != 0 {
        l |= WASHOUT;
    }
    if mask & M_PASSAGE != 0 {
        l |= PATH;
    }
    if mask & M_TEAM_BLOCK != 0 {
        l |= TEAM_BLOCK;
    }
    l
}

/// The section 9 additions to an E5 profile.
#[derive(Clone, Debug)]
pub struct Ext {
    /// Class mass (in S) on classes with no time per metre at or above which the cell blocks.
    pub block_at: u32,
    /// Blocking-hazard probability (1/255) at or above which the cell blocks.
    pub haz_block_at: u8,
    /// Bit per hazard slot: the hazard blocks this profile.
    pub haz_block: u8,
    /// Slowdown per hazard slot at probability 1, per mille: time x (1 + h k / 255000).
    pub haz_k: [u32; NHAZ],
    /// Per level slot: block above this value (u16::MAX = never).
    pub lev_block: [u16; NLEV],
    /// Per level slot: slowdown per unit of the level, per mille.
    pub lev_k: [u32; NLEV],
    /// Speed per mille across debris or fallen trees (0 = impassable), also on ways.
    pub obst_rel: u32,
}

pub struct Rules {
    pub p: Profile,
    pub x: Ext,
}

// hazard slots: 0 flood, 1 rubble, 2 windthrow, 3 fire, 4 snow_cover, 5 ice, 6 landslide, 7 closure
/// Walker: flood, fire and closure block; rubble, windthrow and landslide debris slow (E14
/// assumptions, to be calibrated with E5's other walker values in the field trial).
pub fn walker_ext() -> Ext {
    Ext {
        block_at: S / 2,
        haz_block_at: 128,
        haz_block: 1 << 0 | 1 << 3 | 1 << 7,
        haz_k: [500, 1500, 2000, 0, 800, 1000, 1500, 0],
        lev_block: [50, 120, u16::MAX],
        lev_k: [20, 8, 0],
        obst_rel: 400,
    }
}

/// Light 4x4: anything on the ground blocks it except snow cover and ice, which slow it.
pub fn vehicle_ext() -> Ext {
    Ext {
        block_at: S / 4,
        haz_block_at: 64,
        haz_block: 1 << 0 | 1 << 1 | 1 << 2 | 1 << 3 | 1 << 6 | 1 << 7,
        haz_k: [0, 0, 0, 0, 1500, 2000, 0, 0],
        lev_block: [30, 25, u16::MAX],
        lev_k: [20, 30, 0],
        obst_rel: 0,
    }
}

impl Rules {
    /// Canonical bytes of the profile tables (embedded in the surface; the profile hash is their
    /// blake3). All integers little-endian.
    pub fn bytes(&self) -> Vec<u8> {
        let (p, x) = (&self.p, &self.x);
        let mut o = Vec::new();
        o.extend_from_slice(b"WIP0");
        put_varint(&mut o, p.name.len() as u64);
        o.extend_from_slice(p.name.as_bytes());
        o.extend_from_slice(&p.unit_us.to_le_bytes());
        for v in p.cover_t.iter().chain(&p.way_t).chain(&p.steep_rel).chain(&p.barrier_rel) {
            o.extend_from_slice(&v.to_le_bytes());
        }
        o.extend_from_slice(&(p.grade_rel.len() as u16).to_le_bytes());
        for v in &p.grade_rel {
            o.extend_from_slice(&v.to_le_bytes());
        }
        o.extend_from_slice(&x.block_at.to_le_bytes());
        o.push(x.haz_block_at);
        o.push(x.haz_block);
        for v in &x.haz_k {
            o.extend_from_slice(&v.to_le_bytes());
        }
        for v in &x.lev_block {
            o.extend_from_slice(&v.to_le_bytes());
        }
        for v in &x.lev_k {
            o.extend_from_slice(&v.to_le_bytes());
        }
        o.extend_from_slice(&x.obst_rel.to_le_bytes());
        o
    }

    pub fn from_bytes(b: &[u8]) -> Option<Rules> {
        let mut pos = 0;
        let take = |pos: &mut usize, n: usize| -> Option<&[u8]> {
            let s = b.get(*pos..*pos + n)?;
            *pos += n;
            Some(s)
        };
        let u32_ = |pos: &mut usize| -> Option<u32> { Some(u32::from_le_bytes(b.get(*pos..*pos + 4)?.try_into().ok()?)).inspect(|_| *pos += 4) };
        if take(&mut pos, 4)? != b"WIP0" {
            return None;
        }
        let n = get_varint(b, &mut pos)? as usize;
        let name: &'static str = Box::leak(String::from_utf8(take(&mut pos, n)?.to_vec()).ok()?.into_boxed_str());
        let unit_us = u64::from_le_bytes(take(&mut pos, 8)?.try_into().ok()?);
        let arr = |k: usize, pos: &mut usize| -> Option<Vec<u32>> { (0..k).map(|_| u32_(pos)).collect() };
        let cover_t: [u32; 16] = arr(16, &mut pos)?.try_into().ok()?;
        let way_t: [u32; 8] = arr(8, &mut pos)?.try_into().ok()?;
        let steep_rel: [u32; 16] = arr(16, &mut pos)?.try_into().ok()?;
        let barrier_rel: [u32; 5] = arr(5, &mut pos)?.try_into().ok()?;
        let ng = u16::from_le_bytes(take(&mut pos, 2)?.try_into().ok()?) as usize;
        let grade_rel = arr(ng, &mut pos)?;
        let block_at = u32_(&mut pos)?;
        let hb = take(&mut pos, 2)?;
        let (haz_block_at, haz_block) = (hb[0], hb[1]);
        let haz_k: [u32; NHAZ] = arr(NHAZ, &mut pos)?.try_into().ok()?;
        let mut lev_block = [0u16; NLEV];
        for v in lev_block.iter_mut() {
            *v = u16::from_le_bytes(take(&mut pos, 2)?.try_into().ok()?);
        }
        let lev_k: [u32; NLEV] = arr(NLEV, &mut pos)?.try_into().ok()?;
        let obst_rel = u32_(&mut pos)?;
        (pos == b.len()).then_some(Rules {
            p: Profile { name, unit_us, cover_t, way_t, steep_rel, barrier_rel, grade_rel },
            x: Ext { block_at, haz_block_at, haz_block, haz_k, lev_block, lev_k, obst_rel },
        })
    }

    pub fn hash(&self) -> [u8; 32] {
        *blake3::hash(&self.bytes()).as_bytes()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cell {
    pub t: u16,
    pub risk: u8,
    pub why: u8,
}

fn closed(why: u8, sub: usize, risk: u8) -> Cell {
    Cell { t: 0, risk, why: why << 4 | sub as u8 }
}

/// Probability (1/255) that the cell is impassable: 1 - (1 - Pb/S) prod (1 - h/255) over the
/// blocking, non-superseded hazards.
fn risk_of(pb: u32, haz: &[u8; NHAZ], blocking: &[bool; NHAZ]) -> u8 {
    let (mut num, mut den) = ((S - pb) as u128, S as u128);
    for s in 0..NHAZ {
        if blocking[s] {
            num *= 255 - haz[s] as u128;
            den *= 255;
        }
    }
    (255 - div_round(255 * num as i128, den as i128)) as u8
}

/// The cost rules of contract section 5: time per metre, risk and reason for one post.
pub fn cell(r: &Rules, p: &[u16; NCLASS], haz: &[u8; NHAZ], lev: &[(u16, u8); NLEV], mask: u16, steep: u8, lines: u16) -> Cell {
    let (pr, x) = (&r.p, &r.x);
    let l = lines | effect_lines(mask);
    if l & TEAM_BLOCK != 0 {
        return closed(WHY_TEAM, 0, 255);
    }
    if l & WASHOUT != 0 {
        return closed(WHY_WASHOUT, 0, 255);
    }
    let mut blocking = [false; NHAZ];
    for s in 0..NHAZ {
        blocking[s] = x.haz_block >> s & 1 == 1;
    }
    for (h, lv) in SUPERSEDE {
        if lev[lv].1 > 0 {
            blocking[h] = false;
        }
    }
    let ways = l & WAYS;
    let way = (0..8).filter(|k| ways & (1 << k) != 0).map(|k| pr.way_t[k]).filter(|&t| t > 0).min();
    let mut pb = 0u32;
    let mut t: u64;
    if let Some(tw) = way {
        t = tw as u64;
    } else if ways & MOTORWAY != 0 {
        return closed(WHY_MOTORWAY, 0, 255);
    } else {
        let mut acc = 0u64;
        for k in 0..NCLASS {
            let tk = pr.cover_t[k];
            if tk == 0 {
                pb += p[k] as u32;
            } else {
                acc += tk as u64 * p[k] as u64;
            }
        }
        if pb >= x.block_at {
            return closed(WHY_CLASS, 0, risk_of(pb, haz, &blocking));
        }
        t = div_round(acc as i128, (S - pb) as i128) as u64;
        let sr = if steep == 0xFF { 1000 } else { pr.steep_rel[steep as usize & 15] };
        if sr == 0 {
            return closed(WHY_STEEP, 0, 255);
        }
        let mut rel: u64 = 1000 * sr as u64 / 1000;
        for (k, bit) in [RIVER, STREAM, RAIL, FENCE, CLIFF].iter().enumerate() {
            if l & bit != 0 {
                if pr.barrier_rel[k] == 0 {
                    return closed(WHY_BARRIER, k, 255);
                }
                rel = rel * pr.barrier_rel[k] as u64 / 1000;
            }
        }
        t = div_round(t as i128 * 1000, rel.max(1) as i128) as u64;
    }
    for s in 0..NHAZ {
        if blocking[s] && haz[s] >= x.haz_block_at {
            return closed(WHY_HAZARD, s, risk_of(pb, haz, &blocking));
        }
    }
    let hk: u64 = (0..NHAZ).map(|s| x.haz_k[s] as u64 * haz[s] as u64).sum();
    if hk > 0 {
        t = div_round(t as i128 * (255_000 + hk) as i128, 255_000) as u64;
    }
    for s in 0..NLEV {
        let (v, c) = lev[s];
        if c == 0 {
            continue;
        }
        if v > x.lev_block[s] {
            return closed(WHY_LEVEL, s, 255);
        }
        if x.lev_k[s] > 0 {
            t = div_round(t as i128 * (1000 + v as u64 * x.lev_k[s] as u64) as i128, 1000) as u64;
        }
    }
    if l & OBSTRUCTION != 0 {
        if x.obst_rel == 0 {
            return closed(WHY_OBSTRUCTION, 0, 255);
        }
        t = div_round(t as i128 * 1000, x.obst_rel as i128) as u64;
    }
    Cell { t: t.clamp(1, 65535) as u16, risk: risk_of(pb, haz, &blocking), why: 0 }
}

// ---- palette-run coding of one block plane (E5's coding, any value width up to u16) ----------

pub fn pr_encode(v: &[u16]) -> Vec<u8> {
    let mut pal: Vec<u16> = Vec::new();
    let idx: Vec<usize> = v
        .iter()
        .map(|x| match pal.iter().position(|p| p == x) {
            Some(k) => k,
            None => {
                pal.push(*x);
                pal.len() - 1
            }
        })
        .collect();
    let k = (usize::BITS - (pal.len().max(2) - 1).leading_zeros()) as u64;
    let mut buf = Vec::new();
    put_varint(&mut buf, pal.len() as u64);
    for &p in &pal {
        put_varint(&mut buf, p as u64);
    }
    let mut i = 0;
    while i < idx.len() {
        let mut j = i + 1;
        while j < idx.len() && idx[j] == idx[i] {
            j += 1;
        }
        put_varint(&mut buf, (((j - i - 1) as u64) << k) | idx[i] as u64);
        i = j;
    }
    buf
}

/// A stored plane: a coding byte, then 0 palette-runs, or 1 zlib (RFC 1950) of the raw plane
/// (u8, or u16 little-endian). The writer keeps the shorter, palette-runs on a tie; identity is
/// over decoded values, so the compressor's exact bytes do not matter.
pub fn plane_encode(v: &[u16], wide: bool) -> Vec<u8> {
    use std::io::Write;
    let pr = pr_encode(v);
    let raw: Vec<u8> = if wide { v.iter().flat_map(|x| x.to_le_bytes()).collect() } else { v.iter().map(|&x| x as u8).collect() };
    let mut e = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::new(9));
    e.write_all(&raw).unwrap();
    let z = e.finish().unwrap();
    let mut o = Vec::with_capacity(1 + pr.len().min(z.len()));
    if z.len() < pr.len() {
        o.push(1);
        o.extend_from_slice(&z);
    } else {
        o.push(0);
        o.extend_from_slice(&pr);
    }
    o
}

pub fn plane_decode(b: &[u8], wide: bool) -> Option<Vec<u16>> {
    use std::io::Read;
    match *b.first()? {
        0 => surface::decode_palette_runs(&b[1..]),
        1 => {
            let n = BLOCK_POSTS * if wide { 2 } else { 1 };
            let mut raw = Vec::with_capacity(n);
            flate2::read::ZlibDecoder::new(&b[1..]).take(n as u64 + 1).read_to_end(&mut raw).ok()?;
            if raw.len() != n {
                return None;
            }
            Some(if wide { raw.chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect() } else { raw.into_iter().map(|x| x as u16).collect() })
        }
        _ => None,
    }
}

fn u8s(v: &[u8]) -> Vec<u16> {
    v.iter().map(|&x| x as u16).collect()
}

// ---- blocks ------------------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProfPlanes {
    pub t: Vec<u16>,
    pub risk: Vec<u8>,
    pub why: Vec<u8>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SBlock {
    /// Hashes of the grid layers that are some post's source here, in fusion order.
    pub table: Vec<[u8; 32]>,
    pub top: Vec<u8>,
    pub cert: Vec<u8>,
    /// 0 base, k = table[k - 1], 255 team overlay.
    pub src: Vec<u8>,
    pub prof: Vec<ProfPlanes>,
    pub hash: [u8; 32],
}

impl SBlock {
    /// Canonical decoded bytes: key, table, then the planes (u8 as bytes, u16 little-endian).
    pub fn canonical(&self, id: BlockId) -> Vec<u8> {
        let mut o = Vec::with_capacity(BLOCK_POSTS * (3 + 4 * self.prof.len()) + 64);
        o.extend_from_slice(&id.key().to_le_bytes());
        o.push(self.table.len() as u8);
        for h in &self.table {
            o.extend_from_slice(h);
        }
        o.extend_from_slice(&self.top);
        o.extend_from_slice(&self.cert);
        o.extend_from_slice(&self.src);
        for pp in &self.prof {
            for v in &pp.t {
                o.extend_from_slice(&v.to_le_bytes());
            }
            o.extend_from_slice(&pp.risk);
            o.extend_from_slice(&pp.why);
        }
        o
    }
    pub fn rehash(&mut self, id: BlockId) {
        self.hash = *blake3::hash(&self.canonical(id)).as_bytes();
    }
    /// Stored payload: table, then each plane as varint length + palette-runs.
    pub fn payload(&self) -> Vec<u8> {
        let mut o = Vec::new();
        o.push(self.table.len() as u8);
        for h in &self.table {
            o.extend_from_slice(h);
        }
        let mut plane = |v: Vec<u16>, wide: bool| {
            let b = plane_encode(&v, wide);
            put_varint(&mut o, b.len() as u64);
            o.extend_from_slice(&b);
        };
        plane(u8s(&self.top), false);
        plane(u8s(&self.cert), false);
        plane(u8s(&self.src), false);
        for pp in &self.prof {
            plane(pp.t.clone(), true);
            plane(u8s(&pp.risk), false);
            plane(u8s(&pp.why), false);
        }
        o
    }
    pub fn from_payload(id: BlockId, b: &[u8], nprof: usize) -> Result<SBlock, String> {
        let mut pos = 0;
        let n = *b.first().ok_or("empty block")? as usize;
        pos += 1;
        let mut table = Vec::new();
        for _ in 0..n {
            table.push(b.get(pos..pos + 32).ok_or("short table")?.try_into().unwrap());
            pos += 32;
        }
        let mut plane = |wide: bool| -> Result<Vec<u16>, String> {
            let len = get_varint(b, &mut pos).ok_or("bad plane length")? as usize;
            let v = plane_decode(b.get(pos..pos + len).ok_or("short plane")?, wide).ok_or("bad plane")?;
            pos += len;
            if !wide && v.iter().any(|&x| x > 255) {
                return Err("u8 plane value over 255".into());
            }
            Ok(v)
        };
        let n8 = |v: Vec<u16>| v.into_iter().map(|x| x as u8).collect::<Vec<u8>>();
        let top = n8(plane(false)?);
        let cert = n8(plane(false)?);
        let src = n8(plane(false)?);
        let mut prof = Vec::new();
        for _ in 0..nprof {
            let t = plane(true)?;
            let risk = n8(plane(false)?);
            let why = n8(plane(false)?);
            prof.push(ProfPlanes { t, risk, why });
        }
        if pos != b.len() {
            return Err("trailing bytes in block".into());
        }
        if src.iter().any(|&s| s != 0 && s != 255 && s as usize > table.len()) {
            return Err("source index outside the block table".into());
        }
        let mut sb = SBlock { table, top, cert, src, prof, hash: [0; 32] };
        sb.rehash(id);
        Ok(sb)
    }
}

/// The stage-1 planes the cost rules read besides the fused state (E5's base), in block order.
pub struct Terrain {
    pub blocks: Vec<BlockId>,
    pub steep: Vec<u8>,
    pub lines: Vec<u16>,
    /// Stands for the terrain file's hash: blake3 of keys, heights, steepness, land cover,
    /// confidence bit and line mask.
    pub hash: [u8; 32],
}

impl Terrain {
    pub fn new(blocks: Vec<BlockId>, dm: &[i32], steep: Vec<u8>, lc: &[u8], conf: &[u8], lines: Vec<u16>) -> Terrain {
        let mut h = blake3::Hasher::new();
        h.update(b"wis0/terrain");
        for b in &blocks {
            h.update(&b.key().to_le_bytes());
        }
        for v in dm {
            h.update(&v.to_le_bytes());
        }
        h.update(&steep);
        h.update(lc);
        h.update(conf);
        for v in &lines {
            h.update(&v.to_le_bytes());
        }
        Terrain { blocks, steep, lines, hash: *h.finalize().as_bytes() }
    }
}

/// Surface blocks from fused blocks: top class, certainty, block-local source, and per profile
/// the cell rules.
pub fn build_blocks(ter: &Terrain, fused: &[(BlockId, FusedBlock)], rec: &Record, rules: &[Rules]) -> Vec<(BlockId, SBlock)> {
    assert!(rec.applied.len() < 254, "v0 sources index at most 253 layers");
    fused
        .par_iter()
        .map(|(id, fb)| {
            let k = ter.blocks.binary_search(id).expect("fused block outside the terrain");
            let steep = &ter.steep[k * BLOCK_POSTS..(k + 1) * BLOCK_POSTS];
            let lines = &ter.lines[k * BLOCK_POSTS..(k + 1) * BLOCK_POSTS];
            let used: BTreeSet<u8> = fb.src.iter().copied().filter(|&s| s != 0 && s != 255).collect();
            let table: Vec<[u8; 32]> = used.iter().map(|&s| rec.applied[s as usize - 1].hash).collect();
            let local: BTreeMap<u8, u8> = used.iter().enumerate().map(|(n, &s)| (s, n as u8 + 1)).collect();
            let src = fb.src.iter().map(|s| *local.get(s).unwrap_or(s)).collect();
            let top = fb
                .p
                .iter()
                .map(|p| {
                    let mx = *p.iter().max().unwrap();
                    p.iter().position(|&v| v == mx).unwrap() as u8
                })
                .collect();
            let prof = rules
                .iter()
                .map(|r| {
                    let mut pp = ProfPlanes { t: vec![0; BLOCK_POSTS], risk: vec![0; BLOCK_POSTS], why: vec![0; BLOCK_POSTS] };
                    for q in 0..BLOCK_POSTS {
                        let c = cell(r, &fb.p[q], &fb.haz[q], &fb.lev[q], fb.mask[q], steep[q], lines[q]);
                        (pp.t[q], pp.risk[q], pp.why[q]) = (c.t, c.risk, c.why);
                    }
                    pp
                })
                .collect();
            let mut sb = SBlock { table, top, cert: fb.cert.clone(), src, prof, hash: [0; 32] };
            sb.rehash(*id);
            (*id, sb)
        })
        .collect()
}

// ---- the surface file --------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AppliedRef {
    pub hash: [u8; 32],
    pub kind: u8,
    pub observed_at: i64,
    pub valid_until: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Header {
    pub incident: [u8; 16],
    pub spacing: u32,
    pub epoch_s: u32,
    pub t: i64,
    pub same_until: i64,
    pub terrain: [u8; 32],
    pub prior: [u8; 32],
    pub overlay: [u8; 32],
    pub allow_research: u8,
    pub feature_min_conf: u8,
    pub level_min_w: u8,
    pub applied: Vec<AppliedRef>,
    pub left_out: Vec<([u8; 32], u8)>,
    pub profiles: Vec<Vec<u8>>,
}

impl Header {
    fn encode(&self) -> Vec<u8> {
        let mut o = Vec::new();
        o.extend_from_slice(&self.incident);
        o.extend_from_slice(&self.spacing.to_le_bytes());
        o.extend_from_slice(&self.epoch_s.to_le_bytes());
        o.extend_from_slice(&self.t.to_le_bytes());
        o.extend_from_slice(&self.same_until.to_le_bytes());
        o.extend_from_slice(&self.terrain);
        o.extend_from_slice(&self.prior);
        o.extend_from_slice(&self.overlay);
        o.extend_from_slice(&[self.allow_research, self.feature_min_conf, self.level_min_w]);
        put_varint(&mut o, self.applied.len() as u64);
        for a in &self.applied {
            o.extend_from_slice(&a.hash);
            o.push(a.kind);
            o.extend_from_slice(&a.observed_at.to_le_bytes());
            o.extend_from_slice(&a.valid_until.to_le_bytes());
        }
        put_varint(&mut o, self.left_out.len() as u64);
        for (h, r) in &self.left_out {
            o.extend_from_slice(h);
            o.push(*r);
        }
        o.push(self.profiles.len() as u8);
        for p in &self.profiles {
            put_varint(&mut o, p.len() as u64);
            o.extend_from_slice(p);
        }
        o
    }

    fn decode(b: &[u8], pos: &mut usize) -> Option<Header> {
        let take = |pos: &mut usize, n: usize| -> Option<&[u8]> {
            let s = b.get(*pos..*pos + n)?;
            *pos += n;
            Some(s)
        };
        let h32 = |pos: &mut usize| -> Option<[u8; 32]> { take(pos, 32)?.try_into().ok() };
        let i64_ = |pos: &mut usize| -> Option<i64> { Some(i64::from_le_bytes(take(pos, 8)?.try_into().ok()?)) };
        let incident = take(pos, 16)?.try_into().ok()?;
        let spacing = u32::from_le_bytes(take(pos, 4)?.try_into().ok()?);
        let epoch_s = u32::from_le_bytes(take(pos, 4)?.try_into().ok()?);
        let t = i64_(pos)?;
        let same_until = i64_(pos)?;
        let (terrain, prior, overlay) = (h32(pos)?, h32(pos)?, h32(pos)?);
        let pr = take(pos, 3)?;
        let (allow_research, feature_min_conf, level_min_w) = (pr[0], pr[1], pr[2]);
        let n = get_varint(b, pos)? as usize;
        let mut applied = Vec::new();
        for _ in 0..n {
            let hash = h32(pos)?;
            let kind = take(pos, 1)?[0];
            applied.push(AppliedRef { hash, kind, observed_at: i64_(pos)?, valid_until: i64_(pos)? });
        }
        let n = get_varint(b, pos)? as usize;
        let mut left_out = Vec::new();
        for _ in 0..n {
            let h = h32(pos)?;
            left_out.push((h, take(pos, 1)?[0]));
        }
        let np = take(pos, 1)?[0] as usize;
        let mut profiles = Vec::new();
        for _ in 0..np {
            let n = get_varint(b, pos)? as usize;
            profiles.push(take(pos, n)?.to_vec());
        }
        Some(Header {
            incident, spacing, epoch_s, t, same_until, terrain, prior, overlay, allow_research, feature_min_conf, level_min_w,
            applied, left_out, profiles,
        })
    }

    /// What the routes depend on besides the blocks: spacing, terrain (the DEM), profile tables.
    fn content_prefix(&self, h: &mut blake3::Hasher) {
        h.update(b"wis0/content");
        h.update(&MAJOR.to_le_bytes());
        h.update(&self.spacing.to_le_bytes());
        h.update(&self.terrain);
        h.update(&[self.profiles.len() as u8]);
        for p in &self.profiles {
            h.update(blake3::hash(p).as_bytes());
        }
    }
}

#[derive(Clone, Debug)]
pub struct Surface {
    pub h: Header,
    pub blocks: BTreeMap<u64, (BlockId, SBlock)>,
}

impl Surface {
    pub fn content_hash(&self) -> [u8; 32] {
        let mut h = blake3::Hasher::new();
        self.h.content_prefix(&mut h);
        h.update(&(self.blocks.len() as u32).to_le_bytes());
        for (k, (_, b)) in &self.blocks {
            h.update(&k.to_le_bytes());
            h.update(&b.hash);
        }
        *h.finalize().as_bytes()
    }

    /// Every input: the header (which holds T, the layers applied and left out, the overlay,
    /// the params and the profile tables), the versions and the AOI.
    pub fn recipe_hash(&self) -> [u8; 32] {
        let mut h = blake3::Hasher::new();
        h.update(b"wis0/recipe");
        h.update(&MAJOR.to_le_bytes());
        h.update(&MINOR.to_le_bytes());
        h.update(&crate::layers::MAJOR.to_le_bytes());
        h.update(&self.h.encode());
        h.update(&(self.blocks.len() as u32).to_le_bytes());
        for k in self.blocks.keys() {
            h.update(&k.to_le_bytes());
        }
        *h.finalize().as_bytes()
    }

    /// The file: magic, versions, header, recipe and content hashes, block index (key, length,
    /// block hash), payloads in key order.
    pub fn write(&self) -> Vec<u8> {
        let mut o = Vec::new();
        o.extend_from_slice(MAGIC);
        o.extend_from_slice(&MAJOR.to_le_bytes());
        o.extend_from_slice(&MINOR.to_le_bytes());
        o.extend_from_slice(&self.h.encode());
        o.extend_from_slice(&self.recipe_hash());
        o.extend_from_slice(&self.content_hash());
        put_varint(&mut o, self.blocks.len() as u64);
        let pays: Vec<Vec<u8>> = self.blocks.values().map(|(_, b)| b.payload()).collect();
        for ((k, (_, b)), p) in self.blocks.iter().zip(&pays) {
            o.extend_from_slice(&k.to_le_bytes());
            o.extend_from_slice(&(p.len() as u32).to_le_bytes());
            o.extend_from_slice(&b.hash);
        }
        for p in pays {
            o.extend_from_slice(&p);
        }
        o
    }

    pub fn read(b: &[u8]) -> Result<Surface, String> {
        if b.get(..4) != Some(MAGIC) {
            return Err("not an incident surface".into());
        }
        let major = u16::from_le_bytes(b[4..6].try_into().unwrap());
        if major != MAJOR {
            return Err(format!("surface contract major {major}, reader {MAJOR}"));
        }
        let mut pos = 8;
        let h = Header::decode(b, &mut pos).ok_or("bad header")?;
        let recipe: [u8; 32] = b.get(pos..pos + 32).ok_or("short")?.try_into().unwrap();
        let content: [u8; 32] = b.get(pos + 32..pos + 64).ok_or("short")?.try_into().unwrap();
        pos += 64;
        let (blocks, end) = read_blocks(b, pos, h.profiles.len())?;
        if end != b.len() {
            return Err("trailing bytes".into());
        }
        let s = Surface { h, blocks };
        if s.content_hash() != content {
            return Err("content hash mismatch".into());
        }
        if s.recipe_hash() != recipe {
            return Err("recipe hash mismatch".into());
        }
        Ok(s)
    }

    pub fn rules(&self) -> Vec<Rules> {
        self.h.profiles.iter().map(|b| Rules::from_bytes(b).expect("profile tables")).collect()
    }
}

fn read_blocks(b: &[u8], mut pos: usize, nprof: usize) -> Result<(BTreeMap<u64, (BlockId, SBlock)>, usize), String> {
    let n = get_varint(b, &mut pos).ok_or("bad block count")? as usize;
    let mut idx = Vec::with_capacity(n);
    for _ in 0..n {
        let e = b.get(pos..pos + 44).ok_or("short index")?;
        let key = u64::from_le_bytes(e[..8].try_into().unwrap());
        let len = u32::from_le_bytes(e[8..12].try_into().unwrap()) as usize;
        let hash: [u8; 32] = e[12..44].try_into().unwrap();
        idx.push((key, len, hash));
        pos += 44;
    }
    if idx.windows(2).any(|w| w[0].0 >= w[1].0) {
        return Err("block index not sorted".into());
    }
    let mut out = BTreeMap::new();
    for (key, len, hash) in idx {
        let id = BlockId { bi: ((key >> 32) as u32 ^ 0x8000_0000) as i32, bj: (key as u32 ^ 0x8000_0000) as i32 };
        let sb = SBlock::from_payload(id, b.get(pos..pos + len).ok_or("short payload")?, nprof)?;
        if sb.hash != hash {
            return Err(format!("block {key:x} hash mismatch"));
        }
        out.insert(key, (id, sb));
        pos += len;
    }
    Ok((out, pos))
}

/// A delta from `old` to `new`: the new header, both content hashes, the changed or added blocks
/// (op 1, with payload) and removed blocks (op 0).
pub fn delta(old: &Surface, new: &Surface) -> Vec<u8> {
    let mut o = Vec::new();
    o.extend_from_slice(DELTA_MAGIC);
    o.extend_from_slice(&MAJOR.to_le_bytes());
    o.extend_from_slice(&MINOR.to_le_bytes());
    o.extend_from_slice(&old.content_hash());
    o.extend_from_slice(&new.h.encode());
    o.extend_from_slice(&new.recipe_hash());
    o.extend_from_slice(&new.content_hash());
    let mut ch: Vec<(u64, Option<&SBlock>)> = Vec::new();
    for (k, (_, b)) in &new.blocks {
        if old.blocks.get(k).map(|x| x.1.hash) != Some(b.hash) {
            ch.push((*k, Some(b)));
        }
    }
    for k in old.blocks.keys() {
        if !new.blocks.contains_key(k) {
            ch.push((*k, None));
        }
    }
    ch.sort_by_key(|c| c.0);
    put_varint(&mut o, ch.len() as u64);
    for (k, b) in ch {
        o.extend_from_slice(&k.to_le_bytes());
        match b {
            None => o.push(0),
            Some(b) => {
                o.push(1);
                let p = b.payload();
                o.extend_from_slice(&(p.len() as u32).to_le_bytes());
                o.extend_from_slice(&b.hash);
                o.extend_from_slice(&p);
            }
        }
    }
    o
}

pub fn apply_delta(old: &Surface, b: &[u8]) -> Result<Surface, String> {
    if b.get(..4) != Some(DELTA_MAGIC) || u16::from_le_bytes(b[4..6].try_into().unwrap()) != MAJOR {
        return Err("not a surface delta of this major".into());
    }
    if b.get(8..40) != Some(&old.content_hash()[..]) {
        return Err("delta is not from this surface".into());
    }
    let mut pos = 40;
    let h = Header::decode(b, &mut pos).ok_or("bad header")?;
    let recipe: [u8; 32] = b.get(pos..pos + 32).ok_or("short")?.try_into().unwrap();
    let content: [u8; 32] = b.get(pos + 32..pos + 64).ok_or("short")?.try_into().unwrap();
    pos += 64;
    if h.profiles != old.h.profiles {
        return Err("a delta cannot change the profile tables".into());
    }
    let mut blocks = old.blocks.clone();
    let n = get_varint(b, &mut pos).ok_or("bad count")? as usize;
    for _ in 0..n {
        let key = u64::from_le_bytes(b.get(pos..pos + 8).ok_or("short")?.try_into().unwrap());
        let op = *b.get(pos + 8).ok_or("short")?;
        pos += 9;
        let id = BlockId { bi: ((key >> 32) as u32 ^ 0x8000_0000) as i32, bj: (key as u32 ^ 0x8000_0000) as i32 };
        if op == 0 {
            blocks.remove(&key).ok_or("removing a block that is not there")?;
            continue;
        }
        let len = u32::from_le_bytes(b.get(pos..pos + 4).ok_or("short")?.try_into().unwrap()) as usize;
        let hash: [u8; 32] = b.get(pos + 4..pos + 36).ok_or("short")?.try_into().unwrap();
        pos += 36;
        let sb = SBlock::from_payload(id, b.get(pos..pos + len).ok_or("short")?, h.profiles.len())?;
        if sb.hash != hash {
            return Err("delta block hash mismatch".into());
        }
        pos += len;
        blocks.insert(key, (id, sb));
    }
    if pos != b.len() {
        return Err("trailing bytes".into());
    }
    let s = Surface { h, blocks };
    if s.content_hash() != content || s.recipe_hash() != recipe {
        return Err("delta result hash mismatch".into());
    }
    Ok(s)
}

// ---- epochs ------------------------------------------------------------------------------------

pub fn epoch_of(now: i64, e: i64) -> i64 {
    now.div_euclid(e) * e
}
fn ceil_e(x: i64, e: i64) -> i64 {
    -((-x).div_euclid(e)) * e
}

/// The first epoch start after T at which some input's activity or effective confidence can
/// change (i64::MAX if none): each layer's valid_from and valid_until rounded up to an epoch, the
/// first epoch after fade_from, and T + E while a layer is fading.
pub fn same_until(t: i64, e: i64, ms: &[&Manifest]) -> i64 {
    let mut best = i64::MAX;
    for m in ms {
        let mut c = vec![ceil_e(m.valid_from, e), ceil_e(m.valid_until, e)];
        if m.fade_from < m.valid_until {
            c.push(m.fade_from.div_euclid(e) * e + e);
            if m.fade_from < t && t < m.valid_until {
                c.push(t + e);
            }
        }
        for x in c {
            if x > t {
                best = best.min(x);
            }
        }
    }
    best
}

/// Blocks a layer can change: its grid blocks and the blocks its feature lines cross.
pub fn touched_blocks(l: &Layer, spacing: i64) -> BTreeSet<BlockId> {
    let mut s: BTreeSet<BlockId> = l.blocks.iter().map(|b| b.id).collect();
    for f in &l.features {
        for seg in f.pts.windows(2) {
            fuse::supercover(seg[0], seg[1], spacing, &mut |i, j| {
                s.insert(BlockId { bi: i.div_euclid(BLOCK as i64) as i32, bj: j.div_euclid(BLOCK as i64) as i32 });
            });
        }
    }
    s
}

/// Blocks an overlay item covers.
pub fn overlay_blocks(it: &fuse::OverlayItem, spacing: i64) -> BTreeSet<BlockId> {
    let mut s = BTreeSet::new();
    let (i0, i1) = (ceil_div(it.lat0, spacing), ceil_div(it.lat1, spacing) - 1);
    let (j0, j1) = (ceil_div(it.lon0, spacing), ceil_div(it.lon1, spacing) - 1);
    for bi in i0.div_euclid(64)..=i1.div_euclid(64) {
        for bj in j0.div_euclid(64)..=j1.div_euclid(64) {
            s.insert(BlockId { bi: bi as i32, bj: bj as i32 });
        }
    }
    s
}
fn ceil_div(a: i64, b: i64) -> i64 {
    -((-a).div_euclid(b))
}

// ---- building a surface ------------------------------------------------------------------------

pub struct Inputs<'a> {
    pub incident: [u8; 16],
    pub base: &'a fuse::Base,
    pub ter: &'a Terrain,
    pub layers: Vec<(&'a Layer, [u8; 32])>,
    pub overlay: Vec<fuse::OverlayItem>,
    pub rules: &'a [Rules],
    pub epoch_s: i64,
    pub spacing: i64,
    pub allow_research: bool,
    pub feature_min_conf: u8,
    pub level_min_w: u8,
}

impl Inputs<'_> {
    fn params(&self, t: i64) -> fuse::Params {
        fuse::Params { t, allow_research: self.allow_research, feature_min_conf: self.feature_min_conf, level_min_w: self.level_min_w, spacing: self.spacing }
    }

    /// The surface at epoch T. With `prev` and `only`, only those blocks are re-fused and costed;
    /// the rest are taken from `prev` (the caller names every block whose inputs changed).
    pub fn surface(&self, t: i64, prev: Option<&Surface>, only: Option<&[BlockId]>) -> Surface {
        assert_eq!(t, epoch_of(t, self.epoch_s), "T must be an epoch start");
        let (fused, rec) = fuse::fuse(self.base, &self.layers, &self.overlay, &self.params(t), only);
        let built = build_blocks(self.ter, &fused, &rec, self.rules);
        let mut blocks = match (prev, only) {
            (Some(p), Some(_)) => p.blocks.clone(),
            _ => BTreeMap::new(),
        };
        for (id, sb) in built {
            blocks.insert(id.key(), (id, sb));
        }
        let ms: Vec<&Manifest> = self
            .layers
            .iter()
            .filter(|(l, h)| {
                rec.ignored.iter().all(|(ih, why)| ih != h || !matches!(reason_code(why), 2 | 3)) && (l.m.licence == 0 || self.allow_research)
            })
            .map(|(l, _)| &l.m)
            .collect();
        let until: BTreeMap<[u8; 32], i64> = self.layers.iter().map(|(l, h)| (*h, l.m.valid_until)).collect();
        let h = Header {
            incident: self.incident,
            spacing: self.spacing as u32,
            epoch_s: self.epoch_s as u32,
            t,
            same_until: same_until(t, self.epoch_s, &ms),
            terrain: self.ter.hash,
            prior: rec.prior,
            overlay: rec.overlay,
            allow_research: self.allow_research as u8,
            feature_min_conf: self.feature_min_conf,
            level_min_w: self.level_min_w,
            applied: rec
                .applied
                .iter()
                .map(|a| AppliedRef { hash: a.hash, kind: kind_u8(a.kind), observed_at: a.observed_at, valid_until: until[&a.hash] })
                .collect(),
            left_out: rec.ignored.iter().map(|(h, why)| (*h, reason_code(why))).collect(),
            profiles: self.rules.iter().map(|r| r.bytes()).collect(),
        };
        Surface { h, blocks }
    }
}

pub fn kind_u8(k: Kind) -> u8 {
    match k {
        Kind::Class => 1,
        Kind::Hazard => 2,
        Kind::Level => 3,
        Kind::Feature => 4,
    }
}

// ---- queries: dense planes, canonical path, route confidence ----------------------------------

/// The surface's planes on E5's dense rectangle (rows from the south), for the search.
pub struct Dense {
    pub t: Vec<Vec<u16>>,
    pub risk: Vec<Vec<u8>>,
    pub cert: Vec<u8>,
    /// 0 base, 255 team, else 1 + index into the header's applied list.
    pub src: Vec<u8>,
}

pub fn dense(s: &Surface, b: &surface::Base) -> Dense {
    let n = b.h * b.w;
    let np = s.h.profiles.len();
    let mut d = Dense { t: vec![vec![0; n]; np], risk: vec![vec![0; n]; np], cert: vec![0; n], src: vec![0; n] };
    let pos: BTreeMap<[u8; 32], u8> = s.h.applied.iter().enumerate().map(|(k, a)| (a.hash, k as u8 + 1)).collect();
    for (id, sb) in s.blocks.values() {
        let (r0, c0) = ((id.bi as i64 - b.bi0) as usize * BLOCK, (id.bj as i64 - b.bj0) as usize * BLOCK);
        for q in 0..BLOCK_POSTS {
            let i = (r0 + q / BLOCK) * b.w + c0 + q % BLOCK;
            d.cert[i] = sb.cert[q];
            d.src[i] = match sb.src[q] {
                0 => 0,
                255 => 255,
                k => pos[&sb.table[k as usize - 1]],
            };
            for p in 0..np {
                d.t[p][i] = sb.prof[p].t[q];
                d.risk[p][i] = sb.prof[p].risk[q];
            }
        }
    }
    d
}

/// The canonical path to `dst` after a Dijkstra run from the source reached it: from dst back,
/// the predecessor u with d(u) + c(u, v) = d(v), smallest cell index first.
pub fn path(g: &surface::Graph, se: &surface::Search, src: usize, dst: usize) -> Vec<(usize, usize)> {
    let w = g.b.w as i64;
    let mut out = Vec::new(); // (from cell, move)
    let mut v = dst;
    while v != src {
        let (vr, vc) = ((v as i64) / w, (v as i64) % w);
        let mut best: Option<(usize, usize)> = None;
        for (m, &(dr, dc)) in surface::MOVES.iter().enumerate() {
            let (ur, uc) = (vr - dr as i64, vc - dc as i64);
            if ur < 0 || uc < 0 || ur >= g.b.h as i64 || uc >= w {
                continue;
            }
            let u = (ur * w + uc) as usize;
            let du = se.d(u);
            if du == u64::MAX {
                continue;
            }
            if let Some((z, c)) = g.cost(ur as usize, uc as usize, m) {
                debug_assert_eq!(z, v);
                if du + c == se.d(v) && best.is_none_or(|b| u < b.0) {
                    best = Some((u, m));
                }
            }
        }
        let (u, m) = best.expect("no predecessor on a settled path");
        out.push((u, m));
        v = u;
    }
    out.reverse();
    out
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RouteConf {
    pub cost_us: u64,
    pub len_mm: u64,
    pub mean_cert: u8,
    pub min_cert: u8,
    pub layer_permille: u32,
    pub team_permille: u32,
    pub max_risk: u8,
    pub risky_permille: u32,
    /// Indexes (1-based) into the header's applied list of the layers the route rests on.
    pub layers: Vec<u8>,
    /// The route touches the outermost ring of the AOI: a larger AOI might give a faster route.
    pub at_edge: bool,
}

/// Route confidence (contract section 7): each move's length is shared equally among the cells it
/// crosses (2, or 4 for a knight move), as weights 4 L / n.
pub fn route_conf(g: &surface::Graph, d: &Dense, prof: usize, steps: &[(usize, usize)], cost_us: u64) -> RouteConf {
    let w = g.b.w;
    let (mut wsum, mut csum, mut lay, mut team, mut risky, mut len) = (0u64, 0u64, 0u64, 0u64, 0u64, 0u64);
    let (mut minc, mut maxr) = (255u8, 0u8);
    let mut layers = BTreeSet::new();
    let mut at_edge = false;
    for &(u, m) in steps {
        let (r, c) = (u / w, u % w);
        let (dr, dc) = surface::MOVES[m];
        let l = g.len[(2 * r as i64 + dr as i64 + 2) as usize][m] as u64;
        len += l;
        let mut cells = vec![(r as i64, c as i64), (r as i64 + dr as i64, c as i64 + dc as i64)];
        if dr.abs() + dc.abs() == 3 {
            if dr.abs() == 1 {
                cells.push((r as i64, c as i64 + dc as i64 / 2));
                cells.push((r as i64 + dr as i64, c as i64 + dc as i64 / 2));
            } else {
                cells.push((r as i64 + dr as i64 / 2, c as i64));
                cells.push((r as i64 + dr as i64 / 2, c as i64 + dc as i64));
            }
        }
        let wt = 4 * l / cells.len() as u64;
        for (cr, cc) in cells {
            if cr == 0 || cc == 0 || cr == g.b.h as i64 - 1 || cc == w as i64 - 1 {
                at_edge = true;
            }
            let i = cr as usize * w + cc as usize;
            wsum += wt;
            csum += wt * d.cert[i] as u64;
            minc = minc.min(d.cert[i]);
            match d.src[i] {
                0 => {}
                255 => team += wt,
                k => {
                    lay += wt;
                    layers.insert(k);
                }
            }
            let rk = d.risk[prof][i];
            maxr = maxr.max(rk);
            if rk > 0 {
                risky += wt;
            }
        }
    }
    let pm = |x: u64| if wsum == 0 { 0 } else { div_round(1000 * x as i128, wsum as i128) as u32 };
    RouteConf {
        cost_us,
        len_mm: len,
        mean_cert: if wsum == 0 { 0 } else { div_round(csum as i128, wsum as i128) as u8 },
        min_cert: if wsum == 0 { 0 } else { minc },
        layer_permille: pm(lay),
        team_permille: pm(team),
        max_risk: maxr,
        risky_permille: pm(risky),
        layers: layers.into_iter().collect(),
        at_edge,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn onehot(c: usize) -> [u16; NCLASS] {
        let mut p = [0u16; NCLASS];
        p[c] = S as u16;
        p
    }

    #[test]
    fn rules_roundtrip() {
        let r = Rules { p: surface::vehicle(), x: vehicle_ext() };
        let b = r.bytes();
        let back = Rules::from_bytes(&b).unwrap();
        assert_eq!(back.bytes(), b);
    }

    #[test]
    fn one_hot_is_e5() {
        let grade: Vec<u32> = (0..201).map(|_| 1000).collect();
        for r in [Rules { p: surface::walker(grade), x: walker_ext() }, Rules { p: surface::vehicle(), x: vehicle_ext() }] {
            for c in 0..16 {
                for steep in [0u8, 9, 12, 0xFF] {
                    for lines in [0u16, PATH, RIVER, STREAM | FENCE, MOTORWAY, MOTORWAY | MINOR, CLIFF | TRACK] {
                        let e5 = surface::cell_t(&r.p, &[(c as u8, S)], steep, lines);
                        let got = cell(&r, &onehot(c), &[0; NHAZ], &[(0, 0); NLEV], 0, steep, lines);
                        assert_eq!(got.t as u32, e5, "class {c} steep {steep} lines {lines:x}");
                        assert_eq!(got.risk, if e5 > 0 { 0 } else { 255 });
                    }
                }
            }
        }
    }

    #[test]
    fn blocking_mass_and_hazards() {
        let grade: Vec<u32> = (0..201).map(|_| 1000).collect();
        let r = Rules { p: surface::walker(grade), x: walker_ext() };
        let mut p = onehot(3);
        p[3] = S as u16 - 82; // 2 % water
        p[8] = 82;
        let c = cell(&r, &p, &[0; NHAZ], &[(0, 0); NLEV], 0, 0, 0);
        assert!(c.t > 0 && c.risk == 5, "{c:?}");
        // a meadow with 60 % flood probability blocks; a measured shallow depth lifts it
        let mut h = [0u8; NHAZ];
        h[0] = 153;
        assert_eq!(cell(&r, &onehot(3), &h, &[(0, 0); NLEV], 0, 0, 0).why, WHY_HAZARD << 4);
        let lev = [(20, 200), (0, 0), (0, 0)];
        let c = cell(&r, &onehot(3), &h, &lev, 0, 0, 0);
        assert!(c.t > 0 && c.risk == 0, "{c:?}");
        // team block beats a path
        assert_eq!(cell(&r, &onehot(3), &[0; NHAZ], &[(0, 0); NLEV], M_TEAM_BLOCK, 0, PATH).t, 0);
    }
}

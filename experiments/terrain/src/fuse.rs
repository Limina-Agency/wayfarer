//! E10: fusion of the stage-1 base with terrain layers into the per-cell state the incident
//! surface is costed from (PLAN.md section 7 rules, contract v0).
//!
//! Per post, in this order:
//! 1. The base: E3's 4-bit class and confidence bit, turned into a class distribution by a
//!    prior table (one-hot by default, so E5's surface is unchanged without layers).
//! 2. Grid layers active at the incident time T, oldest first by (observed_at, content hash):
//!    class layers blend the distribution as `P <- (1 - w) P + w (Q + q_keep P)`, hazard layers
//!    blend each probability as `h <- (1 - w) h + w q`, level layers replace the value when
//!    `w >= 1/2`. A newer confident layer therefore wins, a weak one blends, and w = 0 is a no-op.
//! 3. Feature layers: lines at or above the confidence threshold OR their effect bits into the
//!    posts they cross (exact supercover on the lattice).
//! 4. Team overlay items, last: they beat every raster and feature.
//!
//! All arithmetic is integer with one rounding rule per step, and each post depends only on its
//! own inputs, so any block order, thread count or incremental re-fusion gives the same bytes.

use crate::lattice::{BLOCK, BLOCK_POSTS, BlockId};
use crate::layers::{self, FEAT_DEBRIS, FEAT_FALLEN, FEAT_PASSAGE, FEAT_WASHOUT, HAZARDS, Kind, LEVELS, Layer, NCLASS, NHAZ, NLEV};
use rayon::prelude::*;

/// The fused class distribution sums to this: E5's PSCALE, so the surface builder takes it as is.
pub const S: u32 = 1 << 12;

// Effect bits a feature or an overlay item sets (to be ORed into E5's per-post mask).
pub const M_BARRIER: u16 = 1 << 0; // a format-4 BarrierLines class 1-16 seen by a model
pub const M_DEBRIS: u16 = 1 << 1;
pub const M_FALLEN: u16 = 1 << 2;
pub const M_WASHOUT: u16 = 1 << 3;
pub const M_PASSAGE: u16 = 1 << 4;
pub const M_TEAM_BLOCK: u16 = 1 << 5;

pub fn feature_bits(class: u8) -> u16 {
    match class {
        1..=16 => M_BARRIER,
        FEAT_DEBRIS => M_DEBRIS,
        FEAT_FALLEN => M_FALLEN,
        FEAT_WASHOUT => M_WASHOUT,
        FEAT_PASSAGE => M_PASSAGE,
        _ => 0,
    }
}

pub struct Base {
    pub blocks: Vec<BlockId>,
    pub class: Vec<u8>,
    pub conf: Vec<u8>,
    /// Distribution for (class, confidence bit) = index class * 2 + bit; sums to S.
    pub prior: Vec<[u16; NCLASS]>,
    pub hash: [u8; 32],
}

impl Base {
    pub fn new(blocks: Vec<BlockId>, class: Vec<u8>, conf: Vec<u8>) -> Base {
        assert_eq!(class.len(), blocks.len() * BLOCK_POSTS);
        let mut prior = vec![[0u16; NCLASS]; NCLASS * 2];
        for c in 0..NCLASS {
            for bit in 0..2 {
                prior[c * 2 + bit][c] = S as u16;
            }
        }
        let mut h = blake3::Hasher::new();
        for b in &blocks {
            h.update(&b.key().to_le_bytes());
        }
        h.update(&class);
        h.update(&conf);
        Base { blocks, class, conf, prior, hash: *h.finalize().as_bytes() }
    }
    pub fn prior_hash(&self) -> [u8; 32] {
        let mut h = blake3::Hasher::new();
        for d in &self.prior {
            for v in d {
                h.update(&v.to_le_bytes());
            }
        }
        *h.finalize().as_bytes()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Effect {
    Block,
    /// The team walked it: hazards and levels cleared.
    Clear,
    #[allow(dead_code)] // not used by the E10 synthetic overlay
    SetHazard(u16, u8),
}

/// A team overlay item over a box of post positions in ticks, `[lat0, lat1) x [lon0, lon1)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OverlayItem {
    pub lat0: i64,
    pub lat1: i64,
    pub lon0: i64,
    pub lon1: i64,
    pub effect: Effect,
}

pub struct Params {
    pub t: i64,
    pub allow_research: bool,
    pub feature_min_conf: u8,
    pub level_min_w: u8,
    pub spacing: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FusedBlock {
    pub p: Vec<[u16; NCLASS]>,
    pub haz: Vec<[u8; NHAZ]>,
    pub lev: Vec<[(u16, u8); NLEV]>,
    pub mask: Vec<u16>,
    /// max class probability, in 1/255
    pub cert: Vec<u8>,
    /// 0 base, k = k-th applied grid layer (1-based, saturating at 254), 255 team overlay
    pub src: Vec<u8>,
}

impl FusedBlock {
    pub fn hash_into(&self, id: BlockId, h: &mut blake3::Hasher) {
        h.update(&self.bytes(id));
    }

    /// Canonical bytes of a fused block: key, then per post P (16 u16), hazards (8 u8), levels
    /// (3 x u16 value + u8 confidence), effect bits u16, certainty u8, source u8; little-endian.
    pub fn bytes(&self, id: BlockId) -> Vec<u8> {
        let mut buf = Vec::with_capacity(BLOCK_POSTS * 48);
        buf.extend_from_slice(&id.key().to_le_bytes());
        for q in 0..BLOCK_POSTS {
            for v in self.p[q] {
                buf.extend_from_slice(&v.to_le_bytes());
            }
            buf.extend_from_slice(&self.haz[q]);
            for (v, c) in self.lev[q] {
                buf.extend_from_slice(&v.to_le_bytes());
                buf.push(c);
            }
            buf.extend_from_slice(&self.mask[q].to_le_bytes());
            buf.push(self.cert[q]);
            buf.push(self.src[q]);
        }
        buf
    }
}

pub struct Applied {
    pub hash: [u8; 32],
    pub kind: Kind,
    pub observed_at: i64,
}

pub struct Record {
    pub t: i64,
    pub base: [u8; 32],
    pub prior: [u8; 32],
    pub applied: Vec<Applied>,
    pub ignored: Vec<([u8; 32], String)>,
    pub overlay: [u8; 32],
    pub fused: [u8; 32],
}

/// Effective confidence at T: the stored w, faded linearly to 0 over [fade_from, valid_until).
pub fn w_eff(w: u8, m: &layers::Manifest, t: i64) -> u8 {
    if t < m.fade_from {
        return w;
    }
    let num = (m.valid_until - t) as i128;
    let den = (m.valid_until - m.fade_from) as i128;
    (w as i128 * num / den) as u8
}

/// P <- (1 - w) P + w (Q + q_keep P), all in exact integers, then largest-remainder to S.
pub fn blend_class(p: &mut [u16; NCLASS], q: &[u8; NCLASS], w: u8) {
    if w == 0 {
        return;
    }
    let (w, keep) = (w as u64, q[0] as u64);
    let mut raw = [0u64; NCLASS];
    for k in 0..NCLASS {
        let qk = if k == 0 { 0 } else { q[k] as u64 };
        // units of 1 / (255^2 S): (255 - w) * 255 * P + w * (q * S + keep * P)
        raw[k] = (255 - w) * 255 * p[k] as u64 + w * (qk * S as u64 + keep * p[k] as u64);
    }
    let mut out = [0u64; NCLASS];
    layers::normalise(&raw, S as u64, &mut out);
    for k in 0..NCLASS {
        p[k] = out[k] as u16;
    }
}

pub fn blend_hazard(h: u8, q: u8, w: u8) -> u8 {
    let (h, q, w) = (h as u32, q as u32, w as u32);
    (((255 - w) * h + w * q + 127) / 255) as u8
}

fn hazard_slot(code: u16) -> Option<usize> {
    HAZARDS.iter().position(|h| h.0 == code)
}
fn level_slot(code: u16) -> Option<usize> {
    LEVELS.iter().position(|h| h.0 == code)
}

/// Exact supercover of a segment over the lattice cells: calls `visit(i, j)` for every post whose
/// cell `[p - s/2, p + s/2)` the segment touches (both neighbours at an exact corner crossing).
/// Points are E7 degrees; E7 x 27 and ticks x 625 share a unit.
pub fn supercover(a: (i32, i32), b: (i32, i32), s: i64, visit: &mut impl FnMut(i64, i64)) {
    let c = s * 625;
    let half = (s / 2) * 625;
    let (ay, ax) = (a.0 as i64 * 27 + half, a.1 as i64 * 27 + half);
    let (by, bx) = (b.0 as i64 * 27 + half, b.1 as i64 * 27 + half);
    let (mut cy, mut cx) = (ay.div_euclid(c), ax.div_euclid(c));
    let (ey, ex) = (by.div_euclid(c), bx.div_euclid(c));
    let (dy, dx) = (by - ay, bx - ax);
    let (sy, sx) = (dy.signum(), dx.signum());
    visit(cy, cx);
    let mut guard = 0u64;
    while (cy, cx) != (ey, ex) {
        guard += 1;
        assert!(guard < 10_000_000, "supercover did not terminate");
        // distance to the next boundary along each axis, compared as ny/|dy| vs nx/|dx|
        let nx = if sx > 0 { (cx + 1) * c - ax } else { ax - cx * c };
        let ny = if sy > 0 { (cy + 1) * c - ay } else { ay - cy * c };
        let tx = if dx == 0 { None } else { Some(nx as i128 * dy.abs() as i128) };
        let ty = if dy == 0 { None } else { Some(ny as i128 * dx.abs() as i128) };
        match (tx, ty) {
            (Some(x), Some(y)) if x == y => {
                visit(cy, cx + sx);
                visit(cy + sy, cx);
                cx += sx;
                cy += sy;
            }
            (Some(x), Some(y)) if x < y => cx += sx,
            (Some(_), None) => cx += sx,
            _ => cy += sy,
        }
        visit(cy, cx);
    }
}

fn block_of(i: i64, j: i64) -> (BlockId, usize) {
    let (bi, bj) = (i.div_euclid(BLOCK as i64), j.div_euclid(BLOCK as i64));
    let p = (i.rem_euclid(BLOCK as i64) * BLOCK as i64 + j.rem_euclid(BLOCK as i64)) as usize;
    (BlockId { bi: bi as i32, bj: bj as i32 }, p)
}

pub fn overlay_hash(items: &[OverlayItem]) -> [u8; 32] {
    let mut h = blake3::Hasher::new();
    for it in items {
        for v in [it.lat0, it.lat1, it.lon0, it.lon1] {
            h.update(&v.to_le_bytes());
        }
        let (k, c, v) = match it.effect {
            Effect::Block => (1u8, 0u16, 0u8),
            Effect::Clear => (2, 0, 0),
            Effect::SetHazard(c, v) => (3, c, v),
        };
        h.update(&[k]);
        h.update(&c.to_le_bytes());
        h.update(&[v]);
    }
    *h.finalize().as_bytes()
}

fn contributes(l: &Layer, prm: &Params) -> bool {
    match l.m.kind {
        Kind::Feature => l.features.iter().any(|f| f.conf >= prm.feature_min_conf && feature_bits(f.class) != 0),
        Kind::Level => l.blocks.iter().any(|b| (0..BLOCK_POSTS).any(|q| b.has(q) && w_eff(b.conf[q], &l.m, prm.t) >= prm.level_min_w)),
        _ => l.blocks.iter().any(|b| (0..BLOCK_POSTS).any(|q| b.has(q) && w_eff(b.conf[q], &l.m, prm.t) > 0)),
    }
}

/// Fuse `only` (a subset of the base blocks, or all when None). Returns blocks in base order.
pub fn fuse(base: &Base, layers_in: &[&Layer], overlay: &[OverlayItem], prm: &Params, only: Option<&[BlockId]>)
    -> (Vec<(BlockId, FusedBlock)>, Record) {
    // Which layers apply, in fusion order.
    let mut active: Vec<(i64, [u8; 32], &Layer)> = Vec::new();
    let mut ignored = Vec::new();
    for l in layers_in {
        let h = l.content_hash();
        let m = &l.m;
        let why = if !(m.valid_from <= prm.t && prm.t < m.valid_until) {
            Some("outside its validity window at T")
        } else if m.licence == 1 && !prm.allow_research {
            Some("research-only licence")
        } else if m.s_lat as i64 != prm.spacing || m.s_lon as i64 != prm.spacing {
            Some("spacing differs from the surface (resample at import)")
        } else if !contributes(l, prm) {
            // A layer with no weight anywhere at T changes nothing; leaving it out of the record
            // keeps the source indexes of the others stable.
            Some("no confidence anywhere at T")
        } else {
            None
        };
        match why {
            Some(r) => ignored.push((h, r.to_string())),
            None => active.push((m.observed_at, h, l)),
        }
    }
    active.sort_by(|a, b| (a.0, a.1).cmp(&(b.0, b.1)));
    active.dedup_by(|a, b| a.1 == b.1);
    ignored.sort();

    // Feature effects, per block, from every active feature layer (OR, so order-free).
    let mut feat: std::collections::BTreeMap<BlockId, Vec<u16>> = Default::default();
    for (_, _, l) in active.iter().filter(|a| a.2.m.kind == Kind::Feature) {
        for f in &l.features {
            if f.conf < prm.feature_min_conf {
                continue;
            }
            let bits = feature_bits(f.class);
            if bits == 0 {
                continue;
            }
            for seg in f.pts.windows(2) {
                supercover(seg[0], seg[1], prm.spacing, &mut |i, j| {
                    let (b, p) = block_of(i, j);
                    feat.entry(b).or_insert_with(|| vec![0; BLOCK_POSTS])[p] |= bits;
                });
            }
        }
    }
    let grids: Vec<(usize, &Layer)> =
        active.iter().enumerate().filter(|(_, a)| a.2.m.kind != Kind::Feature).map(|(k, a)| (k + 1, a.2)).collect();

    let pick: Vec<usize> = match only {
        None => (0..base.blocks.len()).collect(),
        Some(set) => (0..base.blocks.len()).filter(|&k| set.binary_search(&base.blocks[k]).is_ok()).collect(),
    };
    let s = prm.spacing;
    let out: Vec<(BlockId, FusedBlock)> = pick
        .par_iter()
        .map(|&k| {
            let id = base.blocks[k];
            let cls = &base.class[k * BLOCK_POSTS..(k + 1) * BLOCK_POSTS];
            let cf = &base.conf[k * BLOCK_POSTS..(k + 1) * BLOCK_POSTS];
            let mut fb = FusedBlock {
                p: (0..BLOCK_POSTS).map(|q| base.prior[cls[q] as usize * 2 + (cf[q] & 1) as usize]).collect(),
                haz: vec![[0; NHAZ]; BLOCK_POSTS],
                lev: vec![[(0, 0); NLEV]; BLOCK_POSTS],
                mask: feat.get(&id).cloned().unwrap_or_else(|| vec![0; BLOCK_POSTS]),
                cert: vec![0; BLOCK_POSTS],
                src: vec![0; BLOCK_POSTS],
            };
            for &(idx, l) in &grids {
                let Ok(bk) = l.blocks.binary_search_by_key(&id.key(), |b| b.id.key()) else { continue };
                let lb = &l.blocks[bk];
                let src = idx.min(254) as u8;
                for q in 0..BLOCK_POSTS {
                    if !lb.has(q) {
                        continue;
                    }
                    let w = w_eff(lb.conf[q], &l.m, prm.t);
                    if w == 0 {
                        continue;
                    }
                    match l.m.kind {
                        Kind::Class => {
                            let mut qv = [0u8; NCLASS];
                            for c in 0..NCLASS {
                                qv[c] = lb.vals[c][q] as u8;
                            }
                            blend_class(&mut fb.p[q], &qv, w);
                        }
                        Kind::Hazard => {
                            for (ch, &code) in l.m.channels.iter().enumerate() {
                                if let Some(sl) = hazard_slot(code) {
                                    fb.haz[q][sl] = blend_hazard(fb.haz[q][sl], lb.vals[ch][q] as u8, w);
                                }
                            }
                        }
                        Kind::Level => {
                            if w < prm.level_min_w {
                                continue;
                            }
                            for (ch, &code) in l.m.channels.iter().enumerate() {
                                if let Some(sl) = level_slot(code) {
                                    fb.lev[q][sl] = (lb.vals[ch][q], w);
                                }
                            }
                        }
                        Kind::Feature => unreachable!(),
                    }
                    fb.src[q] = src;
                }
            }
            // Team overlay items, in the order given (the overlay set's own order), last.
            for it in overlay {
                for q in 0..BLOCK_POSTS {
                    let i = id.i0() + (q / BLOCK) as i64;
                    let j = id.j0() + (q % BLOCK) as i64;
                    let (la, lo) = (i * s, j * s);
                    if la < it.lat0 || la >= it.lat1 || lo < it.lon0 || lo >= it.lon1 {
                        continue;
                    }
                    match it.effect {
                        Effect::Block => fb.mask[q] |= M_TEAM_BLOCK,
                        Effect::Clear => {
                            fb.haz[q] = [0; NHAZ];
                            fb.lev[q] = [(0, 0); NLEV];
                            fb.mask[q] &= !(M_DEBRIS | M_FALLEN | M_WASHOUT | M_TEAM_BLOCK);
                        }
                        Effect::SetHazard(code, v) => {
                            if let Some(sl) = hazard_slot(code) {
                                fb.haz[q][sl] = v;
                            }
                        }
                    }
                    fb.src[q] = 255;
                }
            }
            for q in 0..BLOCK_POSTS {
                let mx = *fb.p[q].iter().max().unwrap() as u32;
                fb.cert[q] = ((mx * 255 + S / 2) / S) as u8;
            }
            (id, fb)
        })
        .collect();

    let mut h = blake3::Hasher::new();
    for (id, fb) in &out {
        fb.hash_into(*id, &mut h);
    }
    let rec = Record {
        t: prm.t,
        base: base.hash,
        prior: base.prior_hash(),
        applied: active.iter().map(|a| Applied { hash: a.1, kind: a.2.m.kind, observed_at: a.0 }).collect(),
        ignored,
        overlay: overlay_hash(overlay),
        fused: *h.finalize().as_bytes(),
    };
    (out, rec)
}

pub fn hash_blocks(v: &[(BlockId, FusedBlock)]) -> [u8; 32] {
    let mut h = blake3::Hasher::new();
    for (id, fb) in v {
        fb.hash_into(*id, &mut h);
    }
    *h.finalize().as_bytes()
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
    fn blend_rules() {
        let mut q = [0u8; NCLASS];
        q[3] = 200;
        q[1] = 55;
        // w = 0: no-op
        let mut p = onehot(1);
        blend_class(&mut p, &q, 0);
        assert_eq!(p, onehot(1));
        // w = 255, no keep: replaced by Q exactly (scaled)
        blend_class(&mut p, &q, 255);
        assert_eq!(p.iter().map(|&v| v as u32).sum::<u32>(), S);
        // 200 S / 255 = 3212.55 and 55 S / 255 = 883.45: the leftover unit goes to class 3
        assert_eq!((p[3], p[1]), (3213, 883));
        // all keep: no change whatever w is
        let mut keep = [0u8; NCLASS];
        keep[0] = 255;
        let before = p;
        blend_class(&mut p, &keep, 255);
        assert_eq!(p, before);
        // hazards
        assert_eq!(blend_hazard(10, 200, 0), 10);
        assert_eq!(blend_hazard(10, 200, 255), 200);
        assert_eq!(blend_hazard(0, 255, 128), 128);
    }

    #[test]
    fn supercover_cells() {
        let s = 120;
        // 1" = 1/3600 degree = 2777.78 E7 units; a horizontal line across three cells
        let mut v = Vec::new();
        supercover((0, 0), (0, 7000), s, &mut |i, j| v.push((i, j)));
        assert_eq!(v, vec![(0, 0), (0, 1), (0, 2), (0, 3)]);
        // reversed direction visits the same cells
        let mut r = Vec::new();
        supercover((0, 7000), (0, 0), s, &mut |i, j| r.push((i, j)));
        r.reverse();
        assert_eq!(r, v);
        // a diagonal through an exact corner: E7 1388.88 is not a whole number, so pick a corner
        // that is: post cell edges sit at (k + 1/2)" = (2k + 1) * 50000 / 36 E7, whole at k = 17
        // (35 * 50000 / 36 = 48611.1, no); use a symmetric line through the origin cell instead.
        let mut d = Vec::new();
        supercover((-5000, -5000), (5000, 5000), s, &mut |i, j| d.push((i, j)));
        assert_eq!(d.first(), Some(&(-2, -2)));
        assert_eq!(d.last(), Some(&(2, 2)));
        assert!(d.windows(2).all(|w| (w[1].0 - w[0].0).abs() <= 1 && (w[1].1 - w[0].1).abs() <= 1));
    }
}

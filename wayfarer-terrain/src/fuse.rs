//! Fusion of the base with terrain layers and the team overlay into the per-post state the
//! incident surface is costed from (`docs/design/terrain-layers.md`, contract v0; the overlay follows the incident surface
//! contract v1 draft).
//!
//! Per post, in this order:
//! 1. The base: the land-cover code and confidence bit, turned into a class distribution by
//!    the [`Prior`] (one-hot by default, so a surface with no layers costs the base as is).
//! 2. Grid layers active at the incident time T, oldest first by (observed_at, content hash):
//!    class layers blend the distribution as `P <- (1 - w) P + w (Q + q_keep P)`, hazard layers
//!    blend each probability as `h <- (1 - w) h + w q`, level layers replace the value when
//!    `w >= level_min_w`. A newer confident layer wins, a weak one blends, and w = 0 is a no-op.
//! 3. Feature layers: lines at or above the confidence threshold OR their effect bits into the
//!    posts they cross (exact supercover on the lattice).
//! 4. Team overlay items, last, in (observed time, item bytes) order: they beat every raster
//!    and feature, and a later observation beats an earlier one.
//!
//! All arithmetic is integer with one rounding rule per step, and each post depends only on its
//! own inputs, so any block order, thread count or incremental re-fusion gives the same bytes.

use crate::lattice::{BLOCK, BLOCK_POSTS, BlockId, TICKS_PER_DEG, ceil_div, ticks_of_deg};
use crate::layer::{
    self, FEAT_DEBRIS, FEAT_FALLEN, FEAT_PASSAGE, FEAT_WASHOUT, HAZARDS, Kind, LEVELS,
    LICENCE_RESEARCH, Layer, NCLASS, NHAZ, NLEV,
};
use crate::terrain::Terrain;
use crate::{Error, Result};
use rayon::prelude::*;
use std::collections::BTreeMap;

/// The fused class distribution sums to this.
pub const S: u32 = 1 << 12;

// Effect bits a feature or an overlay item sets in the fused mask.
/// A format-4 `BarrierLines` class 1-16 seen by a survey.
pub const M_BARRIER: u16 = 1 << 0;
pub const M_DEBRIS: u16 = 1 << 1;
pub const M_FALLEN: u16 = 1 << 2;
pub const M_WASHOUT: u16 = 1 << 3;
pub const M_PASSAGE: u16 = 1 << 4;
pub const M_TEAM_BLOCK: u16 = 1 << 5;

/// The effect bit of a feature class (0 = no effect).
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

/// A layer with its content hash, checked once when it was read or built.
#[derive(Clone, Debug)]
pub struct HashedLayer {
    layer: Layer,
    hash: [u8; 32],
}

impl HashedLayer {
    /// Canonicalises (validates) the layer and hashes it.
    pub fn new(mut layer: Layer) -> Result<HashedLayer> {
        layer.canonicalise()?;
        let hash = layer.content_hash();
        Ok(HashedLayer { layer, hash })
    }
    /// Reads and checks a layer file.
    pub fn read(bytes: &[u8]) -> Result<HashedLayer> {
        let layer = Layer::read(bytes)?;
        let hash = layer.content_hash();
        Ok(HashedLayer { layer, hash })
    }
    pub fn layer(&self) -> &Layer {
        &self.layer
    }
    pub fn hash(&self) -> [u8; 32] {
        self.hash
    }
}

/// The land-cover prior: the class distribution each (land-cover code, confidence bit) of the
/// base starts from before any layer is fused, at index `code * 2 + bit`; each sums to [`S`].
/// A fixed input of the incident (one-hot by default), not a previous surface.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Prior {
    table: Vec<[u16; NCLASS]>,
}

impl Default for Prior {
    /// One-hot: the base class with certainty, whatever its confidence bit.
    fn default() -> Self {
        let mut table = vec![[0u16; NCLASS]; NCLASS * 2];
        for (i, d) in table.iter_mut().enumerate() {
            d[i / 2] = S as u16;
        }
        Prior { table }
    }
}

impl Prior {
    pub fn new(table: Vec<[u16; NCLASS]>) -> Result<Prior> {
        if table.len() != NCLASS * 2
            || table
                .iter()
                .any(|d| d.iter().map(|&v| v as u32).sum::<u32>() != S)
        {
            return Err(Error::invalid(
                "a prior has 32 distributions summing to 4096",
            ));
        }
        Ok(Prior { table })
    }
    pub fn get(&self, code: u8, bit: u8) -> [u16; NCLASS] {
        self.table[(code as usize & 15) * 2 + (bit & 1) as usize]
    }
    pub fn hash(&self) -> [u8; 32] {
        let mut h = blake3::Hasher::new();
        for d in &self.table {
            for v in d {
                h.update(&v.to_le_bytes());
            }
        }
        *h.finalize().as_bytes()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Effect {
    /// The team closed the ground.
    Block,
    /// The team walked it: hazards, levels, obstructions and team blocks cleared.
    Clear,
    /// The team set a hazard (code, probability in 1/255).
    SetHazard(u16, u8),
}

/// A team overlay item: an effect over a box of post positions in ticks, `[lat0, lat1) x
/// [lon0, lon1)`, observed at a time (UTC seconds).
///
/// An overlay is a set, not a list: fusion applies its items in (observed time, item bytes)
/// order, so items that arrive in any order from any number of devices give the same surface.
/// Where two items cover a post, the one observed later wins.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct OverlayItem {
    pub lat0: i64,
    pub lat1: i64,
    pub lon0: i64,
    pub lon1: i64,
    pub observed_at: i64,
    pub effect: Effect,
}

impl OverlayItem {
    /// An item from a box in f64 degrees, its corners in any order. Each edge goes onto the
    /// lattice by [`lattice::ticks_of_deg`](crate::lattice::ticks_of_deg), the one rounding
    /// rule, so every device that holds the same shape gets the same item.
    pub fn from_degrees(
        lat: (f64, f64),
        lon: (f64, f64),
        observed_at: i64,
        effect: Effect,
    ) -> Result<OverlayItem> {
        let t =
            |d: f64| ticks_of_deg(d).ok_or_else(|| Error::invalid("overlay corner outside ±180°"));
        let (a, b, c, d) = (t(lat.0)?, t(lat.1)?, t(lon.0)?, t(lon.1)?);
        if a.abs() > 90 * TICKS_PER_DEG || b.abs() > 90 * TICKS_PER_DEG {
            return Err(Error::invalid("overlay latitude outside ±90°"));
        }
        let it = OverlayItem {
            lat0: a.min(b),
            lat1: a.max(b),
            lon0: c.min(d),
            lon1: c.max(d),
            observed_at,
            effect,
        };
        if it.lat0 == it.lat1 || it.lon0 == it.lon1 {
            return Err(Error::invalid("an overlay box has no area on the lattice"));
        }
        Ok(it)
    }

    /// The item's canonical bytes: observed time, box, effect.
    pub fn bytes(&self) -> [u8; 44] {
        let mut o = [0u8; 44];
        o[..8].copy_from_slice(&self.observed_at.to_le_bytes());
        for (k, v) in [self.lat0, self.lat1, self.lon0, self.lon1]
            .iter()
            .enumerate()
        {
            o[8 + 8 * k..16 + 8 * k].copy_from_slice(&v.to_le_bytes());
        }
        let (e, c, v) = match self.effect {
            Effect::Block => (1u8, 0u16, 0u8),
            Effect::Clear => (2, 0, 0),
            Effect::SetHazard(c, v) => (3, c, v),
        };
        o[40] = e;
        o[41..43].copy_from_slice(&c.to_le_bytes());
        o[43] = v;
        o
    }
}

/// The overlay as fusion applies it: sorted by (observed time, item bytes), duplicates removed.
/// Any order of the same items gives the same result.
pub fn canonical_overlay(items: &[OverlayItem]) -> Vec<OverlayItem> {
    // By observed time as a number first: the bytes are little-endian, so they alone would not
    // sort by time.
    let mut v: Vec<(i64, [u8; 44], OverlayItem)> = items
        .iter()
        .map(|it| (it.observed_at, it.bytes(), *it))
        .collect();
    v.sort_by_key(|a| (a.0, a.1));
    v.dedup_by(|a, b| a.1 == b.1);
    v.into_iter().map(|x| x.2).collect()
}

/// Hash of an overlay set: of its canonical form, so the items' order does not matter.
pub fn overlay_hash(items: &[OverlayItem]) -> [u8; 32] {
    let mut h = blake3::Hasher::new();
    h.update(b"wis1/overlay");
    let c = canonical_overlay(items);
    h.update(&(c.len() as u64).to_le_bytes());
    for it in &c {
        h.update(&it.bytes());
    }
    *h.finalize().as_bytes()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Params {
    /// The incident epoch, UTC seconds.
    pub t: i64,
    pub allow_research: bool,
    /// Features below this confidence (1/255) have no effect.
    pub feature_min_conf: u8,
    /// Level values below this confidence are not applied.
    pub level_min_w: u8,
}

/// Why a layer was left out of fusion (the code is stored in the surface header).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum LeftOut {
    OutsideValidity = 1,
    ResearchOnly = 2,
    Spacing = 3,
    NoWeight = 4,
}

impl LeftOut {
    pub fn code(self) -> u8 {
        self as u8
    }
    pub fn from_code(c: u8) -> Option<LeftOut> {
        Some(match c {
            1 => LeftOut::OutsideValidity,
            2 => LeftOut::ResearchOnly,
            3 => LeftOut::Spacing,
            4 => LeftOut::NoWeight,
            _ => return None,
        })
    }
    pub fn reason(self) -> &'static str {
        match self {
            LeftOut::OutsideValidity => "outside its validity window at T",
            LeftOut::ResearchOnly => "research-only licence",
            LeftOut::Spacing => "spacing differs from the surface (resample at import)",
            LeftOut::NoWeight => "no confidence anywhere at T",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FusedBlock {
    pub p: Vec<[u16; NCLASS]>,
    pub haz: Vec<[u8; NHAZ]>,
    /// (value, confidence) per level slot.
    pub lev: Vec<[(u16, u8); NLEV]>,
    /// Effect bits (`M_*`).
    pub mask: Vec<u16>,
    /// Largest class probability, in 1/255.
    pub cert: Vec<u8>,
    /// 0 base, k = the k-th applied grid layer (1-based, saturating at 254), 255 team overlay.
    pub src: Vec<u8>,
}

impl FusedBlock {
    /// Canonical bytes: key, then per post P (16 u16), hazards (8 u8), levels (3 × u16 value +
    /// u8 confidence), effect bits u16, certainty u8, source u8; little-endian.
    pub fn bytes(&self, id: BlockId) -> Vec<u8> {
        let mut buf = Vec::with_capacity(BLOCK_POSTS * 48 + 8);
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

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Applied {
    pub hash: [u8; 32],
    pub kind: Kind,
    pub observed_at: i64,
    pub valid_until: i64,
}

/// What a fusion used: the layers applied in fusion order, those left out and why (sorted by
/// hash), the prior, the overlay, and `fused`: blake3 over each fused block's hash
/// (blake3 of [`FusedBlock::bytes`]) in terrain order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Record {
    pub t: i64,
    pub terrain: [u8; 32],
    pub prior: [u8; 32],
    pub applied: Vec<Applied>,
    pub left_out: Vec<([u8; 32], LeftOut)>,
    pub overlay: [u8; 32],
    pub fused: [u8; 32],
}

/// Effective confidence at T: the stored w, faded linearly to 0 over [fade_from, valid_until).
pub fn w_eff(w: u8, m: &layer::Manifest, t: i64) -> u8 {
    if t < m.fade_from {
        return w;
    }
    let num = (m.valid_until - t) as i128;
    let den = (m.valid_until - m.fade_from) as i128;
    if den <= 0 || num <= 0 {
        return 0;
    }
    (w as i128 * num / den) as u8
}

/// `P <- (1 - w) P + w (Q + q_keep P)`, exact in integers, then largest-remainder to S.
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
    layer::normalise(&raw, S as u64, &mut out);
    for k in 0..NCLASS {
        p[k] = out[k] as u16;
    }
}

pub fn blend_hazard(h: u8, q: u8, w: u8) -> u8 {
    let (h, q, w) = (h as u32, q as u32, w as u32);
    (((255 - w) * h + w * q + 127) / 255) as u8
}

/// The slot of a hazard code.
pub fn hazard_slot(code: u16) -> Option<usize> {
    HAZARDS.iter().position(|h| h.0 == code)
}
/// The slot of a level code.
pub fn level_slot(code: u16) -> Option<usize> {
    LEVELS.iter().position(|h| h.0 == code)
}

/// Exact supercover of a segment over the lattice cells: calls `visit(i, j)` for every post whose
/// cell `[p - s/2, p + s/2)` the segment touches (both neighbours at an exact corner crossing).
/// Points are E7 degrees; E7 × 27 and ticks × 625 share a unit.
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
    while (cy, cx) != (ey, ex) {
        // distance to the next boundary along each axis, compared as ny/|dy| vs nx/|dx|
        let nx = if sx > 0 {
            (cx + 1) * c - ax
        } else {
            ax - cx * c
        };
        let ny = if sy > 0 {
            (cy + 1) * c - ay
        } else {
            ay - cy * c
        };
        let tx = if dx == 0 {
            None
        } else {
            Some(nx as i128 * dy.abs() as i128)
        };
        let ty = if dy == 0 {
            None
        } else {
            Some(ny as i128 * dx.abs() as i128)
        };
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

fn contributes(l: &Layer, prm: &Params) -> bool {
    match l.m.kind {
        Kind::Feature => l
            .features
            .iter()
            .any(|f| f.conf >= prm.feature_min_conf && feature_bits(f.class) != 0),
        Kind::Level => l.blocks.iter().any(|b| {
            (0..BLOCK_POSTS).any(|q| b.has(q) && w_eff(b.conf[q], &l.m, prm.t) >= prm.level_min_w)
        }),
        _ => l
            .blocks
            .iter()
            .any(|b| (0..BLOCK_POSTS).any(|q| b.has(q) && w_eff(b.conf[q], &l.m, prm.t) > 0)),
    }
}

/// Why a layer would be left out at T, or None if it applies.
pub fn left_out(l: &Layer, spacing: i64, prm: &Params) -> Option<LeftOut> {
    let m = &l.m;
    if !(m.valid_from <= prm.t && prm.t < m.valid_until) {
        Some(LeftOut::OutsideValidity)
    } else if m.licence == LICENCE_RESEARCH && !prm.allow_research {
        Some(LeftOut::ResearchOnly)
    } else if m.s_lat as i64 != spacing || m.s_lon as i64 != spacing {
        Some(LeftOut::Spacing)
    } else if !contributes(l, prm) {
        // A layer with no weight anywhere at T changes nothing; leaving it out of the record
        // keeps the source indexes of the others stable.
        Some(LeftOut::NoWeight)
    } else {
        None
    }
}

/// Fuses `only` (a sorted subset of the terrain's blocks, or all of them when None). Returns
/// the blocks in terrain order and the record of what was used.
pub fn fuse(
    ter: &Terrain,
    prior: &Prior,
    layers: &[&HashedLayer],
    overlay: &[OverlayItem],
    prm: &Params,
    only: Option<&[BlockId]>,
) -> (Vec<(BlockId, FusedBlock)>, Record) {
    fuse_with(ter, prior, layers, overlay, prm, only, |_, _, fb| fb)
}

/// [`fuse`], handing each fused block to `f` (with the block's terrain index and the layers
/// applied) as soon as it is made, so only the blocks in flight hold fusion state. Returns
/// `f`'s results in terrain order.
#[allow(clippy::too_many_arguments)]
pub fn fuse_with<T: Send>(
    ter: &Terrain,
    prior: &Prior,
    layers: &[&HashedLayer],
    overlay: &[OverlayItem],
    prm: &Params,
    only: Option<&[BlockId]>,
    f: impl Fn(usize, &[Applied], FusedBlock) -> T + Sync,
) -> (Vec<(BlockId, T)>, Record) {
    let spacing = ter.spacing().ticks();
    let overlay = &canonical_overlay(overlay)[..];
    // Which layers apply, in fusion order.
    let mut active: Vec<(i64, [u8; 32], &Layer)> = Vec::new();
    let mut ignored = Vec::new();
    for hl in layers {
        let l = hl.layer();
        match left_out(l, spacing, prm) {
            Some(r) => ignored.push((hl.hash(), r)),
            None => active.push((l.m.observed_at, hl.hash(), l)),
        }
    }
    active.sort_by_key(|a| (a.0, a.1));
    active.dedup_by(|a, b| a.1 == b.1);
    ignored.sort();
    ignored.dedup();

    // Feature effects, per block, from every active feature layer (OR, so order-free).
    let mut feat: BTreeMap<BlockId, Vec<u16>> = BTreeMap::new();
    for (_, _, l) in active.iter().filter(|a| a.2.m.kind == Kind::Feature) {
        for f in &l.features {
            let bits = feature_bits(f.class);
            if f.conf < prm.feature_min_conf || bits == 0 {
                continue;
            }
            for seg in f.pts.windows(2) {
                supercover(seg[0], seg[1], spacing, &mut |i, j| {
                    let (b, p) = BlockId::of_post(i, j);
                    feat.entry(b).or_insert_with(|| vec![0; BLOCK_POSTS])[p] |= bits;
                });
            }
        }
    }
    let grids: Vec<(usize, &Layer)> = active
        .iter()
        .enumerate()
        .filter(|(_, a)| a.2.m.kind != Kind::Feature)
        .map(|(k, a)| (k + 1, a.2))
        .collect();

    let blocks = ter.blocks();
    let pick: Vec<usize> = match only {
        None => (0..blocks.len()).collect(),
        Some(set) => (0..blocks.len())
            .filter(|&k| set.binary_search(&blocks[k]).is_ok())
            .collect(),
    };
    let applied: Vec<Applied> = active
        .iter()
        .map(|a| Applied {
            hash: a.1,
            kind: a.2.m.kind,
            observed_at: a.0,
            valid_until: a.2.m.valid_until,
        })
        .collect();
    let done: Vec<(BlockId, [u8; 32], T)> = pick
        .par_iter()
        .map(|&k| {
            let id = blocks[k];
            let fb = fuse_block(
                id,
                ter.cover(k),
                ter.conf(k),
                prior,
                &grids,
                feat.get(&id),
                overlay,
                prm,
                spacing,
            );
            let hb = *blake3::hash(&fb.bytes(id)).as_bytes();
            (id, hb, f(k, &applied, fb))
        })
        .collect();

    let mut h = blake3::Hasher::new();
    let mut out = Vec::with_capacity(done.len());
    for (id, hb, t) in done {
        h.update(&hb);
        out.push((id, t));
    }
    let rec = Record {
        t: prm.t,
        terrain: ter.hash(),
        prior: prior.hash(),
        applied,
        left_out: ignored,
        overlay: overlay_hash(overlay),
        fused: *h.finalize().as_bytes(),
    };
    (out, rec)
}

fn fuse_block(
    id: BlockId,
    cls: &[u8],
    cf: &[u8],
    prior: &Prior,
    grids: &[(usize, &Layer)],
    feat: Option<&Vec<u16>>,
    overlay: &[OverlayItem],
    prm: &Params,
    s: i64,
) -> FusedBlock {
    let mut fb = FusedBlock {
        p: (0..BLOCK_POSTS).map(|q| prior.get(cls[q], cf[q])).collect(),
        haz: vec![[0; NHAZ]; BLOCK_POSTS],
        lev: vec![[(0, 0); NLEV]; BLOCK_POSTS],
        mask: feat.cloned().unwrap_or_else(|| vec![0; BLOCK_POSTS]),
        cert: vec![0; BLOCK_POSTS],
        src: vec![0; BLOCK_POSTS],
    };
    for &(idx, l) in grids {
        let Ok(bk) = l.blocks.binary_search_by_key(&id.key(), |b| b.id.key()) else {
            continue;
        };
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
                    for (c, v) in qv.iter_mut().enumerate() {
                        *v = lb.vals[c][q] as u8;
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
                Kind::Feature => unreachable!("feature layers are not grids"),
            }
            fb.src[q] = src;
        }
    }
    // Team overlay items, last, in canonical order: a later observation wins.
    let rows = |lo: i64, hi: i64, first: i64| {
        // Posts p of this block with lo <= p * s < hi, as offsets 0..64 into the block.
        let a = (ceil_div(lo, s) - first).max(0);
        let b = (ceil_div(hi, s) - first).min(BLOCK as i64);
        a as usize..b.max(a) as usize
    };
    for it in overlay {
        for r in rows(it.lat0, it.lat1, id.i0()) {
            for c in rows(it.lon0, it.lon1, id.j0()) {
                let q = r * BLOCK + c;
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
    }
    for q in 0..BLOCK_POSTS {
        let mx = *fb.p[q].iter().max().expect("16 classes") as u32;
        fb.cert[q] = ((mx * 255 + S / 2) / S) as u8;
    }
    fb
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
        let mut p = onehot(1);
        blend_class(&mut p, &q, 0);
        assert_eq!(p, onehot(1));
        blend_class(&mut p, &q, 255);
        assert_eq!(p.iter().map(|&v| v as u32).sum::<u32>(), S);
        // 200 S / 255 = 3212.55 and 55 S / 255 = 883.45: the leftover unit goes to class 3
        assert_eq!((p[3], p[1]), (3213, 883));
        let mut keep = [0u8; NCLASS];
        keep[0] = 255;
        let before = p;
        blend_class(&mut p, &keep, 255);
        assert_eq!(p, before);
        assert_eq!(blend_hazard(10, 200, 0), 10);
        assert_eq!(blend_hazard(10, 200, 255), 200);
        assert_eq!(blend_hazard(0, 255, 128), 128);
    }

    fn test_manifest(kind: Kind, channels: Vec<u16>) -> layer::Manifest {
        layer::Manifest {
            kind,
            licence: 0,
            s_lat: 120,
            s_lon: 120,
            observed_at: 1_000,
            valid_from: 0,
            valid_until: 10_000,
            fade_from: 10_000,
            channels,
            model_id: "test".into(),
            model_version: "0".into(),
            weights: [1; 32],
            dataset_manifest: [2; 32],
            eval_set: [3; 32],
            ece_permille: 0xFFFF,
            backend: "none".into(),
            attribution: String::new(),
            inputs: vec![],
            parents: vec![],
        }
    }

    #[test]
    fn fade() {
        let mut m = test_manifest(Kind::Hazard, vec![1]);
        (m.valid_from, m.fade_from, m.valid_until) = (0, 100, 200);
        assert_eq!(w_eff(200, &m, 50), 200);
        assert_eq!(w_eff(200, &m, 150), 100);
        assert_eq!(w_eff(200, &m, 200), 0);
    }

    #[test]
    fn supercover_cells() {
        let s = 120;
        let mut v = Vec::new();
        supercover((0, 0), (0, 7000), s, &mut |i, j| v.push((i, j)));
        assert_eq!(v, vec![(0, 0), (0, 1), (0, 2), (0, 3)]);
        let mut r = Vec::new();
        supercover((0, 7000), (0, 0), s, &mut |i, j| r.push((i, j)));
        r.reverse();
        assert_eq!(r, v);
        let mut d = Vec::new();
        supercover((-5000, -5000), (5000, 5000), s, &mut |i, j| d.push((i, j)));
        assert_eq!(d.first(), Some(&(-2, -2)));
        assert_eq!(d.last(), Some(&(2, 2)));
        assert!(
            d.windows(2)
                .all(|w| (w[1].0 - w[0].0).abs() <= 1 && (w[1].1 - w[0].1).abs() <= 1)
        );
    }

    #[test]
    fn priors() {
        let p = Prior::default();
        assert_eq!(p.get(3, 1), onehot(3));
        assert!(Prior::new(vec![[0; NCLASS]; 32]).is_err());
        assert_eq!(Prior::new(p.table.clone()).unwrap(), p);
    }
}

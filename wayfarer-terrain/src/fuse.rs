//! Fusion of the base with terrain layers and the team overlay into the per-post state the
//! incident surface is costed from (`docs/design/terrain-layers.md` section 8; the overlay is
//! `docs/design/incident-surface.md` section 5, both contract v1).
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
use std::collections::{BTreeMap, BTreeSet};

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

/// Vertices an overlay item may hold, over all its rings.
pub const MAX_OVERLAY_VERTICES: usize = 65_536;

/// The ground an overlay item covers, in ticks, `(lat, lon)` per vertex. Each shape has one
/// rasterising rule, exact in integers, so every device marks the same posts.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum Shape {
    /// The posts p with `lat0 <= p.lat < lat1` and `lon0 <= p.lon < lon1`.
    Box {
        lat0: i64,
        lat1: i64,
        lon0: i64,
        lon1: i64,
    },
    /// One or more closed rings (the last vertex joins the first), filled even-odd: a post is
    /// inside when a ray from it towards +lon crosses the rings an odd number of times,
    /// counting an edge where `min(lat) <= p.lat < max(lat)` and its crossing is at or west of
    /// the post. A ring inside another is a hole. A box as a ring marks the posts of
    /// [`Shape::Box`].
    Polygon(Vec<Vec<(i64, i64)>>),
    /// A polyline: the posts whose cell `[p - s/2, p + s/2)` a segment touches, the exact
    /// supercover feature lines use.
    Line(Vec<(i64, i64)>),
}

/// A team overlay item: an effect over a shape of post positions, observed at a time (UTC
/// seconds).
///
/// An overlay is a set, not a list: fusion applies its items in (observed time, item bytes)
/// order, so items that arrive in any order from any number of devices give the same surface.
/// Where two items cover a post, the one observed later wins.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct OverlayItem {
    pub shape: Shape,
    pub observed_at: i64,
    pub effect: Effect,
}

fn tick_of(d: f64, max_deg: i64, what: &str) -> Result<i64> {
    let t = ticks_of_deg(d).ok_or_else(|| Error::invalid(format!("{what} outside ±180°")))?;
    if t.abs() > max_deg * TICKS_PER_DEG {
        return Err(Error::invalid(format!("{what} outside ±{max_deg}°")));
    }
    Ok(t)
}

fn vertex_of(p: (f64, f64)) -> Result<(i64, i64)> {
    Ok((
        tick_of(p.0, 90, "overlay latitude")?,
        tick_of(p.1, 180, "overlay longitude")?,
    ))
}

impl OverlayItem {
    /// A box item from f64 degrees, its corners in any order. Each edge goes onto the lattice
    /// by [`lattice::ticks_of_deg`](crate::lattice::ticks_of_deg), the one rounding rule, so
    /// every device that holds the same shape gets the same item.
    pub fn from_degrees(
        lat: (f64, f64),
        lon: (f64, f64),
        observed_at: i64,
        effect: Effect,
    ) -> Result<OverlayItem> {
        let (a, c) = vertex_of((lat.0, lon.0))?;
        let (b, d) = vertex_of((lat.1, lon.1))?;
        let it = OverlayItem {
            shape: Shape::Box {
                lat0: a.min(b),
                lat1: a.max(b),
                lon0: c.min(d),
                lon1: c.max(d),
            },
            observed_at,
            effect,
        };
        it.check()?;
        Ok(it)
    }

    /// A polygon item from rings of `(lat, lon)` f64 degrees, each vertex rounded by
    /// [`lattice::ticks_of_deg`](crate::lattice::ticks_of_deg). Rings are closed implicitly.
    pub fn polygon_from_degrees(
        rings: &[Vec<(f64, f64)>],
        observed_at: i64,
        effect: Effect,
    ) -> Result<OverlayItem> {
        let rings = rings
            .iter()
            .map(|r| r.iter().map(|&p| vertex_of(p)).collect::<Result<Vec<_>>>())
            .collect::<Result<Vec<_>>>()?;
        let it = OverlayItem {
            shape: Shape::Polygon(rings),
            observed_at,
            effect,
        };
        it.check()?;
        Ok(it)
    }

    /// A line item from a polyline of `(lat, lon)` f64 degrees, each vertex rounded by
    /// [`lattice::ticks_of_deg`](crate::lattice::ticks_of_deg).
    pub fn line_from_degrees(
        pts: &[(f64, f64)],
        observed_at: i64,
        effect: Effect,
    ) -> Result<OverlayItem> {
        let pts = pts
            .iter()
            .map(|&p| vertex_of(p))
            .collect::<Result<Vec<_>>>()?;
        let it = OverlayItem {
            shape: Shape::Line(pts),
            observed_at,
            effect,
        };
        it.check()?;
        Ok(it)
    }

    /// Checks the shape: positions within ±90° latitude and ±180° longitude, a box with area,
    /// rings of at least 3 vertices, a line of at least 1, and at most
    /// [`MAX_OVERLAY_VERTICES`] in all.
    pub fn check(&self) -> Result<()> {
        let ok = |&(la, lo): &(i64, i64)| {
            la.abs() <= 90 * TICKS_PER_DEG && lo.abs() <= 180 * TICKS_PER_DEG
        };
        match &self.shape {
            Shape::Box {
                lat0,
                lat1,
                lon0,
                lon1,
            } => {
                if !ok(&(*lat0, *lon0)) || !ok(&(*lat1, *lon1)) {
                    return Err(Error::invalid("an overlay box lies outside the globe"));
                }
                if lat0 >= lat1 || lon0 >= lon1 {
                    return Err(Error::invalid("an overlay box has no area on the lattice"));
                }
            }
            Shape::Polygon(rings) => {
                if rings.is_empty() || rings.iter().any(|r| r.len() < 3) {
                    return Err(Error::invalid(
                        "an overlay polygon has rings of 3 or more vertices",
                    ));
                }
                let n: usize = rings.iter().map(Vec::len).sum();
                if n > MAX_OVERLAY_VERTICES {
                    return Err(Error::invalid("an overlay item has too many vertices"));
                }
                if !rings.iter().flatten().all(ok) {
                    return Err(Error::invalid("an overlay polygon lies outside the globe"));
                }
            }
            Shape::Line(pts) => {
                if pts.is_empty() || pts.len() > MAX_OVERLAY_VERTICES {
                    return Err(Error::invalid("an overlay line has 1 to 65,536 vertices"));
                }
                if !pts.iter().all(ok) {
                    return Err(Error::invalid("an overlay line lies outside the globe"));
                }
            }
        }
        Ok(())
    }

    /// The item's canonical bytes: observed time (i64), effect (tag u8, hazard code u16,
    /// probability u8), shape tag u8 (1 box, 2 polygon, 3 line), then the shape: a box's
    /// lat0, lat1, lon0, lon1 (i64); a polygon's ring count (u32) and each ring as a vertex
    /// count (u32) and its vertices; a line's vertex count (u32) and vertices. A vertex is lat,
    /// lon (i64). All little-endian. The encoding delimits itself, so items concatenate.
    pub fn bytes(&self) -> Vec<u8> {
        let mut o = Vec::with_capacity(46);
        o.extend_from_slice(&self.observed_at.to_le_bytes());
        let (e, c, v) = match self.effect {
            Effect::Block => (1u8, 0u16, 0u8),
            Effect::Clear => (2, 0, 0),
            Effect::SetHazard(c, v) => (3, c, v),
        };
        o.push(e);
        o.extend_from_slice(&c.to_le_bytes());
        o.push(v);
        let pts = |o: &mut Vec<u8>, p: &[(i64, i64)]| {
            o.extend_from_slice(&(p.len() as u32).to_le_bytes());
            for (la, lo) in p {
                o.extend_from_slice(&la.to_le_bytes());
                o.extend_from_slice(&lo.to_le_bytes());
            }
        };
        match &self.shape {
            Shape::Box {
                lat0,
                lat1,
                lon0,
                lon1,
            } => {
                o.push(1);
                for x in [lat0, lat1, lon0, lon1] {
                    o.extend_from_slice(&x.to_le_bytes());
                }
            }
            Shape::Polygon(rings) => {
                o.push(2);
                o.extend_from_slice(&(rings.len() as u32).to_le_bytes());
                for r in rings {
                    pts(&mut o, r);
                }
            }
            Shape::Line(p) => {
                o.push(3);
                pts(&mut o, p);
            }
        }
        o
    }

    /// The box of positions `[lat0, lat1] x [lon0, lon1]` the shape lies in.
    fn bounds(&self) -> (i64, i64, i64, i64) {
        let of = |mut it: Box<dyn Iterator<Item = &(i64, i64)> + '_>| {
            let first = *it.next().expect("checked: a shape has a vertex");
            it.fold((first.0, first.0, first.1, first.1), |b, &(la, lo)| {
                (b.0.min(la), b.1.max(la), b.2.min(lo), b.3.max(lo))
            })
        };
        match &self.shape {
            Shape::Box {
                lat0,
                lat1,
                lon0,
                lon1,
            } => (*lat0, *lat1, *lon0, *lon1),
            Shape::Polygon(rings) => of(Box::new(rings.iter().flatten())),
            Shape::Line(p) => of(Box::new(p.iter())),
        }
    }

    /// The blocks that hold a post the item covers.
    pub fn blocks(&self, s: i64) -> BTreeSet<BlockId> {
        let (la0, la1, lo0, lo1) = self.bounds();
        // A line also marks the posts whose cell it touches: one post past its box.
        let (i0, i1) = (la0.div_euclid(s) - 1, la1.div_euclid(s) + 1);
        let (j0, j1) = (lo0.div_euclid(s) - 1, lo1.div_euclid(s) + 1);
        let b = BLOCK as i64;
        let mut out = BTreeSet::new();
        for bi in i0.div_euclid(b)..=i1.div_euclid(b) {
            for bj in j0.div_euclid(b)..=j1.div_euclid(b) {
                let id = BlockId {
                    bi: bi as i32,
                    bj: bj as i32,
                };
                let mut any = false;
                self.cover_block(id, s, &mut |_, _| any = true);
                if any {
                    out.insert(id);
                }
            }
        }
        out
    }

    /// Calls `visit(i, j)` for each post `(i, j)` of `block` the item covers (a post may come
    /// more than once). Every effect is idempotent per post, so repeats do not matter.
    pub fn cover_block(&self, block: BlockId, s: i64, visit: &mut impl FnMut(i64, i64)) {
        let (i0, j0) = (block.i0(), block.j0());
        let (i1, j1) = (i0 + BLOCK as i64, j0 + BLOCK as i64);
        // Posts p of this block with lo <= p * s < hi.
        let span = |lo: i64, hi: i64, first: i64, end: i64| {
            let a = ceil_div(lo, s).max(first);
            a..ceil_div(hi, s).min(end).max(a)
        };
        let (la0, la1, lo0, lo1) = self.bounds();
        match &self.shape {
            Shape::Box { .. } => {
                for i in span(la0, la1, i0, i1) {
                    for j in span(lo0, lo1, j0, j1) {
                        visit(i, j);
                    }
                }
            }
            Shape::Polygon(rings) => {
                let rows = span(la0, la1, i0, i1);
                // Posts west or east of every vertex are outside.
                if rows.is_empty() || lo0 >= j1 * s || lo1 < j0 * s {
                    return;
                }
                // Edges that cross the block's rows, as (lat low, lon at low, lat high, lon at
                // high), not horizontal.
                let (ylo, yhi) = (rows.start * s, (rows.end - 1) * s);
                let mut edges = Vec::new();
                for r in rings {
                    for k in 0..r.len() {
                        let (a, b) = (r[k], r[(k + 1) % r.len()]);
                        let (lo, hi) = if a.0 < b.0 { (a, b) } else { (b, a) };
                        if lo.0 == hi.0 || hi.0 <= ylo || lo.0 > yhi {
                            continue;
                        }
                        edges.push((lo.0, lo.1, hi.0, hi.1));
                    }
                }
                let mut xs: Vec<i64> = Vec::new();
                for i in rows {
                    let y = i * s;
                    xs.clear();
                    for &(ya, xa, yb, xb) in &edges {
                        if ya <= y && y < yb {
                            // The first post at or east of the crossing
                            // x = xa + (y - ya)(xb - xa) / (yb - ya).
                            let dy = (yb - ya) as i128;
                            let num = xa as i128 * dy + (y - ya) as i128 * (xb - xa) as i128;
                            let den = dy * s as i128;
                            let q = num.div_euclid(den) + (num.rem_euclid(den) != 0) as i128;
                            xs.push(q.clamp(j0 as i128, j1 as i128) as i64);
                        }
                    }
                    xs.sort_unstable();
                    for pair in xs.as_chunks::<2>().0 {
                        for j in pair[0]..pair[1] {
                            visit(i, j);
                        }
                    }
                }
            }
            Shape::Line(p) => {
                // A segment touches the cells of posts within s/2 + s of its box.
                let (c0, c1) = ((i0 - 1) * s, (i1 + 1) * s);
                let (d0, d1) = ((j0 - 1) * s, (j1 + 1) * s);
                let seg = |a: (i64, i64), b: (i64, i64), visit: &mut dyn FnMut(i64, i64)| {
                    if a.0.max(b.0) < c0
                        || a.0.min(b.0) > c1
                        || a.1.max(b.1) < d0
                        || a.1.min(b.1) > d1
                    {
                        return;
                    }
                    supercover_ticks(a, b, s, &mut |i, j| {
                        if (i0..i1).contains(&i) && (j0..j1).contains(&j) {
                            visit(i, j)
                        }
                    });
                };
                if p.len() == 1 {
                    seg(p[0], p[0], visit);
                }
                for w in p.windows(2) {
                    seg(w[0], w[1], visit);
                }
            }
        }
    }
}

/// The overlay as fusion applies it: sorted by (observed time, item bytes), duplicates removed.
/// Any order of the same items gives the same result.
pub fn canonical_overlay(items: &[OverlayItem]) -> Vec<OverlayItem> {
    // By observed time as a number first: the bytes are little-endian, so they alone would not
    // sort by time.
    let mut v: Vec<(i64, Vec<u8>, &OverlayItem)> = items
        .iter()
        .map(|it| (it.observed_at, it.bytes(), it))
        .collect();
    v.sort_by(|a, b| (a.0, &a.1).cmp(&(b.0, &b.1)));
    v.dedup_by(|a, b| a.1 == b.1);
    v.into_iter().map(|x| x.2.clone()).collect()
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
    let u = |p: (i32, i32)| (p.0 as i64 * 27, p.1 as i64 * 27);
    supercover_units(u(a), u(b), s * 625, (s / 2) * 625, visit);
}

/// [`supercover`] for points in ticks: the same cells for the same positions.
pub fn supercover_ticks(a: (i64, i64), b: (i64, i64), s: i64, visit: &mut impl FnMut(i64, i64)) {
    supercover_units(a, b, s, s / 2, visit);
}

/// The supercover in a unit where a cell is `c` wide and a post sits `half` past its edge.
fn supercover_units(
    a: (i64, i64),
    b: (i64, i64),
    c: i64,
    half: i64,
    visit: &mut impl FnMut(i64, i64),
) {
    let (ay, ax) = (a.0 + half, a.1 + half);
    let (by, bx) = (b.0 + half, b.1 + half);
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
                        *v = lb.vals[c].get(q) as u8;
                    }
                    blend_class(&mut fb.p[q], &qv, w);
                }
                Kind::Hazard => {
                    for (ch, &code) in l.m.channels.iter().enumerate() {
                        if let Some(sl) = hazard_slot(code) {
                            fb.haz[q][sl] =
                                blend_hazard(fb.haz[q][sl], lb.vals[ch].get(q) as u8, w);
                        }
                    }
                }
                Kind::Level => {
                    if w < prm.level_min_w {
                        continue;
                    }
                    for (ch, &code) in l.m.channels.iter().enumerate() {
                        if let Some(sl) = level_slot(code) {
                            fb.lev[q][sl] = (lb.vals[ch].get(q), w);
                        }
                    }
                }
                Kind::Feature => unreachable!("feature layers are not grids"),
            }
            fb.src[q] = src;
        }
    }
    // Team overlay items, last, in canonical order: a later observation wins.
    let (i0, j0) = (id.i0(), id.j0());
    for it in overlay {
        it.cover_block(id, s, &mut |i, j| {
            let q = (i - i0) as usize * BLOCK + (j - j0) as usize;
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
        });
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

    /// A small deterministic generator for the shape tests.
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self, n: i64) -> i64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((self.0 >> 33) % n as u64) as i64
        }
    }

    fn covered(it: &OverlayItem, blocks: &[BlockId], s: i64) -> BTreeSet<(i64, i64)> {
        let mut v = BTreeSet::new();
        for &b in blocks {
            it.cover_block(b, s, &mut |i, j| {
                v.insert((i, j));
            });
        }
        v
    }

    fn around(bi: i32, bj: i32) -> Vec<BlockId> {
        let mut v = Vec::new();
        for di in -1..=1 {
            for dj in -1..=1 {
                v.push(BlockId {
                    bi: bi + di,
                    bj: bj + dj,
                });
            }
        }
        v
    }

    /// Even-odd by the documented rule, post by post.
    fn inside(rings: &[Vec<(i64, i64)>], y: i64, x: i64) -> bool {
        let mut odd = false;
        for r in rings {
            for k in 0..r.len() {
                let (a, b) = (r[k], r[(k + 1) % r.len()]);
                let (lo, hi) = if a.0 < b.0 { (a, b) } else { (b, a) };
                if lo.0 == hi.0 || !(lo.0 <= y && y < hi.0) {
                    continue;
                }
                // The crossing is at or west of x: xa + (y - ya)(xb - xa)/(yb - ya) <= x.
                let dy = (hi.0 - lo.0) as i128;
                let lhs = lo.1 as i128 * dy + (y - lo.0) as i128 * (hi.1 - lo.1) as i128;
                if lhs <= x as i128 * dy {
                    odd = !odd;
                }
            }
        }
        odd
    }

    #[test]
    fn overlay_polygons_fill_even_odd() {
        let mut g = Lcg(7);
        for s in [120, 90, 7] {
            let (bi, bj) = (-3, 5);
            let blocks = around(bi, bj);
            let (y0, x0) = (bi as i64 * 64 * s, bj as i64 * 64 * s);
            let span = 64 * s;
            for case in 0..60 {
                // Concave rings, often with vertices on post rows, sometimes with a hole.
                let mut rings = Vec::new();
                for _ in 0..1 + (case % 3 == 0) as usize {
                    let n = 3 + g.next(9) as usize;
                    let r: Vec<(i64, i64)> = (0..n)
                        .map(|_| {
                            let snap = g.next(2) == 0;
                            let (y, x) = (g.next(span), g.next(span));
                            let (y, x) = if snap { (y / s * s, x / s * s) } else { (y, x) };
                            (y0 + y, x0 + x)
                        })
                        .collect();
                    rings.push(r);
                }
                let it = OverlayItem {
                    shape: Shape::Polygon(rings.clone()),
                    observed_at: 0,
                    effect: Effect::Block,
                };
                it.check().unwrap();
                let got = covered(&it, &blocks, s);
                let mut want = BTreeSet::new();
                for b in &blocks {
                    for i in b.i0()..b.i0() + 64 {
                        for j in b.j0()..b.j0() + 64 {
                            if inside(&rings, i * s, j * s) {
                                want.insert((i, j));
                            }
                        }
                    }
                }
                assert_eq!(got, want, "spacing {s}, case {case}");
                let marked: BTreeSet<BlockId> =
                    got.iter().map(|&(i, j)| BlockId::of_post(i, j).0).collect();
                assert_eq!(it.blocks(s), marked, "spacing {s}, case {case}");
            }
        }
    }

    #[test]
    fn a_box_as_a_ring_marks_the_box() {
        let mut g = Lcg(11);
        let s = 120;
        let blocks = around(4, -2);
        let (y0, x0) = (4 * 64 * s, -2 * 64 * s);
        for _ in 0..200 {
            let (a, b) = (g.next(3 * 64 * s) - 64 * s, g.next(3 * 64 * s) - 64 * s);
            let (c, d) = (g.next(3 * 64 * s) - 64 * s, g.next(3 * 64 * s) - 64 * s);
            let (lat0, lat1, lon0, lon1) = (
                y0 + a.min(b),
                y0 + a.max(b) + 1,
                x0 + c.min(d),
                x0 + c.max(d) + 1,
            );
            let bx = OverlayItem {
                shape: Shape::Box {
                    lat0,
                    lat1,
                    lon0,
                    lon1,
                },
                observed_at: 0,
                effect: Effect::Clear,
            };
            let ring = OverlayItem {
                shape: Shape::Polygon(vec![vec![
                    (lat0, lon0),
                    (lat1, lon0),
                    (lat1, lon1),
                    (lat0, lon1),
                ]]),
                ..bx.clone()
            };
            let want = covered(&bx, &blocks, s);
            assert_eq!(covered(&ring, &blocks, s), want);
            for &(i, j) in &want {
                assert!(lat0 <= i * s && i * s < lat1 && lon0 <= j * s && j * s < lon1);
            }
            assert_eq!(bx.blocks(s), ring.blocks(s));
        }
    }

    #[test]
    fn overlay_lines_are_the_feature_supercover() {
        let s = 120;
        // Ticks that are whole E7 degrees: ticks x 625 = E7 x 27.
        let pts = [
            (0i64, 0i64),
            (27 * 300, 27 * 41),
            (-27 * 200, 27 * 520),
            (-27 * 200, 27 * 520),
        ];
        let e7 = |p: (i64, i64)| ((p.0 * 625 / 27) as i32, (p.1 * 625 / 27) as i32);
        let mut want = BTreeSet::new();
        for w in pts.windows(2) {
            supercover(e7(w[0]), e7(w[1]), s, &mut |i, j| {
                want.insert((i, j));
            });
            let mut t = BTreeSet::new();
            supercover_ticks(w[0], w[1], s, &mut |i, j| {
                t.insert((i, j));
            });
            let mut e = BTreeSet::new();
            supercover(e7(w[0]), e7(w[1]), s, &mut |i, j| {
                e.insert((i, j));
            });
            assert_eq!(t, e, "the same cells in ticks and in E7");
        }
        let it = OverlayItem {
            shape: Shape::Line(pts.to_vec()),
            observed_at: 0,
            effect: Effect::SetHazard(1, 200),
        };
        assert_eq!(covered(&it, &around(0, 0), s), want);
        let marked: BTreeSet<BlockId> = want
            .iter()
            .map(|&(i, j)| BlockId::of_post(i, j).0)
            .collect();
        assert_eq!(it.blocks(s), marked);
        // One vertex marks one post.
        let dot = OverlayItem {
            shape: Shape::Line(vec![(130, -10)]),
            ..it.clone()
        };
        assert_eq!(covered(&dot, &around(0, 0), s), BTreeSet::from([(1, 0)]));
    }

    #[test]
    fn overlay_items_check_and_encode() {
        let it = |shape| OverlayItem {
            shape,
            observed_at: 5,
            effect: Effect::Block,
        };
        let bx = it(Shape::Box {
            lat0: 0,
            lat1: 10,
            lon0: 0,
            lon1: 10,
        });
        let tri = it(Shape::Polygon(vec![vec![(0, 0), (10, 0), (0, 10)]]));
        let line = it(Shape::Line(vec![(0, 0), (10, 0), (0, 10)]));
        for x in [&bx, &tri, &line] {
            x.check().unwrap();
        }
        let enc: BTreeSet<Vec<u8>> = [&bx, &tri, &line].iter().map(|x| x.bytes()).collect();
        assert_eq!(enc.len(), 3, "shapes encode apart");
        assert_eq!(bx.bytes().len(), 8 + 4 + 1 + 32);
        assert_eq!(tri.bytes().len(), 8 + 4 + 1 + 4 + 4 + 3 * 16);
        // Splitting a ring's vertices across two rings is a different item.
        let split = it(Shape::Polygon(vec![
            vec![(0, 0), (10, 0), (0, 10)],
            vec![(0, 0), (10, 0), (0, 10)],
        ]));
        assert_ne!(split.bytes(), tri.bytes());
        for bad in [
            Shape::Box {
                lat0: 0,
                lat1: 0,
                lon0: 0,
                lon1: 10,
            },
            Shape::Polygon(vec![]),
            Shape::Polygon(vec![vec![(0, 0), (1, 1)]]),
            Shape::Line(vec![]),
            Shape::Line(vec![(91 * TICKS_PER_DEG, 0)]),
            Shape::Polygon(vec![vec![(0, 0); MAX_OVERLAY_VERTICES + 1]]),
        ] {
            assert!(it(bad).check().is_err());
        }
        let p = OverlayItem::polygon_from_degrees(
            &[vec![(49.0, 20.0), (49.01, 20.0), (49.0, 20.01)]],
            5,
            Effect::Block,
        )
        .unwrap();
        assert_eq!(
            p.shape,
            Shape::Polygon(vec![vec![
                (49 * 432_000, 20 * 432_000),
                (49 * 432_000 + 4320, 20 * 432_000),
                (49 * 432_000, 20 * 432_000 + 4320)
            ]])
        );
        assert!(OverlayItem::line_from_degrees(&[(0.0, 181.0)], 5, Effect::Block).is_err());
    }

    #[test]
    fn priors() {
        let p = Prior::default();
        assert_eq!(p.get(3, 1), onehot(3));
        assert!(Prior::new(vec![[0; NCLASS]; 32]).is_err());
        assert_eq!(Prior::new(p.table.clone()).unwrap(), p);
    }
}

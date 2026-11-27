//! Stage 1: heights, steepness-max and land cover per post, one block row at a time.
//!
//! ## Samples
//!
//! Every post's cell `[p − s/2, p + s/2)` is sampled at the centres of a `K × K` grid of
//! sub-cells, `K = s / 15` ticks (8 at 1″, 4 at ½″), so the samples of every spacing sit on one
//! global 1/8″ grid. A sample's projected position comes from the source's coordinate
//! system: exact integer arithmetic for geographic sources, and for UTM sources the forward
//! projection (`libm`, the same bits everywhere) at anchors every 8 posts, interpolated
//! bilinearly in integers in between (under a millimetre from the exact projection).
//! Positions are in 1/65,536 pixel.
//!
//! ## Heights
//!
//! A sample's height is the bilinear value of the four source pixels around it, when all four
//! have data, in whole millimetres (half away from zero). Where mosaics of one tier overlap,
//! a sample takes the mean of their values. A post takes the first tier (GEM-15 before GLO-30,
//! say) in which at least half its samples have a height, and the mean of those samples in
//! decimetres, half away from zero; otherwise it has no height.
//!
//! ## Steepness-max
//!
//! Each source pixel with all eight neighbours gets the Horn slope class of
//! `format-4.md` section 4 on its own grid (UTM: the grid step on both axes; geographic: the
//! step's length on a sphere of mean radius at the pixel's latitude), compared exactly in
//! integers. A post's class is the largest class of the pixels its samples fall in, from the
//! tier that gave its height, else from the next tier that has one. A post without a height
//! has no class.
//!
//! ## Land cover
//!
//! A post's code is the majority WorldCover class (mapped to the 4-bit codes of
//! [`wayfarer_terrain::profile`]) of the WorldCover pixels whose centres fall in its cell,
//! ties to the class that blocks or endangers more (water, then wetland, built-up, bare...).
//!
//! With an OSM extract (see [`crate::osm`] for how its areas reach the posts), OSM overrides
//! WorldCover where an OSM area covers at least half a post's 16 samples, except for forest,
//! where the two agree almost everywhere, and scrub below 1,300 m, which is mostly
//! overgrowing farmland. Scrub at or above 1,300 m (by the post's own height) is dwarf pine
//! (code 12); bare rock is 13, scree 14 and bog or swamp forest 15, classes WorldCover lacks
//! (E3's recommended fusion).
//!
//! The confidence bit is set where OSM overrides and agrees with WorldCover (in the coarse
//! classes: mangrove counts as tree, moss as grass, dwarf pine as shrub, rock and scree as
//! bare, bog as wetland), or where OSM does not override and the WorldCover class holds at
//! least three quarters of the cell's pixels.
//!
//! ## Linear features
//!
//! The lines plane is the OSM lines mask of [`crate::osm`], zero without an extract.

use crate::geo::Utm;
use crate::osm::{self, OsmPlanes};
use crate::source::{ChunkCache, MM_NONE, Mosaic, Window};
use crate::{Error, Result};
use wayfarer_terrain::lattice::{BLOCK, BLOCK_POSTS, BlockId, Spacing, div_round};
use wayfarer_terrain::terrain::{DEM_NONE, STEEP_NONE};

pub(crate) const FRAC: i64 = 1 << 16;
/// Half-ticks between neighbouring samples (1/8″).
pub(crate) const SAMPLE_HALF_TICKS: i64 = 30;

/// Samples per post side at a spacing.
pub fn samples_per_side(s: Spacing) -> Result<usize> {
    if s.ticks() % 15 != 0 {
        return Err(Error::Options(format!(
            "spacing {} ticks is not a multiple of 1/8 arc-second",
            s.ticks()
        )));
    }
    Ok((s.ticks() / 15) as usize)
}

/// Half-tick latitude (or longitude) of sample `t` of a block whose first post is `p0`.
fn sample_pos(p0: i64, s: i64, k: i64, t: i64) -> i64 {
    2 * p0 * s + (1 - k) * 15 + t * SAMPLE_HALF_TICKS
}

/// Where a block's samples fall on one mosaic's pixel grid.
enum Coords {
    /// Geographic: rows from latitude only, columns from longitude only.
    Separable { u: Vec<i64>, v: Vec<i64> },
    /// UTM: anchors every `g` samples, `n × n` of them, interpolated.
    Anchored {
        g: i64,
        n: usize,
        u: Vec<i64>,
        v: Vec<i64>,
    },
}

impl Coords {
    #[inline]
    fn at(&self, sy: usize, sx: usize) -> (i64, i64) {
        match self {
            Coords::Separable { u, v } => (u[sx], v[sy]),
            Coords::Anchored { g, n, u, v } => {
                let g = *g as usize;
                let (qy, fy) = (sy / g, (sy % g) as i64);
                let (qx, fx) = (sx / g, (sx % g) as i64);
                let gg = g as i64;
                let i = qy * n + qx;
                let w = [
                    (gg - fy) * (gg - fx),
                    (gg - fy) * fx,
                    fy * (gg - fx),
                    fy * fx,
                ];
                let ix = [i, i + 1, i + n, i + n + 1];
                let mut su = 0i64;
                let mut sv = 0i64;
                for k in 0..4 {
                    if w[k] != 0 {
                        su += u[ix[k]] * w[k];
                        sv += v[ix[k]] * w[k];
                    }
                }
                let d = gg * gg;
                (div_round_i64(su, d), div_round_i64(sv, d))
            }
        }
    }
    /// Smallest and largest (u, v) over the block's samples.
    fn bounds(&self) -> (i64, i64, i64, i64) {
        let (u, v) = match self {
            Coords::Separable { u, v } | Coords::Anchored { u, v, .. } => (u, v),
        };
        (
            *u.iter().min().unwrap(),
            *u.iter().max().unwrap(),
            *v.iter().min().unwrap(),
            *v.iter().max().unwrap(),
        )
    }
}

#[inline]
fn div_round_i64(num: i64, den: i64) -> i64 {
    if num >= 0 {
        (2 * num + den) / (2 * den)
    } else {
        -((-2 * num + den) / (2 * den))
    }
}

fn coords(m: &Mosaic, b: BlockId, s: Spacing, k: usize) -> Coords {
    let g = m.grid;
    let n = BLOCK * k;
    let (s, k64) = (s.ticks(), k as i64);
    match g.crs {
        crate::geo::Crs::Geographic => {
            let v = (0..n as i64)
                .map(|t| ((g.y0 - sample_pos(b.i0(), s, k64, t)) * FRAC).div_euclid(g.dy))
                .collect();
            let u = (0..n as i64)
                .map(|t| ((sample_pos(b.j0(), s, k64, t) - g.x0) * FRAC).div_euclid(g.dx))
                .collect();
            Coords::Separable { u, v }
        }
        crate::geo::Crs::Utm { zone, north } => {
            let proj = Utm::new(zone, north);
            let ga = 8 * k;
            let na = n / ga + 1;
            let mut u = Vec::with_capacity(na * na);
            let mut v = Vec::with_capacity(na * na);
            for ay in 0..na {
                let lat = sample_pos(b.i0(), s, k64, (ay * ga) as i64) as f64
                    / crate::source::HALF_TICKS_PER_DEG;
                for ax in 0..na {
                    let lon = sample_pos(b.j0(), s, k64, (ax * ga) as i64) as f64
                        / crate::source::HALF_TICKS_PER_DEG;
                    let (e, nn) = proj.forward(lat, lon);
                    let fx = (e * crate::source::MM_FP - g.x0 as f64) / g.dx as f64;
                    let fy = (g.y0 as f64 - nn * crate::source::MM_FP) / g.dy as f64;
                    u.push(libm::round(fx * FRAC as f64) as i64);
                    v.push(libm::round(fy * FRAC as f64) as i64);
                }
            }
            Coords::Anchored {
                g: ga as i64,
                n: na,
                u,
                v,
            }
        }
    }
}

/// Slope class bounds in degrees (`format-4.md` section 4).
pub const STEEP_BOUNDS_DEG: [f64; 15] = [
    2., 5., 8., 11., 15., 20., 25., 30., 35., 40., 45., 50., 55., 60., 70.,
];
const EARTH_R_MM: f64 = 6_371_008_800.0;

/// round(tan²(bound) · 2³²) per bound.
pub(crate) fn tan2_bounds() -> [i128; 15] {
    let mut t2 = [0i128; 15];
    for (i, d) in STEEP_BOUNDS_DEG.iter().enumerate() {
        let t = libm::tan(d * core::f64::consts::PI / 180.0);
        t2[i] = libm::round(t * t * 4_294_967_296.0) as i128;
    }
    t2
}

/// Horn slope class of a 3 × 3 window (mm, north row first, west to east), ground steps in mm.
pub(crate) fn horn_class(t2: &[i128; 15], w: &[i64; 9], dx: i64, dy: i64) -> u8 {
    let gx = ((w[2] + 2 * w[5] + w[8]) - (w[0] + 2 * w[3] + w[6])) as i128;
    let gy = ((w[0] + 2 * w[1] + w[2]) - (w[6] + 2 * w[7] + w[8])) as i128;
    let (dx, dy) = (dx as i128, dy as i128);
    let num = (gx * gx * dy * dy + gy * gy * dx * dx) << 32;
    let den = 64 * dx * dx * dy * dy;
    t2.iter().filter(|&&t| num > t * den).count() as u8
}

/// Ground length in mm of `units` of the grid (half-ticks of latitude for geographic grids,
/// 1/1024 m for UTM), and of longitude at latitude `lat_h` half-ticks.
fn ground_mm(m: &Mosaic, units: i64, lat_h: Option<i64>) -> i64 {
    match m.grid.crs {
        crate::geo::Crs::Geographic => {
            let rad =
                units as f64 / crate::source::HALF_TICKS_PER_DEG * core::f64::consts::PI / 180.0;
            let c = lat_h.map_or(1.0, |l| {
                libm::cos(
                    l as f64 / crate::source::HALF_TICKS_PER_DEG * core::f64::consts::PI / 180.0,
                )
            });
            libm::round(rad * EARTH_R_MM * c) as i64
        }
        crate::geo::Crs::Utm { .. } => div_round_i64(units * 1000, 1024),
    }
}

/// Slope classes (0xFF = none) of a height window's pixels.
fn classes_of(m: &Mosaic, w: &Window<i32>, t2: &[i128; 15]) -> Vec<u8> {
    let mut out = vec![STEEP_NONE; w.rows * w.cols];
    let dy = ground_mm(m, m.grid.dy, None);
    for y in 1..w.rows.saturating_sub(1) {
        let lat = m.grid.y0 - (w.r0 + y as i64) * m.grid.dy;
        let dx = ground_mm(m, m.grid.dx, Some(lat));
        if dx <= 0 || dy <= 0 {
            continue;
        }
        'px: for x in 1..w.cols - 1 {
            let mut z = [0i64; 9];
            for (i, (dr, dc)) in [
                (-1, -1),
                (-1, 0),
                (-1, 1),
                (0, -1),
                (0, 0),
                (0, 1),
                (1, -1),
                (1, 0),
                (1, 1),
            ]
            .iter()
            .enumerate()
            {
                let v = w.v[(y as i64 + dr) as usize * w.cols + (x as i64 + dc) as usize];
                if v == MM_NONE {
                    continue 'px;
                }
                z[i] = v as i64;
            }
            out[y * w.cols + x] = horn_class(t2, &z, dx, dy);
        }
    }
    out
}

/// One mosaic prepared for one block.
struct Prepared {
    coords: Coords,
    heights: Window<i32>,
    classes: Vec<u8>,
}

impl Prepared {
    #[inline]
    fn height(&self, u: i64, v: i64) -> Option<i64> {
        let (c, fx) = (u >> 16, u & (FRAC - 1));
        let (r, fy) = (v >> 16, v & (FRAC - 1));
        let w = &self.heights;
        let a = w.at(r, c)?;
        let b = w.at(r, c + 1)?;
        let cc = w.at(r + 1, c)?;
        let d = w.at(r + 1, c + 1)?;
        if a == MM_NONE || b == MM_NONE || cc == MM_NONE || d == MM_NONE {
            return None;
        }
        let num = a as i64 * (FRAC - fx) * (FRAC - fy)
            + b as i64 * fx * (FRAC - fy)
            + cc as i64 * (FRAC - fx) * fy
            + d as i64 * fx * fy;
        Some(div_round_i64(num, FRAC * FRAC))
    }
    #[inline]
    fn class(&self, u: i64, v: i64) -> u8 {
        let (c, r) = ((u + FRAC / 2) >> 16, (v + FRAC / 2) >> 16);
        let w = &self.heights;
        let (y, x) = (r - w.r0, c - w.c0);
        if y < 0 || x < 0 || y as usize >= w.rows || x as usize >= w.cols {
            return STEEP_NONE;
        }
        self.classes[y as usize * w.cols + x as usize]
    }
}

fn prepare(
    m: &Mosaic,
    cache: &ChunkCache,
    b: BlockId,
    s: Spacing,
    k: usize,
    t2: &[i128; 15],
) -> Result<Option<Prepared>> {
    let coords = coords(m, b, s, k);
    let (u0, u1, v0, v1) = coords.bounds();
    // Bilinear corners and the nearest pixel, plus one pixel of Horn neighbours.
    let (c0, c1) = ((u0 >> 16) - 1, (u1 >> 16) + 3);
    let (r0, r1) = ((v0 >> 16) - 1, (v1 >> 16) + 3);
    if !m.touches(r0, r1, c0, c1) {
        return Ok(None);
    }
    if (c1 - c0) * (r1 - r0) > 64 << 20 {
        return Err(Error::Source(
            "a source is too fine for this spacing (over 64 M pixels per block)".into(),
        ));
    }
    let heights = m.heights(cache, r0, r1, c0, c1)?;
    let classes = classes_of(m, &heights, t2);
    Ok(Some(Prepared {
        coords,
        heights,
        classes,
    }))
}

/// One block's planes.
pub(crate) struct Planes {
    pub dm: Vec<i32>,
    pub steep: Vec<u8>,
    pub cover: Vec<u8>,
    pub conf: Vec<u8>,
    pub lines: Vec<u16>,
    /// Posts whose height came from each tier, and posts whose class came from a later tier.
    pub by_tier: Vec<u64>,
    pub steep_fallback: u64,
    /// Posts whose land cover OSM set.
    pub osm_cover: u64,
}

pub(crate) struct Ctx<'a> {
    pub tiers: &'a [Vec<Mosaic>],
    pub cover: &'a [Mosaic],
    pub cache: &'a ChunkCache,
    pub spacing: Spacing,
    pub k: usize,
    pub t2: [i128; 15],
}

pub(crate) fn block(ctx: &Ctx, b: BlockId, osm: Option<&OsmPlanes>) -> Result<Planes> {
    let k = ctx.k;
    let kk = (k * k) as u64;
    let mut p = Planes {
        dm: vec![DEM_NONE; BLOCK_POSTS],
        steep: vec![STEEP_NONE; BLOCK_POSTS],
        cover: vec![0; BLOCK_POSTS],
        conf: vec![0; BLOCK_POSTS],
        lines: vec![0; BLOCK_POSTS],
        by_tier: vec![0; ctx.tiers.len()],
        steep_fallback: 0,
        osm_cover: 0,
    };
    // Posts still wanting a height, or (with a height) a class.
    let mut want_h = vec![true; BLOCK_POSTS];
    let mut want_s = vec![false; BLOCK_POSTS];
    for (t, tier) in ctx.tiers.iter().enumerate() {
        if !want_h.iter().any(|&x| x) && !want_s.iter().any(|&x| x) {
            break;
        }
        let mut prep = Vec::new();
        for m in tier {
            if let Some(x) = prepare(m, ctx.cache, b, ctx.spacing, k, &ctx.t2)? {
                prep.push(x);
            }
        }
        if prep.is_empty() {
            continue;
        }
        for q in 0..BLOCK_POSTS {
            if !want_h[q] && !want_s[q] {
                continue;
            }
            let (r, c) = (q / BLOCK, q % BLOCK);
            let (mut sum, mut cnt) = (0i64, 0u64);
            let mut cls = STEEP_NONE;
            for a in 0..k {
                let sy = r * k + a;
                for bb in 0..k {
                    let sx = c * k + bb;
                    let (mut ss, mut sn) = (0i64, 0i64);
                    for pm in &prep {
                        let (u, v) = pm.coords.at(sy, sx);
                        if want_h[q]
                            && let Some(h) = pm.height(u, v)
                        {
                            ss += h;
                            sn += 1;
                        }
                        let c = pm.class(u, v);
                        if c != STEEP_NONE && (cls == STEEP_NONE || c > cls) {
                            cls = c;
                        }
                    }
                    if sn > 0 {
                        sum += if sn == 1 { ss } else { div_round_i64(ss, sn) };
                        cnt += 1;
                    }
                }
            }
            if want_h[q] && 2 * cnt >= kk && cnt > 0 {
                p.dm[q] = div_round(sum as i128, cnt as i128 * 100) as i32;
                p.by_tier[t] += 1;
                want_h[q] = false;
                if cls != STEEP_NONE {
                    p.steep[q] = cls;
                } else {
                    want_s[q] = true;
                }
            } else if want_s[q] && cls != STEEP_NONE {
                p.steep[q] = cls;
                p.steep_fallback += 1;
                want_s[q] = false;
            }
        }
    }
    cover(ctx, b, &mut p)?;
    if let Some(o) = osm {
        fuse(&mut p, o);
    }
    Ok(p)
}

/// The lowest height of dwarf pine, in decimetres.
const DWARF_PINE_MIN_DM: i32 = 13_000;
/// The coarse class of each 4-bit code, for agreement.
const COARSE: [u8; 16] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 1, 3, 2, 6, 6, 9];

/// OSM's override of the WorldCover planes, and the lines plane.
fn fuse(p: &mut Planes, o: &OsmPlanes) {
    for q in 0..BLOCK_POSTS {
        let fine = o.fine[q];
        let code = match fine {
            osm::FOREST => 0,
            osm::SCRUB if p.dm[q] != DEM_NONE && p.dm[q] >= DWARF_PINE_MIN_DM => 12,
            _ => osm::FINE_TO_COVER[fine as usize],
        };
        let wc = p.cover[q];
        if 2 * o.cov[q] as usize >= osm::SUB * osm::SUB && code != 0 {
            p.conf[q] = (COARSE[code as usize] == COARSE[wc as usize]) as u8;
            p.cover[q] = code;
            p.osm_cover += 1;
        }
    }
    p.lines.copy_from_slice(&o.lines);
}

/// WorldCover code to the 4-bit class (0 = none).
pub fn worldcover_class(code: u8) -> u8 {
    match code {
        10 => 1,
        20 => 2,
        30 => 3,
        40 => 4,
        50 => 5,
        60 => 6,
        70 => 7,
        80 => 8,
        90 => 9,
        95 => 10,
        100 => 11,
        _ => 0,
    }
}

/// Tie priority per class (higher wins): water, wetland, built-up, bare, snow, tree, shrub,
/// moss, grass, crop, mangrove.
const PRIORITY: [u8; 12] = [0, 6, 5, 3, 2, 9, 8, 7, 11, 10, 1, 4];

fn cover(ctx: &Ctx, b: BlockId, p: &mut Planes) -> Result<()> {
    if ctx.cover.is_empty() {
        return Ok(());
    }
    let s = ctx.spacing.ticks();
    let mut hist = vec![[0u16; 12]; BLOCK_POSTS];
    for m in ctx.cover {
        let g = m.grid;
        if g.crs != crate::geo::Crs::Geographic {
            return Err(Error::Source(
                "land cover must be geographic (EPSG:4326)".into(),
            ));
        }
        // Cells in half-ticks: post i covers [2is − s, 2is + s).
        let cell_lat = |i: i64| (2 * i * s - s, 2 * i * s + s);
        let rows_of = |(lo, hi): (i64, i64)| {
            (
                (g.y0 - hi).div_euclid(g.dy) + 1,
                (g.y0 - lo).div_euclid(g.dy),
            )
        };
        let cols_of = |(lo, hi): (i64, i64)| {
            (
                -((g.x0 - lo).div_euclid(g.dx)),
                -((g.x0 - hi).div_euclid(g.dx)) - 1,
            )
        };
        let (r0, _) = rows_of((
            cell_lat(b.i0() + BLOCK as i64 - 1).0,
            cell_lat(b.i0() + BLOCK as i64 - 1).1,
        ));
        let (_, r1) = rows_of(cell_lat(b.i0()));
        let (c0, _) = cols_of(cell_lat(b.j0()));
        let (_, c1) = cols_of(cell_lat(b.j0() + BLOCK as i64 - 1));
        if r0 > r1 || c0 > c1 || !m.touches(r0, r1 + 1, c0, c1 + 1) {
            continue;
        }
        let w = m.classes(ctx.cache, r0, r1 + 1, c0, c1 + 1)?;
        for r in 0..BLOCK {
            let (ra, rb) = rows_of(cell_lat(b.i0() + r as i64));
            for c in 0..BLOCK {
                let (ca, cb) = cols_of(cell_lat(b.j0() + c as i64));
                let h = &mut hist[r * BLOCK + c];
                for y in ra..=rb {
                    for x in ca..=cb {
                        if let Some(code) = w.at(y, x) {
                            let cl = worldcover_class(code);
                            if cl != 0 {
                                h[cl as usize] += 1;
                            }
                        }
                    }
                }
            }
        }
    }
    for (q, h) in hist.iter().enumerate() {
        let mut best = 0usize;
        let mut tot = 0u32;
        for cl in 1..12 {
            tot += h[cl] as u32;
            if h[cl] > h[best] || (h[cl] == h[best] && h[cl] > 0 && PRIORITY[cl] > PRIORITY[best]) {
                best = cl;
            }
        }
        if h[best] > 0 {
            p.cover[q] = best as u8;
            p.conf[q] = (4 * h[best] as u32 >= 3 * tot) as u8;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn horn_classes() {
        let t2 = tan2_bounds();
        let ramp = |step: i64| {
            let r = [0, step, 2 * step];
            [r[0], r[1], r[2], r[0], r[1], r[2], r[0], r[1], r[2]]
        };
        // 1 m east per 30 m: about 1.9 degrees.
        assert_eq!(horn_class(&t2, &ramp(1000), 30_000, 30_000), 0);
        // 45 degrees exactly is not above 45: class 10.
        assert_eq!(horn_class(&t2, &ramp(30_000), 30_000, 30_000), 10);
        assert_eq!(horn_class(&t2, &ramp(30_001), 30_000, 30_000), 11);
        assert_eq!(horn_class(&t2, &ramp(10_000_000), 30_000, 30_000), 15);
    }

    #[test]
    fn sample_grid() {
        let s = Spacing::new(120).unwrap();
        assert_eq!(samples_per_side(s).unwrap(), 8);
        // Post 0's first sample is 52.5 ticks south of it; samples are 15 ticks apart.
        assert_eq!(sample_pos(0, 120, 8, 0), -105);
        assert_eq!(sample_pos(0, 120, 8, 7), 105);
        assert_eq!(sample_pos(1, 120, 8, 0), 240 - 105);
        assert!(samples_per_side(Spacing::new(36).unwrap()).is_err());
    }
}

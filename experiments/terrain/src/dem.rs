//! DEM onto the lattice, and the 4-bit steepness-max layer from the full-resolution source.
//!
//! Spacing at or below 1" (120 ticks): bilinear at the post. Coarser spacings that are a
//! multiple of 1": the mean of the 1" posts over the post's cell (`k = s / 1"` posts across,
//! the two end posts at half weight when k is even), NoData dropped and renormalised. That
//! is the "deterministic integer mean" of feat-td.md T4 with a cell centred on its post.
//! `Point` mode takes the bilinear value at the post for every spacing (decimation), for
//! comparison.

use crate::lattice::{BLOCK, BLOCK_POSTS, BlockId, Spacing, TICKS_PER_ARCSEC, TICKS_PER_DEG, div_round};
use crate::mosaic::Mosaic;
use crate::raster::DEM_NONE;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Area,
    Point,
}

pub fn dem_block(m: &Mosaic<i32>, s: Spacing, b: BlockId, mode: Mode) -> Vec<i32> {
    let mut out = vec![DEM_NONE; BLOCK_POSTS];
    let fine = TICKS_PER_ARCSEC;
    if s.0 <= fine || mode == Mode::Point || s.0 % fine != 0 {
        for r in 0..BLOCK {
            for c in 0..BLOCK {
                let lat = s.pos(b.i0() + r as i64);
                let lon = s.pos(b.j0() + c as i64);
                out[r * BLOCK + c] = m.bilinear(lat, lon).unwrap_or(DEM_NONE);
            }
        }
        return out;
    }
    let k = s.0 / fine;
    let h = k / 2;
    let weight = |d: i64| (k + 1 - 2 * d.abs()).clamp(0, 2);
    // 1" posts covering the block's cells.
    let fi0 = (b.i0()) * k - h;
    let fj0 = (b.j0()) * k - h;
    let n = (BLOCK as i64 - 1) * k + 2 * h + 1;
    let mut f = vec![DEM_NONE; (n * n) as usize];
    for y in 0..n {
        for x in 0..n {
            f[(y * n + x) as usize] = m.bilinear((fi0 + y) * fine, (fj0 + x) * fine).unwrap_or(DEM_NONE);
        }
    }
    for r in 0..BLOCK as i64 {
        for c in 0..BLOCK as i64 {
            let (cy, cx) = (r * k + h, c * k + h);
            let (mut num, mut den) = (0i128, 0i128);
            for dy in -h..=h {
                let wy = weight(dy);
                for dx in -h..=h {
                    let w = wy * weight(dx);
                    if w == 0 {
                        continue;
                    }
                    let v = f[((cy + dy) * n + cx + dx) as usize];
                    if v != DEM_NONE {
                        num += v as i128 * w as i128;
                        den += w as i128;
                    }
                }
            }
            if den > 0 {
                out[(r * BLOCK as i64 + c) as usize] = div_round(num, den) as i32;
            }
        }
    }
    out
}

/// Slope class upper bounds in degrees; class = number of bounds the slope exceeds (0..=15).
pub const STEEP_BOUNDS_DEG: [f64; 15] = [2., 5., 8., 11., 15., 20., 25., 30., 35., 40., 45., 50., 55., 60., 70.];
const EARTH_R_MM: f64 = 6_371_008_800.0;

pub struct SlopeClassifier {
    /// round(tan^2(bound) * 2^32)
    t2: [i128; 15],
}

impl SlopeClassifier {
    pub fn new() -> Self {
        let mut t2 = [0i128; 15];
        for (i, d) in STEEP_BOUNDS_DEG.iter().enumerate() {
            let t = libm::tan(d * core::f64::consts::PI / 180.0);
            t2[i] = libm::round(t * t * 4_294_967_296.0) as i128;
        }
        SlopeClassifier { t2 }
    }
    /// Ground metres (in mm) of `ticks` of longitude at latitude `lat` ticks, and of latitude.
    pub fn dx_mm(lat: i64, ticks: i64) -> i64 {
        let phi = lat as f64 / TICKS_PER_DEG as f64 * core::f64::consts::PI / 180.0;
        libm::round(ticks as f64 / TICKS_PER_DEG as f64 * core::f64::consts::PI / 180.0 * EARTH_R_MM * libm::cos(phi)) as i64
    }
    pub fn dy_mm(ticks: i64) -> i64 {
        libm::round(ticks as f64 / TICKS_PER_DEG as f64 * core::f64::consts::PI / 180.0 * EARTH_R_MM) as i64
    }
    /// Horn slope class from a 3x3 window (decimetres, north row first), or None.
    pub fn class(&self, w: &[i32; 9], dx_mm: i64, dy_mm: i64) -> Option<u8> {
        if w.iter().any(|&v| v == DEM_NONE) {
            return None;
        }
        let v = |i: usize| w[i] as i128;
        let gx = (v(2) + 2 * v(5) + v(8)) - (v(0) + 2 * v(3) + v(6));
        let gy = (v(0) + 2 * v(1) + v(2)) - (v(6) + 2 * v(7) + v(8));
        let (dx, dy) = (dx_mm as i128, dy_mm as i128);
        // slope^2 = (gx*100 / (8 dx))^2 + (gy*100 / (8 dy))^2  (dm -> mm)
        let num = (gx * 100) * (gx * 100) * dy * dy + (gy * 100) * (gy * 100) * dx * dx;
        let den = 64 * dx * dx * dy * dy;
        let lhs = num << 32;
        Some(self.t2.iter().filter(|&&t| lhs > t * den).count() as u8)
    }
}

/// Steepness-max per lattice cell: the largest Horn slope class of any source pixel whose
/// centre falls in the cell, on each source raster's own grid. 0xFF = no valid pixel.
pub fn steep_block(m: &Mosaic<i32>, s: Spacing, b: BlockId, cl: &SlopeClassifier) -> Vec<u8> {
    let mut out = vec![0xFFu8; BLOCK_POSTS];
    let half = s.0 / 2;
    let lat_lo = s.pos(b.i0()) - half;
    let lat_hi = s.pos(b.i0() + BLOCK as i64 - 1) + half; // exclusive
    let lon_lo = s.pos(b.j0()) - half;
    let lon_hi = s.pos(b.j0() + BLOCK as i64 - 1) + half;
    for (k, ras) in m.rasters.iter().enumerate() {
        let g = &ras.grid;
        // rows whose centre latitude is in [lat_lo, lat_hi)
        let r_first = (g.lat0 - (lat_hi - 1)).div_euclid(g.dlat).max(0);
        let r_last = (g.lat0 - lat_lo).div_euclid(g.dlat).min(g.height as i64 - 1);
        let c_first = (lon_lo - g.lon0 + g.dlon - 1).div_euclid(g.dlon).max(0);
        let c_last = (lon_hi - 1 - g.lon0).div_euclid(g.dlon).min(g.width as i64 - 1);
        if r_first > r_last || c_first > c_last {
            continue;
        }
        let dy = SlopeClassifier::dy_mm(g.dlat);
        for r in r_first..=r_last {
            let lat = g.lat0 - r * g.dlat;
            if lat < lat_lo || lat >= lat_hi {
                continue;
            }
            let dx = SlopeClassifier::dx_mm(lat, g.dlon);
            let pr = (s.post_of(lat) - b.i0()) as usize;
            for c in c_first..=c_last {
                let lon = g.lon0 + c * g.dlon;
                if lon < lon_lo || lon >= lon_hi {
                    continue;
                }
                let pc = (s.post_of(lon) - b.j0()) as usize;
                let mut w = [DEM_NONE; 9];
                let mut ok = true;
                for (i, (dr, dc)) in [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 0), (0, 1), (1, -1), (1, 0), (1, 1)].iter().enumerate() {
                    match m.pixel(k, r + dr, c + dc) {
                        Some(v) => w[i] = v,
                        None => {
                            ok = false;
                            break;
                        }
                    }
                }
                if !ok {
                    continue;
                }
                if let Some(cls) = cl.class(&w, dx, dy) {
                    let o = &mut out[pr * BLOCK + pc];
                    if *o == 0xFF || cls > *o {
                        *o = cls;
                    }
                }
            }
        }
    }
    out
}

/// Slope class of each interior post computed on the lattice grid itself (what a reader of
/// the resampled DEM alone would see); edges of the block are 0xFF.
pub fn lattice_slope_block(dm: &[i32], s: Spacing, b: BlockId, cl: &SlopeClassifier) -> Vec<u8> {
    let mut out = vec![0xFFu8; BLOCK_POSTS];
    let dy = SlopeClassifier::dy_mm(s.0);
    for r in 1..BLOCK - 1 {
        let dx = SlopeClassifier::dx_mm(s.pos(b.i0() + r as i64), s.0);
        for c in 1..BLOCK - 1 {
            // north row first: block rows run south to north
            let mut w = [0i32; 9];
            for (i, (dr, dc)) in [(1i64, -1i64), (1, 0), (1, 1), (0, -1), (0, 0), (0, 1), (-1, -1), (-1, 0), (-1, 1)].iter().enumerate() {
                w[i] = dm[((r as i64 + dr) as usize) * BLOCK + (c as i64 + dc) as usize];
            }
            if let Some(x) = cl.class(&w, dx, dy) {
                out[r * BLOCK + c] = x;
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn slope_classes() {
        let cl = SlopeClassifier::new();
        // 30 m cells, 1 m rise per cell east: ~1.9 degrees -> class 0
        let row = |a: i32| [a, a + 10, a + 20];
        let mk = |step: i32| {
            let r = [0, step, 2 * step];
            [r[0], r[1], r[2], r[0], r[1], r[2], r[0], r[1], r[2]]
        };
        let _ = row;
        assert_eq!(cl.class(&mk(10), 30_000, 30_000), Some(0));
        // 30 m rise over 30 m: 45 degrees exactly is not > 45 -> class 10
        assert_eq!(cl.class(&mk(300), 30_000, 30_000), Some(10));
        assert_eq!(cl.class(&mk(301), 30_000, 30_000), Some(11));
        assert_eq!(cl.class(&mk(100_000), 30_000, 30_000), Some(15));
    }
}

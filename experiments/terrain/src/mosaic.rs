//! A mosaic of source rasters indexed by their own geotransforms (never by file name).
//! Lookups are exact integer tick arithmetic. Shared edge rows (HGT, NASADEM) are allowed
//! and counted when they disagree; the first raster in (north, west, name) order wins.

use crate::lattice::div_round;
use crate::raster::{DEM_NONE, Raster};
use std::sync::atomic::{AtomicU64, Ordering};

pub struct Mosaic<T> {
    pub rasters: Vec<Raster<T>>,
    /// Corners that fell outside the home raster and on no pixel centre of any raster
    /// (spacing changes, e.g. GLO-30 at 50 N); dropped and the weights renormalised.
    pub misaligned: AtomicU64,
}

pub struct Home {
    pub k: usize,
    pub r: usize,
    pub fy: i64,
    pub c: usize,
    pub fx: i64,
}

impl<T: Copy + PartialEq> Mosaic<T> {
    pub fn new(mut rasters: Vec<Raster<T>>) -> Self {
        rasters.sort_by(|a, b| (-a.grid.lat0, a.grid.lon0, &a.name).cmp(&(-b.grid.lat0, b.grid.lon0, &b.name)));
        Mosaic { rasters, misaligned: AtomicU64::new(0) }
    }

    /// The raster whose pixel cells, extended one step east and south, hold the point.
    pub fn home(&self, lat: i64, lon: i64) -> Option<Home> {
        for (k, r) in self.rasters.iter().enumerate() {
            let g = &r.grid;
            let dy = g.lat0 - lat;
            let dx = lon - g.lon0;
            if dy < 0 || dx < 0 || dy >= g.height as i64 * g.dlat || dx >= g.width as i64 * g.dlon {
                continue;
            }
            return Some(Home { k, r: (dy / g.dlat) as usize, fy: dy % g.dlat, c: (dx / g.dlon) as usize, fx: dx % g.dlon });
        }
        None
    }

    /// The value at a pixel centre exactly at (lat, lon), from the first raster that has one.
    pub fn exact(&self, lat: i64, lon: i64) -> Option<T> {
        for r in &self.rasters {
            let g = &r.grid;
            let dy = g.lat0 - lat;
            let dx = lon - g.lon0;
            if dy < 0 || dx < 0 || dy % g.dlat != 0 || dx % g.dlon != 0 {
                continue;
            }
            let (rr, cc) = ((dy / g.dlat) as usize, (dx / g.dlon) as usize);
            if rr < g.height && cc < g.width {
                return Some(r.at(rr, cc));
            }
        }
        None
    }

    /// Pixel (r, c) of raster k, or the neighbouring raster's pixel at that position.
    pub fn pixel(&self, k: usize, r: i64, c: i64) -> Option<T> {
        let ras = &self.rasters[k];
        let g = &ras.grid;
        if r >= 0 && c >= 0 && (r as usize) < g.height && (c as usize) < g.width {
            return Some(ras.at(r as usize, c as usize));
        }
        self.exact(g.lat0 - r * g.dlat, g.lon0 + c * g.dlon)
    }
}

impl Mosaic<i32> {
    /// Count shared-edge pixels that disagree between rasters (HGT/NASADEM edge rows).
    pub fn shared_edge_mismatches(&self) -> (u64, u64) {
        let (mut shared, mut bad) = (0u64, 0u64);
        for (k, r) in self.rasters.iter().enumerate() {
            let g = &r.grid;
            for rr in 0..g.height {
                for cc in [0usize, g.width - 1] {
                    let (lat, lon) = (g.lat0 - rr as i64 * g.dlat, g.lon0 + cc as i64 * g.dlon);
                    for (k2, r2) in self.rasters.iter().enumerate() {
                        if k2 <= k {
                            continue;
                        }
                        let g2 = &r2.grid;
                        let (dy, dx) = (g2.lat0 - lat, lon - g2.lon0);
                        if dy >= 0 && dx >= 0 && dy % g2.dlat == 0 && dx % g2.dlon == 0 {
                            let (a, b) = ((dy / g2.dlat) as usize, (dx / g2.dlon) as usize);
                            if a < g2.height && b < g2.width {
                                shared += 1;
                                if r2.at(a, b) != r.at(rr, cc) {
                                    bad += 1;
                                }
                            }
                        }
                    }
                }
            }
        }
        (shared, bad)
    }

    /// Bilinear height (decimetres) at a point; NoData corners are dropped and the weights
    /// renormalised; no valid corner gives `None`.
    pub fn bilinear(&self, lat: i64, lon: i64) -> Option<i32> {
        let h = self.home(lat, lon)?;
        let g = &self.rasters[h.k].grid;
        let (dy, dx) = (g.dlat, g.dlon);
        let corners = [
            (0i64, 0i64, (dy - h.fy) * (dx - h.fx)),
            (1, 0, h.fy * (dx - h.fx)),
            (0, 1, (dy - h.fy) * h.fx),
            (1, 1, h.fy * h.fx),
        ];
        let (mut num, mut den) = (0i128, 0i128);
        for (dr, dc, w) in corners {
            if w == 0 {
                continue;
            }
            match self.pixel(h.k, h.r as i64 + dr, h.c as i64 + dc) {
                Some(v) if v != DEM_NONE => {
                    num += v as i128 * w as i128;
                    den += w as i128;
                }
                Some(_) => {}
                None => {
                    self.misaligned.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        if den == 0 { None } else { Some(div_round(num, den) as i32) }
    }
}

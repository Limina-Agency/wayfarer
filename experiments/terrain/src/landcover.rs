//! WorldCover onto the lattice: per post, the majority class of the source pixels whose
//! centres fall in the post's cell `[p - s/2, p + s/2)` in both axes (half-open, so every
//! pixel counts in exactly one cell). Ties go to the class with the higher priority below
//! (barriers and hazards first, so a tie never hides water or rock).

use crate::lattice::{BLOCK, BLOCK_POSTS, BlockId, Spacing};
use crate::mosaic::Mosaic;

/// WorldCover code -> 4-bit class. 0 = none. 12-15 are left for OSM-only classes
/// (rock/scree, dwarf pine, marsh, ...), see E3.
pub fn wc_class(code: u8) -> u8 {
    match code {
        10 => 1,  // tree cover
        20 => 2,  // shrubland
        30 => 3,  // grassland
        40 => 4,  // cropland
        50 => 5,  // built-up
        60 => 6,  // bare / sparse vegetation
        70 => 7,  // snow and ice
        80 => 8,  // permanent water
        90 => 9,  // herbaceous wetland
        95 => 10, // mangroves
        100 => 11, // moss and lichen
        _ => 0,
    }
}

pub const CLASS_NAMES: [&str; 12] =
    ["none", "tree", "shrub", "grass", "crop", "built", "bare", "snow", "water", "wetland", "mangrove", "moss"];

/// Tie priority per 4-bit class (higher wins).
const PRIORITY: [u8; 16] = [0, 6, 5, 3, 2, 9, 8, 7, 11, 10, 1, 4, 0, 0, 0, 0];

pub struct LcBlock {
    pub class: Vec<u8>,
    /// Majority count and total valid pixels per post (purity = maj / total).
    pub maj: Vec<u16>,
    pub total: Vec<u16>,
}

pub fn lc_block(m: &Mosaic<u8>, s: Spacing, b: BlockId) -> LcBlock {
    let mut hist = vec![[0u16; 12]; BLOCK_POSTS];
    let half = s.0 / 2;
    let lat_lo = s.pos(b.i0()) - half;
    let lat_hi = s.pos(b.i0() + BLOCK as i64 - 1) + half;
    let lon_lo = s.pos(b.j0()) - half;
    let lon_hi = s.pos(b.j0() + BLOCK as i64 - 1) + half;
    for ras in &m.rasters {
        let g = &ras.grid;
        let r_first = (g.lat0 - (lat_hi - 1)).div_euclid(g.dlat).max(0);
        let r_last = (g.lat0 - lat_lo).div_euclid(g.dlat).min(g.height as i64 - 1);
        let c_first = (lon_lo - g.lon0 + g.dlon - 1).div_euclid(g.dlon).max(0);
        let c_last = (lon_hi - 1 - g.lon0).div_euclid(g.dlon).min(g.width as i64 - 1);
        if r_first > r_last || c_first > c_last {
            continue;
        }
        for r in r_first..=r_last {
            let lat = g.lat0 - r * g.dlat;
            if lat < lat_lo || lat >= lat_hi {
                continue;
            }
            let pr = (s.post_of(lat) - b.i0()) as usize;
            let row = &ras.data[r as usize * g.width..(r as usize + 1) * g.width];
            for c in c_first..=c_last {
                let lon = g.lon0 + c * g.dlon;
                if lon < lon_lo || lon >= lon_hi {
                    continue;
                }
                let cls = wc_class(row[c as usize]);
                if cls == 0 {
                    continue;
                }
                let pc = (s.post_of(lon) - b.j0()) as usize;
                hist[pr * BLOCK + pc][cls as usize] += 1;
            }
        }
    }
    let mut out = LcBlock { class: vec![0; BLOCK_POSTS], maj: vec![0; BLOCK_POSTS], total: vec![0; BLOCK_POSTS] };
    for (p, h) in hist.iter().enumerate() {
        let mut best = 0usize;
        let mut tot = 0u16;
        for cls in 1..12 {
            tot += h[cls];
            if h[cls] > h[best] || (h[cls] == h[best] && h[cls] > 0 && PRIORITY[cls] > PRIORITY[best]) {
                best = cls;
            }
        }
        out.class[p] = if h[best] > 0 { best as u8 } else { 0 };
        out.maj[p] = h[best];
        out.total[p] = tot;
    }
    out
}

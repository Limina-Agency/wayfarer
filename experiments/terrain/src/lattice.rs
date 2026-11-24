//! The format-3 global lattice (docs/design/format-3.md section 9), in integer ticks.
//!
//! Positions are counted in ticks of 1/120 arc-second, so every spacing this experiment
//! touches is a whole number of ticks: 1/2" = 60, 1" = 120, 2" = 240, 3" = 360, the
//! GLO-30 longitude spacing above 50 N (1.5") = 180, and the WorldCover pixel (0.3") = 36.
//! Format 3 states spacings in whole arc-seconds; 60 ticks (1/2") is here only to measure
//! what a sub-arc-second spacing would cost.

pub const TICKS_PER_ARCSEC: i64 = 120;
pub const TICKS_PER_DEG: i64 = 3600 * TICKS_PER_ARCSEC;
pub const BLOCK: usize = 64;
pub const BLOCK_POSTS: usize = BLOCK * BLOCK;

/// A lattice spacing in ticks; it must divide a whole degree, as format 3 requires of
/// arc-second spacings (whole degrees are post lines).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Spacing(pub i64);

impl Spacing {
    pub fn new(ticks: i64) -> Result<Self, String> {
        if ticks <= 0 || TICKS_PER_DEG % ticks != 0 || ticks % 2 != 0 {
            return Err(format!("spacing {ticks} ticks does not divide a degree into even ticks"));
        }
        Ok(Spacing(ticks))
    }
    /// Latitude (ticks) of post row `i`.
    pub fn pos(self, i: i64) -> i64 {
        i * self.0
    }
    /// Post whose cell `[i*s - s/2, i*s + s/2)` holds the position `t` (ticks).
    pub fn post_of(self, t: i64) -> i64 {
        (t + self.0 / 2).div_euclid(self.0)
    }
}

/// A 64 x 64 block `(bi, bj)` = `(floor(i / 64), floor(j / 64))`; posts are stored row-major
/// from the south-west corner: index `r * 64 + c` is post `(bi*64 + r, bj*64 + c)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BlockId {
    pub bi: i32,
    pub bj: i32,
}

impl BlockId {
    pub fn i0(self) -> i64 {
        self.bi as i64 * BLOCK as i64
    }
    pub fn j0(self) -> i64 {
        self.bj as i64 * BLOCK as i64
    }
    /// The u64 key format 3 sorts blocks by (lexicographic on (bi, bj), signed).
    pub fn key(self) -> u64 {
        (((self.bi as u32 ^ 0x8000_0000) as u64) << 32) | (self.bj as u32 ^ 0x8000_0000) as u64
    }
}

/// Integer division rounding half away from zero (the rule every resampler here uses).
pub fn div_round(num: i128, den: i128) -> i128 {
    debug_assert!(den > 0);
    if num >= 0 { (2 * num + den) / (2 * den) } else { -((-2 * num + den) / (2 * den)) }
}

pub fn read_blocks(path: &str) -> Result<Vec<BlockId>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
    let mut out = Vec::new();
    for line in text.lines() {
        let mut it = line.split_whitespace();
        let (Some(a), Some(b)) = (it.next(), it.next()) else { continue };
        out.push(BlockId { bi: a.parse().map_err(|e| format!("{e}"))?, bj: b.parse().map_err(|e| format!("{e}"))? });
    }
    out.sort();
    out.dedup();
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rounding() {
        assert_eq!(div_round(5, 10), 1);
        assert_eq!(div_round(4, 10), 0);
        assert_eq!(div_round(-5, 10), -1);
        assert_eq!(div_round(-4, 10), 0);
        assert_eq!(div_round(15, 10), 2);
        assert_eq!(div_round(-15, 10), -2);
    }
    #[test]
    fn cells() {
        let s = Spacing::new(360).unwrap();
        assert_eq!(s.post_of(-180), 0);
        assert_eq!(s.post_of(179), 0);
        assert_eq!(s.post_of(180), 1);
        assert_eq!(s.post_of(-181), -1);
        assert!(BlockId { bi: -1, bj: 5 }.key() < BlockId { bi: 0, bj: 0 }.key());
    }
}

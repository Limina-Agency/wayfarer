//! The global lattice of `docs/design/format-3.md` section 9, in integer ticks.
//!
//! Positions are counted in ticks of 1/120 arc-second, anchored at 0°, 0°. Post `(i, j)` sits
//! at latitude `i * s` and longitude `j * s` ticks for a spacing of `s` ticks, and owns the cell
//! `[p - s/2, p + s/2)` on each axis. Posts are grouped in 64 × 64 blocks, stored row-major from
//! the south-west corner, so neighbouring regions tile exactly.

use crate::{Error, Result};

pub const TICKS_PER_ARCSEC: i64 = 120;
pub const TICKS_PER_DEG: i64 = 3600 * TICKS_PER_ARCSEC;
/// Posts per block side.
pub const BLOCK: usize = 64;
/// Posts per block.
pub const BLOCK_POSTS: usize = BLOCK * BLOCK;

/// A lattice spacing in ticks (120 = 1"). It divides a whole degree into an even number of
/// ticks, so whole degrees are post lines and cell edges fall on whole ticks.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Spacing(i64);

impl Spacing {
    pub fn new(ticks: i64) -> Result<Self> {
        if ticks <= 0 || TICKS_PER_DEG % ticks != 0 || ticks % 2 != 0 {
            return Err(Error::invalid(format!(
                "spacing {ticks} ticks does not divide a degree into even ticks"
            )));
        }
        Ok(Spacing(ticks))
    }
    /// The spacing in ticks.
    pub fn ticks(self) -> i64 {
        self.0
    }
    /// Position (ticks) of post row or column `i`.
    pub fn pos(self, i: i64) -> i64 {
        i * self.0
    }
    /// The post whose cell `[i*s - s/2, i*s + s/2)` holds position `t` (ticks).
    pub fn post_of(self, t: i64) -> i64 {
        (t + self.0 / 2).div_euclid(self.0)
    }
}

/// A 64 × 64 block `(bi, bj)` = `(floor(i / 64), floor(j / 64))`. Post index `r * 64 + c`
/// within it is post `(bi*64 + r, bj*64 + c)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BlockId {
    pub bi: i32,
    pub bj: i32,
}

impl BlockId {
    /// First post row.
    pub fn i0(self) -> i64 {
        self.bi as i64 * BLOCK as i64
    }
    /// First post column.
    pub fn j0(self) -> i64 {
        self.bj as i64 * BLOCK as i64
    }
    /// The u64 key blocks are sorted and indexed by: (bi, bj) lexicographic, signed.
    pub fn key(self) -> u64 {
        (((self.bi as u32 ^ 0x8000_0000) as u64) << 32) | (self.bj as u32 ^ 0x8000_0000) as u64
    }
    pub fn from_key(key: u64) -> BlockId {
        BlockId {
            bi: ((key >> 32) as u32 ^ 0x8000_0000) as i32,
            bj: (key as u32 ^ 0x8000_0000) as i32,
        }
    }
    /// The block of post `(i, j)` and the post's index in it.
    pub fn of_post(i: i64, j: i64) -> (BlockId, usize) {
        let (bi, bj) = (i.div_euclid(BLOCK as i64), j.div_euclid(BLOCK as i64));
        let q = (i.rem_euclid(BLOCK as i64) * BLOCK as i64 + j.rem_euclid(BLOCK as i64)) as usize;
        (
            BlockId {
                bi: bi as i32,
                bj: bj as i32,
            },
            q,
        )
    }
}

/// Integer division rounding half away from zero: the rounding rule of every step.
pub fn div_round(num: i128, den: i128) -> i128 {
    debug_assert!(den > 0);
    if num >= 0 {
        (2 * num + den) / (2 * den)
    } else {
        -((-2 * num + den) / (2 * den))
    }
}

/// `div_round` in i64, for hot paths.
#[inline]
pub fn div_round_i64(num: i64, den: i64) -> i64 {
    if num >= 0 {
        (2 * num + den) / (2 * den)
    } else {
        -((-2 * num + den) / (2 * den))
    }
}

/// Ceiling division for a positive divisor.
pub fn ceil_div(a: i64, b: i64) -> i64 {
    -((-a).div_euclid(b))
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
        assert_eq!(div_round_i64(-15, 10), -2);
        assert_eq!(ceil_div(7, 2), 4);
        assert_eq!(ceil_div(-7, 2), -3);
    }

    #[test]
    fn cells_and_keys() {
        let s = Spacing::new(360).unwrap();
        assert_eq!(s.post_of(-180), 0);
        assert_eq!(s.post_of(179), 0);
        assert_eq!(s.post_of(180), 1);
        assert_eq!(s.post_of(-181), -1);
        assert!(Spacing::new(7).is_err());
        assert!(Spacing::new(0).is_err());
        let b = BlockId { bi: -1, bj: 5 };
        assert!(b.key() < BlockId { bi: 0, bj: 0 }.key());
        assert_eq!(BlockId::from_key(b.key()), b);
        assert_eq!(
            BlockId::of_post(-1, 64),
            (BlockId { bi: -1, bj: 1 }, 63 * 64)
        );
    }
}

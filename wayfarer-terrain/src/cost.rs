//! The cost rules (`docs/design/incident-surface.md` section 5): one post's time per metre,
//! the risk that it is impassable, and why a closed post is closed.
//!
//! In order: a team block or a washout closes the cell; a usable way sets the flat time
//! (a motorway with no usable way closes it); off ways, the class mass with no time per metre
//! closes the cell at the profile's `block_at`, otherwise the time is the expectation over the
//! passable mass, then steepness and barriers scale or close it; a blocking hazard at or above
//! `haz_block_at` closes it unless a measured level supersedes it; other hazards and levels
//! slow it or close it above a depth; an obstruction slows or closes it.

use crate::fuse::{M_BARRIER, M_DEBRIS, M_FALLEN, M_PASSAGE, M_TEAM_BLOCK, M_WASHOUT, S};
use crate::lattice::div_round;
use crate::layer::{NCLASS, NHAZ, NLEV};
use crate::lines::*;
use crate::profile::Rules;
use crate::terrain::STEEP_NONE;

/// Why a cell is impassable: `reason << 4 | channel` in the `why` plane.
pub const WHY_CLASS: u8 = 1;
pub const WHY_STEEP: u8 = 2;
/// Channel: the barrier's index in [`BARRIERS`].
pub const WHY_BARRIER: u8 = 3;
pub const WHY_MOTORWAY: u8 = 4;
/// Channel: the hazard slot.
pub const WHY_HAZARD: u8 = 5;
/// Channel: the level slot.
pub const WHY_LEVEL: u8 = 6;
pub const WHY_OBSTRUCTION: u8 = 7;
pub const WHY_WASHOUT: u8 = 8;
pub const WHY_TEAM: u8 = 9;

/// A blocking hazard is superseded by a measured level: (hazard slot, level slot).
/// Flood by water depth, snow cover by snow depth.
pub const SUPERSEDE: [(usize, usize); 2] = [(0, 0), (4, 1)];

/// The line bits a fused effect mask adds to the base mask.
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cell {
    /// Time per metre in the profile's unit; 0 = impassable.
    pub t: u16,
    /// Probability (1/255) that the cell is impassable.
    pub risk: u8,
    /// 0 when passable, else `reason << 4 | channel`.
    pub why: u8,
}

fn closed(why: u8, sub: usize, risk: u8) -> Cell {
    Cell {
        t: 0,
        risk,
        why: why << 4 | sub as u8,
    }
}

/// `1 - (1 - Pb/S) prod (1 - h/255)` over the blocking, non-superseded hazards, in 1/255.
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

/// The cost of one post from its fused state and the base's steepness and lines.
pub fn cell(
    r: &Rules,
    p: &[u16; NCLASS],
    haz: &[u8; NHAZ],
    lev: &[(u16, u8); NLEV],
    mask: u16,
    steep: u8,
    lines: u16,
) -> Cell {
    let (pr, x) = (&r.p, &r.x);
    let l = lines | effect_lines(mask);
    if l & TEAM_BLOCK != 0 {
        return closed(WHY_TEAM, 0, 255);
    }
    if l & WASHOUT != 0 {
        return closed(WHY_WASHOUT, 0, 255);
    }
    let mut blocking = [false; NHAZ];
    for (s, b) in blocking.iter_mut().enumerate() {
        *b = x.haz_block >> s & 1 == 1;
    }
    for (h, lv) in SUPERSEDE {
        if lev[lv].1 > 0 {
            blocking[h] = false;
        }
    }
    let ways = l & WAYS;
    let way = (0..8)
        .filter(|k| ways & (1 << k) != 0)
        .map(|k| pr.way_t[k])
        .filter(|&t| t > 0)
        .min();
    let mut pb = 0u32;
    let mut t: u64;
    if let Some(tw) = way {
        t = tw as u64;
    } else if ways & MOTORWAY != 0 {
        return closed(WHY_MOTORWAY, 0, 255);
    } else {
        let mut acc = 0u64;
        for (&tk, &pk) in pr.cover_t.iter().zip(p) {
            if tk == 0 {
                pb += pk as u32;
            } else {
                acc += tk as u64 * pk as u64;
            }
        }
        if pb >= x.block_at {
            return closed(WHY_CLASS, 0, risk_of(pb, haz, &blocking));
        }
        t = div_round(acc as i128, (S - pb) as i128) as u64;
        let sr = if steep == STEEP_NONE {
            1000
        } else {
            pr.steep_rel[steep as usize & 15]
        };
        if sr == 0 {
            return closed(WHY_STEEP, 0, 255);
        }
        let mut rel: u64 = sr as u64;
        for (k, bit) in BARRIERS.iter().enumerate() {
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
    for (s, &(v, c)) in lev.iter().enumerate() {
        if c == 0 {
            continue;
        }
        if v > x.lev_block[s] {
            return closed(WHY_LEVEL, s, 255);
        }
        if x.lev_k[s] > 0 {
            t = div_round(
                t as i128 * (1000 + v as u64 * x.lev_k[s] as u64) as i128,
                1000,
            ) as u64;
        }
    }
    if l & OBSTRUCTION != 0 {
        if x.obst_rel == 0 {
            return closed(WHY_OBSTRUCTION, 0, 255);
        }
        t = div_round(t as i128 * 1000, x.obst_rel as i128) as u64;
    }
    Cell {
        t: t.clamp(1, 65535) as u16,
        risk: risk_of(pb, haz, &blocking),
        why: 0,
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

    /// E5's one-hot cell rule, the reference the cost rules must equal with no layers.
    fn e5_cell_t(r: &Rules, code: usize, steep: u8, lines: u16) -> u32 {
        let p = &r.p;
        let ways = lines & WAYS;
        if ways != 0 {
            if let Some(t) = (0..8)
                .filter(|k| ways & (1 << k) != 0)
                .map(|k| p.way_t[k])
                .filter(|&t| t > 0)
                .min()
            {
                return t;
            }
            if ways & MOTORWAY != 0 {
                return 0;
            }
        }
        let t = p.cover_t[code];
        if t == 0 {
            return 0;
        }
        let sr = if steep == 0xFF {
            1000
        } else {
            p.steep_rel[steep as usize & 15]
        };
        if sr == 0 {
            return 0;
        }
        let mut rel = sr as u64;
        for (k, bit) in BARRIERS.iter().enumerate() {
            if lines & bit != 0 {
                if p.barrier_rel[k] == 0 {
                    return 0;
                }
                rel = rel * p.barrier_rel[k] as u64 / 1000;
            }
        }
        (div_round(t as i128 * 1000, rel.max(1) as i128) as u64).clamp(1, 65535) as u32
    }

    #[test]
    fn one_hot_is_the_base() {
        for r in [Rules::walker_default(), Rules::vehicle()] {
            for c in 0..16 {
                for steep in [0u8, 9, 12, STEEP_NONE] {
                    for lines in [
                        0u16,
                        PATH,
                        RIVER,
                        STREAM | FENCE,
                        MOTORWAY,
                        MOTORWAY | MINOR,
                        CLIFF | TRACK,
                    ] {
                        let e5 = e5_cell_t(&r, c, steep, lines);
                        let got =
                            cell(&r, &onehot(c), &[0; NHAZ], &[(0, 0); NLEV], 0, steep, lines);
                        assert_eq!(got.t as u32, e5, "class {c} steep {steep} lines {lines:x}");
                        assert_eq!(got.risk, if e5 > 0 { 0 } else { 255 });
                    }
                }
            }
        }
    }

    #[test]
    fn blocking_mass_and_hazards() {
        let r = Rules::walker_default();
        let mut p = onehot(3);
        p[3] = S as u16 - 82; // 2 % water
        p[8] = 82;
        let c = cell(&r, &p, &[0; NHAZ], &[(0, 0); NLEV], 0, 0, 0);
        assert!(c.t > 0 && c.risk == 5, "{c:?}");
        // a meadow with 60 % flood probability blocks; a measured shallow depth lifts it
        let mut h = [0u8; NHAZ];
        h[0] = 153;
        assert_eq!(
            cell(&r, &onehot(3), &h, &[(0, 0); NLEV], 0, 0, 0).why,
            WHY_HAZARD << 4
        );
        let lev = [(20, 200), (0, 0), (0, 0)];
        let c = cell(&r, &onehot(3), &h, &lev, 0, 0, 0);
        assert!(c.t > 0 && c.risk == 0, "{c:?}");
        // deep water closes it
        assert_eq!(
            cell(&r, &onehot(3), &h, &[(80, 200), (0, 0), (0, 0)], 0, 0, 0).why,
            WHY_LEVEL << 4
        );
        // team block beats a path; fallen trees slow a walker and close the 4x4
        assert_eq!(
            cell(
                &r,
                &onehot(3),
                &[0; NHAZ],
                &[(0, 0); NLEV],
                M_TEAM_BLOCK,
                0,
                PATH
            )
            .t,
            0
        );
        let on_path = cell(&r, &onehot(3), &[0; NHAZ], &[(0, 0); NLEV], 0, 0, PATH).t;
        assert_eq!(
            cell(
                &r,
                &onehot(3),
                &[0; NHAZ],
                &[(0, 0); NLEV],
                M_FALLEN,
                0,
                PATH
            )
            .t as u64,
            div_round(on_path as i128 * 1000, 400) as u64
        );
        assert_eq!(
            cell(
                &Rules::vehicle(),
                &onehot(3),
                &[0; NHAZ],
                &[(0, 0); NLEV],
                M_FALLEN,
                0,
                TRACK
            )
            .why,
            WHY_OBSTRUCTION << 4
        );
    }
}

//! E5: an incident cost surface on the lattice, and integer 16-connected Dijkstra / A* on it.
//!
//! Stage 2 (fusion) builds, per profile, a flat time per metre for every cell (u16 in the
//! profile's unit, 0 = impassable) from the stage-1 base layers: a land-cover class
//! distribution over E3's 4-bit codes (one-hot today; ML layers give a real vector later),
//! the steepness-max class and the linear-feature bitmask. Slope is not baked in: it depends on
//! the direction of travel, so the search applies a pace-by-grade table per edge from the DEM.
//!
//! Edge cost (microseconds), for a move of length L mm through the cells it crosses (the two
//! end cells, plus the two cells a knight move passes between them):
//!   cost = round(L * sum(t) * unit_us / (n * rel(grade)))
//! with rel the per-mille speed for the edge's grade (whole percent, from the DEM ends). A
//! knight move needs both crossed cells passable, so a 4-connected barrier line cannot be
//! jumped. Everything is integer; ground lengths come from libm once per half-row.

use crate::dem::SlopeClassifier;
use crate::encode::put_varint;
use crate::lattice::{BLOCK, BLOCK_POSTS, BlockId, Spacing, div_round};
use crate::lines::*;
use rayon::prelude::*;
use std::cmp::Reverse;
use std::collections::BinaryHeap;

/// Fixed-point scale of a class distribution: probabilities sum exactly to this.
pub const PSCALE: u32 = 1 << 12;

pub struct Profile {
    pub name: &'static str,
    /// Microseconds per stored time unit (walker 1 ms/m -> 1000, 4x4 0.1 ms/m -> 100).
    pub unit_us: u64,
    /// Flat time per metre per E3 code 0-15, in units (0 = impassable).
    pub cover_t: [u32; 16],
    /// Flat time per metre on a way of each kind (bits 0-7; 0 = not usable).
    pub way_t: [u32; 8],
    /// Speed multiplier per steepness-max class (per mille; 0 = impassable), off-way only.
    pub steep_rel: [u32; 16],
    /// Speed multiplier for river, stream, rail, fence, cliff (per mille; 0 = impassable), off-way only.
    pub barrier_rel: [u32; 5],
    /// Speed per mille by grade, -100 %..+100 % (index grade + 100).
    pub grade_rel: Vec<u32>,
}

fn t_of(speed_mmps: f64, unit_us: u64) -> u32 {
    if speed_mmps <= 0.0 { 0 } else { libm::round(1e9 / speed_mmps / unit_us as f64) as u32 }
}

/// Walker: E4 expected-pace curve (1.04 m/s flat; `grade_rel` from E4's pace_table_50.csv),
/// E4 trail factors, feat-x.md X3 cover factors plus E3's dwarf pine 0.15. Steepness and
/// barrier factors are E5 assumptions.
pub fn walker(grade_rel: Vec<u32>) -> Profile {
    let v0 = 1040.0;
    // E3 codes: 0 none, 1 tree, 2 shrub, 3 grass, 4 crop, 5 built, 6 bare, 7 snow, 8 water,
    // 9 wetland, 10 mangrove, 11 moss, 12 dwarf pine, 13 rock, 14 scree, 15 bog/swamp forest
    let f = [0.6, 0.6, 0.35, 0.8, 0.7, 0.5, 0.4, 0.5, 0.0, 0.25, 0.25, 0.7, 0.15, 0.4, 0.4, 0.2];
    let w = [0.0, 1.08, 1.08, 1.09, 1.00, 0.85, 0.83, 0.92];
    Profile {
        name: "walker",
        unit_us: 1000,
        cover_t: f.map(|x| t_of(v0 * x, 1000)),
        way_t: w.map(|x| t_of(v0 * x, 1000)),
        steep_rel: [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 800, 600, 400, 250, 0, 0, 0, 0],
        barrier_rel: [0, 600, 500, 500, 0],
        grade_rel,
    }
}

/// Light 4x4: roads and tracks at fixed speeds, open ground slow, forest and wet ground no-go.
/// All E5 assumptions (PLAN.md section 6 item 7: go / slow-go / no-go).
pub fn vehicle() -> Profile {
    let kmh = |x: f64| t_of(x / 3.6 * 1000.0, 100);
    let f = [0.0, 0.0, 3.0, 12.0, 8.0, 5.0, 6.0, 0.0, 0.0, 0.0, 0.0, 8.0, 0.0, 0.0, 0.0, 0.0];
    let w = [0.0, 60.0, 40.0, 20.0, 8.0, 0.0, 0.0, 0.0];
    let grade_rel = (-100..=100).map(|g: i32| if g.abs() <= 10 { 1000 } else { (1000 - 23 * (g.abs() - 10)).max(300) as u32 }).collect();
    Profile {
        name: "vehicle",
        unit_us: 100,
        cover_t: f.map(kmh),
        way_t: w.map(kmh),
        steep_rel: [1000, 1000, 1000, 1000, 700, 400, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        barrier_rel: [0, 300, 0, 0, 0],
        grade_rel,
    }
}

/// Base layers on a dense rectangle of posts, rows from the south.
pub struct Base {
    pub s: Spacing,
    pub bi0: i64,
    pub bj0: i64,
    pub h: usize,
    pub w: usize,
    pub dm: Vec<i32>,
    pub steep: Vec<u8>,
    pub lc: Vec<u8>,
    pub lines: Vec<u16>,
}

impl Base {
    pub fn from_blocks(s: Spacing, blocks: &[BlockId], dm: &[i32], steep: &[u8], lc: &[u8], lines: &[u16]) -> Base {
        let bi0 = blocks.iter().map(|b| b.bi).min().unwrap() as i64;
        let bi1 = blocks.iter().map(|b| b.bi).max().unwrap() as i64;
        let bj0 = blocks.iter().map(|b| b.bj).min().unwrap() as i64;
        let bj1 = blocks.iter().map(|b| b.bj).max().unwrap() as i64;
        assert_eq!(blocks.len() as i64, (bi1 - bi0 + 1) * (bj1 - bj0 + 1), "surface: blocks must form a rectangle");
        let (h, w) = (((bi1 - bi0 + 1) * BLOCK as i64) as usize, ((bj1 - bj0 + 1) * BLOCK as i64) as usize);
        let mut b = Base { s, bi0, bj0, h, w, dm: vec![0; h * w], steep: vec![0; h * w], lc: vec![0; h * w], lines: vec![0; h * w] };
        for (k, blk) in blocks.iter().enumerate() {
            let (r0, c0) = ((blk.bi as i64 - bi0) as usize * BLOCK, (blk.bj as i64 - bj0) as usize * BLOCK);
            for r in 0..BLOCK {
                for c in 0..BLOCK {
                    let (src, dst) = (k * BLOCK_POSTS + r * BLOCK + c, (r0 + r) * w + c0 + c);
                    b.dm[dst] = dm[src];
                    b.steep[dst] = steep[src];
                    b.lc[dst] = lc[src];
                    b.lines[dst] = lines[src];
                }
            }
        }
        b
    }
}

/// Flat time per metre of one cell from a class distribution `(code, p)` with sum p = PSCALE:
/// the expected time per metre sum(p t) / PSCALE (time adds along a path, so this is the
/// right mean). Any probability mass on an impassable class blocks the cell (exact for
/// one-hot input; how much water should block a cell is a question for the layer contract).
/// A usable way on the cell overrides cover, steepness and barriers.
pub fn cell_t(p: &Profile, dist: &[(u8, u32)], steep: u8, lines: u16) -> u32 {
    let ways = lines & WAYS;
    if ways != 0 {
        let best = (0..8).filter(|k| ways & (1 << k) != 0).map(|k| p.way_t[k]).filter(|&t| t > 0).min();
        if let Some(t) = best {
            return t;
        }
        if ways & MOTORWAY != 0 {
            return 0;
        }
    }
    let mut acc: u64 = 0;
    for &(c, pr) in dist {
        let t = p.cover_t[c as usize & 15];
        if t == 0 && pr > 0 {
            return 0;
        }
        acc += t as u64 * pr as u64;
    }
    let mut t = div_round(acc as i128, PSCALE as i128) as u64;
    let mut rel: u64 = 1000;
    let sr = if steep == 0xFF { 1000 } else { p.steep_rel[steep as usize & 15] };
    if sr == 0 {
        return 0;
    }
    rel = rel * sr as u64 / 1000;
    for (k, bit) in [RIVER, STREAM, RAIL, FENCE, CLIFF].iter().enumerate() {
        if lines & bit != 0 {
            if p.barrier_rel[k] == 0 {
                return 0;
            }
            rel = rel * p.barrier_rel[k] as u64 / 1000;
        }
    }
    t = div_round(t as i128 * 1000, rel.max(1) as i128) as u64;
    t.clamp(1, 65535) as u32
}

/// Stage 2: the friction grid of one profile (u16 per cell, 0 = impassable).
pub fn build(p: &Profile, b: &Base) -> Vec<u16> {
    let mut out = vec![0u16; b.h * b.w];
    out.par_chunks_mut(b.w).enumerate().for_each(|(r, row)| {
        for c in 0..b.w {
            let i = r * b.w + c;
            let code = b.lc[i];
            row[c] = cell_t(p, &[(code, PSCALE)], b.steep[i], b.lines[i]) as u16;
        }
    });
    out
}

/// Per block: a palette of distinct values (count varint, then the values as varints, in
/// first-seen order), then runs over palette indices varint((len - 1) << k | idx) with k the
/// palette index width. Returns the blocks' payloads in key order.
pub fn encode_palette_runs(b: &Base, f: &[u16]) -> Vec<(BlockId, Vec<u8>)> {
    let (nbi, nbj) = (b.h / BLOCK, b.w / BLOCK);
    let mut out = Vec::new();
    for bi in 0..nbi {
        for bj in 0..nbj {
            let mut v = Vec::with_capacity(BLOCK_POSTS);
            for r in 0..BLOCK {
                let i = (bi * BLOCK + r) * b.w + bj * BLOCK;
                v.extend_from_slice(&f[i..i + BLOCK]);
            }
            let mut pal: Vec<u16> = Vec::new();
            let idx: Vec<usize> = v
                .iter()
                .map(|x| match pal.iter().position(|p| p == x) {
                    Some(k) => k,
                    None => {
                        pal.push(*x);
                        pal.len() - 1
                    }
                })
                .collect();
            let k = (usize::BITS - (pal.len().max(2) - 1).leading_zeros()) as u64;
            let mut buf = Vec::new();
            put_varint(&mut buf, pal.len() as u64);
            for &p in &pal {
                put_varint(&mut buf, p as u64);
            }
            let mut i = 0;
            while i < idx.len() {
                let mut j = i + 1;
                while j < idx.len() && idx[j] == idx[i] {
                    j += 1;
                }
                put_varint(&mut buf, (((j - i - 1) as u64) << k) | idx[i] as u64);
                i = j;
            }
            out.push((BlockId { bi: (b.bi0 as usize + bi) as i32, bj: (b.bj0 as usize + bj) as i32 }, buf));
        }
    }
    out
}

pub fn decode_palette_runs(buf: &[u8]) -> Option<Vec<u16>> {
    let mut pos = 0;
    let n = crate::encode::get_varint(buf, &mut pos)? as usize;
    let mut pal = Vec::with_capacity(n);
    for _ in 0..n {
        pal.push(crate::encode::get_varint(buf, &mut pos)? as u16);
    }
    let k = (usize::BITS - (n.max(2) - 1).leading_zeros()) as u64;
    let mut out = Vec::with_capacity(BLOCK_POSTS);
    while pos < buf.len() {
        let x = crate::encode::get_varint(buf, &mut pos)?;
        let len = (x >> k) as usize + 1;
        out.extend(std::iter::repeat_n(*pal.get((x & ((1 << k) - 1)) as usize)?, len));
    }
    (out.len() == BLOCK_POSTS).then_some(out)
}

/// The 16 moves: (dr, dc), and for knight moves the two crossed cells.
pub const MOVES: [(i32, i32); 16] =
    [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1), (1, 2), (2, 1), (2, -1), (1, -2), (-1, -2), (-2, -1), (-2, 1), (-1, 2)];

fn crossed(dr: i32, dc: i32) -> Option<[(i32, i32); 2]> {
    if dr.abs() == 1 && dc.abs() == 2 {
        Some([(0, dc / 2), (dr, dc / 2)])
    } else if dr.abs() == 2 && dc.abs() == 1 {
        Some([(dr / 2, 0), (dr / 2, dc)])
    } else {
        None
    }
}

/// `lattice::div_round` in i64 (same rounding, half away from zero), for the search's hot path.
#[inline]
fn div_round_i64(num: i64, den: i64) -> i64 {
    if num >= 0 { (2 * num + den) / (2 * den) } else { -((-2 * num + den) / (2 * den)) }
}

fn isqrt(n: u64) -> u64 {
    let mut x = (n as f64).sqrt() as u64;
    while x * x > n {
        x -= 1;
    }
    while (x + 1) * (x + 1) <= n {
        x += 1;
    }
    x
}

pub struct Graph<'a> {
    pub b: &'a Base,
    pub p: &'a Profile,
    pub t: &'a [u16],
    /// Move lengths (mm) per half-row k = 2r + dr (the move's mid-latitude) and move.
    len: Vec<[u32; 16]>,
    pub dy_mm: i64,
    pub dx_min_mm: i64,
    pub t_min: u64,
    pub rel_max: u64,
}

impl<'a> Graph<'a> {
    pub fn new(b: &'a Base, p: &'a Profile, t: &'a [u16]) -> Self {
        let s = b.s.0;
        let dy = SlopeClassifier::dy_mm(s);
        let lat0 = b.s.pos(b.bi0 * BLOCK as i64); // ticks of row 0
        let mut len = Vec::with_capacity(2 * b.h + 4);
        let mut dx_min = i64::MAX;
        for k in -2..(2 * b.h as i64 + 2) {
            let lat = lat0 * 2 + k * s; // in half-ticks... (lat0 + k s / 2) * 2
            let dx = SlopeClassifier::dx_mm(lat / 2, s);
            dx_min = dx_min.min(dx);
            let mut l = [0u32; 16];
            for (m, &(dr, dc)) in MOVES.iter().enumerate() {
                l[m] = isqrt(((dr as i64 * dy).pow(2) + (dc as i64 * dx).pow(2)) as u64) as u32;
            }
            len.push(l);
        }
        let t_min = t.iter().filter(|&&x| x > 0).min().copied().unwrap_or(1) as u64;
        let rel_max = *p.grade_rel.iter().max().unwrap() as u64;
        Graph { b, p, t, len, dy_mm: dy, dx_min_mm: dx_min, t_min, rel_max }
    }

    /// Cost (us) of the move m from cell (r, c), or None.
    #[inline]
    pub fn cost(&self, r: usize, c: usize, m: usize) -> Option<(usize, u64)> {
        let (dr, dc) = MOVES[m];
        let (nr, nc) = (r as i64 + dr as i64, c as i64 + dc as i64);
        if nr < 0 || nc < 0 || nr >= self.b.h as i64 || nc >= self.b.w as i64 {
            return None;
        }
        let w = self.b.w;
        let (a, z) = (r * w + c, nr as usize * w + nc as usize);
        let (ta, tz) = (self.t[a] as u64, self.t[z] as u64);
        if ta == 0 || tz == 0 {
            return None;
        }
        let (mut sum, mut n) = (ta + tz, 2u64);
        if let Some(x) = crossed(dr, dc) {
            for (er, ec) in x {
                let tx = self.t[(r as i64 + er as i64) as usize * w + (c as i64 + ec as i64) as usize] as u64;
                if tx == 0 {
                    return None;
                }
                sum += tx;
                n += 1;
            }
        }
        let l = self.len[(2 * r as i64 + dr as i64 + 2) as usize][m] as u64;
        let dh_mm = (self.b.dm[z] as i64 - self.b.dm[a] as i64) * 100;
        let g = div_round_i64(dh_mm * 100, l as i64).clamp(-100, 100);
        let rel = self.p.grade_rel[(g + 100) as usize] as u64;
        let (num, den) = (l * sum * self.p.unit_us, n * rel); // < 2^43, so u64 is exact
        let cost = (2 * num + den) / (2 * den);
        Some((z, cost.max(1)))
    }

    /// Admissible, consistent lower bound (us) from cell i to cell j.
    #[inline]
    pub fn h(&self, i: usize, j: usize) -> u64 {
        let w = self.b.w as i64;
        let (dr, dc) = ((i as i64 / w - j as i64 / w).abs(), (i as i64 % w - j as i64 % w).abs());
        let l = isqrt(((dr * self.dy_mm).pow(2) + (dc * self.dx_min_mm).pow(2)) as u64);
        // 0.999: room for the rounding of lengths and costs
        (l * self.t_min * self.p.unit_us / self.rel_max) * 999 / 1000
    }
}

pub struct Search {
    pub dist: Vec<u64>,
    stamp: Vec<u32>,
    epoch: u32,
    heap: BinaryHeap<Reverse<(u64, u32)>>,
    pub settled: u64,
    pub pushes: u64,
}

impl Search {
    pub fn new(n: usize) -> Self {
        Search { dist: vec![u64::MAX; n], stamp: vec![0; n], epoch: 0, heap: BinaryHeap::new(), settled: 0, pushes: 0 }
    }
    #[inline]
    fn d(&self, i: usize) -> u64 {
        if self.stamp[i] == self.epoch { self.dist[i] } else { u64::MAX }
    }
    /// Dijkstra (astar = false) or A* to `target`; with no target, the full field. Returns the
    /// target's cost. Ties are broken by cell index, so results are deterministic.
    pub fn run(&mut self, g: &Graph, src: usize, target: Option<usize>, astar: bool) -> Option<u64> {
        self.epoch += 1;
        self.heap.clear();
        self.settled = 0;
        self.pushes = 0;
        let w = g.b.w;
        let hof = |i: usize| if astar { g.h(i, target.unwrap()) } else { 0 };
        self.dist[src] = 0;
        self.stamp[src] = self.epoch;
        self.heap.push(Reverse((hof(src), src as u32)));
        while let Some(Reverse((k, i))) = self.heap.pop() {
            let i = i as usize;
            let di = self.d(i);
            if k != di + hof(i) {
                continue;
            }
            self.settled += 1;
            if Some(i) == target {
                return Some(di);
            }
            let (r, c) = (i / w, i % w);
            for m in 0..16 {
                if let Some((z, cost)) = g.cost(r, c, m) {
                    let nd = di + cost;
                    if nd < self.d(z) {
                        self.dist[z] = nd;
                        self.stamp[z] = self.epoch;
                        self.heap.push(Reverse((nd + hof(z), z as u32)));
                        self.pushes += 1;
                    }
                }
            }
        }
        if target.is_none() { Some(0) } else { None }
    }
    pub fn field_ds(&self, n: usize) -> Vec<u32> {
        (0..n).map(|i| { let d = self.d(i); if d == u64::MAX { u32::MAX } else { div_round(d as i128, 100_000) as u32 } }).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn palette_roundtrip() {
        let b = Base { s: Spacing(120), bi0: 0, bj0: 0, h: 64, w: 128, dm: vec![], steep: vec![], lc: vec![], lines: vec![] };
        let f: Vec<u16> = (0..64 * 128).map(|i| ((i / 7) % 5 * 300 + (i % 128 / 60)) as u16).collect();
        let enc = encode_palette_runs(&b, &f);
        for (k, (_, buf)) in enc.iter().enumerate() {
            let d = decode_palette_runs(buf).unwrap();
            for r in 0..64 {
                for c in 0..64 {
                    assert_eq!(d[r * 64 + c], f[r * 128 + k * 64 + c]);
                }
            }
        }
    }
    #[test]
    fn knight_needs_crossed_cells() {
        assert_eq!(crossed(1, 2), Some([(0, 1), (1, 1)]));
        assert_eq!(crossed(-2, 1), Some([(-1, 0), (-1, 1)]));
        assert_eq!(crossed(1, 1), None);
    }
}

//! Least-cost search on an incident surface: 16-connected integer Dijkstra and A*, canonical
//! paths and route confidence.
//!
//! Edge cost (microseconds) of a move of length L mm through the cells it crosses (the two end
//! cells, plus the two cells a knight move passes between them):
//!
//! ```text
//! cost = round(L * sum(t) * unit_us / (n * rel(grade)))
//! ```
//!
//! with `rel` the profile's per-mille speed for the move's grade (whole percent from the heights
//! at its ends). Slope is not baked into the surface because it depends on the direction of
//! travel. A knight move needs both crossed cells passable, so a 4-connected barrier line cannot
//! be jumped. Ties are broken by cell index, so every result is deterministic.

use crate::lattice::{BLOCK, BlockId, TICKS_PER_DEG, div_round, div_round_i64};
use crate::profile::Rules;
use crate::surface::{SRC_BASE, SRC_TEAM, Surface};
use crate::terrain::{DEM_NONE, Terrain};
use crate::{Error, Result};
use std::cmp::Reverse;
use std::collections::{BTreeMap, BTreeSet, BinaryHeap};

/// Mean Earth radius (IUGG), mm.
pub const EARTH_R_MM: f64 = 6_371_008_800.0;

/// Ground length (mm) of `ticks` of longitude at latitude `lat` ticks.
pub fn dx_mm(lat: i64, ticks: i64) -> i64 {
    let phi = lat as f64 / TICKS_PER_DEG as f64 * core::f64::consts::PI / 180.0;
    libm::round(
        ticks as f64 / TICKS_PER_DEG as f64 * core::f64::consts::PI / 180.0
            * EARTH_R_MM
            * libm::cos(phi),
    ) as i64
}

/// Ground length (mm) of `ticks` of latitude.
pub fn dy_mm(ticks: i64) -> i64 {
    libm::round(ticks as f64 / TICKS_PER_DEG as f64 * core::f64::consts::PI / 180.0 * EARTH_R_MM)
        as i64
}

/// The 16 moves (dr, dc): 4 orthogonal, 4 diagonal, 8 knight.
pub const MOVES: [(i32, i32); 16] = [
    (0, 1),
    (1, 0),
    (0, -1),
    (-1, 0),
    (1, 1),
    (1, -1),
    (-1, 1),
    (-1, -1),
    (1, 2),
    (2, 1),
    (2, -1),
    (1, -2),
    (-1, -2),
    (-2, -1),
    (-2, 1),
    (-1, 2),
];

/// The two cells a knight move passes between, relative to its start.
fn crossed(dr: i32, dc: i32) -> Option<[(i32, i32); 2]> {
    if dr.abs() == 1 && dc.abs() == 2 {
        Some([(0, dc / 2), (dr, dc / 2)])
    } else if dr.abs() == 2 && dc.abs() == 1 {
        Some([(dr / 2, 0), (dr / 2, dc)])
    } else {
        None
    }
}

/// floor(sqrt(n)), exact: the f64 estimate is corrected in u128.
fn isqrt(n: u64) -> u64 {
    let mut x = (n as f64).sqrt() as u128;
    let n = n as u128;
    while x * x > n {
        x -= 1;
    }
    while (x + 1) * (x + 1) <= n {
        x += 1;
    }
    x as u64
}

/// A surface's planes and the terrain's heights on the dense rectangle of posts its blocks span
/// (rows from the south). Posts of blocks the surface does not hold are impassable.
pub struct Grid {
    pub spacing: i64,
    /// First block row and column of the rectangle.
    pub bi0: i64,
    pub bj0: i64,
    /// Rows and columns of posts.
    pub h: usize,
    pub w: usize,
    pub dm: Vec<i32>,
    /// Per profile.
    pub t: Vec<Vec<u16>>,
    pub risk: Vec<Vec<u8>>,
    pub cert: Vec<u8>,
    /// [`SRC_BASE`], [`SRC_TEAM`], else 1 + index into the surface header's applied list.
    pub src: Vec<u8>,
    pub rules: Vec<Rules>,
}

impl Grid {
    pub fn new(ter: &Terrain, s: &Surface) -> Result<Grid> {
        if s.h.terrain != ter.hash() || s.h.spacing as i64 != ter.spacing().ticks() {
            return Err(Error::invalid("the surface was not built on this terrain"));
        }
        let ids: Vec<BlockId> = s.blocks.values().map(|b| b.0).collect();
        if ids.is_empty() {
            return Err(Error::invalid("the surface has no blocks"));
        }
        let (bi0, bi1) = (
            ids.iter().map(|b| b.bi).min().unwrap() as i64,
            ids.iter().map(|b| b.bi).max().unwrap() as i64,
        );
        let (bj0, bj1) = (
            ids.iter().map(|b| b.bj).min().unwrap() as i64,
            ids.iter().map(|b| b.bj).max().unwrap() as i64,
        );
        let (h, w) = (
            (bi1 - bi0 + 1) as usize * BLOCK,
            (bj1 - bj0 + 1) as usize * BLOCK,
        );
        let n = h * w;
        let rules = s.rules();
        let np = rules.len();
        let mut g = Grid {
            spacing: ter.spacing().ticks(),
            bi0,
            bj0,
            h,
            w,
            dm: vec![0; n],
            t: vec![vec![0; n]; np],
            risk: vec![vec![255; n]; np],
            cert: vec![0; n],
            src: vec![SRC_BASE; n],
            rules,
        };
        let pos: BTreeMap<[u8; 32], u8> =
            s.h.applied
                .iter()
                .enumerate()
                .map(|(k, a)| (a.hash, k as u8 + 1))
                .collect();
        for (id, sb) in s.blocks.values() {
            let k = ter
                .block_index(*id)
                .ok_or_else(|| Error::invalid("surface block outside the terrain"))?;
            let dm = ter.dm(k);
            let (r0, c0) = (
                (id.bi as i64 - bi0) as usize * BLOCK,
                (id.bj as i64 - bj0) as usize * BLOCK,
            );
            for (q, &h) in dm.iter().enumerate() {
                let i = (r0 + q / BLOCK) * w + c0 + q % BLOCK;
                g.dm[i] = h;
                g.cert[i] = sb.cert[q];
                g.src[i] = match sb.src[q] {
                    SRC_BASE => SRC_BASE,
                    SRC_TEAM => SRC_TEAM,
                    k => *pos.get(&sb.table[k as usize - 1]).ok_or_else(|| {
                        Error::invalid("block table names a layer the header does not")
                    })?,
                };
                for p in 0..np {
                    g.t[p][i] = sb.prof[p].t[q];
                    g.risk[p][i] = sb.prof[p].risk[q];
                }
            }
        }
        Ok(g)
    }

    /// The cell of global post (i, j), if it is in the rectangle.
    pub fn cell_of(&self, i: i64, j: i64) -> Option<usize> {
        let (r, c) = (i - self.bi0 * BLOCK as i64, j - self.bj0 * BLOCK as i64);
        (r >= 0 && c >= 0 && (r as usize) < self.h && (c as usize) < self.w)
            .then(|| r as usize * self.w + c as usize)
    }

    /// The global post (i, j) of a cell.
    pub fn post_of(&self, cell: usize) -> (i64, i64) {
        (
            (cell / self.w) as i64 + self.bi0 * BLOCK as i64,
            (cell % self.w) as i64 + self.bj0 * BLOCK as i64,
        )
    }

    /// The index of a profile by name.
    pub fn profile(&self, name: &str) -> Option<usize> {
        self.rules.iter().position(|r| r.p.name == name)
    }
}

/// The move graph of one profile on a grid.
pub struct Graph<'a> {
    pub g: &'a Grid,
    pub rules: &'a Rules,
    pub t: &'a [u16],
    /// Move lengths (mm) per half-row k = 2r + dr (+2) (the move's mid-latitude) and move.
    pub len: Vec<[u32; 16]>,
    dy_mm: i64,
    dx_min_mm: i64,
    t_min: u64,
    rel_max: u64,
}

impl<'a> Graph<'a> {
    pub fn new(g: &'a Grid, prof: usize) -> Graph<'a> {
        let s = g.spacing;
        let dy = dy_mm(s);
        let lat0 = g.bi0 * BLOCK as i64 * s; // ticks of row 0
        let mut len = Vec::with_capacity(2 * g.h + 4);
        let mut dx_min = i64::MAX;
        for k in -2..(2 * g.h as i64 + 2) {
            let lat2 = lat0 * 2 + k * s; // half-ticks
            let dx = dx_mm(lat2 / 2, s);
            dx_min = dx_min.min(dx);
            let mut l = [0u32; 16];
            for (m, &(dr, dc)) in MOVES.iter().enumerate() {
                l[m] = isqrt(((dr as i64 * dy).pow(2) + (dc as i64 * dx).pow(2)) as u64) as u32;
            }
            len.push(l);
        }
        let t = &g.t[prof];
        let t_min = t.iter().filter(|&&x| x > 0).min().copied().unwrap_or(1) as u64;
        let rules = &g.rules[prof];
        let rel_max = *rules.p.grade_rel.iter().max().expect("201 grades") as u64;
        Graph {
            g,
            rules,
            t,
            len,
            dy_mm: dy,
            dx_min_mm: dx_min,
            t_min,
            rel_max,
        }
    }

    /// Length (mm) of move `m` from row `r`.
    pub fn move_len(&self, r: usize, m: usize) -> u64 {
        self.len[(2 * r as i64 + MOVES[m].0 as i64 + 2) as usize][m] as u64
    }

    /// Cost (µs) of move `m` from cell (r, c) and the cell it reaches, or None.
    #[inline]
    pub fn cost(&self, r: usize, c: usize, m: usize) -> Option<(usize, u64)> {
        let (dr, dc) = MOVES[m];
        let (nr, nc) = (r as i64 + dr as i64, c as i64 + dc as i64);
        if nr < 0 || nc < 0 || nr >= self.g.h as i64 || nc >= self.g.w as i64 {
            return None;
        }
        let w = self.g.w;
        let (a, z) = (r * w + c, nr as usize * w + nc as usize);
        let (ta, tz) = (self.t[a] as u64, self.t[z] as u64);
        if ta == 0 || tz == 0 {
            return None;
        }
        let (mut sum, mut n) = (ta + tz, 2u64);
        if let Some(x) = crossed(dr, dc) {
            for (er, ec) in x {
                let tx = self.t
                    [(r as i64 + er as i64) as usize * w + (c as i64 + ec as i64) as usize]
                    as u64;
                if tx == 0 {
                    return None;
                }
                sum += tx;
                n += 1;
            }
        }
        let l = self.move_len(r, m);
        let (ha, hz) = (self.g.dm[a], self.g.dm[z]);
        let g = if ha == DEM_NONE || hz == DEM_NONE {
            0
        } else {
            div_round_i64((hz as i64 - ha as i64) * 100 * 100, l as i64).clamp(-100, 100)
        };
        let rel = self.rules.p.grade_rel[(g + 100) as usize] as u64;
        let (num, den) = (l * sum * self.rules.p.unit_us, n * rel); // < 2^50 for valid profiles
        let cost = (2 * num + den) / (2 * den);
        Some((z, cost.max(1)))
    }

    /// An admissible, consistent lower bound (µs) from cell i to cell j.
    #[inline]
    pub fn h(&self, i: usize, j: usize) -> u64 {
        let w = self.g.w as i64;
        let (dr, dc) = (
            (i as i64 / w - j as i64 / w).abs(),
            (i as i64 % w - j as i64 % w).abs(),
        );
        let l = isqrt(((dr * self.dy_mm).pow(2) + (dc * self.dx_min_mm).pow(2)) as u64);
        // 0.999: room for the rounding of lengths and costs
        (l * self.t_min * self.rules.p.unit_us / self.rel_max) * 999 / 1000
    }
}

/// Reusable search state.
pub struct Search {
    dist: Vec<u64>,
    stamp: Vec<u32>,
    epoch: u32,
    heap: BinaryHeap<Reverse<(u64, u32)>>,
    pub settled: u64,
    pub pushes: u64,
}

impl Search {
    pub fn new(n: usize) -> Self {
        Search {
            dist: vec![u64::MAX; n],
            stamp: vec![0; n],
            epoch: 0,
            heap: BinaryHeap::new(),
            settled: 0,
            pushes: 0,
        }
    }

    /// Cost (µs) from the last run's source to cell i; `u64::MAX` if not reached.
    #[inline]
    pub fn d(&self, i: usize) -> u64 {
        if self.stamp[i] == self.epoch {
            self.dist[i]
        } else {
            u64::MAX
        }
    }

    /// Dijkstra (`astar = false`) or A* to `target`; with no target, the full field. Returns the
    /// target's cost (or `Some(0)` for a full field, `None` if the target is unreachable).
    pub fn run(
        &mut self,
        g: &Graph,
        src: usize,
        target: Option<usize>,
        astar: bool,
    ) -> Option<u64> {
        if self.epoch == u32::MAX {
            self.stamp.fill(0);
            self.epoch = 0;
        }
        self.epoch += 1;
        self.heap.clear();
        self.settled = 0;
        self.pushes = 0;
        if g.t[src] == 0 {
            return None;
        }
        let w = g.g.w;
        let astar = astar && target.is_some();
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
}

/// The canonical path to `dst` after a run from `src` settled it, as (from cell, move) steps:
/// from dst back, the predecessor u with d(u) + c(u, v) = d(v), smallest cell index first.
/// Valid after a Dijkstra run settled `dst` (a full field or one stopped at `dst`): every
/// predecessor on a least-cost path then has a smaller, final cost. Not valid after A*.
pub fn path(g: &Graph, se: &Search, src: usize, dst: usize) -> Option<Vec<(usize, usize)>> {
    if se.d(dst) == u64::MAX {
        return None;
    }
    let w = g.g.w as i64;
    let mut out = Vec::new();
    let mut v = dst;
    while v != src {
        let (vr, vc) = ((v as i64) / w, (v as i64) % w);
        let mut best: Option<(usize, usize)> = None;
        for (m, &(dr, dc)) in MOVES.iter().enumerate() {
            let (ur, uc) = (vr - dr as i64, vc - dc as i64);
            if ur < 0 || uc < 0 || ur >= g.g.h as i64 || uc >= w {
                continue;
            }
            let u = (ur * w + uc) as usize;
            let du = se.d(u);
            if du == u64::MAX {
                continue;
            }
            if let Some((_, c)) = g.cost(ur as usize, uc as usize, m)
                && du + c == se.d(v)
                && best.is_none_or(|b| u < b.0)
            {
                best = Some((u, m));
            }
        }
        let (u, m) = best?;
        out.push((u, m));
        v = u;
    }
    out.reverse();
    Some(out)
}

/// How far to trust a route (`docs/design/incident-surface.md` section 7).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RouteConf {
    pub cost_us: u64,
    pub len_mm: u64,
    /// Length-weighted mean of the fused certainty (1/255).
    pub mean_cert: u8,
    pub min_cert: u8,
    /// Share of the length on cells a layer set, and on cells the team set, per mille.
    pub layer_permille: u32,
    pub team_permille: u32,
    /// Highest risk of a cell on the route (1/255), and the share of length with any risk.
    pub max_risk: u8,
    pub risky_permille: u32,
    /// Indexes (1-based) into the header's applied list of the layers the route rests on.
    pub layers: Vec<u8>,
    /// The route touches the outermost ring of the grid: a larger AOI might give a faster route.
    pub at_edge: bool,
}

/// Route confidence of a path: each move's length is shared equally among the cells it crosses
/// (2, or 4 for a knight move), as weights 4 L / n.
pub fn route_conf(g: &Graph, prof: usize, steps: &[(usize, usize)], cost_us: u64) -> RouteConf {
    let grid = g.g;
    let w = grid.w;
    let (mut wsum, mut csum, mut lay, mut team, mut risky, mut len) =
        (0u64, 0u64, 0u64, 0u64, 0u64, 0u64);
    let (mut minc, mut maxr) = (255u8, 0u8);
    let mut layers = BTreeSet::new();
    let mut at_edge = false;
    for &(u, m) in steps {
        let (r, c) = (u / w, u % w);
        let (dr, dc) = MOVES[m];
        let l = g.move_len(r, m);
        len += l;
        let (r, c, dr, dc) = (r as i64, c as i64, dr as i64, dc as i64);
        let mut cells = vec![(r, c), (r + dr, c + dc)];
        if dr.abs() + dc.abs() == 3 {
            if dr.abs() == 1 {
                cells.extend([(r, c + dc / 2), (r + dr, c + dc / 2)]);
            } else {
                cells.extend([(r + dr / 2, c), (r + dr / 2, c + dc)]);
            }
        }
        let wt = 4 * l / cells.len() as u64;
        for (cr, cc) in cells {
            if cr == 0 || cc == 0 || cr == grid.h as i64 - 1 || cc == w as i64 - 1 {
                at_edge = true;
            }
            let i = cr as usize * w + cc as usize;
            wsum += wt;
            csum += wt * grid.cert[i] as u64;
            minc = minc.min(grid.cert[i]);
            match grid.src[i] {
                SRC_BASE => {}
                SRC_TEAM => team += wt,
                k => {
                    lay += wt;
                    layers.insert(k);
                }
            }
            let rk = grid.risk[prof][i];
            maxr = maxr.max(rk);
            if rk > 0 {
                risky += wt;
            }
        }
    }
    let pm = |x: u64| {
        if wsum == 0 {
            0
        } else {
            div_round(1000 * x as i128, wsum as i128) as u32
        }
    };
    RouteConf {
        cost_us,
        len_mm: len,
        mean_cert: if wsum == 0 {
            0
        } else {
            div_round(csum as i128, wsum as i128) as u8
        },
        min_cert: if wsum == 0 { 0 } else { minc },
        layer_permille: pm(lay),
        team_permille: pm(team),
        max_risk: maxr,
        risky_permille: pm(risky),
        layers: layers.into_iter().collect(),
        at_edge,
    }
}

/// A route between two cells: its steps and confidence.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Route {
    pub steps: Vec<(usize, usize)>,
    pub conf: RouteConf,
}

/// The least-cost route between two cells for profile `prof` (Dijkstra to the target, then the
/// canonical path), or None if unreachable on the grid.
pub fn route(grid: &Grid, prof: usize, src: usize, dst: usize) -> Option<Route> {
    let g = Graph::new(grid, prof);
    let mut se = Search::new(grid.h * grid.w);
    let cost = se.run(&g, src, Some(dst), false)?;
    let steps = path(&g, &se, src, dst)?;
    let conf = route_conf(&g, prof, &steps, cost);
    Some(Route { steps, conf })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn knight_needs_crossed_cells() {
        assert_eq!(crossed(1, 2), Some([(0, 1), (1, 1)]));
        assert_eq!(crossed(-2, 1), Some([(-1, 0), (-1, 1)]));
        assert_eq!(crossed(1, 1), None);
    }

    #[test]
    fn ground_lengths() {
        // 1" of latitude is about 30.9 m; of longitude at 49.25 N about 20.2 m
        assert_eq!(dy_mm(120), 30_888);
        let dx = dx_mm(49 * TICKS_PER_DEG + TICKS_PER_DEG / 4, 120);
        assert!((20_100..20_200).contains(&dx), "{dx}");
        assert_eq!(isqrt(0), 0);
        assert_eq!(isqrt(99), 9);
        assert_eq!(isqrt(u32::MAX as u64 * u32::MAX as u64), u32::MAX as u64);
    }
}

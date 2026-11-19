//! CCH lower-bound potentials ("CH-potentials", Strasser and Zeitz) for the
//! edge-based A* of `package::search`.
//!
//! Built from an open package on first use
//! ([`RoutingPackage::prepare_potentials`](super::RoutingPackage::prepare_potentials)):
//!
//! 1. **Order.** The package stores the node order (`Section::CchRank`, by
//!    nested dissection of the node graph; see `wayfarer_package::cch_order`,
//!    which the compiler runs).
//! 2. **Topology.** For each node by rank, its lowest upward neighbour `u`
//!    inherits its other upward neighbours; `u` is its elimination-tree
//!    parent. Arcs are stored once (lower → higher rank) with a weight per
//!    direction.
//! 3. **Customization** per stored cost column: arcs start at the least column
//!    cost of the edges they stand for (turn costs left out, which keeps the
//!    metric a lower bound: turn costs are never negative), then every lower
//!    triangle is relaxed, node by node in elimination-tree level order, in
//!    parallel within a level ("pull": a node writes only its own arcs).
//!    Weights are f32 rounded *down* from f64 sums, so every stored weight
//!    respects its lower triangles exactly and the up-down distances are exact
//!    shortest distances of a metric no higher than the column.
//! 4. **Potential.** From the targets, an elimination-tree walk over their
//!    ancestors gives `B(y)`, the least down-path cost from `y` to a target;
//!    then `pot(v) = min(B(v), min over upward arcs (v, y) of w(v, y) +
//!    pot(y))`, memoised along `v`'s elimination-tree path (its upward search
//!    space), top down. `pot` is the exact node-level distance to the targets
//!    under the column, hence admissible and consistent for the edge-based
//!    search, and the average-potential stop stays exact.

use std::cell::RefCell;

use std::sync::{Arc, Mutex, OnceLock};
use std::time::Instant;

use rayon::prelude::*;
use rustc_hash::FxHashMap;

use wayfarer_package::cch_order::{NodeGraph, is_permutation};

use super::Sections;
use super::columns::MetricKey;

const NO_NODE: u32 = u32::MAX;
/// Marks an edge's arc as lower → higher rank in [`CchTopology::edge_arc`].
const UP_BIT: u32 = 1 << 31;
/// Safety factor on potentials against f64 rounding in path sums (a
/// consistent potential scaled by a factor below 1 stays consistent).
pub(crate) const POTENTIAL_MARGIN: f64 = 1.0 - 1e-9;

/// What [`RoutingPackage::prepare_potentials`](super::RoutingPackage::prepare_potentials)
/// built, and how long it took.
#[derive(Debug, Clone, Default, PartialEq)]
#[non_exhaustive]
pub struct PotentialStats {
    pub node_count: usize,
    pub arc_count: usize,
    /// Elimination-tree height: the largest upward search space.
    pub elimination_tree_height: usize,
    /// Stored cost columns customized.
    pub column_count: usize,
    pub order_ms: f64,
    pub topology_ms: f64,
    /// Customization time summed over the columns.
    pub customize_ms: f64,
    /// Topology and metrics in memory, without the per-search scratch
    /// (24 bytes per node and direction, allocated by the first search that
    /// uses the potentials on a pooled search state).
    pub heap_bytes: usize,
}

/// The metric-independent part: order, arcs, elimination tree.
pub(crate) struct CchTopology {
    /// Node → rank.
    rank: Vec<u32>,
    /// Rank → node.
    node: Vec<u32>,
    /// By rank: upward arcs `first_up[x]..first_up[x + 1]`, heads by rank in
    /// ascending order.
    first_up: Vec<u32>,
    up_head: Vec<u32>,
    /// By rank: elimination-tree parent (the lowest upward neighbour).
    parent: Vec<u32>,
    /// By rank: downward arcs as (lower node, its arc to this one).
    first_down: Vec<u32>,
    down_tail: Vec<u32>,
    down_arc: Vec<u32>,
    /// Ranks in elimination-tree level order (leaves first), and where each
    /// level starts.
    level_order: Vec<u32>,
    level_start: Vec<u32>,
    /// Per package edge: its arc, with [`UP_BIT`] when it runs lower → higher
    /// rank; `NO_NODE` for a loop.
    edge_arc: Vec<u32>,
}

/// One customized metric: per arc, the weight lower → higher rank (`up`) and
/// higher → lower (`down`).
pub(crate) struct CchMetric {
    up: Vec<f32>,
    down: Vec<f32>,
}

/// The topology and the metric of every stored column.
pub(crate) struct CchPotentials {
    pub topology: CchTopology,
    /// By stored column.
    pub metrics: Vec<CchMetric>,
    pub stats: PotentialStats,
    /// Metrics of profiles' own costs (`columns::metric_costs`), customized
    /// on first use, least recently used first.
    own: Mutex<Vec<(MetricKey, Arc<OwnMetric>)>>,
}

/// Own metrics kept customized (see [`CchPotentials::own_metric`]).
const OWN_METRIC_CAPACITY: usize = 4;

/// A metric customized from per-edge costs other than a stored column's,
/// with those costs.
pub(crate) struct OwnMetric {
    pub costs: Vec<f32>,
    pub metric: CchMetric,
}

impl CchPotentials {
    /// The metric of `key`, customized from `costs()` on a miss.
    pub(crate) fn own_metric(
        &self,
        key: MetricKey,
        costs: impl FnOnce() -> Vec<f32>,
    ) -> Arc<OwnMetric> {
        if let Ok(mut own) = self.own.lock()
            && let Some(at) = own.iter().position(|(k, _)| *k == key)
        {
            let entry = own.remove(at);
            let metric = entry.1.clone();
            own.push(entry);
            return metric;
        }
        let costs = costs();
        let metric = Arc::new(OwnMetric {
            metric: self.topology.customize(&costs),
            costs,
        });
        if let Ok(mut own) = self.own.lock() {
            if let Some(at) = own.iter().position(|(k, _)| *k == key) {
                return own[at].1.clone();
            }
            if own.len() >= OWN_METRIC_CAPACITY {
                own.remove(0);
            }
            own.push((key, metric.clone()));
        }
        metric
    }
}

/// A package's potentials, built on first use.
#[derive(Default)]
pub(crate) struct CchCache {
    ready: OnceLock<Option<Arc<CchPotentials>>>,
}

impl CchCache {
    /// Builds the potentials on the first call; later calls return them.
    pub(crate) fn prepare(&self, g: &Sections<'_>) -> Option<&Arc<CchPotentials>> {
        self.ready
            .get_or_init(|| CchPotentials::build(g).map(Arc::new))
            .as_ref()
    }
}

impl CchPotentials {
    fn build(g: &Sections<'_>) -> Option<Self> {
        let (topology, mut stats) = CchTopology::build(g)?;
        let columns = g.edge_cost.len().checked_div(g.edge_count).unwrap_or(0);
        let started = Instant::now();
        let metrics = (0..columns)
            .map(|column| {
                let range = column * g.edge_count..(column + 1) * g.edge_count;
                topology.customize(&g.edge_cost[range])
            })
            .collect::<Vec<_>>();
        stats.customize_ms = started.elapsed().as_secs_f64() * 1e3;
        stats.column_count = columns;
        stats.heap_bytes = topology.heap_bytes()
            + metrics
                .iter()
                .map(|m| 4 * (m.up.len() + m.down.len()))
                .sum::<usize>();
        Some(Self {
            topology,
            metrics,
            stats,
            own: Mutex::new(Vec::new()),
        })
    }
}

// -------------------------------------------------------------------------
// Topology
// -------------------------------------------------------------------------

impl CchTopology {
    /// The topology of `g`'s node graph in its stored order; `None` without
    /// a stored order, or when the arcs outgrow the 31-bit arc ids (far
    /// beyond a regional package).
    pub(crate) fn build(g: &Sections<'_>) -> Option<(Self, PotentialStats)> {
        let mut stats = PotentialStats {
            node_count: g.node_count,
            ..Default::default()
        };
        let started = Instant::now();
        let graph = NodeGraph::new(g.first_out, g.edge_target);
        if g.cch_rank.len() != g.node_count || !is_permutation(g.cch_rank) {
            return None;
        }
        let mut order = vec![0u32; g.node_count];
        for (node, r) in g.cch_rank.iter().enumerate() {
            order[*r as usize] = node as u32;
        }
        stats.order_ms = started.elapsed().as_secs_f64() * 1e3;

        let started = Instant::now();
        let n = g.node_count;
        let mut rank = vec![0u32; n];
        for (r, node) in order.iter().enumerate() {
            rank[*node as usize] = r as u32;
        }
        // Upward neighbour sets with fill.
        let mut up = (0..n).map(|_| Vec::new()).collect::<Vec<Vec<u32>>>();
        for node in 0..n as u32 {
            let rn = rank[node as usize];
            for w in graph.neighbours(node) {
                let rw = rank[*w as usize];
                if rw > rn {
                    up[rn as usize].push(rw);
                }
            }
        }
        drop(graph);
        let mut parent = vec![NO_NODE; n];
        for x in 0..n {
            let mut list = std::mem::take(&mut up[x]);
            list.sort_unstable();
            list.dedup();
            if let Some(&p) = list.first() {
                parent[x] = p;
                let rest = list[1..].to_vec();
                up[p as usize].extend(rest);
            }
            list.shrink_to_fit();
            up[x] = list;
        }
        let mut first_up = Vec::with_capacity(n + 1);
        first_up.push(0u32);
        let mut total = 0u64;
        for list in &up {
            total += list.len() as u64;
            first_up.push(total as u32);
        }
        if total >= u64::from(UP_BIT) {
            return None;
        }
        let mut up_head = Vec::with_capacity(total as usize);
        for list in &mut up {
            up_head.extend_from_slice(list);
            *list = Vec::new();
        }
        drop(up);
        let arcs = up_head.len();
        // Downward lists.
        let mut down_count = vec![0u32; n + 1];
        for head in &up_head {
            down_count[*head as usize] += 1;
        }
        let mut first_down = vec![0u32; n + 1];
        for y in 0..n {
            first_down[y + 1] = first_down[y] + down_count[y];
        }
        let mut fill = first_down.clone();
        let mut down_tail = vec![0u32; arcs];
        let mut down_arc = vec![0u32; arcs];
        for x in 0..n {
            for a in first_up[x]..first_up[x + 1] {
                let y = up_head[a as usize] as usize;
                down_tail[fill[y] as usize] = x as u32;
                down_arc[fill[y] as usize] = a;
                fill[y] += 1;
            }
        }
        // Elimination-tree levels and heights.
        let mut level = vec![0u32; n];
        for x in 0..n {
            let p = parent[x];
            if p != NO_NODE {
                level[p as usize] = level[p as usize].max(level[x] + 1);
            }
        }
        let level_count = level.iter().copied().max().map_or(0, |l| l as usize + 1);
        let mut level_size = vec![0u32; level_count + 1];
        for l in &level {
            level_size[*l as usize] += 1;
        }
        let mut level_start = vec![0u32; level_count + 1];
        for l in 0..level_count {
            level_start[l + 1] = level_start[l] + level_size[l];
        }
        let mut fill = level_start.clone();
        let mut level_order = vec![0u32; n];
        for (x, l) in level.iter().enumerate() {
            let l = *l as usize;
            level_order[fill[l] as usize] = x as u32;
            fill[l] += 1;
        }
        // Package edges → arcs.
        let mut edge_arc = vec![NO_NODE; g.edge_count];
        for u in 0..n {
            for e in g.first_out[u]..g.first_out[u + 1] {
                let v = g.edge_target[e as usize] as usize;
                if v == u {
                    continue;
                }
                let (ru, rv) = (rank[u], rank[v]);
                let (lo, hi) = if ru < rv { (ru, rv) } else { (rv, ru) };
                let list =
                    &up_head[first_up[lo as usize] as usize..first_up[lo as usize + 1] as usize];
                let at = list.binary_search(&hi).expect("every edge has an arc");
                let arc = first_up[lo as usize] + at as u32;
                edge_arc[e as usize] = if ru < rv { arc | UP_BIT } else { arc };
            }
        }
        // Statistics: elimination-tree depth = upward search space size.
        let mut depth = vec![0u32; n];
        for x in (0..n).rev() {
            let p = parent[x];
            depth[x] = if p == NO_NODE {
                1
            } else {
                depth[p as usize] + 1
            };
        }
        stats.elimination_tree_height = depth.iter().copied().max().unwrap_or(0) as usize;
        stats.arc_count = arcs;
        stats.topology_ms = started.elapsed().as_secs_f64() * 1e3;
        let mut node = vec![0u32; n];
        for (v, r) in rank.iter().enumerate() {
            node[*r as usize] = v as u32;
        }
        let topology = Self {
            rank,
            node,
            first_up,
            up_head,
            parent,
            first_down,
            down_tail,
            down_arc,
            level_order,
            level_start,
            edge_arc,
        };
        Some((topology, stats))
    }

    fn heap_bytes(&self) -> usize {
        4 * (self.rank.len()
            + self.node.len()
            + self.first_up.len()
            + self.up_head.len()
            + self.parent.len()
            + self.first_down.len()
            + self.down_tail.len()
            + self.down_arc.len()
            + self.level_order.len()
            + self.level_start.len()
            + self.edge_arc.len())
    }

    pub(crate) fn arc_count(&self) -> usize {
        self.up_head.len()
    }

    // ---------------------------------------------------------------------
    // Customization
    // ---------------------------------------------------------------------

    /// Input weights: per arc the least cost of its edges each way.
    fn input_weights(&self, cost: &[f32]) -> (Vec<f32>, Vec<f32>) {
        let arcs = self.arc_count();
        let mut up = vec![f32::INFINITY; arcs];
        let mut down = vec![f32::INFINITY; arcs];
        for (edge, code) in self.edge_arc.iter().enumerate() {
            if *code == NO_NODE {
                continue;
            }
            let c = cost[edge];
            if !c.is_finite() {
                continue;
            }
            let a = (*code & !UP_BIT) as usize;
            if *code & UP_BIT != 0 {
                up[a] = up[a].min(c);
            } else {
                down[a] = down[a].min(c);
            }
        }
        (up, down)
    }

    /// Customizes `cost` (a column: one cost per package edge, infinite =
    /// unusable).
    pub(crate) fn customize(&self, cost: &[f32]) -> CchMetric {
        let (mut up, mut down) = self.input_weights(cost);
        let up_ptr = SyncPtr(up.as_mut_ptr());
        let down_ptr = SyncPtr(down.as_mut_ptr());
        for level in 0..self.level_start.len() - 1 {
            let ranks = &self.level_order
                [self.level_start[level] as usize..self.level_start[level + 1] as usize];
            let work = |y: &u32| {
                // SAFETY: a node writes only its own arcs, and reads arcs of
                // lower levels (finished) or its own; nodes of one level are
                // distinct.
                unsafe { self.relax_node(*y as usize, up_ptr, down_ptr) };
            };
            if ranks.len() > 64 {
                ranks.par_iter().for_each(work);
            } else {
                ranks.iter().for_each(work);
            }
        }
        CchMetric { up, down }
    }

    /// Relaxes every lower triangle of node `y`'s arcs (pull).
    ///
    /// # Safety
    /// Only `y` may write `y`'s arcs while this runs, and the arcs of `y`'s
    /// downward neighbours must be final.
    unsafe fn relax_node(&self, y: usize, up_ptr: SyncPtr, down_ptr: SyncPtr) {
        let (ys, ye) = (self.first_up[y] as usize, self.first_up[y + 1] as usize);
        if ys == ye {
            return;
        }
        let heads = &self.up_head[ys..ye];
        let up = up_ptr.0;
        let down = down_ptr.0;
        for k in self.first_down[y] as usize..self.first_down[y + 1] as usize {
            let x = self.down_tail[k] as usize;
            let a_xy = self.down_arc[k] as usize;
            // SAFETY: arcs of x are final (lower level).
            let (w_xy, w_yx) = unsafe { (*up.add(a_xy), *down.add(a_xy)) };
            if !w_xy.is_finite() && !w_yx.is_finite() {
                continue;
            }
            let x_end = self.first_up[x + 1] as usize;
            let mut j = 0usize;
            for a_xz in a_xy + 1..x_end {
                let z = self.up_head[a_xz];
                while heads[j] < z {
                    j += 1;
                }
                debug_assert_eq!(heads[j], z);
                let a_yz = ys + j;
                // SAFETY: as above; a_yz is y's own arc.
                unsafe {
                    let (w_xz, w_zx) = (*up.add(a_xz), *down.add(a_xz));
                    // y -> x -> z
                    let via_up = f64::from(w_yx) + f64::from(w_xz);
                    if via_up < f64::from(*up.add(a_yz)) {
                        *up.add(a_yz) = round_down(via_up);
                    }
                    // z -> x -> y
                    let via_down = f64::from(w_zx) + f64::from(w_xy);
                    if via_down < f64::from(*down.add(a_yz)) {
                        *down.add(a_yz) = round_down(via_down);
                    }
                }
            }
        }
    }
}

#[derive(Clone, Copy)]
struct SyncPtr(*mut f32);
// SAFETY: used only under the disjoint-write discipline of `customize`.
unsafe impl Send for SyncPtr {}
unsafe impl Sync for SyncPtr {}

/// The largest f32 at or below `value`.
#[inline]
fn round_down(value: f64) -> f32 {
    let rounded = value as f32;
    if f64::from(rounded) > value {
        rounded.next_down()
    } else {
        rounded
    }
}

// -------------------------------------------------------------------------
// Potentials
// -------------------------------------------------------------------------

/// Per-direction query scratch, sized to the node count once and reset by
/// epochs.
#[derive(Default)]
pub(crate) struct PotentialScratch {
    /// By rank: the node's entry in `entries`, valid only when that entry
    /// names the rank back (a sparse set: never cleared, 4 bytes per node).
    slot: Vec<u32>,
    /// The nodes this query touched: rank, `B` (infinite unless an ancestor
    /// of a target, see the module docs) and the memoised potential (NaN
    /// until computed).
    entries: Vec<ScratchEntry>,
    ancestors: Vec<u32>,
    path: Vec<u32>,
}

#[derive(Clone, Copy)]
struct ScratchEntry {
    rank: u32,
    b: f64,
    pot: f64,
}

/// Entries a scratch keeps its capacity for between queries.
const SCRATCH_KEEP: usize = 1 << 16;

impl PotentialScratch {
    fn begin(&mut self, n: usize) {
        if self.slot.len() != n {
            self.slot = vec![u32::MAX; n];
        }
        if self.entries.capacity() > SCRATCH_KEEP {
            self.entries = Vec::new();
        } else {
            self.entries.clear();
        }
    }

    #[inline]
    fn find(&self, rank: u32) -> Option<usize> {
        let index = self.slot[rank as usize] as usize;
        (index < self.entries.len() && self.entries[index].rank == rank).then_some(index)
    }

    /// The entry of `rank`, added (no `B`, no potential) if missing.
    #[inline]
    fn entry(&mut self, rank: u32) -> usize {
        if let Some(index) = self.find(rank) {
            return index;
        }
        let index = self.entries.len();
        self.slot[rank as usize] = index as u32;
        self.entries.push(ScratchEntry {
            rank,
            b: f64::INFINITY,
            pot: f64::NAN,
        });
        index
    }
}

/// The CCH potential towards a set of targets (`reversed`: from a set of
/// sources), times `factor`.
pub(crate) struct CchPotential<'a> {
    topology: &'a CchTopology,
    up: &'a [f32],
    down: &'a [f32],
    reversed: bool,
    factor: f64,
    scratch: &'a RefCell<PotentialScratch>,
}

impl<'a> CchPotential<'a> {
    /// `targets` are (node, initial distance) pairs; `factor` scales the
    /// column metric to the query's (≤ 1).
    pub(crate) fn new(
        topology: &'a CchTopology,
        metric: &'a CchMetric,
        targets: &[(u32, f64)],
        reversed: bool,
        factor: f64,
        scratch: &'a RefCell<PotentialScratch>,
    ) -> Self {
        let potential = Self {
            topology,
            up: &metric.up,
            down: &metric.down,
            reversed,
            factor: factor * POTENTIAL_MARGIN,
            scratch,
        };
        potential.init(targets);
        potential
    }

    fn init(&self, targets: &[(u32, f64)]) {
        let t = self.topology;
        let mut s = self.scratch.borrow_mut();
        let s = &mut *s;
        s.begin(t.rank.len());
        s.ancestors.clear();
        for (node, initial) in targets {
            let r = t.rank[*node as usize];
            if let Some(index) = s.find(r) {
                let b = &mut s.entries[index].b;
                *b = b.min(*initial);
                continue;
            }
            let index = s.entry(r);
            s.entries[index].b = *initial;
            s.ancestors.push(r);
            let mut x = t.parent[r as usize];
            while x != NO_NODE && s.find(x).is_none() {
                s.entry(x);
                s.ancestors.push(x);
                x = t.parent[x as usize];
            }
        }
        s.ancestors.sort_unstable();
        // B(y) = min over downward arcs (y, x) of w(y -> x) + B(x) (forward;
        // reversed: w(x -> y)).
        let toward = if self.reversed { self.up } else { self.down };
        for i in 0..s.ancestors.len() {
            let x = s.ancestors[i];
            let bx = s.entries[s.slot[x as usize] as usize].b;
            if !bx.is_finite() {
                continue;
            }
            let arcs = t.first_up[x as usize] as usize..t.first_up[x as usize + 1] as usize;
            for (y, w) in t.up_head[arcs.clone()].iter().zip(&toward[arcs]) {
                let value = bx + f64::from(*w);
                let b = &mut s.entries[s.slot[*y as usize] as usize].b;
                if value < *b {
                    *b = value;
                }
            }
        }
    }

    /// The potential towards `targets` (at 0) and `seeds` (node, offset in
    /// request costs): `min(pot(v), min over seeds of d(v, u) + offset)`.
    pub(crate) fn with_offsets(
        topology: &'a CchTopology,
        metric: &'a CchMetric,
        targets: &[u32],
        seeds: &[(u32, f64)],
        reversed: bool,
        factor: f64,
        scratch: &'a RefCell<PotentialScratch>,
    ) -> Self {
        let scale = factor * POTENTIAL_MARGIN;
        let all = targets
            .iter()
            .map(|node| (*node, 0.0))
            .chain(
                seeds
                    .iter()
                    .filter(|(_, offset)| offset.is_finite())
                    .map(|(node, offset)| (*node, offset / scale * (1.0 - 1e-12))),
            )
            .collect::<Vec<_>>();
        Self::new(topology, metric, &all, reversed, factor, scratch)
    }

    /// The potential at package node `node` (infinite: no target reachable).
    #[inline]
    pub(crate) fn get(&self, node: u32) -> f64 {
        let t = self.topology;
        let r = t.rank[node as usize];
        let mut s = self.scratch.borrow_mut();
        let s = &mut *s;
        let memoised = s.find(r).map(|index| s.entries[index].pot);
        let value = match memoised {
            Some(value) if !value.is_nan() => value,
            _ => {
                s.path.clear();
                let mut x = r;
                while x != NO_NODE {
                    let index = s.entry(x);
                    if !s.entries[index].pot.is_nan() {
                        break;
                    }
                    s.path.push(index as u32);
                    x = t.parent[x as usize];
                }
                // pot(v) = min(B(v), min over upward arcs (v, y) of
                // w(v -> y) + pot(y)) (forward; reversed: w(y -> v)). Every
                // upward neighbour is an ancestor, memoised by now.
                let away = if self.reversed { self.down } else { self.up };
                let mut best = f64::INFINITY;
                for i in (0..s.path.len()).rev() {
                    let index = s.path[i] as usize;
                    let ScratchEntry { rank: x, b, .. } = s.entries[index];
                    best = b;
                    let arcs = t.first_up[x as usize] as usize..t.first_up[x as usize + 1] as usize;
                    for (y, w) in t.up_head[arcs.clone()].iter().zip(&away[arcs]) {
                        let value = f64::from(*w) + s.entries[s.slot[*y as usize] as usize].pot;
                        if value < best {
                            best = value;
                        }
                    }
                    s.entries[index].pot = best;
                }
                best
            }
        };
        if value < f64::INFINITY {
            value * self.factor
        } else {
            f64::INFINITY
        }
    }

    /// The unscaled distance (the exactness check).
    #[cfg(test)]
    pub(crate) fn distance(&self, node: u32) -> f64 {
        let value = self.get(node);
        if value < f64::INFINITY {
            value / self.factor
        } else {
            value
        }
    }
}

/// Rounds of gate offsets (see [`GatedPotential`]).
const GATE_ROUNDS: usize = 6;
/// Distinct gate nodes up to which gate offsets are solved exactly.
pub(crate) const EXACT_GATE_NODES: usize = 256;

/// An f64 ordered by `total_cmp`, for heaps.
#[derive(Clone, Copy, PartialEq)]
struct OrderedCost(f64);
impl Eq for OrderedCost {}
impl PartialOrd for OrderedCost {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for OrderedCost {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.total_cmp(&other.0)
    }
}

/// A gate of a [`GatedPotential`] on one side: the node a path reaches it at
/// (forward: the edge's source; backward: its target), the node it continues
/// from (the other end), and the request cost of the edge.
#[derive(Debug, Clone, Copy)]
pub(crate) struct PotentialGate {
    pub node: u32,
    pub end: u32,
    pub cost: f64,
}

/// CCH potentials for a request with discount gates (edges it makes cheaper
/// than the stored metric bounds, or usable where it is not; see
/// `search::gates`).
///
/// A path to the targets either uses no gate, and costs at least `pot(v)`,
/// or reaches a first gate `e` at its node `u` through edges the metric
/// bounds. So `pi(v) = min(pot(v), min over gates of d(v, u) + off(e))` with
/// `off(e) = c(e) + pi'(w)`, for any consistent lower bound `pi'` on the
/// request's costs from `w`, is a lower bound. That is one elimination-tree
/// query with the gates' nodes as further targets at their offsets.
///
/// With at most [`EXACT_GATE_NODES`] gate nodes the offsets are exact:
/// `off(e) = c(e) + min(pot(w), min over gates g of d(w, u_g) + off(g))`,
/// a Dijkstra over the gates with CCH distances between their ends and
/// nodes (see [`exact_offsets`]). Beyond that the offsets start from `pi' = floor * pot` (or 0) and are raised in rounds
/// with the previous round's `pi`. Within a round, gates that continue
/// straight into another gate (its node is this one's end: a discounted
/// corridor, both directions of an opened way) take `off(e) = min(c(e) +
/// pi'(w), c(e) + off(g))` by a Dijkstra over the gates, so chains settle at
/// once. Each round is a valid lower bound at least as high as the one
/// before, so every gate edge keeps `pi(u) <= c(e) + pi(w)` and the bound
/// stays consistent. With every gate an edge the
/// metric bounds, each edge costs at least `floor` times its metric cost,
/// and `floor * pot` is a consistent lower bound too: the larger of the two
/// is used.
pub(crate) struct GatedPotential<'a> {
    base: CchPotential<'a>,
    gated: Option<CchPotential<'a>>,
    floor: f64,
}

impl<'a> GatedPotential<'a> {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        topology: &'a CchTopology,
        metric: &'a CchMetric,
        targets: &[u32],
        reversed: bool,
        factor: f64,
        gates: &[PotentialGate],
        floor: f64,
        scratch: &'a RefCell<PotentialScratch>,
        gated_scratch: &'a RefCell<PotentialScratch>,
    ) -> Self {
        let seeds = targets.iter().map(|node| (*node, 0.0)).collect::<Vec<_>>();
        let base = CchPotential::new(topology, metric, &seeds, reversed, factor, scratch);
        if gates.is_empty() {
            return Self {
                base,
                gated: None,
                floor: 0.0,
            };
        }
        // Offsets in request costs; seeds in metric units (`get` scales
        // them back), lowered by a hair against rounding.
        let scale = base.factor;
        // Gates by the node they continue from (`end`), to relax the gates
        // ending where a settled gate starts.
        let mut by_end = FxHashMap::<u32, Vec<usize>>::default();
        for (index, gate) in gates.iter().enumerate() {
            by_end.entry(gate.end).or_default().push(index);
        }
        let chained = |mut offsets: Vec<f64>| {
            use std::cmp::Reverse;
            let mut heap = std::collections::BinaryHeap::new();
            for (index, offset) in offsets.iter().enumerate() {
                if offset.is_finite() {
                    heap.push((Reverse(OrderedCost(*offset)), index));
                }
            }
            while let Some((Reverse(OrderedCost(offset)), index)) = heap.pop() {
                if offset > offsets[index] {
                    continue;
                }
                for &other in by_end.get(&gates[index].node).into_iter().flatten() {
                    let value = gates[other].cost + offset;
                    if value < offsets[other] {
                        offsets[other] = value;
                        heap.push((Reverse(OrderedCost(value)), other));
                    }
                }
            }
            offsets
        };
        let build = |offsets: &[f64]| {
            let mut all = seeds.clone();
            all.extend(
                gates
                    .iter()
                    .zip(offsets)
                    .filter(|(_, offset)| offset.is_finite())
                    .map(|(gate, offset)| (gate.node, offset / scale * (1.0 - 1e-12))),
            );
            CchPotential::new(topology, metric, &all, reversed, factor, gated_scratch)
        };
        if let Some(offsets) = exact_offsets(topology, metric, reversed, scale, gates, |node| {
            base.get(node)
        }) {
            let potential = Self {
                gated: Some(build(&offsets)),
                base,
                floor,
            };
            return potential;
        }
        let mut offsets = chained(
            gates
                .iter()
                .map(|gate| {
                    let lower = if floor > 0.0 {
                        floor * base.get(gate.end)
                    } else {
                        0.0
                    };
                    gate.cost + lower
                })
                .collect(),
        );
        let mut potential = Self {
            gated: Some(build(&offsets)),
            base,
            floor,
        };
        for _ in 1..GATE_ROUNDS {
            let raised = chained(
                gates
                    .iter()
                    .map(|gate| gate.cost + potential.get(gate.end))
                    .collect(),
            );
            let higher = raised
                .iter()
                .zip(&offsets)
                .any(|(new, old)| *new > *old + 1e-9 * old.abs().max(1.0));
            if !higher {
                break;
            }
            offsets = raised;
            potential.gated = Some(build(&offsets));
        }
        potential
    }

    /// The lower bound at `node` (infinite: no target is reachable).
    #[inline]
    pub(crate) fn get(&self, node: u32) -> f64 {
        let Some(gated) = &self.gated else {
            return self.base.get(node);
        };
        let value = gated.get(node);
        if self.floor > 0.0 {
            value.max(self.floor * self.base.get(node))
        } else {
            value
        }
    }
}

/// CCH distances from any node to each of a set of gate nodes (forward;
/// `reversed`: from each gate node to any node), in metric units.
///
/// An up-down shortest path meets at a common elimination-tree ancestor `z`,
/// so `d(w, u) = min over z of up(w, z) + down(z, u)`. Each gate node's
/// search up its ancestors is bucketed at every ancestor; a query searches up
/// from its node and scans the buckets it meets.
pub(crate) struct GateDistances<'a> {
    topology: &'a CchTopology,
    /// The metric's weights in the direction leaving the query node.
    from_node: &'a [f32],
    buckets: FxHashMap<u32, Vec<(usize, f64)>>,
    count: usize,
}

impl<'a> GateDistances<'a> {
    pub(crate) fn new(
        topology: &'a CchTopology,
        metric: &'a CchMetric,
        reversed: bool,
        gate_nodes: &[u32],
    ) -> Self {
        // Forward: up(w, z) leaves `w` along arcs low → high (`up`), and
        // down(z, u) reaches `u` along arcs high → low (`down`), searched
        // from `u` upwards. Backward the other way round.
        let (from_node, into_gate) = if reversed {
            (&metric.down[..], &metric.up[..])
        } else {
            (&metric.up[..], &metric.down[..])
        };
        let mut buckets = FxHashMap::<u32, Vec<(usize, f64)>>::default();
        let mut space = Vec::new();
        for (index, node) in gate_nodes.iter().enumerate() {
            elimination_search(topology, *node, into_gate, &mut space);
            for (z, d) in &space {
                if d.is_finite() {
                    buckets.entry(*z).or_default().push((index, *d));
                }
            }
        }
        Self {
            topology,
            from_node,
            buckets,
            count: gate_nodes.len(),
        }
    }

    /// Distances from `node` to every gate node (`reversed`: from them).
    pub(crate) fn from(&self, node: u32) -> Vec<f64> {
        let mut distance = vec![f64::INFINITY; self.count];
        let mut space = Vec::new();
        elimination_search(self.topology, node, self.from_node, &mut space);
        for (z, d) in &space {
            if !d.is_finite() {
                continue;
            }
            for (index, down) in self.buckets.get(z).into_iter().flatten() {
                let value = d + down;
                if value < distance[*index] {
                    distance[*index] = value;
                }
            }
        }
        distance
    }
}

/// Elimination-tree search from `node` up its ancestors (in rank order):
/// (ancestor rank, distance) into `out`, with `weights[a]` the cost of arc
/// `a` in the direction searched.
fn elimination_search(t: &CchTopology, node: u32, weights: &[f32], out: &mut Vec<(u32, f64)>) {
    out.clear();
    let mut x = t.rank[node as usize];
    while x != NO_NODE {
        out.push((x, f64::INFINITY));
        x = t.parent[x as usize];
    }
    out[0].1 = 0.0;
    // Every upward arc of a node on the path leads to a later node on it.
    for i in 0..out.len() {
        let (x, dx) = out[i];
        if !dx.is_finite() {
            continue;
        }
        let arcs = t.first_up[x as usize] as usize..t.first_up[x as usize + 1] as usize;
        let mut j = i + 1;
        for (y, w) in t.up_head[arcs.clone()].iter().zip(&weights[arcs]) {
            while out[j].0 < *y {
                j += 1;
            }
            let value = dx + f64::from(*w);
            if value < out[j].1 {
                out[j].1 = value;
            }
        }
    }
}

/// Exact gate offsets (see [`GatedPotential`]), or `None` with more than
/// [`EXACT_GATE_NODES`] distinct gate nodes or ends.
///
/// `d(w, u)` between every gate end `w` and gate node `u` comes from their
/// elimination-tree searches: an up-down shortest path meets at a common
/// ancestor `z`, so `d(w, u) = min over z of up(w, z) + down(z, u)`, with the
/// nodes' searches bucketed at each ancestor. Forward, `up` follows arcs
/// away from `w` and `down` arcs into `u`; backward (paths from the origins,
/// so from `u` to `w`) the other way round. The offsets then settle in
/// increasing order, each gate relaxing the gates whose end reaches it.
fn exact_offsets(
    topology: &CchTopology,
    metric: &CchMetric,
    reversed: bool,
    scale: f64,
    gates: &[PotentialGate],
    base: impl Fn(u32) -> f64,
) -> Option<Vec<f64>> {
    // (the distances come from `GateDistances`)
    let mut nodes = gates.iter().map(|gate| gate.node).collect::<Vec<_>>();
    nodes.sort_unstable();
    nodes.dedup();
    let mut ends = gates.iter().map(|gate| gate.end).collect::<Vec<_>>();
    ends.sort_unstable();
    ends.dedup();
    if nodes.len() > EXACT_GATE_NODES || ends.len() > EXACT_GATE_NODES {
        return None;
    }
    let distances = GateDistances::new(topology, metric, reversed, &nodes);
    // distance[end][node], in metric units.
    let distance = ends
        .iter()
        .map(|end| distances.from(*end))
        .collect::<Vec<_>>();
    let end_row = |node: u32| ends.binary_search(&node).ok();
    let node_column = |node: u32| nodes.binary_search(&node).ok();
    let mut offsets = gates
        .iter()
        .map(|gate| gate.cost + base(gate.end))
        .collect::<Vec<_>>();
    let mut settled = vec![false; gates.len()];
    while let Some(next) = (0..gates.len())
        .filter(|index| !settled[*index] && offsets[*index].is_finite())
        .min_by(|a, b| offsets[*a].total_cmp(&offsets[*b]).then(a.cmp(b)))
    {
        settled[next] = true;
        let column = node_column(gates[next].node)?;
        for (index, gate) in gates.iter().enumerate() {
            if settled[index] {
                continue;
            }
            let row = end_row(gate.end)?;
            let value = gate.cost + scale * distance[row][column] + offsets[next];
            if value < offsets[index] {
                offsets[index] = value;
            }
        }
    }
    Some(offsets)
}

// -------------------------------------------------------------------------
// Read-only accessors for the regions prototype (`crate::regions`)
// -------------------------------------------------------------------------

#[cfg(feature = "experimental-regions")]
impl CchTopology {
    /// Nodes in the order.
    pub(crate) fn node_count(&self) -> usize {
        self.rank.len()
    }

    /// The package node at rank `rank`.
    pub(crate) fn node_at(&self, rank: u32) -> u32 {
        self.node[rank as usize]
    }

    /// The elimination-tree parent of rank `rank` (`None` at a root). Nodes
    /// in different subtrees of one node share no edge: every edge joins a
    /// node to one of its ancestors.
    pub(crate) fn parent_of(&self, rank: u32) -> Option<u32> {
        let parent = self.parent[rank as usize];
        (parent != NO_NODE).then_some(parent)
    }
}

// -------------------------------------------------------------------------
// One-to-all distances and via candidates (alternatives)
// -------------------------------------------------------------------------

impl CchTopology {
    /// Node-level distances from `node` to every node (`reversed`: from every
    /// node to `node`), by rank, in metric units (PHAST: the elimination-tree
    /// search up from `node`, then one sweep down the ranks).
    pub(crate) fn one_to_all(&self, metric: &CchMetric, node: u32, reversed: bool) -> Vec<f32> {
        let n = self.rank.len();
        // Forward: up the tree along arcs low → high (`up`), then down along
        // high → low (`down`). Backward the other way round.
        let (up_weights, down_weights) = if reversed {
            (&metric.down, &metric.up)
        } else {
            (&metric.up, &metric.down)
        };
        let mut distance = vec![f32::INFINITY; n];
        let mut space = Vec::new();
        elimination_search(self, node, up_weights, &mut space);
        for (x, d) in &space {
            distance[*x as usize] = *d as f32;
        }
        // Every upward neighbour has a higher rank: one sweep down the
        // ranks, in memory order.
        for x in (0..n).rev() {
            let arcs = self.first_up[x] as usize..self.first_up[x + 1] as usize;
            let mut best = distance[x];
            for (y, w) in self.up_head[arcs.clone()].iter().zip(&down_weights[arcs]) {
                let value = distance[*y as usize] + *w;
                if value < best {
                    best = value;
                }
            }
            distance[x] = best;
        }
        distance
    }
}

/// A via edge for an alternative route: the middle of a plateau, a stretch
/// of edges on both the shortest-path tree from the origin and the one to
/// the destination (Bader et al.), so that the route through it is locally
/// optimal over the plateau's length.
/// Single-edge via candidates after the plateaus (see [`via_candidates`]).
const VIA_FALLBACK_EDGES: usize = 24;

#[derive(Debug, Clone, Copy)]
pub(crate) struct ViaCandidate {
    pub edge: u32,
    /// Node-level cost of the route through the plateau (metric units).
    pub cost: f64,
    /// The plateau's length (metric units).
    pub plateau: f64,
}

/// Via candidates between nodes `source` and `target` on `metric` (customized
/// from per-edge `costs`), using none of the `excluded` edges, whose route
/// costs at most `max_ratio * D` (`D` the shortest distance): first the
/// plateaus at least `min_plateau * D` long, longest first; then, for short
/// detours a plateau cannot catch (both trees may leave a short branch at
/// its ends), up to [`VIA_FALLBACK_EDGES`] single edges by the cost of the
/// route through them (plateau 0).
#[allow(clippy::too_many_arguments)]
pub(crate) fn via_candidates(
    g: &Sections<'_>,
    topology: &CchTopology,
    metric: &CchMetric,
    costs: &[f32],
    source: u32,
    target: u32,
    max_ratio: f64,
    min_plateau: f64,
    excluded: &rustc_hash::FxHashSet<u32>,
) -> Vec<ViaCandidate> {
    let (from, to) = rayon::join(
        || topology.one_to_all(metric, source, false),
        || topology.one_to_all(metric, target, true),
    );
    let rank = &topology.rank;
    let total = f64::from(from[rank[target as usize] as usize]);
    if !total.is_finite() || total <= 0.0 {
        return Vec::new();
    }
    let bound = (max_ratio * total) as f32;
    // f32 sums along the sweeps: equal within a few ulps of the distance.
    let tolerance = |value: f32| 1e-5 * value.max(1.0) + 1e-5;
    let inside = |node: u32| {
        let r = rank[node as usize] as usize;
        from[r] + to[r] <= bound
    };
    // Plateau edges: (tail, edge, head), on both trees.
    let plateau = (0..g.node_count)
        .into_par_iter()
        .filter(|r| from[*r] + to[*r] <= bound)
        .filter_map(|ru| {
            let u = topology.node[ru];
            (g.first_out[u as usize]..g.first_out[u as usize + 1]).find_map(|e| {
                let w = g.edge_target[e as usize];
                let c = costs[e as usize];
                if w == u || !c.is_finite() || !inside(w) {
                    return None;
                }
                let rw = rank[w as usize] as usize;
                let on_from = (from[ru] + c - from[rw]).abs() <= tolerance(from[rw]);
                let on_to = (to[rw] + c - to[ru]).abs() <= tolerance(to[ru]);
                (on_from && on_to).then_some((u, e, w))
            })
        })
        .collect::<Vec<_>>();
    let next = plateau
        .iter()
        .map(|(u, e, w)| (*u, (*e, *w)))
        .collect::<FxHashMap<_, _>>();
    let heads = plateau
        .iter()
        .map(|(_, _, w)| *w)
        .collect::<rustc_hash::FxHashSet<_>>();
    let mut candidates = Vec::new();
    let mut seen = rustc_hash::FxHashSet::default();
    for (start, _, _) in &plateau {
        if heads.contains(start) || !seen.insert(*start) {
            continue;
        }
        let mut chain = Vec::new();
        let mut node = *start;
        while let Some((edge, head)) = next.get(&node) {
            chain.push((*edge, *head));
            if !seen.insert(*head) {
                break;
            }
            node = *head;
        }
        if chain.iter().any(|(edge, _)| excluded.contains(edge)) {
            continue;
        }
        let rs = rank[*start as usize] as usize;
        let length = f64::from(from[rank[node as usize] as usize] - from[rs]);
        if length < min_plateau * total {
            continue;
        }
        let middle = f64::from(from[rs]) + length / 2.0;
        let edge = chain
            .iter()
            .find(|(_, head)| f64::from(from[rank[*head as usize] as usize]) >= middle)
            .map_or(chain[0].0, |(edge, _)| *edge);
        candidates.push(ViaCandidate {
            edge,
            cost: f64::from(from[rs] + to[rs]),
            plateau: length,
        });
    }
    candidates.sort_by(|a, b| {
        b.plateau
            .total_cmp(&a.plateau)
            .then(a.cost.total_cmp(&b.cost))
            .then(a.edge.cmp(&b.edge))
    });
    // Single edges: the cheapest routes through an edge off the primary.
    let mut singles = (0..g.node_count)
        .into_par_iter()
        .filter(|r| from[*r] + to[*r] <= bound)
        .flat_map_iter(|ru| {
            let u = topology.node[ru];
            let (from, to) = (&from, &to);
            (g.first_out[u as usize]..g.first_out[u as usize + 1]).filter_map(move |e| {
                let w = g.edge_target[e as usize];
                let c = costs[e as usize];
                if w == u || !c.is_finite() || excluded.contains(&e) {
                    return None;
                }
                let cost = from[ru] + c + to[rank[w as usize] as usize];
                (cost <= bound).then_some((cost, e))
            })
        })
        .collect::<Vec<_>>();
    let keep = VIA_FALLBACK_EDGES.min(singles.len());
    if keep > 0 && keep < singles.len() {
        singles.select_nth_unstable_by(keep - 1, |a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        singles.truncate(keep);
    }
    singles.sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    candidates.extend(singles.into_iter().map(|(cost, edge)| ViaCandidate {
        edge,
        cost: f64::from(cost),
        plateau: 0.0,
    }));
    candidates
}

/// Compares the potentials of every stored column, both directions, from
/// every node with plain node-level Dijkstra: (pairs compared, mismatches
/// beyond 1e-6 relative or above the true distance).
#[cfg(test)]
#[allow(clippy::needless_range_loop)]
pub(crate) fn exactness_mismatches(g: &Sections<'_>, potentials: &CchPotentials) -> (usize, usize) {
    use std::cmp::Reverse;
    use std::collections::BinaryHeap;
    #[derive(Clone, Copy, PartialEq)]
    struct Key(f64);
    impl Eq for Key {}
    impl PartialOrd for Key {
        fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
            Some(self.cmp(other))
        }
    }
    impl Ord for Key {
        fn cmp(&self, other: &Self) -> std::cmp::Ordering {
            self.0.total_cmp(&other.0)
        }
    }
    let n = g.node_count;
    let scratch = RefCell::new(PotentialScratch::default());
    let (mut pairs, mut mismatches) = (0usize, 0usize);
    for (column, metric) in potentials.metrics.iter().enumerate() {
        let cost = &g.edge_cost[column * g.edge_count..(column + 1) * g.edge_count];
        let mut forward = vec![Vec::new(); n];
        let mut reverse = vec![Vec::new(); n];
        for u in 0..n {
            for e in g.first_out[u]..g.first_out[u + 1] {
                let c = cost[e as usize];
                let v = g.edge_target[e as usize] as usize;
                if c.is_finite() && v != u {
                    forward[u].push((v as u32, c));
                    reverse[v].push((u as u32, c));
                }
            }
        }
        for root in 0..n as u32 {
            // Forward potential: distances to `root` (Dijkstra on the reverse
            // graph); reversed: distances from `root`.
            for reversed in [false, true] {
                let adjacency = if reversed { &forward } else { &reverse };
                let mut dist = vec![f64::INFINITY; n];
                let mut heap = BinaryHeap::new();
                dist[root as usize] = 0.0;
                heap.push((Reverse(Key(0.0)), root));
                while let Some((Reverse(Key(d)), u)) = heap.pop() {
                    if d > dist[u as usize] {
                        continue;
                    }
                    for (v, c) in &adjacency[u as usize] {
                        let value = d + f64::from(*c);
                        if value < dist[*v as usize] {
                            dist[*v as usize] = value;
                            heap.push((Reverse(Key(value)), *v));
                        }
                    }
                }
                let potential = CchPotential::new(
                    &potentials.topology,
                    metric,
                    &[(root, 0.0)],
                    reversed,
                    1.0,
                    &scratch,
                );
                for node in 0..n as u32 {
                    let truth = dist[node as usize];
                    let value = potential.distance(node);
                    pairs += 1;
                    let exact = if truth.is_finite() {
                        value <= truth + 1e-9 * truth.max(1.0)
                            && (truth - value).abs() <= 1e-6 * truth.max(1.0)
                    } else {
                        !value.is_finite()
                    };
                    if !exact {
                        mismatches += 1;
                    }
                }
            }
        }
    }
    (pairs, mismatches)
}

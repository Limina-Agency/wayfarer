//! CCH lower-bound potentials ("CH-potentials", Strasser and Zeitz) for the
//! edge-based A* of `package::search`.
//!
//! Built from an open package on first use: the topology by the first search,
//! each stored column's metric by the first search on that column
//! ([`RoutingPackage::prepare_potentials`](super::RoutingPackage::prepare_potentials)
//! builds all of them):
//!
//! 1. **Order.** The package stores the node order (`Section::CchRank`, by
//!    nested dissection of the node graph; see `wayfarer_package::cch_order`,
//!    which the compiler runs).
//! 2. **Topology.** For each node by rank, its lowest upward neighbour `u`
//!    inherits its other upward neighbours; `u` is its elimination-tree
//!    parent. Arcs are stored once (lower → higher rank) with a weight per
//!    direction. Built in one pass in rank order: a node's upward list is
//!    final once its children (lower ranks) have been eliminated, so it is
//!    appended to the arc array directly.
//! 3. **Customization** per stored cost column: arcs start at the least column
//!    cost of the edges they stand for (turn costs left out, which keeps the
//!    metric a lower bound: turn costs are never negative), then every lower
//!    triangle is relaxed, node by node in elimination-tree level order, in
//!    parallel within a level ("pull": a node writes only its own arcs);
//!    above the last wide level, each node as soon as its children are done
//!    (see `CchTopology::customize`).
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

use std::sync::atomic::AtomicU32;
use std::sync::atomic::Ordering::Relaxed;
use std::sync::{Arc, Condvar, Mutex, OnceLock, PoisonError};
use std::time::Instant;

use rayon::prelude::*;
use rustc_hash::FxHashMap;

#[cfg(test)]
use wayfarer_package::cch_order::is_permutation;

use super::Sections;
use super::columns::MetricKey;
use super::paged::PagedSlots;

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
    /// Stored cost columns customized (all of them once
    /// [`RoutingPackage::prepare_potentials`](super::RoutingPackage::prepare_potentials)
    /// returns: searches customize a column on first use).
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
    /// The arrays only customization reads, built with the topology. Unless
    /// the topology keeps them ([`Self::keep_customization_arrays`]), each
    /// customization drops them when it ends and no other one is running,
    /// and the next one builds them again from the package.
    customization: Mutex<Option<Arc<CustomizationArrays>>>,
    keep_customization: bool,
}

/// The arrays only customization reads (see [`CchTopology::customization`]).
pub(crate) struct CustomizationArrays {
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
    /// The edges by blocks of [`SOURCE_BLOCK`] source nodes: where each
    /// block starts, then the edge count. All the edges of one arc and
    /// direction leave the same node (the lower one up, the higher one
    /// down), so they fall in one block.
    edge_blocks: Vec<u32>,
}

/// One customized metric: per arc, the weight lower → higher rank (`up`) and
/// higher → lower (`down`).
pub(crate) struct CchMetric {
    up: Vec<f32>,
    down: Vec<f32>,
    /// Unique in the process, so that a potential's memo can name the
    /// metric it was computed on (see [`PotentialScratch`]).
    id: u64,
}

impl CchMetric {
    /// The weights up and down, by arc (tests).
    #[cfg(test)]
    pub(crate) fn weights(&self) -> (&[f32], &[f32]) {
        (&self.up, &self.down)
    }
}

/// The next [`CchMetric::id`].
static NEXT_METRIC_ID: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// The topology and the metric of every stored column.
pub(crate) struct CchPotentials {
    pub topology: CchTopology,
    /// By stored column, customized on first use (see [`Self::stored_metric`]).
    metrics: Vec<StoredMetric>,
    /// The topology's part of [`Self::stats`].
    topology_stats: PotentialStats,
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

/// A stored column's metric, customized by the first search that needs it.
///
/// Customization runs in parallel on rayon. A thread that finds another one
/// customizing waits for it. The customizing thread itself can come back
/// here while it waits for its parallel work, when rayon runs another search
/// on it (cells of a matrix): it then customizes a copy, since waiting would
/// never end, and the first metric stored wins. The two are the same bits
/// (customization does not depend on the order of its parallel work).
#[derive(Default)]
struct StoredMetric {
    /// The metric and its customization time, milliseconds.
    ready: OnceLock<(CchMetric, f64)>,
    /// The thread customizing it, if any.
    building: Mutex<Option<std::thread::ThreadId>>,
    built: Condvar,
}

impl StoredMetric {
    fn get_or_customize(
        &self,
        topology: &CchTopology,
        g: &Sections<'_>,
        cost: &[f32],
    ) -> &CchMetric {
        if let Some((metric, _)) = self.ready.get() {
            return metric;
        }
        let me = std::thread::current().id();
        let owner = {
            let mut building = self.building.lock().unwrap_or_else(PoisonError::into_inner);
            loop {
                if let Some((metric, _)) = self.ready.get() {
                    return metric;
                }
                match *building {
                    Some(other) if other != me => {
                        building = self
                            .built
                            .wait(building)
                            .unwrap_or_else(PoisonError::into_inner);
                    }
                    Some(_) => break false,
                    None => {
                        *building = Some(me);
                        break true;
                    }
                }
            }
        };
        // Clears `building` and wakes the waiters when this returns, or
        // unwinds: one of them then customizes.
        let _done = owner.then_some(Built(self));
        let started = Instant::now();
        let metric = topology.customize(g, cost);
        let _ = self
            .ready
            .set((metric, started.elapsed().as_secs_f64() * 1e3));
        &self.ready.get().expect("a metric is stored").0
    }
}

/// See [`StoredMetric::get_or_customize`].
struct Built<'a>(&'a StoredMetric);

impl Drop for Built<'_> {
    fn drop(&mut self) {
        *self
            .0
            .building
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = None;
        self.0.built.notify_all();
    }
}

impl CchPotentials {
    /// The metric of stored column `column`, customized on first use;
    /// `None` when the package has no such column.
    pub(crate) fn stored_metric(&self, g: &Sections<'_>, column: usize) -> Option<&CchMetric> {
        let cost = g
            .edge_cost
            .get(column * g.edge_count..(column + 1) * g.edge_count)?;
        Some(
            self.metrics
                .get(column)?
                .get_or_customize(&self.topology, g, cost),
        )
    }

    /// The package's stored cost columns.
    pub(crate) fn stored_column_count(&self) -> usize {
        self.metrics.len()
    }

    /// The topology's statistics, and the stored columns' customized so far.
    pub(crate) fn stats(&self) -> PotentialStats {
        let mut stats = self.topology_stats.clone();
        stats.heap_bytes = self.topology.heap_bytes();
        for (metric, ms) in self.metrics.iter().filter_map(|m| m.ready.get()) {
            stats.column_count += 1;
            stats.customize_ms += ms;
            stats.heap_bytes += 4 * (metric.up.len() + metric.down.len());
        }
        stats
    }
}

impl CchPotentials {
    /// The metric of `key`, customized from `costs()` on a miss.
    pub(crate) fn own_metric(
        &self,
        g: &Sections<'_>,
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
            metric: self.topology.customize(g, &costs),
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
pub(crate) struct CchCache {
    ready: OnceLock<Option<Arc<CchPotentials>>>,
    /// Whether the topology keeps its customization arrays
    /// ([`CchTopology::keep_customization_arrays`]).
    keep_customization: bool,
}

impl Default for CchCache {
    fn default() -> Self {
        Self::new(true)
    }
}

impl CchCache {
    /// A cache whose topology keeps its customization arrays or not
    /// (`keep_customization`).
    pub(crate) fn new(keep_customization: bool) -> Self {
        Self {
            ready: OnceLock::new(),
            keep_customization,
        }
    }

    /// Builds the topology on the first call; later calls return it. Each
    /// stored column's metric is customized on first use
    /// ([`CchPotentials::stored_metric`]).
    pub(crate) fn prepare(&self, g: &Sections<'_>) -> Option<&Arc<CchPotentials>> {
        self.ready
            .get_or_init(|| CchPotentials::build(g, self.keep_customization).map(Arc::new))
            .as_ref()
    }

    /// [`Self::prepare`], with every stored column's metric customized.
    pub(crate) fn prepare_all(&self, g: &Sections<'_>) -> Option<&Arc<CchPotentials>> {
        let potentials = self.prepare(g)?;
        // Columns customize side by side: each one's narrow top keeps few
        // threads busy.
        (0..potentials.stored_column_count())
            .into_par_iter()
            .for_each(|column| {
                potentials.stored_metric(g, column);
            });
        Some(potentials)
    }
}

impl CchPotentials {
    fn build(g: &Sections<'_>, keep_customization: bool) -> Option<Self> {
        let (mut topology, topology_stats) = CchTopology::build(g)?;
        topology.keep_customization_arrays(keep_customization);
        let columns = g.edge_cost.len().checked_div(g.edge_count).unwrap_or(0);
        Some(Self {
            topology,
            metrics: (0..columns).map(|_| StoredMetric::default()).collect(),
            topology_stats,
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
        let n = g.node_count;
        let started = Instant::now();
        if g.cch_rank.len() != n {
            return None;
        }
        let (rank, node) = rank_and_node(g.cch_rank)?;
        stats.order_ms = started.elapsed().as_secs_f64() * 1e3;

        let started = Instant::now();
        // The node graph by rank, its elimination tree (Liu's algorithm on
        // the lower neighbours), then the elimination itself: a node's
        // upward neighbours are its own and its children's but the child
        // itself (each child's list starts with this node, its parent).
        // Subtrees of the tree are independent, so each task eliminates one
        // in post-order; the nodes above them follow in rank order. Every
        // list is a sorted set, whichever thread builds it, so the arrays
        // are the same on any number of threads.
        // On one or two threads the tasks cost more than they save, so the
        // elimination runs in rank order on one thread instead; the arrays
        // are the same.
        let (first_up, up_head, parent) = if rayon::current_num_threads() < PARALLEL_THREADS {
            eliminate_in_rank_order(g, &rank)?
        } else {
            let adjacency = RankAdjacency::build(g, &rank, &node)?;
            let lists = eliminate(&adjacency)?;
            drop(adjacency);
            let (first_up, up_head) = lists.assemble()?;
            (first_up, up_head, lists.parent)
        };
        stats.arc_count = up_head.len();
        let topology = Self {
            rank,
            node,
            first_up,
            up_head,
            parent,
            customization: Mutex::new(None),
            keep_customization: true,
        };
        let arrays = CustomizationArrays::build(g, &topology);
        // Levels count from the leaves, so there are as many as the longest
        // path down the tree has nodes.
        stats.elimination_tree_height = arrays.level_start.len() - 1;
        *topology
            .customization
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = Some(Arc::new(arrays));
        stats.topology_ms = started.elapsed().as_secs_f64() * 1e3;
        Some((topology, stats))
    }

    /// [`Self::build`] the way it was first written: from the node graph,
    /// with per-node upward lists that a node's parent inherits. Kept to check
    /// that `build` gives the same arrays.
    #[cfg(test)]
    pub(crate) fn build_reference(g: &Sections<'_>) -> Option<(Self, PotentialStats)> {
        let mut stats = PotentialStats {
            node_count: g.node_count,
            ..Default::default()
        };
        let started = Instant::now();
        let graph = wayfarer_package::cch_order::NodeGraph::new(g.first_out, g.edge_target);
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
        let arrays = CustomizationArrays {
            first_down,
            down_tail,
            down_arc,
            level_order,
            level_start,
            edge_arc,
            edge_blocks: source_blocks(g),
        };
        let topology = Self {
            rank,
            node,
            first_up,
            up_head,
            parent,
            customization: Mutex::new(Some(Arc::new(arrays))),
            keep_customization: true,
        };
        Some((topology, stats))
    }

    /// Whether `other` has the same arrays (tests); the customization
    /// arrays of both must be built.
    #[cfg(test)]
    pub(crate) fn same_arrays(&self, other: &Self) -> bool {
        let (ours, theirs) = (self.held_arrays(), other.held_arrays());
        let (Some(a), Some(b)) = (ours, theirs) else {
            return false;
        };
        self.rank == other.rank
            && self.node == other.node
            && self.first_up == other.first_up
            && self.up_head == other.up_head
            && self.parent == other.parent
            && a.first_down == b.first_down
            && a.down_tail == b.down_tail
            && a.down_arc == b.down_arc
            && a.level_order == b.level_order
            && a.level_start == b.level_start
            && a.edge_arc == b.edge_arc
            && a.edge_blocks == b.edge_blocks
    }

    /// Topology arrays in memory, with the customization arrays when they
    /// are held.
    fn heap_bytes(&self) -> usize {
        4 * (self.rank.len()
            + self.node.len()
            + self.first_up.len()
            + self.up_head.len()
            + self.parent.len())
            + self.held_arrays().map_or(0, |arrays| arrays.heap_bytes())
    }

    /// The customization arrays, if held now.
    pub(crate) fn held_arrays(&self) -> Option<Arc<CustomizationArrays>> {
        self.customization
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    /// Whether customizations keep their arrays (see [`Self::customization`]);
    /// a topology keeps them unless told otherwise. A topology that does not
    /// still holds the arrays it was built with until its first
    /// customization ends.
    pub(crate) fn keep_customization_arrays(&mut self, keep: bool) {
        self.keep_customization = keep;
    }

    /// The customization arrays, built from `g` when they are not held.
    fn customization_arrays(&self, g: &Sections<'_>) -> Arc<CustomizationArrays> {
        let mut held = self
            .customization
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        held.get_or_insert_with(|| Arc::new(CustomizationArrays::build(g, self)))
            .clone()
    }

    /// Drops the customization arrays when they are not kept and no
    /// customization holds them but the caller's `arrays`.
    fn release_customization_arrays(&self, arrays: Arc<CustomizationArrays>) {
        if self.keep_customization {
            return;
        }
        let mut held = self
            .customization
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        // The topology's and the caller's.
        if Arc::strong_count(&arrays) == 2 {
            *held = None;
        }
    }

    pub(crate) fn arc_count(&self) -> usize {
        self.up_head.len()
    }
}

/// A customization's view of a topology and its customization arrays.
struct Customizer<'a> {
    t: &'a CchTopology,
    c: &'a CustomizationArrays,
}

impl CchTopology {
    /// Customizes `cost` (a column of `g`: one cost per package edge,
    /// infinite = unusable); see [`Customizer::customize`].
    pub(crate) fn customize(&self, g: &Sections<'_>, cost: &[f32]) -> CchMetric {
        let arrays = self.customization_arrays(g);
        let metric = Customizer {
            t: self,
            c: &arrays,
        }
        .customize(cost);
        self.release_customization_arrays(arrays);
        metric
    }

    /// [`Customizer::customize_sequential`] (tests).
    #[cfg(test)]
    pub(crate) fn customize_sequential(
        &self,
        g: &Sections<'_>,
        cost: &[f32],
    ) -> (Vec<f32>, Vec<f32>) {
        let arrays = self.customization_arrays(g);
        Customizer {
            t: self,
            c: &arrays,
        }
        .customize_sequential(cost)
    }
}

impl Customizer<'_> {
    /// Input weights: per arc the least cost of its edges each way.
    ///
    /// In parallel by blocks of source nodes ([`CustomizationArrays::edge_blocks`]): the
    /// edges of one arc and direction are all in one block, so each weight is
    /// written by one task, from its edges in edge order, as a single pass
    /// would.
    fn input_weights(&self, cost: &[f32]) -> (Vec<f32>, Vec<f32>) {
        let arcs = self.t.arc_count();
        let mut up = vec![f32::INFINITY; arcs];
        let mut down = vec![f32::INFINITY; arcs];
        let up_ptr = SyncPtr(up.as_mut_ptr());
        let down_ptr = SyncPtr(down.as_mut_ptr());
        self.c.edge_blocks.par_windows(2).for_each(|block| {
            let edges = block[0] as usize..block[1] as usize;
            for (code, c) in self.c.edge_arc[edges.clone()].iter().zip(&cost[edges]) {
                if *code == NO_NODE || !c.is_finite() {
                    continue;
                }
                let a = (*code & !UP_BIT) as usize;
                let weights = if *code & UP_BIT != 0 {
                    up_ptr
                } else {
                    down_ptr
                };
                // SAFETY: only this block has edges of arc `a` this way.
                unsafe {
                    let weight = weights.0.add(a);
                    *weight = (*weight).min(*c);
                }
            }
        });
        (up, down)
    }

    /// Customizes `cost` (a column: one cost per package edge, infinite =
    /// unusable).
    ///
    /// Wide elimination-tree levels go bottom up, each relaxing its nodes in
    /// parallel. Above the last wide level (the top separators: few nodes,
    /// each with long arc lists), a node starts as soon as its children are
    /// done rather than with its level, and a node with much work splits its
    /// downward arcs among tasks ([`Self::relax_node_split`]). Every weight
    /// is the least of its input and its triangles' sums rounded down,
    /// whatever the order, so the metric is the same bits on any number of
    /// threads.
    pub(crate) fn customize(&self, cost: &[f32]) -> CchMetric {
        let (mut up, mut down) = self.input_weights(cost);
        let up_ptr = SyncPtr(up.as_mut_ptr());
        let down_ptr = SyncPtr(down.as_mut_ptr());
        let levels = self.c.level_start.len() - 1;
        // The levels from `top` up are all narrow.
        let top = (0..levels)
            .rev()
            .find(|level| {
                self.c.level_start[level + 1] - self.c.level_start[*level]
                    > NARROW_LEVEL_NODES as u32
            })
            .map_or(0, |level| level + 1);
        for level in 0..top {
            let ranks = &self.c.level_order
                [self.c.level_start[level] as usize..self.c.level_start[level + 1] as usize];
            // SAFETY: a node writes only its own arcs, and reads arcs of
            // lower levels (finished) or its own; nodes of one level are
            // distinct.
            ranks
                .par_iter()
                .for_each(|y| unsafe { self.relax_node(*y as usize, up_ptr, down_ptr) });
        }
        let narrow = &self.c.level_order[self.c.level_start[top] as usize..];
        if !narrow.is_empty() {
            // SAFETY: as `relax_top` requires: the levels below are done.
            unsafe { self.relax_top(narrow, up_ptr, down_ptr) };
        }
        CchMetric {
            up,
            down,
            id: NEXT_METRIC_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed),
        }
    }

    /// Relaxes the nodes `ranks`, the top levels of the elimination tree
    /// (every ancestor of one of them is one of them), each once its
    /// children among them are done.
    ///
    /// # Safety
    /// Every other node must be relaxed, and nothing else may write the
    /// arcs of `ranks` meanwhile.
    unsafe fn relax_top(&self, ranks: &[u32], up_ptr: SyncPtr, down_ptr: SyncPtr) {
        use std::sync::atomic::AtomicU32;
        let index = ranks
            .iter()
            .enumerate()
            .map(|(index, y)| (*y, index))
            .collect::<FxHashMap<_, _>>();
        let parent_index = ranks
            .iter()
            .map(|y| index.get(&self.t.parent[*y as usize]).copied())
            .collect::<Vec<_>>();
        let mut children = vec![0u32; ranks.len()];
        for parent in parent_index.iter().flatten() {
            children[*parent] += 1;
        }
        let ready = (0..ranks.len())
            .filter(|at| children[*at] == 0)
            .collect::<Vec<_>>();
        let pending = children.into_iter().map(AtomicU32::new).collect::<Vec<_>>();
        let top = Top {
            customizer: self,
            ranks,
            parent_index,
            pending,
            up_ptr,
            down_ptr,
        };
        let top = &top;
        rayon::scope(|scope| {
            for at in ready {
                scope.spawn(move |scope| top.run(scope, at));
            }
        });
    }

    /// The triangles node `y` relaxes, plus its downward arcs: about its
    /// share of a customization's work.
    fn triangle_work(&self, y: usize) -> usize {
        (self.c.first_down[y] as usize..self.c.first_down[y + 1] as usize)
            .map(|k| {
                let x = self.c.down_tail[k] as usize;
                self.t.first_up[x + 1] as usize - self.c.down_arc[k] as usize
            })
            .sum()
    }

    /// Relaxes every lower triangle of node `y`'s arcs (pull).
    ///
    /// # Safety
    /// Only `y` may write `y`'s arcs while this runs, and the arcs of `y`'s
    /// downward neighbours must be final.
    unsafe fn relax_node(&self, y: usize, up_ptr: SyncPtr, down_ptr: SyncPtr) {
        let ys = self.t.first_up[y] as usize;
        let downs = self.c.first_down[y] as usize..self.c.first_down[y + 1] as usize;
        // SAFETY: y's own arcs, which only this call writes.
        unsafe {
            self.relax_triangles(
                y,
                downs,
                up_ptr,
                down_ptr,
                up_ptr.0.add(ys),
                down_ptr.0.add(ys),
            );
        }
    }

    /// [`Self::relax_node`] with `y`'s downward arcs split into chunks of
    /// about [`SPLIT_CHUNK_WORK`], relaxed in parallel, each into weights of
    /// its own, which then lower `y`'s by their least: the same bits, since
    /// each weight is the least of the same values.
    ///
    /// # Safety
    /// As [`Self::relax_node`].
    unsafe fn relax_node_split(&self, y: usize, up_ptr: SyncPtr, down_ptr: SyncPtr) {
        let (ys, ye) = (self.t.first_up[y] as usize, self.t.first_up[y + 1] as usize);
        let degree = ye - ys;
        if degree == 0 {
            return;
        }
        // Chunks of downward arcs with about `SPLIT_CHUNK_WORK` each.
        let mut chunks = Vec::new();
        let downs = self.c.first_down[y] as usize..self.c.first_down[y + 1] as usize;
        let (mut start, mut work) = (downs.start, 0);
        for k in downs {
            let x = self.c.down_tail[k] as usize;
            work += self.t.first_up[x + 1] as usize - self.c.down_arc[k] as usize;
            if work >= SPLIT_CHUNK_WORK {
                chunks.push(start..k + 1);
                (start, work) = (k + 1, 0);
            }
        }
        if start < self.c.first_down[y + 1] as usize {
            chunks.push(start..self.c.first_down[y + 1] as usize);
        }
        let infinite = || (vec![f32::INFINITY; degree], vec![f32::INFINITY; degree]);
        let (least_up, least_down) = chunks
            .into_par_iter()
            .map(|downs| {
                let (mut up, mut down) = infinite();
                // SAFETY: reads arcs of x (final) and writes the chunk's own
                // weights.
                unsafe {
                    self.relax_triangles(
                        y,
                        downs,
                        up_ptr,
                        down_ptr,
                        up.as_mut_ptr(),
                        down.as_mut_ptr(),
                    );
                }
                (up, down)
            })
            .reduce(infinite, |(mut up, mut down), (other_up, other_down)| {
                for (weight, other) in up.iter_mut().zip(other_up) {
                    *weight = weight.min(other);
                }
                for (weight, other) in down.iter_mut().zip(other_down) {
                    *weight = weight.min(other);
                }
                (up, down)
            });
        // SAFETY: y's own arcs, which only this call writes.
        unsafe {
            for (j, (least_up, least_down)) in least_up.into_iter().zip(least_down).enumerate() {
                let (up, down) = (up_ptr.0.add(ys + j), down_ptr.0.add(ys + j));
                *up = (*up).min(least_up);
                *down = (*down).min(least_down);
            }
        }
    }

    /// Lowers `out_up[j]` and `out_down[j]`, the weights of `y`'s `j`-th
    /// upward arc (to `z`) each way, by the triangles `y`-`x`-`z` of `y`'s
    /// downward arcs `downs` (indices into `down_tail`).
    ///
    /// # Safety
    /// The arcs of every `x` must be final, and `out_up` and `out_down` must
    /// hold one weight per upward arc of `y` that nothing else writes.
    unsafe fn relax_triangles(
        &self,
        y: usize,
        downs: std::ops::Range<usize>,
        up_ptr: SyncPtr,
        down_ptr: SyncPtr,
        out_up: *mut f32,
        out_down: *mut f32,
    ) {
        let (ys, ye) = (self.t.first_up[y] as usize, self.t.first_up[y + 1] as usize);
        if ys == ye {
            return;
        }
        let heads = &self.t.up_head[ys..ye];
        let up = up_ptr.0;
        let down = down_ptr.0;
        for k in downs {
            let x = self.c.down_tail[k] as usize;
            let a_xy = self.c.down_arc[k] as usize;
            // SAFETY: arcs of x are final (lower level).
            let (w_xy, w_yx) = unsafe { (*up.add(a_xy), *down.add(a_xy)) };
            if !w_xy.is_finite() && !w_yx.is_finite() {
                continue;
            }
            let x_end = self.t.first_up[x + 1] as usize;
            let mut j = 0usize;
            for a_xz in a_xy + 1..x_end {
                let z = self.t.up_head[a_xz];
                while heads[j] < z {
                    j += 1;
                }
                debug_assert_eq!(heads[j], z);
                // SAFETY: as above; `j` is one of y's arcs.
                unsafe {
                    let (w_xz, w_zx) = (*up.add(a_xz), *down.add(a_xz));
                    // y -> x -> z
                    let via_up = f64::from(w_yx) + f64::from(w_xz);
                    if via_up < f64::from(*out_up.add(j)) {
                        *out_up.add(j) = round_down(via_up);
                    }
                    // z -> x -> y
                    let via_down = f64::from(w_zx) + f64::from(w_xy);
                    if via_down < f64::from(*out_down.add(j)) {
                        *out_down.add(j) = round_down(via_down);
                    }
                }
            }
        }
    }

    /// [`Self::customize`] the way it was first written: input weights and
    /// nodes in sequence. Kept to check that `customize` gives the same bits.
    #[cfg(test)]
    pub(crate) fn customize_sequential(&self, cost: &[f32]) -> (Vec<f32>, Vec<f32>) {
        let arcs = self.t.arc_count();
        let mut up = vec![f32::INFINITY; arcs];
        let mut down = vec![f32::INFINITY; arcs];
        for (edge, code) in self.c.edge_arc.iter().enumerate() {
            let c = cost[edge];
            if *code == NO_NODE || !c.is_finite() {
                continue;
            }
            let a = (*code & !UP_BIT) as usize;
            if *code & UP_BIT != 0 {
                up[a] = up[a].min(c);
            } else {
                down[a] = down[a].min(c);
            }
        }
        let up_ptr = SyncPtr(up.as_mut_ptr());
        let down_ptr = SyncPtr(down.as_mut_ptr());
        for y in &self.c.level_order {
            // SAFETY: one node at a time, in level order.
            unsafe { self.relax_node(*y as usize, up_ptr, down_ptr) };
        }
        (up, down)
    }
}

// -------------------------------------------------------------------------
// Topology build, in parallel
// -------------------------------------------------------------------------

/// Ranks per chunk of the topology build's parallel passes (tests: few, so
/// that the small test packages have several chunks).
const RANK_CHUNK: usize = if cfg!(test) { 8 } else { 16_384 };
/// Subtrees of the elimination tree of at most this many nodes are
/// eliminated by one task each (tests: tiny, so that the small test packages
/// have tasks and nodes above them too).
const TASK_NODES: u32 = if cfg!(test) { 4 } else { 4096 };
/// [`Lists::owner`] of a rank above the tasks' subtrees (and of one not
/// placed yet).
const TOP: u32 = NO_NODE;

/// `values` without their atomics.
fn plain(values: Vec<AtomicU32>) -> Vec<u32> {
    values.into_iter().map(AtomicU32::into_inner).collect()
}

/// `len` zeros to fill from many threads.
fn atomic_zeros(len: usize) -> Vec<AtomicU32> {
    (0..len)
        .into_par_iter()
        .with_min_len(RANK_CHUNK)
        .map(|_| AtomicU32::new(0))
        .collect()
}

/// Exclusive prefix sums of `counts` (one per rank) as `first`, `n + 1`
/// long; `None` when the total reaches `limit`.
fn prefix_sums(counts: &[u32], limit: u64) -> Option<Vec<u32>> {
    let mut first = Vec::with_capacity(counts.len() + 1);
    let mut total = 0u64;
    first.push(0);
    for count in counts {
        total += u64::from(*count);
        if total >= limit {
            return None;
        }
        first.push(total as u32);
    }
    Some(first)
}

/// `data`, a CSR array with ranges `first[x]..first[x + 1]` by rank, cut
/// into the ranges of chunks of [`RANK_CHUNK`] ranks: (the chunk's first
/// rank, its part of `data`).
fn rank_chunks<'a, T>(mut data: &'a mut [T], first: &[u32]) -> Vec<(usize, &'a mut [T])> {
    let n = first.len() - 1;
    let mut chunks = Vec::with_capacity(n.div_ceil(RANK_CHUNK));
    for start in (0..n).step_by(RANK_CHUNK) {
        let end = (start + RANK_CHUNK).min(n);
        let (chunk, rest) = data.split_at_mut((first[end] - first[start]) as usize);
        chunks.push((start, chunk));
        data = rest;
    }
    chunks
}

/// Threads from which the topology build runs in parallel (on Małopolskie
/// the parallel build is slower on two threads and faster on three).
const PARALLEL_THREADS: usize = 3;

/// The upward lists (`first_up`, `up_head`) and the elimination tree, on one
/// thread: the node graph's upward neighbours by rank (lower → higher),
/// grouped by a counting sort on the lower rank, then the elimination in
/// rank order. A node's upward neighbours are its own and its children's
/// but the child itself (each child's list starts with this node, its
/// parent). Children have lower ranks, so their lists are final, and so is
/// this node's once built: it goes to the arc array as it is. Repeats (an
/// edge and its twin) are dropped by the elimination. `None` when the arcs
/// outgrow the 31-bit arc ids.
fn eliminate_in_rank_order(
    g: &Sections<'_>,
    rank: &[u32],
) -> Option<(Vec<u32>, Vec<u32>, Vec<u32>)> {
    let n = g.node_count;
    let ends = |u: usize, e: u32| {
        let v = g.edge_target[e as usize] as usize;
        (v != u).then(|| {
            let (ru, rv) = (rank[u], rank[v]);
            if ru < rv { (ru, rv) } else { (rv, ru) }
        })
    };
    let mut first_orig = vec![0u32; n + 1];
    for u in 0..n {
        for e in g.first_out[u]..g.first_out[u + 1] {
            if let Some((lo, _)) = ends(u, e) {
                first_orig[lo as usize + 1] += 1;
            }
        }
    }
    for x in 0..n {
        first_orig[x + 1] += first_orig[x];
    }
    let mut fill = first_orig.clone();
    let mut orig_head = vec![0u32; first_orig[n] as usize];
    for u in 0..n {
        for e in g.first_out[u]..g.first_out[u + 1] {
            if let Some((lo, hi)) = ends(u, e) {
                orig_head[fill[lo as usize] as usize] = hi;
                fill[lo as usize] += 1;
            }
        }
    }
    drop(fill);
    let mut first_up = Vec::with_capacity(n + 1);
    first_up.push(0u32);
    let mut up_head: Vec<u32> = Vec::with_capacity(orig_head.len());
    let mut parent = vec![NO_NODE; n];
    let mut first_child = vec![NO_NODE; n];
    let mut next_sibling = vec![NO_NODE; n];
    let mut list: Vec<u32> = Vec::new();
    for x in 0..n {
        list.clear();
        list.extend_from_slice(&orig_head[first_orig[x] as usize..first_orig[x + 1] as usize]);
        let mut child = first_child[x];
        while child != NO_NODE {
            let c = child as usize;
            list.extend_from_slice(&up_head[first_up[c] as usize + 1..first_up[c + 1] as usize]);
            child = next_sibling[c];
        }
        list.sort_unstable();
        list.dedup();
        if let Some(&p) = list.first() {
            parent[x] = p;
            next_sibling[x] = first_child[p as usize];
            first_child[p as usize] = x as u32;
        }
        up_head.extend_from_slice(&list);
        if up_head.len() as u64 >= u64::from(UP_BIT) {
            return None;
        }
        first_up.push(up_head.len() as u32);
    }
    drop((orig_head, first_orig, first_child, next_sibling, list));
    up_head.shrink_to_fit();
    Some((first_up, up_head, parent))
}

/// The stored order (node → rank) and its inverse; `None` unless it is a
/// permutation.
fn rank_and_node(cch_rank: &[u32]) -> Option<(Vec<u32>, Vec<u32>)> {
    let n = cch_rank.len();
    if n >= NO_NODE as usize {
        return None;
    }
    let node: Vec<AtomicU32> = (0..n)
        .into_par_iter()
        .with_min_len(RANK_CHUNK)
        .map(|_| AtomicU32::new(NO_NODE))
        .collect();
    // n ranks below n, each claimed once: a permutation.
    let claimed = cch_rank
        .par_iter()
        .with_min_len(RANK_CHUNK)
        .enumerate()
        .all(|(v, r)| {
            node.get(*r as usize).is_some_and(|slot| {
                slot.compare_exchange(NO_NODE, v as u32, Relaxed, Relaxed)
                    .is_ok()
            })
        });
    claimed.then(|| (cch_rank.to_vec(), plain(node)))
}

/// The node graph by rank: per rank, the ranks of its neighbours along every
/// edge but loops, out-edges and in-edges alike (an edge and its twin give
/// the same neighbour twice): those below it, then those above.
struct RankAdjacency {
    first: Vec<u32>,
    /// Where each rank's neighbours above it start.
    mid: Vec<u32>,
    heads: Vec<u32>,
}

impl RankAdjacency {
    fn build(g: &Sections<'_>, rank: &[u32], node: &[u32]) -> Option<Self> {
        let n = g.node_count;
        // A node's in-edges are the twins of its out-edges, whose sources
        // are those out-edges' targets, and its lone in-edges.
        let lone_in = |u: usize| {
            let present = g
                .lone_in_present
                .get(u / 64)
                .is_some_and(|bits| bits & (1 << (u % 64)) != 0);
            let target = |edge: &u32| g.edge_target[*edge as usize] as usize;
            let (start, end) = if present {
                (
                    g.lone_in_edge.partition_point(|edge| target(edge) < u),
                    g.lone_in_edge.partition_point(|edge| target(edge) <= u),
                )
            } else {
                (0, 0)
            };
            g.lone_in_edge[start..end].iter().copied()
        };
        let neighbours = |x: usize| {
            let u = node[x] as usize;
            let out = (g.first_out[u]..g.first_out[u + 1]).map(|e| g.edge_target[e as usize]);
            let into = lone_in(u).map(|edge| g.edge_source(edge as usize));
            out.chain(into)
                .filter(move |v| *v as usize != u)
                .map(|v| rank[v as usize])
        };
        // Per rank: its neighbours, and those below it.
        let (counts, lower): (Vec<u32>, Vec<u32>) = (0..n)
            .into_par_iter()
            .with_min_len(RANK_CHUNK)
            .map(|x| {
                neighbours(x).fold((0, 0), |(all, below), r| {
                    (all + 1, below + u32::from((r as usize) < x))
                })
            })
            .unzip();
        let first = prefix_sums(&counts, u64::from(u32::MAX))?;
        drop(counts);
        let mid = (0..n)
            .into_par_iter()
            .with_min_len(RANK_CHUNK)
            .map(|x| first[x] + lower[x])
            .collect();
        drop(lower);
        let mut heads = vec![0u32; first[n] as usize];
        rank_chunks(&mut heads, &first)
            .into_par_iter()
            .for_each(|(start, chunk)| {
                let mut at = 0;
                for x in start..(start + RANK_CHUNK).min(n) {
                    let slot = &mut chunk[at..at + (first[x + 1] - first[x]) as usize];
                    let (mut below, mut above) = (0, slot.len());
                    for r in neighbours(x) {
                        if (r as usize) < x {
                            slot[below] = r;
                            below += 1;
                        } else {
                            above -= 1;
                            slot[above] = r;
                        }
                    }
                    at += slot.len();
                }
            });
        Some(Self { first, mid, heads })
    }

    /// Rank `x`'s neighbours below it.
    fn lower(&self, x: usize) -> &[u32] {
        &self.heads[self.first[x] as usize..self.mid[x] as usize]
    }

    /// Rank `x`'s neighbours above it.
    fn upper(&self, x: usize) -> &[u32] {
        &self.heads[self.mid[x] as usize..self.first[x + 1] as usize]
    }
}

/// Every rank's upward list (see [`eliminate`]), in the buffers of the
/// task, or the top, that built it.
struct Lists {
    /// The elimination-tree parent of every rank: the lowest of its upward
    /// neighbours, `NO_NODE` for a root.
    parent: Vec<u32>,
    /// By rank: the task that built its list, or [`TOP`].
    owner: Vec<u32>,
    tasks: Vec<Vec<u32>>,
    top: Vec<u32>,
    start: Vec<u32>,
    len: Vec<u32>,
}

/// The elimination tree's child links while it is built: children by
/// sibling lists, each node's list complete once the tree reaches past it.
struct ChildLinks {
    first_child: Vec<AtomicU32>,
    next_sibling: Vec<AtomicU32>,
    /// Each rank's parent (`NO_NODE` for a root), set as the parent's list
    /// is built.
    parent: Vec<AtomicU32>,
}

impl ChildLinks {
    fn children(&self, x: usize) -> impl Iterator<Item = usize> + '_ {
        let mut child = self.first_child[x].load(Relaxed);
        std::iter::from_fn(move || {
            (child != NO_NODE).then(|| {
                let c = child as usize;
                child = self.next_sibling[c].load(Relaxed);
                c
            })
        })
    }
}

/// Where each rank's list is while the tasks run.
struct Placement {
    start: Vec<AtomicU32>,
    len: Vec<AtomicU32>,
    owner: Vec<AtomicU32>,
}

impl Placement {
    fn range(&self, x: usize) -> std::ops::Range<usize> {
        let start = self.start[x].load(Relaxed) as usize;
        start..start + self.len[x].load(Relaxed) as usize
    }

    fn set(&self, x: usize, start: usize, len: usize, owner: u32) {
        self.start[x].store(start as u32, Relaxed);
        self.len[x].store(len as u32, Relaxed);
        self.owner[x].store(owner, Relaxed);
    }
}

/// Builds rank `x`'s upward list into `list`: its upward neighbours and its
/// children's lists but the child itself (each child's list starts with
/// its parent, this rank), as a sorted set. `list_of` finds a child's.
fn eliminate_one<'b>(
    adjacency: &RankAdjacency,
    links: &ChildLinks,
    x: usize,
    list: &mut Vec<u32>,
    list_of: impl Fn(usize) -> &'b [u32],
) {
    list.clear();
    list.extend_from_slice(adjacency.upper(x));
    for child in links.children(x) {
        links.parent[child].store(x as u32, Relaxed);
        let child_list = list_of(child);
        debug_assert_eq!(child_list.first(), Some(&(x as u32)));
        list.extend_from_slice(&child_list[1..]);
    }
    list.sort_unstable();
    list.dedup();
}

/// The lists of the subtrees of `roots` (complete), each in post-order:
/// every child before its parent. `None` when they outgrow the 31-bit arc
/// ids.
fn eliminate_subtrees(
    adjacency: &RankAdjacency,
    links: &ChildLinks,
    placement: &Placement,
    roots: &[u32],
    task: u32,
) -> Option<Vec<u32>> {
    let mut heads = Vec::new();
    let mut list = Vec::new();
    let mut stack = Vec::new();
    for root in roots {
        let root = *root as usize;
        stack.push((root, links.first_child[root].load(Relaxed)));
        while let Some((x, next)) = stack.last_mut() {
            let x = *x;
            if *next != NO_NODE {
                let child = *next as usize;
                *next = links.next_sibling[child].load(Relaxed);
                stack.push((child, links.first_child[child].load(Relaxed)));
                continue;
            }
            stack.pop();
            eliminate_one(adjacency, links, x, &mut list, |c| {
                &heads[placement.range(c)]
            });
            placement.set(x, heads.len(), list.len(), task);
            heads.extend_from_slice(&list);
            if heads.len() as u64 >= u64::from(UP_BIT) {
                return None;
            }
        }
    }
    Some(heads)
}

/// Marks a subtree size in [`eliminate`] whose subtree holds a task's.
const HOLDS_TASK: u32 = 1 << 31;

/// Small trees waiting for a task, and their nodes.
#[derive(Default)]
struct Batch {
    roots: Vec<u32>,
    nodes: u32,
}

impl Batch {
    /// Adds the tree of `root`, with `size` nodes; the batch to spawn once
    /// it holds [`TASK_NODES`].
    fn add(&mut self, root: usize, size: u32) -> Option<Vec<u32>> {
        self.roots.push(root as u32);
        self.nodes += size;
        (self.nodes >= TASK_NODES).then(|| {
            self.nodes = 0;
            std::mem::take(&mut self.roots)
        })
    }
}

/// Every rank's upward list. The elimination tree comes first, by Liu's
/// algorithm: each lower neighbour's tree so far, found through ancestors
/// with path compression, hangs from the rank that reaches it first. A tree
/// is complete once hung (its nodes all rank below its root), so tasks
/// eliminate trees while the rest of the tree is still being built: each
/// tree of at least [`TASK_NODES`] nodes with no task's inside, and the
/// smaller trees hanging from a rank whose subtree holds a task's, in
/// batches of about as many nodes. The ranks whose subtrees hold a task's
/// follow in rank order. `None` when the arcs outgrow the 31-bit arc ids.
fn eliminate(adjacency: &RankAdjacency) -> Option<Lists> {
    let n = adjacency.first.len() - 1;
    let no_nodes = || -> Vec<AtomicU32> {
        (0..n)
            .into_par_iter()
            .with_min_len(RANK_CHUNK)
            .map(|_| AtomicU32::new(NO_NODE))
            .collect()
    };
    let links = ChildLinks {
        first_child: no_nodes(),
        next_sibling: no_nodes(),
        parent: no_nodes(),
    };
    let placement = Placement {
        start: atomic_zeros(n),
        len: atomic_zeros(n),
        owner: no_nodes(),
    };
    let results: Mutex<Vec<(u32, Option<Vec<u32>>)>> = Mutex::new(Vec::new());
    let run = |roots: &[u32], task: u32| {
        let heads = eliminate_subtrees(adjacency, &links, &placement, roots, task);
        results
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .push((task, heads));
    };
    let mut holds_task = Vec::new();
    let mut tasks = 0u32;
    let mut batch = Batch::default();
    rayon::scope(|scope| {
        let (links, run) = (&links, &run);
        let mut spawn = |roots: Vec<u32>| {
            let task = tasks;
            tasks += 1;
            scope.spawn(move |_| run(&roots, task));
        };
        // Per rank, side by side (one cache line): its ancestor so far, and
        // its subtree's size with [`HOLDS_TASK`] when the subtree holds a
        // task's.
        let mut nodes = vec![[NO_NODE, 1u32]; n];
        for x in 0..n {
            for low in adjacency.lower(x) {
                let mut r = *low as usize;
                loop {
                    let next = nodes[r][0];
                    if next == x as u32 {
                        break;
                    }
                    nodes[r][0] = x as u32;
                    if next != NO_NODE {
                        r = next as usize;
                        continue;
                    }
                    // r's tree hangs from x.
                    let [_, r_size] = nodes[r];
                    nodes[x][1] += r_size & !HOLDS_TASK;
                    let sibling = links.first_child[x].load(Relaxed);
                    links.next_sibling[r].store(sibling, Relaxed);
                    links.first_child[x].store(r as u32, Relaxed);
                    let r_holds = r_size & HOLDS_TASK != 0;
                    let large = r_size >= TASK_NODES;
                    if nodes[x][1] & HOLDS_TASK != 0 {
                        if r_holds {
                        } else if large {
                            spawn(vec![r as u32]);
                        } else if let Some(roots) = batch.add(r, r_size) {
                            spawn(roots);
                        }
                    } else if r_holds || large {
                        // x's subtree now holds a task's: its other
                        // children so far wait no longer.
                        nodes[x][1] |= HOLDS_TASK;
                        for child in links.children(x).skip(1) {
                            if let Some(roots) = batch.add(child, nodes[child][1]) {
                                spawn(roots);
                            }
                        }
                        if !r_holds {
                            spawn(vec![r as u32]);
                        }
                    }
                    break;
                }
            }
        }
        // Whole trees (never hung), and which ranks hold a task.
        for (root, [ancestor, size]) in nodes.iter().enumerate() {
            if *ancestor == NO_NODE
                && size & HOLDS_TASK == 0
                && let Some(roots) = batch.add(root, *size)
            {
                spawn(roots);
            }
        }
        if !batch.roots.is_empty() {
            spawn(std::mem::take(&mut batch.roots));
        }
        holds_task = nodes
            .par_iter()
            .with_min_len(RANK_CHUNK)
            .map(|[_, size]| size & HOLDS_TASK != 0)
            .collect();
    });
    let mut buffers = std::mem::take(&mut *results.lock().unwrap_or_else(PoisonError::into_inner));
    buffers.sort_unstable_by_key(|(task, _)| *task);
    let tasks = buffers
        .into_iter()
        .map(|(_, heads)| heads)
        .collect::<Option<Vec<_>>>()?;
    // The ranks whose subtrees hold a task's, in rank order.
    let mut top = Vec::new();
    let mut list = Vec::new();
    for x in (0..n).filter(|x| holds_task[*x]) {
        eliminate_one(adjacency, &links, x, &mut list, |c| {
            match placement.owner[c].load(Relaxed) {
                TOP => &top[placement.range(c)],
                task => &tasks[task as usize][placement.range(c)],
            }
        });
        placement.set(x, top.len(), list.len(), TOP);
        top.extend_from_slice(&list);
        if top.len() as u64 >= u64::from(UP_BIT) {
            return None;
        }
    }
    Some(Lists {
        parent: plain(links.parent),
        owner: plain(placement.owner),
        tasks,
        top,
        start: plain(placement.start),
        len: plain(placement.len),
    })
}

impl Lists {
    /// The lists as one array by rank (`first_up`, `up_head`).
    fn assemble(&self) -> Option<(Vec<u32>, Vec<u32>)> {
        let n = self.len.len();
        let first_up = prefix_sums(&self.len, u64::from(UP_BIT))?;
        let mut up_head = vec![0u32; first_up[n] as usize];
        rank_chunks(&mut up_head, &first_up)
            .into_par_iter()
            .for_each(|(start, chunk)| {
                let mut at = 0;
                for x in start..(start + RANK_CHUNK).min(n) {
                    let buffer = match self.owner[x] {
                        TOP => &self.top,
                        task => &self.tasks[task as usize],
                    };
                    let (s, len) = (self.start[x] as usize, self.len[x] as usize);
                    chunk[at..at + len].copy_from_slice(&buffer[s..s + len]);
                    at += len;
                }
            });
        Some((first_up, up_head))
    }
}

impl CustomizationArrays {
    /// The arrays of `topology`, the topology of `g`.
    fn build(g: &Sections<'_>, topology: &CchTopology) -> Self {
        let (
            (first_down, down_tail, down_arc),
            ((level_order, level_start), (edge_arc, edge_blocks)),
        ) = rayon::join(
            || Self::down_lists(topology),
            || rayon::join(|| Self::levels(topology), || Self::edge_arcs(g, topology)),
        );
        Self {
            first_down,
            down_tail,
            down_arc,
            level_order,
            level_start,
            edge_arc,
            edge_blocks,
        }
    }

    /// By rank: downward arcs as (lower node, its arc to this one), in rank
    /// order (`first_down`, `down_tail`, `down_arc`).
    fn down_lists(topology: &CchTopology) -> (Vec<u32>, Vec<u32>, Vec<u32>) {
        let (first_up, up_head) = (topology.first_up.as_slice(), topology.up_head.as_slice());
        let n = first_up.len() - 1;
        let arcs = up_head.len();
        // Downward lists: each of a few ranges of upper ends goes over every
        // arc in rank order and keeps its own, so the lists come out in rank
        // order, as one pass would fill them.
        let parts = rayon::current_num_threads().clamp(1, n.max(1));
        let bounds = (0..=parts).map(|p| p * n / parts).collect::<Vec<_>>();
        let mut counts = vec![0u32; n];
        let mut rest = counts.as_mut_slice();
        let mut slices = Vec::with_capacity(parts);
        for p in 0..parts {
            let (slice, tail) = rest.split_at_mut(bounds[p + 1] - bounds[p]);
            slices.push((bounds[p], slice));
            rest = tail;
        }
        slices.into_par_iter().for_each(|(low, counts)| {
            let high = low + counts.len();
            for y in up_head {
                let y = *y as usize;
                if (low..high).contains(&y) {
                    counts[y - low] += 1;
                }
            }
        });
        let first_down =
            prefix_sums(&counts, u64::from(u32::MAX) + 1).expect("as many downward arcs as arcs");
        drop(counts);
        // Most arcs lead to high ranks: the fill splits the arcs evenly.
        let bounds = (0..=parts)
            .map(|p| match p {
                0 => 0,
                p if p == parts => n,
                p => first_down.partition_point(|first| (*first as usize) < p * arcs / parts),
            })
            .collect::<Vec<_>>();
        let (mut down_tail, mut down_arc) = (vec![0u32; arcs], vec![0u32; arcs]);
        let (mut tails, mut arcs_of) = (down_tail.as_mut_slice(), down_arc.as_mut_slice());
        let mut slices = Vec::with_capacity(parts);
        for p in 0..parts {
            let len = (first_down[bounds[p + 1]] - first_down[bounds[p]]) as usize;
            let (tail_slice, tail_rest) = tails.split_at_mut(len);
            let (arc_slice, arc_rest) = arcs_of.split_at_mut(len);
            slices.push((bounds[p], bounds[p + 1], tail_slice, arc_slice));
            (tails, arcs_of) = (tail_rest, arc_rest);
        }
        slices
            .into_par_iter()
            .for_each(|(low, high, tails, arcs_of)| {
                let base = first_down[low];
                let mut fill = first_down[low..high]
                    .iter()
                    .map(|first| first - base)
                    .collect::<Vec<_>>();
                for x in 0..n {
                    for a in first_up[x]..first_up[x + 1] {
                        let y = up_head[a as usize] as usize;
                        if (low..high).contains(&y) {
                            let at = &mut fill[y - low];
                            tails[*at as usize] = x as u32;
                            arcs_of[*at as usize] = a;
                            *at += 1;
                        }
                    }
                }
            });
        (first_down, down_tail, down_arc)
    }

    /// Ranks in elimination-tree level order, leaves first, and where each
    /// level starts (`level_order`, `level_start`).
    fn levels(topology: &CchTopology) -> (Vec<u32>, Vec<u32>) {
        let parent = topology.parent.as_slice();
        let n = parent.len();
        // Elimination-tree levels, then the ranks by level (in rank order
        // within a level), by chunks of ranks in parallel.
        let mut level = vec![0u32; n];
        for x in 0..n {
            let p = parent[x];
            if p != NO_NODE {
                level[p as usize] = level[p as usize].max(level[x] + 1);
            }
        }
        let level_count = level.iter().copied().max().map_or(0, |l| l as usize + 1);
        let mut at: Vec<Vec<u32>> = level
            .par_chunks(RANK_CHUNK)
            .map(|levels| {
                let mut count = vec![0u32; level_count];
                for l in levels {
                    count[*l as usize] += 1;
                }
                count
            })
            .collect();
        let mut level_start = Vec::with_capacity(level_count + 1);
        let mut total = 0u32;
        for l in 0..level_count {
            level_start.push(total);
            for chunk in &mut at {
                let count = chunk[l];
                chunk[l] = total;
                total += count;
            }
        }
        level_start.push(total);
        let level_order = atomic_zeros(n);
        level
            .par_chunks(RANK_CHUNK)
            .zip(at)
            .enumerate()
            .for_each(|(chunk, (levels, mut at))| {
                for (i, l) in levels.iter().enumerate() {
                    let slot = &mut at[*l as usize];
                    level_order[*slot as usize].store((chunk * RANK_CHUNK + i) as u32, Relaxed);
                    *slot += 1;
                }
            });
        let level_order = plain(level_order);
        (level_order, level_start)
    }

    /// Per package edge its arc, and the edge blocks (`edge_arc`,
    /// `edge_blocks`).
    fn edge_arcs(g: &Sections<'_>, topology: &CchTopology) -> (Vec<u32>, Vec<u32>) {
        let n = g.node_count;
        let (rank, first_up, up_head) = (
            topology.rank.as_slice(),
            topology.first_up.as_slice(),
            topology.up_head.as_slice(),
        );
        let ends = |u: usize, e: u32| {
            let v = g.edge_target[e as usize] as usize;
            (v != u).then(|| {
                let (ru, rv) = (rank[u], rank[v]);
                if ru < rv { (ru, rv) } else { (rv, ru) }
            })
        };
        // Package edges → arcs, by blocks of source nodes.
        let mut edge_arc = vec![NO_NODE; g.edge_count];
        const BLOCK: usize = SOURCE_BLOCK;
        let edge_blocks = source_blocks(g);
        let mut rest = edge_arc.as_mut_slice();
        let mut chunks = Vec::with_capacity(edge_blocks.len());
        for pair in edge_blocks.windows(2) {
            let (chunk, tail) = rest.split_at_mut((pair[1] - pair[0]) as usize);
            chunks.push(chunk);
            rest = tail;
        }
        chunks
            .into_par_iter()
            .enumerate()
            .for_each(|(block, arcs_of)| {
                let first_edge = g.first_out[block * BLOCK] as usize;
                for u in block * BLOCK..((block + 1) * BLOCK).min(n) {
                    for e in g.first_out[u]..g.first_out[u + 1] {
                        let Some((lo, hi)) = ends(u, e) else {
                            continue;
                        };
                        let list = &up_head
                            [first_up[lo as usize] as usize..first_up[lo as usize + 1] as usize];
                        let at = list.binary_search(&hi).expect("every edge has an arc");
                        let arc = first_up[lo as usize] + at as u32;
                        arcs_of[e as usize - first_edge] =
                            if rank[u] < rank[g.edge_target[e as usize] as usize] {
                                arc | UP_BIT
                            } else {
                                arc
                            };
                    }
                }
            });
        (edge_arc, edge_blocks)
    }

    fn heap_bytes(&self) -> usize {
        4 * (self.first_down.len()
            + self.down_tail.len()
            + self.down_arc.len()
            + self.level_order.len()
            + self.level_start.len()
            + self.edge_arc.len()
            + self.edge_blocks.len())
    }
}

/// Source nodes per block of [`CustomizationArrays::edge_blocks`].
const SOURCE_BLOCK: usize = 4096;

/// [`CustomizationArrays::edge_blocks`] of `g`.
fn source_blocks(g: &Sections<'_>) -> Vec<u32> {
    (0..g.node_count.div_ceil(SOURCE_BLOCK))
        .map(|block| g.first_out[block * SOURCE_BLOCK])
        .chain(std::iter::once(g.edge_count as u32))
        .collect()
}

/// Levels of up to this many nodes are narrow (see
/// [`Customizer::customize`]; tests: few, so that the small test packages
/// have wide levels too).
const NARROW_LEVEL_NODES: usize = if cfg!(test) { 2 } else { 64 };
/// Above the wide levels, a node with more [`Customizer::triangle_work`]
/// than this splits its downward arcs among tasks of about
/// [`SPLIT_CHUNK_WORK`] each (tests: tiny, so that the small test packages
/// split too). On Malopolskie a unit is about 8 ns on one thread.
const SPLIT_WORK: usize = if cfg!(test) { 4 } else { 40_000 };
const SPLIT_CHUNK_WORK: usize = if cfg!(test) { 2 } else { 10_000 };

/// The narrow top of a customization ([`Customizer::relax_top`]).
struct Top<'t> {
    customizer: &'t Customizer<'t>,
    /// The nodes, and by node the index of its parent among them.
    ranks: &'t [u32],
    parent_index: Vec<Option<usize>>,
    /// By node: its children not yet relaxed.
    pending: Vec<std::sync::atomic::AtomicU32>,
    up_ptr: SyncPtr,
    down_ptr: SyncPtr,
}

impl<'t> Top<'t> {
    /// Relaxes node `at` (an index into `ranks`), then its parent once it was
    /// the last child left.
    fn run<'s>(&'s self, scope: &rayon::Scope<'s>, at: usize)
    where
        't: 's,
    {
        let y = self.ranks[at] as usize;
        let customizer = self.customizer;
        // SAFETY: every descendant of y (each of its downward neighbours is
        // one) is done: those among `ranks` released it, and the others were
        // relaxed before the top.
        unsafe {
            if customizer.triangle_work(y) > SPLIT_WORK {
                customizer.relax_node_split(y, self.up_ptr, self.down_ptr);
            } else {
                customizer.relax_node(y, self.up_ptr, self.down_ptr);
            }
        }
        if let Some(parent) = self.parent_index[at]
            && self.pending[parent].fetch_sub(1, std::sync::atomic::Ordering::AcqRel) == 1
        {
            scope.spawn(move |scope| self.run(scope, parent));
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

/// Per-direction query scratch, reset by epochs.
pub(crate) struct PotentialScratch {
    /// By rank: the node's entry in `entries`, valid only when that entry
    /// names the rank back (a sparse set: never cleared).
    slot: RankSlots,
    /// The nodes this query touched: rank, `B` (infinite unless an ancestor
    /// of a target, see the module docs) and the memoised potential (NaN
    /// until computed).
    entries: Vec<ScratchEntry>,
    ancestors: Vec<u32>,
    path: Vec<u32>,
    /// What `entries` were computed for, if anything: their values depend
    /// only on it, so a potential for the same key keeps them (matrix cells
    /// that share an endpoint).
    key: Option<MemoKey>,
}

/// A potential's metric (by id), direction and targets, with their initial
/// distances' bits.
#[derive(PartialEq)]
struct MemoKey {
    metric: u64,
    reversed: bool,
    targets: Vec<(u32, u64)>,
}

#[derive(Clone, Copy)]
struct ScratchEntry {
    rank: u32,
    b: f64,
    pot: f64,
}

/// Entries a scratch keeps its capacity for between queries.
const SCRATCH_KEEP: usize = 1 << 16;

/// A scratch's slots by rank. Potential evaluation looks a slot up for
/// every upward arc on a node's elimination-tree path, so the slots are its
/// hottest memory: dense (4 bytes per rank of the package, allocated by the
/// first query) they are a single load; paged they hold only the ranks
/// queries touched, at a few percent of a drive's time.
enum RankSlots {
    Dense(Vec<u32>),
    Paged(PagedSlots<u32>),
}

impl RankSlots {
    #[inline]
    fn get(&self, rank: u32) -> u32 {
        match self {
            Self::Dense(slots) => slots[rank as usize],
            Self::Paged(slots) => slots.get(rank),
        }
    }

    #[inline]
    fn set(&mut self, rank: u32, index: u32) {
        match self {
            Self::Dense(slots) => slots[rank as usize] = index,
            Self::Paged(slots) => slots.set(rank, index),
        }
    }

    fn ensure_len(&mut self, n: usize) {
        match self {
            Self::Dense(slots) => {
                if slots.len() < n {
                    slots.resize(n, u32::MAX);
                }
            }
            Self::Paged(slots) => slots.ensure_len(n),
        }
    }
}

impl Default for PotentialScratch {
    /// A scratch with dense slots.
    fn default() -> Self {
        Self::with_slots(RankSlots::Dense(Vec::new()))
    }
}

impl PotentialScratch {
    /// A scratch whose slots are allocated in pages as queries touch ranks.
    pub(crate) fn paged() -> Self {
        Self::with_slots(RankSlots::Paged(PagedSlots::new(0, u32::MAX)))
    }

    fn with_slots(slot: RankSlots) -> Self {
        Self {
            slot,
            entries: Vec::new(),
            ancestors: Vec::new(),
            path: Vec::new(),
            key: None,
        }
    }

    /// Bytes held by paged slots (dense slots are not counted: they stay for
    /// the scratch's life).
    pub(crate) fn page_bytes(&self) -> usize {
        match &self.slot {
            RankSlots::Dense(_) => 0,
            RankSlots::Paged(slots) => slots.page_bytes(),
        }
    }

    /// Frees paged slots (and with them the memo: `entries` can no longer
    /// be found by rank).
    pub(crate) fn release(&mut self) {
        if let RankSlots::Paged(slots) = &mut self.slot {
            slots.release();
            self.key = None;
        }
    }

    fn begin(&mut self, n: usize) {
        self.key = None;
        self.slot.ensure_len(n);
        if self.entries.capacity() > SCRATCH_KEEP {
            self.entries = Vec::new();
        } else {
            self.entries.clear();
        }
    }

    #[inline]
    fn find(&self, rank: u32) -> Option<usize> {
        let index = self.slot.get(rank) as usize;
        (index < self.entries.len() && self.entries[index].rank == rank).then_some(index)
    }

    /// The entry of `rank`, added (no `B`, no potential) if missing.
    #[inline]
    fn entry(&mut self, rank: u32) -> usize {
        if let Some(index) = self.find(rank) {
            return index;
        }
        let index = self.entries.len();
        self.slot.set(rank, index as u32);
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
        let key = MemoKey {
            metric: metric.id,
            reversed,
            targets: targets
                .iter()
                .map(|(node, initial)| (*node, initial.to_bits()))
                .collect(),
        };
        // The same potential as the scratch's last one (the factor only
        // scales what `get` returns): its memo still holds.
        if scratch.borrow().key.as_ref() != Some(&key) {
            potential.init(targets);
            scratch.borrow_mut().key = Some(key);
        }
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
            let bx = s.entries[s.slot.get(x) as usize].b;
            if !bx.is_finite() {
                continue;
            }
            let arcs = t.first_up[x as usize] as usize..t.first_up[x as usize + 1] as usize;
            for (y, w) in t.up_head[arcs.clone()].iter().zip(&toward[arcs]) {
                let value = bx + f64::from(*w);
                let b = &mut s.entries[s.slot.get(*y) as usize].b;
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
                        let value = f64::from(*w) + s.entries[s.slot.get(*y) as usize].pot;
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
        targets: &[(u32, f64)],
        reversed: bool,
        factor: f64,
        gates: &[PotentialGate],
        floor: f64,
        scratch: &'a RefCell<PotentialScratch>,
        gated_scratch: &'a RefCell<PotentialScratch>,
    ) -> Self {
        // Target offsets in request costs; seeds in metric units (`get`
        // scales them back), lowered by a hair against rounding.
        let scale = factor * POTENTIAL_MARGIN;
        let seeds = targets
            .iter()
            .filter(|(_, offset)| offset.is_finite())
            .map(|(node, offset)| {
                let offset = if *offset > 0.0 {
                    offset / scale * (1.0 - 1e-12)
                } else {
                    0.0
                };
                (*node, offset)
            })
            .collect::<Vec<_>>();
        let base = CchPotential::new(topology, metric, &seeds, reversed, factor, scratch);
        if gates.is_empty() {
            return Self {
                base,
                gated: None,
                floor: 0.0,
            };
        }
        // Gate offsets are scaled as the targets' are.
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
    /// Node-level distances from `source` to every node and from every node
    /// to `target`, by rank, in metric units, as `[from, to]` pairs (PHAST:
    /// the elimination-tree search up from each, then one sweep down the
    /// ranks for both trees, which reads every arc once).
    pub(crate) fn both_to_all(
        &self,
        metric: &CchMetric,
        source: u32,
        target: u32,
    ) -> Vec<[f32; 2]> {
        let n = self.rank.len();
        let mut distance = vec![[f32::INFINITY; 2]; n];
        let mut space = Vec::new();
        // Forward: up the tree along arcs low → high (`up`), then down along
        // high → low (`down`). Backward the other way round.
        elimination_search(self, source, &metric.up, &mut space);
        for (x, d) in &space {
            distance[*x as usize][0] = *d as f32;
        }
        elimination_search(self, target, &metric.down, &mut space);
        for (x, d) in &space {
            distance[*x as usize][1] = *d as f32;
        }
        // Every upward neighbour has a higher rank: one sweep down the
        // ranks, in memory order.
        for x in (0..n).rev() {
            let arcs = self.first_up[x] as usize..self.first_up[x + 1] as usize;
            let [mut from, mut to] = distance[x];
            for ((y, down), up) in self.up_head[arcs.clone()]
                .iter()
                .zip(&metric.down[arcs.clone()])
                .zip(&metric.up[arcs])
            {
                let [from_y, to_y] = distance[*y as usize];
                let value = from_y + *down;
                if value < from {
                    from = value;
                }
                let value = to_y + *up;
                if value < to {
                    to = value;
                }
            }
            distance[x] = [from, to];
        }
        distance
    }
}

/// Single-edge via candidates after the plateaus (see [`via_plateaus`]).
const VIA_FALLBACK_EDGES: usize = 24;

/// Ranks per task of the plateau scan.
const PLATEAU_SCAN_CHUNK: usize = 16_384;

/// A via edge for an alternative route: the middle of a plateau, a stretch
/// of edges on both the shortest-path tree from the origin and the one to
/// the destination (Bader et al.), so that the route through it is locally
/// optimal over the plateau's length.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ViaCandidate {
    pub edge: u32,
    /// Node-level cost of the route through the plateau (metric units).
    pub cost: f64,
    /// The plateau's length (metric units).
    pub plateau: f64,
}

/// The two node-level trees of [`via_plateaus`] (by rank: distance from the
/// source, distance to the target) and its cost bound, kept for
/// [`via_singles`].
pub(crate) struct ViaTrees {
    distance: Vec<[f32; 2]>,
    bound: f32,
}

/// Via candidates between nodes `source` and `target` on `metric` (customized
/// from per-edge `costs`), using none of the `excluded` edges, whose route
/// costs at most `max_ratio * D` (`D` the shortest distance): the plateaus
/// at least `min_plateau * D` long, longest first. For short detours a
/// plateau cannot catch (both trees may leave a short branch at its ends),
/// [`via_singles`] lists single edges from the returned trees, which are
/// `None` when the target cannot be reached.
#[allow(clippy::too_many_arguments)]
pub(crate) fn via_plateaus(
    g: &Sections<'_>,
    topology: &CchTopology,
    metric: &CchMetric,
    costs: &[f32],
    source: u32,
    target: u32,
    max_ratio: f64,
    min_plateau: f64,
    excluded: &rustc_hash::FxHashSet<u32>,
) -> (Vec<ViaCandidate>, Option<ViaTrees>) {
    let distance = topology.both_to_all(metric, source, target);
    let rank = &topology.rank;
    let total = f64::from(distance[rank[target as usize] as usize][0]);
    if !total.is_finite() || total <= 0.0 {
        return (Vec::new(), None);
    }
    let bound = (max_ratio * total) as f32;
    let plateau = plateau_edges(g, topology, costs, &distance, bound);
    let candidates = plateau_chains(topology, &plateau, &distance, total, min_plateau, excluded);
    (candidates, Some(ViaTrees { distance, bound }))
}

/// Plateau edges: (tail, edge, head) on both trees, at most one out of each
/// tail (its first), by the tail's rank.
fn plateau_edges(
    g: &Sections<'_>,
    topology: &CchTopology,
    costs: &[f32],
    distance: &[[f32; 2]],
    bound: f32,
) -> Vec<(u32, u32, u32)> {
    let rank = &topology.rank;
    // f32 sums along the sweeps: equal within a few ulps of the distance.
    let tolerance = |value: f32| 1e-5 * value.max(1.0) + 1e-5;
    let inside = |from: f32, to: f32| from + to <= bound;
    let n = g.node_count;
    (0..n.div_ceil(PLATEAU_SCAN_CHUNK))
        .into_par_iter()
        .map(|chunk| {
            let ranks = chunk * PLATEAU_SCAN_CHUNK..((chunk + 1) * PLATEAU_SCAN_CHUNK).min(n);
            let mut found = Vec::new();
            for (ru, [from_u, to_u]) in ranks.clone().zip(&distance[ranks]) {
                if !inside(*from_u, *to_u) {
                    continue;
                }
                let u = topology.node[ru];
                for e in g.first_out[u as usize]..g.first_out[u as usize + 1] {
                    let w = g.edge_target[e as usize];
                    let c = costs[e as usize];
                    if w == u || !c.is_finite() {
                        continue;
                    }
                    let [from_w, to_w] = distance[rank[w as usize] as usize];
                    if !inside(from_w, to_w) {
                        continue;
                    }
                    let on_from = (from_u + c - from_w).abs() <= tolerance(from_w);
                    let on_to = (to_w + c - to_u).abs() <= tolerance(*to_u);
                    if on_from && on_to {
                        found.push((u, e, w));
                        break;
                    }
                }
            }
            found
        })
        .collect::<Vec<_>>()
        .concat()
}

/// The plateaus among `plateau`'s edges (chains of them, each started at a
/// tail no plateau edge enters, in `plateau`'s order) at least `min_plateau
/// * total` long and on none of the `excluded` edges, longest first.
fn plateau_chains(
    topology: &CchTopology,
    plateau: &[(u32, u32, u32)],
    distance: &[[f32; 2]],
    total: f64,
    min_plateau: f64,
    excluded: &rustc_hash::FxHashSet<u32>,
) -> Vec<ViaCandidate> {
    let rank = &topology.rank;
    let n = rank.len();
    // By the tail's rank: its plateau edge's index in `plateau`, plus one
    // (0: none); a tail has at most one.
    let mut next = vec![0u32; n];
    let mut heads = Bits::new(n);
    for (index, (u, _, w)) in plateau.iter().enumerate() {
        next[rank[*u as usize] as usize] = index as u32 + 1;
        heads.insert(rank[*w as usize] as usize);
    }
    let from = |r: usize| distance[r][0];
    let mut seen = Bits::new(n);
    let mut candidates = Vec::new();
    let mut chain = Vec::new();
    for (start, _, _) in plateau {
        let rs = rank[*start as usize] as usize;
        if heads.contains(rs) || !seen.insert(rs) {
            continue;
        }
        // (edge, rank of its head) along the chain.
        chain.clear();
        let mut last = rs;
        while next[last] != 0 {
            let (_, edge, head) = plateau[next[last] as usize - 1];
            let rh = rank[head as usize] as usize;
            chain.push((edge, rh));
            if !seen.insert(rh) {
                break;
            }
            last = rh;
        }
        if chain.iter().any(|(edge, _)| excluded.contains(edge)) {
            continue;
        }
        let length = f64::from(from(last) - from(rs));
        if length < min_plateau * total {
            continue;
        }
        let middle = f64::from(from(rs)) + length / 2.0;
        let edge = chain
            .iter()
            .find(|(_, rh)| f64::from(from(*rh)) >= middle)
            .map_or(chain[0].0, |(edge, _)| *edge);
        candidates.push(ViaCandidate {
            edge,
            cost: f64::from(distance[rs][0] + distance[rs][1]),
            plateau: length,
        });
    }
    candidates.sort_by(|a, b| {
        b.plateau
            .total_cmp(&a.plateau)
            .then(a.cost.total_cmp(&b.cost))
            .then(a.edge.cmp(&b.edge))
    });
    candidates
}

/// A fixed-size bit set.
struct Bits(Vec<u64>);

impl Bits {
    fn new(len: usize) -> Self {
        Self(vec![0; len.div_ceil(64)])
    }

    fn contains(&self, index: usize) -> bool {
        self.0[index >> 6] >> (index & 63) & 1 != 0
    }

    /// Whether `index` was not in the set yet.
    fn insert(&mut self, index: usize) -> bool {
        let fresh = !self.contains(index);
        self.0[index >> 6] |= 1 << (index & 63);
        fresh
    }
}

/// The single-edge via candidates that follow [`via_plateaus`]' plateaus: up
/// to [`VIA_FALLBACK_EDGES`] edges on none of the `excluded` edges, by the
/// cost of the cheapest route through them within the bound (plateau 0),
/// then by edge.
pub(crate) fn via_singles(
    g: &Sections<'_>,
    topology: &CchTopology,
    costs: &[f32],
    trees: &ViaTrees,
    excluded: &rustc_hash::FxHashSet<u32>,
) -> Vec<ViaCandidate> {
    let (distance, bound, rank) = (&trees.distance, trees.bound, &topology.rank);
    let mut singles = (0..g.node_count)
        .into_par_iter()
        .filter(|r| distance[*r][0] + distance[*r][1] <= bound)
        .flat_map_iter(|ru| {
            let u = topology.node[ru];
            (g.first_out[u as usize]..g.first_out[u as usize + 1]).filter_map(move |e| {
                let w = g.edge_target[e as usize];
                let c = costs[e as usize];
                if w == u || !c.is_finite() || excluded.contains(&e) {
                    return None;
                }
                let cost = distance[ru][0] + c + distance[rank[w as usize] as usize][1];
                (cost <= bound).then_some((cost, e))
            })
        })
        .collect::<Vec<_>>();
    let order = |a: &(f32, u32), b: &(f32, u32)| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1));
    let keep = VIA_FALLBACK_EDGES.min(singles.len());
    if keep > 0 && keep < singles.len() {
        singles.select_nth_unstable_by(keep - 1, order);
        singles.truncate(keep);
    }
    singles.sort_unstable_by(order);
    singles
        .into_iter()
        .map(|(cost, edge)| ViaCandidate {
            edge,
            cost: f64::from(cost),
            plateau: 0.0,
        })
        .collect()
}

// The via candidates as they were first computed (two separate sweeps, the
// plateau chains in hash maps, the single edges always listed): the
// reference the tests compare `via_plateaus` and `via_singles` with.

#[cfg(test)]
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

/// The plateaus of [`via_plateaus`] followed by every candidate of
/// [`via_singles`].
#[cfg(test)]
#[allow(clippy::too_many_arguments)]
pub(crate) fn via_candidates_reference(
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
    for column in 0..potentials.stored_column_count() {
        let cost = &g.edge_cost[column * g.edge_count..(column + 1) * g.edge_count];
        let metric = potentials
            .stored_metric(g, column)
            .expect("a stored column");
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

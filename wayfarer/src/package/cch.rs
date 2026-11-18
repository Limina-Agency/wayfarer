//! CCH lower-bound potentials ("CH-potentials", Strasser and Zeitz) for the
//! edge-based A* of `package::search`.
//!
//! Opt-in and built from an open package, with no format change
//! ([`RoutingPackage::prepare_potentials`](super::RoutingPackage::prepare_potentials)):
//!
//! 1. **Order.** The node-level graph (one undirected arc per node pair any
//!    edge joins) loses its nodes of degree ≤ 2 first, iteratively (chains
//!    and trees, each eliminated node adds at most one arc); the core left is
//!    ordered by nested dissection with inertial flow (4 directions, 25 %
//!    sources and sinks, unit-capacity max flow by Dinic, node separator from
//!    the smaller side of the cut edges' endpoints), down to cells of
//!    [`LEAF_CELL`] nodes ordered by minimum degree. Ties break on node ids,
//!    so the order is deterministic on one platform. It uses floating-point
//!    projections, so another platform may order differently: that changes
//!    how many edges a search visits, never a route's cost.
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
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, OnceLock};
use std::time::Instant;

use rayon::prelude::*;

use super::Sections;

const NO_NODE: u32 = u32::MAX;
/// Marks an edge's arc as lower → higher rank in [`CchTopology::edge_arc`].
const UP_BIT: u32 = 1 << 31;
/// Cells of at most this many nodes are ordered by minimum degree (bitsets).
const LEAF_CELL: usize = 64;
/// Safety factor on potentials against f64 rounding in path sums (a
/// consistent potential scaled by a factor below 1 stays consistent).
const POTENTIAL_MARGIN: f64 = 1.0 - 1e-9;

/// What [`RoutingPackage::prepare_potentials`](super::RoutingPackage::prepare_potentials)
/// built, and how long it took.
#[derive(Debug, Clone, Default, PartialEq)]
#[non_exhaustive]
pub struct PotentialStats {
    pub node_count: usize,
    /// Nodes eliminated before the nested dissection (degree ≤ 2).
    pub degree2_eliminated: usize,
    /// The largest separator's node count.
    pub top_separator: usize,
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
}

/// A package's potentials, once prepared.
#[derive(Default)]
pub(crate) struct CchCache {
    ready: OnceLock<Option<Arc<CchPotentials>>>,
}

impl CchCache {
    /// The potentials, if prepared (and they fit: see [`CchTopology::build`]).
    pub(crate) fn get(&self) -> Option<&Arc<CchPotentials>> {
        self.ready.get().and_then(Option::as_ref)
    }

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
        })
    }
}

// -------------------------------------------------------------------------
// Order
// -------------------------------------------------------------------------

/// Undirected simple node graph in CSR form (both directions listed).
struct NodeGraph {
    first: Vec<u32>,
    head: Vec<u32>,
}

impl NodeGraph {
    fn from_sections(g: &Sections<'_>) -> Self {
        let n = g.node_count;
        let mut pairs = Vec::with_capacity(g.edge_count);
        for u in 0..n {
            for e in g.first_out[u]..g.first_out[u + 1] {
                let v = g.edge_target[e as usize];
                if v as usize != u {
                    let (a, b) = if (u as u32) < v {
                        (u as u32, v)
                    } else {
                        (v, u as u32)
                    };
                    pairs.push((a, b));
                }
            }
        }
        pairs.par_sort_unstable();
        pairs.dedup();
        let mut degree = vec![0u32; n + 1];
        for (a, b) in &pairs {
            degree[*a as usize] += 1;
            degree[*b as usize] += 1;
        }
        let mut first = vec![0u32; n + 1];
        for i in 0..n {
            first[i + 1] = first[i] + degree[i];
        }
        let mut fill = first.clone();
        let mut head = vec![0u32; first[n] as usize];
        for (a, b) in &pairs {
            head[fill[*a as usize] as usize] = *b;
            fill[*a as usize] += 1;
            head[fill[*b as usize] as usize] = *a;
            fill[*b as usize] += 1;
        }
        // Sorted neighbour lists.
        for i in 0..n {
            head[first[i] as usize..first[i + 1] as usize].sort_unstable();
        }
        Self { first, head }
    }

    fn neighbours(&self, node: u32) -> &[u32] {
        &self.head[self.first[node as usize] as usize..self.first[node as usize + 1] as usize]
    }
}

/// Shared state of the nested dissection.
struct Dissection<'a> {
    /// Core adjacency (after degree-2 elimination), sorted per node.
    adjacency: &'a [Vec<u32>],
    x: &'a [f64],
    y: &'a [f64],
    /// Per node: the cell it is in now, and its index there.
    cell_of: Vec<AtomicU32>,
    local: Vec<AtomicU32>,
    next_cell: AtomicU32,
    top_separator: AtomicU32,
}

/// A cell's induced graph in local indexes, with twin arcs for unit flow.
struct LocalGraph {
    first: Vec<u32>,
    head: Vec<u32>,
    twin: Vec<u32>,
}

impl Dissection<'_> {
    /// Assigns a fresh cell id to `nodes` and their local indexes.
    fn claim(&self, nodes: &[u32]) -> u32 {
        let id = self.next_cell.fetch_add(1, Ordering::Relaxed);
        for (index, node) in nodes.iter().enumerate() {
            self.cell_of[*node as usize].store(id, Ordering::Relaxed);
            self.local[*node as usize].store(index as u32, Ordering::Relaxed);
        }
        id
    }

    fn local_graph(&self, nodes: &[u32], id: u32) -> LocalGraph {
        let mut first = Vec::with_capacity(nodes.len() + 1);
        let mut head = Vec::new();
        first.push(0u32);
        for node in nodes {
            for w in &self.adjacency[*node as usize] {
                if self.cell_of[*w as usize].load(Ordering::Relaxed) == id {
                    head.push(self.local[*w as usize].load(Ordering::Relaxed));
                }
            }
            let start = *first.last().unwrap() as usize;
            head[start..].sort_unstable();
            first.push(head.len() as u32);
        }
        let mut twin = vec![0u32; head.len()];
        for u in 0..nodes.len() {
            for a in first[u]..first[u + 1] {
                let v = head[a as usize] as usize;
                let list = &head[first[v] as usize..first[v + 1] as usize];
                let at = list
                    .binary_search(&(u as u32))
                    .expect("undirected adjacency");
                twin[a as usize] = first[v] + at as u32;
            }
        }
        LocalGraph { first, head, twin }
    }

    /// Connected components of `nodes` (one cell, local graph `lg`), less
    /// the nodes with `removed` set, as lists of global nodes in ascending
    /// local order of their first node.
    fn components(&self, nodes: &[u32], lg: &LocalGraph, removed: &[bool]) -> Vec<Vec<u32>> {
        let n = nodes.len();
        let mut seen = removed.to_vec();
        let mut parts = Vec::new();
        let mut queue = Vec::new();
        for start in 0..n {
            if seen[start] {
                continue;
            }
            seen[start] = true;
            queue.clear();
            queue.push(start as u32);
            let mut cursor = 0;
            while cursor < queue.len() {
                let u = queue[cursor] as usize;
                cursor += 1;
                for a in lg.first[u]..lg.first[u + 1] {
                    let v = lg.head[a as usize] as usize;
                    if !seen[v] {
                        seen[v] = true;
                        queue.push(v as u32);
                    }
                }
            }
            let mut part = queue.iter().map(|u| nodes[*u as usize]).collect::<Vec<_>>();
            part.sort_unstable();
            parts.push(part);
        }
        parts
    }

    /// The order (lowest rank first) of a cell.
    fn order(&self, nodes: Vec<u32>, depth: usize) -> Vec<u32> {
        if nodes.len() <= LEAF_CELL {
            let id = self.claim(&nodes);
            let lg = self.local_graph(&nodes, id);
            return min_degree_order(&nodes, &lg);
        }
        let id = self.claim(&nodes);
        let lg = self.local_graph(&nodes, id);
        let parts = self.components(&nodes, &lg, &vec![false; nodes.len()]);
        if parts.len() > 1 {
            drop(lg);
            // Components are cells of their own within the parent's.
            let orders = parts
                .into_par_iter()
                .map(|part| self.order(part, depth))
                .collect::<Vec<_>>();
            return orders.concat();
        }
        let separator = self.separator(&nodes, &lg);
        if depth == 0 {
            self.top_separator
                .store(separator.len() as u32, Ordering::Relaxed);
        }
        let mut removed = vec![false; nodes.len()];
        for local in &separator {
            removed[*local as usize] = true;
        }
        let parts = self.components(&nodes, &lg, &removed);
        let separator_nodes = separator
            .iter()
            .map(|local| nodes[*local as usize])
            .collect::<Vec<_>>();
        drop(lg);
        drop(nodes);
        let orders = parts
            .into_par_iter()
            .map(|part| self.order(part, depth + 1))
            .collect::<Vec<_>>();
        let mut order = orders.concat();
        order.extend(separator_nodes);
        order
    }

    /// A node separator of a connected cell (local indexes): the best of
    /// four inertial-flow cuts.
    fn separator(&self, nodes: &[u32], lg: &LocalGraph) -> Vec<u32> {
        const DIRECTIONS: [(f64, f64); 4] = [(1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, -1.0)];
        let candidates = DIRECTIONS
            .par_iter()
            .map(|(dx, dy)| {
                let mut by_projection = (0..nodes.len() as u32).collect::<Vec<_>>();
                let key = |local: u32| {
                    let node = nodes[local as usize] as usize;
                    self.x[node] * dx + self.y[node] * dy
                };
                by_projection.sort_unstable_by(|a, b| {
                    key(*a)
                        .total_cmp(&key(*b))
                        .then(nodes[*a as usize].cmp(&nodes[*b as usize]))
                });
                let k = (nodes.len() / 4).max(1);
                let mut side = vec![0u8; nodes.len()];
                for local in &by_projection[..k] {
                    side[*local as usize] = 1;
                }
                for local in &by_projection[nodes.len() - k..] {
                    side[*local as usize] = 2;
                }
                inertial_cut(lg, &side)
            })
            .collect::<Vec<_>>();
        candidates
            .into_iter()
            .min_by_key(|separator| separator.len())
            .unwrap_or_default()
    }
}

/// Unit-capacity max flow (Dinic) from the nodes with `side` 1 to those with
/// 2, then the node separator: the smaller endpoint side of the cut edges.
fn inertial_cut(lg: &LocalGraph, side: &[u8]) -> Vec<u32> {
    let n = side.len();
    let arcs = lg.head.len();
    // Flow on each arc in its own direction: -1, 0 or 1 (twin = -flow).
    let mut flow = vec![0i8; arcs];
    let mut level = vec![u32::MAX; n];
    let mut it = vec![0u32; n];
    let mut queue = Vec::with_capacity(n);
    let sources = (0..n as u32)
        .filter(|u| side[*u as usize] == 1)
        .collect::<Vec<_>>();
    let mut path_arcs: Vec<u32> = Vec::new();
    let mut path_nodes: Vec<u32> = Vec::new();
    loop {
        // BFS level graph from all sources over residual arcs.
        level.fill(u32::MAX);
        queue.clear();
        for s in &sources {
            level[*s as usize] = 0;
            queue.push(*s);
        }
        let mut cursor = 0;
        let mut sink_level = u32::MAX;
        while cursor < queue.len() {
            let u = queue[cursor] as usize;
            cursor += 1;
            if level[u] >= sink_level {
                break;
            }
            if side[u] == 2 {
                continue;
            }
            for a in lg.first[u]..lg.first[u + 1] {
                let v = lg.head[a as usize] as usize;
                if flow[a as usize] < 1 && level[v] == u32::MAX {
                    level[v] = level[u] + 1;
                    if side[v] == 2 {
                        sink_level = sink_level.min(level[v]);
                    }
                    queue.push(v as u32);
                }
            }
        }
        if sink_level == u32::MAX {
            break;
        }
        // Blocking flow by iterative DFS with current-arc pointers.
        it.copy_from_slice(&lg.first[..n]);
        for s in &sources {
            'restart: loop {
                path_arcs.clear();
                path_nodes.clear();
                let mut u = *s as usize;
                loop {
                    if side[u] == 2 {
                        for a in &path_arcs {
                            flow[*a as usize] += 1;
                            flow[lg.twin[*a as usize] as usize] -= 1;
                        }
                        continue 'restart;
                    }
                    let mut advanced = false;
                    while it[u] < lg.first[u + 1] {
                        let a = it[u] as usize;
                        let v = lg.head[a] as usize;
                        if flow[a] < 1
                            && level[v] != u32::MAX
                            && level[v] == level[u] + 1
                            && side[v] != 1
                        {
                            path_arcs.push(a as u32);
                            path_nodes.push(u as u32);
                            u = v;
                            advanced = true;
                            break;
                        }
                        it[u] += 1;
                    }
                    if !advanced {
                        // Dead end: retreat.
                        level[u] = u32::MAX;
                        match path_nodes.pop() {
                            None => break 'restart,
                            Some(previous) => {
                                path_arcs.pop();
                                u = previous as usize;
                                it[u] += 1;
                            }
                        }
                    }
                }
            }
        }
    }
    // Source side: reachable from the sources in the residual graph.
    let mut reach = vec![false; n];
    queue.clear();
    for s in &sources {
        reach[*s as usize] = true;
        queue.push(*s);
    }
    let mut cursor = 0;
    while cursor < queue.len() {
        let u = queue[cursor] as usize;
        cursor += 1;
        for a in lg.first[u]..lg.first[u + 1] {
            let v = lg.head[a as usize] as usize;
            if !reach[v] && (flow[a as usize] < 1 || side[u] == 1 && side[v] == 1) {
                reach[v] = true;
                queue.push(v as u32);
            }
        }
    }
    let (mut inner, mut outer) = (Vec::new(), Vec::new());
    for u in 0..n {
        let crosses =
            (lg.first[u]..lg.first[u + 1]).any(|a| reach[u] != reach[lg.head[a as usize] as usize]);
        if crosses {
            if reach[u] {
                inner.push(u as u32);
            } else {
                outer.push(u as u32);
            }
        }
    }
    if inner.len() <= outer.len() {
        inner
    } else {
        outer
    }
}

/// Minimum-degree order of a small cell (≤ 64 nodes) with fill, on bitsets.
fn min_degree_order(nodes: &[u32], lg: &LocalGraph) -> Vec<u32> {
    let n = nodes.len();
    debug_assert!(n <= 64);
    let mut adjacency = vec![0u64; n];
    for (u, bits) in adjacency.iter_mut().enumerate() {
        for a in lg.first[u]..lg.first[u + 1] {
            *bits |= 1u64 << lg.head[a as usize];
        }
    }
    let mut remaining: u64 = if n == 64 { u64::MAX } else { (1u64 << n) - 1 };
    let mut order = Vec::with_capacity(n);
    while remaining != 0 {
        let mut best = usize::MAX;
        let mut best_degree = u32::MAX;
        let mut rest = remaining;
        while rest != 0 {
            let u = rest.trailing_zeros() as usize;
            rest &= rest - 1;
            let degree = (adjacency[u] & remaining).count_ones();
            if degree < best_degree {
                best_degree = degree;
                best = u;
            }
        }
        let neighbours = adjacency[best] & remaining & !(1u64 << best);
        let mut rest = neighbours;
        while rest != 0 {
            let w = rest.trailing_zeros() as usize;
            rest &= rest - 1;
            adjacency[w] |= neighbours & !(1u64 << w);
        }
        remaining &= !(1u64 << best);
        order.push(nodes[best]);
    }
    order
}

/// Planar coordinates for inertial flow: each node's unit vector projected on
/// east and north at the graph's mean position (mul/add only).
fn coordinates(g: &Sections<'_>) -> (Vec<f64>, Vec<f64>) {
    let n = g.node_count;
    let mut mean = [0.0f64; 3];
    for node in 0..n as u32 {
        let p = g.xyz(node);
        mean[0] += p[0];
        mean[1] += p[1];
        mean[2] += p[2];
    }
    let norm = (mean[0] * mean[0] + mean[1] * mean[1] + mean[2] * mean[2]).sqrt();
    let c = if norm > 0.0 {
        [mean[0] / norm, mean[1] / norm, mean[2] / norm]
    } else {
        [1.0, 0.0, 0.0]
    };
    // east = z × c, north = c × east
    let mut east = [-c[1], c[0], 0.0];
    let en = (east[0] * east[0] + east[1] * east[1]).sqrt();
    if en > 0.0 {
        east = [east[0] / en, east[1] / en, 0.0];
    } else {
        east = [0.0, 1.0, 0.0];
    }
    let north = [
        c[1] * east[2] - c[2] * east[1],
        c[2] * east[0] - c[0] * east[2],
        c[0] * east[1] - c[1] * east[0],
    ];
    (0..n as u32)
        .map(|node| {
            let p = g.xyz(node);
            (
                p[0] * east[0] + p[1] * east[1] + p[2] * east[2],
                p[0] * north[0] + p[1] * north[1] + p[2] * north[2],
            )
        })
        .unzip()
}

/// The elimination order (rank → node) and some statistics.
fn elimination_order(g: &Sections<'_>, graph: &NodeGraph, stats: &mut PotentialStats) -> Vec<u32> {
    let n = g.node_count;
    // Degree ≤ 2 nodes first, iteratively.
    let mut adjacency = (0..n as u32)
        .map(|node| graph.neighbours(node).to_vec())
        .collect::<Vec<_>>();
    let mut eliminated = vec![false; n];
    let mut order = Vec::with_capacity(n);
    let mut stack = (0..n as u32)
        .rev()
        .filter(|node| adjacency[*node as usize].len() <= 2)
        .collect::<Vec<_>>();
    while let Some(v) = stack.pop() {
        let vi = v as usize;
        if eliminated[vi] || adjacency[vi].len() > 2 {
            continue;
        }
        eliminated[vi] = true;
        order.push(v);
        let neighbours = std::mem::take(&mut adjacency[vi]);
        for a in &neighbours {
            let list = &mut adjacency[*a as usize];
            if let Ok(at) = list.binary_search(&v) {
                list.remove(at);
            }
        }
        if let [a, b] = neighbours[..] {
            let list = &mut adjacency[a as usize];
            if let Err(at) = list.binary_search(&b) {
                list.insert(at, b);
                let list = &mut adjacency[b as usize];
                let at = list.binary_search(&a).unwrap_err();
                list.insert(at, a);
            }
        }
        for a in neighbours {
            if !eliminated[a as usize] && adjacency[a as usize].len() <= 2 {
                stack.push(a);
            }
        }
    }
    stats.degree2_eliminated = order.len();
    let core = (0..n as u32)
        .filter(|node| !eliminated[*node as usize])
        .collect::<Vec<_>>();
    let (x, y) = coordinates(g);
    let dissection = Dissection {
        adjacency: &adjacency,
        x: &x,
        y: &y,
        cell_of: (0..n).map(|_| AtomicU32::new(NO_NODE)).collect(),
        local: (0..n).map(|_| AtomicU32::new(0)).collect(),
        next_cell: AtomicU32::new(0),
        top_separator: AtomicU32::new(0),
    };
    let core_order = dissection.order(core, 0);
    stats.top_separator = dissection.top_separator.load(Ordering::Relaxed) as usize;
    order.extend(core_order);
    debug_assert_eq!(order.len(), n);
    order
}

// -------------------------------------------------------------------------
// Topology
// -------------------------------------------------------------------------

impl CchTopology {
    /// The order and topology of `g`'s node graph; `None` when the arcs
    /// outgrow the 31-bit arc ids (far beyond a regional package).
    pub(crate) fn build(g: &Sections<'_>) -> Option<(Self, PotentialStats)> {
        let mut stats = PotentialStats {
            node_count: g.node_count,
            ..Default::default()
        };
        let started = Instant::now();
        let graph = NodeGraph::from_sections(g);
        let order = elimination_order(g, &graph, &mut stats);
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
        let topology = Self {
            rank,
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
    b: Vec<f64>,
    b_epoch: Vec<u32>,
    pot: Vec<f64>,
    pot_epoch: Vec<u32>,
    epoch: u32,
    ancestors: Vec<u32>,
    path: Vec<u32>,
}

impl PotentialScratch {
    fn begin(&mut self, n: usize) {
        if self.b.len() != n {
            self.b = vec![f64::INFINITY; n];
            self.b_epoch = vec![0; n];
            self.pot = vec![f64::INFINITY; n];
            self.pot_epoch = vec![0; n];
            self.epoch = 0;
        }
        self.epoch = self.epoch.wrapping_add(1);
        if self.epoch == 0 {
            self.b_epoch.fill(0);
            self.pot_epoch.fill(0);
            self.epoch = 1;
        }
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
        let epoch = s.epoch;
        s.ancestors.clear();
        for (node, initial) in targets {
            let r = t.rank[*node as usize] as usize;
            if s.b_epoch[r] != epoch {
                s.b_epoch[r] = epoch;
                s.b[r] = *initial;
                s.ancestors.push(r as u32);
                let mut x = t.parent[r];
                while x != NO_NODE && s.b_epoch[x as usize] != epoch {
                    s.b_epoch[x as usize] = epoch;
                    s.b[x as usize] = f64::INFINITY;
                    s.ancestors.push(x);
                    x = t.parent[x as usize];
                }
            } else {
                s.b[r] = s.b[r].min(*initial);
            }
        }
        s.ancestors.sort_unstable();
        // B(y) = min over downward arcs (y, x) of w(y -> x) + B(x) (forward;
        // reversed: w(x -> y)).
        let toward = if self.reversed { self.up } else { self.down };
        for i in 0..s.ancestors.len() {
            let x = s.ancestors[i] as usize;
            let bx = s.b[x];
            if !bx.is_finite() {
                continue;
            }
            let arcs = t.first_up[x] as usize..t.first_up[x + 1] as usize;
            for (y, w) in t.up_head[arcs.clone()].iter().zip(&toward[arcs]) {
                let value = bx + f64::from(*w);
                if value < s.b[*y as usize] {
                    s.b[*y as usize] = value;
                }
            }
        }
    }

    /// The potential at package node `node` (infinite: no target reachable).
    #[inline]
    pub(crate) fn get(&self, node: u32) -> f64 {
        let t = self.topology;
        let r = t.rank[node as usize];
        let mut s = self.scratch.borrow_mut();
        let s = &mut *s;
        let epoch = s.epoch;
        if s.pot_epoch[r as usize] != epoch {
            s.path.clear();
            let mut x = r;
            while x != NO_NODE && s.pot_epoch[x as usize] != epoch {
                s.path.push(x);
                x = t.parent[x as usize];
            }
            // pot(v) = min(B(v), min over upward arcs (v, y) of w(v -> y) +
            // pot(y)) (forward; reversed: w(y -> v)).
            let away = if self.reversed { self.down } else { self.up };
            for i in (0..s.path.len()).rev() {
                let x = s.path[i] as usize;
                let mut best = if s.b_epoch[x] == epoch {
                    s.b[x]
                } else {
                    f64::INFINITY
                };
                let arcs = t.first_up[x] as usize..t.first_up[x + 1] as usize;
                for (y, w) in t.up_head[arcs.clone()].iter().zip(&away[arcs]) {
                    let value = f64::from(*w) + s.pot[*y as usize];
                    if value < best {
                        best = value;
                    }
                }
                s.pot[x] = best;
                s.pot_epoch[x] = epoch;
            }
        }
        let value = s.pot[r as usize];
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

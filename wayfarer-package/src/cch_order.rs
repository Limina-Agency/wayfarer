//! The node order of CCH potentials (see the engine's `package::cch`): the
//! node-level graph loses its nodes of degree ≤ 2 first, iteratively (chains
//! and trees, each eliminated node adds at most one arc); the core left is
//! ordered by nested dissection with inertial flow (4 directions, 25 %
//! sources and sinks, unit-capacity max flow, node separator from
//! the smaller side of the cut edges' endpoints), down to cells of
//! 64 nodes ordered by minimum degree. Ties break on node ids and
//! coordinates are integers, so the order is the same on every platform: the
//! compiler stores it (`Section::CchRank`).
//!
//! The separator depends only on the cell and its sources and sinks, not on
//! how the max flow is found: the nodes the sources reach in the residual
//! graph of *any* maximum flow are the same set (the minimal min cut). So
//! the flow runs on the cell with its source and sink blocks contracted to
//! one node each, by shortest augmenting paths with distance labels (ISAP),
//! which repairs labels locally instead of searching the whole cell once
//! per augmenting path as Dinic does on these graphs (one path per phase).

use std::sync::atomic::{AtomicU32, Ordering};

use rayon::prelude::*;

const NO_NODE: u32 = u32::MAX;
/// Cells of at most this many nodes are ordered by minimum degree (bitsets).
const LEAF_CELL: usize = 64;
/// Cells of at least this many nodes build their local graph in parallel.
const PARALLEL_CELL: usize = 50_000;

/// Undirected simple node graph in CSR form (both directions listed).
pub struct NodeGraph {
    first: Vec<u32>,
    head: Vec<u32>,
}

impl NodeGraph {
    /// The node graph of a directed CSR graph (`first_out`, `edge_target`):
    /// one undirected arc per node pair an edge joins, loops left out.
    pub fn new(first_out: &[u32], edge_target: &[u32]) -> Self {
        let n = first_out.len().saturating_sub(1);
        let mut pairs = Vec::with_capacity(edge_target.len());
        for u in 0..n {
            for e in first_out[u]..first_out[u + 1] {
                let v = edge_target[e as usize];
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

    /// Undirected arcs (each counted once).
    pub fn arc_count(&self) -> usize {
        self.head.len() / 2
    }

    /// `node`'s neighbours, ascending.
    pub fn neighbours(&self, node: u32) -> &[u32] {
        &self.head[self.first[node as usize] as usize..self.first[node as usize + 1] as usize]
    }
}

/// Shared state of the nested dissection.
struct Dissection<'a> {
    /// Core adjacency (after degree-2 elimination), sorted per node.
    adjacency: &'a NodeGraph,
    x: &'a [i64],
    y: &'a [i64],
    /// Per node: the cell it is in now, and its index there.
    cell_of: Vec<AtomicU32>,
    local: Vec<AtomicU32>,
    next_cell: AtomicU32,
    top_separator: AtomicU32,
}

/// A cell's induced graph in local indexes.
struct LocalGraph {
    first: Vec<u32>,
    head: Vec<u32>,
}

impl Dissection<'_> {
    /// Assigns a fresh cell id to `nodes` and their local indexes.
    fn claim(&self, nodes: &[u32]) -> u32 {
        let id = self.next_cell.fetch_add(1, Ordering::Relaxed);
        let store = |(index, node): (usize, &u32)| {
            self.cell_of[*node as usize].store(id, Ordering::Relaxed);
            self.local[*node as usize].store(index as u32, Ordering::Relaxed);
        };
        if nodes.len() >= PARALLEL_CELL {
            nodes.par_iter().enumerate().for_each(store);
        } else {
            nodes.iter().enumerate().for_each(store);
        }
        id
    }

    fn local_graph(&self, nodes: &[u32], id: u32) -> LocalGraph {
        if nodes.len() >= PARALLEL_CELL {
            return self.local_graph_parallel(nodes, id);
        }
        let mut first = Vec::with_capacity(nodes.len() + 1);
        let mut head = Vec::new();
        first.push(0u32);
        for node in nodes {
            for w in self.adjacency.neighbours(*node) {
                if self.cell_of[*w as usize].load(Ordering::Relaxed) == id {
                    head.push(self.local[*w as usize].load(Ordering::Relaxed));
                }
            }
            let start = *first.last().unwrap() as usize;
            head[start..].sort_unstable();
            first.push(head.len() as u32);
        }
        LocalGraph { first, head }
    }

    /// [`Self::local_graph`] for a big cell, built in parallel (the same
    /// arrays).
    fn local_graph_parallel(&self, nodes: &[u32], id: u32) -> LocalGraph {
        let inside = |w: &u32| self.cell_of[*w as usize].load(Ordering::Relaxed) == id;
        let mut first = Vec::with_capacity(nodes.len() + 1);
        first.push(0u32);
        let degrees = nodes
            .par_iter()
            .map(|node| {
                self.adjacency
                    .neighbours(*node)
                    .iter()
                    .filter(|w| inside(w))
                    .count() as u32
            })
            .collect::<Vec<_>>();
        for degree in degrees {
            first.push(first.last().unwrap() + degree);
        }
        let mut head = vec![0u32; *first.last().unwrap() as usize];
        split_by(&mut head, &first)
            .into_par_iter()
            .zip(nodes.par_iter())
            .for_each(|(list, node)| {
                let locals = self
                    .adjacency
                    .neighbours(*node)
                    .iter()
                    .filter(|w| inside(w));
                for (slot, w) in list.iter_mut().zip(locals) {
                    *slot = self.local[*w as usize].load(Ordering::Relaxed);
                }
                list.sort_unstable();
            });
        LocalGraph { first, head }
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
            // One top-level cut per connected component of the core.
            self.top_separator
                .fetch_max(separator.len() as u32, Ordering::Relaxed);
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
        const DIRECTIONS: [(i128, i128); 4] = [(1, 0), (0, 1), (1, 1), (1, -1)];
        let candidates = DIRECTIONS
            .par_iter()
            .map(|(dx, dy)| {
                // Only which nodes are the lowest and highest quarter
                // matters: two selections on (projection, node) give the
                // same sets as a full sort.
                let mut keyed = nodes
                    .iter()
                    .enumerate()
                    .map(|(local, node)| {
                        let at = *node as usize;
                        let key = i128::from(self.x[at]) * dx + i128::from(self.y[at]) * dy;
                        (key, *node, local as u32)
                    })
                    .collect::<Vec<_>>();
                let k = (nodes.len() / 4).max(1);
                let mut side = vec![0u8; nodes.len()];
                keyed.select_nth_unstable(k - 1);
                for (_, _, local) in &keyed[..k] {
                    side[*local as usize] = 1;
                }
                let rest = &mut keyed[k..];
                let at = rest.len() - k;
                rest.select_nth_unstable(at);
                for (_, _, local) in &rest[at..] {
                    side[*local as usize] = 2;
                }
                drop(keyed);
                inertial_cut(lg, &side)
            })
            .collect::<Vec<_>>();
        candidates
            .into_iter()
            .min_by_key(|separator| separator.len())
            .unwrap_or_default()
    }
}

/// A cell's flow network: the nodes of `side` 0 keep their local order as
/// `0..k`, the sources (`side` 1) are contracted to `k` and the sinks (2) to
/// `k + 1`; one arc pair per cell edge between different images (parallel
/// arcs to the terminals kept), with twins.
struct FlowNetwork {
    image: Vec<u32>,
    first: Vec<u32>,
    head: Vec<u32>,
    twin: Vec<u32>,
}

impl FlowNetwork {
    fn new(lg: &LocalGraph, side: &[u8]) -> Self {
        let n = side.len();
        let middle = side.iter().filter(|s| **s == 0).count() as u32;
        let mut image = vec![0u32; n];
        let mut next = 0u32;
        for (u, s) in side.iter().enumerate() {
            image[u] = match s {
                0 => {
                    next += 1;
                    next - 1
                }
                1 => middle,
                _ => middle + 1,
            };
        }
        let count = middle as usize + 2;
        // Each cell edge once (u < v), unless both ends are the same terminal.
        let image_ref = &image;
        let edges = || {
            (0..n).flat_map(move |u| {
                (lg.first[u]..lg.first[u + 1]).filter_map(move |a| {
                    let v = lg.head[a as usize] as usize;
                    let (iu, iv) = (image_ref[u], image_ref[v]);
                    (v > u && !(iu == iv && iu >= middle)).then_some((iu, iv))
                })
            })
        };
        let mut first = vec![0u32; count + 1];
        for (iu, iv) in edges() {
            first[iu as usize + 1] += 1;
            first[iv as usize + 1] += 1;
        }
        for i in 0..count {
            first[i + 1] += first[i];
        }
        let mut fill = first.clone();
        let mut head = vec![0u32; first[count] as usize];
        let mut twin = vec![0u32; head.len()];
        for (iu, iv) in edges() {
            let (p, q) = (fill[iu as usize], fill[iv as usize]);
            fill[iu as usize] += 1;
            fill[iv as usize] += 1;
            head[p as usize] = iv;
            head[q as usize] = iu;
            twin[p as usize] = q;
            twin[q as usize] = p;
        }
        Self {
            image,
            first,
            head,
            twin,
        }
    }

    fn arcs(&self, u: usize) -> std::ops::Range<u32> {
        self.first[u]..self.first[u + 1]
    }
}

/// Unit-capacity max flow from the nodes with `side` 1 to those with 2, then
/// the node separator: the smaller endpoint side of the cut edges, where the
/// source side is what the sources reach in the residual graph.
///
/// The flow is ISAP on the [`FlowNetwork`]: exact distance labels to the sink
/// from a reverse search, advance along admissible arcs, relabel on a dead
/// end, stop at a gap; the labels are recomputed exactly whenever the work
/// since the last search exceeds half the network, which keeps local
/// relabelling from climbing one level at a time.
fn inertial_cut(lg: &LocalGraph, side: &[u8]) -> Vec<u32> {
    let net = FlowNetwork::new(lg, side);
    let count = net.first.len() - 1;
    let (s, t) = (count - 2, count - 1);
    let none = count as u32;
    // Flow on each arc in its own direction: -1, 0 or 1 (twin = -flow).
    let mut flow = vec![0i8; net.head.len()];
    let mut label = vec![none; count];
    let mut at_label = vec![0u32; count + 1];
    let mut current = vec![0u32; count];
    let mut queue = Vec::with_capacity(count);
    let mut path: Vec<u32> = Vec::new();
    let relabel_all =
        |flow: &[i8], label: &mut [u32], at_label: &mut [u32], queue: &mut Vec<u32>| {
            label.fill(none);
            at_label.fill(0);
            queue.clear();
            label[t] = 0;
            queue.push(t as u32);
            let mut cursor = 0;
            while cursor < queue.len() {
                let v = queue[cursor] as usize;
                cursor += 1;
                for a in net.arcs(v) {
                    let w = net.head[a as usize] as usize;
                    if label[w] == none && flow[net.twin[a as usize] as usize] < 1 {
                        label[w] = label[v] + 1;
                        queue.push(w as u32);
                    }
                }
            }
            for l in label.iter() {
                at_label[*l as usize] += 1;
            }
        };
    relabel_all(&flow, &mut label, &mut at_label, &mut queue);
    current.copy_from_slice(&net.first[..count]);
    let budget = (net.head.len() + count) as u64 / 2;
    let mut work = 0u64;
    let mut u = s;
    while label[s] < none {
        let end = net.first[u + 1];
        let mut a = current[u];
        while a < end {
            let v = net.head[a as usize] as usize;
            if flow[a as usize] < 1 && label[u] == label[v] + 1 {
                break;
            }
            a += 1;
        }
        work += u64::from(a - current[u]);
        current[u] = a;
        if a < end {
            path.push(a);
            u = net.head[a as usize] as usize;
            if u == t {
                for arc in &path {
                    flow[*arc as usize] += 1;
                    flow[net.twin[*arc as usize] as usize] -= 1;
                }
                path.clear();
                u = s;
            }
            continue;
        }
        // Dead end: relabel u to one more than its lowest residual neighbour.
        let lowest = net
            .arcs(u)
            .filter(|a| flow[*a as usize] < 1)
            .map(|a| label[net.head[a as usize] as usize])
            .min()
            .map_or(none, |l| (l + 1).min(none));
        work += u64::from(end - net.first[u]);
        let old = label[u] as usize;
        at_label[old] -= 1;
        if at_label[old] == 0 {
            // A gap: no node is `old` away from the sink any more, so u and
            // everything labelled higher (the source among them: labels
            // fall by one along the path) cannot reach it.
            break;
        }
        label[u] = lowest;
        at_label[lowest as usize] += 1;
        current[u] = net.first[u];
        if let Some(arc) = path.pop() {
            u = net.head[net.twin[arc as usize] as usize] as usize;
        }
        if work > budget {
            work = 0;
            relabel_all(&flow, &mut label, &mut at_label, &mut queue);
            current.copy_from_slice(&net.first[..count]);
            path.clear();
            u = s;
        }
    }
    // Source side: reachable from the sources in the residual graph.
    let mut reach = vec![false; count];
    queue.clear();
    reach[s] = true;
    queue.push(s as u32);
    let mut cursor = 0;
    while cursor < queue.len() {
        let x = queue[cursor] as usize;
        cursor += 1;
        for a in net.arcs(x) {
            let v = net.head[a as usize] as usize;
            if !reach[v] && flow[a as usize] < 1 {
                reach[v] = true;
                queue.push(v as u32);
            }
        }
    }
    let n = side.len();
    let reach = (0..n)
        .map(|u| reach[net.image[u] as usize])
        .collect::<Vec<_>>();
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

/// `values` cut into one slice per node of the CSR offsets `first`.
fn split_by<'v>(mut values: &'v mut [u32], first: &[u32]) -> Vec<&'v mut [u32]> {
    let mut slices = Vec::with_capacity(first.len().saturating_sub(1));
    for pair in first.windows(2) {
        let (slice, rest) = values.split_at_mut((pair[1] - pair[0]) as usize);
        slices.push(slice);
        values = rest;
    }
    slices
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

/// Planar integer coordinates for inertial flow: each node's fixed-point
/// unit vector (`NodeXyzFixed`, three `i32` per node) projected on east and
/// north at the nodes' mean position. Integer arithmetic only, so every
/// platform orders alike (packages are the same bytes everywhere).
fn coordinates(xyz_fixed: &[i32]) -> (Vec<i64>, Vec<i64>) {
    let n = xyz_fixed.len() / 3;
    let mut sum = [0i128; 3];
    for node in 0..n {
        for (axis, total) in sum.iter_mut().enumerate() {
            *total += i128::from(xyz_fixed[node * 3 + axis]);
        }
    }
    let count = (n as i128).max(1);
    let c = sum.map(|total| total / count);
    // east = z × c, north = c × east, scaled back to |east| by |c|.
    let east = [-c[1], c[0], 0i128];
    let north = [
        c[1] * east[2] - c[2] * east[1],
        c[2] * east[0] - c[0] * east[2],
        c[0] * east[1] - c[1] * east[0],
    ];
    let length = isqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).max(1);
    let north = north.map(|v| v / length);
    // A degenerate mean (the nodes straddle a pole or the origin): the raw
    // x and y axes.
    let (east, north) = if east == [0, 0, 0] || north == [0, 0, 0] {
        ([1, 0, 0], [0, 1, 0])
    } else {
        (east, north)
    };
    (0..n)
        .map(|node| {
            let p = [
                i128::from(xyz_fixed[node * 3]),
                i128::from(xyz_fixed[node * 3 + 1]),
                i128::from(xyz_fixed[node * 3 + 2]),
            ];
            let dot = |axis: [i128; 3]| (p[0] * axis[0] + p[1] * axis[1] + p[2] * axis[2]) >> 8;
            (dot(east) as i64, dot(north) as i64)
        })
        .unzip()
}

/// The integer square root of `value` (floor), by Newton's method.
fn isqrt(value: i128) -> i128 {
    if value < 2 {
        return value.max(0);
    }
    let mut x = value;
    let mut y = (x + 1) / 2;
    while y < x {
        x = y;
        y = (x + value / x) / 2;
    }
    x
}

/// What [`elimination_order`] found besides the order.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct OrderStats {
    /// Nodes eliminated before the nested dissection (degree ≤ 2).
    pub degree2_eliminated: usize,
    /// The largest top-level separator's node count (over the core's
    /// connected components).
    pub top_separator: usize,
}

/// The CCH elimination order (rank → node) of `graph`, whose nodes have
/// fixed-point unit vectors `xyz_fixed` (three per node).
pub fn elimination_order(graph: &NodeGraph, xyz_fixed: &[i32]) -> (Vec<u32>, OrderStats) {
    let mut stats = OrderStats::default();
    let n = graph.first.len().saturating_sub(1);
    // Degree ≤ 2 nodes first, iteratively, in place in a copy of the CSR:
    // eliminating v (neighbours a, b) takes v from a's list before it may
    // add b, so no list outgrows its slot. `len` is each list's length.
    let first = graph.first.clone();
    let mut head = graph.head.clone();
    let mut len = (0..n).map(|u| first[u + 1] - first[u]).collect::<Vec<_>>();
    let list = |len: &[u32], u: usize| -> std::ops::Range<usize> {
        first[u] as usize..(first[u] + len[u]) as usize
    };
    let mut eliminated = vec![false; n];
    let mut order = Vec::with_capacity(n);
    let mut stack = (0..n as u32)
        .rev()
        .filter(|node| len[*node as usize] <= 2)
        .collect::<Vec<_>>();
    while let Some(v) = stack.pop() {
        let vi = v as usize;
        if eliminated[vi] || len[vi] > 2 {
            continue;
        }
        eliminated[vi] = true;
        order.push(v);
        let mut neighbours = [0u32; 2];
        let count = len[vi] as usize;
        neighbours[..count].copy_from_slice(&head[list(&len, vi)]);
        len[vi] = 0;
        for a in &neighbours[..count] {
            let range = list(&len, *a as usize);
            if let Ok(at) = head[range.clone()].binary_search(&v) {
                head.copy_within(range.start + at + 1..range.end, range.start + at);
                len[*a as usize] -= 1;
            }
        }
        if let [a, b] = neighbours[..count] {
            let range = list(&len, a as usize);
            if let Err(at) = head[range.clone()].binary_search(&b) {
                head.copy_within(range.start + at..range.end, range.start + at + 1);
                head[range.start + at] = b;
                len[a as usize] += 1;
                let range = list(&len, b as usize);
                let at = head[range.clone()].binary_search(&a).unwrap_err();
                head.copy_within(range.start + at..range.end, range.start + at + 1);
                head[range.start + at] = a;
                len[b as usize] += 1;
            }
        }
        for a in &neighbours[..count] {
            if !eliminated[*a as usize] && len[*a as usize] <= 2 {
                stack.push(*a);
            }
        }
    }
    stats.degree2_eliminated = order.len();
    let core = (0..n as u32)
        .filter(|node| !eliminated[*node as usize])
        .collect::<Vec<_>>();
    drop(eliminated);
    // The core adjacency, compacted (eliminated nodes' lists are empty).
    let mut compact = 0usize;
    let mut core_first = Vec::with_capacity(n + 1);
    core_first.push(0u32);
    for u in 0..n {
        let range = list(&len, u);
        head.copy_within(range.clone(), compact);
        compact += range.len();
        core_first.push(compact as u32);
    }
    head.truncate(compact);
    head.shrink_to_fit();
    drop(len);
    let (first, head) = (core_first, head);
    let adjacency = NodeGraph { first, head };
    let (x, y) = coordinates(xyz_fixed);
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
    (order, stats)
}

/// Rank per node of an elimination `order` (rank → node).
pub fn ranks(order: &[u32]) -> Vec<u32> {
    let mut rank = vec![0u32; order.len()];
    for (r, node) in order.iter().enumerate() {
        rank[*node as usize] = r as u32;
    }
    rank
}

/// Whether `rank` (per node) is a permutation of `0..rank.len()`: a stored
/// order (`Section::CchRank`) comes from the package file.
pub fn is_permutation(rank: &[u32]) -> bool {
    let mut seen = vec![false; rank.len()];
    rank.iter().all(|r| {
        let r = *r as usize;
        r < seen.len() && !std::mem::replace(&mut seen[r], true)
    })
}

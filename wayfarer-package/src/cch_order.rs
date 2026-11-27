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

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use rayon::prelude::*;

const NO_NODE: u32 = u32::MAX;
/// Cells of at most this many nodes are ordered by minimum degree (bitsets).
const LEAF_CELL: usize = 64;
/// Cells of at least this many nodes build their local graph in parallel.
const PARALLEL_CELL: usize = 50_000;
/// Cells of at least this many nodes ask whether to stop first.
const STOP_CHECK_CELL: usize = 4_096;

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
    /// Asked before each cell of [`STOP_CHECK_CELL`] nodes or more; once it
    /// returns true the dissection gives up.
    stop: &'a (dyn Fn() -> bool + Sync),
    stopped: AtomicBool,
}

/// A cell of the nested dissection: its subcells (the parts its separator
/// leaves, or its connected components) and its separator, in order (a leaf
/// cell's nodes in minimum-degree order).
#[derive(Default)]
struct Cell {
    children: Vec<Cell>,
    separator: Vec<u32>,
}

impl Cell {
    /// The cell's order: its children's, then its separator.
    fn flatten(self, order: &mut Vec<u32>) {
        let mut stack = vec![(self, false)];
        while let Some((mut cell, expanded)) = stack.pop() {
            if expanded {
                order.extend(cell.separator);
                continue;
            }
            let children = std::mem::take(&mut cell.children);
            stack.push((cell, true));
            stack.extend(children.into_iter().rev().map(|child| (child, false)));
        }
    }
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

    /// The dissection of a cell: its subcells and its separator, whose
    /// post-order ([`Cell::flatten`]) is the cell's order, lowest rank first.
    fn order(&self, nodes: Vec<u32>, depth: usize) -> Cell {
        if self.stopped.load(Ordering::Relaxed) || (nodes.len() >= STOP_CHECK_CELL && (self.stop)())
        {
            self.stopped.store(true, Ordering::Relaxed);
            return Cell::default();
        }
        if nodes.len() <= LEAF_CELL {
            let id = self.claim(&nodes);
            let lg = self.local_graph(&nodes, id);
            return Cell {
                children: Vec::new(),
                separator: min_degree_order(&nodes, &lg),
            };
        }
        let id = self.claim(&nodes);
        let lg = self.local_graph(&nodes, id);
        let parts = self.components(&nodes, &lg, &vec![false; nodes.len()]);
        if parts.len() > 1 {
            drop(lg);
            // Components are cells of their own within the parent's.
            let children = parts
                .into_par_iter()
                .map(|part| self.order(part, depth))
                .collect::<Vec<_>>();
            return Cell {
                children,
                separator: Vec::new(),
            };
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
        let children = parts
            .into_par_iter()
            .map(|part| self.order(part, depth + 1))
            .collect::<Vec<_>>();
        Cell {
            children,
            separator: separator_nodes,
        }
    }

    /// A node separator of a connected cell (local indexes): the best of
    /// four inertial-flow cuts.
    fn separator(&self, nodes: &[u32], lg: &LocalGraph) -> Vec<u32> {
        const DIRECTIONS: [(i128, i128); 4] = [(1, 0), (0, 1), (1, 1), (1, -1)];
        // A cell lists its nodes in ascending order, so local indexes break
        // ties between equal projections as the node ids do.
        debug_assert!(nodes.is_sorted());
        let candidates = DIRECTIONS
            .par_iter()
            .map(|(dx, dy)| {
                let key = |local: usize| {
                    let at = nodes[local] as usize;
                    i128::from(self.x[at]) * dx + i128::from(self.y[at]) * dy
                };
                let side = quarter_sides(nodes.len(), key);
                inertial_cut(lg, &side)
            })
            .collect::<Vec<_>>();
        candidates
            .into_iter()
            .min_by_key(|separator| separator.len())
            .unwrap_or_default()
    }
}

/// The sources (1) and sinks (2) of an inertial-flow cut of a cell of `n`
/// nodes, by local index: the lowest quarter by (projection `key`, local
/// index) and the highest quarter of the rest. Only which nodes are in them
/// matters, so two selections give the same sets as a full sort; the pairs
/// are packed into a `u64` (projection above the index) when the cell's
/// projections span less than 2^32, as real coordinates always do.
fn quarter_sides(n: usize, key: impl Fn(usize) -> i128) -> Vec<u8> {
    let k = (n / 4).max(1);
    let mut side = vec![0u8; n];
    let (low, high) = (0..n).fold((i128::MAX, i128::MIN), |(low, high), local| {
        let value = key(local);
        (low.min(value), high.max(value))
    });
    if high - low <= i128::from(u32::MAX) {
        let mut keyed = (0..n)
            .map(|local| ((key(local) - low) as u64) << 32 | local as u64)
            .collect::<Vec<_>>();
        keyed.select_nth_unstable(k - 1);
        for packed in &keyed[..k] {
            side[(*packed as u32) as usize] = 1;
        }
        let rest = &mut keyed[k..];
        let at = rest.len() - k;
        rest.select_nth_unstable(at);
        for packed in &rest[at..] {
            side[(*packed as u32) as usize] = 2;
        }
    } else {
        let mut keyed = (0..n)
            .map(|local| (key(local), local as u32))
            .collect::<Vec<_>>();
        keyed.select_nth_unstable(k - 1);
        for (_, local) in &keyed[..k] {
            side[*local as usize] = 1;
        }
        let rest = &mut keyed[k..];
        let at = rest.len() - k;
        rest.select_nth_unstable(at);
        for (_, local) in &rest[at..] {
            side[*local as usize] = 2;
        }
    }
    side
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
    elimination_order_until(graph, xyz_fixed, &|| false).expect("an order that never stops")
}

/// [`elimination_order`], given up (`None`) once `stop` returns true: it is
/// asked now and then (between big cells of the dissection), from the thread
/// pool's threads.
pub fn elimination_order_until(
    graph: &NodeGraph,
    xyz_fixed: &[i32],
    stop: &(dyn Fn() -> bool + Sync),
) -> Option<(Vec<u32>, OrderStats)> {
    let Dissected {
        mut chains,
        root,
        stats,
        ..
    } = dissect(graph, xyz_fixed, stop)?;
    root.flatten(&mut chains);
    debug_assert_eq!(chains.len(), graph.first.len().saturating_sub(1));
    Some((chains, stats))
}

/// One node of the dissection tree of a cell-major order
/// ([`cell_major_order_until`]), as format 3's `CchCellTree` stores it: the
/// cell's rank range `first_rank..end_rank` holds the chain nodes it owns
/// (`first_rank..chains_end`), its children's ranges, then its separator
/// (`separator_start..end_rank`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CellTreeEntry {
    /// The parent's index in the tree (post-order), [`NO_PARENT`] for the
    /// root.
    pub parent: u32,
    pub first_rank: u32,
    pub chains_end: u32,
    pub separator_start: u32,
    pub end_rank: u32,
}

/// [`CellTreeEntry::parent`] of the root.
pub const NO_PARENT: u32 = u32::MAX;
/// The bytes of one [`CellTreeEntry`].
pub const CELL_TREE_ENTRY_BYTES: usize = 20;

impl CellTreeEntry {
    pub fn to_bytes(self) -> [u8; CELL_TREE_ENTRY_BYTES] {
        let mut bytes = [0u8; CELL_TREE_ENTRY_BYTES];
        let values = [
            self.parent,
            self.first_rank,
            self.chains_end,
            self.separator_start,
            self.end_rank,
        ];
        for (at, value) in values.iter().enumerate() {
            bytes[at * 4..at * 4 + 4].copy_from_slice(&value.to_le_bytes());
        }
        bytes
    }

    pub fn from_bytes(bytes: &[u8; CELL_TREE_ENTRY_BYTES]) -> Self {
        let at =
            |index: usize| u32::from_le_bytes(bytes[index * 4..index * 4 + 4].try_into().unwrap());
        Self {
            parent: at(0),
            first_rank: at(1),
            chains_end: at(2),
            separator_start: at(3),
            end_rank: at(4),
        }
    }
}

/// Whether `tree` is the dissection tree of a cell-major order of
/// `node_count` nodes: entries in post-order, each child's range inside its
/// parent's, between the parent's chain nodes and its separator, the
/// children in rank order and leaving no gap, and the root (the last entry)
/// ending at `node_count`. An empty tree is valid (no core to dissect).
pub fn is_cell_tree(tree: &[CellTreeEntry], node_count: usize) -> bool {
    let Some(root) = tree.last() else {
        return true;
    };
    if root.parent != NO_PARENT || root.end_rank as usize != node_count {
        return false;
    }
    // Where the next child of each entry must start.
    let mut next_child = tree
        .iter()
        .map(|entry| entry.chains_end)
        .collect::<Vec<_>>();
    for (index, entry) in tree.iter().enumerate() {
        if !(entry.first_rank <= entry.chains_end
            && entry.chains_end <= entry.separator_start
            && entry.separator_start <= entry.end_rank)
        {
            return false;
        }
        // Every child came before its parent and closed its gap.
        if next_child[index] != entry.separator_start {
            return false;
        }
        if index + 1 == tree.len() {
            break;
        }
        let parent = entry.parent as usize;
        if entry.parent == NO_PARENT
            || parent <= index
            || parent >= tree.len()
            || next_child[parent] != entry.first_rank
        {
            return false;
        }
        next_child[parent] = entry.end_rank;
    }
    // Post-order: each entry's children are the subtrees right before it,
    // so every subtree is one run of entries.
    let mut child_count = vec![0usize; tree.len()];
    for entry in &tree[..tree.len() - 1] {
        child_count[entry.parent as usize] += 1;
    }
    let mut open = Vec::new();
    for (index, count) in child_count.into_iter().enumerate() {
        let Some(first) = open.len().checked_sub(count) else {
            return false;
        };
        if open[first..]
            .iter()
            .any(|child: &usize| tree[*child].parent as usize != index)
        {
            return false;
        }
        open.truncate(first);
        open.push(index);
    }
    open.len() == 1
}

/// A cell-major CCH elimination order of `graph` (rank → node), its
/// dissection tree in post-order, and the order's stats; given up (`None`)
/// once `stop` returns true. It has the same fill and elimination tree as
/// [`elimination_order_until`]'s and differs only in where the nodes of
/// degree ≤ 2 go: each goes to the deepest dissection cell that owns one of
/// its neighbours when it is eliminated, at the start of that cell's range,
/// so every cell, at every level, is one range of ranks. Chain nodes of
/// components without a core come first. Within each group nodes keep their
/// relative order.
pub fn cell_major_order_until(
    graph: &NodeGraph,
    xyz_fixed: &[i32],
    stop: &(dyn Fn() -> bool + Sync),
) -> Option<(Vec<u32>, Vec<CellTreeEntry>, OrderStats)> {
    let n = graph.first.len().saturating_sub(1);
    let Dissected {
        chains,
        chain_neighbours,
        root,
        stats,
    } = dissect(graph, xyz_fixed, stop)?;
    // The tree in an arena, in pre-order: each cell's parent, depth,
    // children and separator.
    struct Node {
        parent: u32,
        depth: u32,
        children: Vec<u32>,
        separator: Vec<u32>,
    }
    let mut arena = Vec::<Node>::new();
    let mut owner = vec![NO_NODE; n];
    // No core (every node has degree ≤ 2 in the end): no tree.
    let mut stack = if root.children.is_empty() && root.separator.is_empty() {
        Vec::new()
    } else {
        vec![(root, NO_NODE, 0u32)]
    };
    while let Some((cell, parent, depth)) = stack.pop() {
        let index = arena.len() as u32;
        if parent != NO_NODE {
            arena[parent as usize].children.push(index);
        }
        for node in &cell.separator {
            owner[*node as usize] = index;
        }
        arena.push(Node {
            parent,
            depth,
            children: Vec::new(),
            separator: cell.separator,
        });
        // Children are pushed in reverse so that they are numbered, and
        // listed in their parent, in order.
        stack.extend(
            cell.children
                .into_iter()
                .rev()
                .map(|child| (child, index, depth + 1)),
        );
    }
    // A chain node's owner: the deeper of its neighbours' owners when it
    // was eliminated (its later chain neighbours are resolved first).
    for (at, node) in chains.iter().enumerate().rev() {
        let mut best = NO_NODE;
        for neighbour in chain_neighbours[at] {
            if neighbour == NO_NODE {
                continue;
            }
            let candidate = owner[neighbour as usize];
            if candidate != NO_NODE
                && (best == NO_NODE || arena[candidate as usize].depth > arena[best as usize].depth)
            {
                best = candidate;
            }
        }
        owner[*node as usize] = best;
    }
    // Each cell's chain nodes, in elimination order.
    let mut owned = vec![Vec::new(); arena.len()];
    let mut order = Vec::with_capacity(n);
    for node in &chains {
        match owner[*node as usize] {
            NO_NODE => order.push(*node),
            cell => owned[cell as usize].push(*node),
        }
    }
    drop(owner);
    // Ranks: each cell's chains, its children, its separator; the tree in
    // post-order.
    let mut post_index = vec![NO_NODE; arena.len()];
    let mut entries = Vec::with_capacity(arena.len());
    let mut first = vec![(0u32, 0u32); arena.len()];
    let mut stack = Vec::new();
    if !arena.is_empty() {
        stack.push((0u32, false));
    }
    while let Some((index, done)) = stack.pop() {
        let at = index as usize;
        if !done {
            let first_rank = order.len() as u32;
            order.extend(std::mem::take(&mut owned[at]));
            first[at] = (first_rank, order.len() as u32);
            stack.push((index, true));
            stack.extend(arena[at].children.iter().rev().map(|child| (*child, false)));
            continue;
        }
        let separator_start = order.len() as u32;
        order.extend(std::mem::take(&mut arena[at].separator));
        post_index[at] = entries.len() as u32;
        entries.push(CellTreeEntry {
            parent: arena[at].parent,
            first_rank: first[at].0,
            chains_end: first[at].1,
            separator_start,
            end_rank: order.len() as u32,
        });
    }
    for entry in &mut entries {
        if entry.parent != NO_NODE {
            entry.parent = post_index[entry.parent as usize];
        }
    }
    debug_assert_eq!(order.len(), n);
    debug_assert!(is_cell_tree(&entries, n));
    Some((order, entries, stats))
}

/// The nodes of degree ≤ 2 in elimination order, with their (up to two)
/// neighbours when eliminated, and the core's dissection.
struct Dissected {
    chains: Vec<u32>,
    chain_neighbours: Vec<[u32; 2]>,
    root: Cell,
    stats: OrderStats,
}

fn dissect(
    graph: &NodeGraph,
    xyz_fixed: &[i32],
    stop: &(dyn Fn() -> bool + Sync),
) -> Option<Dissected> {
    if stop() {
        return None;
    }
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
    let mut order = Vec::new();
    let mut chain_neighbours = Vec::new();
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
        let mut recorded = [NO_NODE; 2];
        recorded[..count].copy_from_slice(&neighbours[..count]);
        chain_neighbours.push(recorded);
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
        stop,
        stopped: AtomicBool::new(false),
    };
    let root = if core.is_empty() {
        Cell::default()
    } else {
        dissection.order(core, 0)
    };
    if dissection.stopped.load(Ordering::Relaxed) {
        return None;
    }
    stats.top_separator = dissection.top_separator.load(Ordering::Relaxed) as usize;
    Some(Dissected {
        chains: order,
        chain_neighbours,
        root,
        stats,
    })
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

#[cfg(test)]
mod tests {
    use super::*;

    /// The sides a full sort of (projection, local index) gives.
    fn sorted_sides(keys: &[i128]) -> Vec<u8> {
        let n = keys.len();
        let k = (n / 4).max(1);
        let mut by_key = (0..n).collect::<Vec<_>>();
        by_key.sort_by_key(|local| (keys[*local], *local));
        let mut side = vec![0u8; n];
        for local in &by_key[..k] {
            side[*local] = 1;
        }
        for local in &by_key[n - k..] {
            side[*local] = 2;
        }
        side
    }

    /// A `side` × `side` grid of nodes 0.001° apart, four-neighboured.
    fn grid(side: u32) -> (NodeGraph, Vec<i32>) {
        let mut first_out = vec![0u32];
        let mut target = Vec::new();
        let mut xyz = Vec::new();
        for row in 0..side {
            for column in 0..side {
                let node = row * side + column;
                if column > 0 {
                    target.push(node - 1);
                }
                if column + 1 < side {
                    target.push(node + 1);
                }
                if row > 0 {
                    target.push(node - side);
                }
                if row + 1 < side {
                    target.push(node + side);
                }
                first_out.push(target.len() as u32);
                let (lat, lng) = (
                    50.0 + f64::from(row) * 1e-3,
                    19.0 + f64::from(column) * 1e-3,
                );
                xyz.extend(crate::format::fixed_xyz(crate::format::unit_xyz(lat, lng)));
            }
        }
        (NodeGraph::new(&first_out, &target), xyz)
    }

    #[test]
    fn a_stopped_order_gives_up() {
        let (graph, xyz) = grid(100);
        let (order, _) = elimination_order(&graph, &xyz);
        assert!(is_permutation(&ranks(&order)));
        assert_eq!(
            elimination_order_until(&graph, &xyz, &|| false).map(|(order, _)| order),
            Some(order)
        );
        assert!(elimination_order_until(&graph, &xyz, &|| true).is_none());
        // Stopped once the dissection has started.
        let asked = AtomicU32::new(0);
        let stop = || asked.fetch_add(1, Ordering::Relaxed) >= 1;
        assert!(elimination_order_until(&graph, &xyz, &stop).is_none());
    }

    /// The filled graph of `order` as node pairs (lower node first) and
    /// each node's parent in the elimination tree (`NO_NODE` for a root).
    fn fill(
        graph: &NodeGraph,
        order: &[u32],
    ) -> (std::collections::BTreeSet<(u32, u32)>, Vec<u32>) {
        let n = order.len();
        let rank = ranks(order);
        let mut up = (0..n)
            .map(|_| std::collections::BTreeSet::new())
            .collect::<Vec<_>>();
        for node in 0..n as u32 {
            for w in graph.neighbours(node) {
                if rank[*w as usize] > rank[node as usize] {
                    up[rank[node as usize] as usize].insert(rank[*w as usize]);
                }
            }
        }
        let mut arcs = std::collections::BTreeSet::new();
        let mut parent = vec![NO_NODE; n];
        for r in 0..n {
            let set = std::mem::take(&mut up[r]);
            for w in &set {
                let (a, b) = (order[r], order[*w as usize]);
                arcs.insert((a.min(b), a.max(b)));
            }
            if let Some(p) = set.iter().next().copied() {
                parent[order[r] as usize] = order[p as usize];
                for w in set.iter().skip(1) {
                    up[p as usize].insert(*w);
                }
            }
        }
        (arcs, parent)
    }

    /// A grid with holes, dangling chains, a tree and isolated paths and
    /// cycles: every kind of node of degree ≤ 2.
    fn ragged(side: u32, seed: u64) -> (NodeGraph, Vec<i32>) {
        let mut state = seed;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        let mut pairs = Vec::new();
        let mut positions = Vec::new();
        for row in 0..side {
            for column in 0..side {
                positions.push((
                    50.0 + f64::from(row) * 1e-3,
                    19.0 + f64::from(column) * 1e-3,
                ));
                let node = row * side + column;
                if column + 1 < side && next() % 5 != 0 {
                    pairs.push((node, node + 1));
                }
                if row + 1 < side && next() % 5 != 0 {
                    pairs.push((node, node + side));
                }
            }
        }
        let mut add =
            |positions: &mut Vec<(f64, f64)>, from: Option<u32>, length: u32, ring: bool| {
                let start = positions.len() as u32;
                for at in 0..length {
                    positions.push((
                        49.0 + f64::from(start + at) * 1e-4,
                        18.0 + f64::from(at) * 1e-4,
                    ));
                    if at > 0 {
                        pairs.push((start + at - 1, start + at));
                    }
                }
                if let Some(from) = from {
                    pairs.push((from, start));
                }
                if ring {
                    pairs.push((start, start + length - 1));
                }
            };
        for at in 0..side {
            add(
                &mut positions,
                Some(at * 7 % (side * side)),
                1 + at % 5,
                false,
            );
        }
        add(&mut positions, None, 9, false);
        add(&mut positions, None, 6, true);
        add(&mut positions, None, 1, false);
        let n = positions.len();
        let mut targets = vec![Vec::new(); n];
        for (a, b) in &pairs {
            targets[*a as usize].push(*b);
            targets[*b as usize].push(*a);
        }
        let mut first_out = vec![0u32];
        let mut target = Vec::new();
        for list in &targets {
            target.extend(list);
            first_out.push(target.len() as u32);
        }
        let xyz = positions
            .iter()
            .flat_map(|(lat, lng)| crate::format::fixed_xyz(crate::format::unit_xyz(*lat, *lng)))
            .collect();
        (NodeGraph::new(&first_out, &target), xyz)
    }

    /// The cell-major order fills the graph exactly as the plain one does,
    /// with the same elimination tree, and every cell is one rank range.
    #[test]
    fn a_cell_major_order_keeps_the_fill_and_the_elimination_tree() {
        for (side, seed) in [(12, 1), (30, 7), (45, 99)] {
            let (graph, xyz) = ragged(side, seed);
            let (plain, plain_stats) = elimination_order(&graph, &xyz);
            let (order, tree, stats) = cell_major_order_until(&graph, &xyz, &|| false).unwrap();
            assert_eq!(stats, plain_stats);
            assert!(is_permutation(&ranks(&order)));
            assert_eq!(fill(&graph, &order), fill(&graph, &plain), "side {side}");
            let n = order.len();
            assert!(is_cell_tree(&tree, n));
            assert!(!tree.is_empty());
            // The isolated paths and the ring have no core: they come first,
            // in the plain order's relative order.
            let front = tree.iter().map(|entry| entry.first_rank).min().unwrap() as usize;
            assert!(front >= 16, "{front}");
            let plain_rank = ranks(&plain);
            assert!(
                order[..front]
                    .windows(2)
                    .all(|pair| plain_rank[pair[0] as usize] < plain_rank[pair[1] as usize])
            );
            // A cell's chain nodes join only nodes of the cell or of its
            // ancestors' separators.
            let rank = ranks(&order);
            for entry in &tree {
                for r in entry.first_rank..entry.chains_end {
                    let node = order[r as usize];
                    for w in graph.neighbours(node) {
                        let rw = rank[*w as usize];
                        assert!(rw >= entry.first_rank, "rank {r} joins {rw}");
                    }
                }
            }
            // Bytes round-trip.
            for entry in &tree {
                assert_eq!(CellTreeEntry::from_bytes(&entry.to_bytes()), *entry);
            }
        }
        // No core at all: no tree.
        let graph = NodeGraph::new(&[0, 1, 2], &[1, 0]);
        let xyz = [0i32; 6];
        let (order, tree, _) = cell_major_order_until(&graph, &xyz, &|| false).unwrap();
        assert_eq!((order.len(), tree.len()), (2, 0));
        assert!(cell_major_order_until(&graph, &xyz, &|| true).is_none());
    }

    #[test]
    fn cell_trees_are_checked() {
        let leaf = |parent, first_rank, chains_end, separator_start, end_rank| CellTreeEntry {
            parent,
            first_rank,
            chains_end,
            separator_start,
            end_rank,
        };
        // Two leaves under a root: 0..2 front, root chains 2..3, leaves
        // 3..5 and 5..8, root separator 8..10.
        let good = vec![
            leaf(2, 3, 3, 3, 5),
            leaf(2, 5, 6, 6, 8),
            leaf(NO_PARENT, 2, 3, 8, 10),
        ];
        assert!(is_cell_tree(&good, 10));
        assert!(is_cell_tree(&[], 4));
        assert!(!is_cell_tree(&good, 11));
        let cases: [fn(&mut Vec<CellTreeEntry>); 6] = [
            |tree| tree[1].first_rank = 6,
            |tree| tree[0].parent = 0,
            |tree| tree[0].parent = 5,
            |tree| tree[2].parent = 1,
            |tree| tree[1].chains_end = 9,
            |tree| tree.swap(0, 1),
        ];
        for (at, edit) in cases.iter().enumerate() {
            let mut tree = good.clone();
            edit(&mut tree);
            assert!(!is_cell_tree(&tree, 10), "case {at}");
        }
    }

    #[test]
    fn quarter_sides_match_a_full_sort() {
        let mut state = 0x2545_F491_4F6C_DD1Du64;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        for n in [65usize, 100, 257, 1_000] {
            // Narrow keys with many ties, real-coordinate spans, and spans
            // past 2^32 (the unpacked path).
            for span in [8u64, 1 << 26, 1 << 40] {
                let keys = (0..n)
                    .map(|_| (next() % span) as i128 - (span / 2) as i128)
                    .collect::<Vec<_>>();
                assert_eq!(
                    quarter_sides(n, |local| keys[local]),
                    sorted_sides(&keys),
                    "n {n}, span {span}"
                );
            }
        }
    }
}

//! The node order of CCH potentials (see the engine's `package::cch`): the
//! node-level graph loses its nodes of degree ≤ 2 first, iteratively (chains
//! and trees, each eliminated node adds at most one arc); the core left is
//! ordered by nested dissection with inertial flow (4 directions, 25 %
//! sources and sinks, unit-capacity max flow by Dinic, node separator from
//! the smaller side of the cut edges' endpoints), down to cells of
//! 64 nodes ordered by minimum degree. Ties break on node ids and
//! coordinates are integers, so the order is the same on every platform: the
//! compiler stores it (`Section::CchRank`).

use std::sync::atomic::{AtomicU32, Ordering};

use rayon::prelude::*;

const NO_NODE: u32 = u32::MAX;
/// Cells of at most this many nodes are ordered by minimum degree (bitsets).
const LEAF_CELL: usize = 64;

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
    adjacency: &'a [Vec<u32>],
    x: &'a [i64],
    y: &'a [i64],
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
        const DIRECTIONS: [(i128, i128); 4] = [(1, 0), (0, 1), (1, 1), (1, -1)];
        let candidates = DIRECTIONS
            .par_iter()
            .map(|(dx, dy)| {
                let mut by_projection = (0..nodes.len() as u32).collect::<Vec<_>>();
                let key = |local: u32| {
                    let node = nodes[local as usize] as usize;
                    i128::from(self.x[node]) * dx + i128::from(self.y[node]) * dy
                };
                by_projection.sort_unstable_by(|a, b| {
                    key(*a)
                        .cmp(&key(*b))
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
    /// The largest separator's node count.
    pub top_separator: usize,
}

/// The CCH elimination order (rank → node) of `graph`, whose nodes have
/// fixed-point unit vectors `xyz_fixed` (three per node).
pub fn elimination_order(graph: &NodeGraph, xyz_fixed: &[i32]) -> (Vec<u32>, OrderStats) {
    let mut stats = OrderStats::default();
    let n = graph.first.len().saturating_sub(1);
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

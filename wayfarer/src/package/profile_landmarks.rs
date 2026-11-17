//! Landmarks of a derived column's own metric, computed on the device at first
//! use from the package's landmark nodes.
//!
//! A derived column borrows its stored column's landmarks scaled by the least
//! cost ratio over its edges (`columns`). For Emergency and Tracked that ratio
//! is set by tracks, which they cost far below Wheeled (0.45 and 0.031; 0.0077
//! for Emergency at override strength 1), and every bound shrinks with it.
//! Here the same landmark nodes are measured again on the profile's own
//! *metric*, so its columns keep their bounds at scale 1:
//!
//! - the profile's costs under the request's costing options with legal
//!   access (destination-only edges allowed), and, on a package with SAR
//!   data, every physically drivable edge the profile may not use at that
//!   cost times the package's minimum R1 and R2 factors (as the Wheeled
//!   landmarks of the package are built, `SarPackageInfo`). The strict
//!   destination-only pass and every SAR column of the profile cost each edge
//!   at least that, so they share the landmarks.
//!
//! The Dijkstras are the compiler's (`package_build::landmarks`): one quantum
//! per direction from the 0.995 quantile of exact distances from and to a
//! hub, then whole quanta over arcs of `floor(c / q)` clamped at
//! `LANDMARK_SATURATED`, which makes the bounds consistent as well as
//! admissible. Distances are stored as `u32` per row, with rows of their
//! own in the package's sparse layout (`LandmarkRows::sparse`): the nodes
//! the first landmark reaches or is reached from on the metric. The metric
//! may reach nodes the stored column's rows leave out (an R5 metric's
//! paths). The landmarks lie in one strongly connected component of the
//! stored landmark metric, so they normally reach the same nodes on these
//! metrics too (checked: a finite distance outside the rows abandons the
//! build, and the column keeps its scaled bounds).

use wayfarer_package::format::LandmarkRows;

use std::time::Instant;

use rayon::prelude::*;
use wayfarer_package::format::{LANDMARK_INFINITY, LANDMARK_SATURATED, LandmarkTable};

use super::SearchGraph;

/// The quantization range covers this share of finite distances (as the
/// compiler's).
const RANGE_QUANTILE: f64 = 0.995;
const MAX_FINITE: f64 = (u32::MAX - 2) as f64;

/// A derived metric's landmark tables over a stored column's rows.
pub(crate) struct ProfileLandmarks {
    /// The stored column whose landmark nodes and rows these reuse.
    pub column: usize,
    /// Landmarks per row.
    pub count: usize,
    /// `[column][landmark][from|to]` quanta, laid out as the package's
    /// `LandmarkScale` for `count` landmarks (zero in other columns).
    pub scales: Vec<f32>,
    /// `[row][landmark]`, over the rows of `present` and `rank`.
    from: Vec<u32>,
    to: Vec<u32>,
    /// Rows: `[column][node / 64]` presence bits and the rows before each
    /// word (zero in other columns), as `LandmarkRows` reads them.
    present: Vec<u64>,
    rank: Vec<u32>,
    node_count: usize,
    /// Time spent building them (diagnostics).
    pub build_ms: f64,
}

impl ProfileLandmarks {
    /// Measures the first `count` landmarks of stored `column` on `metric`
    /// (a cost per edge, infinite where unusable). `None` when the package
    /// has no landmarks, or a landmark reaches a node the first one does
    /// not.
    pub(crate) fn build(
        view: &SearchGraph<'_>,
        metric: &[f32],
        column: usize,
        count: usize,
    ) -> Option<Self> {
        let started = Instant::now();
        let stored = view.landmark_count;
        if stored == 0 || count == 0 {
            return None;
        }
        let count = count.min(stored);
        let nodes = &view.landmark_node[column * stored..column * stored + count];
        let forward = Arcs::new(view, metric, true);
        let backward = Arcs::new(view, metric, false);
        let quantum = quantum(&forward, &backward, nodes[0]);

        let words = view.node_count.div_ceil(64);
        let mut present = vec![0u64; (column + 1) * words];
        let mut rank = vec![0u32; (column + 1) * words];
        // Node -> row (u32::MAX: no row), from the first landmark's tables.
        let mut row_of = Vec::new();
        let mut from = Vec::new();
        let mut to = Vec::new();
        // A few landmarks at a time (one Dijkstra per thread): each holds a
        // distance per node.
        let batch = (rayon::current_num_threads() / 2).max(1);
        for (first, chunk) in nodes.chunks(batch).enumerate() {
            let tables = chunk
                .par_iter()
                .flat_map_iter(|node| [(&forward, *node), (&backward, *node)])
                .collect::<Vec<_>>()
                .into_par_iter()
                .map(|(arcs, node)| quantized_dijkstra(arcs, node, quantum))
                .collect::<Vec<_>>();
            if first == 0 {
                let (forward, backward) = (&tables[0], &tables[1]);
                let words_of_column = &mut present[column * words..];
                for node in 0..view.node_count {
                    if forward[node] != LANDMARK_INFINITY || backward[node] != LANDMARK_INFINITY {
                        words_of_column[node / 64] |= 1 << (node % 64);
                    }
                }
                let mut rows = 0u32;
                for word in 0..words {
                    rank[column * words + word] = rows;
                    rows += present[column * words + word].count_ones();
                }
                row_of = (0..view.node_count)
                    .map(|node| {
                        let word = column * words + node / 64;
                        let bit = 1u64 << (node % 64);
                        if present[word] & bit == 0 {
                            u32::MAX
                        } else {
                            rank[word] + (present[word] & (bit - 1)).count_ones()
                        }
                    })
                    .collect();
                from = vec![LANDMARK_INFINITY; rows as usize * count];
                to = vec![LANDMARK_INFINITY; rows as usize * count];
            }
            for (index, pair) in tables.chunks(2).enumerate() {
                let landmark = first * batch + index;
                for (table, out) in pair.iter().zip([&mut from, &mut to]) {
                    for (node, value) in table.iter().enumerate() {
                        let row = row_of[node];
                        if row == u32::MAX {
                            if *value != LANDMARK_INFINITY {
                                return None;
                            }
                            continue;
                        }
                        out[row as usize * count + landmark] = *value;
                    }
                }
            }
        }
        let mut scales = vec![0.0f32; (column + 1) * count * 2];
        for landmark in 0..count {
            let slot = (column * count + landmark) * 2;
            scales[slot] = quantum;
            scales[slot + 1] = quantum;
        }
        Some(Self {
            column,
            count,
            scales,
            from,
            to,
            present,
            rank,
            node_count: view.node_count,
            build_ms: started.elapsed().as_secs_f64() * 1e3,
        })
    }

    /// The from and to tables, over their own rows.
    pub(crate) fn tables(&self) -> (LandmarkTable<'_>, LandmarkTable<'_>) {
        let rows = LandmarkRows::sparse(&self.present, &self.rank, self.node_count);
        (
            LandmarkTable::column_values(&self.from, rows, self.column, self.count),
            LandmarkTable::column_values(&self.to, rows, self.column, self.count),
        )
    }

    /// Heap bytes held.
    pub(crate) fn byte_len(&self) -> usize {
        (self.from.len() + self.to.len()) * 4
            + self.scales.len() * 4
            + self.present.len() * 8
            + self.rank.len() * 4
    }
}

/// One direction's usable arcs, packed per node.
struct Arcs {
    first: Vec<u32>,
    arcs: Vec<(u32, f32)>,
}

impl Arcs {
    fn new(view: &SearchGraph<'_>, metric: &[f32], forward: bool) -> Self {
        let usable = |edge: usize| metric[edge].is_finite() && metric[edge] >= 0.0;
        let mut first = Vec::with_capacity(view.node_count + 1);
        let mut arcs = Vec::new();
        first.push(0);
        for node in 0..view.node_count {
            if forward {
                for edge in view.first_out[node]..view.first_out[node + 1] {
                    let edge = edge as usize;
                    if usable(edge) {
                        arcs.push((view.edge_target[edge], metric[edge]));
                    }
                }
            } else {
                for (edge, source) in view.in_edges(node) {
                    let edge = edge as usize;
                    if usable(edge) {
                        arcs.push((source, metric[edge]));
                    }
                }
            }
            first.push(arcs.len() as u32);
        }
        arcs.shrink_to_fit();
        Self { first, arcs }
    }

    #[inline]
    fn of(&self, node: u32) -> &[(u32, f32)] {
        &self.arcs[self.first[node as usize] as usize..self.first[node as usize + 1] as usize]
    }
}

/// Monotone priority queue on `u64` keys (see the compiler's `RadixHeap`).
struct RadixHeap {
    last: u64,
    len: usize,
    buckets: Vec<Vec<(u64, u32)>>,
}

impl RadixHeap {
    fn new() -> Self {
        Self {
            last: 0,
            len: 0,
            buckets: (0..=64).map(|_| Vec::new()).collect(),
        }
    }

    #[inline]
    fn bucket(&self, key: u64) -> usize {
        (64 - (key ^ self.last).leading_zeros()) as usize
    }

    #[inline]
    fn push(&mut self, key: u64, value: u32) {
        debug_assert!(key >= self.last);
        let bucket = self.bucket(key);
        self.buckets[bucket].push((key, value));
        self.len += 1;
    }

    fn pop(&mut self) -> Option<(u64, u32)> {
        if self.len == 0 {
            return None;
        }
        if self.buckets[0].is_empty() {
            let index = (1..self.buckets.len()).find(|index| !self.buckets[*index].is_empty())?;
            let mut moved = std::mem::take(&mut self.buckets[index]);
            self.last = moved.iter().map(|(key, _)| *key).min()?;
            for (key, value) in moved.drain(..) {
                let bucket = self.bucket(key);
                self.buckets[bucket].push((key, value));
            }
            self.buckets[index] = moved;
        }
        self.len -= 1;
        self.buckets[0].pop()
    }
}

/// Exact single-source costs (keys are the bits of non-negative `f64`s).
fn dijkstra(arcs: &Arcs, source: u32) -> Vec<f64> {
    let mut dist = vec![f64::INFINITY; arcs.first.len() - 1];
    let mut heap = RadixHeap::new();
    dist[source as usize] = 0.0;
    heap.push(0f64.to_bits(), source);
    while let Some((key, node)) = heap.pop() {
        let d = f64::from_bits(key);
        if d > dist[node as usize] {
            continue;
        }
        for (next, cost) in arcs.of(node) {
            let candidate = d + f64::from(*cost);
            if candidate < dist[*next as usize] {
                dist[*next as usize] = candidate;
                heap.push(candidate.to_bits(), *next);
            }
        }
    }
    dist
}

/// The compiler's rule (`column_quantum`): `MAX_FINITE` steps cover the
/// range quantile of distances from `hub` plus that of distances to it.
fn quantum(forward: &Arcs, backward: &Arcs, hub: u32) -> f32 {
    let range = |mut finite: Vec<f64>| {
        finite.retain(|d| d.is_finite());
        if finite.is_empty() {
            return 0.0;
        }
        let rank = ((finite.len() - 1) as f64 * RANGE_QUANTILE) as usize;
        *finite.select_nth_unstable_by(rank, f64::total_cmp).1
    };
    let (from, to) = rayon::join(
        || range(dijkstra(forward, hub)),
        || range(dijkstra(backward, hub)),
    );
    ((from + to) / MAX_FINITE).max(1e-9) as f32
}

/// `floor(cost / quantum)`, less one where the division rounded up.
#[inline]
fn arc_quanta(cost: f32, quantum: f64) -> u64 {
    let cost = f64::from(cost);
    let units = (cost / quantum).floor();
    if units * quantum > cost {
        (units - 1.0) as u64
    } else {
        units as u64
    }
}

/// Distances in whole quanta over arcs of `floor(c / q)`, clamped to
/// `LANDMARK_SATURATED` (`LANDMARK_INFINITY` when unreachable). Labels are
/// clamped as they are relaxed: `min(d + a, S)` is monotone in `d`, so the
/// search yields `min(D, S)` exactly, as the compiler's `u64` labels do.
fn quantized_dijkstra(arcs: &Arcs, source: u32, quantum: f32) -> Vec<u32> {
    const { assert!(LANDMARK_SATURATED < LANDMARK_INFINITY) };
    let quantum = f64::from(quantum);
    let saturated = u64::from(LANDMARK_SATURATED);
    let mut dist = vec![LANDMARK_INFINITY; arcs.first.len() - 1];
    let mut heap = RadixHeap::new();
    dist[source as usize] = 0;
    heap.push(0, source);
    while let Some((d, node)) = heap.pop() {
        if d > u64::from(dist[node as usize]) {
            continue;
        }
        for (next, cost) in arcs.of(node) {
            let candidate = (d + arc_quanta(*cost, quantum).min(saturated)).min(saturated);
            if candidate < u64::from(dist[*next as usize]) {
                dist[*next as usize] = candidate as u32;
                heap.push(candidate, *next);
            }
        }
    }
    dist
}

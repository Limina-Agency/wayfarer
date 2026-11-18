//! ALT landmarks: selection, Dijkstras per landmark and direction, and
//! quantization into the package's landmark sections.
//!
//! Selection is maxCover (Goldberg & Werneck) over geometric candidates: the
//! farthest node in each of `4 k` angular sectors around the centre of the
//! column's largest strongly connected component is measured as a landmark,
//! and `k` of them are kept greedily, each covering the most arcs not yet
//! covered, an arc being covered when some kept landmark's stored bound is
//! tight on it.
//!
//! Each column gets one quantum `q`, from two exact Dijkstras (from and to a
//! node `x` of its main component): every finite landmark distance is at
//! most `max d(., x) + max d(x, .)` by the triangle inequality, and
//! `LANDMARK_MAX_FINITE` steps of `q` cover that. Each landmark's Dijkstras
//! then run in whole quanta over arcs of `floor(c / q)`, stored as `u32`. On every arc `u -> v` the stored
//! distances satisfy `D(L, v) - D(L, u) <= floor(c / q)` (and likewise towards
//! the landmark), so the query's bounds `q * (D(L, t) - D(L, v))` and
//! `q * (D(v, L) - D(t, L))` are consistent as well as admissible, which the
//! bidirectional stop needs to be exact. Distances beyond the range are
//! stored clamped to `LANDMARK_SATURATED`: `min(D, S)` keeps both properties.
//!
//! The quantized Dijkstras run on the column's graph contracted to its
//! junctions ([`ColumnGraph`]): every other node lies in a dead-end tree or
//! on a chain whose one or two anchor junctions every path to it passes, so
//! its distances are its anchors' plus offsets inside its piece. Candidates
//! expand those to every node for maxCover; the package stores rows only for
//! junctions (and the pieces that hold a landmark or have no anchor), and
//! anchors and offsets for the rest.
//!
//! A landmark lies in its column's largest strongly connected component, so a
//! node's row is finite for every landmark or for none; the rows of nodes no
//! landmark reaches and none is reached from (footways in the driving column,
//! say) are left out of the tables, and a presence bit per (column, node) with
//! a rank count per 64-bit word locates the others (`LandmarkRows`).

use std::collections::VecDeque;

use wayfarer_package::format::{
    LANDMARK_INFINITY, LANDMARK_SATURATED, NO_ANCHOR_ROW, ONE_ANCHOR_WORDS, RoutingPackageError,
    TWO_ANCHOR_WORDS, derived_landmark_value,
};
use wayfarer_package::math;

use rayon::prelude::*;

use super::cancelled_error;
use crate::CompileStage;
use crate::job::{Job, UnitCounter};

/// Settled nodes between checks for a cancel inside one Dijkstra (a few
/// milliseconds of work).
const CANCEL_CHECK_NODES: usize = 1 << 16;

/// Landmarks per column a package stores when asked for them
/// ([`super::PackageBuildOptions::landmark_count`]; the default stores none).
pub const DEFAULT_LANDMARK_COUNT: usize = 16;
/// Candidates measured per landmark kept (at most 64 in all: the cover masks
/// are `u64`).
const CANDIDATES_PER_LANDMARK: usize = 2;
const LANDMARK_MAX_FINITE: f64 = (u32::MAX - 2) as f64;
/// The quantization range covers this share of finite distances; the rest
/// saturate. A few pathological nodes (behind huge access penalties) would
/// otherwise set a coarse quantum for every node.
const LANDMARK_RANGE_QUANTILE: f64 = 0.995;

pub(crate) struct LandmarkInput<'a> {
    pub node_count: usize,
    pub edge_count: usize,
    pub first_out: &'a [u32],
    pub first_in: &'a [u32],
    pub in_edges: &'a [u32],
    pub edge_source: &'a [u32],
    pub edge_target: &'a [u32],
    pub edge_cost: &'a [f32],
    pub node_lat: &'a [f64],
    pub node_lng: &'a [f64],
}

pub(crate) struct LandmarkData {
    /// `[column][landmark]` node index.
    pub nodes: Vec<u32>,
    /// `[column][landmark][from=0|to=1]` quantum in cost units.
    pub scales: Vec<f32>,
    /// `[stored row][landmark]` min(D(landmark, node), saturated), with D
    /// the distance over `floor(c / q)` arcs.
    pub from: Vec<u32>,
    /// `[stored row][landmark]` min(D(node, landmark), saturated).
    pub to: Vec<u32>,
    /// `[column][node / 64]`: bits of the (column, node) rows stored.
    pub present: Vec<u64>,
    /// `[column][node / 64]`: rows stored before the word.
    pub rank: Vec<u32>,
    /// Rows stored, over all columns.
    pub row_count: usize,
    /// `[column][node / 64]`: bits of the (column, node)s derived from one
    /// anchor, and records before the word.
    pub one_present: Vec<u64>,
    pub one_rank: Vec<u32>,
    /// `[record][anchor row, from offset, to offset]`.
    pub one: Vec<u32>,
    /// Likewise for two anchors: `[record][row a, row b, from a, to a, from
    /// b, to b]`.
    pub two_present: Vec<u64>,
    pub two_rank: Vec<u32>,
    pub two: Vec<u32>,
    /// Anchored (column, node)s per column: (one anchor, two anchors).
    pub anchored: Vec<(usize, usize)>,
}

/// Monotone priority queue on `u64` keys (whole quanta, or the bits of
/// non-negative `f64`s, whose order matches the values'): a key goes to the
/// bucket of the highest bit in which it differs from the last popped key.
/// Dijkstra's keys never drop below the last popped one, which is all the
/// structure needs.
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
        debug_assert!(key >= self.last, "radix heap keys must not decrease");
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
            // Keep the emptied bucket's allocation for later rounds.
            self.buckets[index] = moved;
        }
        self.len -= 1;
        self.buckets[0].pop()
    }
}

/// One column's arcs in one direction, with usable (finite, non-negative)
/// costs inline: the 32 Dijkstras of a column read it sequentially per node
/// instead of gathering targets and costs from separate edge arrays.
struct PackedArcs {
    first: Vec<u32>,
    arcs: Vec<(u32, f32)>,
}

impl PackedArcs {
    fn new(input: &LandmarkInput<'_>, column: usize, forward: bool) -> Self {
        let costs = &input.edge_cost[column * input.edge_count..(column + 1) * input.edge_count];
        // Costs are non-negative by construction; a negative one would break
        // the monotone queue (and every ALT bound), so it is not an arc.
        let usable = |edge: usize| costs[edge].is_finite() && costs[edge] >= 0.0;
        let mut first = Vec::with_capacity(input.node_count + 1);
        let mut arcs = Vec::with_capacity(input.edge_count);
        first.push(0);
        for node in 0..input.node_count {
            if forward {
                for edge in input.first_out[node]..input.first_out[node + 1] {
                    let edge = edge as usize;
                    if usable(edge) {
                        arcs.push((input.edge_target[edge], costs[edge]));
                    }
                }
            } else {
                for slot in input.first_in[node]..input.first_in[node + 1] {
                    let edge = input.in_edges[slot as usize] as usize;
                    if usable(edge) {
                        arcs.push((input.edge_source[edge], costs[edge]));
                    }
                }
            }
            first.push(arcs.len() as u32);
        }
        Self { first, arcs }
    }
}

/// Exact single-source costs (for the quantum). The queue order among equal
/// keys does not affect the result: every node's cost is the minimum over its
/// settled neighbours, and adding a non-negative cost never decreases a float.
/// Stops early, with partial costs, once `job` is cancelled.
fn dijkstra(arcs: &PackedArcs, source: u32, job: &Job<'_>) -> Vec<f64> {
    let mut dist = vec![f64::INFINITY; arcs.first.len() - 1];
    let mut heap = RadixHeap::new();
    dist[source as usize] = 0.0;
    heap.push(0f64.to_bits(), source);
    let mut settled = 0usize;
    while let Some((key, node)) = heap.pop() {
        let d = f64::from_bits(key);
        if d > dist[node as usize] {
            continue;
        }
        settled += 1;
        if settled.is_multiple_of(CANCEL_CHECK_NODES) && job.cancelled() {
            break;
        }
        let range = arcs.first[node as usize] as usize..arcs.first[node as usize + 1] as usize;
        for (next, cost) in &arcs.arcs[range] {
            let candidate = d + f64::from(*cost);
            if candidate < dist[*next as usize] {
                dist[*next as usize] = candidate;
                heap.push(candidate.to_bits(), *next);
            }
        }
    }
    dist
}

/// Largest strongly connected component of the column's usable arcs
/// (Kosaraju): a landmark there has finite distances from and to all of it.
fn main_component(forward: &PackedArcs, backward: &PackedArcs) -> Vec<bool> {
    let n = forward.first.len() - 1;
    // Nodes in order of finishing a depth-first search over forward arcs.
    let mut seen = vec![false; n];
    let mut finished = Vec::with_capacity(n);
    let mut stack = Vec::<(u32, u32)>::new();
    for start in 0..n {
        if seen[start] {
            continue;
        }
        seen[start] = true;
        stack.push((start as u32, forward.first[start]));
        while let Some((node, next)) = stack.last_mut() {
            if *next < forward.first[*node as usize + 1] {
                let target = forward.arcs[*next as usize].0;
                *next += 1;
                if !seen[target as usize] {
                    seen[target as usize] = true;
                    stack.push((target, forward.first[target as usize]));
                }
            } else {
                finished.push(*node);
                stack.pop();
            }
        }
    }
    // Components over backward arcs, latest finished first.
    let mut component = vec![u32::MAX; n];
    let mut sizes = Vec::<usize>::new();
    let mut queue = VecDeque::new();
    for start in finished.into_iter().rev() {
        if component[start as usize] != u32::MAX {
            continue;
        }
        let id = sizes.len() as u32;
        let mut size = 0usize;
        component[start as usize] = id;
        queue.push_back(start);
        while let Some(node) = queue.pop_front() {
            size += 1;
            let range =
                backward.first[node as usize] as usize..backward.first[node as usize + 1] as usize;
            for (previous, _) in &backward.arcs[range] {
                if component[*previous as usize] == u32::MAX {
                    component[*previous as usize] = id;
                    queue.push_back(*previous);
                }
            }
        }
        sizes.push(size);
    }
    let largest = sizes
        .iter()
        .enumerate()
        .max_by_key(|(_, size)| **size)
        .map_or(0, |(id, _)| id as u32);
    component.iter().map(|id| *id == largest).collect()
}

/// A measured landmark: its node, the column's quantum, and per direction
/// (from, to) its distances over the contracted graph.
struct Landmark {
    node: u32,
    quantum: f32,
    distances: [Distances; 2],
}

impl Landmark {
    /// Partial distances once `job` is cancelled.
    fn new(graph: &ColumnGraph<'_>, node: u32, job: &Job<'_>) -> Self {
        let (from, to) = rayon::join(
            || graph.distances(node, 0, job),
            || graph.distances(node, 1, job),
        );
        Self {
            node,
            quantum: graph.quantum as f32,
            distances: [from, to],
        }
    }
}

/// Whether the stored bound is tight on the arc `u -> v` of cost `cost`: it
/// rises by the whole arc from the landmark, or falls by it towards the
/// landmark (saturated values are not exact distances).
#[inline]
fn is_tight(u: [u32; 2], v: [u32; 2], quantum: f64, cost: f32) -> bool {
    let exact = |value: u32| value < LANDMARK_SATURATED;
    let quanta = || arc_quanta(cost, quantum);
    let ([from_u, to_u], [from_v, to_v]) = (u, v);
    (exact(from_u) && exact(from_v) && from_v >= from_u && u64::from(from_v - from_u) == quanta())
        || (exact(to_u) && exact(to_v) && to_u >= to_v && u64::from(to_u - to_v) == quanta())
}

/// A measured candidate, without its tables: its node and a bitset over
/// `forward`'s arcs (in their order) of those its stored bounds are tight on.
/// Holding tables for every candidate at once would cost 8 bytes per node per
/// candidate.
struct Candidate {
    node: u32,
    tight: Vec<u64>,
}

impl Candidate {
    /// Not measured (and tight on nothing) once `job` is cancelled.
    fn new(graph: &ColumnGraph<'_>, node: u32, job: &Job<'_>) -> Self {
        let forward = graph.forward;
        let mut tight = vec![0u64; forward.arcs.len().div_ceil(64)];
        if job.cancelled() {
            return Self { node, tight };
        }
        let landmark = Landmark::new(graph, node, job);
        // Values derive from the junctions' on the fly: no table per node
        // (on worker threads, freed tables tend to stay resident).
        let [from, to] = &landmark.distances;
        let values = |node: usize| [graph.value(from, node, 0), graph.value(to, node, 1)];
        for u in 0..forward.first.len() - 1 {
            let at_u = values(u);
            for arc in forward.first[u] as usize..forward.first[u + 1] as usize {
                let (v, cost) = forward.arcs[arc];
                let at_v = values(v as usize);
                if is_tight(at_u, at_v, graph.quantum, cost) {
                    tight[arc / 64] |= 1 << (arc % 64);
                }
            }
        }
        Self { node, tight }
    }
}

/// Farthest node of `inside` in each of `count` angular sectors around its
/// centre: landmarks "behind" any target give the tightest bounds.
fn sector_candidates(input: &LandmarkInput<'_>, inside: &[bool], count: usize) -> Vec<u32> {
    let nodes = (0..input.node_count).filter(|node| inside[*node]);
    let (sum_lat, sum_lng, n) = nodes.clone().fold((0.0, 0.0, 0usize), |(a, b, n), node| {
        (a + input.node_lat[node], b + input.node_lng[node], n + 1)
    });
    if n == 0 {
        return Vec::new();
    }
    let (center_lat, center_lng) = (sum_lat / n as f64, sum_lng / n as f64);
    let cos_lat = math::cos(center_lat.to_radians());
    let mut best = vec![(f64::MIN, u32::MAX); count];
    for node in nodes {
        let dx = (input.node_lng[node] - center_lng) * cos_lat;
        let dy = input.node_lat[node] - center_lat;
        let angle = math::atan2(dy, dx) + std::f64::consts::PI;
        let sector = ((angle / std::f64::consts::TAU) * count as f64) as usize % count;
        let distance = dx * dx + dy * dy;
        if distance > best[sector].0 {
            best[sector] = (distance, node as u32);
        }
    }
    best.into_iter()
        .filter(|(_, node)| *node != u32::MAX)
        .map(|(_, node)| node)
        .collect()
}

/// Greedy maxCover: `count` of the (at most 64) candidates, each step taking
/// the one tight on the most arcs no kept landmark is tight on (ties to the
/// earlier candidate), in that order.
/// Fewer than `count` once `job` is cancelled.
fn max_cover(
    forward: &PackedArcs,
    candidates: &[Candidate],
    count: usize,
    job: &Job<'_>,
) -> Vec<usize> {
    debug_assert!(candidates.len() <= 64);
    // Per arc, the candidates tight on it.
    let mut masks = vec![0u64; forward.arcs.len()];
    masks
        .par_chunks_mut(64)
        .enumerate()
        .for_each(|(word, masks)| {
            for (index, candidate) in candidates.iter().enumerate() {
                let mut bits = candidate.tight[word];
                while bits != 0 {
                    masks[bits.trailing_zeros() as usize] |= 1 << index;
                    bits &= bits - 1;
                }
            }
        });
    let mut kept = 0u64;
    let mut order = Vec::with_capacity(count);
    while order.len() < count.min(candidates.len()) {
        if job.cancelled() {
            break;
        }
        let gain = masks
            .par_iter()
            .fold(
                || [0usize; 64],
                |mut gain, mask| {
                    if mask & kept == 0 {
                        let mut bits = *mask;
                        while bits != 0 {
                            gain[bits.trailing_zeros() as usize] += 1;
                            bits &= bits - 1;
                        }
                    }
                    gain
                },
            )
            .reduce(
                || [0usize; 64],
                |mut a, b| {
                    for (a, b) in a.iter_mut().zip(b) {
                        *a += b;
                    }
                    a
                },
            );
        let Some(best) = (0..candidates.len())
            .filter(|index| kept & (1 << index) == 0)
            .max_by(|a, b| gain[*a].cmp(&gain[*b]).then(b.cmp(a)))
        else {
            break;
        };
        kept |= 1 << best;
        order.push(best);
    }
    order
}

/// Sector candidates measured per column for `count` landmarks.
fn sector_count(count: usize) -> usize {
    (count * CANDIDATES_PER_LANDMARK).clamp(count, 64.max(count))
}

/// Units of landmark progress per column: the quantum, each candidate
/// measured, each kept landmark measured.
fn column_units(count: usize) -> usize {
    1 + sector_count(count) + count
}

/// The column's landmarks: sector candidates in its main component, measured
/// in parallel, then greedy maxCover, then the kept ones measured again for
/// their distances, with the contracted graph they are measured on (`None`
/// when the column has no usable arc). Reports [`column_units`] to
/// `progress`.
fn select_landmarks<'a>(
    input: &LandmarkInput<'_>,
    forward: &'a PackedArcs,
    backward: &'a PackedArcs,
    count: usize,
    job: &Job<'_>,
    progress: &UnitCounter<'_, '_>,
) -> Result<Option<(ColumnGraph<'a>, Vec<Landmark>)>, RoutingPackageError> {
    let check = || {
        if job.cancelled() {
            Err(cancelled_error())
        } else {
            Ok(())
        }
    };
    let inside = main_component(forward, backward);
    check()?;
    let sectors = sector_count(count);
    let nodes = sector_candidates(input, &inside, sectors);
    let Some(hub) = inside.iter().position(|inside| *inside) else {
        progress.add(column_units(count));
        return Ok(None);
    };
    drop(inside);
    let quantum = column_quantum(forward, backward, hub as u32, job);
    check()?;
    let graph = ColumnGraph::new(forward, backward, quantum)?;
    check()?;
    progress.add(1);
    if nodes.len() > 64 {
        // More landmarks than the masks hold: keep the sectors as they are.
        let landmarks: Vec<Landmark> = nodes
            .into_par_iter()
            .map(|node| {
                let landmark = Landmark::new(&graph, node, job);
                progress.add(1);
                landmark
            })
            .collect();
        check()?;
        return Ok(Some((graph, landmarks)));
    }
    let measured = nodes.len();
    let candidates = nodes
        .into_par_iter()
        .map(|node| {
            let candidate = Candidate::new(&graph, node, job);
            progress.add(1);
            candidate
        })
        .collect::<Vec<_>>();
    check()?;
    // Sectors with no node have no candidate.
    progress.add(sectors.saturating_sub(measured));
    let kept = max_cover(forward, &candidates, count, job);
    check()?;
    let landmarks = kept
        .into_par_iter()
        .map(|index| {
            let landmark = Landmark::new(&graph, candidates[index].node, job);
            progress.add(1);
            landmark
        })
        .collect::<Vec<_>>();
    check()?;
    progress.add(count.saturating_sub(landmarks.len()));
    Ok(Some((graph, landmarks)))
}

/// The column's quantum: `LANDMARK_MAX_FINITE` steps cover a distance from
/// or to a node of the main component up to the range quantile of exact
/// distances to `hub` plus that of distances from it (an upper bound on the
/// rest by the triangle inequality, short of a few pathological nodes behind
/// huge access penalties, whose distances are stored saturated). The lower
/// limit only keeps an empty range from giving a zero quantum.
fn column_quantum(forward: &PackedArcs, backward: &PackedArcs, hub: u32, job: &Job<'_>) -> f32 {
    let range = |mut finite: Vec<f64>| {
        finite.retain(|d| d.is_finite());
        if finite.is_empty() {
            return 0.0;
        }
        let rank = ((finite.len() - 1) as f64 * LANDMARK_RANGE_QUANTILE) as usize;
        *finite.select_nth_unstable_by(rank, f64::total_cmp).1
    };
    let (from, to) = rayon::join(
        || range(dijkstra(forward, hub, job)),
        || range(dijkstra(backward, hub, job)),
    );
    ((from + to) / LANDMARK_MAX_FINITE).max(1e-9) as f32
}

/// `floor(cost / quantum)`, less one where the division rounded up, so that
/// `units * quantum <= cost` holds in f64 (as the query evaluates bounds).
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

/// Stops within about a second once `job` is cancelled, returning an error.
pub(crate) fn build_landmarks(
    input: &LandmarkInput<'_>,
    column_count: usize,
    landmark_count: usize,
    anchored: bool,
    job: &Job<'_>,
) -> Result<LandmarkData, RoutingPackageError> {
    let progress = UnitCounter::new(
        job,
        CompileStage::Landmarks,
        column_count * column_units(landmark_count),
        "Computing landmarks",
    );
    job.report(CompileStage::Landmarks, Some(0.0), 0, "Computing landmarks");
    let n = input.node_count;
    let words = n.div_ceil(64);
    let mut data = LandmarkData {
        nodes: vec![0; column_count * landmark_count],
        scales: vec![0.0; column_count * landmark_count * 2],
        from: Vec::new(),
        to: Vec::new(),
        present: vec![0; column_count * words],
        rank: vec![0; column_count * words],
        row_count: 0,
        one_present: vec![0; column_count * words],
        one_rank: vec![0; column_count * words],
        one: Vec::new(),
        two_present: vec![0; column_count * words],
        two_rank: vec![0; column_count * words],
        two: Vec::new(),
        anchored: vec![(0, 0); column_count],
    };
    if landmark_count == 0 || n == 0 {
        return Ok(data);
    }
    // One column at a time (its candidates' Dijkstras run in parallel), its
    // rows written before the next, so that only one column's contracted
    // graph and distances are held at once.
    for column in 0..column_count {
        let (forward, backward) = rayon::join(
            || PackedArcs::new(input, column, true),
            || PackedArcs::new(input, column, false),
        );
        if job.cancelled() {
            return Err(cancelled_error());
        }
        let selected =
            select_landmarks(input, &forward, &backward, landmark_count, job, &progress)?;
        let Some((graph, landmarks)) = selected else {
            // No usable arc: every row is infinite. Ranks still count.
            write_ranks_only(&mut data, column, n);
            continue;
        };
        // Short selections are padded by repetition so the layout stays
        // fixed-width.
        let slots = (0..landmark_count)
            .filter_map(|index| landmarks.get(index).or(landmarks.first()))
            .collect::<Vec<_>>();
        for (index, landmark) in slots.iter().enumerate() {
            data.nodes[column * landmark_count + index] = landmark.node;
            for direction in 0..2 {
                data.scales[(column * landmark_count + index) * 2 + direction] = landmark.quantum;
            }
        }
        write_column(
            &mut data,
            column,
            n,
            landmark_count,
            &graph,
            &slots,
            anchored,
        )?;
    }
    if data.row_count > u32::MAX as usize {
        return Err(RoutingPackageError::Unsupported(format!(
            "{} landmark rows exceed u32 indexing",
            data.row_count
        )));
    }
    Ok(data)
}

const NONE: u32 = u32::MAX;

/// Contracted arcs in one direction: per junction its range (`first`) of
/// `(junction, quanta)`.
type JunctionArcs = (Vec<u32>, Vec<(u32, u64)>);

/// A column's landmark graph classified and contracted to its junctions.
///
/// On the undirected simple graph of usable arcs, repeatedly peeling nodes of
/// degree at most one leaves the 2-core; its nodes of degree three or more
/// are *junctions*. Removing the junctions splits the other nodes into
/// *pieces* (a dead-end tree, or a chain of degree-2 nodes with the trees
/// hanging off it), and every neighbour of a piece outside it is a junction:
/// its *anchors*, at most two. A path from outside a piece to a node in it
/// passes through an anchor last and then stays inside, so the floor
/// Dijkstra's `D(L, v) = min over anchors A of D(L, A) + d_piece(A, v)` (and
/// likewise towards the landmark) for every landmark `L` outside the piece,
/// clamping included (`derived_landmark_value`).
///
/// Distances between junctions run over the contracted graph: the arcs
/// between junctions, and per two-anchor piece a shortcut each way between
/// its anchors (the floor distance inside it). A source inside a piece seeds
/// its anchors from a Dijkstra inside its piece, and its own piece's values
/// are the least of the inside distance and the anchors'.
struct ColumnGraph<'a> {
    forward: &'a PackedArcs,
    backward: &'a PackedArcs,
    quantum: f64,
    /// Per node: its junction index, or `NONE`.
    junction: Vec<u32>,
    /// Per node: its piece, or `NONE` for a junction.
    piece: Vec<u32>,
    /// Per node: its position in its piece's members.
    position: Vec<u32>,
    /// Per piece: its members' range in `members`.
    piece_first: Vec<u32>,
    members: Vec<u32>,
    /// Per piece: its anchors' nodes, sorted (`NONE` for none).
    anchors: Vec<[u32; 2]>,
    /// Per node of a piece: `[anchor][direction]` floor distances inside the
    /// piece from the anchor (direction 0) and to it (1), clamped to
    /// `LANDMARK_SATURATED`; `LANDMARK_INFINITY` without a path.
    offsets: Vec<[[u32; 2]; 2]>,
    /// Contracted arcs per direction (forward, backward): per junction its
    /// range of `(junction, quanta)`.
    arcs: [JunctionArcs; 2],
}

/// One landmark's distances in one direction: per junction, and over the
/// landmark's own piece (`NONE` for a junction landmark); every other node's
/// derive from its anchors. Values are stored values: clamped, or
/// `LANDMARK_INFINITY`.
struct Distances {
    source_piece: u32,
    junction: Vec<u32>,
    /// Per member of the source's piece, in `members` order.
    local: Vec<u32>,
}

#[inline]
fn stored(distance: u64) -> u32 {
    if distance == u64::MAX {
        LANDMARK_INFINITY
    } else {
        distance.min(u64::from(LANDMARK_SATURATED)) as u32
    }
}

/// The distinct neighbours of `node` over the column's usable arcs in either
/// direction, itself excluded, into `out` (sorted).
fn neighbours(forward: &PackedArcs, backward: &PackedArcs, node: usize, out: &mut Vec<u32>) {
    out.clear();
    for arcs in [forward, backward] {
        let range = arcs.first[node] as usize..arcs.first[node + 1] as usize;
        out.extend(
            arcs.arcs[range]
                .iter()
                .map(|(next, _)| *next)
                .filter(|next| *next as usize != node),
        );
    }
    out.sort_unstable();
    out.dedup();
}

impl<'a> ColumnGraph<'a> {
    fn new(
        forward: &'a PackedArcs,
        backward: &'a PackedArcs,
        quantum: f32,
    ) -> Result<Self, RoutingPackageError> {
        let quantum = f64::from(quantum);
        let n = forward.first.len() - 1;
        let mut scratch = Vec::new();
        // Peel to the 2-core.
        let mut degree = (0..n)
            .map(|node| {
                neighbours(forward, backward, node, &mut scratch);
                scratch.len() as u32
            })
            .collect::<Vec<_>>();
        let mut removed = vec![false; n];
        let mut stack = (0..n as u32)
            .filter(|node| degree[*node as usize] <= 1)
            .collect::<Vec<_>>();
        while let Some(node) = stack.pop() {
            if removed[node as usize] {
                continue;
            }
            removed[node as usize] = true;
            neighbours(forward, backward, node as usize, &mut scratch);
            for next in &scratch {
                let next = *next as usize;
                if !removed[next] {
                    degree[next] -= 1;
                    if degree[next] == 1 {
                        stack.push(next as u32);
                    }
                }
            }
        }
        let mut junction = vec![NONE; n];
        let mut junction_count = 0u32;
        for node in 0..n {
            if !removed[node] && degree[node] >= 3 {
                junction[node] = junction_count;
                junction_count += 1;
            }
        }
        drop(removed);
        drop(degree);

        // Pieces, their members and anchors.
        let mut piece = vec![NONE; n];
        let mut position = vec![NONE; n];
        let mut piece_first = vec![0u32];
        let mut members = Vec::with_capacity(n - junction_count as usize);
        let mut anchors = Vec::new();
        let mut found = Vec::new();
        for start in 0..n {
            if junction[start] != NONE || piece[start] != NONE {
                continue;
            }
            let id = anchors.len() as u32;
            let first = members.len();
            found.clear();
            piece[start] = id;
            members.push(start as u32);
            let mut at = first;
            while at < members.len() {
                let node = members[at] as usize;
                position[node] = (at - first) as u32;
                at += 1;
                neighbours(forward, backward, node, &mut scratch);
                for next in &scratch {
                    let next = *next as usize;
                    if junction[next] != NONE {
                        found.push(next as u32);
                    } else if piece[next] == NONE {
                        piece[next] = id;
                        members.push(next as u32);
                    }
                }
            }
            found.sort_unstable();
            found.dedup();
            if found.len() > 2 {
                // Impossible by construction: a tree hangs off one core node,
                // a chain has two ends.
                return Err(RoutingPackageError::Unsupported(format!(
                    "a landmark graph piece has {} anchors",
                    found.len()
                )));
            }
            anchors.push([
                found.first().copied().unwrap_or(NONE),
                found.get(1).copied().unwrap_or(NONE),
            ]);
            piece_first.push(members.len() as u32);
        }

        // Offsets inside each piece, and the shortcuts between anchors.
        let mut offsets = vec![[[LANDMARK_INFINITY; 2]; 2]; n];
        let mut contracted = Vec::<(u32, u32, u64)>::new();
        let mut dist = Vec::new();
        let mut heap = RadixHeap::new();
        for (id, pair) in anchors.iter().enumerate() {
            let range = piece_first[id] as usize..piece_first[id + 1] as usize;
            for (slot, anchor) in pair.iter().enumerate() {
                if *anchor == NONE {
                    continue;
                }
                let other = pair[1 - slot];
                for (direction, arcs) in [forward, backward].into_iter().enumerate() {
                    let (inside, to_other) = inside_dijkstra(
                        arcs,
                        &piece,
                        &position,
                        id as u32,
                        range.len(),
                        *anchor,
                        other,
                        quantum,
                        &mut dist,
                        &mut heap,
                    );
                    for (index, member) in members[range.clone()].iter().enumerate() {
                        offsets[*member as usize][slot][direction] = stored(inside[index]);
                    }
                    // Forward from either anchor: the shortcut to the other.
                    if direction == 0 && other != NONE && to_other != u64::MAX {
                        contracted.push((
                            junction[*anchor as usize],
                            junction[other as usize],
                            to_other,
                        ));
                    }
                }
            }
        }
        // Arcs between junctions.
        for (node, index) in junction.iter().enumerate() {
            if *index == NONE {
                continue;
            }
            let range = forward.first[node] as usize..forward.first[node + 1] as usize;
            for (next, cost) in &forward.arcs[range] {
                let target = junction[*next as usize];
                if target != NONE && target != *index {
                    let quanta = arc_quanta(*cost, quantum).min(u64::from(LANDMARK_SATURATED));
                    contracted.push((*index, target, quanta));
                }
            }
        }
        let csr = |reverse: bool| {
            let mut first = vec![0u32; junction_count as usize + 1];
            for (from, to, _) in &contracted {
                first[if reverse { *to } else { *from } as usize + 1] += 1;
            }
            for index in 1..first.len() {
                first[index] += first[index - 1];
            }
            let mut next = first.clone();
            let mut out = vec![(0u32, 0u64); contracted.len()];
            for (from, to, quanta) in &contracted {
                let (at, target) = if reverse { (*to, *from) } else { (*from, *to) };
                out[next[at as usize] as usize] = (target, *quanta);
                next[at as usize] += 1;
            }
            (first, out)
        };
        let arcs = [csr(false), csr(true)];
        Ok(Self {
            forward,
            backward,
            quantum,
            junction,
            piece,
            position,
            piece_first,
            members,
            anchors,
            offsets,
            arcs,
        })
    }

    #[cfg(test)]
    fn node_count(&self) -> usize {
        self.junction.len()
    }

    fn junction_count(&self) -> usize {
        self.arcs[0].0.len() - 1
    }

    /// The source's distances in `direction` (0: from it over forward arcs,
    /// 1: to it, over backward arcs). Partial once `job` is cancelled.
    fn distances(&self, source: u32, direction: usize, job: &Job<'_>) -> Distances {
        let arcs = [self.forward, self.backward][direction];
        let mut label = vec![u64::MAX; self.junction_count()];
        let mut heap = RadixHeap::new();
        let source_piece = self.piece[source as usize];
        let mut inside = Vec::new();
        if source_piece == NONE {
            label[self.junction[source as usize] as usize] = 0;
        } else {
            let id = source_piece as usize;
            let len = (self.piece_first[id + 1] - self.piece_first[id]) as usize;
            let mut dist = Vec::new();
            let mut local_heap = RadixHeap::new();
            let anchors = self.anchors[id];
            let (seen, _) = inside_dijkstra(
                arcs,
                &self.piece,
                &self.position,
                source_piece,
                len,
                source,
                NONE,
                self.quantum,
                &mut dist,
                &mut local_heap,
            );
            inside = seen;
            // The anchors' labels: the least inside distance to a member
            // next to them, plus the arc.
            for anchor in anchors.into_iter().filter(|anchor| *anchor != NONE) {
                let best = self.distance_into_anchor(arcs, id, &inside, anchor);
                label[self.junction[anchor as usize] as usize] = best;
            }
        }
        // Seeds may go in any order: nothing has been popped yet.
        for (index, value) in label.iter().enumerate() {
            if *value != u64::MAX {
                heap.push(*value, index as u32);
            }
        }
        let (first, contracted) = &self.arcs[direction];
        let mut settled = 0usize;
        while let Some((d, node)) = heap.pop() {
            if d > label[node as usize] {
                continue;
            }
            settled += 1;
            if settled.is_multiple_of(CANCEL_CHECK_NODES) && job.cancelled() {
                break;
            }
            let range = first[node as usize] as usize..first[node as usize + 1] as usize;
            for (next, quanta) in &contracted[range] {
                let candidate = d + quanta;
                if candidate < label[*next as usize] {
                    label[*next as usize] = candidate;
                    heap.push(candidate, *next);
                }
            }
        }
        let junction = label.iter().map(|value| stored(*value)).collect::<Vec<_>>();
        let local = if source_piece == NONE {
            Vec::new()
        } else {
            let id = source_piece as usize;
            let range = self.piece_first[id] as usize..self.piece_first[id + 1] as usize;
            self.members[range]
                .iter()
                .zip(&inside)
                .map(|(member, inside)| {
                    let derived = self.derive(&junction, *member as usize, direction);
                    stored(*inside).min(derived)
                })
                .collect()
        };
        Distances {
            source_piece,
            junction,
            local,
        }
    }

    /// The least inside distance to `anchor` over the arcs into it from the
    /// members of piece `id` (`inside` per member, in `arcs`' direction).
    fn distance_into_anchor(
        &self,
        arcs: &PackedArcs,
        id: usize,
        inside: &[u64],
        anchor: u32,
    ) -> u64 {
        let range = self.piece_first[id] as usize..self.piece_first[id + 1] as usize;
        let mut best = u64::MAX;
        for (member, d) in self.members[range].iter().zip(inside) {
            if *d == u64::MAX {
                continue;
            }
            let arcs_range =
                arcs.first[*member as usize] as usize..arcs.first[*member as usize + 1] as usize;
            for (next, cost) in &arcs.arcs[arcs_range] {
                if *next == anchor {
                    let quanta = arc_quanta(*cost, self.quantum).min(u64::from(LANDMARK_SATURATED));
                    best = best.min(d + quanta);
                }
            }
        }
        best
    }

    /// The derived value at piece node `node` from its anchors' values.
    #[inline]
    fn derive(&self, junction: &[u32], node: usize, direction: usize) -> u32 {
        let anchors = self.anchors[self.piece[node] as usize];
        let offsets = &self.offsets[node];
        let value = |anchor: u32| {
            if anchor == NONE {
                LANDMARK_INFINITY
            } else {
                junction[self.junction[anchor as usize] as usize]
            }
        };
        derived_landmark_value(
            [value(anchors[0]), value(anchors[1])],
            [offsets[0][direction], offsets[1][direction]],
        )
    }

    /// The stored value at `node` of `distances` (measured in `direction`).
    #[inline]
    fn value(&self, distances: &Distances, node: usize, direction: usize) -> u32 {
        let index = self.junction[node];
        if index != NONE {
            return distances.junction[index as usize];
        }
        if self.piece[node] == distances.source_piece {
            return distances.local[self.position[node] as usize];
        }
        self.derive(&distances.junction, node, direction)
    }

    /// Every node's stored value.
    #[cfg(test)]
    fn expand(&self, distances: &Distances, direction: usize) -> Vec<u32> {
        (0..self.node_count())
            .map(|node| self.value(distances, node, direction))
            .collect()
    }
}

/// Floor Dijkstra from `source` (an anchor of piece `id`, or a member)
/// over `arcs` inside the piece: per member, in `position` order, the
/// distance (`u64::MAX` without a path), and the distance to `other` (an
/// anchor, reached but not expanded; `u64::MAX` for none). Only members are
/// expanded (and the source).
#[allow(clippy::too_many_arguments)]
fn inside_dijkstra(
    arcs: &PackedArcs,
    piece: &[u32],
    position: &[u32],
    id: u32,
    len: usize,
    source: u32,
    other: u32,
    quantum: f64,
    dist: &mut Vec<u64>,
    heap: &mut RadixHeap,
) -> (Vec<u64>, u64) {
    dist.clear();
    dist.resize(len, u64::MAX);
    let mut to_other = u64::MAX;
    *heap = RadixHeap::new();
    let source_member = piece[source as usize] == id;
    if source_member {
        dist[position[source as usize] as usize] = 0;
    }
    heap.push(0, source);
    while let Some((d, node)) = heap.pop() {
        let member = piece[node as usize] == id;
        if member && d > dist[position[node as usize] as usize] {
            continue;
        }
        if !member && node != source {
            continue;
        }
        let range = arcs.first[node as usize] as usize..arcs.first[node as usize + 1] as usize;
        for (next, cost) in &arcs.arcs[range] {
            let quanta = arc_quanta(*cost, quantum).min(u64::from(LANDMARK_SATURATED));
            let candidate = d + quanta;
            if piece[*next as usize] == id {
                let slot = &mut dist[position[*next as usize] as usize];
                if candidate < *slot {
                    *slot = candidate;
                    heap.push(candidate, *next);
                }
            } else if *next == other && member {
                to_other = to_other.min(candidate);
            }
        }
    }
    (dist.clone(), to_other)
}

/// What a (column, node)'s landmark distances become in the package.
#[derive(Clone, Copy, PartialEq, Eq)]
enum RowKind {
    Absent,
    Stored,
    One,
    Two,
}

/// A column with no usable arc: no row, ranks only.
fn write_ranks_only(data: &mut LandmarkData, column: usize, n: usize) {
    let words = n.div_ceil(64);
    for word in 0..words {
        data.rank[column * words + word] = data.row_count as u32;
        data.one_rank[column * words + word] = (data.one.len() / ONE_ANCHOR_WORDS) as u32;
        data.two_rank[column * words + word] = (data.two.len() / TWO_ANCHOR_WORDS) as u32;
    }
}

/// Writes the column's rows: a node in a piece with an anchor, no landmark
/// of the column and a path to or from an anchor derives from its anchors
/// (unless `anchored` is off); every other node with a finite distance
/// stores its row. A row finite for some landmarks and infinite for others is
/// an error: every landmark lies in the column's largest strongly connected
/// component, so it would mean the distances are wrong.
fn write_column(
    data: &mut LandmarkData,
    column: usize,
    n: usize,
    landmark_count: usize,
    graph: &ColumnGraph<'_>,
    slots: &[&Landmark],
    anchored: bool,
) -> Result<(), RoutingPackageError> {
    let words = n.div_ceil(64);
    let mut holds_landmark = vec![false; graph.anchors.len()];
    for landmark in slots {
        let piece = graph.piece[landmark.node as usize];
        if piece != NONE {
            holds_landmark[piece as usize] = true;
        }
    }
    // Per node: its anchors' slots in use (an anchor with no path either way
    // inside the piece drops out).
    let used = |node: usize| -> ([usize; 2], usize) {
        let mut slots = [0usize; 2];
        let mut count = 0;
        let piece = graph.piece[node];
        if !anchored || piece == NONE || holds_landmark[piece as usize] {
            return (slots, 0);
        }
        for (slot, anchor) in graph.anchors[piece as usize].iter().enumerate() {
            if *anchor != NONE && graph.offsets[node][slot] != [LANDMARK_INFINITY; 2] {
                slots[count] = slot;
                count += 1;
            }
        }
        (slots, count)
    };
    let first = slots.first().copied();
    let mut kind = vec![RowKind::Absent; n];
    let mut counts = (0usize, 0usize, 0usize);
    for (node, kind) in kind.iter_mut().enumerate() {
        let Some(first) = first else { break };
        let finite = (0..2).any(|direction| {
            graph.value(&first.distances[direction], node, direction) != LANDMARK_INFINITY
        });
        *kind = match (finite, used(node).1) {
            (false, _) => RowKind::Absent,
            (true, 0) => {
                counts.0 += 1;
                RowKind::Stored
            }
            (true, 1) => {
                counts.1 += 1;
                RowKind::One
            }
            (true, _) => {
                counts.2 += 1;
                RowKind::Two
            }
        };
    }
    data.from.reserve_exact(counts.0 * landmark_count);
    data.to.reserve_exact(counts.0 * landmark_count);
    data.one.reserve_exact(counts.1 * ONE_ANCHOR_WORDS);
    data.two.reserve_exact(counts.2 * TWO_ANCHOR_WORDS);
    // Stored rows' positions, for the anchors: two passes, since an anchor
    // may come after the nodes it anchors.
    let mut stored_row = vec![NONE; graph.junction_count()];
    let mut row = data.row_count;
    for (node, kind) in kind.iter().enumerate() {
        if *kind == RowKind::Stored {
            let index = graph.junction[node];
            if index != NONE {
                stored_row[index as usize] = row as u32;
            }
            row += 1;
        }
    }
    let anchor_row = |anchor: u32| {
        if anchor == NONE {
            NO_ANCHOR_ROW
        } else {
            stored_row[graph.junction[anchor as usize] as usize]
        }
    };
    for (node, kind) in kind.iter().enumerate() {
        let word = column * words + node / 64;
        if node % 64 == 0 {
            data.rank[word] = data.row_count as u32;
            data.one_rank[word] = (data.one.len() / ONE_ANCHOR_WORDS) as u32;
            data.two_rank[word] = (data.two.len() / TWO_ANCHOR_WORDS) as u32;
        }
        let bit = 1u64 << (node % 64);
        match kind {
            RowKind::Absent => {}
            RowKind::Stored => {
                data.present[word] |= bit;
                for (direction, out) in [&mut data.from, &mut data.to].into_iter().enumerate() {
                    let start = out.len();
                    out.extend(slots.iter().map(|landmark| {
                        graph.value(&landmark.distances[direction], node, direction)
                    }));
                    let infinite = out[start..]
                        .iter()
                        .filter(|value| **value == LANDMARK_INFINITY)
                        .count();
                    if infinite != 0 && infinite != landmark_count {
                        return Err(RoutingPackageError::Unsupported(format!(
                            "the landmark row of node {node} in column {column} is infinite for \
                             {infinite} of {landmark_count} landmarks"
                        )));
                    }
                }
                data.row_count += 1;
            }
            RowKind::One | RowKind::Two => {
                let (used_slots, count) = used(node);
                let anchors = graph.anchors[graph.piece[node] as usize];
                let offsets = graph.offsets[node];
                if count == 1 {
                    data.one_present[word] |= bit;
                    let slot = used_slots[0];
                    data.one.extend([
                        anchor_row(anchors[slot]),
                        offsets[slot][0],
                        offsets[slot][1],
                    ]);
                } else {
                    data.two_present[word] |= bit;
                    let (a, b) = (used_slots[0], used_slots[1]);
                    data.two.extend([
                        anchor_row(anchors[a]),
                        anchor_row(anchors[b]),
                        offsets[a][0],
                        offsets[a][1],
                        offsets[b][0],
                        offsets[b][1],
                    ]);
                }
            }
        }
    }
    data.anchored[column] = (counts.1, counts.2);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The floor Dijkstra over every node, as the compiler ran it before the
    /// contracted graph: the reference.
    fn full_dijkstra(arcs: &PackedArcs, source: u32, quantum: f64) -> Vec<u32> {
        let mut dist = vec![u64::MAX; arcs.first.len() - 1];
        let mut heap = std::collections::BinaryHeap::new();
        dist[source as usize] = 0;
        heap.push(std::cmp::Reverse((0u64, source)));
        while let Some(std::cmp::Reverse((d, node))) = heap.pop() {
            if d > dist[node as usize] {
                continue;
            }
            let range = arcs.first[node as usize] as usize..arcs.first[node as usize + 1] as usize;
            for (next, cost) in &arcs.arcs[range] {
                let candidate = d + arc_quanta(*cost, quantum).min(u64::from(LANDMARK_SATURATED));
                if candidate < dist[*next as usize] {
                    dist[*next as usize] = candidate;
                    heap.push(std::cmp::Reverse((candidate, *next)));
                }
            }
        }
        dist.into_iter().map(stored).collect()
    }

    /// A small deterministic graph with junctions, chains, trees hanging off
    /// chains and junctions, a chain that rejoins its junction, one-way
    /// arcs, parallel arcs, an arc whose quanta saturate, an infinite cost
    /// and a separate component.
    fn graph(seed: u64) -> (usize, Vec<(u32, u32, f32)>) {
        let mut state = seed;
        let mut random = move |below: u64| {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (state >> 33) % below
        };
        let mut edges = Vec::new();
        let both = |edges: &mut Vec<(u32, u32, f32)>, a: u32, b: u32, cost: f32, oneway: bool| {
            edges.push((a, b, cost));
            if !oneway {
                edges.push((b, a, cost * 1.25));
            }
        };
        // A 4 x 4 grid of junctions (0..16).
        for row in 0..4u32 {
            for col in 0..4u32 {
                let node = row * 4 + col;
                if col < 3 {
                    both(&mut edges, node, node + 1, 10.0 + random(50) as f32, false);
                }
                if row < 3 {
                    both(
                        &mut edges,
                        node,
                        node + 4,
                        10.0 + random(50) as f32,
                        random(5) == 0,
                    );
                }
            }
        }
        let mut next = 16u32;
        // Chains between random junctions, some one-way, with trees.
        for _ in 0..12 {
            let (a, b) = (random(16) as u32, random(16) as u32);
            let len = 1 + random(4) as u32;
            let oneway = random(4) == 0;
            let mut previous = a;
            for _ in 0..len {
                both(&mut edges, previous, next, 1.0 + random(20) as f32, oneway);
                if random(3) == 0 {
                    // A tree of two nodes off the chain node.
                    both(
                        &mut edges,
                        next,
                        next + 1,
                        2.0 + random(9) as f32,
                        random(4) == 0,
                    );
                    both(
                        &mut edges,
                        next + 1,
                        next + 2,
                        2.0 + random(9) as f32,
                        false,
                    );
                    previous = next;
                    next += 3;
                } else {
                    previous = next;
                    next += 1;
                }
            }
            both(&mut edges, previous, b, 1.0 + random(20) as f32, oneway);
        }
        // Trees off junctions.
        for _ in 0..6 {
            let root = random(16) as u32;
            both(
                &mut edges,
                root,
                next,
                3.0 + random(9) as f32,
                random(5) == 0,
            );
            both(&mut edges, next, next + 1, 3.0 + random(9) as f32, false);
            both(&mut edges, next, next + 2, 3.0 + random(9) as f32, false);
            next += 3;
        }
        // A parallel arc, a saturating arc, an infinite cost.
        both(&mut edges, 0, 1, 3.0, true);
        both(&mut edges, 15, next, 1e9, false);
        both(&mut edges, next, next + 1, f32::INFINITY, false);
        next += 2;
        // A separate component: a short road.
        both(&mut edges, next, next + 1, 5.0, false);
        next += 2;
        (next as usize, edges)
    }

    fn packed(n: usize, edges: &[(u32, u32, f32)], forward: bool) -> PackedArcs {
        let mut first = vec![0u32; n + 1];
        let mut arcs = Vec::new();
        for node in 0..n as u32 {
            for (from, to, cost) in edges {
                let (at, target) = if forward { (*from, *to) } else { (*to, *from) };
                if at == node && cost.is_finite() {
                    arcs.push((target, *cost));
                }
            }
            first[node as usize + 1] = arcs.len() as u32;
        }
        PackedArcs { first, arcs }
    }

    fn empty_data(columns: usize, n: usize) -> LandmarkData {
        let words = n.div_ceil(64);
        LandmarkData {
            nodes: Vec::new(),
            scales: Vec::new(),
            from: Vec::new(),
            to: Vec::new(),
            present: vec![0; columns * words],
            rank: vec![0; columns * words],
            row_count: 0,
            one_present: vec![0; columns * words],
            one_rank: vec![0; columns * words],
            one: Vec::new(),
            two_present: vec![0; columns * words],
            two_rank: vec![0; columns * words],
            two: Vec::new(),
            anchored: vec![(0, 0); columns],
        }
    }

    fn test_job<T>(run: impl FnOnce(&Job<'_>) -> T) -> T {
        let never = || false;
        let mut ignore = |_: &crate::job::Report| {};
        run(&Job::new(&never, &mut ignore))
    }

    /// Two landmarks in separate components: a row is finite for one and
    /// infinite for the other, which `write_column` refuses (it would mean
    /// a landmark outside the column's main component).
    #[test]
    fn a_partly_infinite_row_fails_the_build() {
        // Two triangles, both ways round: no junction, so every row is stored.
        let mut edges = Vec::new();
        for base in [0u32, 3] {
            for (a, b) in [(0, 1), (1, 2), (2, 0)] {
                edges.push((base + a, base + b, 1.0f32));
                edges.push((base + b, base + a, 1.0f32));
            }
        }
        let (forward, backward) = (packed(6, &edges, true), packed(6, &edges, false));
        let graph = ColumnGraph::new(&forward, &backward, 1.0).unwrap();
        let landmarks = test_job(|job| [0, 3].map(|node| Landmark::new(&graph, node, job)));
        let mut data = empty_data(1, 6);
        assert!(matches!(
            write_column(&mut data, 0, 6, 2, &graph, &[&landmarks[0], &landmarks[1]], false),
            Err(RoutingPackageError::Unsupported(message))
                if message.contains("node 0 in column 0") && message.contains("1 of 2")
        ));
    }

    /// Rows are compacted: a node unreachable from and to every landmark
    /// stores nothing, a row infinite in one direction only stays, ranks
    /// count rows of earlier columns, and with anchored rows the piece nodes
    /// keep a record instead of a row.
    #[test]
    fn rows_are_compacted_across_columns() {
        const I: u32 = LANDMARK_INFINITY;
        // A complete graph on 0-3 (both ways, cost 2: four junctions), node 4
        // a dead end reached one way from 0 (cost 3), node 5 a dead end both
        // ways to 1 (cost 1), and 6 <-> 7 a separate component.
        let mut edges = Vec::new();
        for a in 0..4u32 {
            for b in 0..4u32 {
                if a != b {
                    edges.push((a, b, 2.0f32));
                }
            }
        }
        edges.extend([
            (0, 4, 3.0),
            (1, 5, 1.0),
            (5, 1, 1.0),
            (6, 7, 1.0),
            (7, 6, 1.0),
        ]);
        let n = 8;
        let (forward, backward) = (packed(n, &edges, true), packed(n, &edges, false));
        let graph = ColumnGraph::new(&forward, &backward, 1.0).unwrap();
        assert_eq!(graph.junction_count(), 4);
        let landmarks = test_job(|job| [2, 3].map(|node| Landmark::new(&graph, node, job)));
        let slots = [&landmarks[0], &landmarks[1]];
        // Column 0 without anchored rows, column 1 with.
        let mut data = empty_data(2, n);
        write_column(&mut data, 0, n, 2, &graph, &slots, false).unwrap();
        write_column(&mut data, 1, n, 2, &graph, &slots, true).unwrap();
        // Column 0: nodes 0-5 stored, 6 and 7 absent.
        assert_eq!(data.present[0], 0b11_1111);
        // Column 1: only the junctions; 4 and 5 hang off one anchor each.
        assert_eq!(data.present[1], 0b1111);
        assert_eq!(data.one_present, [0, 0b11_0000]);
        assert_eq!(data.two_present, [0, 0]);
        assert_eq!(data.rank, [0, 6]);
        assert_eq!(data.one_rank, [0, 0]);
        assert_eq!(data.row_count, 10);
        assert_eq!(data.anchored, [(0, 0), (2, 0)]);
        // Per row, (landmark 2, landmark 3) in whole quanta of 1.0: column
        // 0's nodes 0-5, then column 1's junctions 0-3.
        let junctions = [2, 2, 2, 2, 0, 2, 2, 0];
        assert_eq!(data.from[..8], junctions);
        assert_eq!(data.from[8..12], [5, 5, 3, 3]);
        assert_eq!(data.from[12..], junctions);
        assert_eq!(data.to[..8], junctions);
        assert_eq!(data.to[8..12], [I, I, 3, 3]);
        assert_eq!(data.to[12..], junctions);
        // Node 4: anchor 0 (column 1's first row, 6), 3 quanta from it,
        // none back; node 5: anchor 1 (row 7), 1 quantum each way.
        assert_eq!(data.one, [6, 3, I, 7, 1, 1]);
    }

    #[test]
    fn contracted_distances_equal_the_full_floor_dijkstra() {
        let never = || false;
        let mut ignore = |_: &crate::job::Report| {};
        let job = Job::new(&never, &mut ignore);
        for seed in 0..20 {
            let (n, edges) = graph(seed);
            let (forward, backward) = (packed(n, &edges, true), packed(n, &edges, false));
            // A coarse quantum so that some arcs floor to zero quanta.
            let quantum = 0.7f32;
            let graph = ColumnGraph::new(&forward, &backward, quantum).unwrap();
            assert!(graph.junction_count() >= 8, "seed {seed}");
            for source in 0..n as u32 {
                for (direction, arcs) in [&forward, &backward].into_iter().enumerate() {
                    let expected = full_dijkstra(arcs, source, f64::from(quantum));
                    let distances = graph.distances(source, direction, &job);
                    assert_eq!(
                        graph.expand(&distances, direction),
                        expected,
                        "seed {seed}, source {source}, direction {direction}"
                    );
                }
            }
        }
    }
}

//! Many-to-many matrices with buckets: one bounded backward search per
//! target stores its labels in per-edge buckets, then one forward search per
//! source scans the buckets of every edge it relaxes and joins the two halves
//! at that transition.
//!
//! Semantics are the route engine's bidirectional search (`package/search.rs`)
//! with zero potentials, so a cell equals the pairwise cell (and the route)
//! between the same points:
//!
//! - forward labels on edges carry the cost to the end of the edge, seeded on
//!   the partially traversed origin edges; backward labels carry the cost from
//!   entering the edge to the destination point, seeded on the destination
//!   edges' partial costs;
//! - labels are kept per edge and complex-restriction state on both sides
//!   (see `package::restrictions`), so both halves are exact under complex
//!   restrictions, and a backward chain never runs over its own destination
//!   edge before finishing on it;
//! - a forward label on `a` and a backward label on `b` join across the
//!   transition `a -> b` with its turn cost (none from an untraversed origin
//!   straight onto a destination edge), after the forward side's restriction
//!   check; the joined path is re-checked for a repeated destination edge and
//!   complex restrictions spanning the join;
//! - an origin and destination on the same edge, the destination ahead, take
//!   the direct path and are not searched.
//!
//! **Exactness.** A backward search stops at a radius `R(t)`: every backward
//! label cheaper than `R(t)` is settled and every predecessor of a settled
//! label is labelled (frontier labels are in the buckets too, as upper
//! bounds). On a shortest path `P` of cost `D`, take the last forward-settled
//! edge `e_i`; when the forward queue has reached `p`, the edge after next,
//! `e_{i+2}`, costs at most `D - p` to the destination, so once
//! `p + R(t) > D` it is backward-settled, `e_{i+1}` is labelled with its exact
//! suffix cost, and the join `e_i -> e_{i+1}` was scanned when `e_i` settled.
//! A target is therefore final once the forward queue reaches
//! `best(t) - R(t)` and its best is no dearer than every origin seed not yet
//! settled (a path through one costs at least the seed), and a search that
//! exhausts its queue is final for every target.
//!
//! The last open targets of a row (usually far or penalised ones, which would
//! otherwise make every row flood the region) are handed to the pairwise cell
//! search instead. (A* towards the open targets' balls on landmark bounds was
//! tried for them and cost more CPU in bound evaluations than it saved.)
//!
//! Both halves search (edge, restriction state) labels, so the argument
//! above holds label by label under complex restrictions, and a cell equals
//! the exact pairwise cell.
//!
//! The radius trades backward work and bucket memory against forward work:
//! each target's search runs to a fraction of its farthest source's
//! straight-line lower bound, capped by a per-target share of a global label
//! budget.

use std::collections::BinaryHeap;

use rayon::prelude::*;

use super::analysis::{
    AnalysisCounters, AnalysisPath, AnalysisPoint, AnalysisQuery, direct_same_edge,
};
use super::{HeapEntry, NO_LABEL, SearchState, tie_window};
use crate::package::restrictions::{ComplexAutomata, ROOT};
use crate::package::{
    EDGE_FLAG_COMPLEX_RESTRICTION_TO, EDGE_FLAG_SIMPLE_RESTRICTION_FROM, RoutingPackage,
    SearchGraph, chord_m, unit_xyz,
};
use crate::types::CorrelatedLocation;

const SEED_TRAVERSED_FRACTION_EPSILON: f64 = 1e-9;

/// Backward labels kept over all targets (bucket memory is about 48 bytes
/// per label). Each target's search stops at its share.
const DEFAULT_LABEL_BUDGET: usize = 8_000_000;
/// Backward radius as a fraction of the farthest source's straight-line
/// lower bound.
const DEFAULT_RADIUS_FACTOR: f64 = 0.7;
/// Open targets a forward search hands to cell searches.
const DEFAULT_STRAGGLERS: usize = 10;

/// Tuning knobs, from the environment for measurements.
#[derive(Debug, Clone, Copy)]
pub(crate) struct ManyToManyTuning {
    pub radius_factor: f64,
    pub label_budget: usize,
    /// A forward search hands its last this many open targets to
    /// point-to-point cell searches.
    pub stragglers: usize,
}

impl ManyToManyTuning {
    pub(crate) fn from_env() -> Self {
        let parse = |name: &str| {
            std::env::var(name)
                .ok()
                .and_then(|value| value.parse().ok())
        };
        Self {
            radius_factor: parse("WAYFARER_M2M_RADIUS_FACTOR").unwrap_or(DEFAULT_RADIUS_FACTOR),
            label_budget: parse("WAYFARER_M2M_LABEL_BUDGET")
                .map(|value: f64| value as usize)
                .unwrap_or(DEFAULT_LABEL_BUDGET),
            stragglers: parse("WAYFARER_M2M_STRAGGLERS")
                .map(|value: f64| value as usize)
                .unwrap_or(DEFAULT_STRAGGLERS),
        }
    }
}

#[derive(Clone, Copy)]
struct BackwardLabel {
    edge: u32,
    next: u32,
    cost: f64,
    destination: u16,
    /// Backward complex-restriction state of the chain from `edge` on.
    state: u32,
}

/// One target's backward search, frozen.
struct TargetTree {
    labels: Vec<BackwardLabel>,
    /// Every label cheaper than this is settled (infinite: exhausted).
    radius: f64,
    /// The current label of every labelled edge and restriction state.
    current: Vec<(u32, u32)>,
    counters: AnalysisCounters,
}

/// Per-edge buckets over all target trees, as CSR arrays.
struct Buckets {
    offsets: Vec<u32>,
    target: Vec<u32>,
    label: Vec<u32>,
    cost: Vec<f64>,
}

/// One source row of results.
pub(crate) struct ManyToManyRow {
    pub paths: Vec<Option<AnalysisPath>>,
    pub budget_exceeded: Vec<bool>,
    pub counters: AnalysisCounters,
    /// Bucket entries scanned.
    pub scanned: usize,
}

pub(crate) struct ManyToManyInput<'a> {
    pub sources: &'a [&'a [CorrelatedLocation]],
    pub source_points: &'a [Vec<AnalysisPoint>],
    pub targets: &'a [&'a [CorrelatedLocation]],
    pub target_points: &'a [Vec<AnalysisPoint>],
    /// `connected[source][target]`: cells across weak components are not
    /// searched.
    pub connected: &'a [Vec<bool>],
}

/// Every cell of the matrix. Rows are `None` for sources without
/// candidates; the second value counts the backward searches' work.
pub(crate) fn many_to_many(
    compact: &RoutingPackage,
    graph: SearchGraph<'_>,
    query: &AnalysisQuery<'_>,
    heuristic_cost_per_m: f64,
    input: &ManyToManyInput<'_>,
    tuning: ManyToManyTuning,
) -> (Vec<Option<ManyToManyRow>>, AnalysisCounters) {
    let started = std::time::Instant::now();
    let source_count = input.sources.len();
    let target_count = input.targets.len();
    let searched_targets = (0..target_count)
        .filter(|target| !input.target_points[*target].is_empty())
        .collect::<Vec<_>>();
    // Backward radius per target: a fraction of the farthest connected
    // source's straight-line lower bound.
    let points = |locations: &[CorrelatedLocation]| {
        locations
            .iter()
            .map(|location| unit_xyz(location.snapped_point.lat, location.snapped_point.lng))
            .collect::<Vec<_>>()
    };
    let source_xyz = input.sources.iter().map(|s| points(s)).collect::<Vec<_>>();
    let share = (tuning.label_budget / searched_targets.len().max(1)).max(1);
    let trees = searched_targets
        .par_iter()
        .map(|target| {
            let target_xyz = points(input.targets[*target]);
            let farthest = (0..source_count)
                .filter(|source| input.connected[*source][*target])
                .flat_map(|source| {
                    source_xyz[source].iter().flat_map(|from| {
                        target_xyz
                            .iter()
                            .map(move |to| chord_m(*from, *to) * heuristic_cost_per_m)
                    })
                })
                .fold(0.0f64, f64::max);
            let mut state = compact.take_state();
            let tree = backward_tree(
                graph,
                &mut state,
                query,
                input.targets[*target],
                &input.target_points[*target],
                farthest * tuning.radius_factor,
                share,
            );
            compact.return_state(state);
            tree
        })
        .collect::<Vec<_>>();
    let mut backward = AnalysisCounters::default();
    for tree in &trees {
        backward.visited_edge_count += tree.counters.visited_edge_count;
        backward.settled_edge_count += tree.counters.settled_edge_count;
    }
    let mut tree_of = vec![u32::MAX; target_count];
    for (slot, target) in searched_targets.iter().enumerate() {
        tree_of[*target] = slot as u32;
    }
    let backward_ms = started.elapsed().as_secs_f64() * 1e3;
    let buckets = build_buckets(graph.edge_count, &trees);

    let rows = (0..source_count)
        .into_par_iter()
        .map(|source| {
            if input.source_points[source].is_empty() {
                return None;
            }
            let mut state = compact.take_state();
            let row = forward_row(
                graph,
                &mut state,
                query,
                heuristic_cost_per_m,
                input,
                source,
                &trees,
                &tree_of,
                &buckets,
                tuning,
            );
            compact.return_state(state);
            Some(row)
        })
        .collect::<Vec<Option<ManyToManyRow>>>();
    if crate::debug::trace() {
        let (mut visited, mut settled, mut scanned) = (0, 0, 0);
        for row in rows.iter().flatten() {
            visited += row.counters.visited_edge_count;
            settled += row.counters.settled_edge_count;
            scanned += row.scanned;
        }
        let exhausted = trees
            .iter()
            .filter(|tree| tree.radius.is_infinite())
            .count();
        eprintln!(
            "m2m: {} trees ({exhausted} exhausted), backward visited {} settled {} in {backward_ms:.0} ms, {} bucket entries, forward visited {visited} settled {settled} scanned {scanned}, total {:.0} ms",
            trees.len(),
            backward.visited_edge_count,
            backward.settled_edge_count,
            buckets.target.len(),
            started.elapsed().as_secs_f64() * 1e3
        );
    }
    (rows, backward)
}

fn build_buckets(edge_count: usize, trees: &[TargetTree]) -> Buckets {
    let mut offsets = vec![0u32; edge_count + 1];
    for tree in trees {
        for (edge, _) in &tree.current {
            offsets[*edge as usize + 1] += 1;
        }
    }
    for index in 0..edge_count {
        offsets[index + 1] += offsets[index];
    }
    let total = offsets[edge_count] as usize;
    let mut cursor = offsets.clone();
    let (mut target, mut label, mut cost) =
        (vec![0u32; total], vec![0u32; total], vec![0.0; total]);
    for (slot, tree) in trees.iter().enumerate() {
        for (edge, index) in &tree.current {
            let at = &mut cursor[*edge as usize];
            target[*at as usize] = slot as u32;
            label[*at as usize] = *index;
            cost[*at as usize] = tree.labels[*index as usize].cost;
            *at += 1;
        }
    }
    Buckets {
        offsets,
        target,
        label,
        cost,
    }
}

/// Whether a simple restriction forbids `from -> to` for this profile.
#[inline]
fn simple_restricted(graph: &SearchGraph<'_>, profile_bit: u8, from: u32, to: u32) -> bool {
    graph.edge_flags[from as usize] & EDGE_FLAG_SIMPLE_RESTRICTION_FROM != 0
        && graph.simple_restricted(from, to, profile_bit)
}

/// Backward Dijkstra from one target's candidates up to `radius`, or until
/// `max_settled` labels are settled.
fn backward_tree(
    graph: SearchGraph<'_>,
    state: &mut SearchState,
    query: &AnalysisQuery<'_>,
    destinations: &[CorrelatedLocation],
    points: &[AnalysisPoint],
    radius: f64,
    max_settled: usize,
) -> TargetTree {
    if state.b_edge_label.len() != graph.edge_count {
        state.b_edge_label = vec![NO_LABEL; graph.edge_count];
    }
    let automata = graph.complex_automata(query.profile_bit);
    let mut labels = Vec::<BackwardLabel>::new();
    let mut heap = std::mem::take(&mut state.b_heap);
    heap.clear();
    let mut seq = 0u32;
    let mut counters = AnalysisCounters::default();
    let mut push = |state: &mut SearchState,
                    labels: &mut Vec<BackwardLabel>,
                    heap: &mut BinaryHeap<HeapEntry>,
                    label: BackwardLabel| {
        let slot = state.b_slot(label.edge, label.state);
        if slot != NO_LABEL && labels[slot as usize].cost <= label.cost {
            return;
        }
        let index = labels.len() as u32;
        state.set_b_slot(label.edge, label.state, index);
        seq = seq.wrapping_add(1);
        heap.push(HeapEntry {
            priority: label.cost,
            seq,
            label: index,
        });
        labels.push(label);
    };
    for (index, (_, point)) in destinations.iter().zip(points).enumerate() {
        let cost = f64::from(query.edge_cost(graph, point.edge));
        if !cost.is_finite() {
            continue;
        }
        push(
            state,
            &mut labels,
            &mut heap,
            BackwardLabel {
                edge: point.edge,
                next: NO_LABEL,
                cost: cost * point.fraction.clamp(0.0, 1.0) + point.approach,
                destination: index as u16,
                // A one-edge chain never starts a whole forbidden sequence.
                state: automata
                    .backward_prepend(&graph, ROOT, point.edge)
                    .unwrap_or(ROOT),
            },
        );
    }
    let mut reached = f64::INFINITY;
    while let Some(entry) = heap.pop() {
        let label = labels[entry.label as usize];
        if state.b_slot(label.edge, label.state) != entry.label {
            continue;
        }
        if entry.priority >= radius
            || counters.settled_edge_count >= max_settled
            || query
                .cancellation
                .interrupted_at(counters.settled_edge_count)
        {
            reached = entry.priority;
            break;
        }
        counters.settled_edge_count += 1;
        let node = graph.edge_source(label.edge as usize);
        let own_destination = points[usize::from(label.destination)].edge;
        for (previous, _) in graph.in_edges(node as usize) {
            counters.visited_edge_count += 1;
            let edge_cost = f64::from(query.edge_cost(graph, previous));
            if !edge_cost.is_finite() {
                continue;
            }
            if simple_restricted(&graph, query.profile_bit, previous, label.edge) {
                continue;
            }
            let Some(next_state) = automata.backward_prepend(&graph, label.state, previous) else {
                continue;
            };
            // A path finishes on its destination edge only once.
            if previous == own_destination {
                continue;
            }
            let turn = graph.turn_cost(previous, label.edge);
            // A SAR barrier may close the transition.
            if !turn.is_finite() {
                continue;
            }
            let cost = label.cost + edge_cost + turn;
            push(
                state,
                &mut labels,
                &mut heap,
                BackwardLabel {
                    edge: previous,
                    next: entry.label,
                    cost,
                    destination: label.destination,
                    state: next_state,
                },
            );
        }
    }
    let mut current = state
        .b_touched
        .iter()
        .map(|edge| (*edge, state.b_edge_label[*edge as usize]))
        .collect::<Vec<_>>();
    let mut others = state.b_edge_extra.all().collect::<Vec<_>>();
    others.sort_unstable();
    current.extend(others);
    heap.clear();
    state.b_heap = heap;
    TargetTree {
        labels,
        radius: reached,
        current,
        counters,
    }
}

#[derive(Clone, Copy)]
struct ForwardLabel {
    edge: u32,
    pred: u32,
    cost: f64,
    origin: u16,
    traversed: bool,
    /// Forward complex-restriction state of the traversed path.
    state: u32,
}

#[derive(Clone, Copy)]
struct Meeting {
    forward: u32,
    tree: u32,
    backward: u32,
    cost: f64,
}

struct Forward<'a, 'g> {
    g: SearchGraph<'g>,
    q: &'a AnalysisQuery<'a>,
    automata: &'g ComplexAutomata,
    labels: Vec<ForwardLabel>,
}

impl Forward<'_, '_> {
    #[inline]
    fn transition_restricted(&self, state: u32, last_traversed: Option<u32>, next: u32) -> bool {
        if let Some(previous) = last_traversed
            && self.g.simple_restricted(previous, next, self.q.profile_bit)
        {
            return true;
        }
        self.automata.forbids(state, next)
    }

    fn traversed_edges(&self, label: u32) -> Vec<u32> {
        let mut edges = Vec::new();
        let mut current = label;
        while current != NO_LABEL {
            let entry = self.labels[current as usize];
            if entry.traversed {
                edges.push(entry.edge);
            }
            current = entry.pred;
        }
        edges.reverse();
        edges
    }

    /// Whether joined path `edges` (forward part from `forward`) repeats its
    /// destination edge; the origin seed's piece of it counts only where the
    /// two overlap (see `search::Run::finish_repeats`).
    fn finish_repeats(
        &self,
        forward: u32,
        edges: &[u32],
        origins: &[AnalysisPoint],
        destination: AnalysisPoint,
    ) -> bool {
        let Some((last, before)) = edges.split_last() else {
            return false;
        };
        let mut seed = forward;
        while self.labels[seed as usize].pred != NO_LABEL {
            seed = self.labels[seed as usize].pred;
        }
        let seed = self.labels[seed as usize];
        before.iter().enumerate().any(|(index, edge)| {
            edge == last
                && (index > 0
                    || !seed.traversed
                    || origins[usize::from(seed.origin)].fraction.clamp(0.0, 1.0)
                        <= destination.fraction.clamp(0.0, 1.0))
        })
    }

    fn joined_edges(&self, forward: u32, tree: &TargetTree, backward: u32) -> Vec<u32> {
        let mut edges = self.traversed_edges(forward);
        let mut chain = backward;
        while chain != NO_LABEL {
            let label = tree.labels[chain as usize];
            edges.push(label.edge);
            chain = label.next;
        }
        edges
    }

    fn sequence_violates_complex(&self, edges: &[u32]) -> bool {
        let g = &self.g;
        edges.iter().enumerate().skip(1).any(|(index, edge)| {
            if g.edge_flags[*edge as usize] & EDGE_FLAG_COMPLEX_RESTRICTION_TO == 0 {
                return false;
            }
            let start = g.complex_to.partition_point(|to| *to < *edge);
            (start..g.complex_to.len())
                .take_while(|row| g.complex_to[*row] == *edge)
                .any(|row| {
                    g.complex_mask[row] & self.q.profile_bit != 0 && {
                        let prefix = &g.complex_prefix_pool[g.complex_prefix_start[row] as usize
                            ..g.complex_prefix_start[row + 1] as usize];
                        edges[..index].ends_with(prefix)
                    }
                })
        })
    }

    fn path(
        &self,
        meeting: &Meeting,
        tree: &TargetTree,
        origins: &[AnalysisPoint],
        destinations: &[AnalysisPoint],
    ) -> AnalysisPath {
        let g = &self.g;
        let column = self.q.column;
        let (mut distance_m, mut duration_s) = (0.0, 0.0);
        let mut approach = 0.0;
        let mut current = meeting.forward;
        while current != NO_LABEL {
            let entry = self.labels[current as usize];
            let fraction = if entry.pred == NO_LABEL {
                approach += origins[usize::from(entry.origin)].approach;
                (1.0 - origins[usize::from(entry.origin)].fraction).clamp(0.0, 1.0)
            } else {
                // Where the search added a turn cost.
                let pred = self.labels[entry.pred as usize].edge;
                duration_s += g.junction_delay(&column, pred, entry.edge);
                1.0
            };
            distance_m += f64::from(g.edge_length[entry.edge as usize]) * fraction;
            duration_s += f64::from(column.duration(entry.edge)) * fraction;
            current = entry.pred;
        }
        // The join, as `forward_row` costs it.
        let (f, b) = (
            self.labels[meeting.forward as usize],
            tree.labels[meeting.backward as usize],
        );
        if f.traversed || b.next != NO_LABEL {
            duration_s += g.junction_delay(&column, f.edge, b.edge);
        }
        let mut chain = meeting.backward;
        while chain != NO_LABEL {
            let label = tree.labels[chain as usize];
            let fraction = if label.next == NO_LABEL {
                approach += destinations[usize::from(label.destination)].approach;
                destinations[usize::from(label.destination)]
                    .fraction
                    .clamp(0.0, 1.0)
            } else {
                1.0
            };
            distance_m += f64::from(g.edge_length[label.edge as usize]) * fraction;
            duration_s += f64::from(column.duration(label.edge)) * fraction;
            if label.next != NO_LABEL {
                let next = tree.labels[label.next as usize].edge;
                duration_s += g.junction_delay(&column, label.edge, next);
            }
            chain = label.next;
        }
        AnalysisPath {
            edges: self.joined_edges(meeting.forward, tree, meeting.backward),
            cost: meeting.cost - approach,
            distance_m,
            duration_s,
        }
    }
}

#[allow(clippy::too_many_arguments, clippy::needless_range_loop)]
fn forward_row(
    graph: SearchGraph<'_>,
    state: &mut SearchState,
    query: &AnalysisQuery<'_>,
    heuristic_cost_per_m: f64,
    input: &ManyToManyInput<'_>,
    source: usize,
    trees: &[TargetTree],
    tree_of: &[u32],
    buckets: &Buckets,
    tuning: ManyToManyTuning,
) -> ManyToManyRow {
    let target_count = input.targets.len();
    let origins = &input.source_points[source];
    let mut row = ManyToManyRow {
        paths: vec![None; target_count],
        budget_exceeded: vec![false; target_count],
        counters: AnalysisCounters::default(),
        scanned: 0,
    };
    // Open targets by tree slot, and each slot's target.
    let mut tree_target = vec![0; trees.len()];
    for (target, slot) in tree_of.iter().enumerate() {
        if *slot != u32::MAX {
            tree_target[*slot as usize] = target;
        }
    }
    let mut open = vec![false; trees.len()];
    let mut best = vec![None::<Meeting>; trees.len()];
    let mut open_count = 0usize;
    let mut direct_targets = Vec::new();
    for target in 0..target_count {
        let slot = tree_of[target];
        if slot == u32::MAX || !input.connected[source][target] {
            continue;
        }
        // A direct path competes with searched ones: the cell search
        // compares them (`analysis::matrix_cell`).
        if direct_same_edge(
            graph,
            query,
            origins,
            input.target_points[target].iter().copied(),
        )
        .is_some()
        {
            direct_targets.push(target);
            continue;
        }
        open[slot as usize] = true;
        open_count += 1;
    }
    let stragglers = tuning.stragglers;
    let mut handed_off = false;
    // The forward queue priority at which a target's best join is final.
    let threshold = |best: &Option<Meeting>, tree: &TargetTree| {
        if tree.radius.is_infinite() {
            return f64::NEG_INFINITY;
        }
        best.map_or(f64::INFINITY, |meeting| meeting.cost - tree.radius)
    };

    let mut search = Forward {
        g: graph,
        q: query,
        automata: graph.complex_automata(query.profile_bit),
        labels: Vec::new(),
    };
    let mut heap = std::mem::take(&mut state.heap);
    heap.clear();
    let mut seq = 0u32;
    let mut push = |state: &mut SearchState,
                    labels: &mut Vec<ForwardLabel>,
                    heap: &mut BinaryHeap<HeapEntry>,
                    label: ForwardLabel,
                    priority: f64|
     -> bool {
        let slot = state.slot(label.edge, label.state);
        if slot != NO_LABEL && labels[slot as usize].cost <= label.cost {
            return false;
        }
        let index = labels.len() as u32;
        state.set_slot(label.edge, label.state, index);
        seq = seq.wrapping_add(1);
        heap.push(HeapEntry {
            priority,
            seq,
            label: index,
        });
        labels.push(label);
        true
    };

    // Costs of seed labels not yet settled: a path through one costs at
    // least that much, so a target is final only once its best is no more
    // than the cheapest of them.
    let mut pending_seeds = Vec::<((u32, u32), f64)>::new();
    // Nothing to search when every target is direct or unreachable.
    let seeded = if open_count == 0 {
        &[][..]
    } else {
        &origins[..]
    };
    for (index, origin) in seeded.iter().enumerate() {
        let cost = f64::from(query.edge_cost(graph, origin.edge));
        if !cost.is_finite() {
            continue;
        }
        let remaining = (1.0 - origin.fraction).clamp(0.0, 1.0);
        let start = cost * remaining + origin.approach;
        let traversed = remaining > SEED_TRAVERSED_FRACTION_EPSILON;
        // A seed that traverses nothing is outside every restriction context.
        let seed_state = if traversed {
            search.automata.forward_next(&graph, ROOT, origin.edge)
        } else {
            ROOT
        };
        if push(
            state,
            &mut search.labels,
            &mut heap,
            ForwardLabel {
                edge: origin.edge,
                pred: NO_LABEL,
                cost: start,
                origin: index as u16,
                traversed,
                state: seed_state,
            },
            start,
        ) {
            let key = (origin.edge, seed_state);
            pending_seeds.retain(|(seed, _)| *seed != key);
            pending_seeds.push((key, start));
        }
    }
    let least_pending = |pending: &[((u32, u32), f64)]| {
        pending
            .iter()
            .map(|(_, cost)| *cost)
            .fold(f64::INFINITY, f64::min)
    };
    let mut min_pending = least_pending(&pending_seeds);
    // Whether target `slot` is final with the queue at `priority`.
    // Ties are searched too (past the tie window), as in every search.
    let is_final = |best: &Option<Meeting>, tree: &TargetTree, priority: f64, min_pending: f64| {
        let cost = best.map_or(f64::INFINITY, |meeting| meeting.cost);
        let window = if cost.is_finite() {
            tie_window(cost)
        } else {
            0.0
        };
        threshold(best, tree) + window < priority && cost <= min_pending
    };
    // The queue priority at which target `slot` is next worth checking
    // (infinite while it waits on a pending seed; a seed's settling
    // re-checks every target).
    let check_at = |best: &Option<Meeting>, tree: &TargetTree, min_pending: f64| {
        if best.map_or(f64::INFINITY, |meeting| meeting.cost) <= min_pending {
            threshold(best, tree)
        } else {
            f64::INFINITY
        }
    };

    let mut settle_at = f64::INFINITY;
    let mut exceeded = false;
    'search: while let Some(entry) = heap.pop() {
        let label = search.labels[entry.label as usize];
        if state.slot(label.edge, label.state) != entry.label {
            continue;
        }
        if entry.priority >= settle_at {
            settle_at = f64::INFINITY;
            for (slot, tree) in trees.iter().enumerate() {
                if !open[slot] {
                    continue;
                }
                if is_final(&best[slot], tree, entry.priority, min_pending) {
                    open[slot] = false;
                    open_count -= 1;
                } else {
                    settle_at = settle_at.min(check_at(&best[slot], tree, min_pending));
                }
            }
            if open_count == 0 {
                break;
            }
            // The last few targets are usually far or penalised: a
            // goal-directed cell search each is cheaper than flooding on.
            if open_count <= stragglers {
                handed_off = true;
                break;
            }
        }
        if label.pred == NO_LABEL {
            pending_seeds.retain(|(seed, _)| *seed != (label.edge, label.state));
            min_pending = least_pending(&pending_seeds);
            for (slot, tree) in trees.iter().enumerate() {
                if open[slot] {
                    settle_at = settle_at.min(check_at(&best[slot], tree, min_pending));
                }
            }
        }
        row.counters.settled_edge_count += 1;
        let node = graph.edge_target[label.edge as usize];
        let last_traversed = label.traversed.then_some(label.edge);
        for next in graph.first_out[node as usize]..graph.first_out[node as usize + 1] {
            row.counters.visited_edge_count += 1;
            if query
                .max_visited_edge_count
                .is_some_and(|max| row.counters.visited_edge_count > max)
                || query
                    .cancellation
                    .interrupted_at(row.counters.visited_edge_count)
            {
                exceeded = true;
                break 'search;
            }
            let edge_cost = f64::from(query.edge_cost(graph, next));
            if !edge_cost.is_finite() {
                continue;
            }
            if search.transition_restricted(label.state, last_traversed, next) {
                continue;
            }
            let turn = graph.turn_cost(label.edge, next);
            if !turn.is_finite() {
                continue;
            }
            let (start, end) = (
                buckets.offsets[next as usize] as usize,
                buckets.offsets[next as usize + 1] as usize,
            );
            row.scanned += end - start;
            for at in start..end {
                let slot = buckets.target[at] as usize;
                if !open[slot] {
                    continue;
                }
                let tree = &trees[slot];
                let backward = buckets.label[at];
                let seed = tree.labels[backward as usize].next == NO_LABEL;
                let cost = label.cost
                    + if !label.traversed && seed { 0.0 } else { turn }
                    + buckets.cost[at];
                if best[slot].is_some_and(|best| cost > best.cost + tie_window(best.cost)) {
                    continue;
                }
                let edges = search.joined_edges(entry.label, tree, backward);
                let destination = input.target_points[tree_target[slot]]
                    [usize::from(tree.labels[backward as usize].destination)];
                if search.finish_repeats(entry.label, &edges, origins, destination)
                    || search.sequence_violates_complex(&edges)
                {
                    continue;
                }
                // Ties go to the route search's order (`search::Run::try_meet`).
                if let Some(current) = best[slot]
                    && cost >= current.cost - tie_window(current.cost)
                {
                    let key = |edges: &[u32]| {
                        (
                            edges.len(),
                            edges
                                .iter()
                                .map(|edge| graph.edge_id(*edge as usize))
                                .collect::<Vec<_>>(),
                        )
                    };
                    let current_edges = search.joined_edges(
                        current.forward,
                        &trees[current.tree as usize],
                        current.backward,
                    );
                    if key(&edges) >= key(&current_edges) {
                        continue;
                    }
                }
                best[slot] = Some(Meeting {
                    forward: entry.label,
                    tree: slot as u32,
                    backward,
                    cost,
                });
                settle_at = settle_at.min(check_at(&best[slot], tree, min_pending));
            }
            let cost = label.cost + edge_cost + turn;
            push(
                state,
                &mut search.labels,
                &mut heap,
                ForwardLabel {
                    edge: next,
                    pred: entry.label,
                    cost,
                    origin: label.origin,
                    traversed: true,
                    state: search.automata.forward_next(&graph, label.state, next),
                },
                cost,
            );
        }
    }
    heap.clear();
    state.heap = heap;

    for target in 0..target_count {
        let slot = tree_of[target];
        if slot == u32::MAX || row.paths[target].is_some() {
            continue;
        }
        let slot = slot as usize;
        // A target still open when the budget ran out has no proven cell;
        // it keeps its best path so far, as a row's target does.
        if exceeded && open[slot] {
            row.budget_exceeded[target] = true;
            if let Some(meeting) = best[slot] {
                row.paths[target] = Some(search.path(
                    &meeting,
                    &trees[meeting.tree as usize],
                    origins,
                    &input.target_points[target],
                ));
            }
        } else if handed_off && open[slot] {
            continue;
        } else if let Some(meeting) = best[slot] {
            row.paths[target] = Some(search.path(
                &meeting,
                &trees[meeting.tree as usize],
                origins,
                &input.target_points[target],
            ));
        }
    }
    state.reset();
    for target in 0..target_count {
        let slot = tree_of[target];
        let handed = handed_off && slot != u32::MAX && open[slot as usize];
        if !handed && !direct_targets.contains(&target) {
            continue;
        }
        let cell = super::analysis::matrix_cell(
            graph,
            state,
            query,
            heuristic_cost_per_m,
            input.sources[source],
            origins,
            input.targets[target],
            &input.target_points[target],
        );
        row.counters.visited_edge_count += cell.counters.visited_edge_count;
        row.counters.settled_edge_count += cell.counters.settled_edge_count;
        row.paths[target] = cell.path;
        row.budget_exceeded[target] = cell.budget_exceeded;
    }
    row
}

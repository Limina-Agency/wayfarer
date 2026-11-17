//! Edge-based A* over the routing package.
//!
//! Semantics (kept from the former tile-backed directed search, and written
//! out independently as plain Dijkstra over the road graph's records in
//! `crate::reference`, the parity oracle): one label per directed edge and
//! complex-restriction state (see `package::restrictions`), seeds on
//! partially traversed origin edges, turn costs between consecutive edges,
//! simple (from, via, to) and complex (required-prefix) restrictions, and
//! destinations finished when the source node of a destination edge is
//! settled (never onto an edge already on the path). Root-state labels live
//! in dense per-edge slots and the rare others in [`StateSlots`]; costs are
//! read from columns in place. Results are exact shortest paths under the
//! cost model.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use rustc_hash::{FxHashMap, FxHashSet};
use wayfarer_package::format::{ColumnPieceKinds, PieceKind};

use super::columns::{CostColumn, LandmarkColumn};
use super::landmarks::AltBound;
use super::restrictions::{ComplexAutomata, ROOT};
use super::{
    EDGE_FLAG_COMPLEX_RESTRICTION_TO, EDGE_FLAG_LINK, EDGE_FLAG_SHAPE_REVERSED,
    EDGE_FLAG_SIMPLE_RESTRICTION_FROM, NO_HEADING, NO_TOKEN, SearchGraph, chord_m, junction,
    unit_xyz,
};
use crate::costing::{
    JunctionTransition, TURN_COST_SECONDS_PER_LEGACY_UNIT, U_TURN_LEGACY_COST, edge_speed_mps,
    heading_delta, sorted_token_sets_intersect,
};
use crate::types::CorrelatedLocation;

pub(crate) mod analysis;
pub(crate) mod gates;
pub(crate) mod many_to_many;
pub(crate) mod patch;

/// Per-node (forward, backward) potential bounds, NaN until computed, and the
/// nodes whose bounds were set, borrowed by the bidirectional search.
type NodeBoundsCache = std::cell::RefCell<(Vec<(f64, f64)>, Vec<u32>)>;

const NO_LABEL: u32 = u32::MAX;
const SEED_TRAVERSED_FRACTION_EPSILON: f64 = 1e-9;

/// Costs within this window of each other are ties: f32 edge weights summed
/// in a different order differ in the last bits for the same physical route.
pub(crate) fn tie_window(reference: f64) -> f64 {
    1e-7 * reference.abs().max(1.0)
}

/// Base of the polynomial path hashes labels carry, so that joins can tell a
/// path they already hold from another of the same cost without walking it.
const PATH_HASH_BASE: u64 = 0x9e37_79b9_7f4a_7c15;

#[inline]
fn path_hash_edge(edge: u32) -> u64 {
    // splitmix64 finaliser: spreads consecutive edge indexes.
    let mut z = u64::from(edge).wrapping_add(0x9e37_79b9_7f4a_7c15);
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

/// Approach cost per metre of snap distance beyond an endpoint's nearest
/// candidate: the extra distance at an off-road pace (0.47 m/s, a third of
/// walking). It must exceed walking along a road, or a walk "approaches" a
/// candidate far along its own road in a straight line instead of walking
/// there (Valhalla snaps to the nearest road).
pub(crate) const APPROACH_COST_PER_M: f64 = 3.0 / 1.4;
/// Beyond this much farther than the nearest candidate, an approach costs
/// [`FAR_APPROACH_FACTOR`] times as much per metre: the other carriageway or
/// a parallel road a few metres off stays a choice, but a car beside a track
/// drives out along it rather than "approaching" a road 100 m away across a
/// field (Valhalla starts on the nearest road).
pub(crate) const APPROACH_WINDOW_M: f64 = 30.0;
pub(crate) const FAR_APPROACH_FACTOR: f64 = 10.0;

/// The approach cost of a candidate `extra_m` farther than its endpoint's
/// nearest candidate.
pub(crate) fn approach_cost_of(extra_m: f64) -> f64 {
    let extra_m = extra_m.max(0.0);
    let near = extra_m.min(APPROACH_WINDOW_M);
    (near + (extra_m - near) * FAR_APPROACH_FACTOR) * APPROACH_COST_PER_M
}

/// The approach cost of each candidate of one endpoint: its snap distance
/// beyond the nearest candidate's, priced by [`approach_cost_of`]. Searches add
/// it to a candidate's seed or finish, so a farther candidate (a parallel
/// road, the other carriageway, a bridge overhead) wins only when it saves
/// more than the extra approach, and leave it out of the reported cost; the
/// nearest candidates cost nothing extra.
pub(crate) fn approach_costs(candidates: &[CorrelatedLocation]) -> Vec<f64> {
    candidates
        .iter()
        .map(|candidate| approach_cost(candidate, candidates))
        .collect()
}

/// The approach cost of `chosen` among one endpoint's `candidates` (see
/// [`approach_costs`]).
pub(crate) fn approach_cost(chosen: &CorrelatedLocation, candidates: &[CorrelatedLocation]) -> f64 {
    let nearest = candidates
        .iter()
        .map(snap_distance)
        .fold(f64::INFINITY, f64::min);
    approach_cost_of(snap_distance(chosen) - nearest)
}

/// [`approach_costs`] for the candidates of several targets at once, each
/// relative to the nearest candidate of its own target (`targets[i]` is
/// candidate `i`'s).
pub(crate) fn approach_costs_by_target(
    candidates: &[CorrelatedLocation],
    targets: &[usize],
) -> Vec<f64> {
    let mut nearest = FxHashMap::<usize, f64>::default();
    for (candidate, target) in candidates.iter().zip(targets) {
        let entry = nearest.entry(*target).or_insert(f64::INFINITY);
        *entry = entry.min(snap_distance(candidate));
    }
    candidates
        .iter()
        .zip(targets)
        .map(|(candidate, target)| approach_cost_of(snap_distance(candidate) - nearest[target]))
        .collect()
}

fn snap_distance(candidate: &CorrelatedLocation) -> f64 {
    if candidate.distance_m.is_finite() {
        candidate.distance_m.max(0.0)
    } else {
        0.0
    }
}

#[derive(Clone, Copy)]
struct Label {
    edge: u32,
    pred: u32,
    cost: f64,
    origin: u16,
    traversed: bool,
    /// Forward complex-restriction state of the traversed path.
    state: u32,
    /// Traversed edges on the path to here, and their polynomial hash.
    path_len: u32,
    path_hash: u64,
    /// The first edge of the walk that ended on `edge` (see
    /// [`Run::walk_forward`]), or `NO_LABEL` when the label follows its
    /// predecessor's edge directly. With `back`, the walk turned back at the
    /// end of `via` onto `edge`.
    via: u32,
    back: bool,
}

#[derive(Clone, Copy)]
struct HeapEntry {
    priority: f64,
    seq: u32,
    label: u32,
}

impl PartialEq for HeapEntry {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}
impl Eq for HeapEntry {}
impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for HeapEntry {
    // Min-heap on (priority, insertion order).
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .priority
            .total_cmp(&self.priority)
            .then_with(|| other.seq.cmp(&self.seq))
    }
}

/// Labels in complex-restriction states other than the root, by edge (or
/// patch arc): `(state, label)` pairs. Only edges inside some forbidden
/// sequence ever hold one (`ComplexAutomata::has_states`), so this stays
/// small and the dense per-edge slots keep the common case.
#[derive(Default)]
pub(crate) struct StateSlots {
    slots: FxHashMap<u32, Vec<(u32, u32)>>,
}

impl StateSlots {
    #[inline]
    pub(crate) fn is_empty(&self) -> bool {
        self.slots.is_empty()
    }

    /// The label of `id` in `state`, or `NO_LABEL`.
    #[inline]
    pub(crate) fn get(&self, id: u32, state: u32) -> u32 {
        self.slots
            .get(&id)
            .and_then(|slots| slots.iter().find(|(at, _)| *at == state))
            .map_or(NO_LABEL, |(_, label)| *label)
    }

    /// Sets the label of `id` in `state`; returns whether `id` had none in
    /// any state here before.
    pub(crate) fn set(&mut self, id: u32, state: u32, label: u32) -> bool {
        match self.slots.entry(id) {
            std::collections::hash_map::Entry::Occupied(mut entry) => {
                match entry.get_mut().iter_mut().find(|(at, _)| *at == state) {
                    Some(slot) => slot.1 = label,
                    None => entry.get_mut().push((state, label)),
                }
                false
            }
            std::collections::hash_map::Entry::Vacant(entry) => {
                entry.insert(vec![(state, label)]);
                true
            }
        }
    }

    /// Every label of `id` held here.
    pub(crate) fn labels(&self, id: u32) -> impl Iterator<Item = u32> + '_ {
        self.slots
            .get(&id)
            .into_iter()
            .flatten()
            .map(|(_, label)| *label)
    }

    /// Every (id, label) held here.
    pub(crate) fn all(&self) -> impl Iterator<Item = (u32, u32)> + '_ {
        self.slots
            .iter()
            .flat_map(|(id, slots)| slots.iter().map(move |(_, label)| (*id, *label)))
    }

    pub(crate) fn clear(&mut self) {
        self.slots.clear();
    }
}

/// Reusable per-query buffers. `edge_label` is sized to the graph once and
/// reset sparsely through `touched`, so a query never clears O(E) memory.
/// Both directions keep root-state labels there and labels in other
/// complex-restriction states in a [`StateSlots`].
pub(crate) struct SearchState {
    edge_label: Vec<u32>,
    /// Edges labelled in any state (possibly repeated).
    touched: Vec<u32>,
    edge_extra: StateSlots,
    labels: Vec<Label>,
    heap: BinaryHeap<HeapEntry>,
    /// Backward search buffers; `b_edge_label` is allocated on first use.
    b_edge_label: Vec<u32>,
    b_touched: Vec<u32>,
    b_edge_extra: StateSlots,
    b_labels: Vec<BackwardLabel>,
    b_heap: BinaryHeap<HeapEntry>,
    /// Per-node (bound to targets, bound from sources), NaN when not computed;
    /// a node is reached through several edges but bounded once.
    node_bounds: Vec<(f64, f64)>,
    node_bounds_touched: Vec<u32>,
    /// Matrix/isochrone labels; those queries index `edge_label` into these.
    analysis_labels: Vec<analysis::AnalysisLabel>,
    /// Restriction states of the matrix/isochrone labels not in the root
    /// state, by label (those labels stay as small as before).
    analysis_label_states: FxHashMap<u32, u32>,
    /// Per-edge isochrone reach slot, allocated on first use and reset
    /// through `touched` (every reached edge is labelled).
    edge_aux: Vec<u32>,
}

/// Backward label on edge `edge`: cost from entering `edge` at its source to
/// the destination point, including the edge's own (possibly partial) cost.
#[derive(Clone, Copy)]
struct BackwardLabel {
    edge: u32,
    /// Successor label (towards the destination); `NO_LABEL` for a seed.
    next: u32,
    cost: f64,
    destination: u16,
    /// Backward complex-restriction state of the chain from `edge` on.
    state: u32,
    /// Edges of the chain from `edge` on, their polynomial hash, and the
    /// hash base raised to their count (to append the chain to a prefix).
    path_len: u32,
    path_hash: u64,
    path_pow: u64,
    /// The last edge of the backward walk that ended on `edge` (the edge
    /// before the successor's in path order, see [`Run::walk_backward`]), or
    /// `NO_LABEL`; with `back`, `edge` turned back onto `via`.
    via: u32,
    back: bool,
}

impl SearchState {
    pub(crate) fn new(edge_count: usize) -> Self {
        Self {
            edge_label: vec![NO_LABEL; edge_count],
            touched: Vec::new(),
            edge_extra: StateSlots::default(),
            labels: Vec::new(),
            heap: BinaryHeap::new(),
            b_edge_label: Vec::new(),
            b_touched: Vec::new(),
            b_edge_extra: StateSlots::default(),
            b_labels: Vec::new(),
            b_heap: BinaryHeap::new(),
            node_bounds: Vec::new(),
            node_bounds_touched: Vec::new(),
            analysis_labels: Vec::new(),
            analysis_label_states: FxHashMap::default(),
            edge_aux: Vec::new(),
        }
    }

    pub(crate) fn reset(&mut self) {
        let has_aux = !self.edge_aux.is_empty();
        for edge in self.touched.drain(..) {
            self.edge_label[edge as usize] = NO_LABEL;
            if has_aux {
                self.edge_aux[edge as usize] = NO_LABEL;
            }
        }
        self.edge_extra.clear();
        self.labels.clear();
        self.analysis_labels.clear();
        self.analysis_label_states.clear();
        self.heap.clear();
        for edge in self.b_touched.drain(..) {
            self.b_edge_label[edge as usize] = NO_LABEL;
        }
        self.b_edge_extra.clear();
        self.b_labels.clear();
        self.b_heap.clear();
        for node in self.node_bounds_touched.drain(..) {
            self.node_bounds[node as usize] = (f64::NAN, f64::NAN);
        }
    }

    /// Allocates the buffers a bidirectional route search sizes on first use
    /// (backward labels, node bounds), so a pooled state serves the first
    /// request without that cost (see `RoutingPackage::prefetch`).
    /// The forward label of `edge` in `state`, or `NO_LABEL`.
    #[inline]
    pub(crate) fn slot(&self, edge: u32, state: u32) -> u32 {
        if state == ROOT {
            self.edge_label[edge as usize]
        } else {
            self.edge_extra.get(edge, state)
        }
    }

    /// Makes `label` the forward label of `edge` in `state`.
    #[inline]
    pub(crate) fn set_slot(&mut self, edge: u32, state: u32, label: u32) {
        if state == ROOT {
            if self.edge_label[edge as usize] == NO_LABEL {
                self.touched.push(edge);
            }
            self.edge_label[edge as usize] = label;
        } else if self.edge_extra.set(edge, state, label) {
            self.touched.push(edge);
        }
    }

    /// The backward label of `edge` in `state`, or `NO_LABEL`.
    #[inline]
    pub(crate) fn b_slot(&self, edge: u32, state: u32) -> u32 {
        if state == ROOT {
            self.b_edge_label[edge as usize]
        } else {
            self.b_edge_extra.get(edge, state)
        }
    }

    /// Makes `label` the backward label of `edge` in `state`.
    #[inline]
    pub(crate) fn set_b_slot(&mut self, edge: u32, state: u32, label: u32) {
        if state == ROOT {
            if self.b_edge_label[edge as usize] == NO_LABEL {
                self.b_touched.push(edge);
            }
            self.b_edge_label[edge as usize] = label;
        } else {
            self.b_edge_extra.set(edge, state, label);
        }
    }

    /// Releases what a large search grew beyond a route's needs (label
    /// vectors, heaps, the isochrone's per-edge slots), keeping the dense
    /// per-edge label slots. For pooled states under a bounded memory
    /// profile; call after `reset`.
    pub(crate) fn trim(&mut self) {
        const KEEP: usize = 1 << 14;
        self.edge_aux = Vec::new();
        self.touched.shrink_to(KEEP);
        self.labels.shrink_to(KEEP);
        self.heap.shrink_to(KEEP);
        self.b_touched.shrink_to(KEEP);
        self.b_labels.shrink_to(KEEP);
        self.b_heap.shrink_to(KEEP);
        self.node_bounds_touched.shrink_to(KEEP);
        self.analysis_labels.shrink_to(KEEP);
        self.analysis_label_states.shrink_to(KEEP);
        self.edge_extra = StateSlots::default();
        self.b_edge_extra = StateSlots::default();
    }

    pub(crate) fn preallocate_route_buffers(&mut self, edge_count: usize, node_count: usize) {
        if self.b_edge_label.len() != edge_count {
            self.b_edge_label = vec![NO_LABEL; edge_count];
        }
        if self.node_bounds.len() != node_count {
            self.node_bounds = vec![(f64::NAN, f64::NAN); node_count];
        }
    }
}

pub(crate) struct SearchQuery<'q> {
    pub column: CostColumn<'q>,
    pub profile_bit: u8,
    pub heuristic_cost_per_m: f64,
    pub origins: &'q [CorrelatedLocation],
    pub destinations: &'q [CorrelatedLocation],
    /// Compact edge of each origin/destination candidate, resolved by the caller.
    pub origin_edges: &'q [u32],
    pub destination_edges: &'q [u32],
    /// Exact per-request edge costs (blocked = infinity) for edges touched by
    /// request options; everything else uses the precomputed column.
    pub cost_overrides: &'q FxHashMap<u32, f32>,
    pub max_visited_edge_count: Option<usize>,
    /// Stops the search like the budget; the request reports `Cancelled`.
    pub cancellation: &'q crate::RouteCancellation,
    /// False when the landmarks cannot bound the request (see
    /// `analysis::override_bounds`).
    pub use_landmarks: bool,
    /// Edges the overrides make cheaper than the column: gates of the
    /// landmark bounds (see `gates`).
    pub gates: &'q [gates::DiscountGate],
    /// Search from both ends (default) or forward only.
    pub bidirectional: bool,
    /// The SAR `turn_restrictions` relaxation: a restricted transition costs
    /// this much per violated restriction instead of being banned (`None`:
    /// banned).
    pub restriction_penalty: Option<f64>,
    /// The stored column's piece kinds: the bidirectional search walks
    /// through its marked nodes (see [`Run::walk_forward`]). `None` keeps the per-edge
    /// search; set only where the walks are exact (a stored column, no edge
    /// opened by an override, no SAR transitions or restriction penalty).
    pub pieces: Option<ColumnPieceKinds<'q>>,
}

impl<'q> SearchQuery<'q> {
    /// The landmark tables to bound with, when the query allows them.
    pub(crate) fn landmarks(&self) -> Option<LandmarkColumn<'q>> {
        self.column.landmarks.filter(|_| self.use_landmarks)
    }
}

/// A restriction broken by a path (see `SearchGraph::restriction_violations`):
/// the indexes of its first edge (the restriction's `from`) and last (`to`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct RestrictionViolation {
    pub first: usize,
    pub last: usize,
    pub complex: bool,
}

#[derive(Debug)]
pub(crate) struct SearchPath {
    /// Traversed compact edges in order, ending with the destination edge.
    pub edges: Vec<u32>,
    pub origin_index: usize,
    pub destination_index: usize,
    pub cost: f64,
    pub distance_m: f64,
    pub duration_s: f64,
}

#[derive(Debug)]
pub(crate) enum SearchOutcome {
    Found(SearchPath),
    NoPath,
    /// The budget ran out: `best` is the best path found so far, and every
    /// path not yet found costs at least `lower_bound` (by the stop's own
    /// bound; both include approach costs).
    BudgetExceeded {
        max_visited_edge_count: usize,
        best: Option<SearchPath>,
        lower_bound: f64,
    },
}

#[derive(Debug, Default, Clone, Copy)]
pub(crate) struct SearchCounters {
    /// Edges relaxed out of queued labels, and turns back out of pieces
    /// (each may queue a label); the visited-edge budget counts these.
    pub visited_edge_count: usize,
    /// Edges walked through inside pieces (see [`Run::walk_forward`]), which
    /// queue nothing and bound nothing; not budgeted.
    pub walked_edge_count: usize,
    pub settled_edge_count: usize,
    pub label_count: usize,
    pub restricted_transition_count: usize,
    pub inaccessible_edge_count: usize,
    pub origin_heuristic: f64,
}

struct Destination {
    edge: u32,
    source: u32,
    fraction: f64,
}

struct Finish {
    label: u32,
    destination: usize,
    cost: f64,
}

pub(crate) fn run(
    graph: SearchGraph<'_>,
    state: &mut SearchState,
    query: &SearchQuery<'_>,
    counters: &mut SearchCounters,
) -> SearchOutcome {
    let mut search = Run {
        g: graph,
        q: query,
        automata: graph.complex_automata(query.profile_bit),
        state,
        counters,
        seq: 0,
        origin_approach: approach_costs(query.origins),
        destination_approach: approach_costs(query.destinations),
        walk: None,
    };
    if query.bidirectional {
        search.execute_bidirectional()
    } else {
        search.execute()
    }
}

struct Run<'a, 'g, 'q> {
    g: SearchGraph<'g>,
    q: &'a SearchQuery<'q>,
    automata: &'g ComplexAutomata,
    state: &'a mut SearchState,
    counters: &'a mut SearchCounters,
    seq: u32,
    /// Approach cost of each origin and destination candidate (see
    /// [`approach_costs`]), in every seed and finish, never in the result.
    origin_approach: Vec<f64>,
    destination_approach: Vec<f64>,
    /// Clean pieces the bidirectional search walks through (see
    /// [`Run::walk_forward`]).
    walk: Option<Walk<'q>>,
}

/// A request's walkable nodes: the column's marked nodes, less the pieces
/// holding a node of an origin or destination edge.
struct Walk<'g> {
    kinds: ColumnPieceKinds<'g>,
    endpoint_nodes: FxHashSet<u32>,
}

/// What a turn back onto the same segment costs: no walk passes a turn
/// dearer than this (see [`Run::walk_forward`]).
const U_TURN_COST: f64 = U_TURN_LEGACY_COST * TURN_COST_SECONDS_PER_LEGACY_UNIT;

/// Tests: turn the walks off (the per-edge search, to compare with), and
/// count the edges walked.
pub(crate) mod walks {
    #[cfg(test)]
    thread_local! {
        static OFF: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
        static WALKED: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
    }

    #[cfg(test)]
    pub(crate) fn set_off(value: bool) {
        OFF.with(|flag| flag.set(value));
    }

    #[inline]
    pub(crate) fn off() -> bool {
        #[cfg(test)]
        return OFF.with(std::cell::Cell::get);
        #[cfg(not(test))]
        false
    }

    /// Edges walked since the last call.
    #[cfg(test)]
    pub(crate) fn take_walked() -> usize {
        WALKED.with(|count| count.replace(0))
    }

    #[allow(unused_variables)]
    pub(crate) fn record(walked: usize) {
        #[cfg(test)]
        WALKED.with(|count| count.set(count.get() + walked));
    }
}

/// How a walk into a piece ended.
#[derive(PartialEq, Eq)]
enum WalkEnd {
    Done,
    /// The visited-edge budget ran out (or the request was cancelled).
    Exhausted,
    /// The turn back at the first node costs more than a U-turn (a barrier
    /// there): the per-edge search relaxes the first edge instead.
    PerEdge,
}

impl<'q> Run<'_, '_, 'q> {
    #[inline]
    fn edge_cost(&self, edge: u32) -> f32 {
        let base = self.q.column.cost(edge);
        if self.q.cost_overrides.is_empty() {
            return base;
        }
        self.q.cost_overrides.get(&edge).copied().unwrap_or(base)
    }

    fn destinations(&self) -> Vec<Destination> {
        self.q
            .destinations
            .iter()
            .zip(self.q.destination_edges)
            .map(|(destination, edge)| Destination {
                edge: *edge,
                source: self.g.edge_source(*edge as usize),
                fraction: destination.fraction_along_edge.clamp(0.0, 1.0),
            })
            .collect()
    }

    fn execute(&mut self) -> SearchOutcome {
        let g = self.g;
        let origins = self.q.origin_edges;
        let destinations = self
            .q
            .destinations
            .iter()
            .zip(self.q.destination_edges)
            .map(|(destination, edge)| Destination {
                edge: *edge,
                source: g.edge_source(*edge as usize),
                fraction: destination.fraction_along_edge.clamp(0.0, 1.0),
            })
            .collect::<Vec<_>>();

        // Heuristic: chord distance to a disc that contains every destination
        // snapped point. Lipschitz in the chord metric, hence consistent. The
        // disc grows by how far a stored node position may be from its exact
        // one (`xyz_margin_m`): the snapped points are exact.
        let center = self
            .q
            .destinations
            .first()
            .map(|d| unit_xyz(d.snapped_point.lat, d.snapped_point.lng))
            .unwrap_or([0.0; 3]);
        let radius_m = self
            .q
            .destinations
            .iter()
            .map(|d| chord_m(center, unit_xyz(d.snapped_point.lat, d.snapped_point.lng)))
            .fold(0.0f64, f64::max)
            + g.xyz_margin_m();
        let k = self.q.heuristic_cost_per_m;
        let alt = self.q.landmarks().and_then(|landmarks| {
            let targets = destinations.iter().map(|d| d.source).collect::<Vec<_>>();
            let seeds = origins
                .iter()
                .map(|edge| g.edge_target[*edge as usize])
                .collect::<Vec<_>>();
            let base = AltBound::new(
                landmarks.from,
                landmarks.to,
                landmarks.scales,
                landmarks.count,
                landmarks.active,
                g.node_count,
                landmarks.column,
                &targets,
                &seeds,
            )?;
            gates::GatedAlt::new(landmarks, vec![base], &seeds, false, self.q.gates)
        });
        let heuristic = |node: u32| {
            let geometric = k * (chord_m(g.xyz(node), center) - radius_m).max(0.0);
            alt.as_ref()
                .map_or(geometric, |alt| alt.bound(node).max(geometric))
        };

        self.counters.origin_heuristic = origins
            .iter()
            .map(|edge| heuristic(g.edge_target[*edge as usize]))
            .fold(f64::INFINITY, f64::min);
        for (origin_index, (origin, edge)) in self.q.origins.iter().zip(origins).enumerate() {
            let cost = f64::from(self.edge_cost(*edge));
            if !cost.is_finite() {
                continue;
            }
            let remaining = (1.0 - origin.fraction_along_edge).clamp(0.0, 1.0);
            let start = cost * remaining + self.origin_approach[origin_index];
            let target = g.edge_target[*edge as usize];
            self.push_seed(
                *edge,
                start,
                start + heuristic(target),
                origin_index as u16,
                remaining > SEED_TRAVERSED_FRACTION_EPSILON,
            );
        }

        let mut best: Option<Finish> = None;
        while let Some(entry) = self.state.heap.pop() {
            if best
                .as_ref()
                // Ties are searched too, so the tie-break sees them all.
                .is_some_and(|best| entry.priority > best.cost + tie_window(best.cost))
            {
                break;
            }
            let label = self.state.labels[entry.label as usize];
            if self.state.slot(label.edge, label.state) != entry.label {
                continue;
            }
            self.counters.settled_edge_count += 1;
            let node = g.edge_target[label.edge as usize];
            let last_traversed = label.traversed.then_some(label.edge);

            for (index, destination) in destinations.iter().enumerate() {
                if destination.source != node {
                    continue;
                }
                if !self.count_visit() {
                    return self.exhausted_forward(best.as_ref(), &destinations, entry.priority);
                }
                let Some(penalty) =
                    self.transition_penalty(label.state, last_traversed, destination.edge)
                else {
                    self.counters.restricted_transition_count += 1;
                    continue;
                };
                let edge_cost = f64::from(self.edge_cost(destination.edge));
                if !edge_cost.is_finite() {
                    continue;
                }
                if self.finish_repeats(entry.label, destination) {
                    continue;
                }
                let turn =
                    last_traversed.map_or(0.0, |last| self.turn_cost(last, destination.edge));
                // A SAR barrier may close the transition.
                if !turn.is_finite() {
                    continue;
                }
                let cost = label.cost
                    + turn
                    + edge_cost * destination.fraction
                    + self.destination_approach[index]
                    + penalty;
                let better = match &best {
                    None => true,
                    Some(current) => {
                        cost < current.cost - tie_window(current.cost)
                            || ((cost - current.cost).abs() <= tie_window(current.cost)
                                && self.path_key(entry.label, destination.edge)
                                    < self.path_key(
                                        current.label,
                                        destinations[current.destination].edge,
                                    ))
                    }
                };
                if better {
                    best = Some(Finish {
                        label: entry.label,
                        destination: index,
                        cost,
                    });
                }
            }

            let (start, end) = (
                g.first_out[node as usize] as usize,
                g.first_out[node as usize + 1] as usize,
            );
            for next in start as u32..end as u32 {
                if !self.count_visit() {
                    return self.exhausted_forward(best.as_ref(), &destinations, entry.priority);
                }
                let edge_cost = f64::from(self.edge_cost(next));
                if !edge_cost.is_finite() {
                    self.counters.inaccessible_edge_count += 1;
                    continue;
                }
                let Some(penalty) = self.transition_penalty(label.state, last_traversed, next)
                else {
                    self.counters.restricted_transition_count += 1;
                    continue;
                };
                let turn = self.turn_cost(label.edge, next);
                if !turn.is_finite() {
                    continue;
                }
                let cost = label.cost + edge_cost + turn + penalty;
                let target = g.edge_target[next as usize];
                let state = self.automata.forward_next(&g, label.state, next);
                self.push(
                    next,
                    entry.label,
                    cost,
                    cost + heuristic(target),
                    label.origin,
                    true,
                    state,
                );
            }
        }

        let Some(best) = best else {
            return SearchOutcome::NoPath;
        };
        SearchOutcome::Found(self.reconstruct(&best, &destinations))
    }

    /// Counts one visited edge; false once the budget is exceeded or the
    /// request is cancelled.
    #[inline]
    fn count_visit(&mut self) -> bool {
        self.counters.visited_edge_count += 1;
        self.q
            .max_visited_edge_count
            .is_none_or(|max| self.counters.visited_edge_count <= max)
            && !self
                .q
                .cancellation
                .interrupted_at(self.counters.visited_edge_count)
    }

    /// The outcome of a search out of budget (see
    /// [`SearchOutcome::BudgetExceeded`]).
    fn exhausted(&self, best: Option<SearchPath>, lower_bound: f64) -> SearchOutcome {
        SearchOutcome::BudgetExceeded {
            max_visited_edge_count: self.q.max_visited_edge_count.unwrap_or(0),
            best,
            lower_bound,
        }
    }

    /// The least key left in the forward queue, `at_least` included.
    fn forward_floor(&self, at_least: f64) -> f64 {
        self.state
            .heap
            .peek()
            .map_or(at_least, |entry| entry.priority.min(at_least))
    }

    /// The unidirectional search out of budget with the queue at `priority`.
    fn exhausted_forward(
        &self,
        best: Option<&Finish>,
        destinations: &[Destination],
        priority: f64,
    ) -> SearchOutcome {
        let best = best.map(|finish| self.reconstruct(finish, destinations));
        self.exhausted(best, self.forward_floor(priority))
    }

    /// Labels origin edge `edge`; a seed that traverses nothing is outside
    /// every restriction context.
    fn push_seed(&mut self, edge: u32, cost: f64, priority: f64, origin: u16, traversed: bool) {
        let state = if traversed {
            self.automata.forward_next(&self.g, ROOT, edge)
        } else {
            ROOT
        };
        self.push(edge, NO_LABEL, cost, priority, origin, traversed, state);
    }

    #[inline]
    #[allow(clippy::too_many_arguments)]
    fn push(
        &mut self,
        edge: u32,
        pred: u32,
        cost: f64,
        priority: f64,
        origin: u16,
        traversed: bool,
        state: u32,
    ) {
        let slot = self.state.slot(edge, state);
        if slot != NO_LABEL && self.state.labels[slot as usize].cost <= cost {
            return;
        }
        let (mut path_len, mut path_hash) = match pred {
            NO_LABEL => (0, 0),
            pred => {
                let pred = &self.state.labels[pred as usize];
                (pred.path_len, pred.path_hash)
            }
        };
        if traversed {
            path_len += 1;
            path_hash = path_hash
                .wrapping_mul(PATH_HASH_BASE)
                .wrapping_add(path_hash_edge(edge));
        }
        self.push_label(
            Label {
                edge,
                pred,
                cost,
                origin,
                traversed,
                state,
                path_len,
                path_hash,
                via: NO_LABEL,
                back: false,
            },
            priority,
        );
    }

    /// Queues `label` unless its edge holds a label in its state at most as
    /// dear.
    #[inline]
    fn push_label(&mut self, label: Label, priority: f64) {
        let slot = self.state.slot(label.edge, label.state);
        if slot != NO_LABEL && self.state.labels[slot as usize].cost <= label.cost {
            return;
        }
        let (edge, state) = (label.edge, label.state);
        let index = self.state.labels.len() as u32;
        self.state.labels.push(label);
        self.state.set_slot(edge, state, index);
        self.seq = self.seq.wrapping_add(1);
        self.state.heap.push(HeapEntry {
            priority,
            seq: self.seq,
            label: index,
        });
        self.counters.label_count += 1;
    }

    #[inline]
    fn turn_cost(&self, previous: u32, next: u32) -> f64 {
        self.g.turn_cost(previous, next)
    }

    /// Whether a path in forward restriction `state`, whose last traversed
    /// edge is `last_traversed`, may not enter `next`.
    #[inline]
    fn transition_restricted(&self, state: u32, last_traversed: Option<u32>, next: u32) -> bool {
        if let Some(previous) = last_traversed
            && self.g.simple_restricted(previous, next, self.q.profile_bit)
        {
            return true;
        }
        self.automata.forbids(state, next)
    }

    /// The restriction cost of entering `next` from forward `state`: `None`
    /// when a restriction bans it, else the SAR penalty for the restrictions
    /// it breaks (0 without the relaxation or without a restriction).
    #[inline]
    fn transition_penalty(
        &self,
        state: u32,
        last_traversed: Option<u32>,
        next: u32,
    ) -> Option<f64> {
        match self.q.restriction_penalty {
            None => (!self.transition_restricted(state, last_traversed, next)).then_some(0.0),
            Some(penalty) => {
                let simple = last_traversed.is_some_and(|previous| {
                    self.g.simple_restricted(previous, next, self.q.profile_bit)
                });
                let count = u32::from(simple) + self.automata.forbidden_count(state, next);
                Some(penalty * f64::from(count))
            }
        }
    }

    /// Whether finishing on `destination` after `label` repeats an edge
    /// already on the path. The origin seed's own edge counts only where the
    /// two pieces overlap: a destination behind the origin on that edge is
    /// reached around a loop, onto the stretch the seed did not cover.
    fn finish_repeats(&self, label: u32, destination: &Destination) -> bool {
        let mut current = label;
        while current != NO_LABEL {
            let entry = self.state.labels[current as usize];
            if entry.traversed && entry.edge == destination.edge {
                return entry.pred != NO_LABEL
                    || self.origin_fraction(entry.origin) <= destination.fraction;
            }
            current = entry.pred;
        }
        false
    }

    fn origin_fraction(&self, origin: u16) -> f64 {
        self.q.origins[usize::from(origin)]
            .fraction_along_edge
            .clamp(0.0, 1.0)
    }

    fn traversed_edges(&self, label: u32) -> Vec<u32> {
        let mut edges = Vec::new();
        let mut current = label;
        while current != NO_LABEL {
            let entry = self.state.labels[current as usize];
            if entry.traversed {
                self.push_label_edges_reversed(entry.edge, entry.via, entry.back, &mut edges);
            }
            current = entry.pred;
        }
        edges.reverse();
        edges
    }

    /// Every edge of forward label `label`'s path, its seed's edge first
    /// (traversed or not).
    fn forward_edges(&self, label: u32) -> Vec<u32> {
        let mut edges = Vec::new();
        let mut current = label;
        while current != NO_LABEL {
            let entry = self.state.labels[current as usize];
            self.push_label_edges_reversed(entry.edge, entry.via, entry.back, &mut edges);
            current = entry.pred;
        }
        edges.reverse();
        edges
    }

    /// Every edge of backward label `label`'s chain, in path order.
    fn backward_edges(&self, label: u32) -> Vec<u32> {
        let mut edges = Vec::new();
        let mut chain = label;
        while chain != NO_LABEL {
            let entry = self.state.b_labels[chain as usize];
            self.push_backward_label_edges(&entry, &mut edges);
            chain = entry.next;
        }
        edges
    }

    /// Pushes the edges a forward label covers, last first: `edge` alone, or
    /// its walk from `via` (see [`Self::walk_forward`]).
    fn push_label_edges_reversed(&self, edge: u32, via: u32, back: bool, out: &mut Vec<u32>) {
        if via == NO_LABEL {
            out.push(edge);
        } else if back {
            out.extend([edge, via]);
        } else {
            let start = out.len();
            self.push_walk_edges(via, edge, out);
            out[start..].reverse();
        }
    }

    /// Pushes the edges a backward label covers, in path order: `edge` alone,
    /// or its walk to `via` (see [`Self::walk_backward`]).
    fn push_backward_label_edges(&self, label: &BackwardLabel, out: &mut Vec<u32>) {
        if label.via == NO_LABEL {
            out.push(label.edge);
        } else if label.back {
            out.extend([label.edge, label.via]);
        } else {
            self.push_walk_edges(label.edge, label.via, out);
        }
    }

    /// Pushes the walk from `first` to `last` along a piece's chain, both
    /// included: each edge's [`Self::continuation`], as the walks took them.
    fn push_walk_edges(&self, first: u32, last: u32, out: &mut Vec<u32>) {
        let mut edge = first;
        out.push(edge);
        while edge != last {
            edge = self
                .continuation(edge)
                .expect("a walk follows its piece's chain to its last edge");
            out.push(edge);
        }
    }

    /// Legacy tie-break key: (edge count, edge ids) of the finished path.
    fn path_key(&self, label: u32, destination_edge: u32) -> (usize, Vec<u64>) {
        let mut ids = self
            .traversed_edges(label)
            .into_iter()
            .map(|edge| self.g.edge_id(edge as usize))
            .collect::<Vec<_>>();
        ids.push(self.g.edge_id(destination_edge as usize));
        (ids.len(), ids)
    }

    fn reconstruct(&self, finish: &Finish, destinations: &[Destination]) -> SearchPath {
        let g = &self.g;
        let column = self.q.column;
        let mut distance_m = 0.0;
        let mut duration_s = 0.0;
        let mut seed = finish.label;
        let mut current = finish.label;
        while current != NO_LABEL {
            let entry = self.state.labels[current as usize];
            if entry.pred == NO_LABEL {
                seed = current;
            } else {
                distance_m += f64::from(g.edge_length[entry.edge as usize]);
                duration_s += f64::from(column.duration(entry.edge));
                // Where the search added a turn cost.
                let pred = self.state.labels[entry.pred as usize].edge;
                duration_s += g.junction_delay(&column, pred, entry.edge);
            }
            current = entry.pred;
        }
        let seed_label = self.state.labels[seed as usize];
        let origin_index = usize::from(seed_label.origin);
        let remaining = (1.0 - self.q.origins[origin_index].fraction_along_edge).clamp(0.0, 1.0);
        distance_m += f64::from(g.edge_length[seed_label.edge as usize]) * remaining;
        duration_s += f64::from(column.duration(seed_label.edge)) * remaining;

        let destination = &destinations[finish.destination];
        distance_m += f64::from(g.edge_length[destination.edge as usize]) * destination.fraction;
        duration_s += f64::from(column.duration(destination.edge)) * destination.fraction;
        let last = self.state.labels[finish.label as usize];
        if last.traversed {
            duration_s += g.junction_delay(&column, last.edge, destination.edge);
        }
        let mut edges = self.traversed_edges(finish.label);
        edges.push(destination.edge);
        SearchPath {
            edges,
            origin_index,
            destination_index: finish.destination,
            cost: finish.cost
                - self.origin_approach[origin_index]
                - self.destination_approach[finish.destination],
            distance_m,
            duration_s,
        }
    }

    // ---------------------------------------------------------------------
    // Bidirectional search
    // ---------------------------------------------------------------------

    /// Bidirectional A* with average potentials: forward labels on edges (cost
    /// to reach the end of the edge), backward labels on edges (cost from
    /// entering the edge to the destination point). Paths are joined at
    /// transitions e -> f where e carries a forward and f a backward label.
    /// The search stops once the two best keys sum to the best joined cost
    /// (exact with consistent bounds, as package landmarks are), or with
    /// `exact_stop` once either side's best key under its own bound does (see
    /// `bidirectional_body`); exhausting either side also proves optimality,
    /// because every path's first (or last) transition is checked against the
    /// seeds.
    fn execute_bidirectional(&mut self) -> SearchOutcome {
        let g = self.g;
        if self.state.b_edge_label.len() != g.edge_count {
            self.state.b_edge_label = vec![NO_LABEL; g.edge_count];
        }
        if self.state.node_bounds.len() != g.node_count {
            self.state.node_bounds = vec![(f64::NAN, f64::NAN); g.node_count];
        }
        let cache = std::cell::RefCell::new((
            std::mem::take(&mut self.state.node_bounds),
            std::mem::take(&mut self.state.node_bounds_touched),
        ));
        let outcome = self.bidirectional_body(&cache);
        let (values, touched) = cache.into_inner();
        self.state.node_bounds = values;
        self.state.node_bounds_touched = touched;
        outcome
    }

    fn bidirectional_body(&mut self, cache: &NodeBoundsCache) -> SearchOutcome {
        self.walk = self.walk_setup();
        let outcome = self.bidirectional_search(cache);
        walks::record(self.counters.walked_edge_count);
        outcome
    }

    fn bidirectional_search(&mut self, cache: &NodeBoundsCache) -> SearchOutcome {
        let g = self.g;
        let origins = self.q.origin_edges;
        let destinations = self.destinations();
        let k = self.q.heuristic_cost_per_m;
        let disc = |points: &mut dyn Iterator<Item = [f64; 3]>| {
            let points = points.collect::<Vec<_>>();
            let center = points.first().copied().unwrap_or([0.0; 3]);
            // Grown by the stored positions' margin, as in `execute`.
            let radius = points
                .iter()
                .map(|point| chord_m(center, *point))
                .fold(0.0f64, f64::max)
                + g.xyz_margin_m();
            (center, radius)
        };
        let (d_center, d_radius) = disc(
            &mut self
                .q
                .destinations
                .iter()
                .map(|d| unit_xyz(d.snapped_point.lat, d.snapped_point.lng)),
        );
        let (o_center, o_radius) = disc(
            &mut self
                .q
                .origins
                .iter()
                .map(|o| unit_xyz(o.snapped_point.lat, o.snapped_point.lng)),
        );
        let target_nodes = destinations.iter().map(|d| d.source).collect::<Vec<_>>();
        let origin_nodes = origins
            .iter()
            .map(|edge| g.edge_target[*edge as usize])
            .collect::<Vec<_>>();
        let landmarks = |targets: &[u32], seeds: &[u32], reversed: bool| {
            let landmarks = self.q.landmarks()?;
            let base = AltBound::new_directed(
                landmarks.from,
                landmarks.to,
                landmarks.scales,
                landmarks.count,
                landmarks.active,
                g.node_count,
                landmarks.column,
                targets,
                seeds,
                reversed,
            )?;
            gates::GatedAlt::new(landmarks, vec![base], seeds, reversed, self.q.gates)
        };
        let forward_alt = landmarks(&target_nodes, &origin_nodes, false);
        let backward_alt = landmarks(&origin_nodes, &target_nodes, true);
        let forward_h = |node: u32| {
            let geometric = k * (chord_m(g.xyz(node), d_center) - d_radius).max(0.0);
            forward_alt
                .as_ref()
                .map_or(geometric, |alt| alt.bound(node).max(geometric))
        };
        let backward_h = |node: u32| {
            let geometric = k * (chord_m(g.xyz(node), o_center) - o_radius).max(0.0);
            backward_alt
                .as_ref()
                .map_or(geometric, |alt| alt.bound(node).max(geometric))
        };
        // Average potentials (Ikeda et al.): p_f = (h_f - h_b) / 2 and
        // p_b = -p_f, and the search stops once the two best keys sum to at
        // least the best joined cost. That stop assumes consistent bounds,
        // which package landmark bounds are (see `package::landmarks`). With
        // `exact`, each side is keyed by its own bound instead (the symmetric approach):
        // labels are queued again whenever they improve, so each side alone
        // is an A* that keeps an open label of optimal cost on the optimal
        // path until that path is joined, and the search stops once either
        // side's best key reaches the best joined cost. An infinite own-side
        // bound proves the node useless; the other side's bound is then
        // irrelevant and clamped.
        let exact = crate::debug::exact_stop();
        let bounds = |node: u32| {
            let mut cache = cache.borrow_mut();
            let cached = cache.0[node as usize];
            if !cached.0.is_nan() {
                return cached;
            }
            let value = (forward_h(node), backward_h(node));
            cache.0[node as usize] = value;
            cache.1.push(node);
            value
        };
        let forward_potential = |node: u32| {
            let (to_target, from_source) = bounds(node);
            if to_target.is_infinite() {
                return f64::INFINITY;
            }
            if exact {
                return to_target;
            }
            let from_source = if from_source.is_finite() {
                from_source
            } else {
                0.0
            };
            (to_target - from_source) * 0.5
        };
        let backward_potential = |node: u32| {
            let (to_target, from_source) = bounds(node);
            if from_source.is_infinite() {
                return f64::INFINITY;
            }
            if exact {
                return from_source;
            }
            let to_target = if to_target.is_finite() {
                to_target
            } else {
                0.0
            };
            (from_source - to_target) * 0.5
        };
        self.counters.origin_heuristic = origin_nodes
            .iter()
            .map(|node| forward_h(*node))
            .fold(f64::INFINITY, f64::min);

        for (origin_index, (origin, edge)) in self.q.origins.iter().zip(origins).enumerate() {
            let cost = f64::from(self.edge_cost(*edge));
            if !cost.is_finite() {
                continue;
            }
            let remaining = (1.0 - origin.fraction_along_edge).clamp(0.0, 1.0);
            let start = cost * remaining + self.origin_approach[origin_index];
            let target = g.edge_target[*edge as usize];
            self.push_seed(
                *edge,
                start,
                start + forward_potential(target),
                origin_index as u16,
                remaining > SEED_TRAVERSED_FRACTION_EPSILON,
            );
        }
        for (index, destination) in destinations.iter().enumerate() {
            let cost = f64::from(self.edge_cost(destination.edge));
            if !cost.is_finite() {
                continue;
            }
            let start = cost * destination.fraction + self.destination_approach[index];
            // A one-edge chain never starts a whole forbidden sequence.
            let state = self
                .automata
                .backward_prepend(&g, ROOT, destination.edge)
                .unwrap_or(ROOT);
            self.push_backward(
                destination.edge,
                NO_LABEL,
                start,
                start + backward_potential(destination.source),
                index as u16,
                state,
            );
        }

        let mut best: Option<Meeting> = None;
        let debug = crate::debug::trace();
        let (mut forward_settled, mut backward_settled, mut first_meet) = (0usize, 0usize, None);
        loop {
            if debug
                && first_meet.is_none()
                && let Some(meeting) = &best
            {
                first_meet = Some((forward_settled, backward_settled, meeting.cost));
            }
            let forward_top = self.state.heap.peek().map(|entry| entry.priority);
            let backward_top = self.state.b_heap.peek().map(|entry| entry.priority);
            let bound = best.as_ref().map_or(f64::INFINITY, |best| best.cost);
            // An exhausted side has joined every path through its labels.
            let (Some(forward_top), Some(backward_top)) = (forward_top, backward_top) else {
                break;
            };
            let stop = if exact {
                forward_top.max(backward_top)
            } else {
                forward_top + backward_top
            };
            // Ties are searched too, so the tie-break sees them all.
            if stop > bound + tie_window(bound) {
                break;
            }
            // Balance by frontier size: key order is a poor guide when one
            // side starts with a large seed offset (e.g. a long, slow origin
            // edge), and correctness does not depend on the choice.
            if self.state.heap.len() <= self.state.b_heap.len() {
                let entry = self.state.heap.pop().expect("peeked");
                let label = self.state.labels[entry.label as usize];
                if self.state.slot(label.edge, label.state) != entry.label {
                    continue;
                }
                self.counters.settled_edge_count += 1;
                forward_settled += 1;
                let node = g.edge_target[label.edge as usize];
                let last_traversed = label.traversed.then_some(label.edge);
                for next in g.first_out[node as usize]..g.first_out[node as usize + 1] {
                    if !self.count_visit() {
                        return self.exhausted_meeting(
                            best.as_ref(),
                            &destinations,
                            (forward_top, backward_top),
                            exact,
                        );
                    }
                    let edge_cost = f64::from(self.edge_cost(next));
                    if !edge_cost.is_finite() {
                        self.counters.inaccessible_edge_count += 1;
                        continue;
                    }
                    let Some(penalty) = self.transition_penalty(label.state, last_traversed, next)
                    else {
                        self.counters.restricted_transition_count += 1;
                        continue;
                    };
                    let backward = self.state.b_edge_label[next as usize];
                    if backward != NO_LABEL {
                        self.try_meet(entry.label, backward, &destinations, &mut best);
                    }
                    if !self.state.b_edge_extra.is_empty() && self.automata.has_states(next) {
                        let others = self.state.b_edge_extra.labels(next).collect::<Vec<_>>();
                        for backward in others {
                            self.try_meet(entry.label, backward, &destinations, &mut best);
                        }
                    }
                    let turn = self.turn_cost(label.edge, next);
                    if !turn.is_finite() {
                        continue;
                    }
                    let cost = label.cost + edge_cost + turn + penalty;
                    let state = self.automata.forward_next(&g, label.state, next);
                    if let Some(kind) = self.walk_kind(g.edge_target[next as usize]) {
                        match self.walk_forward(
                            entry.label,
                            next,
                            cost,
                            state,
                            kind,
                            &forward_potential,
                        ) {
                            WalkEnd::Done => continue,
                            WalkEnd::Exhausted => {
                                return self.exhausted_meeting(
                                    best.as_ref(),
                                    &destinations,
                                    (forward_top, backward_top),
                                    exact,
                                );
                            }
                            WalkEnd::PerEdge => {}
                        }
                    }
                    let potential = forward_potential(g.edge_target[next as usize]);
                    if potential.is_infinite() {
                        continue;
                    }
                    self.push(
                        next,
                        entry.label,
                        cost,
                        cost + potential,
                        label.origin,
                        true,
                        state,
                    );
                }
            } else {
                let entry = self.state.b_heap.pop().expect("peeked");
                let label = self.state.b_labels[entry.label as usize];
                if self.state.b_slot(label.edge, label.state) != entry.label {
                    continue;
                }
                self.counters.settled_edge_count += 1;
                backward_settled += 1;
                let node = g.edge_source(label.edge as usize);
                let own_destination = destinations[usize::from(label.destination)].edge;
                for (previous, previous_source) in g.in_edges(node as usize) {
                    if !self.count_visit() {
                        return self.exhausted_meeting(
                            best.as_ref(),
                            &destinations,
                            (forward_top, backward_top),
                            exact,
                        );
                    }
                    let edge_cost = f64::from(self.edge_cost(previous));
                    if !edge_cost.is_finite() {
                        self.counters.inaccessible_edge_count += 1;
                        continue;
                    }
                    // Joins carry their own restriction checks (the forward
                    // path is known there).
                    let forward = self.state.edge_label[previous as usize];
                    if forward != NO_LABEL {
                        self.try_meet(forward, entry.label, &destinations, &mut best);
                    }
                    if !self.state.edge_extra.is_empty() && self.automata.has_states(previous) {
                        let others = self.state.edge_extra.labels(previous).collect::<Vec<_>>();
                        for forward in others {
                            self.try_meet(forward, entry.label, &destinations, &mut best);
                        }
                    }
                    let Some((state, penalty)) = self.backward_step(previous, &label) else {
                        self.counters.restricted_transition_count += 1;
                        continue;
                    };
                    // A path finishes on its destination edge only once.
                    if previous == own_destination {
                        continue;
                    }
                    let turn = self.turn_cost(previous, label.edge);
                    if !turn.is_finite() {
                        continue;
                    }
                    let cost = label.cost + edge_cost + turn + penalty;
                    if let Some(kind) = self.walk_kind(previous_source) {
                        match self.walk_backward(
                            entry.label,
                            previous,
                            cost,
                            state,
                            kind,
                            &backward_potential,
                        ) {
                            WalkEnd::Done => continue,
                            WalkEnd::Exhausted => {
                                return self.exhausted_meeting(
                                    best.as_ref(),
                                    &destinations,
                                    (forward_top, backward_top),
                                    exact,
                                );
                            }
                            WalkEnd::PerEdge => {}
                        }
                    }
                    let potential = backward_potential(previous_source);
                    if potential.is_infinite() {
                        continue;
                    }
                    self.push_backward(
                        previous,
                        entry.label,
                        cost,
                        cost + potential,
                        label.destination,
                        state,
                    );
                }
            }
        }

        if debug {
            eprintln!(
                "  bidir: fwd_settled={forward_settled} bwd_settled={backward_settled} first_meet={first_meet:?} best={:?} fwd_top={:?} bwd_top={:?} h_f(origin)={:.1} h_b(dest)={:.1}",
                best.as_ref().map(|b| b.cost),
                self.state.heap.peek().map(|e| e.priority),
                self.state.b_heap.peek().map(|e| e.priority),
                origin_nodes
                    .iter()
                    .map(|n| forward_h(*n))
                    .fold(f64::INFINITY, f64::min),
                target_nodes
                    .iter()
                    .map(|n| backward_h(*n))
                    .fold(f64::INFINITY, f64::min),
            );
        }
        let Some(best) = best else {
            return SearchOutcome::NoPath;
        };
        SearchOutcome::Found(self.reconstruct_meeting(&best, &destinations))
    }

    // ---------------------------------------------------------------------
    // Walks through pieces
    // ---------------------------------------------------------------------

    /// The walks of this request, if any: the query's marked nodes less the
    /// pieces holding a node of an origin or destination edge (found by a
    /// search inside each such piece over the arcs the column or the
    /// request's overrides can use).
    fn walk_setup(&self) -> Option<Walk<'q>> {
        let kinds = self.q.pieces.filter(|_| !walks::off())?;
        let g = &self.g;
        let column = &self.q.column;
        let usable = |edge: u32| column.cost(edge).is_finite() || self.edge_cost(edge).is_finite();
        let mut endpoint_nodes = FxHashSet::default();
        let mut queue = Vec::new();
        let edges = self.q.origin_edges.iter().chain(self.q.destination_edges);
        for edge in edges {
            for node in [g.edge_source(*edge as usize), g.edge_target[*edge as usize]] {
                if kinds.kind(node) == PieceKind::Other || !endpoint_nodes.insert(node) {
                    continue;
                }
                queue.push(node);
                while let Some(node) = queue.pop() {
                    let node = node as usize;
                    let out = (g.first_out[node]..g.first_out[node + 1])
                        .filter(|edge| usable(*edge))
                        .map(|edge| g.edge_target[edge as usize]);
                    let into = g
                        .in_edges(node)
                        .filter(|(edge, _)| usable(*edge))
                        .map(|(_, source)| source);
                    for next in out.chain(into) {
                        if kinds.kind(next) != PieceKind::Other && endpoint_nodes.insert(next) {
                            queue.push(next);
                        }
                    }
                }
            }
        }
        Some(Walk {
            kinds,
            endpoint_nodes,
        })
    }

    /// The kind of `node` when a walk may enter it: a chain or tree node of
    /// a piece that holds no endpoint.
    #[inline]
    fn walk_kind(&self, node: u32) -> Option<PieceKind> {
        let walk = self.walk.as_ref()?;
        match walk.kinds.kind(node) {
            PieceKind::Other => None,
            kind => (!walk.endpoint_nodes.contains(&node)).then_some(kind),
        }
    }

    #[inline]
    fn is_chain(&self, node: u32) -> bool {
        self.walk
            .as_ref()
            .is_some_and(|walk| walk.kinds.kind(node) == PieceKind::Chain)
    }

    #[inline]
    fn is_tree(&self, node: u32) -> bool {
        self.walk
            .as_ref()
            .is_some_and(|walk| walk.kinds.kind(node) == PieceKind::Tree)
    }

    /// The edge continuing a piece's chain after `edge`, which ends at a
    /// chain node: its usable out-edge to the node's other core neighbour
    /// (not back, not into a tree). A marked chain node has at most one.
    fn continuation(&self, edge: u32) -> Option<u32> {
        let g = &self.g;
        let (back, node) = (g.edge_source(edge as usize), g.edge_target[edge as usize]);
        let mut found = None;
        for next in g.first_out[node as usize]..g.first_out[node as usize + 1] {
            let target = g.edge_target[next as usize];
            if target == back || self.is_tree(target) || !self.edge_cost(next).is_finite() {
                continue;
            }
            debug_assert!(found.is_none(), "a chain node continues one way");
            found = Some(next);
            if !cfg!(debug_assertions) {
                break;
            }
        }
        found
    }

    /// The edge before `edge` along a piece's chain, which starts at a chain
    /// node: the inverse of [`Self::continuation`].
    fn chain_predecessor(&self, edge: u32) -> Option<u32> {
        let g = &self.g;
        let (node, ahead) = (g.edge_source(edge as usize), g.edge_target[edge as usize]);
        let mut found = None;
        for (previous, source) in g.in_edges(node as usize) {
            if source == ahead || self.is_tree(source) || !self.edge_cost(previous).is_finite() {
                continue;
            }
            debug_assert!(found.is_none(), "a chain node continues one way");
            found = Some(previous);
            if !cfg!(debug_assertions) {
                break;
            }
        }
        found
    }

    /// The usable edge back from `edge`'s target to its source: at a marked
    /// node at most one, of the same segment, so turning onto it costs a
    /// U-turn.
    fn opposing(&self, edge: u32) -> Option<u32> {
        let g = &self.g;
        let (source, node) = (g.edge_source(edge as usize), g.edge_target[edge as usize]);
        (g.first_out[node as usize]..g.first_out[node as usize + 1]).find(|next| {
            g.edge_target[*next as usize] == source && self.edge_cost(*next).is_finite()
        })
    }

    /// Relaxes `first`, out of forward label `pred` (cost `cost` at the end
    /// of `first`), into a piece without an endpoint, whose marked node
    /// `first` enters is of `kind`. If turning back at that node costs more
    /// than a U-turn (a barrier there), returns [`WalkEnd::PerEdge`] and
    /// queues nothing.
    ///
    /// Inside such a piece the per-edge search's labels are dominated by two
    /// paths, so only those are queued:
    ///
    /// - *Back*: the U-turn at `first`'s end onto its opposing edge. Any
    ///   other way back out over that edge turns back somewhere inside (the
    ///   piece is a tree of single segments), paying a U-turn and more.
    /// - *Through* (chain nodes only): along the chain, edge by edge with the
    ///   column's costs and turn costs, to the edge entering the next node
    ///   that is not a marked chain node, and queued there. Leaving the
    ///   chain anywhere means coming back to the same node after a U-turn,
    ///   which costs at least as much as going straight on wherever that
    ///   turn costs at most a U-turn; where it costs more, the walk stops
    ///   and queues the edge before it, and the per-edge search continues
    ///   from there, dead ends included.
    ///
    /// Both sides stop at the same transitions, so they meet where the
    /// per-edge search would. Labels inside are neither queued nor bounded.
    #[allow(clippy::too_many_arguments)]
    fn walk_forward(
        &mut self,
        pred: u32,
        first: u32,
        cost: f64,
        state: u32,
        kind: PieceKind,
        potential: &dyn Fn(u32) -> f64,
    ) -> WalkEnd {
        let g = self.g;
        let back = self
            .opposing(first)
            .map(|back| (back, self.turn_cost(first, back)));
        if back.is_some_and(|(_, turn)| turn.is_nan() || turn > U_TURN_COST) {
            return WalkEnd::PerEdge;
        }
        let pred_label = self.state.labels[pred as usize];
        let origin = pred_label.origin;
        let path_len = pred_label.path_len + 1;
        let path_hash = pred_label
            .path_hash
            .wrapping_mul(PATH_HASH_BASE)
            .wrapping_add(path_hash_edge(first));
        if let Some((back, turn)) = back {
            if !self.count_visit() {
                return WalkEnd::Exhausted;
            }
            let back_cost = cost + f64::from(self.edge_cost(back)) + turn;
            let bound = potential(g.edge_source(first as usize));
            if back_cost.is_finite() && bound.is_finite() {
                self.push_label(
                    Label {
                        edge: back,
                        pred,
                        cost: back_cost,
                        origin,
                        traversed: true,
                        state: self.automata.forward_next(&g, state, back),
                        path_len: path_len + 1,
                        path_hash: path_hash
                            .wrapping_mul(PATH_HASH_BASE)
                            .wrapping_add(path_hash_edge(back)),
                        via: first,
                        back: true,
                    },
                    back_cost + bound,
                );
            }
        }
        if kind != PieceKind::Chain {
            return WalkEnd::Done;
        }
        let (mut edge, mut cost, mut state, mut path_len, mut path_hash) =
            (first, cost, state, path_len, path_hash);
        loop {
            let at_end = !self.is_chain(g.edge_target[edge as usize]);
            let next = if at_end {
                None
            } else {
                self.continuation(edge)
            };
            let turn = match next {
                Some(next) => {
                    self.counters.walked_edge_count += 1;
                    self.turn_cost(edge, next)
                }
                None => f64::INFINITY,
            };
            if let (Some(next), true) = (next, turn <= U_TURN_COST) {
                cost = cost + f64::from(self.edge_cost(next)) + turn;
                state = self.automata.forward_next(&g, state, next);
                path_len += 1;
                path_hash = path_hash
                    .wrapping_mul(PATH_HASH_BASE)
                    .wrapping_add(path_hash_edge(next));
                edge = next;
                continue;
            }
            // A dead end (nothing to queue: the way back is the U-turn
            // above), or the edge before a junction or a dear turn.
            if next.is_none() && !at_end {
                return WalkEnd::Done;
            }
            let bound = potential(g.edge_target[edge as usize]);
            if bound.is_finite() {
                self.push_label(
                    Label {
                        edge,
                        pred,
                        cost,
                        origin,
                        traversed: true,
                        state,
                        path_len,
                        path_hash,
                        via: if edge == first { NO_LABEL } else { first },
                        back: false,
                    },
                    cost + bound,
                );
            }
            return WalkEnd::Done;
        }
    }

    /// [`Self::walk_forward`] for the backward search: relaxes `first`
    /// (whose source is of `kind`) before backward label `next_label`,
    /// `cost` from entering `first`, in backward restriction `state`.
    #[allow(clippy::too_many_arguments)]
    fn walk_backward(
        &mut self,
        next_label: u32,
        first: u32,
        cost: f64,
        state: u32,
        kind: PieceKind,
        potential: &dyn Fn(u32) -> f64,
    ) -> WalkEnd {
        let g = self.g;
        let back = self
            .opposing(first)
            .map(|back| (back, self.turn_cost(back, first)));
        if back.is_some_and(|(_, turn)| turn.is_nan() || turn > U_TURN_COST) {
            return WalkEnd::PerEdge;
        }
        let after = self.state.b_labels[next_label as usize];
        let destination = after.destination;
        let prepend = |(len, hash, pow): (u32, u64, u64), edge: u32| {
            (
                len + 1,
                path_hash_edge(edge).wrapping_mul(pow).wrapping_add(hash),
                pow.wrapping_mul(PATH_HASH_BASE),
            )
        };
        let path = prepend((after.path_len, after.path_hash, after.path_pow), first);
        if let Some((back, turn)) = back {
            if !self.count_visit() {
                return WalkEnd::Exhausted;
            }
            let back_cost = cost + f64::from(self.edge_cost(back)) + turn;
            let bound = potential(g.edge_source(back as usize));
            if let Some(back_state) = self.automata.backward_prepend(&g, state, back)
                && back_cost.is_finite()
                && bound.is_finite()
            {
                let (path_len, path_hash, path_pow) = prepend(path, back);
                self.push_backward_label(
                    BackwardLabel {
                        edge: back,
                        next: next_label,
                        cost: back_cost,
                        destination,
                        state: back_state,
                        path_len,
                        path_hash,
                        path_pow,
                        via: first,
                        back: true,
                    },
                    back_cost + bound,
                );
            }
        }
        if kind != PieceKind::Chain {
            return WalkEnd::Done;
        }
        let (mut edge, mut cost, mut state, mut path) = (first, cost, state, path);
        loop {
            let at_end = !self.is_chain(g.edge_source(edge as usize));
            let previous = if at_end {
                None
            } else {
                self.chain_predecessor(edge)
            };
            let turn = match previous {
                Some(previous) => {
                    self.counters.walked_edge_count += 1;
                    self.turn_cost(previous, edge)
                }
                None => f64::INFINITY,
            };
            if let (Some(previous), true) = (previous, turn <= U_TURN_COST) {
                let Some(next_state) = self.automata.backward_prepend(&g, state, previous) else {
                    // No marked node is a restriction's via node.
                    return WalkEnd::Done;
                };
                cost = cost + f64::from(self.edge_cost(previous)) + turn;
                state = next_state;
                path = prepend(path, previous);
                edge = previous;
                continue;
            }
            if previous.is_none() && !at_end {
                return WalkEnd::Done;
            }
            let bound = potential(g.edge_source(edge as usize));
            if bound.is_finite() {
                let (path_len, path_hash, path_pow) = path;
                self.push_backward_label(
                    BackwardLabel {
                        edge,
                        next: next_label,
                        cost,
                        destination,
                        state,
                        path_len,
                        path_hash,
                        path_pow,
                        via: if edge == first { NO_LABEL } else { first },
                        back: false,
                    },
                    cost + bound,
                );
            }
            return WalkEnd::Done;
        }
    }

    /// The bidirectional search out of budget, its queues' least keys at
    /// `tops` before this step: the lower bound is the stop's own (exact with
    /// `exact`, approximate like the stop otherwise).
    fn exhausted_meeting(
        &self,
        best: Option<&Meeting>,
        destinations: &[Destination],
        (forward_top, backward_top): (f64, f64),
        exact: bool,
    ) -> SearchOutcome {
        let forward = self.forward_floor(forward_top);
        let backward = self
            .state
            .b_heap
            .peek()
            .map_or(backward_top, |entry| entry.priority.min(backward_top));
        let lower_bound = if exact {
            forward.max(backward)
        } else {
            forward + backward
        };
        let best = best.map(|meeting| self.reconstruct_meeting(meeting, destinations));
        self.exhausted(best, lower_bound)
    }

    #[inline]
    fn push_backward(
        &mut self,
        edge: u32,
        next: u32,
        cost: f64,
        priority: f64,
        destination: u16,
        state: u32,
    ) {
        let slot = self.state.b_slot(edge, state);
        if slot != NO_LABEL && self.state.b_labels[slot as usize].cost <= cost {
            return;
        }
        let (path_len, path_hash, path_pow) = match next {
            NO_LABEL => (0, 0, 1),
            next => {
                let next = &self.state.b_labels[next as usize];
                (next.path_len, next.path_hash, next.path_pow)
            }
        };
        self.push_backward_label(
            BackwardLabel {
                edge,
                next,
                cost,
                destination,
                state,
                path_len: path_len + 1,
                path_hash: path_hash_edge(edge)
                    .wrapping_mul(path_pow)
                    .wrapping_add(path_hash),
                path_pow: path_pow.wrapping_mul(PATH_HASH_BASE),
                via: NO_LABEL,
                back: false,
            },
            priority,
        );
    }

    /// Queues backward `label` unless its edge holds one in its state at
    /// most as dear.
    #[inline]
    fn push_backward_label(&mut self, label: BackwardLabel, priority: f64) {
        let slot = self.state.b_slot(label.edge, label.state);
        if slot != NO_LABEL && self.state.b_labels[slot as usize].cost <= label.cost {
            return;
        }
        let (edge, state) = (label.edge, label.state);
        let index = self.state.b_labels.len() as u32;
        self.state.b_labels.push(label);
        self.state.set_b_slot(edge, state, index);
        self.seq = self.seq.wrapping_add(1);
        self.state.b_heap.push(HeapEntry {
            priority,
            seq: self.seq,
            label: index,
        });
        self.counters.label_count += 1;
    }

    /// The backward restriction state of `previous` prepended to the chain
    /// at `label`, or `None` when that forms a restricted sequence: a simple
    /// (previous -> first) restriction, or a complex restriction's whole
    /// forbidden sequence starting at `previous`.
    #[inline]
    fn backward_restricted(&self, previous: u32, label: &BackwardLabel) -> Option<u32> {
        let g = &self.g;
        if g.edge_flags[previous as usize] & EDGE_FLAG_SIMPLE_RESTRICTION_FROM != 0
            && self.simple_restricted(previous, label.edge)
        {
            return None;
        }
        self.automata.backward_prepend(g, label.state, previous)
    }

    /// [`Self::backward_restricted`] under the query's restriction rule: the
    /// state of `previous` prepended to the chain at `label` and the SAR
    /// penalty of the restrictions that completes, or `None` when banned.
    #[inline]
    fn backward_step(&self, previous: u32, label: &BackwardLabel) -> Option<(u32, f64)> {
        let Some(penalty) = self.q.restriction_penalty else {
            return self
                .backward_restricted(previous, label)
                .map(|state| (state, 0.0));
        };
        let g = &self.g;
        let simple = g.edge_flags[previous as usize] & EDGE_FLAG_SIMPLE_RESTRICTION_FROM != 0
            && self.simple_restricted(previous, label.edge);
        let (state, complex) = self.automata.backward_step(g, label.state, previous);
        Some((state, penalty * f64::from(u32::from(simple) + complex)))
    }

    fn simple_restricted(&self, from_edge: u32, to_edge: u32) -> bool {
        self.g
            .simple_restricted_rows(from_edge, to_edge, self.q.profile_bit)
    }

    /// Joins forward label `forward` (ending at the source of the backward
    /// label's edge) with backward label `backward`.
    fn try_meet(
        &self,
        forward: u32,
        backward: u32,
        destinations: &[Destination],
        best: &mut Option<Meeting>,
    ) {
        let f = self.state.labels[forward as usize];
        let b = self.state.b_labels[backward as usize];
        let destination_seed = b.next == NO_LABEL;
        // Legacy semantics: an untraversed origin seed imposes no restriction,
        // and finishing straight onto a destination edge from it costs no turn.
        let turn = if !f.traversed && destination_seed {
            0.0
        } else {
            self.turn_cost(f.edge, b.edge)
        };
        if !turn.is_finite() {
            return;
        }
        let mut cost = f.cost + turn + b.cost;
        if best
            .as_ref()
            .is_some_and(|best| cost > best.cost + tie_window(best.cost))
        {
            return;
        }
        let spanning = match self.q.restriction_penalty {
            None => {
                if f.traversed && self.transition_restricted(f.state, Some(f.edge), b.edge) {
                    return;
                }
                None
            }
            // Restrictions spanning the join, which neither side has paid.
            Some(penalty) => {
                let edges = self.meeting_edges(forward, backward);
                let boundary = f.path_len as usize;
                let count = self
                    .g
                    .restriction_violations(&edges, self.q.profile_bit)
                    .iter()
                    .filter(|violation| violation.first < boundary && violation.last >= boundary)
                    .count();
                cost += penalty * count as f64;
                if best
                    .as_ref()
                    .is_some_and(|best| cost > best.cost + tie_window(best.cost))
                {
                    return;
                }
                Some(edges)
            }
        };
        let path_len = f.path_len + b.path_len;
        let path_hash = f
            .path_hash
            .wrapping_mul(b.path_pow)
            .wrapping_add(b.path_hash);
        if let Some(current) = best.as_ref()
            && cost >= current.cost - tie_window(current.cost)
            && (path_len > current.path_len
                || (path_len == current.path_len && path_hash == current.path_hash))
        {
            // A tie loses on edge count before any walk, and the best path met
            // again at another edge (equal count and hash) changes nothing.
            return;
        }
        let edges = spanning.unwrap_or_else(|| self.meeting_edges(forward, backward));
        debug_assert_eq!(edges.len(), path_len as usize);
        if let Some(current) = best.as_ref()
            && cost >= current.cost - tie_window(current.cost)
        {
            // Legacy tie-break: fewer edges, then lexicographically smaller ids.
            let key = |edges: &[u32]| {
                (
                    edges.len(),
                    edges
                        .iter()
                        .map(|edge| self.g.edge_id(*edge as usize))
                        .collect::<Vec<_>>(),
                )
            };
            if key(&edges) >= key(&self.meeting_edges(current.forward, current.backward)) {
                return;
            }
        }
        // A path finishes on its destination edge only once (other edges may
        // repeat where restrictions force a loop, as in the forward search);
        // the origin seed's piece of it counts only where the two overlap.
        if let Some((destination, before)) = edges.split_last() {
            let seed = self.state.labels[self.seed_label(forward) as usize];
            let fraction = destinations[usize::from(b.destination)].fraction;
            let repeats = before.iter().enumerate().any(|(index, edge)| {
                edge == destination
                    && (index > 0
                        || !seed.traversed
                        || self.origin_fraction(seed.origin) <= fraction)
            });
            if repeats {
                return;
            }
        }
        if self.q.restriction_penalty.is_none() && self.sequence_violates_complex(&edges) {
            return;
        }
        *best = Some(Meeting {
            forward,
            backward,
            cost,
            path_len,
            path_hash,
        });
    }

    /// The seed label at the start of forward label `label`'s path.
    fn seed_label(&self, label: u32) -> u32 {
        let mut current = label;
        while self.state.labels[current as usize].pred != NO_LABEL {
            current = self.state.labels[current as usize].pred;
        }
        current
    }

    /// Traversed edges of the joined path, origin to destination.
    fn meeting_edges(&self, forward: u32, backward: u32) -> Vec<u32> {
        let mut edges = self.traversed_edges(forward);
        let mut chain = backward;
        while chain != NO_LABEL {
            let label = self.state.b_labels[chain as usize];
            self.push_backward_label_edges(&label, &mut edges);
            chain = label.next;
        }
        edges
    }

    /// Full-path complex restriction check for joined paths (sequences may
    /// span the join, which neither side could see alone).
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

    fn reconstruct_meeting(&self, meeting: &Meeting, destinations: &[Destination]) -> SearchPath {
        let g = &self.g;
        let column = self.q.column;
        let mut distance_m = 0.0;
        let mut duration_s = 0.0;
        // Summed from the meeting back to the seed, as the labels lie.
        let forward = self.forward_edges(meeting.forward);
        for index in (1..forward.len()).rev() {
            let edge = forward[index];
            distance_m += f64::from(g.edge_length[edge as usize]);
            duration_s += f64::from(column.duration(edge));
            duration_s += g.junction_delay(&column, forward[index - 1], edge);
        }
        // The join, as `try_meet` costs it.
        let (f, b) = (
            self.state.labels[meeting.forward as usize],
            self.state.b_labels[meeting.backward as usize],
        );
        if f.traversed || b.next != NO_LABEL {
            duration_s += g.junction_delay(&column, f.edge, b.edge);
        }
        let seed_label = self.state.labels[self.seed_label(meeting.forward) as usize];
        let origin_index = usize::from(seed_label.origin);
        let remaining = (1.0 - self.q.origins[origin_index].fraction_along_edge).clamp(0.0, 1.0);
        distance_m += f64::from(g.edge_length[seed_label.edge as usize]) * remaining;
        duration_s += f64::from(column.duration(seed_label.edge)) * remaining;

        let backward = self.backward_edges(meeting.backward);
        let mut destination_index = 0;
        let mut chain = meeting.backward;
        while chain != NO_LABEL {
            let label = self.state.b_labels[chain as usize];
            if label.next == NO_LABEL {
                destination_index = usize::from(label.destination);
            }
            chain = label.next;
        }
        for (index, edge) in backward.iter().enumerate() {
            let last = index + 1 == backward.len();
            let fraction = if last {
                destinations[destination_index].fraction
            } else {
                1.0
            };
            distance_m += f64::from(g.edge_length[*edge as usize]) * fraction;
            duration_s += f64::from(column.duration(*edge)) * fraction;
            if !last {
                duration_s += g.junction_delay(&column, *edge, backward[index + 1]);
            }
        }
        SearchPath {
            edges: self.meeting_edges(meeting.forward, meeting.backward),
            origin_index,
            destination_index,
            cost: meeting.cost
                - self.origin_approach[origin_index]
                - self.destination_approach[destination_index],
            distance_m,
            duration_s,
        }
    }
}

struct Meeting {
    forward: u32,
    backward: u32,
    cost: f64,
    path_len: u32,
    path_hash: u64,
}

impl SearchGraph<'_> {
    /// Legacy `transition_cost_with_shared_info` over compact attributes,
    /// plus the request's transition costs (barriers, SAR R2; infinite: a
    /// barrier closes the transition).
    #[inline]
    pub(crate) fn turn_cost(&self, previous: u32, next: u32) -> f64 {
        let turn = self.legacy_turn_cost(previous, next);
        match &self.transitions {
            None => turn,
            Some(transitions) => turn + transitions.cost(self, previous, next),
        }
    }

    #[inline]
    fn legacy_turn_cost(&self, previous: u32, next: u32) -> f64 {
        let (previous, next) = (previous as usize, next as usize);
        let (si_previous, si_next) = (self.edge_shared_info[previous], self.edge_shared_info[next]);
        // One segment: straight on along it (a closed loop), or a U-turn onto
        // its opposing edge.
        if si_previous == si_next {
            let reversed =
                (self.edge_flags[previous] ^ self.edge_flags[next]) & EDGE_FLAG_SHAPE_REVERSED;
            if reversed == 0 {
                return 0.0;
            }
            return U_TURN_LEGACY_COST * TURN_COST_SECONDS_PER_LEGACY_UNIT;
        }
        let (heading_out, heading_in) = (self.heading_end(previous), self.heading_start[next]);
        if heading_out == NO_HEADING || heading_in == NO_HEADING {
            return 0.0;
        }
        let delta = heading_delta(heading_out, heading_in);
        let base = if delta < 35 {
            return 0.0;
        } else if delta < 100 {
            35.0
        } else if delta < 155 {
            90.0
        } else {
            180.0
        };
        let link = (self.edge_flags[previous] | self.edge_flags[next]) & EDGE_FLAG_LINK != 0;
        let mut cost: f64 = if link && delta < 155 {
            (base * 0.5f64).max(15.0)
        } else {
            base
        };
        if delta < 100 && self.same_corridor(si_previous as usize, si_next as usize) {
            cost *= 0.7;
        } else if !link {
            let (rank_previous, rank_next) = (self.road_rank[previous], self.road_rank[next]);
            if rank_previous > rank_next {
                cost *= 1.0 + f64::from(rank_previous - rank_next) * 0.2;
            }
        }
        cost * TURN_COST_SECONDS_PER_LEGACY_UNIT
    }

    /// Seconds lost at the junction moving from `previous` onto `next` under
    /// `column` (its profile's model and speeds, see
    /// `costing::junction`): added to durations wherever [`Self::turn_cost`]
    /// is added to costs. Mirrors `CostModel::junction_delay_for_records`.
    #[inline]
    pub(crate) fn junction_delay(&self, column: &CostColumn<'_>, previous: u32, next: u32) -> f64 {
        let Some(model) = column.junction else {
            return 0.0;
        };
        let (p, n) = (previous as usize, next as usize);
        let turn_deg = if self.edge_shared_info[p] == self.edge_shared_info[n] {
            if (self.edge_flags[p] ^ self.edge_flags[n]) & EDGE_FLAG_SHAPE_REVERSED == 0 {
                return 0.0;
            }
            Some(180)
        } else {
            let (heading_out, heading_in) = (self.heading_end(p), self.heading_start[n]);
            (heading_out != NO_HEADING && heading_in != NO_HEADING)
                .then(|| heading_delta(heading_out, heading_in))
        };
        let junction = self.junction(p);
        // Straight on, uncontrolled and not yielding: nothing lost.
        if junction == 0 && turn_deg.is_none_or(|deg| deg < 35) {
            return 0.0;
        }
        let (previous_length, next_length) = (
            f64::from(self.edge_length[p]),
            f64::from(self.edge_length[n]),
        );
        model.delay_s(&JunctionTransition {
            turn_deg,
            previous_speed_mps: edge_speed_mps(
                previous_length,
                f64::from(column.duration(previous)),
            ),
            previous_length_m: previous_length,
            next_speed_mps: edge_speed_mps(next_length, f64::from(column.duration(next))),
            next_length_m: next_length,
            control: junction::control(junction),
            minor_approach: junction::minor_approach(junction),
        })
    }

    fn same_corridor(&self, left: usize, right: usize) -> bool {
        let token_eq = |a: u32, b: u32| a != NO_TOKEN && a == b;
        self.si_way.same_way(left, right)
            || token_eq(self.si_ref[left], self.si_ref[right])
            || token_eq(self.si_name[left], self.si_name[right])
            || sorted_token_sets_intersect(self.route_refs(left), self.route_refs(right))
    }

    fn route_refs(&self, shared_info: usize) -> &[u32] {
        self.si_route_refs.get(shared_info)
    }

    /// Every restriction the traversed edge sequence `edges` breaks for the
    /// profile: each simple (from, to) restriction and each distinct complex
    /// restriction (required prefix, to) once per occurrence, by the indexes
    /// of its first and last edge. The SAR `turn_restrictions` relaxation
    /// charges each one, and reports them.
    pub(crate) fn restriction_violations(
        &self,
        edges: &[u32],
        profile_bit: u8,
    ) -> Vec<RestrictionViolation> {
        let mut violations = Vec::new();
        for index in 1..edges.len() {
            let to = edges[index];
            if self.simple_restricted(edges[index - 1], to, profile_bit) {
                violations.push(RestrictionViolation {
                    first: index - 1,
                    last: index,
                    complex: false,
                });
            }
            let mut prefixes = self
                .complex_prefixes_to(to, profile_bit)
                .filter(|prefix| edges[..index].ends_with(prefix))
                .collect::<Vec<_>>();
            prefixes.sort_unstable();
            prefixes.dedup();
            violations.extend(prefixes.into_iter().map(|prefix| RestrictionViolation {
                first: index - prefix.len(),
                last: index,
                complex: true,
            }));
        }
        violations
    }

    /// Whether a simple (from, to) restriction applies to this profile.
    #[inline]
    pub(crate) fn simple_restricted(&self, from_edge: u32, to_edge: u32, profile_bit: u8) -> bool {
        self.edge_flags[from_edge as usize] & EDGE_FLAG_SIMPLE_RESTRICTION_FROM != 0
            && self.simple_restricted_rows(from_edge, to_edge, profile_bit)
    }

    fn simple_restricted_rows(&self, from_edge: u32, to_edge: u32, profile_bit: u8) -> bool {
        let from = self.simple_from;
        let start = from.partition_point(|edge| *edge < from_edge);
        (start..from.len())
            .take_while(|row| from[*row] == from_edge)
            .any(|row| self.simple_to[row] == to_edge && self.simple_mask[row] & profile_bit != 0)
    }

    /// Required traversed prefixes of the complex restrictions that forbid
    /// entering `to_edge` for this profile; empty for unflagged edges.
    pub(crate) fn complex_prefixes_to(
        &self,
        to_edge: u32,
        profile_bit: u8,
    ) -> impl Iterator<Item = &[u32]> + '_ {
        let rows = if self.edge_flags[to_edge as usize] & EDGE_FLAG_COMPLEX_RESTRICTION_TO != 0 {
            let start = self.complex_to.partition_point(|edge| *edge < to_edge);
            start..self.complex_to.len()
        } else {
            0..0
        };
        rows.take_while(move |row| self.complex_to[*row] == to_edge)
            .filter(move |row| self.complex_mask[*row] & profile_bit != 0)
            .map(move |row| {
                &self.complex_prefix_pool[self.complex_prefix_start[row] as usize
                    ..self.complex_prefix_start[row + 1] as usize]
            })
    }
}

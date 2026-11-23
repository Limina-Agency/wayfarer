//! One-to-many (matrix row) and bounded one-to-all (isochrone) Dijkstra over
//! the routing package.
//!
//! Semantics follow the former tile-backed matrix and isochrone expansions
//! (and `crate::reference` writes them out as the parity oracle): one label
//! per directed edge and complex-restriction state (see
//! `package::restrictions`), partially traversed seed edges (always part of
//! the path), turn costs between consecutive edges, simple and complex
//! restrictions, matrix destinations finished when the source node of a
//! destination edge is settled (never onto an edge already on the path).
//! Isochrones pay a U-turn onto the opposing edge like any turn, as routes
//! do (the tile-backed expansion banned it). Per-edge
//! block/avoid/multiplier options are exact cost overrides on the precomputed
//! column, as for routes. Two deliberate differences:
//!
//! - A matrix row keeps searching until every target's best finish is proven
//!   optimal (the popped cost reaches it), where the tile-backed expansion
//!   stopped at each target's *first* finish. Results are exact shortest
//!   paths, so a cell can be cheaper, never more expensive.
//! - Loop checks on expansion are dropped: with one label per edge and state
//!   and non-negative costs, an edge already on the settled path is only
//!   relabelled in another restriction state, where a complex restriction
//!   made the loop necessary.

use std::collections::BinaryHeap;

use super::{HeapEntry, NO_LABEL, SEED_TRAVERSED_FRACTION_EPSILON, SearchState, tie_window};
use crate::costing::{CostingPolicy, EdgeCostDecision};
use crate::package::SearchGraph;
use crate::package::cch::{GatedPotential, PotentialScratch};
use crate::package::columns::CostColumn;
use crate::package::profile_bit;
use crate::package::restrictions::{ComplexAutomata, ROOT};
use crate::package::search::gates::{DiscountGate, discount_gates, gate_limit};
use crate::types::{MobilityProfile, RouteOptions};
use rustc_hash::FxHashMap;

/// Above this many open target nodes a matrix row runs plain Dijkstra until
/// enough targets are settled: each bound costs a pass over every target.
const MAX_POTENTIAL_TARGET_NODES: usize = 8;
/// Open target nodes up to which a matrix row bounds with CCH potentials
/// (one elimination-tree query for all of them);
/// [`crate::debug::cch_row_targets`] overrides it.
fn cch_max_target_nodes() -> usize {
    crate::debug::cch_row_targets().unwrap_or(MAX_POTENTIAL_TARGET_NODES)
}

/// Lower bound on the cost from a node to the closest of a set of target
/// nodes: CCH potentials towards every open target at once, with the
/// request's discount gates (see `gates`), or none.
enum TargetPotential<'g> {
    /// No goal direction: plain Dijkstra.
    None,
    /// CCH potentials (`package::cch`), towards every open target at once.
    Cch(GatedPotential<'g>),
}

impl TargetPotential<'_> {
    #[inline]
    fn bound(&self, node: u32) -> f64 {
        match self {
            Self::None => 0.0,
            Self::Cch(potential) => potential.get(node),
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) struct AnalysisLabel {
    edge: u32,
    pred: u32,
    cost: f64,
    duration_s: f64,
    distance_m: f64,
}

/// A correlated start or end point on a compact edge.
#[derive(Debug, Clone, Copy)]
pub(crate) struct AnalysisPoint {
    pub edge: u32,
    pub fraction: f64,
    /// Approach cost of the candidate (see `search::approach_costs`): in its
    /// seed or finish, never in a reported cost.
    pub approach: f64,
}

impl AnalysisPoint {
    /// The points of one endpoint's candidates on compact edges `edges`.
    pub(crate) fn of_candidates(
        candidates: &[crate::types::CorrelatedLocation],
        edges: impl IntoIterator<Item = u32>,
    ) -> Vec<Self> {
        candidates
            .iter()
            .zip(edges)
            .zip(super::approach_costs(candidates))
            .map(|((candidate, edge), approach)| Self {
                edge,
                fraction: candidate.fraction_along_edge,
                approach,
            })
            .collect()
    }
}

pub(crate) struct AnalysisQuery<'c> {
    pub column: CostColumn<'c>,
    pub profile_bit: u8,
    pub max_visited_edge_count: Option<usize>,
    /// Stops each search like the budget; the request reports `Cancelled`.
    pub cancellation: &'c crate::RouteCancellation,
    /// Exact request costs for the edges named by block/avoid/multiplier
    /// options (see [`request_cost_overrides`]); every other edge keeps its
    /// column cost.
    pub cost_overrides: FxHashMap<u32, f32>,
    /// Edges the overrides make cheaper than the column: gates of the
    /// potentials (see `gates`).
    pub gates: Vec<DiscountGate>,
    /// The SAR `turn_restrictions` penalty per violated restriction (`None`:
    /// restrictions ban; see `SearchQuery::restriction_penalty`).
    pub restriction_penalty: Option<f64>,
}

impl AnalysisQuery<'_> {
    /// The request's cost of `edge`: its override, else the column cost.
    #[inline]
    pub(crate) fn edge_cost(&self, _graph: SearchGraph<'_>, edge: u32) -> f32 {
        let base = self.column.cost(edge);
        if self.cost_overrides.is_empty() {
            return base;
        }
        self.cost_overrides.get(&edge).copied().unwrap_or(base)
    }
}

/// Exact request-specific costs for every compact edge named by the request's
/// block/avoid/multiplier options, computed with the request's own costing
/// policy from the package's raw costing inputs (`CostingPolicy` over the
/// stored attributes and conditional tags), so they equal the road graph
/// record's costing edge for edge. Blocked or inaccessible edges cost
/// infinity. Shared by routes, matrices and isochrones.
pub(crate) fn request_cost_overrides(
    graph: SearchGraph<'_>,
    options: &RouteOptions,
    costing: &CostingPolicy<'_>,
    profile: MobilityProfile,
) -> FxHashMap<u32, f32> {
    let edge_ids = options
        .blocked_edge_ids
        .iter()
        .chain(options.avoided_edge_ids.iter())
        .chain(options.edge_cost_multipliers.keys())
        .copied()
        .chain(
            options
                .blocked_edge_ranges
                .iter()
                .map(|range| range.edge_id),
        )
        .chain(
            options
                .avoided_edge_ranges
                .iter()
                .map(|range| range.edge_id),
        )
        .chain(
            options
                .edge_cost_multiplier_ranges
                .iter()
                .map(|range| range.edge_id),
        )
        .chain(duration_changing_edge_ids(options));
    let mut overrides = FxHashMap::default();
    for edge_id in edge_ids {
        let Some(edge) = graph.edge_for_id(edge_id) else {
            continue;
        };
        if overrides.contains_key(&edge) {
            continue;
        }
        overrides.insert(edge, request_edge_cost(graph, costing, profile, edge));
    }
    overrides
}

/// Edges whose duration the request may change: speed overrides, discounts
/// (multipliers below 1) and opened edges. Slow-go multipliers, blocks and
/// avoidances change costs only.
fn duration_changing_edge_ids(options: &RouteOptions) -> impl Iterator<Item = u64> + '_ {
    options
        .edge_speed_overrides
        .keys()
        .copied()
        .chain(
            options
                .edge_speed_override_ranges
                .iter()
                .map(|range| range.edge_id),
        )
        .chain(options.opened_edge_ids.iter().copied())
        .chain(
            options
                .edge_cost_multipliers
                .iter()
                .filter(|(_, multiplier)| **multiplier < 1.0)
                .map(|(edge_id, _)| *edge_id),
        )
        .chain(
            options
                .edge_cost_multiplier_ranges
                .iter()
                .filter(|range| range.multiplier < 1.0)
                .map(|range| range.edge_id),
        )
}

/// The request's duration overrides: conditional tags at its evaluation time
/// (see [`conditional_overrides`], which also fills their cost overrides),
/// then every edge whose duration a speed override, a discount or an opening
/// changes, costed by the request's own policy. Without such options and
/// conditional tags it is empty, so the column's durations stand.
pub(crate) fn request_duration_overrides(
    graph: SearchGraph<'_>,
    column: CostColumn<'_>,
    options: &RouteOptions,
    costing: &CostingPolicy<'_>,
    profile: MobilityProfile,
    cost_overrides: &mut FxHashMap<u32, f32>,
) -> FxHashMap<u32, f32> {
    let mut durations =
        conditional_overrides(graph, column, options, costing, profile, cost_overrides);
    for edge_id in duration_changing_edge_ids(options) {
        let Some(edge) = graph.edge_for_id(edge_id) else {
            continue;
        };
        if let EdgeCostDecision::Allowed(costed) = graph.edge_cost_decision(
            costing,
            edge,
            profile_bit(profile),
            profile.mobility_mode_mask(),
        ) {
            durations.insert(edge, costed.duration_s as f32);
        }
    }
    durations
}

/// Conditional tags at the request's evaluation time as sparse overrides:
/// request columns ignore them (see `package::columns`), so each edge that
/// carries them is costed here with the request's policy, tags included, and
/// its cost (unless `cost_overrides` already has it, costed the same way)
/// and duration override the column's where they differ. Returns the
/// duration overrides; none without an evaluation time.
pub(crate) fn conditional_overrides(
    graph: SearchGraph<'_>,
    column: CostColumn<'_>,
    options: &RouteOptions,
    costing: &CostingPolicy<'_>,
    profile: MobilityProfile,
    cost_overrides: &mut FxHashMap<u32, f32>,
) -> FxHashMap<u32, f32> {
    let mut durations = FxHashMap::default();
    if options.costing.conditional_restriction_eval_at_ms.is_none() {
        return durations;
    }
    for edge in graph.conditional_edges {
        let (cost, duration) = match graph.edge_cost_decision(
            costing,
            *edge,
            profile_bit(profile),
            profile.mobility_mode_mask(),
        ) {
            EdgeCostDecision::Allowed(costed) => (costed.cost as f32, costed.duration_s as f32),
            EdgeCostDecision::Blocked | EdgeCostDecision::NotAccessible => {
                (f32::INFINITY, f32::INFINITY)
            }
        };
        if cost.to_bits() != column.cost(*edge).to_bits() {
            cost_overrides.entry(*edge).or_insert(cost);
        }
        if duration.to_bits() != column.duration(*edge).to_bits() {
            durations.insert(*edge, duration);
        }
    }
    durations
}

/// The request's cost of compact edge `edge` (infinity when blocked or
/// inaccessible).
pub(crate) fn request_edge_cost(
    graph: SearchGraph<'_>,
    costing: &CostingPolicy<'_>,
    profile: MobilityProfile,
    edge: u32,
) -> f32 {
    match graph.edge_cost_decision(
        costing,
        edge,
        profile_bit(profile),
        profile.mobility_mode_mask(),
    ) {
        EdgeCostDecision::Allowed(costed) => costed.cost as f32,
        EdgeCostDecision::Blocked | EdgeCostDecision::NotAccessible => f32::INFINITY,
    }
}

/// The compact edge of edge `edge_id`, if the package holds it. Builds the
/// edge id lookup on first use: for edge ids a request names.
pub(crate) fn resolve_edge_id(graph: SearchGraph<'_>, edge_id: u64) -> Option<u32> {
    graph.edge_for_id(edge_id)
}

/// The compact edge of a correlated location: the one correlation recorded
/// when it is this package's edge with the location's id, else through the
/// lookup ([`resolve_edge_id`]).
pub(crate) fn resolve_location(
    graph: SearchGraph<'_>,
    location: &crate::types::CorrelatedLocation,
) -> Option<u32> {
    graph.edge_for_id_hinted(location.compact_edge.0, location.edge_id)
}

/// Split edges of a dynamic overlay leave the static graph: each gets an
/// infinite override, and its usable fragments are costed per unit fraction
/// by the request's own policy, which carries the partial-block multiplier.
/// Returns that per-fraction cost for every split edge the graph holds.
pub(crate) fn overlay_split_costs(
    graph: SearchGraph<'_>,
    overlay: &crate::types::RouteDynamicOverlay,
    costing: &CostingPolicy<'_>,
    profile: MobilityProfile,
    overrides: &mut FxHashMap<u32, f32>,
) -> FxHashMap<u32, f32> {
    let mut split_cost = FxHashMap::default();
    for split in &overlay.split_edge_ranges {
        let Some(edge) = resolve_edge_id(graph, split.edge_id) else {
            continue;
        };
        split_cost.insert(edge, request_edge_cost(graph, costing, profile, edge));
        overrides.insert(edge, f32::INFINITY);
    }
    split_cost
}

/// Goal-direction bounds that stay admissible and consistent under
/// per-request overrides (see [`override_bounds`]).
pub(crate) struct OverrideBounds {
    /// The column's straight-line factor, lowered for the overrides.
    pub heuristic_cost_per_m: f64,
    /// Edges the request makes cheaper than the column, bounded through
    /// gates (see `gates`); empty in the fallback.
    pub gates: Vec<DiscountGate>,
    /// Whether the request needed the fallback (too many gates): only the
    /// straight-line factor, scaled, bounds it then.
    fallback: bool,
}

impl OverrideBounds {
    /// Whether the request needed the fallback.
    pub(crate) fn fell_back(&self) -> bool {
        self.fallback
    }
}

/// Goal-direction bounds that stay admissible and consistent under
/// `overrides` and a patch's split fragment costs (`split_cost`).
///
/// Every edge they make cheaper than its column cost, or usable where the
/// column is not, becomes a discount gate (`gates`): the potentials stay at
/// scale 1 and searches add the gates' terms, so a route that does not use a
/// discount searches as without it. The straight-line factor drops to the
/// least cost per metre of those edges.
///
/// With more than `gates::MAX_GATE_NODES` gate nodes on a side (a whole
/// region discounted), the fallback: with `r = min override / column cost`
/// over the edges an override lowers, every path costs at least `r` times its
/// column cost, so the straight-line factor is scaled by `r`; an edge the
/// column cannot use lowers the factor to that edge's cost per metre. (The
/// route searches' potentials scale the same way, see
/// `search::potential_plan`.)
pub(crate) fn override_bounds(
    graph: SearchGraph<'_>,
    column: CostColumn<'_>,
    overrides: &FxHashMap<u32, f32>,
    split_cost: &FxHashMap<u32, f32>,
) -> OverrideBounds {
    override_bounds_with_limit(graph, column, overrides, split_cost, gate_limit())
}

/// [`override_bounds`] with at most `max_gate_nodes` gate nodes per side.
pub(crate) fn override_bounds_with_limit(
    graph: SearchGraph<'_>,
    column: CostColumn<'_>,
    overrides: &FxHashMap<u32, f32>,
    split_cost: &FxHashMap<u32, f32>,
    max_gate_nodes: usize,
) -> OverrideBounds {
    /// Safety margin on the scale against rounding in f32 weights.
    const MARGIN: f64 = 1.0 - 1e-6;
    let span = |edge: u32| {
        let index = edge as usize;
        f64::from(graph.edge_length[index]).max(crate::package::chord_m(
            graph.xyz(graph.edge_source(index)),
            graph.xyz(graph.edge_target[index]),
        ))
    };
    if let Some(gates) = discount_gates(graph, column, overrides, split_cost, max_gate_nodes) {
        let mut heuristic_cost_per_m = column.heuristic_cost_per_m;
        for gate in &gates {
            let span = span(gate.edge);
            if span > 0.0 {
                heuristic_cost_per_m = heuristic_cost_per_m.min(gate.cost / span * 0.999);
            }
        }
        // Every edge also costs at least the least ratio times its column
        // cost when the column can use every gate.
        let ratio = gates.iter().map(|gate| gate.ratio).fold(1.0, f64::min);
        if ratio > 0.0 {
            heuristic_cost_per_m =
                heuristic_cost_per_m.max(column.heuristic_cost_per_m * ratio * MARGIN);
        }
        return OverrideBounds {
            heuristic_cost_per_m,
            gates,
            fallback: false,
        };
    }
    let mut heuristic_cost_per_m = column.heuristic_cost_per_m;
    let mut ratio = 1.0f64;
    for (edge, cost) in overrides.iter().chain(split_cost) {
        let base = column.cost(*edge);
        if !cost.is_finite() || *cost >= base {
            continue;
        }
        if base.is_finite() {
            if base > 0.0 {
                ratio = ratio.min(f64::from(*cost) / f64::from(base) * MARGIN);
            }
        } else {
            let span = span(*edge);
            if span > 0.0 {
                heuristic_cost_per_m = heuristic_cost_per_m.min(f64::from(*cost) / span * 0.999);
            }
        }
    }
    OverrideBounds {
        heuristic_cost_per_m: heuristic_cost_per_m * ratio.max(0.0),
        gates: Vec::new(),
        fallback: true,
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub(crate) struct AnalysisCounters {
    pub visited_edge_count: usize,
    pub settled_edge_count: usize,
}

/// A destination candidate of matrix target `target`.
#[derive(Debug, Clone, Copy)]
pub(crate) struct MatrixTargetCandidate {
    pub target: usize,
    pub point: AnalysisPoint,
}

#[derive(Debug, Clone)]
pub(crate) struct AnalysisPath {
    /// Compact edges in order: the origin edge first, the destination edge last.
    pub edges: Vec<u32>,
    pub cost: f64,
    pub distance_m: f64,
    pub duration_s: f64,
}

#[derive(Debug)]
pub(crate) struct MatrixRow {
    /// One slot per target index.
    pub paths: Vec<Option<AnalysisPath>>,
    pub counters: AnalysisCounters,
    pub budget_exceeded: bool,
    /// When the row stopped early (`stop_after`): each target it left open,
    /// with a lower bound on its path's cost. Those targets have no path in
    /// `paths`, found or not.
    pub open_bounds: Vec<(usize, f64)>,
}

/// The outcome of one matrix cell searched on its own.
#[derive(Debug)]
pub(crate) struct MatrixCell {
    pub path: Option<AnalysisPath>,
    pub counters: AnalysisCounters,
    pub budget_exceeded: bool,
}

/// One isochrone contour: a limit on cost or on duration.
#[derive(Debug, Clone, Copy)]
pub(crate) struct AnalysisContour {
    pub limit: f64,
    pub on_duration: bool,
}

/// An edge reached within some contour, with the least cost and the least
/// duration at its end over every relaxation that reached it.
#[derive(Debug, Clone, Copy)]
pub(crate) struct ReachedEdge {
    pub edge: u32,
    pub cost: f64,
    pub duration_s: f64,
}

/// Where contour `contour` crosses a relaxed edge: the edge is reached from
/// `start_fraction` (at `start_metric`) up to `end_fraction`.
#[derive(Debug, Clone, Copy)]
pub(crate) struct BoundaryCrossing {
    pub contour: usize,
    pub edge: u32,
    pub start_fraction: f64,
    pub end_fraction: f64,
    pub start_metric: f64,
}

#[derive(Debug, Default)]
pub(crate) struct IsochroneOutcome {
    pub reached: Vec<ReachedEdge>,
    pub crossings: Vec<BoundaryCrossing>,
    pub counters: AnalysisCounters,
    /// The search stopped at `max_visited_edge_count`.
    pub truncated: bool,
}

struct Finish {
    label: u32,
    candidate: usize,
    cost: f64,
    /// The junction delay of turning onto the destination edge.
    junction_s: f64,
}

struct Expansion<'a, 'g> {
    g: SearchGraph<'g>,
    q: &'a AnalysisQuery<'a>,
    automata: &'g ComplexAutomata,
    state: &'a mut SearchState,
    seq: u32,
}

impl Expansion<'_, '_> {
    #[inline]
    fn cost(&self, edge: u32) -> f64 {
        f64::from(self.q.edge_cost(self.g, edge))
    }

    #[inline]
    fn duration(&self, edge: u32) -> f64 {
        f64::from(self.q.column.duration(edge))
    }

    /// The junction delay added to durations where `turn_cost` is added to
    /// costs (`SearchGraph::junction_delay`).
    #[inline]
    fn junction_delay(&self, previous: u32, next: u32) -> f64 {
        self.g.junction_delay(&self.q.column, previous, next)
    }

    /// The forward restriction state of label `index`. Labels are kept
    /// small: only states other than the root are stored, by label, and
    /// only labels on edges inside some forbidden sequence can have one.
    #[inline]
    fn label_state(&self, index: u32) -> u32 {
        let states = &self.state.analysis_label_states;
        if states.is_empty()
            || !self
                .automata
                .has_states(self.state.analysis_labels[index as usize].edge)
        {
            return ROOT;
        }
        states.get(&index).copied().unwrap_or(ROOT)
    }

    /// The restriction state of a path in `state` that traverses `edge`.
    #[inline]
    fn next_state(&self, state: u32, edge: u32) -> u32 {
        self.automata.forward_next(&self.g, state, edge)
    }

    /// Labels `edge` in restriction `state` if `cost` improves on its
    /// current label there, queued at `priority`; returns whether it did.
    #[allow(clippy::too_many_arguments)]
    fn push(
        &mut self,
        edge: u32,
        pred: u32,
        cost: f64,
        duration_s: f64,
        distance_m: f64,
        priority: f64,
        state: u32,
    ) -> bool {
        let slot = self.state.slot(edge, state);
        if slot != NO_LABEL && self.state.analysis_labels[slot as usize].cost <= cost {
            return false;
        }
        let label = self.state.analysis_labels.len() as u32;
        self.state.analysis_labels.push(AnalysisLabel {
            edge,
            pred,
            cost,
            duration_s,
            distance_m,
        });
        if state != ROOT {
            self.state.analysis_label_states.insert(label, state);
        }
        self.state.set_slot(edge, state, label);
        self.seq = self.seq.wrapping_add(1);
        self.state.heap.push(HeapEntry {
            priority,
            seq: self.seq,
            label,
        });
        true
    }

    /// Pops the next live label, its restriction state and its queue
    /// priority, skipping superseded heap entries.
    fn pop(&mut self) -> Option<(u32, AnalysisLabel, u32, f64)> {
        while let Some(entry) = self.state.heap.pop() {
            let label = self.state.analysis_labels[entry.label as usize];
            let state = self.label_state(entry.label);
            if self.state.slot(label.edge, state) == entry.label {
                return Some((entry.label, label, state, entry.priority));
            }
        }
        None
    }

    /// Re-queues every live label at `cost + potential(end node)`, dropping
    /// superseded entries.
    fn rekey(&mut self, potential: &TargetPotential<'_>) {
        let entries = std::mem::take(&mut self.state.heap).into_vec();
        let mut live = Vec::with_capacity(entries.len());
        for mut entry in entries {
            let label = self.state.analysis_labels[entry.label as usize];
            if self.state.slot(label.edge, self.label_state(entry.label)) != entry.label {
                continue;
            }
            entry.priority = label.cost + potential.bound(self.g.edge_target[label.edge as usize]);
            // The potentials prove no open target is reachable from here.
            if entry.priority.is_finite() {
                live.push(entry);
            }
        }
        self.state.heap = BinaryHeap::from(live);
    }

    /// Whether a path in restriction `state` ending with `from_edge` may not
    /// continue onto `to_edge`.
    #[inline]
    fn restricted(&self, state: u32, from_edge: u32, to_edge: u32) -> bool {
        self.g
            .simple_restricted(from_edge, to_edge, self.q.profile_bit)
            || self.automata.forbids(state, to_edge)
    }

    /// The restriction cost of a path in `state`, ending with `from_edge`
    /// (`traversed`: else an untraversed seed, outside every simple
    /// restriction), continuing onto `to_edge`: `None` when banned, else the
    /// SAR penalty of the restrictions it breaks.
    #[inline]
    fn restriction_penalty(
        &self,
        state: u32,
        from_edge: u32,
        traversed: bool,
        to_edge: u32,
    ) -> Option<f64> {
        match self.q.restriction_penalty {
            None => {
                let restricted = if traversed {
                    self.restricted(state, from_edge, to_edge)
                } else {
                    self.automata.forbids(state, to_edge)
                };
                (!restricted).then_some(0.0)
            }
            Some(penalty) => {
                let simple = traversed
                    && self
                        .g
                        .simple_restricted(from_edge, to_edge, self.q.profile_bit);
                let count = u32::from(simple) + self.automata.forbidden_count(state, to_edge);
                Some(penalty * f64::from(count))
            }
        }
    }

    fn path_contains(&self, label: u32, edge: u32) -> bool {
        let mut current = label;
        while current != NO_LABEL {
            let entry = self.state.analysis_labels[current as usize];
            if entry.edge == edge {
                return true;
            }
            current = entry.pred;
        }
        false
    }

    fn out_edges(&self, node: u32) -> std::ops::Range<u32> {
        self.g.first_out[node as usize]..self.g.first_out[node as usize + 1]
    }
}

/// The cheapest direct path from an origin to a destination ahead of it on
/// the same edge, with its cost including both approach costs (the path's
/// own cost leaves them out). It competes with the searched paths, which may
/// start or end on nearer candidates, and wins ties.
pub(super) fn direct_same_edge(
    graph: SearchGraph<'_>,
    query: &AnalysisQuery<'_>,
    origins: &[AnalysisPoint],
    destinations: impl Iterator<Item = AnalysisPoint> + Clone,
) -> Option<(f64, AnalysisPath)> {
    let mut best = None::<(f64, AnalysisPath)>;
    for origin in origins {
        for destination in destinations.clone() {
            if origin.edge != destination.edge || origin.fraction > destination.fraction {
                continue;
            }
            let cost = f64::from(query.edge_cost(graph, origin.edge));
            if !cost.is_finite() {
                continue;
            }
            let fraction = (destination.fraction - origin.fraction).clamp(0.0, 1.0);
            let path = AnalysisPath {
                edges: vec![origin.edge],
                cost: cost * fraction,
                distance_m: f64::from(graph.edge_length[origin.edge as usize]) * fraction,
                duration_s: f64::from(query.column.duration(origin.edge)) * fraction,
            };
            let total = path.cost + origin.approach + destination.approach;
            if best.as_ref().is_none_or(|(existing, _)| total < *existing) {
                best = Some((total, path));
            }
        }
    }
    best
}

/// One matrix cell as a point-to-point search: the route engine's
/// bidirectional A* search between the source's and the target's candidates,
/// so the cell agrees with the route between the same two points.
/// `heuristic_cost_per_m` must already be scaled for the query's overrides
/// (see [`override_bounds`]).
#[allow(clippy::too_many_arguments)]
pub(crate) fn matrix_cell(
    graph: SearchGraph<'_>,
    state: &mut SearchState,
    query: &AnalysisQuery<'_>,
    heuristic_cost_per_m: f64,
    origins: &[crate::types::CorrelatedLocation],
    origin_points: &[AnalysisPoint],
    destinations: &[crate::types::CorrelatedLocation],
    destination_points: &[AnalysisPoint],
) -> MatrixCell {
    let direct = direct_same_edge(
        graph,
        query,
        origin_points,
        destination_points.iter().copied(),
    );
    let origin_edges = origin_points
        .iter()
        .map(|point| point.edge)
        .collect::<Vec<_>>();
    let destination_edges = destination_points
        .iter()
        .map(|point| point.edge)
        .collect::<Vec<_>>();
    let route_query = super::SearchQuery {
        column: query.column,
        profile_bit: query.profile_bit,
        heuristic_cost_per_m,
        origins,
        destinations,
        origin_edges: &origin_edges,
        destination_edges: &destination_edges,
        cost_overrides: &query.cost_overrides,
        max_visited_edge_count: query.max_visited_edge_count,
        cancellation: query.cancellation,
        gates: &query.gates,
        bidirectional: true,
        restriction_penalty: query.restriction_penalty,
        pieces: None,
    };
    let mut counters = super::SearchCounters::default();
    let outcome = super::run(graph, state, &route_query, &mut counters);
    state.reset();
    let counters = AnalysisCounters {
        visited_edge_count: counters.visited_edge_count,
        settled_edge_count: counters.settled_edge_count,
    };
    let searched_total = |path: &super::SearchPath| {
        path.cost
            + origin_points[path.origin_index].approach
            + destination_points[path.destination_index].approach
    };
    // Out of budget, a cell keeps its best path so far, as a row does.
    let (outcome, budget_exceeded) = match outcome {
        super::SearchOutcome::BudgetExceeded {
            best: Some(path), ..
        } => (super::SearchOutcome::Found(path), true),
        outcome => {
            let exceeded = matches!(outcome, super::SearchOutcome::BudgetExceeded { .. });
            (outcome, exceeded)
        }
    };
    match (outcome, direct) {
        (super::SearchOutcome::Found(path), Some((total, direct)))
            if total <= searched_total(&path) + tie_window(searched_total(&path)) =>
        {
            MatrixCell {
                path: Some(direct),
                counters,
                budget_exceeded,
            }
        }
        (super::SearchOutcome::Found(path), _) => MatrixCell {
            path: Some(AnalysisPath {
                edges: path.edges,
                cost: path.cost,
                distance_m: path.distance_m,
                duration_s: path.duration_s,
            }),
            counters,
            budget_exceeded,
        },
        (_, Some((_, direct))) => MatrixCell {
            path: Some(direct),
            counters,
            budget_exceeded,
        },
        (super::SearchOutcome::NoPath, None) => MatrixCell {
            path: None,
            counters,
            budget_exceeded: false,
        },
        (super::SearchOutcome::BudgetExceeded { .. }, None) => MatrixCell {
            path: None,
            counters,
            budget_exceeded: true,
        },
    }
}

/// Shortest paths from one matrix source (its correlated candidates) to every
/// target (each a set of candidates), sharing one expansion across targets.
///
/// With `stop_after`, the row stops once that many targets have their final
/// path (targets settle in the order of their costs with both approach
/// costs, ties together), and reports a lower bound for each target still
/// open (see [`MatrixRow::open_bounds`]). Rows that stop are searched with
/// no approach costs, so a bound holds for every candidate pair.
pub(crate) fn matrix_row(
    graph: SearchGraph<'_>,
    state: &mut SearchState,
    query: &AnalysisQuery<'_>,
    origins: &[AnalysisPoint],
    candidates: &[MatrixTargetCandidate],
    target_count: usize,
    stop_after: Option<usize>,
) -> MatrixRow {
    let scratch = std::mem::take(&mut state.cch_scratch).map(std::cell::RefCell::new);
    let row = matrix_row_with(
        graph,
        state,
        query,
        origins,
        candidates,
        target_count,
        stop_after,
        &scratch,
    );
    state.cch_scratch = scratch.map(std::cell::RefCell::into_inner);
    row
}

/// [`matrix_row`] with the CCH potential scratch taken out of `state`.
#[allow(clippy::too_many_arguments)]
fn matrix_row_with(
    graph: SearchGraph<'_>,
    state: &mut SearchState,
    query: &AnalysisQuery<'_>,
    origins: &[AnalysisPoint],
    candidates: &[MatrixTargetCandidate],
    target_count: usize,
    stop_after: Option<usize>,
    scratch: &[std::cell::RefCell<PotentialScratch>; 4],
) -> MatrixRow {
    let mut search = Expansion {
        g: graph,
        q: query,
        automata: graph.complex_automata(query.profile_bit),
        state,
        seq: 0,
    };
    let mut row = MatrixRow {
        paths: vec![None; target_count],
        counters: AnalysisCounters::default(),
        budget_exceeded: false,
        open_bounds: Vec::new(),
    };

    // A destination ahead of an origin on the same edge is reached directly;
    // that path (its cost with both approach costs) competes with the search
    // and wins ties.
    let mut searched = vec![false; target_count];
    for candidate in candidates {
        searched[candidate.target] = true;
    }
    let direct = (0..target_count)
        .map(|target| {
            let points = candidates
                .iter()
                .filter(move |candidate| candidate.target == target)
                .map(|candidate| candidate.point);
            direct_same_edge(graph, query, origins, points)
        })
        .collect::<Vec<_>>();
    let direct_total = direct
        .iter()
        .map(|direct| direct.as_ref().map_or(f64::INFINITY, |(total, _)| *total))
        .collect::<Vec<_>>();

    let mut candidates_by_node = FxHashMap::<u32, Vec<usize>>::default();
    for (index, candidate) in candidates.iter().enumerate() {
        if searched[candidate.target] {
            candidates_by_node
                .entry(graph.edge_source(candidate.point.edge as usize))
                .or_default()
                .push(index);
        }
    }
    let mut open_targets = searched.iter().filter(|searched| **searched).count();
    if open_targets == 0 {
        return row;
    }

    // Goal-directed: A* on a bound to the closest still-open target.
    // A target is settled once the queue's priority reaches its best finish
    // (every later finish costs at least that much); the potential is then
    // rebuilt over the remaining targets and the queue re-keyed. Raising a
    // consistent potential mid-search keeps settled labels exact, so the last,
    // most expensive target gets a focused search rather than a flood.
    let open_target_nodes = |open: &[bool]| {
        let mut nodes = candidates
            .iter()
            .filter(|candidate| open[candidate.target])
            .map(|candidate| graph.edge_source(candidate.point.edge as usize))
            .collect::<Vec<_>>();
        nodes.sort_unstable();
        nodes.dedup();
        nodes
    };
    let mut open = searched;
    let plan = super::potential_plan(
        graph,
        &query.column,
        &query.cost_overrides,
        !query.gates.is_empty(),
    );
    let potential_for = |open: &[bool]| match &plan {
        Some(plan) => {
            let targets = open_target_nodes(open);
            if targets.is_empty() || targets.len() > cch_max_target_nodes() {
                TargetPotential::None
            } else {
                TargetPotential::Cch(plan.potential(
                    &targets,
                    false,
                    &query.gates,
                    &scratch[0],
                    &scratch[2],
                ))
            }
        }
        None => TargetPotential::None,
    };
    let mut potential = potential_for(&open);

    // Seeds as in the route search: a seed that traverses nothing is outside
    // every restriction context and turns onto a destination edge for free.
    // `seeds` maps each seed label to its origin.
    let mut seeds = FxHashMap::<u32, usize>::default();
    for (index, origin) in origins.iter().enumerate() {
        let cost = search.cost(origin.edge);
        if !cost.is_finite() {
            continue;
        }
        let remaining = (1.0 - origin.fraction).clamp(0.0, 1.0);
        let start = cost * remaining + origin.approach;
        let priority = start + potential.bound(graph.edge_target[origin.edge as usize]);
        if !priority.is_finite() {
            continue;
        }
        let state = if remaining > SEED_TRAVERSED_FRACTION_EPSILON {
            search.next_state(ROOT, origin.edge)
        } else {
            ROOT
        };
        let label = search.state.analysis_labels.len() as u32;
        if search.push(
            origin.edge,
            NO_LABEL,
            start,
            search.duration(origin.edge) * remaining,
            f64::from(graph.edge_length[origin.edge as usize]) * remaining,
            priority,
            state,
        ) {
            seeds.insert(label, index);
        }
    }
    // Whether label `label` (on its own edge) traversed it.
    let traversed = |label: u32, analysis_labels: &[AnalysisLabel]| {
        analysis_labels[label as usize].pred != NO_LABEL
            || seeds.get(&label).is_some_and(|origin| {
                1.0 - origins[*origin].fraction.clamp(0.0, 1.0) > SEED_TRAVERSED_FRACTION_EPSILON
            })
    };
    // The traversed edges of the path at `label`.
    let traversed_path = |label: u32, analysis_labels: &[AnalysisLabel]| {
        let mut edges = Vec::new();
        let mut current = label;
        while current != NO_LABEL {
            if traversed(current, analysis_labels) {
                edges.push(analysis_labels[current as usize].edge);
            }
            current = analysis_labels[current as usize].pred;
        }
        edges.reverse();
        edges
    };
    // Whether finishing on `destination` repeats an edge already on the path
    // at `label`; the seed's own edge counts only where the two pieces
    // overlap (see `search::Run::finish_repeats`).
    let finish_repeats =
        |label: u32, destination: AnalysisPoint, analysis_labels: &[AnalysisLabel]| {
            let mut current = label;
            while current != NO_LABEL {
                let entry = analysis_labels[current as usize];
                if entry.edge == destination.edge && traversed(current, analysis_labels) {
                    return seeds.get(&current).is_none_or(|origin| {
                        origins[*origin].fraction.clamp(0.0, 1.0)
                            <= destination.fraction.clamp(0.0, 1.0)
                    });
                }
                current = entry.pred;
            }
            false
        };

    let mut best = (0..target_count)
        .map(|_| None::<Finish>)
        .collect::<Vec<_>>();
    // A target's best so far: its best finish or its direct path.
    let best_cost = |best: &[Option<Finish>], target: usize| {
        best[target]
            .as_ref()
            .map_or(f64::INFINITY, |finish| finish.cost)
            .min(direct_total[target])
    };
    // Least best among open targets: nothing settles before the queue
    // reaches it.
    let least_open_best = |best: &[Option<Finish>], open: &[bool]| {
        (0..best.len())
            .filter(|target| open[*target])
            .map(|target| best_cost(best, target))
            .fold(f64::INFINITY, f64::min)
    };
    let mut settle_at = least_open_best(&best, &open);
    // Targets settled with a path, and the priority the row stopped at.
    let mut found = 0;
    let mut stopped_at = None;
    // With `stop_after`: per target, the least cost of any finish evaluated.
    let mut floor = vec![
        f64::INFINITY;
        if stop_after.is_some() {
            target_count
        } else {
            0
        }
    ];
    'search: while let Some((label_index, label, label_state, priority)) = search.pop() {
        if priority >= settle_at {
            let was_open = open_targets;
            for (target, open) in open.iter_mut().enumerate() {
                // Ties are searched too, so the tie-break sees them all.
                let best = best_cost(&best, target);
                if *open && best + tie_window(best) < priority {
                    *open = false;
                    open_targets -= 1;
                    found += 1;
                }
            }
            if open_targets == 0 {
                break;
            }
            if stop_after.is_some_and(|after| found >= after) {
                stopped_at = Some(priority);
                break;
            }
            settle_at = least_open_best(&best, &open);
            // Within a best's tie window nothing settles yet: keep the queue.
            if open_targets < was_open {
                potential = potential_for(&open);
                search.rekey(&potential);
            }
            // This label was the minimum under the previous potential, so it
            // is exact and is settled below as usual.
        }
        row.counters.settled_edge_count += 1;
        let node = graph.edge_target[label.edge as usize];

        if let Some(at_node) = candidates_by_node.get(&node) {
            for &index in at_node {
                let candidate = candidates[index];
                if !open[candidate.target] {
                    continue;
                }
                let destination = candidate.point.edge;
                let labels = &search.state.analysis_labels;
                let label_traversed = traversed(label_index, labels);
                if finish_repeats(label_index, candidate.point, labels) {
                    continue;
                }
                let Some(penalty) = search.restriction_penalty(
                    label_state,
                    label.edge,
                    label_traversed,
                    destination,
                ) else {
                    continue;
                };
                let edge_cost = search.cost(destination);
                if !edge_cost.is_finite() {
                    continue;
                }
                let turn = if label_traversed {
                    graph.turn_cost(label.edge, destination)
                } else {
                    0.0
                };
                // A SAR barrier may close the transition.
                if !turn.is_finite() {
                    continue;
                }
                let junction_s = if label_traversed {
                    search.junction_delay(label.edge, destination)
                } else {
                    0.0
                };
                let cost = label.cost
                    + turn
                    + edge_cost * candidate.point.fraction.clamp(0.0, 1.0)
                    + candidate.point.approach
                    + penalty;
                if let Some(floor) = floor.get_mut(candidate.target) {
                    *floor = floor.min(cost);
                }
                // Ties go to the route search's order: fewer edges, then
                // lexicographically smaller ids.
                let key = |label: u32, destination: u32| {
                    let mut ids = traversed_path(label, labels)
                        .into_iter()
                        .map(|edge| graph.edge_id(edge as usize))
                        .collect::<Vec<_>>();
                    ids.push(graph.edge_id(destination as usize));
                    (ids.len(), ids)
                };
                let better = match &best[candidate.target] {
                    // The direct path wins ties.
                    _ if cost
                        >= direct_total[candidate.target]
                            - tie_window(direct_total[candidate.target]) =>
                    {
                        false
                    }
                    None => true,
                    Some(current) => {
                        cost < current.cost - tie_window(current.cost)
                            || ((cost - current.cost).abs() <= tie_window(current.cost)
                                && key(label_index, destination)
                                    < key(current.label, candidates[current.candidate].point.edge))
                    }
                };
                if better {
                    best[candidate.target] = Some(Finish {
                        label: label_index,
                        candidate: index,
                        cost,
                        junction_s,
                    });
                    settle_at = settle_at.min(cost);
                }
            }
        }

        for next in search.out_edges(node) {
            row.counters.visited_edge_count += 1;
            if query
                .max_visited_edge_count
                .is_some_and(|max| row.counters.visited_edge_count > max)
                || query
                    .cancellation
                    .interrupted_at(row.counters.visited_edge_count)
            {
                row.budget_exceeded = true;
                break 'search;
            }
            let Some(penalty) = search.restriction_penalty(
                label_state,
                label.edge,
                traversed(label_index, &search.state.analysis_labels),
                next,
            ) else {
                continue;
            };
            let edge_cost = search.cost(next);
            if !edge_cost.is_finite() {
                continue;
            }
            let cost = label.cost + edge_cost + graph.turn_cost(label.edge, next) + penalty;
            let duration_s =
                label.duration_s + search.junction_delay(label.edge, next) + search.duration(next);
            let priority = cost + potential.bound(graph.edge_target[next as usize]);
            // The potentials prove no open target is reachable through `next`; the
            // open set only shrinks, so that stays true.
            if !priority.is_finite() {
                continue;
            }
            search.push(
                next,
                label_index,
                cost,
                duration_s,
                label.distance_m + f64::from(graph.edge_length[next as usize]),
                priority,
                search.next_state(label_state, next),
            );
        }
    }

    // A target left open gets no path, only a lower bound on its cost: the
    // least of its finishes evaluated so far, its direct paths along one
    // edge, and any finish from a label popped later. Such a label lies at
    // the source node of one of the target's candidate edges, where the
    // potential is 0, so it costs at least the priority the row stopped at
    // (within a tie window). Without approach costs (see `RowStop`), that
    // bounds the path between any of the source's candidates and any of the
    // target's, here or on a pass whose edge costs are no lower.
    if let Some(priority) = stopped_at {
        let later = priority - tie_window(priority);
        for target in (0..target_count).filter(|target| open[*target]) {
            best[target] = None;
            let direct = candidates
                .iter()
                .filter(|candidate| candidate.target == target)
                .flat_map(|candidate| {
                    origins
                        .iter()
                        .filter(move |origin| {
                            origin.edge == candidate.point.edge
                                && origin.fraction <= candidate.point.fraction
                        })
                        .map(move |origin| {
                            f64::from(query.edge_cost(graph, origin.edge))
                                * (candidate.point.fraction - origin.fraction).clamp(0.0, 1.0)
                        })
                })
                .fold(f64::INFINITY, f64::min);
            let bound = later.min(floor[target]).min(direct);
            row.open_bounds.push((target, bound - tie_window(bound)));
        }
    }
    for (target, finish) in best.into_iter().enumerate() {
        if stopped_at.is_some() && open[target] {
            continue;
        }
        let Some(finish) = finish else {
            row.paths[target] = direct[target].as_ref().map(|(_, path)| path.clone());
            continue;
        };
        let labels = &search.state.analysis_labels;
        let label = labels[finish.label as usize];
        let destination = candidates[finish.candidate].point;
        let fraction = destination.fraction.clamp(0.0, 1.0);
        let mut seed = finish.label;
        while labels[seed as usize].pred != NO_LABEL {
            seed = labels[seed as usize].pred;
        }
        let origin_approach = seeds
            .get(&seed)
            .map_or(0.0, |origin| origins[*origin].approach);
        let mut edges = traversed_path(finish.label, labels);
        edges.push(destination.edge);
        row.paths[target] = Some(AnalysisPath {
            edges,
            cost: finish.cost - origin_approach - destination.approach,
            distance_m: label.distance_m
                + f64::from(graph.edge_length[destination.edge as usize]) * fraction,
            duration_s: label.duration_s
                + finish.junction_s
                + search.duration(destination.edge) * fraction,
        });
    }
    row
}

/// Records that `edge` was reached at (`cost`, `duration_s`), keeping the least
/// of each. `edge_aux` maps a reached edge to its entry; every reached edge is
/// also labelled, so the state's touched list resets it.
fn reach(
    state: &mut SearchState,
    reached: &mut Vec<ReachedEdge>,
    edge: u32,
    cost: f64,
    duration_s: f64,
) {
    let slot = state.edge_aux.get_mut(edge);
    if *slot == NO_LABEL {
        *slot = reached.len() as u32;
        reached.push(ReachedEdge {
            edge,
            cost,
            duration_s,
        });
    } else {
        let entry = &mut reached[*slot as usize];
        entry.cost = entry.cost.min(cost);
        entry.duration_s = entry.duration_s.min(duration_s);
    }
}

/// Every edge reachable from the origin candidates within the largest
/// contour, plus the points where each contour crosses a relaxed edge.
pub(crate) fn isochrone(
    graph: SearchGraph<'_>,
    state: &mut SearchState,
    query: &AnalysisQuery<'_>,
    origins: &[AnalysisPoint],
    contours: &[AnalysisContour],
) -> IsochroneOutcome {
    let max_cost = contours
        .iter()
        .filter(|contour| !contour.on_duration)
        .map(|contour| contour.limit)
        .fold(0.0, f64::max);
    let max_duration = contours
        .iter()
        .filter(|contour| contour.on_duration)
        .map(|contour| contour.limit)
        .fold(0.0, f64::max);
    let within = |cost: f64, duration_s: f64| {
        (max_cost > 0.0 && cost <= max_cost) || (max_duration > 0.0 && duration_s <= max_duration)
    };
    state.edge_aux.ensure_len(state.edge_label.capacity());
    let mut search = Expansion {
        g: graph,
        q: query,
        automata: graph.complex_automata(query.profile_bit),
        state,
        seq: 0,
    };
    let mut outcome = IsochroneOutcome::default();
    let cross = |crossings: &mut Vec<BoundaryCrossing>,
                 edge: u32,
                 start_fraction: f64,
                 (start_cost, end_cost): (f64, f64),
                 (start_duration, end_duration): (f64, f64)| {
        for (index, contour) in contours.iter().enumerate() {
            let (start, end) = if contour.on_duration {
                (start_duration, end_duration)
            } else {
                (start_cost, end_cost)
            };
            if !(start < contour.limit && contour.limit < end) || end - start <= f64::EPSILON {
                continue;
            }
            let ratio = ((contour.limit - start) / (end - start)).clamp(0.0, 1.0);
            crossings.push(BoundaryCrossing {
                contour: index,
                edge,
                start_fraction,
                end_fraction: start_fraction + (1.0 - start_fraction) * ratio,
                start_metric: start,
            });
        }
    };

    for origin in origins {
        let edge_cost = search.cost(origin.edge);
        if !edge_cost.is_finite() {
            continue;
        }
        // A farther candidate starts at its approach cost. The approach is a
        // penalty, not travel, so time starts at zero (as in matrix rows).
        let remaining = (1.0 - origin.fraction).clamp(0.0, 1.0);
        let cost = origin.approach + edge_cost * remaining;
        let duration_s = search.duration(origin.edge) * remaining;
        cross(
            &mut outcome.crossings,
            origin.edge,
            origin.fraction,
            (origin.approach, cost),
            (0.0, duration_s),
        );
        // A seed is labelled even beyond every contour (its successors are
        // then out of range too), but only reported when within one.
        let state = search.next_state(ROOT, origin.edge);
        search.push(origin.edge, NO_LABEL, cost, duration_s, 0.0, cost, state);
        if within(cost, duration_s) {
            reach(
                search.state,
                &mut outcome.reached,
                origin.edge,
                cost,
                duration_s,
            );
        }
    }

    'search: while let Some((label_index, label, label_state, _)) = search.pop() {
        outcome.counters.settled_edge_count += 1;
        let node = graph.edge_target[label.edge as usize];
        for next in search.out_edges(node) {
            outcome.counters.visited_edge_count += 1;
            if query
                .max_visited_edge_count
                .is_some_and(|max| outcome.counters.visited_edge_count > max)
                || query
                    .cancellation
                    .interrupted_at(outcome.counters.visited_edge_count)
            {
                outcome.truncated = true;
                break 'search;
            }
            // Isochrone seeds count as traversed.
            let Some(penalty) = search.restriction_penalty(label_state, label.edge, true, next)
            else {
                continue;
            };
            let edge_cost = search.cost(next);
            if !edge_cost.is_finite() {
                continue;
            }
            // A SAR restriction penalty is paid at the junction, like the turn.
            let turn = graph.turn_cost(label.edge, next) + penalty;
            if !turn.is_finite() {
                continue;
            }
            let cost = label.cost + edge_cost + turn;
            // The junction's time is spent before entering `next`, as its turn
            // cost is.
            let entered_s = label.duration_s + search.junction_delay(label.edge, next);
            let duration_s = entered_s + search.duration(next);
            // Re-entering an edge already on this path is never an
            // improvement (it was reached cheaper), so it only matters when it
            // would record a second boundary crossing on that edge, which the
            // tile-backed expansion never produced. Walk the path only then.
            let crosses = contours.iter().any(|contour| {
                let (start, end) = if contour.on_duration {
                    (entered_s, duration_s)
                } else {
                    (label.cost + turn, cost)
                };
                start < contour.limit && contour.limit < end && end - start > f64::EPSILON
            });
            if crosses && search.path_contains(label_index, next) {
                continue;
            }
            cross(
                &mut outcome.crossings,
                next,
                0.0,
                (label.cost + turn, cost),
                (entered_s, duration_s),
            );
            if !within(cost, duration_s) {
                continue;
            }
            let state = search.next_state(label_state, next);
            search.push(next, label_index, cost, duration_s, 0.0, cost, state);
            reach(search.state, &mut outcome.reached, next, cost, duration_s);
        }
    }
    outcome
}

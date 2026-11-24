//! Dynamic overlays on the routing package: a per-request patch of virtual
//! nodes and arcs over the static graph.
//!
//! - Split edges are removed (their cost is an infinite override); each usable
//!   range becomes a chain of sub-edge arcs between *stops*: the range ends and
//!   every connector attachment inside it. A stop at fraction 0 or 1 is the
//!   edge's real end node, any other stop a virtual point node.
//! - Unsplit edges with interior connector attachments keep their real arc and
//!   gain the same chain of sub-edge arcs.
//! - Connectors are virtual arcs with their given cost; endpoint connectors
//!   attach to virtual origin and destination nodes.
//!
//! A sub-edge arc costs `fraction x` the edge's request cost (for a partly
//! blocked edge that cost carries the costing policy's partial-block
//! multiplier). Arcs that enter an edge at a real node pay the turn cost from
//! the previous real edge and obey simple and complex restrictions; a
//! connector starts a new restriction context. Labels are kept per arc and
//! complex-restriction state of that context (see `package::restrictions`;
//! consecutive arcs along one edge count as that edge once), so every
//! search here is exact under complex restrictions.
//!
//! Each search adds its own arcs to the patch: *seeds* from each origin point
//! to the first stop ahead of it, *finishes* from each stop (or edge start)
//! behind a destination point to that point, and an *end* arc out of the
//! destination endpoint. Every path is then a chain of arcs from a seed to a
//! finish, and its cost is fixed by consecutive pairs of arcs (the arc's cost
//! plus the turn between them), apart from complex restrictions, which need
//! the path's restriction context.
//!
//! The route search is a bidirectional, arc-based A* with average potentials,
//! like the static search (`package/search.rs`): forward labels on arcs (cost
//! to the arc's end), backward labels on arcs (cost from entering the arc to
//! the destination), joined at transitions. Both sides use admissible lower
//! bounds on the patched graph (see [`Bounds`]): the straight-line
//! bound with the factor lowered to the cheapest ratio any patch arc
//! realises, raised to connector-aware CCH potentials when they apply.
//! Results are exact shortest paths under these semantics (the former
//! tile-backed overlay search was not: its heuristic was in metres while
//! costs are in seconds, and it ignored complex restrictions); the reference
//! oracle (`crate::reference`) checks them.

use rustc_hash::FxHashMap;
use wayfarer_package::format::ColumnPieceKinds;

use super::gates::{DiscountGate, gate_ends};
use super::{HeapEntry, NO_LABEL, SearchState, tie_window};
use crate::package::cch::{CchPotential, GateDistances, PotentialScratch};
use crate::package::columns::CostColumn;
use crate::package::restrictions::{ComplexAutomata, ROOT};
use crate::package::search::{PotentialPlan, potential_plan};
use crate::package::{
    EDGE_FLAG_COMPLEX_RESTRICTION_TO, MeasuredPolyline, SearchGraph, chord_m, unit_xyz,
};
use crate::types::{
    CorrelatedLocation, GeoPoint, RouteDynamicEndpointKind, RouteDynamicOverlay,
    RouteDynamicVirtualEdge,
};

mod walk;

const FRACTION_EPSILON: f64 = f64::EPSILON;
/// A seed shorter than this traverses nothing (legacy semantics).
const TRAVERSED_EPSILON: f64 = 1e-9;
const NO_NODE: u32 = u32::MAX;

/// One step of a patched path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum PatchStep {
    /// A whole compact edge.
    Edge(u32),
    /// Part of compact edge `edge`; `split` marks overlay fragments.
    Fragment {
        edge: u32,
        start: f64,
        end: f64,
        split: bool,
    },
    /// `overlay.virtual_edges[index]`.
    Connector(usize),
}

#[derive(Debug, Clone, Copy)]
enum ArcKind {
    /// Part of a compact edge, between two stops (or, for a seed, from an
    /// origin point to the next stop).
    Fragment {
        edge: u32,
        start: f64,
        end: f64,
        split: bool,
    },
    Connector(usize),
    /// From a stop (or the edge's start) to destination candidate
    /// `destination` on `edge`.
    Finish {
        destination: usize,
        edge: u32,
        start: f64,
        end: f64,
        split: bool,
    },
    /// Out of the destination endpoint node: the path is complete.
    End,
}

#[derive(Debug, Clone, Copy)]
struct Arc {
    /// `NO_NODE` for a seed (and a direct same-edge path).
    from: u32,
    /// `NO_NODE` for a finish or end arc.
    to: u32,
    kind: ArcKind,
    cost: f64,
    duration_s: f64,
    distance_m: f64,
}

impl Arc {
    /// The compact edge an arc runs along (`None` for a connector or end).
    fn edge(&self) -> Option<u32> {
        match self.kind {
            ArcKind::Fragment { edge, .. } | ArcKind::Finish { edge, .. } => Some(edge),
            ArcKind::Connector(_) | ArcKind::End => None,
        }
    }

    /// Entering the edge at its real start node: turn costs and restrictions
    /// apply.
    fn enters_edge(&self) -> bool {
        matches!(
            self.kind,
            ArcKind::Fragment { start, .. } | ArcKind::Finish { start, .. }
                if start <= FRACTION_EPSILON
        )
    }

    /// Whether the arc counts as traversing its edge in a restriction
    /// context (a seed that covers nothing does not).
    fn traversed(&self) -> bool {
        match self.kind {
            ArcKind::Fragment { start, end, .. } => {
                self.from != NO_NODE || end - start > TRAVERSED_EPSILON
            }
            ArcKind::Finish { start, end, .. } => end - start > TRAVERSED_EPSILON,
            ArcKind::Connector(_) | ArcKind::End => false,
        }
    }

    fn is_terminal(&self) -> bool {
        matches!(self.kind, ArcKind::Finish { .. } | ArcKind::End)
    }
}

/// The request's edge costs, as the patch sees them.
pub(crate) struct PatchCosts<'c> {
    pub column: CostColumn<'c>,
    /// Exact request cost per compact edge (overrides, else the column);
    /// split edges are infinite here.
    pub edge_cost: &'c (dyn Fn(u32) -> f32 + Sync),
    /// Cost of a split edge's usable fragments per unit fraction.
    pub split_cost: &'c FxHashMap<u32, f32>,
}

impl PatchCosts<'_> {
    /// Cost per unit fraction of a fragment of `edge`.
    fn fragment_cost(&self, edge: u32) -> f64 {
        f64::from(
            self.split_cost
                .get(&edge)
                .copied()
                .unwrap_or_else(|| (self.edge_cost)(edge)),
        )
    }
}

pub(crate) struct PatchGraph<'o> {
    overlay: &'o RouteDynamicOverlay,
    node_count: u32,
    edge_count: u32,
    /// Unit vectors of the virtual nodes (index - node_count).
    positions: Vec<[f64; 3]>,
    /// The edge and fraction of each virtual point node (`None` for endpoints).
    point_edges: Vec<Option<(u32, f64)>>,
    points: FxHashMap<(u32, i64), u32>,
    origin_endpoint: Option<u32>,
    destination_endpoint: Option<u32>,
    /// Usable ranges of split edges.
    split: FxHashMap<u32, Vec<(f64, f64)>>,
    /// Sorted stops (fraction, node) along each edge with fragments.
    stops: FxHashMap<u32, Vec<(f64, u32)>>,
    arcs: Vec<Arc>,
    out_arcs: FxHashMap<u32, Vec<u32>>,
    in_arcs: FxHashMap<u32, Vec<u32>>,
}

fn fraction_key(fraction: f64) -> i64 {
    (fraction.clamp(0.0, 1.0) * 1_000_000.0).round() as i64
}

impl<'o> PatchGraph<'o> {
    /// Builds the patch for `overlay`; `resolve` maps an archive edge id to
    /// its compact edge.
    pub(crate) fn new(
        graph: SearchGraph<'_>,
        overlay: &'o RouteDynamicOverlay,
        costs: &PatchCosts<'_>,
        resolve: &dyn Fn(u64) -> Option<u32>,
    ) -> Self {
        let mut patch = Self {
            overlay,
            node_count: graph.node_count as u32,
            edge_count: graph.edge_count as u32,
            positions: Vec::new(),
            point_edges: Vec::new(),
            points: FxHashMap::default(),
            origin_endpoint: None,
            destination_endpoint: None,
            split: FxHashMap::default(),
            stops: FxHashMap::default(),
            arcs: Vec::new(),
            out_arcs: FxHashMap::default(),
            in_arcs: FxHashMap::default(),
        };
        let mut shape = MeasuredPolyline::default();
        let mut attachments = Vec::<(u32, f64)>::new();
        for split in &overlay.split_edge_ranges {
            let Some(edge) = resolve(split.edge_id) else {
                continue;
            };
            let ranges = split
                .usable_ranges
                .iter()
                .map(|range| {
                    (
                        range.start_fraction.clamp(0.0, 1.0),
                        range.end_fraction.clamp(0.0, 1.0),
                    )
                })
                .filter(|(start, end)| end > start)
                .collect::<Vec<_>>();
            for (start, end) in &ranges {
                attachments.push((edge, *start));
                attachments.push((edge, *end));
            }
            patch.split.insert(edge, ranges);
        }
        // Connector ends: endpoint nodes or points on edges.
        let mut connector_ends = Vec::with_capacity(overlay.virtual_edges.len());
        for connector in &overlay.virtual_edges {
            let source = match connector.source_endpoint {
                Some(kind) => patch.endpoint(kind, connector.source_point),
                None => connector.source_edge_id.and_then(resolve).map(|edge| {
                    let fraction = connector.source_fraction.unwrap_or(1.0).clamp(0.0, 1.0);
                    if connector.source_fraction.is_some() {
                        attachments.push((edge, fraction));
                    }
                    patch.node_at(graph, &mut shape, edge, fraction)
                }),
            };
            let target = match connector.target_endpoint {
                Some(kind) => patch.endpoint(kind, connector.target_point),
                None => connector.target_edge_id.and_then(resolve).map(|edge| {
                    let fraction = connector.target_fraction.unwrap_or(0.0).clamp(0.0, 1.0);
                    if connector.target_fraction.is_some() {
                        attachments.push((edge, fraction));
                    }
                    patch.node_at(graph, &mut shape, edge, fraction)
                }),
            };
            connector_ends.push(source.zip(target));
        }
        // Stops: every attachment inside a usable range of its edge.
        let mut by_edge = FxHashMap::<u32, Vec<f64>>::default();
        for (edge, fraction) in attachments {
            by_edge.entry(edge).or_default().push(fraction);
        }
        let mut by_edge = by_edge.into_iter().collect::<Vec<_>>();
        by_edge.sort_by_key(|(edge, _)| *edge);
        for (edge, mut fractions) in by_edge {
            fractions.sort_by(f64::total_cmp);
            fractions.dedup_by_key(|fraction| fraction_key(*fraction));
            let ranges = patch
                .split
                .get(&edge)
                .cloned()
                .unwrap_or_else(|| vec![(0.0, 1.0)]);
            let mut stops = Vec::new();
            for (start, end) in ranges {
                let inside = fractions
                    .iter()
                    .copied()
                    .filter(|fraction| {
                        *fraction + FRACTION_EPSILON >= start && *fraction <= end + FRACTION_EPSILON
                    })
                    .collect::<Vec<_>>();
                // An unsplit edge needs fragments only for interior stops.
                if !patch.split.contains_key(&edge)
                    && inside.iter().all(|fraction| {
                        *fraction <= FRACTION_EPSILON || *fraction >= 1.0 - FRACTION_EPSILON
                    })
                {
                    continue;
                }
                let mut range_stops = std::iter::once(start)
                    .chain(inside)
                    .chain(std::iter::once(end))
                    .map(|fraction| fraction.clamp(start, end))
                    .collect::<Vec<_>>();
                range_stops.sort_by(f64::total_cmp);
                range_stops.dedup_by_key(|fraction| fraction_key(*fraction));
                for pair in range_stops.windows(2) {
                    let from = patch.node_at(graph, &mut shape, edge, pair[0]);
                    let to = patch.node_at(graph, &mut shape, edge, pair[1]);
                    if from == to {
                        continue;
                    }
                    let fraction = pair[1] - pair[0];
                    patch.push_arc(Arc {
                        from,
                        to,
                        kind: ArcKind::Fragment {
                            edge,
                            start: pair[0],
                            end: pair[1],
                            split: true,
                        },
                        cost: costs.fragment_cost(edge) * fraction,
                        duration_s: f64::from(costs.column.duration(edge)) * fraction,
                        distance_m: f64::from(graph.edge_length[edge as usize]) * fraction,
                    });
                }
                for fraction in range_stops {
                    let node = patch.node_at(graph, &mut shape, edge, fraction);
                    stops.push((fraction, node));
                }
            }
            if !stops.is_empty() {
                stops.sort_by(|a, b| a.0.total_cmp(&b.0));
                patch.stops.insert(edge, stops);
            }
        }
        for (index, (connector, ends)) in
            overlay.virtual_edges.iter().zip(connector_ends).enumerate()
        {
            let Some((from, to)) = ends else {
                continue;
            };
            if from == to || !connector.cost.is_finite() || connector.cost < 0.0 {
                continue;
            }
            patch.push_arc(Arc {
                from,
                to,
                kind: ArcKind::Connector(index),
                cost: connector.cost,
                duration_s: connector.cost,
                distance_m: connector.distance_m,
            });
        }
        patch
    }

    /// Whether the patch adds nothing to the graph (only full blocks).
    pub(crate) fn is_empty(&self) -> bool {
        self.arcs.is_empty()
            && self.origin_endpoint.is_none()
            && self.destination_endpoint.is_none()
            && self.split.values().all(|ranges| ranges.is_empty())
    }

    pub(crate) fn connector(&self, index: usize) -> &RouteDynamicVirtualEdge {
        &self.overlay.virtual_edges[index]
    }

    fn endpoint(&mut self, kind: RouteDynamicEndpointKind, point: Option<GeoPoint>) -> Option<u32> {
        let point = point?;
        let slot = match kind {
            RouteDynamicEndpointKind::Origin => &mut self.origin_endpoint,
            RouteDynamicEndpointKind::Destination => &mut self.destination_endpoint,
        };
        if let Some(node) = *slot {
            return Some(node);
        }
        let node = self.node_count + self.positions.len() as u32;
        self.positions.push(unit_xyz(point.lat, point.lng));
        self.point_edges.push(None);
        *slot = Some(node);
        Some(node)
    }

    fn node_at(
        &mut self,
        graph: SearchGraph<'_>,
        shape: &mut MeasuredPolyline,
        edge: u32,
        fraction: f64,
    ) -> u32 {
        if fraction <= FRACTION_EPSILON {
            return graph.edge_source(edge as usize);
        }
        if fraction >= 1.0 - FRACTION_EPSILON {
            return graph.edge_target[edge as usize];
        }
        let key = (edge, fraction_key(fraction));
        if let Some(node) = self.points.get(&key) {
            return *node;
        }
        shape.load_edge(&graph, edge);
        let position = shape
            .point_at_fraction(fraction)
            .map(|point| unit_xyz(point.lat, point.lng))
            .unwrap_or_else(|| graph.xyz(graph.edge_source(edge as usize)));
        let node = self.node_count + self.positions.len() as u32;
        self.positions.push(position);
        self.point_edges.push(Some((edge, fraction)));
        self.points.insert(key, node);
        node
    }

    fn push_arc(&mut self, arc: Arc) {
        let index = self.arcs.len() as u32;
        self.out_arcs.entry(arc.from).or_default().push(index);
        self.in_arcs.entry(arc.to).or_default().push(index);
        self.arcs.push(arc);
    }

    fn position(&self, graph: &SearchGraph<'_>, node: u32) -> [f64; 3] {
        if node < self.node_count {
            graph.xyz(node)
        } else {
            self.positions[(node - self.node_count) as usize]
        }
    }

    /// Whether fragments lead from `fraction` on `edge` to the edge's end.
    fn reaches_target(&self, edge: u32, fraction: f64) -> bool {
        self.split.get(&edge).is_none_or(|ranges| {
            ranges.iter().any(|(start, end)| {
                fraction + FRACTION_EPSILON >= *start && *end >= 1.0 - FRACTION_EPSILON
            })
        })
    }

    /// Whether fragments lead from the edge's start to `fraction` on `edge`.
    fn reached_from_source(&self, edge: u32, fraction: f64) -> bool {
        self.split.get(&edge).is_none_or(|ranges| {
            ranges.iter().any(|(start, end)| {
                *start <= FRACTION_EPSILON && fraction <= *end + FRACTION_EPSILON
            })
        })
    }

    /// Seed arcs for `origins` (each with its origin index), or the origin
    /// endpoint's connectors (origin `u32::MAX`) when the overlay has one.
    fn seeds(
        &self,
        graph: SearchGraph<'_>,
        costs: &PatchCosts<'_>,
        origins: &[CorrelatedLocation],
        origin_edges: &[u32],
        approach: &[f64],
        arcs: &mut RunArcs,
    ) -> Vec<(u32, u32)> {
        let mut seeds = Vec::new();
        if let Some(origin) = self.origin_endpoint {
            for arc in self.out_arcs.get(&origin).into_iter().flatten() {
                seeds.push((*arc, u32::MAX));
            }
            return seeds;
        }
        // A seed costs its candidate's approach too (see
        // `search::approach_costs`); `PatchRun::reconstruct` takes it out.
        for (index, (location, edge)) in origins.iter().zip(origin_edges).enumerate() {
            let from = location.fraction_along_edge.clamp(0.0, 1.0);
            let unit_cost = costs.fragment_cost(*edge);
            if !unit_cost.is_finite() {
                continue;
            }
            let split = self.split.contains_key(edge);
            // The first stop at or after the origin within its usable range.
            let next = match self.stops.get(edge) {
                Some(stops) => {
                    let range = self
                        .split
                        .get(edge)
                        .and_then(|ranges| {
                            ranges.iter().copied().find(|(start, end)| {
                                from + FRACTION_EPSILON >= *start && from <= *end + FRACTION_EPSILON
                            })
                        })
                        .unwrap_or((0.0, 1.0));
                    if split
                        && !self.split[edge].iter().any(|(start, end)| {
                            from + FRACTION_EPSILON >= *start && from <= *end + FRACTION_EPSILON
                        })
                    {
                        continue;
                    }
                    // The next stop ahead; at the range end, that end itself
                    // (a seed always arrives along the edge, as in legacy).
                    let within = |fraction: f64| fraction <= range.1 + FRACTION_EPSILON;
                    stops
                        .iter()
                        .copied()
                        .find(|(fraction, _)| *fraction > from + 1e-9 && within(*fraction))
                        .or_else(|| {
                            stops
                                .iter()
                                .copied()
                                .find(|(fraction, _)| *fraction + 1e-9 >= from && within(*fraction))
                        })
                        .map(|(fraction, node)| (fraction.max(from), node, true))
                }
                None if split => None,
                None => Some((1.0, graph.edge_target[*edge as usize], false)),
            };
            let Some((to_fraction, node, fragment)) = next else {
                continue;
            };
            let fraction = (to_fraction - from).max(0.0);
            let arc = arcs.push(Arc {
                from: NO_NODE,
                to: node,
                kind: ArcKind::Fragment {
                    edge: *edge,
                    start: from,
                    end: to_fraction,
                    split: fragment && (split || to_fraction < 1.0 - FRACTION_EPSILON),
                },
                cost: unit_cost * fraction + approach[index],
                duration_s: f64::from(costs.column.duration(*edge)) * fraction,
                distance_m: f64::from(graph.edge_length[*edge as usize]) * fraction,
            });
            seeds.push((arc, index as u32));
        }
        seeds
    }

    /// Finish arcs into `destinations`, or the end arc out of the destination
    /// endpoint when the overlay has one.
    fn finishes(
        &self,
        graph: SearchGraph<'_>,
        costs: &PatchCosts<'_>,
        destinations: &[CorrelatedLocation],
        destination_edges: &[u32],
        approach: &[f64],
        arcs: &mut RunArcs,
    ) {
        if let Some(node) = self.destination_endpoint {
            arcs.push(Arc {
                from: node,
                to: NO_NODE,
                kind: ArcKind::End,
                cost: 0.0,
                duration_s: 0.0,
                distance_m: 0.0,
            });
            return;
        }
        // A finish costs its candidate's approach too, as a seed does.
        for (index, (location, edge)) in destinations.iter().zip(destination_edges).enumerate() {
            let end = location.fraction_along_edge.clamp(0.0, 1.0);
            let split = self.split.contains_key(edge);
            let unit = costs.fragment_cost(*edge);
            if !unit.is_finite() {
                continue;
            }
            let mut options = Vec::new();
            match self.stops.get(edge) {
                Some(stops) => {
                    let range = self.split.get(edge).and_then(|ranges| {
                        ranges.iter().copied().find(|(start, stop)| {
                            end + FRACTION_EPSILON >= *start && end <= *stop + FRACTION_EPSILON
                        })
                    });
                    if split && range.is_none() {
                        continue;
                    }
                    let (start, _) = range.unwrap_or((0.0, 1.0));
                    for (fraction, node) in stops {
                        if *fraction + FRACTION_EPSILON >= start && *fraction <= end + 1e-9 {
                            options.push((*fraction, *node, true));
                        }
                    }
                }
                None if split => continue,
                None => {}
            }
            if !split {
                options.push((0.0, graph.edge_source(*edge as usize), false));
            }
            for (start, node, fragment) in options {
                let start = start.min(end);
                let fraction = end - start;
                arcs.push(Arc {
                    from: node,
                    to: NO_NODE,
                    kind: ArcKind::Finish {
                        destination: index,
                        edge: *edge,
                        start,
                        end,
                        split: fragment && (split || start > FRACTION_EPSILON),
                    },
                    cost: unit * fraction + approach[index],
                    duration_s: f64::from(costs.column.duration(*edge)) * fraction,
                    distance_m: f64::from(graph.edge_length[*edge as usize]) * fraction,
                });
            }
        }
    }
}

/// The patch's arcs plus one search's seeds and finishes. Arc `i` of the
/// search is real edge `i` below `edge_count`, else patch arc
/// `i - edge_count` (the patch's own first, then the search's).
struct RunArcs<'a, 'o> {
    patch: &'a PatchGraph<'o>,
    extra: Vec<Arc>,
    /// Finish and end arcs by their start node.
    extra_out: FxHashMap<u32, Vec<u32>>,
    /// Seed arcs by their end node.
    extra_in: FxHashMap<u32, Vec<u32>>,
}

impl<'a, 'o> RunArcs<'a, 'o> {
    fn new(patch: &'a PatchGraph<'o>) -> Self {
        Self {
            patch,
            extra: Vec::new(),
            extra_out: FxHashMap::default(),
            extra_in: FxHashMap::default(),
        }
    }

    fn len(&self) -> usize {
        self.patch.arcs.len() + self.extra.len()
    }

    #[inline]
    fn arc(&self, index: u32) -> &Arc {
        let fixed = self.patch.arcs.len();
        if (index as usize) < fixed {
            &self.patch.arcs[index as usize]
        } else {
            &self.extra[index as usize - fixed]
        }
    }

    fn push(&mut self, arc: Arc) -> u32 {
        let index = self.len() as u32;
        if arc.from != NO_NODE && arc.is_terminal() {
            self.extra_out.entry(arc.from).or_default().push(index);
        } else if arc.from == NO_NODE && arc.to != NO_NODE {
            self.extra_in.entry(arc.to).or_default().push(index);
        }
        self.extra.push(arc);
        index
    }

    fn out_arcs(&self, node: u32) -> impl Iterator<Item = u32> + '_ {
        self.patch
            .out_arcs
            .get(&node)
            .into_iter()
            .flatten()
            .chain(self.extra_out.get(&node).into_iter().flatten())
            .copied()
    }

    fn in_arcs(&self, node: u32) -> impl Iterator<Item = u32> + '_ {
        self.patch
            .in_arcs
            .get(&node)
            .into_iter()
            .flatten()
            .chain(self.extra_in.get(&node).into_iter().flatten())
            .copied()
    }

    fn all(&self) -> impl Iterator<Item = &Arc> + '_ {
        self.patch.arcs.iter().chain(self.extra.iter())
    }
}

// ---------------------------------------------------------------------------
// Lower bounds
// ---------------------------------------------------------------------------

/// Tests: run the patch search as plain unidirectional Dijkstra (no lower
/// bounds), an oracle for the bounds' admissibility and the bidirectional
/// search's joins.
pub(crate) mod dijkstra {
    #[cfg(test)]
    thread_local! {
        static FORCED: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    }

    #[cfg(test)]
    pub(crate) fn force(value: bool) {
        FORCED.with(|flag| flag.set(value));
    }

    #[inline]
    pub(crate) fn forced() -> bool {
        #[cfg(test)]
        return FORCED.with(std::cell::Cell::get);
        #[cfg(not(test))]
        false
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
enum Gate {
    /// The connector starts (forward) or ends (backward) at a real node.
    Real(u32),
    /// The connector attaches at a virtual point of this edge.
    Edge(u32),
    /// The connector starts or ends at an endpoint node.
    Endpoint(u32),
}

#[derive(Clone, Copy, PartialEq)]
enum Side {
    /// Bounds on the cost from a node to the destination.
    Forward,
    /// Bounds on the cost from the origin to a node.
    Backward,
}

/// Lower bounds that survive the patch, for one side, on CCH potentials.
/// Every path from a real node to the destination either uses no connector,
/// and then reaches a final-approach node (the source of a destination
/// edge), or has a first connector `c`, reached through its *gate*: the
/// connector's own start node, or the source of the edge it attaches to. So
///
/// `pi(v) = min(pot(v, final), min over gates g of d(v, g) + off(g))`
///
/// where `off(g)` is the least `cost(c) + pi(end of c)` over the gate's
/// connectors: one elimination-tree query towards the final-approach nodes
/// at 0 and every gate's node at its offset. At a virtual point on edge `e`,
/// the path runs along `e` to its end (`pi(target) + cost of the rest of
/// e`), finishes on `e`, or leaves by a connector attached to `e`. The
/// offsets solve that system as a shortest path problem over gates, so `pi`
/// is a lower bound that is consistent on every arc, real or patch (the
/// minimum of consistent bounds is consistent, and `pi(start of c) <=
/// cost(c) + pi(end of c)` holds by construction). The backward side mirrors
/// this with connector ends as gates and the origin's first nodes in place
/// of the final approach. The request's discount gates join as gates too
/// (see `gates`).
struct PatchCch<'g> {
    potential: CchPotential<'g>,
    /// Offsets of edge gates, used at virtual points of the edge.
    edge_gates: FxHashMap<u32, f64>,
    /// Offsets of endpoint gates, used at the endpoint node itself.
    endpoint_gates: FxHashMap<u32, f64>,
}

/// A virtual node, as the bounds see it.
#[derive(Clone, Copy)]
enum VirtualNode {
    Point {
        edge: u32,
        fraction: f64,
        /// Fragments lead on to the edge's far end (forward: its target;
        /// backward: from its source).
        reaches_end: bool,
        /// The edge carries a destination (forward) or origin (backward).
        terminal: bool,
    },
    /// The side's own endpoint (forward: the destination endpoint).
    OwnEndpoint,
    OtherEndpoint,
}

/// Consistent lower bounds for one side of the search.
struct Bounds<'g> {
    graph: SearchGraph<'g>,
    side: Side,
    node_count: u32,
    positions: Vec<[f64; 3]>,
    virtual_nodes: Vec<VirtualNode>,
    column: CostColumn<'g>,
    center: [f64; 3],
    radius: f64,
    k: f64,
    potential: Option<PatchCch<'g>>,
    /// Request cost of each discount gate's edge, below its column cost.
    cheaper: FxHashMap<u32, f64>,
}

impl<'g> Bounds<'g> {
    #[allow(clippy::too_many_arguments)]
    fn new(
        graph: SearchGraph<'g>,
        arcs: &RunArcs<'_, '_>,
        side: Side,
        column: CostColumn<'g>,
        k: f64,
        goal_points: &[[f64; 3]],
        terminal_edges: &[u32],
        gates: &[DiscountGate],
        cch: Option<(
            &'g PotentialPlan<'g>,
            &'g std::cell::RefCell<PotentialScratch>,
        )>,
    ) -> Self {
        let patch = arcs.patch;
        let center = goal_points.first().copied().unwrap_or([0.0; 3]);
        let radius = goal_points
            .iter()
            .map(|point| chord_m(center, *point))
            .fold(0.0f64, f64::max);
        let own_endpoint = match side {
            Side::Forward => patch.destination_endpoint,
            Side::Backward => patch.origin_endpoint,
        };
        let virtual_nodes = patch
            .point_edges
            .iter()
            .enumerate()
            .map(|(index, point)| match point {
                Some((edge, fraction)) => VirtualNode::Point {
                    edge: *edge,
                    fraction: *fraction,
                    reaches_end: match side {
                        Side::Forward => patch.reaches_target(*edge, *fraction),
                        Side::Backward => patch.reached_from_source(*edge, *fraction),
                    },
                    terminal: terminal_edges.contains(edge),
                },
                None if Some(patch.node_count + index as u32) == own_endpoint => {
                    VirtualNode::OwnEndpoint
                }
                None => VirtualNode::OtherEndpoint,
            })
            .collect::<Vec<_>>();
        let mut bounds = Self {
            graph,
            side,
            node_count: patch.node_count,
            positions: patch.positions.clone(),
            virtual_nodes,
            column,
            center,
            radius,
            k,
            potential: None,
            cheaper: gates.iter().map(|gate| (gate.edge, gate.cost)).collect(),
        };
        if let Some((plan, scratch)) = cch {
            bounds.potential = bounds.build_cch(arcs, plan, scratch, terminal_edges, gates);
        }
        bounds
    }

    /// The real node at the far end of `edge` for this side, and a lower
    /// bound on the cost from a virtual point at `fraction` to it: the column
    /// cost, or a discount gate's lower request cost.
    #[inline]
    fn along(&self, edge: u32, fraction: f64) -> (u32, f64) {
        let cost = self
            .cheaper
            .get(&edge)
            .copied()
            .unwrap_or_else(|| f64::from(self.column.cost(edge)));
        match self.side {
            Side::Forward => (
                self.graph.edge_target[edge as usize],
                cost * (1.0 - fraction),
            ),
            Side::Backward => (self.graph.edge_source(edge as usize), cost * fraction),
        }
    }

    /// The bounds of [`PatchCch`]: every real or edge gate and every
    /// discount gate node is a slot of its own, its terms the CCH distances
    /// from a connector's far end (`GateDistances`).
    /// `None` with more slots than `cch::EXACT_GATE_NODES`.
    fn build_cch(
        &self,
        arcs: &RunArcs<'_, '_>,
        plan: &'g PotentialPlan<'g>,
        scratch: &'g std::cell::RefCell<PotentialScratch>,
        terminal_edges: &[u32],
        gates: &[DiscountGate],
    ) -> Option<PatchCch<'g>> {
        let g = self.graph;
        let reversed = self.side == Side::Backward;
        let topology = &plan.potentials.topology;
        let metric = plan.metric.metric();
        let scale = plan.factor * crate::package::cch::POTENTIAL_MARGIN;
        let mut finals = terminal_edges
            .iter()
            .map(|edge| match self.side {
                Side::Forward => g.edge_source(*edge as usize),
                Side::Backward => g.edge_target[*edge as usize],
            })
            .collect::<Vec<_>>();
        finals.sort_unstable();
        finals.dedup();
        let node_count = self.node_count;
        let gate_of = |node: u32| -> Gate {
            if node < node_count {
                Gate::Real(node)
            } else {
                match self.virtual_nodes[(node - node_count) as usize] {
                    VirtualNode::Point { edge, .. } => Gate::Edge(edge),
                    _ => Gate::Endpoint(node),
                }
            }
        };
        let connectors = arcs
            .all()
            .filter(|arc| matches!(arc.kind, ArcKind::Connector(_)))
            .map(|arc| match self.side {
                Side::Forward => (gate_of(arc.from), arc.to, arc.cost),
                Side::Backward => (gate_of(arc.to), arc.from, arc.cost),
            })
            .collect::<Vec<_>>();
        let mut gate_keys = connectors
            .iter()
            .map(|(gate, _, _)| *gate)
            .collect::<Vec<_>>();
        gate_keys.sort_unstable();
        gate_keys.dedup();
        // Slots with a real node: connector gates, then discount gate nodes;
        // then endpoint gates.
        let mut slot_nodes = Vec::<u32>::new();
        let mut slot_of = FxHashMap::<Gate, usize>::default();
        let mut edge_slots = Vec::<(u32, usize)>::new();
        for gate in &gate_keys {
            let node = match (gate, self.side) {
                (Gate::Real(node), _) => *node,
                (Gate::Edge(edge), Side::Forward) => g.edge_source(*edge as usize),
                (Gate::Edge(edge), Side::Backward) => g.edge_target[*edge as usize],
                (Gate::Endpoint(_), _) => continue,
            };
            if let Gate::Edge(edge) = gate {
                edge_slots.push((*edge, slot_nodes.len()));
            }
            slot_of.insert(*gate, slot_nodes.len());
            slot_nodes.push(node);
        }
        let mut discount_slot = FxHashMap::<u32, usize>::default();
        for gate in gates {
            let (node, _) = gate_ends(gate, reversed);
            discount_slot.entry(node).or_insert_with(|| {
                slot_nodes.push(node);
                slot_nodes.len() - 1
            });
        }
        if slot_nodes.len() > crate::package::cch::EXACT_GATE_NODES {
            return None;
        }
        let real_slots = slot_nodes.len();
        let mut endpoint_slots = Vec::<(u32, usize)>::new();
        for gate in &gate_keys {
            if let Gate::Endpoint(node) = gate {
                let slot = real_slots + endpoint_slots.len();
                slot_of.insert(*gate, slot);
                endpoint_slots.push((*node, slot));
            }
        }
        let slot_count = real_slots + endpoint_slots.len();

        let base = CchPotential::with_offsets(
            topology,
            metric,
            &finals,
            &[],
            reversed,
            plan.factor,
            scratch,
        );
        let distances = GateDistances::new(topology, metric, reversed, &slot_nodes);
        let real_terms = |node: u32| -> (f64, Vec<f64>) {
            let mut terms = distances
                .from(node)
                .into_iter()
                .map(|d| {
                    if d.is_finite() {
                        d * scale
                    } else {
                        f64::INFINITY
                    }
                })
                .collect::<Vec<_>>();
            terms.resize(slot_count, f64::INFINITY);
            (base.get(node), terms)
        };
        let discount_rows = gates.iter().map(|gate| {
            let (node, end) = gate_ends(gate, reversed);
            let (base, terms) = real_terms(end);
            (discount_slot[&node], gate.cost, base, terms)
        });
        let mut rows = Vec::new();
        for (gate, end, cost) in &connectors {
            let Some(slot) = slot_of.get(gate).copied() else {
                continue;
            };
            let (base, terms) = if *end < node_count {
                real_terms(*end)
            } else {
                match self.virtual_nodes[(*end - node_count) as usize] {
                    VirtualNode::Point {
                        edge,
                        fraction,
                        reaches_end,
                        terminal,
                    } => {
                        let mut base = if terminal { 0.0 } else { f64::INFINITY };
                        let mut terms = vec![f64::INFINITY; slot_count];
                        if reaches_end {
                            let (node, along) = self.along(edge, fraction);
                            let (far_base, far_terms) = real_terms(node);
                            base = base.min(along + far_base);
                            for (term, far) in terms.iter_mut().zip(far_terms) {
                                *term = along + far;
                            }
                        }
                        if let Some(own) = slot_of.get(&Gate::Edge(edge)) {
                            terms[*own] = 0.0;
                        }
                        (base, terms)
                    }
                    VirtualNode::OwnEndpoint => (0.0, vec![f64::INFINITY; slot_count]),
                    VirtualNode::OtherEndpoint => {
                        let mut terms = vec![f64::INFINITY; slot_count];
                        if let Some(own) = slot_of.get(&Gate::Endpoint(*end)) {
                            terms[*own] = 0.0;
                        }
                        (f64::INFINITY, terms)
                    }
                }
            };
            rows.push((slot, *cost, base, terms));
        }
        rows.extend(discount_rows);
        // Bellman-Ford over gates, as in `build_alt`.
        let mut offsets = vec![f64::INFINITY; slot_count];
        for _ in 0..=slot_count {
            let mut changed = false;
            for (slot, cost, base, terms) in &rows {
                let far = terms
                    .iter()
                    .zip(&offsets)
                    .map(|(term, offset)| term + offset)
                    .fold(*base, f64::min);
                let value = cost + far;
                if value < offsets[*slot] {
                    offsets[*slot] = value;
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
        let seeds = slot_nodes
            .iter()
            .zip(&offsets)
            .map(|(node, offset)| (*node, *offset))
            .collect::<Vec<_>>();
        let potential = CchPotential::with_offsets(
            topology,
            metric,
            &finals,
            &seeds,
            reversed,
            plan.factor,
            scratch,
        );
        Some(PatchCch {
            potential,
            edge_gates: edge_slots
                .iter()
                .map(|(edge, slot)| (*edge, offsets[*slot]))
                .collect(),
            endpoint_gates: endpoint_slots
                .iter()
                .map(|(node, slot)| (*node, offsets[*slot]))
                .collect(),
        })
    }

    fn position(&self, node: u32) -> [f64; 3] {
        if node < self.node_count {
            self.graph.xyz(node)
        } else {
            self.positions[(node - self.node_count) as usize]
        }
    }

    /// The lower bound at `node` (infinite: the goal is unreachable).
    fn at(&self, node: u32) -> f64 {
        let geometric = self.k * (chord_m(self.position(node), self.center) - self.radius).max(0.0);
        let Some(cch) = &self.potential else {
            return geometric;
        };
        let bound = if node < self.node_count {
            cch.potential.get(node)
        } else {
            match self.virtual_nodes[(node - self.node_count) as usize] {
                VirtualNode::Point {
                    edge,
                    fraction,
                    reaches_end,
                    terminal,
                } => {
                    let mut value = if terminal { 0.0 } else { f64::INFINITY };
                    if let Some(offset) = cch.edge_gates.get(&edge) {
                        value = value.min(*offset);
                    }
                    if reaches_end && value > 0.0 {
                        let (far, along) = self.along(edge, fraction);
                        value = value.min(along + cch.potential.get(far));
                    }
                    value
                }
                VirtualNode::OwnEndpoint => 0.0,
                VirtualNode::OtherEndpoint => cch
                    .endpoint_gates
                    .get(&node)
                    .copied()
                    .unwrap_or(f64::INFINITY),
            }
        };
        if bound.is_finite() {
            geometric.max(bound)
        } else {
            f64::INFINITY
        }
    }
}

// ---------------------------------------------------------------------------
// Route search
// ---------------------------------------------------------------------------

pub(crate) struct PatchQuery<'q> {
    pub costs: PatchCosts<'q>,
    /// Whether the CCH potentials, where the package has them, bound the
    /// request: false in the scaled fallback (see `analysis::override_bounds`),
    /// which does not cover the virtual points either.
    pub potentials: bool,
    /// Edges and split edges the request makes cheaper than the column:
    /// gates of the potentials (see `gates`).
    pub gates: &'q [DiscountGate],
    pub profile_bit: u16,
    pub origins: &'q [CorrelatedLocation],
    pub destinations: &'q [CorrelatedLocation],
    pub origin_edges: &'q [u32],
    pub destination_edges: &'q [u32],
    /// The origin each origin candidate belongs to (a multi-origin
    /// isochrone's), or empty for one endpoint's candidates: approach costs
    /// are per origin.
    pub origin_targets: &'q [usize],
    /// The target of each destination (a matrix row's), or empty for one
    /// endpoint's candidates: approach costs are per target.
    pub destination_targets: &'q [usize],
    pub max_visited_edge_count: Option<usize>,
    /// Stops the search like the budget; the request reports `Cancelled`.
    pub cancellation: &'q crate::RouteCancellation,
    /// The column's straight-line factor, already lowered for request
    /// overrides (see `analysis::override_bounds`).
    pub heuristic_cost_per_m: f64,
    /// Search from both ends (default) or forward only.
    pub bidirectional: bool,
    /// The SAR `turn_restrictions` penalty per violated restriction (`None`:
    /// restrictions ban). With a penalty the patch searches forward only.
    pub restriction_penalty: Option<f64>,
    /// The stored column's piece kinds: the bidirectional search walks
    /// through the marked nodes of pieces the patch does not touch (see
    /// `walk`). `None` keeps the per-edge search; set only where the static
    /// search's walks would be exact.
    pub pieces: Option<ColumnPieceKinds<'q>>,
}

impl<'q> PatchQuery<'q> {
    /// The approach costs of the origin and destination candidates (see
    /// `search::approach_costs`), each relative to the nearest candidate of
    /// its own origin or target.
    fn approaches(&self) -> (Vec<f64>, Vec<f64>) {
        let by_target = |candidates, targets: &[usize]| {
            if targets.is_empty() {
                super::approach_costs(candidates)
            } else {
                super::approach_costs_by_target(candidates, targets)
            }
        };
        (
            by_target(self.origins, self.origin_targets),
            by_target(self.destinations, self.destination_targets),
        )
    }

    /// The same query between other origins and destinations.
    pub(crate) fn between<'a>(
        &'a self,
        origins: &'a [CorrelatedLocation],
        origin_edges: &'a [u32],
        destinations: &'a [CorrelatedLocation],
        destination_edges: &'a [u32],
    ) -> PatchQuery<'a>
    where
        'q: 'a,
    {
        PatchQuery {
            costs: PatchCosts {
                column: self.costs.column,
                edge_cost: self.costs.edge_cost,
                split_cost: self.costs.split_cost,
            },
            potentials: self.potentials,
            gates: self.gates,
            profile_bit: self.profile_bit,
            origins,
            destinations,
            origin_edges,
            destination_edges,
            origin_targets: &[],
            destination_targets: &[],
            max_visited_edge_count: self.max_visited_edge_count,
            cancellation: self.cancellation,
            heuristic_cost_per_m: self.heuristic_cost_per_m,
            bidirectional: self.bidirectional,
            restriction_penalty: self.restriction_penalty,
            pieces: self.pieces,
        }
    }
}

#[derive(Debug)]
pub(crate) struct PatchPath {
    pub steps: Vec<PatchStep>,
    /// Origin candidate index, or `None` when the path starts at the origin
    /// endpoint node.
    pub origin_index: Option<usize>,
    /// Destination candidate index, or `None` for the destination endpoint.
    pub destination_index: Option<usize>,
    pub cost: f64,
    pub distance_m: f64,
    pub duration_s: f64,
}

#[derive(Debug)]
pub(crate) enum PatchOutcome {
    Found(PatchPath),
    NoPath,
    /// The budget ran out; a route search keeps its best path so far with
    /// how much dearer than optimal it can be at most (by the stop's own
    /// lower bound).
    BudgetExceeded {
        max_visited_edge_count: usize,
        best: Option<(PatchPath, f64)>,
    },
}

#[derive(Debug, Default, Clone, Copy)]
pub(crate) struct PatchCounters {
    pub visited_edge_count: usize,
    pub settled_edge_count: usize,
    pub restricted_transition_count: usize,
}

/// Forward label: the path from a seed to the end of `arc`.
#[derive(Clone, Copy)]
struct Label {
    /// Real edge id (< edge_count) or `edge_count + patch arc`.
    arc: u32,
    pred: u32,
    cost: f64,
    duration_s: f64,
    distance_m: f64,
    /// Last real edge (turn and restriction context), `NO_NODE` after a
    /// connector.
    last_edge: u32,
    /// Whether this arc traversed part of `last_edge` (a seed may not).
    traversed: bool,
    origin: u32,
    /// Forward complex-restriction state of the context since the last
    /// connector.
    state: u32,
    /// The first edge of the walk that ended on `arc`, or `NO_LABEL` (see
    /// `walk`); with `back`, the walk turned back at that edge's end onto
    /// `arc`.
    via: u32,
    back: bool,
}

/// Backward label: the path from entering `arc` to the destination.
#[derive(Clone, Copy)]
struct BackwardLabel {
    arc: u32,
    /// Successor label (towards the destination); `NO_LABEL` for a finish.
    next: u32,
    cost: f64,
    duration_s: f64,
    distance_m: f64,
    /// Backward complex-restriction state of the chain's context up to the
    /// first connector.
    state: u32,
    /// The last edge (in path order) of the backward walk that ended on
    /// `arc`, or `NO_LABEL` (see `walk`); with `back`, the walk turned back
    /// onto it at `arc`'s end.
    via: u32,
    back: bool,
}

/// Everything the search needs about one arc.
#[derive(Clone, Copy)]
struct ArcInfo {
    tail: u32,
    head: u32,
    /// The compact edge it runs along, `NO_NODE` for a connector or end.
    edge: u32,
    enters: bool,
    traversed: bool,
    terminal: bool,
    cost: f64,
    duration_s: f64,
    distance_m: f64,
}

enum Best {
    /// Origin and destination on the same stretch of one edge.
    Direct { arc: u32, origin: u32, cost: f64 },
    /// A forward label run to a finish arc (unidirectional).
    Finish { label: u32, arc: u32, cost: f64 },
    /// A forward and a backward label joined (bidirectional).
    Meet {
        forward: u32,
        backward: u32,
        cost: f64,
    },
}

impl Best {
    fn cost(&self) -> f64 {
        match self {
            Best::Direct { cost, .. } | Best::Finish { cost, .. } | Best::Meet { cost, .. } => {
                *cost
            }
        }
    }
}

/// Lowers the straight-line factor `k` so it never exceeds `cost / chord`.
fn lower(k: &mut f64, cost: f64, chord: f64) {
    if chord > 0.0 {
        *k = k.min(cost / chord * 0.999);
    }
}

pub(crate) fn run(
    graph: SearchGraph<'_>,
    state: &mut SearchState,
    patch: &PatchGraph<'_>,
    query: &PatchQuery<'_>,
    counters: &mut PatchCounters,
) -> PatchOutcome {
    let costs = &query.costs;
    let mut arcs = RunArcs::new(patch);
    let (origin_approach, destination_approach) = query.approaches();
    let seeds = patch.seeds(
        graph,
        costs,
        query.origins,
        query.origin_edges,
        &origin_approach,
        &mut arcs,
    );
    patch.finishes(
        graph,
        costs,
        query.destinations,
        query.destination_edges,
        &destination_approach,
        &mut arcs,
    );

    // Direct: origin and destination on the same stretch of one edge.
    let mut best: Option<Best> = None;
    if patch.origin_endpoint.is_none() && patch.destination_endpoint.is_none() {
        for (origin, _, arc, cost) in patch.direct_paths(
            graph,
            costs,
            query,
            (&origin_approach, &destination_approach),
            &mut arcs,
        ) {
            if best.as_ref().is_none_or(|best| cost < best.cost()) {
                best = Some(Best::Direct {
                    arc,
                    origin: origin as u32,
                    cost,
                });
            }
        }
    }

    // Straight-line factor: the column's, lowered to every patch arc's,
    // seed's and finish's cost per metre of chord.
    let mut k = query.heuristic_cost_per_m;
    let origin_points = match patch.origin_endpoint {
        Some(node) => vec![patch.position(&graph, node)],
        None => query
            .origins
            .iter()
            .map(|o| unit_xyz(o.snapped_point.lat, o.snapped_point.lng))
            .collect(),
    };
    let destination_points = match patch.destination_endpoint {
        Some(node) => vec![patch.position(&graph, node)],
        None => query
            .destinations
            .iter()
            .map(|d| unit_xyz(d.snapped_point.lat, d.snapped_point.lng))
            .collect::<Vec<_>>(),
    };
    for arc in arcs.all() {
        if arc.from == NO_NODE {
            continue;
        }
        let end = match arc.kind {
            ArcKind::Finish { destination, .. } => {
                let d = &query.destinations[destination];
                unit_xyz(d.snapped_point.lat, d.snapped_point.lng)
            }
            ArcKind::End => continue,
            _ => patch.position(&graph, arc.to),
        };
        lower(
            &mut k,
            arc.cost,
            chord_m(patch.position(&graph, arc.from), end),
        );
    }
    for (seed, origin) in &seeds {
        let arc = arcs.arc(*seed);
        if arc.from != NO_NODE {
            continue;
        }
        let o = &query.origins[*origin as usize];
        lower(
            &mut k,
            arc.cost,
            chord_m(
                unit_xyz(o.snapped_point.lat, o.snapped_point.lng),
                patch.position(&graph, arc.to),
            ),
        );
    }
    let forced = dijkstra::forced();
    let k = if forced { 0.0 } else { k.max(0.0) };
    // CCH potentials on the column's metric; the patch's connectors and the
    // request's discount gates join them as gates of their own.
    let plan = (query.potentials && !forced)
        .then(|| potential_plan(graph, &costs.column, &FxHashMap::default(), false))
        .flatten();
    let scratch = std::mem::take(&mut state.cch_scratch).map(std::cell::RefCell::new);
    let (origin_terminals, destination_terminals) = (
        if patch.origin_endpoint.is_some() {
            Vec::new()
        } else {
            query.origin_edges.to_vec()
        },
        if patch.destination_endpoint.is_some() {
            Vec::new()
        } else {
            query.destination_edges.to_vec()
        },
    );
    let forward = Bounds::new(
        graph,
        &arcs,
        Side::Forward,
        costs.column,
        k,
        &destination_points,
        &destination_terminals,
        query.gates,
        plan.as_ref().map(|plan| (plan, &scratch[0])),
    );

    // Restriction penalties are charged forward only (the joins of the
    // bidirectional patch search assume bans).
    let bidirectional = query.bidirectional && !forced && query.restriction_penalty.is_none();
    let mut search = PatchRun {
        g: graph,
        automata: graph.complex_automata(query.profile_bit),
        arcs: &arcs,
        state,
        counters,
        query,
        labels: Vec::new(),
        patch_slots: vec![NO_LABEL; arcs.len()],
        b_labels: Vec::new(),
        b_patch_slots: Vec::new(),
        seq: 0,
        origin_approach: origin_approach.clone(),
        destination_approach: destination_approach.clone(),
        walk: query
            .pieces
            .filter(|_| bidirectional && !super::walks::off())
            .map(|kinds| walk::PatchWalk::new(graph, kinds, costs, &arcs)),
    };
    let outcome = if bidirectional {
        let backward = Bounds::new(
            graph,
            &arcs,
            Side::Backward,
            costs.column,
            k,
            &origin_points,
            &origin_terminals,
            query.gates,
            plan.as_ref().map(|plan| (plan, &scratch[1])),
        );
        search.bidirectional(&seeds, &forward, &backward, best)
    } else {
        search.unidirectional(&seeds, &forward, best)
    };
    let result = match outcome {
        Ok(Some(best)) => PatchOutcome::Found(search.reconstruct(&best)),
        Ok(None) => PatchOutcome::NoPath,
        Err(outcome) => outcome,
    };
    let state = search.state;
    drop(forward);
    state.cch_scratch = scratch.map(std::cell::RefCell::into_inner);
    result
}

struct PatchRun<'a, 'g, 'p, 'o, 'q> {
    g: SearchGraph<'g>,
    automata: &'g ComplexAutomata,
    arcs: &'a RunArcs<'p, 'o>,
    state: &'a mut SearchState,
    counters: &'a mut PatchCounters,
    query: &'a PatchQuery<'q>,
    labels: Vec<Label>,
    patch_slots: Vec<u32>,
    b_labels: Vec<BackwardLabel>,
    b_patch_slots: Vec<u32>,
    seq: u32,
    /// Approach costs of the origin and destination candidates, in seeds and
    /// finishes and taken out of a path's cost (`PatchQuery::approaches`).
    origin_approach: Vec<f64>,
    destination_approach: Vec<f64>,
    /// The pieces the bidirectional search walks through (see `walk`).
    walk: Option<walk::PatchWalk<'q>>,
}

impl PatchRun<'_, '_, '_, '_, '_> {
    #[inline]
    fn edge_count(&self) -> u32 {
        self.arcs.patch.edge_count
    }

    #[inline]
    fn info(&self, id: u32) -> ArcInfo {
        let g = &self.g;
        if id < self.edge_count() {
            let edge = id as usize;
            ArcInfo {
                tail: g.edge_source(edge),
                head: g.edge_target[edge],
                edge: id,
                enters: true,
                traversed: true,
                terminal: false,
                cost: f64::from((self.query.costs.edge_cost)(id)),
                duration_s: f64::from(self.query.costs.column.duration(id)),
                distance_m: f64::from(g.edge_length[edge]),
            }
        } else {
            let arc = self.arcs.arc(id - self.edge_count());
            ArcInfo {
                tail: arc.from,
                head: arc.to,
                edge: arc.edge().unwrap_or(NO_NODE),
                enters: arc.enters_edge(),
                traversed: arc.traversed(),
                terminal: arc.is_terminal(),
                cost: arc.cost,
                duration_s: arc.duration_s,
                distance_m: arc.distance_m,
            }
        }
    }

    /// The forward label of `arc` in restriction `state`, or `NO_LABEL`.
    fn slot(&self, arc: u32, state: u32) -> u32 {
        if state != ROOT {
            self.state.edge_extra.get(arc, state)
        } else if arc < self.edge_count() {
            self.state.edge_label.get(arc)
        } else {
            self.patch_slots[(arc - self.edge_count()) as usize]
        }
    }

    /// The backward label of `arc` in restriction `state`, or `NO_LABEL`.
    fn b_slot(&self, arc: u32, state: u32) -> u32 {
        if state != ROOT {
            self.state.b_edge_extra.get(arc, state)
        } else if arc < self.edge_count() {
            self.state.b_edge_label.get(arc)
        } else {
            self.b_patch_slots[(arc - self.edge_count()) as usize]
        }
    }

    /// Whether labels on an arc along `edge` may be in a restriction state
    /// other than the root.
    #[inline]
    fn has_states(&self, edge: u32) -> bool {
        edge != NO_NODE && self.automata.has_states(edge)
    }

    /// The forward labels of `arc` (along `edge`) in states other than the
    /// root.
    fn forward_extra(&self, arc: u32, edge: u32) -> Vec<u32> {
        if self.state.edge_extra.is_empty() || !self.has_states(edge) {
            return Vec::new();
        }
        self.state.edge_extra.labels(arc).collect()
    }

    /// The backward labels of `arc` (along `edge`) in states other than the
    /// root.
    fn backward_extra(&self, arc: u32, edge: u32) -> Vec<u32> {
        if self.state.b_edge_extra.is_empty() || !self.has_states(edge) {
            return Vec::new();
        }
        self.state.b_edge_extra.labels(arc).collect()
    }

    /// The forward restriction state after `label` continues onto `next`:
    /// a connector starts a new context, and consecutive traversed arcs
    /// along one edge count as that edge once.
    #[inline]
    fn forward_state(&self, label: &Label, next: &ArcInfo) -> u32 {
        if next.edge == NO_NODE {
            ROOT
        } else if label.traversed && label.last_edge == next.edge {
            label.state
        } else {
            self.automata.forward_next(&self.g, label.state, next.edge)
        }
    }

    /// The forward restriction state of a seed arc.
    #[inline]
    fn seed_state(&self, seed: &ArcInfo, traversed: bool) -> u32 {
        if seed.edge == NO_NODE || !traversed {
            ROOT
        } else {
            self.automata.forward_next(&self.g, ROOT, seed.edge)
        }
    }

    /// The backward restriction state of `previous` prepended to the chain
    /// at `label`, or `None` when that starts a whole forbidden sequence (the
    /// chain's context ends at its first connector, and consecutive arcs
    /// along one edge count as that edge once).
    #[inline]
    fn backward_state(&self, previous: &ArcInfo, label: &BackwardLabel) -> Option<u32> {
        if previous.edge == NO_NODE {
            return Some(ROOT);
        }
        if self.info(label.arc).edge == previous.edge {
            return Some(label.state);
        }
        self.automata
            .backward_prepend(&self.g, label.state, previous.edge)
    }

    /// Counts one visited edge; a cancelled request stops as if out of
    /// budget (and reports `Cancelled`).
    fn count_visit(&mut self) -> Result<(), PatchOutcome> {
        self.counters.visited_edge_count += 1;
        let visited = self.counters.visited_edge_count;
        match self.query.max_visited_edge_count {
            Some(max) if visited > max => Err(PatchOutcome::BudgetExceeded {
                max_visited_edge_count: max,
                best: None,
            }),
            max if self.query.cancellation.interrupted_at(visited) => {
                Err(PatchOutcome::BudgetExceeded {
                    max_visited_edge_count: max.unwrap_or(0),
                    best: None,
                })
            }
            _ => Ok(()),
        }
    }

    /// `outcome` (out of budget) with `best`, the best path so far, and its
    /// cost above `lower_bound`, which every path not yet found costs.
    fn exhausted(
        &self,
        outcome: PatchOutcome,
        best: Option<&Best>,
        lower_bound: f64,
    ) -> PatchOutcome {
        match outcome {
            PatchOutcome::BudgetExceeded {
                max_visited_edge_count,
                ..
            } => PatchOutcome::BudgetExceeded {
                max_visited_edge_count,
                best: best
                    .map(|best| (self.reconstruct(best), (best.cost() - lower_bound).max(0.0))),
            },
            outcome => outcome,
        }
    }

    /// The least key left in the forward queue, `at_least` included.
    fn forward_floor(&self, at_least: f64) -> f64 {
        self.state
            .heap
            .peek()
            .map_or(at_least, |entry| entry.priority.min(at_least))
    }

    /// Out-arcs of `node`: real out-edges, patch arcs, finishes.
    fn for_out_arcs(&self, node: u32, out: &mut Vec<u32>) {
        out.clear();
        if node < self.arcs.patch.node_count {
            out.extend(self.g.first_out[node as usize]..self.g.first_out[node as usize + 1]);
        }
        let base = self.edge_count();
        out.extend(self.arcs.out_arcs(node).map(|arc| base + arc));
    }

    /// In-arcs of `node`: real in-edges, patch arcs, seeds.
    fn for_in_arcs(&self, node: u32, out: &mut Vec<u32>) {
        out.clear();
        if node < self.arcs.patch.node_count {
            out.extend(self.g.in_edges(node as usize).map(|(edge, _)| edge));
        }
        let base = self.edge_count();
        out.extend(self.arcs.in_arcs(node).map(|arc| base + arc));
    }

    /// Turn cost and junction delay of continuing from context (`last_edge`,
    /// `traversed`) onto `next`, or `None` when a simple restriction forbids
    /// it.
    #[inline]
    fn transition(&self, last_edge: u32, traversed: bool, next: &ArcInfo) -> Option<(f64, f64)> {
        if !next.enters || last_edge == NO_NODE {
            return Some((0.0, 0.0));
        }
        if traversed
            && self.query.restriction_penalty.is_none()
            && self
                .g
                .simple_restricted(last_edge, next.edge, self.query.profile_bit)
        {
            return None;
        }
        // A SAR barrier may close the transition.
        let turn = self.g.turn_cost(last_edge, next.edge);
        if !turn.is_finite() {
            return None;
        }
        Some((
            turn,
            self.g
                .junction_delay(&self.query.costs.column, last_edge, next.edge),
        ))
    }

    /// The junction delay of [`Self::transition`] (0 where it is forbidden).
    fn transition_delay(&self, last_edge: u32, traversed: bool, next: &ArcInfo) -> f64 {
        self.transition(last_edge, traversed, next)
            .map_or(0.0, |(_, delay)| delay)
    }

    /// The restriction cost of entering `next` after forward label `label`
    /// beyond [`Self::transition`]'s simple-restriction ban: `None` when a
    /// complex restriction bans it, else the SAR penalty of every restriction
    /// it breaks (simple ones included: `transition` does not ban them under
    /// the relaxation).
    #[inline]
    fn restriction_penalty(&self, label: u32, next: &ArcInfo) -> Option<f64> {
        let Some(penalty) = self.query.restriction_penalty else {
            return (!self.forward_complex_restricted(label, next)).then_some(0.0);
        };
        let label = &self.labels[label as usize];
        if !next.enters || label.last_edge == NO_NODE {
            return Some(0.0);
        }
        let simple = label.traversed
            && self
                .g
                .simple_restricted(label.last_edge, next.edge, self.query.profile_bit);
        let count = u32::from(simple) + self.automata.forbidden_count(label.state, next.edge);
        Some(penalty * f64::from(count))
    }

    /// Whether entering `next` after forward label `label` breaks a complex
    /// restriction. The context is the real edges since the last connector,
    /// each edge once, which the label's restriction state stands for.
    #[inline]
    fn forward_complex_restricted(&self, label: u32, next: &ArcInfo) -> bool {
        let label = &self.labels[label as usize];
        next.enters && label.last_edge != NO_NODE && self.automata.forbids(label.state, next.edge)
    }

    #[allow(clippy::too_many_arguments)]
    fn push(
        &mut self,
        arc: u32,
        pred: u32,
        cost: f64,
        duration_s: f64,
        distance_m: f64,
        last_edge: u32,
        traversed: bool,
        origin: u32,
        state: u32,
        priority: f64,
    ) {
        self.push_label(
            Label {
                arc,
                pred,
                cost,
                duration_s,
                distance_m,
                last_edge,
                traversed,
                origin,
                state,
                via: NO_LABEL,
                back: false,
            },
            priority,
        );
    }

    fn push_label(&mut self, label: Label, priority: f64) {
        self.push_label_by(label, priority, false);
    }

    /// [`Self::push_label`], dominated by duration instead of cost when
    /// `by_duration` (fastest-time isochrones).
    #[inline]
    /// Queues `label` unless its slot holds one at least as good; whether it
    /// was queued.
    fn push_label_by(&mut self, label: Label, priority: f64, by_duration: bool) -> bool {
        let (arc, state) = (label.arc, label.state);
        // The lower bound proves the destination unreachable from here.
        if !priority.is_finite() {
            return false;
        }
        let slot = self.slot(arc, state);
        if slot != NO_LABEL {
            let existing = &self.labels[slot as usize];
            let dominated = if by_duration {
                existing.duration_s <= label.duration_s
            } else {
                existing.cost <= label.cost
            };
            if dominated {
                return false;
            }
        }
        let entry = label;
        let label = self.labels.len() as u32;
        self.labels.push(entry);
        if arc < self.edge_count() {
            self.state.set_slot(arc, state, label);
        } else if state != ROOT {
            self.state.edge_extra.set(arc, state, label);
        } else {
            let index = (arc - self.edge_count()) as usize;
            self.patch_slots[index] = label;
        }
        self.seq = self.seq.wrapping_add(1);
        self.state.heap.push(HeapEntry {
            priority,
            seq: self.seq,
            label,
        });
        true
    }

    #[allow(clippy::too_many_arguments)]
    fn push_backward(
        &mut self,
        arc: u32,
        next: u32,
        cost: f64,
        duration_s: f64,
        distance_m: f64,
        state: u32,
        priority: f64,
    ) {
        self.push_backward_label(
            BackwardLabel {
                arc,
                next,
                cost,
                duration_s,
                distance_m,
                state,
                via: NO_LABEL,
                back: false,
            },
            priority,
        );
    }

    fn push_backward_label(&mut self, label: BackwardLabel, priority: f64) {
        let (arc, cost, state) = (label.arc, label.cost, label.state);
        if !priority.is_finite() {
            return;
        }
        let slot = self.b_slot(arc, state);
        if slot != NO_LABEL && self.b_labels[slot as usize].cost <= cost {
            return;
        }
        let entry = label;
        let label = self.b_labels.len() as u32;
        self.b_labels.push(entry);
        if arc < self.edge_count() {
            self.state.set_b_slot(arc, state, label);
        } else if state != ROOT {
            self.state.b_edge_extra.set(arc, state, label);
        } else {
            let index = (arc - self.edge_count()) as usize;
            self.b_patch_slots[index] = label;
        }
        self.seq = self.seq.wrapping_add(1);
        self.state.b_heap.push(HeapEntry {
            priority,
            seq: self.seq,
            label,
        });
    }

    fn push_seeds(&mut self, seeds: &[(u32, u32)], key: &mut dyn FnMut(u32) -> f64) {
        for (arc, origin) in seeds {
            let id = self.edge_count() + arc;
            let info = self.info(id);
            if !info.cost.is_finite() {
                continue;
            }
            self.push(
                id,
                NO_LABEL,
                info.cost,
                info.duration_s,
                info.distance_m,
                info.edge,
                info.traversed,
                *origin,
                self.seed_state(&info, info.traversed),
                info.cost + key(info.head),
            );
        }
    }

    /// Forward A* to the first proven-cheapest finish.
    fn unidirectional(
        &mut self,
        seeds: &[(u32, u32)],
        bounds: &Bounds<'_>,
        mut best: Option<Best>,
    ) -> Result<Option<Best>, PatchOutcome> {
        let mut cache = FxHashMap::<u32, f64>::default();
        let mut heuristic = |node: u32| *cache.entry(node).or_insert_with(|| bounds.at(node));
        self.push_seeds(seeds, &mut heuristic);
        let mut out = Vec::new();
        while let Some(entry) = self.state.heap.pop() {
            if best
                .as_ref()
                // Ties are searched too, so the tie-break sees them all.
                .is_some_and(|best| entry.priority > best.cost() + tie_window(best.cost()))
            {
                break;
            }
            let label = self.labels[entry.label as usize];
            if self.slot(label.arc, label.state) != entry.label {
                continue;
            }
            self.counters.settled_edge_count += 1;
            let node = self.info(label.arc).head;
            self.for_out_arcs(node, &mut out);
            for next in out.iter().copied() {
                if let Err(outcome) = self.count_visit() {
                    let floor = self.forward_floor(entry.priority);
                    return Err(self.exhausted(outcome, best.as_ref(), floor));
                }
                let info = self.info(next);
                if !info.cost.is_finite() {
                    continue;
                }
                let Some((turn, delay)) = self.transition(label.last_edge, label.traversed, &info)
                else {
                    self.counters.restricted_transition_count += 1;
                    continue;
                };
                let Some(penalty) = self.restriction_penalty(entry.label, &info) else {
                    self.counters.restricted_transition_count += 1;
                    continue;
                };
                let total = label.cost + turn + info.cost + penalty;
                if info.terminal {
                    let candidate = Best::Finish {
                        label: entry.label,
                        arc: next,
                        cost: total,
                    };
                    if self.improves(&candidate, best.as_ref()) {
                        best = Some(candidate);
                    }
                    continue;
                }
                let (last_edge, traversed) = if info.edge == NO_NODE {
                    (NO_NODE, false)
                } else {
                    (info.edge, true)
                };
                let priority = total + heuristic(info.head);
                self.push(
                    next,
                    entry.label,
                    total,
                    label.duration_s + delay + info.duration_s,
                    label.distance_m + info.distance_m,
                    last_edge,
                    traversed,
                    label.origin,
                    self.forward_state(&label, &info),
                    priority,
                );
            }
        }
        Ok(best)
    }

    /// Bidirectional A* with average potentials, or the exact symmetric stop
    /// (see `package/search.rs`).
    fn bidirectional(
        &mut self,
        seeds: &[(u32, u32)],
        forward: &Bounds<'_>,
        backward: &Bounds<'_>,
        mut best: Option<Best>,
    ) -> Result<Option<Best>, PatchOutcome> {
        let g = self.g;
        self.state.ensure_route_len(g.edge_count, g.node_count);
        self.b_patch_slots = vec![NO_LABEL; self.arcs.len()];
        let node_count = self.arcs.patch.node_count;
        let virtual_bounds = (0..self.arcs.patch.positions.len() as u32)
            .map(|index| {
                let node = node_count + index;
                (forward.at(node), backward.at(node))
            })
            .collect::<Vec<_>>();
        let mut values = self.state.node_bounds.take();
        let mut touched = std::mem::take(&mut self.state.node_bounds_touched);
        let mut bounds = |node: u32| -> (f64, f64) {
            if node >= node_count {
                return virtual_bounds[(node - node_count) as usize];
            }
            let cached = values.get(node);
            if !cached.0.is_nan() {
                return cached;
            }
            let value = (forward.at(node), backward.at(node));
            values.set(node, value);
            touched.push(node);
            value
        };
        let result = self.bidirectional_body(seeds, &mut bounds, best.take());
        self.state.node_bounds = values;
        self.state.node_bounds_touched = touched;
        result
    }

    fn bidirectional_body(
        &mut self,
        seeds: &[(u32, u32)],
        bounds: &mut dyn FnMut(u32) -> (f64, f64),
        mut best: Option<Best>,
    ) -> Result<Option<Best>, PatchOutcome> {
        // Average potentials (Ikeda et al.): p_f = (h_f - h_b) / 2 and
        // p_b = -p_f, or each side's own bound with `exact` (see
        // `package/search.rs`); an infinite own-side bound proves the node
        // useless.
        let exact = crate::debug::exact_stop();
        let forward_potential = |bounds: &mut dyn FnMut(u32) -> (f64, f64), node: u32| {
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
        let backward_potential = |bounds: &mut dyn FnMut(u32) -> (f64, f64), node: u32| {
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
        self.push_seeds(seeds, &mut |node| forward_potential(bounds, node));
        // Backward seeds: every finish and end arc.
        let base = self.edge_count();
        let terminals = (0..self.arcs.len() as u32)
            .filter(|arc| {
                let arc = self.arcs.arc(*arc);
                arc.is_terminal() && arc.from != NO_NODE
            })
            .collect::<Vec<_>>();
        for arc in terminals {
            let id = base + arc;
            let info = self.info(id);
            if !info.cost.is_finite() {
                continue;
            }
            let priority = info.cost + backward_potential(bounds, info.tail);
            // A one-edge chain never starts a whole forbidden sequence.
            let state = if info.edge == NO_NODE {
                ROOT
            } else {
                self.automata
                    .backward_prepend(&self.g, ROOT, info.edge)
                    .unwrap_or(ROOT)
            };
            self.push_backward(
                id,
                NO_LABEL,
                info.cost,
                info.duration_s,
                info.distance_m,
                state,
                priority,
            );
        }

        let mut arcs = Vec::new();
        // An exhausted side has joined every path through its labels.
        while let (Some(forward_top), Some(backward_top)) = (
            self.state.heap.peek().map(|entry| entry.priority),
            self.state.b_heap.peek().map(|entry| entry.priority),
        ) {
            let bound = best.as_ref().map_or(f64::INFINITY, Best::cost);
            let stop = if exact {
                forward_top.max(backward_top)
            } else {
                forward_top + backward_top
            };
            // The stop's lower bound on paths not yet found, for a search
            // that runs out of budget during this step.
            let floor = |run: &Self| {
                let forward = run.forward_floor(forward_top);
                let backward = run
                    .state
                    .b_heap
                    .peek()
                    .map_or(backward_top, |entry| entry.priority.min(backward_top));
                if exact {
                    forward.max(backward)
                } else {
                    forward + backward
                }
            };
            // Ties are searched too, so the tie-break sees them all.
            if stop > bound + tie_window(bound) {
                break;
            }
            if self.state.heap.len() <= self.state.b_heap.len() {
                let entry = self.state.heap.pop().expect("peeked");
                let label = self.labels[entry.label as usize];
                if self.slot(label.arc, label.state) != entry.label {
                    continue;
                }
                self.counters.settled_edge_count += 1;
                let node = self.info(label.arc).head;
                self.for_out_arcs(node, &mut arcs);
                for next in arcs.iter().copied() {
                    if let Err(outcome) = self.count_visit() {
                        return Err(self.exhausted(outcome, best.as_ref(), floor(self)));
                    }
                    let info = self.info(next);
                    if !info.cost.is_finite() {
                        continue;
                    }
                    let Some((turn, delay)) =
                        self.transition(label.last_edge, label.traversed, &info)
                    else {
                        self.counters.restricted_transition_count += 1;
                        continue;
                    };
                    if self.forward_complex_restricted(entry.label, &info) {
                        self.counters.restricted_transition_count += 1;
                        continue;
                    }
                    let backward = self.b_slot(next, ROOT);
                    if backward != NO_LABEL {
                        self.try_meet(entry.label, backward, &mut best);
                    }
                    for backward in self.backward_extra(next, info.edge) {
                        self.try_meet(entry.label, backward, &mut best);
                    }
                    if info.terminal {
                        continue;
                    }
                    let cost = label.cost + turn + info.cost;
                    if self.walks_into(next, info.head) {
                        let at = (
                            cost,
                            label.duration_s + delay + info.duration_s,
                            label.distance_m + info.distance_m,
                        );
                        let state = self.forward_state(&label, &info);
                        let mut potential = |node| forward_potential(bounds, node);
                        match self.walk_forward(entry.label, next, at, state, &mut potential) {
                            walk::WalkEnd::Done => continue,
                            walk::WalkEnd::Exhausted(outcome) => {
                                return Err(self.exhausted(outcome, best.as_ref(), floor(self)));
                            }
                            walk::WalkEnd::PerEdge => {}
                        }
                    }
                    let potential = forward_potential(bounds, info.head);
                    let (last_edge, traversed) = if info.edge == NO_NODE {
                        (NO_NODE, false)
                    } else {
                        (info.edge, true)
                    };
                    self.push(
                        next,
                        entry.label,
                        cost,
                        label.duration_s + delay + info.duration_s,
                        label.distance_m + info.distance_m,
                        last_edge,
                        traversed,
                        label.origin,
                        self.forward_state(&label, &info),
                        cost + potential,
                    );
                }
            } else {
                let entry = self.state.b_heap.pop().expect("peeked");
                let label = self.b_labels[entry.label as usize];
                if self.b_slot(label.arc, label.state) != entry.label {
                    continue;
                }
                self.counters.settled_edge_count += 1;
                let after = self.info(label.arc);
                self.for_in_arcs(after.tail, &mut arcs);
                for previous in arcs.iter().copied() {
                    if let Err(outcome) = self.count_visit() {
                        return Err(self.exhausted(outcome, best.as_ref(), floor(self)));
                    }
                    let info = self.info(previous);
                    if !info.cost.is_finite() {
                        continue;
                    }
                    // Joins carry their own restriction checks.
                    let forward = self.slot(previous, ROOT);
                    if forward != NO_LABEL {
                        self.try_meet(forward, entry.label, &mut best);
                    }
                    for forward in self.forward_extra(previous, info.edge) {
                        self.try_meet(forward, entry.label, &mut best);
                    }
                    // Seeds start at the origin: nothing precedes them.
                    if info.tail == NO_NODE {
                        continue;
                    }
                    let Some((turn, delay)) = self.transition(info.edge, info.traversed, &after)
                    else {
                        self.counters.restricted_transition_count += 1;
                        continue;
                    };
                    let Some(state) = self.backward_state(&info, &label) else {
                        self.counters.restricted_transition_count += 1;
                        continue;
                    };
                    let cost = label.cost + turn + info.cost;
                    if self.walks_into(previous, info.tail) {
                        let at = (
                            cost,
                            label.duration_s + delay + info.duration_s,
                            label.distance_m + info.distance_m,
                        );
                        let mut potential = |node| backward_potential(bounds, node);
                        match self.walk_backward(entry.label, previous, at, state, &mut potential) {
                            walk::WalkEnd::Done => continue,
                            walk::WalkEnd::Exhausted(outcome) => {
                                return Err(self.exhausted(outcome, best.as_ref(), floor(self)));
                            }
                            walk::WalkEnd::PerEdge => {}
                        }
                    }
                    let potential = backward_potential(bounds, info.tail);
                    self.push_backward(
                        previous,
                        entry.label,
                        cost,
                        label.duration_s + delay + info.duration_s,
                        label.distance_m + info.distance_m,
                        state,
                        cost + potential,
                    );
                }
            }
        }
        Ok(best)
    }

    /// The joined path's arcs, origin to destination.
    fn meeting_arcs(&self, forward: u32, backward: u32) -> Vec<u32> {
        let mut arcs = Vec::new();
        let mut current = forward;
        while current != NO_LABEL {
            let label = self.labels[current as usize];
            self.push_label_arcs_reversed(&label, &mut arcs);
            current = label.pred;
        }
        arcs.reverse();
        let mut chain = backward;
        while chain != NO_LABEL {
            let label = self.b_labels[chain as usize];
            self.push_backward_label_arcs(&label, &mut arcs);
            chain = label.next;
        }
        arcs
    }

    /// Whether a full path breaks a simple or complex restriction anywhere.
    fn path_violates(&self, arcs: &[u32]) -> bool {
        let infos = arcs.iter().map(|arc| self.info(*arc)).collect::<Vec<_>>();
        for index in 1..infos.len() {
            let (previous, next) = (&infos[index - 1], &infos[index]);
            if !next.enters || previous.edge == NO_NODE {
                continue;
            }
            if self
                .transition(previous.edge, previous.traversed, next)
                .is_none()
            {
                return true;
            }
            if self.g.edge_flags[next.edge as usize] & EDGE_FLAG_COMPLEX_RESTRICTION_TO == 0 {
                continue;
            }
            for prefix in self
                .g
                .complex_prefixes_to(next.edge, self.query.profile_bit)
            {
                // Context: distinct consecutive traversed edges since the last
                // connector, walking back from `index - 1`.
                let mut expected = prefix.iter().rev().peekable();
                let mut previous_edge = NO_NODE;
                let mut position = index;
                let matched = loop {
                    let Some(want) = expected.peek().copied() else {
                        break true;
                    };
                    if position == 0 {
                        break false;
                    }
                    position -= 1;
                    let info = &infos[position];
                    if info.edge == NO_NODE {
                        break false;
                    }
                    if !info.traversed || info.edge == previous_edge {
                        continue;
                    }
                    previous_edge = info.edge;
                    if info.edge != *want {
                        break false;
                    }
                    expected.next();
                };
                if matched {
                    return true;
                }
            }
        }
        false
    }

    /// Joins forward label `forward` (ending where the backward label's arc
    /// starts) with backward label `backward`.
    fn try_meet(&self, forward: u32, backward: u32, best: &mut Option<Best>) {
        let f = self.labels[forward as usize];
        let b = self.b_labels[backward as usize];
        let next = self.info(b.arc);
        let Some((turn, _)) = self.transition(f.last_edge, f.traversed, &next) else {
            return;
        };
        let cost = f.cost + turn + b.cost;
        if best
            .as_ref()
            .is_some_and(|best| cost > best.cost() + tie_window(best.cost()))
        {
            return;
        }
        // Arcs may repeat where restrictions force a loop, as in the forward
        // search.
        let arcs = self.meeting_arcs(forward, backward);
        if self.path_violates(&arcs) {
            return;
        }
        let candidate = Best::Meet {
            forward,
            backward,
            cost,
        };
        if self.improves(&candidate, best.as_ref()) {
            *best = Some(candidate);
        }
    }

    /// Whether `candidate` beats `current`: cheaper beyond the tie window, or
    /// tied with it and first in the legacy tie-break order (fewer steps, then
    /// lexicographically smaller edge ids), the static search's rule.
    fn improves(&self, candidate: &Best, current: Option<&Best>) -> bool {
        let Some(current) = current else {
            return true;
        };
        let (cost, reference) = (candidate.cost(), current.cost());
        if cost < reference - tie_window(reference) {
            return true;
        }
        if cost > reference + tie_window(reference) {
            return false;
        }
        let key = |best: &Best| {
            let ids = self
                .arcs
                .patch
                .step_edge_ids(self.g, &self.reconstruct(best).steps);
            (ids.len(), ids)
        };
        key(candidate) < key(current)
    }

    fn step(&self, id: u32, steps: &mut Vec<PatchStep>) {
        if id < self.edge_count() {
            steps.push(PatchStep::Edge(id));
            return;
        }
        let arc = self.arcs.arc(id - self.edge_count());
        match arc.kind {
            ArcKind::Fragment {
                edge,
                start,
                end,
                split,
            }
            | ArcKind::Finish {
                edge,
                start,
                end,
                split,
                ..
            } => {
                // Legacy: a seed or finish that covers nothing adds no step.
                let empty = end - start <= FRACTION_EPSILON;
                let keep = match arc.kind {
                    ArcKind::Fragment { .. } => !(arc.from == NO_NODE && empty),
                    _ => !empty,
                };
                if keep {
                    steps.push(PatchStep::Fragment {
                        edge,
                        start,
                        end,
                        split,
                    });
                }
            }
            ArcKind::Connector(index) => steps.push(PatchStep::Connector(index)),
            ArcKind::End => {}
        }
    }

    fn destination_of(&self, id: u32) -> Option<usize> {
        if id < self.edge_count() {
            return None;
        }
        match self.arcs.arc(id - self.edge_count()).kind {
            ArcKind::Finish { destination, .. } => Some(destination),
            _ => None,
        }
    }

    fn reconstruct(&self, best: &Best) -> PatchPath {
        let (arcs, origin, cost, distance_m, duration_s) = match *best {
            Best::Direct { arc, origin, cost } => {
                let info = self.info(self.edge_count() + arc);
                (
                    vec![self.edge_count() + arc],
                    origin,
                    cost,
                    info.distance_m,
                    info.duration_s,
                )
            }
            Best::Finish { label, arc, cost } => {
                let l = self.labels[label as usize];
                let info = self.info(arc);
                let mut arcs = self.meeting_arcs(label, NO_LABEL);
                arcs.push(arc);
                let first = self.labels[self.first_label(label) as usize].origin;
                let delay = self.transition_delay(l.last_edge, l.traversed, &info);
                (
                    arcs,
                    first,
                    cost,
                    l.distance_m + info.distance_m,
                    l.duration_s + delay + info.duration_s,
                )
            }
            Best::Meet {
                forward,
                backward,
                cost,
            } => {
                let first = self.labels[self.first_label(forward) as usize].origin;
                let arcs = self.meeting_arcs(forward, backward);
                // Summed in path order, not as the two halves' labels lie, so
                // the bits do not depend on where they met.
                let (path_cost, distance_m, duration_s) = self.path_sums(&arcs);
                debug_assert!(
                    (path_cost - cost).abs() <= 1e-9 * cost.abs().max(1.0),
                    "path-order cost {path_cost} of a joined patch path, searched {cost}"
                );
                (arcs, first, path_cost, distance_m, duration_s)
            }
        };
        let mut steps = Vec::with_capacity(arcs.len());
        for arc in &arcs {
            self.step(*arc, &mut steps);
        }
        let origin_index = (origin != u32::MAX).then_some(origin as usize);
        let destination_index = arcs.last().and_then(|arc| self.destination_of(*arc));
        // Seeds and finishes carry their approach costs; the path's cost
        // does not.
        let approach = origin_index.map_or(0.0, |index| self.origin_approach[index])
            + destination_index.map_or(0.0, |index| self.destination_approach[index]);
        PatchPath {
            steps,
            origin_index,
            destination_index,
            cost: cost - approach,
            distance_m,
            duration_s,
        }
    }

    /// The cost (approach costs included), distance and duration of the
    /// path `arcs` (a seed first, a finish or end last), summed from the
    /// origin in the forward search's order: each arc's turn cost before its
    /// own cost, each junction delay before its duration. The bidirectional
    /// patch search charges no restriction penalties (they make it run
    /// forward only).
    fn path_sums(&self, arcs: &[u32]) -> (f64, f64, f64) {
        let seed = self.info(arcs[0]);
        let (mut cost, mut distance_m, mut duration_s) =
            (seed.cost, seed.distance_m, seed.duration_s);
        let (mut last_edge, mut traversed) = (seed.edge, seed.traversed);
        for arc in &arcs[1..] {
            let info = self.info(*arc);
            let (turn, delay) = self
                .transition(last_edge, traversed, &info)
                .unwrap_or((0.0, 0.0));
            cost = cost + turn + info.cost;
            duration_s = duration_s + delay + info.duration_s;
            distance_m += info.distance_m;
            (last_edge, traversed) = if info.edge == NO_NODE {
                (NO_NODE, false)
            } else {
                (info.edge, true)
            };
        }
        (cost, distance_m, duration_s)
    }

    fn first_label(&self, label: u32) -> u32 {
        let mut current = label;
        while self.labels[current as usize].pred != NO_LABEL {
            current = self.labels[current as usize].pred;
        }
        current
    }

    /// Whether the path ending at `label` runs along `edge` (whole or in
    /// part).
    fn path_contains_edge(&self, label: u32, edge: u32) -> bool {
        let mut current = label;
        while current != NO_LABEL {
            let entry = self.labels[current as usize];
            if entry.last_edge == edge {
                return true;
            }
            current = entry.pred;
        }
        false
    }
}

// ---------------------------------------------------------------------------
// Matrices and isochrones
// ---------------------------------------------------------------------------

/// The overlay a matrix or isochrone runs on: endpoint connectors belong to
/// a route's own origin and destination, so they are dropped.
pub(crate) fn analysis_overlay(overlay: &RouteDynamicOverlay) -> RouteDynamicOverlay {
    let mut overlay = overlay.clone();
    overlay
        .virtual_edges
        .retain(|edge| edge.source_endpoint.is_none() && edge.target_endpoint.is_none());
    overlay
}

impl PatchGraph<'_> {
    /// Whether the patch adds connectors (which may join weak components).
    pub(crate) fn has_connectors(&self) -> bool {
        self.arcs
            .iter()
            .any(|arc| matches!(arc.kind, ArcKind::Connector(_)))
    }

    /// Archive edge ids of a patched path: compact edges and fragments by
    /// their edge, connectors by their virtual edge id.
    pub(crate) fn step_edge_ids(&self, graph: SearchGraph<'_>, steps: &[PatchStep]) -> Vec<u64> {
        steps
            .iter()
            .map(|step| match step {
                PatchStep::Edge(edge) | PatchStep::Fragment { edge, .. } => {
                    graph.edge_id(*edge as usize)
                }
                PatchStep::Connector(index) => self.connector(*index).edge_id,
            })
            .collect()
    }

    /// Whether an arc ending at `end` on `edge` completes its usable range
    /// (the edge's end, or the end of a split range).
    fn completes_range(&self, edge: u32, end: f64) -> bool {
        match self.split.get(&edge) {
            Some(ranges) => ranges.iter().any(|(_, stop)| (stop - end).abs() <= 1e-9),
            None => end >= 1.0 - FRACTION_EPSILON,
        }
    }

    /// Paths from `origins` to `destinations` on the same usable stretch of
    /// one edge, as direct finish arcs: (origin, destination, arc, cost).
    fn direct_paths(
        &self,
        graph: SearchGraph<'_>,
        costs: &PatchCosts<'_>,
        query: &PatchQuery<'_>,
        (origin_approach, destination_approach): (&[f64], &[f64]),
        arcs: &mut RunArcs,
    ) -> Vec<(usize, usize, u32, f64)> {
        let mut direct = Vec::new();
        for (o, (origin, edge)) in query.origins.iter().zip(query.origin_edges).enumerate() {
            for (d, (destination, other)) in query
                .destinations
                .iter()
                .zip(query.destination_edges)
                .enumerate()
            {
                let (from, to) = (
                    origin.fraction_along_edge.clamp(0.0, 1.0),
                    destination.fraction_along_edge.clamp(0.0, 1.0),
                );
                if edge != other || from > to {
                    continue;
                }
                let same_range = self.split.get(edge).is_none_or(|ranges| {
                    ranges.iter().any(|(start, end)| {
                        from + FRACTION_EPSILON >= *start && to <= *end + FRACTION_EPSILON
                    })
                });
                let cost = costs.fragment_cost(*edge) * (to - from)
                    + origin_approach[o]
                    + destination_approach[d];
                if !same_range || !cost.is_finite() {
                    continue;
                }
                let arc = arcs.push(Arc {
                    from: NO_NODE,
                    to: NO_NODE,
                    kind: ArcKind::Finish {
                        destination: d,
                        edge: *edge,
                        start: from,
                        end: to,
                        split: self.split.contains_key(edge),
                    },
                    cost,
                    duration_s: f64::from(costs.column.duration(*edge)) * (to - from),
                    distance_m: f64::from(graph.edge_length[*edge as usize]) * (to - from),
                });
                direct.push((o, d, arc, cost));
            }
        }
        direct
    }
}

/// One matrix row over the patched graph.
pub(crate) struct PatchRow {
    /// One slot per target index.
    pub paths: Vec<Option<PatchPath>>,
    pub counters: PatchCounters,
    pub budget_exceeded: bool,
}

/// Shortest paths from one matrix source (`query.origins`) to every target,
/// sharing one Dijkstra expansion over the patched graph. `query.destinations`
/// holds every target candidate and `target_of` maps each to its target.
/// Paths follow the route search's semantics, so a cell equals the route
/// between its two points. A target is settled once the queue reaches its
/// best finish.
pub(crate) fn matrix_row(
    graph: SearchGraph<'_>,
    state: &mut SearchState,
    patch: &PatchGraph<'_>,
    query: &PatchQuery<'_>,
    target_of: &[usize],
    target_count: usize,
) -> PatchRow {
    let costs = &query.costs;
    let mut arcs = RunArcs::new(patch);
    let (origin_approach, destination_approach) = query.approaches();
    let seeds = patch.seeds(
        graph,
        costs,
        query.origins,
        query.origin_edges,
        &origin_approach,
        &mut arcs,
    );
    patch.finishes(
        graph,
        costs,
        query.destinations,
        query.destination_edges,
        &destination_approach,
        &mut arcs,
    );
    let mut best = (0..target_count)
        .map(|_| None)
        .collect::<Vec<Option<Best>>>();
    let mut open = vec![false; target_count];
    for arc in arcs.all() {
        if let ArcKind::Finish { destination, .. } = arc.kind {
            open[target_of[destination]] = true;
        }
    }
    for (origin, destination, arc, cost) in patch.direct_paths(
        graph,
        costs,
        query,
        (&origin_approach, &destination_approach),
        &mut arcs,
    ) {
        let slot = &mut best[target_of[destination]];
        if slot.as_ref().is_none_or(|best| cost < best.cost()) {
            *slot = Some(Best::Direct {
                arc,
                origin: origin as u32,
                cost,
            });
        }
    }

    let mut counters = PatchCounters::default();
    let mut search = PatchRun {
        g: graph,
        automata: graph.complex_automata(query.profile_bit),
        arcs: &arcs,
        state,
        counters: &mut counters,
        query,
        labels: Vec::new(),
        patch_slots: vec![NO_LABEL; arcs.len()],
        b_labels: Vec::new(),
        b_patch_slots: Vec::new(),
        seq: 0,
        origin_approach: origin_approach.clone(),
        destination_approach: destination_approach.clone(),
        walk: None,
    };
    let budget_exceeded = search.row(&seeds, target_of, &mut best, &mut open).is_err();
    let paths = best
        .iter()
        .map(|best| best.as_ref().map(|best| search.reconstruct(best)))
        .collect();
    PatchRow {
        paths,
        counters,
        budget_exceeded,
    }
}

impl PatchRun<'_, '_, '_, '_, '_> {
    fn row(
        &mut self,
        seeds: &[(u32, u32)],
        target_of: &[usize],
        best: &mut [Option<Best>],
        open: &mut [bool],
    ) -> Result<(), PatchOutcome> {
        let least_open = |best: &[Option<Best>], open: &[bool]| {
            best.iter()
                .zip(open.iter())
                .filter(|(_, open)| **open)
                .filter_map(|(best, _)| best.as_ref().map(Best::cost))
                .fold(f64::INFINITY, f64::min)
        };
        let mut open_count = open.iter().filter(|open| **open).count();
        if open_count == 0 {
            return Ok(());
        }
        self.push_seeds(seeds, &mut |_| 0.0);
        let mut settle_at = least_open(best, open);
        let mut out = Vec::new();
        while let Some(entry) = self.state.heap.pop() {
            let label = self.labels[entry.label as usize];
            if self.slot(label.arc, label.state) != entry.label {
                continue;
            }
            if entry.priority >= settle_at {
                for (target, slot) in best.iter().enumerate() {
                    // Ties are searched too, so the tie-break sees them all.
                    if open[target]
                        && slot
                            .as_ref()
                            .is_some_and(|b| b.cost() + tie_window(b.cost()) < entry.priority)
                    {
                        open[target] = false;
                        open_count -= 1;
                    }
                }
                if open_count == 0 {
                    break;
                }
                settle_at = least_open(best, open);
            }
            self.counters.settled_edge_count += 1;
            let node = self.info(label.arc).head;
            self.for_out_arcs(node, &mut out);
            for next in out.iter().copied() {
                self.count_visit()?;
                let info = self.info(next);
                if !info.cost.is_finite() {
                    continue;
                }
                let Some((turn, delay)) = self.transition(label.last_edge, label.traversed, &info)
                else {
                    continue;
                };
                let Some(penalty) = self.restriction_penalty(entry.label, &info) else {
                    continue;
                };
                let total = label.cost + turn + info.cost + penalty;
                if info.terminal {
                    if let Some(destination) = self.destination_of(next) {
                        let target = target_of[destination];
                        let candidate = Best::Finish {
                            label: entry.label,
                            arc: next,
                            cost: total,
                        };
                        if open[target] && self.improves(&candidate, best[target].as_ref()) {
                            best[target] = Some(candidate);
                            settle_at = settle_at.min(total);
                        }
                    }
                    continue;
                }
                let (last_edge, traversed) = if info.edge == NO_NODE {
                    (NO_NODE, false)
                } else {
                    (info.edge, true)
                };
                self.push(
                    next,
                    entry.label,
                    total,
                    label.duration_s + delay + info.duration_s,
                    label.distance_m + info.distance_m,
                    last_edge,
                    traversed,
                    label.origin,
                    self.forward_state(&label, &info),
                    total,
                );
            }
        }
        Ok(())
    }
}

/// Every patched arc reachable from the origin candidates within the largest
/// contour, with the isochrone's transition rules (see
/// `analysis::isochrone`): seeds always count as traversed, entering an edge
/// pays the turn from the previous one (a U-turn onto the opposing edge
/// included) and obeys both restriction kinds, and a connector starts a new
/// context. An edge is reported once an arc completes its usable range (the
/// whole edge, or a split edge's usable stretch) within a contour; crossings
/// are reported on every edge or fragment a contour cuts.
pub(crate) fn isochrone(
    graph: SearchGraph<'_>,
    state: &mut SearchState,
    patch: &PatchGraph<'_>,
    query: &PatchQuery<'_>,
    contours: &[super::analysis::AnalysisContour],
    seeding: super::analysis::IsochroneSeeding<'_>,
) -> super::analysis::IsochroneOutcome {
    use super::analysis::{BoundaryCrossing, IsochroneOutcome, ReachedEdge};
    // A label's origin (its seed's candidate's); the overlay's endpoint
    // connectors belong to routes and seed none here.
    let origin_of = |candidate: u32| {
        if candidate == u32::MAX {
            0
        } else {
            seeding.origin(candidate as usize)
        }
    };
    let mut arcs = RunArcs::new(patch);
    let (origin_approach, _) = query.approaches();
    let seeds = patch.seeds(
        graph,
        &query.costs,
        query.origins,
        query.origin_edges,
        &origin_approach,
        &mut arcs,
    );
    let contour_reach = super::analysis::ContourReach::of(contours);
    let within = |cost: f64, duration_s: f64, distance_m: f64| {
        contour_reach.within(cost, duration_s, distance_m)
    };
    type Spans = ((f64, f64), (f64, f64), (f64, f64));
    let crosses = |(costs, durations, distances): Spans| {
        contours.iter().any(|contour| {
            let (start, end) = contour.metric.of(costs, durations, distances);
            start < contour.limit && contour.limit < end && end - start > f64::EPSILON
        })
    };
    let cross = |crossings: &mut Vec<BoundaryCrossing>,
                 edge: u32,
                 (start_fraction, end_fraction): (f64, f64),
                 (costs, durations, distances): Spans,
                 origin: u16| {
        for (index, contour) in contours.iter().enumerate() {
            let (start, end) = contour.metric.of(costs, durations, distances);
            if !(start < contour.limit && contour.limit < end) || end - start <= f64::EPSILON {
                continue;
            }
            let ratio = ((contour.limit - start) / (end - start)).clamp(0.0, 1.0);
            crossings.push(BoundaryCrossing {
                contour: index,
                edge,
                start_fraction,
                end_fraction: start_fraction + (end_fraction - start_fraction) * ratio,
                start_metric: start,
                origin,
            });
        }
    };
    let mut outcome = IsochroneOutcome::default();
    let mut reached_slot = FxHashMap::<u32, usize>::default();
    let reach = |reached: &mut Vec<ReachedEdge>,
                 reached_slot: &mut FxHashMap<u32, usize>,
                 edge: u32,
                 metrics: (f64, f64, f64),
                 origin: u16| match reached_slot.get(&edge) {
        Some(slot) => reached[*slot].merge(metrics, origin),
        None => {
            reached_slot.insert(edge, reached.len());
            reached.push(ReachedEdge {
                edge,
                cost: metrics.0,
                duration_s: metrics.1,
                distance_m: metrics.2,
                origins: [origin; 3],
            });
        }
    };
    // The least key (cost, or duration when ordered by it) each edge was
    // completed at in the root restriction state, with the least cost at
    // that key, for the dominance check where labels are settled.
    let mut root_completed = FxHashMap::<u32, (f64, f64)>::default();
    let complete_in_root =
        |completed: &mut FxHashMap<u32, (f64, f64)>, edge: u32, key: f64, cost: f64| {
            let entry = completed
                .entry(edge)
                .or_insert((f64::INFINITY, f64::INFINITY));
            if (key, cost) < *entry {
                *entry = (key, cost);
            }
        };
    let span = |arc: &Arc| match arc.kind {
        ArcKind::Fragment { start, end, .. } | ArcKind::Finish { start, end, .. } => (start, end),
        _ => (0.0, 1.0),
    };

    let mut counters = PatchCounters::default();
    let mut search = PatchRun {
        g: graph,
        automata: graph.complex_automata(query.profile_bit),
        arcs: &arcs,
        state,
        counters: &mut counters,
        query,
        labels: Vec::new(),
        patch_slots: vec![NO_LABEL; arcs.len()],
        b_labels: Vec::new(),
        b_patch_slots: Vec::new(),
        seq: 0,
        origin_approach: origin_approach.clone(),
        destination_approach: Vec::new(),
        walk: None,
    };
    let base = search.edge_count();
    for (arc, origin) in &seeds {
        let id = base + arc;
        let info = search.info(id);
        if !info.cost.is_finite() {
            continue;
        }
        // A farther candidate starts at its approach cost (already in the
        // seed arc's cost). The approach is a penalty, not travel, so time
        // starts at zero (as in matrix rows and `analysis::isochrone`), or
        // at the origin's start offset.
        let approach = origin_approach
            .get(*origin as usize)
            .copied()
            .unwrap_or(0.0);
        // A costed access stretch (`crate::access`) is travel: time and
        // distance start after it.
        let access = query
            .origins
            .get(*origin as usize)
            .map(crate::access::AccessTotals::of)
            .unwrap_or_default();
        let offset_s = if *origin == u32::MAX {
            0.0
        } else {
            seeding.offset_s(*origin as usize) + access.duration_s
        };
        let duration_s = offset_s + info.duration_s;
        let start_m = access.distance_m;
        let distance_m = start_m + info.distance_m;
        let (start, end) = span(arcs.arc(*arc));
        cross(
            &mut outcome.crossings,
            info.edge,
            (start, end),
            (
                (approach, info.cost),
                (offset_s, duration_s),
                (start_m, distance_m),
            ),
            origin_of(*origin),
        );
        // A seed is labelled even beyond every contour (its successors are
        // then out of range too), and counts as traversing its edge.
        let seed_state = search.seed_state(&info, true);
        let key = seeding.key(info.cost, duration_s);
        // Only a queued label completes its edge: one dominated in its slot
        // is never settled.
        let pushed = search.push_label_by(
            Label {
                arc: id,
                pred: NO_LABEL,
                cost: info.cost,
                duration_s,
                distance_m,
                last_edge: info.edge,
                traversed: true,
                origin: *origin,
                state: seed_state,
                via: NO_LABEL,
                back: false,
            },
            key,
            seeding.by_duration,
        );
        if pushed && seed_state == ROOT && patch.completes_range(info.edge, end) {
            complete_in_root(&mut root_completed, info.edge, key, info.cost);
        }
        if within(info.cost, duration_s, distance_m) && patch.completes_range(info.edge, end) {
            reach(
                &mut outcome.reached,
                &mut reached_slot,
                info.edge,
                (info.cost, duration_s, distance_m),
                origin_of(*origin),
            );
        }
    }

    let mut out = Vec::new();
    'search: while let Some(entry) = search.state.heap.pop() {
        let label = search.labels[entry.label as usize];
        if search.slot(label.arc, label.state) != entry.label {
            continue;
        }
        search.counters.settled_edge_count += 1;
        // An edge with interior attachments is labelled both whole and in
        // fragments, in separate slots, so an arrival through its fragments
        // is not dominated by a cheaper one through the whole edge (or the
        // other way round). Where the edge was already completed more
        // cheaply, this arrival is the dominated one a plain graph never
        // settles: expanding it would report crossings on edges reached in
        // full long before. Ordered by duration, an arrival tied on time but
        // costlier is dominated too: the plain graph keeps the cheaper label
        // in the edge's one slot (two candidates of a costed access origin
        // can meet at a node at the same time, one of them through a turn).
        if label.state == ROOT
            && label.last_edge != NO_NODE
            && root_completed
                .get(&label.last_edge)
                .is_some_and(|(key, cost)| {
                    let own = seeding.key(label.cost, label.duration_s);
                    *key < own - tie_window(own)
                        || (seeding.by_duration
                            && *key <= own + tie_window(own)
                            && *cost < label.cost - tie_window(label.cost))
                })
        {
            continue;
        }
        let node = search.info(label.arc).head;
        search.for_out_arcs(node, &mut out);
        for next in out.iter().copied() {
            search.counters.visited_edge_count += 1;
            if query
                .max_visited_edge_count
                .is_some_and(|max| search.counters.visited_edge_count > max)
                || query
                    .cancellation
                    .interrupted_at(search.counters.visited_edge_count)
            {
                outcome.truncated = true;
                break 'search;
            }
            let info = search.info(next);
            if !info.cost.is_finite() {
                continue;
            }
            let (mut turn, mut delay) = (0.0, 0.0);
            if info.enters && label.last_edge != NO_NODE {
                // Isochrone seeds count as traversed.
                let penalty = match query.restriction_penalty {
                    None => {
                        if graph.simple_restricted(label.last_edge, info.edge, query.profile_bit)
                            || search.forward_complex_restricted(entry.label, &info)
                        {
                            continue;
                        }
                        0.0
                    }
                    Some(penalty) => {
                        let simple =
                            graph.simple_restricted(label.last_edge, info.edge, query.profile_bit);
                        let count = u32::from(simple)
                            + search.automata.forbidden_count(label.state, info.edge);
                        penalty * f64::from(count)
                    }
                };
                // A SAR restriction penalty is paid at the junction, like the
                // turn.
                turn = graph.turn_cost(label.last_edge, info.edge) + penalty;
                delay = graph.junction_delay(&query.costs.column, label.last_edge, info.edge);
                // A SAR barrier may close the transition.
                if !turn.is_finite() {
                    continue;
                }
            }
            let cost = label.cost + info.cost + turn;
            let duration_s = label.duration_s + delay + info.duration_s;
            let distance_m = label.distance_m + info.distance_m;
            let mut completes = false;
            if info.edge != NO_NODE {
                let (start, end) = if next < base {
                    (0.0, 1.0)
                } else {
                    span(arcs.arc(next - base))
                };
                let metrics = (
                    (label.cost + turn, cost),
                    (label.duration_s + delay, duration_s),
                    (label.distance_m, distance_m),
                );
                // Re-entering an arc already on this path never improves it;
                // it only matters where it would record a second crossing.
                if crosses(metrics) && search.path_contains_edge(entry.label, info.edge) {
                    continue;
                }
                let origin = origin_of(label.origin);
                cross(
                    &mut outcome.crossings,
                    info.edge,
                    (start, end),
                    metrics,
                    origin,
                );
                completes = next < base || patch.completes_range(info.edge, end);
                if within(cost, duration_s, distance_m) && completes {
                    reach(
                        &mut outcome.reached,
                        &mut reached_slot,
                        info.edge,
                        (cost, duration_s, distance_m),
                        origin,
                    );
                }
            }
            if !within(cost, duration_s, distance_m) {
                continue;
            }
            let last_edge = if info.edge == NO_NODE {
                NO_NODE
            } else {
                info.edge
            };
            let state = search.forward_state(&label, &info);
            let key = seeding.key(cost, duration_s);
            let pushed = search.push_label_by(
                Label {
                    arc: next,
                    pred: entry.label,
                    cost,
                    duration_s,
                    distance_m,
                    last_edge,
                    traversed: true,
                    origin: label.origin,
                    state,
                    via: NO_LABEL,
                    back: false,
                },
                key,
                seeding.by_duration,
            );
            if pushed && completes && state == ROOT {
                complete_in_root(&mut root_completed, info.edge, key, cost);
            }
        }
    }
    outcome.counters.visited_edge_count = counters.visited_edge_count;
    outcome.counters.settled_edge_count = counters.settled_edge_count;
    // An edge with interior attachments is reached both whole and in
    // fragments; a fragment crossing that the whole edge (or an earlier
    // fragment) already reports from an earlier start is the same boundary.
    let mut crossings = std::mem::take(&mut outcome.crossings);
    crossings.sort_by(|a, b| {
        (a.contour, a.edge)
            .cmp(&(b.contour, b.edge))
            .then(a.start_fraction.total_cmp(&b.start_fraction))
    });
    for (index, crossing) in crossings.iter().enumerate() {
        let duplicate = crossings[..index]
            .iter()
            .rev()
            .take_while(|other| other.contour == crossing.contour && other.edge == crossing.edge)
            .any(|other| {
                other.start_fraction < crossing.start_fraction
                    && (other.end_fraction - crossing.end_fraction).abs() <= 1e-6
            });
        if !duplicate {
            outcome.crossings.push(*crossing);
        }
    }
    outcome
}

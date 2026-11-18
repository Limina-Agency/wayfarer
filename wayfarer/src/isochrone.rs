use std::collections::BTreeSet;

use crate::correlator::Correlator;
use crate::costing::CostingPolicy;
use crate::package::destination::relax_split_costs;
use crate::package::profile_bit;
use crate::package::search::analysis::{
    AnalysisContour, AnalysisPoint, AnalysisQuery, overlay_split_costs, request_cost_overrides,
    request_duration_overrides, resolve_edge_id, resolve_location,
};
use crate::package::search::patch::{PatchCosts, PatchGraph, PatchQuery, analysis_overlay};
use crate::search::RoutingEngine;
use crate::types::{
    CorrelatedLocation, CorrelationPolicy, RouteError, RouteIsochroneBoundaryFragment,
    RouteIsochroneBucket, RouteIsochroneContour, RouteIsochroneMetric, RouteIsochroneRequest,
    RouteIsochroneResult,
};

impl RoutingEngine {
    pub fn isochrone(
        &self,
        request: RouteIsochroneRequest,
    ) -> Result<RouteIsochroneResult, RouteError> {
        if !self.route_graph_view().is_ready() {
            return Err(RouteError::GraphNotReady);
        }
        if !self
            .route_graph_view()
            .has_route_edges_for_profile(request.profile)
        {
            return Err(RouteError::ImpossibleMobilityProfile {
                profile: request.profile,
            });
        }

        self.validate_sar(&request.options, request.profile)?;
        request.options.cancellation.check()?;
        let policy = CorrelationPolicy::from(&request.options);
        let correlator = Correlator::new(self.route_graph_view());
        // The SAR `off_network` relaxation widens the search as routes do.
        let origin_search = if policy.off_network_max_distance_m.is_some() {
            correlator.search_expanding(
                request.origin,
                request.profile,
                &policy,
                &Default::default(),
            )
        } else {
            correlator.search(
                request.origin,
                request.profile,
                &policy,
                &Default::default(),
            )
        };
        let origin = origin_search
            .candidates
            .first()
            .cloned()
            .ok_or(RouteError::NoCandidate { endpoint: "origin" })?;

        let candidates = &origin_search.candidates;
        if !self
            .package()
            .confines_destination_only(&request.options, request.profile)
        {
            return self.isochrone_search(&request, candidates, &origin, true);
        }
        let strict = self.isochrone_search(&request, candidates, &origin, false)?;
        if !trapped(&strict) {
            return Ok(strict);
        }
        // An origin that cannot leave without crossing a destination-only
        // road gets the relaxed search, as routes do.
        let relaxed = self.isochrone_search(&request, candidates, &origin, true)?;
        Ok(pick_relaxed(strict, relaxed))
    }
}

/// A strict isochrone reaching fewer edges than this may be trapped behind a
/// destination-only road.
const TRAPPED_ORIGIN_REACH: usize = 64;

/// The edges an isochrone reaches within its widest contour.
fn reach(result: &RouteIsochroneResult) -> usize {
    result
        .buckets
        .iter()
        .map(|bucket| bucket.edge_ids.len())
        .max()
        .unwrap_or(0)
}

/// Whether a strict isochrone may be trapped behind a destination-only road
/// (see [`TRAPPED_ORIGIN_REACH`]), so that the relaxed one is searched too.
pub(crate) fn trapped(strict: &RouteIsochroneResult) -> bool {
    reach(strict) < TRAPPED_ORIGIN_REACH
}

/// The relaxed isochrone of a trapped origin when it reaches at least twice
/// as far, else the strict one.
pub(crate) fn pick_relaxed(
    strict: RouteIsochroneResult,
    relaxed: RouteIsochroneResult,
) -> RouteIsochroneResult {
    if reach(&relaxed) >= 2 * reach(&strict) {
        relaxed
    } else {
        strict
    }
}

impl RoutingEngine {
    /// The isochrone over the routing package, with a dynamic overlay as a
    /// patch (see `package::search::patch`; endpoint connectors belong to
    /// routes and are ignored). `GraphNotReady` without a package and
    /// `MissingEdgeMetadata` for a candidate edge the package does not hold.
    fn isochrone_search(
        &self,
        request: &RouteIsochroneRequest,
        candidates: &[CorrelatedLocation],
        origin: &CorrelatedLocation,
        allow_destination_only: bool,
    ) -> Result<RouteIsochroneResult, RouteError> {
        let access = self.route_graph_view();
        let graph = access.routing_package();
        let overlay = request
            .options
            .dynamic_overlay
            .as_ref()
            .filter(|overlay| {
                !overlay.split_edge_ranges.is_empty() || !overlay.virtual_edges.is_empty()
            })
            .map(analysis_overlay);
        // SAR R1 and R2 open edges in a derived column; R2's entry and R4's
        // barriers are transition costs the view adds to turn costs.
        let handle = graph.request_column_sar(
            request.profile,
            &request.options.costing,
            allow_destination_only,
            crate::sar::edge_relaxations(&request.options, request.profile),
        );
        let view = graph
            .view()
            .with_transitions(crate::transitions::Transitions::of(
                &request.options,
                request.profile,
            ));
        let column = handle.column(&view);
        let edges = candidates
            .iter()
            .map(|candidate| {
                resolve_location(view, candidate).ok_or(RouteError::MissingEdgeMetadata {
                    edge_id: candidate.edge_id,
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        let origins = AnalysisPoint::of_candidates(candidates, edges);
        let contours = request
            .contours
            .iter()
            .map(|contour| AnalysisContour {
                limit: contour.contour,
                on_duration: contour.metric == RouteIsochroneMetric::TimeSeconds,
            })
            .collect::<Vec<_>>();
        let costing = CostingPolicy::with_destination_only_policy(
            request.profile,
            &request.options,
            allow_destination_only,
        );
        let mut cost_overrides =
            request_cost_overrides(view, &request.options, &costing, request.profile);
        let mut durations = request_duration_overrides(
            view,
            column,
            &request.options,
            &costing,
            request.profile,
            &mut cost_overrides,
        );
        // A strict pass may reach out of the origin's destination-only pocket
        // (see `package::destination`).
        let unblocked = if allow_destination_only {
            Default::default()
        } else {
            graph.unblock_destination_pockets(
                &request.options,
                request.profile,
                origins.iter().map(|point| point.edge),
                &mut cost_overrides,
                &mut durations,
            )
        };
        let column = column.with_durations(&durations);
        let split_cost = match &overlay {
            Some(overlay) => {
                let mut split_cost = overlay_split_costs(
                    view,
                    overlay,
                    &costing,
                    request.profile,
                    &mut cost_overrides,
                );
                relax_split_costs(&unblocked, &mut split_cost);
                split_cost
            }
            None => Default::default(),
        };
        let query = AnalysisQuery {
            column,
            profile_bit: profile_bit(request.profile),
            max_visited_edge_count: request.max_expansion_edge_count,
            cancellation: &request.options.cancellation,
            cost_overrides,
            // A one-to-all expansion has no goal to direct it towards.
            gates: Vec::new(),
            restriction_penalty: crate::sar::restriction_penalty(&request.options),
        };
        let edge_cost = |edge: u32| query.edge_cost(view, edge);
        let costs = PatchCosts {
            column,
            edge_cost: &edge_cost,
            split_cost: &split_cost,
        };
        let resolve_id = |edge_id: u64| resolve_edge_id(view, edge_id);
        let patch = overlay
            .as_ref()
            .map(|overlay| PatchGraph::new(view, overlay, &costs, &resolve_id))
            .filter(|patch| !patch.is_empty());
        let started = std::time::Instant::now();
        let mut state = graph.take_state();
        let outcome = match &patch {
            Some(patch) => {
                let origin_edges = origins.iter().map(|point| point.edge).collect::<Vec<_>>();
                let patch_query = PatchQuery {
                    costs: PatchCosts {
                        column,
                        edge_cost: &edge_cost,
                        split_cost: &split_cost,
                    },
                    potentials: false,
                    gates: &[],
                    profile_bit: profile_bit(request.profile),
                    origins: candidates,
                    destinations: &[],
                    origin_edges: &origin_edges,
                    destination_edges: &[],
                    destination_targets: &[],
                    max_visited_edge_count: request.max_expansion_edge_count,
                    cancellation: &request.options.cancellation,
                    heuristic_cost_per_m: 0.0,
                    bidirectional: false,
                    restriction_penalty: crate::sar::restriction_penalty(&request.options),
                    pieces: None,
                };
                crate::package::search::patch::isochrone(
                    view,
                    &mut state,
                    patch,
                    &patch_query,
                    &contours,
                )
            }
            None => crate::package::search::analysis::isochrone(
                view, &mut state, &query, &origins, &contours,
            ),
        };
        graph.return_state(state);
        // A cancelled search stops as if out of budget.
        request.options.cancellation.check()?;
        let search_ms = started.elapsed().as_secs_f64() * 1e3;

        // Boundary points interpolate along the edge shapes stored in the
        // package (the road graph's geometry).
        let mut shape = crate::package::MeasuredPolyline::default();
        let mut boundary_fragments = Vec::with_capacity(outcome.crossings.len());
        for crossing in &outcome.crossings {
            shape.load_edge(&view, crossing.edge);
            let (Some(start), Some(end)) = (
                shape.point_at_fraction(crossing.start_fraction),
                shape.point_at_fraction(crossing.end_fraction),
            ) else {
                continue;
            };
            let contour = &request.contours[crossing.contour];
            boundary_fragments.push(IsochroneBoundaryCandidate {
                contour: contour.contour,
                metric: contour.metric,
                fragment: RouteIsochroneBoundaryFragment {
                    edge_id: view.edge_id(crossing.edge as usize),
                    start,
                    end,
                    start_fraction: crossing.start_fraction.clamp(0.0, 1.0),
                    end_fraction: crossing.end_fraction.clamp(0.0, 1.0),
                    start_metric: crossing.start_metric,
                    end_metric: contour.contour,
                },
            });
        }
        if crate::debug::trace() {
            eprintln!(
                "package isochrone: search {search_ms:.1} ms, geometry {:.1} ms ({} crossings)",
                started.elapsed().as_secs_f64() * 1e3 - search_ms,
                outcome.crossings.len(),
            );
        }

        let buckets = request
            .contours
            .iter()
            .map(|contour| RouteIsochroneBucket {
                contour: contour.contour,
                metric: contour.metric,
                edge_ids: outcome
                    .reached
                    .iter()
                    .filter(|edge| match contour.metric {
                        RouteIsochroneMetric::Cost => edge.cost <= contour.contour,
                        RouteIsochroneMetric::TimeSeconds => edge.duration_s <= contour.contour,
                    })
                    .map(|edge| view.edge_id(edge.edge as usize))
                    .collect::<BTreeSet<_>>()
                    .into_iter()
                    .collect(),
                boundary_fragments: boundary_fragments_for_contour(&boundary_fragments, contour),
            })
            .collect();
        Ok(RouteIsochroneResult {
            origin: origin.clone(),
            buckets,
            visited_edge_count: outcome.counters.visited_edge_count,
            settled_node_count: outcome.counters.settled_edge_count,
            truncated: outcome.truncated,
        })
    }
}

pub(crate) fn boundary_fragments_for_contour(
    fragments: &[IsochroneBoundaryCandidate],
    contour: &RouteIsochroneContour,
) -> Vec<RouteIsochroneBoundaryFragment> {
    let mut fragments = fragments
        .iter()
        .filter(|candidate| {
            candidate.metric == contour.metric
                && (candidate.contour - contour.contour).abs() < f64::EPSILON
        })
        .map(|candidate| candidate.fragment.clone())
        .collect::<Vec<_>>();
    fragments.sort_by(|left, right| {
        left.edge_id
            .cmp(&right.edge_id)
            .then_with(|| left.start_fraction.total_cmp(&right.start_fraction))
            .then_with(|| left.end_fraction.total_cmp(&right.end_fraction))
    });
    fragments.dedup_by(|left, right| {
        left.edge_id == right.edge_id
            && (left.start_fraction - right.start_fraction).abs() < f64::EPSILON
            && (left.end_fraction - right.end_fraction).abs() < f64::EPSILON
    });
    fragments
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct IsochroneBoundaryCandidate {
    pub(crate) contour: f64,
    pub(crate) metric: RouteIsochroneMetric,
    pub(crate) fragment: RouteIsochroneBoundaryFragment,
}

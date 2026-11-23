//! Routes a request over the engine's routing package: a stored or
//! derived cost column (non-default costing, strict destination-only pass,
//! profiles without a stored column), sparse per-edge overrides, and for
//! dynamic overlays a per-request patch of virtual nodes and arcs. This is
//! the only route search. Its failures are explicit errors:
//!
//! - no routing package (none could be built for an in-memory graph):
//!   `GraphNotReady`;
//! - a candidate edge the package does not hold (impossible for candidates
//!   correlated on the same package): `MissingEdgeMetadata`;
//! - the visited-edge budget, overlay or not: `SearchBudgetExceeded` when no
//!   route was found by then (else the best route so far, flagged in
//!   `RouteDiagnostics::search_budget_exceeded`);
//! - every origin candidate's edge blocked by the request (without an
//!   overlay patch, whose connectors could leave it): `BlockedCorridor`;
//! - `RouteOptions::cancellation`, which stops a search like the budget:
//!   `Cancelled`.

use super::*;
use crate::package::destination::relax_split_costs;
use crate::package::search::analysis::{
    OverrideBounds, overlay_split_costs, override_bounds, request_cost_overrides,
    request_duration_overrides, resolve_edge_id, resolve_location,
};
use crate::package::search::patch::{
    PatchCosts, PatchCounters, PatchGraph, PatchOutcome, PatchQuery, PatchStep, run as run_patch,
};
use crate::package::search::{
    SearchCounters, SearchOutcome, SearchQuery, approach_costs, run, tie_window,
};

impl Search<'_> {
    pub(super) fn search_package(
        &self,
        origin_candidates: &[CorrelatedLocation],
        destination_candidates: &[CorrelatedLocation],
        diagnostics: &RouteDiagnostics,
    ) -> Result<SearchRoute, RouteFailure> {
        let fail = |error| Err(RouteFailure::new(error, diagnostics.clone()));
        let graph = self.access.routing_package();
        let profile = self.request.profile;
        let options = &self.request.options;
        // An overlay without splits or connectors changes no search.
        let overlay = options.dynamic_overlay.as_ref().filter(|overlay| {
            !overlay.split_edge_ranges.is_empty() || !overlay.virtual_edges.is_empty()
        });
        // Non-default costing and the strict destination-only pass read a
        // column derived through the same costing function.
        // SAR R1 and R2 open edges in a derived column; R2's entry and R4's
        // barriers are transition costs the view adds to turn costs.
        let handle = graph.request_column_sar(
            profile,
            &options.costing,
            self.costing.allows_destination_only(),
            self.costing.sar_relaxations(),
        );
        let view = graph
            .view()
            .with_transitions(crate::transitions::Transitions::of(
                options,
                profile,
                graph.profile_bit(profile),
            ));
        let column = handle.column(&view);
        let resolve = |location: &CorrelatedLocation| {
            resolve_location(view, location).ok_or(RouteError::MissingEdgeMetadata {
                edge_id: location.edge_id,
            })
        };
        let origin_edges: Vec<u32> = match origin_candidates.iter().map(resolve).collect() {
            Ok(edges) => edges,
            Err(error) => return fail(error),
        };
        let destination_edges: Vec<u32> = match destination_candidates.iter().map(resolve).collect()
        {
            Ok(edges) => edges,
            Err(error) => return fail(error),
        };
        let mut cost_overrides = request_cost_overrides(view, options, &self.costing, profile);
        let mut durations = request_duration_overrides(
            view,
            column,
            options,
            &self.costing,
            profile,
            &mut cost_overrides,
        );
        // A strict pass may start or end in the destination-only pockets of
        // its endpoints (see `package::destination`).
        let unblocked = if self.costing.allows_destination_only() {
            Default::default()
        } else {
            graph.unblock_destination_pockets(
                options,
                profile,
                origin_edges.iter().chain(&destination_edges).copied(),
                &mut cost_overrides,
                &mut durations,
            )
        };
        let column = column.with_durations(&durations);

        let mut diagnostics = diagnostics.clone();
        if let Some(overlay) = overlay {
            // Split edges leave the static graph: their usable ranges become
            // patch fragments costed by the request's policy, which carries
            // the partial-block multiplier.
            let resolve_id = |edge_id: u64| resolve_edge_id(view, edge_id);
            let mut split_cost =
                overlay_split_costs(view, overlay, &self.costing, profile, &mut cost_overrides);
            relax_split_costs(&unblocked, &mut split_cost);
            let edge_cost = |edge: u32| {
                cost_overrides
                    .get(&edge)
                    .copied()
                    .unwrap_or_else(|| column.cost(edge))
            };
            let costs = PatchCosts {
                column,
                edge_cost: &edge_cost,
                split_cost: &split_cost,
            };
            let patch = PatchGraph::new(view, overlay, &costs, &resolve_id);
            if !patch.is_empty() {
                // Discounted edges and fragments become gates of the
                // patch's potentials.
                let bounds = override_bounds(view, column, &cost_overrides, &split_cost);
                // The patch search walks through the pieces it does not
                // touch where the static search's walks would be exact.
                let pieces = handle.piece_kinds(&view).filter(|_| {
                    crate::sar::restriction_penalty(options).is_none()
                        && self.costing.sar_relaxations().is_none()
                        && view.transitions.as_ref().is_none_or(|t| t.sar.is_none())
                        && !cost_overrides.iter().any(|(edge, cost)| {
                            cost.is_finite()
                                && !column.cost(*edge).is_finite()
                                && !handle.classified_usable(&view, *edge)
                        })
                });
                return self.search_package_patch(
                    graph,
                    &patch,
                    costs,
                    pieces,
                    &bounds,
                    origin_candidates,
                    destination_candidates,
                    &origin_edges,
                    &destination_edges,
                    diagnostics,
                );
            }
            // Only full blocks: the static search with infinite overrides.
        }
        // Every origin edge blocked by the request: nothing can be visited.
        let blocked = |candidates: &[CorrelatedLocation]| {
            candidates
                .iter()
                .filter(|candidate| self.costing.edge_blocked(candidate.edge_id))
                .count()
        };
        let origin_blocked = blocked(origin_candidates);
        if origin_blocked == origin_candidates.len() {
            return Err(RouteFailure::new(
                RouteError::BlockedCorridor {
                    blocked_edge_count: origin_blocked + blocked(destination_candidates),
                    visited_edge_count: 0,
                },
                diagnostics,
            ));
        }
        // An origin behind a destination on the same edge: the cheapest such
        // pair, approach costs included (see `approach_costs`). It competes
        // with the search, whose paths may start or end on nearer candidates.
        let origin_approach = approach_costs(origin_candidates);
        let destination_approach = approach_costs(destination_candidates);
        let mut direct = None::<(f64, SearchRoute)>;
        for (origin, origin_approach) in origin_candidates.iter().zip(&origin_approach) {
            for (destination, destination_approach) in
                destination_candidates.iter().zip(&destination_approach)
            {
                match self.direct_same_edge(origin, destination, &mut diagnostics) {
                    Ok(Some(route)) => {
                        let total = route.cost + origin_approach + destination_approach;
                        if direct.as_ref().is_none_or(|(best, _)| total < *best) {
                            direct = Some((total, route));
                        }
                    }
                    Ok(None) => {}
                    Err(error) => return Err(RouteFailure::new(error, diagnostics)),
                }
            }
        }

        // Overrides may lower an edge's cost below the column's: such edges
        // are gates of the potentials.
        let bounds = override_bounds(view, column, &cost_overrides, &Default::default());
        let restriction_penalty = crate::sar::restriction_penalty(options);
        let bidirectional =
            !crate::debug::unidirectional() && !crate::package::search::patch::dijkstra::forced();
        // The search walks through the column's marked nodes (a derived
        // column's: `columns::ColumnMarks`) where that is exact: the request
        // uses no edge the pieces were classified without (an override opens
        // only edges the classifying column can use, as a strict pass's
        // destination-only pockets), turns cost what the walks assume (no SAR transitions or
        // relaxations), and restrictions ban rather than charge.
        let pieces = handle.piece_kinds(&view).filter(|_| {
            bidirectional
                && restriction_penalty.is_none()
                && self.costing.sar_relaxations().is_none()
                && view.transitions.as_ref().is_none_or(|t| t.sar.is_none())
                && !cost_overrides.iter().any(|(edge, cost)| {
                    cost.is_finite()
                        && !column.cost(*edge).is_finite()
                        && !handle.classified_usable(&view, *edge)
                })
        });
        let query = SearchQuery {
            column,
            profile_bit: view.profile_bit(profile),
            heuristic_cost_per_m: bounds.heuristic_cost_per_m,
            origins: origin_candidates,
            destinations: destination_candidates,
            origin_edges: &origin_edges,
            destination_edges: &destination_edges,
            cost_overrides: &cost_overrides,
            max_visited_edge_count: options.max_visited_edge_count,
            cancellation: &options.cancellation,
            gates: &bounds.gates,
            // Tests force the unidirectional search with the patch's switch.
            bidirectional,
            restriction_penalty,
            pieces,
        };
        let mut counters = SearchCounters::default();
        let mut state = graph.take_state();
        let outcome = run(view, &mut state, &query, &mut counters);
        graph.return_state(state);

        diagnostics.visited_edge_count += counters.visited_edge_count;
        diagnostics.forward_visited_edge_count += counters.visited_edge_count;
        diagnostics.normal_edge_expanded_count += counters.visited_edge_count;
        diagnostics.settled_node_count += counters.settled_edge_count;
        diagnostics.restricted_transition_count += counters.restricted_transition_count;
        diagnostics.inaccessible_edge_count += counters.inaccessible_edge_count;
        // A cancelled search stops as if out of budget.
        if let Err(error) = options.cancellation.check() {
            return Err(RouteFailure::new(error, diagnostics));
        }
        if crate::debug::trace() {
            eprintln!(
                "compact: profile={profile:?} settled={} visited={} labels={} walked={} outcome={}",
                counters.settled_edge_count,
                counters.visited_edge_count,
                counters.label_count,
                counters.walked_edge_count,
                match &outcome {
                    SearchOutcome::Found(path) => format!(
                        "cost={:.1} edges={} h0={:.1}",
                        path.cost,
                        path.edges.len(),
                        counters.origin_heuristic
                    ),
                    other => format!("{other:?}"),
                }
            );
        }
        // Out of budget, the best route found so far stands (the direct pair
        // included), flagged with how much dearer than optimal it can be.
        let (outcome, lower_bound) = match outcome {
            SearchOutcome::BudgetExceeded {
                best: Some(path),
                lower_bound,
                ..
            } => (SearchOutcome::Found(path), Some(lower_bound)),
            SearchOutcome::BudgetExceeded { lower_bound, .. } if direct.is_some() => {
                (SearchOutcome::NoPath, Some(lower_bound))
            }
            outcome => (outcome, None),
        };
        let total = |path: &crate::package::search::SearchPath| {
            path.cost
                + origin_approach[path.origin_index]
                + destination_approach[path.destination_index]
        };
        // The direct pair wins ties (within the tie window: its record cost is
        // f64, the search's column costs f32) and stands in for a failed
        // search.
        let (outcome, chosen_total) = match (outcome, direct) {
            (SearchOutcome::Found(path), Some((direct_total, route)))
                if direct_total <= total(&path) + tie_window(total(&path)) =>
            {
                (Err(route), direct_total)
            }
            (SearchOutcome::Found(path), _) => {
                let path_total = total(&path);
                (Ok(SearchOutcome::Found(path)), path_total)
            }
            (_, Some((direct_total, route))) => (Err(route), direct_total),
            (outcome, None) => (Ok(outcome), f64::NAN),
        };
        if let Some(lower_bound) = lower_bound {
            diagnostics.search_budget_exceeded = true;
            diagnostics.search_optimality_gap_millicost =
                Some(((chosen_total - lower_bound).max(0.0) * 1e3).ceil() as u64);
        }
        let outcome = match outcome {
            Ok(outcome) => outcome,
            Err(mut route) => {
                route.diagnostics = diagnostics;
                return Ok(route);
            }
        };
        let result = match outcome {
            SearchOutcome::Found(path) => Ok(SearchRoute {
                origin: origin_candidates[path.origin_index].clone(),
                destination: destination_candidates[path.destination_index].clone(),
                edge_ids: path
                    .edges
                    .iter()
                    .map(|edge| view.edge_id(*edge as usize))
                    .collect(),
                edge_indexes: path.edges.iter().map(|edge| Some(*edge as usize)).collect(),
                steps: Vec::new(),
                distance_m: path.distance_m,
                duration_s: path.duration_s,
                cost: path.cost,
                diagnostics,
            }),
            SearchOutcome::NoPath => Err(RouteFailure::new(
                RouteError::NoPath {
                    visited_edge_count: diagnostics.visited_edge_count,
                },
                diagnostics,
            )),
            SearchOutcome::BudgetExceeded {
                max_visited_edge_count,
                ..
            } => Err(RouteFailure::new(
                RouteError::SearchBudgetExceeded {
                    visited_edge_count: diagnostics.visited_edge_count,
                    max_visited_edge_count,
                },
                diagnostics,
            )),
        };
        if crate::debug::verify()
            && let Ok(route) = &result
            && let Err(problem) = self.verify_route(route)
        {
            eprintln!(
                "package route verification FAILED ({profile:?}, {} edges): {problem}",
                route.edge_ids.len()
            );
        }
        result
    }

    /// Routes over the routing package plus an overlay patch (see
    /// `package::search::patch`) and returns the path as overlay steps.
    #[allow(clippy::too_many_arguments)]
    fn search_package_patch(
        &self,
        graph: &crate::RoutingPackage,
        patch: &PatchGraph<'_>,
        costs: PatchCosts<'_>,
        pieces: Option<wayfarer_package::format::ColumnPieceKinds<'_>>,
        bounds: &OverrideBounds,
        origin_candidates: &[CorrelatedLocation],
        destination_candidates: &[CorrelatedLocation],
        origin_edges: &[u32],
        destination_edges: &[u32],
        mut diagnostics: RouteDiagnostics,
    ) -> Result<SearchRoute, RouteFailure> {
        let view = graph
            .view()
            .with_transitions(crate::transitions::Transitions::of(
                &self.request.options,
                self.request.profile,
                graph.profile_bit(self.request.profile),
            ));
        let query = PatchQuery {
            heuristic_cost_per_m: bounds.heuristic_cost_per_m,
            // The scaled fallback does not cover the patch's bounds at
            // virtual points, which go along edges at column cost.
            potentials: !bounds.fell_back(),
            gates: &bounds.gates,
            costs,
            profile_bit: view.profile_bit(self.request.profile),
            origins: origin_candidates,
            destinations: destination_candidates,
            origin_edges,
            destination_edges,
            origin_targets: &[],
            destination_targets: &[],
            max_visited_edge_count: self.request.options.max_visited_edge_count,
            cancellation: &self.request.options.cancellation,
            bidirectional: !crate::debug::unidirectional(),
            restriction_penalty: crate::sar::restriction_penalty(&self.request.options),
            pieces,
        };
        let mut counters = PatchCounters::default();
        let mut state = graph.take_state();
        let outcome = run_patch(view, &mut state, patch, &query, &mut counters);
        graph.return_state(state);
        diagnostics.visited_edge_count += counters.visited_edge_count;
        diagnostics.forward_visited_edge_count += counters.visited_edge_count;
        diagnostics.normal_edge_expanded_count += counters.visited_edge_count;
        diagnostics.settled_node_count += counters.settled_edge_count;
        diagnostics.restricted_transition_count += counters.restricted_transition_count;
        if let Err(error) = self.request.options.cancellation.check() {
            return Err(RouteFailure::new(error, diagnostics));
        }
        let path = match outcome {
            PatchOutcome::Found(path) => path,
            // Out of budget, the best route found so far stands, flagged.
            PatchOutcome::BudgetExceeded {
                best: Some((path, gap)),
                ..
            } => {
                diagnostics.search_budget_exceeded = true;
                diagnostics.search_optimality_gap_millicost = Some((gap * 1e3).ceil() as u64);
                path
            }
            PatchOutcome::NoPath => {
                return Err(RouteFailure::new(
                    RouteError::NoPath {
                        visited_edge_count: diagnostics.visited_edge_count,
                    },
                    diagnostics,
                ));
            }
            PatchOutcome::BudgetExceeded {
                max_visited_edge_count,
                ..
            } => {
                return Err(RouteFailure::new(
                    RouteError::SearchBudgetExceeded {
                        visited_edge_count: diagnostics.visited_edge_count,
                        max_visited_edge_count,
                    },
                    diagnostics,
                ));
            }
        };
        // Endpoint connectors start or end at the request's own points.
        let endpoint = |kind, fallback: Option<&CorrelatedLocation>| {
            let mut location = fallback?.clone();
            location.snapped_point = match kind {
                RouteDynamicEndpointKind::Origin => self.request.origin,
                RouteDynamicEndpointKind::Destination => self.request.destination,
            };
            location.distance_m = 0.0;
            location.confidence = 1.0;
            Some(location)
        };
        let origin = match path.origin_index {
            Some(index) => origin_candidates[index].clone(),
            None => endpoint(RouteDynamicEndpointKind::Origin, origin_candidates.first())
                .ok_or_else(|| {
                    RouteFailure::new(
                        RouteError::NoPath {
                            visited_edge_count: diagnostics.visited_edge_count,
                        },
                        diagnostics.clone(),
                    )
                })?,
        };
        let destination = match path.destination_index {
            Some(index) => destination_candidates[index].clone(),
            None => endpoint(
                RouteDynamicEndpointKind::Destination,
                destination_candidates.first(),
            )
            .ok_or_else(|| {
                RouteFailure::new(
                    RouteError::NoPath {
                        visited_edge_count: diagnostics.visited_edge_count,
                    },
                    diagnostics.clone(),
                )
            })?,
        };
        let graph_step =
            |edge: u32, start: f64, end: f64, split: bool| RouteSearchStep::GraphEdge {
                edge_id: view.edge_id(edge as usize),
                start_fraction: start,
                end_fraction: end,
                quality_flags: if split {
                    vec![RouteShapeQualityFlag::SsvgPartialEdgeSplit]
                } else {
                    Vec::new()
                },
            };
        let mut steps = Vec::with_capacity(path.steps.len());
        let mut edge_indexes = Vec::with_capacity(path.steps.len());
        for step in &path.steps {
            match *step {
                PatchStep::Edge(edge) => {
                    steps.push(graph_step(edge, 0.0, 1.0, false));
                    edge_indexes.push(Some(edge as usize));
                }
                PatchStep::Fragment {
                    edge,
                    start,
                    end,
                    split,
                } => {
                    steps.push(graph_step(edge, start, end, split));
                    edge_indexes.push(Some(edge as usize));
                }
                PatchStep::Connector(index) => {
                    let edge = patch.connector(index).clone();
                    diagnostics.overlay_multiplier_count += 1;
                    steps.push(RouteSearchStep::VirtualConnector {
                        edge_id: edge.edge_id,
                        connector_id: edge.connector_id,
                        connector_type: edge.connector_type,
                        geometry: edge.geometry,
                        distance_m: edge.distance_m,
                        cost: edge.cost,
                        source_chunk_ids: edge.source_chunk_ids,
                        source_endpoint: edge.source_endpoint,
                        source_point: edge.source_point,
                        source_edge_id: edge.source_edge_id,
                        source_fraction: edge.source_fraction,
                        target_endpoint: edge.target_endpoint,
                        target_point: edge.target_point,
                        target_edge_id: edge.target_edge_id,
                        target_fraction: edge.target_fraction,
                    });
                    edge_indexes.push(None);
                }
            }
        }
        let edge_ids = steps
            .iter()
            .map(RouteSearchStep::edge_id)
            .collect::<Vec<_>>();
        diagnostics.final_path_edge_count = edge_ids.len();
        if crate::debug::trace() {
            eprintln!(
                "package overlay: settled={} visited={} cost={:.1} steps={}",
                counters.settled_edge_count,
                counters.visited_edge_count,
                path.cost,
                steps.len()
            );
        }
        Ok(SearchRoute {
            origin,
            destination,
            edge_ids,
            edge_indexes,
            steps,
            distance_m: path.distance_m,
            duration_s: path.duration_s,
            cost: path.cost,
            diagnostics,
        })
    }

    /// Re-derives a package route's legality and cost (`WAYFARER_VERIFY`)
    /// independently of the precomputed columns: each edge's record rebuilt
    /// from the view's raw attributes (for a package, its full shared info and
    /// edge fields) and costed by `CostingPolicy`, turn costs from the
    /// records, and both restriction kinds from the view's restriction lists.
    fn verify_route(&self, route: &SearchRoute) -> Result<(), String> {
        let profile = self.request.profile;
        let records = route
            .edge_ids
            .iter()
            .map(|id| {
                self.search_edge_by_id(*id)
                    .ok_or_else(|| format!("edge {id} has no record"))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let origin_record = self
            .search_edge_by_id(route.origin.edge_id)
            .ok_or("origin edge has no record")?;
        let cost_of = |record: &GraphViewEdge| match self.path_costing.edge_cost(record) {
            EdgeCostDecision::Allowed(costed) => Ok(costed.cost),
            _ => Err(format!("edge {} is not accessible", record.edge.edge_id)),
        };
        for pair in records.windows(2) {
            if pair[0].target_index != pair[1].source_index {
                return Err(format!(
                    "edges {} -> {} are not connected",
                    pair[0].edge.edge_id, pair[1].edge.edge_id
                ));
            }
        }
        // Under the SAR `turn_restrictions` relaxation broken restrictions
        // are charged, not banned.
        let penalty = crate::sar::restriction_penalty(&self.request.options);
        let mut restriction_cost = 0.0;
        if let Some(penalty) = penalty {
            let graph = self.access.routing_package().view();
            let edges = route
                .edge_ids
                .iter()
                .map(|id| graph.edge_for_id(*id))
                .collect::<Option<Vec<_>>>()
                .ok_or("route edge not in the package")?;
            restriction_cost = penalty
                * graph
                    .restriction_violations(&edges, graph.profile_bit(profile))
                    .len() as f64;
        }
        for index in (1..records.len()).filter(|_| penalty.is_none()) {
            let (from, to) = (&records[index - 1], &records[index]);
            if self.access.transition_restricted(
                from.edge.edge_id,
                to.edge.edge_id,
                to.source.id,
                profile,
            ) {
                return Err(format!(
                    "turn {} -> {} violates a simple restriction",
                    from.edge.edge_id, to.edge.edge_id
                ));
            }
            if self.access.complex_transition_restricted_for_path(
                &route.edge_ids[..index],
                to.edge.edge_id,
                profile,
            ) {
                return Err(format!(
                    "edge {} violates a complex restriction",
                    to.edge.edge_id
                ));
            }
        }
        let last = records.len() - 1;
        let traversed_origin = route.edge_ids[0] == route.origin.edge_id && last > 0;
        // Transition costs (barriers, SAR R2 entry), from the package's
        // sections, where turn costs are paid. Package records carry their
        // compact edge in `raw_edge_index`.
        let package_view = self.access.routing_package().view();
        let transitions = crate::transitions::Transitions::of(
            &self.request.options,
            profile,
            package_view.profile_bit(profile),
        );
        let edge_of = |record: &GraphViewEdge| -> Result<u32, String> {
            let edge = u32::try_from(record.raw_edge_index)
                .ok()
                .filter(|edge| {
                    (*edge as usize) < package_view.edge_count
                        && package_view.edge_id(*edge as usize) == record.edge.edge_id
                })
                .ok_or_else(|| format!("edge {} is not in the package", record.edge.edge_id))?;
            Ok(edge)
        };
        let transition = |from: &GraphViewEdge, to: &GraphViewEdge| -> Result<f64, String> {
            let cost = transitions.cost(&package_view, edge_of(from)?, edge_of(to)?);
            if cost.is_finite() {
                Ok(cost)
            } else {
                Err(format!(
                    "turn {} -> {} passes a closed barrier",
                    from.edge.edge_id, to.edge.edge_id
                ))
            }
        };
        let mut cost = restriction_cost;
        let mut previous = &origin_record;
        let mut start = 0;
        if traversed_origin {
            cost +=
                cost_of(&origin_record)? * (1.0 - route.origin.fraction_along_edge).clamp(0.0, 1.0);
            start = 1;
        }
        for record in &records[start..last] {
            cost += self
                .path_costing
                .transition_cost_for_records(previous, record)
                + transition(previous, record)?
                + cost_of(record)?;
            previous = record;
        }
        let destination = &records[last];
        if start < last || traversed_origin {
            cost += self
                .path_costing
                .transition_cost_for_records(previous, destination)
                + transition(previous, destination)?;
        }
        cost += cost_of(destination)? * route.destination.fraction_along_edge.clamp(0.0, 1.0);
        if (cost - route.cost).abs() > 1e-3 * route.cost.max(1.0) {
            return Err(format!(
                "record re-costing gives {cost:.3}, search reported {:.3}",
                route.cost
            ));
        }
        Ok(())
    }
}

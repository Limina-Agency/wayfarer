use std::collections::BTreeSet;
use std::sync::Arc;
use std::time::{Duration, Instant};

use rayon::prelude::*;

use crate::correlator::{CandidateSearch, Correlator};
use crate::costing::{CostedEdge, CostingPolicy, EdgeCostDecision};
use crate::graph_view::{GraphViewEdge, PackageGraphView};
use crate::package::RoutingPackage;
use crate::package::search::analysis::request_edge_cost;
use crate::package::search::approach_cost;
use crate::route_leg::build_route_result;
use crate::types::{
    CorrelatedLocation, CorrelationPolicy, GeoPoint, LegRequest, LegResult, RouteAlternative,
    RouteAlternativeLabel, RouteAlternativePolicy, RouteDiagnostics, RouteDynamicEndpointKind,
    RouteDynamicVirtualEdgeKind, RouteError, RouteFailure, RouteOptions, RouteShapeQualityFlag,
};

mod dispatch;

/// Plateaus shorter than this share of the shortest distance are not
/// locally optimal enough to offer as via alternatives.
const VIA_MIN_PLATEAU: f64 = 0.15;

/// A plain package route's via candidates (see
/// `RoutingEngine::via_alternatives`): the plateaus, then the trees the
/// single-edge candidates are listed from once the plateaus run out.
struct ViaCandidates {
    listed: Vec<crate::package::cch::ViaCandidate>,
    trees: Option<crate::package::cch::ViaTrees>,
}

/// What a plain package route's via candidates are listed on: the package
/// graph, its CCH topology, the request column's metric and costs, the
/// primary's first and last nodes, and its edges (which no candidate uses).
struct ViaPlan<'g> {
    graph: crate::package::SearchGraph<'g>,
    topology: &'g crate::package::cch::CchTopology,
    metric: &'g crate::package::cch::CchMetric,
    costs: &'g [f32],
    source: u32,
    target: u32,
    excluded: rustc_hash::FxHashSet<u32>,
}

/// A via alternative accepted before its legs are built: the found route,
/// its edges, the primary's diagnostics before it was accepted, and how it
/// was judged.
struct AcceptedVia {
    route: SearchRoute,
    edges: BTreeSet<u64>,
    diagnostics: Box<RouteDiagnostics>,
    label: RouteAlternativeLabel,
    shared_edge_ratio: f64,
    warning_edge_count: usize,
    quality_score: f64,
    quality_reasons: Vec<String>,
}

/// A found route before its legs are built: the correlated endpoints, the
/// traversed edges (with overlay steps when a dynamic overlay shaped it),
/// and its totals.
#[derive(Debug, Clone)]
pub(crate) struct SearchRoute {
    pub origin: CorrelatedLocation,
    pub destination: CorrelatedLocation,
    pub edge_ids: Vec<u64>,
    /// Each edge's raw index in the routing view (`None` for connectors).
    pub edge_indexes: Vec<Option<usize>>,
    pub steps: Vec<RouteSearchStep>,
    pub distance_m: f64,
    pub duration_s: f64,
    pub cost: f64,
    pub diagnostics: RouteDiagnostics,
}

/// One step of an overlay route: part of a graph edge, or a virtual
/// connector.
#[derive(Debug, Clone)]
pub(crate) enum RouteSearchStep {
    GraphEdge {
        edge_id: u64,
        start_fraction: f64,
        end_fraction: f64,
        quality_flags: Vec<RouteShapeQualityFlag>,
    },
    VirtualConnector {
        edge_id: u64,
        connector_id: String,
        connector_type: RouteDynamicVirtualEdgeKind,
        geometry: Vec<GeoPoint>,
        distance_m: f64,
        cost: f64,
        source_chunk_ids: Vec<String>,
        source_endpoint: Option<RouteDynamicEndpointKind>,
        source_point: Option<GeoPoint>,
        source_edge_id: Option<u64>,
        source_fraction: Option<f64>,
        target_endpoint: Option<RouteDynamicEndpointKind>,
        target_point: Option<GeoPoint>,
        target_edge_id: Option<u64>,
        target_fraction: Option<f64>,
    },
}

impl RouteSearchStep {
    pub(crate) fn edge_id(&self) -> u64 {
        match self {
            Self::GraphEdge { edge_id, .. } | Self::VirtualConnector { edge_id, .. } => *edge_id,
        }
    }
}

/// Routes, matrices and isochrones over one road network. Every search runs
/// on a routing package (`crate::package`): one opened from a file, or one
/// built in memory from a road graph.
pub struct RoutingEngine {
    view: PackageGraphView,
}

impl RoutingEngine {
    /// An engine over the package at `path`, opened with the default memory
    /// profile ([`RoutingPackage::open`]).
    pub fn open(path: &std::path::Path) -> Result<Self, crate::RoutingPackageError> {
        Ok(Self::from_package(Arc::new(RoutingPackage::open(path)?)))
    }

    /// An engine over an opened routing package.
    pub fn from_package(package: Arc<RoutingPackage>) -> Self {
        Self {
            view: PackageGraphView::new(package),
        }
    }

    /// The routing package every search of this engine runs on.
    pub fn package(&self) -> &Arc<RoutingPackage> {
        self.view.package()
    }

    /// The engine's view of its package (internal: see
    /// [`crate::internals`]).
    #[doc(hidden)]
    pub fn route_graph_view(&self) -> &PackageGraphView {
        &self.view
    }

    fn route_view(&self) -> &PackageGraphView {
        &self.view
    }

    /// A route through the request's locations, with its overlays applied:
    /// one search per routed leg, stitched into one result (see
    /// [`RouteRequest`](crate::RouteRequest)). A failed leg's failure carries
    /// its index.
    pub fn route(&self, request: crate::RouteRequest) -> Result<crate::RouteResult, RouteFailure> {
        crate::trip::route(self, request)
    }

    /// One leg between two points: the search [`RoutingEngine::route`] runs
    /// for each routed leg of a request. A failure carries the search's
    /// diagnostics.
    pub fn route_leg(&self, request: LegRequest) -> Result<LegResult, RouteFailure> {
        let total_started = Instant::now();
        let cancellation = &request.options.cancellation;
        cancellation
            .check()
            .map_err(|error| RouteFailure::new(error, RouteDiagnostics::default()))?;
        let correlated = self.correlate(&request)?;
        let route = self.search_correlated(&request, &correlated, total_started)?;
        // The primary's compact edges (alternatives avoid them), for a plain
        // package route.
        let primary_compact = route
            .steps
            .is_empty()
            .then(|| {
                route
                    .edge_indexes
                    .iter()
                    .map(|index| index.and_then(|index| u32::try_from(index).ok()))
                    .collect::<Option<Vec<_>>>()
            })
            .flatten();
        // Via candidates need only the primary's compact edges, not its
        // legs: they are listed while the legs are built.
        let policy = request.options.alternatives.normalized();
        let (primary, listed) = match primary_compact.as_deref() {
            Some(compact) if policy.requested_alternative_count > 0 => rayon::join(
                || self.build_result(&request, route, total_started),
                || self.via_candidates(&request, compact, &policy),
            ),
            _ => (self.build_result(&request, route, total_started), None),
        };
        let mut primary = primary?;
        primary.alternatives = self
            .find_alternatives(
                &request,
                &correlated,
                &mut primary,
                primary_compact.as_deref(),
                listed,
            )
            .map_err(|error| RouteFailure::new(error, primary.diagnostics.clone()))?;
        // Alternatives that stopped are dropped, not reported: the request
        // still fails.
        cancellation
            .check()
            .map_err(|error| RouteFailure::new(error, primary.diagnostics.clone()))?;
        Ok(primary)
    }

    /// Correlates both endpoints of `request` (alternatives reuse them).
    fn correlate(&self, request: &LegRequest) -> Result<Correlated, RouteFailure> {
        let view = self.route_view();
        if !view.is_ready() {
            return Err(RouteFailure::new(
                RouteError::GraphNotReady,
                RouteDiagnostics::default(),
            ));
        }
        self.validate_sar(&request.options, request.profile)
            .map_err(|error| RouteFailure::new(error, RouteDiagnostics::default()))?;
        if !view.has_route_edges_for_profile(request.profile) {
            return Err(RouteFailure::new(
                RouteError::ImpossibleMobilityProfile {
                    profile: request.profile,
                },
                RouteDiagnostics::default(),
            ));
        }

        let policy = CorrelationPolicy::from(&request.options);
        let correlator = Correlator::new(view);
        let correlation_started = Instant::now();
        let mut origin_search = correlator.search_expanding(
            request.origin,
            request.profile,
            &policy,
            &request.origin_options,
        );
        let mut origin_relaxed_correlation_retry_count = 0usize;
        if origin_search.candidates.is_empty()
            && request.origin_options.has_scoring_hints()
            && let Some(retry) = relaxed_correlation_retry(
                &correlator,
                request.origin,
                request.profile,
                &policy,
                &request.origin_options,
                origin_search.clone(),
            )
        {
            origin_search = retry;
            origin_relaxed_correlation_retry_count = 1;
        }
        if origin_search.candidates.is_empty() {
            let diagnostics = RouteDiagnostics {
                origin_candidate_count: 0,
                origin_rejected_candidate_count: origin_search.rejected_candidate_count,
                rejected_candidate_count: origin_search.rejected_candidate_count,
                origin_rejected_candidates: origin_search.rejected_candidates,
                correlation_duration_ns: elapsed_ns(correlation_started.elapsed()),
                correlation_mode: origin_search.mode,
                ..Default::default()
            };
            return Err(RouteFailure::new(
                RouteError::NoCandidate { endpoint: "origin" },
                diagnostics,
            ));
        }
        let mut destination_search = correlator.search_expanding(
            request.destination,
            request.profile,
            &policy,
            &request.destination_options,
        );
        let mut destination_relaxed_correlation_retry_count = 0usize;
        if destination_search.candidates.is_empty()
            && request.destination_options.has_scoring_hints()
            && let Some(retry) = relaxed_correlation_retry(
                &correlator,
                request.destination,
                request.profile,
                &policy,
                &request.destination_options,
                destination_search.clone(),
            )
        {
            destination_search = retry;
            destination_relaxed_correlation_retry_count = 1;
        }
        if destination_search.candidates.is_empty() {
            let diagnostics = RouteDiagnostics {
                origin_candidate_count: origin_search.candidates.len(),
                destination_candidate_count: 0,
                rejected_candidate_count: origin_search.rejected_candidate_count
                    + destination_search.rejected_candidate_count,
                origin_rejected_candidate_count: origin_search.rejected_candidate_count,
                destination_rejected_candidate_count: destination_search.rejected_candidate_count,
                origin_rejected_candidates: origin_search.rejected_candidates,
                destination_rejected_candidates: destination_search.rejected_candidates,
                correlation_duration_ns: elapsed_ns(correlation_started.elapsed()),
                correlation_mode: origin_search.mode,
                ..Default::default()
            };
            return Err(RouteFailure::new(
                RouteError::NoCandidate {
                    endpoint: "destination",
                },
                diagnostics,
            ));
        }

        // Components are the legal network's: a SAR route may join two
        // through the roads its relaxations open.
        if policy.sar_open.is_none()
            && components_are_disconnected(
                view,
                &origin_search.candidates,
                &destination_search.candidates,
            )
        {
            let diagnostics = RouteDiagnostics {
                origin_candidate_count: origin_search.candidates.len(),
                destination_candidate_count: destination_search.candidates.len(),
                correlation_duration_ns: elapsed_ns(correlation_started.elapsed()),
                correlation_mode: origin_search.mode,
                ..Default::default()
            };
            return Err(RouteFailure::new(RouteError::Disconnected, diagnostics));
        }
        let correlation_duration_ns = elapsed_ns(correlation_started.elapsed());

        let diagnostics = RouteDiagnostics {
            origin_candidate_count: origin_search.candidates.len(),
            destination_candidate_count: destination_search.candidates.len(),
            rejected_candidate_count: origin_search.rejected_candidate_count
                + destination_search.rejected_candidate_count,
            origin_rejected_candidate_count: origin_search.rejected_candidate_count,
            destination_rejected_candidate_count: destination_search.rejected_candidate_count,
            origin_best_candidate_score_basis_points: origin_search
                .candidates
                .first()
                .map(|candidate| score_basis_points(candidate.score)),
            destination_best_candidate_score_basis_points: destination_search
                .candidates
                .first()
                .map(|candidate| score_basis_points(candidate.score)),
            origin_best_outbound_reach_edge_count: origin_search
                .candidates
                .first()
                .and_then(|candidate| candidate.outbound_reach_edge_count),
            destination_best_inbound_reach_edge_count: destination_search
                .candidates
                .first()
                .and_then(|candidate| candidate.inbound_reach_edge_count),
            relaxed_correlation_retry_count: origin_relaxed_correlation_retry_count
                + destination_relaxed_correlation_retry_count,
            origin_relaxed_correlation_retry_count,
            destination_relaxed_correlation_retry_count,
            origin_rejected_candidates: origin_search.rejected_candidates,
            destination_rejected_candidates: destination_search.rejected_candidates,
            correlation_duration_ns,
            correlation_mode: origin_search.mode,
            ..Default::default()
        };
        Ok(Correlated {
            origins: origin_search.candidates,
            destinations: destination_search.candidates,
            diagnostics,
        })
    }

    /// Searches the route between correlated endpoints, with the strict
    /// destination-only pass first for the requests it confines (see
    /// `package::destination`).
    fn search_correlated(
        &self,
        request: &LegRequest,
        correlated: &Correlated,
        total_started: Instant,
    ) -> Result<SearchRoute, RouteFailure> {
        let view = self.route_view();
        let diagnostics = &correlated.diagnostics;
        let search_started = Instant::now();
        let strict_destination_only_pass = view
            .routing_package()
            .confines_destination_only(&request.options, request.profile);
        let first_result =
            Search::new_with_destination_only_policy(view, request, !strict_destination_only_pass)
                .find(
                    &correlated.origins,
                    &correlated.destinations,
                    diagnostics.clone(),
                );
        let mut route = match first_result {
            Ok(route) => route,
            Err(failure) if strict_destination_only_pass && failure.error.is_no_path() => {
                let mut retry_diagnostics = *failure.diagnostics;
                retry_diagnostics.destination_only_relaxed_retry_count += 1;
                match Search::new_with_destination_only_policy(view, request, true).find(
                    &correlated.origins,
                    &correlated.destinations,
                    retry_diagnostics,
                ) {
                    Ok(route) => route,
                    Err(mut failure) => {
                        self.enrich_search_failure_diagnostics(
                            &mut failure.diagnostics,
                            search_started,
                            total_started,
                        );
                        return Err(failure);
                    }
                }
            }
            Err(mut failure) => {
                self.enrich_search_failure_diagnostics(
                    &mut failure.diagnostics,
                    search_started,
                    total_started,
                );
                return Err(failure);
            }
        };
        route.diagnostics.search_duration_ns = elapsed_ns(search_started.elapsed());
        Ok(route)
    }

    /// Checks the request's SAR option against the package (see
    /// `crate::sar::validate_request`).
    pub(crate) fn validate_sar(
        &self,
        options: &RouteOptions,
        profile: crate::types::MobilityProfile,
    ) -> Result<(), RouteError> {
        if options.sar.is_none() {
            return Ok(());
        }
        crate::sar::validate_request(
            options.sar.as_ref(),
            profile,
            &self.package().sar_capability(),
        )
    }

    /// Builds the legs of a found route under `request`'s options.
    fn build_result(
        &self,
        request: &LegRequest,
        route: SearchRoute,
        total_started: Instant,
    ) -> Result<LegResult, RouteFailure> {
        let route_leg_started = Instant::now();
        let route_diagnostics = route.diagnostics.clone();
        let mut result =
            build_route_result(self.route_view(), route, request.profile, &request.options)
                .map_err(|error| RouteFailure::new(error, route_diagnostics))?;
        crate::sar::add_off_network_uses(
            &mut result,
            request.origin,
            request.destination,
            request.profile,
            &request.options,
        );
        result.diagnostics.route_leg_build_duration_ns = elapsed_ns(route_leg_started.elapsed());
        result.diagnostics.total_compute_duration_ns = elapsed_ns(total_started.elapsed());
        Ok(result)
    }

    fn enrich_search_failure_diagnostics(
        &self,
        diagnostics: &mut RouteDiagnostics,
        search_started: Instant,
        total_started: Instant,
    ) {
        diagnostics.search_duration_ns = elapsed_ns(search_started.elapsed());
        diagnostics.total_compute_duration_ns = elapsed_ns(total_started.elapsed());
    }

    /// Alternatives by penalising the primary's edges: each attempt searches
    /// the correlated endpoints again with penalty multipliers, is judged on
    /// its cost under the request's own options (the penalties removed) and
    /// on the share of its length it has with the primary, and only accepted
    /// ones get legs.
    fn find_alternatives(
        &self,
        request: &LegRequest,
        correlated: &Correlated,
        primary: &mut LegResult,
        primary_compact: Option<&[u32]>,
        listed: Option<ViaCandidates>,
    ) -> Result<Vec<RouteAlternative>, RouteError> {
        let policy = request.options.alternatives.normalized();
        if policy.requested_alternative_count == 0 {
            return Ok(Vec::new());
        }
        if let Some(compact) = primary_compact
            && let Some(alternatives) =
                self.via_alternatives(request, correlated, primary, compact, &policy, listed)?
        {
            return Ok(alternatives);
        }

        let mut alternatives = Vec::new();
        let primary_penalty_edges = alternative_primary_penalty_edges(&primary.edge_ids);
        let primary_edges = primary.edge_ids.iter().copied().collect::<BTreeSet<_>>();
        let mut persistent_penalized_edges = Vec::new();
        let primary_warning_count =
            warning_edge_count(&primary.edge_ids, &request.options.warning_edge_ids);
        // Routes compete on their cost with approach costs, which their
        // reported costs leave out (see `approach_costs`).
        let total = |cost: f64, origin: &CorrelatedLocation, destination: &CorrelatedLocation| {
            cost + approach_cost(origin, &correlated.origins)
                + approach_cost(destination, &correlated.destinations)
        };
        let primary_total = total(primary.cost, &primary.origin, &primary.destination);

        for attempt in 0..policy.requested_alternative_count.saturating_mul(3).max(1) {
            primary.diagnostics.alternative_attempt_count += 1;
            let mut alternative_request = request.clone();
            alternative_request
                .options
                .alternatives
                .requested_alternative_count = 0;
            let mut penalized_edges = alternative_penalty_window(
                &primary_penalty_edges,
                attempt,
                policy.requested_alternative_count,
            );
            penalized_edges.extend(persistent_penalized_edges.iter().copied());
            penalized_edges.sort_unstable();
            penalized_edges.dedup();
            for edge_id in &penalized_edges {
                let multiplier = alternative_request
                    .options
                    .edge_cost_multipliers
                    .entry(*edge_id)
                    .or_insert(1.0);
                *multiplier *= 5.0 + attempt as f64;
            }

            let Ok(mut candidate) =
                self.search_correlated(&alternative_request, correlated, Instant::now())
            else {
                primary.diagnostics.rejected_alternative_search_failed_count += 1;
                break;
            };
            if candidate.edge_ids == primary.edge_ids
                || alternatives.iter().any(|alternative: &RouteAlternative| {
                    alternative.edge_ids == candidate.edge_ids
                })
            {
                primary.diagnostics.rejected_alternative_duplicate_count += 1;
                persistent_penalized_edges.extend(candidate.edge_ids.iter().copied());
                continue;
            }

            let pieces = self.traversed_pieces(&candidate);
            candidate.cost -=
                self.penalty_cost(request, &alternative_request, &pieces, &penalized_edges);
            let shared_edge_ratio = shared_length_ratio(&primary_edges, &pieces);
            let candidate_total = total(candidate.cost, &candidate.origin, &candidate.destination);
            let cost_ratio = if primary_total > f64::EPSILON {
                candidate_total / primary_total
            } else {
                1.0
            };
            let candidate_warning_count =
                warning_edge_count(&candidate.edge_ids, &request.options.warning_edge_ids);
            let warning_difference = primary_warning_count.saturating_sub(candidate_warning_count);

            if shared_edge_ratio > policy.max_shared_edge_ratio {
                primary.diagnostics.rejected_alternative_shared_edge_count += 1;
                persistent_penalized_edges.extend(candidate.edge_ids.iter().copied());
                continue;
            }
            if cost_ratio > policy.max_cost_ratio {
                primary.diagnostics.rejected_alternative_cost_ratio_count += 1;
                persistent_penalized_edges.extend(candidate.edge_ids.iter().copied());
                continue;
            }
            if warning_difference < policy.minimum_warning_difference {
                primary
                    .diagnostics
                    .rejected_alternative_warning_difference_count += 1;
                persistent_penalized_edges.extend(candidate.edge_ids.iter().copied());
                continue;
            }

            let label = if candidate_warning_count < primary_warning_count {
                RouteAlternativeLabel::Safer
            } else if candidate_warning_count > 0 {
                RouteAlternativeLabel::WarningBearing
            } else {
                RouteAlternativeLabel::Alternative
            };
            let (quality_score, quality_reasons) = route_alternative_quality(
                shared_edge_ratio,
                cost_ratio,
                primary_warning_count,
                candidate_warning_count,
                warning_difference,
                &policy,
            );

            persistent_penalized_edges.extend(candidate.edge_ids.iter().copied());
            // Legs under the request's own options, as for the primary.
            let candidate = self
                .build_result(request, candidate, Instant::now())
                .map_err(|failure| failure.error)?;
            primary.diagnostics.accepted_alternative_count += 1;
            alternatives.push(RouteAlternative {
                label,
                edge_ids: candidate.edge_ids,
                distance_m: candidate.distance_m,
                cost: candidate.cost,
                duration_s: candidate.duration_s,
                shared_edge_ratio,
                warning_edge_count: candidate_warning_count,
                quality_score,
                quality_reasons,
                geometry: candidate.geometry,
                legs: candidate.legs,
                maneuvers: candidate.maneuvers,
                diagnostics: candidate.diagnostics,
            });
            if alternatives.len() >= policy.requested_alternative_count {
                break;
            }
        }

        Ok(alternatives)
    }

    /// Alternatives through via edges (see `package::cch::via_plateaus`):
    /// the middles of plateaus, stretches on both the shortest-path tree from
    /// the origin and the one to the destination, longest first, then single
    /// edges. Each candidate is routed as two searches meeting at the middle
    /// of its via edge (so no turn is made there), then judged as the penalty
    /// method's candidates are, against the primary and every alternative
    /// accepted before it. `listed`: the candidates, if
    /// [`RoutingEngine::via_candidates`] already listed them. `None` when the
    /// request is not a plain package route on a package with CCH potentials
    /// (overlays, SAR relaxations): the penalty method runs instead.
    fn via_alternatives(
        &self,
        request: &LegRequest,
        correlated: &Correlated,
        primary: &mut LegResult,
        primary_compact: &[u32],
        policy: &RouteAlternativePolicy,
        listed: Option<ViaCandidates>,
    ) -> Result<Option<Vec<RouteAlternative>>, RouteError> {
        self.with_via_plan(request, primary_compact, |plan| {
            self.via_alternatives_on(plan, request, correlated, primary, policy, listed)
        })
        .transpose()
    }

    fn via_alternatives_on(
        &self,
        plan: &ViaPlan<'_>,
        request: &LegRequest,
        correlated: &Correlated,
        primary: &mut LegResult,
        policy: &RouteAlternativePolicy,
        listed: Option<ViaCandidates>,
    ) -> Result<Vec<RouteAlternative>, RouteError> {
        let ViaCandidates {
            listed: mut candidates,
            mut trees,
        } = listed.unwrap_or_else(|| {
            let (listed, trees) = crate::package::cch::via_plateaus(
                &plan.graph,
                plan.topology,
                plan.metric,
                plan.costs,
                plan.source,
                plan.target,
                policy.max_cost_ratio,
                VIA_MIN_PLATEAU,
                &plan.excluded,
            );
            ViaCandidates { listed, trees }
        });

        // Accepted routes, their legs built once the search is done.
        let mut accepted = Vec::<AcceptedVia>::new();
        let primary_edges = primary.edge_ids.iter().copied().collect::<BTreeSet<_>>();
        let primary_warning_count =
            warning_edge_count(&primary.edge_ids, &request.options.warning_edge_ids);
        let total = |cost: f64, origin: &CorrelatedLocation, destination: &CorrelatedLocation| {
            cost + approach_cost(origin, &correlated.origins)
                + approach_cost(destination, &correlated.destinations)
        };
        let primary_total = total(primary.cost, &primary.origin, &primary.destination);
        let mut half_request = request.clone();
        half_request
            .options
            .alternatives
            .requested_alternative_count = 0;
        // Searches per request; candidates on a route already searched are
        // skipped (they would find it again).
        let attempts = policy.requested_alternative_count.saturating_mul(3).max(1) + 2;
        let mut covered = rustc_hash::FxHashSet::<u32>::default();
        let mut next = 0;
        loop {
            if primary.diagnostics.alternative_attempt_count >= attempts {
                break;
            }
            if next == candidates.len() {
                // The single edges come after the plateaus, so they are
                // listed only once the plateaus have run out.
                let Some(trees) = trees.take() else {
                    break;
                };
                candidates = crate::package::cch::via_singles(
                    &plan.graph,
                    plan.topology,
                    plan.costs,
                    &trees,
                    &plan.excluded,
                );
                next = 0;
                if candidates.is_empty() {
                    break;
                }
            }
            let candidate = candidates[next];
            next += 1;
            if covered.contains(&candidate.edge) {
                continue;
            }
            primary.diagnostics.alternative_attempt_count += 1;
            let Some(record) = self
                .route_view()
                .edge_record_by_raw_edge_index(candidate.edge as usize)
            else {
                continue;
            };
            let via = via_location(&record, 0.5);
            // The two halves are independent searches.
            let (first, second) = rayon::join(
                || {
                    self.search_correlated(
                        &half_request,
                        &Correlated {
                            origins: correlated.origins.clone(),
                            destinations: vec![via.clone()],
                            diagnostics: correlated.diagnostics.clone(),
                        },
                        Instant::now(),
                    )
                },
                || {
                    self.search_correlated(
                        &half_request,
                        &Correlated {
                            origins: vec![via.clone()],
                            destinations: correlated.destinations.clone(),
                            diagnostics: correlated.diagnostics.clone(),
                        },
                        Instant::now(),
                    )
                },
            );
            let (Ok(first), Ok(second)) = (first, second) else {
                primary.diagnostics.rejected_alternative_search_failed_count += 1;
                continue;
            };
            let Some(route) = stitch_at_via(first, second, record.edge.edge_id) else {
                primary.diagnostics.rejected_alternative_search_failed_count += 1;
                continue;
            };
            // Two searches meeting mid-edge keep no restriction context
            // across the via edge: drop a route that breaks a complex
            // restriction there.
            let compact = route
                .edge_indexes
                .iter()
                .map(|index| index.and_then(|index| u32::try_from(index).ok()))
                .collect::<Option<Vec<_>>>();
            if let Some(edges) = &compact {
                covered.extend(edges.iter().copied());
            }
            if compact.is_none_or(|edges| {
                !plan
                    .graph
                    .restriction_violations(&edges, crate::package::profile_bit(request.profile))
                    .is_empty()
            }) {
                primary.diagnostics.rejected_alternative_search_failed_count += 1;
                continue;
            }
            if route.edge_ids == primary.edge_ids
                || accepted
                    .iter()
                    .any(|alternative| alternative.route.edge_ids == route.edge_ids)
            {
                primary.diagnostics.rejected_alternative_duplicate_count += 1;
                continue;
            }
            let pieces = self.traversed_pieces(&route);
            let shared_edge_ratio = shared_length_ratio(&primary_edges, &pieces);
            let shared_with_accepted = accepted
                .iter()
                .map(|alternative| shared_length_ratio(&alternative.edges, &pieces))
                .fold(0.0f64, f64::max);
            let route_total = total(route.cost, &route.origin, &route.destination);
            let cost_ratio = if primary_total > f64::EPSILON {
                route_total / primary_total
            } else {
                1.0
            };
            let candidate_warning_count =
                warning_edge_count(&route.edge_ids, &request.options.warning_edge_ids);
            let warning_difference = primary_warning_count.saturating_sub(candidate_warning_count);
            if shared_edge_ratio.max(shared_with_accepted) > policy.max_shared_edge_ratio {
                primary.diagnostics.rejected_alternative_shared_edge_count += 1;
                continue;
            }
            if cost_ratio > policy.max_cost_ratio {
                primary.diagnostics.rejected_alternative_cost_ratio_count += 1;
                continue;
            }
            if warning_difference < policy.minimum_warning_difference {
                primary
                    .diagnostics
                    .rejected_alternative_warning_difference_count += 1;
                continue;
            }
            let label = if candidate_warning_count < primary_warning_count {
                RouteAlternativeLabel::Safer
            } else if candidate_warning_count > 0 {
                RouteAlternativeLabel::WarningBearing
            } else {
                RouteAlternativeLabel::Alternative
            };
            let (quality_score, quality_reasons) = route_alternative_quality(
                shared_edge_ratio,
                cost_ratio,
                primary_warning_count,
                candidate_warning_count,
                warning_difference,
                policy,
            );
            accepted.push(AcceptedVia {
                edges: route.edge_ids.iter().copied().collect(),
                diagnostics: Box::new(primary.diagnostics.clone()),
                label,
                shared_edge_ratio,
                warning_edge_count: candidate_warning_count,
                quality_score,
                quality_reasons,
                route,
            });
            primary.diagnostics.accepted_alternative_count += 1;
            if accepted.len() >= policy.requested_alternative_count {
                break;
            }
        }

        // Legs under the request's own options, as for the primary, all at
        // once. A failed build fails the request with the diagnostics as they
        // stood when that route was accepted.
        let built = accepted
            .into_par_iter()
            .map(|alternative| {
                let AcceptedVia {
                    route,
                    diagnostics,
                    label,
                    shared_edge_ratio,
                    warning_edge_count,
                    quality_score,
                    quality_reasons,
                    ..
                } = alternative;
                self.build_result(request, route, Instant::now())
                    .map(|route| RouteAlternative {
                        label,
                        edge_ids: route.edge_ids,
                        distance_m: route.distance_m,
                        cost: route.cost,
                        duration_s: route.duration_s,
                        shared_edge_ratio,
                        warning_edge_count,
                        quality_score,
                        quality_reasons,
                        geometry: route.geometry,
                        legs: route.legs,
                        maneuvers: route.maneuvers,
                        diagnostics: route.diagnostics,
                    })
                    .map_err(|failure| (failure.error, diagnostics))
            })
            .collect::<Vec<_>>();
        let mut alternatives = Vec::with_capacity(built.len());
        for alternative in built {
            match alternative {
                Ok(alternative) => alternatives.push(alternative),
                Err((error, diagnostics)) => {
                    primary.diagnostics = *diagnostics;
                    return Err(error);
                }
            }
        }
        Ok(alternatives)
    }

    /// Lists the via candidates of a plain package route (see
    /// [`RoutingEngine::via_alternatives`]): its plateaus, and the trees for
    /// its single-edge candidates. `None` when the via method does not apply.
    fn via_candidates(
        &self,
        request: &LegRequest,
        primary_compact: &[u32],
        policy: &RouteAlternativePolicy,
    ) -> Option<ViaCandidates> {
        self.with_via_plan(request, primary_compact, |plan| {
            let (listed, trees) = crate::package::cch::via_plateaus(
                &plan.graph,
                plan.topology,
                plan.metric,
                plan.costs,
                plan.source,
                plan.target,
                policy.max_cost_ratio,
                VIA_MIN_PLATEAU,
                &plan.excluded,
            );
            ViaCandidates { listed, trees }
        })
    }

    /// Runs `work` on what a plain package route's via candidates are
    /// listed on; `None` when the request is not a plain package route on a
    /// package with CCH potentials (overlays, SAR relaxations).
    fn with_via_plan<T>(
        &self,
        request: &LegRequest,
        primary_compact: &[u32],
        work: impl FnOnce(&ViaPlan<'_>) -> T,
    ) -> Option<T> {
        if request.options.dynamic_overlay.is_some()
            || request.options.sar.is_some()
            || primary_compact.len() < 3
        {
            return None;
        }
        let package = self.route_view().routing_package();
        let graph = package.view();
        let potentials = graph.cch?;
        let handle =
            package.request_column_sar(request.profile, &request.options.costing, true, None);
        let column = handle.column(&graph);
        let plan = crate::package::search::potential_plan(
            graph,
            &column,
            &rustc_hash::FxHashMap::default(),
            false,
        )?;
        let source = graph.edge_target[primary_compact[0] as usize];
        let target = graph.edge_source(primary_compact[primary_compact.len() - 1] as usize);
        if source == target {
            return None;
        }
        Some(work(&ViaPlan {
            graph,
            topology: &potentials.topology,
            metric: plan.metric.metric(),
            costs: plan.metric.costs(),
            source,
            target,
            excluded: primary_compact.iter().copied().collect(),
        }))
    }

    /// The pieces a found route traverses: (edge id, compact edge, fraction,
    /// length in metres), connectors with no compact edge.
    fn traversed_pieces(&self, route: &SearchRoute) -> Vec<(u64, Option<u32>, f64, f64)> {
        let view = self.route_view().routing_package().view();
        // A package route carries its compact edges (no edge id lookup).
        let piece = |index: usize, edge_id: u64, fraction: f64| {
            let hint = route
                .edge_indexes
                .get(index)
                .copied()
                .flatten()
                .and_then(|edge| u32::try_from(edge).ok());
            let edge = view.edge_for_id_hinted(hint, edge_id);
            let length = edge.map_or(0.0, |edge| f64::from(view.edge_length[edge as usize]));
            (edge_id, edge, fraction, length * fraction)
        };
        if !route.steps.is_empty() {
            return route
                .steps
                .iter()
                .enumerate()
                .map(|(index, step)| match step {
                    RouteSearchStep::GraphEdge {
                        edge_id,
                        start_fraction,
                        end_fraction,
                        ..
                    } => piece(
                        index,
                        *edge_id,
                        (end_fraction - start_fraction).clamp(0.0, 1.0),
                    ),
                    RouteSearchStep::VirtualConnector {
                        edge_id,
                        distance_m,
                        ..
                    } => (*edge_id, None, 1.0, *distance_m),
                })
                .collect();
        }
        let (start, end) = (
            route.origin.fraction_along_edge.clamp(0.0, 1.0),
            route.destination.fraction_along_edge.clamp(0.0, 1.0),
        );
        let last = route.edge_ids.len().saturating_sub(1);
        route
            .edge_ids
            .iter()
            .enumerate()
            .map(|(index, edge_id)| {
                let from = if index == 0 && *edge_id == route.origin.edge_id {
                    start
                } else {
                    0.0
                };
                let to = if index == last { end } else { 1.0 };
                // A single edge is the direct path from the origin.
                let fraction = if last == 0 {
                    to - from
                } else if index == 0 {
                    1.0 - from
                } else {
                    to
                };
                piece(index, *edge_id, fraction.clamp(0.0, 1.0))
            })
            .collect()
    }

    /// What `alternative`'s penalty multipliers on `penalized` add to a path
    /// of `pieces` over `request`'s own costing: the difference of the two
    /// request costs of each penalised piece (turn and connector costs do
    /// not change).
    fn penalty_cost(
        &self,
        request: &LegRequest,
        alternative: &LegRequest,
        pieces: &[(u64, Option<u32>, f64, f64)],
        penalized: &[u64],
    ) -> f64 {
        let view = self.route_view().routing_package().view();
        let own = CostingPolicy::new(request.profile, &request.options);
        let penalised = CostingPolicy::new(alternative.profile, &alternative.options);
        pieces
            .iter()
            .filter(|(edge_id, ..)| penalized.binary_search(edge_id).is_ok())
            .filter_map(|(_, edge, fraction, _)| {
                let edge = (*edge)?;
                let cost =
                    |policy| f64::from(request_edge_cost(view, policy, request.profile, edge));
                let extra = cost(&penalised) - cost(&own);
                extra.is_finite().then_some(extra * fraction)
            })
            .sum()
    }
}

/// A location at `fraction` along a package edge, as a search endpoint (a
/// via edge's middle).
fn via_location(record: &GraphViewEdge, fraction: f64) -> CorrelatedLocation {
    let (a, b) = (
        GeoPoint::new(record.source.lat, record.source.lng),
        GeoPoint::new(record.target.lat, record.target.lng),
    );
    CorrelatedLocation {
        edge_id: record.edge.edge_id,
        compact_edge: crate::types::CompactEdgeHint::of(record.raw_edge_index),
        shared_edge_info_id: record.edge.shared_edge_info_id,
        opposing_edge_id: record.edge.opposing_edge_id,
        road_group_key: None,
        same_road_candidate_count: 0,
        source_node_id: record.source.id,
        target_node_id: record.target.id,
        snapped_point: GeoPoint::new(
            a.lat + (b.lat - a.lat) * fraction,
            a.lng + (b.lng - a.lng) * fraction,
        ),
        distance_m: 0.0,
        fraction_along_edge: fraction,
        heading_deg: None,
        heading_delta_deg: None,
        side: crate::types::CorrelationSide::Unknown,
        component_size: None,
        outbound_reach_edge_count: None,
        inbound_reach_edge_count: None,
        layer: None,
        level: None,
        score: 1.0,
        score_breakdown: Default::default(),
        confidence: 1.0,
    }
}

/// One route from two searches that meet at the same point of edge `via`
/// (the first ends there, the second starts there): `None` unless both are
/// plain package routes through it.
fn stitch_at_via(first: SearchRoute, second: SearchRoute, via: u64) -> Option<SearchRoute> {
    if !first.steps.is_empty()
        || !second.steps.is_empty()
        || first.edge_ids.last() != Some(&via)
        || second.edge_ids.first() != Some(&via)
    {
        return None;
    }
    let mut edge_ids = first.edge_ids;
    edge_ids.extend_from_slice(&second.edge_ids[1..]);
    let mut edge_indexes = first.edge_indexes;
    edge_indexes.extend_from_slice(second.edge_indexes.get(1..)?);
    let mut diagnostics = first.diagnostics;
    diagnostics.visited_edge_count += second.diagnostics.visited_edge_count;
    diagnostics.settled_node_count += second.diagnostics.settled_node_count;
    diagnostics.search_duration_ns += second.diagnostics.search_duration_ns;
    Some(SearchRoute {
        origin: first.origin,
        destination: second.destination,
        edge_ids,
        edge_indexes,
        steps: Vec::new(),
        distance_m: first.distance_m + second.distance_m,
        duration_s: first.duration_s + second.duration_s,
        cost: first.cost + second.cost,
        diagnostics,
    })
}

/// A route request's correlated endpoints and the diagnostics so far.
struct Correlated {
    origins: Vec<CorrelatedLocation>,
    destinations: Vec<CorrelatedLocation>,
    diagnostics: RouteDiagnostics,
}

pub(crate) fn relaxed_correlation_retry(
    correlator: &Correlator<'_>,
    point: GeoPoint,
    profile: crate::types::MobilityProfile,
    policy: &CorrelationPolicy,
    endpoint_options: &crate::types::RouteEndpointOptions,
    previous: CandidateSearch,
) -> Option<CandidateSearch> {
    let relaxed_options = endpoint_options.relaxed_for_retry();
    let retry = correlator.search(point, profile, policy, &relaxed_options);
    if retry.candidates.is_empty() {
        return None;
    }
    Some(merge_correlation_attempts(previous, retry))
}

fn merge_correlation_attempts(
    previous: CandidateSearch,
    mut retry: CandidateSearch,
) -> CandidateSearch {
    retry.rejected_candidate_count += previous.rejected_candidate_count;
    retry
        .rejected_candidates
        .splice(0..0, previous.rejected_candidates);
    retry
}

fn score_basis_points(score: f64) -> u32 {
    (score.clamp(0.0, 1.0) * 10_000.0).round() as u32
}

fn elapsed_ns(duration: Duration) -> u64 {
    duration.as_nanos().try_into().unwrap_or(u64::MAX).max(1)
}

struct Search<'a> {
    access: &'a PackageGraphView,
    request: &'a LegRequest,
    costing: CostingPolicy<'a>,
    /// Costs the edges a search chose: a strict pass starts or ends in
    /// destination-only pockets that `costing` refuses.
    path_costing: CostingPolicy<'a>,
}

impl<'a> Search<'a> {
    fn new_with_destination_only_policy(
        access: &'a PackageGraphView,
        request: &'a LegRequest,
        allow_destination_only: bool,
    ) -> Self {
        Self {
            access,
            request,
            costing: CostingPolicy::with_destination_only_policy(
                request.profile,
                &request.options,
                allow_destination_only,
            ),
            path_costing: CostingPolicy::new(request.profile, &request.options),
        }
    }

    /// The record of `edge_id` (the package rebuilds it from its sections).
    fn search_edge_by_id(&self, edge_id: u64) -> Option<GraphViewEdge> {
        self.access.search_edge_by_id(edge_id)
    }

    /// Every route runs on the routing package (`dispatch`).
    fn find(
        &self,
        origin_candidates: &[CorrelatedLocation],
        destination_candidates: &[CorrelatedLocation],
        diagnostics: RouteDiagnostics,
    ) -> Result<SearchRoute, RouteFailure> {
        self.search_package(origin_candidates, destination_candidates, &diagnostics)
    }

    fn direct_same_edge(
        &self,
        origin: &CorrelatedLocation,
        destination: &CorrelatedLocation,
        diagnostics: &mut RouteDiagnostics,
    ) -> Result<Option<SearchRoute>, RouteError> {
        if origin.edge_id != destination.edge_id {
            return Ok(None);
        }
        if origin.fraction_along_edge > destination.fraction_along_edge {
            return Ok(None);
        }
        let record = self
            .access
            .candidate_edge(origin)
            .and_then(|edge| self.access.edge_record_by_raw_edge_index(edge as usize))
            .ok_or(RouteError::MissingEdgeMetadata {
                edge_id: origin.edge_id,
            })?;
        let Some(edge_cost) = self.cost_edge(&record, diagnostics) else {
            return Ok(None);
        };
        self.record_visited_edge(diagnostics)?;
        let fraction =
            (destination.fraction_along_edge - origin.fraction_along_edge).clamp(0.0, 1.0);
        Ok(Some(SearchRoute {
            origin: origin.clone(),
            destination: destination.clone(),
            edge_ids: vec![origin.edge_id],
            edge_indexes: vec![Some(record.raw_edge_index)],
            steps: Vec::new(),
            distance_m: f64::from(record.edge.distance_m) * fraction,
            duration_s: edge_cost.duration_s * fraction,
            cost: edge_cost.cost * fraction,
            diagnostics: diagnostics.clone(),
        }))
    }

    fn cost_edge(
        &self,
        record: &GraphViewEdge,
        diagnostics: &mut RouteDiagnostics,
    ) -> Option<CostedEdge> {
        match self.path_costing.edge_cost(record) {
            EdgeCostDecision::Allowed(cost) => {
                if cost.is_avoided {
                    diagnostics.avoided_edge_count += 1;
                }
                if cost.is_destination_only_penalized {
                    diagnostics.destination_only_penalty_count += 1;
                }
                if cost.has_ford_penalty {
                    diagnostics.ford_penalty_count += 1;
                }
                if cost.has_toll_penalty {
                    diagnostics.toll_penalty_count += 1;
                }
                if cost.has_ferry_penalty {
                    diagnostics.ferry_penalty_count += 1;
                }
                if cost.has_gate_penalty {
                    diagnostics.gate_penalty_count += 1;
                }
                if cost.has_access_restriction_penalty {
                    diagnostics.access_restriction_penalty_count += 1;
                }
                if cost.has_overlay_multiplier {
                    diagnostics.overlay_multiplier_count += 1;
                }
                Some(cost)
            }
            EdgeCostDecision::Blocked => {
                diagnostics.blocked_edge_count += 1;
                None
            }
            EdgeCostDecision::NotAccessible => {
                diagnostics.inaccessible_edge_count += 1;
                None
            }
        }
    }

    fn record_visited_edge(&self, diagnostics: &mut RouteDiagnostics) -> Result<(), RouteError> {
        diagnostics.visited_edge_count += 1;
        diagnostics.normal_edge_expanded_count += 1;
        diagnostics.forward_visited_edge_count += 1;
        self.check_visited_edge_budget(diagnostics)
    }

    fn check_visited_edge_budget(&self, diagnostics: &RouteDiagnostics) -> Result<(), RouteError> {
        if let Some(max_visited_edge_count) = self.request.options.max_visited_edge_count
            && diagnostics.visited_edge_count > max_visited_edge_count
        {
            return Err(RouteError::SearchBudgetExceeded {
                visited_edge_count: diagnostics.visited_edge_count,
                max_visited_edge_count,
            });
        }
        Ok(())
    }
}

pub(crate) fn components_are_disconnected(
    access: &PackageGraphView,
    origins: &[CorrelatedLocation],
    destinations: &[CorrelatedLocation],
) -> bool {
    let origin_components = origins
        .iter()
        .filter_map(|candidate| access.candidate_component_id(candidate, true))
        .collect::<BTreeSet<_>>();
    let destination_components = destinations
        .iter()
        .filter_map(|candidate| access.candidate_component_id(candidate, false))
        .collect::<BTreeSet<_>>();
    !origin_components.is_empty()
        && !destination_components.is_empty()
        && origin_components.is_disjoint(&destination_components)
}

/// The share of a candidate's traversed length on edges of the primary.
fn shared_length_ratio(primary: &BTreeSet<u64>, pieces: &[(u64, Option<u32>, f64, f64)]) -> f64 {
    let total = pieces.iter().map(|piece| piece.3).sum::<f64>();
    if total <= 0.0 {
        return 0.0;
    }
    let shared = pieces
        .iter()
        .filter(|(edge_id, ..)| primary.contains(edge_id))
        .map(|piece| piece.3)
        .sum::<f64>();
    shared / total
}

fn alternative_primary_penalty_edges(primary: &[u64]) -> Vec<u64> {
    let edges = primary
        .iter()
        .copied()
        .skip(1)
        .take(primary.len().saturating_sub(2))
        .collect::<Vec<_>>();
    if edges.is_empty() {
        primary.to_vec()
    } else {
        edges
    }
}

fn alternative_penalty_window(
    primary_penalty_edges: &[u64],
    attempt: usize,
    requested_alternative_count: usize,
) -> Vec<u64> {
    if primary_penalty_edges.is_empty() {
        return Vec::new();
    }
    let attempt_budget = requested_alternative_count.saturating_mul(3).max(1);
    let window_count = attempt_budget.min(primary_penalty_edges.len()).max(1);
    let window_len = primary_penalty_edges.len().div_ceil(window_count);
    let start = (attempt.saturating_mul(window_len)) % primary_penalty_edges.len();
    (0..window_len)
        .map(|offset| primary_penalty_edges[(start + offset) % primary_penalty_edges.len()])
        .collect()
}

fn route_alternative_quality(
    shared_edge_ratio: f64,
    cost_ratio: f64,
    primary_warning_count: usize,
    candidate_warning_count: usize,
    warning_difference: usize,
    policy: &RouteAlternativePolicy,
) -> (f64, Vec<String>) {
    let diversity_fit = (1.0 - shared_edge_ratio).clamp(0.0, 1.0);
    let cost_budget = (policy.max_cost_ratio - 1.0).max(0.01);
    let cost_fit = if cost_ratio <= 1.0 {
        1.0
    } else {
        (1.0 - ((cost_ratio - 1.0) / cost_budget)).clamp(0.0, 1.0)
    };
    let warning_fit = if primary_warning_count == 0 {
        if candidate_warning_count == 0 {
            1.0
        } else {
            0.5
        }
    } else {
        (warning_difference as f64 / primary_warning_count as f64).clamp(0.0, 1.0)
    };
    let score = (diversity_fit * 0.45 + cost_fit * 0.35 + warning_fit * 0.20).clamp(0.0, 1.0);
    let mut reasons = Vec::new();
    if shared_edge_ratio <= policy.max_shared_edge_ratio * 0.5 {
        reasons.push("low_shared_path".to_string());
    }
    if cost_ratio <= 1.10 {
        reasons.push("near_primary_cost".to_string());
    } else if cost_ratio > 1.25 {
        reasons.push("higher_cost".to_string());
    }
    if candidate_warning_count < primary_warning_count {
        reasons.push("fewer_warnings".to_string());
    } else if candidate_warning_count > 0 {
        reasons.push("warning_bearing".to_string());
    }
    if reasons.is_empty() {
        reasons.push("policy_accepted".to_string());
    }
    (score, reasons)
}

fn warning_edge_count(edge_ids: &[u64], warning_edge_ids: &BTreeSet<u64>) -> usize {
    edge_ids
        .iter()
        .filter(|edge_id| warning_edge_ids.contains(edge_id))
        .count()
}

#[cfg(test)]
mod tests;

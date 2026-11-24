use std::collections::{BTreeMap, BTreeSet};

use crate::correlator::Correlator;
use crate::costing::CostingPolicy;
use crate::limits::{MAX_ISOCHRONE_CONTOURS, MAX_ISOCHRONE_ORIGINS};
use crate::package::destination::relax_split_costs;
use crate::package::search::analysis::{
    AnalysisContour, AnalysisCounters, AnalysisPoint, AnalysisQuery, ContourMetric,
    IsochroneOutcome, IsochroneSeeding, overlay_split_costs, request_cost_overrides,
    request_duration_overrides, resolve_edge_id, resolve_location,
};
use crate::package::search::patch::{PatchCosts, PatchGraph, PatchQuery, analysis_overlay};
use crate::search::RoutingEngine;
use crate::trip::{
    apply_query_overlays, invalid_point, validate_query_options, validate_query_overlays,
};
use crate::types::{
    CorrelatedLocation, CorrelationPolicy, GeoPoint, RouteError, RouteIsochroneBoundaryFragment,
    RouteIsochroneBucket, RouteIsochroneContour, RouteIsochroneMetric, RouteIsochroneOrder,
    RouteIsochroneRequest, RouteIsochroneResult,
};

pub(crate) mod polygon;

impl RoutingEngine {
    pub fn isochrone(
        &self,
        mut request: RouteIsochroneRequest,
    ) -> Result<RouteIsochroneResult, RouteError> {
        validate_request(&request)?;
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
        apply_query_overlays(
            self,
            &request.overlays,
            request.profile,
            request.departure_at_ms,
            &mut request.options,
        )?;
        request.options.cancellation.check()?;
        let mut result = self.isochrone_from(&request)?;
        // Polygons are drawn from the buckets' edges, before they are left out.
        polygon::draw(
            self.route_graph_view().routing_package().view(),
            &request,
            &mut result,
        );
        for bucket in &mut result.buckets {
            if !request.include_edge_ids {
                bucket.edge_ids = Vec::new();
            }
            if !request.include_edge_ids || !request.include_origins {
                bucket.edge_origins = Vec::new();
            }
            if !request.include_origins {
                for fragment in &mut bucket.boundary_fragments {
                    fragment.origin = None;
                }
            }
        }
        Ok(result)
    }

    /// The isochrone of a validated request with its overlays applied.
    fn isochrone_from(
        &self,
        request: &RouteIsochroneRequest,
    ) -> Result<RouteIsochroneResult, RouteError> {
        let policy = CorrelationPolicy::from(&request.options);
        let correlator = Correlator::new(self.route_graph_view());
        // An origin with no connected road within the radius widens the
        // search as route and matrix locations do (and as far as the SAR
        // `off_network` relaxation reaches).
        let origins = origin_candidates(request, |point| {
            correlator
                .search_expanding(point, request.profile, &policy, &Default::default())
                .candidates
        })?;
        if !self
            .package()
            .confines_destination_only(&request.options, request.profile)
        {
            return self.isochrone_search(request, &origins, true);
        }
        if origins.snapped.len() > 1 {
            return self.multi_origin_isochrone(request, &origins);
        }
        let strict = self.isochrone_search(request, &origins, false)?;
        if !trapped(&strict) {
            return Ok(strict);
        }
        // An origin that cannot leave without crossing a destination-only
        // road gets the relaxed search, as routes do.
        let relaxed = self.isochrone_search(request, &origins, true)?;
        Ok(pick_relaxed(strict, relaxed))
    }

    /// The isochrone of several origins on a package that confines
    /// destination-only roads: each origin trapped behind one (see
    /// [`origin_fallbacks`]) contributes its relaxed isochrone, merged with
    /// the strict isochrone of the others.
    fn multi_origin_isochrone(
        &self,
        request: &RouteIsochroneRequest,
        origins: &OriginCandidates,
    ) -> Result<RouteIsochroneResult, RouteError> {
        let contours = analysis_contours(request);
        let view = self.route_graph_view().routing_package().view();
        let reach = |outcome: &IsochroneOutcome| {
            outcome_reach(
                outcome.reached.iter().map(|edge| {
                    (
                        view.edge_id(edge.edge as usize),
                        edge.cost,
                        edge.duration_s,
                        edge.distance_m,
                    )
                }),
                request,
            )
        };
        let mut searched = AnalysisCounters::default();
        let mut pass = |set: &[usize], allow_destination_only: bool, budget: Option<usize>| {
            let outcome = self.isochrone_outcome(
                request,
                &origins.only(set),
                allow_destination_only,
                budget,
            )?;
            searched.visited_edge_count += outcome.counters.visited_edge_count;
            searched.settled_edge_count += outcome.counters.settled_edge_count;
            Ok((reach(&outcome), outcome))
        };
        let (strict, relaxed) = origin_fallbacks(origins.snapped.len(), &mut pass)?;
        let mut merged = None::<IsochroneOutcome>;
        for outcome in strict.into_iter().chain(relaxed) {
            merged = Some(match merged {
                None => outcome,
                Some(merged) => merge_outcomes(merged, outcome, &contours),
            });
        }
        let mut outcome = merged.unwrap_or_default();
        outcome.counters = searched;
        Ok(self.isochrone_result(request, origins, outcome))
    }
}

/// Visited edges up to which a probe of one origin of several looks for
/// [`TRAPPED_ORIGIN_REACH`] edges: a probe that runs out of them reached
/// more than that.
const TRAPPED_PROBE_BUDGET: usize = 4_096;

/// Which origins of several a destination-only road traps, as a
/// single-origin isochrone of each would judge it: each origin alone is
/// probed strictly; one that reaches fewer than [`TRAPPED_ORIGIN_REACH`]
/// edges (and does not run out of its probe budget) is searched relaxed,
/// alone, and that relaxed search replaces it when it reaches at least twice
/// as far. `pass(origins, allow_destination_only, budget)` searches and
/// returns the reach (see [`outcome_reach`]) with the outcome. Returns the
/// strict search of the origins that keep it (none when every origin was
/// replaced) and the relaxed searches of the others.
pub(crate) fn origin_fallbacks<O>(
    origin_count: usize,
    pass: &mut OriginPass<'_, O>,
) -> Result<(Option<O>, Vec<O>), RouteError>
where
    O: Truncation,
{
    let mut strict = Vec::new();
    let mut relaxed = Vec::new();
    for origin in 0..origin_count {
        let (probe_reach, probe) = pass(&[origin], false, Some(TRAPPED_PROBE_BUDGET))?;
        if probe.truncated() || probe_reach >= TRAPPED_ORIGIN_REACH {
            strict.push(origin);
            continue;
        }
        let (relaxed_reach, outcome) = pass(&[origin], true, None)?;
        if relaxed_reach >= 2 * probe_reach {
            relaxed.push(outcome);
        } else {
            strict.push(origin);
        }
    }
    let strict = if strict.is_empty() {
        None
    } else {
        Some(pass(&strict, false, None)?.1)
    };
    Ok((strict, relaxed))
}

/// A search of some origins (`pass(origins, allow_destination_only,
/// budget)`, see [`origin_fallbacks`]) returning its reach and outcome.
pub(crate) type OriginPass<'a, O> =
    dyn FnMut(&[usize], bool, Option<usize>) -> Result<(usize, O), RouteError> + 'a;

/// Whether a search pass stopped at its budget.
pub(crate) trait Truncation {
    fn truncated(&self) -> bool;
}

impl Truncation for IsochroneOutcome {
    fn truncated(&self) -> bool {
        self.truncated
    }
}

/// The edges an isochrone reaches within its widest contour (see
/// [`reach`]), from reached edges as `(edge id, cost, duration, distance)`.
pub(crate) fn outcome_reach(
    reached: impl Iterator<Item = (u64, f64, f64, f64)> + Clone,
    request: &RouteIsochroneRequest,
) -> usize {
    request
        .contours
        .iter()
        .map(|contour| {
            reached
                .clone()
                .filter(|(_, cost, duration_s, distance_m)| {
                    let value = match contour.metric {
                        RouteIsochroneMetric::Cost => *cost,
                        RouteIsochroneMetric::TimeSeconds => *duration_s,
                        RouteIsochroneMetric::DistanceM => *distance_m,
                    };
                    value <= contour.contour
                })
                .map(|(edge_id, ..)| edge_id)
                .collect::<BTreeSet<_>>()
                .len()
        })
        .max()
        .unwrap_or(0)
}

/// Two isochrone searches of different origins as one: each edge at its
/// least value on each metric, and the boundary crossings of both, except
/// on edges the merged isochrone reaches in full within that contour.
fn merge_outcomes(
    mut into: IsochroneOutcome,
    other: IsochroneOutcome,
    contours: &[AnalysisContour],
) -> IsochroneOutcome {
    let mut slot = into
        .reached
        .iter()
        .enumerate()
        .map(|(index, edge)| (edge.edge, index))
        .collect::<rustc_hash::FxHashMap<_, _>>();
    for edge in other.reached {
        match slot.get(&edge.edge) {
            Some(index) => into.reached[*index].merge_edge(&edge),
            None => {
                slot.insert(edge.edge, into.reached.len());
                into.reached.push(edge);
            }
        }
    }
    into.crossings.extend(other.crossings);
    let reached = &into.reached;
    into.crossings.retain(|crossing| {
        let contour = &contours[crossing.contour];
        slot.get(&crossing.edge).is_none_or(|index| {
            let edge = &reached[*index];
            contour
                .metric
                .of(
                    (edge.cost, edge.cost),
                    (edge.duration_s, edge.duration_s),
                    (edge.distance_m, edge.distance_m),
                )
                .1
                > contour.limit
        })
    });
    into.truncated |= other.truncated;
    into
}

/// The request's contours as the search takes them.
fn analysis_contours(request: &RouteIsochroneRequest) -> Vec<AnalysisContour> {
    request
        .contours
        .iter()
        .map(|contour| AnalysisContour {
            limit: contour.contour,
            metric: match contour.metric {
                RouteIsochroneMetric::Cost => ContourMetric::Cost,
                RouteIsochroneMetric::TimeSeconds => ContourMetric::Duration,
                RouteIsochroneMetric::DistanceM => ContourMetric::Distance,
            },
        })
        .collect()
}

/// The candidates of every origin of an isochrone request, in order.
pub(crate) struct OriginCandidates {
    /// Every origin's candidates, origin by origin.
    pub(crate) candidates: Vec<CorrelatedLocation>,
    /// The origin of each candidate (an index into the request's origins,
    /// `origin` being 0).
    pub(crate) targets: Vec<usize>,
    /// Each origin's nearest candidate, as reported.
    pub(crate) snapped: Vec<CorrelatedLocation>,
}

/// How much farther than the nearest candidate a candidate may snap and
/// still count as on the nearest road (see [`origin_candidates`]).
const NEAREST_ROAD_TOLERANCE_M: f64 = 0.01;

/// Snaps every origin of `request` with `search`. `NoCandidate` for the
/// origin endpoint when any origin has no candidate. Ordered by time, only
/// each origin's nearest candidates are kept: time starts on the nearest
/// road, as an approach penalty is no travel time.
pub(crate) fn origin_candidates(
    request: &RouteIsochroneRequest,
    mut search: impl FnMut(GeoPoint) -> Vec<CorrelatedLocation>,
) -> Result<OriginCandidates, RouteError> {
    let mut origins = OriginCandidates {
        candidates: Vec::new(),
        targets: Vec::new(),
        snapped: Vec::new(),
    };
    let points = std::iter::once(request.origin).chain(request.origins.iter().map(|o| o.point));
    for (index, point) in points.enumerate() {
        let mut candidates = search(point);
        let nearest = candidates
            .first()
            .cloned()
            .ok_or(RouteError::NoCandidate { endpoint: "origin" })?;
        if request.order == RouteIsochroneOrder::Time {
            let snap = |candidate: &CorrelatedLocation| {
                if candidate.distance_m.is_finite() {
                    candidate.distance_m.max(0.0)
                } else {
                    0.0
                }
            };
            let least = candidates.iter().map(snap).fold(f64::INFINITY, f64::min);
            // Both directions of the nearest road snap at the same distance,
            // up to rounding.
            candidates.retain(|candidate| snap(candidate) <= least + NEAREST_ROAD_TOLERANCE_M);
        }
        origins.snapped.push(nearest);
        origins
            .targets
            .extend(std::iter::repeat_n(index, candidates.len()));
        origins.candidates.extend(candidates);
    }
    Ok(origins)
}

impl OriginCandidates {
    /// The candidates of the origins in `set` alone (origin indexes kept).
    pub(crate) fn only(&self, set: &[usize]) -> Self {
        let keep = |target: &usize| set.contains(target);
        Self {
            candidates: self
                .candidates
                .iter()
                .zip(&self.targets)
                .filter(|(_, target)| keep(target))
                .map(|(candidate, _)| candidate.clone())
                .collect(),
            targets: self.targets.iter().copied().filter(keep).collect(),
            snapped: self.snapped.clone(),
        }
    }

    /// Each origin's start offset in seconds, origin 0 first.
    pub(crate) fn offsets_s(request: &RouteIsochroneRequest) -> Vec<f64> {
        std::iter::once(0.0)
            .chain(
                request
                    .origins
                    .iter()
                    .map(|origin| origin.start_offset_s.unwrap_or(0.0)),
            )
            .collect()
    }
}

/// A bucket's edge ids, ascending, and the origin of each (the one with the
/// least value on the bucket's metric; on a tie, the lowest), from reached
/// edges as `(edge id, value, origin)`.
pub(crate) fn bucket_edges(
    reached: impl IntoIterator<Item = (u64, f64, u16)>,
    contour: &RouteIsochroneContour,
) -> (Vec<u64>, Vec<u16>) {
    let mut edges = BTreeMap::<u64, (f64, u16)>::new();
    for (edge_id, value, origin) in reached {
        if value > contour.contour {
            continue;
        }
        let entry = edges.entry(edge_id).or_insert((value, origin));
        if (value, origin) < *entry {
            *entry = (value, origin);
        }
    }
    edges
        .into_iter()
        .map(|(edge_id, (_, origin))| (edge_id, origin))
        .unzip()
}

/// The index of `metric` in `ReachedEdge::origins` (cost, time, distance).
fn metric_index(metric: RouteIsochroneMetric) -> usize {
    match metric {
        RouteIsochroneMetric::Cost => 0,
        RouteIsochroneMetric::TimeSeconds => 1,
        RouteIsochroneMetric::DistanceM => 2,
    }
}

/// The request against [`crate::limits`]: one to [`MAX_ISOCHRONE_ORIGINS`]
/// valid origins, one to
/// [`MAX_ISOCHRONE_CONTOURS`] contours, each positive, finite and not repeated
/// on its metric, polygon options in range, and the overlays and options a
/// route accepts.
fn validate_request(request: &RouteIsochroneRequest) -> Result<(), RouteError> {
    if 1 + request.origins.len() > MAX_ISOCHRONE_ORIGINS {
        return Err(RouteError::RequestTooLarge {
            what: "isochrone origins",
        });
    }
    if invalid_point(request.origin)
        || request
            .origins
            .iter()
            .any(|origin| invalid_point(origin.point))
    {
        return Err(RouteError::InvalidRequest {
            reason: "origin coordinates",
        });
    }
    let offsets = request
        .origins
        .iter()
        .filter_map(|origin| origin.start_offset_s);
    if offsets
        .clone()
        .any(|offset| !offset.is_finite() || offset < 0.0)
    {
        return Err(RouteError::InvalidRequest {
            reason: "isochrone start offsets",
        });
    }
    // Under cost order, an offset in seconds would not decide which origin
    // reaches an edge first.
    if request.order != RouteIsochroneOrder::Time && offsets.clone().any(|offset| offset > 0.0) {
        return Err(RouteError::InvalidRequest {
            reason: "isochrone start offsets need order time",
        });
    }
    if request.contours.len() > MAX_ISOCHRONE_CONTOURS {
        return Err(RouteError::RequestTooLarge {
            what: "isochrone contours",
        });
    }
    let invalid_contour = |(index, contour): (usize, &RouteIsochroneContour)| {
        !contour.contour.is_finite()
            || contour.contour <= 0.0
            // Buckets and boundary fragments are matched to contours this way.
            || request.contours[..index].iter().any(|earlier| {
                earlier.metric == contour.metric
                    && (earlier.contour - contour.contour).abs() < f64::EPSILON
            })
    };
    if request.contours.is_empty() || request.contours.iter().enumerate().any(invalid_contour) {
        return Err(RouteError::InvalidRequest {
            reason: "isochrone contours",
        });
    }
    if let Some(options) = &request.polygons {
        polygon::validate(options)?;
    }
    validate_query_overlays(&request.overlays)?;
    validate_query_options(&request.options)
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
    /// The isochrone of `origin_candidates` (see [`Self::isochrone_outcome`]).
    fn isochrone_search(
        &self,
        request: &RouteIsochroneRequest,
        origin_candidates: &OriginCandidates,
        allow_destination_only: bool,
    ) -> Result<RouteIsochroneResult, RouteError> {
        let outcome =
            self.isochrone_outcome(request, origin_candidates, allow_destination_only, None)?;
        Ok(self.isochrone_result(request, origin_candidates, outcome))
    }

    /// One search over the routing package from `origin_candidates`, with a
    /// dynamic overlay as a patch (see `package::search::patch`; endpoint
    /// connectors belong to routes and are ignored), stopping at `budget`
    /// visited edges too when given. `GraphNotReady` without a package and
    /// `MissingEdgeMetadata` for a candidate edge the package does not hold.
    fn isochrone_outcome(
        &self,
        request: &RouteIsochroneRequest,
        origin_candidates: &OriginCandidates,
        allow_destination_only: bool,
        budget: Option<usize>,
    ) -> Result<IsochroneOutcome, RouteError> {
        let budget = match (request.max_expansion_edge_count, budget) {
            (Some(request), Some(budget)) => Some(request.min(budget)),
            (request, budget) => request.or(budget),
        };
        let candidates = &origin_candidates.candidates;
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
                graph.profile_bit(request.profile),
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
        // Approach costs are per origin.
        let origins = AnalysisPoint::of_targets(candidates, &origin_candidates.targets, edges);
        let contours = analysis_contours(request);
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
            profile_bit: view.profile_bit(request.profile),
            max_visited_edge_count: budget,
            cancellation: &request.options.cancellation,
            cost_overrides,
            // A one-to-all expansion has no goal to direct it towards.
            gates: Vec::new(),
            restriction_penalty: crate::sar::restriction_penalty(&request.options),
            request_metric: None,
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
        let offsets_s = OriginCandidates::offsets_s(request);
        let seeding = IsochroneSeeding {
            by_duration: request.order == RouteIsochroneOrder::Time,
            origin_of: &origin_candidates.targets,
            offsets_s: &offsets_s,
        };
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
                    profile_bit: view.profile_bit(request.profile),
                    origins: candidates,
                    destinations: &[],
                    origin_edges: &origin_edges,
                    destination_edges: &[],
                    origin_targets: &origin_candidates.targets,
                    destination_targets: &[],
                    max_visited_edge_count: budget,
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
                    seeding,
                )
            }
            None => crate::package::search::analysis::isochrone(
                view, &mut state, &query, &origins, &contours, seeding,
            ),
        };
        graph.return_state(state);
        // A cancelled search stops as if out of budget.
        request.options.cancellation.check()?;
        if crate::debug::trace() {
            eprintln!(
                "package isochrone: search {:.1} ms",
                started.elapsed().as_secs_f64() * 1e3
            );
        }
        Ok(outcome)
    }

    /// The result of an isochrone search of `origin_candidates`.
    fn isochrone_result(
        &self,
        request: &RouteIsochroneRequest,
        origin_candidates: &OriginCandidates,
        outcome: IsochroneOutcome,
    ) -> RouteIsochroneResult {
        let started = std::time::Instant::now();
        let view = self.route_graph_view().routing_package().view();

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
                    origin: Some(crossing.origin),
                },
            });
        }
        if crate::debug::trace() {
            eprintln!(
                "package isochrone: geometry {:.1} ms ({} crossings)",
                started.elapsed().as_secs_f64() * 1e3,
                outcome.crossings.len(),
            );
        }

        let buckets = request
            .contours
            .iter()
            .map(|contour| {
                let index = metric_index(contour.metric);
                let (edge_ids, edge_origins) = bucket_edges(
                    outcome.reached.iter().map(|edge| {
                        let value = [edge.cost, edge.duration_s, edge.distance_m][index];
                        (view.edge_id(edge.edge as usize), value, edge.origins[index])
                    }),
                    contour,
                );
                RouteIsochroneBucket {
                    contour: contour.contour,
                    metric: contour.metric,
                    edge_ids,
                    edge_origins,
                    boundary_fragments: boundary_fragments_for_contour(
                        &boundary_fragments,
                        contour,
                    ),
                    polygon: None,
                }
            })
            .collect();
        RouteIsochroneResult {
            origin: origin_candidates.snapped[0].clone(),
            origins: origin_candidates.snapped[1..].to_vec(),
            buckets,
            visited_edge_count: outcome.counters.visited_edge_count,
            settled_node_count: outcome.counters.settled_edge_count,
            truncated: outcome.truncated,
            polygons: None,
        }
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

#[cfg(test)]
mod tests {
    use wayfarer_package::{DirectionalAccess, RoadType};
    use wayfarer_road_graph::{RoadGraph, RoadGraphBuilder, RoutableWay, enhance_road_graph};

    use crate::{
        GeoPoint, MobilityProfile, RouteError, RouteIsochroneBucket, RouteIsochroneContour,
        RouteIsochroneMetric, RouteIsochroneRequest, RouteRequest, SnapRequest,
    };

    fn street(id: i64, from: (f64, f64), to: (f64, f64), nodes: (i64, i64)) -> RoutableWay {
        RoutableWay {
            osm_way_id: id,
            points: vec![from, to],
            source_node_ids: vec![Some(nodes.0), Some(nodes.1)],
            road_type: RoadType::Residential,
            directional_access: DirectionalAccess::Both,
            highway: Some("residential".into()),
            ..Default::default()
        }
    }

    fn graph(ways: Vec<RoutableWay>) -> RoadGraph {
        let mut builder = RoadGraphBuilder::new("isochrone");
        for way in ways {
            builder.push_way(way);
        }
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);
        graph
    }

    /// An 8 × 8 grid of streets about 110 m apart, south of latitude 52.0.
    fn grid() -> RoadGraph {
        let node = |row: i64, col: i64| 1_000 + row * 10 + col;
        let at = |row: i64, col: i64| (52.0 - row as f64 * 0.001, 21.0 + col as f64 * 0.0016);
        let mut ways = Vec::new();
        for row in 0..8 {
            for col in 0..8 {
                if col + 1 < 8 {
                    let id = ways.len() as i64 + 1;
                    let nodes = (node(row, col), node(row, col + 1));
                    ways.push(street(id, at(row, col), at(row, col + 1), nodes));
                }
                if row + 1 < 8 {
                    let id = ways.len() as i64 + 1;
                    let nodes = (node(row, col), node(row + 1, col));
                    ways.push(street(id, at(row, col), at(row + 1, col), nodes));
                }
            }
        }
        graph(ways)
    }

    fn minutes(origin: GeoPoint, minutes: f64) -> RouteIsochroneRequest {
        RouteIsochroneRequest::new(
            origin,
            MobilityProfile::OnFoot,
            vec![RouteIsochroneContour {
                contour: minutes * 60.0,
                metric: RouteIsochroneMetric::TimeSeconds,
            }],
        )
    }

    #[test]
    fn an_origin_off_the_road_widens_the_snap_search_like_a_route() {
        let graph = grid();
        let engine = crate::test_support::engine(&graph);
        // About 110 m north of the grid's top street: beyond the default
        // 75 m radius, within the first widening.
        let origin = GeoPoint::new(52.001, 21.0056);
        engine
            .route(RouteRequest::new(
                origin,
                GeoPoint::new(51.995, 21.0056),
                MobilityProfile::OnFoot,
            ))
            .expect("a route widens its snap search");
        let isochrone = engine
            .isochrone(minutes(origin, 5.0))
            .expect("so does an isochrone");
        assert!(isochrone.origin.distance_m > 75.0);
        assert!(!isochrone.buckets[0].edge_ids.is_empty());

        let reference = crate::reference::ReferenceEngine::new(
            &graph,
            std::sync::Arc::new(
                crate::test_support::package(&graph, &crate::test_support::test_options())
                    .unwrap()
                    .0,
            ),
        );
        let expected = reference.isochrone(minutes(origin, 5.0)).unwrap();
        assert_eq!(expected.origin.edge_id, isochrone.origin.edge_id);
        assert_eq!(expected.buckets[0].edge_ids, isochrone.buckets[0].edge_ids);

        // Nothing within the widest radius is still no candidate.
        assert_eq!(
            engine
                .isochrone(minutes(GeoPoint::new(52.1, 21.0), 5.0))
                .unwrap_err(),
            RouteError::NoCandidate { endpoint: "origin" }
        );
    }

    #[test]
    fn time_contours_do_not_count_the_approach_penalty_as_seconds() {
        // Two separate streets, 10 m and 40 m south and north of the origin.
        let graph = graph(vec![
            street(1, (51.99991, 21.0), (51.99991, 21.004), (1, 2)),
            street(2, (52.00036, 21.0), (52.00036, 21.004), (3, 4)),
        ]);
        let engine = crate::test_support::engine(&graph);
        let mut request = minutes(GeoPoint::new(52.0, 21.002), 1.0);
        request.contours.push(RouteIsochroneContour {
            contour: 1.0e6,
            metric: RouteIsochroneMetric::Cost,
        });
        let mut on_far_street = SnapRequest::new(GeoPoint::new(52.00036, 21.002), request.profile);
        on_far_street.max_distance_m = Some(5);
        let far = engine
            .snap(on_far_street)
            .candidates
            .iter()
            .map(|candidate| candidate.edge_id)
            .collect::<Vec<_>>();
        assert!(!far.is_empty());
        let result = engine.isochrone(request.clone()).unwrap();
        assert!(!far.contains(&result.origin.edge_id));
        let starts = |metric: RouteIsochroneMetric| {
            result
                .buckets
                .iter()
                .filter(|bucket| bucket.metric == metric)
                .flat_map(|bucket| &bucket.boundary_fragments)
                .filter(|fragment| far.contains(&fragment.edge_id))
                .map(|fragment| fragment.start_metric)
                .collect::<Vec<_>>()
        };
        // The farther street's seed costs its approach, but its time starts
        // at zero, as a matrix row's does.
        let time = starts(RouteIsochroneMetric::TimeSeconds);
        assert!(!time.is_empty());
        assert!(time.iter().all(|start| *start == 0.0), "{time:?}");

        let reference = crate::reference::ReferenceEngine::new(
            &graph,
            std::sync::Arc::new(
                crate::test_support::package(&graph, &crate::test_support::test_options())
                    .unwrap()
                    .0,
            ),
        );
        // The oracle starts time at zero too.
        let expected = reference.isochrone(request).unwrap();
        for (expected, actual) in expected.buckets.iter().zip(&result.buckets) {
            assert_eq!(expected.edge_ids, actual.edge_ids);
            let starts = |bucket: &RouteIsochroneBucket| {
                bucket
                    .boundary_fragments
                    .iter()
                    .map(|fragment| (fragment.edge_id, fragment.start_metric))
                    .collect::<Vec<_>>()
            };
            assert_eq!(starts(expected), starts(actual));
        }
    }

    #[test]
    fn overlay_time_contours_do_not_count_the_approach_penalty_as_seconds() {
        // The streets of the test above, plus a third one 1 km north that the
        // origin never reaches. Blocking part of it sends the isochrone
        // through the overlay search without changing what it reaches.
        let graph = graph(vec![
            street(1, (51.99991, 21.0), (51.99991, 21.004), (1, 2)),
            street(2, (52.00036, 21.0), (52.00036, 21.004), (3, 4)),
            street(3, (52.01, 21.0), (52.01, 21.004), (5, 6)),
        ]);
        let engine = crate::test_support::engine(&graph);
        let snapped = |at: GeoPoint| {
            let mut snap = SnapRequest::new(at, MobilityProfile::OnFoot);
            snap.max_distance_m = Some(5);
            let edges = engine
                .snap(snap)
                .candidates
                .iter()
                .map(|candidate| candidate.edge_id)
                .collect::<Vec<_>>();
            assert!(!edges.is_empty());
            edges
        };
        let far = snapped(GeoPoint::new(52.00036, 21.002));
        let unreached = snapped(GeoPoint::new(52.01, 21.002));
        let overlay = crate::RouteDynamicOverlay::from_blocked_ranges(
            &unreached
                .iter()
                .map(|edge_id| crate::RouteEdgeRange {
                    edge_id: *edge_id,
                    start_fraction: 0.4,
                    end_fraction: 0.6,
                })
                .collect::<Vec<_>>(),
        );
        assert!(!overlay.split_edge_ranges.is_empty());

        let mut request = minutes(GeoPoint::new(52.0, 21.002), 1.0);
        request.contours.push(RouteIsochroneContour {
            contour: 1.0e6,
            metric: RouteIsochroneMetric::Cost,
        });
        let plain = engine.isochrone(request.clone()).unwrap();
        request.options.dynamic_overlay = Some(overlay);
        let result = engine.isochrone(request).unwrap();
        assert!(!far.contains(&result.origin.edge_id));

        // The farther street's seed costs its approach, but its time starts
        // at zero, as on the plain search.
        let time = result
            .buckets
            .iter()
            .filter(|bucket| bucket.metric == RouteIsochroneMetric::TimeSeconds)
            .flat_map(|bucket| &bucket.boundary_fragments)
            .filter(|fragment| far.contains(&fragment.edge_id))
            .map(|fragment| fragment.start_metric)
            .collect::<Vec<_>>();
        assert!(!time.is_empty());
        assert!(time.iter().all(|start| *start == 0.0), "{time:?}");

        // The overlay reaches exactly what the plain search does.
        assert_eq!(plain.buckets.len(), result.buckets.len());
        for (plain, overlay) in plain.buckets.iter().zip(&result.buckets) {
            assert_eq!(plain.edge_ids, overlay.edge_ids);
            assert_eq!(
                plain.boundary_fragments.len(),
                overlay.boundary_fragments.len()
            );
            for (plain, overlay) in plain
                .boundary_fragments
                .iter()
                .zip(&overlay.boundary_fragments)
            {
                assert_eq!(plain.edge_id, overlay.edge_id);
                for (plain, overlay) in [
                    (plain.start_fraction, overlay.start_fraction),
                    (plain.end_fraction, overlay.end_fraction),
                    (plain.start_metric, overlay.start_metric),
                    (plain.end_metric, overlay.end_metric),
                ] {
                    assert!((plain - overlay).abs() < 1e-6, "{plain} vs {overlay}");
                }
            }
        }
    }

    #[test]
    fn several_origins_are_validated_and_each_must_snap() {
        let graph = grid();
        let engine = crate::test_support::engine(&graph);
        let on_grid = GeoPoint::new(52.0, 21.0056);
        let mut request = minutes(on_grid, 2.0);
        request.origins = vec![
            crate::RouteIsochroneOrigin::new(GeoPoint::new(51.993, 21.0024));
            crate::limits::MAX_ISOCHRONE_ORIGINS - 1
        ];
        let result = engine.isochrone(request.clone()).unwrap();
        assert_eq!(
            result.origins.len(),
            crate::limits::MAX_ISOCHRONE_ORIGINS - 1
        );

        request
            .origins
            .push(crate::RouteIsochroneOrigin::new(on_grid));
        assert_eq!(
            engine.isochrone(request.clone()).unwrap_err(),
            RouteError::RequestTooLarge {
                what: "isochrone origins"
            }
        );
        request.origins = vec![crate::RouteIsochroneOrigin::new(GeoPoint::new(
            f64::NAN,
            21.0,
        ))];
        assert_eq!(
            engine.isochrone(request.clone()).unwrap_err(),
            RouteError::InvalidRequest {
                reason: "origin coordinates"
            }
        );
        // An origin with nothing within the widest radius fails the request,
        // as the first origin would.
        request.origins = vec![crate::RouteIsochroneOrigin::new(GeoPoint::new(52.1, 21.0))];
        assert_eq!(
            engine.isochrone(request).unwrap_err(),
            RouteError::NoCandidate { endpoint: "origin" }
        );
    }

    #[test]
    fn origins_are_optional_on_the_wire() {
        let request = minutes(GeoPoint::new(52.0, 21.0056), 2.0);
        let json = serde_json::to_value(&request).unwrap();
        assert!(json.get("origins").is_none());
        let mut with_more = request.clone();
        with_more.origins = vec![crate::RouteIsochroneOrigin::new(GeoPoint::new(
            51.993, 21.0024,
        ))];
        let json = serde_json::to_value(&with_more).unwrap();
        assert_eq!(json["origins"][0]["point"]["lat"], 51.993);
        assert_eq!(
            serde_json::from_value::<RouteIsochroneRequest>(json).unwrap(),
            with_more
        );

        let graph = grid();
        let engine = crate::test_support::engine(&graph);
        let result = engine.isochrone(request).unwrap();
        let json = serde_json::to_value(&result).unwrap();
        assert!(json.get("origins").is_none());
        let result = engine.isochrone(with_more).unwrap();
        let json = serde_json::to_value(&result).unwrap();
        assert_eq!(json["origins"].as_array().unwrap().len(), 1);
    }

    #[test]
    fn start_offsets_need_time_order_and_a_finite_non_negative_value() {
        let graph = grid();
        let engine = crate::test_support::engine(&graph);
        let mut request = minutes(GeoPoint::new(52.0, 21.0056), 2.0);
        let later = |offset: f64| crate::RouteIsochroneOrigin {
            point: GeoPoint::new(51.993, 21.0024),
            start_offset_s: Some(offset),
        };
        // A zero offset changes nothing, whatever the order.
        request.origins = vec![later(0.0)];
        engine.isochrone(request.clone()).unwrap();
        request.origins = vec![later(30.0)];
        assert_eq!(
            engine.isochrone(request.clone()).unwrap_err(),
            RouteError::InvalidRequest {
                reason: "isochrone start offsets need order time"
            }
        );
        request.order = crate::RouteIsochroneOrder::Time;
        let result = engine.isochrone(request.clone()).unwrap();
        assert!(!result.buckets[0].edge_ids.is_empty());
        for offset in [-1.0, f64::NAN, f64::INFINITY] {
            request.origins = vec![later(offset)];
            assert_eq!(
                engine.isochrone(request.clone()).unwrap_err(),
                RouteError::InvalidRequest {
                    reason: "isochrone start offsets"
                },
                "{offset}"
            );
        }
    }

    #[test]
    fn time_order_starts_on_the_nearest_road_only() {
        // The two streets of the approach tests, 10 m and 40 m from the
        // origin: by cost, the farther one is seeded too (at its approach
        // cost); by time, only the nearer one, so the farther one is
        // reached later, along the streets, or not at all.
        let graph = graph(vec![
            street(1, (51.99991, 21.0), (51.99991, 21.004), (1, 2)),
            street(2, (52.00036, 21.0), (52.00036, 21.004), (3, 4)),
        ]);
        let engine = crate::test_support::engine(&graph);
        let mut request = minutes(GeoPoint::new(52.0, 21.002), 1.0);
        let by_cost = engine.isochrone(request.clone()).unwrap();
        request.order = crate::RouteIsochroneOrder::Time;
        let by_time = engine.isochrone(request).unwrap();
        let edges = |result: &crate::RouteIsochroneResult| {
            result.buckets[0]
                .boundary_fragments
                .iter()
                .map(|fragment| fragment.edge_id)
                .collect::<std::collections::BTreeSet<_>>()
        };
        assert_eq!(
            edges(&by_cost),
            std::collections::BTreeSet::from([1, 2, 3, 4])
        );
        assert_eq!(edges(&by_time), std::collections::BTreeSet::from([1, 2]));
        assert_eq!(by_cost.origin, by_time.origin);
    }

    #[test]
    fn order_and_origin_attribution_are_optional_on_the_wire() {
        let request = minutes(GeoPoint::new(52.0, 21.0056), 2.0);
        let json = serde_json::to_value(&request).unwrap();
        for key in ["order", "includeOrigins"] {
            assert!(json.get(key).is_none(), "{key}");
        }
        let mut timed = request.clone();
        timed.order = crate::RouteIsochroneOrder::Time;
        timed.include_origins = true;
        timed.origins = vec![crate::RouteIsochroneOrigin {
            point: GeoPoint::new(51.993, 21.0024),
            start_offset_s: Some(30.0),
        }];
        let json = serde_json::to_value(&timed).unwrap();
        assert_eq!(json["order"], "time");
        assert_eq!(json["includeOrigins"], true);
        assert_eq!(json["origins"][0]["startOffsetS"], 30.0);
        assert_eq!(
            serde_json::from_value::<RouteIsochroneRequest>(json).unwrap(),
            timed
        );

        let graph = grid();
        let engine = crate::test_support::engine(&graph);
        let json = serde_json::to_value(engine.isochrone(request).unwrap()).unwrap();
        for bucket in json["buckets"].as_array().unwrap() {
            assert!(bucket.get("edgeOrigins").is_none());
            for fragment in bucket["boundaryFragments"].as_array().unwrap() {
                assert!(fragment.get("origin").is_none());
            }
        }
        let json = serde_json::to_value(engine.isochrone(timed).unwrap()).unwrap();
        let bucket = &json["buckets"][0];
        assert_eq!(
            bucket["edgeOrigins"].as_array().unwrap().len(),
            bucket["edgeIds"].as_array().unwrap().len()
        );
    }

    /// Whether `point` is inside the multipolygon (even-odd within each
    /// polygon, so holes are out).
    fn covers(polygon: &crate::GeoJsonMultiPolygon, point: GeoPoint) -> bool {
        polygon.coordinates.iter().any(|rings| {
            rings
                .iter()
                .filter(|ring| {
                    let mut inside = false;
                    for pair in ring.windows(2) {
                        let ([xi, yi], [xj, yj]) = (pair[0], pair[1]);
                        if (yi > point.lat) != (yj > point.lat)
                            && point.lng < (xj - xi) * (point.lat - yi) / (yj - yi) + xi
                        {
                            inside = !inside;
                        }
                    }
                    inside
                })
                .count()
                % 2
                == 1
        })
    }

    /// Twice the signed area of a closed ring, in square degrees.
    fn ring_area(ring: &[[f64; 2]]) -> f64 {
        ring.windows(2)
            .map(|pair| pair[0][0] * pair[1][1] - pair[1][0] * pair[0][1])
            .sum()
    }

    fn reference(graph: &RoadGraph) -> crate::reference::ReferenceEngine<'_> {
        crate::reference::ReferenceEngine::new(
            graph,
            std::sync::Arc::new(
                crate::test_support::package(graph, &crate::test_support::test_options())
                    .unwrap()
                    .0,
            ),
        )
    }

    fn with_polygons(mut request: RouteIsochroneRequest) -> RouteIsochroneRequest {
        request.polygons = Some(crate::RouteIsochronePolygonOptions::default());
        request
    }

    #[test]
    fn polygons_cover_the_reached_roads_nest_by_contour_and_match_the_reference() {
        let graph = grid();
        let engine = crate::test_support::engine(&graph);
        let mut request = with_polygons(minutes(GeoPoint::new(51.996, 21.0056), 1.0));
        for seconds in [120.0, 240.0] {
            request.contours.push(RouteIsochroneContour {
                contour: seconds,
                metric: RouteIsochroneMetric::TimeSeconds,
            });
        }
        request.contours.push(RouteIsochroneContour {
            contour: 200.0,
            metric: RouteIsochroneMetric::DistanceM,
        });
        let result = engine.isochrone(request.clone()).unwrap();
        // The grid's defaults: the reached area is under 2.56 km across, so
        // 10 m cells and 15 m of spread.
        assert_eq!(
            result.polygons,
            Some(crate::RouteIsochronePolygonOptions {
                resolution_m: Some(10.0),
                spread_m: Some(15.0),
                denoise: Some(0.0),
            })
        );
        let view = engine.route_graph_view().routing_package().view();
        let mut shape = Vec::new();
        for bucket in &result.buckets {
            let polygon = bucket.polygon.as_ref().expect("every bucket is drawn");
            assert!(!polygon.coordinates.is_empty());
            for rings in &polygon.coordinates {
                // RFC 7946: closed rings, the outer one counterclockwise and
                // the holes clockwise.
                for (index, ring) in rings.iter().enumerate() {
                    assert!(ring.len() >= 4);
                    assert_eq!(ring.first(), ring.last());
                    assert_eq!(ring_area(ring) > 0.0, index == 0);
                }
            }
            // Every point of every road the bucket reaches in full is inside,
            // and every fragment's start.
            for edge_id in &bucket.edge_ids {
                let edge =
                    crate::package::search::analysis::resolve_edge_id(view, *edge_id).unwrap();
                view.edge_shape(edge, &mut shape);
                let ends = (shape[0], shape[shape.len() - 1]);
                for step in 0..=10 {
                    let t = f64::from(step) / 10.0;
                    let point = GeoPoint::new(
                        ends.0.lat + (ends.1.lat - ends.0.lat) * t,
                        ends.0.lng + (ends.1.lng - ends.0.lng) * t,
                    );
                    assert!(covers(polygon, point), "{edge_id} at {t}");
                }
            }
            for fragment in &bucket.boundary_fragments {
                assert!(covers(polygon, fragment.start), "{fragment:?}");
            }
        }
        // The middle of the next block east, 55 m off every road, is out.
        let block = GeoPoint::new(51.9965, 21.0072);
        assert!(
            result
                .buckets
                .iter()
                .all(|bucket| !covers(bucket.polygon.as_ref().unwrap(), block))
        );
        // A wider time contour covers every point the narrower ones do.
        let probes = (0..=60)
            .flat_map(|row| {
                (0..=60).map(move |col| {
                    GeoPoint::new(
                        52.0 - f64::from(row) * 0.000125,
                        21.0 + f64::from(col) * 0.0002,
                    )
                })
            })
            .collect::<Vec<_>>();
        for pair in result.buckets[..3].windows(2) {
            let (narrow, wide) = (
                pair[0].polygon.as_ref().unwrap(),
                pair[1].polygon.as_ref().unwrap(),
            );
            let inside = probes
                .iter()
                .filter(|point| covers(narrow, **point))
                .count();
            assert!(inside > 0);
            for point in &probes {
                assert!(!covers(narrow, *point) || covers(wide, *point), "{point:?}");
            }
            assert!(probes.iter().filter(|point| covers(wide, **point)).count() > inside);
        }

        // The oracle's buckets draw the same polygons.
        let expected = reference(&graph).isochrone(request.clone()).unwrap();
        assert_eq!(expected.polygons, result.polygons);
        for (expected, actual) in expected.buckets.iter().zip(&result.buckets) {
            assert_eq!(expected.edge_ids, actual.edge_ids);
            assert_eq!(expected.polygon, actual.polygon);
        }
        // So does the same request again, and without edge ids.
        assert_eq!(engine.isochrone(request.clone()).unwrap(), result);
        request.include_edge_ids = false;
        let bare = engine.isochrone(request).unwrap();
        for (bare, full) in bare.buckets.iter().zip(&result.buckets) {
            assert!(bare.edge_ids.is_empty());
            assert_eq!(bare.polygon, full.polygon);
        }
    }

    #[test]
    fn unreached_blocks_are_holes_and_separate_origins_separate_polygons() {
        let graph = grid();
        let engine = crate::test_support::engine(&graph);
        // Half an hour on foot reaches the whole grid: one polygon, with a
        // hole in each of the 7 × 7 blocks (about 110 m square, 15 m of
        // spread on each side).
        let all = engine
            .isochrone(with_polygons(minutes(
                GeoPoint::new(51.9965, 21.0056),
                30.0,
            )))
            .unwrap();
        let polygon = all.buckets[0].polygon.as_ref().unwrap();
        assert_eq!(polygon.coordinates.len(), 1);
        assert_eq!(polygon.coordinates[0].len(), 1 + 49);
        // With more spread than half a block, the blocks fill in.
        let mut request = with_polygons(minutes(GeoPoint::new(51.9965, 21.0056), 30.0));
        request.polygons.as_mut().unwrap().spread_m = Some(80.0);
        let filled = engine.isochrone(request).unwrap();
        let polygon = filled.buckets[0].polygon.as_ref().unwrap();
        assert_eq!(polygon.coordinates.len(), 1);
        assert_eq!(polygon.coordinates[0].len(), 1);
        assert_eq!(filled.polygons.as_ref().unwrap().spread_m, Some(80.0));

        // Two origins in opposite corners, one minute each: two polygons,
        // each around its own origin.
        let corners = (GeoPoint::new(52.0, 21.0008), GeoPoint::new(51.993, 21.0104));
        let mut request = with_polygons(minutes(corners.0, 0.5));
        request.origins = vec![crate::RouteIsochroneOrigin::new(corners.1)];
        let both = engine.isochrone(request.clone()).unwrap();
        let polygon = both.buckets[0].polygon.as_ref().unwrap();
        assert_eq!(polygon.coordinates.len(), 2);
        assert!(covers(polygon, corners.0) && covers(polygon, corners.1));
        let parts = polygon
            .coordinates
            .iter()
            .map(|rings| crate::GeoJsonMultiPolygon {
                coordinates: vec![rings.clone()],
                ..Default::default()
            })
            .collect::<Vec<_>>();
        assert!(
            parts
                .iter()
                .all(|part| covers(part, corners.0) != covers(part, corners.1))
        );
        // Denoise keeps the larger one only (ties go to the westernmost).
        request.polygons.as_mut().unwrap().denoise = Some(1.0);
        let denoised = engine.isochrone(request).unwrap();
        assert_eq!(
            denoised.buckets[0]
                .polygon
                .as_ref()
                .unwrap()
                .coordinates
                .len(),
            1
        );
    }

    #[test]
    fn a_bucket_that_reaches_nothing_has_an_empty_polygon() {
        let graph = grid();
        let engine = crate::test_support::engine(&graph);
        // A one-second contour from mid-block reaches no road in full and
        // no fragment within it.
        let mut request = with_polygons(minutes(GeoPoint::new(51.9965, 21.0056), 30.0));
        request.contours = vec![RouteIsochroneContour {
            contour: 1.0e-3,
            metric: RouteIsochroneMetric::DistanceM,
        }];
        let result = engine.isochrone(request).unwrap();
        let polygon = result.buckets[0].polygon.as_ref().unwrap();
        assert!(polygon.coordinates.is_empty(), "{polygon:?}");
        assert_eq!(
            serde_json::to_value(polygon).unwrap(),
            serde_json::json!({"type": "MultiPolygon", "coordinates": []})
        );
    }

    #[test]
    fn polygons_are_optional_on_the_wire_and_validated() {
        let graph = grid();
        let engine = crate::test_support::engine(&graph);
        let request = minutes(GeoPoint::new(51.9965, 21.0056), 2.0);
        assert!(
            serde_json::to_value(&request)
                .unwrap()
                .get("polygons")
                .is_none()
        );
        let json = serde_json::to_value(engine.isochrone(request.clone()).unwrap()).unwrap();
        assert!(json.get("polygons").is_none());
        assert!(json["buckets"][0].get("polygon").is_none());

        let mut json = serde_json::to_value(&request).unwrap();
        json["polygons"] = serde_json::json!({"resolutionM": 25.0, "denoise": 0.1});
        let drawn: RouteIsochroneRequest = serde_json::from_value(json.clone()).unwrap();
        assert_eq!(
            drawn.polygons,
            Some(crate::RouteIsochronePolygonOptions {
                resolution_m: Some(25.0),
                spread_m: None,
                denoise: Some(0.1),
            })
        );
        let result = serde_json::to_value(engine.isochrone(drawn).unwrap()).unwrap();
        assert_eq!(result["polygons"]["resolutionM"], 25.0);
        assert_eq!(result["polygons"]["spreadM"], 37.5);
        assert_eq!(result["buckets"][0]["polygon"]["type"], "MultiPolygon");
        // A misspelt option is refused, not ignored.
        json["polygons"] = serde_json::json!({"resolution": 25.0});
        assert!(serde_json::from_value::<RouteIsochroneRequest>(json).is_err());

        let mut bad = with_polygons(request);
        bad.polygons.as_mut().unwrap().resolution_m = Some(0.0);
        assert_eq!(
            engine.isochrone(bad).unwrap_err(),
            RouteError::InvalidRequest {
                reason: "isochrone polygon options"
            }
        );
    }
}

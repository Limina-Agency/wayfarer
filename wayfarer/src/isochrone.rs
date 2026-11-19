use std::collections::BTreeSet;

use crate::correlator::Correlator;
use crate::costing::CostingPolicy;
use crate::limits::MAX_ISOCHRONE_CONTOURS;
use crate::package::destination::relax_split_costs;
use crate::package::profile_bit;
use crate::package::search::analysis::{
    AnalysisContour, AnalysisPoint, AnalysisQuery, overlay_split_costs, request_cost_overrides,
    request_duration_overrides, resolve_edge_id, resolve_location,
};
use crate::package::search::patch::{PatchCosts, PatchGraph, PatchQuery, analysis_overlay};
use crate::search::RoutingEngine;
use crate::trip::{invalid_point, validate_query_options};
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
        request.options.cancellation.check()?;
        let policy = CorrelationPolicy::from(&request.options);
        let correlator = Correlator::new(self.route_graph_view());
        // An origin with no connected road within the radius widens the
        // search as route and matrix locations do (and as far as the SAR
        // `off_network` relaxation reaches).
        let origin_search = correlator.search_expanding(
            request.origin,
            request.profile,
            &policy,
            &Default::default(),
        );
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

/// The request against [`crate::limits`]: a valid origin, one to
/// [`MAX_ISOCHRONE_CONTOURS`] contours, each positive, finite and not repeated
/// on its metric, and the options a route accepts.
fn validate_request(request: &RouteIsochroneRequest) -> Result<(), RouteError> {
    if invalid_point(request.origin) {
        return Err(RouteError::InvalidRequest {
            reason: "origin coordinates",
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
}

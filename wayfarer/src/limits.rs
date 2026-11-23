//! Bounds on a request's size. Requests beyond them fail with
//! `RouteError::RequestTooLarge` (or `InvalidRequest`), so a host can pass
//! untrusted input straight through.

pub use crate::trip::{
    MAX_ROUTE_LOCATION_HINTS, MAX_ROUTE_LOCATIONS, MAX_ROUTE_OUTPUT_EDGE_IDS,
    MAX_ROUTE_OUTPUT_GEOMETRY_POINTS, MAX_ROUTE_OUTPUT_MANEUVERS, MAX_ROUTE_OUTPUT_RECOSTS,
    MAX_ROUTE_OVERLAY_POINTS, MAX_ROUTE_OVERLAY_SPEED_KPH, MAX_ROUTE_OVERLAYS,
    MAX_ROUTE_PROJECTION_DISTANCE_M, MAX_ROUTE_PROVENANCE_VALUES, MAX_ROUTE_STRING_BYTES,
    MAX_ROUTE_TOTAL_STRING_BYTES,
};

/// The most correlation candidates a request may ask for.
pub const MAX_REQUEST_CANDIDATES: usize = 64;
/// The most edge ids and ranges (blocks, avoidances, multipliers, …) one
/// request may carry.
pub const MAX_REQUEST_EDGE_CONSTRAINTS: usize = 100_000;
/// The most dynamic-overlay connectors one request may carry.
pub const MAX_REQUEST_DYNAMIC_CONNECTORS: usize = 10_000;
/// The most dynamic-overlay connector geometry points one request may carry.
pub const MAX_REQUEST_DYNAMIC_GEOMETRY_POINTS: usize = 100_000;
/// The farthest a request may look for the road network around a point: the
/// correlation radius of route, matrix and isochrone locations
/// (`RouteOptions::max_correlation_distance_m`), a snap's `max_distance_m`
/// and a map match's radius, including the one widened by a sample's
/// reported accuracy. It matches the SAR off-network maximum
/// ([`crate::sar::SAR_MAX_OFF_NETWORK_LEG_M`]). Projections of overlay geometry are bounded
/// by [`MAX_ROUTE_PROJECTION_DISTANCE_M`] instead.
pub const MAX_CORRELATION_DISTANCE_M: u32 = 10_000;
/// The most cells (sources × targets) one matrix request may ask for.
pub const MAX_MATRIX_CELLS: usize = 250_000;
/// The most contours one isochrone request may ask for.
pub const MAX_ISOCHRONE_CONTOURS: usize = 16;
/// The most origins one isochrone request may carry (`origin` and
/// `origins` together).
pub const MAX_ISOCHRONE_ORIGINS: usize = 64;
/// The most targets one nearest-targets request may carry.
pub const MAX_NEAREST_TARGETS: usize = 10_000;

#[cfg(test)]
mod tests {
    use wayfarer_package::{DirectionalAccess, RoadType};
    use wayfarer_road_graph::{RoadGraph, RoadGraphBuilder, RoutableWay, enhance_road_graph};

    use super::*;
    use crate::{
        GeoPoint, MapMatchRequest, MobilityProfile, NavigationPositionSample, ProjectRequest,
        RouteError, RouteIsochroneContour, RouteIsochroneMetric, RouteIsochroneRequest,
        RouteMatrixCellStatus, RouteMatrixRequest, RouteOverlayGeometryKind, RouteRequest,
        RoutingEngine, SnapRequest,
    };

    /// Two residential streets in a line, east from (52.0, 21.0).
    fn street() -> RoadGraph {
        let mut builder = RoadGraphBuilder::new("limits");
        for (id, from, to) in [(1, 0.0, 0.001), (2, 0.001, 0.002)] {
            builder.push_way(RoutableWay {
                osm_way_id: id,
                points: vec![(52.0, 21.0 + from), (52.0, 21.0 + to)],
                source_node_ids: vec![Some(100 + id), Some(101 + id)],
                road_type: RoadType::Residential,
                directional_access: DirectionalAccess::Both,
                highway: Some("residential".into()),
                ..Default::default()
            });
        }
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);
        graph
    }

    fn engine() -> RoutingEngine {
        crate::test_support::engine(&street())
    }

    fn point(lng: f64) -> GeoPoint {
        GeoPoint::new(52.00002, lng)
    }

    fn contour(contour: f64, metric: RouteIsochroneMetric) -> RouteIsochroneContour {
        RouteIsochroneContour { contour, metric }
    }

    fn isochrone(contours: Vec<RouteIsochroneContour>) -> RouteIsochroneRequest {
        RouteIsochroneRequest::new(point(21.0005), MobilityProfile::Wheeled, contours)
    }

    fn sample(lat: f64, lng: f64, accuracy_meters: Option<f64>) -> NavigationPositionSample {
        NavigationPositionSample {
            position: GeoPoint::new(lat, lng),
            heading_degrees: None,
            speed_meters_per_second: None,
            sampled_at_ms: None,
            accuracy_meters,
            provider: None,
        }
    }

    /// A change that makes a matrix request invalid.
    type Edit = fn(&mut RouteMatrixRequest);

    #[test]
    fn matrix_requests_run_the_route_option_checks() {
        let engine = engine();
        let request = || {
            RouteMatrixRequest::new(
                vec![point(21.0002)],
                vec![point(21.0018), point(21.0012)],
                MobilityProfile::Wheeled,
            )
        };
        let valid = engine.matrix(request());
        assert!(
            valid
                .cells
                .iter()
                .all(|cell| cell.status == RouteMatrixCellStatus::Reachable)
        );

        let cases: [(Edit, &str); 5] = [
            (
                |request| request.options.candidate_limit = MAX_REQUEST_CANDIDATES + 1,
                "request_too_large:candidate limit",
            ),
            (
                |request| {
                    request.options.max_correlation_distance_m = MAX_CORRELATION_DISTANCE_M + 1
                },
                "request_too_large:correlation distance",
            ),
            (
                |request| request.options.costing.avoid_tolls = f64::NAN,
                "invalid_request:costing options",
            ),
            (
                |request| request.options.alternatives.requested_alternative_count = 4,
                "request_too_large:alternatives",
            ),
            (
                |request| request.targets[1].lat = f64::NAN,
                "invalid_request:location coordinates",
            ),
        ];
        for (edit, reason) in cases {
            let mut request = request();
            edit(&mut request);
            assert!(request.validate().is_err(), "{reason}");
            let result = engine.matrix(request);
            assert_eq!(result.cells.len(), 2, "{reason}");
            for cell in &result.cells {
                assert_eq!(cell.status, RouteMatrixCellStatus::Failed, "{reason}");
                assert_eq!(cell.failure_reason.as_deref(), Some(reason));
            }
        }
    }

    #[test]
    fn matrices_are_bounded_by_cell_count() {
        let side = |count: usize| vec![point(21.0005); count];
        let largest = RouteMatrixRequest::new(side(500), side(500), MobilityProfile::Wheeled);
        assert_eq!(
            largest.sources.len() * largest.targets.len(),
            MAX_MATRIX_CELLS
        );
        assert!(largest.validate().is_ok());

        let request = RouteMatrixRequest::new(side(501), side(500), MobilityProfile::Wheeled);
        assert_eq!(
            request.validate(),
            Err(RouteError::RequestTooLarge {
                what: "matrix cells"
            })
        );
        // No grid of failed cells is built for it.
        let result = engine().matrix(request);
        assert_eq!((result.source_count, result.target_count), (501, 500));
        assert!(result.cells.is_empty());
        assert_eq!(result.source_expansion_count, 0);
    }

    #[test]
    fn isochrone_contours_are_checked() {
        let engine = engine();
        let cost = |value| contour(value, RouteIsochroneMetric::Cost);
        let time = |value| contour(value, RouteIsochroneMetric::TimeSeconds);
        let valid = engine
            .isochrone(isochrone(vec![cost(60.0), time(60.0), time(30.0)]))
            .expect("distinct positive contours");
        assert_eq!(valid.buckets.len(), 3);

        let invalid = RouteError::InvalidRequest {
            reason: "isochrone contours",
        };
        for contours in [
            vec![],
            vec![cost(0.0)],
            vec![time(-5.0)],
            vec![cost(f64::NAN)],
            vec![time(f64::INFINITY)],
            vec![cost(60.0), time(30.0), cost(60.0)],
        ] {
            assert_eq!(
                engine.isochrone(isochrone(contours.clone())).unwrap_err(),
                invalid,
                "{contours:?}"
            );
        }
        let many = (1..=MAX_ISOCHRONE_CONTOURS + 1)
            .map(|index| time(index as f64 * 10.0))
            .collect::<Vec<_>>();
        assert!(engine.isochrone(isochrone(many[1..].to_vec())).is_ok());
        assert_eq!(
            engine.isochrone(isochrone(many)).unwrap_err(),
            RouteError::RequestTooLarge {
                what: "isochrone contours"
            }
        );
    }

    #[test]
    fn isochrone_requests_run_the_route_option_checks() {
        let engine = engine();
        let mut request = isochrone(vec![contour(60.0, RouteIsochroneMetric::TimeSeconds)]);
        request.options.max_correlation_distance_m = MAX_CORRELATION_DISTANCE_M + 1;
        assert_eq!(
            engine.isochrone(request.clone()).unwrap_err(),
            RouteError::RequestTooLarge {
                what: "correlation distance"
            }
        );
        request.options.max_correlation_distance_m = 75;
        request.options.edge_cost_multipliers.insert(1, f64::NAN);
        assert_eq!(
            engine.isochrone(request.clone()).unwrap_err(),
            RouteError::InvalidRequest {
                reason: "edge adjustment"
            }
        );
        request.options.edge_cost_multipliers.clear();
        request.origin.lng = 181.0;
        assert_eq!(
            engine.isochrone(request).unwrap_err(),
            RouteError::InvalidRequest {
                reason: "origin coordinates"
            }
        );
    }

    #[test]
    fn routes_bound_the_correlation_distance() {
        let engine = engine();
        let mut request =
            RouteRequest::new(point(21.0002), point(21.0018), MobilityProfile::Wheeled);
        request.options.max_correlation_distance_m = MAX_CORRELATION_DISTANCE_M;
        assert!(engine.route(request.clone()).is_ok());
        request.options.max_correlation_distance_m = MAX_CORRELATION_DISTANCE_M + 1;
        assert_eq!(
            engine.route(request).unwrap_err().error,
            RouteError::RequestTooLarge {
                what: "correlation distance"
            }
        );
    }

    #[test]
    fn snaps_are_bounded_and_clamped() {
        let engine = engine();
        let mut request = SnapRequest::new(point(21.0005), MobilityProfile::Wheeled);
        request.max_distance_m = Some(MAX_CORRELATION_DISTANCE_M);
        request.candidate_limit = Some(MAX_REQUEST_CANDIDATES);
        assert!(request.validate().is_ok());
        let bounded = engine.snap(request.clone());
        assert!(!bounded.candidates.is_empty());

        let mut too_far = request.clone();
        too_far.max_distance_m = Some(u32::MAX);
        assert_eq!(
            too_far.validate(),
            Err(RouteError::RequestTooLarge {
                what: "correlation distance"
            })
        );
        let mut too_many = request.clone();
        too_many.candidate_limit = Some(usize::MAX);
        assert_eq!(
            too_many.validate(),
            Err(RouteError::RequestTooLarge {
                what: "candidate limit"
            })
        );
        // The engine clamps to the same bounds rather than failing.
        too_far.candidate_limit = Some(usize::MAX);
        assert_eq!(engine.snap(too_far), bounded);

        request.point.lat = f64::NAN;
        assert_eq!(
            request.validate(),
            Err(RouteError::InvalidRequest {
                reason: "point coordinates"
            })
        );
    }

    #[test]
    fn projections_are_bounded_like_overlays() {
        let engine = engine();
        let mut request = ProjectRequest {
            geometry_kind: RouteOverlayGeometryKind::Point,
            points: vec![point(21.0005)],
            projection_distance_m: MAX_ROUTE_PROJECTION_DISTANCE_M,
        };
        assert!(request.validate().is_ok());
        let bounded = engine.project(&request);
        assert!(!bounded.edge_ids.is_empty());

        request.projection_distance_m = u32::MAX;
        assert_eq!(
            request.validate(),
            Err(RouteError::RequestTooLarge {
                what: "projection distance"
            })
        );
        assert_eq!(engine.project(&request), bounded);

        request.projection_distance_m = 50;
        request.points.push(GeoPoint::new(91.0, 21.0));
        assert_eq!(
            request.validate(),
            Err(RouteError::InvalidRequest {
                reason: "projection geometry"
            })
        );
        request.points = vec![point(21.0005); MAX_ROUTE_OVERLAY_POINTS + 1];
        assert_eq!(
            request.validate(),
            Err(RouteError::RequestTooLarge {
                what: "projection geometry"
            })
        );
    }

    #[test]
    fn map_matching_bounds_its_radius() {
        let engine = engine();
        let trace = || {
            MapMatchRequest::new(
                vec![
                    sample(52.00002, 21.0002, None),
                    sample(52.00002, 21.0008, None),
                    // Far from the street, with an absurd reported accuracy.
                    sample(52.5, 21.0008, Some(1.0e9)),
                ],
                MobilityProfile::Wheeled,
            )
        };
        let result = engine.map_match(trace()).expect("two samples match");
        assert_eq!(result.rejected_samples.len(), 1);
        assert_eq!(
            result.rejected_samples[0].max_correlation_distance_meters,
            MAX_CORRELATION_DISTANCE_M
        );

        let mut request = trace();
        request.options.max_correlation_distance_m = MAX_CORRELATION_DISTANCE_M + 1;
        assert_eq!(
            engine.map_match(request).unwrap_err(),
            RouteError::RequestTooLarge {
                what: "correlation distance"
            }
        );
        let mut request = trace();
        request.options.candidate_limit = MAX_REQUEST_CANDIDATES + 1;
        assert_eq!(
            engine.map_match(request).unwrap_err(),
            RouteError::RequestTooLarge {
                what: "candidate limit"
            }
        );
    }
}

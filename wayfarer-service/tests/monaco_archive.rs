//! Opt-in smoke test against an already compiled Monaco archive.
use wayfarer::{GeoPoint, LegRequest, MobilityProfile, RouteOptions};
use wayfarer::{RouteLegMode, RouteLocation};
use wayfarer_service::{RouteWorkflowRequest, RoutingService, RoutingSource, RoutingSourceId};

#[test]
#[ignore = "requires WAYFARER_MONACO_ROUTE_ARCHIVE pointing to compiled Monaco routing tiles"]
fn compiled_monaco_routes_and_reopens() {
    let path = std::env::var_os("WAYFARER_MONACO_ROUTE_ARCHIVE")
        .expect("set WAYFARER_MONACO_ROUTE_ARCHIVE");
    let source = RoutingSource::new(
        RoutingSourceId::new("monaco-smoke").unwrap(),
        "compiled",
        path,
    )
    .unwrap();
    let service = RoutingService::default();
    let readiness = service.inspect_source(&source);
    println!("archive: {}", serde_json::to_string(&readiness).unwrap());
    assert!(readiness.is_ready(), "{readiness:?}");
    assert!(!readiness.cache_hit);
    assert!(readiness.node_count > 0 && readiness.directed_edge_count > 0);
    let points = [
        GeoPoint::new(43.7384, 7.4246),
        GeoPoint::new(43.7391, 7.4274),
        GeoPoint::new(43.7402, 7.4298),
    ];
    for profile in [MobilityProfile::OnFoot, MobilityProfile::Wheeled] {
        for (origin, destination) in [(points[0], points[2]), (points[2], points[0])] {
            let mut request = LegRequest::new(origin, destination, profile);
            request.options.max_correlation_distance_m = 150;
            let first = service
                .route_leg(&source, request.clone())
                .expect("Monaco route");
            let route = &first.route;
            assert!(first.cache_hit);
            assert!(route.distance_m.is_finite() && route.distance_m > 0.0);
            assert!(route.duration_s.is_finite() && route.duration_s > 0.0);
            assert!(!route.edge_ids.is_empty() && route.geometry.len() >= 2);
            assert!(!route.maneuvers.is_empty());
            let repeated = service.route_leg(&source, request).expect("cached route");
            assert_eq!(route.edge_ids, repeated.route.edge_ids);
            assert_eq!(route.distance_m, repeated.route.distance_m);
            println!(
                "{profile:?} {origin:?} -> {destination:?}: {:.1} m, {:.1} s, {} edges, {} maneuvers",
                route.distance_m,
                route.duration_s,
                route.edge_ids.len(),
                route.maneuvers.len()
            );
        }
        let workflow = service
            .route(
                &source,
                RouteWorkflowRequest {
                    route: wayfarer::RouteRequest {
                        locations: points.iter().copied().map(RouteLocation::new).collect(),
                        legs: vec![RouteLegMode::Routed; 2],
                        profile,
                        options: RouteOptions {
                            max_correlation_distance_m: 150,
                            ..Default::default()
                        },
                        overlays: vec![],
                        departure_at_ms: None,
                        leg_dynamic_overlays: Default::default(),
                        geometry_format: Default::default(),
                    },
                    overlay_provenance: Default::default(),
                    mobility: None,
                    overlay_sets: Vec::new(),
                    include: None,
                },
            )
            .expect("Monaco waypoint workflow");
        assert_eq!(workflow.route.segments.len(), 2);
        assert!(workflow.route.distance_m > 0.0 && workflow.route.geometry.len() >= 3);
        assert!(!workflow.route.edge_ids.is_empty());
        println!(
            "{profile:?} waypoint workflow: {:.1} m, {} segments",
            workflow.route.distance_m,
            workflow.route.segments.len()
        );
    }
    service.invalidate(source.id());
    let reopened = service.inspect_source(&source);
    assert!(reopened.is_ready() && !reopened.cache_hit);
    assert_eq!(readiness.archive_hash, reopened.archive_hash);
}

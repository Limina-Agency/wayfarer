//! A route request's `include` list: absent, the result is as before; with
//! one, only the core and the listed parts, and no `costBreakdown` in the
//! JSON unless listed.

use std::collections::BTreeSet;

use serde_json::Value;
use wayfarer::{GeoPoint, MobilityProfile, RouteAlternatives, RouteRequest, RoutingEngine};
use wayfarer_road_graph::{
    DirectionalAccess, RoadGraph, RoadGraphBuilder, RoadType, RoutableWay, enhance_road_graph,
};
use wayfarer_service::geojson::route_feature_collection;
use wayfarer_service::{
    RouteResultPart, RouteWorkflowRequest, RouteWorkflowResult, RouteWorkflowSourceContext,
    RoutingSourceId, compile_route_workflow,
};

fn way(id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
    RoutableWay {
        osm_way_id: id,
        points,
        road_type: RoadType::Primary,
        directional_access: DirectionalAccess::Both,
        highway: Some("primary".into()),
        ..Default::default()
    }
}

/// A square of four roads, so a route has a second way round.
fn square() -> RoadGraph {
    let mut builder = RoadGraphBuilder::new("square");
    builder.push_way(way(1, vec![(52.0, 21.0), (52.0, 21.002)]));
    builder.push_way(way(2, vec![(52.0, 21.002), (52.002, 21.002)]));
    builder.push_way(way(3, vec![(52.0, 21.0), (52.002, 21.0)]));
    builder.push_way(way(4, vec![(52.002, 21.0), (52.002, 21.002)]));
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    graph
}

fn engine() -> RoutingEngine {
    let (bytes, _) = wayfarer_compiler::package_build::package_bytes(
        &square(),
        &wayfarer_compiler::package_build::PackageBuildOptions::default(),
    )
    .unwrap();
    RoutingEngine::from_package(std::sync::Arc::new(
        wayfarer::RoutingPackage::from_bytes(&bytes).unwrap(),
    ))
}

fn run(engine: &RoutingEngine, request: RouteWorkflowRequest) -> RouteWorkflowResult {
    let source = RouteWorkflowSourceContext {
        source_id: RoutingSourceId::new("square").unwrap(),
        revision: "v1".to_string(),
        cache_hit: false,
    };
    compile_route_workflow(engine, source, request).expect("the workflow routes")
}

/// Two legs, from the south side round to the north side, with
/// alternatives asked for (a multi-leg route has none).
fn request(include: Option<&[RouteResultPart]>) -> RouteWorkflowRequest {
    let mut route = RouteRequest::through(
        [
            GeoPoint::new(52.0, 21.0005),
            GeoPoint::new(52.001, 21.002),
            GeoPoint::new(52.002, 21.0015),
        ],
        MobilityProfile::Wheeled,
    );
    route.options.alternatives.requested_alternative_count = 1;
    route.options.alternatives.max_shared_edge_ratio = 1.0;
    route.options.alternatives.max_cost_ratio = 10.0;
    let mut request = RouteWorkflowRequest::from(route);
    request.include = include.map(|parts| parts.iter().copied().collect());
    request
}

/// Every key path in `value` (`a.b`, `a[].c`), arrays merged.
fn key_paths(value: &Value, prefix: &str, out: &mut BTreeSet<String>) {
    match value {
        Value::Object(map) => {
            for (key, child) in map {
                let path = if prefix.is_empty() {
                    key.clone()
                } else {
                    format!("{prefix}.{key}")
                };
                out.insert(path.clone());
                key_paths(child, &path, out);
            }
        }
        Value::Array(items) => {
            for item in items {
                key_paths(item, &format!("{prefix}[]"), out);
            }
        }
        _ => {}
    }
}

fn paths(value: &Value) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    key_paths(value, "", &mut out);
    out
}

#[test]
fn without_include_the_result_is_complete() {
    let engine = engine();
    let full = run(&engine, request(None));
    let json = serde_json::to_value(&full).unwrap();
    assert!(json.get("include").is_none());
    assert!(json.get("segmentSummaries").is_none());
    assert_eq!(full.route.segments.len(), 2);
    assert!(!full.route.edge_attributions.is_empty());
    assert!(paths(&json).contains("edgeAttributions[].costBreakdown"));
    assert!(paths(&json).contains("segments[].routed.legs[].edges[].costBreakdown"));

    // Every part listed is the same result.
    let every = [
        RouteResultPart::Segments,
        RouteResultPart::Legs,
        RouteResultPart::EdgeAttributions,
        RouteResultPart::CostBreakdown,
        RouteResultPart::ShapeSegments,
        RouteResultPart::Recosts,
    ];
    let listed = run(&engine, request(Some(&every)));
    assert_eq!(listed.route.segments.len(), full.route.segments.len());
    assert_eq!(listed.route.edge_attributions, full.route.edge_attributions);
    assert_eq!(listed.route.shape_segments, full.route.shape_segments);
    assert_eq!(listed.route.recosts, full.route.recosts);
    assert_eq!(listed.segment_summaries, None);
    // Its JSON has the same keys, and the list.
    let mut keys = paths(&serde_json::to_value(&listed).unwrap());
    assert!(keys.remove("include"));
    assert_eq!(keys, paths(&json));
}

#[test]
fn an_empty_include_keeps_the_core() {
    let engine = engine();
    let full = run(&engine, request(None));
    let core = run(&engine, request(Some(&[])));
    let route = &core.route;
    assert!(route.segments.is_empty());
    assert!(route.edge_attributions.is_empty());
    assert!(route.shape_segments.is_empty());
    assert!(route.recosts.is_empty());
    // The core is unchanged.
    assert_eq!(route.geometry, full.route.geometry);
    assert_eq!(route.maneuvers, full.route.maneuvers);
    assert_eq!(route.edge_ids, full.route.edge_ids);
    assert_eq!(route.location_matches, full.route.location_matches);
    assert_eq!(route.cost_summary, full.route.cost_summary);
    assert_eq!(route.distance_m, full.route.distance_m);
    // Each leg in brief instead of its segment.
    let summaries = core.segment_summaries.as_ref().unwrap();
    assert_eq!(summaries.len(), 2);
    for (summary, segment) in summaries.iter().zip(&full.route.segments) {
        assert_eq!(summary.distance_m, segment.distance_m());
        assert_eq!(summary.duration_s, segment.duration_s());
        assert_eq!(summary.cost, segment.cost());
    }
    let json = serde_json::to_value(&core).unwrap();
    assert_eq!(json["include"], serde_json::json!([]));
    assert_eq!(json["segmentSummaries"][0]["mode"], "routed");
    let full_json = serde_json::to_string(&full).unwrap();
    let core_json = serde_json::to_string(&core).unwrap();
    assert!(
        core_json.len() * 3 < full_json.len(),
        "{} vs {}",
        core_json.len(),
        full_json.len()
    );
    // GeoJSON still has the line and the legs.
    let collection = route_feature_collection(&core).unwrap();
    assert_eq!(collection["features"][0]["properties"]["legs"], 2);
}

#[test]
fn edges_without_cost_breakdowns() {
    let engine = engine();
    let lean = run(
        &engine,
        request(Some(&[
            RouteResultPart::Segments,
            RouteResultPart::Legs,
            RouteResultPart::EdgeAttributions,
        ])),
    );
    let full = run(&engine, request(None));
    // The Rust result keeps the breakdowns; the JSON leaves them out.
    assert_eq!(lean.route.edge_attributions, full.route.edge_attributions);
    let json = serde_json::to_value(&lean).unwrap();
    let keys = paths(&json);
    assert!(keys.contains("edgeAttributions[].edgeId"));
    assert!(keys.contains("segments[].routed.legs[].edges[].edgeId"));
    assert!(
        !keys.iter().any(|key| key.contains("costBreakdown")),
        "{keys:?}"
    );
    assert!(
        !keys.iter().any(|key| key.contains("shapeSegments[]")),
        "{keys:?}"
    );
    assert_eq!(
        json["segments"][0]["routed"]["recosts"],
        serde_json::json!([])
    );
    // Nothing else differs from the full JSON.
    let mut full_keys = paths(&serde_json::to_value(&full).unwrap());
    full_keys.retain(|key| !key.contains("costBreakdown") && !key.contains("shapeSegments[]"));
    full_keys.retain(|key| !key.starts_with("recosts[]") && !key.contains("routed.recosts[]"));
    let mut lean_keys = keys;
    lean_keys.remove("include");
    assert_eq!(lean_keys, full_keys);
}

#[test]
fn alternatives_lose_their_legs_unless_listed() {
    let engine = engine();
    let single = |include: Option<&[RouteResultPart]>| {
        let mut request = request(include);
        request.route.locations.remove(1);
        run(&engine, request)
    };
    let full = single(None);
    let lean = single(Some(&[RouteResultPart::Segments]));
    let (RouteAlternatives::Computed(full), RouteAlternatives::Computed(lean)) =
        (&full.route.alternatives, &lean.route.alternatives)
    else {
        panic!("a single leg computes alternatives");
    };
    assert!(!full.is_empty(), "the square has a second way round");
    for (full, lean) in full.iter().zip(lean) {
        assert!(!full.legs.is_empty());
        assert!(lean.legs.is_empty());
        assert_eq!(lean.geometry, full.geometry);
        assert_eq!(lean.maneuvers, full.maneuvers);
    }
}

#[test]
fn include_parses_from_json_and_rejects_unknown_parts() {
    let request: RouteWorkflowRequest = serde_json::from_str(
        r#"{"locations": [{"point": {"lat": 52.0, "lng": 21.0005}}, {"point": {"lat": 52.002, "lng": 21.0015}}],
            "profile": "wheeled", "geometryFormat": "polyline6",
            "include": ["edgeAttributions", "costBreakdown", "edgeAttributions"]}"#,
    )
    .unwrap();
    assert_eq!(
        request.include,
        Some(BTreeSet::from([
            RouteResultPart::EdgeAttributions,
            RouteResultPart::CostBreakdown
        ]))
    );
    assert_eq!(
        request.route.geometry_format,
        wayfarer::GeometryFormat::Polyline6
    );
    let result = run(&engine(), request);
    let json = serde_json::to_value(&result).unwrap();
    assert!(json["polyline6"].is_string());
    assert!(paths(&json).contains("edgeAttributions[].costBreakdown"));
    // GeoJSON output decodes the polyline.
    let collection = route_feature_collection(&result).unwrap();
    assert!(
        collection["features"][0]["geometry"]["coordinates"]
            .as_array()
            .unwrap()
            .len()
            >= 2
    );

    let error = serde_json::from_str::<RouteWorkflowRequest>(
        r#"{"locations": [], "profile": "wheeled", "include": ["edgeRefs"]}"#,
    )
    .unwrap_err();
    assert!(error.to_string().contains("edgeRefs"), "{error}");
}

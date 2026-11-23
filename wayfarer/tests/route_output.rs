//! Route output: encoded geometry (`geometryFormat`), navigation from an
//! encoded route, and `inclinePercent` in edge attributions.

use std::sync::Arc;

use wayfarer::{
    GeoJsonLineString, GeoPoint, GeometryFormat, MobilityProfile, PreparedNavigationRoute,
    RouteAlternatives, RouteLegMode, RouteNavigationInput, RouteRequest, RouteResult, RouteSegment,
    RoutingEngine, RoutingPackage, decode_polyline6,
};
use wayfarer_compiler::package_build::{PackageBuildOptions, package_bytes};
use wayfarer_road_graph::{
    DirectionalAccess, RoadGraph, RoadGraphBuilder, RoadType, RoutableWay, enhance_road_graph,
};

fn way(id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
    RoutableWay {
        osm_way_id: id,
        points,
        road_type: RoadType::Primary,
        directional_access: DirectionalAccess::Both,
        name: Some(format!("Way {id}").into()),
        highway: Some("primary".into()),
        access: Some("yes".into()),
        ..Default::default()
    }
}

/// A square of four primary roads, 0.002° a side, with a curved east side
/// (an intermediate shape point) and an 8 % incline on the south side,
/// digitised west to east.
fn square() -> RoadGraph {
    let mut builder = RoadGraphBuilder::new("square");
    let mut south = way(1, vec![(52.0, 21.0), (52.0, 21.002)]);
    south.incline = Some("8%".into());
    builder.push_way(south);
    builder.push_way(way(
        2,
        vec![(52.0, 21.002), (52.001, 21.002_3), (52.002, 21.002)],
    ));
    builder.push_way(way(3, vec![(52.0, 21.0), (52.002, 21.0)]));
    builder.push_way(way(4, vec![(52.002, 21.0), (52.002, 21.002)]));
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    graph
}

fn engine() -> RoutingEngine {
    let (bytes, _) = package_bytes(&square(), &PackageBuildOptions::default()).unwrap();
    RoutingEngine::from_package(Arc::new(RoutingPackage::from_bytes(&bytes).unwrap()))
}

/// South side, round the curved east side, then straight on (direct) to a
/// point off the network.
fn request(format: GeometryFormat) -> RouteRequest {
    let mut request = RouteRequest::through(
        [
            GeoPoint::new(52.0, 21.000_23),
            GeoPoint::new(52.001_5, 21.002_1),
            GeoPoint::new(52.003, 21.003),
        ],
        MobilityProfile::Wheeled,
    );
    request.legs = vec![RouteLegMode::Routed, RouteLegMode::Direct];
    request.geometry_format = format;
    request
}

fn assert_same_points(decoded: &[GeoPoint], points: &[GeoPoint]) {
    assert!(!points.is_empty());
    assert_eq!(decoded.len(), points.len());
    for (decoded, point) in decoded.iter().zip(points) {
        assert!(
            (decoded.lat - point.lat).abs() <= 5e-7,
            "{decoded:?} {point:?}"
        );
        assert!(
            (decoded.lng - point.lng).abs() <= 5e-7,
            "{decoded:?} {point:?}"
        );
    }
}

/// Every geometry of `encoded` against the same one in `points`, decoded
/// with `decode`.
fn assert_encoded_like(
    encoded: &RouteResult,
    points: &RouteResult,
    decode: impl Fn(Option<&String>, Option<&GeoJsonLineString>) -> Vec<GeoPoint>,
) {
    assert!(encoded.geometry.is_empty());
    assert_same_points(
        &decode(encoded.polyline6.as_ref(), encoded.geojson.as_ref()),
        &points.geometry,
    );
    assert_same_points(&encoded.points().unwrap(), &points.geometry);
    assert_eq!(encoded.segments.len(), 2);
    for (encoded, points) in encoded.segments.iter().zip(&points.segments) {
        match (encoded, points) {
            (RouteSegment::Routed(encoded), RouteSegment::Routed(points)) => {
                assert!(encoded.geometry.is_empty());
                assert_same_points(
                    &decode(encoded.polyline6.as_ref(), encoded.geojson.as_ref()),
                    &points.geometry,
                );
                assert_eq!(encoded.legs.len(), points.legs.len());
                for (encoded, points) in encoded.legs.iter().zip(&points.legs) {
                    assert!(encoded.geometry.is_empty());
                    assert_same_points(
                        &decode(encoded.polyline6.as_ref(), encoded.geojson.as_ref()),
                        &points.geometry,
                    );
                }
            }
            (RouteSegment::Direct(encoded), RouteSegment::Direct(points)) => {
                assert!(encoded.geometry.is_empty());
                assert_same_points(
                    &decode(encoded.polyline6.as_ref(), encoded.geojson.as_ref()),
                    &points.geometry,
                );
            }
            _ => panic!("segment kinds differ"),
        }
    }
    // Shape indexes point into the encoded vertices as into the points.
    assert_eq!(encoded.maneuvers, points.maneuvers);
    assert_eq!(encoded.shape_segments, points.shape_segments);
}

#[test]
fn polyline6_replaces_every_geometry_and_keeps_its_vertices() {
    let engine = engine();
    let points = engine.route(request(GeometryFormat::Points)).unwrap();
    let encoded = engine.route(request(GeometryFormat::Polyline6)).unwrap();
    assert_encoded_like(&encoded, &points, |polyline6, geojson| {
        assert!(geojson.is_none());
        decode_polyline6(polyline6.expect("a polyline")).unwrap()
    });
    assert_eq!(encoded.distance_m, points.distance_m);
    assert_eq!(encoded.edge_attributions, points.edge_attributions);

    // The default output has neither field.
    let json = serde_json::to_string(&points).unwrap();
    assert!(!json.contains("polyline6") && !json.contains("geojson"));
    let json = serde_json::to_value(&encoded).unwrap();
    assert_eq!(json["geometry"], serde_json::json!([]));
    assert!(json["polyline6"].is_string());
    // A JSON route parses back with its encoding.
    let parsed: RouteResult = serde_json::from_value(json).unwrap();
    assert_eq!(parsed.polyline6, encoded.polyline6);
}

#[test]
fn geojson_replaces_every_geometry_longitude_first() {
    let engine = engine();
    let points = engine.route(request(GeometryFormat::Points)).unwrap();
    let encoded = engine.route(request(GeometryFormat::Geojson)).unwrap();
    assert_encoded_like(&encoded, &points, |polyline6, geojson| {
        assert!(polyline6.is_none());
        geojson.expect("a line string").to_points()
    });
    let json = serde_json::to_value(&encoded).unwrap();
    assert_eq!(json["geojson"]["type"], "LineString");
    let first = &json["geojson"]["coordinates"][0];
    assert_eq!(first[0].as_f64().unwrap(), 21.00023);
    assert_eq!(first[1].as_f64().unwrap(), 52.0);
}

#[test]
fn alternatives_are_encoded_too() {
    let engine = engine();
    let mut request = RouteRequest::new(
        GeoPoint::new(52.0, 21.000_5),
        GeoPoint::new(52.002, 21.001_5),
        MobilityProfile::Wheeled,
    );
    request.options.alternatives.requested_alternative_count = 2;
    request.options.alternatives.max_shared_edge_ratio = 1.0;
    request.options.alternatives.max_cost_ratio = 10.0;
    let points = engine.route(request.clone()).unwrap();
    request.geometry_format = GeometryFormat::Polyline6;
    let encoded = engine.route(request).unwrap();
    let (RouteAlternatives::Computed(points), RouteAlternatives::Computed(encoded)) =
        (&points.alternatives, &encoded.alternatives)
    else {
        panic!("alternatives were requested");
    };
    assert!(!points.is_empty(), "the square has a second way round");
    assert_eq!(points.len(), encoded.len());
    for (points, encoded) in points.iter().zip(encoded) {
        assert!(encoded.geometry.is_empty());
        assert_same_points(
            &decode_polyline6(encoded.polyline6.as_ref().unwrap()).unwrap(),
            &points.geometry,
        );
        for (points, encoded) in points.legs.iter().zip(&encoded.legs) {
            assert!(encoded.geometry.is_empty());
            assert_same_points(
                &decode_polyline6(encoded.polyline6.as_ref().unwrap()).unwrap(),
                &points.geometry,
            );
        }
    }
}

#[test]
fn navigation_follows_an_encoded_route() {
    let engine = engine();
    let points = engine.route(request(GeometryFormat::Points)).unwrap();
    for format in [GeometryFormat::Polyline6, GeometryFormat::Geojson] {
        let encoded = engine.route(request(format)).unwrap();
        let input = RouteNavigationInput::from_route(&encoded);
        let prepared = PreparedNavigationRoute::new(&input).unwrap();
        assert_same_points(prepared.geometry(), &points.geometry);
    }

    // A host can send the polyline in place of the points.
    let encoded = engine.route(request(GeometryFormat::Polyline6)).unwrap();
    let input: RouteNavigationInput = serde_json::from_value(serde_json::json!({
        "polyline6": encoded.polyline6,
        "maneuvers": encoded.maneuvers,
    }))
    .unwrap();
    let prepared = PreparedNavigationRoute::new(&input).unwrap();
    assert_eq!(prepared.geometry().len(), points.geometry.len());

    let mut both = input.clone();
    both.geometry = points.geometry.clone();
    assert!(PreparedNavigationRoute::new(&both).is_err());
    let mut broken = input;
    broken.polyline6 = Some("_izlh".to_string());
    let error = PreparedNavigationRoute::new(&broken).err().unwrap();
    assert!(error.contains("polyline6"), "{error}");
}

#[test]
fn incline_is_signed_by_the_direction_of_travel() {
    let engine = engine();
    let incline_of = |from: GeoPoint, to: GeoPoint| {
        let route = engine
            .route(RouteRequest::new(from, to, MobilityProfile::Wheeled))
            .unwrap();
        route
            .edge_attributions
            .iter()
            .map(|edge| edge.incline_percent)
            .collect::<Vec<_>>()
    };
    let west = GeoPoint::new(52.0, 21.000_2);
    let east = GeoPoint::new(52.0, 21.001_8);
    assert_eq!(incline_of(west, east), [Some(8)]);
    assert_eq!(incline_of(east, west), [Some(-8)]);

    // Edges without one leave the key out.
    let route = engine
        .route(RouteRequest::new(
            GeoPoint::new(52.000_2, 21.0),
            GeoPoint::new(52.001_8, 21.0),
            MobilityProfile::Wheeled,
        ))
        .unwrap();
    assert_eq!(route.edge_attributions[0].incline_percent, None);
    let json = serde_json::to_string(&route).unwrap();
    assert!(!json.contains("inclinePercent"));
    let json = serde_json::to_string(
        &engine
            .route(RouteRequest::new(west, east, MobilityProfile::Wheeled))
            .unwrap(),
    )
    .unwrap();
    assert!(json.contains(r#""inclinePercent":8"#), "{json}");
}

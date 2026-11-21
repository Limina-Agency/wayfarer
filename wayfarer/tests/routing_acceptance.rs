use std::collections::{BTreeSet, HashMap};
use std::sync::Arc;

use tempfile::tempdir;
use wayfarer::internals::{
    Correlator, GraphAccess, GraphViewEdge, RouteGraphIndex, project_route_overlay_geometry,
};
use wayfarer::reference::ReferenceEngine;
use wayfarer::{
    CorrelationMode, CorrelationPolicy, CorrelationRejectReason, GeoPoint, LegRequest, LegResult,
    MobilityProfile, RouteEdgeAvoidanceRange, RouteEdgeCostMultiplierRange, RouteEdgeRange,
    RouteEndpointOptions, RouteError, RouteGuidanceQualityFlag, RouteIsochroneContour,
    RouteIsochroneMetric, RouteIsochroneRequest, RouteManeuverKind, RouteMatrixCellStatus,
    RouteMatrixRequest, RouteOverlayGeometryKind, RouteRequest, RouteShapeQualityFlag, RouteStatus,
    RoutingEngine,
};
use wayfarer::{RoutingPackage, RoutingPackageError};
use wayfarer_compiler::package_build::PackageBuildOptions;
use wayfarer_package::{DirectionalAccess, MobilityMode, OsmMode, RoadType};
use wayfarer_road_graph::{
    NodeBarrierTags, ResolvedEdgeRestriction, RoadGraph, RoadGraphBuilder,
    RoadGraphPackageManifest, RoadWayFacts, RoutableWay, build_road_graph_manifest,
    enhance_road_graph, inspect_road_graph_container, load_road_graph_file,
    persist_road_graph_file, serialize_road_graph,
};

/// The package of `graph`, built in memory with the default options.
fn package(graph: &RoadGraph) -> Arc<RoutingPackage> {
    let (bytes, _) =
        wayfarer_compiler::package_build::package_bytes(graph, &PackageBuildOptions::default())
            .expect("the road graph builds a routing package");
    Arc::new(RoutingPackage::from_bytes(&bytes).unwrap())
}

/// An engine over the package of `graph`.
fn engine(graph: &RoadGraph) -> RoutingEngine {
    RoutingEngine::from_package(package(graph))
}

/// The reference oracle over `graph` and its package.
fn reference(graph: &RoadGraph) -> ReferenceEngine<'_> {
    ReferenceEngine::new(graph, package(graph))
}

fn way(id: i64, points: Vec<(f64, f64)>, road_type: RoadType) -> RoutableWay {
    RoutableWay {
        osm_way_id: id,
        points,
        source_node_ids: Vec::new(),
        road_type,
        directional_access: DirectionalAccess::Both,
        name: Some(format!("Way {id}").into()),
        ref_name: None,
        highway: Some("residential".into()),
        access: Some("yes".into()),
        vehicle: None,
        motor_vehicle: None,
        motorcar: None,
        bicycle: None,
        foot: None,
        oneway_foot: None,
        oneway_bicycle: None,
        motorroad: None,
        service: None,
        surface: None,
        tracktype: None,
        smoothness: None,
        maxspeed: None,
        lanes: None,
        layer: None,
        level: None,
        destination: None,
        destination_ref: None,
        destination_street: None,
        junction_ref: None,
        is_toll: false,
        is_ferry: false,
        has_gate: false,
        is_access_restricted: false,
        has_bridge: false,
        has_tunnel: false,
        has_ford: false,
        is_roundabout: false,
        is_link: false,
        is_destination_only: false,
        access_restricted: false,
        area: None,
        area_highway: None,
        conditional_restrictions: Vec::new(),
        ..Default::default()
    }
}

fn one_way(id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
    let mut route = way(id, points, RoadType::Residential);
    route.directional_access = DirectionalAccess::Forward;
    // Vehicular one-way with no pedestrian access, so these fixtures model a
    // strictly single-direction edge. Pedestrian contraflow on one-ways is
    // covered explicitly by `on_foot_traverses_one_way_against_traffic`.
    route.foot = Some("no".into());
    route
}

fn one_way_foot_allowed(id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
    let mut route = way(id, points, RoadType::Residential);
    route.directional_access = DirectionalAccess::Forward;
    route
}

fn roundabout_way(id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
    let mut route = way(id, points, RoadType::Residential);
    route.is_roundabout = true;
    route
}

fn way_with_road_facts(id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
    let mut route = way(id, points, RoadType::Track);
    route.ref_name = Some("MSR-7".into());
    route.surface = Some("gravel".into());
    route.tracktype = Some("grade2".into());
    route.smoothness = Some("intermediate".into());
    route.lanes = Some("2".into());
    route.layer = Some("1".into());
    route.level = Some("L1".into());
    route.destination = Some("Forward Base".into());
    route.destination_ref = Some("MSR-7".into());
    route.destination_street = Some("Supply Road".into());
    route.junction_ref = Some("7A".into());
    route.is_toll = true;
    route.has_gate = true;
    route.is_access_restricted = true;
    route.has_bridge = true;
    route.has_tunnel = true;
    route.has_ford = true;
    route.is_destination_only = true;
    route
}

fn layered_way(
    id: i64,
    points: Vec<(f64, f64)>,
    layer: i16,
    level: impl AsRef<str>,
) -> RoutableWay {
    let mut route = way(id, points, RoadType::Residential);
    route.layer = Some(layer.to_string().into());
    route.level = Some(level.as_ref().into());
    route
}

fn named_ref_way(
    id: i64,
    points: Vec<(f64, f64)>,
    name: impl AsRef<str>,
    ref_name: impl AsRef<str>,
) -> RoutableWay {
    let mut route = way(id, points, RoadType::Residential);
    route.name = Some(name.as_ref().into());
    route.ref_name = Some(ref_name.as_ref().into());
    route
}

fn destination_only_primary(id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
    let mut route = way(id, points, RoadType::Primary);
    route.is_destination_only = true;
    route
}

fn conditionally_closed_primary(id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
    let mut route = way(id, points, RoadType::Primary);
    route.conditional_restrictions =
        vec!["access:conditional=no @ (2026 May 01-2026 Jun 30)".into()];
    route
}

fn conditionally_slow_primary(id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
    let mut route = way(id, points, RoadType::Primary);
    route.maxspeed = Some("80".into());
    route.conditional_restrictions =
        vec!["maxspeed:conditional=30 @ (Mo-Fr 07:00-09:00,16:00-18:00)".into()];
    route
}

fn signed_link_primary(id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
    let mut route = way(id, points, RoadType::Primary);
    route.name = Some("A4 ramp".into());
    route.ref_name = Some("A4".into());
    route.destination = Some("Krakow".into());
    route.destination_ref = Some("A4".into());
    route.junction_ref = Some("7A".into());
    route.is_link = true;
    route
}

fn controlled_access_primary(id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
    let mut route = way(id, points, RoadType::Primary);
    route.is_toll = true;
    route.is_ferry = true;
    route.has_gate = true;
    route.is_access_restricted = true;
    route.access = Some("permit".into());
    route.access_restricted = true;
    route
}

fn foot_only_path(id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
    let mut route = way(id, points, RoadType::Path);
    route.vehicle = Some("no".into());
    route.motor_vehicle = Some("no".into());
    route.foot = Some("yes".into());
    route
}

fn graph_from_ways(graph_id: &str, ways: Vec<RoutableWay>) -> RoadGraph {
    let mut builder = RoadGraphBuilder::new(graph_id);
    for way in ways {
        builder.push_way(way);
    }
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    graph
}

fn road_way_facts(id: i64, road_type: RoadType) -> RoadWayFacts {
    RoadWayFacts {
        osm_way_id: id,
        road_type,
        directional_access: DirectionalAccess::Both,
        name: Some(format!("Way {id}")),
        ref_name: None,
        highway: Some("primary".to_string()),
        access: Some("yes".to_string()),
        vehicle: None,
        motor_vehicle: None,
        motorcar: None,
        bicycle: None,
        foot: None,
        oneway_foot: None,
        oneway_bicycle: None,
        motorroad: None,
        service: None,
        surface: None,
        tracktype: None,
        smoothness: None,
        maxspeed: None,
        lanes: None,
        turn_lanes: None,
        turn_lanes_forward: None,
        turn_lanes_backward: None,
        lane_connectivity_available: false,
        admin_country_id: None,
        admin_region_id: None,
        timezone_id: None,
        has_bridge: false,
        has_tunnel: false,
        has_ford: false,
        is_roundabout: false,
        is_link: false,
        is_destination_only: false,
        access_restricted: false,
        layer: None,
        level: None,
        destination: None,
        destination_ref: None,
        destination_street: None,
        junction_ref: None,
        is_toll: false,
        is_ferry: false,
        has_gate: false,
        is_access_restricted: false,
        ..Default::default()
    }
}

fn road_graph_manifest(graph: &RoadGraph) -> RoadGraphPackageManifest {
    let bytes = serialize_road_graph(graph).unwrap();
    let container = inspect_road_graph_container(&bytes).unwrap();
    build_road_graph_manifest(graph, &container)
}

fn tight_request(origin: GeoPoint, destination: GeoPoint) -> LegRequest {
    let mut request = LegRequest::new(origin, destination, MobilityProfile::Wheeled);
    request.options.max_correlation_distance_m = 25;
    request
}

fn tight_profile_request(
    origin: GeoPoint,
    destination: GeoPoint,
    profile: MobilityProfile,
) -> LegRequest {
    let mut request = LegRequest::new(origin, destination, profile);
    request.options.max_correlation_distance_m = 25;
    request
}

fn restrict_transition(
    graph: &mut RoadGraph,
    from_edge_id: u64,
    to_edge_id: u64,
    via_node_id: i64,
) {
    graph.set_resolved_restrictions(vec![ResolvedEdgeRestriction::Simple {
        relation_id: 9001,
        restriction: "no_left_turn".to_string(),
        from_edge_id,
        to_edge_id,
        via_node_id,
        osm_mode_mask: OsmMode::MOTOR_VEHICLE,
        mobility_mode_mask: MobilityMode::WHEELED | MobilityMode::TRACKED,
    }]);
}

fn way_edge_ids(graph: &RoadGraph, osm_way_id: i64) -> BTreeSet<u64> {
    let shared_info_ids = graph
        .shared_edge_infos
        .iter()
        .filter(|info| info.osm_way_id == osm_way_id)
        .map(|info| info.shared_edge_info_id)
        .collect::<BTreeSet<_>>();
    graph
        .graph
        .edge_weights()
        .filter(|edge| shared_info_ids.contains(&edge.shared_edge_info_id))
        .map(|edge| edge.edge_id)
        .collect()
}

fn restrict_complex_sequence(
    graph: &mut RoadGraph,
    from_edge_id: u64,
    via_edge_ids: Vec<u64>,
    to_edge_id: u64,
) {
    graph.set_resolved_restrictions(vec![ResolvedEdgeRestriction::Complex {
        relation_id: 9002,
        restriction: "no_straight_on".to_string(),
        from_edge_id,
        to_edge_id,
        via_edge_ids,
        osm_mode_mask: OsmMode::MOTOR_VEHICLE,
        mobility_mode_mask: MobilityMode::WHEELED | MobilityMode::TRACKED,
    }]);
}

fn assert_point_close(actual: GeoPoint, expected: GeoPoint) {
    assert!(
        actual.distance_to(expected) < 0.25,
        "expected {actual:?} within 0.25m of {expected:?}"
    );
}

#[test]
fn graph_view_owned_records_match_indexed_edge_records() {
    let graph = graph_from_ways(
        "owned-graph-view-records",
        vec![way(
            1,
            vec![(52.0, 21.0), (52.0, 21.001)],
            RoadType::Primary,
        )],
    );
    let index = Arc::new(RouteGraphIndex::build(&graph));
    let access = GraphAccess::from_index(&graph, index);
    let record = access.edge_by_id(1).expect("edge exists");
    let owned = record.to_view_edge();

    assert_eq!(owned.edge.edge_id, record.edge.edge_id);
    assert_eq!(owned.source.id, record.source.id);
    assert_eq!(owned.target.id, record.target.id);
    assert_eq!(
        owned.shared_info.shared_edge_info_id,
        record.shared_info.shared_edge_info_id
    );
    assert_eq!(owned.raw_edge_index, record.raw_edge_index);
}

#[test]
fn graph_access_reports_readiness_and_outgoing_edges() {
    let graph = graph_from_ways(
        "straight",
        vec![way(
            1,
            vec![(52.0, 21.0), (52.0, 21.001)],
            RoadType::Residential,
        )],
    );

    let access = GraphAccess::new(&graph);

    assert_eq!(access.graph_id(), "straight");
    assert_eq!(access.node_count(), 2);
    assert_eq!(access.edge_count(), 2);
    assert!(access.is_ready());

    let outgoing = access.outgoing_edges_from_node_id(0).unwrap();
    assert_eq!(outgoing.len(), 1);
    assert_eq!(outgoing[0].edge.edge_id, 1);
    assert!(access.edge_by_id(1).is_some());
}

#[test]
fn graph_access_exposes_dense_raw_indexes_for_search_labels() {
    let graph = graph_from_ways(
        "dense-index",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Residential),
            way(
                2,
                vec![(52.0, 21.001), (52.0, 21.002)],
                RoadType::Residential,
            ),
        ],
    );
    let access = GraphAccess::new(&graph);

    assert_eq!(access.raw_node_count(), access.node_count());
    assert_eq!(access.raw_edge_count(), access.edge_count());
    assert_eq!(access.raw_node_index_for_node_id(0), Some(0));
    assert_eq!(access.node_by_raw_node_index(0).unwrap().id, 0);
    assert_eq!(access.node_id_for_raw_node_index(1), Some(1));

    let first_edge_raw_index = access.raw_edge_index_for_edge_id(1).unwrap();
    let first_edge = access
        .edge_record_by_raw_edge_index(first_edge_raw_index)
        .unwrap();
    assert_eq!(first_edge.edge.edge_id, 1);
    assert_eq!(
        access.raw_node_indexes_for_raw_edge_index(first_edge_raw_index),
        Some((0, 1))
    );

    let outgoing = access.outgoing_raw_edge_indexes_from_node_id(1).unwrap();
    let outgoing_edge_ids = outgoing
        .iter()
        .filter_map(|raw_index| access.edge_record_by_raw_edge_index(*raw_index))
        .map(|record| record.edge.edge_id)
        .collect::<Vec<_>>();
    assert_eq!(outgoing_edge_ids, vec![2, 3]);

    let incoming = access.incoming_raw_edge_indexes_to_node_id(1).unwrap();
    let incoming_edge_ids = incoming
        .iter()
        .filter_map(|raw_index| access.edge_record_by_raw_edge_index(*raw_index))
        .map(|record| record.edge.edge_id)
        .collect::<Vec<_>>();
    assert_eq!(incoming_edge_ids, vec![1, 4]);
}

#[test]
fn waypoint_maneuver_covers_the_stretch_to_the_next_maneuver() {
    let graph = graph_from_ways(
        "waypoint-maneuver-distances",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.002)], RoadType::Primary),
            way(2, vec![(52.0, 21.002), (52.002, 21.002)], RoadType::Primary),
        ],
    );
    let route = engine(&graph)
        .route(RouteRequest::through(
            [
                GeoPoint::new(52.0, 21.0002),
                GeoPoint::new(52.0, 21.001),
                GeoPoint::new(52.0015, 21.002),
            ],
            MobilityProfile::Wheeled,
        ))
        .unwrap();

    let kinds = route
        .maneuvers
        .iter()
        .map(|maneuver| maneuver.kind)
        .collect::<Vec<_>>();
    assert_eq!(
        kinds,
        [
            RouteManeuverKind::Start,
            RouteManeuverKind::Waypoint,
            RouteManeuverKind::Turn,
            RouteManeuverKind::Destination,
        ]
    );
    // Each maneuver runs to the next one, so together they cover the route.
    let waypoint = &route.maneuvers[1];
    let turn = &route.maneuvers[2];
    assert!(waypoint.distance_m > 60.0, "{}", waypoint.distance_m);
    assert!(waypoint.duration_s > 0.0);
    assert_eq!(waypoint.end_shape_index, turn.begin_shape_index);
    let distance_m = route.maneuvers.iter().map(|m| m.distance_m).sum::<f64>();
    let duration_s = route.maneuvers.iter().map(|m| m.duration_s).sum::<f64>();
    assert!((distance_m - route.distance_m).abs() < 1e-6, "{distance_m}");
    assert!((duration_s - route.duration_s).abs() < 1e-6, "{duration_s}");
}

#[test]
fn package_round_trip_matches_in_memory_directed_route() {
    let graph = graph_from_ways(
        "package-round-trip",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
        ],
    );
    let manifest = road_graph_manifest(&graph);
    let (_dir, package) = persisted_package(&graph);
    assert_eq!(
        package.metadata().road_graph_blake3_hash_hex,
        manifest.blake3_hash_hex
    );
    assert_eq!(package.metadata().node_count, manifest.node_count);
    assert_eq!(package.graph_id(), graph.graph_id);

    let request = tight_request((52.0, 21.00005).into(), (52.0, 21.00195).into());
    let in_memory = engine(&graph).route_leg(request.clone()).unwrap();
    let loaded = RoutingEngine::from_package(package)
        .route_leg(request)
        .unwrap();

    assert_eq!(loaded.edge_ids, in_memory.edge_ids);
    assert_eq!(loaded.cost, in_memory.cost);
    assert_eq!(loaded.legs, in_memory.legs);
}

fn many_edge_route_tile_graph() -> RoadGraph {
    graph_from_ways(
        "compact-v3-paged-indexes",
        (0..128)
            .map(|index| {
                let lng = 21.0 + index as f64 * 0.001;
                way(
                    index + 1,
                    vec![(52.0, lng), (52.0, lng + 0.001)],
                    RoadType::Primary,
                )
            })
            .collect(),
    )
}

/// `graph` written as a routing package and opened again.
fn persisted_package(graph: &RoadGraph) -> (tempfile::TempDir, Arc<RoutingPackage>) {
    let manifest = road_graph_manifest(graph);
    let dir = tempdir().unwrap();
    let path = dir.path().join("routing.mrt");
    wayfarer_compiler::package_build::write_package(
        graph,
        Some(&manifest),
        &path,
        &PackageBuildOptions::default(),
    )
    .unwrap();
    let package = RoutingPackage::open(&path).unwrap();
    (dir, Arc::new(package))
}

#[test]
fn package_outbound_reach_matches_graph_access() {
    let graph = many_edge_route_tile_graph();
    let edge = graph.graph.edge_references().next().unwrap();
    let node_id = graph.graph.node(edge.target()).id;
    let (_dir, package) = persisted_package(&graph);
    let engine = RoutingEngine::from_package(package);

    let actual =
        engine
            .route_graph_view()
            .outbound_reach_edge_count(node_id, MobilityProfile::Wheeled, 64);

    assert_eq!(actual, Some(64));
}

#[test]
fn package_inbound_reach_matches_graph_access() {
    let graph = graph_from_ways(
        "package-inbound-reach",
        vec![
            way(1, vec![(52.0, 21.019), (52.0, 21.0198)], RoadType::Primary),
            way(2, vec![(52.0, 21.0198), (52.0, 21.0202)], RoadType::Primary),
            way(3, vec![(52.0, 21.0202), (52.0, 21.021)], RoadType::Primary),
        ],
    );
    let graph_access = GraphAccess::new(&graph);
    let node_id = graph_access.edge_by_id(1).unwrap().source.id;
    let (_dir, package) = persisted_package(&graph);
    let engine = RoutingEngine::from_package(package);
    let actual =
        engine
            .route_graph_view()
            .inbound_reach_edge_count(node_id, MobilityProfile::Wheeled, 64);

    assert_eq!(actual, Some(6));
}

#[test]
fn package_opposing_edges_and_nodes_match_graph_access() {
    let graph = graph_from_ways(
        "package-opposing-edge",
        vec![way(
            1,
            vec![(52.0, 21.0), (52.0, 21.001)],
            RoadType::Primary,
        )],
    );
    let (_dir, package) = persisted_package(&graph);
    let engine = RoutingEngine::from_package(package);
    let view = engine.route_graph_view();
    for edge in graph.graph.edge_weights() {
        assert_eq!(view.opposing_edge_id(edge.edge_id), edge.opposing_edge_id);
        let record = view.edge_by_id(edge.edge_id).unwrap();
        assert_eq!(record.edge.opposing_edge_id, edge.opposing_edge_id);
        let node = view.node_by_id(record.source.id).expect("node by id");
        assert_eq!(node.id, record.source.id);
        assert_eq!((node.lat, node.lng), (record.source.lat, record.source.lng));
    }
    assert_eq!(view.opposing_edge_id(1), Some(2));
}

#[test]
fn package_simple_restrictions_match_graph_access() {
    let mut graph = graph_from_ways(
        "package-simple-restriction",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
        ],
    );
    restrict_transition(&mut graph, 1, 3, 1);
    let (_dir, package) = persisted_package(&graph);
    let engine = RoutingEngine::from_package(package);
    for (from, to) in [(1, 3), (3, 1), (2, 4), (4, 2)] {
        for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
            assert_eq!(
                engine
                    .route_graph_view()
                    .transition_restricted(from, to, 1, profile),
                (from, to) == (1, 3) && profile == MobilityProfile::Wheeled,
                "{from} -> {to} {profile:?}"
            );
        }
    }
    assert!(
        engine
            .route_graph_view()
            .transition_restricted(1, 3, 1, MobilityProfile::Wheeled)
    );
}

#[test]
fn package_complex_restrictions_match_graph_access() {
    let mut graph = graph_from_ways(
        "package-complex-restriction",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(3, vec![(52.0, 21.002), (52.0, 21.003)], RoadType::Primary),
        ],
    );
    restrict_complex_sequence(&mut graph, 1, vec![3], 5);
    let (_dir, package) = persisted_package(&graph);
    let engine = RoutingEngine::from_package(package);
    let view = engine.route_graph_view();
    for to in [3, 5] {
        assert_eq!(
            view.has_complex_restrictions_to_edge(to, MobilityProfile::Wheeled),
            to == 5
        );
        for path in [&[1, 3][..], &[3][..], &[2, 1, 3][..]] {
            assert_eq!(
                view.complex_transition_restricted_for_path(path, to, MobilityProfile::Wheeled),
                to == 5 && path.ends_with(&[1, 3]),
                "{path:?} -> {to}"
            );
        }
    }
    assert!(view.has_complex_restrictions_to_edge(5, MobilityProfile::Wheeled));
    assert!(view.complex_transition_restricted_for_path(&[1, 3], 5, MobilityProfile::Wheeled));
}

#[test]
fn complex_restriction_forces_the_detour_on_a_package_engine() {
    let mut graph = graph_from_ways(
        "compact-v3-tile-local-connection-restriction",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(3, vec![(52.0, 21.002), (52.0, 21.003)], RoadType::Primary),
            way(
                4,
                vec![(52.0, 21.0), (52.0008, 21.0015)],
                RoadType::Residential,
            ),
            way(
                5,
                vec![(52.0008, 21.0015), (52.0, 21.003)],
                RoadType::Residential,
            ),
        ],
    );
    restrict_complex_sequence(&mut graph, 1, vec![3], 5);
    let origin = (52.0, 21.00002).into();
    let destination = (52.0, 21.00298).into();
    let (_dir, package) = persisted_package(&graph);
    let engine = RoutingEngine::from_package(package);

    let route = engine
        .route_leg(tight_request(origin, destination))
        .expect("restricted route");

    // 1 -> 3 -> 5 is forbidden. Turning back twice on the via way (3, 4, 3)
    // would evade it, but each U-turn pays its turn cost, so the residential
    // detour (7, 9) is cheaper; the exact reference (with a path context)
    // agrees.
    assert_eq!(route.edge_ids, vec![7, 9]);
    assert!(route.diagnostics.restricted_transition_count > 0);
    let reference = reference(&graph)
        .with_path_context(2)
        .route_leg(tight_request(origin, destination))
        .unwrap();
    assert_eq!(reference.edge_ids, route.edge_ids);
}

#[test]
fn package_correlation_finds_a_curved_edge_far_from_its_nodes() {
    let graph = graph_from_ways(
        "compact-v3-curved-cross-tile-correlation",
        vec![way(
            1,
            vec![(52.001, 21.001), (52.031, 21.031), (52.001, 21.041)],
            RoadType::Residential,
        )],
    );
    let edge_id = graph.graph.edge_weights().next().unwrap().edge_id;
    let (_dir, package) = persisted_package(&graph);
    let engine = RoutingEngine::from_package(package);

    let search = Correlator::new(engine.route_graph_view()).search(
        GeoPoint::new(52.031, 21.031),
        MobilityProfile::Wheeled,
        &CorrelationPolicy {
            max_distance_m: 25,
            candidate_limit: 8,
            ..CorrelationPolicy::default()
        },
        &Default::default(),
    );

    assert!(
        search
            .candidates
            .iter()
            .any(|candidate| candidate.edge_id == edge_id)
    );
}

#[test]
fn package_correlation_uses_the_nearest_segment_of_a_curved_edge() {
    let graph = graph_from_ways(
        "compact-v3-curved-nearest-posting",
        vec![one_way(
            1,
            vec![(52.00020, 21.00030), (52.00020, 21.00035), (52.0, 21.00010)],
        )],
    );
    let edge_id = graph.graph.edge_weights().next().unwrap().edge_id;
    let (_dir, package) = persisted_package(&graph);
    let engine = RoutingEngine::from_package(package);

    let search = Correlator::new(engine.route_graph_view()).search(
        GeoPoint::new(52.0, 21.0),
        MobilityProfile::Wheeled,
        &CorrelationPolicy {
            max_distance_m: 25,
            candidate_limit: 8,
            ..CorrelationPolicy::default()
        },
        &Default::default(),
    );

    assert!(
        search
            .candidates
            .iter()
            .any(|candidate| candidate.edge_id == edge_id),
        "a bbox-only first posting must not mask a later segment within the radius"
    );
}

#[test]
fn package_opens_verifies_and_routes_without_any_road_graph_files() {
    let graph = graph_from_ways(
        "package-standalone",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
        ],
    );
    let (dir, package) = persisted_package(&graph);
    let path = dir.path().join("routing.mrt");
    let verification = RoutingPackage::verify(&path).unwrap();
    assert_eq!(verification.dataset_hash, package.dataset_hash());
    assert!(
        std::fs::read_dir(dir.path()).unwrap().count() == 1,
        "the package is one file"
    );
    drop(graph);

    let result = RoutingEngine::from_package(package)
        .route_leg(tight_request(
            (52.0, 21.00005).into(),
            (52.0, 21.00195).into(),
        ))
        .unwrap();
    assert!(!result.edge_ids.is_empty());
}

#[test]
fn package_correlation_reach_matches_in_memory_route() {
    let graph = graph_from_ways(
        "package-correlation-reach",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(3, vec![(52.0, 21.002), (52.0, 21.003)], RoadType::Primary),
            way(4, vec![(52.0, 21.003), (52.0, 21.004)], RoadType::Primary),
        ],
    );
    let mut request = tight_request((52.0, 21.00005).into(), (52.0, 21.00395).into());
    request.origin_options = RouteEndpointOptions::default().with_min_outbound_reach_edge_count(2);
    request.destination_options =
        RouteEndpointOptions::default().with_min_inbound_reach_edge_count(2);
    let expected = engine(&graph).route_leg(request.clone()).unwrap();
    let (_dir, package) = persisted_package(&graph);

    let actual = RoutingEngine::from_package(package)
        .route_leg(request)
        .unwrap();

    assert_eq!(actual.origin, expected.origin);
    assert_eq!(actual.destination, expected.destination);
    assert_eq!(actual.edge_ids, expected.edge_ids);
    assert_eq!(actual.cost, expected.cost);
    assert_eq!(actual.distance_m, expected.distance_m);
}

#[test]
fn package_round_trip_routes_equal_to_in_memory_engine() {
    let graph = graph_from_ways(
        "package-round-trip-routes",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(
                3,
                vec![(52.0, 21.002), (52.001, 21.002)],
                RoadType::Residential,
            ),
        ],
    );
    let (_dir, package) = persisted_package(&graph);
    assert!(package.byte_len() > 0);
    assert_eq!(package.counts().edge_count, graph.edge_count());

    let request = tight_request((52.0, 21.00005).into(), (52.00095, 21.002).into());
    let in_memory = engine(&graph).route_leg(request.clone()).unwrap();
    let loaded = RoutingEngine::from_package(package)
        .route_leg(request)
        .unwrap();

    assert_eq!(loaded.edge_ids, in_memory.edge_ids);
    assert!((loaded.distance_m - in_memory.distance_m).abs() < 0.01);
}

#[test]
fn package_candidate_lookup_finds_long_endpoint_edges() {
    let mut ways = Vec::new();
    let mut lng = 21.0;
    for id in 1..=16 {
        ways.push(way(
            id,
            vec![(52.0, lng), (52.0, lng + 0.01)],
            RoadType::Motorway,
        ));
        lng += 0.01;
    }
    let graph = graph_from_ways("package-candidate-long-edges", ways);
    let (_dir, package) = persisted_package(&graph);
    let engine = RoutingEngine::from_package(package);
    let view = engine.route_graph_view();

    let candidates = view.candidate_edges_near(GeoPoint::new(52.0, 21.00002), 25);
    assert!(
        !candidates.is_empty(),
        "the package should find endpoint candidates on long edges"
    );
    for record in &candidates {
        let raw = view
            .raw_edge_index_for_edge_id(record.edge.edge_id)
            .expect("candidate id resolves to an edge");
        assert_eq!(
            view.edge_record_by_raw_edge_index(raw).as_ref(),
            Some(record)
        );
        assert!(
            record.shared_info.geometry.len() >= 2,
            "candidate records carry full snap geometry"
        );
    }
}

#[test]
fn package_holds_every_edge_of_every_road_class_without_changing_routes() {
    let graph = graph_from_ways(
        "package-road-classes",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(
                10,
                vec![(52.001, 21.0), (52.001, 21.001)],
                RoadType::Motorway,
            ),
            way(
                2,
                vec![(52.0, 21.001), (52.0, 21.002)],
                RoadType::Residential,
            ),
            way(3, vec![(52.0, 21.002), (52.0, 21.003)], RoadType::Track),
        ],
    );
    let request = tight_request((52.0, 21.00005).into(), (52.0, 21.00295).into());
    let expected = engine(&graph).route_leg(request.clone()).unwrap();
    let (_dir, package) = persisted_package(&graph);
    let engine = RoutingEngine::from_package(package);
    for edge in graph.graph.edge_weights() {
        assert!(
            engine
                .route_graph_view()
                .raw_edge_index_for_edge_id(edge.edge_id)
                .is_some()
        );
    }
    let actual = engine.route_leg(request).unwrap();
    assert_eq!(actual.edge_ids, expected.edge_ids);
}

#[test]
fn package_edge_lookup_and_adjacency_match_indexed_access() {
    let graph = graph_from_ways(
        "package-adjacency",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(
                2,
                vec![(52.0, 21.001), (52.0, 21.002)],
                RoadType::Residential,
            ),
            way(3, vec![(52.0, 21.001), (52.001, 21.001)], RoadType::Track),
        ],
    );
    let access = GraphAccess::new(&graph);
    let (_dir, package) = persisted_package(&graph);
    let engine = RoutingEngine::from_package(package);
    let view = engine.route_graph_view();
    let ids = |records: Option<Vec<GraphViewEdge>>| {
        let mut ids = records
            .unwrap()
            .into_iter()
            .map(|record| record.edge.edge_id)
            .collect::<Vec<_>>();
        ids.sort_unstable();
        ids
    };
    let road_graph_ids = |records: Option<Vec<wayfarer::internals::EdgeRecord<'_>>>| {
        let mut ids = records
            .unwrap()
            .into_iter()
            .map(|record| record.edge.edge_id)
            .collect::<Vec<_>>();
        ids.sort_unstable();
        ids
    };
    for node in graph.graph.node_weights() {
        assert_eq!(
            ids(view.outgoing_edges_from_node_id(node.id)),
            road_graph_ids(access.outgoing_edges_from_node_id(node.id)),
            "node {}",
            node.id
        );
        assert_eq!(
            ids(view.incoming_edges_to_node_id(node.id)),
            road_graph_ids(access.incoming_edges_to_node_id(node.id)),
            "node {}",
            node.id
        );
        assert_eq!(view.component_id_for_node(node.id), node.component_id);
    }
    for edge in graph.graph.edge_weights() {
        let record = view.edge_by_id(edge.edge_id).unwrap();
        let expected = access.edge_by_id(edge.edge_id).unwrap();
        // Packages store no road hierarchy tier (nothing routes on it).
        let mut expected_info = expected.shared_info.clone();
        expected_info.hierarchy_tier = None;
        assert_eq!(record.shared_info, expected_info);
        assert_eq!(
            (record.source.id, record.target.id),
            (expected.source.id, expected.target.id)
        );
    }
}

#[test]
fn package_rejects_a_corrupted_header() {
    let graph = graph_from_ways(
        "package-corrupt-header",
        vec![way(
            1,
            vec![(52.0, 21.0), (52.0, 21.001)],
            RoadType::Primary,
        )],
    );
    let (dir, package) = persisted_package(&graph);
    // Unmap it first: Windows refuses to overwrite a mapped file.
    drop(package);
    let path = dir.path().join("routing.mrt");
    let mut bytes = std::fs::read(&path).unwrap();
    bytes[20] ^= 0x01;
    std::fs::write(&path, bytes).unwrap();

    let error = RoutingPackage::open(&path).unwrap_err();
    assert!(matches!(error, RoutingPackageError::Invalid(_)), "{error}");
    assert!(RoutingPackage::verify(&path).is_err());
}

#[test]
fn package_rejects_invalid_and_outdated_files() {
    let graph = graph_from_ways(
        "package-invalid",
        vec![way(
            1,
            vec![(52.0, 21.0), (52.0, 21.001)],
            RoadType::Primary,
        )],
    );
    let (dir, package) = persisted_package(&graph);
    // Unmap it first: Windows refuses to overwrite a mapped file.
    drop(package);
    let path = dir.path().join("routing.mrt");
    let bytes = std::fs::read(&path).unwrap();
    std::fs::write(&path, b"bad-package").unwrap();
    assert!(matches!(
        RoutingPackage::open(&path).unwrap_err(),
        RoutingPackageError::Invalid(_)
    ));
    // A package of an older format must be recompiled, not opened.
    let mut old = bytes;
    let version = wayfarer_package::format::HEADER_VERSION;
    old[version..version + 4].copy_from_slice(&0u32.to_le_bytes());
    std::fs::write(&path, old).unwrap();
    let error = RoutingPackage::open(&path).unwrap_err();
    assert!(matches!(error, RoutingPackageError::Outdated(_)), "{error}");
    assert!(
        error.to_string().contains("recompile this package"),
        "{error}"
    );
}

#[test]
fn package_correlation_preserves_candidate_order() {
    let graph = graph_from_ways(
        "package-correlation",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.0), (52.001, 21.0)], RoadType::Residential),
        ],
    );
    let in_memory = engine(&graph);
    let fresh_access = in_memory.route_graph_view();
    let (_dir, package) = persisted_package(&graph);
    let engine = RoutingEngine::from_package(package);
    let point = GeoPoint::new(52.00002, 21.0007);
    let policy = CorrelationPolicy::default();

    let fresh_candidates = Correlator::new(fresh_access)
        .candidates(point, MobilityProfile::Wheeled, &policy)
        .unwrap()
        .into_iter()
        .map(|candidate| candidate.edge_id)
        .collect::<Vec<_>>();
    let package_candidates = Correlator::new(engine.route_graph_view())
        .candidates(point, MobilityProfile::Wheeled, &policy)
        .unwrap()
        .into_iter()
        .map(|candidate| candidate.edge_id)
        .collect::<Vec<_>>();

    assert_eq!(package_candidates, fresh_candidates);
}

#[test]
fn correlator_ranks_nearest_directed_candidates() {
    let graph = graph_from_ways(
        "fork",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.0), (52.001, 21.0)], RoadType::Residential),
        ],
    );
    let in_memory = engine(&graph);
    let access = in_memory.route_graph_view();
    let candidates = Correlator::new(access)
        .candidates(
            (52.00002, 21.0007).into(),
            MobilityProfile::Wheeled,
            &CorrelationPolicy::default(),
        )
        .unwrap();

    assert!(!candidates.is_empty());
    assert_eq!(candidates[0].shared_edge_info_id, 1);
    let other_branch = candidates
        .iter()
        .find(|candidate| candidate.shared_edge_info_id == 2)
        .unwrap();
    assert!(candidates[0].distance_m < other_branch.distance_m);
    assert!(candidates[0].confidence > other_branch.confidence);
}

#[test]
fn indexed_correlation_preserves_nearest_candidate_order() {
    let graph = graph_from_ways(
        "indexed-correlation",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0004, 21.0), (52.0004, 21.001)], RoadType::Track),
            way(
                3,
                vec![(52.001, 21.0), (52.001, 21.001)],
                RoadType::Residential,
            ),
        ],
    );
    let in_memory = engine(&graph);
    let access = in_memory.route_graph_view();
    let candidates = Correlator::new(access)
        .candidates(
            (52.00002, 21.0007).into(),
            MobilityProfile::Wheeled,
            &CorrelationPolicy::default(),
        )
        .unwrap();

    assert_eq!(candidates[0].shared_edge_info_id, 1);
    assert_eq!(candidates[1].shared_edge_info_id, 1);
    assert_eq!(candidates[2].shared_edge_info_id, 2);

    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00098).into(),
        ))
        .unwrap();
    assert_eq!(
        result.diagnostics.correlation_mode,
        CorrelationMode::Indexed
    );
}

#[test]
fn correlation_heading_hint_prefers_aligned_edge_and_records_rejections() {
    let graph = graph_from_ways(
        "heading-correlation",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.0), (52.001, 21.0)], RoadType::Primary),
        ],
    );
    let in_memory = engine(&graph);
    let access = in_memory.route_graph_view();
    let policy = CorrelationPolicy {
        max_distance_m: 20,
        ..CorrelationPolicy::default()
    };
    let endpoint_options = RouteEndpointOptions::default()
        .with_heading(0)
        .with_heading_tolerance(30);

    let search = Correlator::new(access).search(
        (52.0, 21.0).into(),
        MobilityProfile::Wheeled,
        &policy,
        &endpoint_options,
    );

    assert!(!search.candidates.is_empty());
    assert_eq!(search.candidates[0].shared_edge_info_id, 2);
    assert_eq!(search.candidates[0].heading_delta_deg, Some(0));
    assert!(search.rejected_candidate_count > 0);
    assert!(
        search
            .rejected_candidates
            .iter()
            .any(|candidate| candidate.reason == CorrelationRejectReason::HeadingOutsideTolerance)
    );

    let mut request = tight_request((52.0, 21.0).into(), (52.0009, 21.0).into());
    request.origin_options = endpoint_options;
    let result = engine(&graph).route_leg(request).unwrap();
    assert_eq!(result.edge_ids, vec![3]);
    assert!(result.diagnostics.origin_rejected_candidate_count > 0);
    assert!(
        result
            .diagnostics
            .origin_rejected_candidates
            .iter()
            .any(|candidate| candidate.reason == CorrelationRejectReason::HeadingOutsideTolerance)
    );
}

#[test]
fn correlation_layer_and_level_hints_prefer_matching_edges() {
    let graph = graph_from_ways(
        "layer-correlation",
        vec![
            layered_way(
                1,
                vec![(51.999, 21.0), (52.0, 21.0), (52.001, 21.0)],
                0,
                "0",
            ),
            layered_way(
                2,
                vec![(52.0, 20.999), (52.0, 21.0), (52.0, 21.001)],
                1,
                "L1",
            ),
        ],
    );
    let in_memory = engine(&graph);
    let access = in_memory.route_graph_view();
    let policy = CorrelationPolicy {
        max_distance_m: 20,
        candidate_limit: 8,
        ..CorrelationPolicy::default()
    };
    let layer_options = RouteEndpointOptions::default()
        .with_preferred_layer(1)
        .with_preferred_level("L1");

    let search = Correlator::new(access).search(
        (52.0, 21.0).into(),
        MobilityProfile::Wheeled,
        &policy,
        &layer_options,
    );

    assert!(!search.candidates.is_empty());
    assert_eq!(search.candidates[0].layer, Some(1));
    assert_eq!(search.candidates[0].level.as_deref(), Some("L1"));

    let rejected = Correlator::new(access).search(
        (52.0, 21.0).into(),
        MobilityProfile::Wheeled,
        &policy,
        &RouteEndpointOptions::default()
            .with_preferred_layer(2)
            .requiring_layer_match(),
    );
    assert!(rejected.candidates.is_empty());
    assert!(
        rejected
            .rejected_candidates
            .iter()
            .any(
                |candidate| candidate.reason == CorrelationRejectReason::LayerMismatch
                    && candidate.layer.is_some()
            )
    );
}

#[test]
fn correlation_groups_opposing_edges_and_same_named_road_candidates() {
    let graph = graph_from_ways(
        "candidate-groups",
        vec![
            named_ref_way(
                1,
                vec![(52.0, 21.0), (52.0, 21.001)],
                "Main Supply Route",
                "MSR-1",
            ),
            named_ref_way(
                2,
                vec![(52.00012, 21.0), (52.00012, 21.001)],
                "Main Supply Route",
                "MSR-1",
            ),
        ],
    );
    let in_memory = engine(&graph);
    let access = in_memory.route_graph_view();
    let policy = CorrelationPolicy {
        max_distance_m: 40,
        candidate_limit: 8,
        ..CorrelationPolicy::default()
    };
    let candidates = Correlator::new(access)
        .candidates(
            (52.00005, 21.0005).into(),
            MobilityProfile::Wheeled,
            &policy,
        )
        .expect("candidate search succeeds");

    assert_eq!(candidates.len(), 4);
    assert!(
        candidates
            .iter()
            .all(|candidate| candidate.road_group_key.as_deref() == Some("ref:MSR-1"))
    );
    assert!(
        candidates
            .iter()
            .all(|candidate| candidate.same_road_candidate_count == 4)
    );
    assert!(candidates.iter().all(|candidate| {
        candidate
            .opposing_edge_id
            .is_some_and(|opposing_edge_id| opposing_edge_id != candidate.edge_id)
    }));
}

#[test]
fn route_retries_relaxed_correlation_when_strict_heading_rejects_all_candidates() {
    let graph = graph_from_ways(
        "relaxed-correlation-retry",
        vec![way(
            1,
            vec![(52.0, 21.0), (52.0, 21.001)],
            RoadType::Primary,
        )],
    );
    let mut request = tight_request((52.0, 21.00002).into(), (52.0, 21.00098).into());
    request.origin_options = RouteEndpointOptions::default()
        .with_heading(0)
        .with_heading_tolerance(5);

    let result = engine(&graph).route_leg(request).unwrap();

    assert_eq!(result.edge_ids, vec![1]);
    assert_eq!(result.diagnostics.relaxed_correlation_retry_count, 1);
    assert_eq!(result.diagnostics.origin_relaxed_correlation_retry_count, 1);
    assert_eq!(
        result
            .diagnostics
            .destination_relaxed_correlation_retry_count,
        0
    );
    assert!(result.diagnostics.origin_rejected_candidate_count > 0);
    assert!(
        result
            .diagnostics
            .origin_rejected_candidates
            .iter()
            .any(|candidate| candidate.reason == CorrelationRejectReason::HeadingOutsideTolerance)
    );
}

#[test]
fn correlation_component_threshold_filters_tiny_components() {
    let graph = graph_from_ways(
        "component-threshold-correlation",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(
                3,
                vec![(52.00004, 21.0), (52.00004, 21.0002)],
                RoadType::Track,
            ),
        ],
    );
    let in_memory = engine(&graph);
    let access = in_memory.route_graph_view();
    let policy = CorrelationPolicy {
        max_distance_m: 30,
        ..CorrelationPolicy::default()
    };
    let endpoint_options = RouteEndpointOptions::default().with_min_component_size(3);

    let search = Correlator::new(access).search(
        (52.00004, 21.0001).into(),
        MobilityProfile::Wheeled,
        &policy,
        &endpoint_options,
    );

    assert!(!search.candidates.is_empty());
    assert_eq!(search.candidates[0].component_size, Some(3));
    assert!(
        search
            .rejected_candidates
            .iter()
            .any(|candidate| candidate.reason == CorrelationRejectReason::BelowMinComponentSize)
    );
}

#[test]
fn correlation_reach_threshold_filters_short_dead_end_candidates() {
    let graph = graph_from_ways(
        "reach-threshold-correlation",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(
                3,
                vec![(52.00004, 21.0), (52.00004, 21.0002)],
                RoadType::Residential,
            ),
        ],
    );
    let in_memory = engine(&graph);
    let access = in_memory.route_graph_view();
    let policy = CorrelationPolicy {
        max_distance_m: 30,
        ..CorrelationPolicy::default()
    };
    let endpoint_options = RouteEndpointOptions::default().with_min_outbound_reach_edge_count(3);

    let search = Correlator::new(access).search(
        (52.00004, 21.0001).into(),
        MobilityProfile::Wheeled,
        &policy,
        &endpoint_options,
    );

    assert!(!search.candidates.is_empty());
    assert_eq!(search.candidates[0].shared_edge_info_id, 1);
    assert!(
        search.candidates[0]
            .outbound_reach_edge_count
            .is_some_and(|count| count >= 3)
    );
    let rejected = search
        .rejected_candidates
        .iter()
        .find(|candidate| candidate.reason == CorrelationRejectReason::BelowMinReach)
        .expect("short dead-end candidate should be rejected by reach threshold");
    assert_eq!(rejected.edge_id, Some(5));
    assert_eq!(rejected.outbound_reach_edge_count, Some(2));

    let mut request = tight_request((52.00004, 21.0001).into(), (52.0, 21.0019).into());
    request.origin_options = endpoint_options;
    let result = engine(&graph).route_leg(request).unwrap();
    assert!(
        result
            .diagnostics
            .origin_best_outbound_reach_edge_count
            .is_some_and(|count| count >= 3)
    );
}

#[test]
fn route_geometry_starts_and_ends_at_snapped_points() {
    let graph = graph_from_ways(
        "snapped-shape",
        vec![way(
            1,
            vec![(52.0, 21.0), (52.0, 21.002)],
            RoadType::Primary,
        )],
    );

    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.0005).into(),
            (52.0, 21.0015).into(),
        ))
        .unwrap();

    assert_eq!(result.edge_ids, vec![1]);
    assert_point_close(result.geometry[0], result.origin.snapped_point);
    assert_point_close(
        *result.geometry.last().unwrap(),
        result.destination.snapped_point,
    );
    assert_point_close(result.legs[0].geometry[0], result.origin.snapped_point);
    assert_point_close(
        *result.legs[0].geometry.last().unwrap(),
        result.destination.snapped_point,
    );
    assert_eq!(result.legs[0].shape_segments.len(), 1);
    let segment = &result.legs[0].shape_segments[0];
    assert_eq!(segment.edge_id, 1);
    assert_eq!(segment.begin_shape_index, 0);
    assert_eq!(segment.end_shape_index, result.geometry.len() - 1);
    assert!(segment.start_fraction > 0.0);
    assert!(segment.end_fraction < 1.0);
    assert!(
        segment
            .quality_flags
            .contains(&RouteShapeQualityFlag::ClippedOriginEdge)
    );
    assert!(
        segment
            .quality_flags
            .contains(&RouteShapeQualityFlag::ClippedDestinationEdge)
    );
    assert!(
        segment
            .quality_flags
            .contains(&RouteShapeQualityFlag::SameEdgeDirect)
    );
    assert!(result.distance_m > 60.0);
    assert!(result.distance_m < 80.0);
}

#[test]
fn guidance_suppresses_short_connector_maneuver_before_real_turn() {
    let graph = graph_from_ways(
        "guidance-short-connector",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(
                2,
                vec![(52.0, 21.001), (52.0, 21.00105)],
                RoadType::Residential,
            ),
            way(
                3,
                vec![(52.0, 21.00105), (52.001, 21.00105)],
                RoadType::Primary,
            ),
        ],
    );

    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.0001).into(),
            (52.0009, 21.00105).into(),
        ))
        .unwrap();

    assert_eq!(result.edge_ids, vec![1, 3, 5]);
    assert!(
        !result
            .maneuvers
            .iter()
            .any(|maneuver| maneuver.road_name.as_deref() == Some("Way 2"))
    );
    let turn = result
        .maneuvers
        .iter()
        .find(|maneuver| maneuver.kind == RouteManeuverKind::Turn)
        .expect("real turn maneuver is preserved");
    assert_eq!(turn.road_name.as_deref(), Some("Way 3"));
    assert!(
        turn.quality_flags
            .contains(&RouteGuidanceQualityFlag::CombinedShortConnector)
    );
    assert_eq!(
        turn.instruction_parameters
            .get("suppressed_connector_name")
            .map(String::as_str),
        Some("Way 2")
    );
    assert_eq!(
        turn.instruction_parameters
            .get("suppressed_connector_reason")
            .map(String::as_str),
        Some("short_connector")
    );
}

#[test]
fn route_prefers_lower_cost_fork_and_builds_leg_attribution() {
    let graph = graph_from_ways(
        "fork",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(3, vec![(52.0, 21.0), (52.0008, 21.001)], RoadType::Track),
            way(4, vec![(52.0008, 21.001), (52.0, 21.002)], RoadType::Track),
        ],
    );
    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00198).into(),
        ))
        .unwrap();

    assert_eq!(result.status, RouteStatus::Routed);
    assert_eq!(result.edge_ids, vec![1, 3]);
    assert_eq!(result.legs.len(), 1);
    assert_eq!(result.legs[0].edges.len(), 2);
    assert_eq!(result.legs[0].edges[0].road_type, RoadType::Primary);
    assert_eq!(result.maneuvers[0].kind, RouteManeuverKind::Start);
    assert_eq!(
        result.maneuvers.last().unwrap().kind,
        RouteManeuverKind::Destination
    );
    assert!(result.distance_m > 120.0);
    assert!(result.duration_s > 0.0);
    assert!(result.diagnostics.visited_edge_count > 0);
}

#[test]
fn alternatives_return_lower_overlap_fork_candidate() {
    // Phase 0 time metric: on a very small fixture a single turn penalty (now in
    // seconds) can dwarf the few seconds of travel and distort the cost ratio.
    // The geometry is scaled ~10x so travel time dominates and the fork's cost
    // ratio reflects the intended geometric detour (as under the old metric).
    let graph = graph_from_ways(
        "alternatives",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.01)], RoadType::Primary),
            way(2, vec![(52.0, 21.01), (52.0, 21.02)], RoadType::Primary),
            way(3, vec![(52.0, 21.0), (52.008, 21.01)], RoadType::Secondary),
            way(4, vec![(52.008, 21.01), (52.0, 21.02)], RoadType::Secondary),
        ],
    );
    let mut request = tight_request((52.0, 21.0002).into(), (52.0, 21.0198).into());
    request.options.alternatives.requested_alternative_count = 1;
    request.options.alternatives.max_shared_edge_ratio = 0.2;
    request.options.alternatives.max_cost_ratio = 3.0;

    let result = engine(&graph).route_leg(request).unwrap();

    assert_eq!(result.edge_ids, vec![1, 3]);
    assert_eq!(result.alternatives.len(), 1);
    let alternative = &result.alternatives[0];
    // The fork over ways 3 and 4, reached along the origin's and the
    // destination's own roads (2, 4) rather than by snapping onto way 3 and
    // off way 4 at their far-off ends.
    assert_eq!(alternative.edge_ids, vec![2, 5, 7, 4]);
    assert!(alternative.shared_edge_ratio <= 0.2);
    assert!(alternative.quality_score > 0.0);
    assert!(
        alternative
            .quality_reasons
            .iter()
            .any(|reason| reason == "low_shared_path")
    );
    assert!(!alternative.legs[0].edges.is_empty());
    assert!(alternative.diagnostics.final_path_edge_count > 0);
    assert_eq!(result.diagnostics.alternative_attempt_count, 1);
    assert_eq!(result.diagnostics.accepted_alternative_count, 1);
    assert_eq!(result.diagnostics.rejected_alternative_duplicate_count, 0);
    assert_eq!(result.diagnostics.rejected_alternative_shared_edge_count, 0);
}

#[test]
fn alternatives_report_duplicate_rejection_when_only_primary_path_exists() {
    let graph = graph_from_ways(
        "alternative-duplicate-rejection",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
        ],
    );
    let mut request = tight_request((52.0, 21.00002).into(), (52.0, 21.00198).into());
    request.options.alternatives.requested_alternative_count = 1;

    let result = engine(&graph).route_leg(request).unwrap();

    assert!(result.alternatives.is_empty());
    assert!(result.diagnostics.alternative_attempt_count > 0);
    assert_eq!(result.diagnostics.accepted_alternative_count, 0);
    assert!(result.diagnostics.rejected_alternative_duplicate_count > 0);
}

#[test]
fn matrix_returns_reachable_and_unreachable_cells_over_fixture() {
    let graph = graph_from_ways(
        "matrix",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(3, vec![(52.01, 21.01), (52.01, 21.011)], RoadType::Primary),
        ],
    );

    let mut request = RouteMatrixRequest::new(
        vec![(52.0, 21.00002).into()],
        vec![(52.0, 21.00198).into(), (52.01, 21.01098).into()],
        MobilityProfile::Wheeled,
    );
    request.options.max_correlation_distance_m = 20;
    let result = engine(&graph).matrix(request);

    assert_eq!(result.source_count, 1);
    assert_eq!(result.target_count, 2);
    assert_eq!(result.source_expansion_count, 1);
    assert!(result.visited_edge_count > 0);
    assert!(result.settled_node_count > 0);
    assert_eq!(result.cells.len(), 2);
    assert_eq!(result.cells[0].status, RouteMatrixCellStatus::Reachable);
    assert_eq!(result.cells[0].edge_ids, vec![1, 3]);
    assert!(result.cells[0].distance_m.unwrap() > 120.0);
    assert_eq!(result.cells[1].status, RouteMatrixCellStatus::Unreachable);
    assert_eq!(
        result.cells[1].failure_reason.as_deref(),
        Some("disconnected")
    );
}

#[test]
fn package_matrix_matches_in_memory_engine() {
    let graph = graph_from_ways(
        "compact-v3-matrix-parity",
        vec![
            way(1, vec![(52.0, 21.019), (52.0, 21.0198)], RoadType::Primary),
            way(2, vec![(52.0, 21.0198), (52.0, 21.0202)], RoadType::Primary),
            way(3, vec![(52.0, 21.0202), (52.0, 21.021)], RoadType::Primary),
            way(4, vec![(52.01, 21.01), (52.01, 21.011)], RoadType::Primary),
        ],
    );
    let mut request = RouteMatrixRequest::new(
        vec![(52.0, 21.01902).into()],
        vec![(52.0, 21.02098).into(), (52.01, 21.01098).into()],
        MobilityProfile::Wheeled,
    );
    request.options.max_correlation_distance_m = 20;
    let expected = engine(&graph).matrix(request.clone());
    assert!(!expected.cells[0].edge_ids.is_empty());
    let (_dir, package) = persisted_package(&graph);

    let actual = RoutingEngine::from_package(package).matrix(request);

    assert_eq!(actual, expected);
}

#[test]
fn matrix_keeps_distinct_incoming_edge_states_at_same_node() {
    let mut graph = graph_from_ways(
        "matrix-edge-state-turn-restriction",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(
                2,
                vec![(52.0, 21.0), (52.00035, 21.0005)],
                RoadType::Primary,
            ),
            way(
                3,
                vec![(52.00035, 21.0005), (52.0, 21.001)],
                RoadType::Primary,
            ),
            way(4, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
        ],
    );
    restrict_transition(&mut graph, 1, 7, 1);

    let mut request = RouteMatrixRequest::new(
        vec![(52.0, 21.00002).into()],
        vec![(52.0, 21.00198).into()],
        MobilityProfile::Wheeled,
    );
    request.options.max_correlation_distance_m = 20;

    let result = engine(&graph).matrix(request);

    assert_eq!(result.cells.len(), 1);
    assert_eq!(result.cells[0].status, RouteMatrixCellStatus::Reachable);
    assert_eq!(result.cells[0].edge_ids, vec![3, 5, 7]);
}

#[test]
fn matrix_uses_link_transition_relief() {
    let origin = (52.0, 21.00002).into();
    let destination = (52.00098, 21.001).into();
    let normal_graph = graph_from_ways(
        "matrix-normal-transition-cost",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.001, 21.001)], RoadType::Primary),
        ],
    );
    let mut link_edge = way(2, vec![(52.0, 21.001), (52.001, 21.001)], RoadType::Primary);
    link_edge.is_link = true;
    let link_graph = graph_from_ways(
        "matrix-link-transition-cost",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            link_edge,
        ],
    );
    let mut request =
        RouteMatrixRequest::new(vec![origin], vec![destination], MobilityProfile::Wheeled);
    request.options.max_correlation_distance_m = 20;

    let normal = engine(&normal_graph).matrix(request.clone());
    let link = engine(&link_graph).matrix(request);

    assert_eq!(normal.cells[0].status, RouteMatrixCellStatus::Reachable);
    assert_eq!(link.cells[0].status, RouteMatrixCellStatus::Reachable);
    assert_eq!(normal.cells[0].edge_ids, link.cells[0].edge_ids);
    // Phase 0 time metric: turn penalties now in seconds, so the link relief gap
    // is ~2.1 s (legacy 17.5-unit relief converted at the 30 km/h reference).
    assert!(
        normal.cells[0].cost.unwrap() > link.cells[0].cost.unwrap() + 2.0,
        "matrix should use record-aware link transition relief: normal={:?}, link={:?}",
        normal.cells[0].cost,
        link.cells[0].cost
    );
}

#[test]
fn matrix_honors_active_conditional_access_closure() {
    let graph = graph_from_ways(
        "matrix-conditional-access-route",
        vec![
            conditionally_closed_primary(1, vec![(52.0, 21.0), (52.0, 21.002)]),
            way(2, vec![(52.0, 21.0), (52.0008, 21.001)], RoadType::Primary),
            way(
                3,
                vec![(52.0008, 21.001), (52.0, 21.002)],
                RoadType::Primary,
            ),
        ],
    );
    let mut active_request = RouteMatrixRequest::new(
        vec![(52.0, 21.00002).into()],
        vec![(52.0, 21.00198).into()],
        MobilityProfile::Wheeled,
    );
    active_request.options.max_correlation_distance_m = 20;
    // 2026-05-15T08:00:00Z is inside the conditional closure window.
    active_request
        .options
        .costing
        .conditional_restriction_eval_at_ms = Some(1_778_832_000_000);

    let active = engine(&graph).matrix(active_request);

    assert_eq!(active.cells[0].status, RouteMatrixCellStatus::Reachable);
    assert_eq!(active.cells[0].edge_ids, vec![3, 5]);

    let mut inactive_request = RouteMatrixRequest::new(
        vec![(52.0, 21.00002).into()],
        vec![(52.0, 21.00198).into()],
        MobilityProfile::Wheeled,
    );
    inactive_request.options.max_correlation_distance_m = 20;
    // 2026-07-01T08:00:00Z is outside the conditional closure window.
    inactive_request
        .options
        .costing
        .conditional_restriction_eval_at_ms = Some(1_782_892_800_000);

    let inactive = engine(&graph).matrix(inactive_request);

    assert_eq!(inactive.cells[0].status, RouteMatrixCellStatus::Reachable);
    assert_eq!(inactive.cells[0].edge_ids, vec![1]);
}

#[test]
fn matrix_honors_active_conditional_maxspeed_window() {
    let graph = graph_from_ways(
        "matrix-conditional-maxspeed",
        vec![conditionally_slow_primary(
            1,
            vec![(52.0, 21.0), (52.0, 21.002)],
        )],
    );
    let mut active_request = RouteMatrixRequest::new(
        vec![(52.0, 21.00002).into()],
        vec![(52.0, 21.00198).into()],
        MobilityProfile::Wheeled,
    );
    active_request.options.max_correlation_distance_m = 20;
    // 2026-05-19T16:30:00Z is inside the afternoon conditional maxspeed window.
    active_request
        .options
        .costing
        .conditional_restriction_eval_at_ms = Some(1_779_208_200_000);

    let mut inactive_request = active_request.clone();
    // 2026-05-19T12:00:00Z is outside both listed maxspeed windows.
    inactive_request
        .options
        .costing
        .conditional_restriction_eval_at_ms = Some(1_779_192_000_000);

    let active = engine(&graph).matrix(active_request);
    let inactive = engine(&graph).matrix(inactive_request);

    assert_eq!(active.cells[0].status, RouteMatrixCellStatus::Reachable);
    assert_eq!(inactive.cells[0].status, RouteMatrixCellStatus::Reachable);
    assert!(
        active.cells[0].duration_s.unwrap() > inactive.cells[0].duration_s.unwrap() * 2.0,
        "conditional maxspeed should increase matrix travel time: active={:?}, inactive={:?}",
        active.cells[0].duration_s,
        inactive.cells[0].duration_s
    );
}

#[test]
fn isochrone_returns_reachable_edge_buckets_by_cost_contour() {
    let graph = graph_from_ways(
        "isochrone",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(3, vec![(52.0, 21.002), (52.0, 21.003)], RoadType::Primary),
        ],
    );

    let mut request = RouteIsochroneRequest::new(
        (52.0, 21.00002).into(),
        MobilityProfile::Wheeled,
        // Phase 0 time metric: cost is now travel time (seconds), not distance,
        // so the contour thresholds are re-scaled to the time domain. These
        // reproduce the same reachability structure as the old distance contours
        // (160/260): the first contour reaches edges 1/2/3 fully plus a partial
        // fragment on edge 5; the second reaches edge 5 fully.
        vec![
            RouteIsochroneContour {
                contour: 7.2,
                metric: RouteIsochroneMetric::Cost,
            },
            RouteIsochroneContour {
                contour: 10.0,
                metric: RouteIsochroneMetric::Cost,
            },
        ],
    );
    request.options.max_correlation_distance_m = 15;
    let result = engine(&graph).isochrone(request).unwrap();

    assert_eq!(result.buckets.len(), 2);
    assert_eq!(result.buckets[0].edge_ids, vec![1, 2, 3]);
    assert_eq!(result.buckets[0].boundary_fragments.len(), 1);
    let fragment = &result.buckets[0].boundary_fragments[0];
    assert_eq!(fragment.edge_id, 5);
    assert_eq!(fragment.start_fraction, 0.0);
    assert!(fragment.end_fraction > 0.0);
    assert!(fragment.end_fraction < 1.0);
    assert!(fragment.end.distance_to((52.0, 21.00235).into()) < 12.0);
    assert_eq!(result.buckets[1].edge_ids, vec![1, 2, 3, 5]);
    assert!(result.buckets[1].boundary_fragments.is_empty());
    assert!(result.visited_edge_count > 0);
    assert!(result.settled_node_count > 0);
}

#[test]
fn package_isochrone_matches_in_memory_engine() {
    let graph = graph_from_ways(
        "compact-v3-isochrone-parity",
        vec![
            way(1, vec![(52.0, 21.019), (52.0, 21.0198)], RoadType::Primary),
            way(2, vec![(52.0, 21.0198), (52.0, 21.0202)], RoadType::Primary),
            way(3, vec![(52.0, 21.0202), (52.0, 21.021)], RoadType::Primary),
        ],
    );
    let mut request = RouteIsochroneRequest::new(
        (52.0, 21.01902).into(),
        MobilityProfile::Wheeled,
        vec![
            RouteIsochroneContour {
                contour: 160.0,
                metric: RouteIsochroneMetric::Cost,
            },
            RouteIsochroneContour {
                contour: 260.0,
                metric: RouteIsochroneMetric::Cost,
            },
        ],
    );
    request.options.max_correlation_distance_m = 15;
    let expected = engine(&graph).isochrone(request.clone()).unwrap();
    let (_dir, package) = persisted_package(&graph);

    let actual = RoutingEngine::from_package(package)
        .isochrone(request)
        .unwrap();
    assert_eq!(actual.buckets, expected.buckets);
    assert_eq!(actual.origin, expected.origin);
}

#[test]
fn package_isochrone_returns_a_boundary_fragment() {
    let graph = graph_from_ways(
        "compact-v3-isochrone-boundary-fragment",
        vec![
            way(1, vec![(52.0, 21.019), (52.0, 21.020)], RoadType::Primary),
            way(2, vec![(52.0, 21.020), (52.0, 21.021)], RoadType::Primary),
            way(3, vec![(52.0, 21.021), (52.0, 21.022)], RoadType::Primary),
        ],
    );
    let mut request = RouteIsochroneRequest::new(
        (52.0, 21.01902).into(),
        MobilityProfile::Wheeled,
        // Phase 0 time metric: cost is travel time (seconds); a contour of ~4.5 s
        // crosses inside an edge on this ~3 s/edge chain (old distance contour 160).
        vec![RouteIsochroneContour {
            contour: 4.5,
            metric: RouteIsochroneMetric::Cost,
        }],
    );
    request.options.max_correlation_distance_m = 15;
    let expected = engine(&graph).isochrone(request.clone()).unwrap();
    assert!(
        expected
            .buckets
            .iter()
            .any(|bucket| !bucket.boundary_fragments.is_empty()),
        "graph fixture must cross the contour within an edge"
    );
    let (_dir, package) = persisted_package(&graph);
    let engine = RoutingEngine::from_package(package);

    let result = engine.isochrone(request).unwrap();

    assert_eq!(result.buckets, expected.buckets);
}

#[test]
fn package_isochrone_preserves_curved_boundary_fragment_geometry() {
    let graph = graph_from_ways(
        "compact-v3-isochrone-curved-boundary-fragment",
        vec![one_way(
            1,
            vec![(52.0, 21.019), (52.001, 21.020), (52.0, 21.021)],
        )],
    );
    let mut request = RouteIsochroneRequest::new(
        (52.0, 21.019).into(),
        MobilityProfile::Wheeled,
        // Phase 0 time metric: cost is travel time (seconds) times the road
        // type's factor (residential 1.2 at 35 km/h);
        // this contour reaches ~62% along the curved edge (past its apex),
        // matching the boundary the old distance contour 160 produced on this
        // ~260 m edge.
        vec![RouteIsochroneContour {
            contour: 20.0,
            metric: RouteIsochroneMetric::Cost,
        }],
    );
    request.options.max_correlation_distance_m = 15;
    let expected = engine(&graph).isochrone(request.clone()).unwrap();
    let expected_fragment = expected.buckets[0]
        .boundary_fragments
        .first()
        .expect("curved fixture must cross the contour inside its curved edge");
    assert!(
        expected_fragment.end.lat > 52.0005,
        "fixture must distinguish curved geometry from the endpoint chord"
    );
    let (_dir, package) = persisted_package(&graph);

    let actual = RoutingEngine::from_package(package)
        .isochrone(request)
        .unwrap();

    assert_eq!(actual.buckets, expected.buckets);
}

#[test]
fn isochrone_keeps_distinct_incoming_edge_states_at_same_node() {
    let mut graph = graph_from_ways(
        "isochrone-edge-state-turn-restriction",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(
                3,
                vec![(52.0, 21.001), (52.00035, 21.0015)],
                RoadType::Primary,
            ),
            way(
                4,
                vec![(52.00035, 21.0015), (52.0, 21.002)],
                RoadType::Primary,
            ),
            way(5, vec![(52.0, 21.002), (52.0, 21.003)], RoadType::Primary),
        ],
    );
    restrict_transition(&mut graph, 3, 9, 1);

    let mut request = RouteIsochroneRequest::new(
        (52.0, 21.00002).into(),
        MobilityProfile::Wheeled,
        vec![RouteIsochroneContour {
            contour: 400.0,
            metric: RouteIsochroneMetric::Cost,
        }],
    );
    request.options.max_correlation_distance_m = 20;

    let result = engine(&graph).isochrone(request).unwrap();

    assert_eq!(result.buckets.len(), 1);
    assert!(result.buckets[0].edge_ids.contains(&9));
}

#[test]
fn isochrone_uses_link_transition_relief_for_boundary_fragments() {
    let origin = (52.0, 21.00002).into();
    // Phase 0 time metric: turn penalties are now in seconds and are large
    // relative to short edges, so the second (post-turn) edge is lengthened
    // (~666 m) so that BOTH the normal and the (cheaper) link turn leave the
    // cost contour crossing inside it, exposing the relief as a farther boundary.
    let normal_graph = graph_from_ways(
        "isochrone-normal-transition-cost",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.006, 21.001)], RoadType::Primary),
        ],
    );
    let mut link_edge = way(2, vec![(52.0, 21.001), (52.006, 21.001)], RoadType::Primary);
    link_edge.is_link = true;
    let link_graph = graph_from_ways(
        "isochrone-link-transition-cost",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            link_edge,
        ],
    );
    let mut request = RouteIsochroneRequest::new(
        origin,
        MobilityProfile::Wheeled,
        vec![RouteIsochroneContour {
            contour: 25.0,
            metric: RouteIsochroneMetric::Cost,
        }],
    );
    request.options.max_correlation_distance_m = 20;

    let normal = engine(&normal_graph).isochrone(request.clone()).unwrap();
    let link = engine(&link_graph).isochrone(request).unwrap();
    let normal_fragment = normal.buckets[0]
        .boundary_fragments
        .iter()
        .find(|fragment| fragment.edge_id == 3)
        .expect("normal second-edge boundary fragment");
    let link_fragment = link.buckets[0]
        .boundary_fragments
        .iter()
        .find(|fragment| fragment.edge_id == 3)
        .expect("link second-edge boundary fragment");

    assert!(
        link_fragment.end_fraction > normal_fragment.end_fraction,
        "lower link transition cost should push the cost-contour boundary farther along the next edge"
    );
}

#[test]
fn isochrone_honors_active_conditional_access_closure() {
    let graph = graph_from_ways(
        "isochrone-conditional-access-route",
        vec![
            conditionally_closed_primary(1, vec![(52.0, 21.0), (52.0, 21.002)]),
            way(2, vec![(52.0, 21.0), (52.0008, 21.001)], RoadType::Primary),
            way(
                3,
                vec![(52.0008, 21.001), (52.0, 21.002)],
                RoadType::Primary,
            ),
        ],
    );
    let mut active_request = RouteIsochroneRequest::new(
        (52.0, 21.00002).into(),
        MobilityProfile::Wheeled,
        vec![RouteIsochroneContour {
            contour: 1_000.0,
            metric: RouteIsochroneMetric::Cost,
        }],
    );
    active_request.options.max_correlation_distance_m = 20;
    // 2026-05-15T08:00:00Z is inside the conditional closure window.
    active_request
        .options
        .costing
        .conditional_restriction_eval_at_ms = Some(1_778_832_000_000);

    let active = engine(&graph).isochrone(active_request).unwrap();

    assert_eq!(active.buckets.len(), 1);
    assert!(
        !active.buckets[0].edge_ids.contains(&1),
        "active conditional closure must not appear reachable"
    );
    assert!(
        active.buckets[0].edge_ids.contains(&3),
        "open bypass edge should remain reachable"
    );

    let mut inactive_request = RouteIsochroneRequest::new(
        (52.0, 21.00002).into(),
        MobilityProfile::Wheeled,
        vec![RouteIsochroneContour {
            contour: 1_000.0,
            metric: RouteIsochroneMetric::Cost,
        }],
    );
    inactive_request.options.max_correlation_distance_m = 20;
    // 2026-07-01T08:00:00Z is outside the conditional closure window.
    inactive_request
        .options
        .costing
        .conditional_restriction_eval_at_ms = Some(1_782_892_800_000);

    let inactive = engine(&graph).isochrone(inactive_request).unwrap();

    assert!(
        inactive.buckets[0].edge_ids.contains(&1),
        "inactive conditional closure should allow the direct edge"
    );
}

#[test]
fn route_options_json_with_former_search_settings_still_deserializes() {
    let graph = graph_from_ways(
        "former-search-settings",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
        ],
    );
    let origin = (52.0, 21.00002).into();
    let destination = (52.0, 21.00198).into();
    let mut json = serde_json::to_value(tight_request(origin, destination).options).unwrap();
    // Stored JSON from before 2026-10-08 sets the former search's knobs.
    json["algorithm"] = serde_json::json!("bidirectional_a_star");
    json["algorithm_options"] = serde_json::json!({
        "autoBidirectionalMinStraightLineDistanceM": 1,
        "autoDirectedForDynamicPolicy": false,
        "autoDirectedForShortcuts": false,
    });
    json["bidirectional"] = serde_json::json!({
        "maxForwardVisitedEdgeCount": 1,
        "maxReverseVisitedEdgeCount": 1,
        "maxCombinedVisitedEdgeCount": 1,
    });
    json["hierarchy"] = serde_json::json!({ "enabled": true });
    json["shortcuts"] = serde_json::json!({ "enabled": true, "minEdgeCount": 2 });
    let options: wayfarer::RouteOptions = serde_json::from_value(json).unwrap();

    let engine = engine(&graph);
    let plain = engine
        .route_leg(tight_request(origin, destination))
        .unwrap();
    let mut request = tight_request(origin, destination);
    request.options = options;
    let route = engine.route_leg(request).unwrap();
    assert_eq!(route.edge_ids, plain.edge_ids);
    assert_eq!(route.cost, plain.cost);
}

#[test]
fn edge_cost_multipliers_apply_only_to_the_edges_they_name() {
    let graph = graph_from_ways(
        "bidirectional-best-reverse-connection",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.003)], RoadType::Path),
            way(
                3,
                vec![(52.0, 21.001), (52.00035, 21.002)],
                RoadType::Primary,
            ),
            way(
                4,
                vec![(52.00035, 21.002), (52.0, 21.003)],
                RoadType::Primary,
            ),
            way(5, vec![(52.0, 21.003), (52.0, 21.004)], RoadType::Primary),
        ],
    );
    let origin = (52.0, 21.00002).into();
    let destination = (52.0, 21.00398).into();
    let mut directed_request = tight_request(origin, destination);
    directed_request
        .options
        .edge_cost_multipliers
        .insert(3, 50.0);
    let unused_branch = engine(&graph).route_leg(directed_request).unwrap();
    let plain = engine(&graph)
        .route_leg(tight_request(origin, destination))
        .unwrap();
    let mut penalised_request = tight_request(origin, destination);
    penalised_request
        .options
        .edge_cost_multipliers
        .insert(5, 50.0);
    let penalised = engine(&graph).route_leg(penalised_request).unwrap();

    // A multiplier on the branch the route does not take changes nothing; on
    // the branch it takes, it makes the route dearer.
    assert_eq!(unused_branch.edge_ids, vec![1, 5, 7, 9]);
    assert_eq!(unused_branch.edge_ids, plain.edge_ids);
    assert_eq!(unused_branch.cost, plain.cost);
    assert!(penalised.cost > plain.cost);
}

#[test]
fn in_memory_and_package_engines_route_identically() {
    let graph = graph_from_ways(
        "compact-v3-point-search-local-refs",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.01)], RoadType::Primary),
            way(2, vec![(52.0, 21.01), (52.0, 21.02)], RoadType::Primary),
            way(3, vec![(52.0, 21.02), (52.0, 21.03)], RoadType::Primary),
            way(4, vec![(52.0, 21.03), (52.0, 21.04)], RoadType::Primary),
        ],
    );
    let (_dir, package) = persisted_package(&graph);

    let request = tight_request((52.0, 21.00002).into(), (52.0, 21.03998).into());
    let graph_result = engine(&graph).route_leg(request.clone()).unwrap();
    let tile_result = RoutingEngine::from_package(package)
        .route_leg(request)
        .unwrap();

    assert_eq!(tile_result.edge_ids, graph_result.edge_ids);
    assert_eq!(tile_result.cost, graph_result.cost);
    assert_eq!(tile_result.legs, graph_result.legs);
}

#[test]
fn regional_route_set_example_uses_named_bucket_contract() {
    let route_set_path =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/malopolskie.example.json");
    let raw = std::fs::read_to_string(route_set_path).expect("read route set example");
    let routes: serde_json::Value = serde_json::from_str(&raw).expect("valid route set json");
    let routes = routes.as_array().expect("route set array");

    assert!(!routes.is_empty());
    for route in routes {
        assert!(
            route
                .get("label")
                .and_then(serde_json::Value::as_str)
                .is_some()
        );
        assert!(
            route
                .pointer("/origin/lat")
                .and_then(serde_json::Value::as_f64)
                .is_some()
        );
        assert!(
            route
                .pointer("/origin/lng")
                .and_then(serde_json::Value::as_f64)
                .is_some()
        );
        assert!(
            route
                .pointer("/destination/lat")
                .and_then(serde_json::Value::as_f64)
                .is_some()
        );
        assert!(
            route
                .pointer("/destination/lng")
                .and_then(serde_json::Value::as_f64)
                .is_some()
        );
        assert!(matches!(
            route.get("bucket").and_then(serde_json::Value::as_str),
            Some("local" | "city" | "regional" | "long_regional")
        ));
    }
}

#[test]
fn route_leg_attribution_preserves_road_graph_facts() {
    let mut graph = graph_from_ways(
        "route-attribution",
        vec![way_with_road_facts(1, vec![(52.0, 21.0), (52.0, 21.001)])],
    );
    let mut facts = road_way_facts(1, RoadType::Track);
    facts.ref_name = Some("MSR-7".to_string());
    facts.surface = Some("gravel".to_string());
    facts.tracktype = Some("grade2".to_string());
    facts.smoothness = Some("intermediate".to_string());
    facts.lanes = Some("2".to_string());
    facts.layer = Some("1".to_string());
    facts.level = Some("L1".to_string());
    facts.destination = Some("Forward Base".to_string());
    facts.destination_ref = Some("MSR-7".to_string());
    facts.destination_street = Some("Supply Road".to_string());
    facts.junction_ref = Some("7A".to_string());
    facts.admin_country_id = Some("PL".to_string());
    facts.admin_region_id = Some("malopolskie".to_string());
    facts.timezone_id = Some("Europe/Warsaw".to_string());
    facts.has_bridge = true;
    facts.has_tunnel = true;
    facts.has_ford = true;
    facts.is_toll = true;
    facts.has_gate = true;
    facts.is_access_restricted = true;
    facts.is_destination_only = true;
    graph.set_way_facts(vec![facts]);
    graph.refresh_shared_edge_metadata();

    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00098).into(),
        ))
        .unwrap();
    let edge = &result.legs[0].edges[0];

    assert_eq!(edge.ref_name.as_deref(), Some("MSR-7"));
    assert_eq!(edge.surface.as_deref(), Some("gravel"));
    assert_eq!(edge.tracktype.as_deref(), Some("grade2"));
    assert_eq!(edge.smoothness.as_deref(), Some("intermediate"));
    assert_eq!(edge.lane_count, Some(2));
    assert_eq!(edge.layer, Some(1));
    assert_eq!(edge.level.as_deref(), Some("L1"));
    assert_eq!(edge.destination.as_deref(), Some("Forward Base"));
    assert_eq!(edge.destination_ref.as_deref(), Some("MSR-7"));
    assert_eq!(edge.destination_street.as_deref(), Some("Supply Road"));
    assert_eq!(edge.junction_ref.as_deref(), Some("7A"));
    assert_eq!(edge.admin_country_id.as_deref(), Some("PL"));
    assert_eq!(edge.admin_region_id.as_deref(), Some("malopolskie"));
    assert_eq!(edge.timezone_id.as_deref(), Some("Europe/Warsaw"));
    assert!(edge.is_bridge);
    assert!(edge.is_tunnel);
    assert!(edge.has_ford);
    assert!(edge.is_toll);
    assert!(edge.has_gate);
    assert!(edge.is_access_restricted);
    assert!(edge.is_destination_only);
    assert_eq!(
        result.maneuvers[0].sign_text.as_deref(),
        Some("Forward Base")
    );
    assert_eq!(result.maneuvers[0].branch_text.as_deref(), Some("MSR-7"));
    assert_eq!(
        result.maneuvers[0]
            .instruction_parameters
            .get("sign_text")
            .map(String::as_str),
        Some("Forward Base")
    );
    assert_eq!(
        result.maneuvers[0]
            .instruction_parameters
            .get("destination_street")
            .map(String::as_str),
        Some("Supply Road")
    );
    assert_eq!(
        result.maneuvers[0]
            .instruction_parameters
            .get("junction_ref")
            .map(String::as_str),
        Some("7A")
    );
    assert!(edge.cost_breakdown.ford_factor > 1.0);
    assert!(result.diagnostics.ford_penalty_count > 0);
}

#[test]
fn costing_options_can_avoid_destination_only_edges() {
    let graph = graph_from_ways(
        "destination-costing",
        vec![
            destination_only_primary(1, vec![(52.0, 21.0), (52.0, 21.001)]),
            destination_only_primary(2, vec![(52.0, 21.001), (52.0, 21.002)]),
            way(3, vec![(52.0, 21.0), (52.0008, 21.001)], RoadType::Primary),
            way(
                4,
                vec![(52.0008, 21.001), (52.0, 21.002)],
                RoadType::Primary,
            ),
        ],
    );
    let mut request = tight_request((52.0, 21.00002).into(), (52.0, 21.00198).into());
    request.options.costing.avoid_destination_only = 1.0;

    let result = engine(&graph).route_leg(request).unwrap();

    assert_eq!(result.edge_ids, vec![5, 7]);
    assert_eq!(result.diagnostics.destination_only_penalty_count, 0);
    assert_eq!(result.diagnostics.destination_only_relaxed_retry_count, 0);
    assert!(
        result
            .legs
            .iter()
            .flat_map(|leg| &leg.edges)
            .all(|edge| !edge.is_destination_only)
    );
}

#[test]
fn route_on_one_destination_only_road_needs_no_retry() {
    let graph = graph_from_ways(
        "destination-only-pocket",
        vec![
            destination_only_primary(1, vec![(52.0, 21.0), (52.0, 21.001)]),
            destination_only_primary(2, vec![(52.0, 21.001), (52.0, 21.002)]),
        ],
    );
    let request = tight_request((52.0, 21.00002).into(), (52.0, 21.00198).into());

    let result = engine(&graph).route_leg(request).unwrap();

    assert_eq!(result.edge_ids, vec![1, 3]);
    assert_eq!(result.diagnostics.destination_only_relaxed_retry_count, 0);
    assert!(
        result
            .legs
            .iter()
            .flat_map(|leg| &leg.edges)
            .all(|edge| edge.is_destination_only)
    );
}

#[test]
fn route_retries_with_destination_only_relaxed_when_only_a_through_road_connects() {
    let graph = graph_from_ways(
        "destination-only-relaxed-retry",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            destination_only_primary(2, vec![(52.0, 21.001), (52.0, 21.002)]),
            way(3, vec![(52.0, 21.002), (52.0, 21.003)], RoadType::Primary),
        ],
    );
    let request = tight_request((52.0, 21.0002).into(), (52.0, 21.0028).into());

    let result = engine(&graph).route_leg(request).unwrap();

    assert_eq!(result.edge_ids, vec![1, 3, 5]);
    assert_eq!(result.diagnostics.destination_only_relaxed_retry_count, 1);
}

#[test]
fn route_goes_around_a_destination_only_road_it_does_not_end_on() {
    let graph = graph_from_ways(
        "destination-only-through",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            destination_only_primary(2, vec![(52.0, 21.001), (52.0, 21.002)]),
            way(3, vec![(52.0, 21.002), (52.0, 21.003)], RoadType::Primary),
            way(
                4,
                vec![(52.0, 21.001), (52.001, 21.0015)],
                RoadType::Primary,
            ),
            way(
                5,
                vec![(52.001, 21.0015), (52.0, 21.002)],
                RoadType::Primary,
            ),
        ],
    );
    let request = || tight_request((52.0, 21.0002).into(), (52.0, 21.0028).into());

    let engine = engine(&graph);
    let around = engine.route_leg(request()).unwrap();
    assert_eq!(around.edge_ids, vec![1, 7, 9, 5]);
    assert_eq!(around.diagnostics.destination_only_relaxed_retry_count, 0);

    // Ending on it, the road is open; emergency vehicles use it freely.
    let mut ending = request();
    ending.destination = (52.0, 21.0015).into();
    assert_eq!(engine.route_leg(ending).unwrap().edge_ids, vec![1, 3]);
    let mut emergency = request();
    emergency.profile = MobilityProfile::Emergency;
    assert_eq!(engine.route_leg(emergency).unwrap().edge_ids, vec![1, 3, 5]);
}

#[test]
fn active_conditional_access_restriction_changes_route_choice() {
    let graph = graph_from_ways(
        "conditional-access-route",
        vec![
            conditionally_closed_primary(1, vec![(52.0, 21.0), (52.0, 21.002)]),
            way(2, vec![(52.0, 21.0), (52.0008, 21.001)], RoadType::Primary),
            way(
                3,
                vec![(52.0008, 21.001), (52.0, 21.002)],
                RoadType::Primary,
            ),
        ],
    );

    let mut active_request = tight_request((52.0, 21.00002).into(), (52.0, 21.00198).into());
    // 2026-05-15T08:00:00Z is inside the conditional closure window.
    active_request
        .options
        .costing
        .conditional_restriction_eval_at_ms = Some(1_778_832_000_000);
    let active_result = engine(&graph).route_leg(active_request).unwrap();
    assert_eq!(active_result.edge_ids, vec![3, 5]);
    assert!(
        active_result
            .legs
            .iter()
            .flat_map(|leg| &leg.edges)
            .all(|edge| edge.conditional_restrictions.is_empty())
    );

    let mut inactive_request = tight_request((52.0, 21.00002).into(), (52.0, 21.00198).into());
    // 2026-07-01T08:00:00Z is outside the conditional closure window.
    inactive_request
        .options
        .costing
        .conditional_restriction_eval_at_ms = Some(1_782_892_800_000);
    let inactive_result = engine(&graph).route_leg(inactive_request).unwrap();
    assert_eq!(inactive_result.edge_ids, vec![1]);
    assert_eq!(
        inactive_result
            .policy_summary
            .conditional_restriction_eval_at_ms,
        Some(1_782_892_800_000)
    );
}

#[test]
fn costing_records_overlay_multiplier_breakdown() {
    let graph = graph_from_ways(
        "overlay-costing",
        vec![way(
            1,
            vec![(52.0, 21.0), (52.0, 21.001)],
            RoadType::Primary,
        )],
    );
    let mut request = tight_request((52.0, 21.00002).into(), (52.0, 21.00098).into());
    request.options.edge_cost_multipliers.insert(1, 2.5);
    request.options.avoided_edge_ids = BTreeSet::from([99]);
    request.options.warning_edge_ids = BTreeSet::from([1]);
    request.options.max_visited_edge_count = Some(250);
    request.options.costing.avoid_tracks = 1.4;
    request.options.costing.avoid_paths = f64::NAN;
    request.options.costing.avoid_motorways = 1.4;
    request.options.costing.avoid_destination_only = 0.6;
    request.options.costing.allow_ford = false;
    request.options.costing.avoid_tolls = 1.4;
    request.options.costing.avoid_ferries = 0.7;
    request.options.costing.avoid_gates = 0.5;
    request.options.costing.avoid_access_restricted = 0.4;
    request.options.costing.prefer_paved = f64::NAN;
    request.options.costing.prefer_known_route_refs = 0.3;
    request.options.alternatives.requested_alternative_count = 6;
    request.options.alternatives.max_shared_edge_ratio = 1.5;
    request.options.alternatives.max_cost_ratio = 4.0;
    request.options.alternatives.minimum_warning_difference = 2;

    let result = engine(&graph).route_leg(request).unwrap();

    assert_eq!(result.edge_ids, vec![1]);
    assert_eq!(result.diagnostics.overlay_multiplier_count, 1);
    assert_eq!(
        result.legs[0].edges[0].cost_breakdown.overlay_multiplier,
        2.5
    );
    assert_eq!(result.policy_summary.profile, MobilityProfile::Wheeled);
    assert_eq!(result.policy_summary.max_visited_edge_count, Some(250));
    assert_eq!(result.policy_summary.avoided_edge_count, 1);
    assert_eq!(result.policy_summary.warning_edge_count, 1);
    assert_eq!(result.policy_summary.cost_multiplier_edge_count, 1);
    assert_eq!(result.policy_summary.avoid_tracks, 1.0);
    assert_eq!(result.policy_summary.avoid_paths, 0.0);
    assert_eq!(result.policy_summary.avoid_motorways, 1.0);
    assert_eq!(result.policy_summary.avoid_destination_only, 0.6);
    assert!(!result.policy_summary.allow_ford);
    assert_eq!(result.policy_summary.avoid_tolls, 1.0);
    assert_eq!(result.policy_summary.avoid_ferries, 0.7);
    assert_eq!(result.policy_summary.avoid_gates, 0.5);
    assert_eq!(result.policy_summary.avoid_access_restricted, 0.4);
    assert_eq!(result.policy_summary.prefer_paved, 0.0);
    assert_eq!(result.policy_summary.prefer_known_route_refs, 0.3);
    assert_eq!(result.policy_summary.requested_alternative_count, 3);
    assert_eq!(result.policy_summary.max_shared_edge_ratio, 1.0);
    assert_eq!(result.policy_summary.max_cost_ratio, 3.0);
    assert_eq!(result.policy_summary.minimum_warning_difference, 2);
}

#[test]
fn partial_slow_go_range_weights_overlay_multiplier_by_covered_fraction() {
    let graph = graph_from_ways(
        "partial-slow-go-costing",
        vec![way(
            1,
            vec![(52.0, 21.0), (52.0, 21.001)],
            RoadType::Primary,
        )],
    );
    let mut full_request = tight_request((52.0, 21.00002).into(), (52.0, 21.00098).into());
    full_request.options.edge_cost_multipliers.insert(1, 5.0);
    let full = engine(&graph).route_leg(full_request).unwrap();

    let mut partial_request = tight_request((52.0, 21.00002).into(), (52.0, 21.00098).into());
    partial_request
        .options
        .edge_cost_multiplier_ranges
        .push(RouteEdgeCostMultiplierRange {
            edge_id: 1,
            start_fraction: 0.25,
            end_fraction: 0.75,
            multiplier: 5.0,
        });
    let partial = engine(&graph).route_leg(partial_request).unwrap();

    assert_eq!(partial.edge_ids, vec![1]);
    assert_eq!(partial.diagnostics.overlay_multiplier_count, 1);
    let multiplier = partial.legs[0].edges[0].cost_breakdown.overlay_multiplier;
    assert!((multiplier - 3.0).abs() < 0.01, "{multiplier}");
    assert!(
        partial.cost < full.cost,
        "partial slow-go cost should be lower than whole-edge slow-go cost"
    );
}

#[test]
fn partial_avoid_range_weights_avoid_penalty_by_covered_fraction() {
    let graph = graph_from_ways(
        "partial-avoid-costing",
        vec![way(
            1,
            vec![(52.0, 21.0), (52.0, 21.001)],
            RoadType::Primary,
        )],
    );
    let mut full_request = tight_request((52.0, 21.00002).into(), (52.0, 21.00098).into());
    full_request.options.avoided_edge_ids = BTreeSet::from([1]);
    let full = engine(&graph).route_leg(full_request).unwrap();

    let mut partial_request = tight_request((52.0, 21.00002).into(), (52.0, 21.00098).into());
    partial_request
        .options
        .avoided_edge_ranges
        .push(RouteEdgeAvoidanceRange {
            edge_id: 1,
            start_fraction: 0.25,
            end_fraction: 0.75,
        });
    let partial = engine(&graph).route_leg(partial_request).unwrap();

    assert_eq!(partial.edge_ids, vec![1]);
    assert_eq!(partial.diagnostics.avoided_edge_count, 1);
    let penalty = partial.legs[0].edges[0].cost_breakdown.avoided_edge_penalty;
    assert!((penalty - 3.5).abs() < 0.01, "{penalty}");
    assert!(
        partial.cost < full.cost,
        "partial avoid cost should be lower than whole-edge avoid cost"
    );
}

#[test]
fn costing_options_can_avoid_tolls_ferries_gates_and_restricted_access() {
    let graph = graph_from_ways(
        "road-use-costing",
        vec![
            controlled_access_primary(1, vec![(52.0, 21.0), (52.0, 21.001)]),
            controlled_access_primary(2, vec![(52.0, 21.001), (52.0, 21.002)]),
            way(3, vec![(52.0, 21.0), (52.0008, 21.001)], RoadType::Primary),
            way(
                4,
                vec![(52.0008, 21.001), (52.0, 21.002)],
                RoadType::Primary,
            ),
        ],
    );
    let mut request = tight_request((52.0, 21.00002).into(), (52.0, 21.00198).into());
    request.options.costing.avoid_tolls = 1.0;
    request.options.costing.avoid_ferries = 1.0;
    request.options.costing.avoid_gates = 1.0;
    request.options.costing.avoid_access_restricted = 1.0;

    let result = engine(&graph).route_leg(request).unwrap();

    assert_eq!(result.edge_ids, vec![5, 7]);
    assert!(result.legs.iter().flat_map(|leg| &leg.edges).all(|edge| {
        !edge.is_toll && !edge.is_ferry && !edge.has_gate && !edge.is_access_restricted
    }));
}

#[test]
fn default_and_emergency_profiles_are_first_class_route_profiles() {
    assert_eq!(
        serde_json::to_value(MobilityProfile::Default).unwrap(),
        serde_json::json!("default")
    );
    assert_eq!(
        serde_json::to_value(MobilityProfile::Emergency).unwrap(),
        serde_json::json!("emergency")
    );

    let graph = graph_from_ways(
        "route_core-profiles",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
        ],
    );

    let default_route = engine(&graph)
        .route_leg(tight_profile_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00198).into(),
            MobilityProfile::Default,
        ))
        .unwrap();
    let emergency_route = engine(&graph)
        .route_leg(tight_profile_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00198).into(),
            MobilityProfile::Emergency,
        ))
        .unwrap();

    assert_eq!(default_route.edge_ids, vec![1, 3]);
    assert_eq!(emergency_route.edge_ids, vec![1, 3]);
}

#[test]
fn guidance_generates_turn_and_destination_maneuvers() {
    let graph = graph_from_ways(
        "turn-guidance",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(
                2,
                vec![(52.0, 21.001), (52.001, 21.001)],
                RoadType::Secondary,
            ),
        ],
    );
    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.00098, 21.001).into(),
        ))
        .unwrap();

    let turn = result
        .maneuvers
        .iter()
        .find(|maneuver| {
            maneuver.kind == RouteManeuverKind::Turn
                && maneuver.instruction.contains("Way 2")
                && maneuver.turn_angle_deg.is_some()
        })
        .expect("expected a structured turn maneuver");
    assert_eq!(turn.instruction_key, "route.turn");
    assert_eq!(
        turn.pre_transition_instruction.as_deref(),
        Some(turn.instruction.as_str())
    );
    assert_eq!(turn.road_name.as_deref(), Some("Way 2"));
    assert_eq!(
        turn.instruction_parameters
            .get("road_name")
            .map(String::as_str),
        Some("Way 2")
    );
    assert!(turn.bearing_deg.is_some());
    assert!(turn.cardinal_direction.is_some());
    assert_eq!(
        result.maneuvers.last().unwrap().instruction,
        "Arrive at destination"
    );
}

#[test]
fn guidance_includes_turn_lane_hints_from_road_facts() {
    let mut graph = graph_from_ways(
        "turn-lane-guidance",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(
                2,
                vec![(52.0, 21.001), (52.001, 21.001)],
                RoadType::Secondary,
            ),
        ],
    );
    // `turn:lanes` describes the lanes at the end of the way approaching the
    // junction; the entered way's own describe its far end.
    let mut approach = road_way_facts(1, RoadType::Primary);
    approach.turn_lanes_forward = Some("left|through|right".to_string());
    approach.lane_connectivity_available = true;
    let mut facts = road_way_facts(2, RoadType::Secondary);
    facts.name = Some("Lane Ramp".to_string());
    facts.lanes = Some("3".to_string());
    facts.turn_lanes_forward = Some("through|right".to_string());
    facts.lane_connectivity_available = true;
    graph.set_way_facts(vec![approach, facts]);
    graph.refresh_shared_edge_metadata();

    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.00098, 21.001).into(),
        ))
        .unwrap();

    let turn = result
        .maneuvers
        .iter()
        .find(|maneuver| maneuver.kind == RouteManeuverKind::Turn)
        .expect("expected a turn maneuver");
    assert_eq!(turn.road_name.as_deref(), Some("Lane Ramp"));
    assert_eq!(turn.lane_count, Some(3));
    assert_eq!(
        turn.instruction_parameters
            .get("turn_lanes")
            .map(String::as_str),
        Some("left|through|right")
    );
    assert_eq!(
        turn.instruction_parameters
            .get("lane_guidance")
            .map(String::as_str),
        Some("use_left_lane")
    );
    assert_eq!(
        turn.instruction_parameters
            .get("lane_guidance_target")
            .map(String::as_str),
        Some("left")
    );
    assert_eq!(
        turn.instruction_parameters
            .get("lane_guidance_lane_indexes")
            .map(String::as_str),
        Some("1")
    );
    assert_eq!(
        turn.instruction_parameters
            .get("lane_connectivity_available")
            .map(String::as_str),
        Some("true")
    );
    assert_eq!(
        turn.lanes.iter().map(|lane| lane.valid).collect::<Vec<_>>(),
        [true, false, false]
    );
}

#[test]
fn guidance_uses_ramp_instruction_for_link_edges() {
    let graph = graph_from_ways(
        "link-ramp-guidance",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            signed_link_primary(2, vec![(52.0, 21.001), (52.001, 21.001)]),
        ],
    );

    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.00098, 21.001).into(),
        ))
        .unwrap();

    let ramp = result
        .maneuvers
        .iter()
        .find(|maneuver| maneuver.instruction_key == "route.ramp")
        .expect("expected ramp guidance");
    assert_eq!(ramp.kind, RouteManeuverKind::Turn);
    assert_eq!(ramp.road_name.as_deref(), Some("A4 ramp"));
    assert!(ramp.instruction.contains("Take exit 7A onto A4 ramp"));
    assert!(ramp.instruction.contains("toward Krakow"));
    assert_eq!(ramp.sign_text.as_deref(), Some("Krakow"));
    assert_eq!(ramp.branch_text.as_deref(), Some("A4"));
    assert_eq!(
        ramp.instruction_parameters
            .get("junction_ref")
            .map(String::as_str),
        Some("7A")
    );
    assert_eq!(
        ramp.instruction_parameters
            .get("guidance_kind")
            .map(String::as_str),
        Some("ramp")
    );
    assert_eq!(
        ramp.instruction_parameters
            .get("road_use")
            .map(String::as_str),
        Some("link")
    );

    let link_edge = result.legs[0]
        .edges
        .iter()
        .find(|edge| edge.name.as_deref() == Some("A4 ramp"))
        .expect("expected link edge attribution");
    assert!(link_edge.is_link);
}

#[test]
fn guidance_uses_ramp_instruction_for_branching_link_junctions() {
    let graph = graph_from_ways(
        "branching-link-ramp-guidance",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            signed_link_primary(2, vec![(52.0, 21.001), (52.001, 21.001)]),
            way(
                3,
                vec![(52.0, 21.001), (51.9996, 21.0018)],
                RoadType::Secondary,
            ),
        ],
    );

    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.00098, 21.001).into(),
        ))
        .unwrap();

    let ramp = result
        .maneuvers
        .iter()
        .find(|maneuver| maneuver.instruction_key == "route.ramp")
        .expect("expected ramp guidance at branching link junction");

    // The ramp leaves at 90 degrees and the other branch at about 40 degrees:
    // a turn, not a fork, though the junction has two branches.
    assert_eq!(ramp.kind, RouteManeuverKind::Turn);
    assert!(ramp.instruction.contains("Take exit 7A onto A4 ramp"));
    assert!(ramp.instruction.contains("toward Krakow"));
    assert_eq!(
        ramp.instruction_parameters
            .get("guidance_kind")
            .map(String::as_str),
        Some("ramp")
    );
    assert_eq!(
        ramp.instruction_parameters
            .get("junction_ref")
            .map(String::as_str),
        Some("7A")
    );
    assert_eq!(
        ramp.instruction_parameters
            .get("guidance_reason")
            .map(String::as_str),
        Some("significant_turn")
    );
    let outgoing_count = ramp
        .instruction_parameters
        .get("junction_outgoing_edge_count")
        .and_then(|value| value.parse::<usize>().ok())
        .expect("junction outgoing count");
    assert!(outgoing_count > 1);
}

#[test]
fn guidance_classifies_u_turn_maneuvers() {
    let graph = graph_from_ways(
        "uturn-guidance",
        vec![
            one_way(1, vec![(52.0, 21.0), (52.0, 21.001)]),
            one_way(2, vec![(52.0, 21.001), (52.0003, 20.999)]),
        ],
    );
    let mut request = tight_request((52.0, 21.00002).into(), (52.0003, 20.99902).into());
    request.options.max_correlation_distance_m = 6;
    let result = engine(&graph).route_leg(request).unwrap();
    let maneuver = result
        .maneuvers
        .iter()
        .find(|maneuver| maneuver.instruction.contains("U-turn"))
        .expect("expected a U-turn instruction");

    assert_eq!(
        serde_json::to_value(maneuver.kind).unwrap(),
        serde_json::json!("u_turn")
    );
}

#[test]
fn guidance_classifies_keep_maneuvers_for_shallow_forks() {
    let graph = graph_from_ways(
        "keep-guidance",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(
                2,
                vec![(52.0, 21.001), (52.00012, 21.002)],
                RoadType::Secondary,
            ),
        ],
    );
    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.00012, 21.00198).into(),
        ))
        .unwrap();
    let maneuver = result
        .maneuvers
        .iter()
        .find(|maneuver| maneuver.instruction.contains("Keep"))
        .expect("expected a keep instruction for shallow fork");

    assert_eq!(
        serde_json::to_value(maneuver.kind).unwrap(),
        serde_json::json!("keep")
    );
    assert!(maneuver.instruction.contains("Way 2"));
}

#[test]
fn guidance_classifies_fork_maneuvers_at_branching_junctions() {
    let graph = graph_from_ways(
        "fork-guidance",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(
                2,
                vec![(52.0, 21.001), (52.00018, 21.002)],
                RoadType::Secondary,
            ),
            way(
                3,
                vec![(52.0, 21.001), (51.99982, 21.002)],
                RoadType::Secondary,
            ),
        ],
    );
    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.00018, 21.00198).into(),
        ))
        .unwrap();
    let maneuver = result
        .maneuvers
        .iter()
        .find(|maneuver| maneuver.instruction.contains("Fork"))
        .expect("expected a fork instruction for branching junction");

    assert_eq!(
        serde_json::to_value(maneuver.kind).unwrap(),
        serde_json::json!("fork")
    );
    assert!(maneuver.instruction.contains("Way 2"));
}

#[test]
fn guidance_explains_complex_fork_junctions() {
    let graph = graph_from_ways(
        "complex-fork-guidance",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(
                2,
                vec![(52.0, 21.001), (52.00025, 21.002)],
                RoadType::Secondary,
            ),
            way(
                3,
                vec![(52.0, 21.001), (51.99975, 21.002)],
                RoadType::Secondary,
            ),
            way(
                4,
                vec![(52.0, 21.001), (52.0007, 21.0016)],
                RoadType::Secondary,
            ),
            way(
                5,
                vec![(52.0, 21.001), (51.9993, 21.0016)],
                RoadType::Secondary,
            ),
        ],
    );
    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.00025, 21.00198).into(),
        ))
        .unwrap();
    let maneuver = result
        .maneuvers
        .iter()
        .find(|maneuver| maneuver.kind == RouteManeuverKind::Fork)
        .expect("expected a fork instruction for complex branching junction");

    assert!(
        maneuver
            .quality_flags
            .contains(&RouteGuidanceQualityFlag::ComplexJunction)
    );
    assert_eq!(
        maneuver
            .instruction_parameters
            .get("guidance_reason")
            .map(String::as_str),
        Some("branching_junction")
    );
    assert_eq!(
        maneuver
            .instruction_parameters
            .get("significant_turn")
            .map(String::as_str),
        Some("false")
    );
    let outgoing_count = maneuver
        .instruction_parameters
        .get("junction_outgoing_edge_count")
        .and_then(|value| value.parse::<usize>().ok())
        .expect("junction outgoing count");
    assert!(outgoing_count > 1);
    let complex_count = maneuver
        .instruction_parameters
        .get("complex_junction_edge_count")
        .and_then(|value| value.parse::<usize>().ok())
        .expect("complex junction edge count");
    assert!(complex_count > 3);
}

#[test]
fn guidance_classifies_merge_maneuvers_at_converging_junctions() {
    let graph = graph_from_ways(
        "merge-guidance",
        vec![
            one_way(1, vec![(52.0, 21.0), (52.0, 21.001)]),
            one_way(2, vec![(52.00018, 21.0), (52.0, 21.001)]),
            one_way(3, vec![(52.0, 21.001), (52.0, 21.002)]),
        ],
    );
    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00198).into(),
        ))
        .unwrap();
    let maneuver = result
        .maneuvers
        .iter()
        .find(|maneuver| maneuver.instruction.contains("Merge"))
        .expect("expected a merge instruction for converging junction");

    assert_eq!(
        serde_json::to_value(maneuver.kind).unwrap(),
        serde_json::json!("merge")
    );
    assert!(maneuver.instruction.contains("Way 3"));
    assert_eq!(
        maneuver
            .instruction_parameters
            .get("guidance_reason")
            .map(String::as_str),
        Some("merging_junction")
    );
    assert_eq!(
        maneuver
            .instruction_parameters
            .get("junction_incoming_edge_count")
            .map(String::as_str),
        Some("1")
    );
}

#[test]
fn guidance_explains_complex_merge_junctions() {
    let graph = graph_from_ways(
        "complex-merge-guidance",
        vec![
            one_way(1, vec![(52.0, 21.0), (52.0, 21.001)]),
            one_way(2, vec![(52.00018, 21.0), (52.0, 21.001)]),
            one_way(3, vec![(51.99982, 21.0), (52.0, 21.001)]),
            one_way(4, vec![(52.00045, 21.0002), (52.0, 21.001)]),
            one_way(5, vec![(51.99955, 21.0002), (52.0, 21.001)]),
            one_way(6, vec![(52.0, 21.001), (52.0, 21.002)]),
        ],
    );
    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00198).into(),
        ))
        .unwrap();
    let maneuver = result
        .maneuvers
        .iter()
        .find(|maneuver| maneuver.kind == RouteManeuverKind::Merge)
        .expect("expected a merge instruction for complex converging junction");

    assert!(
        maneuver
            .quality_flags
            .contains(&RouteGuidanceQualityFlag::ComplexJunction)
    );
    assert_eq!(
        maneuver
            .instruction_parameters
            .get("guidance_reason")
            .map(String::as_str),
        Some("merging_junction")
    );
    let incoming_count = maneuver
        .instruction_parameters
        .get("junction_incoming_edge_count")
        .and_then(|value| value.parse::<usize>().ok())
        .expect("junction incoming count");
    assert!(incoming_count > 3);
    let complex_count = maneuver
        .instruction_parameters
        .get("complex_junction_edge_count")
        .and_then(|value| value.parse::<usize>().ok())
        .expect("complex junction edge count");
    assert!(complex_count > 3);
}

#[test]
fn guidance_classifies_roundabout_maneuvers_from_road_graph_facts() {
    let graph = graph_from_ways(
        "roundabout-guidance",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            roundabout_way(2, vec![(52.0, 21.001), (52.0004, 21.0014)]),
            way(
                3,
                vec![(52.0004, 21.0014), (52.0008, 21.002)],
                RoadType::Primary,
            ),
        ],
    );
    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.00078, 21.00198).into(),
        ))
        .unwrap();

    let maneuver = result
        .maneuvers
        .iter()
        .find(|maneuver| maneuver.instruction.contains("roundabout"))
        .expect("expected roundabout guidance");

    assert_eq!(maneuver.road_name.as_deref(), Some("Way 2"));
    assert_eq!(maneuver.roundabout_exit_number, Some(1));
    assert!(maneuver.instruction.contains("exit 1"));
    assert_eq!(
        maneuver
            .instruction_parameters
            .get("roundabout_exit_number")
            .map(String::as_str),
        Some("1")
    );
    assert_eq!(
        maneuver
            .instruction_parameters
            .get("roundabout_exit_count")
            .map(String::as_str),
        Some("1")
    );
    let roundabout_edge = result.legs[0]
        .edges
        .iter()
        .find(|edge| edge.is_roundabout)
        .expect("expected roundabout edge attribution");
    assert!(roundabout_edge.roundabout_exit_count > 0);
}

#[test]
fn profile_access_filtering_rejects_motor_vehicle_only_for_foot() {
    let mut graph = graph_from_ways(
        "vehicle-only",
        vec![way(
            1,
            vec![(52.0, 21.0), (52.0, 21.001)],
            RoadType::Residential,
        )],
    );
    for edge in graph.graph.edge_indices() {
        graph
            .graph
            .update_edge(edge, |edge| edge.mobility_mode_mask = MobilityMode::WHEELED);
    }

    let error = engine(&graph)
        .route_leg(LegRequest::new(
            (52.0, 21.0).into(),
            (52.0, 21.001).into(),
            MobilityProfile::OnFoot,
        ))
        .unwrap_err()
        .error;

    assert_eq!(
        error,
        RouteError::ImpossibleMobilityProfile {
            profile: MobilityProfile::OnFoot,
        }
    );
}

#[test]
fn blocked_edges_are_avoided_when_an_alternative_exists() {
    let graph = graph_from_ways(
        "blocked",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(
                3,
                vec![(52.0, 21.0), (52.0008, 21.001)],
                RoadType::Residential,
            ),
            way(
                4,
                vec![(52.0008, 21.001), (52.0, 21.002)],
                RoadType::Residential,
            ),
        ],
    );
    let mut request = tight_request((52.0, 21.00002).into(), (52.0, 21.00198).into());
    request.options.blocked_edge_ids = BTreeSet::from([1, 3]);

    let result = engine(&graph).route_leg(request).unwrap();

    assert_eq!(result.status, RouteStatus::Routed);
    assert_eq!(result.edge_ids, vec![5, 7]);
}

#[test]
fn blocked_edge_range_is_avoided_when_an_alternative_exists() {
    let graph = graph_from_ways(
        "blocked-range",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(
                3,
                vec![(52.0, 21.0), (52.0008, 21.001)],
                RoadType::Residential,
            ),
            way(
                4,
                vec![(52.0008, 21.001), (52.0, 21.002)],
                RoadType::Residential,
            ),
        ],
    );
    let mut request = tight_request((52.0, 21.00002).into(), (52.0, 21.00198).into());
    request.options.blocked_edge_ranges = vec![RouteEdgeRange {
        edge_id: 1,
        start_fraction: 0.25,
        end_fraction: 0.75,
    }];

    let result = engine(&graph).route_leg(request).unwrap();

    assert_eq!(result.status, RouteStatus::Routed);
    assert_eq!(result.edge_ids, vec![5, 7]);
}

#[test]
fn fully_blocked_corridor_reports_typed_failure() {
    let graph = graph_from_ways(
        "blocked-corridor",
        vec![one_way(1, vec![(52.0, 21.0), (52.0, 21.001)])],
    );
    let mut request = tight_request((52.0, 21.00002).into(), (52.0, 21.00098).into());
    request.options.blocked_edge_ids = BTreeSet::from([1]);

    let error = engine(&graph).route_leg(request).unwrap_err().error;

    assert_eq!(
        error,
        RouteError::BlockedCorridor {
            blocked_edge_count: 2,
            visited_edge_count: 0,
        }
    );
}

#[test]
fn blocked_edge_range_without_alternative_reports_typed_failure() {
    let graph = graph_from_ways(
        "blocked-range-corridor",
        vec![one_way(1, vec![(52.0, 21.0), (52.0, 21.001)])],
    );
    let mut request = tight_request((52.0, 21.00002).into(), (52.0, 21.00098).into());
    request.options.blocked_edge_ranges = vec![RouteEdgeRange {
        edge_id: 1,
        start_fraction: 0.25,
        end_fraction: 0.75,
    }];

    let error = engine(&graph).route_leg(request).unwrap_err().error;

    assert_eq!(
        error,
        RouteError::BlockedCorridor {
            blocked_edge_count: 2,
            visited_edge_count: 0,
        }
    );
}

#[test]
fn route_options_default_missing_blocked_edge_ranges() {
    let mut value = serde_json::to_value(wayfarer::RouteOptions::default()).unwrap();
    value.as_object_mut().unwrap().remove("blockedEdgeRanges");
    value["blockedEdgeIds"] = serde_json::json!([1]);
    let options: wayfarer::RouteOptions = serde_json::from_value(value).unwrap();

    assert!(options.blocked_edge_ranges.is_empty());
    assert!(options.blocked_edge_ids.contains(&1));
}

#[test]
fn blocked_and_avoided_edges_steer_the_route() {
    let graph = graph_from_ways(
        "diagnostics",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(
                3,
                vec![(52.0, 21.0), (52.0008, 21.001)],
                RoadType::Residential,
            ),
            way(
                4,
                vec![(52.0008, 21.001), (52.0, 21.002)],
                RoadType::Residential,
            ),
        ],
    );
    let mut request = tight_request((52.0, 21.00002).into(), (52.0, 21.00198).into());
    request.options.blocked_edge_ids = BTreeSet::from([1, 3]);
    request.options.avoided_edge_ids = BTreeSet::from([5]);

    let result = engine(&graph).route_leg(request).unwrap();

    assert_eq!(result.edge_ids, vec![5, 7]);
    assert_eq!(result.diagnostics.final_path_edge_count, 2);
}

/// A warning overlay reaches the route leg and the guidance.
///
/// This and the range test below are what survives of
/// `warning_overlays_are_preserved_on_route_legs_and_guidance` and
/// `line_warning_overlay_preserves_partial_edge_range_for_guidance`. Those two
/// went with the `MeshObjectProjection` overlay compiler, but the compiler was
/// only how they got a warning onto `RouteOptions` — the property they proved
/// is the engine's: a warned edge is flagged on the leg and turns into a Hazard
/// maneuver. Overlay zones set the same options, so the behaviour outlived its
/// source and needs an assertion that does not name the compiler.
#[test]
fn warning_edge_ids_reach_the_route_leg_and_guidance() {
    let graph = graph_from_ways(
        "warning-guidance",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
        ],
    );
    let mut request = tight_request((52.0, 21.00002).into(), (52.0, 21.00198).into());
    request.options.warning_edge_ids = BTreeSet::from([3]);

    let result = engine(&graph).route_leg(request).unwrap();

    assert_eq!(result.edge_ids, vec![1, 3]);
    assert!(!result.legs[0].edges[0].has_warning_overlay);
    assert!(result.legs[0].edges[1].has_warning_overlay);
    assert!(result.maneuvers.iter().any(|maneuver| {
        maneuver.kind == RouteManeuverKind::Hazard
            && maneuver.instruction.contains("Hazard")
            && maneuver.begin_shape_index == 1
    }));
}

/// A warning over PART of an edge keeps its fractions all the way to guidance.
#[test]
fn warning_edge_ranges_reach_the_route_leg_and_guidance() {
    let graph = graph_from_ways(
        "warning-partial-range",
        vec![way(
            1,
            vec![(52.0, 21.0), (52.0, 21.001)],
            RoadType::Primary,
        )],
    );
    let mut request = tight_request((52.0, 21.00002).into(), (52.0, 21.00098).into());
    request.options.warning_edge_ranges = vec![RouteEdgeRange {
        edge_id: 1,
        start_fraction: 0.2,
        end_fraction: 0.8,
    }];

    let result = engine(&graph).route_leg(request).unwrap();

    assert_eq!(result.edge_ids, vec![1]);
    assert_eq!(result.legs[0].edges[0].warning_ranges.len(), 1);
    let hazard = result
        .maneuvers
        .iter()
        .find(|maneuver| maneuver.kind == RouteManeuverKind::Hazard)
        .expect("expected warning hazard maneuver");
    let start_fraction = hazard
        .instruction_parameters
        .get("edge_start_fraction")
        .and_then(|value| value.parse::<f64>().ok())
        .expect("hazard start fraction");
    let end_fraction = hazard
        .instruction_parameters
        .get("edge_end_fraction")
        .and_then(|value| value.parse::<f64>().ok())
        .expect("hazard end fraction");
    assert!((start_fraction - 0.2).abs() < 0.05, "{start_fraction}");
    assert!((end_fraction - 0.8).abs() < 0.05, "{end_fraction}");
    assert!(
        hazard
            .quality_flags
            .contains(&RouteGuidanceQualityFlag::OverlayWarningOnClippedShape)
    );
}

#[test]
fn warning_guidance_uses_clipped_shape_segment_fractions() {
    let graph = graph_from_ways(
        "warning-clipped-shape",
        vec![way(
            1,
            vec![(52.0, 21.0), (52.0, 21.001)],
            RoadType::Primary,
        )],
    );
    let mut request = tight_request((52.0, 21.0004).into(), (52.0, 21.00095).into());
    request.options.warning_edge_ids = BTreeSet::from([1]);

    let result = engine(&graph).route_leg(request).unwrap();

    assert_eq!(result.edge_ids, vec![1]);
    let segment = &result.legs[0].shape_segments[0];
    assert!(segment.start_fraction > 0.0);
    assert!(segment.end_fraction < 1.0);
    let hazard = result
        .maneuvers
        .iter()
        .find(|maneuver| maneuver.kind == RouteManeuverKind::Hazard)
        .expect("expected warning hazard maneuver");
    assert_eq!(hazard.begin_shape_index, segment.begin_shape_index);
    assert_eq!(hazard.end_shape_index, segment.end_shape_index);
    let start_fraction = hazard
        .instruction_parameters
        .get("edge_start_fraction")
        .and_then(|value| value.parse::<f64>().ok())
        .expect("hazard start fraction");
    let end_fraction = hazard
        .instruction_parameters
        .get("edge_end_fraction")
        .and_then(|value| value.parse::<f64>().ok())
        .expect("hazard end fraction");
    assert!((start_fraction - segment.start_fraction).abs() < 0.01);
    assert!((end_fraction - segment.end_fraction).abs() < 0.01);
    assert!(
        hazard
            .quality_flags
            .contains(&RouteGuidanceQualityFlag::OverlayWarningOnClippedShape)
    );
}

#[test]
fn simple_turn_restriction_forces_alternate_path() {
    let mut graph = graph_from_ways(
        "restricted-turn",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(
                3,
                vec![(52.0, 21.0), (52.0008, 21.001)],
                RoadType::Residential,
            ),
            way(
                4,
                vec![(52.0008, 21.001), (52.0, 21.002)],
                RoadType::Residential,
            ),
        ],
    );
    restrict_transition(&mut graph, 1, 3, 1);

    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00198).into(),
        ))
        .unwrap();

    assert_eq!(result.edge_ids, vec![5, 7]);
    assert!(result.diagnostics.restricted_transition_count > 0);
}

#[test]
fn directed_search_keeps_distinct_incoming_edge_labels_at_same_node() {
    let mut graph = graph_from_ways(
        "directed-edge-label-turn-restriction",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(
                2,
                vec![(52.0, 21.0), (52.00035, 21.0005)],
                RoadType::Primary,
            ),
            way(
                3,
                vec![(52.00035, 21.0005), (52.0, 21.001)],
                RoadType::Primary,
            ),
            way(4, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
        ],
    );
    restrict_transition(&mut graph, 1, 7, 1);

    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00198).into(),
        ))
        .unwrap();

    assert_eq!(result.edge_ids, vec![3, 5, 7]);
    assert!(result.diagnostics.restricted_transition_count > 0);
}

#[test]
fn turn_restriction_out_of_a_shared_node_takes_the_other_outgoing_edge() {
    let mut graph = graph_from_ways(
        "bidirectional-reverse-edge-label-turn-restriction",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(
                3,
                vec![(52.0, 21.001), (52.00035, 21.0015)],
                RoadType::Primary,
            ),
            way(
                4,
                vec![(52.00035, 21.0015), (52.0, 21.002)],
                RoadType::Primary,
            ),
        ],
    );
    restrict_transition(&mut graph, 1, 3, 1);

    let request = tight_request((52.0, 21.00002).into(), (52.0, 21.00198).into());
    let result = engine(&graph).route_leg(request).unwrap();

    assert_eq!(result.edge_ids, vec![1, 5, 7]);
    assert!(result.diagnostics.restricted_transition_count > 0);
}

#[test]
fn turn_restriction_into_a_shared_node_takes_the_other_incoming_edge() {
    let mut graph = graph_from_ways(
        "bidirectional-forward-edge-label-turn-restriction",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(
                2,
                vec![(52.0, 21.0), (52.00035, 21.0005)],
                RoadType::Primary,
            ),
            way(
                3,
                vec![(52.00035, 21.0005), (52.0, 21.001)],
                RoadType::Primary,
            ),
            way(4, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
        ],
    );
    restrict_transition(&mut graph, 1, 7, 1);

    let request = tight_request((52.0, 21.00002).into(), (52.0, 21.00198).into());
    let result = engine(&graph).route_leg(request).unwrap();

    assert_eq!(result.edge_ids, vec![3, 5, 7]);
    assert!(result.diagnostics.restricted_transition_count > 0);
}

#[test]
fn only_restricted_transition_reports_no_path() {
    let mut graph = graph_from_ways(
        "restricted-only",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
        ],
    );
    restrict_transition(&mut graph, 1, 3, 1);

    let error = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00198).into(),
        ))
        .unwrap_err()
        .error;

    assert!(error.is_no_path());
}

#[test]
fn complex_via_edge_restriction_forces_alternate_path() {
    let mut graph = graph_from_ways(
        "complex-restricted-turn",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(3, vec![(52.0, 21.002), (52.0, 21.003)], RoadType::Primary),
            way(
                4,
                vec![(52.0, 21.0), (52.0008, 21.0015)],
                RoadType::Residential,
            ),
            way(
                5,
                vec![(52.0008, 21.0015), (52.0, 21.003)],
                RoadType::Residential,
            ),
        ],
    );
    restrict_complex_sequence(&mut graph, 1, vec![3], 5);

    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00298).into(),
        ))
        .unwrap();

    // 1 -> 3 -> 5 is forbidden; turning back twice on the via way (3, 4, 3)
    // pays two U-turns and costs more than the residential detour (7, 9).
    assert_eq!(result.edge_ids, vec![7, 9]);
    assert!(result.diagnostics.restricted_transition_count > 0);

    let bidirectional_request = tight_request((52.0, 21.00002).into(), (52.0, 21.00298).into());
    let bidirectional = engine(&graph).route_leg(bidirectional_request).unwrap();

    assert_eq!(bidirectional.edge_ids, vec![7, 9]);
    assert!(bidirectional.diagnostics.restricted_transition_count > 0);
}

#[test]
fn sharp_turn_penalty_can_select_smoother_path() {
    let graph = graph_from_ways(
        "turn-penalty",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.001, 21.001)], RoadType::Primary),
            way(3, vec![(52.0, 21.0), (52.0005, 21.0005)], RoadType::Primary),
            way(
                4,
                vec![(52.0005, 21.0005), (52.001, 21.001)],
                RoadType::Primary,
            ),
        ],
    );

    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.00098, 21.001).into(),
        ))
        .unwrap();

    assert_eq!(result.edge_ids, vec![5, 7]);
}

#[test]
fn link_transition_penalty_is_lower_than_equivalent_non_link_turn() {
    let origin = (52.0, 21.00002).into();
    let destination = (52.00098, 21.001).into();
    let normal_graph = graph_from_ways(
        "normal-transition-cost",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.001, 21.001)], RoadType::Primary),
        ],
    );
    let mut link_edge = way(2, vec![(52.0, 21.001), (52.001, 21.001)], RoadType::Primary);
    link_edge.is_link = true;
    let link_graph = graph_from_ways(
        "link-transition-cost",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            link_edge,
        ],
    );

    let normal = engine(&normal_graph)
        .route_leg(tight_request(origin, destination))
        .unwrap();
    let link = engine(&link_graph)
        .route_leg(tight_request(origin, destination))
        .unwrap();
    let bidirectional_normal_request = tight_request(origin, destination);
    let bidirectional_link_request = tight_request(origin, destination);
    let bidirectional_normal = engine(&normal_graph)
        .route_leg(bidirectional_normal_request)
        .unwrap();
    let bidirectional_link = engine(&link_graph)
        .route_leg(bidirectional_link_request)
        .unwrap();

    assert_eq!(normal.edge_ids, link.edge_ids);
    assert_eq!(normal.edge_ids.len(), 2, "edge_ids={:?}", normal.edge_ids);
    let normal_turns = normal
        .legs
        .iter()
        .flat_map(|leg| &leg.edges)
        .map(|edge| edge.cost_breakdown.turn_penalty)
        .collect::<Vec<_>>();
    let link_turns = link
        .legs
        .iter()
        .flat_map(|leg| &leg.edges)
        .map(|edge| edge.cost_breakdown.turn_penalty)
        .collect::<Vec<_>>();
    // Phase 0 time metric: turn penalties are now expressed in seconds
    // (legacy 35.0 and 17.5 units converted at the 30 km/h reference).
    assert_eq!(normal_turns, vec![0.0, 4.2]);
    assert_eq!(link_turns, vec![0.0, 2.1]);
    assert!(
        normal.cost > link.cost + 2.0,
        "link transition should lower routed cost as well as attribution: normal={}, link={}",
        normal.cost,
        link.cost
    );
    assert_eq!(bidirectional_normal.edge_ids, normal.edge_ids);
    assert_eq!(bidirectional_link.edge_ids, link.edge_ids);
    assert!(
        bidirectional_normal.cost > bidirectional_link.cost + 2.0,
        "bidirectional link transition should lower routed cost: normal={}, link={}",
        bidirectional_normal.cost,
        bidirectional_link.cost
    );
    let cost_tolerance = 1.0e-9;
    assert!(
        (bidirectional_normal.cost - normal.cost).abs() <= cost_tolerance,
        "bidirectional normal cost should match directed: directed={}, bidirectional={}",
        normal.cost,
        bidirectional_normal.cost
    );
    assert!(
        (bidirectional_link.cost - link.cost).abs() <= cost_tolerance,
        "bidirectional link cost should match directed: directed={}, bidirectional={}",
        link.cost,
        bidirectional_link.cost
    );
}

#[test]
fn same_signed_corridor_relief_lowers_routed_transition_cost() {
    let origin = (52.0, 21.00002).into();
    let destination = (52.00098, 21.001).into();
    let unsigned_graph = graph_from_ways(
        "unsigned-corridor-transition-cost",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.001, 21.001)], RoadType::Primary),
        ],
    );
    let mut signed_a = way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary);
    signed_a.ref_name = Some("DK7".into());
    let mut signed_b = way(2, vec![(52.0, 21.001), (52.001, 21.001)], RoadType::Primary);
    signed_b.ref_name = Some("dk7".into());
    let signed_graph = graph_from_ways("signed-corridor-transition-cost", vec![signed_a, signed_b]);

    let unsigned = engine(&unsigned_graph)
        .route_leg(tight_request(origin, destination))
        .unwrap();
    let signed = engine(&signed_graph)
        .route_leg(tight_request(origin, destination))
        .unwrap();

    assert_eq!(unsigned.edge_ids, signed.edge_ids);
    let unsigned_turns = unsigned
        .legs
        .iter()
        .flat_map(|leg| &leg.edges)
        .map(|edge| edge.cost_breakdown.turn_penalty)
        .collect::<Vec<_>>();
    let signed_turns = signed
        .legs
        .iter()
        .flat_map(|leg| &leg.edges)
        .map(|edge| edge.cost_breakdown.turn_penalty)
        .collect::<Vec<_>>();
    // Phase 0 time metric: turn penalties are now in seconds (legacy 35.0 / 24.5).
    assert_eq!(unsigned_turns, vec![0.0, 4.2]);
    assert_eq!(signed_turns, vec![0.0, 2.94]);
    assert!(
        unsigned.cost > signed.cost + 1.0,
        "same signed corridor should lower routed cost: unsigned={}, signed={}",
        unsigned.cost,
        signed.cost
    );

    let bidirectional_signed_request = tight_request(origin, destination);
    let bidirectional_signed = engine(&signed_graph)
        .route_leg(bidirectional_signed_request)
        .unwrap();
    assert_eq!(bidirectional_signed.edge_ids, signed.edge_ids);
    assert!(
        (bidirectional_signed.cost - signed.cost).abs() <= 1.0e-9,
        "bidirectional signed-corridor cost should match directed: directed={}, bidirectional={}",
        signed.cost,
        bidirectional_signed.cost
    );
}

#[test]
fn major_to_minor_turn_penalty_increases_routed_transition_cost() {
    let origin = (52.0, 21.00002).into();
    let destination = (52.00098, 21.001).into();
    let primary_graph = graph_from_ways(
        "primary-to-primary-transition-cost",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.001, 21.001)], RoadType::Primary),
        ],
    );
    let major_to_minor_graph = graph_from_ways(
        "major-to-minor-transition-cost",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(
                2,
                vec![(52.0, 21.001), (52.001, 21.001)],
                RoadType::Residential,
            ),
        ],
    );

    let primary = engine(&primary_graph)
        .route_leg(tight_request(origin, destination))
        .unwrap();
    let major_to_minor = engine(&major_to_minor_graph)
        .route_leg(tight_request(origin, destination))
        .unwrap();

    assert_eq!(primary.edge_ids, major_to_minor.edge_ids);
    let primary_turns = primary
        .legs
        .iter()
        .flat_map(|leg| &leg.edges)
        .map(|edge| edge.cost_breakdown.turn_penalty)
        .collect::<Vec<_>>();
    let major_to_minor_turns = major_to_minor
        .legs
        .iter()
        .flat_map(|leg| &leg.edges)
        .map(|edge| edge.cost_breakdown.turn_penalty)
        .collect::<Vec<_>>();
    // Phase 0 time metric: turn penalties are now in seconds (legacy 35.0 / 56.0).
    assert_eq!(primary_turns, vec![0.0, 4.2]);
    assert_eq!(major_to_minor_turns, vec![0.0, 6.72]);
    assert!(
        major_to_minor.cost > primary.cost + 2.0,
        "major-to-minor turn should increase routed cost: primary={}, major_to_minor={}",
        primary.cost,
        major_to_minor.cost
    );
}

#[test]
fn tracked_profile_can_prefer_short_track_when_wheeled_avoids_it() {
    let graph = graph_from_ways(
        "profile-costing",
        vec![
            way(1, vec![(52.0, 21.0), (52.001, 21.0015)], RoadType::Primary),
            way(
                2,
                vec![(52.001, 21.0015), (52.0, 21.003)],
                RoadType::Primary,
            ),
            way(3, vec![(52.0, 21.0), (52.0, 21.0015)], RoadType::Track),
            way(4, vec![(52.0, 21.0015), (52.0, 21.003)], RoadType::Track),
        ],
    );

    let wheeled = engine(&graph)
        .route_leg(tight_profile_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00298).into(),
            MobilityProfile::Wheeled,
        ))
        .unwrap();
    let tracked = engine(&graph)
        .route_leg(tight_profile_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00298).into(),
            MobilityProfile::Tracked,
        ))
        .unwrap();

    assert_eq!(wheeled.edge_ids, vec![1, 3]);
    assert_eq!(tracked.edge_ids, vec![5, 7]);
    let wheeled_preview = wheeled
        .recosts
        .iter()
        .find(|summary| summary.profile == MobilityProfile::Wheeled)
        .expect("wheeled recost summary");
    assert!(wheeled_preview.accessible);
    assert!(wheeled_preview.cost.unwrap() > 0.0);
    let tracked_preview = wheeled
        .recosts
        .iter()
        .find(|summary| summary.profile == MobilityProfile::Tracked)
        .expect("tracked recost summary");
    assert!(tracked_preview.accessible);
    assert!(tracked_preview.cost.is_some());
    let tracked_route_preview = tracked
        .recosts
        .iter()
        .find(|summary| summary.profile == MobilityProfile::Tracked)
        .expect("tracked route recost summary");
    assert!(tracked_route_preview.accessible);
    assert!(tracked_route_preview.cost.unwrap() > 0.0);
    assert!(
        wheeled
            .recosts
            .iter()
            .any(|summary| summary.profile == MobilityProfile::OnFoot)
    );
    assert_eq!(wheeled.recosts.len(), 5);
}

#[test]
fn costing_options_can_avoid_motorways() {
    let graph = graph_from_ways(
        "avoid-motorways",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Motorway),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Motorway),
            way(
                3,
                vec![(52.0, 21.0), (52.0005, 21.001)],
                RoadType::Residential,
            ),
            way(
                4,
                vec![(52.0005, 21.001), (52.0, 21.002)],
                RoadType::Residential,
            ),
        ],
    );
    let origin = (52.0, 21.00002).into();
    let destination = (52.0, 21.00198).into();

    let default_route = engine(&graph)
        .route_leg(tight_profile_request(
            origin,
            destination,
            MobilityProfile::Wheeled,
        ))
        .unwrap();
    let mut avoid_request = tight_profile_request(origin, destination, MobilityProfile::Wheeled);
    avoid_request.options.costing.avoid_motorways = 1.0;
    let avoided_route = engine(&graph).route_leg(avoid_request).unwrap();

    assert_eq!(default_route.edge_ids, vec![1, 3]);
    assert_eq!(avoided_route.edge_ids, vec![5, 7]);
    assert_eq!(avoided_route.policy_summary.avoid_motorways, 1.0);
    assert!(
        avoided_route.legs[0]
            .edges
            .iter()
            .all(|edge| edge.road_type != RoadType::Motorway)
    );
}

#[test]
fn costing_options_can_prefer_paved_surface() {
    let mut short_dirt_1 = way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Residential);
    short_dirt_1.surface = Some("dirt".into());
    let mut short_dirt_2 = way(
        2,
        vec![(52.0, 21.001), (52.0, 21.002)],
        RoadType::Residential,
    );
    short_dirt_2.surface = Some("dirt".into());
    // Phase 0 time metric: dirt is slow AND carries a surface cost penalty, so
    // the paved detour must be substantially longer for the short unpaved path to
    // remain the default (fastest-by-cost) choice that `prefer_paved` then flips.
    let mut long_paved_1 = way(
        3,
        vec![(52.0, 21.0), (52.004, 21.001)],
        RoadType::Residential,
    );
    long_paved_1.surface = Some("asphalt".into());
    let mut long_paved_2 = way(
        4,
        vec![(52.004, 21.001), (52.0, 21.002)],
        RoadType::Residential,
    );
    long_paved_2.surface = Some("asphalt".into());
    let graph = graph_from_ways(
        "prefer-paved",
        vec![short_dirt_1, short_dirt_2, long_paved_1, long_paved_2],
    );
    let origin = (52.0, 21.00002).into();
    let destination = (52.0, 21.00198).into();

    let default_route = engine(&graph)
        .route_leg(tight_profile_request(
            origin,
            destination,
            MobilityProfile::Wheeled,
        ))
        .unwrap();
    let mut prefer_paved_request =
        tight_profile_request(origin, destination, MobilityProfile::Wheeled);
    prefer_paved_request.options.costing.prefer_paved = 1.0;
    let paved_route = engine(&graph).route_leg(prefer_paved_request).unwrap();

    assert_eq!(default_route.edge_ids, vec![1, 3]);
    assert_eq!(paved_route.edge_ids, vec![5, 7]);
    assert_eq!(paved_route.policy_summary.prefer_paved, 1.0);
    assert!(
        paved_route.legs[0]
            .edges
            .iter()
            .all(|edge| edge.surface.as_deref() == Some("asphalt"))
    );
}

#[test]
fn vehicle_profiles_use_profile_owned_time_on_rough_tracks() {
    let mut rough_track = way(1, vec![(52.0, 21.0), (52.0, 21.002)], RoadType::Track);
    rough_track.surface = Some("dirt".into());
    rough_track.tracktype = Some("grade3".into());
    rough_track.maxspeed = Some("40".into());
    let graph = graph_from_ways("profile-owned-vehicle-duration", vec![rough_track]);
    let origin = (52.0, 21.00002).into();
    let destination = (52.0, 21.00198).into();

    let wheeled = engine(&graph)
        .route_leg(tight_profile_request(
            origin,
            destination,
            MobilityProfile::Wheeled,
        ))
        .unwrap();
    let tracked = engine(&graph)
        .route_leg(tight_profile_request(
            origin,
            destination,
            MobilityProfile::Tracked,
        ))
        .unwrap();
    let mut emergency_request =
        tight_profile_request(origin, destination, MobilityProfile::Emergency);
    emergency_request
        .options
        .costing
        .emergency_override_strength = 1.0;
    let emergency = engine(&graph).route_leg(emergency_request).unwrap();

    assert_eq!(wheeled.edge_ids, tracked.edge_ids);
    assert_eq!(tracked.edge_ids, emergency.edge_ids);
    assert!(tracked.duration_s < wheeled.duration_s * 0.6);
    assert!(emergency.duration_s <= tracked.duration_s);
    assert!(tracked.legs[0].edges[0].duration_s < wheeled.legs[0].edges[0].duration_s);
    assert!(
        tracked.legs[0].edges[0].cost_breakdown.profile_time_factor
            < wheeled.legs[0].edges[0].cost_breakdown.profile_time_factor
    );
}

#[test]
fn on_foot_prefers_short_path_cut_through_while_wheeled_stays_on_roads() {
    let graph = graph_from_ways(
        "foot-path-cut-through",
        vec![
            way(1, vec![(52.0, 21.0), (52.001, 21.0)], RoadType::Primary),
            way(2, vec![(52.001, 21.0), (52.001, 21.001)], RoadType::Primary),
            way(3, vec![(52.001, 21.001), (52.0, 21.001)], RoadType::Primary),
            foot_only_path(4, vec![(52.0, 21.0), (52.0, 21.0005)]),
            foot_only_path(5, vec![(52.0, 21.0005), (52.0, 21.001)]),
        ],
    );

    let wheeled = engine(&graph)
        .route_leg(tight_profile_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00098).into(),
            MobilityProfile::Wheeled,
        ))
        .unwrap();
    let on_foot = engine(&graph)
        .route_leg(tight_profile_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00098).into(),
            MobilityProfile::OnFoot,
        ))
        .unwrap();

    assert_eq!(wheeled.edge_ids, vec![1, 3, 5]);
    assert_eq!(on_foot.edge_ids, vec![7, 9]);
    let wheeled_route_on_foot_preview = wheeled
        .recosts
        .iter()
        .find(|summary| summary.profile == MobilityProfile::OnFoot)
        .expect("on-foot recost for wheeled route");
    assert!(on_foot.duration_s < wheeled_route_on_foot_preview.duration_s.unwrap());
}

#[test]
fn on_foot_traverses_one_way_against_traffic() {
    // A vehicular one-way residential street (A -> B) that carries pedestrians.
    let graph = graph_from_ways(
        "foot-contraflow",
        vec![one_way_foot_allowed(1, vec![(52.0, 21.0), (52.0, 21.001)])],
    );

    // Walking B -> A runs against the vehicular one-way and must still route.
    let on_foot = engine(&graph)
        .route_leg(tight_profile_request(
            (52.0, 21.00098).into(),
            (52.0, 21.00002).into(),
            MobilityProfile::OnFoot,
        ))
        .unwrap();
    assert_eq!(on_foot.status, RouteStatus::Routed);

    // Driving the wrong way down the same one-way remains impossible.
    let wheeled = engine(&graph).route_leg(tight_profile_request(
        (52.0, 21.00098).into(),
        (52.0, 21.00002).into(),
        MobilityProfile::Wheeled,
    ));
    assert!(wheeled.is_err());
}

#[test]
fn on_foot_uses_walking_time_instead_of_vehicle_time_on_same_edge() {
    let graph = graph_from_ways(
        "foot-duration",
        vec![way(
            1,
            vec![(52.0, 21.0), (52.0, 21.001)],
            RoadType::Residential,
        )],
    );

    let wheeled = engine(&graph)
        .route_leg(tight_profile_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00098).into(),
            MobilityProfile::Wheeled,
        ))
        .unwrap();
    let on_foot = engine(&graph)
        .route_leg(tight_profile_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00098).into(),
            MobilityProfile::OnFoot,
        ))
        .unwrap();

    assert_eq!(wheeled.edge_ids, on_foot.edge_ids);
    assert!(on_foot.duration_s > wheeled.duration_s * 5.0);
    assert!(on_foot.legs[0].edges[0].duration_s > wheeled.legs[0].edges[0].duration_s * 5.0);
    // Phase 0 time metric: `profile_time_factor` is now the edge's true
    // seconds-per-meter (1 / speed), no longer a clamped penalty above distance.
    // Walking the same edge is far slower than driving it, so on-foot's
    // seconds-per-meter is many times the wheeled value, and — because the base
    // cost term is now travel time — the on-foot edge cost is likewise much
    // higher. (The old ProfileTime *penalty* driver / profile_time_penalty_count
    // no longer apply: time is the base of the metric, not an extra penalty.)
    let on_foot_breakdown = on_foot.legs[0].edges[0].cost_breakdown;
    let wheeled_breakdown = wheeled.legs[0].edges[0].cost_breakdown;
    assert!(
        on_foot_breakdown.profile_time_factor > wheeled_breakdown.profile_time_factor * 5.0,
        "on-foot seconds-per-meter {} should dwarf wheeled {}",
        on_foot_breakdown.profile_time_factor,
        wheeled_breakdown.profile_time_factor
    );
    assert_eq!(on_foot.cost_summary.attributed_edge_count, 1);
    assert_eq!(on_foot.cost_summary.costed_edge_count, 1);
    assert!(
        on_foot_breakdown.total_cost > wheeled_breakdown.total_cost * 2.0,
        "on-foot time-dominated cost {} should exceed wheeled {}",
        on_foot_breakdown.total_cost,
        wheeled_breakdown.total_cost
    );
}

#[test]
fn emergency_costing_override_reduces_track_penalty() {
    let graph = graph_from_ways(
        "emergency-costing",
        vec![
            way(1, vec![(52.0, 21.0), (52.001, 21.0015)], RoadType::Primary),
            way(
                2,
                vec![(52.001, 21.0015), (52.0, 21.003)],
                RoadType::Primary,
            ),
            way(3, vec![(52.0, 21.0), (52.0, 21.0015)], RoadType::Track),
            way(4, vec![(52.0, 21.0015), (52.0, 21.003)], RoadType::Track),
        ],
    );
    let mut request = tight_profile_request(
        (52.0, 21.00002).into(),
        (52.0, 21.00298).into(),
        MobilityProfile::Emergency,
    );
    request.options.costing.emergency_override_strength = 1.0;

    let result = engine(&graph).route_leg(request).unwrap();

    assert_eq!(result.edge_ids, vec![5, 7]);
    assert!(
        result.legs[0].edges[0].cost_breakdown.road_type_factor
            < RoadType::Track.cost_factor() as f64 * 1.2
    );
}

#[test]
fn one_way_restriction_reports_no_path_against_direction() {
    let graph = graph_from_ways(
        "one-way",
        vec![one_way(1, vec![(52.0, 21.0), (52.0, 21.001)])],
    );

    let error = engine(&graph)
        .route_leg(LegRequest::new(
            (52.0, 21.001).into(),
            (52.0, 21.0).into(),
            MobilityProfile::Wheeled,
        ))
        .unwrap_err()
        .error;

    assert!(error.is_no_path());
}

#[test]
fn route_diagnostics_include_phase_timing_metadata() {
    let graph = graph_from_ways(
        "route-phase-timing",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
        ],
    );

    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00198).into(),
        ))
        .unwrap();

    assert!(result.diagnostics.correlation_duration_ns > 0);
    assert!(result.diagnostics.search_duration_ns > 0);
    assert!(result.diagnostics.route_leg_build_duration_ns > 0);
    assert!(result.diagnostics.total_compute_duration_ns > 0);
    assert!(
        result.diagnostics.total_compute_duration_ns
            >= result
                .diagnostics
                .correlation_duration_ns
                .max(result.diagnostics.search_duration_ns)
                .max(result.diagnostics.route_leg_build_duration_ns)
    );
}

#[test]
fn disconnected_components_report_no_path() {
    let graph = graph_from_ways(
        "disconnected",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Residential),
            way(
                2,
                vec![(52.01, 21.0), (52.01, 21.001)],
                RoadType::Residential,
            ),
        ],
    );

    let error = engine(&graph)
        .route_leg(LegRequest::new(
            (52.0, 21.0).into(),
            (52.01, 21.001).into(),
            MobilityProfile::Wheeled,
        ))
        .unwrap_err()
        .error;

    assert!(error.is_disconnected() || error.is_no_path());
}

#[test]
fn search_budget_exhaustion_reports_typed_failure() {
    let graph = graph_from_ways(
        "search-budget",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(3, vec![(52.0, 21.002), (52.0, 21.003)], RoadType::Primary),
        ],
    );
    let mut request = tight_request((52.0, 21.00002).into(), (52.0, 21.00298).into());
    request.options.max_visited_edge_count = Some(1);

    let error = engine(&graph).route_leg(request).unwrap_err().error;

    assert!(error.is_search_budget_exceeded());
}

#[test]
fn fixture_graph_can_be_persisted_loaded_and_routed() {
    let graph = graph_from_ways(
        "fixture-smoke",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
        ],
    );
    let dir = tempdir().unwrap();
    let path = dir.path().join("road_graph.mrg");
    persist_road_graph_file(&path, &graph).unwrap();
    let loaded = load_road_graph_file(&path).unwrap();

    let result = engine(&loaded)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.0, 21.00198).into(),
        ))
        .unwrap();

    assert_eq!(result.edge_ids, vec![1, 3]);
    assert!(result.distance_m > 120.0);
}

/// COVERAGE: a request-borne overlay zone reaches the router's edge set for
/// each of the three geometry families.
///
/// This is what survives of
/// `overlay_zone_projection_covers_the_same_edges_as_the_projection_compiler`.
/// That test asserted the entity-borne zone path and the deleted
/// `MeshObjectProjection` compiler covered one edge set — a source-independence
/// property that is meaningless now there is only one source. Its other half is
/// not: it was the only place asserting that `project_route_overlay_geometry`
/// actually covers edges on a real graph for a drawn zone (polygon), a marker or
/// report (point, at the shared 35 m default) and a closure along a way (line).
/// `compute::overlay_zone_tests` covers only the radius arithmetic, so without
/// this a projection that silently returned nothing — a no-go that reaches
/// nowhere, indistinguishable from one that works — would pass every test.
#[test]
fn overlay_zone_projection_covers_edges_for_every_geometry_family() {
    let graph = graph_from_ways(
        "overlay-zone-coverage",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(2, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Primary),
            way(
                3,
                vec![(52.0, 21.0), (52.0008, 21.001)],
                RoadType::Residential,
            ),
            way(
                4,
                vec![(52.0008, 21.001), (52.0, 21.002)],
                RoadType::Residential,
            ),
        ],
    );
    let in_memory = engine(&graph);
    let access = in_memory.route_graph_view();

    let project = |kind: RouteOverlayGeometryKind, coordinates: &[(f64, f64)]| {
        project_route_overlay_geometry(
            access,
            kind,
            &coordinates
                .iter()
                .map(|(lat, lng)| GeoPoint::new(*lat, *lng))
                .collect::<Vec<_>>(),
            35,
        )
        .edge_ids
    };

    // A drawn zone over the western half of the corridor.
    let zone = project(
        RouteOverlayGeometryKind::Polygon,
        &[
            (51.9998, 20.9998),
            (51.9998, 21.0005),
            (52.0002, 21.0005),
            (52.0002, 20.9998),
        ],
    );
    // A closure running the length of the primary corridor.
    let closure = project(
        RouteOverlayGeometryKind::Line,
        &[(52.0, 21.0002), (52.0, 21.0018)],
    );
    // A marker dropped on the first way.
    let marker = project(RouteOverlayGeometryKind::Point, &[(52.0, 21.0005)]);

    for (label, edge_ids) in [("zone", &zone), ("closure", &closure), ("marker", &marker)] {
        assert!(
            !edge_ids.is_empty(),
            "{label}: an overlay that covers no edge never reaches the router"
        );
    }

    // …and it discriminates. The same three shapes, 5 km away from every way in
    // the graph, must cover nothing — otherwise "covers edges" above would hold
    // just as well for a projection that returned the whole graph, and a no-go
    // drawn on the far side of the map would block the route in front of you.
    let far_zone = project(
        RouteOverlayGeometryKind::Polygon,
        &[
            (52.0498, 21.0498),
            (52.0498, 21.0505),
            (52.0502, 21.0505),
            (52.0502, 21.0498),
        ],
    );
    let far_closure = project(
        RouteOverlayGeometryKind::Line,
        &[(52.05, 21.0502), (52.05, 21.0518)],
    );
    let far_marker = project(RouteOverlayGeometryKind::Point, &[(52.05, 21.0505)]);
    for (label, edge_ids) in [
        ("zone", &far_zone),
        ("closure", &far_closure),
        ("marker", &far_marker),
    ] {
        assert!(
            edge_ids.is_empty(),
            "{label}: a shape 5 km off the graph covered {edge_ids:?}"
        );
    }
}

// ---------------------------------------------------------------------------
// Persisted-package helpers
// ---------------------------------------------------------------------------

fn persisted_tile_reader(
    graph: &RoadGraph,
) -> (
    tempfile::TempDir,
    Arc<RoutingPackage>,
    RoadGraphPackageManifest,
) {
    let manifest = road_graph_manifest(graph);
    let (dir, package) = persisted_package(graph);
    (dir, package, manifest)
}

// ---------------------------------------------------------------------------
// perf/routing-and-pipeline change #4: complex-restriction meeting-point fix
// ---------------------------------------------------------------------------
fn via_way_corridor_ways() -> Vec<RoutableWay> {
    let mut ways = Vec::new();
    // Straight eastbound corridor A0..A10 (edges w1..w10).
    for i in 0..10i64 {
        let lng = 21.0 + f64::from(i as i32) * 0.01;
        ways.push(way(
            1 + i,
            vec![(52.0, lng), (52.0, lng + 0.01)],
            RoadType::Primary,
        ));
    }
    // Detour near the destination end bypassing node A8 (between w8 and w9),
    // i.e. an alternative from A7 (21.07) over to A9 (21.09).
    ways.push(way(
        50,
        vec![(52.0, 21.07), (52.006, 21.08)],
        RoadType::Primary,
    ));
    ways.push(way(
        51,
        vec![(52.006, 21.08), (52.0, 21.09)],
        RoadType::Primary,
    ));
    ways
}

// Eastbound edge ids for the corridor (odd ids): w7 -> 13, w8 (via) -> 15,
// w9 -> 17. The complex restriction bans the sequence 13 -> 15 -> 17
// (A6 -> A7 -> A8 -> A9). A detour (edges 21, 23) leaves the corridor at A7 and
// rejoins at A9, providing a legal alternative that skips the via edge 15.
//
// The restriction's `to` edge (17) sits near the destination, so on a
// mid-corridor bidirectional meeting point it is only ever reached during
// reverse expansion. Reverse expansion checks simple restrictions only, so the
// hierarchy meeting point must apply the same full-path complex-restriction
// scan as the flat pass or it will emit the illegal straight-through route.
#[test]
fn complex_via_way_restriction_matches_reference_route() {
    let mut graph = graph_from_ways("via-way-corridor-restriction", via_way_corridor_ways());
    restrict_complex_sequence(&mut graph, 13, vec![15], 17);
    let origin = (52.0, 21.00002).into();
    let destination = (52.0, 21.09998).into();

    let route = assert_matches_reference(&graph, origin, destination, "via-way");
    // The illegal straight route uses the banned via edge 15.
    assert!(
        !route.edge_ids.contains(&15),
        "the route must not traverse the restricted via edge 15: {:?}",
        route.edge_ids
    );
}

// ---------------------------------------------------------------------------
// perf/routing-and-pipeline change #2: admissible heuristic strengthening
// ---------------------------------------------------------------------------
// A straight eastbound corridor (the unique cheapest path) with multi-edge
// perpendicular dead-end spurs at each interior node. A weaker heuristic
// wanders into the spurs; a stronger admissible heuristic prunes that lateral
// exploration. The optimal route is unchanged either way.
fn heuristic_corridor_with_spurs_ways() -> Vec<RoutableWay> {
    let mut ways = Vec::new();
    let corridor_len = 20i64;
    for i in 0..corridor_len {
        let lng = 21.0 + f64::from(i as i32) * 0.01;
        ways.push(way(
            1 + i,
            vec![(52.0, lng), (52.0, lng + 0.01)],
            RoadType::Primary,
        ));
    }
    // 3-edge dead-end spurs heading north from each interior corridor node.
    let mut spur_id = 1_000i64;
    for i in 3..(corridor_len - 3) {
        let lng = 21.0 + f64::from(i as i32) * 0.01;
        for step in 0..3 {
            let lat = 52.0 + f64::from(step) * 0.01;
            ways.push(way(
                spur_id,
                vec![(lat, lng), (lat + 0.01, lng)],
                RoadType::Primary,
            ));
            spur_id += 1;
        }
    }
    ways
}

fn heuristic_directed_request(origin: GeoPoint, destination: GeoPoint) -> LegRequest {
    tight_request(origin, destination)
}

#[test]
fn admissible_heuristic_strengthening_keeps_route_and_reduces_visits() {
    let graph = graph_from_ways(
        "heuristic-corridor-spurs",
        heuristic_corridor_with_spurs_ways(),
    );
    let origin = (52.0, 21.00002).into();
    let destination = (52.0, 21.19998).into();
    let (_dir, package, _m) = persisted_tile_reader(&graph);
    let route = RoutingEngine::from_package(package)
        .route_leg(heuristic_directed_request(origin, destination))
        .expect("directed route");

    // Optimal route is the unique straight corridor (odd eastbound edge ids).
    let expected: Vec<u64> = (0..20u64).map(|i| 1 + i * 2).collect();
    assert_eq!(
        route.edge_ids, expected,
        "strengthened heuristic must not change the optimal route"
    );
    // Visited-edge count must strictly drop versus the legacy 0.25 bound.
    assert!(
        route.diagnostics.visited_edge_count < HEURISTIC_VISITED_BASELINE,
        "expected visited_edge_count {} to drop below legacy baseline {}",
        route.diagnostics.visited_edge_count,
        HEURISTIC_VISITED_BASELINE
    );
}

// Captured from a legacy (0.25 bound) run; see change #2 notes.
const HEURISTIC_VISITED_BASELINE: usize = 195;

// ---------------------------------------------------------------------------
// perf/routing-and-pipeline change #3: bidirectional termination parity harness
// ---------------------------------------------------------------------------
// The package route equals the reference oracle's (plain Dijkstra over the
// road graph's own records): same edges, same cost up to the package
// columns' f32 rounding.
fn assert_matches_reference(
    graph: &RoadGraph,
    origin: GeoPoint,
    destination: GeoPoint,
    label: &str,
) -> LegResult {
    let (_dir, package, _manifest) = persisted_tile_reader(graph);
    let compact = RoutingEngine::from_package(package)
        .route_leg(tight_request(origin, destination))
        .unwrap_or_else(|error| panic!("{label}: package route failed: {error:?}"));
    let reference = reference(graph)
        .route_leg(tight_request(origin, destination))
        .unwrap_or_else(|error| panic!("{label}: reference route failed: {error:?}"));
    assert_eq!(
        compact.edge_ids, reference.edge_ids,
        "{label}: edge_ids differ"
    );
    assert!(
        (compact.cost - reference.cost).abs() <= 1e-4 * reference.cost.max(1.0),
        "{label}: cost differs compact={} reference={}",
        compact.cost,
        reference.cost
    );
    compact
}

#[test]
fn package_routes_match_reference_across_diverse_routes() {
    // Short single-segment route.
    let short = graph_from_ways(
        "term-parity-short",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.01)], RoadType::Primary),
            way(2, vec![(52.0, 21.01), (52.0, 21.02)], RoadType::Primary),
        ],
    );
    assert_matches_reference(
        &short,
        (52.0, 21.00002).into(),
        (52.0, 21.01998).into(),
        "short",
    );

    // Long straight corridor.
    let corridor = graph_from_ways("term-parity-corridor", via_way_corridor_ways());
    assert_matches_reference(
        &corridor,
        (52.0, 21.00002).into(),
        (52.0, 21.09998).into(),
        "corridor",
    );

    // Corridor with spurs (lateral alternatives / asymmetric expansion).
    let spurs = graph_from_ways("term-parity-spurs", heuristic_corridor_with_spurs_ways());
    assert_matches_reference(
        &spurs,
        (52.0, 21.00002).into(),
        (52.0, 21.19998).into(),
        "spurs",
    );

    // Detour graph where the straight path is blocked by a complex restriction.
    let mut restricted = graph_from_ways("term-parity-restricted", via_way_corridor_ways());
    restrict_complex_sequence(&mut restricted, 13, vec![15], 17);
    assert_matches_reference(
        &restricted,
        (52.0, 21.00002).into(),
        (52.0, 21.09998).into(),
        "restricted",
    );

    // Asymmetric graph: dense fan of parallel detours near the origin, a sparse
    // tail near the destination. This stresses termination because the forward
    // frontier can prove its bound while reverse labels are still settling.
    let mut asym = Vec::new();
    for i in 0..8i64 {
        let lng = 21.0 + f64::from(i as i32) * 0.01;
        asym.push(way(
            1 + i,
            vec![(52.0, lng), (52.0, lng + 0.01)],
            RoadType::Primary,
        ));
    }
    // parallel bows over the first few segments (alternate equal-ish paths)
    let mut bow_id = 500i64;
    for i in 0..4i64 {
        let lng = 21.0 + f64::from(i as i32) * 0.01;
        asym.push(way(
            bow_id,
            vec![(52.0, lng), (52.003, lng + 0.005)],
            RoadType::Secondary,
        ));
        asym.push(way(
            bow_id + 1,
            vec![(52.003, lng + 0.005), (52.0, lng + 0.01)],
            RoadType::Secondary,
        ));
        bow_id += 2;
    }
    let asym_graph = graph_from_ways("term-parity-asym", asym);
    assert_matches_reference(
        &asym_graph,
        (52.0, 21.00002).into(),
        (52.0, 21.07998).into(),
        "asymmetric",
    );
}

// ---------------------------------------------------------------------------
// Hierarchy shortcut-edge consumption.
//
// The bidirectional tile-hierarchy pass relaxes restricted-contraction shortcut
// edges, collapsing degree-2 Through corridors into a single frontier label.
// The consumed route MUST be EXACTLY the detail-parity route: shortcut-enabled
// == shortcut-disabled (== directed) in edge ids, cost, distance, duration and
// applied restrictions. These fixtures are the RED/GREEN gate for that.
// ---------------------------------------------------------------------------

// Endpoint (boundary) turn-cost exactness is NOT covered by a bespoke bent
// full-route fixture: it is guaranteed by construction and proven at scale. The
// shortcut relaxation computes the entry turn with the SAME
// `transition_cost_for_records(previous, first_base_edge)` call base expansion
// uses, and carries the real last base edge as the label's edge_key so exit
// turns (next expansion + meeting connection) are likewise computed on the real
// edge. The unit-level guarantee is the shared turn function; the integrated
// guarantee is the real-archive parity gate, where thousands of shortcuts are
// relaxed at real highway boundaries (each with real turn geometry) and cost
// matches the shortcut-disabled AND directed searches to 1e-9. Any dropped or
// baked endpoint turn would surface there as a cost divergence.

// ---------------------------------------------------------------------------
// Phase 3b: monotone climb/descend level policy. These fixtures
// exercise the runtime policy directly. Because default tail windows (5 km) are
// larger than any synthetic fixture, both tests tighten the tails (120 m) so the
// tiny graph actually has a hierarchical "middle" where the monotone rule bites.
// ---------------------------------------------------------------------------

fn pedestrian_plaza(id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
    let mut plaza = way(id, points, RoadType::Path);
    plaza.highway = Some("pedestrian".into());
    plaza.area = Some("yes".into());
    plaza.foot = Some("yes".into());
    plaza.motor_vehicle = Some("no".into());
    plaza.bicycle = Some("no".into());
    plaza
}

fn plaza_foot_connector(id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
    let mut connector = way(id, points, RoadType::Path);
    connector.highway = Some("footway".into());
    connector.foot = Some("yes".into());
    connector.motor_vehicle = Some("no".into());
    connector.bicycle = Some("no".into());
    connector
}

fn synthetic_plaza_edge_ids(graph: &RoadGraph) -> BTreeSet<u64> {
    let synthetic_infos: BTreeSet<u64> = graph
        .shared_edge_infos
        .iter()
        .filter(|info| info.osm_way_id < 0)
        .map(|info| info.shared_edge_info_id)
        .collect();
    graph
        .graph
        .edge_weights()
        .filter(|edge| synthetic_infos.contains(&edge.shared_edge_info_id))
        .map(|edge| edge.edge_id)
        .collect()
}

#[test]
fn foot_route_crosses_pedestrian_plaza_interior() {
    // Square pedestrian plaza with foot connectors reaching opposing corners.
    let a = (50.0620, 19.9360); // NW
    let b = (50.0620, 19.9380); // NE
    let c = (50.0600, 19.9380); // SE
    let d = (50.0600, 19.9360); // SW
    let west = (50.0600, 19.9340);
    let east = (50.0620, 19.9400);
    let graph = graph_from_ways(
        "plaza-foot-crossing",
        vec![
            pedestrian_plaza(1, vec![a, b, c, d, a]),
            plaza_foot_connector(2, vec![west, d]),
            plaza_foot_connector(3, vec![b, east]),
        ],
    );
    let synthetic = synthetic_plaza_edge_ids(&graph);
    assert!(
        !synthetic.is_empty(),
        "plaza interior star edges must be synthesized"
    );

    let result = engine(&graph)
        .route_leg(tight_profile_request(
            west.into(),
            east.into(),
            MobilityProfile::OnFoot,
        ))
        .expect("foot route across plaza");

    assert!(
        result.edge_ids.iter().any(|id| synthetic.contains(id)),
        "foot route should traverse the plaza interior star"
    );
    // The diagonal crossing (~550 m) must beat circling the perimeter (~650 m).
    assert!(
        result.distance_m < 600.0,
        "foot crossing distance {} should be shorter than the perimeter path",
        result.distance_m
    );
}

#[test]
fn wheeled_route_declines_foot_only_plaza_interior() {
    // Roads reach the plaza corners (making them anchors) but the plaza and its
    // synthesized interior are foot-only, so a wheeled route must detour around.
    let a = (50.0620, 19.9360);
    let b = (50.0620, 19.9380);
    let c = (50.0600, 19.9380);
    let d = (50.0600, 19.9360);
    let road_w = (50.0600, 19.9320);
    let road_e = (50.0620, 19.9420);
    let graph = graph_from_ways(
        "plaza-wheeled-detour",
        vec![
            pedestrian_plaza(1, vec![a, b, c, d, a]),
            way(2, vec![road_w, d], RoadType::Residential),
            way(3, vec![b, road_e], RoadType::Residential),
            way(
                4,
                vec![road_w, (50.0560, 19.9320), (50.0560, 19.9420), road_e],
                RoadType::Residential,
            ),
        ],
    );
    let synthetic = synthetic_plaza_edge_ids(&graph);
    assert!(
        !synthetic.is_empty(),
        "plaza interior star edges must be synthesized"
    );

    let result = engine(&graph)
        .route_leg(tight_profile_request(
            road_w.into(),
            road_e.into(),
            MobilityProfile::Wheeled,
        ))
        .expect("wheeled route around plaza");

    assert!(
        result.edge_ids.iter().all(|id| !synthetic.contains(id)),
        "wheeled route must not use the foot-only plaza interior edges"
    );
    // The wheeled path is forced onto the southern detour rather than cutting
    // across the pedestrian square.
    assert!(
        result.distance_m > 1000.0,
        "wheeled route distance {} should reflect the detour around the plaza",
        result.distance_m
    );
}

fn way_with_nodes(
    id: i64,
    points: Vec<(f64, f64)>,
    node_ids: Vec<i64>,
    road_type: RoadType,
) -> RoutableWay {
    let mut route = way(id, points, road_type);
    route.source_node_ids = node_ids.into_iter().map(Some).collect();
    route
}

fn graph_from_ways_with_barriers(
    graph_id: &str,
    ways: Vec<RoutableWay>,
    barriers: HashMap<i64, NodeBarrierTags>,
) -> RoadGraph {
    let mut builder = RoadGraphBuilder::new(graph_id);
    for way in ways {
        builder.push_way(way);
    }
    builder.set_barrier_node_tags(barriers);
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    graph
}

/// A direct way S-M-E (M is the middle barrier node) plus a longer S-N-E detour,
/// with origin/destination stubs so both endpoints snap off the direct way.
fn barrier_detour_ways() -> Vec<RoutableWay> {
    let s = (52.0, 21.0);
    let m = (52.0, 21.001);
    let e = (52.0, 21.002);
    let n = (52.002, 21.001);
    let o = (52.0, 20.999);
    let d = (52.0, 21.003);
    vec![
        way_with_nodes(1, vec![s, m, e], vec![1, 2, 3], RoadType::Residential),
        way_with_nodes(2, vec![s, n], vec![1, 4], RoadType::Residential),
        way_with_nodes(3, vec![n, e], vec![4, 3], RoadType::Residential),
        way_with_nodes(10, vec![o, s], vec![10, 1], RoadType::Residential),
        way_with_nodes(11, vec![e, d], vec![3, 11], RoadType::Residential),
    ]
}

#[test]
fn vehicle_detours_bollard_while_foot_passes_through() {
    let mut barriers = HashMap::new();
    barriers.insert(
        2,
        NodeBarrierTags {
            barrier: "bollard".to_string(),
            ..Default::default()
        },
    );
    let graph = graph_from_ways_with_barriers("bollard-detour", barrier_detour_ways(), barriers);

    let way1 = way_edge_ids(&graph, 1);
    let way2 = way_edge_ids(&graph, 2);
    let way3 = way_edge_ids(&graph, 3);

    let origin: GeoPoint = (52.0, 20.9995).into();
    let destination: GeoPoint = (52.0, 21.0025).into();

    let foot = engine(&graph)
        .route_leg(tight_profile_request(
            origin,
            destination,
            MobilityProfile::OnFoot,
        ))
        .expect("foot route");
    let vehicle = engine(&graph)
        .route_leg(tight_profile_request(
            origin,
            destination,
            MobilityProfile::Wheeled,
        ))
        .expect("vehicle route");

    let foot_edges = foot.edge_ids.iter().copied().collect::<BTreeSet<_>>();
    let vehicle_edges = vehicle.edge_ids.iter().copied().collect::<BTreeSet<_>>();

    // Foot walks straight through the bollard on the direct way.
    assert!(!foot_edges.is_disjoint(&way1));
    assert!(foot_edges.is_disjoint(&way2));
    assert!(foot_edges.is_disjoint(&way3));

    // The vehicle cannot cross the bollard and detours via the S-N-E arc.
    assert!(!vehicle_edges.is_disjoint(&way2));
    assert!(!vehicle_edges.is_disjoint(&way3));
    assert!(vehicle_edges.is_disjoint(&way1));

    assert!(
        foot.distance_m < vehicle.distance_m,
        "foot {} should be shorter than the vehicle detour {}",
        foot.distance_m,
        vehicle.distance_m
    );
}

#[test]
fn foot_refuses_locked_private_gate_and_detours() {
    let mut barriers = HashMap::new();
    barriers.insert(
        2,
        NodeBarrierTags {
            barrier: "gate".to_string(),
            access: Some("private".to_string()),
            locked: Some("yes".to_string()),
            ..Default::default()
        },
    );
    let graph =
        graph_from_ways_with_barriers("locked-gate-detour", barrier_detour_ways(), barriers);

    let way1 = way_edge_ids(&graph, 1);
    let way2 = way_edge_ids(&graph, 2);
    let way3 = way_edge_ids(&graph, 3);

    let origin: GeoPoint = (52.0, 20.9995).into();
    let destination: GeoPoint = (52.0, 21.0025).into();

    let foot = engine(&graph)
        .route_leg(tight_profile_request(
            origin,
            destination,
            MobilityProfile::OnFoot,
        ))
        .expect("foot route detours around the locked gate");
    let foot_edges = foot.edge_ids.iter().copied().collect::<BTreeSet<_>>();

    // Foot refuses the locked gate and takes the longer S-N-E arc.
    assert!(!foot_edges.is_disjoint(&way2));
    assert!(!foot_edges.is_disjoint(&way3));
    assert!(foot_edges.is_disjoint(&way1));

    // Without the gate the same profile would take the direct way instead.
    let open_graph =
        graph_from_ways_with_barriers("open-gate-baseline", barrier_detour_ways(), HashMap::new());
    let open_foot = engine(&open_graph)
        .route_leg(tight_profile_request(
            origin,
            destination,
            MobilityProfile::OnFoot,
        ))
        .expect("baseline foot route");
    let open_edges = open_foot.edge_ids.iter().copied().collect::<BTreeSet<_>>();
    assert!(!open_edges.is_disjoint(&way_edge_ids(&open_graph, 1)));
    assert!(open_foot.distance_m < foot.distance_m);
}

/// A gate costs a delay where it applies: where a way ends at
/// it, or where every way passes through it, never along a way that passes
/// through it while another ends there. The reference oracle agrees.
#[test]
fn gate_delays_routes_under_decision_d2() {
    let gate = || {
        let mut barriers = HashMap::new();
        barriers.insert(
            2,
            NodeBarrierTags {
                barrier: "gate".to_string(),
                ..Default::default()
            },
        );
        barriers
    };
    // A side way from M that ends there.
    let with_side = || {
        let mut ways = barrier_detour_ways();
        ways.push(way_with_nodes(
            12,
            vec![(52.0, 21.001), (51.999, 21.001)],
            vec![2, 12],
            RoadType::Residential,
        ));
        ways
    };
    let origin: GeoPoint = (52.0, 20.9995).into();
    let through: GeoPoint = (52.0, 21.0025).into();
    let side: GeoPoint = (51.9993, 21.001).into();
    let cost = |graph: &RoadGraph, destination: GeoPoint, profile: MobilityProfile| {
        let request = tight_profile_request(origin, destination, profile);
        let route = engine(graph).route_leg(request.clone()).expect("route");
        let oracle = reference(graph)
            .route_leg(request)
            .expect("reference route");
        assert_eq!(route.edge_ids, oracle.edge_ids, "{profile:?}");
        assert!((route.cost - oracle.cost).abs() < 1e-6, "{profile:?}");
        route.cost
    };
    let only_way = graph_from_ways_with_barriers("gate-d2-only", barrier_detour_ways(), gate());
    let only_way_open =
        graph_from_ways_with_barriers("gate-d2-only-open", barrier_detour_ways(), HashMap::new());
    let side_way = graph_from_ways_with_barriers("gate-d2-side", with_side(), gate());
    let side_way_open =
        graph_from_ways_with_barriers("gate-d2-side-open", with_side(), HashMap::new());
    let walk = MobilityProfile::OnFoot;
    // The only way passes through: the gate applies.
    assert!(
        (cost(&only_way, through, walk)
            - cost(&only_way_open, through, walk)
            - wayfarer::GATE_ON_FOOT_S)
            .abs()
            < 1e-6
    );
    // A side way ends there: free along the way, charged turning onto it.
    assert!((cost(&side_way, through, walk) - cost(&side_way_open, through, walk)).abs() < 1e-6);
    assert!(
        (cost(&side_way, side, walk) - cost(&side_way_open, side, walk) - wayfarer::GATE_ON_FOOT_S)
            .abs()
            < 1e-6
    );
    // Vehicles pay their own delay.
    let car = MobilityProfile::Default;
    assert!(
        (cost(&side_way, side, car) - cost(&side_way_open, side, car) - wayfarer::GATE_VEHICLE_S)
            .abs()
            < 1e-6
    );
}

#[test]
fn isochrone_expansion_budget_stops_the_search_and_reports_truncation() {
    let graph = graph_from_ways(
        "isochrone-budget",
        (0..12)
            .map(|index| {
                let start = 21.0 + 0.001 * f64::from(index);
                way(
                    i64::from(index) + 1,
                    vec![(52.0, start), (52.0, start + 0.001)],
                    RoadType::Primary,
                )
            })
            .collect(),
    );
    let mut request = RouteIsochroneRequest::new(
        (52.0, 21.00002).into(),
        MobilityProfile::Wheeled,
        vec![RouteIsochroneContour {
            contour: 1_000.0,
            metric: RouteIsochroneMetric::Cost,
        }],
    );
    request.options.max_correlation_distance_m = 15;
    let engine = engine(&graph);
    let full = engine.isochrone(request.clone()).unwrap();
    assert!(!full.truncated);

    request.max_expansion_edge_count = Some(3);
    let cut = engine.isochrone(request).unwrap();
    assert!(cut.truncated);
    assert_eq!(cut.visited_edge_count, 4);
    assert!(cut.buckets[0].edge_ids.len() < full.buckets[0].edge_ids.len());
    assert!(
        cut.buckets[0]
            .edge_ids
            .iter()
            .all(|edge| full.buckets[0].edge_ids.contains(edge))
    );
}

#[test]
fn isochrone_result_without_truncated_field_deserializes_as_complete() {
    let graph = graph_from_ways(
        "isochrone-serde",
        vec![way(
            1,
            vec![(52.0, 21.0), (52.0, 21.001)],
            RoadType::Primary,
        )],
    );
    let mut request = RouteIsochroneRequest::new(
        (52.0, 21.00002).into(),
        MobilityProfile::Wheeled,
        vec![RouteIsochroneContour {
            contour: 100.0,
            metric: RouteIsochroneMetric::Cost,
        }],
    );
    request.options.max_correlation_distance_m = 15;
    let result = engine(&graph).isochrone(request).unwrap();
    let mut json = serde_json::to_value(&result).unwrap();
    json.as_object_mut().unwrap().remove("truncated");
    let decoded: wayfarer::RouteIsochroneResult = serde_json::from_value(json).unwrap();
    assert!(!decoded.truncated);
    assert_eq!(decoded, result);
}

#[test]
fn guidance_places_turns_at_junctions_of_curved_multi_vertex_ways() {
    // "Main" wiggles east over two ways of several vertices each, then the
    // route turns right (south) onto "Side".
    let mut main_a = way(
        1,
        vec![
            (52.0, 21.0),
            (52.0001, 21.0005),
            (52.0, 21.001),
            (52.0001, 21.0015),
            (52.0, 21.002),
        ],
        RoadType::Primary,
    );
    main_a.name = Some("Main".into());
    let mut main_b = way(
        2,
        vec![(52.0, 21.002), (52.00005, 21.0025), (52.0, 21.003)],
        RoadType::Primary,
    );
    main_b.name = Some("Main".into());
    let mut side = way(
        3,
        vec![(52.0, 21.003), (51.9995, 21.00305), (51.999, 21.003)],
        RoadType::Primary,
    );
    side.name = Some("Side".into());
    let graph = graph_from_ways("curved-guidance", vec![main_a, main_b, side]);

    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (51.99905, 21.003).into(),
        ))
        .unwrap();
    let leg = &result.legs[0];
    assert_eq!(leg.edges.len(), 3);
    assert!(leg.geometry.len() > 6);

    let kinds = result
        .maneuvers
        .iter()
        .map(|maneuver| maneuver.kind)
        .collect::<Vec<_>>();
    assert_eq!(
        kinds,
        vec![
            RouteManeuverKind::Start,
            RouteManeuverKind::Turn,
            RouteManeuverKind::Destination
        ]
    );
    let (start, turn) = (&result.maneuvers[0], &result.maneuvers[1]);
    // At the junction vertex, not at the edge index read as a shape index.
    let junction = leg.geometry[turn.begin_shape_index];
    assert!(junction.distance_to(GeoPoint::new(52.0, 21.003)) < 0.5);
    assert_eq!(
        turn.begin_shape_index,
        leg.shape_segments[2].begin_shape_index
    );
    assert_eq!(turn.road_name.as_deref(), Some("Side"));
    let angle = turn.turn_angle_deg.unwrap();
    assert!((70..=110).contains(&angle), "turn angle {angle}");
    assert!(turn.instruction.starts_with("Turn right"));

    // Start runs over both "Main" edges, up to the turn.
    let main_distance = leg.edges[0].distance_m + leg.edges[1].distance_m;
    assert!((start.distance_m - main_distance).abs() < 1e-6);
    assert!((start.duration_s - (leg.edges[0].duration_s + leg.edges[1].duration_s)).abs() < 1e-6);
    assert_eq!(start.end_shape_index, turn.begin_shape_index);
    assert!((turn.distance_m - leg.edges[2].distance_m).abs() < 1e-6);
    assert_eq!(turn.end_shape_index, leg.geometry.len() - 1);
    assert!((start.distance_m + turn.distance_m - result.distance_m).abs() < 1.0);
}

#[test]
fn guidance_reads_a_right_turn_at_a_four_arm_junction_as_a_turn() {
    let graph = graph_from_ways(
        "four-arm-turn",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(
                2,
                vec![(52.0, 21.001), (52.001, 21.001)],
                RoadType::Secondary,
            ),
            way(3, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Secondary),
            way(
                4,
                vec![(52.0, 21.001), (51.999, 21.001)],
                RoadType::Secondary,
            ),
        ],
    );
    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (51.99902, 21.001).into(),
        ))
        .unwrap();

    let turn = result
        .maneuvers
        .iter()
        .find(|maneuver| maneuver.road_name.as_deref() == Some("Way 4"))
        .expect("maneuver onto Way 4");
    assert_eq!(turn.kind, RouteManeuverKind::Turn);
    assert!(turn.instruction.starts_with("Turn right onto Way 4"));
    let edge = &result.legs[0].edges[1];
    // The other two arms: straight on and left.
    let mut branches = edge.junction_branch_angles_deg.clone();
    branches.sort_unstable();
    assert_eq!(branches.len(), 2);
    assert!((-95..=-85).contains(&branches[0]), "{branches:?}");
    assert!((-5..=5).contains(&branches[1]), "{branches:?}");
}

#[test]
fn guidance_counts_roundabout_exits_the_profile_can_take() {
    // A one-way ring W -> S -> SE -> E -> N -> W with two-way arms at every
    // ring node but SE, which has a footway only.
    let ring = |id: i64, points: Vec<(f64, f64)>| {
        let mut ring = roundabout_way(id, points);
        ring.directional_access = DirectionalAccess::Forward;
        ring.name = Some("Ring".into());
        ring
    };
    let (w, s, se, e, n) = (
        (52.0, 21.0017),
        (51.9998, 21.002),
        (51.99985, 21.00222),
        (52.0, 21.0023),
        (52.0002, 21.002),
    );
    let mut footway = way(5, vec![se, (51.9990, 21.0030)], RoadType::Path);
    footway.vehicle = Some("no".into());
    let graph = graph_from_ways(
        "roundabout-exits",
        vec![
            way(1, vec![(52.0, 21.0), w], RoadType::Primary),
            way(2, vec![s, (51.998, 21.002)], RoadType::Primary),
            way(3, vec![e, (52.0, 21.004)], RoadType::Primary),
            way(4, vec![n, (52.002, 21.002)], RoadType::Primary),
            footway,
            ring(10, vec![w, s]),
            ring(11, vec![s, se]),
            ring(12, vec![se, e]),
            ring(13, vec![e, n]),
            ring(14, vec![n, w]),
        ],
    );
    let result = engine(&graph)
        .route_leg(tight_request(
            (52.0, 21.00002).into(),
            (52.0019, 21.002).into(),
        ))
        .unwrap();
    let leg = &result.legs[0];
    assert_eq!(
        leg.edges.iter().filter(|edge| edge.is_roundabout).count(),
        4,
        "{:?}",
        result.edge_ids
    );

    let kinds = result
        .maneuvers
        .iter()
        .map(|maneuver| maneuver.kind)
        .collect::<Vec<_>>();
    // No turns on the ring or at its exit.
    assert_eq!(
        kinds,
        vec![
            RouteManeuverKind::Start,
            RouteManeuverKind::Roundabout,
            RouteManeuverKind::Destination
        ]
    );
    let roundabout = &result.maneuvers[1];
    // S and E are exits; SE (footway) is not one for this profile.
    assert_eq!(roundabout.roundabout_exit_number, Some(3));
    assert_eq!(
        roundabout.instruction,
        "Enter roundabout and take exit 3 onto Way 4"
    );
    assert_eq!(roundabout.concise_instruction, "Way 4");
    assert_eq!(
        roundabout
            .instruction_parameters
            .get("exit_road_name")
            .map(String::as_str),
        Some("Way 4")
    );
    // The roundabout maneuver runs over the ring and the exit road.
    let covered: f64 = leg.edges[1..].iter().map(|edge| edge.distance_m).sum();
    assert!((roundabout.distance_m - covered).abs() < 1e-6);
}

#[test]
fn overlay_routes_keep_turn_penalties_junction_counts_and_recosts() {
    let graph = graph_from_ways(
        "overlay-attribution",
        vec![
            way(1, vec![(52.0, 21.0), (52.0, 21.001)], RoadType::Primary),
            way(
                2,
                vec![(52.0, 21.001), (52.001, 21.001)],
                RoadType::Secondary,
            ),
            way(3, vec![(52.0, 21.001), (52.0, 21.002)], RoadType::Secondary),
        ],
    );
    let engine = engine(&graph);
    let request = tight_request((52.0, 21.0005).into(), (52.00098, 21.001).into());
    let plain = engine.route_leg(request.clone()).unwrap();
    // A closure behind the origin: the route is unchanged, but it is built
    // from overlay steps.
    let mut closed = request;
    closed.options.dynamic_overlay = Some(wayfarer::RouteDynamicOverlay::from_blocked_ranges(&[
        RouteEdgeRange {
            edge_id: plain.edge_ids[0],
            start_fraction: 0.0,
            end_fraction: 0.1,
        },
    ]));
    let overlay = engine.route_leg(closed).unwrap();

    assert_eq!(overlay.edge_ids, plain.edge_ids);
    assert!(
        overlay.legs[0].shape_segments[0]
            .quality_flags
            .contains(&RouteShapeQualityFlag::SsvgPartialEdgeSplit)
    );
    let (plain_turn, overlay_turn) = (&plain.legs[0].edges[1], &overlay.legs[0].edges[1]);
    assert!(plain_turn.cost_breakdown.turn_penalty > 0.0);
    assert_eq!(
        overlay_turn.cost_breakdown.turn_penalty,
        plain_turn.cost_breakdown.turn_penalty
    );
    assert!(plain_turn.junction_outgoing_edge_count > 0);
    assert_eq!(
        overlay_turn.junction_outgoing_edge_count,
        plain_turn.junction_outgoing_edge_count
    );
    assert_eq!(
        overlay_turn.junction_incoming_edge_count,
        plain_turn.junction_incoming_edge_count
    );
    assert_eq!(
        overlay_turn.junction_branch_angles_deg,
        plain_turn.junction_branch_angles_deg
    );
    assert_eq!(
        overlay.cost_summary.turn_penalty_total,
        plain.cost_summary.turn_penalty_total
    );
    assert_eq!(overlay.recosts.len(), plain.recosts.len());
    assert!(!overlay.recosts.is_empty());
    for (overlay_recost, plain_recost) in overlay.recosts.iter().zip(&plain.recosts) {
        assert_eq!(overlay_recost.profile, plain_recost.profile);
        assert_eq!(overlay_recost.accessible, plain_recost.accessible);
        assert!((overlay_recost.distance_m - plain_recost.distance_m).abs() < 1e-6);
        assert_eq!(
            overlay_recost.turn_penalty_total,
            plain_recost.turn_penalty_total
        );
    }
}

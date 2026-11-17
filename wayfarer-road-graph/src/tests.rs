//! Tests for `lib.rs`.

use super::*;
use tempfile::tempdir;
use wayfarer_package::barrier::BarrierAccess;

fn road_way(access: Option<&str>) -> RoutableWay {
    RoutableWay {
        osm_way_id: 10,
        points: vec![(51.0, 19.0), (51.01, 19.01)],
        source_node_ids: vec![Some(100), Some(101)],
        road_type: RoadType::Track,
        directional_access: DirectionalAccess::Both,
        name: Some("Forest Road".into()),
        ref_name: Some("FR-10".into()),
        highway: Some("track".into()),
        access: access.map(Into::into),
        vehicle: None,
        motor_vehicle: None,
        motorcar: None,
        bicycle: Some("yes".into()),
        foot: Some("yes".into()),
        oneway_foot: None,
        oneway_bicycle: None,
        motorroad: None,
        service: None,
        surface: Some("gravel".into()),
        tracktype: Some("grade2".into()),
        smoothness: Some("intermediate".into()),
        maxspeed: Some("30".into()),
        lanes: Some("1".into()),
        layer: None,
        level: None,
        destination: None,
        destination_ref: None,
        destination_street: None,
        junction_ref: None,
        is_toll: false,
        is_ferry: false,
        has_gate: false,
        is_access_restricted: access.is_some(),
        conditional_restrictions: Vec::new(),
        has_bridge: false,
        has_tunnel: false,
        has_ford: false,
        is_roundabout: false,
        is_link: false,
        is_destination_only: vehicle_access_is_destination_only(None, None, None, access),
        access_restricted: access.is_some(),
        source_lineage_ids: vec![10],
        coverage_id: None,
        area: None,
        area_highway: None,
        ..Default::default()
    }
}

#[test]
fn builds_graph_from_routable_ways() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    builder.push_way(road_way(None));
    let graph = builder.finish();
    assert_eq!(graph.node_count(), 2);
    assert_eq!(graph.edge_count(), 2);
    assert_eq!(graph.shared_edge_infos.len(), 1);
}

#[test]
fn excludes_restricted_paths_without_override() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    let mut restricted = road_way(Some("private"));
    restricted.highway = Some("footway".into());
    restricted.foot = None;
    restricted.bicycle = None;
    builder.push_way(restricted);
    let graph = builder.finish();
    assert_eq!(graph.edge_count(), 0);
}

#[test]
fn keeps_restricted_drivable_roads_for_sar_only() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    let mut restricted = road_way(Some("no"));
    restricted.foot = None;
    restricted.bicycle = None;
    builder.push_way(restricted);
    let mut one_way = road_way(None);
    one_way.osm_way_id = 11;
    one_way.points = vec![(51.0, 19.02), (51.01, 19.03)];
    one_way.source_node_ids = vec![Some(102), Some(103)];
    one_way.directional_access = DirectionalAccess::Forward;
    one_way.motor_vehicle = Some("forestry".into());
    builder.push_way(one_way);
    let (graph, report) = builder.finish_with_report();

    // The closed track: both directions, no legal access, kept for SAR.
    let closed = graph
        .graph
        .edge_weights()
        .filter(|edge| {
            graph
                .shared_edge_info(edge.shared_edge_info_id)
                .unwrap()
                .osm_way_id
                == 10
        })
        .collect::<Vec<_>>();
    assert_eq!(closed.len(), 2);
    assert!(
        closed
            .iter()
            .all(|edge| edge.is_sar_only() && edge.mobility_mode_mask == 0)
    );
    let info = graph
        .shared_edge_info(closed[0].shared_edge_info_id)
        .unwrap();
    let reason = wayfarer_package::sar::LegalReason(info.attrs.legal_reason);
    assert!(reason.vehicle_denied());
    assert_eq!(reason.tag().as_deref(), Some("access=no"));
    assert_eq!(report.sar_only_way_count, 1);
    assert_eq!(report.sar_only_edge_count, 2);
    assert_eq!(report.dropped_restricted_way_count, 0);
    assert_eq!(report.zero_access_edge_count, 0);

    // The forestry one-way keeps its walkers' edges both ways: vehicles are
    // denied, and the reverse edge runs against the one-way.
    let forestry = graph
        .graph
        .edge_weights()
        .filter(|edge| {
            graph
                .shared_edge_info(edge.shared_edge_info_id)
                .unwrap()
                .osm_way_id
                == 11
        })
        .collect::<Vec<_>>();
    assert_eq!(forestry.len(), 2);
    assert!(forestry.iter().all(|edge| !edge.is_sar_only()));
    let info = graph
        .shared_edge_info(forestry[0].shared_edge_info_id)
        .unwrap();
    let reason = wayfarer_package::sar::LegalReason(info.attrs.legal_reason);
    assert!(reason.vehicle_denied());
    assert_eq!(reason.tag().as_deref(), Some("motor_vehicle=forestry"));
    for edge in forestry {
        let bits = wayfarer_package::sar::edge_sar_bits(
            info.attrs,
            edge.traversal_direction,
            edge.mobility_mode_mask,
        );
        let against = edge.traversal_direction == TraversalDirection::Backward;
        assert!(bits & wayfarer_package::sar::ACCESS_VEHICLE_PHYSICAL != 0);
        assert_eq!(bits & wayfarer_package::sar::ACCESS_WRONG_WAY != 0, against);
    }
}

#[test]
fn sar_only_edges_stay_out_of_junctions_and_components() {
    // A public street, a closed driveway at its middle node.
    let mut builder = RoadGraphBuilder::new("pkg-1");
    let street = |id, points: Vec<(f64, f64)>, nodes: Vec<Option<i64>>| RoutableWay {
        osm_way_id: id,
        points,
        source_node_ids: nodes,
        road_type: RoadType::Residential,
        highway: Some("residential".into()),
        ..RoutableWay::default()
    };
    builder.push_way(street(
        1,
        vec![(51.0, 19.0), (51.0, 19.001)],
        vec![Some(1), Some(2)],
    ));
    builder.push_way(street(
        2,
        vec![(51.0, 19.001), (51.0, 19.002)],
        vec![Some(2), Some(3)],
    ));
    let mut driveway = street(
        3,
        vec![(51.0, 19.001), (51.001, 19.001)],
        vec![Some(2), Some(4)],
    );
    driveway.highway = Some("service".into());
    driveway.access = Some("no".into());
    driveway.access_restricted = true;
    builder.push_way(driveway);
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    let middle = graph
        .graph
        .node_weights()
        .find(|node| node.source_node_id == Some(2))
        .unwrap();
    // Two legal neighbours: a through node, as without the driveway.
    assert_eq!(middle.junction_kind, RoadJunctionKind::Through);
    let end = graph
        .graph
        .node_weights()
        .find(|node| node.source_node_id == Some(4))
        .unwrap();
    assert_eq!(end.component_id, None);
    assert!(
        graph
            .graph
            .edge_weights()
            .all(|edge| !edge.end_minor_approach)
    );
}

#[test]
fn restricted_road_with_foot_override_stays_walkable() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    let mut restricted = road_way(Some("private"));
    restricted.bicycle = None;
    restricted.foot = Some("yes".into());
    builder.push_way(restricted);
    let graph = builder.finish();

    // `access=private` with an explicit `foot=yes` is walkable in both
    // directions, and open to vehicles only as a destination.
    assert_eq!(graph.edge_count(), 2);
    assert!(graph.shared_edge_infos.at(0).attrs.is_destination_only);
    for edge in graph.graph.edge_weights() {
        assert_ne!(edge.osm_forward_mode_mask & OsmMode::FOOT, 0);
        assert_ne!(edge.osm_reverse_mode_mask & OsmMode::FOOT, 0);
        assert_ne!(edge.mobility_mode_mask & MobilityMode::ON_FOOT, 0);
    }
}

#[test]
fn private_roads_are_destination_only_for_vehicles_and_closed_to_the_rest() {
    for access in ["private", "customers"] {
        let mut builder = RoadGraphBuilder::new("pkg-1");
        let mut restricted = road_way(Some(access));
        restricted.foot = None;
        restricted.bicycle = None;
        builder.push_way(restricted);
        let (graph, report) = builder.finish_with_report();

        assert_eq!(graph.edge_count(), 2, "{access}");
        assert_eq!(report.sar_only_way_count, 0);
        let info = graph.shared_edge_infos.at(0).attrs;
        assert!(info.is_destination_only);
        let reason = wayfarer_package::sar::LegalReason(info.legal_reason);
        assert!(!reason.vehicle_denied());
        for edge in graph.graph.edge_weights() {
            assert_ne!(edge.mobility_mode_mask & MobilityMode::WHEELED, 0);
            assert_eq!(edge.mobility_mode_mask & MobilityMode::ON_FOOT, 0);
        }
    }
}

#[test]
fn a_private_motor_grant_opens_a_closed_way_as_a_destination() {
    let mask = derive_osm_mode_mask(
        Some("track"),
        Some("no"),
        None,
        Some("private"),
        None,
        None,
        None,
        None,
    );
    assert_ne!(mask & OsmMode::MOTOR_VEHICLE, 0);
    assert_eq!(mask & (OsmMode::FOOT | OsmMode::BICYCLE), 0);
    assert!(vehicle_access_is_destination_only(
        None,
        Some("private"),
        None,
        Some("no")
    ));
}

#[test]
fn the_most_specific_access_tag_decides_destination_only() {
    let only = vehicle_access_is_destination_only;
    assert!(only(None, None, None, Some("private")));
    assert!(only(None, None, None, Some("customers")));
    assert!(only(None, Some("destination"), None, Some("yes")));
    assert!(only(None, None, Some("delivery"), None));
    assert!(only(None, None, None, Some("permit")));
    assert!(!only(None, Some("yes"), None, Some("private")));
    assert!(!only(Some("permissive"), Some("private"), None, None));
    assert!(!only(None, Some("no"), None, Some("private")));
    assert!(!only(None, None, None, Some("yes")));
    assert!(!only(None, None, None, None));
}

#[test]
fn permits_destination_override_for_restricted_roads() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    builder.push_way(road_way(Some("destination")));
    let graph = builder.finish();
    assert_eq!(graph.edge_count(), 2);
}

#[test]
fn finish_with_report_counts_graph_build_diagnostics() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    builder.push_way(road_way(None));
    let mut restricted = road_way(Some("private"));
    restricted.highway = Some("footway".into());
    restricted.foot = None;
    restricted.bicycle = None;
    builder.push_way(restricted);
    let mut closed = road_way(None);
    closed.directional_access = DirectionalAccess::Closed;
    builder.push_way(closed);
    let mut short = road_way(None);
    short.points = vec![(51.0, 19.0)];
    short.source_node_ids = vec![Some(100)];
    builder.push_way(short);

    let (graph, report) = builder.finish_with_report();

    assert_eq!(graph.node_count(), 2);
    assert_eq!(graph.edge_count(), 2);
    assert_eq!(report.accepted_way_count, 1);
    assert_eq!(report.dropped_restricted_way_count, 1);
    assert_eq!(report.dropped_closed_way_count, 1);
    assert_eq!(report.dropped_short_way_count, 1);
    assert!(
        report
            .accepted_way_counts_by_road_type
            .iter()
            .any(|entry| entry.name == "track" && entry.count == 1)
    );
    assert!(
        report
            .rejected_way_counts_by_reason
            .iter()
            .any(|entry| entry.name == "closed" && entry.count == 1)
    );
    assert!(
        report
            .rejected_way_counts_by_reason
            .iter()
            .any(|entry| entry.name == "restricted" && entry.count == 1)
    );
    assert!(
        report
            .rejected_way_counts_by_reason
            .iter()
            .any(|entry| entry.name == "short" && entry.count == 1)
    );
    assert!(!report.access_mask_distribution.is_empty());
    assert_eq!(report.created_node_count, 2);
    assert_eq!(report.created_shared_segment_count, 1);
    assert_eq!(report.created_directed_edge_count, 2);
    assert_eq!(report.zero_access_edge_count, 0);
}

#[test]
fn builds_shared_edge_info_once_per_segment() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    builder.push_way(road_way(None));
    let graph = builder.finish();
    let forward = graph.graph.edge_weights().next().unwrap();
    let backward = graph.graph.edge_weights().nth(1).unwrap();
    assert_eq!(forward.shared_edge_info_id, backward.shared_edge_info_id);
    let shared = graph
        .shared_edge_info(forward.shared_edge_info_id)
        .expect("shared edge info");
    assert_eq!(shared.geometry, vec![(51.0, 19.0), (51.01, 19.01)]);
}

#[test]
fn graph_snapshot_round_trip_preserves_edges() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    builder.push_way(road_way(None));
    let graph = builder.finish();
    let dir = tempdir().unwrap();
    let path = dir.path().join("road_graph.mrg");
    persist_road_graph_file(&path, &graph).unwrap();
    let restored = load_road_graph_file(&path).unwrap();

    assert_eq!(restored.graph_id, "pkg-1");
    assert_eq!(restored.edge_count(), 2);
    assert_eq!(restored.shared_edge_infos.len(), 1);
}

#[test]
fn compressed_container_round_trip_preserves_routing_facts() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    builder.push_way(road_way(None));
    let mut graph = builder.finish();
    graph.set_route_memberships(vec![RoadRouteMembership {
        relation_id: 22,
        way_id: 10,
        route: "road".to_string(),
        network: Some("local".to_string()),
        ref_name: Some("R-22".to_string()),
        relation_name: Some("Responder Spur".to_string()),
        member_role: None,
    }]);
    graph.set_resolved_restrictions(vec![ResolvedEdgeRestriction::Simple {
        relation_id: 99,
        restriction: "no_left_turn".to_string(),
        from_edge_id: 1,
        to_edge_id: 2,
        via_node_id: 101,
        osm_mode_mask: OsmMode::MOTOR_VEHICLE,
        mobility_mode_mask: MobilityMode::WHEELED | MobilityMode::TRACKED,
    }]);

    let bytes = serialize_road_graph(&graph).unwrap();
    assert!(bytes.starts_with(ROAD_GRAPH_CONTAINER_MAGIC));
    let info = inspect_road_graph_container(&bytes).unwrap();
    assert_eq!(info.container_version, ROAD_GRAPH_CONTAINER_VERSION);
    assert_eq!(info.codec, ROAD_GRAPH_CONTAINER_CODEC);
    assert_eq!(info.graph_schema_version, ROAD_GRAPH_SNAPSHOT_VERSION);

    let restored = deserialize_road_graph(&bytes).unwrap();
    assert_eq!(restored.graph_id, graph.graph_id);
    assert_eq!(restored.edge_count(), graph.edge_count());
    assert_eq!(restored.route_memberships.len(), 1);
    assert_eq!(restored.resolved_restrictions.len(), 1);
}

#[test]
fn current_raw_snapshot_still_deserializes() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    builder.push_way(road_way(None));
    let graph = builder.finish();
    let raw = serialize_road_graph_snapshot(&graph).unwrap();
    assert!(!raw.starts_with(ROAD_GRAPH_CONTAINER_MAGIC));

    let restored = deserialize_road_graph(&raw).unwrap();
    assert_eq!(restored.graph_id, "pkg-1");
    assert_eq!(restored.edge_count(), 2);
}

#[test]
fn other_raw_snapshot_schema_version_is_rejected() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    builder.push_way(road_way(None));
    let graph = builder.finish();
    let mut raw = serialize_road_graph_snapshot(&graph).unwrap();
    raw[0..4].copy_from_slice(&6u32.to_le_bytes());

    let error = deserialize_road_graph(&raw).unwrap_err();

    assert!(error.contains("Unsupported road graph snapshot version 6"));
}

#[test]
fn other_container_schema_version_is_rejected() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    builder.push_way(road_way(None));
    let graph = builder.finish();
    let mut bytes = serialize_road_graph(&graph).unwrap();
    bytes[8..12].copy_from_slice(&6u32.to_le_bytes());

    let error = inspect_road_graph_container(&bytes).unwrap_err();

    assert!(error.contains("Unsupported road graph snapshot version 6"));
}

#[test]
fn corrupted_container_checksum_fails() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    builder.push_way(road_way(None));
    let graph = builder.finish();
    let mut bytes = serialize_road_graph(&graph).unwrap();
    bytes[28] ^= 0xff;

    let error = deserialize_road_graph(&bytes).unwrap_err();
    assert!(error.contains("checksum mismatch"));
}

#[test]
fn invalid_container_header_fails_clearly() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    builder.push_way(road_way(None));
    let graph = builder.finish();
    let mut bytes = serialize_road_graph(&graph).unwrap();

    bytes[0] = b'X';
    let error = inspect_road_graph_container(&bytes).unwrap_err();
    assert!(error.contains("magic"));

    let mut bytes = serialize_road_graph(&graph).unwrap();
    // A container of another version is refused, not misread.
    bytes[4] = 2;
    let error = inspect_road_graph_container(&bytes).unwrap_err();
    assert!(error.contains("container version"));

    let mut bytes = serialize_road_graph(&graph).unwrap();
    bytes[6] = 2;
    let error = inspect_road_graph_container(&bytes).unwrap_err();
    assert!(error.contains("codec"));
}

#[test]
fn graph_manifest_counts_quality_metrics() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    builder.push_way(road_way(None));

    let mut one_way = road_way(None);
    one_way.osm_way_id = 11;
    one_way.points = vec![(52.0, 19.0), (52.01, 19.01)];
    one_way.source_node_ids = vec![Some(200), Some(201)];
    one_way.directional_access = DirectionalAccess::Forward;
    one_way.source_lineage_ids = vec![11];
    builder.push_way(one_way);

    let mut closed_to_modes = road_way(None);
    closed_to_modes.osm_way_id = 12;
    closed_to_modes.points = vec![(53.0, 19.0), (53.01, 19.01)];
    closed_to_modes.source_node_ids = vec![Some(300), Some(301)];
    closed_to_modes.access = Some("no".into());
    closed_to_modes.source_lineage_ids = vec![12];
    builder.push_way(closed_to_modes);

    let mut graph = builder.finish();
    let first_edge = graph.graph.edge_indices().next().unwrap();
    graph
        .graph
        .edge_weight_mut(first_edge)
        .unwrap()
        .mobility_mode_mask = 0;
    graph.set_turn_restrictions(vec![RoadTurnRestriction {
        relation_id: 99,
        restriction: "no_left_turn".to_string(),
        from_way_id: 10,
        to_way_id: 11,
        via_member_type: RelationMemberRefType::Node,
        via_member_id: 101,
        via_way_ids: Vec::new(),
        modes: Vec::new(),
        except_modes: Vec::new(),
    }]);
    graph.set_resolved_restrictions(vec![ResolvedEdgeRestriction::Simple {
        relation_id: 99,
        restriction: "no_left_turn".to_string(),
        from_edge_id: 1,
        to_edge_id: 3,
        via_node_id: 101,
        osm_mode_mask: OsmMode::ALL,
        mobility_mode_mask: MobilityMode::ALL_GROUND,
    }]);

    let bytes = serialize_road_graph(&graph).unwrap();
    let info = inspect_road_graph_container(&bytes).unwrap();
    let manifest = build_road_graph_manifest(&graph, &info);

    assert_eq!(manifest.node_count, 6);
    // The one-way segment keeps a reverse edge for pedestrians, so it now
    // contributes a forward (all-mode) and reverse (foot-only) directed edge.
    assert_eq!(manifest.directed_edge_count, 6);
    assert_eq!(manifest.shared_edge_info_count, 3);
    assert_eq!(manifest.raw_turn_restriction_count, 1);
    assert_eq!(manifest.resolved_turn_restriction_count, 1);
    assert_eq!(manifest.zero_access_edge_count, 1);
    assert_eq!(manifest.one_way_edge_count, 2);
    assert_eq!(manifest.bidirectional_segment_count, 2);
    assert!((manifest.largest_weakly_connected_component_ratio - (2.0 / 6.0)).abs() < 1e-9);
}

#[test]
fn manifest_quality_defaults_when_missing_from_older_json() {
    let value = serde_json::json!({
        "graphSchemaVersion": ROAD_GRAPH_SNAPSHOT_VERSION,
        "containerVersion": ROAD_GRAPH_CONTAINER_VERSION,
        "codec": ROAD_GRAPH_CONTAINER_CODEC,
        "uncompressedBytes": 10,
        "compressedBytes": 5,
        "compressionRatio": 0.5,
        "blake3HashHex": "hash",
        "nodeCount": 1,
        "directedEdgeCount": 1,
        "sharedEdgeInfoCount": 1,
        "rawTurnRestrictionCount": 0,
        "resolvedTurnRestrictionCount": 0,
        "routeMembershipCount": 0,
        "zeroAccessEdgeCount": 0,
        "oneWayEdgeCount": 0,
        "bidirectionalSegmentCount": 1,
        "largestWeaklyConnectedComponentRatio": 1.0
    });

    let manifest: RoadGraphPackageManifest = serde_json::from_value(value).unwrap();

    assert_eq!(manifest.quality, RoadGraphQualityManifest::default());
}

#[test]
fn manifest_quality_enhancement_defaults_when_missing_from_older_json() {
    let value = serde_json::json!({
        "builderReport": {
            "acceptedWayCount": 1
        },
        "topologyRepairReport": {},
        "restrictionReport": {},
        "status": "ready",
        "reasons": []
    });

    let quality: RoadGraphQualityManifest = serde_json::from_value(value).unwrap();

    assert_eq!(quality.builder_report.accepted_way_count, 1);
    assert_eq!(
        quality.enhancement_report,
        RoadGraphEnhancementReport::default()
    );
}

#[test]
fn enhancement_assigns_components_degrees_headings_speeds_and_warnings() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    let mut west = road_way(None);
    west.osm_way_id = 20;
    west.points = vec![(51.0, 18.99), (51.0, 19.0)];
    west.source_node_ids = vec![Some(200), Some(100)];
    west.source_lineage_ids = vec![20];
    builder.push_way(west);

    let mut east = road_way(None);
    east.osm_way_id = 21;
    east.points = vec![(51.0, 19.0), (51.0, 19.01)];
    east.source_node_ids = vec![Some(100), Some(201)];
    east.source_lineage_ids = vec![21];
    builder.push_way(east);

    let mut north = road_way(None);
    north.osm_way_id = 22;
    north.points = vec![(51.0, 19.0), (51.01, 19.0)];
    north.source_node_ids = vec![Some(100), Some(202)];
    north.source_lineage_ids = vec![22];
    builder.push_way(north);

    let mut south = road_way(None);
    south.osm_way_id = 23;
    south.points = vec![(51.0, 19.0), (50.99, 19.0)];
    south.source_node_ids = vec![Some(100), Some(203)];
    south.source_lineage_ids = vec![23];
    builder.push_way(south);

    let mut tiny = road_way(None);
    tiny.osm_way_id = 24;
    tiny.points = vec![(52.0, 20.0), (52.0, 20.000001)];
    tiny.source_node_ids = vec![None, None];
    tiny.directional_access = DirectionalAccess::Forward;
    tiny.source_lineage_ids = vec![24];
    builder.push_way(tiny);

    let mut graph = builder.finish();
    let report = enhance_road_graph(&mut graph);

    assert_eq!(report.component_count, 2);
    assert_eq!(report.largest_component_node_count, 5);
    assert_eq!(report.small_component_count, 2);
    assert!(
        report
            .component_size_histogram
            .iter()
            .any(|bucket| bucket.label == "5-9" && bucket.component_count == 1)
    );
    assert!(report.dead_end_node_count >= 6);
    assert_eq!(report.intersection_node_count, 1);
    assert_eq!(report.edges_with_heading_count, graph.edge_count());
    assert_eq!(report.edges_with_inferred_speed_count, graph.edge_count());
    assert_eq!(report.missing_heading_edge_count, 0);
    assert_eq!(report.missing_inferred_speed_edge_count, 0);
    assert_eq!(report.hierarchy_prepared_edge_count, graph.edge_count());
    assert_eq!(report.missing_hierarchy_tier_edge_count, 0);
    assert!(
        report
            .hierarchy_tier_distribution
            .iter()
            .any(|entry| entry.name == "local" && entry.count > 0)
    );
    assert_eq!(report.one_way_asymmetry_count, 1);
    assert!(report.unsnapped_endpoint_cluster_count >= 1);
    assert!(
        report
            .warning_examples
            .iter()
            .any(|example| example.kind == RoadGraphQualityExampleKind::SmallComponent)
    );

    let center = graph
        .graph
        .node_weights()
        .find(|node| node.source_node_id == Some(100))
        .unwrap();
    assert_eq!(center.junction_kind, RoadJunctionKind::Intersection);
    assert_eq!(center.highest_incident_road_type, Some(RoadType::Track));
    assert_eq!(center.component_id, Some(0));
    assert!(!center.incoming_road_type_counts.is_empty());
    assert!(!center.outgoing_road_type_counts.is_empty());
    assert!(!center.is_complex_junction);

    let first_edge = graph.graph.edge_weights().next().unwrap();
    assert!(first_edge.start_heading_deg.is_some());
    assert!(first_edge.end_heading_deg.is_some());
    assert_eq!(first_edge.inferred_speed_kph, Some(22));
    assert!(first_edge.estimated_travel_time_s.unwrap() > 0.0);
    assert_eq!(first_edge.hierarchy_tier, Some(RoadHierarchyTier::Local));
}

#[test]
fn enhancement_reports_valhalla_style_topology_quality_warnings() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    let mut west = road_way(None);
    west.osm_way_id = 100;
    west.points = vec![(51.0, 19.0), (51.0, 19.01)];
    west.source_node_ids = vec![Some(100), Some(101)];
    west.source_lineage_ids = vec![100];
    builder.push_way(west.clone());

    let mut east = road_way(None);
    east.osm_way_id = 101;
    east.points = vec![(51.0, 19.01), (51.0, 19.02)];
    east.source_node_ids = vec![Some(101), Some(102)];
    east.source_lineage_ids = vec![101];
    builder.push_way(east);

    builder.push_way(west);

    let mut isolated_link = road_way(None);
    isolated_link.osm_way_id = 200;
    isolated_link.points = vec![(52.0, 20.0), (52.0, 20.01)];
    isolated_link.source_node_ids = vec![Some(200), Some(201)];
    isolated_link.road_type = RoadType::Primary;
    isolated_link.highway = Some("primary_link".into());
    isolated_link.is_link = true;
    isolated_link.source_lineage_ids = vec![200];
    builder.push_way(isolated_link);

    let mut graph = builder.finish();
    let missing_opposing_shared_id = graph
        .shared_edge_infos
        .iter()
        .find(|info| info.osm_way_id == 100)
        .unwrap()
        .shared_edge_info_id;
    let backward_edge = graph
        .graph
        .edge_references()
        .find(|edge_ref| {
            edge_ref.weight().shared_edge_info_id == missing_opposing_shared_id
                && edge_ref.weight().traversal_direction == TraversalDirection::Backward
        })
        .map(|edge_ref| edge_ref.id())
        .unwrap();
    graph.graph.remove_edge(backward_edge);

    let mismatch_edge_id = graph
        .graph
        .edge_references()
        .find(|edge_ref| edge_ref.weight().shared_edge_info_id == missing_opposing_shared_id)
        .map(|edge_ref| edge_ref.id())
        .unwrap();
    let mismatch_edge = graph.graph.edge_weight_mut(mismatch_edge_id).unwrap();
    mismatch_edge.mobility_mode_mask = 0;
    mismatch_edge.distance_m = 1.0;

    let missing_geometry_shared_id = graph
        .shared_edge_infos
        .iter()
        .find(|info| info.osm_way_id == 101)
        .unwrap()
        .shared_edge_info_id;
    let missing_geometry_index = graph
        .shared_edge_infos
        .iter()
        .position(|info| info.shared_edge_info_id == missing_geometry_shared_id)
        .unwrap();
    *graph.shared_edge_infos.geometry_mut(missing_geometry_index) = vec![(51.0, 19.01)];

    let report = enhance_road_graph(&mut graph);

    assert!(report.missing_opposing_edge_count >= 1);
    assert!(report.directional_access_mismatch_count >= 1);
    assert!(report.duplicate_parallel_edge_count >= 1);
    assert!(report.missing_geometry_count >= 1);
    assert!(report.distance_geometry_mismatch_count >= 1);
    assert!(report.unreachable_major_edge_count >= 1);
    assert!(report.unconnected_link_edge_count >= 1);
    for kind in [
        RoadGraphQualityExampleKind::MissingOpposingEdge,
        RoadGraphQualityExampleKind::DirectionalAccessMismatch,
        RoadGraphQualityExampleKind::DuplicateParallelEdge,
        RoadGraphQualityExampleKind::MissingGeometry,
        RoadGraphQualityExampleKind::DistanceGeometryMismatch,
        RoadGraphQualityExampleKind::UnreachableMajorRoad,
        RoadGraphQualityExampleKind::UnconnectedLink,
    ] {
        assert!(
            report
                .warning_examples
                .iter()
                .any(|example| example.kind == kind),
            "missing quality example for {:?}",
            kind
        );
    }

    let bytes = serialize_road_graph(&graph).unwrap();
    let info = inspect_road_graph_container(&bytes).unwrap();
    let quality = RoadGraphQualityManifest {
        enhancement_report: report,
        ..Default::default()
    };
    let manifest = build_road_graph_manifest_with_quality(&graph, &info, quality);
    let evaluated = evaluate_road_graph_quality(&manifest);

    assert_eq!(evaluated.status, RoadGraphQualityStatus::Warning);
    assert!(
        evaluated
            .reasons
            .iter()
            .any(|reason| reason.contains("missing opposing edges"))
    );
    assert!(
        evaluated
            .reasons
            .iter()
            .any(|reason| reason.contains("duplicate parallel"))
    );
    let duplicate_finding = evaluated
        .findings
        .iter()
        .find(|finding| finding.kind == RoadGraphQualityFindingKind::DuplicateParallelEdge)
        .expect("duplicate parallel finding");
    assert_eq!(
        duplicate_finding.severity,
        RoadGraphQualityFindingSeverity::Warning
    );
    assert!(
        duplicate_finding
            .recommended_action
            .contains("duplicate OSM way lineage")
    );
    assert!(evaluated.findings.iter().any(|finding| {
        finding.kind == RoadGraphQualityFindingKind::MissingOpposingEdge
            && finding.count == Some(evaluated.enhancement_report.missing_opposing_edge_count)
    }));
    let policy_evaluation = evaluate_road_graph_quality_policy(
        &evaluated,
        &RoadGraphQualityPolicy::production_release(),
    );
    assert_eq!(policy_evaluation.status, RoadGraphQualityPolicyStatus::Fail);
    assert!(policy_evaluation.fatal_findings.iter().any(|finding| {
        finding.kind == RoadGraphQualityFindingKind::MissingGeometry
            || finding.kind == RoadGraphQualityFindingKind::UnreachableMajorRoad
    }));
    assert!(
        policy_evaluation
            .advisory_findings
            .iter()
            .any(|finding| { finding.kind == RoadGraphQualityFindingKind::DuplicateParallelEdge })
    );
}

#[test]
fn enhancement_reports_shortcut_candidates_for_linear_chains() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    for (index, (from_lng, to_lng)) in [(19.0, 19.01), (19.01, 19.02), (19.02, 19.03)]
        .into_iter()
        .enumerate()
    {
        let mut way = road_way(None);
        way.osm_way_id = 100 + index as i64;
        way.name = Some("Responder Road".into());
        way.points = vec![(51.0, from_lng), (51.0, to_lng)];
        way.source_node_ids = vec![Some(index as i64), Some(index as i64 + 1)];
        way.source_lineage_ids = vec![way.osm_way_id];
        builder.push_way(way);
    }

    let mut graph = builder.finish();
    let report = enhance_road_graph(&mut graph);

    assert!(report.shortcut_candidate_report.candidate_count >= 1);
    let example = report.shortcut_candidate_report.examples.first().unwrap();
    assert!(example.edge_count >= 2);
    assert_eq!(example.name.as_deref(), Some("Responder Road"));
    assert_eq!(example.road_type, RoadType::Track);
}

#[test]
fn enhancement_classifies_roundabout_and_complex_junctions() {
    let mut graph = RoadGraph::new("junctions");
    let hub = graph.add_node(RoadNode {
        id: 0,
        lat: 51.0,
        lng: 19.0,
        source_node_id: Some(1),
        ..Default::default()
    });
    let mut spoke_nodes = Vec::new();
    for id in 1..=5 {
        spoke_nodes.push(graph.add_node(RoadNode {
            id,
            lat: 51.0 + id as f64 * 0.001,
            lng: 19.0,
            source_node_id: Some(id + 1),
            ..Default::default()
        }));
    }
    let roundabout_a = graph.add_node(RoadNode {
        id: 10,
        lat: 52.0,
        lng: 20.0,
        source_node_id: Some(20),
        ..Default::default()
    });
    let roundabout_b = graph.add_node(RoadNode {
        id: 11,
        lat: 52.0,
        lng: 20.001,
        source_node_id: Some(21),
        ..Default::default()
    });
    graph.shared_edge_infos.push(SharedEdgeInfo {
        shared_edge_info_id: 1,
        osm_way_id: 1,
        road_type: RoadType::Primary,
        geometry: vec![(51.0, 19.0), (51.001, 19.0)],
        directional_access: DirectionalAccess::Forward,
        highway: Some("primary".into()),
        name: None,
        ref_name: None,
        route_refs: Vec::new(),
        access: None,
        service: None,
        surface: None,
        tracktype: None,
        smoothness: None,
        maxspeed_kph: None,
        lane_count: None,
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
        is_link: false,
        is_roundabout: false,
        is_bridge: false,
        is_tunnel: false,
        has_ford: false,
        is_service_like: false,
        is_destination_only: false,
        osm_forward_mode_mask: OsmMode::ALL,
        osm_reverse_mode_mask: 0,
        hierarchy_tier: Some(RoadHierarchyTier::Arterial),
        ..Default::default()
    });
    graph.shared_edge_infos.push(SharedEdgeInfo {
        shared_edge_info_id: 2,
        osm_way_id: 2,
        road_type: RoadType::Residential,
        geometry: vec![(52.0, 20.0), (52.0, 20.001)],
        directional_access: DirectionalAccess::Forward,
        highway: Some("residential".into()),
        name: None,
        ref_name: None,
        route_refs: Vec::new(),
        access: None,
        service: None,
        surface: None,
        tracktype: None,
        smoothness: None,
        maxspeed_kph: None,
        lane_count: None,
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
        is_link: false,
        is_roundabout: true,
        is_bridge: false,
        is_tunnel: false,
        has_ford: false,
        is_service_like: false,
        is_destination_only: false,
        osm_forward_mode_mask: OsmMode::ALL,
        osm_reverse_mode_mask: 0,
        hierarchy_tier: Some(RoadHierarchyTier::Collector),
        ..Default::default()
    });
    for (edge_id, spoke) in spoke_nodes.into_iter().enumerate() {
        graph.add_edge(
            hub,
            spoke,
            RoadEdge {
                edge_id: edge_id as u64 + 1,
                shared_edge_info_id: 1,
                distance_m: 10.0,
                base_cost: 10.0,
                mobility_mode_mask: MobilityMode::ALL_GROUND,
                osm_forward_mode_mask: OsmMode::ALL,
                osm_reverse_mode_mask: 0,
                traversal_direction: TraversalDirection::Forward,
                segment_index: 0,
                source_lineage_ids: vec![1],
                coverage_id: None,
                ..Default::default()
            },
        );
    }
    graph.add_edge(
        roundabout_a,
        roundabout_b,
        RoadEdge {
            edge_id: 20,
            shared_edge_info_id: 2,
            distance_m: 10.0,
            base_cost: 10.0,
            mobility_mode_mask: MobilityMode::ALL_GROUND,
            osm_forward_mode_mask: OsmMode::ALL,
            osm_reverse_mode_mask: 0,
            traversal_direction: TraversalDirection::Forward,
            segment_index: 0,
            source_lineage_ids: vec![2],
            coverage_id: None,
            ..Default::default()
        },
    );

    let report = enhance_road_graph(&mut graph);

    assert_eq!(graph.graph[hub].junction_kind, RoadJunctionKind::Complex);
    assert!(graph.graph[hub].is_complex_junction);
    assert_eq!(
        graph.graph[roundabout_a].junction_kind,
        RoadJunctionKind::Roundabout
    );
    assert_eq!(graph.graph[roundabout_a].roundabout_exit_count, 1);
    assert_eq!(report.complex_junction_node_count, 1);
    assert_eq!(report.roundabout_node_count, 2);
}

#[test]
fn quality_reports_major_road_rich_fact_coverage_gaps() {
    let mut builder = RoadGraphBuilder::new("rich-fact-coverage");
    let mut primary = road_way(None);
    primary.road_type = RoadType::Primary;
    primary.highway = Some("primary".into());
    primary.name = None;
    primary.ref_name = None;
    primary.lanes = None;
    primary.destination = None;
    primary.destination_ref = None;
    primary.destination_street = None;
    primary.junction_ref = None;
    builder.push_way(primary);
    let mut graph = builder.finish();

    let report = enhance_road_graph(&mut graph);
    let directed_edge_count = graph.edge_count();

    assert_eq!(
        report.major_edge_missing_lane_fact_count,
        directed_edge_count
    );
    assert_eq!(
        report.major_edge_missing_sign_or_destination_fact_count,
        directed_edge_count
    );
    assert_eq!(
        report.major_edge_missing_admin_fact_count,
        directed_edge_count
    );
    assert_eq!(
        report.major_edge_missing_timezone_fact_count,
        directed_edge_count
    );
    assert!(
        report
            .warning_examples
            .iter()
            .any(|example| example.kind == RoadGraphQualityExampleKind::MissingRichRoadFacts)
    );

    let bytes = serialize_road_graph(&graph).unwrap();
    let info = inspect_road_graph_container(&bytes).unwrap();
    let quality = RoadGraphQualityManifest {
        enhancement_report: report,
        ..Default::default()
    };
    let manifest = build_road_graph_manifest_with_quality(&graph, &info, quality);
    let evaluated = evaluate_road_graph_quality(&manifest);
    let finding = evaluated
        .findings
        .iter()
        .find(|finding| finding.kind == RoadGraphQualityFindingKind::MissingRichRoadFacts)
        .expect("missing rich road facts finding");

    assert_eq!(finding.severity, RoadGraphQualityFindingSeverity::Info);
    assert_eq!(finding.count, Some(directed_edge_count * 4));
    assert!(
        finding
            .message
            .contains("major-road rich fact coverage is incomplete")
    );
}

#[test]
fn manifest_quality_attachment_preserves_core_fields() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    builder.push_way(road_way(None));
    let (graph, build_report) = builder.finish_with_report();
    let bytes = serialize_road_graph(&graph).unwrap();
    let info = inspect_road_graph_container(&bytes).unwrap();
    let base = build_road_graph_manifest(&graph, &info);
    let quality = RoadGraphQualityManifest {
        builder_report: build_report,
        reasons: vec!["diagnostic warning".to_string()],
        ..Default::default()
    };

    let manifest = build_road_graph_manifest_with_quality(&graph, &info, quality.clone());

    assert_eq!(manifest.graph_schema_version, base.graph_schema_version);
    assert_eq!(manifest.uncompressed_bytes, base.uncompressed_bytes);
    assert_eq!(manifest.compressed_bytes, base.compressed_bytes);
    assert_eq!(manifest.node_count, base.node_count);
    assert_eq!(manifest.quality, quality);
}

#[test]
fn derives_expected_osm_and_mobility_masks() {
    let motor_private = derive_osm_mode_mask(
        None,
        Some("destination"),
        None,
        Some("private"),
        None,
        Some("yes"),
        Some("yes"),
        None,
    );
    // Destination-only for vehicles (see `vehicle_access_is_destination_only`).
    assert_ne!(motor_private & OsmMode::MOTOR_VEHICLE, 0);
    assert_ne!(motor_private & OsmMode::BICYCLE, 0);
    assert_ne!(motor_private & OsmMode::FOOT, 0);

    let mobility = mobility_mode_mask_from_osm(motor_private);
    assert_ne!(mobility & MobilityMode::WHEELED, 0);
    assert_ne!(mobility & MobilityMode::TRACKED, 0);
    assert_ne!(mobility & MobilityMode::ON_FOOT, 0);
    assert_ne!(mobility & MobilityMode::LEGGED, 0);
    assert_eq!(mobility & MobilityMode::AERIAL, 0);
}

#[test]
fn motorways_exclude_foot_and_bicycle_by_default() {
    let motorway = derive_osm_mode_mask(Some("motorway"), None, None, None, None, None, None, None);
    assert_ne!(motorway & OsmMode::MOTOR_VEHICLE, 0);
    assert_eq!(motorway & OsmMode::FOOT, 0);
    assert_eq!(motorway & OsmMode::BICYCLE, 0);

    let link = derive_osm_mode_mask(
        Some("motorway_link"),
        None,
        None,
        None,
        None,
        None,
        None,
        None,
    );
    assert_eq!(link & OsmMode::FOOT, 0);
}

#[test]
fn explicit_foot_grant_reenables_motorway_foot() {
    let motorway = derive_osm_mode_mask(
        Some("motorway"),
        None,
        None,
        None,
        None,
        None,
        Some("yes"),
        None,
    );
    assert_ne!(motorway & OsmMode::FOOT, 0);
    // A generic `access=yes` must not silently re-open a motorway to pedestrians.
    let generic = derive_osm_mode_mask(
        Some("motorway"),
        Some("yes"),
        None,
        None,
        None,
        None,
        None,
        None,
    );
    assert_eq!(generic & OsmMode::FOOT, 0);
}

#[test]
fn foot_ignores_vehicular_oneway_but_vehicles_and_bicycles_obey_it() {
    let base = derive_osm_mode_mask(
        Some("residential"),
        None,
        None,
        None,
        None,
        None,
        None,
        None,
    );
    let (forward, reverse) = split_directional_masks(base, DirectionalAccess::Forward, None, None);

    // Forward carries every mode; reverse keeps only pedestrians walking against
    // the one-way, while vehicles and bicycles remain forward-only.
    assert_eq!(forward, base);
    assert_ne!(reverse & OsmMode::FOOT, 0);
    assert_eq!(reverse & OsmMode::MOTOR_VEHICLE, 0);
    assert_eq!(reverse & OsmMode::BICYCLE, 0);
}

#[test]
fn oneway_foot_restricts_pedestrian_contraflow() {
    let base = derive_osm_mode_mask(
        Some("residential"),
        None,
        None,
        None,
        None,
        None,
        None,
        None,
    );

    // `oneway:foot=yes` pins pedestrians to the way's digitised direction even
    // though the street is two-way for foot by default.
    let (forward, reverse) =
        split_directional_masks(base, DirectionalAccess::Both, Some("yes"), None);
    assert_ne!(forward & OsmMode::FOOT, 0);
    assert_eq!(reverse & OsmMode::FOOT, 0);

    // `oneway:foot=-1` restricts pedestrians to the reverse direction only.
    let (forward, reverse) =
        split_directional_masks(base, DirectionalAccess::Both, Some("-1"), None);
    assert_eq!(forward & OsmMode::FOOT, 0);
    assert_ne!(reverse & OsmMode::FOOT, 0);

    // `oneway:foot=no` is the explicit form of the two-way default.
    let (forward, reverse) =
        split_directional_masks(base, DirectionalAccess::Both, Some("no"), None);
    assert_ne!(forward & OsmMode::FOOT, 0);
    assert_ne!(reverse & OsmMode::FOOT, 0);
}

#[test]
fn oneway_bicycle_no_grants_contraflow_while_motor_stays_one_way() {
    let base = derive_osm_mode_mask(
        Some("residential"),
        None,
        None,
        None,
        None,
        None,
        None,
        None,
    );

    // `oneway:bicycle=no` on a vehicular one-way lets cyclists ride against the
    // flow, while motor vehicles remain forward-only.
    let (forward, reverse) =
        split_directional_masks(base, DirectionalAccess::Forward, None, Some("no"));
    assert_ne!(forward & OsmMode::BICYCLE, 0);
    assert_ne!(reverse & OsmMode::BICYCLE, 0);
    assert_ne!(forward & OsmMode::MOTOR_VEHICLE, 0);
    assert_eq!(reverse & OsmMode::MOTOR_VEHICLE, 0);
}

#[test]
fn motorroad_excludes_foot_and_bicycle_like_motorway() {
    let motorroad = derive_osm_mode_mask(
        Some("primary"),
        None,
        None,
        None,
        None,
        None,
        None,
        Some("yes"),
    );
    assert_ne!(motorroad & OsmMode::MOTOR_VEHICLE, 0);
    assert_eq!(motorroad & OsmMode::FOOT, 0);
    assert_eq!(motorroad & OsmMode::BICYCLE, 0);

    // A generic `access=yes` must not silently re-open a motorroad to pedestrians.
    let generic = derive_osm_mode_mask(
        Some("primary"),
        Some("yes"),
        None,
        None,
        None,
        None,
        None,
        Some("yes"),
    );
    assert_eq!(generic & OsmMode::FOOT, 0);

    // An explicit `foot=yes` re-enables pedestrians on the motorroad.
    let with_foot = derive_osm_mode_mask(
        Some("primary"),
        None,
        None,
        None,
        None,
        None,
        Some("yes"),
        Some("yes"),
    );
    assert_ne!(with_foot & OsmMode::FOOT, 0);
    assert_eq!(with_foot & OsmMode::BICYCLE, 0);
}

#[test]
fn plain_dual_use_road_is_walkable_in_both_directions() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    let mut residential = road_way(None);
    residential.road_type = RoadType::Residential;
    residential.highway = Some("residential".into());
    residential.foot = None;
    residential.bicycle = None;
    builder.push_way(residential);
    let graph = builder.finish();

    assert_eq!(graph.edge_count(), 2);
    for edge in graph.graph.edge_weights() {
        assert_ne!(edge.mobility_mode_mask & MobilityMode::ON_FOOT, 0);
        assert_ne!(edge.mobility_mode_mask & MobilityMode::WHEELED, 0);
    }
}

#[test]
fn one_way_dual_use_road_stays_walkable_both_ways() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    let mut one_way = road_way(None);
    one_way.road_type = RoadType::Residential;
    one_way.highway = Some("residential".into());
    one_way.foot = None;
    one_way.bicycle = None;
    one_way.directional_access = DirectionalAccess::Forward;
    builder.push_way(one_way);
    let graph = builder.finish();

    let mut forward_walkable = false;
    let mut reverse_walkable = false;
    for edge in graph.graph.edge_weights() {
        assert_ne!(edge.mobility_mode_mask & MobilityMode::ON_FOOT, 0);
        match edge.traversal_direction {
            TraversalDirection::Forward => {
                forward_walkable = true;
                assert_ne!(edge.mobility_mode_mask & MobilityMode::WHEELED, 0);
            }
            TraversalDirection::Backward => {
                reverse_walkable = true;
                // Vehicles may not travel the wrong way down the one-way.
                assert_eq!(edge.mobility_mode_mask & MobilityMode::WHEELED, 0);
            }
        }
    }
    assert!(forward_walkable && reverse_walkable);
}

#[test]
fn graph_snapshot_round_trip_preserves_routing_facts() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    builder.push_way(road_way(None));
    let mut graph = builder.finish();
    graph.set_way_facts(vec![RoadWayFacts {
        osm_way_id: 10,
        road_type: RoadType::Track,
        directional_access: DirectionalAccess::Both,
        name: Some("Forest Road".to_string()),
        ref_name: Some("FR-10".to_string()),
        highway: Some("track".to_string()),
        access: Some("destination".to_string()),
        vehicle: None,
        motor_vehicle: Some("destination".to_string()),
        motorcar: None,
        bicycle: Some("yes".to_string()),
        foot: Some("yes".to_string()),
        oneway_foot: None,
        oneway_bicycle: None,
        motorroad: None,
        service: None,
        surface: Some("gravel".to_string()),
        tracktype: Some("grade2".to_string()),
        smoothness: Some("intermediate".to_string()),
        maxspeed: Some("30".to_string()),
        lanes: Some("1".to_string()),
        turn_lanes: Some("left|through".to_string()),
        turn_lanes_forward: Some("through".to_string()),
        turn_lanes_backward: Some("left".to_string()),
        lane_connectivity_available: true,
        admin_country_id: Some("PL".to_string()),
        admin_region_id: Some("malopolskie".to_string()),
        timezone_id: Some("Europe/Warsaw".to_string()),
        layer: Some("1".to_string()),
        level: Some("L1".to_string()),
        destination: Some("Forward Base".to_string()),
        destination_ref: Some("MSR-7".to_string()),
        destination_street: Some("Supply Road".to_string()),
        junction_ref: Some("7A".to_string()),
        is_toll: true,
        is_ferry: false,
        has_gate: true,
        is_access_restricted: true,
        has_bridge: false,
        has_tunnel: false,
        has_ford: false,
        is_roundabout: false,
        is_link: false,
        is_destination_only: true,
        access_restricted: false,
        ..Default::default()
    }]);
    graph.set_route_memberships(vec![RoadRouteMembership {
        relation_id: 22,
        way_id: 10,
        route: "road".to_string(),
        network: Some("local".to_string()),
        ref_name: Some("R-22".to_string()),
        relation_name: Some("Responder Spur".to_string()),
        member_role: None,
    }]);
    graph.set_turn_restrictions(vec![RoadTurnRestriction {
        relation_id: 99,
        restriction: "no_left_turn".to_string(),
        from_way_id: 10,
        to_way_id: 11,
        via_member_type: RelationMemberRefType::Node,
        via_member_id: 101,
        via_way_ids: Vec::new(),
        modes: Vec::new(),
        except_modes: vec!["bicycle".to_string()],
    }]);
    graph.set_resolved_restrictions(vec![ResolvedEdgeRestriction::Simple {
        relation_id: 99,
        restriction: "no_left_turn".to_string(),
        from_edge_id: 1,
        to_edge_id: 3,
        via_node_id: 101,
        osm_mode_mask: OsmMode::MOTOR_VEHICLE,
        mobility_mode_mask: MobilityMode::WHEELED | MobilityMode::TRACKED,
    }]);
    graph.refresh_shared_edge_metadata();

    let bytes = serialize_road_graph(&graph).unwrap();
    let restored = deserialize_road_graph(&bytes).unwrap();

    assert_eq!(restored.way_facts.len(), 1);
    assert_eq!(restored.route_memberships.len(), 1);
    assert_eq!(restored.turn_restrictions.len(), 1);
    assert_eq!(restored.resolved_restrictions.len(), 1);
    let edge = restored.graph.edge_weights().next().unwrap();
    assert_eq!(edge.edge_id, 1);
    assert_eq!(edge.traversal_direction, TraversalDirection::Forward);
    assert_eq!(edge.segment_index, 0);
    assert_eq!(edge.source_lineage_ids, vec![10]);
    let shared = restored
        .shared_edge_info(edge.shared_edge_info_id)
        .expect("shared edge")
        .to_info();
    assert_eq!(shared.route_refs, vec!["R-22".to_string()]);
    assert!(shared.is_destination_only);
    assert_eq!(shared.lanes.as_deref(), Some("1"));
    assert_eq!(shared.turn_lanes.as_deref(), Some("left|through"));
    assert_eq!(shared.turn_lanes_forward.as_deref(), Some("through"));
    assert_eq!(shared.turn_lanes_backward.as_deref(), Some("left"));
    assert!(shared.lane_connectivity_available);
    assert_eq!(shared.speed_source.as_deref(), Some("osm:maxspeed"));
    assert_eq!(shared.free_flow_speed_kph, Some(22.0));
    assert!(
        shared
            .constrained_speed_kph
            .is_some_and(|speed| (speed - 18.7).abs() < 0.001)
    );
    assert_eq!(shared.admin_country_id.as_deref(), Some("PL"));
    assert_eq!(shared.admin_region_id.as_deref(), Some("malopolskie"));
    assert_eq!(shared.timezone_id.as_deref(), Some("Europe/Warsaw"));
    assert_eq!(shared.layer, Some(1));
    assert_eq!(shared.level.as_deref(), Some("L1"));
    assert_eq!(shared.destination.as_deref(), Some("Forward Base"));
    assert_eq!(shared.destination_ref.as_deref(), Some("MSR-7"));
    assert_eq!(shared.destination_street.as_deref(), Some("Supply Road"));
    assert_eq!(shared.junction_ref.as_deref(), Some("7A"));
    assert!(shared.is_toll);
    assert!(!shared.is_ferry);
    assert!(shared.has_gate);
    assert!(shared.is_access_restricted);
}

#[test]
fn discard_source_enrichment_inputs_keeps_applied_shared_metadata() {
    let mut builder = RoadGraphBuilder::new("pkg-1");
    builder.push_way(road_way(None));
    let mut graph = builder.finish();
    graph.set_way_facts(vec![RoadWayFacts {
        osm_way_id: 10,
        road_type: RoadType::Track,
        directional_access: DirectionalAccess::Both,
        name: Some("Forest Road".to_string()),
        ref_name: Some("FR-10".to_string()),
        highway: Some("track".to_string()),
        access: Some("destination".to_string()),
        vehicle: None,
        motor_vehicle: Some("destination".to_string()),
        motorcar: None,
        bicycle: Some("yes".to_string()),
        foot: Some("yes".to_string()),
        oneway_foot: None,
        oneway_bicycle: None,
        motorroad: None,
        service: None,
        surface: Some("gravel".to_string()),
        tracktype: Some("grade2".to_string()),
        smoothness: Some("intermediate".to_string()),
        maxspeed: Some("30".to_string()),
        lanes: Some("1".to_string()),
        turn_lanes: Some("left|through".to_string()),
        turn_lanes_forward: Some("through".to_string()),
        turn_lanes_backward: Some("left".to_string()),
        lane_connectivity_available: true,
        admin_country_id: Some("PL".to_string()),
        admin_region_id: Some("malopolskie".to_string()),
        timezone_id: Some("Europe/Warsaw".to_string()),
        layer: Some("1".to_string()),
        level: Some("L1".to_string()),
        destination: Some("Forward Base".to_string()),
        destination_ref: Some("MSR-7".to_string()),
        destination_street: Some("Supply Road".to_string()),
        junction_ref: Some("7A".to_string()),
        is_toll: true,
        is_ferry: false,
        has_gate: true,
        is_access_restricted: true,
        has_bridge: false,
        has_tunnel: false,
        has_ford: false,
        is_roundabout: false,
        is_link: false,
        is_destination_only: true,
        access_restricted: false,
        ..Default::default()
    }]);
    graph.set_route_memberships(vec![RoadRouteMembership {
        relation_id: 22,
        way_id: 10,
        route: "road".to_string(),
        network: Some("local".to_string()),
        ref_name: Some("R-22".to_string()),
        relation_name: None,
        member_role: None,
    }]);
    graph.set_turn_restrictions(vec![RoadTurnRestriction {
        relation_id: 99,
        restriction: "no_left_turn".to_string(),
        from_way_id: 10,
        to_way_id: 11,
        via_member_type: RelationMemberRefType::Node,
        via_member_id: 101,
        via_way_ids: Vec::new(),
        modes: Vec::new(),
        except_modes: Vec::new(),
    }]);
    graph.set_resolved_restrictions(vec![ResolvedEdgeRestriction::Simple {
        relation_id: 99,
        restriction: "no_left_turn".to_string(),
        from_edge_id: 1,
        to_edge_id: 3,
        via_node_id: 101,
        osm_mode_mask: OsmMode::MOTOR_VEHICLE,
        mobility_mode_mask: MobilityMode::WHEELED,
    }]);
    graph.refresh_shared_edge_metadata();
    graph.discard_source_enrichment_inputs();

    assert!(graph.way_facts.is_empty());
    assert!(graph.route_memberships.is_empty());
    assert!(graph.turn_restrictions.is_empty());
    assert_eq!(graph.resolved_restrictions.len(), 1);

    let bytes = serialize_road_graph(&graph).unwrap();
    let restored = deserialize_road_graph(&bytes).unwrap();
    assert!(restored.way_facts.is_empty());
    assert!(restored.route_memberships.is_empty());
    assert!(restored.turn_restrictions.is_empty());
    assert_eq!(restored.resolved_restrictions.len(), 1);
    let shared = restored
        .shared_edge_infos
        .get(0)
        .expect("shared info")
        .to_info();
    assert_eq!(shared.route_refs, vec!["R-22".to_string()]);
    assert_eq!(shared.name.as_deref(), Some("Forest Road"));
    assert_eq!(shared.lanes.as_deref(), Some("1"));
    assert_eq!(shared.destination_ref.as_deref(), Some("MSR-7"));
    assert!(shared.has_gate);
    assert!(shared.is_destination_only);
}

#[test]
fn parse_maxspeed_numeric_and_mph_unchanged() {
    assert_eq!(parse_maxspeed_kph(Some("50")), Some(50.0));
    assert_eq!(parse_maxspeed_kph(Some("50 mph")), Some(50.0 * 1.609_34));
    assert_eq!(parse_maxspeed_kph(Some("30mph")), Some(30.0 * 1.609_34));
    assert_eq!(parse_maxspeed_kph(Some("  90 ")), Some(90.0));
    assert_eq!(parse_maxspeed_kph(None), None);
    assert_eq!(parse_maxspeed_kph(Some("")), None);
}

#[test]
fn parse_maxspeed_country_implicit_values() {
    assert_eq!(parse_maxspeed_kph(Some("PL:urban")), Some(50.0));
    assert_eq!(parse_maxspeed_kph(Some("de:rural")), Some(100.0));
    assert_eq!(
        parse_maxspeed_kph(Some("none")),
        Some(IMPLICIT_MAXSPEED_UNLIMITED_KPH)
    );
    assert_eq!(parse_maxspeed_kph(Some("walk")), Some(7.0));
    assert_eq!(parse_maxspeed_kph(Some("DE:zone30")), Some(30.0));
    assert_eq!(parse_maxspeed_kph(Some("DE:zone:30")), Some(30.0));
}

#[test]
fn parse_maxspeed_bare_context_resolves_against_country() {
    assert_eq!(
        parse_maxspeed_kph_with_country(Some("urban"), Some("PL")),
        Some(50.0)
    );
    assert_eq!(parse_maxspeed_kph_with_country(Some("urban"), None), None);
    assert_eq!(
        parse_maxspeed_kph_with_country(Some("rural"), Some("de")),
        Some(100.0)
    );
}

#[test]
fn parse_maxspeed_signals_and_unknown_fall_back() {
    assert_eq!(parse_maxspeed_kph(Some("signals")), None);
    assert_eq!(parse_maxspeed_kph(Some("variable")), None);
    assert_eq!(parse_maxspeed_kph(Some("XX:urban")), None);
    assert_eq!(parse_maxspeed_kph(Some("PL:autobahn")), None);
}

fn foot_area_way(
    id: i64,
    points: Vec<(f64, f64)>,
    node_ids: Vec<i64>,
    highway: &str,
    area: Option<&str>,
) -> RoutableWay {
    let source_node_ids = node_ids.into_iter().map(Some).collect();
    RoutableWay {
        osm_way_id: id,
        points,
        source_node_ids,
        road_type: RoadType::Path,
        directional_access: DirectionalAccess::Both,
        name: None,
        ref_name: None,
        highway: Some(highway.into()),
        access: Some("yes".into()),
        vehicle: None,
        motor_vehicle: Some("no".into()),
        motorcar: None,
        bicycle: Some("no".into()),
        foot: Some("yes".into()),
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
        conditional_restrictions: Vec::new(),
        area: area.map(Into::into),
        area_highway: None,
        has_bridge: false,
        has_tunnel: false,
        has_ford: false,
        is_roundabout: false,
        is_link: false,
        is_destination_only: false,
        access_restricted: false,
        source_lineage_ids: vec![id],
        coverage_id: None,
        ..Default::default()
    }
}

fn synthetic_plaza_infos(graph: &RoadGraph) -> Vec<SharedEdgeInfo> {
    graph
        .shared_edge_infos
        .iter()
        .filter(|info| info.osm_way_id < 0)
        .map(|info| info.to_info())
        .collect()
}

#[test]
fn plaza_star_synthesizes_center_and_foot_only_anchor_edges() {
    let a = (50.0620, 19.9360);
    let b = (50.0620, 19.9380);
    let c = (50.0600, 19.9380);
    let d = (50.0600, 19.9360);

    let mut builder = RoadGraphBuilder::new("plaza-star");
    builder.push_way(foot_area_way(
        1,
        vec![a, b, c, d, a],
        vec![1, 2, 3, 4, 1],
        "pedestrian",
        Some("yes"),
    ));
    // Three connecting footways, one per corner A/B/C; corner D stays isolated.
    builder.push_way(foot_area_way(
        2,
        vec![(50.0630, 19.9360), a],
        vec![10, 1],
        "footway",
        None,
    ));
    builder.push_way(foot_area_way(
        3,
        vec![(50.0620, 19.9390), b],
        vec![11, 2],
        "footway",
        None,
    ));
    builder.push_way(foot_area_way(
        4,
        vec![(50.0590, 19.9380), c],
        vec![12, 3],
        "footway",
        None,
    ));

    let (graph, report) = builder.finish_with_report();

    assert_eq!(report.synthesized_plaza_count, 1);
    assert_eq!(report.plaza_anchor_count, 3);
    assert_eq!(report.plaza_interior_edge_count, 6);
    assert_eq!(report.plaza_skipped_insufficient_anchors, 0);

    let synth = synthetic_plaza_infos(&graph);
    assert_eq!(synth.len(), 3);
    for info in &synth {
        assert_eq!(info.osm_forward_mode_mask, OsmMode::FOOT);
        assert_eq!(info.osm_reverse_mode_mask, OsmMode::FOOT);
        assert_eq!(info.road_type, RoadType::Path);
        assert_eq!(info.highway.as_deref(), Some("pedestrian"));
        assert!(info.geometry.len() == 2);
    }

    let synth_ids: std::collections::BTreeSet<u64> =
        synth.iter().map(|info| info.shared_edge_info_id).collect();
    let mut interior_edges = 0;
    for edge in graph.graph.edge_weights() {
        if synth_ids.contains(&edge.shared_edge_info_id) {
            interior_edges += 1;
            assert_eq!(
                edge.mobility_mode_mask,
                mobility_mode_mask_from_osm(OsmMode::FOOT)
            );
            assert_eq!(edge.mobility_mode_mask & MobilityMode::WHEELED, 0);
            assert!(edge.distance_m > 0.0);
        }
    }
    assert_eq!(interior_edges, 6);

    // Exactly one synthesized centre node, with no OSM source node id.
    let center_nodes = graph
        .graph
        .node_weights()
        .filter(|node| node.source_node_id.is_none())
        .count();
    assert_eq!(center_nodes, 1);
}

#[test]
fn plaza_with_single_anchor_is_not_synthesized() {
    let a = (50.0620, 19.9360);
    let b = (50.0620, 19.9380);
    let c = (50.0600, 19.9380);
    let d = (50.0600, 19.9360);

    let mut builder = RoadGraphBuilder::new("plaza-one-anchor");
    builder.push_way(foot_area_way(
        1,
        vec![a, b, c, d, a],
        vec![1, 2, 3, 4, 1],
        "pedestrian",
        Some("yes"),
    ));
    builder.push_way(foot_area_way(
        2,
        vec![(50.0630, 19.9360), a],
        vec![10, 1],
        "footway",
        None,
    ));

    let (graph, report) = builder.finish_with_report();

    assert_eq!(report.synthesized_plaza_count, 0);
    assert_eq!(report.plaza_skipped_insufficient_anchors, 1);
    assert!(synthetic_plaza_infos(&graph).is_empty());
}

#[test]
fn plaza_concave_ring_uses_interior_label_point() {
    use geo::{Centroid, Contains, LineString, Polygon};

    // U-shaped plaza whose centroid falls in the notch (outside the polygon).
    let unit = |x: f64, y: f64| (50.0600 + y * 0.001, 19.9300 + x * 0.001);
    let vertices = [
        unit(0.0, 0.0),
        unit(3.0, 0.0),
        unit(3.0, 3.0),
        unit(2.0, 3.0),
        unit(2.0, 1.0),
        unit(1.0, 1.0),
        unit(1.0, 3.0),
        unit(0.0, 3.0),
    ];
    let mut ring = vertices.to_vec();
    ring.push(vertices[0]);

    let mut builder = RoadGraphBuilder::new("plaza-concave");
    builder.push_way(foot_area_way(
        1,
        ring.clone(),
        vec![1, 2, 3, 4, 5, 6, 7, 8, 1],
        "pedestrian",
        Some("yes"),
    ));
    // Connect two ring vertices so we get two anchors.
    builder.push_way(foot_area_way(
        2,
        vec![unit(0.0, -1.0), vertices[0]],
        vec![20, 1],
        "footway",
        None,
    ));
    builder.push_way(foot_area_way(
        3,
        vec![unit(3.0, -1.0), vertices[1]],
        vec![21, 2],
        "footway",
        None,
    ));

    let (graph, report) = builder.finish_with_report();
    assert_eq!(report.synthesized_plaza_count, 1);

    let polygon = Polygon::new(
        LineString::from(
            ring.iter()
                .map(|(lat, lng)| geo::Coord { x: *lng, y: *lat })
                .collect::<Vec<_>>(),
        ),
        Vec::new(),
    );
    let centroid = polygon.centroid().unwrap();
    assert!(
        !polygon.contains(&centroid),
        "test fixture must have a centroid outside the concave ring"
    );

    let center = graph
        .graph
        .node_weights()
        .find(|node| node.source_node_id.is_none())
        .expect("synthesized centre node");
    let center_point = geo::Point::new(center.lng, center.lat);
    assert!(
        polygon.contains(&center_point),
        "fallback interior point must lie inside the polygon"
    );
}

#[test]
fn tiny_plaza_below_area_threshold_is_skipped() {
    // ~5.5m x ~3.5m square -> under the minimum area.
    let a = (50.06000, 19.93000);
    let b = (50.06000, 19.93005);
    let c = (50.06005, 19.93005);
    let d = (50.06005, 19.93000);

    let mut builder = RoadGraphBuilder::new("plaza-tiny");
    builder.push_way(foot_area_way(
        1,
        vec![a, b, c, d, a],
        vec![1, 2, 3, 4, 1],
        "pedestrian",
        Some("yes"),
    ));

    let (graph, report) = builder.finish_with_report();
    assert_eq!(report.synthesized_plaza_count, 0);
    assert_eq!(report.plaza_skipped_below_min_area, 1);
    assert!(synthetic_plaza_infos(&graph).is_empty());
}

fn node_barrier_tags(barrier: &str) -> NodeBarrierTags {
    NodeBarrierTags {
        barrier: barrier.to_string(),
        ..Default::default()
    }
}

/// A straight, all-mode service road whose middle vertex is the given node.
fn service_way_through_barrier(barrier_node_id: i64) -> RoutableWay {
    RoutableWay {
        osm_way_id: 50,
        points: vec![(51.0, 19.0), (51.0, 19.001), (51.0, 19.002)],
        source_node_ids: vec![Some(500), Some(barrier_node_id), Some(501)],
        road_type: RoadType::Residential,
        directional_access: DirectionalAccess::Both,
        name: None,
        ref_name: None,
        highway: Some("service".into()),
        access: None,
        vehicle: None,
        motor_vehicle: None,
        motorcar: None,
        bicycle: None,
        foot: None,
        oneway_foot: None,
        oneway_bicycle: None,
        motorroad: None,
        service: Some("driveway".into()),
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
        conditional_restrictions: Vec::new(),
        area: None,
        area_highway: None,
        has_bridge: false,
        has_tunnel: false,
        has_ford: false,
        is_roundabout: false,
        is_link: false,
        is_destination_only: false,
        access_restricted: false,
        source_lineage_ids: vec![50],
        coverage_id: None,
        ..Default::default()
    }
}

fn build_with_barrier(node_id: i64, tags: NodeBarrierTags) -> (RoadGraph, RoadGraphBuildReport) {
    let mut builder = RoadGraphBuilder::new("barrier");
    builder.push_way(service_way_through_barrier(node_id));
    let mut barriers = HashMap::new();
    barriers.insert(node_id, tags);
    builder.set_barrier_node_tags(barriers);
    builder.finish_with_report()
}

/// The barrier node's access and the graph's edge count: the way splits at
/// a barrier that changes routing (2 + 2 directed edges), else stays one
/// segment (2 directed edges).
fn barrier_node_access(graph: &RoadGraph, node_id: i64) -> (BarrierAccess, usize) {
    let node = graph
        .graph
        .node_weights()
        .find(|node| node.source_node_id == Some(node_id))
        .map_or(0, |node| node.barrier_access);
    (BarrierAccess(node), graph.edge_count())
}

/// Profile bits: vehicles (Default, Wheeled, Tracked), Emergency, walkers
/// (OnFoot, Legged).
const VEHICLES: u8 = 0b00_0111;
const EMERGENCY: u8 = 0b00_1000;
const WALKERS: u8 = 0b11_0000;

#[test]
fn node_barrier_semantics_cover_representative_values() {
    let bollard = derive_node_barrier(&node_barrier_tags("bollard")).unwrap();
    assert_eq!(bollard.passable_mask, OsmMode::FOOT | OsmMode::BICYCLE);
    assert_eq!(bollard.conditional_mask, 0);
    assert!(bollard.blocks_any());

    let gate = derive_node_barrier(&node_barrier_tags("gate")).unwrap();
    assert_eq!(gate.passable_mask, OsmMode::ALL);
    assert_eq!(gate.conditional_mask, OsmMode::ALL);
    assert!(!gate.blocks_any());
    assert!(gate.has_conditional());

    let mut locked = node_barrier_tags("gate");
    locked.locked = Some("yes".to_string());
    let locked = derive_node_barrier(&locked).unwrap();
    assert_eq!(locked.passable_mask, 0);
    assert!(locked.blocks_any());

    let mut gate_foot = node_barrier_tags("gate");
    gate_foot.foot = Some("yes".to_string());
    let gate_foot = derive_node_barrier(&gate_foot).unwrap();
    assert_ne!(gate_foot.passable_mask & OsmMode::FOOT, 0);
    assert_eq!(gate_foot.conditional_mask & OsmMode::FOOT, 0);
    assert_ne!(gate_foot.conditional_mask & OsmMode::MOTOR_VEHICLE, 0);

    let stile = derive_node_barrier(&node_barrier_tags("stile")).unwrap();
    assert_eq!(stile.passable_mask, OsmMode::FOOT);
    assert_eq!(stile.conditional_mask, OsmMode::FOOT);

    let cattle = derive_node_barrier(&node_barrier_tags("cattle_grid")).unwrap();
    assert!(!cattle.has_effect());
    assert_ne!(cattle.passable_mask & OsmMode::MOTOR_VEHICLE, 0);
    assert_ne!(cattle.passable_mask & OsmMode::FOOT, 0);

    assert!(derive_node_barrier(&node_barrier_tags("piano")).is_none());

    let mut private_gate = node_barrier_tags("gate");
    private_gate.access = Some("private".to_string());
    private_gate.foot = Some("yes".to_string());
    let private_gate = derive_node_barrier(&private_gate).unwrap();
    assert_ne!(private_gate.passable_mask & OsmMode::FOOT, 0);
    // Vehicles going somewhere behind a private gate pass it as a gate.
    assert_ne!(private_gate.conditional_mask & OsmMode::MOTOR_VEHICLE, 0);
    assert_eq!(private_gate.passable_mask & OsmMode::BICYCLE, 0);

    let mut private_bollard = node_barrier_tags("bollard");
    private_bollard.access = Some("private".to_string());
    let private_bollard = derive_node_barrier(&private_bollard).unwrap();
    assert_eq!(private_bollard.passable_mask, 0);

    let mut locked_private = node_barrier_tags("gate");
    locked_private.access = Some("private".to_string());
    locked_private.locked = Some("yes".to_string());
    assert_eq!(
        derive_node_barrier(&locked_private).unwrap().passable_mask,
        0
    );
}

#[test]
fn bollard_node_closes_vehicles() {
    let (graph, report) = build_with_barrier(700, node_barrier_tags("bollard"));

    assert_eq!(report.barrier_nodes_seen, 1);
    assert_eq!(report.barrier_constraints_applied, 1);
    assert_eq!(report.barrier_hard_block_count, 1);
    assert_eq!(report.barrier_conditional_count, 0);
    assert_eq!(
        barrier_node_access(&graph, 700),
        (BarrierAccess::new(VEHICLES | EMERGENCY, 0), 4)
    );
    // The segments either side keep the way's modes: the barrier applies at
    // transitions through its node.
    for edge in graph.graph.edge_weights() {
        assert_ne!(edge.mobility_mode_mask & MobilityMode::WHEELED, 0);
    }
}

#[test]
fn locked_gate_node_closes_every_profile() {
    let mut tags = node_barrier_tags("gate");
    tags.locked = Some("yes".to_string());
    let (graph, report) = build_with_barrier(700, tags);

    assert_eq!(report.barrier_constraints_applied, 1);
    assert_eq!(report.barrier_hard_block_count, 1);
    assert_eq!(report.barrier_conditional_count, 0);
    assert_eq!(
        barrier_node_access(&graph, 700),
        (BarrierAccess::new(VEHICLES | EMERGENCY | WALKERS, 0), 4)
    );
    assert_eq!(graph.node_count(), 3);
}

#[test]
fn plain_gate_node_gates_every_profile() {
    let (graph, report) = build_with_barrier(700, node_barrier_tags("gate"));

    assert_eq!(report.barrier_constraints_applied, 1);
    assert_eq!(report.barrier_hard_block_count, 0);
    assert_eq!(report.barrier_conditional_count, 1);
    assert_eq!(
        barrier_node_access(&graph, 700),
        (BarrierAccess::new(0, VEHICLES | EMERGENCY | WALKERS), 4)
    );
}

#[test]
fn stile_node_gates_walkers_and_closes_the_rest() {
    let (graph, report) = build_with_barrier(700, node_barrier_tags("stile"));

    assert_eq!(report.barrier_constraints_applied, 1);
    assert_eq!(report.barrier_hard_block_count, 1);
    assert_eq!(report.barrier_conditional_count, 1);
    assert_eq!(
        barrier_node_access(&graph, 700),
        (BarrierAccess::new(VEHICLES | EMERGENCY, WALKERS), 4)
    );
}

#[test]
fn cattle_grid_node_does_not_split_the_way() {
    let (graph, report) = build_with_barrier(700, node_barrier_tags("cattle_grid"));

    // Mid-way, it does not become a graph node.
    assert_eq!(report.barrier_nodes_seen, 0);
    assert_eq!(report.barrier_constraints_applied, 0);
    assert_eq!(report.barrier_unknown_value_count, 0);
    assert_eq!(barrier_node_access(&graph, 700), (BarrierAccess::NONE, 2));
}

#[test]
fn unknown_barrier_value_is_counted_not_applied() {
    let (graph, report) = build_with_barrier(700, node_barrier_tags("piano"));

    assert_eq!(report.barrier_nodes_seen, 1);
    assert_eq!(report.barrier_unknown_value_count, 1);
    assert_eq!(report.barrier_constraints_applied, 0);
    // The way splits there all the same: SAR's `barriers` relaxation costs
    // unknown values.
    assert_eq!(barrier_node_access(&graph, 700), (BarrierAccess::NONE, 4));
}

#[test]
fn road_type_labels_are_sorted_and_complete() {
    // Node road-type tallies are fixed arrays now, but they are flattened into
    // the same order the previous `BTreeMap<String, _>` emitted: alphabetical by
    // label. If either the labels or this ordering table drift, node metadata
    // changes silently.
    use crate::{ROAD_TYPE_COUNT, ROAD_TYPES_BY_LABEL, road_type_index, road_type_label};

    let labels: Vec<&str> = ROAD_TYPES_BY_LABEL
        .into_iter()
        .map(road_type_label)
        .collect();
    let mut sorted = labels.clone();
    sorted.sort_unstable();
    assert_eq!(labels, sorted, "ROAD_TYPES_BY_LABEL must be label-sorted");

    let mut slots: Vec<usize> = ROAD_TYPES_BY_LABEL
        .into_iter()
        .map(road_type_index)
        .collect();
    slots.sort_unstable();
    assert_eq!(
        slots,
        (0..ROAD_TYPE_COUNT).collect::<Vec<_>>(),
        "every road type needs exactly one tally slot"
    );
}

/// OSM physical-limit values are free text. These cases are the shapes that
/// actually occur in the planet dump; the parsers exist to turn them into hard
/// gate scalars, so a misparse becomes a wrong routing decision rather than a
/// visible error.
mod physical_limits {
    use crate::{parse_incline_percent, parse_length_cm, parse_mass_kg};

    #[test]
    fn length_defaults_to_metres_and_accepts_explicit_units() {
        assert_eq!(parse_length_cm(Some("3.5")), Some(350));
        assert_eq!(parse_length_cm(Some("3.5 m")), Some(350));
        assert_eq!(parse_length_cm(Some("3.5m")), Some(350));
        assert_eq!(parse_length_cm(Some("350 cm")), Some(350));
        assert_eq!(parse_length_cm(Some("3500mm")), Some(350));
    }

    /// Continental European tagging writes `3,5` for three and a half metres.
    /// Read as an unparseable string this would silently drop a real limit.
    #[test]
    fn length_accepts_comma_decimal_separator() {
        assert_eq!(parse_length_cm(Some("3,5")), Some(350));
        assert_eq!(parse_length_cm(Some("2,75 m")), Some(275));
    }

    #[test]
    fn length_accepts_imperial_shorthand_and_units() {
        assert_eq!(parse_length_cm(Some("12'6\"")), Some(381));
        assert_eq!(parse_length_cm(Some("12'")), Some(366));
        assert_eq!(parse_length_cm(Some("10 ft")), Some(305));
        assert_eq!(parse_length_cm(Some("18 in")), Some(46));
    }

    /// `none` asserts there is no restriction. It must not become a limit, and
    /// neither must a value we failed to understand — an unreadable tag has to
    /// stay unconstrained rather than block a road we cannot prove is blocked.
    #[test]
    fn length_sentinels_and_garbage_are_unconstrained() {
        for value in ["none", "no", "default", "unsigned", "unknown"] {
            assert_eq!(parse_length_cm(Some(value)), None, "value = {value}");
        }
        assert_eq!(parse_length_cm(Some("tall")), None);
        assert_eq!(parse_length_cm(Some("3.5 furlongs")), None);
        assert_eq!(parse_length_cm(Some("")), None);
        assert_eq!(parse_length_cm(Some("0")), None);
        assert_eq!(parse_length_cm(Some("-2")), None);
        assert_eq!(parse_length_cm(None), None);
    }

    #[test]
    fn mass_defaults_to_tonnes_and_accepts_explicit_units() {
        assert_eq!(parse_mass_kg(Some("7.5")), Some(7_500));
        assert_eq!(parse_mass_kg(Some("7.5 t")), Some(7_500));
        assert_eq!(parse_mass_kg(Some("12tonnes")), Some(12_000));
        assert_eq!(parse_mass_kg(Some("500 kg")), Some(500));
        assert_eq!(parse_mass_kg(Some("3,5")), Some(3_500));
    }

    #[test]
    fn mass_accepts_imperial_units() {
        assert_eq!(parse_mass_kg(Some("1000 lbs")), Some(454));
        assert_eq!(parse_mass_kg(Some("2 st")), Some(1_814));
    }

    #[test]
    fn mass_sentinels_and_garbage_are_unconstrained() {
        for value in ["none", "no", "default", "unsigned", "unknown"] {
            assert_eq!(parse_mass_kg(Some(value)), None, "value = {value}");
        }
        assert_eq!(parse_mass_kg(Some("heavy")), None);
        assert_eq!(parse_mass_kg(Some("0")), None);
        assert_eq!(parse_mass_kg(None), None);
    }

    #[test]
    fn incline_reads_signed_percent_and_degrees() {
        assert_eq!(parse_incline_percent(Some("10%")), Some(10));
        assert_eq!(parse_incline_percent(Some("-5%")), Some(-5));
        assert_eq!(parse_incline_percent(Some("7")), Some(7));
        assert_eq!(parse_incline_percent(Some("3,5%")), Some(4));
        // tan(6 deg) = 10.5%
        assert_eq!(parse_incline_percent(Some("6°")), Some(11));
        assert_eq!(parse_incline_percent(Some("6 deg")), Some(11));
    }

    /// `up` / `down` state a direction with no magnitude. Inventing a gradient
    /// for them would put a fabricated number into edge costing.
    #[test]
    fn incline_qualitative_values_have_no_magnitude() {
        for value in ["up", "down", "steep", "yes", "no"] {
            assert_eq!(parse_incline_percent(Some(value)), None, "value = {value}");
        }
    }

    #[test]
    fn incline_clamps_absurd_gradients() {
        assert_eq!(parse_incline_percent(Some("900%")), Some(127));
        assert_eq!(parse_incline_percent(Some("-900%")), Some(-127));
    }
}

#[test]
fn highway_defaults_keep_vehicles_off_footways_and_close_construction() {
    let mask = |highway: &str, tags: &[(&str, &str)]| {
        let tag = |key: &str| {
            tags.iter()
                .find(|(candidate, _)| *candidate == key)
                .map(|(_, value)| *value)
        };
        derive_osm_mode_mask(
            Some(highway),
            tag("access"),
            tag("vehicle"),
            tag("motor_vehicle"),
            tag("motorcar"),
            tag("bicycle"),
            tag("foot"),
            tag("motorroad"),
        )
    };
    for highway in ["footway", "pedestrian", "steps", "corridor", "bridleway"] {
        assert_eq!(mask(highway, &[]), OsmMode::FOOT, "{highway}");
    }
    assert_eq!(mask("path", &[]), OsmMode::FOOT | OsmMode::BICYCLE);
    assert_eq!(mask("cycleway", &[]), OsmMode::FOOT | OsmMode::BICYCLE);
    assert_eq!(mask("track", &[]), OsmMode::ALL);
    assert_eq!(mask("residential", &[]), OsmMode::ALL);
    // A generic grant restores the defaults and no more; a mode's own grant
    // opens it.
    assert_eq!(mask("footway", &[("access", "yes")]), OsmMode::FOOT);
    assert_eq!(
        mask("footway", &[("motor_vehicle", "yes")]),
        OsmMode::FOOT | OsmMode::MOTOR_VEHICLE | OsmMode::BUS | OsmMode::HGV | OsmMode::EMERGENCY
    );
    assert_eq!(
        mask("residential", &[("access", "no"), ("foot", "yes")]),
        OsmMode::FOOT
    );
    for highway in [
        "construction",
        "proposed",
        "abandoned",
        "razed",
        "platform",
        "bus_guideway",
    ] {
        assert_eq!(mask(highway, &[]), 0, "{highway}");
        assert_eq!(mask(highway, &[("access", "yes")]), 0, "{highway}");
    }
    assert_eq!(mask("platform", &[("foot", "yes")]), OsmMode::FOOT);
    // No `highway` (a ferry) admits every mode.
    assert_eq!(
        derive_osm_mode_mask(None, None, None, None, None, None, None, None),
        OsmMode::ALL
    );
}

#[test]
fn ways_no_mode_may_use_are_dropped() {
    let mut builder = RoadGraphBuilder::new("no-mode");
    builder.push_way(RoutableWay {
        osm_way_id: 1,
        points: vec![(52.0, 21.0), (52.0, 21.001)],
        road_type: RoadType::Other,
        highway: Some("construction".into()),
        ..RoutableWay::default()
    });
    let (graph, report) = builder.finish_with_report();
    assert_eq!(graph.edge_count(), 0);
    assert!(graph.shared_edge_infos.is_empty());
    assert_eq!(
        report.rejected_way_counts_by_reason,
        vec![RoadGraphNamedCount {
            name: "no_mode".to_string(),
            count: 1,
        }]
    );
}

#[test]
fn route_refs_come_from_road_routes_only() {
    let mut builder = RoadGraphBuilder::new("route-refs");
    builder.push_way(RoutableWay {
        osm_way_id: 10,
        points: vec![(52.0, 21.0), (52.0, 21.001)],
        road_type: RoadType::Primary,
        ..RoutableWay::default()
    });
    let mut graph = builder.finish();
    let membership = |relation_id: i64, route: &str, ref_name: &str| RoadRouteMembership {
        relation_id,
        way_id: 10,
        route: route.to_string(),
        network: None,
        ref_name: Some(ref_name.to_string()),
        relation_name: None,
        member_role: None,
    };
    graph.set_route_memberships(vec![
        membership(1, "road", "D6007"),
        membership(2, "bus", "12"),
        membership(3, "hiking", "GR 51"),
        membership(4, "bicycle", "EV8"),
    ]);
    graph.refresh_route_membership_metadata();
    assert_eq!(
        graph.shared_edge_infos.at(0).attrs.route_refs,
        vec!["D6007"]
    );
}

fn control_way(osm_way_id: i64, points: Vec<(f64, f64)>, road_type: RoadType) -> RoutableWay {
    let highway = if road_type == RoadType::Primary {
        "primary"
    } else {
        "residential"
    };
    RoutableWay {
        osm_way_id,
        source_node_ids: vec![None; points.len()],
        points,
        road_type,
        directional_access: DirectionalAccess::Both,
        highway: Some(highway.into()),
        source_lineage_ids: vec![osm_way_id],
        ..Default::default()
    }
}

fn control_point(lat: f64, lng: f64, control: TrafficControl) -> TrafficControlPoint {
    TrafficControlPoint {
        lat,
        lng,
        control,
        direction: TrafficControlDirection::Both,
    }
}

/// The edge of `graph` from the node at `from` to the node at `to`.
fn edge_at(graph: &RoadGraph, from: (f64, f64), to: (f64, f64)) -> RoadEdge {
    let at = |index: NodeIndex, (lat, lng): (f64, f64)| {
        let node = &graph.graph[index];
        (node.lat - lat).abs() < 1e-9 && (node.lng - lng).abs() < 1e-9
    };
    graph
        .graph
        .edge_references()
        .find(|edge| at(edge.source(), from) && at(edge.target(), to))
        .map(|edge| edge.weight().clone())
        .expect("edge")
}

#[test]
fn traffic_controls_go_to_the_approach_they_control() {
    // Primary W-J-E through junction J, residential side road J-N. Signals 7 m
    // before J on W-J and 7 m after it on J-E (untagged direction, as stop
    // lines on two-way roads usually are), crossing signals mid-block on J-E,
    // and a give-way sign at N's end of the side road.
    let (w, s1, j, s2, s3, e) = (
        (50.0, 19.0),
        (50.0, 19.0029),
        (50.0, 19.003),
        (50.0, 19.0031),
        (50.0, 19.005),
        (50.0, 19.007),
    );
    let n = (50.002, 19.003);
    let mut builder = RoadGraphBuilder::new("controls");
    builder.push_way(control_way(1, vec![w, s1, j, s2, s3, e], RoadType::Primary));
    builder.push_way(control_way(2, vec![j, n], RoadType::Residential));
    builder.set_traffic_control_points(vec![
        control_point(s1.0, s1.1, TrafficControl::TrafficSignals),
        control_point(s2.0, s2.1, TrafficControl::TrafficSignals),
        control_point(s3.0, s3.1, TrafficControl::CrossingSignals),
        control_point(n.0, n.1, TrafficControl::GiveWay),
    ]);
    let (mut graph, report) = builder.finish_with_report();
    enhance_road_graph(&mut graph);
    let control = |from, to| edge_at(&graph, from, to).end_control;
    // Each junction approach meets its own signals; leaving J meets none.
    assert_eq!(control(w, j), TrafficControl::TrafficSignals);
    assert_eq!(control(e, j), TrafficControl::TrafficSignals);
    assert_eq!(control(j, w), TrafficControl::None);
    // Leaving J eastwards passes only the mid-block crossing.
    assert_eq!(control(j, e), TrafficControl::CrossingSignals);
    // A sign at a node controls the edges arriving there.
    assert_eq!(control(j, n), TrafficControl::GiveWay);
    assert_eq!(control(n, j), TrafficControl::None);
    assert_eq!(report.traffic_controlled_edge_count, 4);
    // The side road yields to the primary at J; the primary does not, and a
    // dead end is no junction.
    assert!(edge_at(&graph, n, j).end_minor_approach);
    assert!(!edge_at(&graph, w, j).end_minor_approach);
    assert!(!edge_at(&graph, j, n).end_minor_approach);
}

#[test]
fn signals_inside_one_junction_count_once() {
    // Two signalised junction nodes 20 m apart (a dual carriageway crossing):
    // the short edge between them is inside the junction.
    let (a, j1, j2, b) = (
        (50.0, 19.0),
        (50.0, 19.002),
        (50.0, 19.00228),
        (50.0, 19.004),
    );
    let mut builder = RoadGraphBuilder::new("internal");
    builder.push_way(control_way(1, vec![a, j1, j2, b], RoadType::Primary));
    builder.push_way(control_way(
        2,
        vec![j1, (50.001, 19.002)],
        RoadType::Residential,
    ));
    builder.push_way(control_way(
        3,
        vec![j2, (49.999, 19.00228)],
        RoadType::Residential,
    ));
    builder.set_traffic_control_points(vec![
        control_point(j1.0, j1.1, TrafficControl::TrafficSignals),
        control_point(j2.0, j2.1, TrafficControl::TrafficSignals),
    ]);
    let graph = builder.finish();
    assert_eq!(
        edge_at(&graph, a, j1).end_control,
        TrafficControl::TrafficSignals
    );
    assert_eq!(edge_at(&graph, j1, j2).end_control, TrafficControl::None);
    assert_eq!(edge_at(&graph, j2, j1).end_control, TrafficControl::None);
    assert_eq!(
        edge_at(&graph, b, j2).end_control,
        TrafficControl::TrafficSignals
    );
}

#[test]
fn barrier_classes_for_sar_follow_the_barrier_and_its_access() {
    use wayfarer_package::sar::SarBarrierClass;
    let tags = |barrier: &str, access: Option<&str>, locked: Option<&str>| NodeBarrierTags {
        barrier: barrier.into(),
        access: access.map(Into::into),
        locked: locked.map(Into::into),
        ..Default::default()
    };
    let cases = [
        (tags("gate", None, None), Some(SarBarrierClass::Gate)),
        (tags("lift_gate", None, None), Some(SarBarrierClass::Gate)),
        // A private gate opens to vehicles going somewhere behind it, as a
        // private road does; locked, it stays shut.
        (
            tags("gate", Some("private"), None),
            Some(SarBarrierClass::Gate),
        ),
        (
            tags("gate", Some("customers"), None),
            Some(SarBarrierClass::Gate),
        ),
        (
            tags("gate", Some("private"), Some("yes")),
            Some(SarBarrierClass::LockedGate),
        ),
        (
            tags("gate", Some("no"), None),
            Some(SarBarrierClass::LockedGate),
        ),
        (
            tags("gate", None, Some("yes")),
            Some(SarBarrierClass::LockedGate),
        ),
        (tags("chain", None, None), Some(SarBarrierClass::LockedGate)),
        (tags("gate", Some("yes"), None), None),
        (tags("gate", Some("destination"), None), None),
        (
            tags("bollard", None, None),
            Some(SarBarrierClass::MotorBlocking),
        ),
        (
            tags("cycle_barrier", None, None),
            Some(SarBarrierClass::MotorBlocking),
        ),
        (tags("stile", None, None), Some(SarBarrierClass::FootOnly)),
        (tags("kerb", None, None), None),
        (tags("cattle_grid", None, None), None),
        (
            tags("wicket_gate", None, None),
            Some(SarBarrierClass::Unknown),
        ),
    ];
    for (tags, expected) in cases {
        assert_eq!(sar_barrier_class(&tags), expected, "{tags:?}");
    }
}

#[test]
fn barrier_nodes_get_their_sar_class() {
    let mut builder = RoadGraphBuilder::new("barriers");
    let mut barriers = HashMap::new();
    barriers.insert(
        101,
        NodeBarrierTags {
            barrier: "gate".into(),
            access: Some("no".into()),
            ..Default::default()
        },
    );
    builder.set_barrier_node_tags(barriers);
    builder.push_way(road_way(None));
    let (graph, report) = builder.finish_with_report();
    let node = graph
        .graph
        .node_weights()
        .find(|node| node.source_node_id == Some(101))
        .unwrap();
    assert_eq!(
        node.barrier_class,
        wayfarer_package::sar::SarBarrierClass::LockedGate as u8
    );
    assert_eq!(report.sar_barrier_node_count, 1);
}

use super::*;

fn attrs(pairs: &[(&str, &str)]) -> Vec<(String, String)> {
    pairs
        .iter()
        .map(|(key, value)| ((*key).to_string(), (*value).to_string()))
        .collect()
}

fn routing(class: &str) -> Option<Arc<RoadRoutingAttrs>> {
    Some(Arc::new(RoadRoutingAttrs {
        road_class: class.to_string(),
        directional_access: DirectionalAccessModel::Both,
        access: None,
        surface: None,
        tracktype: None,
        smoothness: None,
        has_ford: false,
        has_tunnel: false,
        access_restricted: false,
    }))
}

fn road_input(
    feature_id: i64,
    geometry: Vec<(f64, f64)>,
    node_chain: Vec<i64>,
    attrs: Vec<(String, String)>,
) -> SharedLineInput {
    SharedLineInput {
        feature_id,
        layer: SharedLineLayer::Roads,
        geometry,
        node_chain,
        source_lineage_ids: vec![feature_id],
        // Fixtures are written as raw OSM tags, which is what road labels
        // and carriageway widths read.
        compile_inputs: attrs.clone().into(),
        attrs: attrs.into(),
        min_zoom: 12,
        max_zoom: 14,
        sort_order: 1,
        routing: routing("residential"),
    }
}

#[test]
fn nowotarska_service_connections_survive_render_generation() {
    let mut builder = SharedLineNetworkBuilder::default();
    builder.extend_inputs([
        road_input(
            174663545,
            vec![
                (49.2995639, 19.9517011),
                (49.2995530, 19.9516477),
                (49.2995466, 19.9515901),
                (49.2995432, 19.9515409),
                (49.2995317, 19.9513928),
                (49.2995168, 19.9512494),
                (49.2994908, 19.9510854),
                (49.2993426, 19.9504494),
                (49.2992727, 19.9501498),
                (49.2992146, 19.9498763),
                (49.2991720, 19.9496661),
                (49.2991463, 19.9495111),
                (49.2991271, 19.9493793),
                (49.2991231, 19.9492975),
                (49.2991252, 19.9492423),
            ],
            vec![
                1853182026,
                11157816165,
                13979111251,
                3005740943,
                1853183618,
                6177334619,
                13979111256,
                1853377442,
                1853183596,
                1853183589,
                13979111255,
                13979111254,
                3005741019,
                1853183572,
                1853183553,
            ],
            attrs(&[("class", "secondary")]),
        ),
        road_input(
            174686852,
            vec![
                (49.2995143, 19.9489407),
                (49.2993703, 19.9491017),
                (49.2992235, 19.9492471),
                (49.2991953, 19.9492638),
                (49.2991608, 19.9492689),
                (49.2991252, 19.9492423),
            ],
            vec![
                1853377451, 3378281041, 3378281024, 3378281020, 3378281017, 1853183553,
            ],
            attrs(&[("class", "service")]),
        ),
        road_input(
            174686861,
            vec![
                (49.2997438, 19.9493647),
                (49.2993029, 19.9498125),
                (49.2992465, 19.9498599),
                (49.2992146, 19.9498763),
            ],
            vec![1853377462, 3378281031, 3005741127, 1853183589],
            attrs(&[("class", "service")]),
        ),
        road_input(
            174686862,
            vec![
                (49.2993426, 19.9504494),
                (49.2993693, 19.9504330),
                (49.2997250, 19.9500836),
                (49.2998367, 19.9499792),
                (49.2999323, 19.9499208),
                (49.3000403, 19.9498137),
            ],
            vec![
                1853377442, 3005740873, 1853377463, 1853377469, 1853377471, 3378284514,
            ],
            attrs(&[("class", "service")]),
        ),
        road_input(
            174686866,
            vec![
                (49.2992727, 19.9501498),
                (49.2992983, 19.9501312),
                (49.2993541, 19.9501006),
                (49.2994056, 19.9501175),
                (49.2994290, 19.9501722),
                (49.2996528, 19.9499575),
            ],
            vec![
                1853183596, 8829412352, 1853377444, 1853377445, 1853377446, 8829412351,
            ],
            attrs(&[("class", "service")]),
        ),
        road_input(
            296768379,
            vec![
                (49.2993693, 19.9504330),
                (49.2995021, 19.9509776),
                (49.2995870, 19.9513908),
                (49.2996099, 19.9515327),
                (49.2996276, 19.9515556),
                (49.2996670, 19.9517341),
                (49.2997351, 19.9520486),
                (49.2998368, 19.9524465),
                (49.2999795, 19.9530540),
            ],
            vec![
                3005740873, 3005741026, 1853377453, 3005740851, 3005741048, 3005740939, 3005740925,
                3378285798, 3005740970,
            ],
            attrs(&[("class", "footway")]),
        ),
        road_input(
            953951266,
            vec![
                (49.2992144, 19.9488411),
                (49.2991946, 19.9491401),
                (49.2991608, 19.9492689),
                (49.2991571, 19.9493088),
                (49.2991623, 19.9493732),
                (49.2991738, 19.9494757),
                (49.2992465, 19.9498599),
                (49.2992983, 19.9501312),
            ],
            vec![
                3378281022, 3005741132, 3378281017, 3378281016, 4364203977, 3005741247, 3005741127,
                8829412352,
            ],
            attrs(&[("class", "footway")]),
        ),
        road_input(
            953951268,
            vec![(49.2992983, 19.9501312), (49.2993693, 19.9504330)],
            vec![8829412352, 3005740873],
            attrs(&[("class", "footway")]),
        ),
    ]);
    let network = builder.finish();
    let mut missing = Vec::new();
    let junction = (49.2991252, 19.9492423);
    let has_point = |points: &[(f64, f64)]| {
        points
            .iter()
            .any(|p| (p.0 - junction.0).abs() < 1e-7 && (p.1 - junction.1).abs() < 1e-7)
    };
    assert!(
        network
            .topology
            .edges
            .iter()
            .any(|e| e.feature_id == 174686852 && has_point(&e.geometry)),
        "source topology must connect 174686852"
    );
    if !network
        .render_lines_for_layer(SharedLineLayer::Roads)
        .any(|line| line.source_lineage_ids.contains(&174686852) && has_point(&line.geometry))
    {
        missing.push(174686852);
    }
    let junction = (49.2992146, 19.9498763);
    let has_point = |points: &[(f64, f64)]| {
        points
            .iter()
            .any(|p| (p.0 - junction.0).abs() < 1e-7 && (p.1 - junction.1).abs() < 1e-7)
    };
    assert!(
        network
            .topology
            .edges
            .iter()
            .any(|e| e.feature_id == 174686861 && has_point(&e.geometry)),
        "source topology must connect 174686861"
    );
    if !network
        .render_lines_for_layer(SharedLineLayer::Roads)
        .any(|line| line.source_lineage_ids.contains(&174686861) && has_point(&line.geometry))
    {
        missing.push(174686861);
    }
    let junction = (49.2993426, 19.9504494);
    let has_point = |points: &[(f64, f64)]| {
        points
            .iter()
            .any(|p| (p.0 - junction.0).abs() < 1e-7 && (p.1 - junction.1).abs() < 1e-7)
    };
    assert!(
        network
            .topology
            .edges
            .iter()
            .any(|e| e.feature_id == 174686862 && has_point(&e.geometry)),
        "source topology must connect 174686862"
    );
    if !network
        .render_lines_for_layer(SharedLineLayer::Roads)
        .any(|line| line.source_lineage_ids.contains(&174686862) && has_point(&line.geometry))
    {
        missing.push(174686862);
    }
    let junction = (49.2992727, 19.9501498);
    let has_point = |points: &[(f64, f64)]| {
        points
            .iter()
            .any(|p| (p.0 - junction.0).abs() < 1e-7 && (p.1 - junction.1).abs() < 1e-7)
    };
    assert!(
        network
            .topology
            .edges
            .iter()
            .any(|e| e.feature_id == 174686866 && has_point(&e.geometry)),
        "source topology must connect 174686866"
    );
    if !network
        .render_lines_for_layer(SharedLineLayer::Roads)
        .any(|line| line.source_lineage_ids.contains(&174686866) && has_point(&line.geometry))
    {
        missing.push(174686866);
    }
    assert!(
        missing.is_empty(),
        "render products lost source junctions: {missing:?}"
    );
}
#[test]
fn road_render_tier_discriminants_match_the_bucket_order() {
    // `build_road_render_products` indexes its per-tier buckets with
    // `tier as usize`. Reordering either the enum or `ROAD_RENDER_TIERS`
    // would silently route edges into the wrong tier's render pass.
    for (index, tier) in ROAD_RENDER_TIERS.into_iter().enumerate() {
        assert_eq!(tier as usize, index, "{tier:?} moved out of bucket order");
    }
}

#[test]
fn repairs_same_plane_road_crossings_into_connected_topology() {
    let mut builder = SharedLineNetworkBuilder::default();
    builder.extend_inputs([
        road_input(
            1,
            vec![(50.0, 19.0), (50.0, 19.002)],
            vec![101, 102],
            attrs(&[("class", "residential")]),
        ),
        road_input(
            2,
            vec![(49.999, 19.001), (50.001, 19.001)],
            vec![201, 202],
            attrs(&[("class", "residential")]),
        ),
    ]);

    let network = builder.finish();

    assert_eq!(network.topology.road_segments().count(), 4);
    assert_eq!(network.topology.nodes.len(), 5);
    assert_eq!(network.stats.topology_repair_repaired_crossings, 1);
    assert_eq!(network.stats.topology_repair_inserted_topology_nodes, 1);
    assert_eq!(network.stats.topology_repair_split_edges, 2);
}

#[test]
fn bridge_crossing_does_not_connect_to_surface_road() {
    let mut builder = SharedLineNetworkBuilder::default();
    builder.extend_inputs([
        road_input(
            1,
            vec![(50.0, 19.0), (50.0, 19.002)],
            vec![101, 102],
            attrs(&[("class", "residential"), ("bridge", "yes")]),
        ),
        road_input(
            2,
            vec![(49.999, 19.001), (50.001, 19.001)],
            vec![201, 202],
            attrs(&[("class", "residential")]),
        ),
    ]);

    let network = builder.finish();

    assert_eq!(network.topology.road_segments().count(), 2);
    assert_eq!(network.topology.nodes.len(), 4);
    assert_eq!(network.stats.topology_repair_repaired_crossings, 0);
    assert_eq!(network.stats.topology_repair_skipped_incompatible_plane, 1);
}

#[test]
fn tunnel_crossing_does_not_connect_to_surface_road() {
    let mut builder = SharedLineNetworkBuilder::default();
    builder.extend_inputs([
        road_input(
            1,
            vec![(50.0, 19.0), (50.0, 19.002)],
            vec![101, 102],
            attrs(&[("class", "residential"), ("tunnel", "yes")]),
        ),
        road_input(
            2,
            vec![(49.999, 19.001), (50.001, 19.001)],
            vec![201, 202],
            attrs(&[("class", "residential")]),
        ),
    ]);

    let network = builder.finish();

    assert_eq!(network.topology.road_segments().count(), 2);
    assert_eq!(network.topology.nodes.len(), 4);
    assert_eq!(network.stats.topology_repair_repaired_crossings, 0);
    assert_eq!(network.stats.topology_repair_skipped_incompatible_plane, 1);
}

#[test]
fn negated_brunnel_tags_and_layer_zero_stay_on_the_surface() {
    for tags in [
        &[("class", "residential"), ("tunnel", "no")][..],
        &[("class", "residential"), ("bridge", "no")],
        &[("class", "residential"), ("layer", "0")],
    ] {
        let mut builder = SharedLineNetworkBuilder::default();
        builder.extend_inputs([
            road_input(
                1,
                vec![(50.0, 19.0), (50.0, 19.002)],
                vec![101, 102],
                attrs(tags),
            ),
            road_input(
                2,
                vec![(49.999, 19.001), (50.001, 19.001)],
                vec![201, 202],
                attrs(&[("class", "residential")]),
            ),
        ]);

        let network = builder.finish();

        assert_eq!(
            network.stats.topology_repair_repaired_crossings, 1,
            "{tags:?}"
        );
    }
}

#[test]
fn ferry_crossings_are_never_repaired() {
    let mut builder = SharedLineNetworkBuilder::default();
    builder.extend_inputs([
        road_input(
            1,
            vec![(50.0, 19.0), (50.0, 19.002)],
            vec![101, 102],
            attrs(&[("class", "other"), ("route", "ferry")]),
        ),
        road_input(
            2,
            vec![(49.999, 19.001), (50.001, 19.001)],
            vec![201, 202],
            attrs(&[("class", "other"), ("route", "ferry")]),
        ),
        road_input(
            3,
            vec![(49.999, 19.0015), (50.001, 19.0015)],
            vec![301, 302],
            attrs(&[("class", "residential")]),
        ),
    ]);

    let network = builder.finish();

    assert_eq!(network.stats.topology_repair_repaired_crossings, 0);
    assert_eq!(network.stats.topology_repair_skipped_incompatible_plane, 2);
    assert_eq!(network.topology.road_segments().count(), 3);
}

#[test]
fn endpoint_touch_does_not_create_duplicate_repair_node() {
    let mut builder = SharedLineNetworkBuilder::default();
    builder.extend_inputs([
        road_input(
            1,
            vec![(50.0, 19.0), (50.0, 19.001)],
            vec![101, 102],
            attrs(&[("class", "residential")]),
        ),
        road_input(
            2,
            vec![(50.0, 19.001), (50.001, 19.001)],
            vec![102, 202],
            attrs(&[("class", "residential")]),
        ),
    ]);

    let network = builder.finish();

    assert_eq!(network.topology.road_segments().count(), 2);
    assert_eq!(network.topology.nodes.len(), 3);
    assert_eq!(network.stats.topology_repair_repaired_crossings, 0);
    assert_eq!(network.stats.topology_repair_skipped_endpoint_touches, 1);
}

#[test]
fn same_feature_crossing_is_reported_but_not_repaired() {
    let mut builder = SharedLineNetworkBuilder::default();
    builder.push_input(road_input(
        1,
        vec![
            (50.0, 19.0),
            (50.001, 19.001),
            (50.0, 19.001),
            (50.001, 19.0),
        ],
        vec![101, 102, 103, 104],
        attrs(&[("class", "residential")]),
    ));

    let network = builder.finish();

    assert_eq!(network.stats.topology_repair_repaired_crossings, 0);
    assert_eq!(network.stats.topology_repair_skipped_same_feature, 1);
}

#[test]
fn splits_topology_on_shared_source_nodes() {
    let mut builder = SharedLineNetworkBuilder::default();
    builder.extend_inputs([
        SharedLineInput {
            feature_id: 1,
            layer: SharedLineLayer::Roads,
            geometry: vec![(50.0, 19.0), (50.0, 19.001), (50.0, 19.002)],
            node_chain: vec![10, 11, 12],
            source_lineage_ids: vec![1],
            attrs: attrs(&[("class", "residential")]).into(),
            compile_inputs: attrs(&[("class", "residential")]).into(),
            min_zoom: 12,
            max_zoom: 14,
            sort_order: 1,
            routing: Some(Arc::new(RoadRoutingAttrs {
                road_class: "residential".to_string(),
                directional_access: DirectionalAccessModel::Both,
                access: None,
                surface: None,
                tracktype: None,
                smoothness: None,
                has_ford: false,
                has_tunnel: false,
                access_restricted: false,
            })),
        },
        SharedLineInput {
            feature_id: 2,
            layer: SharedLineLayer::Roads,
            geometry: vec![(50.001, 19.001), (50.0, 19.001), (49.999, 19.001)],
            node_chain: vec![20, 11, 21],
            source_lineage_ids: vec![2],
            attrs: attrs(&[("class", "residential")]).into(),
            compile_inputs: attrs(&[("class", "residential")]).into(),
            min_zoom: 12,
            max_zoom: 14,
            sort_order: 1,
            routing: Some(Arc::new(RoadRoutingAttrs {
                road_class: "residential".to_string(),
                directional_access: DirectionalAccessModel::Both,
                access: None,
                surface: None,
                tracktype: None,
                smoothness: None,
                has_ford: false,
                has_tunnel: false,
                access_restricted: false,
            })),
        },
    ]);

    let network = builder.finish();
    assert_eq!(network.topology.road_segments().count(), 4);
    assert_eq!(network.topology.nodes.len(), 5);
}

#[test]
fn stroke_merges_straight_continuation_through_three_way_junction() {
    let mut builder = SharedLineNetworkBuilder::default();
    builder.extend_inputs([
        SharedLineInput {
            feature_id: 1,
            layer: SharedLineLayer::Roads,
            geometry: vec![(50.0, 19.0), (50.0, 19.001)],
            node_chain: vec![1, 2],
            source_lineage_ids: vec![1],
            attrs: attrs(&[("class", "primary")]).into(),
            compile_inputs: attrs(&[("class", "primary")]).into(),
            min_zoom: 10,
            max_zoom: 14,
            sort_order: 1,
            routing: None,
        },
        SharedLineInput {
            feature_id: 2,
            layer: SharedLineLayer::Roads,
            geometry: vec![(50.0, 19.001), (50.0, 19.002)],
            node_chain: vec![2, 3],
            source_lineage_ids: vec![2],
            attrs: attrs(&[("class", "primary")]).into(),
            compile_inputs: attrs(&[("class", "primary")]).into(),
            min_zoom: 10,
            max_zoom: 14,
            sort_order: 1,
            routing: None,
        },
        SharedLineInput {
            feature_id: 3,
            layer: SharedLineLayer::Roads,
            geometry: vec![(50.0, 19.001), (50.001, 19.001)],
            node_chain: vec![2, 4],
            source_lineage_ids: vec![3],
            attrs: attrs(&[("class", "primary")]).into(),
            compile_inputs: attrs(&[("class", "primary")]).into(),
            min_zoom: 10,
            max_zoom: 14,
            sort_order: 1,
            routing: None,
        },
    ]);

    let network = builder.finish();
    let render_lines = network
        .render_lines_for_layer(SharedLineLayer::Roads)
        .collect::<Vec<_>>();
    assert_eq!(render_lines.len(), 2);
    assert!(
        render_lines
            .iter()
            .any(|line| line.source_lineage_ids == vec![1, 2] && line.geometry.len() >= 3)
    );
    assert!(
        render_lines
            .iter()
            .any(|line| line.source_lineage_ids == vec![3] && line.geometry.len() == 2)
    );
}

#[test]
fn one_way_render_strokes_merge_only_when_flow_is_continuous() {
    let mut continuous_left = road_input(
        1,
        vec![(50.0, 19.0), (50.0, 19.001)],
        vec![1, 2],
        attrs(&[("class", "primary"), ("oneway", "yes")]),
    );
    Arc::make_mut(continuous_left.routing.as_mut().unwrap()).directional_access =
        DirectionalAccessModel::Forward;
    let mut continuous_right = road_input(
        2,
        vec![(50.0, 19.001), (50.0, 19.002)],
        vec![2, 3],
        attrs(&[("class", "primary"), ("oneway", "yes")]),
    );
    Arc::make_mut(continuous_right.routing.as_mut().unwrap()).directional_access =
        DirectionalAccessModel::Forward;

    let mut continuous_builder = SharedLineNetworkBuilder::default();
    continuous_builder.extend_inputs([continuous_left, continuous_right]);
    let continuous = continuous_builder.finish();
    let continuous_lines = continuous
        .render_lines_for_layer(SharedLineLayer::Roads)
        .collect::<Vec<_>>();
    assert_eq!(continuous_lines.len(), 1);
    assert_eq!(
        continuous_lines[0].geometry,
        vec![(50.0, 19.0), (50.0, 19.001), (50.0, 19.002)]
    );

    let mut opposing_left = road_input(
        3,
        vec![(50.0, 19.0), (50.0, 19.001)],
        vec![1, 2],
        attrs(&[("class", "primary"), ("oneway", "yes")]),
    );
    Arc::make_mut(opposing_left.routing.as_mut().unwrap()).directional_access =
        DirectionalAccessModel::Forward;
    let mut opposing_right = road_input(
        4,
        vec![(50.0, 19.002), (50.0, 19.001)],
        vec![3, 2],
        attrs(&[("class", "primary"), ("oneway", "yes")]),
    );
    Arc::make_mut(opposing_right.routing.as_mut().unwrap()).directional_access =
        DirectionalAccessModel::Forward;

    let mut opposing_builder = SharedLineNetworkBuilder::default();
    opposing_builder.extend_inputs([opposing_left, opposing_right]);
    let opposing = opposing_builder.finish();
    assert_eq!(
        opposing
            .render_lines_for_layer(SharedLineLayer::Roads)
            .count(),
        2
    );
}

#[test]
fn road_render_strokes_preserve_corridor_width_boundaries() {
    let mut builder = SharedLineNetworkBuilder::default();
    builder.extend_inputs([
        road_input(
            1,
            vec![(50.0, 19.0), (50.0, 19.001)],
            vec![1, 2],
            attrs(&[("class", "primary"), ("width_m", "7.0")]),
        ),
        road_input(
            2,
            vec![(50.0, 19.001), (50.0, 19.002)],
            vec![2, 3],
            attrs(&[("class", "primary"), ("width_m", "10.5")]),
        ),
    ]);

    let network = builder.finish();
    let widths = network
        .render_lines_for_layer(SharedLineLayer::Roads)
        .filter_map(|line| attr_value(&line.attrs, "width_m"))
        .collect::<Vec<_>>();
    assert_eq!(widths, vec!["7.0", "10.5"]);
}

#[test]
fn four_way_junction_pairs_opposite_arms_instead_of_turns() {
    let mut builder = SharedLineNetworkBuilder::default();
    builder.extend_inputs([
        SharedLineInput {
            feature_id: 1,
            layer: SharedLineLayer::Roads,
            geometry: vec![(50.0, 18.999), (50.0, 19.0)],
            node_chain: vec![1, 10],
            source_lineage_ids: vec![1],
            attrs: attrs(&[("class", "primary")]).into(),
            compile_inputs: attrs(&[("class", "primary")]).into(),
            min_zoom: 10,
            max_zoom: 14,
            sort_order: 1,
            routing: None,
        },
        SharedLineInput {
            feature_id: 2,
            layer: SharedLineLayer::Roads,
            geometry: vec![(50.0, 19.0), (50.0, 19.001)],
            node_chain: vec![10, 2],
            source_lineage_ids: vec![2],
            attrs: attrs(&[("class", "primary")]).into(),
            compile_inputs: attrs(&[("class", "primary")]).into(),
            min_zoom: 10,
            max_zoom: 14,
            sort_order: 1,
            routing: None,
        },
        SharedLineInput {
            feature_id: 3,
            layer: SharedLineLayer::Roads,
            geometry: vec![(49.999, 19.0), (50.0, 19.0)],
            node_chain: vec![3, 10],
            source_lineage_ids: vec![3],
            attrs: attrs(&[("class", "primary")]).into(),
            compile_inputs: attrs(&[("class", "primary")]).into(),
            min_zoom: 10,
            max_zoom: 14,
            sort_order: 1,
            routing: None,
        },
        SharedLineInput {
            feature_id: 4,
            layer: SharedLineLayer::Roads,
            geometry: vec![(50.0, 19.0), (50.001, 19.0)],
            node_chain: vec![10, 4],
            source_lineage_ids: vec![4],
            attrs: attrs(&[("class", "primary")]).into(),
            compile_inputs: attrs(&[("class", "primary")]).into(),
            min_zoom: 10,
            max_zoom: 14,
            sort_order: 1,
            routing: None,
        },
    ]);

    let network = builder.finish();
    let render_lines = network
        .render_lines_for_layer(SharedLineLayer::Roads)
        .collect::<Vec<_>>();
    assert_eq!(render_lines.len(), 2);
    assert!(
        render_lines
            .iter()
            .any(|line| line.source_lineage_ids == vec![1, 2])
    );
    assert!(
        render_lines
            .iter()
            .any(|line| line.source_lineage_ids == vec![3, 4])
    );
}

#[test]
fn preserves_source_lineage_on_merged_render_lines() {
    let mut builder = SharedLineNetworkBuilder::default();
    builder.extend_inputs([
        SharedLineInput {
            feature_id: 11,
            layer: SharedLineLayer::Roads,
            geometry: vec![(50.0, 19.0), (50.0, 19.001)],
            node_chain: vec![1, 2],
            source_lineage_ids: vec![11],
            attrs: attrs(&[("class", "residential")]).into(),
            compile_inputs: attrs(&[("class", "residential")]).into(),
            min_zoom: 12,
            max_zoom: 14,
            sort_order: 1,
            routing: None,
        },
        SharedLineInput {
            feature_id: 12,
            layer: SharedLineLayer::Roads,
            geometry: vec![(50.0, 19.001), (50.0, 19.002)],
            node_chain: vec![2, 3],
            source_lineage_ids: vec![12],
            attrs: attrs(&[("class", "residential")]).into(),
            compile_inputs: attrs(&[("class", "residential")]).into(),
            min_zoom: 12,
            max_zoom: 14,
            sort_order: 1,
            routing: None,
        },
    ]);

    let network = builder.finish();
    let line = network
        .render_lines_for_layer(SharedLineLayer::Roads)
        .next()
        .expect("render line");
    assert_eq!(line.source_lineage_ids, vec![11, 12]);
}

#[test]
fn bumps_generic_service_stub_min_zoom() {
    let mut builder = SharedLineNetworkBuilder::default();
    builder.push_input(SharedLineInput {
        feature_id: 1,
        layer: SharedLineLayer::Roads,
        geometry: vec![(50.0, 19.0), (50.0, 19.0003)],
        node_chain: vec![1, 2],
        source_lineage_ids: vec![1],
        attrs: attrs(&[("class", "service"), ("subclass", "service")]).into(),
        compile_inputs: attrs(&[("class", "service"), ("subclass", "service")]).into(),
        min_zoom: 10,
        max_zoom: 14,
        sort_order: 1,
        routing: None,
    });

    let network = builder.finish();
    let line = network
        .render_lines_for_layer(SharedLineLayer::Roads)
        .next()
        .expect("render line");
    assert!(line.min_zoom >= 13);
}

#[test]
fn same_plane_snapped_nodes_connect_distinct_source_ids() {
    let mut builder = SharedLineNetworkBuilder::default();
    builder.extend_inputs([
        SharedLineInput {
            feature_id: 1,
            layer: SharedLineLayer::Roads,
            geometry: vec![(50.0, 19.0), (50.0, 19.001)],
            node_chain: vec![101, 102],
            source_lineage_ids: vec![1],
            attrs: attrs(&[("class", "residential")]).into(),
            compile_inputs: attrs(&[("class", "residential")]).into(),
            min_zoom: 11,
            max_zoom: 14,
            sort_order: 1,
            routing: None,
        },
        SharedLineInput {
            feature_id: 2,
            layer: SharedLineLayer::Roads,
            geometry: vec![(50.0, 19.001), (50.0, 19.002)],
            node_chain: vec![202, 203],
            source_lineage_ids: vec![2],
            attrs: attrs(&[("class", "residential")]).into(),
            compile_inputs: attrs(&[("class", "residential")]).into(),
            min_zoom: 11,
            max_zoom: 14,
            sort_order: 1,
            routing: None,
        },
    ]);

    let network = builder.finish();
    let render_lines = network
        .render_lines_for_layer(SharedLineLayer::Roads)
        .collect::<Vec<_>>();
    assert_eq!(render_lines.len(), 1);
    assert_eq!(render_lines[0].source_lineage_ids, vec![1, 2]);
}

#[test]
fn snapped_bridge_and_surface_lines_do_not_cross_connect() {
    let mut builder = SharedLineNetworkBuilder::default();
    builder.extend_inputs([
        SharedLineInput {
            feature_id: 1,
            layer: SharedLineLayer::Roads,
            geometry: vec![(50.0, 19.0), (50.0, 19.001), (50.0, 19.002)],
            node_chain: vec![],
            source_lineage_ids: vec![1],
            attrs: attrs(&[("class", "primary"), ("bridge", "yes")]).into(),
            compile_inputs: attrs(&[("class", "primary"), ("bridge", "yes")]).into(),
            min_zoom: 10,
            max_zoom: 14,
            sort_order: 1,
            routing: None,
        },
        SharedLineInput {
            feature_id: 2,
            layer: SharedLineLayer::Roads,
            geometry: vec![(49.999, 19.001), (50.0, 19.001), (50.001, 19.001)],
            node_chain: vec![],
            source_lineage_ids: vec![2],
            attrs: attrs(&[("class", "primary")]).into(),
            compile_inputs: attrs(&[("class", "primary")]).into(),
            min_zoom: 10,
            max_zoom: 14,
            sort_order: 1,
            routing: None,
        },
    ]);

    let network = builder.finish();
    assert_eq!(
        network
            .topology
            .edges_for_layer(SharedLineLayer::Roads)
            .count(),
        2
    );
}

#[test]
fn discard_render_products_preserves_topology_and_stats() {
    let mut builder = SharedLineNetworkBuilder::default();
    builder.push_input(SharedLineInput {
        feature_id: 1,
        layer: SharedLineLayer::Roads,
        geometry: vec![(50.0, 19.0), (50.0, 19.001)],
        node_chain: vec![1, 2],
        source_lineage_ids: vec![1],
        attrs: attrs(&[("class", "primary")]).into(),
        compile_inputs: attrs(&[("class", "primary")]).into(),
        min_zoom: 8,
        max_zoom: 14,
        sort_order: 1,
        routing: Some(Arc::new(RoadRoutingAttrs {
            road_class: "primary".to_string(),
            directional_access: DirectionalAccessModel::Both,
            access: None,
            surface: None,
            tracktype: None,
            smoothness: None,
            has_ford: false,
            has_tunnel: false,
            access_restricted: false,
        })),
    });

    let mut network = builder.finish();
    let topology_edges = network.topology.edges.len();
    let topology_nodes = network.topology.nodes.len();
    assert!(!network.render.render_lines.is_empty());

    network.discard_render_products();

    assert_eq!(network.render.render_lines.len(), 0);
    assert_eq!(network.topology.edges.len(), topology_edges);
    assert_eq!(network.topology.nodes.len(), topology_nodes);
    assert_eq!(network.stats.render_lines, 0);
}

#[test]
fn topology_only_finish_matches_full_finish_topology() {
    let road = |feature_id: i64, geometry: Vec<(f64, f64)>, node_chain: Vec<i64>| SharedLineInput {
        feature_id,
        layer: SharedLineLayer::Roads,
        geometry,
        node_chain,
        source_lineage_ids: vec![feature_id],
        attrs: attrs(&[("class", "primary")]).into(),
        compile_inputs: attrs(&[("class", "primary")]).into(),
        min_zoom: 8,
        max_zoom: 14,
        sort_order: 1,
        routing: None,
    };
    let inputs = || {
        vec![
            road(1, vec![(50.0, 19.0), (50.0, 19.002)], vec![1, 2]),
            // Crosses road 1 without a shared node: exercises repair.
            road(2, vec![(49.999, 19.001), (50.001, 19.001)], vec![3, 4]),
            road(3, vec![(50.0, 19.002), (50.001, 19.003)], vec![2, 5]),
        ]
    };
    let mut full = SharedLineNetworkBuilder::default();
    full.extend_inputs(inputs());
    let full = full.finish();
    let mut topology = SharedLineNetworkBuilder::default();
    topology.extend_inputs(inputs());
    let topology = topology.finish_topology_only();

    assert!(!full.render.render_lines.is_empty());
    assert!(topology.render.render_lines.is_empty());
    assert_eq!(topology.topology, full.topology);
    assert_eq!(
        topology.stats.topology_repair_repaired_crossings,
        full.stats.topology_repair_repaired_crossings
    );
    assert_eq!(topology.stats.topology_edges, full.stats.topology_edges);
}

#[test]
fn collinearity_is_checked_in_mercator_space() {
    let point = |lng: f64| PointKey {
        lat: to_fixed(50.0),
        lng: to_fixed(lng),
    };
    // The middle point is on the line between its neighbours, so it goes.
    assert_eq!(
        simplify_anchor_points(vec![point(19.0), point(19.001), point(19.002)]),
        vec![point(19.0), point(19.002)]
    );
}

#[test]
fn straightness_scoring_uses_projected_vectors() {
    let left = RenderSeed {
        feature_id: 1,
        layer: SharedLineLayer::Roads,
        points: vec![
            PointKey {
                lat: to_fixed(50.0),
                lng: to_fixed(19.0),
            },
            PointKey {
                lat: to_fixed(50.0),
                lng: to_fixed(19.001),
            },
        ],
        attrs: attrs(&[("class", "secondary")]).into(),
        merge_signature: attrs(&[("class", "secondary")]).into(),
        min_zoom: 8,
        max_zoom: 14,
        sort_order: 1,
        start_node: 1,
        end_node: 2,
        source_lineage_ids: vec![1],
        topology_edge_ids: vec![1],
        directional_access: DirectionalAccessModel::Both,
    };
    let right = RenderSeed {
        feature_id: 2,
        layer: SharedLineLayer::Roads,
        points: vec![
            PointKey {
                lat: to_fixed(50.0),
                lng: to_fixed(19.001),
            },
            PointKey {
                lat: to_fixed(50.0),
                lng: to_fixed(19.002),
            },
        ],
        attrs: attrs(&[("class", "secondary")]).into(),
        merge_signature: attrs(&[("class", "secondary")]).into(),
        min_zoom: 8,
        max_zoom: 14,
        sort_order: 1,
        start_node: 2,
        end_node: 3,
        source_lineage_ids: vec![2],
        topology_edge_ids: vec![2],
        directional_access: DirectionalAccessModel::Both,
    };

    let score = merge_straightness_score(&left, &right, 2).expect("straightness score");
    assert!(score < -0.99);
}
#[test]
fn short_standalone_roads_and_loops_survive_every_render_tier() {
    for class in [
        "motorway",
        "trunk",
        "primary",
        "secondary",
        "tertiary",
        "unclassified",
        "residential",
        "living_street",
        "pedestrian",
        "service",
        "track",
        "path",
        "footway",
        "cycleway",
        "bridleway",
        "steps",
        "busway",
        "bus_guideway",
        "other",
    ] {
        for closed in [false, true] {
            let mut builder = SharedLineNetworkBuilder::default();
            let mut points = vec![(50., 19.), (50., 19.00001)];
            let mut nodes = vec![1, 2];
            if closed {
                points.extend([(50.00001, 19.00001), (50., 19.)]);
                nodes.extend([3, 1]);
            }
            builder.push_input(road_input(1, points, nodes, attrs(&[("class", class)])));
            let network = builder.finish();
            let lines: Vec<_> = network
                .render_lines_for_layer(SharedLineLayer::Roads)
                .collect();
            assert_eq!(lines.len(), 1, "class={class}, closed={closed}");
            assert_eq!(
                lines[0].geometry.first() == lines[0].geometry.last(),
                closed
            );
        }
    }
}

#[test]
fn short_driveway_and_parking_aisle_keep_class_visibility() {
    for subclass in [
        "driveway",
        "parking_aisle",
        "private",
        "drive_through",
        "emergency_access",
    ] {
        let mut builder = SharedLineNetworkBuilder::default();
        builder.push_input(road_input(
            1,
            vec![(50., 19.), (50., 19.00001)],
            vec![1, 2],
            attrs(&[("class", "service"), ("subclass", subclass)]),
        ));
        let network = builder.finish();
        let line = network
            .render_lines_for_layer(SharedLineLayer::Roads)
            .next()
            .unwrap();
        assert_eq!(line.min_zoom, 14);
    }
}

#[test]
fn short_roads_do_not_merge_across_published_attribute_boundaries() {
    for (key, left, right) in [
        ("surface_class", "paved", "unpaved"),
        ("width_m", "4", "6"),
        ("structure", "ground", "bridge"),
        ("level", "0", "1"),
        ("oneway", "yes", "no"),
    ] {
        let mut builder = SharedLineNetworkBuilder::default();
        builder.extend_inputs([
            road_input(
                1,
                vec![(50., 19.), (50., 19.00004)],
                vec![1, 2],
                attrs(&[("class", "service"), (key, left)]),
            ),
            road_input(
                2,
                vec![(50., 19.00004), (50., 19.00008)],
                vec![2, 3],
                attrs(&[("class", "service"), (key, right)]),
            ),
        ]);
        let network = builder.finish();
        assert_eq!(
            network
                .render_lines_for_layer(SharedLineLayer::Roads)
                .count(),
            2,
            "{key}"
        );
    }
}

#[test]
fn permute_in_place_applies_the_order() {
    let mut items = vec!['a', 'b', 'c', 'd', 'e', 'f'];
    // Cycles (0 3 5), (1 4) and the fixed point 2.
    let order = [3u32, 4, 2, 5, 1, 0];
    let expected = order
        .iter()
        .map(|&from| items[from as usize])
        .collect::<Vec<_>>();
    permute_in_place(&mut items, &order);
    assert_eq!(items, expected);
}

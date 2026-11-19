//! Quality warnings found during enhancement: node clusters, edge quality,
//! rich-fact coverage, topology integrity, dead ends and shortcut candidates.

use super::*;

#[derive(Debug, Clone, Copy)]
pub(crate) enum ClusterWarningKind {
    NearDuplicateNodes,
    UnsnappedEndpoints,
}

/// What one cluster scan found: the cluster count and up to four examples.
pub(crate) struct ClusterWarnings {
    kind: ClusterWarningKind,
    cluster_count: usize,
    examples: Vec<RoadGraphQualityExample>,
}

impl ClusterWarnings {
    pub(crate) fn apply(self, report: &mut RoadGraphEnhancementReport) {
        match self.kind {
            ClusterWarningKind::NearDuplicateNodes => {
                report.near_duplicate_node_cluster_count = self.cluster_count;
            }
            ClusterWarningKind::UnsnappedEndpoints => {
                report.unsnapped_endpoint_cluster_count = self.cluster_count;
            }
        }
        for example in self.examples {
            push_quality_example(&mut report.warning_examples, example);
        }
    }
}

pub(crate) fn cluster_warnings(
    graph: &RoadGraph,
    radius_fixed: i32,
    kind: ClusterWarningKind,
) -> ClusterWarnings {
    let mut warnings = ClusterWarnings {
        kind,
        cluster_count: 0,
        examples: Vec::new(),
    };
    let mut candidates = Vec::new();
    for node_index in graph.graph.node_indices() {
        let node = &graph.graph[node_index];
        if matches!(kind, ClusterWarningKind::UnsnappedEndpoints)
            && (node.source_node_id.is_some() || node.junction_kind != RoadJunctionKind::DeadEnd)
        {
            continue;
        }
        candidates.push((node_index.index(), to_fixed(node.lat), to_fixed(node.lng)));
    }
    if candidates.len() < 2 {
        // The former code returned before recording a count, leaving it 0.
        return warnings;
    }

    let cell_size = radius_fixed.max(1);
    let cell_of = |lat_fixed: i32, lng_fixed: i32| {
        (
            lat_fixed.div_euclid(cell_size),
            lng_fixed.div_euclid(cell_size),
        )
    };
    // Candidates sorted by cell, so each cell's members are one contiguous
    // run in candidate order: what a map of per-cell vectors held, without a
    // vector per cell. The map only locates each run.
    let mut by_cell = candidates
        .iter()
        .enumerate()
        .map(|(candidate_index, &(_, lat_fixed, lng_fixed))| {
            (cell_of(lat_fixed, lng_fixed), candidate_index)
        })
        .collect::<Vec<_>>();
    by_cell.sort_unstable();
    let mut cell_runs = FxHashMap::<(i32, i32), (usize, usize)>::default();
    let mut run_start = 0;
    for run in by_cell.chunk_by(|left, right| left.0 == right.0) {
        cell_runs.insert(run[0].0, (run_start, run_start + run.len()));
        run_start += run.len();
    }

    let mut dsu = DisjointSet::new(candidates.len());
    for (candidate_index, &(_, lat_fixed, lng_fixed)) in candidates.iter().enumerate() {
        let cell = cell_of(lat_fixed, lng_fixed);
        for d_lat in -1..=1 {
            for d_lng in -1..=1 {
                let Some(&(start, end)) = cell_runs.get(&(cell.0 + d_lat, cell.1 + d_lng)) else {
                    continue;
                };
                for &(_, other_index) in &by_cell[start..end] {
                    if other_index <= candidate_index {
                        continue;
                    }
                    let (_, other_lat, other_lng) = candidates[other_index];
                    let d_lat = lat_fixed - other_lat;
                    let d_lng = lng_fixed - other_lng;
                    if d_lat == 0 && d_lng == 0 {
                        continue;
                    }
                    if d_lat.saturating_mul(d_lat) + d_lng.saturating_mul(d_lng)
                        <= radius_fixed.saturating_mul(radius_fixed)
                    {
                        dsu.union(candidate_index, other_index);
                    }
                }
            }
        }
    }

    // Cluster sizes and lowest members per root, read in root order: what a
    // map of member lists keyed by root yielded, without a list per node.
    let mut cluster_size = vec![0u32; candidates.len()];
    let mut cluster_first = vec![0usize; candidates.len()];
    for index in 0..candidates.len() {
        let root = dsu.find(index);
        if cluster_size[root] == 0 {
            cluster_first[root] = index;
        }
        cluster_size[root] += 1;
    }
    let clusters = (0..candidates.len())
        .filter(|&root| cluster_size[root] > 1)
        .map(|root| (cluster_first[root], cluster_size[root] as usize))
        .collect::<Vec<_>>();
    warnings.cluster_count = clusters.len();

    for (first_member, member_count) in clusters.into_iter().take(4) {
        let (node_graph_index, lat_fixed, lng_fixed) = candidates[first_member];
        let node = &graph.graph[NodeIndex::new(node_graph_index)];
        let (example_kind, label) = match kind {
            ClusterWarningKind::NearDuplicateNodes => (
                RoadGraphQualityExampleKind::NearDuplicateNodeCluster,
                "near-duplicate node cluster",
            ),
            ClusterWarningKind::UnsnappedEndpoints => (
                RoadGraphQualityExampleKind::UnsnappedEndpointCluster,
                "unsnapped endpoint cluster",
            ),
        };
        warnings.examples.push(RoadGraphQualityExample {
            kind: example_kind,
            message: format!("{} contains {} nodes", label, member_count),
            lat_fixed: Some(lat_fixed),
            lng_fixed: Some(lng_fixed),
            node_id: Some(node.id),
            component_id: node.component_id,
            ..Default::default()
        });
    }
    warnings
}

pub(crate) fn add_edge_quality_warnings(
    graph: &RoadGraph,
    report: &mut RoadGraphEnhancementReport,
    info_index_by_id: &SharedInfoIndex,
) {
    let mut missing_heading_count = 0usize;
    let mut missing_speed_count = 0usize;
    let mut short_edge_count = 0usize;
    for edge in graph.graph.edge_weights() {
        if edge.start_heading_deg.is_none() || edge.end_heading_deg.is_none() {
            missing_heading_count += 1;
        }
        if edge.inferred_speed_kph.is_none() {
            missing_speed_count += 1;
        }
        if edge.distance_m <= 0.0 || edge.distance_m < ENHANCEMENT_VERY_SHORT_EDGE_M {
            short_edge_count += 1;
            push_quality_example(
                &mut report.warning_examples,
                RoadGraphQualityExample {
                    kind: RoadGraphQualityExampleKind::AbnormalEdgeLength,
                    message: format!(
                        "directed edge {} has abnormal length {:.3} m",
                        edge.edge_id, edge.distance_m
                    ),
                    edge_id: Some(edge.edge_id),
                    component_id: edge.component_id,
                    source_lineage_ids: edge_lineage(graph, info_index_by_id, edge),
                    ..Default::default()
                },
            );
        }
        if edge.inferred_speed_kph.is_none()
            && !info_index_by_id.contains_key(&edge.shared_edge_info_id)
        {
            report.missing_shared_edge_info_count += 1;
            push_quality_example(
                &mut report.warning_examples,
                RoadGraphQualityExample {
                    kind: RoadGraphQualityExampleKind::MissingInferredSpeed,
                    message: format!(
                        "directed edge {} references missing shared edge metadata",
                        edge.edge_id
                    ),
                    edge_id: Some(edge.edge_id),
                    component_id: edge.component_id,
                    source_lineage_ids: edge_lineage(graph, info_index_by_id, edge),
                    ..Default::default()
                },
            );
        }
    }
    if missing_heading_count > 0 {
        report.missing_heading_edge_count = missing_heading_count;
        push_quality_example(
            &mut report.warning_examples,
            RoadGraphQualityExample {
                kind: RoadGraphQualityExampleKind::MissingHeading,
                message: format!(
                    "{} directed edges are missing headings",
                    missing_heading_count
                ),
                ..Default::default()
            },
        );
    }
    if missing_speed_count > 0 {
        report.missing_inferred_speed_edge_count = missing_speed_count;
        push_quality_example(
            &mut report.warning_examples,
            RoadGraphQualityExample {
                kind: RoadGraphQualityExampleKind::MissingInferredSpeed,
                message: format!(
                    "{} directed edges are missing inferred speeds",
                    missing_speed_count
                ),
                ..Default::default()
            },
        );
    }
    if short_edge_count > 0 {
        push_quality_example(
            &mut report.warning_examples,
            RoadGraphQualityExample {
                kind: RoadGraphQualityExampleKind::AbnormalEdgeLength,
                message: format!("{} directed edges are shorter than 1 m", short_edge_count),
                ..Default::default()
            },
        );
    }
}

pub(crate) fn add_rich_road_fact_coverage_warnings(
    graph: &RoadGraph,
    report: &mut RoadGraphEnhancementReport,
    info_index_by_id: &SharedInfoIndex,
) {
    let mut example_count = 0usize;
    for edge in graph.graph.edge_weights() {
        let Some(info) = shared_edge_info(graph, info_index_by_id, edge.shared_edge_info_id) else {
            continue;
        };
        if !is_major_road_for_rich_fact_coverage(info.attrs.road_type) {
            continue;
        }

        let missing_lane = info.attrs.lane_count.is_none()
            && info.attrs.lanes.is_none()
            && info.attrs.turn_lanes.is_none()
            && info.attrs.turn_lanes_forward.is_none()
            && info.attrs.turn_lanes_backward.is_none();
        let missing_sign_or_destination = info.attrs.name.is_none()
            && info.attrs.ref_name.is_none()
            && info.attrs.route_refs.is_empty()
            && info.attrs.destination.is_none()
            && info.attrs.destination_ref.is_none()
            && info.attrs.destination_street.is_none()
            && info.attrs.junction_ref.is_none();
        let missing_admin =
            info.attrs.admin_country_id.is_none() || info.attrs.admin_region_id.is_none();
        let missing_timezone = info.attrs.timezone_id.is_none();

        if missing_lane {
            report.major_edge_missing_lane_fact_count += 1;
        }
        if missing_sign_or_destination {
            report.major_edge_missing_sign_or_destination_fact_count += 1;
        }
        if missing_admin {
            report.major_edge_missing_admin_fact_count += 1;
        }
        if missing_timezone {
            report.major_edge_missing_timezone_fact_count += 1;
        }
        if example_count < 8
            && (missing_lane || missing_sign_or_destination || missing_admin || missing_timezone)
        {
            let mut missing = Vec::new();
            if missing_lane {
                missing.push("lane");
            }
            if missing_sign_or_destination {
                missing.push("sign/destination");
            }
            if missing_admin {
                missing.push("admin");
            }
            if missing_timezone {
                missing.push("timezone");
            }
            push_quality_example(
                &mut report.warning_examples,
                RoadGraphQualityExample {
                    kind: RoadGraphQualityExampleKind::MissingRichRoadFacts,
                    message: format!(
                        "major directed edge {} is missing rich road facts: {}",
                        edge.edge_id,
                        missing.join(", ")
                    ),
                    edge_id: Some(edge.edge_id),
                    component_id: edge.component_id,
                    source_lineage_ids: edge_lineage(graph, info_index_by_id, edge),
                    ..Default::default()
                },
            );
            example_count += 1;
        }
    }
}

fn is_major_road_for_rich_fact_coverage(road_type: RoadType) -> bool {
    matches!(
        road_type,
        RoadType::Motorway | RoadType::Primary | RoadType::Secondary | RoadType::Tertiary
    )
}

pub(crate) fn add_topology_integrity_warnings(
    graph: &RoadGraph,
    report: &mut RoadGraphEnhancementReport,
    info_index_by_id: &SharedInfoIndex,
    incoming_counts: &[u32],
    outgoing_counts: &[u32],
) {
    // Both checks compare edges that leave the same node, so they walk the
    // graph's own adjacency: the former per-edge `BTreeSet` / `BTreeMap`, and
    // then sorted vectors of every edge's key, dominated enhancement's peak
    // memory, yet only a handful of duplicates are ever reported.
    let mut largest_component_id = None::<u32>;
    let mut component_node_counts = BTreeMap::<u32, usize>::new();
    for node in graph.graph.node_weights() {
        if let Some(component_id) = node.component_id {
            *component_node_counts.entry(component_id).or_default() += 1;
        }
    }
    if let Some((component_id, _)) = component_node_counts
        .iter()
        .max_by_key(|(_, node_count)| *node_count)
    {
        largest_component_id = Some(*component_id);
    }

    let raw_edges = graph.graph.raw_edges();
    let has_edge = |source: usize, target: usize, id: u64, direction: TraversalDirection| {
        graph
            .graph
            .edges_connecting(NodeIndex::new(source), NodeIndex::new(target))
            .any(|edge_ref| {
                let edge = edge_ref.weight();
                edge.shared_edge_info_id == id && edge.traversal_direction == direction
            })
    };
    // Both directions of a segment check against the same geometry length, so
    // it is measured once per shared info.
    let geometry_distance_m = graph
        .shared_edge_infos
        .par_iter()
        .map(|info| calculate_path_distance(info.geometry) as f32)
        .collect::<Vec<_>>();

    let edge_geometry_distance_m = |edge: &RoadEdge, info: SharedInfoRef<'_>| {
        info_index_by_id
            .get(edge.shared_edge_info_id)
            .map(|index| geometry_distance_m[index])
            .unwrap_or_else(|| calculate_path_distance(info.geometry) as f32)
    };
    // The checks each directed edge fails, decided in parallel; the walk below
    // counts them and records examples in edge order, as before.
    const DIRECTIONAL_MISMATCH: u8 = 1 << 0;
    const MISSING_OPPOSING: u8 = 1 << 1;
    const MISSING_GEOMETRY: u8 = 1 << 2;
    const DISTANCE_MISMATCH: u8 = 1 << 3;
    const UNREACHABLE_MAJOR: u8 = 1 << 4;
    const UNCONNECTED_LINK: u8 = 1 << 5;
    let failed_checks = raw_edges
        .par_iter()
        .map(|raw_edge| {
            let edge = &raw_edge.weight;
            let Some(info) = shared_edge_info(graph, info_index_by_id, edge.shared_edge_info_id)
            else {
                return 0u8;
            };
            // SAR-only edges have no legal access by design (and no
            // component): they are not the legal network's defects.
            if edge.is_sar_only() {
                return 0u8;
            }
            let source = raw_edge.source().index();
            let target = raw_edge.target().index();
            let mut failed = 0u8;
            let expected_mask = match edge.traversal_direction {
                TraversalDirection::Forward => info.attrs.osm_forward_mode_mask,
                TraversalDirection::Backward => info.attrs.osm_reverse_mode_mask,
            };
            if expected_mask == 0
                || edge.mobility_mode_mask != mobility_mode_mask_from_osm(expected_mask)
            {
                failed |= DIRECTIONAL_MISMATCH;
            }
            if info.attrs.directional_access == DirectionalAccess::Both {
                let opposite_direction = match edge.traversal_direction {
                    TraversalDirection::Forward => TraversalDirection::Backward,
                    TraversalDirection::Backward => TraversalDirection::Forward,
                };
                if !has_edge(target, source, edge.shared_edge_info_id, opposite_direction) {
                    failed |= MISSING_OPPOSING;
                }
            }
            if info.geometry.len() < 2 {
                failed |= MISSING_GEOMETRY;
            } else {
                let geometry_distance_m = edge_geometry_distance_m(edge, info);
                let tolerance_m = 5.0f32.max(geometry_distance_m * 0.15);
                if (edge.distance_m - geometry_distance_m).abs() > tolerance_m {
                    failed |= DISTANCE_MISMATCH;
                }
            }
            if is_major_road_type(info.attrs.road_type)
                && largest_component_id.is_some()
                && edge.component_id != largest_component_id
            {
                failed |= UNREACHABLE_MAJOR;
            }
            let incident = |node: usize| {
                incoming_counts.get(node).copied().unwrap_or_default() as usize
                    + outgoing_counts.get(node).copied().unwrap_or_default() as usize
            };
            let source_incident = incident(source);
            let target_incident = incident(target);
            if info.attrs.is_link
                && (edge.component_id != largest_component_id
                    || source_incident <= 1
                    || target_incident <= 1)
            {
                failed |= UNCONNECTED_LINK;
            }
            failed
        })
        .collect::<Vec<u8>>();

    let mut missing_opposing_examples = 0usize;
    let mut directional_mismatch_examples = 0usize;
    let mut geometry_examples = 0usize;
    let mut distance_examples = 0usize;
    let mut major_examples = 0usize;
    let mut link_examples = 0usize;

    for (edge_ref, failed) in graph.graph.edge_references().zip(failed_checks) {
        if failed == 0 {
            continue;
        }
        let edge = edge_ref.weight();
        let Some(info) = shared_edge_info(graph, info_index_by_id, edge.shared_edge_info_id) else {
            continue;
        };
        if failed & DIRECTIONAL_MISMATCH != 0 {
            report.directional_access_mismatch_count += 1;
            if directional_mismatch_examples < 4 {
                push_quality_example(
                    &mut report.warning_examples,
                    RoadGraphQualityExample {
                        kind: RoadGraphQualityExampleKind::DirectionalAccessMismatch,
                        message: format!(
                            "directed edge {} traversal/access masks do not match shared edge access",
                            edge.edge_id
                        ),
                        edge_id: Some(edge.edge_id),
                        component_id: edge.component_id,
                        source_lineage_ids: edge_lineage(graph, info_index_by_id, edge),
                        ..Default::default()
                    },
                );
                directional_mismatch_examples += 1;
            }
        }

        if failed & MISSING_OPPOSING != 0 {
            report.missing_opposing_edge_count += 1;
            if missing_opposing_examples < 4 {
                push_quality_example(
                    &mut report.warning_examples,
                    RoadGraphQualityExample {
                        kind: RoadGraphQualityExampleKind::MissingOpposingEdge,
                        message: format!(
                            "directed edge {} has no opposing edge for bidirectional shared segment {}",
                            edge.edge_id, edge.shared_edge_info_id
                        ),
                        edge_id: Some(edge.edge_id),
                        component_id: edge.component_id,
                        source_lineage_ids: edge_lineage(graph, info_index_by_id, edge),
                        ..Default::default()
                    },
                );
                missing_opposing_examples += 1;
            }
        }

        if failed & MISSING_GEOMETRY != 0 {
            report.missing_geometry_count += 1;
            if geometry_examples < 4 {
                push_quality_example(
                    &mut report.warning_examples,
                    RoadGraphQualityExample {
                        kind: RoadGraphQualityExampleKind::MissingGeometry,
                        message: format!(
                            "directed edge {} references shared segment {} with fewer than two geometry points",
                            edge.edge_id, edge.shared_edge_info_id
                        ),
                        edge_id: Some(edge.edge_id),
                        component_id: edge.component_id,
                        source_lineage_ids: edge_lineage(graph, info_index_by_id, edge),
                        ..Default::default()
                    },
                );
                geometry_examples += 1;
            }
        } else if failed & DISTANCE_MISMATCH != 0 {
            let geometry_distance_m = edge_geometry_distance_m(edge, info);
            report.distance_geometry_mismatch_count += 1;
            if distance_examples < 4 {
                push_quality_example(
                    &mut report.warning_examples,
                    RoadGraphQualityExample {
                        kind: RoadGraphQualityExampleKind::DistanceGeometryMismatch,
                        message: format!(
                            "directed edge {} distance {:.1} m differs from geometry {:.1} m",
                            edge.edge_id, edge.distance_m, geometry_distance_m
                        ),
                        edge_id: Some(edge.edge_id),
                        component_id: edge.component_id,
                        source_lineage_ids: edge_lineage(graph, info_index_by_id, edge),
                        ..Default::default()
                    },
                );
                distance_examples += 1;
            }
        }

        if failed & UNREACHABLE_MAJOR != 0 {
            report.unreachable_major_edge_count += 1;
            if major_examples < 4 {
                push_quality_example(
                    &mut report.warning_examples,
                    RoadGraphQualityExample {
                        kind: RoadGraphQualityExampleKind::UnreachableMajorRoad,
                        message: format!(
                            "major directed edge {} is outside the largest routeable component",
                            edge.edge_id
                        ),
                        edge_id: Some(edge.edge_id),
                        component_id: edge.component_id,
                        source_lineage_ids: edge_lineage(graph, info_index_by_id, edge),
                        ..Default::default()
                    },
                );
                major_examples += 1;
            }
        }

        if failed & UNCONNECTED_LINK != 0 {
            report.unconnected_link_edge_count += 1;
            if link_examples < 4 {
                push_quality_example(
                    &mut report.warning_examples,
                    RoadGraphQualityExample {
                        kind: RoadGraphQualityExampleKind::UnconnectedLink,
                        message: format!(
                            "link directed edge {} appears disconnected or weakly attached",
                            edge.edge_id
                        ),
                        edge_id: Some(edge.edge_id),
                        component_id: edge.component_id,
                        source_lineage_ids: edge_lineage(graph, info_index_by_id, edge),
                        ..Default::default()
                    },
                );
                link_examples += 1;
            }
        }
    }

    // Legal edges with an info that share their source, target, way and
    // direction: groups in that key's order, each in edge order (the order the
    // former map appended examples in).
    type DuplicateKey = (usize, usize, i64, TraversalDirection);
    let duplicate_groups = (0..graph.graph.node_count())
        .into_par_iter()
        .filter_map(|source| {
            let mut keys = graph
                .graph
                .edges(NodeIndex::new(source))
                .filter(|edge_ref| !edge_ref.weight().is_sar_only())
                .filter_map(|edge_ref| {
                    let edge = edge_ref.weight();
                    let info = shared_edge_info(graph, info_index_by_id, edge.shared_edge_info_id)?;
                    let key: DuplicateKey = (
                        source,
                        edge_ref.target().index(),
                        info.osm_way_id,
                        edge.traversal_direction,
                    );
                    Some((key, edge_ref.id().index()))
                })
                .collect::<Vec<_>>();
            if keys.len() < 2 {
                return None;
            }
            keys.sort_unstable();
            let groups = keys
                .chunk_by(|left, right| left.0 == right.0)
                .filter(|group| group.len() > 1)
                .map(|group| (group[0].0, group.iter().map(|&(_, edge)| edge).collect()))
                .collect::<Vec<(DuplicateKey, Vec<usize>)>>();
            (!groups.is_empty()).then_some(groups)
        })
        .collect::<Vec<_>>();
    for (key, group) in duplicate_groups.iter().flatten() {
        report.duplicate_parallel_edge_count += group.len() - 1;
        for &index in group.iter().skip(1).take(4) {
            let edge = &raw_edges[index].weight;
            push_quality_example(
                &mut report.warning_examples,
                RoadGraphQualityExample {
                    kind: RoadGraphQualityExampleKind::DuplicateParallelEdge,
                    message: format!(
                        "directed edge {} duplicates OSM way {} between the same node pair",
                        edge.edge_id, key.2
                    ),
                    edge_id: Some(edge.edge_id),
                    component_id: edge.component_id,
                    source_lineage_ids: edge_lineage(graph, info_index_by_id, edge),
                    ..Default::default()
                },
            );
        }
    }
}

fn is_major_road_type(road_type: RoadType) -> bool {
    matches!(
        road_type,
        RoadType::Motorway | RoadType::Primary | RoadType::Secondary
    )
}

pub(crate) fn add_dead_end_quality_warnings(
    graph: &RoadGraph,
    report: &mut RoadGraphEnhancementReport,
    info_index_by_id: &SharedInfoIndex,
) {
    let mut examples_added = 0usize;
    for node_index in graph.graph.node_indices() {
        let node = &graph.graph[node_index];
        if node.junction_kind != RoadJunctionKind::DeadEnd {
            continue;
        }
        let mut has_non_service_edge = false;
        let mut source_lineage_ids = BTreeSet::new();
        for edge_ref in graph.graph.edges(node_index) {
            let edge = edge_ref.weight();
            source_lineage_ids.extend(edge_lineage(graph, info_index_by_id, edge));
            if shared_edge_info(graph, info_index_by_id, edge.shared_edge_info_id).is_some_and(
                |info| !info.attrs.is_service_like && info.attrs.road_type != RoadType::Path,
            ) {
                has_non_service_edge = true;
            }
        }
        for edge_ref in graph.graph.edges_directed(node_index, Direction::Incoming) {
            let edge = edge_ref.weight();
            source_lineage_ids.extend(edge_lineage(graph, info_index_by_id, edge));
            if shared_edge_info(graph, info_index_by_id, edge.shared_edge_info_id).is_some_and(
                |info| !info.attrs.is_service_like && info.attrs.road_type != RoadType::Path,
            ) {
                has_non_service_edge = true;
            }
        }
        if !has_non_service_edge {
            continue;
        }
        push_quality_example(
            &mut report.warning_examples,
            RoadGraphQualityExample {
                kind: RoadGraphQualityExampleKind::NonServiceDeadEnd,
                message: format!("dead-end node {} touches non-service road edges", node.id),
                lat_fixed: Some(to_fixed(node.lat)),
                lng_fixed: Some(to_fixed(node.lng)),
                node_id: Some(node.id),
                component_id: node.component_id,
                source_lineage_ids: source_lineage_ids.into_iter().collect(),
                ..Default::default()
            },
        );
        examples_added += 1;
        if examples_added >= 8 {
            break;
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ShortcutKey {
    road_type: RoadType,
    name: Option<Arc<str>>,
    ref_name: Option<Arc<str>>,
    access: Option<Arc<str>>,
}

fn shortcut_key(info: &SharedEdgeInfo) -> ShortcutKey {
    ShortcutKey {
        road_type: info.road_type,
        name: info.name.clone(),
        ref_name: info.ref_name.clone(),
        access: info.access.clone(),
    }
}

pub(crate) fn detect_shortcut_candidates(
    graph: &RoadGraph,
    info_index_by_id: &SharedInfoIndex,
) -> RoadGraphShortcutCandidateReport {
    let mut report = RoadGraphShortcutCandidateReport::default();
    // Edges already in a candidate, by edge index. Edge ids are unique, so this
    // is the former set of edge ids without a tree lookup per edge.
    let mut seen_edges = vec![false; graph.graph.edge_count()];
    for edge_ref in graph.graph.edge_references() {
        let edge = edge_ref.weight();
        if seen_edges[edge_ref.id().index()] {
            continue;
        }
        let Some(info) = shared_edge_info(graph, info_index_by_id, edge.shared_edge_info_id) else {
            continue;
        };
        let key = shortcut_key(info.attrs);
        let start_node = edge_ref.source();
        let mut end_node = edge_ref.target();
        let mut previous_node = start_node;
        let mut edge_ids = vec![edge.edge_id];
        let mut edge_indexes = vec![edge_ref.id()];
        let mut distance_m = edge.distance_m.max(0.0);
        let mut current_edge_index = edge_ref.id();

        loop {
            let node = &graph.graph[end_node];
            if node.junction_kind != RoadJunctionKind::Through {
                break;
            }
            let next = matching_shortcut_successor(
                graph,
                info_index_by_id,
                end_node,
                previous_node,
                current_edge_index,
                &key,
            );
            let Some((next_edge_index, next_edge_id, next_target, next_distance_m)) = next else {
                break;
            };
            if seen_edges[next_edge_index.index()] || edge_ids.contains(&next_edge_id) {
                break;
            }
            edge_ids.push(next_edge_id);
            edge_indexes.push(next_edge_index);
            distance_m += next_distance_m.max(0.0);
            previous_node = end_node;
            end_node = next_target;
            current_edge_index = next_edge_index;
        }

        if edge_ids.len() < 2 {
            continue;
        }

        for edge_index in &edge_indexes {
            seen_edges[edge_index.index()] = true;
        }
        report.candidate_count += 1;
        report.candidate_edge_count += edge_ids.len();
        if report.examples.len() < 8 {
            let start = &graph.graph[start_node];
            let end = &graph.graph[end_node];
            report.examples.push(RoadGraphShortcutCandidateExample {
                start_node_id: start.id,
                end_node_id: end.id,
                edge_count: edge_ids.len(),
                distance_millimeters: (distance_m * 1000.0).round().max(0.0) as u64,
                road_type: key.road_type,
                name: key.name.as_deref().map(str::to_string),
                ref_name: key.ref_name.as_deref().map(str::to_string),
                component_id: start.component_id,
            });
        }
    }
    report
}

fn matching_shortcut_successor(
    graph: &RoadGraph,
    info_index_by_id: &SharedInfoIndex,
    node_index: NodeIndex,
    previous_node: NodeIndex,
    current_edge_index: EdgeIndex,
    key: &ShortcutKey,
) -> Option<(EdgeIndex, u64, NodeIndex, f32)> {
    // The only matching edge, if exactly one matches.
    let mut candidate = None;
    for edge_ref in graph.graph.edges(node_index) {
        if edge_ref.id() == current_edge_index || edge_ref.target() == previous_node {
            continue;
        }
        let edge = edge_ref.weight();
        let Some(info) = shared_edge_info(graph, info_index_by_id, edge.shared_edge_info_id) else {
            continue;
        };
        // `shortcut_key(info) == *key`, without cloning the key.
        if info.attrs.road_type == key.road_type
            && info.attrs.name == key.name
            && info.attrs.ref_name == key.ref_name
            && info.attrs.access == key.access
        {
            if candidate.is_some() {
                return None;
            }
            candidate = Some((
                edge_ref.id(),
                edge.edge_id,
                edge_ref.target(),
                edge.distance_m,
            ));
        }
    }
    candidate
}

/// The source lineage of `edge`: its shared info's (none when the info is
/// missing).
fn edge_lineage(
    graph: &RoadGraph,
    info_index_by_id: &SharedInfoIndex,
    edge: &RoadEdge,
) -> Vec<i64> {
    shared_edge_info(graph, info_index_by_id, edge.shared_edge_info_id)
        .map(|info| info.source_lineage_ids.to_vec())
        .unwrap_or_default()
}

pub(crate) fn push_quality_example(
    examples: &mut Vec<RoadGraphQualityExample>,
    example: RoadGraphQualityExample,
) {
    if examples.len() < ENHANCEMENT_WARNING_EXAMPLE_LIMIT {
        examples.push(example);
    }
}

#[derive(Debug, Clone)]
struct DisjointSet {
    parent: Vec<usize>,
    rank: Vec<u8>,
}

impl DisjointSet {
    fn new(len: usize) -> Self {
        Self {
            parent: (0..len).collect(),
            rank: vec![0; len],
        }
    }

    fn find(&mut self, value: usize) -> usize {
        if self.parent[value] != value {
            self.parent[value] = self.find(self.parent[value]);
        }
        self.parent[value]
    }

    fn union(&mut self, a: usize, b: usize) {
        let mut a_root = self.find(a);
        let mut b_root = self.find(b);
        if a_root == b_root {
            return;
        }
        if self.rank[a_root] < self.rank[b_root] {
            std::mem::swap(&mut a_root, &mut b_root);
        }
        self.parent[b_root] = a_root;
        if self.rank[a_root] == self.rank[b_root] {
            self.rank[a_root] += 1;
        }
    }
}

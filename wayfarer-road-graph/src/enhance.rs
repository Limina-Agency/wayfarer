//! [`enhance_road_graph`]: components, junctions, opposing edges, hierarchy
//! promotion and the enhancement report.

use super::*;

const ENHANCEMENT_NEAR_DUPLICATE_RADIUS_FIXED: i32 = 25;
/// Edges whose derived values are computed (in parallel) before they are set.
const EDGE_UPDATE_CHUNK: usize = 1 << 20;
const ENHANCEMENT_UNSNAPPED_ENDPOINT_RADIUS_FIXED: i32 = 100;
const ENHANCEMENT_SMALL_COMPONENT_NODE_LIMIT: usize = 10;
pub(crate) const ENHANCEMENT_WARNING_EXAMPLE_LIMIT: usize = 32;
pub(crate) const ENHANCEMENT_VERY_SHORT_EDGE_M: f32 = 1.0;

/// Number of [`RoadType`] variants, so per-node tallies can be fixed arrays.
pub(crate) const ROAD_TYPE_COUNT: usize = 8;

/// Slot for `road_type` in a per-node tally.
///
/// An explicit match rather than `as usize`, so reordering the enum cannot
/// silently move counts between road types.
pub(crate) fn road_type_index(road_type: RoadType) -> usize {
    match road_type {
        RoadType::Motorway => 0,
        RoadType::Primary => 1,
        RoadType::Secondary => 2,
        RoadType::Tertiary => 3,
        RoadType::Residential => 4,
        RoadType::Track => 5,
        RoadType::Path => 6,
        RoadType::Other => 7,
    }
}

/// Road types ordered by their label.
///
/// These tallies used to be a `BTreeMap<String, usize>` per node, which emitted
/// alphabetically by label. Flattening in the same order keeps node metadata
/// byte-identical; `road_type_labels_are_sorted` pins it.
pub(crate) const ROAD_TYPES_BY_LABEL: [RoadType; ROAD_TYPE_COUNT] = [
    RoadType::Motorway,
    RoadType::Other,
    RoadType::Path,
    RoadType::Primary,
    RoadType::Residential,
    RoadType::Secondary,
    RoadType::Tertiary,
    RoadType::Track,
];

/// Flatten a per-node tally into the named counts a node carries.
///
/// Sized exactly: a collected `filter_map` starts at four slots, and with two
/// lists per node that spare capacity was ~170 MB of a region's graph.
fn named_counts_from_tally(tally: &[u32; ROAD_TYPE_COUNT]) -> Vec<RoadGraphNamedCount> {
    let mut counts = Vec::with_capacity(tally.iter().filter(|count| **count > 0).count());
    counts.extend(ROAD_TYPES_BY_LABEL.into_iter().filter_map(|road_type| {
        let count = tally[road_type_index(road_type)];
        (count > 0).then(|| RoadGraphNamedCount {
            name: road_type_label(road_type).to_string(),
            count: count as usize,
        })
    }));
    counts
}

/// Undirected neighbours in compressed sparse row form.
///
/// Replaces a `Vec<BTreeSet<usize>>` with one node per entry: for a region-sized
/// graph that was a million separately allocated B-tree nodes, read only to walk
/// components and to ask each node its degree. Two flat arrays do both, with the
/// dedup the set provided done once by sorting.
struct UndirectedAdjacency {
    offsets: Vec<u32>,
    neighbours: Vec<u32>,
}

impl UndirectedAdjacency {
    fn from_pairs(node_count: usize, pairs: &[(u32, u32)]) -> Self {
        let mut offsets = vec![0u32; node_count + 1];
        for (source, target) in pairs {
            offsets[*source as usize + 1] += 1;
            offsets[*target as usize + 1] += 1;
        }
        for index in 0..node_count {
            offsets[index + 1] += offsets[index];
        }
        let mut neighbours = vec![0u32; offsets[node_count] as usize];
        let mut cursor = offsets.clone();
        for (source, target) in pairs {
            neighbours[cursor[*source as usize] as usize] = *target;
            cursor[*source as usize] += 1;
            neighbours[cursor[*target as usize] as usize] = *source;
            cursor[*target as usize] += 1;
        }

        // Sort and dedup each node's run, compacting forward so `offsets` ends
        // up describing distinct neighbours — the set semantics the previous
        // `BTreeSet` gave, without a per-node allocation.
        let mut write = 0usize;
        let mut compacted = vec![0u32; node_count + 1];
        for index in 0..node_count {
            let start = offsets[index] as usize;
            let end = offsets[index + 1] as usize;
            let run = &mut neighbours[start..end];
            run.sort_unstable();
            let mut read = start;
            let run_start = write;
            while read < end {
                let value = neighbours[read];
                if read == start || neighbours[read - 1] != value {
                    neighbours[write] = value;
                    write += 1;
                }
                read += 1;
            }
            compacted[index] = run_start as u32;
            compacted[index + 1] = write as u32;
        }
        neighbours.truncate(write);
        neighbours.shrink_to_fit();
        Self {
            offsets: compacted,
            neighbours,
        }
    }

    fn neighbours(&self, node: usize) -> &[u32] {
        let start = self.offsets[node] as usize;
        let end = self.offsets[node + 1] as usize;
        &self.neighbours[start..end]
    }

    fn degree(&self, node: usize) -> usize {
        (self.offsets[node + 1] - self.offsets[node]) as usize
    }

    fn len(&self) -> usize {
        self.offsets.len().saturating_sub(1)
    }
}

/// What [`enhance_road_graph_with`] derives beyond what routing packages
/// need.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EnhanceOptions {
    /// Each node's incoming and outgoing edge counts by road type
    /// ([`RoadGraph::node_road_type_counts`]): two small vectors of named
    /// counts per node (~0.2 GiB on a region), which no package stores.
    pub node_road_type_counts: bool,
}

impl Default for EnhanceOptions {
    fn default() -> Self {
        Self {
            node_road_type_counts: true,
        }
    }
}

pub fn enhance_road_graph(graph: &mut RoadGraph) -> RoadGraphEnhancementReport {
    enhance_road_graph_with(graph, EnhanceOptions::default())
}

/// [`enhance_road_graph`] with `options`; the rest of the graph and the report
/// do not depend on them.
pub fn enhance_road_graph_with(
    graph: &mut RoadGraph,
    options: EnhanceOptions,
) -> RoadGraphEnhancementReport {
    let node_count = graph.node_count();
    let mut report = RoadGraphEnhancementReport::default();
    if node_count == 0 {
        return report;
    }

    let info_index_by_id = SharedInfoIndex::new(&graph.shared_edge_infos);

    // Endpoint pairs are collected flat and turned into a CSR adjacency after
    // the edge walk; the per-node tallies are fixed arrays indexed by road type
    // rather than a `BTreeMap<String, _>` per node.
    let mut edge_pairs = Vec::<(u32, u32)>::with_capacity(graph.graph.edge_count());
    let mut incoming_counts = vec![0u32; node_count];
    let mut outgoing_counts = vec![0u32; node_count];
    let mut incident_roundabout = vec![false; node_count];
    let mut incoming_roundabout_counts = vec![0u32; node_count];
    let mut outgoing_roundabout_counts = vec![0u32; node_count];
    // Only allocated when they are kept (64 bytes a node).
    let tally_len = if options.node_road_type_counts {
        node_count
    } else {
        0
    };
    let mut incoming_road_type_counts = vec![[0u32; ROAD_TYPE_COUNT]; tally_len];
    let mut outgoing_road_type_counts = vec![[0u32; ROAD_TYPE_COUNT]; tally_len];
    let mut highest_road_type = vec![None::<RoadType>; node_count];

    // SAR-only edges (no legal access) stay out of every node tally, the
    // adjacency and the components, so the legal network's junctions,
    // minor approaches, tiers and components are what they were without
    // them. Their own nodes (none of whose edges is legal) get no component.
    let mut sar_only_node = vec![false; node_count];
    let mut legal_node = vec![false; node_count];
    for edge_ref in graph.graph.edge_references() {
        let source = edge_ref.source().index();
        let target = edge_ref.target().index();
        if source >= node_count || target >= node_count {
            continue;
        }
        if edge_ref.weight().is_sar_only() {
            sar_only_node[source] = true;
            sar_only_node[target] = true;
            continue;
        }
        legal_node[source] = true;
        legal_node[target] = true;
        outgoing_counts[source] += 1;
        incoming_counts[target] += 1;
        edge_pairs.push((source as u32, target as u32));
        if let Some(info) = shared_edge_info(
            graph,
            &info_index_by_id,
            edge_ref.weight().shared_edge_info_id,
        ) {
            // `road_type_label` returns a `&'static str`; keying by it used to
            // allocate a `String` for every directed edge in the extract.
            if options.node_road_type_counts {
                let slot = road_type_index(info.attrs.road_type);
                outgoing_road_type_counts[source][slot] += 1;
                incoming_road_type_counts[target][slot] += 1;
            }
            if info.attrs.is_roundabout {
                incident_roundabout[source] = true;
                incident_roundabout[target] = true;
                outgoing_roundabout_counts[source] += 1;
                incoming_roundabout_counts[target] += 1;
            }
            update_highest_road_type(&mut highest_road_type[source], info.attrs.road_type);
            update_highest_road_type(&mut highest_road_type[target], info.attrs.road_type);
        }
    }

    let adjacency = UndirectedAdjacency::from_pairs(node_count, &edge_pairs);
    drop(edge_pairs);
    for (sar_only, legal) in sar_only_node.iter_mut().zip(&legal_node) {
        *sar_only &= !legal;
    }
    drop(legal_node);
    let (component_ids, component_sizes, component_first_nodes) =
        weak_component_ids(&adjacency, &sar_only_node);
    report.component_count = component_sizes.len();
    report.largest_component_node_count = component_sizes.iter().copied().max().unwrap_or(0);
    report.small_component_count = component_sizes
        .iter()
        .filter(|&&size| size < ENHANCEMENT_SMALL_COMPONENT_NODE_LIMIT)
        .count();
    report.component_size_histogram = component_size_histogram(&component_sizes);

    for (component_id, &size) in component_sizes.iter().enumerate() {
        if size < ENHANCEMENT_SMALL_COMPONENT_NODE_LIMIT && component_sizes.len() > 1 {
            // The traversal records each component's lowest node index, which is
            // exactly what scanning `component_ids` for the first match used to
            // find — but without re-scanning every node per small component.
            if let Some(node_index) = component_first_nodes.get(component_id).copied() {
                let node = &graph.graph[NodeIndex::new(node_index)];
                push_quality_example(
                    &mut report.warning_examples,
                    RoadGraphQualityExample {
                        kind: RoadGraphQualityExampleKind::SmallComponent,
                        message: format!("weak component {} has {} nodes", component_id, size),
                        lat_fixed: Some(to_fixed(node.lat)),
                        lng_fixed: Some(to_fixed(node.lng)),
                        node_id: Some(node.id),
                        component_id: Some(component_id as u32),
                        ..Default::default()
                    },
                );
            }
        }
    }

    for node_index in graph.graph.node_indices() {
        let index = node_index.index();
        let incoming = incoming_counts.get(index).copied().unwrap_or_default() as usize;
        let outgoing = outgoing_counts.get(index).copied().unwrap_or_default() as usize;
        let incident = incoming + outgoing;
        let undirected_degree = if index < adjacency.len() {
            adjacency.degree(index)
        } else {
            0
        };
        let junction_kind = if incident == 0 && sar_only_node[index] {
            RoadJunctionKind::Unknown
        } else if incident == 0 {
            report.isolated_node_count += 1;
            RoadJunctionKind::Unknown
        } else if incident_roundabout.get(index).copied().unwrap_or(false) {
            report.roundabout_node_count += 1;
            RoadJunctionKind::Roundabout
        } else if undirected_degree <= 1 {
            report.dead_end_node_count += 1;
            RoadJunctionKind::DeadEnd
        } else if undirected_degree == 2 {
            report.through_node_count += 1;
            RoadJunctionKind::Through
        } else if undirected_degree <= 4 {
            report.intersection_node_count += 1;
            RoadJunctionKind::Intersection
        } else {
            report.complex_junction_node_count += 1;
            RoadJunctionKind::Complex
        };

        let node = &mut graph.graph[node_index];
        node.component_id = component_ids.get(index).copied().flatten();
        node.incoming_edge_count = saturating_u16(incoming);
        node.outgoing_edge_count = saturating_u16(outgoing);
        node.incident_edge_count = saturating_u16(incident);
        node.junction_kind = junction_kind;
        node.highest_incident_road_type = highest_road_type.get(index).copied().flatten();
        node.roundabout_entry_count = saturating_u16(
            incoming_roundabout_counts
                .get(index)
                .copied()
                .unwrap_or_default() as usize,
        );
        node.roundabout_exit_count = saturating_u16(
            outgoing_roundabout_counts
                .get(index)
                .copied()
                .unwrap_or_default() as usize,
        );
        node.is_complex_junction = junction_kind == RoadJunctionKind::Complex;
    }

    graph.node_road_type_counts = if options.node_road_type_counts {
        incoming_road_type_counts
            .iter()
            .zip(&outgoing_road_type_counts)
            .map(|(incoming, outgoing)| NodeRoadTypeCounts {
                incoming: named_counts_from_tally(incoming),
                outgoing: named_counts_from_tally(outgoing),
            })
            .collect()
    } else {
        Vec::new()
    };
    drop(incoming_road_type_counts);
    drop(outgoing_road_type_counts);

    // By shared-info position (an id's last position, as `info_index_by_id`
    // answers), so the infos of one id all read the tier its last edge set.
    let mut shared_hierarchy_updates =
        vec![None::<RoadHierarchyTier>; graph.shared_edge_infos.len()];
    // Edges per tier, tallied here and named once below.
    let mut tier_edge_counts = [0usize; 4];
    let edge_count = graph.graph.edge_count();
    for chunk_start in (0..edge_count).step_by(EDGE_UPDATE_CHUNK) {
        let chunk_end = (chunk_start + EDGE_UPDATE_CHUNK).min(edge_count);
        // Per-edge derivations read only the graph, so they run in parallel;
        // the results come back in edge order and are applied sequentially
        // below, a chunk at a time, so they are held for one chunk only (a
        // record of some 40 bytes per edge, over 2 GB on Germany at once).
        let edge_updates = graph.graph.raw_edges()[chunk_start..chunk_end]
            .par_iter()
            .enumerate()
            .map(|(offset, edge_ref)| {
                let edge_index = EdgeIndex::new(chunk_start + offset);
                let edge = &edge_ref.weight;
                let component_id = component_ids
                    .get(edge_ref.source().index())
                    .copied()
                    .flatten();
                let info = shared_edge_info(graph, &info_index_by_id, edge.shared_edge_info_id);
                let source_index = edge_ref.source().index();
                let target_index = edge_ref.target().index();
                let source_degree = if source_index < adjacency.len() {
                    adjacency.degree(source_index)
                } else {
                    0
                };
                let target_degree = if target_index < adjacency.len() {
                    adjacency.degree(target_index)
                } else {
                    0
                };
                let hierarchy_tier = info.map(|info| {
                    promoted_hierarchy_tier(
                        info.attrs
                            .hierarchy_tier
                            .unwrap_or_else(|| base_hierarchy_tier(info.attrs.road_type)),
                        source_degree,
                        target_degree,
                    )
                });
                let (start_heading, end_heading) = info
                    .and_then(|info| edge_headings_deg(info.geometry, edge.traversal_direction))
                    .unwrap_or((None, None));
                let inferred_speed = info.map(|info| infer_speed_kph(info.attrs));
                let minor_approach = info.is_some_and(|info| {
                    end_is_minor_approach(
                        info.attrs,
                        target_degree,
                        incident_roundabout
                            .get(target_index)
                            .copied()
                            .unwrap_or(false),
                        highest_road_type.get(target_index).copied().flatten(),
                    )
                });
                let travel_time = inferred_speed.and_then(|speed| {
                    let speed_mps = speed as f32 * 1000.0 / 3600.0;
                    (speed_mps > 0.0).then_some(edge.distance_m / speed_mps)
                });
                (
                    edge_index,
                    component_id,
                    start_heading,
                    end_heading,
                    inferred_speed,
                    travel_time,
                    hierarchy_tier,
                    minor_approach,
                )
            })
            .collect::<Vec<_>>();

        for (
            edge_index,
            component_id,
            start_heading,
            end_heading,
            inferred_speed,
            travel_time,
            hierarchy_tier,
            minor_approach,
        ) in edge_updates
        {
            if minor_approach {
                report.minor_approach_edge_count += 1;
            }
            if start_heading.is_some() && end_heading.is_some() {
                report.edges_with_heading_count += 1;
            }
            if inferred_speed.is_some() {
                report.edges_with_inferred_speed_count += 1;
            }
            if let Some(edge) = graph.graph.edge_weight_mut(edge_index) {
                edge.component_id = component_id;
                edge.start_heading_deg = start_heading;
                edge.end_heading_deg = end_heading;
                edge.inferred_speed_kph = inferred_speed;
                edge.estimated_travel_time_s = travel_time;
                edge.hierarchy_tier = hierarchy_tier;
                edge.end_minor_approach = minor_approach;
                if let Some(tier) = hierarchy_tier {
                    report.hierarchy_prepared_edge_count += 1;
                    tier_edge_counts[tier as usize] += 1;
                    if let Some(info_index) = info_index_by_id.get(edge.shared_edge_info_id) {
                        shared_hierarchy_updates[info_index] = Some(tier);
                    }
                } else {
                    report.missing_hierarchy_tier_edge_count += 1;
                }
            }
        }
    }
    // What `increment_named_count` per edge built: the tiers seen, by name.
    for tier in [
        RoadHierarchyTier::Local,
        RoadHierarchyTier::Collector,
        RoadHierarchyTier::Arterial,
        RoadHierarchyTier::Highway,
    ] {
        let count = tier_edge_counts[tier as usize];
        if count > 0 {
            report
                .hierarchy_tier_distribution
                .push(RoadGraphNamedCount {
                    name: hierarchy_tier_label(tier).to_string(),
                    count,
                });
        }
    }
    report
        .hierarchy_tier_distribution
        .sort_by(|a, b| a.name.cmp(&b.name));

    // The update depends only on an info's class and tier, so it runs once per
    // distinct pair. The pairs are numbered by first use, then the finished
    // attributes go to a new table sized for them, interned in that order, so
    // it comes out compact (numbered by first use) and the old table is not
    // grown past a doubling before it is dropped.
    let mut pair_slots = FxHashMap::<(u32, Option<u8>), u32>::default();
    let mut pairs = Vec::<(u32, Option<RoadHierarchyTier>)>::new();
    let mut classes_of = Vec::with_capacity(graph.shared_edge_infos.len());
    for info in graph.shared_edge_infos.iter() {
        let tier_update = info_index_by_id
            .get(info.shared_edge_info_id)
            .and_then(|info_index| shared_hierarchy_updates[info_index]);
        let key = (info.class, tier_update.map(|tier| tier as u8));
        let slot = *pair_slots.entry(key).or_insert_with(|| {
            pairs.push((info.class, tier_update));
            pairs.len() as u32 - 1
        });
        classes_of.push(slot);
    }
    drop(pair_slots);
    drop(shared_hierarchy_updates);
    let mut finished = SharedInfoClasses::with_capacity(pairs.len());
    let class_of_slot = pairs
        .into_iter()
        .map(|(class, tier_update)| {
            let mut attrs = graph.shared_edge_infos.classes().get(class).clone();
            finish_shared_info_attrs(&mut attrs, tier_update);
            finished.intern(attrs)
        })
        .collect::<Vec<u32>>();
    for class in &mut classes_of {
        *class = class_of_slot[*class as usize];
    }
    graph
        .shared_edge_infos
        .replace_classes(finished, classes_of);

    let opposing = opposing_edge_indices(graph);
    let opposing_edge_ids = opposing
        .iter()
        .map(|&index| {
            (index != u32::MAX).then(|| graph.graph.raw_edges()[index as usize].weight.edge_id)
        })
        .collect::<Vec<_>>();
    drop(opposing);
    for (edge, opposing_edge_id) in graph.graph.edge_weights_mut().zip(opposing_edge_ids) {
        edge.opposing_edge_id = opposing_edge_id;
    }

    // The two cluster scans only read the graph, so they run concurrently and
    // are applied in their former order.
    let (near_duplicates, unsnapped_endpoints) = rayon::join(
        || {
            cluster_warnings(
                graph,
                ENHANCEMENT_NEAR_DUPLICATE_RADIUS_FIXED,
                ClusterWarningKind::NearDuplicateNodes,
            )
        },
        || {
            cluster_warnings(
                graph,
                ENHANCEMENT_UNSNAPPED_ENDPOINT_RADIUS_FIXED,
                ClusterWarningKind::UnsnappedEndpoints,
            )
        },
    );
    near_duplicates.apply(&mut report);
    unsnapped_endpoints.apply(&mut report);
    add_topology_integrity_warnings(
        graph,
        &mut report,
        &info_index_by_id,
        &incoming_counts,
        &outgoing_counts,
    );
    add_edge_quality_warnings(graph, &mut report, &info_index_by_id);
    add_rich_road_fact_coverage_warnings(graph, &mut report, &info_index_by_id);
    add_dead_end_quality_warnings(graph, &mut report, &info_index_by_id);
    // Shared infos of ways closed to every mode (kept for SAR) aside.
    report.one_way_asymmetry_count = graph
        .shared_edge_infos
        .iter()
        .filter(|info| {
            info.attrs.directional_access != DirectionalAccess::Both
                && info.attrs.osm_forward_mode_mask | info.attrs.osm_reverse_mode_mask != 0
        })
        .count();
    report.shortcut_candidate_report = detect_shortcut_candidates(graph, &info_index_by_id);

    report
}

/// Each directed edge's opposing edge, in edge order, as its index in the
/// graph's edges (`u32::MAX` for none): the edge over the same shared info and
/// segment in the other direction. A repeated key resolves to its last edge,
/// as a map collected in edge order does.
///
/// The edges are found through their indices sorted by key (4 bytes an
/// edge) rather than a map from key to edge (~40 bytes an edge).
pub fn opposing_edge_indices(graph: &RoadGraph) -> Vec<u32> {
    let edges = graph.graph.raw_edges();
    let key = |edge: &RoadEdge, direction: TraversalDirection| {
        (edge.shared_edge_info_id, edge.segment_index, direction)
    };
    let edge_key = |index: u32| {
        let edge = &edges[index as usize].weight;
        key(edge, edge.traversal_direction)
    };
    let mut by_key =
        (0..u32::try_from(edges.len()).expect("fewer than 2^32 edges")).collect::<Vec<u32>>();
    // Ties stay in edge order, so a key's last position is its last edge.
    by_key.par_sort_unstable_by_key(|&index| (edge_key(index), index));
    edges
        .par_iter()
        .map(|raw_edge| {
            let edge = &raw_edge.weight;
            let wanted = key(edge, opposite_traversal_direction(edge.traversal_direction));
            let end = by_key.partition_point(|&index| edge_key(index) <= wanted);
            match by_key[..end].last() {
                Some(&last) if edge_key(last) == wanted => last,
                _ => u32::MAX,
            }
        })
        .collect()
}

fn opposite_traversal_direction(direction: TraversalDirection) -> TraversalDirection {
    match direction {
        TraversalDirection::Forward => TraversalDirection::Backward,
        TraversalDirection::Backward => TraversalDirection::Forward,
    }
}

/// Weak components, their sizes, and the lowest node index in each.
///
/// The traversal visits start nodes in ascending order, so `start` is already
/// the lowest-indexed node of the component it opens — the same node a scan of
/// `component_ids` for the first match would return, obtained for free.
/// Weak components of `adjacency`; the `skip` nodes (SAR-only: no legal
/// edge, so no neighbour here) get none.
fn weak_component_ids(
    adjacency: &UndirectedAdjacency,
    skip: &[bool],
) -> (Vec<Option<u32>>, Vec<usize>, Vec<usize>) {
    let node_count = adjacency.len();
    let mut component_ids = vec![None; node_count];
    let mut component_sizes = Vec::new();
    let mut component_first_nodes = Vec::new();
    let mut queue = VecDeque::new();
    for start in 0..node_count {
        if component_ids[start].is_some() || skip.get(start).copied().unwrap_or(false) {
            continue;
        }
        let component_id = component_sizes.len() as u32;
        let mut size = 0usize;
        queue.clear();
        queue.push_back(start);
        component_ids[start] = Some(component_id);
        while let Some(node) = queue.pop_front() {
            size += 1;
            for &next in adjacency.neighbours(node) {
                let next = next as usize;
                if component_ids[next].is_none() {
                    component_ids[next] = Some(component_id);
                    queue.push_back(next);
                }
            }
        }
        component_sizes.push(size);
        component_first_nodes.push(start);
    }
    (component_ids, component_sizes, component_first_nodes)
}

/// Whether an edge of `info` yields where it ends, at a node of
/// `target_degree` neighbours: it enters a roundabout, or a road of a higher
/// class meets it there. A link merging onto a motorway or trunk does not
/// yield (it has its acceleration lane).
/// An info's hierarchy tier (`tier_update`, else its own, else its road
/// type's), speeds and lane connectivity, as enhancement leaves them.
fn finish_shared_info_attrs(info: &mut SharedEdgeInfo, tier_update: Option<RoadHierarchyTier>) {
    info.hierarchy_tier = tier_update
        .or(info.hierarchy_tier)
        .or_else(|| Some(base_hierarchy_tier(info.road_type)));
    if info.speed_source.is_none() {
        info.speed_source = Some(speed_source_for_values(info.maxspeed_kph));
    }
    if info.free_flow_speed_kph.is_none() {
        let speed = infer_speed_kph(info);
        info.free_flow_speed_kph = Some(speed as f32);
        info.constrained_speed_kph = Some(constrained_speed_kph(speed));
    } else if info.constrained_speed_kph.is_none() {
        info.constrained_speed_kph = info
            .free_flow_speed_kph
            .map(|speed| (speed * 0.85).max(3.0));
    }
    info.lane_connectivity_available = info.lane_connectivity_available
        || info.turn_lanes.is_some()
        || info.turn_lanes_forward.is_some()
        || info.turn_lanes_backward.is_some();
}

fn end_is_minor_approach(
    info: &SharedEdgeInfo,
    target_degree: usize,
    target_roundabout: bool,
    target_highest: Option<RoadType>,
) -> bool {
    if target_degree < 3 || info.is_roundabout {
        return false;
    }
    target_roundabout
        || target_highest.is_some_and(|highest| {
            road_type_rank(highest) > road_type_rank(info.road_type)
                && !(info.is_link && highest == RoadType::Motorway)
        })
}

fn update_highest_road_type(slot: &mut Option<RoadType>, road_type: RoadType) {
    if slot
        .map(|current| road_type_rank(road_type) > road_type_rank(current))
        .unwrap_or(true)
    {
        *slot = Some(road_type);
    }
}

fn road_type_rank(road_type: RoadType) -> u8 {
    match road_type {
        RoadType::Path => 1,
        RoadType::Track => 2,
        RoadType::Other => 3,
        RoadType::Residential => 4,
        RoadType::Tertiary => 5,
        RoadType::Secondary => 6,
        RoadType::Primary => 7,
        RoadType::Motorway => 8,
    }
}

pub(crate) fn road_type_label(road_type: RoadType) -> &'static str {
    match road_type {
        RoadType::Motorway => "motorway",
        RoadType::Primary => "primary",
        RoadType::Secondary => "secondary",
        RoadType::Tertiary => "tertiary",
        RoadType::Residential => "residential",
        RoadType::Track => "track",
        RoadType::Path => "path",
        RoadType::Other => "other",
    }
}

pub(crate) fn component_size_histogram(
    component_sizes: &[usize],
) -> Vec<RoadGraphComponentSizeBucket> {
    let ranges = [
        ("1", 1usize, Some(1usize)),
        ("2-4", 2, Some(4)),
        ("5-9", 5, Some(9)),
        ("10-49", 10, Some(49)),
        ("50-199", 50, Some(199)),
        ("200-999", 200, Some(999)),
        ("1000+", 1000, None),
    ];
    ranges
        .into_iter()
        .filter_map(|(label, min_node_count, max_node_count)| {
            let component_count = component_sizes
                .iter()
                .filter(|&&size| {
                    size >= min_node_count
                        && max_node_count
                            .map(|max_node_count| size <= max_node_count)
                            .unwrap_or(true)
                })
                .count();
            (component_count > 0).then_some(RoadGraphComponentSizeBucket {
                label: label.to_string(),
                min_node_count,
                max_node_count,
                component_count,
            })
        })
        .collect()
}

fn saturating_u16(value: usize) -> u16 {
    value.min(u16::MAX as usize) as u16
}

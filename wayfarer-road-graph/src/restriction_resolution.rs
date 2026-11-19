use crate::{
    OsmMode, RelationMemberRefType, ResolvedEdgeRestriction, RoadGraph, RoadGraphNamedCount,
    RoadGraphRestrictionReport, RoadGraphUnresolvedRestrictionExample,
    RoadGraphUnresolvedRestrictionReason, TraversalDirection, mobility_mode_mask_from_osm,
};
use petgraph::Direction;
use petgraph::graph::NodeIndex;
use petgraph::visit::EdgeRef;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque};

#[derive(Debug, Clone)]
struct EdgeResolutionEntry {
    edge_id: u64,
    source_index: usize,
    target_index: usize,
    source_node_id: Option<i64>,
    target_node_id: Option<i64>,
    way_ids: BTreeSet<i64>,
}

/// Resolve OSM turn restrictions against the graph's directed edges and return
/// both the executable restrictions and the quality report for unresolved data.
pub fn resolve_turn_restrictions_with_report(
    graph: &RoadGraph,
) -> (Vec<ResolvedEdgeRestriction>, RoadGraphRestrictionReport) {
    const MAX_UNRESOLVED_RESTRICTION_EXAMPLES: usize = 16;

    let info_by_id = graph
        .shared_edge_infos
        .iter()
        .map(|info| (info.shared_edge_info_id, info))
        .collect::<HashMap<_, _>>();
    // Restrictions are only ever resolved against edges of the ways they name,
    // so only those edges are indexed. Indexing every directed edge (with a
    // way-id set each) cost ~0.65 GiB at this phase's peak on a region, for a
    // few thousand restrictions.
    let referenced_way_ids = graph
        .turn_restrictions
        .iter()
        .flat_map(|restriction| {
            [restriction.from_way_id, restriction.to_way_id]
                .into_iter()
                .chain(via_way_ids(restriction).iter().copied())
        })
        .collect::<HashSet<_>>();
    let mut entries = Vec::new();
    for edge_ref in graph.graph.edge_references() {
        let edge = edge_ref.weight();
        // SAR-only edges (ways closed to every mode) resolve nothing: the
        // restrictions resolve as they did before those ways were kept.
        if edge.is_sar_only() {
            continue;
        }
        // The edge's ways: its shared info's way and source lineage.
        let Some(info) = info_by_id.get(&edge.shared_edge_info_id) else {
            continue;
        };
        if !info
            .source_lineage_ids
            .iter()
            .copied()
            .chain([info.osm_way_id])
            .any(|way_id| referenced_way_ids.contains(&way_id))
        {
            continue;
        }
        let mut way_ids = info
            .source_lineage_ids
            .iter()
            .copied()
            .collect::<BTreeSet<_>>();
        way_ids.insert(info.osm_way_id);
        entries.push(EdgeResolutionEntry {
            edge_id: edge.edge_id,
            source_index: edge_ref.source().index(),
            target_index: edge_ref.target().index(),
            source_node_id: graph.graph[edge_ref.source()].source_node_id,
            target_node_id: graph.graph[edge_ref.target()].source_node_id,
            way_ids,
        });
    }

    let mut entries_by_way: HashMap<i64, Vec<usize>> = HashMap::new();
    for (index, entry) in entries.iter().enumerate() {
        for way_id in &entry.way_ids {
            entries_by_way.entry(*way_id).or_default().push(index);
        }
    }
    let entries_for_way = |way_id: i64| -> &[usize] {
        entries_by_way
            .get(&way_id)
            .map(Vec::as_slice)
            .unwrap_or(&[])
    };

    let mut resolved = Vec::new();
    let mut report = RoadGraphRestrictionReport {
        raw_restriction_count: graph.turn_restrictions.len(),
        ..Default::default()
    };
    for restriction in &graph.turn_restrictions {
        increment_restriction_kind_count(
            &mut report.restriction_kind_counts,
            &restriction.restriction,
        );
        let osm_mode_mask =
            restriction_osm_mode_mask(&restriction.modes, &restriction.except_modes);
        // The mobility modes have no bicycle, bus or truck profile: bicycles
        // share the on-foot modes, which no vehicle's restriction binds, and
        // the wheeled modes are cars. So only the motorcar and foot bits reach
        // the mobility modes.
        let mobility_mode_mask =
            mobility_mode_mask_from_osm(osm_mode_mask & (OsmMode::MOTOR_VEHICLE | OsmMode::FOOT));
        // A restriction that binds none of them (`restriction:hgv`, `:bus`,
        // `:bicycle`, or a plain one with `except=motorcar`) binds no profile,
        // so it is not resolved: a row with no profile bits would read as one
        // for every profile. Its modes stay on the raw restriction.
        if mobility_mode_mask == 0 {
            report.mode_not_routed_restriction_count += 1;
            continue;
        }
        let only = restriction.restriction.starts_with("only_");
        // An `only_*` restriction resolves to bans on the turns it rules out,
        // which may be none at all; it is resolved once its turn is found.
        let mut only_turn_found = false;
        let resolved_before = resolved.len();
        let mut unresolved_reason = None;
        match restriction.via_member_type {
            RelationMemberRefType::Node => {
                let has_from_way = !entries_for_way(restriction.from_way_id).is_empty();
                let has_to_way = !entries_for_way(restriction.to_way_id).is_empty();
                let from_edges = entries_for_way(restriction.from_way_id)
                    .iter()
                    .map(|&index| &entries[index])
                    .filter(|entry| entry.target_node_id == Some(restriction.via_member_id))
                    .collect::<Vec<_>>();
                let to_edges = entries_for_way(restriction.to_way_id)
                    .iter()
                    .map(|&index| &entries[index])
                    .filter(|entry| entry.source_node_id == Some(restriction.via_member_id))
                    .collect::<Vec<_>>();

                if !has_from_way {
                    report.unresolved_missing_from_edge_count += 1;
                    unresolved_reason = Some(RoadGraphUnresolvedRestrictionReason::MissingFromEdge);
                } else if !has_to_way {
                    report.unresolved_missing_to_edge_count += 1;
                    unresolved_reason = Some(RoadGraphUnresolvedRestrictionReason::MissingToEdge);
                } else if from_edges.is_empty() || to_edges.is_empty() {
                    report.unresolved_missing_via_node_or_path_count += 1;
                    unresolved_reason =
                        Some(RoadGraphUnresolvedRestrictionReason::MissingViaNodeOrPath);
                } else if only {
                    for from_edge in from_edges {
                        let allowed = to_edges
                            .iter()
                            .filter(|to_edge| to_edge.source_index == from_edge.target_index)
                            .map(|to_edge| to_edge.edge_id)
                            .collect::<HashSet<_>>();
                        if allowed.is_empty() {
                            continue;
                        }
                        only_turn_found = true;
                        for to_edge_id in
                            other_out_edges(graph, from_edge.target_index, &allowed, osm_mode_mask)
                        {
                            resolved.push(ResolvedEdgeRestriction::Simple {
                                relation_id: restriction.relation_id,
                                restriction: restriction.restriction.clone(),
                                from_edge_id: from_edge.edge_id,
                                to_edge_id,
                                via_node_id: restriction.via_member_id,
                                osm_mode_mask,
                                mobility_mode_mask,
                            });
                        }
                    }
                    if !only_turn_found {
                        report.unresolved_missing_via_node_or_path_count += 1;
                        unresolved_reason =
                            Some(RoadGraphUnresolvedRestrictionReason::MissingViaNodeOrPath);
                    }
                } else {
                    for from_edge in from_edges {
                        for to_edge in &to_edges {
                            resolved.push(ResolvedEdgeRestriction::Simple {
                                relation_id: restriction.relation_id,
                                restriction: restriction.restriction.clone(),
                                from_edge_id: from_edge.edge_id,
                                to_edge_id: to_edge.edge_id,
                                via_node_id: restriction.via_member_id,
                                osm_mode_mask,
                                mobility_mode_mask,
                            });
                        }
                    }
                }
            }
            RelationMemberRefType::Way => {
                let from_edges = entries_for_way(restriction.from_way_id)
                    .iter()
                    .map(|&index| &entries[index])
                    .collect::<Vec<_>>();
                let to_edges = entries_for_way(restriction.to_way_id)
                    .iter()
                    .map(|&index| &entries[index])
                    .collect::<Vec<_>>();
                let via_entries = via_way_ids(restriction)
                    .iter()
                    .flat_map(|&via_way_id| entries_for_way(via_way_id))
                    .map(|&index| entries[index].clone())
                    .collect::<Vec<_>>();
                if from_edges.is_empty() {
                    report.unresolved_missing_from_edge_count += 1;
                    unresolved_reason = Some(RoadGraphUnresolvedRestrictionReason::MissingFromEdge);
                } else if to_edges.is_empty() {
                    report.unresolved_missing_to_edge_count += 1;
                    unresolved_reason = Some(RoadGraphUnresolvedRestrictionReason::MissingToEdge);
                } else if via_entries.is_empty() {
                    report.unresolved_missing_via_node_or_path_count += 1;
                    unresolved_reason =
                        Some(RoadGraphUnresolvedRestrictionReason::MissingViaNodeOrPath);
                } else {
                    // For `only_*`: the turns allowed at the end of each
                    // (from edge, via path), keyed in edge order.
                    let mut only_turns = BTreeMap::<(u64, Vec<u64>), (usize, HashSet<u64>)>::new();
                    for from_edge in from_edges {
                        for to_edge in &to_edges {
                            let Some(via_edge_ids) = resolve_via_way_path(
                                &via_entries,
                                from_edge.target_index,
                                to_edge.source_index,
                            ) else {
                                continue;
                            };
                            if via_edge_ids.is_empty() {
                                continue;
                            }
                            if only {
                                only_turns
                                    .entry((from_edge.edge_id, via_edge_ids))
                                    .or_insert_with(|| (to_edge.source_index, HashSet::new()))
                                    .1
                                    .insert(to_edge.edge_id);
                            } else {
                                resolved.push(ResolvedEdgeRestriction::Complex {
                                    relation_id: restriction.relation_id,
                                    restriction: restriction.restriction.clone(),
                                    from_edge_id: from_edge.edge_id,
                                    to_edge_id: to_edge.edge_id,
                                    via_edge_ids,
                                    osm_mode_mask,
                                    mobility_mode_mask,
                                });
                            }
                        }
                    }
                    only_turn_found = !only_turns.is_empty();
                    for ((from_edge_id, via_edge_ids), (end_index, allowed)) in only_turns {
                        for to_edge_id in other_out_edges(graph, end_index, &allowed, osm_mode_mask)
                        {
                            resolved.push(ResolvedEdgeRestriction::Complex {
                                relation_id: restriction.relation_id,
                                restriction: restriction.restriction.clone(),
                                from_edge_id,
                                to_edge_id,
                                via_edge_ids: via_edge_ids.clone(),
                                osm_mode_mask,
                                mobility_mode_mask,
                            });
                        }
                    }
                    if resolved.len() == resolved_before && !only_turn_found {
                        report.unresolved_missing_via_node_or_path_count += 1;
                        unresolved_reason =
                            Some(RoadGraphUnresolvedRestrictionReason::MissingViaNodeOrPath);
                    }
                }
            }
            RelationMemberRefType::Relation => {
                report.unsupported_via_relation_count += 1;
                unresolved_reason =
                    Some(RoadGraphUnresolvedRestrictionReason::UnsupportedViaRelation);
            }
        }
        if resolved.len() == resolved_before && !only_turn_found {
            report.unresolved_restriction_count += 1;
            if report.unresolved_examples.len() < MAX_UNRESOLVED_RESTRICTION_EXAMPLES {
                report
                    .unresolved_examples
                    .push(RoadGraphUnresolvedRestrictionExample {
                        relation_id: restriction.relation_id,
                        restriction: restriction.restriction.clone(),
                        from_way_id: restriction.from_way_id,
                        to_way_id: restriction.to_way_id,
                        via_member_type: restriction.via_member_type,
                        via_member_id: restriction.via_member_id,
                        reason: unresolved_reason
                            .unwrap_or(RoadGraphUnresolvedRestrictionReason::MissingViaNodeOrPath),
                        except_modes: restriction.except_modes.clone(),
                    });
            }
        }
    }
    report.resolved_restriction_count = resolved.len();
    (resolved, report)
}

/// A restriction's via ways, in member order.
fn via_way_ids(restriction: &crate::RoadTurnRestriction) -> &[i64] {
    match restriction.via_member_type {
        RelationMemberRefType::Way if restriction.via_way_ids.is_empty() => {
            std::slice::from_ref(&restriction.via_member_id)
        }
        RelationMemberRefType::Way => &restriction.via_way_ids,
        _ => &[],
    }
}

/// The edges leaving `node_index` that an `only_*` restriction rules out: all
/// but the `allowed` ones, among those its modes may use.
fn other_out_edges(
    graph: &RoadGraph,
    node_index: usize,
    allowed: &HashSet<u64>,
    osm_mode_mask: u32,
) -> Vec<u64> {
    let mut edge_ids = graph
        .graph
        .edges_directed(NodeIndex::new(node_index), Direction::Outgoing)
        .map(|edge_ref| edge_ref.weight())
        .filter(|edge| {
            let edge_mask = match edge.traversal_direction {
                TraversalDirection::Forward => edge.osm_forward_mode_mask,
                TraversalDirection::Backward => edge.osm_reverse_mode_mask,
            };
            !allowed.contains(&edge.edge_id) && edge_mask & osm_mode_mask != 0
        })
        .map(|edge| edge.edge_id)
        .collect::<Vec<_>>();
    edge_ids.sort_unstable();
    edge_ids
}

fn resolve_via_way_path(
    via_entries: &[EdgeResolutionEntry],
    start_node_index: usize,
    end_node_index: usize,
) -> Option<Vec<u64>> {
    let mut by_source = HashMap::<usize, Vec<&EdgeResolutionEntry>>::new();
    for entry in via_entries {
        by_source.entry(entry.source_index).or_default().push(entry);
    }

    let mut queue = VecDeque::from([start_node_index]);
    let mut seen_nodes = BTreeSet::from([start_node_index]);
    let mut previous = HashMap::<usize, (usize, u64)>::new();
    while let Some(node_index) = queue.pop_front() {
        if node_index == end_node_index {
            break;
        }
        for entry in by_source
            .get(&node_index)
            .into_iter()
            .flat_map(|value| value.iter())
        {
            if seen_nodes.insert(entry.target_index) {
                previous.insert(entry.target_index, (node_index, entry.edge_id));
                queue.push_back(entry.target_index);
            }
        }
    }
    if !seen_nodes.contains(&end_node_index) {
        return None;
    }

    let mut node_index = end_node_index;
    let mut edge_ids = Vec::new();
    while node_index != start_node_index {
        let (previous_node, edge_id) = previous.get(&node_index).copied()?;
        edge_ids.push(edge_id);
        node_index = previous_node;
    }
    edge_ids.reverse();
    Some(edge_ids)
}

fn increment_restriction_kind_count(counts: &mut Vec<RoadGraphNamedCount>, name: &str) {
    if let Some(entry) = counts.iter_mut().find(|entry| entry.name == name) {
        entry.count += 1;
    } else {
        counts.push(RoadGraphNamedCount {
            name: name.to_string(),
            count: 1,
        });
    }
    counts.sort_by(|left, right| left.name.cmp(&right.name));
}

/// The OSM modes a restriction binds: every vehicle, as turn restrictions
/// never bind pedestrians, or the modes of its `restriction:<mode>` key; less
/// its `except` modes.
fn restriction_osm_mode_mask(modes: &[String], except_modes: &[String]) -> u32 {
    let mut mask = if modes.is_empty() {
        OsmMode::ALL & !OsmMode::FOOT
    } else {
        modes
            .iter()
            .fold(0, |mask, mode| mask | restriction_mode_bits(mode))
    };
    for mode in except_modes {
        mask &= !restriction_mode_bits(mode);
    }
    mask
}

/// The OSM modes a restriction's mode key names (`except=*`, `restriction:*`).
fn restriction_mode_bits(mode: &str) -> u32 {
    match mode {
        "vehicle" => OsmMode::MOTOR_VEHICLE | OsmMode::BICYCLE | OsmMode::BUS | OsmMode::HGV,
        "motor_vehicle" | "motorcar" => {
            OsmMode::MOTOR_VEHICLE | OsmMode::BUS | OsmMode::HGV | OsmMode::EMERGENCY
        }
        "bicycle" => OsmMode::BICYCLE,
        "foot" => OsmMode::FOOT,
        "bus" | "psv" => OsmMode::BUS,
        "hgv" | "goods" => OsmMode::HGV,
        "emergency" => OsmMode::EMERGENCY,
        _ => 0,
    }
}

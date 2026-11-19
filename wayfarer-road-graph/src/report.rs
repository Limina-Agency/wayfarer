//! Build, topology-repair, restriction and enhancement reports.

use super::*;

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct RoadGraphBuildReport {
    pub accepted_way_count: usize,
    pub dropped_closed_way_count: usize,
    pub dropped_restricted_way_count: usize,
    pub dropped_short_way_count: usize,
    pub accepted_way_counts_by_road_type: Vec<RoadGraphNamedCount>,
    pub rejected_way_counts_by_reason: Vec<RoadGraphNamedCount>,
    pub access_mask_distribution: Vec<RoadGraphAccessMaskCount>,
    pub created_node_count: usize,
    pub created_shared_segment_count: usize,
    pub created_directed_edge_count: usize,
    pub zero_access_edge_count: usize,
    /// Pedestrian plaza polygons for which interior anchor-star edges were
    /// synthesized (see [`RoadGraphBuilder`] plaza handling).
    pub synthesized_plaza_count: usize,
    /// Total perimeter anchors connected to synthesized plaza centers.
    pub plaza_anchor_count: usize,
    /// Directed interior edges added across all synthesized plazas.
    pub plaza_interior_edge_count: usize,
    /// Plaza polygons skipped because they had fewer than two usable anchors.
    pub plaza_skipped_insufficient_anchors: usize,
    /// Plaza polygons skipped because their area fell below the minimum threshold.
    pub plaza_skipped_below_min_area: usize,
    /// Plaza polygons skipped because a closed ring could not be reconstructed.
    pub plaza_skipped_open_ring: usize,
    /// Graph nodes that are barrier-tagged nodes.
    pub barrier_nodes_seen: usize,
    /// Barrier nodes that affect some profile in normal routing.
    pub barrier_constraints_applied: usize,
    /// Of those, the barriers that close some profile.
    pub barrier_hard_block_count: usize,
    /// Of those, the barriers some profile passes as a gate.
    pub barrier_conditional_count: usize,
    /// Barrier nodes with an unrecognised value (no effect in normal routing).
    pub barrier_unknown_value_count: usize,
    /// Directed edges given a traffic control at their end (signals, stop,
    /// give way, crossing signals).
    #[serde(default)]
    pub traffic_controlled_edge_count: usize,
    /// Drivable ways the law closes to every mode, kept for SAR only.
    #[serde(default)]
    pub sar_only_way_count: usize,
    /// Their directed edges (no legal access; not in the access tallies).
    #[serde(default)]
    pub sar_only_edge_count: usize,
    /// Graph nodes with a barrier class for the SAR `barriers` relaxation.
    #[serde(default)]
    pub sar_barrier_node_count: usize,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct RoadGraphAccessMaskCount {
    pub osm_mode_mask: ModeMask,
    pub mobility_mode_mask: ModeMask,
    pub count: usize,
}

impl RoadGraphBuildReport {
    pub(crate) fn record_accepted_way(&mut self, road_type: RoadType) {
        self.accepted_way_count += 1;
        increment_named_count(
            &mut self.accepted_way_counts_by_road_type,
            road_type_label(road_type),
        );
    }

    pub(crate) fn record_rejected_way(&mut self, reason: &str) {
        increment_named_count(&mut self.rejected_way_counts_by_reason, reason);
    }

    pub(crate) fn record_access_mask(
        &mut self,
        osm_mode_mask: ModeMask,
        mobility_mode_mask: ModeMask,
    ) {
        if let Some(entry) = self.access_mask_distribution.iter_mut().find(|entry| {
            entry.osm_mode_mask == osm_mode_mask && entry.mobility_mode_mask == mobility_mode_mask
        }) {
            entry.count += 1;
        } else {
            self.access_mask_distribution
                .push(RoadGraphAccessMaskCount {
                    osm_mode_mask,
                    mobility_mode_mask,
                    count: 1,
                });
        }
        self.access_mask_distribution.sort_by_key(|entry| {
            (
                entry.mobility_mode_mask,
                entry.osm_mode_mask,
                std::cmp::Reverse(entry.count),
            )
        });
    }
}

fn increment_named_count(counts: &mut Vec<RoadGraphNamedCount>, name: &str) {
    if let Some(entry) = counts.iter_mut().find(|entry| entry.name == name) {
        entry.count += 1;
    } else {
        counts.push(RoadGraphNamedCount {
            name: name.to_string(),
            count: 1,
        });
    }
    counts.sort_by(|a, b| a.name.cmp(&b.name));
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct RoadGraphTopologyRepairReport {
    pub input_road_line_count: usize,
    pub checked_segment_pair_count: usize,
    pub repaired_crossing_count: usize,
    pub inserted_topology_node_count: usize,
    pub split_edge_count: usize,
    pub skipped_endpoint_touch_count: usize,
    pub skipped_same_feature_count: usize,
    pub skipped_incompatible_plane_count: usize,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct RoadGraphRestrictionReport {
    pub raw_restriction_count: usize,
    pub resolved_restriction_count: usize,
    pub unresolved_restriction_count: usize,
    pub unresolved_missing_from_edge_count: usize,
    pub unresolved_missing_to_edge_count: usize,
    pub unresolved_missing_via_node_or_path_count: usize,
    pub unsupported_via_relation_count: usize,
    /// Restrictions left out because they bind only modes no profile routes
    /// (`restriction:hgv`, `:bus`, `:bicycle`, or a plain restriction whose
    /// `except` leaves only those). Neither resolved nor unresolved; left out
    /// of the report when zero, so packages without any keep their bytes.
    #[serde(skip_serializing_if = "is_zero")]
    pub mode_not_routed_restriction_count: usize,
    pub restriction_kind_counts: Vec<RoadGraphNamedCount>,
    pub unresolved_examples: Vec<RoadGraphUnresolvedRestrictionExample>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RoadGraphUnresolvedRestrictionReason {
    MissingFromEdge,
    MissingToEdge,
    #[default]
    MissingViaNodeOrPath,
    UnsupportedViaRelation,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct RoadGraphUnresolvedRestrictionExample {
    pub relation_id: i64,
    pub restriction: String,
    pub from_way_id: i64,
    pub to_way_id: i64,
    pub via_member_type: RelationMemberRefType,
    pub via_member_id: i64,
    pub reason: RoadGraphUnresolvedRestrictionReason,
    pub except_modes: Vec<String>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct RoadGraphEnhancementReport {
    pub component_count: usize,
    pub largest_component_node_count: usize,
    pub small_component_count: usize,
    pub component_size_histogram: Vec<RoadGraphComponentSizeBucket>,
    pub isolated_node_count: usize,
    pub dead_end_node_count: usize,
    pub through_node_count: usize,
    pub intersection_node_count: usize,
    pub roundabout_node_count: usize,
    pub complex_junction_node_count: usize,
    pub edges_with_heading_count: usize,
    pub edges_with_inferred_speed_count: usize,
    /// Edges that yield where they end (see `RoadEdge::end_minor_approach`).
    #[serde(default)]
    pub minor_approach_edge_count: usize,
    pub missing_shared_edge_info_count: usize,
    pub missing_heading_edge_count: usize,
    pub missing_inferred_speed_edge_count: usize,
    pub hierarchy_prepared_edge_count: usize,
    pub missing_hierarchy_tier_edge_count: usize,
    pub hierarchy_tier_distribution: Vec<RoadGraphNamedCount>,
    pub one_way_asymmetry_count: usize,
    pub missing_opposing_edge_count: usize,
    pub directional_access_mismatch_count: usize,
    pub duplicate_parallel_edge_count: usize,
    pub missing_geometry_count: usize,
    pub distance_geometry_mismatch_count: usize,
    pub unreachable_major_edge_count: usize,
    pub unconnected_link_edge_count: usize,
    pub major_edge_missing_lane_fact_count: usize,
    pub major_edge_missing_sign_or_destination_fact_count: usize,
    pub major_edge_missing_admin_fact_count: usize,
    pub major_edge_missing_timezone_fact_count: usize,
    pub near_duplicate_node_cluster_count: usize,
    pub unsnapped_endpoint_cluster_count: usize,
    pub shortcut_candidate_report: RoadGraphShortcutCandidateReport,
    pub warning_examples: Vec<RoadGraphQualityExample>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct RoadGraphComponentSizeBucket {
    pub label: String,
    pub min_node_count: usize,
    pub max_node_count: Option<usize>,
    pub component_count: usize,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct RoadGraphShortcutCandidateReport {
    pub candidate_count: usize,
    pub candidate_edge_count: usize,
    pub examples: Vec<RoadGraphShortcutCandidateExample>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct RoadGraphShortcutCandidateExample {
    pub start_node_id: i64,
    pub end_node_id: i64,
    pub edge_count: usize,
    pub distance_millimeters: u64,
    pub road_type: RoadType,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ref_name: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub component_id: Option<u32>,
}

fn is_zero(count: &usize) -> bool {
    *count == 0
}

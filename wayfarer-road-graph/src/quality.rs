//! The quality manifest and policy: findings, their evaluation, and the
//! package manifest built from a graph.

use super::*;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RoadGraphQualityExampleKind {
    NearDuplicateNodeCluster,
    UnsnappedEndpointCluster,
    #[default]
    SmallComponent,
    NonServiceDeadEnd,
    MissingHeading,
    MissingInferredSpeed,
    AbnormalEdgeLength,
    MissingOpposingEdge,
    DirectionalAccessMismatch,
    DuplicateParallelEdge,
    MissingGeometry,
    DistanceGeometryMismatch,
    UnreachableMajorRoad,
    UnconnectedLink,
    MissingRichRoadFacts,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct RoadGraphQualityExample {
    pub kind: RoadGraphQualityExampleKind,
    pub message: String,
    pub lat_fixed: Option<i32>,
    pub lng_fixed: Option<i32>,
    pub node_id: Option<i64>,
    pub edge_id: Option<u64>,
    pub component_id: Option<u32>,
    #[serde(default)]
    pub source_lineage_ids: Vec<i64>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RoadGraphQualityStatus {
    #[default]
    Ready,
    Warning,
    Stale,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RoadGraphQualityManifest {
    #[serde(default)]
    pub builder_report: RoadGraphBuildReport,
    #[serde(default)]
    pub topology_repair_report: RoadGraphTopologyRepairReport,
    #[serde(default)]
    pub restriction_report: RoadGraphRestrictionReport,
    #[serde(default)]
    pub enhancement_report: RoadGraphEnhancementReport,
    #[serde(default)]
    pub status: RoadGraphQualityStatus,
    #[serde(default)]
    pub reasons: Vec<String>,
    #[serde(default)]
    pub findings: Vec<RoadGraphQualityFinding>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RoadGraphQualityFindingSeverity {
    #[default]
    Info,
    Warning,
    Stale,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RoadGraphQualityFindingKind {
    LargestComponentRatio,
    UnresolvedRestrictions,
    MissingSharedEdgeInfo,
    MissingHeading,
    MissingInferredSpeed,
    MissingHierarchyTier,
    SmallComponents,
    OneWayAsymmetry,
    MissingOpposingEdge,
    DirectionalAccessMismatch,
    DuplicateParallelEdge,
    #[default]
    MissingGeometry,
    DistanceGeometryMismatch,
    UnreachableMajorRoad,
    UnconnectedLink,
    MissingRouteMembership,
    MissingRichRoadFacts,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct RoadGraphQualityFinding {
    pub kind: RoadGraphQualityFindingKind,
    pub severity: RoadGraphQualityFindingSeverity,
    pub count: Option<usize>,
    pub message: String,
    pub recommended_action: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RoadGraphQualityPolicy {
    pub policy_id: String,
    #[serde(default)]
    pub fatal_finding_kinds: Vec<RoadGraphQualityFindingKind>,
    #[serde(default = "default_treat_stale_findings_as_fatal")]
    pub treat_stale_findings_as_fatal: bool,
}

impl RoadGraphQualityPolicy {
    pub fn production_release() -> Self {
        Self {
            policy_id: "production-release-v1".to_string(),
            fatal_finding_kinds: vec![
                RoadGraphQualityFindingKind::LargestComponentRatio,
                RoadGraphQualityFindingKind::UnresolvedRestrictions,
                RoadGraphQualityFindingKind::MissingSharedEdgeInfo,
                RoadGraphQualityFindingKind::MissingGeometry,
                RoadGraphQualityFindingKind::UnreachableMajorRoad,
            ],
            treat_stale_findings_as_fatal: true,
        }
    }
}

impl Default for RoadGraphQualityPolicy {
    fn default() -> Self {
        Self::production_release()
    }
}

fn default_treat_stale_findings_as_fatal() -> bool {
    true
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RoadGraphQualityPolicyStatus {
    #[default]
    Pass,
    Fail,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct RoadGraphQualityPolicyEvaluation {
    pub policy_id: String,
    pub status: RoadGraphQualityPolicyStatus,
    pub fatal_finding_count: usize,
    pub advisory_finding_count: usize,
    pub fatal_findings: Vec<RoadGraphQualityFinding>,
    pub advisory_findings: Vec<RoadGraphQualityFinding>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RoadGraphPackageManifest {
    pub graph_schema_version: u32,
    pub container_version: u16,
    pub codec: String,
    pub uncompressed_bytes: u64,
    pub compressed_bytes: u64,
    pub compression_ratio: f64,
    pub blake3_hash_hex: String,
    pub node_count: usize,
    pub directed_edge_count: usize,
    pub shared_edge_info_count: usize,
    pub raw_turn_restriction_count: usize,
    pub resolved_turn_restriction_count: usize,
    pub route_membership_count: usize,
    pub zero_access_edge_count: usize,
    pub one_way_edge_count: usize,
    pub bidirectional_segment_count: usize,
    pub largest_weakly_connected_component_ratio: f64,
    #[serde(default)]
    pub quality: RoadGraphQualityManifest,
}

pub fn build_road_graph_manifest(
    graph: &RoadGraph,
    container: &RoadGraphContainerInfo,
) -> RoadGraphPackageManifest {
    let stats = RoadGraphStats::from_graph(graph);
    RoadGraphPackageManifest {
        graph_schema_version: container.graph_schema_version,
        container_version: container.container_version,
        codec: container.codec.clone(),
        uncompressed_bytes: container.uncompressed_bytes,
        compressed_bytes: container.compressed_bytes,
        compression_ratio: if container.uncompressed_bytes == 0 {
            1.0
        } else {
            container.compressed_bytes as f64 / container.uncompressed_bytes as f64
        },
        blake3_hash_hex: container.blake3_hash_hex.clone(),
        node_count: stats.node_count,
        directed_edge_count: stats.directed_edge_count,
        shared_edge_info_count: stats.shared_edge_info_count,
        raw_turn_restriction_count: stats.raw_turn_restriction_count,
        resolved_turn_restriction_count: stats.resolved_turn_restriction_count,
        route_membership_count: stats.route_membership_count,
        zero_access_edge_count: stats.zero_access_edge_count,
        one_way_edge_count: stats.one_way_edge_count,
        bidirectional_segment_count: stats.bidirectional_segment_count,
        largest_weakly_connected_component_ratio: stats.largest_weakly_connected_component_ratio,
        quality: RoadGraphQualityManifest::default(),
    }
}

pub fn build_road_graph_manifest_with_quality(
    graph: &RoadGraph,
    container: &RoadGraphContainerInfo,
    quality: RoadGraphQualityManifest,
) -> RoadGraphPackageManifest {
    let mut manifest = build_road_graph_manifest(graph, container);
    manifest.quality = quality;
    manifest
}

pub fn evaluate_road_graph_quality(
    manifest: &RoadGraphPackageManifest,
) -> RoadGraphQualityManifest {
    let mut quality = manifest.quality.clone();
    let mut stale_reasons = Vec::new();
    let mut warning_reasons = if manifest.quality.status == RoadGraphQualityStatus::Stale {
        Vec::new()
    } else {
        manifest.quality.reasons.clone()
    };
    let mut findings = Vec::new();
    let mut push_finding = |kind: RoadGraphQualityFindingKind,
                            severity: RoadGraphQualityFindingSeverity,
                            count: Option<usize>,
                            message: String,
                            recommended_action: &str| {
        findings.push(RoadGraphQualityFinding {
            kind,
            severity,
            count,
            message,
            recommended_action: recommended_action.to_string(),
        });
    };
    if manifest.quality.status == RoadGraphQualityStatus::Stale {
        stale_reasons.extend(manifest.quality.reasons.clone());
    }
    if manifest.node_count >= 1000 && manifest.largest_weakly_connected_component_ratio < 0.80 {
        let message = format!(
            "largest weakly-connected component ratio {:.3} is below 0.800",
            manifest.largest_weakly_connected_component_ratio
        );
        stale_reasons.push(message.clone());
        push_finding(
            RoadGraphQualityFindingKind::LargestComponentRatio,
            RoadGraphQualityFindingSeverity::Stale,
            None,
            message,
            "Inspect disconnected source coverage and topology repair before publishing the package.",
        );
    }
    let restriction_report = &manifest.quality.restriction_report;
    if restriction_report.raw_restriction_count >= 10 {
        let unresolved_ratio = restriction_report.unresolved_restriction_count as f64
            / restriction_report.raw_restriction_count as f64;
        if unresolved_ratio > 0.50 {
            let message = format!(
                "unresolved turn restriction ratio {:.3} is above 0.500",
                unresolved_ratio
            );
            stale_reasons.push(message.clone());
            push_finding(
                RoadGraphQualityFindingKind::UnresolvedRestrictions,
                RoadGraphQualityFindingSeverity::Stale,
                Some(restriction_report.unresolved_restriction_count),
                message,
                "Review restriction relation parsing and source-way edge resolution before using turn guidance.",
            );
        }
    }
    let enhancement = &manifest.quality.enhancement_report;
    if enhancement.missing_shared_edge_info_count > 0 {
        let message = format!(
            "{} directed edges reference missing shared edge metadata",
            enhancement.missing_shared_edge_info_count
        );
        warning_reasons.push(message.clone());
        push_finding(
            RoadGraphQualityFindingKind::MissingSharedEdgeInfo,
            RoadGraphQualityFindingSeverity::Warning,
            Some(enhancement.missing_shared_edge_info_count),
            message,
            "Rebuild the road graph from source OSM and verify shared edge info assignment.",
        );
    }
    if enhancement.missing_heading_edge_count > 0 {
        let message = format!(
            "{} directed edges are missing heading metadata",
            enhancement.missing_heading_edge_count
        );
        warning_reasons.push(message.clone());
        push_finding(
            RoadGraphQualityFindingKind::MissingHeading,
            RoadGraphQualityFindingSeverity::Warning,
            Some(enhancement.missing_heading_edge_count),
            message,
            "Regenerate graph enhancements so maneuver and turn classification can use edge headings.",
        );
    }
    if enhancement.missing_inferred_speed_edge_count > 0 {
        let message = format!(
            "{} directed edges are missing inferred speed metadata",
            enhancement.missing_inferred_speed_edge_count
        );
        warning_reasons.push(message.clone());
        push_finding(
            RoadGraphQualityFindingKind::MissingInferredSpeed,
            RoadGraphQualityFindingSeverity::Warning,
            Some(enhancement.missing_inferred_speed_edge_count),
            message,
            "Refresh speed inference inputs or accept conservative default speeds for affected edges.",
        );
    }
    if enhancement.missing_hierarchy_tier_edge_count > 0 {
        let message = format!(
            "{} directed edges are missing hierarchy tier metadata",
            enhancement.missing_hierarchy_tier_edge_count
        );
        warning_reasons.push(message.clone());
        push_finding(
            RoadGraphQualityFindingKind::MissingHierarchyTier,
            RoadGraphQualityFindingSeverity::Warning,
            Some(enhancement.missing_hierarchy_tier_edge_count),
            message,
            "Re-run hierarchy preparation so long-distance routing can prune consistently.",
        );
    }
    if enhancement.component_count > 1 && enhancement.small_component_count > 0 {
        let message = format!(
            "{} small weak components were detected",
            enhancement.small_component_count
        );
        warning_reasons.push(message.clone());
        push_finding(
            RoadGraphQualityFindingKind::SmallComponents,
            RoadGraphQualityFindingSeverity::Warning,
            Some(enhancement.small_component_count),
            message,
            "Inspect examples near package boundaries and missing connector ways; prune harmless islands only after review.",
        );
    }
    if enhancement.one_way_asymmetry_count > 0 {
        let message = format!(
            "{} one-way shared segments require asymmetric routing checks",
            enhancement.one_way_asymmetry_count
        );
        warning_reasons.push(message.clone());
        push_finding(
            RoadGraphQualityFindingKind::OneWayAsymmetry,
            RoadGraphQualityFindingSeverity::Warning,
            Some(enhancement.one_way_asymmetry_count),
            message,
            "Verify OSM one-way and mode-specific access tags for affected shared segments.",
        );
    }
    if enhancement.missing_opposing_edge_count > 0 {
        let message = format!(
            "{} bidirectional directed edges are missing opposing edges",
            enhancement.missing_opposing_edge_count
        );
        warning_reasons.push(message.clone());
        push_finding(
            RoadGraphQualityFindingKind::MissingOpposingEdge,
            RoadGraphQualityFindingSeverity::Warning,
            Some(enhancement.missing_opposing_edge_count),
            message,
            "Check bidirectional way splitting and opposing-edge lookup before relying on reversible maneuvers.",
        );
    }
    if enhancement.directional_access_mismatch_count > 0 {
        let message = format!(
            "{} directed edges have directional access metadata mismatches",
            enhancement.directional_access_mismatch_count
        );
        warning_reasons.push(message.clone());
        push_finding(
            RoadGraphQualityFindingKind::DirectionalAccessMismatch,
            RoadGraphQualityFindingSeverity::Warning,
            Some(enhancement.directional_access_mismatch_count),
            message,
            "Compare directed edge mode masks with shared OSM access tags and fix access normalization.",
        );
    }
    if enhancement.duplicate_parallel_edge_count > 0 {
        let message = format!(
            "{} duplicate parallel directed edges were detected",
            enhancement.duplicate_parallel_edge_count
        );
        warning_reasons.push(message.clone());
        push_finding(
            RoadGraphQualityFindingKind::DuplicateParallelEdge,
            RoadGraphQualityFindingSeverity::Warning,
            Some(enhancement.duplicate_parallel_edge_count),
            message,
            "Inspect duplicate OSM way lineage and dedupe only when access, layer, or geometry do not justify separate edges.",
        );
    }
    if enhancement.missing_geometry_count > 0 {
        let message = format!(
            "{} directed edges reference missing or invalid geometry",
            enhancement.missing_geometry_count
        );
        warning_reasons.push(message.clone());
        push_finding(
            RoadGraphQualityFindingKind::MissingGeometry,
            RoadGraphQualityFindingSeverity::Warning,
            Some(enhancement.missing_geometry_count),
            message,
            "Rebuild shared geometry from source nodes; affected edges may snap and render poorly.",
        );
    }
    if enhancement.distance_geometry_mismatch_count > 0 {
        let message = format!(
            "{} directed edges have distance/geometry mismatch",
            enhancement.distance_geometry_mismatch_count
        );
        warning_reasons.push(message.clone());
        push_finding(
            RoadGraphQualityFindingKind::DistanceGeometryMismatch,
            RoadGraphQualityFindingSeverity::Warning,
            Some(enhancement.distance_geometry_mismatch_count),
            message,
            "Recompute edge distances from shared geometry and inspect large examples for malformed shapes.",
        );
    }
    if enhancement.unreachable_major_edge_count > 0 {
        let message = format!(
            "{} major directed edges are outside the largest component",
            enhancement.unreachable_major_edge_count
        );
        warning_reasons.push(message.clone());
        push_finding(
            RoadGraphQualityFindingKind::UnreachableMajorRoad,
            RoadGraphQualityFindingSeverity::Warning,
            Some(enhancement.unreachable_major_edge_count),
            message,
            "Inspect missing ramps, boundary cuts, or source gaps around major road examples.",
        );
    }
    if enhancement.unconnected_link_edge_count > 0 {
        let message = format!(
            "{} link directed edges appear disconnected or weakly attached",
            enhancement.unconnected_link_edge_count
        );
        warning_reasons.push(message.clone());
        push_finding(
            RoadGraphQualityFindingKind::UnconnectedLink,
            RoadGraphQualityFindingSeverity::Warning,
            Some(enhancement.unconnected_link_edge_count),
            message,
            "Review ramp/link endpoints and source-node snapping around affected junctions.",
        );
    }
    let missing_rich_fact_total = enhancement.major_edge_missing_lane_fact_count
        + enhancement.major_edge_missing_sign_or_destination_fact_count
        + enhancement.major_edge_missing_admin_fact_count
        + enhancement.major_edge_missing_timezone_fact_count;
    if missing_rich_fact_total > 0 {
        let message = format!(
            "major-road rich fact coverage is incomplete: lane={}, sign_or_destination={}, admin={}, timezone={}",
            enhancement.major_edge_missing_lane_fact_count,
            enhancement.major_edge_missing_sign_or_destination_fact_count,
            enhancement.major_edge_missing_admin_fact_count,
            enhancement.major_edge_missing_timezone_fact_count
        );
        push_finding(
            RoadGraphQualityFindingKind::MissingRichRoadFacts,
            RoadGraphQualityFindingSeverity::Info,
            Some(missing_rich_fact_total),
            message,
            "Improve source fact extraction and package metadata joins for lane guidance, signs, admin summaries, and timezone-aware diagnostics.",
        );
    }
    if manifest.shared_edge_info_count >= 100 && manifest.route_membership_count == 0 {
        let message = "no route membership coverage recorded for a non-trivial graph".to_string();
        warning_reasons.push(message.clone());
        push_finding(
            RoadGraphQualityFindingKind::MissingRouteMembership,
            RoadGraphQualityFindingSeverity::Warning,
            None,
            message,
            "Verify route relation ingestion if shield, ref, or road-name guidance depends on memberships.",
        );
    }
    findings.sort_by(|left, right| {
        left.severity
            .cmp(&right.severity)
            .reverse()
            .then_with(|| left.kind.cmp(&right.kind))
            .then_with(|| left.message.cmp(&right.message))
    });
    findings.dedup_by(|left, right| left.kind == right.kind && left.message == right.message);
    warning_reasons.sort();
    warning_reasons.dedup();
    if stale_reasons.is_empty() {
        quality.status = if warning_reasons.is_empty() {
            RoadGraphQualityStatus::Ready
        } else {
            RoadGraphQualityStatus::Warning
        };
        quality.reasons = warning_reasons;
    } else {
        quality.status = RoadGraphQualityStatus::Stale;
        quality.reasons = stale_reasons;
    }
    quality.findings = findings;
    quality
}

pub fn evaluate_road_graph_quality_policy(
    quality: &RoadGraphQualityManifest,
    policy: &RoadGraphQualityPolicy,
) -> RoadGraphQualityPolicyEvaluation {
    let mut fatal_findings = Vec::new();
    let mut advisory_findings = Vec::new();
    for finding in &quality.findings {
        let is_fatal = policy.fatal_finding_kinds.contains(&finding.kind)
            || (policy.treat_stale_findings_as_fatal
                && finding.severity == RoadGraphQualityFindingSeverity::Stale);
        if is_fatal {
            fatal_findings.push(finding.clone());
        } else {
            advisory_findings.push(finding.clone());
        }
    }
    fatal_findings.sort_by(|left, right| {
        left.kind
            .cmp(&right.kind)
            .then_with(|| left.message.cmp(&right.message))
    });
    advisory_findings.sort_by(|left, right| {
        left.severity
            .cmp(&right.severity)
            .reverse()
            .then_with(|| left.kind.cmp(&right.kind))
            .then_with(|| left.message.cmp(&right.message))
    });
    RoadGraphQualityPolicyEvaluation {
        policy_id: policy.policy_id.clone(),
        status: if fatal_findings.is_empty() {
            RoadGraphQualityPolicyStatus::Pass
        } else {
            RoadGraphQualityPolicyStatus::Fail
        },
        fatal_finding_count: fatal_findings.len(),
        advisory_finding_count: advisory_findings.len(),
        fatal_findings,
        advisory_findings,
    }
}

#[derive(Debug, Clone, PartialEq)]
struct RoadGraphStats {
    node_count: usize,
    directed_edge_count: usize,
    shared_edge_info_count: usize,
    raw_turn_restriction_count: usize,
    resolved_turn_restriction_count: usize,
    route_membership_count: usize,
    zero_access_edge_count: usize,
    one_way_edge_count: usize,
    bidirectional_segment_count: usize,
    largest_weakly_connected_component_ratio: f64,
}

impl RoadGraphStats {
    fn from_graph(graph: &RoadGraph) -> Self {
        let shared_by_id = graph
            .shared_edge_infos
            .iter()
            .map(|info| (info.shared_edge_info_id, info.attrs.directional_access))
            .collect::<HashMap<_, _>>();
        let zero_access_edge_count = graph
            .graph
            .edge_weights()
            .filter(|edge| edge.mobility_mode_mask == 0)
            .count();
        let one_way_edge_count = graph
            .graph
            .edge_weights()
            .filter(|edge| {
                shared_by_id
                    .get(&edge.shared_edge_info_id)
                    .is_some_and(|access| *access != DirectionalAccess::Both)
            })
            .count();
        let bidirectional_segment_count = graph
            .shared_edge_infos
            .iter()
            .filter(|info| info.attrs.directional_access == DirectionalAccess::Both)
            .count();

        Self {
            node_count: graph.node_count(),
            directed_edge_count: graph.edge_count(),
            shared_edge_info_count: graph.shared_edge_infos.len(),
            raw_turn_restriction_count: graph.turn_restrictions.len(),
            resolved_turn_restriction_count: graph.resolved_restrictions.len(),
            route_membership_count: graph.route_memberships.len(),
            zero_access_edge_count,
            one_way_edge_count,
            bidirectional_segment_count,
            largest_weakly_connected_component_ratio: largest_weak_component_ratio(graph),
        }
    }
}

fn largest_weak_component_ratio(graph: &RoadGraph) -> f64 {
    let node_count = graph.node_count();
    if node_count == 0 {
        return 0.0;
    }

    let mut adjacency = vec![Vec::<usize>::new(); node_count];
    for edge in graph.graph.edge_references() {
        let source = edge.source().index();
        let target = edge.target().index();
        if source < node_count && target < node_count {
            adjacency[source].push(target);
            adjacency[target].push(source);
        }
    }

    let mut seen = vec![false; node_count];
    let mut largest = 0usize;
    for start in 0..node_count {
        if seen[start] {
            continue;
        }
        let mut component_size = 0usize;
        let mut queue = VecDeque::from([start]);
        seen[start] = true;
        while let Some(node) = queue.pop_front() {
            component_size += 1;
            for &next in &adjacency[node] {
                if !seen[next] {
                    seen[next] = true;
                    queue.push_back(next);
                }
            }
        }
        largest = largest.max(component_size);
    }

    largest as f64 / node_count as f64
}

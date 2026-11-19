//! One leg between two points: its request, result, maneuvers, attribution, summaries, diagnostics and errors.

use super::*;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct LegRequest {
    pub origin: GeoPoint,
    pub destination: GeoPoint,
    pub profile: MobilityProfile,
    #[serde(default)]
    pub options: RouteOptions,
    #[serde(default)]
    pub origin_options: RouteEndpointOptions,
    #[serde(default)]
    pub destination_options: RouteEndpointOptions,
}

impl LegRequest {
    pub fn new(origin: GeoPoint, destination: GeoPoint, profile: MobilityProfile) -> Self {
        Self {
            origin,
            destination,
            profile,
            options: RouteOptions::default(),
            origin_options: RouteEndpointOptions::default(),
            destination_options: RouteEndpointOptions::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteStatus {
    Routed,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct LegResult {
    pub status: RouteStatus,
    pub origin: CorrelatedLocation,
    pub destination: CorrelatedLocation,
    pub edge_ids: Vec<u64>,
    pub distance_m: f64,
    pub cost: f64,
    pub duration_s: f64,
    pub geometry: Vec<GeoPoint>,
    pub legs: Vec<RouteLeg>,
    pub maneuvers: Vec<RouteManeuver>,
    #[serde(default)]
    pub alternatives: Vec<RouteAlternative>,
    #[serde(default)]
    pub recosts: Vec<RouteRecostSummary>,
    #[serde(default)]
    pub policy_summary: RoutePolicySummary,
    #[serde(default)]
    pub cost_summary: RouteCostSummary,
    pub diagnostics: RouteDiagnostics,
    /// Where the route's legal access is qualified (`destination`,
    /// `permissive`, …), on every route (see `crate::sar`).
    #[serde(default)]
    pub qualified_access: Vec<crate::sar::RouteQualifiedAccess>,
    /// The SAR relaxations the route used; requests with SAR only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sar: Option<crate::sar::RouteSarReport>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteCostDriverKind {
    ProfileTime,
    RoadType,
    Surface,
    TrackQuality,
    RoadUse,
    Overlay,
    Avoidance,
    Turns,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteCostDriverSummary {
    pub kind: RouteCostDriverKind,
    pub cost: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteCostSummary {
    pub attributed_edge_count: usize,
    pub costed_edge_count: usize,
    pub total_cost: f64,
    #[serde(default)]
    pub dominant_driver: Option<RouteCostDriverSummary>,
    pub profile_time_penalty_count: usize,
    pub max_profile_time_factor: f64,
    pub overlay_multiplier_count: usize,
    pub max_overlay_multiplier: f64,
    pub turn_penalty_total: f64,
    pub avoided_penalty_count: usize,
    pub road_use_penalty_count: usize,
    pub max_road_use_penalty: f64,
    #[serde(default)]
    pub conditional_restriction_edge_count: usize,
}

impl Default for RouteCostSummary {
    fn default() -> Self {
        Self {
            attributed_edge_count: 0,
            costed_edge_count: 0,
            total_cost: 0.0,
            dominant_driver: None,
            profile_time_penalty_count: 0,
            max_profile_time_factor: 1.0,
            overlay_multiplier_count: 0,
            max_overlay_multiplier: 1.0,
            turn_penalty_total: 0.0,
            avoided_penalty_count: 0,
            road_use_penalty_count: 0,
            max_road_use_penalty: 1.0,
            conditional_restriction_edge_count: 0,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RoutePolicySummary {
    pub profile: MobilityProfile,
    #[serde(default)]
    pub max_visited_edge_count: Option<usize>,
    pub blocked_edge_count: usize,
    pub avoided_edge_count: usize,
    #[serde(default)]
    pub avoided_range_count: usize,
    pub warning_edge_count: usize,
    pub cost_multiplier_edge_count: usize,
    #[serde(default)]
    pub cost_multiplier_range_count: usize,
    pub avoid_tracks: f64,
    pub avoid_paths: f64,
    #[serde(default)]
    pub avoid_motorways: f64,
    pub avoid_destination_only: f64,
    pub allow_ford: bool,
    #[serde(default)]
    pub avoid_tolls: f64,
    #[serde(default)]
    pub avoid_ferries: f64,
    #[serde(default)]
    pub avoid_gates: f64,
    #[serde(default)]
    pub avoid_access_restricted: f64,
    #[serde(default)]
    pub prefer_paved: f64,
    pub prefer_known_route_refs: f64,
    pub emergency_override_strength: f64,
    #[serde(default)]
    pub speed_preference: RouteSpeedPreference,
    #[serde(default)]
    pub fixed_speed_kph: Option<f64>,
    #[serde(default)]
    pub top_speed_kph: Option<f64>,
    #[serde(default)]
    pub conditional_restriction_eval_at_ms: Option<i64>,
    pub requested_alternative_count: usize,
    pub max_shared_edge_ratio: f64,
    pub max_cost_ratio: f64,
    pub minimum_warning_difference: usize,
    #[serde(default)]
    pub dynamic_overlay_split_edge_count: usize,
    #[serde(default)]
    pub dynamic_overlay_partial_split_edge_count: usize,
    #[serde(default)]
    pub dynamic_overlay_virtual_connector_count: usize,
    /// The effective SAR surcharges; requests with SAR only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sar: Option<crate::sar::RouteSarPolicy>,
}

impl Default for RoutePolicySummary {
    fn default() -> Self {
        Self {
            profile: MobilityProfile::Default,
            max_visited_edge_count: None,
            blocked_edge_count: 0,
            avoided_edge_count: 0,
            avoided_range_count: 0,
            warning_edge_count: 0,
            cost_multiplier_edge_count: 0,
            cost_multiplier_range_count: 0,
            avoid_tracks: 0.0,
            avoid_paths: 0.0,
            avoid_motorways: 0.0,
            avoid_destination_only: 0.0,
            allow_ford: true,
            avoid_tolls: 0.0,
            avoid_ferries: 0.0,
            avoid_gates: 0.0,
            avoid_access_restricted: 0.0,
            prefer_paved: 0.0,
            prefer_known_route_refs: 0.0,
            emergency_override_strength: 0.0,
            speed_preference: RouteSpeedPreference::default(),
            fixed_speed_kph: None,
            top_speed_kph: None,
            conditional_restriction_eval_at_ms: None,
            requested_alternative_count: 0,
            max_shared_edge_ratio: default_max_shared_edge_ratio(),
            max_cost_ratio: default_max_alternative_cost_ratio(),
            minimum_warning_difference: 0,
            dynamic_overlay_split_edge_count: 0,
            dynamic_overlay_partial_split_edge_count: 0,
            dynamic_overlay_virtual_connector_count: 0,
            sar: None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteRecostSummary {
    pub profile: MobilityProfile,
    pub accessible: bool,
    pub distance_m: f64,
    #[serde(default)]
    pub cost: Option<f64>,
    #[serde(default)]
    pub duration_s: Option<f64>,
    pub blocked_edge_count: usize,
    pub inaccessible_edge_count: usize,
    pub destination_only_penalty_count: usize,
    pub ford_penalty_count: usize,
    #[serde(default)]
    pub toll_penalty_count: usize,
    #[serde(default)]
    pub ferry_penalty_count: usize,
    #[serde(default)]
    pub gate_penalty_count: usize,
    #[serde(default)]
    pub access_restriction_penalty_count: usize,
    pub overlay_multiplier_count: usize,
    pub turn_penalty_total: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteAlternativeLabel {
    Alternative,
    Safer,
    WarningBearing,
    BlockedPrimaryFallback,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RouteFailure {
    pub error: RouteError,
    pub diagnostics: Box<RouteDiagnostics>,
    /// The leg that failed, for a leg's search failing inside
    /// [`crate::RoutingEngine::route`].
    pub segment_index: Option<usize>,
}

impl std::fmt::Display for RouteFailure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.error.fmt(f)
    }
}

impl std::error::Error for RouteFailure {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.error)
    }
}

impl From<RouteFailure> for RouteError {
    fn from(failure: RouteFailure) -> Self {
        failure.error
    }
}

impl RouteFailure {
    pub fn new(error: RouteError, diagnostics: RouteDiagnostics) -> Self {
        Self {
            error,
            diagnostics: Box::new(diagnostics),
            segment_index: None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteAlternative {
    pub label: RouteAlternativeLabel,
    pub edge_ids: Vec<u64>,
    pub distance_m: f64,
    pub cost: f64,
    pub duration_s: f64,
    pub shared_edge_ratio: f64,
    pub warning_edge_count: usize,
    #[serde(default)]
    pub quality_score: f64,
    #[serde(default)]
    pub quality_reasons: Vec<String>,
    pub geometry: Vec<GeoPoint>,
    pub legs: Vec<RouteLeg>,
    pub maneuvers: Vec<RouteManeuver>,
    pub diagnostics: RouteDiagnostics,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteLeg {
    pub distance_m: f64,
    pub duration_s: f64,
    pub geometry: Vec<GeoPoint>,
    pub edges: Vec<RouteEdgeAttribution>,
    #[serde(default)]
    pub shape_segments: Vec<RouteShapeSegment>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteShapeQualityFlag {
    ClippedOriginEdge,
    ClippedDestinationEdge,
    WaypointSplit,
    DuplicatedEdgeSuppressed,
    ShapeFallbackUsed,
    SameEdgeDirect,
    SsvgPartialEdgeSplit,
    SsvgConnector,
    /// The leg starts away from where the previous leg ended (the waypoint
    /// snapped to different places for its two legs).
    WaypointGap,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteShapeSegment {
    pub edge_id: u64,
    pub begin_shape_index: usize,
    pub end_shape_index: usize,
    pub start_fraction: f64,
    pub end_fraction: f64,
    #[serde(default)]
    pub quality_flags: Vec<RouteShapeQualityFlag>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dynamic_overlay: Option<RouteShapeDynamicOverlay>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteShapeDynamicOverlayKind {
    PartialEdgeFragment,
    EndpointConnector,
    RoadEntryConnector,
    RoadToRoadBypassConnector,
    CrossCountryConnector,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteShapeDynamicOverlay {
    pub kind: RouteShapeDynamicOverlayKind,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub connector_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub connector_type: Option<RouteDynamicVirtualEdgeKind>,
    #[serde(default)]
    pub source_chunk_ids: Vec<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distance_meters: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cost: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source_endpoint: Option<RouteDynamicEndpointKind>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source_point: Option<GeoPoint>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source_edge_id: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source_fraction: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target_endpoint: Option<RouteDynamicEndpointKind>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target_point: Option<GeoPoint>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target_edge_id: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target_fraction: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteEdgeAttribution {
    pub edge_id: u64,
    pub shared_edge_info_id: u64,
    pub road_type: RoadType,
    pub name: Option<String>,
    pub ref_name: Option<String>,
    pub route_refs: Vec<String>,
    pub surface: Option<String>,
    pub tracktype: Option<String>,
    pub smoothness: Option<String>,
    #[serde(default)]
    pub maxspeed_kph: Option<f32>,
    #[serde(default)]
    pub inferred_speed_kph: Option<u16>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed_source: Option<RouteSpeedSource>,
    #[serde(default)]
    pub lane_count: Option<u8>,
    #[serde(default)]
    pub lanes: Option<String>,
    #[serde(default)]
    pub turn_lanes: Option<String>,
    #[serde(default)]
    pub turn_lanes_forward: Option<String>,
    #[serde(default)]
    pub turn_lanes_backward: Option<String>,
    #[serde(default)]
    pub lane_connectivity_available: bool,
    #[serde(default)]
    pub speed_source_provenance: Option<String>,
    #[serde(default)]
    pub free_flow_speed_kph: Option<f32>,
    #[serde(default)]
    pub constrained_speed_kph: Option<f32>,
    #[serde(default)]
    pub admin_country_id: Option<String>,
    #[serde(default)]
    pub admin_region_id: Option<String>,
    #[serde(default)]
    pub timezone_id: Option<String>,
    #[serde(default)]
    pub conditional_restrictions: Vec<String>,
    #[serde(default)]
    pub opposing_edge_id: Option<u64>,
    #[serde(default)]
    pub layer: Option<i16>,
    #[serde(default)]
    pub level: Option<String>,
    #[serde(default)]
    pub destination: Option<String>,
    #[serde(default)]
    pub destination_ref: Option<String>,
    #[serde(default)]
    pub destination_street: Option<String>,
    #[serde(default)]
    pub junction_ref: Option<String>,
    pub is_bridge: bool,
    pub is_tunnel: bool,
    pub has_ford: bool,
    #[serde(default)]
    pub is_toll: bool,
    #[serde(default)]
    pub is_ferry: bool,
    #[serde(default)]
    pub has_gate: bool,
    #[serde(default)]
    pub is_access_restricted: bool,
    #[serde(default)]
    pub is_destination_only: bool,
    #[serde(default)]
    pub is_link: bool,
    pub is_roundabout: bool,
    #[serde(default)]
    pub roundabout_exit_count: u16,
    pub has_warning_overlay: bool,
    #[serde(default)]
    pub warning_ranges: Vec<RouteEdgeRange>,
    #[serde(default)]
    pub junction_outgoing_edge_count: usize,
    #[serde(default)]
    pub junction_incoming_edge_count: usize,
    /// Signed angles (degrees, negative to the left) from the approach to each
    /// other edge the profile may take at the junction this edge starts at
    /// (U-turns excluded); empty on a leg's first edge.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub junction_branch_angles_deg: Vec<i16>,
    /// Signed angles (degrees) from the arrival of each other edge the profile
    /// may use into that junction to this edge's departure.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub junction_joining_angles_deg: Vec<i16>,
    pub distance_m: f64,
    /// Time on this edge's traversed stretch.
    pub duration_s: f64,
    /// Time lost at the junction this edge starts at, entering it from the
    /// leg's previous edge (turning, yielding, signals); 0 on a leg's first
    /// edge. A leg's duration is its edges' `duration_s` plus their
    /// `junction_delay_s`.
    #[serde(default)]
    pub junction_delay_s: f64,
    pub traversal_direction: TraversalDirection,
    #[serde(default)]
    pub cost_breakdown: RouteCostBreakdown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteManeuverKind {
    Start,
    Continue,
    Keep,
    Fork,
    Merge,
    Roundabout,
    Turn,
    UTurn,
    Hazard,
    /// Arrival at an intermediate waypoint of a multi-leg route.
    Waypoint,
    Destination,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteGuidanceQualityFlag {
    MissingRoadName,
    InferredHeading,
    ComplexJunction,
    WeakManeuverClassification,
    CombinedShortConnector,
    OverlayWarningOnClippedShape,
    SsvgConnector,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteManeuver {
    pub kind: RouteManeuverKind,
    pub instruction: String,
    #[serde(default)]
    pub concise_instruction: String,
    #[serde(default)]
    pub pre_transition_instruction: Option<String>,
    #[serde(default)]
    pub post_transition_instruction: Option<String>,
    #[serde(default)]
    pub alert_instruction: Option<String>,
    #[serde(default)]
    pub cardinal_direction: Option<String>,
    #[serde(default)]
    pub bearing_deg: Option<u16>,
    #[serde(default)]
    pub road_refs: Vec<String>,
    #[serde(default)]
    pub sign_text: Option<String>,
    #[serde(default)]
    pub branch_text: Option<String>,
    #[serde(default)]
    pub lane_count: Option<u8>,
    #[serde(default)]
    pub roundabout_exit_number: Option<u16>,
    #[serde(default)]
    pub instruction_key: String,
    #[serde(default)]
    pub instruction_parameters: BTreeMap<String, String>,
    #[serde(default)]
    pub quality_flags: Vec<RouteGuidanceQualityFlag>,
    pub distance_m: f64,
    pub duration_s: f64,
    pub begin_shape_index: usize,
    pub end_shape_index: usize,
    pub road_name: Option<String>,
    pub turn_angle_deg: Option<i16>,
    /// The lanes of the road approaching the maneuver's junction, left to
    /// right, from that road's `turn:lanes` tags; empty when it has none or
    /// the maneuver is not at a junction.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub lanes: Vec<RouteLane>,
    /// The leg of a multi-leg route this maneuver belongs to.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub leg_index: Option<usize>,
}

/// One lane of the road approaching a maneuver's junction.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteLane {
    /// The lane's marked directions, in tag order; `none` for an unmarked
    /// lane. Marks the engine does not know are left out.
    pub indications: Vec<RouteLaneIndication>,
    /// Whether the lane serves the maneuver.
    pub valid: bool,
}

/// A direction marked on a lane (an OSM `turn:lanes` value).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteLaneIndication {
    /// No marking (`none` or an empty value): usually straight on.
    None,
    Through,
    SlightLeft,
    Left,
    SharpLeft,
    SlightRight,
    Right,
    SharpRight,
    Reverse,
    MergeToLeft,
    MergeToRight,
}

impl Default for RouteManeuver {
    fn default() -> Self {
        Self {
            kind: RouteManeuverKind::Continue,
            instruction: String::new(),
            concise_instruction: String::new(),
            pre_transition_instruction: None,
            post_transition_instruction: None,
            alert_instruction: None,
            cardinal_direction: None,
            bearing_deg: None,
            road_refs: Vec::new(),
            sign_text: None,
            branch_text: None,
            lane_count: None,
            roundabout_exit_number: None,
            instruction_key: String::new(),
            instruction_parameters: BTreeMap::new(),
            quality_flags: Vec::new(),
            distance_m: 0.0,
            duration_s: 0.0,
            begin_shape_index: 0,
            end_shape_index: 0,
            road_name: None,
            turn_angle_deg: None,
            lanes: Vec::new(),
            leg_index: None,
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteDiagnostics {
    pub origin_candidate_count: usize,
    pub destination_candidate_count: usize,
    pub rejected_candidate_count: usize,
    #[serde(default)]
    pub origin_rejected_candidate_count: usize,
    #[serde(default)]
    pub destination_rejected_candidate_count: usize,
    #[serde(default)]
    pub origin_best_candidate_score_basis_points: Option<u32>,
    #[serde(default)]
    pub destination_best_candidate_score_basis_points: Option<u32>,
    #[serde(default)]
    pub origin_best_outbound_reach_edge_count: Option<usize>,
    #[serde(default)]
    pub destination_best_inbound_reach_edge_count: Option<usize>,
    #[serde(default)]
    pub relaxed_correlation_retry_count: usize,
    #[serde(default)]
    pub origin_relaxed_correlation_retry_count: usize,
    #[serde(default)]
    pub destination_relaxed_correlation_retry_count: usize,
    #[serde(default)]
    pub origin_rejected_candidates: Vec<RejectedCorrelationCandidate>,
    #[serde(default)]
    pub destination_rejected_candidates: Vec<RejectedCorrelationCandidate>,
    pub blocked_edge_count: usize,
    pub avoided_edge_count: usize,
    #[serde(default)]
    pub inaccessible_edge_count: usize,
    #[serde(default)]
    pub destination_only_penalty_count: usize,
    #[serde(default)]
    pub destination_only_relaxed_retry_count: usize,
    #[serde(default)]
    pub ford_penalty_count: usize,
    #[serde(default)]
    pub toll_penalty_count: usize,
    #[serde(default)]
    pub ferry_penalty_count: usize,
    #[serde(default)]
    pub gate_penalty_count: usize,
    #[serde(default)]
    pub access_restriction_penalty_count: usize,
    #[serde(default)]
    pub overlay_multiplier_count: usize,
    #[serde(default)]
    pub ssvg_partial_edge_fragment_used_count: usize,
    #[serde(default)]
    pub ssvg_connector_used_count: usize,
    #[serde(default)]
    pub ssvg_endpoint_connector_used_count: usize,
    #[serde(default)]
    pub ssvg_road_entry_connector_used_count: usize,
    #[serde(default)]
    pub ssvg_road_bypass_connector_used_count: usize,
    #[serde(default)]
    pub ssvg_cross_country_connector_used_count: usize,
    #[serde(default)]
    pub ssvg_connector_distance_meters: u64,
    #[serde(default)]
    pub ssvg_connector_cost: u64,
    pub restricted_transition_count: usize,
    #[serde(default)]
    pub normal_edge_expanded_count: usize,
    #[serde(default)]
    pub alternative_attempt_count: usize,
    #[serde(default)]
    pub accepted_alternative_count: usize,
    #[serde(default)]
    pub rejected_alternative_duplicate_count: usize,
    #[serde(default)]
    pub rejected_alternative_shared_edge_count: usize,
    #[serde(default)]
    pub rejected_alternative_cost_ratio_count: usize,
    #[serde(default)]
    pub rejected_alternative_warning_difference_count: usize,
    #[serde(default)]
    pub rejected_alternative_search_failed_count: usize,
    #[serde(default)]
    pub correlation_duration_ns: u64,
    #[serde(default)]
    pub search_duration_ns: u64,
    #[serde(default)]
    pub route_leg_build_duration_ns: u64,
    #[serde(default)]
    pub total_compute_duration_ns: u64,
    pub visited_edge_count: usize,
    pub settled_node_count: usize,
    pub final_path_edge_count: usize,
    #[serde(default)]
    pub forward_visited_edge_count: usize,
    pub correlation_mode: CorrelationMode,
    /// The visited-edge budget ran out after a route was found: the route is
    /// the best found so far, not proven optimal.
    #[serde(default)]
    pub search_budget_exceeded: bool,
    /// With `search_budget_exceeded`, how much dearer than optimal the route
    /// can be, in thousandths of a cost unit (rounded up), by the lower bound
    /// the search stops on: an upper bound for unidirectional searches, with
    /// the exact stop (`WAYFARER_EXACT_STOP`), and with the default
    /// bidirectional stop (the CCH potentials are consistent).
    #[serde(default)]
    pub search_optimality_gap_millicost: Option<u64>,
}

#[derive(Debug, Clone, Error, PartialEq)]
pub enum RouteError {
    #[error("road graph is not ready for routing")]
    GraphNotReady,
    #[error("no routable candidate found for {endpoint}")]
    NoCandidate { endpoint: &'static str },
    #[error("road graph has no edges accessible to mobility profile {profile:?}")]
    ImpossibleMobilityProfile { profile: MobilityProfile },
    #[error("origin and destination are in disconnected road graph components")]
    Disconnected,
    #[error(
        "route corridor is blocked after rejecting {blocked_edge_count} blocked edges and visiting {visited_edge_count} edges"
    )]
    BlockedCorridor {
        blocked_edge_count: usize,
        visited_edge_count: usize,
    },
    #[error("no path found after visiting {visited_edge_count} edges")]
    NoPath { visited_edge_count: usize },
    #[error(
        "route search budget exceeded after visiting {visited_edge_count} edges; max is {max_visited_edge_count}"
    )]
    SearchBudgetExceeded {
        visited_edge_count: usize,
        max_visited_edge_count: usize,
    },
    #[error("road graph is missing metadata for edge {edge_id}")]
    MissingEdgeMetadata { edge_id: u64 },
    /// `RouteOptions::cancellation` stopped the request: a token was
    /// cancelled, or (`deadline_exceeded`) its deadline passed.
    #[error("{}", if *.deadline_exceeded { "request deadline exceeded" } else { "request cancelled" })]
    Cancelled { deadline_exceeded: bool },
    /// The request asks for a SAR relaxation that needs package data this
    /// package does not have (see `RoutingPackage::sar_capability`).
    #[error("SAR relaxation {relaxation} is unavailable: the routing package has no SAR data")]
    SarUnavailable {
        relaxation: crate::sar::SarRelaxationKind,
    },
    /// The request's options are invalid (e.g. a SAR surcharge below the
    /// package minimum).
    #[error("invalid route request: {reason}")]
    InvalidRequest { reason: &'static str },
    /// No sample of a trace matched a road (`RoutingEngine::map_match`).
    #[error("no trace sample matched a road")]
    NoMatch,
    /// The request exceeds a bound in [`crate::limits`].
    #[error("route request exceeds its bounds: {what}")]
    RequestTooLarge { what: &'static str },
}

impl RouteError {
    pub fn is_no_candidate(&self) -> bool {
        matches!(self, Self::NoCandidate { .. })
    }

    pub fn is_impossible_mobility_profile(&self) -> bool {
        matches!(self, Self::ImpossibleMobilityProfile { .. })
    }

    pub fn is_no_path(&self) -> bool {
        matches!(self, Self::NoPath { .. })
    }

    pub fn is_blocked_corridor(&self) -> bool {
        matches!(self, Self::BlockedCorridor { .. })
    }

    pub fn is_disconnected(&self) -> bool {
        matches!(self, Self::Disconnected)
    }

    pub fn is_search_budget_exceeded(&self) -> bool {
        matches!(self, Self::SearchBudgetExceeded { .. })
    }

    pub fn is_cancelled(&self) -> bool {
        matches!(self, Self::Cancelled { .. })
    }

    pub fn is_sar_unavailable(&self) -> bool {
        matches!(self, Self::SarUnavailable { .. })
    }

    pub fn is_invalid_request(&self) -> bool {
        matches!(
            self,
            Self::InvalidRequest { .. } | Self::RequestTooLarge { .. }
        )
    }
}

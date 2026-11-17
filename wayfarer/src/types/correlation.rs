//! Snapping points onto the road network: the policy, the snapped locations and the candidates rejected.

use super::*;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum CorrelationMode {
    #[default]
    Indexed,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct CorrelationPolicy {
    pub max_distance_m: u32,
    pub candidate_limit: usize,
    /// The SAR `off_network` reach: when no connected candidate lies within
    /// the expanded radii (`EXPANDED_CORRELATION_DISTANCES_M`), the search
    /// widens to this distance too.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub off_network_max_distance_m: Option<u32>,
    /// The edges a SAR request's `legal_access` and `wrong_way` open, which
    /// endpoints may snap onto too. Without SAR, candidates are the edges
    /// the profile may legally use.
    #[serde(skip)]
    pub sar_open: Option<crate::sar::SarOpenEdges>,
}

impl Default for CorrelationPolicy {
    fn default() -> Self {
        Self {
            max_distance_m: DEFAULT_CORRELATION_DISTANCE_M as u32,
            candidate_limit: DEFAULT_CANDIDATE_LIMIT,
            off_network_max_distance_m: None,
            sar_open: None,
        }
    }
}

impl From<&RouteOptions> for CorrelationPolicy {
    fn from(options: &RouteOptions) -> Self {
        Self {
            max_distance_m: options.max_correlation_distance_m,
            candidate_limit: options.candidate_limit.min(MAX_CANDIDATE_LIMIT),
            off_network_max_distance_m: options
                .sar
                .as_ref()
                .and_then(crate::sar::RouteSarOptions::off_network_max_leg_m)
                .filter(|distance| distance.is_finite() && *distance > 0.0)
                .map(|distance| distance.ceil() as u32),
            // SAR is for vehicle profiles only (`sar::validate_request`).
            sar_open: crate::sar::SarOpenEdges::of(options),
        }
    }
}

/// The package's compact index of a correlated edge. Correlation sets it so
/// that plain routes, matrices and isochrones go from candidates to results
/// without the edge id lookup (built only for requests that name an
/// edge id). It is a cache of what `edge_id` determines: never serialized,
/// ignored by equality, and used only where the package's edge at that index
/// has `edge_id` (a location from another package falls back to the lookup).
#[derive(Debug, Clone, Copy, Default)]
pub struct CompactEdgeHint(pub(crate) Option<u32>);

impl CompactEdgeHint {
    pub(crate) fn of(raw_edge_index: usize) -> Self {
        Self(u32::try_from(raw_edge_index).ok())
    }
}

impl PartialEq for CompactEdgeHint {
    fn eq(&self, _: &Self) -> bool {
        true
    }
}

#[derive(Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct CorrelatedLocation {
    pub edge_id: u64,
    /// The compact index of `edge_id` in the package that correlated it
    /// (`Default::default()` when building a location by hand).
    #[doc(hidden)]
    #[serde(skip)]
    pub compact_edge: CompactEdgeHint,
    pub shared_edge_info_id: u64,
    #[serde(default)]
    pub opposing_edge_id: Option<u64>,
    #[serde(default)]
    pub road_group_key: Option<String>,
    #[serde(default)]
    pub same_road_candidate_count: usize,
    pub source_node_id: i64,
    pub target_node_id: i64,
    pub snapped_point: GeoPoint,
    pub distance_m: f64,
    pub fraction_along_edge: f64,
    pub heading_deg: Option<u16>,
    #[serde(default)]
    pub heading_delta_deg: Option<u16>,
    #[serde(default)]
    pub side: CorrelationSide,
    #[serde(default)]
    pub component_size: Option<usize>,
    #[serde(default)]
    pub outbound_reach_edge_count: Option<usize>,
    #[serde(default)]
    pub inbound_reach_edge_count: Option<usize>,
    #[serde(default)]
    pub layer: Option<i16>,
    #[serde(default)]
    pub level: Option<String>,
    #[serde(default)]
    pub score: f64,
    #[serde(default)]
    pub score_breakdown: CorrelationScoreBreakdown,
    pub confidence: f64,
}

// Debug leaves out the compact edge hint, a cache (see [`CompactEdgeHint`]).
impl std::fmt::Debug for CorrelatedLocation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CorrelatedLocation")
            .field("edge_id", &self.edge_id)
            .field("shared_edge_info_id", &self.shared_edge_info_id)
            .field("opposing_edge_id", &self.opposing_edge_id)
            .field("road_group_key", &self.road_group_key)
            .field("same_road_candidate_count", &self.same_road_candidate_count)
            .field("source_node_id", &self.source_node_id)
            .field("target_node_id", &self.target_node_id)
            .field("snapped_point", &self.snapped_point)
            .field("distance_m", &self.distance_m)
            .field("fraction_along_edge", &self.fraction_along_edge)
            .field("heading_deg", &self.heading_deg)
            .field("heading_delta_deg", &self.heading_delta_deg)
            .field("side", &self.side)
            .field("component_size", &self.component_size)
            .field("outbound_reach_edge_count", &self.outbound_reach_edge_count)
            .field("inbound_reach_edge_count", &self.inbound_reach_edge_count)
            .field("layer", &self.layer)
            .field("level", &self.level)
            .field("score", &self.score)
            .field("score_breakdown", &self.score_breakdown)
            .field("confidence", &self.confidence)
            .finish()
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct CorrelationScoreBreakdown {
    pub distance_score: f64,
    pub heading_score: f64,
    pub road_class_score: f64,
    pub access_score: f64,
    pub component_score: f64,
    pub reach_score: f64,
    pub side_score: f64,
    pub layer_score: f64,
    pub level_score: f64,
    pub candidate_type_score: f64,
    pub total_score: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum CorrelationRejectReason {
    InaccessibleProfile,
    SnapUnavailable,
    BeyondMaxDistance,
    HeadingOutsideTolerance,
    BelowMinComponentSize,
    BelowMinReach,
    LayerMismatch,
    LevelMismatch,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RejectedCorrelationCandidate {
    pub edge_id: Option<u64>,
    pub reason: CorrelationRejectReason,
    #[serde(default)]
    pub distance_millimeters: Option<u64>,
    #[serde(default)]
    pub heading_delta_deg: Option<u16>,
    #[serde(default)]
    pub component_size: Option<usize>,
    #[serde(default)]
    pub outbound_reach_edge_count: Option<usize>,
    #[serde(default)]
    pub inbound_reach_edge_count: Option<usize>,
    #[serde(default)]
    pub layer: Option<i16>,
    #[serde(default)]
    pub level: Option<String>,
}

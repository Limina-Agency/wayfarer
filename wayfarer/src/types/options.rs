//! Request options: costing and correlation settings, per-request edge blocks, avoidances, multipliers and speeds, alternatives, endpoint preferences.

use super::*;

pub(crate) const DEFAULT_CORRELATION_DISTANCE_M: f64 = 75.0;

pub(crate) const DEFAULT_CANDIDATE_LIMIT: usize = 8;

/// Candidates kept per endpoint at most: landmark bounds track up to 16
/// targets (more would drop them), and candidates beyond that within the
/// correlation distance add only far, unlikely snaps.
pub(crate) const MAX_CANDIDATE_LIMIT: usize = 16;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", default)]
pub struct RouteOptions {
    #[serde(alias = "max_correlation_distance_m")]
    pub max_correlation_distance_m: u32,
    /// Correlation candidates kept per endpoint (at most 16).
    #[serde(alias = "candidate_limit")]
    pub candidate_limit: usize,
    #[serde(alias = "blocked_edge_ids")]
    pub blocked_edge_ids: BTreeSet<u64>,
    #[serde(default, alias = "blocked_edge_ranges")]
    pub blocked_edge_ranges: Vec<RouteEdgeRange>,
    #[serde(alias = "avoided_edge_ids")]
    pub avoided_edge_ids: BTreeSet<u64>,
    #[serde(default, alias = "avoided_edge_ranges")]
    pub avoided_edge_ranges: Vec<RouteEdgeAvoidanceRange>,
    #[serde(default, alias = "warning_edge_ids")]
    pub warning_edge_ids: BTreeSet<u64>,
    #[serde(default, alias = "warning_edge_ranges")]
    pub warning_edge_ranges: Vec<RouteEdgeRange>,
    #[serde(default, alias = "edge_cost_multipliers")]
    pub edge_cost_multipliers: BTreeMap<u64, f64>,
    #[serde(default, alias = "edge_cost_multiplier_ranges")]
    pub edge_cost_multiplier_ranges: Vec<RouteEdgeCostMultiplierRange>,
    /// Speed overrides (km/h) on whole edges: the edge's travel time becomes
    /// its length at that speed, and its cost follows (cost is time times
    /// the edge's factors). Faster than the profile's own speed is a
    /// discount; it is costed exactly, with bounds scaled by the least
    /// cost ratio.
    #[serde(
        default,
        alias = "edge_speed_overrides",
        skip_serializing_if = "BTreeMap::is_empty"
    )]
    pub edge_speed_overrides: BTreeMap<u64, f64>,
    /// Speed overrides on stretches of edges: the stretch takes its length
    /// at the given speed, the rest of the edge its own time.
    #[serde(
        default,
        alias = "edge_speed_override_ranges",
        skip_serializing_if = "Vec::is_empty"
    )]
    pub edge_speed_override_ranges: Vec<RouteEdgeSpeedOverrideRange>,
    /// Edges the request opens for its profile (an overlay `open`): costed
    /// like an accessible edge of the profile even where legal access, a
    /// destination-only rule or a conditional closure closes it. A block
    /// still wins. Endpoints do not snap onto edges the profile cannot use.
    #[serde(
        default,
        alias = "opened_edge_ids",
        skip_serializing_if = "BTreeSet::is_empty"
    )]
    pub opened_edge_ids: BTreeSet<u64>,
    /// Search-and-rescue relaxations (`crate::sar`); off when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sar: Option<crate::sar::RouteSarOptions>,
    /// Budget for one search: the most out-edges it may examine before it
    /// stops with `SearchBudgetExceeded`. A route is one search. A matrix
    /// applies it to each search separately, never to the request as a
    /// whole: to each cell when cells are searched pairwise (on the compact
    /// graph, up to 12 searchable targets on foot and 100 otherwise), and to
    /// each source row's shared expansion otherwise (rows above that cut-off;
    /// each backward and forward search with buckets). Cells a stopped search
    /// had not reached fail with
    /// `search_budget_exceeded`; targets a stopped row had already reached
    /// keep their best path so far. Isochrones ignore it and use
    /// `RouteIsochroneRequest::max_expansion_edge_count`.
    #[serde(default, alias = "max_visited_edge_count")]
    pub max_visited_edge_count: Option<usize>,
    #[serde(default)]
    pub costing: RouteCostingOptions,
    #[serde(default)]
    pub alternatives: RouteAlternativePolicy,
    #[serde(
        default,
        skip_serializing_if = "Option::is_none",
        alias = "dynamic_overlay"
    )]
    pub dynamic_overlay: Option<RouteDynamicOverlay>,
    /// Cancel tokens and a deadline that stop the request's searches, checked
    /// every 1024 visited edges and before and after each search. A route or
    /// isochrone then fails with `RouteError::Cancelled`; a matrix fails each
    /// searchable cell with `cancelled` (or `deadline_exceeded`). Not part of
    /// the JSON form.
    #[serde(skip)]
    pub cancellation: RouteCancellation,
}

impl Default for RouteOptions {
    fn default() -> Self {
        Self {
            max_correlation_distance_m: DEFAULT_CORRELATION_DISTANCE_M as u32,
            candidate_limit: DEFAULT_CANDIDATE_LIMIT,
            blocked_edge_ids: BTreeSet::new(),
            blocked_edge_ranges: Vec::new(),
            avoided_edge_ids: BTreeSet::new(),
            avoided_edge_ranges: Vec::new(),
            warning_edge_ids: BTreeSet::new(),
            warning_edge_ranges: Vec::new(),
            edge_cost_multipliers: BTreeMap::new(),
            edge_cost_multiplier_ranges: Vec::new(),
            edge_speed_overrides: BTreeMap::new(),
            edge_speed_override_ranges: Vec::new(),
            opened_edge_ids: BTreeSet::new(),
            sar: None,
            max_visited_edge_count: None,
            costing: RouteCostingOptions::default(),
            alternatives: RouteAlternativePolicy::default(),
            dynamic_overlay: None,
            cancellation: RouteCancellation::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteEdgeRange {
    pub edge_id: u64,
    pub start_fraction: f64,
    pub end_fraction: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteEdgeAvoidanceRange {
    pub edge_id: u64,
    pub start_fraction: f64,
    pub end_fraction: f64,
}

impl RouteEdgeAvoidanceRange {
    pub fn clipped_to(self, start_fraction: f64, end_fraction: f64) -> Option<Self> {
        let start = self
            .start_fraction
            .clamp(0.0, 1.0)
            .max(start_fraction.clamp(0.0, 1.0));
        let end = self
            .end_fraction
            .clamp(0.0, 1.0)
            .min(end_fraction.clamp(0.0, 1.0));
        (end > start).then_some(Self {
            edge_id: self.edge_id,
            start_fraction: start,
            end_fraction: end,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteEdgeCostMultiplierRange {
    pub edge_id: u64,
    pub start_fraction: f64,
    pub end_fraction: f64,
    pub multiplier: f64,
}

impl RouteEdgeCostMultiplierRange {
    pub fn clipped_to(self, start_fraction: f64, end_fraction: f64) -> Option<Self> {
        let start = self
            .start_fraction
            .clamp(0.0, 1.0)
            .max(start_fraction.clamp(0.0, 1.0));
        let end = self
            .end_fraction
            .clamp(0.0, 1.0)
            .min(end_fraction.clamp(0.0, 1.0));
        (end > start && self.multiplier.is_finite() && self.multiplier > 0.0).then_some(Self {
            edge_id: self.edge_id,
            start_fraction: start,
            end_fraction: end,
            multiplier: self.multiplier,
        })
    }
}

/// A speed override (km/h) on a stretch of an edge
/// (`RouteOptions::edge_speed_override_ranges`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteEdgeSpeedOverrideRange {
    pub edge_id: u64,
    pub start_fraction: f64,
    pub end_fraction: f64,
    #[serde(alias = "speed_kph")]
    pub speed_kph: f64,
}

impl RouteEdgeRange {
    pub fn clipped_to(self, start_fraction: f64, end_fraction: f64) -> Option<Self> {
        let start = self
            .start_fraction
            .clamp(0.0, 1.0)
            .max(start_fraction.clamp(0.0, 1.0));
        let end = self
            .end_fraction
            .clamp(0.0, 1.0)
            .min(end_fraction.clamp(0.0, 1.0));
        (end > start).then_some(Self {
            edge_id: self.edge_id,
            start_fraction: start,
            end_fraction: end,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteAlternativePolicy {
    #[serde(default)]
    pub requested_alternative_count: usize,
    #[serde(default = "default_max_shared_edge_ratio")]
    pub max_shared_edge_ratio: f64,
    #[serde(default = "default_max_alternative_cost_ratio")]
    pub max_cost_ratio: f64,
    #[serde(default)]
    pub minimum_warning_difference: usize,
}

impl Default for RouteAlternativePolicy {
    fn default() -> Self {
        Self {
            requested_alternative_count: 0,
            max_shared_edge_ratio: default_max_shared_edge_ratio(),
            max_cost_ratio: default_max_alternative_cost_ratio(),
            minimum_warning_difference: 0,
        }
    }
}

impl RouteAlternativePolicy {
    pub fn normalized(self) -> Self {
        Self {
            requested_alternative_count: self.requested_alternative_count.min(3),
            max_shared_edge_ratio: bounded_unit_or(self.max_shared_edge_ratio, 0.8),
            max_cost_ratio: if self.max_cost_ratio.is_finite() {
                self.max_cost_ratio.clamp(1.0, 3.0)
            } else {
                default_max_alternative_cost_ratio()
            },
            minimum_warning_difference: self.minimum_warning_difference,
        }
    }
}

pub(crate) fn default_max_shared_edge_ratio() -> f64 {
    0.8
}

pub(crate) fn default_max_alternative_cost_ratio() -> f64 {
    1.6
}

fn bounded_unit_or(value: f64, fallback: f64) -> f64 {
    if value.is_finite() {
        value.clamp(0.0, 1.0)
    } else {
        fallback
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum CorrelationSide {
    Left,
    Right,
    OnEdge,
    #[default]
    Unknown,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum PreferredRoadSide {
    Left,
    Right,
    #[default]
    Either,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum CorrelationRoadClassPreference {
    Motorway,
    Primary,
    Secondary,
    Tertiary,
    Residential,
    Track,
    Path,
    Other,
}

impl CorrelationRoadClassPreference {
    pub fn matches(self, road_type: RoadType) -> bool {
        matches!(
            (self, road_type),
            (Self::Motorway, RoadType::Motorway)
                | (Self::Primary, RoadType::Primary)
                | (Self::Secondary, RoadType::Secondary)
                | (Self::Tertiary, RoadType::Tertiary)
                | (Self::Residential, RoadType::Residential)
                | (Self::Track, RoadType::Track)
                | (Self::Path, RoadType::Path)
                | (Self::Other, RoadType::Other)
        )
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteEndpointOptions {
    #[serde(default, alias = "heading_deg")]
    pub heading_deg: Option<u16>,
    #[serde(default, alias = "heading_tolerance_deg")]
    pub heading_tolerance_deg: Option<u16>,
    #[serde(default, alias = "preferred_side")]
    pub preferred_side: PreferredRoadSide,
    #[serde(default, alias = "min_component_size")]
    pub min_component_size: Option<usize>,
    #[serde(default, alias = "min_outbound_reach_edge_count")]
    pub min_outbound_reach_edge_count: Option<usize>,
    #[serde(default, alias = "min_inbound_reach_edge_count")]
    pub min_inbound_reach_edge_count: Option<usize>,
    #[serde(default, alias = "preferred_layer")]
    pub preferred_layer: Option<i16>,
    #[serde(default, alias = "preferred_level")]
    pub preferred_level: Option<String>,
    #[serde(default, alias = "require_layer_match")]
    pub require_layer_match: bool,
    #[serde(default, alias = "require_level_match")]
    pub require_level_match: bool,
    #[serde(default, alias = "preferred_road_classes")]
    pub preferred_road_classes: Vec<CorrelationRoadClassPreference>,
    #[serde(default, alias = "avoided_road_classes")]
    pub avoided_road_classes: Vec<CorrelationRoadClassPreference>,
    #[serde(default, alias = "preferred_route_refs")]
    pub preferred_route_refs: Vec<String>,
    #[serde(default, alias = "prefer_any_route_ref")]
    pub prefer_any_route_ref: bool,
    #[serde(default, alias = "avoid_service_roads")]
    pub avoid_service_roads: bool,
    #[serde(default, alias = "avoid_private_access")]
    pub avoid_private_access: bool,
}

impl RouteEndpointOptions {
    pub fn with_heading(mut self, heading_deg: u16) -> Self {
        self.heading_deg = Some(heading_deg % 360);
        self
    }

    pub fn with_heading_tolerance(mut self, heading_tolerance_deg: u16) -> Self {
        self.heading_tolerance_deg = Some(heading_tolerance_deg.min(180));
        self
    }

    pub fn with_preferred_side(mut self, preferred_side: PreferredRoadSide) -> Self {
        self.preferred_side = preferred_side;
        self
    }

    pub fn with_min_component_size(mut self, min_component_size: usize) -> Self {
        self.min_component_size = Some(min_component_size);
        self
    }

    pub fn with_min_outbound_reach_edge_count(mut self, min_reach_edge_count: usize) -> Self {
        self.min_outbound_reach_edge_count = Some(min_reach_edge_count);
        self
    }

    pub fn with_min_inbound_reach_edge_count(mut self, min_reach_edge_count: usize) -> Self {
        self.min_inbound_reach_edge_count = Some(min_reach_edge_count);
        self
    }

    pub fn with_preferred_layer(mut self, layer: i16) -> Self {
        self.preferred_layer = Some(layer);
        self
    }

    pub fn with_preferred_level(mut self, level: impl Into<String>) -> Self {
        self.preferred_level = Some(level.into());
        self
    }

    pub fn requiring_layer_match(mut self) -> Self {
        self.require_layer_match = true;
        self
    }

    pub fn requiring_level_match(mut self) -> Self {
        self.require_level_match = true;
        self
    }

    pub fn with_preferred_road_class(mut self, road_class: CorrelationRoadClassPreference) -> Self {
        self.preferred_road_classes.push(road_class);
        self
    }

    pub fn with_avoided_road_class(mut self, road_class: CorrelationRoadClassPreference) -> Self {
        self.avoided_road_classes.push(road_class);
        self
    }

    pub fn with_preferred_route_ref(mut self, route_ref: impl Into<String>) -> Self {
        let route_ref = route_ref.into();
        if !route_ref.trim().is_empty() {
            self.preferred_route_refs.push(route_ref);
        }
        self
    }

    pub fn with_prefer_any_route_ref(mut self) -> Self {
        self.prefer_any_route_ref = true;
        self
    }

    pub fn with_avoid_service_roads(mut self) -> Self {
        self.avoid_service_roads = true;
        self
    }

    pub fn with_avoid_private_access(mut self) -> Self {
        self.avoid_private_access = true;
        self
    }

    pub(crate) fn has_scoring_hints(&self) -> bool {
        self.heading_deg.is_some()
            || self.preferred_side != PreferredRoadSide::Either
            || self.min_component_size.is_some()
            || self.min_outbound_reach_edge_count.is_some()
            || self.min_inbound_reach_edge_count.is_some()
            || self.preferred_layer.is_some()
            || self.preferred_level.is_some()
            || !self.preferred_road_classes.is_empty()
            || !self.avoided_road_classes.is_empty()
            || !self.preferred_route_refs.is_empty()
            || self.prefer_any_route_ref
            || self.avoid_service_roads
            || self.avoid_private_access
    }

    pub(crate) fn relaxed_for_retry(&self) -> Self {
        Self::default()
    }
}

#[cfg(test)]
mod option_casing_tests {
    use super::*;

    /// The option types are camelCase and still read their former
    /// snake_case names.
    #[test]
    fn options_accept_snake_case_aliases() {
        let snake: RouteOptions = serde_json::from_str(
            r#"{"max_correlation_distance_m": 40, "candidate_limit": 3,
                "blocked_edge_ids": [7], "max_visited_edge_count": 900,
                "edge_cost_multipliers": {"8": 2.0}}"#,
        )
        .unwrap();
        let camel: RouteOptions = serde_json::from_str(
            r#"{"maxCorrelationDistanceM": 40, "candidateLimit": 3,
                "blockedEdgeIds": [7], "maxVisitedEdgeCount": 900,
                "edgeCostMultipliers": {"8": 2.0}}"#,
        )
        .unwrap();
        assert_eq!(snake, camel);
        assert_eq!(camel.max_visited_edge_count, Some(900));
        // Serialized names are camelCase.
        let text = serde_json::to_string(&camel).unwrap();
        assert!(text.contains("\"maxVisitedEdgeCount\":900"), "{text}");

        let snake: RouteEndpointOptions = serde_json::from_str(
            r#"{"heading_deg": 90, "min_outbound_reach_edge_count": 5,
                "avoid_service_roads": true}"#,
        )
        .unwrap();
        let camel: RouteEndpointOptions = serde_json::from_str(
            r#"{"headingDeg": 90, "minOutboundReachEdgeCount": 5,
                "avoidServiceRoads": true}"#,
        )
        .unwrap();
        assert_eq!(snake, camel);
        assert_eq!(camel.heading_deg, Some(90));
    }
}

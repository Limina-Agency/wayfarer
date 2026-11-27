//! Routes through two or more locations: [`RoutingEngine::route`].
//!
//! A request lists its locations, how each leg between them is travelled
//! (on the road network, or as a straight line for off-network stretches)
//! and the overlays (closures, slow-go areas, hazards, discounts, openings)
//! that apply to it, as geometry or as package edges. Each routed leg is one
//! search ([`RoutingEngine::route_leg`]); a waypoint is snapped once, so the
//! leg leaving it starts where the previous leg arrived, facing on. The legs
//! are stitched into one result: geometry, maneuvers (a waypoint arrival
//! between legs), costs, recosts, the overlays the route crosses and a short
//! explanation of what shaped it.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

use crate::direct_leg::{DirectRouteLeg, direct_route_leg};
use crate::geometry::{
    GeoJsonLineString, GeometryFormat, PolylineError, decode_polyline6, encode_into,
};
use crate::overlay::project_route_overlay_geometry;
use crate::sar::{RouteQualifiedAccess, RouteSarReport};
use crate::{
    CorrelatedLocation, GeoPoint, LegRequest, LegResult, MobilityProfile, RouteAlternative,
    RouteCostDriverKind, RouteCostDriverSummary, RouteCostSummary, RouteDiagnostics,
    RouteDynamicOverlay, RouteEdgeAttribution, RouteEdgeCostMultiplierRange, RouteEdgeRange,
    RouteEndpointOptions, RouteError, RouteFailure, RouteManeuver, RouteManeuverKind, RouteOptions,
    RouteOverlayDiagnostics, RouteOverlayGeometryKind, RoutePolicySummary, RouteRecostSummary,
    RouteShapeQualityFlag, RouteShapeSegment, RoutingEngine,
};

pub const MAX_ROUTE_LOCATIONS: usize = 34;
pub const MAX_ROUTE_OVERLAYS: usize = 256;
pub const MAX_ROUTE_OVERLAY_POINTS: usize = 100_000;
pub const MAX_ROUTE_PROVENANCE_VALUES: usize = 4_096;
pub const MAX_ROUTE_STRING_BYTES: usize = 256;
pub const MAX_ROUTE_TOTAL_STRING_BYTES: usize = 1_048_576;
pub const MAX_ROUTE_OUTPUT_GEOMETRY_POINTS: usize = 1_000_000;
pub const MAX_ROUTE_OUTPUT_EDGE_IDS: usize = 1_000_000;
pub const MAX_ROUTE_OUTPUT_MANEUVERS: usize = 100_000;
pub const MAX_ROUTE_OUTPUT_RECOSTS: usize = 16;
pub const MAX_ROUTE_LOCATION_HINTS: usize = 4_096;
pub const MAX_ROUTE_PROJECTION_DISTANCE_M: u32 = 100_000;
/// The fastest speed override (km/h) an overlay may set.
pub const MAX_ROUTE_OVERLAY_SPEED_KPH: f64 = 300.0;
/// How far a leg leaving a waypoint may depart from the direction the previous
/// leg arrived in, when the waypoint itself sets no heading.
pub const WAYPOINT_CONTINUITY_HEADING_TOLERANCE_DEG: u16 = 45;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteLocation {
    pub point: GeoPoint,
    #[serde(default)]
    pub options: RouteEndpointOptions,
}

impl RouteLocation {
    pub fn new(point: GeoPoint) -> Self {
        Self {
            point,
            options: RouteEndpointOptions::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteLegMode {
    #[default]
    Routed,
    Direct,
}

/// What an overlay does to the edges it covers.
/// Inside one request: a block wins over a discount and a slow-go; an
/// `open` wins over legal closure and over a block whose `valid_from_ms` is
/// earlier than its own (an absent one is earliest); overlapping discounts
/// multiply, slow-go factors take the largest, and of several speed
/// overrides on one edge the slowest stands.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteOverlayPolicy {
    /// Blocked.
    NoGo,
    /// Dearer: `cost_multiplier` above 1 (default 2), cost only.
    SlowGo,
    /// A hazard warning on the route; costs are unchanged.
    Warning,
    /// Cheaper: `cost_multiplier` in (0, 1) scales cost and duration, and
    /// `speed_kph` sets the speed (cost follows the time). Costed exactly;
    /// goal-directed bounds are scaled by the least cost ratio.
    Discount,
    /// Usable by the request's profile (when `profiles` lists it, or lists
    /// none) whatever its legal access says, at `speed_kph` if given. The
    /// edge must be in the package; endpoints do not snap onto it.
    Open,
}

/// Where an overlay came from, when it came from an overlay set.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteOverlaySource {
    pub set_id: String,
    pub set_version: u64,
    pub item_id: String,
    pub item_version: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub author: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteOverlay {
    pub geometry_kind: RouteOverlayGeometryKind,
    pub points: Vec<GeoPoint>,
    pub policy: RouteOverlayPolicy,
    pub projection_distance_m: u32,
    #[serde(default)]
    pub cost_multiplier: Option<f64>,
    /// `discount` and `open`: the speed (km/h) on the covered edges.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed_kph: Option<f64>,
    /// The profiles the overlay applies to; empty: every profile.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub profiles: Vec<MobilityProfile>,
    /// The overlay is active from this time (UTC ms) on; absent: always.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub valid_from_ms: Option<i64>,
    /// The overlay is active before this time (UTC ms); absent: always.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub valid_to_ms: Option<i64>,
    /// Explicit edges (an overlay set bound to this package): used instead
    /// of projecting `points`, which may then be empty.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub edges: Vec<RouteEdgeRange>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<RouteOverlaySource>,
}

impl RouteOverlay {
    /// A geometry overlay with no time window, profile list or source.
    pub fn new(
        geometry_kind: RouteOverlayGeometryKind,
        points: Vec<GeoPoint>,
        policy: RouteOverlayPolicy,
        projection_distance_m: u32,
    ) -> Self {
        Self {
            geometry_kind,
            points,
            policy,
            projection_distance_m,
            cost_multiplier: None,
            speed_kph: None,
            profiles: Vec::new(),
            valid_from_ms: None,
            valid_to_ms: None,
            edges: Vec::new(),
            source: None,
        }
    }

    /// Whether the overlay is active at `time_ms` (its window holds it).
    pub fn active_at(&self, time_ms: i64) -> bool {
        self.valid_from_ms.is_none_or(|from| from <= time_ms)
            && self.valid_to_ms.is_none_or(|to| time_ms < to)
    }

    fn has_window(&self) -> bool {
        self.valid_from_ms.is_some() || self.valid_to_ms.is_some()
    }

    /// The overlay's label in reports: its set item id, else its index.
    fn label(&self, index: usize) -> String {
        self.source.as_ref().map_or_else(
            || format!("overlay[{index}]"),
            |source| source.item_id.clone(),
        )
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteSegment {
    Routed(Box<LegResult>),
    Direct(Box<DirectRouteLeg>),
}

impl RouteSegment {
    pub fn as_routed(&self) -> Option<&LegResult> {
        match self {
            Self::Routed(route) => Some(route),
            Self::Direct(_) => None,
        }
    }

    /// The leg's points; empty when the request asked for an encoded
    /// `geometryFormat`.
    pub fn geometry(&self) -> &[GeoPoint] {
        match self {
            Self::Routed(route) => &route.geometry,
            Self::Direct(leg) => &leg.geometry,
        }
    }

    /// The leg's length.
    pub fn distance_m(&self) -> f64 {
        match self {
            Self::Routed(route) => route.distance_m,
            Self::Direct(leg) => leg.distance_m,
        }
    }

    /// The leg's travel time.
    pub fn duration_s(&self) -> f64 {
        match self {
            Self::Routed(route) => route.duration_s,
            Self::Direct(leg) => leg.duration_s,
        }
    }

    /// The leg's cost (a direct leg's is its duration).
    pub fn cost(&self) -> f64 {
        match self {
            Self::Routed(route) => route.cost,
            Self::Direct(leg) => leg.duration_s,
        }
    }

    fn maneuvers(&self) -> &[RouteManeuver] {
        match self {
            Self::Routed(route) => &route.maneuvers,
            Self::Direct(leg) => std::slice::from_ref(&leg.maneuver),
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteSummaryDiagnostics {
    pub segment_count: usize,
    pub routed_segment_count: usize,
    pub direct_segment_count: usize,
    pub visited_edge_count: usize,
    pub settled_node_count: usize,
    pub projected_overlay_count: usize,
    pub mobility_adjustment_count: usize,
    pub inaccessible_edge_count: usize,
    pub destination_only_penalty_count: usize,
    pub ford_penalty_count: usize,
    pub toll_penalty_count: usize,
    pub ferry_penalty_count: usize,
    pub gate_penalty_count: usize,
    pub access_restriction_penalty_count: usize,
    pub overlay_multiplier_count: usize,
    pub warning_edge_count: usize,
    pub ssvg_partial_edge_fragment_used_count: usize,
    pub ssvg_connector_used_count: usize,
    pub ssvg_endpoint_connector_used_count: usize,
    pub ssvg_road_entry_connector_used_count: usize,
    pub ssvg_road_bypass_connector_used_count: usize,
    pub ssvg_cross_country_connector_used_count: usize,
    pub origin_candidate_count: usize,
    pub destination_candidate_count: usize,
    pub rejected_candidate_count: usize,
    pub origin_rejected_candidate_count: usize,
    pub destination_rejected_candidate_count: usize,
    pub origin_best_candidate_score_basis_points: Option<u32>,
    pub destination_best_candidate_score_basis_points: Option<u32>,
    /// Waypoints whose outgoing leg starts away from where the incoming one
    /// ended (each such leg's first shape segment is flagged `WaypointGap`).
    pub waypoint_gap_count: usize,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteAlternatives {
    NotRequested,
    Computed(Vec<RouteAlternative>),
    UnavailableForMultiLeg,
    UnavailableForDirectLeg,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteSegmentShape {
    pub route_segment_index: usize,
    pub segment: RouteShapeSegment,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteExplanationSeverity {
    Info,
    Cost,
    Warning,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteExplanationKind {
    DominantCostDriver(RouteCostDriverKind),
    DynamicOverlay,
    MobilityPolicy,
    SpeedPolicy,
    ConditionalRestrictions,
    AccessConstraints,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteExplanation {
    pub kind: RouteExplanationKind,
    pub severity: RouteExplanationSeverity,
    pub edge_count: Option<usize>,
    pub value: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteResult {
    /// One per leg, in order: a routed leg's full single-leg result, or a
    /// direct leg.
    pub segments: Vec<RouteSegment>,
    /// How each of the request's locations snapped onto the road network,
    /// from the leg arriving at it and the leg leaving it.
    pub location_matches: Vec<RouteLocationMatch>,
    pub edge_ids: Vec<u64>,
    pub geometry: Vec<GeoPoint>,
    /// `geometry` as an encoded polyline at precision 6, when the request
    /// asks for `geometryFormat: "polyline6"`; `geometry` is then empty.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub polyline6: Option<String>,
    /// `geometry` as a GeoJSON `LineString`, when the request asks for
    /// `geometryFormat: "geojson"`; `geometry` is then empty.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub geojson: Option<GeoJsonLineString>,
    pub distance_m: f64,
    pub duration_s: f64,
    pub cost: f64,
    pub maneuvers: Vec<RouteManeuver>,
    pub recosts: Vec<RouteRecostSummary>,
    pub edge_attributions: Vec<RouteEdgeAttribution>,
    pub shape_segments: Vec<RouteSegmentShape>,
    pub alternatives: RouteAlternatives,
    pub policy_summary: Option<RoutePolicySummary>,
    pub cost_summary: RouteCostSummary,
    pub explanations: Vec<RouteExplanation>,
    pub overlay_diagnostics: RouteOverlayDiagnostics,
    /// Counts over the whole request (unstable: diagnostics carry no
    /// compatibility promise).
    pub diagnostics: RouteSummaryDiagnostics,
    /// Overlays the route uses: every applied overlay with at least one
    /// route edge, with its set item id and versions when it has them.
    pub overlay_uses: Vec<RouteOverlayUse>,
    /// Overlay time windows (`RouteOverlay::valid_from_ms`), when
    /// any overlay has one.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub overlay_windows: Option<RouteOverlayWindows>,
    /// Qualified legal access along every routed leg, in route order (see
    /// `LegResult::qualified_access`).
    pub qualified_access: Vec<RouteQualifiedAccess>,
    /// The SAR relaxations the routed legs used, merged; SAR requests only.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sar: Option<RouteSarReport>,
}

impl RouteResult {
    /// The route's points: `geometry`, or decoded from the `polyline6` or
    /// `geojson` that replaced it.
    pub fn points(&self) -> Result<Vec<GeoPoint>, PolylineError> {
        match (&self.polyline6, &self.geojson) {
            (Some(encoded), _) if self.geometry.is_empty() => decode_polyline6(encoded),
            (None, Some(line)) if self.geometry.is_empty() => Ok(line.to_points()),
            _ => Ok(self.geometry.clone()),
        }
    }
}

/// One overlay the route crosses.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteOverlayUse {
    /// The overlay's index in the request's overlays (overlay-set items
    /// follow them).
    pub overlay_index: usize,
    pub label: String,
    pub policy: RouteOverlayPolicy,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<RouteOverlaySource>,
    pub route_edge_count: usize,
    pub route_length_m: f64,
}

/// What the time windows of a request's overlays did.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteOverlayWindows {
    /// The time the windows were evaluated at (UTC ms).
    pub evaluated_at_ms: i64,
    /// Overlays outside their window, by label: not applied.
    pub inactive_overlay_items: Vec<String>,
    /// Applied overlays whose window ends before the estimated arrival
    /// (evaluation time plus the route's duration). The search is static: it
    /// does not see an item expire en route.
    pub overlay_window_ends_within_route: bool,
    pub ending_within_route: Vec<String>,
}

/// Correlation evidence for a stop from both adjacent routed legs.
///
/// Keeping both sides avoids the old direct-to-routed gap where a waypoint was
/// considered unsnapped merely because its incoming leg was direct, even though
/// its outgoing routed leg successfully correlated it.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteLocationMatch {
    pub incoming: Option<CorrelatedLocation>,
    pub outgoing: Option<CorrelatedLocation>,
}

impl RouteLocationMatch {
    pub fn preferred(&self) -> Option<&CorrelatedLocation> {
        self.incoming.as_ref().or(self.outgoing.as_ref())
    }
}

/// A route through two or more locations.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteRequest {
    /// The origin, any waypoints in order, and the destination (2 to
    /// [`MAX_ROUTE_LOCATIONS`]).
    pub locations: Vec<RouteLocation>,
    pub profile: MobilityProfile,
    #[serde(default)]
    pub options: RouteOptions,
    /// How each leg is travelled, one per pair of consecutive locations;
    /// empty: every leg is routed.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub legs: Vec<RouteLegMode>,
    /// Closures and other overlays, as geometry or package edges.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub overlays: Vec<RouteOverlay>,
    /// When the trip departs (UTC ms): the time overlay windows are
    /// evaluated at, unless the costing options set
    /// `conditional_restriction_eval_at_ms`; absent both, the time of the
    /// request.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub departure_at_ms: Option<i64>,
    /// Dynamic overlays (split edges, connectors) for one routed leg only,
    /// keyed by leg index, so a leg's connectors are not exposed to the
    /// others' endpoints.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub leg_dynamic_overlays: BTreeMap<usize, RouteDynamicOverlay>,
    /// How the result carries its geometry: as `points` (the default), or
    /// encoded as `polyline6` or `geojson` wherever the result has one
    /// (the route, each segment, leg, alternative and direct leg), with
    /// `geometry` left empty there.
    #[serde(default, skip_serializing_if = "GeometryFormat::is_points")]
    pub geometry_format: GeometryFormat,
}

impl RouteRequest {
    /// A routed trip from `origin` to `destination`.
    pub fn new(origin: GeoPoint, destination: GeoPoint, profile: MobilityProfile) -> Self {
        Self::through([origin, destination], profile)
    }

    /// A routed trip through `points` in order (origin, waypoints,
    /// destination).
    pub fn through(points: impl IntoIterator<Item = GeoPoint>, profile: MobilityProfile) -> Self {
        Self {
            locations: points.into_iter().map(RouteLocation::new).collect(),
            profile,
            options: RouteOptions::default(),
            legs: Vec::new(),
            overlays: Vec::new(),
            departure_at_ms: None,
            leg_dynamic_overlays: BTreeMap::new(),
            geometry_format: GeometryFormat::Points,
        }
    }

    /// Adds an overlay.
    pub fn with_overlay(mut self, overlay: RouteOverlay) -> Self {
        self.overlays.push(overlay);
        self
    }

    /// How leg `index` is travelled.
    pub fn leg_mode(&self, index: usize) -> RouteLegMode {
        self.legs.get(index).copied().unwrap_or_default()
    }
}

fn invalid(reason: &'static str) -> RouteFailure {
    RouteFailure::new(
        RouteError::InvalidRequest { reason },
        RouteDiagnostics::default(),
    )
}

fn too_large(what: &'static str) -> RouteFailure {
    RouteFailure::new(
        RouteError::RequestTooLarge { what },
        RouteDiagnostics::default(),
    )
}

/// [`RoutingEngine::route`].
pub(crate) fn route(
    engine: &RoutingEngine,
    request: RouteRequest,
) -> Result<RouteResult, RouteFailure> {
    validate_request(&request)?;
    validate_sar(engine, &request.options, request.profile)?;
    let geometry_format = request.geometry_format;
    let leg_count = request.locations.len() - 1;
    let legs = (0..leg_count)
        .map(|index| request.leg_mode(index))
        .collect::<Vec<_>>();
    let RouteRequest {
        locations,
        profile,
        mut options,
        overlays,
        departure_at_ms,
        leg_dynamic_overlays,
        ..
    } = request;
    // Overlay windows are evaluated at one time for the whole request.
    let evaluated_at_ms = overlays.iter().any(RouteOverlay::has_window).then(|| {
        options
            .costing
            .conditional_restriction_eval_at_ms
            .or(departure_at_ms)
            .unwrap_or_else(now_ms)
    });
    let alternatives_requested = options.alternatives.requested_alternative_count > 0;
    if legs.len() > 1 || legs.contains(&RouteLegMode::Direct) {
        options.alternatives.requested_alternative_count = 0;
    }

    let mut overlay_diagnostics = RouteOverlayDiagnostics {
        projection_backend: Some(
            engine
                .route_graph_view()
                .overlay_projection_backend()
                .to_string(),
        ),
        ..RouteOverlayDiagnostics::default()
    };
    let applied = apply_overlays(
        engine,
        &overlays,
        profile,
        evaluated_at_ms,
        &mut options,
        &mut overlay_diagnostics,
    );
    validate_options(&options)?;

    let mut segments = Vec::<Option<RouteSegment>>::with_capacity(legs.len());
    for (segment_index, leg) in legs.iter().copied().enumerate() {
        if leg == RouteLegMode::Direct {
            segments.push(None);
            continue;
        }
        let mut leg_options = options.clone();
        if let Some(overlay) = leg_dynamic_overlays.get(&segment_index) {
            merge_dynamic_overlay(&mut leg_options.dynamic_overlay, overlay.clone());
            validate_options(&leg_options)?;
        }
        let request = LegRequest {
            origin: locations[segment_index].point,
            destination: locations[segment_index + 1].point,
            profile,
            options: leg_options,
            origin_options: locations[segment_index].options.clone(),
            destination_options: locations[segment_index + 1].options.clone(),
        };
        // A waypoint is snapped once: the leg leaving it starts where the leg
        // arriving at it ended, facing on, so it neither lands on another edge
        // nor turns back unannounced. When that cannot route, the waypoint is
        // snapped afresh (and any gap shows in the stitched result). A
        // waypoint reached by a costed access stretch is left the same way,
        // by a stretch of its own (`crate::access`).
        let continued = segment_index
            .checked_sub(1)
            .and_then(|previous| segments[previous].as_ref())
            .and_then(RouteSegment::as_routed)
            .filter(|previous| previous.destination.access.is_none())
            .map(|previous| continuing_request(&request, &previous.destination))
            .and_then(|continued| engine.route_leg(continued).ok());
        let route = match continued {
            Some(route) => Ok(route),
            None => engine.route_leg(request),
        }
        .map_err(|mut failure| {
            failure.segment_index = Some(segment_index);
            failure
        })?;
        segments.push(Some(RouteSegment::Routed(Box::new(route))));
    }

    for index in 0..segments.len() {
        if segments[index].is_some() {
            continue;
        }
        let origin = index
            .checked_sub(1)
            .and_then(|previous| segments[previous].as_ref())
            .and_then(|segment| segment.geometry().last().copied())
            .unwrap_or(locations[index].point);
        let destination = segments
            .get(index + 1)
            .and_then(Option::as_ref)
            .and_then(|segment| segment.geometry().first().copied())
            .unwrap_or(locations[index + 1].point);
        segments[index] = Some(RouteSegment::Direct(Box::new(direct_route_leg(
            origin,
            destination,
            profile,
        ))));
    }
    let segments = segments.into_iter().flatten().collect::<Vec<_>>();
    let mut result = assemble_result(segments, overlay_diagnostics, alternatives_requested)?;
    result.overlay_uses = overlay_uses(&applied, &overlays, &result.edge_attributions);
    result.overlay_windows = evaluated_at_ms.map(|evaluated_at_ms| {
        let arrival_ms = evaluated_at_ms.saturating_add((result.duration_s * 1000.0).ceil() as i64);
        let ending_within_route = applied
            .iter()
            .filter(|overlay| {
                overlays[overlay.index]
                    .valid_to_ms
                    .is_some_and(|to| to < arrival_ms)
            })
            .map(|overlay| overlays[overlay.index].label(overlay.index))
            .collect::<Vec<_>>();
        RouteOverlayWindows {
            evaluated_at_ms,
            inactive_overlay_items: overlays
                .iter()
                .enumerate()
                .filter(|(_, overlay)| !overlay.active_at(evaluated_at_ms))
                .map(|(index, overlay)| overlay.label(index))
                .collect(),
            overlay_window_ends_within_route: !ending_within_route.is_empty(),
            ending_within_route,
        }
    });
    encode_geometry(&mut result, geometry_format);
    Ok(result)
}

fn now_ms() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |elapsed| elapsed.as_millis() as i64)
}

/// The request's SAR option against the package (`RouteSarCapability`):
/// typed `SarUnavailable` for a relaxation without package data, an invalid
/// request for a surcharge below the package minimum or a non-Wheeled
/// profile.
fn validate_sar(
    engine: &RoutingEngine,
    options: &RouteOptions,
    profile: MobilityProfile,
) -> Result<(), RouteFailure> {
    crate::sar::validate_request(
        options.sar.as_ref(),
        profile,
        &engine.package().sar_capability(),
    )
    .map_err(|error| match error {
        RouteError::SarUnavailable { relaxation } => RouteFailure::new(
            RouteError::SarUnavailable { relaxation },
            RouteDiagnostics::default(),
        ),
        RouteError::InvalidRequest { reason } => invalid(reason),
        _ => invalid("sar options"),
    })
}

/// An overlay the request applied, with the edges it covered.
struct AppliedOverlay {
    index: usize,
    edge_ids: BTreeSet<u64>,
}

/// Every applied overlay the route crosses, with how much of the route.
fn overlay_uses(
    applied: &[AppliedOverlay],
    overlays: &[RouteOverlay],
    attributions: &[RouteEdgeAttribution],
) -> Vec<RouteOverlayUse> {
    applied
        .iter()
        .filter_map(|overlay| {
            let edges = attributions
                .iter()
                .filter(|edge| overlay.edge_ids.contains(&edge.edge_id))
                .collect::<Vec<_>>();
            if edges.is_empty() {
                return None;
            }
            let definition = &overlays[overlay.index];
            Some(RouteOverlayUse {
                overlay_index: overlay.index,
                label: definition.label(overlay.index),
                policy: definition.policy,
                source: definition.source.clone(),
                route_edge_count: edges.len(),
                route_length_m: edges.iter().map(|edge| edge.distance_m).sum(),
            })
        })
        .collect()
}

/// `request` departing from where the previous leg arrived (`arrival`), in the
/// direction it arrived in unless the waypoint sets a heading of its own.
fn continuing_request(request: &LegRequest, arrival: &CorrelatedLocation) -> LegRequest {
    let mut continued = request.clone();
    continued.origin = arrival.snapped_point;
    if continued.origin_options.heading_deg.is_none()
        && let Some(heading) = arrival.heading_deg
    {
        continued.origin_options.heading_deg = Some(heading);
        continued.origin_options.heading_tolerance_deg = continued
            .origin_options
            .heading_tolerance_deg
            .or(Some(WAYPOINT_CONTINUITY_HEADING_TOLERANCE_DEG));
    }
    continued
}

/// Applies the request's overlays to `options` and returns the ones applied
/// with their edges. An overlay is skipped when its `profiles` leave out
/// `profile` or (with `evaluated_at_ms`) its window does not hold that time.
/// Precedence is fixed (see [`RouteOverlayPolicy`]): a block is
/// dropped on an edge an `open` with a later `valid_from_ms` covers; a block
/// wins over everything else (the engine's blocks do); discounts multiply
/// and slow-go factors take the largest; the slowest speed override stands.
fn apply_overlays(
    engine: &RoutingEngine,
    overlays: &[RouteOverlay],
    profile: MobilityProfile,
    evaluated_at_ms: Option<i64>,
    options: &mut RouteOptions,
    diagnostics: &mut RouteOverlayDiagnostics,
) -> Vec<AppliedOverlay> {
    struct Projected {
        index: usize,
        edge_ids: BTreeSet<u64>,
        edge_ranges: Vec<RouteEdgeRange>,
    }
    let mut projected = Vec::new();
    for (index, overlay) in overlays.iter().enumerate() {
        if !overlay.profiles.is_empty() && !overlay.profiles.contains(&profile) {
            diagnostics.ignored_overlay_count += 1;
            continue;
        }
        if evaluated_at_ms.is_some_and(|time| !overlay.active_at(time)) {
            diagnostics.ignored_overlay_count += 1;
            continue;
        }
        let projection = if overlay.edges.is_empty() {
            project_route_overlay_geometry(
                engine.route_graph_view(),
                overlay.geometry_kind,
                &overlay.points,
                overlay.projection_distance_m,
            )
        } else {
            diagnostics.explicit_edge_reference_count += overlay.edges.len();
            crate::RouteOverlayGeometryProjection {
                edge_ids: overlay.edges.iter().map(|edge| edge.edge_id).collect(),
                edge_ranges: overlay
                    .edges
                    .iter()
                    .filter(|edge| edge.start_fraction > 0.0 || edge.end_fraction < 1.0)
                    .copied()
                    .collect(),
                confidence_basis_points: 10_000,
                ..Default::default()
            }
        };
        if projection.edge_ids.is_empty() {
            continue;
        }
        diagnostics.projected_geometry_count += 1;
        diagnostics.area_inside_edge_count += projection.inside_edge_count;
        diagnostics.area_intersecting_edge_count += projection.intersecting_edge_count;
        diagnostics.area_near_edge_count += projection.near_edge_count;
        diagnostics.partial_edge_range_count += projection.partial_edge_range_count;
        diagnostics.projection_confidence_basis_points = Some(
            diagnostics
                .projection_confidence_basis_points
                .map_or(projection.confidence_basis_points, |current| {
                    current.min(projection.confidence_basis_points)
                }),
        );
        projected.push(Projected {
            index,
            edge_ids: projection.edge_ids,
            edge_ranges: projection.edge_ranges,
        });
    }
    // Each opened edge with its latest opening time (absent: earliest).
    let mut opened = BTreeMap::<u64, Option<i64>>::new();
    for overlay in &projected {
        if overlays[overlay.index].policy == RouteOverlayPolicy::Open {
            let from = overlays[overlay.index].valid_from_ms;
            for edge_id in &overlay.edge_ids {
                let entry = opened.entry(*edge_id).or_insert(from);
                *entry = (*entry).max(from);
            }
        }
    }
    // An open from after the block's start wins over the block.
    let opened_after = |edge_id: u64, block_from: Option<i64>| {
        opened
            .get(&edge_id)
            .is_some_and(|open_from| *open_from > block_from)
    };
    let set_speed = |options: &mut RouteOptions, edge_id: u64, speed_kph: f64| {
        options
            .edge_speed_overrides
            .entry(edge_id)
            .and_modify(|current| *current = current.min(speed_kph))
            .or_insert(speed_kph);
    };
    for overlay in &projected {
        let definition = &overlays[overlay.index];
        let ranged = overlay
            .edge_ranges
            .iter()
            .map(|range| range.edge_id)
            .collect::<BTreeSet<_>>();
        match definition.policy {
            RouteOverlayPolicy::NoGo => {
                diagnostics.no_go_overlay_count += 1;
                let from = definition.valid_from_ms;
                options.blocked_edge_ids.extend(
                    overlay
                        .edge_ids
                        .iter()
                        .copied()
                        .filter(|edge_id| !opened_after(*edge_id, from)),
                );
                options.blocked_edge_ranges.extend(
                    overlay
                        .edge_ranges
                        .iter()
                        .filter(|range| !opened_after(range.edge_id, from))
                        .copied(),
                );
            }
            RouteOverlayPolicy::SlowGo => {
                diagnostics.slow_go_overlay_count += 1;
                let multiplier = definition.cost_multiplier.unwrap_or(2.0);
                for edge_id in &overlay.edge_ids {
                    options
                        .edge_cost_multipliers
                        .entry(*edge_id)
                        .and_modify(|current| *current = current.max(multiplier))
                        .or_insert(multiplier);
                }
                options
                    .edge_cost_multiplier_ranges
                    .extend(
                        overlay
                            .edge_ranges
                            .iter()
                            .map(|range| RouteEdgeCostMultiplierRange {
                                edge_id: range.edge_id,
                                start_fraction: range.start_fraction,
                                end_fraction: range.end_fraction,
                                multiplier,
                            }),
                    );
            }
            RouteOverlayPolicy::Warning => {
                diagnostics.hazard_warning_overlay_count += 1;
                options.warning_edge_ids.extend(&overlay.edge_ids);
                options
                    .warning_edge_ranges
                    .extend(overlay.edge_ranges.iter().copied());
            }
            RouteOverlayPolicy::Discount => {
                diagnostics.discount_overlay_count += 1;
                // A projected range covers its stretch only; an edge without
                // one is covered whole. Discounts are ranges so that
                // overlapping ones multiply (one per overlay and edge).
                let whole = overlay
                    .edge_ids
                    .iter()
                    .filter(|edge_id| !ranged.contains(edge_id))
                    .map(|edge_id| RouteEdgeRange {
                        edge_id: *edge_id,
                        start_fraction: 0.0,
                        end_fraction: 1.0,
                    });
                let stretches = whole
                    .chain(overlay.edge_ranges.iter().copied())
                    .collect::<Vec<_>>();
                if let Some(multiplier) = definition.cost_multiplier {
                    options
                        .edge_cost_multiplier_ranges
                        .extend(stretches.iter().map(|range| RouteEdgeCostMultiplierRange {
                            edge_id: range.edge_id,
                            start_fraction: range.start_fraction,
                            end_fraction: range.end_fraction,
                            multiplier,
                        }));
                }
                if let Some(speed_kph) = definition.speed_kph {
                    for range in &stretches {
                        if range.start_fraction <= 0.0 && range.end_fraction >= 1.0 {
                            set_speed(options, range.edge_id, speed_kph);
                        } else {
                            options.edge_speed_override_ranges.push(
                                crate::RouteEdgeSpeedOverrideRange {
                                    edge_id: range.edge_id,
                                    start_fraction: range.start_fraction,
                                    end_fraction: range.end_fraction,
                                    speed_kph,
                                },
                            );
                        }
                    }
                }
            }
            RouteOverlayPolicy::Open => {
                diagnostics.open_overlay_count += 1;
                // Opening applies to whole edges.
                options.opened_edge_ids.extend(&overlay.edge_ids);
                if let Some(speed_kph) = definition.speed_kph {
                    for edge_id in &overlay.edge_ids {
                        set_speed(options, *edge_id, speed_kph);
                    }
                }
            }
        }
    }
    projected
        .into_iter()
        .map(|overlay| AppliedOverlay {
            index: overlay.index,
            edge_ids: overlay.edge_ids,
        })
        .collect()
}

impl RouteDynamicOverlay {
    /// Adds `incoming`'s split ranges and connectors to `target` (or makes
    /// it `target`), refreshing the counts.
    pub fn merge_into(target: &mut Option<RouteDynamicOverlay>, incoming: RouteDynamicOverlay) {
        merge_dynamic_overlay(target, incoming);
    }
}

fn merge_dynamic_overlay(
    target: &mut Option<RouteDynamicOverlay>,
    mut incoming: RouteDynamicOverlay,
) {
    if let Some(existing) = target.as_mut() {
        existing
            .split_edge_ranges
            .append(&mut incoming.split_edge_ranges);
        existing.virtual_edges.append(&mut incoming.virtual_edges);
        existing.refresh_diagnostics();
    } else {
        incoming.refresh_diagnostics();
        *target = Some(incoming);
    }
}

fn assemble_result(
    segments: Vec<RouteSegment>,
    overlay_diagnostics: RouteOverlayDiagnostics,
    alternatives_requested: bool,
) -> Result<RouteResult, RouteFailure> {
    let mut location_matches = vec![RouteLocationMatch::default(); segments.len() + 1];
    for (index, route) in segments
        .iter()
        .enumerate()
        .filter_map(|(index, segment)| segment.as_routed().map(|route| (index, route)))
    {
        location_matches[index].outgoing = Some(route.origin.clone());
        location_matches[index + 1].incoming = Some(route.destination.clone());
    }
    let mut edge_ids = Vec::new();
    let mut geometry = Vec::<GeoPoint>::new();
    let mut maneuvers = Vec::new();
    let mut edge_attributions = Vec::new();
    let mut shape_segments = Vec::new();
    let mut distance_m = 0.0;
    let mut duration_s = 0.0;
    let mut cost = 0.0;
    let mut visited_edge_count = 0;
    let mut settled_node_count = 0;
    let mut product_diagnostics = RouteSummaryDiagnostics::default();

    for (segment_index, segment) in segments.iter().enumerate() {
        distance_m += segment.distance_m();
        duration_s += segment.duration_s();
        cost += segment.cost();
        // A leg joins the previous one at their shared point; one starting
        // elsewhere keeps its first point, so the gap stays in the geometry.
        let joins_previous = geometry
            .last()
            .zip(segment.geometry().first())
            .is_some_and(|(last, first)| last.distance_to(*first) < 0.01);
        let waypoint_gap = segment_index > 0 && !joins_previous;
        if waypoint_gap {
            product_diagnostics.waypoint_gap_count += 1;
        }
        let shape_offset = geometry.len() - usize::from(joins_previous);
        geometry.extend(
            segment
                .geometry()
                .iter()
                .copied()
                .skip(usize::from(joins_previous)),
        );
        for maneuver in segment.maneuvers() {
            if segment_index > 0 && maneuver.kind == RouteManeuverKind::Start {
                // The Waypoint maneuver ending the previous leg runs on to
                // this leg's next maneuver in place of its Start. (A direct
                // previous leg ends without one.)
                if let Some(waypoint) =
                    maneuvers
                        .last_mut()
                        .filter(|waypoint: &&mut RouteManeuver| {
                            waypoint.kind == RouteManeuverKind::Waypoint
                                && waypoint.leg_index == Some(segment_index - 1)
                        })
                {
                    waypoint.distance_m = maneuver.distance_m;
                    waypoint.duration_s = maneuver.duration_s;
                    waypoint.end_shape_index = maneuver.end_shape_index + shape_offset;
                }
                continue;
            }
            let mut maneuver = maneuver.clone();
            if segment_index + 1 < segments.len() && maneuver.kind == RouteManeuverKind::Destination
            {
                arrive_at_waypoint(&mut maneuver, segment_index + 1);
            }
            maneuver.begin_shape_index += shape_offset;
            maneuver.end_shape_index += shape_offset;
            maneuver.leg_index = Some(segment_index);
            maneuvers.push(maneuver);
        }
        if let Some(route) = segment.as_routed() {
            for edge_id in &route.edge_ids {
                if edge_ids.last() != Some(edge_id) {
                    edge_ids.push(*edge_id);
                }
            }
            visited_edge_count += route.diagnostics.visited_edge_count;
            settled_node_count += route.diagnostics.settled_node_count;
            accumulate_product_diagnostics(&mut product_diagnostics, &route.diagnostics);
            edge_attributions.extend(route.legs.iter().flat_map(|leg| leg.edges.iter().cloned()));
            let local_segments = route
                .legs
                .iter()
                .flat_map(|leg| &leg.shape_segments)
                .collect::<Vec<_>>();
            let local_segment_count = local_segments.len();
            shape_segments.extend(local_segments.into_iter().enumerate().map(
                |(local_index, local)| {
                    let mut segment = local.clone();
                    segment.begin_shape_index += shape_offset;
                    segment.end_shape_index += shape_offset;
                    let touches_waypoint = (segment_index > 0 && local_index == 0)
                        || (segment_index + 1 < segments.len()
                            && local_index + 1 == local_segment_count);
                    if touches_waypoint
                        && !segment
                            .quality_flags
                            .contains(&RouteShapeQualityFlag::WaypointSplit)
                    {
                        segment
                            .quality_flags
                            .push(RouteShapeQualityFlag::WaypointSplit);
                    }
                    if waypoint_gap && local_index == 0 {
                        segment
                            .quality_flags
                            .push(RouteShapeQualityFlag::WaypointGap);
                    }
                    RouteSegmentShape {
                        route_segment_index: segment_index,
                        segment,
                    }
                },
            ));
        }
    }
    let alternative_geometry_count = segments
        .iter()
        .filter_map(RouteSegment::as_routed)
        .flat_map(|route| &route.alternatives)
        .map(|alternative| alternative.geometry.len())
        .sum::<usize>();
    let exposed_geometry_count = geometry.len()
        + segments
            .iter()
            .map(|segment| segment.geometry().len())
            .sum::<usize>()
        + alternative_geometry_count.saturating_mul(2);
    if exposed_geometry_count > MAX_ROUTE_OUTPUT_GEOMETRY_POINTS {
        return Err(too_large("output geometry"));
    }
    let exposed_edge_id_count = edge_ids.len()
        + segments
            .iter()
            .filter_map(RouteSegment::as_routed)
            .map(|route| route.edge_ids.len())
            .sum::<usize>()
        + segments
            .iter()
            .filter_map(RouteSegment::as_routed)
            .flat_map(|route| &route.alternatives)
            .map(|alternative| alternative.edge_ids.len())
            .sum::<usize>()
            .saturating_mul(2);
    let alternative_attribution_count = segments
        .iter()
        .filter_map(RouteSegment::as_routed)
        .flat_map(|route| &route.alternatives)
        .flat_map(|alternative| &alternative.legs)
        .map(|leg| leg.edges.len())
        .fold(0usize, usize::saturating_add);
    let alternative_shape_segment_count = segments
        .iter()
        .filter_map(RouteSegment::as_routed)
        .flat_map(|route| &route.alternatives)
        .flat_map(|alternative| &alternative.legs)
        .map(|leg| leg.shape_segments.len())
        .fold(0usize, usize::saturating_add);
    if exposed_edge_id_count
        .saturating_add(edge_attributions.len())
        .saturating_add(shape_segments.len())
        .saturating_add(alternative_attribution_count)
        .saturating_add(alternative_shape_segment_count)
        > MAX_ROUTE_OUTPUT_EDGE_IDS
    {
        return Err(too_large("output route attribution"));
    }
    let exposed_maneuver_count = maneuvers.len()
        + segments
            .iter()
            .map(|segment| segment.maneuvers().len())
            .sum::<usize>()
        + segments
            .iter()
            .filter_map(RouteSegment::as_routed)
            .flat_map(|route| &route.alternatives)
            .map(|alternative| alternative.maneuvers.len())
            .sum::<usize>()
            .saturating_mul(2);
    if exposed_maneuver_count > MAX_ROUTE_OUTPUT_MANEUVERS {
        return Err(too_large("output maneuvers"));
    }
    let alternatives = if !alternatives_requested {
        RouteAlternatives::NotRequested
    } else if segments.len() > 1 {
        RouteAlternatives::UnavailableForMultiLeg
    } else if let Some(route) = segments[0].as_routed() {
        RouteAlternatives::Computed(route.alternatives.clone())
    } else {
        RouteAlternatives::UnavailableForDirectLeg
    };
    let routed_segment_count = segments
        .iter()
        .filter(|segment| segment.as_routed().is_some())
        .count();
    let policy_summary = segments
        .iter()
        .find_map(RouteSegment::as_routed)
        .map(|route| route.policy_summary.clone());
    let recosts = aggregate_recosts(&segments)?;
    let qualified_access = segments
        .iter()
        .filter_map(RouteSegment::as_routed)
        .flat_map(|route| route.qualified_access.iter().cloned())
        .collect::<Vec<_>>();
    let sar = merge_sar_reports(&segments);
    let cost_summary = summarize_costs(&edge_attributions);
    let explanations = explain_route(
        &edge_attributions,
        policy_summary.as_ref(),
        &cost_summary,
        &overlay_diagnostics,
    );
    product_diagnostics.segment_count = segments.len();
    product_diagnostics.routed_segment_count = routed_segment_count;
    product_diagnostics.direct_segment_count = segments.len() - routed_segment_count;
    product_diagnostics.visited_edge_count = visited_edge_count;
    product_diagnostics.settled_node_count = settled_node_count;
    product_diagnostics.projected_overlay_count = overlay_diagnostics.projected_geometry_count;
    product_diagnostics.warning_edge_count = edge_attributions
        .iter()
        .filter(|edge| edge.has_warning_overlay)
        .count();
    Ok(RouteResult {
        diagnostics: product_diagnostics,
        segments,
        location_matches,
        edge_ids,
        geometry,
        polyline6: None,
        geojson: None,
        distance_m,
        duration_s,
        cost,
        maneuvers,
        recosts,
        edge_attributions,
        shape_segments,
        alternatives,
        policy_summary,
        cost_summary,
        explanations,
        overlay_diagnostics,
        overlay_uses: Vec::new(),
        overlay_windows: None,
        qualified_access,
        sar,
    })
}

/// Moves every geometry of `result` into its `format` encoding, last, once
/// nothing reads the points any more. Shape indexes stay valid: the
/// encodings keep every vertex.
fn encode_geometry(result: &mut RouteResult, format: GeometryFormat) {
    if format.is_points() {
        return;
    }
    fn alternative(format: GeometryFormat, alternative: &mut RouteAlternative) {
        encode_into(
            format,
            &mut alternative.geometry,
            &mut alternative.polyline6,
            &mut alternative.geojson,
        );
        for leg in &mut alternative.legs {
            encode_into(
                format,
                &mut leg.geometry,
                &mut leg.polyline6,
                &mut leg.geojson,
            );
        }
    }
    encode_into(
        format,
        &mut result.geometry,
        &mut result.polyline6,
        &mut result.geojson,
    );
    for segment in &mut result.segments {
        match segment {
            RouteSegment::Routed(route) => {
                encode_into(
                    format,
                    &mut route.geometry,
                    &mut route.polyline6,
                    &mut route.geojson,
                );
                for leg in &mut route.legs {
                    encode_into(
                        format,
                        &mut leg.geometry,
                        &mut leg.polyline6,
                        &mut leg.geojson,
                    );
                }
                for each in &mut route.alternatives {
                    alternative(format, each);
                }
            }
            RouteSegment::Direct(leg) => {
                encode_into(
                    format,
                    &mut leg.geometry,
                    &mut leg.polyline6,
                    &mut leg.geojson,
                );
            }
        }
    }
    if let RouteAlternatives::Computed(alternatives) = &mut result.alternatives {
        for each in alternatives {
            alternative(format, each);
        }
    }
}

/// Turns a non-final leg's destination maneuver into the arrival at waypoint
/// `endpoint_index`.
fn arrive_at_waypoint(maneuver: &mut RouteManeuver, endpoint_index: usize) {
    let instruction = format!("Arrive at waypoint {endpoint_index}");
    maneuver.kind = RouteManeuverKind::Waypoint;
    maneuver.instruction_key = "route.waypoint".to_string();
    maneuver.concise_instruction = instruction.clone();
    maneuver.instruction = instruction;
    maneuver
        .instruction_parameters
        .insert("waypoint_index".to_string(), endpoint_index.to_string());
}

/// The SAR reports of the routed legs as one: their uses in route order and
/// totals over all of them.
fn merge_sar_reports(segments: &[RouteSegment]) -> Option<RouteSarReport> {
    let reports = segments
        .iter()
        .filter_map(RouteSegment::as_routed)
        .filter_map(|route| route.sar.as_ref())
        .collect::<Vec<_>>();
    let first = reports.first()?;
    let uses = reports
        .iter()
        .flat_map(|report| report.uses.iter().cloned())
        .collect::<Vec<_>>();
    let mut totals = Vec::<crate::RouteSarUseTotal>::new();
    for total in reports.iter().flat_map(|report| &report.totals) {
        match totals.iter_mut().find(|merged| merged.kind == total.kind) {
            Some(merged) => {
                merged.count += total.count;
                merged.length_m += total.length_m;
                merged.added_cost_s += total.added_cost_s;
            }
            None => totals.push(total.clone()),
        }
    }
    totals.sort_by_key(|total| total.kind);
    Some(RouteSarReport {
        enabled: first.enabled.clone(),
        uses,
        totals,
    })
}

fn aggregate_recosts(segments: &[RouteSegment]) -> Result<Vec<RouteRecostSummary>, RouteFailure> {
    let mut recosts = Vec::<RouteRecostSummary>::new();
    for incoming in segments
        .iter()
        .filter_map(RouteSegment::as_routed)
        .flat_map(|route| &route.recosts)
    {
        merge_recost(&mut recosts, incoming)?;
    }
    if recosts.iter().any(|recost| {
        !recost.distance_m.is_finite()
            || recost.distance_m < 0.0
            || recost
                .cost
                .is_some_and(|value| !value.is_finite() || value < 0.0)
            || recost
                .duration_s
                .is_some_and(|value| !value.is_finite() || value < 0.0)
            || !recost.turn_penalty_total.is_finite()
            || recost.turn_penalty_total < 0.0
    }) {
        return Err(too_large("output recost values"));
    }
    Ok(recosts)
}

fn merge_recost(
    recosts: &mut Vec<RouteRecostSummary>,
    incoming: &RouteRecostSummary,
) -> Result<(), RouteFailure> {
    let Some(existing) = recosts
        .iter_mut()
        .find(|recost| recost.profile == incoming.profile)
    else {
        recosts.push(*incoming);
        return if recosts.len() > MAX_ROUTE_OUTPUT_RECOSTS {
            Err(too_large("output recosts"))
        } else {
            Ok(())
        };
    };
    existing.accessible &= incoming.accessible;
    existing.distance_m += incoming.distance_m;
    existing.cost = existing
        .cost
        .zip(incoming.cost)
        .map(|(left, right)| left + right);
    existing.duration_s = existing
        .duration_s
        .zip(incoming.duration_s)
        .map(|(left, right)| left + right);
    existing.blocked_edge_count = existing
        .blocked_edge_count
        .saturating_add(incoming.blocked_edge_count);
    existing.inaccessible_edge_count = existing
        .inaccessible_edge_count
        .saturating_add(incoming.inaccessible_edge_count);
    existing.destination_only_penalty_count = existing
        .destination_only_penalty_count
        .saturating_add(incoming.destination_only_penalty_count);
    existing.ford_penalty_count = existing
        .ford_penalty_count
        .saturating_add(incoming.ford_penalty_count);
    existing.toll_penalty_count = existing
        .toll_penalty_count
        .saturating_add(incoming.toll_penalty_count);
    existing.ferry_penalty_count = existing
        .ferry_penalty_count
        .saturating_add(incoming.ferry_penalty_count);
    existing.gate_penalty_count = existing
        .gate_penalty_count
        .saturating_add(incoming.gate_penalty_count);
    existing.access_restriction_penalty_count = existing
        .access_restriction_penalty_count
        .saturating_add(incoming.access_restriction_penalty_count);
    existing.overlay_multiplier_count = existing
        .overlay_multiplier_count
        .saturating_add(incoming.overlay_multiplier_count);
    existing.turn_penalty_total += incoming.turn_penalty_total;
    Ok(())
}

fn accumulate_product_diagnostics(
    summary: &mut RouteSummaryDiagnostics,
    diagnostics: &RouteDiagnostics,
) {
    summary.inaccessible_edge_count += diagnostics.inaccessible_edge_count;
    summary.destination_only_penalty_count += diagnostics.destination_only_penalty_count;
    summary.ford_penalty_count += diagnostics.ford_penalty_count;
    summary.toll_penalty_count += diagnostics.toll_penalty_count;
    summary.ferry_penalty_count += diagnostics.ferry_penalty_count;
    summary.gate_penalty_count += diagnostics.gate_penalty_count;
    summary.access_restriction_penalty_count += diagnostics.access_restriction_penalty_count;
    summary.overlay_multiplier_count += diagnostics.overlay_multiplier_count;
    summary.ssvg_partial_edge_fragment_used_count +=
        diagnostics.ssvg_partial_edge_fragment_used_count;
    summary.ssvg_connector_used_count += diagnostics.ssvg_connector_used_count;
    summary.ssvg_endpoint_connector_used_count += diagnostics.ssvg_endpoint_connector_used_count;
    summary.ssvg_road_entry_connector_used_count +=
        diagnostics.ssvg_road_entry_connector_used_count;
    summary.ssvg_road_bypass_connector_used_count +=
        diagnostics.ssvg_road_bypass_connector_used_count;
    summary.ssvg_cross_country_connector_used_count +=
        diagnostics.ssvg_cross_country_connector_used_count;
    summary.origin_candidate_count += diagnostics.origin_candidate_count;
    summary.destination_candidate_count += diagnostics.destination_candidate_count;
    summary.rejected_candidate_count += diagnostics.rejected_candidate_count;
    summary.origin_rejected_candidate_count += diagnostics.origin_rejected_candidate_count;
    summary.destination_rejected_candidate_count +=
        diagnostics.destination_rejected_candidate_count;
    summary.origin_best_candidate_score_basis_points = diagnostics
        .origin_best_candidate_score_basis_points
        .or(summary.origin_best_candidate_score_basis_points);
    summary.destination_best_candidate_score_basis_points = diagnostics
        .destination_best_candidate_score_basis_points
        .or(summary.destination_best_candidate_score_basis_points);
}

fn summarize_costs(edges: &[RouteEdgeAttribution]) -> RouteCostSummary {
    let mut summary = RouteCostSummary {
        attributed_edge_count: edges.len(),
        conditional_restriction_edge_count: edges
            .iter()
            .filter(|edge| !edge.conditional_restrictions.is_empty())
            .count(),
        ..RouteCostSummary::default()
    };
    let mut drivers = [
        (RouteCostDriverKind::ProfileTime, 0.0),
        (RouteCostDriverKind::RoadType, 0.0),
        (RouteCostDriverKind::Surface, 0.0),
        (RouteCostDriverKind::TrackQuality, 0.0),
        (RouteCostDriverKind::RoadUse, 0.0),
        (RouteCostDriverKind::Overlay, 0.0),
        (RouteCostDriverKind::Avoidance, 0.0),
        (RouteCostDriverKind::Turns, 0.0),
    ];

    for edge in edges {
        let cost = edge.cost_breakdown;
        summary.costed_edge_count += 1;
        summary.total_cost += cost.total_cost;
        summary.turn_penalty_total += cost.turn_penalty;
        if cost.profile_time_factor > 1.0 {
            summary.profile_time_penalty_count += 1;
        }
        summary.max_profile_time_factor = summary
            .max_profile_time_factor
            .max(cost.profile_time_factor);
        if (cost.overlay_multiplier - 1.0).abs() > f64::EPSILON {
            summary.overlay_multiplier_count += 1;
        }
        summary.max_overlay_multiplier =
            summary.max_overlay_multiplier.max(cost.overlay_multiplier);
        if cost.avoided_edge_penalty > 1.0 {
            summary.avoided_penalty_count += 1;
        }
        let road_use = cost.toll_penalty
            * cost.ferry_penalty
            * cost.gate_penalty
            * cost.access_restriction_penalty
            * cost.destination_only_penalty;
        if road_use > 1.0 {
            summary.road_use_penalty_count += 1;
        }
        summary.max_road_use_penalty = summary.max_road_use_penalty.max(road_use);

        add_driver(
            &mut drivers,
            RouteCostDriverKind::ProfileTime,
            positive_extra(cost.base_distance_cost, cost.profile_time_factor),
        );
        add_driver(
            &mut drivers,
            RouteCostDriverKind::RoadType,
            positive_extra(
                cost.base_distance_cost * cost.profile_time_factor,
                cost.road_type_factor,
            ),
        );
        add_driver(
            &mut drivers,
            RouteCostDriverKind::Surface,
            positive_extra(cost.base_distance_cost, cost.surface_factor),
        );
        add_driver(
            &mut drivers,
            RouteCostDriverKind::TrackQuality,
            positive_extra(cost.base_distance_cost, cost.tracktype_factor)
                + positive_extra(cost.base_distance_cost, cost.smoothness_factor),
        );
        add_driver(
            &mut drivers,
            RouteCostDriverKind::RoadUse,
            positive_extra(cost.base_distance_cost, road_use),
        );
        add_driver(
            &mut drivers,
            RouteCostDriverKind::Overlay,
            positive_extra(cost.base_distance_cost, cost.overlay_multiplier),
        );
        add_driver(
            &mut drivers,
            RouteCostDriverKind::Avoidance,
            positive_extra(cost.base_distance_cost, cost.avoided_edge_penalty),
        );
        add_driver(&mut drivers, RouteCostDriverKind::Turns, cost.turn_penalty);
    }
    summary.dominant_driver = drivers
        .into_iter()
        .filter(|(_, cost)| *cost > f64::EPSILON)
        .max_by(|(_, left), (_, right)| left.total_cmp(right))
        .map(|(kind, cost)| RouteCostDriverSummary { kind, cost });
    summary
}

fn add_driver(drivers: &mut [(RouteCostDriverKind, f64)], kind: RouteCostDriverKind, cost: f64) {
    if let Some((_, total)) = drivers.iter_mut().find(|(candidate, _)| *candidate == kind) {
        *total += cost;
    }
}

fn positive_extra(base: f64, factor: f64) -> f64 {
    if base.is_finite() && factor.is_finite() && base > 0.0 && factor > 1.0 {
        base * (factor - 1.0)
    } else {
        0.0
    }
}

fn explain_route(
    edges: &[RouteEdgeAttribution],
    policy: Option<&RoutePolicySummary>,
    cost: &RouteCostSummary,
    overlay: &RouteOverlayDiagnostics,
) -> Vec<RouteExplanation> {
    let mut explanations = Vec::new();
    if let Some(driver) = cost.dominant_driver {
        explanations.push(RouteExplanation {
            kind: RouteExplanationKind::DominantCostDriver(driver.kind),
            severity: RouteExplanationSeverity::Cost,
            edge_count: Some(cost.costed_edge_count),
            value: Some(driver.cost),
        });
    }
    if overlay.projected_geometry_count > 0
        || policy.is_some_and(|policy| {
            policy.dynamic_overlay_partial_split_edge_count > 0
                || policy.dynamic_overlay_virtual_connector_count > 0
        })
    {
        explanations.push(RouteExplanation {
            kind: RouteExplanationKind::DynamicOverlay,
            severity: RouteExplanationSeverity::Warning,
            edge_count: Some(
                overlay.area_inside_edge_count
                    + overlay.area_intersecting_edge_count
                    + overlay.area_near_edge_count,
            ),
            value: policy.map(|policy| policy.dynamic_overlay_virtual_connector_count as f64),
        });
    }
    if policy
        .is_some_and(|policy| policy.fixed_speed_kph.is_some() || policy.top_speed_kph.is_some())
    {
        explanations.push(RouteExplanation {
            kind: RouteExplanationKind::SpeedPolicy,
            severity: RouteExplanationSeverity::Info,
            edge_count: None,
            value: policy.and_then(|policy| policy.fixed_speed_kph.or(policy.top_speed_kph)),
        });
    }
    if cost.conditional_restriction_edge_count > 0 {
        explanations.push(RouteExplanation {
            kind: RouteExplanationKind::ConditionalRestrictions,
            severity: RouteExplanationSeverity::Warning,
            edge_count: Some(cost.conditional_restriction_edge_count),
            value: None,
        });
    }
    let constrained = edges
        .iter()
        .filter(|edge| {
            edge.has_gate
                || edge.is_access_restricted
                || edge.is_destination_only
                || edge.is_toll
                || edge.is_ferry
                || edge.has_ford
        })
        .count();
    if constrained > 0 {
        explanations.push(RouteExplanation {
            kind: RouteExplanationKind::AccessConstraints,
            severity: RouteExplanationSeverity::Warning,
            edge_count: Some(constrained),
            value: Some(cost.max_road_use_penalty),
        });
    }
    explanations.truncate(6);
    explanations
}

fn validate_request(request: &RouteRequest) -> Result<(), RouteFailure> {
    if request.locations.len() < 2 {
        return Err(invalid("at least two locations"));
    }
    if request.locations.len() > MAX_ROUTE_LOCATIONS {
        return Err(too_large("locations"));
    }
    if !request.legs.is_empty() && request.legs.len() != request.locations.len() - 1 {
        return Err(invalid("leg count must equal location count minus one"));
    }
    if request.locations.iter().any(|location| {
        !location.point.lat.is_finite()
            || !location.point.lng.is_finite()
            || !(-90.0..=90.0).contains(&location.point.lat)
            || !(-180.0..=180.0).contains(&location.point.lng)
    }) {
        return Err(invalid("location coordinates"));
    }
    validate_endpoint_hints(&request.locations)?;
    validate_options(&request.options)?;
    if request.overlays.len() > MAX_ROUTE_OVERLAYS {
        return Err(too_large("overlays"));
    }
    let overlay_point_count = request
        .overlays
        .iter()
        .map(|overlay| overlay.points.len())
        .sum::<usize>();
    if overlay_point_count > MAX_ROUTE_OVERLAY_POINTS {
        return Err(too_large("overlay geometry"));
    }
    for overlay in &request.overlays {
        validate_overlay(overlay)?;
    }
    let leg_count = request.locations.len() - 1;
    for (leg_index, overlay) in &request.leg_dynamic_overlays {
        if *leg_index >= leg_count || request.leg_mode(*leg_index) != RouteLegMode::Routed {
            return Err(invalid("leg dynamic overlay"));
        }
        validate_dynamic_overlay(overlay)?;
    }
    validate_dynamic_overlay_totals(
        request.options.dynamic_overlay.as_ref(),
        request.leg_dynamic_overlays.values(),
    )?;
    Ok(())
}

/// One overlay: its geometry (or explicit edges), its multiplier and speed
/// for its policy, and its time window.
fn validate_overlay(overlay: &RouteOverlay) -> Result<(), RouteFailure> {
    let minimum_points = match overlay.geometry_kind {
        _ if !overlay.edges.is_empty() => 0,
        RouteOverlayGeometryKind::Point => 1,
        RouteOverlayGeometryKind::Line => 2,
        RouteOverlayGeometryKind::Polygon => 3,
    };
    if overlay.points.len() < minimum_points
        || (overlay.edges.is_empty()
            && (overlay.projection_distance_m == 0
                || overlay.projection_distance_m > MAX_ROUTE_PROJECTION_DISTANCE_M))
        || overlay.points.iter().copied().any(invalid_point)
        || overlay.edges.iter().any(|range| !valid_range(*range))
    {
        return Err(invalid("overlay geometry"));
    }
    let multiplier = overlay.cost_multiplier;
    let speed_ok = overlay.speed_kph.is_none_or(|speed| {
        speed.is_finite() && speed > 0.0 && speed <= MAX_ROUTE_OVERLAY_SPEED_KPH
    });
    let valid = match overlay.policy {
        RouteOverlayPolicy::SlowGo => {
            multiplier.is_none_or(|value| value.is_finite() && value > 1.0)
                && overlay.speed_kph.is_none()
        }
        RouteOverlayPolicy::Discount => {
            multiplier.is_none_or(|value| value.is_finite() && value > 0.0 && value < 1.0)
                && speed_ok
                && (multiplier.is_some() || overlay.speed_kph.is_some())
        }
        RouteOverlayPolicy::Open => multiplier.is_none() && speed_ok,
        RouteOverlayPolicy::NoGo | RouteOverlayPolicy::Warning => true,
    };
    if !valid {
        return Err(invalid("overlay cost multiplier"));
    }
    if let (Some(from), Some(to)) = (overlay.valid_from_ms, overlay.valid_to_ms)
        && to <= from
    {
        return Err(invalid("overlay time window"));
    }
    if let Some(source) = &overlay.source
        && (invalid_bounded_string(&source.set_id)
            || invalid_bounded_string(&source.item_id)
            || source.author.as_deref().is_some_and(invalid_bounded_string))
    {
        return Err(invalid("overlay source"));
    }
    Ok(())
}

/// [`validate_options`] for the matrix and isochrone entry points, which
/// fail with a bare [`RouteError`].
pub(crate) fn validate_query_options(options: &RouteOptions) -> Result<(), RouteError> {
    validate_options(options).map_err(|failure| failure.error)
}

/// The overlay checks a route runs ([`MAX_ROUTE_OVERLAYS`],
/// [`MAX_ROUTE_OVERLAY_POINTS`], each overlay's geometry, policy values,
/// window and source), for the matrix, isochrone and nearest entry points.
pub(crate) fn validate_query_overlays(overlays: &[RouteOverlay]) -> Result<(), RouteError> {
    if overlays.len() > MAX_ROUTE_OVERLAYS {
        return Err(RouteError::RequestTooLarge { what: "overlays" });
    }
    if overlays
        .iter()
        .map(|overlay| overlay.points.len())
        .sum::<usize>()
        > MAX_ROUTE_OVERLAY_POINTS
    {
        return Err(RouteError::RequestTooLarge {
            what: "overlay geometry",
        });
    }
    for overlay in overlays {
        validate_overlay(overlay).map_err(|failure| failure.error)?;
    }
    Ok(())
}

/// Folds a matrix, isochrone or nearest request's overlays into its
/// options as a route does: windows evaluated at one time
/// (`conditional_restriction_eval_at_ms`, else `departure_at_ms`, else
/// now), overlays for other profiles left out. The options are checked
/// again, since overlays add edge constraints.
pub(crate) fn apply_query_overlays(
    engine: &RoutingEngine,
    overlays: &[RouteOverlay],
    profile: MobilityProfile,
    departure_at_ms: Option<i64>,
    options: &mut RouteOptions,
) -> Result<(), RouteError> {
    if overlays.is_empty() {
        return Ok(());
    }
    let evaluated_at_ms = overlays.iter().any(RouteOverlay::has_window).then(|| {
        options
            .costing
            .conditional_restriction_eval_at_ms
            .or(departure_at_ms)
            .unwrap_or_else(now_ms)
    });
    apply_overlays(
        engine,
        overlays,
        profile,
        evaluated_at_ms,
        options,
        &mut RouteOverlayDiagnostics::default(),
    );
    validate_query_options(options)
}

fn validate_options(options: &RouteOptions) -> Result<(), RouteFailure> {
    if options.candidate_limit > crate::limits::MAX_REQUEST_CANDIDATES {
        return Err(too_large("candidate limit"));
    }
    if let Some(off_network) = &options.off_network {
        off_network.validate().map_err(|error| match error {
            RouteError::InvalidRequest { reason } => invalid(reason),
            _ => invalid("off_network"),
        })?;
    }
    if options.max_correlation_distance_m > crate::limits::MAX_CORRELATION_DISTANCE_M {
        return Err(too_large("correlation distance"));
    }
    if options.alternatives.requested_alternative_count > 3 {
        return Err(too_large("alternatives"));
    }
    let costing_values = [
        options.costing.avoid_tracks,
        options.costing.avoid_paths,
        options.costing.avoid_motorways,
        options.costing.avoid_destination_only,
        options.costing.avoid_tolls,
        options.costing.avoid_ferries,
        options.costing.avoid_gates,
        options.costing.avoid_access_restricted,
        options.costing.prefer_paved,
        options.costing.prefer_known_route_refs,
        options.costing.emergency_override_strength,
        options.alternatives.max_shared_edge_ratio,
        options.alternatives.max_cost_ratio,
    ];
    if costing_values.iter().any(|value| !value.is_finite())
        || options
            .costing
            .fixed_speed_kph
            .is_some_and(|value| !value.is_finite())
        || options
            .costing
            .top_speed_kph
            .is_some_and(|value| !value.is_finite())
    {
        return Err(invalid("costing options"));
    }
    let dynamic_split_range_count = options
        .dynamic_overlay
        .as_ref()
        .map(|overlay| {
            overlay
                .split_edge_ranges
                .iter()
                .map(|split| split.blocked_ranges.len() + split.usable_ranges.len())
                .sum::<usize>()
        })
        .unwrap_or(0);
    if options
        .edge_speed_overrides
        .values()
        .chain(
            options
                .edge_speed_override_ranges
                .iter()
                .map(|range| &range.speed_kph),
        )
        .any(|speed| !speed.is_finite() || *speed <= 0.0 || *speed > MAX_ROUTE_OVERLAY_SPEED_KPH)
        || options.edge_speed_override_ranges.iter().any(|range| {
            !valid_range(RouteEdgeRange {
                edge_id: range.edge_id,
                start_fraction: range.start_fraction,
                end_fraction: range.end_fraction,
            })
        })
    {
        return Err(invalid("speed override"));
    }
    let adjustment_count = options.blocked_edge_ids.len()
        + options.edge_speed_overrides.len()
        + options.edge_speed_override_ranges.len()
        + options.opened_edge_ids.len()
        + options.blocked_edge_ranges.len()
        + options.avoided_edge_ids.len()
        + options.avoided_edge_ranges.len()
        + options.warning_edge_ids.len()
        + options.warning_edge_ranges.len()
        + options.edge_cost_multipliers.len()
        + options.edge_cost_multiplier_ranges.len()
        + dynamic_split_range_count;
    if adjustment_count > crate::limits::MAX_REQUEST_EDGE_CONSTRAINTS {
        return Err(too_large("edge constraints"));
    }
    if options
        .edge_cost_multipliers
        .values()
        .any(|value| !value.is_finite() || *value <= 0.0)
        || options
            .edge_cost_multiplier_ranges
            .iter()
            .any(|range| !valid_multiplier_range(*range))
        || options
            .blocked_edge_ranges
            .iter()
            .chain(&options.warning_edge_ranges)
            .any(|range| !valid_range(*range))
        || options.avoided_edge_ranges.iter().any(|range| {
            !range.start_fraction.is_finite()
                || !range.end_fraction.is_finite()
                || range.start_fraction < 0.0
                || range.end_fraction > 1.0
                || range.end_fraction <= range.start_fraction
        })
    {
        return Err(invalid("edge adjustment"));
    }
    if let Some(overlay) = &options.dynamic_overlay {
        if overlay.split_edge_ranges.len() > crate::limits::MAX_REQUEST_EDGE_CONSTRAINTS {
            return Err(too_large("dynamic split ranges"));
        }
        if overlay.virtual_edges.len() > crate::limits::MAX_REQUEST_DYNAMIC_CONNECTORS {
            return Err(too_large("dynamic connectors"));
        }
        if overlay
            .virtual_edges
            .iter()
            .map(|edge| edge.geometry.len())
            .sum::<usize>()
            > crate::limits::MAX_REQUEST_DYNAMIC_GEOMETRY_POINTS
        {
            return Err(too_large("dynamic connector geometry"));
        }
        validate_dynamic_overlay(overlay)?;
    }
    Ok(())
}

fn validate_endpoint_hints(endpoints: &[RouteLocation]) -> Result<(), RouteFailure> {
    let hint_count = endpoints
        .iter()
        .map(|endpoint| {
            endpoint.options.preferred_road_classes.len()
                + endpoint.options.avoided_road_classes.len()
                + endpoint.options.preferred_route_refs.len()
                + usize::from(endpoint.options.preferred_level.is_some())
        })
        .sum::<usize>();
    if hint_count > MAX_ROUTE_LOCATION_HINTS {
        return Err(too_large("endpoint hints"));
    }
    let mut strings = endpoints.iter().flat_map(|endpoint| {
        endpoint
            .options
            .preferred_route_refs
            .iter()
            .map(String::as_str)
            .chain(endpoint.options.preferred_level.as_deref())
    });
    if strings.clone().map(str::len).sum::<usize>() > MAX_ROUTE_TOTAL_STRING_BYTES {
        return Err(too_large("endpoint hint strings"));
    }
    if strings.any(invalid_bounded_string) {
        return Err(invalid("endpoint hints"));
    }
    Ok(())
}

fn validate_dynamic_overlay(overlay: &RouteDynamicOverlay) -> Result<(), RouteFailure> {
    let nested_range_count = overlay
        .split_edge_ranges
        .iter()
        .map(|split| split.blocked_ranges.len() + split.usable_ranges.len())
        .sum::<usize>();
    if nested_range_count > crate::limits::MAX_REQUEST_EDGE_CONSTRAINTS {
        return Err(too_large("dynamic overlay split ranges"));
    }
    let source_chunk_count = overlay
        .virtual_edges
        .iter()
        .map(|edge| edge.source_chunk_ids.len())
        .sum::<usize>();
    let string_bytes = overlay
        .virtual_edges
        .iter()
        .map(|edge| {
            edge.connector_id.len()
                + edge.provenance.len()
                + edge.source_chunk_ids.iter().map(String::len).sum::<usize>()
        })
        .sum::<usize>();
    if source_chunk_count > MAX_ROUTE_PROVENANCE_VALUES
        || string_bytes > MAX_ROUTE_TOTAL_STRING_BYTES
    {
        return Err(too_large("dynamic overlay provenance"));
    }
    if overlay
        .split_edge_ranges
        .iter()
        .flat_map(|split| split.blocked_ranges.iter().chain(&split.usable_ranges))
        .any(|range| !valid_range(*range))
    {
        return Err(invalid("dynamic overlay split range"));
    }
    for edge in &overlay.virtual_edges {
        if invalid_bounded_string(&edge.connector_id)
            || (!edge.provenance.is_empty() && invalid_bounded_string(&edge.provenance))
            || edge
                .source_chunk_ids
                .iter()
                .any(|value| invalid_bounded_string(value))
            || !edge.distance_m.is_finite()
            || edge.distance_m < 0.0
            || !edge.cost.is_finite()
            || edge.cost < 0.0
            || edge
                .source_fraction
                .into_iter()
                .chain(edge.target_fraction)
                .any(|value| !value.is_finite() || !(0.0..=1.0).contains(&value))
            || edge
                .source_point
                .into_iter()
                .chain(edge.target_point)
                .chain(edge.geometry.iter().copied())
                .any(invalid_point)
        {
            return Err(invalid("dynamic overlay connector"));
        }
    }
    Ok(())
}

fn validate_dynamic_overlay_totals<'a>(
    request_base: Option<&'a RouteDynamicOverlay>,
    leg_overlays: impl Iterator<Item = &'a RouteDynamicOverlay>,
) -> Result<(), RouteFailure> {
    let overlays = request_base.into_iter().chain(leg_overlays);
    let (connectors, geometry_points, nested_ranges, source_chunks, string_bytes) = overlays.fold(
        (0usize, 0usize, 0usize, 0usize, 0usize),
        |totals, overlay| {
            let overlay_geometry = overlay
                .virtual_edges
                .iter()
                .map(|edge| edge.geometry.len())
                .fold(0usize, usize::saturating_add);
            let overlay_nested_ranges = overlay
                .split_edge_ranges
                .iter()
                .map(|split| split.blocked_ranges.len() + split.usable_ranges.len())
                .fold(0usize, usize::saturating_add);
            let overlay_source_chunks = overlay
                .virtual_edges
                .iter()
                .map(|edge| edge.source_chunk_ids.len())
                .fold(0usize, usize::saturating_add);
            let overlay_string_bytes = overlay
                .virtual_edges
                .iter()
                .map(|edge| {
                    edge.connector_id
                        .len()
                        .saturating_add(edge.provenance.len())
                        .saturating_add(
                            edge.source_chunk_ids
                                .iter()
                                .map(String::len)
                                .fold(0usize, usize::saturating_add),
                        )
                })
                .fold(0usize, usize::saturating_add);
            (
                totals.0.saturating_add(overlay.virtual_edges.len()),
                totals.1.saturating_add(overlay_geometry),
                totals.2.saturating_add(overlay_nested_ranges),
                totals.3.saturating_add(overlay_source_chunks),
                totals.4.saturating_add(overlay_string_bytes),
            )
        },
    );
    if connectors > crate::limits::MAX_REQUEST_DYNAMIC_CONNECTORS {
        return Err(too_large("dynamic connectors"));
    }
    if geometry_points > crate::limits::MAX_REQUEST_DYNAMIC_GEOMETRY_POINTS {
        return Err(too_large("dynamic connector geometry"));
    }
    if nested_ranges > crate::limits::MAX_REQUEST_EDGE_CONSTRAINTS {
        return Err(too_large("dynamic split ranges"));
    }
    if source_chunks > MAX_ROUTE_PROVENANCE_VALUES || string_bytes > MAX_ROUTE_TOTAL_STRING_BYTES {
        return Err(too_large("dynamic overlay provenance"));
    }
    Ok(())
}

pub(crate) fn invalid_point(point: GeoPoint) -> bool {
    !point.lat.is_finite()
        || !point.lng.is_finite()
        || !(-90.0..=90.0).contains(&point.lat)
        || !(-180.0..=180.0).contains(&point.lng)
}

fn invalid_bounded_string(value: &str) -> bool {
    value.is_empty() || value.len() > MAX_ROUTE_STRING_BYTES || value.chars().any(char::is_control)
}

fn valid_range(range: RouteEdgeRange) -> bool {
    range.start_fraction.is_finite()
        && range.end_fraction.is_finite()
        && range.start_fraction >= 0.0
        && range.end_fraction <= 1.0
        && range.end_fraction > range.start_fraction
}

fn valid_multiplier_range(range: RouteEdgeCostMultiplierRange) -> bool {
    valid_range(RouteEdgeRange {
        edge_id: range.edge_id,
        start_fraction: range.start_fraction,
        end_fraction: range.end_fraction,
    }) && range.multiplier.is_finite()
        && range.multiplier > 0.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn legs_when_given_are_one_per_pair_of_locations() {
        let mut request = RouteRequest::new(
            GeoPoint::new(52.0, 21.0),
            GeoPoint::new(52.0, 21.1),
            MobilityProfile::Wheeled,
        );
        assert!(validate_request(&request).is_ok());
        request.legs = vec![RouteLegMode::Routed, RouteLegMode::Direct];
        assert_eq!(
            validate_request(&request).unwrap_err().error,
            RouteError::InvalidRequest {
                reason: "leg count must equal location count minus one"
            }
        );
        request.locations.truncate(1);
        assert!(validate_request(&request).is_err());
    }

    #[test]
    fn a_leg_dynamic_overlay_needs_a_routed_leg() {
        let mut request = RouteRequest::new(
            GeoPoint::new(52.0, 21.0),
            GeoPoint::new(52.0, 21.1),
            MobilityProfile::OnFoot,
        );
        request.legs = vec![RouteLegMode::Direct];
        request
            .leg_dynamic_overlays
            .insert(0, RouteDynamicOverlay::default());
        assert_eq!(
            validate_request(&request).unwrap_err().error,
            RouteError::InvalidRequest {
                reason: "leg dynamic overlay"
            }
        );
    }

    #[test]
    fn multi_leg_routes_never_claim_single_search_alternatives() {
        let direct =
            |a, b| RouteSegment::Direct(Box::new(direct_route_leg(a, b, MobilityProfile::OnFoot)));
        let result = assemble_result(
            vec![
                direct(GeoPoint::new(52.0, 21.0), GeoPoint::new(52.0, 21.1)),
                direct(GeoPoint::new(52.0, 21.1), GeoPoint::new(52.0, 21.2)),
            ],
            RouteOverlayDiagnostics::default(),
            true,
        )
        .unwrap();
        assert_eq!(
            result.alternatives,
            RouteAlternatives::UnavailableForMultiLeg
        );
        assert_eq!(result.diagnostics.direct_segment_count, 2);
    }

    #[test]
    fn dynamic_overlay_bounds_cover_the_whole_request() {
        let overlay_with_chunks = |edge_id, count| {
            let mut overlay = RouteDynamicOverlay::default();
            overlay.virtual_edges.push(crate::RouteDynamicVirtualEdge {
                edge_id,
                connector_id: format!("connector-{edge_id}"),
                connector_type: crate::RouteDynamicVirtualEdgeKind::CrossCountry,
                source_endpoint: None,
                source_point: None,
                source_edge_id: Some(1),
                source_fraction: Some(0.25),
                target_endpoint: None,
                target_point: None,
                target_edge_id: Some(2),
                target_fraction: Some(0.75),
                source_chunk_ids: vec!["chunk".to_string(); count],
                geometry: vec![GeoPoint::new(52.0, 21.0), GeoPoint::new(52.0, 21.1)],
                distance_m: 1.0,
                cost: 1.0,
                provenance: "test".to_string(),
            });
            overlay.refresh_diagnostics();
            overlay
        };
        let base = overlay_with_chunks(1, 2_049);
        let leg = overlay_with_chunks(2, 2_049);
        assert!(validate_dynamic_overlay_totals(Some(&base), std::iter::empty()).is_ok());
        assert_eq!(
            validate_dynamic_overlay_totals(Some(&base), std::iter::once(&leg))
                .unwrap_err()
                .error,
            RouteError::RequestTooLarge {
                what: "dynamic overlay provenance"
            }
        );
    }

    #[test]
    fn leg_leaving_a_waypoint_starts_where_the_previous_arrived() {
        let request = LegRequest::new(
            GeoPoint::new(52.0001, 21.0),
            GeoPoint::new(52.0, 21.1),
            MobilityProfile::Wheeled,
        );
        let arrival = CorrelatedLocation {
            edge_id: 1,
            compact_edge: Default::default(),
            shared_edge_info_id: 1,
            opposing_edge_id: Some(2),
            road_group_key: None,
            same_road_candidate_count: 1,
            source_node_id: 1,
            target_node_id: 2,
            snapped_point: GeoPoint::new(52.0, 21.0),
            distance_m: 11.0,
            fraction_along_edge: 0.5,
            heading_deg: Some(93),
            heading_delta_deg: None,
            side: crate::CorrelationSide::Left,
            component_size: None,
            outbound_reach_edge_count: None,
            inbound_reach_edge_count: None,
            layer: None,
            level: None,
            score: 1.0,
            score_breakdown: Default::default(),
            confidence: 1.0,
            access: None,
        };

        let continued = continuing_request(&request, &arrival);
        assert_eq!(continued.origin, arrival.snapped_point);
        assert_eq!(continued.origin_options.heading_deg, Some(93));
        assert_eq!(
            continued.origin_options.heading_tolerance_deg,
            Some(WAYPOINT_CONTINUITY_HEADING_TOLERANCE_DEG)
        );
        assert_eq!(continued.destination, request.destination);

        // A heading the waypoint sets is kept.
        let mut headed = request.clone();
        headed.origin_options = RouteEndpointOptions::default().with_heading(270);
        let continued = continuing_request(&headed, &arrival);
        assert_eq!(continued.origin_options.heading_deg, Some(270));
        assert_eq!(continued.origin_options.heading_tolerance_deg, None);
    }

    #[test]
    fn waypoint_gap_stays_in_the_stitched_geometry() {
        let direct =
            |a, b| RouteSegment::Direct(Box::new(direct_route_leg(a, b, MobilityProfile::OnFoot)));
        let joined_first = direct(GeoPoint::new(52.0, 21.0), GeoPoint::new(52.0, 21.001));
        let result = assemble_result(
            vec![
                joined_first.clone(),
                // Starts 7 m from where the first leg ended.
                direct(GeoPoint::new(52.0, 21.0011), GeoPoint::new(52.0, 21.002)),
            ],
            RouteOverlayDiagnostics::default(),
            false,
        )
        .unwrap();
        assert_eq!(result.diagnostics.waypoint_gap_count, 1);
        let first_len = joined_first.geometry().len();
        assert_eq!(result.geometry[first_len - 1], GeoPoint::new(52.0, 21.001));
        assert_eq!(result.geometry[first_len], GeoPoint::new(52.0, 21.0011));
        let second_leg_start = result
            .maneuvers
            .iter()
            .find(|maneuver| maneuver.leg_index == Some(1))
            .unwrap();
        assert_eq!(second_leg_start.begin_shape_index, first_len);
        let arrival = result
            .maneuvers
            .iter()
            .find(|maneuver| maneuver.kind == RouteManeuverKind::Waypoint);
        assert!(arrival.is_none_or(|arrival| arrival.leg_index == Some(0)));
    }

    #[test]
    fn product_diagnostics_aggregate_engine_counts_without_phase_timings() {
        let engine = RouteDiagnostics {
            origin_candidate_count: 3,
            destination_candidate_count: 4,
            rejected_candidate_count: 2,
            origin_rejected_candidate_count: 1,
            destination_rejected_candidate_count: 1,
            origin_best_candidate_score_basis_points: Some(9_000),
            destination_best_candidate_score_basis_points: Some(8_500),
            inaccessible_edge_count: 5,
            destination_only_penalty_count: 6,
            ford_penalty_count: 7,
            toll_penalty_count: 8,
            ferry_penalty_count: 9,
            gate_penalty_count: 10,
            access_restriction_penalty_count: 11,
            overlay_multiplier_count: 12,
            ssvg_partial_edge_fragment_used_count: 13,
            ssvg_connector_used_count: 14,
            ssvg_endpoint_connector_used_count: 15,
            ssvg_road_entry_connector_used_count: 16,
            ssvg_road_bypass_connector_used_count: 17,
            ssvg_cross_country_connector_used_count: 18,
            ..RouteDiagnostics::default()
        };
        let mut summary = RouteSummaryDiagnostics::default();

        accumulate_product_diagnostics(&mut summary, &engine);

        assert_eq!(summary.inaccessible_edge_count, 5);
        assert_eq!(summary.overlay_multiplier_count, 12);
        assert_eq!(summary.ssvg_cross_country_connector_used_count, 18);
        assert_eq!(summary.origin_candidate_count, 3);
        assert_eq!(
            summary.destination_best_candidate_score_basis_points,
            Some(8_500)
        );
    }

    #[test]
    fn recost_merge_preserves_profile_and_sums_leg_evidence() {
        let recost = |cost: Option<f64>, blocked_edge_count| RouteRecostSummary {
            profile: MobilityProfile::Tracked,
            accessible: cost.is_some(),
            distance_m: 10.0,
            cost,
            duration_s: cost,
            blocked_edge_count,
            inaccessible_edge_count: 1,
            destination_only_penalty_count: 2,
            ford_penalty_count: 3,
            toll_penalty_count: 4,
            ferry_penalty_count: 5,
            gate_penalty_count: 6,
            access_restriction_penalty_count: 7,
            overlay_multiplier_count: 8,
            turn_penalty_total: 9.0,
        };
        let mut merged = Vec::new();
        merge_recost(&mut merged, &recost(Some(5.0), 1)).unwrap();
        merge_recost(&mut merged, &recost(Some(7.0), 2)).unwrap();

        assert_eq!(merged.len(), 1);
        assert_eq!(merged[0].distance_m, 20.0);
        assert_eq!(merged[0].cost, Some(12.0));
        assert_eq!(merged[0].blocked_edge_count, 3);
        assert_eq!(merged[0].turn_penalty_total, 18.0);
    }
}

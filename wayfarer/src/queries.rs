//! The engine's lookups besides routing: map matching a trace, snapping a
//! point onto the road network, and projecting geometry onto package edges.

use serde::{Deserialize, Serialize};

use crate::correlator::Correlator;
use crate::map_matching::match_navigation_trace_cancellable;
use crate::{
    CorrelatedLocation, CorrelationMode, CorrelationPolicy, GeoPoint, MapMatchOptions,
    MapMatchResult, MobilityProfile, NavigationPositionSample, RouteCancellation,
    RouteEndpointOptions, RouteError, RouteOverlayGeometryKind, RouteOverlayGeometryProjection,
    RoutingEngine,
};

/// A trace to match onto the road network ([`RoutingEngine::map_match`]).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct MapMatchRequest {
    pub samples: Vec<NavigationPositionSample>,
    pub profile: MobilityProfile,
    #[serde(default)]
    pub options: MapMatchOptions,
    /// Stops the match (not part of the JSON form).
    #[serde(skip)]
    pub cancellation: RouteCancellation,
}

impl MapMatchRequest {
    pub fn new(samples: Vec<NavigationPositionSample>, profile: MobilityProfile) -> Self {
        Self {
            samples,
            profile,
            options: MapMatchOptions::default(),
            cancellation: RouteCancellation::default(),
        }
    }
}

/// A point to snap onto the roads `profile` may use ([`RoutingEngine::snap`]).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct SnapRequest {
    pub point: GeoPoint,
    pub profile: MobilityProfile,
    /// How far from `point` candidates may lie (default: 75 m, as for a
    /// route's locations).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_distance_m: Option<u32>,
    /// The most candidates to return (default: the routing default).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub candidate_limit: Option<usize>,
    /// Heading, side, layer and road-class preferences, as for a route's
    /// locations.
    #[serde(default)]
    pub options: RouteEndpointOptions,
}

impl SnapRequest {
    pub fn new(point: GeoPoint, profile: MobilityProfile) -> Self {
        Self {
            point,
            profile,
            max_distance_m: None,
            candidate_limit: None,
            options: RouteEndpointOptions::default(),
        }
    }
}

/// Where a point snaps, best candidate first.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct SnapResult {
    pub candidates: Vec<CorrelatedLocation>,
    /// Edges within reach that the profile, layer or preferences ruled out.
    pub rejected_candidate_count: usize,
    pub mode: CorrelationMode,
}

/// Geometry to project onto package edges ([`RoutingEngine::project`]), as a
/// route's overlays are.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct ProjectRequest {
    pub geometry_kind: RouteOverlayGeometryKind,
    pub points: Vec<GeoPoint>,
    /// Edges within this distance of the geometry are covered (inside a
    /// polygon, every edge is).
    pub projection_distance_m: u32,
}

impl RoutingEngine {
    /// Matches a GPS trace onto the roads `request.profile` may use: the
    /// matched samples, the edges driven and any alternatives.
    pub fn map_match(&self, request: MapMatchRequest) -> Result<MapMatchResult, RouteError> {
        if request.samples.is_empty() {
            return Err(RouteError::InvalidRequest {
                reason: "a trace needs at least one sample",
            });
        }
        request.cancellation.check()?;
        match_navigation_trace_cancellable(
            self.route_graph_view(),
            &request.samples,
            request.profile,
            request.options,
            &request.cancellation,
        )
        .map_err(|_| match request.cancellation.check() {
            Err(error) => error,
            Ok(()) => RouteError::NoMatch,
        })
    }

    /// Snaps a point onto the roads `request.profile` may use.
    pub fn snap(&self, request: SnapRequest) -> SnapResult {
        let defaults = CorrelationPolicy::default();
        let policy = CorrelationPolicy {
            max_distance_m: request.max_distance_m.unwrap_or(defaults.max_distance_m),
            candidate_limit: request.candidate_limit.unwrap_or(defaults.candidate_limit),
            ..defaults
        };
        let search = Correlator::new(self.route_graph_view()).search(
            request.point,
            request.profile,
            &policy,
            &request.options,
        );
        SnapResult {
            candidates: search.candidates,
            rejected_candidate_count: search.rejected_candidate_count,
            mode: search.mode,
        }
    }

    /// The package edges a geometry covers, as a route's overlay with this
    /// geometry would: whole edges, and partial ranges where the geometry
    /// covers part of an edge.
    pub fn project(&self, request: &ProjectRequest) -> RouteOverlayGeometryProjection {
        crate::overlay::project_route_overlay_geometry(
            self.route_graph_view(),
            request.geometry_kind,
            &request.points,
            request.projection_distance_m,
        )
    }
}

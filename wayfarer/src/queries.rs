//! The engine's lookups besides routing: map matching a trace, snapping a
//! point onto the road network, and projecting geometry onto package edges.

use serde::{Deserialize, Serialize};

use crate::correlator::Correlator;
use crate::limits::{
    MAX_CORRELATION_DISTANCE_M, MAX_REQUEST_CANDIDATES, MAX_ROUTE_OVERLAY_POINTS,
    MAX_ROUTE_PROJECTION_DISTANCE_M,
};
use crate::map_matching::match_navigation_trace_cancellable;
use crate::trip::invalid_point;
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

impl MapMatchRequest {
    /// Checks the request against [`crate::limits`]: at least one sample, a
    /// radius of at most [`MAX_CORRELATION_DISTANCE_M`] and at most
    /// [`MAX_REQUEST_CANDIDATES`] candidates per sample.
    /// [`RoutingEngine::map_match`] runs it first. A sample's reported
    /// accuracy widens its radius only up to the same limit.
    pub fn validate(&self) -> Result<(), RouteError> {
        if self.samples.is_empty() {
            return Err(RouteError::InvalidRequest {
                reason: "a trace needs at least one sample",
            });
        }
        if self.options.max_correlation_distance_m > MAX_CORRELATION_DISTANCE_M {
            return Err(RouteError::RequestTooLarge {
                what: "correlation distance",
            });
        }
        if self.options.candidate_limit > MAX_REQUEST_CANDIDATES {
            return Err(RouteError::RequestTooLarge {
                what: "candidate limit",
            });
        }
        Ok(())
    }
}

impl SnapRequest {
    /// Checks the request against [`crate::limits`]: valid coordinates, a
    /// `max_distance_m` of at most [`MAX_CORRELATION_DISTANCE_M`] and a
    /// `candidate_limit` of at most [`MAX_REQUEST_CANDIDATES`].
    /// [`RoutingEngine::snap`] cannot fail: it clamps both instead, so a host
    /// passing untrusted input calls this first.
    pub fn validate(&self) -> Result<(), RouteError> {
        if invalid_point(self.point) {
            return Err(RouteError::InvalidRequest {
                reason: "point coordinates",
            });
        }
        if self
            .max_distance_m
            .is_some_and(|distance| distance > MAX_CORRELATION_DISTANCE_M)
        {
            return Err(RouteError::RequestTooLarge {
                what: "correlation distance",
            });
        }
        if self
            .candidate_limit
            .is_some_and(|limit| limit > MAX_REQUEST_CANDIDATES)
        {
            return Err(RouteError::RequestTooLarge {
                what: "candidate limit",
            });
        }
        Ok(())
    }
}

impl ProjectRequest {
    /// Checks the request against the limits route overlays have: at most
    /// [`MAX_ROUTE_OVERLAY_POINTS`] valid points and a `projection_distance_m`
    /// of at most [`MAX_ROUTE_PROJECTION_DISTANCE_M`].
    /// [`RoutingEngine::project`] cannot fail: it clamps the distance
    /// instead, so a host passing untrusted input calls this first.
    pub fn validate(&self) -> Result<(), RouteError> {
        if self.points.len() > MAX_ROUTE_OVERLAY_POINTS {
            return Err(RouteError::RequestTooLarge {
                what: "projection geometry",
            });
        }
        if self.points.iter().copied().any(invalid_point) {
            return Err(RouteError::InvalidRequest {
                reason: "projection geometry",
            });
        }
        if self.projection_distance_m > MAX_ROUTE_PROJECTION_DISTANCE_M {
            return Err(RouteError::RequestTooLarge {
                what: "projection distance",
            });
        }
        Ok(())
    }
}

impl RoutingEngine {
    /// Matches a GPS trace onto the roads `request.profile` may use: the
    /// matched samples, the edges driven and any alternatives. A request
    /// that fails [`MapMatchRequest::validate`] fails with its reason.
    pub fn map_match(&self, request: MapMatchRequest) -> Result<MapMatchResult, RouteError> {
        request.validate()?;
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

    /// Snaps a point onto the roads `request.profile` may use. A radius or
    /// candidate limit beyond [`SnapRequest::validate`]'s bounds is clamped
    /// to them.
    pub fn snap(&self, request: SnapRequest) -> SnapResult {
        let defaults = CorrelationPolicy::default();
        let policy = CorrelationPolicy {
            max_distance_m: request
                .max_distance_m
                .unwrap_or(defaults.max_distance_m)
                .min(MAX_CORRELATION_DISTANCE_M),
            candidate_limit: request
                .candidate_limit
                .unwrap_or(defaults.candidate_limit)
                .min(MAX_REQUEST_CANDIDATES),
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
    /// covers part of an edge. A distance beyond
    /// [`MAX_ROUTE_PROJECTION_DISTANCE_M`] is clamped to it (see
    /// [`ProjectRequest::validate`]).
    pub fn project(&self, request: &ProjectRequest) -> RouteOverlayGeometryProjection {
        crate::overlay::project_route_overlay_geometry(
            self.route_graph_view(),
            request.geometry_kind,
            &request.points,
            request
                .projection_distance_m
                .min(MAX_ROUTE_PROJECTION_DISTANCE_M),
        )
    }
}

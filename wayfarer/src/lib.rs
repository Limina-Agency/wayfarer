//! An offline routing engine over routing packages (`routing.mrt`): routes with
//! alternatives and maneuvers, matrices, isochrones, map matching and
//! navigation progress, on a memory-mapped search graph. Requests can carry
//! dynamic overlays (blocks, avoidances, cost multipliers, connectors) that are
//! costed per request. This crate is library-only: the route compiler writes
//! packages, and the C ABI and service crates build on this one.
//!
//! ```no_run
//! use wayfarer::{
//!     GeoPoint, MobilityProfile, RouteMatrixRequest, RouteOverlay, RouteOverlayGeometryKind,
//!     RouteOverlayPolicy, RouteRequest, RoutingEngine,
//! };
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! // The package is memory-mapped, so opening it is cheap.
//! let engine = RoutingEngine::open("routing.mrt".as_ref())?;
//!
//! let from = GeoPoint::new(43.7397, 7.4275);
//! let via = GeoPoint::new(43.7350, 7.4210);
//! let to = GeoPoint::new(43.7290, 7.4160);
//!
//! // A road closed along a line (edges within 10 m of it are blocked).
//! let closure = RouteOverlay::new(
//!     RouteOverlayGeometryKind::Line,
//!     vec![GeoPoint::new(43.7330, 7.4190), GeoPoint::new(43.7325, 7.4200)],
//!     RouteOverlayPolicy::NoGo,
//!     10,
//! );
//! let request = RouteRequest::through([from, via, to], MobilityProfile::Wheeled)
//!     .with_overlay(closure);
//! let route = engine.route(request)?;
//! println!(
//!     "{:.0} m, {:.0} s, {} maneuvers",
//!     route.distance_m,
//!     route.duration_s,
//!     route.maneuvers.len()
//! );
//!
//! let matrix = engine.matrix(RouteMatrixRequest::new(vec![from, to], vec![to, from], MobilityProfile::OnFoot));
//! println!("{} cells", matrix.cells.len());
//! # Ok(())
//! # }
//! ```
//!
//! # Stability
//!
//! Version 0.x: the API may change between minor releases. An engine opens the
//! package formats `MIN_READ_FORMAT_VERSION..=`[`ROUTING_PACKAGE_FORMAT_VERSION`]
//! and packages of its own or an older costing model ([`COSTING_MODEL_VERSION`]);
//! an older model's package is re-costed from its stored costing inputs, so it
//! routes like a recompiled one, only slower until it is recompiled. The `reference` and `debug-env`
//! features are test and profiling support, not production paths.

// Query-time geometry (snapping, guidance, map matching) may use the platform
// libm: its results never reach a package. The compiler crates may not (see
// `clippy.toml` and `wayfarer_package::math`).
#![allow(clippy::disallowed_methods)]

mod cancellation;
mod correlator;
mod costing;
mod debug;
mod direct_leg;
pub mod geometry;
#[cfg(feature = "reference")]
mod graph_access;
mod graph_view;
mod guidance;
mod isochrone;
pub mod limits;
mod map_matching;
mod matrix;
mod navigation;
mod nearest;
mod overlay;
mod package;
mod queries;
#[cfg(feature = "reference")]
#[doc(hidden)]
pub mod reference;
// Prototype of a search across regions (feature `experimental-regions`): not
// a supported API, changes without notice.
#[cfg(feature = "experimental-regions")]
#[doc(hidden)]
pub mod regions;
mod route_leg;
pub mod sar;
mod search;
mod spatial;
#[cfg(test)]
mod test_support;
mod transitions;
mod trip;
mod types;

/// Engine internals the workspace's harnesses, tests and service use. Not
/// part of the supported API: they change without notice.
#[doc(hidden)]
pub mod internals {
    pub use crate::correlator::{
        CandidateSearch, Correlator, EXPANDED_CORRELATION_DISTANCES_M,
        EXPANDED_CORRELATION_MIN_COMPONENT_SIZE,
    };
    pub use crate::direct_leg::{
        DIRECT_LEG_VERTEX_SPACING_M, MAX_DIRECT_LEG_SEGMENT_COUNT, densify_geodesic,
        direct_route_leg, polyline_length_m,
    };
    #[cfg(feature = "reference")]
    pub use crate::graph_access::{EdgeRecord, GraphAccess, RouteGraphIndex};
    pub use crate::graph_view::{GraphViewEdge, PackageGraphView};
    pub use crate::map_matching::{match_navigation_trace, match_navigation_trace_cancellable};
    pub use crate::overlay::project_route_overlay_geometry;
    pub use crate::package::columns::DerivedColumnProbe;

    /// Turns the CCH potentials of every search `on` (the default) or
    /// `off` (the straight-line bound alone) for the whole process. False
    /// for another name. For harnesses comparing the two in one process.
    pub fn set_potentials(name: &str) -> bool {
        crate::debug::set_potentials(name)
    }
}
pub use package::cch::PotentialStats;
pub use package::{
    PackageCounts, PackageQuality, PackageQualityStatus, ROUTING_PACKAGE_FORMAT_VERSION,
    RoutePackageMemoryProfile, RoutingPackage, RoutingPackageError, RoutingPackageMetadata,
    RoutingPackageSection, RoutingPackageVerification,
};

pub use cancellation::{CancelToken, RouteCancellation};
pub use direct_leg::DirectRouteLeg;
pub use geometry::{
    GeoJsonLineString, GeometryFormat, PolylineError, decode_polyline6, encode_polyline6,
};
pub use map_matching::{
    MapMatchAlternative, MapMatchOptions, MapMatchResult, MapMatchSubPath, MatchedTraceSample,
    RejectedTraceSample,
};
pub use navigation::{
    NavigationPositionSample, PreparedNavigationRoute, RouteNavigationInput,
    RouteNavigationProgress, RouteNavigationProgressOptions, RouteNavigationSnapCandidate,
    RouteNavigationSnapCandidateReason, RouteNavigationSnapCandidateStatus,
    RouteNavigationSnapDiagnostics, RouteNavigationStatus, evaluate_route_navigation_progress,
};
pub use overlay::{
    RouteOverlayDiagnostics, RouteOverlayGeometryKind, RouteOverlayGeometryProjection,
};
pub use queries::{MapMatchRequest, ProjectRequest, SnapRequest, SnapResult};
pub use sar::{
    RouteAccessQualifier, RouteQualifiedAccess, RouteSarCapability, RouteSarOptions,
    RouteSarPolicy, RouteSarRelaxations, RouteSarReport, RouteSarUse, RouteSarUseTotal,
    SarRelaxationKind,
};
pub use search::RoutingEngine;
pub use transitions::{GATE_ON_FOOT_S, GATE_VEHICLE_S};
pub use trip::{
    RouteAlternatives, RouteExplanation, RouteExplanationKind, RouteExplanationSeverity,
    RouteLegMode, RouteLocation, RouteLocationMatch, RouteOverlay, RouteOverlayPolicy,
    RouteOverlaySource, RouteOverlayUse, RouteOverlayWindows, RouteRequest, RouteResult,
    RouteSegment, RouteSegmentShape, RouteSummaryDiagnostics,
    WAYPOINT_CONTINUITY_HEADING_TOLERANCE_DEG,
};
pub use types::{
    CompactEdgeHint, CorrelatedLocation, CorrelationMode, CorrelationPolicy,
    CorrelationRejectReason, CorrelationRoadClassPreference, CorrelationScoreBreakdown,
    CorrelationSide, GeoPoint, LegRequest, LegResult, MobilityProfile, NearestDirection,
    NearestHit, NearestRequest, NearestResult, PreferredRoadSide, RejectedCorrelationCandidate,
    RouteAlternative, RouteAlternativeLabel, RouteAlternativePolicy, RouteCostBreakdown,
    RouteCostDriverKind, RouteCostDriverSummary, RouteCostSummary, RouteCostingOptions,
    RouteDiagnostics, RouteDynamicEndpointKind, RouteDynamicOverlay,
    RouteDynamicOverlayDiagnostics, RouteDynamicSplitEdgeRange, RouteDynamicVirtualEdge,
    RouteDynamicVirtualEdgeKind, RouteEdgeAttribution, RouteEdgeAvoidanceRange,
    RouteEdgeCostMultiplierRange, RouteEdgeRange, RouteEdgeSpeedOverrideRange,
    RouteEndpointOptions, RouteError, RouteFailure, RouteGuidanceQualityFlag,
    RouteIsochroneBoundaryFragment, RouteIsochroneBucket, RouteIsochroneContour,
    RouteIsochroneMetric, RouteIsochroneOrder, RouteIsochroneOrigin, RouteIsochroneRequest,
    RouteIsochroneResult, RouteLane, RouteLaneIndication, RouteLeg, RouteManeuver,
    RouteManeuverKind, RouteMatrixCell, RouteMatrixCellStatus, RouteMatrixRequest,
    RouteMatrixResult, RouteOptions, RoutePolicySummary, RouteRecostSummary,
    RouteShapeDynamicOverlay, RouteShapeDynamicOverlayKind, RouteShapeQualityFlag,
    RouteShapeSegment, RouteSpeedPreference, RouteSpeedSource, RouteStatus,
};
pub use wayfarer_costing::COSTING_MODEL_VERSION;

//! Bounds on a request's size. Requests beyond them fail with
//! `RouteError::RequestTooLarge` (or `InvalidRequest`), so a host can pass
//! untrusted input straight through.

pub use crate::trip::{
    MAX_ROUTE_LOCATION_HINTS, MAX_ROUTE_LOCATIONS, MAX_ROUTE_OUTPUT_EDGE_IDS,
    MAX_ROUTE_OUTPUT_GEOMETRY_POINTS, MAX_ROUTE_OUTPUT_MANEUVERS, MAX_ROUTE_OUTPUT_RECOSTS,
    MAX_ROUTE_OVERLAY_POINTS, MAX_ROUTE_OVERLAY_SPEED_KPH, MAX_ROUTE_OVERLAYS,
    MAX_ROUTE_PROJECTION_DISTANCE_M, MAX_ROUTE_PROVENANCE_VALUES, MAX_ROUTE_STRING_BYTES,
    MAX_ROUTE_TOTAL_STRING_BYTES,
};

/// The most correlation candidates a request may ask for.
pub const MAX_REQUEST_CANDIDATES: usize = 64;
/// The most edge ids and ranges (blocks, avoidances, multipliers, …) one
/// request may carry.
pub const MAX_REQUEST_EDGE_CONSTRAINTS: usize = 100_000;
/// The most dynamic-overlay connectors one request may carry.
pub const MAX_REQUEST_DYNAMIC_CONNECTORS: usize = 10_000;
/// The most dynamic-overlay connector geometry points one request may carry.
pub const MAX_REQUEST_DYNAMIC_GEOMETRY_POINTS: usize = 100_000;

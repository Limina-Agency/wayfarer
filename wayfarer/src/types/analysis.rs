//! Matrix and isochrone requests and results.

use super::*;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteMatrixRequest {
    pub sources: Vec<GeoPoint>,
    pub targets: Vec<GeoPoint>,
    pub profile: MobilityProfile,
    #[serde(default)]
    pub options: RouteOptions,
    /// Closures and other overlays, as geometry or package edges, applied as
    /// a route applies them.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub overlays: Vec<crate::RouteOverlay>,
    /// The time overlay windows are evaluated at (UTC ms), unless the
    /// costing options set `conditional_restriction_eval_at_ms`; absent
    /// both, the time of the request.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub departure_at_ms: Option<i64>,
    /// `false` leaves out the cells' `edge_ids` (the search is the same; the
    /// output is smaller).
    #[serde(default = "include_edge_ids", skip_serializing_if = "is_true")]
    pub include_edge_ids: bool,
}

impl RouteMatrixRequest {
    pub fn new(sources: Vec<GeoPoint>, targets: Vec<GeoPoint>, profile: MobilityProfile) -> Self {
        Self {
            sources,
            targets,
            profile,
            options: RouteOptions::default(),
            overlays: Vec::new(),
            departure_at_ms: None,
            include_edge_ids: true,
        }
    }
}

fn include_edge_ids() -> bool {
    true
}

fn is_true(value: &bool) -> bool {
    *value
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteMatrixCellStatus {
    Reachable,
    Unreachable,
    NoCandidate,
    Failed,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteMatrixCell {
    pub source_index: usize,
    pub target_index: usize,
    pub status: RouteMatrixCellStatus,
    #[serde(default)]
    pub distance_m: Option<f64>,
    #[serde(default)]
    pub duration_s: Option<f64>,
    #[serde(default)]
    pub cost: Option<f64>,
    #[serde(default)]
    pub edge_ids: Vec<u64>,
    #[serde(default)]
    pub failure_reason: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteMatrixResult {
    pub source_count: usize,
    pub target_count: usize,
    pub cells: Vec<RouteMatrixCell>,
    /// Source rows searched.
    #[serde(default)]
    pub source_expansion_count: usize,
    /// Out-edges examined, summed over every search of the request (see
    /// `RouteOptions::max_visited_edge_count` for what one search is).
    #[serde(default)]
    pub visited_edge_count: usize,
    #[serde(default)]
    pub settled_node_count: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteIsochroneMetric {
    Cost,
    TimeSeconds,
    /// Metres along the cheapest path (as time is: labels are ordered by
    /// cost whatever the metric).
    DistanceM,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteIsochroneContour {
    pub contour: f64,
    pub metric: RouteIsochroneMetric,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteIsochroneRequest {
    /// The first origin. With `origins`, it is origin 0 of several.
    pub origin: GeoPoint,
    /// More origins after `origin`, at most 64 in all
    /// (`limits::MAX_ISOCHRONE_ORIGINS`): the isochrone is everywhere
    /// reachable from any of them, each edge at its least value from any
    /// origin. Each origin snaps as `origin` does, and a farther candidate's
    /// approach cost is relative to its own origin's nearest one. Every
    /// origin must snap, or the request fails with `no_candidate`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub origins: Vec<RouteIsochroneOrigin>,
    pub profile: MobilityProfile,
    pub contours: Vec<RouteIsochroneContour>,
    #[serde(default)]
    pub options: RouteOptions,
    /// Expansion budget: the most out-edges the search may examine (the
    /// result's `visited_edge_count`). When the next edge would exceed it, the
    /// search stops and the result is marked `truncated`; its buckets then hold
    /// only what was reached so far. `options.max_visited_edge_count` does not
    /// apply to isochrones.
    #[serde(default)]
    pub max_expansion_edge_count: Option<usize>,
    /// Closures and other overlays, as geometry or package edges, applied as
    /// a route applies them.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub overlays: Vec<crate::RouteOverlay>,
    /// The time overlay windows are evaluated at (UTC ms), unless the
    /// costing options set `conditional_restriction_eval_at_ms`; absent
    /// both, the time of the request.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub departure_at_ms: Option<i64>,
    /// `false` leaves out the buckets' `edge_ids`; boundary fragments keep
    /// theirs (the search is the same; the output is smaller).
    #[serde(default = "include_edge_ids", skip_serializing_if = "is_true")]
    pub include_edge_ids: bool,
}

/// One more origin of a multi-origin isochrone (see
/// `RouteIsochroneRequest::origins`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteIsochroneOrigin {
    pub point: GeoPoint,
}

impl RouteIsochroneOrigin {
    pub fn new(point: GeoPoint) -> Self {
        Self { point }
    }
}

impl RouteIsochroneRequest {
    pub fn new(
        origin: GeoPoint,
        profile: MobilityProfile,
        contours: Vec<RouteIsochroneContour>,
    ) -> Self {
        Self {
            origin,
            origins: Vec::new(),
            profile,
            contours,
            options: RouteOptions::default(),
            max_expansion_edge_count: None,
            overlays: Vec::new(),
            departure_at_ms: None,
            include_edge_ids: true,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteIsochroneBucket {
    pub contour: f64,
    pub metric: RouteIsochroneMetric,
    pub edge_ids: Vec<u64>,
    #[serde(default)]
    pub boundary_fragments: Vec<RouteIsochroneBoundaryFragment>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteIsochroneBoundaryFragment {
    pub edge_id: u64,
    pub start: GeoPoint,
    pub end: GeoPoint,
    pub start_fraction: f64,
    pub end_fraction: f64,
    pub start_metric: f64,
    pub end_metric: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteIsochroneResult {
    pub origin: CorrelatedLocation,
    /// Where each of the request's `origins` snapped, in order (origin 0 is
    /// `origin`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub origins: Vec<CorrelatedLocation>,
    pub buckets: Vec<RouteIsochroneBucket>,
    pub visited_edge_count: usize,
    pub settled_node_count: usize,
    /// The search stopped at `max_expansion_edge_count` before exhausting the
    /// largest contour: buckets and boundary fragments are incomplete (every
    /// edge listed is reachable, but some reachable edges are missing).
    #[serde(default)]
    pub truncated: bool,
}

/// Which way a nearest-targets request measures travel.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum NearestDirection {
    /// From the source to each target (where can I get to soonest).
    #[default]
    FromSource,
    /// From each target to the source (which targets get here soonest).
    ToSource,
}

/// The `k` targets cheapest to travel to from `source` (or from which to
/// travel to it), by route cost ([`crate::RoutingEngine::nearest`]).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct NearestRequest {
    pub source: GeoPoint,
    pub targets: Vec<GeoPoint>,
    /// How many targets to return, at least 1 and at most `targets`' count.
    pub k: usize,
    pub profile: MobilityProfile,
    #[serde(default)]
    pub direction: NearestDirection,
    #[serde(default)]
    pub options: RouteOptions,
    /// Closures and other overlays, as geometry or package edges, applied as
    /// a route applies them.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub overlays: Vec<crate::RouteOverlay>,
    /// The time overlay windows are evaluated at (UTC ms), unless the
    /// costing options set `conditional_restriction_eval_at_ms`; absent
    /// both, the time of the request.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub departure_at_ms: Option<i64>,
    /// `false` leaves out the hits' `edge_ids` (the search is the same; the
    /// output is smaller).
    #[serde(default = "include_edge_ids", skip_serializing_if = "is_true")]
    pub include_edge_ids: bool,
}

impl NearestRequest {
    pub fn new(
        source: GeoPoint,
        targets: Vec<GeoPoint>,
        k: usize,
        profile: MobilityProfile,
    ) -> Self {
        Self {
            source,
            targets,
            k,
            profile,
            direction: NearestDirection::FromSource,
            options: RouteOptions::default(),
            overlays: Vec::new(),
            departure_at_ms: None,
            include_edge_ids: true,
        }
    }
}

/// One of the nearest targets: its matrix cell's path between the source
/// and the target, in the request's direction.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct NearestHit {
    pub target_index: usize,
    pub cost: f64,
    pub distance_m: f64,
    pub duration_s: f64,
    pub edge_ids: Vec<u64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct NearestResult {
    pub target_count: usize,
    /// At most `k` targets, cheapest first (ties by target index): the `k`
    /// cheapest reachable cells of the matrix between the source and every
    /// target. Fewer when fewer targets are reachable.
    pub hits: Vec<NearestHit>,
    /// Targets with no road within the correlation radius.
    #[serde(default)]
    pub no_candidate_target_indexes: Vec<usize>,
    /// Matrix rows searched (a row's cells may be searched one by one).
    pub row_count: usize,
    /// Out-edges examined, summed over every search.
    pub visited_edge_count: usize,
}

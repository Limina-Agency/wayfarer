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

fn is_false(value: &bool) -> bool {
    !*value
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
    /// Per source, the correlation distance its snapping widened to (300 m,
    /// 1 km, or a SAR request's `off_network` reach) because no candidate
    /// within `max_correlation_distance_m` was connected to the network, or
    /// none; empty (and absent from JSON) when no source widened.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub source_expanded_correlation_distances_m: Vec<Option<u32>>,
    /// [`Self::source_expanded_correlation_distances_m`] for the targets.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub target_expanded_correlation_distances_m: Vec<Option<u32>>,
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

/// How an isochrone expansion orders the paths it extends.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteIsochroneOrder {
    /// By cost, as routes choose: each edge is reached along its cheapest
    /// path, and time and distance contours measure that path.
    #[default]
    Cost,
    /// By travel time (fastest-time contours): each edge is reached along
    /// its fastest path, and cost and distance contours measure that path.
    /// Time starts at each origin's nearest road (farther candidates are
    /// not seeded), plus the origin's `startOffsetS`.
    Time,
}

impl RouteIsochroneOrder {
    fn is_cost(&self) -> bool {
        *self == Self::Cost
    }
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
    /// How the expansion orders paths: by cost (default) or by travel time
    /// (fastest-time contours, which start offsets need).
    #[serde(default, skip_serializing_if = "RouteIsochroneOrder::is_cost")]
    pub order: RouteIsochroneOrder,
    /// `true` reports which origin reaches each edge first on each bucket's
    /// metric (`edgeOrigins`, parallel to `edgeIds`) and which origin's
    /// path each boundary fragment ends (`origin`); 0 is `origin`, 1 the
    /// first of `origins`, and so on.
    #[serde(default, skip_serializing_if = "is_false")]
    pub include_origins: bool,
    /// Draws each bucket as a GeoJSON `MultiPolygon` (`polygon`): the
    /// area within a short walk of the roads the bucket reaches. Left out
    /// (the default), buckets carry no polygon.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub polygons: Option<RouteIsochronePolygonOptions>,
}

/// How an isochrone draws its buckets as polygons (see
/// `RouteIsochroneRequest::polygons`). The polygon is drawn on a square grid
/// over the reached roads: a grid node is inside when it lies within
/// `spread_m` of a reached road, where on a boundary fragment the spread
/// shrinks to what remains of the contour at the profile's off-road pace
/// (metres for a distance contour), down to nothing where the fragment ends.
/// Rings follow the grid (marching squares), so the polygon is exact only to
/// about a cell; they never touch, before or after rounding. The bucket's `edgeIds` and boundary fragments stay the
/// exact answer; the polygon is a function of them and the package's road
/// shapes. Every value is optional; the result's `polygons` echoes the
/// values used.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct RouteIsochronePolygonOptions {
    /// The grid's cell size in metres, from 2 to 10,000. By default the
    /// larger side of the reached area over 256, rounded up to whole metres
    /// and kept within 10 m to 500 m.
    /// A grid that would exceed `limits::MAX_ISOCHRONE_POLYGON_GRID_NODES`
    /// nodes takes larger cells.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub resolution_m: Option<f64>,
    /// How far off a reached road the polygon reaches, in metres, at most
    /// 10,000. By default 1.5 cells; never less than 0.75 of a cell, so
    /// that every reached road meets the grid.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spread_m: Option<f64>,
    /// Drops each polygon whose area is less than this share of the largest
    /// polygon's in its bucket (0 to 1; by default 0, which keeps every
    /// one). Holes are kept.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub denoise: Option<f64>,
}

/// One more origin of a multi-origin isochrone (see
/// `RouteIsochroneRequest::origins`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteIsochroneOrigin {
    pub point: GeoPoint,
    /// Seconds after the other origins' start that this one starts ("team 3
    /// leaves in 10 minutes"): its time contours are that much shorter.
    /// Needs `order: time`; finite and not negative.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub start_offset_s: Option<f64>,
}

impl RouteIsochroneOrigin {
    pub fn new(point: GeoPoint) -> Self {
        Self {
            point,
            start_offset_s: None,
        }
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
            order: RouteIsochroneOrder::Cost,
            include_origins: false,
            polygons: None,
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
    /// With `includeOrigins`, the origin that reaches each of `edge_ids`
    /// first on this bucket's metric, parallel to it.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub edge_origins: Vec<u16>,
    #[serde(default)]
    pub boundary_fragments: Vec<RouteIsochroneBoundaryFragment>,
    /// With `polygons`, the bucket drawn as a GeoJSON `MultiPolygon` (see
    /// `RouteIsochronePolygonOptions`); empty when the bucket reaches
    /// nothing.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub polygon: Option<crate::GeoJsonMultiPolygon>,
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
    /// With `includeOrigins`, the origin whose path the fragment ends.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub origin: Option<u16>,
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
    /// With the request's `polygons`, the polygon options as applied: the
    /// grid's cell size, the spread and the denoise share.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub polygons: Option<RouteIsochronePolygonOptions>,
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

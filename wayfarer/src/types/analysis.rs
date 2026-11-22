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
}

impl RouteMatrixRequest {
    pub fn new(sources: Vec<GeoPoint>, targets: Vec<GeoPoint>, profile: MobilityProfile) -> Self {
        Self {
            sources,
            targets,
            profile,
            options: RouteOptions::default(),
        }
    }
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
    pub origin: GeoPoint,
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
}

impl RouteIsochroneRequest {
    pub fn new(
        origin: GeoPoint,
        profile: MobilityProfile,
        contours: Vec<RouteIsochroneContour>,
    ) -> Self {
        Self {
            origin,
            profile,
            contours,
            options: RouteOptions::default(),
            max_expansion_edge_count: None,
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

//! Search and rescue planning over the offline routing engine.
//!
//! Three steps a search planner runs, each a pure function of the routing
//! package, the request and the request's explicit times (no clock is read,
//! so a plan can be replayed in review):
//!
//! - [`predict_reach`]: from an approximate last known position and the time
//!   since the person was last seen, the network they could have reached
//!   (fast, typical and slow person), a probability per edge (probability of
//!   area on the network), the likely routes to the edge of that area and to
//!   named destinations, and the corridors those routes share. Zones the
//!   person cannot enter, tends to avoid, or is drawn to shape both.
//! - [`plan_teams`]: for search segments and teams, each segment's
//!   probability, every team's arrival time at every segment, a first
//!   assignment and, with a shift length, each team's territory.
//! - [`apply_coverage`]: team GPS tracks map-matched into covered paths,
//!   search effort the host measured itself (a probability of detection or
//!   seconds seen, per edge, per grid cell or over a polygon), a probability
//!   of detection per segment, and the probabilities updated after an
//!   unsuccessful search.
//!
//! The model is described in `docs/design/sar-planning.md`. Probability
//! lives on the road and trail network; a position off the paths reaches
//! them over a priced walk (off-trail access legs, on by default), and
//! reach polygons can be asked for. Probability over open terrain comes with
//! the incident surface.
//!
//! # Typed Rust API
//!
//! Requests and results are plain Rust types: build them with their `new`
//! constructors and set the fields that differ from the defaults. Every type
//! is also `serde` (camelCase JSON, the same shape as the CLI and the C ABI
//! take), but a Rust host never needs JSON.
//!
//! ```no_run
//! use wayfarer::{GeoPoint, RoutingEngine};
//! use wayfarer_sar::{ReachRequest, predict_reach};
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let engine = RoutingEngine::open("routing.mrt".as_ref())?;
//! let mut request = ReachRequest::new(GeoPoint::new(49.2325, 19.9818));
//! request.elapsed_s = Some(3.0 * 3600.0);
//! request.uncertainty_m = 200.0;
//! let reach = predict_reach(&engine, &request)?;
//! for area in &reach.areas {
//!     println!("{:?}: {:.1} km of path within {:.0} m", area.person, area.network_length_m / 1e3, area.max_radius_m);
//! }
//! # Ok(())
//! # }
//! ```
//!
//! A host that already serves routes through `wayfarer-service` runs the
//! steps on the engine the service has cached for a source, with no second
//! open of the package, through `RoutingService::execute`:
//!
//! ```no_run
//! use wayfarer::{GeoPoint, RoutePackageMemoryProfile};
//! use wayfarer_sar::{
//!     CoverageRequest, Detection, EdgeEffort, EffortArea, ReachRequest, SearchEffort,
//!     apply_coverage, predict_reach,
//! };
//! use wayfarer_service::{RoutingService, RoutingSource, RoutingSourceId};
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let service = RoutingService::new(4, RoutePackageMemoryProfile::Mobile);
//! let source = RoutingSource::new(RoutingSourceId::new("tatry")?, "2026-11", "tatry.mrt")?;
//! let (reach, coverage) = service
//!     .execute(&source, |engine, _package, _readiness| {
//!         let mut request = ReachRequest::new(GeoPoint::new(49.2325, 19.9818));
//!         request.elapsed_s = Some(3.0 * 3600.0);
//!         let reach = predict_reach(engine, &request)?;
//!         // A drone's video coverage, scored by the host as seconds seen.
//!         let key = reach.probability.edges[0].edge_id;
//!         let mut effort = SearchEffort::new(
//!             "drone-1",
//!             EffortArea::Edges { edges: vec![EdgeEffort::new(key, Detection::SeenS(40.0))] },
//!         );
//!         effort.tau_s = Some(30.0);
//!         let mut search = CoverageRequest::new(Vec::new(), reach.probability.clone());
//!         search.efforts = vec![effort];
//!         let coverage = apply_coverage(engine, &search)?;
//!         Ok::<_, wayfarer_sar::SarError>((reach, coverage))
//!     })
//!     .map_err(|readiness| format!("source not ready: {:?}", readiness.status))??;
//! println!("{} edges, P(success) {:.2}", reach.probability.edges.len(), coverage.probability_of_success);
//! # Ok(())
//! # }
//! ```
//!
//! # Stability
//!
//! Across 0.x, changes to these types are additive. Every public struct and
//! enum is `#[non_exhaustive]`: a release may add a field (with a serde
//! default, so older JSON still reads), an enum variant or an error, and
//! that is not a breaking change. Hosts therefore build requests with the
//! constructors rather than struct literals, and match enums with a
//! wildcard arm. Removing or renaming a field, changing its meaning or
//! units, or changing a default that moves results is a breaking change: it
//! waits for a minor release (0.x to 0.x+1) and is listed in the changelog.
//! The JSON schemas in `schema/sar-*.json` follow the same rule.

mod coverage;
mod effort;
pub mod geojson;
mod network;
mod pace;
mod reach;
mod teams;

use serde::{Deserialize, Serialize};
use wayfarer::{GeoPoint, RouteError};

pub use coverage::{
    CoverageRequest, CoverageResult, CoveredEdge, SegmentCoverage, TeamTrack, TrackCoverage,
    apply_coverage,
};
pub use effort::{
    CellEffort, Detection, EdgeEffort, EffortArea, EffortCoverage, MAX_EFFORT_CELL_DEG,
    MAX_EFFORT_ITEMS, MAX_EFFORTS, SearchEffort,
};
pub use network::SAR_ACCESS_RADIUS_WALKING_M;
pub use pace::{PersonFactors, PersonPercentile};
pub use reach::{
    DistanceRing, EdgeProbability, LikelyRoute, NamedPoint, ProbabilityMap, ReachArea,
    ReachRequest, ReachResult, RouteCorridorEdge, RouteTarget, Stopping, predict_reach,
};
pub use teams::{
    SarTeam, SearchSegment, SegmentPlan, TeamArrival, TeamAssignment, TeamPace, TeamPlanRequest,
    TeamPlanResult, TeamTerritory, plan_teams,
};

/// How a zone shapes the prediction (see `docs/design/sar-planning.md` 2.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
#[non_exhaustive]
pub enum SarZoneKind {
    /// The person cannot enter it (a river, a cliff band): blocked in every
    /// search, so it holds no probability.
    NoGo,
    /// The person tends to avoid it (dense forest, a steep slope, private
    /// ground): blocked in the complying search only, and the result is a
    /// mixture of the complying and the free search by
    /// `ReachRequest::avoid_compliance`.
    Avoid,
    /// The search is unchanged; the probability of the edges inside is
    /// multiplied by `factor` (above 1 for places the person is drawn to,
    /// below 1 for places they are unlikely to stop), then renormalised.
    Weight,
}

/// A zone the planner draws, as a closed polygon (the last point need not
/// repeat the first).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
#[non_exhaustive]
pub struct SarZone {
    /// The planner's label, echoed in results.
    pub id: String,
    pub kind: SarZoneKind,
    /// At least three points.
    pub polygon: Vec<GeoPoint>,
    /// `weight` zones: the probability multiplier, finite and above 0.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub factor: Option<f64>,
}

impl SarZone {
    pub fn new(id: impl Into<String>, kind: SarZoneKind, polygon: Vec<GeoPoint>) -> Self {
        Self {
            id: id.into(),
            kind,
            polygon,
            factor: None,
        }
    }

    fn validate(&self) -> Result<(), SarError> {
        if self.polygon.len() < 3 || self.polygon.iter().any(|point| invalid_point(*point)) {
            return Err(SarError::invalid(format!(
                "zone {} needs a polygon of at least three valid points",
                self.id
            )));
        }
        match (self.kind, self.factor) {
            (SarZoneKind::Weight, Some(factor)) if factor.is_finite() && factor > 0.0 => Ok(()),
            (SarZoneKind::Weight, _) => Err(SarError::invalid(format!(
                "weight zone {} needs a finite factor above 0",
                self.id
            ))),
            (_, Some(_)) => Err(SarError::invalid(format!(
                "zone {}: only weight zones take a factor",
                self.id
            ))),
            (_, None) => Ok(()),
        }
    }
}

/// Why a SAR request failed.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum SarError {
    /// The request is malformed or out of bounds.
    #[error("invalid SAR request: {reason}")]
    InvalidRequest { reason: String },
    /// An engine query failed (no candidate near the last known position, an
    /// impossible profile, a cancelled request, ...).
    #[error(transparent)]
    Route(#[from] RouteError),
}

impl SarError {
    fn invalid(reason: impl Into<String>) -> Self {
        Self::InvalidRequest {
            reason: reason.into(),
        }
    }
}

fn invalid_point(point: GeoPoint) -> bool {
    !point.lat.is_finite()
        || !point.lng.is_finite()
        || !(-90.0..=90.0).contains(&point.lat)
        || !(-180.0..=180.0).contains(&point.lng)
}

fn finite_positive(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

fn unit_interval(value: f64) -> bool {
    value.is_finite() && (0.0..=1.0).contains(&value)
}

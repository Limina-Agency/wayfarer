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
//! - [`apply_coverage`]: team GPS tracks map-matched into covered paths, a
//!   probability of detection per segment, and the probabilities updated
//!   after an unsuccessful search.
//!
//! The model is described in `docs/design/sar-planning.md`. This first slice
//! runs on the road and trail network only; off-network reach (terrain cells)
//! comes with the incident surface.
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

mod coverage;
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

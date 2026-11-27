//! Costed off-network access legs (FEAT-X1, and the SAR `off_network.costed`
//! flag of FEAT-X6).
//!
//! Without them a point off the road network snaps to its nearest road for
//! free: the stretch between the point and the road is in no distance,
//! duration, cost or geometry, and farther candidates only pay a relative
//! tie-breaking penalty (`package::search::approach_costs`).
//!
//! With [`RouteOffNetworkOptions`] on a request (`offNetwork` in JSON) every
//! route, matrix and isochrone point snaps within one radius
//! ([`RouteOffNetworkOptions::max_radius_m`], no 300 m / 1 km widening) to up
//! to [`RouteOffNetworkOptions::max_candidates`] directed edges, and each
//! candidate, the nearest included, carries a [`CandidateAccess`]: a
//! straight stretch from the requested point to the candidate's snap point,
//! `d` metres long, that takes `d / pace` seconds and costs
//! `cost_factor · d / pace`. The searches already seed and finish each
//! candidate at its approach cost and leave it out of the reported cost, so
//! they stay exact over the candidate set and their bounds stay admissible
//! (an access cost is a super-source arc). Results then add the chosen
//! candidate's stretch back (`AccessTotals::of`):
//!
//! - routes: distance, duration and cost include both stretches; the leg's
//!   geometry starts and ends at the requested points, each stretch a shape
//!   segment flagged [`RouteShapeQualityFlag::OffNetworkAccess`] with a
//!   [`RouteShapeDynamicOverlayKind::AccessLeg`] overlay, with its own
//!   maneuver, and [`RouteOffNetworkSummary`] on the leg;
//! - matrices: cell distance, duration and cost include both stretches;
//! - isochrones: reach starts at each candidate's access cost, time and
//!   distance.
//!
//! Everything is query time: no package and no costing-model version
//! changes. The default pricing is [`wayfarer_costing::profile_access`]
//! (decision D4: the profile's off-road pace, a cost factor of 2 for walkers
//! and 1.5 for vehicles; radius 500 m for walkers and 250 m for vehicles; 16
//! candidates, at most 64). See `docs/design/off-road.md`.

use serde::{Deserialize, Serialize};

use crate::direct_leg::{DIRECT_LEG_VERTEX_SPACING_M, densify_geodesic};
use crate::guidance::{bearing_deg, cardinal_direction};
use crate::types::{
    CorrelatedLocation, GeoPoint, LegResult, MobilityProfile, RouteError, RouteManeuver,
    RouteManeuverKind, RouteShapeDynamicOverlay, RouteShapeDynamicOverlayKind,
    RouteShapeQualityFlag, RouteShapeSegment, haversine_m,
};

/// The default access radius for the walking profiles (OnFoot, Legged).
pub const ACCESS_DEFAULT_RADIUS_WALKING_M: f64 = 500.0;
/// The default access radius for the vehicle profiles.
pub const ACCESS_DEFAULT_RADIUS_VEHICLE_M: f64 = 250.0;
/// The widest access radius a request may ask for (the SAR `off_network`
/// maximum, [`crate::sar::SAR_MAX_OFF_NETWORK_LEG_M`]).
pub const ACCESS_MAX_RADIUS_M: f64 = 10_000.0;
/// Directed candidates per point by default (about 8 roads).
pub const ACCESS_DEFAULT_CANDIDATES: usize = 16;
/// The most directed candidates per point a request may ask for.
pub const ACCESS_MAX_CANDIDATES: usize = crate::limits::MAX_REQUEST_CANDIDATES;
/// The fastest access pace a request may set (m/s; 180 km/h).
pub const ACCESS_MAX_PACE_MPS: f64 = 50.0;
/// The largest access cost factor a request may set.
pub const ACCESS_MAX_COST_FACTOR: f64 = 100.0;
/// Stretches shorter than this are in a route's totals and summary but get no
/// geometry, shape segment or maneuver of their own.
pub const ACCESS_MIN_DRAWN_M: f64 = 1.0;

/// Costed off-network access legs on a request (`RouteOptions::off_network`).
/// Every field is optional: `"offNetwork": {}` turns access legs on with the
/// profile's defaults.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", default)]
pub struct RouteOffNetworkOptions {
    /// How far from a point to look for the road network, metres: finite,
    /// above 0 and at most [`ACCESS_MAX_RADIUS_M`]. Defaults to
    /// [`ACCESS_DEFAULT_RADIUS_WALKING_M`] for walkers and
    /// [`ACCESS_DEFAULT_RADIUS_VEHICLE_M`] for vehicles. It replaces
    /// `maxCorrelationDistanceM` and its widening.
    #[serde(alias = "max_radius_m", skip_serializing_if = "Option::is_none")]
    pub max_radius_m: Option<f64>,
    /// Directed candidate edges per point, 1 to [`ACCESS_MAX_CANDIDATES`]
    /// (default [`ACCESS_DEFAULT_CANDIDATES`]). It replaces
    /// `candidateLimit`.
    #[serde(alias = "max_candidates", skip_serializing_if = "Option::is_none")]
    pub max_candidates: Option<usize>,
    /// Pace across open ground, m/s: above 0 and at most
    /// [`ACCESS_MAX_PACE_MPS`]. Defaults to the profile's off-road pace (the
    /// pace of direct legs).
    #[serde(alias = "pace_mps", skip_serializing_if = "Option::is_none")]
    pub pace_mps: Option<f64>,
    /// Cost per second of access time, at least 1 and at most
    /// [`ACCESS_MAX_COST_FACTOR`]: above 1, roads stay preferred over open
    /// ground at the same pace. Defaults to 2 for walkers and 1.5 for
    /// vehicles.
    #[serde(alias = "cost_factor", skip_serializing_if = "Option::is_none")]
    pub cost_factor: Option<f64>,
    /// Prices each endpoint's access itself, in place of the straight-line
    /// formula (a terrain search, `docs/design/portals.md`). Rust only, not
    /// on the wire.
    #[serde(skip)]
    #[cfg_attr(feature = "schema", schemars(skip))]
    pub pricer: Option<AccessPricerHandle>,
}

/// Which end of a search a correlated point is: its access stretch runs from
/// the point for an origin and to it for a destination, which a pricer that
/// reads slopes needs to know.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AccessRole {
    Origin,
    Destination,
}

/// What a pricer gets for one endpoint.
pub struct AccessQuery<'a> {
    pub point: GeoPoint,
    pub profile: MobilityProfile,
    pub role: AccessRole,
    /// The resolved options: radius, candidate count, pace and factor.
    pub policy: AccessPolicy,
    /// The candidates within the radius, priced by the straight-line formula:
    /// up to [`ACCESS_MAX_CANDIDATES`] of them, more than the policy's
    /// candidate count, so that the pricer chooses among them.
    pub candidates: Vec<CorrelatedLocation>,
    /// The package, for edge shapes and attributes.
    pub view: &'a crate::graph_view::PackageGraphView,
}

/// Prices access stretches for a request with costed access legs
/// (`RouteOffNetworkOptions::pricer`). It returns the candidates the endpoint
/// keeps, each with a [`CandidateAccess`] whose [`CandidateAccess::priced`]
/// it may set: the searches, the oracle and the results then read that price.
/// It must return the same candidates for the same query, so engine and
/// oracle agree, best first: the endpoint keeps the first
/// [`AccessPolicy::candidate_limit`] of them.
pub trait AccessPricer: Send + Sync {
    fn price(&self, query: AccessQuery<'_>) -> Vec<CorrelatedLocation>;
}

/// A shared [`AccessPricer`]. Two handles are equal when they hold the same
/// pricer.
#[derive(Clone)]
pub struct AccessPricerHandle(pub std::sync::Arc<dyn AccessPricer>);

impl std::fmt::Debug for AccessPricerHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("AccessPricerHandle")
    }
}

impl PartialEq for AccessPricerHandle {
    fn eq(&self, other: &Self) -> bool {
        std::sync::Arc::ptr_eq(&self.0, &other.0)
    }
}

// Requests are validated before they search (`validate`), and every value
// that passes is finite, so equality is total where it is used.
impl Eq for RouteOffNetworkOptions {}

impl RouteOffNetworkOptions {
    /// Checks the values against the bounds above.
    pub fn validate(&self) -> Result<(), RouteError> {
        let invalid = |reason| Err(RouteError::InvalidRequest { reason });
        if self.max_radius_m.is_some_and(|radius| {
            !(radius.is_finite() && radius > 0.0 && radius <= ACCESS_MAX_RADIUS_M)
        }) {
            return invalid("off_network max_radius_m");
        }
        if self
            .max_candidates
            .is_some_and(|count| count == 0 || count > ACCESS_MAX_CANDIDATES)
        {
            return invalid("off_network max_candidates");
        }
        if self
            .pace_mps
            .is_some_and(|pace| !(pace.is_finite() && pace > 0.0 && pace <= ACCESS_MAX_PACE_MPS))
        {
            return invalid("off_network pace_mps");
        }
        if self.cost_factor.is_some_and(|factor| {
            !(factor.is_finite() && (1.0..=ACCESS_MAX_COST_FACTOR).contains(&factor))
        }) {
            return invalid("off_network cost_factor");
        }
        Ok(())
    }

    /// The options with `profile`'s defaults filled in.
    pub fn resolve(&self, profile: MobilityProfile) -> AccessPolicy {
        let defaults = wayfarer_costing::profile_access(profile);
        let walking = matches!(profile, MobilityProfile::OnFoot | MobilityProfile::Legged);
        let radius_m = self.max_radius_m.unwrap_or(if walking {
            ACCESS_DEFAULT_RADIUS_WALKING_M
        } else {
            ACCESS_DEFAULT_RADIUS_VEHICLE_M
        });
        AccessPolicy {
            radius_m: radius_m.clamp(f64::MIN_POSITIVE, ACCESS_MAX_RADIUS_M),
            candidate_limit: self
                .max_candidates
                .unwrap_or(ACCESS_DEFAULT_CANDIDATES)
                .clamp(1, ACCESS_MAX_CANDIDATES),
            pace_mps: self.pace_mps.unwrap_or(defaults.pace_mps),
            cost_factor: self.cost_factor.unwrap_or(defaults.cost_factor),
        }
    }
}

/// [`RouteOffNetworkOptions`] resolved for one profile.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AccessPolicy {
    pub radius_m: f64,
    pub candidate_limit: usize,
    pub pace_mps: f64,
    pub cost_factor: f64,
}

impl AccessPolicy {
    /// The correlation radius (whole metres, rounded up).
    pub(crate) fn radius_whole_m(&self) -> u32 {
        self.radius_m.ceil() as u32
    }

    /// The access of a candidate snapped from `point`.
    pub(crate) fn access_from(&self, point: GeoPoint) -> CandidateAccess {
        CandidateAccess {
            point,
            pace_mps: self.pace_mps,
            cost_factor: self.cost_factor,
            priced: None,
        }
    }
}

/// The costed access stretch of one correlation candidate: from the
/// requested `point` to the candidate's snap point: a straight line, its
/// length the candidate's `distance_m`, priced at `cost_factor · d / pace`,
/// unless the Rust-only `priced` holds a price and path found by the caller
/// (a terrain search, `docs/design/portals.md`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct CandidateAccess {
    /// The requested point the stretch runs from (an origin) or to (a
    /// destination).
    pub point: GeoPoint,
    pub pace_mps: f64,
    pub cost_factor: f64,
    /// A price found by the caller, used in place of the straight-line
    /// formula by every search, the oracle and the results. Not on the wire.
    #[serde(skip)]
    #[cfg_attr(feature = "schema", schemars(skip))]
    pub priced: Option<std::sync::Arc<PricedAccess>>,
}

impl CandidateAccess {
    /// The time to cross `distance_m` of open ground in a straight line.
    pub fn duration_s(&self, distance_m: f64) -> f64 {
        distance_m / self.pace_mps
    }

    /// The cost of crossing `distance_m` of open ground in a straight line:
    /// its time times the cost factor.
    pub fn cost(&self, distance_m: f64) -> f64 {
        self.cost_factor * self.duration_s(distance_m)
    }

    /// The stretch's cost, time and length for a candidate `distance_m` from
    /// the point: the priced values when there are any, else the
    /// straight-line formula. Searches charge its cost in a candidate's seed
    /// or finish.
    pub(crate) fn totals(&self, distance_m: f64) -> AccessTotals {
        match &self.priced {
            Some(priced) => AccessTotals {
                cost: priced.cost,
                duration_s: priced.duration_s,
                distance_m: priced.distance_m,
            },
            None => AccessTotals {
                cost: self.cost(distance_m),
                duration_s: self.duration_s(distance_m),
                distance_m,
            },
        }
    }
}

/// An access stretch priced by the caller: what it costs, how long it takes,
/// how long it is and the way it goes. Portals (`docs/design/portals.md`)
/// price it with a least-cost search over an incident surface.
#[derive(Debug, Clone, PartialEq)]
pub struct PricedAccess {
    /// Cost in the search's units (seconds times a cost factor); finite and
    /// not negative.
    pub cost: f64,
    pub duration_s: f64,
    pub distance_m: f64,
    /// From the requested point to the candidate's snap point for an origin,
    /// from the snap point to the requested point for a destination. Empty
    /// draws a straight line.
    pub path: Vec<GeoPoint>,
    /// What the pricer reports about the stretch, carried to the result's
    /// [`RouteAccessStretch::pricing`].
    pub pricing: Option<RouteAccessPricing>,
}

/// What the pricer of a stretch reports about it (portals:
/// `docs/design/portals.md`).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteAccessPricing {
    /// Who priced it: `terrain` for a least-cost search over an incident
    /// surface.
    pub source: String,
    /// The surface's content hash (hex), for a terrain stretch.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub surface_hash: Option<String>,
    /// How far to trust a terrain stretch.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub confidence: Option<RouteAccessConfidence>,
    /// The candidates the pricer ranked behind the ones the endpoint kept.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub truncated: Option<RouteAccessTruncation>,
}

/// How far to trust a terrain stretch, over the posts its path crosses
/// (`docs/design/incident-surface.md` section 7).
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteAccessConfidence {
    /// Length-weighted mean and least certainty of the fused land cover
    /// (1/255).
    pub mean_certainty: u8,
    pub min_certainty: u8,
    /// Share of the length on posts a layer decided, and the team, per mille.
    pub layer_permille: u32,
    pub team_permille: u32,
    /// Highest closure risk on the path (1/255) and the share of its length
    /// with any risk, per mille.
    pub max_risk: u8,
    pub risky_permille: u32,
    /// The path touches the edge of the surface: a larger area might give a
    /// faster stretch.
    pub at_edge: bool,
}

/// Candidates a pricer ranked but the endpoint did not keep. The search over
/// the kept candidates is exact; a cut candidate could still have given a
/// cheaper route if its key, less the worst kept key, is less than the road
/// cost it would save.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteAccessTruncation {
    pub cut: usize,
    /// The best cut candidate's key and the worst kept one's, in seconds:
    /// access, switch and the rest of the edge.
    pub best_cut_key_s: f64,
    pub worst_kept_key_s: f64,
}

impl PricedAccess {
    /// Checks that the price can seed a search: finite, not negative.
    pub fn validate(&self) -> Result<(), RouteError> {
        let ok = |value: f64| value.is_finite() && value >= 0.0;
        if ok(self.cost) && ok(self.duration_s) && ok(self.distance_m) {
            Ok(())
        } else {
            Err(RouteError::InvalidRequest {
                reason: "priced access",
            })
        }
    }
}

/// A candidate's access stretch as result totals; zero for a candidate
/// without access (its approach is a penalty, never reported).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct AccessTotals {
    pub cost: f64,
    pub duration_s: f64,
    pub distance_m: f64,
}

impl AccessTotals {
    pub(crate) fn of(candidate: &CorrelatedLocation) -> Self {
        match &candidate.access {
            Some(access) => access.totals(access_distance_m(candidate)),
            None => Self::default(),
        }
    }

    pub(crate) fn plus(self, other: Self) -> Self {
        Self {
            cost: self.cost + other.cost,
            duration_s: self.duration_s + other.duration_s,
            distance_m: self.distance_m + other.distance_m,
        }
    }
}

/// The length of a candidate's access stretch: its snap distance (as the
/// approach costs read it).
pub(crate) fn access_distance_m(candidate: &CorrelatedLocation) -> f64 {
    if candidate.distance_m.is_finite() {
        candidate.distance_m.max(0.0)
    } else {
        0.0
    }
}

/// One costed access stretch of a route leg.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteAccessStretch {
    /// The requested point.
    pub point: GeoPoint,
    /// Where the stretch meets the road network.
    pub join_point: GeoPoint,
    /// The directed edge it joins, and where along it.
    pub edge_id: u64,
    pub fraction: f64,
    pub distance_m: f64,
    pub duration_s: f64,
    pub cost: f64,
    /// The pricing it was costed with.
    pub pace_mps: f64,
    pub cost_factor: f64,
    /// What a pricer reported, for a priced stretch.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pricing: Option<RouteAccessPricing>,
}

/// The costed access stretches of a route leg (`LegResult::off_network`),
/// which its distance, duration and cost include.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteOffNetworkSummary {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub origin: Option<RouteAccessStretch>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub destination: Option<RouteAccessStretch>,
}

fn stretch(candidate: &CorrelatedLocation, access: &CandidateAccess) -> RouteAccessStretch {
    let totals = AccessTotals::of(candidate);
    RouteAccessStretch {
        point: access.point,
        join_point: candidate.snapped_point,
        edge_id: candidate.edge_id,
        fraction: candidate.fraction_along_edge,
        distance_m: totals.distance_m,
        duration_s: totals.duration_s,
        cost: totals.cost,
        pace_mps: access.pace_mps,
        cost_factor: access.cost_factor,
        pricing: access
            .priced
            .as_ref()
            .and_then(|priced| priced.pricing.clone()),
    }
}

/// Adds a found route's access stretches to its leg result: totals, geometry,
/// shape segments, maneuvers and [`RouteOffNetworkSummary`]. Routes with no
/// costed candidate are left as they are. Engine and oracle routes and their
/// alternatives all build their legs through here, so the totals are the
/// search's cost plus each chosen candidate's [`AccessTotals`].
pub(crate) fn add_access_stretches(result: &mut LegResult) {
    let origin = result
        .origin
        .access
        .as_ref()
        .map(|access| stretch(&result.origin, access));
    let destination = result
        .destination
        .access
        .as_ref()
        .map(|access| stretch(&result.destination, access));
    let origin_path = priced_path(&result.origin);
    let destination_path = priced_path(&result.destination);
    if origin.is_none() && destination.is_none() {
        return;
    }
    for stretch in origin.iter().chain(&destination) {
        result.distance_m += stretch.distance_m;
        result.duration_s += stretch.duration_s;
        result.cost += stretch.cost;
    }
    if let Some(stretch) = &origin {
        if let Some(leg) = result.legs.first_mut() {
            leg.distance_m += stretch.distance_m;
            leg.duration_s += stretch.duration_s;
        }
        if stretch.distance_m >= ACCESS_MIN_DRAWN_M {
            draw_origin_stretch(result, stretch, origin_path);
        }
    }
    if let Some(stretch) = &destination {
        if let Some(leg) = result.legs.last_mut() {
            leg.distance_m += stretch.distance_m;
            leg.duration_s += stretch.duration_s;
        }
        if stretch.distance_m >= ACCESS_MIN_DRAWN_M {
            draw_destination_stretch(result, stretch, destination_path);
        }
    }
    result.off_network = Some(RouteOffNetworkSummary {
        origin,
        destination,
    });
}

fn access_segment(
    stretch: &RouteAccessStretch,
    begin_shape_index: usize,
    end_shape_index: usize,
    from: GeoPoint,
    to: GeoPoint,
) -> RouteShapeSegment {
    RouteShapeSegment {
        edge_id: stretch.edge_id,
        begin_shape_index,
        end_shape_index,
        start_fraction: stretch.fraction,
        end_fraction: stretch.fraction,
        quality_flags: vec![RouteShapeQualityFlag::OffNetworkAccess],
        dynamic_overlay: Some(RouteShapeDynamicOverlay {
            kind: RouteShapeDynamicOverlayKind::AccessLeg,
            connector_id: None,
            connector_type: None,
            source_chunk_ids: Vec::new(),
            distance_meters: Some(stretch.distance_m),
            duration_seconds: Some(stretch.duration_s),
            cost: Some(stretch.cost),
            source_endpoint: None,
            source_point: Some(from),
            source_edge_id: None,
            source_fraction: None,
            target_endpoint: None,
            target_point: Some(to),
            target_edge_id: None,
            target_fraction: None,
        }),
    }
}

/// The path of a candidate's priced access, when it has one of at least two
/// points.
fn priced_path(candidate: &CorrelatedLocation) -> Option<Vec<GeoPoint>> {
    let priced = candidate.access.as_ref()?.priced.as_ref()?;
    (priced.path.len() >= 2).then(|| priced.path.clone())
}

/// A stretch's drawn line from `from` to `to`: a priced path, its ends set to
/// the two points, or the densified straight line.
fn stretch_line(from: GeoPoint, to: GeoPoint, path: Option<Vec<GeoPoint>>) -> Vec<GeoPoint> {
    match path {
        Some(mut line) => {
            line[0] = from;
            let last = line.len() - 1;
            line[last] = to;
            line
        }
        None => densify_geodesic(from, to, DIRECT_LEG_VERTEX_SPACING_M),
    }
}

fn draw_origin_stretch(
    result: &mut LegResult,
    stretch: &RouteAccessStretch,
    path: Option<Vec<GeoPoint>>,
) {
    let Some(&join) = result.geometry.first() else {
        return;
    };
    let line = stretch_line(stretch.point, join, path);
    let added = line.len() - 1;
    result.geometry.splice(0..0, line[..added].iter().copied());
    if let Some(leg) = result.legs.first_mut() {
        leg.geometry.splice(0..0, line[..added].iter().copied());
        for segment in &mut leg.shape_segments {
            segment.begin_shape_index += added;
            segment.end_shape_index += added;
        }
        leg.shape_segments
            .insert(0, access_segment(stretch, 0, added, stretch.point, join));
    }
    for maneuver in &mut result.maneuvers {
        maneuver.begin_shape_index += added;
        maneuver.end_shape_index += added;
    }
    // The road's start becomes a maneuver after the stretch, which starts the
    // leg instead.
    let road = result.maneuvers.first_mut().and_then(|first| {
        (first.kind == RouteManeuverKind::Start).then(|| {
            first.kind = RouteManeuverKind::Continue;
            first.road_name.clone()
        })
    });
    let maneuver = access_maneuver(
        AccessSide::Origin,
        &line,
        stretch,
        road.flatten().as_deref(),
        (0, added),
    );
    result.maneuvers.insert(0, maneuver);
}

fn draw_destination_stretch(
    result: &mut LegResult,
    stretch: &RouteAccessStretch,
    path: Option<Vec<GeoPoint>>,
) {
    let Some(&join) = result.geometry.last() else {
        return;
    };
    let line = stretch_line(join, stretch.point, path);
    let added = line.len() - 1;
    let last = result.geometry.len() - 1;
    result.geometry.extend(line[1..].iter().copied());
    if let Some(leg) = result.legs.last_mut() {
        let local_last = leg.geometry.len().saturating_sub(1);
        leg.geometry.extend(line[1..].iter().copied());
        leg.shape_segments.push(access_segment(
            stretch,
            local_last,
            local_last + added,
            join,
            stretch.point,
        ));
    }
    let arrival = result
        .maneuvers
        .iter()
        .rposition(|maneuver| maneuver.kind == RouteManeuverKind::Destination);
    let road = arrival
        .and_then(|index| index.checked_sub(1))
        .and_then(|index| result.maneuvers.get(index))
        .and_then(|maneuver| maneuver.road_name.clone());
    let maneuver = access_maneuver(
        AccessSide::Destination,
        &line,
        stretch,
        road.as_deref(),
        (last, last + added),
    );
    match arrival {
        Some(index) => {
            let destination = &mut result.maneuvers[index];
            destination.begin_shape_index = last + added;
            destination.end_shape_index = last + added;
            result.maneuvers.insert(index, maneuver);
        }
        None => result.maneuvers.push(maneuver),
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum AccessSide {
    Origin,
    Destination,
}

/// The maneuver of an access stretch: a bearing and a length, the only
/// things a straight line across open ground knows, and the road it meets
/// or leaves when that road has a name.
fn access_maneuver(
    side: AccessSide,
    line: &[GeoPoint],
    stretch: &RouteAccessStretch,
    road_name: Option<&str>,
    (begin_shape_index, end_shape_index): (usize, usize),
) -> RouteManeuver {
    let bearing = line
        .first()
        .zip(line.last())
        .filter(|(from, to)| haversine_m(**from, **to) > 0.01)
        .map(|(from, to)| bearing_deg(*from, *to).round() as u16 % 360);
    let cardinal = bearing.map(cardinal_direction);
    let heading = cardinal
        .as_deref()
        .map(|cardinal| format!(" {cardinal}"))
        .unwrap_or_default();
    let distance_m = stretch.distance_m;
    let road = road_name.unwrap_or("the road");
    let (kind, key, instruction) = match side {
        AccessSide::Origin => (
            RouteManeuverKind::Start,
            "route.access_depart",
            format!("Head{heading} across open ground for {distance_m:.0} m to {road}"),
        ),
        AccessSide::Destination => (
            RouteManeuverKind::Continue,
            "route.access_arrive",
            format!("Leave {road} and head{heading} across open ground for {distance_m:.0} m"),
        ),
    };
    let concise_instruction = format!("Head{heading} across open ground");
    let mut instruction_parameters = [
        (
            "guidance_reason".to_string(),
            "off_network_access".to_string(),
        ),
        ("access_distance_m".to_string(), format!("{distance_m:.1}")),
    ]
    .into_iter()
    .collect::<std::collections::BTreeMap<_, _>>();
    if let Some(road_name) = road_name {
        instruction_parameters.insert("road_name".to_string(), road_name.to_string());
    }
    RouteManeuver {
        kind,
        instruction: instruction.clone(),
        concise_instruction,
        pre_transition_instruction: None,
        post_transition_instruction: Some(instruction.clone()),
        alert_instruction: Some(instruction),
        cardinal_direction: cardinal,
        bearing_deg: bearing,
        instruction_key: key.to_string(),
        instruction_parameters,
        distance_m,
        duration_s: stretch.duration_s,
        begin_shape_index,
        end_shape_index,
        ..RouteManeuver::default()
    }
}

/// Whether a shape segment is not on a graph edge (a host connector or an
/// access stretch): it has no edge attribution of its own.
pub(crate) fn is_off_graph_segment(segment: &RouteShapeSegment) -> bool {
    segment.quality_flags.iter().any(|flag| {
        matches!(
            flag,
            RouteShapeQualityFlag::SsvgConnector | RouteShapeQualityFlag::OffNetworkAccess
        )
    })
}

/// The cost factor of a costed SAR `off_network` leg (X6): 1, so its join is
/// the one with the least total time (R7 already prices the leg as time at
/// the profile's off-road pace), and its `added_cost_s` is both the cost and
/// the duration the totals add.
pub const SAR_OFF_NETWORK_COST_FACTOR: f64 = 1.0;

/// A request's access legs: its `off_network` option, else a costed SAR
/// `off_network` (X6) as access legs within the SAR reach, at the profile's
/// off-road pace and [`SAR_OFF_NETWORK_COST_FACTOR`].
pub(crate) fn request_access(
    options: &crate::types::RouteOptions,
) -> Option<RouteOffNetworkOptions> {
    options.off_network.clone().or_else(|| {
        options
            .sar
            .as_ref()
            .and_then(crate::sar::RouteSarOptions::costed_off_network_max_leg_m)
            .map(|max_leg_m| RouteOffNetworkOptions {
                max_radius_m: Some(max_leg_m),
                cost_factor: Some(SAR_OFF_NETWORK_COST_FACTOR),
                ..RouteOffNetworkOptions::default()
            })
    })
}

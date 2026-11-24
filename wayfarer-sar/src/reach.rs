//! Reach prediction: where the person could be, and which way they went.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use wayfarer::{
    CorrelatedLocation, GeoPoint, MobilityProfile, RouteError, RouteIsochroneBoundaryFragment,
    RouteIsochroneContour, RouteIsochroneMetric, RouteIsochroneOrder, RouteIsochroneOrigin,
    RouteIsochroneRequest, RouteIsochroneResult, RouteMatrixCellStatus, RouteMatrixRequest,
    RouteOptions, RouteOverlay, RoutingEngine, SnapRequest,
};

use crate::network::{
    Network, NetworkEdge, bearing_deg, destination, distance_m, midpoint, zone_overlays,
};
use crate::pace::{PersonFactors, PersonPercentile, footpath_ratio, pace_ratio};
use crate::{SarError, SarZone, SarZoneKind, finite_positive, invalid_point, unit_interval};
use wayfarer::pace::{PaceModel, PaceSource};

/// The most time bands a reach is cut into (three more contours are the
/// person percentiles; the engine allows 16 in all).
pub const MAX_REACH_BANDS: usize = 12;
/// The most bearing sectors frontier routes are sampled in.
pub const MAX_ROUTE_SECTORS: usize = 64;
/// The most named destinations of one reach request.
pub const MAX_DESTINATIONS: usize = 256;
/// The longest elapsed time a reach request takes: 30 days.
pub const MAX_ELAPSED_S: f64 = 30.0 * 86_400.0;
/// The largest last-known-position uncertainty, in metres.
pub const MAX_UNCERTAINTY_M: f64 = 5_000.0;
/// Points sampled on the uncertainty circle around the last known position.
const UNCERTAINTY_POINTS: usize = 8;

/// Where could the person be now? See `docs/design/sar-planning.md` 2.1-2.4.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct ReachRequest {
    /// The approximate last known position (LKP) or point last seen.
    pub last_known: GeoPoint,
    /// How far (metres) the true LKP may be from `last_known`, at most
    /// 5,000: the reach starts from `last_known` and eight points on this
    /// circle, each edge at its earliest time from any. 0 (default): exact.
    #[serde(default)]
    pub uncertainty_m: f64,
    /// Seconds since the person was last seen. Give this, or both
    /// `last_seen_at_ms` and `at_ms`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub elapsed_s: Option<f64>,
    /// When the person was last seen (UTC ms).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_seen_at_ms: Option<i64>,
    /// The time the prediction is for (UTC ms); also the time overlay windows
    /// are evaluated at. Explicit, so a plan replays.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub at_ms: Option<i64>,
    /// How the person travels (default `on_foot`).
    #[serde(default = "on_foot")]
    pub profile: MobilityProfile,
    /// The most of the elapsed time the person can have spent moving, as a
    /// share in (0, 1]. 1 (default) is the outer bound: a person who never
    /// rested. The reach areas use it as is; the probability spreads the time
    /// actually moved by `stopping`.
    #[serde(default = "one")]
    pub moving_fraction: f64,
    /// When the person stopped moving, for the probability (default
    /// `uniform`).
    #[serde(default)]
    pub stopping: Stopping,
    /// Speed under the conditions, as a share of the model's (0.9 for snow
    /// or night is E4's starting value), in (0, 2]. Default 1.
    #[serde(default = "one")]
    pub speed_factor: f64,
    /// The walking pace model (`wayfarer pace calibrate` fits one to a team's
    /// or region's GPX tracks); by default the built-in E4 model. On foot,
    /// the person factors are its P10, P50 and P90 times its ratio to the
    /// engine's time on the reached network.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pace: Option<PaceModel>,
    /// Person factors as engine-time multipliers, instead of the pace
    /// model's (and for profiles other than on foot, which default to
    /// [`PersonFactors::FLAT_PACE`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub person_factors: Option<PersonFactors>,
    /// Zones the person cannot enter, tends to avoid, or is drawn to.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub zones: Vec<SarZone>,
    /// The probability the person keeps out of `avoid` zones, in [0, 1]
    /// (default 0.8).
    #[serde(default = "default_compliance")]
    pub avoid_compliance: f64,
    /// Lost-person distance statistics for the subject's category (the host
    /// supplies them; ISRID-style): cumulative probability by straight-line
    /// distance from the LKP, distances increasing.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub distance_rings: Vec<DistanceRing>,
    /// The weight of `distance_rings` against the travel-time model, in
    /// [0, 1] (default 0.5 when rings are given).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ring_weight: Option<f64>,
    /// Places the person may have headed for (a car park, a hut, a village):
    /// a likely route to each.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub destinations: Vec<NamedPoint>,
    /// Frontier routes: one to the farthest point a fast person reaches in
    /// each of this many bearing sectors, at most 64 (default 24; 0: none).
    #[serde(default = "default_sectors")]
    pub route_sectors: usize,
    /// Time bands the reach is cut into for the probability, 1 to 12
    /// (default 12).
    #[serde(default = "default_bands")]
    pub bands: usize,
    /// The share of people faster than the outermost band, in (0, 0.1]
    /// (default 0.02): their probability is the rest of world.
    #[serde(default = "default_outer_share")]
    pub outer_share: f64,
    /// Route options (SAR relaxations, correlation) as for an isochrone.
    #[serde(default)]
    pub options: RouteOptions,
    /// Closures and overlay-set items, applied as an isochrone applies them,
    /// besides the zones.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub overlays: Vec<RouteOverlay>,
    /// The expansion budget of each isochrone (see
    /// `RouteIsochroneRequest::max_expansion_edge_count`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_expansion_edge_count: Option<usize>,
    /// `false` leaves out the areas' `edgeIds` and the routes' `edgeIds`.
    #[serde(default = "yes")]
    pub include_edge_ids: bool,
}

impl ReachRequest {
    pub fn new(last_known: GeoPoint) -> Self {
        Self {
            last_known,
            uncertainty_m: 0.0,
            elapsed_s: None,
            last_seen_at_ms: None,
            at_ms: None,
            profile: MobilityProfile::OnFoot,
            moving_fraction: 1.0,
            stopping: Stopping::default(),
            speed_factor: 1.0,
            pace: None,
            person_factors: None,
            zones: Vec::new(),
            avoid_compliance: default_compliance(),
            distance_rings: Vec::new(),
            ring_weight: None,
            destinations: Vec::new(),
            route_sectors: default_sectors(),
            bands: default_bands(),
            outer_share: default_outer_share(),
            options: RouteOptions::default(),
            overlays: Vec::new(),
            max_expansion_edge_count: None,
            include_edge_ids: true,
        }
    }

    fn elapsed(&self) -> Result<f64, SarError> {
        let elapsed = match (self.elapsed_s, self.last_seen_at_ms, self.at_ms) {
            (Some(elapsed), None, _) => elapsed,
            (None, Some(seen), Some(at)) => (at - seen) as f64 / 1_000.0,
            (Some(_), Some(_), _) => {
                return Err(SarError::invalid(
                    "give elapsedS or lastSeenAtMs with atMs, not both",
                ));
            }
            _ => {
                return Err(SarError::invalid(
                    "the elapsed time needs elapsedS, or lastSeenAtMs and atMs",
                ));
            }
        };
        if !finite_positive(elapsed) || elapsed > MAX_ELAPSED_S {
            return Err(SarError::invalid(
                "the elapsed time must be above 0 and at most 30 days",
            ));
        }
        Ok(elapsed)
    }

    fn validate(&self) -> Result<f64, SarError> {
        if invalid_point(self.last_known) {
            return Err(SarError::invalid("last known position coordinates"));
        }
        if !self.uncertainty_m.is_finite()
            || !(0.0..=MAX_UNCERTAINTY_M).contains(&self.uncertainty_m)
        {
            return Err(SarError::invalid("uncertaintyM must be in [0, 5000]"));
        }
        if !finite_positive(self.moving_fraction) || self.moving_fraction > 1.0 {
            return Err(SarError::invalid("movingFraction must be in (0, 1]"));
        }
        if !finite_positive(self.speed_factor) || self.speed_factor > 2.0 {
            return Err(SarError::invalid("speedFactor must be in (0, 2]"));
        }
        if !unit_interval(self.avoid_compliance) {
            return Err(SarError::invalid("avoidCompliance must be in [0, 1]"));
        }
        if self
            .ring_weight
            .is_some_and(|weight| !unit_interval(weight))
        {
            return Err(SarError::invalid("ringWeight must be in [0, 1]"));
        }
        if !(1..=MAX_REACH_BANDS).contains(&self.bands) {
            return Err(SarError::invalid("bands must be 1 to 12"));
        }
        if self.route_sectors > MAX_ROUTE_SECTORS {
            return Err(SarError::invalid("routeSectors must be at most 64"));
        }
        if !finite_positive(self.outer_share) || self.outer_share > 0.1 {
            return Err(SarError::invalid("outerShare must be in (0, 0.1]"));
        }
        if self.destinations.len() > MAX_DESTINATIONS {
            return Err(SarError::invalid("at most 256 destinations"));
        }
        if self
            .destinations
            .iter()
            .any(|named| invalid_point(named.point))
        {
            return Err(SarError::invalid("destination coordinates"));
        }
        if let Some(factors) = &self.person_factors {
            factors.validate()?;
        }
        if let Some(pace) = &self.pace {
            pace.validate()
                .map_err(|error| SarError::invalid(format!("pace model: {error}")))?;
        }
        for zone in &self.zones {
            zone.validate()?;
        }
        validate_rings(&self.distance_rings)?;
        self.elapsed()
    }
}

fn validate_rings(rings: &[DistanceRing]) -> Result<(), SarError> {
    let mut previous = (0.0, 0.0);
    for ring in rings {
        if !finite_positive(ring.distance_m)
            || !finite_positive(ring.probability)
            || ring.probability > 1.0
            || ring.distance_m <= previous.0
            || ring.probability < previous.1
        {
            return Err(SarError::invalid(
                "distance rings need increasing distances above 0 and non-decreasing probabilities in (0, 1]",
            ));
        }
        previous = (ring.distance_m, ring.probability);
    }
    Ok(())
}

/// When the person stopped moving (design 2.2). Lost people rest, shelter,
/// get hurt or wander back, so most are found far inside their full reach.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum Stopping {
    /// Equally likely at any time up to the moving budget: the time moved is
    /// uniform between 0 and `elapsed * moving_fraction`.
    #[default]
    Uniform,
    /// Never: everyone is at the end of their full reach (the outward bound
    /// of the travel-time model).
    Never,
}

/// Midpoints of the moving-time quadrature of [`Stopping::Uniform`].
const STOPPING_STEPS: usize = 128;

fn on_foot() -> MobilityProfile {
    MobilityProfile::OnFoot
}
fn one() -> f64 {
    1.0
}
fn yes() -> bool {
    true
}
fn default_compliance() -> f64 {
    0.8
}
fn default_sectors() -> usize {
    24
}
fn default_bands() -> usize {
    MAX_REACH_BANDS
}
fn default_outer_share() -> f64 {
    0.02
}

/// One point of a cumulative distance distribution: `probability` of lost
/// persons of the category are found within `distance_m` of the LKP.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct DistanceRing {
    pub probability: f64,
    pub distance_m: f64,
}

/// A point with the planner's label.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct NamedPoint {
    pub id: String,
    pub point: GeoPoint,
}

/// The network a person of one percentile could have reached.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct ReachArea {
    pub person: PersonPercentile,
    /// The contour in model seconds (budget over the person's factor).
    pub contour_s: f64,
    /// Edges reached (both directions count once).
    pub edge_count: usize,
    pub network_length_m: f64,
    /// The farthest reached point's straight-line distance from the LKP:
    /// the search radius for this person.
    pub max_radius_m: f64,
    /// Undirected edge keys (the smaller directed id of each edge), sorted.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub edge_ids: Vec<u64>,
}

/// One edge's probability of holding the person.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct EdgeProbability {
    /// The undirected key (the smaller directed id of the edge's two).
    pub edge_id: u64,
    pub probability: f64,
    pub length_m: f64,
    /// The model time at which the edge is first entered (the complying
    /// search's when it reaches the edge).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub time_s: Option<f64>,
    /// Reached only by crossing an `avoid` zone (or inside one).
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub avoid_only: bool,
}

/// Probability of area on the network: per edge, plus what lies beyond.
/// Edges, `rest_of_world` and `off_network` add up to 1.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct ProbabilityMap {
    /// Sorted by edge key; only edges above 0.
    pub edges: Vec<EdgeProbability>,
    /// Beyond the outermost band (faster than nearly everyone, or outside the
    /// distance rings).
    pub rest_of_world: f64,
    /// Mass of bands or rings with no reachable path in them (the network
    /// ended: the person went off the network or stopped at its end).
    #[serde(default)]
    pub off_network: f64,
}

impl ProbabilityMap {
    pub(crate) fn by_edge(&self) -> BTreeMap<u64, f64> {
        self.edges
            .iter()
            .map(|edge| (edge.edge_id, edge.probability))
            .collect()
    }

    pub(crate) fn validate(&self) -> Result<(), SarError> {
        let values = self
            .edges
            .iter()
            .map(|edge| edge.probability)
            .chain([self.rest_of_world, self.off_network]);
        let mut total = 0.0;
        for value in values {
            if !unit_interval(value) {
                return Err(SarError::invalid("probabilities must be in [0, 1]"));
            }
            total += value;
        }
        if (total - 1.0).abs() > 1e-6 && total > 0.0 {
            return Err(SarError::invalid("probabilities must add up to 1"));
        }
        Ok(())
    }
}

/// Where a likely route goes.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case", tag = "kind")]
pub enum RouteTarget {
    /// One of the request's destinations.
    Destination { id: String, point: GeoPoint },
    /// The farthest point a fast person reaches in one bearing sector.
    Frontier {
        sector: usize,
        bearing_deg: f64,
        point: GeoPoint,
    },
}

/// A route the person may have taken from the LKP.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct LikelyRoute {
    pub target: RouteTarget,
    pub reachable: bool,
    /// Model seconds along the route (the profile's pace).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub duration_s: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distance_m: Option<f64>,
    /// Elapsed seconds after last seen at which a fast, typical and slow
    /// person arrives (with the request's moving fraction and conditions).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub arrival_elapsed_s: BTreeMap<PersonPercentile, f64>,
    /// The share of people who could have got there by now.
    #[serde(default)]
    pub reach_probability: f64,
    /// Directed edge ids in travel order.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub edge_ids: Vec<u64>,
}

/// An edge many likely routes share.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteCorridorEdge {
    /// The undirected key.
    pub edge_id: u64,
    pub route_count: usize,
    /// `route_count` over the reachable routes.
    pub share: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct ReachResult {
    /// The request's last known position.
    pub last_known: GeoPoint,
    pub elapsed_s: f64,
    /// `elapsed_s * moving_fraction`.
    pub moving_s: f64,
    /// The model-time budget: `moving_s * speed_factor`.
    pub budget_s: f64,
    pub pace: ReachPace,
    /// Where each origin snapped: the LKP first, then the uncertainty points
    /// that snapped.
    pub origins: Vec<CorrelatedLocation>,
    /// What a fast, a typical and a slow person could have reached, crossing
    /// `avoid` zones if they must (blocked only by `no_go`).
    pub areas: Vec<ReachArea>,
    /// With `avoid` zones: the same keeping out of them.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub complying_areas: Vec<ReachArea>,
    pub probability: ProbabilityMap,
    pub routes: Vec<LikelyRoute>,
    /// Edges on two or more reachable routes, most shared first.
    pub corridors: Vec<RouteCorridorEdge>,
    /// An isochrone hit its expansion budget: the reach is incomplete.
    #[serde(default)]
    pub truncated: bool,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub warnings: Vec<String>,
}

/// The contours of one set of person factors: the bands up to the outer
/// contour, then the three persons'.
struct Layout {
    bands: Vec<f64>,
    persons: [(PersonPercentile, f64); 3],
    contours: Vec<f64>,
}

impl Layout {
    fn new(budget: f64, factors: &PersonFactors, request: &ReachRequest) -> Self {
        let outer = budget / factors.quantile(request.outer_share);
        let bands = (1..=request.bands)
            .map(|band| outer * band as f64 / request.bands as f64)
            .collect::<Vec<_>>();
        let persons = PersonPercentile::ALL.map(|person| (person, budget / factors.of(person)));
        let mut contours = bands.clone();
        for (_, contour) in persons {
            if !contours
                .iter()
                .any(|existing| (existing - contour).abs() <= f64::EPSILON * contour)
            {
                contours.push(contour);
            }
        }
        Self {
            bands,
            persons,
            contours,
        }
    }
}

/// The pace a reach used.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct ReachPace {
    /// The pace model's source (`default` or a team calibration); none when
    /// the request's person factors or a profile other than on foot decided.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<PaceSource>,
    /// The pace model's expected time over the engine's, as applied: measured
    /// on the network a typical person reaches (kept at the first guess, an
    /// unclassified footpath, when that is within 2 %).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub network_ratio: Option<f64>,
    /// The person factors applied, as engine-time multipliers.
    pub person_factors: PersonFactors,
    /// Whether slope entered the pace (false until packages carry heights).
    pub slope_applied: bool,
}

/// One isochrone run, reduced to undirected keys.
struct Run {
    result: RouteIsochroneResult,
    /// Per key, the model times at which the edge is first entered and
    /// wholly reached (the latter extrapolated along a boundary fragment for
    /// an edge no band reaches whole).
    span: BTreeMap<u64, (f64, f64)>,
    /// Per person percentile, the keys within its contour.
    persons: BTreeMap<PersonPercentile, BTreeSet<u64>>,
}

impl Run {
    fn bucket(
        result: &RouteIsochroneResult,
        contour: f64,
    ) -> impl Iterator<Item = &wayfarer::RouteIsochroneBucket> {
        result.buckets.iter().filter(move |bucket| {
            bucket.metric == RouteIsochroneMetric::TimeSeconds && bucket.contour == contour
        })
    }

    fn fragments(&self, contour: f64) -> Vec<&RouteIsochroneBoundaryFragment> {
        Self::bucket(&self.result, contour)
            .flat_map(|bucket| bucket.boundary_fragments.iter())
            .collect()
    }
}

/// Caches the edges a request touches.
pub(crate) struct Edges<'a> {
    network: Network<'a>,
    cache: BTreeMap<u64, Option<NetworkEdge>>,
}

impl<'a> Edges<'a> {
    pub(crate) fn new(engine: &'a RoutingEngine) -> Self {
        Self {
            network: Network::new(engine),
            cache: BTreeMap::new(),
        }
    }

    pub(crate) fn network(&self) -> &Network<'a> {
        &self.network
    }

    pub(crate) fn get(&mut self, key: u64) -> Option<&NetworkEdge> {
        let network = &self.network;
        self.cache
            .entry(key)
            .or_insert_with(|| network.edge(key))
            .as_ref()
    }

    pub(crate) fn length(&mut self, key: u64) -> f64 {
        self.get(key).map_or(0.0, |edge| edge.length_m)
    }
}

/// Predicts where the person could be (see [`ReachRequest`]).
pub fn predict_reach(
    engine: &RoutingEngine,
    request: &ReachRequest,
) -> Result<ReachResult, SarError> {
    let elapsed = request.validate()?;
    let moving = elapsed * request.moving_fraction;
    let budget = moving * request.speed_factor;
    let mut warnings = Vec::new();
    // Person factors: the request's own, else the pace model's on foot (its
    // ratio to the engine's time measured on the reached network below),
    // else E4's flat-pace factors.
    let default_pace = PaceModel::default();
    let pace = request.pace.as_ref().unwrap_or(&default_pace);
    let use_pace = request.person_factors.is_none() && request.profile == MobilityProfile::OnFoot;
    let mut ratio = use_pace.then(|| footpath_ratio(pace));
    let mut factors = match (request.person_factors, ratio) {
        (Some(factors), _) => factors,
        (None, Some(ratio)) => PersonFactors::from_pace(pace, ratio),
        (None, None) => {
            warnings.push(format!(
                "person factors are calibrated on foot only (E4); {:?} uses the flat-pace ones",
                request.profile
            ));
            PersonFactors::FLAT_PACE
        }
    };

    let origins = uncertainty_origins(engine, request, &mut warnings);
    let mut edges = Edges::new(engine);
    let has_avoid = request
        .zones
        .iter()
        .any(|zone| zone.kind == SarZoneKind::Avoid);
    let free_overlays = [
        request.overlays.clone(),
        zone_overlays(&request.zones, &[SarZoneKind::NoGo]),
    ]
    .concat();
    let complying_overlays = [
        request.overlays.clone(),
        zone_overlays(&request.zones, &[SarZoneKind::NoGo, SarZoneKind::Avoid]),
    ]
    .concat();

    let search =
        |layout: &Layout, overlays: &[RouteOverlay], edges: &mut Edges| -> Result<Run, SarError> {
            let (bands, persons) = (&layout.bands, &layout.persons);
            let mut isochrone = RouteIsochroneRequest::new(
                request.last_known,
                request.profile,
                layout
                    .contours
                    .iter()
                    .map(|contour| RouteIsochroneContour {
                        contour: *contour,
                        metric: RouteIsochroneMetric::TimeSeconds,
                    })
                    .collect(),
            );
            isochrone.origins = origins.clone();
            isochrone.order = RouteIsochroneOrder::Time;
            isochrone.options = request.options.clone();
            isochrone.overlays = overlays.to_vec();
            isochrone.departure_at_ms = request.at_ms;
            isochrone.max_expansion_edge_count = request.max_expansion_edge_count;
            let result = engine.isochrone(isochrone)?;
            let span = edge_spans(&result, bands, edges.network());
            let persons = persons
                .iter()
                .map(|(person, contour)| {
                    let keys = Run::bucket(&result, *contour)
                        .flat_map(|bucket| bucket.edge_ids.iter())
                        .map(|edge_id| edges.network().key(*edge_id))
                        .collect::<BTreeSet<_>>();
                    (*person, keys)
                })
                .collect();
            Ok(Run {
                result,
                span,
                persons,
            })
        };

    let mut layout = Layout::new(budget, &factors, request);
    let mut free = search(&layout, &free_overlays, &mut edges)?;
    if let Some(guess) = ratio {
        // The pace model's time against the engine's on what a typical person
        // reaches; searched again when the first guess was off by over 2 %.
        let reached = free.persons[&PersonPercentile::Typical].iter().copied();
        if let Some(measured) = pace_ratio(
            engine,
            pace,
            request.profile,
            &request.options.costing,
            reached,
        ) && (measured / guess - 1.0).abs() > 0.02
        {
            ratio = Some(measured);
            factors = PersonFactors::from_pace(pace, measured);
            layout = Layout::new(budget, &factors, request);
            free = search(&layout, &free_overlays, &mut edges)?;
        }
    }
    let bands = layout.bands.clone();
    let persons = layout.persons;
    // A last known position inside an avoid zone has no complying search: the
    // person is already in it, so the free search stands alone.
    let complying = if has_avoid {
        match search(&layout, &complying_overlays, &mut edges) {
            Ok(run) => Some(run),
            Err(SarError::Route(error)) if !matches!(error, RouteError::Cancelled { .. }) => {
                warnings.push(format!(
                    "no search keeps out of the avoid zones ({error}); they are ignored"
                ));
                None
            }
            Err(error) => return Err(error),
        }
    } else {
        None
    };
    let truncated =
        free.result.truncated || complying.as_ref().is_some_and(|run| run.result.truncated);
    if truncated {
        warnings.push("an isochrone hit its expansion budget: the reach is incomplete".into());
    }

    // Zone weights per key.
    let mut weights = BTreeMap::<u64, f64>::new();
    for zone in request
        .zones
        .iter()
        .filter(|zone| zone.kind == SarZoneKind::Weight)
    {
        let factor = zone.factor.unwrap_or(1.0);
        for key in edges.network().keys_in_polygon(&zone.polygon) {
            *weights.entry(key).or_insert(1.0) *= factor;
        }
    }

    // The travel-time model, mixed over complying and free searches.
    let shares = match request.stopping {
        Stopping::Never => vec![1.0],
        Stopping::Uniform => (0..STOPPING_STEPS)
            .map(|step| (step as f64 + 0.5) / STOPPING_STEPS as f64)
            .collect(),
    };
    let model = TimeModel {
        bands: &bands,
        budget,
        factors: &factors,
        shares: &shares,
    };
    let free_time = time_distribution(&free, &model, &weights, &mut edges);
    let mut time = match &complying {
        Some(run) => {
            let complying_time = time_distribution(run, &model, &weights, &mut edges);
            mix(&complying_time, request.avoid_compliance, &free_time)
        }
        None => free_time,
    };

    // Distance rings, over everything the free search reaches.
    if !request.distance_rings.is_empty() {
        let reached = free.span.keys().copied().collect::<Vec<_>>();
        let rings = ring_distribution(
            request.last_known,
            &request.distance_rings,
            &reached,
            &weights,
            &mut edges,
        );
        time = mix(&rings, request.ring_weight.unwrap_or(0.5), &time);
    }

    let probability = ProbabilityMap {
        edges: time
            .edges
            .iter()
            .filter(|(_, probability)| **probability > 0.0)
            .map(|(key, probability)| {
                let complying_span = complying.as_ref().map(|run| run.span.get(key));
                let span = match complying_span {
                    Some(Some(span)) => Some(*span),
                    _ => free.span.get(key).copied(),
                };
                EdgeProbability {
                    edge_id: *key,
                    probability: *probability,
                    length_m: edges.length(*key),
                    time_s: span.map(|(entered, _)| entered),
                    avoid_only: matches!(complying_span, Some(None)),
                }
            })
            .collect(),
        rest_of_world: time.rest_of_world,
        off_network: time.off_network,
    };

    let areas = reach_areas(&free, &persons, request, &mut edges);
    let complying_areas = complying
        .as_ref()
        .map(|run| reach_areas(run, &persons, request, &mut edges))
        .unwrap_or_default();

    let route_run = complying.as_ref().unwrap_or(&free);
    let fast_contour = persons[0].1;
    let routes = likely_routes(
        engine,
        request,
        &factors,
        if complying.is_some() {
            &complying_overlays
        } else {
            &free_overlays
        },
        route_run,
        fast_contour,
        budget,
        &mut edges,
        &mut warnings,
    )?;
    let corridors = corridors(&routes, edges.network());
    let mut routes = routes;
    if !request.include_edge_ids {
        for route in &mut routes {
            route.edge_ids = Vec::new();
        }
    }

    let origins = std::iter::once(free.result.origin.clone())
        .chain(free.result.origins.iter().cloned())
        .collect();
    Ok(ReachResult {
        last_known: request.last_known,
        elapsed_s: elapsed,
        moving_s: moving,
        budget_s: budget,
        pace: ReachPace {
            source: ratio.map(|_| pace.source.clone()),
            network_ratio: ratio,
            person_factors: factors,
            slope_applied: false,
        },
        origins,
        areas,
        complying_areas,
        probability,
        routes,
        corridors,
        truncated,
        warnings,
    })
}

/// `last_known` plus the points of the uncertainty circle that snap to the
/// profile's network (an origin that does not snap would fail the request).
fn uncertainty_origins(
    engine: &RoutingEngine,
    request: &ReachRequest,
    warnings: &mut Vec<String>,
) -> Vec<RouteIsochroneOrigin> {
    if request.uncertainty_m <= 0.0 {
        return Vec::new();
    }
    let reach = request.uncertainty_m.clamp(75.0, 1_000.0) as u32;
    let mut origins = Vec::new();
    for index in 0..UNCERTAINTY_POINTS {
        let bearing = 360.0 * index as f64 / UNCERTAINTY_POINTS as f64;
        let point = destination(request.last_known, bearing, request.uncertainty_m);
        let mut snap = SnapRequest::new(point, request.profile);
        snap.max_distance_m = Some(reach);
        if engine.snap(snap).candidates.is_empty() {
            continue;
        }
        origins.push(RouteIsochroneOrigin::new(point));
    }
    if origins.len() < UNCERTAINTY_POINTS {
        warnings.push(format!(
            "{} of {UNCERTAINTY_POINTS} uncertainty points have no path within {reach} m and were left out",
            UNCERTAINTY_POINTS - origins.len()
        ));
    }
    origins
}

/// A distribution over undirected keys plus what lies beyond.
#[derive(Debug, Default)]
struct Distribution {
    edges: BTreeMap<u64, f64>,
    rest_of_world: f64,
    off_network: f64,
}

/// `share * a + (1 - share) * b`.
fn mix(a: &Distribution, share: f64, b: &Distribution) -> Distribution {
    let mut edges = BTreeMap::new();
    for (key, value) in &a.edges {
        *edges.entry(*key).or_insert(0.0) += share * value;
    }
    for (key, value) in &b.edges {
        *edges.entry(*key).or_insert(0.0) += (1.0 - share) * value;
    }
    Distribution {
        edges,
        rest_of_world: share * a.rest_of_world + (1.0 - share) * b.rest_of_world,
        off_network: share * a.off_network + (1.0 - share) * b.off_network,
    }
}

/// Spreads `mass` over `keys` by length times zone weight; returns false
/// when there is nothing to spread it over.
fn spread(
    mass: f64,
    keys: &[u64],
    weights: &BTreeMap<u64, f64>,
    edges: &mut Edges,
    into: &mut BTreeMap<u64, f64>,
) -> bool {
    let weighted = keys
        .iter()
        .map(|key| {
            (
                *key,
                edges.length(*key) * weights.get(key).copied().unwrap_or(1.0),
            )
        })
        .collect::<Vec<_>>();
    let total: f64 = weighted.iter().map(|(_, weight)| weight).sum();
    if total <= 0.0 {
        return false;
    }
    for (key, weight) in weighted {
        *into.entry(key).or_insert(0.0) += mass * weight / total;
    }
    true
}

/// Per undirected key, when the edge is first entered and when it is
/// wholly reached, in model seconds: whole reach from the first band whose
/// bucket lists the edge; entry from the boundary fragments that cut it (or
/// the band's lower bound when none does). An edge no band reaches whole has
/// its whole-reach time extrapolated along its fragment.
fn edge_spans(
    result: &RouteIsochroneResult,
    bands: &[f64],
    network: &Network,
) -> BTreeMap<u64, (f64, f64)> {
    let mut whole = BTreeMap::<u64, (f64, f64)>::new();
    let mut previous = 0.0;
    for contour in bands {
        for bucket in Run::bucket(result, *contour) {
            for edge_id in &bucket.edge_ids {
                whole
                    .entry(network.key(*edge_id))
                    .or_insert((previous, *contour));
            }
        }
        previous = *contour;
    }
    let mut entered = BTreeMap::<u64, f64>::new();
    let mut extrapolated = BTreeMap::<u64, f64>::new();
    for bucket in result
        .buckets
        .iter()
        .filter(|bucket| bucket.metric == RouteIsochroneMetric::TimeSeconds)
    {
        for fragment in &bucket.boundary_fragments {
            let key = network.key(fragment.edge_id);
            let entry = entered.entry(key).or_insert(fragment.start_metric);
            *entry = entry.min(fragment.start_metric);
            let covered = fragment.end_fraction - fragment.start_fraction;
            if covered > 0.0 {
                let rate = (fragment.end_metric - fragment.start_metric) / covered;
                let out = fragment.start_metric + (1.0 - fragment.start_fraction) * rate;
                let entry = extrapolated.entry(key).or_insert(out);
                *entry = entry.min(out);
            }
        }
    }
    let mut spans = BTreeMap::new();
    for (key, (lower, upper)) in &whole {
        let start = entered.get(key).copied().unwrap_or(*lower).min(*upper);
        spans.insert(*key, (start, *upper));
    }
    for (key, start) in entered {
        if spans.contains_key(&key) {
            continue;
        }
        let out = extrapolated
            .get(&key)
            .copied()
            .unwrap_or(f64::INFINITY)
            .max(start);
        spans.insert(key, (start, out));
    }
    spans
}

/// The travel-time model (design 2.2): band `k` holds the people whose
/// reach at `budget / F` ends in it, spread over the path length the band
/// covers (time taken as uniform along each edge).
fn time_distribution(
    run: &Run,
    model: &TimeModel,
    weights: &BTreeMap<u64, f64>,
    edges: &mut Edges,
) -> Distribution {
    let bands = model.bands;
    let mut by_band: Vec<Vec<(u64, f64)>> = vec![Vec::new(); bands.len()];
    for (key, (entered, whole)) in &run.span {
        let length = edges.length(*key) * weights.get(key).copied().unwrap_or(1.0);
        if length <= 0.0 {
            continue;
        }
        let mut lower = 0.0;
        for (band, upper) in bands.iter().enumerate() {
            let share = if whole <= entered {
                f64::from(u8::from(*whole > lower && *whole <= *upper))
            } else if whole.is_infinite() {
                0.0
            } else {
                (whole.min(*upper) - entered.max(lower)).max(0.0) / (whole - entered)
            };
            if share > 0.0 {
                by_band[band].push((*key, length * share));
            }
            lower = *upper;
        }
    }
    let mut distribution = Distribution::default();
    let mut previous = 0.0;
    for (band, upper) in bands.iter().enumerate() {
        let mass = model.mass(previous, *upper);
        let total: f64 = by_band[band].iter().map(|(_, weight)| weight).sum();
        if total > 0.0 {
            for (key, weight) in &by_band[band] {
                *distribution.edges.entry(*key).or_insert(0.0) += mass * weight / total;
            }
        } else {
            distribution.off_network += mass;
        }
        previous = *upper;
    }
    distribution.rest_of_world = model.beyond(previous);
    distribution
}

/// The travel-time model's inputs: bands, model-time budget, person factors
/// and the moving-time shares the stopping model averages over.
struct TimeModel<'a> {
    bands: &'a [f64],
    budget: f64,
    factors: &'a PersonFactors,
    shares: &'a [f64],
}

impl TimeModel<'_> {
    /// The share of people whose reach ends between model times `lower` and
    /// `upper`: those with `lower < share * budget / F <= upper`.
    fn mass(&self, lower: f64, upper: f64) -> f64 {
        let total: f64 = self
            .shares
            .iter()
            .map(|share| {
                let moved = share * self.budget;
                let slower = if lower > 0.0 {
                    self.factors.cdf(moved / lower)
                } else {
                    1.0
                };
                (slower - self.factors.cdf(moved / upper)).max(0.0)
            })
            .sum();
        total / self.shares.len() as f64
    }

    /// The share of people whose reach ends beyond `outer`.
    fn beyond(&self, outer: f64) -> f64 {
        let total: f64 = self
            .shares
            .iter()
            .map(|share| self.factors.cdf(share * self.budget / outer))
            .sum();
        total / self.shares.len() as f64
    }
}

/// The distance-ring model: ring `j`'s mass over the reached edges whose
/// midpoint lies in it.
fn ring_distribution(
    origin: GeoPoint,
    rings: &[DistanceRing],
    reached: &[u64],
    weights: &BTreeMap<u64, f64>,
    edges: &mut Edges,
) -> Distribution {
    let mut in_ring = vec![Vec::new(); rings.len()];
    for key in reached {
        let Some(middle) = edges.get(*key).and_then(|edge| midpoint(&edge.geometry)) else {
            continue;
        };
        let distance = distance_m(origin, middle);
        if let Some(ring) = rings.iter().position(|ring| distance <= ring.distance_m) {
            in_ring[ring].push(*key);
        }
    }
    let mut distribution = Distribution::default();
    let mut previous = 0.0;
    for (index, ring) in rings.iter().enumerate() {
        let mass = ring.probability - previous;
        if !spread(
            mass,
            &in_ring[index],
            weights,
            edges,
            &mut distribution.edges,
        ) {
            distribution.off_network += mass;
        }
        previous = ring.probability;
    }
    distribution.rest_of_world = 1.0 - previous;
    distribution
}

fn reach_areas(
    run: &Run,
    persons: &[(PersonPercentile, f64); 3],
    request: &ReachRequest,
    edges: &mut Edges,
) -> Vec<ReachArea> {
    persons
        .iter()
        .map(|(person, contour)| {
            let keys = &run.persons[person];
            let mut length = 0.0;
            let mut radius = 0.0_f64;
            for key in keys {
                if let Some(edge) = edges.get(*key) {
                    length += edge.length_m;
                    for point in [edge.geometry.first(), edge.geometry.last()]
                        .into_iter()
                        .flatten()
                    {
                        radius = radius.max(distance_m(request.last_known, *point));
                    }
                }
            }
            for fragment in run.fragments(*contour) {
                radius = radius.max(distance_m(request.last_known, fragment.end));
            }
            ReachArea {
                person: *person,
                contour_s: *contour,
                edge_count: keys.len(),
                network_length_m: length,
                max_radius_m: radius,
                edge_ids: if request.include_edge_ids {
                    keys.iter().copied().collect()
                } else {
                    Vec::new()
                },
            }
        })
        .collect()
}

/// Frontier targets: per bearing sector, the reached point farthest from the
/// LKP within the fast person's contour (ties: the lower edge key).
fn frontier_targets(
    run: &Run,
    request: &ReachRequest,
    contour: f64,
    edges: &mut Edges,
) -> Vec<RouteTarget> {
    let sectors = request.route_sectors;
    if sectors == 0 {
        return Vec::new();
    }
    let origin = request.last_known;
    let mut best: BTreeMap<usize, (f64, u64, GeoPoint)> = BTreeMap::new();
    let mut offer = |point: GeoPoint, key: u64| {
        let distance = distance_m(origin, point);
        if distance < 1.0 {
            return;
        }
        let sector =
            ((bearing_deg(origin, point) / 360.0 * sectors as f64) as usize).min(sectors - 1);
        let entry = best.entry(sector).or_insert((distance, key, point));
        if distance > entry.0 || (distance == entry.0 && key < entry.1) {
            *entry = (distance, key, point);
        }
    };
    for fragment in run.fragments(contour) {
        offer(fragment.end, edges.network().key(fragment.edge_id));
    }
    for key in &run.persons[&PersonPercentile::Fast] {
        if let Some(edge) = edges.get(*key) {
            for point in [edge.geometry.first(), edge.geometry.last()]
                .into_iter()
                .flatten()
            {
                offer(*point, *key);
            }
        }
    }
    best.into_iter()
        .map(|(sector, (_, _, point))| RouteTarget::Frontier {
            sector,
            bearing_deg: bearing_deg(origin, point),
            point,
        })
        .collect()
}

fn likely_routes(
    engine: &RoutingEngine,
    request: &ReachRequest,
    factors: &PersonFactors,
    overlays: &[RouteOverlay],
    run: &Run,
    fast_contour: f64,
    budget: f64,
    edges: &mut Edges,
    warnings: &mut Vec<String>,
) -> Result<Vec<LikelyRoute>, SarError> {
    let mut targets = request
        .destinations
        .iter()
        .map(|named| RouteTarget::Destination {
            id: named.id.clone(),
            point: named.point,
        })
        .collect::<Vec<_>>();
    targets.extend(frontier_targets(run, request, fast_contour, edges));
    if targets.is_empty() {
        return Ok(Vec::new());
    }
    let points = targets
        .iter()
        .map(|target| match target {
            RouteTarget::Destination { point, .. } | RouteTarget::Frontier { point, .. } => *point,
        })
        .collect();
    let mut matrix = RouteMatrixRequest::new(vec![request.last_known], points, request.profile);
    matrix.options = request.options.clone();
    matrix.overlays = overlays.to_vec();
    matrix.departure_at_ms = request.at_ms;
    let result = engine.matrix(matrix);
    let scale = 1.0 / (request.speed_factor * request.moving_fraction);
    let mut routes = Vec::with_capacity(targets.len());
    for (index, target) in targets.into_iter().enumerate() {
        let cell = result.cells.iter().find(|cell| cell.target_index == index);
        let route = match cell {
            Some(cell) if cell.status == RouteMatrixCellStatus::Reachable => {
                let duration = cell.duration_s.unwrap_or(0.0);
                LikelyRoute {
                    target,
                    reachable: true,
                    duration_s: cell.duration_s,
                    distance_m: cell.distance_m,
                    arrival_elapsed_s: PersonPercentile::ALL
                        .iter()
                        .map(|person| (*person, duration * factors.of(*person) * scale))
                        .collect(),
                    reach_probability: if duration > 0.0 {
                        factors.cdf(budget / duration)
                    } else {
                        1.0
                    },
                    edge_ids: cell.edge_ids.clone(),
                }
            }
            _ => {
                if let RouteTarget::Destination { id, .. } = &target {
                    warnings.push(format!(
                        "destination {id} has no route from the last known position"
                    ));
                }
                LikelyRoute {
                    target,
                    reachable: false,
                    duration_s: None,
                    distance_m: None,
                    arrival_elapsed_s: BTreeMap::new(),
                    reach_probability: 0.0,
                    edge_ids: Vec::new(),
                }
            }
        };
        routes.push(route);
    }
    Ok(routes)
}

fn corridors(routes: &[LikelyRoute], network: &Network) -> Vec<RouteCorridorEdge> {
    let reachable = routes.iter().filter(|route| route.reachable).count();
    let mut counts = BTreeMap::<u64, usize>::new();
    for route in routes.iter().filter(|route| route.reachable) {
        let keys = route
            .edge_ids
            .iter()
            .map(|edge_id| network.key(*edge_id))
            .collect::<BTreeSet<_>>();
        for key in keys {
            *counts.entry(key).or_insert(0) += 1;
        }
    }
    let mut corridors = counts
        .into_iter()
        .filter(|(_, count)| *count >= 2)
        .map(|(edge_id, route_count)| RouteCorridorEdge {
            edge_id,
            route_count,
            share: route_count as f64 / reachable.max(1) as f64,
        })
        .collect::<Vec<_>>();
    corridors.sort_by(|a, b| {
        b.route_count
            .cmp(&a.route_count)
            .then(a.edge_id.cmp(&b.edge_id))
    });
    corridors
}

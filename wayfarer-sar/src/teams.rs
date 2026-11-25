//! Team reach and assignment (S5): who gets to each segment first.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use wayfarer::{
    GeoPoint, MobilityProfile, RouteIsochroneContour, RouteIsochroneMetric, RouteIsochroneOrder,
    RouteIsochroneRequest, RouteMatrixCellStatus, RouteMatrixRequest, RouteOptions, RouteOverlay,
    RoutingEngine,
};

use crate::network::{centroid, search_options, zone_overlays};
use crate::pace::{footpath_ratio, pace_ratio};
use crate::reach::{Edges, ProbabilityMap};
use crate::{SarError, SarZone, SarZoneKind, finite_positive, invalid_point, unit_interval};
use wayfarer::pace::{PaceModel, PaceSource};

/// The most teams of one plan.
pub const MAX_TEAMS: usize = 64;
/// The most segments of one plan.
pub const MAX_SEGMENTS: usize = 1_000;

/// A search team.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct SarTeam {
    pub id: String,
    /// Where the team starts (the command post, a drop-off point).
    pub point: GeoPoint,
    #[serde(default = "on_foot")]
    pub profile: MobilityProfile,
    /// Seconds from now until the team leaves (default 0).
    #[serde(default)]
    pub start_offset_s: f64,
    /// A further multiplier on the team's travel time (a loaded team is
    /// slower than 1), in (0, 5]. Default 1.
    #[serde(default = "one")]
    pub pace_factor: f64,
    /// The team's own walking pace model (`wayfarer pace calibrate` on its
    /// GPX tracks); by default the plan's `pace`, else the built-in model.
    /// Applies on foot only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pace: Option<PaceModel>,
}

impl SarTeam {
    pub fn new(id: impl Into<String>, point: GeoPoint) -> Self {
        Self {
            id: id.into(),
            point,
            profile: MobilityProfile::OnFoot,
            start_offset_s: 0.0,
            pace_factor: 1.0,
            pace: None,
        }
    }
}

/// A search segment: an area one team searches in one assignment.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct SearchSegment {
    pub id: String,
    /// At least three points.
    pub polygon: Vec<GeoPoint>,
    /// Where a team enters the segment; by default the polygon's centroid.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub access_point: Option<GeoPoint>,
    /// The segment's probability of area when the planner sets it; by
    /// default the sum of the plan's edge probabilities inside the polygon.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub probability: Option<f64>,
}

impl SearchSegment {
    pub fn new(id: impl Into<String>, polygon: Vec<GeoPoint>) -> Self {
        Self {
            id: id.into(),
            polygon,
            access_point: None,
            probability: None,
        }
    }

    pub(crate) fn validate(&self) -> Result<(), SarError> {
        if self.polygon.len() < 3 || self.polygon.iter().any(|point| invalid_point(*point)) {
            return Err(SarError::invalid(format!(
                "segment {} needs a polygon of at least three valid points",
                self.id
            )));
        }
        if self.access_point.is_some_and(invalid_point) {
            return Err(SarError::invalid(format!(
                "segment {} access point",
                self.id
            )));
        }
        if self.probability.is_some_and(|value| !unit_interval(value)) {
            return Err(SarError::invalid(format!(
                "segment {} probability must be in [0, 1]",
                self.id
            )));
        }
        Ok(())
    }

    pub(crate) fn access(&self) -> GeoPoint {
        self.access_point.unwrap_or_else(|| centroid(&self.polygon))
    }
}

/// Which team goes where first. See `docs/design/sar-planning.md` 2.5.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct TeamPlanRequest {
    /// 1 to 64 teams, ids unique.
    pub teams: Vec<SarTeam>,
    /// 1 to 1,000 segments, ids unique.
    pub segments: Vec<SearchSegment>,
    /// A reach prediction's probability (`ReachResult::probability`), for the
    /// segments that set none and for territories.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub probability: Option<ProbabilityMap>,
    /// With a shift length (seconds from now), each team's territory: the
    /// paths it reaches first within the shift.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub shift_s: Option<f64>,
    /// Time bands of a territory, 1 to 16 (default 8): arrival times are
    /// exact to within one band.
    #[serde(default = "default_territory_bands")]
    pub territory_bands: usize,
    /// The pace model of on-foot teams that set none; by default the
    /// built-in E4 model.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pace: Option<PaceModel>,
    /// `no_go` zones block teams too; other zones are ignored here.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub zones: Vec<SarZone>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub overlays: Vec<RouteOverlay>,
    #[serde(default)]
    pub options: RouteOptions,
    /// `true` (default): the walk or drive from a team's start point and to a
    /// segment's access point off the network is priced at the profile's
    /// off-road pace and is in the arrival times (costed access legs, within
    /// 2 km for walkers), unless `options.offNetwork` sets its own. `false`
    /// snaps those points to the network for free.
    #[serde(default = "yes")]
    pub off_trail: bool,
    /// The time overlay windows are evaluated at (UTC ms).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub at_ms: Option<i64>,
    /// `false` leaves out the territories' `edgeIds`.
    #[serde(default = "yes")]
    pub include_edge_ids: bool,
}

impl TeamPlanRequest {
    pub fn new(teams: Vec<SarTeam>, segments: Vec<SearchSegment>) -> Self {
        Self {
            teams,
            segments,
            probability: None,
            shift_s: None,
            territory_bands: default_territory_bands(),
            pace: None,
            zones: Vec::new(),
            overlays: Vec::new(),
            options: RouteOptions::default(),
            off_trail: true,
            at_ms: None,
            include_edge_ids: true,
        }
    }

    fn validate(&self) -> Result<(), SarError> {
        if self.teams.is_empty() || self.teams.len() > MAX_TEAMS {
            return Err(SarError::invalid("a plan needs 1 to 64 teams"));
        }
        if self.segments.is_empty() || self.segments.len() > MAX_SEGMENTS {
            return Err(SarError::invalid("a plan needs 1 to 1000 segments"));
        }
        unique(self.teams.iter().map(|team| team.id.as_str()), "team")?;
        unique(
            self.segments.iter().map(|segment| segment.id.as_str()),
            "segment",
        )?;
        for team in &self.teams {
            if invalid_point(team.point)
                || !team.start_offset_s.is_finite()
                || team.start_offset_s < 0.0
                || !finite_positive(team.pace_factor)
                || team.pace_factor > 5.0
            {
                return Err(SarError::invalid(format!(
                    "team {}: valid point, start offset >= 0 and pace factor in (0, 5]",
                    team.id
                )));
            }
        }
        for segment in &self.segments {
            segment.validate()?;
        }
        for pace in self
            .teams
            .iter()
            .filter_map(|team| team.pace.as_ref())
            .chain(self.pace.as_ref())
        {
            pace.validate()
                .map_err(|error| SarError::invalid(format!("pace model: {error}")))?;
        }
        for zone in &self.zones {
            zone.validate()?;
        }
        if let Some(probability) = &self.probability {
            probability.validate()?;
        }
        if self.shift_s.is_some_and(|shift| !finite_positive(shift)) {
            return Err(SarError::invalid("shiftS must be above 0"));
        }
        if !(1..=16).contains(&self.territory_bands) {
            return Err(SarError::invalid("territoryBands must be 1 to 16"));
        }
        Ok(())
    }
}

pub(crate) fn unique<'a>(ids: impl Iterator<Item = &'a str>, what: &str) -> Result<(), SarError> {
    let mut seen = std::collections::BTreeSet::new();
    for id in ids {
        if !seen.insert(id) {
            return Err(SarError::invalid(format!("{what} id {id} is repeated")));
        }
    }
    Ok(())
}

fn on_foot() -> MobilityProfile {
    MobilityProfile::OnFoot
}
fn one() -> f64 {
    1.0
}
fn yes() -> bool {
    true
}
fn default_territory_bands() -> usize {
    8
}

/// The travel-time multiplier a team's arrivals use.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct TeamPace {
    pub team_id: String,
    /// The pace model's source; none for teams not on foot.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<PaceSource>,
    /// The pace model's expected time over the engine's on the segments'
    /// paths (1 for teams not on foot).
    pub pace_ratio: f64,
    /// `pace_ratio * pace_factor`: the multiplier on engine travel time.
    pub time_factor: f64,
}

/// One cell of the arrival matrix.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct TeamArrival {
    pub team_id: String,
    pub segment_id: String,
    /// Seconds from now until the team reaches the segment's access point
    /// (start offset plus travel at its pace); none when it cannot.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub arrival_s: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distance_m: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct SegmentPlan {
    pub segment_id: String,
    /// Probability of area: the planner's, or summed from the edges inside.
    pub probability: f64,
    /// Paths inside the segment (both directions once).
    pub network_length_m: f64,
    pub edge_count: usize,
    pub access_point: GeoPoint,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub first_team: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub first_arrival_s: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct TeamAssignment {
    pub team_id: String,
    pub segment_id: String,
    pub arrival_s: f64,
    pub probability: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct TeamTerritory {
    pub team_id: String,
    pub edge_count: usize,
    pub network_length_m: f64,
    /// The plan's probability on the territory's edges.
    pub probability: f64,
    /// Undirected edge keys, sorted.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub edge_ids: Vec<u64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct TeamPlanResult {
    /// Each team's travel-time multiplier, in team order.
    pub paces: Vec<TeamPace>,
    /// Every team against every segment, team-major in request order.
    pub arrivals: Vec<TeamArrival>,
    /// In request order.
    pub segments: Vec<SegmentPlan>,
    /// A first assignment: segments by probability, highest first, each to
    /// the free team that arrives first (ties: the earlier team). A team
    /// takes one segment; segments no free team reaches stay unassigned.
    pub assignments: Vec<TeamAssignment>,
    /// With `shift_s`, in team order.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub territories: Vec<TeamTerritory>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub warnings: Vec<String>,
}

/// Plans the first assignment of teams to segments (see [`TeamPlanRequest`]).
pub fn plan_teams(
    engine: &RoutingEngine,
    request: &TeamPlanRequest,
) -> Result<TeamPlanResult, SarError> {
    request.validate()?;
    let mut warnings = Vec::new();
    let mut edges = Edges::new(engine);
    let overlays = [
        request.overlays.clone(),
        zone_overlays(&request.zones, &[SarZoneKind::NoGo]),
    ]
    .concat();
    let edge_probability = request
        .probability
        .as_ref()
        .map(ProbabilityMap::by_edge)
        .unwrap_or_default();
    if request.probability.is_none()
        && request
            .segments
            .iter()
            .any(|segment| segment.probability.is_none())
    {
        warnings.push("segments without a probability and no probability map count as 0".into());
    }

    // Segments: probability and network inside.
    let mut segments = Vec::with_capacity(request.segments.len());
    let mut searched = std::collections::BTreeSet::new();
    for segment in &request.segments {
        let keys = edges.network().keys_in_polygon(&segment.polygon);
        searched.extend(keys.iter().copied());
        let length = keys.iter().map(|key| edges.length(*key)).sum();
        let probability = segment.probability.unwrap_or_else(|| {
            keys.iter()
                .filter_map(|key| edge_probability.get(key))
                .sum::<f64>()
        });
        segments.push(SegmentPlan {
            segment_id: segment.id.clone(),
            probability,
            network_length_m: length,
            edge_count: keys.len(),
            access_point: segment.access(),
            first_team: None,
            first_arrival_s: None,
        });
    }

    // Each team's multiplier on engine time: its pace model's ratio on the
    // segments' paths (on foot), times its own factor.
    let default_pace = PaceModel::default();
    let paces = request
        .teams
        .iter()
        .map(|team| {
            let (source, ratio) = if team.profile == MobilityProfile::OnFoot {
                let pace = team
                    .pace
                    .as_ref()
                    .or(request.pace.as_ref())
                    .unwrap_or(&default_pace);
                let ratio = pace_ratio(
                    engine,
                    pace,
                    team.profile,
                    &request.options.costing,
                    searched.iter().copied(),
                )
                .unwrap_or_else(|| footpath_ratio(pace));
                (Some(pace.source.clone()), ratio)
            } else {
                (None, 1.0)
            };
            TeamPace {
                team_id: team.id.clone(),
                source,
                pace_ratio: ratio,
                time_factor: ratio * team.pace_factor,
            }
        })
        .collect::<Vec<_>>();

    // Arrival matrix, one engine matrix per profile.
    let mut arrival = vec![vec![None::<(f64, f64)>; request.segments.len()]; request.teams.len()];
    let mut profiles: Vec<MobilityProfile> = Vec::new();
    for team in &request.teams {
        if !profiles.contains(&team.profile) {
            profiles.push(team.profile);
        }
    }
    let targets = segments
        .iter()
        .map(|segment| segment.access_point)
        .collect::<Vec<_>>();
    for profile in profiles {
        let members = (0..request.teams.len())
            .filter(|index| request.teams[*index].profile == profile)
            .collect::<Vec<_>>();
        let mut matrix = RouteMatrixRequest::new(
            members
                .iter()
                .map(|index| request.teams[*index].point)
                .collect(),
            targets.clone(),
            profile,
        );
        matrix.options = search_options(&request.options, profile, request.off_trail);
        matrix.overlays = overlays.clone();
        matrix.departure_at_ms = request.at_ms;
        matrix.include_edge_ids = false;
        let result = engine.matrix(matrix);
        for cell in result.cells {
            if cell.status != RouteMatrixCellStatus::Reachable {
                continue;
            }
            let team_index = members[cell.source_index];
            let team = &request.teams[team_index];
            let duration = cell.duration_s.unwrap_or(0.0);
            arrival[team_index][cell.target_index] = Some((
                team.start_offset_s + duration * paces[team_index].time_factor,
                cell.distance_m.unwrap_or(0.0),
            ));
        }
    }
    let mut arrivals = Vec::new();
    for (team_index, team) in request.teams.iter().enumerate() {
        for (segment_index, segment) in segments.iter().enumerate() {
            let cell = arrival[team_index][segment_index];
            arrivals.push(TeamArrival {
                team_id: team.id.clone(),
                segment_id: segment.segment_id.clone(),
                arrival_s: cell.map(|(time, _)| time),
                distance_m: cell.map(|(_, distance)| distance),
            });
        }
    }
    for (segment_index, segment) in segments.iter_mut().enumerate() {
        let first = (0..request.teams.len())
            .filter_map(|team| arrival[team][segment_index].map(|(time, _)| (time, team)))
            .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        if let Some((time, team)) = first {
            segment.first_team = Some(request.teams[team].id.clone());
            segment.first_arrival_s = Some(time);
        } else {
            warnings.push(format!("no team reaches segment {}", segment.segment_id));
        }
    }

    // Greedy first assignment.
    let mut order = (0..segments.len()).collect::<Vec<_>>();
    order.sort_by(|a, b| {
        segments[*b]
            .probability
            .total_cmp(&segments[*a].probability)
            .then(a.cmp(b))
    });
    let mut free = vec![true; request.teams.len()];
    let mut assignments = Vec::new();
    for segment_index in order {
        let best = (0..request.teams.len())
            .filter(|team| free[*team])
            .filter_map(|team| arrival[team][segment_index].map(|(time, _)| (time, team)))
            .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        if let Some((time, team)) = best {
            free[team] = false;
            assignments.push(TeamAssignment {
                team_id: request.teams[team].id.clone(),
                segment_id: segments[segment_index].segment_id.clone(),
                arrival_s: time,
                probability: segments[segment_index].probability,
            });
        }
        if !free.contains(&true) {
            break;
        }
    }

    let territories = match request.shift_s {
        Some(shift) => territories(
            engine,
            request,
            &paces,
            shift,
            &overlays,
            &edge_probability,
            &mut edges,
        )?,
        None => Vec::new(),
    };

    Ok(TeamPlanResult {
        paces,
        arrivals,
        segments,
        assignments,
        territories,
        warnings,
    })
}

/// Each team's isochrone within the shift; every edge to the team that
/// reaches it first (ties: the earlier team).
fn territories(
    engine: &RoutingEngine,
    request: &TeamPlanRequest,
    paces: &[TeamPace],
    shift: f64,
    overlays: &[RouteOverlay],
    edge_probability: &BTreeMap<u64, f64>,
    edges: &mut Edges,
) -> Result<Vec<TeamTerritory>, SarError> {
    let mut first = BTreeMap::<u64, (f64, usize)>::new();
    for (team_index, team) in request.teams.iter().enumerate() {
        let time_factor = paces[team_index].time_factor;
        let budget = (shift - team.start_offset_s) / time_factor;
        if budget <= 0.0 {
            continue;
        }
        let bands = (1..=request.territory_bands)
            .map(|band| budget * band as f64 / request.territory_bands as f64)
            .collect::<Vec<_>>();
        let mut isochrone = RouteIsochroneRequest::new(
            team.point,
            team.profile,
            bands
                .iter()
                .map(|contour| RouteIsochroneContour {
                    contour: *contour,
                    metric: RouteIsochroneMetric::TimeSeconds,
                })
                .collect(),
        );
        isochrone.order = RouteIsochroneOrder::Time;
        isochrone.options = search_options(&request.options, team.profile, request.off_trail);
        isochrone.overlays = overlays.to_vec();
        isochrone.departure_at_ms = request.at_ms;
        let result = engine.isochrone(isochrone)?;
        for contour in &bands {
            let arrival = team.start_offset_s + contour * time_factor;
            for bucket in result
                .buckets
                .iter()
                .filter(|bucket| bucket.contour == *contour)
            {
                for edge_id in &bucket.edge_ids {
                    let key = edges.network().key(*edge_id);
                    let entry = first.entry(key).or_insert((arrival, team_index));
                    if (arrival, team_index) < *entry {
                        *entry = (arrival, team_index);
                    }
                }
            }
        }
    }
    let mut by_team = vec![Vec::new(); request.teams.len()];
    for (key, (_, team)) in first {
        by_team[team].push(key);
    }
    Ok(request
        .teams
        .iter()
        .zip(by_team)
        .map(|(team, keys)| TeamTerritory {
            team_id: team.id.clone(),
            edge_count: keys.len(),
            network_length_m: keys.iter().map(|key| edges.length(*key)).sum(),
            probability: keys
                .iter()
                .filter_map(|key| edge_probability.get(key))
                .sum(),
            edge_ids: if request.include_edge_ids {
                keys
            } else {
                Vec::new()
            },
        })
        .collect())
}

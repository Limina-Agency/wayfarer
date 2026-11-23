use std::borrow::Cow;

use crate::geometry::{GeoJsonLineString, decode_polyline6};
use crate::types::{
    GeoPoint, LegResult, RouteManeuver, RouteManeuverKind, RouteShapeDynamicOverlay,
    RouteShapeQualityFlag, RouteShapeSegment,
};
use serde::{Deserialize, Serialize};

const DEFAULT_OFF_ROUTE_THRESHOLD_M: f64 = 35.0;
const DEFAULT_OFF_ROUTE_SUSTAIN_MS: i64 = 15_000;
const DEFAULT_OFF_ROUTE_SUSTAIN_SAMPLES: u32 = 3;
const DEFAULT_ARRIVAL_THRESHOLD_M: f64 = 20.0;
const DEFAULT_APPROACHING_MANEUVER_DISTANCE_M: f64 = 80.0;
const LOW_ACCURACY_THRESHOLD_M: f64 = 120.0;
/// The stretch of route a sample is snapped in, around the previous progress:
/// a little behind it (position noise) and well ahead of it.
const PROGRESS_WINDOW_BEHIND_M: f64 = 50.0;
const PROGRESS_WINDOW_AHEAD_M: f64 = 500.0;
/// Below this speed a sample's heading is noise and is not used.
const MIN_HEADING_SPEED_M_PER_S: f64 = 1.0;
const SNAP_DIAGNOSTIC_CANDIDATES: usize = 4;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct NavigationPositionSample {
    pub position: GeoPoint,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub heading_degrees: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed_meters_per_second: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sampled_at_ms: Option<i64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub accuracy_meters: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub provider: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteNavigationProgressOptions {
    #[serde(default = "default_off_route_threshold_m")]
    pub off_route_threshold_meters: f64,
    #[serde(default = "default_off_route_sustain_ms")]
    pub off_route_sustain_ms: i64,
    /// Consecutive off-route samples that make a sustained off-route state
    /// when samples carry no `sampled_at_ms`.
    #[serde(default = "default_off_route_sustain_samples")]
    pub off_route_sustain_samples: u32,
    #[serde(default = "default_arrival_threshold_m")]
    pub arrival_threshold_meters: f64,
    #[serde(default = "default_approaching_maneuver_distance_m")]
    pub approaching_maneuver_distance_meters: f64,
}

impl Default for RouteNavigationProgressOptions {
    fn default() -> Self {
        Self {
            off_route_threshold_meters: DEFAULT_OFF_ROUTE_THRESHOLD_M,
            off_route_sustain_ms: DEFAULT_OFF_ROUTE_SUSTAIN_MS,
            off_route_sustain_samples: DEFAULT_OFF_ROUTE_SUSTAIN_SAMPLES,
            arrival_threshold_meters: DEFAULT_ARRIVAL_THRESHOLD_M,
            approaching_maneuver_distance_meters: DEFAULT_APPROACHING_MANEUVER_DISTANCE_M,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteNavigationInput {
    /// The route's points; or empty, with `polyline6`.
    #[serde(default)]
    pub geometry: Vec<GeoPoint>,
    /// The route's points as an encoded polyline at precision 6 (as a route
    /// with `geometryFormat: "polyline6"` returns it), in place of
    /// `geometry`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub polyline6: Option<String>,
    #[serde(default)]
    pub maneuvers: Vec<RouteManeuver>,
    #[serde(default)]
    pub shape_segments: Vec<RouteShapeSegment>,
    /// The duration of each of `shape_segments`, for the remaining duration;
    /// without them it is the total duration in proportion to distance.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub shape_segment_durations_seconds: Vec<f64>,
    /// The speed limit (km/h) of each of `shape_segments`' edges, `null`
    /// where unknown or for a connector, for progress's `speedLimitKph`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub shape_segment_speed_limits_kph: Vec<Option<f32>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distance_meters: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub duration_seconds: Option<f64>,
}

impl RouteNavigationInput {
    /// The navigation input of a route: its geometry, maneuvers and shape
    /// segments, each graph segment with its edge's duration (connectors
    /// have none of their own).
    pub fn from_route(route: &crate::RouteResult) -> Self {
        // Shape segments come from the routed legs in order, as do their
        // edges.
        let mut edges = route.edge_attributions.iter();
        let mut shape_segments = Vec::new();
        let mut shape_segment_durations_seconds = Vec::new();
        let mut shape_segment_speed_limits_kph = Vec::new();
        for shape in &route.shape_segments {
            let edge = if shape
                .segment
                .quality_flags
                .contains(&RouteShapeQualityFlag::SsvgConnector)
            {
                None
            } else {
                edges.next()
            };
            shape_segments.push(shape.segment.clone());
            shape_segment_durations_seconds.push(edge.map_or(0.0, |edge| edge.duration_s));
            shape_segment_speed_limits_kph.push(edge.and_then(|edge| edge.maxspeed_kph));
        }
        Self {
            geometry: carried_points(&route.geometry, route.geojson.as_ref()),
            polyline6: route.polyline6.clone(),
            maneuvers: route.maneuvers.clone(),
            shape_segments,
            shape_segment_durations_seconds,
            shape_segment_speed_limits_kph: speed_limits_if_any(shape_segment_speed_limits_kph),
            distance_meters: Some(route.distance_m),
            duration_seconds: Some(route.duration_s),
        }
    }

    /// The navigation input of one leg ([`crate::RoutingEngine::route_leg`]).
    pub fn from_leg(route: &LegResult) -> Self {
        let mut shape_segments = Vec::new();
        let mut shape_segment_durations_seconds = Vec::new();
        let mut shape_segment_speed_limits_kph = Vec::new();
        let mut shape_offset = 0;
        for leg in &route.legs {
            let mut edges = leg.edges.iter();
            for segment in &leg.shape_segments {
                let mut segment = segment.clone();
                segment.begin_shape_index += shape_offset;
                segment.end_shape_index += shape_offset;
                let edge = if segment
                    .quality_flags
                    .contains(&RouteShapeQualityFlag::SsvgConnector)
                {
                    None
                } else {
                    edges.next()
                };
                shape_segments.push(segment);
                shape_segment_durations_seconds.push(edge.map_or(0.0, |edge| edge.duration_s));
                shape_segment_speed_limits_kph.push(edge.and_then(|edge| edge.maxspeed_kph));
            }
            shape_offset += leg.geometry.len().saturating_sub(1);
        }
        Self {
            geometry: carried_points(&route.geometry, route.geojson.as_ref()),
            polyline6: route.polyline6.clone(),
            maneuvers: route.maneuvers.clone(),
            shape_segments,
            shape_segment_durations_seconds,
            shape_segment_speed_limits_kph: speed_limits_if_any(shape_segment_speed_limits_kph),
            distance_meters: Some(route.distance_m),
            duration_seconds: Some(route.duration_s),
        }
    }
}

/// The speed limits of a route's shape segments, or none when no segment has
/// one (the input then leaves the key out).
fn speed_limits_if_any(limits: Vec<Option<f32>>) -> Vec<Option<f32>> {
    if limits.iter().any(Option::is_some) {
        limits
    } else {
        Vec::new()
    }
}

/// A route's points, from its GeoJSON when they were moved there (a polyline
/// is carried as it is, and decoded when the route is prepared).
fn carried_points(geometry: &[GeoPoint], geojson: Option<&GeoJsonLineString>) -> Vec<GeoPoint> {
    match geojson {
        Some(line) if geometry.is_empty() => line.to_points(),
        _ => geometry.to_vec(),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteNavigationStatus {
    OnRoute,
    ApproachingManeuver,
    OffRouteCandidate,
    OffRoute,
    Arrived,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteNavigationProgress {
    pub status: RouteNavigationStatus,
    pub raw_position: GeoPoint,
    pub snapped_position: GeoPoint,
    pub cross_track_error_meters: f64,
    pub distance_along_route_meters: f64,
    pub distance_remaining_meters: f64,
    pub duration_remaining_seconds: f64,
    pub effective_off_route_threshold_meters: f64,
    pub low_accuracy: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sample_age_ms: Option<i64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub position_provider: Option<String>,
    pub route_fraction: f64,
    pub active_shape_index: usize,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub active_segment_index: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub active_edge_id: Option<u64>,
    /// The speed limit (km/h) of the active edge, from the input's
    /// `shapeSegmentSpeedLimitsKph`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed_limit_kph: Option<f32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub active_dynamic_overlay: Option<RouteShapeDynamicOverlay>,
    pub snap_diagnostics: RouteNavigationSnapDiagnostics,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub current_maneuver_index: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub next_maneuver_index: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distance_to_next_maneuver_meters: Option<f64>,
    #[serde(default)]
    pub upcoming_maneuver_indices: Vec<usize>,
    pub off_route_candidate: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub off_route_candidate_since_ms: Option<i64>,
    /// Consecutive samples, this one included, that were off-route candidates.
    #[serde(default)]
    pub off_route_candidate_sample_count: u32,
    pub reroute_suggested: bool,
    pub arrived: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteNavigationSnapDiagnostics {
    pub candidate_count: usize,
    pub rejected_candidate_count: usize,
    pub accepted_candidate: RouteNavigationSnapCandidate,
    #[serde(default)]
    pub candidates: Vec<RouteNavigationSnapCandidate>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteNavigationSnapCandidateStatus {
    Accepted,
    Rejected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteNavigationSnapCandidateReason {
    /// Accepted: the best of the whole route (no previous progress, or the
    /// route was rejoined elsewhere).
    NearestRouteShapeSegment,
    /// Accepted: the best around the previous progress.
    WithinProgressWindow,
    OutsideProgressWindow,
    WeakerHeadingAlignment,
    FartherFromSample,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteNavigationSnapCandidate {
    pub status: RouteNavigationSnapCandidateStatus,
    pub reason: RouteNavigationSnapCandidateReason,
    pub segment_index: usize,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub edge_id: Option<u64>,
    pub begin_shape_index: usize,
    pub end_shape_index: usize,
    pub snapped_position: GeoPoint,
    pub distance_meters: f64,
    pub distance_along_route_meters: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub heading_delta_degrees: Option<u16>,
    pub distance_score: f64,
    pub heading_score: f64,
    pub total_score: f64,
}

/// Progress for one sample with no memory of earlier ones: the sample is
/// snapped to the whole route. Prefer [`PreparedNavigationRoute::progress`],
/// which follows the route from the previous progress.
pub fn evaluate_route_navigation_progress(
    input: &RouteNavigationInput,
    sample: NavigationPositionSample,
    previous_off_route_candidate_since_ms: Option<i64>,
    options: RouteNavigationProgressOptions,
) -> Result<RouteNavigationProgress, String> {
    PreparedNavigationRoute::new(input)?.evaluate(
        sample,
        Carry {
            distance_along_m: None,
            off_route_candidate_since_ms: previous_off_route_candidate_since_ms,
            off_route_candidate_sample_count: u32::from(
                previous_off_route_candidate_since_ms.is_some(),
            ),
        },
        &options,
    )
}

/// A route prepared once for navigation: distances and durations along its
/// geometry, so each sample is snapped and timed without rebuilding them.
pub struct PreparedNavigationRoute<'a> {
    input: &'a RouteNavigationInput,
    /// `input.geometry`, or its decoded `polyline6`.
    geometry: Cow<'a, [GeoPoint]>,
    cumulative_distances: Vec<f64>,
    total_distance_m: f64,
    /// Seconds from each shape index to the end of the route.
    remaining_seconds: Vec<f64>,
    maneuver_distances: Vec<f64>,
}

/// What a sample's progress takes from the previous one.
struct Carry {
    distance_along_m: Option<f64>,
    off_route_candidate_since_ms: Option<i64>,
    off_route_candidate_sample_count: u32,
}

impl<'a> PreparedNavigationRoute<'a> {
    pub fn new(input: &'a RouteNavigationInput) -> Result<Self, String> {
        let geometry = match &input.polyline6 {
            None => Cow::Borrowed(input.geometry.as_slice()),
            Some(_) if !input.geometry.is_empty() => {
                return Err("route has both geometry and polyline6; give one".to_string());
            }
            Some(encoded) => Cow::Owned(
                decode_polyline6(encoded).map_err(|error| format!("route polyline6: {error}"))?,
            ),
        };
        if geometry.len() < 2 {
            return Err("route geometry must contain at least two points".to_string());
        }
        let mut cumulative_distances = Vec::with_capacity(geometry.len());
        cumulative_distances.push(0.0);
        let mut total = 0.0;
        for segment in geometry.windows(2) {
            total += segment[0].distance_to(segment[1]);
            cumulative_distances.push(total);
        }
        if total <= f64::EPSILON {
            return Err("route geometry length is zero".to_string());
        }
        let segment_seconds = geometry_segment_seconds(input, &cumulative_distances, total);
        let mut remaining_seconds = vec![0.0; geometry.len()];
        for index in (0..segment_seconds.len()).rev() {
            remaining_seconds[index] = remaining_seconds[index + 1] + segment_seconds[index];
        }
        let maneuver_distances = input
            .maneuvers
            .iter()
            .map(|maneuver| {
                cumulative_distances
                    .get(maneuver.begin_shape_index)
                    .copied()
                    .unwrap_or(total)
            })
            .collect();
        Ok(Self {
            input,
            geometry,
            cumulative_distances,
            total_distance_m: total,
            remaining_seconds,
            maneuver_distances,
        })
    }

    /// The route's points (decoded from `polyline6` when it came so).
    pub fn geometry(&self) -> &[GeoPoint] {
        &self.geometry
    }

    /// Progress for `sample`, following the route from `previous` (the last
    /// sample's progress on this route): the sample snaps near it, so an
    /// out-and-back road, a switchback or a loop does not jump ahead.
    pub fn progress(
        &self,
        sample: NavigationPositionSample,
        previous: Option<&RouteNavigationProgress>,
        options: &RouteNavigationProgressOptions,
    ) -> Result<RouteNavigationProgress, String> {
        let carry = match previous {
            Some(previous) => Carry {
                distance_along_m: Some(previous.route_fraction * self.total_distance_m),
                off_route_candidate_since_ms: previous.off_route_candidate_since_ms,
                off_route_candidate_sample_count: previous.off_route_candidate_sample_count,
            },
            None => Carry {
                distance_along_m: None,
                off_route_candidate_since_ms: None,
                off_route_candidate_sample_count: 0,
            },
        };
        self.evaluate(sample, carry, options)
    }

    fn evaluate(
        &self,
        sample: NavigationPositionSample,
        carry: Carry,
        options: &RouteNavigationProgressOptions,
    ) -> Result<RouteNavigationProgress, String> {
        let input = self.input;
        let effective_off_route_threshold_meters = effective_off_route_threshold(
            options.off_route_threshold_meters,
            sample.accuracy_meters,
        );
        let snaps = self.snap(
            &sample,
            carry.distance_along_m,
            effective_off_route_threshold_meters,
        )?;
        let snap = snaps.accepted;
        let total_distance = input.distance_meters.unwrap_or(self.total_distance_m);
        let distance_along = snap.distance_along_m.clamp(0.0, self.total_distance_m);
        let distance_remaining = (self.total_distance_m - distance_along).max(0.0);
        let duration_remaining = self.remaining_seconds[snap.segment_index + 1]
            + (self.remaining_seconds[snap.segment_index]
                - self.remaining_seconds[snap.segment_index + 1])
                * (1.0 - snap.t);
        let route_fraction = distance_along / self.total_distance_m;

        let active_segment = active_shape_segment(&input.shape_segments, snap.segment_index);
        let snap_diagnostics = self.snap_diagnostics(&snaps);
        let current_maneuver_index =
            current_maneuver_index(&self.maneuver_distances, distance_along);
        let next_maneuver_index =
            next_maneuver_index(&input.maneuvers, &self.maneuver_distances, distance_along);
        let distance_to_next_maneuver_meters = next_maneuver_index
            .map(|index| (self.maneuver_distances[index] - distance_along).max(0.0));
        let upcoming_maneuver_indices = upcoming_maneuver_indices(
            &input.maneuvers,
            &self.maneuver_distances,
            distance_along,
            next_maneuver_index,
        );

        let low_accuracy = sample
            .accuracy_meters
            .map(|accuracy| accuracy > LOW_ACCURACY_THRESHOLD_M)
            .unwrap_or(false);
        let sample_age_ms = sample
            .sampled_at_ms
            .and_then(|sampled_at| current_time_ms().map(|now| now.saturating_sub(sampled_at)));
        let off_route_candidate = snap.distance_m > effective_off_route_threshold_meters;
        let (off_route_candidate_since_ms, off_route_candidate_sample_count) =
            if off_route_candidate {
                (
                    carry.off_route_candidate_since_ms.or(sample.sampled_at_ms),
                    carry.off_route_candidate_sample_count.saturating_add(1),
                )
            } else {
                (None, 0)
            };
        // Elapsed time when samples are timed, else a run of off-route samples.
        let sustained_off_route = off_route_candidate
            && match (sample.sampled_at_ms, off_route_candidate_since_ms) {
                (Some(now_ms), Some(since)) => {
                    now_ms.saturating_sub(since) >= options.off_route_sustain_ms
                }
                _ => off_route_candidate_sample_count >= options.off_route_sustain_samples.max(1),
            };
        // Only on the route's final stretch: `distance_remaining` follows the
        // snap, which follows the previous progress.
        let arrived = distance_remaining <= options.arrival_threshold_meters
            && snap.distance_m <= effective_off_route_threshold_meters;
        let approaching_maneuver = distance_to_next_maneuver_meters
            .map(|distance| distance <= options.approaching_maneuver_distance_meters)
            .unwrap_or(false);
        let status = if arrived {
            RouteNavigationStatus::Arrived
        } else if sustained_off_route {
            RouteNavigationStatus::OffRoute
        } else if off_route_candidate {
            RouteNavigationStatus::OffRouteCandidate
        } else if approaching_maneuver {
            RouteNavigationStatus::ApproachingManeuver
        } else {
            RouteNavigationStatus::OnRoute
        };

        Ok(RouteNavigationProgress {
            status,
            raw_position: sample.position,
            snapped_position: snap.point,
            cross_track_error_meters: snap.distance_m,
            distance_along_route_meters: if total_distance > f64::EPSILON {
                total_distance * route_fraction
            } else {
                distance_along
            },
            distance_remaining_meters: if total_distance > f64::EPSILON {
                (total_distance * (1.0 - route_fraction)).max(0.0)
            } else {
                distance_remaining
            },
            duration_remaining_seconds: duration_remaining.max(0.0),
            effective_off_route_threshold_meters,
            low_accuracy,
            sample_age_ms,
            position_provider: sample.provider,
            route_fraction,
            active_shape_index: snap.segment_index,
            active_segment_index: active_segment.map(|(index, _)| index),
            active_edge_id: active_segment.map(|(_, segment)| segment.edge_id),
            speed_limit_kph: active_segment.and_then(|(index, _)| {
                self.input
                    .shape_segment_speed_limits_kph
                    .get(index)
                    .copied()
                    .flatten()
            }),
            active_dynamic_overlay: active_segment
                .and_then(|(_, segment)| segment.dynamic_overlay.clone()),
            snap_diagnostics,
            current_maneuver_index,
            next_maneuver_index,
            distance_to_next_maneuver_meters,
            upcoming_maneuver_indices,
            off_route_candidate,
            off_route_candidate_since_ms,
            off_route_candidate_sample_count,
            reroute_suggested: matches!(status, RouteNavigationStatus::OffRoute) && !low_accuracy,
            arrived,
        })
    }

    /// Snaps the sample to every geometry segment in one pass, keeping the best
    /// of the whole route, the best within the window around
    /// `previous_distance_along_m` and the best few for diagnostics.
    fn snap(
        &self,
        sample: &NavigationPositionSample,
        previous_distance_along_m: Option<f64>,
        off_route_threshold_m: f64,
    ) -> Result<Snaps, String> {
        let geometry = &self.geometry;
        let point = sample.position;
        let meters_per_lat = 111_320.0;
        let meters_per_lng = 111_320.0 * point.lat.to_radians().cos().abs().max(0.01);
        let to_xy = |candidate: GeoPoint| -> (f64, f64) {
            (
                (candidate.lng - point.lng) * meters_per_lng,
                (candidate.lat - point.lat) * meters_per_lat,
            )
        };
        let heading = sample.heading_degrees.filter(|_| {
            sample
                .speed_meters_per_second
                .is_none_or(|speed| speed >= MIN_HEADING_SPEED_M_PER_S)
        });
        let window = previous_distance_along_m.map(|distance| {
            (
                distance - PROGRESS_WINDOW_BEHIND_M,
                distance + PROGRESS_WINDOW_AHEAD_M,
            )
        });

        let mut snaps = Snaps::default();
        let mut best = None::<SegmentSnap>;
        let mut best_in_window = None::<SegmentSnap>;
        for (index, segment) in geometry.windows(2).enumerate() {
            let (a, b) = (segment[0], segment[1]);
            let (ax, ay) = to_xy(a);
            let (bx, by) = to_xy(b);
            let (vx, vy) = (bx - ax, by - ay);
            let length_sq = vx * vx + vy * vy;
            if length_sq <= f64::EPSILON {
                continue;
            }
            snaps.candidate_count += 1;
            let t = (-(ax * vx + ay * vy) / length_sq).clamp(0.0, 1.0);
            let (x, y) = (ax + vx * t, ay + vy * t);
            let distance_m = (x * x + y * y).sqrt();
            let (start, end) = (
                self.cumulative_distances[index],
                self.cumulative_distances[index + 1],
            );
            let heading_delta_degrees = heading.and_then(|heading| {
                heading_delta_degrees(heading, vx.atan2(vy).to_degrees())
                    .map(|delta| delta.round() as u16)
            });
            let distance_score = 1.0 / (1.0 + distance_m / 25.0);
            let heading_score = heading_delta_degrees
                .map(|delta| 1.0 - (f64::from(delta).min(180.0) / 180.0))
                .unwrap_or(1.0);
            let snap = SegmentSnap {
                segment_index: index,
                point: GeoPoint::new(a.lat + (b.lat - a.lat) * t, a.lng + (b.lng - a.lng) * t),
                distance_m,
                distance_along_m: start + (end - start) * t,
                t,
                heading_delta_degrees,
                distance_score,
                heading_score,
                total_score: (distance_score * 0.75) + (heading_score * 0.25),
            };
            if best
                .as_ref()
                .is_none_or(|current| snap.better_than(current, 0.0))
            {
                best = Some(snap);
            }
            if let (Some((from, to)), Some(previous)) = (window, previous_distance_along_m)
                && (from..=to).contains(&snap.distance_along_m)
                && best_in_window
                    .as_ref()
                    .is_none_or(|current| snap.better_than(current, previous))
            {
                best_in_window = Some(snap);
            }
            snaps.keep_for_diagnostics(snap);
        }
        let best = best.ok_or_else(|| "route geometry has no usable segments".to_string())?;
        // The window holds the sample unless the route was rejoined elsewhere:
        // off route around the previous progress, on route somewhere else.
        (snaps.accepted, snaps.windowed) = match best_in_window {
            Some(windowed)
                if windowed.distance_m <= off_route_threshold_m
                    || best.distance_m > off_route_threshold_m =>
            {
                (windowed, true)
            }
            _ => (best, false),
        };
        snaps.window = window;
        Ok(snaps)
    }

    fn snap_diagnostics(&self, snaps: &Snaps) -> RouteNavigationSnapDiagnostics {
        let accepted = snaps.accepted;
        let accepted_candidate = self.snap_candidate(
            &accepted,
            RouteNavigationSnapCandidateStatus::Accepted,
            if snaps.windowed {
                RouteNavigationSnapCandidateReason::WithinProgressWindow
            } else {
                RouteNavigationSnapCandidateReason::NearestRouteShapeSegment
            },
        );
        let mut candidates = Vec::with_capacity(SNAP_DIAGNOSTIC_CANDIDATES);
        candidates.push(accepted_candidate.clone());
        for snap in snaps
            .best
            .iter()
            .filter(|snap| snap.segment_index != accepted.segment_index)
            .take(SNAP_DIAGNOSTIC_CANDIDATES - 1)
        {
            let outside_window = snaps
                .window
                .is_some_and(|(from, to)| !(from..=to).contains(&snap.distance_along_m));
            let reason = if outside_window {
                RouteNavigationSnapCandidateReason::OutsideProgressWindow
            } else if snap.heading_delta_degrees.is_some_and(|delta| delta > 90) {
                RouteNavigationSnapCandidateReason::WeakerHeadingAlignment
            } else {
                RouteNavigationSnapCandidateReason::FartherFromSample
            };
            candidates.push(self.snap_candidate(
                snap,
                RouteNavigationSnapCandidateStatus::Rejected,
                reason,
            ));
        }
        RouteNavigationSnapDiagnostics {
            candidate_count: snaps.candidate_count,
            rejected_candidate_count: snaps.candidate_count.saturating_sub(1),
            accepted_candidate,
            candidates,
        }
    }

    fn snap_candidate(
        &self,
        snap: &SegmentSnap,
        status: RouteNavigationSnapCandidateStatus,
        reason: RouteNavigationSnapCandidateReason,
    ) -> RouteNavigationSnapCandidate {
        let shape_segment = active_shape_segment(&self.input.shape_segments, snap.segment_index)
            .map(|(_, segment)| segment);
        RouteNavigationSnapCandidate {
            status,
            reason,
            segment_index: snap.segment_index,
            edge_id: shape_segment.map(|segment| segment.edge_id),
            begin_shape_index: shape_segment
                .map(|segment| segment.begin_shape_index)
                .unwrap_or(snap.segment_index),
            end_shape_index: shape_segment
                .map(|segment| segment.end_shape_index)
                .unwrap_or(snap.segment_index + 1),
            snapped_position: snap.point,
            distance_meters: snap.distance_m,
            distance_along_route_meters: snap.distance_along_m,
            heading_delta_degrees: snap.heading_delta_degrees,
            distance_score: snap.distance_score,
            heading_score: snap.heading_score,
            total_score: snap.total_score,
        }
    }
}

/// The seconds of travel along each geometry segment: the shape segments'
/// durations spread over their geometry by length (scaled to the route's
/// total duration when it has one, which also covers connectors), else the
/// total duration in proportion to distance.
fn geometry_segment_seconds(
    input: &RouteNavigationInput,
    cumulative_distances: &[f64],
    total_distance_m: f64,
) -> Vec<f64> {
    let segment_count = cumulative_distances.len() - 1;
    let length = |index: usize| cumulative_distances[index + 1] - cumulative_distances[index];
    let mut seconds = vec![0.0; segment_count];
    if input.shape_segment_durations_seconds.len() == input.shape_segments.len() {
        for (segment, duration_s) in input
            .shape_segments
            .iter()
            .zip(&input.shape_segment_durations_seconds)
        {
            let begin = segment.begin_shape_index.min(segment_count);
            let end = segment.end_shape_index.min(segment_count);
            let span_m = cumulative_distances[end] - cumulative_distances[begin];
            if !duration_s.is_finite() || *duration_s <= 0.0 || span_m <= f64::EPSILON {
                continue;
            }
            for (index, value) in seconds.iter_mut().enumerate().take(end).skip(begin) {
                *value += duration_s * length(index) / span_m;
            }
        }
    }
    let known: f64 = seconds.iter().sum();
    let total_duration = input
        .duration_seconds
        .filter(|duration| duration.is_finite() && *duration > 0.0);
    match total_duration {
        Some(total) if known > f64::EPSILON => {
            let scale = total / known;
            seconds.iter_mut().for_each(|value| *value *= scale);
        }
        Some(total) => {
            for (index, value) in seconds.iter_mut().enumerate() {
                *value = total * length(index) / total_distance_m;
            }
        }
        None => {}
    }
    seconds
}

/// A sample's projection onto one geometry segment.
#[derive(Debug, Clone, Copy)]
struct SegmentSnap {
    segment_index: usize,
    point: GeoPoint,
    /// Cross-track error.
    distance_m: f64,
    distance_along_m: f64,
    /// The position along the segment, 0 at its start and 1 at its end.
    t: f64,
    heading_delta_degrees: Option<u16>,
    distance_score: f64,
    heading_score: f64,
    total_score: f64,
}

impl SegmentSnap {
    /// Better scored, or as well scored and nearer `reference_along_m` along
    /// the route (the previous progress, or the start).
    fn better_than(&self, other: &Self, reference_along_m: f64) -> bool {
        self.total_score > other.total_score
            || (self.total_score == other.total_score
                && (self.distance_along_m - reference_along_m).abs()
                    < (other.distance_along_m - reference_along_m).abs())
    }
}

#[derive(Debug, Default)]
struct Snaps {
    accepted: SegmentSnap,
    windowed: bool,
    window: Option<(f64, f64)>,
    candidate_count: usize,
    /// The best-scored snaps, best first.
    best: Vec<SegmentSnap>,
}

impl Default for SegmentSnap {
    fn default() -> Self {
        Self {
            segment_index: 0,
            point: GeoPoint::new(0.0, 0.0),
            distance_m: 0.0,
            distance_along_m: 0.0,
            t: 0.0,
            heading_delta_degrees: None,
            distance_score: 0.0,
            heading_score: 0.0,
            total_score: 0.0,
        }
    }
}

impl Snaps {
    fn keep_for_diagnostics(&mut self, snap: SegmentSnap) {
        let at = self
            .best
            .iter()
            .position(|kept| snap.better_than(kept, 0.0))
            .unwrap_or(self.best.len());
        if at < SNAP_DIAGNOSTIC_CANDIDATES {
            self.best.insert(at, snap);
            self.best.truncate(SNAP_DIAGNOSTIC_CANDIDATES);
        }
    }
}

fn effective_off_route_threshold(base_threshold_m: f64, accuracy_meters: Option<f64>) -> f64 {
    let Some(accuracy) = accuracy_meters else {
        return base_threshold_m;
    };
    if !accuracy.is_finite() || accuracy <= 0.0 {
        return base_threshold_m;
    }
    base_threshold_m.max((accuracy * 1.5).min(LOW_ACCURACY_THRESHOLD_M))
}

fn current_time_ms() -> Option<i64> {
    let duration = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .ok()?;
    Some(duration.as_millis().min(i64::MAX as u128) as i64)
}

fn default_off_route_threshold_m() -> f64 {
    DEFAULT_OFF_ROUTE_THRESHOLD_M
}

fn default_off_route_sustain_ms() -> i64 {
    DEFAULT_OFF_ROUTE_SUSTAIN_MS
}

fn default_off_route_sustain_samples() -> u32 {
    DEFAULT_OFF_ROUTE_SUSTAIN_SAMPLES
}

fn default_arrival_threshold_m() -> f64 {
    DEFAULT_ARRIVAL_THRESHOLD_M
}

fn default_approaching_maneuver_distance_m() -> f64 {
    DEFAULT_APPROACHING_MANEUVER_DISTANCE_M
}

fn heading_delta_degrees(a: f64, b: f64) -> Option<f64> {
    if !a.is_finite() || !b.is_finite() {
        return None;
    }
    let diff = (a.rem_euclid(360.0) - b.rem_euclid(360.0)).abs();
    Some(diff.min(360.0 - diff))
}

fn active_shape_segment(
    shape_segments: &[RouteShapeSegment],
    shape_index: usize,
) -> Option<(usize, &RouteShapeSegment)> {
    shape_segments.iter().enumerate().find(|(_, segment)| {
        segment.begin_shape_index <= shape_index && shape_index < segment.end_shape_index
    })
}

fn current_maneuver_index(maneuver_distances: &[f64], distance_along: f64) -> Option<usize> {
    maneuver_distances
        .iter()
        .enumerate()
        .filter(|(_, distance)| **distance <= distance_along + 1.0)
        .map(|(index, _)| index)
        .next_back()
}

fn next_maneuver_index(
    maneuvers: &[RouteManeuver],
    maneuver_distances: &[f64],
    distance_along: f64,
) -> Option<usize> {
    maneuvers
        .iter()
        .enumerate()
        .find(|(index, maneuver)| {
            maneuver_distances[*index] > distance_along + 1.0
                && maneuver.kind != RouteManeuverKind::Start
        })
        .map(|(index, _)| index)
}

fn upcoming_maneuver_indices(
    maneuvers: &[RouteManeuver],
    maneuver_distances: &[f64],
    distance_along: f64,
    next_maneuver_index: Option<usize>,
) -> Vec<usize> {
    let mut indices = Vec::new();
    if let Some(index) = next_maneuver_index {
        indices.push(index);
    }
    for (index, maneuver) in maneuvers.iter().enumerate() {
        if indices.len() >= 3 {
            break;
        }
        if Some(index) == next_maneuver_index || maneuver.kind == RouteManeuverKind::Start {
            continue;
        }
        if maneuver_distances[index] > distance_along + 1.0 {
            indices.push(index);
        }
    }
    indices
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{
        RouteDynamicEndpointKind, RouteDynamicVirtualEdgeKind, RouteShapeDynamicOverlayKind,
    };

    fn point(lng: f64) -> GeoPoint {
        GeoPoint::new(50.0, lng)
    }

    fn maneuver(kind: RouteManeuverKind, shape_index: usize, instruction: &str) -> RouteManeuver {
        RouteManeuver {
            kind,
            instruction: instruction.to_string(),
            concise_instruction: instruction.to_string(),
            begin_shape_index: shape_index,
            end_shape_index: shape_index,
            ..RouteManeuver::default()
        }
    }

    fn route_input() -> RouteNavigationInput {
        RouteNavigationInput {
            polyline6: None,
            geometry: vec![point(20.0), point(20.001), point(20.002)],
            maneuvers: vec![
                maneuver(RouteManeuverKind::Start, 0, "Start"),
                maneuver(RouteManeuverKind::Turn, 1, "Turn right"),
                maneuver(RouteManeuverKind::Destination, 2, "Arrive"),
            ],
            shape_segments: vec![RouteShapeSegment {
                edge_id: 7,
                begin_shape_index: 0,
                end_shape_index: 1,
                start_fraction: 0.0,
                end_fraction: 1.0,
                quality_flags: Vec::new(),
                dynamic_overlay: None,
            }],
            shape_segment_durations_seconds: Vec::new(),
            shape_segment_speed_limits_kph: Vec::new(),
            distance_meters: None,
            duration_seconds: Some(120.0),
        }
    }

    #[test]
    fn navigation_snaps_to_route_and_computes_remaining_distance() {
        let input = route_input();
        let progress = evaluate_route_navigation_progress(
            &input,
            NavigationPositionSample {
                position: point(20.0005),
                heading_degrees: None,
                speed_meters_per_second: None,
                sampled_at_ms: Some(1_000),
                accuracy_meters: None,
                provider: None,
            },
            None,
            RouteNavigationProgressOptions::default(),
        )
        .unwrap();

        assert!(progress.cross_track_error_meters < 1.0);
        assert!(progress.distance_along_route_meters > 30.0);
        assert!(progress.distance_remaining_meters > 30.0);
        assert_eq!(progress.current_maneuver_index, Some(0));
        assert_eq!(progress.next_maneuver_index, Some(1));
        assert!(progress.distance_to_next_maneuver_meters.unwrap() > 20.0);
        assert_eq!(progress.snap_diagnostics.candidate_count, 2);
        assert_eq!(progress.snap_diagnostics.rejected_candidate_count, 1);
        assert_eq!(
            progress.snap_diagnostics.accepted_candidate.reason,
            RouteNavigationSnapCandidateReason::NearestRouteShapeSegment
        );
        assert!(
            progress
                .snap_diagnostics
                .candidates
                .iter()
                .any(|candidate| candidate.status == RouteNavigationSnapCandidateStatus::Rejected)
        );
        assert_eq!(
            serde_json::to_value(&progress.snap_diagnostics.accepted_candidate).unwrap()["reason"],
            serde_json::json!("nearest_route_shape_segment")
        );
    }

    #[test]
    fn navigation_snap_diagnostics_include_heading_alignment() {
        let input = route_input();
        let progress = evaluate_route_navigation_progress(
            &input,
            NavigationPositionSample {
                position: point(20.0005),
                heading_degrees: Some(270.0),
                speed_meters_per_second: None,
                sampled_at_ms: Some(1_000),
                accuracy_meters: None,
                provider: None,
            },
            None,
            RouteNavigationProgressOptions::default(),
        )
        .unwrap();

        let accepted = &progress.snap_diagnostics.accepted_candidate;
        assert!(accepted.heading_delta_degrees.is_some());
        assert!(accepted.heading_score < 0.6);
    }

    #[test]
    fn navigation_flags_arrival_near_destination() {
        let input = route_input();
        let progress = evaluate_route_navigation_progress(
            &input,
            NavigationPositionSample {
                position: point(20.002),
                heading_degrees: None,
                speed_meters_per_second: None,
                sampled_at_ms: Some(1_000),
                accuracy_meters: None,
                provider: None,
            },
            None,
            RouteNavigationProgressOptions::default(),
        )
        .unwrap();

        assert_eq!(progress.status, RouteNavigationStatus::Arrived);
        assert!(progress.arrived);
    }

    #[test]
    fn navigation_uses_off_route_hysteresis() {
        let input = route_input();
        let options = RouteNavigationProgressOptions {
            off_route_threshold_meters: 10.0,
            off_route_sustain_ms: 15_000,
            ..RouteNavigationProgressOptions::default()
        };
        let candidate = evaluate_route_navigation_progress(
            &input,
            NavigationPositionSample {
                position: GeoPoint::new(50.001, 20.0005),
                heading_degrees: None,
                speed_meters_per_second: None,
                sampled_at_ms: Some(10_000),
                accuracy_meters: None,
                provider: None,
            },
            None,
            options.clone(),
        )
        .unwrap();
        assert_eq!(candidate.status, RouteNavigationStatus::OffRouteCandidate);

        let off_route = evaluate_route_navigation_progress(
            &input,
            NavigationPositionSample {
                position: GeoPoint::new(50.001, 20.0005),
                heading_degrees: None,
                speed_meters_per_second: None,
                sampled_at_ms: Some(26_000),
                accuracy_meters: None,
                provider: None,
            },
            candidate.off_route_candidate_since_ms,
            options,
        )
        .unwrap();
        assert_eq!(off_route.status, RouteNavigationStatus::OffRoute);
        assert!(off_route.reroute_suggested);
    }

    #[test]
    fn navigation_expands_off_route_threshold_for_coarse_accuracy() {
        let input = route_input();
        let options = RouteNavigationProgressOptions {
            off_route_threshold_meters: 10.0,
            ..RouteNavigationProgressOptions::default()
        };
        let progress = evaluate_route_navigation_progress(
            &input,
            NavigationPositionSample {
                position: GeoPoint::new(50.00025, 20.0005),
                heading_degrees: None,
                speed_meters_per_second: None,
                sampled_at_ms: Some(10_000),
                accuracy_meters: Some(40.0),
                provider: Some("browser_geolocation".to_string()),
            },
            None,
            options,
        )
        .unwrap();

        assert_eq!(progress.effective_off_route_threshold_meters, 60.0);
        assert!(!progress.low_accuracy);
        assert!(!progress.off_route_candidate);
        assert_eq!(
            progress.position_provider,
            Some("browser_geolocation".to_string())
        );
    }

    #[test]
    fn navigation_suppresses_reroute_for_low_accuracy_samples() {
        let input = route_input();
        let options = RouteNavigationProgressOptions {
            off_route_threshold_meters: 10.0,
            off_route_sustain_ms: 15_000,
            ..RouteNavigationProgressOptions::default()
        };
        let candidate = evaluate_route_navigation_progress(
            &input,
            NavigationPositionSample {
                position: GeoPoint::new(50.002, 20.0005),
                heading_degrees: None,
                speed_meters_per_second: None,
                sampled_at_ms: Some(10_000),
                accuracy_meters: Some(200.0),
                provider: Some("browser_geolocation".to_string()),
            },
            None,
            options.clone(),
        )
        .unwrap();
        let off_route = evaluate_route_navigation_progress(
            &input,
            NavigationPositionSample {
                position: GeoPoint::new(50.002, 20.0005),
                heading_degrees: None,
                speed_meters_per_second: None,
                sampled_at_ms: Some(26_000),
                accuracy_meters: Some(200.0),
                provider: Some("browser_geolocation".to_string()),
            },
            candidate.off_route_candidate_since_ms,
            options,
        )
        .unwrap();

        assert_eq!(off_route.status, RouteNavigationStatus::OffRoute);
        assert_eq!(off_route.effective_off_route_threshold_meters, 120.0);
        assert!(off_route.low_accuracy);
        assert!(!off_route.reroute_suggested);
    }

    #[test]
    fn navigation_surfaces_dynamic_overlay_segment_context() {
        let mut input = route_input();
        input.shape_segments[0].dynamic_overlay = Some(RouteShapeDynamicOverlay {
            kind: RouteShapeDynamicOverlayKind::EndpointConnector,
            connector_id: Some("connector_1".to_string()),
            connector_type: Some(RouteDynamicVirtualEdgeKind::Endpoint),
            source_chunk_ids: vec!["chunk_1".to_string()],
            distance_meters: Some(12.0),
            cost: Some(24.0),
            source_endpoint: Some(RouteDynamicEndpointKind::Origin),
            source_point: Some(point(20.0)),
            source_edge_id: None,
            source_fraction: None,
            target_endpoint: None,
            target_point: Some(point(20.0002)),
            target_edge_id: Some(7),
            target_fraction: Some(0.2),
        });

        let progress = evaluate_route_navigation_progress(
            &input,
            NavigationPositionSample {
                position: point(20.0002),
                heading_degrees: None,
                speed_meters_per_second: None,
                sampled_at_ms: Some(1_000),
                accuracy_meters: None,
                provider: None,
            },
            None,
            RouteNavigationProgressOptions::default(),
        )
        .unwrap();

        assert_eq!(progress.active_edge_id, Some(7));
        assert_eq!(
            progress.active_dynamic_overlay.unwrap().connector_id,
            Some("connector_1".to_string())
        );
    }

    fn sample(lng: f64, heading_degrees: Option<f64>) -> NavigationPositionSample {
        NavigationPositionSample {
            position: GeoPoint::new(50.00001, lng),
            heading_degrees,
            speed_meters_per_second: None,
            sampled_at_ms: None,
            accuracy_meters: None,
            provider: None,
        }
    }

    /// East along one road for four points, then back west along it.
    fn out_and_back_input() -> RouteNavigationInput {
        let lngs = [
            20.0, 20.001, 20.002, 20.003, 20.004, 20.003, 20.002, 20.001, 20.0,
        ];
        RouteNavigationInput {
            polyline6: None,
            geometry: lngs.iter().map(|lng| point(*lng)).collect(),
            maneuvers: vec![
                maneuver(RouteManeuverKind::Start, 0, "Start"),
                maneuver(RouteManeuverKind::UTurn, 4, "Make a U-turn"),
                maneuver(RouteManeuverKind::Destination, 8, "Arrive"),
            ],
            shape_segments: Vec::new(),
            shape_segment_durations_seconds: Vec::new(),
            shape_segment_speed_limits_kph: Vec::new(),
            distance_meters: None,
            duration_seconds: Some(80.0),
        }
    }

    #[test]
    fn stateful_progress_follows_an_out_and_back_trace() {
        let input = out_and_back_input();
        let route = PreparedNavigationRoute::new(&input).unwrap();
        let options = RouteNavigationProgressOptions::default();
        let trace = [
            20.0, 20.001, 20.002, 20.003, 20.0039, 20.003, 20.002, 20.001, 20.0,
        ];
        let mut previous = None::<RouteNavigationProgress>;
        let mut distances = Vec::new();
        for lng in trace {
            let progress = route
                .progress(sample(lng, None), previous.as_ref(), &options)
                .unwrap();
            distances.push(progress.distance_along_route_meters);
            if lng != 20.0 || previous.is_none() {
                assert!(!progress.arrived, "arrived early at {lng}");
            }
            previous = Some(progress);
        }
        assert!(
            distances.windows(2).all(|pair| pair[1] > pair[0]),
            "progress went backwards: {distances:?}"
        );
        let last = previous.unwrap();
        assert!(last.arrived);
        assert_eq!(last.status, RouteNavigationStatus::Arrived);

        // Without the previous progress the way back snaps onto the way out.
        let stateless = evaluate_route_navigation_progress(
            &input,
            sample(20.001, None),
            None,
            RouteNavigationProgressOptions::default(),
        )
        .unwrap();
        assert!(stateless.route_fraction < 0.5);
        assert!(!stateless.arrived);
    }

    #[test]
    fn heading_picks_the_matching_direction_of_an_out_and_back_route() {
        let input = out_and_back_input();
        let route = PreparedNavigationRoute::new(&input).unwrap();
        let options = RouteNavigationProgressOptions::default();

        let outbound = route
            .progress(sample(20.0015, Some(90.0)), None, &options)
            .unwrap();
        let inbound = route
            .progress(sample(20.0015, Some(270.0)), None, &options)
            .unwrap();

        assert!(outbound.route_fraction < 0.5);
        assert!(inbound.route_fraction > 0.5);
        assert_eq!(
            inbound.snap_diagnostics.candidates[1].reason,
            RouteNavigationSnapCandidateReason::WeakerHeadingAlignment
        );
    }

    #[test]
    fn progress_rejoins_the_route_away_from_the_window() {
        // East 143 m, then north 890 m.
        let input = RouteNavigationInput {
            polyline6: None,
            geometry: vec![
                point(20.0),
                point(20.002),
                GeoPoint::new(50.004, 20.002),
                GeoPoint::new(50.008, 20.002),
            ],
            maneuvers: Vec::new(),
            shape_segments: Vec::new(),
            shape_segment_durations_seconds: Vec::new(),
            shape_segment_speed_limits_kph: Vec::new(),
            distance_meters: None,
            duration_seconds: None,
        };
        let route = PreparedNavigationRoute::new(&input).unwrap();
        let options = RouteNavigationProgressOptions::default();
        let start = route.progress(sample(20.0, None), None, &options).unwrap();
        // On the route, far beyond the window ahead of the start (a shortcut).
        let rejoined = route
            .progress(
                NavigationPositionSample {
                    position: GeoPoint::new(50.007, 20.002),
                    ..sample(20.002, None)
                },
                Some(&start),
                &options,
            )
            .unwrap();

        assert!(rejoined.cross_track_error_meters < 1.0);
        assert!(rejoined.distance_along_route_meters > 800.0);
        assert!(!rejoined.off_route_candidate);
        assert_eq!(
            rejoined.snap_diagnostics.accepted_candidate.reason,
            RouteNavigationSnapCandidateReason::NearestRouteShapeSegment
        );
    }

    #[test]
    fn off_route_is_sustained_by_untimed_samples() {
        let input = route_input();
        let route = PreparedNavigationRoute::new(&input).unwrap();
        let options = RouteNavigationProgressOptions {
            off_route_threshold_meters: 10.0,
            ..RouteNavigationProgressOptions::default()
        };
        let off_route = NavigationPositionSample {
            position: GeoPoint::new(50.001, 20.0005),
            heading_degrees: None,
            speed_meters_per_second: None,
            sampled_at_ms: None,
            accuracy_meters: None,
            provider: None,
        };
        let mut previous = None::<RouteNavigationProgress>;
        let mut statuses = Vec::new();
        for _ in 0..3 {
            let progress = route
                .progress(off_route.clone(), previous.as_ref(), &options)
                .unwrap();
            statuses.push(progress.status);
            previous = Some(progress);
        }
        assert_eq!(
            statuses,
            vec![
                RouteNavigationStatus::OffRouteCandidate,
                RouteNavigationStatus::OffRouteCandidate,
                RouteNavigationStatus::OffRoute,
            ]
        );
        assert!(previous.unwrap().reroute_suggested);
    }

    #[test]
    fn remaining_duration_sums_per_segment_durations() {
        let mut input = route_input();
        input.shape_segments = vec![
            RouteShapeSegment {
                edge_id: 7,
                begin_shape_index: 0,
                end_shape_index: 1,
                start_fraction: 0.0,
                end_fraction: 1.0,
                quality_flags: Vec::new(),
                dynamic_overlay: None,
            },
            RouteShapeSegment {
                edge_id: 8,
                begin_shape_index: 1,
                end_shape_index: 2,
                start_fraction: 0.0,
                end_fraction: 1.0,
                quality_flags: Vec::new(),
                dynamic_overlay: None,
            },
        ];
        // A fast first edge and a slow second one, of equal length.
        input.shape_segment_durations_seconds = vec![10.0, 110.0];
        input.shape_segment_speed_limits_kph = vec![Some(50.0), None];
        let progress = evaluate_route_navigation_progress(
            &input,
            NavigationPositionSample {
                position: point(20.0011),
                heading_degrees: None,
                speed_meters_per_second: None,
                sampled_at_ms: None,
                accuracy_meters: None,
                provider: None,
            },
            None,
            RouteNavigationProgressOptions::default(),
        )
        .unwrap();

        // 90% of the slow edge is left; in proportion to distance it would be
        // 45% of 120 s.
        assert!((progress.duration_remaining_seconds - 99.0).abs() < 0.5);
        assert_eq!(progress.active_edge_id, Some(8));
        // The second edge's limit is unknown.
        assert_eq!(progress.speed_limit_kph, None);
        input.shape_segment_speed_limits_kph = vec![None, Some(30.0)];
        let progress = evaluate_route_navigation_progress(
            &input,
            NavigationPositionSample {
                position: point(20.0011),
                heading_degrees: None,
                speed_meters_per_second: None,
                sampled_at_ms: None,
                accuracy_meters: None,
                provider: None,
            },
            None,
            RouteNavigationProgressOptions::default(),
        )
        .unwrap();
        assert_eq!(progress.speed_limit_kph, Some(30.0));
    }
}

use std::collections::HashMap;

use wayfarer_package::{RoadType, SharedEdgeInfo};

use crate::graph_view::PackageGraphView;
use crate::package::request::ReachCounter;
use crate::types::{
    CorrelatedLocation, CorrelationMode, CorrelationPolicy, CorrelationRejectReason,
    CorrelationScoreBreakdown, CorrelationSide, GeoPoint, MobilityProfile, PreferredRoadSide,
    RejectedCorrelationCandidate, RouteEndpointOptions, RouteError,
};

const MAX_CONFIDENCE: f64 = 1.0;
const MIN_CONFIDENCE: f64 = 0.0;
const DEFAULT_REACH_EDGE_LIMIT: usize = 64;
/// The radii `search_expanding` widens to, in turn, past the policy's.
pub const EXPANDED_CORRELATION_DISTANCES_M: [u32; 2] = [300, 1_000];
/// The smallest component (nodes) `search_expanding` takes for connected to
/// the network; smaller ones are islands.
pub const EXPANDED_CORRELATION_MIN_COMPONENT_SIZE: usize = 50;

pub struct Correlator<'a> {
    access: &'a PackageGraphView,
}

impl<'a> Correlator<'a> {
    pub fn new(access: &'a PackageGraphView) -> Self {
        Self { access }
    }

    pub fn candidates(
        &self,
        point: GeoPoint,
        profile: MobilityProfile,
        policy: &CorrelationPolicy,
    ) -> Result<Vec<CorrelatedLocation>, RouteError> {
        Ok(self
            .search(point, profile, policy, &RouteEndpointOptions::default())
            .candidates)
    }

    pub fn search(
        &self,
        point: GeoPoint,
        profile: MobilityProfile,
        policy: &CorrelationPolicy,
        endpoint_options: &RouteEndpointOptions,
    ) -> CandidateSearch {
        let mut rejected_candidate_count = 0usize;
        let mut rejected_candidates = Vec::new();
        // Candidate edges come from the routing package's request sections:
        // no record is built before the cheap checks pass.
        let request_view = self.access.request();
        let sar = policy.sar_open.map(crate::sar::SarOpenEdges::relaxations);
        let edges = request_view.correlation_edges_sar(point, policy.max_distance_m, profile, sar);
        let mut reach = ReachCounter::new(
            self.access.routing_package().file(),
            profile,
            DEFAULT_REACH_EDGE_LIMIT,
        )
        .with_sar(sar);
        let mut candidates = edges
            .into_iter()
            .filter_map(|edge| {
                // The first checks (access, snap distance) read only a few
                // fields; the full record is built after.
                let (edge_id, accessible, component_size, layer, level) = (
                    edge.edge_id,
                    edge.accessible,
                    edge.component_size,
                    edge.layer,
                    edge.level.clone(),
                );
                if !accessible {
                    rejected_candidate_count += 1;
                    rejected_candidates.push(rejected_candidate(
                        Some(edge_id),
                        CorrelationRejectReason::InaccessibleProfile,
                        None,
                        None,
                        component_size,
                        None,
                        None,
                        layer,
                        level.as_deref().map(str::to_string),
                    ));
                    return None;
                }

                let Some(snap) = snap_to_geometry(point, &edge.geometry) else {
                    rejected_candidate_count += 1;
                    rejected_candidates.push(rejected_candidate(
                        Some(edge_id),
                        CorrelationRejectReason::SnapUnavailable,
                        None,
                        None,
                        component_size,
                        None,
                        None,
                        layer,
                        level.as_deref().map(str::to_string),
                    ));
                    return None;
                };
                if snap.distance_m > f64::from(policy.max_distance_m) {
                    rejected_candidate_count += 1;
                    rejected_candidates.push(rejected_candidate(
                        Some(edge_id),
                        CorrelationRejectReason::BeyondMaxDistance,
                        Some(snap.distance_m),
                        None,
                        component_size,
                        None,
                        None,
                        layer,
                        level.as_deref().map(str::to_string),
                    ));
                    return None;
                }
                let (record, opposing_edge_id) = request_view.correlation_record(edge, profile)?;

                if endpoint_options
                    .min_component_size
                    .is_some_and(|minimum| component_size.unwrap_or_default() < minimum)
                {
                    rejected_candidate_count += 1;
                    rejected_candidates.push(rejected_candidate(
                        Some(record.edge.edge_id),
                        CorrelationRejectReason::BelowMinComponentSize,
                        Some(snap.distance_m),
                        None,
                        component_size,
                        None,
                        None,
                        record.shared_info.layer,
                        record.shared_info.level.as_deref().map(str::to_string),
                    ));
                    return None;
                }

                // The edge's direction where the point snaps, not where it
                // starts: the two differ on any curved edge.
                let candidate_heading_deg = snap.bearing_deg.or(record.edge.start_heading_deg);
                let heading_delta_deg = endpoint_options.heading_deg.and_then(|heading| {
                    candidate_heading_deg.map(|candidate| heading_delta(heading, candidate))
                });
                if endpoint_options
                    .heading_tolerance_deg
                    .zip(heading_delta_deg)
                    .is_some_and(|(tolerance, delta)| delta > tolerance.min(180))
                {
                    rejected_candidate_count += 1;
                    rejected_candidates.push(rejected_candidate(
                        Some(record.edge.edge_id),
                        CorrelationRejectReason::HeadingOutsideTolerance,
                        Some(snap.distance_m),
                        heading_delta_deg,
                        component_size,
                        None,
                        None,
                        record.shared_info.layer,
                        record.shared_info.level.as_deref().map(str::to_string),
                    ));
                    return None;
                }

                if endpoint_options.require_layer_match
                    && endpoint_options
                        .preferred_layer
                        .is_some_and(|layer| record.shared_info.layer != Some(layer))
                {
                    rejected_candidate_count += 1;
                    rejected_candidates.push(rejected_candidate(
                        Some(record.edge.edge_id),
                        CorrelationRejectReason::LayerMismatch,
                        Some(snap.distance_m),
                        heading_delta_deg,
                        component_size,
                        None,
                        None,
                        record.shared_info.layer,
                        record.shared_info.level.as_deref().map(str::to_string),
                    ));
                    return None;
                }

                if endpoint_options.require_level_match
                    && endpoint_options
                        .preferred_level
                        .as_ref()
                        .is_some_and(|level| {
                            record
                                .shared_info
                                .level
                                .as_deref()
                                .is_none_or(|candidate| !level_matches(level, candidate))
                        })
                {
                    rejected_candidate_count += 1;
                    rejected_candidates.push(rejected_candidate(
                        Some(record.edge.edge_id),
                        CorrelationRejectReason::LevelMismatch,
                        Some(snap.distance_m),
                        heading_delta_deg,
                        component_size,
                        None,
                        None,
                        record.shared_info.layer,
                        record.shared_info.level.as_deref().map(str::to_string),
                    ));
                    return None;
                }

                // Reach BFS (up to 2x64 edges) runs only for candidates that
                // survived the cheap distance/component/heading/layer/level
                // rejections; cheap-rejected diagnostics report reach as None.
                // The routing package answers the same reach question without
                // materializing edge records (min(reachable, cap) is order-free).
                let package_reach = reach
                    .count(record.raw_edge_index, true)
                    .and_then(|outbound| {
                        Some((outbound, reach.count(record.raw_edge_index, false)?))
                    });
                let (outbound_reach_edge_count, inbound_reach_edge_count) = match package_reach {
                    Some((outbound, inbound)) => (Some(outbound), Some(inbound)),
                    None => (
                        self.access.outbound_reach_edge_count(
                            record.target.id,
                            profile,
                            DEFAULT_REACH_EDGE_LIMIT,
                        ),
                        self.access.inbound_reach_edge_count(
                            record.source.id,
                            profile,
                            DEFAULT_REACH_EDGE_LIMIT,
                        ),
                    ),
                };
                if endpoint_options
                    .min_outbound_reach_edge_count
                    .is_some_and(|minimum| outbound_reach_edge_count.unwrap_or_default() < minimum)
                    || endpoint_options
                        .min_inbound_reach_edge_count
                        .is_some_and(|minimum| {
                            inbound_reach_edge_count.unwrap_or_default() < minimum
                        })
                {
                    rejected_candidate_count += 1;
                    rejected_candidates.push(rejected_candidate(
                        Some(record.edge.edge_id),
                        CorrelationRejectReason::BelowMinReach,
                        Some(snap.distance_m),
                        heading_delta_deg,
                        component_size,
                        outbound_reach_edge_count,
                        inbound_reach_edge_count,
                        record.shared_info.layer,
                        record.shared_info.level.as_deref().map(str::to_string),
                    ));
                    return None;
                }

                let road_group_key = road_group_key(&record.shared_info);
                let confidence = confidence(snap.distance_m, f64::from(policy.max_distance_m));
                let score_breakdown = score_candidate_breakdown(
                    snap.distance_m,
                    f64::from(policy.max_distance_m),
                    heading_delta_deg,
                    snap.side,
                    endpoint_options.preferred_side,
                    component_size,
                    &record.shared_info,
                    profile,
                    outbound_reach_edge_count,
                    inbound_reach_edge_count,
                    endpoint_options.preferred_layer,
                    record.shared_info.layer,
                    endpoint_options.preferred_level.as_deref(),
                    record.shared_info.level.as_deref(),
                    endpoint_options,
                );
                let score = score_breakdown.total_score;
                Some(CorrelatedLocation {
                    edge_id: record.edge.edge_id,
                    compact_edge: crate::types::CompactEdgeHint::of(record.raw_edge_index),
                    shared_edge_info_id: record.edge.shared_edge_info_id,
                    opposing_edge_id,
                    road_group_key,
                    same_road_candidate_count: 0,
                    source_node_id: record.source.id,
                    target_node_id: record.target.id,
                    snapped_point: snap.point,
                    distance_m: snap.distance_m,
                    fraction_along_edge: snap.fraction,
                    heading_deg: candidate_heading_deg,
                    heading_delta_deg,
                    side: snap.side,
                    component_size,
                    outbound_reach_edge_count,
                    inbound_reach_edge_count,
                    layer: record.shared_info.layer,
                    level: record.shared_info.level.as_deref().map(str::to_string),
                    score,
                    score_breakdown,
                    confidence,
                })
            })
            .collect::<Vec<_>>();

        attach_same_road_candidate_counts(&mut candidates);

        // Both orders end on the (distinct) edge id, so they are total and an
        // unstable sort of indexes gives the same list without moving the
        // large candidate records around.
        let hinted = endpoint_options.has_scoring_hints();
        let compare = |a: &CorrelatedLocation, b: &CorrelatedLocation| {
            if hinted {
                b.score
                    .total_cmp(&a.score)
                    .then_with(|| a.distance_m.total_cmp(&b.distance_m))
                    .then_with(|| a.edge_id.cmp(&b.edge_id))
            } else {
                default_distance_bucket(a.distance_m)
                    .cmp(&default_distance_bucket(b.distance_m))
                    .then_with(|| b.score.total_cmp(&a.score))
                    .then_with(|| a.distance_m.total_cmp(&b.distance_m))
                    .then_with(|| b.confidence.total_cmp(&a.confidence))
                    .then_with(|| a.edge_id.cmp(&b.edge_id))
            }
        };
        let mut order = (0..candidates.len()).collect::<Vec<_>>();
        order.sort_unstable_by(|a, b| compare(&candidates[*a], &candidates[*b]));
        order.truncate(policy.candidate_limit);
        let mut slots = candidates.into_iter().map(Some).collect::<Vec<_>>();
        let candidates = order
            .into_iter()
            .filter_map(|index| slots[index].take())
            .collect::<Vec<_>>();

        CandidateSearch {
            candidates,
            rejected_candidate_count,
            rejected_candidates,
            mode: CorrelationMode::Indexed,
            expanded_max_distance_m: None,
        }
    }

    /// [`Self::search`], widening the radius in turn to each of
    /// [`EXPANDED_CORRELATION_DISTANCES_M`] beyond the policy's while no
    /// candidate is usable or connected: when every candidate was rejected or
    /// lies on an island (a component under
    /// [`EXPANDED_CORRELATION_MIN_COMPONENT_SIZE`] nodes, or a reach under
    /// the reach cap either way). A wider search keeps only connected
    /// candidates, after up to half a candidate limit of the nearer island ones
    /// (an endpoint on the same island still needs those), and records its
    /// radius.
    pub fn search_expanding(
        &self,
        point: GeoPoint,
        profile: MobilityProfile,
        policy: &CorrelationPolicy,
        endpoint_options: &RouteEndpointOptions,
    ) -> CandidateSearch {
        let nearby = self.search(point, profile, policy, endpoint_options);
        // Components count every mode's edges, so the profile's reach must
        // saturate too: a square open to cars but ringed by private streets
        // lies in the city's component and is still an island for cars.
        let connected = |candidate: &CorrelatedLocation| {
            candidate
                .component_size
                .is_some_and(|size| size >= EXPANDED_CORRELATION_MIN_COMPONENT_SIZE)
                && [
                    candidate.outbound_reach_edge_count,
                    candidate.inbound_reach_edge_count,
                ]
                .iter()
                .all(|reach| reach.is_some_and(|reach| reach >= DEFAULT_REACH_EDGE_LIMIT))
        };
        if nearby.candidates.iter().any(connected) {
            return nearby;
        }
        let mut expanded_options = endpoint_options.clone();
        expanded_options.min_component_size = Some(
            endpoint_options
                .min_component_size
                .unwrap_or_default()
                .max(EXPANDED_CORRELATION_MIN_COMPONENT_SIZE),
        );
        expanded_options.min_outbound_reach_edge_count = Some(
            endpoint_options
                .min_outbound_reach_edge_count
                .unwrap_or_default()
                .max(DEFAULT_REACH_EDGE_LIMIT),
        );
        expanded_options.min_inbound_reach_edge_count = Some(
            endpoint_options
                .min_inbound_reach_edge_count
                .unwrap_or_default()
                .max(DEFAULT_REACH_EDGE_LIMIT),
        );
        // The SAR `off_network` reach extends the radii beyond the last one.
        let off_network = policy
            .off_network_max_distance_m
            .filter(|distance| *distance > EXPANDED_CORRELATION_DISTANCES_M[1]);
        for max_distance_m in EXPANDED_CORRELATION_DISTANCES_M
            .into_iter()
            .chain(off_network)
        {
            if max_distance_m <= policy.max_distance_m {
                continue;
            }
            let wider = CorrelationPolicy {
                max_distance_m,
                ..policy.clone()
            };
            let mut expanded = self.search(point, profile, &wider, &expanded_options);
            if expanded.candidates.is_empty() {
                continue;
            }
            // At most half the slots for the nearer island candidates, so the
            // connected ones are always seeded.
            let mut candidates = nearby.candidates;
            candidates.truncate(policy.candidate_limit / 2);
            for candidate in expanded.candidates {
                if !candidates
                    .iter()
                    .any(|nearer| nearer.edge_id == candidate.edge_id)
                {
                    candidates.push(candidate);
                }
            }
            candidates.truncate(policy.candidate_limit.max(1));
            expanded.candidates = candidates;
            expanded.rejected_candidate_count += nearby.rejected_candidate_count;
            expanded
                .rejected_candidates
                .splice(0..0, nearby.rejected_candidates);
            expanded.expanded_max_distance_m = Some(max_distance_m);
            return expanded;
        }
        nearby
    }
}

#[derive(Debug, Clone)]
pub struct CandidateSearch {
    pub candidates: Vec<CorrelatedLocation>,
    pub rejected_candidate_count: usize,
    pub rejected_candidates: Vec<RejectedCorrelationCandidate>,
    pub mode: CorrelationMode,
    /// The radius `Correlator::search_expanding` widened to, if it did.
    pub expanded_max_distance_m: Option<u32>,
}

fn attach_same_road_candidate_counts(candidates: &mut [CorrelatedLocation]) {
    let counts = candidates
        .iter()
        .filter_map(|candidate| candidate.road_group_key.as_deref())
        .fold(HashMap::<String, usize>::new(), |mut counts, key| {
            *counts.entry(key.to_string()).or_default() += 1;
            counts
        });

    for candidate in candidates {
        candidate.same_road_candidate_count = candidate
            .road_group_key
            .as_ref()
            .and_then(|key| counts.get(key))
            .copied()
            .unwrap_or(1);
    }
}

fn road_group_key(info: &SharedEdgeInfo) -> Option<String> {
    normalized_label(info.ref_name.as_deref())
        .map(|label| format!("ref:{label}"))
        .or_else(|| {
            info.route_refs
                .iter()
                .find_map(|route_ref| normalized_label(Some(route_ref)))
                .map(|label| format!("ref:{label}"))
        })
        .or_else(|| normalized_label(info.name.as_deref()).map(|label| format!("name:{label}")))
        .or_else(|| Some(format!("way:{}", info.osm_way_id)))
}

fn normalized_label(value: Option<&str>) -> Option<String> {
    let value = value?;
    // Most labels are already normalized: single spaces between words, none
    // at the ends. Copy those; collapse whitespace otherwise.
    let mut previous_space = true;
    let normalized_already = value.chars().all(|character| {
        let ok = if character.is_whitespace() {
            character == ' ' && !previous_space
        } else {
            true
        };
        previous_space = character.is_whitespace();
        ok
    }) && !previous_space;
    let normalized = if normalized_already {
        value.to_string()
    } else {
        value.split_whitespace().collect::<Vec<_>>().join(" ")
    };
    (!normalized.is_empty()).then_some(normalized)
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct Snap {
    pub(crate) point: GeoPoint,
    pub(crate) distance_m: f64,
    pub(crate) fraction: f64,
    side: CorrelationSide,
    /// The bearing of the geometry's segment at the snapped point.
    pub(crate) bearing_deg: Option<u16>,
}

/// The distance from `point` to `geometry` as correlation measures it.
#[cfg(test)]
pub(crate) fn snap_distance_m(point: GeoPoint, geometry: &[GeoPoint]) -> Option<f64> {
    snap_to_geometry(point, geometry).map(|snap| snap.distance_m)
}

pub(crate) fn snap_to_geometry(point: GeoPoint, geometry: &[GeoPoint]) -> Option<Snap> {
    if geometry.len() < 2 {
        return None;
    }
    let origin_lat = point.lat.to_radians();
    let meters_per_lat = 111_320.0;
    let meters_per_lng = 111_320.0 * origin_lat.cos().abs().max(0.01);
    let to_xy = |p: GeoPoint| -> (f64, f64) {
        (
            (p.lng - point.lng) * meters_per_lng,
            (p.lat - point.lat) * meters_per_lat,
        )
    };

    // Two passes over the windows (the same arithmetic both times) instead of
    // a buffer of segments: this runs for every candidate edge.
    let segments = || {
        geometry.windows(2).map(|segment| {
            let a = segment[0];
            let b = segment[1];
            let (ax, ay) = to_xy(a);
            let (bx, by) = to_xy(b);
            let vx = bx - ax;
            let vy = by - ay;
            let length = (vx * vx + vy * vy).sqrt();
            (a, b, ax, ay, vx, vy, length)
        })
    };
    let mut total_len = 0.0;
    for (.., length) in segments() {
        total_len += length;
    }
    if total_len <= f64::EPSILON {
        return None;
    }

    let mut best = None;
    let mut distance_before = 0.0;
    for (a, b, ax, ay, vx, vy, length) in segments() {
        if length <= f64::EPSILON {
            continue;
        }
        let t = (-(ax * vx + ay * vy) / (length * length)).clamp(0.0, 1.0);
        let x = ax + vx * t;
        let y = ay + vy * t;
        let distance_m = (x * x + y * y).sqrt();
        let snapped = GeoPoint::new(a.lat + (b.lat - a.lat) * t, a.lng + (b.lng - a.lng) * t);
        let fraction = ((distance_before + length * t) / total_len).clamp(0.0, 1.0);
        let snap = Snap {
            point: snapped,
            distance_m,
            fraction,
            side: side_of_segment(ax, ay, vx, vy),
            bearing_deg: Some(vx.atan2(vy).to_degrees().rem_euclid(360.0).round() as u16 % 360),
        };
        if best
            .as_ref()
            .is_none_or(|current: &Snap| snap.distance_m < current.distance_m)
        {
            best = Some(snap);
        }
        distance_before += length;
    }
    best
}

fn confidence(distance_m: f64, max_distance_m: f64) -> f64 {
    if max_distance_m <= f64::EPSILON {
        return MIN_CONFIDENCE;
    }
    (MAX_CONFIDENCE - distance_m / max_distance_m).clamp(MIN_CONFIDENCE, MAX_CONFIDENCE)
}

fn rejected_candidate(
    edge_id: Option<u64>,
    reason: CorrelationRejectReason,
    distance_m: Option<f64>,
    heading_delta_deg: Option<u16>,
    component_size: Option<usize>,
    outbound_reach_edge_count: Option<usize>,
    inbound_reach_edge_count: Option<usize>,
    layer: Option<i16>,
    level: Option<String>,
) -> RejectedCorrelationCandidate {
    RejectedCorrelationCandidate {
        edge_id,
        reason,
        distance_millimeters: distance_m
            .map(|distance| (distance * 1000.0).round().max(0.0) as u64),
        heading_delta_deg,
        component_size,
        outbound_reach_edge_count,
        inbound_reach_edge_count,
        layer,
        level,
    }
}

fn side_of_segment(ax: f64, ay: f64, vx: f64, vy: f64) -> CorrelationSide {
    let cross = vx * -ay - vy * -ax;
    if cross.abs() < 0.01 {
        CorrelationSide::OnEdge
    } else if cross > 0.0 {
        CorrelationSide::Left
    } else {
        CorrelationSide::Right
    }
}

fn heading_delta(a: u16, b: u16) -> u16 {
    let a = i32::from(a % 360);
    let b = i32::from(b % 360);
    let diff = (a - b).abs();
    diff.min(360 - diff) as u16
}

fn score_candidate_breakdown(
    distance_m: f64,
    max_distance_m: f64,
    heading_delta_deg: Option<u16>,
    side: CorrelationSide,
    preferred_side: PreferredRoadSide,
    component_size: Option<usize>,
    shared_info: &SharedEdgeInfo,
    profile: MobilityProfile,
    outbound_reach_edge_count: Option<usize>,
    inbound_reach_edge_count: Option<usize>,
    preferred_layer: Option<i16>,
    candidate_layer: Option<i16>,
    preferred_level: Option<&str>,
    candidate_level: Option<&str>,
    endpoint_options: &RouteEndpointOptions,
) -> CorrelationScoreBreakdown {
    let distance_score = confidence(distance_m, max_distance_m) * 0.31;
    let heading_score = heading_delta_deg
        .map(|delta| (1.0 - f64::from(delta.min(180)) / 180.0).clamp(0.0, 1.0))
        .unwrap_or(0.5)
        * 0.16;
    let road_score = profile_road_score(profile, shared_info.road_type) * 0.13;
    let access_score = access_preference_score(shared_info) * 0.10;
    let component_score = component_size
        .map(|size| ((size as f64).ln_1p() / 8.0).clamp(0.0, 1.0))
        .unwrap_or(0.0)
        * 0.07;
    let reach_score = outbound_reach_edge_count
        .into_iter()
        .chain(inbound_reach_edge_count)
        .max()
        .map(|count| ((count as f64).ln_1p() / 5.0).clamp(0.0, 1.0))
        .unwrap_or(0.0)
        * 0.10;
    let side_score = match preferred_side {
        PreferredRoadSide::Either => 0.5,
        PreferredRoadSide::Left => {
            if side == CorrelationSide::Left || side == CorrelationSide::OnEdge {
                1.0
            } else {
                0.0
            }
        }
        PreferredRoadSide::Right => {
            if side == CorrelationSide::Right || side == CorrelationSide::OnEdge {
                1.0
            } else {
                0.0
            }
        }
    } * 0.05;
    let layer_score = layer_match_score(preferred_layer, candidate_layer) * 0.07;
    let level_score = level_match_score(preferred_level, candidate_level) * 0.05;
    let candidate_type_score =
        (candidate_type_preference_score(shared_info, endpoint_options) - 0.5) * 0.20;
    let total_score = distance_score
        + heading_score
        + road_score
        + access_score
        + component_score
        + reach_score
        + side_score
        + layer_score
        + level_score
        + candidate_type_score;
    CorrelationScoreBreakdown {
        distance_score,
        heading_score,
        road_class_score: road_score,
        access_score,
        component_score,
        reach_score,
        side_score,
        layer_score,
        level_score,
        candidate_type_score,
        total_score,
    }
}

fn default_distance_bucket(distance_m: f64) -> u64 {
    (distance_m.max(0.0) / 2.0).floor() as u64
}

fn access_preference_score(info: &SharedEdgeInfo) -> f64 {
    let mut score: f64 = 1.0;
    if info.is_service_like {
        score -= 0.20;
    }
    if info.is_destination_only {
        score -= 0.20;
    }
    if info.has_gate {
        score -= 0.25;
    }
    if info.is_access_restricted {
        score -= 0.30;
    }
    if info.is_ferry {
        score -= 0.15;
    }
    score.clamp(0.0, 1.0)
}

fn candidate_type_preference_score(
    info: &SharedEdgeInfo,
    endpoint_options: &RouteEndpointOptions,
) -> f64 {
    let has_hints = !endpoint_options.preferred_road_classes.is_empty()
        || !endpoint_options.avoided_road_classes.is_empty()
        || !endpoint_options.preferred_route_refs.is_empty()
        || endpoint_options.prefer_any_route_ref
        || endpoint_options.avoid_service_roads
        || endpoint_options.avoid_private_access;
    if !has_hints {
        return 0.5;
    }

    let mut score: f64 = 0.5;
    if !endpoint_options.preferred_road_classes.is_empty() {
        if endpoint_options
            .preferred_road_classes
            .iter()
            .any(|preference| preference.matches(info.road_type))
        {
            score += 0.25;
        } else {
            score -= 0.15;
        }
    }
    if endpoint_options
        .avoided_road_classes
        .iter()
        .any(|preference| preference.matches(info.road_type))
    {
        score -= 0.25;
    }

    let has_known_route_ref = info
        .ref_name
        .as_ref()
        .is_some_and(|value| !value.trim().is_empty())
        || info.route_refs.iter().any(|value| !value.trim().is_empty());
    if endpoint_options.prefer_any_route_ref {
        if has_known_route_ref {
            score += 0.20;
        } else {
            score -= 0.05;
        }
    }
    if !endpoint_options.preferred_route_refs.is_empty() {
        if preferred_route_ref_matches(info, &endpoint_options.preferred_route_refs) {
            score += 0.25;
        } else {
            score -= 0.10;
        }
    }

    if endpoint_options.avoid_service_roads && info.is_service_like {
        score -= 0.25;
    }
    if endpoint_options.avoid_private_access && private_like_access(info) {
        score -= 0.25;
    }

    score.clamp(0.0, 1.0)
}

fn preferred_route_ref_matches(info: &SharedEdgeInfo, preferred_route_refs: &[String]) -> bool {
    let preferred = preferred_route_refs
        .iter()
        .filter_map(|route_ref| normalized_label(Some(route_ref)))
        .collect::<Vec<_>>();
    if preferred.is_empty() {
        return false;
    }

    normalized_label(info.ref_name.as_deref())
        .into_iter()
        .chain(
            info.route_refs
                .iter()
                .filter_map(|route_ref| normalized_label(Some(route_ref))),
        )
        .any(|candidate| {
            preferred
                .iter()
                .any(|requested| candidate.eq_ignore_ascii_case(requested))
        })
}

fn private_like_access(info: &SharedEdgeInfo) -> bool {
    info.is_access_restricted || info.has_gate || info.is_destination_only
}

fn layer_match_score(preferred_layer: Option<i16>, candidate_layer: Option<i16>) -> f64 {
    let Some(preferred_layer) = preferred_layer else {
        return 0.5;
    };
    let Some(candidate_layer) = candidate_layer else {
        return 0.2;
    };
    let delta = preferred_layer.abs_diff(candidate_layer);
    if delta == 0 {
        1.0
    } else if delta == 1 {
        0.55
    } else {
        0.0
    }
}

fn level_match_score(preferred_level: Option<&str>, candidate_level: Option<&str>) -> f64 {
    let Some(preferred_level) = preferred_level else {
        return 0.5;
    };
    let Some(candidate_level) = candidate_level else {
        return 0.2;
    };
    if level_matches(preferred_level, candidate_level) {
        1.0
    } else {
        0.0
    }
}

fn level_matches(preferred_level: &str, candidate_level: &str) -> bool {
    preferred_level
        .trim()
        .eq_ignore_ascii_case(candidate_level.trim())
}

fn profile_road_score(profile: MobilityProfile, road_type: RoadType) -> f64 {
    match profile {
        MobilityProfile::Default | MobilityProfile::Wheeled | MobilityProfile::Emergency => {
            match road_type {
                RoadType::Motorway | RoadType::Primary | RoadType::Secondary => 1.0,
                RoadType::Tertiary | RoadType::Residential => 0.8,
                RoadType::Track => 0.45,
                RoadType::Path => 0.15,
                RoadType::Other => 0.5,
            }
        }
        MobilityProfile::Tracked => match road_type {
            RoadType::Track => 1.0,
            RoadType::Residential | RoadType::Tertiary | RoadType::Other => 0.75,
            RoadType::Primary | RoadType::Secondary => 0.6,
            RoadType::Motorway => 0.35,
            RoadType::Path => 0.3,
        },
        MobilityProfile::OnFoot | MobilityProfile::Legged => match road_type {
            RoadType::Path | RoadType::Track => 1.0,
            RoadType::Residential | RoadType::Other => 0.7,
            RoadType::Tertiary => 0.45,
            RoadType::Primary | RoadType::Secondary => 0.25,
            RoadType::Motorway => 0.05,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::CorrelationRoadClassPreference;
    use wayfarer_package::DirectionalAccess;
    use wayfarer_road_graph::{RoadGraph, RoadGraphBuilder, RoutableWay, enhance_road_graph};

    fn correlation_test_graph() -> RoadGraph {
        let mut builder = RoadGraphBuilder::new("correlation-test");
        builder.push_way(RoutableWay {
            osm_way_id: 1,
            points: vec![(50.0, 20.0), (50.0, 20.002)],
            source_node_ids: Vec::new(),
            road_type: RoadType::Primary,
            directional_access: DirectionalAccess::Both,
            name: Some("Opolska".into()),
            ref_name: None,
            highway: None,
            access: None,
            vehicle: None,
            motor_vehicle: None,
            motorcar: None,
            bicycle: None,
            foot: None,
            oneway_foot: None,
            oneway_bicycle: None,
            motorroad: None,
            service: None,
            surface: None,
            tracktype: None,
            smoothness: None,
            maxspeed: None,
            lanes: None,
            has_bridge: false,
            has_tunnel: false,
            has_ford: false,
            is_roundabout: false,
            is_link: false,
            is_destination_only: false,
            access_restricted: false,
            area: None,
            area_highway: None,
            layer: None,
            level: None,
            destination: None,
            destination_ref: None,
            destination_street: None,
            junction_ref: None,
            is_toll: false,
            is_ferry: false,
            has_gate: false,
            is_access_restricted: false,
            conditional_restrictions: Vec::new(),
            ..Default::default()
        });
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);
        graph
    }

    #[test]
    fn heading_tolerance_rejects_wrong_direction_candidates() {
        let graph = correlation_test_graph();
        let engine = crate::test_support::engine(&graph);
        let correlator = Correlator::new(engine.route_graph_view());
        let search = correlator.search(
            GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
            &CorrelationPolicy::default(),
            &RouteEndpointOptions::default()
                .with_heading(90)
                .with_heading_tolerance(35),
        );

        assert!(!search.candidates.is_empty());
        assert!(
            search
                .candidates
                .iter()
                .all(|candidate| candidate.heading_delta_deg.is_some_and(|delta| delta <= 35)),
            "all accepted candidates should match the requested heading"
        );
        let best = search.candidates.first().expect("accepted candidate");
        assert_eq!(best.score, best.score_breakdown.total_score);
        assert!(
            best.score_breakdown.heading_score > 0.0,
            "accepted candidate should expose heading contribution"
        );
        assert!(search.rejected_candidates.iter().any(|candidate| {
            candidate.reason == CorrelationRejectReason::HeadingOutsideTolerance
                && candidate.heading_delta_deg.is_some_and(|delta| delta > 35)
        }));
    }

    #[test]
    fn preferred_side_changes_candidate_order_without_rejecting_candidates() {
        let graph = correlation_test_graph();
        let engine = crate::test_support::engine(&graph);
        let correlator = Correlator::new(engine.route_graph_view());
        let point = GeoPoint::new(50.00005, 20.001);
        let policy = CorrelationPolicy::default();

        let left_search = correlator.search(
            point,
            MobilityProfile::Wheeled,
            &policy,
            &RouteEndpointOptions::default().with_preferred_side(PreferredRoadSide::Left),
        );
        let right_search = correlator.search(
            point,
            MobilityProfile::Wheeled,
            &policy,
            &RouteEndpointOptions::default().with_preferred_side(PreferredRoadSide::Right),
        );

        assert_eq!(left_search.rejected_candidate_count, 0);
        assert_eq!(right_search.rejected_candidate_count, 0);
        assert_eq!(
            left_search
                .candidates
                .first()
                .map(|candidate| candidate.side),
            Some(CorrelationSide::Left)
        );
        assert_eq!(
            right_search
                .candidates
                .first()
                .map(|candidate| candidate.side),
            Some(CorrelationSide::Right)
        );
        assert_ne!(
            left_search
                .candidates
                .first()
                .map(|candidate| candidate.edge_id),
            right_search
                .candidates
                .first()
                .map(|candidate| candidate.edge_id)
        );
        assert!(left_search.candidates[0].score_breakdown.side_score > 0.0);
        assert!(right_search.candidates[0].score_breakdown.side_score > 0.0);
    }

    #[test]
    fn default_correlation_prefers_unrestricted_road_in_near_distance_tie() {
        let mut restricted_track = RoutableWay {
            osm_way_id: 1,
            points: vec![(50.0, 20.0), (50.0, 20.002)],
            source_node_ids: Vec::new(),
            road_type: RoadType::Track,
            directional_access: DirectionalAccess::Both,
            name: Some("Restricted track".into()),
            ref_name: None,
            highway: None,
            access: Some("yes".into()),
            vehicle: None,
            motor_vehicle: None,
            motorcar: None,
            bicycle: None,
            foot: None,
            oneway_foot: None,
            oneway_bicycle: None,
            motorroad: None,
            service: Some("driveway".into()),
            surface: None,
            tracktype: None,
            smoothness: None,
            maxspeed: None,
            lanes: None,
            has_bridge: false,
            has_tunnel: false,
            has_ford: false,
            is_roundabout: false,
            is_link: false,
            is_destination_only: true,
            access_restricted: false,
            area: None,
            area_highway: None,
            layer: None,
            level: None,
            destination: None,
            destination_ref: None,
            destination_street: None,
            junction_ref: None,
            is_toll: false,
            is_ferry: false,
            has_gate: false,
            is_access_restricted: false,
            conditional_restrictions: Vec::new(),
            ..Default::default()
        };
        restricted_track.tracktype = Some("grade3".into());
        let public_primary = RoutableWay {
            osm_way_id: 2,
            points: vec![(50.000012, 20.0), (50.000012, 20.002)],
            road_type: RoadType::Primary,
            name: Some("Public road".into()),
            access: Some("yes".into()),
            service: None,
            tracktype: None,
            is_destination_only: false,
            access_restricted: false,
            has_gate: false,
            is_access_restricted: false,
            ..restricted_track.clone()
        };
        let mut builder = RoadGraphBuilder::new("near-tie-correlation");
        builder.push_way(restricted_track);
        builder.push_way(public_primary);
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);

        let engine = crate::test_support::engine(&graph);
        let candidates = Correlator::new(engine.route_graph_view())
            .candidates(
                GeoPoint::new(50.000003, 20.001),
                MobilityProfile::Wheeled,
                &CorrelationPolicy {
                    max_distance_m: 30,
                    candidate_limit: 4,
                    ..CorrelationPolicy::default()
                },
            )
            .expect("candidate search succeeds");

        assert_eq!(
            candidates
                .first()
                .and_then(|candidate| candidate.road_group_key.as_deref()),
            Some("name:Public road"),
            "public road should outrank a slightly closer restricted track"
        );
    }

    #[test]
    fn default_correlation_keeps_much_closer_restricted_candidate_first() {
        let mut restricted_track = RoutableWay {
            osm_way_id: 1,
            points: vec![(50.0, 20.0), (50.0, 20.002)],
            source_node_ids: Vec::new(),
            road_type: RoadType::Track,
            directional_access: DirectionalAccess::Both,
            name: Some("Restricted track".into()),
            ref_name: None,
            highway: None,
            access: Some("yes".into()),
            vehicle: None,
            motor_vehicle: None,
            motorcar: None,
            bicycle: None,
            foot: None,
            oneway_foot: None,
            oneway_bicycle: None,
            motorroad: None,
            service: Some("driveway".into()),
            surface: None,
            tracktype: None,
            smoothness: None,
            maxspeed: None,
            lanes: None,
            has_bridge: false,
            has_tunnel: false,
            has_ford: false,
            is_roundabout: false,
            is_link: false,
            is_destination_only: true,
            access_restricted: false,
            area: None,
            area_highway: None,
            layer: None,
            level: None,
            destination: None,
            destination_ref: None,
            destination_street: None,
            junction_ref: None,
            is_toll: false,
            is_ferry: false,
            has_gate: false,
            is_access_restricted: false,
            conditional_restrictions: Vec::new(),
            ..Default::default()
        };
        restricted_track.tracktype = Some("grade3".into());
        let public_primary = RoutableWay {
            osm_way_id: 2,
            points: vec![(50.00005, 20.0), (50.00005, 20.002)],
            road_type: RoadType::Primary,
            name: Some("Public road".into()),
            access: Some("yes".into()),
            service: None,
            tracktype: None,
            is_destination_only: false,
            access_restricted: false,
            has_gate: false,
            is_access_restricted: false,
            ..restricted_track.clone()
        };
        let mut builder = RoadGraphBuilder::new("clear-distance-correlation");
        builder.push_way(restricted_track);
        builder.push_way(public_primary);
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);

        let engine = crate::test_support::engine(&graph);
        let candidates = Correlator::new(engine.route_graph_view())
            .candidates(
                GeoPoint::new(50.000001, 20.001),
                MobilityProfile::Wheeled,
                &CorrelationPolicy {
                    max_distance_m: 30,
                    candidate_limit: 4,
                    ..CorrelationPolicy::default()
                },
            )
            .expect("candidate search succeeds");

        assert_eq!(
            candidates
                .first()
                .and_then(|candidate| candidate.road_group_key.as_deref()),
            Some("name:Restricted track"),
            "large distance difference should still dominate access preference"
        );
    }

    #[test]
    fn candidate_type_preferences_steer_ambiguous_endpoint_to_signed_road() {
        let service_driveway = RoutableWay {
            osm_way_id: 1,
            points: vec![(50.0, 20.0), (50.0, 20.002)],
            source_node_ids: Vec::new(),
            road_type: RoadType::Residential,
            directional_access: DirectionalAccess::Both,
            name: Some("Depot driveway".into()),
            ref_name: None,
            highway: None,
            access: Some("yes".into()),
            vehicle: None,
            motor_vehicle: None,
            motorcar: None,
            bicycle: None,
            foot: None,
            oneway_foot: None,
            oneway_bicycle: None,
            motorroad: None,
            service: Some("driveway".into()),
            surface: None,
            tracktype: None,
            smoothness: None,
            maxspeed: None,
            lanes: None,
            has_bridge: false,
            has_tunnel: false,
            has_ford: false,
            is_roundabout: false,
            is_link: false,
            is_destination_only: false,
            access_restricted: false,
            area: None,
            area_highway: None,
            layer: None,
            level: None,
            destination: None,
            destination_ref: None,
            destination_street: None,
            junction_ref: None,
            is_toll: false,
            is_ferry: false,
            has_gate: false,
            is_access_restricted: false,
            conditional_restrictions: Vec::new(),
            ..Default::default()
        };
        let signed_primary = RoutableWay {
            osm_way_id: 2,
            points: vec![(50.00005, 20.0), (50.00005, 20.002)],
            road_type: RoadType::Primary,
            name: Some("Public road".into()),
            ref_name: Some("MSR-1".into()),
            service: None,
            ..service_driveway.clone()
        };
        let mut builder = RoadGraphBuilder::new("candidate-type-preference-correlation");
        builder.push_way(service_driveway);
        builder.push_way(signed_primary);
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);

        let engine = crate::test_support::engine(&graph);
        let correlator = Correlator::new(engine.route_graph_view());
        let point = GeoPoint::new(50.000001, 20.001);
        let policy = CorrelationPolicy {
            max_distance_m: 30,
            candidate_limit: 4,
            ..CorrelationPolicy::default()
        };

        let default_candidates = correlator
            .candidates(point, MobilityProfile::Wheeled, &policy)
            .expect("default candidate search succeeds");
        assert_eq!(
            default_candidates
                .first()
                .and_then(|candidate| candidate.road_group_key.as_deref()),
            Some("name:Depot driveway"),
            "without endpoint type hints, the much closer driveway should remain first"
        );

        let preferred_search = correlator.search(
            point,
            MobilityProfile::Wheeled,
            &policy,
            &RouteEndpointOptions::default()
                .with_preferred_road_class(CorrelationRoadClassPreference::Primary)
                .with_prefer_any_route_ref()
                .with_preferred_route_ref("MSR-1")
                .with_avoid_service_roads(),
        );

        assert_eq!(preferred_search.rejected_candidate_count, 0);
        assert_eq!(
            preferred_search
                .candidates
                .first()
                .and_then(|candidate| candidate.road_group_key.as_deref()),
            Some("ref:MSR-1"),
            "candidate type preferences should steer ambiguous endpoints toward the signed primary"
        );
        let preferred = preferred_search.candidates.first().unwrap();
        assert!(
            preferred.score_breakdown.candidate_type_score > 0.0,
            "accepted signed primary should expose positive candidate-type score"
        );
    }

    fn plain_way(osm_way_id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
        RoutableWay {
            osm_way_id,
            points,
            road_type: RoadType::Primary,
            directional_access: DirectionalAccess::Both,
            name: Some(format!("Way {osm_way_id}").into()),
            highway: Some("primary".into()),
            access: Some("yes".into()),
            ..Default::default()
        }
    }

    fn graph_of(ways: Vec<RoutableWay>) -> RoadGraph {
        let mut builder = RoadGraphBuilder::new("correlation-shapes");
        for way in ways {
            builder.push_way(way);
        }
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);
        graph
    }

    #[test]
    fn heading_is_read_where_the_point_snaps_on_a_curved_edge() {
        // One edge: east for 72 m, then north for 111 m.
        let graph = graph_of(vec![plain_way(
            1,
            vec![(50.0, 20.0), (50.0, 20.001), (50.001, 20.001)],
        )]);
        let engine = crate::test_support::engine(&graph);
        let search = Correlator::new(engine.route_graph_view()).search(
            GeoPoint::new(50.0008, 20.00102),
            MobilityProfile::Wheeled,
            &CorrelationPolicy {
                max_distance_m: 25,
                ..CorrelationPolicy::default()
            },
            &RouteEndpointOptions::default()
                .with_heading(0)
                .with_heading_tolerance(30),
        );

        // The northbound direction is kept although the edge starts eastward;
        // the southbound one is rejected.
        assert_eq!(
            search.candidates.len(),
            1,
            "{:?}",
            search.rejected_candidates
        );
        let candidate = &search.candidates[0];
        assert!(candidate.fraction_along_edge > 0.5);
        assert_eq!(candidate.heading_deg, Some(0));
        assert_eq!(candidate.heading_delta_deg, Some(0));
        assert!(search.rejected_candidates.iter().any(|rejected| {
            rejected.reason == CorrelationRejectReason::HeadingOutsideTolerance
                && rejected.heading_delta_deg == Some(180)
        }));
    }

    #[test]
    fn expanding_search_looks_past_an_island_for_the_network() {
        // A network of 60 chained ways (61 nodes) and, 200 m north of it, a
        // lone 70 m way.
        let mut ways = (0..60)
            .map(|index| {
                let lng = 20.0 + f64::from(index) * 0.0005;
                plain_way(
                    i64::from(index) + 1,
                    vec![(50.0, lng), (50.0, lng + 0.0005)],
                )
            })
            .collect::<Vec<_>>();
        ways.push(plain_way(100, vec![(50.0018, 20.0), (50.0018, 20.001)]));
        let graph = graph_of(ways);
        let engine = crate::test_support::engine(&graph);
        let correlator = Correlator::new(engine.route_graph_view());
        let point = GeoPoint::new(50.0019, 20.0005);
        let policy = CorrelationPolicy::default();
        let options = RouteEndpointOptions::default();

        let nearby = correlator.search(point, MobilityProfile::Wheeled, &policy, &options);
        assert!(!nearby.candidates.is_empty());
        assert!(nearby.candidates.iter().all(|candidate| {
            candidate.component_size.unwrap_or_default() < EXPANDED_CORRELATION_MIN_COMPONENT_SIZE
        }));
        assert_eq!(nearby.expanded_max_distance_m, None);

        let expanded =
            correlator.search_expanding(point, MobilityProfile::Wheeled, &policy, &options);
        assert_eq!(expanded.expanded_max_distance_m, Some(300));
        // The island's candidates stay first, then the network's.
        assert_eq!(expanded.candidates[0].edge_id, nearby.candidates[0].edge_id);
        assert!(expanded.candidates.iter().any(|candidate| {
            candidate.component_size.unwrap_or_default() >= EXPANDED_CORRELATION_MIN_COMPONENT_SIZE
                && candidate.distance_m > 150.0
        }));
        assert!(expanded.candidates.len() <= policy.candidate_limit);

        // Next to the network, nothing is widened.
        let connected = correlator.search_expanding(
            GeoPoint::new(50.0001, 20.01),
            MobilityProfile::Wheeled,
            &policy,
            &options,
        );
        assert_eq!(connected.expanded_max_distance_m, None);
    }

    #[test]
    fn expanding_search_looks_past_a_profile_island_in_a_large_component() {
        // The 60-way network and, 200 m north of it, a lone 70 m road joined
        // to it by a footway: one component of 63 nodes, but a car on the
        // road reaches only the road.
        let mut ways = (0..60)
            .map(|index| {
                let lng = 20.0 + f64::from(index) * 0.0005;
                plain_way(
                    i64::from(index) + 1,
                    vec![(50.0, lng), (50.0, lng + 0.0005)],
                )
            })
            .collect::<Vec<_>>();
        ways.push(plain_way(100, vec![(50.0018, 20.0), (50.0018, 20.001)]));
        ways.push(RoutableWay {
            osm_way_id: 101,
            points: vec![(50.0018, 20.0), (50.0, 20.0)],
            road_type: RoadType::Path,
            highway: Some("footway".into()),
            ..Default::default()
        });
        let graph = graph_of(ways);
        let engine = crate::test_support::engine(&graph);
        let correlator = Correlator::new(engine.route_graph_view());
        let point = GeoPoint::new(50.0019, 20.0005);
        let policy = CorrelationPolicy::default();
        let options = RouteEndpointOptions::default();

        let nearby = correlator.search(point, MobilityProfile::Wheeled, &policy, &options);
        assert!(!nearby.candidates.is_empty());
        assert!(nearby.candidates.iter().all(|candidate| {
            candidate.component_size.unwrap_or_default() >= EXPANDED_CORRELATION_MIN_COMPONENT_SIZE
                && candidate.outbound_reach_edge_count.unwrap_or_default()
                    < DEFAULT_REACH_EDGE_LIMIT
        }));

        let expanded =
            correlator.search_expanding(point, MobilityProfile::Wheeled, &policy, &options);
        assert_eq!(expanded.expanded_max_distance_m, Some(300));
        assert!(
            expanded
                .candidates
                .iter()
                .any(|candidate| candidate.distance_m > 150.0)
        );
        let route = engine.route_leg(crate::LegRequest::new(
            point,
            GeoPoint::new(50.0001, 20.02),
            MobilityProfile::Wheeled,
        ));
        assert!(route.is_ok(), "{route:?}");
    }
}

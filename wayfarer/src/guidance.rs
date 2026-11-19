use std::collections::BTreeMap;

use crate::types::{
    GeoPoint, RouteDynamicEndpointKind, RouteDynamicVirtualEdgeKind, RouteEdgeRange,
    RouteGuidanceQualityFlag, RouteLane, RouteLaneIndication, RouteLeg, RouteManeuver,
    RouteManeuverKind, RouteShapeDynamicOverlay, RouteShapeDynamicOverlayKind,
    RouteShapeQualityFlag, RouteShapeSegment,
};

const TURN_THRESHOLD_DEG: f64 = 30.0;
const SHORT_CONNECTOR_SUPPRESSION_DISTANCE_M: f64 = 25.0;
/// How far a turn angle (and a maneuver's bearing) is measured on each side of
/// a junction, within the two edges that meet there.
const BEARING_DISTANCE_M: f64 = 20.0;
/// Branches within this angle of the approach are comparable: taking one of
/// two of them is a fork.
const FORK_MAX_ANGLE_DEG: i16 = 45;
/// Traffic joining within this angle of the departure makes a merge.
const MERGE_MAX_ANGLE_DEG: i16 = 45;

#[cfg(test)]
pub(crate) fn build_maneuvers(leg: &RouteLeg) -> Vec<RouteManeuver> {
    build_maneuvers_with(leg, None)
}

/// A leg's geometry with, when the leg builder kept them, the haversine
/// lengths of its steps (`segment_m[i]` from point `i - 1` to point `i`).
#[derive(Clone, Copy)]
pub(crate) struct Shape<'a> {
    points: &'a [GeoPoint],
    segment_m: Option<&'a [f64]>,
}

impl<'a> Shape<'a> {
    fn plain(points: &'a [GeoPoint]) -> Self {
        Self {
            points,
            segment_m: None,
        }
    }

    /// `points[from].distance_to(points[to])`, kept or computed.
    fn length(&self, from: usize, to: usize) -> f64 {
        if let Some(segment_m) = self.segment_m {
            if to == from + 1 {
                return segment_m[to];
            }
            if from == to + 1 {
                return segment_m[from];
            }
        }
        self.points[from].distance_to(self.points[to])
    }
}

impl std::ops::Deref for Shape<'_> {
    type Target = [GeoPoint];
    fn deref(&self) -> &[GeoPoint] {
        self.points
    }
}

pub(crate) fn build_maneuvers_with(
    leg: &RouteLeg,
    segment_m: Option<&[f64]>,
) -> Vec<RouteManeuver> {
    if leg.geometry.len() < 2 {
        return Vec::new();
    }
    let shape = Shape {
        points: &leg.geometry,
        segment_m: segment_m.filter(|segment_m| segment_m.len() == leg.geometry.len()),
    };

    let shapes = EdgeShapes::new(leg);
    let mut maneuvers = Vec::new();
    // The maneuvers that move the route on, in route order, as (index in
    // `maneuvers`, edge index): each covers the edges up to the next one.
    let mut spans = Vec::new();
    let first_edge = leg.edges.first();
    spans.push((maneuvers.len(), 0));
    maneuvers.push(maneuver(
        RouteManeuverKind::Start,
        instruction("Start on", first_edge.and_then(|edge| edge.name.as_deref())),
        "route.start",
        first_edge,
        None,
        shape,
        0,
        shapes.range(0).1.max(1),
        None,
        None,
        None,
    ));

    let mut pending_suppressed_connector_name = None::<String>;
    // Whether the route is on a roundabout whose entry was announced: the ring
    // and its exit get no maneuvers of their own.
    let mut in_announced_roundabout = false;
    for (index, edge) in leg.edges.iter().enumerate() {
        let (begin_shape_index, end_shape_index) = shapes.range(index);
        if edge.has_warning_overlay {
            let shape_segment = shapes.segment(index);
            let mut hazard = maneuver(
                RouteManeuverKind::Hazard,
                instruction(
                    "Hazard warning on",
                    edge.name.as_deref().or(edge.ref_name.as_deref()),
                ),
                "route.hazard",
                Some(edge),
                None,
                shape,
                begin_shape_index,
                end_shape_index,
                None,
                Some(instruction(
                    "Hazard warning on",
                    edge.name.as_deref().or(edge.ref_name.as_deref()),
                )),
                None,
            );
            if let Some(segment) = shape_segment {
                let warning_range = warning_range_for_edge(edge.warning_ranges.as_slice())
                    .unwrap_or(RouteEdgeRange {
                        edge_id: edge.edge_id,
                        start_fraction: segment.start_fraction,
                        end_fraction: segment.end_fraction,
                    })
                    .clipped_to(segment.start_fraction, segment.end_fraction)
                    .unwrap_or(RouteEdgeRange {
                        edge_id: edge.edge_id,
                        start_fraction: segment.start_fraction,
                        end_fraction: segment.end_fraction,
                    });
                hazard.instruction_parameters.insert(
                    "edge_start_fraction".to_string(),
                    format_fraction(warning_range.start_fraction),
                );
                hazard.instruction_parameters.insert(
                    "edge_end_fraction".to_string(),
                    format_fraction(warning_range.end_fraction),
                );
                if warning_range.start_fraction > f64::EPSILON
                    || warning_range.end_fraction < 1.0 - f64::EPSILON
                {
                    hazard
                        .quality_flags
                        .push(crate::types::RouteGuidanceQualityFlag::OverlayWarningOnClippedShape);
                }
            }
            maneuvers.push(hazard);
        }
        if index == 0 {
            continue;
        }
        if in_announced_roundabout {
            in_announced_roundabout = edge.is_roundabout;
            continue;
        }
        let previous_name = leg.edges[index - 1].name.as_deref();
        let turn_angle = shapes.turn_angle(shape, index);
        let significant_turn = turn_angle
            .map(|angle| f64::from(angle).abs() >= TURN_THRESHOLD_DEG)
            .unwrap_or(false);
        let name_changed = previous_name != edge.name.as_deref();
        let entering_roundabout = edge.is_roundabout && !leg.edges[index - 1].is_roundabout;
        if !entering_roundabout {
            if !significant_turn && !name_changed {
                continue;
            }
            if should_suppress_short_connector(leg, &shapes, index, significant_turn, name_changed)
            {
                pending_suppressed_connector_name = connector_name(edge);
                continue;
            }
        }
        let roundabout_exit = if entering_roundabout {
            roundabout_exit(&leg.edges, index)
        } else {
            None
        };
        let roundabout_exit_number = roundabout_exit.map(|(number, _)| number);
        let fork_side = fork_side(edge, turn_angle);
        let kind = if entering_roundabout {
            RouteManeuverKind::Roundabout
        } else if fork_side.is_some() {
            RouteManeuverKind::Fork
        } else if is_merge(edge, turn_angle) {
            RouteManeuverKind::Merge
        } else if significant_turn && turn_angle.unwrap_or(0).abs() > 135 {
            RouteManeuverKind::UTurn
        } else if significant_turn {
            RouteManeuverKind::Turn
        } else if name_changed && turn_angle.is_some() {
            RouteManeuverKind::Keep
        } else {
            RouteManeuverKind::Continue
        };
        let is_ramp_maneuver = is_ramp_maneuver(edge, kind);
        let instruction_text = if is_ramp_maneuver {
            ramp_instruction(edge)
        } else {
            match kind {
                RouteManeuverKind::Turn | RouteManeuverKind::UTurn => instruction_with_sign(
                    turn_phrase(turn_angle.unwrap_or(0)),
                    edge.name.as_deref(),
                    edge,
                ),
                RouteManeuverKind::Keep => instruction_with_sign(
                    keep_phrase(turn_angle.unwrap_or(0)),
                    edge.name.as_deref(),
                    edge,
                ),
                RouteManeuverKind::Fork => instruction_with_sign(
                    fork_phrase(fork_side.unwrap_or(0)),
                    edge.name.as_deref(),
                    edge,
                ),
                RouteManeuverKind::Merge => {
                    instruction_with_sign("Merge onto", edge.name.as_deref(), edge)
                }
                RouteManeuverKind::Roundabout => roundabout_instruction(
                    edge,
                    roundabout_exit.map(|(number, exit_index)| (number, &leg.edges[exit_index])),
                ),
                _ => instruction_with_sign("Continue onto", edge.name.as_deref(), edge),
            }
        };
        let mut route_maneuver = maneuver(
            kind,
            instruction_text,
            if is_ramp_maneuver {
                "route.ramp"
            } else {
                instruction_key(kind)
            },
            Some(edge),
            // `turn:lanes` describes the lanes at the end of the way that
            // approaches the junction.
            Some(&leg.edges[index - 1]),
            shape,
            begin_shape_index,
            end_shape_index,
            turn_angle,
            None,
            roundabout_exit_number,
        );
        if is_ramp_maneuver {
            route_maneuver
                .instruction_parameters
                .insert("guidance_kind".to_string(), "ramp".to_string());
            route_maneuver
                .instruction_parameters
                .insert("road_use".to_string(), "link".to_string());
        }
        if let Some((_, exit_index)) = roundabout_exit {
            in_announced_roundabout = true;
            if let Some(exit_name) = connector_name(&leg.edges[exit_index]) {
                route_maneuver
                    .instruction_parameters
                    .insert("exit_road_name".to_string(), exit_name);
            }
        } else if entering_roundabout {
            in_announced_roundabout = true;
        }
        if let Some(connector_name) = pending_suppressed_connector_name.take() {
            route_maneuver
                .quality_flags
                .push(crate::types::RouteGuidanceQualityFlag::CombinedShortConnector);
            route_maneuver
                .instruction_parameters
                .insert("suppressed_connector_name".to_string(), connector_name);
            route_maneuver.instruction_parameters.insert(
                "suppressed_connector_reason".to_string(),
                "short_connector".to_string(),
            );
        }
        append_classification_parameters(
            &mut route_maneuver.instruction_parameters,
            edge,
            kind,
            significant_turn,
            name_changed,
            entering_roundabout,
        );
        spans.push((maneuvers.len(), index));
        maneuvers.push(route_maneuver);
    }

    // Each maneuver runs to the next one: its distance, duration and shape end.
    let last_shape_index = leg.geometry.len() - 1;
    for (position, &(maneuver_index, edge_index)) in spans.iter().enumerate() {
        let next = spans.get(position + 1);
        let end_edge_index = next.map_or(leg.edges.len(), |(_, edge_index)| *edge_index);
        let end_shape_index = next.map_or(last_shape_index, |(next_maneuver, _)| {
            maneuvers[*next_maneuver].begin_shape_index
        });
        let covered = leg
            .edges
            .get(edge_index..end_edge_index)
            .unwrap_or_default();
        let maneuver = &mut maneuvers[maneuver_index];
        maneuver.distance_m = covered.iter().map(|edge| edge.distance_m).sum();
        // A maneuver starts at its junction, so it carries that junction's
        // time and those of the junctions it passes through.
        maneuver.duration_s = covered
            .iter()
            .map(|edge| edge.duration_s + edge.junction_delay_s)
            .sum();
        maneuver.end_shape_index = end_shape_index.max(maneuver.begin_shape_index);
    }

    append_dynamic_overlay_maneuvers(leg, &mut maneuvers);

    let mut destination = maneuver(
        RouteManeuverKind::Destination,
        "Arrive at destination".to_string(),
        "route.destination",
        leg.edges.last(),
        None,
        shape,
        last_shape_index,
        last_shape_index,
        None,
        None,
        None,
    );
    destination.distance_m = 0.0;
    destination.duration_s = 0.0;
    maneuvers.push(destination);
    maneuvers.sort_by_key(maneuver_sort_key);
    maneuvers
}

/// Each edge's stretch of a leg's geometry: from its shape segment when the
/// leg has one per edge (connector segments set apart), else the edge index
/// as a shape index (synthetic legs).
struct EdgeShapes<'l> {
    segments: Option<Vec<&'l RouteShapeSegment>>,
    last_shape_index: usize,
}

impl<'l> EdgeShapes<'l> {
    fn new(leg: &'l RouteLeg) -> Self {
        let segments = leg
            .shape_segments
            .iter()
            .filter(|segment| {
                !segment
                    .quality_flags
                    .contains(&RouteShapeQualityFlag::SsvgConnector)
            })
            .collect::<Vec<_>>();
        Self {
            segments: (segments.len() == leg.edges.len()).then_some(segments),
            last_shape_index: leg.geometry.len().saturating_sub(1),
        }
    }

    fn segment(&self, edge_index: usize) -> Option<&'l RouteShapeSegment> {
        self.segments.as_ref()?.get(edge_index).copied()
    }

    /// The first and last shape index of the edge.
    fn range(&self, edge_index: usize) -> (usize, usize) {
        let last = self.last_shape_index;
        match self.segment(edge_index) {
            Some(segment) => (
                segment.begin_shape_index.min(last),
                segment.end_shape_index.min(last),
            ),
            None => (edge_index.min(last), (edge_index + 1).min(last)),
        }
    }

    /// The signed turn (degrees, negative to the left) from edge
    /// `edge_index - 1` onto edge `edge_index`, from bearings over up to
    /// `BEARING_DISTANCE_M` on either edge.
    fn turn_angle(&self, geometry: Shape<'_>, edge_index: usize) -> Option<i16> {
        let previous_begin = self.range(edge_index.checked_sub(1)?).0;
        let (junction, end) = self.range(edge_index);
        let before = bearing_before(geometry, junction, previous_begin)?;
        let after = bearing_after(geometry, junction, end)?;
        let mut delta = after - before;
        while delta > 180.0 {
            delta -= 360.0;
        }
        while delta < -180.0 {
            delta += 360.0;
        }
        Some(delta.round() as i16)
    }
}

/// The side of a fork the route takes onto `edge` (negative left, positive
/// right), when another branch the profile may take is within
/// `FORK_MAX_ANGLE_DEG` of the approach, as the route's own is.
fn fork_side(edge: &crate::types::RouteEdgeAttribution, turn_angle: Option<i16>) -> Option<i16> {
    let angle = turn_angle.filter(|angle| angle.abs() <= FORK_MAX_ANGLE_DEG)?;
    let others = edge
        .junction_branch_angles_deg
        .iter()
        .copied()
        .filter(|branch| branch.abs() <= FORK_MAX_ANGLE_DEG)
        .collect::<Vec<_>>();
    if others.is_empty() {
        return None;
    }
    Some(if others.iter().all(|other| *other > angle) {
        -1
    } else if others.iter().all(|other| *other < angle) {
        1
    } else {
        angle.signum()
    })
}

/// Whether the route joins other traffic onto `edge`: no other branch to take,
/// and some road the profile may use joins within `MERGE_MAX_ANGLE_DEG` of the
/// departure while the route itself bends in by at most a fork's angle.
fn is_merge(edge: &crate::types::RouteEdgeAttribution, turn_angle: Option<i16>) -> bool {
    turn_angle.is_some_and(|angle| angle.abs() <= FORK_MAX_ANGLE_DEG)
        && edge.junction_branch_angles_deg.is_empty()
        && edge
            .junction_joining_angles_deg
            .iter()
            .any(|joining| joining.abs() <= MERGE_MAX_ANGLE_DEG)
}

fn append_dynamic_overlay_maneuvers(leg: &RouteLeg, maneuvers: &mut Vec<RouteManeuver>) {
    for segment in &leg.shape_segments {
        let Some(dynamic_overlay) = &segment.dynamic_overlay else {
            continue;
        };
        if !matches!(
            dynamic_overlay.kind,
            RouteShapeDynamicOverlayKind::EndpointConnector
                | RouteShapeDynamicOverlayKind::RoadEntryConnector
                | RouteShapeDynamicOverlayKind::RoadToRoadBypassConnector
                | RouteShapeDynamicOverlayKind::CrossCountryConnector
        ) {
            continue;
        }
        maneuvers.push(dynamic_overlay_maneuver(
            dynamic_overlay,
            Shape::plain(&leg.geometry),
            segment.begin_shape_index,
            segment.end_shape_index,
        ));
    }
}

fn dynamic_overlay_maneuver(
    dynamic_overlay: &RouteShapeDynamicOverlay,
    geometry: Shape<'_>,
    begin_shape_index: usize,
    end_shape_index: usize,
) -> RouteManeuver {
    let instruction = dynamic_overlay_instruction(dynamic_overlay);
    let mut instruction_parameters = BTreeMap::new();
    instruction_parameters.insert(
        "dynamic_overlay_kind".to_string(),
        dynamic_overlay_kind_name(dynamic_overlay.kind).to_string(),
    );
    if let Some(connector_type) = dynamic_overlay.connector_type {
        instruction_parameters.insert(
            "connector_type".to_string(),
            connector_type_name(connector_type).to_string(),
        );
    }
    if let Some(connector_id) = &dynamic_overlay.connector_id {
        instruction_parameters.insert("connector_id".to_string(), connector_id.clone());
    }
    if !dynamic_overlay.source_chunk_ids.is_empty() {
        instruction_parameters.insert(
            "source_chunk_ids".to_string(),
            dynamic_overlay.source_chunk_ids.join(","),
        );
    }
    if let Some(distance_meters) = dynamic_overlay.distance_meters {
        instruction_parameters.insert(
            "connector_distance_m".to_string(),
            format!("{distance_meters:.1}"),
        );
    }
    if let Some(cost) = dynamic_overlay.cost {
        instruction_parameters.insert("connector_cost".to_string(), format!("{cost:.1}"));
    }
    if let Some(endpoint) = dynamic_overlay.source_endpoint {
        instruction_parameters.insert(
            "source_endpoint".to_string(),
            endpoint_kind_name(endpoint).to_string(),
        );
    }
    if let Some(endpoint) = dynamic_overlay.target_endpoint {
        instruction_parameters.insert(
            "target_endpoint".to_string(),
            endpoint_kind_name(endpoint).to_string(),
        );
    }
    if let Some(edge_id) = dynamic_overlay.source_edge_id {
        instruction_parameters.insert("source_edge_id".to_string(), edge_id.to_string());
    }
    if let Some(edge_id) = dynamic_overlay.target_edge_id {
        instruction_parameters.insert("target_edge_id".to_string(), edge_id.to_string());
    }

    RouteManeuver {
        kind: RouteManeuverKind::Continue,
        concise_instruction: concise_instruction(&instruction),
        pre_transition_instruction: None,
        post_transition_instruction: Some(instruction.clone()),
        alert_instruction: Some(instruction.clone()),
        cardinal_direction: bearing_at(geometry, begin_shape_index, end_shape_index)
            .map(cardinal_direction),
        bearing_deg: bearing_at(geometry, begin_shape_index, end_shape_index),
        road_refs: Vec::new(),
        sign_text: None,
        branch_text: None,
        lane_count: None,
        roundabout_exit_number: None,
        instruction_key: "route.ssvg_connector".to_string(),
        instruction_parameters,
        instruction,
        distance_m: dynamic_overlay.distance_meters.unwrap_or(0.0),
        duration_s: 0.0,
        begin_shape_index,
        end_shape_index,
        road_name: None,
        turn_angle_deg: None,
        quality_flags: vec![RouteGuidanceQualityFlag::SsvgConnector],
        lanes: Vec::new(),
        leg_index: None,
    }
}

fn dynamic_overlay_instruction(dynamic_overlay: &RouteShapeDynamicOverlay) -> String {
    match dynamic_overlay.kind {
        RouteShapeDynamicOverlayKind::EndpointConnector => match (
            dynamic_overlay.source_endpoint,
            dynamic_overlay.target_endpoint,
        ) {
            (Some(RouteDynamicEndpointKind::Origin), _) => {
                "Follow SSVG connector from origin to the road network".to_string()
            }
            (_, Some(RouteDynamicEndpointKind::Destination)) => {
                "Follow SSVG connector from the road network to destination".to_string()
            }
            _ => "Follow SSVG endpoint connector".to_string(),
        },
        RouteShapeDynamicOverlayKind::RoadEntryConnector => {
            "Follow SSVG road-entry connector".to_string()
        }
        RouteShapeDynamicOverlayKind::RoadToRoadBypassConnector => {
            "Follow SSVG bypass around blocked road".to_string()
        }
        RouteShapeDynamicOverlayKind::CrossCountryConnector => {
            match dynamic_overlay.distance_meters {
                Some(distance_meters) => format!("Cross open ground ~{distance_meters:.0} m"),
                None => "Cross open ground".to_string(),
            }
        }
        RouteShapeDynamicOverlayKind::PartialEdgeFragment => {
            "Continue on the usable road fragment".to_string()
        }
    }
}

fn maneuver_sort_key(maneuver: &RouteManeuver) -> (usize, u8) {
    let priority = match maneuver.kind {
        RouteManeuverKind::Start => 0,
        RouteManeuverKind::Destination => 3,
        _ if maneuver
            .quality_flags
            .contains(&RouteGuidanceQualityFlag::SsvgConnector) =>
        {
            1
        }
        _ => 2,
    };
    (maneuver.begin_shape_index, priority)
}

fn dynamic_overlay_kind_name(kind: RouteShapeDynamicOverlayKind) -> &'static str {
    match kind {
        RouteShapeDynamicOverlayKind::PartialEdgeFragment => "partial_edge_fragment",
        RouteShapeDynamicOverlayKind::EndpointConnector => "endpoint_connector",
        RouteShapeDynamicOverlayKind::RoadEntryConnector => "road_entry_connector",
        RouteShapeDynamicOverlayKind::RoadToRoadBypassConnector => "road_to_road_bypass_connector",
        RouteShapeDynamicOverlayKind::CrossCountryConnector => "cross_country_connector",
    }
}

fn connector_type_name(kind: RouteDynamicVirtualEdgeKind) -> &'static str {
    match kind {
        RouteDynamicVirtualEdgeKind::Endpoint => "endpoint",
        RouteDynamicVirtualEdgeKind::RoadEntry => "road_entry",
        RouteDynamicVirtualEdgeKind::RoadToRoadBypass => "road_to_road_bypass",
        RouteDynamicVirtualEdgeKind::CrossCountry => "cross_country",
    }
}

fn endpoint_kind_name(kind: RouteDynamicEndpointKind) -> &'static str {
    match kind {
        RouteDynamicEndpointKind::Origin => "origin",
        RouteDynamicEndpointKind::Destination => "destination",
    }
}

fn warning_range_for_edge(ranges: &[RouteEdgeRange]) -> Option<RouteEdgeRange> {
    let first = ranges.first()?;
    let mut start_fraction = first.start_fraction;
    let mut end_fraction = first.end_fraction;
    for range in ranges.iter().skip(1) {
        start_fraction = start_fraction.min(range.start_fraction);
        end_fraction = end_fraction.max(range.end_fraction);
    }
    Some(RouteEdgeRange {
        edge_id: first.edge_id,
        start_fraction,
        end_fraction,
    })
}

fn format_fraction(value: f64) -> String {
    format!("{:.3}", value.clamp(0.0, 1.0))
}

fn should_suppress_short_connector(
    leg: &RouteLeg,
    shapes: &EdgeShapes<'_>,
    index: usize,
    significant_turn: bool,
    name_changed: bool,
) -> bool {
    if index == 0 || significant_turn || !name_changed {
        return false;
    }
    let Some(edge) = leg.edges.get(index) else {
        return false;
    };
    if edge.has_warning_overlay || edge.distance_m > SHORT_CONNECTOR_SUPPRESSION_DISTANCE_M {
        return false;
    }
    let Some(next_edge) = leg.edges.get(index + 1) else {
        return false;
    };
    let next_turn_is_significant = shapes
        .turn_angle(Shape::plain(&leg.geometry), index + 1)
        .map(|angle| f64::from(angle).abs() >= TURN_THRESHOLD_DEG)
        .unwrap_or(false);
    next_turn_is_significant || next_edge.name != edge.name
}

fn connector_name(edge: &crate::types::RouteEdgeAttribution) -> Option<String> {
    edge.name
        .clone()
        .or_else(|| edge.ref_name.clone())
        .filter(|name| !name.trim().is_empty())
}

/// A maneuver onto `edge`. `approach_edge` is the edge that reaches the
/// maneuver's junction, whose lanes the maneuver describes; `None` for one
/// that is not at a junction.
fn maneuver(
    kind: RouteManeuverKind,
    instruction: String,
    instruction_key: &str,
    edge: Option<&crate::types::RouteEdgeAttribution>,
    approach_edge: Option<&crate::types::RouteEdgeAttribution>,
    geometry: Shape<'_>,
    begin_shape_index: usize,
    end_shape_index: usize,
    turn_angle_deg: Option<i16>,
    alert_instruction: Option<String>,
    roundabout_exit_number: Option<u16>,
) -> RouteManeuver {
    let road_name = edge.and_then(|edge| edge.name.clone());
    let bearing = bearing_after(geometry, begin_shape_index, end_shape_index)
        .map(|bearing| bearing.round() as u16 % 360);
    let mut instruction_parameters = BTreeMap::new();
    if let Some(name) = &road_name {
        instruction_parameters.insert("road_name".to_string(), name.clone());
    }
    if let Some(angle) = turn_angle_deg {
        instruction_parameters.insert("turn_angle_deg".to_string(), angle.to_string());
    }
    if let Some(lane_count) = edge.and_then(|edge| edge.lane_count) {
        instruction_parameters.insert("lane_count".to_string(), lane_count.to_string());
    }
    let lanes = approach_edge.map_or_else(Vec::new, |approach| {
        append_lane_guidance_parameters(&mut instruction_parameters, approach, kind, turn_angle_deg)
    });
    if let Some(exit_number) = roundabout_exit_number {
        instruction_parameters.insert(
            "roundabout_exit_number".to_string(),
            exit_number.to_string(),
        );
    }
    if let Some(roundabout_exit_count) = edge
        .map(|edge| edge.roundabout_exit_count)
        .filter(|count| *count > 0)
    {
        instruction_parameters.insert(
            "roundabout_exit_count".to_string(),
            roundabout_exit_count.to_string(),
        );
    }
    if let Some(sign_text) = edge.and_then(sign_text_for_edge) {
        instruction_parameters.insert("sign_text".to_string(), sign_text);
    }
    if let Some(branch_text) = edge.and_then(branch_text_for_edge) {
        instruction_parameters.insert("branch_text".to_string(), branch_text);
    }
    if let Some(edge) = edge {
        append_sign_fact_parameters(&mut instruction_parameters, edge);
    }
    let mut quality_flags = Vec::new();
    if road_name
        .as_deref()
        .is_none_or(|name| name.trim().is_empty())
        && !matches!(kind, RouteManeuverKind::Destination)
    {
        quality_flags.push(crate::types::RouteGuidanceQualityFlag::MissingRoadName);
    }
    if turn_angle_deg.is_none()
        && !matches!(
            kind,
            RouteManeuverKind::Start
                | RouteManeuverKind::Waypoint
                | RouteManeuverKind::Destination
                | RouteManeuverKind::Hazard
        )
    {
        quality_flags.push(crate::types::RouteGuidanceQualityFlag::InferredHeading);
    }
    if edge.is_some_and(|edge| {
        edge.junction_incoming_edge_count + edge.junction_outgoing_edge_count > 3
    }) {
        quality_flags.push(crate::types::RouteGuidanceQualityFlag::ComplexJunction);
    }
    if matches!(kind, RouteManeuverKind::Continue) {
        quality_flags.push(crate::types::RouteGuidanceQualityFlag::WeakManeuverClassification);
    }

    RouteManeuver {
        kind,
        concise_instruction: concise_instruction(&instruction),
        pre_transition_instruction: pre_transition_instruction(kind, &instruction),
        post_transition_instruction: post_transition_instruction(kind, &instruction),
        alert_instruction,
        cardinal_direction: bearing.map(cardinal_direction),
        bearing_deg: bearing,
        road_refs: edge.map_or_else(Vec::new, |edge| {
            edge.route_refs
                .iter()
                .cloned()
                .chain(edge.ref_name.clone())
                .collect()
        }),
        sign_text: edge.and_then(sign_text_for_edge),
        branch_text: edge.and_then(branch_text_for_edge),
        lane_count: edge.and_then(|edge| edge.lane_count),
        roundabout_exit_number,
        instruction_key: instruction_key.to_string(),
        instruction_parameters,
        instruction,
        distance_m: edge.map_or(0.0, |edge| edge.distance_m),
        duration_s: edge.map_or(0.0, |edge| edge.duration_s),
        begin_shape_index,
        end_shape_index,
        road_name,
        turn_angle_deg,
        quality_flags,
        lanes,
        leg_index: None,
    }
}

fn sign_text_for_edge(edge: &crate::types::RouteEdgeAttribution) -> Option<String> {
    edge.destination
        .clone()
        .or_else(|| edge.destination_street.clone())
        .or_else(|| edge.destination_ref.clone())
        .or_else(|| edge.route_refs.first().cloned())
        .or_else(|| edge.ref_name.clone())
}

fn branch_text_for_edge(edge: &crate::types::RouteEdgeAttribution) -> Option<String> {
    edge.destination_ref
        .clone()
        .or_else(|| edge.junction_ref.clone())
        .or_else(|| edge.ref_name.clone())
}

fn instruction_branch_text_for_edge(edge: &crate::types::RouteEdgeAttribution) -> Option<String> {
    edge.destination_ref
        .clone()
        .or_else(|| edge.ref_name.clone())
        .map(|value| value.trim().to_string())
        .filter(|value| !value.is_empty())
}

fn exit_text_for_edge(edge: &crate::types::RouteEdgeAttribution) -> Option<String> {
    edge.junction_ref
        .clone()
        .map(|value| value.trim().to_string())
        .filter(|value| !value.is_empty())
}

fn append_sign_fact_parameters(
    parameters: &mut BTreeMap<String, String>,
    edge: &crate::types::RouteEdgeAttribution,
) {
    insert_non_empty_parameter(parameters, "destination", edge.destination.as_deref());
    insert_non_empty_parameter(
        parameters,
        "destination_ref",
        edge.destination_ref.as_deref(),
    );
    insert_non_empty_parameter(
        parameters,
        "destination_street",
        edge.destination_street.as_deref(),
    );
    insert_non_empty_parameter(parameters, "junction_ref", edge.junction_ref.as_deref());
}

fn insert_non_empty_parameter(
    parameters: &mut BTreeMap<String, String>,
    key: &str,
    value: Option<&str>,
) {
    let Some(value) = value.map(str::trim).filter(|value| !value.is_empty()) else {
        return;
    };
    parameters.insert(key.to_string(), value.to_string());
}

fn append_classification_parameters(
    parameters: &mut BTreeMap<String, String>,
    edge: &crate::types::RouteEdgeAttribution,
    kind: RouteManeuverKind,
    significant_turn: bool,
    name_changed: bool,
    entering_roundabout: bool,
) {
    parameters.insert(
        "guidance_reason".to_string(),
        guidance_reason(
            edge,
            kind,
            significant_turn,
            name_changed,
            entering_roundabout,
        )
        .to_string(),
    );
    parameters.insert("significant_turn".to_string(), significant_turn.to_string());
    parameters.insert("name_changed".to_string(), name_changed.to_string());
    if edge.junction_outgoing_edge_count > 0 {
        parameters.insert(
            "junction_outgoing_edge_count".to_string(),
            edge.junction_outgoing_edge_count.to_string(),
        );
    }
    if edge.junction_incoming_edge_count > 0 {
        parameters.insert(
            "junction_incoming_edge_count".to_string(),
            edge.junction_incoming_edge_count.to_string(),
        );
    }
    let complex_edge_count = edge.junction_incoming_edge_count + edge.junction_outgoing_edge_count;
    if complex_edge_count > 3 {
        parameters.insert(
            "complex_junction_edge_count".to_string(),
            complex_edge_count.to_string(),
        );
    }
}

fn guidance_reason(
    edge: &crate::types::RouteEdgeAttribution,
    kind: RouteManeuverKind,
    significant_turn: bool,
    name_changed: bool,
    entering_roundabout: bool,
) -> &'static str {
    if entering_roundabout {
        return "entering_roundabout";
    }
    match kind {
        RouteManeuverKind::Fork => "branching_junction",
        RouteManeuverKind::Merge => "merging_junction",
        RouteManeuverKind::UTurn => "u_turn_angle",
        RouteManeuverKind::Turn if significant_turn => "significant_turn",
        RouteManeuverKind::Keep if name_changed => "name_change",
        RouteManeuverKind::Continue if edge.is_link => "link_continuation",
        RouteManeuverKind::Continue => "fallback_continue",
        _ => "maneuver_classification",
    }
}

/// Adds the lane facts of `edge`, the edge approaching a maneuver's junction,
/// to the maneuver's parameters, and returns its lanes.
fn append_lane_guidance_parameters(
    parameters: &mut BTreeMap<String, String>,
    edge: &crate::types::RouteEdgeAttribution,
    kind: RouteManeuverKind,
    turn_angle_deg: Option<i16>,
) -> Vec<RouteLane> {
    let Some(turn_lanes) = selected_turn_lanes(edge).filter(|value| !value.trim().is_empty())
    else {
        return Vec::new();
    };
    parameters.insert("turn_lanes".to_string(), turn_lanes.clone());
    if let Some(forward) = edge.turn_lanes_forward.as_deref() {
        parameters.insert("turn_lanes_forward".to_string(), forward.to_string());
    }
    if let Some(backward) = edge.turn_lanes_backward.as_deref() {
        parameters.insert("turn_lanes_backward".to_string(), backward.to_string());
    }
    if edge.lane_connectivity_available {
        parameters.insert(
            "lane_connectivity_available".to_string(),
            "true".to_string(),
        );
    }

    let target = maneuver_lane_target(kind, turn_angle_deg);
    let lanes = turn_lanes
        .split('|')
        .map(|lane| RouteLane {
            indications: lane_indications(lane),
            valid: target.is_some_and(|target| lane_matches_target(lane, target)),
        })
        .collect();
    if let Some(hint) = target.and_then(|target| lane_guidance_hint(&turn_lanes, target)) {
        parameters.insert("lane_guidance".to_string(), hint.label);
        parameters.insert("lane_guidance_target".to_string(), hint.target);
        parameters.insert(
            "lane_guidance_lane_indexes".to_string(),
            hint.lane_indexes
                .iter()
                .map(usize::to_string)
                .collect::<Vec<_>>()
                .join(","),
        );
        parameters.insert(
            "lane_guidance_lane_count".to_string(),
            hint.count.to_string(),
        );
    }
    lanes
}

/// The `turn:lanes` value for the direction the route travels `edge` in. The
/// unsuffixed tag is the forward one on a two-way road, and the lanes of the
/// only direction on a oneway (an edge without an opposing edge).
fn selected_turn_lanes(edge: &crate::types::RouteEdgeAttribution) -> Option<String> {
    match edge.traversal_direction {
        wayfarer_package::TraversalDirection::Forward => edge
            .turn_lanes_forward
            .clone()
            .or_else(|| edge.turn_lanes.clone()),
        wayfarer_package::TraversalDirection::Backward => {
            edge.turn_lanes_backward.clone().or_else(|| {
                edge.opposing_edge_id
                    .is_none()
                    .then(|| edge.turn_lanes.clone())
                    .flatten()
            })
        }
    }
}

struct LaneGuidanceHint {
    label: String,
    target: String,
    count: usize,
    lane_indexes: Vec<usize>,
}

fn lane_guidance_hint(turn_lanes: &str, target: LaneTarget) -> Option<LaneGuidanceHint> {
    let lane_indexes = turn_lanes
        .split('|')
        .enumerate()
        .filter_map(|(index, lane)| lane_matches_target(lane, target).then_some(index + 1))
        .collect::<Vec<_>>();
    if lane_indexes.is_empty() {
        return None;
    }
    let target_label = target.label();
    Some(LaneGuidanceHint {
        label: format!("use_{target_label}_lane"),
        target: target_label.to_string(),
        count: lane_indexes.len(),
        lane_indexes,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LaneTarget {
    Left,
    Right,
    Through,
    Reverse,
}

impl LaneTarget {
    fn label(self) -> &'static str {
        match self {
            Self::Left => "left",
            Self::Right => "right",
            Self::Through => "through",
            Self::Reverse => "reverse",
        }
    }
}

fn maneuver_lane_target(
    kind: RouteManeuverKind,
    turn_angle_deg: Option<i16>,
) -> Option<LaneTarget> {
    match kind {
        RouteManeuverKind::Turn | RouteManeuverKind::Fork | RouteManeuverKind::Keep => {
            let angle = turn_angle_deg?;
            if angle < 0 {
                Some(LaneTarget::Left)
            } else {
                Some(LaneTarget::Right)
            }
        }
        RouteManeuverKind::UTurn => Some(LaneTarget::Reverse),
        RouteManeuverKind::Merge | RouteManeuverKind::Continue => Some(LaneTarget::Through),
        _ => None,
    }
}

/// Whether a lane (one `turn:lanes` value) serves the target. An unmarked
/// lane (`none` or empty) goes straight on.
fn lane_matches_target(lane: &str, target: LaneTarget) -> bool {
    lane.split(';').any(|token| {
        let token = token.trim();
        match target {
            LaneTarget::Left => matches!(token, "left" | "slight_left" | "sharp_left"),
            LaneTarget::Right => matches!(token, "right" | "slight_right" | "sharp_right"),
            LaneTarget::Through => matches!(
                token,
                "through" | "merge_to_left" | "merge_to_right" | "none" | ""
            ),
            LaneTarget::Reverse => matches!(token, "reverse" | "uturn"),
        }
    })
}

/// The directions marked on a lane (one `turn:lanes` value), unknown marks
/// left out.
fn lane_indications(lane: &str) -> Vec<RouteLaneIndication> {
    let mut indications = Vec::new();
    for token in lane.split(';') {
        let indication = match token.trim() {
            "" | "none" => RouteLaneIndication::None,
            "through" => RouteLaneIndication::Through,
            "slight_left" => RouteLaneIndication::SlightLeft,
            "left" => RouteLaneIndication::Left,
            "sharp_left" => RouteLaneIndication::SharpLeft,
            "slight_right" => RouteLaneIndication::SlightRight,
            "right" => RouteLaneIndication::Right,
            "sharp_right" => RouteLaneIndication::SharpRight,
            "reverse" | "uturn" => RouteLaneIndication::Reverse,
            "merge_to_left" => RouteLaneIndication::MergeToLeft,
            "merge_to_right" => RouteLaneIndication::MergeToRight,
            _ => continue,
        };
        if !indications.contains(&indication) {
            indications.push(indication);
        }
    }
    indications
}

/// The instruction entering a roundabout on `edge`: the exit to take and the
/// road it leads onto (whose signs it reads), when the route leaves the ring.
fn roundabout_instruction(
    edge: &crate::types::RouteEdgeAttribution,
    exit: Option<(u16, &crate::types::RouteEdgeAttribution)>,
) -> String {
    match exit {
        Some((exit_number, exit_edge)) => {
            let suffix = exit_edge.name.as_deref().unwrap_or("unnamed road");
            append_sign_guidance(
                format!("Enter roundabout and take exit {exit_number} onto {suffix}"),
                exit_edge,
            )
        }
        None => append_sign_guidance(
            instruction("Enter roundabout onto", edge.name.as_deref()),
            edge,
        ),
    }
}

fn is_ramp_maneuver(edge: &crate::types::RouteEdgeAttribution, kind: RouteManeuverKind) -> bool {
    edge.is_link
        && matches!(
            kind,
            RouteManeuverKind::Turn
                | RouteManeuverKind::Keep
                | RouteManeuverKind::Fork
                | RouteManeuverKind::Merge
        )
}

fn ramp_instruction(edge: &crate::types::RouteEdgeAttribution) -> String {
    let base = if let Some(exit_text) = exit_text_for_edge(edge) {
        instruction(&format!("Take exit {exit_text} onto"), edge.name.as_deref())
    } else {
        instruction("Take ramp onto", edge.name.as_deref())
    };
    append_sign_guidance(base, edge)
}

/// The exit a route entering a roundabout at `entry_index` takes: its number,
/// counting the ring nodes passed where the profile may leave the ring, and
/// the index of the edge that leaves it. None when the route ends on the ring.
fn roundabout_exit(
    edges: &[crate::types::RouteEdgeAttribution],
    entry_index: usize,
) -> Option<(u16, usize)> {
    let mut exit_number = 1u16;
    for (index, edge) in edges.iter().enumerate().skip(entry_index + 1) {
        if !edge.is_roundabout {
            return Some((exit_number, index));
        }
        if !edge.junction_branch_angles_deg.is_empty() {
            exit_number = exit_number.saturating_add(1);
        }
    }
    None
}

fn instruction_key(kind: RouteManeuverKind) -> &'static str {
    match kind {
        RouteManeuverKind::Start => "route.start",
        RouteManeuverKind::Continue => "route.continue",
        RouteManeuverKind::Keep => "route.keep",
        RouteManeuverKind::Fork => "route.fork",
        RouteManeuverKind::Merge => "route.merge",
        RouteManeuverKind::Roundabout => "route.roundabout",
        RouteManeuverKind::Turn => "route.turn",
        RouteManeuverKind::UTurn => "route.u_turn",
        RouteManeuverKind::Hazard => "route.hazard",
        RouteManeuverKind::Waypoint => "route.waypoint",
        RouteManeuverKind::Destination => "route.destination",
    }
}

fn concise_instruction(instruction: &str) -> String {
    instruction
        .split_once(" onto ")
        .map(|(_, suffix)| suffix.to_string())
        .unwrap_or_else(|| instruction.to_string())
}

fn pre_transition_instruction(kind: RouteManeuverKind, instruction: &str) -> Option<String> {
    match kind {
        RouteManeuverKind::Turn
        | RouteManeuverKind::UTurn
        | RouteManeuverKind::Keep
        | RouteManeuverKind::Fork
        | RouteManeuverKind::Merge
        | RouteManeuverKind::Roundabout => Some(instruction.to_string()),
        _ => None,
    }
}

fn post_transition_instruction(kind: RouteManeuverKind, instruction: &str) -> Option<String> {
    match kind {
        RouteManeuverKind::Start | RouteManeuverKind::Continue => Some(instruction.to_string()),
        _ => None,
    }
}

fn bearing_at(
    geometry: Shape<'_>,
    begin_shape_index: usize,
    end_shape_index: usize,
) -> Option<u16> {
    let from = geometry.get(begin_shape_index)?;
    let to = geometry
        .get(end_shape_index)
        .filter(|point| point.distance_to(*from) > 0.01)
        .or_else(|| geometry.get(begin_shape_index + 1))?;
    Some(bearing_deg(*from, *to).round() as u16 % 360)
}

pub(crate) fn cardinal_direction(bearing: u16) -> String {
    match ((bearing as f64 + 22.5) / 45.0).floor() as u8 % 8 {
        0 => "N",
        1 => "NE",
        2 => "E",
        3 => "SE",
        4 => "S",
        5 => "SW",
        6 => "W",
        _ => "NW",
    }
    .to_string()
}

fn instruction(prefix: &str, road_name: Option<&str>) -> String {
    match road_name {
        Some(name) if !name.trim().is_empty() => format!("{prefix} {name}"),
        _ => prefix.to_string(),
    }
}

fn instruction_with_sign(
    prefix: &str,
    road_name: Option<&str>,
    edge: &crate::types::RouteEdgeAttribution,
) -> String {
    append_sign_guidance(instruction(prefix, road_name), edge)
}

fn append_sign_guidance(base: String, edge: &crate::types::RouteEdgeAttribution) -> String {
    let Some(sign_text) = sign_text_for_edge(edge).map(|value| value.trim().to_string()) else {
        return base;
    };
    if sign_text.is_empty() || base.to_lowercase().contains(&sign_text.to_lowercase()) {
        return base;
    }
    let mut guidance = format!("{base} toward {sign_text}");
    if let Some(branch_text) = instruction_branch_text_for_edge(edge) {
        let guidance_lower = guidance.to_lowercase();
        if !guidance_lower.contains(&branch_text.to_lowercase())
            && !sign_text.eq_ignore_ascii_case(&branch_text)
        {
            guidance = format!("{guidance} via {branch_text}");
        }
    }
    guidance
}

fn turn_phrase(angle: i16) -> &'static str {
    if !(-135..=135).contains(&angle) {
        "Make a U-turn onto"
    } else if angle < 0 {
        "Turn left onto"
    } else {
        "Turn right onto"
    }
}

fn keep_phrase(angle: i16) -> &'static str {
    if angle < 0 {
        "Keep left onto"
    } else {
        "Keep right onto"
    }
}

fn fork_phrase(angle: i16) -> &'static str {
    if angle < 0 {
        "Fork left onto"
    } else {
        "Fork right onto"
    }
}

/// The bearing (degrees) of the route arriving at `shape_index`, from the
/// point up to `BEARING_DISTANCE_M` before it, not before `first_shape_index`.
fn bearing_before(
    geometry: Shape<'_>,
    shape_index: usize,
    first_shape_index: usize,
) -> Option<f64> {
    let to = *geometry.get(shape_index)?;
    let from = point_along(
        geometry,
        (first_shape_index..shape_index)
            .rev()
            .map(|index| (index + 1, index)),
    )?;
    Some(bearing_deg(from, to))
}

/// The bearing (degrees) of the route leaving `shape_index`, to the point up
/// to `BEARING_DISTANCE_M` after it, not after `last_shape_index`.
fn bearing_after(geometry: Shape<'_>, shape_index: usize, last_shape_index: usize) -> Option<f64> {
    let from = *geometry.get(shape_index)?;
    let last_shape_index = last_shape_index.min(geometry.len().saturating_sub(1));
    let to = point_along(
        geometry,
        (shape_index..last_shape_index).map(|index| (index, index + 1)),
    )?;
    Some(bearing_deg(from, to))
}

/// The point `BEARING_DISTANCE_M` along the geometry's `(from, to)` shape
/// steps, or the last point reached when they are shorter; None when they
/// cover no distance.
fn point_along(
    geometry: Shape<'_>,
    steps: impl Iterator<Item = (usize, usize)>,
) -> Option<GeoPoint> {
    let mut remaining = BEARING_DISTANCE_M;
    let mut reached = None;
    for (from, to) in steps {
        let (a, b) = (geometry[from], geometry[to]);
        let length = geometry.length(from, to);
        if length <= 0.01 {
            continue;
        }
        if length >= remaining {
            let t = remaining / length;
            return Some(GeoPoint::new(
                a.lat + (b.lat - a.lat) * t,
                a.lng + (b.lng - a.lng) * t,
            ));
        }
        remaining -= length;
        reached = Some(b);
    }
    reached
}

pub(crate) fn bearing_deg(a: GeoPoint, b: GeoPoint) -> f64 {
    let lat1 = a.lat.to_radians();
    let lat2 = b.lat.to_radians();
    let dlng = (b.lng - a.lng).to_radians();
    let y = dlng.sin() * lat2.cos();
    let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * dlng.cos();
    y.atan2(x).to_degrees().rem_euclid(360.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{
        RouteCostBreakdown, RouteDynamicEndpointKind, RouteDynamicVirtualEdgeKind,
        RouteEdgeAttribution, RouteGuidanceQualityFlag, RouteShapeDynamicOverlay,
        RouteShapeDynamicOverlayKind, RouteShapeSegment,
    };
    use wayfarer_package::{RoadType, TraversalDirection};

    fn edge(edge_id: u64, name: &str) -> RouteEdgeAttribution {
        RouteEdgeAttribution {
            edge_id,
            shared_edge_info_id: edge_id,
            road_type: RoadType::Primary,
            name: Some(name.to_string()),
            ref_name: None,
            route_refs: Vec::new(),
            surface: None,
            tracktype: None,
            smoothness: None,
            maxspeed_kph: None,
            inferred_speed_kph: None,
            speed_source: None,
            lane_count: None,
            lanes: None,
            turn_lanes: None,
            turn_lanes_forward: None,
            turn_lanes_backward: None,
            lane_connectivity_available: false,
            speed_source_provenance: None,
            free_flow_speed_kph: None,
            constrained_speed_kph: None,
            admin_country_id: None,
            admin_region_id: None,
            timezone_id: None,
            conditional_restrictions: Vec::new(),
            opposing_edge_id: None,
            layer: None,
            level: None,
            destination: None,
            destination_ref: None,
            destination_street: None,
            junction_ref: None,
            is_bridge: false,
            is_tunnel: false,
            has_ford: false,
            is_toll: false,
            is_ferry: false,
            has_gate: false,
            is_access_restricted: false,
            is_destination_only: false,
            is_link: false,
            is_roundabout: false,
            roundabout_exit_count: 0,
            has_warning_overlay: false,
            warning_ranges: Vec::new(),
            junction_outgoing_edge_count: 0,
            junction_incoming_edge_count: 0,
            junction_branch_angles_deg: Vec::new(),
            junction_joining_angles_deg: Vec::new(),
            distance_m: 100.0,
            duration_s: 12.0,
            junction_delay_s: 0.0,
            traversal_direction: TraversalDirection::Forward,
            cost_breakdown: RouteCostBreakdown::default(),
        }
    }

    #[test]
    fn turn_instruction_uses_destination_sign_text() {
        // The lanes are those of the road approaching the turn: the entered
        // road's own `turn:lanes` describe its far end.
        let mut approach_edge = edge(1, "Opolska");
        approach_edge.turn_lanes_forward = Some("left|through|right".to_string());
        approach_edge.lane_connectivity_available = true;
        let mut signed_edge = edge(2, "Ramp");
        signed_edge.destination = Some("Krakow".to_string());
        signed_edge.destination_ref = Some("A4".to_string());
        signed_edge.destination_street = Some("Zakopianka".to_string());
        signed_edge.junction_ref = Some("7A".to_string());
        signed_edge.lane_count = Some(2);
        signed_edge.turn_lanes_forward = Some("through|right".to_string());
        let leg = RouteLeg {
            distance_m: 200.0,
            duration_s: 24.0,
            geometry: vec![
                GeoPoint::new(50.0, 20.0),
                GeoPoint::new(50.0, 20.001),
                GeoPoint::new(50.001, 20.001),
            ],
            edges: vec![approach_edge, signed_edge],
            shape_segments: vec![
                RouteShapeSegment {
                    edge_id: 1,
                    begin_shape_index: 0,
                    end_shape_index: 1,
                    start_fraction: 0.0,
                    end_fraction: 1.0,
                    quality_flags: Vec::new(),
                    dynamic_overlay: None,
                },
                RouteShapeSegment {
                    edge_id: 2,
                    begin_shape_index: 1,
                    end_shape_index: 2,
                    start_fraction: 0.0,
                    end_fraction: 1.0,
                    quality_flags: Vec::new(),
                    dynamic_overlay: None,
                },
            ],
        };

        let maneuvers = build_maneuvers(&leg);
        let turn = maneuvers
            .iter()
            .find(|maneuver| maneuver.kind == RouteManeuverKind::Turn)
            .expect("turn maneuver");

        // Maneuvers that are not at a junction carry no lanes.
        for endpoint in maneuvers
            .iter()
            .filter(|maneuver| maneuver.kind != RouteManeuverKind::Turn)
        {
            assert!(endpoint.lanes.is_empty());
            assert!(!endpoint.instruction_parameters.contains_key("turn_lanes"));
        }
        assert_eq!(
            turn.lanes,
            vec![
                RouteLane {
                    indications: vec![RouteLaneIndication::Left],
                    valid: true,
                },
                RouteLane {
                    indications: vec![RouteLaneIndication::Through],
                    valid: false,
                },
                RouteLane {
                    indications: vec![RouteLaneIndication::Right],
                    valid: false,
                },
            ]
        );
        assert!(turn.instruction.contains("toward Krakow"));
        assert_eq!(turn.sign_text.as_deref(), Some("Krakow"));
        assert_eq!(turn.branch_text.as_deref(), Some("A4"));
        assert_eq!(turn.lane_count, Some(2));
        assert_eq!(
            turn.instruction_parameters
                .get("sign_text")
                .map(String::as_str),
            Some("Krakow")
        );
        assert_eq!(
            turn.instruction_parameters
                .get("destination_ref")
                .map(String::as_str),
            Some("A4")
        );
        assert_eq!(
            turn.instruction_parameters
                .get("destination_street")
                .map(String::as_str),
            Some("Zakopianka")
        );
        assert_eq!(
            turn.instruction_parameters
                .get("junction_ref")
                .map(String::as_str),
            Some("7A")
        );
        assert_eq!(
            turn.instruction_parameters
                .get("turn_lanes")
                .map(String::as_str),
            Some("left|through|right")
        );
        assert_eq!(
            turn.instruction_parameters
                .get("lane_guidance")
                .map(String::as_str),
            Some("use_left_lane")
        );
        assert_eq!(
            turn.instruction_parameters
                .get("lane_guidance_target")
                .map(String::as_str),
            Some("left")
        );
        assert_eq!(
            turn.instruction_parameters
                .get("lane_guidance_lane_indexes")
                .map(String::as_str),
            Some("1")
        );
        assert_eq!(
            turn.instruction_parameters
                .get("lane_connectivity_available")
                .map(String::as_str),
            Some("true")
        );
    }

    #[test]
    fn lane_guidance_uses_directional_turn_lane_tags() {
        let mut backward_edge = edge(2, "Return ramp");
        backward_edge.traversal_direction = TraversalDirection::Backward;
        backward_edge.turn_lanes = Some("through|right".to_string());
        backward_edge.turn_lanes_forward = Some("left|through".to_string());
        backward_edge.turn_lanes_backward = Some("through|right".to_string());
        backward_edge.lane_connectivity_available = true;
        let mut parameters = BTreeMap::new();

        append_lane_guidance_parameters(
            &mut parameters,
            &backward_edge,
            RouteManeuverKind::Turn,
            Some(90),
        );

        assert_eq!(
            parameters.get("turn_lanes").map(String::as_str),
            Some("through|right")
        );
        assert_eq!(
            parameters.get("turn_lanes_forward").map(String::as_str),
            Some("left|through")
        );
        assert_eq!(
            parameters.get("turn_lanes_backward").map(String::as_str),
            Some("through|right")
        );
        assert_eq!(
            parameters.get("lane_guidance").map(String::as_str),
            Some("use_right_lane")
        );
        assert_eq!(
            parameters.get("lane_guidance_target").map(String::as_str),
            Some("right")
        );
        assert_eq!(
            parameters
                .get("lane_guidance_lane_indexes")
                .map(String::as_str),
            Some("2")
        );
        assert_eq!(
            parameters
                .get("lane_guidance_lane_count")
                .map(String::as_str),
            Some("1")
        );
    }

    #[test]
    fn unsuffixed_turn_lanes_are_forward_on_two_way_roads() {
        let mut two_way = edge(2, "Two-way");
        two_way.traversal_direction = TraversalDirection::Backward;
        two_way.opposing_edge_id = Some(3);
        two_way.turn_lanes = Some("left|through".to_string());
        let mut parameters = BTreeMap::new();
        let lanes = append_lane_guidance_parameters(
            &mut parameters,
            &two_way,
            RouteManeuverKind::Turn,
            Some(-90),
        );
        assert!(lanes.is_empty());
        assert!(parameters.is_empty());

        // Travelled forward, the tag applies.
        two_way.traversal_direction = TraversalDirection::Forward;
        let lanes = append_lane_guidance_parameters(
            &mut parameters,
            &two_way,
            RouteManeuverKind::Turn,
            Some(-90),
        );
        assert_eq!(lanes.len(), 2);
        assert_eq!(
            parameters.get("lane_guidance").map(String::as_str),
            Some("use_left_lane")
        );

        // A oneway travelled backward (`oneway=-1`) has only these lanes.
        let mut reverse_oneway = edge(4, "Reverse oneway");
        reverse_oneway.traversal_direction = TraversalDirection::Backward;
        reverse_oneway.turn_lanes = Some("left|through".to_string());
        let mut parameters = BTreeMap::new();
        let lanes = append_lane_guidance_parameters(
            &mut parameters,
            &reverse_oneway,
            RouteManeuverKind::Turn,
            Some(-90),
        );
        assert_eq!(lanes.len(), 2);
        assert_eq!(
            parameters.get("turn_lanes").map(String::as_str),
            Some("left|through")
        );
    }

    #[test]
    fn unmarked_and_sharp_lanes_are_matched() {
        let mut approach = edge(1, "Approach");
        approach.turn_lanes_forward = Some("sharp_left|none||slight_right;sharp_right".to_string());

        let mut parameters = BTreeMap::new();
        let lanes = append_lane_guidance_parameters(
            &mut parameters,
            &approach,
            RouteManeuverKind::Continue,
            Some(0),
        );
        assert_eq!(
            parameters
                .get("lane_guidance_lane_indexes")
                .map(String::as_str),
            Some("2,3")
        );
        assert_eq!(
            lanes,
            vec![
                RouteLane {
                    indications: vec![RouteLaneIndication::SharpLeft],
                    valid: false,
                },
                RouteLane {
                    indications: vec![RouteLaneIndication::None],
                    valid: true,
                },
                RouteLane {
                    indications: vec![RouteLaneIndication::None],
                    valid: true,
                },
                RouteLane {
                    indications: vec![
                        RouteLaneIndication::SlightRight,
                        RouteLaneIndication::SharpRight
                    ],
                    valid: false,
                },
            ]
        );

        for (angle, indexes) in [(-150, None), (-120, Some("1")), (120, Some("4"))] {
            let kind = if angle == -150 {
                RouteManeuverKind::UTurn
            } else {
                RouteManeuverKind::Turn
            };
            let mut parameters = BTreeMap::new();
            let lanes =
                append_lane_guidance_parameters(&mut parameters, &approach, kind, Some(angle));
            assert_eq!(
                parameters
                    .get("lane_guidance_lane_indexes")
                    .map(String::as_str),
                indexes
            );
            assert_eq!(
                lanes.iter().filter(|lane| lane.valid).count(),
                usize::from(indexes.is_some())
            );
        }
    }

    #[test]
    fn lane_guidance_reports_all_matching_lane_indexes() {
        let mut through_edge = edge(2, "Merge road");
        through_edge.turn_lanes_forward = Some("through|through;right|right".to_string());
        let mut parameters = BTreeMap::new();

        append_lane_guidance_parameters(
            &mut parameters,
            &through_edge,
            RouteManeuverKind::Merge,
            Some(4),
        );

        assert_eq!(
            parameters.get("lane_guidance").map(String::as_str),
            Some("use_through_lane")
        );
        assert_eq!(
            parameters.get("lane_guidance_target").map(String::as_str),
            Some("through")
        );
        assert_eq!(
            parameters
                .get("lane_guidance_lane_indexes")
                .map(String::as_str),
            Some("1,2")
        );
        assert_eq!(
            parameters
                .get("lane_guidance_lane_count")
                .map(String::as_str),
            Some("2")
        );
    }

    #[test]
    fn link_edge_uses_ramp_instruction() {
        let mut ramp_edge = edge(2, "A4 ramp");
        ramp_edge.is_link = true;
        ramp_edge.destination = Some("Krakow".to_string());
        ramp_edge.destination_ref = Some("A4".to_string());
        ramp_edge.junction_ref = Some("7A".to_string());
        let leg = RouteLeg {
            distance_m: 200.0,
            duration_s: 24.0,
            geometry: vec![
                GeoPoint::new(50.0, 20.0),
                GeoPoint::new(50.0, 20.001),
                GeoPoint::new(50.001, 20.001),
            ],
            edges: vec![edge(1, "Opolska"), ramp_edge],
            shape_segments: vec![
                RouteShapeSegment {
                    edge_id: 1,
                    begin_shape_index: 0,
                    end_shape_index: 1,
                    start_fraction: 0.0,
                    end_fraction: 1.0,
                    quality_flags: Vec::new(),
                    dynamic_overlay: None,
                },
                RouteShapeSegment {
                    edge_id: 2,
                    begin_shape_index: 1,
                    end_shape_index: 2,
                    start_fraction: 0.0,
                    end_fraction: 1.0,
                    quality_flags: Vec::new(),
                    dynamic_overlay: None,
                },
            ],
        };

        let maneuvers = build_maneuvers(&leg);
        let ramp = maneuvers
            .iter()
            .find(|maneuver| maneuver.instruction_key == "route.ramp")
            .expect("ramp maneuver");

        assert_eq!(ramp.kind, RouteManeuverKind::Turn);
        assert_eq!(ramp.instruction, "Take exit 7A onto A4 ramp toward Krakow");
        assert_eq!(
            ramp.instruction_parameters
                .get("guidance_kind")
                .map(String::as_str),
            Some("ramp")
        );
        assert_eq!(
            ramp.instruction_parameters
                .get("road_use")
                .map(String::as_str),
            Some("link")
        );
        assert_eq!(ramp.sign_text.as_deref(), Some("Krakow"));
        assert_eq!(ramp.branch_text.as_deref(), Some("A4"));
        assert_eq!(
            ramp.instruction_parameters
                .get("junction_ref")
                .map(String::as_str),
            Some("7A")
        );
    }

    #[test]
    fn sign_guidance_does_not_duplicate_existing_road_name() {
        let mut signed_edge = edge(2, "Krakow");
        signed_edge.destination = Some("Krakow".to_string());

        assert_eq!(
            instruction_with_sign("Continue onto", signed_edge.name.as_deref(), &signed_edge),
            "Continue onto Krakow"
        );
    }

    #[test]
    fn endpoint_connector_generates_guidance_maneuver() {
        let leg = RouteLeg {
            distance_m: 130.0,
            duration_s: 30.0,
            geometry: vec![
                GeoPoint::new(50.0, 20.0),
                GeoPoint::new(50.0002, 20.0002),
                GeoPoint::new(50.0008, 20.0008),
            ],
            edges: vec![edge(10, "Opolska")],
            shape_segments: vec![
                RouteShapeSegment {
                    edge_id: u64::MAX - 1,
                    begin_shape_index: 0,
                    end_shape_index: 1,
                    start_fraction: 0.0,
                    end_fraction: 1.0,
                    quality_flags: Vec::new(),
                    dynamic_overlay: Some(RouteShapeDynamicOverlay {
                        kind: RouteShapeDynamicOverlayKind::EndpointConnector,
                        connector_id: Some("origin-connector".to_string()),
                        connector_type: Some(RouteDynamicVirtualEdgeKind::Endpoint),
                        source_chunk_ids: vec!["chunk-a".to_string()],
                        distance_meters: Some(28.0),
                        cost: Some(42.0),
                        source_endpoint: Some(RouteDynamicEndpointKind::Origin),
                        source_point: Some(GeoPoint::new(50.0, 20.0)),
                        source_edge_id: None,
                        source_fraction: None,
                        target_endpoint: None,
                        target_point: None,
                        target_edge_id: Some(10),
                        target_fraction: Some(0.2),
                    }),
                },
                RouteShapeSegment {
                    edge_id: 10,
                    begin_shape_index: 1,
                    end_shape_index: 2,
                    start_fraction: 0.2,
                    end_fraction: 1.0,
                    quality_flags: Vec::new(),
                    dynamic_overlay: None,
                },
            ],
        };

        let maneuvers = build_maneuvers(&leg);
        let connector = maneuvers
            .iter()
            .find(|maneuver| {
                maneuver
                    .quality_flags
                    .contains(&RouteGuidanceQualityFlag::SsvgConnector)
            })
            .expect("connector maneuver");

        assert_eq!(connector.instruction_key, "route.ssvg_connector");
        assert_eq!(
            connector.instruction,
            "Follow SSVG connector from origin to the road network"
        );
        assert_eq!(connector.begin_shape_index, 0);
        assert_eq!(
            connector
                .instruction_parameters
                .get("connector_id")
                .map(String::as_str),
            Some("origin-connector")
        );
        assert_eq!(
            connector
                .instruction_parameters
                .get("source_endpoint")
                .map(String::as_str),
            Some("origin")
        );
        assert_eq!(
            connector
                .instruction_parameters
                .get("target_edge_id")
                .map(String::as_str),
            Some("10")
        );
        assert!(
            maneuvers
                .first()
                .is_some_and(|maneuver| maneuver.kind == RouteManeuverKind::Start)
        );
    }

    #[test]
    fn road_to_road_bypass_connector_gets_specific_instruction() {
        let leg = RouteLeg {
            distance_m: 180.0,
            duration_s: 45.0,
            geometry: vec![
                GeoPoint::new(50.0, 20.0),
                GeoPoint::new(50.0002, 20.0002),
                GeoPoint::new(50.0004, 20.0004),
            ],
            edges: vec![edge(20, "Road before"), edge(21, "Road after")],
            shape_segments: vec![RouteShapeSegment {
                edge_id: u64::MAX - 2,
                begin_shape_index: 1,
                end_shape_index: 2,
                start_fraction: 0.0,
                end_fraction: 1.0,
                quality_flags: Vec::new(),
                dynamic_overlay: Some(RouteShapeDynamicOverlay {
                    kind: RouteShapeDynamicOverlayKind::RoadToRoadBypassConnector,
                    connector_id: Some("bypass-1".to_string()),
                    connector_type: Some(RouteDynamicVirtualEdgeKind::RoadToRoadBypass),
                    source_chunk_ids: vec!["chunk-b".to_string()],
                    distance_meters: Some(55.0),
                    cost: Some(75.0),
                    source_endpoint: None,
                    source_point: None,
                    source_edge_id: Some(20),
                    source_fraction: Some(0.45),
                    target_endpoint: None,
                    target_point: None,
                    target_edge_id: Some(21),
                    target_fraction: Some(0.65),
                }),
            }],
        };

        let maneuvers = build_maneuvers(&leg);
        let connector = maneuvers
            .iter()
            .find(|maneuver| {
                maneuver
                    .quality_flags
                    .contains(&RouteGuidanceQualityFlag::SsvgConnector)
            })
            .expect("connector maneuver");

        assert_eq!(
            connector.instruction,
            "Follow SSVG bypass around blocked road"
        );
        assert_eq!(
            connector
                .instruction_parameters
                .get("connector_type")
                .map(String::as_str),
            Some("road_to_road_bypass")
        );
    }
}

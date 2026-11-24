use crate::costing::{
    CostingPolicy, EdgeCostClasses, EdgeCostDecision, selected_vehicle_speed_source,
};
use crate::graph_view::{GraphViewEdge, PackageGraphView};
use crate::guidance::build_maneuvers_with;
use crate::search::{RouteSearchStep, SearchRoute};
use crate::types::{
    GeoPoint, LegResult, MobilityProfile, RouteCostBreakdown, RouteCostDriverKind,
    RouteCostDriverSummary, RouteCostSummary, RouteEdgeAttribution, RouteError, RouteLeg,
    RouteOptions, RoutePolicySummary, RouteRecostSummary, RouteShapeDynamicOverlay,
    RouteShapeDynamicOverlayKind, RouteShapeQualityFlag, RouteShapeSegment, RouteStatus,
};
use rayon::prelude::*;

const RECOST_PROFILES: [MobilityProfile; 5] = [
    MobilityProfile::Wheeled,
    MobilityProfile::Tracked,
    MobilityProfile::Emergency,
    MobilityProfile::OnFoot,
    MobilityProfile::Legged,
];

pub(crate) fn build_route_result(
    access: &PackageGraphView,
    route: SearchRoute,
    profile: MobilityProfile,
    options: &RouteOptions,
) -> Result<LegResult, RouteError> {
    if !route.steps.is_empty() {
        return build_route_result_from_steps(access, route, profile, options);
    }
    let mut geometry = Vec::new();
    let mut edges = Vec::new();
    let mut shape_segments = Vec::new();
    let costing = CostingPolicy::new(profile, options);
    let views = LegViews::of(access);
    let mut scratch = GeometryScratch::default();
    let records = access
        .edge_records_for_path(&route.edge_indexes, &route.edge_ids)
        .into_iter()
        .zip(&route.edge_ids)
        .map(|(record, edge_id)| {
            record.ok_or(RouteError::MissingEdgeMetadata { edge_id: *edge_id })
        })
        .collect::<Result<Vec<_>, _>>()?;

    let classes = records
        .iter()
        .map(|record| EdgeCostClasses::from_shared_info(&record.shared_info))
        .collect::<Vec<_>>();
    let edge_count = records.len();
    edges.reserve_exact(edge_count);
    shape_segments.reserve_exact(edge_count);
    // The origin seed may traverse nothing (an origin at the end of its edge),
    // so the path can start on the next edge, which it then crosses whole.
    let starts_on_origin_edge = route.edge_ids.first() == Some(&route.origin.edge_id);
    let mut recost_steps = Vec::with_capacity(edge_count);
    // The previous record's junction speed, once computed.
    let (mut previous_speed, mut speed) = (None::<f64>, None::<f64>);
    for (index, record) in records.iter().enumerate() {
        let start_fraction = if index == 0 && starts_on_origin_edge {
            route.origin.fraction_along_edge
        } else {
            0.0
        };
        let end_fraction = if index + 1 == edge_count {
            route.destination.fraction_along_edge
        } else {
            1.0
        };
        let append_outcome = append_edge_geometry(
            &mut geometry,
            record,
            start_fraction,
            end_fraction,
            &mut scratch,
        );
        let mut quality_flags = Vec::new();
        if index == 0 && start_fraction > f64::EPSILON {
            quality_flags.push(RouteShapeQualityFlag::ClippedOriginEdge);
        }
        if index + 1 == edge_count && end_fraction < 1.0 - f64::EPSILON {
            quality_flags.push(RouteShapeQualityFlag::ClippedDestinationEdge);
        }
        if edge_count == 1 && start_fraction > f64::EPSILON && end_fraction < 1.0 - f64::EPSILON {
            quality_flags.push(RouteShapeQualityFlag::SameEdgeDirect);
        }
        if append_outcome.duplicated_edge_suppressed {
            quality_flags.push(RouteShapeQualityFlag::DuplicatedEdgeSuppressed);
        }
        if append_outcome.shape_fallback_used {
            quality_flags.push(RouteShapeQualityFlag::ShapeFallbackUsed);
        }
        shape_segments.push(RouteShapeSegment {
            edge_id: record.edge.edge_id,
            begin_shape_index: append_outcome.begin_shape_index,
            end_shape_index: append_outcome.end_shape_index,
            start_fraction,
            end_fraction,
            quality_flags,
            dynamic_overlay: None,
        });
        edges.push(edge_attribution(
            access,
            &views,
            &costing,
            profile,
            options,
            (record, classes[index]),
            index
                .checked_sub(1)
                .map(|previous| (&records[previous], classes[previous])),
            start_fraction,
            end_fraction,
            (&mut previous_speed, &mut speed),
        ));
        previous_speed = speed.take();
        recost_steps.push(RecostStep::Edge {
            record: index,
            start_fraction,
            end_fraction,
        });
    }

    let mut diagnostics = route.diagnostics;
    diagnostics.final_path_edge_count = route.edge_ids.len();
    let leg = RouteLeg {
        distance_m: route.distance_m,
        duration_s: route.duration_s,
        geometry: geometry.clone(),
        polyline6: None,
        geojson: None,
        edges,
        shape_segments,
    };
    let recosts = build_recost_summaries(&records, &classes, &recost_steps, options);
    let maneuvers = build_maneuvers_with(&leg, Some(&scratch.geometry_segment_m));
    let cost_summary = route_cost_summary(&leg);
    let pieces = route_pieces(&records, &recost_steps);
    let qualified_access = crate::sar::qualified_access(&pieces, profile, options);
    let sar = crate::sar::route_report(access, &pieces, profile, options);

    let mut result = LegResult {
        status: RouteStatus::Routed,
        origin: route.origin,
        destination: route.destination,
        edge_ids: route.edge_ids,
        distance_m: route.distance_m,
        cost: route.cost,
        duration_s: route.duration_s,
        geometry,
        polyline6: None,
        geojson: None,
        legs: vec![leg],
        maneuvers,
        alternatives: Vec::new(),
        recosts,
        policy_summary: route_policy_summary(profile, options),
        cost_summary,
        diagnostics,
        qualified_access,
        sar,
        off_network: None,
    };
    crate::access::add_access_stretches(&mut result);
    Ok(result)
}

/// The package views leg building reads per edge, built once per leg.
struct LegViews<'a> {
    request: crate::package::request::RequestView<'a>,
    sections: wayfarer_package::format::Sections<'a>,
}

impl<'a> LegViews<'a> {
    fn of(access: &'a PackageGraphView) -> Self {
        Self {
            request: access.request(),
            sections: access.routing_package().file().view(),
        }
    }
}

/// The traversed graph-edge pieces of a leg, in order, for the access and
/// SAR reports (a piece after a connector starts a new restriction context).
fn route_pieces<'r>(
    records: &'r [GraphViewEdge],
    steps: &[RecostStep],
) -> Vec<crate::sar::RoutePiece<'r>> {
    let mut pieces = Vec::with_capacity(steps.len());
    let mut after_connector = false;
    for step in steps {
        match *step {
            RecostStep::Edge {
                record,
                start_fraction,
                end_fraction,
            } => {
                pieces.push(crate::sar::RoutePiece {
                    record: &records[record],
                    start_fraction,
                    end_fraction,
                    after_connector,
                });
                after_connector = false;
            }
            RecostStep::Connector { .. } => after_connector = true,
        }
    }
    pieces
}

fn build_route_result_from_steps(
    access: &PackageGraphView,
    route: SearchRoute,
    profile: MobilityProfile,
    options: &RouteOptions,
) -> Result<LegResult, RouteError> {
    let mut geometry = Vec::new();
    let mut edges = Vec::new();
    let mut shape_segments = Vec::new();
    let costing = CostingPolicy::new(profile, options);
    let views = LegViews::of(access);
    let mut scratch = GeometryScratch::default();
    let mut records = Vec::new();
    let mut classes = Vec::<EdgeCostClasses>::new();
    let mut recost_steps = Vec::with_capacity(route.steps.len());
    edges.reserve_exact(route.steps.len());
    shape_segments.reserve_exact(route.steps.len());
    records.reserve_exact(route.steps.len());
    // The record the route arrived on, unless it crossed a connector since.
    let mut previous_record = None::<usize>;
    // Its junction speed, once computed.
    let (mut previous_speed, mut speed) = (None::<f64>, None::<f64>);
    for (step_index, step) in route.steps.iter().enumerate() {
        match step {
            RouteSearchStep::GraphEdge {
                edge_id,
                start_fraction,
                end_fraction,
                quality_flags,
            } => {
                let edge_index = route.edge_indexes.get(step_index).copied().flatten();
                let record = access
                    .edge_records_for_path(&[edge_index], &[*edge_id])
                    .pop()
                    .flatten()
                    .ok_or(RouteError::MissingEdgeMetadata { edge_id: *edge_id })?;
                let append_outcome = append_edge_geometry(
                    &mut geometry,
                    &record,
                    *start_fraction,
                    *end_fraction,
                    &mut scratch,
                );
                let mut segment_flags = quality_flags.clone();
                append_endpoint_shape_flags(
                    &mut segment_flags,
                    record.edge.edge_id,
                    *start_fraction,
                    *end_fraction,
                    &route,
                );
                if append_outcome.duplicated_edge_suppressed {
                    segment_flags.push(RouteShapeQualityFlag::DuplicatedEdgeSuppressed);
                }
                if append_outcome.shape_fallback_used {
                    segment_flags.push(RouteShapeQualityFlag::ShapeFallbackUsed);
                }
                let record_classes = EdgeCostClasses::from_shared_info(&record.shared_info);
                let attribution = edge_attribution(
                    access,
                    &views,
                    &costing,
                    profile,
                    options,
                    (&record, record_classes),
                    previous_record.map(|previous| (&records[previous], classes[previous])),
                    *start_fraction,
                    *end_fraction,
                    (&mut previous_speed, &mut speed),
                );
                previous_speed = speed.take();
                let dynamic_overlay = segment_flags
                    .contains(&RouteShapeQualityFlag::SsvgPartialEdgeSplit)
                    .then_some(RouteShapeDynamicOverlay {
                        kind: RouteShapeDynamicOverlayKind::PartialEdgeFragment,
                        connector_id: None,
                        connector_type: None,
                        source_chunk_ids: Vec::new(),
                        distance_meters: Some(attribution.distance_m),
                        duration_seconds: None,
                        cost: Some(
                            attribution.cost_breakdown.edge_cost_without_turn()
                                * traversed_fraction(*start_fraction, *end_fraction),
                        ),
                        source_endpoint: None,
                        source_point: None,
                        source_edge_id: Some(record.edge.edge_id),
                        source_fraction: Some(*start_fraction),
                        target_endpoint: None,
                        target_point: None,
                        target_edge_id: Some(record.edge.edge_id),
                        target_fraction: Some(*end_fraction),
                    });
                shape_segments.push(RouteShapeSegment {
                    edge_id: record.edge.edge_id,
                    begin_shape_index: append_outcome.begin_shape_index,
                    end_shape_index: append_outcome.end_shape_index,
                    start_fraction: *start_fraction,
                    end_fraction: *end_fraction,
                    quality_flags: segment_flags,
                    dynamic_overlay,
                });
                edges.push(attribution);
                records.push(record);
                classes.push(record_classes);
                previous_record = Some(records.len() - 1);
                recost_steps.push(RecostStep::Edge {
                    record: records.len() - 1,
                    start_fraction: *start_fraction,
                    end_fraction: *end_fraction,
                });
            }
            RouteSearchStep::VirtualConnector {
                edge_id,
                connector_id,
                connector_type,
                geometry: connector_geometry,
                distance_m,
                cost,
                source_chunk_ids,
                source_endpoint,
                source_point,
                source_edge_id,
                source_fraction,
                target_endpoint,
                target_point,
                target_edge_id,
                target_fraction,
            } => {
                let begin_shape_index = geometry.len();
                let mut end_shape_index = begin_shape_index;
                for point in connector_geometry {
                    if geometry
                        .last()
                        .is_some_and(|last| last.distance_to(*point) < 0.01)
                    {
                        continue;
                    }
                    geometry.push(*point);
                    end_shape_index = geometry.len().saturating_sub(1);
                }
                shape_segments.push(RouteShapeSegment {
                    edge_id: *edge_id,
                    begin_shape_index,
                    end_shape_index,
                    start_fraction: 0.0,
                    end_fraction: 1.0,
                    quality_flags: vec![RouteShapeQualityFlag::SsvgConnector],
                    dynamic_overlay: Some(RouteShapeDynamicOverlay {
                        kind: connector_segment_kind(*connector_type),
                        connector_id: Some(connector_id.clone()),
                        connector_type: Some(*connector_type),
                        source_chunk_ids: source_chunk_ids.clone(),
                        distance_meters: Some(*distance_m),
                        duration_seconds: None,
                        cost: Some(*cost),
                        source_endpoint: *source_endpoint,
                        source_point: *source_point,
                        source_edge_id: *source_edge_id,
                        source_fraction: *source_fraction,
                        target_endpoint: *target_endpoint,
                        target_point: *target_point,
                        target_edge_id: *target_edge_id,
                        target_fraction: *target_fraction,
                    }),
                });
                previous_record = None;
                previous_speed = None;
                recost_steps.push(RecostStep::Connector {
                    distance_m: *distance_m,
                    cost: *cost,
                });
            }
        }
    }

    let mut diagnostics = route.diagnostics;
    record_used_dynamic_overlay_segments(&mut diagnostics, &shape_segments);
    diagnostics.final_path_edge_count = route.edge_ids.len();
    let leg = RouteLeg {
        distance_m: route.distance_m,
        duration_s: route.duration_s,
        geometry: geometry.clone(),
        polyline6: None,
        geojson: None,
        edges,
        shape_segments,
    };
    let recosts = build_recost_summaries(&records, &classes, &recost_steps, options);
    // Connector points are pushed without their lengths: guidance computes them.
    let maneuvers = build_maneuvers_with(&leg, None);
    let cost_summary = route_cost_summary(&leg);
    let pieces = route_pieces(&records, &recost_steps);
    let qualified_access = crate::sar::qualified_access(&pieces, profile, options);
    let sar = crate::sar::route_report(access, &pieces, profile, options);
    let mut result = LegResult {
        status: RouteStatus::Routed,
        origin: route.origin,
        destination: route.destination,
        edge_ids: route.edge_ids,
        distance_m: route.distance_m,
        cost: route.cost,
        duration_s: route.duration_s,
        geometry,
        polyline6: None,
        geojson: None,
        legs: vec![leg],
        maneuvers,
        alternatives: Vec::new(),
        recosts,
        policy_summary: route_policy_summary(profile, options),
        cost_summary,
        diagnostics,
        qualified_access,
        sar,
        off_network: None,
    };
    crate::access::add_access_stretches(&mut result);
    Ok(result)
}

/// The attribution of one traversed piece (`start_fraction..end_fraction`) of
/// a graph edge. `previous` is the graph edge the route arrived on, when it
/// came through a junction (not across an overlay connector): it sets the
/// turn penalty and the junction counts.
#[allow(clippy::too_many_arguments)]
fn edge_attribution(
    access: &PackageGraphView,
    ctx: &LegViews<'_>,
    costing: &CostingPolicy<'_>,
    profile: MobilityProfile,
    options: &RouteOptions,
    (record, record_classes): (&GraphViewEdge, EdgeCostClasses),
    previous: Option<(&GraphViewEdge, EdgeCostClasses)>,
    start_fraction: f64,
    end_fraction: f64,
    (previous_speed, speed): (&mut Option<f64>, &mut Option<f64>),
) -> RouteEdgeAttribution {
    let warning_ranges = options
        .warning_edge_ranges
        .iter()
        .filter(|range| range.edge_id == record.edge.edge_id)
        .filter_map(|range| range.clipped_to(start_fraction, end_fraction))
        .collect::<Vec<_>>();
    let traversed_fraction = traversed_fraction(start_fraction, end_fraction);
    // Two fragments of one split edge meet mid-edge, not at a junction.
    let junction = previous.filter(|(previous, _)| previous.edge.edge_id != record.edge.edge_id);
    let (junction_outgoing_edge_count, junction_incoming_edge_count) = match junction {
        None => (0, 0),
        Some((previous, _)) => {
            let outgoing_excluded = [previous.edge.shared_edge_info_id];
            let incoming_excluded = [
                previous.edge.shared_edge_info_id,
                record.edge.shared_edge_info_id,
            ];
            let (outgoing, incoming) = access.junction_shared_info_counts_in(
                &ctx.request,
                Some(record.raw_edge_index),
                record.source.id,
                &outgoing_excluded,
                &incoming_excluded,
            );
            (outgoing.unwrap_or_default(), incoming.unwrap_or_default())
        }
    };
    let (junction_branch_angles_deg, junction_joining_angles_deg) = junction
        .map(|(previous, _)| {
            access.junction_branch_angles_in(&ctx.sections, previous, record, profile)
        })
        .unwrap_or_default();
    let mut cost_breakdown =
        edge_cost_breakdown(costing, record, record_classes).unwrap_or_default();
    if let Some((previous, _)) = junction {
        cost_breakdown.turn_penalty = costing.transition_cost_for_records(previous, record);
    }
    let junction_delay_s = junction.map_or(0.0, |(previous, previous_classes)| {
        costing.junction_delay_for_records_memo(
            (previous, previous_classes, previous_speed),
            (record, record_classes, speed),
        )
    });
    cost_breakdown.total_cost =
        cost_breakdown.edge_cost_without_turn() * traversed_fraction + cost_breakdown.turn_penalty;

    RouteEdgeAttribution {
        edge_id: record.edge.edge_id,
        shared_edge_info_id: record.edge.shared_edge_info_id,
        road_type: record.shared_info.road_type,
        name: record
            .shared_info
            .name
            .as_deref()
            .or(record.shared_info.ref_name.as_deref())
            .map(str::to_string),
        ref_name: record.shared_info.ref_name.as_deref().map(str::to_string),
        route_refs: record.shared_info.route_refs.clone(),
        surface: record.shared_info.surface.as_deref().map(str::to_string),
        tracktype: record.shared_info.tracktype.as_deref().map(str::to_string),
        smoothness: record.shared_info.smoothness.as_deref().map(str::to_string),
        maxspeed_kph: record.shared_info.maxspeed_kph,
        inferred_speed_kph: record.edge.inferred_speed_kph,
        speed_source: Some(selected_vehicle_speed_source(
            &record.edge,
            &record.shared_info,
            &options.costing,
        )),
        lane_count: record.shared_info.lane_count,
        lanes: record.shared_info.lanes.as_deref().map(str::to_string),
        turn_lanes: record.shared_info.turn_lanes.as_deref().map(str::to_string),
        turn_lanes_forward: record
            .shared_info
            .turn_lanes_forward
            .as_deref()
            .map(str::to_string),
        turn_lanes_backward: record
            .shared_info
            .turn_lanes_backward
            .as_deref()
            .map(str::to_string),
        lane_connectivity_available: record.shared_info.lane_connectivity_available,
        speed_source_provenance: record
            .shared_info
            .speed_source
            .as_deref()
            .map(str::to_string),
        free_flow_speed_kph: record.shared_info.free_flow_speed_kph,
        constrained_speed_kph: record.shared_info.constrained_speed_kph,
        admin_country_id: record
            .shared_info
            .admin_country_id
            .as_deref()
            .map(str::to_string),
        admin_region_id: record
            .shared_info
            .admin_region_id
            .as_deref()
            .map(str::to_string),
        timezone_id: record
            .shared_info
            .timezone_id
            .as_deref()
            .map(str::to_string),
        conditional_restrictions: record.shared_info.conditional_restrictions.clone(),
        opposing_edge_id: record.edge.opposing_edge_id,
        layer: record.shared_info.layer,
        level: record.shared_info.level.as_deref().map(str::to_string),
        incline_percent: record.shared_info.incline_percent.map(|percent| {
            match record.edge.traversal_direction {
                wayfarer_package::TraversalDirection::Forward => percent,
                wayfarer_package::TraversalDirection::Backward => percent.saturating_neg(),
            }
        }),
        destination: record
            .shared_info
            .destination
            .as_deref()
            .map(str::to_string),
        destination_ref: record
            .shared_info
            .destination_ref
            .as_deref()
            .map(str::to_string),
        destination_street: record
            .shared_info
            .destination_street
            .as_deref()
            .map(str::to_string),
        junction_ref: record
            .shared_info
            .junction_ref
            .as_deref()
            .map(str::to_string),
        is_bridge: record.shared_info.is_bridge,
        is_tunnel: record.shared_info.is_tunnel,
        has_ford: record.shared_info.has_ford,
        is_toll: record.shared_info.is_toll,
        is_ferry: record.shared_info.is_ferry,
        has_gate: record.shared_info.has_gate,
        is_access_restricted: record.shared_info.is_access_restricted,
        is_destination_only: record.shared_info.is_destination_only,
        is_link: record.shared_info.is_link,
        is_roundabout: record.shared_info.is_roundabout,
        roundabout_exit_count: record
            .source
            .roundabout_exit_count
            .max(record.target.roundabout_exit_count),
        has_warning_overlay: options.warning_edge_ids.contains(&record.edge.edge_id)
            || !warning_ranges.is_empty(),
        warning_ranges,
        junction_outgoing_edge_count,
        junction_incoming_edge_count,
        junction_branch_angles_deg,
        junction_joining_angles_deg,
        distance_m: f64::from(record.edge.distance_m) * traversed_fraction,
        duration_s: costing.edge_duration_s_with_classes(record, record_classes)
            * traversed_fraction,
        junction_delay_s,
        traversal_direction: record.edge.traversal_direction,
        cost_breakdown,
    }
}

/// The attribution of each traversed piece `(edge, start_fraction,
/// end_fraction)` of a path that is not a route (a map match's sub-path), as a
/// leg's `edges` are built, costed with `options`. A piece follows the one
/// before through a junction only when it starts at its edge's start and the
/// one before ends at its edge's end; otherwise (a U-turn mid-edge) it has no
/// turn and no junction counts.
pub(crate) fn path_edge_attributions(
    access: &PackageGraphView,
    profile: MobilityProfile,
    options: &RouteOptions,
    pieces: &[(u32, f64, f64)],
) -> Result<Vec<RouteEdgeAttribution>, RouteError> {
    let costing = CostingPolicy::new(profile, options);
    let views = LegViews::of(access);
    let indexes = pieces
        .iter()
        .map(|(edge, _, _)| Some(*edge as usize))
        .collect::<Vec<_>>();
    let edge_ids = pieces
        .iter()
        .map(|(edge, _, _)| access.package().view().edge_id(*edge as usize))
        .collect::<Vec<_>>();
    let records = access
        .edge_records_for_path(&indexes, &edge_ids)
        .into_iter()
        .zip(&edge_ids)
        .map(|(record, edge_id)| {
            record.ok_or(RouteError::MissingEdgeMetadata { edge_id: *edge_id })
        })
        .collect::<Result<Vec<_>, _>>()?;
    let classes = records
        .iter()
        .map(|record| EdgeCostClasses::from_shared_info(&record.shared_info))
        .collect::<Vec<_>>();
    let (mut previous_speed, mut speed) = (None::<f64>, None::<f64>);
    let mut edges = Vec::with_capacity(pieces.len());
    for (index, (record, &(_, start_fraction, end_fraction))) in
        records.iter().zip(pieces).enumerate()
    {
        let through_junction = index > 0
            && start_fraction <= f64::EPSILON
            && pieces[index - 1].2 >= 1.0 - f64::EPSILON;
        edges.push(edge_attribution(
            access,
            &views,
            &costing,
            profile,
            options,
            (record, classes[index]),
            through_junction.then(|| (&records[index - 1], classes[index - 1])),
            start_fraction,
            end_fraction,
            (&mut previous_speed, &mut speed),
        ));
        previous_speed = speed.take();
    }
    Ok(edges)
}

fn traversed_fraction(start_fraction: f64, end_fraction: f64) -> f64 {
    (end_fraction.clamp(0.0, 1.0) - start_fraction.clamp(0.0, 1.0))
        .abs()
        .clamp(0.0, 1.0)
}

fn connector_segment_kind(
    connector_type: crate::types::RouteDynamicVirtualEdgeKind,
) -> RouteShapeDynamicOverlayKind {
    match connector_type {
        crate::types::RouteDynamicVirtualEdgeKind::Endpoint => {
            RouteShapeDynamicOverlayKind::EndpointConnector
        }
        crate::types::RouteDynamicVirtualEdgeKind::RoadEntry => {
            RouteShapeDynamicOverlayKind::RoadEntryConnector
        }
        crate::types::RouteDynamicVirtualEdgeKind::RoadToRoadBypass => {
            RouteShapeDynamicOverlayKind::RoadToRoadBypassConnector
        }
        crate::types::RouteDynamicVirtualEdgeKind::CrossCountry => {
            RouteShapeDynamicOverlayKind::CrossCountryConnector
        }
    }
}

fn record_used_dynamic_overlay_segments(
    diagnostics: &mut crate::types::RouteDiagnostics,
    shape_segments: &[RouteShapeSegment],
) {
    for segment in shape_segments {
        let Some(dynamic_overlay) = &segment.dynamic_overlay else {
            continue;
        };
        match dynamic_overlay.kind {
            RouteShapeDynamicOverlayKind::PartialEdgeFragment => {
                diagnostics.ssvg_partial_edge_fragment_used_count += 1;
            }
            RouteShapeDynamicOverlayKind::EndpointConnector => {
                diagnostics.ssvg_connector_used_count += 1;
                diagnostics.ssvg_endpoint_connector_used_count += 1;
                record_connector_distance_and_cost(diagnostics, dynamic_overlay);
            }
            RouteShapeDynamicOverlayKind::RoadEntryConnector => {
                diagnostics.ssvg_connector_used_count += 1;
                diagnostics.ssvg_road_entry_connector_used_count += 1;
                record_connector_distance_and_cost(diagnostics, dynamic_overlay);
            }
            RouteShapeDynamicOverlayKind::RoadToRoadBypassConnector => {
                diagnostics.ssvg_connector_used_count += 1;
                diagnostics.ssvg_road_bypass_connector_used_count += 1;
                record_connector_distance_and_cost(diagnostics, dynamic_overlay);
            }
            RouteShapeDynamicOverlayKind::CrossCountryConnector => {
                diagnostics.ssvg_connector_used_count += 1;
                diagnostics.ssvg_cross_country_connector_used_count += 1;
                record_connector_distance_and_cost(diagnostics, dynamic_overlay);
            }
            // Access stretches are added after the leg's diagnostics.
            RouteShapeDynamicOverlayKind::AccessLeg => {}
        }
    }
}

fn record_connector_distance_and_cost(
    diagnostics: &mut crate::types::RouteDiagnostics,
    dynamic_overlay: &RouteShapeDynamicOverlay,
) {
    diagnostics.ssvg_connector_distance_meters += dynamic_overlay
        .distance_meters
        .unwrap_or_default()
        .max(0.0)
        .round() as u64;
    diagnostics.ssvg_connector_cost +=
        dynamic_overlay.cost.unwrap_or_default().max(0.0).round() as u64;
}

fn route_cost_summary(leg: &RouteLeg) -> RouteCostSummary {
    let mut summary = RouteCostSummary {
        attributed_edge_count: leg.edges.len(),
        ..Default::default()
    };
    let mut driver_costs = [
        (RouteCostDriverKind::ProfileTime, 0.0),
        (RouteCostDriverKind::RoadType, 0.0),
        (RouteCostDriverKind::Surface, 0.0),
        (RouteCostDriverKind::TrackQuality, 0.0),
        (RouteCostDriverKind::RoadUse, 0.0),
        (RouteCostDriverKind::Overlay, 0.0),
        (RouteCostDriverKind::Avoidance, 0.0),
        (RouteCostDriverKind::Turns, 0.0),
    ];

    for breakdown in leg.edges.iter().map(|edge| edge.cost_breakdown) {
        summary.costed_edge_count += 1;
        summary.total_cost += breakdown.total_cost;
        summary.turn_penalty_total += breakdown.turn_penalty;

        if breakdown.profile_time_factor > 1.0 {
            summary.profile_time_penalty_count += 1;
        }
        summary.max_profile_time_factor = summary
            .max_profile_time_factor
            .max(breakdown.profile_time_factor);

        if (breakdown.overlay_multiplier - 1.0).abs() > f64::EPSILON {
            summary.overlay_multiplier_count += 1;
        }
        summary.max_overlay_multiplier = summary
            .max_overlay_multiplier
            .max(breakdown.overlay_multiplier);

        if breakdown.avoided_edge_penalty > 1.0 {
            summary.avoided_penalty_count += 1;
        }

        let road_use_penalty = road_use_penalty_factor(breakdown);
        if road_use_penalty > 1.0 {
            summary.road_use_penalty_count += 1;
        }
        summary.max_road_use_penalty = summary.max_road_use_penalty.max(road_use_penalty);

        add_driver_cost(
            &mut driver_costs,
            RouteCostDriverKind::ProfileTime,
            positive_extra_cost(breakdown.base_distance_cost, breakdown.profile_time_factor),
        );
        add_driver_cost(
            &mut driver_costs,
            RouteCostDriverKind::RoadType,
            positive_extra_cost(
                breakdown.base_distance_cost * breakdown.profile_time_factor,
                breakdown.road_type_factor,
            ),
        );
        add_driver_cost(
            &mut driver_costs,
            RouteCostDriverKind::Surface,
            positive_extra_cost(breakdown.base_distance_cost, breakdown.surface_factor),
        );
        add_driver_cost(
            &mut driver_costs,
            RouteCostDriverKind::TrackQuality,
            positive_extra_cost(breakdown.base_distance_cost, breakdown.tracktype_factor)
                + positive_extra_cost(breakdown.base_distance_cost, breakdown.smoothness_factor),
        );
        add_driver_cost(
            &mut driver_costs,
            RouteCostDriverKind::RoadUse,
            positive_extra_cost(breakdown.base_distance_cost, road_use_penalty),
        );
        add_driver_cost(
            &mut driver_costs,
            RouteCostDriverKind::Overlay,
            positive_extra_cost(breakdown.base_distance_cost, breakdown.overlay_multiplier),
        );
        add_driver_cost(
            &mut driver_costs,
            RouteCostDriverKind::Avoidance,
            positive_extra_cost(breakdown.base_distance_cost, breakdown.avoided_edge_penalty),
        );
        add_driver_cost(
            &mut driver_costs,
            RouteCostDriverKind::Turns,
            breakdown.turn_penalty,
        );
    }

    summary.conditional_restriction_edge_count = leg
        .edges
        .iter()
        .filter(|edge| !edge.conditional_restrictions.is_empty())
        .count();

    summary.dominant_driver = driver_costs
        .into_iter()
        .filter(|(_, cost)| *cost > f64::EPSILON)
        .max_by(|(_, left), (_, right)| left.total_cmp(right))
        .map(|(kind, cost)| RouteCostDriverSummary { kind, cost });

    summary
}

fn add_driver_cost(
    driver_costs: &mut [(RouteCostDriverKind, f64)],
    kind: RouteCostDriverKind,
    cost: f64,
) {
    if let Some((_, total)) = driver_costs
        .iter_mut()
        .find(|(candidate, _)| *candidate == kind)
    {
        *total += cost;
    }
}

fn road_use_penalty_factor(breakdown: RouteCostBreakdown) -> f64 {
    breakdown.toll_penalty
        * breakdown.ferry_penalty
        * breakdown.gate_penalty
        * breakdown.access_restriction_penalty
        * breakdown.destination_only_penalty
}

fn positive_extra_cost(base_cost: f64, factor: f64) -> f64 {
    if !base_cost.is_finite() || !factor.is_finite() || base_cost <= 0.0 || factor <= 1.0 {
        return 0.0;
    }
    base_cost * (factor - 1.0)
}

fn route_policy_summary(profile: MobilityProfile, options: &RouteOptions) -> RoutePolicySummary {
    let costing = options.costing.normalized();
    let alternatives = options.alternatives.normalized();
    RoutePolicySummary {
        profile,
        max_visited_edge_count: options.max_visited_edge_count,
        blocked_edge_count: options
            .blocked_edge_ids
            .len()
            .saturating_add(options.blocked_edge_ranges.len()),
        avoided_edge_count: options.avoided_edge_ids.len(),
        avoided_range_count: options.avoided_edge_ranges.len(),
        warning_edge_count: options.warning_edge_ids.len(),
        cost_multiplier_edge_count: options
            .edge_cost_multipliers
            .len()
            .saturating_add(options.edge_cost_multiplier_ranges.len()),
        cost_multiplier_range_count: options.edge_cost_multiplier_ranges.len(),
        avoid_tracks: costing.avoid_tracks,
        avoid_paths: costing.avoid_paths,
        avoid_motorways: costing.avoid_motorways,
        avoid_destination_only: costing.avoid_destination_only,
        allow_ford: costing.allow_ford,
        avoid_tolls: costing.avoid_tolls,
        avoid_ferries: costing.avoid_ferries,
        avoid_gates: costing.avoid_gates,
        avoid_access_restricted: costing.avoid_access_restricted,
        prefer_paved: costing.prefer_paved,
        prefer_known_route_refs: costing.prefer_known_route_refs,
        emergency_override_strength: costing.emergency_override_strength,
        speed_preference: costing.speed_preference,
        fixed_speed_kph: costing.fixed_speed_kph,
        top_speed_kph: costing.top_speed_kph,
        conditional_restriction_eval_at_ms: costing.conditional_restriction_eval_at_ms,
        requested_alternative_count: alternatives.requested_alternative_count,
        max_shared_edge_ratio: alternatives.max_shared_edge_ratio,
        max_cost_ratio: alternatives.max_cost_ratio,
        minimum_warning_difference: alternatives.minimum_warning_difference,
        dynamic_overlay_split_edge_count: options
            .dynamic_overlay
            .as_ref()
            .map(|overlay| overlay.diagnostics.split_edge_count)
            .unwrap_or_default(),
        dynamic_overlay_partial_split_edge_count: options
            .dynamic_overlay
            .as_ref()
            .map(|overlay| overlay.diagnostics.partial_split_edge_count)
            .unwrap_or_default(),
        dynamic_overlay_virtual_connector_count: options
            .dynamic_overlay
            .as_ref()
            .map(|overlay| overlay.diagnostics.virtual_connector_count)
            .unwrap_or_default(),
        sar: options.sar.as_ref().map(crate::sar::RouteSarPolicy::of),
    }
}

/// One step of a leg as the recosts read it.
#[derive(Debug, Clone, Copy)]
enum RecostStep {
    /// `records[record]`, traversed from `start_fraction` to `end_fraction`.
    Edge {
        record: usize,
        start_fraction: f64,
        end_fraction: f64,
    },
    /// An overlay connector: its own distance and cost for every profile (the
    /// overlay gives no duration).
    Connector { distance_m: f64, cost: f64 },
}

fn build_recost_summaries(
    records: &[GraphViewEdge],
    classes: &[EdgeCostClasses],
    steps: &[RecostStep],
    options: &RouteOptions,
) -> Vec<RouteRecostSummary> {
    let recost = |profile| recost_profile(records, classes, steps, profile, options);
    // Handing five small jobs to the thread pool costs more than running them
    // (a pool wake-up per request); only long routes are worth splitting.
    if steps.len() < RECOST_PARALLEL_MIN_EDGES {
        RECOST_PROFILES.iter().copied().map(recost).collect()
    } else {
        RECOST_PROFILES.par_iter().copied().map(recost).collect()
    }
}

/// Route length (edges) from which the five recosts run in parallel.
const RECOST_PARALLEL_MIN_EDGES: usize = 768;

fn recost_profile(
    records: &[GraphViewEdge],
    classes: &[EdgeCostClasses],
    steps: &[RecostStep],
    profile: MobilityProfile,
    options: &RouteOptions,
) -> RouteRecostSummary {
    let costing = CostingPolicy::new(profile, options);
    let mut distance_m = 0.0;
    let mut cost = 0.0;
    let mut duration_s = 0.0;
    let mut blocked_edge_count = 0usize;
    let mut inaccessible_edge_count = 0usize;
    let mut destination_only_penalty_count = 0usize;
    let mut ford_penalty_count = 0usize;
    let mut toll_penalty_count = 0usize;
    let mut ferry_penalty_count = 0usize;
    let mut gate_penalty_count = 0usize;
    let mut access_restriction_penalty_count = 0usize;
    let mut overlay_multiplier_count = 0usize;
    let mut turn_penalty_total = 0.0;

    let mut previous = None::<(&GraphViewEdge, EdgeCostClasses)>;
    // The previous record's junction speed, once computed.
    let mut previous_speed = None::<f64>;
    for step in steps {
        let mut speed = None::<f64>;
        let (record, record_classes, start_fraction, end_fraction) = match *step {
            RecostStep::Connector {
                distance_m: connector_distance_m,
                cost: connector_cost,
            } => {
                distance_m += connector_distance_m;
                cost += connector_cost;
                previous = None;
                previous_speed = None;
                continue;
            }
            RecostStep::Edge {
                record,
                start_fraction,
                end_fraction,
            } => (
                &records[record],
                classes[record],
                start_fraction,
                end_fraction,
            ),
        };
        let traversed_fraction = traversed_fraction(start_fraction, end_fraction);
        distance_m += f64::from(record.edge.distance_m) * traversed_fraction;

        match costing.edge_cost_with_classes(record, record_classes) {
            EdgeCostDecision::Allowed(edge_cost) => {
                let mut edge_total = edge_cost.cost * traversed_fraction;
                duration_s += edge_cost.duration_s * traversed_fraction;
                if edge_cost.is_destination_only_penalized {
                    destination_only_penalty_count += 1;
                }
                if edge_cost.has_ford_penalty {
                    ford_penalty_count += 1;
                }
                if edge_cost.has_toll_penalty {
                    toll_penalty_count += 1;
                }
                if edge_cost.has_ferry_penalty {
                    ferry_penalty_count += 1;
                }
                if edge_cost.has_gate_penalty {
                    gate_penalty_count += 1;
                }
                if edge_cost.has_access_restriction_penalty {
                    access_restriction_penalty_count += 1;
                }
                if edge_cost.has_overlay_multiplier {
                    overlay_multiplier_count += 1;
                }
                if let Some((previous, previous_classes)) =
                    previous.filter(|(previous, _)| previous.edge.edge_id != record.edge.edge_id)
                {
                    let turn_penalty = costing.transition_cost_for_records(previous, record);
                    turn_penalty_total += turn_penalty;
                    edge_total += turn_penalty;
                    duration_s += costing.junction_delay_for_records_memo(
                        (previous, previous_classes, &mut previous_speed),
                        (record, record_classes, &mut speed),
                    );
                }
                cost += edge_total;
            }
            EdgeCostDecision::Blocked => {
                blocked_edge_count += 1;
            }
            EdgeCostDecision::NotAccessible => {
                inaccessible_edge_count += 1;
            }
        }
        previous = Some((record, record_classes));
        previous_speed = speed;
    }

    let accessible = blocked_edge_count == 0 && inaccessible_edge_count == 0;
    RouteRecostSummary {
        profile,
        accessible,
        distance_m,
        cost: accessible.then_some(cost),
        duration_s: accessible.then_some(duration_s),
        blocked_edge_count,
        inaccessible_edge_count,
        destination_only_penalty_count,
        ford_penalty_count,
        toll_penalty_count,
        ferry_penalty_count,
        gate_penalty_count,
        access_restriction_penalty_count,
        overlay_multiplier_count,
        turn_penalty_total,
    }
}

fn edge_cost_breakdown(
    costing: &CostingPolicy<'_>,
    record: &GraphViewEdge,
    classes: EdgeCostClasses,
) -> Option<RouteCostBreakdown> {
    match costing.edge_cost_with_classes(record, classes) {
        EdgeCostDecision::Allowed(cost) => Some(cost.breakdown),
        EdgeCostDecision::Blocked | EdgeCostDecision::NotAccessible => None,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct AppendGeometryOutcome {
    begin_shape_index: usize,
    end_shape_index: usize,
    duplicated_edge_suppressed: bool,
    shape_fallback_used: bool,
}

/// Scratch buffers of [`append_edge_geometry`], reused across a leg's edges.
#[derive(Default)]
struct GeometryScratch {
    /// The directed polyline's segment lengths (`segment[i]` from point `i`).
    segment_m: Vec<f64>,
    /// The clipped points, each with the directed point index it is
    /// bit-identical to, when it is one (its distances can be reused).
    clipped: Vec<(GeoPoint, Option<usize>)>,
    /// The leg geometry's step lengths (`geometry_segment_m[i]` from point
    /// `i - 1` to point `i`, NaN for the first), for guidance's bearings.
    geometry_segment_m: Vec<f64>,
}

/// `append_edge_geometry` without its intermediate vectors and with each
/// segment's haversine length computed once: the same arithmetic on the same
/// inputs as [`clipped_directed_geometry`] and the dedupe of
/// `append_edge_geometry_reference`, so the points are bit-identical.
fn append_edge_geometry(
    geometry: &mut Vec<GeoPoint>,
    record: &GraphViewEdge,
    start_fraction: f64,
    end_fraction: f64,
    scratch: &mut GeometryScratch,
) -> AppendGeometryOutcome {
    let raw = &record.shared_info.geometry;
    let n = raw.len();
    let backward =
        record.edge.traversal_direction == wayfarer_package::TraversalDirection::Backward;
    let point = |index: usize| {
        let (lat, lng) = raw[if backward { n - 1 - index } else { index }];
        GeoPoint::new(lat, lng)
    };
    let clipped = &mut scratch.clipped;
    clipped.clear();
    let segment_m = &mut scratch.segment_m;
    segment_m.clear();
    let mut shape_fallback_used = false;
    if n < 2 {
        shape_fallback_used = true;
        clipped.extend((0..n).map(|index| (point(index), Some(index))));
    } else {
        segment_m.extend((0..n - 1).map(|index| point(index).distance_to(point(index + 1))));
        let start_fraction = start_fraction.clamp(0.0, 1.0);
        let end_fraction = end_fraction.clamp(start_fraction, 1.0);
        let total_distance: f64 = segment_m.iter().copied().sum();
        if total_distance <= f64::EPSILON {
            shape_fallback_used = true;
            clipped.extend((0..n).map(|index| (point(index), Some(index))));
        } else {
            let start_distance = total_distance * start_fraction;
            let end_distance = total_distance * end_fraction;
            let at_distance = |distance_m: f64| -> (GeoPoint, Option<usize>) {
                let mut cumulative = 0.0;
                for (index, &segment_distance) in segment_m.iter().enumerate() {
                    if segment_distance <= f64::EPSILON {
                        continue;
                    }
                    if cumulative + segment_distance >= distance_m {
                        let t = ((distance_m - cumulative) / segment_distance).clamp(0.0, 1.0);
                        let (a, b) = (point(index), point(index + 1));
                        let at =
                            GeoPoint::new(a.lat + (b.lat - a.lat) * t, a.lng + (b.lng - a.lng) * t);
                        return (at, same_point_index(at, a, index, b, index + 1));
                    }
                    cumulative += segment_distance;
                }
                (point(n - 1), Some(n - 1))
            };
            clipped.push(at_distance(start_distance));
            let mut cumulative = 0.0;
            for (index, segment_distance) in segment_m.iter().enumerate() {
                let segment_end = cumulative + segment_distance;
                if segment_end > start_distance && segment_end < end_distance {
                    clipped.push((point(index + 1), Some(index + 1)));
                }
                cumulative = segment_end;
            }
            let end_point = at_distance(end_distance);
            let last = *clipped.last().expect("start point");
            if cached_distance(last, end_point, segment_m) >= 0.01 {
                clipped.push(end_point);
            }
        }
    }

    let mut begin_shape_index = geometry.len();
    let mut end_shape_index = begin_shape_index;
    let mut duplicated_edge_suppressed = false;
    // The directed point index geometry's last point is, when this edge pushed it.
    let mut last_index = None::<usize>;
    let geometry_segment_m = &mut scratch.geometry_segment_m;
    for &(point, index) in clipped.iter() {
        let mut step_m = f64::NAN;
        if let Some(last) = geometry.last().copied() {
            let distance = cached_distance((last, last_index), (point, index), segment_m);
            if distance < 0.01 {
                duplicated_edge_suppressed = true;
                begin_shape_index = geometry.len().saturating_sub(1);
                end_shape_index = begin_shape_index;
                continue;
            }
            step_m = distance;
        }
        geometry.push(point);
        geometry_segment_m.push(step_m);
        last_index = index;
        end_shape_index = geometry.len().saturating_sub(1);
    }
    if geometry.is_empty() {
        geometry.push(GeoPoint::new(record.source.lat, record.source.lng));
        geometry_segment_m.push(f64::NAN);
        end_shape_index = 0;
        begin_shape_index = 0;
    }
    AppendGeometryOutcome {
        begin_shape_index,
        end_shape_index,
        duplicated_edge_suppressed,
        shape_fallback_used,
    }
}

/// The directed point index `at` is bit-identical to, if it is `a` or `b`.
fn same_point_index(
    at: GeoPoint,
    a: GeoPoint,
    a_index: usize,
    b: GeoPoint,
    b_index: usize,
) -> Option<usize> {
    let same = |p: GeoPoint, q: GeoPoint| {
        p.lat.to_bits() == q.lat.to_bits() && p.lng.to_bits() == q.lng.to_bits()
    };
    if same(at, a) {
        Some(a_index)
    } else if same(at, b) {
        Some(b_index)
    } else {
        None
    }
}

/// `a.distance_to(b)`, from the segment lengths when `a` and `b` are
/// consecutive directed points (the same haversine of the same inputs).
fn cached_distance(
    (a, a_index): (GeoPoint, Option<usize>),
    (b, b_index): (GeoPoint, Option<usize>),
    segment_m: &[f64],
) -> f64 {
    match (a_index, b_index) {
        (Some(i), Some(j)) if j == i + 1 && i < segment_m.len() => segment_m[i],
        _ => a.distance_to(b),
    }
}

#[cfg(test)]
fn append_edge_geometry_reference(
    geometry: &mut Vec<GeoPoint>,
    record: &GraphViewEdge,
    start_fraction: f64,
    end_fraction: f64,
) -> AppendGeometryOutcome {
    let clipped = clipped_directed_geometry(record, start_fraction, end_fraction);
    let shape_fallback_used = clipped.shape_fallback_used;
    let mut begin_shape_index = geometry.len();
    let mut end_shape_index = begin_shape_index;
    let mut duplicated_edge_suppressed = false;

    for point in clipped.points {
        if geometry
            .last()
            .is_some_and(|last| last.distance_to(point) < 0.01)
        {
            duplicated_edge_suppressed = true;
            begin_shape_index = geometry.len().saturating_sub(1);
            end_shape_index = begin_shape_index;
            continue;
        }
        geometry.push(point);
        end_shape_index = geometry.len().saturating_sub(1);
    }
    if geometry.is_empty() {
        geometry.push(GeoPoint::new(record.source.lat, record.source.lng));
        end_shape_index = 0;
        begin_shape_index = 0;
    }
    AppendGeometryOutcome {
        begin_shape_index,
        end_shape_index,
        duplicated_edge_suppressed,
        shape_fallback_used,
    }
}

fn append_endpoint_shape_flags(
    flags: &mut Vec<RouteShapeQualityFlag>,
    edge_id: u64,
    start_fraction: f64,
    end_fraction: f64,
    route: &SearchRoute,
) {
    let starts_at_origin = edge_id == route.origin.edge_id
        && fractions_close(start_fraction, route.origin.fraction_along_edge);
    let ends_at_destination = edge_id == route.destination.edge_id
        && fractions_close(end_fraction, route.destination.fraction_along_edge);
    if starts_at_origin && start_fraction > f64::EPSILON {
        push_shape_flag(flags, RouteShapeQualityFlag::ClippedOriginEdge);
    }
    if ends_at_destination && end_fraction < 1.0 - f64::EPSILON {
        push_shape_flag(flags, RouteShapeQualityFlag::ClippedDestinationEdge);
    }
    if starts_at_origin
        && ends_at_destination
        && edge_id == route.destination.edge_id
        && start_fraction > f64::EPSILON
        && end_fraction < 1.0 - f64::EPSILON
    {
        push_shape_flag(flags, RouteShapeQualityFlag::SameEdgeDirect);
    }
}

fn push_shape_flag(flags: &mut Vec<RouteShapeQualityFlag>, flag: RouteShapeQualityFlag) {
    if !flags.contains(&flag) {
        flags.push(flag);
    }
}

fn fractions_close(left: f64, right: f64) -> bool {
    (left - right).abs() <= 1e-6
}

#[cfg(test)]
#[derive(Debug, Clone, PartialEq)]
struct ClippedGeometry {
    points: Vec<GeoPoint>,
    shape_fallback_used: bool,
}

#[cfg(test)]
fn clipped_directed_geometry(
    record: &GraphViewEdge,
    start_fraction: f64,
    end_fraction: f64,
) -> ClippedGeometry {
    let directed =
        crate::spatial::directed_geometry(&record.shared_info, record.edge.traversal_direction);
    if directed.len() < 2 {
        return ClippedGeometry {
            points: directed,
            shape_fallback_used: true,
        };
    }

    let start_fraction = start_fraction.clamp(0.0, 1.0);
    let end_fraction = end_fraction.clamp(start_fraction, 1.0);
    let total_distance = polyline_distance_m(&directed);
    if total_distance <= f64::EPSILON {
        return ClippedGeometry {
            points: directed,
            shape_fallback_used: true,
        };
    }

    let start_distance = total_distance * start_fraction;
    let end_distance = total_distance * end_fraction;
    let mut clipped = vec![point_at_distance(&directed, start_distance)];
    let mut cumulative = 0.0;

    for segment in directed.windows(2) {
        let segment_distance = segment[0].distance_to(segment[1]);
        let segment_end = cumulative + segment_distance;
        if segment_end > start_distance && segment_end < end_distance {
            clipped.push(segment[1]);
        }
        cumulative = segment_end;
    }

    let end_point = point_at_distance(&directed, end_distance);
    if clipped
        .last()
        .is_none_or(|last| last.distance_to(end_point) >= 0.01)
    {
        clipped.push(end_point);
    }
    ClippedGeometry {
        points: clipped,
        shape_fallback_used: false,
    }
}

#[cfg(test)]
fn polyline_distance_m(points: &[GeoPoint]) -> f64 {
    points
        .windows(2)
        .map(|segment| segment[0].distance_to(segment[1]))
        .sum()
}

#[cfg(test)]
fn point_at_distance(points: &[GeoPoint], distance_m: f64) -> GeoPoint {
    let mut cumulative = 0.0;
    for segment in points.windows(2) {
        let segment_distance = segment[0].distance_to(segment[1]);
        if segment_distance <= f64::EPSILON {
            continue;
        }
        if cumulative + segment_distance >= distance_m {
            let t = ((distance_m - cumulative) / segment_distance).clamp(0.0, 1.0);
            return GeoPoint::new(
                segment[0].lat + (segment[1].lat - segment[0].lat) * t,
                segment[0].lng + (segment[1].lng - segment[0].lng) * t,
            );
        }
        cumulative += segment_distance;
    }
    points.last().copied().unwrap_or(GeoPoint::new(0.0, 0.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph_view::PackageGraphView;
    use crate::test_support::PackageBuildOptions;
    use crate::types::{CorrelatedLocation, CorrelationSide, RouteDiagnostics};
    use std::sync::Arc;
    use wayfarer_package::{DirectionalAccess, RoadType};
    use wayfarer_road_graph::{RoadGraphBuilder, RoutableWay};

    fn graph_step_way() -> RoutableWay {
        RoutableWay {
            osm_way_id: 1,
            points: (0..=8)
                .map(|index| (52.0, 21.0 + f64::from(index) * 0.001))
                .collect(),
            source_node_ids: Vec::new(),
            road_type: RoadType::Primary,
            directional_access: DirectionalAccess::Forward,
            name: Some("Dynamic overlay path".into()),
            ref_name: None,
            highway: Some("primary".into()),
            access: Some("yes".into()),
            vehicle: None,
            motor_vehicle: None,
            motorcar: None,
            bicycle: None,
            foot: None,
            oneway_foot: None,
            oneway_bicycle: None,
            motorroad: None,
            service: None,
            surface: Some("asphalt".into()),
            tracktype: None,
            smoothness: None,
            maxspeed: None,
            lanes: None,
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
            has_bridge: false,
            has_tunnel: false,
            has_ford: false,
            is_roundabout: false,
            is_link: false,
            is_destination_only: false,
            access_restricted: false,
            area: None,
            area_highway: None,
            conditional_restrictions: Vec::new(),
            ..Default::default()
        }
    }

    fn correlated(record: &GraphViewEdge, fraction_along_edge: f64) -> CorrelatedLocation {
        CorrelatedLocation {
            edge_id: record.edge.edge_id,
            compact_edge: crate::types::CompactEdgeHint::of(record.raw_edge_index),
            shared_edge_info_id: record.edge.shared_edge_info_id,
            opposing_edge_id: record.edge.opposing_edge_id,
            road_group_key: None,
            same_road_candidate_count: 0,
            source_node_id: record.source.id,
            target_node_id: record.target.id,
            snapped_point: if fraction_along_edge <= f64::EPSILON {
                GeoPoint::new(record.source.lat, record.source.lng)
            } else {
                GeoPoint::new(record.target.lat, record.target.lng)
            },
            distance_m: 0.0,
            fraction_along_edge,
            heading_deg: None,
            heading_delta_deg: None,
            side: CorrelationSide::Unknown,
            component_size: None,
            outbound_reach_edge_count: None,
            inbound_reach_edge_count: None,
            layer: None,
            level: None,
            score: 1.0,
            score_breakdown: Default::default(),
            confidence: 1.0,
            access: None,
        }
    }

    #[test]
    fn path_starting_past_the_origin_edge_keeps_its_first_edge_whole() {
        let mut builder = RoadGraphBuilder::new("origin-edge-skipped");
        for index in 0..3 {
            let lng = 21.0 + f64::from(index) * 0.001;
            builder.push_way(RoutableWay {
                osm_way_id: i64::from(index) + 1,
                points: vec![(52.0, lng), (52.0002, lng + 0.0005), (52.0, lng + 0.001)],
                ..graph_step_way()
            });
        }
        let graph = builder.finish();
        let (package, _) =
            crate::test_support::package(&graph, &PackageBuildOptions::default()).unwrap();
        let view = PackageGraphView::new(Arc::new(package));
        let edge_ids = graph
            .graph
            .edge_weights()
            .map(|edge| edge.edge_id)
            .collect::<Vec<_>>();
        assert!(edge_ids.len() >= 3);
        let records = edge_ids
            .iter()
            .map(|edge_id| view.edge_by_id(*edge_id).unwrap())
            .collect::<Vec<_>>();
        // The origin sits at the end of the first edge; the path starts on the
        // second one.
        let origin = correlated(&records[0], 1.0);
        let destination = correlated(&records[2], 1.0);
        let path = edge_ids[1..3].to_vec();
        let result = build_route_result(
            &view,
            SearchRoute {
                origin,
                destination,
                edge_indexes: path
                    .iter()
                    .map(|edge_id| view.raw_edge_index_for_edge_id(*edge_id))
                    .collect(),
                edge_ids: path,
                steps: Vec::new(),
                distance_m: 0.0,
                duration_s: 0.0,
                cost: 0.0,
                diagnostics: RouteDiagnostics::default(),
            },
            MobilityProfile::Wheeled,
            &RouteOptions::default(),
        )
        .unwrap();

        let leg = &result.legs[0];
        assert_eq!(leg.shape_segments[0].start_fraction, 0.0);
        assert!(leg.shape_segments[0].end_shape_index > leg.shape_segments[0].begin_shape_index);
        assert_eq!(
            leg.edges[0].distance_m,
            f64::from(records[1].edge.distance_m)
        );
        assert!(
            !leg.shape_segments[0]
                .quality_flags
                .contains(&RouteShapeQualityFlag::ClippedOriginEdge)
        );
    }

    #[test]
    fn cached_geometry_append_matches_the_reference_clip() {
        let mut builder = RoadGraphBuilder::new("geometry-parity");
        for index in 0..4 {
            let lng = 21.0 + f64::from(index) * 0.002;
            builder.push_way(RoutableWay {
                osm_way_id: i64::from(index) + 1,
                points: vec![
                    (52.0, lng),
                    (52.0003, lng + 0.0004),
                    (52.0003, lng + 0.0004),
                    (52.0001, lng + 0.0011),
                    (52.0, lng + 0.002),
                ],
                directional_access: DirectionalAccess::Both,
                ..graph_step_way()
            });
        }
        let graph = builder.finish();
        let (package, _) =
            crate::test_support::package(&graph, &PackageBuildOptions::default()).unwrap();
        let view = PackageGraphView::new(Arc::new(package));
        let records = graph
            .graph
            .edge_weights()
            .map(|edge| view.edge_by_id(edge.edge_id).unwrap())
            .collect::<Vec<_>>();
        let fractions = [
            (0.0, 1.0),
            (0.0, 0.3),
            (0.25, 1.0),
            (0.4, 0.6),
            (0.5, 0.5),
            (1.0, 1.0),
        ];
        let mut scratch = GeometryScratch::default();
        let (mut fast, mut reference) = (Vec::new(), Vec::new());
        for record in &records {
            for (start, end) in fractions {
                let a = append_edge_geometry(&mut fast, record, start, end, &mut scratch);
                let b = append_edge_geometry_reference(&mut reference, record, start, end);
                assert_eq!(a, b);
                assert_eq!(
                    fast.iter()
                        .map(|p| (p.lat.to_bits(), p.lng.to_bits()))
                        .collect::<Vec<_>>(),
                    reference
                        .iter()
                        .map(|p| (p.lat.to_bits(), p.lng.to_bits()))
                        .collect::<Vec<_>>()
                );
            }
        }
        assert_eq!(scratch.geometry_segment_m.len(), fast.len());
        for index in 1..fast.len() {
            assert_eq!(
                scratch.geometry_segment_m[index].to_bits(),
                fast[index - 1].distance_to(fast[index]).to_bits()
            );
        }
    }

    #[test]
    fn dynamic_overlay_graph_steps_build_the_same_legs_as_plain_paths() {
        let mut builder = RoadGraphBuilder::new("dynamic-step-materialization");
        builder.push_way(graph_step_way());
        let graph = builder.finish();
        let (package, _) =
            crate::test_support::package(&graph, &PackageBuildOptions::default()).unwrap();
        let view = PackageGraphView::new(Arc::new(package));
        let edge_ids = graph
            .graph
            .edge_weights()
            .map(|edge| edge.edge_id)
            .collect::<Vec<_>>();
        let edge_indexes = edge_ids
            .iter()
            .map(|edge_id| view.raw_edge_index_for_edge_id(*edge_id))
            .collect::<Vec<_>>();
        assert!(edge_indexes.iter().all(Option::is_some));
        let records = edge_indexes
            .iter()
            .map(|edge| view.edge_record_by_raw_edge_index(edge.unwrap()).unwrap())
            .collect::<Vec<_>>();
        let origin = correlated(&records[0], 0.0);
        let destination = correlated(records.last().unwrap(), 1.0);
        let distance_m = records
            .iter()
            .map(|record| f64::from(record.edge.distance_m))
            .sum();
        let options = RouteOptions::default();
        let baseline = build_route_result(
            &view,
            SearchRoute {
                origin: origin.clone(),
                destination: destination.clone(),
                edge_ids: edge_ids.clone(),
                edge_indexes: edge_indexes.clone(),
                steps: Vec::new(),
                distance_m,
                duration_s: 0.0,
                cost: 0.0,
                diagnostics: RouteDiagnostics::default(),
            },
            MobilityProfile::Wheeled,
            &options,
        )
        .unwrap();
        let dynamic = build_route_result(
            &view,
            SearchRoute {
                origin,
                destination,
                edge_ids: edge_ids.clone(),
                edge_indexes,
                steps: edge_ids
                    .iter()
                    .map(|edge_id| RouteSearchStep::GraphEdge {
                        edge_id: *edge_id,
                        start_fraction: 0.0,
                        end_fraction: 1.0,
                        quality_flags: Vec::new(),
                    })
                    .collect(),
                distance_m,
                duration_s: 0.0,
                cost: 0.0,
                diagnostics: RouteDiagnostics::default(),
            },
            MobilityProfile::Wheeled,
            &options,
        )
        .unwrap();

        assert_eq!(dynamic.edge_ids, baseline.edge_ids);
        assert_eq!(dynamic.geometry, baseline.geometry);
        assert_eq!(dynamic.legs[0].edges.len(), baseline.legs[0].edges.len());
    }
}

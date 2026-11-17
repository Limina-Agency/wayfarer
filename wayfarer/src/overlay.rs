//! Route overlay geometry projection.
//!
//! What survives here is the *primitive*: given a shape and a projection
//! distance, which road-graph edges does it cover. The caller decides what a
//! covered edge means. Route overlay zones ride entities now, and
//! `route_commands::compute::apply_overlay_zone_requests` is the one place that
//! turns a covered edge into a block or a cost multiplier — there is no second
//! compiler reading a parallel object model and reaching its own verdict.

use std::collections::{BTreeMap, BTreeSet};

use crate::graph_view::{GraphViewEdge, PackageGraphView};
use crate::spatial::directed_geometry;
use crate::types::{GeoPoint, RouteEdgeRange};
use serde::Serialize;

/// Counters describing what the overlay pass did to a route, surfaced on the
/// computed route's diagnostics. Populated by whoever applies the projections.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, serde::Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteOverlayDiagnostics {
    pub reviewed_overlay_count: usize,
    pub unreviewed_warning_overlay_count: usize,
    pub low_confidence_warning_overlay_count: usize,
    pub ignored_overlay_count: usize,
    pub projected_geometry_count: usize,
    pub explicit_edge_reference_count: usize,
    pub warning_edge_count: usize,
    pub closure_overlay_count: usize,
    pub no_go_overlay_count: usize,
    pub slow_go_overlay_count: usize,
    pub discount_overlay_count: usize,
    pub open_overlay_count: usize,
    pub hazard_warning_overlay_count: usize,
    pub report_warning_overlay_count: usize,
    pub entity_avoidance_overlay_count: usize,
    pub subject_constraint_overlay_count: usize,
    pub area_inside_edge_count: usize,
    pub area_intersecting_edge_count: usize,
    pub area_near_edge_count: usize,
    pub partial_edge_range_count: usize,
    pub projection_confidence_basis_points: Option<u32>,
    pub projection_backend: Option<String>,
    pub projected_tile_count: usize,
    pub decoded_tile_count: usize,
    pub compile_cache_hit_count: usize,
    pub compile_cache_miss_count: usize,
    pub compile_duration_ns: u64,
}

#[derive(Debug, Clone, Default, PartialEq)]
struct ProjectedOverlayEdges {
    edge_ids: BTreeSet<u64>,
    edge_ranges: Vec<RouteEdgeRange>,
    inside_edge_count: usize,
    intersecting_edge_count: usize,
    near_edge_count: usize,
    partial_edge_range_count: usize,
    confidence_basis_points: u32,
}

/// The shape of an overlay's geometry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteOverlayGeometryKind {
    Point,
    Line,
    Polygon,
}

/// The package edges a geometry covers ([`crate::RoutingEngine::project`]).
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteOverlayGeometryProjection {
    pub edge_ids: BTreeSet<u64>,
    pub edge_ranges: Vec<RouteEdgeRange>,
    pub inside_edge_count: usize,
    pub intersecting_edge_count: usize,
    pub near_edge_count: usize,
    pub partial_edge_range_count: usize,
    pub confidence_basis_points: u32,
}

pub fn project_route_overlay_geometry(
    view: &PackageGraphView,
    geometry_kind: RouteOverlayGeometryKind,
    points: &[GeoPoint],
    projection_distance_m: u32,
) -> RouteOverlayGeometryProjection {
    project_geometry_edges(view, geometry_kind, points, projection_distance_m).into()
}

impl From<ProjectedOverlayEdges> for RouteOverlayGeometryProjection {
    fn from(value: ProjectedOverlayEdges) -> Self {
        Self {
            edge_ids: value.edge_ids,
            edge_ranges: value.edge_ranges,
            inside_edge_count: value.inside_edge_count,
            intersecting_edge_count: value.intersecting_edge_count,
            near_edge_count: value.near_edge_count,
            partial_edge_range_count: value.partial_edge_range_count,
            confidence_basis_points: value.confidence_basis_points,
        }
    }
}

fn project_geometry_edges(
    access: &PackageGraphView,
    geometry_kind: RouteOverlayGeometryKind,
    points: &[GeoPoint],
    projection_distance_m: u32,
) -> ProjectedOverlayEdges {
    if points.is_empty() {
        return ProjectedOverlayEdges::default();
    }

    match geometry_kind {
        RouteOverlayGeometryKind::Point => {
            let edge_ids = points
                .iter()
                .flat_map(|point| access.candidate_edges_near(*point, projection_distance_m))
                .map(|record| record.edge.edge_id)
                .collect::<BTreeSet<_>>();
            ProjectedOverlayEdges {
                near_edge_count: edge_ids.len(),
                confidence_basis_points: 7_000,
                edge_ids,
                ..Default::default()
            }
        }
        RouteOverlayGeometryKind::Line => project_line_edges(access, points, projection_distance_m),
        RouteOverlayGeometryKind::Polygon => project_polygon_edges(access, points),
    }
}

fn project_line_edges(
    access: &PackageGraphView,
    points: &[GeoPoint],
    projection_distance_m: u32,
) -> ProjectedOverlayEdges {
    let mut edge_ids = BTreeSet::new();
    if points.len() == 1 {
        let edge_ids = access
            .candidate_edges_near(points[0], projection_distance_m)
            .into_iter()
            .map(|record| record.edge.edge_id)
            .collect::<BTreeSet<_>>();
        return ProjectedOverlayEdges {
            near_edge_count: edge_ids.len(),
            confidence_basis_points: 7_000,
            edge_ids,
            ..Default::default()
        };
    }
    let mut edge_ranges = BTreeMap::<u64, RouteEdgeRange>::new();
    for segment in points.windows(2) {
        let (min, max) = expanded_bounds(segment[0], segment[1], projection_distance_m);
        for record in access.candidate_edges_in_envelope(min.lat, min.lng, max.lat, max.lng) {
            let geometry = directed_geometry(&record.shared_info, record.edge.traversal_direction);
            if geometry.windows(2).any(|road_segment| {
                segment_distance_m(segment[0], segment[1], road_segment[0], road_segment[1])
                    <= f64::from(projection_distance_m)
            }) {
                let edge_id = record.edge.edge_id;
                edge_ids.insert(edge_id);
                if let Some(range) =
                    projected_range_for_line_segment(record, segment[0], segment[1])
                {
                    edge_ranges
                        .entry(edge_id)
                        .and_modify(|existing| {
                            existing.start_fraction =
                                existing.start_fraction.min(range.start_fraction);
                            existing.end_fraction = existing.end_fraction.max(range.end_fraction);
                        })
                        .or_insert(range);
                }
            }
        }
    }
    ProjectedOverlayEdges {
        near_edge_count: edge_ids.len(),
        partial_edge_range_count: edge_ids.len(),
        confidence_basis_points: 7_500,
        edge_ids,
        edge_ranges: edge_ranges.into_values().collect(),
        ..Default::default()
    }
}

fn project_polygon_edges(access: &PackageGraphView, polygon: &[GeoPoint]) -> ProjectedOverlayEdges {
    let Some((min, max)) = polygon_bounds(polygon) else {
        return ProjectedOverlayEdges::default();
    };
    let mut edge_ids = BTreeSet::new();
    let mut inside_edge_count = 0;
    let mut intersecting_edge_count = 0;
    for record in access.candidate_edges_in_envelope(min.lat, min.lng, max.lat, max.lng) {
        let geometry = directed_geometry(&record.shared_info, record.edge.traversal_direction);
        let fully_inside = geometry
            .iter()
            .all(|point| point_in_polygon(*point, polygon));
        let intersects = geometry
            .iter()
            .any(|point| point_in_polygon(*point, polygon))
            || geometry.windows(2).any(|road_segment| {
                polygon
                    .iter()
                    .copied()
                    .zip(polygon.iter().copied().cycle().skip(1))
                    .take(polygon.len())
                    .any(|(a, b)| segments_intersect(road_segment[0], road_segment[1], a, b))
            });
        if fully_inside {
            inside_edge_count += 1;
            edge_ids.insert(record.edge.edge_id);
        } else if intersects {
            intersecting_edge_count += 1;
            edge_ids.insert(record.edge.edge_id);
        }
    }
    ProjectedOverlayEdges {
        edge_ids,
        inside_edge_count,
        intersecting_edge_count,
        partial_edge_range_count: intersecting_edge_count,
        confidence_basis_points: if intersecting_edge_count > 0 {
            9_000
        } else {
            9_500
        },
        ..Default::default()
    }
}

fn projected_range_for_line_segment(
    record: GraphViewEdge,
    start: GeoPoint,
    end: GeoPoint,
) -> Option<RouteEdgeRange> {
    let geometry = directed_geometry(&record.shared_info, record.edge.traversal_direction);
    let start_fraction = fraction_along_geometry(start, &geometry)?;
    let end_fraction = fraction_along_geometry(end, &geometry)?;
    let range_start = start_fraction.min(end_fraction).clamp(0.0, 1.0);
    let range_end = start_fraction.max(end_fraction).clamp(0.0, 1.0);
    (range_end > range_start).then_some(RouteEdgeRange {
        edge_id: record.edge.edge_id,
        start_fraction: range_start,
        end_fraction: range_end,
    })
}

fn fraction_along_geometry(point: GeoPoint, geometry: &[GeoPoint]) -> Option<f64> {
    let total_distance_m = geometry_length_m(geometry);
    if total_distance_m <= f64::EPSILON {
        return None;
    }
    let mut best: Option<(f64, f64)> = None;
    let mut distance_before_m = 0.0;
    for segment in geometry.windows(2) {
        let segment_length_m = segment[0].distance_to(segment[1]);
        if segment_length_m <= f64::EPSILON {
            continue;
        }
        let t = projection_fraction_on_segment(point, segment[0], segment[1]);
        let projected = interpolate(segment[0], segment[1], t);
        let distance_m = point.distance_to(projected);
        let fraction =
            ((distance_before_m + segment_length_m * t) / total_distance_m).clamp(0.0, 1.0);
        if best.is_none_or(|(best_distance, _)| distance_m < best_distance) {
            best = Some((distance_m, fraction));
        }
        distance_before_m += segment_length_m;
    }
    best.map(|(_, fraction)| fraction)
}

fn geometry_length_m(geometry: &[GeoPoint]) -> f64 {
    geometry
        .windows(2)
        .map(|segment| segment[0].distance_to(segment[1]))
        .sum()
}

fn projection_fraction_on_segment(point: GeoPoint, start: GeoPoint, end: GeoPoint) -> f64 {
    let mean_lat = ((start.lat + end.lat + point.lat) / 3.0).to_radians();
    let x_scale = mean_lat.cos().abs().max(0.01);
    let ax = start.lng * x_scale;
    let ay = start.lat;
    let bx = end.lng * x_scale;
    let by = end.lat;
    let px = point.lng * x_scale;
    let py = point.lat;
    let dx = bx - ax;
    let dy = by - ay;
    let length_sq = dx * dx + dy * dy;
    if length_sq <= f64::EPSILON {
        return 0.0;
    }
    (((px - ax) * dx + (py - ay) * dy) / length_sq).clamp(0.0, 1.0)
}

fn interpolate(start: GeoPoint, end: GeoPoint, fraction: f64) -> GeoPoint {
    GeoPoint::new(
        start.lat + (end.lat - start.lat) * fraction,
        start.lng + (end.lng - start.lng) * fraction,
    )
}

fn expanded_bounds(a: GeoPoint, b: GeoPoint, distance_m: u32) -> (GeoPoint, GeoPoint) {
    let min_lat = a.lat.min(b.lat);
    let max_lat = a.lat.max(b.lat);
    let min_lng = a.lng.min(b.lng);
    let max_lng = a.lng.max(b.lng);
    let lat_delta = f64::from(distance_m) / 111_320.0;
    let center_lat = ((a.lat + b.lat) / 2.0).to_radians();
    let lng_delta = f64::from(distance_m) / (111_320.0 * center_lat.cos().abs().max(0.01));
    (
        GeoPoint::new(min_lat - lat_delta, min_lng - lng_delta),
        GeoPoint::new(max_lat + lat_delta, max_lng + lng_delta),
    )
}

fn polygon_bounds(points: &[GeoPoint]) -> Option<(GeoPoint, GeoPoint)> {
    let first = points.first()?;
    let mut min_lat = first.lat;
    let mut max_lat = first.lat;
    let mut min_lng = first.lng;
    let mut max_lng = first.lng;
    for point in points.iter().skip(1) {
        min_lat = min_lat.min(point.lat);
        max_lat = max_lat.max(point.lat);
        min_lng = min_lng.min(point.lng);
        max_lng = max_lng.max(point.lng);
    }
    Some((
        GeoPoint::new(min_lat, min_lng),
        GeoPoint::new(max_lat, max_lng),
    ))
}

fn segment_distance_m(a: GeoPoint, b: GeoPoint, c: GeoPoint, d: GeoPoint) -> f64 {
    if segments_intersect(a, b, c, d) {
        return 0.0;
    }
    point_segment_distance_m(a, c, d)
        .min(point_segment_distance_m(b, c, d))
        .min(point_segment_distance_m(c, a, b))
        .min(point_segment_distance_m(d, a, b))
}

fn point_segment_distance_m(point: GeoPoint, a: GeoPoint, b: GeoPoint) -> f64 {
    let origin_lat = point.lat.to_radians();
    let meters_per_lat = 111_320.0;
    let meters_per_lng = 111_320.0 * origin_lat.cos().abs().max(0.01);
    let to_xy = |p: GeoPoint| -> (f64, f64) {
        (
            (p.lng - point.lng) * meters_per_lng,
            (p.lat - point.lat) * meters_per_lat,
        )
    };
    let (ax, ay) = to_xy(a);
    let (bx, by) = to_xy(b);
    let vx = bx - ax;
    let vy = by - ay;
    let length_sq = vx * vx + vy * vy;
    if length_sq <= f64::EPSILON {
        return point.distance_to(a);
    }
    let t = (-(ax * vx + ay * vy) / length_sq).clamp(0.0, 1.0);
    let x = ax + vx * t;
    let y = ay + vy * t;
    (x * x + y * y).sqrt()
}

fn point_in_polygon(point: GeoPoint, polygon: &[GeoPoint]) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    let mut inside = false;
    let mut previous = *polygon.last().unwrap();
    for current in polygon {
        let crosses = (current.lat > point.lat) != (previous.lat > point.lat)
            && point.lng
                < (previous.lng - current.lng) * (point.lat - current.lat)
                    / (previous.lat - current.lat)
                    + current.lng;
        if crosses {
            inside = !inside;
        }
        previous = *current;
    }
    inside
}

fn segments_intersect(a: GeoPoint, b: GeoPoint, c: GeoPoint, d: GeoPoint) -> bool {
    fn orient(a: GeoPoint, b: GeoPoint, c: GeoPoint) -> f64 {
        (b.lng - a.lng) * (c.lat - a.lat) - (b.lat - a.lat) * (c.lng - a.lng)
    }
    fn on_segment(a: GeoPoint, b: GeoPoint, c: GeoPoint) -> bool {
        c.lng >= a.lng.min(b.lng)
            && c.lng <= a.lng.max(b.lng)
            && c.lat >= a.lat.min(b.lat)
            && c.lat <= a.lat.max(b.lat)
    }

    let o1 = orient(a, b, c);
    let o2 = orient(a, b, d);
    let o3 = orient(c, d, a);
    let o4 = orient(c, d, b);
    if o1.abs() <= f64::EPSILON && on_segment(a, b, c) {
        return true;
    }
    if o2.abs() <= f64::EPSILON && on_segment(a, b, d) {
        return true;
    }
    if o3.abs() <= f64::EPSILON && on_segment(c, d, a) {
        return true;
    }
    if o4.abs() <= f64::EPSILON && on_segment(c, d, b) {
        return true;
    }
    (o1 > 0.0) != (o2 > 0.0) && (o3 > 0.0) != (o4 > 0.0)
}

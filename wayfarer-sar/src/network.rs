//! Edge lookups and plane geometry shared by the planning steps.

use std::collections::BTreeSet;

use wayfarer::{
    GeoPoint, ProjectRequest, RouteOverlay, RouteOverlayGeometryKind, RouteOverlayPolicy,
    RoutingEngine,
};

use crate::{SarZone, SarZoneKind};

/// One edge of the network both ways: its key is the smaller of its directed
/// id and its opposite's.
#[derive(Debug, Clone)]
pub(crate) struct NetworkEdge {
    pub(crate) length_m: f64,
    /// The key edge's shape, in its direction.
    pub(crate) geometry: Vec<GeoPoint>,
}

pub(crate) struct Network<'a> {
    engine: &'a RoutingEngine,
}

impl<'a> Network<'a> {
    pub(crate) fn new(engine: &'a RoutingEngine) -> Self {
        Self { engine }
    }

    /// The undirected key of a directed edge id.
    pub(crate) fn key(&self, edge_id: u64) -> u64 {
        self.engine
            .route_graph_view()
            .opposing_edge_id(edge_id)
            .map_or(edge_id, |opposite| opposite.min(edge_id))
    }

    pub(crate) fn edge(&self, key: u64) -> Option<NetworkEdge> {
        let record = self.engine.route_graph_view().edge_by_id(key)?;
        let geometry = wayfarer_package::directed_geometry(
            &record.shared_info,
            record.edge.traversal_direction,
        );
        Some(NetworkEdge {
            length_m: f64::from(record.edge.distance_m).max(0.0),
            geometry,
        })
    }

    /// The undirected keys of the edges inside a polygon.
    pub(crate) fn keys_in_polygon(&self, polygon: &[GeoPoint]) -> BTreeSet<u64> {
        let projection = self.engine.project(&ProjectRequest {
            geometry_kind: RouteOverlayGeometryKind::Polygon,
            points: closed(polygon),
            projection_distance_m: 0,
        });
        projection
            .edge_ids
            .iter()
            .map(|edge_id| self.key(*edge_id))
            .collect()
    }
}

/// A polygon with its first point repeated at the end.
pub(crate) fn closed(polygon: &[GeoPoint]) -> Vec<GeoPoint> {
    let mut points = polygon.to_vec();
    if let (Some(first), Some(last)) = (points.first().copied(), points.last().copied())
        && first != last
    {
        points.push(first);
    }
    points
}

/// The blocking overlays of the zones of `kinds`.
pub(crate) fn zone_overlays(zones: &[SarZone], kinds: &[SarZoneKind]) -> Vec<RouteOverlay> {
    zones
        .iter()
        .filter(|zone| kinds.contains(&zone.kind))
        .map(|zone| {
            RouteOverlay::new(
                RouteOverlayGeometryKind::Polygon,
                closed(&zone.polygon),
                RouteOverlayPolicy::NoGo,
                1,
            )
        })
        .collect()
}

// Plans are replayed on other devices (a command post and a phone), so the
// trigonometry is libm's pure-Rust code, the same bits on every platform.
use libm::{asin, atan2, cos, sin};

const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// Great-circle distance in metres.
pub(crate) fn distance_m(a: GeoPoint, b: GeoPoint) -> f64 {
    let (lat1, lat2) = (a.lat.to_radians(), b.lat.to_radians());
    let half_lat = sin((b.lat - a.lat).to_radians() / 2.0);
    let half_lng = sin((b.lng - a.lng).to_radians() / 2.0);
    let h = half_lat * half_lat + cos(lat1) * cos(lat2) * half_lng * half_lng;
    2.0 * EARTH_RADIUS_M * asin(h.sqrt().min(1.0))
}

/// Initial bearing from `a` to `b`, degrees in `[0, 360)`.
pub(crate) fn bearing_deg(a: GeoPoint, b: GeoPoint) -> f64 {
    let (lat1, lat2) = (a.lat.to_radians(), b.lat.to_radians());
    let dlng = (b.lng - a.lng).to_radians();
    let y = sin(dlng) * cos(lat2);
    let x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlng);
    let bearing = atan2(y, x).to_degrees();
    if bearing < 0.0 {
        bearing + 360.0
    } else if bearing >= 360.0 {
        0.0
    } else {
        bearing
    }
}

/// The point `distance` metres from `origin` on `bearing` degrees
/// (spherical).
pub(crate) fn destination(origin: GeoPoint, bearing: f64, distance: f64) -> GeoPoint {
    let angular = distance / EARTH_RADIUS_M;
    let bearing = bearing.to_radians();
    let lat1 = origin.lat.to_radians();
    let lng1 = origin.lng.to_radians();
    let lat2 = asin(sin(lat1) * cos(angular) + cos(lat1) * sin(angular) * cos(bearing));
    let lng2 = lng1
        + atan2(
            sin(bearing) * sin(angular) * cos(lat1),
            cos(angular) - sin(lat1) * sin(lat2),
        );
    GeoPoint::new(lat2.to_degrees(), lng2.to_degrees())
}

/// The point halfway along a polyline (by length), or its only point.
pub(crate) fn midpoint(points: &[GeoPoint]) -> Option<GeoPoint> {
    let first = *points.first()?;
    let total: f64 = points
        .windows(2)
        .map(|pair| distance_m(pair[0], pair[1]))
        .sum();
    if total <= 0.0 {
        return Some(first);
    }
    let mut remaining = total / 2.0;
    for pair in points.windows(2) {
        let step = distance_m(pair[0], pair[1]);
        if step >= remaining && step > 0.0 {
            let t = remaining / step;
            return Some(GeoPoint::new(
                pair[0].lat + (pair[1].lat - pair[0].lat) * t,
                pair[0].lng + (pair[1].lng - pair[0].lng) * t,
            ));
        }
        remaining -= step;
    }
    points.last().copied()
}

/// The area-weighted centroid of a simple polygon (plane approximation in
/// degrees, fine at segment size), or its vertex mean when degenerate.
pub(crate) fn centroid(polygon: &[GeoPoint]) -> GeoPoint {
    let points = closed(polygon);
    let (mut area, mut lat, mut lng) = (0.0, 0.0, 0.0);
    for pair in points.windows(2) {
        let cross = pair[0].lng * pair[1].lat - pair[1].lng * pair[0].lat;
        area += cross;
        lng += (pair[0].lng + pair[1].lng) * cross;
        lat += (pair[0].lat + pair[1].lat) * cross;
    }
    if area.abs() < 1e-18 {
        let count = polygon.len().max(1) as f64;
        return GeoPoint::new(
            polygon.iter().map(|point| point.lat).sum::<f64>() / count,
            polygon.iter().map(|point| point.lng).sum::<f64>() / count,
        );
    }
    GeoPoint::new(lat / (3.0 * area), lng / (3.0 * area))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plane_helpers() {
        let origin = GeoPoint::new(49.0, 20.0);
        let east = destination(origin, 90.0, 1_000.0);
        assert!((distance_m(origin, east) - 1_000.0).abs() < 0.01);
        assert!((bearing_deg(origin, east) - 90.0).abs() < 0.01);
        let north = destination(origin, 0.0, 500.0);
        assert!(bearing_deg(origin, north) < 0.01 || bearing_deg(origin, north) > 359.99);
        let middle = midpoint(&[origin, east]).unwrap();
        assert!((distance_m(origin, middle) - 500.0).abs() < 0.5);
        let square = [
            GeoPoint::new(0.0, 0.0),
            GeoPoint::new(0.0, 2.0),
            GeoPoint::new(2.0, 2.0),
            GeoPoint::new(2.0, 0.0),
        ];
        let centre = centroid(&square);
        assert!((centre.lat - 1.0).abs() < 1e-9 && (centre.lng - 1.0).abs() < 1e-9);
        assert_eq!(closed(&square).len(), 5);
    }
}

//! A route as a GeoJSON `FeatureCollection` (RFC 7946), for maps and GIS
//! tools: the route's line, with its distance, duration and cost, and one
//! point per location where it snapped onto the road.

use serde_json::{Value, json};
use wayfarer::{GeoPoint, PolylineError};

use crate::RouteWorkflowResult;

fn coordinates(point: GeoPoint) -> Value {
    json!([point.lng, point.lat])
}

/// `result` as a `FeatureCollection` (longitude first): the route's
/// `LineString`, then a `Point` for each location's snapped point. Works
/// with any `geometryFormat`; fails only on a malformed `polyline6`.
pub fn route_feature_collection(result: &RouteWorkflowResult) -> Result<Value, PolylineError> {
    let route = &result.route;
    let mut features = vec![json!({
        "type": "Feature",
        "geometry": {
            "type": "LineString",
            "coordinates": route.points()?.into_iter().map(coordinates).collect::<Vec<_>>(),
        },
        "properties": {
            "distanceM": route.distance_m,
            "durationS": route.duration_s,
            "cost": route.cost,
            "legs": result
                .segment_summaries
                .as_ref()
                .map_or(route.segments.len(), Vec::len),
        },
    })];
    for (index, matched) in route.location_matches.iter().enumerate() {
        if let Some(location) = matched.preferred() {
            features.push(json!({
                "type": "Feature",
                "geometry": { "type": "Point", "coordinates": coordinates(location.snapped_point) },
                "properties": { "location": index, "edgeId": location.edge_id, "snapDistanceM": location.distance_m },
            }));
        }
    }
    Ok(json!({ "type": "FeatureCollection", "features": features }))
}

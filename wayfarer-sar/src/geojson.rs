//! GeoJSON views of planning results, for maps and GIS tools.
//!
//! Coordinates are `[lng, lat]` (RFC 7946). Every feature carries a `kind`
//! property: `last_known`, `radius` (a fast, typical or slow person's
//! straight-line reach as a circle), `probability` (one edge, with its
//! probability and density per km), `route` (a likely route) and `corridor`.

use serde_json::{Value, json};
use wayfarer::{GeoPoint, RoutingEngine};

use crate::network::{Network, destination};
use crate::reach::{ProbabilityMap, ReachResult, RouteTarget};

const CIRCLE_POINTS: usize = 64;

fn position(point: GeoPoint) -> Value {
    json!([point.lng, point.lat])
}

fn line(points: &[GeoPoint]) -> Value {
    json!({
        "type": "LineString",
        "coordinates": points.iter().map(|point| position(*point)).collect::<Vec<_>>(),
    })
}

fn feature(geometry: Value, properties: Value) -> Value {
    json!({ "type": "Feature", "geometry": geometry, "properties": properties })
}

/// The edges of a probability map as LineStrings.
pub fn probability_features(engine: &RoutingEngine, probability: &ProbabilityMap) -> Vec<Value> {
    let network = Network::new(engine);
    probability
        .edges
        .iter()
        .filter_map(|edge| {
            let shape = network.edge(edge.edge_id)?;
            let density = if edge.length_m > 0.0 {
                edge.probability / (edge.length_m / 1_000.0)
            } else {
                0.0
            };
            Some(feature(
                line(&shape.geometry),
                json!({
                    "kind": "probability",
                    "edgeId": edge.edge_id,
                    "probability": edge.probability,
                    "densityPerKm": density,
                    "timeS": edge.time_s,
                    "avoidOnly": edge.avoid_only,
                }),
            ))
        })
        .collect()
}

/// A reach prediction as a FeatureCollection: the last known position, the
/// three radius circles, each person's reach polygon (when the request drew
/// them), the edge probabilities, the likely routes (when their edge ids
/// were kept; from the last known position, through any walk to the paths)
/// and the corridors.
pub fn reach_feature_collection(engine: &RoutingEngine, reach: &ReachResult) -> Value {
    let network = Network::new(engine);
    let mut features = vec![feature(
        json!({ "type": "Point", "coordinates": position(reach.last_known) }),
        json!({ "kind": "last_known", "elapsedS": reach.elapsed_s, "budgetS": reach.budget_s }),
    )];
    for area in &reach.areas {
        let ring = (0..=CIRCLE_POINTS)
            .map(|index| {
                let bearing = 360.0 * (index % CIRCLE_POINTS) as f64 / CIRCLE_POINTS as f64;
                position(destination(reach.last_known, bearing, area.max_radius_m))
            })
            .collect::<Vec<_>>();
        features.push(feature(
            json!({ "type": "Polygon", "coordinates": [ring] }),
            json!({
                "kind": "radius",
                "person": area.person,
                "radiusM": area.max_radius_m,
                "networkLengthM": area.network_length_m,
            }),
        ));
    }
    for area in &reach.areas {
        if let Some(polygon) = &area.polygon {
            features.push(feature(
                json!(polygon),
                json!({
                    "kind": "reach",
                    "person": area.person,
                    "contourS": area.contour_s,
                    "networkLengthM": area.network_length_m,
                }),
            ));
        }
    }
    features.extend(probability_features(engine, &reach.probability));
    for route in reach
        .routes
        .iter()
        .filter(|route| route.reachable && !route.edge_ids.is_empty())
    {
        let mut points: Vec<GeoPoint> = vec![reach.last_known];
        for edge_id in &route.edge_ids {
            let Some(record) = engine.route_graph_view().edge_by_id(*edge_id) else {
                continue;
            };
            let shape = wayfarer_package::directed_geometry(
                &record.shared_info,
                record.edge.traversal_direction,
            );
            for point in shape {
                if points.last() != Some(&point) {
                    points.push(point);
                }
            }
        }
        if let RouteTarget::Destination { point, .. } = &route.target
            && points.last() != Some(point)
        {
            points.push(*point);
        }
        let target = match &route.target {
            RouteTarget::Destination { id, .. } => json!({ "destination": id }),
            RouteTarget::Frontier {
                sector,
                bearing_deg,
                ..
            } => {
                json!({ "sector": sector, "bearingDeg": bearing_deg })
            }
        };
        features.push(feature(
            line(&points),
            json!({
                "kind": "route",
                "target": target,
                "durationS": route.duration_s,
                "distanceM": route.distance_m,
                "reachProbability": route.reach_probability,
                "arrivalElapsedS": route.arrival_elapsed_s,
            }),
        ));
    }
    for corridor in &reach.corridors {
        if let Some(shape) = network.edge(corridor.edge_id) {
            features.push(feature(
                line(&shape.geometry),
                json!({
                    "kind": "corridor",
                    "edgeId": corridor.edge_id,
                    "routeCount": corridor.route_count,
                    "share": corridor.share,
                }),
            ));
        }
    }
    json!({ "type": "FeatureCollection", "features": features })
}

//! Straight-line ("direct") route legs.
//!
//! A direct leg is the operator declaring "get from here to there over open
//! ground" — no road graph is consulted. The engine owns the synthesis rather
//! than the caller drawing its own two-point line, so that a mixed path stays a
//! single route: one geometry, one distance, one continuous maneuver list, and
//! the same overlay/blocked-edge policy applied to whichever legs *are* routed.
//!
//! Two properties are load-bearing here:
//!
//! * the emitted geometry is a densified **great circle**, so the drawn line and
//!   the reported distance describe the same path (a lon/lat lerp does not — it
//!   is neither a geodesic nor a rhumb line, and its length disagrees with the
//!   haversine distance between its endpoints); and
//! * the reported distance is measured **from the emitted vertices**, not
//!   recomputed end to end, so the two cannot drift apart no matter how the
//!   densification is tuned.

use crate::costing::profile_off_road_speed_mps;
use crate::guidance::{bearing_deg, cardinal_direction};
use crate::types::{GeoPoint, MobilityProfile, RouteManeuver, RouteManeuverKind, haversine_m};
use serde::Serialize;

/// Target spacing between synthesized great-circle vertices.
///
/// The constraint is that the rendered polyline must not visibly depart from the
/// geodesic it claims to be. Each emitted sub-segment is drawn as a straight line
/// in Web Mercator; over a span `s` the worst-case departure from the true
/// geodesic grows like `s² / (8 R)` (scaled by roughly `tan(latitude)` for the
/// projection). At 250 m and 60° north that is about 2 mm — far below a pixel at
/// any zoom the map offers, with several orders of magnitude of headroom. Going
/// finer buys nothing visible and only inflates the payload.
///
/// 250 m also doubles as a usable elevation-sampling interval, so a direct leg
/// produces a real elevation profile instead of a two-point straight line.
pub const DIRECT_LEG_VERTEX_SPACING_M: f64 = 250.0;

/// Hard cap on sub-segments per direct leg, so a very long leg degrades to
/// coarser spacing instead of an unbounded payload. At the cap a 1 000 km leg
/// still spaces vertices under 2 km apart, which stays sub-pixel at every zoom
/// where a leg that long is actually on screen.
pub const MAX_DIRECT_LEG_SEGMENT_COUNT: usize = 512;

/// A synthesized straight-line leg, shaped so a caller can merge it beside a
/// graph-routed segment without special-casing the merge.
#[derive(Debug, Clone, PartialEq, Serialize, serde::Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct DirectRouteLeg {
    /// Densified great-circle vertices, origin first and destination last.
    pub geometry: Vec<GeoPoint>,
    /// `geometry` as an encoded polyline at precision 6, when the request
    /// asks for `geometryFormat: "polyline6"`; `geometry` is then empty.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub polyline6: Option<String>,
    /// `geometry` as a GeoJSON `LineString`, when the request asks for
    /// `geometryFormat: "geojson"`; `geometry` is then empty.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub geojson: Option<crate::GeoJsonLineString>,
    /// Length of `geometry`, summed over its own vertices.
    pub distance_m: f64,
    /// Estimated traverse time at the profile's off-road speed.
    pub duration_s: f64,
    /// The single maneuver covering the whole leg.
    pub maneuver: RouteManeuver,
}

/// Synthesizes the leg from `origin` to `destination` for `profile`.
pub fn direct_route_leg(
    origin: GeoPoint,
    destination: GeoPoint,
    profile: MobilityProfile,
) -> DirectRouteLeg {
    let geometry = densify_geodesic(origin, destination, DIRECT_LEG_VERTEX_SPACING_M);
    let distance_m = polyline_length_m(&geometry);
    let duration_s = distance_m / profile_off_road_speed_mps(profile);
    let maneuver = direct_leg_maneuver(&geometry, distance_m, duration_s);
    DirectRouteLeg {
        geometry,
        polyline6: None,
        geojson: None,
        distance_m,
        duration_s,
        maneuver,
    }
}

/// Samples the great circle from `a` to `b` at no more than `spacing_m` apart,
/// always including both endpoints exactly as given. Degenerate input (the same
/// point twice) yields the two endpoints and a zero length.
pub fn densify_geodesic(a: GeoPoint, b: GeoPoint, spacing_m: f64) -> Vec<GeoPoint> {
    let distance_m = haversine_m(a, b);
    let spacing_m = if spacing_m.is_finite() && spacing_m > 0.0 {
        spacing_m
    } else {
        DIRECT_LEG_VERTEX_SPACING_M
    };
    let segment_count = if distance_m.is_finite() && distance_m > spacing_m {
        ((distance_m / spacing_m).ceil() as usize).min(MAX_DIRECT_LEG_SEGMENT_COUNT)
    } else {
        1
    };
    if segment_count <= 1 {
        return vec![a, b];
    }

    let mut points = Vec::with_capacity(segment_count + 1);
    points.push(a);
    for index in 1..segment_count {
        points.push(interpolate_geodesic(
            a,
            b,
            index as f64 / segment_count as f64,
        ));
    }
    // The endpoints are copied rather than interpolated so a direct leg always
    // starts and ends exactly on the operator's waypoints, with no rounding gap
    // where it meets the neighbouring leg.
    points.push(b);
    points
}

/// Point at `fraction` along the great circle from `a` to `b`, by spherical
/// linear interpolation of the two unit vectors.
fn interpolate_geodesic(a: GeoPoint, b: GeoPoint, fraction: f64) -> GeoPoint {
    let lat1 = a.lat.to_radians();
    let lng1 = a.lng.to_radians();
    let lat2 = b.lat.to_radians();
    let lng2 = b.lng.to_radians();
    let angular_distance = central_angle(lat1, lng1, lat2, lng2);
    // Coincident (or antipodal, where the great circle is not unique) endpoints
    // make the slerp weights singular; fall back to the nearer endpoint.
    if !angular_distance.is_finite() || angular_distance.sin().abs() < f64::EPSILON {
        return if fraction < 0.5 { a } else { b };
    }

    let sin_distance = angular_distance.sin();
    let weight_a = ((1.0 - fraction) * angular_distance).sin() / sin_distance;
    let weight_b = (fraction * angular_distance).sin() / sin_distance;
    let x = weight_a * lat1.cos() * lng1.cos() + weight_b * lat2.cos() * lng2.cos();
    let y = weight_a * lat1.cos() * lng1.sin() + weight_b * lat2.cos() * lng2.sin();
    let z = weight_a * lat1.sin() + weight_b * lat2.sin();
    GeoPoint::new(z.atan2(x.hypot(y)).to_degrees(), y.atan2(x).to_degrees())
}

/// Angular separation of two points on the sphere, in radians.
fn central_angle(lat1: f64, lng1: f64, lat2: f64, lng2: f64) -> f64 {
    let half_dlat = ((lat2 - lat1) / 2.0).sin();
    let half_dlng = ((lng2 - lng1) / 2.0).sin();
    let h = half_dlat.powi(2) + lat1.cos() * lat2.cos() * half_dlng.powi(2);
    2.0 * h.sqrt().clamp(0.0, 1.0).asin()
}

/// Length of a polyline, summed segment by segment over its own vertices.
pub fn polyline_length_m(points: &[GeoPoint]) -> f64 {
    points
        .windows(2)
        .map(|pair| haversine_m(pair[0], pair[1]))
        .sum()
}

/// The one maneuver a direct leg contributes.
///
/// It is deliberately not a turn instruction: nothing here follows a road, so
/// there is no road to name and no junction to describe. The wording mirrors the
/// existing cross-country connector guidance ("Cross open ground …") and states
/// the bearing and length, which is all a straight-line leg actually knows. The
/// kind stays [`RouteManeuverKind::Continue`] — the same choice the SSVG
/// connector maneuver makes — so that the waypoint-boundary rewriting a caller
/// applies to `Start`/`Destination` maneuvers leaves it alone and the merged
/// list stays uniform.
fn direct_leg_maneuver(geometry: &[GeoPoint], distance_m: f64, duration_s: f64) -> RouteManeuver {
    let bearing = geometry
        .first()
        .zip(geometry.last())
        .filter(|(from, to)| haversine_m(**from, **to) > 0.01)
        .map(|(from, to)| bearing_deg(*from, *to).round() as u16 % 360);
    let heading = bearing
        .map(cardinal_direction)
        .map(|cardinal| format!(" {cardinal}"))
        .unwrap_or_default();
    let instruction =
        format!("Cross open ground{heading} ~{distance_m:.0} m in a straight line (no road)");
    let concise_instruction = format!("Cross open ground{heading}");
    let instruction_parameters = [
        ("guidance_reason".to_string(), "direct_leg".to_string()),
        ("leg_mode".to_string(), "direct".to_string()),
        ("leg_distance_m".to_string(), format!("{distance_m:.1}")),
    ]
    .into_iter()
    .collect();

    RouteManeuver {
        kind: RouteManeuverKind::Continue,
        instruction: instruction.clone(),
        concise_instruction,
        pre_transition_instruction: None,
        post_transition_instruction: Some(instruction.clone()),
        alert_instruction: Some(instruction),
        cardinal_direction: bearing.map(cardinal_direction),
        bearing_deg: bearing,
        instruction_key: "route.direct_leg".to_string(),
        instruction_parameters,
        distance_m,
        duration_s,
        begin_shape_index: 0,
        end_shape_index: geometry.len().saturating_sub(1),
        ..RouteManeuver::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn kilometers_apart(distance_m: f64) -> (GeoPoint, GeoPoint) {
        // Due east from Kraków, along a parallel so the lerp/geodesic split is
        // visible: at 50° north a lon/lat lerp is measurably off the great circle.
        let origin = GeoPoint::new(50.0614, 19.9366);
        let destination = GeoPoint::new(
            50.0614,
            19.9366 + distance_m / (111_320.0 * 50.0614_f64.to_radians().cos()),
        );
        (origin, destination)
    }

    #[test]
    fn direct_leg_geometry_length_matches_reported_distance() {
        let (origin, destination) = kilometers_apart(12_000.0);

        let leg = direct_route_leg(origin, destination, MobilityProfile::OnFoot);

        assert_eq!(leg.geometry.first().copied(), Some(origin));
        assert_eq!(leg.geometry.last().copied(), Some(destination));
        let drawn = polyline_length_m(&leg.geometry);
        assert!(
            (drawn - leg.distance_m).abs() < 1e-6,
            "drawn {drawn} vs reported {}",
            leg.distance_m
        );
        // ... and the drawn path is the geodesic, not something longer.
        let end_to_end = haversine_m(origin, destination);
        assert!(
            (drawn - end_to_end).abs() < 0.5,
            "drawn {drawn} vs end-to-end {end_to_end}"
        );
    }

    #[test]
    fn direct_leg_densifies_at_the_configured_spacing() {
        let (origin, destination) = kilometers_apart(10_000.0);

        let leg = direct_route_leg(origin, destination, MobilityProfile::Wheeled);

        assert_eq!(leg.geometry.len(), 41); // 10 km / 250 m = 40 segments.
        for pair in leg.geometry.windows(2) {
            assert!(haversine_m(pair[0], pair[1]) <= DIRECT_LEG_VERTEX_SPACING_M + 1.0);
        }
    }

    #[test]
    fn direct_leg_shorter_than_the_spacing_stays_two_points() {
        let (origin, destination) = kilometers_apart(80.0);

        let leg = direct_route_leg(origin, destination, MobilityProfile::Wheeled);

        assert_eq!(leg.geometry.len(), 2);
        assert!((leg.distance_m - 80.0).abs() < 1.0);
    }

    #[test]
    fn direct_leg_vertex_count_is_capped_on_a_very_long_leg() {
        let (origin, destination) = kilometers_apart(1_000_000.0);

        let leg = direct_route_leg(origin, destination, MobilityProfile::Wheeled);

        assert_eq!(leg.geometry.len(), MAX_DIRECT_LEG_SEGMENT_COUNT + 1);
    }

    #[test]
    fn direct_leg_between_identical_points_is_degenerate_but_valid() {
        let origin = GeoPoint::new(50.0614, 19.9366);

        let leg = direct_route_leg(origin, origin, MobilityProfile::OnFoot);

        assert_eq!(leg.geometry, vec![origin, origin]);
        assert_eq!(leg.distance_m, 0.0);
        assert_eq!(leg.duration_s, 0.0);
    }

    #[test]
    fn direct_leg_duration_uses_the_profile_off_road_speed() {
        let (origin, destination) = kilometers_apart(5_000.0);

        let on_foot = direct_route_leg(origin, destination, MobilityProfile::OnFoot);
        let wheeled = direct_route_leg(origin, destination, MobilityProfile::Wheeled);
        let tracked = direct_route_leg(origin, destination, MobilityProfile::Tracked);

        assert!((on_foot.distance_m - wheeled.distance_m).abs() < 1e-9);
        // Off road, a tracked vehicle beats a wheeled one, which beats walking.
        assert!(tracked.duration_s < wheeled.duration_s);
        assert!(wheeled.duration_s < on_foot.duration_s);
        // And none of them claims a road-speed ETA: 5 km on foot is over an hour.
        assert!(on_foot.duration_s > 3_600.0);
    }

    #[test]
    fn direct_leg_maneuver_describes_a_straight_line_not_a_turn() {
        let (origin, destination) = kilometers_apart(3_000.0);

        let leg = direct_route_leg(origin, destination, MobilityProfile::Wheeled);

        assert_eq!(leg.maneuver.kind, RouteManeuverKind::Continue);
        assert_eq!(leg.maneuver.instruction_key, "route.direct_leg");
        assert_eq!(leg.maneuver.cardinal_direction.as_deref(), Some("E"));
        assert!(leg.maneuver.instruction.contains("Cross open ground"));
        assert!(leg.maneuver.road_name.is_none());
        assert!(leg.maneuver.road_refs.is_empty());
        assert_eq!(leg.maneuver.turn_angle_deg, None);
        assert_eq!(leg.maneuver.begin_shape_index, 0);
        assert_eq!(leg.maneuver.end_shape_index, leg.geometry.len() - 1);
        assert_eq!(
            leg.maneuver.instruction_parameters.get("leg_mode"),
            Some(&"direct".to_string())
        );
    }

    #[test]
    fn densified_vertices_track_the_great_circle_not_a_lon_lat_lerp() {
        // A long leg at high latitude, where the two diverge by kilometres.
        let origin = GeoPoint::new(60.0, -5.0);
        let destination = GeoPoint::new(60.0, 35.0);

        let midpoint = interpolate_geodesic(origin, destination, 0.5);

        // The great circle bulges poleward; a lon/lat lerp would stay on 60.0.
        assert!(
            midpoint.lat > 60.5,
            "great-circle midpoint latitude {}",
            midpoint.lat
        );
        assert!((midpoint.lng - 15.0).abs() < 1e-9);
    }
}

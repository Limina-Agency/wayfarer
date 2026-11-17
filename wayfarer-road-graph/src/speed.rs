//! Edge speed inference: posted or implicit limits, class defaults and
//! penalties.

use super::*;

pub fn infer_speed_kph(info: &SharedEdgeInfo) -> u16 {
    let base = info
        .maxspeed_kph
        .filter(|speed| speed.is_finite() && *speed > 0.0)
        .unwrap_or_else(|| default_speed_kph(info.road_type) as f32);
    let speed = base * speed_penalty_factor(info);
    speed.round().clamp(3.0, 130.0) as u16
}

pub(crate) fn infer_speed_kph_for_facts(
    road_type: RoadType,
    maxspeed_kph: Option<f32>,
    surface: Option<&str>,
    tracktype: Option<&str>,
    smoothness: Option<&str>,
    has_ford: bool,
) -> u16 {
    let info = SharedEdgeInfo {
        road_type,
        maxspeed_kph,
        surface: surface.map(Arc::from),
        tracktype: tracktype.map(Arc::from),
        smoothness: smoothness.map(Arc::from),
        has_ford,
        ..Default::default()
    };
    infer_speed_kph(&info)
}

pub(crate) fn speed_source_for_values(maxspeed_kph: Option<f32>) -> Arc<str> {
    // Two values for every shared edge info: hand out shared allocations.
    static TAGGED: std::sync::OnceLock<Arc<str>> = std::sync::OnceLock::new();
    static INFERRED: std::sync::OnceLock<Arc<str>> = std::sync::OnceLock::new();
    if maxspeed_kph.is_some_and(|speed| speed.is_finite() && speed > 0.0) {
        TAGGED.get_or_init(|| Arc::from("osm:maxspeed")).clone()
    } else {
        INFERRED
            .get_or_init(|| Arc::from("profile_inferred"))
            .clone()
    }
}

pub(crate) fn constrained_speed_kph(free_flow_speed_kph: u16) -> f32 {
    ((free_flow_speed_kph as f32) * 0.85).max(3.0)
}

/// Typical speed of a road without a posted limit, close to Valhalla's
/// defaults for the class (a road with a posted limit is driven at it).
fn default_speed_kph(road_type: RoadType) -> u16 {
    match road_type {
        RoadType::Motorway => 90,
        RoadType::Primary => 75,
        RoadType::Secondary => 60,
        RoadType::Tertiary => 50,
        RoadType::Residential => 35,
        RoadType::Track => 25,
        RoadType::Path => 8,
        RoadType::Other => 30,
    }
}

fn speed_penalty_factor(info: &SharedEdgeInfo) -> f32 {
    let mut factor = 1.0f32;
    factor *= match info.surface.as_deref() {
        Some("gravel" | "compacted" | "fine_gravel") => 0.85,
        Some("dirt" | "earth" | "ground") => 0.70,
        Some("mud") => 0.45,
        Some("sand") => 0.50,
        Some("cobblestone" | "sett" | "paving_stones") => 0.75,
        _ => 1.0,
    };
    factor *= match info.tracktype.as_deref() {
        Some("grade2") => 0.85,
        Some("grade3") => 0.70,
        Some("grade4") => 0.55,
        Some("grade5") => 0.40,
        _ => 1.0,
    };
    factor *= match info.smoothness.as_deref() {
        Some("bad") => 0.75,
        Some("very_bad") => 0.55,
        Some("horrible" | "very_horrible" | "impassable") => 0.35,
        _ => 1.0,
    };
    if info.has_ford {
        factor *= 0.60;
    }
    if info.is_service_like {
        factor *= 0.80;
    }
    factor
}

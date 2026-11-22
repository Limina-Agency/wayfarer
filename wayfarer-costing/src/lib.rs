//! The edge-cost model routing packages are built and queried with: costing
//! options, per-profile speeds and factors, attribute classes, conditional
//! restrictions and turn costs. The route compiler bakes each profile's
//! default costs with it; the engine derives columns for other options and
//! layers a request's blocks, avoidances and multipliers on through
//! [`EdgeOverrides`].
//!
//! ```
//! use wayfarer_costing::{
//!     COSTING_MODEL_VERSION, SurfaceCostClass, road_type_default_speed_kph, surface_cost_class,
//!     time_condition_active,
//! };
//! use wayfarer_package::RoadType;
//!
//! // OSM tags are classified once, at build time, into small enums.
//! assert_eq!(surface_cost_class(Some("asphalt")), SurfaceCostClass::Paved);
//! assert_eq!(surface_cost_class(Some("sett")), SurfaceCostClass::Cobblestone);
//!
//! // Free-flow defaults per road type, in km/h.
//! assert_eq!(road_type_default_speed_kph(RoadType::Motorway), 90.0);
//!
//! // Conditional restrictions are evaluated against a time of day.
//! assert_eq!(time_condition_active("07:00-09:00", 8 * 60), Some(true));
//!
//! // Packages record the model their stored costs were baked with.
//! println!("costing model {COSTING_MODEL_VERSION}");
//! ```
//!
//! # Stability
//!
//! [`COSTING_MODEL_VERSION`] is bumped whenever a change here changes any cost
//! or duration the compiler stores, and such a change ships the compiler and
//! the engine together. The engine opens packages of its own model and of
//! older ones: an older model's package is re-costed from its stored costing
//! inputs with the engine's model (each column once, on first use) and routes
//! like a recompiled one until it is recompiled. A newer model fails to open
//! as too new. Turn costs and junction delays
//! ([`junction`]) are not stored: the engine evaluates them at query time from
//! stored inputs, and the search bounds ignore them, so a change that only
//! raises them changes routes and durations on existing packages without a
//! bump.

use chrono::{Datelike, Timelike, Weekday};
use serde::{Deserialize, Serialize};
use wayfarer_package::format::road_type_rank;
use wayfarer_package::sar::{
    SarEdgeFacts, SarPathClass, SarWidthClass, cost_classes as sar_cost_classes,
};
use wayfarer_package::{GraphViewEdge, MobilityProfile, RoadEdge, RoadType, SharedEdgeInfo};

pub mod junction;
pub use junction::{JunctionDelay, JunctionTransition, edge_speed_mps};

/// Build-time cost classification of the free-form OSM `surface` tag.
///
/// Every costing function that reads `surface` partitions the tag values into
/// exactly these groups, so the class — computed once by the tile writer with
/// [`surface_cost_class`] — yields bit-identical factors to matching on the
/// original string at query time.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum SurfaceCostClass {
    #[default]
    Other,
    Paved,
    Gravel,
    Dirt,
    Mud,
    Sand,
    Cobblestone,
}

pub fn surface_cost_class(surface: Option<&str>) -> SurfaceCostClass {
    match surface {
        Some("asphalt" | "paved" | "concrete" | "concrete:plates" | "concrete:lanes") => {
            SurfaceCostClass::Paved
        }
        Some("gravel" | "compacted" | "fine_gravel") => SurfaceCostClass::Gravel,
        Some("dirt" | "earth" | "ground") => SurfaceCostClass::Dirt,
        Some("mud") => SurfaceCostClass::Mud,
        Some("sand") => SurfaceCostClass::Sand,
        Some("cobblestone" | "sett" | "paving_stones") => SurfaceCostClass::Cobblestone,
        _ => SurfaceCostClass::Other,
    }
}

/// Build-time cost classification of the OSM `tracktype` tag (see
/// [`SurfaceCostClass`] for the parity argument).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum TracktypeCostClass {
    #[default]
    Other,
    Grade1,
    Grade2,
    Grade3,
    Grade4,
    Grade5,
}

pub fn tracktype_cost_class(tracktype: Option<&str>) -> TracktypeCostClass {
    match tracktype {
        Some("grade1") => TracktypeCostClass::Grade1,
        Some("grade2") => TracktypeCostClass::Grade2,
        Some("grade3") => TracktypeCostClass::Grade3,
        Some("grade4") => TracktypeCostClass::Grade4,
        Some("grade5") => TracktypeCostClass::Grade5,
        _ => TracktypeCostClass::Other,
    }
}

/// Build-time cost classification of the OSM `smoothness` tag (see
/// [`SurfaceCostClass`] for the parity argument).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum SmoothnessCostClass {
    #[default]
    Other,
    Good,
    Intermediate,
    Bad,
    VeryBad,
    Horrible,
}

pub fn smoothness_cost_class(smoothness: Option<&str>) -> SmoothnessCostClass {
    match smoothness {
        Some("excellent" | "good") => SmoothnessCostClass::Good,
        Some("intermediate") => SmoothnessCostClass::Intermediate,
        Some("bad") => SmoothnessCostClass::Bad,
        Some("very_bad") => SmoothnessCostClass::VeryBad,
        Some("horrible" | "very_horrible" | "impassable") => SmoothnessCostClass::Horrible,
        _ => SmoothnessCostClass::Other,
    }
}

/// Build-time cost classification of the OSM `highway` tag as read by the
/// pedestrian speed models (see [`SurfaceCostClass`] for the parity argument).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum WalkwayCostClass {
    #[default]
    Other,
    Steps,
    Footpath,
    Track,
}

pub fn walkway_cost_class(highway: Option<&str>) -> WalkwayCostClass {
    match highway {
        Some("steps") => WalkwayCostClass::Steps,
        Some("footway" | "path" | "pedestrian" | "bridleway" | "cycleway") => {
            WalkwayCostClass::Footpath
        }
        Some("track") => WalkwayCostClass::Track,
        _ => WalkwayCostClass::Other,
    }
}

/// Build-time cost classification of the OSM `sac_scale` hiking-difficulty tag
/// (see [`SurfaceCostClass`] for the parity argument).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum SacScaleClass {
    #[default]
    Other,
    Hiking,
    MountainHiking,
    DemandingMountainHiking,
    AlpineHiking,
    DemandingAlpineHiking,
    DifficultAlpineHiking,
}

pub fn sac_scale_class(sac_scale: Option<&str>) -> SacScaleClass {
    match sac_scale {
        Some("hiking") => SacScaleClass::Hiking,
        Some("mountain_hiking") => SacScaleClass::MountainHiking,
        Some("demanding_mountain_hiking") => SacScaleClass::DemandingMountainHiking,
        Some("alpine_hiking") => SacScaleClass::AlpineHiking,
        Some("demanding_alpine_hiking") => SacScaleClass::DemandingAlpineHiking,
        Some("difficult_alpine_hiking") => SacScaleClass::DifficultAlpineHiking,
        _ => SacScaleClass::Other,
    }
}

/// Build-time cost classification of the OSM `mtb:scale` tag. Values are
/// `0`..`6`; anything else is `Other`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum MtbScaleClass {
    #[default]
    Other,
    Scale0,
    Scale1,
    Scale2,
    Scale3,
    Scale4,
    Scale5,
    Scale6,
}

pub fn mtb_scale_class(mtb_scale: Option<&str>) -> MtbScaleClass {
    // `mtb:scale` is sometimes tagged with a `+`/`-` qualifier (`3+`); the base
    // grade is what costing needs, so read the leading digit.
    match mtb_scale.map(|value| value.trim()).and_then(|value| {
        value
            .chars()
            .next()
            .filter(|character| character.is_ascii_digit())
    }) {
        Some('0') => MtbScaleClass::Scale0,
        Some('1') => MtbScaleClass::Scale1,
        Some('2') => MtbScaleClass::Scale2,
        Some('3') => MtbScaleClass::Scale3,
        Some('4') => MtbScaleClass::Scale4,
        Some('5') => MtbScaleClass::Scale5,
        Some('6') => MtbScaleClass::Scale6,
        _ => MtbScaleClass::Other,
    }
}

/// Build-time cost classification of the OSM `trail_visibility` tag.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum TrailVisibilityClass {
    #[default]
    Other,
    Excellent,
    Good,
    Intermediate,
    Bad,
    Horrible,
    NoVisibility,
}

pub fn trail_visibility_class(trail_visibility: Option<&str>) -> TrailVisibilityClass {
    match trail_visibility {
        Some("excellent") => TrailVisibilityClass::Excellent,
        Some("good") => TrailVisibilityClass::Good,
        Some("intermediate") => TrailVisibilityClass::Intermediate,
        Some("bad") => TrailVisibilityClass::Bad,
        Some("horrible") => TrailVisibilityClass::Horrible,
        Some("no") => TrailVisibilityClass::NoVisibility,
        _ => TrailVisibilityClass::Other,
    }
}

/// Build-time cost classification of the OSM `sidewalk` tag — which side of a
/// carriageway carries a walkway, for pedestrian costing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum SidewalkClass {
    #[default]
    Other,
    None,
    Left,
    Right,
    Both,
    Separate,
}

pub fn sidewalk_class(sidewalk: Option<&str>) -> SidewalkClass {
    match sidewalk {
        Some("no" | "none") => SidewalkClass::None,
        Some("left") => SidewalkClass::Left,
        Some("right") => SidewalkClass::Right,
        Some("both" | "yes") => SidewalkClass::Both,
        Some("separate") => SidewalkClass::Separate,
        _ => SidewalkClass::Other,
    }
}

/// Build-time cost classification of the OSM `crossing` tag.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum CrossingClass {
    #[default]
    Other,
    TrafficSignals,
    Marked,
    Unmarked,
    Informal,
}

pub fn crossing_class(crossing: Option<&str>) -> CrossingClass {
    match crossing {
        Some("traffic_signals") => CrossingClass::TrafficSignals,
        Some("marked" | "zebra" | "uncontrolled") => CrossingClass::Marked,
        Some("unmarked") => CrossingClass::Unmarked,
        Some("informal" | "no") => CrossingClass::Informal,
        _ => CrossingClass::Other,
    }
}

/// All string-derived cost inputs an edge needs during search, precomputed at
/// tile-build time so the search payload does not need to ship the attribution
/// strings themselves.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct EdgeCostClasses {
    pub surface: SurfaceCostClass,
    pub tracktype: TracktypeCostClass,
    pub smoothness: SmoothnessCostClass,
    pub walkway: WalkwayCostClass,
    /// `!route_refs.is_empty()` on the original shared info (note: counts raw
    /// entries, matching the historical `route_ref_factor` gate exactly).
    pub has_route_refs: bool,
    pub sac_scale: SacScaleClass,
    pub mtb_scale: MtbScaleClass,
    pub trail_visibility: TrailVisibilityClass,
    pub sidewalk: SidewalkClass,
    pub crossing: CrossingClass,
    /// The path class R5 `vehicle_on_paths` opens (none outside the
    /// `footpath` walkway class). Not a cost input: packed with `walkway`.
    #[serde(default)]
    pub path_class: SarPathClass,
    /// The way's width class (`width`, else `est_width`), for R5's width
    /// filter. Not a cost input.
    #[serde(default)]
    pub width: SarWidthClass,
}

impl EdgeCostClasses {
    pub fn from_shared_info(info: &SharedEdgeInfo) -> Self {
        Self {
            surface: surface_cost_class(info.surface.as_deref()),
            tracktype: tracktype_cost_class(info.tracktype.as_deref()),
            smoothness: smoothness_cost_class(info.smoothness.as_deref()),
            walkway: walkway_cost_class(info.highway.as_deref()),
            has_route_refs: !info.route_refs.is_empty(),
            sac_scale: sac_scale_class(info.sac_scale.as_deref()),
            mtb_scale: mtb_scale_class(info.mtb_scale.as_deref()),
            trail_visibility: trail_visibility_class(info.trail_visibility.as_deref()),
            sidewalk: sidewalk_class(info.sidewalk.as_deref()),
            crossing: crossing_class(info.crossing.as_deref()),
            path_class: SarPathClass::from_highway(info.highway.as_deref()),
            width: SarWidthClass::from_width_cm(info.physical_width_cm),
        }
    }
}

impl EdgeCostClasses {
    /// Packs every class into one word (3 bits per enum, bit 31 for
    /// `has_route_refs`), for the routing package's per-shared-info section.
    /// The walkway field (bits 9–11) also carries the path class, and bits
    /// 27–29 the width class (`wayfarer_package::sar::cost_classes`).
    pub fn pack(&self) -> u32 {
        let walkway = match self.walkway {
            WalkwayCostClass::Footpath => sar_cost_classes::path_code(self.path_class)
                .unwrap_or(sar_cost_classes::FOOTPATH_CODE),
            other => other as u32,
        };
        (self.surface as u32)
            | (self.tracktype as u32) << 3
            | (self.smoothness as u32) << 6
            | walkway << sar_cost_classes::WALKWAY_SHIFT
            | (self.sac_scale as u32) << 12
            | (self.mtb_scale as u32) << 15
            | (self.trail_visibility as u32) << 18
            | (self.sidewalk as u32) << 21
            | (self.crossing as u32) << 24
            | sar_cost_classes::width_bits(self.width)
            | u32::from(self.has_route_refs) << 31
    }

    /// Inverse of [`Self::pack`]; out-of-range fields read as `Other`.
    pub fn unpack(word: u32) -> Self {
        use CrossingClass as C;
        use MtbScaleClass as M;
        use SacScaleClass as A;
        use SidewalkClass as W;
        use SmoothnessCostClass as S;
        use SurfaceCostClass as F;
        use TracktypeCostClass as T;
        use TrailVisibilityClass as V;
        use WalkwayCostClass as K;
        fn pick<T: Copy + Default>(values: &[T], index: u32) -> T {
            values.get(index as usize).copied().unwrap_or_default()
        }
        let field = |shift: u32| (word >> shift) & 0b111;
        Self {
            surface: pick(
                &[
                    F::Other,
                    F::Paved,
                    F::Gravel,
                    F::Dirt,
                    F::Mud,
                    F::Sand,
                    F::Cobblestone,
                ],
                field(0),
            ),
            tracktype: pick(
                &[
                    T::Other,
                    T::Grade1,
                    T::Grade2,
                    T::Grade3,
                    T::Grade4,
                    T::Grade5,
                ],
                field(3),
            ),
            smoothness: pick(
                &[
                    S::Other,
                    S::Good,
                    S::Intermediate,
                    S::Bad,
                    S::VeryBad,
                    S::Horrible,
                ],
                field(6),
            ),
            walkway: if sar_cost_classes::is_footpath_code(field(9)) {
                K::Footpath
            } else {
                pick(&[K::Other, K::Steps, K::Footpath, K::Track], field(9))
            },
            sac_scale: pick(
                &[
                    A::Other,
                    A::Hiking,
                    A::MountainHiking,
                    A::DemandingMountainHiking,
                    A::AlpineHiking,
                    A::DemandingAlpineHiking,
                    A::DifficultAlpineHiking,
                ],
                field(12),
            ),
            mtb_scale: pick(
                &[
                    M::Other,
                    M::Scale0,
                    M::Scale1,
                    M::Scale2,
                    M::Scale3,
                    M::Scale4,
                    M::Scale5,
                    M::Scale6,
                ],
                field(15),
            ),
            trail_visibility: pick(
                &[
                    V::Other,
                    V::Excellent,
                    V::Good,
                    V::Intermediate,
                    V::Bad,
                    V::Horrible,
                    V::NoVisibility,
                ],
                field(18),
            ),
            sidewalk: pick(
                &[W::Other, W::None, W::Left, W::Right, W::Both, W::Separate],
                field(21),
            ),
            crossing: pick(
                &[
                    C::Other,
                    C::TrafficSignals,
                    C::Marked,
                    C::Unmarked,
                    C::Informal,
                ],
                field(24),
            ),
            has_route_refs: word >> 31 != 0,
            path_class: sar_cost_classes::path_class(word),
            width: sar_cost_classes::width_class(word),
        }
    }
}

/// Boolean shared-info attributes that costing reads, packed into one byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EdgeCostFlags(pub u8);

impl EdgeCostFlags {
    pub const TOLL: u8 = 1 << 0;
    pub const FERRY: u8 = 1 << 1;
    pub const GATE: u8 = 1 << 2;
    pub const ACCESS_RESTRICTED: u8 = 1 << 3;
    pub const DESTINATION_ONLY: u8 = 1 << 4;
    pub const FORD: u8 = 1 << 5;
    pub const TUNNEL: u8 = 1 << 6;

    pub fn from_shared_info(info: &SharedEdgeInfo) -> Self {
        let bit = |set: bool, bit: u8| if set { bit } else { 0 };
        Self(
            bit(info.is_toll, Self::TOLL)
                | bit(info.is_ferry, Self::FERRY)
                | bit(info.has_gate, Self::GATE)
                | bit(info.is_access_restricted, Self::ACCESS_RESTRICTED)
                | bit(info.is_destination_only, Self::DESTINATION_ONLY)
                | bit(info.has_ford, Self::FORD)
                | bit(info.is_tunnel, Self::TUNNEL),
        )
    }

    #[inline]
    pub fn has(self, bit: u8) -> bool {
        self.0 & bit != 0
    }
}

/// Everything the edge costing function reads about one directed edge. A
/// materialized record gives it ([`EdgeCostInputs::from_record`]: the
/// package build, per-edge overrides and the reference oracle); the package's
/// attribute sections give it for derived columns. Both cost it with
/// [`CostModel::edge_cost_inputs`], so derived columns equal record costs by
/// construction.
#[derive(Debug, Clone, Copy)]
pub struct EdgeCostInputs<'a> {
    pub edge_id: u64,
    pub distance_m: f32,
    pub mobility_mode_mask: wayfarer_package::ModeMask,
    pub inferred_speed_kph: Option<u16>,
    pub road_type: RoadType,
    pub flags: EdgeCostFlags,
    pub classes: EdgeCostClasses,
    pub maxspeed_kph: Option<f32>,
    pub free_flow_speed_kph: Option<f32>,
    pub constrained_speed_kph: Option<f32>,
    /// Raw `*:conditional` strings; only read when the options carry an
    /// evaluation time.
    pub conditional_restrictions: &'a [String],
    /// The edge's SAR bits (`wayfarer_package::sar::ACCESS_*`) and
    /// its way's legal reason byte: what a SAR request's relaxations need to
    /// open an edge its profile may not legally use.
    pub sar_access: u8,
    pub legal_reason: u8,
}

impl<'a> EdgeCostInputs<'a> {
    pub fn from_parts(edge: &RoadEdge, info: &'a SharedEdgeInfo, classes: EdgeCostClasses) -> Self {
        Self {
            edge_id: edge.edge_id,
            distance_m: edge.distance_m,
            mobility_mode_mask: edge.mobility_mode_mask,
            inferred_speed_kph: edge.inferred_speed_kph,
            road_type: info.road_type,
            flags: EdgeCostFlags::from_shared_info(info),
            classes,
            maxspeed_kph: info.maxspeed_kph,
            free_flow_speed_kph: info.free_flow_speed_kph,
            constrained_speed_kph: info.constrained_speed_kph,
            conditional_restrictions: &info.conditional_restrictions,
            sar_access: wayfarer_package::sar::edge_sar_bits(
                info,
                edge.traversal_direction,
                edge.mobility_mode_mask,
            ),
            legal_reason: info.legal_reason,
        }
    }

    pub fn from_record(record: &'a GraphViewEdge) -> Self {
        Self::from_parts(
            &record.edge,
            &record.shared_info,
            EdgeCostClasses::from_shared_info(&record.shared_info),
        )
    }

    #[inline]
    fn has_ford(&self) -> bool {
        self.flags.has(EdgeCostFlags::FORD)
    }

    /// What the SAR relaxations read about the edge.
    pub fn sar_facts(&self) -> SarEdgeFacts {
        SarEdgeFacts {
            access: self.sar_access,
            reason: wayfarer_package::sar::LegalReason(self.legal_reason),
            path: self.classes.path_class,
            width: self.classes.width,
        }
    }
}

/// Phase 0 time metric: legacy turn/transition penalties were expressed in the
/// distance cost domain (meter-equivalents, since the old base edge cost equaled
/// `distance_m` for edges >= 30 km/h). The base edge cost is now travel time in
/// seconds, so a turn penalty is converted to seconds by treating each
/// meter-equivalent of penalty as the time to cover it at a 30 km/h reference
/// speed — the same reference the old `profile_time_factor` used as its floor.
/// This keeps turn costs in the same (time) units as edge costs so they compose
/// correctly, while preserving the relative weighting of turns versus travel.
pub const TURN_COST_SECONDS_PER_LEGACY_UNIT: f64 = 3_600.0 / 30_000.0;

/// Legacy-unit cost of reversing onto the opposing edge of the same segment:
/// the cost of any turn of 155 degrees or more, so a U-turn in the middle of
/// a road costs what one at a junction does.
pub const U_TURN_LEGACY_COST: f64 = 180.0;

// Upper speed clamps (km/h) per vehicle-like profile. These are the fastest
// speeds each profile's `edge_duration_s` can ever realize; they are reused by
// `profile_max_speed_mps` to derive the admissible time heuristic,
// so a single source of truth keeps the heuristic's `v_max` from drifting below
// the true maximum modeled speed.
pub const WHEELED_MAX_SPEED_KPH: f64 = 140.0;
pub const TRACKED_MAX_SPEED_KPH: f64 = 55.0;
pub const EMERGENCY_MAX_SPEED_KPH: f64 = 140.0;
pub const ONFOOT_MAX_SPEED_KPH: f64 = 5.5;
pub const LEGGED_MAX_SPEED_KPH: f64 = 7.0;

/// Speed (m/s) assumed for a straight-line leg, which by definition leaves the
/// road graph and therefore has no edge to read a speed from.
///
/// Rather than invent a second speed model, the profile's existing on-graph
/// model is evaluated against the worst ground it still calls traversable — an
/// unimproved dirt track. Vehicle profiles get the `Track` default speed scaled
/// by their own dirt-surface factor; pedestrian profiles get their track walking
/// speed scaled the same way. `Emergency` may use either drivetrain, so it takes
/// the faster of the two vehicle estimates.
///
/// The result is deliberately pessimistic: a direct leg crosses unsurveyed
/// ground, and reporting a road-speed ETA for it would be a lie. It is an
/// estimate, not a measurement.
pub fn profile_off_road_speed_mps(profile: MobilityProfile) -> f64 {
    // No tracktype and no smoothness: open ground carries neither tag, so only
    // the surface class (and the pedestrian walkway class) applies.
    let classes = EdgeCostClasses {
        surface: SurfaceCostClass::Dirt,
        walkway: WalkwayCostClass::Track,
        ..EdgeCostClasses::default()
    };
    let track_speed_kph = road_type_default_speed_kph(RoadType::Track);
    let wheeled_kph = track_speed_kph * wheeled_surface_speed_factor(classes.surface, false);
    let tracked_kph = track_speed_kph * tracked_surface_speed_factor(classes.surface, false);
    let speed_kph = match ProfilePolicy::from_profile(profile) {
        ProfilePolicy::Wheeled(_) => wheeled_kph,
        ProfilePolicy::Tracked(_) => tracked_kph,
        ProfilePolicy::Emergency(_) => wheeled_kph.max(tracked_kph),
        ProfilePolicy::OnFoot(_) => walking_speed_kph(classes, false),
        ProfilePolicy::Legged(_) => legged_speed_kph(classes, false),
    };
    speed_kph.max(1.0) * 1000.0 / 3600.0
}

#[derive(Debug, Clone, PartialEq)]
pub struct CostedEdge {
    pub cost: f64,
    pub duration_s: f64,
    pub is_avoided: bool,
    pub is_destination_only_penalized: bool,
    pub has_ford_penalty: bool,
    pub has_toll_penalty: bool,
    pub has_ferry_penalty: bool,
    pub has_gate_penalty: bool,
    pub has_access_restriction_penalty: bool,
    pub has_overlay_multiplier: bool,
    pub breakdown: RouteCostBreakdown,
}

#[derive(Debug, Clone, PartialEq)]
pub enum EdgeCostDecision {
    Allowed(CostedEdge),
    Blocked,
    NotAccessible,
}

/// The version of the edge-cost model. Packages record the model their stored
/// costs were baked with, and the engine opens only packages of its own model.
/// Bump it whenever a change to this crate changes any cost or duration the
/// compiler stores.
pub const COSTING_MODEL_VERSION: u32 = 1;

/// Per-edge request inputs the model leaves to its caller: blocked edges, the
/// avoidance penalty and the cost multiplier of each edge. The compiler bakes
/// costs with [`NoOverrides`]; the engine passes the request's.
pub trait EdgeOverrides {
    /// Whether the edge is blocked outright.
    fn edge_blocked(&self, edge_id: u64) -> bool;
    /// The avoidance penalty factor (1 when the edge is not avoided).
    fn avoidance_penalty(&self, edge_id: u64) -> f64;
    /// The cost multiplier (1 when the edge has none): slow-go factors and
    /// discounts (factors in (0, 1)) together.
    fn overlay_multiplier(&self, edge_id: u64) -> f64;
    /// The factor a discount applies to the edge's duration (1 without one):
    /// a discount lowers cost and duration alike, a slow-go factor only cost.
    fn duration_factor(&self, _edge_id: u64) -> f64 {
        1.0
    }
    /// The edge's travel time under a request speed override, given its
    /// length and its time without one (`duration_s` when it has none).
    fn overridden_duration_s(&self, _edge_id: u64, _distance_m: f64, duration_s: f64) -> f64 {
        duration_s
    }
    /// Whether the request opens the edge: it is costed for the profile even
    /// where its legal access, a destination-only rule or a conditional
    /// closure would close it.
    fn edge_opened(&self, _edge_id: u64) -> bool {
        false
    }
    /// Whether active conditional closures (`access:conditional=no @ …`) are
    /// ignored (the SAR `conditional` relaxation).
    fn ignores_conditional_access(&self) -> bool {
        false
    }
    /// The SAR relaxations (R1 `legal_access`, R2 `wrong_way`, R5
    /// `vehicle_on_paths`) that open edges the profile may not legally use,
    /// at a factor on their cost.
    fn sar_relaxations(&self) -> Option<wayfarer_package::sar::SarEdgeRelaxations> {
        None
    }
}

/// No per-edge request inputs: the package's default costing.
#[derive(Debug, Clone, Copy, Default)]
pub struct NoOverrides;

impl EdgeOverrides for NoOverrides {
    fn edge_blocked(&self, _edge_id: u64) -> bool {
        false
    }

    fn avoidance_penalty(&self, _edge_id: u64) -> f64 {
        1.0
    }

    fn overlay_multiplier(&self, _edge_id: u64) -> f64 {
        1.0
    }
}

/// The edge-cost model: a profile's speeds, factors, penalties and turn costs
/// under a set of costing options.
#[derive(Debug, Clone, Copy)]
pub struct CostModel<'a> {
    costing: &'a RouteCostingOptions,
    profile_policy: ProfilePolicy,
    allow_destination_only: bool,
    junction: Option<JunctionDelay>,
}

impl<'a> CostModel<'a> {
    pub fn new(profile: MobilityProfile, costing: &'a RouteCostingOptions) -> Self {
        Self::with_destination_only_policy(profile, costing, true)
    }

    pub fn with_destination_only_policy(
        profile: MobilityProfile,
        costing: &'a RouteCostingOptions,
        allow_destination_only: bool,
    ) -> Self {
        Self {
            costing,
            profile_policy: ProfilePolicy::from_profile(profile),
            allow_destination_only,
            junction: JunctionDelay::for_profile(profile),
        }
    }

    pub fn edge_cost(
        &self,
        record: &GraphViewEdge,
        overrides: &impl EdgeOverrides,
    ) -> EdgeCostDecision {
        self.edge_cost_inputs(&EdgeCostInputs::from_record(record), overrides)
    }

    /// Whether destination-only edges are usable (false on the strict pass of
    /// a request that avoids them fully).
    pub fn allows_destination_only(&self) -> bool {
        self.allow_destination_only
    }

    /// The edge costing function shared by record costing and the compact
    /// graph's derived columns.
    pub fn edge_cost_inputs(
        &self,
        record: &EdgeCostInputs<'_>,
        overrides: &impl EdgeOverrides,
    ) -> EdgeCostDecision {
        if overrides.edge_blocked(record.edge_id) {
            return EdgeCostDecision::Blocked;
        }
        // An opened edge is usable whatever its legal access says.
        let opened = overrides.edge_opened(record.edge_id);
        let mut flags = record.flags;
        if !opened && !overrides.ignores_conditional_access() {
            match active_conditional_access(
                record.conditional_restrictions,
                self.costing_options(),
                self.profile_policy,
            ) {
                ConditionalAccess::Closed => return EdgeCostDecision::NotAccessible,
                ConditionalAccess::DestinationOnly => flags.0 |= EdgeCostFlags::DESTINATION_ONLY,
                ConditionalAccess::Unchanged => {}
            }
        }
        // A SAR relaxation opens an edge the profile may not legally use, at
        // its cost with legal access times the relaxation's factor.
        let mut sar_factor = 1.0;
        if !opened && record.mobility_mode_mask & self.profile_policy.mobility_mode_mask() == 0 {
            match overrides
                .sar_relaxations()
                .and_then(|relaxations| relaxations.factor(record.sar_facts()))
            {
                Some(factor) => sar_factor = factor,
                None => return EdgeCostDecision::NotAccessible,
            }
        }
        if record.has_ford() && !self.costing_options().allow_ford {
            return EdgeCostDecision::NotAccessible;
        }
        if !opened && flags.has(EdgeCostFlags::DESTINATION_ONLY) && !self.allow_destination_only {
            return EdgeCostDecision::NotAccessible;
        }

        let classes = record.classes;
        let base_distance_cost = f64::from(record.distance_m).max(0.0);
        let speed_time_s = overrides.overridden_duration_s(
            record.edge_id,
            base_distance_cost,
            self.profile_policy
                .edge_duration_s(record, self.costing_options()),
        );
        let road_type_factor = self.profile_policy.road_type_cost_factor(
            record.road_type,
            classes,
            self.costing_options(),
        );
        // Walking pace already slows on rough surfaces (`walking_speed_kph`);
        // the vehicle surface, track-grade and smoothness penalties on top
        // would steer walks off tracks Valhalla's pedestrian costing uses.
        let walking = matches!(self.profile_policy, ProfilePolicy::OnFoot(_));
        let surface_factor =
            if walking {
                1.0
            } else {
                surface_factor(classes.surface)
            } * prefer_paved_surface_factor(classes.surface, self.costing_options());
        let tracktype_factor = if walking {
            1.0
        } else {
            tracktype_factor(classes.tracktype)
        };
        let smoothness_factor = if walking {
            1.0
        } else {
            smoothness_factor(classes.smoothness)
        };
        let ford_factor = if record.has_ford() { 1.4 } else { 1.0 };
        let bridge_tunnel_factor = bridge_tunnel_factor(flags.has(EdgeCostFlags::TUNNEL));
        let toll_penalty = if flags.has(EdgeCostFlags::TOLL) {
            1.0 + bounded(self.costing_options().avoid_tolls) * 3.0
        } else {
            1.0
        };
        let ferry_penalty = if flags.has(EdgeCostFlags::FERRY) {
            1.0 + bounded(self.costing_options().avoid_ferries) * 8.0
        } else {
            1.0
        };
        let gate_penalty = if flags.has(EdgeCostFlags::GATE) {
            1.0 + bounded(self.costing_options().avoid_gates) * 2.0
        } else {
            1.0
        };
        let access_restriction_penalty = if flags.has(EdgeCostFlags::ACCESS_RESTRICTED) {
            1.0 + bounded(self.costing_options().avoid_access_restricted) * 3.0
        } else {
            1.0
        };
        let destination_only_penalty = if flags.has(EdgeCostFlags::DESTINATION_ONLY) {
            1.0 + bounded(self.costing_options().avoid_destination_only) * 4.0
        } else {
            1.0
        };
        let avoided_edge_penalty = overrides.avoidance_penalty(record.edge_id);
        let is_avoided = avoided_edge_penalty > 1.0;
        let overlay_multiplier = overrides.overlay_multiplier(record.edge_id);
        let route_ref_factor = if classes.has_route_refs {
            1.0 - bounded(self.costing_options().prefer_known_route_refs) * 0.25
        } else {
            1.0
        };

        let mut breakdown = RouteCostBreakdown {
            base_distance_cost,
            speed_time_s,
            profile_time_factor: RouteCostBreakdown::calculate_profile_time_factor(
                base_distance_cost,
                speed_time_s,
            ),
            road_type_factor,
            surface_factor,
            tracktype_factor,
            smoothness_factor,
            ford_factor,
            bridge_tunnel_factor,
            toll_penalty,
            ferry_penalty,
            gate_penalty,
            access_restriction_penalty,
            destination_only_penalty,
            avoided_edge_penalty,
            overlay_multiplier,
            route_ref_factor,
            sar_factor,
            turn_penalty: 0.0,
            total_cost: 0.0,
        };
        breakdown.total_cost = breakdown.edge_cost_without_turn();

        EdgeCostDecision::Allowed(CostedEdge {
            cost: breakdown.total_cost,
            duration_s: speed_time_s * overrides.duration_factor(record.edge_id),
            is_avoided,
            is_destination_only_penalized: destination_only_penalty > 1.0,
            has_ford_penalty: ford_factor > 1.0,
            has_toll_penalty: toll_penalty > 1.0,
            has_ferry_penalty: ferry_penalty > 1.0,
            has_gate_penalty: gate_penalty > 1.0,
            has_access_restriction_penalty: access_restriction_penalty > 1.0,
            has_overlay_multiplier: (overlay_multiplier - 1.0).abs() > f64::EPSILON,
            breakdown,
        })
    }

    #[doc(hidden)]
    pub fn transition_cost(&self, previous: &RoadEdge, next: &RoadEdge) -> f64 {
        self.transition_cost_breakdown(previous, next)
    }

    pub fn transition_cost_for_records(
        &self,
        previous: &GraphViewEdge,
        next: &GraphViewEdge,
    ) -> f64 {
        self.transition_cost_with_shared_info(
            &previous.edge,
            &previous.shared_info,
            &next.edge,
            &next.shared_info,
            &|| same_signed_corridor(&previous.shared_info, &next.shared_info),
        )
    }

    #[doc(hidden)]
    pub fn transition_cost_breakdown(&self, previous: &RoadEdge, next: &RoadEdge) -> f64 {
        heading_transition_cost(previous, next)
    }

    #[doc(hidden)]
    pub fn transition_cost_with_shared_info(
        &self,
        previous: &RoadEdge,
        previous_info: &SharedEdgeInfo,
        next: &RoadEdge,
        next_info: &SharedEdgeInfo,
        same_corridor: &dyn Fn() -> bool,
    ) -> f64 {
        // One segment: straight on along it (a closed loop), or a U-turn onto
        // its opposing edge.
        if previous.shared_edge_info_id == next.shared_edge_info_id {
            if previous.traversal_direction == next.traversal_direction {
                return 0.0;
            }
            return U_TURN_LEGACY_COST * TURN_COST_SECONDS_PER_LEGACY_UNIT;
        }
        let base_cost = heading_transition_cost(previous, next);
        if base_cost <= 0.0 {
            return 0.0;
        }
        let legacy_cost = match transition_heading_delta(previous, next) {
            None => base_cost,
            Some(delta) => {
                let link_transition = link_transition(previous_info, next_info);
                let mut cost = if link_transition && delta < 155 {
                    (base_cost * 0.5).max(15.0)
                } else {
                    base_cost
                };
                if same_corridor() && delta < 100 {
                    cost *= 0.7;
                } else if !link_transition && delta >= 35 {
                    cost *= road_class_turn_factor(previous_info.road_type, next_info.road_type);
                }
                cost
            }
        };
        // Phase 0 time metric: convert the legacy distance-domain turn penalty
        // into seconds so it composes with the now time-domain edge cost.
        legacy_cost * TURN_COST_SECONDS_PER_LEGACY_UNIT
    }

    fn costing_options(&self) -> &RouteCostingOptions {
        self.costing
    }

    pub fn edge_duration_s(&self, record: &GraphViewEdge) -> f64 {
        self.profile_policy
            .edge_duration_s(&EdgeCostInputs::from_record(record), self.costing_options())
    }

    /// [`Self::edge_duration_s`] with the record's cost classes already
    /// derived (`EdgeCostClasses::from_shared_info` of its shared info).
    pub fn edge_duration_s_with_classes(
        &self,
        record: &GraphViewEdge,
        classes: EdgeCostClasses,
    ) -> f64 {
        self.profile_policy.edge_duration_s(
            &EdgeCostInputs::from_parts(&record.edge, &record.shared_info, classes),
            self.costing_options(),
        )
    }

    /// The junction delay (seconds) of moving from `previous` onto `next`:
    /// zero for walking profiles and for continuing along one segment (see
    /// [`junction`]). Speeds are the records' durations under this model.
    pub fn junction_delay_for_records(
        &self,
        previous: &GraphViewEdge,
        next: &GraphViewEdge,
    ) -> f64 {
        self.junction_delay_for_records_with(previous, next, &NoOverrides)
    }

    /// [`Self::junction_delay_for_records`] with the edges' speeds under the
    /// request's speed overrides and discounts.
    pub fn junction_delay_for_records_with(
        &self,
        previous: &GraphViewEdge,
        next: &GraphViewEdge,
        overrides: &impl EdgeOverrides,
    ) -> f64 {
        self.junction_delay_for_records_with_classes(
            previous,
            EdgeCostClasses::from_shared_info(&previous.shared_info),
            next,
            EdgeCostClasses::from_shared_info(&next.shared_info),
            overrides,
        )
    }

    /// [`Self::junction_delay_for_records_with`] with both records' cost
    /// classes already derived.
    pub fn junction_delay_for_records_with_classes(
        &self,
        previous: &GraphViewEdge,
        previous_classes: EdgeCostClasses,
        next: &GraphViewEdge,
        next_classes: EdgeCostClasses,
        overrides: &impl EdgeOverrides,
    ) -> f64 {
        self.junction_delay_for_records_memo(
            (previous, previous_classes, &mut None),
            (next, next_classes, &mut None),
            overrides,
        )
    }

    /// [`Self::junction_delay_for_records_with_classes`] remembering each
    /// record's junction speed in its `Option` (filled when computed, read
    /// when already filled), so a leg computes each edge's speed once for
    /// the junctions on both its ends.
    pub fn junction_delay_for_records_memo(
        &self,
        (previous, previous_classes, previous_speed): (
            &GraphViewEdge,
            EdgeCostClasses,
            &mut Option<f64>,
        ),
        (next, next_classes, next_speed): (&GraphViewEdge, EdgeCostClasses, &mut Option<f64>),
        overrides: &impl EdgeOverrides,
    ) -> f64 {
        let Some(model) = self.junction else {
            return 0.0;
        };
        let turn_deg = if previous.edge.shared_edge_info_id == next.edge.shared_edge_info_id {
            if previous.edge.traversal_direction == next.edge.traversal_direction {
                return 0.0;
            }
            Some(180)
        } else {
            transition_heading_delta(&previous.edge, &next.edge)
        };
        let speed = |record: &GraphViewEdge, classes: EdgeCostClasses, memo: &mut Option<f64>| {
            *memo.get_or_insert_with(|| {
                let length_m = f64::from(record.edge.distance_m);
                let edge_id = record.edge.edge_id;
                let duration_s = overrides.overridden_duration_s(
                    edge_id,
                    length_m.max(0.0),
                    self.edge_duration_s_with_classes(record, classes),
                ) * overrides.duration_factor(edge_id);
                edge_speed_mps(length_m, duration_s)
            })
        };
        let previous_speed_mps = speed(previous, previous_classes, previous_speed);
        let next_speed_mps = speed(next, next_classes, next_speed);
        model.delay_s(&JunctionTransition {
            turn_deg,
            previous_speed_mps,
            previous_length_m: f64::from(previous.edge.distance_m),
            next_speed_mps,
            next_length_m: f64::from(next.edge.distance_m),
            control: previous.edge.end_control,
            minor_approach: previous.edge.end_minor_approach,
        })
    }
}

pub fn transition_heading_delta(previous: &RoadEdge, next: &RoadEdge) -> Option<u16> {
    Some(heading_delta(
        previous.end_heading_deg?,
        next.start_heading_deg?,
    ))
}

pub fn heading_transition_cost(previous: &RoadEdge, next: &RoadEdge) -> f64 {
    let Some(delta) = transition_heading_delta(previous, next) else {
        return 0.0;
    };
    if delta < 35 {
        0.0
    } else if delta < 100 {
        35.0
    } else if delta < 155 {
        90.0
    } else {
        180.0
    }
}

pub fn link_transition(previous_info: &SharedEdgeInfo, next_info: &SharedEdgeInfo) -> bool {
    previous_info.is_link || next_info.is_link
}

pub fn same_signed_corridor(previous_info: &SharedEdgeInfo, next_info: &SharedEdgeInfo) -> bool {
    previous_info.osm_way_id != 0 && previous_info.osm_way_id == next_info.osm_way_id
        || non_empty_eq(
            previous_info.ref_name.as_deref(),
            next_info.ref_name.as_deref(),
        )
        || non_empty_eq(previous_info.name.as_deref(), next_info.name.as_deref())
        || previous_info.route_refs.iter().any(|left| {
            let left = left.trim();
            !left.is_empty()
                && next_info
                    .route_refs
                    .iter()
                    .any(|right| left.eq_ignore_ascii_case(right.trim()))
        })
}

pub fn sorted_token_sets_intersect(left: &[u32], right: &[u32]) -> bool {
    let (mut left_iter, mut right_iter) = (left.iter().peekable(), right.iter().peekable());
    while let (Some(left_token), Some(right_token)) = (left_iter.peek(), right_iter.peek()) {
        match left_token.cmp(right_token) {
            std::cmp::Ordering::Equal => return true,
            std::cmp::Ordering::Less => {
                left_iter.next();
            }
            std::cmp::Ordering::Greater => {
                right_iter.next();
            }
        }
    }
    false
}

pub fn non_empty_eq(left: Option<&str>, right: Option<&str>) -> bool {
    match (left.map(str::trim), right.map(str::trim)) {
        (Some(left), Some(right)) if !left.is_empty() && !right.is_empty() => {
            left.eq_ignore_ascii_case(right)
        }
        _ => false,
    }
}

pub fn road_class_turn_factor(previous: RoadType, next: RoadType) -> f64 {
    let previous_rank = road_type_rank(previous);
    let next_rank = road_type_rank(next);
    if previous_rank <= next_rank {
        return 1.0;
    }
    1.0 + f64::from(previous_rank - next_rank) * 0.2
}

#[derive(Debug, Clone, Copy)]
pub enum ProfilePolicy {
    Wheeled(WheeledPolicy),
    Tracked(TrackedPolicy),
    Emergency(EmergencyPolicy),
    OnFoot(OnFootPolicy),
    Legged(LeggedPolicy),
}

impl ProfilePolicy {
    fn from_profile(profile: MobilityProfile) -> Self {
        match profile {
            MobilityProfile::Default | MobilityProfile::Wheeled => Self::Wheeled(WheeledPolicy),
            MobilityProfile::Tracked => Self::Tracked(TrackedPolicy),
            MobilityProfile::Emergency => Self::Emergency(EmergencyPolicy),
            MobilityProfile::OnFoot => Self::OnFoot(OnFootPolicy),
            MobilityProfile::Legged => Self::Legged(LeggedPolicy),
        }
    }

    fn mobility_mode_mask(self) -> u32 {
        match self {
            Self::Wheeled(_) => MobilityProfile::Wheeled.mobility_mode_mask(),
            Self::Tracked(_) => MobilityProfile::Tracked.mobility_mode_mask(),
            Self::Emergency(_) => MobilityProfile::Emergency.mobility_mode_mask(),
            Self::OnFoot(_) => MobilityProfile::OnFoot.mobility_mode_mask(),
            Self::Legged(_) => MobilityProfile::Legged.mobility_mode_mask(),
        }
    }

    fn road_type_cost_factor(
        self,
        road_type: RoadType,
        classes: EdgeCostClasses,
        options: &RouteCostingOptions,
    ) -> f64 {
        match self {
            Self::Wheeled(policy) => policy.road_type_cost_factor(road_type, options),
            Self::Tracked(policy) => policy.road_type_cost_factor(road_type, options),
            Self::Emergency(policy) => policy.road_type_cost_factor(road_type, options),
            Self::OnFoot(policy) => policy.road_type_cost_factor(road_type, classes, options),
            Self::Legged(policy) => policy.road_type_cost_factor(road_type, options),
        }
    }

    fn edge_duration_s(self, edge: &EdgeCostInputs<'_>, options: &RouteCostingOptions) -> f64 {
        match self {
            Self::Wheeled(policy) => policy.edge_duration_s(edge, options),
            Self::Tracked(policy) => policy.edge_duration_s(edge, options),
            Self::Emergency(policy) => policy.edge_duration_s(edge, options),
            Self::OnFoot(policy) => policy.edge_duration_s(edge),
            Self::Legged(policy) => policy.edge_duration_s(edge),
        }
    }

    fn is_vehicle_like(self) -> bool {
        matches!(
            self,
            Self::Wheeled(_) | Self::Tracked(_) | Self::Emergency(_)
        )
    }

    fn is_pedestrian_like(self) -> bool {
        matches!(self, Self::OnFoot(_) | Self::Legged(_))
    }
}

#[derive(Debug, Clone, Copy)]
pub struct WheeledPolicy;

impl WheeledPolicy {
    /// Close to driving time, as in Valhalla's auto costing: a mild
    /// preference for the main network (residential also carries
    /// unclassified rural roads and service roads), tracks and paths kept
    /// for when nothing else connects.
    fn road_type_cost_factor(self, road_type: RoadType, options: &RouteCostingOptions) -> f64 {
        let base = match road_type {
            RoadType::Motorway | RoadType::Primary => 1.0,
            RoadType::Secondary => 1.05,
            RoadType::Tertiary => 1.1,
            RoadType::Residential => 1.2,
            RoadType::Other => 1.5,
            RoadType::Track => 8.0,
            RoadType::Path => 24.0,
        };
        base * avoid_track_path_factor(road_type, options)
            * avoid_motorway_factor(road_type, options)
    }

    fn edge_duration_s(self, edge: &EdgeCostInputs<'_>, options: &RouteCostingOptions) -> f64 {
        let speed_kph = vehicle_base_speed_kph(edge, options)
            * wheeled_surface_speed_factor(edge.classes.surface, edge.has_ford())
            * wheeled_tracktype_speed_factor(edge.classes.tracktype);
        profile_duration_s(edge, speed_kph.clamp(3.0, WHEELED_MAX_SPEED_KPH))
    }
}

#[derive(Debug, Clone, Copy)]
pub struct TrackedPolicy;

impl TrackedPolicy {
    fn road_type_cost_factor(self, road_type: RoadType, options: &RouteCostingOptions) -> f64 {
        let base = match road_type {
            RoadType::Track => 0.15,
            RoadType::Path => 0.8,
            _ => 1.0,
        };
        RoadTypeCost::vehicle_base(road_type)
            * base
            * avoid_track_path_factor(road_type, options)
            * avoid_motorway_factor(road_type, options)
    }

    fn edge_duration_s(self, edge: &EdgeCostInputs<'_>, options: &RouteCostingOptions) -> f64 {
        let terrain_speed = match edge.road_type {
            RoadType::Track => 30.0,
            RoadType::Path => 16.0,
            RoadType::Residential | RoadType::Other => 32.0,
            RoadType::Tertiary => 36.0,
            RoadType::Motorway | RoadType::Primary | RoadType::Secondary => 42.0,
        };
        let base = vehicle_base_speed_kph(edge, options).min(terrain_speed);
        let speed_kph = base
            * tracked_surface_speed_factor(edge.classes.surface, edge.has_ford())
            * tracked_tracktype_speed_factor(edge.classes.tracktype);
        profile_duration_s(edge, speed_kph.clamp(4.0, TRACKED_MAX_SPEED_KPH))
    }
}

#[derive(Debug, Clone, Copy)]
pub struct EmergencyPolicy;

impl EmergencyPolicy {
    fn road_type_cost_factor(self, road_type: RoadType, options: &RouteCostingOptions) -> f64 {
        let normal_profile = match road_type {
            RoadType::Track => 1.2,
            RoadType::Path => 2.5,
            _ => 1.0,
        };
        let override_strength = bounded(options.emergency_override_strength);
        let road_base = road_type.cost_factor() as f64;
        let override_profile = if road_base > f64::EPSILON {
            1.0 / road_base
        } else {
            1.0
        };
        let profile =
            normal_profile * (1.0 - override_strength) + override_profile * override_strength;
        RoadTypeCost::vehicle_base(road_type)
            * profile
            * avoid_track_path_factor(road_type, options)
            * avoid_motorway_factor(road_type, options)
    }

    fn edge_duration_s(self, edge: &EdgeCostInputs<'_>, options: &RouteCostingOptions) -> f64 {
        let override_strength = bounded(options.emergency_override_strength);
        let classes = edge.classes;
        let wheeled_speed = vehicle_base_speed_kph(edge, options)
            * wheeled_surface_speed_factor(classes.surface, edge.has_ford())
            * wheeled_tracktype_speed_factor(classes.tracktype)
            * 1.15;
        let tracked_speed = match edge.road_type {
            RoadType::Track => 32.0,
            RoadType::Path => 18.0,
            _ => vehicle_base_speed_kph(edge, options).min(48.0),
        } * tracked_surface_speed_factor(classes.surface, edge.has_ford())
            * tracked_tracktype_speed_factor(classes.tracktype);
        let speed_kph =
            wheeled_speed * (1.0 - override_strength) + tracked_speed * override_strength;
        profile_duration_s(edge, speed_kph.clamp(5.0, EMERGENCY_MAX_SPEED_KPH))
    }
}

#[derive(Debug, Clone, Copy)]
pub struct OnFootPolicy;

impl OnFootPolicy {
    /// Close to walking time, as in Valhalla's pedestrian costing: a mild
    /// preference against main roads, none where they are tagged with a
    /// sidewalk, and a stronger one where the sidewalk is mapped as its own
    /// way (`sidewalk=separate`), which then carries the walk.
    fn road_type_cost_factor(
        self,
        road_type: RoadType,
        classes: EdgeCostClasses,
        options: &RouteCostingOptions,
    ) -> f64 {
        let base = match road_type {
            RoadType::Motorway => 12.0,
            RoadType::Primary | RoadType::Secondary => match classes.sidewalk {
                SidewalkClass::Left | SidewalkClass::Right | SidewalkClass::Both => 1.0,
                SidewalkClass::Separate => 1.3,
                SidewalkClass::None | SidewalkClass::Other => 1.1,
            },
            RoadType::Tertiary => match classes.sidewalk {
                SidewalkClass::Separate => 1.2,
                _ => 1.05,
            },
            RoadType::Residential | RoadType::Other | RoadType::Track | RoadType::Path => 1.0,
        };
        base * avoid_track_path_factor(road_type, options)
            * avoid_motorway_factor(road_type, options)
    }

    fn edge_duration_s(self, edge: &EdgeCostInputs<'_>) -> f64 {
        profile_duration_s(edge, walking_speed_kph(edge.classes, edge.has_ford()))
    }
}

#[derive(Debug, Clone, Copy)]
pub struct LeggedPolicy;

impl LeggedPolicy {
    fn road_type_cost_factor(self, road_type: RoadType, options: &RouteCostingOptions) -> f64 {
        let base = match road_type {
            RoadType::Motorway => 10.0,
            RoadType::Primary | RoadType::Secondary => 2.0,
            RoadType::Tertiary => 1.5,
            RoadType::Residential | RoadType::Other => 0.9,
            RoadType::Track | RoadType::Path => 0.45,
        };
        base * avoid_track_path_factor(road_type, options)
            * avoid_motorway_factor(road_type, options)
    }

    fn edge_duration_s(self, edge: &EdgeCostInputs<'_>) -> f64 {
        profile_duration_s(edge, legged_speed_kph(edge.classes, edge.has_ford()))
    }
}

pub struct RoadTypeCost;

impl RoadTypeCost {
    fn vehicle_base(road_type: RoadType) -> f64 {
        road_type.cost_factor() as f64
    }
}

pub fn vehicle_base_speed_kph(edge: &EdgeCostInputs<'_>, options: &RouteCostingOptions) -> f64 {
    if let Some(speed) = normalized_speed_kph(options.fixed_speed_kph) {
        return speed.max(1.0);
    }
    if let Some(speed) = active_conditional_maxspeed_kph(edge.conditional_restrictions, options) {
        return normalized_speed_kph(options.top_speed_kph)
            .map(|top_speed| speed.min(top_speed))
            .unwrap_or(speed)
            .max(1.0);
    }
    let selected = selected_vehicle_speed_kph(edge, options.speed_preference)
        .unwrap_or_else(|| road_type_default_speed_kph(edge.road_type));
    normalized_speed_kph(options.top_speed_kph)
        .map(|top_speed| selected.min(top_speed))
        .unwrap_or(selected)
        .max(1.0)
}

pub fn selected_vehicle_speed_source(
    edge: &RoadEdge,
    info: &SharedEdgeInfo,
    options: &RouteCostingOptions,
) -> RouteSpeedSource {
    if normalized_speed_kph(options.fixed_speed_kph).is_some() {
        return RouteSpeedSource::Fixed;
    }
    if active_conditional_maxspeed_kph(&info.conditional_restrictions, options).is_some() {
        return RouteSpeedSource::Conditional;
    }
    let inputs = EdgeCostInputs::from_parts(edge, info, EdgeCostClasses::default());
    match selected_vehicle_speed_source_and_kph(&inputs, options.speed_preference) {
        Some((source, _)) => source,
        None => RouteSpeedSource::Default,
    }
}

pub fn selected_vehicle_speed_kph(
    edge: &EdgeCostInputs<'_>,
    preference: RouteSpeedPreference,
) -> Option<f64> {
    selected_vehicle_speed_source_and_kph(edge, preference).map(|(_, speed)| speed)
}

pub fn selected_vehicle_speed_source_and_kph(
    edge: &EdgeCostInputs<'_>,
    preference: RouteSpeedPreference,
) -> Option<(RouteSpeedSource, f64)> {
    let posted = positive_speed(edge.maxspeed_kph.map(f64::from));
    let inferred = positive_speed(edge.inferred_speed_kph.map(f64::from));
    let free_flow = positive_speed(edge.free_flow_speed_kph.map(f64::from));
    let constrained = positive_speed(edge.constrained_speed_kph.map(f64::from));

    match preference {
        RouteSpeedPreference::PostedOrInferred => posted
            .map(|speed| (RouteSpeedSource::Posted, speed))
            .or_else(|| inferred.map(|speed| (RouteSpeedSource::Inferred, speed))),
        RouteSpeedPreference::Posted => posted.map(|speed| (RouteSpeedSource::Posted, speed)),
        RouteSpeedPreference::Inferred => inferred.map(|speed| (RouteSpeedSource::Inferred, speed)),
        RouteSpeedPreference::FreeFlow => {
            free_flow.map(|speed| (RouteSpeedSource::FreeFlow, speed))
        }
        RouteSpeedPreference::Constrained => {
            constrained.map(|speed| (RouteSpeedSource::Constrained, speed))
        }
        RouteSpeedPreference::RoadTypeDefault => None,
    }
}

pub fn positive_speed(speed: Option<f64>) -> Option<f64> {
    speed.filter(|value| value.is_finite() && *value > 0.0)
}

/// What the active `*:conditional` access tags do to an edge for a profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ConditionalAccess {
    /// No active tag binds the profile, or none limits it.
    Unchanged,
    /// Open only to reach somewhere on it (`destination`, `private`, …), as
    /// a way with that static access is (see `EdgeCostFlags::DESTINATION_ONLY`).
    DestinationOnly,
    /// Closed (`no`, or `private` to everyone but motor vehicles).
    Closed,
}

/// The strongest effect of the `*:conditional` access tags active at the
/// options' evaluation time on `profile`; [`ConditionalAccess::Unchanged`]
/// without an evaluation time.
pub fn active_conditional_access(
    conditional_restrictions: &[String],
    options: &RouteCostingOptions,
    profile: ProfilePolicy,
) -> ConditionalAccess {
    let Some(timestamp_ms) = options.conditional_restriction_eval_at_ms else {
        return ConditionalAccess::Unchanged;
    };
    conditional_restrictions
        .iter()
        .flat_map(|restriction| parse_conditional_restrictions(restriction))
        .filter(|parsed| conditional_key_blocks_profile(&parsed.key, profile))
        .filter(|parsed| parsed_condition_active(parsed.condition.as_deref(), timestamp_ms))
        .map(|parsed| parsed.access_effect(profile))
        .max()
        .unwrap_or(ConditionalAccess::Unchanged)
}

/// Whether an active `*:conditional` tag closes the edge to `profile` (see
/// [`active_conditional_access`]).
pub fn active_conditional_access_block(
    conditional_restrictions: &[String],
    options: &RouteCostingOptions,
    profile: ProfilePolicy,
) -> bool {
    active_conditional_access(conditional_restrictions, options, profile)
        == ConditionalAccess::Closed
}

/// The first `*:conditional` tag whose closure (`no` at the options'
/// evaluation time) applies to `profile`, as reported by the SAR
/// `conditional` relaxation; `None` without an evaluation time.
pub fn active_conditional_access_closure<'r>(
    conditional_restrictions: &'r [String],
    options: &RouteCostingOptions,
    profile: MobilityProfile,
) -> Option<&'r str> {
    let timestamp_ms = options.conditional_restriction_eval_at_ms?;
    let profile = ProfilePolicy::from_profile(profile);
    conditional_restrictions
        .iter()
        .find(|restriction| {
            parse_conditional_restrictions(restriction)
                .into_iter()
                .any(|parsed| {
                    conditional_key_blocks_profile(&parsed.key, profile)
                        && parsed.access_effect(profile) == ConditionalAccess::Closed
                        && parsed_condition_active(parsed.condition.as_deref(), timestamp_ms)
                })
        })
        .map(String::as_str)
}

pub fn active_conditional_maxspeed_kph(
    conditional_restrictions: &[String],
    options: &RouteCostingOptions,
) -> Option<f64> {
    let timestamp_ms = options.conditional_restriction_eval_at_ms?;
    conditional_restrictions
        .iter()
        .flat_map(|restriction| parse_conditional_restrictions(restriction))
        .filter(|restriction| restriction.key == "maxspeed")
        .filter(|restriction| {
            parsed_condition_active(restriction.condition.as_deref(), timestamp_ms)
        })
        .filter_map(|restriction| parse_conditional_speed_kph(&restriction.value))
        .next()
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParsedConditionalRestriction {
    key: String,
    value: String,
    condition: Option<String>,
}

impl ParsedConditionalRestriction {
    /// What the value does to `profile` while active, as the same static
    /// access value does: `no` (and `military`, `agricultural`, `forestry`)
    /// closes; `private` and `customers` leave motor
    /// vehicles a destination-only way and close it to everyone else;
    /// `destination`, `delivery` and `permit` make it destination-only for
    /// vehicles.
    fn access_effect(&self, profile: ProfilePolicy) -> ConditionalAccess {
        match self.value.as_str() {
            "no" | "military" | "agricultural" | "forestry" => ConditionalAccess::Closed,
            "private" | "customers" if profile.is_vehicle_like() => {
                ConditionalAccess::DestinationOnly
            }
            "private" | "customers" => ConditionalAccess::Closed,
            "destination" | "delivery" | "permit" if profile.is_vehicle_like() => {
                ConditionalAccess::DestinationOnly
            }
            _ => ConditionalAccess::Unchanged,
        }
    }
}

pub fn parse_conditional_restrictions(value: &str) -> Vec<ParsedConditionalRestriction> {
    let mut restrictions = Vec::new();
    let mut inherited_key: Option<String> = None;
    for clause in split_outside_parentheses(value, ';')
        .map(str::trim)
        .filter(|clause| !clause.is_empty())
    {
        if let Some(parsed) = parse_conditional_restriction(clause) {
            inherited_key = Some(parsed.key.clone());
            restrictions.push(parsed);
        } else if let Some(key) = inherited_key.as_deref()
            && let Some(parsed) = parse_conditional_value_clause(key, clause)
        {
            restrictions.push(parsed);
        }
    }
    restrictions
}

/// `value` split at each `separator` outside parentheses: a condition in
/// parentheses may hold `;` itself (`no @ (Mo-Fr 07:00-09:00; Sa 08:00-12:00)`).
fn split_outside_parentheses(value: &str, separator: char) -> impl Iterator<Item = &str> {
    let mut depth = 0usize;
    value.split(move |ch: char| {
        match ch {
            '(' => depth += 1,
            ')' => depth = depth.saturating_sub(1),
            _ => {}
        }
        ch == separator && depth == 0
    })
}

pub fn parse_conditional_restriction(value: &str) -> Option<ParsedConditionalRestriction> {
    let (left, condition) = split_conditional_clause(value);
    let (key, raw_value) = left.split_once('=')?;
    let key = key
        .trim()
        .trim_end_matches(":conditional")
        .to_ascii_lowercase();
    if key.is_empty() {
        return None;
    }
    Some(ParsedConditionalRestriction {
        key,
        value: raw_value.trim().to_ascii_lowercase(),
        condition: condition
            .map(str::trim)
            .filter(|condition| !condition.is_empty())
            .map(ToOwned::to_owned),
    })
}

pub fn parse_conditional_value_clause(
    key: &str,
    value: &str,
) -> Option<ParsedConditionalRestriction> {
    let (raw_value, condition) = split_conditional_clause(value);
    let value = raw_value.trim().to_ascii_lowercase();
    if key.is_empty() || value.is_empty() {
        return None;
    }
    Some(ParsedConditionalRestriction {
        key: key.to_ascii_lowercase(),
        value,
        condition: condition
            .map(str::trim)
            .filter(|condition| !condition.is_empty())
            .map(ToOwned::to_owned),
    })
}

pub fn split_conditional_clause(value: &str) -> (&str, Option<&str>) {
    value
        .split_once('@')
        .map_or((value, None), |(left, condition)| {
            (
                left,
                Some(condition.trim().trim_matches(|ch| ch == '(' || ch == ')')),
            )
        })
}

/// Whether a conditional access key binds `profile`. The profiles are cars
/// (and other motor vehicles) and walkers: `bicycle`, `hgv`, `goods`, `bus`
/// and `psv` bind none of them.
pub fn conditional_key_blocks_profile(key: &str, profile: ProfilePolicy) -> bool {
    match key {
        "access" => true,
        "vehicle" | "motor_vehicle" | "motorcar" => profile.is_vehicle_like(),
        "foot" | "pedestrian" => profile.is_pedestrian_like(),
        _ => false,
    }
}

pub fn parse_conditional_speed_kph(value: &str) -> Option<f64> {
    let digits = value
        .split_whitespace()
        .next()
        .unwrap_or(value)
        .trim()
        .trim_end_matches("km/h")
        .trim_end_matches("kph");
    digits
        .parse::<f64>()
        .ok()
        .filter(|&speed| speed.is_finite() && speed > 0.0)
}

pub fn parsed_condition_active(condition: Option<&str>, timestamp_ms: i64) -> bool {
    let Some(condition) = condition.map(str::trim).filter(|value| !value.is_empty()) else {
        return true;
    };
    let Some(datetime) = chrono::DateTime::<chrono::Utc>::from_timestamp_millis(timestamp_ms)
    else {
        return false;
    };
    condition
        .split(';')
        .map(str::trim)
        .filter(|part| !part.is_empty())
        .any(|part| condition_part_active(part, datetime))
}

/// Whether one `;`-separated part of a condition holds at `datetime`. Each
/// `AND` term must hold; a term with no date, weekday or time (a vehicle
/// property such as `weight>7.5`, or `wet`) never holds, as the request has
/// no vehicle that could meet it.
pub fn condition_part_active(part: &str, datetime: chrono::DateTime<chrono::Utc>) -> bool {
    let words = part.split_whitespace().collect::<Vec<_>>();
    words
        .split(|word| *word == "AND")
        .all(|term| condition_term_active(&term.join(" "), datetime))
}

fn condition_term_active(part: &str, datetime: chrono::DateTime<chrono::Utc>) -> bool {
    let date_active = date_range_active(part, datetime.date_naive());
    let weekday_active = weekday_condition_active(part, datetime.weekday());
    let time_active = time_condition_active(part, datetime.hour() * 60 + datetime.minute());

    (date_active.unwrap_or(true) && weekday_active.unwrap_or(true) && time_active.unwrap_or(true))
        && (date_active.is_some() || weekday_active.is_some() || time_active.is_some())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct MonthDay {
    month: u32,
    day: u32,
}

pub fn date_range_active(part: &str, date: chrono::NaiveDate) -> Option<bool> {
    let month_names = [
        "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec",
    ];
    if !month_names.iter().any(|month| part.contains(month)) {
        return None;
    }
    let (start, end) = part.split_once('-')?;
    if let (Some(start), Some(end)) = (
        parse_osm_absolute_date(start.trim()),
        parse_osm_absolute_date(end.trim()),
    ) {
        return Some(date >= start && date <= end);
    }
    let start = parse_osm_month_day(start.trim())?;
    let end = parse_osm_month_day(end.trim())?;
    Some(month_day_in_range(
        MonthDay {
            month: date.month(),
            day: date.day(),
        },
        start,
        end,
    ))
}

pub fn parse_osm_absolute_date(value: &str) -> Option<chrono::NaiveDate> {
    chrono::NaiveDate::parse_from_str(value, "%Y %b %d")
        .or_else(|_| chrono::NaiveDate::parse_from_str(value, "%Y %B %d"))
        .ok()
}

pub fn parse_osm_month_day(value: &str) -> Option<MonthDay> {
    let date = chrono::NaiveDate::parse_from_str(&format!("2000 {value}"), "%Y %b %d")
        .or_else(|_| chrono::NaiveDate::parse_from_str(&format!("2000 {value}"), "%Y %B %d"))
        .ok()?;
    Some(MonthDay {
        month: date.month(),
        day: date.day(),
    })
}

pub fn month_day_in_range(day: MonthDay, start: MonthDay, end: MonthDay) -> bool {
    if start <= end {
        day >= start && day <= end
    } else {
        day >= start || day <= end
    }
}

pub fn weekday_condition_active(part: &str, day: Weekday) -> Option<bool> {
    let ranges = parse_weekday_ranges(part)?;
    Some(
        ranges
            .iter()
            .any(|(start, end)| weekday_in_range(day, *start, *end)),
    )
}

pub fn parse_weekday_ranges(part: &str) -> Option<Vec<(Weekday, Weekday)>> {
    let token = part.split_whitespace().find(|token| {
        token
            .split(',')
            .any(|segment| parse_weekday_segment(segment).is_some())
    })?;
    let ranges = token
        .split(',')
        .filter_map(parse_weekday_segment)
        .collect::<Vec<_>>();
    (!ranges.is_empty()).then_some(ranges)
}

pub fn parse_weekday_segment(value: &str) -> Option<(Weekday, Weekday)> {
    let value = value.trim_matches(|ch| matches!(ch, ',' | ';' | '(' | ')'));
    if let Some((start, end)) = value.split_once('-') {
        return Some((parse_weekday(start)?, parse_weekday(end)?));
    }
    let day = parse_weekday(value)?;
    Some((day, day))
}

pub fn parse_weekday(value: &str) -> Option<Weekday> {
    match value.trim_matches(|ch| matches!(ch, ',' | ';' | '(' | ')')) {
        "Mo" => Some(Weekday::Mon),
        "Tu" => Some(Weekday::Tue),
        "We" => Some(Weekday::Wed),
        "Th" => Some(Weekday::Thu),
        "Fr" => Some(Weekday::Fri),
        "Sa" => Some(Weekday::Sat),
        "Su" => Some(Weekday::Sun),
        _ => None,
    }
}

pub fn weekday_in_range(day: Weekday, start: Weekday, end: Weekday) -> bool {
    let day = day.num_days_from_monday();
    let start = start.num_days_from_monday();
    let end = end.num_days_from_monday();
    if start <= end {
        day >= start && day <= end
    } else {
        day >= start || day <= end
    }
}

pub fn time_condition_active(part: &str, minutes: u32) -> Option<bool> {
    let ranges = parse_time_ranges(part)?;
    Some(ranges.iter().any(|(start_minutes, end_minutes)| {
        if start_minutes <= end_minutes {
            minutes >= *start_minutes && minutes <= *end_minutes
        } else {
            minutes >= *start_minutes || minutes <= *end_minutes
        }
    }))
}

pub fn parse_time_ranges(part: &str) -> Option<Vec<(u32, u32)>> {
    let token = part.split_whitespace().find(|token| {
        token
            .split(',')
            .any(|segment| parse_time_segment(segment).is_some())
    })?;
    let ranges = token
        .split(',')
        .filter_map(parse_time_segment)
        .collect::<Vec<_>>();
    (!ranges.is_empty()).then_some(ranges)
}

pub fn parse_time_segment(value: &str) -> Option<(u32, u32)> {
    let value = value.trim_matches(|ch| matches!(ch, ',' | ';' | '(' | ')'));
    let (start, end) = value.split_once('-')?;
    Some((parse_time_minutes(start)?, parse_time_minutes(end)?))
}

pub fn parse_time_minutes(value: &str) -> Option<u32> {
    let (hour, minute) = value
        .trim_matches(|ch| matches!(ch, ',' | ';' | '(' | ')'))
        .split_once(':')?;
    let hour = hour.parse::<u32>().ok()?;
    let minute = minute.parse::<u32>().ok()?;
    // `24:00` is the end of the day (`22:00-24:00`).
    if (hour < 24 && minute < 60) || (hour, minute) == (24, 0) {
        Some(hour * 60 + minute)
    } else {
        None
    }
}

pub fn road_type_default_speed_kph(road_type: RoadType) -> f64 {
    match road_type {
        RoadType::Motorway => 90.0,
        RoadType::Primary => 75.0,
        RoadType::Secondary => 60.0,
        RoadType::Tertiary => 50.0,
        RoadType::Residential => 35.0,
        RoadType::Track => 18.0,
        RoadType::Path => 8.0,
        RoadType::Other => 30.0,
    }
}

pub fn wheeled_surface_speed_factor(surface: SurfaceCostClass, has_ford: bool) -> f64 {
    let mut factor = match surface {
        SurfaceCostClass::Paved => 1.0,
        SurfaceCostClass::Gravel => 0.75,
        SurfaceCostClass::Dirt => 0.55,
        SurfaceCostClass::Mud => 0.25,
        SurfaceCostClass::Sand => 0.3,
        SurfaceCostClass::Cobblestone => 0.65,
        SurfaceCostClass::Other => 1.0,
    };
    if has_ford {
        factor *= 0.65;
    }
    factor
}

pub fn tracked_surface_speed_factor(surface: SurfaceCostClass, has_ford: bool) -> f64 {
    let mut factor = match surface {
        SurfaceCostClass::Paved => 1.0,
        SurfaceCostClass::Gravel => 0.95,
        SurfaceCostClass::Dirt => 0.9,
        SurfaceCostClass::Mud => 0.55,
        SurfaceCostClass::Sand => 0.65,
        SurfaceCostClass::Cobblestone => 0.85,
        SurfaceCostClass::Other => 1.0,
    };
    if has_ford {
        factor *= 0.8;
    }
    factor
}

pub fn wheeled_tracktype_speed_factor(tracktype: TracktypeCostClass) -> f64 {
    match tracktype {
        TracktypeCostClass::Grade1 => 0.85,
        TracktypeCostClass::Grade2 => 0.7,
        TracktypeCostClass::Grade3 => 0.55,
        TracktypeCostClass::Grade4 => 0.4,
        TracktypeCostClass::Grade5 => 0.3,
        TracktypeCostClass::Other => 1.0,
    }
}

pub fn tracked_tracktype_speed_factor(tracktype: TracktypeCostClass) -> f64 {
    match tracktype {
        TracktypeCostClass::Grade1 => 1.0,
        TracktypeCostClass::Grade2 => 0.95,
        TracktypeCostClass::Grade3 => 0.85,
        TracktypeCostClass::Grade4 => 0.7,
        TracktypeCostClass::Grade5 => 0.55,
        TracktypeCostClass::Other => 1.0,
    }
}

pub fn profile_duration_s(edge: &EdgeCostInputs<'_>, speed_kph: f64) -> f64 {
    let speed_mps = speed_kph.max(1.0) * 1000.0 / 3600.0;
    f64::from(edge.distance_m).max(0.0) / speed_mps
}

pub fn walking_speed_kph(classes: EdgeCostClasses, has_ford: bool) -> f64 {
    // Valhalla's pedestrian default is 5.1 km/h.
    let mut speed = match classes.walkway {
        WalkwayCostClass::Steps => 2.8,
        WalkwayCostClass::Footpath | WalkwayCostClass::Other => 5.1,
        WalkwayCostClass::Track => 4.8,
    };
    speed *= pedestrian_surface_speed_factor(classes, has_ford);
    speed.clamp(1.5, ONFOOT_MAX_SPEED_KPH)
}

pub fn legged_speed_kph(classes: EdgeCostClasses, has_ford: bool) -> f64 {
    let mut speed = match classes.walkway {
        WalkwayCostClass::Steps => 3.2,
        WalkwayCostClass::Footpath => 6.0,
        WalkwayCostClass::Track => 6.5,
        WalkwayCostClass::Other => 5.8,
    };
    speed *= pedestrian_surface_speed_factor(classes, has_ford);
    speed.clamp(2.0, LEGGED_MAX_SPEED_KPH)
}

pub fn pedestrian_surface_speed_factor(classes: EdgeCostClasses, has_ford: bool) -> f64 {
    let mut factor = 1.0;
    factor *= match classes.surface {
        SurfaceCostClass::Gravel => 0.95,
        SurfaceCostClass::Dirt => 0.9,
        SurfaceCostClass::Mud => 0.65,
        SurfaceCostClass::Sand => 0.7,
        SurfaceCostClass::Cobblestone => 0.9,
        SurfaceCostClass::Paved | SurfaceCostClass::Other => 1.0,
    };
    factor *= match classes.smoothness {
        SmoothnessCostClass::Bad => 0.85,
        SmoothnessCostClass::VeryBad => 0.7,
        SmoothnessCostClass::Horrible => 0.5,
        SmoothnessCostClass::Good
        | SmoothnessCostClass::Intermediate
        | SmoothnessCostClass::Other => 1.0,
    };
    if has_ford {
        factor *= 0.7;
    }
    factor
}

pub fn avoid_track_path_factor(road_type: RoadType, options: &RouteCostingOptions) -> f64 {
    match road_type {
        RoadType::Track => 1.0 + bounded(options.avoid_tracks) * 4.0,
        RoadType::Path => 1.0 + bounded(options.avoid_paths) * 4.0,
        _ => 1.0,
    }
}

pub fn avoid_motorway_factor(road_type: RoadType, options: &RouteCostingOptions) -> f64 {
    match road_type {
        RoadType::Motorway => 1.0 + bounded(options.avoid_motorways) * 8.0,
        _ => 1.0,
    }
}

pub fn surface_factor(surface: SurfaceCostClass) -> f64 {
    match surface {
        SurfaceCostClass::Paved => 0.95,
        SurfaceCostClass::Gravel => 1.15,
        SurfaceCostClass::Dirt => 1.35,
        SurfaceCostClass::Mud => 2.2,
        SurfaceCostClass::Sand => 2.0,
        SurfaceCostClass::Cobblestone => 1.2,
        SurfaceCostClass::Other => 1.0,
    }
}

pub fn prefer_paved_surface_factor(
    surface: SurfaceCostClass,
    options: &RouteCostingOptions,
) -> f64 {
    let preference = bounded(options.prefer_paved);
    if preference <= f64::EPSILON {
        return 1.0;
    }

    match surface {
        SurfaceCostClass::Paved => 1.0 - preference * 0.08,
        SurfaceCostClass::Cobblestone => 1.0 + preference * 0.25,
        SurfaceCostClass::Gravel => 1.0 + preference * 0.6,
        SurfaceCostClass::Dirt => 1.0 + preference * 1.2,
        SurfaceCostClass::Mud => 1.0 + preference * 2.2,
        SurfaceCostClass::Sand => 1.0 + preference * 1.8,
        SurfaceCostClass::Other => 1.0,
    }
}

pub fn tracktype_factor(tracktype: TracktypeCostClass) -> f64 {
    match tracktype {
        TracktypeCostClass::Grade1 => 0.95,
        TracktypeCostClass::Grade2 => 1.05,
        TracktypeCostClass::Grade3 => 1.2,
        TracktypeCostClass::Grade4 => 1.45,
        TracktypeCostClass::Grade5 => 1.8,
        TracktypeCostClass::Other => 1.0,
    }
}

pub fn smoothness_factor(smoothness: SmoothnessCostClass) -> f64 {
    match smoothness {
        SmoothnessCostClass::Good => 0.95,
        SmoothnessCostClass::Intermediate => 1.1,
        SmoothnessCostClass::Bad => 1.3,
        SmoothnessCostClass::VeryBad => 1.55,
        SmoothnessCostClass::Horrible => 2.0,
        SmoothnessCostClass::Other => 1.0,
    }
}

pub fn bridge_tunnel_factor(is_tunnel: bool) -> f64 {
    let mut factor = 1.0;
    if is_tunnel {
        factor *= 1.05;
    }
    factor
}

pub fn bounded(value: f64) -> f64 {
    if value.is_finite() {
        value.clamp(0.0, 1.0)
    } else {
        0.0
    }
}

pub fn heading_delta(a: u16, b: u16) -> u16 {
    let raw = a.abs_diff(b);
    raw.min(360 - raw)
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteCostingOptions {
    #[serde(default)]
    pub avoid_tracks: f64,
    #[serde(default)]
    pub avoid_paths: f64,
    #[serde(default)]
    pub avoid_motorways: f64,
    #[serde(default)]
    pub avoid_destination_only: f64,
    #[serde(default = "default_allow_ford")]
    pub allow_ford: bool,
    #[serde(default)]
    pub avoid_tolls: f64,
    #[serde(default)]
    pub avoid_ferries: f64,
    #[serde(default)]
    pub avoid_gates: f64,
    #[serde(default)]
    pub avoid_access_restricted: f64,
    #[serde(default)]
    pub prefer_paved: f64,
    #[serde(default)]
    pub prefer_known_route_refs: f64,
    #[serde(default)]
    pub emergency_override_strength: f64,
    #[serde(default)]
    pub speed_preference: RouteSpeedPreference,
    #[serde(default)]
    pub fixed_speed_kph: Option<f64>,
    #[serde(default)]
    pub top_speed_kph: Option<f64>,
    #[serde(default)]
    pub conditional_restriction_eval_at_ms: Option<i64>,
}

impl Default for RouteCostingOptions {
    fn default() -> Self {
        Self {
            avoid_tracks: 0.0,
            avoid_paths: 0.0,
            avoid_motorways: 0.0,
            avoid_destination_only: 0.0,
            allow_ford: true,
            avoid_tolls: 0.0,
            avoid_ferries: 0.0,
            avoid_gates: 0.0,
            avoid_access_restricted: 0.0,
            prefer_paved: 0.0,
            prefer_known_route_refs: 0.0,
            emergency_override_strength: 0.0,
            speed_preference: RouteSpeedPreference::default(),
            fixed_speed_kph: None,
            top_speed_kph: None,
            conditional_restriction_eval_at_ms: None,
        }
    }
}

impl RouteCostingOptions {
    pub fn normalized(self) -> Self {
        Self {
            avoid_tracks: bounded_unit(self.avoid_tracks),
            avoid_paths: bounded_unit(self.avoid_paths),
            avoid_motorways: bounded_unit(self.avoid_motorways),
            avoid_destination_only: bounded_unit(self.avoid_destination_only),
            allow_ford: self.allow_ford,
            avoid_tolls: bounded_unit(self.avoid_tolls),
            avoid_ferries: bounded_unit(self.avoid_ferries),
            avoid_gates: bounded_unit(self.avoid_gates),
            avoid_access_restricted: bounded_unit(self.avoid_access_restricted),
            prefer_paved: bounded_unit(self.prefer_paved),
            prefer_known_route_refs: bounded_unit(self.prefer_known_route_refs),
            emergency_override_strength: bounded_unit(self.emergency_override_strength),
            speed_preference: self.speed_preference,
            fixed_speed_kph: normalized_speed_kph(self.fixed_speed_kph),
            top_speed_kph: normalized_speed_kph(self.top_speed_kph),
            conditional_restriction_eval_at_ms: self
                .conditional_restriction_eval_at_ms
                .filter(|value| *value >= 0),
        }
    }
}

fn default_allow_ford() -> bool {
    true
}

pub fn normalized_speed_kph(speed: Option<f64>) -> Option<f64> {
    speed
        .filter(|value| value.is_finite() && *value > 0.0)
        .map(|value| value.clamp(1.0, 300.0))
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteSpeedPreference {
    #[default]
    PostedOrInferred,
    Posted,
    Inferred,
    FreeFlow,
    Constrained,
    RoadTypeDefault,
}

impl RouteSpeedPreference {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::PostedOrInferred => "posted_or_inferred",
            Self::Posted => "posted",
            Self::Inferred => "inferred",
            Self::FreeFlow => "free_flow",
            Self::Constrained => "constrained",
            Self::RoadTypeDefault => "road_type_default",
        }
    }
}

fn default_cost_factor() -> f64 {
    1.0
}

fn bounded_unit(value: f64) -> f64 {
    if value.is_finite() {
        value.clamp(0.0, 1.0)
    } else {
        0.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteCostBreakdown {
    pub base_distance_cost: f64,
    pub speed_time_s: f64,
    #[serde(default = "default_cost_factor")]
    pub profile_time_factor: f64,
    pub road_type_factor: f64,
    pub surface_factor: f64,
    pub tracktype_factor: f64,
    pub smoothness_factor: f64,
    pub ford_factor: f64,
    pub bridge_tunnel_factor: f64,
    #[serde(default = "default_cost_factor")]
    pub toll_penalty: f64,
    #[serde(default = "default_cost_factor")]
    pub ferry_penalty: f64,
    #[serde(default = "default_cost_factor")]
    pub gate_penalty: f64,
    #[serde(default = "default_cost_factor")]
    pub access_restriction_penalty: f64,
    pub destination_only_penalty: f64,
    pub avoided_edge_penalty: f64,
    pub overlay_multiplier: f64,
    pub route_ref_factor: f64,
    /// The SAR relaxation's factor on an edge the profile may not legally
    /// use (1 elsewhere).
    #[serde(default = "default_cost_factor")]
    pub sar_factor: f64,
    pub turn_penalty: f64,
    pub total_cost: f64,
}

impl RouteCostBreakdown {
    pub fn edge_cost_without_turn(self) -> f64 {
        self.base_distance_cost
            * self.profile_time_factor
            * self.road_type_factor
            * self.surface_factor
            * self.tracktype_factor
            * self.smoothness_factor
            * self.ford_factor
            * self.bridge_tunnel_factor
            * self.toll_penalty
            * self.ferry_penalty
            * self.gate_penalty
            * self.access_restriction_penalty
            * self.destination_only_penalty
            * self.avoided_edge_penalty
            * self.overlay_multiplier
            * self.route_ref_factor
            * self.sar_factor
    }

    /// Phase 0 time-dominated metric.
    ///
    /// The base cost term is now real travel time in seconds, not distance. We
    /// keep `base_distance_cost` = `distance_m` for truthful reporting/attribution
    /// and express the time as a per-meter factor so that
    ///   `base_distance_cost * profile_time_factor == speed_time_s` (duration_s).
    /// The whole edge cost therefore becomes `duration_s * penalty_factors`.
    ///
    /// This deliberately removes the old `clamp(1.0, 12.0)` behavior, whose 1.0
    /// floor made every edge at >= 30 km/h cost the same per meter (speed above
    /// 30 km/h was free). The factor is now the true seconds-per-meter of the
    /// edge (`1 / speed_mps`), so a faster edge yields a smaller factor and hence
    /// strictly lower cost per meter.
    pub fn calculate_profile_time_factor(base_distance_cost: f64, speed_time_s: f64) -> f64 {
        if !base_distance_cost.is_finite()
            || !speed_time_s.is_finite()
            || base_distance_cost <= f64::EPSILON
            || speed_time_s <= f64::EPSILON
        {
            return 1.0;
        }

        speed_time_s / base_distance_cost
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteSpeedSource {
    Posted,
    Inferred,
    FreeFlow,
    Constrained,
    Conditional,
    Fixed,
    Default,
}

impl Default for RouteCostBreakdown {
    fn default() -> Self {
        Self {
            base_distance_cost: 0.0,
            speed_time_s: 0.0,
            profile_time_factor: 1.0,
            road_type_factor: 1.0,
            surface_factor: 1.0,
            tracktype_factor: 1.0,
            smoothness_factor: 1.0,
            ford_factor: 1.0,
            bridge_tunnel_factor: 1.0,
            toll_penalty: 1.0,
            ferry_penalty: 1.0,
            gate_penalty: 1.0,
            access_restriction_penalty: 1.0,
            destination_only_penalty: 1.0,
            avoided_edge_penalty: 1.0,
            overlay_multiplier: 1.0,
            route_ref_factor: 1.0,
            sar_factor: 1.0,
            turn_penalty: 0.0,
            total_cost: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CAR: ProfilePolicy = ProfilePolicy::Wheeled(WheeledPolicy);
    const WALKER: ProfilePolicy = ProfilePolicy::OnFoot(OnFootPolicy);

    /// Options evaluating conditionals on 2026-11-`day` (the 18th is a
    /// Wednesday, the 21st a Saturday) at `hour`:`minute` UTC.
    fn at(day: u32, hour: u32, minute: u32) -> RouteCostingOptions {
        let time = chrono::NaiveDate::from_ymd_opt(2026, 11, day)
            .unwrap()
            .and_hms_opt(hour, minute, 0)
            .unwrap()
            .and_utc();
        RouteCostingOptions {
            conditional_restriction_eval_at_ms: Some(time.timestamp_millis()),
            ..RouteCostingOptions::default()
        }
    }

    fn access(
        tag: &str,
        options: &RouteCostingOptions,
        profile: ProfilePolicy,
    ) -> ConditionalAccess {
        active_conditional_access(&[tag.to_string()], options, profile)
    }

    #[test]
    fn conditional_keys_bind_only_the_profiles_they_name() {
        let options = at(18, 8, 0);
        let always = "@ (Mo-Su 00:00-24:00)";
        for (key, car, walker) in [
            (
                "access",
                ConditionalAccess::Closed,
                ConditionalAccess::Closed,
            ),
            (
                "vehicle",
                ConditionalAccess::Closed,
                ConditionalAccess::Unchanged,
            ),
            (
                "motor_vehicle",
                ConditionalAccess::Closed,
                ConditionalAccess::Unchanged,
            ),
            (
                "motorcar",
                ConditionalAccess::Closed,
                ConditionalAccess::Unchanged,
            ),
            (
                "foot",
                ConditionalAccess::Unchanged,
                ConditionalAccess::Closed,
            ),
            (
                "bicycle",
                ConditionalAccess::Unchanged,
                ConditionalAccess::Unchanged,
            ),
            (
                "hgv",
                ConditionalAccess::Unchanged,
                ConditionalAccess::Unchanged,
            ),
            (
                "goods",
                ConditionalAccess::Unchanged,
                ConditionalAccess::Unchanged,
            ),
            (
                "bus",
                ConditionalAccess::Unchanged,
                ConditionalAccess::Unchanged,
            ),
            (
                "psv",
                ConditionalAccess::Unchanged,
                ConditionalAccess::Unchanged,
            ),
        ] {
            let tag = format!("{key}:conditional=no {always}");
            assert_eq!(access(&tag, &options, CAR), car, "{tag} for cars");
            assert_eq!(access(&tag, &options, WALKER), walker, "{tag} for walkers");
        }
    }

    #[test]
    fn destination_and_private_limit_vehicles_to_destination_traffic() {
        let weekend = "motor_vehicle:conditional=destination @ (Sa,Su)";
        assert_eq!(
            access(weekend, &at(21, 12, 0), CAR),
            ConditionalAccess::DestinationOnly
        );
        assert_eq!(
            access(weekend, &at(18, 12, 0), CAR),
            ConditionalAccess::Unchanged
        );
        assert_eq!(
            access(weekend, &at(21, 12, 0), WALKER),
            ConditionalAccess::Unchanged
        );
        // As static `access=private`: motor vehicles may reach somewhere on
        // the way, everyone else is kept out.
        let private = "access:conditional=private @ (Mo-Fr 06:00-18:00)";
        assert_eq!(
            access(private, &at(18, 8, 0), CAR),
            ConditionalAccess::DestinationOnly
        );
        assert_eq!(
            access(private, &at(18, 8, 0), WALKER),
            ConditionalAccess::Closed
        );
        let delivery = "access:conditional=delivery @ (Mo-Fr 06:00-18:00)";
        assert_eq!(
            access(delivery, &at(18, 8, 0), CAR),
            ConditionalAccess::DestinationOnly
        );
        assert_eq!(
            access(delivery, &at(18, 8, 0), WALKER),
            ConditionalAccess::Unchanged
        );
        // Neither counts as a closure for the SAR `conditional` relaxation.
        assert_eq!(
            active_conditional_access_closure(
                &[weekend.to_string()],
                &at(21, 12, 0),
                MobilityProfile::Wheeled
            ),
            None
        );
    }

    #[test]
    fn the_strongest_active_clause_wins() {
        let tag = "access:conditional=destination @ (Mo-Fr); no @ (Mo-Fr 07:00-09:00)";
        assert_eq!(access(tag, &at(18, 8, 0), CAR), ConditionalAccess::Closed);
        assert_eq!(
            access(tag, &at(18, 12, 0), CAR),
            ConditionalAccess::DestinationOnly
        );
        assert_eq!(
            access(tag, &at(21, 8, 0), CAR),
            ConditionalAccess::Unchanged
        );
    }

    #[test]
    fn semicolons_inside_parentheses_separate_condition_parts_not_clauses() {
        let tag = "access:conditional=no @ (Mo-Fr 07:00-09:00; Sa 10:00-12:00); destination @ (Su)";
        let parsed = parse_conditional_restrictions(tag);
        assert_eq!(
            parsed
                .iter()
                .map(|parsed| (
                    parsed.key.as_str(),
                    parsed.value.as_str(),
                    parsed.condition.as_deref()
                ))
                .collect::<Vec<_>>(),
            [
                ("access", "no", Some("Mo-Fr 07:00-09:00; Sa 10:00-12:00")),
                ("access", "destination", Some("Su")),
            ]
        );
        assert_eq!(access(tag, &at(18, 8, 0), CAR), ConditionalAccess::Closed);
        assert_eq!(access(tag, &at(21, 11, 0), CAR), ConditionalAccess::Closed);
        assert_eq!(
            access(tag, &at(21, 13, 0), CAR),
            ConditionalAccess::Unchanged
        );
        assert_eq!(
            access(tag, &at(22, 13, 0), CAR),
            ConditionalAccess::DestinationOnly
        );
    }

    #[test]
    fn every_and_term_must_hold() {
        // No request vehicle weighs over 7.5 t, so the closure never binds.
        let heavy = "motor_vehicle:conditional=no @ (weight>7.5 AND Mo-Fr 06:00-22:00)";
        assert_eq!(
            access(heavy, &at(18, 8, 0), CAR),
            ConditionalAccess::Unchanged
        );
        let timed = "motor_vehicle:conditional=no @ (Mo-Fr AND 06:00-22:00)";
        assert_eq!(access(timed, &at(18, 8, 0), CAR), ConditionalAccess::Closed);
        assert_eq!(
            access(timed, &at(18, 23, 0), CAR),
            ConditionalAccess::Unchanged
        );
        assert_eq!(
            access(timed, &at(21, 8, 0), CAR),
            ConditionalAccess::Unchanged
        );
    }

    #[test]
    fn midnight_may_be_written_as_24_00() {
        assert_eq!(parse_time_minutes("24:00"), Some(1440));
        assert_eq!(parse_time_minutes("24:30"), None);
        assert_eq!(parse_time_minutes("25:00"), None);
        let late = "access:conditional=no @ (22:00-24:00)";
        assert_eq!(
            access(late, &at(18, 23, 30), CAR),
            ConditionalAccess::Closed
        );
        assert_eq!(
            access(late, &at(18, 21, 30), CAR),
            ConditionalAccess::Unchanged
        );
    }

    #[test]
    fn no_evaluation_time_changes_nothing() {
        let tag = "access:conditional=no @ (Mo-Su 00:00-24:00)";
        assert_eq!(
            access(tag, &RouteCostingOptions::default(), CAR),
            ConditionalAccess::Unchanged
        );
    }
}

//! OSM tag interpretation: mode masks from highway and access tags, oneway,
//! barriers, physical limits, speed limits, layers and way facts.

use super::*;

pub(crate) fn road_surface_factor(way: &RoutableWay) -> f32 {
    let mut factor = 1.0f32;

    factor *= match way.surface.as_deref() {
        Some("asphalt" | "paved" | "concrete" | "concrete:plates" | "concrete:lanes") => 0.95,
        Some("gravel" | "compacted" | "fine_gravel") => 1.15,
        Some("dirt" | "earth" | "ground") => 1.35,
        Some("mud") => 2.2,
        Some("sand") => 2.0,
        Some("cobblestone" | "sett" | "paving_stones") => 1.2,
        _ => 1.0,
    };

    factor *= match way.tracktype.as_deref() {
        Some("grade1") => 0.95,
        Some("grade2") => 1.05,
        Some("grade3") => 1.2,
        Some("grade4") => 1.45,
        Some("grade5") => 1.8,
        _ => 1.0,
    };

    factor *= match way.smoothness.as_deref() {
        Some("excellent" | "good") => 0.95,
        Some("intermediate") => 1.1,
        Some("bad") => 1.3,
        Some("very_bad") => 1.55,
        Some("horrible" | "very_horrible" | "impassable") => 2.0,
        _ => 1.0,
    };

    if way.has_ford {
        factor *= 1.4;
    }
    if way.has_tunnel {
        factor *= 1.05;
    }

    factor
}

pub(crate) fn derive_osm_mode_masks_from_way(way: &RoutableWay) -> (ModeMask, ModeMask) {
    let base_mask = derive_osm_mode_mask(
        way.highway.as_deref(),
        way.access.as_deref(),
        way.vehicle.as_deref(),
        way.motor_vehicle.as_deref(),
        way.motorcar.as_deref(),
        way.bicycle.as_deref(),
        way.foot.as_deref(),
        way.motorroad.as_deref(),
    );
    split_directional_masks(
        base_mask,
        way.directional_access,
        way.oneway_foot.as_deref(),
        way.oneway_bicycle.as_deref(),
    )
}

pub(crate) const MOTOR_MODES: ModeMask =
    OsmMode::MOTOR_VEHICLE | OsmMode::BUS | OsmMode::HGV | OsmMode::EMERGENCY;

/// The modes a way admits by its `highway` value alone, before its access
/// tags, after the OSM worldwide defaults as OSRM and Valhalla apply them:
///
/// | `highway` | modes |
/// |---|---|
/// | `motorway`, `motorway_link` | motor vehicles |
/// | `trunk`, `primary`, `secondary`, `tertiary` (and their links), `unclassified`, `residential`, `living_street`, `service`, `track`, `road` | all |
/// | `path` | foot, bicycle |
/// | `cycleway` | bicycle, foot |
/// | `footway`, `pedestrian`, `steps`, `corridor`, `bridleway` | foot |
/// | `busway` | bus, emergency |
/// | `construction`, `proposed`, `abandoned`, `razed`, `platform` and any other value | none |
///
/// A way with no `highway` (a ferry, or a way built directly) admits every
/// mode. `motorroad=yes` removes foot and bicycle as a motorway does. Ways with
/// no default mode are kept only when a mode-specific tag grants one.
pub fn highway_default_mode_mask(highway: Option<&str>, motorroad: Option<&str>) -> ModeMask {
    let mask = match highway.map(str::trim) {
        None => OsmMode::ALL,
        Some("motorway" | "motorway_link") => MOTOR_MODES,
        Some(
            "trunk" | "trunk_link" | "primary" | "primary_link" | "secondary" | "secondary_link"
            | "tertiary" | "tertiary_link" | "unclassified" | "residential" | "living_street"
            | "service" | "track" | "road",
        ) => OsmMode::ALL,
        Some("path" | "cycleway") => OsmMode::FOOT | OsmMode::BICYCLE,
        Some("footway" | "pedestrian" | "steps" | "corridor" | "bridleway") => OsmMode::FOOT,
        Some("busway") => OsmMode::BUS | OsmMode::EMERGENCY,
        Some(_) => 0,
    };
    if oneway_value_is_yes(motorroad) {
        mask & !(OsmMode::FOOT | OsmMode::BICYCLE)
    } else {
        mask
    }
}

/// A way's modes: its [`highway_default_mode_mask`], then its access tags from
/// the least to the most specific. The generic `access` and `vehicle` grants
/// restore at most the highway's defaults (so `access=yes` opens neither a
/// footway to cars nor a motorway to pedestrians); `motor_vehicle`,
/// `motorcar`, `bicycle` and `foot` grant their modes outright. Any of them
/// denies its modes.
pub fn derive_osm_mode_mask(
    highway: Option<&str>,
    access: Option<&str>,
    vehicle: Option<&str>,
    motor_vehicle: Option<&str>,
    motorcar: Option<&str>,
    bicycle: Option<&str>,
    foot: Option<&str>,
    motorroad: Option<&str>,
) -> ModeMask {
    let defaults = highway_default_mode_mask(highway, motorroad);
    let mut mask = defaults;

    mask = apply_osm_access_override(mask, access, AccessScope::All, defaults);
    mask = apply_osm_access_override(mask, vehicle, AccessScope::Vehicle, defaults);
    mask = apply_osm_access_override(mask, motor_vehicle, AccessScope::MotorVehicle, OsmMode::ALL);
    mask = apply_osm_access_override(mask, motorcar, AccessScope::MotorVehicle, OsmMode::ALL);
    mask = apply_osm_access_override(mask, bicycle, AccessScope::Bicycle, OsmMode::ALL);
    mask = apply_osm_access_override(mask, foot, AccessScope::Foot, OsmMode::ALL);

    mask
}

/// Direction a mode-specific one-way tag (`oneway:foot`, `oneway:bicycle`) grants.
#[derive(Clone, Copy, PartialEq, Eq)]
enum OnewayDirection {
    Both,
    Forward,
    Backward,
}

fn parse_oneway_direction(value: Option<&str>) -> Option<OnewayDirection> {
    match value.map(str::trim) {
        Some("yes" | "true" | "1") => Some(OnewayDirection::Forward),
        Some("-1" | "reverse") => Some(OnewayDirection::Backward),
        Some("no" | "false" | "0") => Some(OnewayDirection::Both),
        _ => None,
    }
}

pub(crate) fn oneway_value_is_yes(value: Option<&str>) -> bool {
    matches!(value.map(str::trim), Some("yes" | "true" | "1"))
}

/// Split a bidirectional mode mask into forward/reverse directed masks.
///
/// Vehicular modes honour the way's `oneway` direction. Pedestrians may walk
/// against a vehicular one-way unless the footway itself is directed via
/// `oneway:foot`, so the `FOOT` bit is preserved in both directions by default.
/// Bicycles follow the vehicular one-way unless `oneway:bicycle` overrides it
/// (`no` grants contraflow both ways; `yes`/`-1` pin a single direction).
pub(crate) fn split_directional_masks(
    base_mask: ModeMask,
    access: DirectionalAccess,
    foot_oneway: Option<&str>,
    bicycle_oneway: Option<&str>,
) -> (ModeMask, ModeMask) {
    let mut forward = 0;
    let mut reverse = 0;

    let foot_bit = base_mask & OsmMode::FOOT;
    let (foot_forward, foot_reverse) = match parse_oneway_direction(foot_oneway) {
        Some(OnewayDirection::Forward) => (true, false),
        Some(OnewayDirection::Backward) => (false, true),
        Some(OnewayDirection::Both) | None => (true, true),
    };
    if foot_forward {
        forward |= foot_bit;
    }
    if foot_reverse {
        reverse |= foot_bit;
    }

    let bicycle_bit = base_mask & OsmMode::BICYCLE;
    let (bicycle_forward, bicycle_reverse) = match parse_oneway_direction(bicycle_oneway) {
        Some(OnewayDirection::Both) => (true, true),
        Some(OnewayDirection::Forward) => (true, false),
        Some(OnewayDirection::Backward) => (false, true),
        None => (access.allows_forward(), access.allows_backward()),
    };
    if bicycle_forward {
        forward |= bicycle_bit;
    }
    if bicycle_reverse {
        reverse |= bicycle_bit;
    }

    let vehicular = base_mask & !(OsmMode::FOOT | OsmMode::BICYCLE);
    if access.allows_forward() {
        forward |= vehicular;
    }
    if access.allows_backward() {
        reverse |= vehicular;
    }

    (forward, reverse)
}

#[derive(Clone, Copy)]
enum AccessScope {
    All,
    Vehicle,
    MotorVehicle,
    Bicycle,
    Foot,
}

/// Applies one access tag: a grant adds the scope's modes within `grantable`,
/// a denial removes them all, and a destination-only value (`private`,
/// `customers`) grants the scope's motor modes (the way is then flagged
/// destination-only, see [`vehicle_access_is_destination_only`]) and removes
/// the rest.
fn apply_osm_access_override(
    current: ModeMask,
    value: Option<&str>,
    scope: AccessScope,
    grantable: ModeMask,
) -> ModeMask {
    let Some(value) = value.map(str::trim).filter(|value| !value.is_empty()) else {
        return current;
    };
    let scope_mask = match scope {
        AccessScope::All => OsmMode::ALL,
        AccessScope::Vehicle => {
            OsmMode::MOTOR_VEHICLE | OsmMode::BICYCLE | OsmMode::BUS | OsmMode::HGV
        }
        AccessScope::MotorVehicle => {
            OsmMode::MOTOR_VEHICLE | OsmMode::BUS | OsmMode::HGV | OsmMode::EMERGENCY
        }
        AccessScope::Bicycle => OsmMode::BICYCLE,
        AccessScope::Foot => OsmMode::FOOT,
    };
    if osm_access_allows(value) {
        current | (scope_mask & grantable)
    } else if osm_access_limits_to_destination(value) {
        (current | (scope_mask & MOTOR_MODES & grantable)) & !(scope_mask & !MOTOR_MODES)
    } else if osm_access_denies(value) {
        current & !scope_mask
    } else {
        current
    }
}

fn osm_access_allows(value: &str) -> bool {
    matches!(
        value,
        "yes" | "designated" | "official" | "destination" | "permit" | "permissive" | "delivery"
    )
}

fn osm_access_denies(value: &str) -> bool {
    matches!(value, "no" | "military" | "agricultural" | "forestry")
}

/// Values that open a way to motor vehicles going somewhere on it, and close
/// it to everyone else.
fn osm_access_limits_to_destination(value: &str) -> bool {
    matches!(value, "private" | "customers")
}

/// Whether a way's access tags let motor vehicles in only to reach somewhere
/// on it, never through it (Valhalla's destination-only roads): the most
/// specific tag present among `motorcar`, `motor_vehicle`, `vehicle` and
/// `access` is `destination`, `private`, `customers`, `permit` or
/// `delivery`.
pub fn vehicle_access_is_destination_only(
    motorcar: Option<&str>,
    motor_vehicle: Option<&str>,
    vehicle: Option<&str>,
    access: Option<&str>,
) -> bool {
    [motorcar, motor_vehicle, vehicle, access]
        .into_iter()
        .flatten()
        .map(str::trim)
        .find(|value| !value.is_empty())
        .is_some_and(|value| {
            matches!(
                value,
                "destination" | "private" | "customers" | "permit" | "delivery"
            )
        })
}

/// Per-mode passability a physical barrier node imposes on the way threading
/// through it. `passable_mask` holds the OSM mode bits that may still cross;
/// `conditional_mask` (always a subset) holds the bits that cross only with a
/// gate penalty. Bits absent from `passable_mask` are hard-blocked.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NodeBarrier {
    pub passable_mask: ModeMask,
    pub conditional_mask: ModeMask,
}

impl NodeBarrier {
    /// True when some mode is hard-blocked (a mask bit is cleared).
    pub fn blocks_any(&self) -> bool {
        self.passable_mask & OsmMode::ALL != OsmMode::ALL
    }

    /// True when some passing mode is only conditionally passable (penalized).
    pub fn has_conditional(&self) -> bool {
        self.conditional_mask != 0
    }

    /// True when the barrier changes routability at all. Recognised values that
    /// neither block nor penalize any mode (e.g. `cattle_grid`, `kerb`) have no
    /// effect and are not materialised as crossings.
    pub fn has_effect(&self) -> bool {
        self.blocks_any() || self.has_conditional()
    }

    /// The barrier's effect per profile in normal routing: Default, Wheeled
    /// and Tracked follow motor vehicles, Emergency emergency vehicles,
    /// OnFoot and Legged walkers.
    pub fn profile_access(&self) -> wayfarer_package::barrier::BarrierAccess {
        use wayfarer_package::MobilityProfile as P;
        use wayfarer_package::format::profile_bit;
        let (mut closed, mut gate) = (0u8, 0u8);
        for profile in [
            P::Default,
            P::Wheeled,
            P::Tracked,
            P::Emergency,
            P::OnFoot,
            P::Legged,
        ] {
            let mode = match profile {
                P::Default | P::Wheeled | P::Tracked => OsmMode::MOTOR_VEHICLE,
                P::Emergency => OsmMode::EMERGENCY,
                P::OnFoot | P::Legged => OsmMode::FOOT,
            };
            if self.passable_mask & mode == 0 {
                closed |= profile_bit(profile);
            } else if self.conditional_mask & mode != 0 {
                gate |= profile_bit(profile);
            }
        }
        wayfarer_package::barrier::BarrierAccess::new(closed, gate)
    }
}

/// Raw barrier-relevant tags read from an OSM node, mirroring the way-level
/// access fields consumed by [`derive_osm_mode_mask`].
#[derive(Debug, Clone, Default)]
pub struct NodeBarrierTags {
    pub barrier: String,
    pub access: Option<String>,
    pub foot: Option<String>,
    pub bicycle: Option<String>,
    pub vehicle: Option<String>,
    pub motor_vehicle: Option<String>,
    pub locked: Option<String>,
}

const BARRIER_VEHICLE_MASK: ModeMask =
    OsmMode::MOTOR_VEHICLE | OsmMode::BICYCLE | OsmMode::BUS | OsmMode::HGV | OsmMode::EMERGENCY;
const BARRIER_MOTOR_MASK: ModeMask =
    OsmMode::MOTOR_VEHICLE | OsmMode::BUS | OsmMode::HGV | OsmMode::EMERGENCY;

#[derive(Clone, Copy, PartialEq, Eq)]
enum BarrierPass {
    Free,
    Conditional,
    Blocked,
}

/// Partition of every OSM mode bit into the three passability classes. The three
/// masks are disjoint and together cover [`OsmMode::ALL`].
#[derive(Clone, Copy)]
struct BarrierClasses {
    free: ModeMask,
    conditional: ModeMask,
    blocked: ModeMask,
}

impl BarrierClasses {
    fn uniform(pass: BarrierPass) -> Self {
        let mut classes = Self {
            free: 0,
            conditional: 0,
            blocked: 0,
        };
        classes.set(OsmMode::ALL, pass);
        classes
    }

    fn set(&mut self, bits: ModeMask, pass: BarrierPass) {
        self.free &= !bits;
        self.conditional &= !bits;
        self.blocked &= !bits;
        match pass {
            BarrierPass::Free => self.free |= bits,
            BarrierPass::Conditional => self.conditional |= bits,
            BarrierPass::Blocked => self.blocked |= bits,
        }
    }
}

/// Default per-mode passability for a bare `barrier=*` value, before node access
/// tags are applied. Returns `None` for unrecognised values (which impose no
/// constraint and are counted as unknown in diagnostics).
fn barrier_base_classes(value: &str) -> Option<BarrierClasses> {
    let mut classes = BarrierClasses::uniform(BarrierPass::Free);
    match value {
        // Rising posts / blocks: foot and bicycle slip past, motor traffic stops.
        "bollard" | "block" | "motorcycle_barrier" => {
            classes.set(OsmMode::FOOT | OsmMode::BICYCLE, BarrierPass::Free);
            classes.set(BARRIER_MOTOR_MASK, BarrierPass::Blocked);
        }
        // Cycle barrier: walk through freely, cyclists dismount (penalized).
        "cycle_barrier" => {
            classes.set(OsmMode::FOOT, BarrierPass::Free);
            classes.set(OsmMode::BICYCLE, BarrierPass::Conditional);
            classes.set(BARRIER_MOTOR_MASK, BarrierPass::Blocked);
        }
        // Gates are usually openable: everyone conditional until access says so.
        "gate" | "lift_gate" | "swing_gate" | "sliding_gate" => {
            classes.set(OsmMode::ALL, BarrierPass::Conditional);
        }
        // Pedestrian-only squeeze points.
        "stile" | "kissing_gate" | "turnstile" => {
            classes.set(OsmMode::FOOT, BarrierPass::Conditional);
            classes.set(BARRIER_VEHICLE_MASK, BarrierPass::Blocked);
        }
        // Cattle grid: passable by all; nothing to block or penalize.
        "cattle_grid" => {
            classes = BarrierClasses::uniform(BarrierPass::Free);
        }
        // Car traps: designed to stop private cars while buses/emergency pass.
        "bus_trap" | "sump_buster" => {
            classes.set(
                OsmMode::FOOT | OsmMode::BICYCLE | OsmMode::BUS | OsmMode::EMERGENCY,
                BarrierPass::Free,
            );
            classes.set(OsmMode::MOTOR_VEHICLE | OsmMode::HGV, BarrierPass::Blocked);
        }
        // Obstructions: passable on foot with effort, impassable to any vehicle.
        "chain" | "debris" | "log" => {
            classes.set(OsmMode::FOOT, BarrierPass::Conditional);
            classes.set(BARRIER_VEHICLE_MASK, BarrierPass::Blocked);
        }
        // Recognised but routing-irrelevant node barriers.
        "kerb" | "entrance" | "height_restrictor" => {
            classes = BarrierClasses::uniform(BarrierPass::Free);
        }
        _ => return None,
    }
    Some(classes)
}

/// Whether a barrier node changes routing: it affects some mode, or its
/// value is unknown (the SAR `barriers` relaxation costs those). The graph
/// builder makes such nodes graph nodes.
pub fn barrier_node_applies(tags: &NodeBarrierTags) -> bool {
    derive_node_barrier(tags).is_none_or(|barrier| barrier.has_effect())
}

/// Resolve a barrier node's per-mode passability from its tags. `None` marks an
/// unrecognised `barrier=*` value that imposes no constraint.
pub fn derive_node_barrier(tags: &NodeBarrierTags) -> Option<NodeBarrier> {
    let mut classes = barrier_base_classes(tags.barrier.trim())?;

    // Explicit node access tags override the value defaults, most-specific last
    // so that e.g. `foot=yes` survives a blanket `access=private`.
    apply_barrier_access(&mut classes, tags.access.as_deref(), OsmMode::ALL);
    apply_barrier_access(&mut classes, tags.vehicle.as_deref(), BARRIER_VEHICLE_MASK);
    apply_barrier_access(
        &mut classes,
        tags.motor_vehicle.as_deref(),
        BARRIER_MOTOR_MASK,
    );
    apply_barrier_access(&mut classes, tags.bicycle.as_deref(), OsmMode::BICYCLE);
    apply_barrier_access(&mut classes, tags.foot.as_deref(), OsmMode::FOOT);

    // A locked barrier hard-blocks every mode that is not explicitly permitted,
    // i.e. everything still only conditionally passable after the grants above.
    if barrier_is_locked(tags.locked.as_deref()) {
        let still_conditional = classes.conditional;
        classes.set(still_conditional, BarrierPass::Blocked);
    }

    Some(NodeBarrier {
        passable_mask: classes.free | classes.conditional,
        conditional_mask: classes.conditional,
    })
}

fn apply_barrier_access(classes: &mut BarrierClasses, value: Option<&str>, scope: ModeMask) {
    let Some(value) = value.map(str::trim).filter(|value| !value.is_empty()) else {
        return;
    };
    if osm_access_allows(value) {
        classes.set(scope, BarrierPass::Free);
    } else if osm_access_limits_to_destination(value) {
        // As on a private road: motor vehicles going somewhere behind it get
        // through (as through any gate; the roads behind are destination-only),
        // everyone else is stopped, and a barrier vehicles cannot pass (a
        // bollard) stays shut to them.
        let open = scope & MOTOR_MODES & (classes.free | classes.conditional);
        classes.set(scope & !open, BarrierPass::Blocked);
        classes.set(open, BarrierPass::Conditional);
    } else if osm_access_denies(value) {
        classes.set(scope, BarrierPass::Blocked);
    }
}

fn barrier_is_locked(value: Option<&str>) -> bool {
    matches!(value.map(str::trim), Some("yes" | "true" | "1"))
}

/// Sentinel values that mean "explicitly no limit" on a physical-limit tag.
/// These are *not* missing data — `maxheight=none` asserts there is no height
/// restriction — but both resolve to "does not constrain", so both map to
/// `None` here and a profile treats an absent limit as unconstrained.
const NO_PHYSICAL_LIMIT: &[&str] = &["none", "no", "default", "unsigned", "unknown"];

/// Normalize an OSM numeric value written with a comma decimal separator
/// (`"3,5"`), which is common in continental European tagging.
fn normalize_decimal_comma(value: &str) -> String {
    value.replace(',', ".")
}

/// Parse an OSM length value (`maxheight`, `maxwidth`, `width`) into
/// centimetres.
///
/// Bare numbers are metres, per the OSM convention. Recognized suffixes are
/// `m`, `cm`, `mm`, `km`, `ft`/`feet`, `in`/`inch`/`inches`, plus the
/// imperial shorthand `12'6"` and `12'`. Comma decimals are accepted.
/// Sentinels in `NO_PHYSICAL_LIMIT` and anything unparseable yield `None`,
/// so an unreadable tag is indistinguishable from an absent one — a limit we
/// cannot read must never silently become a limit we enforce.
///
/// Centimetres (not metres) because the value is a hard gate compared against
/// a vehicle dimension: an integer avoids float-equality edge cases at the
/// threshold, and `u16` spans 0..655 m, far beyond any real clearance.
pub fn parse_length_cm(value: Option<&str>) -> Option<u16> {
    let raw = value?.trim();
    if raw.is_empty() {
        return None;
    }
    let lower = normalize_decimal_comma(&raw.to_ascii_lowercase());
    if NO_PHYSICAL_LIMIT.contains(&lower.as_str()) {
        return None;
    }

    // Imperial shorthand: 12'6" / 12' / 12'6
    if let Some((feet, rest)) = lower.split_once('\'') {
        let feet: f64 = feet.trim().parse().ok()?;
        let inches: f64 = {
            let rest = rest.trim().trim_end_matches('"').trim();
            if rest.is_empty() {
                0.0
            } else {
                rest.parse().ok()?
            }
        };
        return cm_from_f64((feet * 12.0 + inches) * 2.54);
    }

    let (number, unit) = split_number_and_unit(&lower)?;
    let metres = match unit {
        "" | "m" | "meter" | "metre" | "meters" | "metres" => number,
        "cm" => number / 100.0,
        "mm" => number / 1000.0,
        "km" => number * 1000.0,
        "ft" | "feet" | "foot" => number * 0.3048,
        "in" | "\"" | "inch" | "inches" => number * 0.0254,
        _ => return None,
    };
    cm_from_f64(metres * 100.0)
}

fn cm_from_f64(centimetres: f64) -> Option<u16> {
    if !centimetres.is_finite() || centimetres <= 0.0 {
        return None;
    }
    let rounded = centimetres.round();
    (rounded <= f64::from(u16::MAX)).then_some(rounded as u16)
}

/// Parse an OSM mass value (`maxweight`, `maxaxleload`) into kilograms.
///
/// Bare numbers are metric tonnes, per the OSM convention. Recognized
/// suffixes are `t`/`tonne(s)`/`ton(s)`, `kg`, `lb`/`lbs`, and `st` (US short
/// ton). Sentinels and unparseable values yield `None`, on the same
/// fail-open-to-unconstrained rule as [`parse_length_cm`].
pub fn parse_mass_kg(value: Option<&str>) -> Option<u32> {
    let raw = value?.trim();
    if raw.is_empty() {
        return None;
    }
    let lower = normalize_decimal_comma(&raw.to_ascii_lowercase());
    if NO_PHYSICAL_LIMIT.contains(&lower.as_str()) {
        return None;
    }

    let (number, unit) = split_number_and_unit(&lower)?;
    let kilograms = match unit {
        "" | "t" | "ton" | "tons" | "tonne" | "tonnes" => number * 1000.0,
        "kg" | "kgs" => number,
        "lb" | "lbs" | "pound" | "pounds" => number * 0.453_592_37,
        "st" | "short_ton" | "shortton" => number * 907.184_74,
        _ => return None,
    };
    if !kilograms.is_finite() || kilograms <= 0.0 {
        return None;
    }
    let rounded = kilograms.round();
    (rounded <= f64::from(u32::MAX)).then_some(rounded as u32)
}

/// Parse an OSM `incline` value into signed percent, positive uphill in the
/// way's digitisation direction.
///
/// Accepts `"10%"`, `"-5%"`, a bare number (percent), and degrees (`"6°"`,
/// `"6 deg"`) converted via `tan`. The qualitative values `up`/`down`/`steep`
/// carry a direction but no magnitude, so they yield `None` rather than an
/// invented gradient. Clamped to `i8` — beyond ±127% is not a road.
pub fn parse_incline_percent(value: Option<&str>) -> Option<i8> {
    let raw = value?.trim();
    if raw.is_empty() {
        return None;
    }
    let lower = normalize_decimal_comma(&raw.to_ascii_lowercase());
    match lower.as_str() {
        "up" | "down" | "steep" | "yes" | "no" => return None,
        _ => {}
    }

    let percent = if let Some(rest) = lower
        .strip_suffix('°')
        .or_else(|| lower.strip_suffix("deg"))
        .or_else(|| lower.strip_suffix("degrees"))
    {
        let degrees: f64 = rest.trim().parse().ok()?;
        math::tan(degrees.to_radians()) * 100.0
    } else {
        let (number, unit) = split_number_and_unit(&lower)?;
        match unit {
            "" | "%" => number,
            _ => return None,
        }
    };
    if !percent.is_finite() {
        return None;
    }
    Some(percent.round().clamp(-127.0, 127.0) as i8)
}

/// Split a lowercased value into its leading number and trailing unit. The
/// unit is returned with whitespace stripped and may be empty.
fn split_number_and_unit(value: &str) -> Option<(f64, &str)> {
    let end = value
        .char_indices()
        .find(|(_, character)| {
            !character.is_ascii_digit()
                && *character != '.'
                && *character != '-'
                && *character != '+'
        })
        .map(|(index, _)| index)
        .unwrap_or(value.len());
    let number: f64 = value[..end].parse().ok()?;
    Some((number, value[end..].trim()))
}

pub fn parse_maxspeed_kph(value: Option<&str>) -> Option<f32> {
    parse_maxspeed_kph_with_country(value, None)
}

/// Speed cap (km/h) applied to unlimited implicit values such as `none` on
/// German autobahns. Kept in step with the free-flow speed clamp used when
/// baking edge speeds so an "unlimited" way never resolves faster than the
/// costing model can travel.
pub(crate) const IMPLICIT_MAXSPEED_UNLIMITED_KPH: f32 = 130.0;
const IMPLICIT_MAXSPEED_WALK_KPH: f32 = 7.0;

/// Region-idiomatic implicit maxspeed defaults keyed by lowercase
/// `(country, context)` from the OSM default-maxspeed table. Values are the
/// conservative bound where a country splits limits by sub-region.
const IMPLICIT_MAXSPEED_TABLE: &[(&str, &str, f32)] = &[
    ("pl", "living_street", 20.0),
    ("pl", "urban", 50.0),
    ("pl", "rural", 90.0),
    ("pl", "trunk", 120.0),
    ("pl", "motorway", 140.0),
    ("de", "living_street", 7.0),
    ("de", "urban", 50.0),
    ("de", "rural", 100.0),
    ("de", "trunk", 100.0),
    ("de", "motorway", IMPLICIT_MAXSPEED_UNLIMITED_KPH),
    ("fr", "living_street", 20.0),
    ("fr", "urban", 50.0),
    ("fr", "rural", 80.0),
    ("fr", "trunk", 110.0),
    ("fr", "motorway", 130.0),
    ("es", "living_street", 20.0),
    ("es", "urban", 50.0),
    ("es", "rural", 90.0),
    ("es", "trunk", 90.0),
    ("es", "motorway", 120.0),
    ("it", "living_street", 20.0),
    ("it", "urban", 50.0),
    ("it", "rural", 90.0),
    ("it", "trunk", 110.0),
    ("it", "motorway", 130.0),
    ("gb", "living_street", 20.0),
    ("gb", "urban", 48.0),
    ("gb", "rural", 96.0),
    ("gb", "trunk", 112.0),
    ("gb", "motorway", 112.0),
    ("cz", "living_street", 20.0),
    ("cz", "urban", 50.0),
    ("cz", "rural", 90.0),
    ("cz", "trunk", 110.0),
    ("cz", "motorway", 130.0),
    ("sk", "living_street", 20.0),
    ("sk", "urban", 50.0),
    ("sk", "rural", 90.0),
    ("sk", "trunk", 90.0),
    ("sk", "motorway", 130.0),
    ("at", "living_street", 20.0),
    ("at", "urban", 50.0),
    ("at", "rural", 100.0),
    ("at", "trunk", 100.0),
    ("at", "motorway", 130.0),
    ("ch", "living_street", 20.0),
    ("ch", "urban", 50.0),
    ("ch", "rural", 80.0),
    ("ch", "trunk", 100.0),
    ("ch", "motorway", 120.0),
    ("nl", "living_street", 15.0),
    ("nl", "urban", 50.0),
    ("nl", "rural", 80.0),
    ("nl", "trunk", 100.0),
    ("nl", "motorway", 100.0),
    ("be", "living_street", 20.0),
    ("be", "urban", 50.0),
    ("be", "rural", 70.0),
    ("be", "trunk", 120.0),
    ("be", "motorway", 120.0),
    ("dk", "living_street", 15.0),
    ("dk", "urban", 50.0),
    ("dk", "rural", 80.0),
    ("dk", "trunk", 80.0),
    ("dk", "motorway", 130.0),
    ("se", "living_street", 20.0),
    ("se", "urban", 50.0),
    ("se", "rural", 70.0),
    ("se", "trunk", 90.0),
    ("se", "motorway", 110.0),
    ("no", "living_street", 20.0),
    ("no", "urban", 50.0),
    ("no", "rural", 80.0),
    ("no", "trunk", 80.0),
    ("no", "motorway", 100.0),
    ("fi", "living_street", 20.0),
    ("fi", "urban", 50.0),
    ("fi", "rural", 80.0),
    ("fi", "trunk", 100.0),
    ("fi", "motorway", 120.0),
    ("pt", "living_street", 20.0),
    ("pt", "urban", 50.0),
    ("pt", "rural", 90.0),
    ("pt", "trunk", 100.0),
    ("pt", "motorway", 120.0),
    ("ro", "living_street", 20.0),
    ("ro", "urban", 50.0),
    ("ro", "rural", 90.0),
    ("ro", "trunk", 100.0),
    ("ro", "motorway", 130.0),
    ("hu", "living_street", 20.0),
    ("hu", "urban", 50.0),
    ("hu", "rural", 90.0),
    ("hu", "trunk", 110.0),
    ("hu", "motorway", 130.0),
    ("ua", "living_street", 20.0),
    ("ua", "urban", 50.0),
    ("ua", "rural", 90.0),
    ("ua", "trunk", 110.0),
    ("ua", "motorway", 130.0),
    ("lt", "living_street", 20.0),
    ("lt", "urban", 50.0),
    ("lt", "rural", 90.0),
    ("lt", "trunk", 120.0),
    ("lt", "motorway", 130.0),
    ("lv", "living_street", 20.0),
    ("lv", "urban", 50.0),
    ("lv", "rural", 90.0),
    ("lv", "trunk", 100.0),
    ("lv", "motorway", 110.0),
    ("ee", "living_street", 20.0),
    ("ee", "urban", 50.0),
    ("ee", "rural", 90.0),
    ("ee", "trunk", 100.0),
    ("ee", "motorway", 110.0),
    ("us", "living_street", 16.0),
    ("us", "urban", 40.0),
    ("us", "rural", 89.0),
    ("us", "trunk", 89.0),
    ("us", "motorway", 105.0),
];

/// Parse an OSM `maxspeed` value into km/h. Bare numbers and `mph` values keep
/// their historical handling; region-idiomatic implicit values (`PL:urban`,
/// `DE:rural`, `none`, `walk`, ...) resolve through `IMPLICIT_MAXSPEED_TABLE`,
/// using `country` (an ISO 3166-1 alpha-2 code from the build request) to
/// resolve bare `urban`/`rural`-style contexts. `signals`/`variable` and
/// unknown contexts return `None` so the road-type default applies.
pub fn parse_maxspeed_kph_with_country(value: Option<&str>, country: Option<&str>) -> Option<f32> {
    let raw = value?.trim();
    if raw.is_empty() {
        return None;
    }
    let lower = raw.to_ascii_lowercase();
    if let Some(number) = lower
        .strip_suffix(" mph")
        .or_else(|| lower.strip_suffix("mph"))
    {
        return number.trim().parse::<f32>().ok().map(|mph| mph * 1.609_34);
    }
    if let Some(number) = lower
        .split_whitespace()
        .next()
        .and_then(|token| token.parse::<f32>().ok())
    {
        return Some(number);
    }
    implicit_maxspeed_kph(&lower, country)
}

fn implicit_maxspeed_kph(value: &str, country: Option<&str>) -> Option<f32> {
    match value {
        "none" => return Some(IMPLICIT_MAXSPEED_UNLIMITED_KPH),
        "walk" => return Some(IMPLICIT_MAXSPEED_WALK_KPH),
        "signals" | "variable" => return None,
        _ => {}
    }

    let (country_code, context) = match value.split_once(':') {
        Some((code, rest)) => (Some(code.trim()), rest.trim()),
        None => (None, value.trim()),
    };
    let country_code = country_code
        .filter(|code| !code.is_empty())
        .or(country)?
        .trim();
    if country_code.is_empty() {
        return None;
    }

    if let Some(rest) = context.strip_prefix("zone")
        && let Ok(kph) = rest.trim_start_matches(':').trim().parse::<f32>()
    {
        return Some(kph);
    }

    let country_code = country_code.to_ascii_lowercase();
    IMPLICIT_MAXSPEED_TABLE
        .iter()
        .find(|(code, ctx, _)| *code == country_code && *ctx == context)
        .map(|(_, _, kph)| *kph)
}

pub fn parse_lane_count(value: Option<&str>) -> Option<u8> {
    value?
        .split(';')
        .next()
        .map(str::trim)
        .filter(|value| !value.is_empty())
        .and_then(|value| value.parse::<u8>().ok())
}

pub fn parse_layer(value: Option<&str>) -> Option<i16> {
    value?
        .trim()
        .parse::<i16>()
        .ok()
        .map(|layer| layer.clamp(-10, 10))
}

pub(crate) fn base_hierarchy_tier(road_type: RoadType) -> RoadHierarchyTier {
    match road_type {
        RoadType::Motorway => RoadHierarchyTier::Highway,
        RoadType::Primary | RoadType::Secondary => RoadHierarchyTier::Arterial,
        RoadType::Tertiary | RoadType::Residential => RoadHierarchyTier::Collector,
        RoadType::Track | RoadType::Path | RoadType::Other => RoadHierarchyTier::Local,
    }
}

pub(crate) fn hierarchy_tier_label(tier: RoadHierarchyTier) -> &'static str {
    match tier {
        RoadHierarchyTier::Local => "local",
        RoadHierarchyTier::Collector => "collector",
        RoadHierarchyTier::Arterial => "arterial",
        RoadHierarchyTier::Highway => "highway",
    }
}

pub(crate) fn promoted_hierarchy_tier(
    base: RoadHierarchyTier,
    source_degree: usize,
    target_degree: usize,
) -> RoadHierarchyTier {
    match base {
        RoadHierarchyTier::Local if source_degree >= 3 && target_degree >= 3 => {
            RoadHierarchyTier::Collector
        }
        RoadHierarchyTier::Collector if source_degree >= 4 && target_degree >= 4 => {
            RoadHierarchyTier::Arterial
        }
        _ => base,
    }
}

/// Copies a way's facts onto its shared edge info.
pub(crate) fn apply_way_facts(
    info: &mut SharedEdgeInfo,
    facts: &RoadWayFacts,
    country_code: Option<&str>,
    interner: &mut TagInterner,
) {
    info.road_type = facts.road_type;
    info.directional_access = facts.directional_access;
    info.highway = interner.intern_opt(facts.highway.as_deref());
    info.name = interner.intern_opt(facts.name.as_deref());
    info.ref_name = interner.intern_opt(facts.ref_name.as_deref());
    info.access = interner.intern_opt(facts.access.as_deref());
    info.service = interner.intern_opt(facts.service.as_deref());
    info.surface = interner.intern_opt(facts.surface.as_deref());
    info.tracktype = interner.intern_opt(facts.tracktype.as_deref());
    info.smoothness = interner.intern_opt(facts.smoothness.as_deref());
    info.maxspeed_kph = parse_maxspeed_kph_with_country(facts.maxspeed.as_deref(), country_code);
    info.maxspeed_forward_kph =
        parse_maxspeed_kph_with_country(facts.maxspeed_forward.as_deref(), country_code);
    info.maxspeed_backward_kph =
        parse_maxspeed_kph_with_country(facts.maxspeed_backward.as_deref(), country_code);
    info.hgv_maxspeed_kph =
        parse_maxspeed_kph_with_country(facts.maxspeed_hgv.as_deref(), country_code);
    info.max_height_cm = parse_length_cm(facts.maxheight.as_deref());
    info.max_width_cm = parse_length_cm(facts.maxwidth.as_deref());
    info.physical_width_cm = parse_length_cm(facts.width.as_deref());
    info.max_weight_kg = parse_mass_kg(facts.maxweight.as_deref());
    info.max_axle_load_kg = parse_mass_kg(facts.maxaxleload.as_deref());
    info.incline_percent = parse_incline_percent(facts.incline.as_deref());
    info.is_4wd_only = facts.is_4wd_only;
    info.is_motorroad = oneway_value_is_yes(facts.motorroad.as_deref());
    info.is_lit = facts.is_lit;
    info.sac_scale = interner.intern_opt(facts.sac_scale.as_deref());
    info.mtb_scale = interner.intern_opt(facts.mtb_scale.as_deref());
    info.trail_visibility = interner.intern_opt(facts.trail_visibility.as_deref());
    info.sidewalk = interner.intern_opt(facts.sidewalk.as_deref());
    info.crossing = interner.intern_opt(facts.crossing.as_deref());
    info.lane_count = parse_lane_count(facts.lanes.as_deref());
    info.lanes = interner.intern_opt(facts.lanes.as_deref());
    info.turn_lanes = interner.intern_opt(facts.turn_lanes.as_deref());
    info.turn_lanes_forward = interner.intern_opt(facts.turn_lanes_forward.as_deref());
    info.turn_lanes_backward = interner.intern_opt(facts.turn_lanes_backward.as_deref());
    info.lane_connectivity_available = facts.lane_connectivity_available
        || info.turn_lanes.is_some()
        || info.turn_lanes_forward.is_some()
        || info.turn_lanes_backward.is_some();
    info.speed_source = Some(speed_source_for_values(info.maxspeed_kph));
    let speed = infer_speed_kph_for_facts(
        facts.road_type,
        info.maxspeed_kph,
        facts.surface.as_deref(),
        facts.tracktype.as_deref(),
        facts.smoothness.as_deref(),
        facts.has_ford,
    );
    info.free_flow_speed_kph = Some(speed as f32);
    info.constrained_speed_kph = Some(constrained_speed_kph(speed));
    info.admin_country_id = interner.intern_opt(facts.admin_country_id.as_deref());
    info.admin_region_id = interner.intern_opt(facts.admin_region_id.as_deref());
    info.timezone_id = interner.intern_opt(facts.timezone_id.as_deref());
    info.conditional_restrictions = facts.conditional_restrictions.clone();
    info.layer = parse_layer(facts.layer.as_deref());
    info.level = interner.intern_opt(facts.level.as_deref());
    info.destination = interner.intern_opt(facts.destination.as_deref());
    info.destination_ref = interner.intern_opt(facts.destination_ref.as_deref());
    info.destination_street = interner.intern_opt(facts.destination_street.as_deref());
    info.junction_ref = interner.intern_opt(facts.junction_ref.as_deref());
    info.is_toll = facts.is_toll;
    info.is_ferry = facts.is_ferry;
    info.has_gate = facts.has_gate;
    info.is_access_restricted = facts.is_access_restricted;
    info.is_link = facts.is_link;
    info.is_roundabout = facts.is_roundabout;
    info.is_bridge = facts.has_bridge;
    info.is_tunnel = facts.has_tunnel;
    info.has_ford = facts.has_ford;
    info.is_service_like =
        facts.service.is_some() || matches!(facts.highway.as_deref(), Some("service"));
    info.is_destination_only = facts.is_destination_only;
    info.hierarchy_tier = Some(base_hierarchy_tier(facts.road_type));
    let base_mask = derive_osm_mode_mask(
        facts.highway.as_deref(),
        facts.access.as_deref(),
        facts.vehicle.as_deref(),
        facts.motor_vehicle.as_deref(),
        facts.motorcar.as_deref(),
        facts.bicycle.as_deref(),
        facts.foot.as_deref(),
        facts.motorroad.as_deref(),
    );
    let (forward, reverse) = split_directional_masks(
        base_mask,
        facts.directional_access,
        facts.oneway_foot.as_deref(),
        facts.oneway_bicycle.as_deref(),
    );
    info.osm_forward_mode_mask = forward;
    info.osm_reverse_mode_mask = reverse;
    info.legal_reason = LegalReason::from_tags(
        facts.access.as_deref(),
        facts.vehicle.as_deref(),
        facts.motor_vehicle.as_deref(),
        facts.motorcar.as_deref(),
        &facts.conditional_restrictions,
        vehicle_legally_denied(facts.is_ferry, facts.highway.as_deref(), forward | reverse),
    )
    .0;
}

/// Vehicles may not legally use a way that is physically drivable (a
/// drivable class, not a ferry) in either direction: the SAR `legal_access`
/// relaxation would be needed to drive it.
pub fn vehicle_legally_denied(is_ferry: bool, highway: Option<&str>, osm_mask: ModeMask) -> bool {
    !is_ferry
        && vehicle_drivable_class(highway)
        && mobility_mode_mask_from_osm(osm_mask) & MobilityMode::WHEELED == 0
}

/// How a barrier node affects vehicles, for the SAR `barriers` relaxation:
/// `None` when it does not (a kerb, a gate open to motor vehicles).
pub fn sar_barrier_class(tags: &NodeBarrierTags) -> Option<SarBarrierClass> {
    let value = tags.barrier.trim();
    let Some(barrier) = derive_node_barrier(tags) else {
        return Some(SarBarrierClass::Unknown);
    };
    let motor = OsmMode::MOTOR_VEHICLE;
    if barrier.passable_mask & motor == 0 {
        return Some(match value {
            "gate" | "lift_gate" | "swing_gate" | "sliding_gate" | "chain" => {
                SarBarrierClass::LockedGate
            }
            "stile" | "kissing_gate" | "turnstile" => SarBarrierClass::FootOnly,
            _ => SarBarrierClass::MotorBlocking,
        });
    }
    (barrier.conditional_mask & motor != 0).then_some(SarBarrierClass::Gate)
}

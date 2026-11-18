//! Search-and-rescue (SAR) data in the routing package, shared by the road graph builder, the compiler that writes it, the
//! engine that reads it and the reference oracle:
//!
//! - the *physical graph*: drivable ways the law closes to vehicles are kept,
//!   with no legal access, and two [`Section::EdgeProfileAccess`] bits
//!   beyond the profile slots say what a vehicle could physically do
//!   ([`ACCESS_VEHICLE_PHYSICAL`], [`ACCESS_WRONG_WAY`]);
//! - a [`LegalReason`] byte per shared info: the tag that decides vehicle
//!   access, its value class, whether a conditional access tag applies, and
//!   whether vehicles are denied (R1 `legal_access` needed);
//! - [`SarBarrierClass`] per barrier node (R4 `barriers`);
//! - a [`SarPathClass`] and a [`SarWidthClass`] per shared info, in the
//!   `SharedInfoCostClasses` word (R5 `vehicle_on_paths`): footways, paths
//!   and cycleways are physically drivable, with their width when tagged;
//! - [`SarPackageInfo`] in the metadata: the least factors requests may set
//!   for R1, R2 and R5, which the engine's bound metrics of SAR columns use.
//!
//! [`Section::EdgeProfileAccess`]: crate::format::Section::EdgeProfileAccess

use serde::{Deserialize, Serialize};

use crate::{DirectionalAccess, MobilityMode, ModeMask, SharedEdgeInfo, TraversalDirection};

/// [`crate::format::Section::EdgeProfileAccess`] bit 6: a wheeled or tracked
/// vehicle can physically drive the edge in its direction (legal or not).
pub const ACCESS_VEHICLE_PHYSICAL: u8 = 1 << 6;
/// Bit 7: the edge's shared info is a vehicle one-way and the edge runs
/// against it (set only with [`ACCESS_VEHICLE_PHYSICAL`], on edges no vehicle
/// may legally use).
pub const ACCESS_WRONG_WAY: u8 = 1 << 7;
/// The profile slot bits (0–5) of an `EdgeProfileAccess` byte.
pub const ACCESS_PROFILE_BITS: u8 = 0b0011_1111;

/// The least R1 factor a request may set (the default).
pub const SAR_LEGAL_ACCESS_MIN_FACTOR: f64 = 1.2;
/// The least R2 factor a request may set (the default).
pub const SAR_WRONG_WAY_MIN_FACTOR: f64 = 2.0;
/// The least R5 factor a request may set (the default: ×3 over the wheeled
/// path cost).
pub const SAR_VEHICLE_ON_PATHS_MIN_FACTOR: f64 = 3.0;

/// Highway classes a car can physically drive (their default modes include
/// motor vehicles).
pub fn vehicle_drivable_class(highway: Option<&str>) -> bool {
    matches!(
        highway.map(str::trim),
        Some(
            "motorway"
                | "motorway_link"
                | "trunk"
                | "trunk_link"
                | "primary"
                | "primary_link"
                | "secondary"
                | "secondary_link"
                | "tertiary"
                | "tertiary_link"
                | "unclassified"
                | "residential"
                | "living_street"
                | "service"
                | "track"
                | "road"
        )
    )
}

/// Whether a shared info's way is physically drivable whatever the law says:
/// a drivable class that is not a ferry.
pub fn shared_info_drivable(info: &SharedEdgeInfo) -> bool {
    !info.is_ferry && vehicle_drivable_class(info.highway.as_deref())
}

/// Whether a way of this class is a path R5 can open: a drivable path class
/// ([`SarPathClass`]) that is not a ferry.
pub fn path_physically_drivable(is_ferry: bool, highway: Option<&str>) -> bool {
    !is_ferry && SarPathClass::from_highway(highway) != SarPathClass::None
}

/// The SAR bits (6–7) of a directed edge's `EdgeProfileAccess` byte:
/// physical when the edge is legal for vehicles, its way is drivable, or it
/// is a path (R5) in a direction its way's one-way allows; wrong way when a
/// drivable way's edge no vehicle may legally use runs against the way's
/// vehicle one-way. A path edge against its one-way is not physical: R5
/// opens paths in their own direction only.
pub fn edge_sar_bits(
    info: &SharedEdgeInfo,
    traversal: TraversalDirection,
    mobility_mode_mask: ModeMask,
) -> u8 {
    let legal = mobility_mode_mask & MobilityMode::WHEELED != 0;
    if legal {
        return ACCESS_VEHICLE_PHYSICAL;
    }
    let against = matches!(
        (info.directional_access, traversal),
        (DirectionalAccess::Forward, TraversalDirection::Backward)
            | (DirectionalAccess::Backward, TraversalDirection::Forward)
    );
    if path_physically_drivable(info.is_ferry, info.highway.as_deref()) {
        return if against { 0 } else { ACCESS_VEHICLE_PHYSICAL };
    }
    if !shared_info_drivable(info) {
        return 0;
    }
    ACCESS_VEHICLE_PHYSICAL | if against { ACCESS_WRONG_WAY } else { 0 }
}

/// The path class of a way R5 `vehicle_on_paths` can open (`highway=*`).
/// `steps` and `corridor` are never drivable and have none. Stored with the
/// walkway cost class in the `SharedInfoCostClasses` word (bits 9–11, see
/// [`cost_classes`]).
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
#[repr(u8)]
pub enum SarPathClass {
    /// Not a path R5 opens.
    #[default]
    None = 0,
    Pedestrian = 1,
    Cycleway = 2,
    Footway = 3,
    Path = 4,
    Bridleway = 5,
}

impl SarPathClass {
    /// The classes R5 can open, in request order.
    pub const ALL: [Self; 5] = [
        Self::Pedestrian,
        Self::Cycleway,
        Self::Footway,
        Self::Path,
        Self::Bridleway,
    ];

    pub fn from_highway(highway: Option<&str>) -> Self {
        match highway.map(str::trim) {
            Some("pedestrian") => Self::Pedestrian,
            Some("cycleway") => Self::Cycleway,
            Some("footway") => Self::Footway,
            Some("path") => Self::Path,
            Some("bridleway") => Self::Bridleway,
            _ => Self::None,
        }
    }

    /// The class of a request name (the `highway` value); `None` for
    /// anything else, `steps` and `corridor` included.
    pub fn from_name(name: &str) -> Option<Self> {
        Some(Self::from_highway(Some(name))).filter(|class| *class != Self::None)
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Pedestrian => "pedestrian",
            Self::Cycleway => "cycleway",
            Self::Footway => "footway",
            Self::Path => "path",
            Self::Bridleway => "bridleway",
        }
    }

    /// The bit of this class in a set of classes (a request's R5 classes).
    pub fn bit(self) -> u8 {
        1 << (self as u8)
    }
}

/// A way's physical width class, from `width`, else `est_width` (3 bits of
/// the `SharedInfoCostClasses` word, see [`cost_classes`]).
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
#[repr(u8)]
pub enum SarWidthClass {
    /// No width tagged, or none that parses.
    #[default]
    Unknown = 0,
    /// Under 2.5 m.
    Narrow = 1,
    /// 2.5 m to under 3 m.
    Medium = 2,
    /// 3 m or more.
    Wide = 3,
}

impl SarWidthClass {
    pub const ALL: [Self; 4] = [Self::Unknown, Self::Narrow, Self::Medium, Self::Wide];

    pub fn from_width_cm(width_cm: Option<u16>) -> Self {
        match width_cm {
            None => Self::Unknown,
            Some(cm) if cm < 250 => Self::Narrow,
            Some(cm) if cm < 300 => Self::Medium,
            Some(_) => Self::Wide,
        }
    }

    fn from_bits(bits: u32) -> Self {
        Self::ALL
            .get((bits & 0b111) as usize)
            .copied()
            .unwrap_or(Self::Unknown)
    }

    /// The least width of the class in metres (`None` when unknown).
    pub fn lower_bound_m(self) -> Option<f64> {
        match self {
            Self::Unknown => None,
            Self::Narrow => Some(0.0),
            Self::Medium => Some(2.5),
            Self::Wide => Some(3.0),
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Unknown => "unknown",
            Self::Narrow => "under 2.5 m",
            Self::Medium => "2.5-3 m",
            Self::Wide => "3 m or more",
        }
    }

    pub fn bit(self) -> u8 {
        1 << (self as u8)
    }
}

/// The SAR fields of the `SharedInfoCostClasses` word
/// (`wayfarer_costing::EdgeCostClasses::pack`):
///
/// - bits 9–11 hold the walkway cost class *and* the path class: `0` other,
///   `1` steps, `3` track, and for the walkway class `footpath` the path
///   class: `2` footway, `4` pedestrian, `5` cycleway, `6` path, `7`
///   bridleway;
/// - bits 27–29 hold the [`SarWidthClass`].
pub mod cost_classes {
    use super::{SarPathClass, SarWidthClass};

    pub const WALKWAY_SHIFT: u32 = 9;
    pub const WIDTH_SHIFT: u32 = 27;
    /// The walkway code of `footpath` (and of a footway).
    pub const FOOTPATH_CODE: u32 = 2;

    /// The walkway code of a path class (`None` for the others).
    pub fn path_code(class: SarPathClass) -> Option<u32> {
        match class {
            SarPathClass::None => None,
            SarPathClass::Footway => Some(FOOTPATH_CODE),
            SarPathClass::Pedestrian => Some(4),
            SarPathClass::Cycleway => Some(5),
            SarPathClass::Path => Some(6),
            SarPathClass::Bridleway => Some(7),
        }
    }

    /// The path class of a walkway code: `None` unless the code is
    /// `footpath`'s or a path class's.
    pub fn path_class_of_code(code: u32) -> SarPathClass {
        match code & 0b111 {
            FOOTPATH_CODE => SarPathClass::Footway,
            4 => SarPathClass::Pedestrian,
            5 => SarPathClass::Cycleway,
            6 => SarPathClass::Path,
            7 => SarPathClass::Bridleway,
            _ => SarPathClass::None,
        }
    }

    /// Whether a walkway code is `footpath`'s (one of the path classes).
    pub fn is_footpath_code(code: u32) -> bool {
        path_class_of_code(code) != SarPathClass::None
    }

    pub fn path_class(word: u32) -> SarPathClass {
        path_class_of_code(word >> WALKWAY_SHIFT)
    }

    pub fn width_class(word: u32) -> SarWidthClass {
        SarWidthClass::from_bits(word >> WIDTH_SHIFT)
    }

    pub fn width_bits(width: SarWidthClass) -> u32 {
        (width as u32) << WIDTH_SHIFT
    }
}

/// What the SAR costing reads about one directed edge: its
/// `EdgeProfileAccess` SAR bits, its way's legal reason, path class and
/// width class.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SarEdgeFacts {
    pub access: u8,
    pub reason: LegalReason,
    pub path: SarPathClass,
    pub width: SarWidthClass,
}

impl SarEdgeFacts {
    /// The facts of an edge with `access` and `reason` whose shared info has
    /// the `SharedInfoCostClasses` word `classes`.
    pub fn from_cost_classes(access: u8, reason: LegalReason, classes: u32) -> Self {
        Self {
            access,
            reason,
            path: cost_classes::path_class(classes),
            width: cost_classes::width_class(classes),
        }
    }
}

/// The access key that decides a way's vehicle access, most specific last.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum LegalKey {
    Access = 0,
    Vehicle = 1,
    MotorVehicle = 2,
    Motorcar = 3,
}

impl LegalKey {
    pub const ALL: [Self; 4] = [
        Self::Access,
        Self::Vehicle,
        Self::MotorVehicle,
        Self::Motorcar,
    ];

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Access => "access",
            Self::Vehicle => "vehicle",
            Self::MotorVehicle => "motor_vehicle",
            Self::Motorcar => "motorcar",
        }
    }

    fn from_bits(bits: u8) -> Self {
        Self::ALL[usize::from(bits & 0b11)]
    }
}

/// The class of an access value (4 bits of the [`LegalReason`] byte).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum LegalValue {
    /// No access key decides: the class default holds.
    None = 0,
    Yes = 1,
    /// `designated` or `official`.
    Designated = 2,
    Destination = 3,
    Permissive = 4,
    Permit = 5,
    Delivery = 6,
    Private = 7,
    No = 8,
    Customers = 9,
    Military = 10,
    Agricultural = 11,
    Forestry = 12,
}

impl LegalValue {
    pub const ALL: [Self; 13] = [
        Self::None,
        Self::Yes,
        Self::Designated,
        Self::Destination,
        Self::Permissive,
        Self::Permit,
        Self::Delivery,
        Self::Private,
        Self::No,
        Self::Customers,
        Self::Military,
        Self::Agricultural,
        Self::Forestry,
    ];

    /// The class of an OSM access value the mode masks act on (allow and
    /// deny values); `None` for any other value, which decides nothing.
    pub fn from_osm(value: &str) -> Option<Self> {
        Some(match value.trim() {
            "yes" => Self::Yes,
            "designated" | "official" => Self::Designated,
            "destination" => Self::Destination,
            "permissive" => Self::Permissive,
            "permit" => Self::Permit,
            "delivery" => Self::Delivery,
            "private" => Self::Private,
            "no" => Self::No,
            "customers" => Self::Customers,
            "military" => Self::Military,
            "agricultural" => Self::Agricultural,
            "forestry" => Self::Forestry,
            _ => return None,
        })
    }

    /// The class's request and report name (`designated` for both
    /// `designated` and `official`).
    pub fn as_str(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Yes => "yes",
            Self::Designated => "designated",
            Self::Destination => "destination",
            Self::Permissive => "permissive",
            Self::Permit => "permit",
            Self::Delivery => "delivery",
            Self::Private => "private",
            Self::No => "no",
            Self::Customers => "customers",
            Self::Military => "military",
            Self::Agricultural => "agricultural",
            Self::Forestry => "forestry",
        }
    }

    /// A value that closes a way to its key's modes.
    pub fn denies(self) -> bool {
        matches!(
            self,
            Self::No | Self::Military | Self::Agricultural | Self::Forestry
        )
    }

    /// A value that opens a way to vehicles going somewhere on it and closes
    /// it to through traffic and every other mode (`private`, `customers`).
    pub fn limits_to_destination(self) -> bool {
        matches!(self, Self::Private | Self::Customers)
    }

    /// A value that grants a qualified right (reported on every route).
    pub fn qualifies(self) -> bool {
        matches!(
            self,
            Self::Destination
                | Self::Permissive
                | Self::Permit
                | Self::Delivery
                | Self::Private
                | Self::Customers
        )
    }

    fn from_bits(bits: u8) -> Self {
        Self::ALL
            .get(usize::from(bits & 0b1111))
            .copied()
            .unwrap_or(Self::None)
    }

    /// The bit of this class in a set of classes (a request's R1 values).
    pub fn bit(self) -> u16 {
        1 << (self as u8)
    }
}

/// One byte per shared info (`SharedInfoLegalReason`): bits 0–1 the
/// [`LegalKey`] that decides vehicle access, bits 2–5 its [`LegalValue`]
/// (`None`: no key decides), bit 6 a conditional access tag on one of the
/// vehicle keys, bit 7 vehicles legally denied on a physically drivable way
/// (the R1 relaxation is needed to drive it).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct LegalReason(pub u8);

impl LegalReason {
    const CONDITIONAL: u8 = 1 << 6;
    const VEHICLE_DENIED: u8 = 1 << 7;
    /// The highest valid byte's value bits.
    const MAX_VALUE: u8 = LegalValue::Forestry as u8;

    /// The reason for a way with these vehicle access tags: the most specific
    /// key with an allow or deny value decides (the order the mode masks
    /// apply them). `conditional` are the way's `key:conditional=value`
    /// tags; `vehicle_denied` whether vehicles may not legally use a way
    /// that is physically drivable.
    pub fn from_tags(
        access: Option<&str>,
        vehicle: Option<&str>,
        motor_vehicle: Option<&str>,
        motorcar: Option<&str>,
        conditional: &[String],
        vehicle_denied: bool,
    ) -> Self {
        let mut byte = 0u8;
        for (key, value) in
            LegalKey::ALL
                .into_iter()
                .zip([access, vehicle, motor_vehicle, motorcar])
        {
            if let Some(class) = value.and_then(LegalValue::from_osm) {
                byte = key as u8 | (class as u8) << 2;
            }
        }
        let conditional = conditional.iter().any(|tag| {
            tag.split_once('=')
                .and_then(|(key, _)| key.trim().strip_suffix(":conditional"))
                .is_some_and(|base| LegalKey::ALL.iter().any(|key| key.as_str() == base))
        });
        if conditional {
            byte |= Self::CONDITIONAL;
        }
        if vehicle_denied {
            byte |= Self::VEHICLE_DENIED;
        }
        Self(byte)
    }

    pub fn value(self) -> LegalValue {
        LegalValue::from_bits(self.0 >> 2)
    }

    /// The deciding key, when a key decides.
    pub fn key(self) -> Option<LegalKey> {
        (self.value() != LegalValue::None).then(|| LegalKey::from_bits(self.0))
    }

    pub fn conditional(self) -> bool {
        self.0 & Self::CONDITIONAL != 0
    }

    pub fn vehicle_denied(self) -> bool {
        self.0 & Self::VEHICLE_DENIED != 0
    }

    /// The deciding tag as `key=value` (`motor_vehicle=permissive`); the
    /// value is the class name, so `official` reads `designated`.
    pub fn tag(self) -> Option<String> {
        self.key()
            .map(|key| format!("{}={}", key.as_str(), self.value().as_str()))
    }

    /// Whether the byte is one the format defines.
    pub fn is_valid(self) -> bool {
        (self.0 >> 2) & 0b1111 <= Self::MAX_VALUE
            && (self.value() != LegalValue::None || self.0 & 0b11 == 0)
    }
}

/// How a barrier node affects a vehicle (`BarrierClass`, low 3 bits; 0 for
/// a barrier with no effect on vehicles). Normal routing applies a barrier's
/// [`crate::barrier::BarrierAccess`]; the SAR `barriers`
/// relaxation (R4) costs vehicles by this class instead.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
#[repr(u8)]
pub enum SarBarrierClass {
    /// An openable gate (`gate`, `lift_gate`, `swing_gate`, `sliding_gate`).
    Gate = 1,
    /// A gate closed to vehicles by its access tags or `locked=yes`, or a
    /// chain.
    LockedGate = 2,
    /// Blocks motor vehicles (bollards, blocks, cycle barriers, car traps,
    /// debris, logs).
    MotorBlocking = 3,
    /// A pedestrian squeeze point (`stile`, `kissing_gate`, `turnstile`).
    FootOnly = 4,
    /// A `barrier=*` value the compiler does not know.
    Unknown = 5,
}

impl SarBarrierClass {
    pub const ALL: [Self; 5] = [
        Self::Gate,
        Self::LockedGate,
        Self::MotorBlocking,
        Self::FootOnly,
        Self::Unknown,
    ];

    pub fn from_code(code: u8) -> Option<Self> {
        Self::ALL.into_iter().find(|class| *class as u8 == code)
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Gate => "gate",
            Self::LockedGate => "locked_gate",
            Self::MotorBlocking => "motor_blocking",
            Self::FootOnly => "foot_only",
            Self::Unknown => "unknown",
        }
    }
}

/// The package's SAR data (`RoutingPackageMetadata::sar`): present when the
/// package holds the physical graph, the legal reasons and the barrier
/// nodes. A vehicle profile's SAR columns are bounded by the potentials of
/// a *bound metric*: the profile's costs, extended to every physically
/// drivable edge R1 and R2 can open at its cost times these minimum factors
/// (and, for R5 columns, every path at R5's). A request's factors may not
/// be lower, so every SAR column stays at or above its metric and the
/// bounds hold at scale 1.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct SarPackageInfo {
    pub legal_access_min_factor: f64,
    pub wrong_way_min_factor: f64,
    /// R5 `vehicle_on_paths`: the least factor a request may set. Paths
    /// are in the bound metric of R5 columns only.
    pub vehicle_on_paths_min_factor: f64,
}

impl Default for SarPackageInfo {
    fn default() -> Self {
        Self {
            legal_access_min_factor: SAR_LEGAL_ACCESS_MIN_FACTOR,
            wrong_way_min_factor: SAR_WRONG_WAY_MIN_FACTOR,
            vehicle_on_paths_min_factor: SAR_VEHICLE_ON_PATHS_MIN_FACTOR,
        }
    }
}

impl SarPackageInfo {
    /// Whether every factor is finite and at least 1.
    pub fn is_valid(&self) -> bool {
        let valid = |factor: f64| factor.is_finite() && factor >= 1.0;
        valid(self.legal_access_min_factor)
            && valid(self.wrong_way_min_factor)
            && valid(self.vehicle_on_paths_min_factor)
    }
}

/// A request's R5, as the edge costing applies it: the path classes and
/// width classes it opens and its factor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SarPathRelaxation {
    /// [`SarPathClass::bit`]s.
    pub classes: u8,
    /// [`SarWidthClass::bit`]s: the known classes whose least width is at
    /// least the request's `min_width_m`, and `Unknown` when it allows
    /// unknown widths.
    pub widths: u8,
    pub factor: f64,
}

impl SarPathRelaxation {
    /// The width classes a request with `min_width_m` admits: a known class
    /// when its least width reaches the minimum (a class's widths are not
    /// known more finely), unknown widths when `allow_unknown`.
    pub fn width_bits(min_width_m: f64, allow_unknown: bool) -> u8 {
        SarWidthClass::ALL
            .into_iter()
            .filter(|class| match class.lower_bound_m() {
                None => allow_unknown,
                Some(lower) => lower >= min_width_m,
            })
            .fold(0, |bits, class| bits | class.bit())
    }

    pub fn admits(&self, path: SarPathClass, width: SarWidthClass) -> bool {
        path != SarPathClass::None
            && self.classes & path.bit() != 0
            && self.widths & width.bit() != 0
    }
}

/// A request's R1, R2 and R5, as the edge costing applies them: which legal
/// values R1 opens, which paths R5 opens, and the factor of each.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SarEdgeRelaxations {
    /// R1: the [`LegalValue::bit`]s it relaxes and its factor.
    pub legal_access: Option<(u16, f64)>,
    /// R2: its factor.
    pub wrong_way: Option<f64>,
    /// R5.
    pub vehicle_on_paths: Option<SarPathRelaxation>,
}

/// Which relaxations an edge vehicles may not legally use needs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SarEdgeNeeds {
    pub legal_access: bool,
    pub wrong_way: bool,
    pub vehicle_on_paths: bool,
}

impl SarEdgeRelaxations {
    /// What an edge with these facts needs, when it is physically drivable
    /// but closed to vehicles (`None` when it is not physically drivable, or
    /// needs nothing a relaxation gives).
    pub fn needs(facts: SarEdgeFacts) -> Option<SarEdgeNeeds> {
        if facts.access & ACCESS_VEHICLE_PHYSICAL == 0 {
            return None;
        }
        let needs = SarEdgeNeeds {
            legal_access: facts.reason.vehicle_denied(),
            wrong_way: facts.access & ACCESS_WRONG_WAY != 0,
            vehicle_on_paths: facts.path != SarPathClass::None,
        };
        (needs.legal_access || needs.wrong_way || needs.vehicle_on_paths).then_some(needs)
    }

    /// The factor on the edge's base Wheeled cost when the request's
    /// relaxations open it, `None` when they do not.
    pub fn factor(&self, facts: SarEdgeFacts) -> Option<f64> {
        let needs = Self::needs(facts)?;
        let mut factor = 1.0;
        if needs.legal_access {
            let (values, legal) = self.legal_access?;
            if values & facts.reason.value().bit() == 0 {
                return None;
            }
            factor *= legal;
        }
        if needs.wrong_way {
            factor *= self.wrong_way?;
        }
        if needs.vehicle_on_paths {
            let paths = self.vehicle_on_paths?;
            if !paths.admits(facts.path, facts.width) {
                return None;
            }
            factor *= paths.factor;
        }
        Some(factor)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_most_specific_key_with_a_known_value_decides() {
        let reason = LegalReason::from_tags(
            Some("private"),
            None,
            Some("permissive"),
            Some("bogus"),
            &[],
            false,
        );
        assert_eq!(reason.key(), Some(LegalKey::MotorVehicle));
        assert_eq!(reason.value(), LegalValue::Permissive);
        assert_eq!(reason.tag().as_deref(), Some("motor_vehicle=permissive"));
        assert!(!reason.conditional() && !reason.vehicle_denied());
        assert!(reason.is_valid());

        let none = LegalReason::from_tags(None, None, None, None, &[], false);
        assert_eq!(none.key(), None);
        assert_eq!(none.tag(), None);

        let conditional = LegalReason::from_tags(
            Some("forestry"),
            None,
            None,
            None,
            &["motor_vehicle:conditional=no @ (Mo-Fr)".to_string()],
            true,
        );
        assert!(conditional.conditional() && conditional.vehicle_denied());
        assert_eq!(conditional.tag().as_deref(), Some("access=forestry"));
        assert!(!LegalReason(0b0011_1100).is_valid());
    }

    #[test]
    fn relaxations_open_only_what_the_request_names() {
        let denied = LegalReason::from_tags(Some("no"), None, None, None, &[], true);
        let military = LegalReason::from_tags(Some("military"), None, None, None, &[], true);
        let legal = LegalReason::default();
        let facts = |access, reason| SarEdgeFacts {
            access,
            reason,
            ..SarEdgeFacts::default()
        };
        let r1 = SarEdgeRelaxations {
            legal_access: Some((LegalValue::No.bit(), 1.5)),
            wrong_way: None,
            vehicle_on_paths: None,
        };
        assert_eq!(r1.factor(facts(ACCESS_VEHICLE_PHYSICAL, denied)), Some(1.5));
        assert_eq!(r1.factor(facts(ACCESS_VEHICLE_PHYSICAL, military)), None);
        let wrong = ACCESS_VEHICLE_PHYSICAL | ACCESS_WRONG_WAY;
        assert_eq!(r1.factor(facts(wrong, legal)), None);
        assert_eq!(r1.factor(facts(0, denied)), None);
        let both = SarEdgeRelaxations {
            wrong_way: Some(2.0),
            ..r1
        };
        assert_eq!(both.factor(facts(wrong, legal)), Some(2.0));
        assert_eq!(both.factor(facts(wrong, denied)), Some(3.0));
    }

    #[test]
    fn vehicle_on_paths_opens_the_named_classes_at_admitted_widths() {
        let path = |class, width| SarEdgeFacts {
            access: ACCESS_VEHICLE_PHYSICAL,
            reason: LegalReason::default(),
            path: class,
            width,
        };
        let r5 = SarEdgeRelaxations {
            legal_access: None,
            wrong_way: None,
            vehicle_on_paths: Some(SarPathRelaxation {
                classes: SarPathClass::Footway.bit() | SarPathClass::Cycleway.bit(),
                widths: SarPathRelaxation::width_bits(2.5, false),
                factor: 3.0,
            }),
        };
        use SarPathClass as P;
        use SarWidthClass as W;
        assert_eq!(r5.factor(path(P::Footway, W::Medium)), Some(3.0));
        assert_eq!(r5.factor(path(P::Cycleway, W::Wide)), Some(3.0));
        assert_eq!(r5.factor(path(P::Footway, W::Narrow)), None);
        assert_eq!(r5.factor(path(P::Footway, W::Unknown)), None);
        assert_eq!(r5.factor(path(P::Path, W::Wide)), None);
        // Not physical (against the path's one-way): never.
        assert_eq!(
            r5.factor(SarEdgeFacts {
                access: 0,
                ..path(P::Footway, W::Wide)
            }),
            None
        );
        // R1 and R2 alone do not open paths.
        let r1 = SarEdgeRelaxations {
            legal_access: Some((u16::MAX, 1.2)),
            wrong_way: Some(2.0),
            vehicle_on_paths: None,
        };
        assert_eq!(r1.factor(path(P::Footway, W::Wide)), None);

        let bits = SarPathRelaxation::width_bits;
        assert_eq!(bits(2.5, false), W::Medium.bit() | W::Wide.bit());
        assert_eq!(bits(2.6, true), W::Wide.bit() | W::Unknown.bit());
        assert_eq!(
            bits(0.0, false),
            W::Narrow.bit() | W::Medium.bit() | W::Wide.bit()
        );
        assert_eq!(bits(3.5, false), 0);

        let info = SarPackageInfo::default();
        assert!(info.is_valid());
        assert!(
            !SarPackageInfo {
                vehicle_on_paths_min_factor: 0.5,
                ..info
            }
            .is_valid()
        );
    }

    #[test]
    fn width_and_path_classes_round_trip_through_the_cost_classes_word() {
        for width in SarWidthClass::ALL {
            assert_eq!(
                cost_classes::width_class(cost_classes::width_bits(width)),
                width
            );
        }
        for class in SarPathClass::ALL {
            let code = cost_classes::path_code(class).expect("a path class has a code");
            assert_eq!(
                cost_classes::path_class(code << cost_classes::WALKWAY_SHIFT),
                class
            );
            assert!(cost_classes::is_footpath_code(code));
            assert_eq!(SarPathClass::from_name(class.as_str()), Some(class));
        }
        assert_eq!(SarPathClass::from_name("steps"), None);
        assert_eq!(SarPathClass::from_name("corridor"), None);
        assert_eq!(SarWidthClass::from_width_cm(None), SarWidthClass::Unknown);
        assert_eq!(
            SarWidthClass::from_width_cm(Some(249)),
            SarWidthClass::Narrow
        );
        assert_eq!(
            SarWidthClass::from_width_cm(Some(250)),
            SarWidthClass::Medium
        );
        assert_eq!(SarWidthClass::from_width_cm(Some(300)), SarWidthClass::Wide);
    }
}

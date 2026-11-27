//! Search-and-rescue (SAR) routing: per-request relaxations of legal limits
//!.
//!
//! SAR is off by default. A request enables relaxations one by one, by name,
//! in [`RouteSarOptions`]; there is no switch that relaxes everything. A
//! relaxation only adds cost on top of the base profile (a vehicle profile:
//! Wheeled, Emergency or Tracked), never lowers it, so the
//! search bounds stay valid.
//!
//! Relaxations on the road network alone:
//!
//! - `turn_restrictions` (R3): a turn banned by a simple or complex
//!   restriction is allowed at `penalty_s` per violated restriction;
//! - `conditional` (R6): active conditional closures
//!   (`access:conditional=no @ …`) are ignored;
//! - `off_network` (R7): an endpoint may lie up to `max_leg_m` from the road
//!   network, and the straight-line leg to the road is reported.
//!
//! Relaxations on the package's SAR data (the physical graph, legal reasons,
//! barrier nodes, path and width classes, `wayfarer_package::sar`):
//!
//! - `legal_access` (R1): ways whose deciding vehicle access tag has one of
//!   the listed deny values, at `factor` times their cost. `private` and
//!   `customers` close no road to vehicles (they make it destination-only):
//!   listing either lets the request cross destination-only roads, which
//!   drives otherwise use only to start or end on;
//! - `wrong_way` (R2): edges against a vehicle one-way that exist (for
//!   walkers), at `factor` times their cost plus `entry_s` on entering a
//!   wrong-way stretch;
//! - `barriers` (R4): gates at `gate_s`, locked gates (or gates closed to
//!   vehicles by their access tags) at `locked_gate_s`; motor-blocking and foot-only barriers stay closed.
//!   Barriers stay out of normal routing;
//! - `vehicle_on_paths` (R5): the listed path classes (`pedestrian`,
//!   `cycleway`, `footway`, `path`, `bridleway`; never `steps` or
//!   `corridor`) in their own direction, at `factor` times their Wheeled
//!   cost, excluding known widths under `min_width_m`, and unknown widths
//!   unless `unknown_width` is `allow`. On Emergency and Tracked it fails
//!   with [`RouteError::SarUnavailable`].
//!
//! Every SAR use on a route is reported in [`RouteSarReport`], and every
//! route, SAR or not, reports qualified legal access (see
//! [`RouteQualifiedAccess`]).

use serde::{Deserialize, Serialize};

use wayfarer_package::sar::{
    ACCESS_WRONG_WAY, LegalValue, SarBarrierClass, SarEdgeFacts, SarEdgeRelaxations,
    SarPackageInfo, SarPathClass, SarPathRelaxation, SarWidthClass,
};

use crate::types::{GeoPoint, MobilityProfile, RouteError};

/// The default R3 surcharge: seconds per violated turn restriction.
pub const SAR_DEFAULT_TURN_RESTRICTION_PENALTY_S: f64 = 60.0;
/// The least R3 surcharge a request may set. Restrictions are applied in the
/// search, outside the bounds' metric, so any penalty of at least zero keeps
/// the bounds valid; a negative one would reward breaking a restriction.
pub const SAR_MIN_TURN_RESTRICTION_PENALTY_S: f64 = 0.0;
/// The default R7 reach: the longest straight-line leg between an endpoint
/// and the road network.
pub const SAR_DEFAULT_OFF_NETWORK_MAX_LEG_M: f64 = 2_000.0;
/// The longest R7 leg a request may ask for.
pub const SAR_MAX_OFF_NETWORK_LEG_M: f64 = 10_000.0;

/// One SAR relaxation, by its request name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum SarRelaxationKind {
    /// R1: vehicle use of ways closed by legal access tags.
    LegalAccess,
    /// R2: driving against a vehicle one-way.
    WrongWay,
    /// R3: turns banned by turn restrictions.
    TurnRestrictions,
    /// R4: passing gates and other barriers.
    Barriers,
    /// R5: vehicles on footways, paths and cycleways.
    VehicleOnPaths,
    /// R6: ignoring time-limited (conditional) closures.
    Conditional,
    /// R7: endpoints away from the road network.
    OffNetwork,
}

impl SarRelaxationKind {
    pub const ALL: [Self; 7] = [
        Self::LegalAccess,
        Self::WrongWay,
        Self::TurnRestrictions,
        Self::Barriers,
        Self::VehicleOnPaths,
        Self::Conditional,
        Self::OffNetwork,
    ];

    pub fn as_str(self) -> &'static str {
        match self {
            Self::LegalAccess => "legal_access",
            Self::WrongWay => "wrong_way",
            Self::TurnRestrictions => "turn_restrictions",
            Self::Barriers => "barriers",
            Self::VehicleOnPaths => "vehicle_on_paths",
            Self::Conditional => "conditional",
            Self::OffNetwork => "off_network",
        }
    }
}

impl std::fmt::Display for SarRelaxationKind {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(self.as_str())
    }
}

/// The SAR option of a request (`RouteOptions::sar`). An absent relaxation
/// is not relaxed. JSON names are snake_case with camelCase aliases.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", default)]
pub struct RouteSarOptions {
    pub relaxations: RouteSarRelaxations,
}

impl RouteSarOptions {
    /// The relaxations the request names, in [`SarRelaxationKind::ALL`]
    /// order. A `conditional` relaxation with `ignore: false` relaxes
    /// nothing and is not listed.
    pub fn enabled(&self) -> Vec<SarRelaxationKind> {
        let r = &self.relaxations;
        SarRelaxationKind::ALL
            .into_iter()
            .filter(|kind| match kind {
                SarRelaxationKind::LegalAccess => r.legal_access.is_some(),
                SarRelaxationKind::WrongWay => r.wrong_way.is_some(),
                SarRelaxationKind::TurnRestrictions => r.turn_restrictions.is_some(),
                SarRelaxationKind::Barriers => r.barriers.is_some(),
                SarRelaxationKind::VehicleOnPaths => r.vehicle_on_paths.is_some(),
                SarRelaxationKind::Conditional => r.conditional.as_ref().is_some_and(|c| c.ignore),
                SarRelaxationKind::OffNetwork => r.off_network.is_some(),
            })
            .collect()
    }

    /// The R3 surcharge per violated restriction, when R3 is enabled.
    pub fn turn_restriction_penalty_s(&self) -> Option<f64> {
        self.relaxations
            .turn_restrictions
            .as_ref()
            .map(|relaxation| relaxation.penalty_s)
    }

    /// Whether R6 ignores conditional closures.
    pub fn ignores_conditional(&self) -> bool {
        self.relaxations
            .conditional
            .as_ref()
            .is_some_and(|relaxation| relaxation.ignore)
    }

    /// The R7 reach of a costed `off_network` (X6), when it is enabled.
    pub fn costed_off_network_max_leg_m(&self) -> Option<f64> {
        self.relaxations
            .off_network
            .as_ref()
            .filter(|relaxation| relaxation.costed)
            .map(|relaxation| relaxation.max_leg_m)
    }

    /// The R7 reach, when R7 is enabled.
    pub fn off_network_max_leg_m(&self) -> Option<f64> {
        self.relaxations
            .off_network
            .as_ref()
            .map(|relaxation| relaxation.max_leg_m)
    }
}

/// The relaxations of a SAR request, each absent unless named.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", default)]
pub struct RouteSarRelaxations {
    #[serde(alias = "legal_access", skip_serializing_if = "Option::is_none")]
    pub legal_access: Option<SarLegalAccess>,
    #[serde(alias = "wrong_way", skip_serializing_if = "Option::is_none")]
    pub wrong_way: Option<SarWrongWay>,
    #[serde(alias = "turn_restrictions", skip_serializing_if = "Option::is_none")]
    pub turn_restrictions: Option<SarTurnRestrictions>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub barriers: Option<SarBarriers>,
    #[serde(alias = "vehicle_on_paths", skip_serializing_if = "Option::is_none")]
    pub vehicle_on_paths: Option<SarVehicleOnPaths>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub conditional: Option<SarConditional>,
    #[serde(alias = "off_network", skip_serializing_if = "Option::is_none")]
    pub off_network: Option<SarOffNetwork>,
}

/// R1. `military` is relaxed only when listed; `private` or `customers`
/// lift the destination-only confinement of the whole request (see
/// `lifts_destination_only`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", default)]
pub struct SarLegalAccess {
    pub values: Vec<String>,
    pub factor: f64,
}

impl Default for SarLegalAccess {
    fn default() -> Self {
        Self {
            values: ["private", "no", "customers", "forestry", "agricultural"]
                .map(String::from)
                .to_vec(),
            factor: 1.2,
        }
    }
}

/// R2.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", default)]
pub struct SarWrongWay {
    pub factor: f64,
    #[serde(alias = "entry_s")]
    pub entry_s: f64,
}

impl Default for SarWrongWay {
    fn default() -> Self {
        Self {
            factor: 2.0,
            entry_s: 30.0,
        }
    }
}

/// R3: each violated simple or complex restriction costs `penalty_s`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", default)]
pub struct SarTurnRestrictions {
    #[serde(alias = "penalty_s")]
    pub penalty_s: f64,
}

impl Default for SarTurnRestrictions {
    fn default() -> Self {
        Self {
            penalty_s: SAR_DEFAULT_TURN_RESTRICTION_PENALTY_S,
        }
    }
}

/// How R4 treats barriers that block motor vehicles (bollards, blocks).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum SarMotorBlocking {
    /// Impassable for vehicles (the default).
    #[default]
    Impassable,
    /// Passable on foot only: the route ends its vehicle part there.
    FootOnly,
}

/// R4.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", default)]
pub struct SarBarriers {
    #[serde(alias = "gate_s")]
    pub gate_s: f64,
    #[serde(alias = "locked_gate_s")]
    pub locked_gate_s: f64,
    #[serde(alias = "motor_blocking")]
    pub motor_blocking: SarMotorBlocking,
}

impl Default for SarBarriers {
    fn default() -> Self {
        Self {
            gate_s: 90.0,
            locked_gate_s: 300.0,
            motor_blocking: SarMotorBlocking::Impassable,
        }
    }
}

/// Whether R5 admits paths of unknown width.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum SarUnknownWidth {
    #[default]
    Deny,
    Allow,
}

/// R5.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", default)]
pub struct SarVehicleOnPaths {
    pub classes: Vec<String>,
    pub factor: f64,
    #[serde(alias = "min_width_m")]
    pub min_width_m: f64,
    #[serde(alias = "unknown_width")]
    pub unknown_width: SarUnknownWidth,
}

impl Default for SarVehicleOnPaths {
    fn default() -> Self {
        Self {
            classes: ["pedestrian", "cycleway", "footway", "path", "bridleway"]
                .map(String::from)
                .to_vec(),
            factor: 3.0,
            min_width_m: 2.5,
            unknown_width: SarUnknownWidth::Deny,
        }
    }
}

/// R6: `ignore` (default true when the relaxation is named).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", default)]
pub struct SarConditional {
    pub ignore: bool,
}

impl Default for SarConditional {
    fn default() -> Self {
        Self { ignore: true }
    }
}

/// R7.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", default)]
pub struct SarOffNetwork {
    #[serde(alias = "max_leg_m")]
    pub max_leg_m: f64,
    /// X6: cost the leg (`crate::access`, at the profile's off-road pace and
    /// a cost factor of 1). The join is then chosen by total time among the
    /// candidates within `max_leg_m`, and the route's cost, duration,
    /// distance, geometry and maneuvers include the leg; the report's
    /// `added_cost_s` is both the cost and the duration they add. Off by
    /// default: the leg is reported and left out of the totals.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub costed: bool,
}

impl Default for SarOffNetwork {
    fn default() -> Self {
        Self {
            max_leg_m: SAR_DEFAULT_OFF_NETWORK_MAX_LEG_M,
            costed: false,
        }
    }
}

/// Which SAR relaxations a package supports
/// (`RoutingPackage::sar_capability`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteSarCapability {
    /// Relaxations a request may use on this package.
    pub available: Vec<SarRelaxationKind>,
    /// Relaxations that need SAR package data this package lacks; requests
    /// for them fail with `SarUnavailable`.
    pub unavailable: Vec<SarRelaxationKind>,
    /// Why `unavailable` is not empty.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
    /// The least R1 and R2 factors a request may set (the bound metric of
    /// SAR columns), on a package with SAR data.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub legal_access_min_factor: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub wrong_way_min_factor: Option<f64>,
    /// The least R5 factor, on a package with path and width classes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vehicle_on_paths_min_factor: Option<f64>,
}

// The factors come from a package's metadata, which open checks to be
// finite, so equality is total.
impl Eq for RouteSarCapability {}

impl RouteSarCapability {
    /// The capability of a package with SAR data `sar` (its bound
    /// metric): every relaxation, at the metric's least factors.
    pub fn with_package_data(sar: SarPackageInfo) -> Self {
        Self {
            available: SarRelaxationKind::ALL.to_vec(),
            unavailable: Vec::new(),
            reason: None,
            legal_access_min_factor: Some(sar.legal_access_min_factor),
            wrong_way_min_factor: Some(sar.wrong_way_min_factor),
            vehicle_on_paths_min_factor: Some(sar.vehicle_on_paths_min_factor),
        }
    }

    pub fn supports(&self, kind: SarRelaxationKind) -> bool {
        self.available.contains(&kind)
    }
}

/// Whether SAR relaxations apply to `profile`: the vehicle profiles, whose
/// costs they surcharge (`Default` is Wheeled).
pub(crate) fn sar_profile(profile: MobilityProfile) -> bool {
    matches!(
        profile,
        MobilityProfile::Wheeled
            | MobilityProfile::Default
            | MobilityProfile::Emergency
            | MobilityProfile::Tracked
    )
}

/// Checks a request's SAR option against the package's capability: SAR
/// surcharges a vehicle profile's costs, a relaxation without package
/// data fails with `SarUnavailable` (as `vehicle_on_paths` does on Emergency
/// and Tracked), and a surcharge below the package minimum is an invalid
/// request (never silently clamped).
pub fn validate_request(
    sar: Option<&RouteSarOptions>,
    profile: MobilityProfile,
    capability: &RouteSarCapability,
) -> Result<(), RouteError> {
    let Some(sar) = sar else {
        return Ok(());
    };
    if !sar_profile(profile) {
        return Err(RouteError::InvalidRequest {
            reason: "sar requires a vehicle profile",
        });
    }
    let wheeled = matches!(profile, MobilityProfile::Wheeled | MobilityProfile::Default);
    for kind in sar.enabled() {
        if !capability.supports(kind) || (kind == SarRelaxationKind::VehicleOnPaths && !wheeled) {
            return Err(RouteError::SarUnavailable { relaxation: kind });
        }
    }
    let below_minimum = Err(RouteError::InvalidRequest {
        reason: "sar surcharge below package minimum",
    });
    let at_least = |value: f64, minimum: f64| value.is_finite() && value >= minimum;
    if let Some(penalty_s) = sar.turn_restriction_penalty_s()
        && !at_least(penalty_s, SAR_MIN_TURN_RESTRICTION_PENALTY_S)
    {
        return below_minimum;
    }
    let r = &sar.relaxations;
    if let Some(legal) = &r.legal_access {
        if !at_least(
            legal.factor,
            capability.legal_access_min_factor.unwrap_or(f64::INFINITY),
        ) {
            return below_minimum;
        }
        if legal.values.iter().any(|value| {
            !LegalValue::from_osm(value)
                .is_some_and(|value| value.denies() || value.limits_to_destination())
        }) {
            return Err(RouteError::InvalidRequest {
                reason: "sar legal_access value is not a denying access value",
            });
        }
    }
    if let Some(wrong_way) = &r.wrong_way
        && !(at_least(
            wrong_way.factor,
            capability.wrong_way_min_factor.unwrap_or(f64::INFINITY),
        ) && at_least(wrong_way.entry_s, 0.0))
    {
        return below_minimum;
    }
    if let Some(barriers) = &r.barriers
        && !(at_least(barriers.gate_s, 0.0) && at_least(barriers.locked_gate_s, 0.0))
    {
        return below_minimum;
    }
    if let Some(paths) = &r.vehicle_on_paths {
        if !at_least(
            paths.factor,
            capability
                .vehicle_on_paths_min_factor
                .unwrap_or(f64::INFINITY),
        ) {
            return below_minimum;
        }
        if paths
            .classes
            .iter()
            .any(|class| SarPathClass::from_name(class).is_none())
        {
            return Err(RouteError::InvalidRequest {
                reason: "sar vehicle_on_paths class is not a drivable path class",
            });
        }
        if !at_least(paths.min_width_m, 0.0) {
            return Err(RouteError::InvalidRequest {
                reason: "sar vehicle_on_paths min_width_m",
            });
        }
    }
    if let Some(max_leg_m) = sar.off_network_max_leg_m()
        && !(max_leg_m.is_finite() && max_leg_m > 0.0 && max_leg_m <= SAR_MAX_OFF_NETWORK_LEG_M)
    {
        return Err(RouteError::InvalidRequest {
            reason: "sar off_network max_leg_m",
        });
    }
    Ok(())
}

/// Whether a vehicle request's R1 names `private` or `customers` (its
/// default does): it may then cross destination-only roads, as it could
/// cross private ones when they were closures R1 opened.
pub(crate) fn lifts_destination_only(
    options: &crate::types::RouteOptions,
    profile: MobilityProfile,
) -> bool {
    sar_profile(profile)
        && options
            .sar
            .as_ref()
            .and_then(|sar| sar.relaxations.legal_access.as_ref())
            .is_some_and(|legal| {
                legal.values.iter().any(|value| {
                    LegalValue::from_osm(value).is_some_and(LegalValue::limits_to_destination)
                })
            })
}

/// The R1 and R2 of a vehicle request as the edge costing applies them,
/// when it enables either.
pub(crate) fn edge_relaxations(
    options: &crate::types::RouteOptions,
    profile: MobilityProfile,
) -> Option<SarEdgeRelaxations> {
    if !sar_profile(profile) {
        return None;
    }
    let relaxations = &options.sar.as_ref()?.relaxations;
    let legal_access = relaxations.legal_access.as_ref().map(|legal| {
        let values = legal
            .values
            .iter()
            .filter_map(|value| LegalValue::from_osm(value))
            .filter(|value| value.denies())
            .fold(0u16, |bits, value| bits | value.bit());
        (values, legal.factor)
    });
    let wrong_way = relaxations
        .wrong_way
        .as_ref()
        .map(|wrong_way| wrong_way.factor);
    let vehicle_on_paths = relaxations
        .vehicle_on_paths
        .as_ref()
        .map(SarVehicleOnPaths::relaxation);
    (legal_access.is_some() || wrong_way.is_some() || vehicle_on_paths.is_some()).then_some(
        SarEdgeRelaxations {
            legal_access,
            wrong_way,
            vehicle_on_paths,
        },
    )
}

impl SarVehicleOnPaths {
    /// As the edge costing applies it: class and width bits, and the factor.
    pub(crate) fn relaxation(&self) -> SarPathRelaxation {
        SarPathRelaxation {
            classes: self
                .classes
                .iter()
                .filter_map(|class| SarPathClass::from_name(class))
                .fold(0, |bits, class| bits | class.bit()),
            widths: SarPathRelaxation::width_bits(
                self.min_width_m,
                self.unknown_width == SarUnknownWidth::Allow,
            ),
            factor: self.factor,
        }
    }
}

/// Which edges a SAR request's R1, R2 and R5 open (factors aside): what
/// endpoint correlation may snap onto beyond the legal edges.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SarOpenEdges {
    /// R1's legal values (`LegalValue::bit`s), when R1 is enabled.
    pub legal_values: Option<u16>,
    pub wrong_way: bool,
    /// R5's path class and width class bits, when R5 is enabled.
    pub paths: Option<(u8, u8)>,
}

impl SarOpenEdges {
    /// The open edges of a request's SAR option (vehicle requests only, which
    /// validation enforces), when R1, R2 or R5 is enabled.
    pub(crate) fn of(options: &crate::types::RouteOptions) -> Option<Self> {
        let relaxations = edge_relaxations(options, MobilityProfile::Wheeled)?;
        Some(Self {
            legal_values: relaxations.legal_access.map(|(values, _)| values),
            wrong_way: relaxations.wrong_way.is_some(),
            paths: relaxations
                .vehicle_on_paths
                .map(|paths| (paths.classes, paths.widths)),
        })
    }

    /// As relaxations at factor 1 (only which edges open matters).
    pub(crate) fn relaxations(self) -> SarEdgeRelaxations {
        SarEdgeRelaxations {
            legal_access: self.legal_values.map(|values| (values, 1.0)),
            wrong_way: self.wrong_way.then_some(1.0),
            vehicle_on_paths: self.paths.map(|(classes, widths)| SarPathRelaxation {
                classes,
                widths,
                factor: 1.0,
            }),
        }
    }
}

/// The SAR costs a request adds at transitions between edges (outside the
/// bounds' metric, like turn costs): R2's entry into a wrong-way stretch and
/// R4's barriers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SarTransitions {
    /// R2: seconds on entering an edge against a one-way from one that is
    /// not.
    pub wrong_way_entry_s: Option<f64>,
    /// R4: (gate, locked gate) seconds.
    pub barriers: Option<(f64, f64)>,
}

impl SarTransitions {
    /// The transitions of a vehicle request, when it has any.
    pub(crate) fn of(
        options: &crate::types::RouteOptions,
        profile: MobilityProfile,
    ) -> Option<Self> {
        if !sar_profile(profile) {
            return None;
        }
        let relaxations = &options.sar.as_ref()?.relaxations;
        let transitions = Self {
            wrong_way_entry_s: relaxations
                .wrong_way
                .as_ref()
                .map(|wrong_way| wrong_way.entry_s)
                .filter(|entry_s| *entry_s > 0.0),
            barriers: relaxations
                .barriers
                .as_ref()
                .map(|barriers| (barriers.gate_s, barriers.locked_gate_s)),
        };
        (transitions.wrong_way_entry_s.is_some() || transitions.barriers.is_some())
            .then_some(transitions)
    }

    /// R4's cost of passing a barrier of `class` (infinite: closed).
    pub(crate) fn barrier_cost(&self, class: SarBarrierClass) -> f64 {
        let Some((gate_s, locked_gate_s)) = self.barriers else {
            return 0.0;
        };
        match class {
            SarBarrierClass::Gate | SarBarrierClass::Unknown => gate_s,
            SarBarrierClass::LockedGate => locked_gate_s,
            SarBarrierClass::MotorBlocking | SarBarrierClass::FootOnly => f64::INFINITY,
        }
    }

    /// R2's cost moving from compact edge `previous` onto `next` (R4's
    /// barriers are costed with every barrier, `crate::transitions`).
    #[inline]
    pub(crate) fn entry_cost(
        &self,
        g: &wayfarer_package::format::Sections<'_>,
        previous: u32,
        next: u32,
    ) -> f64 {
        let Some(entry_s) = self.wrong_way_entry_s else {
            return 0.0;
        };
        let wrong = |edge: u32| g.edge_access.sar(edge as usize) & ACCESS_WRONG_WAY != 0;
        if wrong(next) && !wrong(previous) {
            entry_s
        } else {
            0.0
        }
    }
}

/// Whether a barrier at `node` applies to the transition from compact edge
/// `previous` onto `next`: when
/// either edge's way ends at the node, or when no way ends there (every way
/// passes through, so the barrier closes them all). A U-turn on one segment
/// does not pass the barrier.
pub(crate) fn barrier_applies(
    g: &wayfarer_package::format::Sections<'_>,
    node: u32,
    previous: u32,
    next: u32,
) -> bool {
    if g.edge_shared_info[previous as usize] == g.edge_shared_info[next as usize] {
        return false;
    }
    let node = node as usize;
    let incident = || {
        let outgoing = g.first_out[node]..g.first_out[node + 1];
        let incoming = g.in_edges(node).map(|(edge, _)| edge);
        outgoing.chain(incoming)
    };
    // A way passes through the node when another of its segments (another
    // shared info of the same way) meets there.
    let passes_through = |shared: usize| {
        incident().any(|edge| {
            let other = g.edge_shared_info[edge as usize] as usize;
            other != shared && g.si_way.same_way(shared, other)
        })
    };
    let ends = |edge: u32| !passes_through(g.edge_shared_info[edge as usize] as usize);
    if ends(previous) || ends(next) {
        return true;
    }
    incident().all(|edge| !ends(edge))
}

/// The R3 penalty per violated restriction of a request, when it enables
/// the relaxation (`None`: restrictions ban, as without SAR).
pub(crate) fn restriction_penalty(options: &crate::types::RouteOptions) -> Option<f64> {
    options
        .sar
        .as_ref()
        .and_then(RouteSarOptions::turn_restriction_penalty_s)
}

/// What a route did under SAR (`LegResult::sar`, SAR requests only).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteSarReport {
    /// The relaxations the request enabled.
    pub enabled: Vec<SarRelaxationKind>,
    /// Every place the route uses a relaxation, in route order. An operator
    /// must see these before driving.
    pub uses: Vec<RouteSarUse>,
    /// Per relaxation used: how often, how far, and the cost it added.
    pub totals: Vec<RouteSarUseTotal>,
}

/// One use of a relaxation on a route.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteSarUse {
    pub kind: SarRelaxationKind,
    /// The route edges involved: for `turn_restrictions` the restriction's
    /// whole sequence (from, via…, to), for `conditional` the closed edge(s),
    /// empty for `off_network`.
    pub edge_ids: Vec<u64>,
    /// The traversed piece of the first and last edge.
    pub start_fraction: f64,
    pub end_fraction: f64,
    pub length_m: f64,
    /// Cost the relaxation added (seconds): the R3 penalty, or for R7 the
    /// leg's time at the off-road pace (which the route's own cost and
    /// duration leave out), or with `costed` the leg's cost, which the
    /// route's cost includes (and, at the costed leg's cost factor of 1,
    /// its time, which the duration includes).
    pub added_cost_s: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub osm_way_id: Option<i64>,
    /// The rule broken: `restriction=simple` / `restriction=complex`, the
    /// conditional tag, or `off_network`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub legal_tag: Option<String>,
    /// Data-quality notes for the operator.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub confidence: Vec<String>,
    /// For `off_network`: the requested point and the road point it joins.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub geometry: Vec<GeoPoint>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteSarUseTotal {
    pub kind: SarRelaxationKind,
    pub count: usize,
    pub length_m: f64,
    pub added_cost_s: f64,
}

/// The effective SAR surcharges of a request (`RoutePolicySummary::sar`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteSarPolicy {
    pub enabled: Vec<SarRelaxationKind>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub turn_restriction_penalty_s: Option<f64>,
    pub ignore_conditional: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub off_network_max_leg_m: Option<f64>,
    /// R1: the access values it opens and its factor.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub legal_access_values: Vec<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub legal_access_factor: Option<f64>,
    /// R2: its factor and entry time.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub wrong_way_factor: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub wrong_way_entry_s: Option<f64>,
    /// R4: a gate's and a locked gate's time.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub barrier_gate_s: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub barrier_locked_gate_s: Option<f64>,
    /// R5: the path classes it opens, its factor, the least known width and
    /// whether unknown widths are allowed.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub vehicle_on_paths_classes: Vec<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vehicle_on_paths_factor: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vehicle_on_paths_min_width_m: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vehicle_on_paths_unknown_width: Option<SarUnknownWidth>,
}

impl RouteSarPolicy {
    pub(crate) fn of(sar: &RouteSarOptions) -> Self {
        let r = &sar.relaxations;
        Self {
            enabled: sar.enabled(),
            turn_restriction_penalty_s: sar.turn_restriction_penalty_s(),
            ignore_conditional: sar.ignores_conditional(),
            off_network_max_leg_m: sar.off_network_max_leg_m(),
            legal_access_values: r
                .legal_access
                .as_ref()
                .map(|legal| legal.values.clone())
                .unwrap_or_default(),
            legal_access_factor: r.legal_access.as_ref().map(|legal| legal.factor),
            wrong_way_factor: r.wrong_way.as_ref().map(|wrong_way| wrong_way.factor),
            wrong_way_entry_s: r.wrong_way.as_ref().map(|wrong_way| wrong_way.entry_s),
            barrier_gate_s: r.barriers.as_ref().map(|barriers| barriers.gate_s),
            barrier_locked_gate_s: r.barriers.as_ref().map(|barriers| barriers.locked_gate_s),
            vehicle_on_paths_classes: r
                .vehicle_on_paths
                .as_ref()
                .map(|paths| paths.classes.clone())
                .unwrap_or_default(),
            vehicle_on_paths_factor: r.vehicle_on_paths.as_ref().map(|paths| paths.factor),
            vehicle_on_paths_min_width_m: r
                .vehicle_on_paths
                .as_ref()
                .map(|paths| paths.min_width_m),
            vehicle_on_paths_unknown_width: r
                .vehicle_on_paths
                .as_ref()
                .map(|paths| paths.unknown_width),
        }
    }
}

/// The qualifier of a legal right that is not plain `yes`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteAccessQualifier {
    /// Access for destination traffic only.
    Destination,
    /// Access the owner tolerates and may withdraw.
    Permissive,
    /// Access with a permit.
    Permit,
    /// Access for deliveries.
    Delivery,
    /// Private access: vehicles going somewhere on the way (with the owner's
    /// leave), no through traffic.
    Private,
    /// Access for the customers of a place on the way.
    Customers,
    /// The profile may use a way whose class excludes it by default
    /// (`highway=pedestrian` open to cars): some tag granted it, but the
    /// package does not record which, so the qualifier is unknown.
    Unrecorded,
}

/// One stretch of a route whose legal access is qualified
/// (`LegResult::qualified_access`, on every route).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteQualifiedAccess {
    pub qualifier: RouteAccessQualifier,
    /// Consecutive route edges of one way with this qualifier.
    pub edge_ids: Vec<u64>,
    pub start_fraction: f64,
    pub end_fraction: f64,
    pub length_m: f64,
    pub osm_way_id: i64,
    /// The tag that qualifies the access, as the package records it
    /// (`access=permissive`, `motor_vehicle:conditional=delivery @ …`); for
    /// `destination` without a recorded `access=destination` it is
    /// `destination` (the key is not recorded), for `unrecorded` the way's
    /// class (`highway=pedestrian`).
    pub legal_tag: String,
    /// Whether the qualifier holds only under a condition (a conditional
    /// tag), at the time the request was evaluated or not.
    #[serde(default)]
    pub conditional: bool,
}

// ---------------------------------------------------------------------------
// Route reports
// ---------------------------------------------------------------------------

/// One traversed piece of a graph edge on a route, in route order.
pub(crate) struct RoutePiece<'a> {
    pub record: &'a crate::graph_view::GraphViewEdge,
    pub start_fraction: f64,
    pub end_fraction: f64,
    /// The piece follows an overlay connector: a new restriction context.
    pub after_connector: bool,
}

impl RoutePiece<'_> {
    fn length_m(&self) -> f64 {
        f64::from(self.record.edge.distance_m)
            * (self.end_fraction.clamp(0.0, 1.0) - self.start_fraction.clamp(0.0, 1.0)).abs()
    }
}

/// Values of an access tag that qualify a legal right.
fn qualifier_of(value: &str) -> Option<RouteAccessQualifier> {
    match value.trim() {
        "destination" => Some(RouteAccessQualifier::Destination),
        "permissive" => Some(RouteAccessQualifier::Permissive),
        "permit" => Some(RouteAccessQualifier::Permit),
        "delivery" => Some(RouteAccessQualifier::Delivery),
        "private" => Some(RouteAccessQualifier::Private),
        "customers" => Some(RouteAccessQualifier::Customers),
        _ => None,
    }
}

/// Classes whose default access excludes vehicles.
fn vehicle_excluding_class(highway: &str) -> bool {
    matches!(
        highway,
        "pedestrian" | "footway" | "path" | "cycleway" | "bridleway" | "steps" | "corridor"
    )
}

fn is_vehicle(profile: MobilityProfile) -> bool {
    !matches!(profile, MobilityProfile::OnFoot | MobilityProfile::Legged)
}

/// The access keys that decide the profile's legal access.
fn access_keys(profile: MobilityProfile) -> &'static [&'static str] {
    if is_vehicle(profile) {
        &["access", "vehicle", "motor_vehicle", "motorcar"]
    } else {
        &["access", "foot"]
    }
}

/// The qualifiers of one edge for `profile`, with the tag that sets each, as
/// far as the package records them. For a vehicle on a package with legal
/// reasons, the tag deciding vehicle access (`motor_vehicle=permissive`);
/// otherwise the way's `access` value, the destination-only flag (set by
/// the most specific of `motorcar`, `motor_vehicle`, `vehicle` and `access`
/// = `destination`, `private`, `customers`, `permit` or `delivery`) and,
/// for vehicles, access on a class that excludes them by default (the
/// granting tag is not recorded). Conditional access tags with a qualifying
/// value in both cases.
fn edge_qualifiers(
    record: &crate::graph_view::GraphViewEdge,
    profile: MobilityProfile,
) -> Vec<(RouteAccessQualifier, String, bool)> {
    let info = &record.shared_info;
    let mut qualifiers = Vec::new();
    // A package with SAR data records the tag that decides vehicle access
    // (`motor_vehicle=permissive`): for vehicles it replaces what older
    // packages could only infer from `access`, the destination flag and the
    // way's class.
    let reason = wayfarer_package::sar::LegalReason(info.legal_reason);
    let recorded = is_vehicle(profile) && reason.key().is_some();
    if recorded {
        if let Some(tag) = reason.tag()
            && let Some(qualifier) = qualifier_of(reason.value().as_str())
        {
            qualifiers.push((qualifier, tag, false));
        }
    } else {
        let access = info.access.as_deref().and_then(|value| {
            qualifier_of(value).map(|qualifier| (qualifier, format!("access={}", value.trim())))
        });
        // `access` alone may make the way destination-only.
        let access_limits = matches!(
            access,
            Some((
                RouteAccessQualifier::Destination
                    | RouteAccessQualifier::Permit
                    | RouteAccessQualifier::Delivery
                    | RouteAccessQualifier::Private
                    | RouteAccessQualifier::Customers,
                _
            ))
        );
        if let Some((qualifier, tag)) = access {
            qualifiers.push((qualifier, tag, false));
        }
        if is_vehicle(profile) && info.is_destination_only && !access_limits {
            qualifiers.push((
                RouteAccessQualifier::Destination,
                "destination".to_string(),
                false,
            ));
        }
    }
    for tag in &info.conditional_restrictions {
        let Some((key, value)) = tag.split_once('=') else {
            continue;
        };
        let Some(base) = key.trim().strip_suffix(":conditional") else {
            continue;
        };
        if !access_keys(profile).contains(&base) {
            continue;
        }
        let mut found = value
            .split(';')
            .filter_map(|part| qualifier_of(part.split('@').next().unwrap_or_default()))
            .collect::<Vec<_>>();
        found.sort_unstable();
        found.dedup();
        qualifiers.extend(
            found
                .into_iter()
                .map(|qualifier| (qualifier, tag.clone(), true)),
        );
    }
    if is_vehicle(profile)
        && !recorded
        && let Some(highway) = info.highway.as_deref()
        && vehicle_excluding_class(highway)
    {
        qualifiers.push((
            RouteAccessQualifier::Unrecorded,
            format!("highway={highway}"),
            false,
        ));
    }
    // Private and customer roads are closed to walkers: a walker on one has
    // a `foot` grant, which the package does not record.
    if !is_vehicle(profile) {
        qualifiers.retain(|(qualifier, ..)| {
            !matches!(
                qualifier,
                RouteAccessQualifier::Private | RouteAccessQualifier::Customers
            )
        });
    }
    qualifiers
}

/// Qualified access along a route's pieces (every route).
/// Consecutive pieces of one way with the same qualifier and tag form one
/// entry. Edges the request opened are reported as overlay uses, not here.
pub(crate) fn qualified_access(
    pieces: &[RoutePiece<'_>],
    profile: MobilityProfile,
    options: &crate::types::RouteOptions,
) -> Vec<RouteQualifiedAccess> {
    let mut entries = Vec::<RouteQualifiedAccess>::new();
    // The entries the previous piece extended or opened.
    let mut open = Vec::<usize>::new();
    for piece in pieces {
        let record = piece.record;
        if options.opened_edge_ids.contains(&record.edge.edge_id) {
            open.clear();
            continue;
        }
        let mut next_open = Vec::new();
        for (qualifier, legal_tag, conditional) in edge_qualifiers(record, profile) {
            let osm_way_id = record.shared_info.osm_way_id;
            let continued = open.iter().copied().find(|index| {
                let entry = &entries[*index];
                entry.qualifier == qualifier
                    && entry.legal_tag == legal_tag
                    && entry.osm_way_id == osm_way_id
            });
            match continued {
                Some(index) => {
                    let entry = &mut entries[index];
                    if entry.edge_ids.last() != Some(&record.edge.edge_id) {
                        entry.edge_ids.push(record.edge.edge_id);
                    }
                    entry.end_fraction = piece.end_fraction;
                    entry.length_m += piece.length_m();
                    next_open.push(index);
                }
                None => {
                    entries.push(RouteQualifiedAccess {
                        qualifier,
                        edge_ids: vec![record.edge.edge_id],
                        start_fraction: piece.start_fraction,
                        end_fraction: piece.end_fraction,
                        length_m: piece.length_m(),
                        osm_way_id,
                        legal_tag,
                        conditional,
                    });
                    next_open.push(entries.len() - 1);
                }
            }
        }
        open = next_open;
    }
    entries
}

/// The compact edge of a route piece: its record's raw index, which a route
/// over the package carries (no edge id lookup).
fn piece_edge(graph: crate::package::SearchGraph<'_>, piece: &RoutePiece<'_>) -> Option<u32> {
    graph.edge_for_id_hinted(
        u32::try_from(piece.record.raw_edge_index).ok(),
        piece.record.edge.edge_id,
    )
}

/// The SAR report of a route from its pieces: the restrictions it breaks
/// under `turn_restrictions` and the conditional closures it ignores under
/// `conditional`. Off-network legs are added by [`add_off_network_uses`],
/// which knows the requested points.
pub(crate) fn route_report(
    view: &crate::graph_view::PackageGraphView,
    pieces: &[RoutePiece<'_>],
    profile: MobilityProfile,
    options: &crate::types::RouteOptions,
) -> Option<RouteSarReport> {
    let sar = options.sar.as_ref()?;
    // (index of the first piece involved, use), sorted into route order.
    let mut uses = Vec::<(usize, RouteSarUse)>::new();
    let graph = view.routing_package().view();
    if let Some(penalty_s) = sar.turn_restriction_penalty_s() {
        let bit = graph.profile_bit(profile);
        // Restriction contexts: the traversed edges between connectors, an
        // edge split into several pieces counting once.
        let mut contexts = Vec::<Vec<usize>>::new();
        for (index, piece) in pieces.iter().enumerate() {
            match contexts.last_mut() {
                Some(context) if !piece.after_connector => {
                    let last = pieces[*context.last().expect("non-empty")]
                        .record
                        .edge
                        .edge_id;
                    if last != piece.record.edge.edge_id {
                        context.push(index);
                    }
                }
                _ => contexts.push(vec![index]),
            }
        }
        for context in contexts {
            let edges = context
                .iter()
                .filter_map(|index| piece_edge(graph, &pieces[*index]))
                .collect::<Vec<_>>();
            if edges.len() != context.len() {
                continue;
            }
            for violation in graph.restriction_violations(&edges, bit) {
                let first = &pieces[context[violation.first]];
                let last = &pieces[context[violation.last]];
                uses.push((
                    context[violation.first],
                    RouteSarUse {
                        kind: SarRelaxationKind::TurnRestrictions,
                        edge_ids: context[violation.first..=violation.last]
                            .iter()
                            .map(|index| pieces[*index].record.edge.edge_id)
                            .collect(),
                        start_fraction: first.start_fraction,
                        end_fraction: last.end_fraction,
                        length_m: 0.0,
                        added_cost_s: penalty_s,
                        osm_way_id: Some(last.record.shared_info.osm_way_id),
                        legal_tag: Some(
                            if violation.complex {
                                "type=restriction (via way)"
                            } else {
                                "type=restriction"
                            }
                            .to_string(),
                        ),
                        confidence: Vec::new(),
                        geometry: Vec::new(),
                    },
                ));
            }
        }
    }
    if sar.ignores_conditional() {
        let mut open = None::<usize>;
        for (index, piece) in pieces.iter().enumerate() {
            let info = &piece.record.shared_info;
            let Some(tag) = wayfarer_costing::active_conditional_access_closure(
                &info.conditional_restrictions,
                &options.costing,
                profile,
            ) else {
                open = None;
                continue;
            };
            if let Some(last) = open.map(|at| &mut uses[at].1).filter(|last| {
                last.osm_way_id == Some(info.osm_way_id) && last.legal_tag.as_deref() == Some(tag)
            }) {
                extend_use(last, piece);
                continue;
            }
            uses.push((
                index,
                RouteSarUse {
                    kind: SarRelaxationKind::Conditional,
                    edge_ids: vec![piece.record.edge.edge_id],
                    start_fraction: piece.start_fraction,
                    end_fraction: piece.end_fraction,
                    length_m: piece.length_m(),
                    added_cost_s: 0.0,
                    osm_way_id: Some(info.osm_way_id),
                    legal_tag: Some(tag.to_string()),
                    confidence: Vec::new(),
                    geometry: Vec::new(),
                },
            ));
            open = Some(uses.len() - 1);
        }
    }
    physical_uses(&graph, pieces, profile, options, &mut uses);
    uses.sort_by_key(|(position, _)| *position);
    let uses = uses.into_iter().map(|(_, used)| used).collect::<Vec<_>>();
    Some(RouteSarReport {
        enabled: sar.enabled(),
        totals: totals(&uses),
        uses,
    })
}

fn extend_use(last: &mut RouteSarUse, piece: &RoutePiece<'_>) {
    if last.edge_ids.last() != Some(&piece.record.edge.edge_id) {
        last.edge_ids.push(piece.record.edge.edge_id);
    }
    last.end_fraction = piece.end_fraction;
    last.length_m += piece.length_m();
}

/// Data-quality notes on a way a SAR relaxation opens for a vehicle: rough
/// tracks and surfaces, a hiking grade, and an unknown width (on a path, R5:
/// neither `width` nor `est_width`; elsewhere neither `width` nor
/// `maxwidth`).
fn vehicle_confidence(
    info: &wayfarer_package::SharedEdgeInfo,
    path: Option<SarWidthClass>,
) -> Vec<String> {
    let mut notes = Vec::new();
    if let Some(grade) = info.tracktype.as_deref().filter(|grade| *grade == "grade5") {
        notes.push(format!("tracktype={grade}"));
    }
    if let Some(smoothness) = info
        .smoothness
        .as_deref()
        .filter(|value| matches!(*value, "horrible" | "very_horrible" | "impassable"))
    {
        notes.push(format!("smoothness={smoothness}"));
    }
    if let Some(scale) = info.sac_scale.as_deref() {
        notes.push(format!("sac_scale={scale}"));
    }
    let width_unknown = match path {
        Some(width) => width == SarWidthClass::Unknown,
        None => info.physical_width_cm.is_none() && info.max_width_cm.is_none(),
    };
    if width_unknown {
        notes.push("width unknown".to_string());
    }
    notes
}

/// The uses of the relaxations that need SAR package data: `legal_access`,
/// `wrong_way` and `vehicle_on_paths` per stretch of one way, with the cost
/// each adds, R2's entries, and every barrier `barriers` passes.
fn physical_uses(
    graph: &crate::package::SearchGraph<'_>,
    pieces: &[RoutePiece<'_>],
    profile: MobilityProfile,
    options: &crate::types::RouteOptions,
    uses: &mut Vec<(usize, RouteSarUse)>,
) {
    let relaxations = edge_relaxations(options, profile);
    let transitions = SarTransitions::of(options, profile);
    if relaxations.is_none() && transitions.is_none() {
        return;
    }
    let bit = graph.profile_bit(profile);
    let policy = crate::costing::CostingPolicy::new(profile, options);
    let compact = |piece: &RoutePiece<'_>| piece_edge(*graph, piece);
    // The stretch each kind extends, while consecutive pieces continue it.
    let mut open = [None::<usize>; 3];
    let mut previous = None::<(u32, &RoutePiece<'_>)>;
    for (index, piece) in pieces.iter().enumerate() {
        let Some(edge) = compact(piece) else {
            open = [None; 3];
            previous = None;
            continue;
        };
        let info = &piece.record.shared_info;
        let (access, sar_bits) = (
            graph.edge_access.profiles(edge as usize),
            graph.edge_access.sar(edge as usize),
        );
        let shared = graph.edge_shared_info[edge as usize] as usize;
        let facts = SarEdgeFacts::from_cost_classes(
            sar_bits,
            graph.legal_reason(shared),
            graph.si_cost_classes[graph.cost_tuple(shared)],
        );
        let reason = facts.reason;
        // Transition costs at the start of this piece (R2 entry, R4).
        let mut entry_s = 0.0;
        if let (Some(transitions), Some((from, from_piece))) = (transitions, previous)
            && from != edge
            && !piece.after_connector
        {
            if let Some(entry) = transitions.wrong_way_entry_s
                && sar_bits & ACCESS_WRONG_WAY != 0
                && graph.edge_access.sar(from as usize) & ACCESS_WRONG_WAY == 0
            {
                entry_s = entry;
            }
            let node = graph.edge_target[from as usize];
            if transitions.barriers.is_some()
                && let Some(class) = graph.barrier(node)
                && barrier_applies(graph, node, from, edge)
            {
                let cost = transitions.barrier_cost(class);
                let point = GeoPoint {
                    lat: from_piece.record.target.lat,
                    lng: from_piece.record.target.lng,
                };
                uses.push((
                    index,
                    RouteSarUse {
                        kind: SarRelaxationKind::Barriers,
                        edge_ids: vec![from_piece.record.edge.edge_id, piece.record.edge.edge_id],
                        start_fraction: from_piece.end_fraction,
                        end_fraction: piece.start_fraction,
                        length_m: 0.0,
                        added_cost_s: cost,
                        osm_way_id: Some(info.osm_way_id),
                        legal_tag: Some(format!("barrier:{}", class.as_str())),
                        confidence: if class == SarBarrierClass::Unknown {
                            vec!["barrier of unknown type".to_string()]
                        } else {
                            Vec::new()
                        },
                        geometry: vec![point],
                    },
                ));
            }
        }
        previous = Some((edge, piece));
        let needs = (access & bit == 0)
            .then(|| relaxations.and_then(|relaxations| relaxations.factor(facts)))
            .flatten()
            .and_then(|_| SarEdgeRelaxations::needs(facts));
        let Some(needs) = needs else {
            open = [None; 3];
            continue;
        };
        let relaxations = relaxations.expect("an opened edge has relaxations");
        let legal_factor = if needs.legal_access {
            relaxations.legal_access.map_or(1.0, |(_, factor)| factor)
        } else {
            1.0
        };
        let wrong_way_factor = if needs.wrong_way {
            relaxations.wrong_way.unwrap_or(1.0)
        } else {
            1.0
        };
        let path_factor = if needs.vehicle_on_paths {
            relaxations
                .vehicle_on_paths
                .map_or(1.0, |paths| paths.factor)
        } else {
            1.0
        };
        // The edge's cost with legal access: its SAR cost over the factors.
        let base = match policy.edge_cost(piece.record) {
            crate::costing::EdgeCostDecision::Allowed(costed) => {
                costed.cost / (legal_factor * wrong_way_factor * path_factor)
            }
            _ => 0.0,
        };
        let share =
            (piece.end_fraction.clamp(0.0, 1.0) - piece.start_fraction.clamp(0.0, 1.0)).abs();
        let kinds = [
            (
                SarRelaxationKind::LegalAccess,
                needs.legal_access,
                base * (legal_factor - 1.0) * share,
                reason.tag().unwrap_or_else(|| "access".to_string()),
            ),
            (
                SarRelaxationKind::WrongWay,
                needs.wrong_way,
                base * legal_factor * (wrong_way_factor - 1.0) * share + entry_s,
                match info.directional_access {
                    wayfarer_package::DirectionalAccess::Backward => "oneway=-1",
                    _ => "oneway=yes",
                }
                .to_string(),
            ),
            (
                SarRelaxationKind::VehicleOnPaths,
                needs.vehicle_on_paths,
                base * legal_factor * wrong_way_factor * (path_factor - 1.0) * share,
                format!("highway={}", facts.path.as_str()),
            ),
        ];
        for (slot, (kind, needed, added, tag)) in kinds.into_iter().enumerate() {
            if !needed {
                open[slot] = None;
                continue;
            }
            let continues = open[slot].map(|at| &mut uses[at].1).filter(|last| {
                last.osm_way_id == Some(info.osm_way_id) && last.legal_tag.as_deref() == Some(&tag)
            });
            // An R2 entry starts a new stretch.
            if let Some(last) =
                continues.filter(|_| kind != SarRelaxationKind::WrongWay || entry_s == 0.0)
            {
                extend_use(last, piece);
                last.added_cost_s += added;
                continue;
            }
            uses.push((
                index,
                RouteSarUse {
                    kind,
                    edge_ids: vec![piece.record.edge.edge_id],
                    start_fraction: piece.start_fraction,
                    end_fraction: piece.end_fraction,
                    length_m: piece.length_m(),
                    added_cost_s: added,
                    osm_way_id: Some(info.osm_way_id),
                    legal_tag: Some(tag),
                    confidence: vehicle_confidence(
                        info,
                        needs.vehicle_on_paths.then_some(facts.width),
                    ),
                    geometry: Vec::new(),
                },
            ));
            open[slot] = Some(uses.len() - 1);
        }
    }
}

/// Adds the off-network legs of a SAR route (`off_network`): an endpoint
/// snapped farther from the requested point than the request's own
/// correlation distance, which only the relaxation's wider search reaches.
/// The leg's time is at the profile's off-road pace; the route's own cost
/// and duration leave it out, as they leave out every snap. A `costed` leg
/// (X6) is in the route's totals, and `added_cost_s` is the cost they
/// include.
pub(crate) fn add_off_network_uses(
    result: &mut crate::types::LegResult,
    origin: GeoPoint,
    destination: GeoPoint,
    profile: MobilityProfile,
    options: &crate::types::RouteOptions,
) {
    let Some(report) = result.sar.as_mut() else {
        return;
    };
    let enabled = options
        .sar
        .as_ref()
        .and_then(RouteSarOptions::off_network_max_leg_m)
        .is_some();
    if !enabled {
        return;
    }
    let pace = wayfarer_costing::profile_off_road_speed_mps(profile);
    let base = f64::from(options.max_correlation_distance_m);
    let legs = [
        (origin, &result.origin, true),
        (destination, &result.destination, false),
    ];
    let mut added = Vec::new();
    for (requested, location, is_origin) in legs {
        if location.distance_m <= base {
            continue;
        }
        // A costed leg (X6, `crate::access`) is in the route's totals: its
        // drawn line and the cost they include.
        let (geometry, added_cost_s) = match location.access {
            Some(_) => {
                let (from, to) = if is_origin {
                    (requested, location.snapped_point)
                } else {
                    (location.snapped_point, requested)
                };
                (
                    crate::direct_leg::densify_geodesic(
                        from,
                        to,
                        crate::direct_leg::DIRECT_LEG_VERTEX_SPACING_M,
                    ),
                    crate::access::AccessTotals::of(location).cost,
                )
            }
            None if is_origin => (
                vec![requested, location.snapped_point],
                location.distance_m / pace,
            ),
            None => (
                vec![location.snapped_point, requested],
                location.distance_m / pace,
            ),
        };
        added.push((
            is_origin,
            RouteSarUse {
                kind: SarRelaxationKind::OffNetwork,
                edge_ids: Vec::new(),
                start_fraction: 0.0,
                end_fraction: 1.0,
                length_m: location.distance_m,
                added_cost_s,
                osm_way_id: None,
                legal_tag: Some("off_network".to_string()),
                confidence: vec!["straight line across unsurveyed ground".to_string()],
                geometry,
            },
        ));
    }
    for (is_origin, leg) in added {
        if is_origin {
            report.uses.insert(0, leg);
        } else {
            report.uses.push(leg);
        }
    }
    report.totals = totals(&report.uses);
}

fn totals(uses: &[RouteSarUse]) -> Vec<RouteSarUseTotal> {
    let mut totals = Vec::<RouteSarUseTotal>::new();
    for kind in SarRelaxationKind::ALL {
        let mut total = RouteSarUseTotal {
            kind,
            count: 0,
            length_m: 0.0,
            added_cost_s: 0.0,
        };
        for used in uses.iter().filter(|used| used.kind == kind) {
            total.count += 1;
            total.length_m += used.length_m;
            total.added_cost_s += used.added_cost_s;
        }
        if total.count > 0 {
            totals.push(total);
        }
    }
    totals
}

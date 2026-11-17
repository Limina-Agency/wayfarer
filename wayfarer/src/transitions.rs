//! Costs a request adds at transitions between edges, outside the landmark
//! metric like turn costs (they only raise costs, so landmark bounds hold):
//! barriers and a SAR request's
//! entry into a wrong-way stretch (R2). A SAR request's `barriers`
//! relaxation (R4) replaces a barrier's normal effect on vehicles with its
//! own costs.

use wayfarer_package::barrier::{BarrierAccess, BarrierEffect};
use wayfarer_package::format::{EDGE_FLAG_BARRIER_AT_TARGET, Sections};
use wayfarer_package::sar::SarBarrierClass;

use crate::sar::{SarTransitions, barrier_applies};
use crate::types::MobilityProfile;

/// Seconds a vehicle loses passing a gate.
pub const GATE_VEHICLE_S: f64 = 30.0;
/// Seconds a walker loses passing a gate or a stile.
pub const GATE_ON_FOOT_S: f64 = 10.0;

/// The seconds `profile` loses passing a gate.
pub fn gate_s(profile: MobilityProfile) -> f64 {
    match profile {
        MobilityProfile::OnFoot | MobilityProfile::Legged => GATE_ON_FOOT_S,
        _ => GATE_VEHICLE_S,
    }
}

/// A request's transition costs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Transitions {
    profile_bit: u8,
    gate_s: f64,
    /// A SAR request's transition costs (R2 entry, R4 barriers).
    pub(crate) sar: Option<SarTransitions>,
}

impl Transitions {
    pub(crate) fn of(options: &crate::types::RouteOptions, profile: MobilityProfile) -> Self {
        Self {
            profile_bit: crate::package::profile_bit(profile),
            gate_s: gate_s(profile),
            sar: SarTransitions::of(options, profile),
        }
    }

    /// The cost added moving from compact edge `previous` onto `next`
    /// (infinite: a barrier closes the transition).
    #[inline]
    pub(crate) fn cost(&self, g: &Sections<'_>, previous: u32, next: u32) -> f64 {
        let mut cost = 0.0;
        if let Some(sar) = &self.sar {
            cost += sar.entry_cost(g, previous, next);
        }
        if g.edge_flags[previous as usize] & EDGE_FLAG_BARRIER_AT_TARGET != 0 {
            cost += self.barrier_cost(g, previous, next);
        }
        cost
    }

    /// The cost of the barrier at the node between `previous` and `next`,
    /// when the transition passes it.
    fn barrier_cost(&self, g: &Sections<'_>, previous: u32, next: u32) -> f64 {
        let node = g.edge_target[previous as usize];
        let Some((class, access)) = g.barrier_at(node) else {
            return 0.0;
        };
        let cost = self.barrier_cost_of(class, access);
        if cost == 0.0 || !barrier_applies(g, node, previous, next) {
            return 0.0;
        }
        cost
    }

    /// What passing a barrier of SAR `class` and normal `access` costs this
    /// request: R4's cost for a vehicle class when the request enables it,
    /// else the barrier's effect on the profile.
    pub(crate) fn barrier_cost_of(
        &self,
        class: Option<SarBarrierClass>,
        access: BarrierAccess,
    ) -> f64 {
        if let (Some(sar), Some(class)) = (self.sar.filter(|sar| sar.barriers.is_some()), class) {
            return sar.barrier_cost(class);
        }
        match access.effect(self.profile_bit) {
            BarrierEffect::Free => 0.0,
            BarrierEffect::Gate => self.gate_s,
            BarrierEffect::Closed => f64::INFINITY,
        }
    }
}

//! Barrier nodes in normal routing.
//!
//! A package lists the nodes with a barrier ([`crate::format::Section::BarrierNode`])
//! and stores, per node, a [`BarrierAccess`]: which profiles the barrier
//! closes and which pass it as a gate. The engine applies it at transitions
//! through the node (a barrier closes the ways that end at
//! its node, or every way when all pass through). A SAR request's `barriers`
//! relaxation replaces it for vehicles with its own costs per
//! [`crate::sar::SarBarrierClass`].

use crate::format::PROFILE_SLOTS;

/// How a barrier node affects each profile: bits 0-7 the profile bits it
/// closes, bits 8-15 those that pass it as a gate (with a delay). The two
/// sets are disjoint; profiles in neither pass freely.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct BarrierAccess(pub u16);

/// What a barrier does to one profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BarrierEffect {
    Free,
    Gate,
    Closed,
}

impl BarrierAccess {
    pub const NONE: Self = Self(0);

    /// The access closing the profile bits `closed` and gating `gate` (less
    /// any it closes).
    pub fn new(closed: u8, gate: u8) -> Self {
        Self(u16::from(closed) | (u16::from(gate & !closed) << 8))
    }

    pub fn closed(self) -> u8 {
        self.0 as u8
    }

    pub fn gate(self) -> u8 {
        (self.0 >> 8) as u8
    }

    /// Whether the barrier affects some profile.
    pub fn has_effect(self) -> bool {
        self.0 != 0
    }

    /// What the barrier does to the profile with `profile_bit`.
    #[inline]
    pub fn effect(self, profile_bit: u8) -> BarrierEffect {
        if self.closed() & profile_bit != 0 {
            BarrierEffect::Closed
        } else if self.gate() & profile_bit != 0 {
            BarrierEffect::Gate
        } else {
            BarrierEffect::Free
        }
    }

    /// Whether the value is one the format defines: disjoint sets of
    /// profile bits.
    pub fn is_valid(self) -> bool {
        let profiles = (1u16 << PROFILE_SLOTS) - 1;
        self.closed() & self.gate() == 0 && u16::from(self.closed() | self.gate()) & !profiles == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn closed_wins_over_gate_and_values_validate() {
        let access = BarrierAccess::new(0b0000_0110, 0b0011_0110);
        assert_eq!(access.closed(), 0b0000_0110);
        assert_eq!(access.gate(), 0b0011_0000);
        assert_eq!(access.effect(0b10), BarrierEffect::Closed);
        assert_eq!(access.effect(0b1_0000), BarrierEffect::Gate);
        assert_eq!(access.effect(0b1), BarrierEffect::Free);
        assert!(access.is_valid());
        assert!(!BarrierAccess(0x0101).is_valid());
        assert!(!BarrierAccess(0x0040).is_valid());
        assert!(!BarrierAccess::NONE.has_effect());
    }
}

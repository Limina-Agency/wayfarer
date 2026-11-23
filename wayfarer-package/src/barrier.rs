//! Barrier nodes in normal routing.
//!
//! A package lists the nodes with a barrier ([`crate::format::Section::BarrierNode`])
//! and stores, per node, a [`BarrierAccess`]: which profiles the barrier
//! closes and which pass it as a gate. The engine applies it at transitions
//! through the node (a barrier closes the ways that end at
//! its node, or every way when all pass through). A SAR request's `barriers`
//! relaxation replaces it for vehicles with its own costs per
//! [`crate::sar::SarBarrierClass`].

/// How a barrier node affects each profile: bits 0-15 the slot bits it
/// closes, bits 16-31 those that pass it as a gate (with a delay). The two
/// sets are disjoint; profiles in neither pass freely. Format 3 stores it as
/// is; format 2 and road graphs store a u16 with eight bits each
/// ([`Self::from_format_2`], [`Self::to_format_2`]).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct BarrierAccess(pub u32);

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
    pub fn new(closed: u16, gate: u16) -> Self {
        Self(u32::from(closed) | (u32::from(gate & !closed) << 16))
    }

    /// Format 2's u16: closed bits 0-7, gate bits 8-15.
    pub fn from_format_2(access: u16) -> Self {
        Self(u32::from(access & 0xff) | (u32::from(access >> 8) << 16))
    }

    /// Format 2's u16 of an access whose bits are format 2's profile bits.
    pub fn to_format_2(self) -> u16 {
        (self.closed() & 0xff) | ((self.gate() & 0xff) << 8)
    }

    pub fn closed(self) -> u16 {
        self.0 as u16
    }

    pub fn gate(self) -> u16 {
        (self.0 >> 16) as u16
    }

    /// Whether the barrier affects some profile.
    pub fn has_effect(self) -> bool {
        self.0 != 0
    }

    /// What the barrier does to the profile with `profile_bit`.
    #[inline]
    pub fn effect(self, profile_bit: u16) -> BarrierEffect {
        if self.closed() & profile_bit != 0 {
            BarrierEffect::Closed
        } else if self.gate() & profile_bit != 0 {
            BarrierEffect::Gate
        } else {
            BarrierEffect::Free
        }
    }

    /// Whether the value is one the format defines for a package with
    /// `slot_count` profile slots: disjoint sets of slot bits.
    pub fn is_valid(self, slot_count: usize) -> bool {
        let slots = ((1u32 << slot_count.min(16)) - 1) as u16;
        self.closed() & self.gate() == 0 && (self.closed() | self.gate()) & !slots == 0
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
        assert!(access.is_valid(6));
        assert!(!BarrierAccess(0x0001_0001).is_valid(6));
        assert!(!BarrierAccess(0x0040).is_valid(6));
        assert!(BarrierAccess(0x0040).is_valid(7));
        assert!(!BarrierAccess::NONE.has_effect());
        // Format 2's u16 round-trips.
        assert_eq!(
            BarrierAccess::from_format_2(0x3006),
            BarrierAccess::new(0b110, 0b11_0000)
        );
        assert_eq!(BarrierAccess::new(0b110, 0b11_0000).to_format_2(), 0x3006);
    }
}

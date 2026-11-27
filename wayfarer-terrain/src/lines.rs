//! The per-post linear-feature mask (u16): which linear features pass through a post's cell.
//!
//! Bits 0-12 come from the base (OSM ways and barriers, marked with an exact supercover so a
//! line is always 4-connected and an 8-connected mover cannot slip across it). Bits 13-15 are
//! added at surface build time from fused layer effects and the team overlay
//! (`docs/design/incident-surface.md` section 5).

pub const MOTORWAY: u16 = 1 << 0;
pub const MAJOR: u16 = 1 << 1;
pub const MINOR: u16 = 1 << 2;
pub const TRACK: u16 = 1 << 3;
pub const PATH: u16 = 1 << 4;
pub const PATH_MOUNTAIN: u16 = 1 << 5;
pub const PATH_DEMANDING: u16 = 1 << 6;
pub const STEPS: u16 = 1 << 7;
pub const RIVER: u16 = 1 << 8;
pub const STREAM: u16 = 1 << 9;
pub const RAIL: u16 = 1 << 10;
pub const FENCE: u16 = 1 << 11;
pub const CLIFF: u16 = 1 << 12;
/// Debris or fallen trees across the cell (from layer features).
pub const OBSTRUCTION: u16 = 1 << 13;
/// A washout or collapse: the cell is cut.
pub const WASHOUT: u16 = 1 << 14;
/// The team closed the cell.
pub const TEAM_BLOCK: u16 = 1 << 15;

/// The way kinds (bits 0-7); a usable way on a cell overrides its cover.
pub const WAYS: u16 = 0xFF;
/// The barriers, in the order of a profile's `barrier_rel`.
pub const BARRIERS: [u16; 5] = [RIVER, STREAM, RAIL, FENCE, CLIFF];
/// Bits the base may carry; the others are reserved for surface effects.
pub const BASE_BITS: u16 = 0x1FFF;

pub const BIT_NAMES: [&str; 16] = [
    "motorway",
    "major",
    "minor",
    "track",
    "path",
    "path_mountain",
    "path_demanding",
    "steps",
    "river",
    "stream",
    "rail",
    "fence",
    "cliff",
    "obstruction",
    "washout",
    "team_block",
];

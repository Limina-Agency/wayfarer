//! Off-road terrain for Wayfarer: the three stages of
//! `docs/design/terrain-layers.md` and `docs/design/incident-surface.md`.
//!
//! 1. **Base** ([`terrain`]): heights, steepness-max, land cover with its confidence bit and
//!    the linear-feature mask per post of the global arc-second lattice ([`lattice`]), in
//!    64 × 64-post blocks, read from a terrain file.
//! 2. **Fusion** ([`layer`], [`fuse`], [`surface`]): terrain layers (model output, hazard
//!    rasters, drone features) and a team overlay fused with the base at an incident epoch, then
//!    costed per profile ([`profile`], [`cost`]) into an incident surface: a file with a
//!    content hash (same hash, same routes), a recipe hash over every input, and deltas.
//! 3. **Query** ([`search`]): 16-connected integer Dijkstra and A* on the surface with
//!    pace by grade from the heights, canonical paths, and route confidence.
//!
//! Everything that changes a result is integer arithmetic with one rounding rule per step,
//! and each block depends only on its own inputs, so the same inputs give the same bytes on
//! every target, thread count and update order. The few floating-point steps (profile tables
//! and ground lengths) use `libm`, which gives the same bits everywhere.
//!
//! Status: experimental. The file formats follow contract drafts v0 and change until they are
//! pinned as v1; the profile values are assumptions until the field trial.
//!
//! ```
//! use wayfarer_terrain::{lattice::{BlockId, Spacing}, terrain::Terrain};
//!
//! // One block of flat meadow at 1".
//! let s = Spacing::new(120).unwrap();
//! let n = wayfarer_terrain::lattice::BLOCK_POSTS;
//! let t = Terrain::new(s, vec![BlockId { bi: 6000, bj: 2500 }], vec![10_000; n], vec![0; n], vec![3; n], vec![1; n], vec![0; n]).unwrap();
//! let file = t.write();
//! assert_eq!(Terrain::read(&file).unwrap().hash(), t.hash());
//! ```

pub mod codec;
pub mod cost;
pub mod fuse;
pub mod lattice;
pub mod layer;
pub mod lines;
pub mod profile;
pub mod search;
pub mod surface;
pub mod terrain;

/// Why a file or an input was refused.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    /// The bytes do not start with the magic of the expected file.
    #[error("not a {0}")]
    Magic(&'static str),
    /// A major version this reader does not know.
    #[error("{what} major version {found}, this reader knows {known}")]
    Version {
        what: &'static str,
        found: u16,
        known: u16,
    },
    /// The bytes end before the structure does.
    #[error("truncated {0}")]
    Truncated(&'static str),
    /// A stored hash does not match the decoded content.
    #[error("{0} hash mismatch")]
    Hash(&'static str),
    /// A value or a combination of values the contract does not allow.
    #[error("{0}")]
    Invalid(String),
}

impl Error {
    pub(crate) fn invalid(msg: impl Into<String>) -> Error {
        Error::Invalid(msg.into())
    }
}

pub type Result<T, E = Error> = std::result::Result<T, E>;

/// Lower-case hex of a hash, for logs and records.
pub fn hex(h: &[u8; 32]) -> String {
    h.iter().map(|b| format!("{b:02x}")).collect()
}

/// Compiles the README's example as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
struct ReadmeDoctests;

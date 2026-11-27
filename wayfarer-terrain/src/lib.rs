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
//! The lattice, the layer format and the byte codings live in the small `wayfarer-layer` crate,
//! so producers can write layers on any device; they are re-exported here as [`lattice`],
//! [`layer`] and [`codec`].
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

pub use wayfarer_layer::{codec, lattice, layer};

pub mod cost;
pub mod fuse;
pub mod lines;
pub mod profile;
pub mod search;
pub mod surface;
pub mod terrain;

pub use wayfarer_layer::{Error, Result, hex};

/// Compiles the README's example as a doctest.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
struct ReadmeDoctests;

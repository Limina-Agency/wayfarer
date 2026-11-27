//! Terrain layer files for Wayfarer (`docs/design/terrain-layers.md`, contract draft v0).
//!
//! A layer is what a producer (a model on a phone or a Mac, a hazard raster, a drone survey)
//! hands to the engine's fusion: values per post of the global arc-second lattice with a
//! per-post confidence, or a list of linear features, plus a manifest that says where it came
//! from and when it holds. This crate holds what a producer needs to write one, and nothing
//! else: the lattice ([`lattice`]), the layer format ([`layer`]) and the byte codings they share
//! ([`codec`]). Its only dependencies are `blake3`, `flate2` and `thiserror`, so it builds for
//! any device. Fusing layers into an incident surface is `wayfarer-terrain`'s job.
//!
//! Every identity is a blake3 hash of decoded values, and the one float step (the quantiser
//! [`layer::quantise_probs`]) is exact in f64, so two devices that write the same layer get
//! the same hash.
//!
//! Status: experimental. The format follows contract draft v0 and changes until it is pinned
//! as v1.
//!
//! ```
//! use wayfarer_layer::lattice::BlockId;
//! use wayfarer_layer::layer::{GridBlock, Kind, Layer, Manifest};
//!
//! // A flood probability of 200/255 on one row of posts, from a model run at 1″.
//! let m = Manifest {
//!     kind: Kind::Hazard, licence: 0, s_lat: 120, s_lon: 120,
//!     observed_at: 1_795_510_800, valid_from: 1_795_510_800,
//!     valid_until: 1_795_510_800 + 86_400, fade_from: 1_795_510_800 + 43_200,
//!     channels: vec![1], model_id: "ai4g-flood".into(), model_version: "1".into(),
//!     weights: [0; 32], dataset_manifest: [0; 32], eval_set: [0; 32], ece_permille: 0xFFFF,
//!     backend: "tract".into(), attribution: String::new(), inputs: vec![], parents: vec![],
//! };
//! let (id, row) = BlockId::of_post(177_440, 71_936);
//! let (slots, width) = m.slots();
//! let mut b = GridBlock::new(id, slots, width);
//! for p in row..row + 64 {
//!     b.set_present(p);
//!     b.conf[p] = 255;
//!     b.vals[0].set(p, 200);
//! }
//! let mut layer = Layer { m, blocks: vec![b], features: vec![] };
//! layer.canonicalise().unwrap();
//! let file = layer.write();
//! assert_eq!(Layer::read(&file).unwrap().content_hash(), layer.content_hash());
//! ```

pub mod codec;
pub mod lattice;
pub mod layer;

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
    /// An [`Error::Invalid`] from a message.
    #[doc(hidden)]
    pub fn invalid(msg: impl Into<String>) -> Error {
        Error::Invalid(msg.into())
    }
}

pub type Result<T, E = Error> = std::result::Result<T, E>;

/// Lower-case hex of a hash, for logs and records.
pub fn hex(h: &[u8; 32]) -> String {
    h.iter().map(|b| format!("{b:02x}")).collect()
}

//! The routing package (`routing.mrt`): one flat, fixed-layout,
//! memory-mappable file holding everything a route, matrix or isochrone
//! request reads.
//!
//! The compiler writes it straight from the in-memory `RoadGraph`
//! (`build.rs`): global dense node and edge indexes, CSR adjacency in spatial
//! (Hilbert) order, per-profile edge costs and durations precomputed with the
//! costing policy for default options, the raw costing inputs every other
//! option set is derived from (`columns.rs`), restrictions, landmarks, edge
//! shapes, and the request sections endpoint correlation and route legs read
//! (`request.rs`). Queries read it in place and decode nothing per request.
//!
//! # Layout
//!
//! A 256-byte header, a section table (offset, length and blake3 hash of
//! every section), then the sections, each aligned to 64 bytes and padded
//! with zeros. The header's *root checksum* is the blake3 hash of the header
//! (with that field zeroed) and the section table; since the table carries
//! every section's hash, the root checksum covers every byte of the file.
//! It doubles as the package's dataset hash.
//!
//! [`RoutingPackage::open`] checks the header, the root checksum over the
//! header and table, and the sections' bounds and structure; it never hashes
//! the sections themselves. [`RoutingPackage::verify`] re-hashes every
//! section (an install-time check for packages shipped over poor links).

pub(crate) mod columns;
pub(crate) mod destination;
pub(crate) mod landmarks;
pub(crate) mod profile_landmarks;
pub(crate) mod request;
pub(crate) mod restrictions;
pub(crate) mod search;
#[cfg(test)]
mod tests;

use std::ops::Deref;
use std::path::Path;
use std::sync::{Mutex, OnceLock};

use serde::{Deserialize, Serialize};

#[cfg(test)]
use crate::types::MobilityProfile;
pub(crate) use search::SearchState;
pub(crate) use wayfarer_package::format::*;
pub use wayfarer_package::format::{
    PackageCounts, PackageQuality, PackageQualityStatus, ROUTING_PACKAGE_FORMAT_VERSION,
    RoutingPackageError, RoutingPackageMetadata, RoutingPackageSection, RoutingPackageVerification,
};

/// How much memory a package keeps for queries. Desktop keeps a search state
/// per core (at least four), six derived cost columns and runs matrix cells
/// on every core; mobile bounds each to the minimum a request needs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RoutePackageMemoryProfile {
    #[default]
    Desktop,
    Mobile,
}

impl RoutePackageMemoryProfile {
    /// Search states kept between requests (each holds per-edge label slots,
    /// about 16 bytes per edge once a bidirectional search has run): one per
    /// matrix thread, so a matrix allocates its states once rather than per
    /// cell or row.
    pub fn pooled_search_states(self) -> usize {
        match self {
            Self::Desktop => rayon::current_num_threads().max(4),
            Self::Mobile => self.matrix_threads().unwrap_or(1),
        }
    }

    /// Derived cost columns kept in the LRU (8 bytes per edge each).
    pub fn derived_column_capacity(self) -> usize {
        match self {
            Self::Desktop => 6,
            Self::Mobile => 1,
        }
    }

    /// Landmark sets of derived metrics kept (`profile_landmarks`; about
    /// 8 bytes per landmark and node each).
    pub fn profile_landmark_capacity(self) -> usize {
        match self {
            Self::Desktop => 2,
            Self::Mobile => 1,
        }
    }

    /// Threads a matrix searches its cells on; `None` uses the global pool.
    pub fn matrix_threads(self) -> Option<usize> {
        match self {
            Self::Desktop => None,
            Self::Mobile => Some(2),
        }
    }
}

/// A routing package opened for queries: the validated file plus the
/// query-time state the engine keeps per package (pooled search states,
/// derived cost columns, complex-restriction automata, the matrix thread
/// pool). Cheap to share behind an `Arc`; derefs to the file.
pub struct RoutingPackage {
    file: PackageFile,
    memory: RoutePackageMemoryProfile,
    state_pool: Mutex<Vec<SearchState>>,
    derived_columns: columns::DerivedColumnCache,
    /// Landmarks of derived metrics (Emergency, Tracked), built on first use.
    profile_landmarks: columns::ProfileLandmarkCache,
    /// Per profile slot, built on first use (see `restrictions`).
    complex_automata: [OnceLock<restrictions::ComplexAutomata>; PROFILE_SLOTS],
    /// The matrix thread pool of a bounded memory profile, built on first use.
    matrix_pool: OnceLock<Option<rayon::ThreadPool>>,
    /// The package's stored costs are of an older costing model: every
    /// request is costed from its stored costing inputs with this build's
    /// model (derived columns), never from the stored columns.
    recost: bool,
    /// Per profile slot, grouped on first use (see `destination`).
    destination_pockets: [OnceLock<destination::DestinationPockets>; PROFILE_SLOTS],
}

impl std::fmt::Debug for RoutingPackage {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RoutingPackage")
            .field("file", &self.file)
            .field("memory", &self.memory)
            .finish()
    }
}

impl RoutingPackage {
    /// The package file underneath: its sections and layout. Internal (for
    /// the workspace's tests and harnesses); tools reading the format open it
    /// with `wayfarer_package` directly.
    #[doc(hidden)]
    pub fn file(&self) -> &PackageFile {
        &self.file
    }

    /// The package format version in the file's header.
    pub fn format_version(&self) -> u32 {
        self.file.format_version()
    }

    /// Node, edge and other element counts.
    pub fn counts(&self) -> PackageCounts {
        self.file.counts()
    }

    /// The package's root checksum, in hex: identifies its content.
    pub fn dataset_hash(&self) -> &str {
        self.file.dataset_hash()
    }

    /// What the compiler recorded: graph id, country, costing model, quality,
    /// source extract.
    pub fn metadata(&self) -> &RoutingPackageMetadata {
        self.file.metadata()
    }

    pub fn graph_id(&self) -> &str {
        self.file.graph_id()
    }

    /// Whether the package stores costs for `profile`.
    pub fn supports_profile(&self, profile: crate::types::MobilityProfile) -> bool {
        self.file.supports_profile(profile)
    }

    /// The package's size in bytes.
    pub fn byte_len(&self) -> usize {
        self.file.byte_len()
    }

    /// Every section's name and size, for size reports.
    pub fn section_sizes(&self) -> Vec<(&'static str, usize)> {
        self.file.section_sizes()
    }

    /// Ready to route: nodes, edges, and some ground-mode edge.
    pub fn is_ready(&self) -> bool {
        self.file.is_ready()
    }
    /// The SAR relaxations requests may use on this package, with the
    /// least factors its landmark metric allows
    /// (`RoutingPackageMetadata::sar`).
    pub fn sar_capability(&self) -> crate::sar::RouteSarCapability {
        crate::sar::RouteSarCapability::with_package_data(self.file.metadata().sar)
    }

    /// Opens a package with the desktop memory profile.
    pub fn open(path: &Path) -> Result<Self, RoutingPackageError> {
        Self::open_with_profile(path, RoutePackageMemoryProfile::Desktop)
    }

    /// Opens a package (see [`PackageFile::open`] for what is checked).
    pub fn open_with_profile(
        path: &Path,
        memory: RoutePackageMemoryProfile,
    ) -> Result<Self, RoutingPackageError> {
        Self::with_runtime(PackageFile::open(path)?, memory)
    }

    /// Opens a package held in memory (desktop memory profile).
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, RoutingPackageError> {
        Self::with_runtime(
            PackageFile::from_bytes(bytes)?,
            RoutePackageMemoryProfile::Desktop,
        )
    }

    /// Checks a package file end to end (see [`PackageFile::verify`]).
    pub fn verify(path: &Path) -> Result<RoutingPackageVerification, RoutingPackageError> {
        PackageFile::verify(path)
    }

    /// Wraps an opened file with query-time state. A package of an older
    /// costing model is re-costed: its stored costing inputs (the same ones
    /// derived columns read) are costed with this build's model, so it routes
    /// as a recompiled package would, at the price of deriving each column
    /// once. A newer model is `TooNew`.
    fn with_runtime(
        file: PackageFile,
        memory: RoutePackageMemoryProfile,
    ) -> Result<Self, RoutingPackageError> {
        let model = file.metadata().costing_model_version;
        if model > wayfarer_costing::COSTING_MODEL_VERSION {
            return Err(RoutingPackageError::TooNew(format!(
                "costing model {model}, this build costs with model {}",
                wayfarer_costing::COSTING_MODEL_VERSION
            )));
        }
        Ok(Self {
            file,
            memory,
            state_pool: Mutex::new(Vec::new()),
            derived_columns: columns::DerivedColumnCache::with_capacity(
                memory.derived_column_capacity(),
            ),
            profile_landmarks: columns::ProfileLandmarkCache::with_capacity(
                memory.profile_landmark_capacity(),
            ),
            complex_automata: Default::default(),
            matrix_pool: OnceLock::new(),
            recost: model < wayfarer_costing::COSTING_MODEL_VERSION,
            destination_pockets: Default::default(),
        })
    }

    /// Whether the package's stored costs are of an older costing model, so
    /// that requests are costed from its inputs instead (see `with_runtime`).
    pub fn is_recosted(&self) -> bool {
        self.recost
    }

    pub fn memory_profile(&self) -> RoutePackageMemoryProfile {
        self.memory
    }

    /// Faults the mapped file in, pools one search state with its route
    /// buffers allocated and derives the cost column a default drive starts
    /// on, as a service does in the background after opening a source.
    /// Returns the bytes touched.
    pub fn prefetch(&self) -> usize {
        let bytes = self.file.prefetch_pages();
        let mut state = self.take_state();
        state.preallocate_route_buffers(self.counts().edge_count, self.counts().node_count);
        self.return_state(state);
        // The strict column of a default drive (see `destination`), so that
        // it is not derived on the first request.
        let options = crate::types::RouteOptions::default();
        let wheeled = crate::types::MobilityProfile::Wheeled;
        if self.confines_destination_only(&options, wheeled) {
            self.request_column_sar(wheeled, &options.costing, false, None);
        }
        bytes
    }

    /// The sections, with the complex-restriction automata the searches build.
    pub(crate) fn view(&self) -> SearchGraph<'_> {
        SearchGraph {
            sections: self.file.view(),
            complex_automata: &self.complex_automata,
            transitions: None,
        }
    }

    pub(crate) fn take_state(&self) -> SearchState {
        self.state_pool
            .lock()
            .ok()
            .and_then(|mut pool| pool.pop())
            .unwrap_or_else(|| SearchState::new(self.counts().edge_count))
    }

    pub(crate) fn return_state(&self, mut state: SearchState) {
        state.reset();
        if self.memory == RoutePackageMemoryProfile::Mobile {
            state.trim();
        }
        if let Ok(mut pool) = self.state_pool.lock()
            && pool.len() < self.memory.pooled_search_states()
        {
            pool.push(state);
        }
    }

    /// Runs `work` on the matrix thread pool of the memory profile (the
    /// global pool on desktop).
    pub(crate) fn in_matrix_pool<T: Send>(&self, work: impl FnOnce() -> T + Send) -> T {
        let pool = self.matrix_pool.get_or_init(|| {
            self.memory.matrix_threads().and_then(|threads| {
                rayon::ThreadPoolBuilder::new()
                    .num_threads(threads)
                    .thread_name(|index| format!("wayfarer-matrix-{index}"))
                    .build()
                    .ok()
            })
        });
        match pool {
            Some(pool) => pool.install(work),
            None => work(),
        }
    }
}

/// The package's sections plus the complex-restriction automata, built per
/// profile on first use. Derefs to the sections.
#[derive(Clone, Copy)]
pub(crate) struct SearchGraph<'g> {
    sections: Sections<'g>,
    complex_automata: &'g [OnceLock<restrictions::ComplexAutomata>; PROFILE_SLOTS],
    /// A request's transition costs (barriers, a SAR request's R2 entry and
    /// R4 barriers), which [`SearchGraph::turn_cost`] adds; `None` outside a
    /// request (landmarks, map matching).
    pub(crate) transitions: Option<crate::transitions::Transitions>,
}

impl<'g> Deref for SearchGraph<'g> {
    type Target = Sections<'g>;

    fn deref(&self) -> &Sections<'g> {
        &self.sections
    }
}

impl<'g> SearchGraph<'g> {
    /// This view with a request's transition costs.
    pub(crate) fn with_transitions(self, transitions: crate::transitions::Transitions) -> Self {
        Self {
            transitions: Some(transitions),
            ..self
        }
    }

    pub(crate) fn complex_automata(&self, profile_bit: u8) -> &'g restrictions::ComplexAutomata {
        let slot = (profile_bit.trailing_zeros() as usize).min(PROFILE_SLOTS - 1);
        let automata: &'g [OnceLock<restrictions::ComplexAutomata>; PROFILE_SLOTS] =
            self.complex_automata;
        automata[slot].get_or_init(|| restrictions::ComplexAutomata::new(self, profile_bit))
    }
}

#[cfg(test)]
impl RoutingPackage {
    /// Legacy correlator reach metric: distinct edges whose mode mask admits
    /// `profile`, reachable forward from the end of the edge (or backward from
    /// its start), capped at `max_edge_count`. The count is min(reachable,
    /// cap), so traversal order does not matter. The correlator counts with
    /// `request::ReachCounter`; this is its reference in tests.
    #[cfg(test)]
    pub(crate) fn reach_edge_count(
        &self,
        edge: u32,
        profile: MobilityProfile,
        max_edge_count: usize,
        outgoing: bool,
    ) -> Option<usize> {
        if max_edge_count == 0 {
            return Some(0);
        }
        let g = self.view();
        let bit = profile_bit(profile);
        let start = if outgoing {
            g.edge_target[edge as usize]
        } else {
            g.edge_source(edge as usize)
        };
        // At most `max_edge_count` edges are recorded and each recorded edge
        // contributes at most one new node, so small linear buffers suffice.
        let mut seen_edges = Vec::with_capacity(max_edge_count.min(256));
        let mut nodes = Vec::with_capacity(max_edge_count.min(256) + 1);
        nodes.push(start);
        let mut cursor = 0;
        while cursor < nodes.len() {
            let node = nodes[cursor] as usize;
            cursor += 1;
            let candidates: Vec<u32> = if outgoing {
                (g.first_out[node]..g.first_out[node + 1]).collect()
            } else {
                g.in_edges(node).map(|(edge, _)| edge).collect()
            };
            for candidate in candidates {
                if g.edge_profile_access[candidate as usize] & bit == 0
                    || seen_edges.contains(&candidate)
                {
                    continue;
                }
                seen_edges.push(candidate);
                if seen_edges.len() >= max_edge_count {
                    return Some(seen_edges.len());
                }
                let next = if outgoing {
                    g.edge_target[candidate as usize]
                } else {
                    g.edge_source(candidate as usize)
                };
                if !nodes.contains(&next) {
                    nodes.push(next);
                }
            }
        }
        Some(seen_edges.len())
    }
}

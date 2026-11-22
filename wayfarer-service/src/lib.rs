//! Source management and route workflows over the offline routing engine. A
//! host resolves a routing package to a [`RoutingSource`] (a stable id, a
//! revision and a local path) and decides what to do with the computed route;
//! the service verifies and opens packages, keeps a bounded cache of them,
//! reports readiness and computes routes and waypoint workflows.
//!
//! ```no_run
//! use wayfarer::{GeoPoint, MobilityProfile, LegRequest};
//! use wayfarer_service::{RoutingService, RoutingSource, RoutingSourceId};
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let service = RoutingService::default(); // caches up to 4 opened sources
//! let source = RoutingSource::new(RoutingSourceId::new("monaco")?, "2026-10-01", "routing.mrt")?;
//!
//! // Once, when the package is installed: checks every section's checksum.
//! service.verify_source(&source)?;
//!
//! // Opens (or reuses) the package and says whether it can route.
//! let readiness = service.inspect_source(&source);
//! assert!(readiness.is_ready(), "{readiness:?}");
//!
//! let computation = service.route_leg(
//!     &source,
//!     LegRequest::new(
//!         GeoPoint::new(43.7397, 7.4275),
//!         GeoPoint::new(43.7290, 7.4160),
//!         MobilityProfile::Wheeled,
//!     ),
//! )?;
//! println!(
//!     "{:.0} m, {:.0} s (cache hit: {})",
//!     computation.route.distance_m, computation.route.duration_s, computation.cache_hit
//! );
//! # Ok(())
//! # }
//! ```
//!
//! # Stability
//!
//! Version 0.x: the API may change between minor releases. Package format and
//! costing model compatibility are the engine's; an outdated package is
//! reported as [`SourceProblem::PackageOutdated`] in the readiness result.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Condvar, Mutex, Weak};

use serde::Serialize;
use thiserror::Error;
use wayfarer::PackageQualityStatus;
use wayfarer::internals::Correlator;
use wayfarer::{
    CancelToken, CorrelationPolicy, GeoPoint, LegRequest, LegResult, MobilityProfile,
    RouteEndpointOptions, RouteFailure, RoutePackageMemoryProfile, RoutingEngine, RoutingPackage,
    RoutingPackageError, RoutingPackageVerification,
};

pub mod geojson;
pub mod include;
pub mod overlay_set;
mod source_cache;
mod workflow;

use source_cache::SourceCache;

pub use include::{RouteResultPart, RouteSegmentSummary};

pub use workflow::{
    MAX_WORKFLOW_MOBILITY_ADJUSTMENTS, MAX_WORKFLOW_OVERLAY_SETS, RouteWorkflowError,
    RouteWorkflowMobilityInput, RouteWorkflowMobilityProvenance, RouteWorkflowOverlayProvenance,
    RouteWorkflowRequest, RouteWorkflowResult, RouteWorkflowSourceContext, compile_route_workflow,
};

const DEFAULT_SOURCE_CACHE_CAPACITY: usize = 4;
const MAX_SOURCE_ID_BYTES: usize = 128;
const MAX_SOURCE_REVISION_BYTES: usize = 256;
use wayfarer::limits::{
    MAX_REQUEST_CANDIDATES, MAX_REQUEST_DYNAMIC_CONNECTORS, MAX_REQUEST_DYNAMIC_GEOMETRY_POINTS,
    MAX_REQUEST_EDGE_CONSTRAINTS,
};

/// Stable identity used by the host to refer to a route source.
///
/// It is intentionally unrelated to package ids. An application embedding the
/// engine can use any stable namespace it owns.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(transparent)]
pub struct RoutingSourceId(String);

impl RoutingSourceId {
    pub fn new(value: impl Into<String>) -> Result<Self, SourceDescriptorError> {
        let value = value.into();
        if value.trim().is_empty() {
            return Err(SourceDescriptorError::EmptyId);
        }
        if value.len() > MAX_SOURCE_ID_BYTES {
            return Err(SourceDescriptorError::IdTooLong);
        }
        if value.chars().any(char::is_control) {
            return Err(SourceDescriptorError::InvalidId);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// Rust-internal descriptor for one immutable revision of a routing package.
///
/// The path is deliberately private and this type is not serializable. SDK
/// records expose only source identity and revision; platform hosts resolve
/// their own storage paths before constructing this descriptor.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RoutingSource {
    id: RoutingSourceId,
    revision: String,
    archive_path: PathBuf,
}

impl RoutingSource {
    pub fn new(
        id: RoutingSourceId,
        revision: impl Into<String>,
        archive_path: impl Into<PathBuf>,
    ) -> Result<Self, SourceDescriptorError> {
        let revision = revision.into();
        if revision.trim().is_empty() {
            return Err(SourceDescriptorError::EmptyRevision);
        }
        if revision.len() > MAX_SOURCE_REVISION_BYTES {
            return Err(SourceDescriptorError::RevisionTooLong);
        }
        if revision.chars().any(char::is_control) {
            return Err(SourceDescriptorError::InvalidRevision);
        }
        let archive_path = archive_path.into();
        if archive_path.as_os_str().is_empty() {
            return Err(SourceDescriptorError::EmptyArchivePath);
        }
        Ok(Self {
            id,
            revision,
            archive_path,
        })
    }

    pub fn id(&self) -> &RoutingSourceId {
        &self.id
    }

    pub fn revision(&self) -> &str {
        &self.revision
    }

    fn archive_path(&self) -> &Path {
        &self.archive_path
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum SourceDescriptorError {
    #[error("routing source id must not be empty")]
    EmptyId,
    #[error("routing source id is too long")]
    IdTooLong,
    #[error("routing source id contains control characters")]
    InvalidId,
    #[error("routing source revision must not be empty")]
    EmptyRevision,
    #[error("routing source revision is too long")]
    RevisionTooLong,
    #[error("routing source revision contains control characters")]
    InvalidRevision,
    #[error("routing source archive path must not be empty")]
    EmptyArchivePath,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum SourceReadinessStatus {
    Ready,
    Missing,
    Invalid,
    Stale,
    NotRoutable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum SourceProblem {
    ArchiveMissing,
    ArchiveInvalid,
    ArchiveStale,
    GraphNotRoutable,
    ServiceUnavailable,
    /// The file is a routing package of an older format: recompile the
    /// package.
    PackageOutdated,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum SourceQuality {
    Ready,
    Warning,
    Stale,
}

/// Portable package readiness. It intentionally contains no filesystem path,
/// package state, memory-map implementation, or platform runtime type.
/// `archive_hash` is the package's root checksum; `tile_count` and
/// `cross_tile_edge_count` described the former tile archives and are zero.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct SourceReadiness {
    pub source_id: RoutingSourceId,
    pub revision: String,
    pub status: SourceReadinessStatus,
    pub graph_id: Option<String>,
    pub archive_hash: Option<String>,
    pub node_count: u64,
    pub directed_edge_count: u64,
    pub tile_count: u64,
    pub cross_tile_edge_count: u64,
    pub quality: Option<SourceQuality>,
    pub quality_reasons: Vec<String>,
    pub cache_hit: bool,
    pub problem: Option<SourceProblem>,
    /// The SAR relaxations the package supports (`RouteSarCapability`).
    /// `None` when the package did not open.
    pub sar_capability: Option<wayfarer::RouteSarCapability>,
}

impl SourceReadiness {
    pub fn is_ready(&self) -> bool {
        self.status == SourceReadinessStatus::Ready
    }

    fn failed(
        source: &RoutingSource,
        status: SourceReadinessStatus,
        problem: SourceProblem,
    ) -> Self {
        Self {
            source_id: source.id.clone(),
            revision: source.revision.clone(),
            status,
            graph_id: None,
            archive_hash: None,
            node_count: 0,
            directed_edge_count: 0,
            tile_count: 0,
            cross_tile_edge_count: 0,
            quality: None,
            quality_reasons: Vec::new(),
            cache_hit: false,
            problem: Some(problem),
            sar_capability: None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct EndpointReadiness {
    pub candidate_count: u64,
    pub rejected_candidate_count: u64,
    pub best_candidate_score_basis_points: Option<u32>,
    pub best_component_id: Option<u32>,
    pub best_component_size: Option<u64>,
    pub best_outbound_reach_edge_count: Option<u64>,
    pub best_inbound_reach_edge_count: Option<u64>,
    pub best_layer: Option<i64>,
    pub best_level: Option<String>,
    pub best_road_group_key: Option<String>,
    pub same_road_candidate_count: Option<u64>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct CorridorReadiness {
    pub source: SourceReadiness,
    pub origin: EndpointReadiness,
    pub destination: EndpointReadiness,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CorridorQuery {
    pub origin: GeoPoint,
    pub destination: GeoPoint,
    pub profile: MobilityProfile,
    pub correlation_policy: CorrelationPolicy,
    pub origin_options: RouteEndpointOptions,
    pub destination_options: RouteEndpointOptions,
}

/// Rust-domain result returned by the kernel. The SDK must deliberately map
/// this rich engine result to its fixed-width, bounded binding contract rather
/// than exporting engine internals accidentally.
#[derive(Debug, Clone, PartialEq)]
pub struct RouteComputation {
    pub source_id: RoutingSourceId,
    pub revision: String,
    pub cache_hit: bool,
    pub route: LegResult,
}

#[derive(Debug, Error)]
pub enum RoutingServiceError {
    #[error("routing request exceeds service bounds: {0}")]
    RequestTooLarge(&'static str),
    #[error("routing source is not ready: {0:?}")]
    SourceNotReady(Box<SourceReadiness>),
    #[error("routing computation failed: {}", .0.error)]
    RouteFailed(Box<RouteFailure>),
    #[error(transparent)]
    WorkflowFailed(#[from] RouteWorkflowError),
    #[error("routing package failed verification: {0}")]
    PackageVerificationFailed(String),
}

#[derive(Debug)]
pub struct RoutingService {
    memory_profile: RoutePackageMemoryProfile,
    cache: Mutex<SourceCache<RoutingSourceId, Arc<CachedSource>>>,
    admissions: Mutex<HashMap<RoutingSourceId, Weak<Mutex<()>>>>,
    handles: Mutex<HashMap<RoutingSourceId, Vec<Weak<CachedSource>>>>,
}

impl Default for RoutingService {
    fn default() -> Self {
        Self::new(
            DEFAULT_SOURCE_CACHE_CAPACITY,
            RoutePackageMemoryProfile::Desktop,
        )
    }
}

impl RoutingService {
    /// A service keeping at most `capacity` opened sources, each with the
    /// memory bounds of `memory_profile`.
    pub fn new(capacity: usize, memory_profile: RoutePackageMemoryProfile) -> Self {
        Self {
            memory_profile,
            cache: Mutex::new(SourceCache::new(capacity)),
            admissions: Mutex::new(HashMap::new()),
            handles: Mutex::new(HashMap::new()),
        }
    }

    /// Checks a source's package file end to end (every section against its
    /// checksum, padding, length and structure): the install-time check for a
    /// package that came over a network. It reads the whole file once and
    /// does not touch the cache.
    pub fn verify_source(
        &self,
        source: &RoutingSource,
    ) -> Result<RoutingPackageVerification, RoutingServiceError> {
        RoutingPackage::verify(source.archive_path())
            .map_err(|error| RoutingServiceError::PackageVerificationFailed(error.to_string()))
    }

    /// Opens and validates a source, retaining a bounded archive handle on
    /// success. Reusing the same descriptor is a cache hit; changing revision
    /// or archive location atomically replaces the old handle for that id.
    /// Computations still running on the old handle are cancelled: they stop
    /// within one cancellation check and fail with `RouteError::Cancelled`
    /// (an [`Self::execute`] callback runs to its end). Call
    /// [`Self::invalidate`] first when the old archive's storage is to be
    /// replaced in place.
    pub fn inspect_source(&self, source: &RoutingSource) -> SourceReadiness {
        match self.load(source) {
            Ok(loaded) => loaded.readiness.clone(),
            Err(readiness) => *readiness,
        }
    }

    /// Runs advanced Rust-only orchestration against the cached source while
    /// retaining its invalidation lease for the entire callback.
    ///
    /// The engine and package are borrowed, so neither can escape and keep a
    /// package mmap alive after this call returns.
    pub fn execute<T>(
        &self,
        source: &RoutingSource,
        operation: impl FnOnce(&RoutingEngine, &RoutingPackage, &SourceReadiness) -> T,
    ) -> Result<T, Box<SourceReadiness>> {
        let loaded = self.load(source)?;
        let engine = loaded.engine();
        Ok(operation(
            &engine,
            loaded.owner.package.as_ref(),
            &loaded.readiness,
        ))
    }

    /// Drops a source and waits until nothing reads its archive any more, so
    /// its storage can be replaced in place. Computations on it are cancelled
    /// first (as on a revision swap), so the wait is bounded by one
    /// cancellation check, plus any [`Self::execute`] callback still running.
    pub fn invalidate(&self, source_id: &RoutingSourceId) {
        let admission = self.admission(source_id);
        let _guard = admission
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        self.cache
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .pop(source_id);
        let handles = self
            .handles
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .remove(source_id)
            .unwrap_or_default();
        for handle in handles.into_iter().filter_map(|handle| handle.upgrade()) {
            handle.cancel.cancel();
            handle.close_and_wait();
        }
    }

    pub fn inspect_corridor(
        &self,
        source: &RoutingSource,
        query: CorridorQuery,
    ) -> CorridorReadiness {
        let loaded = match self.load(source) {
            Ok(loaded) => loaded,
            Err(readiness) => {
                return CorridorReadiness {
                    source: *readiness,
                    origin: empty_endpoint_readiness(),
                    destination: empty_endpoint_readiness(),
                };
            }
        };
        let engine = loaded.engine();
        let access = engine.route_graph_view();
        let correlator = Correlator::new(access);
        let origin_search = correlator.search(
            query.origin,
            query.profile,
            &query.correlation_policy,
            &query.origin_options,
        );
        let destination_search = correlator.search(
            query.destination,
            query.profile,
            &query.correlation_policy,
            &query.destination_options,
        );
        let origin = endpoint_readiness(access, &origin_search);
        let destination = endpoint_readiness(access, &destination_search);
        let mut readiness = loaded.readiness.clone();

        let missing_endpoint = origin.candidate_count == 0 || destination.candidate_count == 0;
        let disconnected = origin.best_component_id.is_some()
            && destination.best_component_id.is_some()
            && origin.best_component_id != destination.best_component_id;
        if missing_endpoint || disconnected {
            readiness.status = SourceReadinessStatus::NotRoutable;
            readiness.problem = Some(SourceProblem::GraphNotRoutable);
        }

        CorridorReadiness {
            source: readiness,
            origin,
            destination,
        }
    }

    /// Routes one leg between two points on the source
    /// ([`wayfarer::RoutingEngine::route_leg`]).
    pub fn route_leg(
        &self,
        source: &RoutingSource,
        mut request: LegRequest,
    ) -> Result<RouteComputation, RoutingServiceError> {
        validate_request_bounds(&request)?;
        let loaded = self
            .load(source)
            .map_err(RoutingServiceError::SourceNotReady)?;
        loaded.cancel_with_source(&mut request.options);
        let cache_hit = loaded.readiness.cache_hit;
        let route = loaded
            .engine()
            .route_leg(request)
            .map_err(Box::new)
            .map_err(RoutingServiceError::RouteFailed)?;
        Ok(RouteComputation {
            source_id: source.id.clone(),
            revision: source.revision.clone(),
            cache_hit,
            route,
        })
    }

    /// Routes a request through two or more locations on the source
    /// ([`wayfarer::RoutingEngine::route`]), with its overlay sets and
    /// mobility input folded in. Resolving and loading those artifacts
    /// remains the host's job.
    pub fn route(
        &self,
        source: &RoutingSource,
        mut request: RouteWorkflowRequest,
    ) -> Result<RouteWorkflowResult, RoutingServiceError> {
        let loaded = self
            .load(source)
            .map_err(RoutingServiceError::SourceNotReady)?;
        loaded.cancel_with_source(&mut request.route.options);
        compile_route_workflow(
            &loaded.engine(),
            RouteWorkflowSourceContext {
                source_id: source.id.clone(),
                revision: source.revision.clone(),
                cache_hit: loaded.readiness.cache_hit,
            },
            request,
        )
        .map_err(Into::into)
    }

    fn load(&self, source: &RoutingSource) -> Result<LoadedSource, Box<SourceReadiness>> {
        // Admission is per source: replacement and invalidation serialize for
        // this id without holding the global LRU lock during archive I/O.
        let admission = self.admission(source.id());
        let _guard = admission.lock().map_err(|_| unavailable(source))?;
        let cached = self
            .cache
            .lock()
            .map_err(|_| unavailable(source))?
            .get(source.id())
            .cloned();
        if let Some(cached) = cached {
            if cached.descriptor == *source {
                return cached.lease(true).map_err(|_| unavailable(source));
            }
            self.cache
                .lock()
                .map_err(|_| unavailable(source))?
                .pop(source.id());
            // Closed to new leases and its computations cancelled, not
            // drained: waiting here, under the admission lock, would hold
            // every request for this id behind the old revision's in-flight
            // work. The new revision opens its own file; `invalidate` still
            // drains the old handle (it stays in the handle registry) before
            // storage is replaced.
            cached.begin_close();
            cached.cancel.cancel();
        }

        // Opening checks the header, the root checksum and every section's
        // structure, never hashing the whole file (see `verify_source`).
        let package = RoutingPackage::open_with_profile(source.archive_path(), self.memory_profile)
            .map_err(|error| Box::new(readiness_from_package_error(source, error)))?;
        let package = Arc::new(package);
        let metadata = package.metadata();
        let quality = source_quality(metadata.quality.status);
        let mut readiness = SourceReadiness {
            source_id: source.id.clone(),
            revision: source.revision.clone(),
            status: SourceReadinessStatus::Ready,
            graph_id: Some(package.graph_id().to_string()),
            archive_hash: Some(package.dataset_hash().to_string()),
            node_count: metadata.node_count as u64,
            directed_edge_count: metadata.directed_edge_count as u64,
            tile_count: 0,
            cross_tile_edge_count: 0,
            quality: Some(quality),
            quality_reasons: metadata.quality.reasons.clone(),
            cache_hit: false,
            problem: None,
            sar_capability: Some(package.sar_capability()),
        };
        if quality == SourceQuality::Stale {
            readiness.status = SourceReadinessStatus::Stale;
            readiness.problem = Some(SourceProblem::ArchiveStale);
            return Err(Box::new(readiness));
        }
        if package.is_recosted() {
            // Still exact, but every request derives its cost column.
            readiness.quality = Some(SourceQuality::Warning);
            readiness.quality_reasons.push(format!(
                "package costs were baked with costing model {}; requests are re-costed with \
                 model {} (recompile the package to restore its stored columns' speed)",
                metadata.costing_model_version,
                wayfarer::COSTING_MODEL_VERSION
            ));
        }
        if !package.is_ready() {
            readiness.status = SourceReadinessStatus::NotRoutable;
            readiness.problem = Some(SourceProblem::GraphNotRoutable);
            return Err(Box::new(readiness));
        }
        // Fault the package in and pool a search state off the request path,
        // so the first route is not several times slower than the next ones.
        {
            let package = package.clone();
            let _ = std::thread::Builder::new()
                .name("routing-package-prefetch".into())
                .spawn(move || {
                    package.prefetch();
                });
        }
        let cached = Arc::new(CachedSource {
            descriptor: source.clone(),
            package,
            readiness: readiness.clone(),
            leases: Mutex::new(LeaseState::default()),
            lease_drained: Condvar::new(),
            cancel: CancelToken::new(),
        });
        self.register_handle(source.id(), &cached);
        let evicted = self
            .cache
            .lock()
            .map_err(|_| unavailable(source))?
            .push(source.id.clone(), cached.clone());
        // An evicted package may finish an in-flight computation. It is closed
        // to new leases immediately; invalidation finds it through the weak
        // handle registry and drains it before storage replacement.
        if let Some((_id, evicted)) = evicted
            && !Arc::ptr_eq(&evicted, &cached)
        {
            evicted.begin_close();
        }
        cached.lease(false).map_err(|_| unavailable(source))
    }

    fn admission(&self, source_id: &RoutingSourceId) -> Arc<Mutex<()>> {
        let mut admissions = self
            .admissions
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        admissions.retain(|_, admission| admission.strong_count() != 0);
        if let Some(admission) = admissions.get(source_id).and_then(Weak::upgrade) {
            return admission;
        }
        let admission = Arc::new(Mutex::new(()));
        admissions.insert(source_id.clone(), Arc::downgrade(&admission));
        admission
    }

    fn register_handle(&self, source_id: &RoutingSourceId, handle: &Arc<CachedSource>) {
        let mut handles = self
            .handles
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        handles.retain(|_, source_handles| {
            source_handles.retain(|source_handle| source_handle.strong_count() != 0);
            !source_handles.is_empty()
        });
        handles
            .entry(source_id.clone())
            .or_default()
            .push(Arc::downgrade(handle));
    }
}

fn validate_request_bounds(request: &LegRequest) -> Result<(), RoutingServiceError> {
    if request.options.candidate_limit > MAX_REQUEST_CANDIDATES {
        return Err(RoutingServiceError::RequestTooLarge("candidate limit"));
    }
    let edge_constraint_count = request.options.blocked_edge_ids.len()
        + request.options.blocked_edge_ranges.len()
        + request.options.avoided_edge_ids.len()
        + request.options.avoided_edge_ranges.len()
        + request.options.warning_edge_ids.len()
        + request.options.warning_edge_ranges.len()
        + request.options.edge_cost_multipliers.len()
        + request.options.edge_cost_multiplier_ranges.len();
    if edge_constraint_count > MAX_REQUEST_EDGE_CONSTRAINTS {
        return Err(RoutingServiceError::RequestTooLarge("edge constraints"));
    }
    if let Some(overlay) = request.options.dynamic_overlay.as_ref() {
        if overlay.virtual_edges.len() > MAX_REQUEST_DYNAMIC_CONNECTORS {
            return Err(RoutingServiceError::RequestTooLarge("dynamic connectors"));
        }
        let geometry_points = overlay
            .virtual_edges
            .iter()
            .map(|edge| edge.geometry.len())
            .sum::<usize>();
        if geometry_points > MAX_REQUEST_DYNAMIC_GEOMETRY_POINTS {
            return Err(RoutingServiceError::RequestTooLarge(
                "dynamic connector geometry",
            ));
        }
    }
    Ok(())
}

#[derive(Debug)]
struct LoadedSource {
    owner: Arc<CachedSource>,
    readiness: SourceReadiness,
}

impl LoadedSource {
    fn engine(&self) -> RoutingEngine {
        RoutingEngine::from_package(self.owner.package.clone())
    }

    /// Adds the source's token to `options`: superseding or invalidating the
    /// source cancels the computation.
    fn cancel_with_source(&self, options: &mut wayfarer::RouteOptions) {
        options.cancellation =
            std::mem::take(&mut options.cancellation).with_token(self.owner.cancel.clone());
    }
}

impl Drop for LoadedSource {
    fn drop(&mut self) {
        self.owner.release();
    }
}

#[derive(Debug)]
struct CachedSource {
    descriptor: RoutingSource,
    package: Arc<RoutingPackage>,
    readiness: SourceReadiness,
    leases: Mutex<LeaseState>,
    lease_drained: Condvar,
    /// Cancelled when the source is superseded or invalidated.
    cancel: CancelToken,
}

#[derive(Debug, Default)]
struct LeaseState {
    active: usize,
    closing: bool,
}

impl CachedSource {
    fn lease(self: &Arc<Self>, cache_hit: bool) -> Result<LoadedSource, ()> {
        let mut leases = self.leases.lock().map_err(|_| ())?;
        if leases.closing {
            return Err(());
        }
        leases.active += 1;
        drop(leases);
        let mut readiness = self.readiness.clone();
        readiness.cache_hit = cache_hit;
        Ok(LoadedSource {
            owner: self.clone(),
            readiness,
        })
    }

    fn release(&self) {
        let mut leases = self
            .leases
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        leases.active = leases.active.saturating_sub(1);
        if leases.active == 0 {
            self.lease_drained.notify_all();
        }
    }

    fn close_and_wait(&self) {
        let mut leases = self
            .leases
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        leases.closing = true;
        while leases.active != 0 {
            leases = self
                .lease_drained
                .wait(leases)
                .unwrap_or_else(|poisoned| poisoned.into_inner());
        }
    }

    fn begin_close(&self) {
        self.leases
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .closing = true;
    }
}

fn source_quality(status: PackageQualityStatus) -> SourceQuality {
    match status {
        PackageQualityStatus::Ready => SourceQuality::Ready,
        PackageQualityStatus::Warning => SourceQuality::Warning,
        PackageQualityStatus::Stale => SourceQuality::Stale,
    }
}

fn readiness_from_package_error(
    source: &RoutingSource,
    error: RoutingPackageError,
) -> SourceReadiness {
    if error.is_missing() {
        return SourceReadiness::failed(
            source,
            SourceReadinessStatus::Missing,
            SourceProblem::ArchiveMissing,
        );
    }
    match error {
        RoutingPackageError::Outdated(_) => SourceReadiness::failed(
            source,
            SourceReadinessStatus::Stale,
            SourceProblem::PackageOutdated,
        ),
        _ => SourceReadiness::failed(
            source,
            SourceReadinessStatus::Invalid,
            SourceProblem::ArchiveInvalid,
        ),
    }
}

fn unavailable(source: &RoutingSource) -> Box<SourceReadiness> {
    Box::new(SourceReadiness::failed(
        source,
        SourceReadinessStatus::Invalid,
        SourceProblem::ServiceUnavailable,
    ))
}

fn endpoint_readiness(
    access: &wayfarer::internals::PackageGraphView,
    search: &wayfarer::internals::CandidateSearch,
) -> EndpointReadiness {
    let best = search.candidates.first();
    EndpointReadiness {
        candidate_count: search.candidates.len() as u64,
        rejected_candidate_count: search.rejected_candidate_count as u64,
        best_candidate_score_basis_points: best
            .map(|candidate| (candidate.score.clamp(0.0, 1.0) * 10_000.0).round() as u32),
        best_component_id: best
            .and_then(|candidate| access.node_by_id(candidate.source_node_id))
            .and_then(|node| node.component_id),
        best_component_size: best
            .and_then(|candidate| candidate.component_size)
            .map(|value| value as u64),
        best_outbound_reach_edge_count: best
            .and_then(|candidate| candidate.outbound_reach_edge_count)
            .map(|value| value as u64),
        best_inbound_reach_edge_count: best
            .and_then(|candidate| candidate.inbound_reach_edge_count)
            .map(|value| value as u64),
        best_layer: best.and_then(|candidate| candidate.layer).map(i64::from),
        best_level: best.and_then(|candidate| candidate.level.clone()),
        best_road_group_key: best.and_then(|candidate| candidate.road_group_key.clone()),
        same_road_candidate_count: best.map(|candidate| candidate.same_road_candidate_count as u64),
    }
}

fn empty_endpoint_readiness() -> EndpointReadiness {
    EndpointReadiness {
        candidate_count: 0,
        rejected_candidate_count: 0,
        best_candidate_score_basis_points: None,
        best_component_id: None,
        best_component_size: None,
        best_outbound_reach_edge_count: None,
        best_inbound_reach_edge_count: None,
        best_layer: None,
        best_level: None,
        best_road_group_key: None,
        same_road_candidate_count: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;
    use wayfarer::{
        RouteAlternatives, RouteLegMode, RouteLocation, RouteManeuverKind, RouteOverlay,
        RouteOverlayGeometryKind, RouteOverlayPolicy,
    };
    use wayfarer_compiler::package_build::PackageBuildOptions;
    use wayfarer_road_graph::{
        DirectionalAccess, RoadGraphBuilder, RoadType, RoutableWay, build_road_graph_manifest,
        enhance_road_graph, inspect_road_graph_container, serialize_road_graph,
    };

    #[test]
    fn source_replacement_drops_the_old_handle_and_loads_the_new_revision() {
        let dir = tempdir().unwrap();
        let first_path = dir.path().join("first.mrt");
        let second_path = dir.path().join("second.mrt");
        write_archive(&first_path, "first", 21.0);
        write_archive(&second_path, "second", 22.0);
        let id = RoutingSourceId::new("regional").unwrap();
        let first = RoutingSource::new(id.clone(), "v1", &first_path).unwrap();
        let second = RoutingSource::new(id, "v2", &second_path).unwrap();
        let service = RoutingService::new(2, RoutePackageMemoryProfile::Mobile);

        let cold = service.inspect_source(&first);
        let warm = service.inspect_source(&first);
        let replacement = service.inspect_source(&second);

        assert_eq!(cold.graph_id.as_deref(), Some("first"));
        assert!(!cold.cache_hit);
        assert!(warm.cache_hit);
        assert_eq!(replacement.graph_id.as_deref(), Some("second"));
        assert!(!replacement.cache_hit);
        // The only retained mmap belongs to `second`; this removal also catches
        // accidental old-handle retention on Windows.
        std::fs::remove_file(&first_path).unwrap();
    }

    #[test]
    fn revision_swap_does_not_wait_for_old_revision_computations() {
        let dir = tempdir().unwrap();
        let first_path = dir.path().join("first.mrt");
        let second_path = dir.path().join("second.mrt");
        write_archive(&first_path, "first", 21.0);
        write_archive(&second_path, "second", 22.0);
        let id = RoutingSourceId::new("regional").unwrap();
        let first = RoutingSource::new(id.clone(), "v1", &first_path).unwrap();
        let second = RoutingSource::new(id.clone(), "v2", &second_path).unwrap();
        let service = Arc::new(RoutingService::new(2, RoutePackageMemoryProfile::Mobile));
        // A long computation on the old revision holds its lease.
        let old_lease = service.load(&first).unwrap();

        let (done_tx, done_rx) = std::sync::mpsc::channel();
        let swapping_service = service.clone();
        let swap = std::thread::spawn(move || {
            done_tx
                .send(swapping_service.inspect_source(&second))
                .unwrap();
        });
        let replacement = done_rx
            .recv_timeout(std::time::Duration::from_secs(2))
            .expect("the swap waited for the old revision's lease");
        swap.join().unwrap();
        assert_eq!(replacement.graph_id.as_deref(), Some("second"));
        // The old computation still reads its own package.
        assert_eq!(old_lease.owner.package.graph_id(), "first");
        drop(old_lease);
        service.invalidate(&id);
        std::fs::remove_file(&first_path).unwrap();
    }

    fn short_route(longitude: f64) -> LegRequest {
        let mut request = LegRequest::new(
            GeoPoint::new(52.0, longitude + 0.0002),
            GeoPoint::new(52.0, longitude + 0.0018),
            MobilityProfile::Wheeled,
        );
        request.options.max_correlation_distance_m = 25;
        request
    }

    #[test]
    fn revision_swap_and_invalidation_cancel_the_old_revisions_computations() {
        let cancelled = wayfarer::RouteError::Cancelled {
            deadline_exceeded: false,
        };
        let dir = tempdir().unwrap();
        let first_path = dir.path().join("first.mrt");
        let second_path = dir.path().join("second.mrt");
        write_archive(&first_path, "first", 21.0);
        write_archive(&second_path, "second", 22.0);
        let id = RoutingSourceId::new("regional").unwrap();
        let first = RoutingSource::new(id.clone(), "v1", &first_path).unwrap();
        let second = RoutingSource::new(id.clone(), "v2", &second_path).unwrap();
        let service = Arc::new(RoutingService::new(2, RoutePackageMemoryProfile::Mobile));

        // A computation in flight on the old revision, carrying its token.
        let old = service.load(&first).unwrap();
        let mut request = short_route(21.0);
        old.cancel_with_source(&mut request.options);
        assert!(old.engine().route_leg(request.clone()).is_ok());
        assert!(service.inspect_source(&second).is_ready());
        let failure = old.engine().route_leg(request).unwrap_err();
        assert_eq!(failure.error, cancelled);
        drop(old);

        // The new revision computes; a caller's own token still applies.
        service.route_leg(&second, short_route(22.0)).unwrap();
        let token = wayfarer::CancelToken::new();
        token.cancel();
        let mut request = short_route(22.0);
        request.options.cancellation = request.options.cancellation.with_token(token);
        match service.route_leg(&second, request) {
            Err(RoutingServiceError::RouteFailed(failure)) => assert_eq!(failure.error, cancelled),
            other => panic!("{other:?}"),
        }

        // Invalidation cancels first, then waits for the lease.
        let lease = service.load(&second).unwrap();
        let mut request = short_route(22.0);
        lease.cancel_with_source(&mut request.options);
        let invalidating_service = service.clone();
        let invalidation = std::thread::spawn(move || invalidating_service.invalidate(&id));
        let started = std::time::Instant::now();
        while !request.options.cancellation.is_cancelled() {
            assert!(started.elapsed() < std::time::Duration::from_secs(2));
            std::thread::yield_now();
        }
        let failure = lease.engine().route_leg(request).unwrap_err();
        assert_eq!(failure.error, cancelled);
        drop(lease);
        invalidation.join().unwrap();
        std::fs::remove_file(&second_path).unwrap();
    }

    #[test]
    fn routed_waypoint_is_snapped_once_and_announced() {
        let dir = tempdir().unwrap();
        let archive_path = dir.path().join("route.mrt");
        write_archive(&archive_path, "route", 21.0);
        let source = source("route", "v1", &archive_path);
        let service = RoutingService::new(1, RoutePackageMemoryProfile::Mobile);
        let options = wayfarer::RouteOptions {
            max_correlation_distance_m: 25,
            ..Default::default()
        };

        let result = service
            .route(
                &source,
                RouteWorkflowRequest {
                    route: wayfarer::RouteRequest {
                        locations: vec![
                            RouteLocation::new(GeoPoint::new(52.0, 21.0002)),
                            RouteLocation::new(GeoPoint::new(52.00004, 21.0006)),
                            RouteLocation::new(GeoPoint::new(52.0, 21.0018)),
                        ],
                        legs: vec![RouteLegMode::Routed, RouteLegMode::Routed],
                        profile: MobilityProfile::Wheeled,
                        options,
                        overlays: Vec::new(),
                        departure_at_ms: None,
                        leg_dynamic_overlays: Default::default(),
                        geometry_format: Default::default(),
                    },
                    overlay_provenance: RouteWorkflowOverlayProvenance::default(),
                    mobility: None,
                    overlay_sets: Vec::new(),
                    include: None,
                },
            )
            .unwrap();

        let waypoint = &result.route.location_matches[1];
        let (incoming, outgoing) = (
            waypoint.incoming.as_ref().unwrap(),
            waypoint.outgoing.as_ref().unwrap(),
        );
        assert_eq!(incoming.edge_id, outgoing.edge_id);
        assert!(incoming.snapped_point.distance_to(outgoing.snapped_point) < 0.01);
        assert_eq!(result.route.diagnostics.waypoint_gap_count, 0);
        assert!(result.route.shape_segments.iter().all(|segment| {
            !segment
                .segment
                .quality_flags
                .contains(&wayfarer::RouteShapeQualityFlag::WaypointGap)
        }));
        let kinds = result
            .route
            .maneuvers
            .iter()
            .map(|maneuver| maneuver.kind)
            .collect::<Vec<_>>();
        assert!(!kinds.contains(&RouteManeuverKind::UTurn), "{kinds:?}");
        let arrivals = result
            .route
            .maneuvers
            .iter()
            .filter(|maneuver| maneuver.kind == RouteManeuverKind::Waypoint)
            .collect::<Vec<_>>();
        assert_eq!(arrivals.len(), 1);
        assert_eq!(arrivals[0].instruction, "Arrive at waypoint 1");
        assert_eq!(arrivals[0].leg_index, Some(0));
        assert_eq!(
            result.route.maneuvers.last().map(|maneuver| maneuver.kind),
            Some(RouteManeuverKind::Destination)
        );
        assert_eq!(result.route.maneuvers.last().unwrap().leg_index, Some(1));
        let leg_indexes = result
            .route
            .maneuvers
            .iter()
            .map(|maneuver| maneuver.leg_index.unwrap())
            .collect::<Vec<_>>();
        assert!(leg_indexes.windows(2).all(|pair| pair[0] <= pair[1]));
        // One shared point at the waypoint: no gap and no duplicate.
        assert_eq!(
            result.route.geometry.len(),
            result.route.segments[0].geometry().len() + result.route.segments[1].geometry().len()
                - 1
        );
    }

    #[test]
    fn invalidation_drains_the_mmap_before_same_path_replacement() {
        let dir = tempdir().unwrap();
        let archive_path = dir.path().join("route.mrt");
        write_archive(&archive_path, "first", 21.0);
        let id = RoutingSourceId::new("regional").unwrap();
        let first = RoutingSource::new(id.clone(), "v1", &archive_path).unwrap();
        let service = RoutingService::new(1, RoutePackageMemoryProfile::Mobile);
        assert_eq!(
            service.inspect_source(&first).graph_id.as_deref(),
            Some("first")
        );

        service.invalidate(&id);
        // Must succeed on Windows too: invalidate does not return while a
        // service-owned archive lease or mmap remains active.
        std::fs::remove_file(&archive_path).unwrap();
        write_archive(&archive_path, "second", 22.0);
        let second = RoutingSource::new(id, "v2", &archive_path).unwrap();

        let readiness = service.inspect_source(&second);
        assert_eq!(readiness.graph_id.as_deref(), Some("second"));
        assert!(!readiness.cache_hit);
        // The package carries SAR data: every relaxation is available,
        // `vehicle_on_paths` (path and width classes) included.
        let sar = readiness.sar_capability.as_ref().unwrap();
        assert!(sar.unavailable.is_empty());
        assert_eq!(sar.vehicle_on_paths_min_factor, Some(3.0));
        assert!(sar.reason.is_none());
    }

    #[test]
    fn invalidation_waits_for_active_archive_leases() {
        let dir = tempdir().unwrap();
        let archive_path = dir.path().join("route.mrt");
        write_archive(&archive_path, "route", 21.0);
        let source = source("route", "v1", &archive_path);
        let service = Arc::new(RoutingService::new(1, RoutePackageMemoryProfile::Mobile));
        let lease = service.load(&source).unwrap();
        let id = source.id().clone();
        let (done_tx, done_rx) = std::sync::mpsc::channel();
        let invalidating_service = service.clone();
        let invalidation = std::thread::spawn(move || {
            invalidating_service.invalidate(&id);
            done_tx.send(()).unwrap();
        });

        assert!(
            done_rx
                .recv_timeout(std::time::Duration::from_millis(50))
                .is_err()
        );
        drop(lease);
        done_rx
            .recv_timeout(std::time::Duration::from_secs(2))
            .unwrap();
        invalidation.join().unwrap();
    }

    #[test]
    fn execute_callback_holds_the_archive_lease_until_it_returns() {
        let dir = tempdir().unwrap();
        let archive_path = dir.path().join("route.mrt");
        write_archive(&archive_path, "first", 21.0);
        let first_source = source("route", "v1", &archive_path);
        let source_id = first_source.id().clone();
        let service = Arc::new(RoutingService::new(1, RoutePackageMemoryProfile::Mobile));
        let (entered_tx, entered_rx) = std::sync::mpsc::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel();
        let executing_service = Arc::clone(&service);
        let execution = std::thread::spawn(move || {
            executing_service
                .execute(&first_source, |engine, reader, readiness| {
                    assert!(engine.route_graph_view().is_ready());
                    assert_eq!(reader.graph_id(), "first");
                    assert!(readiness.is_ready());
                    entered_tx.send(()).unwrap();
                    release_rx.recv().unwrap();
                })
                .unwrap();
        });
        entered_rx.recv().unwrap();

        let (done_tx, done_rx) = std::sync::mpsc::channel();
        let invalidating_service = Arc::clone(&service);
        let invalidation = std::thread::spawn(move || {
            invalidating_service.invalidate(&source_id);
            done_tx.send(()).unwrap();
        });
        assert!(
            done_rx
                .recv_timeout(std::time::Duration::from_millis(50))
                .is_err()
        );
        release_tx.send(()).unwrap();
        execution.join().unwrap();
        done_rx
            .recv_timeout(std::time::Duration::from_secs(2))
            .unwrap();
        invalidation.join().unwrap();

        std::fs::remove_file(&archive_path).unwrap();
        write_archive(&archive_path, "second", 22.0);
        let replacement = source("route", "v2", &archive_path);
        assert_eq!(
            service.inspect_source(&replacement).graph_id.as_deref(),
            Some("second")
        );
    }

    #[test]
    fn source_identity_fields_are_bounded() {
        assert_eq!(
            RoutingSourceId::new("x".repeat(MAX_SOURCE_ID_BYTES + 1)),
            Err(SourceDescriptorError::IdTooLong)
        );
        let id = RoutingSourceId::new("source").unwrap();
        assert_eq!(
            RoutingSource::new(id, "x".repeat(MAX_SOURCE_REVISION_BYTES + 1), "route.mrt"),
            Err(SourceDescriptorError::RevisionTooLong)
        );
    }

    #[test]
    fn cache_capacity_is_bounded_and_evicted_sources_reopen_cold() {
        let dir = tempdir().unwrap();
        let first_path = dir.path().join("first.mrt");
        let second_path = dir.path().join("second.mrt");
        write_archive(&first_path, "first", 21.0);
        write_archive(&second_path, "second", 22.0);
        let first = source("first", "v1", &first_path);
        let second = source("second", "v1", &second_path);
        let service = RoutingService::new(1, RoutePackageMemoryProfile::Mobile);

        assert!(!service.inspect_source(&first).cache_hit);
        assert!(!service.inspect_source(&second).cache_hit);
        assert!(!service.inspect_source(&first).cache_hit);
    }

    #[test]
    fn base_compute_and_corridor_inspection_need_no_platform_enrichment() {
        let dir = tempdir().unwrap();
        let archive_path = dir.path().join("route.mrt");
        write_archive(&archive_path, "route", 21.0);
        let source = source("route", "v1", &archive_path);
        let service = RoutingService::new(1, RoutePackageMemoryProfile::Mobile);
        let origin = GeoPoint::new(52.0, 21.00005);
        let destination = GeoPoint::new(52.0, 21.00195);

        let corridor = service.inspect_corridor(
            &source,
            CorridorQuery {
                origin,
                destination,
                profile: MobilityProfile::Wheeled,
                correlation_policy: CorrelationPolicy {
                    max_distance_m: 25,
                    ..CorrelationPolicy::default()
                },
                origin_options: RouteEndpointOptions::default(),
                destination_options: RouteEndpointOptions::default(),
            },
        );
        assert!(corridor.source.is_ready());
        assert!(corridor.origin.candidate_count > 0);
        assert!(corridor.destination.candidate_count > 0);

        let mut request = LegRequest::new(origin, destination, MobilityProfile::Wheeled);
        request.options.max_correlation_distance_m = 25;
        let computed = service.route_leg(&source, request).unwrap();
        assert_eq!(computed.source_id.as_str(), "route");
        assert!(computed.cache_hit);
        assert!(computed.route.distance_m > 0.0);
        assert!(!computed.route.edge_ids.is_empty());
    }

    #[test]
    fn workflow_owns_waypoint_stitching_and_exposes_outgoing_snap_evidence() {
        let dir = tempdir().unwrap();
        let archive_path = dir.path().join("route.mrt");
        write_archive(&archive_path, "route", 21.0);
        let source = source("route", "v1", &archive_path);
        let service = RoutingService::new(1, RoutePackageMemoryProfile::Mobile);
        let mut options = wayfarer::RouteOptions {
            max_correlation_distance_m: 25,
            ..Default::default()
        };
        options.alternatives.requested_alternative_count = 1;
        let mut origin = RouteLocation::new(GeoPoint::new(52.0, 21.00005));
        // The origin departs on a direct leg. Its reach hint must not leak onto
        // the routed leg that departs from the intermediate waypoint.
        origin.options.min_outbound_reach_edge_count = Some(usize::MAX);

        let result = service
            .route(
                &source,
                RouteWorkflowRequest {
                    route: wayfarer::RouteRequest {
                        locations: vec![
                            origin,
                            RouteLocation::new(GeoPoint::new(52.0, 21.001)),
                            RouteLocation::new(GeoPoint::new(52.0, 21.00195)),
                        ],
                        legs: vec![RouteLegMode::Direct, RouteLegMode::Routed],
                        profile: MobilityProfile::OnFoot,
                        options,
                        overlays: vec![RouteOverlay::new(
                            RouteOverlayGeometryKind::Point,
                            vec![GeoPoint::new(52.0, 21.0015)],
                            RouteOverlayPolicy::Warning,
                            25,
                        )],
                        departure_at_ms: None,
                        leg_dynamic_overlays: Default::default(),
                        geometry_format: Default::default(),
                    },
                    overlay_provenance: RouteWorkflowOverlayProvenance {
                        policy_version: Some("request-overlay-v2".to_string()),
                        source_ids: vec!["entity-zone-1".to_string()],
                    },
                    mobility: None,
                    overlay_sets: Vec::new(),
                    include: None,
                },
            )
            .unwrap();

        assert_eq!(result.route.segments.len(), 2);
        assert!(matches!(
            result.route.alternatives,
            RouteAlternatives::UnavailableForMultiLeg
        ));
        assert!(result.route.location_matches[1].incoming.is_none());
        assert!(result.route.location_matches[1].outgoing.is_some());
        assert!(result.route.location_matches[1].preferred().is_some());
        assert_eq!(
            result.route.geometry.first(),
            result.route.segments[0].geometry().first()
        );
        assert_eq!(
            result.route.geometry.last(),
            result.route.segments[1].geometry().last()
        );
        assert_eq!(
            result
                .route
                .overlay_diagnostics
                .hazard_warning_overlay_count,
            1
        );
        assert!(result.route.diagnostics.origin_candidate_count > 0);
        assert!(result.route.diagnostics.destination_candidate_count > 0);
        assert!(result.route.diagnostics.warning_edge_count > 0);
        assert!(!result.route.edge_attributions.is_empty());
        assert!(!result.route.shape_segments.is_empty());
        assert!(result.route.cost_summary.costed_edge_count > 0);
        assert!(!result.route.explanations.is_empty());
        assert_eq!(
            result.overlay_provenance.policy_version.as_deref(),
            Some("request-overlay-v2")
        );
    }

    #[test]
    fn workflow_failure_preserves_source_segment_and_engine_diagnostics() {
        let dir = tempdir().unwrap();
        let archive_path = dir.path().join("route.mrt");
        write_archive(&archive_path, "route", 21.0);
        let source = source("route", "v1", &archive_path);
        let service = RoutingService::new(1, RoutePackageMemoryProfile::Mobile);
        let mut options = wayfarer::RouteOptions::default();
        // The fixture contains two bidirectional ways, hence four directed
        // edges. Blocking the full fixture avoids correlation selecting the
        // opposing direction as a valid fallback.
        options.blocked_edge_ids.extend([1, 2, 3, 4]);

        let error = service
            .route(
                &source,
                RouteWorkflowRequest {
                    route: wayfarer::RouteRequest {
                        locations: vec![
                            RouteLocation::new(GeoPoint::new(52.0, 21.00005)),
                            RouteLocation::new(GeoPoint::new(52.0, 21.00195)),
                        ],
                        legs: vec![RouteLegMode::Routed],
                        profile: MobilityProfile::Wheeled,
                        options,
                        overlays: Vec::new(),
                        departure_at_ms: None,
                        leg_dynamic_overlays: Default::default(),
                        geometry_format: Default::default(),
                    },
                    overlay_provenance: RouteWorkflowOverlayProvenance::default(),
                    mobility: None,
                    overlay_sets: Vec::new(),
                    include: None,
                },
            )
            .unwrap_err();
        let RoutingServiceError::WorkflowFailed(RouteWorkflowError::Route(failure)) = error else {
            panic!("expected a segment-qualified workflow failure");
        };
        assert_eq!(failure.segment_index, Some(0));
        assert_eq!(failure.diagnostics.final_path_edge_count, 0);
    }

    #[test]
    fn missing_archive_is_reported_without_leaking_its_path() {
        let source = source(
            "missing",
            "v1",
            Path::new("/sensitive/application/storage/route.mrt"),
        );
        let service = RoutingService::new(1, RoutePackageMemoryProfile::Mobile);

        let readiness = service.inspect_source(&source);
        let encoded = serde_json::to_string(&readiness).unwrap();

        assert_eq!(readiness.status, SourceReadinessStatus::Missing);
        assert_eq!(readiness.problem, Some(SourceProblem::ArchiveMissing));
        assert!(readiness.sar_capability.is_none());
        assert!(!encoded.contains("sensitive"));
        assert!(!encoded.contains("route.mrt"));
    }

    fn source(id: &str, revision: &str, path: &Path) -> RoutingSource {
        RoutingSource::new(RoutingSourceId::new(id).unwrap(), revision, path).unwrap()
    }

    fn write_archive(path: &Path, graph_id: &str, longitude: f64) {
        let mut builder = RoadGraphBuilder::new(graph_id);
        builder.push_way(RoutableWay {
            osm_way_id: 1,
            points: vec![(52.0, longitude), (52.0, longitude + 0.001)],
            road_type: RoadType::Primary,
            directional_access: DirectionalAccess::Both,
            name: Some("First".into()),
            highway: Some("primary".into()),
            access: Some("yes".into()),
            ..RoutableWay::default()
        });
        builder.push_way(RoutableWay {
            osm_way_id: 2,
            points: vec![(52.0, longitude + 0.001), (52.0, longitude + 0.002)],
            road_type: RoadType::Primary,
            directional_access: DirectionalAccess::Both,
            name: Some("Second".into()),
            highway: Some("primary".into()),
            access: Some("yes".into()),
            ..RoutableWay::default()
        });
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);
        let bytes = serialize_road_graph(&graph).unwrap();
        let container = inspect_road_graph_container(&bytes).unwrap();
        let manifest = build_road_graph_manifest(&graph, &container);
        wayfarer_compiler::package_build::write_package(
            &graph,
            Some(&manifest),
            path,
            &PackageBuildOptions::default(),
        )
        .unwrap();
    }

    #[test]
    fn verification_and_outdated_packages_are_reported() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("route.mrt");
        write_archive(&path, "route", 21.0);
        let route = source("route", "v1", &path);
        let service = RoutingService::new(1, RoutePackageMemoryProfile::Mobile);
        let verified = service.verify_source(&route).unwrap();
        let readiness = service.inspect_source(&route);
        assert_eq!(
            readiness.archive_hash.as_deref(),
            Some(verified.dataset_hash.as_str())
        );
        assert_eq!(readiness.tile_count, 0);

        // A corrupted byte past the header fails verification only.
        let mut bytes = std::fs::read(&path).unwrap();
        let last = bytes.len() - 1;
        let at = (0..=last).rev().find(|at| bytes[*at] != 0).unwrap();
        bytes[at] ^= 0x40;
        let corrupt = dir.path().join("corrupt.mrt");
        std::fs::write(&corrupt, &bytes).unwrap();
        let error = service
            .verify_source(&source("corrupt", "v1", &corrupt))
            .unwrap_err();
        assert!(matches!(
            error,
            RoutingServiceError::PackageVerificationFailed(_)
        ));

        // A package of an older format is outdated, not invalid and not a
        // panic.
        let mut archive = std::fs::read(&path).unwrap();
        // The header's format version, after the 8-byte magic.
        archive[8..12].copy_from_slice(&0u32.to_le_bytes());
        let old = dir.path().join("old.mrt");
        std::fs::write(&old, archive).unwrap();
        let readiness = service.inspect_source(&source("old", "v1", &old));
        assert_eq!(readiness.status, SourceReadinessStatus::Stale);
        assert_eq!(readiness.problem, Some(SourceProblem::PackageOutdated));
        let error = service
            .verify_source(&source("old", "v1", &old))
            .unwrap_err();
        assert!(error.to_string().contains("recompile"), "{error}");
    }
}

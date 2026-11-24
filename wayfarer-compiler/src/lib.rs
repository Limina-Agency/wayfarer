//! Compiles an OpenStreetMap PBF extract into a routing package
//! (`routing.mrt`): ingest, road graph, baked edge costs, the node order of
//! CCH potentials, and the package file. This deliberately knows nothing about vector layers or map
//! tiles; routing compilation starts from the PBF on its own. The command
//! line is the `wayfarer` binary (crate `wayfarer-cli`).
//!
//! ```no_run
//! use wayfarer_compiler::{CancelToken, CompileRequest, compile};
//!
//! # fn main() -> Result<(), wayfarer_compiler::CompileError> {
//! let request = CompileRequest::new("monaco.osm.pbf", "routing.mrt")
//!     .country("MC")
//!     .threads(2); // or leave unset for the current rayon pool
//! let cancel = CancelToken::new(); // `cancel.cancel()` from any thread stops it
//! let result = compile(&request, &cancel, |progress| {
//!     eprintln!("{} {:.0}%", progress.stage.as_str(), progress.overall * 100.0)
//! })?;
//! println!("{} bytes, dataset hash {}", result.byte_len, result.dataset_hash);
//! # Ok(())
//! # }
//! ```
//!
//! A host that cannot take callbacks starts a [`CompileJob`] and polls it.
//!
//! # Stability
//!
//! Version 0.x: the library API may change between minor releases. The package
//! it writes has a layout version and records the costing model it was baked
//! with; an engine opens only packages of a format it reads and of its own
//! costing model. The compiler writes the same bytes for the same input on
//! every platform, build profile, memory profile and thread count.

pub mod package_build;

use osm_pbf_ingest::Tags;
use osm_pbf_ingest::{
    INGEST_CANCELLED, IngestBuildProfile, IngestStage, LocateOptions, LocatedNodes, NodeBatch,
    NodeLocations, NormalizedRelation, Pass2Options, Pass2Sink, RelationBatch, RelationMemberType,
    SharedIngestProgress, TagLookup as PbfTagLookup, WayBatch, locate_nodes,
    run_pass2_with_options,
};
pub use package_build::{PackageBuildOptions, PackageBuildReport};
use rayon::prelude::*;
use rustc_hash::FxHashMap;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use wayfarer_line_network::{
    DirectionalAccessModel, RoadRoutingAttrs, SharedLineInput, SharedLineLayer, SharedLineNetwork,
    SharedLineNetworkBuilder, SharedLineTopology, SharedTopologyEdge,
};
use wayfarer_package::{codec, math};
use wayfarer_road_graph::{
    DirectionalAccess, EnhanceOptions, NodeBarrierTags, ROAD_GRAPH_SNAPSHOT_VERSION,
    RelationMemberRefType, RoadGraph, RoadGraphBuilder, RoadGraphInputReport,
    RoadGraphPackageManifest, RoadGraphQualityManifest, RoadGraphTopologyRepairReport,
    RoadTurnRestriction, RoadType, RoadWayFacts, RoutableWay, SharedInfoIndex, TagInterner,
    TemplateWay, TrafficControl, TrafficControlDirection, TrafficControlPoint, WayTemplate,
    enhance_road_graph_with, evaluate_road_graph_quality, resolve_turn_restrictions_with_report,
    vehicle_access_is_destination_only,
};

mod api;
mod compile_job;
mod job;
mod profiling;
mod route_memberships;

pub use api::{
    CompileError, CompileProgress, CompileRequest, CompileResult, MemoryProfile, compile,
    host_protocol,
};
pub use compile_job::CompileJob;
pub use wayfarer_package::{CancelToken, MobilityProfile};

use job::{Job, ProgressFn};
use profiling::Phase;
use route_memberships::CollectedRouteMemberships;

/// The road-graph half of a compile, as `api` resolves it from a
/// [`CompileRequest`].
#[derive(Debug, Clone)]
pub(crate) struct RoadGraphCompileRequest {
    pub input_pbf_path: PathBuf,
    /// Scratch space (the encoded way facts while the graph is built). The
    /// compile owns it:
    /// it is removed when the compile ends, whether it succeeds, fails or is
    /// cancelled.
    pub temp_dir: PathBuf,
    pub graph_id: String,
    pub country_code: Option<String>,
    /// Changes memory use and speed, never the package's bytes.
    pub build_profile: IngestBuildProfile,
    /// The order the road segments reach the graph builder in. Never changes
    /// the package's bytes either: the package's order and ids follow from
    /// the graph alone, which is what lets a tile of a larger compile number
    /// its part as the whole compile would.
    pub segment_order: SegmentOrder,
}

/// See [`RoadGraphCompileRequest::segment_order`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) enum SegmentOrder {
    /// The line network's order (last segment first).
    #[default]
    Topology,
    /// Tests: a pseudo-random permutation from the seed.
    #[cfg(test)]
    Shuffled(u64),
}

#[derive(Debug)]
struct RoadGraphCompileResult {
    pub road_graph: RoadGraph,
    pub road_graph_quality: RoadGraphQualityManifest,
    /// Counted before the graph discards its raw restrictions and memberships.
    pub raw_turn_restriction_count: usize,
    pub route_membership_count: usize,
}

/// The stages of a compile, in the order they run. The serialized names
/// (`as_str`) are stable ids an app can key its own text on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum CompileStage {
    /// The ingest's first pass: road ways and their relations (turn
    /// restrictions, road routes), from the way and relation blobs.
    CollectRoads,
    /// The ingest's second pass: the locations of the road ways' nodes, and
    /// the barrier and traffic-control nodes, from the node blobs. It runs
    /// after [`CollectRoads`](Self::CollectRoads); its id keeps the name it
    /// had when it ran first and indexed every node.
    IndexNodes,
    /// The road graph: topology, restrictions, enrichment and repair.
    FinalizeGraph,
    /// Edge costs and the package's sections other than the node order.
    BuildPackage,
    /// The node order of CCH potentials (nested dissection).
    CchOrder,
    /// Checksums, the file written and synced, then reopened.
    WritePackage,
}

impl CompileStage {
    /// Every stage, in the order they run.
    pub const ALL: [Self; 6] = [
        Self::CollectRoads,
        Self::IndexNodes,
        Self::FinalizeGraph,
        Self::BuildPackage,
        Self::CchOrder,
        Self::WritePackage,
    ];

    pub fn as_str(self) -> &'static str {
        match self {
            Self::CollectRoads => "collect_roads",
            Self::IndexNodes => "index_nodes",
            Self::FinalizeGraph => "finalize_graph",
            Self::BuildPackage => "build_package",
            Self::CchOrder => "cch_order",
            Self::WritePackage => "write_package",
        }
    }

    /// The stage's approximate share of a regional compile's time (rough
    /// estimates; only `overall` depends on them).
    fn weight(self) -> f64 {
        match self {
            Self::CollectRoads => 0.20,
            Self::IndexNodes => 0.20,
            Self::FinalizeGraph => 0.25,
            Self::BuildPackage => 0.10,
            Self::CchOrder => 0.20,
            Self::WritePackage => 0.05,
        }
    }

    /// The share of the whole compile done when `fraction` of this stage is
    /// (0 when unknown), 0 to 1; never decreasing over a compile's reports.
    pub fn overall(self, fraction: Option<f64>) -> f64 {
        let before = Self::ALL
            .iter()
            .take_while(|stage| **stage != self)
            .map(|stage| stage.weight())
            .sum::<f64>();
        let total = Self::ALL.iter().map(|stage| stage.weight()).sum::<f64>();
        let within = fraction.unwrap_or(0.0).clamp(0.0, 1.0) * self.weight();
        ((before + within) / total).clamp(0.0, 1.0)
    }

    fn of_ingest(stage: IngestStage) -> Self {
        match stage {
            IngestStage::IngestNodeLocations | IngestStage::IngestPass1Index => Self::IndexNodes,
            _ => Self::CollectRoads,
        }
    }
}

/// A whole compile, resolved from a [`CompileRequest`].
#[derive(Debug, Clone)]
pub(crate) struct RouteArchiveCompileRequest {
    pub graph: RoadGraphCompileRequest,
    /// Where the routing package (`routing.mrt`) is written. It is written
    /// to `<path>.tmp` and renamed into place at the very end, so an existing
    /// package there is replaced only by a complete one.
    pub route_archive_path: PathBuf,
    /// As [`CompileRequest::threads`].
    pub threads: Option<usize>,
    /// The package build's options ([`CompileRequest::profiles`]).
    pub package_options: PackageBuildOptions,
}

/// The written routing package, as the compile saw it.
#[derive(Debug)]
pub(crate) struct RouteArchiveCompileResult {
    pub byte_len: u64,
    pub dataset_hash: String,
    pub format_version: u32,
    pub costing_model_version: u32,
    pub node_count: u64,
    pub edge_count: u64,
    pub report: PackageBuildReport,
    pub graph_manifest: RoadGraphPackageManifest,
}

/// Compiles the complete routing product: one routing package written
/// straight from the road graph, which never leaves this call. Runs on a
/// pool of `request.threads` threads or the current one; removes the scratch
/// directory whatever the outcome.
pub(crate) fn compile_route_archive(
    request: &RouteArchiveCompileRequest,
    should_cancel: &(dyn Fn() -> bool + Sync),
    on_progress: &mut ProgressFn<'_>,
) -> Result<RouteArchiveCompileResult, CompileError> {
    let profile = request.graph.build_profile;
    let mut compile = |workers: usize| {
        let job = Job::new(should_cancel, on_progress);
        compile_with_job(request, workers, &job)
    };
    let result = match request.threads {
        Some(0) => Err(CompileError::InvalidRequest(
            "the compile needs at least one thread".into(),
        )),
        Some(threads) => rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .thread_name(|index| format!("route-compiler-{index}"))
            .build()
            .map_err(|error| {
                CompileError::Failed(format!("Failed to start the compile's threads: {error}"))
            })
            .and_then(|pool| pool.install(|| compile(threads))),
        None => compile(default_decode_workers(
            profile,
            rayon::current_num_threads(),
        )),
    };
    if result.is_err() && request.graph.temp_dir.exists() {
        let _ = std::fs::remove_dir_all(&request.graph.temp_dir);
    }
    result
}

/// The ingest's decode workers when the request sets no thread count: one
/// per thread of the pool the compile runs on for the standard profile, and
/// the low-memory profile's own (at most two) within that pool.
///
/// The standard profile's own default in `osm-pbf-ingest` leaves two cores
/// for the reader and the collector, but both are nearly idle while the
/// workers decode, so on four cores it left half of them unused (Małopolskie's
/// ingest took 4.2 s with it and 3.8 s with four workers).
fn default_decode_workers(profile: IngestBuildProfile, pool_threads: usize) -> usize {
    match profile {
        IngestBuildProfile::DesktopFull => pool_threads,
        IngestBuildProfile::MobileOperational => profile.worker_threads().min(pool_threads),
    }
    .max(1)
}

fn compile_with_job(
    request: &RouteArchiveCompileRequest,
    workers: usize,
    job: &Job<'_>,
) -> Result<RouteArchiveCompileResult, CompileError> {
    job.check()?;
    let compiled = compile_road_graph_from_pbf(&request.graph, workers, job)?;
    let mut road_graph = compiled.road_graph;
    let quality = compiled.road_graph_quality;
    let largest_component_ratio = if road_graph.node_count() == 0 {
        1.0
    } else {
        quality.enhancement_report.largest_component_node_count as f64
            / road_graph.node_count() as f64
    };
    let mut graph_manifest = RoadGraphPackageManifest {
        graph_schema_version: ROAD_GRAPH_SNAPSHOT_VERSION,
        node_count: road_graph.node_count(),
        directed_edge_count: road_graph.edge_count(),
        shared_edge_info_count: road_graph.shared_edge_infos.len(),
        raw_turn_restriction_count: compiled.raw_turn_restriction_count,
        resolved_turn_restriction_count: road_graph.resolved_restrictions.len(),
        route_membership_count: compiled.route_membership_count,
        largest_weakly_connected_component_ratio: largest_component_ratio,
        quality,
        ..RoadGraphPackageManifest::default()
    };
    graph_manifest.quality = evaluate_road_graph_quality(&graph_manifest);
    // Queries read neither way facts, route memberships nor the raw turn
    // restrictions (only the resolved ones): free them before the build.
    road_graph.way_facts = Vec::new();
    road_graph.route_memberships = Vec::new();
    road_graph.turn_restrictions = Vec::new();
    job.check()?;
    #[cfg(feature = "benchmark-profiling")]
    let _ = package_build::PACKAGE_BUILD_CHECKPOINT.set(profiling::checkpoint);
    let phase = Phase::start("routing_package");
    let report = package_build::write_package_for_job(
        road_graph,
        Some(&graph_manifest),
        &request.route_archive_path,
        &request.package_options,
        Some(&request.graph.temp_dir),
        job,
    )
    .map_err(|error| {
        if job.cancelled() {
            CompileError::Cancelled
        } else {
            CompileError::Failed(format!("Failed to write routing package: {error}"))
        }
    })?;
    drop(phase);
    // The package is complete and in place: from here a cancel is too late.
    let phase = Phase::start("package_open");
    let package = wayfarer_package::format::PackageFile::open(&request.route_archive_path)
        .map_err(|error| {
            let _ = std::fs::remove_file(&request.route_archive_path);
            format!("Written routing package cannot be opened: {error}")
        })?;
    let counts = package.counts();
    let written = RouteArchiveCompileResult {
        byte_len: package.byte_len() as u64,
        dataset_hash: package.dataset_hash().to_string(),
        format_version: package.format_version(),
        costing_model_version: package.metadata().costing_model_version,
        node_count: counts.node_count as u64,
        edge_count: counts.edge_count as u64,
        report,
        graph_manifest,
    };
    drop(package);
    drop(phase);
    let phase = Phase::start("scratch_cleanup");
    if request.graph.temp_dir.exists() {
        std::fs::remove_dir_all(&request.graph.temp_dir).map_err(|error| {
            format!(
                "The routing package was written but routing scratch {:?} could not be removed: {error}",
                request.graph.temp_dir
            )
        })?;
    }
    drop(phase);
    Ok(written)
}

/// Compile the complete in-memory road graph directly from a PBF, without any
/// vector-compiler or source-store state.
///
/// The ingest reads the ways before the nodes. Pass 2 (way and relation blobs
/// only) collects the road ways with their node IDs; [`locate_nodes`] then
/// reads the node blobs once and keeps the locations of exactly those nodes,
/// and the barrier and traffic-control nodes. There is no node-location
/// store: no lookup goes to disk or jumps around a store of every node of the
/// file, so a country-sized extract no longer depends on the page cache.
fn compile_road_graph_from_pbf(
    request: &RoadGraphCompileRequest,
    workers: usize,
    job: &Job<'_>,
) -> Result<RoadGraphCompileResult, CompileError> {
    let mut forward = |progress: SharedIngestProgress| {
        job.report_ingest(CompileStage::of_ingest(progress.stage), progress);
    };
    let ingest_error = |error: String| {
        if error == INGEST_CANCELLED || job.cancelled() {
            CompileError::Cancelled
        } else {
            CompileError::Failed(error)
        }
    };
    let phase = Phase::start("pass2_collect_roads");
    job.report(
        CompileStage::CollectRoads,
        Some(0.0),
        0,
        "Collecting road ways",
    );
    let mut roads = OsmRoadNetworkCollector::default();
    let summary = run_pass2_with_options(
        &request.input_pbf_path,
        request.build_profile,
        Pass2Options::new()
            .worker_threads(workers)
            .skip_nodes()
            .keep_ways(|tags| is_road_way(tags)),
        &mut roads,
        &mut forward,
        || job.cancelled(),
    )
    .map_err(ingest_error)?;
    job.check()?;
    job.report(
        CompileStage::CollectRoads,
        Some(1.0),
        summary.way_objects + summary.relation_objects,
        "Collected road ways",
    );
    drop(phase);
    let phase = Phase::start("locate_road_nodes");
    job.report(
        CompileStage::IndexNodes,
        Some(0.0),
        0,
        "Locating the road ways' nodes",
    );
    let node_ids = roads.take_node_ids().map_err(CompileError::Failed)?;
    let located = locate_nodes(
        &request.input_pbf_path,
        request.build_profile,
        node_ids,
        LocateOptions::new()
            .worker_threads(workers)
            .keep_nodes(|tags| is_kept_node(tags)),
        &mut forward,
        || job.cancelled(),
    )
    .map_err(ingest_error)?;
    job.check()?;
    job.report(
        CompileStage::IndexNodes,
        Some(1.0),
        located.metrics.scanned_node_objects,
        "Located the road ways' nodes",
    );
    drop(phase);
    let phase = Phase::start("place_road_ways");
    let roads = roads.place(located)?;
    drop(phase);
    let phase = Phase::start("shared_line_network_finish");
    report_finalize_checkpoint(job, "before_line_network_finish", 0)?;
    let inputs = roads
        .finish(&request.temp_dir)
        .map_err(CompileError::Failed)?;
    drop(phase);
    build_road_graph(
        &request.graph_id,
        request.country_code.as_deref(),
        request.segment_order,
        inputs,
        job,
    )
}

/// Pass 2's sink: the road ways, their facts and the relations, before any
/// node is located.
#[derive(Default)]
pub(crate) struct OsmRoadNetworkCollector {
    /// The routable ways' lines in file order, pushed without geometry:
    /// `node_chain` holds every node ID of the way until
    /// [`place`](Self::place) keeps the located ones.
    builder: SharedLineNetworkBuilder,
    /// How many lines `builder` holds.
    pushed: usize,
    /// The last way batch, collected but not yet appended: see
    /// [`on_way_batch`](Pass2Sink::on_way_batch).
    collected: Vec<Result<CollectedWay, String>>,
    /// Ferries whose `duration` becomes their speed once their length is
    /// known, with the position of their line in `builder`.
    ferries: Vec<(usize, Box<PendingFerry>)>,
    /// Link ways with no `junction:ref`, as (their entry in the way facts'
    /// index, their departure node): a motorway junction node there gives
    /// them its exit number ([`apply_junction_exits`]).
    exit_links: Vec<(usize, i64)>,
    turn_restrictions: Vec<RoadTurnRestriction>,
    route_memberships: CollectedRouteMemberships,
    way_facts: EncodedWayFacts,
    /// The last way id appended, and whether some way id was not above the
    /// one before it: only then can a way id appear twice (a file sorted by
    /// id, as PBF extracts are, never sets it).
    last_way_id: Option<i64>,
    way_ids_unordered: bool,
    /// While `way_ids_unordered`: each line's position in the way facts'
    /// index, by its position in `builder`, to keep the facts of the copy of
    /// a repeated way whose line is kept.
    line_facts: Vec<u32>,
}

/// A routable way as pass 2 saw it: its line without geometry, and
/// `node_chain` every node ID of the way, located or not.
struct UnplacedWay {
    input: SharedLineInput,
    /// A ferry whose `duration` becomes its speed once its length is known;
    /// boxed, as `RoadWayFacts` is about 1 KB and ferries are few.
    ferry: Option<Box<PendingFerry>>,
}

/// A ferry's facts, waiting for the length of its located geometry.
struct PendingFerry {
    facts: RoadWayFacts,
    duration_s: f64,
    /// Where its encoded facts go in [`EncodedWayFacts`].
    slot: usize,
}

/// Per-way facts for every highway way, kept encoded ([`codec`]) from collection
/// until the graph reads them back one way at a time.
///
/// A decoded `RoadWayFacts` is ~1 KB of mostly-empty optional fields; held as
/// a map for a region (~1M ways) it was ~1.1 GiB live through line-network
/// finish, graph building and enrichment. Encoded, Małopolskie's 670k ways
/// take 57 MB plus a 16-byte index entry per way.
///
/// Between collection and enrichment the bytes wait in a scratch file
/// ([`spill`](EncodedWayFacts::spill)): nothing reads them while the line
/// network and the graph are built, the compile's highest steps before the
/// package build. Enrichment then reads them from the file, mapped
/// ([`restore`](EncodedWayFacts::restore)): its pages are the file's, which
/// the system can drop and read again under pressure, not the heap's.
#[derive(Default)]
struct EncodedWayFacts {
    /// `(way id, byte offset)`, sorted by way id once collection ends.
    index: Vec<(i64, u64)>,
    bytes: Vec<u8>,
    /// Where the bytes are while spilled.
    spilled: Option<PathBuf>,
    /// The spilled file once restored, and its path, removed on drop.
    mapped: Option<(memmap2::Mmap, PathBuf)>,
}

impl EncodedWayFacts {
    /// Appends facts already encoded with [`codec::to_vec`].
    fn insert_encoded(&mut self, way_id: i64, encoded: &[u8]) {
        self.index.push((way_id, self.bytes.len() as u64));
        self.bytes.extend_from_slice(encoded);
    }

    /// Takes `way_id`'s place in the index now, for facts encoded later and
    /// given to [`fill`](Self::fill) with the returned slot, so that a way id
    /// inserted more than once resolves as if they had been inserted here.
    fn reserve(&mut self, way_id: i64) -> usize {
        self.index.push((way_id, u64::MAX));
        self.index.len() - 1
    }

    /// The facts for a slot [`reserve`](Self::reserve) returned.
    fn fill(&mut self, slot: usize, encoded: &[u8]) {
        self.index[slot].1 = self.bytes.len() as u64;
        self.bytes.extend_from_slice(encoded);
    }

    /// Sorts the index for lookup. A way id inserted more than once keeps its
    /// last facts, as the map's `insert` did.
    fn seal(&mut self) {
        debug_assert!(self.index.iter().all(|(_, offset)| *offset != u64::MAX));
        self.index.sort_by_key(|(way_id, _)| *way_id);
        let mut sealed = Vec::<(i64, u64)>::with_capacity(self.index.len());
        for entry in self.index.drain(..) {
            match sealed.last_mut() {
                Some(last) if last.0 == entry.0 => *last = entry,
                _ => sealed.push(entry),
            }
        }
        sealed.shrink_to_fit();
        self.index = sealed;
        self.bytes.shrink_to_fit();
    }

    /// Moves the bytes to a file in `dir` until [`restore`](Self::restore).
    fn spill(&mut self, dir: &Path) -> Result<(), String> {
        let path = dir.join("way-facts.spill");
        std::fs::create_dir_all(dir)
            .and_then(|()| std::fs::write(&path, &self.bytes))
            .map_err(|error| format!("Failed to spill way facts to {path:?}: {error}"))?;
        self.bytes = Vec::new();
        self.spilled = Some(path);
        Ok(())
    }

    /// Maps the spilled file for reading; it is removed when the facts drop.
    fn restore(&mut self) -> Result<(), String> {
        let Some(path) = self.spilled.take() else {
            return Ok(());
        };
        let map = std::fs::File::open(&path)
            .and_then(|file| {
                if file.metadata()?.len() == 0 {
                    // Nothing to map (and an empty mapping is refused).
                    return Ok(None);
                }
                // SAFETY: the file is this compile's own scratch file, written
                // by `spill` and not changed or truncated while mapped.
                unsafe { memmap2::Mmap::map(&file) }.map(Some)
            })
            .map_err(|error| format!("Failed to map spilled way facts {path:?}: {error}"))?;
        match map {
            Some(map) => self.mapped = Some((map, path)),
            None => {
                let _ = std::fs::remove_file(&path);
            }
        }
        Ok(())
    }

    fn bytes(&self) -> &[u8] {
        match &self.mapped {
            Some((map, _)) => map,
            None => &self.bytes,
        }
    }

    fn get(&self, way_id: i64) -> Result<Option<RoadWayFacts>, String> {
        debug_assert!(self.spilled.is_none(), "way facts read while spilled");
        let Ok(position) = self.index.binary_search_by_key(&way_id, |(id, _)| *id) else {
            return Ok(None);
        };
        let offset = self.index[position].1 as usize;
        codec::take_from_slice(&self.bytes()[offset..])
            .map(|(facts, _)| Some(facts))
            .map_err(|error| format!("Failed to decode way facts for way {way_id}: {error}"))
    }
}

impl Drop for EncodedWayFacts {
    fn drop(&mut self) {
        if let Some((map, path)) = self.mapped.take() {
            // Unmapped first: a mapped file cannot be removed on Windows. The
            // scratch directory goes at the end of the compile regardless.
            drop(map);
            let _ = std::fs::remove_file(path);
        }
    }
}

struct CollectedRoadInputs {
    /// The line network, its topology's edges taken out into
    /// `topology_edges`.
    network: SharedLineNetwork,
    /// The topology's edges, in edge id order, a batch at a time.
    topology_edges: Vec<Vec<SharedTopologyEdge>>,
    turn_restrictions: Vec<RoadTurnRestriction>,
    route_memberships: CollectedRouteMemberships,
    barrier_node_tags: HashMap<i64, NodeBarrierTags>,
    traffic_controls: Vec<TrafficControlPoint>,
    way_facts: EncodedWayFacts,
    input_report: RoadGraphInputReport,
}

impl OsmRoadNetworkCollector {
    /// Appends the last way batch, and returns the distinct node IDs of the
    /// routable ways, sorted: the nodes to locate.
    fn take_node_ids(&mut self) -> Result<Vec<i64>, String> {
        let last = std::mem::take(&mut self.collected);
        self.append(last)?;
        let inputs = self.builder.inputs_mut();
        let total = inputs.iter().map(|input| input.node_chain.len()).sum();
        let mut node_ids = Vec::with_capacity(total);
        for input in inputs.iter() {
            node_ids.extend_from_slice(&input.node_chain);
        }
        node_ids.par_sort_unstable();
        node_ids.dedup();
        node_ids.shrink_to_fit();
        Ok(node_ids)
    }

    /// Gives every routable way the geometry of its located nodes, cut where
    /// the file does not hold a node ([`place_line`]), and drops the lines
    /// with fewer than two; keeps one copy of a way id the file repeats
    /// ([`drop_repeated_ways`]); takes the barrier and traffic-control nodes.
    /// What it cut or dropped goes in the [`RoadGraphInputReport`].
    fn place(mut self, located: LocatedNodes) -> Result<UnfinishedRoadInputs, String> {
        let mut barrier_node_tags = HashMap::new();
        let mut traffic_controls = Vec::new();
        let mut junctions = HashMap::new();
        for node in &located.tagged_nodes {
            if let Some(tags) = barrier_node_tags_from_tags(&node.tags) {
                barrier_node_tags.insert(node.node_id, tags);
            }
            traffic_controls.extend(traffic_control_point(node));
            if let Some(junction) = junction_exit_from_tags(&node.tags) {
                junctions.insert(node.node_id, junction);
            }
        }
        apply_junction_exits(&mut self.way_facts, &self.exit_links, &junctions)?;
        drop(junctions);
        drop(std::mem::take(&mut self.exit_links));
        let locations = &located.locations;
        let inputs = self.builder.inputs_mut();
        // The lines with nodes the file does not hold, cut there, by position.
        let cut_lines = inputs
            .par_iter_mut()
            .enumerate()
            .filter_map(|(position, input)| place_line(locations, input).map(|cut| (position, cut)))
            .collect::<Vec<_>>();
        let cut_line = |position: usize| {
            cut_lines
                .binary_search_by_key(&position, |(at, _)| *at)
                .ok()
                .map(|found| &cut_lines[found].1)
        };
        for (position, mut ferry) in self.ferries {
            let input = &inputs[position];
            let pieces = cut_line(position).map_or(&[][..], |cut| &cut.pieces[..]);
            let length_m = std::iter::once(input)
                .chain(pieces)
                .map(|piece| {
                    piece
                        .geometry
                        .windows(2)
                        .map(|pair| haversine_m(pair[0], pair[1]))
                        .sum::<f64>()
                })
                .sum::<f64>();
            if length_m > 0.0 {
                ferry.facts.maxspeed = Some(format!("{:.1}", length_m / ferry.duration_s * 3.6));
            }
            let encoded = encode_way_facts(input.feature_id, &ferry.facts)?;
            self.way_facts.fill(ferry.slot, &encoded);
        }
        let mut input_report = RoadGraphInputReport::default();
        // The lines of repeated ways that are not kept, by position.
        let dropped_lines = if self.way_ids_unordered {
            let located_count = |position: usize| {
                cut_line(position).map_or(inputs[position].node_chain.len(), |cut| cut.located)
            };
            drop_repeated_ways(
                &mut self.way_facts,
                &self.line_facts,
                located_count,
                &mut input_report,
            )
        } else {
            Vec::new()
        };
        drop(std::mem::take(&mut self.line_facts));
        let is_dropped = |position: usize| dropped_lines.binary_search(&position).is_ok();
        let mut missing_node_way_ids = Vec::new();
        let mut pieces = Vec::new();
        for (position, cut) in cut_lines {
            if is_dropped(position) {
                continue;
            }
            input_report.way_with_missing_nodes_count += 1;
            input_report.missing_node_ref_count += cut.missing;
            input_report.cut_piece_count += cut.pieces.len();
            if inputs[position].geometry.len() < 2 {
                input_report.dropped_missing_node_way_count += 1;
            }
            missing_node_way_ids.push(inputs[position].feature_id);
            pieces.extend(cut.pieces);
        }
        input_report.missing_node_way_id_examples = smallest_ids(missing_node_way_ids);
        let mut position = 0;
        self.builder.retain_inputs(|input| {
            let keep = input.geometry.len() >= 2 && !is_dropped(position);
            position += 1;
            keep
        });
        // After the lines they were cut from: lines of the same way keep the
        // order they were pushed in when the network sorts them.
        self.builder.extend_inputs(pieces);
        Ok(UnfinishedRoadInputs {
            builder: self.builder,
            turn_restrictions: self.turn_restrictions,
            route_memberships: self.route_memberships,
            barrier_node_tags,
            traffic_controls,
            way_facts: self.way_facts,
            input_report,
        })
    }
}

/// `line_facts`' value for a line whose way has no facts.
const NO_FACTS: u32 = u32::MAX;

/// Keeps one copy of each way id that appears more than once: the copy
/// whose line has the most located nodes (a copy that is no line has none),
/// then the earliest in the file. Drops the other copies' facts from
/// `way_facts` and returns the positions of their lines, ascending.
fn drop_repeated_ways(
    way_facts: &mut EncodedWayFacts,
    line_facts: &[u32],
    located_count: impl Fn(usize) -> usize,
    report: &mut RoadGraphInputReport,
) -> Vec<usize> {
    let entries = &way_facts.index;
    let mut line_of_entry = vec![u32::MAX; entries.len()];
    for (line, &entry) in line_facts.iter().enumerate() {
        if entry != NO_FACTS {
            line_of_entry[entry as usize] = line as u32;
        }
    }
    let line_of = |entry: u32| {
        let line = line_of_entry[entry as usize];
        (line != u32::MAX).then_some(line as usize)
    };
    let mut order = (0..entries.len() as u32).collect::<Vec<_>>();
    order.par_sort_unstable_by_key(|&entry| (entries[entry as usize].0, entry));
    let mut dropped_entries = Vec::new();
    let mut dropped_lines = Vec::new();
    let mut repeated_way_ids = Vec::new();
    for copies in order.chunk_by(|&a, &b| entries[a as usize].0 == entries[b as usize].0) {
        if copies.len() < 2 {
            continue;
        }
        let score = |entry: u32| line_of(entry).map_or(0, &located_count);
        // `copies` is in file order: the first of the highest scores wins.
        let kept = copies
            .iter()
            .copied()
            .reduce(|best, entry| {
                if score(entry) > score(best) {
                    entry
                } else {
                    best
                }
            })
            .expect("a repeated way has copies");
        for &entry in copies {
            if entry != kept {
                dropped_entries.push(entry as usize);
                dropped_lines.extend(line_of(entry));
            }
        }
        repeated_way_ids.push(entries[kept as usize].0);
        report.duplicate_way_id_count += 1;
        report.dropped_duplicate_way_count += copies.len() - 1;
    }
    drop(order);
    drop(line_of_entry);
    report.duplicate_way_id_examples = smallest_ids(repeated_way_ids);
    if !dropped_entries.is_empty() {
        dropped_entries.sort_unstable();
        let mut entry = 0;
        way_facts.index.retain(|_| {
            let keep = dropped_entries.binary_search(&entry).is_err();
            entry += 1;
            keep
        });
    }
    dropped_lines.sort_unstable();
    dropped_lines
}

/// The [`RoadGraphInputReport::EXAMPLE_COUNT`] smallest of `ids`, ascending.
fn smallest_ids(mut ids: Vec<i64>) -> Vec<i64> {
    ids.sort_unstable();
    ids.dedup();
    ids.truncate(RoadGraphInputReport::EXAMPLE_COUNT);
    ids
}

/// What pass 2 collected, before the line network is built from it.
struct UnfinishedRoadInputs {
    builder: SharedLineNetworkBuilder,
    turn_restrictions: Vec<RoadTurnRestriction>,
    route_memberships: CollectedRouteMemberships,
    barrier_node_tags: HashMap<i64, NodeBarrierTags>,
    traffic_controls: Vec<TrafficControlPoint>,
    way_facts: EncodedWayFacts,
    input_report: RoadGraphInputReport,
}

impl UnfinishedRoadInputs {
    /// Builds the line network; the way facts wait in `scratch` meanwhile.
    fn finish(mut self, scratch: &Path) -> Result<CollectedRoadInputs, String> {
        self.way_facts.seal();
        self.way_facts.spill(scratch)?;
        // Routing consumes topology only: render products belong exclusively
        // to vector compilation and are never derived here.
        #[cfg(not(feature = "benchmark-profiling"))]
        let (network, topology_edges) = self.builder.finish_topology_batches();
        #[cfg(feature = "benchmark-profiling")]
        let (network, topology_edges) = self
            .builder
            .finish_topology_batches_with_observer(&mut |name| profiling::checkpoint(name));
        Ok(CollectedRoadInputs {
            network,
            topology_edges,
            turn_restrictions: self.turn_restrictions,
            route_memberships: self.route_memberships,
            barrier_node_tags: self.barrier_node_tags,
            traffic_controls: self.traffic_controls,
            way_facts: self.way_facts,
            input_report: self.input_report,
        })
    }
}

impl Pass2Sink for OsmRoadNetworkCollector {
    fn on_relation_batch(&mut self, batch: RelationBatch) -> Result<(), String> {
        for relation in &batch.relations {
            self.turn_restrictions
                .extend(turn_restrictions_from_relation(relation));
            self.route_memberships.push_relation(relation);
        }
        Ok(())
    }

    /// Pass 2 hands batches to this sink on one thread while its decode workers
    /// wait on it, so the per-way work (fact parsing and encoding, attribute
    /// building) runs across the batch in parallel. Only the order-dependent
    /// appends stay sequential, in the batch's order.
    fn on_way_batch(&mut self, batch: WayBatch) -> Result<(), String> {
        // The batch before this one is appended while this one is collected
        // across the pool, so the appends don't hold the decode workers up.
        let previous = std::mem::take(&mut self.collected);
        let (collected, appended) = rayon::join(
            || {
                batch
                    .ways
                    .into_par_iter()
                    .map(collect_way)
                    .collect::<Vec<_>>()
            },
            || self.append(previous),
        );
        self.collected = collected;
        appended
    }

    /// Never called: pass 2 skips the node blobs, and [`locate_nodes`] keeps
    /// the barrier and traffic-control nodes.
    fn on_node_batch(&mut self, _batch: NodeBatch) -> Result<(), String> {
        Ok(())
    }
}

impl OsmRoadNetworkCollector {
    /// Appends one batch's ways, in order: their facts, and their lines to
    /// the builder.
    fn append(&mut self, collected: Vec<Result<CollectedWay, String>>) -> Result<(), String> {
        for way in collected {
            let way = way?;
            if !self.way_ids_unordered && self.last_way_id.is_some_and(|last| way.way_id <= last) {
                self.start_tracking_line_facts();
            }
            self.last_way_id = Some(way.way_id);
            let facts_entry = self.way_facts.index.len();
            if let Some(facts) = way.encoded_facts {
                self.way_facts.insert_encoded(way.way_id, &facts);
                if let Some(node_id) = way.exit_node {
                    self.exit_links.push((facts_entry, node_id));
                }
            }
            if let Some(UnplacedWay { input, ferry }) = way.unplaced {
                if let Some(mut ferry) = ferry {
                    ferry.slot = self.way_facts.reserve(input.feature_id);
                    self.ferries.push((self.pushed, ferry));
                }
                if self.way_ids_unordered {
                    self.line_facts
                        .push(if self.way_facts.index.len() > facts_entry {
                            facts_entry as u32
                        } else {
                            NO_FACTS
                        });
                }
                self.builder.push_input(input);
                self.pushed += 1;
            }
        }
        Ok(())
    }

    /// Records, from now on, where each line's facts are (`line_facts`),
    /// starting with the lines already pushed. Their way ids are ascending,
    /// as are those of the facts so far, each way's at most once, so a binary
    /// search finds them.
    fn start_tracking_line_facts(&mut self) {
        self.way_ids_unordered = true;
        let index = &self.way_facts.index;
        self.line_facts = self
            .builder
            .inputs_mut()
            .iter()
            .map(|input| {
                index
                    .binary_search_by_key(&input.feature_id, |(way_id, _)| *way_id)
                    .map_or(NO_FACTS, |entry| entry as u32)
            })
            .collect();
    }
}

/// One way's pass-2 products, built independently of every other way.
struct CollectedWay {
    way_id: i64,
    /// Bincode-encoded [`RoadWayFacts`], for every highway way but a routable
    /// ferry with a `duration`, whose facts wait in [`UnplacedWay::ferry`].
    encoded_facts: Option<Vec<u8>>,
    /// The routable line, when the way is one, still to be located.
    unplaced: Option<UnplacedWay>,
    /// For a link way with facts and no `junction:ref`, the node it leaves
    /// from ([`link_departure_node`]).
    exit_node: Option<i64>,
}

fn collect_way(way: osm_pbf_ingest::NormalizedWay) -> Result<CollectedWay, String> {
    let facts = (way.tags.contains_key("highway") || is_ferry_line(&way.tags))
        .then(|| road_way_facts_from_tags(way.way_id, &way.tags));
    let encode = |facts: Option<RoadWayFacts>| {
        facts
            .map(|facts| encode_way_facts(way.way_id, &facts))
            .transpose()
    };
    if !is_routable_line(&way.tags, way.is_closed) {
        return Ok(CollectedWay {
            way_id: way.way_id,
            encoded_facts: encode(facts)?,
            unplaced: None,
            exit_node: None,
        });
    }
    // A ferry's `duration` becomes its speed over the whole crossing, carried
    // as the way's maxspeed: vehicle costing reads the edge speed from there.
    // The length comes from the located nodes, so these facts wait for them.
    let ferry_duration_s = facts
        .as_ref()
        .filter(|facts| facts.is_ferry)
        .and_then(|_| way.tags.get("duration").and_then(osm_duration_s));
    let (encoded_facts, ferry) = match (facts, ferry_duration_s) {
        (Some(facts), Some(duration_s)) => (
            None,
            Some(Box::new(PendingFerry {
                facts,
                duration_s,
                slot: usize::MAX,
            })),
        ),
        (facts, _) => (encode(facts)?, None),
    };
    let exit_node = encoded_facts
        .as_ref()
        .and_then(|_| link_departure_node(&way.tags, &way.node_ids));
    let road_class = road_class(&way.tags);
    let attrs = routing_topology_attrs(&way.tags, road_class);
    let routing = road_routing_attrs(&way.tags, road_class);
    Ok(CollectedWay {
        way_id: way.way_id,
        encoded_facts,
        unplaced: Some(UnplacedWay {
            input: SharedLineInput {
                feature_id: way.way_id,
                layer: SharedLineLayer::Roads,
                geometry: Vec::new(),
                node_chain: way.node_ids,
                source_lineage_ids: vec![way.way_id],
                attrs,
                compile_inputs: Arc::from([]),
                min_zoom: road_min_zoom(road_class),
                max_zoom: 14,
                sort_order: road_sort_order(road_class),
                routing: Some(Arc::new(routing)),
            },
            ferry,
        }),
        exit_node,
    })
}

/// The node a link way (`highway=*_link`) with no `junction:ref` leaves
/// from: its first node, or its last for `oneway=-1`. A motorway junction
/// node there numbers the exit the link takes; the far end is where it
/// joins, so an on-ramp ending at a junction node is never an exit.
fn link_departure_node(tags: &(impl PbfTagLookup + ?Sized), node_ids: &[i64]) -> Option<i64> {
    if !tags
        .tag("highway")
        .is_some_and(|value| value.ends_with("_link"))
        || tags
            .tag("junction:ref")
            .is_some_and(|value| !value.trim().is_empty())
    {
        return None;
    }
    if matches!(tags.tag("oneway"), Some("-1" | "reverse")) {
        node_ids.last().copied()
    } else {
        node_ids.first().copied()
    }
}

/// A `highway=motorway_junction` node's exit number (`ref`) and, from
/// `exit_to`, where the exit leads.
struct JunctionExit {
    exit_ref: String,
    exit_to: Option<String>,
}

fn junction_exit_from_tags(tags: &(impl PbfTagLookup + ?Sized)) -> Option<JunctionExit> {
    if tags.tag("highway") != Some("motorway_junction") {
        return None;
    }
    let non_empty = |key: &str| {
        tags.tag(key)
            .map(str::trim)
            .filter(|value| !value.is_empty())
            .map(ToString::to_string)
    };
    Some(JunctionExit {
        exit_ref: non_empty("ref")?,
        exit_to: non_empty("exit_to"),
    })
}

/// Gives each link way that leaves from a motorway junction node, and has
/// no `junction:ref` of its own, the node's `ref` as its `junction_ref`,
/// and its `exit_to` as its `destination` when it has none. The facts are
/// encoded again in the same index entries, in collection order, so the
/// result does not depend on threads or map order.
fn apply_junction_exits(
    way_facts: &mut EncodedWayFacts,
    exit_links: &[(usize, i64)],
    junctions: &HashMap<i64, JunctionExit>,
) -> Result<(), String> {
    if junctions.is_empty() {
        return Ok(());
    }
    for &(entry, node_id) in exit_links {
        let Some(junction) = junctions.get(&node_id) else {
            continue;
        };
        let (way_id, offset) = way_facts.index[entry];
        let (mut facts, _): (RoadWayFacts, _) =
            codec::take_from_slice(&way_facts.bytes()[offset as usize..])
                .map_err(|error| format!("Failed to decode way facts for way {way_id}: {error}"))?;
        facts.junction_ref = Some(junction.exit_ref.clone());
        if facts.destination.is_none() {
            facts.destination.clone_from(&junction.exit_to);
        }
        let encoded = encode_way_facts(way_id, &facts)?;
        way_facts.fill(entry, &encoded);
    }
    Ok(())
}

fn encode_way_facts(way_id: i64, facts: &RoadWayFacts) -> Result<Vec<u8>, String> {
    codec::to_vec(facts)
        .map_err(|error| format!("Failed to encode way facts for way {way_id}: {error}"))
}

/// Gives a line its nodes' positions as its geometry. A node the file does not
/// hold cuts the line there: the line keeps its first stretch of two or more
/// located nodes in a row (or none, and is then dropped), and the later
/// stretches come back as [`CutLine::pieces`]. `None` when every node was
/// located.
fn place_line(locations: &NodeLocations, input: &mut SharedLineInput) -> Option<CutLine> {
    let mut cursor = locations.cursor();
    let mut geometry = Vec::with_capacity(input.node_chain.len());
    // Where the located nodes are cut: the number of located nodes before
    // each missing one.
    let mut cuts = Vec::new();
    let mut missing = 0;
    input
        .node_chain
        .retain(|node_id| match cursor.get(*node_id) {
            Some(position) => {
                geometry.push(position);
                true
            }
            None => {
                missing += 1;
                if cuts.last() != Some(&geometry.len()) {
                    cuts.push(geometry.len());
                }
                false
            }
        });
    if missing == 0 {
        input.geometry = geometry;
        return None;
    }
    let located = geometry.len();
    cuts.push(located);
    let mut stretches = Vec::new();
    let mut start = 0;
    for end in cuts {
        if end - start >= 2 {
            stretches.push(start..end);
        }
        start = end;
    }
    let mut stretches = stretches.into_iter();
    let first = stretches.next();
    let pieces = stretches
        .map(|stretch| SharedLineInput {
            feature_id: input.feature_id,
            layer: input.layer,
            geometry: geometry[stretch.clone()].to_vec(),
            node_chain: input.node_chain[stretch].to_vec(),
            source_lineage_ids: input.source_lineage_ids.clone(),
            attrs: Arc::clone(&input.attrs),
            compile_inputs: Arc::clone(&input.compile_inputs),
            min_zoom: input.min_zoom,
            max_zoom: input.max_zoom,
            sort_order: input.sort_order,
            routing: input.routing.clone(),
        })
        .collect();
    match first {
        Some(first) => {
            input.node_chain.truncate(first.end);
            input.node_chain.drain(..first.start);
            geometry.truncate(first.end);
            geometry.drain(..first.start);
            input.geometry = geometry;
        }
        None => {
            input.node_chain.clear();
            input.geometry = Vec::new();
        }
    }
    Some(CutLine {
        located,
        missing,
        pieces,
    })
}

/// A line [`place_line`] cut at nodes the file does not hold.
struct CutLine {
    /// The line's located nodes, over every stretch.
    located: usize,
    /// Its node refs the file does not hold.
    missing: usize,
    /// Its stretches after the first, as lines of their own.
    pieces: Vec<SharedLineInput>,
}

fn build_road_graph(
    graph_id: &str,
    country_code: Option<&str>,
    segment_order: SegmentOrder,
    inputs: CollectedRoadInputs,
    job: &Job<'_>,
) -> Result<RoadGraphCompileResult, CompileError> {
    let CollectedRoadInputs {
        network,
        topology_edges,
        turn_restrictions,
        route_memberships,
        barrier_node_tags,
        traffic_controls,
        mut way_facts,
        input_report,
    } = inputs;
    let phase = Phase::start("topology_to_graph_builder");
    let topology_repair_report = topology_repair_report(&network);
    let SharedLineTopology { nodes, edges } = network.topology;
    debug_assert!(edges.is_empty(), "the edges come in batches");
    #[cfg_attr(not(test), allow(unused_mut))]
    let mut topology_edges = topology_edges;
    match segment_order {
        SegmentOrder::Topology => {}
        #[cfg(test)]
        SegmentOrder::Shuffled(seed) => {
            let mut edges = topology_edges.into_iter().flatten().collect::<Vec<_>>();
            // Fisher–Yates with a splitmix64 stream.
            let mut state = seed;
            let mut next = || {
                state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
                let mut z = state;
                z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
                z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
                z ^ (z >> 31)
            };
            for index in (1..edges.len()).rev() {
                edges.swap(index, (next() % (index as u64 + 1)) as usize);
            }
            topology_edges = vec![edges];
        }
    }
    // Topology node ids are small and dense, so they index a vector; a later
    // node with the same id wins, as a collected map's insert did.
    let mut node_source_ids = Vec::<Option<Option<i64>>>::new();
    for node in &nodes {
        let slot = node.id as usize;
        if node_source_ids.len() <= slot {
            node_source_ids.resize(slot + 1, None);
        }
        node_source_ids[slot] = Some(node.source_node_id);
    }
    let source_node_id_of = |node_id: u32| {
        node_source_ids
            .get(node_id as usize)
            .copied()
            .flatten()
            .flatten()
    };
    let mut builder =
        RoadGraphBuilder::new(graph_id).with_country_code(country_code.map(str::to_string));
    let mut tags = TagInterner::new();
    drop(nodes);
    // A segment's way attributes come from its tag list and routing values,
    // both shared handles (`SharedLineNetworkBuilder` interns them), so each
    // distinct pair becomes a builder template once. The handles are kept
    // with their template so their addresses stay theirs.
    let mut templates = FxHashMap::<(usize, usize), (WayTemplate, SegmentHandles)>::default();
    let mut rows_written = 0;
    // The road segments (as `SharedLineTopology::road_segments` picks them),
    // consumed a batch at a time: each segment's geometry moves to the
    // builder, and the rest of it, and each batch's vector, is freed as the
    // builder grows.
    for mut batch in topology_edges {
        batch.retain(|edge| edge.layer == SharedLineLayer::Roads && edge.routing.is_some());
        let segment_templates = batch
            .iter()
            .map(|segment| {
                let routing = segment.routing.as_ref().expect("a road segment's routing");
                let key = (
                    Arc::as_ptr(&segment.attrs) as *const () as usize,
                    Arc::as_ptr(routing) as usize,
                );
                let (template, _) = templates.entry(key).or_insert_with(|| {
                    let template = builder.way_template(segment_way_template(
                        &segment.attrs,
                        routing,
                        &mut tags,
                    ));
                    (template, (Arc::clone(&segment.attrs), Arc::clone(routing)))
                });
                *template
            })
            .collect::<Vec<_>>();
        let ways = batch
            .into_par_iter()
            .zip(segment_templates)
            .map(|(segment, template)| {
                let start_source_node_id = source_node_id_of(segment.start_node);
                let end_source_node_id = source_node_id_of(segment.end_node);
                template_way_from_segment(
                    segment,
                    template,
                    start_source_node_id,
                    end_source_node_id,
                )
            })
            .collect::<Vec<_>>();
        rows_written += ways.len();
        builder.push_template_ways(ways);
    }
    drop(templates);
    drop(node_source_ids);
    drop(phase);

    report_finalize_checkpoint(job, "before_builder_finish", rows_written)?;
    let phase = Phase::start("graph_builder_finish");
    builder.set_barrier_node_tags(barrier_node_tags);
    builder.set_traffic_control_points(traffic_controls);
    let (mut road_graph, builder_report) = builder.finish_with_report();
    drop(phase);
    report_finalize_checkpoint(job, "after_builder_finish", rows_written)?;

    let phase = Phase::start("graph_source_enrichment");
    // The graph's ways are those before the facts apply; they filter the
    // memberships and restrictions first, so their list is gone before the
    // facts' classes are built. Only road routes' memberships reach the
    // graph; all of them count.
    let graph_way_ids = graph_way_ids(&road_graph);
    let (route_membership_count, route_memberships) = route_memberships.finish(&graph_way_ids);
    let turn_restrictions = filtered_turn_restrictions(turn_restrictions, &graph_way_ids);
    drop(graph_way_ids);
    way_facts.restore().map_err(CompileError::Failed)?;
    road_graph.refresh_shared_edge_way_facts_par_with(|way_id| way_facts.get(way_id))?;
    // Not read again.
    drop(way_facts);
    report_finalize_checkpoint(job, "after_apply_way_facts", rows_written)?;
    road_graph.set_route_memberships(route_memberships);
    report_finalize_checkpoint(job, "after_load_route_memberships", rows_written)?;
    road_graph.set_turn_restrictions(turn_restrictions);
    report_finalize_checkpoint(job, "after_load_turn_restrictions", rows_written)?;
    road_graph.refresh_route_membership_metadata();
    let raw_turn_restriction_count = road_graph.turn_restrictions.len();
    report_finalize_checkpoint(job, "after_refresh_shared_edge_metadata", rows_written)?;
    drop(phase);
    let phase = Phase::start("resolve_turn_restrictions");
    let (resolved, restriction_report) = resolve_turn_restrictions_with_report(&road_graph);
    road_graph.set_resolved_restrictions(resolved);
    drop(phase);
    report_finalize_checkpoint(job, "after_resolve_turn_restrictions", rows_written)?;
    let phase = Phase::start("discard_source_enrichment");
    road_graph.discard_source_enrichment_inputs();
    drop(phase);
    report_finalize_checkpoint(job, "after_discard_source_enrichment_inputs", rows_written)?;
    let phase = Phase::start("graph_enhancement");
    // Packages store no per-node road-type tallies, so none are derived.
    let enhancement_report = enhance_road_graph_with(
        &mut road_graph,
        EnhanceOptions {
            node_road_type_counts: false,
        },
    );
    drop(phase);
    // The package's ids, from the graph rather than from the order it was
    // built in (see `package_build::renumber_in_package_order`).
    let phase = Phase::start("renumber_in_package_order");
    package_build::renumber_in_package_order(&mut road_graph)
        .map_err(|error| CompileError::Failed(error.to_string()))?;
    drop(phase);
    report_finalize_checkpoint(job, "after_enhance_road_graph", rows_written)?;

    let mut quality = RoadGraphQualityManifest {
        builder_report,
        topology_repair_report,
        restriction_report,
        enhancement_report,
        input_report,
        ..Default::default()
    };
    quality.reasons = graph_quality_warning_reasons(&quality);
    Ok(RoadGraphCompileResult {
        road_graph,
        road_graph_quality: quality,
        raw_turn_restriction_count,
        route_membership_count,
    })
}

/// The road graph build's checkpoints, in order (the worker protocol's
/// `graph_progress` messages).
const FINALIZE_CHECKPOINTS: [&str; 10] = [
    "before_line_network_finish",
    "before_builder_finish",
    "after_builder_finish",
    "after_apply_way_facts",
    "after_load_route_memberships",
    "after_load_turn_restrictions",
    "after_refresh_shared_edge_metadata",
    "after_resolve_turn_restrictions",
    "after_discard_source_enrichment_inputs",
    "after_enhance_road_graph",
];

/// Reports a checkpoint of the graph build and checks for a cancel.
fn report_finalize_checkpoint(
    job: &Job<'_>,
    checkpoint: &'static str,
    rows_written: usize,
) -> Result<(), job::Cancelled> {
    let step = FINALIZE_CHECKPOINTS
        .iter()
        .position(|known| *known == checkpoint);
    debug_assert!(step.is_some(), "unknown checkpoint {checkpoint}");
    job.report(
        CompileStage::FinalizeGraph,
        step.map(|step| step as f64 / (FINALIZE_CHECKPOINTS.len() - 1) as f64),
        rows_written,
        checkpoint,
    );
    job.check()
}

fn topology_repair_report(network: &SharedLineNetwork) -> RoadGraphTopologyRepairReport {
    let stats = &network.stats;
    RoadGraphTopologyRepairReport {
        input_road_line_count: stats.topology_repair_input_road_lines,
        checked_segment_pair_count: stats.topology_repair_checked_segment_pairs,
        repaired_crossing_count: stats.topology_repair_repaired_crossings,
        inserted_topology_node_count: stats.topology_repair_inserted_topology_nodes,
        split_edge_count: stats.topology_repair_split_edges,
        skipped_endpoint_touch_count: stats.topology_repair_skipped_endpoint_touches,
        skipped_same_feature_count: stats.topology_repair_skipped_same_feature,
        skipped_incompatible_plane_count: stats.topology_repair_skipped_incompatible_plane,
    }
}

fn graph_quality_warning_reasons(quality: &RoadGraphQualityManifest) -> Vec<String> {
    let mut reasons = Vec::new();
    if quality.topology_repair_report.repaired_crossing_count > 0 {
        reasons.push(format!(
            "repaired {} same-plane road crossings",
            quality.topology_repair_report.repaired_crossing_count
        ));
    }
    if quality.restriction_report.unresolved_restriction_count > 0 {
        reasons.push(format!(
            "{} turn restrictions were not resolved",
            quality.restriction_report.unresolved_restriction_count
        ));
    }
    if !quality.enhancement_report.warning_examples.is_empty() {
        reasons.push(format!(
            "{} road graph enhancement warning examples were recorded",
            quality.enhancement_report.warning_examples.len()
        ));
    }
    reasons.extend(quality.input_report.reasons());
    reasons
}

/// Every way id the graph carries for some mode, sorted and distinct: the
/// ways kept for SAR only (closed to every mode) name no restriction or
/// route the legal network resolves, as before they were kept.
fn graph_way_ids(graph: &RoadGraph) -> Vec<i64> {
    // The lineage of every shared info a legal edge uses.
    let info_index = SharedInfoIndex::new(&graph.shared_edge_infos);
    let mut legal_info = vec![false; graph.shared_edge_infos.len()];
    for edge in 0..graph.graph.edge_count() {
        if graph.graph.edge_direction_osm_mode_mask(edge) != 0
            && let Some(index) = info_index.get(graph.graph.edge_shared_info_id(edge))
        {
            legal_info[index] = true;
        }
    }
    let mut way_ids = graph
        .shared_edge_infos
        .iter()
        .filter(|info| info.attrs.osm_forward_mode_mask | info.attrs.osm_reverse_mode_mask != 0)
        .map(|info| info.osm_way_id)
        .chain(
            graph
                .shared_edge_infos
                .iter()
                .zip(legal_info)
                .filter(|(_, legal)| *legal)
                .flat_map(|(info, _)| info.source_lineage_ids.iter().copied()),
        )
        .collect::<Vec<_>>();
    way_ids.par_sort_unstable();
    way_ids.dedup();
    way_ids
}

fn filtered_turn_restrictions(
    restrictions: Vec<RoadTurnRestriction>,
    graph_way_ids: &[i64],
) -> Vec<RoadTurnRestriction> {
    let mut restrictions = restrictions
        .into_iter()
        .filter(|restriction| {
            graph_way_ids
                .binary_search(&restriction.from_way_id)
                .is_ok()
                || graph_way_ids.binary_search(&restriction.to_way_id).is_ok()
        })
        .collect::<Vec<_>>();
    restrictions.sort_by_key(|restriction| restriction.relation_id);
    restrictions
}

/// The modes a `restriction:<mode>` key may name.
const RESTRICTION_MODES: &[&str] = &[
    "bicycle",
    "bus",
    "emergency",
    "foot",
    "goods",
    "hgv",
    "motor_vehicle",
    "motorcar",
    "psv",
    "vehicle",
];

/// A `type=restriction` relation's rows: one per `from`/`to` way pair (several
/// `from` ways make a `no_entry`, several `to` ways a `no_exit`) and per value,
/// the plain `restriction` and each `restriction:<mode>`. A mode with a value
/// of its own is excepted from the plain one. The via is one node, one or more
/// ways (in member order), or a relation; any other mix is malformed.
fn turn_restrictions_from_relation(relation: &NormalizedRelation) -> Vec<RoadTurnRestriction> {
    if relation.tags.get("type") != Some("restriction") {
        return Vec::new();
    }
    let value = |key: &str| {
        relation
            .tags
            .get(key)
            .map(|value| value.trim())
            .filter(|value| !value.is_empty())
            .map(str::to_string)
    };
    let mut except_modes = value("except")
        .map(|value| {
            value
                .split(';')
                .map(str::trim)
                .filter(|value| !value.is_empty())
                .map(str::to_string)
                .collect::<Vec<_>>()
        })
        .unwrap_or_default();
    let mode_values = RESTRICTION_MODES
        .iter()
        .filter_map(|mode| Some((*mode, value(&format!("restriction:{mode}"))?)))
        .collect::<Vec<_>>();
    let mut values = mode_values
        .iter()
        .map(|(mode, restriction)| {
            (
                restriction.clone(),
                vec![mode.to_string()],
                except_modes.clone(),
            )
        })
        .collect::<Vec<_>>();
    if let Some(restriction) = value("restriction") {
        except_modes.extend(mode_values.iter().map(|(mode, _)| mode.to_string()));
        values.insert(0, (restriction, Vec::new(), except_modes));
    }

    let mut from_way_ids = Vec::new();
    let mut to_way_ids = Vec::new();
    let mut via_nodes = Vec::new();
    let mut via_ways = Vec::new();
    let mut via_relations = Vec::new();
    for member in &relation.members {
        match (member.role.as_str(), member.member_type) {
            ("from", RelationMemberType::Way) => from_way_ids.push(member.member_id),
            ("to", RelationMemberType::Way) => to_way_ids.push(member.member_id),
            ("via", RelationMemberType::Node) => via_nodes.push(member.member_id),
            ("via", RelationMemberType::Way) => via_ways.push(member.member_id),
            ("via", RelationMemberType::Relation) => via_relations.push(member.member_id),
            _ => {}
        }
    }
    let (via_member_type, via_member_id, via_way_ids) =
        match (&via_nodes[..], &via_ways[..], &via_relations[..]) {
            ([node], [], []) => (RelationMemberRefType::Node, *node, Vec::new()),
            ([], [first, ..], []) => (RelationMemberRefType::Way, *first, via_ways.clone()),
            ([], [], [first, ..]) => (RelationMemberRefType::Relation, *first, Vec::new()),
            _ => return Vec::new(),
        };

    let mut restrictions = Vec::new();
    for (restriction, modes, except_modes) in values {
        for &from_way_id in &from_way_ids {
            for &to_way_id in &to_way_ids {
                restrictions.push(RoadTurnRestriction {
                    relation_id: relation.relation_id,
                    restriction: restriction.clone(),
                    from_way_id,
                    to_way_id,
                    via_member_type,
                    via_member_id,
                    via_way_ids: via_way_ids.clone(),
                    modes: modes.clone(),
                    except_modes: except_modes.clone(),
                });
            }
        }
    }
    restrictions
}

fn barrier_tag(tags: &(impl PbfTagLookup + ?Sized)) -> Option<&str> {
    tags.tag("barrier")
        .filter(|value| !value.is_empty() && *value != "no")
}

/// The nodes [`barrier_node_tags_from_tags`] reads.
fn is_barrier_node(tags: &(impl PbfTagLookup + ?Sized)) -> bool {
    barrier_tag(tags).is_some()
}

/// The nodes the ingest keeps: barriers, traffic controls and numbered
/// motorway junctions.
fn is_kept_node(tags: &(impl PbfTagLookup + ?Sized)) -> bool {
    is_barrier_node(tags)
        || traffic_control_from_tags(tags).is_some()
        || junction_exit_from_tags(tags).is_some()
}

/// The traffic control an OSM node's tags describe: `highway=traffic_signals`
/// (signals of a pedestrian crossing when it also carries `crossing=*` or
/// `traffic_signals=crossing`), `highway=crossing` with
/// `crossing=traffic_signals` or `crossing:signals=yes`, `highway=stop` and
/// `highway=give_way`; facing `traffic_signals:direction` or `direction`
/// when that is `forward` or `backward`.
fn traffic_control_from_tags(
    tags: &(impl PbfTagLookup + ?Sized),
) -> Option<(TrafficControl, TrafficControlDirection)> {
    let tag = |key: &str| tags.tag(key);
    let control = match tag("highway")? {
        "traffic_signals"
            if tag("crossing").is_some_and(|value| value != "no")
                || tag("traffic_signals") == Some("crossing") =>
        {
            TrafficControl::CrossingSignals
        }
        "traffic_signals" => TrafficControl::TrafficSignals,
        "crossing"
            if tag("crossing") == Some("traffic_signals")
                || tag("crossing:signals") == Some("yes") =>
        {
            TrafficControl::CrossingSignals
        }
        "stop" => TrafficControl::Stop,
        "give_way" => TrafficControl::GiveWay,
        _ => return None,
    };
    let direction_key = if matches!(
        control,
        TrafficControl::TrafficSignals | TrafficControl::CrossingSignals
    ) {
        tag("traffic_signals:direction").or_else(|| tag("direction"))
    } else {
        tag("direction")
    };
    let direction = match direction_key {
        Some("forward") => TrafficControlDirection::Forward,
        Some("backward") => TrafficControlDirection::Backward,
        _ => TrafficControlDirection::Both,
    };
    Some((control, direction))
}

fn traffic_control_point(node: &osm_pbf_ingest::NormalizedNode) -> Option<TrafficControlPoint> {
    let (control, direction) = traffic_control_from_tags(&node.tags)?;
    Some(TrafficControlPoint {
        lat: node.lat,
        lng: node.lng,
        control,
        direction,
    })
}

/// The ways `collect_way` reads: every other way yields neither facts nor a
/// routable line.
fn is_road_way(tags: &(impl PbfTagLookup + ?Sized)) -> bool {
    tags.has_tag("highway") || is_ferry_line(tags)
}

fn barrier_node_tags_from_tags(tags: &Tags) -> Option<NodeBarrierTags> {
    let barrier = barrier_tag(tags)?;
    Some(NodeBarrierTags {
        barrier: barrier.to_string(),
        access: tag_owned(tags, "access"),
        foot: tag_owned(tags, "foot"),
        bicycle: tag_owned(tags, "bicycle"),
        vehicle: tag_owned(tags, "vehicle"),
        motor_vehicle: tag_owned(tags, "motor_vehicle"),
        locked: tag_owned(tags, "locked"),
    })
}

fn road_way_facts_from_tags(way_id: i64, tags: &Tags) -> RoadWayFacts {
    let highway = tag_owned(tags, "highway");
    RoadWayFacts {
        osm_way_id: way_id,
        road_type: RoadType::from_highway(highway.as_deref()),
        directional_access: directional_access_from_tags(tags),
        name: tag_owned(tags, "name"),
        ref_name: tag_owned(tags, "ref"),
        highway: highway.clone(),
        access: tag_owned(tags, "access"),
        vehicle: tag_owned(tags, "vehicle"),
        motor_vehicle: tag_owned(tags, "motor_vehicle"),
        motorcar: tag_owned(tags, "motorcar"),
        bicycle: tag_owned(tags, "bicycle"),
        foot: tag_owned(tags, "foot"),
        oneway_foot: tag_owned(tags, "oneway:foot"),
        oneway_bicycle: tag_owned(tags, "oneway:bicycle"),
        motorroad: tag_owned(tags, "motorroad"),
        service: tag_owned(tags, "service"),
        surface: tag_owned(tags, "surface"),
        tracktype: tag_owned(tags, "tracktype"),
        smoothness: tag_owned(tags, "smoothness"),
        maxspeed: tag_owned(tags, "maxspeed"),
        maxspeed_forward: tag_owned(tags, "maxspeed:forward"),
        maxspeed_backward: tag_owned(tags, "maxspeed:backward"),
        maxspeed_hgv: tag_owned(tags, "maxspeed:hgv"),
        maxheight: tag_owned(tags, "maxheight"),
        maxwidth: tag_owned(tags, "maxwidth"),
        // R5's width classes read `est_width` where `width` is missing.
        width: tag_owned(tags, "width").or_else(|| tag_owned(tags, "est_width")),
        maxweight: tag_owned(tags, "maxweight"),
        maxaxleload: tag_owned(tags, "maxaxleload"),
        incline: tag_owned(tags, "incline"),
        sac_scale: tag_owned(tags, "sac_scale"),
        mtb_scale: tag_owned(tags, "mtb:scale"),
        trail_visibility: tag_owned(tags, "trail_visibility"),
        sidewalk: tag_owned(tags, "sidewalk"),
        crossing: tag_owned(tags, "crossing"),
        is_4wd_only: tag_is_present(tags, "4wd_only"),
        is_lit: tag_is_present(tags, "lit"),
        lanes: tag_owned(tags, "lanes"),
        turn_lanes: tag_owned(tags, "turn:lanes"),
        turn_lanes_forward: tag_owned(tags, "turn:lanes:forward"),
        turn_lanes_backward: tag_owned(tags, "turn:lanes:backward"),
        lane_connectivity_available: tags.contains_key("turn:lanes")
            || tags.contains_key("turn:lanes:forward")
            || tags.contains_key("turn:lanes:backward"),
        admin_country_id: tags
            .get("addr:country")
            .or_else(|| tags.get("is_in:country_code"))
            .map(ToString::to_string),
        admin_region_id: tags
            .get("addr:region")
            .or_else(|| tags.get("is_in:state"))
            .map(ToString::to_string),
        timezone_id: tags
            .get("timezone")
            .or_else(|| tags.get("tz"))
            .map(ToString::to_string),
        layer: tag_owned(tags, "layer"),
        level: tag_owned(tags, "level"),
        destination: tag_owned(tags, "destination"),
        destination_ref: tag_owned(tags, "destination:ref"),
        destination_street: tag_owned(tags, "destination:street"),
        junction_ref: tag_owned(tags, "junction:ref"),
        conditional_restrictions: conditional_restrictions_from_tags(tags),
        is_toll: tag_is_present(tags, "toll"),
        is_ferry: is_ferry_line(tags),
        has_gate: tag_is_gate(tags),
        has_bridge: tag_is_present(tags, "bridge"),
        has_tunnel: tag_is_present(tags, "tunnel"),
        has_ford: tag_is_present(tags, "ford"),
        is_roundabout: matches!(tags.get("junction"), Some("roundabout" | "circular")),
        is_link: highway
            .as_deref()
            .is_some_and(|value| value.ends_with("_link")),
        is_destination_only: vehicle_access_is_destination_only(
            tags.get("motorcar"),
            tags.get("motor_vehicle"),
            tags.get("vehicle"),
            tags.get("access"),
        ),
        access_restricted: matches!(
            tags.get("access"),
            Some("private" | "no" | "permit" | "customers" | "military")
        ),
        is_access_restricted: matches!(
            tags.get("access"),
            Some("private" | "no" | "permit" | "customers" | "military")
        ),
    }
}

/// The way's vehicular direction. Roundabouts and motorways (with their links)
/// are one-way unless tagged otherwise. `oneway=reversible` changes direction
/// with the time of day, so with no schedule to follow it is closed, as OSRM
/// closes it; `oneway=alternating` (one lane, directions taking turns) is
/// passable both ways and stays two-way.
fn directional_access_from_tags(tags: &Tags) -> DirectionalAccess {
    match tags.get("oneway") {
        Some("yes" | "true" | "1") => DirectionalAccess::Forward,
        Some("-1" | "reverse") => DirectionalAccess::Backward,
        Some("no" | "false" | "0" | "alternating") => DirectionalAccess::Both,
        Some("reversible") => DirectionalAccess::Closed,
        _ if matches!(tags.get("junction"), Some("roundabout" | "circular"))
            || matches!(tags.get("highway"), Some("motorway" | "motorway_link")) =>
        {
            DirectionalAccess::Forward
        }
        _ => DirectionalAccess::Both,
    }
}

fn tag_owned(tags: &Tags, key: &str) -> Option<String> {
    tags.get(key).map(ToString::to_string)
}

/// The handles a segment's way template is derived from (see
/// [`segment_way_template`]).
type SegmentHandles = (Arc<[(String, String)]>, Arc<RoadRoutingAttrs>);

/// A road segment's way attributes, from its topology attributes and routing
/// values: the [`RoutableWay`] the builder takes as a template, without the
/// segment's id, lineage, points and source nodes.
fn segment_way_template(
    segment_attrs: &[(String, String)],
    routing: &RoadRoutingAttrs,
    tags: &mut TagInterner,
) -> RoutableWay {
    // Routing topology carries only a handful of attrs; a linear lookup avoids
    // building a string map per segment for mostly-missing keys.
    let attrs = SegmentAttrs(segment_attrs);
    let attr = |key: &str| attrs.value(key);
    let road_type = match routing.road_class.as_str() {
        "motorway" | "trunk" => RoadType::Motorway,
        "primary" => RoadType::Primary,
        "secondary" => RoadType::Secondary,
        "tertiary" => RoadType::Tertiary,
        "residential" | "living_street" | "service" | "unclassified" => RoadType::Residential,
        "track" => RoadType::Track,
        "path" | "footway" | "cycleway" | "bridleway" | "pedestrian" | "steps" => RoadType::Path,
        _ => RoadType::Other,
    };

    RoutableWay {
        road_type,
        directional_access: match routing.directional_access {
            DirectionalAccessModel::Both => DirectionalAccess::Both,
            DirectionalAccessModel::Forward => DirectionalAccess::Forward,
            DirectionalAccessModel::Backward => DirectionalAccess::Backward,
            DirectionalAccessModel::Closed => DirectionalAccess::Closed,
        },
        name: tags.intern_opt(attr("name")),
        ref_name: tags.intern_opt(attr("ref")),
        highway: tags.intern_opt(attr("highway")),
        access: tags.intern_opt(routing.access.as_deref()),
        vehicle: tags.intern_opt(attr("vehicle")),
        motor_vehicle: tags.intern_opt(attr("motor_vehicle")),
        motorcar: tags.intern_opt(attr("motorcar")),
        bicycle: tags.intern_opt(attr("bicycle")),
        foot: tags.intern_opt(attr("foot")),
        oneway_foot: tags.intern_opt(attr("oneway:foot")),
        oneway_bicycle: tags.intern_opt(attr("oneway:bicycle")),
        motorroad: tags.intern_opt(attr("motorroad")),
        service: tags.intern_opt(attr("service").or_else(|| attr("subclass"))),
        surface: tags.intern_opt(routing.surface.as_deref()),
        tracktype: tags.intern_opt(routing.tracktype.as_deref()),
        smoothness: tags.intern_opt(routing.smoothness.as_deref()),
        maxspeed: tags.intern_opt(attr("maxspeed")),
        maxspeed_forward: tags.intern_opt(attr("maxspeed:forward")),
        maxspeed_backward: tags.intern_opt(attr("maxspeed:backward")),
        maxspeed_hgv: tags.intern_opt(attr("maxspeed:hgv")),
        maxheight: tags.intern_opt(attr("maxheight")),
        maxwidth: tags.intern_opt(attr("maxwidth")),
        width: tags.intern_opt(attr("width").or_else(|| attr("est_width"))),
        maxweight: tags.intern_opt(attr("maxweight")),
        maxaxleload: tags.intern_opt(attr("maxaxleload")),
        incline: tags.intern_opt(attr("incline")),
        sac_scale: tags.intern_opt(attr("sac_scale")),
        mtb_scale: tags.intern_opt(attr("mtb:scale")),
        trail_visibility: tags.intern_opt(attr("trail_visibility")),
        sidewalk: tags.intern_opt(attr("sidewalk")),
        crossing: tags.intern_opt(attr("crossing")),
        is_4wd_only: tag_is_truthy(&attrs, "4wd_only"),
        is_lit: tag_is_truthy(&attrs, "lit"),
        lanes: tags.intern_opt(attr("lanes")),
        layer: tags.intern_opt(attr("layer")),
        level: tags.intern_opt(attr("level")),
        destination: tags.intern_opt(attr("destination")),
        destination_ref: tags.intern_opt(attr("destination:ref")),
        destination_street: tags.intern_opt(attr("destination:street")),
        junction_ref: tags.intern_opt(attr("junction:ref")),
        // Topology attrs carry no `*:conditional` tags; the way facts do, and
        // they replace this when the graph's shared infos are enriched.
        conditional_restrictions: Vec::new(),
        area: tags.intern_opt(attr("area")),
        area_highway: tags.intern_opt(attr("area:highway")),
        is_toll: tag_is_truthy(&attrs, "toll"),
        is_ferry: matches!(attr("route"), Some("ferry" | "shuttle_train")),
        has_gate: tag_is_gate(&attrs),
        has_bridge: tag_is_present(&attrs, "bridge"),
        has_ford: routing.has_ford,
        has_tunnel: routing.has_tunnel,
        is_roundabout: matches!(attr("junction"), Some("roundabout" | "circular")),
        is_link: routing.road_class.ends_with("_link")
            || attr("highway").is_some_and(|value| value.ends_with("_link")),
        is_destination_only: vehicle_access_is_destination_only(
            attr("motorcar"),
            attr("motor_vehicle"),
            attr("vehicle"),
            attr("access").or(routing.access.as_deref()),
        ),
        access_restricted: routing.access_restricted,
        is_access_restricted: routing.access_restricted,
        coverage_id: tags.intern_opt(
            segment_attrs
                .iter()
                .find(|(key, _)| key == "coverage_id")
                .map(|(_, value)| value.as_str()),
        ),
        ..Default::default()
    }
}

/// A road segment as the builder takes it with its way `template`: its id,
/// lineage and points, and its end nodes' source nodes.
fn template_way_from_segment(
    segment: SharedTopologyEdge,
    template: WayTemplate,
    start_source_node_id: Option<i64>,
    end_source_node_id: Option<i64>,
) -> TemplateWay {
    let last = segment.geometry.len().saturating_sub(1) as u32;
    let mut source_node_ids = Vec::new();
    source_node_ids.extend(start_source_node_id.map(|node_id| (0, node_id)));
    if last > 0 {
        source_node_ids.extend(end_source_node_id.map(|node_id| (last, node_id)));
    }
    TemplateWay {
        template,
        osm_way_id: segment.feature_id,
        points: segment.geometry,
        source_node_ids,
        source_lineage_ids: segment.source_lineage_ids.to_vec(),
    }
}

trait TagLookup {
    fn value(&self, key: &str) -> Option<&str>;
}

impl TagLookup for Tags {
    fn value(&self, key: &str) -> Option<&str> {
        self.get(key)
    }
}

/// A topology segment's sorted attribute pairs. Lookups take the last pair
/// for a key, matching the previous `HashMap` collection (last insert wins).
struct SegmentAttrs<'a>(&'a [(String, String)]);

impl TagLookup for SegmentAttrs<'_> {
    fn value(&self, key: &str) -> Option<&str> {
        self.0
            .iter()
            .rev()
            .find(|(candidate, _)| candidate == key)
            .map(|(_, value)| value.as_str())
    }
}

impl TagLookup for HashMap<String, String> {
    fn value(&self, key: &str) -> Option<&str> {
        self.get(key).map(|value| value.as_str())
    }
}

fn tag_is_present(tags: &impl TagLookup, key: &str) -> bool {
    tags.value(key)
        .is_some_and(|value| !value.is_empty() && value != "no")
}

fn tag_is_truthy(tags: &impl TagLookup, key: &str) -> bool {
    tags.value(key)
        .is_some_and(|value| !value.is_empty() && value != "no" && value != "false" && value != "0")
}

fn tag_is_gate(tags: &impl TagLookup) -> bool {
    matches!(
        tags.value("barrier"),
        Some("gate" | "lift_gate" | "swing_gate" | "bollard" | "block" | "stile" | "kissing_gate")
    )
}

/// `key=value` for every `*:conditional` tag, sorted and deduplicated, e.g.
/// `access:conditional=no @ (Mo-Fr 07:00-09:00)`. The value keeps its
/// condition so costing can evaluate it at the request's time.
fn conditional_restrictions_from_tags(tags: &Tags) -> Vec<String> {
    let mut values = tags
        .iter()
        .filter_map(|(key, value)| {
            let key = key.trim();
            let value = value.trim();
            (!key.is_empty() && !value.is_empty() && key.ends_with(":conditional"))
                .then(|| format!("{key}={value}"))
        })
        .collect::<Vec<_>>();
    values.sort();
    values.dedup();
    values
}

/// Highways, ferries and shuttle trains, but not the outlines of highway
/// areas, except pedestrian squares: the graph walks their outline and
/// synthesizes paths across them.
fn is_routable_line(tags: &Tags, is_closed: bool) -> bool {
    if is_ferry_line(tags) && !tags.contains_key("highway") {
        return true;
    }
    let is_area =
        is_closed && (matches!(tags.get("area"), Some("yes")) || tags.contains_key("area:highway"));
    let is_pedestrian_square = !tags.contains_key("area:highway")
        && matches!(tags.get("highway"), Some("pedestrian" | "footway"));
    tags.contains_key("highway") && (!is_area || is_pedestrian_square)
}

fn is_ferry_line(tags: &(impl PbfTagLookup + ?Sized)) -> bool {
    matches!(tags.tag("route"), Some("ferry" | "shuttle_train"))
}

/// An OSM `duration` in seconds: `mm`, `hh:mm`, `hh:mm:ss` or ISO 8601
/// (`PT1H30M`, `P1DT2H`).
fn osm_duration_s(value: &str) -> Option<f64> {
    let value = value.trim();
    let seconds = if let Some(iso) = value.strip_prefix('P') {
        let mut seconds = 0.0;
        let mut number = String::new();
        let mut in_time = false;
        for character in iso.chars() {
            match character {
                'T' => in_time = true,
                '0'..='9' | '.' => number.push(character),
                unit => {
                    let amount = number.parse::<f64>().ok()?;
                    number.clear();
                    seconds += amount
                        * match (unit, in_time) {
                            ('D', false) => 86_400.0,
                            ('H', true) => 3_600.0,
                            ('M', true) => 60.0,
                            ('S', true) => 1.0,
                            _ => return None,
                        };
                }
            }
        }
        number.is_empty().then_some(seconds)?
    } else {
        let parts = value
            .split(':')
            .map(|part| part.trim().parse::<f64>().ok())
            .collect::<Option<Vec<_>>>()?;
        match parts[..] {
            [minutes] => minutes * 60.0,
            [hours, minutes] => hours * 3_600.0 + minutes * 60.0,
            [hours, minutes, seconds] => hours * 3_600.0 + minutes * 60.0 + seconds,
            _ => return None,
        }
    };
    (seconds.is_finite() && seconds > 0.0).then_some(seconds)
}

fn haversine_m(from: (f64, f64), to: (f64, f64)) -> f64 {
    math::haversine_m(from, to, wayfarer_package::format::EARTH_RADIUS_M)
}

fn road_class(tags: &Tags) -> &'static str {
    match tags.get("highway") {
        Some("motorway" | "motorway_link") => "motorway",
        Some("trunk" | "trunk_link") => "trunk",
        Some("primary" | "primary_link") => "primary",
        Some("secondary" | "secondary_link") => "secondary",
        Some("tertiary" | "tertiary_link") => "tertiary",
        Some("unclassified") => "unclassified",
        Some("residential") => "residential",
        Some("living_street") => "living_street",
        Some("service") => "service",
        Some("track") => "track",
        Some("path") => "path",
        Some("footway") => "footway",
        Some("cycleway") => "cycleway",
        Some("bridleway") => "bridleway",
        Some("pedestrian") => "pedestrian",
        Some("steps") => "steps",
        _ => "other",
    }
}

fn road_min_zoom(class: &str) -> u8 {
    match class {
        "motorway" | "trunk" => 3,
        "primary" => 4,
        "secondary" => 5,
        "tertiary" => 6,
        "unclassified" | "residential" | "living_street" => 12,
        "steps" => 14,
        _ => 13,
    }
}

fn road_sort_order(class: &str) -> u8 {
    match class {
        "motorway" => 240,
        "trunk" => 230,
        "primary" => 220,
        "secondary" => 200,
        "tertiary" => 180,
        "unclassified" => 170,
        "residential" => 160,
        "living_street" => 150,
        "service" => 140,
        "track" => 120,
        "path" => 110,
        "footway" => 105,
        "cycleway" => 104,
        "bridleway" => 103,
        "pedestrian" => 102,
        "steps" => 101,
        _ => 80,
    }
}

/// The way tags the topology and the graph builder read before the way facts
/// are applied: the plane tags, which keep crossing repair from joining a
/// bridge or tunnel to the road it passes over or under, and the tags the
/// builder derives mode masks and pedestrian squares from, so a way is kept
/// or dropped on the same tags its facts carry.
const TOPOLOGY_TAG_KEYS: &[&str] = &[
    "area",
    "bicycle",
    "foot",
    "highway",
    "layer",
    "motor_vehicle",
    "motorcar",
    "motorroad",
    "oneway:bicycle",
    "oneway:foot",
    "route",
    "vehicle",
];

// These are topology keys, not a vector-tile schema. `class` preserves stable
// ordering while the source store remains authoritative for complete way tags.
fn routing_topology_attrs(tags: &Tags, class: &str) -> Arc<[(String, String)]> {
    let mut attrs = vec![
        ("class".to_string(), class.to_string()),
        ("osm_type".to_string(), "way".to_string()),
    ];
    if class == "service" {
        attrs.push((
            "subclass".to_string(),
            tags.get("service")
                .map(ToString::to_string)
                .unwrap_or_else(|| "service".to_string()),
        ));
    }
    for key in TOPOLOGY_TAG_KEYS {
        if let Some(value) = tags.get(key) {
            attrs.push((key.to_string(), value.to_string()));
        }
    }
    // `bridge=no`, `tunnel=no` and `ford=no` are the surface.
    for key in ["bridge", "tunnel", "ford"] {
        if let Some(value) = tags
            .value(key)
            .filter(|value| !value.is_empty() && *value != "no")
        {
            attrs.push((key.to_string(), value.to_string()));
        }
    }
    attrs.into()
}

fn road_routing_attrs(tags: &Tags, road_class: &str) -> RoadRoutingAttrs {
    RoadRoutingAttrs {
        road_class: road_class.to_string(),
        directional_access: match directional_access_from_tags(tags) {
            DirectionalAccess::Both => DirectionalAccessModel::Both,
            DirectionalAccess::Forward => DirectionalAccessModel::Forward,
            DirectionalAccess::Backward => DirectionalAccessModel::Backward,
            DirectionalAccess::Closed => DirectionalAccessModel::Closed,
        },
        access: tags.get("access").map(ToString::to_string),
        surface: tags.get("surface").map(ToString::to_string),
        tracktype: tags.get("tracktype").map(ToString::to_string),
        smoothness: tags.get("smoothness").map(ToString::to_string),
        has_ford: tag_is_present(tags, "ford"),
        has_tunnel: tag_is_present(tags, "tunnel"),
        access_restricted: matches!(
            tags.get("access"),
            Some("private" | "no" | "permit" | "customers" | "military")
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use wayfarer::{GeoPoint, LegRequest, MobilityProfile, RoutingEngine, RoutingPackage};
    use wayfarer_road_graph::{ResolvedEdgeRestriction, RoutableWay, TraversalDirection};

    fn package_engine(path: &std::path::Path) -> RoutingEngine {
        RoutingEngine::from_package(Arc::new(RoutingPackage::open(path).unwrap()))
    }

    #[test]
    fn encoded_way_facts_round_trip_and_keep_the_last_insert() {
        let facts = |way_id: i64, name: &str| RoadWayFacts {
            osm_way_id: way_id,
            name: Some(name.to_string()),
            surface: Some("asphalt".to_string()),
            ..Default::default()
        };
        let mut store = EncodedWayFacts::default();
        store.insert_encoded(30, &codec::to_vec(&facts(30, "third")).unwrap());
        store.insert_encoded(10, &codec::to_vec(&facts(10, "first")).unwrap());
        store.insert_encoded(20, &codec::to_vec(&facts(20, "stale")).unwrap());
        store.insert_encoded(20, &codec::to_vec(&facts(20, "second")).unwrap());
        store.seal();

        assert_eq!(store.get(10).unwrap(), Some(facts(10, "first")));
        assert_eq!(store.get(20).unwrap(), Some(facts(20, "second")));
        assert_eq!(store.get(30).unwrap(), Some(facts(30, "third")));
        assert_eq!(store.get(15).unwrap(), None);
    }

    fn protobuf_varint(mut value: u64, output: &mut Vec<u8>) {
        while value >= 0x80 {
            output.push((value as u8) | 0x80);
            value >>= 7;
        }
        output.push(value as u8);
    }

    fn protobuf_key(field: u32, wire: u8, output: &mut Vec<u8>) {
        protobuf_varint(u64::from((field << 3) | u32::from(wire)), output);
    }

    fn protobuf_bytes(field: u32, bytes: &[u8], output: &mut Vec<u8>) {
        protobuf_key(field, 2, output);
        protobuf_varint(bytes.len() as u64, output);
        output.extend_from_slice(bytes);
    }

    fn protobuf_uint(field: u32, value: u64, output: &mut Vec<u8>) {
        protobuf_key(field, 0, output);
        protobuf_varint(value, output);
    }

    fn zigzag(value: i64) -> u64 {
        ((value << 1) ^ (value >> 63)) as u64
    }

    fn pbf_blob(kind: &str, payload: &[u8], output: &mut Vec<u8>) {
        let mut blob = Vec::new();
        protobuf_bytes(1, payload, &mut blob);
        let mut header = Vec::new();
        protobuf_bytes(1, kind.as_bytes(), &mut header);
        protobuf_uint(3, blob.len() as u64, &mut header);
        output.extend_from_slice(&(header.len() as u32).to_be_bytes());
        output.extend_from_slice(&header);
        output.extend_from_slice(&blob);
    }

    fn write_minimal_osm_pbf(path: &std::path::Path) {
        write_osm_pbf_with_way_tags(path, &[("highway", "residential")]);
    }

    /// Three nodes along 52°N joined by way 1, which carries `way_tags`.
    fn write_osm_pbf_with_way_tags(path: &std::path::Path, way_tags: &[(&str, &str)]) {
        write_osm_pbf(
            path,
            &[(1, 52.0, 21.0), (2, 52.0, 21.001), (3, 52.0, 21.002)],
            &[FixtureWay {
                id: 1,
                nodes: &[1, 2, 3],
                tags: way_tags,
            }],
            &[],
        );
    }

    type FixtureTags<'a> = &'a [(&'a str, &'a str)];

    struct FixtureWay<'a> {
        id: i64,
        nodes: &'a [i64],
        tags: FixtureTags<'a>,
    }

    struct FixtureRelation<'a> {
        id: i64,
        tags: FixtureTags<'a>,
        /// `(role, member type, member id)`; the type is 0 for a node, 1 for a
        /// way, 2 for a relation.
        members: &'a [(&'a str, u64, i64)],
    }

    const NODE: u64 = 0;
    const WAY: u64 = 1;

    /// A PBF of `(id, lat, lon)` nodes, ways and relations, one data block
    /// each, sharing one string table.
    fn write_osm_pbf(
        path: &std::path::Path,
        nodes: &[(i64, f64, f64)],
        ways: &[FixtureWay<'_>],
        relations: &[FixtureRelation<'_>],
    ) {
        write_osm_pbf_with_node_tags(path, nodes, &[], ways, relations);
    }

    /// [`write_osm_pbf`], with tags on the nodes `node_tags` names.
    fn write_osm_pbf_with_node_tags(
        path: &std::path::Path,
        nodes: &[(i64, f64, f64)],
        node_tags: &[(i64, FixtureTags<'_>)],
        ways: &[FixtureWay<'_>],
        relations: &[FixtureRelation<'_>],
    ) {
        let mut file = Vec::new();
        let mut header = Vec::new();
        protobuf_bytes(4, b"OsmSchema-V0.6", &mut header);
        pbf_blob("OSMHeader", &header, &mut file);

        // String 0 is the empty string.
        let mut strings = vec![String::new()];
        fn string_id(strings: &mut Vec<String>, value: &str) -> u64 {
            match strings.iter().position(|candidate| candidate == value) {
                Some(index) => index as u64,
                None => {
                    strings.push(value.to_string());
                    strings.len() as u64 - 1
                }
            }
        }
        fn tag_fields(strings: &mut Vec<String>, tags: FixtureTags<'_>, output: &mut Vec<u8>) {
            let mut keys = Vec::new();
            let mut values = Vec::new();
            for (key, value) in tags {
                protobuf_varint(string_id(strings, key), &mut keys);
                protobuf_varint(string_id(strings, value), &mut values);
            }
            protobuf_bytes(2, &keys, output);
            protobuf_bytes(3, &values, output);
        }
        let mut node_group = Vec::new();
        for &(id, lat, lon) in nodes {
            let mut node = Vec::new();
            protobuf_uint(1, zigzag(id), &mut node);
            if let Some((_, tags)) = node_tags.iter().find(|(node_id, _)| *node_id == id) {
                tag_fields(&mut strings, tags, &mut node);
            }
            protobuf_uint(8, zigzag((lat * 1e7).round() as i64), &mut node);
            protobuf_uint(9, zigzag((lon * 1e7).round() as i64), &mut node);
            protobuf_bytes(1, &node, &mut node_group);
        }
        let mut way_group = Vec::new();
        for way in ways {
            let mut encoded = Vec::new();
            protobuf_uint(1, way.id as u64, &mut encoded);
            tag_fields(&mut strings, way.tags, &mut encoded);
            let mut refs = Vec::new();
            let mut previous = 0;
            for &node in way.nodes {
                protobuf_varint(zigzag(node - previous), &mut refs);
                previous = node;
            }
            protobuf_bytes(8, &refs, &mut encoded);
            protobuf_bytes(3, &encoded, &mut way_group);
        }
        let mut relation_group = Vec::new();
        for relation in relations {
            let mut encoded = Vec::new();
            protobuf_uint(1, relation.id as u64, &mut encoded);
            tag_fields(&mut strings, relation.tags, &mut encoded);
            let (mut roles, mut ids, mut types) = (Vec::new(), Vec::new(), Vec::new());
            let mut previous = 0;
            for &(role, member_type, id) in relation.members {
                protobuf_varint(string_id(&mut strings, role), &mut roles);
                protobuf_varint(zigzag(id - previous), &mut ids);
                previous = id;
                protobuf_varint(member_type, &mut types);
            }
            protobuf_bytes(8, &roles, &mut encoded);
            protobuf_bytes(9, &ids, &mut encoded);
            protobuf_bytes(10, &types, &mut encoded);
            protobuf_bytes(4, &encoded, &mut relation_group);
        }

        let mut string_table = Vec::new();
        for value in &strings {
            protobuf_bytes(1, value.as_bytes(), &mut string_table);
        }
        for group in [node_group, way_group, relation_group] {
            if group.is_empty() {
                continue;
            }
            let mut block = Vec::new();
            protobuf_bytes(1, &string_table, &mut block);
            protobuf_bytes(2, &group, &mut block);
            pbf_blob("OSMData", &block, &mut file);
        }
        std::fs::write(path, file).unwrap();
    }

    /// The road graph compiled from a fixture PBF.
    fn compile_fixture_graph(
        nodes: &[(i64, f64, f64)],
        ways: &[FixtureWay<'_>],
        relations: &[FixtureRelation<'_>],
    ) -> RoadGraphCompileResult {
        compile_fixture_graph_with_node_tags(nodes, &[], ways, relations)
    }

    /// [`compile_fixture_graph`], with tags on the nodes `node_tags` names.
    fn compile_fixture_graph_with_node_tags(
        nodes: &[(i64, f64, f64)],
        node_tags: &[(i64, FixtureTags<'_>)],
        ways: &[FixtureWay<'_>],
        relations: &[FixtureRelation<'_>],
    ) -> RoadGraphCompileResult {
        let dir = tempfile::tempdir().unwrap();
        let pbf = dir.path().join("fixture.osm.pbf");
        write_osm_pbf_with_node_tags(&pbf, nodes, node_tags, ways, relations);
        let never = || false;
        let mut ignore = |_: &job::Report| {};
        compile_road_graph_from_pbf(
            &RoadGraphCompileRequest {
                input_pbf_path: pbf,
                temp_dir: dir.path().join("scratch"),
                graph_id: "fixture".to_string(),
                country_code: None,
                build_profile: IngestBuildProfile::DesktopFull,
                segment_order: SegmentOrder::Topology,
            },
            2,
            &Job::new(&never, &mut ignore),
        )
        .unwrap()
    }

    /// Each node's ways, by node index.
    fn ways_at_nodes(graph: &RoadGraph) -> Vec<std::collections::BTreeSet<i64>> {
        let mut ways = vec![std::collections::BTreeSet::new(); graph.node_count()];
        for edge in graph.graph.edge_references() {
            let way_id = graph
                .shared_edge_info(edge.weight().shared_edge_info_id)
                .unwrap()
                .osm_way_id;
            ways[edge.source().index()].insert(way_id);
            ways[edge.target().index()].insert(way_id);
        }
        ways
    }

    /// The directed edge of `way_id` from source node `from` to `to`.
    fn edge_between(graph: &RoadGraph, way_id: i64, from: i64, to: i64) -> u64 {
        graph
            .graph
            .edge_references()
            .find(|edge| {
                graph
                    .shared_edge_info(edge.weight().shared_edge_info_id)
                    .unwrap()
                    .osm_way_id
                    == way_id
                    && graph.graph.node_source_node_id(edge.source().index()) == Some(from)
                    && graph.graph.node_source_node_id(edge.target().index()) == Some(to)
            })
            .unwrap_or_else(|| panic!("no edge of way {way_id} from {from} to {to}"))
            .weight()
            .edge_id
    }

    #[test]
    fn bridges_cross_roads_without_a_junction_but_tunnel_no_is_the_surface() {
        let crossing = |tags: FixtureTags<'_>| {
            let compiled = compile_fixture_graph(
                &[
                    (1, 52.0, 21.0),
                    (2, 52.0, 21.002),
                    (3, 51.999, 21.001),
                    (4, 52.001, 21.001),
                ],
                &[
                    FixtureWay {
                        id: 1,
                        nodes: &[1, 2],
                        tags: &[("highway", "residential")],
                    },
                    FixtureWay {
                        id: 2,
                        nodes: &[3, 4],
                        tags,
                    },
                ],
                &[],
            );
            let joined = ways_at_nodes(&compiled.road_graph)
                .iter()
                .filter(|ways| ways.len() > 1)
                .count();
            (
                joined,
                compiled
                    .road_graph_quality
                    .topology_repair_report
                    .repaired_crossing_count,
            )
        };
        assert_eq!(
            crossing(&[("highway", "primary"), ("bridge", "yes"), ("layer", "1")]),
            (0, 0)
        );
        assert_eq!(
            crossing(&[("highway", "primary"), ("tunnel", "yes")]),
            (0, 0)
        );
        assert_eq!(crossing(&[("highway", "primary"), ("layer", "-1")]), (0, 0));
        // The same plane: a crossing mapped without its shared node is joined.
        assert_eq!(
            crossing(&[("highway", "primary"), ("tunnel", "no")]),
            (1, 1)
        );
        assert_eq!(crossing(&[("highway", "primary")]), (1, 1));
    }

    /// A four-way junction at node 10: way 1 from the west, ways 2, 3 and 4
    /// to the east, north and south.
    const JUNCTION_NODES: &[(i64, f64, f64)] = &[
        (10, 52.0, 21.0),
        (11, 52.0, 20.999),
        (12, 52.0, 21.001),
        (13, 52.001, 21.0),
        (14, 51.999, 21.0),
    ];
    const RESIDENTIAL: FixtureTags<'static> = &[("highway", "residential")];
    const JUNCTION_WAYS: &[FixtureWay<'static>] = &[
        FixtureWay {
            id: 1,
            nodes: &[11, 10],
            tags: RESIDENTIAL,
        },
        FixtureWay {
            id: 2,
            nodes: &[10, 12],
            tags: RESIDENTIAL,
        },
        FixtureWay {
            id: 3,
            nodes: &[10, 13],
            tags: RESIDENTIAL,
        },
        FixtureWay {
            id: 4,
            nodes: &[10, 14],
            tags: RESIDENTIAL,
        },
    ];

    mod embedding;

    #[test]
    fn only_restrictions_ban_every_other_turn_for_vehicles_only() {
        let compiled = compile_fixture_graph(
            JUNCTION_NODES,
            JUNCTION_WAYS,
            &[FixtureRelation {
                id: 100,
                tags: &[("type", "restriction"), ("restriction", "only_straight_on")],
                members: &[("from", WAY, 1), ("via", NODE, 10), ("to", WAY, 2)],
            }],
        );
        let graph = &compiled.road_graph;
        let from = edge_between(graph, 1, 11, 10);
        let mut banned = Vec::new();
        for restriction in &graph.resolved_restrictions {
            let ResolvedEdgeRestriction::Simple {
                from_edge_id,
                to_edge_id,
                osm_mode_mask,
                mobility_mode_mask,
                ..
            } = restriction
            else {
                panic!("unexpected {restriction:?}");
            };
            assert_eq!(*from_edge_id, from);
            assert_eq!(osm_mode_mask & wayfarer_road_graph::OsmMode::FOOT, 0);
            assert_eq!(
                *mobility_mode_mask,
                wayfarer_road_graph::MobilityMode::WHEELED
                    | wayfarer_road_graph::MobilityMode::TRACKED
                    | wayfarer_road_graph::MobilityMode::EMERGENCY
            );
            banned.push(*to_edge_id);
        }
        banned.sort_unstable();
        let mut expected = vec![
            edge_between(graph, 1, 10, 11),
            edge_between(graph, 3, 10, 13),
            edge_between(graph, 4, 10, 14),
        ];
        expected.sort_unstable();
        assert_eq!(banned, expected);
        assert_eq!(
            compiled
                .road_graph_quality
                .restriction_report
                .unresolved_restriction_count,
            0
        );
    }

    #[test]
    fn restrictions_keep_every_via_way_and_every_from_way() {
        // 11 -1- 10 -5- 20 -6- 21 -7- 22, with way 9 from 15 into 10 and way 8
        // from 21 to 23.
        let compiled = compile_fixture_graph(
            &[
                (10, 52.0, 21.0),
                (11, 52.0, 20.999),
                (15, 51.999, 21.0),
                (20, 52.0, 21.001),
                (21, 52.0, 21.002),
                (22, 52.0, 21.003),
                (23, 52.001, 21.002),
            ],
            &[
                FixtureWay {
                    id: 1,
                    nodes: &[11, 10],
                    tags: RESIDENTIAL,
                },
                FixtureWay {
                    id: 9,
                    nodes: &[15, 10],
                    tags: RESIDENTIAL,
                },
                FixtureWay {
                    id: 5,
                    nodes: &[10, 20],
                    tags: RESIDENTIAL,
                },
                FixtureWay {
                    id: 6,
                    nodes: &[20, 21],
                    tags: RESIDENTIAL,
                },
                FixtureWay {
                    id: 7,
                    nodes: &[21, 22],
                    tags: RESIDENTIAL,
                },
                FixtureWay {
                    id: 8,
                    nodes: &[21, 23],
                    tags: RESIDENTIAL,
                },
            ],
            &[
                FixtureRelation {
                    id: 200,
                    tags: &[("type", "restriction"), ("restriction", "no_straight_on")],
                    members: &[
                        ("from", WAY, 1),
                        ("from", WAY, 9),
                        ("via", WAY, 5),
                        ("via", WAY, 6),
                        ("to", WAY, 7),
                    ],
                },
                FixtureRelation {
                    id: 201,
                    tags: &[("type", "restriction"), ("restriction", "only_left_turn")],
                    members: &[
                        ("from", WAY, 1),
                        ("via", WAY, 5),
                        ("via", WAY, 6),
                        ("to", WAY, 8),
                    ],
                },
            ],
        );
        let graph = &compiled.road_graph;
        let via = vec![
            edge_between(graph, 5, 10, 20),
            edge_between(graph, 6, 20, 21),
        ];
        let complex = |relation: i64| {
            let mut rows = graph
                .resolved_restrictions
                .iter()
                .filter_map(|restriction| match restriction {
                    ResolvedEdgeRestriction::Complex {
                        relation_id,
                        from_edge_id,
                        to_edge_id,
                        via_edge_ids,
                        ..
                    } if *relation_id == relation => {
                        Some((*from_edge_id, via_edge_ids.clone(), *to_edge_id))
                    }
                    _ => None,
                })
                .collect::<Vec<_>>();
            rows.sort();
            rows
        };
        let to = edge_between(graph, 7, 21, 22);
        let mut expected = vec![
            (edge_between(graph, 1, 11, 10), via.clone(), to),
            (edge_between(graph, 9, 15, 10), via.clone(), to),
        ];
        expected.sort();
        assert_eq!(complex(200), expected);
        // `only_left_turn` onto way 8 bans going on (way 7) and turning back
        // (way 6) at the end of the via ways.
        let from = edge_between(graph, 1, 11, 10);
        let mut expected = vec![
            (from, via.clone(), to),
            (from, via.clone(), edge_between(graph, 6, 21, 20)),
        ];
        expected.sort();
        assert_eq!(complex(201), expected);
        assert_eq!(
            compiled
                .road_graph_quality
                .restriction_report
                .unresolved_restriction_count,
            0
        );
    }

    #[test]
    fn mode_specific_restrictions_bind_their_mode_and_are_excepted_from_the_plain_one() {
        let mut tags = Tags::new();
        for (key, value) in [
            ("type", "restriction"),
            ("restriction", "no_left_turn"),
            ("restriction:hgv", "no_straight_on"),
            ("restriction:conditional", "no_u_turn @ (Mo-Fr 07:00-09:00)"),
            ("except", "bus"),
        ] {
            tags.insert(key, value);
        }
        let member =
            |role: &str, member_type, member_id| osm_pbf_ingest::NormalizedRelationMember {
                member_type,
                member_id,
                role: role.to_string(),
            };
        let rows = turn_restrictions_from_relation(&NormalizedRelation {
            relation_id: 7,
            tags,
            members: vec![
                member("from", RelationMemberType::Way, 1),
                member("via", RelationMemberType::Node, 10),
                member("to", RelationMemberType::Way, 3),
            ],
        });
        let summary = rows
            .iter()
            .map(|row| {
                (
                    row.restriction.as_str(),
                    row.modes.clone(),
                    row.except_modes.clone(),
                )
            })
            .collect::<Vec<_>>();
        assert_eq!(
            summary,
            [
                (
                    "no_left_turn",
                    vec![],
                    vec!["bus".to_string(), "hgv".to_string()]
                ),
                (
                    "no_straight_on",
                    vec!["hgv".to_string()],
                    vec!["bus".to_string()]
                ),
            ]
        );
    }

    /// Turns resolved for a no-left-turn from way 1 onto way 3 at node 10,
    /// with extra relation tags, and the compile's restriction report.
    fn resolve_left_turn_ban(
        extra_tags: FixtureTags<'_>,
    ) -> (
        Vec<ResolvedEdgeRestriction>,
        wayfarer_road_graph::RoadGraphRestrictionReport,
    ) {
        let mut tags = vec![("type", "restriction")];
        tags.extend_from_slice(extra_tags);
        let compiled = compile_fixture_graph(
            JUNCTION_NODES,
            JUNCTION_WAYS,
            &[FixtureRelation {
                id: 300,
                tags: &tags,
                members: &[("from", WAY, 1), ("via", NODE, 10), ("to", WAY, 3)],
            }],
        );
        (
            compiled.road_graph.resolved_restrictions.clone(),
            compiled.road_graph_quality.restriction_report,
        )
    }

    #[test]
    fn restrictions_for_modes_no_profile_routes_ban_no_turn() {
        for tags in [
            &[("restriction:hgv", "no_left_turn")][..],
            &[("restriction:bus", "no_left_turn")],
            &[("restriction:psv", "no_left_turn")],
            &[("restriction:goods", "no_left_turn")],
            &[("restriction:emergency", "no_left_turn")],
            &[("restriction:bicycle", "no_left_turn")],
            &[("restriction", "no_left_turn"), ("except", "motorcar")],
            &[("restriction", "no_left_turn"), ("except", "motor_vehicle")],
        ] {
            let (resolved, report) = resolve_left_turn_ban(tags);
            assert!(resolved.is_empty(), "{tags:?} resolved to {resolved:?}");
            assert_eq!(report.mode_not_routed_restriction_count, 1, "{tags:?}");
            assert_eq!(report.unresolved_restriction_count, 0, "{tags:?}");
            assert_eq!(report.raw_restriction_count, 1, "{tags:?}");
        }
    }

    #[test]
    fn car_restrictions_still_bind_cars_whatever_they_except() {
        let wheeled = wayfarer_road_graph::MobilityMode::WHEELED
            | wayfarer_road_graph::MobilityMode::TRACKED
            | wayfarer_road_graph::MobilityMode::EMERGENCY;
        for tags in [
            &[("restriction", "no_left_turn")][..],
            &[("restriction:motorcar", "no_left_turn")],
            &[("restriction:motor_vehicle", "no_left_turn")],
            &[("restriction", "no_left_turn"), ("except", "bus;bicycle")],
            &[("restriction", "no_left_turn"), ("except", "hgv")],
        ] {
            let (resolved, report) = resolve_left_turn_ban(tags);
            let [
                ResolvedEdgeRestriction::Simple {
                    mobility_mode_mask, ..
                },
            ] = resolved.as_slice()
            else {
                panic!("{tags:?} resolved to {resolved:?}");
            };
            assert_eq!(*mobility_mode_mask, wheeled, "{tags:?}");
            assert_eq!(report.mode_not_routed_restriction_count, 0, "{tags:?}");
        }
    }

    #[test]
    fn manifest_counts_raw_restrictions_and_route_memberships() {
        let dir = tempfile::tempdir().unwrap();
        let pbf = dir.path().join("junction.osm.pbf");
        write_osm_pbf(
            &pbf,
            JUNCTION_NODES,
            JUNCTION_WAYS,
            &[
                FixtureRelation {
                    id: 100,
                    tags: &[("type", "restriction"), ("restriction", "no_left_turn")],
                    members: &[("from", WAY, 1), ("via", NODE, 10), ("to", WAY, 3)],
                },
                FixtureRelation {
                    id: 300,
                    tags: &[("type", "route"), ("route", "road"), ("ref", "D1")],
                    members: &[("", WAY, 1), ("", WAY, 2)],
                },
            ],
        );
        let result = compile(
            &CompileRequest::new(pbf, dir.path().join("routing.mrt"))
                .scratch_dir(dir.path().join("scratch"))
                .graph_id("junction"),
            &CancelToken::new(),
            |_| {},
        )
        .unwrap();
        assert_eq!(result.graph_manifest.raw_turn_restriction_count, 1);
        assert_eq!(result.graph_manifest.resolved_turn_restriction_count, 1);
        assert_eq!(result.graph_manifest.route_membership_count, 2);
    }

    #[test]
    fn way_tags_decide_modes_and_direction() {
        use wayfarer_road_graph::OsmMode;
        const MOTOR: u32 =
            OsmMode::MOTOR_VEHICLE | OsmMode::BUS | OsmMode::HGV | OsmMode::EMERGENCY;
        // (forward mask, reverse mask) of the way's one segment; no edges is
        // (0, 0).
        let masks = |tags: FixtureTags<'_>| {
            let compiled = compile_fixture_graph(
                &[(1, 52.0, 21.0), (2, 52.0, 21.001), (3, 52.0, 21.002)],
                &[FixtureWay {
                    id: 1,
                    nodes: &[1, 2, 3],
                    tags,
                }],
                &[],
            );
            let mut masks = (0, 0);
            for edge in compiled.road_graph.graph.edge_weights() {
                match edge.traversal_direction {
                    TraversalDirection::Forward => masks.0 = edge.osm_forward_mode_mask,
                    TraversalDirection::Backward => masks.1 = edge.osm_reverse_mode_mask,
                }
            }
            masks
        };
        let foot = OsmMode::FOOT;
        // C3: per-highway defaults.
        assert_eq!(masks(&[("highway", "footway")]), (foot, foot));
        assert_eq!(masks(&[("highway", "construction")]), (0, 0));
        assert_eq!(masks(&[("highway", "proposed")]), (0, 0));
        assert_eq!(
            masks(&[("highway", "platform"), ("foot", "yes")]),
            (foot, foot)
        );
        // C4: a mode's own grant survives `access=no`.
        assert_eq!(
            masks(&[
                ("highway", "residential"),
                ("access", "no"),
                ("foot", "yes")
            ]),
            (foot, foot)
        );
        // C11: implied and special one-ways.
        assert_eq!(masks(&[("highway", "motorway")]), (MOTOR, 0));
        assert_eq!(masks(&[("highway", "motorway_link")]), (MOTOR, 0));
        assert_eq!(
            masks(&[("highway", "motorway"), ("oneway", "no")]),
            (MOTOR, MOTOR)
        );
        assert_eq!(
            masks(&[("highway", "residential"), ("oneway", "reversible")]),
            (0, 0)
        );
        assert_eq!(
            masks(&[("highway", "residential"), ("oneway", "alternating")]),
            (OsmMode::ALL, OsmMode::ALL)
        );
        // C7: ferries and shuttle trains, open to every mode unless tagged.
        assert_eq!(masks(&[("route", "ferry")]), (OsmMode::ALL, OsmMode::ALL));
        assert_eq!(
            masks(&[("route", "shuttle_train"), ("foot", "no")]),
            (OsmMode::ALL & !foot, OsmMode::ALL & !foot)
        );
    }

    /// A package of a 140 m `Shortcut` way between nodes 1 and 2, tagged
    /// `shortcut`, and a 580 m residential detour round it, with the
    /// directory that holds it.
    fn shortcut_engine(shortcut: FixtureTags<'_>) -> (tempfile::TempDir, RoutingEngine) {
        let dir = tempfile::tempdir().unwrap();
        let pbf = dir.path().join("shortcut.osm.pbf");
        let mut tags = shortcut.to_vec();
        tags.push(("name", "Shortcut"));
        write_osm_pbf(
            &pbf,
            &[
                (1, 52.0, 21.0),
                (2, 52.0, 21.002),
                (3, 52.002, 21.0),
                (4, 52.002, 21.002),
            ],
            &[
                FixtureWay {
                    id: 10,
                    nodes: &[1, 2],
                    tags: &tags,
                },
                FixtureWay {
                    id: 11,
                    nodes: &[1, 3, 4, 2],
                    tags: &[("highway", "residential"), ("name", "Detour")],
                },
            ],
            &[],
        );
        let archive_path = dir.path().join("routing.mrt");
        compile(
            &CompileRequest::new(pbf, archive_path.clone()).graph_id("shortcut"),
            &CancelToken::new(),
            |_| {},
        )
        .unwrap();
        let engine = package_engine(&archive_path);
        (dir, engine)
    }

    /// Whether `request` (from node 1 to node 2) takes the shortcut.
    fn takes_shortcut(engine: &RoutingEngine, request: LegRequest) -> bool {
        engine.route_leg(request).unwrap().legs[0]
            .edges
            .iter()
            .any(|edge| edge.name.as_deref() == Some("Shortcut"))
    }

    fn shortcut_request(profile: MobilityProfile) -> LegRequest {
        LegRequest::new(
            GeoPoint::new(52.0, 21.0),
            GeoPoint::new(52.0, 21.002),
            profile,
        )
    }

    /// The profiles that take the shortcut from node 1 to node 2.
    fn profiles_taking_shortcut(shortcut: FixtureTags<'_>) -> Vec<MobilityProfile> {
        let (_dir, engine) = shortcut_engine(shortcut);
        [
            MobilityProfile::Wheeled,
            MobilityProfile::Tracked,
            MobilityProfile::Emergency,
            MobilityProfile::OnFoot,
            MobilityProfile::Legged,
        ]
        .into_iter()
        .filter(|profile| takes_shortcut(&engine, shortcut_request(*profile)))
        .collect()
    }

    #[test]
    fn bicycle_only_ways_are_closed_to_walkers_but_not_to_legged() {
        use MobilityProfile::{Legged, OnFoot};
        // P0a: bicycle access alone no longer opens a way to `OnFoot`;
        // `Legged` ignores `foot=no` (decision D2).
        assert_eq!(
            profiles_taking_shortcut(&[("highway", "cycleway"), ("foot", "no")]),
            [Legged]
        );
        assert_eq!(
            profiles_taking_shortcut(&[("highway", "track"), ("access", "no"), ("bicycle", "yes")]),
            [Legged]
        );
        // Cycleways are walkable by default, and footways stay walkable.
        assert_eq!(
            profiles_taking_shortcut(&[("highway", "cycleway")]),
            [OnFoot, Legged]
        );
        assert_eq!(
            profiles_taking_shortcut(&[("highway", "footway")]),
            [OnFoot, Legged]
        );
    }

    #[test]
    fn busways_are_closed_to_cars_but_not_to_emergency_vehicles() {
        use MobilityProfile::{Emergency, Legged, OnFoot, Tracked, Wheeled};
        // P0d: bus access alone no longer opens a way to cars.
        assert_eq!(
            profiles_taking_shortcut(&[("highway", "busway")]),
            [Emergency]
        );
        // Roads open to cars stay open to emergency vehicles.
        assert_eq!(
            profiles_taking_shortcut(&[("highway", "residential")]),
            [Wheeled, Tracked, Emergency, OnFoot, Legged]
        );
        assert_eq!(
            profiles_taking_shortcut(&[("highway", "residential"), ("motor_vehicle", "no")]),
            [OnFoot, Legged]
        );
    }

    /// The graph's edges between source nodes, as `(way, from, to)`, sorted.
    fn way_edges(graph: &RoadGraph) -> Vec<(i64, i64, i64)> {
        let mut edges = graph
            .graph
            .edge_references()
            .map(|edge| {
                let way_id = graph
                    .shared_edge_info(edge.weight().shared_edge_info_id)
                    .unwrap()
                    .osm_way_id;
                let node = |index: usize| graph.graph.node_source_node_id(index).unwrap();
                (
                    way_id,
                    node(edge.source().index()),
                    node(edge.target().index()),
                )
            })
            .collect::<Vec<_>>();
        edges.sort_unstable();
        edges
    }

    #[test]
    fn ways_are_cut_at_nodes_the_file_does_not_hold() {
        // Node 3 is missing: way 1 is two lines, 1-2 and 4-5, with nothing
        // across the gap. Way 2 keeps no two located nodes in a row.
        let compiled = compile_fixture_graph(
            &[
                (1, 52.0, 21.0),
                (2, 52.0, 21.001),
                (4, 52.0, 21.003),
                (5, 52.0, 21.004),
                (6, 52.001, 21.0),
                (7, 52.001, 21.002),
            ],
            &[
                FixtureWay {
                    id: 1,
                    nodes: &[1, 2, 3, 4, 5],
                    tags: &[("highway", "residential"), ("name", "Cut")],
                },
                FixtureWay {
                    id: 2,
                    nodes: &[6, 3, 7, 8],
                    tags: &[("highway", "residential")],
                },
            ],
            &[],
        );
        assert_eq!(
            way_edges(&compiled.road_graph),
            [(1, 1, 2), (1, 2, 1), (1, 4, 5), (1, 5, 4)]
        );
        let quality = &compiled.road_graph_quality;
        assert_eq!(
            quality.input_report,
            RoadGraphInputReport {
                way_with_missing_nodes_count: 2,
                missing_node_ref_count: 3,
                cut_piece_count: 1,
                dropped_missing_node_way_count: 1,
                missing_node_way_id_examples: vec![1, 2],
                ..Default::default()
            }
        );
        assert!(
            quality.reasons.contains(
                &"2 road ways refer to 3 nodes the input does not hold; they were cut there"
                    .to_string()
            ),
            "{:?}",
            quality.reasons
        );
    }

    #[test]
    fn a_repeated_way_keeps_its_copy_with_the_most_located_nodes_then_the_first() {
        let nodes = [
            (1, 52.0, 21.0),
            (2, 52.0, 21.001),
            (3, 52.0, 21.002),
            (4, 52.001, 21.0),
            (5, 52.001, 21.001),
            (6, 52.0015, 21.0005),
        ];
        let compiled = compile_fixture_graph(
            &nodes,
            &[
                // Way 7: the second copy has more located nodes.
                FixtureWay {
                    id: 7,
                    nodes: &[1, 2],
                    tags: &[("highway", "residential"), ("name", "Old")],
                },
                FixtureWay {
                    id: 3,
                    nodes: &[4, 5],
                    tags: &[("highway", "service"), ("name", "First")],
                },
                FixtureWay {
                    id: 7,
                    nodes: &[1, 2, 3],
                    tags: &[("highway", "residential"), ("name", "New")],
                },
                // Way 3: as many located nodes, so the first copy stays; the
                // last, an area and no line, has none.
                FixtureWay {
                    id: 3,
                    nodes: &[4, 5],
                    tags: &[("highway", "service"), ("name", "Second")],
                },
                FixtureWay {
                    id: 3,
                    nodes: &[4, 5, 6, 4],
                    tags: &[("highway", "service"), ("area", "yes"), ("name", "Area")],
                },
            ],
            &[],
        );
        let graph = &compiled.road_graph;
        assert_eq!(
            way_edges(graph),
            [(3, 4, 5), (3, 5, 4), (7, 1, 3), (7, 3, 1)]
        );
        let names = graph
            .shared_edge_infos
            .iter()
            .map(|info| {
                (
                    info.osm_way_id,
                    info.attrs.name.as_deref().unwrap().to_string(),
                )
            })
            .collect::<std::collections::BTreeSet<_>>();
        assert_eq!(
            names,
            [(3, "First".to_string()), (7, "New".to_string())]
                .into_iter()
                .collect()
        );
        let quality = &compiled.road_graph_quality;
        assert_eq!(
            quality.input_report,
            RoadGraphInputReport {
                duplicate_way_id_count: 2,
                dropped_duplicate_way_count: 3,
                duplicate_way_id_examples: vec![3, 7],
                ..Default::default()
            }
        );
        assert!(
            quality.reasons.contains(
                &"2 road way ids appear more than once in the input; one copy of each was kept"
                    .to_string()
            ),
            "{:?}",
            quality.reasons
        );
    }

    #[test]
    fn a_clean_input_leaves_the_input_report_out_of_the_manifest() {
        let compiled = compile_fixture_graph(
            &[(1, 52.0, 21.0), (2, 52.0, 21.001)],
            &[FixtureWay {
                id: 1,
                nodes: &[1, 2],
                tags: &[("highway", "residential")],
            }],
            &[],
        );
        let quality = &compiled.road_graph_quality;
        assert!(quality.input_report.is_clean());
        let json = serde_json::to_value(quality).unwrap();
        assert!(json.get("inputReport").is_none(), "{json}");
    }

    #[test]
    fn ferries_are_routable_and_take_their_speed_from_their_duration() {
        let compiled = compile_fixture_graph(
            &[(1, 52.0, 21.0), (2, 52.0, 21.001), (3, 52.0, 21.002)],
            &[FixtureWay {
                id: 1,
                nodes: &[1, 2, 3],
                tags: &[("route", "ferry"), ("duration", "00:01"), ("name", "Bac")],
            }],
            &[],
        );
        let graph = &compiled.road_graph;
        assert_eq!(graph.edge_count(), 2);
        let info = graph.shared_edge_infos.at(0).attrs;
        assert!(info.is_ferry);
        assert_eq!(info.name.as_deref(), Some("Bac"));
        // 137 m in a minute.
        let length_m: f64 = haversine_m((52.0, 21.0), (52.0, 21.001)) * 2.0;
        let expected_kph = (length_m / 60.0 * 3.6 * 10.0).round() / 10.0;
        assert_eq!(info.maxspeed_kph, Some(expected_kph as f32));

        assert_eq!(osm_duration_s("45"), Some(2_700.0));
        assert_eq!(osm_duration_s("1:30"), Some(5_400.0));
        assert_eq!(osm_duration_s("01:30:15"), Some(5_415.0));
        assert_eq!(osm_duration_s("PT1H30M"), Some(5_400.0));
        assert_eq!(osm_duration_s("P1DT2H"), Some(93_600.0));
        assert_eq!(osm_duration_s("soon"), None);
        assert_eq!(osm_duration_s("PT"), None);
    }

    #[test]
    fn links_leaving_a_motorway_junction_take_its_exit_number() {
        // The motorway 1-2-3 with junction 2 ("7A", exit to Tarnów). Way 10
        // leaves it at 2; way 11 joins it at 2 (an on-ramp); way 12 leaves it
        // at 2 drawn backwards (oneway=-1); way 13 has its own junction:ref
        // and way 14 its own destination; way 15 is no link.
        let junction = [
            ("highway", "motorway_junction"),
            ("ref", "7A"),
            ("exit_to", "Tarnów"),
        ];
        let link = |oneway: &'static str| [("highway", "motorway_link"), ("oneway", oneway)];
        let compiled = compile_fixture_graph_with_node_tags(
            &[
                (1, 52.0, 21.0),
                (2, 52.0, 21.001),
                (3, 52.0, 21.002),
                (4, 52.001, 21.002),
                (5, 51.999, 21.0),
                (6, 52.002, 21.002),
                (7, 51.998, 21.002),
                (8, 51.997, 21.002),
                (9, 52.003, 21.002),
                (10, 52.004, 21.002),
            ],
            &[(2, &junction)],
            &[
                FixtureWay {
                    id: 1,
                    nodes: &[1, 2, 3],
                    tags: &[("highway", "motorway"), ("oneway", "yes")],
                },
                FixtureWay {
                    id: 10,
                    nodes: &[2, 4],
                    tags: &link("yes"),
                },
                FixtureWay {
                    id: 11,
                    nodes: &[5, 2],
                    tags: &link("yes"),
                },
                FixtureWay {
                    id: 12,
                    nodes: &[6, 2],
                    tags: &link("-1"),
                },
                FixtureWay {
                    id: 13,
                    nodes: &[2, 7],
                    tags: &[
                        ("highway", "motorway_link"),
                        ("oneway", "yes"),
                        ("junction:ref", "8"),
                    ],
                },
                FixtureWay {
                    id: 14,
                    nodes: &[2, 8],
                    tags: &[
                        ("highway", "motorway_link"),
                        ("oneway", "yes"),
                        ("destination", "Rzeszów"),
                    ],
                },
                FixtureWay {
                    id: 15,
                    nodes: &[2, 9, 10],
                    tags: &[("highway", "primary")],
                },
            ],
            &[],
        );
        let graph = &compiled.road_graph;
        let signs = |way_id: i64| {
            let info = graph
                .graph
                .edge_references()
                .map(|edge| {
                    graph
                        .shared_edge_info(edge.weight().shared_edge_info_id)
                        .unwrap()
                })
                .find(|info| info.osm_way_id == way_id)
                .unwrap_or_else(|| panic!("no edge of way {way_id}"));
            (
                info.attrs.junction_ref.as_deref().map(str::to_string),
                info.attrs.destination.as_deref().map(str::to_string),
            )
        };
        let some = |value: &str| Some(value.to_string());
        assert_eq!(signs(10), (some("7A"), some("Tarnów")));
        assert_eq!(signs(11), (None, None));
        assert_eq!(signs(12), (some("7A"), some("Tarnów")));
        assert_eq!(signs(13), (some("8"), None));
        assert_eq!(signs(14), (some("7A"), some("Rzeszów")));
        assert_eq!(signs(15), (None, None));
        assert_eq!(signs(1), (None, None));
    }

    #[test]
    fn pedestrian_squares_are_walked_around_and_across() {
        // The square 1-2-3-4, entered by footways at corners 1 and 3.
        let compiled = compile_fixture_graph(
            &[
                (1, 52.0, 21.0),
                (2, 52.0, 21.001),
                (3, 52.001, 21.001),
                (4, 52.001, 21.0),
                (5, 51.999, 20.999),
                (6, 52.002, 21.002),
            ],
            &[
                FixtureWay {
                    id: 1,
                    nodes: &[1, 2, 3, 4, 1],
                    tags: &[("highway", "pedestrian"), ("area", "yes")],
                },
                FixtureWay {
                    id: 2,
                    nodes: &[5, 1],
                    tags: &[("highway", "footway")],
                },
                FixtureWay {
                    id: 3,
                    nodes: &[3, 6],
                    tags: &[("highway", "footway")],
                },
            ],
            &[],
        );
        let report = &compiled.road_graph_quality.builder_report;
        assert_eq!(report.synthesized_plaza_count, 1);
        assert_eq!(report.plaza_anchor_count, 2);
        let graph = &compiled.road_graph;
        assert!(
            graph
                .shared_edge_infos
                .iter()
                .any(|info| info.osm_way_id == 1)
        );
        for edge in graph.graph.edge_weights() {
            assert_eq!(
                edge.osm_forward_mode_mask & wayfarer_road_graph::OsmMode::MOTOR_VEHICLE,
                0
            );
        }
    }

    #[test]
    fn graph_compiler_package_contract_is_standalone_and_serialization_stable() {
        let mut builder = RoadGraphBuilder::new("standalone-contract");
        builder.push_way(RoutableWay {
            osm_way_id: 1,
            points: vec![(52.0, 21.0), (52.0, 21.001), (52.0, 21.002)],
            source_node_ids: vec![Some(1), Some(2), Some(3)],
            road_type: RoadType::Residential,
            ..RoutableWay::default()
        });
        let mut graph = builder.finish();
        wayfarer_road_graph::enhance_road_graph(&mut graph);
        let graph_manifest = RoadGraphPackageManifest {
            graph_schema_version: ROAD_GRAPH_SNAPSHOT_VERSION,
            node_count: graph.node_count(),
            directed_edge_count: graph.edge_count(),
            shared_edge_info_count: graph.shared_edge_infos.len(),
            ..RoadGraphPackageManifest::default()
        };
        let dir = tempfile::tempdir().unwrap();
        let archive_path = dir.path().join("contract.mrt");
        package_build::write_package(
            &graph,
            Some(&graph_manifest),
            &archive_path,
            &PackageBuildOptions::default(),
        )
        .unwrap();

        assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 1);
        let request = LegRequest::new(
            GeoPoint::new(52.0, 21.00005),
            GeoPoint::new(52.0, 21.00195),
            MobilityProfile::Wheeled,
        );
        let request_json = serde_json::to_string(&request).unwrap();
        let request: LegRequest = serde_json::from_str(&request_json).unwrap();
        assert_eq!(serde_json::to_string(&request).unwrap(), request_json);
        let result = package_engine(&archive_path).route_leg(request).unwrap();
        let response = serde_json::json!({
            "distanceM": (result.distance_m * 1000.0).round() / 1000.0,
            "edgeCount": result.edge_ids.len(),
            "geometryPoints": result.geometry.len(),
        });
        // Endpoints sit at 2.5% and 97.5% of the ~137 m three-point edge. The
        // former 123.225 m came from snap fractions measured along a single
        // correlation window instead of the whole edge.
        assert_eq!(
            serde_json::to_string(&response).unwrap(),
            r#"{"distanceM":130.071,"edgeCount":1,"geometryPoints":3}"#
        );
    }

    #[test]
    fn pbf_compiler_to_package_engine_contract_needs_no_wayfarer_data_dir() {
        let dir = tempfile::tempdir().unwrap();
        let pbf = dir.path().join("fixture.osm.pbf");
        write_minimal_osm_pbf(&pbf);
        let archive_path = dir.path().join("routing.mrt");
        let result = compile(
            &CompileRequest::new(pbf.clone(), archive_path.clone()).graph_id("pbf-contract"),
            &CancelToken::new(),
            |_| {},
        )
        .unwrap();

        std::fs::remove_file(&pbf).unwrap();
        // The package is the only output.
        assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 1);
        let verified = RoutingPackage::verify(&archive_path).unwrap();
        assert_eq!(verified.dataset_hash, result.dataset_hash);
        assert_eq!(verified.byte_len, result.byte_len);
        let engine = package_engine(&archive_path);
        let view = engine.route_graph_view();
        assert_eq!(view.graph_id(), "pbf-contract");
        let mut nodes = (0..view.raw_node_count())
            .filter_map(|node| view.node_by_raw_node_index(node))
            .collect::<Vec<_>>();
        nodes.sort_by_key(|node| node.id);
        let origin = nodes.first().expect("PBF graph has nodes");
        let destination = nodes.last().expect("PBF graph has nodes");
        let request = LegRequest::new(
            GeoPoint::new(origin.lat, origin.lng),
            GeoPoint::new(destination.lat, destination.lng),
            MobilityProfile::Wheeled,
        );
        let request_json = serde_json::to_string(&request).unwrap();
        let result = engine
            .route_leg(serde_json::from_str(&request_json).unwrap())
            .unwrap();
        let response_json = serde_json::to_string(&serde_json::json!({
            "distanceM": (result.distance_m * 1000.0).round() / 1000.0,
            "edgeCount": result.edge_ids.len(),
            "geometryPoints": result.geometry.len(),
        }))
        .unwrap();
        assert_eq!(
            response_json,
            r#"{"distanceM":136.917,"edgeCount":1,"geometryPoints":2}"#
        );
    }

    #[test]
    fn conditional_tags_reach_the_package_and_gate_access_by_time() {
        let dir = tempfile::tempdir().unwrap();
        let pbf = dir.path().join("conditional.osm.pbf");
        write_osm_pbf_with_way_tags(
            &pbf,
            &[
                ("highway", "residential"),
                ("access:conditional", "no @ (2026 May 01-2026 Jun 30)"),
                ("maxspeed:conditional", "20 @ (22:00-06:00)"),
                ("name", "Seasonal Lane"),
            ],
        );
        let archive_path = dir.path().join("conditional.mrt");
        compile(
            &CompileRequest::new(pbf, archive_path.clone()).graph_id("conditional"),
            &CancelToken::new(),
            |_| {},
        )
        .unwrap();

        let engine = package_engine(&archive_path);
        let view = engine.route_graph_view();
        assert!(view.raw_edge_count() > 0);
        for edge in 0..view.raw_edge_count() {
            let info = view
                .edge_record_by_raw_edge_index(edge)
                .unwrap()
                .shared_info;
            assert_eq!(
                info.conditional_restrictions
                    .iter()
                    .map(String::as_str)
                    .collect::<Vec<_>>(),
                [
                    "access:conditional=no @ (2026 May 01-2026 Jun 30)",
                    "maxspeed:conditional=20 @ (22:00-06:00)",
                ],
                "way {} lost its conditional tags",
                info.osm_way_id
            );
        }

        let route_at = |eval_at_ms: Option<i64>| {
            let mut request = LegRequest::new(
                GeoPoint::new(52.0, 21.0002),
                GeoPoint::new(52.0, 21.0018),
                MobilityProfile::Wheeled,
            );
            request.options.costing.conditional_restriction_eval_at_ms = eval_at_ms;
            engine.route_leg(request)
        };
        // 2026-05-15T00:00Z is inside the closure, 2026-07-01T08:00Z outside.
        assert!(route_at(Some(1_778_803_200_000)).is_err());
        assert!(route_at(Some(1_782_892_800_000)).is_ok());
        assert!(route_at(None).is_ok());
    }

    #[test]
    fn way_facts_collect_sorted_conditional_tags() {
        let mut tags = Tags::new();
        tags.insert("highway", "residential");
        tags.insert("motor_vehicle:conditional", " destination @ (Sa,Su) ");
        tags.insert("access:conditional", "no @ (Mo-Fr 07:00-09:00)");
        tags.insert("maxspeed", "50");
        tags.insert("empty:conditional", " ");
        let facts = road_way_facts_from_tags(7, &tags);
        assert_eq!(
            facts.conditional_restrictions,
            [
                "access:conditional=no @ (Mo-Fr 07:00-09:00)",
                "motor_vehicle:conditional=destination @ (Sa,Su)",
            ]
        );
    }

    #[test]
    fn excludes_closed_highway_areas_but_pedestrian_squares() {
        let mut tags = Tags::new();
        tags.insert("highway", "service");
        tags.insert("area", "yes");
        assert!(!is_routable_line(&tags, true));
        assert!(is_routable_line(&tags, false));
        tags.insert("highway", "pedestrian");
        assert!(is_routable_line(&tags, true));
        tags.insert("area:highway", "pedestrian");
        assert!(!is_routable_line(&tags, true));
    }

    #[test]
    fn folds_link_classes_for_graph_hierarchy() {
        let mut tags = Tags::new();
        tags.insert("highway", "motorway_link");
        assert_eq!(road_class(&tags), "motorway");
        assert_eq!(road_sort_order(road_class(&tags)), 240);
    }

    /// The package compiled from `pbf` with the road segments reaching the
    /// builder in `order`.
    fn package_in_segment_order(pbf: &Path, dir: &Path, order: SegmentOrder) -> PathBuf {
        std::fs::create_dir_all(dir).unwrap();
        let request = RouteArchiveCompileRequest {
            graph: RoadGraphCompileRequest {
                input_pbf_path: pbf.to_path_buf(),
                temp_dir: dir.join("scratch"),
                graph_id: "segment-order".to_string(),
                country_code: None,
                build_profile: IngestBuildProfile::DesktopFull,
                segment_order: order,
            },
            route_archive_path: dir.join("routing.mrt"),
            threads: Some(2),
            package_options: PackageBuildOptions::default(),
        };
        compile_route_archive(&request, &|| false, &mut |_| {}).unwrap();
        request.route_archive_path
    }

    /// What a tiled compile needs from the id scheme: every order and id in
    /// a package follows from the graph, not from the order its parts were
    /// built in. Shuffling the segments the builder gets renumbers every
    /// road graph node, edge and shared info and reorders plaza rings, and
    /// must leave every section alone but the metadata. That holds the
    /// build's quality report, whose examples are the first ones the build
    /// met, named by its own ids (a tiled compile reports its own).
    #[test]
    fn packages_do_not_depend_on_the_order_segments_reach_the_builder() {
        for fixture in ["monaco.osm.pbf", "krakow-centre.osm.pbf"] {
            let pbf = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../fixtures")
                .join(fixture);
            let root = tempfile::tempdir().unwrap();
            let reference = package_in_segment_order(
                &pbf,
                &root.path().join("topology"),
                SegmentOrder::Topology,
            );
            let expected = wayfarer_package::format::PackageFile::verify(&reference).unwrap();
            for seed in [1, 2] {
                let shuffled = package_in_segment_order(
                    &pbf,
                    &root.path().join(format!("shuffled-{seed}")),
                    SegmentOrder::Shuffled(seed),
                );
                let actual = wayfarer_package::format::PackageFile::verify(&shuffled).unwrap();
                let differing = expected
                    .sections
                    .iter()
                    .zip(&actual.sections)
                    .filter(|(a, b)| a != b && a.name != "Metadata")
                    .map(|(a, _)| a.name)
                    .collect::<Vec<_>>();
                assert!(
                    differing.is_empty(),
                    "{fixture}, seed {seed}: sections differ: {differing:?}"
                );
            }
        }
    }

    #[test]
    fn monaco_packages_are_deterministic_and_behaviorally_stable() {
        let pbf = std::env::var_os("WAYFARER_ROUTE_PARITY_PBF")
            .map(PathBuf::from)
            .unwrap_or_else(|| {
                PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../fixtures/monaco.osm.pbf")
            });
        assert!(pbf.is_file(), "missing Monaco parity PBF at {:?}", pbf);
        let root = tempfile::tempdir().unwrap();
        let first = root.path().join("first");
        let second = root.path().join("second");
        std::fs::create_dir_all(&first).unwrap();
        std::fs::create_dir_all(&second).unwrap();

        for output in [&first, &second] {
            compile(
                &CompileRequest::new(pbf.clone(), output.join("routing.mrt"))
                    .scratch_dir(output.join("scratch"))
                    .graph_id("monaco-parity")
                    .country("MC"),
                &CancelToken::new(),
                |_| {},
            )
            .unwrap();
            assert!(!output.join("scratch").exists());
        }

        assert_eq!(
            std::fs::read(first.join("routing.mrt")).unwrap(),
            std::fs::read(second.join("routing.mrt")).unwrap(),
            "routing packages differ across identical builds"
        );

        let engines = [&first, &second].map(|output| package_engine(&output.join("routing.mrt")));
        // Re-baselined on 2026-11-17: the goldens went stale while this test
        // was ignored (later engine and compiler changes moved these routes
        // and their edge ids); the package still matches
        // fixtures/monaco-info.json. The totals must match to f32 rounding
        // and the edge fingerprints exactly. Fingerprints re-recorded on
        // 2026-11-21 when edge ids became positions in the package order
        // (`package_build::renumber_in_package_order`): the same routes,
        // with the same totals, under new ids.
        let golden: [(f64, f64, f64, usize, u64); 8] = [
            (
                2047.6120446791913,
                190.58227701867665,
                217.15802776261896,
                57,
                2361029945959432928,
            ),
            (
                1945.352435067371,
                1373.1899599859382,
                1461.2465279433395,
                72,
                18153314860976308213,
            ),
            (
                3072.489087949827,
                208.33119192260511,
                252.9142019371356,
                89,
                13544991242613529721,
            ),
            (
                2047.6120446791913,
                215.8750875036877,
                337.44146348403336,
                57,
                2361029945959432928,
            ),
            (
                2401.0263224734517,
                201.42088467548498,
                196.40290802292756,
                62,
                17325639605596943052,
            ),
            (
                1934.7695955783877,
                1365.719717485633,
                1458.796490425792,
                72,
                3222675881509893555,
            ),
            (
                2401.0263224734517,
                174.75927405121425,
                201.07278831440556,
                62,
                17325639605596943052,
            ),
            (
                2348.2805601252767,
                254.42299810042812,
                293.3094785621453,
                57,
                17410398497319394578,
            ),
        ];
        let mut golden_index = 0usize;
        for (origin, destination) in [
            (
                GeoPoint::new(43.7280, 7.4145),
                GeoPoint::new(43.7395, 7.4270),
            ),
            (
                GeoPoint::new(43.7395, 7.4270),
                GeoPoint::new(43.7280, 7.4145),
            ),
        ] {
            for profile in [
                MobilityProfile::Wheeled,
                MobilityProfile::OnFoot,
                MobilityProfile::Emergency,
                MobilityProfile::Tracked,
            ] {
                let route = |engine: &RoutingEngine| {
                    engine
                        .route_leg(LegRequest::new(origin, destination, profile))
                        .unwrap()
                };
                let first_directed = route(&engines[0]);
                let second_directed = route(&engines[1]);
                let (distance, duration, cost, edge_count, edge_fingerprint) = golden[golden_index];
                golden_index += 1;
                let close = |actual: f64, expected: f64| {
                    (actual - expected).abs() <= 1e-5 * expected.abs().max(1.0)
                };
                assert!(close(first_directed.distance_m, distance), "{profile:?}");
                assert!(close(first_directed.duration_s, duration), "{profile:?}");
                assert!(close(first_directed.cost, cost), "{profile:?}");
                assert_eq!(first_directed.edge_ids.len(), edge_count);
                assert_eq!(
                    stable_edge_fingerprint(&first_directed.edge_ids),
                    edge_fingerprint
                );
                assert_eq!(first_directed.edge_ids, second_directed.edge_ids);
                assert_eq!(
                    first_directed.distance_m.to_bits(),
                    second_directed.distance_m.to_bits()
                );
                assert_eq!(
                    first_directed.duration_s.to_bits(),
                    second_directed.duration_s.to_bits()
                );
                assert_eq!(
                    first_directed.cost.to_bits(),
                    second_directed.cost.to_bits()
                );
            }
        }
    }

    fn stable_edge_fingerprint(edge_ids: &[u64]) -> u64 {
        let mut hash = 14_695_981_039_346_656_037u64;
        for edge_id in edge_ids {
            for byte in edge_id.to_le_bytes() {
                hash ^= u64::from(byte);
                hash = hash.wrapping_mul(1_099_511_628_211);
            }
        }
        hash
    }
}

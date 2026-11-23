//! Builds a routing package straight from an in-memory `RoadGraph`.
//!
//! Every directed edge is costed with the default costing model
//! (`wayfarer_costing`) from the road graph's own records, the costing
//! the reference oracle (`wayfarer::reference`) applies to the same
//! records, so the stored weights are the record weights cast to f32 by
//! construction. Nodes are the
//! endpoints of edges, ordered along a Hilbert curve over a fixed world
//! lattice; edges are grouped by source node and ordered by edge id within a
//! node, and shared infos by id. A compile first gives the road graph its
//! package ids ([`renumber_in_package_order`]), so those ids, and with them
//! every order and id in the package, follow from the graph itself rather
//! than from the order the compiler built it in.

mod order;
mod pieces;
mod request;

use order::package_node_key;
pub use order::renumber_in_package_order;

/// Reports named checkpoints of a package build (with their live heap) when
/// set; the compiler's opt-in `benchmark-profiling` build installs a reporter.
/// Unset (the normal case) it costs one atomic load per checkpoint.
pub static PACKAGE_BUILD_CHECKPOINT: std::sync::OnceLock<fn(&str)> = std::sync::OnceLock::new();

/// `WAYFARER_DEBUG`: print build sub-phase timings and section stats.
/// Read only with the `debug-env` feature, so a library build never prints.
fn trace() -> bool {
    cfg!(feature = "debug-env") && std::env::var_os("WAYFARER_DEBUG").is_some()
}

use std::collections::{BTreeMap, HashMap};
use std::io::Write;
use std::time::Instant;

use crate::CompileStage;
use crate::job::Job;
use serde::{Deserialize, Serialize};

use rayon::prelude::*;
use rustc_hash::FxHashMap;
use wayfarer_package::{RoadEdge, SharedEdgeInfo, TraversalDirection};
use wayfarer_road_graph::{
    EdgeColumn, NodeColumn, ResolvedEdgeRestriction, RoadGraph, RoadGraphPackageManifest,
    RoadGraphQualityManifest, RoadNetwork, SharedInfoIndex, opposing_edge_indices_of,
};

use wayfarer_costing::{
    COSTING_MODEL_VERSION, CostModel, EdgeCostClasses, EdgeCostDecision, EdgeCostFlags,
    EdgeCostInputs, NoOverrides, RouteCostingOptions,
};
use wayfarer_package::MobilityProfile;
use wayfarer_package::format::{
    EDGE_FLAG_BARRIER_AT_TARGET, EDGE_FLAG_COMPLEX_RESTRICTION_FIRST,
    EDGE_FLAG_COMPLEX_RESTRICTION_TO, EDGE_FLAG_LINK, EDGE_FLAG_SHAPE_REVERSED,
    EDGE_FLAG_SIMPLE_RESTRICTION_FROM, FORMAT_3_DRAFT_REVISION, MAX_COST_TUPLES, NO_COLUMN,
    NO_HEADING, NO_TOKEN, NO_TWIN, PROFILE_SLOTS, PackageCounts, PackageFile, PackageFormat,
    PackageLayout, PackageParams, PackageWriter, RoutingPackageError, RoutingPackageMetadata,
    SHAPE_START_STRIDE, Section, WAY_ID_WIDE, e7_from_fixed_xyz, edge_id_span_limit, encode_shape,
    fixed_xyz, is_exact_e7, profile_slot, reverse_heading, to_e7, unit_xyz, xyz_from_fixed,
};
use wayfarer_package::format::{junction, road_type_rank};
use wayfarer_package::request::derived_opposing;
use wayfarer_package::sar::{SarPackageInfo, edge_sar_bits};

const ALL_PROFILES: [MobilityProfile; PROFILE_SLOTS] = [
    MobilityProfile::Default,
    MobilityProfile::Wheeled,
    MobilityProfile::Tracked,
    MobilityProfile::Emergency,
    MobilityProfile::OnFoot,
    MobilityProfile::Legged,
];

#[derive(Debug, Clone)]
pub struct PackageBuildOptions {
    /// Profiles that receive a cost/duration column. `Default` always shares
    /// the `Wheeled` column because both use the same costing policy.
    pub profiles: Vec<MobilityProfile>,
    /// Store the CCH elimination order (`Section::CchRank`), from which the
    /// engine builds its potentials (the default). Without it searches
    /// bound with the straight line alone (tests and benchmarks).
    pub cch_order: bool,
    /// The package format to write: format 2 (the default), or format 3's
    /// current draft, which only a build of the same draft revision opens
    /// (`docs/design/format-3.md`). Format 3 stays behind this switch until
    /// its release.
    pub package_format: PackageFormat,
}

impl Default for PackageBuildOptions {
    fn default() -> Self {
        Self {
            profiles: vec![MobilityProfile::Wheeled, MobilityProfile::OnFoot],
            cch_order: true,
            package_format: PackageFormat::V2,
        }
    }
}

/// What the package build made: counts of what it stored and its timings.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", default)]
pub struct PackageBuildReport {
    pub node_count: usize,
    pub edge_count: usize,
    pub shared_info_count: usize,
    pub simple_restriction_count: usize,
    pub complex_restriction_count: usize,
    pub skipped_restriction_count: usize,
    pub skipped_simple_missing_to_edge: usize,
    pub byte_len: usize,
    pub heuristic_cost_per_m: Vec<(MobilityProfile, f64)>,
    /// Points across all stored shapes (one per shared info).
    pub shape_point_count: usize,
    /// Shapes with a point that is not an exact E7 value (stored exactly too).
    pub shape_inexact_count: usize,
    /// Shapes stored with explicit endpoints (they differ from the nodes').
    pub shape_explicit_count: usize,
    pub shape_byte_len: usize,
    /// Nodes whose position is not an exact E7 value (positions stored as f64).
    pub node_inexact_count: usize,
    /// Bytes of the request sections (`request.rs`).
    pub request_byte_len: usize,
    /// Edges whose opposing edge the adjacency rule does not reproduce.
    pub request_opposing_exception_count: usize,
    /// Edges without a twin (`Section::LoneInEdge`).
    #[serde(default)]
    pub lone_edge_count: usize,
    /// Edges whose mode mask or tier is stored as an exception.
    pub mode_mask_exception_count: usize,
    pub tier_exception_count: usize,
    /// Distinct (cost flags, cost classes, speeds) tuples of the shared infos.
    pub cost_tuple_count: usize,
    /// Shared infos whose way id does not fit `u32` (synthetic negative ids).
    pub way_id_wide_count: usize,
    /// Paths R5 `vehicle_on_paths` can open (physically drivable, no legal
    /// vehicle access), by width class: unknown, under 2.5 m, 2.5–3 m, 3 m
    /// or more.
    pub sar_path_edge_count_by_width: [usize; 4],
    /// Barrier nodes: with an effect in normal routing or a SAR class.
    pub barrier_node_count: usize,
    pub assemble_ms: u128,
    /// Computing the CCH order (nested dissection).
    pub cch_order_ms: u128,
    pub request_ms: u128,
    pub total_ms: u128,
}

/// The road graph's normalized identity tokens (corridor turn discounts
/// compare names, refs and route refs by them). Token equality is exactly
/// the trimmed, ASCII-case-insensitive string equality the turn costing
/// applies, and tokens are numbered in a fixed order over the shared infos.
struct TokenInterner {
    normalized: HashMap<String, u32>,
}

impl TokenInterner {
    fn from_shared_infos<'a>(shared_edge_infos: impl Iterator<Item = &'a SharedEdgeInfo>) -> Self {
        let mut interner = Self {
            normalized: HashMap::new(),
        };
        for info in shared_edge_infos {
            interner.intern(info.ref_name.as_deref());
            interner.intern(info.name.as_deref());
            for route_ref in &info.route_refs {
                interner.intern(Some(route_ref));
            }
        }
        interner
    }

    fn normalize(value: Option<&str>) -> Option<String> {
        let trimmed = value?.trim();
        (!trimmed.is_empty()).then(|| trimmed.to_ascii_lowercase())
    }

    fn intern(&mut self, value: Option<&str>) {
        let Some(normalized) = Self::normalize(value) else {
            return;
        };
        if !self.normalized.contains_key(&normalized) {
            let token = u32::try_from(self.normalized.len()).expect("token table overflow");
            self.normalized.insert(normalized, token);
        }
    }

    fn token(&self, value: Option<&str>) -> Option<u32> {
        Self::normalize(value).and_then(|normalized| self.normalized.get(&normalized).copied())
    }
}

#[cfg(test)]
thread_local! {
    /// Tests: store node E7 positions even when they are derivable, to
    /// exercise both layouts.
    static STORE_DERIVABLE: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

fn store_derivable() -> bool {
    #[cfg(test)]
    return STORE_DERIVABLE.with(std::cell::Cell::get);
    #[cfg(not(test))]
    false
}

/// The sub-phases of the package build before the CCH order, in order: each
/// reports its share of [`CompileStage::BuildPackage`] as it ends.
const BUILD_STEPS: [&str; 8] = [
    "nodes",
    "edge order",
    "edge fields",
    "costs",
    "restrictions",
    "shapes",
    "request sections",
    "adjacency, positions, heuristic",
];

/// The error a build returns when its job was cancelled; the caller reads the
/// job, not this error, to tell a cancel from a failure.
pub(crate) fn cancelled_error() -> RoutingPackageError {
    RoutingPackageError::Invalid("the build was cancelled".into())
}

/// Ends sub-phases: checks the job for a cancel, reports progress, and prints
/// timings with `WAYFARER_DEBUG` (feature `debug-env`).
struct PhaseClock<'j, 'a> {
    enabled: bool,
    last: Instant,
    job: &'j Job<'a>,
}

impl<'j, 'a> PhaseClock<'j, 'a> {
    fn new(job: &'j Job<'a>) -> Self {
        Self {
            enabled: trace(),
            last: Instant::now(),
            job,
        }
    }

    fn lap(&mut self, phase: &str) -> Result<(), RoutingPackageError> {
        if let Some(step) = BUILD_STEPS.iter().position(|step| *step == phase) {
            self.job.report(
                CompileStage::BuildPackage,
                Some((step + 1) as f64 / BUILD_STEPS.len() as f64),
                0,
                format!("Built {phase}"),
            );
        }
        if self.job.cancelled() {
            return Err(cancelled_error());
        }
        self.time(phase);
        Ok(())
    }

    /// Ends a sub-phase without checking for a cancel (once the package is
    /// in place, a cancel is too late).
    fn time(&mut self, phase: &str) {
        if self.enabled {
            eprintln!(
                "package build: {phase} {:.0} ms",
                self.last.elapsed().as_secs_f64() * 1e3
            );
        }
        if let Some(hook) = PACKAGE_BUILD_CHECKPOINT.get() {
            hook(phase);
        }
        self.last = Instant::now();
    }
}

/// Runs `build` with a job that is never cancelled and reports nowhere.
fn without_job<T>(build: impl FnOnce(&Job<'_>) -> T) -> T {
    let never = || false;
    let mut ignore = |_: &crate::job::Report| {};
    build(&Job::new(&never, &mut ignore))
}

/// Profiles whose mode mask overlaps an edge's or a restriction's (none for
/// zero: a restriction that binds no routed mode binds no profile).
fn profile_bits(mobility_mode_mask: u32) -> u8 {
    ALL_PROFILES.iter().fold(0u8, |bits, profile| {
        if mobility_mode_mask & profile.mobility_mode_mask() != 0 {
            bits | (1 << profile_slot(*profile))
        } else {
            bits
        }
    })
}

fn restriction_mobility_mode_mask(restriction: &ResolvedEdgeRestriction) -> u32 {
    match restriction {
        ResolvedEdgeRestriction::Simple {
            mobility_mode_mask, ..
        }
        | ResolvedEdgeRestriction::Complex {
            mobility_mode_mask, ..
        } => *mobility_mode_mask,
    }
}

/// Road graph package metadata for `graph`: its manifest's schema, hash,
/// counts and quality when the caller has one, else the graph's own counts.
pub fn package_metadata(
    graph: &RoadGraph,
    manifest: Option<&RoadGraphPackageManifest>,
) -> RoutingPackageMetadata<RoadGraphQualityManifest> {
    let mode_mask_union = graph
        .graph
        .edge_weights()
        .fold(0, |mask, edge| mask | edge.mobility_mode_mask);
    match manifest {
        Some(manifest) => RoutingPackageMetadata {
            graph_id: graph.graph_id.clone(),
            country_code: graph.country_code.clone(),
            road_graph_schema_version: manifest.graph_schema_version,
            road_graph_blake3_hash_hex: manifest.blake3_hash_hex.clone(),
            node_count: manifest.node_count,
            directed_edge_count: manifest.directed_edge_count,
            shared_edge_info_count: manifest.shared_edge_info_count,
            mode_mask_union,
            quality: manifest.quality.clone(),
            costing_model_version: COSTING_MODEL_VERSION,
            sar: SarPackageInfo::default(),
        },
        None => RoutingPackageMetadata {
            graph_id: graph.graph_id.clone(),
            country_code: graph.country_code.clone(),
            road_graph_schema_version: wayfarer_road_graph::ROAD_GRAPH_SNAPSHOT_VERSION,
            road_graph_blake3_hash_hex: String::new(),
            node_count: graph.node_count(),
            directed_edge_count: graph.edge_count(),
            shared_edge_info_count: graph.shared_edge_infos.len(),
            mode_mask_union,
            quality: Default::default(),
            costing_model_version: COSTING_MODEL_VERSION,
            sar: SarPackageInfo::default(),
        },
    }
}

/// Builds the package sections of `graph`.
pub fn build_package(
    graph: &RoadGraph,
    metadata: &RoutingPackageMetadata<RoadGraphQualityManifest>,
    options: &PackageBuildOptions,
) -> Result<(PackageWriter, PackageBuildReport), RoutingPackageError> {
    without_job(|job| {
        build_package_from(GraphSource::Borrowed(graph), metadata, options, job, None)
    })
}

/// The road graph a package is built from: borrowed (and copied), or owned,
/// so that the build can reorder it and free each column once the sections
/// derived from it are built.
enum GraphSource<'a> {
    Borrowed(&'a RoadGraph),
    Owned(Box<RoadGraph>),
}

/// Builds the sections of `source`'s graph. With a `spill` file each section
/// goes there as soon as it is built and nothing later reads it (see
/// [`PackageWriter::spilling`]); the build computes the sections in an order
/// that frees most of them before the request sections (the build's peak)
/// and the CCH order.
fn build_package_from(
    source: GraphSource<'_>,
    metadata: &RoutingPackageMetadata<RoadGraphQualityManifest>,
    options: &PackageBuildOptions,
    job: &Job<'_>,
    spill: Option<std::fs::File>,
) -> Result<(PackageWriter, PackageBuildReport), RoutingPackageError> {
    // The graph is reordered into the package's order and its columns are
    // freed as the sections built from them are put; a borrowed graph is
    // copied first.
    let graph = match source {
        GraphSource::Borrowed(graph) => Box::new(graph.clone()),
        GraphSource::Owned(graph) => graph,
    };
    let mut writer = match spill {
        Some(file) => PackageWriter::spilling(file),
        None => PackageWriter::in_memory(),
    }
    .with_format(options.package_format);
    let implied_ids = options.package_format == PackageFormat::V3;
    writer.put_metadata(metadata);
    let started = Instant::now();
    job.report(
        CompileStage::BuildPackage,
        Some(0.0),
        0,
        "Costing edges and building the package sections",
    );
    let mut clock = PhaseClock::new(job);
    let mut report = PackageBuildReport::default();

    // Columns: one per distinct costing policy requested.
    let mut column_profiles = Vec::<MobilityProfile>::new();
    let mut profile_columns = [NO_COLUMN; PROFILE_SLOTS];
    for profile in &options.profiles {
        let canonical = match profile {
            MobilityProfile::Default => MobilityProfile::Wheeled,
            other => *other,
        };
        if !column_profiles.contains(&canonical) {
            column_profiles.push(canonical);
        }
    }
    for (column, profile) in column_profiles.iter().enumerate() {
        profile_columns[profile_slot(*profile)] = column as u8;
        if *profile == MobilityProfile::Wheeled {
            profile_columns[profile_slot(MobilityProfile::Default)] = column as u8;
        }
    }

    let RoadGraph {
        graph: mut network,
        shared_edge_infos: shared_infos,
        resolved_restrictions,
        ..
    } = *graph;
    let raw_edge_count = network.edge_count();
    let raw_node_count = network.node_count();
    if raw_edge_count == 0 {
        return Err(RoutingPackageError::Unsupported(
            "the graph has no edges".into(),
        ));
    }
    if raw_edge_count >= u32::MAX as usize || raw_node_count >= u32::MAX as usize {
        return Err(RoutingPackageError::Unsupported(
            "the graph exceeds u32 indexing".into(),
        ));
    }
    // Shared infos by id (the last of a duplicated id wins, as in the road
    // graph's own index). Every edge must resolve its shared info.
    let shared_by_id = SharedInfoIndex::new(&shared_infos);
    let edge_shared_raw = (0..raw_edge_count)
        .into_par_iter()
        .map(|raw_edge| {
            shared_by_id
                .get(network.edge_shared_info_id(raw_edge))
                .map(|index| index as u32)
        })
        .collect::<Option<Vec<u32>>>()
        .ok_or_else(|| {
            RoutingPackageError::Unsupported("an edge's shared edge info is missing".into())
        })?;
    drop(shared_by_id);
    // Correlation pairs opposing edges by (shared info, segment, direction),
    // the last edge of a key winning: found in the graph's own edge order,
    // before the edges are reordered.
    let correlation_raw = opposing_edge_indices_of(&network);
    // Each component's node count over the whole graph, edgeless nodes too.
    let mut component_sizes = FxHashMap::<u32, usize>::default();
    for node in 0..raw_node_count {
        if let Some(component) = network.node_component_id(node) {
            *component_sizes.entry(component).or_default() += 1;
        }
    }
    // Not read by the package.
    network.release_node_columns(&[
        NodeColumn::SourceNodeId,
        NodeColumn::EdgeCounts,
        NodeColumn::Junction,
        NodeColumn::RoundaboutEntryCount,
    ]);
    network.release_edge_columns(&[
        EdgeColumn::BaseCost,
        EdgeColumn::OsmModeMasks,
        EdgeColumn::SegmentIndex,
    ]);

    // Nodes: the endpoints of edges, ordered along a Hilbert curve over the world
    // lattice (see `package_node_key`), then by node id: after
    // `renumber_in_package_order`, the order the ids already follow.
    let mut node_used = vec![false; raw_node_count];
    for (source, target) in network.sources().iter().zip(network.targets()) {
        node_used[*source as usize] = true;
        node_used[*target as usize] = true;
    }
    let mut nodes = (0..raw_node_count as u32)
        .filter(|raw| node_used[*raw as usize])
        .collect::<Vec<_>>();
    drop(node_used);
    nodes.par_sort_by_cached_key(|raw| {
        let (lat, lng) = network.node_lat_lng(*raw as usize);
        (
            package_node_key(lat, lng, None),
            network.node_id(*raw as usize),
            *raw,
        )
    });
    let node_count = nodes.len();

    // Edges: grouped by compact source node, ordered by edge id within a node
    // (the adjacency order of the road graph's index, which keeps
    // tie-breaking comparable). Edge ids are unique (checked below), so the
    // order is total.
    let mut edge_raw = (0..raw_edge_count as u32).collect::<Vec<_>>();
    {
        let mut raw_node_to_compact = vec![u32::MAX; raw_node_count];
        for (index, raw) in nodes.iter().enumerate() {
            raw_node_to_compact[*raw as usize] = index as u32;
        }
        let sources = network.sources();
        let network = &network;
        edge_raw.par_sort_unstable_by_key(|&raw| {
            (
                raw_node_to_compact[sources[raw as usize] as usize],
                network.edge_id(raw as usize),
                raw,
            )
        });
    }
    // The graph in package order: node `i` and edge `i` are compact node and
    // edge `i` from here on.
    let edge_shared_raw = edge_raw
        .par_iter()
        .map(|raw| edge_shared_raw[*raw as usize])
        .collect::<Vec<_>>();
    let correlation_opposing = {
        let mut compact_of_raw = vec![0u32; raw_edge_count];
        for (compact, raw) in edge_raw.iter().enumerate() {
            compact_of_raw[*raw as usize] = compact as u32;
        }
        edge_raw
            .par_iter()
            .map(|raw| match correlation_raw[*raw as usize] {
                u32::MAX => u32::MAX,
                opposing => compact_of_raw[opposing as usize],
            })
            .collect::<Vec<_>>()
    };
    drop(correlation_raw);
    network.permute_edges(&edge_raw);
    drop(edge_raw);
    network.permute_nodes(&nodes);
    drop(nodes);
    let edge_count = network.edge_count();
    let edge_source = network.take_sources();
    let edge_target = network.take_targets();

    // Barrier nodes: compact nodes whose barrier affects some profile
    // (`BarrierAccess`) or vehicles under SAR `barriers` (a class).
    let mut barrier_node = Vec::new();
    let mut barrier_class = Vec::new();
    let mut barrier_access = Vec::new();
    for compact in 0..node_count {
        let (class, access) = network.node_barrier(compact);
        if class != 0 || access != 0 {
            barrier_node.push(compact as u32);
            barrier_class.push(class);
            barrier_access.push(access);
        }
    }
    network.release_node_columns(&[NodeColumn::Barrier]);
    report.barrier_node_count = barrier_node.len();
    writer.put(Section::BarrierClass, barrier_class);
    writer.put(Section::BarrierAccess, barrier_access);
    clock.lap("nodes")?;

    // Edge ids: a dense lookup from (id - base) to the compact edge.
    let edge_id_base = (0..edge_count)
        .map(|edge| network.edge_id(edge))
        .min()
        .unwrap_or(0);
    let edge_id_span = (0..edge_count)
        .map(|edge| network.edge_id(edge) - edge_id_base)
        .max()
        .map_or(0, |span| span as usize + 1);
    if edge_id_span > edge_id_span_limit(edge_count) {
        return Err(RoutingPackageError::Unsupported(format!(
            "edge ids span {edge_id_span} values for {edge_count} edges"
        )));
    }
    let mut edge_id_lookup = vec![u32::MAX; edge_id_span];
    for edge in 0..edge_count {
        let id = network.edge_id(edge);
        let slot = &mut edge_id_lookup[(id - edge_id_base) as usize];
        if *slot != u32::MAX {
            return Err(RoutingPackageError::Unsupported(format!(
                "edge id {id} is not unique"
            )));
        }
        *slot = edge as u32;
    }
    clock.lap("edge order")?;

    // Shared infos used by some edge, in shared-info id order, so a shared
    // info's id follows from its index (see `request::shared_info_id`).
    let mut shared_used = vec![false; shared_infos.len()];
    for raw in &edge_shared_raw {
        shared_used[*raw as usize] = true;
    }
    let mut shared_raw = (0..shared_infos.len() as u32)
        .filter(|raw| shared_used[*raw as usize])
        .collect::<Vec<_>>();
    drop(shared_used);
    shared_raw.sort_unstable_by_key(|raw| shared_infos.at(*raw as usize).shared_edge_info_id);
    let mut shared_compact_by_raw = vec![u32::MAX; shared_infos.len()];
    for (shared, raw) in shared_raw.iter().enumerate() {
        shared_compact_by_raw[*raw as usize] = shared as u32;
    }
    // The shape of a shared info runs between the nodes of its first edge
    // (in compact order); its endpoints are implicit when every edge using
    // it agrees on those nodes' positions.
    const NO_NODES: (u32, u32) = (u32::MAX, u32::MAX);
    let mut si_nodes = vec![NO_NODES; shared_raw.len()];
    let mut si_consistent = vec![true; shared_raw.len()];
    let mut edge_shared_info = Vec::with_capacity(edge_count);
    let node_e7_of = |network: &RoadNetwork, compact: u32| {
        let (lat, lng) = network.node_lat_lng(compact as usize);
        (to_e7(lat), to_e7(lng))
    };
    for edge in 0..edge_count {
        let shared = shared_compact_by_raw[edge_shared_raw[edge] as usize];
        let backward = network.edge_traversal_direction(edge) == TraversalDirection::Backward;
        let shape_nodes = if backward {
            (edge_target[edge], edge_source[edge])
        } else {
            (edge_source[edge], edge_target[edge])
        };
        let first = &mut si_nodes[shared as usize];
        if *first == NO_NODES {
            *first = shape_nodes;
        }
        let (from, to) = *first;
        if node_e7_of(&network, from) != node_e7_of(&network, shape_nodes.0)
            || node_e7_of(&network, to) != node_e7_of(&network, shape_nodes.1)
        {
            si_consistent[shared as usize] = false;
        }
        edge_shared_info.push(shared);
    }
    drop(shared_compact_by_raw);
    drop(edge_shared_raw);
    let shared_count = shared_raw.len();
    let info_of = |shared: u32| shared_infos.at(shared_raw[shared as usize] as usize);

    // Per-edge fields.
    let edge_length = network.take_distances();
    let heading_start = (0..edge_count)
        .into_par_iter()
        .map(|edge| network.edge_start_heading_deg(edge).unwrap_or(NO_HEADING))
        .collect::<Vec<_>>();
    let heading_end = (0..edge_count)
        .into_par_iter()
        .map(|edge| network.edge_end_heading_deg(edge).unwrap_or(NO_HEADING))
        .collect::<Vec<_>>();
    network.release_edge_columns(&[EdgeColumn::Headings]);
    // Profile bits (legal access), plus the SAR bits: physically drivable,
    // and against a vehicle one-way (`wayfarer_package::sar`).
    let edge_profile_access = edge_shared_info
        .par_iter()
        .enumerate()
        .map(|(edge, shared)| {
            let mobility_mode_mask = network.edge_mobility_mode_mask(edge);
            profile_bits(mobility_mode_mask)
                | edge_sar_bits(
                    info_of(*shared).attrs,
                    network.edge_traversal_direction(edge),
                    mobility_mode_mask,
                )
        })
        .collect::<Vec<_>>();
    writer.put(
        Section::SharedInfoLegalReason,
        (0..shared_count as u32)
            .map(|shared| info_of(shared).attrs.legal_reason)
            .collect::<Vec<u8>>(),
    );
    writer.put(
        Section::EdgeJunction,
        (0..edge_count)
            .into_par_iter()
            .map(|edge| {
                junction::encode(
                    network.edge_end_control(edge),
                    network.edge_end_minor_approach(edge),
                )
            })
            .collect::<Vec<u8>>(),
    );
    writer.put(
        Section::EdgeRoadRank,
        edge_shared_info
            .par_iter()
            .map(|shared| road_type_rank(info_of(*shared).attrs.road_type))
            .collect::<Vec<_>>(),
    );
    // Inferred speed + 1, 0 for none.
    let edge_inferred_speed = (0..edge_count)
        .into_par_iter()
        .map(|edge| match network.edge_inferred_speed_kph(edge) {
            None => Ok(0),
            Some(speed) => speed.checked_add(1).ok_or_else(|| {
                RoutingPackageError::Unsupported(format!("inferred speed {speed} is out of range"))
            }),
        })
        .collect::<Result<Vec<u16>, _>>()?;
    writer.put(Section::EdgeInferredSpeed, edge_inferred_speed);
    let mut edge_flags = edge_shared_info
        .par_iter()
        .enumerate()
        .map(|(edge, shared)| {
            let mut flags = if info_of(*shared).attrs.is_link {
                EDGE_FLAG_LINK
            } else {
                0
            };
            if network.edge_traversal_direction(edge) == TraversalDirection::Backward {
                flags |= EDGE_FLAG_SHAPE_REVERSED;
            }
            flags
        })
        .collect::<Vec<_>>();
    for (flags, target) in edge_flags.iter_mut().zip(&edge_target) {
        if barrier_node.binary_search(target).is_ok() {
            *flags |= EDGE_FLAG_BARRIER_AT_TARGET;
        }
    }
    writer.put(Section::BarrierNode, barrier_node);

    // Costing attributes (derived columns read these): a table of the
    // distinct (flags, classes, speeds) tuples in order of first use, and each
    // shared info's row of it.
    let classes = (0..shared_count as u32)
        .into_par_iter()
        .map(|shared| EdgeCostClasses::from_shared_info(info_of(shared).attrs))
        .collect::<Vec<_>>();
    let shared_cost_inputs = (0..shared_count as u32)
        .into_par_iter()
        .map(|shared| {
            let info = info_of(shared).attrs;
            let speed = |value: Option<f32>| value.unwrap_or(f32::NAN);
            (
                EdgeCostFlags::from_shared_info(info).0,
                classes[shared as usize].pack(),
                [
                    speed(info.maxspeed_kph),
                    speed(info.free_flow_speed_kph),
                    speed(info.constrained_speed_kph),
                ],
            )
        })
        .collect::<Vec<_>>();
    let mut tuple_rows = FxHashMap::<(u8, u32, [u32; 3]), u16>::default();
    let (mut si_cost_flags, mut si_cost_classes, mut si_speeds) =
        (Vec::new(), Vec::new(), Vec::new());
    let mut si_cost_tuple = Vec::with_capacity(shared_count);
    for (flags, classes, speeds) in &shared_cost_inputs {
        let key = (*flags, *classes, speeds.map(f32::to_bits));
        let next = si_cost_flags.len();
        let row = match tuple_rows.get(&key) {
            Some(row) => *row,
            None => {
                if next >= MAX_COST_TUPLES {
                    return Err(RoutingPackageError::Unsupported(format!(
                        "more than {MAX_COST_TUPLES} distinct costing tuples"
                    )));
                }
                si_cost_flags.push(*flags);
                si_cost_classes.push(*classes);
                si_speeds.extend_from_slice(speeds);
                tuple_rows.insert(key, next as u16);
                next as u16
            }
        };
        si_cost_tuple.push(row);
    }
    drop(shared_cost_inputs);
    drop(tuple_rows);
    report.cost_tuple_count = si_cost_flags.len();
    writer.put(Section::SharedInfoCostFlags, si_cost_flags);
    writer.put(Section::SharedInfoCostClasses, si_cost_classes);
    writer.put(Section::SharedInfoSpeeds, si_speeds);
    writer.put(Section::SharedInfoCostTuple, si_cost_tuple);
    writer.put(
        Section::ConditionalEdges,
        (0..edge_count as u32)
            .filter(|edge| {
                !info_of(edge_shared_info[*edge as usize])
                    .attrs
                    .conditional_restrictions
                    .is_empty()
            })
            .collect::<Vec<_>>(),
    );

    // Identity tokens per shared info.
    // Numbered over the package's shared infos in order, so the numbers
    // follow from the package (not from every shared info the graph holds,
    // in the graph's build order).
    let interner = TokenInterner::from_shared_infos(
        (0..shared_count as u32).map(|shared| info_of(shared).attrs),
    );
    // Way ids: `u32`, and the others (synthetic negative ids) apart.
    let (mut si_way_wide_shared, mut si_way_wide) = (Vec::new(), Vec::new());
    let si_way = (0..shared_count as u32)
        .map(|shared| {
            let id = info_of(shared).osm_way_id;
            match u32::try_from(id) {
                Ok(narrow) if narrow != WAY_ID_WIDE => narrow,
                _ => {
                    si_way_wide_shared.push(shared);
                    si_way_wide.push(id);
                    WAY_ID_WIDE
                }
            }
        })
        .collect::<Vec<_>>();
    report.way_id_wide_count = si_way_wide.len();
    writer.put(Section::SharedInfoWay, si_way);
    writer.put(Section::SharedInfoWayWideShared, si_way_wide_shared);
    writer.put(Section::SharedInfoWayWide, si_way_wide);
    let si_name = (0..shared_count as u32)
        .map(|shared| {
            interner
                .token(info_of(shared).attrs.name.as_deref())
                .unwrap_or(NO_TOKEN)
        })
        .collect::<Vec<_>>();
    let si_ref = (0..shared_count as u32)
        .map(|shared| {
            interner
                .token(info_of(shared).attrs.ref_name.as_deref())
                .unwrap_or(NO_TOKEN)
        })
        .collect::<Vec<_>>();
    let mut si_route_ref_start = vec![0u32];
    let mut si_route_ref_pool = Vec::new();
    for shared in 0..shared_count as u32 {
        let mut tokens = info_of(shared)
            .attrs
            .route_refs
            .iter()
            .filter_map(|route_ref| interner.token(Some(route_ref)))
            .collect::<Vec<_>>();
        tokens.sort_unstable();
        tokens.dedup();
        si_route_ref_pool.extend(tokens);
        si_route_ref_start.push(si_route_ref_pool.len() as u32);
    }
    drop(interner);
    clock.lap("edge fields")?;

    // Costs and durations per column, from the road graph's records.
    let column_count = column_profiles.len();
    let costing = RouteCostingOptions::default();
    let mut edge_cost = vec![f32::INFINITY; edge_count * column_count];
    let mut edge_duration = vec![f32::INFINITY; edge_count * column_count];
    for (column, profile) in column_profiles.iter().enumerate() {
        let policy = CostModel::new(*profile, &costing);
        let range = column * edge_count..(column + 1) * edge_count;
        edge_cost[range.clone()]
            .par_iter_mut()
            .zip(&mut edge_duration[range])
            .enumerate()
            .for_each(|(edge, (cost, duration))| {
                let shared = edge_shared_info[edge];
                // The fields costing reads.
                let road_edge = RoadEdge {
                    edge_id: network.edge_id(edge),
                    distance_m: edge_length[edge],
                    mobility_mode_mask: network.edge_mobility_mode_mask(edge),
                    inferred_speed_kph: network.edge_inferred_speed_kph(edge),
                    traversal_direction: network.edge_traversal_direction(edge),
                    ..RoadEdge::default()
                };
                let inputs = EdgeCostInputs::from_parts(
                    &road_edge,
                    info_of(shared).attrs,
                    classes[shared as usize],
                );
                if let EdgeCostDecision::Allowed(costed) =
                    policy.edge_cost_inputs(&inputs, &NoOverrides)
                {
                    *cost = costed.cost as f32;
                    *duration = costed.duration_s as f32;
                }
            });
    }
    let wheeled = 1u8 << profile_slot(MobilityProfile::Wheeled);
    for edge in 0..edge_count {
        let access = edge_profile_access[edge];
        let class = classes[edge_shared_info[edge] as usize];
        if access & wheeled == 0
            && access & wayfarer_package::sar::ACCESS_VEHICLE_PHYSICAL != 0
            && class.path_class != wayfarer_package::sar::SarPathClass::None
        {
            report.sar_path_edge_count_by_width[class.width as usize] += 1;
        }
    }
    drop(classes);
    network.release_edge_columns(&[EdgeColumn::InferredSpeed]);
    writer.put(Section::EdgeDuration, edge_duration);
    clock.lap("costs")?;

    // Restrictions, from the road graph's resolved restrictions.
    let edge_for_id = |edge_id: u64| -> Option<u32> {
        edge_id
            .checked_sub(edge_id_base)
            .and_then(|slot| edge_id_lookup.get(slot as usize))
            .copied()
            .filter(|edge| *edge != u32::MAX)
    };
    let mut simple_by_pair = BTreeMap::<(u64, u64), Vec<(i64, u32)>>::new();
    let mut complex_by_to = BTreeMap::<u64, Vec<(Vec<u64>, u32)>>::new();
    // A row that binds no profile (a road graph resolved before such
    // restrictions were left out) would only flag its edges.
    for restriction in resolved_restrictions
        .iter()
        .filter(|restriction| restriction_mobility_mode_mask(restriction) != 0)
    {
        match restriction {
            ResolvedEdgeRestriction::Simple {
                from_edge_id,
                to_edge_id,
                via_node_id,
                mobility_mode_mask,
                ..
            } => simple_by_pair
                .entry((*from_edge_id, *to_edge_id))
                .or_default()
                .push((*via_node_id, *mobility_mode_mask)),
            ResolvedEdgeRestriction::Complex {
                from_edge_id,
                to_edge_id,
                via_edge_ids,
                mobility_mode_mask,
                ..
            } => {
                let mut prefix = Vec::with_capacity(via_edge_ids.len() + 1);
                prefix.push(*from_edge_id);
                prefix.extend(via_edge_ids.iter().copied());
                complex_by_to
                    .entry(*to_edge_id)
                    .or_default()
                    .push((prefix, *mobility_mode_mask));
            }
        }
    }
    let mut simple_rows = Vec::<(u32, u32, u32, u8)>::new();
    for ((from_id, to_id), restrictions) in &simple_by_pair {
        let Some(from) = edge_for_id(*from_id) else {
            report.skipped_restriction_count += restrictions.len();
            continue;
        };
        // The via node is the end of the from edge; the stored id is an OSM id.
        match edge_for_id(*to_id) {
            Some(to) => {
                for (_via, mask) in restrictions {
                    simple_rows.push((from, to, edge_target[from as usize], profile_bits(*mask)));
                }
                edge_flags[from as usize] |= EDGE_FLAG_SIMPLE_RESTRICTION_FROM;
            }
            None => {
                report.skipped_restriction_count += restrictions.len();
                report.skipped_simple_missing_to_edge += restrictions.len();
            }
        }
    }
    drop(simple_by_pair);
    let mut complex_rows = Vec::<(u32, Vec<u32>, u8)>::new();
    for (to_id, restrictions) in &complex_by_to {
        let Some(to) = edge_for_id(*to_id) else {
            report.skipped_restriction_count += restrictions.len();
            continue;
        };
        for (prefix, mask) in restrictions {
            let mapped = prefix
                .iter()
                .map(|id| edge_for_id(*id))
                .collect::<Option<Vec<_>>>();
            match mapped {
                Some(prefix) if !prefix.is_empty() => {
                    complex_rows.push((to, prefix, profile_bits(*mask)));
                    edge_flags[to as usize] |= EDGE_FLAG_COMPLEX_RESTRICTION_TO;
                }
                _ => report.skipped_restriction_count += 1,
            }
        }
    }
    drop(complex_by_to);
    drop(resolved_restrictions);
    simple_rows.sort_unstable();
    complex_rows.sort();
    let mut complex_by_first = (0..complex_rows.len() as u32).collect::<Vec<_>>();
    complex_by_first.sort_by_key(|row| (complex_rows[*row as usize].1[0], *row));
    let complex_first = complex_by_first
        .iter()
        .map(|row| complex_rows[*row as usize].1[0])
        .collect::<Vec<_>>();
    for first in &complex_first {
        edge_flags[*first as usize] |= EDGE_FLAG_COMPLEX_RESTRICTION_FIRST;
    }
    let mut complex_prefix_start = vec![0u32];
    let mut complex_prefix_pool = Vec::new();
    for (_, prefix, _) in &complex_rows {
        complex_prefix_pool.extend_from_slice(prefix);
        complex_prefix_start.push(complex_prefix_pool.len() as u32);
    }
    // Via nodes: where a restricted transition happens (for the pieces).
    let mut restricted = vec![false; node_count];
    for (from, _, _, _) in &simple_rows {
        restricted[edge_target[*from as usize] as usize] = true;
    }
    for edge in &complex_prefix_pool {
        restricted[edge_target[*edge as usize] as usize] = true;
    }
    let (simple_restriction_count, complex_restriction_count) =
        (simple_rows.len(), complex_rows.len());
    writer.put(
        Section::SimpleFrom,
        simple_rows.iter().map(|row| row.0).collect::<Vec<_>>(),
    );
    writer.put(
        Section::SimpleTo,
        simple_rows.iter().map(|row| row.1).collect::<Vec<_>>(),
    );
    writer.put(
        Section::SimpleVia,
        simple_rows.iter().map(|row| row.2).collect::<Vec<_>>(),
    );
    writer.put(
        Section::SimpleMask,
        simple_rows.iter().map(|row| row.3).collect::<Vec<_>>(),
    );
    drop(simple_rows);
    writer.put(
        Section::ComplexTo,
        complex_rows.iter().map(|row| row.0).collect::<Vec<_>>(),
    );
    writer.put(
        Section::ComplexMask,
        complex_rows.iter().map(|row| row.2).collect::<Vec<_>>(),
    );
    drop(complex_rows);
    writer.put(Section::ComplexPrefixStart, complex_prefix_start);
    writer.put(Section::ComplexPrefixPool, complex_prefix_pool);
    writer.put(Section::ComplexFirst, complex_first);
    writer.put(Section::ComplexByFirst, complex_by_first);
    clock.lap("restrictions")?;

    // Shapes: the road graph geometry of each shared info, as E7 deltas with
    // implicit endpoints where every edge agrees on the nodes; shapes with a
    // point that is not an exact E7 value are also stored exactly.
    let mut shape_start = Vec::with_capacity(shared_count.div_ceil(SHAPE_START_STRIDE) + 1);
    let mut shape_bytes = Vec::new();
    let mut exact_shape_shared = Vec::new();
    let mut exact_shape_start = vec![0u32];
    let mut exact_shape_points = Vec::<f64>::new();
    shape_start.push(0u32);
    for shared in 0..shared_count {
        let info = info_of(shared as u32);
        let (from, to) = si_nodes[shared];
        let endpoints =
            si_consistent[shared].then(|| (node_e7_of(&network, from), node_e7_of(&network, to)));
        let exact = info
            .geometry
            .iter()
            .all(|(lat, lng)| is_exact_e7(lat) && is_exact_e7(lng));
        let points = info
            .geometry
            .iter()
            .map(|(lat, lng)| (to_e7(lat), to_e7(lng)))
            .collect::<Vec<_>>();
        report.shape_point_count += points.len();
        if !exact {
            report.shape_inexact_count += 1;
            exact_shape_shared.push(shared as u32);
            for (lat, lng) in info.geometry.iter() {
                exact_shape_points.extend([lat, lng]);
            }
            exact_shape_start.push((exact_shape_points.len() / 2) as u32);
        }
        let before = shape_bytes.len();
        encode_shape(&mut shape_bytes, &points, endpoints);
        if shape_bytes
            .get(before)
            .is_some_and(|header| header & 1 != 0)
        {
            report.shape_explicit_count += 1;
        }
        let end = u32::try_from(shape_bytes.len())
            .map_err(|_| RoutingPackageError::Unsupported("shape section exceeds 4 GiB".into()))?;
        // The offset of every SHAPE_START_STRIDEth shape, then the end.
        if (shared + 1).is_multiple_of(SHAPE_START_STRIDE) || shared + 1 == shared_count {
            shape_start.push(end);
        }
    }
    report.shape_byte_len = shape_bytes.len() + shape_start.len() * 4;
    drop(si_consistent);
    writer.put(Section::ShapeStart, shape_start);
    writer.put(Section::ShapeBytes, shape_bytes);
    writer.put(Section::ExactShapeShared, exact_shape_shared);
    writer.put(Section::ExactShapeStart, exact_shape_start);
    writer.put(Section::ExactShapePoints, exact_shape_points);
    clock.lap("shapes")?;

    // Node positions: unit vectors in fixed point, and E7 / exact positions
    // only when they cannot be recomputed from them. A node whose position
    // is not an E7 value reads its E7 position from its exact one, so only
    // the others need to round-trip.
    let node_lat = |node: usize| network.node_lat_lng(node).0;
    let node_lng = |node: usize| network.node_lat_lng(node).1;
    let mut node_xyz = Vec::with_capacity(node_count * 3);
    for node in 0..node_count {
        node_xyz.extend_from_slice(&fixed_xyz(unit_xyz(node_lat(node), node_lng(node))));
    }
    let node_lat_e7 = (0..node_count)
        .map(|node| to_e7(node_lat(node)))
        .collect::<Vec<_>>();
    let node_lng_e7 = (0..node_count)
        .map(|node| to_e7(node_lng(node)))
        .collect::<Vec<_>>();
    let node_e7_derived = !store_derivable()
        && (0..node_count).all(|node| {
            let at = node * 3;
            !is_exact_e7(node_lat(node))
                || !is_exact_e7(node_lng(node))
                || e7_from_fixed_xyz([node_xyz[at], node_xyz[at + 1], node_xyz[at + 2]])
                    == (node_lat_e7[node], node_lng_e7[node])
        });
    let (node_lat_e7, node_lng_e7) = if node_e7_derived {
        (Vec::new(), Vec::new())
    } else {
        (node_lat_e7, node_lng_e7)
    };
    writer.put(Section::NodeLatE7, node_lat_e7);
    writer.put(Section::NodeLngE7, node_lng_e7);
    report.node_inexact_count = (0..node_count)
        .filter(|node| !is_exact_e7(node_lat(*node)) || !is_exact_e7(node_lng(*node)))
        .count();
    // Exact positions of the nodes that are not their E7 value; the others
    // read their E7 position.
    let node_lat_lng_nodes = (0..node_count as u32)
        .filter(|node| {
            let node = *node as usize;
            !is_exact_e7(node_lat(node)) || !is_exact_e7(node_lng(node))
        })
        .collect::<Vec<_>>();
    let node_lat_lng = node_lat_lng_nodes
        .iter()
        .flat_map(|node| [node_lat(*node as usize), node_lng(*node as usize)])
        .collect::<Vec<_>>();
    network.release_node_columns(&[NodeColumn::LatLng]);
    writer.put(Section::NodeLatLng, node_lat_lng);
    writer.put(Section::NodeLatLngNodes, node_lat_lng_nodes);
    let layout = PackageLayout {
        node_e7_derived,
        edge_id_base,
    };

    // Out-adjacency.
    let mut first_out = vec![0u32; node_count + 1];
    for source in &edge_source {
        first_out[*source as usize + 1] += 1;
    }
    for node in 0..node_count {
        first_out[node + 1] += first_out[node];
    }

    // Admissible, consistent A* factor: the smallest cost per metre any edge
    // realises, measured against max(polyline length, endpoint chord) so it
    // also bounds partially traversed destination edges.
    let mut heuristic_cost_per_m = [0.0f64; PROFILE_SLOTS];
    for (column, profile) in column_profiles.iter().enumerate() {
        let mut factor = f64::INFINITY;
        for edge in 0..edge_count {
            let cost = f64::from(edge_cost[column * edge_count + edge]);
            if !cost.is_finite() {
                continue;
            }
            let (s, t) = (edge_source[edge] as usize, edge_target[edge] as usize);
            // Against the stored (fixed-point) vectors, as the query measures
            // chords: the bound stays consistent.
            let xyz = |node: usize| {
                xyz_from_fixed([
                    node_xyz[node * 3],
                    node_xyz[node * 3 + 1],
                    node_xyz[node * 3 + 2],
                ])
            };
            let chord = wayfarer_package::format::chord_m(xyz(s), xyz(t));
            let span = f64::from(edge_length[edge]).max(chord);
            if span > 0.0 {
                factor = factor.min(cost / span);
            }
        }
        let factor = if factor.is_finite() {
            factor * 0.999
        } else {
            0.0
        };
        heuristic_cost_per_m[profile_slot(*profile)] = factor;
        if *profile == MobilityProfile::Wheeled {
            heuristic_cost_per_m[profile_slot(MobilityProfile::Default)] = factor;
        }
        report.heuristic_cost_per_m.push((*profile, factor));
    }
    writer.put(Section::EdgeLength, edge_length);
    clock.lap("adjacency, positions, heuristic")?;

    // Clean pieces per column, for the route search's walks, from the
    // incoming adjacency: compact edges grouped by target, ascending edge
    // index.
    let mut first_in = vec![0u32; node_count + 1];
    for target in &edge_target {
        first_in[*target as usize + 1] += 1;
    }
    for node in 0..node_count {
        first_in[node + 1] += first_in[node];
    }
    let mut in_cursor = first_in.clone();
    let mut in_edges = vec![0u32; edge_count];
    for (edge, target) in edge_target.iter().enumerate() {
        let slot = &mut in_cursor[*target as usize];
        in_edges[*slot as usize] = edge as u32;
        *slot += 1;
    }
    drop(in_cursor);
    let piece_bits = pieces::piece_bits(
        &pieces::PieceInput {
            node_count,
            edge_count,
            first_out: &first_out,
            first_in: &first_in,
            in_edges: &in_edges,
            edge_source: &edge_source,
            edge_target: &edge_target,
            edge_cost: &edge_cost,
            edge_shared_info: &edge_shared_info,
            restricted: &restricted,
        },
        column_count,
    );
    drop(restricted);
    drop(first_in);
    drop(in_edges);
    if trace() {
        eprintln!(
            "  piece (chain, tree) nodes per column: {:?}",
            piece_bits.counts
        );
    }
    writer.put(Section::PieceChain, piece_bits.chain);
    writer.put(Section::PieceTree, piece_bits.tree);
    writer.put(Section::EdgeCost, edge_cost);
    clock.lap("pieces")?;

    // Twins: stored as each edge's reverse edge (`Section::EdgeTwin`) and the
    // few edges without one.
    let edge_twin = edge_twins(
        &first_out,
        &edge_source,
        &edge_target,
        &edge_shared_info,
        &edge_flags,
        &heading_start,
        &heading_end,
    );
    let mut lone_in_edge = (0..edge_count as u32)
        .filter(|edge| edge_twin[*edge as usize] == NO_TWIN)
        .collect::<Vec<_>>();
    lone_in_edge.sort_unstable_by_key(|edge| (edge_target[*edge as usize], *edge));
    let mut lone_in_present = vec![0u64; node_count.div_ceil(64)];
    for edge in &lone_in_edge {
        let target = edge_target[*edge as usize] as usize;
        lone_in_present[target / 64] |= 1 << (target % 64);
    }
    report.lone_edge_count = lone_in_edge.len();
    writer.put(Section::EdgeHeadingStart, heading_start);
    // Derived from the twins' start headings, but for the edges without a
    // twin.
    let lone_heading_end = lone_in_edge
        .iter()
        .map(|edge| heading_end[*edge as usize])
        .collect::<Vec<_>>();
    drop(heading_end);
    writer.put(Section::LoneHeadingEnd, lone_heading_end);
    writer.put(Section::EdgeTwin, edge_twin);
    writer.put(Section::LoneInPresent, lone_in_present);
    writer.put(Section::LoneInEdge, lone_in_edge);
    clock.lap("twins")?;

    // Request sections (correlation, legs).
    let request_started = Instant::now();
    let request_sections = request::build_request_sections(&request::RequestBuildInput {
        network: &network,
        shared_infos: &shared_infos,
        component_sizes: &component_sizes,
        first_out: &first_out,
        edge_source: &edge_source,
        edge_target: &edge_target,
        edge_id_base,
        edge_id_lookup: &edge_id_lookup,
        correlation_opposing: &correlation_opposing,
        edge_shared_info: &edge_shared_info,
        edge_flags: &edge_flags,
        edge_profile_access: &edge_profile_access,
        shared_raw: &shared_raw,
        implied_ids,
    })?;
    report.request_byte_len = request_sections.byte_len();
    report.request_opposing_exception_count = request_sections.opposing_exception_count();
    report.mode_mask_exception_count = request_sections.mode_mask_exception_count();
    report.tier_exception_count = request_sections.tier_exception_count();
    report.request_ms = request_started.elapsed().as_millis();
    drop(shared_raw);
    drop(correlation_opposing);
    drop(component_sizes);
    // Not stored: engines build the lookup from the ids on first use.
    drop(edge_id_lookup);
    // Derived from the twins and the out-adjacency.
    drop(edge_source);
    writer.put(Section::EdgeProfileAccess, edge_profile_access);
    writer.put(Section::EdgeFlags, edge_flags);
    if implied_ids {
        // Format 3 stores no edge ids: edge `i` has id `edge_id_base + i`.
        if (0..edge_count)
            .into_par_iter()
            .any(|edge| network.edge_id(edge) - edge_id_base != edge as u64)
        {
            return Err(RoutingPackageError::Unsupported(
                "format 3 needs edge ids in package order (edge i has id base + i); renumber the graph first (`renumber_in_package_order`), as a compile does".into(),
            ));
        }
    } else {
        writer.put(
            Section::EdgeId,
            (0..edge_count)
                .map(|edge| (network.edge_id(edge) - edge_id_base) as u32)
                .collect::<Vec<_>>(),
        );
    }
    drop(network);
    // Name and ref tokens per attribute class: the class key holds the name
    // and ref, so every shared info of a class has the same tokens.
    let shared_class = request_sections.shared_attr_class();
    let class_count = request_sections.class_count();
    let si_name = per_class(&si_name, shared_class, class_count, "name")?;
    let si_ref = per_class(&si_ref, shared_class, class_count, "ref")?;
    let (si_route_ref_start, si_route_ref_pool) = per_class_lists(
        &si_route_ref_start,
        &si_route_ref_pool,
        shared_class,
        class_count,
    )?;
    writer.put(Section::SharedInfoName, si_name);
    writer.put(Section::SharedInfoRef, si_ref);
    writer.put(Section::SharedInfoRouteRefStart, si_route_ref_start);
    writer.put(Section::SharedInfoRouteRefPool, si_route_ref_pool);
    writer.put(Section::EdgeSharedInfo, edge_shared_info);
    writer.put(
        Section::PackageParams,
        PackageParams {
            costing_model_version: metadata.costing_model_version,
            sar: metadata.sar,
            mode_mask_union: metadata.mode_mask_union,
            default_component: request_sections.default_component(),
            reach_cap: wayfarer_package::request::SATURATION_CAP as u64,
            draft_revision: Some(FORMAT_3_DRAFT_REVISION),
        }
        .to_section(),
    );
    request_sections.put(&mut writer);
    clock.lap("request sections")?;

    drop(shared_infos);
    // The CCH order, from the node graph and the fixed-point positions
    // (integer arithmetic: the same on every platform).
    job.report(
        CompileStage::CchOrder,
        Some(0.0),
        0,
        "Ordering nodes for CCH potentials",
    );
    let order_started = Instant::now();
    let cch_rank = if options.cch_order {
        let graph = wayfarer_package::cch_order::NodeGraph::new(&first_out, &edge_target);
        let (order, _) = wayfarer_package::cch_order::elimination_order(&graph, &node_xyz);
        wayfarer_package::cch_order::ranks(&order)
    } else {
        Vec::new()
    };
    report.cch_order_ms = order_started.elapsed().as_millis();
    job.check().map_err(|_| cancelled_error())?;
    job.report(
        CompileStage::CchOrder,
        Some(1.0),
        0,
        "Ordered nodes for CCH potentials",
    );
    clock.lap("cch order")?;

    let counts = PackageCounts {
        node_count,
        edge_count,
        shared_info_count: shared_count,
        simple_restriction_count,
        complex_restriction_count,
        column_count,
    };
    writer.set_header(counts, layout, profile_columns, heuristic_cost_per_m);
    writer.put(Section::NodeFirstOut, first_out);
    writer.put(Section::NodeXyzFixed, node_xyz);
    writer.put(Section::CchRank, cch_rank);
    writer.put(Section::EdgeTarget, edge_target);

    report.node_count = node_count;
    report.edge_count = edge_count;
    report.shared_info_count = counts.shared_info_count;
    report.simple_restriction_count = counts.simple_restriction_count;
    report.complex_restriction_count = counts.complex_restriction_count;
    report.byte_len = writer.byte_len();
    report.assemble_ms = started
        .elapsed()
        .as_millis()
        .saturating_sub(report.cch_order_ms);
    report.total_ms = started.elapsed().as_millis();
    Ok((writer, report))
}

/// Each edge's twin ([`Section::EdgeTwin`]): the edge the adjacency rule
/// for opposing edges finds (`request::derived_opposing`: out of the target,
/// back to the source over the same shared info, against its shape), when
/// the rule pairs the two both ways and each one's end heading is the
/// other's start heading reversed ([`reverse_heading`]); [`NO_TWIN`]
/// otherwise. A twin then determines the edge's source, its end heading and
/// its place among its target's in-edges.
fn edge_twins(
    first_out: &[u32],
    edge_source: &[u32],
    edge_target: &[u32],
    edge_shared_info: &[u32],
    edge_flags: &[u8],
    heading_start: &[u16],
    heading_end: &[u16],
) -> Vec<u32> {
    let rule = |edge: u32| {
        derived_opposing(
            first_out,
            edge_source,
            edge_target,
            edge_shared_info,
            edge_flags,
            edge,
        )
    };
    (0..edge_source.len() as u32)
        .into_par_iter()
        .map(|edge| {
            let Some(twin) = rule(edge) else {
                return NO_TWIN;
            };
            let (e, t) = (edge as usize, twin as usize);
            let paired = rule(twin) == Some(edge)
                && heading_end[e] == reverse_heading(heading_start[t])
                && heading_end[t] == reverse_heading(heading_start[e]);
            if paired { twin } else { NO_TWIN }
        })
        .collect()
}

/// Builds the package of `graph` in memory (in-memory engines and tests).
/// `values` (one per shared info) as one value per attribute class, failing
/// if two shared infos of a class disagree: the reader takes the class's
/// value for every shared info of it.
fn per_class(
    values: &[u32],
    shared_class: &[u32],
    class_count: usize,
    what: &str,
) -> Result<Vec<u32>, RoutingPackageError> {
    const UNSET: u32 = u32::MAX - 1;
    debug_assert!(values.iter().all(|value| *value != UNSET));
    let mut by_class = vec![UNSET; class_count];
    for (value, class) in values.iter().zip(shared_class) {
        let slot = &mut by_class[*class as usize];
        if *slot == UNSET {
            *slot = *value;
        } else if *slot != *value {
            return Err(RoutingPackageError::Invalid(format!(
                "attribute class {class} has shared infos with different {what} tokens"
            )));
        }
    }
    // A class no shared info uses (none today) reads as no token.
    for slot in &mut by_class {
        if *slot == UNSET {
            *slot = NO_TOKEN;
        }
    }
    Ok(by_class)
}

/// Lists (`start` offsets into `pool`, one per shared info + 1) as one list
/// per attribute class, failing if two shared infos of a class disagree.
fn per_class_lists(
    start: &[u32],
    pool: &[u32],
    shared_class: &[u32],
    class_count: usize,
) -> Result<(Vec<u32>, Vec<u32>), RoutingPackageError> {
    let list = |shared: usize| &pool[start[shared] as usize..start[shared + 1] as usize];
    let mut first = vec![u32::MAX; class_count];
    for (shared, class) in shared_class.iter().enumerate() {
        let slot = &mut first[*class as usize];
        if *slot == u32::MAX {
            *slot = shared as u32;
        } else if list(*slot as usize) != list(shared) {
            return Err(RoutingPackageError::Invalid(format!(
                "attribute class {class} has shared infos with different route refs"
            )));
        }
    }
    let mut class_start = Vec::with_capacity(class_count + 1);
    let mut class_pool = Vec::new();
    class_start.push(0u32);
    for shared in first {
        // A class no shared info uses (none today) has no route refs.
        if shared != u32::MAX {
            class_pool.extend_from_slice(list(shared as usize));
        }
        class_start.push(class_pool.len() as u32);
    }
    Ok((class_start, class_pool))
}

pub fn package_bytes(
    graph: &RoadGraph,
    options: &PackageBuildOptions,
) -> Result<(Vec<u8>, PackageBuildReport), RoutingPackageError> {
    let metadata = package_metadata(graph, None);
    let (writer, report) = build_package(graph, &metadata, options)?;
    Ok((writer.finish(), report))
}

/// Writes the routing package of `graph` to `path` atomically. The manifest
/// supplies the road graph's schema, hash, counts and quality.
pub fn write_package(
    graph: &RoadGraph,
    manifest: Option<&RoadGraphPackageManifest>,
    path: &std::path::Path,
    options: &PackageBuildOptions,
) -> Result<PackageBuildReport, RoutingPackageError> {
    let metadata = package_metadata(graph, manifest);
    let (writer, report) = build_package(graph, &metadata, options)?;
    without_job(|job| {
        let mut clock = PhaseClock::new(job);
        PackageFile::write_encoded(path, &writer)?;
        clock.time("write");
        Ok::<_, RoutingPackageError>(())
    })?;
    Ok(report)
}

/// [`write_package`], consuming `graph` and freeing it before the CCH
/// order.
pub fn write_package_consuming(
    graph: RoadGraph,
    manifest: Option<&RoadGraphPackageManifest>,
    path: &std::path::Path,
    options: &PackageBuildOptions,
) -> Result<PackageBuildReport, RoutingPackageError> {
    without_job(|job| write_package_for_job(graph, manifest, path, options, None, job))
}

/// [`write_package_consuming`] for a compile job: reports the build, CCH
/// order and write stages, and stops within about a second of a cancel, leaving no
/// file at `path` (nor its `.tmp`). After a cancel the error says only that;
/// the job tells the caller it was a cancel. With a `spill_dir` the sections
/// go to a file there as they are built (removed after the write), instead
/// of staying in memory until the write.
pub(crate) fn write_package_for_job(
    graph: RoadGraph,
    manifest: Option<&RoadGraphPackageManifest>,
    path: &std::path::Path,
    options: &PackageBuildOptions,
    spill_dir: Option<&std::path::Path>,
    job: &Job<'_>,
) -> Result<PackageBuildReport, RoutingPackageError> {
    let metadata = package_metadata(&graph, manifest);
    let spill_path = spill_dir.map(|dir| dir.join("package-sections.spill"));
    let spill = match &spill_path {
        Some(spill_path) => {
            if let Some(dir) = spill_path.parent() {
                std::fs::create_dir_all(dir)?;
            }
            Some(
                std::fs::OpenOptions::new()
                    .read(true)
                    .write(true)
                    .create(true)
                    .truncate(true)
                    .open(spill_path)?,
            )
        }
        None => None,
    };
    let result = (|| {
        let (writer, report) = build_package_from(
            GraphSource::Owned(Box::new(graph)),
            &metadata,
            options,
            job,
            spill,
        )?;
        let mut clock = PhaseClock::new(job);
        write_file(path, &writer, job)?;
        clock.time("write");
        Ok(report)
    })();
    if let Some(spill_path) = &spill_path {
        let _ = std::fs::remove_file(spill_path);
    }
    result
}

/// Bytes handed to the file per write while writing a package, so a cancel
/// or a progress report never waits on more than this.
const WRITE_CHUNK: usize = 4 << 20;

/// Writes `writer` to `path` as [`PackageFile::write_encoded`] does (the same
/// bytes, through `path.tmp`, synced, then moved into place with
/// [`PackageFile::move_into_place`], which replaces an open package), reporting
/// [`CompileStage::WritePackage`] and checking the job between chunks.
/// On any error or cancel the `.tmp` file is removed and `path` is untouched.
fn write_file(
    path: &std::path::Path,
    writer: &PackageWriter,
    job: &Job<'_>,
) -> Result<(), RoutingPackageError> {
    let total = writer.byte_len();
    job.report(
        CompileStage::WritePackage,
        Some(0.0),
        0,
        "Checksumming the package sections",
    );
    // Hashes every section; the write below reuses the result.
    let _ = writer.seal();
    if job.cancelled() {
        return Err(cancelled_error());
    }
    // The directories this write creates, deepest first, so that a failed
    // or cancelled write takes them away again.
    let mut created = Vec::new();
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        let mut missing = Some(parent);
        while let Some(dir) = missing.filter(|dir| !dir.as_os_str().is_empty() && !dir.exists()) {
            created.push(dir.to_path_buf());
            missing = dir.parent();
        }
        std::fs::create_dir_all(parent)?;
    }
    let mut temp = path.as_os_str().to_owned();
    temp.push(".tmp");
    let temp = std::path::PathBuf::from(temp);
    let result = (|| -> Result<(), RoutingPackageError> {
        let mut file = std::fs::File::create(&temp)?;
        let mut out = ProgressFile {
            file: &mut file,
            written: 0,
            reported: 0,
            total,
            job,
        };
        let mut buffered = std::io::BufWriter::with_capacity(1 << 20, &mut out);
        writer.write_to(&mut buffered)?;
        buffered.flush()?;
        drop(buffered);
        file.sync_all()?;
        if job.cancelled() {
            return Err(cancelled_error());
        }
        // Replaces a package that is open, on Windows too.
        PackageFile::move_into_place(&temp, path)?;
        Ok(())
    })();
    if result.is_err() {
        let _ = std::fs::remove_file(&temp);
        for dir in &created {
            // Only if empty: something else may have been put there meanwhile.
            let _ = std::fs::remove_dir(dir);
        }
        if job.cancelled() {
            return Err(cancelled_error());
        }
    }
    result?;
    job.report(
        CompileStage::WritePackage,
        Some(1.0),
        total,
        "Routing package written",
    );
    Ok(())
}

/// The package file being written: hands the file at most [`WRITE_CHUNK`] per
/// write, reports progress every 1% and fails the write once the job is
/// cancelled.
struct ProgressFile<'f, 'j, 'a> {
    file: &'f mut std::fs::File,
    written: usize,
    reported: usize,
    total: usize,
    job: &'j Job<'a>,
}

impl Write for ProgressFile<'_, '_, '_> {
    fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
        if self.job.cancelled() {
            return Err(std::io::Error::other("the write was cancelled"));
        }
        let count = self.file.write(&bytes[..bytes.len().min(WRITE_CHUNK)])?;
        self.written += count;
        if self.written - self.reported >= (self.total / 100).max(1) {
            self.reported = self.written;
            self.job.report(
                CompileStage::WritePackage,
                Some(self.written as f64 / self.total.max(1) as f64),
                self.written,
                "Writing the routing package",
            );
        }
        Ok(count)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.file.flush()
    }
}

#[cfg(test)]
mod tests {
    use wayfarer_package::{DirectionalAccess, RoadType};
    use wayfarer_road_graph::{RoadGraph, RoadGraphBuilder, RoutableWay};

    use super::*;
    use wayfarer::internals::GraphAccess;

    /// The package of `graph`, opened by the engine.
    fn in_memory(graph: &RoadGraph) -> (wayfarer::RoutingPackage, PackageBuildReport) {
        let (bytes, report) = package_bytes(graph, &PackageBuildOptions::default()).unwrap();
        (
            wayfarer::RoutingPackage::from_bytes(&bytes).unwrap(),
            report,
        )
    }
    use wayfarer_package::{GeoPoint, directed_geometry};

    /// Curved two-way and one-way streets sharing nodes, so edges run both
    /// with and against their shared-info shapes.
    fn curved_graph() -> RoadGraph {
        let mut builder = RoadGraphBuilder::new("compact-shapes");
        let ways = [
            (
                1,
                vec![
                    (50.0, 19.9),
                    (50.0004, 19.9003),
                    (50.0011, 19.9004),
                    (50.0015, 19.9011),
                ],
                vec![Some(1), None, None, Some(2)],
                DirectionalAccess::Both,
            ),
            (
                2,
                vec![(50.0015, 19.9011), (50.0019, 19.90155), (50.0021, 19.9023)],
                vec![Some(2), None, Some(3)],
                DirectionalAccess::Forward,
            ),
            (
                3,
                vec![(50.0021, 19.9023), (50.00135, 19.90251), (50.0, 19.9)],
                vec![Some(3), None, Some(1)],
                DirectionalAccess::Both,
            ),
        ];
        for (osm_way_id, points, source_node_ids, directional_access) in ways {
            builder.push_way(RoutableWay {
                osm_way_id,
                points,
                source_node_ids,
                road_type: RoadType::Residential,
                directional_access,
                highway: Some("residential".into()),
                ..Default::default()
            });
        }
        builder.finish()
    }

    #[test]
    fn derived_and_stored_layouts_reproduce_road_graph_ids_and_geometry() {
        let road_graph = curved_graph();
        let access = GraphAccess::new(&road_graph);
        for stored in [false, true] {
            STORE_DERIVABLE.with(|flag| flag.set(stored));
            let (graph, report) = in_memory(&road_graph);
            STORE_DERIVABLE.with(|flag| flag.set(false));
            assert_eq!(!graph.file().layout().node_e7_derived, stored);
            assert!(report.shape_point_count > 2 * report.shared_info_count);
            assert_eq!(report.shape_explicit_count, 0);
            let view = graph.file().view();
            let mut shape = Vec::new();
            let mut reversed = 0;
            for edge in 0..view.edge_count as u32 {
                let edge_id = view.edge_id(edge as usize);
                assert_eq!(view.edge_for_id(edge_id), Some(edge));
                let record = access.edge_by_id(edge_id).unwrap();
                view.edge_shape(edge, &mut shape);
                let expected: Vec<GeoPoint> =
                    directed_geometry(record.shared_info, record.edge.traversal_direction);
                assert!(expected.len() >= 3);
                assert_eq!(shape, expected, "edge {edge} stored={stored}");
                reversed += usize::from(
                    view.edge_flags[edge as usize]
                        & wayfarer_package::format::EDGE_FLAG_SHAPE_REVERSED
                        != 0,
                );
            }
            assert!(reversed > 0, "some edge runs against its shape");
        }
    }

    #[test]
    fn inexact_coordinates_are_kept_exactly() {
        let mut builder = RoadGraphBuilder::new("inexact");
        builder.push_way(RoutableWay {
            osm_way_id: 1,
            points: vec![(50.0, 19.9), (50.0 + 0.1 + 0.2, 19.9 + 1.0 / 3.0)],
            road_type: RoadType::Residential,
            directional_access: DirectionalAccess::Both,
            highway: Some("residential".into()),
            ..Default::default()
        });
        let road_graph = builder.finish();
        let (graph, report) = in_memory(&road_graph);
        assert!(report.shape_inexact_count > 0);
        assert!(report.node_inexact_count > 0);
        let access = GraphAccess::new(&road_graph);
        let view = graph.file().view();
        let request = graph.file().request_view().unwrap();
        let mut shape = Vec::new();
        for edge in 0..view.edge_count as u32 {
            let record = access.edge_by_id(view.edge_id(edge as usize)).unwrap();
            view.edge_shape(edge, &mut shape);
            assert_eq!(
                shape,
                directed_geometry(record.shared_info, record.edge.traversal_direction)
            );
            let leg = request.leg_record(edge).unwrap();
            assert_eq!(leg.source.lat, record.source.lat);
            assert_eq!(leg.target.lng, record.target.lng);
        }
    }

    /// Streets with posted limits (some shared, so costing tuples repeat), a
    /// synthetic and an over-`u32` way id, steps past the end of the streets
    /// (outside the driving column's network: unlike a footway, R5
    /// never opens them) and a node that is not an E7 position.
    fn compact_layout_graph() -> RoadGraph {
        let mut builder = RoadGraphBuilder::new("compact-layout");
        let street =
            |osm_way_id: i64, points: Vec<(f64, f64)>, maxspeed: Option<&str>| RoutableWay {
                osm_way_id,
                points,
                road_type: RoadType::Residential,
                directional_access: DirectionalAccess::Both,
                highway: Some("residential".into()),
                maxspeed: maxspeed.map(Into::into),
                ..Default::default()
            };
        builder.push_way(street(1, vec![(50.0, 19.9), (50.001, 19.9)], Some("30")));
        builder.push_way(street(
            -9_000_000_000_001,
            vec![(50.001, 19.9), (50.001, 19.901)],
            Some("30"),
        ));
        builder.push_way(street(
            5_000_000_000,
            vec![(50.001, 19.901), (50.0, 19.901)],
            Some("50"),
        ));
        builder.push_way(street(
            7,
            vec![(50.0, 19.901), (50.0 + 0.0001 / 3.0, 19.901 + 0.0001 / 7.0)],
            None,
        ));
        builder.push_way(RoutableWay {
            osm_way_id: 9,
            points: vec![(50.001, 19.901), (50.002, 19.902)],
            road_type: RoadType::Path,
            directional_access: DirectionalAccess::Both,
            highway: Some("steps".into()),
            ..Default::default()
        });
        builder.finish()
    }

    #[test]
    fn compact_layout_reproduces_the_road_graph() {
        let road_graph = compact_layout_graph();
        let access = GraphAccess::new(&road_graph);
        let (graph, report) = in_memory(&road_graph);
        let view = graph.file().view();
        let request = graph.file().request_view().unwrap();
        assert!(!graph.file().edge_id_lookup_built());
        assert_eq!(report.way_id_wide_count, 2);
        assert!(report.cost_tuple_count < view.si_cost_tuple.len());
        assert_eq!(report.node_inexact_count, view.node_lat_lng_nodes.len());
        assert!(report.node_inexact_count > 0);
        for edge in 0..view.edge_count as u32 {
            let edge_id = view.edge_id(edge as usize);
            // The lookup built on first use.
            assert_eq!(view.edge_for_id(edge_id), Some(edge));
            let record = access.edge_by_id(edge_id).unwrap();
            let shared = view.edge_shared_info[edge as usize] as usize;
            let info = record.shared_info;
            assert_eq!(view.si_way.get(shared), info.osm_way_id);
            let tuple = view.cost_tuple(shared);
            assert_eq!(
                view.si_cost_flags[tuple],
                EdgeCostFlags::from_shared_info(info).0
            );
            assert_eq!(
                view.si_cost_classes[tuple],
                EdgeCostClasses::from_shared_info(info).pack()
            );
            let speed = |value: Option<f32>| value.unwrap_or(f32::NAN).to_bits();
            assert_eq!(
                view.si_speeds[tuple * 3..tuple * 3 + 3]
                    .iter()
                    .map(|value| value.to_bits())
                    .collect::<Vec<_>>(),
                [
                    speed(info.maxspeed_kph),
                    speed(info.free_flow_speed_kph),
                    speed(info.constrained_speed_kph)
                ]
            );
            let leg = request.leg_record(edge).unwrap();
            assert_eq!(leg.shared_info.osm_way_id, info.osm_way_id);
            for (node, expected) in [
                (view.edge_source(edge as usize), &record.source),
                (view.edge_target[edge as usize], &record.target),
            ] {
                let (lat, lng) = view.node_lat_lng(node);
                assert_eq!(
                    (lat.to_bits(), lng.to_bits()),
                    (expected.lat.to_bits(), expected.lng.to_bits())
                );
            }
        }
        // Ways compare by id, wide ones included.
        for left in 0..view.si_way.len() {
            for right in 0..view.si_way.len() {
                let (a, b) = (view.si_way.get(left), view.si_way.get(right));
                assert_eq!(view.si_way.same_way(left, right), a != 0 && a == b);
            }
        }
    }

    #[test]
    fn open_rejects_duplicate_edge_ids() {
        let (mut bytes, _) =
            package_bytes(&compact_layout_graph(), &PackageBuildOptions::default()).unwrap();
        let (offset, len) = PackageFile::from_bytes(&bytes)
            .unwrap()
            .section_range(Section::EdgeId);
        assert!(len >= 8);
        let first = bytes[offset..offset + 4].to_vec();
        bytes[offset + 4..offset + 8].copy_from_slice(&first);
        assert!(matches!(
            PackageFile::from_bytes(&bytes),
            Err(RoutingPackageError::Invalid(message)) if message.contains("edge ids")
        ));
    }
}

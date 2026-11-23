//! The compiler's public entry point: [`compile`] a [`CompileRequest`] into a
//! package, with [`CompileProgress`] reports and a [`CancelToken`], ending in a
//! [`CompileResult`] or a [`CompileError`].

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

use osm_pbf_ingest::IngestBuildProfile;
use serde::{Deserialize, Serialize};
use wayfarer_package::format::{PackageFormat, profile_slot};
use wayfarer_package::{CancelToken, MobilityProfile};
use wayfarer_road_graph::RoadGraphPackageManifest;

use crate::job::{self, Report};
use crate::{
    CompileStage, PackageBuildOptions, PackageBuildReport, RoadGraphCompileRequest,
    RouteArchiveCompileRequest, compile_route_archive,
};

/// What to compile, where to, and how. The JSON form (camelCase, unknown
/// fields rejected) is what the `wayfarer compile --request` command and the
/// C ABI take:
///
/// ```json
/// {"input": "monaco.osm.pbf", "output": "routing.mrt", "country": "MC",
///  "memory": "low", "threads": 2, "profiles": ["wheeled", "on_foot", "legged"]}
/// ```
///
/// Only `input` and `output` are required. In Rust, start from
/// [`CompileRequest::new`] and set the options with its builder methods (or
/// the public fields): the struct is `#[non_exhaustive]`, so options added
/// later don't break callers.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
#[non_exhaustive]
pub struct CompileRequest {
    /// The OpenStreetMap extract (`.osm.pbf`).
    pub input: PathBuf,
    /// Where the package (`routing.mrt`) is written. It is written to
    /// `<output>.tmp` and renamed into place at the very end, so a package
    /// already there is replaced only by a complete one, and kept if the
    /// compile fails or is cancelled. Missing parent directories are created
    /// (and removed again if the compile fails). On Windows a package that
    /// is open (memory-mapped by an engine in any process) can't be replaced:
    /// the compile fails with access denied and leaves it intact, so compile
    /// to a new path and switch the engine over.
    pub output: PathBuf,
    /// Where the compile keeps scratch files (the encoded way facts while the
    /// graph is built). The compile works in a fresh directory of its own
    /// inside it and removes that directory when it ends, whatever the
    /// outcome; nothing else in `scratch_dir` is touched. Default: the
    /// directory of `output`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scratch_dir: Option<PathBuf>,
    /// The package's graph id. Default: the input's file name without
    /// `.osm.pbf` (`monaco` for `monaco.osm.pbf`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub graph_id: Option<String>,
    /// The ISO country code recorded in the package (`"PL"`), if any.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub country: Option<String>,
    /// Memory against speed for the ingest; see [`MemoryProfile`].
    #[serde(default)]
    pub memory: MemoryProfile,
    /// Threads for the compile: a rayon pool of this many is built for it (the
    /// global pool is not used) and the ingest runs this many decode workers,
    /// plus a reader thread. `None` runs on the current rayon pool (the global
    /// one, or the caller's inside `ThreadPool::install`, which is how a
    /// caller sets thread names, stack sizes or priorities) with one decode
    /// worker per thread of that pool (at most two with
    /// [`MemoryProfile::Low`]). The package's bytes are the same for every
    /// thread count.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub threads: Option<usize>,
    /// The profiles whose costs the package stores, a column each
    /// (`default` is `wheeled`'s); the engine derives the others' costs from
    /// the package when a profile is first used. A stored column adds about
    /// 8 bytes an edge to the package and is customized when the package
    /// opens; a derived one is computed and held in memory on first use.
    /// The order and repeats don't matter: the same set gives the same
    /// bytes. Default: `["wheeled", "on_foot"]`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub profiles: Option<Vec<MobilityProfile>>,
    /// The package format: format 2 unless set. Format 3 is a draft that
    /// only a build of the same draft revision opens; it is not part of the
    /// JSON request until it is released.
    #[doc(hidden)]
    #[serde(skip)]
    pub package_format: PackageFormat,
}

impl CompileRequest {
    /// A request with every option at its default.
    pub fn new(input: impl Into<PathBuf>, output: impl Into<PathBuf>) -> Self {
        Self {
            input: input.into(),
            output: output.into(),
            scratch_dir: None,
            graph_id: None,
            country: None,
            memory: MemoryProfile::default(),
            threads: None,
            profiles: None,
            package_format: PackageFormat::V2,
        }
    }

    /// See [`CompileRequest::scratch_dir`](#structfield.scratch_dir).
    pub fn scratch_dir(mut self, dir: impl Into<PathBuf>) -> Self {
        self.scratch_dir = Some(dir.into());
        self
    }

    /// See [`CompileRequest::graph_id`](#structfield.graph_id).
    pub fn graph_id(mut self, graph_id: impl Into<String>) -> Self {
        self.graph_id = Some(graph_id.into());
        self
    }

    /// See [`CompileRequest::country`](#structfield.country).
    pub fn country(mut self, country: impl Into<String>) -> Self {
        self.country = Some(country.into());
        self
    }

    /// See [`MemoryProfile`].
    pub fn memory(mut self, memory: MemoryProfile) -> Self {
        self.memory = memory;
        self
    }

    /// See [`CompileRequest::threads`](#structfield.threads).
    pub fn threads(mut self, threads: usize) -> Self {
        self.threads = Some(threads);
        self
    }

    /// See [`CompileRequest::profiles`](#structfield.profiles).
    pub fn profiles(mut self, profiles: impl IntoIterator<Item = MobilityProfile>) -> Self {
        self.profiles = Some(profiles.into_iter().collect());
        self
    }

    /// The package build's options: the profiles to store, in slot order
    /// without repeats, so that the same set always gives the same columns.
    pub(crate) fn package_build_options(&self) -> PackageBuildOptions {
        let mut options = PackageBuildOptions::default();
        if let Some(profiles) = &self.profiles {
            let mut profiles = profiles.clone();
            profiles.sort_by_key(|profile| profile_slot(*profile));
            profiles.dedup();
            options.profiles = profiles;
        }
        options.package_format = self.package_format;
        options
    }

    /// The graph id the package records: the request's, or one from the input
    /// file's name.
    pub(crate) fn resolved_graph_id(&self) -> String {
        if let Some(graph_id) = &self.graph_id {
            return graph_id.clone();
        }
        self.input
            .file_stem()
            .and_then(|stem| stem.to_str())
            .map(|stem| stem.trim_end_matches(".osm").to_string())
            .filter(|stem| !stem.is_empty())
            .unwrap_or_else(|| "routing".into())
    }

    fn validate(&self) -> Result<(), CompileError> {
        let invalid = |message: &str| Err(CompileError::InvalidRequest(message.into()));
        if self.input.as_os_str().is_empty() {
            return invalid("the input path is empty");
        }
        if self.output.file_name().is_none() {
            return invalid("the output path names no file");
        }
        if self.input == self.output {
            return invalid("the output path is the input path");
        }
        if self.threads == Some(0) {
            return invalid("the compile needs at least one thread");
        }
        if self.graph_id.as_deref().is_some_and(str::is_empty) {
            return invalid("the graph id is empty");
        }
        if self.country.as_deref().is_some_and(str::is_empty) {
            return invalid("the country code is empty");
        }
        if self.profiles.as_ref().is_some_and(Vec::is_empty) {
            return invalid("the profile list is empty");
        }
        Ok(())
    }
}

/// Memory against speed. It changes how much the ingest holds at once (batch
/// and queue sizes) and its default number of decode workers, and so memory
/// use and speed; never the package's bytes.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum MemoryProfile {
    /// Large batches and deep queues: the fastest, for desktops and servers.
    #[default]
    Standard,
    /// Small batches and queues and at most two default decode workers: for
    /// phones and other memory-constrained devices.
    Low,
}

impl MemoryProfile {
    fn ingest_profile(self) -> IngestBuildProfile {
        match self {
            Self::Standard => IngestBuildProfile::DesktopFull,
            Self::Low => IngestBuildProfile::MobileOperational,
        }
    }
}

/// One progress report, for every stage alike. Reports arrive in stage order;
/// within a stage `stage_fraction` never decreases, and `overall` never
/// decreases over a compile. The last report of a successful compile is
/// [`CompileStage::WritePackage`] at fraction 1 (`overall` 1).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct CompileProgress {
    pub stage: CompileStage,
    /// The share of this stage done, 0 to 1, when the stage's size is known:
    /// blobs read in the ingest, steps of the graph and package builds,
    /// bytes written.
    pub stage_fraction: Option<f64>,
    /// The share of the whole compile done, 0 to 1 (stages weighted by their
    /// usual share of the time).
    pub overall: f64,
    /// Diagnostic text in English, such as the objects read so far in the
    /// ingest ("Located 1200 of 1250 nodes in 4000 scanned"). An app shows its
    /// own text per stage.
    pub message: Option<String>,
}

impl CompileProgress {
    /// The report before the compile's first.
    pub fn starting() -> Self {
        Self {
            stage: CompileStage::CollectRoads,
            stage_fraction: None,
            overall: 0.0,
            message: None,
        }
    }

    pub(crate) fn of(report: &Report) -> Self {
        Self {
            stage: report.stage,
            stage_fraction: report.fraction,
            overall: report.overall,
            message: (!report.message.is_empty()).then(|| report.message.clone()),
        }
    }
}

/// The package a compile wrote. Later releases may add fields.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
#[non_exhaustive]
pub struct CompileResult {
    /// The package's path (the request's `output`).
    pub package: PathBuf,
    pub byte_len: u64,
    /// The package's root checksum.
    pub dataset_hash: String,
    /// The package layout version (`ROUTING_PACKAGE_FORMAT_VERSION`).
    pub format_version: u32,
    /// The costing model the package's costs were baked with.
    pub costing_model_version: u32,
    pub graph_id: String,
    pub country: Option<String>,
    /// Nodes and directed edges in the package.
    pub node_count: u64,
    pub edge_count: u64,
    /// What the package build stored (restrictions, shapes, pieces, ...)
    /// and its timings.
    pub report: PackageBuildReport,
    /// The road graph's own manifest: counts before the package build and its
    /// quality evaluation.
    #[serde(skip)]
    pub graph_manifest: RoadGraphPackageManifest,
}

/// Why a compile did not produce a package.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum CompileError {
    /// The request cannot be compiled as given (no thread, an output path
    /// without a file name, ...). Nothing was started.
    #[error("invalid compile request: {0}")]
    InvalidRequest(String),
    /// The cancel token was cancelled. No package, temporary file or scratch
    /// directory is left behind.
    #[error("the routing compile was cancelled")]
    Cancelled,
    /// The compile failed (unreadable input, no space, a graph the package
    /// cannot hold, ...). Nothing is left behind either.
    #[error("{0}")]
    Failed(String),
}

impl CompileError {
    /// A stable id for the kind of error: `"invalid_request"`, `"cancelled"`
    /// or `"failed"`.
    pub fn code(&self) -> &'static str {
        match self {
            Self::InvalidRequest(_) => "invalid_request",
            Self::Cancelled => "cancelled",
            Self::Failed(_) => "failed",
        }
    }

    pub fn is_cancelled(&self) -> bool {
        matches!(self, Self::Cancelled)
    }
}

/// `{"code": "...", "message": "..."}`.
impl Serialize for CompileError {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeStruct;
        let mut error = serializer.serialize_struct("CompileError", 2)?;
        error.serialize_field("code", self.code())?;
        error.serialize_field("message", &self.to_string())?;
        error.end()
    }
}

impl From<String> for CompileError {
    fn from(message: String) -> Self {
        Self::Failed(message)
    }
}

impl From<job::Cancelled> for CompileError {
    fn from(_: job::Cancelled) -> Self {
        Self::Cancelled
    }
}

/// So callers that return `Result<_, String>` can keep using `?`.
impl From<CompileError> for String {
    fn from(error: CompileError) -> Self {
        error.to_string()
    }
}

/// Compiles `request.input` into a routing package at `request.output`.
///
/// Made to run inside an app's process, on a background thread:
///
/// - `on_progress` gets every stage's reports, ingest included
///   ([`CompileProgress`]). It may be called from the compile's pool
///   threads, hence `Send`.
/// - `cancel` is checked at least every second or so in every stage on a
///   desktop; once it is cancelled the compile stops with
///   [`CompileError::Cancelled`].
/// - On cancel and on error the scratch directory, any partial package and
///   any directory the compile created are removed; a package at
///   `request.output` from before is kept.
/// - It prints nothing, never exits the process, installs no allocator and
///   reads no environment or working directory: every path comes from the
///   request.
pub fn compile(
    request: &CompileRequest,
    cancel: &CancelToken,
    on_progress: impl FnMut(&CompileProgress) + Send,
) -> Result<CompileResult, CompileError> {
    let mut on_progress = on_progress;
    compile_reporting(request, &|| cancel.is_cancelled(), &mut |report| {
        on_progress(&CompileProgress::of(report))
    })
}

/// [`compile`], polling `should_cancel` (tests count the polls) and sending
/// the internal reports.
pub(crate) fn compile_reporting(
    request: &CompileRequest,
    should_cancel: &(dyn Fn() -> bool + Sync),
    on_report: &mut (dyn FnMut(&Report) + Send),
) -> Result<CompileResult, CompileError> {
    request.validate()?;
    let graph_id = request.resolved_graph_id();
    let scratch_root = match &request.scratch_dir {
        Some(dir) => dir.clone(),
        None => request
            .output
            .parent()
            .map(Path::to_path_buf)
            .unwrap_or_default(),
    };
    // Directories the compile may create (the output's and the scratch's
    // missing parents): removed again at the end if they are left empty.
    let mut created = missing_ancestors(&scratch_root);
    if let Some(parent) = request.output.parent() {
        created.extend(missing_ancestors(parent));
    }
    let scratch = fresh_scratch_dir(&scratch_root, &request.output);
    let internal = RouteArchiveCompileRequest {
        graph: RoadGraphCompileRequest {
            input_pbf_path: request.input.clone(),
            temp_dir: scratch,
            graph_id: graph_id.clone(),
            country_code: request.country.clone(),
            build_profile: request.memory.ingest_profile(),
            segment_order: crate::SegmentOrder::Topology,
        },
        route_archive_path: request.output.clone(),
        threads: request.threads,
        package_options: request.package_build_options(),
    };
    let result = compile_route_archive(&internal, should_cancel, on_report);
    // Deepest first, so a parent goes once its children have.
    created.sort_by_key(|dir| std::cmp::Reverse(dir.components().count()));
    created.dedup();
    for dir in created {
        let _ = std::fs::remove_dir(dir);
    }
    let written = result?;
    Ok(CompileResult {
        package: request.output.clone(),
        byte_len: written.byte_len,
        dataset_hash: written.dataset_hash,
        format_version: written.format_version,
        costing_model_version: written.costing_model_version,
        graph_id,
        country: request.country.clone(),
        node_count: written.node_count,
        edge_count: written.edge_count,
        report: written.report,
        graph_manifest: written.graph_manifest,
    })
}

/// `dir` and those of its ancestors that do not exist.
fn missing_ancestors(dir: &Path) -> Vec<PathBuf> {
    dir.ancestors()
        .filter(|ancestor| !ancestor.as_os_str().is_empty())
        .take_while(|ancestor| !ancestor.exists())
        .map(Path::to_path_buf)
        .collect()
}

/// A path inside `root` that does not exist yet, named after the package:
/// `<root>/<output file name>.scratch-<process id>-<n>`.
fn fresh_scratch_dir(root: &Path, output: &Path) -> PathBuf {
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let name = output
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_else(|| "routing.mrt".into());
    loop {
        let n = NEXT.fetch_add(1, Ordering::Relaxed);
        let dir = root.join(format!("{name}.scratch-{}-{n}", std::process::id()));
        if !dir.exists() {
            return dir;
        }
    }
}

/// For the `wayfarer` binary's host protocol (protocol version 1), which
/// sends the ingest's own counts and pass names; not for other use, and not
/// covered by the crate's API guarantees.
#[doc(hidden)]
pub mod host_protocol {
    use super::*;

    /// What a report carries beyond [`CompileProgress`].
    pub struct Detail<'a> {
        /// Objects read (ingest), ways pushed (graph) or bytes written
        /// (write); 0 where the stage counts nothing.
        pub rows_written: usize,
        /// Set when the report forwards one of the ingest's own.
        pub ingest: Option<IngestDetail<'a>>,
    }

    pub struct IngestDetail<'a> {
        /// The ingest pass (`ingest_pass1_index`, `ingest_pass2_ways`, ...).
        pub stage: &'static str,
        pub batches_read: usize,
        pub objects_read: usize,
        pub message: &'a str,
        /// The ingest's own fraction (not clamped).
        pub fraction: Option<f64>,
    }

    /// [`compile`](super::compile), with each report's [`Detail`].
    pub fn compile(
        request: &CompileRequest,
        cancel: &CancelToken,
        on_progress: impl FnMut(&CompileProgress, &Detail<'_>) + Send,
    ) -> Result<CompileResult, CompileError> {
        let mut on_progress = on_progress;
        compile_reporting(request, &|| cancel.is_cancelled(), &mut |report| {
            let detail = Detail {
                rows_written: report.rows_written,
                ingest: report.ingest.as_ref().map(|ingest| IngestDetail {
                    stage: ingest.stage.as_str(),
                    batches_read: ingest.batches_read,
                    objects_read: ingest.objects_read,
                    message: &ingest.message,
                    fraction: ingest.fraction,
                }),
            };
            on_progress(&CompileProgress::of(report), &detail)
        })
    }
}

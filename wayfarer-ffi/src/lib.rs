//! C ABI for the routing engine, version 1 (`include/wayfarer.h`).
//!
//! A host opens a routing package and gets an opaque handle, then sends route,
//! matrix, isochrone, waypoint workflow and map matching requests as JSON (the
//! engine's serde types) and gets JSON back; navigation progress runs on a
//! prepared-route handle. Every call returns a status code; on failure the out
//! string holds an error object `{"code": ..., "message": ...}` (route
//! failures add the search's `diagnostics`). Strings the library returns are
//! freed with [`wayfarer_string_free`]. No panic crosses the boundary.
//!
//! Cancel tokens ([`wayfarer_cancel_token_new`]) are taken by the
//! `_with_cancel` variants and the other long-running request functions:
//! cancelling one, or its deadline passing, stops the request within one
//! cancellation check with [`WAYFARER_CANCELLED`].
//!
//! Package handles, cancel tokens and prepared navigation routes may be used
//! from several threads at once; free each only after every call using it has
//! returned. The header documents ownership and JSON casing per type.
//!
//! The header is `include/wayfarer.h` and `examples/route.c` is a
//! complete C client. From Rust, the same functions are plain `extern "C"`
//! items:
//!
//! ```
//! use wayfarer_ffi::{
//!     WAYFARER_ABI_VERSION, wayfarer_abi_version,
//!     wayfarer_string_free, wayfarer_versions,
//! };
//! use std::ffi::CStr;
//!
//! assert_eq!(wayfarer_abi_version(), WAYFARER_ABI_VERSION);
//!
//! // The library, ABI, readable package formats and costing model, as JSON.
//! let text = wayfarer_versions();
//! let json = unsafe { CStr::from_ptr(text) }.to_str().unwrap().to_owned();
//! // SAFETY: `text` came from this library and is freed exactly once.
//! unsafe { wayfarer_string_free(text) };
//! assert!(json.contains("readableFormats"));
//! ```
//!
//! # Stability
//!
//! [`WAYFARER_ABI_VERSION`] changes when the header changes incompatibly.
//! The package formats and costing model the library accepts are those of the engine it wraps (see
//! `wayfarer_versions`); the JSON shapes follow the engine's types.

use std::ffi::{CStr, CString, c_char};
use std::mem::ManuallyDrop;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::Path;
use std::ptr::{self, NonNull};
use std::sync::Arc;
use std::time::Duration;

use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use wayfarer::{
    CancelToken, MapMatchRequest, NavigationPositionSample, PackageQualityStatus,
    PreparedNavigationRoute, ProjectRequest, RouteCancellation, RouteError, RouteIsochroneRequest,
    RouteMatrixRequest, RouteNavigationInput, RouteNavigationProgress,
    RouteNavigationProgressOptions, RoutePackageMemoryProfile, RouteResult, RoutingEngine,
    RoutingPackage, RoutingPackageError, SnapRequest, limits,
};
use wayfarer_service::{
    RouteWorkflowError, RouteWorkflowRequest, RouteWorkflowSourceContext, RoutingSourceId,
    compile_route_workflow,
};

#[cfg(feature = "compiler")]
mod compile;
#[cfg(feature = "compiler")]
pub use compile::*;

/// The version of this C ABI. It changes when `wayfarer.h` changes
/// incompatibly.
pub const WAYFARER_ABI_VERSION: u32 = 1;

/// Success; the out string holds the result.
pub const WAYFARER_OK: i32 = 0;
/// A null pointer, a string that is not UTF-8, or JSON that does not parse
/// as the request.
pub const WAYFARER_INVALID_ARGUMENT: i32 = 1;
/// The package could not be opened or verified (`code`: `not_found`, `io`,
/// `invalid_package`, `outdated_package`, `package_too_new`, `unsupported`).
pub const WAYFARER_PACKAGE_ERROR: i32 = 2;
/// The request was valid but has no answer (`code`: `no_candidate`, `no_path`,
/// `disconnected`, `search_budget_exceeded`, ...).
pub const WAYFARER_ROUTE_ERROR: i32 = 3;
/// A bug in the library (a caught panic).
pub const WAYFARER_INTERNAL_ERROR: i32 = 4;
/// The request's cancel token was cancelled (`code`: `cancelled`) or its
/// deadline passed (`deadline_exceeded`). Only calls that take a token
/// return it.
pub const WAYFARER_CANCELLED: i32 = 5;

/// An opened routing package. Opaque to C.
pub struct WayfarerPackage {
    engine: RoutingEngine,
}

/// A cancel token with an optional deadline. Opaque to C.
pub struct WayfarerCancelToken {
    token: CancelToken,
    cancellation: RouteCancellation,
}

/// A route prepared for navigation progress. Opaque to C.
pub struct WayfarerNavigationRoute {
    /// Borrows `*input`, so it is dropped first.
    prepared: ManuallyDrop<PreparedNavigationRoute<'static>>,
    /// Owned: from `Box::leak`, freed in `drop`.
    input: NonNull<RouteNavigationInput>,
}

impl Drop for WayfarerNavigationRoute {
    fn drop(&mut self) {
        // SAFETY: `prepared` is dropped once, before the input it borrows,
        // which came from `Box::leak` and is freed once.
        unsafe {
            ManuallyDrop::drop(&mut self.prepared);
            drop(Box::from_raw(self.input.as_ptr()));
        }
    }
}

#[derive(Deserialize, Default)]
#[serde(rename_all = "camelCase", default, deny_unknown_fields)]
struct OpenOptions {
    memory_profile: RoutePackageMemoryProfile,
}

struct Failure {
    status: i32,
    error: Value,
}

impl Failure {
    fn new(status: i32, code: &str, message: impl Into<String>) -> Self {
        Self {
            status,
            error: json!({ "code": code, "message": message.into() }),
        }
    }

    fn invalid(message: impl Into<String>) -> Self {
        Self::new(WAYFARER_INVALID_ARGUMENT, "invalid_argument", message)
    }

    fn package(error: RoutingPackageError) -> Self {
        let code = if error.is_missing() {
            "not_found"
        } else {
            match error {
                RoutingPackageError::Io(_) => "io",
                RoutingPackageError::Invalid(_) => "invalid_package",
                RoutingPackageError::Outdated(_) => "outdated_package",
                RoutingPackageError::TooNew(_) => "package_too_new",
                RoutingPackageError::Unsupported(_) => "unsupported",
            }
        };
        Self::new(WAYFARER_PACKAGE_ERROR, code, error.to_string())
    }

    fn route(error: &RouteError) -> Self {
        let status = if error.is_cancelled() {
            WAYFARER_CANCELLED
        } else if error.is_invalid_request() {
            WAYFARER_INVALID_ARGUMENT
        } else {
            WAYFARER_ROUTE_ERROR
        };
        let code = match error {
            RouteError::GraphNotReady => "graph_not_ready",
            RouteError::NoCandidate { .. } => "no_candidate",
            RouteError::ImpossibleMobilityProfile { .. } => "impossible_mobility_profile",
            RouteError::Disconnected => "disconnected",
            RouteError::BlockedCorridor { .. } => "blocked_corridor",
            RouteError::NoPath { .. } => "no_path",
            RouteError::SearchBudgetExceeded { .. } => "search_budget_exceeded",
            RouteError::MissingEdgeMetadata { .. } => "missing_edge_metadata",
            RouteError::Cancelled {
                deadline_exceeded: false,
            } => "cancelled",
            RouteError::Cancelled {
                deadline_exceeded: true,
            } => "deadline_exceeded",
            RouteError::SarUnavailable { .. } => "sar_unavailable",
            RouteError::InvalidRequest { .. } => "invalid_request",
            RouteError::RequestTooLarge { .. } => "request_too_large",
            RouteError::NoMatch => "no_match",
        };
        let mut failure = Self::new(status, code, error.to_string());
        if let RouteError::SarUnavailable { relaxation } = error {
            failure.error["relaxation"] = json!(relaxation);
        }
        failure
    }

    fn workflow(error: RouteWorkflowError) -> Self {
        match error {
            RouteWorkflowError::InvalidRequest(_) => Self::invalid(error.to_string()),
            RouteWorkflowError::RequestTooLarge(_) => Self::new(
                WAYFARER_INVALID_ARGUMENT,
                "request_too_large",
                error.to_string(),
            ),
            RouteWorkflowError::OverlaySet(_) => Self::new(
                WAYFARER_INVALID_ARGUMENT,
                "invalid_overlay_set",
                error.to_string(),
            ),
            RouteWorkflowError::Route(failure) => {
                let mut error = Self::route(&failure.error);
                if let Some(index) = failure.segment_index {
                    error.error["segmentIndex"] = json!(index);
                }
                error.error["diagnostics"] =
                    serde_json::to_value(&failure.diagnostics).expect("diagnostics serialize");
                error
            }
        }
    }
}

/// Runs `body`, writes its result or error as JSON to `out`, and returns the
/// status. Panics become `WAYFARER_INTERNAL_ERROR`.
///
/// # Safety
/// `out` is null or valid for one pointer write.
unsafe fn respond<T: Serialize>(
    out: *mut *mut c_char,
    body: impl FnOnce() -> Result<T, Failure>,
) -> i32 {
    let outcome = catch_unwind(AssertUnwindSafe(|| {
        // Serialized once, straight from the engine's types.
        body().and_then(|value| {
            serde_json::to_string(&value).map_err(|error| {
                Failure::new(WAYFARER_INTERNAL_ERROR, "internal", error.to_string())
            })
        })
    }));
    let (status, text) = match outcome {
        Ok(Ok(text)) => (WAYFARER_OK, text),
        Ok(Err(failure)) => (failure.status, failure.error.to_string()),
        Err(panic) => {
            let message = panic
                .downcast_ref::<&str>()
                .map(|text| text.to_string())
                .or_else(|| panic.downcast_ref::<String>().cloned())
                .unwrap_or_else(|| "panic".into());
            (
                WAYFARER_INTERNAL_ERROR,
                json!({ "code": "internal", "message": message }).to_string(),
            )
        }
    };
    if !out.is_null() {
        // JSON escapes control characters, so the text holds no NUL.
        let text = CString::new(text).expect("JSON has no NUL");
        // SAFETY: the caller passes a writable pointer or null (checked above).
        unsafe { *out = text.into_raw() };
    }
    status
}

/// # Safety
/// `text` is null or a NUL-terminated string.
unsafe fn read_str<'a>(text: *const c_char, what: &str) -> Result<&'a str, Failure> {
    if text.is_null() {
        return Err(Failure::invalid(format!("{what} is null")));
    }
    // SAFETY: non-null and NUL-terminated per the contract.
    unsafe { CStr::from_ptr(text) }
        .to_str()
        .map_err(|_| Failure::invalid(format!("{what} is not UTF-8")))
}

/// A path passed as a NUL-terminated UTF-16 string (`wchar_t` on Windows).
/// Any sequence of units is a valid Windows path, unpaired surrogates too.
///
/// # Safety
/// `text` is null or a NUL-terminated UTF-16 string.
#[cfg(windows)]
unsafe fn read_wide_path(text: *const u16, what: &str) -> Result<std::path::PathBuf, Failure> {
    use std::os::windows::ffi::OsStringExt;
    if text.is_null() {
        return Err(Failure::invalid(format!("{what} is null")));
    }
    let mut len = 0;
    // SAFETY: NUL-terminated per the contract, so every unit up to the NUL
    // is readable.
    while unsafe { *text.add(len) } != 0 {
        len += 1;
    }
    // SAFETY: the `len` units before the NUL, read above.
    let units = unsafe { std::slice::from_raw_parts(text, len) };
    Ok(std::ffi::OsString::from_wide(units).into())
}

/// # Safety
/// `text` is null or a NUL-terminated string.
unsafe fn read_json<T: for<'de> Deserialize<'de>>(
    text: *const c_char,
    what: &str,
) -> Result<T, Failure> {
    let text = unsafe { read_str(text, what) }?;
    serde_json::from_str(text).map_err(|error| Failure::invalid(format!("{what}: {error}")))
}

/// # Safety
/// `package` is null or a handle from `wayfarer_open` not yet closed.
unsafe fn engine<'a>(package: *const WayfarerPackage) -> Result<&'a RoutingEngine, Failure> {
    if package.is_null() {
        return Err(Failure::invalid("package is null"));
    }
    // SAFETY: a live handle per the contract.
    Ok(unsafe { &(*package).engine })
}

/// The cancellation of `cancel`, or none for null.
///
/// # Safety
/// `cancel` is null or a live token from `wayfarer_cancel_token_new`.
unsafe fn cancellation(cancel: *const WayfarerCancelToken) -> RouteCancellation {
    if cancel.is_null() {
        RouteCancellation::default()
    } else {
        // SAFETY: a live token per the contract.
        unsafe { (*cancel).cancellation.clone() }
    }
}

fn package_info(package: &RoutingPackage) -> Value {
    let metadata = package.metadata();
    let counts = package.counts();
    json!({
        "formatVersion": package.format_version(),
        "costingModelVersion": metadata.costing_model_version,
        "graphId": metadata.graph_id,
        "countryCode": metadata.country_code,
        "datasetHash": package.dataset_hash(),
        "byteLen": package.byte_len(),
        "nodeCount": counts.node_count,
        "edgeCount": counts.edge_count,
        "memoryProfile": package.memory_profile(),
        "quality": {
            "status": metadata.quality.status,
            "reasons": metadata.quality.reasons,
            "ready": metadata.quality.status != PackageQualityStatus::Stale,
        },
        "sarCapability": package.sar_capability(),
    })
}

/// The version of this C ABI ([`WAYFARER_ABI_VERSION`]).
#[unsafe(no_mangle)]
pub extern "C" fn wayfarer_abi_version() -> u32 {
    WAYFARER_ABI_VERSION
}

/// The library's versions as JSON: crate version, ABI version, the package
/// formats it reads, and its costing model. The caller frees the string.
#[unsafe(no_mangle)]
pub extern "C" fn wayfarer_versions() -> *mut c_char {
    let value = json!({
        "library": env!("CARGO_PKG_VERSION"),
        "abi": WAYFARER_ABI_VERSION,
        "readableFormats": [
            wayfarer_package::format::MIN_READ_FORMAT_VERSION,
            wayfarer::ROUTING_PACKAGE_FORMAT_VERSION,
        ],
        "costingModel": wayfarer_costing::COSTING_MODEL_VERSION,
        // Whether this build has the wayfarer_compile_* calls.
        "compiler": cfg!(feature = "compiler"),
    });
    CString::new(value.to_string())
        .expect("JSON has no NUL")
        .into_raw()
}

/// Opens the routing package at `path` (UTF-8). `options_json` is null or
/// `{"memoryProfile": "desktop" | "mobile"}`. On success `*package` is the new
/// handle and `*out_json` (if `out_json` is not null) the package info.
///
/// On Windows a C program's `argv` and the Win32 "A" functions use the ANSI
/// code page, not UTF-8: convert with `WideCharToMultiByte(CP_UTF8, ...)`,
/// or pass the UTF-16 path to `wayfarer_open_w`.
///
/// # Safety
/// `path` and `options_json` are null or NUL-terminated strings; `package` is
/// valid for one pointer write; `out_json` is null or valid for one write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_open(
    path: *const c_char,
    options_json: *const c_char,
    package: *mut *mut WayfarerPackage,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe {
        respond(out_json, || {
            let path = read_str(path, "path")?;
            open_package(Path::new(path), options_json, package)
        })
    }
}

/// [`wayfarer_open`] with the path as a NUL-terminated UTF-16 string
/// (`const wchar_t *`), as Windows hands it to `wmain` and the Win32 "W"
/// functions. Windows only.
///
/// # Safety
/// `path` is null or a NUL-terminated UTF-16 string; otherwise as
/// [`wayfarer_open`].
#[cfg(windows)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_open_w(
    path: *const u16,
    options_json: *const c_char,
    package: *mut *mut WayfarerPackage,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe {
        respond(out_json, || {
            let path = read_wide_path(path, "path")?;
            open_package(&path, options_json, package)
        })
    }
}

/// The body of [`wayfarer_open`] once the path is read.
///
/// # Safety
/// As [`wayfarer_open`].
unsafe fn open_package(
    path: &Path,
    options_json: *const c_char,
    package: *mut *mut WayfarerPackage,
) -> Result<Value, Failure> {
    if package.is_null() {
        return Err(Failure::invalid("package out-pointer is null"));
    }
    let options: OpenOptions = if options_json.is_null() {
        OpenOptions::default()
    } else {
        unsafe { read_json(options_json, "options") }?
    };
    let opened = RoutingPackage::open_with_profile(path, options.memory_profile)
        .map_err(Failure::package)?;
    let info = package_info(&opened);
    let handle = Box::new(WayfarerPackage {
        engine: RoutingEngine::from_package(Arc::new(opened)),
    });
    // SAFETY: checked non-null; valid for one write per the contract.
    unsafe { *package = Box::into_raw(handle) };
    Ok(info)
}

/// Closes a package handle. Null is ignored.
///
/// # Safety
/// `package` is null or a handle from `wayfarer_open`, with no call on
/// it in flight; it is closed exactly once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_close(package: *mut WayfarerPackage) {
    if !package.is_null() {
        // SAFETY: an owned handle per the contract.
        drop(unsafe { Box::from_raw(package) });
    }
}

/// The package's info (format and costing versions, counts, dataset hash,
/// quality) as JSON.
///
/// # Safety
/// `package` is a live handle; `out_json` is null or valid for one write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_info(
    package: *const WayfarerPackage,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe { respond(out_json, || Ok(package_info(engine(package)?.package()))) }
}

/// Checks a package file end to end (every section against its checksum).
/// On success `*out_json` holds the dataset hash, length and sections.
/// `path` is UTF-8, as for [`wayfarer_open`].
///
/// # Safety
/// `path` is null or a NUL-terminated string; `out_json` is null or valid for
/// one write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_verify(path: *const c_char, out_json: *mut *mut c_char) -> i32 {
    unsafe {
        respond(out_json, || {
            let path = read_str(path, "path")?;
            verify_package(Path::new(path))
        })
    }
}

/// [`wayfarer_verify`] with the path as a NUL-terminated UTF-16 string
/// (`const wchar_t *`). Windows only.
///
/// # Safety
/// `path` is null or a NUL-terminated UTF-16 string; `out_json` is null or
/// valid for one write.
#[cfg(windows)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_verify_w(path: *const u16, out_json: *mut *mut c_char) -> i32 {
    unsafe {
        respond(out_json, || {
            let path = read_wide_path(path, "path")?;
            verify_package(&path)
        })
    }
}

fn verify_package(path: &Path) -> Result<Value, Failure> {
    let verification = RoutingPackage::verify(path).map_err(Failure::package)?;
    Ok(json!({
        "datasetHash": verification.dataset_hash,
        "byteLen": verification.byte_len,
        "sections": verification.sections.iter().map(|section| json!({
            "name": section.name,
            "byteLen": section.byte_len,
            "blake3": section.blake3_hex,
        })).collect::<Vec<_>>(),
    }))
}

/// Routes `request_json`: a `RouteRequest` (two or more locations, legs,
/// overlays) with optional `overlaySets` and `mobility`. On success
/// `*out_json` is the route (a `RouteResult` with the package's id and
/// revision); on a failed leg, an error with its `segmentIndex` and the
/// search's diagnostics.
///
/// # Safety
/// `package` is a live handle; `request_json` is null or a NUL-terminated
/// string; `out_json` is null or valid for one write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_route(
    package: *const WayfarerPackage,
    request_json: *const c_char,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe { wayfarer_route_with_cancel(package, request_json, ptr::null(), out_json) }
}

/// [`wayfarer_route`], stopped by `cancel` (null: never) with
/// [`WAYFARER_CANCELLED`].
///
/// # Safety
/// As for [`wayfarer_route`]; `cancel` is null or a live token.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_route_with_cancel(
    package: *const WayfarerPackage,
    request_json: *const c_char,
    cancel: *const WayfarerCancelToken,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe {
        respond(out_json, || {
            let engine = engine(package)?;
            let mut request: RouteWorkflowRequest = read_json(request_json, "request")?;
            request.route.options.cancellation = cancellation(cancel);
            let package = engine.package();
            let source = RouteWorkflowSourceContext {
                source_id: RoutingSourceId::new(package.graph_id())
                    .or_else(|_| RoutingSourceId::new("package"))
                    .expect("a valid source id"),
                revision: package.dataset_hash().to_string(),
                cache_hit: false,
            };
            compile_route_workflow(engine, source, request).map_err(Failure::workflow)
        })
    }
}

/// Answers `request_json` (a `RouteMatrixRequest`). Unreachable cells are
/// reported per cell in the `RouteMatrixResult`, not as an error.
///
/// # Safety
/// As for [`wayfarer_route`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_matrix(
    package: *const WayfarerPackage,
    request_json: *const c_char,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe { wayfarer_matrix_with_cancel(package, request_json, ptr::null(), out_json) }
}

/// [`wayfarer_matrix`], stopped by `cancel` (null: never): a
/// cancelled matrix is an error with [`WAYFARER_CANCELLED`], not a
/// result of failed cells.
///
/// # Safety
/// As for [`wayfarer_route`]; `cancel` is null or a live token.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_matrix_with_cancel(
    package: *const WayfarerPackage,
    request_json: *const c_char,
    cancel: *const WayfarerCancelToken,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe {
        respond(out_json, || {
            let engine = engine(package)?;
            let mut request: RouteMatrixRequest = read_json(request_json, "request")?;
            request.options.cancellation = cancellation(cancel);
            let matrix = engine.matrix(request);
            // A cancelled matrix fails every searchable cell alike.
            for cell in &matrix.cells {
                let deadline_exceeded = match cell.failure_reason.as_deref() {
                    Some("cancelled") => false,
                    Some("deadline_exceeded") => true,
                    _ => continue,
                };
                return Err(Failure::route(&RouteError::Cancelled { deadline_exceeded }));
            }
            Ok(matrix)
        })
    }
}

/// Answers `request_json` (a `RouteIsochroneRequest`) with a
/// `RouteIsochroneResult`.
///
/// # Safety
/// As for [`wayfarer_route`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_isochrone(
    package: *const WayfarerPackage,
    request_json: *const c_char,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe { wayfarer_isochrone_with_cancel(package, request_json, ptr::null(), out_json) }
}

/// [`wayfarer_isochrone`], stopped by `cancel` (null: never) with
/// [`WAYFARER_CANCELLED`].
///
/// # Safety
/// As for [`wayfarer_route`]; `cancel` is null or a live token.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_isochrone_with_cancel(
    package: *const WayfarerPackage,
    request_json: *const c_char,
    cancel: *const WayfarerCancelToken,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe {
        respond(out_json, || {
            let engine = engine(package)?;
            let mut request: RouteIsochroneRequest = read_json(request_json, "request")?;
            request.options.cancellation = cancellation(cancel);
            engine
                .isochrone(request)
                .map_err(|error| Failure::route(&error))
        })
    }
}

/// Matches `request_json` (`{"samples": [NavigationPositionSample],
/// "profile", "options": MapMatchOptions}`) to the road network with a
/// `MapMatchResult`. A trace with no matchable sample fails with code
/// `no_match`. `cancel` is checked before the match starts.
///
/// # Safety
/// As for [`wayfarer_route`]; `cancel` is null or a live token.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_map_match(
    package: *const WayfarerPackage,
    request_json: *const c_char,
    cancel: *const WayfarerCancelToken,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe {
        respond(out_json, || {
            let engine = engine(package)?;
            let mut request: MapMatchRequest = read_json(request_json, "request")?;
            if request.samples.is_empty() {
                return Err(Failure::invalid("request: samples is empty"));
            }
            request.cancellation = cancellation(cancel);
            engine
                .map_match(request)
                .map_err(|error| Failure::route(&error))
        })
    }
}

/// Snaps `request_json` (a `SnapRequest`: `{"point", "profile",
/// "maxDistanceM", "candidateLimit", "options"}`) onto the roads its profile
/// may use. `*out_json` is the `SnapResult`, best candidate first (no
/// candidate is not an error).
///
/// # Safety
/// As for [`wayfarer_route`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_snap(
    package: *const WayfarerPackage,
    request_json: *const c_char,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe {
        respond(out_json, || {
            let engine = engine(package)?;
            let request: SnapRequest = read_json(request_json, "request")?;
            Ok(engine.snap(request))
        })
    }
}

/// Projects `request_json` (a `ProjectRequest`: `{"geometryKind": "point" |
/// "line" | "polygon", "points", "projectionDistanceM"}`) onto the
/// package's edges, as a route overlay with that geometry would be.
/// `*out_json` is the `RouteOverlayGeometryProjection`: the edge ids, the
/// partial ranges and how they were found.
///
/// # Safety
/// As for [`wayfarer_route`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_project(
    package: *const WayfarerPackage,
    request_json: *const c_char,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe {
        respond(out_json, || {
            let engine = engine(package)?;
            let request: ProjectRequest = read_json(request_json, "request")?;
            if request.points.is_empty() || request.points.len() > limits::MAX_ROUTE_OVERLAY_POINTS
            {
                return Err(Failure::invalid("request: points"));
            }
            Ok(engine.project(&request))
        })
    }
}

/// A new cancel token. With a non-zero `timeout_ms` it also stops requests
/// once that many milliseconds have passed from now (`deadline_exceeded`;
/// a timeout beyond the clock's range never passes). Free it with
/// [`wayfarer_cancel_token_free`].
#[unsafe(no_mangle)]
pub extern "C" fn wayfarer_cancel_token_new(timeout_ms: u64) -> *mut WayfarerCancelToken {
    let token = CancelToken::new();
    let mut cancellation = RouteCancellation::default().with_token(token.clone());
    if timeout_ms != 0 {
        cancellation = cancellation.with_timeout(Duration::from_millis(timeout_ms));
    }
    Box::into_raw(Box::new(WayfarerCancelToken {
        token,
        cancellation,
    }))
}

/// Cancels `cancel`: every request using it stops with
/// [`WAYFARER_CANCELLED`], and later ones fail at once. Callable from
/// any thread, any number of times; null is ignored.
///
/// # Safety
/// `cancel` is null or a live token.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_cancel_token_cancel(cancel: *const WayfarerCancelToken) {
    if !cancel.is_null() {
        // SAFETY: a live token per the contract.
        unsafe { (*cancel).token.cancel() };
    }
}

/// Frees a cancel token. Null is ignored.
///
/// # Safety
/// `cancel` is null or a token from [`wayfarer_cancel_token_new`],
/// with no call using it in flight; it is freed exactly once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_cancel_token_free(cancel: *mut WayfarerCancelToken) {
    if !cancel.is_null() {
        // SAFETY: an owned token per the contract.
        drop(unsafe { Box::from_raw(cancel) });
    }
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
struct NavigationRouteSource {
    route: Option<RouteResult>,
    input: Option<RouteNavigationInput>,
}

/// Prepares a route for navigation progress from `route_json`: either
/// `{"route": LegResult}` (as `wayfarer_route` returns it) or
/// `{"input": RouteNavigationInput}`. On success `*route` is the new handle
/// and `*out_json` (if not null) `{"geometryPointCount", "maneuverCount"}`.
/// Free the handle with [`wayfarer_navigation_route_free`].
///
/// # Safety
/// `route_json` is null or a NUL-terminated string; `route` is valid for one
/// pointer write; `out_json` is null or valid for one write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_navigation_route_new(
    route_json: *const c_char,
    route: *mut *mut WayfarerNavigationRoute,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe {
        respond(out_json, || {
            if route.is_null() {
                return Err(Failure::invalid("route out-pointer is null"));
            }
            let source: NavigationRouteSource = read_json(route_json, "route")?;
            let input = match source {
                NavigationRouteSource {
                    route: Some(result),
                    input: None,
                } => RouteNavigationInput::from_route(&result),
                NavigationRouteSource {
                    route: None,
                    input: Some(input),
                } => input,
                _ => {
                    return Err(Failure::invalid(
                        "route: give exactly one of route and input",
                    ));
                }
            };
            let input: &'static RouteNavigationInput = Box::leak(Box::new(input));
            let owned = NonNull::from(input);
            let prepared = match PreparedNavigationRoute::new(input) {
                Ok(prepared) => prepared,
                Err(message) => {
                    // SAFETY: leaked above and borrowed by nothing any more.
                    drop(Box::from_raw(owned.as_ptr()));
                    return Err(Failure::invalid(format!("route: {message}")));
                }
            };
            let info = json!({
                "geometryPointCount": input.geometry.len(),
                "maneuverCount": input.maneuvers.len(),
            });
            // SAFETY: checked non-null; valid for one write per the contract.
            *route = Box::into_raw(Box::new(WayfarerNavigationRoute {
                prepared: ManuallyDrop::new(prepared),
                input: owned,
            }));
            Ok(info)
        })
    }
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
struct NavigationProgressRequest {
    sample: NavigationPositionSample,
    #[serde(default)]
    previous: Option<RouteNavigationProgress>,
    #[serde(default)]
    options: RouteNavigationProgressOptions,
}

/// Progress along a prepared route for `request_json`: `{"sample":
/// NavigationPositionSample, "previous": RouteNavigationProgress (the last
/// result on this route, or omitted), "options":
/// RouteNavigationProgressOptions}`. On success `*out_json` is the
/// `RouteNavigationProgress`. The handle keeps no state between samples: pass
/// the previous result back.
///
/// # Safety
/// `route` is a live handle; `request_json` is null or a NUL-terminated
/// string; `out_json` is null or valid for one write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_navigation_progress(
    route: *const WayfarerNavigationRoute,
    request_json: *const c_char,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe {
        respond(out_json, || {
            if route.is_null() {
                return Err(Failure::invalid("route is null"));
            }
            let request: NavigationProgressRequest = read_json(request_json, "request")?;
            // SAFETY: a live handle per the contract.
            (*route)
                .prepared
                .progress(request.sample, request.previous.as_ref(), &request.options)
                .map_err(|message| Failure::invalid(format!("request: {message}")))
        })
    }
}

/// Frees a prepared navigation route. Null is ignored.
///
/// # Safety
/// `route` is null or a handle from [`wayfarer_navigation_route_new`],
/// with no call on it in flight; it is freed exactly once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_navigation_route_free(route: *mut WayfarerNavigationRoute) {
    if !route.is_null() {
        // SAFETY: an owned handle per the contract.
        drop(unsafe { Box::from_raw(route) });
    }
}

/// Frees a string the library returned. Null is ignored.
///
/// # Safety
/// `text` is null or a string from this library, freed exactly once.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_string_free(text: *mut c_char) {
    if !text.is_null() {
        // SAFETY: allocated by `CString::into_raw` in this library.
        drop(unsafe { CString::from_raw(text) });
    }
}

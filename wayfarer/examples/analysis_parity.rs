//! Compares matrix and isochrone results between the routing package's
//! engine and the reference oracle (`wayfarer::reference`, plain
//! Dijkstra over the road graph's own records) on a real package, with warm
//! timings.
//!
//! ```sh
//! cargo run --release -p wayfarer --example analysis_parity -- \
//!   <routing.mrt> [repeats=3]
//! ```
//!
//! The reference needs the road graph the package was compiled from
//! (`REFERENCE_ROAD_GRAPH=<snapshot>`); without it only the package engine
//! runs.
//!
//! Each engine line gives wall and process-CPU times; on a busy machine the
//! CPU figure is the steadier one.
//!
//! Points are fixed locations in Malopolskie (Krakow and the region around
//! it); the 50x50 and 100x100 matrices add a fixed pseudo-random sample of
//! road points (run those with `ANALYSIS_PACKAGE_ONLY=1`).
//! `ANALYSIS_ONLY=<label>` runs one case; `ANALYSIS_PACKAGE_ONLY=1` skips the
//! reference (for profiling); `ANALYSIS_REFERENCE_REPEATS=<n>` runs it fewer
//! times (one Dijkstra per matrix row over the road graph's records, so slow);
//! `ANALYSIS_REFERENCE_ONLY=1` skips the package engine; `ANALYSIS_CELLS=1`
//! prints the package matrix cells; `ANALYSIS_DUMP=<dir>` writes each
//! engine's full result as `<dir>/<case>.<engine>.json`, for exact
//! before/after diffs.
//!
//! `ANALYSIS_OVERLAY=1` gives every request one dynamic overlay derived from
//! two plain package routes (a Krakow drive and a regional drive): on each, a
//! full block a third of the way along and a partial block two thirds along
//! with a cheap bypass connector. Matrices are compared with the reference's
//! overlay matrices, and each package matrix cell of up to 10x10 is also
//! checked against the package overlay route between the same two points.
//! The reference models isochrone overlays of full blocks only, so overlay
//! isochrones run on the package engine alone.
//!
//! `ANALYSIS_DISCOUNT=<m>` adds a discount `m` in (0, 1) on every edge of
//! those two plain routes (cleared corridors; costed exactly, with bounds
//! scaled by the least cost ratio), and `ANALYSIS_SAR=<RouteSarOptions JSON>`
//! (e.g. `{"relaxations": {"turn_restrictions": {}}}`) SAR relaxations on
//! every wheeled request; both combine with `ANALYSIS_OVERLAY`.
//! `ANALYSIS_OFF_NETWORK=<RouteOffNetworkOptions JSON>` (e.g. `{}`) turns on
//! costed access legs for every request.
//!
//! `ANALYSIS_REFERENCE_CONTEXT=<k>` makes the reference exact under complex
//! restrictions with up to k prefix edges (one label per edge and context).
//! `ANALYSIS_SOURCE_LIST=<i,j,...>` keeps only those sources of a matrix
//! case, to check chosen rows of a large matrix against it.
//! `ANALYSIS_CHECK_CELLS=5:95,50:66` routes the listed matrix cells
//! (source:target) of each matrix case through the package engine (its
//! bidirectional and unidirectional searches) and the reference, printing
//! each cost and path length, to explain differences between matrix modes.

use std::collections::BTreeSet;
use std::sync::Arc;
use std::time::Instant;

use wayfarer::internals::Correlator;
use wayfarer::reference::ReferenceEngine;
use wayfarer::{
    CorrelationPolicy, GeoPoint, LegRequest, MobilityProfile, RouteDynamicOverlay,
    RouteDynamicVirtualEdge, RouteDynamicVirtualEdgeKind, RouteEdgeRange, RouteIsochroneContour,
    RouteIsochroneMetric, RouteIsochroneRequest, RouteIsochroneResult, RouteMatrixCellStatus,
    RouteMatrixRequest, RouteMatrixResult, RouteOptions, RoutingEngine, RoutingPackage,
};

/// The requests both engines answer.
trait Engine {
    fn matrix(&self, request: RouteMatrixRequest) -> RouteMatrixResult;
    fn isochrone(&self, request: RouteIsochroneRequest) -> Result<RouteIsochroneResult, String>;
}

impl Engine for RoutingEngine {
    fn matrix(&self, request: RouteMatrixRequest) -> RouteMatrixResult {
        RoutingEngine::matrix(self, request)
    }
    fn isochrone(&self, request: RouteIsochroneRequest) -> Result<RouteIsochroneResult, String> {
        RoutingEngine::isochrone(self, request).map_err(|err| format!("{err:?}"))
    }
}

impl Engine for ReferenceEngine<'_> {
    fn matrix(&self, request: RouteMatrixRequest) -> RouteMatrixResult {
        ReferenceEngine::matrix(self, request)
    }
    fn isochrone(&self, request: RouteIsochroneRequest) -> Result<RouteIsochroneResult, String> {
        ReferenceEngine::isochrone(self, request).map_err(|err| format!("{err:?}"))
    }
}

fn point(lng: f64, lat: f64) -> GeoPoint {
    GeoPoint::new(lat, lng)
}

fn krakow_points() -> Vec<GeoPoint> {
    vec![
        point(19.9448, 50.0647),
        point(20.0381, 50.0720),
        point(19.9384, 50.0614),
        point(19.9100, 50.0300),
        point(19.9900, 50.0950),
        point(20.0100, 50.0400),
        point(19.8900, 50.0800),
        point(19.9600, 50.0100),
    ]
}

fn regional_points() -> Vec<GeoPoint> {
    vec![
        point(19.9448, 50.0647),
        point(20.9857, 50.0121),
        point(20.6970, 49.6218),
        point(19.9525, 49.2992),
        point(20.0644, 49.9872),
        point(20.4303, 49.9691),
        point(20.0324, 49.4774),
    ]
}

/// Krakow's city area and the package's whole region, as lng/lat boxes.
const KRAKOW_BOX: [f64; 4] = [19.82, 49.98, 20.12, 50.13];
const REGION_BOX: [f64; 4] = [19.20, 49.25, 21.30, 50.45];

/// `count` points: `seeds` first, then a fixed pseudo-random sample of the
/// lng/lat box `[min_lng, min_lat, max_lng, max_lat]`, keeping only points
/// that correlate to a road for `profile` (as a matrix request would).
fn sampled_points(
    engine: &RoutingEngine,
    seeds: Vec<GeoPoint>,
    [min_lng, min_lat, max_lng, max_lat]: [f64; 4],
    count: usize,
    profile: MobilityProfile,
) -> Vec<GeoPoint> {
    let correlator = Correlator::new(engine.route_graph_view());
    let options = RouteOptions {
        max_correlation_distance_m: 150,
        ..RouteOptions::default()
    };
    let policy = CorrelationPolicy::from(&options);
    let mut state = 0x9e37_79b9_7f4a_7c15_u64 ^ count as u64;
    let mut unit = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 11) as f64 / (1u64 << 53) as f64
    };
    let mut points = seeds;
    points.truncate(count);
    let mut attempts = 0;
    while points.len() < count && attempts < count * 100 {
        attempts += 1;
        let candidate = point(
            min_lng + (max_lng - min_lng) * unit(),
            min_lat + (max_lat - min_lat) * unit(),
        );
        if correlator
            .candidates(candidate, profile, &policy)
            .is_ok_and(|candidates| !candidates.is_empty())
        {
            points.push(candidate);
        }
    }
    points
}

enum Case {
    Matrix {
        label: &'static str,
        sources: Vec<GeoPoint>,
        targets: Vec<GeoPoint>,
        profile: MobilityProfile,
    },
    Isochrone {
        label: &'static str,
        origin: GeoPoint,
        profile: MobilityProfile,
        contours_s: Vec<f64>,
    },
}

impl Case {
    fn label(&self) -> &'static str {
        match self {
            Case::Matrix { label, .. } | Case::Isochrone { label, .. } => label,
        }
    }
}

fn cases(engine: &RoutingEngine) -> Vec<Case> {
    let krakow = krakow_points();
    let regional = regional_points();
    let only = std::env::var("ANALYSIS_ONLY").ok();
    let mut cases = Vec::new();
    // Large matrices, square: compact only (the tile-backed engine takes far
    // too long). Krakow keeps its 8 fixed points (Old Town included) first.
    for (label, seeds, area, count, profile) in [
        (
            "matrix_krakow_50x50_drive",
            &krakow,
            KRAKOW_BOX,
            50,
            MobilityProfile::Wheeled,
        ),
        (
            "matrix_krakow_100x100_drive",
            &krakow,
            KRAKOW_BOX,
            100,
            MobilityProfile::Wheeled,
        ),
        (
            "matrix_krakow_50x50_walk",
            &krakow,
            KRAKOW_BOX,
            50,
            MobilityProfile::OnFoot,
        ),
        (
            "matrix_krakow_100x100_walk",
            &krakow,
            KRAKOW_BOX,
            100,
            MobilityProfile::OnFoot,
        ),
        (
            "matrix_region_50x50_drive",
            &regional,
            REGION_BOX,
            50,
            MobilityProfile::Wheeled,
        ),
        (
            "matrix_region_100x100_drive",
            &regional,
            REGION_BOX,
            100,
            MobilityProfile::Wheeled,
        ),
    ] {
        if only.as_deref().is_some_and(|only| only != label) {
            continue;
        }
        let points = sampled_points(engine, seeds.clone(), area, count, profile);
        cases.push(Case::Matrix {
            label,
            sources: points.clone(),
            targets: points,
            profile,
        });
    }
    // Large matrices, run only when named by `ANALYSIS_ONLY` (time a few rows
    // with `ANALYSIS_SOURCES`): beyond the old 400-target vehicle cut-off.
    for (label, seeds, area, count, profile) in [
        (
            "matrix_krakow_1500x1500_drive",
            &krakow,
            KRAKOW_BOX,
            1500,
            MobilityProfile::Wheeled,
        ),
        (
            "matrix_region_1000x1000_drive",
            &regional,
            REGION_BOX,
            1000,
            MobilityProfile::Wheeled,
        ),
    ] {
        if only.as_deref() != Some(label) {
            continue;
        }
        // `ANALYSIS_POINTS=<n>` samples n points instead.
        let count = std::env::var("ANALYSIS_POINTS")
            .ok()
            .and_then(|value| value.parse().ok())
            .unwrap_or(count);
        let points = sampled_points(engine, seeds.clone(), area, count, profile);
        cases.push(Case::Matrix {
            label,
            sources: points.clone(),
            targets: points,
            profile,
        });
    }
    cases.extend([
        Case::Matrix {
            label: "matrix_krakow_8x8_drive",
            sources: krakow.clone(),
            targets: krakow.clone(),
            profile: MobilityProfile::Wheeled,
        },
        Case::Matrix {
            label: "matrix_krakow_8x8_walk",
            sources: krakow.clone(),
            targets: krakow,
            profile: MobilityProfile::OnFoot,
        },
        Case::Matrix {
            label: "matrix_region_7x7_drive",
            sources: regional.clone(),
            targets: regional,
            profile: MobilityProfile::Wheeled,
        },
        Case::Isochrone {
            label: "iso_krakow_drive_5_10_15min",
            origin: point(19.9448, 50.0647),
            profile: MobilityProfile::Wheeled,
            contours_s: vec![300.0, 600.0, 900.0],
        },
        Case::Isochrone {
            label: "iso_krakow_drive_30min",
            origin: point(19.9448, 50.0647),
            profile: MobilityProfile::Wheeled,
            contours_s: vec![1800.0],
        },
        Case::Isochrone {
            label: "iso_krakow_walk_15min",
            origin: point(19.9384, 50.0614),
            profile: MobilityProfile::OnFoot,
            contours_s: vec![900.0],
        },
        Case::Isochrone {
            label: "iso_zakopane_drive_20min",
            origin: point(19.9525, 49.2992),
            profile: MobilityProfile::Wheeled,
            contours_s: vec![1200.0],
        },
    ]);
    cases
}

// Short-lived, one per case; boxing the matrix would only add noise.
#[allow(clippy::large_enum_variant)]
enum Outcome {
    Matrix(RouteMatrixResult),
    Isochrone(Result<RouteIsochroneResult, String>),
}

/// `ANALYSIS_COSTING=<json>` applies a `RouteCostingOptions` to every request.
fn costing_options() -> wayfarer::RouteCostingOptions {
    std::env::var("ANALYSIS_COSTING")
        .ok()
        .map(|json| serde_json::from_str(&json).expect("ANALYSIS_COSTING is a costing JSON"))
        .unwrap_or_default()
}

/// The `ANALYSIS_OVERLAY` overlay: per plain route, a full block a third of
/// the way along and a partial block two thirds along with a bypass.
fn overlay_for(engine: &RoutingEngine) -> RouteDynamicOverlay {
    let mut ranges = Vec::new();
    let mut bypasses = Vec::new();
    for (origin, destination) in [
        (point(19.9448, 50.0647), point(20.0381, 50.0720)),
        (point(19.9448, 50.0647), point(20.9857, 50.0121)),
    ] {
        let mut request = LegRequest::new(origin, destination, MobilityProfile::Wheeled);
        request.options.max_correlation_distance_m = 150;
        let route = engine.route_leg(request).expect("overlay base route");
        let edges = &route.edge_ids;
        let (full, partial) = (edges[edges.len() / 3], edges[2 * edges.len() / 3]);
        ranges.push(RouteEdgeRange {
            edge_id: full,
            start_fraction: 0.0,
            end_fraction: 1.0,
        });
        ranges.push(RouteEdgeRange {
            edge_id: partial,
            start_fraction: 0.3,
            end_fraction: 0.6,
        });
        bypasses.push(partial);
    }
    let mut overlay = RouteDynamicOverlay::from_blocked_ranges(&ranges);
    for (index, partial) in bypasses.into_iter().enumerate() {
        overlay.virtual_edges.push(RouteDynamicVirtualEdge {
            edge_id: (1 << 63) | index as u64,
            connector_id: format!("bypass-{index}"),
            connector_type: RouteDynamicVirtualEdgeKind::RoadToRoadBypass,
            source_endpoint: None,
            source_point: None,
            source_edge_id: Some(partial),
            source_fraction: Some(0.3),
            target_endpoint: None,
            target_point: None,
            target_edge_id: Some(partial),
            target_fraction: Some(0.6),
            source_chunk_ids: Vec::new(),
            geometry: Vec::new(),
            distance_m: 30.0,
            cost: 5.0,
            provenance: "analysis_parity".into(),
        });
    }
    overlay.refresh_diagnostics();
    overlay
}

static OVERLAY: std::sync::OnceLock<Option<RouteDynamicOverlay>> = std::sync::OnceLock::new();

fn overlay() -> Option<RouteDynamicOverlay> {
    OVERLAY.get().cloned().flatten()
}

/// `ANALYSIS_DISCOUNT=<m>`: a discount `m` in (0, 1) on every edge of the two
/// `ANALYSIS_OVERLAY` base routes (cleared corridors), costed exactly with
/// bounds scaled by the least cost ratio.
static DISCOUNTS: std::sync::OnceLock<std::collections::BTreeMap<u64, f64>> =
    std::sync::OnceLock::new();

fn discounts_for(engine: &RoutingEngine, multiplier: f64) -> std::collections::BTreeMap<u64, f64> {
    let mut discounts = std::collections::BTreeMap::new();
    for (origin, destination) in [
        (point(19.9448, 50.0647), point(20.0381, 50.0720)),
        (point(19.9448, 50.0647), point(20.9857, 50.0121)),
    ] {
        let mut request = LegRequest::new(origin, destination, MobilityProfile::Wheeled);
        request.options.max_correlation_distance_m = 150;
        let route = engine.route_leg(request).expect("discount base route");
        discounts.extend(route.edge_ids.iter().map(|edge| (*edge, multiplier)));
    }
    discounts
}

/// `ANALYSIS_SAR=<RouteSarOptions JSON>`: SAR relaxations on every wheeled
/// request (SAR needs a vehicle profile).
fn sar_options() -> Option<wayfarer::RouteSarOptions> {
    std::env::var("ANALYSIS_SAR")
        .ok()
        .map(|json| serde_json::from_str(&json).expect("ANALYSIS_SAR is a SAR options JSON"))
}

/// `ANALYSIS_OFF_NETWORK=<RouteOffNetworkOptions JSON>` (e.g. `{}` for the
/// profile defaults, or `{"maxCandidates": 8}`): costed access legs on every
/// request (`wayfarer::access`).
fn off_network_options() -> Option<wayfarer::access::RouteOffNetworkOptions> {
    std::env::var("ANALYSIS_OFF_NETWORK").ok().map(|json| {
        serde_json::from_str(&json).expect("ANALYSIS_OFF_NETWORK is an off-network options JSON")
    })
}

/// The request options of every mode: the overlay, discounts, SAR and
/// costed access legs.
fn extend_options(options: &mut RouteOptions, profile: MobilityProfile) {
    options.dynamic_overlay = overlay();
    options.off_network = off_network_options();
    if let Some(discounts) = DISCOUNTS.get() {
        options.edge_cost_multipliers.extend(discounts);
    }
    if profile == MobilityProfile::Wheeled {
        options.sar = sar_options();
    }
}

/// Compact overlay matrix cells against package overlay routes.
fn check_overlay_cells(engine: &RoutingEngine, case: &Case, result: &RouteMatrixResult) {
    let Case::Matrix {
        sources,
        targets,
        profile,
        ..
    } = case
    else {
        return;
    };
    if sources.len() > 10 || targets.len() > 10 {
        return;
    }
    let (mut compared, mut problems) = (0, 0);
    for cell in &result.cells {
        let mut request = LegRequest::new(
            sources[cell.source_index],
            targets[cell.target_index],
            *profile,
        );
        request.options.max_correlation_distance_m = 150;
        extend_options(&mut request.options, *profile);
        let route = engine.route_leg(request).ok();
        match (cell.cost, route) {
            (Some(cost), Some(route)) => {
                compared += 1;
                if (cost - route.cost).abs() > 1e-6 * cost.max(1.0) {
                    problems += 1;
                    println!(
                        "  DIFF cell {}->{}: matrix {cost:.3} vs route {:.3}",
                        cell.source_index, cell.target_index, route.cost
                    );
                }
            }
            (None, None) => {}
            (cost, route) => {
                problems += 1;
                println!(
                    "  DIFF cell {}->{}: matrix {cost:?} vs route {:?}",
                    cell.source_index,
                    cell.target_index,
                    route.map(|route| route.cost)
                );
            }
        }
    }
    println!("  overlay route parity: {compared} cells compared, {problems} differ");
}

fn run(engine: &dyn Engine, case: &Case) -> Outcome {
    match case {
        Case::Matrix {
            sources,
            targets,
            profile,
            ..
        } => {
            // `ANALYSIS_SOURCES=<n>` keeps the first n sources (to time a few
            // rows of a slow matrix).
            let source_count = std::env::var("ANALYSIS_SOURCES")
                .ok()
                .and_then(|value| value.parse().ok())
                .unwrap_or(sources.len())
                .min(sources.len());
            // `ANALYSIS_TARGETS=<n>` likewise keeps the first n targets.
            let target_count = std::env::var("ANALYSIS_TARGETS")
                .ok()
                .and_then(|value| value.parse().ok())
                .unwrap_or(targets.len())
                .min(targets.len());
            // `ANALYSIS_SOURCE_LIST=<i,j,...>` keeps just those sources (to
            // check chosen rows of a large matrix against the reference).
            let sources = match std::env::var("ANALYSIS_SOURCE_LIST") {
                Ok(list) => list
                    .split(',')
                    .filter_map(|index| index.trim().parse::<usize>().ok())
                    .filter_map(|index| sources.get(index).copied())
                    .collect::<Vec<_>>(),
                Err(_) => sources[..source_count].to_vec(),
            };
            let mut request =
                RouteMatrixRequest::new(sources, targets[..target_count].to_vec(), *profile);
            request.options.max_correlation_distance_m = 150;
            request.options.costing = costing_options();
            extend_options(&mut request.options, *profile);
            Outcome::Matrix(engine.matrix(request))
        }
        Case::Isochrone {
            origin,
            profile,
            contours_s,
            ..
        } => {
            let mut request = RouteIsochroneRequest::new(
                *origin,
                *profile,
                contours_s
                    .iter()
                    .map(|contour| RouteIsochroneContour {
                        contour: *contour,
                        metric: RouteIsochroneMetric::TimeSeconds,
                    })
                    .collect(),
            );
            request.options.max_correlation_distance_m = 150;
            request.options.costing = costing_options();
            extend_options(&mut request.options, *profile);
            Outcome::Isochrone(engine.isochrone(request))
        }
    }
}

/// Process CPU time in seconds (all threads). Other processes competing for
/// cores inflate wall time but not this, so it is the steadier figure on a
/// busy machine.
#[cfg(unix)]
fn cpu_s() -> f64 {
    #[repr(C)]
    struct Timespec {
        tv_sec: i64,
        tv_nsec: i64,
    }
    unsafe extern "C" {
        fn clock_gettime(clock: i32, time: *mut Timespec) -> i32;
    }
    #[cfg(target_os = "macos")]
    const CLOCK_PROCESS_CPUTIME_ID: i32 = 12;
    #[cfg(not(target_os = "macos"))]
    const CLOCK_PROCESS_CPUTIME_ID: i32 = 2;
    let mut time = Timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `time` is a valid, writable timespec for the call's duration.
    if unsafe { clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &mut time) } != 0 {
        return f64::NAN;
    }
    time.tv_sec as f64 + time.tv_nsec as f64 * 1e-9
}

/// Process CPU time in seconds (all threads): kernel plus user time from
/// `GetProcessTimes`, in 100 ns units.
#[cfg(windows)]
fn cpu_s() -> f64 {
    use std::ffi::c_void;
    #[repr(C)]
    #[derive(Default)]
    struct FileTime {
        low: u32,
        high: u32,
    }
    #[link(name = "kernel32")]
    unsafe extern "system" {
        fn GetCurrentProcess() -> *mut c_void;
        fn GetProcessTimes(
            process: *mut c_void,
            creation: *mut FileTime,
            exit: *mut FileTime,
            kernel: *mut FileTime,
            user: *mut FileTime,
        ) -> i32;
    }
    let (mut creation, mut exit, mut kernel, mut user) = (
        FileTime::default(),
        FileTime::default(),
        FileTime::default(),
        FileTime::default(),
    );
    // SAFETY: the pseudo-handle needs no closing, and the four out-pointers
    // are valid, writable FILETIMEs for the call's duration.
    let ok = unsafe {
        GetProcessTimes(
            GetCurrentProcess(),
            &mut creation,
            &mut exit,
            &mut kernel,
            &mut user,
        )
    };
    if ok == 0 {
        return f64::NAN;
    }
    let ticks = |time: &FileTime| ((u64::from(time.high) << 32) | u64::from(time.low)) as f64;
    (ticks(&kernel) + ticks(&user)) * 1e-7
}

/// No process CPU clock on this platform: CPU times read as NaN.
#[cfg(not(any(unix, windows)))]
fn cpu_s() -> f64 {
    f64::NAN
}

/// The first outcome, then cold and median-warm wall times and the cold and
/// median-warm CPU times, all in ms.
struct Timed {
    outcome: Outcome,
    cold_ms: f64,
    warm_ms: f64,
    cold_cpu_ms: f64,
    warm_cpu_ms: f64,
}

fn timed(engine: &dyn Engine, case: &Case, repeats: usize) -> Timed {
    let (started, cpu) = (Instant::now(), cpu_s());
    let outcome = run(engine, case);
    let cold_ms = started.elapsed().as_secs_f64() * 1e3;
    let cold_cpu_ms = (cpu_s() - cpu) * 1e3;
    let mut warm = Vec::new();
    let mut warm_cpu = Vec::new();
    for _ in 1..repeats {
        let (started, cpu) = (Instant::now(), cpu_s());
        let _ = run(engine, case);
        warm.push(started.elapsed().as_secs_f64() * 1e3);
        warm_cpu.push((cpu_s() - cpu) * 1e3);
    }
    warm.sort_by(f64::total_cmp);
    warm_cpu.sort_by(f64::total_cmp);
    Timed {
        outcome,
        cold_ms,
        warm_ms: warm.get(warm.len() / 2).copied().unwrap_or(cold_ms),
        cold_cpu_ms,
        warm_cpu_ms: warm_cpu
            .get(warm_cpu.len() / 2)
            .copied()
            .unwrap_or(cold_cpu_ms),
    }
}

fn timing_line(engine: &str, timed: &Timed) -> String {
    format!(
        "  {engine:<9} cold {:>9.2} ms  warm {:>9.2} ms  cpu cold {:>9.1} ms  warm {:>9.1} ms  {}",
        timed.cold_ms,
        timed.warm_ms,
        timed.cold_cpu_ms,
        timed.warm_cpu_ms,
        summary(&timed.outcome)
    )
}

fn summary(outcome: &Outcome) -> String {
    match outcome {
        Outcome::Matrix(result) => {
            let reachable = result
                .cells
                .iter()
                .filter(|cell| cell.status == RouteMatrixCellStatus::Reachable)
                .count();
            format!(
                "{reachable}/{} reachable, visited {}",
                result.cells.len(),
                result.visited_edge_count
            )
        }
        Outcome::Isochrone(Ok(result)) => format!(
            "edges {:?}, fragments {:?}, visited {}",
            result
                .buckets
                .iter()
                .map(|bucket| bucket.edge_ids.len())
                .collect::<Vec<_>>(),
            result
                .buckets
                .iter()
                .map(|bucket| bucket.boundary_fragments.len())
                .collect::<Vec<_>>(),
            result.visited_edge_count
        ),
        Outcome::Isochrone(Err(error)) => format!("error {error}"),
    }
}

/// Differences between two outcomes, as human-readable lines.
fn compare(reference: &Outcome, compact: &Outcome) -> Vec<String> {
    let mut problems = Vec::new();
    match (reference, compact) {
        (Outcome::Matrix(left), Outcome::Matrix(right)) => {
            for (a, b) in left.cells.iter().zip(&right.cells) {
                let cell = format!("cell {}->{}", a.source_index, a.target_index);
                if a.status != b.status {
                    problems.push(format!("{cell}: status {:?} vs {:?}", a.status, b.status));
                    continue;
                }
                if let (Some(x), Some(y)) = (a.cost, b.cost) {
                    let delta = (y - x) / x.max(1e-9);
                    if delta.abs() > 1e-6 {
                        problems.push(format!(
                            "{cell}: cost {x:.3} vs {y:.3} ({:+.4}%){}",
                            delta * 100.0,
                            if a.edge_ids == b.edge_ids {
                                " same path"
                            } else {
                                ""
                            }
                        ));
                    }
                }
            }
        }
        (Outcome::Isochrone(Ok(left)), Outcome::Isochrone(Ok(right))) => {
            for (a, b) in left.buckets.iter().zip(&right.buckets) {
                let left_edges = a.edge_ids.iter().copied().collect::<BTreeSet<_>>();
                let right_edges = b.edge_ids.iter().copied().collect::<BTreeSet<_>>();
                let only_left = left_edges.difference(&right_edges).count();
                let only_right = right_edges.difference(&left_edges).count();
                if only_left + only_right > 0 {
                    problems.push(format!(
                        "contour {}: {} edges only reference, {} only compact (of {}/{})",
                        a.contour,
                        only_left,
                        only_right,
                        left_edges.len(),
                        right_edges.len()
                    ));
                }
                let fragment_key = |bucket: &wayfarer::RouteIsochroneBucket| {
                    bucket
                        .boundary_fragments
                        .iter()
                        .map(|fragment| {
                            (
                                fragment.edge_id,
                                (fragment.start_fraction * 1e6).round() as i64,
                                (fragment.end_fraction * 1e6).round() as i64,
                            )
                        })
                        .collect::<BTreeSet<_>>()
                };
                let (left_fragments, right_fragments) = (fragment_key(a), fragment_key(b));
                // Paired in sorted order: how far apart the matching ends are.
                if a.boundary_fragments.len() == b.boundary_fragments.len() {
                    let (mut max_fraction, mut max_m) = (0.0f64, 0.0f64);
                    for (x, y) in a.boundary_fragments.iter().zip(&b.boundary_fragments) {
                        if x.edge_id == y.edge_id {
                            max_fraction =
                                max_fraction.max((x.end_fraction - y.end_fraction).abs());
                            max_m = max_m.max(x.end.distance_to(y.end));
                        }
                    }
                    problems.push(format!(
                        "contour {}: fragment ends differ by at most {max_fraction:.2e} of an \
                         edge, {max_m:.3} m",
                        a.contour
                    ));
                }
                if left_fragments != right_fragments {
                    problems.push(format!(
                        "contour {}: boundary fragments differ ({} only reference, {} only compact)",
                        a.contour,
                        left_fragments.difference(&right_fragments).count(),
                        right_fragments.difference(&left_fragments).count()
                    ));
                }
            }
        }
        (Outcome::Isochrone(left), Outcome::Isochrone(right)) => {
            if left.is_ok() != right.is_ok() {
                problems.push(format!(
                    "ok/err mismatch: {:?} vs {:?}",
                    left.as_ref().err(),
                    right.as_ref().err()
                ));
            }
        }
        _ => problems.push("outcome kinds differ".to_string()),
    }
    problems
}

/// Writes `outcome` as JSON to `$ANALYSIS_DUMP/<label>.<engine>.json`.
fn dump(label: &str, engine: &str, outcome: &Outcome) {
    let Some(dir) = std::env::var_os("ANALYSIS_DUMP") else {
        return;
    };
    let json = match outcome {
        Outcome::Matrix(result) => serde_json::to_string_pretty(result),
        Outcome::Isochrone(Ok(result)) => serde_json::to_string_pretty(result),
        Outcome::Isochrone(Err(error)) => serde_json::to_string_pretty(error),
    }
    .expect("serialize outcome");
    std::fs::create_dir_all(&dir).expect("create dump directory");
    let path = std::path::Path::new(&dir).join(format!("{label}.{engine}.json"));
    std::fs::write(&path, json).expect("write dump");
}

/// Routes the cells listed in `cells` (`source:target,...`) of a matrix case
/// through the package engine and the reference.
fn check_cells(
    compact: &RoutingEngine,
    reference: Option<&ReferenceEngine<'_>>,
    case: &Case,
    cells: &str,
) {
    let Case::Matrix {
        label,
        sources,
        targets,
        profile,
    } = case
    else {
        return;
    };
    println!("== {label}");
    for cell in cells.split(',') {
        let Some((source, target)) = cell.split_once(':') else {
            continue;
        };
        let (Ok(source), Ok(target)) = (source.parse::<usize>(), target.parse::<usize>()) else {
            continue;
        };
        let request = || {
            let mut request = LegRequest::new(sources[source], targets[target], *profile);
            request.options.max_correlation_distance_m = 150;
            request.options.costing = costing_options();
            extend_options(&mut request.options, *profile);
            request
        };
        let describe = |result: Result<wayfarer::LegResult, _>| match result {
            Ok(route) => format!(
                "cost {:.4} edges {} first {:?} last {:?}",
                route.cost,
                route.edge_ids.len(),
                route.edge_ids.first(),
                route.edge_ids.last()
            ),
            Err(error) => format!("error {error:?}"),
        };
        println!(
            "  cell {source}->{target}: {:?} -> {:?}",
            sources[source], targets[target]
        );
        println!(
            "    compact bidirectional  {}",
            describe(compact.route_leg(request()))
        );
        // SAFETY: single-threaded here; the variable is read per route.
        unsafe { std::env::set_var("WAYFARER_UNIDIRECTIONAL", "1") };
        println!(
            "    compact unidirectional {}",
            describe(compact.route_leg(request()))
        );
        unsafe { std::env::remove_var("WAYFARER_UNIDIRECTIONAL") };
        if let Some(reference) = reference {
            println!(
                "    reference              {}",
                describe(
                    reference.route_leg(request()).map_err(|error| {
                        wayfarer::RouteFailure::new(error, Default::default())
                    })
                )
            );
        }
    }
}

/// The reference oracle over a road graph snapshot
/// (`wayfarer_road_graph::persist_road_graph_file`), with the routing package
/// built from it in memory.
fn load_reference(path: &std::path::Path) -> ReferenceEngine<'static> {
    let graph = Arc::new(
        wayfarer_road_graph::load_road_graph_file(path).expect("load reference road graph"),
    );
    let (bytes, _) = wayfarer_compiler::package_build::package_bytes(
        &graph,
        // Format 2 keeps the graph's ids, which the oracle reads the package
        // by; format 3 would renumber the package's copy by CCH rank.
        &wayfarer_compiler::package_build::PackageBuildOptions {
            package_format: wayfarer_package::format::PackageFormat::V2,
            ..Default::default()
        },
    )
    .expect("the reference road graph builds a routing package");
    let package = Arc::new(RoutingPackage::from_bytes(&bytes).expect("open the built package"));
    ReferenceEngine::from_owned_graph(graph, package)
}

fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    let archive = args.get(1).expect("archive path");
    let repeats = args.get(2).and_then(|v| v.parse().ok()).unwrap_or(3usize);
    let graph =
        Arc::new(RoutingPackage::open(std::path::Path::new(archive)).expect("open package"));
    // `ANALYSIS_REFERENCE_CONTEXT=<k>` keeps one reference label per edge and
    // last k traversed edges, exact under complex restrictions of up to k
    // prefix edges (see `ReferenceEngine::with_path_context`).
    let reference_context = std::env::var("ANALYSIS_REFERENCE_CONTEXT")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(0);
    let reference = std::env::var_os("REFERENCE_ROAD_GRAPH").map(|path| {
        load_reference(std::path::Path::new(&path)).with_path_context(reference_context)
    });
    let compact = RoutingEngine::from_package(graph);
    let only = std::env::var("ANALYSIS_ONLY").ok();
    let package_only = std::env::var_os("ANALYSIS_PACKAGE_ONLY").is_some();
    let reference_only = std::env::var_os("ANALYSIS_REFERENCE_ONLY").is_some();
    let overlay_mode = std::env::var_os("ANALYSIS_OVERLAY").is_some();
    OVERLAY
        .set(overlay_mode.then(|| overlay_for(&compact)))
        .expect("overlay set once");
    if let Some(overlay) = overlay() {
        println!(
            "overlay: {} split edges, {} connectors",
            overlay.split_edge_ranges.len(),
            overlay.virtual_edges.len()
        );
    }
    if let Some(multiplier) = std::env::var("ANALYSIS_DISCOUNT")
        .ok()
        .and_then(|value| value.parse::<f64>().ok())
    {
        let discounts = discounts_for(&compact, multiplier);
        println!("discount: {multiplier} on {} edges", discounts.len());
        DISCOUNTS.set(discounts).expect("discounts set once");
    }
    if let Some(sar) = sar_options() {
        println!("sar: {:?}", sar.enabled());
    }

    for case in cases(&compact) {
        if only.as_deref().is_some_and(|only| only != case.label()) {
            continue;
        }
        if let Ok(cells) = std::env::var("ANALYSIS_CHECK_CELLS") {
            check_cells(&compact, reference.as_ref(), &case, &cells);
            continue;
        }
        println!("== {}", case.label());
        let package_run = (!reference_only).then(|| timed(&compact, &case, repeats));
        if let Some(package_run) = &package_run {
            println!("{}", timing_line("package", package_run));
            dump(case.label(), "package", &package_run.outcome);
        }
        if std::env::var_os("ANALYSIS_CELLS").is_some()
            && let Some(Timed {
                outcome: Outcome::Matrix(result),
                ..
            }) = &package_run
        {
            for cell in &result.cells {
                println!(
                    "    {}->{} cost {:>8.1} dur {:>7.1} km {:>6.2} edges {}",
                    cell.source_index,
                    cell.target_index,
                    cell.cost.unwrap_or(f64::NAN),
                    cell.duration_s.unwrap_or(f64::NAN),
                    cell.distance_m.unwrap_or(f64::NAN) / 1000.0,
                    cell.edge_ids.len()
                );
            }
        }
        if overlay_mode
            && let Some(Timed {
                outcome: Outcome::Matrix(result),
                ..
            }) = &package_run
        {
            check_overlay_cells(&compact, &case, result);
        }
        let Some(reference) = reference.as_ref().filter(|_| !package_only) else {
            continue;
        };
        if overlay_mode && matches!(case, Case::Isochrone { .. }) {
            println!("  reference: overlay isochrones are not modelled");
            continue;
        }
        let reference_repeats = std::env::var("ANALYSIS_REFERENCE_REPEATS")
            .ok()
            .and_then(|value| value.parse().ok())
            .unwrap_or(repeats);
        let reference_run = timed(reference, &case, reference_repeats);
        println!("{}", timing_line("reference", &reference_run));
        dump(case.label(), "reference", &reference_run.outcome);
        let Some(package_run) = &package_run else {
            continue;
        };
        let problems = compare(&reference_run.outcome, &package_run.outcome);
        if problems.is_empty() {
            println!("  parity: identical");
        } else {
            for problem in problems.iter().take(12) {
                println!("  DIFF {problem}");
            }
            if problems.len() > 12 {
                println!("  ... {} more", problems.len() - 12);
            }
        }
    }
}

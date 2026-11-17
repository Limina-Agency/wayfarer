//! Heap use of one route, an 8×8 matrix and an isochrone on a routing
//! package, under a memory profile, measured with a counting allocator.
//!
//! ```sh
//! cargo run --release -p wayfarer --example package_memory -- \
//!   <routing.mrt> [desktop|mobile]
//! ```
//!
//! For each request: the peak live heap above the heap before it, and the
//! heap the package keeps afterwards (pooled search states, derived
//! columns). The package itself is memory-mapped, not heap; its resident
//! pages are reported separately as the process RSS (`ps`), which also
//! counts the mapped file pages a request touched. None of these requests
//! names an edge id, so none builds the package's edge id lookup.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

use wayfarer::{
    GeoPoint, LegRequest, MobilityProfile, RouteCostingOptions, RouteIsochroneContour,
    RouteIsochroneMetric, RouteIsochroneRequest, RouteMatrixRequest, RoutePackageMemoryProfile,
    RoutingEngine, RoutingPackage,
};

static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);

struct Counting;

// SAFETY: forwards to the system allocator and only counts bytes.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let pointer = unsafe { System.alloc(layout) };
        if !pointer.is_null() {
            let live = LIVE.fetch_add(layout.size(), Ordering::Relaxed) + layout.size();
            PEAK.fetch_max(live, Ordering::Relaxed);
        }
        pointer
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        unsafe { System.dealloc(pointer, layout) };
        LIVE.fetch_sub(layout.size(), Ordering::Relaxed);
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let moved = unsafe { System.realloc(pointer, layout, new_size) };
        if !moved.is_null() {
            if new_size > layout.size() {
                let live = LIVE.fetch_add(new_size - layout.size(), Ordering::Relaxed)
                    + (new_size - layout.size());
                PEAK.fetch_max(live, Ordering::Relaxed);
            } else {
                LIVE.fetch_sub(layout.size() - new_size, Ordering::Relaxed);
            }
        }
        moved
    }
}

#[global_allocator]
static ALLOCATOR: Counting = Counting;

fn mib(bytes: usize) -> f64 {
    bytes as f64 / (1024.0 * 1024.0)
}

/// Resident set size of this process in MiB, from `ps`.
fn rss_mib() -> f64 {
    std::process::Command::new("ps")
        .args(["-o", "rss=", "-p", &std::process::id().to_string()])
        .output()
        .ok()
        .and_then(|output| String::from_utf8(output.stdout).ok())
        .and_then(|text| text.trim().parse::<f64>().ok())
        .map_or(f64::NAN, |kib| kib / 1024.0)
}

/// Runs `work`, counting its result in the peak but not in what is kept.
fn measure<T>(label: &str, work: impl FnOnce() -> T) {
    let before = LIVE.load(Ordering::Relaxed);
    PEAK.store(before, Ordering::Relaxed);
    let started = Instant::now();
    drop(work());
    let ms = started.elapsed().as_secs_f64() * 1e3;
    let peak = PEAK.load(Ordering::Relaxed);
    let after = LIVE.load(Ordering::Relaxed);
    println!(
        "{label:<34} peak +{:>7.1} MiB  kept +{:>7.1} MiB  heap {:>7.1} MiB  rss {:>7.1} MiB  {ms:>8.1} ms",
        mib(peak - before),
        mib(after.saturating_sub(before)),
        mib(after),
        rss_mib(),
    );
}

fn point(lng: f64, lat: f64) -> GeoPoint {
    GeoPoint::new(lat, lng)
}

fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    let path = args.get(1).expect("routing package path");
    let profile = match args.get(2).map(String::as_str) {
        Some("mobile") => RoutePackageMemoryProfile::Mobile,
        _ => RoutePackageMemoryProfile::Desktop,
    };
    println!(
        "profile {profile:?}: {} pooled search states, {} derived columns, matrix threads {:?}",
        profile.pooled_search_states(),
        profile.derived_column_capacity(),
        profile.matrix_threads()
    );
    let started = Instant::now();
    let package = RoutingPackage::open_with_profile(std::path::Path::new(path), profile)
        .expect("open package");
    println!(
        "open: {:.1} ms, heap {:.1} MiB, rss {:.1} MiB",
        started.elapsed().as_secs_f64() * 1e3,
        mib(LIVE.load(Ordering::Relaxed)),
        rss_mib()
    );
    let package = Arc::new(package);
    let engine = RoutingEngine::from_package(package.clone());
    let route = |costing: RouteCostingOptions| {
        let mut request = LegRequest::new(
            point(19.9448, 50.0647),
            point(20.0381, 50.0720),
            MobilityProfile::Wheeled,
        );
        request.options.max_correlation_distance_m = 150;
        request.options.max_visited_edge_count = Some(2_000_000);
        request.options.costing = costing;
        engine.route_leg(request).expect("route")
    };
    measure("route, Krakow drive (first)", || route(Default::default()));
    measure("route, Krakow drive (again)", || route(Default::default()));
    let regional = || {
        let mut request = LegRequest::new(
            point(19.9448, 50.0647),
            point(20.6905, 49.6249),
            MobilityProfile::Wheeled,
        );
        request.options.max_correlation_distance_m = 150;
        request.options.max_visited_edge_count = Some(2_000_000);
        engine.route_leg(request).expect("regional route")
    };
    measure("route, Krakow-Nowy Sacz drive", regional);
    let avoid = RouteCostingOptions {
        avoid_motorways: 0.5,
        ..Default::default()
    };
    measure("route, derived column (first)", || route(avoid));
    let krakow = vec![
        point(19.9448, 50.0647),
        point(20.0381, 50.0720),
        point(19.9384, 50.0614),
        point(19.9100, 50.0300),
        point(19.9900, 50.0950),
        point(20.0100, 50.0400),
        point(19.8900, 50.0800),
        point(19.9600, 50.0100),
    ];
    let matrix = || {
        let mut request =
            RouteMatrixRequest::new(krakow.clone(), krakow.clone(), MobilityProfile::Wheeled);
        request.options.max_correlation_distance_m = 150;
        engine.matrix(request)
    };
    measure("matrix 8x8, Krakow drive", || {
        let result = matrix();
        assert_eq!(result.cells.len(), 64);
        result
    });
    let isochrone = |minutes: f64| {
        let mut request = RouteIsochroneRequest::new(
            point(19.9448, 50.0647),
            MobilityProfile::Wheeled,
            vec![RouteIsochroneContour {
                contour: minutes * 60.0,
                metric: RouteIsochroneMetric::TimeSeconds,
            }],
        );
        request.options.max_correlation_distance_m = 150;
        engine.isochrone(request).expect("isochrone")
    };
    measure("isochrone, Krakow drive 15 min", || isochrone(15.0));
    measure("isochrone, Krakow drive 30 min", || isochrone(30.0));
    measure("route, Krakow drive (after all)", || {
        route(Default::default())
    });
    // Built only for requests that name an edge id.
    println!(
        "edge id lookup built: {}",
        package.file().edge_id_lookup_built()
    );
}

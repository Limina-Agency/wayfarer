//! Times building the CCH potentials of a routing package: the topology and
//! every stored column's customization, on fresh opens.
//!
//! ```sh
//! cargo run --release -p wayfarer --example potentials_bench -- \
//!   <routing.mrt> [runs=5] [desktop|mobile]
//! ```
//!
//! `RAYON_NUM_THREADS` sets the threads. Prints each run's stats (the heap
//! the potentials keep afterwards: on the Mobile profile without the arrays
//! only customization reads) and the minimum topology and customization
//! times.

use std::path::Path;

use wayfarer::{RoutePackageMemoryProfile, RoutingPackage};

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .expect("usage: potentials_bench <routing.mrt> [runs]");
    let runs = args.next().and_then(|runs| runs.parse().ok()).unwrap_or(5);
    let profile = match args.next().as_deref() {
        Some("mobile") => RoutePackageMemoryProfile::Mobile,
        _ => RoutePackageMemoryProfile::Desktop,
    };
    let (mut topology_ms, mut customize_ms) = (f64::INFINITY, f64::INFINITY);
    for run in 0..runs {
        let package = RoutingPackage::open_with_profile(Path::new(&path), profile)
            .expect("a routing package");
        let started = std::time::Instant::now();
        let stats = package
            .prepare_potentials()
            .expect("a package with a CCH order");
        let wall_ms = started.elapsed().as_secs_f64() * 1e3;
        println!(
            "run {run}: nodes {} arcs {} height {} columns {} topology {:.1} ms \
             customize {:.1} ms wall {wall_ms:.1} ms heap {:.1} MiB",
            stats.node_count,
            stats.arc_count,
            stats.elimination_tree_height,
            stats.column_count,
            stats.topology_ms,
            stats.customize_ms,
            stats.heap_bytes as f64 / (1 << 20) as f64,
        );
        topology_ms = topology_ms.min(stats.topology_ms);
        customize_ms = customize_ms.min(stats.customize_ms);
    }
    println!("min: topology {topology_ms:.1} ms customize {customize_ms:.1} ms");
}

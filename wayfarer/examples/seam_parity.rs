//! Parity of the seam prototype (`wayfarer::regions`, feature
//! `experimental-regions`) with the package search on a real package: the
//! package is cut into two regions at the top separator of its node order,
//! and random pairs are routed three ways on the same query:
//!
//! - `seam`: the forward A* guided by the regions' own CCHs and the portal
//!   overlay (what a bundle of two regions would hold);
//! - `bounded`: the same A* guided by the package's CCH;
//! - `package`: the package's own search (bidirectional, as a plain route).
//!
//! ```sh
//! cargo run --release -p wayfarer --features experimental-regions \
//!   --example seam_parity -- <routing.mrt> [pairs=200] [seed=1]
//! ```
//!
//! Points are random package nodes' positions, snapped within 150 m, the
//! profile alternating wheeled and on foot. Costs are compared with the
//! approach costs the candidates were chosen by, within the package
//! search's tie window. `SEAM_PARITY_DUMP=<file>` writes one JSON line per
//! case. Prints a JSON summary; exits 1 on a cost mismatch.

use std::io::Write;
use std::sync::Arc;
use std::time::Instant;

use wayfarer::regions::{SeamBound, SeamPath, SeamRouter};
use wayfarer::{GeoPoint, MobilityProfile, RoutingEngine, RoutingPackage, SnapRequest};

struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.0 >> 33
    }
}

#[derive(Default)]
struct Totals {
    cases: usize,
    routed: usize,
    no_path: usize,
    cost_mismatches: usize,
    edges_identical_package: usize,
    edges_identical_bounded: usize,
    crossing: usize,
    crossings: usize,
    settled: [usize; 3],
    ms: [f64; 3],
    overlay_us: f64,
}

fn ms(started: Instant) -> f64 {
    started.elapsed().as_secs_f64() * 1e3
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    let path = args
        .first()
        .ok_or("usage: seam_parity <routing.mrt> [pairs] [seed]")?;
    let pairs = args.get(1).map_or(Ok(200), |value| value.parse())?;
    let seed = args.get(2).map_or(Ok(1), |value| value.parse())?;
    let mut dump = std::env::var_os("SEAM_PARITY_DUMP")
        .map(std::fs::File::create)
        .transpose()?
        .map(std::io::BufWriter::new);

    let started = Instant::now();
    let package = Arc::new(RoutingPackage::open(path.as_ref())?);
    let open_ms = ms(started);
    let started = Instant::now();
    let potentials = package.prepare_potentials();
    let potentials_ms = ms(started);
    let started = Instant::now();
    let router = SeamRouter::new(package.clone())?;
    let cut_ms = ms(started);
    let summary = router.summary().clone();
    let mut overlays = Vec::new();
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        overlays.push((profile, router.overlay(profile)?));
    }
    let engine = RoutingEngine::from_package(package.clone());

    let nodes = package.counts().node_count as u64;
    let mut random = Lcg(seed);
    let mut totals = Totals::default();
    for case in 0..pairs {
        let profile = if case % 2 == 0 {
            MobilityProfile::Wheeled
        } else {
            MobilityProfile::OnFoot
        };
        let mut point = || router.node_point((random.next() % nodes) as u32);
        let (from, to) = (point(), point());
        let snap = |point: GeoPoint| {
            let mut request = SnapRequest::new(point, profile);
            request.max_distance_m = Some(150);
            engine.snap(request).candidates
        };
        let (origins, destinations) = (snap(from), snap(to));
        if origins.is_empty() || destinations.is_empty() {
            continue;
        }
        totals.cases += 1;
        let timed = |run: &dyn Fn() -> Option<SeamPath>| {
            let started = Instant::now();
            let path = run();
            (path, ms(started))
        };
        let (seam, seam_ms) = timed(&|| {
            router
                .route(profile, &origins, &destinations, SeamBound::Regions)
                .expect("a seam query")
        });
        let (bounded, bounded_ms) = timed(&|| {
            router
                .route(profile, &origins, &destinations, SeamBound::Package)
                .expect("a bounded query")
        });
        let (package_path, package_ms) = timed(&|| {
            router
                .package_route(profile, &origins, &destinations)
                .expect("a package query")
        });
        let status = match (&seam, &bounded, &package_path) {
            (None, None, None) => {
                totals.no_path += 1;
                "no_path"
            }
            (Some(seam), Some(bounded), Some(package_path)) => {
                totals.routed += 1;
                let window = 1e-7 * package_path.total_cost.abs().max(1.0);
                let equal = (seam.total_cost - package_path.total_cost).abs() <= window
                    && (bounded.total_cost - package_path.total_cost).abs() <= window;
                totals.cost_mismatches += usize::from(!equal);
                totals.edges_identical_package +=
                    usize::from(seam.edge_ids == package_path.edge_ids);
                totals.edges_identical_bounded += usize::from(seam.edge_ids == bounded.edge_ids);
                totals.crossing += usize::from(seam.portal_crossings > 0);
                totals.crossings += seam.portal_crossings;
                totals.settled[0] += seam.settled_edge_count;
                totals.settled[1] += bounded.settled_edge_count;
                totals.settled[2] += package_path.settled_edge_count;
                totals.ms[0] += seam_ms;
                totals.ms[1] += bounded_ms;
                totals.ms[2] += package_ms;
                totals.overlay_us += seam.overlay.as_ref().map_or(0.0, |o| o.query_us);
                if equal { "equal" } else { "cost_mismatch" }
            }
            _ => {
                totals.cost_mismatches += 1;
                "one_side_no_path"
            }
        };
        if let Some(dump) = dump.as_mut() {
            let path_json = |path: &Option<SeamPath>, ms: f64| match path {
                Some(path) => serde_json::json!({
                    "total_cost": path.total_cost,
                    "cost": path.cost,
                    "edges": path.edge_ids.len(),
                    "settled": path.settled_edge_count,
                    "portal_crossings": path.portal_crossings,
                    "ms": ms,
                }),
                None => serde_json::Value::Null,
            };
            writeln!(
                dump,
                "{}",
                serde_json::json!({
                    "case": case,
                    "profile": format!("{profile:?}"),
                    "from": [from.lng, from.lat],
                    "to": [to.lng, to.lat],
                    "status": status,
                    "seam": path_json(&seam, seam_ms),
                    "bounded": path_json(&bounded, bounded_ms),
                    "package": path_json(&package_path, package_ms),
                    "seam_edges_equal_package": match (&seam, &package_path) {
                        (Some(a), Some(b)) => a.edge_ids == b.edge_ids,
                        _ => true,
                    },
                })
            )?;
        }
    }
    let routed = totals.routed.max(1) as f64;
    let report = serde_json::json!({
        "package": path,
        "nodes": nodes,
        "edges": package.counts().edge_count,
        "open_ms": open_ms,
        "potentials_ms": potentials_ms,
        "potential_arcs": potentials.as_ref().map(|stats| stats.arc_count),
        "cut_ms": cut_ms,
        "seam": {
            "node_count": summary.node_count,
            "edge_count": summary.edge_count,
            "separator_node_count": summary.separator_node_count,
            "portal_edge_count": summary.portal_edge_count,
            "component_count": summary.component_count,
            "subtree_count": summary.subtree_count,
        },
        "overlays": overlays.iter().map(|(profile, stats)| serde_json::json!({
            "profile": format!("{profile:?}"),
            "vertex_count": stats.vertex_count,
            "portal_arc_count": stats.portal_arc_count,
            "clique_arc_count": stats.clique_arc_count,
            "build_ms": stats.build_ms,
        })).collect::<Vec<_>>(),
        "cases": totals.cases,
        "routed": totals.routed,
        "no_path": totals.no_path,
        "cost_mismatches": totals.cost_mismatches,
        "edges_identical_to_package": totals.edges_identical_package,
        "edges_identical_to_bounded": totals.edges_identical_bounded,
        "routes_crossing_the_seam": totals.crossing,
        "portal_crossings": totals.crossings,
        "mean_settled": {
            "seam": totals.settled[0] as f64 / routed,
            "bounded": totals.settled[1] as f64 / routed,
            "package": totals.settled[2] as f64 / routed,
        },
        "mean_ms": {
            "seam": totals.ms[0] / routed,
            "bounded": totals.ms[1] / routed,
            "package": totals.ms[2] / routed,
            "seam_overlay_query": totals.overlay_us / routed / 1e3,
        },
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    if totals.cost_mismatches > 0 {
        std::process::exit(1);
    }
    Ok(())
}

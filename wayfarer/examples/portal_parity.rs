//! Portals on a real surface (`docs/design/portals.md` section 5): routes and
//! matrices from open ground on a terrain file's area, the package engine
//! against the reference oracle, both with the same `TerrainPricer`.
//!
//! ```sh
//! cargo run --release -p wayfarer --features terrain,reference \
//!   --example portal_parity -- <extract.osm.pbf> <terrain.wtr> <work dir> [pairs=60]
//! ```
//!
//! The first run compiles the extract into `<work dir>/routing.mrt` and writes
//! the road graph the package is built from to `<work dir>/road-graph.mrg`
//! (the oracle's graph); later runs reuse the snapshot. The incident is the
//! terrain with no layers, for a walker and the 4×4, at a fixed time.
//!
//! Each pair is two posts the walker can stand on, drawn with a fixed seed;
//! an endpoint farther than the snap radius from every road is priced by the
//! terrain. For walkers and cars it checks that engine and oracle agree on
//! cost, duration and distance, that the totals are the road's plus the
//! stretches, and, where an endpoint cut candidates, that a run with every
//! candidate is never dearer: it reports how often the cut cost a cheaper
//! route, against the gap between the best cut and the worst kept key.
//! Then an 8 × 8 matrix per profile against the oracle. It prints one JSON
//! summary.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Instant;

use wayfarer::access::{AccessPricer, AccessPricerHandle, AccessQuery, RouteOffNetworkOptions};
use wayfarer::portals::TerrainPricer;
use wayfarer::reference::ReferenceEngine;
use wayfarer::{
    CorrelatedLocation, GeoPoint, LegRequest, LegResult, MobilityProfile, RouteMatrixRequest,
    RoutingEngine, RoutingPackage,
};
use wayfarer_terrain::incident::{Incident, Settings};
use wayfarer_terrain::lattice::TICKS_PER_DEG;
use wayfarer_terrain::profile::Rules;
use wayfarer_terrain::search::Grid;
use wayfarer_terrain::terrain::Terrain;

const T0: i64 = 1_795_510_800; // 2026-11-24 09:00 UTC

/// A pricer that times the one it wraps.
struct Timed {
    inner: TerrainPricer,
    calls: std::sync::Mutex<Vec<f64>>,
}

impl AccessPricer for Timed {
    fn price(&self, query: AccessQuery<'_>) -> Vec<CorrelatedLocation> {
        let start = Instant::now();
        let out = self.inner.price(query);
        self.calls
            .lock()
            .unwrap()
            .push(start.elapsed().as_secs_f64() * 1e3);
        out
    }
}

fn compile(pbf: &Path, dir: &Path) -> PathBuf {
    let snapshot = dir.join("road-graph.mrg");
    if snapshot.is_file() {
        return snapshot;
    }
    std::fs::create_dir_all(dir).expect("work dir");
    let mut request = wayfarer_compiler::CompileRequest::new(pbf, dir.join("routing.mrt"))
        .scratch_dir(dir.join("scratch"));
    request.road_graph_snapshot = Some(snapshot.clone());
    let start = Instant::now();
    wayfarer_compiler::compile(&request, &wayfarer::CancelToken::new(), |_| {})
        .expect("the extract compiles");
    eprintln!("compiled in {:.0} s", start.elapsed().as_secs_f64());
    snapshot
}

/// A deterministic mixer.
fn mix(x: u64) -> u64 {
    let mut z = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// Posts the walker can stand on, away from the grid's edge, as points.
fn sample(grid: &Grid, walker: usize, n: usize, seed: u64) -> Vec<GeoPoint> {
    let s = grid.spacing as f64 / TICKS_PER_DEG as f64;
    let margin = 64;
    let mut out = Vec::new();
    let mut k = seed;
    while out.len() < n {
        k += 1;
        let r = margin + (mix(k) % (grid.h - 2 * margin) as u64) as usize;
        let c = margin + (mix(k ^ 0xABCD) % (grid.w - 2 * margin) as u64) as usize;
        let cell = r * grid.w + c;
        if grid.t[walker][cell] == 0 {
            continue;
        }
        let (i, j) = grid.post_of(cell);
        out.push(GeoPoint::new(i as f64 * s, j as f64 * s));
    }
    out
}

fn request(
    origin: GeoPoint,
    destination: GeoPoint,
    profile: MobilityProfile,
    pricer: &AccessPricerHandle,
    max_candidates: Option<usize>,
) -> LegRequest {
    let mut request = LegRequest::new(origin, destination, profile);
    let mode = std::env::var("PORTAL_PARITY_PRICER").unwrap_or_default();
    if mode == "none" {
        return request;
    }
    let off = mode == "off";
    request.options.off_network = Some(RouteOffNetworkOptions {
        pricer: (!off).then(|| pricer.clone()),
        max_candidates,
        ..Default::default()
    });
    request
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-4 * a.abs().max(1.0)
}

#[derive(Default)]
struct Tally {
    pairs: usize,
    routed: usize,
    both_failed: usize,
    mismatched: Vec<String>,
    totals_off: usize,
    priced_ends: usize,
    straight_ends: usize,
    truncated_ends: usize,
    cut_cost_route: usize,
    cut_gaps_s: Vec<f64>,
    saved_s: Vec<f64>,
    stretch_s: Vec<f64>,
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let pbf = PathBuf::from(args.get(1).expect("extract.osm.pbf"));
    let terrain_path = PathBuf::from(args.get(2).expect("terrain.wtr"));
    let dir = PathBuf::from(args.get(3).expect("work dir"));
    let pairs: usize = args.get(4).and_then(|v| v.parse().ok()).unwrap_or(60);

    let snapshot = compile(&pbf, &dir);
    let start = Instant::now();
    let graph = Arc::new(wayfarer_road_graph::load_road_graph_file(&snapshot).expect("snapshot"));
    let (bytes, _) = wayfarer_compiler::package_build::package_bytes(
        &graph,
        // Format 2 keeps the graph's ids, which the oracle reads the package
        // by; format 3 would renumber the package's copy by CCH rank.
        &wayfarer_compiler::package_build::PackageBuildOptions {
            package_format: wayfarer_package::format::PackageFormat::V2,
            ..Default::default()
        },
    )
    .expect("package from the snapshot");
    let package = Arc::new(RoutingPackage::from_bytes(&bytes).expect("open the package"));
    let oracle = ReferenceEngine::from_owned_graph(graph, package.clone());
    let engine = RoutingEngine::from_package(package);
    eprintln!("engines in {:.0} s", start.elapsed().as_secs_f64());

    let terrain = Terrain::read(&std::fs::read(&terrain_path).expect("terrain file")).unwrap();
    let mut incident = Incident::new(
        wayfarer_terrain::surface::NO_INCIDENT,
        &terrain,
        None,
        vec![Rules::walker_default(), Rules::vehicle()],
        Settings::default(),
    )
    .unwrap();
    let start = Instant::now();
    incident.surface_at(T0).unwrap();
    let grid = incident.grid().unwrap();
    eprintln!("surface and grid in {:.1} s", start.elapsed().as_secs_f64());
    let timed = Arc::new(Timed {
        inner: TerrainPricer::from_incident(&incident).unwrap(),
        calls: Default::default(),
    });
    let handle = AccessPricerHandle(timed.clone());
    let walker = grid.profile("walker").unwrap();
    let points = sample(&grid, walker, (2 * pairs).max(16), 14);

    let mut report = serde_json::Map::new();
    for profile in [MobilityProfile::OnFoot, MobilityProfile::Wheeled] {
        let mut tally = Tally::default();
        timed.calls.lock().unwrap().clear();
        for k in 0..pairs {
            let (origin, destination) = (points[2 * k], points[2 * k + 1]);
            tally.pairs += 1;
            if std::env::var_os("PORTAL_PARITY_TRACE").is_some() {
                eprintln!("{profile:?} {k}: {origin:?} -> {destination:?}");
            }
            if std::env::var("PORTAL_PARITY_ONLY").is_ok_and(|only| only != k.to_string()) {
                continue;
            }
            let side = std::env::var("PORTAL_PARITY_SIDE").unwrap_or_default();
            let skipped = || Err(wayfarer::RouteError::InvalidRequest { reason: "skipped" });
            let actual = if side == "oracle" {
                skipped()
            } else {
                engine
                    .route_leg(request(origin, destination, profile, &handle, None))
                    .map_err(|failure| failure.error)
            };
            if std::env::var_os("PORTAL_PARITY_TRACE").is_some() {
                eprintln!("  engine done: {:?}", actual.as_ref().map(|r| r.cost));
            }
            let expected = if side == "engine" {
                skipped()
            } else {
                oracle.route_leg(request(origin, destination, profile, &handle, None))
            };
            let (actual, expected) = match (actual, expected) {
                (Ok(a), Ok(e)) => (a, e),
                (Err(_), Err(_)) => {
                    tally.both_failed += 1;
                    continue;
                }
                (a, e) => {
                    tally.mismatched.push(format!(
                        "{k}: engine {:?} oracle {:?}",
                        a.map(|r| r.cost),
                        e.map(|r| r.cost)
                    ));
                    continue;
                }
            };
            tally.routed += 1;
            if !(close(actual.cost, expected.cost)
                && close(actual.duration_s, expected.duration_s)
                && close(actual.distance_m, expected.distance_m))
            {
                tally.mismatched.push(format!(
                    "{k}: cost {} vs {}, duration {} vs {}",
                    actual.cost, expected.cost, actual.duration_s, expected.duration_s
                ));
            }
            check_totals(&actual, &mut tally);
            let truncated = [&actual.origin, &actual.destination].iter().any(|end| {
                end.access
                    .as_ref()
                    .and_then(|access| access.priced.as_ref())
                    .and_then(|priced| priced.pricing.as_ref())
                    .is_some_and(|pricing| pricing.truncated.is_some())
            });
            if truncated {
                let all = engine
                    .route_leg(request(
                        origin,
                        destination,
                        profile,
                        &handle,
                        Some(wayfarer::access::ACCESS_MAX_CANDIDATES),
                    ))
                    .map_err(|failure| failure.error)
                    .expect("more candidates still route");
                assert!(
                    all.cost <= actual.cost + 1e-6,
                    "{k}: more candidates cost more"
                );
                if all.cost < actual.cost - 1e-6 {
                    tally.cut_cost_route += 1;
                    tally.saved_s.push(actual.cost - all.cost);
                }
            }
        }
        let mut matrix =
            RouteMatrixRequest::new(points[..8].to_vec(), points[8..16].to_vec(), profile);
        matrix.options.off_network = Some(RouteOffNetworkOptions {
            pricer: Some(handle.clone()),
            ..Default::default()
        });
        let (expected, actual) = (oracle.matrix(matrix.clone()), engine.matrix(matrix));
        let mut cells = 0;
        let mut cell_mismatch = 0;
        for (e, a) in expected.cells.iter().zip(&actual.cells) {
            cells += 1;
            match (e.cost, a.cost) {
                (Some(x), Some(y)) if close(x, y) => {}
                (None, None) => {}
                _ => cell_mismatch += 1,
            }
        }
        let mut calls = timed.calls.lock().unwrap().clone();
        calls.sort_by(f64::total_cmp);
        let pct = |v: &[f64], p: f64| -> f64 {
            if v.is_empty() {
                return 0.0;
            }
            v[((v.len() - 1) as f64 * p).round() as usize]
        };
        let mut stretch = tally.stretch_s.clone();
        stretch.sort_by(f64::total_cmp);
        tally.cut_gaps_s.sort_by(f64::total_cmp);
        report.insert(
            format!("{profile:?}"),
            serde_json::json!({
                "pairs": tally.pairs,
                "routed": tally.routed,
                "both_failed": tally.both_failed,
                "mismatched": tally.mismatched,
                "totals_off": tally.totals_off,
                "priced_ends": tally.priced_ends,
                "straight_ends": tally.straight_ends,
                "truncated_ends": tally.truncated_ends,
                "cut_gap_s_p50": pct(&tally.cut_gaps_s, 0.5),
                "routes_a_cut_made_dearer": tally.cut_cost_route,
                "saved_by_all_candidates_s": tally.saved_s,
                "stretch_s_p50": pct(&stretch, 0.5),
                "stretch_s_p90": pct(&stretch, 0.9),
                "pricer_ms_p50": pct(&calls, 0.5),
                "pricer_ms_p90": pct(&calls, 0.9),
                "pricer_ms_max": calls.last().copied().unwrap_or(0.0),
                "pricer_calls": calls.len(),
                "matrix_cells": cells,
                "matrix_cells_mismatched": cell_mismatch,
            }),
        );
    }
    report.insert(
        "surface_hash".into(),
        wayfarer_terrain::hex(&timed.inner.surface().content_hash()).into(),
    );
    println!("{}", serde_json::to_string_pretty(&report).unwrap());
}

fn check_totals(result: &LegResult, tally: &mut Tally) {
    let Some(summary) = &result.off_network else {
        return;
    };
    let road_m: f64 = result.legs[0]
        .edges
        .iter()
        .map(|edge| edge.distance_m)
        .sum();
    let mut stretches_m = 0.0;
    for stretch in [&summary.origin, &summary.destination]
        .into_iter()
        .flatten()
    {
        stretches_m += stretch.distance_m;
        match &stretch.pricing {
            Some(pricing) => {
                tally.priced_ends += 1;
                tally.stretch_s.push(stretch.duration_s);
                if let Some(cut) = &pricing.truncated {
                    tally.truncated_ends += 1;
                    tally
                        .cut_gaps_s
                        .push(cut.best_cut_key_s - cut.worst_kept_key_s);
                }
            }
            None => tally.straight_ends += 1,
        }
    }
    if (result.distance_m - road_m - stretches_m).abs() > 1e-6 * result.distance_m.max(1.0) {
        tally.totals_off += 1;
    }
}

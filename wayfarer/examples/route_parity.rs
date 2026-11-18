//! Compares the routing package's search with the reference oracle
//! (`wayfarer::reference`, plain Dijkstra over the road graph's own
//! records) on a real package: route cost/path parity plus warm timings.
//!
//! ```sh
//! cargo run --release -p wayfarer --example route_parity -- \
//!   <routing.mrt> <corpus.json> [random_pairs=40] [repeats=3]
//! ```
//!
//! The reference needs the road graph the package was compiled from:
//! `REFERENCE_ROAD_GRAPH=<snapshot>` (written with
//! `wayfarer_road_graph::persist_road_graph_file`). Without it the reference
//! columns report `skipped`.
//!
//! The corpus is a JSON array of `{label, origin:[lng,lat], destination:[lng,lat],
//! profile:"wheeled"|"on_foot"}`. Random pairs are drawn inside the corpus
//! bounding box with a fixed seed. Requests use 150 m correlation and a
//! visited-edge budget of `ROUTE_PARITY_BUDGET` (default 250k) for the compact
//! engine; the reference always searches to completion, once per case.
//!
//! `ROUTE_PARITY_DUMP=<file>` writes one JSON line per case with both engines'
//! status, cost, distance, duration, full edge-id path, visited edges and SAR
//! use totals, for exact before/after diffs of either column.
//!
//! Option sets: `ROUTE_PARITY_COSTING` takes a JSON `RouteCostingOptions` (e.g.
//! `{"avoidMotorways":1,"preferPaved":0.5}`) applied to every request, and
//! `ROUTE_PARITY_PROFILES` maps the corpus profiles (`wheeled=tracked,on_foot=legged`).
//! The derived column for each profile is probed first (build time, how the
//! potentials bound it, and, for default options, bit equality with the
//! stored column).
//! `ROUTE_PARITY_OVERLAY=1` gives each case a dynamic overlay derived from its
//! plain route (a full block, and a partial block with a bypass connector).
//! `ROUTE_PARITY_SKIP_REFERENCE=1` runs only the package engine (the reference
//! columns report `skipped`), for before/after dumps of the package search
//! alone. With `WAYFARER_VERIFY=1` every package route is re-checked
//! edge by edge from the package's raw attributes (not its precomputed
//! columns) through the costing policy and the restriction lists.
//! `ROUTE_PARITY_REFERENCE_CONTEXT=<k>` gives the reference a path context of k
//! edges, exact under complex restrictions with up to k prefix edges.
//!
//! `ROUTE_PARITY_DISCOUNT=<m>` gives each case a discount `m` in (0, 1)
//! (a cleared route), on the edges `ROUTE_PARITY_DISCOUNT_ON` names: `own`
//! (default) its plain route, `parallel` the edges of the route that avoids
//! its plain route, `far` the next case's route. Discounts are costed exactly
//! and are bounded through discount gates, so this measures what they
//! cost in visited edges (compare the totals with a run without it).
//! `ROUTE_PARITY_OPEN=<own|far>` opens a road kept for SAR only (no legal
//! access for any mode) for each case: every directed edge of the kept way
//! nearest to the middle of its plain route (`own`) or of the next case's
//! (`far`), costed by the profile's own model.
//! `ROUTE_PARITY_SAR=<RouteSarOptions JSON>` (e.g.
//! `{"relaxations": {"turn_restrictions": {}}}`) adds SAR relaxations to every
//! wheeled case and drops the others (SAR needs a vehicle profile; map the
//! wheeled cases with `ROUTE_PARITY_PROFILES`).

use std::sync::Arc;
use std::time::Instant;

use wayfarer::reference::ReferenceEngine;
use wayfarer::{
    GeoPoint, LegRequest, LegResult, MobilityProfile, RouteCostingOptions, RouteDynamicOverlay,
    RouteDynamicVirtualEdge, RouteDynamicVirtualEdgeKind, RouteEdgeRange, RoutingEngine,
    RoutingPackage,
};

/// A failure: its error name, search and correlation ms, visited edges.
type Failure = (String, f64, f64, usize);

/// The engines compared.
trait Router {
    fn route_outcome(&self, request: LegRequest) -> Result<LegResult, Failure>;
}

fn failure_name(error: &impl std::fmt::Debug) -> String {
    format!("{error:?}")
        .split_whitespace()
        .next()
        .unwrap_or_default()
        .to_string()
}

impl Router for RoutingEngine {
    fn route_outcome(&self, request: LegRequest) -> Result<LegResult, Failure> {
        self.route_leg(request).map_err(|failure| {
            (
                failure_name(&failure.error),
                failure.diagnostics.search_duration_ns as f64 / 1e6,
                failure.diagnostics.correlation_duration_ns as f64 / 1e6,
                failure.diagnostics.visited_edge_count,
            )
        })
    }
}

impl Router for ReferenceEngine<'_> {
    fn route_outcome(&self, request: LegRequest) -> Result<LegResult, Failure> {
        self.route_leg(request)
            .map_err(|error| (failure_name(&error), f64::NAN, f64::NAN, 0))
    }
}

struct Case {
    label: String,
    origin: GeoPoint,
    destination: GeoPoint,
    profile: MobilityProfile,
    overlay: Option<RouteDynamicOverlay>,
    /// Edges discounted by `ROUTE_PARITY_DISCOUNT`.
    discounted: Vec<u64>,
    /// Edges opened by `ROUTE_PARITY_OPEN`.
    opened: Vec<u64>,
}

fn discount_multiplier() -> Option<f64> {
    std::env::var("ROUTE_PARITY_DISCOUNT")
        .ok()
        .and_then(|value| value.parse().ok())
}

fn sar_options() -> Option<wayfarer::RouteSarOptions> {
    std::env::var("ROUTE_PARITY_SAR")
        .ok()
        .map(|json| serde_json::from_str(&json).expect("ROUTE_PARITY_SAR is a SAR options JSON"))
}

#[derive(Clone)]
struct Outcome {
    ok: bool,
    error: String,
    cost: f64,
    distance_m: f64,
    duration_s: f64,
    edge_ids: Vec<u64>,
    total_ms: f64,
    search_ms: f64,
    correlation_ms: f64,
    visited: usize,
    /// The SAR report's totals per relaxation (`RouteSarUseTotal`), on SAR
    /// routes.
    sar_totals: serde_json::Value,
}

fn costing_options() -> RouteCostingOptions {
    std::env::var("ROUTE_PARITY_COSTING")
        .ok()
        .map(|json| serde_json::from_str(&json).expect("ROUTE_PARITY_COSTING is a costing JSON"))
        .unwrap_or_default()
}

fn parse_profile(name: &str) -> MobilityProfile {
    match name {
        "on_foot" | "walking" => MobilityProfile::OnFoot,
        "tracked" => MobilityProfile::Tracked,
        "emergency" => MobilityProfile::Emergency,
        "legged" => MobilityProfile::Legged,
        _ => MobilityProfile::Wheeled,
    }
}

/// `ROUTE_PARITY_PROFILES=wheeled=tracked,on_foot=legged` remaps corpus profiles.
fn mapped_profile(profile: MobilityProfile) -> MobilityProfile {
    let Ok(spec) = std::env::var("ROUTE_PARITY_PROFILES") else {
        return profile;
    };
    spec.split(',')
        .filter_map(|pair| pair.split_once('='))
        .find(|(from, _)| parse_profile(from) == profile)
        .map_or(profile, |(_, to)| parse_profile(to))
}

fn run(engine: &dyn Router, case: &Case) -> Outcome {
    let mut request = LegRequest::new(case.origin, case.destination, case.profile);
    request.options.costing = costing_options();
    request.options.dynamic_overlay = case.overlay.clone();
    if let Some(multiplier) = discount_multiplier() {
        request
            .options
            .edge_cost_multipliers
            .extend(case.discounted.iter().map(|edge| (*edge, multiplier)));
    }
    request
        .options
        .opened_edge_ids
        .extend(case.opened.iter().copied());
    request.options.sar = sar_options();
    request.options.max_correlation_distance_m = 150;
    request.options.max_visited_edge_count = Some(
        std::env::var("ROUTE_PARITY_BUDGET")
            .ok()
            .and_then(|value| value.parse().ok())
            .unwrap_or(250_000),
    );
    let started = Instant::now();
    let result = engine.route_outcome(request);
    let total_ms = started.elapsed().as_secs_f64() * 1e3;
    match result {
        Ok(route) => Outcome {
            ok: true,
            error: String::new(),
            cost: route.cost,
            distance_m: route.distance_m,
            duration_s: route.duration_s,
            edge_ids: route.edge_ids.clone(),
            total_ms,
            search_ms: route.diagnostics.search_duration_ns as f64 / 1e6,
            correlation_ms: route.diagnostics.correlation_duration_ns as f64 / 1e6,
            visited: route.diagnostics.visited_edge_count,
            sar_totals: route
                .sar
                .as_ref()
                .map(|report| serde_json::json!(report.totals))
                .unwrap_or(serde_json::Value::Null),
        },
        Err((error, search_ms, correlation_ms, visited)) => Outcome {
            ok: false,
            error,
            cost: f64::NAN,
            distance_m: f64::NAN,
            duration_s: f64::NAN,
            edge_ids: Vec::new(),
            total_ms,
            search_ms,
            correlation_ms,
            visited,
            sar_totals: serde_json::Value::Null,
        },
    }
}

fn median(mut values: Vec<f64>) -> f64 {
    values.sort_by(f64::total_cmp);
    values.get(values.len() / 2).copied().unwrap_or(f64::NAN)
}

fn timed(engine: &dyn Router, case: &Case, repeats: usize) -> Outcome {
    let first = run(engine, case);
    let mut runs = vec![first.clone()];
    for _ in 1..repeats.max(1) {
        runs.push(run(engine, case));
    }
    let warm = if runs.len() > 1 {
        &runs[1..]
    } else {
        &runs[..]
    };
    Outcome {
        total_ms: median(warm.iter().map(|o| o.total_ms).collect()),
        search_ms: median(warm.iter().map(|o| o.search_ms).collect()),
        correlation_ms: median(warm.iter().map(|o| o.correlation_ms).collect()),
        ..first
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
        &wayfarer_compiler::package_build::PackageBuildOptions::default(),
    )
    .expect("the reference road graph builds a routing package");
    let package = Arc::new(RoutingPackage::from_bytes(&bytes).expect("open the built package"));
    ReferenceEngine::from_owned_graph(graph, package)
}

fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    let archive = args.get(1).expect("archive path");
    let corpus_path = args.get(2).expect("corpus path");
    let random_pairs = args.get(3).and_then(|v| v.parse().ok()).unwrap_or(40usize);
    let repeats = args.get(4).and_then(|v| v.parse().ok()).unwrap_or(3usize);

    let started = Instant::now();
    let graph = RoutingPackage::open(std::path::Path::new(archive)).expect("open package");
    println!(
        "package open: {:.1} ms, {} nodes, {} edges, {:.1} MB",
        started.elapsed().as_secs_f64() * 1e3,
        graph.counts().node_count,
        graph.counts().edge_count,
        graph.byte_len() as f64 / 1e6
    );
    let profile_only = std::env::var("ROUTE_PARITY_PROFILE_ONLY").is_ok();
    let costing = costing_options();
    println!("costing options: {costing:?}");
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        let profile = mapped_profile(profile);
        // Twice: the second build is warm (pages faulted in).
        for _ in 0..2 {
            match graph.derived_column_probe(profile, &costing) {
                Some(probe) => println!(
                    "derived {profile:?} column: {:.1} ms, potential factor {:?} (own metric {}), k {:.5}, {} usable edges, stored mismatches {:?}",
                    probe.build_ms,
                    probe.potential_factor,
                    probe.own_metric,
                    probe.heuristic_cost_per_m,
                    probe.usable_edge_count,
                    probe.stored_mismatch_count
                ),
                None => println!("derived {profile:?} column: not probed"),
            }
        }
    }
    let graph = Arc::new(graph);

    let corpus: Vec<serde_json::Value> =
        serde_json::from_str(&std::fs::read_to_string(corpus_path).expect("read corpus"))
            .expect("parse corpus");
    let mut cases = corpus
        .iter()
        .map(|entry| {
            let point = |key: &str| {
                let pair = entry[key].as_array().expect("point");
                GeoPoint::new(pair[1].as_f64().unwrap(), pair[0].as_f64().unwrap())
            };
            Case {
                label: entry["label"].as_str().unwrap_or("case").to_string(),
                origin: point("origin"),
                destination: point("destination"),
                profile: mapped_profile(parse_profile(
                    entry["profile"].as_str().unwrap_or("wheeled"),
                )),
                overlay: None,
                discounted: Vec::new(),
                opened: Vec::new(),
            }
        })
        .collect::<Vec<_>>();
    let (min_lat, max_lat, min_lng, max_lng) = cases.iter().fold(
        (f64::MAX, f64::MIN, f64::MAX, f64::MIN),
        |(a, b, c, d), case| {
            (
                a.min(case.origin.lat).min(case.destination.lat),
                b.max(case.origin.lat).max(case.destination.lat),
                c.min(case.origin.lng).min(case.destination.lng),
                d.max(case.origin.lng).max(case.destination.lng),
            )
        },
    );
    let mut seed = 0x9E37_79B9_7F4A_7C15u64;
    let mut next = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    for index in 0..random_pairs {
        let point = |r1: f64, r2: f64| {
            GeoPoint::new(
                min_lat + (max_lat - min_lat) * r1,
                min_lng + (max_lng - min_lng) * r2,
            )
        };
        let origin = point(next(), next());
        let destination = point(next(), next());
        cases.push(Case {
            label: format!("random_{index:03}"),
            origin,
            destination,
            profile: mapped_profile(if index % 4 == 3 {
                MobilityProfile::OnFoot
            } else {
                MobilityProfile::Wheeled
            }),
            overlay: None,
            discounted: Vec::new(),
            opened: Vec::new(),
        });
    }

    if let Ok(only) = std::env::var("ROUTE_PARITY_ONLY_CASE") {
        cases.retain(|case| case.label == only);
    }
    if let Some(sar) = sar_options() {
        cases.retain(|case| {
            matches!(
                case.profile,
                MobilityProfile::Wheeled | MobilityProfile::Emergency | MobilityProfile::Tracked
            )
        });
        println!(
            "sar mode: {:?}, {} vehicle cases",
            sar.enabled(),
            cases.len()
        );
    }
    // `ROUTE_PARITY_REFERENCE_CONTEXT=<k>` makes the reference exact under complex
    // restrictions of up to k prefix edges (`ReferenceEngine::with_path_context`).
    let reference_context = std::env::var("ROUTE_PARITY_REFERENCE_CONTEXT")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(0);
    let reference = std::env::var_os("REFERENCE_ROAD_GRAPH").map(|path| {
        load_reference(std::path::Path::new(&path)).with_path_context(reference_context)
    });
    if profile_only {
        // Compact engine only, many repeats: for CPU sampling and phase timing.
        let compact = RoutingEngine::from_package(graph.clone());
        for case in cases.iter().take(9) {
            let mut runs = Vec::new();
            for _ in 0..repeats {
                runs.push(run(&compact, case));
            }
            let warm = &runs[1.min(runs.len() - 1)..];
            println!(
                "{:<34} total {:>8.2}  corr {:>8.2}  search {:>7.2}  other {:>8.2}  ok={}",
                case.label,
                median(warm.iter().map(|o| o.total_ms).collect()),
                median(warm.iter().map(|o| o.correlation_ms).collect()),
                median(warm.iter().map(|o| o.search_ms).collect()),
                median(
                    warm.iter()
                        .map(|o| o.total_ms - o.correlation_ms - o.search_ms)
                        .collect()
                ),
                runs[0].ok
            );
        }
        return;
    }
    let compact = RoutingEngine::from_package(graph.clone());
    if std::env::var_os("ROUTE_PARITY_OVERLAY").is_some() {
        // Per case: fully block the edge a third of the way along the plain
        // package route, and partly block the edge two thirds along with a
        // cheap bypass connector around the gap.
        for case in &mut cases {
            let plain = run(&compact, case);
            let edges = &plain.edge_ids;
            if !plain.ok || edges.len() < 4 {
                continue;
            }
            let (full, partial) = (edges[edges.len() / 3], edges[2 * edges.len() / 3]);
            let mut overlay = RouteDynamicOverlay::from_blocked_ranges(&[
                RouteEdgeRange {
                    edge_id: full,
                    start_fraction: 0.0,
                    end_fraction: 1.0,
                },
                RouteEdgeRange {
                    edge_id: partial,
                    start_fraction: 0.3,
                    end_fraction: 0.6,
                },
            ]);
            overlay.virtual_edges.push(RouteDynamicVirtualEdge {
                edge_id: (1 << 63) | 1,
                connector_id: "bypass".into(),
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
                provenance: "route_parity".into(),
            });
            overlay.refresh_diagnostics();
            case.overlay = Some(overlay);
        }
        cases.retain(|case| case.overlay.is_some());
        println!("overlay mode: {} cases", cases.len());
    }

    if let Some(multiplier) = discount_multiplier() {
        // The plain package routes (without the discount) pick the edges.
        let plain = cases
            .iter()
            .map(|case| {
                let mut request = LegRequest::new(case.origin, case.destination, case.profile);
                request.options.costing = costing_options();
                request.options.max_correlation_distance_m = 150;
                compact
                    .route_leg(request)
                    .map(|route| route.edge_ids)
                    .unwrap_or_default()
            })
            .collect::<Vec<_>>();
        let target = std::env::var("ROUTE_PARITY_DISCOUNT_ON").unwrap_or_else(|_| "own".into());
        for (index, case) in cases.iter_mut().enumerate() {
            case.discounted = match target.as_str() {
                "far" => {
                    let own = plain[index]
                        .iter()
                        .collect::<std::collections::BTreeSet<_>>();
                    plain[(index + 1) % plain.len()]
                        .iter()
                        .filter(|edge| !own.contains(edge))
                        .copied()
                        .collect()
                }
                "parallel" => {
                    let mut request = LegRequest::new(case.origin, case.destination, case.profile);
                    request.options.costing = costing_options();
                    request.options.max_correlation_distance_m = 150;
                    request.options.avoided_edge_ids = plain[index].iter().copied().collect();
                    compact
                        .route_leg(request)
                        .map(|route| {
                            route
                                .edge_ids
                                .into_iter()
                                .filter(|edge| !plain[index].contains(edge))
                                .collect()
                        })
                        .unwrap_or_default()
                }
                _ => plain[index].clone(),
            };
        }
        println!(
            "discount mode: {multiplier} on {target}, {} edges over {} cases",
            cases
                .iter()
                .map(|case| case.discounted.len())
                .sum::<usize>(),
            cases.len()
        );
    }

    if let Ok(target) = std::env::var("ROUTE_PARITY_OPEN") {
        // Kept roads: edges no mode may legally use, by way.
        let view = compact.route_graph_view();
        let mut kept = std::collections::BTreeMap::<i64, Vec<(u64, GeoPoint)>>::new();
        for index in 0..view.raw_edge_count() {
            let Some(record) = view.edge_record_by_raw_edge_index(index) else {
                continue;
            };
            if record.edge.is_sar_only() {
                kept.entry(record.shared_info.osm_way_id)
                    .or_default()
                    .push((
                        record.edge.edge_id,
                        GeoPoint::new(record.source.lat, record.source.lng),
                    ));
            }
        }
        let middles = cases
            .iter()
            .map(|case| {
                let mut request = LegRequest::new(case.origin, case.destination, case.profile);
                request.options.costing = costing_options();
                request.options.max_correlation_distance_m = 150;
                let route = compact.route_leg(request).ok()?;
                let edge = *route.edge_ids.get(route.edge_ids.len() / 2)?;
                let record = view.search_edge_by_id(edge)?;
                Some(GeoPoint::new(record.source.lat, record.source.lng))
            })
            .collect::<Vec<_>>();
        let far = target == "far";
        let count = cases.len();
        for (index, case) in cases.iter_mut().enumerate() {
            let Some(middle) = middles[if far { (index + 1) % count } else { index }] else {
                continue;
            };
            // Nearest in degrees, longitude scaled by cos 50° (southern
            // Poland): only an order of candidates.
            let squared = |point: &GeoPoint| {
                let (dlat, dlng) = (point.lat - middle.lat, (point.lng - middle.lng) * 0.643);
                dlat * dlat + dlng * dlng
            };
            case.opened = kept
                .values()
                .min_by(|a, b| {
                    let nearest = |edges: &Vec<(u64, GeoPoint)>| {
                        edges
                            .iter()
                            .map(|(_, point)| squared(point))
                            .fold(f64::INFINITY, f64::min)
                    };
                    nearest(a).total_cmp(&nearest(b))
                })
                .map(|edges| edges.iter().map(|(edge, _)| *edge).collect())
                .unwrap_or_default();
        }
        println!(
            "open mode: kept road near {target}, {} edges over {} cases ({} kept ways)",
            cases.iter().map(|case| case.opened.len()).sum::<usize>(),
            cases.len(),
            kept.len()
        );
    }

    println!(
        "\n{:<34} {:>7} | {:>9} {:>9} {:>8} | {:>9} {:>9} {:>8} | {:>9} {:>8} {:>6}",
        "case",
        "km",
        "ref_ms",
        "ref_srch",
        "ref_vis",
        "cmp_ms",
        "cmp_srch",
        "cmp_vis",
        "cost_d%",
        "dur_d%",
        "path"
    );
    let (mut same, mut better, mut worse, mut both_fail, mut mismatch, mut compared) =
        (0, 0, 0, 0, 0, 0);
    let (mut reference_search, mut package_search) = (Vec::new(), Vec::new());
    let (mut package_visited, mut package_ok) = (0usize, 0usize);
    let mut dump = std::env::var_os("ROUTE_PARITY_DUMP")
        .map(|path| std::fs::File::create(path).expect("create ROUTE_PARITY_DUMP file"));
    let skip_reference = std::env::var_os("ROUTE_PARITY_SKIP_REFERENCE").is_some();
    for case in &cases {
        let reference = reference.as_ref().filter(|_| !skip_reference);
        let left = if let Some(reference) = reference {
            timed(reference, case, 1)
        } else {
            Outcome {
                ok: false,
                error: "skipped".into(),
                cost: f64::NAN,
                distance_m: f64::NAN,
                duration_s: f64::NAN,
                edge_ids: Vec::new(),
                total_ms: f64::NAN,
                search_ms: f64::NAN,
                correlation_ms: f64::NAN,
                visited: 0,
                sar_totals: serde_json::Value::Null,
            }
        };
        let right = timed(&compact, case, repeats);
        package_visited += right.visited;
        package_ok += usize::from(right.ok);
        if let Some(file) = dump.as_mut() {
            use std::io::Write;
            let outcome = |outcome: &Outcome| {
                serde_json::json!({
                    "ok": outcome.ok,
                    "error": outcome.error,
                    "cost": outcome.ok.then_some(outcome.cost),
                    "distanceM": outcome.ok.then_some(outcome.distance_m),
                    "durationS": outcome.ok.then_some(outcome.duration_s),
                    "edgeIds": outcome.edge_ids,
                    "visited": outcome.visited,
                    "sarTotals": outcome.sar_totals,
                })
            };
            let line = serde_json::json!({
                "label": case.label,
                "profile": format!("{:?}", case.profile),
                "visited": right.visited,
                "reference": outcome(&left),
                "package": outcome(&right),
            });
            writeln!(file, "{line}").expect("write ROUTE_PARITY_DUMP line");
        }
        if !left.ok && !right.ok {
            both_fail += 1;
            if left.error == "NoCandidate" {
                continue;
            }
        }
        let km = if right.ok {
            right.distance_m
        } else {
            left.distance_m
        } / 1000.0;
        let (delta, path) = if left.ok && right.ok {
            compared += 1;
            reference_search.push(left.total_ms);
            package_search.push(right.search_ms);
            let delta = (right.cost - left.cost) / left.cost.max(1e-9) * 100.0;
            let path = if left.edge_ids == right.edge_ids {
                same += 1;
                "same"
            } else if delta < -0.01 {
                better += 1;
                "better"
            } else if delta > 0.01 {
                worse += 1;
                "WORSE"
            } else {
                same += 1;
                "equal"
            };
            (format!("{delta:+.3}"), path.to_string())
        } else if left.ok != right.ok {
            mismatch += 1;
            (
                "-".to_string(),
                format!(
                    "{}/{}",
                    if left.ok { "ok" } else { &left.error },
                    if right.ok { "ok" } else { &right.error }
                ),
            )
        } else {
            ("-".to_string(), left.error.clone())
        };
        println!(
            "{:<34} {:>7.1} | {:>9.2} {:>9.2} {:>8} | {:>9.2} {:>9.2} {:>8} | {:>9} {:>8} {:>6}",
            case.label,
            km,
            left.total_ms,
            left.search_ms,
            left.visited,
            right.total_ms,
            right.search_ms,
            right.visited,
            delta,
            if left.ok && right.ok {
                format!(
                    "{:+.3}",
                    (right.duration_s - left.duration_s) / left.duration_s.max(1e-9) * 100.0
                )
            } else {
                "-".to_string()
            },
            path
        );
    }
    println!(
        "\ncompared {compared}: same/equal-cost {same}, compact cheaper {better}, compact MORE EXPENSIVE {worse}; ok/err mismatches {mismatch}; both failed {both_fail}"
    );
    println!(
        "median ms: reference request {:.2}, package search {:.3}",
        median(reference_search),
        median(package_search)
    );
    println!(
        "package visited edges: {package_visited} over {} cases ({package_ok} routed)",
        cases.len()
    );
}

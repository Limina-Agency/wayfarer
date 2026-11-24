//! Dumps route responses and correlation candidates on a real routing
//! package, for diffing two builds of the engine, and times route requests.
//!
//! ```sh
//! cargo run --release -p wayfarer --example route_response_dump -- \
//!   <routing.mrt> <corpus.json> [random_pairs=200] [out.jsonl] [correlation_points=2000]
//! ```
//!
//! Every line of `out.jsonl` is one route response (edge ids, geometry, legs,
//! maneuvers, recosts, endpoints, alternatives) or one correlation candidate
//! search, with timing and tile-cache diagnostics removed, so two builds that
//! answer identically produce identical files. Requests use a host app's options
//! (150 m correlation, 2 M visited-edge budget).
//!
//! With `DUMP_COLD_CASE=<label>` the example routes only that case, once per
//! process start, and prints the first request's phase timings, then the warm
//! median of `DUMP_WARM_REPEATS` (default 20) more. `DUMP_PREFETCH=1`
//! prefetches before the first request; `DUMP_WARM_UP=1` starts the
//! background warm-up and routes at once, or after `DUMP_FIRST_DELAY_MS`.

use std::io::Write;
use std::sync::Arc;
use std::time::Instant;

use wayfarer::{
    CorrelationPolicy, CorrelationRoadClassPreference, GeoPoint, LegRequest, LegResult,
    MobilityProfile, PreferredRoadSide, RouteEndpointOptions, RoutingEngine, RoutingPackage,
};

use serde_json::Value;
use wayfarer::internals::Correlator;

#[derive(Clone)]
struct Case {
    label: String,
    origin: GeoPoint,
    destination: GeoPoint,
    profile: MobilityProfile,
    hints: bool,
    alternatives: usize,
}

fn request(case: &Case) -> LegRequest {
    let mut request = LegRequest::new(case.origin, case.destination, case.profile);
    request.options.max_correlation_distance_m = 150;
    request.options.max_visited_edge_count = Some(2_000_000);
    request.options.alternatives.requested_alternative_count = case.alternatives;
    if case.hints {
        request.origin_options = hinted_endpoint(17);
        request.destination_options = hinted_endpoint(211);
    }
    request
}

fn hinted_endpoint(heading: u16) -> RouteEndpointOptions {
    RouteEndpointOptions {
        heading_deg: Some(heading),
        heading_tolerance_deg: Some(120),
        preferred_side: PreferredRoadSide::Right,
        preferred_layer: Some(0),
        avoid_private_access: true,
        preferred_road_classes: vec![
            CorrelationRoadClassPreference::Primary,
            CorrelationRoadClassPreference::Secondary,
        ],
        avoid_service_roads: true,
        prefer_any_route_ref: true,
        min_outbound_reach_edge_count: Some(8),
        ..RouteEndpointOptions::default()
    }
}

/// Diagnostics that describe the answer rather than how it was computed.
const KEPT_DIAGNOSTICS: &[&str] = &[
    "originCandidateCount",
    "destinationCandidateCount",
    "rejectedCandidateCount",
    "originRejectedCandidateCount",
    "destinationRejectedCandidateCount",
    "originBestCandidateScoreBasisPoints",
    "destinationBestCandidateScoreBasisPoints",
    "originBestOutboundReachEdgeCount",
    "destinationBestInboundReachEdgeCount",
    "relaxedCorrelationRetryCount",
    "originRejectedCandidates",
    "destinationRejectedCandidates",
    "visitedEdgeCount",
    "finalPathEdgeCount",
    "acceptedAlternativeCount",
];

fn strip_diagnostics(value: &mut Value) {
    match value {
        Value::Object(map) => {
            if let Some(Value::Object(diagnostics)) = map.get_mut("diagnostics") {
                diagnostics.retain(|key, _| KEPT_DIAGNOSTICS.contains(&key.as_str()));
            }
            for child in map.values_mut() {
                strip_diagnostics(child);
            }
        }
        Value::Array(items) => items.iter_mut().for_each(strip_diagnostics),
        _ => {}
    }
}

fn response_json(label: &str, result: &Result<LegResult, wayfarer::RouteFailure>) -> Value {
    let mut value = match result {
        Ok(route) => serde_json::to_value(route).expect("serialize route"),
        Err(failure) => serde_json::json!({
            "error": format!("{:?}", failure.error),
            "diagnostics": serde_json::to_value(&failure.diagnostics).expect("serialize"),
        }),
    };
    strip_diagnostics(&mut value);
    serde_json::json!({ "case": label, "response": value })
}

fn median(mut values: Vec<f64>) -> f64 {
    values.sort_by(f64::total_cmp);
    values.get(values.len() / 2).copied().unwrap_or(f64::NAN)
}

fn percentile(mut values: Vec<f64>, fraction: f64) -> f64 {
    values.sort_by(f64::total_cmp);
    let index = ((values.len() as f64 * fraction).ceil() as usize).saturating_sub(1);
    values.get(index).copied().unwrap_or(f64::NAN)
}

fn phases(route: &Result<LegResult, wayfarer::RouteFailure>) -> String {
    let d = match route {
        Ok(route) => &route.diagnostics,
        Err(failure) => &failure.diagnostics,
    };
    let ms = |ns: u64| ns as f64 / 1e6;
    format!(
        "corr {:.2}  search {:.2}  leg {:.2}",
        ms(d.correlation_duration_ns),
        ms(d.search_duration_ns),
        ms(d.route_leg_build_duration_ns),
    )
}

fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    let archive = args.get(1).expect("archive path");
    let corpus_path = args.get(2).expect("corpus path");
    let random_pairs = args.get(3).and_then(|v| v.parse().ok()).unwrap_or(200usize);
    let out_path = args.get(4).cloned();
    let correlation_points = args
        .get(5)
        .and_then(|v| v.parse().ok())
        .unwrap_or(2000usize);

    let process_started = Instant::now();
    let graph = RoutingPackage::open(std::path::Path::new(archive)).expect("open package");
    // DUMP_PREFETCH=1: fault the package in before the first request, as a
    // service does in the background after opening a source.
    let prefetch_ms = std::env::var_os("DUMP_PREFETCH").map(|_| {
        let started = Instant::now();
        graph.prefetch();
        started.elapsed().as_secs_f64() * 1e3
    });
    if let Some(ms) = prefetch_ms {
        eprintln!("prefetch {ms:.0} ms");
    }
    let open_ms = process_started.elapsed().as_secs_f64() * 1e3;
    let graph = Arc::new(graph);
    // DUMP_WARM_UP=1: start the opt-in background warm-up right after the
    // open, as a host with `warmUp` does, and route at once.
    let _warm_up = std::env::var_os("DUMP_WARM_UP").and_then(|_| graph.warm_up_in_background());
    let engine = RoutingEngine::from_package(graph);

    let corpus: Vec<Value> =
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
                profile: match entry["profile"].as_str() {
                    Some("on_foot") | Some("walking") => MobilityProfile::OnFoot,
                    _ => MobilityProfile::Wheeled,
                },
                hints: false,
                alternatives: 0,
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
    let mut next = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    let random_point = |next: &mut dyn FnMut() -> f64| {
        GeoPoint::new(
            min_lat + (max_lat - min_lat) * next(),
            min_lng + (max_lng - min_lng) * next(),
        )
    };
    for case in cases.clone() {
        // The corpus again with endpoint hints and alternatives.
        cases.push(Case {
            label: format!("{}_hinted_alt", case.label),
            hints: true,
            alternatives: 2,
            ..case
        });
    }
    for index in 0..random_pairs {
        let origin = random_point(&mut next);
        let destination = random_point(&mut next);
        cases.push(Case {
            label: format!("random_{index:03}"),
            origin,
            destination,
            profile: if index % 4 == 3 {
                MobilityProfile::OnFoot
            } else {
                MobilityProfile::Wheeled
            },
            hints: index % 10 == 7,
            alternatives: usize::from(index % 15 == 4),
        });
    }

    // DUMP_PHASE_BENCH=<n>: per corpus case, the minimum and median over n
    // runs of correlation (both endpoints) and of the whole request. Minimums
    // are robust to other load on the machine.
    if let Some(runs) = std::env::var("DUMP_PHASE_BENCH")
        .ok()
        .and_then(|value| value.parse::<usize>().ok())
    {
        let correlator = Correlator::new(engine.route_graph_view());
        let policy = CorrelationPolicy {
            max_distance_m: 150,
            ..CorrelationPolicy::default()
        };
        for case in cases.iter().filter(|case| !case.hints).take(corpus.len()) {
            let (mut correlation, mut total) = (Vec::new(), Vec::new());
            for _ in 0..runs {
                let started = Instant::now();
                for point in [case.origin, case.destination] {
                    std::hint::black_box(correlator.search(
                        point,
                        case.profile,
                        &policy,
                        &RouteEndpointOptions::default(),
                    ));
                }
                correlation.push(started.elapsed().as_secs_f64() * 1e3);
                let started = Instant::now();
                std::hint::black_box(engine.route_leg(request(case)).is_ok());
                total.push(started.elapsed().as_secs_f64() * 1e3);
            }
            let min = |values: &[f64]| values.iter().copied().fold(f64::INFINITY, f64::min);
            println!(
                "{:<34} correlation min {:>6.3} med {:>6.3} | request min {:>7.3} med {:>7.3}",
                case.label,
                min(&correlation),
                median(correlation.clone()),
                min(&total),
                median(total.clone()),
            );
        }
        return;
    }

    if let Ok(only) = std::env::var("DUMP_COLD_CASE") {
        let case = cases
            .iter()
            .find(|case| case.label == only)
            .expect("DUMP_COLD_CASE names a case");
        // DUMP_FIRST_DELAY_MS: a host's idle time between the open and the
        // first request (a user choosing a destination).
        if let Some(ms) = std::env::var("DUMP_FIRST_DELAY_MS")
            .ok()
            .and_then(|ms| ms.parse().ok())
        {
            std::thread::sleep(std::time::Duration::from_millis(ms));
        }
        let started = Instant::now();
        let first = engine.route_leg(request(case));
        let first_ms = started.elapsed().as_secs_f64() * 1e3;
        let first_phases = phases(&first);
        let repeats = std::env::var("DUMP_WARM_REPEATS")
            .ok()
            .and_then(|value| value.parse().ok())
            .unwrap_or(20usize);
        let mut warm = Vec::new();
        let mut last = first;
        for _ in 0..repeats {
            let started = Instant::now();
            last = engine.route_leg(request(case));
            warm.push(started.elapsed().as_secs_f64() * 1e3);
        }
        println!(
            "{:<34} open {:>6.0}  first {:>7.2}  warm {:>6.2}  ratio {:>4.2}  ok={}\n  first: {first_phases}\n  warm:  {}",
            case.label,
            open_ms,
            first_ms,
            median(warm.clone()),
            first_ms / median(warm),
            last.is_ok(),
            phases(&last),
        );
        return;
    }

    let mut out: Box<dyn Write> = match &out_path {
        Some(path) => Box::new(std::io::BufWriter::new(
            std::fs::File::create(path).expect("create output"),
        )),
        None => Box::new(std::io::sink()),
    };
    let mut totals = Vec::new();
    for case in &cases {
        let started = Instant::now();
        let result = engine.route_leg(request(case));
        totals.push(started.elapsed().as_secs_f64() * 1e3);
        writeln!(out, "{}", response_json(&case.label, &result)).expect("write");
    }
    println!(
        "routes: {} cases, median request {:.2} ms, p95 {:.2} ms",
        cases.len(),
        median(totals.clone()),
        percentile(totals, 0.95)
    );

    let view = engine.route_graph_view();
    let correlator = Correlator::new(view);
    let mut correlation_ms = Vec::new();
    for index in 0..correlation_points {
        let point = random_point(&mut next);
        let profile = if index % 3 == 2 {
            MobilityProfile::OnFoot
        } else {
            MobilityProfile::Wheeled
        };
        let policy = CorrelationPolicy {
            max_distance_m: [150, 40, 500][index % 3],
            ..CorrelationPolicy::default()
        };
        let options = if index % 5 == 4 {
            hinted_endpoint((index * 37 % 360) as u16)
        } else {
            RouteEndpointOptions::default()
        };
        let started = Instant::now();
        let search = correlator.search(point, profile, &policy, &options);
        correlation_ms.push(started.elapsed().as_secs_f64() * 1e3);
        writeln!(
            out,
            "{}",
            serde_json::json!({
                "correlation": index,
                "point": [point.lat, point.lng],
                "profile": format!("{profile:?}"),
                "search": format!("{search:?}"),
            })
        )
        .expect("write");
    }
    println!(
        "correlation: {} points, median {:.3} ms",
        correlation_points,
        median(correlation_ms)
    );
}

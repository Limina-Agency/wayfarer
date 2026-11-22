//! Real-archive lifecycle and continuous rerouting benchmark, with a persistent
//! JSON-lines sidecar using exactly the same RoutingService and request policy.
use std::collections::BTreeMap;
use std::error::Error;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::time::Instant;

use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use wayfarer::{
    GeoPoint, LegRequest, MobilityProfile, NavigationPositionSample, RouteDiagnostics,
    RouteManeuver, RouteNavigationInput, RouteNavigationProgressOptions, RoutePackageMemoryProfile,
    evaluate_route_navigation_progress,
};
use wayfarer_service::{RoutingService, RoutingServiceError, RoutingSource, RoutingSourceId};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Clone, Serialize, Deserialize)]
struct Case {
    label: String,
    /// Longitude, latitude, matching the native application and OSRM.
    origin: [f64; 2],
    destination: [f64; 2],
    profile: MobilityProfile,
}

#[derive(Clone, Serialize, Deserialize)]
struct Response {
    ok: bool,
    error: Option<String>,
    service_ms: f64,
    source_cache_hit: bool,
    distance_m: Option<f64>,
    duration_s: Option<f64>,
    cost: Option<f64>,
    edge_count: usize,
    edge_fingerprint: Option<String>,
    origin_snap_m: Option<f64>,
    destination_snap_m: Option<f64>,
    snapped_origin: Option<GeoPoint>,
    snapped_destination: Option<GeoPoint>,
    geometry: Vec<GeoPoint>,
    maneuvers: Vec<RouteManeuver>,
    diagnostics: Option<RouteDiagnostics>,
}

#[derive(Serialize)]
struct Sample {
    mode: String,
    label: String,
    iteration: usize,
    origin: [f64; 2],
    destination: [f64; 2],
    profile: MobilityProfile,
    latency_ms: f64,
    geometry_point_count: usize,
    geometry_length_m: f64,
    geometry_distance_ratio: Option<f64>,
    geometry_origin_gap_m: Option<f64>,
    geometry_destination_gap_m: Option<f64>,
    valid_geometry: bool,
    same_as_cold_status: Option<bool>,
    same_as_cold_edges: Option<bool>,
    same_as_cold_distance: Option<bool>,
    same_as_in_process_edges: Option<bool>,
    same_as_in_process_status: Option<bool>,
    remaining_distance_increase_m: Option<f64>,
    #[serde(flatten)]
    response: Response,
}

fn source(path: &Path) -> Result<RoutingSource> {
    Ok(RoutingSource::new(
        RoutingSourceId::new("regional-benchmark")?,
        "immutable-benchmark",
        path,
    )?)
}

fn compute(service: &RoutingService, source: &RoutingSource, case: &Case) -> Response {
    let start = Instant::now();
    // Match RoutingHost::compute, including its preflight source inspection.
    let readiness = service.inspect_source(source);
    let mut request = LegRequest::new(
        GeoPoint::new(case.origin[1], case.origin[0]),
        GeoPoint::new(case.destination[1], case.destination[0]),
        case.profile,
    );
    request.options.max_correlation_distance_m = 150;
    request.options.max_visited_edge_count = Some(250_000);
    let result = service.route_leg(source, request);
    let elapsed = start.elapsed().as_secs_f64() * 1000.0;
    let mut response = Response {
        ok: false,
        error: None,
        service_ms: elapsed,
        source_cache_hit: readiness.cache_hit,
        distance_m: None,
        duration_s: None,
        cost: None,
        edge_count: 0,
        edge_fingerprint: None,
        origin_snap_m: None,
        destination_snap_m: None,
        snapped_origin: None,
        snapped_destination: None,
        geometry: Vec::new(),
        maneuvers: Vec::new(),
        diagnostics: None,
    };
    match result {
        Ok(computation) => {
            let route = computation.route;
            let mut fingerprint = 14_695_981_039_346_656_037u64;
            for edge in &route.edge_ids {
                for byte in edge.to_le_bytes() {
                    fingerprint ^= u64::from(byte);
                    fingerprint = fingerprint.wrapping_mul(1_099_511_628_211);
                }
            }
            response.ok = true;
            response.edge_count = route.edge_ids.len();
            response.edge_fingerprint = Some(format!("{fingerprint:016x}"));
            response.distance_m = Some(route.distance_m);
            response.duration_s = Some(route.duration_s);
            response.cost = Some(route.cost);
            response.origin_snap_m = Some(route.origin.distance_m);
            response.destination_snap_m = Some(route.destination.distance_m);
            response.snapped_origin = Some(route.origin.snapped_point);
            response.snapped_destination = Some(route.destination.snapped_point);
            response.geometry = route.geometry;
            response.maneuvers = route.maneuvers;
            response.diagnostics = Some(route.diagnostics);
        }
        Err(error) => {
            response.error = Some(error.to_string());
            if let RoutingServiceError::RouteFailed(failure) = error {
                response.diagnostics = Some(*failure.diagnostics);
            }
        }
    }
    response
}

fn serve(archive: &Path) -> Result<()> {
    let source = source(archive)?;
    let service = RoutingService::default();
    let mut out = BufWriter::new(std::io::stdout().lock());
    // Ready means process/protocol ready; first request still pays archive opening.
    writeln!(out, "{{\"ready\":true}}")?;
    out.flush()?;
    for line in std::io::stdin().lock().lines() {
        let case: Case = serde_json::from_str(&line?)?;
        serde_json::to_writer(&mut out, &compute(&service, &source, &case))?;
        writeln!(out)?;
        out.flush()?;
    }
    Ok(())
}

struct Sidecar {
    child: Child,
    input: BufWriter<ChildStdin>,
    output: BufReader<ChildStdout>,
    startup_ms: f64,
}

impl Sidecar {
    fn new(archive: &Path) -> Result<Self> {
        let start = Instant::now();
        let mut child = Command::new(std::env::current_exe()?)
            .arg("serve")
            .arg(archive)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()?;
        let input = BufWriter::new(child.stdin.take().ok_or("missing child stdin")?);
        let mut output = BufReader::new(child.stdout.take().ok_or("missing child stdout")?);
        let mut line = String::new();
        output.read_line(&mut line)?;
        if serde_json::from_str::<Value>(&line)?["ready"] != true {
            return Err("sidecar did not become ready".into());
        }
        Ok(Self {
            child,
            input,
            output,
            startup_ms: start.elapsed().as_secs_f64() * 1000.0,
        })
    }

    fn route_leg(&mut self, case: &Case) -> Result<Response> {
        serde_json::to_writer(&mut self.input, case)?;
        writeln!(self.input)?;
        self.input.flush()?;
        let mut line = String::new();
        if self.output.read_line(&mut line)? == 0 {
            return Err("sidecar closed stdout before response".into());
        }
        Ok(serde_json::from_str(&line)?)
    }
}

impl Drop for Sidecar {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

fn sample(
    mode: &str,
    case: &Case,
    iteration: usize,
    latency_ms: f64,
    mut response: Response,
    baseline: Option<&Response>,
    previous_distance: Option<f64>,
    in_process: Option<&Response>,
) -> Sample {
    let same_as_cold_edges = baseline
        .filter(|b| b.ok && response.ok)
        .map(|b| b.edge_fingerprint == response.edge_fingerprint);
    let same_as_cold_status = baseline.map(|b| b.ok == response.ok);
    let same_as_in_process_status = in_process.map(|b| b.ok == response.ok);
    let same_as_cold_distance = baseline
        .filter(|b| b.ok && response.ok)
        .map(|b| (b.distance_m.unwrap() - response.distance_m.unwrap()).abs() < 1e-6);
    let same_as_in_process_edges = in_process
        .filter(|b| b.ok && response.ok)
        .map(|b| b.edge_fingerprint == response.edge_fingerprint);
    let remaining_distance_increase_m = previous_distance
        .zip(response.distance_m)
        .map(|(a, b)| b - a);
    let geometry_point_count = response.geometry.len();
    let geometry_length_m: f64 = response
        .geometry
        .windows(2)
        .map(|p| p[0].distance_to(p[1]))
        .sum();
    let geometry_distance_ratio = response
        .distance_m
        .filter(|d| *d > 0.0)
        .map(|d| geometry_length_m / d);
    let geometry_origin_gap_m = response
        .geometry
        .first()
        .zip(response.snapped_origin)
        .map(|(a, b)| a.distance_to(b));
    let geometry_destination_gap_m = response
        .geometry
        .last()
        .zip(response.snapped_destination)
        .map(|(a, b)| a.distance_to(b));
    let valid_geometry = response.ok
        && (2..=65_536).contains(&geometry_point_count)
        && response.geometry.iter().all(|p| {
            p.lat.is_finite()
                && p.lng.is_finite()
                && (-90.0..=90.0).contains(&p.lat)
                && (-180.0..=180.0).contains(&p.lng)
        })
        && response
            .distance_m
            .is_some_and(|d| d.is_finite() && d >= 0.0)
        && response
            .duration_s
            .is_some_and(|d| d.is_finite() && d >= 0.0)
        && response.cost.is_some_and(|d| d.is_finite() && d >= 0.0)
        && geometry_origin_gap_m.is_some_and(|d| d <= 0.1)
        && geometry_destination_gap_m.is_some_and(|d| d <= 0.1);
    // Geometry traverses the IPC boundary (as it would in the app), but the
    // compact persisted report retains its count and validity only.
    response.geometry.clear();
    response.maneuvers.clear();
    Sample {
        mode: mode.into(),
        label: case.label.clone(),
        iteration,
        origin: case.origin,
        destination: case.destination,
        profile: case.profile,
        latency_ms,
        geometry_point_count,
        geometry_length_m,
        geometry_distance_ratio,
        geometry_origin_gap_m,
        geometry_destination_gap_m,
        valid_geometry,
        same_as_cold_status,
        same_as_cold_edges,
        same_as_cold_distance,
        same_as_in_process_edges,
        same_as_in_process_status,
        remaining_distance_increase_m,
        response,
    }
}

/// Equidistant origins on a successful route, ending at 90% to avoid a trivial
/// identical-endpoint query. This measures full recomputation, not map matching.
fn advancing_cases(case: &Case, route: &Response, count: usize) -> Vec<Case> {
    if !route.ok || route.geometry.len() < 2 {
        return Vec::new();
    }
    let mut cumulative = vec![0.0];
    for pair in route.geometry.windows(2) {
        cumulative.push(cumulative.last().unwrap() + pair[0].distance_to(pair[1]));
    }
    let total = *cumulative.last().unwrap();
    (1..=count)
        .map(|step| {
            let target = total * 0.9 * step as f64 / count as f64;
            let end = cumulative
                .partition_point(|d| *d < target)
                .clamp(1, cumulative.len() - 1);
            let a = route.geometry[end - 1];
            let b = route.geometry[end];
            let segment = cumulative[end] - cumulative[end - 1];
            let fraction = if segment > 0.0 {
                (target - cumulative[end - 1]) / segment
            } else {
                0.0
            };
            Case {
                origin: [
                    a.lng + (b.lng - a.lng) * fraction,
                    a.lat + (b.lat - a.lat) * fraction,
                ],
                ..case.clone()
            }
        })
        .collect()
}

fn percentile(values: &mut [f64], percentile: f64) -> Option<f64> {
    if values.is_empty() {
        return None;
    }
    values.sort_by(f64::total_cmp);
    Some(values[((values.len() as f64 * percentile).ceil() as usize).saturating_sub(1)])
}

fn navigation_progress_bench(case: &Case, route: &Response) -> Value {
    let input = RouteNavigationInput {
        polyline6: None,
        geometry: route.geometry.clone(),
        maneuvers: route.maneuvers.clone(),
        shape_segments: Vec::new(),
        shape_segment_durations_seconds: Vec::new(),
        distance_meters: route.distance_m,
        duration_seconds: route.duration_s,
    };
    let mut times = Vec::new();
    let mut records = Vec::new();
    let mut candidate_since = None;
    for (step, case) in advancing_cases(case, route, 100).into_iter().enumerate() {
        let position = NavigationPositionSample {
            position: GeoPoint::new(case.origin[1], case.origin[0]),
            heading_degrees: None,
            speed_meters_per_second: None,
            sampled_at_ms: Some(step as i64 * 1000),
            accuracy_meters: Some(10.0),
            provider: Some("synthetic_on_route_replay".into()),
        };
        let start = Instant::now();
        let result = evaluate_route_navigation_progress(
            &input,
            position,
            candidate_since,
            RouteNavigationProgressOptions::default(),
        );
        let latency_ms = start.elapsed().as_secs_f64() * 1000.0;
        times.push(latency_ms);
        match result {
            Ok(progress) => {
                candidate_since = progress.off_route_candidate_since_ms;
                records.push(
                    json!({"step":step,"latency_ms":latency_ms,"status":progress.status,
                    "cross_track_error_m":progress.cross_track_error_meters,
                    "remaining_distance_m":progress.distance_remaining_meters,
                    "reroute_suggested":progress.reroute_suggested}),
                );
            }
            Err(error) => records.push(json!({"step":step,"latency_ms":latency_ms,"error":error})),
        }
    }
    json!({"label":case.label,"count":times.len(),"p50_ms":percentile(&mut times,0.5),
        "p95_ms":percentile(&mut times,0.95),"samples":records})
}

fn summaries<'a>(samples: impl IntoIterator<Item = &'a Sample>) -> Value {
    let mut groups = BTreeMap::<String, Vec<&Sample>>::new();
    for s in samples {
        groups.entry(s.mode.clone()).or_default().push(s);
    }
    Value::Object(groups.into_iter().map(|(mode, group)| {
        let mut latencies: Vec<_> = group.iter().map(|s| s.latency_ms).collect();
        let mut successful: Vec<_> = group.iter().filter(|s| s.response.ok).map(|s| s.latency_ms).collect();
        let summary = json!({
            "count":group.len(), "successes":successful.len(), "failures":group.len()-successful.len(),
            "p50_ms":percentile(&mut latencies, 0.50), "p95_ms":percentile(&mut latencies, 0.95),
            "successful_p50_ms":percentile(&mut successful,0.50), "successful_p95_ms":percentile(&mut successful,0.95),
            "under_100_ms":latencies.iter().filter(|v| **v<=100.0).count(),
            "under_1000_ms":latencies.iter().filter(|v| **v<=1000.0).count(),
            "invalid_success_geometry":group.iter().filter(|s|s.response.ok && !s.valid_geometry).count(),
            "edge_parity_mismatches":group.iter().filter(|s|s.same_as_cold_edges==Some(false) || s.same_as_in_process_edges==Some(false)).count(),
            "status_parity_mismatches":group.iter().filter(|s|s.same_as_cold_status==Some(false) || s.same_as_in_process_status==Some(false)).count(),
        });
        (mode, summary)
    }).collect())
}

fn bench(
    archive: &Path,
    corpus: &Path,
    output: &Path,
    repeats: usize,
    continuous: usize,
) -> Result<()> {
    let cases: Vec<Case> = serde_json::from_slice(&std::fs::read(corpus)?)?;
    if cases.is_empty() || repeats == 0 {
        return Err("corpus and repeat count must be nonempty".into());
    }
    for c in &cases {
        for [lng, lat] in [c.origin, c.destination] {
            if !lng.is_finite()
                || !lat.is_finite()
                || !(-180.0..=180.0).contains(&lng)
                || !(-90.0..=90.0).contains(&lat)
            {
                return Err(format!("invalid coordinates in {}", c.label).into());
            }
        }
    }
    let source = source(archive)?;
    let mut samples = Vec::new();
    let mut baseline = Vec::new();
    for case in &cases {
        eprintln!("cold_service {}", case.label);
        let start = Instant::now();
        let service = RoutingService::default();
        let response = compute(&service, &source, case);
        samples.push(sample(
            "cold_service",
            case,
            0,
            start.elapsed().as_secs_f64() * 1000.0,
            response.clone(),
            None,
            None,
            None,
        ));
        baseline.push(response);
    }
    let service = RoutingService::default();
    let baseline_export: Vec<_> = cases
        .iter()
        .zip(&baseline)
        .map(|(case, response)| json!({"request":case,"response":response}))
        .collect();
    std::fs::write(
        output.with_extension("baseline.json"),
        serde_json::to_vec_pretty(&baseline_export)?,
    )?;
    drop(baseline_export);
    // Share these exact coordinates with reference engines. Their source is
    // explicitly the Wayfarer baseline geometry, rather than independent traces.
    let continuous_corpus: Vec<_> = cases
        .iter()
        .zip(&baseline)
        .flat_map(|(case, base)| {
            advancing_cases(case, base, continuous)
                .into_iter()
                .enumerate()
                .map(|(step, mut c)| {
                    c.label = format!("{}::wayfarer_geometry_step_{step:03}", c.label);
                    c
                })
        })
        .collect();
    std::fs::write(
        output.with_extension("continuous.json"),
        serde_json::to_vec_pretty(&continuous_corpus)?,
    )?;
    let mut sidecar = Sidecar::new(archive)?;
    for iteration in 0..repeats {
        for (case, base) in cases.iter().zip(&baseline) {
            // Alternate order by repeat to reduce systematic OS-cache/order bias.
            for ipc in if iteration % 2 == 0 {
                [false, true]
            } else {
                [true, false]
            } {
                let mode = if ipc {
                    "persistent_sidecar"
                } else {
                    "persistent_service"
                };
                eprintln!("{mode} {} repeat {iteration}", case.label);
                let start = Instant::now();
                let response = if ipc {
                    sidecar.route_leg(case)?
                } else {
                    compute(&service, &source, case)
                };
                samples.push(sample(
                    mode,
                    case,
                    iteration,
                    start.elapsed().as_secs_f64() * 1000.0,
                    response,
                    Some(base),
                    None,
                    None,
                ));
            }
        }
    }
    for (case, base) in cases.iter().zip(&baseline) {
        let mut previous = [base.distance_m, base.distance_m];
        for (iteration, advancing) in advancing_cases(case, base, continuous).iter().enumerate() {
            eprintln!("continuous {} step {iteration}", case.label);
            let (local, local_ms, remote, remote_ms) = if iteration % 2 == 0 {
                let start = Instant::now();
                let local = compute(&service, &source, advancing);
                let local_ms = start.elapsed().as_secs_f64() * 1000.0;
                let start = Instant::now();
                let remote = sidecar.route_leg(advancing)?;
                (
                    local,
                    local_ms,
                    remote,
                    start.elapsed().as_secs_f64() * 1000.0,
                )
            } else {
                let start = Instant::now();
                let remote = sidecar.route_leg(advancing)?;
                let remote_ms = start.elapsed().as_secs_f64() * 1000.0;
                let start = Instant::now();
                let local = compute(&service, &source, advancing);
                (
                    local,
                    start.elapsed().as_secs_f64() * 1000.0,
                    remote,
                    remote_ms,
                )
            };
            samples.push(sample(
                "continuous_service",
                advancing,
                iteration,
                local_ms,
                local.clone(),
                None,
                previous[0],
                None,
            ));
            previous[0] = local.distance_m;
            let remote_distance = remote.distance_m;
            samples.push(sample(
                "continuous_sidecar",
                advancing,
                iteration,
                remote_ms,
                remote,
                None,
                previous[1],
                Some(&local),
            ));
            previous[1] = remote_distance;
        }
    }
    let readiness = service.inspect_source(&source);
    let navigation: Vec<_> = cases
        .iter()
        .zip(&baseline)
        .filter(|(_, b)| b.ok)
        .map(|(c, b)| navigation_progress_bench(c, b))
        .collect();
    let repeated_corpus = |s: &&Sample| s.mode.starts_with("persistent_") && s.iteration > 0;
    let repeat_summary_by_case: serde_json::Map<String, Value> = cases
        .iter()
        .map(|case| {
            (
                case.label.clone(),
                summaries(
                    samples
                        .iter()
                        .filter(repeated_corpus)
                        .filter(|s| s.label == case.label),
                ),
            )
        })
        .collect();
    let report = json!({
        "schema_version":2, "archive":archive, "archive_bytes":std::fs::metadata(archive)?.len(),
        "corpus":corpus, "repeats":repeats, "continuous_steps_per_successful_case":continuous,
        "sidecar_startup_ms":sidecar.startup_ms, "readiness":readiness,
        "policy":{"max_correlation_distance_m":150,"max_visited_edge_count":250000},
        "notes":["Cold means new RoutingService/reader; OS file cache is not flushed.",
            "Persistent modes include first archive open; per-sample source_cache_hit distinguishes it.",
            "Source-warm summary includes only samples whose source was already cached before preflight; it does not imply every needed tile is resident.",
            "Repeat summaries exclude the entire first corpus pass (iteration=0), so only requests previously computed in that persistent mode are included; tiles may still be evicted.",
            "Mode execution order alternates per corpus repetition and per continuous step, but the OS file cache remains shared.",
            "Sidecar latency includes JSON request/response serialization, pipe IPC and full geometry/diagnostics payload; startup is separate.",
            "Continuous origins advance evenly along each successful cold route to 90%; fixed destination, complete reroute, no GPS noise or incremental algorithm.",
            "Continuous cases whose cold baseline failed are skipped and remain visible as cold failures.",
            "Wheeled is Wayfarer's mobility policy, not an assertion of OSRM driving equivalence.",
            "Successful geometry validation and same-engine parity are sanity checks, not legal-road-access ground truth.",
            "Sequential requests; no concurrency/HTTP/network load or realtime scheduling is simulated."],
        "navigation_progress_note":"100 synthetic clean on-route samples per successful baseline at nominal 1s timestamps; geometry/maneuvers retained, no shape-segment attribution; progress updates do not recompute routes. Runtime is throughput, not a realtime paced replay.",
        "navigation_progress":navigation,
        "summary":summaries(&samples),
        "source_warm_summary":summaries(samples.iter().filter(|s|s.response.source_cache_hit)),
        "repeat_summary":summaries(samples.iter().filter(repeated_corpus)),
        "repeat_summary_by_case":repeat_summary_by_case,
        "samples":samples,
    });
    std::fs::write(output, serde_json::to_vec_pretty(&report)?)?;
    println!("{}", serde_json::to_string_pretty(&report["summary"])?);
    Ok(())
}

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("profile") if args.len()==7 => profile(Path::new(&args[2]),Path::new(&args[3]),Path::new(&args[4]),&args[5],args[6].parse()?),
        Some("serve") if args.len()==3 => serve(Path::new(&args[2])),
        Some("bench") if (5..=7).contains(&args.len()) => bench(Path::new(&args[2]), Path::new(&args[3]), Path::new(&args[4]),
            args.get(5).map(|s|s.parse()).transpose()?.unwrap_or(3),
            args.get(6).map(|s|s.parse()).transpose()?.unwrap_or(10)),
        _ => Err("usage: regional_benchmark bench <archive> <corpus.json> <output.json> [repeats=3] [continuous=10] | serve <archive> | profile <archive> <corpus.json> <output.json> <mobile|desktop|regional> <repeats>".into()),
    }
}

/// Diagnostic-only comparison of package memory profiles. Archive opening is
/// measured separately; no production defaults change.
fn profile(
    archive: &Path,
    corpus: &Path,
    output: &Path,
    cache: &str,
    repeats: usize,
) -> Result<()> {
    let cache_profile = match cache {
        "mobile" => RoutePackageMemoryProfile::Mobile,
        "desktop" | "regional" => RoutePackageMemoryProfile::Desktop,
        _ => return Err("unknown memory profile".into()),
    };
    let cases: Vec<Case> = serde_json::from_slice(&std::fs::read(corpus)?)?;
    if cases.is_empty() || repeats == 0 {
        return Err("empty corpus or repeat count".into());
    }
    let source = source(archive)?;
    let service = RoutingService::new(1, cache_profile);
    let open_started = Instant::now();
    let readiness = service.inspect_source(&source);
    let source_open_ms = open_started.elapsed().as_secs_f64() * 1000.0;
    let mut samples = Vec::new();
    let mut reader_snapshots = Vec::new();
    let mut baselines = Vec::new();
    for iteration in 0..repeats {
        for (case_index, case) in cases.iter().enumerate() {
            eprintln!("profile {} repeat {iteration}", case.label);
            let before = service
                .execute(&source, |_, package, _| package.byte_len())
                .ok();
            let started = Instant::now();
            let response = compute(&service, &source, case);
            let latency_ms = started.elapsed().as_secs_f64() * 1000.0;
            let after = service
                .execute(&source, |_, package, _| package.byte_len())
                .ok();
            samples.push(sample(
                "profile",
                case,
                iteration,
                latency_ms,
                response.clone(),
                baselines.get(case_index),
                None,
                None,
            ));
            if iteration == 0 {
                baselines.push(response);
            }
            reader_snapshots.push(
                json!({"label":case.label,"iteration":iteration,"before":before,"after":after}),
            );
        }
    }
    let report = json!({"schema_version":1,"kind":"cache_profile_diagnostic",
        "archive":archive,"cache_profile":cache_profile,"repeats":repeats,
        "source_open_ms":source_open_ms,"readiness":readiness,
        "notes":["One retained RoutingService and one immutable source; reader snapshots are outside query timing.",
            "Source opening is measured separately before first request. OS page cache is not flushed.",
            "All default policies are unchanged, except the explicitly selected memory profile.",
            "Queries retain the native150m correlation radius and250000visited-edge budget. Errors stay in all-sample summaries.",
            "Reader snapshots record the opened package's size before and after each request.",
            "Same-cold comparisons here refer to the first pass of this variant; external cross-variant parity must be assessed separately.",
            "Cache byte limits count serialized payload size and may be exceeded while request references pin tiles; they are not RSS limits."],
        "summary":summaries(&samples),"repeat_summary":summaries(samples.iter().filter(|s|s.iteration>0)),
        "samples":samples,"reader_snapshots":reader_snapshots});
    std::fs::write(output, serde_json::to_vec_pretty(&report)?)?;
    println!(
        "{}",
        serde_json::to_string_pretty(&report["repeat_summary"])?
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> (Case, Response) {
        let archive =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/routing/short_road.mrt");
        let case = Case {
            label: "fixture".into(),
            origin: [21.00005, 52.0],
            destination: [21.00195, 52.0],
            profile: MobilityProfile::Wheeled,
        };
        let response = compute(
            &RoutingService::default(),
            &source(&archive).unwrap(),
            &case,
        );
        assert!(response.ok, "{:?}", response.error);
        (case, response)
    }

    #[test]
    fn nearest_rank_percentiles_handle_empty_and_tail() {
        assert_eq!(percentile(&mut [], 0.95), None);
        assert_eq!(percentile(&mut [4.0, 1.0, 3.0, 2.0], 0.5), Some(2.0));
        assert_eq!(percentile(&mut [4.0, 1.0, 3.0, 2.0], 0.95), Some(4.0));
    }

    #[test]
    fn geometry_validation_rejects_endpoint_corruption() {
        let (case, mut response) = fixture();
        // Test the validator against an explicitly aligned response. The real
        // archive's endpoint agreement is measured independently by bench.
        response.geometry[0] = response.snapped_origin.unwrap();
        *response.geometry.last_mut().unwrap() = response.snapped_destination.unwrap();
        assert!(
            sample("fixture", &case, 0, 0.0, response.clone(), None, None, None).valid_geometry
        );
        response.geometry[0].lat += 1.0;
        assert!(!sample("fixture", &case, 0, 0.0, response, None, None, None).valid_geometry);
    }

    #[test]
    fn advancing_origins_progress_and_failed_baselines_are_skipped() {
        let (case, mut response) = fixture();
        let advances = advancing_cases(&case, &response, 3);
        assert_eq!(advances.len(), 3);
        assert!(advances.windows(2).all(|p| p[0].origin[0] < p[1].origin[0]));
        assert!(advances[2].origin[0] < case.destination[0]);
        response.ok = false;
        assert!(advancing_cases(&case, &response, 3).is_empty());
    }
}

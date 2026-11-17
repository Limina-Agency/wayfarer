//! PERF-2 spike harness: route latency per request class for each arm
//! (lower bound × alternatives method), alternating arms within one process.
//!
//! ```sh
//! cargo run --release -p wayfarer --example perf2_bench -- \
//!   <routing.mrt> <corpus.json> [random_pairs=200] [repeats=3] [rounds=2] \
//!   [arms=alt6,alt16,cch] [out.jsonl]
//! ```
//!
//! An arm is `<potential>[:<alternatives>]`: potential `alt6` (today),
//! `alt16` (every landmark for vehicles) or `cch`; alternatives `penalty`
//! (today, default), or `+`-joined `budget<k>`, `bound`, `recustom` (see
//! `wayfarer::debug`). Arms with the default alternatives run every class;
//! the others run only the alternatives class.
//!
//! Classes: the corpus (`corpus` drives, `corpus_walk`), the corpus again
//! with 2 alternatives (`corpus_alt2`), and `random_pairs` seeded random pairs
//! in the corpus bounding box (`random_drive`, every 4th `random_walk`).
//! Requests as `route_response_dump`: 150 m correlation, 2 M visited-edge
//! budget. Each round runs every arm in turn (A B C A B C ...), each case
//! `repeats` times back to back; a case's time is its minimum over every run
//! of every round, and p50/p95/p99 are over cases. Each round starts with a
//! fixed CPU calibration loop, so contention from other processes shows up
//! as a slower calibration.
//!
//! Primary costs are compared with the first arm (a mismatch is a relative
//! difference above 1e-9); alternatives with the first arm's of the same
//! class: accepted count, mean cost ratio and shared ratio, identical sets.

use std::collections::BTreeMap;
use std::io::Write;
use std::sync::Arc;
use std::time::Instant;

use serde_json::{Value, json};
use wayfarer::internals::perf2;
use wayfarer::{GeoPoint, LegRequest, MobilityProfile, RoutingEngine, RoutingPackage};

#[derive(Clone)]
struct Case {
    label: String,
    kind: &'static str,
    origin: GeoPoint,
    destination: GeoPoint,
    profile: MobilityProfile,
    alternatives: usize,
}

#[derive(Clone, Default)]
struct Outcome {
    ok: bool,
    cost: f64,
    visited: usize,
    search_ms: f64,
    leg_ms: f64,
    /// Per alternative: (cost ratio to the primary, shared ratio, edge hash).
    alternatives: Vec<(f64, f64, u64)>,
    attempts: usize,
}

fn quantile(values: &[f64], q: f64) -> f64 {
    let mut v = values.to_vec();
    v.sort_by(f64::total_cmp);
    if v.is_empty() {
        return f64::NAN;
    }
    let index = ((v.len() as f64 * q).ceil() as usize).saturating_sub(1);
    v[index.min(v.len() - 1)]
}

fn load_corpus(path: &str) -> Vec<Case> {
    let corpus: Vec<Value> =
        serde_json::from_str(&std::fs::read_to_string(path).expect("corpus")).expect("json");
    corpus
        .iter()
        .map(|entry| {
            let point = |key: &str| {
                let pair = entry[key].as_array().expect("point");
                GeoPoint::new(pair[1].as_f64().unwrap(), pair[0].as_f64().unwrap())
            };
            let walk = matches!(entry["profile"].as_str(), Some("on_foot" | "walking"));
            Case {
                label: entry["label"].as_str().unwrap_or("case").to_string(),
                kind: if walk { "corpus_walk" } else { "corpus" },
                origin: point("origin"),
                destination: point("destination"),
                profile: if walk {
                    MobilityProfile::OnFoot
                } else {
                    MobilityProfile::Wheeled
                },
                alternatives: 0,
            }
        })
        .collect()
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

fn request(case: &Case) -> LegRequest {
    let mut request = LegRequest::new(case.origin, case.destination, case.profile);
    request.options.max_correlation_distance_m = 150;
    request.options.max_visited_edge_count = Some(2_000_000);
    request.options.alternatives.requested_alternative_count = case.alternatives;
    request
}

fn edge_hash(edges: &[u64]) -> u64 {
    edges.iter().fold(0xcbf2_9ce4_8422_2325u64, |hash, edge| {
        (hash ^ edge).wrapping_mul(0x0100_0000_01b3)
    })
}

fn run(engine: &RoutingEngine, case: &Case) -> Outcome {
    match engine.route_leg(request(case)) {
        Ok(result) => Outcome {
            ok: true,
            cost: result.cost,
            visited: result.diagnostics.visited_edge_count,
            search_ms: result.diagnostics.search_duration_ns as f64 / 1e6,
            leg_ms: result.diagnostics.route_leg_build_duration_ns as f64 / 1e6,
            alternatives: result
                .alternatives
                .iter()
                .map(|alternative| {
                    (
                        alternative.cost / result.cost.max(f64::EPSILON),
                        alternative.shared_edge_ratio,
                        edge_hash(&alternative.edge_ids),
                    )
                })
                .collect(),
            attempts: result.diagnostics.alternative_attempt_count,
        },
        Err(failure) => Outcome {
            ok: false,
            visited: failure.diagnostics.visited_edge_count,
            ..Default::default()
        },
    }
}

/// A fixed integer workload (ms): slower under contention.
fn calibrate() -> f64 {
    let started = Instant::now();
    let mut x = 0x9E37_79B9_7F4A_7C15u64;
    let mut acc = 0u64;
    for _ in 0..60_000_000u64 {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        acc = acc.wrapping_add(x >> 7);
    }
    std::hint::black_box(acc);
    started.elapsed().as_secs_f64() * 1e3
}

fn set_arm(arm: &str) -> (String, String) {
    let (potential, alternatives) = arm.split_once(':').unwrap_or((arm, "penalty"));
    assert!(perf2::set_potential(potential), "unknown potential {potential}");
    assert!(
        perf2::set_alternatives(alternatives),
        "unknown alternatives mode {alternatives}"
    );
    (potential.to_string(), alternatives.to_string())
}

fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    let package_path = args.get(1).expect("package path");
    let corpus_path = args.get(2).expect("corpus path");
    let random_pairs: usize = args.get(3).and_then(|v| v.parse().ok()).unwrap_or(200);
    let repeats: usize = args.get(4).and_then(|v| v.parse().ok()).unwrap_or(3);
    let rounds: usize = args.get(5).and_then(|v| v.parse().ok()).unwrap_or(2);
    let arms = args
        .get(6)
        .map(String::as_str)
        .unwrap_or("alt6,alt16,cch")
        .split(',')
        .map(str::to_string)
        .collect::<Vec<_>>();
    let out_path = args.get(7).cloned();
    let skip_plain = std::env::var_os("PERF2_ONLY_ALT").is_some();
    let skip_alt = std::env::var_os("PERF2_SKIP_ALT").is_some();

    let started = Instant::now();
    let package = RoutingPackage::open(std::path::Path::new(package_path)).expect("open");
    let open_ms = started.elapsed().as_secs_f64() * 1e3;
    let started = Instant::now();
    package.prefetch();
    let prefetch_ms = started.elapsed().as_secs_f64() * 1e3;
    println!(
        "package {} nodes {} edges {} bytes {} open {open_ms:.1} ms prefetch {prefetch_ms:.1} ms threads {}",
        package_path,
        package.counts().node_count,
        package.counts().edge_count,
        package.byte_len(),
        std::thread::available_parallelism().map_or(0, |n| n.get())
    );
    if arms.iter().any(|arm| arm.starts_with("cch")) {
        let started = Instant::now();
        let (stats, times) = package.perf2_prepare_cch();
        println!(
            "cch prepared in {:.1} ms: {stats:?}",
            started.elapsed().as_secs_f64() * 1e3
        );
        for (column, parallel, single) in times {
            println!(
                "cch customize column {column}: parallel {parallel:.1} ms, single thread {single:.1} ms"
            );
        }
        if std::env::var_os("PERF2_PHAST").is_some() {
            for column in 0..2 {
                let (ms, mismatches) = package.perf2_phast(column, 20);
                println!(
                    "cch phast column {column}: {ms:.2} ms per one-to-all sweep, {mismatches} mismatches against Dijkstra"
                );
            }
        }
        if let Some(samples) = std::env::var("PERF2_CHECK")
            .ok()
            .and_then(|value| value.parse::<usize>().ok())
        {
            for column in 0..2 {
                let started = Instant::now();
                let (pairs, mismatches, worst) = package.perf2_check_cch(column, samples);
                println!(
                    "cch exactness column {column}: {pairs} pairs, {mismatches} mismatches, worst relative error {worst:.3e} ({:.0} ms)",
                    started.elapsed().as_secs_f64() * 1e3
                );
            }
        }
    }
    let engine = RoutingEngine::from_package(Arc::new(package));

    let corpus = load_corpus(corpus_path);
    let (mut min_lat, mut max_lat, mut min_lng, mut max_lng) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for case in &corpus {
        for point in [case.origin, case.destination] {
            min_lat = min_lat.min(point.lat);
            max_lat = max_lat.max(point.lat);
            min_lng = min_lng.min(point.lng);
            max_lng = max_lng.max(point.lng);
        }
    }
    let mut plain = corpus.clone();
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    for index in 0..random_pairs {
        let origin = GeoPoint::new(
            min_lat + (max_lat - min_lat) * rng.next(),
            min_lng + (max_lng - min_lng) * rng.next(),
        );
        let destination = GeoPoint::new(
            min_lat + (max_lat - min_lat) * rng.next(),
            min_lng + (max_lng - min_lng) * rng.next(),
        );
        let walk = index % 4 == 3;
        plain.push(Case {
            label: format!("random_{index:03}"),
            kind: if walk { "random_walk" } else { "random_drive" },
            origin,
            destination,
            profile: if walk {
                MobilityProfile::OnFoot
            } else {
                MobilityProfile::Wheeled
            },
            alternatives: 0,
        });
    }
    let alt_cases = corpus
        .iter()
        .map(|case| Case {
            label: format!("{}_alt2", case.label),
            kind: "corpus_alt2",
            alternatives: 2,
            ..case.clone()
        })
        .collect::<Vec<_>>();

    // (arm, case label) -> (min ms, last outcome)
    let mut times = BTreeMap::<(String, String), (f64, Outcome)>::new();
    let mut kinds = BTreeMap::<String, &'static str>::new();
    let mut calibration = Vec::new();
    for case in plain.iter().chain(&alt_cases) {
        kinds.insert(case.label.clone(), case.kind);
    }
    // Warm-up: every arm once over every case it runs.
    for arm in &arms {
        let (_, alternatives) = set_arm(arm);
        let plain_arm = alternatives == "penalty";
        for case in plain.iter().filter(|_| plain_arm && !skip_plain) {
            let _ = run(&engine, case);
        }
        for case in alt_cases.iter().filter(|_| !skip_alt) {
            let _ = run(&engine, case);
        }
        perf2::take_counters();
    }
    let mut counters = BTreeMap::<String, Vec<(&'static str, u64)>>::new();
    for round in 0..rounds {
        let calibration_ms = calibrate();
        calibration.push(calibration_ms);
        println!("round {round}: calibration {calibration_ms:.1} ms");
        for arm in &arms {
            let (_, alternatives) = set_arm(arm);
            let plain_arm = alternatives == "penalty";
            let arm_started = Instant::now();
            let cases = plain
                .iter()
                .filter(|_| plain_arm && !skip_plain)
                .chain(alt_cases.iter().filter(|_| !skip_alt));
            for case in cases {
                for _ in 0..repeats {
                    let started = Instant::now();
                    let outcome = run(&engine, case);
                    let ms = started.elapsed().as_secs_f64() * 1e3;
                    let entry = times
                        .entry((arm.clone(), case.label.clone()))
                        .or_insert((f64::INFINITY, Outcome::default()));
                    if ms < entry.0 {
                        entry.0 = ms;
                    }
                    entry.1 = outcome;
                }
            }
            let taken = perf2::take_counters();
            if round == 0 {
                counters.insert(arm.clone(), taken);
            }
            println!(
                "  arm {arm}: {:.1} s",
                arm_started.elapsed().as_secs_f64()
            );
        }
    }

    let mut out = out_path.map(|path| std::io::BufWriter::new(std::fs::File::create(path).expect("out")));
    if let Some(out) = &mut out {
        for ((arm, label), (ms, outcome)) in &times {
            let row = json!({
                "arm": arm, "label": label, "kind": kinds[label], "ok": outcome.ok,
                "min_ms": ms, "cost": outcome.cost, "visited": outcome.visited,
                "search_ms": outcome.search_ms, "leg_ms": outcome.leg_ms,
                "attempts": outcome.attempts,
                "alternatives": outcome.alternatives.iter().map(|(c, s, h)| json!([c, s, format!("{h:016x}")])).collect::<Vec<_>>(),
            });
            writeln!(out, "{row}").unwrap();
        }
        out.flush().unwrap();
    }

    println!(
        "\ncalibration ms per round: {:?} (quiet machine ~{:.0})",
        calibration.iter().map(|v| (v * 10.0).round() / 10.0).collect::<Vec<_>>(),
        calibration.iter().copied().fold(f64::INFINITY, f64::min)
    );
    let baseline = &arms[0];
    let classes = ["corpus", "random_drive", "drives", "corpus_walk", "random_walk", "walks", "corpus_alt2"];
    println!(
        "\n{:<22} {:<13} {:>4} {:>8} {:>8} {:>8} {:>8} {:>9} {:>9} {:>6}",
        "arm", "class", "n", "p50", "p95", "p99", "max", "visit p50", "visit p95", "costΔ"
    );
    for arm in &arms {
        for class in classes {
            let rows = times
                .iter()
                .filter(|((a, label), (_, outcome))| {
                    a == arm
                        && outcome.ok
                        && match class {
                            "drives" => matches!(kinds[label], "corpus" | "random_drive"),
                            "walks" => matches!(kinds[label], "corpus_walk" | "random_walk"),
                            other => kinds[label] == other,
                        }
                })
                .collect::<Vec<_>>();
            if rows.is_empty() {
                continue;
            }
            let ms = rows.iter().map(|(_, (ms, _))| *ms).collect::<Vec<_>>();
            let visited = rows.iter().map(|(_, (_, o))| o.visited as f64).collect::<Vec<_>>();
            let mismatches = rows
                .iter()
                .filter(|((_, label), (_, outcome))| {
                    let Some((_, base)) = times.get(&(baseline.clone(), label.clone())) else {
                        return false;
                    };
                    base.ok
                        && (outcome.cost - base.cost).abs() > 1e-9 * base.cost.abs().max(1.0)
                })
                .count();
            println!(
                "{:<22} {:<13} {:>4} {:>8.2} {:>8.2} {:>8.2} {:>8.2} {:>9.0} {:>9.0} {:>6}",
                arm,
                class,
                rows.len(),
                quantile(&ms, 0.5),
                quantile(&ms, 0.95),
                quantile(&ms, 0.99),
                quantile(&ms, 1.0),
                quantile(&visited, 0.5),
                quantile(&visited, 0.95),
                mismatches
            );
        }
    }
    // Alternatives quality per arm.
    let alt_baseline = arms
        .iter()
        .find(|arm| !arm.contains(':'))
        .unwrap_or(&arms[0])
        .clone();
    println!(
        "\nalternatives (corpus_alt2; identical = same accepted set as {alt_baseline}):"
    );
    println!(
        "{:<22} {:>5} {:>8} {:>10} {:>11} {:>9} {:>9}",
        "arm", "cases", "accepted", "cost ratio", "shared mean", "identical", "attempts"
    );
    for arm in &arms {
        let rows = times
            .iter()
            .filter(|((a, label), _)| a == arm && kinds[label] == "corpus_alt2")
            .collect::<Vec<_>>();
        if rows.is_empty() {
            continue;
        }
        let alternatives = rows
            .iter()
            .flat_map(|(_, (_, o))| o.alternatives.iter())
            .collect::<Vec<_>>();
        let identical = rows
            .iter()
            .filter(|((_, label), (_, outcome))| {
                times
                    .get(&(alt_baseline.clone(), label.clone()))
                    .is_some_and(|(_, base)| {
                        let set = |o: &Outcome| {
                            let mut hashes = o.alternatives.iter().map(|a| a.2).collect::<Vec<_>>();
                            hashes.sort_unstable();
                            hashes
                        };
                        set(base) == set(outcome)
                    })
            })
            .count();
        let mean = |values: Vec<f64>| values.iter().sum::<f64>() / values.len().max(1) as f64;
        println!(
            "{:<22} {:>5} {:>8} {:>10.3} {:>11.3} {:>9} {:>9}",
            arm,
            rows.len(),
            alternatives.len(),
            mean(alternatives.iter().map(|a| a.0).collect()),
            mean(alternatives.iter().map(|a| a.1).collect()),
            identical,
            rows.iter().map(|(_, (_, o))| o.attempts).sum::<usize>(),
        );
    }
    println!("\ncounters (round 0, per arm): ");
    for (arm, taken) in &counters {
        println!(
            "  {arm}: {}",
            taken
                .iter()
                .filter(|(_, value)| *value > 0)
                .map(|(name, value)| format!("{name}={value}"))
                .collect::<Vec<_>>()
                .join(" ")
        );
    }
}

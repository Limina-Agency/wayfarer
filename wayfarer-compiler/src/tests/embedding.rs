//! The compile as an app runs it in its own process: progress, cancel in
//! every stage, cleanup, the caller's thread count, the request's JSON and
//! the polled job.

use super::*;
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

/// A fixture extract (a junction with a turn restriction) in its own
/// directory, and the request compiling it into that directory.
fn fixture(dir: &std::path::Path, threads: Option<usize>) -> CompileRequest {
    let pbf = dir.join("fixture.osm.pbf");
    write_osm_pbf(
        &pbf,
        JUNCTION_NODES,
        JUNCTION_WAYS,
        &[FixtureRelation {
            id: 100,
            tags: &[("type", "restriction"), ("restriction", "no_left_turn")],
            members: &[("from", WAY, 1), ("via", NODE, 10), ("to", WAY, 3)],
        }],
    );
    let mut request = CompileRequest::new(pbf, dir.join("out").join("routing.mrt"))
        .scratch_dir(dir.join("scratch"))
        .graph_id("junction")
        .memory(MemoryProfile::Low);
    request.threads = threads;
    request
}

/// Every file and directory under `dir`, relative to it with `/` between
/// components on every platform, sorted.
fn tree(dir: &std::path::Path) -> Vec<String> {
    fn walk(root: &std::path::Path, dir: &std::path::Path, out: &mut Vec<String>) {
        for entry in std::fs::read_dir(dir).unwrap() {
            let path = entry.unwrap().path();
            let relative = path.strip_prefix(root).unwrap().components();
            out.push(
                relative
                    .map(|part| part.as_os_str().to_string_lossy())
                    .collect::<Vec<_>>()
                    .join("/"),
            );
            if path.is_dir() {
                walk(root, &path, out);
            }
        }
    }
    let mut out = Vec::new();
    walk(dir, dir, &mut out);
    out.sort();
    out
}

fn compile_collecting(
    request: &CompileRequest,
    should_cancel: impl Fn() -> bool + Sync,
) -> (Result<CompileResult, CompileError>, Vec<CompileProgress>) {
    let reports = Mutex::new(Vec::new());
    let result = api::compile_reporting(request, &should_cancel, &mut |report| {
        reports.lock().unwrap().push(CompileProgress::of(report))
    });
    (result, reports.into_inner().unwrap())
}
#[test]
fn progress_runs_through_every_stage_in_order_and_ends_complete() {
    let dir = tempfile::tempdir().unwrap();
    let request = fixture(dir.path(), Some(2));
    let (result, reports) = compile_collecting(&request, || false);
    result.unwrap();

    let index = |stage| CompileStage::ALL.iter().position(|s| *s == stage);
    let mut seen = Vec::new();
    for pair in reports.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        assert!(index(a.stage) <= index(b.stage), "{a:?} then {b:?}");
        assert!(
            a.overall <= b.overall,
            "overall went back: {a:?} then {b:?}"
        );
        if a.stage == b.stage
            && let (Some(x), Some(y)) = (a.stage_fraction, b.stage_fraction)
        {
            assert!(x <= y, "fraction went back: {a:?} then {b:?}");
        }
    }
    for report in &reports {
        assert!((0.0..=1.0).contains(&report.overall), "{report:?}");
        if let Some(fraction) = report.stage_fraction {
            assert!((0.0..=1.0).contains(&fraction), "{report:?}");
        }
        if !seen.contains(&report.stage) {
            seen.push(report.stage);
        }
    }
    assert_eq!(seen, CompileStage::ALL);
    // Every stage reaches its end.
    for stage in CompileStage::ALL {
        assert!(
            reports
                .iter()
                .any(|report| report.stage == stage && report.stage_fraction == Some(1.0)),
            "{stage:?} never reported completion"
        );
    }
    let last = reports.last().unwrap();
    assert_eq!(last.stage, CompileStage::WritePackage);
    assert_eq!(last.stage_fraction, Some(1.0));
    assert_eq!(last.overall, 1.0);
    // The package and the extract are all that is left.
    assert_eq!(
        tree(dir.path()),
        ["fixture.osm.pbf", "out", "out/routing.mrt"]
    );
}

/// Cancels when the first report of `stage` arrives: the compile stops in
/// that stage, says it was cancelled and leaves only the extract.
fn cancel_in(stage: CompileStage) {
    let dir = tempfile::tempdir().unwrap();
    let request = fixture(dir.path(), Some(2));
    let cancel = CancelToken::new();
    let reports = Mutex::new(Vec::new());
    let result = compile(&request, &cancel, |progress: &CompileProgress| {
        if progress.stage == stage {
            cancel.cancel();
        }
        reports.lock().unwrap().push(progress.stage);
    });
    assert_eq!(result.unwrap_err(), CompileError::Cancelled, "{stage:?}");
    let reports = reports.into_inner().unwrap();
    assert_eq!(
        reports.last(),
        Some(&stage),
        "a later stage ran: {reports:?}"
    );
    assert_eq!(tree(dir.path()), ["fixture.osm.pbf"], "{stage:?}");
}

#[test]
fn a_cancel_while_locating_nodes_stops_the_compile_and_leaves_nothing() {
    cancel_in(CompileStage::IndexNodes);
}

#[test]
fn a_cancel_while_collecting_roads_stops_the_compile_and_leaves_nothing() {
    cancel_in(CompileStage::CollectRoads);
}

#[test]
fn a_cancel_while_building_the_graph_stops_the_compile_and_leaves_nothing() {
    cancel_in(CompileStage::FinalizeGraph);
}

#[test]
fn a_cancel_while_building_the_package_stops_the_compile_and_leaves_nothing() {
    cancel_in(CompileStage::BuildPackage);
}

#[test]
fn a_cancel_while_ordering_nodes_stops_the_compile_and_leaves_nothing() {
    cancel_in(CompileStage::CchOrder);
}

#[test]
fn a_cancel_while_writing_stops_the_compile_and_leaves_nothing() {
    cancel_in(CompileStage::WritePackage);
}

/// A cancel at any of the compile's polls of `should_cancel` (not only at a
/// stage's start) ends it cancelled, with nothing left behind.
#[test]
fn a_cancel_at_any_poll_leaves_nothing_behind() {
    let dir = tempfile::tempdir().unwrap();
    let polls = AtomicUsize::new(0);
    let request = fixture(dir.path(), Some(2));
    let (result, _) = compile_collecting(&request, || {
        polls.fetch_add(1, Ordering::Relaxed);
        false
    });
    result.unwrap();
    let total = polls.load(Ordering::Relaxed);
    assert!(total > 10, "{total} polls");

    let step = (total / 40).max(1);
    for cancel_at in (0..total).step_by(step) {
        let dir = tempfile::tempdir().unwrap();
        let request = fixture(dir.path(), Some(2));
        let polls = AtomicUsize::new(0);
        let (result, _) = compile_collecting(&request, || {
            polls.fetch_add(1, Ordering::Relaxed) >= cancel_at
        });
        // The ingest polls while it waits for blobs, so a run can poll fewer
        // times than the first and finish before `cancel_at`.
        if result.is_ok() {
            assert!(polls.load(Ordering::Relaxed) <= cancel_at);
            continue;
        }
        assert_eq!(
            result.unwrap_err(),
            CompileError::Cancelled,
            "cancel at poll {cancel_at} of {total}"
        );
        assert_eq!(
            tree(dir.path()),
            ["fixture.osm.pbf"],
            "cancel at poll {cancel_at} of {total}"
        );
    }
}

#[test]
fn a_cancelled_compile_keeps_the_package_it_would_have_replaced() {
    let dir = tempfile::tempdir().unwrap();
    let request = fixture(dir.path(), None);
    std::fs::create_dir_all(dir.path().join("out")).unwrap();
    std::fs::write(&request.output, b"the previous package").unwrap();
    let cancel = CancelToken::new();
    let result = compile(&request, &cancel, |progress: &CompileProgress| {
        if progress.stage == CompileStage::WritePackage {
            cancel.cancel();
        }
    });
    let error = result.unwrap_err();
    assert!(error.is_cancelled());
    assert_eq!(error.code(), "cancelled");
    assert_eq!(
        std::fs::read(&request.output).unwrap(),
        b"the previous package"
    );
    assert_eq!(
        tree(dir.path()),
        ["fixture.osm.pbf", "out", "out/routing.mrt"]
    );
}

#[test]
fn a_failed_compile_is_not_a_cancel_and_leaves_nothing() {
    let dir = tempfile::tempdir().unwrap();
    let mut request = fixture(dir.path(), Some(1));
    request.input = dir.path().join("missing.osm.pbf");
    let (result, _) = compile_collecting(&request, || false);
    let error = result.unwrap_err();
    assert!(matches!(error, CompileError::Failed(_)), "{error:?}");
    assert!(!error.is_cancelled());
    assert_eq!(error.code(), "failed");
    assert_eq!(tree(dir.path()), ["fixture.osm.pbf"]);
}

#[test]
fn an_invalid_request_starts_nothing() {
    let dir = tempfile::tempdir().unwrap();
    let request = fixture(dir.path(), Some(1));
    for invalid in [
        request.clone().threads(0),
        request.clone().graph_id(""),
        request.clone().country(""),
        CompileRequest::new(&request.input, &request.input),
        CompileRequest::new(&request.input, dir.path().join("..")),
    ] {
        let error = compile(&invalid, &CancelToken::new(), |_| {}).unwrap_err();
        assert!(
            matches!(error, CompileError::InvalidRequest(_)),
            "{invalid:?}: {error:?}"
        );
        assert_eq!(error.code(), "invalid_request");
        assert_eq!(tree(dir.path()), ["fixture.osm.pbf"]);
    }
    let json = serde_json::to_value(CompileError::InvalidRequest("no thread".into())).unwrap();
    assert_eq!(
        json,
        serde_json::json!({"code": "invalid_request", "message": "invalid compile request: no thread"})
    );
}

/// With no scratch directory the compile works next to the output, and a
/// missing output directory it made goes again when it fails.
#[test]
fn the_default_scratch_sits_next_to_the_output_and_goes_whatever_the_outcome() {
    let dir = tempfile::tempdir().unwrap();
    let mut request = fixture(dir.path(), Some(2));
    request.scratch_dir = None;
    request.output = dir.path().join("a").join("b").join("routing.mrt");
    let cancel = CancelToken::new();
    let result = compile(&request, &cancel, |progress: &CompileProgress| {
        if progress.stage == CompileStage::CchOrder {
            cancel.cancel();
        }
    });
    assert!(result.unwrap_err().is_cancelled());
    assert_eq!(tree(dir.path()), ["fixture.osm.pbf"]);

    let result = compile(&request, &CancelToken::new(), |_| {}).unwrap();
    assert_eq!(result.package, request.output);
    assert_eq!(result.graph_id, "junction");
    assert_eq!(
        tree(dir.path()),
        ["a", "a/b", "a/b/routing.mrt", "fixture.osm.pbf"]
    );
}

#[test]
fn the_result_describes_the_package_written() {
    let dir = tempfile::tempdir().unwrap();
    let mut request = fixture(dir.path(), Some(2)).country("PL");
    request.graph_id = None;
    let result = compile(&request, &CancelToken::new(), |_| {}).unwrap();
    let package = wayfarer_package::format::PackageFile::open(&request.output).unwrap();
    assert_eq!(result.graph_id, "fixture");
    assert_eq!(result.country.as_deref(), Some("PL"));
    assert_eq!(result.byte_len, package.byte_len() as u64);
    assert_eq!(result.dataset_hash, package.dataset_hash());
    assert_eq!(result.format_version, package.format_version());
    assert_eq!(
        result.costing_model_version,
        package.metadata().costing_model_version
    );
    assert_eq!(result.node_count, package.counts().node_count as u64);
    assert_eq!(result.edge_count, package.counts().edge_count as u64);
    assert_eq!(result.report.byte_len as u64, result.byte_len);
    assert_eq!(result.graph_manifest.raw_turn_restriction_count, 1);
    let json = serde_json::to_value(&result).unwrap();
    assert_eq!(json["datasetHash"], result.dataset_hash.as_str());
    assert_eq!(json["graphId"], "fixture");
    assert!(json["report"]["cchOrderMs"].is_u64(), "{json}");
    assert!(json.get("graphManifest").is_none());
}

#[test]
fn the_request_json_is_camel_case_with_defaults_and_rejects_unknown_fields() {
    let request: CompileRequest = serde_json::from_str(
        r#"{"input": "monaco.osm.pbf", "output": "out/routing.mrt", "scratchDir": "work",
            "graphId": "mc", "country": "MC", "memory": "low", "threads": 2}"#,
    )
    .unwrap();
    assert_eq!(
        request,
        CompileRequest::new("monaco.osm.pbf", "out/routing.mrt")
            .scratch_dir("work")
            .graph_id("mc")
            .country("MC")
            .memory(MemoryProfile::Low)
            .threads(2)
    );
    let minimal: CompileRequest =
        serde_json::from_str(r#"{"input": "monaco.osm.pbf", "output": "routing.mrt"}"#).unwrap();
    assert_eq!(
        minimal,
        CompileRequest::new("monaco.osm.pbf", "routing.mrt")
    );
    assert_eq!(minimal.memory, MemoryProfile::Standard);
    assert_eq!(minimal.resolved_graph_id(), "monaco");
    assert_eq!(
        serde_json::to_string(&minimal).unwrap(),
        r#"{"input":"monaco.osm.pbf","output":"routing.mrt","memory":"standard"}"#
    );
    assert!(
        serde_json::from_str::<CompileRequest>(
            r#"{"input": "a.osm.pbf", "output": "b.mrt", "mobile": true}"#
        )
        .is_err()
    );
    assert!(
        serde_json::from_str::<CompileRequest>(
            r#"{"input": "a.osm.pbf", "output": "b.mrt", "memory": "huge"}"#
        )
        .is_err()
    );
}

#[test]
fn the_compile_runs_on_a_pool_of_the_callers_size_with_the_same_bytes() {
    let mut packages = Vec::new();
    for threads in [Some(1), Some(3), None] {
        for memory in [MemoryProfile::Standard, MemoryProfile::Low] {
            let dir = tempfile::tempdir().unwrap();
            let request = fixture(dir.path(), threads).memory(memory);
            let pool_sizes = Mutex::new(Vec::new());
            compile(&request, &CancelToken::new(), |_| {
                pool_sizes
                    .lock()
                    .unwrap()
                    .push(rayon::current_num_threads())
            })
            .unwrap();
            if let Some(threads) = threads {
                let sizes = pool_sizes.into_inner().unwrap();
                assert!(sizes.iter().all(|size| *size == threads), "{sizes:?}");
            }
            packages.push(std::fs::read(&request.output).unwrap());
        }
    }
    assert!(packages.windows(2).all(|pair| pair[0] == pair[1]));
}

fn monaco_request(dir: &std::path::Path) -> CompileRequest {
    let pbf = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/monaco.osm.pbf");
    CompileRequest::new(pbf, dir.join("out").join("routing.mrt"))
        .country("MC")
        .threads(2)
}

/// Polls `job` until `done` holds for its progress or it finishes.
fn poll_until(job: &CompileJob, done: impl Fn(&CompileProgress) -> bool) {
    while !job.is_finished() && !done(&job.progress()) {
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
}

#[test]
fn a_compile_job_reports_progress_to_the_end_and_returns_the_package() {
    let dir = tempfile::tempdir().unwrap();
    let request = monaco_request(dir.path());
    let job = CompileJob::start(request.clone(), CancelToken::new());
    let mut seen = Vec::new();
    while !job.is_finished() {
        let progress = job.progress();
        assert!((0.0..=1.0).contains(&progress.overall), "{progress:?}");
        if seen.last() != Some(&progress.stage) {
            seen.push(progress.stage);
        }
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
    let last = job.progress();
    assert_eq!(last.stage, CompileStage::WritePackage);
    assert_eq!(last.overall, 1.0);
    let result = job.wait().unwrap();
    assert_eq!(result.graph_id, "monaco");
    assert_eq!(
        std::fs::read(&request.output).unwrap().len() as u64,
        result.byte_len
    );
    assert!(seen.len() > 1, "{seen:?}");
    assert_eq!(tree(dir.path()), ["out", "out/routing.mrt"]);
}

#[test]
fn a_cancelled_compile_job_ends_cancelled_without_a_package() {
    let dir = tempfile::tempdir().unwrap();
    let cancel = CancelToken::new();
    let job = CompileJob::start(monaco_request(dir.path()), cancel.clone());
    poll_until(&job, |progress| {
        progress.stage == CompileStage::FinalizeGraph
    });
    cancel.cancel();
    assert_eq!(job.wait().unwrap_err(), CompileError::Cancelled);
    assert!(tree(dir.path()).is_empty(), "{:?}", tree(dir.path()));
}

#[test]
fn dropping_an_unfinished_compile_job_cancels_it_and_cleans_up() {
    let dir = tempfile::tempdir().unwrap();
    let cancel = CancelToken::new();
    let job = CompileJob::start(monaco_request(dir.path()), cancel.clone());
    poll_until(&job, |progress| {
        progress.stage == CompileStage::CollectRoads
    });
    let finished = job.is_finished();
    drop(job);
    if !finished {
        assert!(cancel.is_cancelled());
        assert!(tree(dir.path()).is_empty(), "{:?}", tree(dir.path()));
    }
}

#[test]
fn the_overall_share_rises_through_the_stages() {
    let mut last = 0.0;
    for stage in CompileStage::ALL {
        for fraction in [None, Some(0.0), Some(0.5), Some(1.0)] {
            let overall = stage.overall(fraction);
            assert!(overall >= last, "{stage:?} {fraction:?}");
            last = overall;
        }
    }
    assert_eq!(CompileStage::CollectRoads.overall(Some(0.0)), 0.0);
    assert_eq!(CompileStage::WritePackage.overall(Some(1.0)), 1.0);
}

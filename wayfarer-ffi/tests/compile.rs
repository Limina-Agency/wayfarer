//! Compiling through the C ABI (the `compiler` feature): a job compiles the
//! Monaco fixture to the same bytes as the CLI, and a cancelled job leaves
//! nothing behind.
#![cfg(feature = "compiler")]

use std::ffi::{CStr, CString, c_char};
use std::ptr;

use serde_json::Value;
use wayfarer_ffi::*;

fn take(text: *mut c_char) -> Value {
    assert!(!text.is_null());
    let value = serde_json::from_str(unsafe { CStr::from_ptr(text) }.to_str().unwrap()).unwrap();
    unsafe { wayfarer_string_free(text) };
    value
}

fn request(output: &std::path::Path) -> CString {
    let input = concat!(env!("CARGO_MANIFEST_DIR"), "/../fixtures/monaco.osm.pbf");
    CString::new(
        serde_json::json!({
            "input": input,
            "output": output,
            "graphId": "monaco",
            "country": "MC",
            "threads": 2,
        })
        .to_string(),
    )
    .unwrap()
}

#[test]
fn a_job_compiles_the_fixture_and_reports_progress() {
    let dir = tempfile::tempdir().unwrap();
    let output = dir.path().join("monaco.mrt");
    let request = request(&output);
    let mut job = ptr::null_mut();
    let mut out = ptr::null_mut();
    let status =
        unsafe { wayfarer_compile_start(request.as_ptr(), ptr::null(), &mut job, &mut out) };
    let first = take(out);
    assert_eq!(status, WAYFARER_OK, "{first}");
    assert!(first["overall"].as_f64().unwrap() < 1.0);

    let mut last = 0.0;
    while unsafe { wayfarer_compile_is_finished(job) } == 0 {
        let mut out = ptr::null_mut();
        assert_eq!(
            unsafe { wayfarer_compile_progress(job, &mut out) },
            WAYFARER_OK
        );
        let overall = take(out)["overall"].as_f64().unwrap();
        assert!(
            overall >= last - 1e-9,
            "progress went back: {last} -> {overall}"
        );
        last = overall;
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    let mut out = ptr::null_mut();
    let status = unsafe { wayfarer_compile_wait(job, &mut out) };
    let result = take(out);
    assert_eq!(status, WAYFARER_OK, "{result}");
    let fixture: Value = serde_json::from_str(
        &std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../fixtures/monaco-info.json"
        ))
        .unwrap(),
    )
    .unwrap();
    assert_eq!(result["datasetHash"], fixture["datasetHash"]);
    assert!(output.exists());
    // Waiting again returns the same outcome.
    let mut out = ptr::null_mut();
    assert_eq!(unsafe { wayfarer_compile_wait(job, &mut out) }, WAYFARER_OK);
    assert_eq!(take(out)["datasetHash"], fixture["datasetHash"]);
    // Progress after the result is the compile's last report.
    let mut out = ptr::null_mut();
    assert_eq!(
        unsafe { wayfarer_compile_progress(job, &mut out) },
        WAYFARER_OK
    );
    let last = take(out);
    assert_eq!(last["stage"], "write_package", "{last}");
    assert_eq!(last["overall"], 1.0, "{last}");
    unsafe { wayfarer_compile_free(job) };

    let versions = take(wayfarer_versions());
    assert_eq!(versions["compiler"], true);
}

#[test]
fn a_cancelled_job_leaves_no_package() {
    let dir = tempfile::tempdir().unwrap();
    let output = dir.path().join("monaco.mrt");
    let request = request(&output);
    let cancel = wayfarer_cancel_token_new(0);
    let mut job = ptr::null_mut();
    let mut out = ptr::null_mut();
    let status = unsafe { wayfarer_compile_start(request.as_ptr(), cancel, &mut job, &mut out) };
    take(out);
    assert_eq!(status, WAYFARER_OK);
    unsafe { wayfarer_cancel_token_cancel(cancel) };
    let mut out = ptr::null_mut();
    let status = unsafe { wayfarer_compile_wait(job, &mut out) };
    let error = take(out);
    assert_eq!(status, WAYFARER_CANCELLED, "{error}");
    assert_eq!(error["code"], "cancelled");
    assert!(!output.exists());
    // A cancelled job's progress does not claim it finished.
    let mut out = ptr::null_mut();
    unsafe { wayfarer_compile_progress(job, &mut out) };
    assert!(take(out)["overall"].as_f64().unwrap() < 1.0);
    assert_eq!(
        std::fs::read_dir(dir.path()).unwrap().count(),
        0,
        "scratch left behind"
    );
    unsafe {
        wayfarer_compile_free(job);
        wayfarer_cancel_token_free(cancel);
    }

    // An invalid request writes no handle.
    let bad = CString::new(r#"{"input": "x.osm.pbf"}"#).unwrap();
    let mut job = ptr::null_mut();
    let mut out = ptr::null_mut();
    let status = unsafe { wayfarer_compile_start(bad.as_ptr(), ptr::null(), &mut job, &mut out) };
    take(out);
    assert_eq!(status, WAYFARER_INVALID_ARGUMENT);
    assert!(job.is_null());
}

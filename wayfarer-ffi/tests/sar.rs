//! Search and rescue planning through the C ABI (the `sar` feature), on the
//! synthetic short-road package: a reach, a team plan and a coverage update
//! round-trip as JSON, invalid requests fail as invalid, and a cancelled
//! token stops the steps that take one.
#![cfg(feature = "sar")]

use std::ffi::{CStr, CString, c_char};
use std::ptr;

use serde_json::{Value, json};
use wayfarer_ffi::*;

fn take(text: *mut c_char) -> Value {
    assert!(!text.is_null());
    let value = serde_json::from_str(unsafe { CStr::from_ptr(text) }.to_str().unwrap()).unwrap();
    unsafe { wayfarer_string_free(text) };
    value
}

fn open() -> *mut WayfarerPackage {
    let path = CString::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../fixtures/routing/short_road.mrt"
    ))
    .unwrap();
    let (mut package, mut out) = (ptr::null_mut(), ptr::null_mut());
    let status = unsafe { wayfarer_open(path.as_ptr(), ptr::null(), &mut package, &mut out) };
    let info = take(out);
    assert_eq!(status, WAYFARER_OK, "{info}");
    package
}

type Step = unsafe extern "C" fn(
    *const WayfarerPackage,
    *const c_char,
    *const WayfarerCancelToken,
    *mut *mut c_char,
) -> i32;

fn step(
    function: Step,
    package: *const WayfarerPackage,
    request: &Value,
    cancel: *const WayfarerCancelToken,
) -> (i32, Value) {
    let request = CString::new(request.to_string()).unwrap();
    let mut out = ptr::null_mut();
    let status = unsafe { function(package, request.as_ptr(), cancel, &mut out) };
    (status, take(out))
}

fn reach_request() -> Value {
    json!({
        "lastKnown": { "lat": 52.0, "lng": 21.001 },
        "elapsedS": 120,
        "profile": "on_foot",
        "routeSectors": 4,
        "maxEdges": 1,
    })
}

#[test]
fn versions_report_the_sar_build() {
    assert_eq!(take(wayfarer_versions())["sar"], true);
}

#[test]
fn reach_teams_and_coverage_round_trip_as_json() {
    let package = open();
    let (status, reach) = step(wayfarer_sar_reach, package, &reach_request(), ptr::null());
    assert_eq!(status, WAYFARER_OK, "{reach}");
    assert_eq!(reach["areas"].as_array().unwrap().len(), 3);
    let probability = &reach["probability"];
    assert_eq!(probability["edges"].as_array().unwrap().len(), 1);
    let total = probability["edges"][0]["probability"].as_f64().unwrap()
        + probability["restOfWorld"].as_f64().unwrap()
        + probability["offNetwork"].as_f64().unwrap()
        + probability["unlisted"].as_f64().unwrap_or(0.0);
    assert!((total - 1.0).abs() < 1e-9, "{probability}");

    let segment = json!({
        "id": "road",
        "polygon": [
            { "lat": 51.9995, "lng": 21.0000 },
            { "lat": 51.9995, "lng": 21.0020 },
            { "lat": 52.0005, "lng": 21.0020 },
            { "lat": 52.0005, "lng": 21.0000 },
        ],
    });
    let teams = json!({
        "teams": [{ "id": "alpha", "point": { "lat": 52.0, "lng": 21.0002 }, "profile": "on_foot" }],
        "segments": [segment],
        "probability": probability,
        "shiftS": 600,
    });
    let (status, plan) = step(wayfarer_sar_plan_teams, package, &teams, ptr::null());
    assert_eq!(status, WAYFARER_OK, "{plan}");
    assert_eq!(plan["assignments"][0]["teamId"], "alpha");

    let samples = (0..=8)
        .map(|index| {
            json!({
                "position": { "lat": 52.0, "lng": 21.0002 + 0.0002 * f64::from(index) },
                "sampledAtMs": 1_000 * i64::from(index) * 15,
            })
        })
        .collect::<Vec<_>>();
    let coverage = json!({
        "tracks": [{ "teamId": "alpha", "samples": samples, "profile": "on_foot" }],
        "probability": probability,
        "segments": [segment],
    });
    let request = CString::new(coverage.to_string()).unwrap();
    let mut out = ptr::null_mut();
    let status = unsafe { wayfarer_sar_coverage(package, request.as_ptr(), &mut out) };
    let covered = take(out);
    assert_eq!(status, WAYFARER_OK, "{covered}");
    assert!(
        covered["probabilityOfSuccess"].as_f64().unwrap() > 0.0,
        "{covered}"
    );

    unsafe { wayfarer_close(package) };
}

#[test]
fn invalid_and_cancelled_steps_fail_with_their_status() {
    let package = open();
    let mut no_time = reach_request();
    no_time.as_object_mut().unwrap().remove("elapsedS");
    let (status, error) = step(wayfarer_sar_reach, package, &no_time, ptr::null());
    assert_eq!(status, WAYFARER_INVALID_ARGUMENT, "{error}");
    assert_eq!(error["code"], "invalid_request");
    let (status, error) = step(
        wayfarer_sar_reach,
        package,
        &json!({ "typo": 1 }),
        ptr::null(),
    );
    assert_eq!(status, WAYFARER_INVALID_ARGUMENT, "{error}");
    assert_eq!(error["code"], "invalid_argument");

    let cancelled = wayfarer_cancel_token_new(0);
    unsafe { wayfarer_cancel_token_cancel(cancelled) };
    let (status, error) = step(wayfarer_sar_reach, package, &reach_request(), cancelled);
    assert_eq!(status, WAYFARER_CANCELLED, "{error}");
    assert_eq!(error["code"], "cancelled");
    unsafe {
        wayfarer_cancel_token_free(cancelled);
        wayfarer_close(package);
    }
}

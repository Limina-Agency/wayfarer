//! The C ABI as a host calls it, on the synthetic short-road package.

use std::ffi::{CStr, CString, c_char};
use std::ptr;

use serde_json::Value;
use wayfarer_ffi::*;

fn fixture() -> CString {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../fixtures/routing/short_road.mrt"
    );
    CString::new(path).unwrap()
}

/// Takes ownership of a returned string and parses it.
fn take(text: *mut c_char) -> Value {
    assert!(!text.is_null());
    let value = serde_json::from_str(unsafe { CStr::from_ptr(text) }.to_str().unwrap()).unwrap();
    unsafe { wayfarer_string_free(text) };
    value
}

fn open() -> *mut WayfarerPackage {
    let path = fixture();
    let mut package = ptr::null_mut();
    let mut out = ptr::null_mut();
    let status = unsafe { wayfarer_open(path.as_ptr(), ptr::null(), &mut package, &mut out) };
    let info = take(out);
    assert_eq!(status, WAYFARER_OK, "{info}");
    assert_eq!(info["formatVersion"], 1);
    assert_eq!(info["costingModelVersion"], 1);
    package
}

fn call(
    function: unsafe extern "C" fn(*const WayfarerPackage, *const c_char, *mut *mut c_char) -> i32,
    package: *const WayfarerPackage,
    request: &str,
) -> (i32, Value) {
    let request = CString::new(request).unwrap();
    let mut out = ptr::null_mut();
    let status = unsafe { function(package, request.as_ptr(), &mut out) };
    (status, take(out))
}

#[test]
fn versions_describe_the_library() {
    assert_eq!(wayfarer_abi_version(), 1);
    let versions = take(wayfarer_versions());
    assert_eq!(versions["abi"], 1);
    assert_eq!(versions["readableFormats"][0], 1);
    assert_eq!(versions["readableFormats"][1], 1);
    assert_eq!(versions["costingModel"], 1);
}

#[test]
fn routes_matrices_and_isochrones_round_trip_as_json() {
    let package = open();
    let (status, route) = call(
        wayfarer_route,
        package,
        r#"{"locations": [{"point": {"lat": 52.0, "lng": 21.0002}}, {"point": {"lat": 52.0, "lng": 21.0018}}], "profile": "wheeled"}"#,
    );
    assert_eq!(status, WAYFARER_OK, "{route}");
    assert!(route["distanceM"].as_f64().unwrap() > 100.0);

    let (status, matrix) = call(
        wayfarer_matrix,
        package,
        r#"{"sources": [{"lat": 52.0, "lng": 21.0002}], "targets": [{"lat": 52.0, "lng": 21.0018}], "profile": "wheeled"}"#,
    );
    assert_eq!(status, WAYFARER_OK, "{matrix}");
    assert_eq!(matrix["cells"].as_array().unwrap().len(), 1);

    let (status, isochrone) = call(
        wayfarer_isochrone,
        package,
        r#"{"origin": {"lat": 52.0, "lng": 21.0002}, "profile": "wheeled", "contours": [{"contour": 60.0, "metric": "time_seconds"}]}"#,
    );
    assert_eq!(status, WAYFARER_OK, "{isochrone}");

    let mut out = ptr::null_mut();
    assert_eq!(unsafe { wayfarer_info(package, &mut out) }, WAYFARER_OK);
    assert_eq!(take(out)["edgeCount"], 2);
    unsafe { wayfarer_close(package) };
}

#[test]
fn failures_carry_a_status_and_a_code() {
    let package = open();
    let (status, error) = call(wayfarer_route, package, "{not json");
    assert_eq!(status, WAYFARER_INVALID_ARGUMENT);
    assert_eq!(error["code"], "invalid_argument");

    let (status, error) = call(
        wayfarer_route,
        package,
        r#"{"locations": [{"point": {"lat": 10.0, "lng": 10.0}}, {"point": {"lat": 52.0, "lng": 21.0018}}], "profile": "wheeled"}"#,
    );
    assert_eq!(status, WAYFARER_ROUTE_ERROR);
    assert_eq!(error["code"], "no_candidate");
    assert!(error["diagnostics"].is_object());

    let (status, error) = call(wayfarer_route, ptr::null(), "{}");
    assert_eq!(status, WAYFARER_INVALID_ARGUMENT, "{error}");
    unsafe { wayfarer_close(package) };

    let missing = CString::new("/nonexistent/routing.mrt").unwrap();
    let mut handle = ptr::null_mut();
    let mut out = ptr::null_mut();
    let status = unsafe { wayfarer_open(missing.as_ptr(), ptr::null(), &mut handle, &mut out) };
    assert_eq!(status, WAYFARER_PACKAGE_ERROR);
    assert_eq!(take(out)["code"], "not_found");
    assert!(handle.is_null());

    let options = CString::new(r#"{"memoryProfile": "tiny"}"#).unwrap();
    let path = fixture();
    let status = unsafe { wayfarer_open(path.as_ptr(), options.as_ptr(), &mut handle, &mut out) };
    assert_eq!(status, WAYFARER_INVALID_ARGUMENT);
    take(out);

    // Null strings and handles are ignored.
    unsafe {
        wayfarer_string_free(ptr::null_mut());
        wayfarer_close(ptr::null_mut());
    }
}

#[test]
fn verify_hashes_every_section() {
    let path = fixture();
    let mut out = ptr::null_mut();
    let status = unsafe { wayfarer_verify(path.as_ptr(), &mut out) };
    let verification = take(out);
    assert_eq!(status, WAYFARER_OK, "{verification}");
    assert!(verification["sections"].as_array().unwrap().len() > 50);
}

const ROUTE: &str = r#"{"locations": [{"point": {"lat": 52.0, "lng": 21.0002}}, {"point": {"lat": 52.0, "lng": 21.0018}}], "profile": "wheeled"}"#;
const MATRIX: &str = r#"{"sources": [{"lat": 52.0, "lng": 21.0002}], "targets": [{"lat": 52.0, "lng": 21.0018}], "profile": "wheeled"}"#;
const ISOCHRONE: &str = r#"{"origin": {"lat": 52.0, "lng": 21.0002}, "profile": "wheeled", "contours": [{"contour": 60.0, "metric": "time_seconds"}]}"#;
const WORKFLOW: &str = r#"{"locations": [{"point": {"lat": 52.0, "lng": 21.0002}}, {"point": {"lat": 52.00004, "lng": 21.0006}}, {"point": {"lat": 52.0, "lng": 21.0018}}], "legs": ["routed", "routed"], "profile": "wheeled", "options": {"maxCorrelationDistanceM": 25}}"#;
const MAP_MATCH: &str = r#"{"samples": [{"position": {"lat": 52.0, "lng": 21.0003}}, {"position": {"lat": 52.0, "lng": 21.0009}}, {"position": {"lat": 52.0, "lng": 21.0015}}], "profile": "wheeled"}"#;

type CancellableCall = unsafe extern "C" fn(
    *const WayfarerPackage,
    *const c_char,
    *const WayfarerCancelToken,
    *mut *mut c_char,
) -> i32;

fn call_with_cancel(
    function: CancellableCall,
    package: *const WayfarerPackage,
    request: &str,
    cancel: *const WayfarerCancelToken,
) -> (i32, Value) {
    let request = CString::new(request).unwrap();
    let mut out = ptr::null_mut();
    let status = unsafe { function(package, request.as_ptr(), cancel, &mut out) };
    (status, take(out))
}

const CANCELLABLE: [(&str, CancellableCall, &str); 5] = [
    ("route", wayfarer_route_with_cancel, ROUTE),
    ("matrix", wayfarer_matrix_with_cancel, MATRIX),
    ("isochrone", wayfarer_isochrone_with_cancel, ISOCHRONE),
    ("workflow", wayfarer_route_with_cancel, WORKFLOW),
    ("map match", wayfarer_map_match, MAP_MATCH),
];

#[test]
fn cancel_tokens_stop_every_request_that_takes_one() {
    let package = open();
    let live = wayfarer_cancel_token_new(0);
    let cancelled = wayfarer_cancel_token_new(0);
    let expiring = wayfarer_cancel_token_new(1);
    assert!(!live.is_null() && !cancelled.is_null() && !expiring.is_null());
    unsafe {
        wayfarer_cancel_token_cancel(cancelled);
        // Cancelling twice, or a null token, is harmless.
        wayfarer_cancel_token_cancel(cancelled);
        wayfarer_cancel_token_cancel(ptr::null());
    }
    std::thread::sleep(std::time::Duration::from_millis(5));
    for (name, function, request) in CANCELLABLE {
        for cancel in [ptr::null(), live.cast_const()] {
            let (status, result) = call_with_cancel(function, package, request, cancel);
            assert_eq!(status, WAYFARER_OK, "{name}: {result}");
        }
        let (status, error) = call_with_cancel(function, package, request, cancelled);
        assert_eq!(status, WAYFARER_CANCELLED, "{name}: {error}");
        assert_eq!(error["code"], "cancelled", "{name}");
        let (status, error) = call_with_cancel(function, package, request, expiring);
        assert_eq!(status, WAYFARER_CANCELLED, "{name}: {error}");
        assert_eq!(error["code"], "deadline_exceeded", "{name}");
    }
    // A cancelled route still reports its diagnostics; a workflow, its leg.
    let (_, error) = call_with_cancel(wayfarer_route_with_cancel, package, ROUTE, cancelled);
    assert!(error["diagnostics"].is_object());
    let (_, error) = call_with_cancel(wayfarer_route_with_cancel, package, WORKFLOW, cancelled);
    assert_eq!(error["segmentIndex"], 0);
    // A timeout beyond the clock's range never passes.
    let distant = wayfarer_cancel_token_new(u64::MAX);
    let (status, route) = call_with_cancel(wayfarer_route_with_cancel, package, ROUTE, distant);
    assert_eq!(status, WAYFARER_OK, "{route}");
    unsafe {
        wayfarer_cancel_token_free(distant);
        wayfarer_cancel_token_free(live);
        wayfarer_cancel_token_free(cancelled);
        wayfarer_cancel_token_free(expiring);
        wayfarer_cancel_token_free(ptr::null_mut());
        wayfarer_close(package);
    }
}

#[test]
fn a_token_is_cancelled_from_another_thread() {
    // Raw pointers are not `Send`; the header makes tokens shareable.
    struct Shared(*mut WayfarerCancelToken);
    unsafe impl Send for Shared {}
    let package = open();
    let cancel = wayfarer_cancel_token_new(0);
    let shared = Shared(cancel);
    std::thread::spawn(move || {
        let shared = shared;
        unsafe { wayfarer_cancel_token_cancel(shared.0) };
    })
    .join()
    .unwrap();
    let (status, error) = call_with_cancel(wayfarer_route_with_cancel, package, ROUTE, cancel);
    assert_eq!(status, WAYFARER_CANCELLED, "{error}");
    unsafe {
        wayfarer_cancel_token_free(cancel);
        wayfarer_close(package);
    }
}

#[test]
fn workflows_route_waypoints_as_camel_case_json() {
    let package = open();
    let (status, workflow) =
        call_with_cancel(wayfarer_route_with_cancel, package, WORKFLOW, ptr::null());
    assert_eq!(status, WAYFARER_OK, "{workflow}");
    assert_eq!(workflow["segments"].as_array().unwrap().len(), 2);
    // Each routed segment is a LegResult.
    assert!(
        workflow["segments"][0]["routed"]["distanceM"]
            .as_f64()
            .unwrap()
            > 0.0
    );
    assert!(workflow["distanceM"].as_f64().unwrap() > 100.0);
    assert_eq!(workflow["diagnostics"]["segmentCount"], 2);
    assert_eq!(workflow["alternatives"], "not_requested");
    assert_eq!(workflow["locationMatches"].as_array().unwrap().len(), 3);
    assert!(workflow["sourceId"].is_string());
    assert_eq!(workflow["revision"].as_str().unwrap().len(), 64);

    let (status, error) = call_with_cancel(
        wayfarer_route_with_cancel,
        package,
        r#"{"locations": [], "legs": [], "profile": "wheeled"}"#,
        ptr::null(),
    );
    assert_eq!(status, WAYFARER_INVALID_ARGUMENT, "{error}");
    let far = WORKFLOW.replace("21.0018", "10.0");
    let (status, error) = call_with_cancel(wayfarer_route_with_cancel, package, &far, ptr::null());
    assert_eq!(status, WAYFARER_ROUTE_ERROR, "{error}");
    assert_eq!(error["code"], "no_candidate");
    assert_eq!(error["segmentIndex"], 1);
    unsafe { wayfarer_close(package) };
}

#[test]
fn traces_map_match_to_the_road() {
    let package = open();
    let (status, matched) = call_with_cancel(wayfarer_map_match, package, MAP_MATCH, ptr::null());
    assert_eq!(status, WAYFARER_OK, "{matched}");
    assert_eq!(matched["sampleCount"], 3);
    assert_eq!(matched["matchedSampleCount"], 3);
    assert!(!matched["matchedEdgeIds"].as_array().unwrap().is_empty());

    for (request, status, code) in [
        (
            r#"{"samples": [], "profile": "wheeled"}"#,
            WAYFARER_INVALID_ARGUMENT,
            "invalid_argument",
        ),
        (
            r#"{"samples": [{"position": {"lat": 10.0, "lng": 10.0}}], "profile": "wheeled"}"#,
            WAYFARER_ROUTE_ERROR,
            "no_match",
        ),
        (
            r#"{"samples": [], "profile": "wheeled", "unknown": 1}"#,
            WAYFARER_INVALID_ARGUMENT,
            "invalid_argument",
        ),
    ] {
        let (actual, error) = call_with_cancel(wayfarer_map_match, package, request, ptr::null());
        assert_eq!(actual, status, "{error}");
        assert_eq!(error["code"], code);
    }
    unsafe { wayfarer_close(package) };
}

fn prepare(route_json: &str) -> (i32, *mut WayfarerNavigationRoute, Value) {
    let route_json = CString::new(route_json).unwrap();
    let mut route = ptr::null_mut();
    let mut out = ptr::null_mut();
    let status =
        unsafe { wayfarer_navigation_route_new(route_json.as_ptr(), &mut route, &mut out) };
    (status, route, take(out))
}

fn progress(route: *const WayfarerNavigationRoute, request: &str) -> (i32, Value) {
    let request = CString::new(request).unwrap();
    let mut out = ptr::null_mut();
    let status = unsafe { wayfarer_navigation_progress(route, request.as_ptr(), &mut out) };
    (status, take(out))
}

#[test]
fn navigation_progress_follows_a_prepared_route() {
    let package = open();
    let (status, route) = call(wayfarer_route, package, ROUTE);
    assert_eq!(status, WAYFARER_OK, "{route}");
    unsafe { wayfarer_close(package) };

    // Prepared from the route as the route call returned it; no package needed.
    let (status, prepared, info) = prepare(&serde_json::json!({ "route": route }).to_string());
    assert_eq!(status, WAYFARER_OK, "{info}");
    assert!(info["geometryPointCount"].as_u64().unwrap() >= 2);

    let (status, first) = progress(
        prepared,
        r#"{"sample": {"position": {"lat": 52.0, "lng": 21.0006}, "sampledAtMs": 1000}}"#,
    );
    assert_eq!(status, WAYFARER_OK, "{first}");
    assert_eq!(first["status"], "on_route");
    let along = first["distanceAlongRouteMeters"].as_f64().unwrap();
    assert!(along > 0.0, "{first}");

    // The next sample continues from the previous progress.
    let request = serde_json::json!({
        "sample": {"position": {"lat": 52.0, "lng": 21.0012}, "sampledAtMs": 6000},
        "previous": first,
        "options": {"arrivalThresholdMeters": 5.0},
    });
    let (status, second) = progress(prepared, &request.to_string());
    assert_eq!(status, WAYFARER_OK, "{second}");
    assert!(second["distanceAlongRouteMeters"].as_f64().unwrap() > along);
    assert!(
        second["distanceRemainingMeters"].as_f64().unwrap()
            < first["distanceRemainingMeters"].as_f64().unwrap()
    );

    let (status, error) = progress(prepared, r#"{"position": {}}"#);
    assert_eq!(status, WAYFARER_INVALID_ARGUMENT, "{error}");
    let (status, error) = progress(
        ptr::null(),
        r#"{"sample": {"position": {"lat": 52.0, "lng": 21.0}}}"#,
    );
    assert_eq!(status, WAYFARER_INVALID_ARGUMENT, "{error}");
    unsafe { wayfarer_navigation_route_free(prepared) };

    // A RouteNavigationInput directly.
    let (status, prepared, _) = prepare(
        r#"{"input": {"geometry": [{"lat": 52.0, "lng": 21.0002}, {"lat": 52.0, "lng": 21.0018}]}}"#,
    );
    assert_eq!(status, WAYFARER_OK);
    let (status, arrived) = progress(
        prepared,
        r#"{"sample": {"position": {"lat": 52.0, "lng": 21.0018}}}"#,
    );
    assert_eq!(status, WAYFARER_OK, "{arrived}");
    assert_eq!(arrived["arrived"], true);
    unsafe { wayfarer_navigation_route_free(prepared) };

    // Neither or both sources, or a degenerate route, write no handle.
    for bad in [
        "{}".to_string(),
        serde_json::json!({ "route": route, "input": {"geometry": []} }).to_string(),
        r#"{"input": {"geometry": [{"lat": 52.0, "lng": 21.0}]}}"#.to_string(),
    ] {
        let (status, prepared, error) = prepare(&bad);
        assert_eq!(status, WAYFARER_INVALID_ARGUMENT, "{error}");
        assert!(prepared.is_null());
    }
    unsafe { wayfarer_navigation_route_free(ptr::null_mut()) };
}

/// SAR through the C ABI: the package info lists what is available and the
/// minimum factors, an invalid surcharge is an invalid argument, and R1, R2,
/// R3 and R5 route with their report; qualified access is on every route.
/// (`sar_unavailable` is tested by the engine.)
#[test]
fn sar_requests_report_capability_and_typed_failures() {
    let package = open();
    let mut out = ptr::null_mut();
    assert_eq!(unsafe { wayfarer_info(package, &mut out) }, WAYFARER_OK);
    let info = take(out);
    let capability = &info["sarCapability"];
    assert_eq!(capability["unavailable"], serde_json::json!([]));
    assert_eq!(
        capability["available"],
        serde_json::json!([
            "legal_access",
            "wrong_way",
            "turn_restrictions",
            "barriers",
            "vehicle_on_paths",
            "conditional",
            "off_network"
        ])
    );
    assert_eq!(capability["legalAccessMinFactor"], 1.2);
    assert_eq!(capability["wrongWayMinFactor"], 2.0);
    assert_eq!(capability["vehicleOnPathsMinFactor"], 3.0);
    let route = |sar: &str| {
        call(
            wayfarer_route,
            package,
            &format!(
                r#"{{"locations": [{{"point": {{"lat": 52.0, "lng": 21.0002}}}}, {{"point": {{"lat": 52.0, "lng": 21.0018}}}}],
                    "profile": "wheeled", "options": {{"sar": {sar}}}}}"#
            ),
        )
    };
    let (status, result) =
        route(r#"{"relaxations": {"vehicleOnPaths": {"unknownWidth": "allow"}}}"#);
    assert_eq!(status, WAYFARER_OK, "{result}");
    assert_eq!(
        result["sar"]["enabled"],
        serde_json::json!(["vehicle_on_paths"])
    );
    assert_eq!(
        result["policySummary"]["sar"]["vehicleOnPathsUnknownWidth"],
        "allow"
    );
    let (status, error) = route(r#"{"relaxations": {"vehicleOnPaths": {"factor": 2}}}"#);
    assert_eq!(status, WAYFARER_INVALID_ARGUMENT, "{error}");
    let (status, result) = route(r#"{"relaxations": {"wrongWay": {}, "legalAccess": {}}}"#);
    assert_eq!(status, WAYFARER_OK, "{result}");
    assert_eq!(
        result["sar"]["enabled"],
        serde_json::json!(["legal_access", "wrong_way"])
    );
    assert_eq!(result["policySummary"]["sar"]["wrongWayFactor"], 2.0);
    let (status, error) = route(r#"{"relaxations": {"legalAccess": {"factor": 1.0}}}"#);
    assert_eq!(status, WAYFARER_INVALID_ARGUMENT, "{error}");
    let (status, error) = route(r#"{"relaxations": {"turn_restrictions": {"penalty_s": -1}}}"#);
    assert_eq!(status, WAYFARER_INVALID_ARGUMENT, "{error}");
    assert_eq!(error["code"], "invalid_request");
    let (status, result) = route(r#"{"relaxations": {"turnRestrictions": {"penaltyS": 30}}}"#);
    assert_eq!(status, WAYFARER_OK, "{result}");
    assert_eq!(
        result["sar"]["enabled"],
        serde_json::json!(["turn_restrictions"])
    );
    assert_eq!(
        result["policySummary"]["sar"]["turnRestrictionPenaltyS"],
        30.0
    );
    assert!(result["qualifiedAccess"].is_array());
    // The workflow reports the same failure.
    let request = CString::new(
        r#"{"locations": [{"point": {"lat": 52.0, "lng": 21.0002}}, {"point": {"lat": 52.0, "lng": 21.0018}}],
            "legs": ["routed"], "profile": "wheeled",
            "options": {"sar": {"relaxations": {"vehicle_on_paths": {"classes": ["steps"]}}}}}"#,
    )
    .unwrap();
    let mut out = ptr::null_mut();
    let status =
        unsafe { wayfarer_route_with_cancel(package, request.as_ptr(), ptr::null(), &mut out) };
    let error = take(out);
    assert_eq!(status, WAYFARER_INVALID_ARGUMENT, "{error}");
    assert_eq!(error["code"], "invalid_request");
    unsafe { wayfarer_close(package) };
}

/// Every key path in `value` (`a.b`, `a[].c`), arrays merged.
fn key_paths(value: &Value, prefix: &str, out: &mut std::collections::BTreeSet<String>) {
    match value {
        Value::Object(map) => {
            for (key, child) in map {
                let path = if prefix.is_empty() {
                    key.clone()
                } else {
                    format!("{prefix}.{key}")
                };
                out.insert(path.clone());
                key_paths(child, &path, out);
            }
        }
        Value::Array(items) => {
            for item in items {
                key_paths(item, &format!("{prefix}[]"), out);
            }
        }
        _ => {}
    }
}

/// The JSON keys of every response kind are part of the ABI: renaming or
/// removing one breaks hosts. This pins the keys the fixture requests return
/// (`tests/abi-keys.json`); after an intended change, rerun with
/// `UPDATE_ABI_KEYS=1` and bump `WAYFARER_ABI_VERSION` if a key a host
/// may read was renamed or removed.
#[test]
fn response_keys_are_pinned() {
    let mut responses = Vec::<(&str, Value)>::new();
    responses.push(("versions", take(wayfarer_versions())));
    let path = fixture();
    let mut package = ptr::null_mut();
    let mut out = ptr::null_mut();
    let status = unsafe { wayfarer_open(path.as_ptr(), ptr::null(), &mut package, &mut out) };
    assert_eq!(status, WAYFARER_OK);
    responses.push(("open", take(out)));
    let mut out = ptr::null_mut();
    assert_eq!(
        unsafe { wayfarer_verify(path.as_ptr(), &mut out) },
        WAYFARER_OK
    );
    responses.push(("verify", take(out)));
    for (name, function, request) in CANCELLABLE {
        let (status, value) = call_with_cancel(function, package, request, ptr::null());
        assert_eq!(status, WAYFARER_OK, "{name}: {value}");
        responses.push((name, value));
    }
    let (status, error) = call(
        wayfarer_route,
        package,
        r#"{"locations": [{"point": {"lat": 10.0, "lng": 10.0}}, {"point": {"lat": 52.0, "lng": 21.0018}}], "profile": "wheeled"}"#,
    );
    assert_eq!(status, WAYFARER_ROUTE_ERROR);
    responses.push(("route error", error));
    let far = WORKFLOW.replace("21.0018", "10.0");
    let (status, error) = call_with_cancel(wayfarer_route_with_cancel, package, &far, ptr::null());
    assert_eq!(status, WAYFARER_ROUTE_ERROR);
    responses.push(("workflow error", error));
    let (_, route) = call(wayfarer_route, package, ROUTE);
    unsafe { wayfarer_close(package) };
    let (status, prepared, info) = prepare(&serde_json::json!({ "route": route }).to_string());
    assert_eq!(status, WAYFARER_OK, "{info}");
    responses.push(("navigation route", info));
    let (status, first) = progress(
        prepared,
        r#"{"sample": {"position": {"lat": 52.0, "lng": 21.0006}, "sampledAtMs": 1000}}"#,
    );
    assert_eq!(status, WAYFARER_OK, "{first}");
    responses.push(("navigation progress", first));
    unsafe { wayfarer_navigation_route_free(prepared) };

    // `ABI_RESPONSE_DIR=<dir>` writes each response, to check them against
    // `schema/` with an outside validator.
    if let Some(dir) = std::env::var_os("ABI_RESPONSE_DIR") {
        for (name, value) in &responses {
            let path = std::path::Path::new(&dir).join(format!("{}.json", name.replace(' ', "-")));
            std::fs::write(path, serde_json::to_string_pretty(value).unwrap()).unwrap();
        }
    }
    let actual = Value::Object(
        responses
            .into_iter()
            .map(|(name, value)| {
                let mut paths = std::collections::BTreeSet::new();
                key_paths(&value, "", &mut paths);
                (name.to_string(), Value::from_iter(paths))
            })
            .collect(),
    );
    let pinned = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/abi-keys.json");
    if std::env::var_os("UPDATE_ABI_KEYS").is_some() {
        std::fs::write(
            pinned,
            serde_json::to_string_pretty(&actual).unwrap() + "\n",
        )
        .unwrap();
    }
    let expected: Value = serde_json::from_str(
        &std::fs::read_to_string(pinned)
            .expect("tests/abi-keys.json (UPDATE_ABI_KEYS=1 writes it)"),
    )
    .unwrap();
    assert!(
        actual == expected,
        "response keys changed; rerun with UPDATE_ABI_KEYS=1 if intended (see the test's docs)"
    );
}

#[test]
fn points_snap_and_geometry_projects_onto_the_road() {
    let package = open();
    let (status, snapped) = call(
        wayfarer_snap,
        package,
        r#"{"point": {"lat": 52.0001, "lng": 21.001}, "profile": "wheeled"}"#,
    );
    assert_eq!(status, WAYFARER_OK, "{snapped}");
    let candidates = snapped["candidates"].as_array().unwrap();
    assert!(!candidates.is_empty(), "{snapped}");
    let (status, nothing) = call(
        wayfarer_snap,
        package,
        r#"{"point": {"lat": 10.0, "lng": 10.0}, "profile": "wheeled"}"#,
    );
    assert_eq!(status, WAYFARER_OK, "{nothing}");
    assert!(nothing["candidates"].as_array().unwrap().is_empty());

    let (status, projected) = call(
        wayfarer_project,
        package,
        r#"{"geometryKind": "line", "points": [{"lat": 52.0001, "lng": 21.0005}, {"lat": 52.0001, "lng": 21.0012}], "projectionDistanceM": 30}"#,
    );
    assert_eq!(status, WAYFARER_OK, "{projected}");
    assert!(
        !projected["edgeIds"].as_array().unwrap().is_empty(),
        "{projected}"
    );
    let (status, error) = call(
        wayfarer_project,
        package,
        r#"{"geometryKind": "line", "points": [], "projectionDistanceM": 30}"#,
    );
    assert_eq!(status, WAYFARER_INVALID_ARGUMENT, "{error}");
    unsafe { wayfarer_close(package) };
}

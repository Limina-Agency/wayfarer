//! Request JSON through the C ABI, on a Monaco package.
//!
//! The first byte picks the call; the rest is the request. Every call must
//! return a status, and any text it hands back must be JSON. A successful
//! answer stays within `wayfarer::limits`: matrix cells, isochrone buckets
//! and snap candidates.

#![no_main]

use std::ffi::{CStr, CString, c_char};
use std::ptr;
use std::sync::OnceLock;

use libfuzzer_sys::fuzz_target;
use serde_json::Value;
use wayfarer::limits::{MAX_ISOCHRONE_CONTOURS, MAX_MATRIX_CELLS, MAX_REQUEST_CANDIDATES};
use wayfarer_ffi::{
    WAYFARER_OK, WayfarerPackage, wayfarer_isochrone, wayfarer_map_match, wayfarer_matrix,
    wayfarer_open, wayfarer_project, wayfarer_route, wayfarer_snap, wayfarer_string_free,
};

struct Package(*mut WayfarerPackage);
// SAFETY: the C ABI's package handle is safe to share between threads.
unsafe impl Send for Package {}
unsafe impl Sync for Package {}

fn package() -> *const WayfarerPackage {
    static PACKAGE: OnceLock<Package> = OnceLock::new();
    PACKAGE
        .get_or_init(|| {
            let path = wayfarer_fuzz::monaco_package().to_str().expect("UTF-8 path");
            let path = CString::new(path).expect("path without NUL");
            let mut package = ptr::null_mut();
            let mut text = ptr::null_mut();
            // SAFETY: valid C strings and out-pointers; the text is freed.
            let status = unsafe { wayfarer_open(path.as_ptr(), ptr::null(), &mut package, &mut text) };
            unsafe { wayfarer_string_free(text) };
            assert_eq!(status, WAYFARER_OK, "the Monaco package opens");
            Package(package)
        })
        .0
}

type Call = unsafe extern "C" fn(*const WayfarerPackage, *const c_char, *mut *mut c_char) -> i32;

/// `wayfarer_map_match` without a cancel token (null is allowed).
unsafe extern "C" fn map_match(
    package: *const WayfarerPackage,
    request: *const c_char,
    out: *mut *mut c_char,
) -> i32 {
    // SAFETY: forwards the caller's pointers; a null token is allowed.
    unsafe { wayfarer_map_match(package, request, ptr::null(), out) }
}

const CALLS: [Call; 6] = [
    wayfarer_route,
    wayfarer_matrix,
    wayfarer_isochrone,
    map_match,
    wayfarer_snap,
    wayfarer_project,
];

fuzz_target!(|data: &[u8]| {
    let Some((&which, request)) = data.split_first() else {
        return;
    };
    let Ok(request) = CString::new(request) else {
        return;
    };
    let which = usize::from(which) % CALLS.len();
    let mut text = ptr::null_mut();
    // SAFETY: a valid package handle, a NUL-terminated request and an
    // out-pointer; the returned text is freed below.
    let status = unsafe { CALLS[which](package(), request.as_ptr(), &mut text) };
    if !text.is_null() {
        // SAFETY: the call returned a NUL-terminated string it allocated.
        let out = unsafe { CStr::from_ptr(text) }.to_str().expect("results are UTF-8");
        let value = serde_json::from_str::<Value>(out).expect("results are JSON");
        if status == WAYFARER_OK {
            check_bounds(which, &value);
        }
        unsafe { wayfarer_string_free(text) };
    }
});

/// The number of items in `value[key]`, an array.
fn count(value: &Value, key: &str) -> usize {
    value[key].as_array().map_or(0, Vec::len)
}

/// A successful answer of call `which` (an index into [`CALLS`]) is within
/// the request limits.
fn check_bounds(which: usize, value: &Value) {
    match which {
        1 => {
            let cells = count(value, "cells");
            assert!(cells <= MAX_MATRIX_CELLS, "{cells} matrix cells");
            let asked = value["sourceCount"].as_u64().zip(value["targetCount"].as_u64());
            assert_eq!(
                asked.and_then(|(sources, targets)| sources.checked_mul(targets)),
                Some(cells as u64),
                "a cell for every pair"
            );
        }
        2 => {
            let buckets = count(value, "buckets");
            assert!(buckets <= MAX_ISOCHRONE_CONTOURS, "{buckets} isochrone buckets");
        }
        4 => {
            let candidates = count(value, "candidates");
            assert!(candidates <= MAX_REQUEST_CANDIDATES, "{candidates} snap candidates");
        }
        _ => {}
    }
}

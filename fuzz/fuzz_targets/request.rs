//! Request JSON through the C ABI, on a Monaco package.
//!
//! The first byte picks the call; the rest is the request. Every call must
//! return a status, and any text it hands back must be JSON.

#![no_main]

use std::ffi::{CStr, CString, c_char};
use std::ptr;
use std::sync::OnceLock;

use libfuzzer_sys::fuzz_target;
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
    let call = CALLS[usize::from(which) % CALLS.len()];
    let mut text = ptr::null_mut();
    // SAFETY: a valid package handle, a NUL-terminated request and an
    // out-pointer; the returned text is freed below.
    let _status = unsafe { call(package(), request.as_ptr(), &mut text) };
    if !text.is_null() {
        // SAFETY: the call returned a NUL-terminated string it allocated.
        let out = unsafe { CStr::from_ptr(text) }.to_str().expect("results are UTF-8");
        serde_json::from_str::<serde_json::Value>(out).expect("results are JSON");
        unsafe { wayfarer_string_free(text) };
    }
});

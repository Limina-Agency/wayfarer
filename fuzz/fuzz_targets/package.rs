//! A hostile routing package: opened from memory, then queried.
//!
//! The root checksum guards against corruption, not attackers, so the target
//! recomputes it after the fuzzer's mutation: the input then reaches the
//! header, section table and section checks behind it, and the queries run on
//! whatever opens.

#![no_main]

use std::sync::Arc;

use libfuzzer_sys::fuzz_target;
use wayfarer::{
    GeoPoint, MobilityProfile, RouteIsochroneRequest, RouteMatrixRequest, RouteRequest,
    RoutingEngine, RoutingPackage, SnapRequest,
};
use wayfarer_package::format::{
    HEADER_FIXED_BYTES, HEADER_ROOT_CHECKSUM, SECTION_COUNT, TABLE_ENTRY_BYTES,
};

/// Queries near the seed packages' roads: `fixtures/routing/short_road.mrt`
/// (52.0, 21.0..21.002) and Monaco.
const POINTS: [(f64, f64); 4] = [
    (52.0, 21.0002),
    (52.0, 21.0018),
    (43.7397, 7.4275),
    (43.7290, 7.4160),
];

fn with_root_checksum(mut bytes: Vec<u8>) -> Vec<u8> {
    let table_end = HEADER_FIXED_BYTES + SECTION_COUNT * TABLE_ENTRY_BYTES;
    if bytes.len() >= table_end {
        let mut header = bytes[..HEADER_FIXED_BYTES].to_vec();
        header[HEADER_ROOT_CHECKSUM..HEADER_ROOT_CHECKSUM + 32].fill(0);
        let mut root = blake3::Hasher::new();
        root.update(&header);
        root.update(&bytes[HEADER_FIXED_BYTES..table_end]);
        bytes[HEADER_ROOT_CHECKSUM..HEADER_ROOT_CHECKSUM + 32]
            .copy_from_slice(root.finalize().as_bytes());
    }
    bytes
}

fn point(index: usize) -> GeoPoint {
    let (lat, lng) = POINTS[index];
    GeoPoint::new(lat, lng)
}

fuzz_target!(|data: &[u8]| {
    let bytes = with_root_checksum(data.to_vec());
    let Ok(package) = RoutingPackage::from_bytes(&bytes) else {
        return;
    };
    let _ = package.sar_capability();
    let engine = RoutingEngine::from_package(Arc::new(package));
    for (from, to) in [(0, 1), (2, 3)] {
        for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
            let _ = engine.route(RouteRequest::new(point(from), point(to), profile));
        }
        let _ = engine.snap(SnapRequest::new(point(from), MobilityProfile::Wheeled));
    }
    let matrix: RouteMatrixRequest = serde_json::from_str(
        r#"{"sources": [{"lat": 52.0, "lng": 21.0002}, {"lat": 43.7397, "lng": 7.4275}],
            "targets": [{"lat": 52.0, "lng": 21.0018}, {"lat": 43.7290, "lng": 7.4160}],
            "profile": "wheeled"}"#,
    )
    .expect("matrix request");
    let _ = engine.matrix(matrix);
    let isochrone: RouteIsochroneRequest = serde_json::from_str(
        r#"{"origin": {"lat": 52.0, "lng": 21.0002}, "profile": "wheeled",
            "contours": [{"contour": 120.0, "metric": "time_seconds"}]}"#,
    )
    .expect("isochrone request");
    let _ = engine.isochrone(isochrone);
});

//! A hostile routing package: opened from memory, then queried.
//!
//! The root checksum guards against corruption, not attackers, so the target
//! recomputes it after the fuzzer's mutation: the input then reaches the
//! header, section table and section checks behind it, and the queries run on
//! whatever opens, again after building CCH potentials for it.

#![no_main]

use std::sync::Arc;

use libfuzzer_sys::fuzz_target;
use wayfarer::{
    GeoPoint, MobilityProfile, RouteIsochroneRequest, RouteMatrixRequest, RouteRequest,
    RoutingEngine, RoutingPackage, SnapRequest,
};
use wayfarer_package::raw::resign;

/// Queries near the seed packages' roads: `fixtures/routing/short_road.mrt`
/// (52.0, 21.0..21.002) and Monaco.
const POINTS: [(f64, f64); 4] = [
    (52.0, 21.0002),
    (52.0, 21.0018),
    (43.7397, 7.4275),
    (43.7290, 7.4160),
];

/// The input with a root checksum that matches its header and table, in
/// either format.
fn with_root_checksum(mut bytes: Vec<u8>) -> Vec<u8> {
    resign(&mut bytes);
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
    let package = Arc::new(package);
    let engine = RoutingEngine::from_package(package.clone());
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
    // The CCH potentials, built from whatever opened, then the routes again
    // bounded by them.
    if package.prepare_potentials().is_some() {
        for (from, to) in [(0, 1), (2, 3)] {
            for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
                let _ = engine.route(RouteRequest::new(point(from), point(to), profile));
            }
        }
    }
});

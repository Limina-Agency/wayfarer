//! Overlay sets received from other devices: parsed, round-tripped, merged and
//! applied to a route on a Monaco package.

#![no_main]

use std::sync::OnceLock;

use libfuzzer_sys::fuzz_target;
use wayfarer::{GeoPoint, MobilityProfile, RouteRequest, RoutingEngine};
use wayfarer_service::overlay_set::RouteOverlaySet;

fn engine() -> &'static (RoutingEngine, String) {
    static ENGINE: OnceLock<(RoutingEngine, String)> = OnceLock::new();
    ENGINE.get_or_init(|| {
        let path = wayfarer_fuzz::monaco_package();
        let hash = wayfarer::RoutingPackage::open(path)
            .expect("the Monaco package opens")
            .dataset_hash()
            .to_string();
        (RoutingEngine::open(path).expect("the Monaco package opens"), hash)
    })
}

fuzz_target!(|data: &[u8]| {
    let Ok(text) = std::str::from_utf8(data) else {
        return;
    };
    let Ok(set) = RouteOverlaySet::from_json(text) else {
        return;
    };
    // A set that was accepted must survive its own serialization.
    let again = RouteOverlaySet::from_json(&set.to_json_with_hash())
        .expect("an accepted set round-trips through to_json_with_hash");
    let _ = RouteOverlaySet::merge(&[set.clone(), again]);
    let (engine, dataset_hash) = engine();
    for hash in [dataset_hash.as_str(), "0000"] {
        let (overlays, _binding) = set.workflow_overlays(hash);
        let mut request = RouteRequest::new(
            GeoPoint::new(43.7397, 7.4275),
            GeoPoint::new(43.7290, 7.4160),
            MobilityProfile::Wheeled,
        );
        request.overlays = overlays;
        let _ = engine.route(request);
    }
});

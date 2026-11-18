//! Route parity between the two lower bounds, on a Monaco package: the
//! landmarks, and the CCH potentials of `RoutingPackage::prepare_potentials`.
//!
//! The input picks a profile, two points in Monaco, costing options,
//! alternatives, and blocks, cost multipliers (raising and discounting) and
//! avoidances on edges of the plain route. Both bounds are exact, so both
//! engines must agree on the status, the cost and the edge path of the route
//! and its alternatives.

#![no_main]

use std::sync::{Arc, OnceLock};

use libfuzzer_sys::fuzz_target;
use wayfarer::{
    GeoPoint, LegRequest, LegResult, MobilityProfile, RouteCostingOptions, RoutingEngine,
    RoutingPackage,
};

fn engines() -> &'static (RoutingEngine, RoutingEngine) {
    static ENGINES: OnceLock<(RoutingEngine, RoutingEngine)> = OnceLock::new();
    ENGINES.get_or_init(|| {
        let path = wayfarer_fuzz::monaco_package();
        let landmarks = RoutingPackage::open(path).expect("the Monaco package opens");
        let potentials = RoutingPackage::open(path).expect("the Monaco package opens");
        potentials
            .prepare_potentials()
            .expect("potentials for Monaco");
        (
            RoutingEngine::from_package(Arc::new(landmarks)),
            RoutingEngine::from_package(Arc::new(potentials)),
        )
    })
}

const PROFILES: [MobilityProfile; 6] = [
    MobilityProfile::Wheeled,
    MobilityProfile::OnFoot,
    MobilityProfile::Tracked,
    MobilityProfile::Emergency,
    MobilityProfile::Legged,
    MobilityProfile::Wheeled,
];

struct Input<'a>(&'a [u8]);

impl Input<'_> {
    fn byte(&mut self) -> u8 {
        let Some((&first, rest)) = self.0.split_first() else {
            return 0;
        };
        self.0 = rest;
        first
    }

    fn unit(&mut self) -> f64 {
        f64::from(u16::from_le_bytes([self.byte(), self.byte()])) / f64::from(u16::MAX)
    }

    fn point(&mut self) -> GeoPoint {
        GeoPoint::new(43.724 + 0.028 * self.unit(), 7.405 + 0.035 * self.unit())
    }
}

fn summary(result: &Result<LegResult, wayfarer::RouteFailure>) -> String {
    match result {
        Ok(result) => format!("cost {} edges {:?}", result.cost, result.edge_ids),
        Err(failure) => format!("{:?}", failure.error),
    }
}

fuzz_target!(|data: &[u8]| {
    let (landmarks, potentials) = engines();
    let mut input = Input(data);
    let profile = PROFILES[usize::from(input.byte()) % PROFILES.len()];
    let (origin, destination) = (input.point(), input.point());
    let flags = input.byte();
    let mut request = LegRequest::new(origin, destination, profile);
    request.options.max_correlation_distance_m = 150;
    request.options.max_visited_edge_count = Some(500_000);
    if flags & 1 != 0 {
        request.options.costing = RouteCostingOptions {
            avoid_motorways: f64::from(input.byte()) / 255.0,
            prefer_paved: f64::from(input.byte()) / 255.0,
            avoid_paths: f64::from(input.byte()) / 255.0,
            ..RouteCostingOptions::default()
        };
    }
    if flags & 2 != 0 {
        request.options.alternatives.requested_alternative_count = 1 + usize::from(flags >> 7);
    }
    if flags & 4 != 0 {
        // Edges of the plain route: blocked, avoided, or costed by a
        // multiplier below or above 1.
        let Ok(plain) = landmarks.route_leg(request.clone()) else {
            return;
        };
        if plain.edge_ids.is_empty() {
            return;
        }
        for _ in 0..usize::from(input.byte() % 6) {
            let edge = plain.edge_ids[usize::from(input.byte()) % plain.edge_ids.len()];
            match input.byte() % 4 {
                0 => {
                    request.options.blocked_edge_ids.insert(edge);
                }
                1 => {
                    request.options.avoided_edge_ids.insert(edge);
                }
                _ => {
                    let multiplier = 0.05 + f64::from(input.byte()) / 32.0;
                    request.options.edge_cost_multipliers.insert(edge, multiplier);
                }
            }
        }
    }
    let expected = landmarks.route_leg(request.clone());
    let actual = potentials.route_leg(request);
    let context = format!("{profile:?} {origin:?} -> {destination:?}");
    match (&expected, &actual) {
        (Ok(expected), Ok(actual)) => {
            assert!(
                (expected.cost - actual.cost).abs() <= 1e-9 * expected.cost.abs().max(1.0),
                "{context}: landmarks {} vs potentials {}",
                expected.cost,
                actual.cost
            );
            assert_eq!(expected.edge_ids, actual.edge_ids, "{context}");
            let paths = |result: &LegResult| {
                result
                    .alternatives
                    .iter()
                    .map(|alternative| alternative.edge_ids.clone())
                    .collect::<Vec<_>>()
            };
            assert_eq!(paths(expected), paths(actual), "{context}: alternatives");
        }
        (Err(expected), Err(actual)) => {
            assert_eq!(
                std::mem::discriminant(&expected.error),
                std::mem::discriminant(&actual.error),
                "{context}: {:?} vs {:?}",
                expected.error,
                actual.error
            );
        }
        _ => panic!(
            "{context}: landmarks {} vs potentials {}",
            summary(&expected),
            summary(&actual)
        ),
    }
});

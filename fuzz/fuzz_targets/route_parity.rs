//! Route parity between the bounds, on the Monaco package: CCH potentials on
//! its stored order against plain A* on the straight-line bound alone
//! (`internals::set_potentials("off")`). Both are exact.
//!
//! The input picks a profile, two points in Monaco, costing options,
//! alternatives, and blocks, cost multipliers (raising and discounting) and
//! avoidances on edges of the plain route. Both runs must agree on the
//! status, the cost and the edge path of the route. Alternatives differ by
//! design (penalties against via edges); those of the potentials must be
//! distinct routes within the policy's cost limit.

#![no_main]

use std::sync::{Arc, OnceLock};

use libfuzzer_sys::fuzz_target;
use wayfarer::{
    GeoPoint, LegRequest, LegResult, MobilityProfile, RouteCostingOptions, RoutingEngine,
    RoutingPackage,
};

fn engine() -> &'static RoutingEngine {
    static ENGINE: OnceLock<RoutingEngine> = OnceLock::new();
    ENGINE.get_or_init(|| {
        let package = RoutingPackage::open(wayfarer_fuzz::monaco_package())
            .expect("the Monaco package opens");
        assert_eq!(package.format_version(), 2);
        package.prepare_potentials().expect("potentials for Monaco");
        RoutingEngine::from_package(Arc::new(package))
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

fn route(request: &LegRequest, potentials: &str) -> Result<LegResult, wayfarer::RouteFailure> {
    assert!(wayfarer::internals::set_potentials(potentials));
    engine().route_leg(request.clone())
}

fuzz_target!(|data: &[u8]| {
    let mut input = Input(data);
    let profile = PROFILES[usize::from(input.byte()) % PROFILES.len()];
    let (origin, destination) = (input.point(), input.point());
    let flags = input.byte();
    let mut request = LegRequest::new(origin, destination, profile);
    request.options.max_correlation_distance_m = 150;
    request.options.max_visited_edge_count = Some(2_000_000);
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
        let Ok(plain) = route(&request, "on") else {
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
    let expected = route(&request, "off");
    let actual = route(&request, "on");
    let context = format!("{profile:?} {origin:?} -> {destination:?}");
    match (&expected, &actual) {
        (Ok(expected), Ok(actual)) => {
            assert!(
                (expected.cost - actual.cost).abs() <= 1e-9 * expected.cost.abs().max(1.0),
                "{context}: straight-line {} vs potentials {}",
                expected.cost,
                actual.cost
            );
            assert_eq!(expected.edge_ids, actual.edge_ids, "{context}");
            let policy = request.options.alternatives.normalized();
            assert!(actual.alternatives.len() <= policy.requested_alternative_count);
            let mut seen = vec![actual.edge_ids.clone()];
            for alternative in &actual.alternatives {
                assert!(!alternative.edge_ids.is_empty(), "{context}");
                assert!(!seen.contains(&alternative.edge_ids), "{context}: duplicate");
                seen.push(alternative.edge_ids.clone());
                assert!(
                    alternative.cost.is_finite() && alternative.cost >= 0.0,
                    "{context}: {}",
                    alternative.cost
                );
                assert!(
                    alternative.shared_edge_ratio <= policy.max_shared_edge_ratio + 1e-9,
                    "{context}"
                );
            }
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
            "{context}: straight-line {} vs potentials {}",
            summary(&expected),
            summary(&actual)
        ),
    }
});

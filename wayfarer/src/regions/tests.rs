//! The seam prototype: the cut, the bounds (admissible and consistent, never
//! above the package's), and routes equal to the package search's and the
//! reference oracle's.

use std::cell::RefCell;
use std::sync::Arc;

use wayfarer_package::{DirectionalAccess, RoadType};
use wayfarer_road_graph::{RoadGraph, RoadGraphBuilder, RoutableWay, enhance_road_graph};

use super::*;
use crate::package::search::tie_window;
use crate::reference::ReferenceEngine;
use crate::test_support::{package, test_options};
use crate::{GeoPoint, LegRequest, RoutingEngine, SnapRequest};

/// A deterministic pseudo-random sequence (64-bit LCG).
struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.0 >> 33
    }

    fn unit(&mut self) -> f64 {
        self.next() as f64 / f64::from(1u32 << 31)
    }
}

const GRID: usize = 14;
const STEP: f64 = 0.0012;

/// A jittered `GRID` x `GRID` street grid with mixed classes, some one-ways
/// and footways and a few missing blocks, so ties are rare and the two
/// profiles see different networks.
fn grid_graph() -> RoadGraph {
    let mut random = Lcg(7);
    let mut point = vec![(0.0, 0.0); GRID * GRID];
    for row in 0..GRID {
        for col in 0..GRID {
            point[row * GRID + col] = (
                50.0 + row as f64 * STEP + (random.unit() - 0.5) * STEP * 0.3,
                19.9 + col as f64 * STEP * 1.5 + (random.unit() - 0.5) * STEP * 0.3,
            );
        }
    }
    let node_id = |index: usize| 10_000 + index as i64;
    let mut builder = RoadGraphBuilder::new("seam-grid");
    let mut way_id = 1;
    for row in 0..GRID {
        for col in 0..GRID {
            let from = row * GRID + col;
            for to in [
                (col + 1 < GRID).then_some(from + 1),
                (row + 1 < GRID).then_some(from + GRID),
            ]
            .into_iter()
            .flatten()
            {
                let roll = random.next() % 100;
                if roll < 4 {
                    continue;
                }
                let (road_type, highway) = if row == GRID / 2 || col == GRID / 3 {
                    (RoadType::Primary, "primary")
                } else if roll < 12 {
                    (RoadType::Path, "footway")
                } else if roll < 30 {
                    (RoadType::Secondary, "secondary")
                } else {
                    (RoadType::Residential, "residential")
                };
                let directional_access =
                    if road_type == RoadType::Residential && roll.is_multiple_of(9) {
                        DirectionalAccess::Forward
                    } else {
                        DirectionalAccess::Both
                    };
                builder.push_way(RoutableWay {
                    osm_way_id: way_id,
                    points: vec![point[from], point[to]],
                    source_node_ids: vec![Some(node_id(from)), Some(node_id(to))],
                    road_type,
                    directional_access,
                    highway: Some(highway.into()),
                    ..Default::default()
                });
                way_id += 1;
            }
        }
    }
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    graph
}

fn grid_point(row: usize, col: usize) -> GeoPoint {
    // Between two grid nodes, off any junction.
    GeoPoint::new(
        50.0 + (row as f64 + 0.5) * STEP,
        19.9 + (col as f64 + 0.37) * STEP * 1.5,
    )
}

fn router(graph: &RoadGraph) -> SeamRouter {
    let (package, _) = package(graph, &test_options()).expect("a package");
    SeamRouter::new(Arc::new(package)).expect("a seam")
}

/// Checks the cut: both regions hold a fair share, and every edge between
/// them touches the separator; region ids round-trip.
fn check_cut(router: &SeamRouter) {
    let summary = router.summary().clone();
    let total = summary.node_count[0] + summary.node_count[1];
    for count in summary.node_count {
        assert!(count * 5 >= total, "an unbalanced cut: {summary:?}");
    }
    assert!(summary.separator_node_count > 0 && summary.portal_edge_count > 0);
    let view = router.package.view();
    let separator = router.separator_nodes();
    let mut portals = 0;
    for node in 0..view.node_count as u32 {
        for edge in view.first_out[node as usize]..view.first_out[node as usize + 1] {
            let target = view.edge_target[edge as usize];
            if router.region_of_node(node) != router.region_of_node(target) {
                portals += 1;
                assert!(
                    separator.contains(&node) || separator.contains(&target),
                    "edge {edge} joins the regions away from the separator"
                );
            }
            let id = router.region_edge(edge);
            assert_eq!(id.region, router.region_of_node(node));
            assert_eq!(router.package_edge(id), Some(edge));
        }
    }
    assert_eq!(portals, summary.portal_edge_count);
}

/// Checks the seam potential towards `targets` against every usable edge
/// (consistent) and against the package potential, the exact node-level
/// distance: the overlay's cliques are exact for that metric, so the two
/// agree up to the f32 rounding of the arc weights (lower is admissible
/// too, higher would not be).
fn check_potential(router: &SeamRouter, profile: MobilityProfile, targets: &[u32]) {
    router
        .with_query(profile, &[], &[], |prepared| {
            let g = prepared.view;
            let scratch: [RefCell<PotentialScratch>; 2] = Default::default();
            let (seam, _) = prepared.bounds.potential(
                prepared.topology,
                &router.seam,
                targets,
                prepared.factor,
                &scratch,
            );
            let package_scratch: [RefCell<PotentialScratch>; 2] = Default::default();
            let plan = potential_plan(g, &prepared.query.column, &Default::default(), false)
                .expect("a plan");
            let exact = plan.potential(
                targets,
                false,
                &[],
                &package_scratch[0],
                &package_scratch[1],
            );
            for node in 0..g.node_count as u32 {
                let (bound, distance) = (seam.get(node), exact.get(node));
                if distance.is_finite() {
                    assert!(
                        (bound - distance).abs() <= 1e-6 * distance + 1e-9,
                        "node {node}: seam bound {bound}, package bound {distance}"
                    );
                } else {
                    assert!(bound.is_infinite(), "node {node} reaches no target");
                }
                for edge in g.first_out[node as usize]..g.first_out[node as usize + 1] {
                    let cost = f64::from(prepared.query.column.cost(edge));
                    let head = seam.get(g.edge_target[edge as usize]);
                    if cost.is_finite() && head.is_finite() {
                        assert!(
                            bound <= cost + head + 1e-9 * bound.abs().max(1.0),
                            "edge {edge}: {bound} > {cost} + {head}"
                        );
                    }
                }
            }
            Ok(())
        })
        .expect("a query")
}

/// The seam search, the package-bound A* and the package search on one
/// query; all three must agree on the cost, and the seam route on the
/// edges with the package-bound A* (same search, same ties).
fn compare(
    router: &SeamRouter,
    profile: MobilityProfile,
    origins: &[CorrelatedLocation],
    destinations: &[CorrelatedLocation],
) -> Option<(SeamPath, SeamPath)> {
    let seam = router
        .route(profile, origins, destinations, SeamBound::Regions)
        .unwrap();
    let bounded = router
        .route(profile, origins, destinations, SeamBound::Package)
        .unwrap();
    let package = router
        .package_route(profile, origins, destinations)
        .unwrap();
    match (seam, bounded, package) {
        (None, None, None) => None,
        (Some(seam), Some(bounded), Some(package)) => {
            // Approach costs count: candidates are chosen with them.
            let total = |path: &SeamPath| path.total_cost;
            let close =
                |a: &SeamPath, b: &SeamPath| (total(a) - total(b)).abs() <= tie_window(total(b));
            assert!(
                close(&seam, &package) && close(&bounded, &package),
                "costs differ: seam {} ({}, {}) bounded {} package {} ({}, {})",
                total(&seam),
                seam.origin_index,
                seam.destination_index,
                total(&bounded),
                total(&package),
                package.origin_index,
                package.destination_index,
            );
            assert_eq!(seam.edge_ids, bounded.edge_ids);
            Some((seam, package))
        }
        other => panic!("one search found no path: {other:?}"),
    }
}

#[test]
fn cut_splits_a_grid_at_its_top_separator() {
    let router = router(&grid_graph());
    check_cut(&router);
    let summary = router.summary();
    assert_eq!(summary.component_count, 1);
    assert!(
        summary.separator_node_count <= GRID * 2,
        "a separator of {} nodes for a {GRID}x{GRID} grid",
        summary.separator_node_count
    );
}

#[test]
fn seam_bounds_are_consistent_and_equal_the_package_bounds() {
    let router = router(&grid_graph());
    let nodes = router.package.counts().node_count as u32;
    let separator = router.separator_nodes().to_vec();
    let in_region = |region: u8| {
        (0..nodes)
            .filter(|node| router.region_of_node(*node) == region && !separator.contains(node))
            .collect::<Vec<_>>()
    };
    let (zero, one) = (in_region(0), in_region(1));
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        for targets in [
            vec![zero[0]],
            vec![one[one.len() / 2]],
            vec![zero[zero.len() - 1], one[0]],
            vec![separator[0]],
        ] {
            check_potential(&router, profile, &targets);
        }
    }
}

#[test]
fn seam_routes_equal_the_package_search_and_the_oracle_on_a_grid() {
    let graph = grid_graph();
    let router = router(&graph);
    let oracle = ReferenceEngine::new(&graph, router.package.clone());
    let (mut routes, mut crossing, mut identical) = (0, 0, 0);
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        for (from, to) in [
            ((0, 0), (GRID - 2, GRID - 2)),
            ((GRID - 2, 0), (0, GRID - 2)),
            ((1, 3), (GRID - 3, 9)),
            ((6, 1), (6, GRID - 3)),
            ((GRID - 2, 5), (2, 4)),
            ((3, 10), (10, 2)),
            ((0, 7), (GRID - 2, 7)),
            ((5, 5), (8, 8)),
            ((2, 1), (3, 2)),
        ] {
            let mut request =
                LegRequest::new(grid_point(from.0, from.1), grid_point(to.0, to.1), profile);
            request.options.max_correlation_distance_m = 120;
            let Ok(expected) = oracle.route_leg(request) else {
                continue;
            };
            if expected.origin.edge_id == expected.destination.edge_id {
                continue;
            }
            let (seam, package) = compare(
                &router,
                profile,
                std::slice::from_ref(&expected.origin),
                std::slice::from_ref(&expected.destination),
            )
            .expect("a route");
            assert!(
                (seam.cost - expected.cost).abs() <= 1e-6 * expected.cost.max(1.0),
                "{profile:?} {from:?} -> {to:?}: seam {} oracle {}",
                seam.cost,
                expected.cost
            );
            assert_eq!(
                seam.edge_ids, expected.edge_ids,
                "{profile:?} {from:?} -> {to:?}"
            );
            identical += usize::from(seam.edge_ids == package.edge_ids);
            crossing += usize::from(seam.portal_crossings > 0);
            routes += 1;
        }
    }
    assert!(routes >= 12, "only {routes} routes");
    assert!(
        crossing >= routes / 3,
        "only {crossing} of {routes} routes cross the seam"
    );
    assert_eq!(identical, routes);
}

#[test]
fn seam_routes_from_several_candidates_equal_the_package_search() {
    let router = router(&grid_graph());
    let engine = RoutingEngine::from_package(router.package.clone());
    let snap = |point: GeoPoint, profile| {
        let mut request = SnapRequest::new(point, profile);
        request.max_distance_m = Some(150);
        engine.snap(request).candidates
    };
    let mut found = 0;
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        for (from, to) in [
            ((0, 2), (GRID - 2, 11)),
            ((9, 0), (1, 12)),
            ((4, 4), (11, 6)),
        ] {
            let origins = snap(grid_point(from.0, from.1), profile);
            let destinations = snap(grid_point(to.0, to.1), profile);
            assert!(origins.len() > 1 && destinations.len() > 1);
            found += usize::from(compare(&router, profile, &origins, &destinations).is_some());
        }
    }
    assert_eq!(found, 6);
}

/// The Monaco fixture, compiled as `scripts/check.sh` does.
fn monaco() -> (tempfile::TempDir, Arc<RoutingPackage>) {
    let pbf = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/monaco.osm.pbf");
    let dir = tempfile::tempdir().unwrap();
    let output = dir.path().join("monaco.mrt");
    wayfarer_compiler::compile(
        &wayfarer_compiler::CompileRequest::new(pbf, output.clone())
            .scratch_dir(dir.path().join("scratch"))
            .graph_id("monaco")
            .country("MC"),
        &wayfarer_compiler::CancelToken::new(),
        |_| {},
    )
    .unwrap();
    let package = Arc::new(RoutingPackage::open(&output).unwrap());
    (dir, package)
}

#[test]
fn seam_routes_equal_the_package_search_on_monaco() {
    let (_dir, package) = monaco();
    let router = SeamRouter::new(package.clone()).unwrap();
    check_cut(&router);
    let engine = RoutingEngine::from_package(package);
    let view = router.package.view();
    let nodes = view.node_count as u64;
    let mut random = Lcg(42);
    let (mut routes, mut crossing, mut identical) = (0, 0, 0);
    for case in 0..80 {
        let profile = if case % 2 == 0 {
            MobilityProfile::Wheeled
        } else {
            MobilityProfile::OnFoot
        };
        // At a random node's stored position.
        let mut point = || {
            let [x, y, z] = view.xyz((random.next() % nodes) as u32);
            GeoPoint::new(z.asin().to_degrees(), y.atan2(x).to_degrees())
        };
        let (from, to) = (point(), point());
        let snap = |point| {
            let mut request = SnapRequest::new(point, profile);
            request.max_distance_m = Some(150);
            engine.snap(request).candidates
        };
        let (origins, destinations) = (snap(from), snap(to));
        if origins.is_empty() || destinations.is_empty() {
            continue;
        }
        if let Some((seam, package)) = compare(&router, profile, &origins, &destinations) {
            routes += 1;
            crossing += usize::from(seam.portal_crossings > 0);
            identical += usize::from(seam.edge_ids == package.edge_ids);
        }
    }
    assert!(routes >= 50, "only {routes} routes");
    assert!(
        crossing >= routes / 4,
        "only {crossing} of {routes} routes cross the seam"
    );
    // The package search is bidirectional: only exact ties may differ.
    assert!(
        identical * 10 >= routes * 9,
        "{identical} of {routes} identical"
    );
    let nodes = router.package.counts().node_count as u32;
    let targets = [0, nodes / 3, nodes - 1];
    check_potential(&router, MobilityProfile::Wheeled, &targets);
}

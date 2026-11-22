//! Routing packages: format integrity, and parity of their searches with the
//! reference oracle (`crate::reference`, plain Dijkstra over the road graph's
//! own records) on small synthetic graphs.

use std::path::Path;
use std::sync::Arc;

use tempfile::tempdir;
use wayfarer_package::{DirectionalAccess, MobilityMode, OsmMode, RoadType, TrafficControl};
use wayfarer_road_graph::{
    ResolvedEdgeRestriction, RoadGraph, RoadGraphBuilder, RoadGraphQualityManifest, RoutableWay,
    TrafficControlDirection, TrafficControlPoint, build_road_graph_manifest_with_quality,
    enhance_road_graph, inspect_road_graph_container, serialize_road_graph,
};

use super::*;
use crate::graph_access::GraphAccess;
use crate::graph_access::RouteGraphIndex;
use crate::graph_view::PackageGraphView;
use crate::reference::ReferenceEngine;
use crate::test_support::PackageBuildOptions;
use crate::types::MobilityProfile;
use crate::{GeoPoint, LegRequest, LegResult, RoutingEngine};

fn way(
    osm_way_id: i64,
    a: (f64, f64),
    b: (f64, f64),
    node_a: i64,
    node_b: i64,
    road_type: RoadType,
    directional_access: DirectionalAccess,
) -> RoutableWay {
    let highway = match road_type {
        RoadType::Primary => "primary",
        RoadType::Secondary => "secondary",
        RoadType::Path => "footway",
        _ => "residential",
    };
    RoutableWay {
        osm_way_id,
        points: vec![a, b],
        source_node_ids: vec![Some(node_a), Some(node_b)],
        road_type,
        directional_access,
        highway: Some(highway.into()),
        ..Default::default()
    }
}

/// Irregular 5x5 grid with mixed classes, a one-way street and a footway, so
/// equal-cost ties are rare and both profiles see different networks.
fn grid_graph() -> RoadGraph {
    grid_graph_with_controls(Vec::new())
}

const GRID_LAT: [f64; 5] = [0.0, 0.0011, 0.0023, 0.0031, 0.0044];
const GRID_LNG: [f64; 5] = [0.0, 0.0017, 0.0029, 0.0042, 0.0051];

/// [`grid_graph`] with traffic controls at grid nodes.
fn grid_graph_with_controls(controls: Vec<TrafficControlPoint>) -> RoadGraph {
    grid_graph_with(controls, |_, _, _| {})
}

/// [`grid_graph_with_controls`], each way edited by `edit` with its end
/// nodes' `(row, col)`.
fn grid_graph_with(
    controls: Vec<TrafficControlPoint>,
    edit: impl Fn(&mut RoutableWay, (usize, usize), (usize, usize)),
) -> RoadGraph {
    let (lat, lng) = (GRID_LAT, GRID_LNG);
    let node = |row: usize, col: usize| 1_000 + (row * 10 + col) as i64;
    let mut builder = RoadGraphBuilder::new("compact-grid");
    builder.set_traffic_control_points(controls);
    let mut way_id = 1;
    for row in 0..5 {
        for col in 0..5 {
            if col + 1 < 5 {
                let road_type = if row == 2 {
                    RoadType::Primary
                } else {
                    RoadType::Residential
                };
                let access = if row == 3 && col == 1 {
                    DirectionalAccess::Forward
                } else {
                    DirectionalAccess::Both
                };
                let mut horizontal = way(
                    way_id,
                    (lat[row], lng[col]),
                    (lat[row], lng[col + 1]),
                    node(row, col),
                    node(row, col + 1),
                    road_type,
                    access,
                );
                edit(&mut horizontal, (row, col), (row, col + 1));
                builder.push_way(horizontal);
                way_id += 1;
            }
            if row + 1 < 5 {
                let road_type = if col == 3 && row == 1 {
                    RoadType::Path
                } else if col == 2 {
                    RoadType::Secondary
                } else {
                    RoadType::Residential
                };
                let mut vertical = way(
                    way_id,
                    (lat[row], lng[col]),
                    (lat[row + 1], lng[col]),
                    node(row, col),
                    node(row + 1, col),
                    road_type,
                    DirectionalAccess::Both,
                );
                edit(&mut vertical, (row, col), (row + 1, col));
                builder.push_way(vertical);
                way_id += 1;
            }
        }
    }
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    graph
}

/// T junction W-J-E with branch J-N and a detour E-NE-N.
fn junction_graph() -> RoadGraph {
    let mut builder = RoadGraphBuilder::new("compact-junction");
    let residential = RoadType::Residential;
    let both = DirectionalAccess::Both;
    builder.push_way(way(1, (0.0, 0.0), (0.0, 0.002), 10, 11, residential, both));
    builder.push_way(way(
        2,
        (0.0, 0.002),
        (0.0, 0.004),
        11,
        12,
        residential,
        both,
    ));
    builder.push_way(way(
        3,
        (0.0, 0.002),
        (0.002, 0.002),
        11,
        13,
        residential,
        both,
    ));
    builder.push_way(way(
        4,
        (0.0, 0.004),
        (0.002, 0.004),
        12,
        14,
        residential,
        both,
    ));
    builder.push_way(way(
        5,
        (0.002, 0.004),
        (0.002, 0.002),
        14,
        13,
        residential,
        both,
    ));
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    graph
}

/// Directed edge of OSM way `osm_way_id` from OSM node `from` to `to`.
fn edge_between(graph: &RoadGraph, osm_way_id: i64, from: i64, to: i64) -> u64 {
    graph
        .graph
        .edge_references()
        .find(|edge| {
            let info = graph
                .shared_edge_infos
                .iter()
                .find(|info| info.shared_edge_info_id == edge.weight().shared_edge_info_id)
                .expect("shared info");
            info.osm_way_id == osm_way_id
                && graph.graph.node(edge.source()).source_node_id == Some(from)
                && graph.graph.node(edge.target()).source_node_id == Some(to)
        })
        .map(|edge| edge.weight().edge_id)
        .expect("directed edge")
}

/// A road graph and the routing package written from it (to a file, then
/// opened again).
struct Fixture {
    graph: Arc<RoadGraph>,
    package: Arc<RoutingPackage>,
}

impl Fixture {
    /// The road graph's own view (the reference oracle's).
    fn access(&self) -> GraphAccess<'static> {
        GraphAccess::from_owned(
            self.graph.clone(),
            Arc::new(RouteGraphIndex::build(&self.graph)),
        )
    }

    fn reference(&self) -> ReferenceEngine<'static> {
        ReferenceEngine::from_access(self.access(), self.package.clone())
    }
}

fn persist(graph: &RoadGraph, path: &Path) -> Fixture {
    let bytes = serialize_road_graph(graph).unwrap();
    let container = inspect_road_graph_container(&bytes).unwrap();
    let manifest = build_road_graph_manifest_with_quality(
        graph,
        &container,
        RoadGraphQualityManifest::default(),
    );
    wayfarer_compiler::package_build::write_package(
        graph,
        Some(&manifest),
        path,
        &crate::test_support::test_options(),
    )
    .unwrap();
    Fixture {
        graph: Arc::new(graph.clone()),
        package: Arc::new(RoutingPackage::open(path).unwrap()),
    }
}

/// The reference oracle over the road graph and an engine over its package.
fn engines(fixture: &Fixture) -> (ReferenceEngine<'static>, RoutingEngine) {
    (
        fixture.reference(),
        RoutingEngine::from_package(fixture.package.clone()),
    )
}

/// Requests both engines answer.
trait Router {
    fn route_request(&self, request: LegRequest) -> Result<LegResult, crate::RouteError>;
    fn matrix(&self, request: crate::RouteMatrixRequest) -> crate::RouteMatrixResult;
    fn isochrone(
        &self,
        request: crate::RouteIsochroneRequest,
    ) -> Result<crate::RouteIsochroneResult, crate::RouteError>;
}

impl Router for RoutingEngine {
    fn route_request(&self, request: LegRequest) -> Result<LegResult, crate::RouteError> {
        RoutingEngine::route_leg(self, request).map_err(|failure| failure.error)
    }
    fn matrix(&self, request: crate::RouteMatrixRequest) -> crate::RouteMatrixResult {
        RoutingEngine::matrix(self, request)
    }
    fn isochrone(
        &self,
        request: crate::RouteIsochroneRequest,
    ) -> Result<crate::RouteIsochroneResult, crate::RouteError> {
        RoutingEngine::isochrone(self, request)
    }
}

impl Router for ReferenceEngine<'_> {
    fn route_request(&self, request: LegRequest) -> Result<LegResult, crate::RouteError> {
        ReferenceEngine::route_leg(self, request)
    }
    fn matrix(&self, request: crate::RouteMatrixRequest) -> crate::RouteMatrixResult {
        ReferenceEngine::matrix(self, request)
    }
    fn isochrone(
        &self,
        request: crate::RouteIsochroneRequest,
    ) -> Result<crate::RouteIsochroneResult, crate::RouteError> {
        ReferenceEngine::isochrone(self, request)
    }
}

fn route(
    engine: &(impl Router + ?Sized),
    origin: GeoPoint,
    destination: GeoPoint,
    profile: MobilityProfile,
    configure: impl FnOnce(&mut LegRequest),
) -> Option<LegResult> {
    let mut request = LegRequest::new(origin, destination, profile);
    request.options.max_correlation_distance_m = 60;
    configure(&mut request);
    engine.route_request(request).ok()
}

#[test]
fn package_round_trips_and_rejects_outdated_or_corrupt_bytes() {
    let dir = tempdir().unwrap();
    let path = dir.path().join("grid.mrt");
    let road_graph = grid_graph();
    let metadata = wayfarer_compiler::package_build::package_metadata(&road_graph, None);
    let (writer, report) = wayfarer_compiler::package_build::build_package(
        &road_graph,
        &metadata,
        &PackageBuildOptions::default(),
    )
    .unwrap();
    let bytes = writer.finish();
    let graph = RoutingPackage::from_bytes(&bytes).unwrap();
    assert_eq!(graph.counts().edge_count, road_graph.edge_count());
    assert_eq!(report.edge_count, road_graph.edge_count());
    assert_eq!(graph.graph_id(), road_graph.graph_id);
    assert!(graph.supports_profile(MobilityProfile::Wheeled));
    assert!(graph.supports_profile(MobilityProfile::Default));
    assert!(graph.supports_profile(MobilityProfile::OnFoot));
    assert!(!graph.supports_profile(MobilityProfile::Tracked));

    // Written, opened and verified, with one dataset hash.
    PackageFile::write_file(&path, &bytes).unwrap();
    let opened = RoutingPackage::open(&path).unwrap();
    assert_eq!(opened.dataset_hash(), graph.dataset_hash());
    let verified = RoutingPackage::verify(&path).unwrap();
    assert_eq!(verified.dataset_hash, graph.dataset_hash());
    assert_eq!(verified.byte_len as usize, bytes.len());
    // The same graph builds the same bytes.
    let (again, _) = wayfarer_compiler::package_build::build_package(
        &road_graph,
        &metadata,
        &PackageBuildOptions::default(),
    )
    .unwrap();
    assert_eq!(again.finish(), bytes);

    let mut corrupt = bytes.to_vec();
    corrupt[0] ^= 0xff;
    assert!(matches!(
        RoutingPackage::from_bytes(&corrupt),
        Err(RoutingPackageError::Invalid(_))
    ));
    // A changed header field fails the root checksum.
    let mut corrupt = bytes.to_vec();
    corrupt[HEADER_COUNTS] ^= 1;
    assert!(matches!(
        RoutingPackage::from_bytes(&corrupt),
        Err(RoutingPackageError::Invalid(_))
    ));
    // An out-of-range edge target must be caught by validation, not by a
    // panic during search; verification also reports the section.
    let (offset, _) = graph.file().section_range(Section::EdgeTarget);
    let mut corrupt = bytes.to_vec();
    corrupt[offset..offset + 4].copy_from_slice(&u32::MAX.to_le_bytes());
    assert!(matches!(
        RoutingPackage::from_bytes(&corrupt),
        Err(RoutingPackageError::Invalid(_))
    ));
    // A flipped bit that keeps the structure valid (a cost) opens, but
    // fails verification.
    let (offset, _) = graph.file().section_range(Section::EdgeCost);
    let mut corrupt = bytes.to_vec();
    corrupt[offset] ^= 1;
    assert!(RoutingPackage::from_bytes(&corrupt).is_ok());
    let corrupt_path = dir.path().join("corrupt.mrt");
    PackageFile::write_file(&corrupt_path, &corrupt).unwrap();
    let error = RoutingPackage::verify(&corrupt_path).unwrap_err();
    assert!(error.to_string().contains("EdgeCost"), "{error}");
    // Truncated files fail both.
    PackageFile::write_file(&corrupt_path, &bytes[..bytes.len() - 64]).unwrap();
    assert!(RoutingPackage::open(&corrupt_path).is_err());
    assert!(RoutingPackage::verify(&corrupt_path).is_err());

    // An older format is outdated (recompile), a newer one too new.
    let mut old = bytes.to_vec();
    old[HEADER_VERSION..HEADER_VERSION + 4].copy_from_slice(&0u32.to_le_bytes());
    PackageFile::write_file(&corrupt_path, &old).unwrap();
    for error in [
        RoutingPackage::open(&corrupt_path).unwrap_err(),
        RoutingPackage::verify(&corrupt_path).unwrap_err(),
    ] {
        assert!(matches!(error, RoutingPackageError::Outdated(_)), "{error}");
        assert!(error.to_string().contains("recompile"), "{error}");
    }
    let mut newer = bytes.to_vec();
    newer[HEADER_VERSION..HEADER_VERSION + 4].copy_from_slice(
        &(wayfarer_package::format::ROUTING_PACKAGE_FORMAT_VERSION + 1).to_le_bytes(),
    );
    assert!(matches!(
        RoutingPackage::from_bytes(&newer),
        Err(RoutingPackageError::TooNew(_))
    ));
}

#[test]
fn package_search_matches_reference_costs_on_grid() {
    let dir = tempdir().unwrap();
    let reader = persist(&grid_graph(), &dir.path().join("grid.mrt"));
    let (oracle, compact) = engines(&reader);
    let points = [
        GeoPoint::new(0.00005, 0.0008),
        GeoPoint::new(0.0044, 0.0050),
        GeoPoint::new(0.0023, 0.0035),
        GeoPoint::new(0.0016, 0.0042),
        GeoPoint::new(0.0031, 0.0005),
        GeoPoint::new(0.0039, 0.0029),
    ];
    let mut compared = 0;
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        for origin in points {
            for destination in points {
                if origin == destination {
                    continue;
                }
                let expected = route(&oracle, origin, destination, profile, |_| {});
                let actual = route(&compact, origin, destination, profile, |_| {});
                assert_eq!(
                    expected.is_some(),
                    actual.is_some(),
                    "{profile:?} {origin:?} -> {destination:?}"
                );
                let (Some(expected), Some(actual)) = (expected, actual) else {
                    continue;
                };
                compared += 1;
                assert!(
                    (expected.cost - actual.cost).abs() <= 1e-3 * expected.cost.max(1.0),
                    "{profile:?} {origin:?} -> {destination:?}: reference {} vs compact {}",
                    expected.cost,
                    actual.cost
                );
                assert!(
                    (expected.duration_s - actual.duration_s).abs()
                        <= 1e-3 * expected.duration_s.max(1.0)
                );
            }
        }
    }
    assert!(
        compared > 20,
        "package search served only {compared} routes"
    );
}

/// The grid with signals on the primary row and at a secondary crossing, a
/// stop sign and a give-way sign.
fn signalled_grid_graph() -> RoadGraph {
    let at = |row: usize, col: usize, control| TrafficControlPoint {
        lat: GRID_LAT[row],
        lng: GRID_LNG[col],
        control,
        direction: TrafficControlDirection::Both,
    };
    grid_graph_with_controls(vec![
        at(2, 1, TrafficControl::TrafficSignals),
        at(2, 2, TrafficControl::TrafficSignals),
        at(2, 3, TrafficControl::CrossingSignals),
        at(1, 1, TrafficControl::Stop),
        at(3, 3, TrafficControl::GiveWay),
    ])
}

/// Junction delays enter durations the same way everywhere: routes equal the
/// reference's, legs add up to their routes, matrix cells to the routes
/// between their points, isochrones to the reference's; walks have none and
/// costs do not include them.
#[test]
fn junction_delays_agree_across_routes_legs_matrices_and_isochrones() {
    let dir = tempdir().unwrap();
    let signalled = persist(&signalled_grid_graph(), &dir.path().join("signals.mrt"));
    let plain = persist(&grid_graph(), &dir.path().join("plain.mrt"));
    let (oracle, compact) = engines(&signalled);
    let (_, unsignalled) = engines(&plain);
    let points = grid_points();
    let (mut delayed, mut controlled) = (0, 0);
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        for origin in &points {
            for destination in &points {
                if origin == destination {
                    continue;
                }
                let (Some(expected), Some(actual)) = (
                    route(&oracle, *origin, *destination, profile, |_| {}),
                    route(&compact, *origin, *destination, profile, |_| {}),
                ) else {
                    continue;
                };
                let context = format!("{profile:?} {origin:?} -> {destination:?}");
                assert!(
                    (expected.duration_s - actual.duration_s).abs()
                        <= 1e-4 * expected.duration_s.max(1.0),
                    "{context}: reference {} vs package {}",
                    expected.duration_s,
                    actual.duration_s
                );
                let edges = &actual.legs[0].edges;
                let on_road: f64 = edges.iter().map(|edge| edge.duration_s).sum();
                let at_junctions: f64 = edges.iter().map(|edge| edge.junction_delay_s).sum();
                assert!(
                    (on_road + at_junctions - actual.duration_s).abs()
                        <= 1e-3 * actual.duration_s.max(1.0),
                    "{context}: legs {on_road} + {at_junctions} vs route {}",
                    actual.duration_s
                );
                let maneuvers: f64 = actual.maneuvers.iter().map(|m| m.duration_s).sum();
                assert!(
                    (maneuvers - on_road - at_junctions).abs() <= 1e-6 * maneuvers.max(1.0),
                    "{context}"
                );
                if profile == MobilityProfile::OnFoot {
                    assert_eq!(at_junctions, 0.0, "{context}");
                } else if at_junctions > 0.0 {
                    delayed += 1;
                }
                // Controls change durations only: costs and paths stay.
                let before = route(&unsignalled, *origin, *destination, profile, |_| {}).unwrap();
                assert_eq!(before.edge_ids, actual.edge_ids, "{context}");
                assert!(
                    (before.cost - actual.cost).abs() <= 1e-9 * actual.cost,
                    "{context}"
                );
                assert!(actual.duration_s >= before.duration_s - 1e-6, "{context}");
                if actual.duration_s > before.duration_s + 2.0 {
                    controlled += 1;
                }
            }
        }
        // Pairwise cells are the routes between their points, rows (more
        // targets than the pairwise cut-off) the reference's cells.
        let cells = matrix(&compact, points.clone(), points.clone(), profile);
        for cell in &cells.cells {
            let (Some(duration), Some(route)) = (
                cell.duration_s,
                route(
                    &compact,
                    points[cell.source_index],
                    points[cell.target_index],
                    profile,
                    |_| {},
                ),
            ) else {
                continue;
            };
            assert!(
                (route.duration_s - duration).abs() <= 1e-6 * duration.max(1.0),
                "{profile:?} cell {}->{}: matrix {duration} vs route {}",
                cell.source_index,
                cell.target_index,
                route.duration_s
            );
        }
        let targets = (0..60)
            .map(|index| {
                let (row, col) = (index / 8, index % 8);
                GeoPoint::new(
                    0.0002 + 0.0040 * row as f64 / 7.0,
                    0.0003 + 0.0047 * col as f64 / 7.0,
                )
            })
            .collect::<Vec<_>>();
        let expected = matrix(&oracle, points[..3].to_vec(), targets.clone(), profile);
        let actual = matrix(&compact, points[..3].to_vec(), targets, profile);
        assert_matrix_parity(&expected, &actual);
        for (left, right) in expected.cells.iter().zip(&actual.cells) {
            if let (Some(left), Some(right)) = (left.duration_s, right.duration_s) {
                assert!((left - right).abs() <= 1e-4 * left.max(1.0), "{profile:?}");
            }
        }
        let limits = match profile {
            MobilityProfile::OnFoot => [60.0, 150.0, 300.0],
            _ => [20.0, 45.0, 90.0],
        };
        let request = grid_isochrone_request(profile, limits);
        let expected = oracle.isochrone(request.clone()).unwrap();
        let actual = compact.isochrone(request).unwrap();
        assert_isochrone_parity(&expected, &actual, &format!("{profile:?} signalled"));
    }
    assert!(delayed > 10, "only {delayed} drives met a junction delay");
    assert!(controlled > 5, "only {controlled} drives met a control");
}

#[test]
fn package_search_honours_turn_restriction_keyed_by_osm_via_node() {
    let dir = tempdir().unwrap();
    let mut graph = junction_graph();
    let from = edge_between(&graph, 1, 10, 11);
    let to = edge_between(&graph, 3, 11, 13);
    // Real archives carry the OSM via-node id (here 11), not a graph index.
    graph.set_resolved_restrictions(vec![ResolvedEdgeRestriction::Simple {
        relation_id: 77,
        restriction: "no_left_turn".into(),
        from_edge_id: from,
        to_edge_id: to,
        via_node_id: 11,
        osm_mode_mask: OsmMode::MOTOR_VEHICLE,
        mobility_mode_mask: MobilityMode::WHEELED,
    }]);
    let reader = persist(&graph, &dir.path().join("junction.mrt"));
    let (oracle, compact) = engines(&reader);
    let origin = GeoPoint::new(0.0, 0.0004);
    let destination = GeoPoint::new(0.0016, 0.002);

    let restricted = route(
        &compact,
        origin,
        destination,
        MobilityProfile::Wheeled,
        |_| {},
    )
    .expect("detour exists");
    assert!(
        !restricted
            .edge_ids
            .windows(2)
            .any(|pair| pair == [from, to]),
        "package route used the banned turn: {:?}",
        restricted.edge_ids
    );
    let reference = route(
        &oracle,
        origin,
        destination,
        MobilityProfile::Wheeled,
        |_| {},
    )
    .expect("detour exists");
    assert_eq!(reference.edge_ids, restricted.edge_ids);

    // The restriction is wheeled-only: walkers take the direct branch.
    let walk = route(
        &compact,
        origin,
        destination,
        MobilityProfile::OnFoot,
        |_| {},
    )
    .expect("walk route");
    assert!(walk.edge_ids.windows(2).any(|pair| pair == [from, to]));
}

/// A restriction row that binds no routed mode (mobility mask 0, as a road
/// graph resolved before `restriction:hgv` and the like were left out still
/// carries) bans the turn for no profile, in the package and the oracle.
#[test]
fn restriction_binding_no_routed_mode_bans_no_turn() {
    let dir = tempdir().unwrap();
    let mut graph = junction_graph();
    let from = edge_between(&graph, 1, 10, 11);
    let to = edge_between(&graph, 3, 11, 13);
    graph.set_resolved_restrictions(vec![ResolvedEdgeRestriction::Simple {
        relation_id: 78,
        restriction: "no_left_turn".into(),
        from_edge_id: from,
        to_edge_id: to,
        via_node_id: 11,
        osm_mode_mask: OsmMode::HGV,
        mobility_mode_mask: 0,
    }]);
    let reader = persist(&graph, &dir.path().join("junction.mrt"));
    let (oracle, compact) = engines(&reader);
    let origin = GeoPoint::new(0.0, 0.0004);
    let destination = GeoPoint::new(0.0016, 0.002);
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        let routes = [
            route(&oracle, origin, destination, profile, |_| {}),
            route(&compact, origin, destination, profile, |_| {}),
        ];
        for routed in routes {
            let routed = routed.expect("route");
            assert!(
                routed.edge_ids.windows(2).any(|pair| pair == [from, to]),
                "{profile:?} avoided a turn no profile is banned from: {:?}",
                routed.edge_ids
            );
        }
    }
}

/// Plain routes, matrices and isochrones carry compact edges from correlation
/// to the result, so they never build the edge id lookup; a request
/// that names an edge id does.
#[test]
fn plain_requests_leave_the_edge_id_lookup_unbuilt() {
    let dir = tempdir().unwrap();
    let reader = persist(&grid_graph(), &dir.path().join("grid.mrt"));
    let package = reader.package.clone();
    let engine = RoutingEngine::from_package(package.clone());
    let many_targets = many_grid_targets();
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        for (origin, destination) in grid_points().iter().zip(grid_points().iter().rev()) {
            let routed = route(&engine, *origin, *destination, profile, |_| {});
            assert!(routed.is_some_and(|route| !route.edge_ids.is_empty()));
        }
        // Pairwise cells and rows, few and many sources.
        let cells = matrix(&engine, grid_points(), grid_points(), profile);
        assert!(cells.cells.iter().any(|cell| cell.cost.is_some()));
        let rows = matrix(
            &engine,
            grid_points()[..2].to_vec(),
            many_targets.clone(),
            profile,
        );
        assert!(rows.cells.iter().any(|cell| cell.cost.is_some()));
        let square = matrix(&engine, many_targets.clone(), many_targets.clone(), profile);
        assert!(square.cells.iter().any(|cell| cell.cost.is_some()));
        let limits = isochrone_limits(profile);
        let isochrone = engine
            .isochrone(grid_isochrone_request(profile, limits))
            .unwrap();
        assert!(!isochrone.buckets.is_empty());
        assert!(
            !package.file().edge_id_lookup_built(),
            "{profile:?}: a plain request built the edge id lookup"
        );
    }
    // A request naming an edge id builds it.
    let edge_id = route(
        &engine,
        grid_points()[0],
        grid_points()[1],
        MobilityProfile::Wheeled,
        |_| {},
    )
    .unwrap()
    .edge_ids[0];
    route(
        &engine,
        grid_points()[0],
        grid_points()[1],
        MobilityProfile::Wheeled,
        |request| {
            request.options.blocked_edge_ids.insert(edge_id);
        },
    );
    assert!(package.file().edge_id_lookup_built());
}

#[test]
fn package_search_honours_blocked_edges_like_reference() {
    let dir = tempdir().unwrap();
    let graph = junction_graph();
    let reader = persist(&graph, &dir.path().join("junction.mrt"));
    let (oracle, compact) = engines(&reader);
    let blocked = edge_between(&graph, 3, 11, 13);
    let origin = GeoPoint::new(0.0, 0.0004);
    let destination = GeoPoint::new(0.0016, 0.002);
    let block = |request: &mut LegRequest| {
        request.options.blocked_edge_ids.insert(blocked);
    };

    let direct = route(
        &compact,
        origin,
        destination,
        MobilityProfile::Wheeled,
        |_| {},
    )
    .unwrap();
    assert!(direct.edge_ids.contains(&blocked));
    let detour = route(
        &compact,
        origin,
        destination,
        MobilityProfile::Wheeled,
        block,
    )
    .unwrap();
    assert!(!detour.edge_ids.contains(&blocked));
    let reference = route(
        &oracle,
        origin,
        destination,
        MobilityProfile::Wheeled,
        block,
    )
    .unwrap();
    assert_eq!(reference.edge_ids, detour.edge_ids);
    assert!((reference.cost - detour.cost).abs() <= 1e-3 * reference.cost);
}

fn grid_points() -> Vec<GeoPoint> {
    vec![
        GeoPoint::new(0.00005, 0.0008),
        GeoPoint::new(0.0044, 0.0050),
        GeoPoint::new(0.0023, 0.0035),
        GeoPoint::new(0.0016, 0.0042),
        GeoPoint::new(0.0031, 0.0005),
        GeoPoint::new(0.0039, 0.0029),
    ]
}

fn matrix(
    engine: &(impl Router + ?Sized),
    sources: Vec<GeoPoint>,
    targets: Vec<GeoPoint>,
    profile: MobilityProfile,
) -> crate::RouteMatrixResult {
    let mut request = crate::RouteMatrixRequest::new(sources, targets, profile);
    request.options.max_correlation_distance_m = 60;
    engine.matrix(request)
}

/// Compact cells are exact shortest paths, as the reference cells are: equal
/// statuses and costs (up to the compact columns' f32 rounding).
fn assert_matrix_parity(reference: &crate::RouteMatrixResult, compact: &crate::RouteMatrixResult) {
    assert_eq!(reference.cells.len(), compact.cells.len());
    for (expected, actual) in reference.cells.iter().zip(&compact.cells) {
        let cell = (expected.source_index, expected.target_index);
        assert_eq!(expected.status, actual.status, "cell {cell:?}");
        let (Some(expected_cost), Some(actual_cost)) = (expected.cost, actual.cost) else {
            continue;
        };
        assert!(
            (actual_cost - expected_cost).abs() <= 1e-4 * expected_cost.max(1.0),
            "cell {cell:?}: reference {expected_cost} vs compact {actual_cost}"
        );
    }
}

#[test]
fn package_matrix_matches_reference_matrix_on_grid() {
    let dir = tempdir().unwrap();
    let reader = persist(&grid_graph(), &dir.path().join("grid.mrt"));
    let (oracle, compact) = engines(&reader);
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        let expected = matrix(&oracle, grid_points(), grid_points(), profile);
        let actual = matrix(&compact, grid_points(), grid_points(), profile);
        assert_matrix_parity(&expected, &actual);
        // A pairwise cell is the route between the same two points.
        for cell in &actual.cells {
            if cell.source_index == cell.target_index {
                continue;
            }
            let (Some(cost), Some(route)) = (
                cell.cost,
                route(
                    &compact,
                    grid_points()[cell.source_index],
                    grid_points()[cell.target_index],
                    profile,
                    |_| {},
                ),
            ) else {
                continue;
            };
            assert!(
                (route.cost - cost).abs() <= 1e-6 * cost.max(1.0),
                "{profile:?} cell {}->{}: matrix {cost} vs route {}",
                cell.source_index,
                cell.target_index,
                route.cost
            );
        }
    }
}

#[test]
fn package_one_to_many_matrix_matches_reference_matrix_on_grid() {
    let dir = tempdir().unwrap();
    let reader = persist(&grid_graph(), &dir.path().join("grid.mrt"));
    let (oracle, compact) = engines(&reader);
    // More targets than the pairwise cut-off, so rows share one expansion.
    let targets = (0..60)
        .map(|index| {
            let (row, col) = (index / 8, index % 8);
            GeoPoint::new(
                0.0002 + 0.0040 * row as f64 / 7.0,
                0.0003 + 0.0047 * col as f64 / 7.0,
            )
        })
        .collect::<Vec<_>>();
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        let expected = matrix(
            &oracle,
            grid_points()[..3].to_vec(),
            targets.clone(),
            profile,
        );
        let actual = matrix(
            &compact,
            grid_points()[..3].to_vec(),
            targets.clone(),
            profile,
        );
        assert_matrix_parity(&expected, &actual);
        assert!(
            actual
                .cells
                .iter()
                .filter(|cell| cell.cost.is_some())
                .count()
                > 100
        );
    }
}

#[test]
fn package_isochrone_matches_reference_isochrone_on_grid() {
    let dir = tempdir().unwrap();
    let reader = persist(&grid_graph(), &dir.path().join("grid.mrt"));
    let (oracle, compact) = engines(&reader);
    for (profile, limits) in [
        (MobilityProfile::Wheeled, [20.0, 45.0, 90.0]),
        (MobilityProfile::OnFoot, [60.0, 150.0, 300.0]),
    ] {
        let mut request = crate::RouteIsochroneRequest::new(
            GeoPoint::new(0.0023, 0.0035),
            profile,
            limits
                .iter()
                .map(|limit| crate::RouteIsochroneContour {
                    contour: *limit,
                    metric: crate::RouteIsochroneMetric::TimeSeconds,
                })
                .chain(std::iter::once(crate::RouteIsochroneContour {
                    contour: limits[1],
                    metric: crate::RouteIsochroneMetric::Cost,
                }))
                .collect(),
        );
        request.options.max_correlation_distance_m = 60;
        let expected = oracle.isochrone(request.clone()).unwrap();
        let actual = compact.isochrone(request).unwrap();
        assert_isochrone_parity(&expected, &actual, &format!("{profile:?}"));
    }
}

/// Directed grid edge from grid node `from` to `to`, as (row, col).
fn grid_edge(graph: &RoadGraph, from: (usize, usize), to: (usize, usize)) -> u64 {
    let node = |(row, col): (usize, usize)| Some(1_000 + (row * 10 + col) as i64);
    graph
        .graph
        .edge_references()
        .find(|edge| {
            graph.graph.node(edge.source()).source_node_id == node(from)
                && graph.graph.node(edge.target()).source_node_id == node(to)
        })
        .map(|edge| edge.weight().edge_id)
        .expect("grid edge")
}

/// Blocks, avoids and multiplies grid edges, whole and by range, including
/// multipliers below 1 (which become discount gates). Returns the blocked
/// edges.
fn apply_per_edge_options(graph: &RoadGraph, options: &mut crate::RouteOptions) -> Vec<u64> {
    let blocked = vec![
        grid_edge(graph, (2, 1), (2, 2)),
        grid_edge(graph, (2, 2), (2, 1)),
        grid_edge(graph, (4, 2), (4, 3)),
    ];
    options.blocked_edge_ids.extend(&blocked[..2]);
    options.blocked_edge_ranges.push(crate::RouteEdgeRange {
        edge_id: blocked[2],
        start_fraction: 0.4,
        end_fraction: 0.6,
    });
    options
        .avoided_edge_ids
        .insert(grid_edge(graph, (2, 3), (2, 2)));
    options
        .avoided_edge_ranges
        .push(crate::RouteEdgeAvoidanceRange {
            edge_id: grid_edge(graph, (1, 0), (1, 1)),
            start_fraction: 0.0,
            end_fraction: 0.5,
        });
    for (from, to, multiplier) in [
        ((1, 2), (2, 2), 3.0),
        ((2, 2), (1, 2), 3.0),
        ((2, 3), (2, 4), 0.3),
        ((3, 2), (3, 3), 0.5),
        ((3, 3), (3, 2), 0.5),
    ] {
        options
            .edge_cost_multipliers
            .insert(grid_edge(graph, from, to), multiplier);
    }
    options
        .edge_cost_multiplier_ranges
        .push(crate::RouteEdgeCostMultiplierRange {
            edge_id: grid_edge(graph, (0, 1), (0, 2)),
            start_fraction: 0.2,
            end_fraction: 0.7,
            multiplier: 4.0,
        });
    blocked
}

/// More targets than any profile's pairwise cut-off, so rows share one
/// expansion.
fn many_grid_targets() -> Vec<GeoPoint> {
    (0..120)
        .map(|index| {
            let (row, col) = (index / 10, index % 10);
            GeoPoint::new(
                0.0002 + 0.0040 * row as f64 / 11.0,
                0.0003 + 0.0047 * col as f64 / 9.0,
            )
        })
        .collect()
}

#[test]
fn package_matrix_honours_per_edge_options_like_reference() {
    let dir = tempdir().unwrap();
    let graph = grid_graph();
    let reader = persist(&graph, &dir.path().join("grid.mrt"));
    let (oracle, compact) = engines(&reader);
    let request = |sources: &[GeoPoint], targets: &[GeoPoint], profile| {
        let mut request =
            crate::RouteMatrixRequest::new(sources.to_vec(), targets.to_vec(), profile);
        request.options.max_correlation_distance_m = 60;
        let blocked = apply_per_edge_options(&graph, &mut request.options);
        (request, blocked)
    };
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        // Pairwise (few targets) and one-to-many rows (more than the cut-off).
        for (sources, targets) in [
            (grid_points(), grid_points()),
            (grid_points()[..3].to_vec(), many_grid_targets()),
        ] {
            let (request, blocked) = request(&sources, &targets, profile);
            let expected = oracle.matrix(request.clone());
            let actual = compact.matrix(request.clone());
            assert_matrix_parity(&expected, &actual);
            for cell in &actual.cells {
                assert!(
                    !cell.edge_ids.iter().any(|edge| blocked.contains(edge)),
                    "{profile:?} cell {}->{} crosses a blocked edge",
                    cell.source_index,
                    cell.target_index
                );
            }
            let plain = matrix(&compact, sources.clone(), targets.clone(), profile);
            assert!(
                plain
                    .cells
                    .iter()
                    .zip(&actual.cells)
                    .any(|(plain, actual)| plain.cost != actual.cost),
                "{profile:?}: the options changed no cell"
            );
            if targets.len() > sources.len() {
                continue;
            }
            // Pairwise cells are exact: the reference route with the same
            // options.
            for cell in &actual.cells {
                if cell.source_index == cell.target_index {
                    continue;
                }
                let reference = route(
                    &oracle,
                    sources[cell.source_index],
                    targets[cell.target_index],
                    profile,
                    |route| route.options = request.options.clone(),
                );
                assert_eq!(cell.cost.is_some(), reference.is_some());
                let (Some(cost), Some(reference)) = (cell.cost, reference) else {
                    continue;
                };
                assert!(
                    (reference.cost - cost).abs() <= 1e-3 * cost.max(1.0),
                    "{profile:?} cell {}->{}: matrix {cost} vs reference route {}",
                    cell.source_index,
                    cell.target_index,
                    reference.cost
                );
            }
        }
    }
}

fn assert_isochrone_parity(
    expected: &crate::RouteIsochroneResult,
    actual: &crate::RouteIsochroneResult,
    context: &str,
) {
    assert_eq!(expected.buckets.len(), actual.buckets.len());
    for (expected, actual) in expected.buckets.iter().zip(&actual.buckets) {
        let bucket = format!("{context} {:?} {}", expected.metric, expected.contour);
        assert_eq!(expected.edge_ids, actual.edge_ids, "{bucket}");
        // Two equal-cost predecessors can make either engine report the same
        // crossing twice (equal up to f32 summation order, which moves its
        // fraction by about 1e-7 of an edge); compare distinct crossings.
        let distinct = |fragments: &[crate::RouteIsochroneBoundaryFragment]| {
            let mut kept: Vec<crate::RouteIsochroneBoundaryFragment> = Vec::new();
            for fragment in fragments {
                if !kept.iter().any(|seen| {
                    seen.edge_id == fragment.edge_id
                        && (seen.end_fraction - fragment.end_fraction).abs() < 1e-6
                        && (seen.start_metric - fragment.start_metric).abs()
                            < 1e-6 * fragment.start_metric.abs().max(1.0)
                }) {
                    kept.push(fragment.clone());
                }
            }
            kept
        };
        let (expected_fragments, actual_fragments) = (
            distinct(&expected.boundary_fragments),
            distinct(&actual.boundary_fragments),
        );
        assert_eq!(expected_fragments.len(), actual_fragments.len(), "{bucket}");
        for (left, right) in expected_fragments.iter().zip(&actual_fragments) {
            assert_eq!(left.edge_id, right.edge_id, "{bucket}");
            assert!(
                (left.end_fraction - right.end_fraction).abs() < 1e-4,
                "{bucket}"
            );
            assert!(left.end.distance_to(right.end) < 0.05, "{bucket}");
        }
    }
}

fn grid_isochrone_request(
    profile: MobilityProfile,
    limits: [f64; 3],
) -> crate::RouteIsochroneRequest {
    let mut request = crate::RouteIsochroneRequest::new(
        GeoPoint::new(0.0023, 0.0035),
        profile,
        limits
            .iter()
            .map(|limit| crate::RouteIsochroneContour {
                contour: *limit,
                metric: crate::RouteIsochroneMetric::TimeSeconds,
            })
            .chain(std::iter::once(crate::RouteIsochroneContour {
                contour: limits[1],
                metric: crate::RouteIsochroneMetric::Cost,
            }))
            .collect(),
    );
    request.options.max_correlation_distance_m = 60;
    request
}

#[test]
fn package_isochrone_honours_per_edge_options_like_reference() {
    let dir = tempdir().unwrap();
    let graph = grid_graph();
    let reader = persist(&graph, &dir.path().join("grid.mrt"));
    let (oracle, compact) = engines(&reader);
    for (profile, limits) in [
        (MobilityProfile::Wheeled, [20.0, 45.0, 90.0]),
        (MobilityProfile::OnFoot, [60.0, 150.0, 300.0]),
    ] {
        let plain = compact
            .isochrone(grid_isochrone_request(profile, limits))
            .unwrap();
        let mut request = grid_isochrone_request(profile, limits);
        let blocked = apply_per_edge_options(&graph, &mut request.options);
        let expected = oracle.isochrone(request.clone()).unwrap();
        let actual = compact.isochrone(request).unwrap();
        assert_isochrone_parity(&expected, &actual, &format!("{profile:?}"));
        assert!(
            actual
                .buckets
                .iter()
                .all(|bucket| !bucket.edge_ids.iter().any(|edge| blocked.contains(edge))),
            "{profile:?}: a blocked edge was reached"
        );
        assert_ne!(
            plain.buckets, actual.buckets,
            "{profile:?}: the options changed nothing"
        );
    }
}

#[test]
fn empty_dynamic_overlay_keeps_requests_on_the_package() {
    let dir = tempdir().unwrap();
    let reader = persist(&grid_graph(), &dir.path().join("grid.mrt"));
    let (_, compact) = engines(&reader);
    let empty = Some(crate::RouteDynamicOverlay::default());
    let points = grid_points();
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        let plain = route(&compact, points[0], points[1], profile, |_| {}).unwrap();
        let overlaid = route(&compact, points[0], points[1], profile, |request| {
            request.options.dynamic_overlay = empty.clone();
        })
        .unwrap();
        assert_eq!(plain.edge_ids, overlaid.edge_ids);
        assert_eq!(plain.cost, overlaid.cost);
        let mut request = crate::RouteMatrixRequest::new(points.clone(), points.clone(), profile);
        request.options.max_correlation_distance_m = 60;
        let plain = compact.matrix(request.clone());
        request.options.dynamic_overlay = empty.clone();
        assert_eq!(plain.cells, compact.matrix(request).cells);

        let request = grid_isochrone_request(profile, [20.0, 45.0, 90.0]);
        let plain = compact.isochrone(request.clone()).unwrap();
        let mut overlaid = request;
        overlaid.options.dynamic_overlay = empty.clone();
        assert_eq!(plain.buckets, compact.isochrone(overlaid).unwrap().buckets);
    }
}

#[test]
fn isochrone_stops_at_its_expansion_budget_and_reports_truncation() {
    let dir = tempdir().unwrap();
    let reader = persist(&grid_graph(), &dir.path().join("grid.mrt"));
    let (oracle, compact) = engines(&reader);
    let engines: [(&str, &dyn Router); 2] = [("reference", &oracle), ("compact", &compact)];
    for (name, engine) in engines {
        for (profile, limits) in [
            (MobilityProfile::Wheeled, [20.0, 45.0, 90.0]),
            (MobilityProfile::OnFoot, [60.0, 150.0, 300.0]),
        ] {
            let context = format!("{name} {profile:?}");
            let mut request = grid_isochrone_request(profile, limits);
            let full = engine.isochrone(request.clone()).unwrap();
            assert!(!full.truncated, "{context}");

            // A budget the search never reaches changes nothing.
            request.max_expansion_edge_count = Some(full.visited_edge_count);
            let exact = engine.isochrone(request.clone()).unwrap();
            assert!(!exact.truncated, "{context}");
            assert_eq!(exact.buckets, full.buckets, "{context}");

            let budget = full.visited_edge_count / 3;
            request.max_expansion_edge_count = Some(budget);
            let cut = engine.isochrone(request).unwrap();
            assert!(cut.truncated, "{context}");
            // The search stops at the first edge over budget.
            assert_eq!(cut.visited_edge_count, budget + 1, "{context}");
            assert!(
                cut.settled_node_count < full.settled_node_count,
                "{context}"
            );
            // What a truncated search reports is reachable, just incomplete.
            for (cut, full) in cut.buckets.iter().zip(&full.buckets) {
                assert!(
                    cut.edge_ids
                        .iter()
                        .all(|edge| full.edge_ids.binary_search(edge).is_ok()),
                    "{context}"
                );
            }
            assert!(
                cut.buckets
                    .iter()
                    .zip(&full.buckets)
                    .any(|(cut, full)| cut.edge_ids.len() < full.edge_ids.len()),
                "{context}"
            );
        }
    }
}

fn budget_matrix(
    engine: &dyn Router,
    sources: &[GeoPoint],
    targets: &[GeoPoint],
    profile: MobilityProfile,
    budget: Option<usize>,
) -> crate::RouteMatrixResult {
    let mut request = crate::RouteMatrixRequest::new(sources.to_vec(), targets.to_vec(), profile);
    request.options.max_correlation_distance_m = 60;
    request.options.max_visited_edge_count = budget;
    engine.matrix(request)
}

/// The grid with the three ways at node (2, 3) destination-only: a block
/// that [`grid_points`] start and end both inside and around.
fn grid_with_private_block() -> RoadGraph {
    grid_graph_with(Vec::new(), |way, from, to| {
        way.is_destination_only = matches!(
            (from, to),
            ((2, 2), (2, 3)) | ((2, 3), (2, 4)) | ((1, 3), (2, 3))
        );
    })
}

/// Routes, matrices and isochrones confined to destination-only pockets
/// (drives) and not (walkers) agree with the reference.
#[test]
fn destination_only_pockets_match_the_reference() {
    use crate::matrix::{MatrixMode, test_mode};
    let dir = tempdir().unwrap();
    let graph = grid_with_private_block();
    let reader = persist(&graph, &dir.path().join("grid.mrt"));
    let (oracle, compact) = engines(&reader);
    for profile in [
        MobilityProfile::Wheeled,
        MobilityProfile::Tracked,
        MobilityProfile::OnFoot,
    ] {
        for origin in grid_points() {
            for destination in grid_points() {
                let context = format!("{profile:?} {origin:?} -> {destination:?}");
                let expected = route(&oracle, origin, destination, profile, |_| {});
                for walks_off in [false, true] {
                    crate::package::search::walks::set_off(walks_off);
                    let actual = route(&compact, origin, destination, profile, |_| {});
                    crate::package::search::walks::set_off(false);
                    let (Some(expected), Some(actual)) = (&expected, actual) else {
                        panic!("{context}: only one engine routed");
                    };
                    assert!(
                        (expected.cost - actual.cost).abs() <= 1e-3 * expected.cost.max(1.0),
                        "{context} walks off {walks_off}: {} vs {}",
                        expected.cost,
                        actual.cost
                    );
                    assert_eq!(
                        expected.diagnostics.destination_only_relaxed_retry_count,
                        actual.diagnostics.destination_only_relaxed_retry_count,
                        "{context}"
                    );
                }
            }
        }
        let expected = matrix(&oracle, grid_points(), grid_points(), profile);
        for mode in [MatrixMode::Pairwise, MatrixMode::Rows] {
            test_mode::set(Some(mode));
            let actual = matrix(&compact, grid_points(), grid_points(), profile);
            test_mode::set(None);
            assert_matrix_parity(&expected, &actual);
        }
        for origin in [
            GeoPoint::new(0.0023, 0.0035),
            GeoPoint::new(0.00005, 0.0008),
        ] {
            let limits = match profile {
                MobilityProfile::OnFoot => [60.0, 150.0, 300.0],
                _ => [20.0, 45.0, 90.0],
            };
            let mut request = grid_isochrone_request(profile, limits);
            request.origin = origin;
            let expected = oracle.isochrone(request.clone()).unwrap();
            let actual = compact.isochrone(request).unwrap();
            assert_isochrone_parity(&expected, &actual, &format!("{profile:?} {origin:?}"));
        }
    }
}

/// The grid with its 2x2 north-west corner ringed by destination-only ways:
/// a drive between the corner and the rest needs the relaxed pass.
fn grid_with_private_ring() -> RoadGraph {
    grid_graph_with(Vec::new(), |way, from, to| {
        way.is_destination_only = matches!(
            (from, to),
            ((0, 1), (0, 2)) | ((1, 1), (1, 2)) | ((1, 0), (2, 0)) | ((1, 1), (2, 1))
        );
    })
}

/// A matrix's relaxed pass searches only the cells the strict pass could not
/// connect (in rows, only their sources), and every cell is the one the old
/// relaxed re-run of the whole matrix gave.
#[test]
fn the_relaxed_matrix_pass_searches_only_retried_cells() {
    use crate::matrix::{MatrixMode, test_mode};
    let dir = tempdir().unwrap();
    let reader = persist(&grid_with_private_ring(), &dir.path().join("grid.mrt"));
    let (_, compact) = engines(&reader);
    // Source 0 is inside the ring (on the corner's top edge), the other
    // points outside it.
    let sources = grid_points();
    let targets = grid_points()[1..].to_vec();
    let run = |mode, full| {
        test_mode::set(Some(mode));
        test_mode::set_full_relaxed_pass(full);
        let result = matrix(
            &compact,
            sources.clone(),
            targets.clone(),
            MobilityProfile::Wheeled,
        );
        test_mode::set_full_relaxed_pass(false);
        test_mode::set(None);
        result
    };
    for mode in [MatrixMode::Pairwise, MatrixMode::Rows] {
        let only = run(mode, false);
        let full = run(mode, true);
        assert_eq!(only.cells, full.cells, "{mode:?}");
        // The corner reaches every target across the ring.
        for cell in &only.cells {
            assert_eq!(
                cell.status,
                crate::RouteMatrixCellStatus::Reachable,
                "{mode:?} {cell:?}"
            );
        }
        let strict = sources.len();
        assert_eq!(full.source_expansion_count, 2 * strict, "{mode:?}");
        assert_eq!(only.source_expansion_count, strict + 1, "{mode:?}");
        assert!(
            only.visited_edge_count < full.visited_edge_count,
            "{mode:?}"
        );
    }
}

/// The 60 matrix targets of the one-to-many tests, a lattice over the grid.
fn grid_lattice() -> Vec<GeoPoint> {
    (0..60)
        .map(|index| {
            let (row, col) = (index / 8, index % 8);
            GeoPoint::new(
                0.0002 + 0.0040 * row as f64 / 7.0,
                0.0003 + 0.0047 * col as f64 / 7.0,
            )
        })
        .collect()
}

/// `engine.nearest` against the `k` cheapest reachable cells of the full
/// matrix row (or column) searched by rows, as the early-stopping row is:
/// the same targets in the same order, at the same costs. Returns the
/// nearest search's visited edges and the full row's.
fn assert_nearest_is_the_cheapest_cells(
    engine: &RoutingEngine,
    source: GeoPoint,
    targets: &[GeoPoint],
    k: usize,
    profile: MobilityProfile,
    direction: crate::NearestDirection,
    configure: impl Fn(&mut crate::RouteOptions),
) -> (usize, usize) {
    use crate::matrix::{MatrixMode, test_mode};
    let mut request = crate::NearestRequest::new(source, targets.to_vec(), k, profile);
    request.direction = direction;
    request.options.max_correlation_distance_m = 60;
    configure(&mut request.options);
    let (sources, cells_to) = match direction {
        crate::NearestDirection::FromSource => (vec![source], targets.to_vec()),
        crate::NearestDirection::ToSource => (targets.to_vec(), vec![source]),
    };
    let mut matrix_request = crate::RouteMatrixRequest::new(sources, cells_to, profile);
    matrix_request.options = request.options.clone();
    test_mode::set(Some(MatrixMode::Rows));
    let full = engine.matrix(matrix_request);
    test_mode::set(None);
    let mut expected = full
        .cells
        .iter()
        .filter(|cell| cell.status == crate::RouteMatrixCellStatus::Reachable)
        .map(|cell| {
            let target = match direction {
                crate::NearestDirection::FromSource => cell.target_index,
                crate::NearestDirection::ToSource => cell.source_index,
            };
            (target, cell.cost.unwrap(), cell.edge_ids.clone())
        })
        .collect::<Vec<_>>();
    expected.sort_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)));
    expected.truncate(k);

    test_mode::set(Some(MatrixMode::Rows));
    let result = engine.nearest(request).unwrap();
    test_mode::set(None);
    let context = format!("{profile:?} {direction:?} k={k} from {source:?}");
    assert_eq!(
        result
            .hits
            .iter()
            .map(|hit| hit.target_index)
            .collect::<Vec<_>>(),
        expected
            .iter()
            .map(|(target, _, _)| *target)
            .collect::<Vec<_>>(),
        "{context}"
    );
    for (hit, (_, cost, edge_ids)) in result.hits.iter().zip(&expected) {
        assert!(
            (hit.cost - cost).abs() <= 1e-9 * cost.max(1.0),
            "{context}: target {} costs {} against {cost}",
            hit.target_index,
            hit.cost
        );
        assert_eq!(&hit.edge_ids, edge_ids, "{context}");
    }
    (result.visited_edge_count, full.visited_edge_count)
}

/// Nearest targets are the cheapest cells of the matrix row (or column),
/// for every `k`, and a row that stops early examines fewer edges than the
/// full row.
#[test]
fn nearest_targets_are_the_cheapest_matrix_cells_on_grid() {
    let dir = tempdir().unwrap();
    let reader = persist(&grid_graph(), &dir.path().join("grid.mrt"));
    let (_, compact) = engines(&reader);
    let targets = grid_lattice();
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        let (mut early, mut full) = (0, 0);
        for source in grid_points() {
            for k in [1, 3, 10, 60] {
                for direction in [
                    crate::NearestDirection::FromSource,
                    crate::NearestDirection::ToSource,
                ] {
                    let (visited, full_visited) = assert_nearest_is_the_cheapest_cells(
                        &compact,
                        source,
                        &targets,
                        k,
                        profile,
                        direction,
                        |_| {},
                    );
                    if k == 1 && direction == crate::NearestDirection::FromSource {
                        early += visited;
                        full += full_visited;
                    }
                }
            }
        }
        assert!(
            early * 4 < full,
            "{profile:?}: nearest visited {early} edges, the full rows {full}"
        );
    }
}

/// With destination-only pockets the cells are the strict matrix's: a
/// target inside the private block is reached as a route to it is, and the
/// others never cross the block, though the relaxed row that bounds the
/// search does.
#[test]
fn nearest_targets_keep_the_matrix_destination_only_semantics() {
    let dir = tempdir().unwrap();
    let reader = persist(&grid_with_private_block(), &dir.path().join("grid.mrt"));
    let (_, compact) = engines(&reader);
    let mut targets = grid_lattice();
    targets.push(GeoPoint::new(0.0023, 0.00465));
    for source in [GeoPoint::new(0.0023, 0.0008), GeoPoint::new(0.0027, 0.0051)] {
        for k in [1, 2, 5, 20, 61] {
            assert_nearest_is_the_cheapest_cells(
                &compact,
                source,
                &targets,
                k,
                MobilityProfile::Wheeled,
                crate::NearestDirection::FromSource,
                |options| options.max_correlation_distance_m = 15,
            );
        }
    }
}

#[test]
fn nearest_requests_are_validated() {
    let dir = tempdir().unwrap();
    let reader = persist(&grid_graph(), &dir.path().join("grid.mrt"));
    let (_, compact) = engines(&reader);
    let source = grid_points()[0];
    let request = |k, targets: Vec<GeoPoint>| {
        compact.nearest(crate::NearestRequest::new(
            source,
            targets,
            k,
            MobilityProfile::Wheeled,
        ))
    };
    for (k, targets) in [(0, grid_points()), (9, grid_points()[..3].to_vec())] {
        assert!(matches!(
            request(k, targets),
            Err(crate::RouteError::InvalidRequest { .. })
        ));
    }
    assert!(matches!(
        request(1, vec![source; crate::limits::MAX_NEAREST_TARGETS + 1]),
        Err(crate::RouteError::RequestTooLarge { .. })
    ));
    // A target off the network is listed, not an error; the source is one.
    let far = GeoPoint::new(0.5, 0.5);
    let result = request(2, vec![far, grid_points()[1], grid_points()[2]]).unwrap();
    assert_eq!(result.no_candidate_target_indexes, vec![0]);
    assert_eq!(result.hits.len(), 2);
    assert!(matches!(
        compact.nearest(crate::NearestRequest::new(
            far,
            grid_points(),
            1,
            MobilityProfile::Wheeled
        )),
        Err(crate::RouteError::NoCandidate { endpoint: "origin" })
    ));
}

/// A target no source can reach in a strict pass (ringed by destination-only
/// ways) leaves the strict rows instead of making each flood the grid; every
/// cell is the one the flooding rows gave.
#[test]
fn matrix_rows_skip_targets_their_sources_cannot_reach() {
    use crate::matrix::{MatrixMode, test_mode};
    let dir = tempdir().unwrap();
    let reader = persist(&grid_with_private_ring(), &dir.path().join("grid.mrt"));
    let (_, compact) = engines(&reader);
    // Target 0 is inside the ring, the sources outside it.
    let sources = grid_points()[1..].to_vec();
    let targets = grid_points();
    let run = |keep| {
        test_mode::set(Some(MatrixMode::Rows));
        test_mode::set_keep_unreachable_targets(keep);
        let result = matrix(
            &compact,
            sources.clone(),
            targets.clone(),
            MobilityProfile::Wheeled,
        );
        test_mode::set_keep_unreachable_targets(false);
        test_mode::set(None);
        result
    };
    let (skipped, kept) = (run(false), run(true));
    assert_eq!(skipped.cells, kept.cells);
    assert!(
        skipped
            .cells
            .iter()
            .all(|cell| cell.status == crate::RouteMatrixCellStatus::Reachable)
    );
    assert!(skipped.visited_edge_count < kept.visited_edge_count);
}

/// A drive between the block's two sides goes around it (row 2 is the
/// primary road through it); emergency vehicles, and a SAR request whose
/// `legal_access` names `private`, go straight on.
#[test]
fn a_through_drive_avoids_a_destination_only_block() {
    let dir = tempdir().unwrap();
    let graph = grid_with_private_block();
    let private = [
        ((2, 2), (2, 3)),
        ((2, 3), (2, 2)),
        ((2, 3), (2, 4)),
        ((2, 4), (2, 3)),
    ]
    .map(|(from, to)| grid_edge(&graph, from, to));
    let reader = persist(&graph, &dir.path().join("grid.mrt"));
    let (oracle, compact) = engines(&reader);
    let (west, east) = (GeoPoint::new(0.0023, 0.0008), GeoPoint::new(0.0027, 0.0051));
    let crosses = |engine: &dyn Router, profile, sar: Option<&str>| {
        route(engine, west, east, profile, |request| {
            request.options.max_correlation_distance_m = 15;
            request.options.sar = sar.map(sar_option);
        })
        .unwrap()
        .edge_ids
        .iter()
        .any(|edge| private.contains(edge))
    };
    let lifted = r#"{"relaxations":{"legal_access":{}}}"#;
    let military = r#"{"relaxations":{"legal_access":{"values":["military"]}}}"#;
    for engine in [&oracle as &dyn Router, &compact] {
        assert!(!crosses(engine, MobilityProfile::Wheeled, None));
        assert!(!crosses(engine, MobilityProfile::Tracked, None));
        assert!(crosses(engine, MobilityProfile::Emergency, None));
        assert!(crosses(engine, MobilityProfile::Wheeled, Some(lifted)));
        assert!(!crosses(engine, MobilityProfile::Wheeled, Some(military)));
    }
}

/// A matrix cell crosses no destination-only pocket but its own two
/// points', as a route between them does: with a target inside the block,
/// the drive between its two sides still goes around it, and the target
/// inside is reached (matrices used to open every point's pocket for every
/// cell).
#[test]
fn matrix_cells_cross_no_pocket_of_another_point() {
    use crate::matrix::{MatrixMode, test_mode};
    let dir = tempdir().unwrap();
    let graph = grid_with_private_block();
    let private = [
        ((2, 2), (2, 3)),
        ((2, 3), (2, 2)),
        ((2, 3), (2, 4)),
        ((2, 4), (2, 3)),
    ]
    .map(|(from, to)| grid_edge(&graph, from, to));
    let reader = persist(&graph, &dir.path().join("grid.mrt"));
    let (oracle, compact) = engines(&reader);
    let (west, east) = (GeoPoint::new(0.0023, 0.0008), GeoPoint::new(0.0027, 0.0051));
    let inside = GeoPoint::new(0.0023, 0.00465);
    for (name, engine) in [("reference", &oracle as &dyn Router), ("compact", &compact)] {
        let around = route(engine, west, east, MobilityProfile::Wheeled, |request| {
            request.options.max_correlation_distance_m = 15;
        })
        .unwrap();
        assert!(!around.edge_ids.iter().any(|edge| private.contains(edge)));
        for mode in [MatrixMode::Pairwise, MatrixMode::Rows] {
            let context = format!("{name} {mode:?}");
            let mut request = crate::RouteMatrixRequest::new(
                vec![west],
                vec![east, inside],
                MobilityProfile::Wheeled,
            );
            request.options.max_correlation_distance_m = 15;
            test_mode::set(Some(mode));
            let result = engine.matrix(request);
            test_mode::set(None);
            let [across, into] = &result.cells[..] else {
                panic!("{context}: {result:?}");
            };
            assert_eq!(
                across.status,
                crate::RouteMatrixCellStatus::Reachable,
                "{context}"
            );
            assert!(
                !across.edge_ids.iter().any(|edge| private.contains(edge)),
                "{context}: {across:?}"
            );
            let cost = across.cost.unwrap();
            assert!(
                (cost - around.cost).abs() <= 1e-3 * around.cost,
                "{context}: matrix {cost} vs route {}",
                around.cost
            );
            assert_eq!(
                into.status,
                crate::RouteMatrixCellStatus::Reachable,
                "{context}"
            );
        }
    }
}

fn budget_failures(result: &crate::RouteMatrixResult) -> Vec<(usize, usize)> {
    result
        .cells
        .iter()
        .filter(|cell| cell.failure_reason.as_deref() == Some("search_budget_exceeded"))
        .map(|cell| (cell.source_index, cell.target_index))
        .collect()
}

/// `max_visited_edge_count` bounds each search: a pairwise cell's search,
/// and a source row's shared expansion in one-to-many mode. A search's count
/// is what the same search reports as a one-cell or one-row matrix.
#[test]
fn matrix_budget_bounds_each_search() {
    let dir = tempdir().unwrap();
    let reader = persist(&grid_graph(), &dir.path().join("grid.mrt"));
    let (_, compact) = engines(&reader);
    let profile = MobilityProfile::Wheeled;
    let points = grid_points();

    // Pairwise: one search per cell.
    let cell_counts = points
        .iter()
        .map(|source| {
            points
                .iter()
                .map(|target| {
                    budget_matrix(&compact, &[*source], &[*target], profile, None)
                        .visited_edge_count
                })
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let largest = cell_counts.iter().flatten().copied().max().unwrap();
    let unbounded = budget_matrix(&compact, &points, &points, profile, None);
    let at_largest = budget_matrix(&compact, &points, &points, profile, Some(largest));
    assert_eq!(unbounded.cells, at_largest.cells);
    // Far below what the whole matrix visits: the budget is not shared.
    assert!(unbounded.visited_edge_count > 2 * largest);
    let tight = budget_matrix(&compact, &points, &points, profile, Some(largest - 1));
    let over = (0..points.len())
        .flat_map(|source| (0..points.len()).map(move |target| (source, target)))
        .filter(|(source, target)| cell_counts[*source][*target] == largest)
        .collect::<Vec<_>>();
    // A cell over budget fails, or keeps the best path its search had found
    // (never cheaper than the exact cell); every other cell is exact.
    let failures = budget_failures(&tight);
    assert!(
        failures.iter().all(|cell| over.contains(cell)),
        "{failures:?}"
    );
    for (exact, bounded) in unbounded.cells.iter().zip(&tight.cells) {
        let cell = (bounded.source_index, bounded.target_index);
        if !over.contains(&cell) {
            assert_eq!(exact, bounded, "{cell:?}");
        } else if let (Some(exact), Some(bounded)) = (exact.cost, bounded.cost) {
            assert!(bounded >= exact - 1e-9, "{cell:?}: {bounded} < {exact}");
        }
    }
    // Each search stops at its first edge over budget.
    let capped = |counts: &mut dyn Iterator<Item = usize>, budget: usize| {
        counts.map(|count| count.min(budget + 1)).sum::<usize>()
    };
    assert_eq!(
        tight.visited_edge_count,
        capped(&mut cell_counts.iter().flatten().copied(), largest - 1)
    );

    // Rows: one shared expansion per source (forced: the pairwise cut-off
    // depends on whether CCH potentials bound the cells).
    crate::matrix::test_mode::set(Some(crate::matrix::MatrixMode::Rows));
    let sources = &points[..3];
    for (name, engine, targets) in [("compact", &compact, many_grid_targets())] {
        let row_counts = sources
            .iter()
            .map(|source| {
                budget_matrix(engine, &[*source], &targets, profile, None).visited_edge_count
            })
            .collect::<Vec<_>>();
        let largest = *row_counts.iter().max().unwrap();
        let unbounded = budget_matrix(engine, sources, &targets, profile, None);
        let at_largest = budget_matrix(engine, sources, &targets, profile, Some(largest));
        assert_eq!(unbounded.cells, at_largest.cells, "{name}");
        assert!(unbounded.visited_edge_count > largest, "{name}");

        // Rows under budget are untouched; the others stop at budget + 1.
        let budget = largest - 1;
        let tight = budget_matrix(engine, sources, &targets, profile, Some(budget));
        assert_eq!(
            tight.visited_edge_count,
            capped(&mut row_counts.iter().copied(), budget),
            "{name}"
        );
        for (unbounded, tight) in unbounded.cells.iter().zip(&tight.cells) {
            if row_counts[unbounded.source_index] <= budget {
                assert_eq!(unbounded, tight, "{name}");
            }
        }

        // A budget every row exceeds: each row still gets all of it.
        let budget = row_counts.iter().min().unwrap() / 2;
        let small = budget_matrix(engine, sources, &targets, profile, Some(budget));
        assert_eq!(
            small.visited_edge_count,
            sources.len() * (budget + 1),
            "{name}"
        );
        for source in 0..sources.len() {
            let row = &small.cells[source * targets.len()..(source + 1) * targets.len()];
            // A stopped row keeps what it had reached (a compact row may
            // already hold a path to every target, just not proven shortest).
            assert!(
                row.iter().any(|cell| cell.cost.is_some()),
                "{name}: row {source} found nothing"
            );
        }
    }
    crate::matrix::test_mode::set(None);
}

// ---------------------------------------------------------------------------
// Non-default costing options (derived columns)
// ---------------------------------------------------------------------------

/// The grid with a cost-relevant attribute mix: every road class, surfaces,
/// tracktypes, smoothness, posted speeds, tolls, ferries, gates, access
/// restrictions, destination-only streets, fords, tunnels, route refs and
/// conditional access and speed tags, so every costing option changes some
/// edge's cost.
fn costing_grid_graph() -> RoadGraph {
    let lat = [0.0, 0.0011, 0.0023, 0.0031, 0.0044];
    let lng = [0.0, 0.0017, 0.0029, 0.0042, 0.0051];
    let node = |row: usize, col: usize| 1_000 + (row * 10 + col) as i64;
    let mut builder = RoadGraphBuilder::new("compact-costing-grid");
    let mut segments = Vec::new();
    for row in 0..5 {
        for col in 0..5 {
            if col + 1 < 5 {
                segments.push(((row, col), (row, col + 1)));
            }
            if row + 1 < 5 {
                segments.push(((row, col), (row + 1, col)));
            }
        }
    }
    for (index, (a, b)) in segments.into_iter().enumerate() {
        let k = index + 1;
        let (road_type, highway) = [
            (RoadType::Residential, "residential"),
            (RoadType::Primary, "primary"),
            (RoadType::Track, "track"),
            (RoadType::Secondary, "secondary"),
            (RoadType::Residential, "residential"),
            (RoadType::Path, "path"),
            (RoadType::Tertiary, "tertiary"),
            (RoadType::Motorway, "motorway"),
            (RoadType::Track, "track"),
        ][k % 9];
        let mut way = way(
            k as i64,
            (lat[a.0], lng[a.1]),
            (lat[b.0], lng[b.1]),
            node(a.0, a.1),
            node(b.0, b.1),
            road_type,
            if k % 19 == 4 {
                DirectionalAccess::Forward
            } else {
                DirectionalAccess::Both
            },
        );
        way.highway = Some(highway.into());
        way.surface = [
            None,
            Some("asphalt"),
            Some("gravel"),
            Some("dirt"),
            Some("cobblestone"),
            Some("sand"),
            Some("mud"),
        ][k % 7]
            .map(Into::into);
        way.tracktype = match k % 5 {
            2 => Some("grade3".into()),
            4 => Some("grade1".into()),
            _ => None,
        };
        way.smoothness = match k % 6 {
            1 => Some("bad".into()),
            4 => Some("good".into()),
            5 => Some("horrible".into()),
            _ => None,
        };
        way.maxspeed = match k % 4 {
            0 => Some("50".into()),
            3 if k % 3 == 0 => Some("90".into()),
            _ => None,
        };
        way.is_toll = k % 13 == 5;
        way.has_gate = k % 10 == 7;
        way.is_ferry = k % 17 == 9;
        way.access_restricted = k % 9 == 8;
        way.is_access_restricted = k % 9 == 8;
        way.is_destination_only = k % 8 == 3;
        way.has_ford = k % 12 == 6;
        way.has_tunnel = k % 15 == 10;
        way.conditional_restrictions = match k % 14 {
            1 => vec!["access=no @ (Mo-Fr 07:00-09:00)".into()],
            8 => vec!["maxspeed=20 @ (22:00-06:00)".into()],
            _ => Vec::new(),
        };
        builder.push_way(way);
    }
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    for index in (0..graph.shared_edge_infos.len()).step_by(3) {
        graph
            .shared_edge_infos
            .update(index, |attrs| attrs.route_refs = vec!["E77".into()]);
    }
    graph
}

fn utc_ms(day: u32, hour: u32) -> i64 {
    chrono::NaiveDate::from_ymd_opt(2026, 10, day)
        .unwrap()
        .and_hms_opt(hour, 0, 0)
        .unwrap()
        .and_utc()
        .timestamp_millis()
}

/// One option set per costing option class.
fn costing_option_sets() -> Vec<(&'static str, crate::RouteCostingOptions)> {
    use crate::{RouteCostingOptions as O, RouteSpeedPreference as S};
    let base = O::default();
    vec![
        ("default", base),
        (
            "avoid road classes",
            O {
                avoid_tracks: 0.7,
                avoid_paths: 0.5,
                avoid_motorways: 1.0,
                ..base
            },
        ),
        (
            "avoid flagged edges",
            O {
                avoid_tolls: 1.0,
                avoid_ferries: 0.5,
                avoid_gates: 1.0,
                avoid_access_restricted: 0.6,
                ..base
            },
        ),
        (
            "avoid destination-only",
            O {
                avoid_destination_only: 0.5,
                ..base
            },
        ),
        (
            "strict destination-only",
            O {
                avoid_destination_only: 1.0,
                ..base
            },
        ),
        (
            "no fords",
            O {
                allow_ford: false,
                ..base
            },
        ),
        (
            "prefer paved and route refs",
            O {
                prefer_paved: 1.0,
                prefer_known_route_refs: 0.8,
                ..base
            },
        ),
        (
            "emergency override",
            O {
                emergency_override_strength: 0.7,
                ..base
            },
        ),
        (
            "free-flow speeds",
            O {
                speed_preference: S::FreeFlow,
                ..base
            },
        ),
        (
            "constrained speeds",
            O {
                speed_preference: S::Constrained,
                ..base
            },
        ),
        (
            "road-type speeds",
            O {
                speed_preference: S::RoadTypeDefault,
                ..base
            },
        ),
        (
            "posted speeds",
            O {
                speed_preference: S::Posted,
                ..base
            },
        ),
        (
            "fixed speed",
            O {
                fixed_speed_kph: Some(20.0),
                ..base
            },
        ),
        (
            "top speed",
            O {
                top_speed_kph: Some(40.0),
                ..base
            },
        ),
        (
            "conditional access",
            O {
                conditional_restriction_eval_at_ms: Some(utc_ms(5, 8)),
                ..base
            },
        ),
        (
            "conditional speed",
            O {
                conditional_restriction_eval_at_ms: Some(utc_ms(5, 23)),
                ..base
            },
        ),
    ]
}

const ALL_TEST_PROFILES: [MobilityProfile; 5] = [
    MobilityProfile::Wheeled,
    MobilityProfile::Tracked,
    MobilityProfile::Emergency,
    MobilityProfile::OnFoot,
    MobilityProfile::Legged,
];

fn isochrone_limits(profile: MobilityProfile) -> [f64; 3] {
    match profile {
        MobilityProfile::OnFoot | MobilityProfile::Legged => [60.0, 150.0, 300.0],
        _ => [20.0, 45.0, 90.0],
    }
}

#[test]
fn derived_default_columns_equal_stored_columns() {
    let dir = tempdir().unwrap();
    let reader = persist(&costing_grid_graph(), &dir.path().join("costing.mrt"));
    let graph = reader.package.clone();
    let view = graph.view();
    assert!(!view.conditional_edges.is_empty());
    for profile in [
        MobilityProfile::Default,
        MobilityProfile::Wheeled,
        MobilityProfile::OnFoot,
    ] {
        let stored = graph.request_column(profile, &Default::default(), true);
        assert!(matches!(stored, columns::ColumnHandle::Stored { .. }));
        let derived = columns::derive_column(
            &view,
            columns::ColumnKey {
                profile,
                costing: crate::RouteCostingOptions::default(),
                allow_destination_only: true,
                sar: None,
            },
            graph.file().column_for(profile),
        );
        let derived = columns::ColumnHandle::Derived(Arc::new(derived));
        let (stored, derived) = (stored.column(&view), derived.column(&view));
        let bits = |values: &[f32]| values.iter().map(|v| v.to_bits()).collect::<Vec<_>>();
        assert_eq!(bits(stored.cost), bits(derived.cost), "{profile:?}");
        assert_eq!(bits(stored.duration), bits(derived.duration), "{profile:?}");
        // Same costs, same factor.
        assert_eq!(stored.heuristic_cost_per_m, derived.heuristic_cost_per_m);
    }
}

#[test]
fn package_routes_match_reference_routes_for_every_costing_option_class() {
    let dir = tempdir().unwrap();
    let reader = persist(&costing_grid_graph(), &dir.path().join("costing.mrt"));
    let (oracle, compact) = engines(&reader);
    let points = grid_points();
    for (name, costing) in costing_option_sets() {
        let mut found = 0;
        let mut differs_from_default = false;
        for profile in ALL_TEST_PROFILES {
            for origin in &points {
                for destination in &points {
                    if origin == destination {
                        continue;
                    }
                    let context = format!("{name} {profile:?} {origin:?} -> {destination:?}");
                    let configure = |request: &mut LegRequest| request.options.costing = costing;
                    let expected = route(&oracle, *origin, *destination, profile, configure);
                    let actual = route(&compact, *origin, *destination, profile, configure);
                    assert_eq!(expected.is_some(), actual.is_some(), "{context}");
                    let (Some(expected), Some(actual)) = (expected, actual) else {
                        continue;
                    };
                    found += 1;
                    assert!(
                        (expected.cost - actual.cost).abs() <= 1e-4 * expected.cost.max(1.0),
                        "{context}: reference {} vs compact {}",
                        expected.cost,
                        actual.cost
                    );
                    assert!(
                        (expected.duration_s - actual.duration_s).abs()
                            <= 1e-4 * expected.duration_s.max(1.0),
                        "{context}: reference {}s vs compact {}s",
                        expected.duration_s,
                        actual.duration_s
                    );
                    if name != "default" && !differs_from_default {
                        let plain = route(&compact, *origin, *destination, profile, |_| {});
                        differs_from_default = plain.is_none_or(|plain| plain.cost != actual.cost);
                    }
                }
            }
        }
        assert!(found > 50, "{name}: only {found} routes");
        assert!(
            name == "default" || differs_from_default,
            "{name}: the options changed no route"
        );
    }
}

#[test]
fn package_matrices_match_reference_matrices_for_every_costing_option_class() {
    let dir = tempdir().unwrap();
    let reader = persist(&costing_grid_graph(), &dir.path().join("costing.mrt"));
    let (oracle, compact) = engines(&reader);
    for (name, costing) in costing_option_sets() {
        for profile in ALL_TEST_PROFILES {
            for (sources, targets) in [
                (grid_points(), grid_points()),
                (grid_points()[..2].to_vec(), many_grid_targets()),
            ] {
                let mut request = crate::RouteMatrixRequest::new(sources, targets, profile);
                request.options.max_correlation_distance_m = 60;
                request.options.costing = costing;
                let expected = oracle.matrix(request.clone());
                let actual = compact.matrix(request);
                assert_matrix_parity(&expected, &actual);
                assert!(
                    actual.cells.iter().any(|cell| cell.cost.is_some()),
                    "{name} {profile:?}"
                );
            }
        }
    }
}

#[test]
fn package_isochrones_match_reference_isochrones_for_every_costing_option_class() {
    let dir = tempdir().unwrap();
    let reader = persist(&costing_grid_graph(), &dir.path().join("costing.mrt"));
    let (oracle, compact) = engines(&reader);
    for (name, costing) in costing_option_sets() {
        let mut reached = 0;
        for profile in ALL_TEST_PROFILES {
            for origin in grid_points() {
                let mut request = grid_isochrone_request(profile, isochrone_limits(profile));
                request.origin = origin;
                request.options.costing = costing;
                let (Ok(expected), Ok(actual)) = (
                    oracle.isochrone(request.clone()),
                    compact.isochrone(request),
                ) else {
                    continue;
                };
                assert_isochrone_parity(
                    &expected,
                    &actual,
                    &format!("{name} {profile:?} {origin:?}"),
                );
                reached += actual
                    .buckets
                    .iter()
                    .map(|bucket| bucket.edge_ids.len())
                    .sum::<usize>();
            }
        }
        assert!(reached > 100, "{name}: {reached} reached edges");
    }
}

#[test]
fn derived_columns_are_cached_per_option_set() {
    let dir = tempdir().unwrap();
    let reader = persist(&costing_grid_graph(), &dir.path().join("costing.mrt"));
    let graph = reader.package.clone();
    let options = crate::RouteCostingOptions {
        avoid_motorways: 0.5,
        ..Default::default()
    };
    let column = |profile, options: &crate::RouteCostingOptions| match graph
        .request_column(profile, options, true)
    {
        columns::ColumnHandle::Derived(column) => column,
        _ => panic!("expected a derived column"),
    };
    let first = column(MobilityProfile::Wheeled, &options);
    // Same normalized options and an equivalent profile share the column.
    assert!(Arc::ptr_eq(
        &first,
        &column(MobilityProfile::Default, &options)
    ));
    let unnormalized = crate::RouteCostingOptions {
        avoid_motorways: 0.5,
        avoid_tolls: -3.0,
        ..Default::default()
    };
    assert!(Arc::ptr_eq(
        &first,
        &column(MobilityProfile::Wheeled, &unnormalized)
    ));
    // A different policy does not.
    assert!(!Arc::ptr_eq(
        &first,
        &column(MobilityProfile::OnFoot, &options)
    ));
    // Profiles without a stored column derive one even for default options.
    let tracked = column(MobilityProfile::Tracked, &Default::default());
    assert!(Arc::ptr_eq(
        &tracked,
        &column(MobilityProfile::Tracked, &Default::default())
    ));
    // The cache is bounded: old entries are evicted.
    for step in 1..=12 {
        let options = crate::RouteCostingOptions {
            avoid_tracks: 0.05 * f64::from(step),
            ..Default::default()
        };
        column(MobilityProfile::OnFoot, &options);
    }
    assert!(!Arc::ptr_eq(
        &first,
        &column(MobilityProfile::Wheeled, &options)
    ));
}

// ---------------------------------------------------------------------------
// Dynamic overlays (patch graph)
// ---------------------------------------------------------------------------

/// A connector between two points on edges.
fn edge_connector(
    id: u64,
    kind: crate::RouteDynamicVirtualEdgeKind,
    source: (u64, f64),
    target: (u64, f64),
    cost: f64,
) -> crate::RouteDynamicVirtualEdge {
    crate::RouteDynamicVirtualEdge {
        edge_id: (1 << 63) | id,
        connector_id: format!("connector-{id}"),
        connector_type: kind,
        source_endpoint: None,
        source_point: None,
        source_edge_id: Some(source.0),
        source_fraction: Some(source.1),
        target_endpoint: None,
        target_point: None,
        target_edge_id: Some(target.0),
        target_fraction: Some(target.1),
        source_chunk_ids: Vec::new(),
        geometry: Vec::new(),
        distance_m: 10.0,
        cost,
        provenance: "test".into(),
    }
}

/// Patch arcs that no shortest path should use (connectors far costlier
/// than any route) must leave every route exactly as the static search
/// finds it: same cost, same edges. This checks the patch search's handling
/// of real edges, turn costs and both restriction kinds against the static
/// bidirectional search as an exact oracle.
#[test]
fn irrelevant_overlay_patch_leaves_routes_unchanged() {
    let dir = tempdir().unwrap();
    // Junction graph with a simple and a complex restriction.
    let mut junction = junction_graph();
    let w_j = edge_between(&junction, 1, 10, 11);
    let j_n = edge_between(&junction, 3, 11, 13);
    let j_e = edge_between(&junction, 2, 11, 12);
    let e_ne = edge_between(&junction, 4, 12, 14);
    junction.set_resolved_restrictions(vec![
        ResolvedEdgeRestriction::Simple {
            relation_id: 77,
            restriction: "no_left_turn".into(),
            from_edge_id: w_j,
            to_edge_id: j_n,
            via_node_id: 11,
            osm_mode_mask: OsmMode::MOTOR_VEHICLE,
            mobility_mode_mask: MobilityMode::WHEELED,
        },
        ResolvedEdgeRestriction::Complex {
            relation_id: 78,
            restriction: "no_straight_on".into(),
            from_edge_id: w_j,
            to_edge_id: e_ne,
            via_edge_ids: vec![j_e],
            osm_mode_mask: OsmMode::MOTOR_VEHICLE,
            mobility_mode_mask: MobilityMode::WHEELED,
        },
    ]);
    let grid = grid_graph();
    let cases = [
        (
            "grid",
            grid.clone(),
            grid_points(),
            vec![
                (grid_edge(&grid, (1, 1), (1, 2)), 0.5),
                (grid_edge(&grid, (3, 3), (3, 4)), 0.3),
                (grid_edge(&grid, (2, 2), (2, 3)), 0.6),
            ],
        ),
        (
            "junction",
            junction.clone(),
            vec![
                GeoPoint::new(0.0, 0.0004),
                GeoPoint::new(0.0016, 0.002),
                GeoPoint::new(0.002, 0.0031),
                GeoPoint::new(0.0, 0.0037),
            ],
            vec![(w_j, 0.5), (e_ne, 0.4), (j_n, 0.7)],
        ),
    ];
    for (name, graph, points, attachments) in cases {
        let reader = persist(&graph, &dir.path().join(format!("{name}.mrt")));
        let (_, compact) = engines(&reader);
        let mut overlay = crate::RouteDynamicOverlay::default();
        for (index, pair) in attachments.windows(2).enumerate() {
            overlay.virtual_edges.push(edge_connector(
                index as u64,
                crate::RouteDynamicVirtualEdgeKind::RoadToRoadBypass,
                pair[0],
                pair[1],
                1e7,
            ));
        }
        overlay.refresh_diagnostics();
        let mut compared = 0;
        for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
            for origin in &points {
                for destination in &points {
                    if origin == destination {
                        continue;
                    }
                    let context = format!("{name} {profile:?} {origin:?} -> {destination:?}");
                    let plain = route(&compact, *origin, *destination, profile, |_| {});
                    let patched = route(&compact, *origin, *destination, profile, |request| {
                        request.options.dynamic_overlay = Some(overlay.clone());
                    });
                    assert_eq!(plain.is_some(), patched.is_some(), "{context}");
                    let (Some(plain), Some(patched)) = (plain, patched) else {
                        continue;
                    };
                    assert!(
                        (plain.cost - patched.cost).abs() <= 1e-6 * plain.cost.max(1.0),
                        "{context}: static {} vs patched {}",
                        plain.cost,
                        patched.cost
                    );
                    assert!(
                        (plain.duration_s - patched.duration_s).abs()
                            <= 1e-6 * plain.duration_s.max(1.0),
                        "{context}"
                    );
                    compared += 1;
                }
            }
        }
        assert!(compared > 10, "{name}: {compared}");
    }
}

/// A partly blocked origin edge: the route leaves on the usable fragment,
/// which pays the partial-block multiplier and still counts as traversing
/// the edge for a complex restriction (which the former tile-backed overlay
/// search ignored).
#[test]
fn overlay_fragments_keep_complex_restrictions() {
    let dir = tempdir().unwrap();
    let mut graph = junction_graph();
    let w_j = edge_between(&graph, 1, 10, 11);
    let j_e = edge_between(&graph, 2, 11, 12);
    let e_ne = edge_between(&graph, 4, 12, 14);
    graph.set_resolved_restrictions(vec![ResolvedEdgeRestriction::Complex {
        relation_id: 78,
        restriction: "no_straight_on".into(),
        from_edge_id: w_j,
        to_edge_id: e_ne,
        via_edge_ids: vec![j_e],
        osm_mode_mask: OsmMode::MOTOR_VEHICLE,
        mobility_mode_mask: MobilityMode::WHEELED,
    }]);
    let reader = persist(&graph, &dir.path().join("junction.mrt"));
    let (_, compact) = engines(&reader);
    let origin = GeoPoint::new(0.0, 0.001);
    let destination = GeoPoint::new(0.0008, 0.004);
    let overlay = crate::RouteDynamicOverlay::from_blocked_ranges(&[crate::RouteEdgeRange {
        edge_id: w_j,
        start_fraction: 0.1,
        end_fraction: 0.2,
    }]);
    let profile = MobilityProfile::Wheeled;
    // Unrestricted, the route runs straight through the restricted sequence.
    let free = persist(&junction_graph(), &dir.path().join("free.mrt"));
    let (_, free) = engines(&free);
    let unrestricted = route(&free, origin, destination, profile, |request| {
        request.options.dynamic_overlay = Some(overlay.clone());
    })
    .expect("unrestricted route");
    assert!(
        unrestricted
            .edge_ids
            .windows(3)
            .any(|window| window == [w_j, j_e, e_ne]),
        "{:?}",
        unrestricted.edge_ids
    );
    let plain = route(&compact, origin, destination, profile, |_| {}).expect("plain route");
    let patched = route(&compact, origin, destination, profile, |request| {
        request.options.dynamic_overlay = Some(overlay.clone());
    })
    .expect("patched route");
    assert_eq!(plain.edge_ids, patched.edge_ids);
    assert!(
        !patched
            .edge_ids
            .windows(3)
            .any(|window| window == [w_j, j_e, e_ne]),
        "{:?}",
        patched.edge_ids
    );
    // Same path; only the origin fragment's multiplier differs.
    let fragment = &patched.legs[0].edges[0];
    assert_eq!(fragment.edge_id, w_j);
    assert!(
        patched.legs[0].shape_segments[0]
            .quality_flags
            .contains(&crate::RouteShapeQualityFlag::SsvgPartialEdgeSplit)
    );
    let plain_first = plain.legs[0].edges[0].cost_breakdown.total_cost;
    assert!(
        (patched.cost - plain.cost - (fragment.cost_breakdown.total_cost - plain_first)).abs()
            <= 1e-4 * patched.cost,
        "{} vs {} ({} vs {})",
        patched.cost,
        plain.cost,
        fragment.cost_breakdown.total_cost,
        plain_first
    );
    assert!(patched.cost > plain.cost);
}

/// Fully blocked splits are infinite overrides: the same route as blocking
/// the edges by id.
#[test]
fn fully_blocked_overlay_splits_match_blocked_edges() {
    let dir = tempdir().unwrap();
    let graph = grid_graph();
    let reader = persist(&graph, &dir.path().join("grid.mrt"));
    let (_, compact) = engines(&reader);
    let blocked = [
        grid_edge(&graph, (2, 1), (2, 2)),
        grid_edge(&graph, (2, 2), (2, 1)),
        grid_edge(&graph, (1, 2), (2, 2)),
    ];
    let overlay = crate::RouteDynamicOverlay::from_blocked_ranges(
        &blocked
            .iter()
            .map(|edge_id| crate::RouteEdgeRange {
                edge_id: *edge_id,
                start_fraction: 0.0,
                end_fraction: 1.0,
            })
            .collect::<Vec<_>>(),
    );
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        for origin in grid_points() {
            for destination in grid_points() {
                if origin == destination {
                    continue;
                }
                let by_id = route(&compact, origin, destination, profile, |request| {
                    request.options.blocked_edge_ids.extend(blocked);
                });
                let by_overlay = route(&compact, origin, destination, profile, |request| {
                    request.options.dynamic_overlay = Some(overlay.clone());
                });
                assert_eq!(
                    by_id
                        .as_ref()
                        .map(|route| (route.cost, route.edge_ids.clone())),
                    by_overlay
                        .as_ref()
                        .map(|route| (route.cost, route.edge_ids.clone())),
                    "{profile:?} {origin:?} -> {destination:?}"
                );
            }
        }
    }
}

/// Partial blocks with bypass connectors around them and a long road-to-road
/// connector: every package overlay route is the reference overlay route
/// (same cost and duration; the same path except on exact ties), and its
/// steps reproduce its edge ids, connectors and flags.
#[test]
fn package_overlay_routes_match_reference_overlay_routes() {
    let dir = tempdir().unwrap();
    let graph = grid_graph();
    let reader = persist(&graph, &dir.path().join("grid.mrt"));
    let (oracle, compact) = engines(&reader);
    let splits = [
        (grid_edge(&graph, (1, 1), (1, 2)), 0.3, 0.6),
        (grid_edge(&graph, (1, 2), (1, 1)), 0.4, 0.7),
        (grid_edge(&graph, (2, 3), (3, 3)), 0.0, 0.5),
        (grid_edge(&graph, (3, 2), (3, 3)), 0.45, 1.0),
    ];
    let mut overlay = crate::RouteDynamicOverlay::from_blocked_ranges(
        &splits
            .iter()
            .map(|(edge_id, start, end)| crate::RouteEdgeRange {
                edge_id: *edge_id,
                start_fraction: *start,
                end_fraction: *end,
            })
            .collect::<Vec<_>>(),
    );
    for (index, (edge_id, start, end)) in splits.iter().enumerate() {
        if *start > 0.0 && *end < 1.0 {
            overlay.virtual_edges.push(edge_connector(
                index as u64,
                crate::RouteDynamicVirtualEdgeKind::RoadToRoadBypass,
                (*edge_id, *start),
                (*edge_id, *end),
                12.0,
            ));
        }
    }
    overlay.virtual_edges.push(edge_connector(
        10,
        crate::RouteDynamicVirtualEdgeKind::CrossCountry,
        (grid_edge(&graph, (0, 2), (0, 3)), 0.5),
        (grid_edge(&graph, (4, 1), (4, 2)), 0.5),
        25.0,
    ));
    overlay.refresh_diagnostics();
    let (mut compared, mut same_path, mut via_connector) = (0, 0, 0);
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        for origin in grid_points() {
            for destination in grid_points() {
                if origin == destination {
                    continue;
                }
                let context = format!("{profile:?} {origin:?} -> {destination:?}");
                let configure = |request: &mut LegRequest| {
                    request.options.dynamic_overlay = Some(overlay.clone());
                };
                let reference = route(&oracle, origin, destination, profile, configure);
                let patched = route(&compact, origin, destination, profile, configure);
                assert_eq!(reference.is_some(), patched.is_some(), "{context}");
                let (Some(reference), Some(patched)) = (reference, patched) else {
                    continue;
                };
                assert!(
                    (patched.cost - reference.cost).abs() <= 1e-4 * reference.cost.max(1.0),
                    "{context}: reference {} vs compact {}",
                    reference.cost,
                    patched.cost
                );
                assert!(
                    (patched.duration_s - reference.duration_s).abs()
                        <= 1e-4 * reference.duration_s.max(1.0),
                    "{context}: reference {}s vs compact {}s",
                    reference.duration_s,
                    patched.duration_s
                );
                compared += 1;
                same_path += usize::from(patched.edge_ids == reference.edge_ids);
                via_connector += usize::from(patched.edge_ids.iter().any(|id| id >> 63 == 1));
                // Steps reproduce the route: edge ids, connectors and flags.
                let segments = patched
                    .legs
                    .iter()
                    .flat_map(|leg| leg.shape_segments.iter())
                    .collect::<Vec<_>>();
                assert_eq!(segments.len(), patched.edge_ids.len(), "{context}");
                for segment in segments {
                    let connector = segment
                        .quality_flags
                        .contains(&crate::RouteShapeQualityFlag::SsvgConnector);
                    assert_eq!(connector, segment.edge_id >> 63 == 1, "{context}");
                }
            }
        }
    }
    assert!(compared > 40, "{compared}");
    assert!(via_connector > 0, "{via_connector}");
    eprintln!("overlay parity: {compared} compared, {same_path} on the same path");
}

/// Off-road endpoints joined to one road by endpoint connectors: the route
/// is the two connectors plus the road between their attachment points.
#[test]
fn package_overlay_connects_offroad_endpoints() {
    let dir = tempdir().unwrap();
    let graph = grid_graph();
    let reader = persist(&graph, &dir.path().join("grid.mrt"));
    let (_, compact) = engines(&reader);
    let edge = grid_edge(&graph, (0, 1), (0, 2));
    let along = |fraction: f64| GeoPoint::new(0.0, 0.0017 + (0.0029 - 0.0017) * fraction);
    let origin = GeoPoint::new(0.0002, along(0.2).lng);
    let destination = GeoPoint::new(0.0002, along(0.8).lng);
    let mut overlay = crate::RouteDynamicOverlay::default();
    let mut origin_connector = edge_connector(
        1,
        crate::RouteDynamicVirtualEdgeKind::Endpoint,
        (edge, 0.0),
        (edge, 0.2),
        20.0,
    );
    origin_connector.source_edge_id = None;
    origin_connector.source_fraction = None;
    origin_connector.source_endpoint = Some(crate::RouteDynamicEndpointKind::Origin);
    origin_connector.source_point = Some(origin);
    let mut destination_connector = edge_connector(
        2,
        crate::RouteDynamicVirtualEdgeKind::Endpoint,
        (edge, 0.8),
        (edge, 1.0),
        20.0,
    );
    destination_connector.target_edge_id = None;
    destination_connector.target_fraction = None;
    destination_connector.target_endpoint = Some(crate::RouteDynamicEndpointKind::Destination);
    destination_connector.target_point = Some(destination);
    overlay.virtual_edges = vec![origin_connector, destination_connector];
    overlay.refresh_diagnostics();
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        // The road stretch alone, as a same-edge route.
        let stretch = route(&compact, along(0.2), along(0.8), profile, |_| {}).unwrap();
        assert_eq!(stretch.edge_ids, vec![edge], "{profile:?}");
        let patched = route(&compact, origin, destination, profile, |request| {
            request.options.dynamic_overlay = Some(overlay.clone());
        })
        .unwrap();
        assert_eq!(
            patched.edge_ids,
            vec![
                overlay.virtual_edges[0].edge_id,
                edge,
                overlay.virtual_edges[1].edge_id
            ],
            "{profile:?}"
        );
        assert!(
            (patched.cost - (40.0 + stretch.cost)).abs() <= 1e-6 * patched.cost,
            "{profile:?}: {} vs 40 + {}",
            patched.cost,
            stretch.cost
        );
        assert_eq!(patched.origin.snapped_point, origin);
        assert_eq!(patched.destination.snapped_point, destination);
        assert_eq!(patched.diagnostics.ssvg_endpoint_connector_used_count, 2);
        assert!(
            patched.legs[0].shape_segments[1]
                .quality_flags
                .contains(&crate::RouteShapeQualityFlag::SsvgPartialEdgeSplit)
        );
    }
}

/// Random overlays (partial and full blocks, edge-to-edge connectors and
/// endpoint connectors with random costs): the goal-directed patch search
/// (straight-line and connector-aware potential bounds) finds exactly the
/// costs plain Dijkstra over the same patch finds.
#[test]
fn overlay_patch_bounds_are_admissible() {
    use crate::package::search::patch::dijkstra;
    let dir = tempdir().unwrap();
    let graph = costing_grid_graph();
    let reader = persist(&graph, &dir.path().join("costing.mrt"));
    let (_, compact) = engines(&reader);
    let edges = graph
        .graph
        .edge_references()
        .map(|edge| edge.weight().edge_id)
        .collect::<Vec<_>>();
    let mut seed = 0x2545_F491_4F6C_DD1Du64;
    let mut next = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    let points = grid_points();
    let (mut compared, mut used_connector) = (0, 0);
    for round in 0..12 {
        let pick = |r: f64| edges[(r * edges.len() as f64) as usize % edges.len()];
        let ranges = (0..1 + round % 4)
            .map(|_| {
                let start = next() * 0.8;
                crate::RouteEdgeRange {
                    edge_id: pick(next()),
                    start_fraction: if next() < 0.2 { 0.0 } else { start },
                    end_fraction: if next() < 0.2 {
                        1.0
                    } else {
                        start + 0.05 + next() * 0.15
                    },
                }
            })
            .collect::<Vec<_>>();
        let mut overlay = crate::RouteDynamicOverlay::from_blocked_ranges(&ranges);
        for index in 0..1 + round % 3 {
            let (source, target) = ((pick(next()), next()), (pick(next()), next()));
            overlay.virtual_edges.push(edge_connector(
                index as u64,
                crate::RouteDynamicVirtualEdgeKind::CrossCountry,
                source,
                target,
                next() * 120.0,
            ));
        }
        if round % 4 == 3 {
            let mut endpoint = edge_connector(
                9,
                crate::RouteDynamicVirtualEdgeKind::Endpoint,
                (pick(next()), next()),
                (pick(next()), next()),
                next() * 60.0,
            );
            endpoint.source_edge_id = None;
            endpoint.source_fraction = None;
            endpoint.source_endpoint = Some(crate::RouteDynamicEndpointKind::Origin);
            endpoint.source_point = Some(points[0]);
            overlay.virtual_edges.push(endpoint);
        }
        overlay.refresh_diagnostics();
        for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
            for (index, origin) in points.iter().enumerate() {
                for destination in &points {
                    if origin == destination {
                        continue;
                    }
                    // The origin endpoint connector starts at points[0].
                    if round % 4 == 3 && index != 0 {
                        continue;
                    }
                    let run = |plain: bool| {
                        dijkstra::force(plain);
                        let result = route(&compact, *origin, *destination, profile, |request| {
                            request.options.dynamic_overlay = Some(overlay.clone());
                        });
                        dijkstra::force(false);
                        result
                    };
                    let (bounded, plain) = (run(false), run(true));
                    let context =
                        format!("round {round} {profile:?} {origin:?} -> {destination:?}");
                    assert_eq!(bounded.is_some(), plain.is_some(), "{context}");
                    let (Some(bounded), Some(plain)) = (bounded, plain) else {
                        continue;
                    };
                    assert!(
                        (bounded.cost - plain.cost).abs() <= 1e-9 * plain.cost.max(1.0),
                        "{context}: bounded {} vs Dijkstra {}",
                        bounded.cost,
                        plain.cost
                    );
                    compared += 1;
                    used_connector += usize::from(bounded.edge_ids.iter().any(|id| id >> 63 == 1));
                }
            }
        }
    }
    assert!(compared > 200, "{compared}");
    assert!(used_connector > 20, "{used_connector}");
}

/// Partial blocks with bypass connectors, a cross-country connector and a
/// full block on the grid (the overlay of the route parity test above).
fn grid_overlay(graph: &RoadGraph) -> crate::RouteDynamicOverlay {
    let splits = [
        (grid_edge(graph, (1, 1), (1, 2)), 0.3, 0.6),
        (grid_edge(graph, (1, 2), (1, 1)), 0.4, 0.7),
        (grid_edge(graph, (2, 3), (3, 3)), 0.0, 0.5),
        (grid_edge(graph, (3, 2), (3, 3)), 0.45, 1.0),
        (grid_edge(graph, (2, 1), (2, 2)), 0.0, 1.0),
    ];
    let mut overlay = crate::RouteDynamicOverlay::from_blocked_ranges(
        &splits
            .iter()
            .map(|(edge_id, start, end)| crate::RouteEdgeRange {
                edge_id: *edge_id,
                start_fraction: *start,
                end_fraction: *end,
            })
            .collect::<Vec<_>>(),
    );
    for (index, (edge_id, start, end)) in splits.iter().enumerate() {
        if *start > 0.0 && *end < 1.0 {
            overlay.virtual_edges.push(edge_connector(
                index as u64,
                crate::RouteDynamicVirtualEdgeKind::RoadToRoadBypass,
                (*edge_id, *start),
                (*edge_id, *end),
                12.0,
            ));
        }
    }
    overlay.virtual_edges.push(edge_connector(
        10,
        crate::RouteDynamicVirtualEdgeKind::CrossCountry,
        (grid_edge(graph, (0, 2), (0, 3)), 0.5),
        (grid_edge(graph, (4, 1), (4, 2)), 0.5),
        25.0,
    ));
    overlay.refresh_diagnostics();
    overlay
}

/// Overlay matrices run on the patch: every cell, pairwise or in a shared
/// row, is exactly the package overlay route between its two points and the
/// reference overlay matrix cell.
#[test]
fn package_overlay_matrix_cells_equal_overlay_routes() {
    let dir = tempdir().unwrap();
    let graph = grid_graph();
    let reader = persist(&graph, &dir.path().join("grid.mrt"));
    let (oracle, compact) = engines(&reader);
    let overlay = grid_overlay(&graph);
    let (mut compared, mut via_connector) = (0, 0);
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        // Pairwise (few targets) and one-to-many rows (more than the cut-off).
        for (sources, targets) in [
            (grid_points(), grid_points()),
            (grid_points()[..3].to_vec(), many_grid_targets()),
        ] {
            let mut request =
                crate::RouteMatrixRequest::new(sources.clone(), targets.clone(), profile);
            request.options.max_correlation_distance_m = 60;
            request.options.dynamic_overlay = Some(overlay.clone());
            let actual = compact.matrix(request.clone());
            assert_matrix_parity(&oracle.matrix(request.clone()), &actual);
            for cell in &actual.cells {
                if cell.source_index == cell.target_index && targets.len() == sources.len() {
                    continue;
                }
                let (origin, destination) =
                    (sources[cell.source_index], targets[cell.target_index]);
                let context = format!("{profile:?} {origin:?} -> {destination:?}");
                let configure = |request: &mut LegRequest| {
                    request.options.dynamic_overlay = Some(overlay.clone());
                };
                let patched = route(&compact, origin, destination, profile, configure);
                assert_eq!(cell.cost.is_some(), patched.is_some(), "{context}");
                let (Some(cost), Some(patched)) = (cell.cost, patched) else {
                    continue;
                };
                assert!(
                    (patched.cost - cost).abs() <= 1e-6 * cost.max(1.0),
                    "{context}: matrix {cost} vs overlay route {}",
                    patched.cost
                );
                assert_eq!(cell.edge_ids, patched.edge_ids, "{context}");
                compared += 1;
                via_connector += usize::from(cell.edge_ids.iter().any(|id| id >> 63 == 1));
            }
        }
    }
    assert!(compared > 300, "{compared}");
    assert!(via_connector > 10, "{via_connector}");
}

/// An overlay of full blocks and connectors no shortest path uses changes
/// nothing beyond the blocks: overlay matrices equal the reference overlay
/// matrix cell for cell, and the package matrix with the same edges blocked
/// by id.
#[test]
fn package_overlay_matrix_with_irrelevant_patch_matches_reference() {
    let dir = tempdir().unwrap();
    let graph = grid_graph();
    let reader = persist(&graph, &dir.path().join("grid.mrt"));
    let (oracle, compact) = engines(&reader);
    let blocked = [
        grid_edge(&graph, (2, 1), (2, 2)),
        grid_edge(&graph, (1, 3), (2, 3)),
    ];
    let mut overlay = crate::RouteDynamicOverlay::from_blocked_ranges(
        &blocked
            .iter()
            .map(|edge_id| crate::RouteEdgeRange {
                edge_id: *edge_id,
                start_fraction: 0.0,
                end_fraction: 1.0,
            })
            .collect::<Vec<_>>(),
    );
    overlay.virtual_edges.push(edge_connector(
        1,
        crate::RouteDynamicVirtualEdgeKind::CrossCountry,
        (grid_edge(&graph, (0, 2), (0, 3)), 0.5),
        (grid_edge(&graph, (3, 1), (3, 2)), 0.4),
        1e7,
    ));
    overlay.refresh_diagnostics();
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        for (sources, targets) in [
            (grid_points(), grid_points()),
            (grid_points()[..3].to_vec(), many_grid_targets()),
        ] {
            let mut request =
                crate::RouteMatrixRequest::new(sources.clone(), targets.clone(), profile);
            request.options.max_correlation_distance_m = 60;
            let mut by_id = request.clone();
            by_id.options.blocked_edge_ids.extend(blocked);
            request.options.dynamic_overlay = Some(overlay.clone());
            let expected = oracle.matrix(request.clone());
            let actual = compact.matrix(request);
            assert_matrix_parity(&expected, &actual);
            let by_id = compact.matrix(by_id);
            for (left, right) in by_id.cells.iter().zip(&actual.cells) {
                let cell = (left.source_index, left.target_index);
                assert_eq!(left.status, right.status, "{profile:?} {cell:?}");
                let (Some(left), Some(right)) = (left.cost, right.cost) else {
                    continue;
                };
                assert!(
                    (left - right).abs() <= 1e-6 * left.max(1.0),
                    "{profile:?} {cell:?}: blocked by id {left} vs overlay {right}"
                );
            }
        }
    }
}

/// An isochrone with full blocks and connectors no reachable path uses,
/// attached inside unsplit edges (so those edges are also reached in
/// fragments): exactly the reference isochrone with the same edges blocked.
#[test]
fn package_overlay_isochrone_with_irrelevant_patch_matches_reference() {
    let dir = tempdir().unwrap();
    let graph = grid_graph();
    let reader = persist(&graph, &dir.path().join("grid.mrt"));
    let (oracle, compact) = engines(&reader);
    let blocked = [
        grid_edge(&graph, (2, 2), (2, 3)),
        grid_edge(&graph, (1, 3), (2, 3)),
    ];
    let mut overlay = crate::RouteDynamicOverlay::from_blocked_ranges(
        &blocked
            .iter()
            .map(|edge_id| crate::RouteEdgeRange {
                edge_id: *edge_id,
                start_fraction: 0.0,
                end_fraction: 1.0,
            })
            .collect::<Vec<_>>(),
    );
    for (index, (from, to)) in [
        (((2, 3), (2, 4)), ((3, 2), (3, 3))),
        (((1, 2), (1, 3)), ((2, 4), (3, 4))),
        (((3, 3), (2, 3)), ((2, 1), (1, 1))),
    ]
    .into_iter()
    .enumerate()
    {
        overlay.virtual_edges.push(edge_connector(
            index as u64,
            crate::RouteDynamicVirtualEdgeKind::CrossCountry,
            (grid_edge(&graph, from.0, from.1), 0.35),
            (grid_edge(&graph, to.0, to.1), 0.6),
            1e7,
        ));
    }
    overlay.refresh_diagnostics();
    for (profile, limits) in [
        (MobilityProfile::Wheeled, [20.0, 45.0, 90.0]),
        (MobilityProfile::OnFoot, [60.0, 150.0, 300.0]),
    ] {
        let mut blocked_by_id = grid_isochrone_request(profile, limits);
        blocked_by_id.options.blocked_edge_ids.extend(blocked);
        let expected = oracle.isochrone(blocked_by_id).unwrap();
        let mut request = grid_isochrone_request(profile, limits);
        request.options.dynamic_overlay = Some(overlay.clone());
        let actual = compact.isochrone(request).unwrap();
        assert_isochrone_parity(&expected, &actual, &format!("{profile:?}"));
        let plain = compact
            .isochrone(grid_isochrone_request(profile, limits))
            .unwrap();
        assert_ne!(
            plain.buckets, actual.buckets,
            "{profile:?}: nothing blocked"
        );
    }
}

/// Isochrones on the patch: partial blocks only remove reach (every unsplit
/// edge reached is reached without the overlay too, and no crossing lies in
/// a blocked range), and a cheap connector out of the reached area extends
/// it to edges the plain isochrone never reaches.
#[test]
fn package_overlay_isochrones_follow_splits_and_connectors() {
    let dir = tempdir().unwrap();
    let graph = grid_graph();
    let reader = persist(&graph, &dir.path().join("grid.mrt"));
    let (_, compact) = engines(&reader);
    let splits = [
        (grid_edge(&graph, (2, 2), (2, 3)), 0.3, 0.6),
        (grid_edge(&graph, (2, 3), (2, 2)), 0.2, 0.5),
        (grid_edge(&graph, (1, 3), (2, 3)), 0.5, 1.0),
        (grid_edge(&graph, (2, 3), (3, 3)), 0.0, 0.4),
    ];
    let overlay = crate::RouteDynamicOverlay::from_blocked_ranges(
        &splits
            .iter()
            .map(|(edge_id, start, end)| crate::RouteEdgeRange {
                edge_id: *edge_id,
                start_fraction: *start,
                end_fraction: *end,
            })
            .collect::<Vec<_>>(),
    );
    let far = grid_edge(&graph, (4, 0), (4, 1));
    // From just ahead of the origin (on the origin's edge) to a far corner.
    let mut connected = crate::RouteDynamicOverlay::default();
    for (index, edge) in [
        grid_edge(&graph, (2, 2), (2, 3)),
        grid_edge(&graph, (2, 3), (2, 2)),
    ]
    .into_iter()
    .enumerate()
    {
        connected.virtual_edges.push(edge_connector(
            index as u64,
            crate::RouteDynamicVirtualEdgeKind::CrossCountry,
            (edge, 0.8),
            (far, 0.1),
            1.0,
        ));
    }
    connected.refresh_diagnostics();
    let split_ids = splits.iter().map(|split| split.0).collect::<Vec<_>>();
    for (profile, limits) in [
        (MobilityProfile::Wheeled, [20.0, 45.0, 90.0]),
        (MobilityProfile::OnFoot, [60.0, 150.0, 300.0]),
    ] {
        let plain = compact
            .isochrone(grid_isochrone_request(profile, limits))
            .unwrap();
        let isochrone = |overlay: &crate::RouteDynamicOverlay| {
            let mut request = grid_isochrone_request(profile, limits);
            request.options.dynamic_overlay = Some(overlay.clone());
            compact.isochrone(request).unwrap()
        };
        let split = isochrone(&overlay);
        assert_ne!(
            plain.buckets, split.buckets,
            "{profile:?}: splits changed nothing"
        );
        for (plain, split) in plain.buckets.iter().zip(&split.buckets) {
            let bucket = format!("{profile:?} {:?} {}", plain.metric, plain.contour);
            for edge in &split.edge_ids {
                assert!(
                    split_ids.contains(edge) || plain.edge_ids.contains(edge),
                    "{bucket}: edge {edge} reached only with blocks"
                );
            }
            for fragment in &split.boundary_fragments {
                let Some((_, start, end)) = splits.iter().find(|s| s.0 == fragment.edge_id) else {
                    continue;
                };
                let (low, high) = (
                    fragment.start_fraction.min(fragment.end_fraction),
                    fragment.start_fraction.max(fragment.end_fraction),
                );
                assert!(
                    high <= start + 1e-9 || low >= end - 1e-9,
                    "{bucket}: crossing {low}..{high} inside blocked {start}..{end}"
                );
            }
        }
        let extended = isochrone(&connected);
        assert!(
            extended
                .buckets
                .iter()
                .zip(&plain.buckets)
                .any(|(extended, plain)| extended.edge_ids.contains(&far)
                    && !plain.edge_ids.contains(&far)),
            "{profile:?}: the connector did not extend the isochrone"
        );
        for (plain, extended) in plain.buckets.iter().zip(&extended.buckets) {
            for edge in &plain.edge_ids {
                assert!(extended.edge_ids.contains(edge), "{profile:?}: lost {edge}");
            }
        }
    }
}

/// Streets around the corner of four detail tiles (lat 0, lng 0): curved
/// two-way and one-way ways, a closed loop, a long diagonal crossing tile
/// boundaries, a footway, a named primary with a ref, and a bridge on layer 1.
fn correlation_graph() -> RoadGraph {
    let mut builder = RoadGraphBuilder::new("compact-correlation");
    type Way = (Vec<(f64, f64)>, Vec<i64>, RoadType, DirectionalAccess);
    let ways: Vec<Way> = vec![
        (
            vec![
                (-0.0010, -0.0012),
                (-0.0004, -0.0003),
                (0.0003, 0.0002),
                (0.0009, 0.0011),
            ],
            vec![1, 0, 0, 2],
            RoadType::Primary,
            DirectionalAccess::Both,
        ),
        (
            vec![(0.0009, 0.0011), (0.0012, 0.0004), (0.0011, -0.0006)],
            vec![2, 0, 3],
            RoadType::Residential,
            DirectionalAccess::Forward,
        ),
        (
            vec![(0.0011, -0.0006), (-0.0010, -0.0012)],
            vec![3, 1],
            RoadType::Residential,
            DirectionalAccess::Both,
        ),
        // Closed loop at node 2.
        (
            vec![
                (0.0009, 0.0011),
                (0.0014, 0.0016),
                (0.0017, 0.0010),
                (0.0009, 0.0011),
            ],
            vec![2, 0, 0, 2],
            RoadType::Residential,
            DirectionalAccess::Both,
        ),
        // Long diagonal across the tile corner.
        (
            vec![(-0.0030, 0.0025), (0.0028, -0.0031)],
            vec![4, 5],
            RoadType::Secondary,
            DirectionalAccess::Both,
        ),
        (
            vec![(0.0028, -0.0031), (0.0011, -0.0006)],
            vec![5, 3],
            RoadType::Path,
            DirectionalAccess::Both,
        ),
        (
            vec![(-0.0030, 0.0025), (-0.0004, -0.0003)],
            vec![4, 6],
            RoadType::Track,
            DirectionalAccess::Both,
        ),
    ];
    for (index, (points, nodes, road_type, directional_access)) in ways.into_iter().enumerate() {
        let highway = match road_type {
            RoadType::Primary => "primary",
            RoadType::Secondary => "secondary",
            RoadType::Path => "footway",
            RoadType::Track => "track",
            _ => "residential",
        };
        builder.push_way(RoutableWay {
            osm_way_id: 100 + index as i64,
            source_node_ids: nodes
                .iter()
                .map(|node| (*node != 0).then_some(*node))
                .collect(),
            points,
            road_type,
            directional_access,
            highway: Some(highway.into()),
            name: (index % 3 == 0).then(|| format!("Street  {index}").into()),
            ref_name: (index == 0).then(|| "79".into()),
            layer: (index == 4).then(|| "1".into()),
            level: (index == 5).then(|| "0;1".into()),
            has_bridge: index == 4,
            service: (index == 2).then(|| "alley".into()),
            is_roundabout: index == 3,
            surface: (index == 1).then(|| "gravel".into()),
            maxspeed: (index == 0).then(|| "50".into()),
            ..Default::default()
        });
    }
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    graph
}

/// Correlation runs only on the package, so its inputs are checked against
/// the road graph it was built from: the edges the correlation sections select
/// around a point, each candidate's cheap-check fields and geometry, and the
/// reach counts and opposing edges read for accepted candidates.
#[test]
fn package_correlation_inputs_match_road_graph() {
    let dir = tempdir().unwrap();
    for (name, graph) in [("grid", grid_graph()), ("correlation", correlation_graph())] {
        let archive = dir.path().join(format!("{name}.mrt"));
        let reader = persist(&graph, &archive);
        let access = reader.access();
        let package = reader.package.clone();
        let view = PackageGraphView::new(package.clone());
        let request = view.request();
        let mut found = 0;
        // Includes points exactly on the tile corner (lat 0, lng 0).
        for row in 0..18 {
            for col in 0..18 {
                let point = GeoPoint::new(
                    -0.0035 + f64::from(row) * 0.0005,
                    -0.0035 + f64::from(col) * 0.0005,
                );
                for max_distance_m in [20, 250] {
                    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
                        let context = format!("{name}: {point:?} {max_distance_m} m {profile:?}");
                        let selected = request.correlation_edges(point, max_distance_m, profile);
                        let offered = access.candidate_edges_near(point, max_distance_m);
                        for edge in &selected {
                            let record = offered
                                .iter()
                                .find(|record| record.edge.edge_id == edge.edge_id)
                                .unwrap_or_else(|| {
                                    panic!("{context}: edge {} not offered", edge.edge_id)
                                });
                            assert_eq!(
                                edge.accessible,
                                record.edge.mobility_mode_mask & profile.mobility_mode_mask() != 0,
                                "{context}: edge {}",
                                edge.edge_id
                            );
                            assert_eq!(
                                edge.component_size,
                                access.component_size_for_node(record.source.id),
                                "{context}: edge {}",
                                edge.edge_id
                            );
                            assert_eq!(edge.layer, record.shared_info.layer, "{context}");
                            assert_eq!(edge.level, record.shared_info.level, "{context}");
                            assert_eq!(
                                edge.geometry,
                                crate::spatial::directed_geometry(
                                    record.shared_info,
                                    record.edge.traversal_direction
                                ),
                                "{context}: edge {}",
                                edge.edge_id
                            );
                        }
                        // The road graph's segment index has no detail-cell
                        // filter and may offer more edges, but every edge
                        // within reach must be selected.
                        for record in &offered {
                            let geometry = crate::spatial::directed_geometry(
                                record.shared_info,
                                record.edge.traversal_direction,
                            );
                            let within = crate::correlator::snap_distance_m(point, &geometry)
                                .is_some_and(|distance| distance <= f64::from(max_distance_m));
                            if within {
                                found += 1;
                                assert!(
                                    selected
                                        .iter()
                                        .any(|edge| edge.edge_id == record.edge.edge_id),
                                    "{context}: edge {} within reach but not selected",
                                    record.edge.edge_id
                                );
                            }
                        }
                    }
                }
            }
        }
        assert!(found > 0, "{name}: no point was within reach of a road");
        for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
            let mut reach = request::ReachCounter::new(package.file(), profile, 64);
            for edge in graph.graph.edge_weights() {
                let raw = view.raw_edge_index_for_edge_id(edge.edge_id).unwrap();
                let record = access.edge_by_id(edge.edge_id).unwrap();
                let context = format!("{name}: edge {} {profile:?}", edge.edge_id);
                assert_eq!(
                    reach.count(raw, true),
                    access.outbound_reach_edge_count(record.target.id, profile, 64),
                    "{context}"
                );
                assert_eq!(
                    reach.count(raw, false),
                    access.inbound_reach_edge_count(record.source.id, profile, 64),
                    "{context}"
                );
                assert_eq!(
                    view.opposing_edge_id(edge.edge_id),
                    access.opposing_edge_id(edge.edge_id),
                    "{context}"
                );
            }
        }
    }
}

#[test]
fn reach_counter_matches_reach_edge_count() {
    let dir = tempdir().unwrap();
    let archive = dir.path().join("grid.mrt");
    let reader = persist(&grid_graph(), &archive);
    let graph = reader.package.clone();
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        for cap in [0, 1, 5, 64] {
            let mut counter = request::ReachCounter::new(graph.file(), profile, cap);
            for raw in 0..graph.counts().edge_count {
                for outgoing in [true, false] {
                    assert_eq!(
                        counter.count(raw, outgoing),
                        graph.reach_edge_count(raw as u32, profile, cap, outgoing),
                        "{profile:?} cap {cap} raw {raw} outgoing {outgoing}"
                    );
                }
            }
        }
    }
}

/// The road graph's junction count at `node_id`: edges leaving (or entering)
/// it over a shared info not in `excluded`.
fn junction_count(
    access: &GraphAccess<'_>,
    node_id: i64,
    excluded: &[u64],
    outgoing: bool,
) -> Option<usize> {
    let edges = if outgoing {
        access.outgoing_edges_from_node_id(node_id)
    } else {
        access.incoming_edges_to_node_id(node_id)
    }?;
    Some(
        edges
            .into_iter()
            .filter(|record| !excluded.contains(&record.edge.shared_edge_info_id))
            .count(),
    )
}

#[test]
fn leg_records_match_road_graph_records() {
    let dir = tempdir().unwrap();
    for (name, graph) in [("grid", grid_graph()), ("correlation", correlation_graph())] {
        let archive = dir.path().join(format!("{name}.mrt"));
        let reader = persist(&graph, &archive);
        let access = reader.access();
        let graph = reader.package.clone();
        let view = graph.file().request_view().unwrap();
        let g = graph.view();
        for edge in 0..graph.counts().edge_count as u32 {
            let mut expected = access
                .edge_by_id(g.edge_id(edge as usize))
                .unwrap()
                .to_view_edge();
            // Packages store no road hierarchy tier (nothing routes on it).
            expected.edge.hierarchy_tier = None;
            expected.shared_info.hierarchy_tier = None;
            let actual = view.leg_record(edge).unwrap();
            let context = format!("{name} edge {}", expected.edge.edge_id);
            // Every field legs, costing and guidance read.
            assert_eq!(actual.shared_info, expected.shared_info, "{context}");
            assert_eq!(actual.raw_edge_index, edge as usize, "{context}");
            let (a, e) = (&actual.edge, &expected.edge);
            assert_eq!(
                (
                    a.edge_id,
                    a.shared_edge_info_id,
                    a.distance_m,
                    a.traversal_direction,
                    a.start_heading_deg,
                    a.end_heading_deg,
                    a.inferred_speed_kph,
                    a.opposing_edge_id,
                    a.hierarchy_tier,
                ),
                (
                    e.edge_id,
                    e.shared_edge_info_id,
                    e.distance_m,
                    e.traversal_direction,
                    e.start_heading_deg,
                    e.end_heading_deg,
                    e.inferred_speed_kph,
                    e.opposing_edge_id,
                    e.hierarchy_tier,
                ),
                "{context}"
            );
            for profile in [
                MobilityProfile::Wheeled,
                MobilityProfile::Tracked,
                MobilityProfile::Emergency,
                MobilityProfile::OnFoot,
                MobilityProfile::Legged,
            ] {
                assert_eq!(
                    a.mobility_mode_mask & profile.mobility_mode_mask() != 0,
                    e.mobility_mode_mask & profile.mobility_mode_mask() != 0,
                    "{context} {profile:?}"
                );
            }
            for (actual, expected) in [
                (&actual.source, &expected.source),
                (&actual.target, &expected.target),
            ] {
                assert_eq!(
                    (
                        actual.id,
                        actual.lat,
                        actual.lng,
                        actual.roundabout_exit_count
                    ),
                    (
                        expected.id,
                        expected.lat,
                        expected.lng,
                        expected.roundabout_exit_count
                    ),
                    "{context}"
                );
            }
            // Junction counts with the exclusions legs pass.
            for excluded in [
                vec![],
                vec![e.shared_edge_info_id],
                vec![e.shared_edge_info_id, e.shared_edge_info_id + 1],
            ] {
                assert_eq!(
                    Some(view.junction_edge_count(edge, &excluded, true)),
                    junction_count(&access, expected.source.id, &excluded, true),
                    "{context} outgoing {excluded:?}"
                );
                assert_eq!(
                    Some(view.junction_edge_count(edge, &excluded, false)),
                    junction_count(&access, expected.source.id, &excluded, false),
                    "{context} incoming {excluded:?}"
                );
            }
            assert_eq!(view.edge_for_id(e.edge_id), Some(edge), "{context}");
            for node in [view.edge_source(edge), view.edge_target(edge)] {
                let id = if node == view.edge_source(edge) {
                    expected.source.id
                } else {
                    expected.target.id
                };
                assert_eq!(
                    view.component_id(node),
                    access.component_id_for_node(id),
                    "{context}"
                );
            }
        }
    }
}

/// O-S-A-B-C-T with a slightly longer second way onto A (S-Y-A) and a long
/// detour B-D-C, and a wheeled complex restriction S-A, A-B -> B-C.
fn one_label_trap_graph() -> RoadGraph {
    let mut builder = RoadGraphBuilder::new("one-label-trap");
    let residential = RoadType::Residential;
    let both = DirectionalAccess::Both;
    let (o, s, a, b, c, t) = (
        (0.0, 0.0),
        (0.0, 0.001),
        (0.0, 0.002),
        (0.0, 0.003),
        (0.0, 0.004),
        (0.0, 0.005),
    );
    let (y, d) = ((0.0005, 0.0015), (-0.003, 0.0035));
    for (id, from, to, from_node, to_node) in [
        (1, o, s, 1, 2),
        (2, s, a, 2, 3),
        (3, a, b, 3, 4),
        (4, b, c, 4, 5),
        (5, c, t, 5, 6),
        (6, s, y, 2, 7),
        (7, y, a, 7, 3),
        (8, b, d, 4, 8),
        (9, d, c, 8, 5),
    ] {
        builder.push_way(way(id, from, to, from_node, to_node, residential, both));
    }
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    let s_a = edge_between(&graph, 2, 2, 3);
    let a_b = edge_between(&graph, 3, 3, 4);
    let b_c = edge_between(&graph, 4, 4, 5);
    graph.set_resolved_restrictions(vec![ResolvedEdgeRestriction::Complex {
        relation_id: 90,
        restriction: "no_straight_on".into(),
        from_edge_id: s_a,
        to_edge_id: b_c,
        via_edge_ids: vec![a_b],
        osm_mode_mask: OsmMode::MOTOR_VEHICLE,
        mobility_mode_mask: MobilityMode::WHEELED,
    }]);
    graph
}

/// The cheapest way onto A-B comes from S-A, the start of the forbidden
/// sequence, so B-C is closed to it, and the dearer way from Y-A that may
/// continue must be kept: one label per edge dropped it and detoured via D
/// (the reference without a path context still does). Labels keyed by
/// restriction state keep both, so every search finds the legal, cheaper
/// route through Y that the exact reference finds: routes (bidirectional and
/// forward only), matrices in every mode, isochrones and overlays.
#[test]
fn restriction_states_keep_a_dearer_legal_continuation() {
    use super::search::patch::dijkstra;
    use crate::matrix::{MatrixMode, test_mode};
    let dir = tempdir().unwrap();
    let graph = one_label_trap_graph();
    let reader = persist(&graph, &dir.path().join("trap.mrt"));
    let (one_label, compact) = engines(&reader);
    let exact = reader.reference().with_path_context(2);
    let (origin, destination) = (GeoPoint::new(0.0, 0.0002), GeoPoint::new(0.0, 0.0045));
    let y_a = edge_between(&graph, 7, 7, 3);
    let b_d = edge_between(&graph, 8, 4, 8);
    let profile = MobilityProfile::Wheeled;

    let expected = route(&exact, origin, destination, profile, |_| {}).unwrap();
    assert!(expected.edge_ids.contains(&y_a), "{:?}", expected.edge_ids);
    assert!(!expected.edge_ids.contains(&b_d), "{:?}", expected.edge_ids);
    let detour = route(&one_label, origin, destination, profile, |_| {}).unwrap();
    assert!(detour.edge_ids.contains(&b_d), "{:?}", detour.edge_ids);
    assert!(detour.cost > expected.cost + 1.0);
    for forward_only in [false, true] {
        dijkstra::force(forward_only);
        let actual = route(&compact, origin, destination, profile, |_| {});
        dijkstra::force(false);
        let actual = actual.unwrap();
        assert_eq!(
            actual.edge_ids, expected.edge_ids,
            "forward only {forward_only}"
        );
        assert!((actual.cost - expected.cost).abs() <= 1e-6 * expected.cost);
    }
    // Walkers are not restricted: every search takes the straight way.
    let walk = route(
        &compact,
        origin,
        destination,
        MobilityProfile::OnFoot,
        |_| {},
    )
    .unwrap();
    assert!(!walk.edge_ids.contains(&y_a) && !walk.edge_ids.contains(&b_d));

    // Matrices, with enough points to search in rows.
    let points = (0..36)
        .map(|index| GeoPoint::new(0.00002, 0.0001 + 0.0048 * f64::from(index) / 35.0))
        .chain([origin, destination])
        .collect::<Vec<_>>();
    let mut request = crate::RouteMatrixRequest::new(points.clone(), points.clone(), profile);
    request.options.max_correlation_distance_m = 60;
    let reference = exact.matrix(request.clone());
    let cell = reference
        .cells
        .iter()
        .find(|cell| cell.source_index == 36 && cell.target_index == 37)
        .unwrap();
    assert!((cell.cost.unwrap() - expected.cost).abs() <= 1e-6 * expected.cost);
    for mode in [MatrixMode::Pairwise, MatrixMode::Rows] {
        test_mode::set(Some(mode));
        let actual = compact.matrix(request.clone());
        test_mode::set(None);
        assert_matrix_parity(&reference, &actual);
    }

    // An isochrone whose cost contour lies between the legal route and the
    // detour reaches the destination edge only through Y.
    let contour = (expected.cost + detour.cost) / 2.0;
    let mut isochrone = crate::RouteIsochroneRequest::new(
        origin,
        profile,
        vec![
            crate::RouteIsochroneContour {
                contour,
                metric: crate::RouteIsochroneMetric::Cost,
            },
            crate::RouteIsochroneContour {
                contour: contour * 0.8,
                metric: crate::RouteIsochroneMetric::TimeSeconds,
            },
        ],
    );
    isochrone.options.max_correlation_distance_m = 60;
    let reference = exact.isochrone(isochrone.clone()).unwrap();
    let one_label_reach = one_label.isochrone(isochrone.clone()).unwrap();
    let actual = compact.isochrone(isochrone.clone()).unwrap();
    assert_ne!(
        reference.buckets[0].edge_ids,
        one_label_reach.buckets[0].edge_ids
    );
    assert_isochrone_parity(&reference, &actual, "trap isochrone");

    // On an overlay patch: an unusable connector and a full block off the
    // route keep the same answers.
    let o_s = edge_between(&graph, 1, 1, 2);
    let c_t = edge_between(&graph, 5, 5, 6);
    let t_c = edge_between(&graph, 5, 6, 5);
    let mut overlay = crate::RouteDynamicOverlay::from_blocked_ranges(&[crate::RouteEdgeRange {
        edge_id: t_c,
        start_fraction: 0.0,
        end_fraction: 1.0,
    }]);
    overlay.virtual_edges.push(edge_connector(
        1,
        crate::RouteDynamicVirtualEdgeKind::CrossCountry,
        (o_s, 0.5),
        (c_t, 0.5),
        1e7,
    ));
    overlay.refresh_diagnostics();
    let configure = |request: &mut LegRequest| {
        request.options.dynamic_overlay = Some(overlay.clone());
    };
    let expected = route(&exact, origin, destination, profile, configure).unwrap();
    assert!(expected.edge_ids.contains(&y_a), "{:?}", expected.edge_ids);
    assert!(
        route(&one_label, origin, destination, profile, configure)
            .is_none_or(|detour| detour.cost > expected.cost + 1.0)
    );
    for forward_only in [false, true] {
        dijkstra::force(forward_only);
        let actual = route(&compact, origin, destination, profile, configure);
        dijkstra::force(false);
        let actual = actual.unwrap();
        assert_eq!(
            actual.edge_ids, expected.edge_ids,
            "overlay forward only {forward_only}"
        );
        assert!((actual.cost - expected.cost).abs() <= 1e-6 * expected.cost);
    }
    request.options.dynamic_overlay = Some(overlay.clone());
    let reference = exact.matrix(request.clone());
    for mode in [MatrixMode::Pairwise, MatrixMode::Rows] {
        test_mode::set(Some(mode));
        let actual = compact.matrix(request.clone());
        test_mode::set(None);
        assert_matrix_parity(&reference, &actual);
    }
    // The reference isochrone models full blocks only: block by id there.
    let mut by_id = isochrone.clone();
    by_id.options.blocked_edge_ids.insert(t_c);
    isochrone.options.dynamic_overlay = Some(overlay);
    let reference = exact.isochrone(by_id).unwrap();
    let actual = compact.isochrone(isochrone).unwrap();
    assert_isochrone_parity(&reference, &actual, "trap overlay isochrone");
}

/// Random via-way restrictions on the grid, overlapping and nested (one
/// sequence's suffix starting another's, shared prefixes, U-turns), for
/// wheeled traffic and for every ground mode: random walks, plus windows of
/// exact routes (which forbid the cheapest way through an edge, the case one
/// label per edge gets wrong). Compact routes (bidirectional and forward
/// only), matrices in every mode, isochrones and overlay routes and matrices
/// equal the exact reference's.
#[test]
fn package_searches_are_exact_under_random_complex_restrictions() {
    use super::search::patch::dijkstra;
    use crate::matrix::{MatrixMode, test_mode};
    let dir = tempdir().unwrap();
    let mut graph = grid_graph();
    let points = grid_points()
        .into_iter()
        .chain(many_grid_targets().into_iter().step_by(9))
        .collect::<Vec<_>>();
    let mut out = std::collections::BTreeMap::<i64, Vec<(i64, u64)>>::new();
    for edge in graph.graph.edge_references() {
        let (Some(from), Some(to)) = (
            graph.graph.node(edge.source()).source_node_id,
            graph.graph.node(edge.target()).source_node_id,
        ) else {
            continue;
        };
        out.entry(from)
            .or_default()
            .push((to, edge.weight().edge_id));
    }
    let nodes = out.keys().copied().collect::<Vec<_>>();
    let mut seed = 0x9e37_79b9_7f4a_7c15_u64;
    let mut below = |bound: usize| {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed % bound as u64) as usize
    };
    let mut restrictions = Vec::new();
    let restriction = |walk: &[u64], index: usize| ResolvedEdgeRestriction::Complex {
        relation_id: 1_000 + index as i64,
        restriction: "no_straight_on".into(),
        from_edge_id: walk[0],
        to_edge_id: walk[walk.len() - 1],
        via_edge_ids: walk[1..walk.len() - 1].to_vec(),
        osm_mode_mask: OsmMode::MOTOR_VEHICLE,
        mobility_mode_mask: if index.is_multiple_of(3) {
            MobilityMode::ALL_GROUND
        } else {
            MobilityMode::WHEELED
        },
    };
    while restrictions.len() < 30 {
        // A walk of 2 to 4 edges: required prefix, then the restricted edge.
        let mut node = nodes[below(nodes.len())];
        let mut walk = Vec::new();
        for _ in 0..2 + below(3) {
            let Some(next) = out.get(&node).filter(|next| !next.is_empty()) else {
                break;
            };
            let (to, edge) = next[below(next.len())];
            walk.push(edge);
            node = to;
        }
        if walk.len() >= 2 {
            restrictions.push(restriction(&walk, restrictions.len()));
        }
    }
    for round in 0..3 {
        graph.set_resolved_restrictions(restrictions.clone());
        let reader = persist(&graph, &dir.path().join(format!("round-{round}.mrt")));
        let exact = reader.reference().with_path_context(4);
        for _ in 0..12 {
            let (origin, destination) = (points[below(points.len())], points[below(points.len())]);
            let profile = if below(3) == 0 {
                MobilityProfile::OnFoot
            } else {
                MobilityProfile::Wheeled
            };
            let Some(found) = route(&exact, origin, destination, profile, |_| {}) else {
                continue;
            };
            let length = 3 + below(2);
            if found.edge_ids.len() < length + 1 {
                continue;
            }
            let start = below(found.edge_ids.len() - length);
            let index = restrictions.len();
            let mut window = restriction(&found.edge_ids[start..start + length], index);
            if profile == MobilityProfile::OnFoot
                && let ResolvedEdgeRestriction::Complex {
                    mobility_mode_mask, ..
                } = &mut window
            {
                *mobility_mode_mask = MobilityMode::ALL_GROUND;
            }
            restrictions.push(window);
        }
    }
    graph.set_resolved_restrictions(restrictions);
    let reader = persist(&graph, &dir.path().join("restricted-grid.mrt"));
    let (one_label, compact) = engines(&reader);
    let exact = reader.reference().with_path_context(4);
    let overlay = grid_overlay(&graph);
    let (mut compared, mut cheaper_than_one_label) = (0, 0);
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        for with_overlay in [false, true] {
            let configure = |request: &mut LegRequest| {
                if with_overlay {
                    request.options.dynamic_overlay = Some(overlay.clone());
                }
            };
            for origin in &points {
                for destination in &points {
                    if origin == destination {
                        continue;
                    }
                    let context =
                        format!("{profile:?} overlay {with_overlay} {origin:?} -> {destination:?}");
                    let expected = route(&exact, *origin, *destination, profile, configure);
                    let loose = route(&one_label, *origin, *destination, profile, configure);
                    for forward_only in [false, true] {
                        dijkstra::force(forward_only);
                        let actual = route(&compact, *origin, *destination, profile, configure);
                        dijkstra::force(false);
                        assert_eq!(expected.is_some(), actual.is_some(), "{context}");
                        let (Some(expected), Some(actual)) = (&expected, actual) else {
                            continue;
                        };
                        assert!(
                            (actual.cost - expected.cost).abs() <= 1e-4 * expected.cost.max(1.0),
                            "{context} forward only {forward_only}: exact {} vs compact {}",
                            expected.cost,
                            actual.cost
                        );
                        compared += 1;
                    }
                    if let (Some(expected), Some(loose)) = (&expected, &loose)
                        && expected.cost < loose.cost - 1e-6
                    {
                        cheaper_than_one_label += 1;
                    }
                }
            }
            // Sources off the edge ends: a matrix row counts every origin
            // candidate as traversed, where routes and the reference do not
            // count one that starts at the end of its edge.
            let mut request =
                crate::RouteMatrixRequest::new(grid_points(), many_grid_targets(), profile);
            request.options.max_correlation_distance_m = 60;
            if with_overlay {
                request.options.dynamic_overlay = Some(overlay.clone());
            }
            let reference = exact.matrix(request.clone());
            for mode in [MatrixMode::Pairwise, MatrixMode::Rows] {
                test_mode::set(Some(mode));
                let actual = compact.matrix(request.clone());
                test_mode::set(None);
                assert_matrix_parity(&reference, &actual);
            }
        }
        let limits = match profile {
            MobilityProfile::OnFoot => [60.0, 150.0, 300.0],
            _ => [20.0, 45.0, 90.0],
        };
        for origin in grid_points() {
            let mut request = grid_isochrone_request(profile, limits);
            request.origin = origin;
            let reference = exact.isochrone(request.clone()).unwrap();
            let actual = compact.isochrone(request).unwrap();
            assert_isochrone_parity(&reference, &actual, &format!("{profile:?} {origin:?}"));
        }
    }
    eprintln!(
        "restriction parity: {compared} routes compared, {cheaper_than_one_label} pairs cheaper \
         than with one label per edge"
    );
    assert!(compared > 1000, "{compared}");
    assert!(cheaper_than_one_label > 5, "{cheaper_than_one_label}");
}

/// Complex-restriction statistics of a routing package
/// (`WAYFARER_RESTRICTION_STATS_ARCHIVE=<routing.mrt>`): restriction count
/// and prefix lengths, the automata's states, how many states overlap
/// another sequence, and the edges that may carry a non-root state.
#[test]
#[ignore = "needs WAYFARER_RESTRICTION_STATS_ARCHIVE"]
fn complex_restriction_statistics() {
    let Some(archive) = std::env::var_os("WAYFARER_RESTRICTION_STATS_ARCHIVE") else {
        return;
    };
    let archive = Path::new(&archive);
    let graph = RoutingPackage::open(archive).unwrap();
    let view = graph.view();
    let mut lengths = std::collections::BTreeMap::<usize, usize>::new();
    let mut edge_uses = std::collections::HashMap::<u32, std::collections::BTreeSet<usize>>::new();
    for row in 0..view.complex_to.len() {
        let prefix = &view.complex_prefix_pool
            [view.complex_prefix_start[row] as usize..view.complex_prefix_start[row + 1] as usize];
        *lengths.entry(prefix.len()).or_default() += 1;
        for edge in prefix.iter().chain(std::iter::once(&view.complex_to[row])) {
            edge_uses.entry(*edge).or_default().insert(row);
        }
    }
    let sharing = (0..view.complex_to.len())
        .filter(|row| {
            edge_uses
                .values()
                .any(|rows| rows.len() > 1 && rows.contains(row))
        })
        .count();
    println!(
        "complex restrictions: {}, prefix lengths {lengths:?}, sharing an edge with another: {sharing}",
        view.complex_to.len()
    );
    // `WAYFARER_RESTRICTION_STATS_EDGES=<file>`: the ids of every
    // edge in a forbidden sequence, one per line (to pick routes near them).
    if let Some(path) = std::env::var_os("WAYFARER_RESTRICTION_STATS_EDGES") {
        let mut ids = edge_uses
            .keys()
            .map(|edge| view.edge_id(*edge as usize).to_string())
            .collect::<Vec<_>>();
        ids.sort();
        std::fs::write(path, ids.join("\n")).unwrap();
    }
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        let automata = view.complex_automata(profile_bit(profile));
        println!(
            "{profile:?}: {} restrictions, states (forward, backward) {:?}, overlapping states {:?}, member edges {}",
            automata.restriction_count(),
            automata.state_counts(),
            automata.overlapping_state_counts(),
            automata.member_count()
        );
    }
}

// ---------------------------------------------------------------------------
// Search-core review fixes (2026-10-28): U-turns, loops back onto the origin
// edge, approach costs, the exact stop, override bounds, budgets
// ---------------------------------------------------------------------------

fn small_graph(name: &str, ways: Vec<RoutableWay>) -> RoadGraph {
    let mut builder = RoadGraphBuilder::new(name);
    for way in ways {
        builder.push_way(way);
    }
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    graph
}

/// The package route equals the reference route: path and cost.
fn assert_route_parity(
    oracle: &ReferenceEngine<'_>,
    compact: &RoutingEngine,
    origin: GeoPoint,
    destination: GeoPoint,
    configure: impl Fn(&mut LegRequest),
) -> LegResult {
    let expected = route(
        oracle,
        origin,
        destination,
        MobilityProfile::Wheeled,
        &configure,
    )
    .expect("reference route");
    let actual = route(
        compact,
        origin,
        destination,
        MobilityProfile::Wheeled,
        &configure,
    )
    .expect("package route");
    assert_eq!(expected.edge_ids, actual.edge_ids);
    assert!(
        (expected.cost - actual.cost).abs() <= 1e-6 * expected.cost.max(1.0),
        "reference {} vs package {}",
        expected.cost,
        actual.cost
    );
    actual
}

/// A dead-end spur: a route that starts heading into it must turn back at
/// its end, which costs a U-turn; an isochrone reaches the spur's way out
/// the same way (the tile-backed expansion never reversed, so it missed it).
#[test]
fn u_turns_onto_the_opposing_edge_pay_the_u_turn_cost() {
    let dir = tempdir().unwrap();
    let both = DirectionalAccess::Both;
    let residential = RoadType::Residential;
    let graph = small_graph(
        "u-turn",
        vec![
            way(1, (0.0, 0.0), (0.0, 0.002), 10, 11, residential, both),
            way(2, (0.0, 0.002), (0.0, 0.004), 11, 12, residential, both),
        ],
    );
    let (into_spur, out_of_spur) = (
        edge_between(&graph, 2, 11, 12),
        edge_between(&graph, 2, 12, 11),
    );
    let reader = persist(&graph, &dir.path().join("u-turn.mrt"));
    let (oracle, compact) = engines(&reader);
    let (origin, destination) = (GeoPoint::new(0.0, 0.003), GeoPoint::new(0.0, 0.001));
    let eastbound = |request: &mut LegRequest| {
        request.origin_options.heading_deg = Some(90);
        request.origin_options.heading_tolerance_deg = Some(20);
    };
    let turned = assert_route_parity(&oracle, &compact, origin, destination, eastbound);
    assert_eq!(&turned.edge_ids[..2], &[into_spur, out_of_spur]);
    let direct = assert_route_parity(&oracle, &compact, origin, destination, |_| {});
    assert_eq!(direct.edge_ids[0], out_of_spur);
    let u_turn =
        crate::costing::U_TURN_LEGACY_COST * crate::costing::TURN_COST_SECONDS_PER_LEGACY_UNIT;
    // Turning back adds the spur's far half twice and the U-turn.
    assert!(
        turned.cost > direct.cost + u_turn,
        "{} vs {}",
        turned.cost,
        direct.cost
    );

    let request = crate::RouteIsochroneRequest::new(
        destination,
        MobilityProfile::Wheeled,
        vec![crate::RouteIsochroneContour {
            contour: 600.0,
            metric: crate::RouteIsochroneMetric::TimeSeconds,
        }],
    );
    let expected = oracle.isochrone(request.clone()).unwrap();
    let actual = compact.isochrone(request).unwrap();
    assert_isochrone_parity(&expected, &actual, "u-turn");
    assert!(actual.buckets[0].edge_ids.contains(&out_of_spur));
}

/// A one-way loop: a destination behind the origin on the origin's own
/// one-way edge is reached around the loop, onto the stretch the origin
/// seed did not cover (it used to be `NoPath`), by routes and matrices.
#[test]
fn a_destination_behind_the_origin_on_a_one_way_edge_is_reached_around_the_loop() {
    let dir = tempdir().unwrap();
    let both = DirectionalAccess::Both;
    let residential = RoadType::Residential;
    let graph = small_graph(
        "one-way-loop",
        vec![
            way(
                1,
                (0.0, 0.0),
                (0.0, 0.002),
                10,
                11,
                residential,
                DirectionalAccess::Forward,
            ),
            way(2, (0.0, 0.002), (0.002, 0.002), 11, 12, residential, both),
            way(3, (0.002, 0.002), (0.002, 0.0), 12, 13, residential, both),
            way(4, (0.002, 0.0), (0.0, 0.0), 13, 10, residential, both),
        ],
    );
    let one_way = edge_between(&graph, 1, 10, 11);
    let reader = persist(&graph, &dir.path().join("loop.mrt"));
    let (oracle, compact) = engines(&reader);
    let (origin, destination) = (GeoPoint::new(0.0, 0.0015), GeoPoint::new(0.0, 0.0005));
    for force in [false, true] {
        crate::package::search::patch::dijkstra::force(force);
        let route = assert_route_parity(&oracle, &compact, origin, destination, |_| {});
        assert_eq!(route.edge_ids.len(), 5, "{:?}", route.edge_ids);
        assert_eq!(route.edge_ids.first(), Some(&one_way));
        assert_eq!(route.edge_ids.last(), Some(&one_way));
        crate::package::search::patch::dijkstra::force(false);
        let mut request = crate::RouteMatrixRequest::new(
            vec![origin],
            vec![destination],
            MobilityProfile::Wheeled,
        );
        request.options.max_correlation_distance_m = 60;
        let cells = compact.matrix(request.clone());
        assert_matrix_parity(&oracle.matrix(request), &cells);
        assert_eq!(cells.cells[0].edge_ids, route.edge_ids);
    }
}

/// Snap distance enters the search as an approach cost: a candidate 45 m
/// away no longer wins with a direct same-edge path when the nearest
/// candidate's route costs less than that direct path plus the extra
/// approach. Routes, matrices and isochrones agree with the reference, and
/// reported costs leave the approach out.
#[test]
fn far_candidates_pay_their_approach() {
    let dir = tempdir().unwrap();
    let graph = junction_graph();
    let (j_n, n_ne) = (
        edge_between(&graph, 3, 11, 13),
        edge_between(&graph, 5, 13, 14),
    );
    let reader = persist(&graph, &dir.path().join("junction.mrt"));
    let (oracle, compact) = engines(&reader);
    let (origin, destination) = (GeoPoint::new(0.0016, 0.002), GeoPoint::new(0.002, 0.0031));
    let route = assert_route_parity(&oracle, &compact, origin, destination, |_| {});
    // Not the direct path from the junction node 45 m away along N -> NE.
    assert_eq!(route.edge_ids, vec![j_n, n_ne]);
    let records = |id: u64| {
        compact
            .route_graph_view()
            .search_edge_by_id(id)
            .expect("record")
    };
    let options = crate::RouteOptions::default();
    let policy = crate::costing::CostingPolicy::new(MobilityProfile::Wheeled, &options);
    let cost = |id: u64| match policy.edge_cost(&records(id)) {
        crate::costing::EdgeCostDecision::Allowed(costed) => costed.cost,
        _ => panic!("edge {id} is not accessible"),
    };
    let unpriced = cost(j_n) * 0.2
        + policy.transition_cost_for_records(&records(j_n), &records(n_ne))
        + cost(n_ne) * 0.55;
    assert!(
        (route.cost - unpriced).abs() <= 1e-3,
        "{} vs {unpriced}",
        route.cost
    );

    let mut request =
        crate::RouteMatrixRequest::new(vec![origin], vec![destination], MobilityProfile::Wheeled);
    request.options.max_correlation_distance_m = 60;
    let cells = compact.matrix(request.clone());
    assert_matrix_parity(&oracle.matrix(request), &cells);
    assert_eq!(cells.cells[0].edge_ids, route.edge_ids);

    let mut request = crate::RouteIsochroneRequest::new(
        origin,
        MobilityProfile::Wheeled,
        vec![crate::RouteIsochroneContour {
            contour: 40.0,
            metric: crate::RouteIsochroneMetric::TimeSeconds,
        }],
    );
    request.options.max_correlation_distance_m = 60;
    let expected = oracle.isochrone(request.clone()).unwrap();
    let actual = compact.isochrone(request).unwrap();
    assert_isochrone_parity(&expected, &actual, "approach");
}

/// Asserts that `bound` is consistent on every edge of `costs` (forward, or
/// `reversed`): it drops by at most the edge's cost. Returns the edges checked.
fn assert_consistent(
    g: &SearchGraph<'_>,
    costs: &[f64],
    reversed: bool,
    bound: impl Fn(u32) -> f64,
    context: &str,
) -> usize {
    let mut checked = 0;
    for (edge, cost) in costs.iter().enumerate() {
        if !cost.is_finite() {
            continue;
        }
        let (mut from, mut to) = (g.edge_source(edge), g.edge_target[edge]);
        if reversed {
            std::mem::swap(&mut from, &mut to);
        }
        let (high, low) = (bound(from), bound(to));
        // An infinite bound proves the node useless.
        if high.is_finite() {
            assert!(
                high <= low + cost + 1e-6 * high.max(1.0),
                "{context} edge {edge} {reversed}: {high} > {low} + {cost}"
            );
            checked += 1;
        }
    }
    checked
}

/// Across every usable edge `u -> v` of a column, the CCH potential towards
/// a target set drops by at most the edge's cost, and the potential from it
/// rises by at most that: the bounds are consistent, which the default
/// bidirectional stop needs to be exact.
#[test]
fn cch_potentials_are_consistent() {
    let dir = tempdir().unwrap();
    let none = rustc_hash::FxHashMap::default();
    for (name, graph) in [("grid", grid_graph()), ("costing", costing_grid_graph())] {
        let reader = persist(&graph, &dir.path().join(format!("{name}.mrt")));
        let g = reader.package.view();
        let nodes = (0..g.node_count as u32).collect::<Vec<_>>();
        let mut checked = 0;
        for profile in ALL_TEST_PROFILES {
            let handle = reader
                .package
                .request_column(profile, &Default::default(), true);
            let column = handle.column(&g);
            let Some(plan) = search::potential_plan(g, &column, &none, false) else {
                continue;
            };
            let costs = (0..g.edge_count as u32)
                .map(|edge| f64::from(column.cost(edge)))
                .collect::<Vec<_>>();
            let scratch = <[std::cell::RefCell<cch::PotentialScratch>; 2]>::default();
            for (targets, reversed) in nodes
                .chunks(3)
                .flat_map(|targets| [(targets, false), (targets, true)])
            {
                let potential = plan.potential(targets, reversed, &[], &scratch[0], &scratch[1]);
                for target in targets {
                    assert_eq!(potential.get(*target), 0.0);
                }
                let context = format!("{name} {profile:?} {targets:?}");
                checked +=
                    assert_consistent(&g, &costs, reversed, |node| potential.get(node), &context);
            }
        }
        assert!(checked > 1_000, "{name}: {checked}");
    }
}

/// Gated CCH potentials (see `search::gates`) under random discounts and
/// opened edges stay consistent on every edge at its request cost, forward
/// and backward, and vanish at their targets.
#[test]
fn discount_gate_bounds_are_consistent() {
    use super::search::gates::DiscountGate;
    let dir = tempdir().unwrap();
    let graph = costing_grid_graph();
    let reader = persist(&graph, &dir.path().join("gates.mrt"));
    let g = reader.package.view();
    let mut seed = 0x9E37_79B9_7F4A_7C15u64;
    let mut next = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    let scratch = <[std::cell::RefCell<cch::PotentialScratch>; 2]>::default();
    let mut checked = 0;
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        let handle = reader
            .package
            .request_column(profile, &Default::default(), true);
        let column = handle.column(&g);
        for round in 0..8 {
            // Discounts on a random share of the edges, and every edge the
            // column cannot use opened at a random cost per metre.
            let share = [0.02, 0.1, 0.3, 0.6][round % 4];
            let mut request_cost = (0..g.edge_count as u32)
                .map(|edge| f64::from(column.cost(edge)))
                .collect::<Vec<_>>();
            let mut gates = Vec::new();
            let mut overrides = rustc_hash::FxHashMap::default();
            for edge in 0..g.edge_count as u32 {
                let base = f64::from(column.cost(edge));
                let cost = if base.is_finite() {
                    if next() >= share {
                        continue;
                    }
                    base * (0.02 + 0.9 * next())
                } else if round % 2 == 1 {
                    f64::from(g.edge_length[edge as usize]) * (0.01 + 0.5 * next())
                } else {
                    continue;
                };
                // The bounds see the cost the search reads.
                let cost = f64::from(cost as f32);
                request_cost[edge as usize] = cost;
                overrides.insert(edge, cost as f32);
                gates.push(DiscountGate {
                    edge,
                    source: g.edge_source(edge as usize),
                    target: g.edge_target[edge as usize],
                    cost,
                    ratio: if base.is_finite() { cost / base } else { 0.0 },
                });
            }
            let plan = search::potential_plan(g, &column, &overrides, true).expect("gated plan");
            for reversed in [false, true] {
                let targets = (0..3)
                    .map(|_| (next() * g.node_count as f64) as u32)
                    .collect::<Vec<_>>();
                let potential =
                    plan.potential(&targets, reversed, &gates, &scratch[0], &scratch[1]);
                for target in &targets {
                    assert_eq!(potential.get(*target), 0.0);
                }
                let context = format!("{profile:?} round {round}");
                checked += assert_consistent(
                    &g,
                    &request_cost,
                    reversed,
                    |node| potential.get(node),
                    &context,
                );
            }
        }
    }
    assert!(checked > 1_000, "{checked}");
}

/// Random discounts, fast speed overrides and opened edges, with and without
/// an overlay patch, under both bidirectional stops and forward only: routes
/// and matrices keep the reference's costs through the discount gates, and
/// the scaled fallback (past the gate limit) does too.
#[test]
fn discount_gates_keep_routes_exact() {
    use crate::package::search::patch::dijkstra;
    let dir = tempdir().unwrap();
    let graph = grid_graph();
    let reader = persist(&graph, &dir.path().join("gates.mrt"));
    let (oracle, compact) = engines(&reader);
    let edges = graph
        .graph
        .edge_references()
        .map(|edge| edge.weight().edge_id)
        .collect::<Vec<_>>();
    let footway = [
        grid_edge(&graph, (1, 3), (2, 3)),
        grid_edge(&graph, (2, 3), (1, 3)),
    ];
    let overlay = grid_overlay(&graph);
    let mut seed = 0x2545_F491_4F6C_DD1Du64;
    let mut next = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    let points = grid_points();
    let mut compared = 0;
    for round in 0..6 {
        let mut options = crate::RouteOptions::default();
        let share = [0.05, 0.2, 0.5][round % 3];
        for edge in &edges {
            if next() < share {
                if next() < 0.7 {
                    options
                        .edge_cost_multipliers
                        .insert(*edge, 0.05 + 0.9 * next());
                } else {
                    options
                        .edge_speed_overrides
                        .insert(*edge, 60.0 + 100.0 * next());
                }
            }
        }
        if round % 2 == 1 {
            options.opened_edge_ids.extend(footway);
            options.edge_cost_multipliers.insert(footway[0], 0.05);
        }
        for (overlaid, exact, forward, fallback) in [
            (false, false, false, false),
            (false, true, false, false),
            (false, false, true, false),
            (true, false, false, false),
            (true, true, false, false),
            (false, false, false, true),
            (true, false, false, true),
        ] {
            crate::debug::force_exact_stop(exact);
            dijkstra::force(forward);
            super::search::gates::force_gate_limit(fallback.then_some(0));
            for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
                let configure = |request: &mut LegRequest| {
                    request.options = crate::RouteOptions {
                        max_correlation_distance_m: request.options.max_correlation_distance_m,
                        ..options.clone()
                    };
                    if overlaid {
                        request.options.dynamic_overlay = Some(overlay.clone());
                    }
                };
                for origin in &points {
                    for destination in &points {
                        if origin == destination {
                            continue;
                        }
                        let expected = route(&oracle, *origin, *destination, profile, configure);
                        let actual = route(&compact, *origin, *destination, profile, configure);
                        let context = format!(
                            "round {round} overlay {overlaid} exact {exact} forward {forward} \
                             fallback {fallback} {profile:?} {origin:?} -> {destination:?}"
                        );
                        assert_eq!(expected.is_some(), actual.is_some(), "{context}");
                        if let (Some(expected), Some(actual)) = (expected, actual) {
                            assert!(
                                (expected.cost - actual.cost).abs()
                                    <= 1e-4 * expected.cost.max(1.0),
                                "{context}: reference {} vs package {}",
                                expected.cost,
                                actual.cost
                            );
                            compared += 1;
                        }
                    }
                }
                if !forward {
                    let mut request =
                        crate::RouteMatrixRequest::new(points.clone(), points.clone(), profile);
                    request.options = options.clone();
                    request.options.max_correlation_distance_m = 60;
                    if overlaid {
                        request.options.dynamic_overlay = Some(overlay.clone());
                    }
                    assert_matrix_parity(&oracle.matrix(request.clone()), &compact.matrix(request));
                }
            }
            dijkstra::force(false);
            crate::debug::force_exact_stop(false);
            super::search::gates::force_gate_limit(None);
        }
    }
    assert!(compared > 1_000, "{compared}");
}

/// The default stop (on consistent bounds) and the exact (symmetric) stop
/// agree with the reference on every grid route, plain and overlaid.
#[test]
fn routes_match_reference_with_either_stop() {
    let dir = tempdir().unwrap();
    let graph = grid_graph();
    let reader = persist(&graph, &dir.path().join("grid.mrt"));
    let (oracle, compact) = engines(&reader);
    let overlay = grid_overlay(&graph);
    for exact in [false, true] {
        crate::debug::force_exact_stop(exact);
        assert_routes_match_reference(&oracle, &compact, &overlay, exact);
    }
    crate::debug::force_exact_stop(false);
}

fn assert_routes_match_reference(
    oracle: &ReferenceEngine<'_>,
    compact: &RoutingEngine,
    overlay: &crate::RouteDynamicOverlay,
    exact: bool,
) {
    let mut compared = 0;
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        for origin in grid_points() {
            for destination in grid_points() {
                if origin == destination {
                    continue;
                }
                for overlaid in [false, true] {
                    let configure = |request: &mut LegRequest| {
                        if overlaid {
                            request.options.dynamic_overlay = Some(overlay.clone());
                        }
                    };
                    let expected = route(oracle, origin, destination, profile, configure);
                    let actual = route(compact, origin, destination, profile, configure);
                    let context = format!(
                        "exact stop {exact}: {profile:?} {origin:?} -> {destination:?} {overlaid}"
                    );
                    assert_eq!(expected.is_some(), actual.is_some(), "{context}");
                    if let (Some(expected), Some(actual)) = (expected, actual) {
                        assert!(
                            (expected.cost - actual.cost).abs() <= 1e-6 * expected.cost.max(1.0),
                            "{context}: reference {} vs package {}",
                            expected.cost,
                            actual.cost
                        );
                        compared += 1;
                    }
                }
            }
        }
    }
    assert!(compared > 100, "{compared}");
}

/// Overrides that lower costs become discount gates: the potentials keep
/// their scale and the straight-line factor drops to the least cost per
/// metre of the gates. Past the gate limit, the fallback scales the bounds by
/// the least ratio (not the product of all ratios), and so does the
/// potential plan.
#[test]
fn lowered_costs_become_gates_or_scale_the_bounds_by_the_least_ratio() {
    let dir = tempdir().unwrap();
    let graph = grid_graph();
    let reader = persist(&graph, &dir.path().join("grid.mrt"));
    let package = &reader.package;
    let view = package.view();
    let handle = package.request_column(MobilityProfile::Wheeled, &Default::default(), true);
    let column = handle.column(&view);
    let lowered = [
        (resolve(&view, grid_edge(&graph, (2, 3), (2, 4))), 0.3),
        (resolve(&view, grid_edge(&graph, (3, 2), (3, 3))), 0.5),
    ];
    let overrides = lowered
        .iter()
        .map(|(edge, ratio)| (*edge, column.cost(*edge) * *ratio as f32))
        .collect::<rustc_hash::FxHashMap<_, _>>();
    let none = rustc_hash::FxHashMap::default();

    let gated = super::search::analysis::override_bounds(view, column, &overrides, &none);
    assert!(!gated.fell_back());
    let mut gate_edges = gated.gates.iter().map(|gate| gate.edge).collect::<Vec<_>>();
    gate_edges.sort_unstable();
    let mut expected = lowered.iter().map(|(edge, _)| *edge).collect::<Vec<_>>();
    expected.sort_unstable();
    assert_eq!(gate_edges, expected);
    for gate in &gated.gates {
        assert_eq!(gate.cost, f64::from(overrides[&gate.edge]));
    }
    assert!(gated.heuristic_cost_per_m <= column.heuristic_cost_per_m);
    assert!(gated.heuristic_cost_per_m >= column.heuristic_cost_per_m * 0.3 * (1.0 - 1e-6));
    let plain = search::potential_plan(view, &column, &none, false).expect("plan");
    let gated_plan = search::potential_plan(view, &column, &overrides, true).expect("plan");
    assert_eq!(gated_plan.factor, plain.factor);

    let bounds =
        super::search::analysis::override_bounds_with_limit(view, column, &overrides, &none, 1);
    assert!(bounds.fell_back() && bounds.gates.is_empty());
    let least = 0.3 * (1.0 - 1e-6);
    assert!(
        (bounds.heuristic_cost_per_m - column.heuristic_cost_per_m * least).abs()
            <= 1e-6 * column.heuristic_cost_per_m
    );
    let scaled = search::potential_plan(view, &column, &overrides, false).expect("plan");
    assert!((scaled.factor - plain.factor * least).abs() <= 1e-6 * plain.factor);
}

fn resolve(view: &SearchGraph<'_>, edge_id: u64) -> u32 {
    super::search::analysis::resolve_edge_id(*view, edge_id).expect("compact edge")
}

/// A candidate limit beyond what the searches track is clamped.
#[test]
fn candidate_limit_is_clamped() {
    let options = crate::RouteOptions {
        candidate_limit: 300,
        ..Default::default()
    };
    assert_eq!(crate::CorrelationPolicy::from(&options).candidate_limit, 16);
    let dir = tempdir().unwrap();
    let reader = persist(&grid_graph(), &dir.path().join("grid.mrt"));
    let (oracle, compact) = engines(&reader);
    let (origin, destination) = (grid_points()[0], grid_points()[3]);
    assert_route_parity(&oracle, &compact, origin, destination, |request| {
        request.options.candidate_limit = 300;
        request.options.max_correlation_distance_m = 400;
    });
}

/// One-to-many rows break equal-cost ties like routes, so a row's cell has
/// the route's path.
#[test]
fn matrix_rows_take_the_routes_paths() {
    let dir = tempdir().unwrap();
    let reader = persist(&grid_graph(), &dir.path().join("grid.mrt"));
    let (_, compact) = engines(&reader);
    let sources = &grid_points()[..3];
    let targets = many_grid_targets();
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        let mut request =
            crate::RouteMatrixRequest::new(sources.to_vec(), targets.clone(), profile);
        request.options.max_correlation_distance_m = 60;
        for cell in compact.matrix(request).cells {
            let (origin, destination) = (sources[cell.source_index], targets[cell.target_index]);
            let Some(route) = route(&compact, origin, destination, profile, |_| {}) else {
                assert!(cell.cost.is_none());
                continue;
            };
            assert_eq!(
                cell.edge_ids, route.edge_ids,
                "{profile:?} {origin:?} -> {destination:?}"
            );
        }
    }
}

/// A route search that runs out of budget after finding a route returns it,
/// flagged, with a gap that bounds how much dearer than optimal it is.
#[test]
fn routes_out_of_budget_keep_their_best_route() {
    let dir = tempdir().unwrap();
    let reader = persist(&grid_graph(), &dir.path().join("grid.mrt"));
    let (_, compact) = engines(&reader);
    let (mut flagged, mut failed) = (0, 0);
    for origin in grid_points() {
        for destination in grid_points() {
            let profile = MobilityProfile::Wheeled;
            if origin == destination {
                continue;
            }
            let Some(exact) = route(&compact, origin, destination, profile, |_| {}) else {
                continue;
            };
            let visited = exact.diagnostics.visited_edge_count;
            for budget in [visited / 2, visited.saturating_sub(1)] {
                let mut request = LegRequest::new(origin, destination, profile);
                request.options.max_correlation_distance_m = 60;
                request.options.max_visited_edge_count = Some(budget);
                match compact.route_leg(request) {
                    // The budget counts the search's own visits (not the
                    // direct-pair checks), so a route may still be proven.
                    Ok(bounded) if !bounded.diagnostics.search_budget_exceeded => {
                        assert_eq!(bounded.edge_ids, exact.edge_ids);
                    }
                    Ok(bounded) => {
                        let gap = bounded.diagnostics.search_optimality_gap_millicost.unwrap();
                        // Reported costs leave approach costs out, so compare
                        // routes between the same end edges.
                        if bounded.edge_ids.first() == exact.edge_ids.first()
                            && bounded.edge_ids.last() == exact.edge_ids.last()
                        {
                            assert!(bounded.cost >= exact.cost - 1e-6);
                            assert!(bounded.cost - exact.cost <= gap as f64 / 1e3 + 1e-6);
                        }
                        flagged += 1;
                    }
                    Err(error) => {
                        assert!(error.error.is_search_budget_exceeded(), "{error:?}");
                        failed += 1;
                    }
                }
            }
        }
    }
    assert!(
        flagged > 0 && failed > 0,
        "{flagged} flagged, {failed} failed"
    );
}

/// A desktop package pools a search state per matrix thread.
#[test]
fn desktop_pools_a_state_per_matrix_thread() {
    assert!(
        RoutePackageMemoryProfile::Desktop.pooled_search_states() >= rayon::current_num_threads()
    );
    assert_eq!(
        RoutePackageMemoryProfile::Mobile.pooled_search_states(),
        RoutePackageMemoryProfile::Mobile.matrix_threads().unwrap()
    );
}

/// Conditional tags are sparse overrides: requests at different evaluation
/// times share one column (the stored one for otherwise default options).
#[test]
fn conditional_evaluation_times_share_one_column() {
    let dir = tempdir().unwrap();
    let reader = persist(&costing_grid_graph(), &dir.path().join("costing.mrt"));
    let package = &reader.package;
    assert!(!package.view().conditional_edges.is_empty());
    let at = |hour, avoid_motorways| crate::RouteCostingOptions {
        avoid_motorways,
        conditional_restriction_eval_at_ms: Some(utc_ms(3, hour)),
        ..Default::default()
    };
    let derived = |options: &crate::RouteCostingOptions| match package.request_column(
        MobilityProfile::Wheeled,
        options,
        true,
    ) {
        columns::ColumnHandle::Derived(column) => Some(column),
        columns::ColumnHandle::Stored { .. } => None,
    };
    assert!(derived(&at(8, 0.0)).is_none());
    let (morning, night) = (
        derived(&at(8, 0.5)).unwrap(),
        derived(&at(23, 0.5)).unwrap(),
    );
    assert!(Arc::ptr_eq(&morning, &night));
}

/// Two ways around a block after a shared stretch: the alternative is
/// judged and reported by its cost under the request's own options (without
/// the search's penalty on the shared, penalised stretch), which is the cost
/// of routing it with the primary's own branch blocked, and its shared share
/// is by length.
#[test]
fn alternatives_report_their_own_cost() {
    let dir = tempdir().unwrap();
    let both = DirectionalAccess::Both;
    let residential = RoadType::Residential;
    let graph = small_graph(
        "diamond",
        vec![
            way(1, (0.0, -0.001), (0.0, 0.0), 10, 11, residential, both),
            way(2, (0.0, 0.0), (0.0, 0.0005), 11, 16, residential, both),
            way(3, (0.0, 0.0005), (0.001, 0.0015), 16, 12, residential, both),
            way(4, (0.001, 0.0015), (0.0, 0.0025), 12, 13, residential, both),
            way(
                5,
                (0.0, 0.0005),
                (-0.0012, 0.0015),
                16,
                14,
                residential,
                both,
            ),
            way(
                6,
                (-0.0012, 0.0015),
                (0.0, 0.0025),
                14,
                13,
                residential,
                both,
            ),
            way(7, (0.0, 0.0025), (0.0, 0.0035), 13, 15, residential, both),
        ],
    );
    let (shared, branch) = (
        edge_between(&graph, 2, 11, 16),
        [
            edge_between(&graph, 3, 16, 12),
            edge_between(&graph, 4, 12, 13),
        ],
    );
    let reader = persist(&graph, &dir.path().join("diamond.mrt"));
    let (oracle, compact) = engines(&reader);
    let (origin, destination) = (GeoPoint::new(0.0, -0.0005), GeoPoint::new(0.0, 0.003));
    // Candidates on the approach ways only, none at the junctions.
    let near = |request: &mut LegRequest| request.options.max_correlation_distance_m = 20;
    let primary = route(
        &compact,
        origin,
        destination,
        MobilityProfile::Wheeled,
        |request| {
            near(request);
            request.options.alternatives.requested_alternative_count = 1;
        },
    )
    .unwrap();
    assert_eq!(&primary.edge_ids[1..4], &[shared, branch[0], branch[1]]);
    let [alternative] = &primary.alternatives[..] else {
        panic!("{:?}", primary.diagnostics);
    };
    // The alternative runs over the penalised shared stretch too.
    assert_eq!(alternative.edge_ids[1], shared);
    let detour = assert_route_parity(&oracle, &compact, origin, destination, |request| {
        near(request);
        request.options.blocked_edge_ids.extend(branch);
    });
    assert_eq!(alternative.edge_ids, detour.edge_ids);
    assert!(
        (alternative.cost - detour.cost).abs() <= 1e-6 * detour.cost,
        "alternative {} vs detour {}",
        alternative.cost,
        detour.cost
    );
    assert!((alternative.duration_s - detour.duration_s).abs() <= 1e-6 * detour.duration_s);
    // Half of each approach way and the shared stretch: about a third of the
    // length (three of five edges by count).
    assert!(
        (0.2..0.45).contains(&alternative.shared_edge_ratio),
        "{}",
        alternative.shared_edge_ratio
    );
}

/// Deterministic standard normal draws (xorshift64* and Box-Muller).
struct Gaussian(u64);

impl Gaussian {
    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        ((self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    fn sample(&mut self) -> f64 {
        let (u, v) = (self.uniform(), self.uniform());
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

/// The point `distance_m` along `points`.
fn point_along(points: &[GeoPoint], distance_m: f64) -> GeoPoint {
    let mut covered = 0.0;
    for segment in points.windows(2) {
        let length = segment[0].distance_to(segment[1]);
        if length > 0.0 && covered + length >= distance_m {
            let t = (distance_m - covered) / length;
            return GeoPoint::new(
                segment[0].lat + (segment[1].lat - segment[0].lat) * t,
                segment[0].lng + (segment[1].lng - segment[0].lng) * t,
            );
        }
        covered += length;
    }
    *points.last().unwrap()
}

#[test]
fn map_matching_recovers_a_noisy_route_on_the_grid() {
    const SEEDS: u64 = 20;
    let graph = grid_graph();
    let engine = crate::test_support::engine(&graph);
    let points = grid_points();
    let route = route(
        &engine,
        points[0],
        points[1],
        MobilityProfile::Wheeled,
        |_| {},
    )
    .unwrap();
    assert!(route.edge_ids.len() >= 6, "{:?}", route.edge_ids);

    // The route's stretch of each edge (partial at either end), to know the
    // true edge of every sample.
    let view = engine.route_graph_view();
    let last = route.edge_ids.len() - 1;
    let stretches = route
        .edge_ids
        .iter()
        .enumerate()
        .map(|(at, edge_id)| {
            let length = f64::from(view.edge_by_id(*edge_id).unwrap().edge.distance_m);
            let from = if at == 0 {
                route.origin.fraction_along_edge
            } else {
                0.0
            };
            let to = if at == last {
                route.destination.fraction_along_edge
            } else {
                1.0
            };
            length * (to - from)
        })
        .collect::<Vec<_>>();
    let geometry_m = route
        .geometry
        .windows(2)
        .map(|segment| segment[0].distance_to(segment[1]))
        .sum::<f64>();
    let scale = stretches.iter().sum::<f64>() / geometry_m;

    // 1 Hz at 8 m/s with 5 m Gaussian noise; every 25th sample is an
    // outlier 40 m north and 40 m east of the road. Several noise seeds.
    let (mut scored, mut on_true_edge, mut genuine_rejected, mut sample_count) = (0, 0, 0, 0);
    for seed in 1..=SEEDS {
        let mut noise = Gaussian(0x9E37_79B9_7F4A_7C15 ^ seed);
        let (mut samples, mut truth) = (Vec::new(), Vec::new());
        let mut along = 0.0;
        while along <= geometry_m {
            let point = point_along(&route.geometry, along);
            let outlier = samples.len() % 25 == 12;
            let (north, east) = if outlier {
                (40.0, 40.0)
            } else {
                (5.0 * noise.sample(), 5.0 * noise.sample())
            };
            samples.push(crate::NavigationPositionSample {
                position: GeoPoint::new(
                    point.lat + north / 111_320.0,
                    point.lng + east / (111_320.0 * point.lat.to_radians().cos()),
                ),
                heading_degrees: None,
                speed_meters_per_second: None,
                sampled_at_ms: Some(samples.len() as i64 * 1000),
                accuracy_meters: None,
                provider: None,
            });
            let mut on = along * scale;
            let edge = stretches
                .iter()
                .position(|stretch| {
                    let inside = on <= *stretch;
                    on -= stretch;
                    inside
                })
                .unwrap_or(last);
            truth.push((!outlier).then_some(route.edge_ids[edge]));
            along += 8.0;
        }

        let result = crate::map_matching::match_navigation_trace(
            view,
            &samples,
            MobilityProfile::Wheeled,
            crate::MapMatchOptions::default(),
        )
        .unwrap();

        assert_eq!(result.sub_paths.len(), 1, "seed {seed}");
        assert_eq!(result.matched_edge_ids, route.edge_ids, "seed {seed}");
        let outliers = truth.iter().filter(|edge| edge.is_none()).count();
        let outliers_rejected = result
            .rejected_samples
            .iter()
            .filter(|sample| truth[sample.sample_index].is_none())
            .count();
        assert_eq!(outliers_rejected, outliers, "seed {seed}");
        genuine_rejected += result.rejected_sample_count - outliers_rejected;
        scored += samples.len() - outliers;
        on_true_edge += result
            .samples
            .iter()
            .filter(|sample| truth[sample.sample_index] == Some(sample.matched_edge_id))
            .count();
        sample_count += samples.len();
    }
    eprintln!(
        "grid traces: {SEEDS} seeds, {sample_count} samples, {on_true_edge}/{scored} genuine \
         samples on the true edge, {genuine_rejected} genuine samples rejected as outliers, \
         every path recovered and every outlier rejected"
    );
    assert!(
        on_true_edge as f64 >= 0.95 * scored as f64,
        "{on_true_edge}/{scored} samples on the true edge"
    );
}

// ---------------------------------------------------------------------------
// SAR stage 0 and cost-lowering overlays
// ---------------------------------------------------------------------------

/// SAR options with the `turn_restrictions` relaxation at `penalty_s`.
fn sar_turn_restrictions(penalty_s: f64) -> crate::RouteSarOptions {
    let mut sar = crate::RouteSarOptions::default();
    sar.relaxations.turn_restrictions = Some(crate::sar::SarTurnRestrictions { penalty_s });
    sar
}

/// [`grid_graph`] with random simple and complex restrictions for wheeled
/// vehicles, some overlapping (from `package_searches_are_exact_under_random_complex_restrictions`).
fn randomly_restricted_grid(seed: u64) -> RoadGraph {
    let mut graph = grid_graph();
    let mut out = std::collections::BTreeMap::<i64, Vec<(i64, u64)>>::new();
    for edge in graph.graph.edge_references() {
        let (Some(from), Some(to)) = (
            graph.graph.node(edge.source()).source_node_id,
            graph.graph.node(edge.target()).source_node_id,
        ) else {
            continue;
        };
        out.entry(from)
            .or_default()
            .push((to, edge.weight().edge_id));
    }
    let nodes = out.keys().copied().collect::<Vec<_>>();
    let mut seed = seed;
    let mut below = |bound: usize| {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed % bound as u64) as usize
    };
    let mut restrictions = Vec::new();
    while restrictions.len() < 40 {
        let mut node = nodes[below(nodes.len())];
        let mut walk = Vec::new();
        for _ in 0..2 + below(3) {
            let Some(next) = out.get(&node).filter(|next| !next.is_empty()) else {
                break;
            };
            let (to, edge) = next[below(next.len())];
            walk.push(edge);
            node = to;
        }
        if walk.len() < 2 {
            continue;
        }
        let index = restrictions.len() as i64;
        restrictions.push(if walk.len() == 2 && index % 2 == 0 {
            ResolvedEdgeRestriction::Simple {
                relation_id: 2_000 + index,
                restriction: "no_left_turn".into(),
                from_edge_id: walk[0],
                to_edge_id: walk[1],
                via_node_id: 0,
                osm_mode_mask: OsmMode::MOTOR_VEHICLE,
                mobility_mode_mask: MobilityMode::WHEELED,
            }
        } else {
            ResolvedEdgeRestriction::Complex {
                relation_id: 2_000 + index,
                restriction: "no_straight_on".into(),
                from_edge_id: walk[0],
                to_edge_id: walk[walk.len() - 1],
                via_edge_ids: walk[1..walk.len() - 1].to_vec(),
                osm_mode_mask: OsmMode::MOTOR_VEHICLE,
                mobility_mode_mask: MobilityMode::WHEELED,
            }
        });
    }
    graph.set_resolved_restrictions(restrictions);
    graph
}

/// R3: with restrictions as penalties, every package search (routes in both
/// directions and forward only, overlay routes, matrices in every mode,
/// isochrones) costs exactly what the reference does, with labels per path
/// context. A penalised route is never cheaper than the same route with no
/// restrictions at all, nor dearer than with them banned.
#[test]
fn sar_restriction_penalties_match_the_reference() {
    use super::search::patch::dijkstra;
    use crate::matrix::{MatrixMode, test_mode};
    let dir = tempdir().unwrap();
    let graph = randomly_restricted_grid(0x51a7_e5ed_0bad_cafe);
    let reader = persist(&graph, &dir.path().join("sar-grid.mrt"));
    let compact = RoutingEngine::from_package(reader.package.clone());
    let exact = reader.reference().with_path_context(4);
    let mut free_graph = graph.clone();
    free_graph.set_resolved_restrictions(Vec::new());
    let free = persist(&free_graph, &dir.path().join("free-grid.mrt"));
    let free = RoutingEngine::from_package(free.package.clone());
    let overlay = grid_overlay(&graph);
    let points = grid_points()
        .into_iter()
        .chain(many_grid_targets().into_iter().step_by(7))
        .collect::<Vec<_>>();
    let (mut compared, mut penalised) = (0, 0);
    for penalty in [0.0, 7.5, 60.0] {
        for with_overlay in [false, true] {
            let configure = |request: &mut LegRequest| {
                request.options.sar = Some(sar_turn_restrictions(penalty));
                if with_overlay {
                    request.options.dynamic_overlay = Some(overlay.clone());
                }
            };
            for origin in &points {
                for destination in &points {
                    if origin == destination {
                        continue;
                    }
                    let context = format!(
                        "penalty {penalty} overlay {with_overlay} {origin:?} -> {destination:?}"
                    );
                    let expected = route(
                        &exact,
                        *origin,
                        *destination,
                        MobilityProfile::Wheeled,
                        configure,
                    );
                    for forward_only in [false, true] {
                        dijkstra::force(forward_only);
                        let actual = route(
                            &compact,
                            *origin,
                            *destination,
                            MobilityProfile::Wheeled,
                            configure,
                        );
                        dijkstra::force(false);
                        assert_eq!(expected.is_some(), actual.is_some(), "{context}");
                        let (Some(expected), Some(actual)) = (&expected, actual) else {
                            continue;
                        };
                        assert!(
                            (actual.cost - expected.cost).abs() <= 1e-4 * expected.cost.max(1.0),
                            "{context} forward only {forward_only}: exact {} vs package {}",
                            expected.cost,
                            actual.cost
                        );
                        // The report charges what the search did.
                        let report = actual.sar.as_ref().expect("a SAR report");
                        let charged = report
                            .uses
                            .iter()
                            .filter(|used| used.kind == crate::SarRelaxationKind::TurnRestrictions)
                            .count();
                        penalised += usize::from(charged > 0 && penalty > 0.0);
                        compared += 1;
                    }
                    if with_overlay {
                        continue;
                    }
                    // Reported costs leave out approach costs, so only routes
                    // between the same candidates compare.
                    let same_ends = |left: &LegResult, right: &LegResult| {
                        left.origin.edge_id == right.origin.edge_id
                            && left.origin.fraction_along_edge == right.origin.fraction_along_edge
                            && left.destination.edge_id == right.destination.edge_id
                            && left.destination.fraction_along_edge
                                == right.destination.fraction_along_edge
                    };
                    let Some(expected) = expected else {
                        continue;
                    };
                    if let Some(unrestricted) = route(
                        &free,
                        *origin,
                        *destination,
                        MobilityProfile::Wheeled,
                        |_| {},
                    ) && same_ends(&expected, &unrestricted)
                    {
                        assert!(
                            expected.cost >= unrestricted.cost - 1e-6 * unrestricted.cost.max(1.0),
                            "{context}: a relaxation lowered the cost"
                        );
                        if penalty == 0.0 {
                            assert!(
                                (expected.cost - unrestricted.cost).abs()
                                    <= 1e-6 * unrestricted.cost.max(1.0),
                                "{context}"
                            );
                        }
                    }
                    if let Some(banned) = route(
                        &exact,
                        *origin,
                        *destination,
                        MobilityProfile::Wheeled,
                        |_| {},
                    ) && same_ends(&expected, &banned)
                    {
                        assert!(
                            expected.cost <= banned.cost + 1e-6 * banned.cost.max(1.0),
                            "{context}: penalised {} vs banned {}",
                            expected.cost,
                            banned.cost
                        );
                    }
                }
            }
            let mut request = crate::RouteMatrixRequest::new(
                grid_points(),
                many_grid_targets(),
                MobilityProfile::Wheeled,
            );
            request.options.max_correlation_distance_m = 60;
            request.options.sar = Some(sar_turn_restrictions(penalty));
            if with_overlay {
                request.options.dynamic_overlay = Some(overlay.clone());
            }
            let reference = exact.matrix(request.clone());
            for mode in [MatrixMode::Pairwise, MatrixMode::Rows] {
                test_mode::set(Some(mode));
                let actual = compact.matrix(request.clone());
                test_mode::set(None);
                assert_matrix_parity(&reference, &actual);
            }
        }
        for origin in grid_points() {
            let mut request = grid_isochrone_request(MobilityProfile::Wheeled, [20.0, 45.0, 90.0]);
            request.origin = origin;
            request.options.sar = Some(sar_turn_restrictions(penalty));
            let reference = exact.isochrone(request.clone()).unwrap();
            let actual = compact.isochrone(request).unwrap();
            assert_isochrone_parity(
                &reference,
                &actual,
                &format!("penalty {penalty} {origin:?}"),
            );
        }
    }
    eprintln!("SAR restriction parity: {compared} routes, {penalised} charged a penalty");
    assert!(compared > 500, "{compared}");
    assert!(penalised > 20, "{penalised}");
}

/// A turn ban that forces a detour: banned, the route detours; under R3 it
/// takes the turn for the penalty when that is cheaper, reports it, and
/// costs exactly the unrestricted route plus the penalty.
#[test]
fn sar_route_takes_a_restricted_turn_for_its_penalty_and_reports_it() {
    let dir = tempdir().unwrap();
    let mut graph = grid_graph();
    // From (2,1)->(2,2) (primary) the left turn north to (1,2) is banned.
    let from = grid_edge(&graph, (2, 1), (2, 2));
    let to = grid_edge(&graph, (2, 2), (1, 2));
    graph.set_resolved_restrictions(vec![ResolvedEdgeRestriction::Simple {
        relation_id: 7,
        restriction: "no_left_turn".into(),
        from_edge_id: from,
        to_edge_id: to,
        via_node_id: 0,
        osm_mode_mask: OsmMode::MOTOR_VEHICLE,
        mobility_mode_mask: MobilityMode::WHEELED,
    }]);
    let reader = persist(&graph, &dir.path().join("turn.mrt"));
    let (oracle, compact) = engines(&reader);
    let origin = GeoPoint::new(GRID_LAT[2], (GRID_LNG[0] + GRID_LNG[1]) / 2.0);
    let destination = GeoPoint::new((GRID_LAT[1] + GRID_LAT[2]) / 2.0, GRID_LNG[2]);
    let banned = route(
        &compact,
        origin,
        destination,
        MobilityProfile::Wheeled,
        |_| {},
    )
    .unwrap();
    assert!(
        !banned.edge_ids.windows(2).any(|pair| pair == [from, to]),
        "the ban holds"
    );
    assert!(banned.sar.is_none());
    for penalty in [1.0, 5.0] {
        let relaxed = route(
            &compact,
            origin,
            destination,
            MobilityProfile::Wheeled,
            |request| {
                request.options.sar = Some(sar_turn_restrictions(penalty));
            },
        )
        .unwrap();
        let reference = route(
            &oracle,
            origin,
            destination,
            MobilityProfile::Wheeled,
            |request| {
                request.options.sar = Some(sar_turn_restrictions(penalty));
            },
        )
        .unwrap();
        assert!((relaxed.cost - reference.cost).abs() < 1e-6 * relaxed.cost.max(1.0));
        assert!(
            relaxed.edge_ids.windows(2).any(|pair| pair == [from, to]),
            "penalty {penalty}: takes the turn"
        );
        assert!(relaxed.cost < banned.cost);
        let report = relaxed.sar.as_ref().unwrap();
        assert_eq!(
            report.enabled,
            vec![crate::SarRelaxationKind::TurnRestrictions]
        );
        assert_eq!(report.uses.len(), 1, "{report:?}");
        let used = &report.uses[0];
        assert_eq!(used.kind, crate::SarRelaxationKind::TurnRestrictions);
        assert_eq!(used.edge_ids, vec![from, to]);
        assert_eq!(used.added_cost_s, penalty);
        assert_eq!(used.legal_tag.as_deref(), Some("type=restriction"));
        assert_eq!(report.totals[0].count, 1);
        assert_eq!(
            relaxed
                .policy_summary
                .sar
                .as_ref()
                .unwrap()
                .turn_restriction_penalty_s,
            Some(penalty)
        );
        // The route without the restriction, plus the penalty.
        let mut free_graph = graph.clone();
        free_graph.set_resolved_restrictions(Vec::new());
        let free = persist(&free_graph, &dir.path().join(format!("free-{penalty}.mrt")));
        let free = RoutingEngine::from_package(free.package.clone());
        let unrestricted =
            route(&free, origin, destination, MobilityProfile::Wheeled, |_| {}).unwrap();
        assert_eq!(unrestricted.edge_ids, relaxed.edge_ids);
        assert!((relaxed.cost - unrestricted.cost - penalty).abs() < 1e-4);
    }
    // A penalty dearer than the detour keeps the detour.
    let detour = route(
        &compact,
        origin,
        destination,
        MobilityProfile::Wheeled,
        |request| {
            request.options.sar = Some(sar_turn_restrictions(10_000.0));
        },
    )
    .unwrap();
    assert_eq!(detour.edge_ids, banned.edge_ids);
    assert!(detour.sar.as_ref().unwrap().uses.is_empty());
}

/// Discounts (multipliers in (0, 1), whole and ranged) and speed overrides
/// (whole and ranged) are costed exactly: routes, matrices and isochrones
/// equal the reference's, and they are cheaper than without.
#[test]
fn discounts_and_speed_overrides_are_costed_exactly() {
    let dir = tempdir().unwrap();
    let graph = grid_graph();
    let reader = persist(&graph, &dir.path().join("discount.mrt"));
    let (oracle, compact) = engines(&reader);
    let configure = |options: &mut crate::RouteOptions| {
        for (from, to, multiplier) in [
            ((0, 1), (0, 2), 0.25),
            ((0, 2), (0, 3), 0.25),
            ((1, 2), (2, 2), 0.6),
            ((2, 2), (1, 2), 0.6),
        ] {
            options
                .edge_cost_multipliers
                .insert(grid_edge(&graph, from, to), multiplier);
        }
        // Overlapping discount ranges multiply.
        for (start, end, multiplier) in [(0.0, 1.0, 0.5), (0.2, 0.9, 0.5)] {
            options
                .edge_cost_multiplier_ranges
                .push(crate::RouteEdgeCostMultiplierRange {
                    edge_id: grid_edge(&graph, (3, 0), (3, 1)),
                    start_fraction: start,
                    end_fraction: end,
                    multiplier,
                });
        }
        options
            .edge_speed_overrides
            .insert(grid_edge(&graph, (4, 0), (4, 1)), 120.0);
        options
            .edge_speed_overrides
            .insert(grid_edge(&graph, (4, 1), (4, 2)), 120.0);
        options
            .edge_speed_override_ranges
            .push(crate::RouteEdgeSpeedOverrideRange {
                edge_id: grid_edge(&graph, (2, 3), (2, 4)),
                start_fraction: 0.0,
                end_fraction: 0.5,
                speed_kph: 100.0,
            });
        // A slow speed override is dearer, and a slow-go factor stays
        // cost-only.
        options
            .edge_speed_overrides
            .insert(grid_edge(&graph, (1, 0), (1, 1)), 5.0);
        options
            .edge_cost_multipliers
            .insert(grid_edge(&graph, (3, 3), (3, 4)), 3.0);
    };
    let points = grid_points();
    let (mut cheaper, mut compared) = (0, 0);
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        for origin in &points {
            for destination in &points {
                if origin == destination {
                    continue;
                }
                let plain = route(&compact, *origin, *destination, profile, |_| {});
                let expected = route(&oracle, *origin, *destination, profile, |request| {
                    configure(&mut request.options)
                });
                let actual = route(&compact, *origin, *destination, profile, |request| {
                    configure(&mut request.options)
                });
                assert_eq!(expected.is_some(), actual.is_some());
                let (Some(expected), Some(actual)) = (expected, actual) else {
                    continue;
                };
                let context = format!("{profile:?} {origin:?} -> {destination:?}");
                assert!(
                    (actual.cost - expected.cost).abs() <= 1e-4 * expected.cost.max(1.0),
                    "{context}: reference {} vs package {}",
                    expected.cost,
                    actual.cost
                );
                assert!(
                    (actual.duration_s - expected.duration_s).abs()
                        <= 1e-3 * expected.duration_s.max(1.0),
                    "{context}: duration reference {} vs package {}",
                    expected.duration_s,
                    actual.duration_s
                );
                // The legs' durations add up to the route's.
                let leg_duration = actual.legs[0]
                    .edges
                    .iter()
                    .map(|edge| edge.duration_s + edge.junction_delay_s)
                    .sum::<f64>();
                assert!(
                    (leg_duration - actual.duration_s).abs() <= 1e-3 * actual.duration_s.max(1.0),
                    "{context}: legs {leg_duration} vs route {}",
                    actual.duration_s
                );
                compared += 1;
                if let Some(plain) = plain
                    && actual.cost < plain.cost - 1e-6
                {
                    cheaper += 1;
                }
            }
        }
        let mut request = crate::RouteMatrixRequest::new(points.clone(), points.clone(), profile);
        request.options.max_correlation_distance_m = 60;
        configure(&mut request.options);
        assert_matrix_parity(&oracle.matrix(request.clone()), &compact.matrix(request));
        let mut request = grid_isochrone_request(profile, isochrone_limits(profile));
        configure(&mut request.options);
        assert_isochrone_parity(
            &oracle.isochrone(request.clone()).unwrap(),
            &compact.isochrone(request).unwrap(),
            &format!("{profile:?} discounts"),
        );
    }
    assert!(compared > 40, "{compared}");
    assert!(cheaper > 10, "{cheaper}");
}

/// An opened edge is usable by a profile its access excludes (the grid's
/// footway, for wheeled), costed by the profile's own model, in routes,
/// matrices and isochrones alike; a block still wins.
#[test]
fn opened_edges_admit_the_profile() {
    let dir = tempdir().unwrap();
    let graph = grid_graph();
    let reader = persist(&graph, &dir.path().join("open.mrt"));
    let (oracle, compact) = engines(&reader);
    let footway = [
        grid_edge(&graph, (1, 3), (2, 3)),
        grid_edge(&graph, (2, 3), (1, 3)),
    ];
    let origin = GeoPoint::new(GRID_LAT[1], (GRID_LNG[3] + GRID_LNG[4]) / 2.0);
    let destination = GeoPoint::new(GRID_LAT[2], (GRID_LNG[2] + GRID_LNG[3]) / 2.0);
    let plain = route(
        &compact,
        origin,
        destination,
        MobilityProfile::Wheeled,
        |_| {},
    )
    .unwrap();
    assert!(!plain.edge_ids.iter().any(|edge| footway.contains(edge)));
    // Opened at 30 km/h and discounted: a footway costs a car far more than
    // its time (the wheeled path factor).
    let open_options = |options: &mut crate::RouteOptions| {
        options.opened_edge_ids.extend(footway);
        options.edge_speed_overrides.insert(footway[0], 30.0);
        options.edge_cost_multipliers.insert(footway[0], 0.02);
    };
    let open = |request: &mut LegRequest| open_options(&mut request.options);
    let opened = route(
        &compact,
        origin,
        destination,
        MobilityProfile::Wheeled,
        open,
    )
    .unwrap();
    let reference = route(&oracle, origin, destination, MobilityProfile::Wheeled, open).unwrap();
    assert!(
        opened.edge_ids.contains(&footway[0]),
        "{:?}",
        opened.edge_ids
    );
    assert_eq!(opened.edge_ids, reference.edge_ids);
    assert!((opened.cost - reference.cost).abs() < 1e-4 * opened.cost);
    assert!((opened.duration_s - reference.duration_s).abs() < 1e-3 * opened.duration_s);
    assert!(opened.cost < plain.cost);
    // An opened footway is reported as an overlay use, not as qualified
    // access of the class.
    assert!(
        opened
            .qualified_access
            .iter()
            .all(|entry| !entry.edge_ids.contains(&footway[0]))
    );
    let blocked = route(
        &compact,
        origin,
        destination,
        MobilityProfile::Wheeled,
        |request| {
            open(request);
            request.options.blocked_edge_ids.insert(footway[0]);
        },
    )
    .unwrap();
    assert!(!blocked.edge_ids.contains(&footway[0]));
    let points = vec![origin, destination, grid_points()[2]];
    let mut request =
        crate::RouteMatrixRequest::new(points.clone(), points, MobilityProfile::Wheeled);
    request.options.max_correlation_distance_m = 60;
    open_options(&mut request.options);
    assert_matrix_parity(&oracle.matrix(request.clone()), &compact.matrix(request));
    let mut request = grid_isochrone_request(MobilityProfile::Wheeled, [20.0, 45.0, 90.0]);
    open_options(&mut request.options);
    let actual = compact.isochrone(request.clone()).unwrap();
    assert_isochrone_parity(&oracle.isochrone(request).unwrap(), &actual, "opened");
    assert!(
        actual
            .buckets
            .iter()
            .any(|bucket| bucket.edge_ids.contains(&footway[0]))
    );
}

/// R6: an active conditional closure (`access=no @ (Mo-Fr 07:00-09:00)` on
/// the costing grid) closes its edges at a time inside the window; the SAR
/// `conditional` relaxation opens them again at their unconditional cost,
/// reports each closure it ignored, and matches the reference.
#[test]
fn sar_ignores_conditional_closures_and_reports_them() {
    let dir = tempdir().unwrap();
    let graph = costing_grid_graph();
    let reader = persist(&graph, &dir.path().join("conditional.mrt"));
    let (oracle, compact) = engines(&reader);
    // Tuesday 2026-10-06, 08:00 UTC: inside the window.
    let closed_at = utc_ms(6, 8);
    let conditional = |options: &mut crate::RouteOptions| {
        let mut sar = crate::RouteSarOptions::default();
        sar.relaxations.conditional = Some(crate::sar::SarConditional { ignore: true });
        options.sar = Some(sar);
    };
    let points = grid_points();
    let (mut compared, mut reported) = (0, 0);
    for origin in &points {
        for destination in &points {
            if origin == destination {
                continue;
            }
            let at = |options: &mut crate::RouteOptions| {
                options.costing.conditional_restriction_eval_at_ms = Some(closed_at);
            };
            let open = route(
                &compact,
                *origin,
                *destination,
                MobilityProfile::Wheeled,
                |_| {},
            );
            let closed = route(
                &compact,
                *origin,
                *destination,
                MobilityProfile::Wheeled,
                |r| at(&mut r.options),
            );
            let relaxed = route(
                &compact,
                *origin,
                *destination,
                MobilityProfile::Wheeled,
                |r| {
                    at(&mut r.options);
                    conditional(&mut r.options);
                },
            );
            let reference = route(
                &oracle,
                *origin,
                *destination,
                MobilityProfile::Wheeled,
                |r| {
                    at(&mut r.options);
                    conditional(&mut r.options);
                },
            );
            assert_eq!(relaxed.is_some(), reference.is_some());
            let (Some(relaxed), Some(reference)) = (relaxed, reference) else {
                continue;
            };
            assert!((relaxed.cost - reference.cost).abs() <= 1e-4 * relaxed.cost.max(1.0));
            // Ignoring the closures routes as at a time outside them.
            let open = open.expect("routes without the closure");
            assert!((relaxed.cost - open.cost).abs() <= 1e-4 * open.cost.max(1.0));
            if let Some(closed) = closed {
                assert!(relaxed.cost <= closed.cost + 1e-6 * closed.cost.max(1.0));
            }
            let report = relaxed.sar.as_ref().unwrap();
            for used in &report.uses {
                assert_eq!(used.kind, crate::SarRelaxationKind::Conditional);
                assert_eq!(
                    used.legal_tag.as_deref(),
                    Some("access=no @ (Mo-Fr 07:00-09:00)")
                );
                assert!(used.length_m > 0.0);
                reported += 1;
            }
            compared += 1;
        }
    }
    assert!(compared > 10, "{compared}");
    assert!(reported > 0, "no route crossed a closure");
}

/// An 8 x 8 grid of residential streets, 200 m apart: large enough for the
/// widened correlation search, which only keeps candidates in components of
/// at least 50 nodes.
fn large_grid_graph() -> RoadGraph {
    let node = |row: usize, col: usize| 5_000 + (row * 10 + col) as i64;
    let at = |row: usize, col: usize| (row as f64 * 0.0018, col as f64 * 0.0018);
    let mut builder = RoadGraphBuilder::new("large-grid");
    let mut way_id = 1;
    for row in 0..8 {
        for col in 0..8 {
            for (next_row, next_col) in [(row, col + 1), (row + 1, col)] {
                if next_row < 8 && next_col < 8 {
                    builder.push_way(way(
                        way_id,
                        at(row, col),
                        at(next_row, next_col),
                        node(row, col),
                        node(next_row, next_col),
                        RoadType::Residential,
                        DirectionalAccess::Both,
                    ));
                    way_id += 1;
                }
            }
        }
    }
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    graph
}

/// R7: an endpoint far off the road network has no candidate; with the
/// `off_network` relaxation it snaps up to `max_leg_m` away, and the
/// straight-line leg is reported with its off-road time.
#[test]
fn sar_off_network_legs_reach_far_endpoints_and_are_reported() {
    let dir = tempdir().unwrap();
    let reader = persist(&large_grid_graph(), &dir.path().join("off-network.mrt"));
    let (oracle, compact) = engines(&reader);
    // About 1.4 km south of the grid's south-west corner.
    let far = GeoPoint::new(-0.0125, 0.0);
    let destination = GeoPoint::new(0.0063, 0.0072);
    let error = compact
        .route_leg(LegRequest::new(far, destination, MobilityProfile::Wheeled))
        .unwrap_err()
        .error;
    assert!(error.is_no_candidate(), "{error:?}");
    let off_network = |request: &mut LegRequest| {
        let mut sar = crate::RouteSarOptions::default();
        sar.relaxations.off_network = Some(crate::sar::SarOffNetwork { max_leg_m: 2_000.0 });
        request.options.sar = Some(sar);
    };
    let mut request = LegRequest::new(far, destination, MobilityProfile::Wheeled);
    off_network(&mut request);
    let routed = compact.route_leg(request.clone()).unwrap();
    let reference = oracle.route_leg(request).unwrap();
    assert!((routed.cost - reference.cost).abs() <= 1e-4 * routed.cost);
    let report = routed.sar.as_ref().unwrap();
    assert_eq!(report.uses.len(), 1, "{report:?}");
    let leg = &report.uses[0];
    assert_eq!(leg.kind, crate::SarRelaxationKind::OffNetwork);
    assert!(
        (1_300.0..1_500.0).contains(&leg.length_m),
        "{}",
        leg.length_m
    );
    assert_eq!(leg.geometry[0], far);
    assert_eq!(leg.geometry[1], routed.origin.snapped_point);
    let pace = wayfarer_costing::profile_off_road_speed_mps(MobilityProfile::Wheeled);
    assert!((leg.added_cost_s - leg.length_m / pace).abs() < 1e-6);
    assert_eq!(report.totals[0].kind, crate::SarRelaxationKind::OffNetwork);
    assert_eq!(reference.sar.as_ref().unwrap().uses, report.uses);
    // A reach shorter than the leg still fails.
    let mut request = LegRequest::new(far, destination, MobilityProfile::Wheeled);
    let mut sar = crate::RouteSarOptions::default();
    sar.relaxations.off_network = Some(crate::sar::SarOffNetwork { max_leg_m: 1_200.0 });
    request.options.sar = Some(sar);
    assert!(
        compact
            .route_leg(request)
            .unwrap_err()
            .error
            .is_no_candidate()
    );
    // Matrices and isochrones correlate the same way.
    let mut matrix =
        crate::RouteMatrixRequest::new(vec![far], vec![destination], MobilityProfile::Wheeled);
    let plain_cell = compact.matrix(matrix.clone()).cells[0].clone();
    assert_eq!(plain_cell.status, crate::RouteMatrixCellStatus::NoCandidate);
    let mut sar = crate::RouteSarOptions::default();
    sar.relaxations.off_network = Some(crate::sar::SarOffNetwork::default());
    matrix.options.sar = Some(sar.clone());
    let cell = compact.matrix(matrix).cells[0].clone();
    assert_eq!(cell.status, crate::RouteMatrixCellStatus::Reachable);
    assert!((cell.cost.unwrap() - routed.cost).abs() <= 1e-4 * routed.cost);
    let mut isochrone = grid_isochrone_request(MobilityProfile::Wheeled, [20.0, 45.0, 90.0]);
    isochrone.origin = far;
    isochrone.options.max_correlation_distance_m = 75;
    assert!(compact.isochrone(isochrone.clone()).is_err());
    isochrone.options.sar = Some(sar);
    assert!(compact.isochrone(isochrone).is_ok());
}

/// The package carries SAR data, path and width classes included: every
/// relaxation is available at the package's minimum factors. SAR needs a
/// vehicle profile, and `vehicle_on_paths` the Wheeled one: elsewhere it
/// fails with `SarUnavailable` (routes, matrices, isochrones). A surcharge
/// below the package minimum is an invalid request.
#[test]
fn sar_relaxations_fail_typed() {
    use crate::SarRelaxationKind as Kind;
    let dir = tempdir().unwrap();
    let reader = persist(&grid_graph(), &dir.path().join("sar-errors.mrt"));
    let compact = RoutingEngine::from_package(reader.package.clone());
    let capability = compact.package().sar_capability();
    assert!(capability.unavailable.is_empty());
    assert_eq!(capability.available, Kind::ALL.to_vec());
    assert_eq!(capability.reason, None);
    assert_eq!(capability.legal_access_min_factor, Some(1.2));
    assert_eq!(capability.wrong_way_min_factor, Some(2.0));
    assert_eq!(capability.vehicle_on_paths_min_factor, Some(3.0));
    let points = grid_points();
    let kind = Kind::VehicleOnPaths;
    let mut sar = sar_turn_restrictions(60.0);
    sar.relaxations.vehicle_on_paths = Some(Default::default());
    let mut request = LegRequest::new(points[0], points[1], MobilityProfile::Emergency);
    request.options.sar = Some(sar.clone());
    assert_eq!(
        compact.route_leg(request).unwrap_err().error,
        crate::RouteError::SarUnavailable { relaxation: kind }
    );
    let mut matrix =
        crate::RouteMatrixRequest::new(points.clone(), points.clone(), MobilityProfile::Emergency);
    matrix.options.sar = Some(sar.clone());
    let cells = compact.matrix(matrix).cells;
    assert!(cells.iter().all(|cell| {
        cell.status == crate::RouteMatrixCellStatus::Failed
            && cell.failure_reason.as_deref() == Some(&format!("sar_unavailable:{kind}"))
    }));
    let mut isochrone = grid_isochrone_request(MobilityProfile::Emergency, [20.0, 45.0, 90.0]);
    isochrone.options.sar = Some(sar);
    assert_eq!(
        compact.isochrone(isochrone).unwrap_err(),
        crate::RouteError::SarUnavailable { relaxation: kind }
    );
    // Surcharges below the package minimum, and R1 values that deny nothing,
    // are invalid requests.
    let below = |configure: fn(&mut crate::RouteSarRelaxations)| {
        let mut sar = crate::RouteSarOptions::default();
        configure(&mut sar.relaxations);
        let mut request = LegRequest::new(points[0], points[1], MobilityProfile::Wheeled);
        request.options.sar = Some(sar);
        compact.route_leg(request).unwrap_err().error
    };
    let minimum = crate::RouteError::InvalidRequest {
        reason: "sar surcharge below package minimum",
    };
    assert_eq!(
        below(|r| {
            r.legal_access = Some(crate::sar::SarLegalAccess {
                factor: 1.1,
                ..Default::default()
            })
        }),
        minimum
    );
    assert_eq!(
        below(|r| {
            r.wrong_way = Some(crate::sar::SarWrongWay {
                factor: 1.5,
                entry_s: 30.0,
            })
        }),
        minimum
    );
    assert_eq!(
        below(|r| {
            r.barriers = Some(crate::sar::SarBarriers {
                gate_s: -1.0,
                ..Default::default()
            })
        }),
        minimum
    );
    assert_eq!(
        below(|r| {
            r.vehicle_on_paths = Some(crate::sar::SarVehicleOnPaths {
                factor: 2.9,
                ..Default::default()
            })
        }),
        minimum
    );
    assert_eq!(
        below(|r| {
            r.vehicle_on_paths = Some(crate::sar::SarVehicleOnPaths {
                classes: vec!["footway".into(), "steps".into()],
                ..Default::default()
            })
        }),
        crate::RouteError::InvalidRequest {
            reason: "sar vehicle_on_paths class is not a drivable path class"
        }
    );
    assert_eq!(
        below(|r| {
            r.vehicle_on_paths = Some(crate::sar::SarVehicleOnPaths {
                min_width_m: f64::NAN,
                ..Default::default()
            })
        }),
        crate::RouteError::InvalidRequest {
            reason: "sar vehicle_on_paths min_width_m"
        }
    );
    assert_eq!(
        below(|r| {
            r.legal_access = Some(crate::sar::SarLegalAccess {
                values: vec!["permissive".into()],
                factor: 1.2,
            })
        }),
        crate::RouteError::InvalidRequest {
            reason: "sar legal_access value is not a denying access value"
        }
    );
    let invalid = |configure: fn(&mut LegRequest)| {
        let mut request = LegRequest::new(points[0], points[1], MobilityProfile::Wheeled);
        configure(&mut request);
        compact.route_leg(request).unwrap_err().error
    };
    assert_eq!(
        invalid(|request| request.options.sar = Some(sar_turn_restrictions(-1.0))),
        crate::RouteError::InvalidRequest {
            reason: "sar surcharge below package minimum"
        }
    );
    assert_eq!(
        invalid(|request| {
            request.profile = MobilityProfile::OnFoot;
            request.options.sar = Some(sar_turn_restrictions(60.0));
        }),
        crate::RouteError::InvalidRequest {
            reason: "sar requires a vehicle profile"
        }
    );
    for profile in [MobilityProfile::Emergency, MobilityProfile::Tracked] {
        let mut request = LegRequest::new(points[0], points[1], profile);
        request.options.sar = Some(sar_turn_restrictions(60.0));
        assert!(compact.route_leg(request.clone()).is_ok(), "{profile:?}");
        request.options.sar = Some(crate::RouteSarOptions {
            relaxations: crate::RouteSarRelaxations {
                vehicle_on_paths: Some(Default::default()),
                ..Default::default()
            },
        });
        assert_eq!(
            compact.route_leg(request).unwrap_err().error,
            crate::RouteError::SarUnavailable {
                relaxation: Kind::VehicleOnPaths
            },
            "{profile:?}"
        );
    }
    // A request without SAR, or with an empty SAR option, routes as before.
    let plain = route(
        &compact,
        points[0],
        points[1],
        MobilityProfile::Wheeled,
        |_| {},
    )
    .unwrap();
    let empty = route(
        &compact,
        points[0],
        points[1],
        MobilityProfile::Wheeled,
        |request| {
            request.options.sar = Some(crate::RouteSarOptions::default());
        },
    )
    .unwrap();
    assert_eq!(plain.edge_ids, empty.edge_ids);
    assert_eq!(plain.cost.to_bits(), empty.cost.to_bits());
    assert!(plain.sar.is_none());
    assert_eq!(empty.sar.as_ref().unwrap().uses, Vec::new());
}

/// SAR options parse from snake_case and camelCase alike, with the
/// defaults for every field left out.
#[test]
fn sar_options_parse_with_aliases_and_defaults() {
    let snake: crate::RouteOptions = serde_json::from_str(
        r#"{"sar": {"relaxations": {"turn_restrictions": {}, "off_network": {"max_leg_m": 900},
            "conditional": {}}}}"#,
    )
    .unwrap();
    let camel: crate::RouteOptions = serde_json::from_str(
        r#"{"sar": {"relaxations": {"turnRestrictions": {}, "offNetwork": {"maxLegM": 900},
            "conditional": {"ignore": true}}}}"#,
    )
    .unwrap();
    assert_eq!(snake, camel);
    let sar = snake.sar.unwrap();
    assert_eq!(sar.turn_restriction_penalty_s(), Some(60.0));
    assert_eq!(sar.off_network_max_leg_m(), Some(900.0));
    assert!(sar.ignores_conditional());
    let legal: crate::RouteSarOptions =
        serde_json::from_str(r#"{"relaxations": {"legal_access": {}, "wrong_way": {}}}"#).unwrap();
    let legal_access = legal.relaxations.legal_access.unwrap();
    assert_eq!(legal_access.factor, 1.2);
    assert!(!legal_access.values.iter().any(|value| value == "military"));
    assert_eq!(legal.relaxations.wrong_way.unwrap().entry_s, 30.0);
    let overrides: crate::RouteOptions = serde_json::from_str(
        r#"{"edgeSpeedOverrides": {"5": 30}, "openedEdgeIds": [7],
            "edgeSpeedOverrideRanges": [{"edgeId": 5, "startFraction": 0, "endFraction": 0.5, "speedKph": 20}]}"#,
    )
    .unwrap();
    assert_eq!(overrides.edge_speed_overrides[&5], 30.0);
    assert!(overrides.opened_edge_ids.contains(&7));
    assert_eq!(overrides.edge_speed_override_ranges[0].speed_kph, 20.0);
    // Options without the new fields serialize as before.
    let text = serde_json::to_string(&crate::RouteOptions::default()).unwrap();
    assert!(!text.contains("sar") && !text.contains("SpeedOverride") && !text.contains("opened"));
}

// ---------------------------------------------------------------------------
// SAR stage 1: the physical graph (R1 `legal_access`, R2 `wrong_way`) and
// barrier nodes (R4 `barriers`).
// ---------------------------------------------------------------------------

/// Grid node `(row, col)`'s OSM id in [`sar_grid_graph`].
fn sar_grid_node(row: usize, col: usize) -> i64 {
    2_000 + (row * 10 + col) as i64
}

/// A 5x5 grid whose row 0 is one way through every node (so a barrier on it
/// sits where that way passes through), with ways the law closes to
/// vehicles, one-ways walkers use both ways, and barrier nodes. Without
/// `sar_ways` the ways closed to every mode are left out.
fn sar_grid_graph(sar_ways: bool) -> RoadGraph {
    use wayfarer_road_graph::NodeBarrierTags;
    let (lat, lng) = (GRID_LAT, GRID_LNG);
    let node = sar_grid_node;
    let mut builder = RoadGraphBuilder::new("sar-grid");
    let mut barriers = std::collections::HashMap::new();
    let barrier = |value: &str, access: Option<&str>| NodeBarrierTags {
        barrier: value.into(),
        access: access.map(Into::into),
        ..Default::default()
    };
    barriers.insert(node(0, 2), barrier("gate", None));
    barriers.insert(node(2, 3), barrier("gate", Some("no")));
    barriers.insert(node(4, 3), barrier("bollard", None));
    barriers.insert(node(1, 0), barrier("wicket_gate", None));
    builder.set_barrier_node_tags(barriers);
    // Row 0: one way through all five nodes.
    builder.push_way(RoutableWay {
        osm_way_id: 100,
        points: (0..5).map(|col| (lat[0], lng[col])).collect(),
        source_node_ids: (0..5).map(|col| Some(node(0, col))).collect(),
        road_type: RoadType::Residential,
        highway: Some("residential".into()),
        ..Default::default()
    });
    let mut way_id = 200;
    for row in 0..5 {
        for col in 0..5 {
            let mut ways = Vec::new();
            if col + 1 < 5 && row > 0 {
                let mut horizontal = way(
                    way_id,
                    (lat[row], lng[col]),
                    (lat[row], lng[col + 1]),
                    node(row, col),
                    node(row, col + 1),
                    if row == 2 {
                        RoadType::Primary
                    } else {
                        RoadType::Residential
                    },
                    DirectionalAccess::Both,
                );
                match (row, col) {
                    (1, 1) => {
                        horizontal.highway = Some("service".into());
                        horizontal.access = Some("no".into());
                        horizontal.access_restricted = true;
                    }
                    (4, 1) => {
                        horizontal.access = Some("military".into());
                        horizontal.access_restricted = true;
                    }
                    (3, 2) => {
                        horizontal.highway = Some("track".into());
                        horizontal.road_type = RoadType::Track;
                        horizontal.motor_vehicle = Some("forestry".into());
                    }
                    (3, 1) => horizontal.directional_access = DirectionalAccess::Forward,
                    (1, 3) => horizontal.directional_access = DirectionalAccess::Backward,
                    _ => {}
                }
                ways.push(horizontal);
                way_id += 1;
            }
            if row + 1 < 5 {
                let mut vertical = way(
                    way_id,
                    (lat[row], lng[col]),
                    (lat[row + 1], lng[col]),
                    node(row, col),
                    node(row + 1, col),
                    if col == 3 && row == 1 {
                        RoadType::Path
                    } else if col == 2 {
                        RoadType::Secondary
                    } else {
                        RoadType::Residential
                    },
                    DirectionalAccess::Both,
                );
                if (row, col) == (2, 4) {
                    vertical.access = Some("agricultural".into());
                    vertical.access_restricted = true;
                }
                ways.push(vertical);
                way_id += 1;
            }
            for way in ways {
                let closed_to_all = matches!(
                    way.access.as_deref(),
                    Some("no" | "military" | "agricultural")
                );
                if sar_ways || !closed_to_all {
                    builder.push_way(way);
                }
            }
        }
    }
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    graph
}

fn sar_grid_points() -> Vec<GeoPoint> {
    let (lat, lng) = (GRID_LAT, GRID_LNG);
    let mid = |a: f64, b: f64| (a + b) / 2.0;
    vec![
        GeoPoint::new(0.00005, mid(lng[0], lng[1])),
        GeoPoint::new(0.00005, mid(lng[3], lng[4])),
        GeoPoint::new(lat[4] - 0.00003, mid(lng[3], lng[4])),
        GeoPoint::new(mid(lat[2], lat[3]), lng[4] + 0.00003),
        GeoPoint::new(lat[2] + 0.00003, mid(lng[0], lng[1])),
        GeoPoint::new(mid(lat[3], lat[4]), lng[0] + 0.00003),
        // On the closed road, the military road and the forestry track.
        GeoPoint::new(lat[1] + 0.00002, mid(lng[1], lng[2])),
        GeoPoint::new(lat[4] - 0.00002, mid(lng[1], lng[2])),
        GeoPoint::new(lat[3] + 0.00002, mid(lng[2], lng[3])),
        // Both ends of the one-way at (3, 1).
        GeoPoint::new(lat[3] + 0.00002, lng[1] + 0.0002),
        GeoPoint::new(lat[3] + 0.00002, lng[2] - 0.0002),
    ]
}

fn sar_option(json: &str) -> crate::RouteSarOptions {
    serde_json::from_str(json).unwrap()
}

/// The SAR option sets the stage 1 tests run.
fn sar_stage1_sets() -> Vec<(&'static str, crate::RouteSarOptions)> {
    [
        ("r1", r#"{"relaxations":{"legal_access":{}}}"#),
        (
            "r1-military-x2",
            r#"{"relaxations":{"legal_access":{"values":["private","no","customers","military","forestry","agricultural"],"factor":2.0}}}"#,
        ),
        ("r2", r#"{"relaxations":{"wrong_way":{}}}"#),
        (
            "r2-x3-no-entry",
            r#"{"relaxations":{"wrong_way":{"factor":3.0,"entry_s":0}}}"#,
        ),
        ("r4", r#"{"relaxations":{"barriers":{}}}"#),
        (
            "r1-r2-r4",
            r#"{"relaxations":{"legal_access":{},"wrong_way":{},"barriers":{"gate_s":20,"locked_gate_s":45}}}"#,
        ),
        (
            "r1-r2-r3-r4",
            r#"{"relaxations":{"legal_access":{},"wrong_way":{},"turn_restrictions":{},"barriers":{}}}"#,
        ),
    ]
    .into_iter()
    .map(|(name, json)| (name, sar_option(json)))
    .collect()
}

/// R1, R2 and R4 against the reference oracle: routes (bidirectional and
/// forward only), matrices in every mode and isochrones have its costs.
#[test]
fn sar_physical_relaxations_match_the_reference() {
    use super::search::patch::dijkstra;
    use crate::matrix::{MatrixMode, test_mode};
    let dir = tempdir().unwrap();
    let reader = persist(&sar_grid_graph(true), &dir.path().join("sar1.mrt"));
    let (oracle, compact) = engines(&reader);
    let points = sar_grid_points();
    let mut compared = 0;
    let mut used = std::collections::BTreeMap::<crate::SarRelaxationKind, usize>::new();
    for (name, sar) in sar_stage1_sets() {
        let configure = |request: &mut LegRequest| request.options.sar = Some(sar.clone());
        for origin in &points {
            for destination in &points {
                if origin == destination {
                    continue;
                }
                let context = format!("{name} {origin:?} -> {destination:?}");
                let expected = route(
                    &oracle,
                    *origin,
                    *destination,
                    MobilityProfile::Wheeled,
                    configure,
                );
                for forward_only in [false, true] {
                    dijkstra::force(forward_only);
                    let actual = route(
                        &compact,
                        *origin,
                        *destination,
                        MobilityProfile::Wheeled,
                        configure,
                    );
                    dijkstra::force(false);
                    assert_eq!(expected.is_some(), actual.is_some(), "{context}");
                    let (Some(expected), Some(actual)) = (&expected, actual) else {
                        continue;
                    };
                    assert!(
                        (actual.cost - expected.cost).abs() <= 1e-4 * expected.cost.max(1.0),
                        "{context} forward only {forward_only}: reference {} vs package {}",
                        expected.cost,
                        actual.cost
                    );
                    let report = actual.sar.as_ref().expect("a SAR report");
                    for used_relaxation in &report.uses {
                        assert!(
                            report.enabled.contains(&used_relaxation.kind),
                            "{context}: {used_relaxation:?}"
                        );
                        *used.entry(used_relaxation.kind).or_default() += 1;
                    }
                    compared += 1;
                }
            }
        }
        let mut request = crate::RouteMatrixRequest::new(
            points.clone(),
            points.clone(),
            MobilityProfile::Wheeled,
        );
        request.options.max_correlation_distance_m = 60;
        request.options.sar = Some(sar.clone());
        let reference = oracle.matrix(request.clone());
        for mode in [MatrixMode::Pairwise, MatrixMode::Rows] {
            test_mode::set(Some(mode));
            let actual = compact.matrix(request.clone());
            test_mode::set(None);
            assert_matrix_parity(&reference, &actual);
        }
        for origin in points.iter().take(6) {
            let mut request = grid_isochrone_request(MobilityProfile::Wheeled, [20.0, 45.0, 90.0]);
            request.origin = *origin;
            request.options.sar = Some(sar.clone());
            let reference = oracle.isochrone(request.clone()).unwrap();
            let actual = compact.isochrone(request).unwrap();
            assert_isochrone_parity(&reference, &actual, &format!("{name} {origin:?}"));
        }
    }
    eprintln!("SAR stage 1 parity: {compared} routes, uses {used:?}");
    for kind in [
        crate::SarRelaxationKind::LegalAccess,
        crate::SarRelaxationKind::WrongWay,
        crate::SarRelaxationKind::Barriers,
    ] {
        assert!(
            used.get(&kind).copied().unwrap_or(0) > 5,
            "{kind}: {used:?}"
        );
    }
}

/// Exact node-level distances over `costs` to `targets` (`reversed`: from
/// them), without turn costs: what a potential may not exceed.
fn exact_node_distances(
    g: &SearchGraph<'_>,
    costs: &[f32],
    targets: &[u32],
    reversed: bool,
) -> Vec<f64> {
    use std::cmp::Reverse;
    struct Key(f64);
    impl PartialEq for Key {
        fn eq(&self, other: &Self) -> bool {
            self.cmp(other).is_eq()
        }
    }
    impl Eq for Key {}
    impl PartialOrd for Key {
        fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
            Some(self.cmp(other))
        }
    }
    impl Ord for Key {
        fn cmp(&self, other: &Self) -> std::cmp::Ordering {
            self.0.total_cmp(&other.0)
        }
    }
    let mut dist = vec![f64::INFINITY; g.node_count];
    let mut heap = std::collections::BinaryHeap::new();
    for target in targets {
        dist[*target as usize] = 0.0;
        heap.push(Reverse((Key(0.0), *target)));
    }
    while let Some(Reverse((Key(d), node))) = heap.pop() {
        if d > dist[node as usize] {
            continue;
        }
        // Towards the targets: relax the edges into `node`; from them, out.
        let edges: Vec<u32> = if reversed {
            (g.first_out[node as usize]..g.first_out[node as usize + 1]).collect()
        } else {
            g.in_edges(node as usize).map(|(edge, _)| edge).collect()
        };
        for edge in edges {
            let cost = f64::from(costs[edge as usize]);
            if !cost.is_finite() {
                continue;
            }
            let next = if reversed {
                g.edge_target[edge as usize]
            } else {
                g.edge_source(edge as usize)
            };
            if d + cost < dist[next as usize] {
                dist[next as usize] = d + cost;
                heap.push(Reverse((Key(d + cost), next)));
            }
        }
    }
    dist
}

/// Emergency and Tracked columns (plain, at override strength 1, the strict
/// destination-only pass, and with SAR R1 and R2) get potentials of their own
/// metric where the stored column's would be scaled below 0.9, and so do
/// Wheeled R5 columns, which the stored column cannot bound. Those
/// potentials are consistent on every edge the column can use and never
/// exceed the exact distance, towards and from every target set.
#[test]
fn own_metric_potentials_are_consistent_and_admissible() {
    let dir = tempdir().unwrap();
    let override_one = crate::RouteCostingOptions {
        emergency_override_strength: 1.0,
        ..Default::default()
    };
    let sar = sar_option(
        r#"{"relaxations":{"legal_access":{"values":["private","no","customers","military","forestry","agricultural"]},"wrong_way":{}}}"#,
    );
    let mut adopted = Vec::new();
    let paths = sar_stage2_sets();
    for (name, graph) in [
        ("costing", costing_grid_graph()),
        ("sar", sar_grid_graph(true)),
        ("paths", sar_path_grid_graph(true)),
    ] {
        let reader = persist(&graph, &dir.path().join(format!("{name}.mrt")));
        let package = &reader.package;
        let g = package.view();
        let nodes = (0..g.node_count as u32).collect::<Vec<_>>();
        let cases: Vec<(
            MobilityProfile,
            crate::RouteCostingOptions,
            bool,
            Option<&crate::RouteSarOptions>,
        )> = if name == "paths" {
            paths
                .iter()
                .map(|(_, sar)| {
                    (
                        MobilityProfile::Wheeled,
                        Default::default(),
                        true,
                        Some(sar),
                    )
                })
                .collect()
        } else {
            vec![
                (MobilityProfile::Emergency, Default::default(), true, None),
                (MobilityProfile::Emergency, override_one, true, None),
                (MobilityProfile::Emergency, Default::default(), false, None),
                (MobilityProfile::Tracked, Default::default(), true, None),
                (
                    MobilityProfile::Emergency,
                    Default::default(),
                    true,
                    Some(&sar),
                ),
                (
                    MobilityProfile::Tracked,
                    Default::default(),
                    true,
                    Some(&sar),
                ),
            ]
        };
        for (profile, costing, allow_destination_only, with_sar) in cases {
            let context = format!(
                "{name} {profile:?} {costing:?} {allow_destination_only} {:?}",
                with_sar.map(|sar| &sar.relaxations)
            );
            let relaxations = with_sar.map(|sar| {
                let options = crate::RouteOptions {
                    sar: Some(sar.clone()),
                    ..Default::default()
                };
                crate::sar::edge_relaxations(&options, profile).expect("relaxations")
            });
            let handle =
                package.request_column_sar(profile, &costing, allow_destination_only, relaxations);
            let columns::ColumnHandle::Derived(derived) = &handle else {
                panic!("{context}: a derived column");
            };
            let Some((base, factor)) = derived.potential else {
                panic!("{context}: no potentials");
            };
            if !matches!(base, columns::PotentialBase::Metric(_)) {
                // The stored column bounds it well enough as it is.
                assert!(factor >= 0.9, "{context}: factor {factor}");
                continue;
            }
            assert!(factor > 0.9999, "{context}: factor {factor}");
            adopted.push(context.clone());
            let column = handle.column(&g);
            let none = rustc_hash::FxHashMap::default();
            let plan = search::potential_plan(g, &column, &none, false).expect("plan");
            let costs = column
                .cost
                .iter()
                .map(|cost| f64::from(*cost))
                .collect::<Vec<_>>();
            let scratch = <[std::cell::RefCell<cch::PotentialScratch>; 2]>::default();
            let mut checked = 0;
            for (targets, reversed) in nodes
                .chunks(4)
                .flat_map(|targets| [(targets, false), (targets, true)])
            {
                let potential = plan.potential(targets, reversed, &[], &scratch[0], &scratch[1]);
                let bound = |node: u32| potential.get(node);
                let at = format!("{context} {targets:?}");
                checked += assert_consistent(&g, &costs, reversed, bound, &at);
                let exact = exact_node_distances(&g, column.cost, targets, reversed);
                for node in &nodes {
                    let (bound, exact) = (bound(*node), exact[*node as usize]);
                    assert!(
                        bound <= exact + 1e-6 * exact.max(1.0),
                        "{context} node {node} {targets:?} {reversed}: bound {bound} > exact {exact}"
                    );
                }
            }
            assert!(checked > 500, "{context}: {checked}");
        }
    }
    eprintln!("own metrics: {adopted:#?}");
    assert!(adopted.len() >= 16, "{adopted:#?}");
    assert_eq!(
        adopted
            .iter()
            .filter(|context| context.starts_with("paths"))
            .count(),
        paths.len(),
        "{adopted:#?}"
    );
}

/// SAR on Emergency and Tracked (R1, R2 and R4, alone and combined, with R3)
/// against the reference oracle: routes (bidirectional and forward only),
/// matrices in every mode and isochrones have its costs, bounded by the
/// potentials of the profiles' own metrics.
#[test]
fn sar_on_emergency_and_tracked_matches_the_reference() {
    use super::search::patch::dijkstra;
    use crate::matrix::{MatrixMode, test_mode};
    let dir = tempdir().unwrap();
    let reader = persist(&sar_grid_graph(true), &dir.path().join("sar3.mrt"));
    let (oracle, compact) = engines(&reader);
    let points = sar_grid_points();
    let mut compared = 0;
    let mut used = std::collections::BTreeMap::<crate::SarRelaxationKind, usize>::new();
    for profile in [MobilityProfile::Emergency, MobilityProfile::Tracked] {
        for (name, sar) in sar_stage1_sets() {
            let configure = |request: &mut LegRequest| request.options.sar = Some(sar.clone());
            for origin in &points {
                for destination in &points {
                    if origin == destination {
                        continue;
                    }
                    let context = format!("{profile:?} {name} {origin:?} -> {destination:?}");
                    let expected = route(&oracle, *origin, *destination, profile, configure);
                    for forward_only in [false, true] {
                        dijkstra::force(forward_only);
                        let actual = route(&compact, *origin, *destination, profile, configure);
                        dijkstra::force(false);
                        assert_eq!(expected.is_some(), actual.is_some(), "{context}");
                        let (Some(expected), Some(actual)) = (&expected, actual) else {
                            continue;
                        };
                        assert!(
                            (actual.cost - expected.cost).abs() <= 1e-4 * expected.cost.max(1.0),
                            "{context} forward only {forward_only}: reference {} vs package {}",
                            expected.cost,
                            actual.cost
                        );
                        let report = actual.sar.as_ref().expect("a SAR report");
                        for used_relaxation in &report.uses {
                            *used.entry(used_relaxation.kind).or_default() += 1;
                        }
                        compared += 1;
                    }
                }
            }
            let mut request =
                crate::RouteMatrixRequest::new(points.clone(), points.clone(), profile);
            request.options.max_correlation_distance_m = 60;
            request.options.sar = Some(sar.clone());
            let reference = oracle.matrix(request.clone());
            for mode in [MatrixMode::Pairwise, MatrixMode::Rows] {
                test_mode::set(Some(mode));
                let actual = compact.matrix(request.clone());
                test_mode::set(None);
                assert_matrix_parity(&reference, &actual);
            }
            for origin in points.iter().take(4) {
                let mut request = grid_isochrone_request(profile, [20.0, 45.0, 90.0]);
                request.origin = *origin;
                request.options.sar = Some(sar.clone());
                let reference = oracle.isochrone(request.clone()).unwrap();
                let actual = compact.isochrone(request).unwrap();
                assert_isochrone_parity(
                    &reference,
                    &actual,
                    &format!("{profile:?} {name} {origin:?}"),
                );
            }
        }
    }
    eprintln!("SAR stage 3 parity: {compared} routes, uses {used:?}");
    for kind in [
        crate::SarRelaxationKind::LegalAccess,
        crate::SarRelaxationKind::WrongWay,
        crate::SarRelaxationKind::Barriers,
    ] {
        assert!(
            used.get(&kind).copied().unwrap_or(0) > 5,
            "{kind}: {used:?}"
        );
    }
}

/// Normal requests never use, nor snap onto, the roads kept for SAR: on
/// the grid with them their routes cost exactly what they cost on the grid
/// without them; matrices and isochrones too.
#[test]
fn roads_kept_for_sar_leave_normal_requests_unchanged() {
    let dir = tempdir().unwrap();
    let (with_graph, without_graph) = (sar_grid_graph(true), sar_grid_graph(false));
    let with = persist(&with_graph, &dir.path().join("with.mrt"));
    let without = persist(&without_graph, &dir.path().join("without.mrt"));
    let (with, without) = (
        RoutingEngine::from_package(with.package.clone()),
        RoutingEngine::from_package(without.package.clone()),
    );
    let points = sar_grid_points();
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        for origin in &points {
            for destination in &points {
                if origin == destination {
                    continue;
                }
                let context = format!("{profile:?} {origin:?} -> {destination:?}");
                let left = route(&with, *origin, *destination, profile, |_| {});
                let right = route(&without, *origin, *destination, profile, |_| {});
                assert_eq!(left.is_some(), right.is_some(), "{context}");
                let (Some(left), Some(right)) = (left, right) else {
                    continue;
                };
                assert_eq!(left.cost.to_bits(), right.cost.to_bits(), "{context}");
                assert_eq!(
                    left.distance_m.to_bits(),
                    right.distance_m.to_bits(),
                    "{context}"
                );
                assert_eq!(
                    left.origin.snapped_point, right.origin.snapped_point,
                    "{context}"
                );
                assert_eq!(
                    left.destination.snapped_point, right.destination.snapped_point,
                    "{context}"
                );
                assert!(left.sar.is_none());
            }
        }
        assert_matrix_parity(
            &matrix(&with, points.clone(), points.clone(), profile),
            &matrix(&without, points.clone(), points.clone(), profile),
        );
        // Edge ids differ between the two packages: compare the reached
        // edges by (way, from node, to node).
        let request = grid_isochrone_request(profile, [20.0, 45.0, 90.0]);
        let reached = |engine: &RoutingEngine, graph: &RoadGraph| {
            let ids = graph
                .graph
                .edge_references()
                .map(|edge| {
                    let info = graph
                        .shared_edge_info(edge.weight().shared_edge_info_id)
                        .unwrap();
                    (
                        edge.weight().edge_id,
                        (
                            info.osm_way_id,
                            graph.graph.node(edge.source()).source_node_id,
                            graph.graph.node(edge.target()).source_node_id,
                        ),
                    )
                })
                .collect::<std::collections::HashMap<_, _>>();
            engine
                .isochrone(request.clone())
                .unwrap()
                .buckets
                .iter()
                .map(|bucket| {
                    let mut edges = bucket.edge_ids.iter().map(|id| ids[id]).collect::<Vec<_>>();
                    edges.sort_unstable();
                    edges
                })
                .collect::<Vec<_>>()
        };
        assert_eq!(
            reached(&with, &with_graph),
            reached(&without, &without_graph),
            "{profile:?}"
        );
    }
}

/// Correlation snaps onto a road the law closes only for a SAR request
/// whose relaxations open it; `military` opens only when listed.
#[test]
fn correlation_snaps_onto_closed_roads_only_for_sar() {
    let dir = tempdir().unwrap();
    let graph = sar_grid_graph(true);
    let reader = persist(&graph, &dir.path().join("snap.mrt"));
    let compact = RoutingEngine::from_package(reader.package.clone());
    let points = sar_grid_points();
    let closed_edges = graph
        .graph
        .edge_references()
        .filter(|edge| edge.weight().is_sar_only())
        .map(|edge| edge.weight().edge_id)
        .collect::<std::collections::BTreeSet<_>>();
    assert!(!closed_edges.is_empty());
    let snap = |point: GeoPoint, sar: Option<&str>| {
        let mut request = LegRequest::new(point, points[0], MobilityProfile::Wheeled);
        // Far enough that a legal road is within reach of every point.
        request.options.max_correlation_distance_m = 150;
        request.options.sar = sar.map(sar_option);
        compact.route_leg(request).unwrap()
    };
    for point in [points[6], points[7]] {
        let plain = snap(point, None);
        assert!(!closed_edges.contains(&plain.origin.edge_id), "{point:?}");
        assert!(plain.origin.distance_m > 10.0, "{point:?}");
    }
    let r1 = r#"{"relaxations":{"legal_access":{}}}"#;
    let closed_route = snap(points[6], Some(r1));
    assert!(closed_edges.contains(&closed_route.origin.edge_id));
    assert!(closed_route.origin.distance_m < 5.0);
    let report = closed_route.sar.unwrap();
    let use_ = &report.uses[0];
    assert_eq!(use_.kind, crate::SarRelaxationKind::LegalAccess);
    assert_eq!(use_.legal_tag.as_deref(), Some("access=no"));
    assert!(use_.added_cost_s > 0.0);
    // Military stays closed unless listed.
    let military = snap(points[7], Some(r1));
    assert!(!closed_edges.contains(&military.origin.edge_id));
    let listed = snap(
        points[7],
        Some(r#"{"relaxations":{"legal_access":{"values":["military"]}}}"#),
    );
    assert!(closed_edges.contains(&listed.origin.edge_id));
    assert_eq!(
        listed.sar.unwrap().uses[0].legal_tag.as_deref(),
        Some("access=military")
    );
}

/// R2 drives a one-way's walker edge against it at its factor plus the
/// entry time; R4 charges a gate where a way ends at it, not where the way
/// passes through, and never passes a bollard.
#[test]
fn sar_wrong_way_and_barriers_cost_and_report_what_they_do() {
    let dir = tempdir().unwrap();
    let graph = sar_grid_graph(true);
    let reader = persist(&graph, &dir.path().join("ww.mrt"));
    let compact = RoutingEngine::from_package(reader.package.clone());
    let route_with = |origin: GeoPoint, destination: GeoPoint, sar: Option<&str>| {
        route(
            &compact,
            origin,
            destination,
            MobilityProfile::Wheeled,
            |request| {
                request.options.sar = sar.map(sar_option);
            },
        )
        .unwrap()
    };
    let (lat, lng) = (GRID_LAT, GRID_LNG);
    // Along the one-way (3,1)->(3,2), from its east end to its west end.
    let (east, west) = (
        GeoPoint::new(lat[3] + 0.00002, lng[2] - 0.0002),
        GeoPoint::new(lat[3] + 0.00002, lng[1] + 0.0002),
    );
    let legal = route_with(east, west, None);
    let wrong = route_with(east, west, Some(r#"{"relaxations":{"wrong_way":{}}}"#));
    assert!(wrong.cost < legal.cost, "{} vs {}", wrong.cost, legal.cost);
    assert!(wrong.distance_m < legal.distance_m);
    let report = wrong.sar.unwrap();
    assert_eq!(report.uses.len(), 1, "{report:?}");
    assert_eq!(report.uses[0].kind, crate::SarRelaxationKind::WrongWay);
    assert_eq!(report.uses[0].legal_tag.as_deref(), Some("oneway=yes"));
    // The route starts on the wrong-way edge: no entry, only the factor
    // (x2: the added cost is the edge's own cost over its traversed piece).
    assert!((report.uses[0].added_cost_s * 2.0 - wrong.cost).abs() < 1e-3 * wrong.cost);
    // Entering it from the street before it costs the entry time too (the
    // vertical into its east end: the corner at (3,3) is a vehicle island,
    // walled off by the gate at (2,3), the bollard at (4,3) and closed ways).
    let before = GeoPoint::new((lat[2] + lat[3]) / 2.0, lng[2] + 0.00002);
    let entered = route_with(before, west, Some(r#"{"relaxations":{"wrong_way":{}}}"#));
    let report = entered.sar.unwrap();
    let wrong_way = report
        .uses
        .iter()
        .find(|used| used.kind == crate::SarRelaxationKind::WrongWay);
    if let Some(wrong_way) = wrong_way {
        assert!(wrong_way.added_cost_s >= 30.0, "{wrong_way:?}");
    }

    // Row 0 passes through the gate at (0,2): free along it under R4.
    let r4 = Some(r#"{"relaxations":{"barriers":{}}}"#);
    let (west0, east0) = (
        GeoPoint::new(0.00002, (lng[1] + lng[2]) / 2.0),
        GeoPoint::new(0.00002, (lng[2] + lng[3]) / 2.0),
    );
    let through = route_with(west0, east0, r4);
    let plain = route_with(west0, east0, None);
    assert_eq!(through.cost.to_bits(), plain.cost.to_bits());
    assert!(through.sar.unwrap().uses.is_empty());
    // Turning off it onto the vertical that ends at the gate pays the gate
    // (a cheap one: at 90 s the route goes round the block instead).
    let north = GeoPoint::new(lat[1] / 2.0, lng[2] + 0.00003);
    let plain_turn = route_with(west0, north, None);
    let around = route_with(west0, north, r4);
    assert!(around.cost > plain_turn.cost);
    assert!(around.sar.unwrap().uses.is_empty(), "round the gate");
    let turned = route_with(
        west0,
        north,
        Some(r#"{"relaxations":{"barriers":{"gate_s":1}}}"#),
    );
    assert_eq!(turned.edge_ids, plain_turn.edge_ids);
    let report = turned.sar.clone().unwrap();
    let gate = report
        .uses
        .iter()
        .find(|used| used.kind == crate::SarRelaxationKind::Barriers)
        .expect("the gate is charged");
    assert_eq!(gate.legal_tag.as_deref(), Some("barrier:gate"));
    assert_eq!(gate.added_cost_s, 1.0);
    assert_eq!(gate.geometry.len(), 1);
    // R4's 1 s replaces the gate's normal cost.
    assert!((turned.cost - plain_turn.cost - 1.0 + crate::GATE_VEHICLE_S).abs() < 1e-6);
    // No route passes the bollard at (4,3), every way ending there: normal
    // routing closes it to vehicles, and R4 never opens it.
    let (left, right) = (
        GeoPoint::new(lat[4] - 0.00002, (lng[2] + lng[3]) / 2.0),
        GeoPoint::new(lat[4] - 0.00002, (lng[3] + lng[4]) / 2.0),
    );
    let bollard = graph
        .graph
        .node_weights()
        .find(|node| node.source_node_id == Some(sar_grid_node(4, 3)))
        .unwrap();
    assert_eq!(
        bollard.barrier_class,
        wayfarer_package::sar::SarBarrierClass::MotorBlocking as u8
    );
    let detour = route_with(left, right, r4);
    let plain = route_with(left, right, None);
    let bollard_edges = graph
        .graph
        .edge_references()
        .filter(|edge| {
            [edge.source(), edge.target()]
                .iter()
                .any(|node| graph.graph.node(*node).source_node_id == Some(sar_grid_node(4, 3)))
        })
        .map(|edge| edge.weight().edge_id)
        .collect::<std::collections::BTreeSet<_>>();
    // No transition at the bollard: no two consecutive route edges meet
    // there.
    for route in [&detour, &plain] {
        let through = route
            .edge_ids
            .windows(2)
            .filter(|pair| bollard_edges.contains(&pair[0]) && bollard_edges.contains(&pair[1]))
            .count();
        assert_eq!(through, 0, "{:?}", route.edge_ids);
    }
    assert!(
        detour
            .sar
            .unwrap()
            .uses
            .iter()
            .all(|used| used.legal_tag.as_deref() != Some("barrier:motor_blocking"))
    );
}

// ---------------------------------------------------------------------------
// SAR stage 2: R5 `vehicle_on_paths` and width classes.
// ---------------------------------------------------------------------------

/// Where [`sar_path_grid_graph`]'s islands lie: streets reachable by car only
/// over paths.
const SAR_ISLAND_EAST_LNG: f64 = 0.0066;
const SAR_ISLAND_WEST_LNG: f64 = -0.0015;
const SAR_ISLAND_SOUTH_LAT: f64 = -0.0012;

/// [`grid_graph`] (residential streets, one footway) and, with `paths`,
/// three islands of streets that a car reaches only over paths:
///
/// - east: a footway 3 m wide from (2, 4), and a footway 1.5 m wide from
///   (4, 4);
/// - west: a path of unknown width from (2, 0), and steps from (1, 0);
/// - south: a cycleway 2.6 m wide (by `est_width`'s stand-in, `width`) from
///   (0, 2) that bicycles may use one way only and walkers not at all, so
///   its other direction is a SAR-only edge;
///
/// plus a diagonal pedestrian street of unknown width across the block
/// (1, 1)–(2, 2). Without `paths` the islands' streets are there but the
/// paths are not.
fn sar_path_grid_graph(paths: bool) -> RoadGraph {
    let (lat, lng) = (GRID_LAT, GRID_LNG);
    let mut builder = RoadGraphBuilder::new("sar-path-grid");
    let grid_node = |row: usize, col: usize| 1_000 + (row * 10 + col) as i64;
    let mut way_id = 1;
    for row in 0..5 {
        for col in 0..5 {
            if col + 1 < 5 {
                let road_type = if row == 2 {
                    RoadType::Primary
                } else {
                    RoadType::Residential
                };
                builder.push_way(way(
                    way_id,
                    (lat[row], lng[col]),
                    (lat[row], lng[col + 1]),
                    grid_node(row, col),
                    grid_node(row, col + 1),
                    road_type,
                    DirectionalAccess::Both,
                ));
                way_id += 1;
            }
            if row + 1 < 5 {
                let road_type = if col == 3 && row == 1 {
                    RoadType::Path
                } else {
                    RoadType::Residential
                };
                builder.push_way(way(
                    way_id,
                    (lat[row], lng[col]),
                    (lat[row + 1], lng[col]),
                    grid_node(row, col),
                    grid_node(row + 1, col),
                    road_type,
                    DirectionalAccess::Both,
                ));
                way_id += 1;
            }
        }
    }
    let street = |id: i64, points: Vec<(f64, f64)>| RoutableWay {
        osm_way_id: id,
        points,
        road_type: RoadType::Residential,
        directional_access: DirectionalAccess::Both,
        highway: Some("residential".into()),
        ..Default::default()
    };
    let (east, west, south) = (
        SAR_ISLAND_EAST_LNG,
        SAR_ISLAND_WEST_LNG,
        SAR_ISLAND_SOUTH_LAT,
    );
    builder.push_way(street(
        500,
        vec![(lat[1], east), (lat[2], east), (lat[3], east)],
    ));
    builder.push_way(street(
        501,
        vec![(lat[1], west), (lat[2], west), (lat[3], west)],
    ));
    builder.push_way(street(
        502,
        vec![(south, lng[1]), (south, lng[2]), (south, lng[3])],
    ));
    if paths {
        let path = |id: i64, a: (f64, f64), b: (f64, f64), highway: &str, width: Option<&str>| {
            RoutableWay {
                osm_way_id: id,
                points: vec![a, b],
                road_type: RoadType::Path,
                directional_access: DirectionalAccess::Both,
                highway: Some(highway.into()),
                width: width.map(Into::into),
                ..Default::default()
            }
        };
        builder.push_way(path(
            600,
            (lat[2], lng[4]),
            (lat[2], east),
            "footway",
            Some("3"),
        ));
        builder.push_way(path(
            601,
            (lat[4], lng[4]),
            (lat[3], east),
            "footway",
            Some("1.5 m"),
        ));
        builder.push_way(path(602, (lat[2], lng[0]), (lat[2], west), "path", None));
        builder.push_way(path(
            603,
            (lat[1], lng[0]),
            (lat[1], west),
            "steps",
            Some("4"),
        ));
        builder.push_way(RoutableWay {
            foot: Some("no".into()),
            oneway_bicycle: Some("yes".into()),
            ..path(604, (0.0, lng[2]), (south, lng[2]), "cycleway", Some("2.6"))
        });
        builder.push_way(path(
            605,
            (lat[1], lng[1]),
            (lat[2], lng[2]),
            "pedestrian",
            None,
        ));
    }
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    graph
}

/// Grid points, then one on each island's street, then one on the wide
/// footway and one on the pedestrian street.
fn sar_path_grid_points() -> Vec<GeoPoint> {
    let (lat, lng) = (GRID_LAT, GRID_LNG);
    let mid = |a: f64, b: f64| (a + b) / 2.0;
    vec![
        GeoPoint::new(0.00004, mid(lng[0], lng[1])),
        GeoPoint::new(lat[4] - 0.00003, mid(lng[3], lng[4])),
        GeoPoint::new(mid(lat[2], lat[3]), lng[3] + 0.00003),
        GeoPoint::new(lat[3] + 0.00003, mid(lng[1], lng[2])),
        GeoPoint::new(mid(lat[1], lat[2]) + 0.0002, SAR_ISLAND_EAST_LNG + 0.00003),
        GeoPoint::new(mid(lat[1], lat[2]) + 0.0002, SAR_ISLAND_WEST_LNG - 0.00003),
        GeoPoint::new(SAR_ISLAND_SOUTH_LAT - 0.00003, mid(lng[1], lng[2])),
        GeoPoint::new(lat[2] + 0.00002, mid(lng[4], SAR_ISLAND_EAST_LNG)),
        GeoPoint::new(mid(lat[1], lat[2]) + 0.00002, mid(lng[1], lng[2])),
    ]
}

/// The R5 option sets the stage 2 tests run.
fn sar_stage2_sets() -> Vec<(&'static str, crate::RouteSarOptions)> {
    [
        ("r5", r#"{"relaxations":{"vehicle_on_paths":{}}}"#),
        (
            "r5-unknown",
            r#"{"relaxations":{"vehicle_on_paths":{"unknown_width":"allow"}}}"#,
        ),
        (
            "r5-any-width-x4",
            r#"{"relaxations":{"vehicleOnPaths":{"minWidthM":0,"unknownWidth":"allow","factor":4}}}"#,
        ),
        (
            "r5-wide-footways",
            r#"{"relaxations":{"vehicle_on_paths":{"classes":["footway"],"min_width_m":3}}}"#,
        ),
        (
            "r1-r3-r5",
            r#"{"relaxations":{"legal_access":{},"turn_restrictions":{},"vehicle_on_paths":{"unknown_width":"allow"}}}"#,
        ),
    ]
    .into_iter()
    .map(|(name, json)| (name, sar_option(json)))
    .collect()
}

/// R5 against the reference oracle: routes (bidirectional and forward
/// only), matrices in every mode and isochrones have its costs. Paths stay
/// out of the stored Wheeled column, so an R5 column is bounded by
/// potentials of its own metric (paths at R5's minimum factor), and an R1
/// column by the stored column's.
#[test]
fn sar_vehicle_on_paths_matches_the_reference() {
    use super::search::patch::dijkstra;
    use crate::matrix::{MatrixMode, test_mode};
    let dir = tempdir().unwrap();
    let reader = persist(&sar_path_grid_graph(true), &dir.path().join("sar2.mrt"));
    let (oracle, compact) = engines(&reader);
    let points = sar_path_grid_points();
    let mut compared = 0;
    let mut used = 0;
    for (name, sar) in sar_stage2_sets() {
        let configure = |request: &mut LegRequest| request.options.sar = Some(sar.clone());
        for origin in &points {
            for destination in &points {
                if origin == destination {
                    continue;
                }
                let context = format!("{name} {origin:?} -> {destination:?}");
                let expected = route(
                    &oracle,
                    *origin,
                    *destination,
                    MobilityProfile::Wheeled,
                    configure,
                );
                for forward_only in [false, true] {
                    dijkstra::force(forward_only);
                    let actual = route(
                        &compact,
                        *origin,
                        *destination,
                        MobilityProfile::Wheeled,
                        configure,
                    );
                    dijkstra::force(false);
                    assert_eq!(expected.is_some(), actual.is_some(), "{context}");
                    let (Some(expected), Some(actual)) = (&expected, actual) else {
                        continue;
                    };
                    assert!(
                        (actual.cost - expected.cost).abs() <= 1e-4 * expected.cost.max(1.0),
                        "{context} forward only {forward_only}: reference {} vs package {}",
                        expected.cost,
                        actual.cost
                    );
                    let report = actual.sar.as_ref().expect("a SAR report");
                    used += report
                        .uses
                        .iter()
                        .filter(|used| used.kind == crate::SarRelaxationKind::VehicleOnPaths)
                        .count();
                    compared += 1;
                }
            }
        }
        let mut request = crate::RouteMatrixRequest::new(
            points.clone(),
            points.clone(),
            MobilityProfile::Wheeled,
        );
        request.options.max_correlation_distance_m = 60;
        request.options.sar = Some(sar.clone());
        let reference = oracle.matrix(request.clone());
        for mode in [MatrixMode::Pairwise, MatrixMode::Rows] {
            test_mode::set(Some(mode));
            let actual = compact.matrix(request.clone());
            test_mode::set(None);
            assert_matrix_parity(&reference, &actual);
        }
        for origin in points.iter().skip(3).take(4) {
            let mut request = grid_isochrone_request(MobilityProfile::Wheeled, [20.0, 45.0, 90.0]);
            request.origin = *origin;
            request.options.sar = Some(sar.clone());
            let reference = oracle.isochrone(request.clone()).unwrap();
            let actual = compact.isochrone(request).unwrap();
            assert_isochrone_parity(&reference, &actual, &format!("{name} {origin:?}"));
        }
    }
    eprintln!("SAR stage 2 parity: {compared} routes, {used} vehicle_on_paths uses");
    assert!(used > 20, "{used}");
    let package = compact.package();
    let scale = |json: &str| {
        let key = super::columns::ColumnKey {
            profile: MobilityProfile::Wheeled,
            costing: crate::RouteCostingOptions::default().normalized(),
            allow_destination_only: true,
            sar: crate::sar::edge_relaxations(
                &crate::RouteOptions {
                    sar: Some(sar_option(json)),
                    ..Default::default()
                },
                MobilityProfile::Wheeled,
            ),
        };
        super::columns::derive_column(
            &package.view(),
            key,
            package.file().column_for(MobilityProfile::Wheeled),
        )
        .potential
    };
    // The stored column cannot use the paths: the profile's own metric
    // bounds R5.
    assert!(matches!(
        scale(r#"{"relaxations":{"vehicle_on_paths":{}}}"#),
        Some((super::columns::PotentialBase::Metric(_), _))
    ));
    let Some((super::columns::PotentialBase::Stored(_), legal)) =
        scale(r#"{"relaxations":{"legal_access":{}}}"#)
    else {
        panic!("the stored column bounds R1");
    };
    assert!(legal > 0.99 && legal <= 1.0, "{legal}");
}

/// Normal requests never use, nor snap onto, the paths R5 opens: Wheeled
/// routes and matrices on the grid with the paths cost exactly what they cost
/// without them (the islands stay unreachable by car).
#[test]
fn paths_opened_for_sar_leave_normal_requests_unchanged() {
    let dir = tempdir().unwrap();
    let with = persist(&sar_path_grid_graph(true), &dir.path().join("with.mrt"));
    let without = persist(&sar_path_grid_graph(false), &dir.path().join("without.mrt"));
    let (with, without) = (
        RoutingEngine::from_package(with.package.clone()),
        RoutingEngine::from_package(without.package.clone()),
    );
    let points = sar_path_grid_points();
    let profile = MobilityProfile::Wheeled;
    let mut routed = 0;
    for origin in &points {
        for destination in &points {
            if origin == destination {
                continue;
            }
            let context = format!("{origin:?} -> {destination:?}");
            let mut request = LegRequest::new(*origin, *destination, profile);
            request.options.max_correlation_distance_m = 60;
            let left = with.route_leg(request.clone());
            let right = without.route_leg(request);
            assert_eq!(
                left.is_ok(),
                right.is_ok(),
                "{context}: {left:?} vs {right:?}"
            );
            let (Ok(left), Ok(right)) = (left, right) else {
                continue;
            };
            assert_eq!(left.cost.to_bits(), right.cost.to_bits(), "{context}");
            assert_eq!(
                left.distance_m.to_bits(),
                right.distance_m.to_bits(),
                "{context}"
            );
            assert_eq!(
                left.origin.snapped_point, right.origin.snapped_point,
                "{context}"
            );
            assert_eq!(
                left.destination.snapped_point, right.destination.snapped_point,
                "{context}"
            );
            assert!(left.sar.is_none());
            routed += 1;
        }
    }
    // The grid points; the islands' are unreachable by car, the paths' too
    // far from a street.
    assert!(routed >= 12, "{routed}");
    assert_matrix_parity(
        &matrix(&with, points.clone(), points.clone(), profile),
        &matrix(&without, points.clone(), points.clone(), profile),
    );
}

/// R5 opens paths of the listed classes whose known width reaches
/// `min_width_m` (and of unknown width only with `unknown_width: allow`),
/// never steps, at its factor over the wheeled path cost, and reports each
/// stretch with its class, length, added cost and width confidence.
#[test]
fn vehicle_on_paths_filters_widths_and_reports_its_uses() {
    use crate::SarRelaxationKind as Kind;
    let dir = tempdir().unwrap();
    let graph = sar_path_grid_graph(true);
    let reader = persist(&graph, &dir.path().join("r5.mrt"));
    let compact = RoutingEngine::from_package(reader.package.clone());
    let points = sar_path_grid_points();
    let (grid, east, west, south, on_footway, on_pedestrian) = (
        points[0], points[4], points[5], points[6], points[7], points[8],
    );
    let route_with = |origin: GeoPoint, destination: GeoPoint, sar: Option<&str>| {
        route(
            &compact,
            origin,
            destination,
            MobilityProfile::Wheeled,
            |request| request.options.sar = sar.map(sar_option),
        )
    };
    let uses = |result: &LegResult| {
        result
            .sar
            .as_ref()
            .unwrap()
            .uses
            .iter()
            .filter(|used| used.kind == Kind::VehicleOnPaths)
            .cloned()
            .collect::<Vec<_>>()
    };
    // The islands are unreachable by car without R5.
    for island in [east, west, south] {
        assert!(route_with(grid, island, None).is_none(), "{island:?}");
    }
    // Default R5 (≥ 2.5 m, unknown denied): the 3 m footway east, the 2.6 m
    // cycleway south in both directions (its return is SAR-only); neither
    // the unknown-width path nor the steps west.
    let r5 = Some(r#"{"relaxations":{"vehicle_on_paths":{}}}"#);
    let to_east = route_with(grid, east, r5).expect("east over the wide footway");
    let footway = uses(&to_east);
    assert_eq!(footway.len(), 1, "{footway:?}");
    assert_eq!(footway[0].legal_tag.as_deref(), Some("highway=footway"));
    assert_eq!(footway[0].osm_way_id, Some(600));
    assert!(footway[0].confidence.is_empty(), "{footway:?}");
    // The whole footway (167 m), within the route.
    assert!(footway[0].length_m > 160.0 && footway[0].length_m <= to_east.distance_m);
    // The footway costs the route ×3 its wheeled path cost: the use adds
    // two thirds of what the route pays for it.
    let totals = &to_east.sar.as_ref().unwrap().totals;
    let total = totals
        .iter()
        .find(|total| total.kind == Kind::VehicleOnPaths)
        .unwrap();
    assert_eq!(total.count, 1);
    assert!(total.added_cost_s > 0.0 && total.added_cost_s < to_east.cost);
    let x4 = route_with(
        grid,
        east,
        Some(r#"{"relaxations":{"vehicle_on_paths":{"factor":4}}}"#),
    )
    .unwrap();
    assert_eq!(x4.edge_ids, to_east.edge_ids);
    let added = |result: &LegResult| uses(result)[0].added_cost_s;
    // Added cost is (factor - 1) times the footway's wheeled cost.
    assert!((added(&x4) / added(&to_east) - 1.5).abs() < 1e-6);
    assert!((x4.cost - to_east.cost - added(&to_east) / 2.0).abs() < 1e-6 * x4.cost);
    let from_east = route_with(east, grid, r5).expect("back over the wide footway");
    assert_eq!(uses(&from_east).len(), 1);
    for (origin, destination) in [(grid, south), (south, grid)] {
        let crossed = route_with(origin, destination, r5).expect("the cycleway both ways");
        let cycleway = uses(&crossed);
        assert_eq!(cycleway[0].legal_tag.as_deref(), Some("highway=cycleway"));
    }
    assert!(route_with(grid, west, r5).is_none(), "unknown width denied");
    // The SAR-only return of the cycleway exists in the road graph.
    assert!(graph.graph.edge_references().any(|edge| {
        edge.weight().is_sar_only()
            && graph
                .shared_edge_info(edge.weight().shared_edge_info_id)
                .is_some_and(|info| info.attrs.highway.as_deref() == Some("cycleway"))
    }));
    // Unknown widths when allowed, reported as such; steps never.
    let allow = Some(r#"{"relaxations":{"vehicle_on_paths":{"unknown_width":"allow"}}}"#);
    let to_west = route_with(grid, west, allow).expect("west over the path");
    let path = uses(&to_west);
    assert_eq!(path.len(), 1, "{path:?}");
    assert_eq!(path[0].legal_tag.as_deref(), Some("highway=path"));
    assert_eq!(path[0].confidence, vec!["width unknown".to_string()]);
    assert_eq!(path[0].osm_way_id, Some(602));
    // The narrow footway (1.5 m) is never opened under the default minimum,
    // even when the wide one is blocked; with `min_width_m: 0` it is.
    let wide_edges = graph
        .graph
        .edge_references()
        .filter(|edge| {
            graph
                .shared_edge_info(edge.weight().shared_edge_info_id)
                .is_some_and(|info| info.osm_way_id == 600)
        })
        .map(|edge| edge.weight().edge_id)
        .collect::<Vec<_>>();
    let blocked = |sar: &str| {
        let mut request = LegRequest::new(grid, east, MobilityProfile::Wheeled);
        request.options.max_correlation_distance_m = 60;
        request.options.sar = Some(sar_option(sar));
        request.options.blocked_edge_ids = wide_edges.iter().copied().collect();
        compact.route_leg(request)
    };
    assert!(blocked(r#"{"relaxations":{"vehicle_on_paths":{"unknown_width":"allow"}}}"#).is_err());
    let narrow = blocked(r#"{"relaxations":{"vehicle_on_paths":{"min_width_m":0}}}"#)
        .expect("over the narrow footway");
    assert_eq!(uses(&narrow)[0].osm_way_id, Some(601));
    // Classes and widths filter: only footways of 3 m or more.
    let wide_footways =
        Some(r#"{"relaxations":{"vehicle_on_paths":{"classes":["footway"],"min_width_m":3}}}"#);
    assert!(route_with(grid, east, wide_footways).is_some());
    assert!(route_with(grid, south, wide_footways).is_none());
    // Correlation snaps onto a path only for R5, and only onto one it opens
    // (both points lie over 60 m from a street; a street within reach scores
    // above a path for a car).
    let snap = |point: GeoPoint, sar: Option<&str>| {
        route(&compact, point, grid, MobilityProfile::Wheeled, |request| {
            request.options.sar = sar.map(sar_option)
        })
    };
    assert!(snap(on_footway, None).is_none());
    let snapped = snap(on_footway, r5).expect("snaps onto the wide footway");
    assert!(snapped.origin.distance_m < 5.0);
    assert_eq!(uses(&snapped)[0].osm_way_id, Some(600));
    assert!(snap(on_pedestrian, r5).is_none(), "unknown width denied");
    let pedestrian = snap(on_pedestrian, allow).expect("snaps onto the pedestrian street");
    assert!(pedestrian.origin.distance_m < 5.0);
    assert_eq!(
        uses(&pedestrian)[0].legal_tag.as_deref(),
        Some("highway=pedestrian")
    );
    // The policy summary lists the effective R5 options.
    let summary = to_west.policy_summary.sar.as_ref().unwrap();
    assert_eq!(summary.vehicle_on_paths_factor, Some(3.0));
    assert_eq!(summary.vehicle_on_paths_min_width_m, Some(2.5));
    assert_eq!(
        summary.vehicle_on_paths_unknown_width,
        Some(crate::sar::SarUnknownWidth::Allow)
    );
}

// ---------------------------------------------------------------------------
// Walks through clean pieces (`search::Run::walk_forward`)
// ---------------------------------------------------------------------------

/// Four junctions round a middle junction, the south side a chain of four
/// nodes with dead-end trees hanging off it (a branching one at its second
/// node, a single edge at its third), and dead ends off two junctions. The
/// chain is one-way eastbound with `one_way`; `barriers` are OSM node ids
/// with a gate.
const PIECE_NODES: [(i64, (f64, f64)); 17] = [
    (1, (0.0, 0.0)),
    (11, (0.00002, 0.002)),
    (12, (0.0, 0.004)),
    (13, (0.00003, 0.006)),
    (14, (0.00001, 0.008)),
    (2, (0.0, 0.010)),
    (3, (0.004, 0.0)),
    (31, (0.0042, 0.005)),
    (4, (0.004, 0.010)),
    (5, (0.002, 0.005)),
    (21, (-0.001, 0.004)),
    (22, (-0.002, 0.0041)),
    (23, (-0.0015, 0.0049)),
    (24, (-0.0008, 0.006)),
    (25, (-0.001, 0.0101)),
    (26, (-0.0005, -0.0015)),
    (27, (-0.0012, -0.0016)),
];

/// The position of [`piece_graph`]'s OSM node `id`.
fn piece_node(id: i64) -> (f64, f64) {
    PIECE_NODES.iter().find(|(node, _)| *node == id).unwrap().1
}

fn piece_graph(one_way: bool, barriers: &[i64]) -> RoadGraph {
    piece_graph_with(one_way, barriers, &[])
}

/// [`piece_graph`] with `extra` ways between its OSM nodes (residential,
/// both ways; ids from 121).
fn piece_graph_with(one_way: bool, barriers: &[i64], extra: &[(i64, i64)]) -> RoadGraph {
    let at = piece_node;
    let chain = [(1, 11), (11, 12), (12, 13), (13, 14), (14, 2)];
    let others = [
        (1, 3),
        (2, 4),
        (3, 31),
        (31, 4),
        (1, 5),
        (2, 5),
        (3, 5),
        (4, 5),
        (12, 21),
        (21, 22),
        (21, 23),
        (13, 24),
        (2, 25),
        (1, 26),
        (26, 27),
    ];
    let mut builder = RoadGraphBuilder::new("pieces");
    for (index, (a, b)) in chain.iter().chain(&others).chain(extra).enumerate() {
        let access = if one_way && index < chain.len() {
            DirectionalAccess::Forward
        } else {
            DirectionalAccess::Both
        };
        builder.push_way(way(
            101 + index as i64,
            at(*a),
            at(*b),
            *a,
            *b,
            RoadType::Residential,
            access,
        ));
    }
    builder.set_barrier_node_tags(
        barriers
            .iter()
            .map(|node| {
                (
                    *node,
                    wayfarer_road_graph::NodeBarrierTags {
                        barrier: "gate".to_string(),
                        ..Default::default()
                    },
                )
            })
            .collect(),
    );
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    graph
}

/// Points on every way of [`piece_graph`] and off its corners.
fn piece_points() -> Vec<GeoPoint> {
    vec![
        GeoPoint::new(0.00001, 0.001),
        GeoPoint::new(0.00001, 0.005),
        GeoPoint::new(0.00002, 0.009),
        GeoPoint::new(0.002, 0.0),
        GeoPoint::new(0.002, 0.010),
        GeoPoint::new(0.0041, 0.0025),
        GeoPoint::new(0.001, 0.0025),
        GeoPoint::new(0.003, 0.0075),
        GeoPoint::new(-0.0015, 0.00405),
        GeoPoint::new(-0.0012, 0.0045),
        GeoPoint::new(-0.0004, 0.006),
        GeoPoint::new(-0.0005, 0.0101),
        GeoPoint::new(-0.0009, -0.00155),
    ]
}

/// Routes every ordered pair of `points` with walks and without, and against
/// the reference: the same costs and paths, durations within rounding.
/// Returns the routes found and the edges walked.
fn assert_walks_match_per_edge(
    reader: &Fixture,
    points: &[GeoPoint],
    configure: impl Fn(&mut LegRequest),
) -> (usize, usize) {
    assert_profile_walks_match_per_edge(
        reader,
        points,
        &[MobilityProfile::Wheeled, MobilityProfile::OnFoot],
        configure,
    )
}

/// [`assert_walks_match_per_edge`] for `profiles`.
fn assert_profile_walks_match_per_edge(
    reader: &Fixture,
    points: &[GeoPoint],
    profiles: &[MobilityProfile],
    configure: impl Fn(&mut LegRequest),
) -> (usize, usize) {
    let (oracle, compact) = engines(reader);
    let mut found = 0;
    search::walks::take_walked();
    for profile in profiles.iter().copied() {
        for origin in points {
            for destination in points {
                if origin == destination {
                    continue;
                }
                let case = format!("{profile:?} {origin:?} -> {destination:?}");
                let walked = route(&compact, *origin, *destination, profile, &configure);
                search::walks::set_off(true);
                let per_edge = route(&compact, *origin, *destination, profile, &configure);
                search::walks::set_off(false);
                let reference = route(&oracle, *origin, *destination, profile, &configure);
                assert_eq!(walked.is_some(), per_edge.is_some(), "{case}");
                assert_eq!(walked.is_some(), reference.is_some(), "{case}");
                let (Some(walked), Some(per_edge), Some(reference)) = (walked, per_edge, reference)
                else {
                    continue;
                };
                found += 1;
                assert!(
                    (walked.cost - per_edge.cost).abs() <= search::tie_window(per_edge.cost),
                    "{case}: walks {} vs per edge {}",
                    walked.cost,
                    per_edge.cost
                );
                assert_eq!(walked.edge_ids, per_edge.edge_ids, "{case}");
                assert!(
                    (walked.duration_s - per_edge.duration_s).abs() <= 1e-9 * walked.duration_s
                );
                assert!(
                    (walked.distance_m - per_edge.distance_m).abs() <= 1e-9 * walked.distance_m
                );
                assert!(
                    (walked.cost - reference.cost).abs() <= 1e-3 * reference.cost.max(1.0),
                    "{case}: walks {} vs reference {}",
                    walked.cost,
                    reference.cost
                );
            }
        }
    }
    (found, search::walks::take_walked())
}

#[test]
fn walks_through_pieces_match_the_per_edge_search() {
    let dir = tempdir().unwrap();
    let reader = persist(&piece_graph(false, &[]), &dir.path().join("pieces.mrt"));
    // Endpoints inside pieces (on the chain, in each dead-end tree) and
    // outside; the two sides meet across the chain from corner to corner.
    let (found, walked) = assert_walks_match_per_edge(&reader, &piece_points(), |_| {});
    assert!(found > 250, "{found} routes");
    assert!(walked > 100, "{walked} edges walked");
}

#[test]
fn walks_match_the_per_edge_search_on_a_one_way_chain() {
    let dir = tempdir().unwrap();
    let reader = persist(&piece_graph(true, &[]), &dir.path().join("one-way.mrt"));
    let (found, walked) = assert_walks_match_per_edge(&reader, &piece_points(), |_| {});
    assert!(found > 250 && walked > 0, "{found} routes, {walked} walked");
}

#[test]
fn walks_match_the_per_edge_search_with_gates_inside_pieces() {
    let dir = tempdir().unwrap();
    // A gate on the chain (13) and one in a dead-end tree (21): each costs
    // more than a U-turn for vehicles, less for walkers.
    let reader = persist(
        &piece_graph(false, &[13, 21]),
        &dir.path().join("gates.mrt"),
    );
    let (found, walked) = assert_walks_match_per_edge(&reader, &piece_points(), |_| {});
    assert!(found > 250 && walked > 0, "{found} routes, {walked} walked");
}

#[test]
fn walks_match_the_per_edge_search_around_restrictions() {
    let dir = tempdir().unwrap();
    let mut graph = piece_graph(false, &[]);
    let edge = |way: i64, from: i64, to: i64| edge_between(&graph, way, from, to);
    let restriction = |relation_id: i64, from_edge_id: u64, to_edge_id: u64, via_node_id: i64| {
        ResolvedEdgeRestriction::Simple {
            relation_id,
            restriction: "no_left_turn".into(),
            from_edge_id,
            to_edge_id,
            via_node_id,
            osm_mode_mask: OsmMode::MOTOR_VEHICLE,
            mobility_mode_mask: MobilityMode::WHEELED,
        }
    };
    // Via chain node 12: its piece (the chain and its trees) keeps the
    // per-edge search. At junction 2, eastbound traffic off the chain may
    // not turn north towards 4 or 5: it turns round in the dead end at 25
    // (way 118), or further on.
    let restrictions = vec![
        restriction(1, edge(102, 11, 12), edge(103, 12, 13), 12),
        restriction(2, edge(105, 14, 2), edge(107, 2, 4), 2),
        restriction(3, edge(105, 14, 2), edge(111, 2, 5), 2),
    ];
    let dead_end = [edge(118, 2, 25), edge(118, 25, 2)];
    graph.set_resolved_restrictions(restrictions);
    let reader = persist(&graph, &dir.path().join("restrictions.mrt"));
    let (found, _) = assert_walks_match_per_edge(&reader, &piece_points(), |_| {});
    assert!(found > 250, "{found} routes");
    // The dead end's turn: from the chain north towards 4.
    let (_, compact) = engines(&reader);
    let turn = route(
        &compact,
        GeoPoint::new(0.00001, 0.005),
        GeoPoint::new(0.002, 0.010),
        MobilityProfile::Wheeled,
        |_| {},
    )
    .expect("route");
    assert!(
        turn.edge_ids.windows(2).any(|pair| pair == dead_end),
        "{:?}",
        turn.edge_ids
    );
}

#[test]
fn walks_match_the_per_edge_search_with_blocked_edges() {
    let dir = tempdir().unwrap();
    let graph = piece_graph(false, &[]);
    // A blocked chain edge and a blocked tree edge: the chain becomes two
    // dead ends.
    let blocked = [
        edge_between(&graph, 103, 12, 13),
        edge_between(&graph, 115, 21, 22),
    ];
    let reader = persist(&graph, &dir.path().join("blocked.mrt"));
    let (found, walked) = assert_walks_match_per_edge(&reader, &piece_points(), |request| {
        request.options.blocked_edge_ids.extend(blocked);
    });
    assert!(found > 200 && walked > 0, "{found} routes, {walked} walked");
}

/// The incoming adjacency, sources and end headings derive from edge twins.
/// A node's in-edges are the twins of its out-edges and its edges without a
/// twin, in ascending edge order, each with its source; an edge's end heading is its twin's start reversed, or
/// stored for an edge without a twin.
#[test]
fn twins_derive_in_edges_sources_and_end_headings() {
    let dir = tempdir().unwrap();
    // A one-way motorway has one directed edge (no mode may go against it,
    // nor is it drivable that way), without a twin.
    let (both, residential) = (DirectionalAccess::Both, RoadType::Residential);
    let motorway = |id, a, b, node_a, node_b| RoutableWay {
        highway: Some("motorway".into()),
        ..way(
            id,
            a,
            b,
            node_a,
            node_b,
            RoadType::Motorway,
            DirectionalAccess::Forward,
        )
    };
    let graph = small_graph(
        "twins",
        vec![
            motorway(1, (0.0, 0.0), (0.0, 0.002), 10, 11),
            way(2, (0.0, 0.002), (0.002, 0.002), 11, 12, residential, both),
            way(3, (0.002, 0.002), (0.002, 0.0), 12, 13, residential, both),
            motorway(4, (0.0, 0.0), (0.002, 0.0), 10, 13),
            motorway(6, (0.002, 0.0), (0.0, 0.0), 13, 10),
            way(5, (0.002, 0.0), (0.0, 0.002), 13, 11, residential, both),
        ],
    );
    let reader = persist(&graph, &dir.path().join("twins.mrt"));
    let view = reader.package.view();
    use wayfarer_package::format::{NO_HEADING, NO_TWIN};
    let lone = view
        .edge_twin
        .iter()
        .filter(|twin| **twin == NO_TWIN)
        .count();
    assert!(
        lone > 0 && lone < view.edge_count,
        "{lone} edges without a twin"
    );
    assert_eq!(view.lone_in_edge.len(), lone);
    let source_of = |edge: u32| {
        (0..view.node_count as u32)
            .find(|node| {
                (view.first_out[*node as usize]..view.first_out[*node as usize + 1]).contains(&edge)
            })
            .unwrap()
    };
    for node in 0..view.node_count {
        let expected = (0..view.edge_count as u32)
            .filter(|edge| view.edge_target[*edge as usize] as usize == node)
            .map(|edge| (edge, source_of(edge)))
            .collect::<Vec<_>>();
        assert_eq!(
            view.in_edges(node).collect::<Vec<_>>(),
            expected,
            "node {node}"
        );
    }
    // Sources: the twin's target, or by the out-adjacency.
    for edge in 0..view.edge_count {
        assert_eq!(
            view.edge_source(edge),
            source_of(edge as u32),
            "edge {edge}"
        );
    }
    // End headings: the twin's start heading reversed, or stored.
    let end_heading = graph
        .graph
        .edge_references()
        .map(|edge| (edge.weight().edge_id, edge.weight().end_heading_deg))
        .collect::<std::collections::HashMap<_, _>>();
    for edge in 0..view.edge_count {
        let expected = end_heading[&view.edge_id(edge)].unwrap_or(NO_HEADING);
        assert_eq!(view.heading_end(edge), expected, "edge {edge}");
    }
}

#[test]
fn piece_kinds_mark_the_chain_and_its_trees() {
    let dir = tempdir().unwrap();
    let reader = persist(&piece_graph(false, &[13]), &dir.path().join("kinds.mrt"));
    let view = reader.package.view();
    let column = reader
        .package
        .file()
        .column_for(MobilityProfile::Wheeled)
        .unwrap();
    let kinds = view.piece_kinds.column(column).expect("piece kinds");
    let node = |osm: i64| {
        let (lat, lng) = piece_node(osm);
        // The package stores the vector in fixed point, so compare its
        // round trip.
        use wayfarer_package::format::{fixed_xyz, unit_xyz, xyz_from_fixed};
        let xyz = xyz_from_fixed(fixed_xyz(unit_xyz(lat, lng)));
        (0..view.node_count as u32)
            .find(|node| view.xyz(*node) == xyz)
            .unwrap()
    };
    use wayfarer_package::format::PieceKind;
    for (osm, kind) in [
        (1, PieceKind::Other),
        (2, PieceKind::Other),
        (5, PieceKind::Other),
        (31, PieceKind::Chain),
        (11, PieceKind::Chain),
        (12, PieceKind::Chain),
        (13, PieceKind::Chain),
        (14, PieceKind::Chain),
        (21, PieceKind::Tree),
        (22, PieceKind::Tree),
        (24, PieceKind::Tree),
        (25, PieceKind::Tree),
        (27, PieceKind::Tree),
    ] {
        assert_eq!(kinds.kind(node(osm)), kind, "node {osm}");
    }
}

/// A chain from junction 1 east along a primary road to 2, where it turns
/// back west-north-west onto a residential road (a hairpin dearer than a
/// U-turn) towards junction 3, with a 7 m dead end straight on at 2.
/// Junctions 1 and 3 also meet through 4 and 5, far to the west.
fn hairpin_graph() -> RoadGraph {
    let at = |id: i64| match id {
        1 => (0.0, 0.0),
        2 => (0.0, 0.004),
        3 => (0.0008, 0.0002),
        6 => (0.0, 0.00406),
        4 => (0.0004, -0.006),
        5 => (0.0006, -0.008),
        _ => unreachable!(),
    };
    let ways = [
        (1, 2, RoadType::Primary),
        (2, 3, RoadType::Residential),
        (2, 6, RoadType::Residential),
        (1, 4, RoadType::Residential),
        (4, 3, RoadType::Residential),
        (1, 5, RoadType::Residential),
        (5, 3, RoadType::Residential),
    ];
    let mut builder = RoadGraphBuilder::new("hairpin");
    for (index, (a, b, road_type)) in ways.iter().enumerate() {
        builder.push_way(way(
            201 + index as i64,
            at(*a),
            at(*b),
            *a,
            *b,
            *road_type,
            DirectionalAccess::Both,
        ));
    }
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    graph
}

#[test]
fn walks_stop_at_a_turn_dearer_than_a_u_turn() {
    let dir = tempdir().unwrap();
    let graph = hairpin_graph();
    let excursion = [
        edge_between(&graph, 201, 1, 2),
        edge_between(&graph, 203, 2, 6),
        edge_between(&graph, 203, 6, 2),
        edge_between(&graph, 202, 2, 3),
    ];
    let reader = persist(&graph, &dir.path().join("hairpin.mrt"));
    let points = [
        GeoPoint::new(0.0001, -0.0015),
        GeoPoint::new(0.0007, -0.0015),
        GeoPoint::new(0.0005, -0.007),
    ];
    let (found, walked) = assert_walks_match_per_edge(&reader, &points, |_| {});
    assert!(found >= 6 && walked > 0, "{found} routes, {walked} walked");
    // From the south-west road to the north-west one, cars turn round in the
    // dead end rather than take the hairpin.
    let (_, compact) = engines(&reader);
    let route = route(
        &compact,
        points[0],
        points[1],
        MobilityProfile::Wheeled,
        |_| {},
    )
    .expect("route");
    assert!(
        route.edge_ids.windows(4).any(|window| window == excursion),
        "{:?}",
        route.edge_ids
    );
}

// ---------------------------------------------------------------------------
// Walks in overlay searches (`search::patch::walk`)
// ---------------------------------------------------------------------------

/// A point on [`piece_graph`]'s way `.0` from OSM node `.1` to `.2`, at
/// fraction `.3`.
type PiecePoint = (i64, i64, i64, f64);

/// Partial blocks of `ranges` (way, from, to, start, end) on
/// [`piece_graph`], with `connectors` ((way, from, to, fraction) at both
/// ends, cost) added.
fn piece_overlay(
    graph: &RoadGraph,
    ranges: &[(i64, i64, i64, f64, f64)],
    connectors: &[(PiecePoint, PiecePoint, f64)],
) -> crate::RouteDynamicOverlay {
    let edge = |(way, from, to): (i64, i64, i64)| edge_between(graph, way, from, to);
    let mut overlay = crate::RouteDynamicOverlay::from_blocked_ranges(
        &ranges
            .iter()
            .map(|(way, from, to, start, end)| crate::RouteEdgeRange {
                edge_id: edge((*way, *from, *to)),
                start_fraction: *start,
                end_fraction: *end,
            })
            .collect::<Vec<_>>(),
    );
    for (index, (source, target, cost)) in connectors.iter().enumerate() {
        overlay.virtual_edges.push(edge_connector(
            index as u64,
            crate::RouteDynamicVirtualEdgeKind::RoadToRoadBypass,
            (edge((source.0, source.1, source.2)), source.3),
            (edge((target.0, target.1, target.2)), target.3),
            *cost,
        ));
    }
    overlay.refresh_diagnostics();
    overlay
}

/// Routes every pair of [`piece_points`] over `overlay` with walks, without
/// and against the reference.
fn assert_overlay_walks_match(
    name: &str,
    barriers: &[i64],
    overlay: crate::RouteDynamicOverlay,
) -> (usize, usize) {
    let dir = tempdir().unwrap();
    let graph = piece_graph(false, barriers);
    let reader = persist(&graph, &dir.path().join(format!("{name}.mrt")));
    assert_walks_match_per_edge(&reader, &piece_points(), |request| {
        request.options.dynamic_overlay = Some(overlay.clone());
    })
}

#[test]
fn overlay_walks_match_with_a_split_edge_on_a_chain() {
    let graph = piece_graph(false, &[]);
    // The chain edge 12 -> 13 partly blocked with a bypass round the gap,
    // and 13 -> 12 blocked from the middle to its end.
    let overlay = piece_overlay(
        &graph,
        &[(103, 12, 13, 0.3, 0.6), (103, 13, 12, 0.5, 1.0)],
        &[((103, 12, 13, 0.3), (103, 12, 13, 0.6), 30.0)],
    );
    let (found, walked) = assert_overlay_walks_match("chain-split", &[], overlay);
    assert!(found > 250 && walked > 0, "{found} routes, {walked} walked");
}

#[test]
fn overlay_walks_match_with_a_split_edge_in_a_dead_end() {
    let graph = piece_graph(false, &[]);
    // The far half of the dead end 21 -> 22 blocked (a dead end inside the
    // tree's edge), and the dead end off junction 2 cut in its middle.
    let overlay = piece_overlay(
        &graph,
        &[
            (115, 21, 22, 0.5, 1.0),
            (118, 2, 25, 0.4, 0.6),
            (118, 25, 2, 0.4, 0.6),
        ],
        &[],
    );
    let (found, walked) = assert_overlay_walks_match("tree-split", &[], overlay);
    assert!(found > 200 && walked > 0, "{found} routes, {walked} walked");
}

#[test]
fn overlay_walks_match_with_connectors_inside_pieces_and_at_junctions() {
    let graph = piece_graph(false, &[]);
    // From inside the chain's tree at 24 to junction 4 (fraction 1 of
    // 31 -> 4), and from junction 1 (fraction 0 of 1 -> 26) into the dead
    // end 21 -> 23.
    let overlay = piece_overlay(
        &graph,
        &[],
        &[
            ((117, 13, 24, 0.5), (109, 31, 4, 1.0), 40.0),
            ((119, 1, 26, 0.0), (116, 21, 23, 0.5), 20.0),
        ],
    );
    let (found, walked) = assert_overlay_walks_match("connectors", &[], overlay);
    assert!(found > 250 && walked > 0, "{found} routes, {walked} walked");
}

#[test]
fn overlay_walks_match_where_the_best_path_takes_a_connector_across_pieces() {
    let graph = piece_graph(false, &[]);
    // A cheap connector from the dead end off junction 2 (at 25) to the one
    // off junction 1 (at 26), across the untouched chain between them:
    // routes between the roads north of 2 and of 1 take it, and the
    // searches walk the chain on the way.
    let overlay = piece_overlay(&graph, &[], &[((118, 2, 25, 0.5), (119, 1, 26, 0.5), 2.0)]);
    let (found, walked) = assert_overlay_walks_match("shortcut", &[], overlay.clone());
    assert!(found > 250 && walked > 0, "{found} routes, {walked} walked");
    // With gates on the chain (13) and in its tree (21): turning back at a
    // gate costs more than a U-turn for vehicles.
    let (found, walked) = assert_overlay_walks_match("shortcut-gates", &[13, 21], overlay.clone());
    assert!(found > 250 && walked > 0, "{found} routes, {walked} walked");
    let dir = tempdir().unwrap();
    let reader = persist(&graph, &dir.path().join("shortcut-route.mrt"));
    let (_, compact) = engines(&reader);
    let taken = route(
        &compact,
        GeoPoint::new(0.002, 0.010),
        GeoPoint::new(0.002, 0.0),
        MobilityProfile::Wheeled,
        |request| request.options.dynamic_overlay = Some(overlay.clone()),
    )
    .expect("route");
    assert!(
        taken.edge_ids.iter().any(|id| id >> 63 == 1),
        "{:?}",
        taken.edge_ids
    );
}

#[test]
fn overlay_walks_stop_at_a_turn_dearer_than_a_u_turn() {
    let dir = tempdir().unwrap();
    let graph = hairpin_graph();
    let excursion = [
        edge_between(&graph, 201, 1, 2),
        edge_between(&graph, 203, 2, 6),
        edge_between(&graph, 203, 6, 2),
        edge_between(&graph, 202, 2, 3),
    ];
    // A dear connector between the two western roads touches their pieces,
    // not the chain from 1 through 2 to 3.
    let edge = |way: i64, from: i64, to: i64| edge_between(&graph, way, from, to);
    let mut overlay = crate::RouteDynamicOverlay::default();
    overlay.virtual_edges.push(edge_connector(
        0,
        crate::RouteDynamicVirtualEdgeKind::CrossCountry,
        (edge(204, 1, 4), 0.5),
        (edge(206, 1, 5), 0.5),
        1e5,
    ));
    overlay.refresh_diagnostics();
    let reader = persist(&graph, &dir.path().join("hairpin-overlay.mrt"));
    let points = [
        GeoPoint::new(0.0001, -0.0015),
        GeoPoint::new(0.0007, -0.0015),
        GeoPoint::new(0.0005, -0.007),
    ];
    let configure = |request: &mut LegRequest| {
        request.options.dynamic_overlay = Some(overlay.clone());
    };
    let (found, walked) = assert_walks_match_per_edge(&reader, &points, configure);
    assert!(found >= 6 && walked > 0, "{found} routes, {walked} walked");
    let (_, compact) = engines(&reader);
    let route = route(
        &compact,
        points[0],
        points[1],
        MobilityProfile::Wheeled,
        configure,
    )
    .expect("route");
    assert!(
        route.edge_ids.windows(4).any(|window| window == excursion),
        "{:?}",
        route.edge_ids
    );
}

// ---------------------------------------------------------------------------
// Walks on derived columns (`columns::ColumnMarks`)
// ---------------------------------------------------------------------------

/// Sets the mobility mode mask of every edge of OSM way `way` in `graph`.
fn set_way_modes(graph: &mut RoadGraph, way: i64, mask: u32) {
    let infos = graph
        .shared_edge_infos
        .iter()
        .filter(|info| info.osm_way_id == way)
        .map(|info| info.shared_edge_info_id)
        .collect::<Vec<_>>();
    let mut changed = 0;
    for edge in graph.graph.edge_indices() {
        changed += graph.graph.update_edge(edge, |edge| {
            let found = infos.contains(&edge.shared_edge_info_id);
            if found {
                edge.mobility_mode_mask = mask;
            }
            usize::from(found)
        });
    }
    assert!(changed > 0, "way {way}");
}

#[test]
fn derived_column_walks_match_with_options_and_profiles_without_a_column() {
    let dir = tempdir().unwrap();
    let reader = persist(
        &piece_graph(false, &[13, 21]),
        &dir.path().join("derived.mrt"),
    );
    // Every derived column here uses only edges its stored column uses.
    let profiles = [
        MobilityProfile::Emergency,
        MobilityProfile::Tracked,
        MobilityProfile::Legged,
    ];
    let (found, walked) =
        assert_profile_walks_match_per_edge(&reader, &piece_points(), &profiles, |_| {});
    assert!(found > 350 && walked > 0, "{found} routes, {walked} walked");
    let (found, walked) = assert_walks_match_per_edge(&reader, &piece_points(), |request| {
        request.options.costing.prefer_paved = 0.5;
        request.options.costing.avoid_gates = 1.0;
    });
    assert!(found > 250 && walked > 0, "{found} routes, {walked} walked");
}

#[test]
fn derived_column_walks_match_where_the_column_closes_a_chain_edge() {
    let dir = tempdir().unwrap();
    // The chain's middle segment (12 - 13) and a tree edge (21 - 22) are
    // for wheeled vehicles only: Tracked's column cannot use them and walks
    // with Wheeled's marks.
    let mut graph = piece_graph(false, &[]);
    set_way_modes(&mut graph, 103, MobilityMode::WHEELED);
    set_way_modes(&mut graph, 115, MobilityMode::WHEELED);
    let reader = persist(&graph, &dir.path().join("closed.mrt"));
    let (found, walked) = assert_profile_walks_match_per_edge(
        &reader,
        &piece_points(),
        &[MobilityProfile::Tracked],
        |_| {},
    );
    assert!(found > 100 && walked > 0, "{found} routes, {walked} walked");
}

#[test]
fn derived_column_walks_match_where_the_column_opens_an_edge_at_a_marked_node() {
    let dir = tempdir().unwrap();
    // A track from tree node 24 (the dead end off chain node 13) to junction
    // 5, for tracked vehicles only: Emergency's column (wheeled or tracked)
    // uses it, Wheeled's, whose marks Emergency borrows, does not. The dead
    // end at 24 is then no dead end: the whole piece, chain and trees, loses
    // its marks; the dead end 1 - 26 - 27 keeps them. With 5's other roads
    // but the one to 2 blocked, routes from the west to 5 run along the
    // chain into the dead end and over the track.
    let mut graph = piece_graph_with(false, &[], &[(24, 5)]);
    set_way_modes(&mut graph, 121, MobilityMode::TRACKED);
    let blocked = [(110, 1, 5), (112, 3, 5), (113, 4, 5)]
        .into_iter()
        .flat_map(|(way, a, b)| {
            [
                edge_between(&graph, way, a, b),
                edge_between(&graph, way, b, a),
            ]
        })
        .collect::<Vec<_>>();
    let reader = persist(&graph, &dir.path().join("opened.mrt"));
    let configure = |request: &mut LegRequest| {
        request
            .options
            .blocked_edge_ids
            .extend(blocked.iter().copied());
    };
    let mut points = piece_points();
    points.push(GeoPoint::new(0.0016, 0.006));
    let (found, _) = assert_profile_walks_match_per_edge(
        &reader,
        &points,
        &[MobilityProfile::Emergency],
        configure,
    );
    assert!(found > 100, "{found} routes");
    let (_, compact) = engines(&reader);
    let track = [edge_between(&graph, 121, 24, 5)];
    let taken = route(
        &compact,
        GeoPoint::new(0.002, 0.0),
        GeoPoint::new(0.0016, 0.006),
        MobilityProfile::Emergency,
        configure,
    )
    .expect("route");
    assert!(
        taken.edge_ids.iter().any(|id| track.contains(id)),
        "{:?}",
        taken.edge_ids
    );
}

/// Engines over the fixture's graph: one package without a CCH order (bounded
/// by the straight line alone), one with it (what the compiler writes by
/// default), its potentials prepared.
fn straight_line_and_potential_engines(
    fixture: &Fixture,
    path: &Path,
) -> (RoutingEngine, RoutingEngine) {
    let build = |name: &str, cch_order: bool| {
        let file = path.with_file_name(name);
        let options = PackageBuildOptions {
            cch_order,
            ..PackageBuildOptions::default()
        };
        wayfarer_compiler::package_build::write_package(&fixture.graph, None, &file, &options)
            .unwrap();
        RoutingPackage::open(&file).unwrap()
    };
    let straight = build("straight-line.mrt", false);
    assert!(straight.file.view().cch_rank.is_empty());
    assert!(straight.prepare_potentials().is_none());
    let potentials = build("potentials.mrt", true);
    let stats = potentials.prepare_potentials().expect("potentials");
    assert_eq!(
        stats.column_count,
        potentials.file.view().edge_cost.len() / potentials.counts().edge_count
    );
    (
        RoutingEngine::from_package(Arc::new(straight)),
        RoutingEngine::from_package(Arc::new(potentials)),
    )
}

#[test]
fn cch_potentials_are_exact_node_distances() {
    let dir = tempdir().unwrap();
    let graphs = [
        ("grid", grid_graph()),
        ("costing", costing_grid_graph()),
        ("junction", junction_graph()),
        ("restricted", randomly_restricted_grid(7)),
    ];
    for (name, graph) in graphs {
        let fixture = persist(&graph, &dir.path().join(format!("{name}.mrt")));
        let package = &fixture.package;
        let view = package.file.view();
        let potentials = package.cch.prepare(&view).expect("potentials").clone();
        let (pairs, mismatches) = cch::exactness_mismatches(&view, &potentials);
        assert!(pairs > 0, "{name}");
        assert_eq!(mismatches, 0, "{name}: {mismatches} of {pairs} pairs");
    }
}

/// A potential scratch keeps its memo for the next potential with the same
/// metric, direction and targets, and only for it: each potential reads the
/// same values, bit for bit, as one on a fresh scratch.
#[test]
fn cch_potentials_reuse_a_memo_only_for_the_same_query() {
    use std::cell::RefCell;
    let dir = tempdir().unwrap();
    let fixture = persist(&costing_grid_graph(), &dir.path().join("costing.mrt"));
    let package = &fixture.package;
    let view = package.file.view();
    let potentials = package.cch.prepare(&view).expect("potentials").clone();
    let topology = &potentials.topology;
    let metrics = &potentials.metrics;
    assert!(metrics.len() >= 2);
    let (a, b) = (0u32, view.node_count as u32 - 1);
    let queries = [
        (0, false, vec![(a, 0.0)]),
        (0, false, vec![(a, 0.0)]),
        (0, true, vec![(a, 0.0)]),
        (1, true, vec![(a, 0.0)]),
        (1, true, vec![(a, 0.0), (b, 2.5)]),
        (1, true, vec![(a, 0.0), (b, 3.5)]),
        (0, false, vec![(b, 0.0)]),
        (0, false, vec![(a, 0.0)]),
    ];
    let shared = RefCell::new(cch::PotentialScratch::default());
    for (metric, reversed, targets) in &queries {
        let metric = &metrics[*metric];
        let fresh = RefCell::new(cch::PotentialScratch::default());
        let expected = cch::CchPotential::new(topology, metric, targets, *reversed, 0.5, &fresh);
        let actual = cch::CchPotential::new(topology, metric, targets, *reversed, 0.5, &shared);
        for node in 0..view.node_count as u32 {
            assert_eq!(
                actual.get(node).to_bits(),
                expected.get(node).to_bits(),
                "{reversed} {targets:?}: node {node}"
            );
        }
    }
}

/// Paged potential slots read the same values as dense ones, and freeing
/// the pages drops the memo: the next potential for the same query is
/// computed again rather than read from entries no slot names any more.
#[test]
fn cch_paged_potential_slots_match_dense_and_forget_on_release() {
    use std::cell::RefCell;
    let dir = tempdir().unwrap();
    let fixture = persist(&costing_grid_graph(), &dir.path().join("costing.mrt"));
    let package = &fixture.package;
    let view = package.file.view();
    let potentials = package.cch.prepare(&view).expect("potentials").clone();
    let topology = &potentials.topology;
    let metric = &potentials.metrics[0];
    let targets = vec![(0u32, 0.0)];
    let paged = RefCell::new(cch::PotentialScratch::paged());
    for release in [false, true, false] {
        if release {
            paged.borrow_mut().release();
            assert_eq!(paged.borrow().page_bytes(), 0);
        }
        let dense = RefCell::new(cch::PotentialScratch::default());
        let expected = cch::CchPotential::new(topology, metric, &targets, false, 0.5, &dense);
        let actual = cch::CchPotential::new(topology, metric, &targets, false, 0.5, &paged);
        for node in 0..view.node_count as u32 {
            assert_eq!(
                actual.get(node).to_bits(),
                expected.get(node).to_bits(),
                "release {release}: node {node}"
            );
        }
        assert!(paged.borrow().page_bytes() > 0);
    }
}

#[test]
fn via_candidates_match_the_reference_listing() {
    let dir = tempdir().unwrap();
    let graphs = [
        ("grid", grid_graph()),
        ("costing", costing_grid_graph()),
        ("junction", junction_graph()),
        ("restricted", randomly_restricted_grid(7)),
    ];
    let mut listed = 0usize;
    for (name, graph) in graphs {
        let fixture = persist(&graph, &dir.path().join(format!("{name}.mrt")));
        let package = &fixture.package;
        let view = package.file.view();
        let potentials = package.cch.prepare(&view).expect("potentials").clone();
        let topology = &potentials.topology;
        for (column, metric) in potentials.metrics.iter().enumerate() {
            let costs = &view.edge_cost[column * view.edge_count..(column + 1) * view.edge_count];
            for source in 0..view.node_count as u32 {
                for target in 0..view.node_count as u32 {
                    // Both trees in one sweep: the two sweeps' values, bit for bit.
                    let both = topology.both_to_all(metric, source, target);
                    let from = topology.one_to_all(metric, source, false);
                    let to = topology.one_to_all(metric, target, true);
                    assert!(
                        both.iter()
                            .zip(from.iter().zip(&to))
                            .all(|(pair, (f, t))| pair[0].to_bits() == f.to_bits()
                                && pair[1].to_bits() == t.to_bits()),
                        "{name}: trees {source} -> {target}"
                    );
                    let excluded = [
                        rustc_hash::FxHashSet::default(),
                        (0..view.edge_count as u32).step_by(5).collect(),
                    ];
                    for excluded in &excluded {
                        for (max_ratio, min_plateau) in [(1.6, 0.15), (3.0, 0.0)] {
                            let expected = cch::via_candidates_reference(
                                &view,
                                topology,
                                metric,
                                costs,
                                source,
                                target,
                                max_ratio,
                                min_plateau,
                                excluded,
                            );
                            let (mut candidates, trees) = cch::via_plateaus(
                                &view,
                                topology,
                                metric,
                                costs,
                                source,
                                target,
                                max_ratio,
                                min_plateau,
                                excluded,
                            );
                            if let Some(trees) = trees {
                                candidates.extend(cch::via_singles(
                                    &view, topology, costs, &trees, excluded,
                                ));
                            }
                            assert_eq!(candidates, expected, "{name}: {source} -> {target}");
                            listed += candidates.len();
                        }
                    }
                }
            }
        }
    }
    assert!(listed > 0);
}

#[test]
fn routes_bounded_by_cch_potentials_equal_routes_bounded_by_the_straight_line() {
    let dir = tempdir().unwrap();
    let path = dir.path().join("costing.mrt");
    let graph = costing_grid_graph();
    let fixture = persist(&graph, &path);
    let (straight, potentials) = straight_line_and_potential_engines(&fixture, &path);
    let points = grid_points()
        .into_iter()
        .chain(many_grid_targets().into_iter().step_by(30))
        .collect::<Vec<_>>();
    let mut per_edge = crate::RouteOptions::default();
    apply_per_edge_options(&graph, &mut per_edge);
    let overlay = grid_overlay(&graph);
    let (mut compared, mut straight_visited, mut potential_visited) = (0usize, 0usize, 0usize);
    for (name, costing) in costing_option_sets() {
        for variant in [
            "plain",
            "per-edge",
            "scaled",
            "alternatives",
            "overlay",
            "overlay-per-edge",
        ] {
            if matches!(variant, "alternatives" | "overlay" | "overlay-per-edge")
                && name != "default"
            {
                continue;
            }
            // "scaled": the per-edge options past the gate limit, bounded by
            // scaling instead of gates.
            search::gates::force_gate_limit((variant == "scaled").then_some(0));
            for profile in ALL_TEST_PROFILES {
                for origin in &points {
                    for destination in &points {
                        if origin == destination {
                            continue;
                        }
                        let request = || {
                            let mut request = LegRequest::new(*origin, *destination, profile);
                            if matches!(variant, "per-edge" | "scaled" | "overlay-per-edge") {
                                request.options = per_edge.clone();
                            }
                            if variant.starts_with("overlay") {
                                request.options.dynamic_overlay = Some(overlay.clone());
                            }
                            request.options.max_correlation_distance_m = 60;
                            request.options.costing = costing;
                            if variant == "alternatives" {
                                request.options.alternatives.requested_alternative_count = 2;
                            }
                            request
                        };
                        let context =
                            format!("{name} {variant} {profile:?} {origin:?} -> {destination:?}");
                        let expected = straight.route_leg(request());
                        let actual = potentials.route_leg(request());
                        match (expected, actual) {
                            (Ok(expected), Ok(actual)) => {
                                compared += 1;
                                straight_visited += expected.diagnostics.visited_edge_count;
                                potential_visited += actual.diagnostics.visited_edge_count;
                                assert!(
                                    (expected.cost - actual.cost).abs()
                                        <= 1e-9 * expected.cost.max(1.0),
                                    "{context}: straight line {} vs potentials {}",
                                    expected.cost,
                                    actual.cost
                                );
                                assert_eq!(expected.edge_ids, actual.edge_ids, "{context}");
                                // Alternatives differ by design: with potentials
                                // they go through via edges (see
                                // `via_alternatives_are_valid_routes_within_the_policy`).
                            }
                            (Err(expected), Err(actual)) => {
                                // Visited counts differ by design.
                                assert_eq!(
                                    std::mem::discriminant(&expected.error),
                                    std::mem::discriminant(&actual.error),
                                    "{context}: {:?} vs {:?}",
                                    expected.error,
                                    actual.error
                                );
                            }
                            (expected, actual) => panic!(
                                "{context}: straight line {:?} vs potentials {:?}",
                                expected.map(|result| result.cost).map_err(|f| f.error),
                                actual.map(|result| result.cost).map_err(|f| f.error)
                            ),
                        }
                    }
                }
            }
        }
    }
    search::gates::force_gate_limit(None);
    assert!(compared > 500, "only {compared} routes");
    // Under `WAYFARER_POTENTIALS=off` both engines bound with the straight
    // line.
    assert!(
        crate::debug::potentials_off() || potential_visited < straight_visited,
        "potentials visited {potential_visited} edges, the straight line {straight_visited}"
    );
}

#[test]
fn routes_under_random_complex_restrictions_are_unchanged_by_cch_potentials() {
    let dir = tempdir().unwrap();
    let points = grid_points()
        .into_iter()
        .chain(many_grid_targets().into_iter().step_by(5))
        .collect::<Vec<_>>();
    for seed in 1..=4 {
        let path = dir.path().join(format!("restricted-{seed}.mrt"));
        let fixture = persist(&randomly_restricted_grid(seed), &path);
        let (straight, potentials) = straight_line_and_potential_engines(&fixture, &path);
        for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
            for origin in &points {
                for destination in &points {
                    if origin == destination {
                        continue;
                    }
                    let context = format!("seed {seed} {profile:?} {origin:?} -> {destination:?}");
                    let expected = route(&straight, *origin, *destination, profile, |_| {});
                    let actual = route(&potentials, *origin, *destination, profile, |_| {});
                    assert_eq!(expected.is_some(), actual.is_some(), "{context}");
                    let (Some(expected), Some(actual)) = (expected, actual) else {
                        continue;
                    };
                    assert!(
                        (expected.cost - actual.cost).abs() <= 1e-9 * expected.cost.max(1.0),
                        "{context}: straight line {} vs potentials {}",
                        expected.cost,
                        actual.cost
                    );
                    assert_eq!(expected.edge_ids, actual.edge_ids, "{context}");
                }
            }
        }
    }
}

#[test]
fn matrices_bounded_by_cch_potentials_equal_matrices_bounded_by_the_straight_line() {
    use crate::matrix::{MatrixMode, test_mode};
    let dir = tempdir().unwrap();
    let path = dir.path().join("costing.mrt");
    let graph = costing_grid_graph();
    let fixture = persist(&graph, &path);
    let (straight, potentials) = straight_line_and_potential_engines(&fixture, &path);
    let sources = grid_points();
    let targets = many_grid_targets()
        .into_iter()
        .step_by(4)
        .collect::<Vec<_>>();
    let mut per_edge = crate::RouteOptions::default();
    apply_per_edge_options(&graph, &mut per_edge);
    let overlay = grid_overlay(&graph);
    let mut cells = 0usize;
    for (name, costing) in costing_option_sets() {
        for variant in ["plain", "per-edge", "overlay"] {
            for profile in ALL_TEST_PROFILES {
                for mode in [MatrixMode::Pairwise, MatrixMode::Rows] {
                    let request = || {
                        let mut request = crate::RouteMatrixRequest::new(
                            sources.clone(),
                            targets.clone(),
                            profile,
                        );
                        if variant == "per-edge" {
                            request.options = per_edge.clone();
                        }
                        if variant == "overlay" {
                            request.options.dynamic_overlay = Some(overlay.clone());
                        }
                        request.options.max_correlation_distance_m = 60;
                        request.options.costing = costing;
                        request
                    };
                    test_mode::set(Some(mode));
                    let expected = straight.matrix(request());
                    let actual = potentials.matrix(request());
                    test_mode::set(None);
                    let context = format!("{name} {variant} {profile:?} {mode:?}");
                    assert_eq!(expected.cells.len(), actual.cells.len(), "{context}");
                    for (expected, actual) in expected.cells.iter().zip(&actual.cells) {
                        let cell = format!(
                            "{context} cell {} -> {}",
                            expected.source_index, expected.target_index
                        );
                        assert_eq!(expected.status, actual.status, "{cell}");
                        assert_eq!(expected.failure_reason, actual.failure_reason, "{cell}");
                        if let (Some(a), Some(b)) = (expected.cost, actual.cost) {
                            assert!((a - b).abs() <= 1e-9 * a.max(1.0), "{cell}: {a} vs {b}");
                            cells += 1;
                        }
                        // An overlay path may list an edge once per fragment
                        // it runs along: equal paths, other stops.
                        let mut expected_edges = expected.edge_ids.clone();
                        let mut actual_edges = actual.edge_ids.clone();
                        expected_edges.dedup();
                        actual_edges.dedup();
                        assert_eq!(expected_edges, actual_edges, "{cell}");
                    }
                }
            }
        }
    }
    assert!(cells > 1_000, "only {cells} cells");
}

/// Via-edge alternatives (packages with CCH potentials): every alternative
/// is a connected path (from any of the endpoints' candidates) that breaks no
/// restriction, differs from the primary and the other alternatives, and
/// keeps to the policy's sharing limit and (up to approach costs, which the
/// policy counts and reported costs leave out) its cost limit.
#[test]
fn via_alternatives_are_valid_routes_within_the_policy() {
    let dir = tempdir().unwrap();
    let path = dir.path().join("costing.mrt");
    let graph = costing_grid_graph();
    let fixture = persist(&graph, &path);
    let (_, potentials) = straight_line_and_potential_engines(&fixture, &path);
    let view = potentials.route_graph_view().routing_package().view();
    let points = grid_points()
        .into_iter()
        .chain(many_grid_targets().into_iter().step_by(13))
        .collect::<Vec<_>>();
    let policy = crate::RouteAlternativePolicy {
        requested_alternative_count: 2,
        ..Default::default()
    }
    .normalized();
    let mut found = 0usize;
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        for origin in &points {
            for destination in &points {
                if origin == destination {
                    continue;
                }
                let Some(primary) = route(&potentials, *origin, *destination, profile, |request| {
                    request.options.alternatives = policy;
                }) else {
                    continue;
                };
                let context = format!("{profile:?} {origin:?} -> {destination:?}");
                let mut seen = vec![primary.edge_ids.clone()];
                for alternative in &primary.alternatives {
                    found += 1;
                    assert!(
                        !seen.contains(&alternative.edge_ids),
                        "{context}: duplicate"
                    );
                    seen.push(alternative.edge_ids.clone());
                    let edges = alternative
                        .edge_ids
                        .iter()
                        .map(|id| view.edge_for_id(*id).expect("package edge"))
                        .collect::<Vec<_>>();
                    for pair in edges.windows(2) {
                        assert_eq!(
                            view.edge_target[pair[0] as usize],
                            view.edge_source(pair[1] as usize),
                            "{context}: not connected"
                        );
                    }
                    assert!(
                        view.restriction_violations(&edges, profile_bit(profile))
                            .is_empty(),
                        "{context}: breaks a restriction"
                    );
                    assert!(
                        alternative.cost
                            <= policy.max_cost_ratio * primary.cost * (1.0 + 1e-9) + 1.0,
                        "{context}: cost {} against {}",
                        alternative.cost,
                        primary.cost
                    );
                    assert!(
                        alternative.shared_edge_ratio <= policy.max_shared_edge_ratio + 1e-9,
                        "{context}"
                    );
                }
            }
        }
    }
    assert!(found > 20, "only {found} alternatives");
}

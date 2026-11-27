//! Reach prediction, team plans and coverage on a footpath grid.

use std::sync::Arc;

use wayfarer::{GeoPoint, NavigationPositionSample, RoutingEngine, RoutingPackage};
use wayfarer_road_graph::{
    DirectionalAccess, RoadGraph, RoadGraphBuilder, RoadType, RoutableWay, enhance_road_graph,
};
use wayfarer_sar::{
    CoverageRequest, Detection, DistanceRing, EdgeEffort, EffortArea, NamedPoint, PersonFactors,
    PersonPercentile, ReachRequest, ReachResult, RouteTarget, SarError, SarTeam, SarZone,
    SarZoneKind, SearchEffort, SearchSegment, Stopping, TeamPlanRequest, TeamTrack, apply_coverage,
    plan_teams, predict_reach,
};

/// Grid spacing in degrees (about 222 m at the equator).
const STEP: f64 = 0.002;
/// Nodes per side.
const SIDE: usize = 13;
const CENTRE: usize = SIDE / 2;

fn node(row: usize, col: usize) -> (f64, f64) {
    (row as f64 * STEP, col as f64 * STEP)
}

fn node_id(row: usize, col: usize) -> i64 {
    (row * 100 + col + 1) as i64
}

fn footway(id: i64, a: (usize, usize), b: (usize, usize)) -> RoutableWay {
    RoutableWay {
        osm_way_id: id,
        points: vec![node(a.0, a.1), node(b.0, b.1)],
        source_node_ids: vec![Some(node_id(a.0, a.1)), Some(node_id(b.0, b.1))],
        road_type: RoadType::Path,
        directional_access: DirectionalAccess::Both,
        highway: Some("footway".into()),
        ..Default::default()
    }
}

/// A `SIDE` x `SIDE` grid of footways.
fn grid() -> RoadGraph {
    let mut builder = RoadGraphBuilder::new("grid");
    let mut id = 1;
    for row in 0..SIDE {
        for col in 0..SIDE {
            if col + 1 < SIDE {
                builder.push_way(footway(id, (row, col), (row, col + 1)));
                id += 1;
            }
            if row + 1 < SIDE {
                builder.push_way(footway(id, (row, col), (row + 1, col)));
                id += 1;
            }
        }
    }
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    graph
}

fn engine() -> RoutingEngine {
    let (bytes, _) = wayfarer_compiler::package_build::package_bytes(
        &grid(),
        &wayfarer_compiler::package_build::PackageBuildOptions::default(),
    )
    .unwrap();
    RoutingEngine::from_package(Arc::new(RoutingPackage::from_bytes(&bytes).unwrap()))
}

fn point(row: f64, col: f64) -> GeoPoint {
    GeoPoint::new(row * STEP, col * STEP)
}

fn centre() -> GeoPoint {
    point(CENTRE as f64, CENTRE as f64)
}

/// A box over grid rows and columns (inclusive, padded by a quarter step).
fn grid_box(rows: (f64, f64), cols: (f64, f64)) -> Vec<GeoPoint> {
    let pad = 0.25;
    vec![
        point(rows.0 - pad, cols.0 - pad),
        point(rows.0 - pad, cols.1 + pad),
        point(rows.1 + pad, cols.1 + pad),
        point(rows.1 + pad, cols.0 - pad),
    ]
}

fn reach_request(minutes: f64) -> ReachRequest {
    let mut request = ReachRequest::new(centre());
    request.elapsed_s = Some(minutes * 60.0);
    request
}

fn total(reach: &ReachResult) -> f64 {
    reach
        .probability
        .edges
        .iter()
        .map(|edge| edge.probability)
        .sum::<f64>()
        + reach.probability.rest_of_world
        + reach.probability.off_network
        + reach.probability.unlisted
}

/// Longitude of an edge's middle, in grid columns.
fn edge_col(engine: &RoutingEngine, edge_id: u64) -> f64 {
    let record = engine.route_graph_view().edge_by_id(edge_id).unwrap();
    let geometry = &record.shared_info.geometry;
    (geometry[0].1 + geometry[geometry.len() - 1].1) / 2.0 / STEP
}

#[test]
fn reach_bands_nest_and_probability_adds_up() {
    let engine = engine();
    let mut request = reach_request(15.0);
    request.stopping = Stopping::Never;
    let reach = predict_reach(&engine, &request).unwrap();
    assert_eq!(reach.areas.len(), 3);
    let [fast, typical, slow] = [&reach.areas[0], &reach.areas[1], &reach.areas[2]];
    assert_eq!(
        (fast.person, typical.person, slow.person),
        (
            PersonPercentile::Fast,
            PersonPercentile::Typical,
            PersonPercentile::Slow
        )
    );
    assert!(fast.edge_count > typical.edge_count && typical.edge_count > slow.edge_count);
    assert!(fast.max_radius_m > typical.max_radius_m && typical.max_radius_m > slow.max_radius_m);
    assert!(
        fast.edge_ids
            .iter()
            .all(|key| !typical.edge_ids.contains(key) || fast.edge_ids.contains(key))
    );
    // The built-in pace model walks a footpath at 1,041 mm/s x 0.996, the
    // engine at 5.1 km/h: a ratio of 1.366, so the fast person (P10 0.728)
    // gets 905 engine seconds, about 1.28 km of path.
    let pace = &reach.pace;
    assert_eq!(pace.source, Some(wayfarer::pace::PaceSource::Default));
    let ratio = pace.network_ratio.unwrap();
    assert!((ratio - 1.366).abs() < 0.01, "{ratio}");
    assert!((pace.person_factors.p10 - ratio * 0.728).abs() < 1e-9);
    assert!(!pace.slope_applied);
    assert!((fast.contour_s - 900.0 / pace.person_factors.p10).abs() < 1e-6);
    assert!(
        fast.max_radius_m > 900.0 && fast.max_radius_m < 1_300.0,
        "{}",
        fast.max_radius_m
    );
    assert!((total(&reach) - 1.0).abs() < 1e-9, "{}", total(&reach));
    // The grid is wide enough that only the outer share is beyond the bands.
    assert!((reach.probability.rest_of_world - 0.02).abs() < 1e-6);
    assert!(reach.probability.off_network < 1e-9);
    assert!(!reach.truncated);
    assert!(reach.warnings.is_empty(), "{:?}", reach.warnings);

    // People who stop at any time are found nearer: more mass within the
    // slow person's reach, less beyond the bands.
    let stopping = predict_reach(&engine, &reach_request(15.0)).unwrap();
    assert!((total(&stopping) - 1.0).abs() < 1e-9);
    assert!(stopping.probability.rest_of_world < 0.02);
    assert_eq!(stopping.areas, reach.areas);
    let near = |reach: &ReachResult| {
        reach
            .probability
            .edges
            .iter()
            .filter(|edge| edge.time_s.is_some_and(|time| time < slow.contour_s))
            .map(|edge| edge.probability)
            .sum::<f64>()
    };
    assert!(
        near(&stopping) > 2.0 * near(&reach),
        "{} vs {}",
        near(&stopping),
        near(&reach)
    );
}

#[test]
fn team_pace_model_and_explicit_factors() {
    let engine = engine();
    let base = predict_reach(&engine, &reach_request(15.0)).unwrap();

    // A team calibration that walks 25 % faster on the flat.
    let fast = wayfarer::pace::PaceModel {
        flat_speed_mm_s: 1_301,
        source: wayfarer::pace::PaceSource::Team {
            label: "GOPR Podhale".into(),
            tracks: 12,
            distance_m: 84_000,
            moving_s: 70_000,
            curve_refitted: false,
        },
        ..Default::default()
    };
    let json = serde_json::to_value(&fast).unwrap();
    let mut request: ReachRequest = serde_json::from_value(serde_json::json!({
        "lastKnown": { "lat": centre().lat, "lng": centre().lng },
        "elapsedS": 900,
        "pace": json,
    }))
    .unwrap();
    let team = predict_reach(&engine, &request).unwrap();
    assert!(matches!(
        team.pace.source,
        Some(wayfarer::pace::PaceSource::Team { .. })
    ));
    let ratio = team.pace.network_ratio.unwrap() / base.pace.network_ratio.unwrap();
    assert!((ratio - 1_041.0 / 1_301.0).abs() < 0.01, "{ratio}");
    assert!(team.areas[0].edge_count > base.areas[0].edge_count);

    // An invalid model is refused.
    request.pace.as_mut().unwrap().slope_permille.pop();
    assert!(matches!(
        predict_reach(&engine, &request),
        Err(SarError::InvalidRequest { .. })
    ));

    // Explicit person factors replace the pace model.
    let mut explicit = reach_request(15.0);
    explicit.person_factors = Some(PersonFactors::FLAT_PACE);
    let reach = predict_reach(&engine, &explicit).unwrap();
    assert_eq!(reach.pace.source, None);
    assert!((reach.areas[0].contour_s - 900.0 / 1.03).abs() < 1e-6);
}

#[test]
fn reach_is_deterministic() {
    let engine = engine();
    let mut request = reach_request(20.0);
    request.uncertainty_m = 150.0;
    request.destinations = vec![NamedPoint::new("hut", point(0.0, 0.0))];
    let a = serde_json::to_string(&predict_reach(&engine, &request).unwrap()).unwrap();
    let b = serde_json::to_string(&predict_reach(&engine, &request).unwrap()).unwrap();
    assert_eq!(a, b);
}

#[test]
fn no_go_zone_holds_no_probability_and_blocks_reach() {
    let engine = engine();
    let mut request = reach_request(15.0);
    // Everything east of the centre column (exclusive) is a river bank.
    request.zones = vec![SarZone::new(
        "river",
        SarZoneKind::NoGo,
        grid_box((-1.0, SIDE as f64), (CENTRE as f64 + 0.5, SIDE as f64)),
    )];
    let reach = predict_reach(&engine, &request).unwrap();
    assert!((total(&reach) - 1.0).abs() < 1e-9);
    for edge in &reach.probability.edges {
        assert!(
            edge_col(&engine, edge.edge_id) <= CENTRE as f64 + 0.5,
            "edge {}",
            edge.edge_id
        );
    }
    let open = predict_reach(&engine, &reach_request(15.0)).unwrap();
    assert!(reach.areas[0].edge_count < open.areas[0].edge_count);
}

#[test]
fn avoid_zone_mixes_complying_and_free_searches() {
    let engine = engine();
    let mut request = reach_request(15.0);
    let forest = grid_box((-1.0, SIDE as f64), (CENTRE as f64 + 0.5, SIDE as f64));
    request.zones = vec![SarZone::new("forest", SarZoneKind::Avoid, forest)];
    request.avoid_compliance = 0.8;
    let reach = predict_reach(&engine, &request).unwrap();
    assert!((total(&reach) - 1.0).abs() < 1e-9);
    assert_eq!(reach.complying_areas.len(), 3);
    assert!(reach.complying_areas[0].edge_count < reach.areas[0].edge_count);
    let east = |reach: &ReachResult| {
        reach
            .probability
            .edges
            .iter()
            .filter(|edge| edge_col(&engine, edge.edge_id) > CENTRE as f64 + 0.5)
            .map(|edge| edge.probability)
            .sum::<f64>()
    };
    let inside = east(&reach);
    let open = east(&predict_reach(&engine, &reach_request(15.0)).unwrap());
    // Only the 20 % who ignore the zone go east.
    assert!(inside > 0.0 && inside < 0.3 * open, "{inside} vs {open}");
    assert!(
        reach
            .probability
            .edges
            .iter()
            .filter(|edge| edge_col(&engine, edge.edge_id) > CENTRE as f64 + 0.5)
            .all(|edge| edge.avoid_only)
    );
    // Routes keep out of the zone.
    for route in reach.routes.iter().filter(|route| route.reachable) {
        for edge_id in &route.edge_ids {
            assert!(edge_col(&engine, *edge_id) <= CENTRE as f64 + 0.5);
        }
    }
}

#[test]
fn weight_zone_shifts_probability() {
    let engine = engine();
    let north = grid_box((CENTRE as f64 + 0.5, SIDE as f64), (-1.0, SIDE as f64));
    let mass = |factor: Option<f64>| {
        let mut request = reach_request(15.0);
        if let Some(factor) = factor {
            let mut zone = SarZone::new("drainage", SarZoneKind::Weight, north.clone());
            zone.factor = Some(factor);
            request.zones = vec![zone];
        }
        let reach = predict_reach(&engine, &request).unwrap();
        assert!((total(&reach) - 1.0).abs() < 1e-9);
        let rows = |edge_id: u64| {
            let record = engine.route_graph_view().edge_by_id(edge_id).unwrap();
            let geometry = &record.shared_info.geometry;
            (geometry[0].0 + geometry[geometry.len() - 1].0) / 2.0 / STEP
        };
        reach
            .probability
            .edges
            .iter()
            .filter(|edge| rows(edge.edge_id) > CENTRE as f64 + 0.5)
            .map(|edge| edge.probability)
            .sum::<f64>()
    };
    let plain = mass(None);
    let drawn = mass(Some(3.0));
    let repelled = mass(Some(0.3));
    assert!(drawn > plain * 1.3, "{drawn} vs {plain}");
    assert!(repelled < plain * 0.7, "{repelled} vs {plain}");
}

#[test]
fn routes_reach_the_frontier_and_share_corridors() {
    let engine = engine();
    let mut request = reach_request(15.0);
    request.destinations = vec![
        NamedPoint::new("near", point(CENTRE as f64, CENTRE as f64 + 2.0)),
        NamedPoint::new("far", point(0.0, 0.0)),
    ];
    let reach = predict_reach(&engine, &request).unwrap();
    let frontier = reach
        .routes
        .iter()
        .filter(|route| matches!(route.target, RouteTarget::Frontier { .. }))
        .count();
    assert!(frontier >= 8, "{frontier}");
    assert!(reach.routes.iter().all(|route| route.reachable));
    let near = &reach.routes[0];
    let far = &reach.routes[1];
    assert!(near.reach_probability > 0.99, "{}", near.reach_probability);
    // The corner is about 2.7 km of path away: beyond nearly everyone.
    assert!(far.reach_probability < 0.01, "{}", far.reach_probability);
    let arrivals = &near.arrival_elapsed_s;
    assert!(arrivals[&PersonPercentile::Fast] < arrivals[&PersonPercentile::Slow]);
    assert!(!reach.corridors.is_empty());
    assert!(
        reach
            .corridors
            .iter()
            .all(|corridor| corridor.route_count >= 2)
    );
    assert!(
        reach
            .corridors
            .windows(2)
            .all(|pair| pair[0].route_count >= pair[1].route_count)
    );
}

#[test]
fn uncertainty_and_rings() {
    let engine = engine();
    let mut request = reach_request(10.0);
    request.uncertainty_m = 200.0;
    let wide = predict_reach(&engine, &request).unwrap();
    assert_eq!(
        wide.origins.len(),
        9,
        "{:?} {:?}",
        wide.warnings,
        wide.origins
            .iter()
            .map(|o| o.snapped_point)
            .collect::<Vec<_>>()
    );
    let exact = predict_reach(&engine, &reach_request(10.0)).unwrap();
    assert!(wide.areas[0].edge_count > exact.areas[0].edge_count);

    let mut request = reach_request(15.0);
    request.stopping = Stopping::Never;
    request.distance_rings = vec![
        DistanceRing::new(0.5, 400.0),
        DistanceRing::new(0.95, 900.0),
    ];
    let ringed = predict_reach(&engine, &request).unwrap();
    assert!((total(&ringed) - 1.0).abs() < 1e-9);
    // Half the ring weight's 5 % beyond the last ring, half the time model's 2 %.
    assert!((ringed.probability.rest_of_world - (0.5 * 0.05 + 0.5 * 0.02)).abs() < 1e-6);
}

#[test]
fn trimmed_probability_keeps_the_most_probable_edges() {
    let engine = engine();
    let full = predict_reach(&engine, &reach_request(15.0)).unwrap();
    assert_eq!(full.probability.unlisted, 0.0);
    let mut ranked = full.probability.edges.clone();
    ranked.sort_by(|a, b| {
        b.probability
            .total_cmp(&a.probability)
            .then(a.edge_id.cmp(&b.edge_id))
    });

    let mut request = reach_request(15.0);
    request.max_edges = Some(10);
    let top = predict_reach(&engine, &request).unwrap();
    assert_eq!(top.probability.edges.len(), 10);
    assert!((total(&top) - 1.0).abs() < 1e-9, "{}", total(&top));
    let mut expected = ranked[..10].iter().map(|e| e.edge_id).collect::<Vec<_>>();
    expected.sort_unstable();
    let listed = top
        .probability
        .edges
        .iter()
        .map(|e| e.edge_id)
        .collect::<Vec<_>>();
    assert_eq!(listed, expected, "kept in key order");
    let dropped: f64 = ranked[10..].iter().map(|e| e.probability).sum();
    assert!((top.probability.unlisted - dropped).abs() < 1e-12);
    // Only the map is trimmed: the reach and the routes are the same.
    assert_eq!(top.areas, full.areas);
    assert_eq!(top.routes, full.routes);

    let mut floor = reach_request(15.0);
    floor.min_edge_probability = ranked[20].probability;
    let floored = predict_reach(&engine, &floor).unwrap();
    assert!(
        floored
            .probability
            .edges
            .iter()
            .all(|edge| edge.probability >= ranked[20].probability)
    );
    assert!(floored.probability.edges.len() >= 21);
    assert!((total(&floored) - 1.0).abs() < 1e-9);

    // A trimmed map is a valid input to coverage, and its unlisted mass is
    // rescaled with the rest.
    let track = TeamTrack::new(
        "alpha",
        track_along_row(CENTRE as f64, (CENTRE as f64 - 3.0, CENTRE as f64 + 3.0)),
    );
    let coverage =
        apply_coverage(&engine, &CoverageRequest::new(vec![track], top.probability)).unwrap();
    assert!(coverage.probability.unlisted > dropped);

    let mut zero = reach_request(15.0);
    zero.max_edges = Some(0);
    assert!(predict_reach(&engine, &zero).is_err());
    let mut one = reach_request(15.0);
    one.min_edge_probability = 1.0;
    assert!(predict_reach(&engine, &one).is_err());
}

#[test]
fn off_trail_positions_pay_the_walk_to_the_paths() {
    let engine = engine();
    // A last known position 1.5 steps (about 333 m) west of the grid's edge.
    let off = point(CENTRE as f64, -1.5);
    let mut request = ReachRequest::new(off);
    request.elapsed_s = Some(20.0 * 60.0);
    request.destinations = vec![NamedPoint::new("centre", centre())];
    let priced = predict_reach(&engine, &request).unwrap();
    request.off_trail = false;
    let free = predict_reach(&engine, &request).unwrap();

    let pace = wayfarer_costing::profile_access(wayfarer::MobilityProfile::OnFoot).pace_mps;
    let walk_in = 333.0 / pace;
    let first = |reach: &ReachResult| {
        reach
            .probability
            .edges
            .iter()
            .filter_map(|edge| edge.time_s)
            .fold(f64::INFINITY, f64::min)
    };
    assert!(
        first(&priced) > 0.9 * walk_in,
        "{} vs {walk_in}",
        first(&priced)
    );
    assert!(first(&free) < 1.0, "{}", first(&free));
    for (priced, free) in priced.areas.iter().zip(&free.areas) {
        assert!(
            priced.network_length_m < free.network_length_m,
            "{:?}: {} vs {}",
            priced.person,
            priced.network_length_m,
            free.network_length_m
        );
    }
    // Routes run from the requested point, so they include the walk in.
    let to_centre = |reach: &ReachResult| {
        reach
            .routes
            .iter()
            .find(|route| matches!(&route.target, RouteTarget::Destination { id, .. } if id == "centre"))
            .and_then(|route| route.duration_s)
            .unwrap()
    };
    let added = to_centre(&priced) - to_centre(&free);
    assert!(
        (added - walk_in).abs() < 0.1 * walk_in,
        "{added} vs {walk_in}"
    );

    // An explicit offNetwork wins: a radius short of the paths finds none.
    let mut short = ReachRequest::new(off);
    short.elapsed_s = Some(20.0 * 60.0);
    short.options.off_network = Some(wayfarer::access::RouteOffNetworkOptions {
        max_radius_m: Some(100.0),
        ..Default::default()
    });
    assert!(matches!(
        predict_reach(&engine, &short),
        Err(SarError::Route(_))
    ));
}

#[test]
fn reach_polygons_draw_each_person() {
    let engine = engine();
    let plain = predict_reach(&engine, &reach_request(10.0)).unwrap();
    assert!(plain.areas.iter().all(|area| area.polygon.is_none()));
    let mut request = reach_request(10.0);
    request.polygons = Some(wayfarer::RouteIsochronePolygonOptions::default());
    let drawn = predict_reach(&engine, &request).unwrap();
    // Polygons change nothing else.
    assert_eq!(drawn.probability, plain.probability);
    let extent = |area: &wayfarer_sar::ReachArea| {
        let polygon = area.polygon.as_ref().expect("a polygon per person");
        let (mut lat, mut lng) = ((f64::MAX, f64::MIN), (f64::MAX, f64::MIN));
        for ring in polygon.coordinates.iter().flatten() {
            for [x, y] in ring {
                lng = (lng.0.min(*x), lng.1.max(*x));
                lat = (lat.0.min(*y), lat.1.max(*y));
            }
        }
        (lat.1 - lat.0) * (lng.1 - lng.0)
    };
    let sizes = drawn.areas.iter().map(extent).collect::<Vec<_>>();
    // Fast, typical, slow: each reaches less far.
    assert!(sizes[0] > sizes[1] && sizes[1] > sizes[2], "{sizes:?}");
    // The polygon covers the last known position.
    let slow = drawn.areas[2].polygon.as_ref().unwrap();
    let centre = centre();
    assert!(slow.coordinates.iter().flatten().flatten().any(|[x, y]| {
        (x - centre.lng).abs() < 3.0 * STEP && (y - centre.lat).abs() < 3.0 * STEP
    }));
}

#[test]
fn elapsed_time_from_timestamps() {
    let engine = engine();
    let mut request = ReachRequest::new(centre());
    request.last_seen_at_ms = Some(1_700_000_000_000);
    request.at_ms = Some(1_700_000_000_000 + 30 * 60_000);
    request.moving_fraction = 0.5;
    request.speed_factor = 0.9;
    let reach = predict_reach(&engine, &request).unwrap();
    assert_eq!(reach.elapsed_s, 1_800.0);
    assert_eq!(reach.moving_s, 900.0);
    assert!((reach.budget_s - 810.0).abs() < 1e-9);
}

#[test]
fn invalid_requests_fail() {
    let engine = engine();
    let invalid = |request: ReachRequest| {
        matches!(
            predict_reach(&engine, &request),
            Err(SarError::InvalidRequest { .. })
        )
    };
    assert!(invalid(ReachRequest::new(centre())));
    let mut both = reach_request(10.0);
    both.last_seen_at_ms = Some(0);
    both.at_ms = Some(1);
    assert!(invalid(both));
    let mut fraction = reach_request(10.0);
    fraction.moving_fraction = 0.0;
    assert!(invalid(fraction));
    let mut zone = reach_request(10.0);
    zone.zones = vec![SarZone::new(
        "x",
        SarZoneKind::Weight,
        grid_box((0.0, 1.0), (0.0, 1.0)),
    )];
    assert!(invalid(zone));
    let mut rings = reach_request(10.0);
    rings.distance_rings = vec![
        DistanceRing::new(0.5, 900.0),
        DistanceRing::new(0.4, 1_000.0),
    ];
    assert!(invalid(rings));
    let json =
        serde_json::json!({ "lastKnown": { "lat": 0.0, "lng": 0.0 }, "elapsedS": 60, "typo": 1 });
    assert!(serde_json::from_value::<ReachRequest>(json).is_err());
}

fn west_and_east_segments() -> Vec<SearchSegment> {
    vec![
        SearchSegment::new("west", grid_box((2.0, 10.0), (1.0, 4.0))),
        SearchSegment::new("east", grid_box((2.0, 10.0), (8.0, 11.0))),
    ]
}

#[test]
fn teams_go_to_the_nearer_segment_by_probability() {
    let engine = engine();
    let mut reach_request = reach_request(20.0);
    // Make the east twice as likely.
    let mut zone = SarZone::new(
        "east",
        SarZoneKind::Weight,
        grid_box((-1.0, SIDE as f64), (7.0, SIDE as f64)),
    );
    zone.factor = Some(4.0);
    reach_request.zones = vec![zone];
    let reach = predict_reach(&engine, &reach_request).unwrap();

    let mut alpha = SarTeam::new("alpha", point(6.0, 0.0));
    alpha.pace_factor = 1.2;
    let mut bravo = SarTeam::new("bravo", point(6.0, 12.0));
    bravo.start_offset_s = 300.0;
    let mut request = TeamPlanRequest::new(vec![alpha, bravo], west_and_east_segments());
    request.probability = Some(reach.probability.clone());
    request.shift_s = Some(1_800.0);
    let plan = plan_teams(&engine, &request).unwrap();

    assert_eq!(plan.arrivals.len(), 4);
    // Footways: the built-in pace model takes 1.366 times the engine's time.
    assert_eq!(plan.paces.len(), 2);
    assert!((plan.paces[0].pace_ratio - 1.366).abs() < 0.01);
    assert!((plan.paces[0].time_factor - 1.2 * plan.paces[0].pace_ratio).abs() < 1e-9);
    let west = &plan.segments[0];
    let east = &plan.segments[1];
    assert!(east.probability > west.probability);
    assert!(west.network_length_m > 0.0 && east.edge_count > 0);
    assert_eq!(west.first_team.as_deref(), Some("alpha"));
    assert_eq!(east.first_team.as_deref(), Some("bravo"));
    // Bravo leaves 5 minutes late, so it arrives after its travel time plus that.
    assert!(east.first_arrival_s.unwrap() > 300.0);
    assert_eq!(plan.assignments.len(), 2);
    assert_eq!(plan.assignments[0].segment_id, "east");
    assert_eq!(plan.assignments[0].team_id, "bravo");
    assert_eq!(plan.assignments[1].team_id, "alpha");

    assert_eq!(plan.territories.len(), 2);
    let (a, b) = (&plan.territories[0], &plan.territories[1]);
    assert!(a.edge_count > 0 && b.edge_count > 0);
    assert!(a.edge_ids.iter().all(|key| !b.edge_ids.contains(key)));
    for key in &a.edge_ids {
        assert!(edge_col(&engine, *key) < 8.0, "alpha holds edge {key}");
    }
    assert!(plan.warnings.is_empty(), "{:?}", plan.warnings);
}

#[test]
fn off_trail_team_starts_pay_the_walk_in() {
    let engine = engine();
    // A team starting about 333 m west of the grid.
    let team = SarTeam::new("alpha", point(6.0, -1.5));
    let mut request = TeamPlanRequest::new(vec![team], west_and_east_segments());
    let priced = plan_teams(&engine, &request).unwrap();
    request.off_trail = false;
    let free = plan_teams(&engine, &request).unwrap();
    let pace = wayfarer_costing::profile_access(wayfarer::MobilityProfile::OnFoot).pace_mps;
    let walk_in = 333.0 / pace * priced.paces[0].time_factor;
    for (priced, free) in priced.arrivals.iter().zip(&free.arrivals) {
        let added = priced.arrival_s.unwrap() - free.arrival_s.unwrap();
        assert!(
            (added - walk_in).abs() < 0.15 * walk_in,
            "{}: {added} vs {walk_in}",
            priced.segment_id
        );
    }
}

fn track_along_row(row: f64, cols: (f64, f64)) -> Vec<NavigationPositionSample> {
    let mut samples = Vec::new();
    let mut col = cols.0;
    let mut time = 1_700_000_000_000_i64;
    while col <= cols.1 + 1e-9 {
        samples.push(NavigationPositionSample {
            position: point(row, col),
            heading_degrees: Some(90.0),
            speed_meters_per_second: Some(1.3),
            sampled_at_ms: Some(time),
            accuracy_meters: Some(5.0),
            provider: None,
        });
        col += 0.25;
        time += 45_000;
    }
    samples
}

#[test]
fn coverage_lowers_searched_probability() {
    let engine = engine();
    let reach = predict_reach(&engine, &reach_request(15.0)).unwrap();
    let track = TeamTrack::new(
        "alpha",
        track_along_row(CENTRE as f64, (CENTRE as f64 - 3.0, CENTRE as f64 + 3.0)),
    );
    let mut request = CoverageRequest::new(vec![track], reach.probability.clone());
    request.segments = vec![
        SearchSegment::new(
            "middle",
            grid_box(
                (CENTRE as f64, CENTRE as f64),
                (CENTRE as f64 - 3.0, CENTRE as f64 + 3.0),
            ),
        ),
        SearchSegment::new("corner", grid_box((0.0, 2.0), (0.0, 2.0))),
    ];
    let coverage = apply_coverage(&engine, &request).unwrap();
    assert_eq!(coverage.tracks[0].failure, None);
    assert!(
        coverage.tracks[0].covered_length_m > 1_000.0,
        "{}",
        coverage.tracks[0].covered_length_m
    );
    assert!(
        coverage.covered_edges.len() >= 5,
        "{}",
        coverage.covered_edges.len()
    );
    assert!(
        coverage
            .covered_edges
            .iter()
            .all(|edge| (edge.pod - 0.8).abs() < 1e-9 || edge.passes > 1)
    );
    assert!(coverage.probability_of_success > 0.0 && coverage.probability_of_success < 1.0);
    let updated = &coverage.probability;
    let sum = updated
        .edges
        .iter()
        .map(|edge| edge.probability)
        .sum::<f64>()
        + updated.rest_of_world
        + updated.off_network
        + updated.unlisted;
    assert!((sum - 1.0).abs() < 1e-9, "{sum}");
    assert!(updated.rest_of_world > reach.probability.rest_of_world);
    let middle = &coverage.segments[0];
    // The row's six edges of the box's 20 (it also cuts the 14 crossing paths).
    assert!(middle.coverage > 0.25, "{}", middle.coverage);
    assert!(middle.pod > 0.1, "{}", middle.pod);
    assert!(middle.probability_after < middle.probability_before);
    let corner = &coverage.segments[1];
    assert_eq!(corner.covered_length_m, 0.0);
    assert!(corner.probability_after >= corner.probability_before);
}

#[test]
fn host_effort_counts_as_a_search() {
    let engine = engine();
    let reach = predict_reach(&engine, &reach_request(15.0)).unwrap();
    // A drone's video grid in JSON, as a host sends it: one cell a whole grid
    // step wide over the centre row's middle, seen for 2 tau, and the cell
    // beside it never seen.
    let corner = point(CENTRE as f64 - 0.5, CENTRE as f64 - 0.5);
    let effort: SearchEffort = serde_json::from_value(serde_json::json!({
        "sourceId": "drone-1",
        "tauS": 30.0,
        "area": {
            "kind": "grid",
            "southWest": { "lat": corner.lat, "lng": corner.lng },
            "cellLatDeg": STEP,
            "cellLngDeg": STEP,
            "cells": [{ "row": 0, "col": 0, "seenS": 60.0 }, { "row": 0, "col": 1, "pod": 0.0 }],
        },
    }))
    .unwrap();
    let mut request = CoverageRequest::new(Vec::new(), reach.probability.clone());
    request.efforts = vec![effort];
    request.segments = vec![SearchSegment::new(
        "middle",
        grid_box(
            (CENTRE as f64, CENTRE as f64),
            (CENTRE as f64 - 3.0, CENTRE as f64 + 3.0),
        ),
    )];
    let coverage = apply_coverage(&engine, &request).unwrap();
    let seen = 1.0 - libm::exp(-2.0);
    let report = &coverage.efforts[0];
    assert_eq!(report.source_id, "drone-1");
    // Four edges meet at the centre node, each half inside the cell.
    assert_eq!(report.edge_count, 4, "{report:?}");
    assert!((report.mean_pod - seen / 2.0).abs() < 0.02, "{report:?}");
    for edge in &coverage.covered_edges {
        assert_eq!(edge.passes, 0);
        assert!((edge.pod - edge.effort_pod).abs() < 1e-12);
    }
    assert!(coverage.probability_of_success > 0.0);
    assert!(coverage.segments[0].probability_after < coverage.segments[0].probability_before);

    // Effort and a track over the same edge are two searches: misses multiply.
    let key = coverage.covered_edges[0].edge_id;
    let effort_pod = coverage.covered_edges[0].pod;
    let mut both = CoverageRequest::new(
        vec![TeamTrack::new(
            "alpha",
            track_along_row(CENTRE as f64, (CENTRE as f64 - 3.0, CENTRE as f64 + 3.0)),
        )],
        reach.probability.clone(),
    );
    both.efforts = vec![SearchEffort::new(
        "sweep",
        EffortArea::Edges {
            edges: vec![EdgeEffort::new(key, Detection::Pod(effort_pod))],
        },
    )];
    let combined = apply_coverage(&engine, &both).unwrap();
    let edge = combined
        .covered_edges
        .iter()
        .find(|edge| edge.edge_id == key)
        .unwrap();
    let track_pod = if edge.passes > 0 {
        1.0 - (0..edge.passes).fold(1.0, |miss, _| miss * 0.2)
    } else {
        0.0
    };
    let expected = 1.0 - (1.0 - track_pod) * (1.0 - effort_pod);
    assert!((edge.pod - expected).abs() < 1e-12, "{edge:?}");

    // A polygon effort: every edge inside takes its detection.
    let mut area = CoverageRequest::new(Vec::new(), reach.probability.clone());
    area.efforts = vec![SearchEffort::new(
        "hasty",
        EffortArea::Polygon {
            polygon: grid_box((0.0, 2.0), (0.0, 2.0)),
            detection: Detection::Pod(0.5),
        },
    )];
    let swept = apply_coverage(&engine, &area).unwrap();
    assert!(swept.efforts[0].edge_count >= 12, "{:?}", swept.efforts[0]);
    assert!(
        swept
            .covered_edges
            .iter()
            .all(|edge| (edge.pod - 0.5).abs() < 1e-12)
    );

    // Nothing to apply, or seconds without a time constant, is invalid.
    assert!(
        apply_coverage(
            &engine,
            &CoverageRequest::new(Vec::new(), reach.probability.clone())
        )
        .is_err()
    );
    let mut no_tau = CoverageRequest::new(Vec::new(), reach.probability);
    no_tau.efforts = vec![SearchEffort::new(
        "cam",
        EffortArea::Edges {
            edges: vec![EdgeEffort::new(key, Detection::SeenS(10.0))],
        },
    )];
    assert!(apply_coverage(&engine, &no_tau).is_err());
}

#[test]
fn geojson_has_every_layer() {
    let engine = engine();
    let mut request = reach_request(15.0);
    request.route_sectors = 8;
    request.polygons = Some(wayfarer::RouteIsochronePolygonOptions::default());
    let reach = predict_reach(&engine, &request).unwrap();
    let collection = wayfarer_sar::geojson::reach_feature_collection(&engine, &reach);
    let features = collection["features"].as_array().unwrap();
    let count = |kind: &str| {
        features
            .iter()
            .filter(|feature| feature["properties"]["kind"] == kind)
            .count()
    };
    assert_eq!(count("last_known"), 1);
    assert_eq!(count("radius"), 3);
    assert_eq!(count("reach"), 3);
    assert_eq!(count("probability"), reach.probability.edges.len());
    assert!(count("route") >= 4);
    assert_eq!(count("corridor"), reach.corridors.len());
}

//! Workflow overlays that lower costs or open roads (`discount`, `open`),
//! their time windows and precedence, overlay sets, and the SAR option's
//! typed failures.

use wayfarer::{
    GeoPoint, MobilityProfile, RouteEdgeRange, RouteOptions, RouteSarOptions, RoutingEngine,
    SarRelaxationKind,
};
use wayfarer::{RouteLegMode, RouteOverlay, RouteOverlayGeometryKind, RouteOverlayPolicy};
use wayfarer_road_graph::{
    DirectionalAccess, RoadGraph, RoadGraphBuilder, RoadType, RoutableWay, enhance_road_graph,
};
use wayfarer_service::overlay_set::{OverlaySetError, RouteOverlaySet};
use wayfarer_service::{
    RouteWorkflowError, RouteWorkflowOverlayProvenance, RouteWorkflowRequest, RouteWorkflowResult,
    RouteWorkflowSourceContext, RoutingSourceId, compile_route_workflow,
};

const STEP: f64 = 0.002;

fn way(id: i64, a: (f64, f64), b: (f64, f64), nodes: (i64, i64), kind: &str) -> RoutableWay {
    let road_type = match kind {
        "track" => RoadType::Track,
        "footway" => RoadType::Path,
        _ => RoadType::Residential,
    };
    RoutableWay {
        osm_way_id: id,
        points: vec![a, b],
        source_node_ids: vec![Some(nodes.0), Some(nodes.1)],
        road_type,
        directional_access: DirectionalAccess::Both,
        highway: Some(kind.into()),
        ..Default::default()
    }
}

/// A ladder: a residential top row and a track bottom row, four nodes each
/// (`STEP` apart), joined at both ends, and a footway rung in the middle
/// (top node 1 to bottom node 1).
fn ladder() -> RoadGraph {
    let top = |col: usize| (STEP, col as f64 * STEP);
    let bottom = |col: usize| (0.0, col as f64 * STEP);
    let mut builder = RoadGraphBuilder::new("ladder");
    let mut id = 1;
    for col in 0..3 {
        builder.push_way(way(
            id,
            top(col),
            top(col + 1),
            (10 + col as i64, 11 + col as i64),
            "residential",
        ));
        builder.push_way(way(
            id + 1,
            bottom(col),
            bottom(col + 1),
            (20 + col as i64, 21 + col as i64),
            "track",
        ));
        id += 2;
    }
    builder.push_way(way(id, top(0), bottom(0), (10, 20), "residential"));
    builder.push_way(way(id + 1, top(3), bottom(3), (13, 23), "residential"));
    builder.push_way(way(id + 2, top(1), bottom(1), (11, 21), "footway"));
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    graph
}

fn engine() -> RoutingEngine {
    let (bytes, _) = wayfarer_compiler::package_build::package_bytes(
        &ladder(),
        &wayfarer_compiler::package_build::PackageBuildOptions::default(),
    )
    .unwrap();
    RoutingEngine::from_package(std::sync::Arc::new(
        wayfarer::RoutingPackage::from_bytes(&bytes).unwrap(),
    ))
}

fn source() -> RouteWorkflowSourceContext {
    RouteWorkflowSourceContext {
        source_id: RoutingSourceId::new("ladder").unwrap(),
        revision: "v1".to_string(),
        cache_hit: false,
    }
}

fn request(from: GeoPoint, to: GeoPoint, overlays: Vec<RouteOverlay>) -> RouteWorkflowRequest {
    RouteWorkflowRequest {
        route: wayfarer::RouteRequest {
            overlays,
            legs: vec![RouteLegMode::Routed],
            options: RouteOptions::default(),
            ..wayfarer::RouteRequest::new(from, to, MobilityProfile::Wheeled)
        },
        overlay_provenance: RouteWorkflowOverlayProvenance::default(),
        mobility: None,
        overlay_sets: Vec::new(),
    }
}

/// The bottom row, end to end, as a line overlay.
fn bottom_row(policy: RouteOverlayPolicy) -> RouteOverlay {
    RouteOverlay::new(
        RouteOverlayGeometryKind::Line,
        vec![GeoPoint::new(0.0, 0.0), GeoPoint::new(0.0, 3.0 * STEP)],
        policy,
        5,
    )
}

/// The footway rung as a line overlay.
fn rung(policy: RouteOverlayPolicy) -> RouteOverlay {
    RouteOverlay::new(
        RouteOverlayGeometryKind::Line,
        vec![
            GeoPoint::new(STEP * 0.95, STEP),
            GeoPoint::new(STEP * 0.05, STEP),
        ],
        policy,
        5,
    )
}

fn run(engine: &RoutingEngine, request: RouteWorkflowRequest) -> RouteWorkflowResult {
    compile_route_workflow(engine, source(), request).expect("the workflow routes")
}

/// Bottom row, west end to east end: the track itself, or round by the top.
fn along_bottom() -> (GeoPoint, GeoPoint) {
    (
        GeoPoint::new(0.0, STEP * 0.5),
        GeoPoint::new(0.0, STEP * 2.5),
    )
}

/// The rung's two ends, beside the rows' middle nodes.
fn across_rung() -> (GeoPoint, GeoPoint) {
    (
        GeoPoint::new(STEP, STEP * 1.05),
        GeoPoint::new(0.0, STEP * 1.05),
    )
}

fn uses_rung(result: &RouteWorkflowResult) -> bool {
    result
        .route
        .edge_attributions
        .iter()
        .any(|edge| edge.road_type == RoadType::Path)
}

#[test]
fn discount_overlays_lower_cost_and_duration_and_are_reported() {
    let engine = engine();
    let (from, to) = along_bottom();
    let plain = run(&engine, request(from, to, Vec::new()));
    let mut discount = bottom_row(RouteOverlayPolicy::Discount);
    discount.cost_multiplier = Some(0.5);
    let discounted = run(&engine, request(from, to, vec![discount.clone()]));
    assert!(
        discounted.route.cost < plain.route.cost * 0.75,
        "{} vs {}",
        discounted.route.cost,
        plain.route.cost
    );
    assert!(discounted.route.duration_s < plain.route.duration_s);
    assert_eq!(
        discounted.route.overlay_diagnostics.discount_overlay_count,
        1
    );
    assert_eq!(discounted.route.overlay_uses.len(), 1);
    let used = &discounted.route.overlay_uses[0];
    assert_eq!(used.policy, RouteOverlayPolicy::Discount);
    assert_eq!(used.label, "overlay[0]");
    assert!(used.route_length_m > 300.0);
    // Overlapping discounts multiply.
    let twice = run(&engine, request(from, to, vec![discount.clone(), discount]));
    assert!(twice.route.cost < discounted.route.cost * 0.75);
    // A speed override: cost and duration follow the time.
    let mut speed = bottom_row(RouteOverlayPolicy::Discount);
    speed.speed_kph = Some(80.0);
    let fast = run(&engine, request(from, to, vec![speed]));
    assert!(fast.route.duration_s < plain.route.duration_s * 0.5);
    assert!(fast.route.cost < plain.route.cost);
    // A block wins over a discount.
    let mut block = bottom_row(RouteOverlayPolicy::NoGo);
    block.points = vec![
        GeoPoint::new(0.0, STEP * 1.2),
        GeoPoint::new(0.0, STEP * 1.8),
    ];
    let mut discount = bottom_row(RouteOverlayPolicy::Discount);
    discount.cost_multiplier = Some(0.1);
    let blocked = run(&engine, request(from, to, vec![discount, block]));
    assert!(
        blocked
            .route
            .edge_attributions
            .iter()
            .all(|edge| edge.road_type != RoadType::Track
                || edge.distance_m < STEP * 111_000.0 * 0.6),
        "the blocked stretch is not driven"
    );
    // Invalid discounts are rejected.
    let mut invalid = bottom_row(RouteOverlayPolicy::Discount);
    invalid.cost_multiplier = Some(1.5);
    assert!(matches!(
        compile_route_workflow(&engine, source(), request(from, to, vec![invalid])),
        Err(RouteWorkflowError::Route(failure)) if failure.error == wayfarer::RouteError::InvalidRequest { reason: "overlay cost multiplier" }
    ));
    let empty = bottom_row(RouteOverlayPolicy::Discount);
    assert!(compile_route_workflow(&engine, source(), request(from, to, vec![empty])).is_err());
}

/// `open` makes the footway legal for cars, costed by their own model (a
/// footway costs a car its path class factor); a `discount` from the same
/// time makes it the cheap way across, as an operator opening and
/// prioritising a road would.
fn opened(valid_from_ms: Option<i64>, profiles: Vec<MobilityProfile>) -> Vec<RouteOverlay> {
    let mut open = rung(RouteOverlayPolicy::Open);
    open.speed_kph = Some(20.0);
    open.valid_from_ms = valid_from_ms;
    open.profiles = profiles.clone();
    let mut discount = rung(RouteOverlayPolicy::Discount);
    discount.cost_multiplier = Some(0.02);
    discount.valid_from_ms = valid_from_ms;
    discount.profiles = profiles;
    vec![open, discount]
}

#[test]
fn open_overlays_admit_a_closed_road_and_beat_earlier_blocks_only() {
    let engine = engine();
    let (from, to) = across_rung();
    let plain = run(&engine, request(from, to, Vec::new()));
    assert!(!uses_rung(&plain), "cars do not use the footway");
    // A discount alone does not open it.
    assert!(!uses_rung(&run(
        &engine,
        request(from, to, opened(None, Vec::new())[1..].to_vec())
    )));
    let result = run(&engine, request(from, to, opened(None, Vec::new())));
    assert!(uses_rung(&result), "the opened footway is driven");
    assert!(result.route.distance_m < plain.route.distance_m);
    assert_eq!(result.route.overlay_diagnostics.open_overlay_count, 1);
    assert!(
        result
            .route
            .overlay_uses
            .iter()
            .any(|used| used.policy == RouteOverlayPolicy::Open)
    );
    // Not for the profiles it does not list.
    let for_tracked = opened(None, vec![MobilityProfile::Tracked]);
    let skipped = run(&engine, request(from, to, for_tracked));
    assert!(!uses_rung(&skipped));
    assert_eq!(skipped.route.overlay_diagnostics.ignored_overlay_count, 2);
    let with = |overlays: Vec<RouteOverlay>| {
        let mut request = request(from, to, overlays);
        request.route.departure_at_ms = Some(3_000);
        run(&engine, request)
    };
    let block = |valid_from_ms| {
        let mut block = rung(RouteOverlayPolicy::NoGo);
        block.valid_from_ms = valid_from_ms;
        block
    };
    // An open from after a block's start wins over it ...
    let mut overlays = vec![block(Some(1_000))];
    overlays.extend(opened(Some(2_000), Vec::new()));
    assert!(uses_rung(&with(overlays)));
    // ... a block from after the open's start, or from the same time, wins.
    let mut overlays = opened(Some(2_000), Vec::new());
    overlays.push(block(Some(2_500)));
    assert!(!uses_rung(&with(overlays)));
    let mut overlays = opened(None, Vec::new());
    overlays.push(block(None));
    assert!(!uses_rung(&with(overlays)));
}

#[test]
fn overlay_windows_apply_at_the_evaluation_time() {
    let engine = engine();
    let (from, to) = along_bottom();
    let plain = run(&engine, request(from, to, Vec::new()));
    assert!(plain.route.overlay_windows.is_none());
    let mut discount = bottom_row(RouteOverlayPolicy::Discount);
    discount.cost_multiplier = Some(0.5);
    discount.valid_from_ms = Some(10_000);
    discount.valid_to_ms = Some(20_000);
    let at = |departure: i64| {
        let mut request = request(from, to, vec![discount.clone()]);
        request.route.departure_at_ms = Some(departure);
        run(&engine, request)
    };
    let before = at(5_000);
    assert_eq!(before.route.cost, plain.route.cost);
    let windows = before.route.overlay_windows.as_ref().unwrap();
    assert_eq!(windows.evaluated_at_ms, 5_000);
    assert_eq!(
        windows.inactive_overlay_items,
        vec!["overlay[0]".to_string()]
    );
    assert!(!windows.overlay_window_ends_within_route);
    let during = at(10_000);
    assert!(during.route.cost < plain.route.cost);
    assert!(
        during
            .route
            .overlay_windows
            .as_ref()
            .unwrap()
            .inactive_overlay_items
            .is_empty()
    );
    // Departing a second before the window closes, the route outlasts it.
    let late = at(19_000);
    let windows = late.route.overlay_windows.as_ref().unwrap();
    assert!(windows.overlay_window_ends_within_route);
    assert_eq!(windows.ending_within_route, vec!["overlay[0]".to_string()]);
    assert_eq!(at(20_000).route.cost, plain.route.cost);
    // The conditional evaluation time takes precedence over the departure.
    let mut request = request(from, to, vec![discount.clone()]);
    request.route.departure_at_ms = Some(5_000);
    request
        .route
        .options
        .costing
        .conditional_restriction_eval_at_ms = Some(15_000);
    assert!(run(&engine, request).route.cost < plain.route.cost);
    let mut reversed = discount;
    reversed.valid_to_ms = Some(9_000);
    assert!(matches!(
        compile_route_workflow(&engine, source(), self::request(from, to, vec![reversed])),
        Err(RouteWorkflowError::Route(failure)) if failure.error == wayfarer::RouteError::InvalidRequest { reason: "overlay time window" }
    ));
}

fn set_json(set_id: &str, version: u64, dataset_hash: &str, items: &str) -> String {
    format!(
        r#"{{"format": "wayfarer-overlay-set", "formatVersion": 1, "setId": "{set_id}",
            "version": {version}, "createdAtMs": 1762128000000, "author": "team-{set_id}",
            "package": {{"graphId": "ladder", "datasetHash": "{dataset_hash}", "formatVersion": 1}},
            "items": [{items}]}}"#
    )
}

#[test]
fn overlay_sets_bind_to_their_package_merge_deterministically_and_check_their_hash() {
    let engine = engine();
    let hash = engine.package().dataset_hash().to_string();
    let view = engine.route_graph_view();
    // The bottom row's edges, as the bound package numbers them.
    let edges = wayfarer::internals::project_route_overlay_geometry(
        view,
        wayfarer::RouteOverlayGeometryKind::Line,
        &[GeoPoint::new(0.0, 0.0), GeoPoint::new(0.0, 3.0 * STEP)],
        5,
    )
    .edge_ids;
    let edge_json = edges
        .iter()
        .map(|edge| format!(r#"{{"edgeId": {edge}}}"#))
        .collect::<Vec<_>>()
        .join(",");
    let item = |version: u64, multiplier: f64, geometry: bool, bound_edges: bool| {
        let geometry = if geometry {
            format!(
                r#", "geometry": {{"kind": "line", "points": [[0.0, 0.0], [0.0, {}]], "projectionDistanceM": 5}}"#,
                3.0 * STEP
            )
        } else {
            String::new()
        };
        let edges = if bound_edges {
            format!(r#", "edges": [{edge_json}]"#)
        } else {
            String::new()
        };
        format!(
            r#"{{"itemId": "clear-1", "version": {version}, "kind": "discount",
                "costMultiplier": {multiplier}{geometry}{edges}}}"#
        )
    };
    let (from, to) = along_bottom();
    let plain = run(&engine, request(from, to, Vec::new()));
    let with_sets = |sets: Vec<RouteOverlaySet>| {
        let mut request = request(from, to, Vec::new());
        request.overlay_sets = sets;
        run(&engine, request)
    };

    // Bound to this package: the edges are used as they stand.
    let bound =
        RouteOverlaySet::from_json(&set_json("s1", 3, &hash, &item(3, 0.5, false, true))).unwrap();
    let result = with_sets(vec![bound.clone()]);
    assert!(result.route.cost < plain.route.cost);
    assert_eq!(result.overlay_sets[0].bound_item_count, 1);
    assert!(result.overlay_sets[0].bound_to_package);
    let used = &result.route.overlay_uses[0];
    let source = used.source.as_ref().unwrap();
    assert_eq!(
        (source.set_id.as_str(), source.item_id.as_str()),
        ("s1", "clear-1")
    );
    assert_eq!((source.set_version, source.item_version), (3, 3));
    assert_eq!(source.author.as_deref(), Some("team-s1"));
    assert_eq!(used.label, "clear-1");
    // Bound to another package: re-projected from the geometry; an item
    // with only stale edges is skipped and reported.
    let stale = RouteOverlaySet::from_json(&set_json(
        "s2",
        1,
        "0000",
        &format!(
            "{},{}",
            item(1, 0.5, true, true),
            item(1, 0.5, false, true).replace("clear-1", "clear-2")
        ),
    ))
    .unwrap();
    let result = with_sets(vec![stale]);
    assert!(result.route.cost < plain.route.cost);
    let binding = &result.overlay_sets[0];
    assert!(!binding.bound_to_package);
    assert_eq!(binding.reprojected_item_count, 1);
    assert_eq!(binding.skipped_item_ids, vec!["clear-2".to_string()]);

    // Merging: the higher version wins; at equal versions the tombstone,
    // else the greater canonical JSON, in any order.
    let older =
        RouteOverlaySet::from_json(&set_json("s1", 2, &hash, &item(2, 0.2, true, false))).unwrap();
    let merged = RouteOverlaySet::merge(&[older.clone(), bound.clone()])
        .unwrap()
        .unwrap();
    assert_eq!(merged.items[0].cost_multiplier, Some(0.5));
    assert_eq!(merged.version, 3);
    assert_eq!(merged.items[0].author.as_deref(), Some("team-s1"));
    let tombstone = RouteOverlaySet::from_json(&set_json(
        "s1",
        3,
        &hash,
        r#"{"itemId": "clear-1", "version": 3, "deleted": true}"#,
    ))
    .unwrap();
    for sets in [
        [bound.clone(), tombstone.clone()],
        [tombstone.clone(), bound.clone()],
    ] {
        let merged = RouteOverlaySet::merge(&sets).unwrap().unwrap();
        assert!(merged.items[0].deleted);
        let mut request = request(from, to, Vec::new());
        request.overlay_sets = vec![merged];
        let result = run(&engine, request);
        assert_eq!(result.route.cost, plain.route.cost);
        assert_eq!(result.overlay_sets[0].tombstone_count, 1);
    }
    let rival =
        RouteOverlaySet::from_json(&set_json("s1", 3, &hash, &item(3, 0.3, false, true))).unwrap();
    let forward = RouteOverlaySet::merge(&[bound.clone(), rival.clone()])
        .unwrap()
        .unwrap();
    let backward = RouteOverlaySet::merge(&[rival.clone(), bound.clone()])
        .unwrap()
        .unwrap();
    assert_eq!(forward.items, backward.items);
    // Sets with one id are merged in the workflow too.
    let merged_result = with_sets(vec![older, bound.clone()]);
    assert_eq!(merged_result.overlay_sets.len(), 1);
    assert_eq!(
        merged_result.route.cost,
        result_cost(&engine, from, to, &bound)
    );
    assert!(matches!(
        RouteOverlaySet::merge(&[
            bound.clone(),
            RouteOverlaySet::from_json(&set_json("other", 1, &hash, "")).unwrap()
        ]),
        Err(OverlaySetError::DifferentSets(..))
    ));

    // Content hash: written, verified, and a tampered set is refused.
    let text = bound.to_json_with_hash();
    let read = RouteOverlaySet::from_json(&text).unwrap();
    assert_eq!(read.items, bound.items);
    let tampered = text.replace("0.5", "0.05");
    assert_eq!(
        RouteOverlaySet::from_json(&tampered),
        Err(OverlaySetError::ContentHash)
    );
    // Malformed sets are refused.
    assert!(matches!(
        RouteOverlaySet::from_json(&set_json("s1", 1, &hash, &item(1, 1.5, true, false))),
        Err(OverlaySetError::Invalid(_))
    ));
    assert!(matches!(
        RouteOverlaySet::from_json(
            &set_json("s1", 1, &hash, "").replace("wayfarer-overlay-set", "other")
        ),
        Err(OverlaySetError::Invalid("format"))
    ));
}

fn result_cost(engine: &RoutingEngine, from: GeoPoint, to: GeoPoint, set: &RouteOverlaySet) -> f64 {
    let mut request = request(from, to, Vec::new());
    request.overlay_sets = vec![set.clone()];
    run(engine, request).route.cost
}

/// SAR in workflows: R1, R3 and R5 route with a report; non-Wheeled SAR and
/// surcharges below the package minimum are invalid (`sar_unavailable`: see
/// the engine's `sar_relaxations_fail_typed`).
#[test]
fn workflow_sar_requests_fail_typed_or_report_their_relaxations() {
    let engine = engine();
    let (from, to) = along_bottom();
    let with_sar = |configure: fn(&mut RouteSarOptions)| {
        let mut sar = RouteSarOptions::default();
        configure(&mut sar);
        let mut request = request(from, to, Vec::new());
        request.route.options.sar = Some(sar);
        compile_route_workflow(&engine, source(), request)
    };
    let paths =
        with_sar(|sar| sar.relaxations.vehicle_on_paths = Some(Default::default())).unwrap();
    assert_eq!(
        paths.route.sar.unwrap().enabled,
        vec![SarRelaxationKind::VehicleOnPaths]
    );
    assert!(matches!(
        with_sar(|sar| {
            sar.relaxations.vehicle_on_paths = Some(wayfarer::sar::SarVehicleOnPaths {
                factor: 1.5,
                ..Default::default()
            })
        }),
        Err(RouteWorkflowError::Route(failure)) if failure.error == wayfarer::RouteError::InvalidRequest { reason: "sar surcharge below package minimum" }
    ));
    let legal = with_sar(|sar| sar.relaxations.legal_access = Some(Default::default())).unwrap();
    assert_eq!(
        legal.route.sar.unwrap().enabled,
        vec![SarRelaxationKind::LegalAccess]
    );
    assert!(matches!(
        with_sar(|sar| {
            sar.relaxations.wrong_way = Some(wayfarer::sar::SarWrongWay {
                factor: 1.0,
                entry_s: 0.0,
            })
        }),
        Err(RouteWorkflowError::Route(failure)) if failure.error == wayfarer::RouteError::InvalidRequest { reason: "sar surcharge below package minimum" }
    ));
    assert!(matches!(
        with_sar(|sar| {
            sar.relaxations.turn_restrictions =
                Some(wayfarer::sar::SarTurnRestrictions { penalty_s: -5.0 })
        }),
        Err(RouteWorkflowError::Route(failure)) if failure.error == wayfarer::RouteError::InvalidRequest { reason: "sar surcharge below package minimum" }
    ));
    let result =
        with_sar(|sar| sar.relaxations.turn_restrictions = Some(Default::default())).unwrap();
    let report = result.route.sar.unwrap();
    assert_eq!(report.enabled, vec![SarRelaxationKind::TurnRestrictions]);
    assert!(report.uses.is_empty());
    let mut request = request(from, to, Vec::new());
    request.route.profile = MobilityProfile::OnFoot;
    request.route.options.sar = Some(RouteSarOptions::default());
    assert!(matches!(
        compile_route_workflow(&engine, source(), request),
        Err(RouteWorkflowError::Route(failure)) if failure.error == wayfarer::RouteError::InvalidRequest { reason: "sar requires a vehicle profile" }
    ));
    // Explicit-edge overlays validate their ranges.
    let mut bad = bottom_row(RouteOverlayPolicy::Warning);
    bad.points.clear();
    bad.edges = vec![RouteEdgeRange {
        edge_id: 1,
        start_fraction: 0.7,
        end_fraction: 0.2,
    }];
    assert!(compile_route_workflow(&engine, source(), self::request(from, to, vec![bad])).is_err());
}

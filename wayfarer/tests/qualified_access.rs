//! Qualified legal access on a real network: a drive across Kraków's Rynek
//! Główny (`fixtures/krakow-centre.osm.pbf`) reports the square's qualified
//! access, SAR or not.
//!
//! The ring of the square is `highway=living_street` with
//! `motor_vehicle=permissive` and `motor_vehicle:conditional=delivery @ …`;
//! its pedestrian parts are `highway=pedestrian` with
//! `motor_vehicle=permissive`. The package records the tag that
//! decides vehicle access (the SAR legal reason byte), so the route names
//! `motor_vehicle=permissive`, next to the ring's conditional delivery
//! access.
//!
//! A private road is destination-only: a normal drive ends on it and reports
//! its `access=private`. The SAR tests drive the same package with
//! `legal_access` onto a road closed to vehicles, with `wrong_way` against a
//! one-way street and with `vehicle_on_paths` out of the square over
//! pedestrian streets, each reported in `LegResult::sar`.

use std::path::PathBuf;
use std::sync::Arc;

use wayfarer::{
    GeoPoint, LegRequest, MobilityProfile, RouteAccessQualifier, RoutingEngine, RoutingPackage,
};
use wayfarer_compiler::{CancelToken, CompileRequest, compile};

/// The Kraków centre package: `WAYFARER_KRAKOW_CENTRE_PACKAGE`, or compiled
/// from the fixture into `dir`.
fn krakow_centre(dir: &std::path::Path) -> RoutingEngine {
    let path = match std::env::var_os("WAYFARER_KRAKOW_CENTRE_PACKAGE") {
        Some(path) => PathBuf::from(path),
        None => {
            let pbf =
                PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../fixtures/krakow-centre.osm.pbf");
            let path = dir.join("routing.mrt");
            compile(
                &CompileRequest::new(pbf, &path)
                    .graph_id("krakow-centre")
                    .country("PL"),
                &CancelToken::new(),
                |_| {},
            )
            .expect("the Kraków centre fixture compiles");
            path
        }
    };
    RoutingEngine::from_package(Arc::new(RoutingPackage::open(&path).unwrap()))
}

#[test]
fn a_drive_across_rynek_glowny_reports_its_qualified_access() {
    let dir = tempfile::tempdir().unwrap();
    let engine = krakow_centre(dir.path());
    // From the south side of the square's ring, round it, and onto its
    // pedestrian north-west part. (From the middle of the square, as the
    // gate corpus's `krakow-local-core` starts, a car snaps onto the ring,
    // 81 m away, which it leaves over destination-only streets.)
    let request = LegRequest::new(
        GeoPoint::new(50.0610, 19.9373),
        GeoPoint::new(50.0628, 19.9368),
        MobilityProfile::Wheeled,
    );
    let route = engine.route_leg(request.clone()).unwrap();
    assert!(route.sar.is_none(), "no SAR without the option");
    let rynek = route
        .qualified_access
        .iter()
        .filter(|entry| {
            route.legs[0].edges.iter().any(|edge| {
                entry.edge_ids.contains(&edge.edge_id)
                    && edge.name.as_deref() == Some("Rynek Główny")
            })
        })
        .collect::<Vec<_>>();
    assert!(
        !rynek.is_empty(),
        "the route crosses Rynek Główny without a qualifier: {:?} / {:?}",
        route.qualified_access,
        route.legs[0]
            .edges
            .iter()
            .map(|edge| (edge.edge_id, edge.name.clone()))
            .collect::<Vec<_>>()
    );
    for qualifier in [
        RouteAccessQualifier::Delivery,
        RouteAccessQualifier::Permissive,
    ] {
        assert!(
            rynek.iter().any(|entry| entry.qualifier == qualifier),
            "no {qualifier:?} on Rynek Główny: {rynek:?}"
        );
    }
    for entry in &rynek {
        match entry.qualifier {
            RouteAccessQualifier::Delivery => {
                assert!(entry.conditional);
                assert!(
                    entry
                        .legal_tag
                        .starts_with("motor_vehicle:conditional=delivery @"),
                    "{entry:?}"
                );
            }
            RouteAccessQualifier::Permissive => {
                assert_eq!(entry.legal_tag, "motor_vehicle=permissive", "{entry:?}");
                assert!(!entry.conditional);
            }
            other => panic!("unexpected qualifier {other:?} on Rynek Główny: {entry:?}"),
        }
        assert!(entry.length_m > 0.0);
        assert!(!entry.edge_ids.is_empty());
    }
    // Every entry lies on the route, in route order.
    let route_edges = route.edge_ids.clone();
    let mut last = 0;
    for entry in &route.qualified_access {
        let position = route_edges
            .iter()
            .position(|edge| *edge == entry.edge_ids[0])
            .expect("on the route");
        assert!(position >= last, "{:?}", route.qualified_access);
        last = position;
    }
    eprintln!("qualified access: {:#?}", route.qualified_access);

    // The same with SAR: still reported, next to the SAR report.
    let mut sar_request = request;
    let mut sar = wayfarer::RouteSarOptions::default();
    sar.relaxations.turn_restrictions = Some(Default::default());
    sar_request.options.sar = Some(sar);
    let sar_route = engine.route_leg(sar_request).unwrap();
    assert!(sar_route.sar.is_some());
    assert!(
        sar_route
            .qualified_access
            .iter()
            .any(|entry| rynek.iter().any(|plain| plain.edge_ids == entry.edge_ids))
    );
}

/// A private road (`access=private`, way 1088598573) is destination-only: a
/// normal drive ends on it, and reports it as qualified access.
#[test]
fn a_drive_ends_on_a_private_road_and_reports_it() {
    let dir = tempfile::tempdir().unwrap();
    let engine = krakow_centre(dir.path());
    let request = LegRequest::new(
        GeoPoint::new(50.060148, 19.923060),
        GeoPoint::new(50.060545, 19.922998),
        MobilityProfile::Wheeled,
    );
    let route = engine.route_leg(request).unwrap();
    assert!(route.sar.is_none());
    assert!(
        route.destination.distance_m < 5.0,
        "the drive ends on the private road: {:?}",
        route.destination
    );
    let private = route
        .qualified_access
        .iter()
        .find(|entry| entry.qualifier == RouteAccessQualifier::Private)
        .expect("the private road is reported");
    assert_eq!(private.osm_way_id, 1_088_598_573);
    assert_eq!(private.legal_tag, "access=private");
}

/// SAR routes on the Kraków centre package: `legal_access` reaches a point
/// on a road closed to vehicles (`access=no`, way 368500255) that a normal
/// route can only approach from 13 m away; `wrong_way` drives 60 m against
/// the one-way way 26516070 where a normal route goes 500 m round the
/// block. Both report what they relaxed in `LegResult::sar`.
#[test]
fn sar_routes_use_a_closed_road_and_drive_against_a_one_way() {
    let dir = tempfile::tempdir().unwrap();
    let engine = krakow_centre(dir.path());
    let sar = |json: &str| -> wayfarer::RouteSarOptions { serde_json::from_str(json).unwrap() };

    // A road closed to vehicles.
    let request = LegRequest::new(
        GeoPoint::new(50.047500, 19.930000),
        GeoPoint::new(50.048116, 19.929630),
        MobilityProfile::Wheeled,
    );
    let normal = engine.route_leg(request.clone()).unwrap();
    assert!(normal.sar.is_none());
    assert!(
        normal.destination.distance_m > 10.0,
        "a normal route does not snap onto the closed road: {:?}",
        normal.destination
    );
    let mut closed = request.clone();
    closed.options.sar = Some(sar(r#"{"relaxations":{"legal_access":{}}}"#));
    let relaxed = engine.route_leg(closed).unwrap();
    assert!(
        relaxed.destination.distance_m < 1.0,
        "{:?}",
        relaxed.destination
    );
    assert!(relaxed.cost < normal.cost);
    let report = relaxed.sar.as_ref().unwrap();
    eprintln!(
        "closed road: {}",
        serde_json::to_string_pretty(report).unwrap()
    );
    let used = report
        .uses
        .iter()
        .find(|used| used.osm_way_id == Some(368_500_255))
        .expect("a legal_access use on the closed road");
    assert_eq!(used.kind, wayfarer::SarRelaxationKind::LegalAccess);
    assert_eq!(used.legal_tag.as_deref(), Some("access=no"));
    assert!(used.length_m > 20.0 && used.added_cost_s > 0.0);
    assert!(
        report
            .totals
            .iter()
            .any(|total| total.kind == wayfarer::SarRelaxationKind::LegalAccess)
    );

    // Against a one-way.
    let request = LegRequest::new(
        GeoPoint::new(50.047217, 19.925149),
        GeoPoint::new(50.047720, 19.925424),
        MobilityProfile::Wheeled,
    );
    let normal = engine.route_leg(request.clone()).unwrap();
    let mut against = request;
    against.options.sar = Some(sar(r#"{"relaxations":{"wrong_way":{}}}"#));
    let relaxed = engine.route_leg(against).unwrap();
    assert!(normal.distance_m > 400.0, "{}", normal.distance_m);
    assert!(relaxed.distance_m < 100.0, "{}", relaxed.distance_m);
    assert!(relaxed.cost < normal.cost);
    let report = relaxed.sar.as_ref().unwrap();
    eprintln!(
        "wrong way: {}",
        serde_json::to_string_pretty(report).unwrap()
    );
    let used = report
        .uses
        .iter()
        .find(|used| used.kind == wayfarer::SarRelaxationKind::WrongWay)
        .expect("a wrong_way use");
    assert_eq!(used.osm_way_id, Some(26_516_070));
    assert_eq!(used.legal_tag.as_deref(), Some("oneway=yes"));
    assert!(used.length_m > 50.0);
    // The route starts on the one-way: no entry, so R2 adds the edge's
    // cost once more (factor 2).
    assert!((used.added_cost_s * 2.0 - relaxed.cost).abs() < 0.01 * relaxed.cost);
}

/// R5 on the Kraków centre package: from the middle of Rynek Główny, where a
/// normal drive snaps 81 m away, onto the square's ring (which it leaves
/// over destination-only streets), `vehicle_on_paths` with unknown widths
/// allowed starts 2 m away, on the square, and drives out over stretches of
/// pedestrian streets and footways, each reported with its class, length,
/// added cost and, where no width is tagged, "width unknown". With the
/// default (unknown widths denied) it starts where a normal drive does and
/// uses no path.
#[test]
fn sar_drives_out_of_rynek_glowny_over_pedestrian_streets() {
    use wayfarer::SarRelaxationKind;
    let dir = tempfile::tempdir().unwrap();
    let engine = krakow_centre(dir.path());
    let sar = |json: &str| -> wayfarer::RouteSarOptions { serde_json::from_str(json).unwrap() };
    let request = LegRequest::new(
        GeoPoint::new(50.0617, 19.9373),
        GeoPoint::new(50.0590, 19.9440),
        MobilityProfile::Wheeled,
    );
    let normal = engine.route_leg(request.clone()).unwrap();
    assert!(normal.origin.distance_m > 50.0, "{:?}", normal.origin);
    let mut relaxed = request.clone();
    relaxed.options.sar = Some(sar(
        r#"{"relaxations":{"vehicle_on_paths":{"unknown_width":"allow"}}}"#,
    ));
    let relaxed = engine.route_leg(relaxed).unwrap();
    assert!(relaxed.origin.distance_m < 5.0, "{:?}", relaxed.origin);
    let report = relaxed.sar.as_ref().unwrap();
    eprintln!(
        "out of Rynek Główny: {}",
        serde_json::to_string_pretty(report).unwrap()
    );
    assert!(!report.uses.is_empty());
    for used in &report.uses {
        assert_eq!(used.kind, SarRelaxationKind::VehicleOnPaths, "{used:?}");
        assert!(used.length_m > 0.0 && used.added_cost_s > 0.0, "{used:?}");
        let tag = used.legal_tag.as_deref().unwrap_or_default();
        assert!(
            [
                "highway=pedestrian",
                "highway=footway",
                "highway=cycleway",
                "highway=path"
            ]
            .contains(&tag),
            "{used:?}"
        );
    }
    assert!(report.uses.iter().any(|used| {
        used.legal_tag.as_deref() == Some("highway=pedestrian")
            && used.confidence.contains(&"width unknown".to_string())
    }));
    let total = &report.totals[0];
    assert_eq!(total.kind, SarRelaxationKind::VehicleOnPaths);
    assert_eq!(total.count, report.uses.len());
    // The paths' surcharge is two thirds of what the route pays for them,
    // and less than the whole route's cost.
    assert!(total.added_cost_s < relaxed.cost);
    // Every use is on the route, in route order.
    let mut last = 0;
    for used in &report.uses {
        let position = relaxed
            .edge_ids
            .iter()
            .position(|edge| *edge == used.edge_ids[0])
            .expect("on the route");
        assert!(position >= last);
        last = position;
    }

    let mut default = request;
    default.options.sar = Some(sar(r#"{"relaxations":{"vehicle_on_paths":{}}}"#));
    let default = engine.route_leg(default).unwrap();
    assert!(
        (default.origin.distance_m - normal.origin.distance_m).abs() < 1.0,
        "{:?}",
        default.origin
    );
    assert!(default.sar.as_ref().unwrap().uses.is_empty());
}

/// SAR on the Emergency profile (stage 3): the same closed road and
/// one-way as above, relaxed together, costed by Emergency (its derived
/// column, bounded by potentials of its own metric). `vehicle_on_paths`
/// stays Wheeled only.
#[test]
fn an_emergency_sar_route_uses_a_closed_road_and_drives_against_a_one_way() {
    let dir = tempfile::tempdir().unwrap();
    let engine = krakow_centre(dir.path());
    let sar = serde_json::from_str::<wayfarer::RouteSarOptions>(
        r#"{"relaxations":{"legal_access":{},"wrong_way":{}}}"#,
    )
    .unwrap();
    let route = |from: GeoPoint, to: GeoPoint, sar: Option<&wayfarer::RouteSarOptions>| {
        let mut request = LegRequest::new(from, to, MobilityProfile::Emergency);
        request.options.sar = sar.cloned();
        engine.route_leg(request)
    };

    // Onto the closed road (way 368500255).
    let (from, to) = (
        GeoPoint::new(50.047500, 19.930000),
        GeoPoint::new(50.048116, 19.929630),
    );
    let normal = route(from, to, None).unwrap();
    assert!(normal.sar.is_none());
    assert!(
        normal.destination.distance_m > 10.0,
        "{:?}",
        normal.destination
    );
    let relaxed = route(from, to, Some(&sar)).unwrap();
    eprintln!(
        "emergency, closed road: {:.1} m, {:.1} s, cost {:.1} (normal {:.1} m, cost {:.1}); {}",
        relaxed.distance_m,
        relaxed.duration_s,
        relaxed.cost,
        normal.distance_m,
        normal.cost,
        serde_json::to_string_pretty(relaxed.sar.as_ref().unwrap()).unwrap()
    );
    assert!(
        relaxed.destination.distance_m < 1.0,
        "{:?}",
        relaxed.destination
    );
    assert!(relaxed.cost < normal.cost);
    let report = relaxed.sar.as_ref().unwrap();
    let used = report
        .uses
        .iter()
        .find(|used| used.osm_way_id == Some(368_500_255))
        .expect("a legal_access use on the closed road");
    assert_eq!(used.kind, wayfarer::SarRelaxationKind::LegalAccess);
    assert_eq!(used.legal_tag.as_deref(), Some("access=no"));
    assert!(used.length_m > 20.0 && used.added_cost_s > 0.0);

    // Against the one-way (way 26516070).
    let (from, to) = (
        GeoPoint::new(50.047217, 19.925149),
        GeoPoint::new(50.047720, 19.925424),
    );
    let normal = route(from, to, None).unwrap();
    let relaxed = route(from, to, Some(&sar)).unwrap();
    assert!(normal.distance_m > 400.0, "{}", normal.distance_m);
    assert!(relaxed.distance_m < 100.0, "{}", relaxed.distance_m);
    assert!(relaxed.cost < normal.cost);
    let used = relaxed
        .sar
        .as_ref()
        .unwrap()
        .uses
        .iter()
        .find(|used| used.kind == wayfarer::SarRelaxationKind::WrongWay)
        .expect("a wrong_way use");
    assert_eq!(used.osm_way_id, Some(26_516_070));
    assert_eq!(used.legal_tag.as_deref(), Some("oneway=yes"));

    // Vehicles on paths are not available on Emergency.
    let paths = serde_json::from_str::<wayfarer::RouteSarOptions>(
        r#"{"relaxations":{"vehicle_on_paths":{}}}"#,
    )
    .unwrap();
    assert!(matches!(
        route(from, to, Some(&paths)).map_err(|failure| failure.error),
        Err(wayfarer::RouteError::SarUnavailable {
            relaxation: wayfarer::SarRelaxationKind::VehicleOnPaths
        })
    ));
}

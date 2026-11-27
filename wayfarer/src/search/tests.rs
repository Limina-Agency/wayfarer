//! Tests for `search.rs`: alternative penalties and destination candidates
//! on the package route search.

use wayfarer_package::{DirectionalAccess, RoadType};
use wayfarer_road_graph::{RoadGraph, RoadGraphBuilder, RoutableWay, enhance_road_graph};

use super::*;
use crate::types::MobilityProfile;

fn test_way(id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
    RoutableWay {
        osm_way_id: id,
        points,
        road_type: RoadType::Primary,
        directional_access: DirectionalAccess::Both,
        ..Default::default()
    }
}

fn test_graph(name: &str, ways: Vec<Vec<(f64, f64)>>) -> RoadGraph {
    let mut builder = RoadGraphBuilder::new(name);
    for (index, points) in ways.into_iter().enumerate() {
        builder.push_way(test_way(index as i64 + 1, points));
    }
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    graph
}

fn test_correlated_record(record: &GraphViewEdge, fraction_along_edge: f64) -> CorrelatedLocation {
    CorrelatedLocation {
        edge_id: record.edge.edge_id,
        compact_edge: Default::default(),
        shared_edge_info_id: record.edge.shared_edge_info_id,
        opposing_edge_id: None,
        road_group_key: None,
        same_road_candidate_count: 0,
        source_node_id: record.source.id,
        target_node_id: record.target.id,
        snapped_point: GeoPoint::new(record.target.lat, record.target.lng),
        distance_m: 0.0,
        fraction_along_edge,
        heading_deg: None,
        heading_delta_deg: None,
        side: crate::types::CorrelationSide::Unknown,
        component_size: None,
        outbound_reach_edge_count: None,
        inbound_reach_edge_count: None,
        layer: None,
        level: None,
        score: 1.0,
        score_breakdown: Default::default(),
        confidence: 1.0,
        access: None,
    }
}

fn test_edge_by_lngs(view: &PackageGraphView, source_lng: f64, target_lng: f64) -> GraphViewEdge {
    (0..view.raw_edge_count())
        .filter_map(|index| view.edge_record_by_raw_edge_index(index))
        .find(|record| {
            (record.source.lng - source_lng).abs() <= 1e-9
                && (record.target.lng - target_lng).abs() <= 1e-9
        })
        .expect("test edge by longitude")
}

fn search_route(
    view: &PackageGraphView,
    request: &LegRequest,
    origins: &[CorrelatedLocation],
    destinations: &[CorrelatedLocation],
) -> SearchRoute {
    Search::new_with_destination_only_policy(view, request, true)
        .find(origins, destinations, RouteDiagnostics::default())
        .expect("package route")
}

#[test]
fn alternative_penalty_edges_prefer_internal_route_edges() {
    assert_eq!(
        alternative_primary_penalty_edges(&[10, 11, 12, 13]),
        vec![11, 12]
    );
    assert_eq!(alternative_primary_penalty_edges(&[10, 11]), vec![10, 11]);
    assert_eq!(alternative_primary_penalty_edges(&[10]), vec![10]);
}

#[test]
fn alternative_penalty_window_rotates_over_primary_edges() {
    let edges = vec![1, 2, 3, 4, 5, 6];

    assert_eq!(alternative_penalty_window(&edges, 0, 2), vec![1]);
    assert_eq!(alternative_penalty_window(&edges, 1, 2), vec![2]);
    assert_eq!(alternative_penalty_window(&edges, 5, 2), vec![6]);
    assert_eq!(alternative_penalty_window(&edges, 6, 2), vec![1]);
    assert!(alternative_penalty_window(&[], 0, 2).is_empty());
}

#[test]
fn search_chooses_lowest_cost_destination_candidate_for_same_source_node() {
    let graph = test_graph(
        "destination-candidate-cost",
        vec![
            vec![(50.0, 20.0), (50.0, 20.001)],
            vec![(50.0, 20.001), (50.0, 20.011)],
            vec![(50.0, 20.001), (50.0, 20.002)],
        ],
    );
    let engine = crate::test_support::engine(&graph);
    let view = engine.route_graph_view();
    let origin_edge = test_edge_by_lngs(view, 20.0, 20.001);
    let expensive_destination_edge = test_edge_by_lngs(view, 20.001, 20.011);
    let cheap_destination_edge = test_edge_by_lngs(view, 20.001, 20.002);
    let request = LegRequest::new(
        GeoPoint::new(50.0, 20.0),
        GeoPoint::new(50.0, 20.002),
        MobilityProfile::Wheeled,
    );

    let route = search_route(
        view,
        &request,
        &[test_correlated_record(&origin_edge, 0.0)],
        &[
            test_correlated_record(&expensive_destination_edge, 1.0),
            test_correlated_record(&cheap_destination_edge, 1.0),
        ],
    );

    assert_eq!(
        route.edge_ids.last().copied(),
        Some(cheap_destination_edge.edge.edge_id),
        "the search should evaluate every destination candidate at the same source node"
    );
}

#[test]
fn search_does_not_stop_at_costlier_early_destination_candidate() {
    let graph = test_graph(
        "destination-frontier-bound",
        vec![
            vec![(50.0, 20.0), (50.0, 20.001)],
            vec![(50.0, 20.001), (50.0, 20.011)],
            vec![(50.0, 20.001), (50.0, 20.002)],
            vec![(50.0, 20.002), (50.0, 20.003)],
        ],
    );
    let engine = crate::test_support::engine(&graph);
    let view = engine.route_graph_view();
    let origin_edge = test_edge_by_lngs(view, 20.0, 20.001);
    let early_expensive_destination_edge = test_edge_by_lngs(view, 20.001, 20.011);
    let cheap_connector_edge = test_edge_by_lngs(view, 20.001, 20.002);
    let later_cheap_destination_edge = test_edge_by_lngs(view, 20.002, 20.003);
    let request = LegRequest::new(
        GeoPoint::new(50.0, 20.0),
        GeoPoint::new(50.0, 20.0021),
        MobilityProfile::Wheeled,
    );

    let route = search_route(
        view,
        &request,
        &[test_correlated_record(&origin_edge, 0.0)],
        &[
            test_correlated_record(&early_expensive_destination_edge, 1.0),
            test_correlated_record(&later_cheap_destination_edge, 0.1),
        ],
    );

    assert!(
        route.edge_ids.contains(&cheap_connector_edge.edge.edge_id),
        "the search should keep exploring a cheaper frontier before accepting an early destination"
    );
    assert_eq!(
        route.edge_ids.last().copied(),
        Some(later_cheap_destination_edge.edge.edge_id)
    );
}

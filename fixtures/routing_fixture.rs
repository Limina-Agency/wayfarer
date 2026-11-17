//! Small reproducible routing package shared by core and transport tests.
use wayfarer_compiler::package_build::{PackageBuildOptions, write_package};
use wayfarer_road_graph::{
    DirectionalAccess, RoadGraphBuilder, RoadType, RoutableWay, build_road_graph_manifest,
    enhance_road_graph, inspect_road_graph_container, serialize_road_graph,
};

pub fn write(path: &std::path::Path) {
    let mut builder = RoadGraphBuilder::new("dataset-fixture");
    builder.push_way(RoutableWay {
        osm_way_id: 1,
        points: vec![(52.0, 21.0), (52.0, 21.001), (52.0, 21.002)],
        road_type: RoadType::Primary,
        directional_access: DirectionalAccess::Both,
        name: Some("Fixture road".into()),
        highway: Some("primary".into()),
        access: Some("yes".into()),
        ..RoutableWay::default()
    });
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    let bytes = serialize_road_graph(&graph).unwrap();
    let container = inspect_road_graph_container(&bytes).unwrap();
    let manifest = build_road_graph_manifest(&graph, &container);
    write_package(
        &graph,
        Some(&manifest),
        path,
        &PackageBuildOptions::default(),
    )
    .unwrap();
}

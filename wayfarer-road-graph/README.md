# wayfarer-road-graph

The routing road graph: directed edges over shared way attributes, built from
normalized OSM ways, then enhanced (connected components, junctions, opposing
edges, resolved turn restrictions) and checked for quality. It is the
in-memory form the route compiler builds from a PBF extract and writes out as a
routing package; it can also be saved and reloaded as a compressed snapshot.

The vocabulary the graph speaks (`RoadEdge`, `SharedEdgeInfo`, `RoadType`,
`DirectionalAccess`, ...) is defined in the routing package format crate and
re-exported here.

## Example

```rust
use wayfarer_road_graph::{
    DirectionalAccess, RoadGraphBuilder, RoadType, RoutableWay, enhance_road_graph,
};

let mut builder = RoadGraphBuilder::new("example");
builder.push_way(RoutableWay {
    osm_way_id: 1,
    points: vec![(52.0, 21.0), (52.0, 21.001), (52.0, 21.002)], // (lat, lng)
    road_type: RoadType::Primary,
    directional_access: DirectionalAccess::Both,
    name: Some("Example road".into()),
    highway: Some("primary".into()),
    access: Some("yes".into()),
    ..RoutableWay::default()
});

let mut graph = builder.finish();
enhance_road_graph(&mut graph);
println!("{} nodes, {} directed edges", graph.node_count(), graph.edge_count());
```

From here, `serialize_road_graph` and `deserialize_road_graph` (with the
`snapshot` feature, which brings zstd and BLAKE3) write and read a snapshot,
and `evaluate_road_graph_quality` produces the manifest that
decides whether a graph is `ready`, a `warning` or `stale`. Turning a graph
into a routing package is the route compiler's job.

## Status

Version 0.x: the API may change between minor releases.

The snapshot carries its own versions (`ROAD_GRAPH_SNAPSHOT_VERSION` and
`ROAD_GRAPH_CONTAINER_VERSION`, 1 today). They describe an internal, intermediate
form: it is not the distribution format, which is the routing package. Do not
depend on snapshots written by another release.

## Data

Graphs built from OpenStreetMap ways are OSM data; anything derived from them
is a derived database under the Open Database License 1.0. Credit
"© OpenStreetMap contributors" (<https://www.openstreetmap.org/copyright>) when
you distribute it.

## License

Licensed under the Apache License, Version 2.0 (`LICENSE`).

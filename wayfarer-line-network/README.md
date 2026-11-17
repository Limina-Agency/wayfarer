# wayfarer-line-network

Turns OSM ways into a line network: ways are split at the nodes they share into
a topology of segments with merged attributes, and crossings that lack a shared
node are repaired. Routing and rendering built on the same network therefore
see the same junctions. The route compiler uses it to produce the road
segments its graph is built from; the layers it models are roads, water lines
and railways.

It works on plain geometry and tag lists, so it has no dependency on any map
file format. Reading a `.osm.pbf` is the job of the PBF ingest crate.

## Example

```rust
use wayfarer_line_network::{
    DirectionalAccessModel, RoadRoutingAttrs, SharedLineInput, SharedLineLayer,
    SharedLineNetworkBuilder,
};

fn road(feature_id: i64, geometry: Vec<(f64, f64)>, node_chain: Vec<i64>) -> SharedLineInput {
    SharedLineInput {
        feature_id,
        layer: SharedLineLayer::Roads,
        geometry, // (lat, lng) per vertex
        node_chain, // the OSM node id of each vertex
        source_lineage_ids: vec![feature_id],
        attrs: Default::default(),
        compile_inputs: Default::default(),
        min_zoom: 0,
        max_zoom: 14,
        sort_order: 0,
        routing: Some(RoadRoutingAttrs {
            road_class: "residential".into(),
            directional_access: DirectionalAccessModel::Both,
            access: None,
            surface: None,
            tracktype: None,
            smoothness: None,
            has_ford: false,
            has_tunnel: false,
            access_restricted: false,
        }),
    }
}

// Two ways that cross at OSM node 2.
let mut builder = SharedLineNetworkBuilder::default();
builder.push_input(road(10, vec![(52.000, 21.000), (52.001, 21.001), (52.002, 21.002)], vec![1, 2, 3]));
builder.push_input(road(11, vec![(52.002, 21.000), (52.001, 21.001), (52.000, 21.002)], vec![4, 2, 5]));

// Topology only: no render products, which a routing build would discard.
let network = builder.finish_topology_only();

// Each way is split at the shared node: four segments meeting at one junction.
assert_eq!(network.topology.edges.len(), 4);
assert_eq!(network.road_segments().count(), 4);
```

`finish` also derives render lines (duplicate and stub removal, merged
strokes), with their zoom range and sort order from the inputs;
`finish_with_observer` reports a checkpoint after each phase, which is where a
build's memory peaks.

## Status

Version 0.x: the API may change between minor releases. It was written for the
route compiler and for a vector basemap compiler; the render-oriented fields
(`min_zoom`, `max_zoom`, `sort_order`, `compile_inputs`) exist for the latter
and are ignored by routing.

## Data

Networks built from OpenStreetMap ways are OSM data; anything derived from them
is a derived database under the Open Database License 1.0. Credit
"© OpenStreetMap contributors" (<https://www.openstreetmap.org/copyright>) when
you distribute it.

## License

Licensed under the Apache License, Version 2.0 (`LICENSE`).

# wayfarer-costing

The edge-cost model that routing packages are built and queried with: costing
options, per-profile speeds and factors, attribute classes, conditional
restrictions, turn costs and junction delays. The route compiler bakes each profile's default
costs into a package with it, and the routing engine uses the same code to
derive columns for other costing options and to layer a request's blocks,
avoidances and multipliers on top (`EdgeOverrides`). One model on both sides
keeps a package's stored costs and the engine's per-request costs consistent.

Most applications use it indirectly, through the engine. Depend on it directly
to classify OSM tags the way the model does, to parse conditional restrictions,
or to cost edges yourself with `CostModel`.

## Example

```rust
use wayfarer_costing::{
    COSTING_MODEL_VERSION, SurfaceCostClass, road_type_default_speed_kph, surface_cost_class,
    time_condition_active,
};
use wayfarer_package::RoadType;

// OSM tags are classified once, at build time, into small enums.
assert_eq!(surface_cost_class(Some("asphalt")), SurfaceCostClass::Paved);
assert_eq!(surface_cost_class(Some("sett")), SurfaceCostClass::Cobblestone);

// Free-flow defaults per road type, in km/h.
assert_eq!(road_type_default_speed_kph(RoadType::Motorway), 90.0);

// Conditional restrictions are evaluated against a time of day.
assert_eq!(time_condition_active("07:00-09:00", 8 * 60), Some(true));

// Packages record the model their stored costs were baked with.
println!("costing model {COSTING_MODEL_VERSION}");
```

`RoadType`, `MobilityProfile` and the other road vocabulary come from the
routing package format crate, which this crate depends on.

## Status

Version 0.x: the API may change between minor releases.

`COSTING_MODEL_VERSION` (1 today) is bumped whenever a change here changes any
cost or duration the compiler stores. A package records the model it was baked
with; an engine re-costs a package of an older model from its stored costing
inputs (slower until it is recompiled) and refuses a newer one, so such a
change ships the compiler and the engine together, and existing packages
should be recompiled. Turn costs and junction delays (`junction`: the time lost turning,
yielding and waiting at signals or signs, added to durations) are not stored:
the engine evaluates them at query time from stored inputs and landmark bounds
ignore them, so a change that only raises them needs no bump.

## Data

The crate contains no map data. Packages whose costs it computes are derived
from OpenStreetMap, which is under the Open Database License 1.0; credit
"© OpenStreetMap contributors" (<https://www.openstreetmap.org/copyright>) when
you distribute them.

## License

Licensed under the Apache License, Version 2.0 (`LICENSE`).

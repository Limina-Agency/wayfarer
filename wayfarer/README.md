# wayfarer

An offline routing engine over routing packages. Open a package compiled from an
OpenStreetMap extract and ask for routes (with alternatives and turn-by-turn
maneuvers), distance and duration matrices, isochrones, map-matched traces and
navigation progress, all on a memory-mapped search graph: bidirectional A* with
CCH lower-bound potentials and exact shortest paths. Requests can carry dynamic overlays
(blocked or avoided edges, cost multipliers, connectors) that are costed per
request, with no recompile.

It was built for regional packages on laptops and phones with poor or no
connectivity. The package format has its own crate, the route compiler writes
packages, a service crate adds source caching and route workflows on top of
this engine, and a C ABI crate exposes it to other languages.

## Example

```rust,no_run
use wayfarer::{
    GeoPoint, MobilityProfile, RouteMatrixRequest, RouteOverlay, RouteOverlayGeometryKind,
    RouteOverlayPolicy, RouteRequest, RoutingEngine, SnapRequest,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // The package is memory-mapped, so opening it is cheap.
    let engine = RoutingEngine::open("routing.mrt".as_ref())?;

    let from = GeoPoint::new(43.7397, 7.4275);
    let via = GeoPoint::new(43.7350, 7.4210);
    let to = GeoPoint::new(43.7290, 7.4160);

    // Through a waypoint, around a closed road (edges within 10 m of the line).
    let closure = RouteOverlay::new(
        RouteOverlayGeometryKind::Line,
        vec![GeoPoint::new(43.7330, 7.4190), GeoPoint::new(43.7325, 7.4200)],
        RouteOverlayPolicy::NoGo,
        10,
    );
    let request = RouteRequest::through([from, via, to], MobilityProfile::Wheeled)
        .with_overlay(closure);
    let route = engine.route(request)?;
    println!(
        "{:.0} m, {:.0} s, {} maneuvers",
        route.distance_m,
        route.duration_s,
        route.maneuvers.len()
    );

    let matrix = engine.matrix(RouteMatrixRequest::new(vec![from, to], vec![to, from], MobilityProfile::OnFoot));
    println!("{} cells", matrix.cells.len());

    let snapped = engine.snap(SnapRequest::new(from, MobilityProfile::Wheeled));
    println!("{} candidates", snapped.candidates.len());
    Ok(())
}
```

The engine answers:

- `route`: two or more locations, each leg routed or a straight line, with
  overlays (no-go, slow-go, warning, discount, open; as geometry or package
  edges, optionally per profile and time window). One search per routed leg
  (`route_leg` runs a single one).
- `matrix`, `isochrone`, `map_match` (GPS traces), `snap` (where a point meets
  the road network) and `project` (the package edges a geometry covers).
- Navigation progress along a prepared route (`PreparedNavigationRoute`).

Requests and results are serde types; their JSON keys are camelCase and enum
values snake_case. With the `schema` feature they derive `schemars::JsonSchema`
(the repository's `schema/` directory holds the generated schemas). Request
size bounds are in `limits`. Mobility profiles are `Default` (as `Wheeled`),
`Wheeled`, `Tracked`, `Emergency`, `OnFoot` and `Legged`; `MobilityProfile`
documents how each is routed and costed.

## Status

Version 0.x: the API may change between minor releases.

- **Package format.** An engine opens the formats
  `MIN_READ_FORMAT_VERSION..=ROUTING_PACKAGE_FORMAT_VERSION` (1 today).
  Older packages fail to open as outdated, newer ones as too new.
- **Costing model.** A package records the costing model its costs were baked
  with (`COSTING_MODEL_VERSION`, 1 today). Packages of an older model open and
  are re-costed per request; a newer model fails as too new.
- **Features.** `schema` derives JSON Schemas. `reference` adds a
  plain-Dijkstra oracle for parity tests, and `debug-env` reads debug switches
  from the environment; both are test and profiling support.

## Data

Routes computed on a package are derived from OpenStreetMap data, under the Open
Database License 1.0. When you display or distribute results or packages, credit
"© OpenStreetMap contributors" (<https://www.openstreetmap.org/copyright>).

## License

Licensed under the Apache License, Version 2.0 (`LICENSE`).

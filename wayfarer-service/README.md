# wayfarer-service

Routing sources over the offline routing engine. A host describes where a
routing package lives (a stable source id, a revision and a local path); the
service verifies it, opens it, keeps a bounded cache of opened packages,
invalidates them when a source changes, reports readiness, and routes against
it. Route requests may also carry overlay sets (versioned overlay bundles shared
between devices, `overlay_set`) and precompiled mobility input, which the
service folds into the engine's request.

The host keeps what only it knows: which package to use, where it is stored and
what to do with the result. The service never serializes filesystem paths.

## Example

```rust,no_run
use wayfarer::{GeoPoint, MobilityProfile, RouteRequest};
use wayfarer_service::{RoutingService, RoutingSource, RoutingSourceId};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let service = RoutingService::default(); // caches up to 4 opened sources
    let source = RoutingSource::new(RoutingSourceId::new("monaco")?, "2026-10-01", "routing.mrt")?;

    // Once, when the package is installed: checks every section's checksum.
    service.verify_source(&source)?;

    // Opens (or reuses) the package and says whether it can route.
    let readiness = service.inspect_source(&source);
    assert!(readiness.is_ready(), "{readiness:?}");

    let result = service.route(
        &source,
        RouteRequest::through(
            [GeoPoint::new(43.7397, 7.4275), GeoPoint::new(43.7290, 7.4160)],
            MobilityProfile::Wheeled,
        )
        .into(),
    )?;
    println!(
        "{:.0} m, {:.0} s (cache hit: {})",
        result.route.distance_m, result.route.duration_s, result.cache_hit
    );
    Ok(())
}
```

`route` takes a `RouteWorkflowRequest`: the engine's `RouteRequest` (two or
more locations, legs, overlays) plus `overlay_sets` and `mobility`;
`route_leg` routes one leg. `RoutingService::new` sets the cache capacity and
the memory profile (desktop or mobile), and `invalidate` drops a source whose
package was replaced.

## Status

Version 0.x: the API may change between minor releases. Package format and
costing model compatibility are those of the engine it wraps: it opens the
formats the engine reads, and only packages of the engine's costing model; an
outdated package is reported as such in the readiness result rather than
failing opaquely.

## Data

Routes come from packages compiled from OpenStreetMap, which is under the Open
Database License 1.0. When you display or distribute results or packages, credit
"© OpenStreetMap contributors" (<https://www.openstreetmap.org/copyright>).

## License

Licensed under the Apache License, Version 2.0 (`LICENSE`).

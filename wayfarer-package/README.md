# wayfarer-package

The file format of offline routing packages. A package (`routing.mrt`) is one
memory-mappable file holding the road network of an OpenStreetMap extract:
nodes, directed edges and the attributes they share, per-profile edge costs,
turn restrictions and the node order of CCH potentials, in checksummed
sections. This crate
opens, verifies, reads and writes such files, and defines the road vocabulary
(`RoadEdge`, `SharedEdgeInfo`, `MobilityProfile`, ...) that the route compiler
and the routing engine share.

It does not compile a package from OSM data or answer routes: the route
compiler writes packages through it, and the engine reads them through it.
Reach for it directly to inspect or verify a package, or to read its sections.

## Example

```rust,no_run
use wayfarer_package::format::{PackageFile, ROUTING_PACKAGE_FORMAT_VERSION};
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = Path::new("routing.mrt");

    // Hashes every section against the section table. Reads the whole file,
    // so do it once, when a package is installed.
    let verification = PackageFile::verify(path)?;
    println!("{} bytes, dataset hash {}", verification.byte_len, verification.dataset_hash);

    // Opening maps the file and checks the header and each section's bounds,
    // without hashing the sections.
    let package = PackageFile::open(path)?;
    let counts = package.counts();
    println!(
        "{}: format {} (this build writes {}), {} nodes, {} directed edges",
        package.graph_id(),
        package.format_version(),
        ROUTING_PACKAGE_FORMAT_VERSION,
        counts.node_count,
        counts.edge_count,
    );
    Ok(())
}
```

Opening a package of a format older than `MIN_READ_FORMAT_VERSION` fails with
`RoutingPackageError::Outdated`; a format newer than this build reads fails
with `TooNew`.

## Reading sections

`PackageFile::view` returns `Sections`, borrowed slices of every section plus
accessors for the ones that are not plain arrays. Several sections are stored
compactly or derived from others, so read them through the accessors:

| Data | Accessor | Layout |
|---|---|---|
| Exact node positions | `Sections::node_lat_lng` | Only the nodes that are not their E7 position (`NodeLatLngNodes`, sorted); the others read their E7 position. |
| Costing inputs | `si_cost_flags[cost_tuple(shared)]` (likewise classes, speeds) | A table of distinct (flags, classes, speeds) tuples and a tuple row per shared info (`CostTuples`): `u16` in format 2 (`SharedInfoCostTuple`), `u32` in format 3 (`SharedInfoCostTuple32`). |
| Way ids | `si_way.get(shared)`, `si_way.same_way(a, b)` | `u32`, with `WAY_ID_WIDE` marking ids kept apart (`SharedInfoWayWide*`): synthetic negative ids. |
| Edge id → edge | `Sections::edge_for_id` | Not stored: built from `EdgeId` on first use and kept by the `PackageFile`. |
| Name and ref tokens | `si_name[shared]`, `si_ref[shared]` (`ClassTokens`) | One token per attribute class (`SharedAttrClass`): the class key holds the name and ref. |
| Route refs | `si_route_refs.get(shared)` | Offsets per attribute class into `SharedInfoRouteRefPool`. |
| Shapes | `Sections::shared_shape`, `Sections::edge_shape` | `ShapeStart` (`u32`, format 2) or `ShapeStart64` (`u64`, format 3) holds every `SHAPE_START_STRIDE`th (16th) offset (`ByteOffsets`); the shapes between are skipped (`skip_shape`). |
| Node unit vectors | `Sections::xyz` | Fixed point (`NodeXyzFixed`), within `FIXED_XYZ_MARGIN_M` of the exact vector. |
| In-edges, edge sources, end headings | `Sections::in_edges`, `Sections::edge_source`, `Sections::heading_end` | Derived from each edge's twin (`EdgeTwin`, the reverse edge of the same shared info) and the edges without one (`LoneInEdge`, `LoneHeadingEnd`). |

## Status

Version 0.x: the API may change between minor releases. The file format is
versioned separately:

- `ROUTING_PACKAGE_FORMAT_VERSION` (2 today) changes whenever the layout or
  the meaning of a section changes. A build opens the formats
  `MIN_READ_FORMAT_VERSION..=ROUTING_PACKAGE_FORMAT_VERSION` (2..=2; 0.2.0
  dropped format 1, whose landmark sections are now reserved); a
  layout change keeps reading the previous format for at least one release,
  and raising the minimum drops a format. Format 3 is being built behind a
  compiler switch: its draft packages open only in a build of the same draft
  revision (`FORMAT_3_DRAFT_REVISION`).
- A package records the costing model its stored costs were baked with. An
  engine re-costs a package of an older model from its stored costing inputs
  and refuses a newer one, so a change to stored costs ships the compiler and
  the engine together.
- The format is little-endian; the crate does not build for big-endian targets.
- The `request` module is `#[doc(hidden)]`: it is shared with the compiler and
  engine and is not part of the supported API.

## Data

Packages compiled from OpenStreetMap are derived databases under the Open
Database License 1.0. If you distribute one, credit "© OpenStreetMap
contributors" and offer it under the ODbL; see
<https://www.openstreetmap.org/copyright>.

## License

Licensed under the Apache License, Version 2.0 (`LICENSE`).

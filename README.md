# Wayfarer

Offline routing for regional OpenStreetMap extracts: a compiler that turns an
`.osm.pbf` extract into one memory-mappable routing package (`routing.mrt`),
and an engine that answers route, matrix and isochrone requests on it, in Rust
or through a C ABI.

## Built for search and rescue

Wayfarer was designed around search-and-rescue (SAR) and disaster-response
work: teams in the field, on laptops, tablets and phones, with poor or no
connectivity, on roads that change by the hour.

- **Compile on the device.** Turn a fresh OSM extract into a routing package
  on the laptop or tablet that will use it, with no server and no network.
  The compiler is a library: it runs inside your app's process (from Rust, or
  `wayfarer_compile_*` in the C library), with progress, cancellation, a
  thread budget and a low-memory profile. A region the size of Małopolskie
  (194 MiB extract) compiles in about 17 s with a memory peak of about
  1.2 GiB on an 8-core laptop (Ryzen 7 5800H, 16 GB: 16.8–17.5 s and
  1.13 GiB peak working set on Windows, 18.4 s and 1.17 GiB peak RSS under
  WSL).
- **Everything in-process.** No routing server, no sidecar, no tile folder:
  one file, memory-mapped, and one library call per request. The same
  package works on every platform: packages are byte-identical wherever and
  however they're compiled, so a team can check them by hash.
- **A lean runtime.** The routing-only C library is about 4.6 MB with 29
  third-party crates, and the compiler library prints nothing, reads no
  environment and installs no allocator. Typical requests take a few
  milliseconds (median 3.6 ms on Małopolskie, single thread). The packages
  themselves aren't small: Małopolskie's is 260 MiB, 1.16× Valhalla's tiles
  for the same region (see [Limits](#limits)).
- **Change the roads per request.** Closures, blocked bridges, slow-go
  stretches, newly opened tracks and ad-hoc connectors are overlays on a
  request, not a recompile. Overlay sets bind to a package by its hash, so a
  team can share them.
- **SAR routing.** For vehicle profiles, off by default: a request can relax
  turn restrictions, legal and conditional closures, one-way rules, barriers
  and path-width limits at a stated cost, reach the last stretch off the
  network, and get every relaxed stretch reported. Emergency vehicles may cross private and
  destination-only roads; every route reports where its legal access is
  qualified.

Route quality is checked against Valhalla on the same extract
(`benchmarks/valhalla/README.md`): median length and time within 1%, path
overlap 0.97.

## Limits

- **Regions, not continents.** Wayfarer is built and measured on regional
  extracts: a province or a small country. Małopolskie (194 MiB extract,
  1.07 million nodes, 2.56 million edges) gives a 176 MiB package (format 2;
  260 MiB in format 1, a third of it landmark distances) and needs about
  1.2 GiB to compile (`--low-memory` barely changes that at this size).
  Packages and compile memory grow with the road network. Larger extracts are
  untested.
- **Slow tails.** The median request is a few milliseconds, but the slowest
  long drives take far longer: p95 153 ms on Małopolskie, against Valhalla's
  16 ms.
- **No elevation.** Costs and durations ignore climbs: there is no elevation
  model, and OSM `incline` tags are stored but not costed. A host that cares
  can add cost overlays.
- **SAR is for vehicles.** The SAR relaxations apply to the `Default`,
  `Wheeled`, `Tracked` and `Emergency` profiles, not `OnFoot` or `Legged`.
- **One region per package**, compiled whole: no incremental updates, no
  traffic data and no departure-time routing (conditional restrictions and
  overlay time windows are evaluated at one time per request).
- **Replacing an open package on FAT32 or exFAT leaves a file for a while.**
  Where Windows can't rename with POSIX semantics (FAT32, exFAT, some
  network shares), the open package is renamed aside to
  `<name>.replaced-<unique>` and deleted once nothing has it open, at the
  latest by the next replacement of the same path. On NTFS and ReFS, Linux
  and macOS it is replaced in place. A process that opened the package
  without delete sharing, outside Wayfarer, still blocks it.

## Crates

| Crate | What it is |
|---|---|
| `wayfarer-package` | The package format: open and verify `routing.mrt` files, read their sections, write them |
| `wayfarer-costing` | The edge-cost model packages are built and queried with |
| `wayfarer` | The engine: routes with alternatives, matrices, isochrones, map matching, turn-by-turn guidance, dynamic overlays |
| `wayfarer-service` | Routing sources, readiness checks and route workflows (waypoints, direct legs) over the engine |
| `wayfarer-ffi` | The C ABI (`include/wayfarer.h`): JSON requests and results |
| `wayfarer-compiler` | The compiler library: `compile` a `CompileRequest`, with progress and cancellation, or poll a `CompileJob` |
| `wayfarer-cli` | The `wayfarer` command line: `compile`, `info`, `verify`, `route`, `matrix`, `isochrone`, `match`, `snap` |
| `wayfarer-road-graph`, `wayfarer-line-network` | The compiler's road graph and line network |

The compiler reads PBF through [`osm-pbf-ingest`](https://github.com/Limina-Agency/osm-pbf-ingest),
published from its own repository.

## Compile a package

```sh
cargo build --release -p wayfarer-cli
curl -LO https://download.geofabrik.de/europe/monaco-latest.osm.pbf
target/release/wayfarer compile monaco-latest.osm.pbf -o routing.mrt --country MC
target/release/wayfarer info routing.mrt
```

`compile` prints progress on stderr and the result (size, dataset hash,
versions, counts) as JSON on stdout; `--low-memory` trades speed for memory
without changing the package's bytes. `verify` checks every section against
its checksum. From Rust, `wayfarer_compiler::compile` does the same in
your process.

## Route from Rust

(`wayfarer/examples/route.rs`)

```rust
use wayfarer::{GeoPoint, MobilityProfile, RouteRequest, RoutingEngine};

let engine = RoutingEngine::open("routing.mrt".as_ref())?;
let route = engine.route(RouteRequest::new(
    GeoPoint::new(43.7397, 7.4275),
    GeoPoint::new(43.7290, 7.4160),
    MobilityProfile::Wheeled,
))?;
println!("{:.0} m, {:.0} s", route.distance_m, route.duration_s);
```

## Route from C (or any language with a C FFI)

```sh
cargo build --release -p wayfarer-ffi
cc wayfarer-ffi/examples/route.c -Iwayfarer-ffi/include \
   target/release/libwayfarer_ffi.a -lpthread -ldl -lm -o route          # Linux
cc wayfarer-ffi/examples/route.c -Iwayfarer-ffi/include \
   target/release/libwayfarer_ffi.a -framework CoreFoundation -o route   # macOS
./route routing.mrt 43.7397 7.4275 43.7290 7.4160
```

Open a package, then send requests as JSON and get JSON back: routes through
two or more locations with closures and other overlays (as geometry or package
edges), matrices, isochrones, map matching, snapping a point to the road and
projecting geometry onto package edges. Errors are `{"code", "message"}` with a
status code. Every JSON key is camelCase and every enum value snake_case;
`schema/` has a JSON Schema for each request and result, to generate host types
from. Navigation progress runs on a route prepared once
(`wayfarer_navigation_route_new`). A cancel token
(`wayfarer_cancel_token_new`, optionally with a timeout) passed to a
request stops it from any thread with `WAYFARER_CANCELLED`;
`examples/cancel.c` cancels a worker thread's routes. The header,
`wayfarer.h` (ABI version 1), documents each call, thread safety, and
which strings and handles the caller frees.

## Compatibility

- **Package format.** An engine opens formats
  `MIN_READ_FORMAT_VERSION..=ROUTING_PACKAGE_FORMAT_VERSION` (1..=1 today).
  A layout change bumps the format and keeps reading the previous one for at
  least a release; every readable format has a frozen package under
  `fixtures/routing/format-<N>/` that the tests open and route.
- **Costing model.** A package records the costing model its costs were baked
  with (`COSTING_MODEL_VERSION`). An engine opens packages of its own model and
  of older ones; an older model's package is re-costed from its stored costing
  inputs with the engine's model, so it routes like a recompiled package, only
  slower (each column is derived once) until it is recompiled, and with the
  speeds its own compiler inferred for roads without a posted limit. Readiness
  reports such a package with a warning. A newer model is `TooNew`.
- **Determinism.** A compiler of one release writes the same bytes for the
  same input on every platform, in debug and release builds and at any thread
  count, so packages can be checked by hash wherever they were compiled.
  Everything float-derived that reaches a package uses IEEE arithmetic and the
  pure-Rust `libm` crate, never the platform's C math library
  (`wayfarer_package::math`; `clippy.toml` enforces it). Bytes still
  change between releases when the format, the costing or the compiler's
  output changes; the changelog says so.

## Data licensing

Packages compiled from OpenStreetMap are derived databases under the Open
Database License 1.0. If you distribute them, credit "© OpenStreetMap
contributors" and offer them under the ODbL
(https://www.openstreetmap.org/copyright).

## Development

```sh
scripts/check.sh
```

runs what CI runs: formatting, clippy (warnings are errors), tests, the Monaco
package built, verified and described with the release CLI (checked against
`fixtures/monaco-info.json`), a route, matrix and isochrone through the CLI, a
route and cancelled routes through the C ABI (`wayfarer-ffi/examples/route.c`
and `cancel.c`), and the Kraków centre regression gate.
`benchmarks/gates/` holds the change gates (byte-identical packages, identical
route, matrix and isochrone dumps, timings) on a Małopolskie extract; see
`benchmarks/README.md`.

## License

Copyright 2026 Limina. Licensed under the Apache License, Version 2.0
(`LICENSE`).

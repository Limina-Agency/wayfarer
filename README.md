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
  (194 MiB extract) compiles in about 18 s with a memory peak of about
  0.5 GiB on a 4-core cloud machine (0.2 took about 17 s and 1.2 GiB on an
  8-core laptop), and a country the size of Poland in about 4 minutes with
  4.5 GiB (see Limits).
- **Everything in-process.** No routing server, no sidecar, no tile folder:
  one file, memory-mapped, and one library call per request. The same
  package works on every platform: packages are byte-identical wherever and
  however they're compiled, so a team can check them by hash.
- **A lean runtime.** The routing-only C library is about 4 MB (stripped,
  Linux x86-64) with 32 third-party crates, mimalloc as its allocator among
  them, and the compiler library prints nothing, reads no environment and
  installs no allocator. Typical requests take a few
  milliseconds: on Małopolskie on a 4-core cloud machine, drives p95 6–7 ms,
  walks p95 21–22 ms and two alternatives p95 37–40 ms (0.2.0 on the same
  machine: 8–11, 23–29 and 79–87 ms), and a 100×100 vehicle matrix took
  2.2 s on an 8-core laptop with 0.2 (`benchmarks/gates`). Searches are
  bounded by CCH potentials built from a node order stored in the package. Packages are compact: Małopolskie's is
  176 MiB, 0.79× Valhalla's tiles for the same region.
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

- **Regions and countries, not continents.** Wayfarer is built and measured
  on regional and country extracts. On a 4-core cloud machine with 15 GiB:

  | Extract | Nodes, edges | Package | Compile | Peak memory |
  |---|---|---|---|---|
  | Małopolskie (194 MiB) | 1.07 M, 2.57 M | 176 MiB | about 18 s | 0.5 GiB |
  | Poland (2.3 GiB) | 10.6 M, 27.3 M | 1.84 GiB | about 4 min | 4.5 GiB |
  | Germany (5.5 GiB) | 23.6 M, 60.4 M | 4.26 GiB | about 11 min | 10.2 GiB |

  Packages and compile memory grow with the road network, so a compile of
  Germany needs about 11 GiB free; larger extracts (Europe) are untested
  and need more than 16 GB. A country package opens in one or two seconds,
  but its first route customizes the CCH potentials: about 6–7 s and
  1.5 GiB of heap on Poland, and 30 s and 4.2 GiB on Germany as 0.3.0
  measured it (`prepare_potentials` does it ahead of time). Routes after that take milliseconds (an 890 km drive
  across either country: 14–17 ms).
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
| `wayfarer-sar` | Search and rescue planning over the engine: reach prediction (probability of area), likely routes, team assignment and track coverage |
| `wayfarer-ffi` | The C ABI (`include/wayfarer.h`): JSON requests and results |
| `wayfarer-compiler` | The compiler library: `compile` a `CompileRequest`, with progress and cancellation, or poll a `CompileJob` |
| `wayfarer-cli` | The `wayfarer` command line: `compile`, `info`, `verify`, `route`, `matrix`, `nearest`, `isochrone`, `match`, `snap` |
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
  `MIN_READ_FORMAT_VERSION..=ROUTING_PACKAGE_FORMAT_VERSION` (2..=3 since
  0.5, which writes format 3; 0.2.0 dropped format 1, so recompile older
  packages). Engines before 0.6 don't open format 3. A layout change bumps
  the format and keeps reading the previous one for at least a release;
  every readable format has a frozen package under
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
`fixtures/monaco-info.json`), a route, matrix, nearest and isochrone through
the CLI, a route and cancelled routes through the C ABI
(`wayfarer-ffi/examples/route.c` and `cancel.c`), and the Kraków centre regression gate. CI runs it on Linux
and macOS (Apple silicon), runs the tests and the Monaco package check on
Windows and a 32-bit Linux target, and builds the crates with the oldest Rust
they claim (`rust-version`).
`benchmarks/gates/` holds the change gates (byte-identical packages, identical
route, matrix and isochrone dumps, timings) on a Małopolskie extract; see
`benchmarks/README.md`.

## License

Copyright 2026 Limina. Licensed under the Apache License, Version 2.0
(`LICENSE`).

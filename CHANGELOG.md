# Changelog

Two release lines share this repository: `wayfarer-v*` (the package, costing,
engine, service and C ABI crates) and `wayfarer-compiler-v*` (the compiler, the
`wayfarer` binary and their crates). Each entry names the package format, the
costing model and the C ABI version a release reads or writes.

## Unreleased

- **A package that is open can be replaced on any Windows file system.** On
  NTFS and ReFS the rename already replaced a package an engine had open, as
  on Linux and macOS. Where Windows can't rename with POSIX semantics
  (FAT32, exFAT, some network shares), the rename over the open,
  memory-mapped package fails with "Access is denied". There the open
  package is now renamed aside to `<name>.replaced-<unique>`, the new one
  takes its place, and the old file is deleted once nothing has it open (at
  the latest by the next replacement of the same path).
  `PackageFile::move_into_place` does this for hosts that write packages.
- **C ABI: `wayfarer_open_w` and `wayfarer_verify_w` (Windows only)** take
  a `wchar_t` path, for hosts holding UTF-16 paths. Additive; the C ABI
  version stays 1. The header and FFI README now say that every path is
  UTF-8 on Windows too, and the C examples take their arguments from `wmain`
  and convert them, so paths outside the ANSI code page open.
- **Docs: static linking on Windows needs `/MD`** (the FFI README), with the
  system libraries and the `+crt-static` route for `/MT` hosts.
- **Vehicle matrices are faster.** A strict matrix (destination-only roads
  confined) re-ran the whole matrix relaxed whenever one cell needed it; it
  now searches only the retried cells (pairwise) or the retried sources
  towards their retried targets (rows). Rows leave out targets their source
  cannot reach instead of flooding the region for them, and vehicle
  matrices share row expansions from 51 targets (was 101). Malopolskie
  100x100 drops from minutes to seconds; see the pull request for the
  table. Cells take the same paths and statuses as before; matrices of 51
  to 100 targets, now rows, differ from pairwise in the last bits of their
  costs, distances and durations only.

## 0.1.3 (2026-11-17)

Both lines. Package format 1, costing model 1, C ABI version 1, as 0.1.0.

- **Hostile packages are rejected when they open.** A package whose
  landmark names a node it doesn't have made engines panic when they
  computed a profile's own landmarks (Tracked, Emergency); one with a
  negative or NaN edge cost made a route search loop forever rebuilding its
  path. `open` and `from_bytes` now check that landmark nodes are in range,
  that edge costs and durations are non-negative (infinity marks an unusable
  edge), and that edge lengths, landmark scales and the heuristic factors are
  finite and non-negative. Compiled packages are unaffected.
- **Fuzz targets** (`fuzz/`, `cargo fuzz`): `package` (hostile packages,
  re-signed after each mutation), `request` (request JSON through the C ABI
  on Monaco) and `overlay_set` (overlay sets, round-tripped, merged and
  routed with). They found both package bugs above; request JSON and overlay
  sets came through 780,000 and 240,000 inputs clean.
- **The compiler reads PBF with `osm-pbf-ingest` 0.2.0**, which no longer
  depends on `osmpbfreader` at runtime. That drops the unmaintained
  `smartstring`, `futures` 0.1 and related crates, and with them every WTFPL
  and MPL-2.0 dependency: all builds now link only Apache-2.0, MIT, zlib and
  Unicode-3.0 crates. Packages are byte-identical.

## 0.1.2 (2026-11-17)

Both lines. Package format 1, costing model 1, C ABI version 1, as 0.1.0.
The 0.1.0 crates and `wayfarer-package` 0.1.1 are yanked: 0.1.2 renames
public Rust items they use.

- **Rust API (breaking): the mobility mode mask is renamed.** `AresMode` is
  `MobilityMode`, `MobilityProfile::ares_mode_mask` is `mobility_mode_mask`,
  and in `wayfarer-road-graph` `ares_mode_bit` and `ares_mode_mask_from_osm`
  are `mobility_mode_bit` and `mobility_mode_mask_from_osm`; struct fields
  named `ares_mode_mask` are `mobility_mode_mask`. The JSON requests and
  results and the C ABI are unchanged.
- Package bytes change once: the compiler's build report in the package
  metadata names the key `mobilityModeMask` (was `aresModeMask`). Routes,
  matrices and isochrones are identical, and engines of either version open
  packages of the other, but dataset hashes differ from 0.1.0's, so overlay
  sets bound to a 0.1.0 package fall back to their geometry.
- Builds and tests on Windows: the read-ahead hint in
  `PackageFile::prefetch_pages` (`madvise`) is Unix-only (first shipped in
  `wayfarer-package` 0.1.1); the examples' CPU clock and `cancel.c`'s thread
  and sleep calls have Windows versions; tests no longer assume `/`
  separators or overwrite a mapped package; `.gitattributes` keeps text files
  LF.
- On Windows a package that is open can't be replaced: compile to a new path
  and switch the engine over (`CompileRequest::output`, README "Limits").
- `MobilityProfile` documents each profile; the README gains a "Limits"
  section and current size and memory figures.

## 0.1.0 (2026-11-17)

First public release of both lines. Package format 1, costing model 1, C ABI
version 1.

### Compiler (`wayfarer-compiler`, `wayfarer-cli`)

- Compiles an OpenStreetMap `.osm.pbf` extract into one memory-mappable
  routing package (`routing.mrt`): `wayfarer compile <input.osm.pbf> -o
  <routing.mrt>`, or `wayfarer_compiler::compile` in your own process with
  progress, cancellation and a thread count (`CompileJob` for hosts that poll).
  The PBF is read through the [`osm-pbf-ingest`](https://github.com/Limina-Agency/osm-pbf-ingest)
  crate.
- Packages are the same bytes on every platform, build profile, thread count
  and memory profile (`--low-memory`), so a package can be verified by its
  dataset hash wherever it was built. `wayfarer verify` checks every section's
  checksum; `wayfarer info` describes a package.
- Reads OSM the way routers are expected to: default access per highway type,
  per-mode access tags (`foot=yes` on an `access=no` way), `only_*` and
  `no_*` turn restrictions with via ways and conditional restrictions,
  bridges and tunnels that don't join the roads they cross, ferries,
  pedestrian areas, barriers (gates, bollards, locked or not), traffic
  signals, stop and give-way signs, and destination-only roads (`private`,
  `customers`, `destination`, `permit`, `delivery`).
- Compact packages: landmark rows stored only where they can't be derived,
  fixed-point node positions, edge twins instead of stored in-edges, and
  segment fields kept once per attribute class.
- The `wayfarer` binary also routes on a package: `route`, `matrix`,
  `isochrone`, `snap` and `match`, taking the C ABI's request JSON.

### Engine (`wayfarer`, `wayfarer-service`, `wayfarer-ffi`)

- Exact routes with alternatives, waypoints and turn-by-turn guidance
  (roundabout exits, forks, distances to the next maneuver), many-to-many
  matrices, isochrones, map matching, snapping and navigation progress on a
  prepared route.
- Profiles: `Default` (as `Wheeled`), `Wheeled`, `Tracked`, `Emergency`,
  `OnFoot` and `Legged` (on foot, keeping to tracks and paths), with costing
  options (avoid motorways, prefer paved, …); `MobilityProfile` documents
  each. Drives are costed close to driving time and walks close to walking
  time; vehicle durations include junction delay (turns, yielding, stop
  signs, signals).
- Destination-only roads: a drive may start or end on a private, customer,
  destination, permit or delivery road, never pass through one. Emergency
  vehicles and walkers are not confined.
- Dynamic overlays per request, no recompile: blocks, avoidances, slow-go
  multipliers, discounts, speed overrides, opened roads and ad-hoc
  connectors, with time windows. Overlay sets (`wayfarer-overlay-set` files)
  bind edges to a package's dataset hash and fall back to geometry elsewhere.
- Search-and-rescue (SAR) routing for vehicle profiles, off by default:
  relaxations that let a request break turn restrictions, conditional and
  legal closures, one-way rules, barriers and path-width limits at a stated
  cost, go off the network for the last stretch, and report every relaxed
  stretch. Every route reports qualified legal access along it.
- Search: A* with landmark bounds on per-profile cost columns, bidirectional
  with an exact stop, walking through dead ends and degree-2 chains between
  junctions; cooperative cancellation and deadlines on every search.
- C ABI (`wayfarer-ffi/include/wayfarer.h`): open, info and verify a package;
  route, matrix, isochrone, map matching, snap and projection requests as
  JSON (camelCase keys), with cancel tokens; navigation progress on a
  prepared-route handle; the compiler build adds `wayfarer_compile_*`. JSON
  Schemas of every request and result are in `schema/`.
- Route quality against Valhalla 3.9 on Małopolskie is in
  `benchmarks/valhalla/README.md`.

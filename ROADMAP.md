# Wayfarer roadmap to 1.0

Status: proposal, written against 0.1.3 (2026-11-17) and updated on
2026-11-19 after the 0.2.0 release. Section 2.1 says what 0.2.0 shipped.
The plan covers off-network legs blended with the network (sub-lane
`FEAT-X`, 6.2.1) and routing across regions, larger compiles and streamed
cells (lane `REG`, 6.7). The lane tables in section 7 include the
corrections from the feature and regions analyses of 2026-11-19 (code read
against 0.2.0, Poland compiled on a cloud runner), and 7.14 splits the next
releases into work packages that can run in parallel. It is meant to be
argued with: every lane lists what it delivers, why, and what has to be
decided first. Open questions are collected at the end.

## 1. What 1.0 means

1.0 is a promise, not a feature count. At 1.0 we promise that:

1. **The four public surfaces are versioned and stable** (section 4): the Rust
   crates we name as public, the JSON request/result wire format, the C ABI,
   and the routing package format. Within 1.x they change only additively.
2. **Results are trustworthy and predictable**: route quality is measured
   against a reference on more than one region, latency has a published p50
   *and* p95 budget, and packages stay byte-deterministic.
3. **It is safe to embed**: hostile packages, request JSON and overlay sets are
   fuzzed continuously, the C ABI never panics across the boundary, and
   failures are typed, documented errors.
4. **It runs where its users are**: Linux, macOS, Windows, iOS and Android, with
   release artifacts for each and first-party bindings for the main host
   languages.

Breaking changes are cheap now and expensive after 1.0, so the plan front-loads
the API redesign and leaves feature work that is additive for later
releases.

## 2. Where Wayfarer stands (0.1.3)

About 92k lines of Rust in 9 workspace crates, 526 tests, fuzz targets for the
three untrusted inputs, JSON Schemas for every request and result, byte-level
change gates on two regions, and a quality comparison with Valhalla 3.9.

**Strengths to protect**

- On-device compile: Małopolskie (194 MiB PBF) in ~7 s, faster than Valhalla's
  tile build, at ~2 GiB peak. The compiler is a well-behaved library (no
  globals, no prints, cancellable, progress, thread budget).
- One memory-mapped, checksummed, byte-deterministic file. Determinism is
  enforced (`libm`, `clippy.toml`), which almost no other router offers.
- Per-request overlays (no-go, slow-go, discount, open, connectors, time
  windows) and shareable overlay sets bound by dataset hash.
- SAR relaxations with explicit reporting of every relaxed stretch, qualified
  access on every route. This is genuinely unique.
- Exact shortest paths (bidirectional edge-based A* with ALT bounds) checked
  against a plain-Dijkstra oracle (`reference` feature).
- Quality parity with Valhalla: median length/time within 1%, overlap 0.97.

**Known weaknesses (from the README's own Limits and from reading the code)**

- Tail latency: p95 153 ms on Małopolskie against Valhalla's 16 ms. Landmarks
  are 34% of the package and still leave long drives slow.
- Scale: regions only; country- and continent-size extracts are untested.
- No elevation, no time-dependent routing, no traffic, one region per package,
  no incremental update.
- Profiles are a closed enum of six; no bicycle, truck or motorcycle profile,
  although bicycle access and `maxheight`/`maxweight` tags are already read by
  the compiler.
- Guidance strings are English only (an `instruction_key` and parameters exist,
  but no localization layer).
- CI runs on Linux only; there are no macOS release archives.
- The public API surface is very wide and was not designed for stability
  (section 3).

### 2.1 Shipped in 0.2.0 (2026-11-19)

- **The speed-up is chosen: CCH potentials** (PERF-2 to PERF-6). The
  compiler stores a deterministic nested-dissection node order (`CchRank`).
  The engine customizes CCH lower bounds per cost column when a package
  opens or on first use, and every A* is bounded by them: routes, overlays,
  SAR openings, derived columns, matrices. Searches stay exact and are still
  checked against the oracle. On Małopolskie, drives are p95 9.3 ms and walks
  p95 25 ms.
- **Package format 2.** Format 1 is no longer read (recompile), and
  landmarks are gone: 260 → 176 MiB on Małopolskie.
- **Alternatives through via edges** (PERF-7): two alternatives at p95 74 ms.
- **Matrices:** pairwise cells in potential-bounded corridors up to 400
  targets. Bucket many-to-many is removed. 100x100 takes 2.2 s.
- **Platforms and safety:** an open package can be replaced on any Windows
  file system, `_w` C entry points, CI on Windows and i686 (part of FFI-1;
  macOS is still missing), the 32-bit open fix, and header-overflow and
  fuzz-found package crash fixes (#8, #13, #22).

Gaps left against the proposed 1.0 budgets: alternatives p95 74 ms (target
60), a 100x100 matrix in 2.2 s (target 2 s), about 135 MB of RAM for the
potentials on Małopolskie, and 0.7 s from open to the first route. Faster
leg building (turning a found path into geometry, maneuvers and attributes)
is the next speed win (PERF-13). The tail-latency and landmark-size
weaknesses above are resolved.

## 3. Gap analysis

### 3.1 Against mature engines

Reference points: OSRM (CH/MLD, very fast, fixed profiles at build time),
Valhalla (tiled hierarchy, dynamic costing, rich guidance, elevation,
time-dependent), GraphHopper (CH/LM, custom models, elevation, many vehicles),
RoutingKit/CCH (customizable contraction hierarchies).

| Capability | OSRM | Valhalla | GraphHopper | Wayfarer 0.1 | 1.0 target |
|---|---|---|---|---|---|
| Point-to-point route, waypoints, alternatives | ✓ | ✓ | ✓ | ✓ | ✓ |
| Matrix | ✓ fast | ✓ | ✓ | ✓ | ✓, faster for large N |
| Isochrones | – | ✓ polygons | ✓ | ✓ buckets/fragments | ✓ + GeoJSON polygons, multi-origin, reverse |
| Map matching | ✓ | ✓ (Meili) | ✓ | ✓ HMM/Viterbi | ✓ + attribute output |
| Trip optimization (TSP) | ✓ `trip` | ✓ `optimized_route` | ✓ (paid) | – | ✓ (P1) |
| Speed-up technique | CH / MLD | tiled hierarchy | CH / LM | ALT only | CCH potentials (done on `main`) |
| Per-request dynamic costing | limited | ✓ | ✓ custom models | ✓ overlays + options | ✓, one unified model |
| Elevation | – | ✓ | ✓ | – | ✓ (P1) |
| Time-dependent / departure / arrive-by | – | ✓ | partial | one evaluation time | departure + arrive-by (P1) |
| Traffic / historical speeds | ✓ (CSV) | ✓ | ✓ | – | speed overlay input (P2) |
| Bicycle / truck / motorcycle | ✓ (Lua) | ✓ | ✓ | – | bicycle + truck (P1) |
| Custom profiles | Lua at build | JSON costing | custom model | closed enum + options | parametric profiles (P1) |
| Localized guidance, voice, lanes, signs | ✓ | ✓ rich | ✓ | English text, lanes stored | localization + lanes + signs (P1) |
| Encoded polyline output | ✓ | ✓ | ✓ | – | ✓ (P0, cheap) |
| Continent scale | ✓ | ✓ | ✓ | regions | country-scale measured (P1) |
| Routing across regions (portals) | one dataset | ✓ tiles are the regions | one dataset | – | hooks at 1.0, bundles in 1.x (`REG`) |
| Streamed, cached hierarchical tiles | – | ✓ `tile_url`, tile cache | – | – | after 1.0 (`REG`) |
| Off-network start or end blended with the network | – | – (nearest road) | – | approach penalty; SAR straight legs not costed | costed access legs, terrain-aware on foot (`FEAT-X`, P1) |
| Incremental / diff updates | – | partial | – | – | post-1.0 |
| HTTP server | ✓ | ✓ | ✓ | – (by design) | optional, see open questions |
| In-process, offline, on-device compile | – | – | partial | ✓ | ✓ (keep) |
| Deterministic packages | – | – | – | ✓ | ✓ (keep) |
| SAR relaxations, qualified access | – | – | – | ✓ | ✓ (keep) |

### 3.2 Domain-specific gaps (search and rescue)

These are where Wayfarer can lead rather than catch up:

- **Off-network / cross-country movement.** SAR today reaches the last stretch
  off the network. Teams on foot need terrain-aware travel over open ground:
  a cost surface from elevation and land cover, joined to the road graph.
- **Search-area coverage.** Multi-origin isochrones ("everywhere reachable by
  any team within 30 min"), reverse isochrones ("where could the subject have
  come from"), and reachability of a polygon.
- **Assignment and sequencing.** Nearest-k facilities/teams by travel time,
  and multi-stop trip ordering for sweeping a set of points.
- **Overlay exchange.** Overlay sets already bind by dataset hash and fall back
  to geometry; a stable edge reference (section 5.4) would make them survive
  recompiles exactly.
- Off-network movement is planned as sub-lane `FEAT-X` (6.2.1).
- **Terrain-aware walking.** `OnFoot`/`Legged` would gain the most from
  elevation (Tobler-style hiking speed).

## 4. The 1.0 stability contract

| Surface | 0.1 today | At 1.0 | Rule within 1.x |
|---|---|---|---|
| Rust API | 9 crates, all 0.x, wide re-exports | A named set of public crates (proposal below); the rest internal | semver; `cargo-semver-checks` in CI |
| JSON wire format | Follows the engine crate; no version field | `apiVersion` in requests/results; schemas published per version | Additive only: new optional fields, new enum values (clients must tolerate unknowns) |
| C ABI | ABI 1 | ABI 2, designed once (FFI-4) | Additive only: new functions, never changed signatures |
| Package format | Format 1, reads N-1 for one release (format 2 on `main`, reads only 2) | Format 3 frozen at 1.0, with an optional-section table and region identity so regions and cells come in 1.x without a new format (6.7) | Every 1.x engine reads every package a 1.x compiler wrote |
| Costing model | Model 1; older models re-costed | Versioned separately | A model change never needs a new engine to open old packages |
| Determinism | Same bytes on every platform | Same | Same |

**Proposed public crates at 1.0:** `wayfarer` (engine), `wayfarer-compiler`,
`wayfarer-ffi` (as the C ABI; its Rust API is not public), `wayfarer-cli` (as a
command line), and `wayfarer-package` read-only inspection. `wayfarer-costing`,
`wayfarer-road-graph` and `wayfarer-line-network` become internal: published
only because cargo requires it, documented as unstable (`#[doc(hidden)]`
modules or a `-internal` suffix), or merged into the compiler.
`wayfarer-service` is either folded into `wayfarer` (sources and caching are a
thin layer) or kept public with a small surface; see open questions.

## 5. API redesign (breaking, before 1.0)

Each item is grounded in the current code; file paths are as of 0.1.3.

### 5.1 Shape of the Rust API

- **No future-proofing today.** No type in the workspace is
  `#[non_exhaustive]`. Every public struct with public fields and every enum
  (`RouteError`, `MobilityProfile`, `RouteManeuverKind`, …) is a breaking
  change waiting to happen. Mark result types and enums `#[non_exhaustive]`;
  build requests through constructors and builders.
- **Flat root with ~125 re-exports** (`wayfarer/src/lib.rs`). Group into
  modules: `route`, `matrix`, `isochrone`, `matching`, `navigation`, `overlay`,
  `sar`, `package`. Keep a small prelude.
- **`Route*` prefix on everything** (`RouteMatrixRequest`,
  `RouteIsochroneRequest`, `RouteIsochroneContour`, …). Inside modules, drop it:
  `matrix::Request`, `isochrone::Contour`.
- **`#[doc(hidden)] pub mod internals`** is used by the service, harnesses and
  tests. Move those users behind a non-default feature or into the crate, so
  the published API is what the docs show.
- **`wayfarer-road-graph` `pub use *` from ten modules** (~130 public items)
  and `wayfarer-package` `#[doc(hidden)] pub mod request`: internal plumbing
  on public crates.
- **Legacy names**: `MobilityProfile::Default` (identical to `Wheeled`),
  `TURN_COST_SECONDS_PER_LEGACY_UNIT`, `U_TURN_LEGACY_COST`,
  `RouteGuidanceQualityFlag::SsvgConnector`, comments about a "former
  tile-backed" search. Remove or rename before freezing.

### 5.2 Requests

- **Two overlay systems and a dozen edge-level knobs.** A route can be
  changed through `RouteOptions` (`blocked_edge_ids`, `blocked_edge_ranges`,
  `avoided_edge_ids`, `avoided_edge_ranges`, `warning_edge_ids`,
  `warning_edge_ranges`, `edge_cost_multipliers`,
  `edge_cost_multiplier_ranges`, `edge_speed_overrides`,
  `edge_speed_override_ranges`, `opened_edge_ids`), `RouteOverlay` (geometry or
  edges, with a policy), `RouteDynamicOverlay` (split edges and connectors),
  `RouteRequest::leg_dynamic_overlays`, and overlay sets in the service. One
  model at 1.0: an `Overlay` is a *target* (geometry, edge refs, or edge
  ranges) plus an *effect* (block, avoid, warn, multiply, set speed, open,
  connect) plus optional profile filter and time window. The edge-id fields
  become sugar or disappear.
- **Serde aliases.** 45 `alias = "snake_case"` attributes accept a second
  spelling of the same keys. Remove them so the wire format has one spelling.
- **Unknown fields are silently ignored on route, matrix and isochrone
  requests**, but rejected on snap, map-match and project
  (`deny_unknown_fields` in `wayfarer/src/queries.rs` only). A misspelled
  option should fail. Pick one rule for every request; given additive
  evolution, reject unknown fields in requests and tolerate them in results.
- **Profiles as a closed enum.** Move to a profile *base* (vehicle, foot,
  bicycle, …) plus typed parameters (dimensions, weights, speed caps,
  preferences), so truck, bicycle and the existing `Tracked`/`Legged` are
  presets, not variants. `RouteCostingOptions` (16 fields of `avoid_*`
  weights) folds into the same structure.
- **Time.** `departure_at_ms` on the route request and
  `conditional_restriction_eval_at_ms` on costing options both set "when".
  One `time` object: `depart_at` | `arrive_by`, with the evaluation-time
  semantics documented.
- **Limits.** `MAX_ROUTE_LOCATIONS = 34` and the request bounds in `limits`
  should be documented on the wire (schema `maxItems`) and configurable per
  engine.

### 5.3 Results

- **`RouteResult` is everything at once**: 20 fields including diagnostics,
  recosts, cost breakdowns, overlay diagnostics, explanations and per-edge
  attributions. `schema/route-result.json` is 100 KB. Split into a lean core
  (geometry, distance, duration, legs, maneuvers) and opt-in detail selected by
  an `include` list in the request. Diagnostics already "carry no
  compatibility promise": put them under one explicitly unstable key.
- **Geometry encoding.** Add encoded polyline (precision 6) and GeoJSON as
  options; arrays of `{lat, lng}` objects are large over FFI.
- **Inconsistent fallibility.** `route` returns `Result<_, RouteFailure>`,
  `isochrone` returns `Result<_, RouteError>`, `matrix` returns a bare
  `RouteMatrixResult` with per-cell statuses. Keep per-cell status for
  matrices, but make request-level failures (bad request, cancelled, package
  not ready) a `Result` everywhere.
- **Edge ids in results.** `edge_ids` are build-local sequential numbers
  (`RoadGraphBuilder::next_edge_id`); they are meaningless across packages.
  See 5.4.

### 5.4 Stable edge references

Edge ids are assigned in build order, so they change whenever the extract
changes. Overlay sets cope by binding to a dataset hash and falling back to
geometry. For 1.0, define a stable reference that survives recompiles of a
newer extract of the same area: OSM way id + from/to OSM node ids + direction,
with a fraction for ranges, and a matching resolver in the engine. Keep
compact edge ids as a fast path when the dataset hash matches. This is what
makes shared closures and field reports dependable over days.

Design the reference so it can also name a region (a package in a bundle or
a cell, 6.7): `(region, way, from, to, direction, fraction)`, with the region
optional inside one package. Adding it after 1.0 would change the wire
format.

### 5.5 Errors

- One error vocabulary across Rust, JSON and C: stable `code` strings
  (`no_candidate`, `no_path`, `disconnected`, `cancelled`, …) with typed
  details, documented in one table. Today the C ABI has numeric statuses,
  JSON has `{code, message}`, and Rust has `RouteError` with
  `&'static str` reasons that cannot carry data.
- `RouteError::NoCandidate { endpoint: &'static str }` should say which
  location (index) failed, so a host can show it.

### 5.6 C ABI 2

- Keep the JSON-in/JSON-out design: it is what keeps the ABI small.
- Fold `wayfarer_route` / `wayfarer_route_with_cancel` (and matrix, isochrone)
  into one call each that takes an optional cancel token.
- Add an options struct to `wayfarer_open` (memory profile, limits, logging
  callback).
- Version the JSON wire separately from the ABI (`apiVersion`).
- Keep `abi-keys.json` and the schema test; add a header-diff check in CI.
- Open takes a *source*: a path or bytes today, a region bundle later (6.7),
  so bundles don't need a new entry point in 1.x.

### 5.7 Compiler and CLI

- **Two compile request formats**: `CompileRequest` (`input`, `output`,
  `memory: standard|low`) and the host protocol's own (`inputPbfPath`,
  `routeArchivePath`, `buildProfile: desktop_full|mobile_operational`). Keep
  `CompileRequest` and deprecate the other (the protocol already accepts
  both).
- **Compiler options** to expose before 1.0: which profiles get baked columns,
  landmark count (package size vs speed), bounding polygon clip, optional
  elevation source.

### 5.8 Documentation drift to fix now

- `wayfarer-compiler/README.md` says an engine opens only packages of its own
  costing model; the engine docs and the root README say older models are
  re-costed. The engine is right; fix the compiler README.
- `wayfarer-ffi/README.md` shows a route request with `origin`/`destination`;
  the request type and `examples/route.c` use `locations`. Because unknown
  fields are ignored, that snippet fails with a missing-field error rather
  than a clear one.

## 6. Systems and features

Priorities: **P0** required for 1.0, **P1** strongly wanted for 1.0,
**P2** after 1.0 unless a user needs it sooner.

### 6.1 Performance (P0)

Status on `main`: CCH potentials were chosen and shipped with format 2 (2.1).
What is left is PERF-13 to PERF-16 (7.4).

- **Speed-up technique.** Close the p95 gap (153 ms vs 16 ms). Candidates:
  - *Customizable contraction hierarchies (CCH)*: metric-independent order at
    compile time, fast customization per profile, and per-request changes can
    be handled by partial re-customization or by searching the overlay patch
    over the hierarchy. Best fit for dynamic costing; most work.
  - *Core-ALT / hierarchy on top of ALT*: contract degree-2 and low-importance
    nodes, run ALT on the core. Less work, smaller win.
  - *Multi-level partitions (MLD-style)*: also fits dynamic metrics and
    makes country scale possible.
  Run a spike on Małopolskie with each before choosing (PERF-2). The
  exactness guarantee and the `reference` oracle stay.
- **Package size.** Landmarks are 34%. A hierarchy may replace most landmark
  rows; otherwise make landmark count a compile option.
- **Matrix scaling.** Bucket-based many-to-many on the hierarchy for
  1000×1000.
- **Latency budget in CI.** p50 and p95 on Małopolskie as gates, not only
  "within 3% of the previous release".

### 6.2 Routing features

- **Elevation (P1).** DEM ingest at compile time (SRTM / Copernicus GLO-30,
  tiled GeoTIFF), stored per edge as climb/descent; costing for foot (Tobler),
  bicycle, and vehicle grade; elevation profile in route results. Keep it
  deterministic (fixed-point, `libm`).
- **Time-dependent routing (P1).** Departure and arrive-by; conditional
  restrictions and overlay windows evaluated along the route, not once per
  request. Historical speed profiles as an optional input later (P2).
- **Profiles (P1).** Bicycle (the compiler already reads bicycle access) and
  truck (dimensions and weights: `maxheight`, `maxweight` are parsed but not
  costed). Motorcycle is a preset (P2).
- **Trip optimization (P1).** TSP with fixed or free start/end over a matrix;
  simple and exact for small N, heuristic beyond.
- **Isochrones (P1).** GeoJSON polygon output, multi-origin, reverse
  (arrive-at), distance or time.
- **Locate / nearest-k (P1).** Nearest k targets by travel time from one
  source, reusing the matrix search.
- **Map matching output (P2).** Per-sample attributes (road, speed, name),
  like Valhalla's `trace_attributes`.
- **Off-network legs blended with the network (P1, staged).** See 6.2.1.
  The full terrain cost surface is its P2 end.
- **Traffic input (P2).** Speed overlays already exist; a bulk speed-file
  input would cover live or historical traffic without a recompile.

#### 6.2.1 Off-network legs blended with the network (`FEAT-X`)

A team on foot, or a vehicle starting in a field, needs to know where to
join the network so the whole trip is fastest, not which road is nearest.
The pieces exist on `main` but don't blend:

| Mechanism | What it does | What's missing |
|---|---|---|
| Endpoint candidates with approach costs (`approach_cost_of`, `wayfarer/src/package/search.rs`) | Several snap candidates per endpoint compete; a farther one pays a penalty (30 m window, then 10× per metre) | A tie-breaker, not travel time: not in the duration, no geometry for the approach |
| R7 `off_network` (`wayfarer/src/sar.rs`) | A straight snap leg up to 10 km, reported | Vehicles only; left out of cost and duration |
| `RouteLegMode::Direct` (`wayfarer/src/direct_leg.rs`) | A great-circle leg at a constant off-road pace | The whole leg is off-network; no join point is chosen |
| Host connectors (`CrossCountry`, `Endpoint`) | Exact in the patch search and the oracle | The host computes the cost; ends only at endpoints or points on edges |

The connector machinery is the join point: it is exact and the oracle checks
it. The plan grows it in stages, each usable on its own:

1. **X1, costed access legs (P1, additive).** An endpoint option
   `offNetwork: {maxRadiusM}`, off by default. Every search already seeds
   several snap candidates per endpoint with an approach cost (routes, the
   patch search, matrix rows, isochrones, alternatives, the oracle); X1
   turns that tie-breaker into real, reported time and distance, so the
   search picks the entry by total time. It is built on these seeds, not on
   engine-made `Endpoint` connectors: matrices and isochrones strip
   connectors, an origin connector disables candidate seeds in the patch
   search, connector time always equals cost, and past 256 gates the CCH
   bounds fall back to the straight-line bound. Access costs go into the
   potentials as offsets (`GatedPotential` takes them), which keeps the
   bounds tight. The costing crate's off-road pace is too fast for walkers
   (Legged off-road is faster than on a walkway), so access legs carry a
   cost factor on top of time (about 2 for walkers, 1.5 for vehicles). The
   access stretch is a step with geometry, distance and duration, counted
   in the totals and flagged. It works for walkers, who have no relaxation
   today, and for matrix and isochrone origins. Matrix rows lose their
   potentials beyond 8 target nodes, so the candidate count is chosen after
   measuring 100x100 with X1 on.
2. **X2, barriers (P1).** Access legs don't cross water, rivers without a
   ford, motorways, railways, fences or walls. Motorways and trunk
   motorroads come first and need no new data (X2-lite: derived from edges
   already in the package through the request grid). The rest needs new
   data, because the compiler keeps only highway and ferry ways: a compact
   barrier-line index as a format-3 optional section; candidates whose
   straight line crosses a barrier are dropped.
3. **X3, terrain-aware pace (P1 for SAR, format 3).** Access legs are priced
   by slope (Tobler) on a coarse DEM grid in the package (FEAT-T4) and by
   land cover read from the same PBF (wood, scrub, wetland, scree). Land
   cover is its own item (X3a, L): the compiler assembles no polygons today,
   and forests and lakes are mostly multipolygon relations, rasterized at
   25–50 m into 4-bit classes.
4. **X4, least-cost access paths (P2).** A raster search: a fixed-point
   friction raster (DEM and land cover) within 3 km of the point, a Dijkstra
   from the point, and the cheapest cell on each reachable edge fed to the
   search as a candidate seed (like X1) with the traced path as geometry.
   Integer costs and ties broken by a global cell index keep it
   deterministic. EXP-2 prototyped nothing yet.
5. **X5, off the network beyond the endpoints.** X5a (P2): isochrones that
   spread off the network from every reached node (team coverage on foot),
   a joint network-and-raster Dijkstra. X5b (1.x, research): routes that
   leave the network mid-route and rejoin it; the CCH bounds can't cover a
   raster, so this needs a new bound model.

X1 is additive and can land any time. R7 then becomes X1 with SAR reporting
(FEAT-X6): a `costed` opt-in with X1, then the default flips inside the API
window with a migration note. X2 proper, X3a and the DEM grid need sections
and land as format-3 optional sections. Nothing here needs the regions work,
and the regions work doesn't need it, but two hooks must be in the 1.0
formats: connectors and endpoint candidates are region-qualified on the
wire, and the DEM and raster grids use one global lattice (fixed arc-second
cells anchored at 0°, 0°), so neighbouring regions tile exactly.

### 6.3 Guidance (P1)

- Localization: message catalogs keyed by the existing `instruction_key` and
  `instruction_parameters`, shipped for a handful of languages, with English
  as the fallback; hosts can bring their own catalog.
- Lanes from `turn:lanes` (already stored), signs from `destination*` tags,
  exit numbers, and voice-ready instruction timing.
- Units and rounding per locale.

### 6.4 Data and compiler

- **Scale (P1).** Measure and fix the compiler at country scale (Poland,
  Germany): memory (external sorting where needed), compile time, package
  size, open time. Publish the numbers like the Małopolskie ones.
- **Clip polygon and multi-extract input (P1).** Compile a polygon of a larger
  extract, or several extracts into one package (cross-border regions).
- **Regions, portals and streamed cells.** See 6.7 and lane `REG`.
- **Incremental updates (P2).** Per region once bundles exist (REG-6);
  before that, OSM change files applied to a package, or
  partial rebuilds per partition (easier once a partitioning exists).
- **Field data (P2).** Roads and tracks collected in the field as GeoJSON
  input next to the PBF, or promoted from overlays.

### 6.5 Platforms and bindings (P0 for platforms, P1 for bindings)

- CI on Linux, macOS and Windows; cross-builds for iOS and Android checked in
  CI.
- Release artifacts: macOS archives (missing today), an XCFramework for
  Apple platforms and an AAR for Android.
- First-party bindings generated from the C ABI and the JSON Schemas: Swift
  and Kotlin first (field apps), Python second (analysis, benchmarks),
  TypeScript/WASM third (browser offline). Each with a small typed request
  layer over the schemas.
- MSRV declared (1.95 today) and tested.

### 6.6 Quality, safety and operations (P0)

- Fuzz targets run on a schedule (nightly, time-boxed) with corpus kept.
- The Małopolskie gate runs nightly, not only on demand.
- A second and third region in the quality comparison (a dense city, a
  mountainous rural region), so quality claims aren't one region's.
- `cargo-semver-checks`, schema diff and C header diff on every PR.
- `tracing` spans with stable names for request phases, off by default.
- `#![deny(missing_docs)]` on public crates.
- A security support policy for 1.x (fixes for the latest 1.x minor).

### 6.7 Regions, portals and streamed cells (`REG`)

Today one region is one `.mrt`. Three needs push past that: larger areas (a
country, a search area across a border), devices that can't hold or
customize a large package, and a server that answers for a continent and
hands devices only the parts they need, the way Valhalla serves tiles.

**What the current architecture already gives**

- Deterministic, content-hashed packages: a unit to compile, cache and
  download whose bytes are checked by hash, with no signing service.
- The CCH order is a nested dissection. Its separators are a multi-level
  partition of the graph that the compiler already computes, so cells and
  their boundaries fall out of it.
- An edge-based A* with exact bounds, and a matrix search to compute
  distances between boundary edges.
- Every node's OSM id and every shared info's OSM way are stored, so two
  packages can agree on a seam without coordination.

**What it doesn't (yet)**

- Edge ids, component ids, the request grid and the CCH order are global
  and sequential. Anything crossing a boundary needs `(region, local)` ids
  and stable references (API-5).
- Potentials need the whole graph's elimination tree, and their memory grows
  with the graph. Measured on Poland with the 0.2.0 compiler (cloud runner,
  4 cores): compile 7 min at 10.8 GiB peak, a 1.97 GB package, and an
  engine that needs 1.6 GiB of potentials plus 0.9 GiB of search arrays
  sized to the whole graph on its first route (8 s to prepare). The node
  order is the compile's scaling problem (40 %, about n^1.5). A single
  package reaches a country on a desktop or server, not a continent, and
  not a country on a phone.
- The engine, the service and the C ABI open one file; results and overlay
  sets bind to one dataset hash.
- Features that need whole shortest-path trees (via-edge alternatives,
  isochrones, per-request SAR columns) need care across boundaries.

**Options**

| Option | What | Exact? | Cost |
|---|---|---|---|
| A. Bigger single packages | Country compile: slimmer build graph, streamed shared infos, clip polygon, several extracts (FEAT-D1 to D3); small field packages clipped from it | Yes | M–L; metadata only |
| B. Region bundles with portals | Regions compiled on their own, joined at portal edges; a small bundle index holds the graph between regions | Yes (two-level, MLD-style) | L–XL |
| C. Multi-level cells, streamed | B applied recursively to a continent build: 2–3 levels of cells from the nested dissection, served immutable by hash and cached on devices | Yes | XL |
| C'. Valhalla-style grid tiles | Fixed geographic grid per road-class level, lazy loading, LRU tile cache | No: the road-class hierarchy is a heuristic | L |

Recommendation: A before 1.0. B and C after 1.0, as additive 1.x releases,
with every hook they need in place before the freeze. Not C': it gives up
exactness, which the oracle guards and which is one of Wayfarer's
strengths, while the nested-dissection cells already give a multi-level
structure that stays exact.

**Portals (B)**

- *Cut and ownership.* Each region compiles with a boundary polygon (admin
  areas, a drawn search area, or a grid) and a halo of its neighbour's data
  (complete ways within a few km), because way splitting at shared nodes,
  crossing repair, plaza synthesis and inference all look at neighbouring
  geometry. Nodes have owners; a portal is an edge whose ends have
  different owners, kept in both packages with the foreign end as a stub,
  and named by stable reference (OSM way, from and to OSM nodes,
  direction), so neighbours compiled apart still agree. The cut follows a
  minimum edge cut within a band around the polygon's border (±5 km cuts
  portals 4–5.5× against a straight or legal border), and never passes
  through synthetic ids, via-way restrictions, ferries or barrier nodes.
  Portals are edges, not node separators, so turn costs and restrictions at
  the seam stay inside one region's search.
- *Per region and stored metric:* a portal table and a portal-to-portal
  distance matrix (a clique), computed after customization with the existing
  matrix search, stored as optional sections or computed at open.
- *Bundle index:* regions, extents, hashes, portal pairs, and the overlay
  graph of cliques with its own CCH order, so the overlay search is bounded
  by potentials too.
- *Query, two searches.* (b) **Potential-guided, built first:** every
  region keeps its own CCH; portal *bounds* (CCH distances between portal
  nodes, cheap) give each region's portals an offset towards the targets,
  and the region's potential is `CchPotential::with_offsets(targets ∪
  portals)`, which exists today. The A* runs edge-based over the regions
  along the corridor, so it stays exact for everything a request can carry
  (overlays, profile options, SAR relaxations, conditionals, connectors) and
  the oracle argument is unchanged. (a) **Overlay-exact, the fast path:**
  exact cliques per region and stored column, full searches only in the end
  regions; exact only for plain stored-column requests, and it falls back
  to (b) for anything else.
- *Updates:* recompiling one region changes only its package and the index;
  neighbours keep their bytes. This is EXP-6's per-partition rebuild without
  renumbering the rest.

**Streamed cells and a server (C)**

- *Cells:* a continent build cut at the nested dissection's levels (for
  example level-0 cells of 50–100 k nodes, then groups of them, then the
  continent), each a deterministic blob named by its hash, plus one overlay
  per level.
- *Server* (`wayfarer-server`, EXP-4 graduated): the JSON API over a bundle,
  with an LRU of memory-mapped cells like Valhalla's tile cache, and an
  endpoint serving immutable cells and overlays by hash that any HTTP cache
  or CDN can hold.
- *Devices:* fetch the overlay and the cells for an area or a corridor,
  check them by hash, keep a cache within a byte budget, and route offline
  over what they hold. An intermediate cell is needed only for geometry and
  guidance, not for the search (its clique is in the overlay), so a device
  can route across a country with the overlay and the two end cells and
  fetch the middle ones for display.
- *On-device compile stays:* a device can compile its own region and join it
  to a bundle, because portals are named by OSM references, not by a server.

**Where it sits**

- *Before 1.0, so 1.x stays additive:* region-ready stable references, and
  `(region, edge)` in results and overlay sets (API-5, API-6, REG-4); format
  3 with a tagged optional-section table, region identity, an identity hash
  separate from the content hash, and reserved portal and cell sections
  (REG-3); cell-major numbering in the format-3 compiler (REG-13); C ABI 2
  open taking a source (FFI-4); the single-package country compile measured
  and fixed (option A, REG-1, REG-2); search state sized by what a search
  touches (REG-12, no format change); and the potential-guided search
  prototyped across a seam (EXP-5b) before format 3 is frozen.
- *After 1.0:* the paged package for devices (stored potentials, cell
  directory, lazy validation: REG-10, REG-13), bundles (B), cells and
  streaming (C), the server, per-region updates (REG-5a to REG-9).

## 7. Lanes and releases

The work runs in parallel **lanes**. Each lane has its own ordered backlog and
can move at its own pace; a **release** (0.2 up to 0.99) ships whatever items
across lanes are ready. The version number doesn't name a phase; a small set
of **gates** at fixed versions says when breaking changes stop.

### 7.1 Rules

- **Items have ids** (`PERF-3`, `FFI-2`, `FEAT-T1`, …) so issues, PRs and the
  changelog can point at them. Each item says its priority (P0/P1/P2,
  section 6), what it depends on, and whether it breaks anything.
- **A release is small and can pull from several lanes.** Every 0.x minor is
  a breaking release in cargo's eyes, so additive-only releases are still
  minor bumps; patches (`0.x.y`) are fixes only.
- **Breaking items carry a migration note** and, where possible, deprecate in
  one release and remove in the next.
- **At most one package-format bump per ten versions**, and the engine keeps
  reading the previous format (section 4).
- **Lanes don't block each other** except where a dependency is written down
  (7.12).

### 7.2 Gates

| Gate | From | What it means |
|---|---|---|
| Tooling on | by 0.5 | Semver, schema and C header diffs run on every PR (informational) |
| API redesign window | 0.10 – 0.39 | The API lane's breaking items land here, one area per release |
| Breaking cut-off | 0.80 | After this, only the hardening review may break anything |
| Format 3 | one release, 0.12 – 0.39 | The only package-format bump left before 1.0 (format 2 shipped in 0.2). It carries the changes to existing layouts and the optional-section table; content sections (elevation, barriers, land cover, bicycle data, cells) follow as optional sections without another bump (7.12) |
| Freeze | 0.90 | API, wire format, C ABI, package format and costing model frozen; diffs become blocking; releases are candidates |
| 1.0 | — | Stability contract (section 4) published and enforced |

### 7.3 Lane overview

| Lane | Id | Goal | Breaks things? |
|---|---|---|---|
| Performance | `PERF` | Close the p95 gap, smaller packages, fast matrices | Package format |
| FFI, bindings and portability | `FFI` | C ABI 2, every platform, first-party bindings | C ABI once |
| API and stability | `API` | The redesign in section 5, then the contract | Yes, inside the window |
| Features | `FEAT-*` | New capabilities, in sub-lanes | Mostly additive |
| Quality and safety | `QA` | Gates, fuzzing, regions, security | No |
| Docs and developer experience | `DOC` | Docs that match the code, guides, examples | No |
| Regions and scale | `REG` | Bigger compiles, region identity, portals between packages, streamed cells and a server | Package format 3 and wire references before 1.0; additive in 1.x |
| Experimental | `EXP` | Ideas that may never ship, behind flags | Never (no promise) |

### 7.4 Lane: Performance (`PERF`)

| Id | Item | Pri | Depends on |
|---|---|---|---|
| PERF-1 | Latency p50/p95 recorded per nightly run on Małopolskie | P0 | QA-2 |
| PERF-2 | Speed-up spike: CCH vs core-ALT vs partitions, written decision with numbers (latency, size, compile time, how overlays are searched) | P0 | PERF-1 |
| PERF-3 | Package format 2 for the chosen technique; format 1 still read | P0 | PERF-2 |
| PERF-4 | Deterministic hierarchy build in the compiler; compile time and memory gated | P0 | PERF-3 |
| PERF-5 | Point-to-point on the hierarchy, exact against the `reference` oracle; p95 under 30 ms on Małopolskie (proposal) | P0 | PERF-4 |
| PERF-6 | Overlays and SAR relaxations on the hierarchy, still exact | P0 | PERF-5, API-6 |
| PERF-7 | Alternatives on the new search | P1 | PERF-5 |
| PERF-8 | Bucket many-to-many; 1000×1000 measured | P1 | PERF-5 |
| PERF-9 | Isochrones and map matching on the new structure where it helps | P2 | PERF-5 |
| PERF-10 | Package size: landmark rows trimmed or optional; not larger than 0.1 | P1 | PERF-5 |
| PERF-11 | p50/p95 budgets enforced in CI | P0 | PERF-5 |
| PERF-12 | Moved to REG-1 and REG-2 | — | — |
| PERF-13 | Faster leg building: a found path to geometry, maneuvers and attributes (profile first) | P0 | — |
| PERF-14 | Alternatives p95 ≤ 60 ms on Małopolskie (74 ms on `main`) | P1 | PERF-13 |
| PERF-15 | 100x100 vehicle matrix ≤ 2 s (2.2 s on `main`); 1000x1000 measured | P1 | PERF-13 |
| PERF-16 | Potentials memory (about 135 MB on Małopolskie) and open-to-first-route time (0.7 s): leaner topology and weights, columns customized lazily, or customized weights as an optional section (bigger package, faster open on mobile) | P1 | — |

Status on `main` (2026-11-19): PERF-2 to PERF-7 and PERF-10 are done (CCH
potentials; format 2, with format 1 dropped rather than still read;
via-edge alternatives; 176 MiB on Małopolskie). PERF-8 changed: bucket
many-to-many was removed, because pairwise cells bounded by the potentials
were faster. PERF-5's 30 ms target is met (drives p95 9.3 ms). Leg
building is 57–69 % of drive time on Małopolskie (perf analysis in
progress, 2026-11-19), so PERF-13 leads 0.3; PERF-13 to PERF-16 will be
re-scoped from that analysis. REG-12 (7.8) covers the search arrays'
share of memory.

### 7.5 Lane: FFI, bindings and portability (`FFI`)

| Id | Item | Pri | Depends on |
|---|---|---|---|
| FFI-1 | CI on macOS and Windows, MSRV job | P0 | — |
| FFI-2 | iOS and Android cross-builds checked in CI | P0 | FFI-1 |
| FFI-3 | Release archives for macOS; XCFramework; AAR | P0 | FFI-2 |
| FFI-4 | C ABI 2: one call per request with an optional cancel token, open options (memory profile, limits, log callback), `apiVersion` separate from ABI (5.6) | P0 | API-3, API-4 |
| FFI-5 | Header diff in CI next to `abi-keys.json` | P0 | QA-1 |
| FFI-6 | Encoded polyline (precision 6) and GeoJSON geometry options | P0 | — |
| FFI-7 | Swift binding over the C ABI and schemas | P1 | FFI-4 |
| FFI-8 | Kotlin binding | P1 | FFI-4 |
| FFI-9 | Python binding | P1 | FFI-4 |
| FFI-10 | TypeScript types from the schemas | P2 | API-4 |
| FFI-11 | Examples per binding, tested in CI | P1 | FFI-7..9 |
| FFI-12 | One compile request format; host protocol's own format deprecated (5.7) | P1 | — |

### 7.6 Lane: API and stability (`API`)

The redesign of section 5, one area per release, inside the 0.10 – 0.39
window. Order puts the foundations first and overlays after the speed-up
decision.

| Id | Item | Pri | Depends on |
|---|---|---|---|
| API-0 | Decisions recorded: public crate set, release lines, compatibility window, audience (open questions 1, 4, 5, 6); deprecation warnings added | P0 | — |
| API-1 | Modules, `Route*` prefixes dropped, prelude, `#[non_exhaustive]`, legacy names removed (5.1) | P0 | API-0 |
| API-2 | Internal crates hidden or merged; `internals` behind a feature; `wayfarer-service` folded or trimmed | P0 | API-0 |
| API-3 | One error vocabulary, consistent `Result` (5.5, 5.3) | P0 | API-1 |
| API-4 | Wire rules: `apiVersion`, aliases removed, one unknown-field rule (5.2) | P0 | — |
| API-5 | Stable edge references and resolver (5.4) | P0 | — |
| API-6 | Unified overlay model (5.2) | P0 | API-5, PERF-2 |
| API-7 | Profile base + parameters; costing options folded in; one time object (5.2) | P0 | API-1 |
| API-8 | Lean results with `include`; diagnostics under one unstable key (5.3) | P1 | API-1 |
| API-9 | Compiler options: baked profiles, landmark count, clip polygon (5.7) | P1 | — |
| API-10 | Migration guide 0.1 → current, kept up to date per breaking release | P0 | — |
| API-11 | Final API review: every public item read once more (at 0.80) | P0 | all above |

### 7.7 Lane: Features (`FEAT`)

Sub-lanes run independently of each other.

**`FEAT-T` Terrain and elevation**

| Id | Item | Pri | Depends on |
|---|---|---|---|
| FEAT-T0 | Incline costing scaffold on OSM `incline` (format 2, opt-in), plumbing for T2; `incline_percent` in edge attributes | P2 | — |
| FEAT-T1 | DEM ingest at compile time (SRTM HGT, GeoTIFF; integer sampling), per-segment climb, descent and Tobler factor (about 6 % of the package). Reader and sampler behind `experimental-elevation` first; storage as a format-3 optional section | P1 | REG-3 for storage |
| FEAT-T2 | Elevation costing on foot (Tobler) for `OnFoot` and `Legged` (costing model 2 only in packages with elevation); elevation profile in results | P1 | FEAT-T1 |
| FEAT-T3 | Vehicle grade costing | P2 | FEAT-T1 |
| FEAT-T4 | Coarse DEM grid (3″, global lattice) in the package for off-network legs | P1 | FEAT-T1, REG-3, REG-11 |
| FEAT-T5 | Height lookup query on the DEM grid | P2 | FEAT-T4 |

**`FEAT-X` Off-network legs blended with the network** (6.2.1)

| Id | Item | Pri | Depends on |
|---|---|---|---|
| FEAT-X1 | Costed access legs (M): endpoint `offNetwork` option on the existing candidate seeds, a cost factor on top of time, access costs as potential offsets, the stretch counted in totals; walkers too; matrix and isochrone origins | P1 | approach offsets (QW-3), two-phase snapping (QW-2) |
| FEAT-X1b | Automatic choice between a direct and a routed leg | P1 | FEAT-X1 |
| FEAT-X2a | X2-lite: access legs don't cross motorways and trunk motorroads, derived from package edges (no format change) | P1 | FEAT-X1 |
| FEAT-X2 | Access legs don't cross water, rail or fences, from a barrier-line optional section (M–L) | P1 | REG-3 |
| FEAT-X3a | Land cover: polygons and multipolygons assembled in the compiler, rasterized into an optional section (L) | P1 | REG-3, REG-11 |
| FEAT-X3 | Terrain-aware access pace: Tobler on the coarse DEM grid, land cover (M) | P1 | FEAT-T4, FEAT-X3a |
| FEAT-X4 | Least-cost access paths over a local friction raster, fed as candidate seeds (L) | P2 | FEAT-X3 |
| FEAT-X5a | Isochrones spreading off the network (L) | P2 | FEAT-X4 |
| FEAT-X5b | Open-ground shortcuts mid-route (XL, research, 1.x: needs a bound model beyond CCH gates) | P2 | FEAT-X4 |
| FEAT-X6 | R7 `off_network` rebuilt on X1: `costed` opt-in with X1, default flipped in the API window with a migration note (S–M) | P1 | FEAT-X1, API window |

**`FEAT-P` Profiles**

| Id | Item | Pri | Depends on |
|---|---|---|---|
| FEAT-P0a | Bug: bicycle-only ways open to walkers | P0 | — |
| FEAT-P0b | Bug: conditional keys bind the wrong profiles (`bicycle:conditional` closes walkers); `destination`/`private` treated as closures | P0 | — |
| FEAT-P0c | Bug: mode-specific turn restrictions (`restriction:hgv`, `except=motorcar`, …) ban the turn for every profile | P0 | — |
| FEAT-P0d | Bug: busways open to cars | P0 | — |
| FEAT-P1 | Bicycle preset: needs a profile slot (u16 access with a slot table) | P1 | format 3, API-7 (wire shape) |
| FEAT-P2 | Truck preset: a Wheeled-derived column from the format-2 data (height, width, weight, axle load, `maxspeed:hgv`, HGV access); `restriction:hgv`, `maxlength`, hazmat in format 3 | P1 | API-7 (wire shape only) |
| FEAT-P3 | Motorcycle preset | P2 | FEAT-P2 |
| FEAT-P4 | Baked profiles as a compile option | P1 | — |

**`FEAT-W` Time**

| Id | Item | Pri | Depends on |
|---|---|---|---|
| FEAT-W0 | One evaluation time in local time: `departure_at_ms` drives conditionals, package `timeZone` (POSIX rule) or a request offset, per-leg departure; parser fixes (`;` in parentheses, `AND`, `24:00`) | P0 | — |
| FEAT-W1 | Depart-at with restrictions and overlay windows evaluated along the route | P1 | FEAT-W0, API-7a |
| FEAT-W2 | Arrive-by | P1 | FEAT-W1 |
| FEAT-W3 | Bulk speed layer (engine-level derived column) for traffic and historical speeds | P2 | API-6 |
| FEAT-W4 | Conditional turn restrictions (`restriction:conditional`, ignored today) | P2 | format 3, FEAT-W0 |

**`FEAT-A` Analysis (SAR field tools)**

| Id | Item | Pri | Depends on |
|---|---|---|---|
| FEAT-A1 | Isochrone polygons as GeoJSON (marching squares on a scalar grid); after A2 so multi-origin polygons ship once | P1 | — |
| FEAT-A2 | Multi-origin isochrones (team coverage): core S, offsets and attribution M | P1 | — |
| FEAT-A3 | Reverse isochrones (arrive-at) | P1 | — |
| FEAT-A4 | Nearest-k targets by travel time | P1 | — |
| FEAT-A5 | Trip optimization, exact for small N (Held-Karp to 13 stops) | P1 | — |
| FEAT-A6 | Trip optimization, heuristic for larger N | P2 | FEAT-A5, PERF-15 |
| FEAT-A7 | Map matching attributes per sample | P2 | — |
| FEAT-A8 | Polygon reachability (time to reach an area, coverage of an area) | P1 | FEAT-A2, FEAT-A4 |
| FEAT-A9 | Distance contours | P1 | — |
| FEAT-A10 | Fastest-time (duration-ordered) isochrones as an option | P1 | FEAT-A2 |
| FEAT-A11 | Overlays on matrix and isochrone requests | P1 | — |

**`FEAT-G` Guidance**

| Id | Item | Pri | Depends on |
|---|---|---|---|
| FEAT-G0 | Structured instruction parameters (`modifier`, `exit_number`, `toward`, `via`) | P1 | — |
| FEAT-G1 | Localization layer: catalogs keyed by `instruction_key`; English moved into a catalog; host catalogs | P1 | FEAT-G0 |
| FEAT-G2 | First languages (proposal: Polish, German, French, Spanish, Ukrainian) | P1 | FEAT-G1 |
| FEAT-G3 | Lanes from `turn:lanes`, read from the approach edge (a bug fix today) | P0 | — |
| FEAT-G4a | Exit numbers from `motorway_junction` nodes (no format change) | P1 | — |
| FEAT-G4b | Directional, per-lane and symbol destinations | P1 | format 3 |
| FEAT-G5a | Voice timing | P2 | — |
| FEAT-G5b | Units and rounding per locale | P2 | FEAT-G1 |

**`FEAT-D` Data and scale**

| Id | Item | Pri | Depends on |
|---|---|---|---|
| FEAT-D1 | Country-scale measurement (Poland), numbers published (measured 2026-11-19, see REG-1) | P1 | — |
| FEAT-D2 | Clip polygon compile from a larger extract (M); canonical polygon hash shared with REG-3 | P1 | API-9 |
| FEAT-D3 | Multi-extract compile (cross-border regions in one package) (M) | P1 | — |

FEAT-D1 to D3 are option A of 6.7 and feed lane `REG`.

### 7.8 Lane: Regions and scale (`REG`)

Design in 6.7. Hooks before 1.0; bundles, cells and the server in 1.x.

| Id | Item | Pri | Depends on |
|---|---|---|---|
| REG-1 | Country scale measured: Poland compiled on a cloud runner (time, peak memory, package size, open and customize time, potentials memory); was PERF-12. **Measured on 0.2.0**: 7 min, 10.8 GiB peak, 1.97 GB, 1.6 GiB potentials heap; repeat on the release candidate | P1 | FEAT-D1 |
| REG-2 | Compile headroom at country scale: faster node order (40 % of Poland's compile, about n^1.5), slimmer build graph and streamed shared infos | P1 | REG-1 |
| REG-3 | Format 3 hooks: tagged section table (id, REQUIRED and IDENTITY flags, element size), identity hash separate from the content hash, a required `RegionInfo`, reserved ids for `PortalEdge`, `PortalRef`, `NodeOwner`, `PortalClique`, `PortalBound`, `ComponentPortal`, `CellDirectory`, `CchUp`, `CchMetric`; `CchCellTree` written; in the one format-3 bump (7.12) | P0 | API-5, EXP-5b |
| REG-4 | Region-ready references: `(region, edge)` in results, overlay sets bound per region, connectors and endpoint candidates region-qualified, a defined form for synthetic ways and repaired crossings, `region_missing` error code, C ABI 2 open taking a source | P0 | API-5, FFI-4 |
| REG-5a | Region bundles: halo compile, banded min-cut borders, portal tables matched by reference, potential-guided search with CCH portal bounds, exact against the oracle on a merged compile | P2 (1.x) | REG-3, REG-4 |
| REG-5b | Exact cliques and the overlay-exact search as the fast path for plain requests | P2 (1.x) | REG-5a |
| REG-6 | Rebuild one region and refresh the bundle index | P2 (1.x) | REG-5a |
| REG-7 | Multi-level cells from a continent build, one overlay per level | P2 (1.x) | REG-5a |
| REG-8 | `wayfarer-server`: JSON API over bundles with a cell cache, and an immutable cell endpoint | P2 (1.x) | REG-5a, EXP-4 |
| REG-9 | Device cell cache: fetch by area or corridor, check by hash, byte budget, route over the cells held; the cells are REG-13's byte ranges as hash-named blobs | P2 (1.x) | REG-7, REG-8, REG-13 |
| REG-10 | Stored potentials: CCH topology and stored-column metrics as optional mapped sections, derived arrays built per cell (open without the 8 s preparation on Poland) | P1 (1.x) | REG-3 |
| REG-11 | One global lattice for the DEM grid and rasters (FEAT-T4, X3a, X4) | P1 | before format 3 |
| REG-12 | Search state sized by what a search touches: paged edge labels, node bounds and potential slots; floor-scaled stored metrics instead of own metrics on the mobile profile. No format change; removes 0.9 GiB of Poland's 2.5 GiB first-route heap | P1 | — |
| REG-13 | Paged package: cell-major numbering in the format-3 compiler (hook), then `CellDirectory` with per-cell hashes, lazy per-cell validation and `memoryBudgetBytes` (1.x). Target: Poland on a phone with tens of MB resident | P1 | REG-3, REG-10 |

### 7.9 Lane: Quality and safety (`QA`)

| Id | Item | Pri | Depends on |
|---|---|---|---|
| QA-1 | `cargo-semver-checks`, schema diff, header diff on every PR | P0 | — |
| QA-2 | Nightly Małopolskie gate | P0 | — |
| QA-3 | Scheduled, time-boxed fuzzing with a kept corpus | P0 | — |
| QA-4 | Two more regions (dense city, mountainous rural) in the Valhalla comparison and gates | P0 | — |
| QA-5 | Fuzz targets for every new input (edge refs, profiles, catalogs, DEM tiles) as they land | P0 | per item |
| QA-6 | `tracing` spans with stable names | P1 | — |
| QA-7 | Security policy for 1.x; fuzz coverage review | P0 | — |
| QA-8 | Field trial with a real team before the freeze | P0 | — |
| QA-9 | Request limits on every query: matrix and isochrone validated like routes, a matrix size cap, contour checks, an upper bound on snap and projection radii | P0 | — |

### 7.10 Lane: Docs and developer experience (`DOC`)

| Id | Item | Pri | Depends on |
|---|---|---|---|
| DOC-1 | Fix documentation drift (5.8) | P0 | — |
| DOC-2 | Issue tracker seeded with every item in this document, labelled by lane and priority | P0 | — |
| DOC-3 | `#![deny(missing_docs)]` on public crates | P0 | API-2 |
| DOC-4 | Guide: concepts, compile, route, overlays, SAR, embedding | P0 | — |
| DOC-5 | Changelog entries name item ids and lanes | P1 | DOC-2 |

### 7.11 Lane: Experimental (`EXP`)

Ideas worth trying that may never ship. They live behind
`experimental-*` cargo features (or separate binaries), carry no stability
promise, and are excluded from the 1.0 contract. An item **graduates** into
another lane when it has a written design, tests, a gate where it matters,
and a user who needs it.

| Id | Item | Graduates to |
|---|---|---|
| EXP-1 | Speed-up prototypes from PERF-2 that lose the comparison, kept for reference or removed | — |
| EXP-2 | Cross-country movement: terrain cost surface joined to the graph, for SAR on foot | FEAT-X (X4, X5) |
| EXP-3 | WASM build for browser offline routing | FFI |
| EXP-4 | Optional HTTP server binary (own API, maybe a Valhalla/OSRM-compatible subset) | REG-8 |
| EXP-5 | Multiple packages per engine, routing across adjacent packages | REG-5a |
| EXP-5b | Two steps before format 3 is frozen: (1) the potential-guided search across a virtual seam inside one package (cut at the top separator), checked against the single-package search and the oracle; (2) two halves compiled separately with a halo and a banded cut, portals matched by reference | REG-3, REG-5a |
| EXP-6 | Incremental updates from OSM change files or per-partition rebuilds | FEAT-D |
| EXP-7 | Field-collected roads (GeoJSON) as compiler input, or overlays promoted into the graph | FEAT-D |
| EXP-8 | Learned or data-driven speed inference for roads without a posted limit | FEAT-W |

### 7.12 Cross-lane dependencies

The ones that decide ordering:

- **PERF-2 → API-6.** Don't freeze the overlay model before knowing how
  overlays are searched on the chosen speed-up.
- **API-6 → PERF-6.** Overlays on the hierarchy implement the new model, not
  the old one.
- **API-3, API-4 → FFI-4.** C ABI 2 is designed once, on the new errors and
  wire rules.
- **FFI-4 → FFI-7..9.** Bindings wrap ABI 2, never ABI 1.
- **API-7 → FEAT-P, FEAT-W (wire shape only).** The profile bug fixes,
  local time (W0) and the truck data need nothing from API-7; only their
  final request shape does. Build the machinery early, expose it through
  API-7.
- **QA-1 before the redesign window.** Every breaking release is measured.
- **API-5 → REG-3, REG-4.** Stable references can name a region before the
  format and the wire are frozen.
- **EXP-5b → REG-3.** A seam is prototyped before format 3's region sections
  are fixed.
- **One format-3 bump, layout changes only.** Format 2 cannot carry an
  optional section (exact section count, strict `LegAttrs`), so the bump
  carries REG-3's section table and `RegionInfo`, the profile slot table
  and u16 access and restriction masks (FEAT-P1's prerequisite), a versioned
  `LegAttrs`, and REG-13's cell-major numbering. Content sections (FEAT-T1
  and T4, FEAT-X2, X3a, FEAT-G4b, FEAT-W4, truck limits, REG-10 and REG-13's
  directory) follow as optional sections, one release at a time. Format 2
  shipped in 0.2, so the bump lands no earlier than 0.12.
- **FEAT-T4, FEAT-X3a → FEAT-X3.** Terrain-aware access legs need the DEM
  grid and land cover.
- **QW-3, QW-2 → FEAT-X1.** Access costs as potential offsets, and snapping
  that doesn't grow with the radius squared.

### 7.13 What the first releases could look like

A sketch to show how releases pull from lanes, not a commitment:

| Release | Items |
|---|---|
| 0.2 | Released 2026-11-19: PERF-2 to PERF-7 and PERF-10 (CCH potentials, format 2, via-edge alternatives, faster matrices), part of FFI-1 (Windows, i686), fixes #8, #13, #22 |
| 0.3 | Correctness: QA-9 (request limits), FEAT-P0b, FEAT-P0c, the conditional parser fixes, FEAT-G3 and multi-leg maneuver distances, isochrone origin snapping and duration fixes; PERF-13 (leg building); rest of DOC-1; FFI-6 with a first `include` list if ready |
| 0.4 | Small additive: FEAT-P0a, P0d, W0, P4, G0, G4a, A4, A9, A7, A11, the route request quick wins (radius per location, through waypoints, destination side, hard exclusions, road facts), two-phase snapping, approach offsets (QW-3), compiler input fixes; REG-12; PERF-14 to PERF-16; QA-1, QA-2, QA-3, PERF-1, rest of FFI-1 (macOS) |
| 0.5 | FEAT-X1 (costed access legs, opt-in) with FEAT-X6's `costed` flag, FEAT-A2 core, FEAT-D2 (clip), FEAT-P2 compiler data, FEAT-G1 with Polish, EXP-5b step 1 (potential-guided seam search), REG-2 |
| 0.6 – 0.9 | FEAT-X2a, X1b, A10, A1, A3, A8, A5, D3, G5a; FEAT-T1 reader behind `experimental-elevation`; API-0 (with the open questions); QA-4; the format-3 design written (REG-3, REG-11, REG-13 hooks) |
| 0.10 – 0.11 | API-1 (modules and names), API-2 (internal crates), FFI-3 (release artifacts), EXP-5b step 2 (real seam) |
| 0.12 – 0.2x | API-3, API-4, API-5; format 3 (layout changes only); FEAT-X6 default flip (≈0.13); API-7a time object (≈0.17), then FEAT-W1/W2 and API-7b presets; REG-4 with FFI-4 |
| After the bump | Optional sections as each is ready: FEAT-T1/T2/T4/T5, X2, X3a then X3, P1, P2 extras, G4b, W4 |
| … | further items as they are ready |

After the API window closes, releases carry PERF, FFI, FEAT and QA items until
the 0.80 cut-off, then hardening (API-11, DOC-3, DOC-4, QA-7, QA-8) and the
0.90 freeze.

### 7.14 Work packages and parallel threads

The next releases are cut into PR-sized work packages grouped by the files
they touch, so several threads can work at once without editing the same
functions. One thread per lane, one PR in flight per lane.

| Thread lane | Owns | First packages | Release |
|---|---|---|---|
| Request safety | `limits.rs`, `queries.rs`, `correlator.rs`, request validation in `trip.rs`, `matrix.rs` and `isochrone.rs` | QA-9 request limits; isochrone origin fixes; two-phase snapping | 0.3, 0.4 |
| Profiles, access, time | road-graph tags, types and restriction resolution, `wayfarer-costing`, restriction masks in the compiler and `graph_access.rs` | FEAT-P0c, then P0b and the parser fixes; P0a/P0d; W0 | 0.3, 0.4 |
| Legs, guidance, output | `guidance.rs`, `route_leg.rs`, `navigation.rs`, result assembly in `trip.rs`, polyline code | FEAT-G3 and multi-leg maneuvers; PERF-13; FFI-6 with `include`; G0 and G4a | 0.3, 0.4 |
| Search core and memory | `package/search.rs`, `search/patch.rs`, `package/cch.rs`, `package/mod.rs` open path, `reference.rs` | REG-12; PERF-14 to PERF-16; approach offsets; FEAT-X1 | 0.4, 0.5 |
| Analysis tools | `isochrone.rs`, `types/analysis.rs`, new trip and polygon modules | FEAT-A4, A9, A11, A7; A2; A10, A1, A3, A8, A5 | 0.4 – 0.8 |
| Compiler options and data | compiler way collection, `CompileRequest`, CLI compile | input fixes; `#[non_exhaustive]` and baked profiles; D2; D3 | 0.4 – 0.7 |
| Regions and scale | new `regions` module behind `experimental-regions`, `cch_order.rs`, design docs | EXP-5b step 1; REG-2; the format-3 design; EXP-5b step 2 | 0.5 – 0.11 |
| CI, gates, docs | workflows, gate scripts, READMEs | macOS and MSRV jobs; API diffs; nightly Małopolskie gate and scheduled fuzzing; DOC-1 | 0.3, 0.4 |

Validation follows the change: every PR runs the test suite and the Monaco
and Kraków byte-identical checks (re-recorded on purpose, case by case, when
a fix changes routes); changes meant to affect performance or package output
also run the Małopolskie gate and timings, on cloud machines or CI. Generated
schemas are regenerated after a rebase, never merged by hand, and changelog
entries are collected in the release PR.

## 8. Non-goals for 1.0

- Public transit and multimodal journeys.
- Planet-scale packages on a phone. (After 1.0 a phone may hold an overlay
  and the cells it needs, REG-9.)
- A hosted service. Wayfarer stays in-process. The server (REG-8, after
  1.0) is a thin optional binary anyone can run, not a service we operate.
- Lua or scripting for profiles: parameters and presets, not code.

## 9. Open questions

1. **Audience for 1.0.** Is search and rescue still the primary user, or is
   1.0 a general-purpose offline router? This decides whether elevation and
   cross-country movement outrank bicycle and truck profiles.
2. **Scale target.** Is a country (Poland) required for 1.0, or are regions
   enough with country scale after? (Measured: Poland compiles as one
   package in 16 GB on 0.2.0; on a phone it needs REG-12, REG-10 and
   REG-13.)
3. ~~Speed-up technique.~~ Answered in 0.2: CCH potentials, format 2.
4. **Which Rust crates are public at 1.0**, and does `wayfarer-service` stay
   separate?
5. **Two release lines.** Keep `wayfarer-v*` and `wayfarer-compiler-v*`, or
   one version for the whole workspace? One line is simpler for users; two
   allow compiler fixes without engine releases.
6. **Package compatibility across 1.x.** Proposal: every 1.x engine reads
   every 1.x package. Is a narrower window (N-1 formats) acceptable?
7. **HTTP server.** Wanted at all? If so, our own API only, or a
   Valhalla/OSRM-compatible subset for drop-in use?
8. **Bindings.** Which languages and platforms do current users actually
   embed from?
9. **`wayfarer-line-network` render fields** (`min_zoom`, `max_zoom`,
   `sort_order`) serve a vector basemap compiler. Does that compiler stay a
   user of this crate, or does the crate become routing-only?
10. **Team and timeline.** The lane order holds either way; dates depend
    on who works on it.
11. **Off-network defaults.** Should costed access legs (FEAT-X1) be on by
    default for walkers, and with what radius? May R7's totals change
    (FEAT-X6)?
12. **Region boundaries.** Admin areas (cross-border and county edits stay
    local), a fixed grid (simplest for downloads), or the nested
    dissection's own cells (fewest portals)? Bundles (REG-5a) can take any;
    streamed cells (REG-7) want the last.
13. **Server and streaming.** Is a continent-scale server with device cell
    caches a 1.x goal with a named user, or are regions on devices enough?
    It decides whether REG-7 to REG-9 are planned or stay in `EXP`.
14. **Poland on a phone, offline.** A 1.0 or a 1.x goal? Proposal: REG-12
    in 0.4 and REG-13's hooks in format 3, the paged package itself in 1.x.
15. **SAR on foot and `foot=no`.** Once bicycle-only ways close to walkers
    (FEAT-P0a), should Legged follow `foot=no` strictly? Proposal: strict,
    with an `open` overlay when a team needs it.
16. **Time.** One time zone per package with a request offset for
    multi-zone extracts? Without a departure, reject windowed overlays or
    treat them as always active? Proposal: keep the fallback but report it
    until API-7a, then require a time.
17. **Costed access legs' pricing.** Cost factor (proposal 2.0 walkers, 1.5
    vehicles), default radius and candidate count for FEAT-X1.
18. **Format 3 shape.** Optional sections addable after the bump with a
    per-section REQUIRED flag (proposal), or everything in the one bump? u16
    profile access with a slot table (proposal) or u8?
19. **Elevation data.** SRTM/NASADEM (public domain) or Copernicus GLO-30
    (attribution, a surface model) as the default DEM? Wait for format 3
    (proposal) or bend format 2's `Reserved12`?
20. **Isochrone semantics.** Keep cost-ordered time contours or move to
    fastest-time (FEAT-A10)? Proposal: an option now, the default in API-7.
21. **Guidance rendering.** Hybrid (keys are the contract, bundled
    languages rendered on request; proposal) or keys only?

# Wayfarer roadmap to 1.0

Status: proposal, written against 0.1.3 (2026-11-17). It is meant to be argued
with: every milestone lists what it delivers, why, and what has to be decided
first. Open questions are collected at the end.

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
milestones.

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
| Speed-up technique | CH / MLD | tiled hierarchy | CH / LM | ALT only | hierarchy or CCH (P0) |
| Per-request dynamic costing | limited | ✓ | ✓ custom models | ✓ overlays + options | ✓, one unified model |
| Elevation | – | ✓ | ✓ | – | ✓ (P1) |
| Time-dependent / departure / arrive-by | – | ✓ | partial | one evaluation time | departure + arrive-by (P1) |
| Traffic / historical speeds | ✓ (CSV) | ✓ | ✓ | – | speed overlay input (P2) |
| Bicycle / truck / motorcycle | ✓ (Lua) | ✓ | ✓ | – | bicycle + truck (P1) |
| Custom profiles | Lua at build | JSON costing | custom model | closed enum + options | parametric profiles (P1) |
| Localized guidance, voice, lanes, signs | ✓ | ✓ rich | ✓ | English text, lanes stored | localization + lanes + signs (P1) |
| Encoded polyline output | ✓ | ✓ | ✓ | – | ✓ (P0, cheap) |
| Continent scale | ✓ | ✓ | ✓ | regions | country-scale measured (P1) |
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
- **Terrain-aware walking.** `OnFoot`/`Legged` would gain the most from
  elevation (Tobler-style hiking speed).

## 4. The 1.0 stability contract

| Surface | 0.1 today | At 1.0 | Rule within 1.x |
|---|---|---|---|
| Rust API | 9 crates, all 0.x, wide re-exports | A named set of public crates (proposal below); the rest internal | semver; `cargo-semver-checks` in CI |
| JSON wire format | Follows the engine crate; no version field | `apiVersion` in requests/results; schemas published per version | Additive only: new optional fields, new enum values (clients must tolerate unknowns) |
| C ABI | ABI 1 | ABI 2, designed once at 0.3 | Additive only: new functions, never changed signatures |
| Package format | Format 1, reads N-1 for one release | Format frozen at 1.0 | Every 1.x engine reads every package a 1.x compiler wrote |
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

- **Speed-up technique.** Close the p95 gap (153 ms vs 16 ms). Candidates:
  - *Customizable contraction hierarchies (CCH)*: metric-independent order at
    compile time, fast customization per profile, and per-request changes can
    be handled by partial re-customization or by searching the overlay patch
    over the hierarchy. Best fit for dynamic costing; most work.
  - *Core-ALT / hierarchy on top of ALT*: contract degree-2 and low-importance
    nodes, run ALT on the core. Less work, smaller win.
  - *Multi-level partitions (MLD-style)*: also fits dynamic metrics and
    makes country scale possible.
  Run a spike on Małopolskie with each before choosing (milestone 0.4). The
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
- **Cross-country movement (P2).** Terrain cost surface joined to the graph,
  for SAR on foot. Big; plan after elevation.
- **Traffic input (P2).** Speed overlays already exist; a bulk speed-file
  input would cover live or historical traffic without a recompile.

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
- **Multiple packages per engine (P2).** Route across adjacent packages.
- **Incremental updates (P2).** OSM change files applied to a package, or
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

## 7. Milestones

Durations are rough and assume the current pace; they are meant to set order,
not dates.

### 0.2 — Ground truth (short)

Low-risk work that makes later breaking changes safe.

- Fix the documentation drift (5.8).
- CI on macOS and Windows; MSRV job; nightly Małopolskie gate and fuzzing.
- `cargo-semver-checks`, schema diff and header diff jobs (informational
  until 1.0).
- Decide the public crate set and the release-line question (open questions 1
  and 5).
- Seed the issue tracker with this roadmap's items, one issue each, labelled by
  milestone and priority.
- Encoded polyline output (additive, cheap).

### 0.3 — API redesign (breaking)

The one big breaking release.

- Module layout, prefix removal, `#[non_exhaustive]`, builders (5.1).
- Unified overlay model (5.2) and stable edge references (5.4).
- Profile base + parameters, one time object (5.2).
- Lean results with `include`, consistent fallibility (5.3).
- One error vocabulary (5.5).
- JSON wire `apiVersion` 1; serde aliases removed; one unknown-field rule.
- C ABI 2 (5.6); compile request unified (5.7).
- Internal crates hidden; `wayfarer-service` folded or trimmed.
- Migration guide from 0.1.

Dependency: the overlay model must be designed with the 0.4 speed-up in mind
(how an overlay patch is searched over a hierarchy). Do the 0.4 spike before
freezing the overlay semantics.

### 0.4 — Speed

- Speed-up spike (CCH vs core-ALT vs partitions), then implementation.
- p95 target met on Małopolskie (proposal: under 30 ms single thread),
  package not larger than today.
- Many-to-many on the new structure.
- Package format 2 (keeps reading format 1 per the policy).

### 0.5 — Terrain and time

- Elevation ingest, costing and profiles.
- Departure and arrive-by, time-evaluated restrictions and overlay windows.
- Bicycle and truck presets.

### 0.6 — Field features

- Trip optimization, nearest-k, isochrone polygons, multi-origin and reverse
  isochrones.
- Localized guidance, lanes and signs.
- Clip polygon and multi-extract compile.

### 0.7 — Scale and platforms

- Country-scale measurements and fixes.
- iOS/Android artifacts; Swift, Kotlin and Python bindings.
- Quality comparison on three regions.

### 0.9 — Release candidate

- API freeze: only fixes and additive changes from here.
- Remove every deprecation.
- Docs: a guide (concepts, compile, route, overlays, SAR, embedding), API
  reference complete, examples per binding.
- Package format and costing model frozen for 1.x.
- Soak: fuzzing, gates and a field trial with a real team.

### 1.0

- Stability contract (section 4) published in the README and enforced in CI.

### After 1.0 (candidates)

Cross-country movement, traffic input, multiple packages per engine,
incremental updates, field data input, WASM, an optional HTTP server.

## 8. Non-goals for 1.0

- Public transit and multimodal journeys.
- Planet-scale packages on a phone.
- A hosted service. Wayfarer stays in-process; a server, if any, is a thin
  optional binary.
- Lua or scripting for profiles: parameters and presets, not code.

## 9. Open questions

1. **Audience for 1.0.** Is search and rescue still the primary user, or is
   1.0 a general-purpose offline router? This decides whether elevation and
   cross-country movement outrank bicycle and truck profiles.
2. **Scale target.** Is a country (Poland) required for 1.0, or are regions
   enough with country scale after?
3. **Speed-up technique.** Are we willing to change the package format and
   grow compile time to close the p95 gap? What p95 is "good enough"?
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
10. **Team and timeline.** The milestone order holds either way; dates depend
    on who works on it.

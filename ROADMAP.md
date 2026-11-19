# Wayfarer roadmap to 1.0

Status: proposal, written against 0.1.3 (2026-11-17). It is meant to be argued
with: every lane lists what it delivers, why, and what has to be decided
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
| C ABI | ABI 1 | ABI 2, designed once (FFI-4) | Additive only: new functions, never changed signatures |
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
  (7.10).

### 7.2 Gates

| Gate | From | What it means |
|---|---|---|
| Tooling on | by 0.5 | Semver, schema and C header diffs run on every PR (informational) |
| API redesign window | 0.10 – 0.39 | The API lane's breaking items land here, one area per release |
| Breaking cut-off | 0.80 | After this, only the hardening review may break anything |
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
| PERF-12 | Country-scale compile memory and time (external sorting where needed), open time on mobile | P1 | FEAT-D1 |

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
| FEAT-T1 | DEM ingest at compile time (SRTM / Copernicus GLO-30), deterministic, per-edge climb and descent | P1 | — |
| FEAT-T2 | Elevation costing on foot (Tobler) for `OnFoot` and `Legged`; elevation profile in results | P1 | FEAT-T1 |
| FEAT-T3 | Vehicle grade costing | P2 | FEAT-T1 |

**`FEAT-P` Profiles**

| Id | Item | Pri | Depends on |
|---|---|---|---|
| FEAT-P1 | Bicycle preset (access already read) | P1 | API-7 |
| FEAT-P2 | Truck preset: dimensions and weights (`maxheight`, `maxweight` already parsed) | P1 | API-7 |
| FEAT-P3 | Motorcycle preset | P2 | API-7 |

**`FEAT-W` Time**

| Id | Item | Pri | Depends on |
|---|---|---|---|
| FEAT-W1 | Depart-at with restrictions and overlay windows evaluated along the route | P1 | API-7 |
| FEAT-W2 | Arrive-by | P1 | FEAT-W1 |
| FEAT-W3 | Bulk speed-file input (traffic, historical speeds) as speed overlays | P2 | API-6 |

**`FEAT-A` Analysis (SAR field tools)**

| Id | Item | Pri | Depends on |
|---|---|---|---|
| FEAT-A1 | Isochrone polygons as GeoJSON | P1 | FFI-6 |
| FEAT-A2 | Multi-origin isochrones (team coverage) | P1 | — |
| FEAT-A3 | Reverse isochrones (arrive-at) | P1 | — |
| FEAT-A4 | Nearest-k targets by travel time | P1 | — |
| FEAT-A5 | Trip optimization, exact for small N | P1 | — |
| FEAT-A6 | Trip optimization, heuristic for larger N | P2 | FEAT-A5, PERF-8 |
| FEAT-A7 | Map matching attributes per sample | P2 | — |

**`FEAT-G` Guidance**

| Id | Item | Pri | Depends on |
|---|---|---|---|
| FEAT-G1 | Localization layer: catalogs keyed by `instruction_key`; English moved into a catalog; host catalogs | P1 | — |
| FEAT-G2 | First languages (proposal: Polish, German, French, Spanish, Ukrainian) | P1 | FEAT-G1 |
| FEAT-G3 | Lanes from `turn:lanes` | P1 | — |
| FEAT-G4 | Signs and exits from `destination*` tags | P1 | — |
| FEAT-G5 | Voice timing; units and rounding per locale | P2 | FEAT-G1 |

**`FEAT-D` Data and scale**

| Id | Item | Pri | Depends on |
|---|---|---|---|
| FEAT-D1 | Country-scale measurement (Poland), numbers published | P1 | — |
| FEAT-D2 | Clip polygon compile from a larger extract | P1 | API-9 |
| FEAT-D3 | Multi-extract compile (cross-border regions in one package) | P1 | — |

### 7.8 Lane: Quality and safety (`QA`)

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

### 7.9 Lane: Docs and developer experience (`DOC`)

| Id | Item | Pri | Depends on |
|---|---|---|---|
| DOC-1 | Fix documentation drift (5.8) | P0 | — |
| DOC-2 | Issue tracker seeded with every item in this document, labelled by lane and priority | P0 | — |
| DOC-3 | `#![deny(missing_docs)]` on public crates | P0 | API-2 |
| DOC-4 | Guide: concepts, compile, route, overlays, SAR, embedding | P0 | — |
| DOC-5 | Changelog entries name item ids and lanes | P1 | DOC-2 |

### 7.10 Lane: Experimental (`EXP`)

Ideas worth trying that may never ship. They live behind
`experimental-*` cargo features (or separate binaries), carry no stability
promise, and are excluded from the 1.0 contract. An item **graduates** into
another lane when it has a written design, tests, a gate where it matters,
and a user who needs it.

| Id | Item | Graduates to |
|---|---|---|
| EXP-1 | Speed-up prototypes from PERF-2 that lose the comparison, kept for reference or removed | — |
| EXP-2 | Cross-country movement: terrain cost surface joined to the graph, for SAR on foot | FEAT-T |
| EXP-3 | WASM build for browser offline routing | FFI |
| EXP-4 | Optional HTTP server binary (own API, maybe a Valhalla/OSRM-compatible subset) | FFI |
| EXP-5 | Multiple packages per engine, routing across adjacent packages | FEAT-D |
| EXP-6 | Incremental updates from OSM change files or per-partition rebuilds | FEAT-D |
| EXP-7 | Field-collected roads (GeoJSON) as compiler input, or overlays promoted into the graph | FEAT-D |
| EXP-8 | Learned or data-driven speed inference for roads without a posted limit | FEAT-W |

### 7.11 Cross-lane dependencies

The ones that decide ordering:

- **PERF-2 → API-6.** Don't freeze the overlay model before knowing how
  overlays are searched on the chosen speed-up.
- **API-6 → PERF-6.** Overlays on the hierarchy implement the new model, not
  the old one.
- **API-3, API-4 → FFI-4.** C ABI 2 is designed once, on the new errors and
  wire rules.
- **FFI-4 → FFI-7..9.** Bindings wrap ABI 2, never ABI 1.
- **API-7 → FEAT-P, FEAT-W.** New profiles and time are built on the new
  profile and time types.
- **QA-1 before the redesign window.** Every breaking release is measured.

### 7.12 What the first releases could look like

A sketch to show how releases pull from lanes, not a commitment:

| Release | Items |
|---|---|
| 0.2 | DOC-1, DOC-2, FFI-1 |
| 0.3 | FFI-6 (polyline, GeoJSON), QA-2, FFI-2 |
| 0.4 | QA-1, QA-3, PERF-1 |
| 0.5 | QA-4, API-0, FEAT-G3 (lanes) |
| 0.6 | PERF-2 decision, FEAT-A2 (multi-origin isochrones) |
| 0.10 | API-1 (modules and names), FFI-3 (release artifacts) |
| 0.11 | API-2 (internal crates), FEAT-A4 (nearest-k) |
| 0.12 | API-3 (errors), FEAT-T1 (DEM ingest, behind a compile option) |
| 0.13 | API-4 (wire rules), PERF-3 (format 2) |
| … | further items as they are ready |

After the API window closes, releases carry PERF, FFI, FEAT and QA items until
the 0.80 cut-off, then hardening (API-11, DOC-3, DOC-4, QA-7, QA-8) and the
0.90 freeze.

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
10. **Team and timeline.** The lane order holds either way; dates depend
    on who works on it.

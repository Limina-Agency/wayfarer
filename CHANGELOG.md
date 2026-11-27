# Changelog

Two release lines share this repository: `wayfarer-v*` (the package, costing,
engine, service and C ABI crates) and `wayfarer-compiler-v*` (the compiler, the
`wayfarer` binary and their crates). Each entry names the package format, the
costing model and the C ABI version a release reads or writes.

## 0.6.0 (2026-11-27)

Both lines. **Package format 3** written (reads 2 and 3), costing model 1, C
ABI version 1 (additive). There was no 0.5.0: everything on `main` since
0.4.0 ships here, with search and rescue planning and the first off-road
terrain crates.

### Migrating from 0.4

- **Recompiled packages are format 3, which 0.4 engines don't open** (#110).
  0.6 opens 0.4's format-2 packages and routes on them as before, so there is
  no need to recompile at once. To keep a package that older engines read,
  compile format 2 on request: `CompileRequest::package_format`
  (`PackageFormat::V2`) or `wayfarer compile --package-format 2`; it is the
  same bytes as before. Format-3 drafts that development builds wrote before
  0.6 are refused as outdated: recompile them.
- **A recompile changes ids and hashes.** A format-3 package numbers nodes by
  CCH rank (#101), so edge and node ids differ from format 2, and its dataset
  hash is an identity hash over the header and the identity sections (#82):
  routes and costs are the same under the new ids. Rebind overlay sets bound
  by edge id or dataset hash, or they fall back to their geometry. Format 3
  also stores every node's OSM id (#96). Info, verify and compile results
  gain `contentHash` beside `datasetHash` (C ABI, CLI and host protocol).
- **Walking routes change after a recompile** (#97, P0a and P0d): `on_foot`
  takes foot access only, so cycleways and `foot=no` ways close to walkers
  (`legged` keeps them), and bus or HGV access alone no longer opens a way
  to cars. Emergency vehicles keep busways through a new mode bit
  (`MobilityMode::EMERGENCY`). 24 of Małopolskie's walking gate cases cost
  more; no other profile changes.
- **Exit numbers and signs** (#87): the compiler keeps `motorway_junction`
  nodes, so a recompile moves the dataset hash, and maneuvers gain direction
  modifiers, exit numbers, toward and via lists and the facts of the road
  they enter. Boarding a ferry is its own maneuver (`route.ferry`). "Toward"
  text comes from destination tags only, and an unnamed road is entered onto
  its ref, so instruction text changes on some maneuvers.
- **Last bits of costs** (#79, #90): a found path's cost, duration and
  distance are summed in path order, and vehicle matrices stay pairwise at
  any size, so costs can differ from 0.4 in the last bits. Paths and
  statuses are the same; matrix rows now equal pairwise cells bit for bit.
- **Overlay isochrones** from a farther origin candidate start their time at
  zero, as plain isochrones do (#80): time contours reach a little farther.
- **Every new request field is optional and off by default:** `offNetwork`
  on any request (#108), isochrone `origins`, `order: "time"`,
  `startOffsetS`, `includeOrigins` and `polygons` (#85, #91, #107). Routes
  and matrices report the radius snapping widened to, only when it widened
  (#83).
- **Opening:** `prefetch()` builds the first drive's work first and faults
  the file in last (#105). The C ABI's open takes `"warmUp": true` to warm
  up on a thread of its own (`RoutingPackage::warm_up_in_background`), and
  the service uses it. On the Mobile profile the CCH customization arrays
  are dropped after each customization and built again when needed (#86).
- **New crates:** `wayfarer-sar` (search and rescue planning, #111) and the
  experimental `wayfarer-layer` and `wayfarer-terrain` (#112 to #119) are
  published. `wayfarer-terrain-compiler` (#116) is not published yet: build
  it from the tag. No terrain file ships; an app compiles its own.
- **Rust API (breaking):** see Changes; struct literals of the request and
  result types named there need the new fields, and `::new` and `Default`
  are unchanged.

### Changes

- **Added: package format 3** (#82, #88, #92, #96, #101, #109, #110; REG-3,
  FMT 1 to FMT 7, `docs/design/format-3.md`). A tagged section table with
  per-section blake3 hashes, where readers skip unknown optional sections
  and refuse unknown required ones; profile slots with u16 masks (up to 14
  profiles); versioned attribute-class records that newer readers and older
  records both read; a region info section and OSM node ids; nodes numbered
  by CCH rank, with each dissection cell one range of ranks and the cell
  tree stored; u64 byte offsets and u32 cost tuple rows; the request grid on
  64 × 64 blocks of the global 12″ lattice. `wayfarer info` lists unknown
  sections. Fixtures `fixtures/routing/format-2/` and `format-3/` are both
  read in CI; new fuzz targets `package_table` and `leg_attrs`.
- **Added: costed off-network access legs** (#108; FEAT-X1, X6,
  `docs/design/off-road.md`). With `offNetwork` in a request's options, a
  point off the roads snaps to up to `maxCandidates` directed edges within
  `maxRadiusM` and each candidate pays `costFactor × d / paceMps` to reach
  (defaults: 500 m and 2 for walkers, 250 m and 1.5 for vehicles). Totals
  include both stretches; route geometry starts and ends at the requested
  points, with an `off_network_access` shape segment, an `access_leg`
  overlay, a maneuver and an `offNetwork` summary per stretch. Routes, trips,
  matrices and isochrones all take it. SAR's `off_network.costed` makes the
  off-network leg an access leg in the totals.
- **Added: isochrones from several origins** (#85; FEAT-A2 core), at most
  64. Each origin snaps as `origin` does; an origin trapped behind
  destination-only roads gets its own relaxed search.
- **Added: fastest-time isochrones** (#91; FEAT-A10). `order: "time"`
  orders labels by travel time, so a time contour holds exactly what is
  reachable within it. Extra origins take `startOffsetS`, and
  `includeOrigins` reports which origin reaches each edge first.
- **Added: isochrone polygons** (#107; FEAT-A1). `polygons` puts a GeoJSON
  MultiPolygon on each bucket: the area within a short walk of the roads it
  reaches, drawn on one grid per request (`resolutionM`, `spreadM`,
  `denoise`). New `GeoJsonMultiPolygon`, `RouteIsochronePolygonOptions`
  and `MAX_ISOCHRONE_POLYGON_GRID_NODES`.
- **Added: structured guidance** (#87): see Migrating. Navigation input
  carries each shape segment's speed limit and progress reports the active
  edge's. `docs/guidance.md` is the renderer key table.
- **Added: walking pace model** (#99). `wayfarer_costing::pace`, an
  integer model (slope curve, trail factors from `highway` and
  `sac_scale`, P10/P50/P90 spread) fitted on OSM public GPS traces, and
  `wayfarer::pace_calibration`, which fits a team's own pace from its GPX
  tracks. CLI: `wayfarer pace default` and `wayfarer pace calibrate`. No
  route reads it yet; stored costs are unchanged.
- **Added: search and rescue planning** (#111; new crate `wayfarer-sar`,
  `docs/design/sar-planning.md`). `predict_reach` (fast, typical and slow
  reach from a last known position, edge probabilities, likely routes and
  corridors, zones), `plan_teams` (segment probabilities, a team × segment
  arrival matrix, a first assignment and territories) and `apply_coverage`
  (tracks and measured effort to probability of detection, with the Bayes
  update), with GeoJSON output. CLI `wayfarer sar reach|teams|coverage`;
  C ABI `wayfarer_sar_reach`, `wayfarer_sar_plan_teams` (cancellable) and
  `wayfarer_sar_coverage` in the library's `sar` build, with schemas in
  `schema/sar-*.json`. Every public type is `#[non_exhaustive]`. On
  Małopolskie a 3-hour reach takes 0.8 s.
- **Added: off-road terrain, experimental** (#112 to #119; new crates
  `wayfarer-layer`, `wayfarer-terrain`, `wayfarer-terrain-compiler`).
  - `wayfarer-layer`: the terrain layer file a model, hazard raster or drone
    survey writes (contract v1, wire version 0.1,
    `docs/design/terrain-layers.md`), with only `blake3`, `flate2` and
    `thiserror` as dependencies so it builds on phones.
  - `wayfarer-terrain`: the terrain file on the global arc-second lattice;
    fusion of layers and a team overlay (boxes, even-odd polygons and
    supercover lines) into an incident surface (contract v1,
    `docs/design/incident-surface.md`) with content and recipe hashes and
    delta updates; 16-connected Dijkstra and A* with pace by grade. An
    owned `incident::Incident` holds the inputs for a host and re-fuses
    only what changed. Blocks are costed as they are fused: a 900 km²
    surface and a walker route take 89 MB of peak heap. Everything is
    integer arithmetic or `libm`, pinned by hashes in CI.
  - `wayfarer-terrain-compiler` (not published): compiles GEM-15, GLO-30
    and WorldCover COG tiles into a terrain file one block row at a time,
    from Rust (`compile_to_file`, `TerrainJob`, cancellable, with progress)
    or the `wayfarer-terrain` binary. Małopolskie compiles in 33 s at
    110 MiB of heap on 4 cores, the same bytes on every target and thread
    count.
- **Faster first route** (#98, #102, #105, #86; POT 1 to POT 3, OM-5,
  OM-6). The CCH topology builds in parallel (on three or more threads),
  open checks run in parallel (on one thread they took 88 ms on
  Małopolskie and about 1 s on Poland), the top of the elimination tree customizes in parallel,
  and the opt-in background warm-up does the first drive's work first.
  Metrics are the same bits on any number of threads.
- **Faster matrices and snapping** (#79, #83, #93, #94): vehicle matrices
  stay pairwise with potentials (a 1 × 1,500 Kraków drive on Małopolskie
  takes 0.4 s instead of 14 s); snapping scores only the candidates that
  can make the limit; CCH potentials start from the candidates' offsets;
  discounted matrices bound their cells with a metric from the request's
  own costs, and matrix paths are moved instead of cloned.
- **Compile headroom** (#81): the road-graph builder emits segment batches
  in parallel, topology edges are handed over in batches (Poland's compile
  peak 4.15 to 3.76 GiB), and a cancel now stops the CCH order (it waited
  170 s on Germany). Output is byte-identical.
- **Added: CLI** `wayfarer nearest` and `--distance-m` contours on
  `wayfarer isochrone` (#84).
- **CI:** the Kraków gate, its timings and the fuzz targets run every night
  (#103); the i686 job tests the layer and terrain crates.
- **Docs:** `ROADMAP.md` (#104, #115), the API-0 decisions (#89), stored
  CCH potentials (#106), the format 4 terrain draft (#95, #115) and the v1
  layer and incident surface contracts (#119).

## 0.4.0 (2026-11-23)

Both lines. Package format 2 (reads 2 only), costing model 1, C ABI version 1,
as 0.3.0. Compiler output is unchanged for complete, id-sorted extracts: they
compile to the same bytes as with 0.3.0, so there is no need to recompile.

### Migrating from 0.3

- **No recompile.** 0.3 packages open, route and give the same results as
  before, and the 0.4 compiler writes the same bytes for them. Only extracts
  with missing node refs or repeated way ids compile differently (#75; see
  Changes), and those now say so in their quality report. Extracts clipped
  at a border can have a few: openstreetmap.fr's Poland extract has 50 ways
  that refer to 265 nodes it does not hold, so its package changes.
- **Results are unchanged, and so is default output, with one addition:**
  route edge attributions gain `inclinePercent` where the way has an OSM
  `incline` (#71; 44 of 2,324 Małopolskie gate responses). Every new
  request field is optional and off by default: route `geometryFormat` and
  `include` (#71), map match `options.attributes` (#73), and matrix,
  isochrone and nearest `overlays`, `departureAtMs` and `includeEdgeIds`
  (#77).
- **The compiler's host protocol request is deprecated** (#76). The old
  request (`inputPbfPath`, `tempDir`, `routeArchivePath`, `buildProfile`) is
  still read, with a warning on stderr, and will be removed before 1.0.
  Send a `CompileRequest`, the JSON the C ABI takes; the CLI README maps the
  old fields.
- **The C library allocates with mimalloc** (#70). Strings it returns are
  still freed with `wayfarer_string_free`, so hosts don't change. A host that
  builds `wayfarer-ffi` from source with its own Rust allocator, or wants the
  system allocator, builds it with `--no-default-features` (and `compiler`
  if it compiles packages). On Windows with `/MT`, the default build also
  links mimalloc's C code against the static CRT.
- **`prefetch()` builds less** (#72): the CCH topology and the default drive
  profile's metric, not the walking one. Call `prepare_potentials()` to
  build every column ahead of time, as before.
- **Release builds of this workspace take about twice as long** (#70: fat
  LTO, one codegen unit). Crates used as dependencies follow the dependent's
  own profile.
- **Rust API (breaking):**
  - `wayfarer-compiler`: `CompileRequest` and `CompileResult` are
    `#[non_exhaustive]`. Build a request with `CompileRequest::new` and its
    builder methods (`.profiles(...)` is new), or set its public fields;
    struct literals no longer compile. `MobilityProfile` is re-exported for
    `.profiles(...)` (#76).
  - `wayfarer`: `RouteMatrixRequest` and `RouteIsochroneRequest` gain
    `overlays`, `departure_at_ms` and `include_edge_ids`;
    `RouteIsochroneMetric` gains `DistanceM`, so
    exhaustive matches need an arm (#77). `RouteRequest` gains
    `geometry_format`, route results gain `polyline6` and `geojson` on every
    geometry, `RouteEdgeAttribution` gains `incline_percent`, and
    `RouteNavigationInput` gains `polyline6` (#71). `MapMatchOptions` gains
    `attributes`, `MapMatchSubPath` gains `edges` and `duration_s`, and
    `MatchedTraceSample` gains `edge_index` and `observed_speed_mps` (#73).
    Struct literals need the new fields; `::new` and `Default` are
    unchanged.
  - `wayfarer-service`: `RouteWorkflowRequest` gains `include` and
    `RouteWorkflowResult` gains `segment_summaries` and `include` (#71).

### Changes

- **Faster release builds of the engine** (#70; perf.md SC-1, SC-2). Release
  builds use fat LTO with one codegen unit, so the engine's hot paths inline
  across `wayfarer-package` and `wayfarer-costing`, and the C library
  allocates with mimalloc (new default feature `mimalloc` on
  `wayfarer-ffi`; `--no-default-features` keeps the system allocator; a
  compile through the C ABI returns freed pages at once, as the binary
  does). Random drives take about 16 % fewer instructions and 12 % less
  time on Linux, where glibc's allocator is already close to mimalloc; the
  gain against the macOS and Windows allocators should be larger. The
  `wayfarer` binary is 7.8 MB instead of 11.6 MB and the C library about
  4.0 MB stripped instead of 4.3 MB. The fuzz targets build the C library
  without mimalloc, so the sanitizers see every allocation.
- **Faster open to first route** (#72; PERF-16, perf.md OM-1 to OM-3). The
  CCH topology is built in one pass, each stored cost column's potentials
  are customized the first time a search needs them, and the strict drive
  column is masked from the stored column instead of costed again. On
  Małopolskie (4 vCPU) the first drive after opening takes 0.34–0.39 s
  instead of 0.74–0.95 s, at about half the CPU, and a drive-only session
  holds about 28 MiB less heap. On Poland the first 892 km drive takes
  6.1–6.9 s and 1,506 MiB of heap instead of 8.9 s and 1,871 MiB.
  `prepare_potentials()` still builds everything and is about a third
  faster. Results are unchanged.
- **Added: nearest targets by route cost** (#74; FEAT-A4).
  `RoutingEngine::nearest` and `wayfarer_nearest` (C ABI 1, additive,
  cancellable) return the k targets cheapest to reach from a source, or to
  reach it from, equal to the k cheapest cells of the full matrix. From the
  source the search stops early (on foot with 1,500 Kraków targets and
  k = 1, about 1,000 times fewer edges; never slower than the full row).
  At most `MAX_NEAREST_TARGETS` (10,000) targets.
- **Added: compact route output** (#71; FFI-6, the first slice of API-8).
  `geometryFormat` on route requests (`points`, the default, `polyline6`
  or `geojson`) puts every route geometry in `polyline6` or `geojson` and
  leaves `geometry` empty; shape indexes point into the decoded vertices
  as before. `include` keeps only the listed optional parts (`segments`,
  `legs`, `edgeAttributions`, `costBreakdown`, `shapeSegments`,
  `recosts`); without `segments` the result has `segmentSummaries`, and the
  result echoes `include`. On Monaco a 3-point route with `include: []` and
  `polyline6` is 17 times smaller. New public `wayfarer::geometry`
  (polyline6 encode and decode, `GeoJsonLineString`), navigation input
  from `polyline6`, `inclinePercent` on edge attributions (OSM `incline`,
  signed by the direction of travel), `wayfarer route --geometry-format`
  and `--include`, and the route GeoJSON FeatureCollection in
  `wayfarer_service::geojson`.
- **Added: road attributes in map matches** (#73; FEAT-A7). With
  `options.attributes: true`, each sub-path lists its edges' attributes as a
  route leg's `edges` (clipped to the matched positions) and its expected
  duration, and each matched sample gives its edge index and the observed
  speed since the previous sample.
- **Added: distance contours, overlays and lean output on analysis
  requests** (#77; FEAT-A9, FEAT-A11). Isochrone contours can be on
  `"distance_m"`, metres along the cheapest path. Matrix, isochrone and
  nearest requests take `overlays` and `departureAtMs`, applied as on a
  route, and `includeEdgeIds: false` to leave out the cells', buckets' and
  hits' edge ids.
- **Added: stored profiles** (#76; K2, P4). `CompileRequest::profiles` and
  `wayfarer compile --profiles` choose the profiles whose costs a package
  stores, one column each (default `wheeled` and `on_foot`; the engine
  derives the others on first use, as before). The same set gives the same
  bytes in any order. On Monaco a third stored profile (legged) adds 71 KB.
- **Fixed: ways with missing nodes and repeated way ids** (#75; K1, bug 15).
  The compiler cuts a way at node refs the input does not hold instead of
  bridging the gap with a straight line (extracts cut with
  `osmium extract -s simple` or `osmconvert -b`), and keeps one copy of a
  way id that appears more than once (merged extracts) instead of building
  overlapping edges. Both are counted in the quality manifest's new
  `inputReport` and reported as quality warnings, which `wayfarer info` and
  `wayfarer compile` show. Packages from complete, id-sorted extracts are
  unchanged.
- **Docs:** the design of package format 3, the one format bump left before
  1.0, with its confirmed decisions (#69, `docs/design/format-3.md`). Format
  3 is not written yet.

## 0.3.0 (2026-11-22)

Both lines. Package format 2 (reads 2 only), costing model 1, C ABI version 1,
as 0.2.0. Compiler output changes: packages compiled by 0.3 have new edge ids
and dataset hashes.

### Migrating from 0.2

- **Recompile to pick up the compiler fixes.** 0.2 packages open and route
  as before (format 2 and costing model 1 are unchanged), but a recompile is
  what brings the turn restriction fix (#27) and the tile-ready ids (#65).
  A recompiled package has new edge ids and a new dataset hash: rebind
  overlay sets bound by edge id, or they fall back to their geometry.
- **Routes change after a recompile** where a turn restriction was tagged
  only for trucks, buses, PSV, goods, emergency vehicles or bicycles (#27;
  common in Germany, absent from Małopolskie), and where endpoint snapping
  chose between equally good candidates by edge id (#65). Requests that set
  `conditionalRestrictionEvalAtMs` route differently on any package (#28).
- **Matrix cells that crossed another point's destination-only area** (a
  pedestrian old town that a third point of the matrix lies in) now take
  the route between their own two points, with its cost (#41; 58 of 10,000
  cells of a Kraków 100×100 drive matrix change).
- **Requests over the new limits are rejected** (#30): more than 250,000
  matrix cells, more than 16 isochrone contours (or empty, non-positive,
  non-finite or repeated ones), correlation radii over 10 km, snap and map
  match `candidateLimit` over 64, and projections over 100 km or 100,000
  points. Matrix and isochrone requests now run the route option checks.
  The C ABI fails them with `request_too_large` or `invalid_request`;
  `RoutingEngine::matrix` fails every cell, `snap` and `project` clamp.
- **Walk matrices of 13 to 64 targets** (with potentials) search pairwise
  and differ from 0.2 in the last bits of their costs, distances and
  durations; statuses and paths are the same (#40).
- **Isochrones reach a little farther** (time contours from a farther origin
  candidate no longer start late), and origins off the road snap as routes
  do instead of failing with `no_candidate` (#35).
- **Lane guidance changes** (#42): `turn_lanes` and `lane_guidance*` now
  describe the edge approaching the junction.
- **Compile progress:** the `collect_roads` stage now comes before
  `index_nodes` (#50), and the ingest message reads "Located N of M nodes
  in K scanned".
- **Rust API (breaking):**
  - `wayfarer`: `RouteManeuver` gains `lanes` (`RouteLane`,
    `RouteLaneIndication`), so struct literals need it (#42).
  - `wayfarer-road-graph`: `RoadNetwork` is a columnar type of its own
    (with `Adjacency`, `EdgeColumn`, `NodeColumn`, `RoadEdgeRef`) instead of
    `petgraph::Graph<RoadNode, RoadEdge>` (#64). `SharedInfoRef::geometry`
    is an `InfoGeometry` (#56) and `SharedInfoRef` gains
    `source_lineage_ids` and `coverage_id`; `SharedEdgeInfos::geometry_mut`
    is `set_geometry`, `push_with_class` takes `&[(f64, f64)]` and
    `SharedEdge` has no public `geometry` (#46). `RoadGraph` gains
    `node_road_type_counts` (#46). New: `WayTemplate`, `TemplateWay`,
    `RoadGraphBuilder::way_template` and `push_template_ways` (#67),
    `SharedInfoIndex` (#46).
  - `wayfarer-package`: `RoadEdge` loses `source_lineage_ids`,
    `coverage_id`, `component_id` and `estimated_travel_time_s` (now a
    method; use the source node's `component_id`), and `RoadNode` loses its
    road type counts (#46, #56). `PackageWriter::section_bytes` returns an
    `Option` (`None` for a spilled section); new `PackageWriter::spilling`,
    `in_memory`, `set_header` and `section_len` (#48).
  - `wayfarer-line-network`: `SharedLineInput.routing` is
    `Option<Arc<RoadRoutingAttrs>>`, and `SharedTopologyEdge`'s
    `source_lineage_ids` and `routing` are `Arc`s (#47, #53).
  - `wayfarer-compiler`: the package build request gains `segment_order`,
    and `package_build::renumber_in_package_order` is public (#65).

### Changes

- **Tile-ready ids (compiler; package bytes change once).** Every id a
  package stores is now a position in an order that follows from the graph
  itself, not from the order the compiler happened to build it in: nodes run
  along a Hilbert curve over one fixed world lattice of E7 positions (not
  over the extract's bounding box), a node's edges are ordered by their way,
  direction, target, segment and length, and node `i`, edge `i` and shared
  info `i` have ids `i`, `i + 1` and `i + 1`; components are numbered by
  their first node and name tokens by their first use. Plaza ways get the
  synthetic id `-9000000000000 - way id` instead of a running number. So a
  later compile split into tiles along ranges of the node order can number
  each tile from the counts of the tiles before it and get the same ids. The
  road graph is renumbered after enhancement
  (`package_build::renumber_in_package_order`, public so a reference oracle
  can share the package's ids); a test shuffles the order segments reach the
  graph builder and checks that every section but the metadata keeps its
  bytes. Format 2 is unchanged and old packages open as before, but a
  recompiled package has new edge ids and a new dataset hash (Monaco
  `991ab6f1…`), so overlay sets bound by edge id need rebinding. The graph
  is the same: every edge with its ends, shape and attributes matches
  0.2.0's, except that plaza centres move in the last bit. Routes keep their
  costs except where endpoint snapping chose between equally good
  candidates by edge id. The road graph's nodes, edges and shared infos are
  stored in the new order, so their ids take no space and the package
  build reads them in place. The renumbering pass costs time in the graph
  step that a faster package build only partly wins back: Małopolskie
  compiles in 17–18 s against 16–17 s before (about +1.4 s, +8 %; five
  alternating runs on a 4-core cloud machine), Poland within run-to-run
  spread (224–233 s against 215–237 s). Peak memory is unchanged.
- **Fixed: turn restrictions for trucks, buses and bicycles banned every
  profile** (compiler). A restriction tagged only for trucks, buses, PSV,
  goods vehicles, emergency vehicles or bicycles (`restriction:hgv` and
  similar, or `except=motorcar`) banned the turn for cars and walkers too.
  It now binds no current profile; the restriction report counts such rows
  in `modeNotRoutedRestrictionCount` (left out when zero). Recompile to pick
  this up. (#27)
- **Fixed: conditional access binds the profiles it names** (engine, with an
  evaluation time). `bicycle:conditional` no longer closes ways to walkers,
  and `hgv:`, `goods:`, `bus:` and `psv:conditional` no longer close them to
  cars. An active `destination`, `delivery`, `permit`, `private` or
  `customers` makes the way destination-only for vehicles instead of closing
  it, and `private`/`customers` close it to walkers. Conditions with `;`
  inside parentheses, `AND` terms (every term must hold) and `24:00` parse
  correctly. Conditions are still evaluated in UTC. (#28)
- **Fixed: unbounded requests** (#30). Every request that takes JSON runs
  the checks `wayfarer::limits` promises, with new limits
  `MAX_MATRIX_CELLS`, `MAX_ISOCHRONE_CONTOURS` and
  `MAX_CORRELATION_DISTANCE_M`, and new `validate()` on
  `RouteMatrixRequest`, `SnapRequest`, `ProjectRequest` and
  `MapMatchRequest` (see Migrating).
- **Fixed: isochrone origins and time contours** (#35). An origin off the
  road widens its snap search as route and matrix locations do, and time
  contours no longer start a farther origin candidate's time at its
  approach cost (Kraków 5-minute drive: +15 % edges).
- **Fixed: matrix cells crossing another point's destination-only area**
  (#41). A cell now opens only its own two points' destination-only areas,
  as routes do.
- **Fixed: lane guidance** (#42). Lanes come from `turn:lanes` on the edge
  approaching the junction instead of the edge entered; unmarked lanes
  count as through lanes, sharp turn lanes match, and the unsuffixed
  `turn:lanes` no longer applies backward on two-way roads. Added: typed
  `lanes` on route maneuvers (indications and `valid`, left to right;
  skipped in JSON when empty).
- **Fixed: maneuver distances on multi-leg routes** (#42). The Waypoint
  maneuver covers the stretch to the next maneuver, so maneuver distances
  and durations add up to the route's.
- **Fixed: cancelled row matrices stop promptly** (#63), within a few
  milliseconds instead of first finishing their reaching-edge pass over
  every target.
- **Faster leg building** (#43): building a route leg's geometry,
  attributes and maneuvers takes about 30 % fewer instructions, with
  identical results (Małopolskie warm median request −19 %).
- **Faster alternatives** (#38): via candidates are listed about twice as
  fast (Małopolskie p95 with two alternatives 96 → 41 ms drive, 43 → 32 ms
  walk), with the same routes.
- **Faster matrices** (#39, #40): pairwise cells of one source reuse the
  CCH potential from it (Kraków 100×100 drive −26 % CPU), and on-foot
  matrices with potentials stay pairwise up to 64 targets (Kraków 50×50
  walk −30 % CPU).
- **Search state is paged** (#44): edge labels, node bounds and potential
  slots are allocated in pages as searches reach the graph, and pooled
  states free pages beyond a per-profile limit (64 MiB desktop, 8 MiB
  mobile). A Poland package's first-route heap is 442 MiB smaller, and a
  matrix no longer leaves graph-sized states behind (8×8 Kraków matrix on
  Poland: +1,350 → +20 MiB kept).
- **Compiler: country-scale compiles in about a third of the memory.**
  The road graph is columnar and drained into the package as it is
  written (#64); the line network and road graph records are slimmer (#46,
  #47, #56); route memberships, routing values and class tables are held
  compactly and once (#51, #53, #57, #59); way facts and finished package
  sections wait in the scratch directory (#48, #55, #58); and the line
  network, graph builder intake and enhancement hold smaller temporaries
  (#52, #54, #60). Packages are byte-identical through all of these. On a
  4-core cloud machine with 15 GiB, Poland's peak goes from 10.7 GiB in
  0.2.0 to 4.5 GiB, Małopolskie's from 1.1 to 0.5 GiB, and Germany (23.6 M
  nodes, 60.4 M edges, 4.26 GiB package) now compiles, with a 10.2 GiB
  peak; 0.2 ran out of memory.
- **Compiler: faster at country scale.** The CCH node order runs ISAP on a
  contracted flow network with the same separators (Poland 151 → 34 s,
  #45); the ingest decodes on every core (#49) and reads the road ways
  first, then only their nodes' locations in one sequential pass, with no
  node store (#50); and the line network topology and the road graph are
  built in parallel up to its emission (#66, #67). On the same machine
  Poland compiles in 224–263 s (0.2.0: 414 s), Germany in 654 s, and
  Małopolskie in 17–20 s (0.2.0: 26–31 s).
- **CI** (#31 to #33, #36, #37): macOS (Apple silicon) and MSRV (1.95) jobs
  and macOS arm64 release archives; API diffs on every pull request
  (`cargo semver-checks`, schema, C header and ABI keys, and a check that
  `wayfarer.h` matches the exported functions); a Kraków centre gate with
  timings and time-boxed fuzzing, run by hand; every job on GitHub-hosted
  runners.
- `experimental-regions` (hidden, unstable) gains a prototype of a
  bounds-guided search across a virtual seam (#29).
- Gate baselines: Małopolskie re-recorded on the 2026-11-18 extract (#26),
  Kraków centre on main (#34) and after the tile-ready ids (#65).

## 0.2.0 (2026-11-19)

Both lines. Package format 2 (reads 2 only), costing model 1, C ABI version 1.

### Migrating from 0.1

- **Recompile every package.** Format 1 packages fail to open with
  `Outdated`; compile the extract again with a 0.2 compiler. Dataset hashes
  change, so overlay sets bound to a 0.1 package fall back to their geometry.
- **Removed Rust items:** `PackageCounts::landmark_count`,
  `profile_landmark_capacity`, `DEFAULT_LANDMARK_COUNT`,
  `PackageBuildOptions::landmark_count`, the landmark and SAR metric fields
  of `PackageBuildReport`, and the `PotentialStats` fields `stored_order`,
  `degree2_eliminated` and `top_separator`. The compile stage `landmarks` is
  `cch_order` (`CompileStage::CchOrder`, `PackageBuildReport::cch_order_ms`,
  JSON `cchOrderMs`); `wayfarer info` drops `landmarkCount`.
- **Debug switches:** `WAYFARER_POTENTIAL=auto|alt|cch` is
  `WAYFARER_POTENTIALS=on|off` (`internals::set_potentials` replaces
  `internals::set_potential`); `WAYFARER_MATRIX_MODE=buckets` is gone, and
  `WAYFARER_MATRIX_MODE`, `WAYFARER_MATRIX_PAIRWISE_MAX_TARGETS` and
  `WAYFARER_CCH_ROW_TARGETS` need the `debug-env` feature.
- **Alternatives change.** Alternatives on packages with potentials come from
  via edges, so the alternatives returned differ from 0.1's (the best route
  does not). Vehicle matrices of 51 to 100 targets differ in the last bits of
  their costs, distances and durations.
- The JSON request and result shapes and the C ABI are otherwise unchanged;
  `wayfarer_open_w` and `wayfarer_verify_w` are new on Windows.

### Changes

- **Package format 2: CCH potentials instead of landmarks.** The compiler
  stores the node order of CCH potentials (new section `CchRank`, computed
  with integer arithmetic only, so packages stay the same bytes on every
  platform) and no landmark distances. Malopolskie: 260 MiB -> 176 MiB,
  compile 20 s, about 0.7 s from open to the first route (the potentials
  are customized then, or in `prefetch`). Routes do not change: both bounds
  are exact.
- **Format 1 is no longer read, and landmarks are gone.** Opening a format 1
  package fails with `Outdated` (recompile it); `MIN_READ_FORMAT_VERSION` is
  2. The engine has no landmark bounds and the compiler cannot write
  landmarks: format 1's landmark sections are `Reserved1` to `Reserved12`
  (written as a package without landmarks wrote them, so format 2 packages
  keep their bytes) and header count slot 6 must be zero. A package without
  a CCH order is bounded by the straight line alone. Breaking API changes:
  `PackageCounts::landmark_count`, `profile_landmark_capacity`, the
  `PotentialStats` fields `stored_order`, `degree2_eliminated` and
  `top_separator`, the `PackageBuildOptions` field `landmark_count` and
  `DEFAULT_LANDMARK_COUNT`, and the `PackageBuildReport` landmark and SAR
  metric fields are removed; the compile stage `landmarks` is now
  `cch_order` (`CompileStage::CchOrder`, the report's `cch_order_ms`).
  `wayfarer info` drops `landmarkCount` and lists `readableFormats` [2]. The
  debug switch `WAYFARER_POTENTIAL=auto|alt|cch` is now
  `WAYFARER_POTENTIALS=on|off` (`internals::set_potentials` replaces
  `internals::set_potential`).
- **Bucket many-to-many matrices removed.** They were opt-in
  (`WAYFARER_MATRIX_MODE=buckets`) and slower than pairwise cells bounded by
  the potentials. `WAYFARER_MATRIX_MODE`, `WAYFARER_MATRIX_PAIRWISE_MAX_TARGETS`
  and `WAYFARER_CCH_ROW_TARGETS` are now read only with the `debug-env`
  feature, like the other debug switches.
- **Every search bounds with the potentials:** stored and derived columns,
  the strict destination-only pass, discounts, opened roads and SAR
  openings (gates at exact offsets), Emergency, Tracked and R5 columns (their
  own metric, customized on first use), matrix rows and overlay patches. On
  Malopolskie drive p95 15.3 -> 9.3 ms, walk p95 55 -> 25 ms.
- **Faster vehicle matrices with potentials:** cells search corridors, so
  vehicle matrices stay pairwise up to 400 targets. Malopolskie 100x100
  16 s -> 2.2 s, 50x50 30 s -> 0.6 s; same statuses and paths.
- **Alternatives through via edges** (packages with potentials): plateaus of
  the shortest-path trees from the origin and to the destination, each
  routed as two searches meeting mid-edge. Two alternatives on the
  Malopolskie corpus: p95 74 ms (862 ms with penalties), and more of them
  (122 against 95). The alternatives returned change; overlays and SAR
  requests keep the penalty method.
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

- **Packages open on 32-bit targets.** Every package was rejected as
  invalid there: a `u32::MAX` key in `ComponentSizes` compared equal to the
  "any key" bound. Header counts, offsets and lengths are now converted
  with `usize::try_from` instead of truncated.
- **More hostile packages are rejected instead of panicking** (found by the
  `package` fuzz target with debug assertions): header counts whose section
  sizes overflow, an edge id base near `u64::MAX`, a request grid whose cell
  count overflows, headings of 360 degrees or more, and piece chains that
  mark a junction. Opening Malopolskie checks the chains in about 3 ms.
  `fuzz/README.md` says to fuzz with `--debug-assertions`.
- **CI on Windows and i686** (workspace tests, the Monaco package compared
  with `fixtures/monaco-info.json` byte for byte, and a route), the Monaco
  determinism test runs by default again, and CI checks rustdoc warnings
  and dependencies (`cargo deny`: advisories, licenses, sources).

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

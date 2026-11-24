# Off-road legs: costed access (X1) and costed SAR off-network (X6)

Status: built on `feat/costed-access-legs` (OFF 1, tranche 0.5), off by
default. Written against `main` at `a6638ab` (0.4.0 plus #93, the approach
offsets of the CCH potentials) and rebased onto `9e58ff8`. Roadmap items: FEAT-X1 (costed access legs) and
step 1 of FEAT-X6 (`off_network.costed`). Terrain (format 4, #95) is a separate
lane: nothing here reads terrain, and an access stretch is priced at one pace
over flat open ground.

The lane analysis this follows (FEAT-X, November 2026) and the off-road SAR plan
are project notes kept outside the repository. Numbers marked *measured* come
from this branch on a 4-core cloud machine.

## 0. Summary

1. **A request option, off by default.** `offNetwork: { maxRadiusM?,
   maxCandidates?, paceMps?, costFactor? }` in `RouteOptions`, so routes, trips,
   matrices and isochrones take it (nearest and snap too, whose candidates then
   carry their access price). `"offNetwork": {}` turns
   it on with the profile's defaults. Without it every result is the same as
   before (section 7).
2. **Every candidate pays its stretch, the nearest one included.** A point
   snaps to up to *k* directed edges within one radius, and each candidate
   costs `costFactor · d / pace` to reach, where `d` is the straight line
   from the point to the candidate's snap point. The searches already seed
   and finish candidates at an approach cost and leave it out of the reported
   cost, so they stay exact over the candidate set. Only the approach
   function changes. With #93 the CCH potentials are seeded with the same
   offsets, so the bounds stay tight.
3. **Results include the stretch.** Route distance, duration and cost include
   both stretches. The leg's geometry starts and ends at the requested points,
   with a shape segment, a maneuver and an `offNetwork` summary for each
   stretch. Matrix cells include both stretches. Isochrones start each
   candidate at its access time, cost and distance.
4. **X6 step 1.** `sar.relaxations.offNetwork.costed: true` turns the R7
   off-network leg into an access leg within `maxLegM`, chosen by total time
   and included in the totals, with `sar.uses[offNetwork].addedCostS` equal
   to what they include. The default stays `false`. Flipping it is the
   breaking step for the API window.
5. **The oracle has its own copy** of the access formula, and parity tests
   cover routes, matrices (pairwise and rows), isochrones (cost and time
   order, plain and through an overlay), trips through an off-network
   waypoint, SAR costed, validation, and "access costs more than the free
   snap".

## 1. The option

```json
"options": {
  "offNetwork": {
    "maxRadiusM": 500,
    "maxCandidates": 16,
    "paceMps": 1.1,
    "costFactor": 2.0
  }
}
```

| Field | Default | Bounds | Notes |
|---|---|---|---|
| `maxRadiusM` | 500 (OnFoot, Legged), 250 (vehicles) | finite, > 0, ≤ 10,000 | Replaces `maxCorrelationDistanceM` and its 300 m / 1 km widening. Rounded up to whole metres, as correlation distances are. |
| `maxCandidates` | 16 | 1 to 64 (`MAX_REQUEST_CANDIDATES`) | Directed edges, so about 8 roads. Replaces `candidateLimit`. |
| `paceMps` | the profile's off-road pace | > 0, ≤ 50 | The same pace as direct legs and R7, so the three agree. |
| `costFactor` | 2.0 (OnFoot, Legged), 1.5 (vehicles) | 1 to 100 | Cost per second of access time. Above 1, roads stay preferred over open ground at the same pace. |

A value out of bounds is `InvalidRequest` with a reason naming the field
(`off_network max_radius_m` and so on), for routes, trips, matrices and
isochrones alike.

The defaults are decision D4 as written in the lane analysis. They live in
`wayfarer_costing::profile_access` (pace and factor) and `wayfarer::access`
(radius and candidates). Nothing is baked into packages, so
`COSTING_MODEL_VERSION` does not change.

The option is request-wide. A per-location override in
`RouteEndpointOptions` was left out of this step (section 8).

## 2. Pricing

For a candidate at straight-line distance `d` metres from the requested
point:

- time `d / pace` seconds;
- cost `costFactor · d / pace`;
- distance `d`.

The search minimises cost and reports time and distance, as it does for
edges. The default paces (`profile_off_road_speed_mps`, open ground as a dirt
track) and what they give:

| Profile | Pace | Factor | Cost pace | 500 m stretch |
|---|---|---|---|---|
| OnFoot | 1.20 m/s | 2.0 | 0.60 m/s | 417 s, cost 833 |
| Legged | 1.63 m/s | 2.0 | 0.81 m/s | 308 s, cost 615 |
| Wheeled | 2.75 m/s | 1.5 | 1.83 m/s | 182 s, cost 273 |
| Tracked, Emergency | 4.50 m/s | 1.5 | 3.00 m/s | 111 s, cost 167 |

Emergency takes the faster of the wheeled and tracked paces.

With the defaults a walker's cost pace is 0.6 to 0.8 m/s, close to the
0.47 m/s constant of today's relative approach penalty, so access legs land on
about the same roads that the free snap picks in a city, and the totals now
say what the walk costs.

`AccessPolicy::access_from` prices a point once and every candidate of that
point carries the result (`CorrelatedLocation.access`, a `CandidateAccess` of
the requested point, pace and factor). Every call site that read an approach
cost now reads it through the candidate, so routes, alternatives, matrices,
isochrones and the patch search all switch together.

## 3. Candidates

With the option on, the correlator makes one search within the radius for up
to *k* candidates (`Correlator::search_access`), ordered by distance (and so
by access cost, since a point has one price per metre), then edge id. It keeps
the connectivity rule of `search_expanding`: when no candidate is connected (a
component of at least 50 nodes and a saturated 64-edge reach both ways), a
second search for connected candidates only fills the slots, with islands in
at most half of them. There is no widening ladder: a point with nothing within
the radius has no candidate, which is `NoCandidate` as usual.

## 4. Searches

The searches already did what X1 needs. Each one seeds an origin candidate at
its approach cost, finishes at a destination candidate's approach cost, and
subtracts both from the cost it reports. X1 changes the approach function
only:

- `package::search::approach_cost` and `approach_costs_by_target` return the
  access cost for a candidate with access, and today's relative penalty
  otherwise;
- `unreported_approach_cost` is the part of the approach a leg result does
  not carry: zero with access, since the leg's totals add the stretch back.
  Alternatives' totals use it for the primary route and the full approach for
  each alternative, so a primary and its alternatives compare on the same
  terms.

An access cost is a super-source arc S → s, so bounds built without it stay
admissible. #93 (QW-3) already passes the offsets into the CCH potentials, so
the bidirectional search keeps a tight bound and does not lose the access time
from it.

### A tie in fastest-time isochrones through overlays

Fastest-time isochrones (`order: time`) through an overlay patch reported
extra boundary crossings once access was on. Two candidates of one point can
meet at a node at exactly the same time but different cost: one is the
candidate at the start of the edge, the other is the candidate at the end of
the previous edge plus a turn that costs but takes no time. The plain search
keeps the first label in the edge's one slot. The patch labels a seed fragment
and the whole edge in separate slots, so it settled both and reported
crossings from the costlier one. Its dominance check now breaks a tie on time
by cost, and only a label that was actually queued completes an edge. The
change only matters where two labels tie on time at different costs on an edge
with a patch fragment. Without access, the candidates of one point all start at
the same time, because fastest-time isochrones keep only the nearest road's
candidates. The overlay analysis dumps in section 7 are unchanged.

## 5. Results

### Routes

- `distanceM`, `durationS` and `cost` of the route and its leg include both
  stretches.
- The geometry starts and ends at the requested points. Each stretch of 1 m
  or more is a densified geodesic (`densify_geodesic`, as direct legs use)
  with:
  - a shape segment flagged `off_network_access`, with a dynamic overlay of
    kind `access_leg` carrying its distance, cost and duration;
  - a maneuver. At the origin it is the `start` ("Head NE across open ground
    for 340 m to Main Street", key `route.access_depart`), and the road's old
    start becomes a `continue`. At the destination it is a `continue` before
    the `destination` ("route.access_arrive").
- `offNetwork: { origin?, destination? }` on the leg gives each stretch's
  point, join point, edge id, fraction, distance, duration, cost, pace and
  factor. Shorter stretches are in the totals and the summary only.
- Edge ids and attributions are unchanged: a stretch is on no edge.
- Navigation and guidance treat the access segment as off-graph, as they treat
  host connectors, and take its time from the overlay's `durationSeconds`.

### Multi-leg trips

A waypoint reached by an access stretch is not continued from its snapped
point: the next leg starts from the waypoint itself with a stretch of its own.
A trip through an off-network waypoint then passes through it, with no
`waypointGap`.

### Matrices

Cell cost, duration and distance include both stretches. `AnalysisPoint`
carries the access totals separately from the approach cost (which bug B2
conflated), and `AnalysisPath::with_access` adds them in every way a cell is
found: same-edge direct, pairwise, rows and patch rows.

### Isochrones

Each candidate starts at its access time and distance, and at its access cost.
Fastest-time isochrones keep every candidate when access is on, since each
starts after its own stretch. Without access they keep only the nearest
road's candidates, as before.

### SAR off-network (X6 step 1)

`sar.relaxations.offNetwork.costed: true` (default `false`, not serialised
when false):

- turns the leg into an access leg within `maxLegM` (default 2 km, at most
  10 km), with the profile's off-road pace and a cost factor of 1, so the join
  is chosen by total time, as the lane analysis proposes for X6, rather than
  by nearest connected road;
- puts it in the totals, geometry and maneuvers as above;
- keeps `sar.uses[offNetwork]`, now with densified geometry, and with
  `addedCostS` equal to what the totals include. With a factor of 1 that is
  both the added cost and the added duration, so the API window's migration
  note ("subtract `addedCostS` for the old value") holds for durations.

A request's own `offNetwork` wins over a costed SAR one. Without `costed`, R7
is unchanged.

## 6. Oracle and tests

`reference.rs` has the access formula written out (`access_of`), its own
approach costs, and adds the stretch in its own path building for routes,
matrices (`PlainPath::matrix_path`, `OverlayPath`) and isochrone seeds. It
shares the correlator, which is acceptable because correlation is not what the
oracle checks.

New tests (`package/tests.rs`), on `large_grid_graph` with points at block
centres:

| Test | Checks |
|---|---|
| `access_leg_routes_match_the_reference_and_include_their_stretches` | engine vs oracle per profile; totals = search + stretches; geometry ends at the requested points; segments, maneuvers, summary |
| `access_legs_cost_more_than_the_free_snap` | the same route with and without access |
| `access_leg_matrices_match_the_reference_and_routes` | pairwise and rows vs oracle, and each cell vs its route |
| `access_leg_isochrones_match_the_reference` | cost and time order vs oracle; through an overlay patch vs plain; a contour shorter than the nearest stretch reaches nothing |
| `access_legs_pass_through_off_network_waypoints` | a three-point trip, no waypoint gap, geometry through the waypoint |
| `costed_sar_off_network_legs_are_in_the_totals` | X6: totals, `addedCostS`, geometry |
| `access_leg_options_are_validated` | every bound, routes, trips, matrices, isochrones |

## 7. Measurements and unchanged results

**Unchanged with the option off** (*measured*). `gates.sh dumps` on
Małopolskie (the 2026-11-18 extract, 184,954,688 bytes) and Monaco, with
binaries built from `main` at `9e58ff8` and from this branch, gives 31
byte-identical files: route responses and correlation candidates, the route
parity harness (default, non-default costing, overlays) and the matrix and
isochrone dumps, plain and with overlays. The Kraków centre gate passes with
the same visited edges as its baseline (corpus 20,984, random 29,470), and the
Monaco package info is unchanged.

**100×100 matrices with X1 on** (*measured*, `analysis_parity` with
`ANALYSIS_OFF_NETWORK`, package engine, Małopolskie; warm wall time of two
rounds of 3 repeats, on a 4-core machine that the runs share, so differences
under about 10 % are noise; measured before the rebase onto `9e58ff8`, whose
access fix (#97) changes the Małopolskie package a little):

| Matrix | Off | On, k = 16 | On, k = 8 | Visited edges off → k = 16 → k = 8 |
|---|---|---|---|---|
| Kraków drive | 1.42 / 1.46 s | 1.42 / 1.36 s | 1.41 / 1.53 s | 10.36 M → 10.15 M → 10.17 M |
| Kraków walk | 6.85 / 6.90 s | 7.40 / 7.57 s | 6.67 / 6.99 s | 233.2 M → 231.2 M → 224.4 M |
| Region drive | 5.33 / 4.33 s | 4.21 / 4.59 s | 3.91 / 4.29 s | 61.6 M → 60.2 M → 60.0 M |

- Every matrix visits slightly fewer edges with access on. The risk the
  analysis named (more than 8 distinct target nodes dropping the row search's
  CCH potentials) does not show: with the offsets of #93 the bounds stay
  tight. The walking matrix at k = 16 takes 8 to 10 % longer, from handling
  twice the seeds per row; k = 8 is level with off.
- Cells grow by what the stretches add: median +10 s and +19 m (Kraków
  drive), +29 s and +35 m (Kraków walk), +28 s and +59 m (region drive). The
  95th percentile cell is 8 % longer in Kraków by car and 2.4 % on foot.
  Some cells get shorter, by up to 21 minutes: the free snap's nearest road
  can be a long way round, and with access a slightly farther road wins.
- **Two points become unreachable** (198 of 10,000 cells, one point in each
  of two matrices): one in eastern Kraków for walking (50.0796,
  20.0815), one in the south of the region for driving (49.4810, 20.2646). Neither has a connected road within its radius (500 m, 250 m),
  only island candidates. Without access, correlation widens to 1 km and
  finds one. With access the radius is a cap by design (one search, no
  widening ladder), so a host that wants such points reached sets a larger
  radius.

## 8. Limitations and follow-ups

- **No per-location override.** The option is request-wide. Matrix and
  isochrone requests have no per-point options, and a per-location
  `offNetwork` in `RouteEndpointOptions` for routes is a small additive
  follow-up.
- **Host origin connectors still win.** With a host origin-endpoint connector
  the patch search drops candidate seeds (B5), so access candidates do not
  compete with it. The lane analysis proposes that both compete.
- **No barriers.** A stretch is a straight line and crosses buildings, yards,
  rail and water. X2-lite (motorway barriers from edges) is next, then X2 with
  format 3 barrier sections. Until then the cost factor is what keeps walkers
  on streets.
- **No cap per shared info.** The analysis suggests at most 2 directed
  candidates per shared info. Today the k nearest directed edges are kept,
  which in a city is about k/2 roads anyway.
- **No two-phase scan.** A 10 km SAR radius pays for every in-window edge's
  record and reach (QW-2).
- **Join point at the foot.** The candidate is the nearest point on the edge.
  The Snell refinement (enter `d / sqrt(r² − 1)` ahead of the foot) saves a
  few per cent at a factor of 2 and is a follow-up (0.6).
- **No matrix cell access fields.** `sourceAccessM` and `targetAccessM` were
  optional in the analysis and are left out.
- **Not on by default.** Open question 11 (walkers on by default with a small
  radius) waits for X2-lite.
- **Follow-ups:** X1b (automatic leg mode: a direct leg when it is faster than
  the routed leg with its stretches), the X6 default flip in the API window,
  and terrain-aware pace (X3) on format 4.

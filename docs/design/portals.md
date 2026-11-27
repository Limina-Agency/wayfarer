# Portals: routes that start or end on open ground (W3)

Status: proposal with the first two pieces built (the priced-access seam,
section 2, and the terrain fields, section 3.1), for review before portal
generation lands. Written against
`main` at `e1d070c` (0.6.0). Off-road plan W3, "portals and blending": PLAN
section 8 ("0.8 incident-surface portals: each cell a road passes through,
cheapest per directed edge, truncation reported"). The terrain side is
`wayfarer-terrain` (incident surface, `docs/design/incident-surface.md`). The
road side is X1's costed access legs (`docs/design/off-road.md`), which this
generalises.

## 0. Summary

1. **A portal is an X1 candidate whose access is priced by the terrain.**
   X1 prices a candidate at `costFactor · d / pace` along a straight line. A
   portal carries a cost, a time, a distance and a path that a least-cost
   search over the incident surface found, from the requested point to a post
   that a road edge passes through. Every search, the oracle and the result
   code already treat a candidate's access as a super-source arc. They keep
   doing that and read the price from the candidate, so routes, matrices,
   isochrones, alternatives and trips take portals with no new search code.
   They stay exact over the candidate set, as X1 is.
2. **The engine gets a priced-access seam, not a terrain dependency in its
   search code.** `CandidateAccess` gains a priced form (section 2). The
   searches and result code read only that. Portal generation (section 3) is
   a separate module, `wayfarer::portals`, behind an optional `terrain`
   feature that pulls in `wayfarer-terrain`. It turns a requested point, a
   profile and an incident surface into priced candidates.
3. **One terrain search per endpoint, then one candidate per directed edge.**
   A bounded Dijkstra over the surface runs from the point (from origins
   forward, to destinations backward, because slope cost depends on the
   direction of travel). Every directed road edge within the bound is walked
   post by post along its shape. Its portal is the reached post with a legal
   exit that minimises access plus the edge's remainder. Candidates are ranked
   by that key, and the best `maxCandidates` are kept. The result reports how
   many were cut (truncation, section 4).
4. **Exit legality and switch cost live in portal generation**, shared with
   the oracle (section 3.3). No exit is allowed onto motorways, motorroads,
   bridges, tunnels or layers other than 0. None is allowed across a barrier
   line on the portal post, or from a post steeper than the profile's
   embankment class. Vehicles exit only from tracks, unpaved and minor roads,
   onto posts their profile can pass. A fixed switch cost per profile is
   added once per stretch.
5. **Results:** totals include the terrain stretch. The geometry is the
   terrain path. The `offNetwork` summary gains the stretch's confidence
   (`RouteConf`) and the surface's content hash. `truncated` says whether
   cut candidates could have won.
6. **Scope of W3:** the Rust API (`RoutingEngine` calls that take an incident
   surface), the oracle and parity tests. The JSON request, the service and
   the C ABI come with W4 (incident API and FFI). Two-way portals mid-route
   (walkers leaving and rejoining the road) are 0.9 (W7), and they reuse the
   same portal generation.

## 1. Why candidates, not a joint graph

A joint search over road edges and surface posts would be exact everywhere. It
would also break the CCH bounds, the stored potentials and the matrix modes,
and it would need its own oracle. Portals keep the road search as it is. Open
ground is one priced hop at each end. That hop is exact as long as the best
route leaves the road network at most once at each end. This is the case W3
covers: a last known position, a team start or a helicopter landing zone off
the road. A route that leaves and rejoins the network in the middle is W7.

## 2. The seam: priced access (built)

```rust
pub struct CandidateAccess {
    pub point: GeoPoint,
    pub pace_mps: f64,
    pub cost_factor: f64,
    /// A price found by the caller, in place of the straight-line formula.
    pub priced: Option<Arc<PricedAccess>>,   // not on the wire
}

pub struct PricedAccess {
    pub cost: f64,
    pub duration_s: f64,
    pub distance_m: f64,
    /// From the requested point to the snap point (origin) or back.
    pub path: Vec<GeoPoint>,
}

pub trait AccessPricer: Send + Sync {
    fn price(&self, query: AccessQuery<'_>) -> Vec<CorrelatedLocation>;
}
// RouteOffNetworkOptions::pricer: Option<AccessPricerHandle>  (Rust only)
// AccessQuery { point, profile, role: AccessRole::{Origin, Destination},
//               policy, candidates, view }
```

- With `offNetwork` and a `pricer` on a request, the correlator hands each
  endpoint's candidates to the pricer and keeps what it returns, at most 64.
  Every query correlates through `Correlator::search_expanding`, which now
  takes the point's `AccessRole`. Routes, matrices (sources are origins,
  targets destinations), isochrones, nearest (by direction) and the oracle all
  pass it, so engine and oracle get the same priced candidates.
- `AccessTotals::of` and the approach costs (package search, patch search,
  oracle) read the priced totals when present, and the straight-line formula
  otherwise. Nothing else in the searches changes. `CandidateAccess` is no
  longer `Copy`.
- A stretch is drawn along `path` when it has two points or more, with its
  ends set to the requested point and the join. Otherwise it is X1's
  densified straight line. The shape segment, overlay, maneuver and summary
  code is X1's.
- Matrices, isochrones (cost and time order) and trips need nothing more.
  Their seeds come from the candidate's access cost and time already.
- `priced_access_routes_and_matrices_match_the_reference` checks this with a
  test pricer. It drops some candidates, sets made-up prices and a bent path,
  and records the roles. Routes and matrices match the oracle. Totals are the
  road's plus the priced stretches, the drawn stretch is the path, and each
  end is priced with its own role.

Bounds stay admissible because a priced access is still a super-source arc. The
CCH potentials are seeded with candidate offsets (#93), so they take the
priced cost as they take X1's.

## 3. Portal generation (`wayfarer::portals`, feature `terrain`)

```rust
pub struct TerrainPricer {     // implements AccessPricer
    pub surface: Arc<Surface>,
    pub grid: Arc<Grid>,           // built once per surface (Incident caches it)
    pub profiles: ProfileMap,      // MobilityProfile -> surface profile index
    pub max_access_s: f64,         // default 1,800 walkers, 600 vehicles
    pub switch_cost_s: f64,        // default 10 walkers, 45 vehicles
}
// a request: offNetwork { maxRadiusM: 3000, maxCandidates: 16,
//                         pricer: TerrainPricer }
```

A request's endpoints are correlated as today. An endpoint is priced by the
terrain when the `TerrainPricer`'s surface covers it and it is farther than the snap radius
from every road, or when the request asks for it. That decision is section 6,
question 2.

### 3.1 The terrain search

- Bounded Dijkstra on `wayfarer_terrain::search::Graph` from the point's post
  (`Search::run_field(.., Toward::From, limit)`, built), stopping at
  `max_access_s`. Destinations run `Toward::To`, so each move costs what it
  costs walked towards the destination, and `path_to` reads their paths in
  walking order. `fields_from_and_to_a_cell` checks both against the
  point-to-point search, the walked cost of each path, and the bound.
- The point's own post is the start, even when the point lies inside a cell.
  The difference is at most half a post (about 15 m at 1″). It is charged at
  the post's own time per metre, as the first move.

### 3.2 Portals on each directed edge

- Directed edges come from the package's spatial grid within the box (the
  correlator's index). The profile's access mask is the same one
  `search_access` applies.
- Each edge's shape is walked through lattice posts with the same exact
  supercover the overlay lines use (`fuse::supercover_*`). Each post it
  crosses becomes a portal at the shape point closest to the post's centre,
  with that point's `fraction`.
- The key of a portal at fraction `f` on an origin edge is the access cost,
  plus the switch cost, plus `(1 − f)` of the edge's cost: what the seed would
  cost at the edge's head. For a destination edge it is the access cost, plus
  the switch cost, plus `f` of the edge's cost. The engine computes the
  edge-cost part with the request's costing, as seeds already do. Each
  directed edge keeps its portal with the least key. Ties go to the lower post
  index, then the lower fraction.
- All directed edges are ranked by key, then by edge id, and the first
  `max_candidates` are kept. Fractions and costs are fixed point (µs and
  1/2^32 of the edge) until they are handed to the engine as `f64`, so the
  choice is the same on every target.

### 3.3 Exit legality and switch cost

A post is a legal portal for a profile when all of these hold:

| Rule | Source |
|---|---|
| The edge is not a motorway, motorway link, motorroad, bridge or tunnel, and its layer is 0 | edge attributes in the package |
| The surface lets the profile stand on the post (time per metre above 0, closure risk below the profile's limit) | the surface |
| The post's line mask has no barrier bit (`lines::BARRIERS`: river, stream, rail, fence, cliff) and no team block | the surface's line mask |
| The post's steepness-max class is at most the profile's embankment class (walkers 9, vehicles 5; field trial) | the terrain |
| Vehicles: the edge is a track, unpaved, or of road class minor or lower | edge attributes |

The switch cost is added once per stretch, in time and in cost. It stops
search results from zigzagging between road and terrain for a few seconds.
The defaults are 10 s for walkers and 45 s for vehicles, until the field trial
calibrates them.

### 3.4 Cost of the stretch

The terrain search gives time in µs. The stretch's duration is that time plus
the switch cost. Its cost is duration × `costFactor`, where the factor
defaults to 1 for portals: the surface's time already says what open ground
costs. X1's 2 and 1.5 were a stand-in for that and are not applied again. The
distance is the length of the terrain path. Its confidence is
`search::route_conf` of the path.

## 4. Truncation

A search over the kept candidates is exact. A cut candidate could still have
given a cheaper route if its key, minus the best kept candidate's key, is
less than the network cost it would save, which is unknown. The result
reports both:

- `truncated: { cut: n, bestCutKey, worstKeptKey }` on the leg's off-network
  summary, when `n > 0`;
- the oracle tests compare against a run with no cut (`max_candidates` = all)
  and check that the same route comes back whenever `bestCutKey − worstKeptKey`
  exceeds the route's cost spread. This is the plan's "truncation reported".

## 5. Tests and measurements

- **Parity:** routes, matrices, isochrones and trips with portal candidates
  against the reference oracle with the same candidates. This uses the
  analysis and route parity suites, with candidates from the E14 Tatra
  surface on the Małopolskie test package.
- **Plan acceptance (0.8 row):** a route from an open-ground point to a road
  destination through a portal matches the oracle, and the terrain and road
  totals add up.
- **Determinism:** the same surface and request give the same candidates and
  bytes on x86_64, i686 and aarch64. This is pinned by hash, as the terrain
  tests are.
- **Cost:** a 30-minute walker field on the Tatra AOI is about 30,000 to
  60,000 posts. With E14's search speed that is a few ms. The plan budget is
  5 to 15 ms per endpoint on a phone.

## 6. Questions for Anton

1. **Where portal generation lives.** Recommended: `wayfarer::portals` behind
   a `terrain` feature, so exit legality reads the package's edge attributes
   directly and the oracle shares it. The alternative is a separate glue crate
   that builds `PricedAccess` from outside. It keeps `wayfarer` free of the
   terrain crates, but it needs a public edge-attribute and shape API first.
   `wayfarer-sar` already wants that API (#111 review notes).
2. **When an endpoint uses the terrain.** Recommended: only when the request
   names a surface and the point is farther than the snap radius (25 m) from
   every road edge its profile may use. A point on a road snaps as today. A
   switch to force terrain pricing is for SAR's last known position.
3. **The cost factor on terrain stretches.** Recommended: 1, as section 3.4
   says. X1 keeps 2 and 1.5 for straight lines with no terrain.

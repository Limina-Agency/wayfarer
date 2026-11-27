# SAR planning: reach prediction, likely routes and team coverage

Status: design and first slice (road and trail network). Crate: `wayfarer-sar`.
Builds on `off-road-sar/PLAN.md` section 3 (S4 probability v1, S5 team reach and
assignment, S6 track coverage, S9 probability v2, S10 hazard overlays) and the E4
pace results (`off-road-sar/exp/E4/README.md`).

## 1. What a search planner asks

1. **Where could the person be now?** From an approximate last known position
   (LKP, with an uncertainty radius) and the time since they were last seen: the
   area they could have reached, for a fast, a typical and a slow person.
2. **Which way did they likely go?** The routes from the LKP to the edge of that
   area and to places the planner names (car parks, huts, villages), and the
   corridors many of those routes share.
3. **What will they avoid?** Zones the person cannot enter (a river, a cliff
   band: hard) or tends to avoid (dense forest, a steep slope, private ground:
   soft), and zones they are drawn to (trails, drainages, structures).
4. **Where should teams go first?** For each search segment, its probability
   (POA), which team gets there first and when, a first assignment, and each
   team's reach within its shift.
5. **What has been searched?** Team GPS tracks turned into covered paths, a
   probability of detection (POD) per segment, and the probabilities updated
   after an unsuccessful search.

## 2. Model (deterministic, replayable)

Everything runs on the engine's isochrones, matrices, projections and map
matching, so a plan is a pure function of the package, the request and the
explicit times (S3 provenance can record it). No wall clock is read.

### 2.1 Time budget and person factor

- `elapsed = atMs - lastSeenAtMs` (or `elapsedS`). The most time moved is
  `elapsed * movingFraction` (default 1, the outer bound: a person who never
  rests). Conditions scale speed (`speedFactor`, e.g. 0.9 for snow or night,
  E4 point 8), so the model time budget is `moving * speedFactor`.
- The engine's path time `t(e)` is the time at the profile's costing speed
  (OnFoot: 5.1 km/h on footpaths, less on rough surfaces). A person with time
  factor `F` (actual time / engine time) reaches `e` when `F * t(e) <= budget`.
  `F` is log-normal, fitted to three percentiles (`p10`, `p50`, `p90`): median
  `p50`, sigma from `(ln p90 - ln p10) / 2.563`.
- Reach contours are `budget / p10` (fast person: the outer bound),
  `budget / p50` (typical) and `budget / p90` (slow).

**Where the factors come from: the walking pace model** (`wayfarer::pace`,
`docs/design/pace-model.md`). On foot, the request's `pace` (a team or region
calibration from `wayfarer pace calibrate` on its own GPX tracks) or the
built-in E4 model sets them:

1. `ratio` = the pace model's expected time over the engine's, summed over the
   edges a typical person reaches (trail class from `highway` and `sac_scale`).
   The first search uses an unclassified footpath's ratio (1.366 for the
   built-in model); when the measured ratio differs by more than 2 %, the
   search runs again with it.
2. `p10, p50, p90 = ratio * the model's time percentiles` (built in: 0.728,
   0.982, 1.321).

Packages carry no heights yet, so every stretch is flat and only the flat speed
and trail factors apply (`slopeApplied: false`); the slope curve applies with
format 4. A team model makes the whole reach faster or slower and shifts
the trail-class mix. `personFactors` in the request replaces all of this, and
profiles other than on foot use E4's flat-pace factors (1.03, 1.42, 2.10) with
a warning. Teams in a plan take their own `pace` (or the plan's), with the
ratio measured on the segments' paths, times their `paceFactor`.

### 2.2 Where the person is (probability of area v1, on the network)

The person walked along some path for `u * budget / F` of model time and is at
its end, where `u` is the share of the moving budget they used before they
stopped (rested, sheltered, got hurt, wandered back). `stopping: uniform`
(default) takes `u` uniform in (0, 1], averaged over 128 midpoints;
`stopping: never` takes `u = 1`, everyone at the end of their full reach (the
outward bound). The reach isochrone is cut into time bands `[t(k-1), t(k)]`
(default 12 equal bands up to the outer contour `budget / F(0.02)`). The mass
of band `k` is `P(t(k-1) < u * budget / F <= t(k))`. What lies beyond the last
band is **rest of world** (ROW), reported, never dropped. Inside a band, mass
is spread over the path length the band covers (time taken as uniform along
each edge, from when the edge is first entered to when it is wholly reached),
times zone weights. A band with no path in it (the network ended) puts its
mass in `offNetwork`.

Optional distance rings (host-supplied lost-person statistics by subject
category, ISRID-style; PLAN decision 4: Wayfarer bundles no table): cumulative
`(probability, distanceM)` pairs from the LKP. Ring mass is spread over the
reachable network in each ring by length; the two models are mixed,
`P = (1 - ringWeight) * P_time + ringWeight * P_rings` (default 0.5 when rings
are given). This is the consensus approach SAR planners already use.

### 2.2b Off the paths (X1 access legs) and reach polygons (A1)

A last known position is often off the paths: in forest, on open ground, at
a crag. With `offTrail` (default `true`), every search a planning step runs
turns the engine's costed access legs on (`options.offNetwork`, X1, unless the
request sets its own): the LKP, each uncertainty point and each destination
reach up to 16 path candidates within 2 km (walkers; vehicles take the
engine's 250 m), each at the straight-line distance over the profile's
off-road pace, and that time is in every contour, band and route. The person
factor scales it like any other model time. Two effects on the Tatra demo
(measured, 1 h): from a point 384 m off the paths the fast, typical and slow
reach is 25 / 14 / 10 km of path against 15 / 8 / 4 km with the old free snap,
because the person can walk to any path within reach and not only to the
nearest one; from 30 m off a path the 3 h reach changes by under 3 %.
`offTrail: false` gives the old free snap. Team start points and segment access
points are priced the same way in a team plan.

`polygons` (A1's `RouteIsochronePolygonOptions`) draws each person's reach as a
GeoJSON `MultiPolygon` (`areas[].polygon`, and `complyingAreas[].polygon` with
avoid zones): the ground within a short walk of the reached paths, and where
the reach ends part way along a path, the rest of the time at the off-road
pace. On the Tatra demo (3 h) it adds 1.1 s and 2.5 MB of JSON; the `geojson`
output gains a `reach` feature per person. Probability stays on the paths
until the incident surface (step 3).

### 2.3 Zones

| Kind | Effect on the search | Effect on probability |
|---|---|---|
| `no_go` | `NoGo` overlay in every run (cannot enter) | none inside |
| `avoid` | `NoGo` in the *complying* run only | mixture: `compliance * P(complying run) + (1 - compliance) * P(free run)`; `avoidCompliance` default 0.8 |
| `weight` | none | edge weight times `factor` (e.g. 2 near a trail junction, 0.5 for open fields), then renormalised |

Two isochrones (complying and free) cost one more search and keep the time
contours exact under fastest-time order, which a `SlowGo` cost multiplier would
not (it changes cost, not time).

### 2.4 Likely routes and corridors

A one-to-many matrix from the LKP to (a) the planner's destinations and (b)
frontier points sampled from the fast contour's boundary fragments, one per
bearing sector (default 24 sectors, the farthest fragment in each). Each route
gives model time, arrival time for P10/P50/P90 people, whether each could have
got there, distance and edges. Corridors: per edge, how many sampled routes use
it (share of routes), which highlights the trails most paths funnel through. The
routes respect `no_go` and `avoid` zones (the complying run).

### 2.5 Teams (S5)

- **Segments as drawn:** the planner's polygons are used as given: any
  winding, never split, merged or clipped. An edge in two overlapping segments
  counts in each, and a segment with no path inside still gets arrival times.
- **Time to segment:** a matrix per profile from team start points to each
  segment's access point (given, or a point inside the polygon: its centroid,
  or for a concave shape whose centroid falls outside, the middle of its widest
  span), times the team's pace factor plus its start offset.
- **Segment POA:** given, or the sum of the reach edge probabilities inside the
  segment polygon (engine projection).
- **First assignment (greedy, deterministic):** segments by POA, highest first;
  each takes the free team that arrives first (ties by team id). The full
  arrival matrix is returned so the planner can override.
- **Territories:** with a shift length, one isochrone per team (mixed profiles,
  own pace and start offset); each edge goes to the team that reaches it first.

### 2.6 Coverage and update (S6)

- Each team track is map-matched (the engine's HMM) into covered edges, both
  directions as one path. Passes add up: `POD_edge = 1 - (1 - pod)^passes`, with
  the track's `pod` (default 0.8, a hasty search along a trail; uncalibrated, the
  host sets it per team or task).
- **Host-supplied effort** (`efforts`): search the host measured itself, such
  as a drone's or camera's video coverage, counts like a track. Each effort
  names its source and gives a detection as a probability (`pod`) or as seconds
  seen (`seenS`, turned into `1 - exp(-seen / tauS)` with the effort's
  `tauS`), in one of three shapes:
  - `edges`: per edge id, for a host that scores the network itself;
  - `grid`: cells of a regular lat/lng grid from `southWest` (only searched
    cells listed); an edge takes the length-weighted mean of the cells along
    its shape, the chance of detecting a person anywhere on it;
  - `polygon`: one detection for every edge inside a polygon (a sweep the team
    scored by hand).
  Tracks and efforts are independent searches, so an edge's misses multiply:
  `POD_edge = 1 - prod(1 - POD_track) * prod(1 - POD_effort)`. The result
  reports each effort's edge count, covered length and mean POD.
- Segment POD is probability-weighted: `sum(POA_e * POD_e) / sum(POA_e)`; segment
  coverage is covered length over the segment's network length.
- Bayes after an unsuccessful search: `POA'_e = POA_e (1 - POD_e) / (1 - sum POA POD)`,
  ROW included, so the probability flows to unsearched ground and ROW.

## 2.7 What the planner supplies, and where it comes from

| Input | Where to find it |
|---|---|
| LKP or point last seen, time last seen | the reporting party, phone location, a found item |
| Subject category distance statistics (`distanceRings`) | Koester, *Lost Person Behavior* (ISRID data; licensed, so the host enters its own table), or the team's own incident history |
| `no_go` zones | water bodies, cliff bands, closed areas: local knowledge, OSM water/natural areas, park rules |
| `avoid` and `weight` zones | dense forest or dwarf pine, private ground (avoid); trails, drainages, huts, viewpoints, roads (weight above 1): local knowledge and subject profile |
| `personFactors`, `speedFactor`, `movingFraction` | E4 defaults; the subject's fitness, age and conditions (snow, night) |
| Team `paceFactor`, track `pod` | the team's own GPX residuals (S1 import) and sweep-width tables; start values here are uncalibrated |
| Effort `pod`, `seenS` and `tauS` | the host's own sensor model (for video: seconds a cell was in view at usable resolution, and the time constant from its detection trials) |

## 3. What ships now vs. after terrain

| Step | What | Needs | When |
|---|---|---|---|
| **1 (this branch)** | `wayfarer-sar` crate: reach prediction with P10/P50/P90 bands, LKP uncertainty, elapsed time, conditions, zones, distance rings, likely routes and corridors, segment POA, team arrival matrix and assignment, team territories, track coverage, POD and Bayes update, GeoJSON output | isochrones (A2, A10), matrices, projection, map matching: all on main | 0.5 / 0.6 |
| **1b (this branch)** | Reach polygons per person (`polygons`, section 2.2b) | A1 (#107, on main) | 0.5 / 0.6 |
| **1c (this branch)** | Off-trail starts and ends priced: the LKP, uncertainty points, destinations, team starts and segment access points (`offTrail`, section 2.2b) | X1 costed access legs (#108, on main) | 0.5 / 0.6 |
| **1d (this branch)** | CLI `wayfarer sar reach|teams|coverage`; C ABI `wayfarer_sar_reach`, `wayfarer_sar_plan_teams`, `wayfarer_sar_coverage` (the library's `sar` build); `schema/sar-*.json`; `maxEdges` and `minEdgeProbability` trim the probability map | this crate | 0.5 / 0.6 |
| **1e (this branch)** | Host-supplied search effort (`efforts`: POD or seconds seen per edge, grid cell or polygon); drawn segments taken as given; every public type `#[non_exhaustive]` with constructors (section 5) | this crate | 0.5 |
| 2 | Pace on slope: switch default factors to the E4 curve basis | T2 Tobler/E4 pace in OnFoot costing, T1 elevation | 0.8 |
| 3 | Reach and probability per terrain cell (off-network spread, S9) | incident surface / X5a | 0.9 |
| 3b | Hazard rasters and ML layers as zones (S10) | raster reader, layer contract | 0.9 |
| 3c | Segment boundary suggestions (S8 replacement) | linear features, DEM | 0.9 |

### Measured: Tatra scenario (Małopolskie package, 4 vCPU cloud machine, release build)

LKP at the Murowaniec hut (Hala Gąsienicowa), 300 m uncertainty, 3 h elapsed,
one avoid zone (Kościelec ridge) and one weight zone (valley floor), four
named destinations, 24 frontier sectors:

| | Fast (P10) | Typical (P50) | Slow (P90) |
|---|---|---|---|
| Path within reach | 1,241 km | 683 km | 344 km |
| Straight-line radius | 13.7 km | 10.0 km | 7.8 km |

With the built-in pace model the measured ratio to the engine's time is 1.31
(Zakopane's paved streets pull it below a footpath's 1.37), so the person
factors are 0.95 / 1.28 / 1.73. `predict_reach` takes 0.8 s (three 15-contour
isochrones from 9 origins with access legs (2.2b), one of them the re-run with
the measured ratio, a 1 x 28 matrix, 20,890 edges with probability; 75 % of
the probability lies within the slow person's contour, ROW 0.2 %). Without
access legs (`offTrail: false`) it was 1,233 / 680 / 335 km. A team plan with
3 teams (one legged), 4 segments and 2 h territories takes 0.9 s with access
legs (0.5 s without); coverage of one 6.9 km track takes 0.2 s. The JSON
result is about 4.7 MB, most of it the per-edge probabilities; `maxEdges:
2000` cuts it to 1.6 MB (the 2,000 most probable
edges hold 76 % of the mass, the rest is `unlisted`).

Without heights the pace is flat: on mountain paths the model gives 1.29 to
1.51 times the engine's 4.8-5.1 km/h, against the 1.42 E4 measured for today's
flat pace on mountain routes, so it is about right on average and too fast on
steep climbs until format 4.

Nothing in step 1 changes a route, matrix or package byte: the crate only calls
the engine's public queries, so Monaco and Kraków stay byte-identical.

## 4. Limits and honesty

- Network only: a person off trail walks to the paths at the off-road pace
  (2.2b) and is then on them. Until step 3 the area between paths is not
  scored; the reach polygons (1b) draw it. The off-road pace is flat and one
  pace for all ground until terrain (format 4).
- An edge's entry and whole-reach times come from the band buckets and the
  boundary fragments, so they are exact to within one band (default outer
  contour / 12) where no contour cuts the edge.
- The uniform stopping model is a maximum-entropy default, not a fit. Host
  distance rings (ISRID-style statistics by subject category) correct it
  where they exist.
- Frontier routes are cheapest by the profile's cost, the reach is by time;
  they agree on foot except where costs prefer paths over roads.
- `pod` defaults and factors are starting values; QA-8 field trials and team GPX
  residuals calibrate them.
- Team tracks are personal data: the crate keeps only covered edge ids per team.
- Host efforts are trusted as given: the crate does not check a grid against
  the sensor's footprint or the time of day.

## 5. Rust API and stability

`wayfarer-sar` is published on crates.io from 0.5 with a typed Rust API: the
requests and results are Rust structs built with `new` constructors, so a Rust
host never needs the JSON or the C ABI (both stay for other hosts). A host on
`wayfarer-service` runs the steps on its cached engine with
`RoutingService::execute`, which holds the source's lease for the call.

Across 0.x, changes are additive. Every public struct and enum is
`#[non_exhaustive]`: a release may add a field (with a serde default, so older
JSON still reads), an enum variant or an error variant without a breaking
change, and hosts build requests with the constructors and match with a
wildcard arm. Removing or renaming a field, changing its meaning or units, or
changing a default that moves results waits for the next minor release and is
listed in the changelog. The `schema/sar-*.json` schemas follow the same rule.

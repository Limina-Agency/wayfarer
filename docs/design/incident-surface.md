# Incident surface: the contract for the fused per-incident cost surface (v1)

Status: v1, implemented by `wayfarer-terrain` (published from 0.6 as
experimental), with fuse version 1. v0 was the terrain lane's
experiment E14 (`off-road-sar/exp/E14`, branch `exp/terrain-incident`); its
cost rules carry over unchanged, and the crate replays E14's timeline block
for block. v1 adds a fuse version in the recipe, the incident id, an overlay
that is a set of shapes applied by observed time, the AOI cut, the epoch taken
from the host's clock, and a terrain file hashed per block (appendix A). The
profile values stay assumptions until the field trial; they change through the
fuse version, not through this contract.

The incident surface is what stage 2 of the off-road plan produces: the base
terrain fused with the terrain layers and team overlay items active at the
incident time, costed for each travel profile. Every off-road query of an
incident (access legs, travel-time fields, segment boundaries, landing zones,
the probability update) reads it and nothing else, apart from the DEM in the
terrain file. This document says what the surface holds, how it is
identified, how it changes when a layer arrives or time passes, and the cost
and query rules that make two devices with the same inputs get the same routes.

What a layer is and how fusion combines layers is `terrain-layers.md`. This
document starts where its section 9 ("Hand-off to the cost surface") stops,
and pins those rules. The base is `format-4.md`; the cost tables and the search
are the terrain lane's E5 (`off-road-sar/exp/E5`). Like the analyses
format-3.md cites, the lane's notes are kept outside the repository.

## 1. Where it sits

| Stage | Reads | Writes |
|---|---|---|
| 1 Base (compile time) | DEM, land cover, OSM | terrain file: heights, steepness-max, 4-bit land cover and confidence bit, linear-feature mask |
| 2 Fusion (per incident, per epoch) | terrain file, layers, team overlay set, profile tables | **the incident surface** |
| 3 Query (per request) | the incident surface, the terrain file's DEM | routes, fields, their confidence |

Fusion runs on every device with the same code (decided 2026-11-24): there is
no command post that fuses for the others. Devices that hold the same inputs
build the same surface and agree by its recipe hash; a device that is offline
fuses for itself and gets the bytes any other member would have.

Not stored: access legs, road-graph portals, travel-time fields and any
search state. They are per query (PLAN.md section 9).

## 2. Inputs

A surface is a pure function of these inputs, all named in its header:

- the **fuse version**: the version of the fusion and cost rules and the
  default constants (section 13);
- the **incident id**, 16 bytes chosen by the host, or all zeros
  (`NO_INCIDENT`) for a surface that belongs to no incident. It enters the
  recipe hash only, never the content, so it cannot change a route;
- the **terrain** (the terrain file's hash, appendix A, which covers heights,
  steepness, land cover, the confidence bit and the line mask) and the
  **prior** table that turns the base class into a distribution
  (`terrain-layers.md` 8);
- the **layers**, each by content hash: the ones fusion applied, in fusion
  order, and the ones it left out, each with a reason code (1 outside its
  validity window, 2 research-only licence, 3 other spacing, 4 no confidence
  anywhere at T, 5 another layer-contract major, refused when read);
- the **team overlay set**, by its hash (section 5);
- the **profile tables** (section 6.3), embedded in full;
- the **params**: research layers allowed, feature confidence threshold,
  level confidence threshold;
- the **AOI**: a sorted set of 64 × 64-post blocks at the surface's spacing
  (1″ default). The host cuts the terrain to a box of positions in ticks,
  `[lat0, lat1) × [lon0, lon1)`: the AOI is every block that holds a post
  `p` with `lat0 ≤ p · s < lat1` and `lon0 ≤ p · s < lon1`, and the cut
  terrain has its own terrain hash. A block's bytes depend only on its own
  inputs, so a surface on a cut terrain holds the full surface's blocks for
  that area;
- the **incident epoch T** and the epoch length E (section 3);
- the versions of this contract and of the layer contract.

## 3. Incident time and epochs

Fade-outs make fusion depend on T to the second, so two devices that each fuse
"now" would never agree. T is therefore an **epoch start**: `T = floor(now /
E) · E` with E = 900 s by default (part of the recipe). `now` is the host's
clock in UTC seconds: the host passes it (`surface_at(now)`), and the engine
never reads a clock. Every device that fuses in the same quarter hour with the
same inputs gets the same surface. A device that was suspended for hours
builds for the epoch it wakes in; a surface depends on its inputs and T only,
so there is nothing to replay. A build at a T that is not an epoch start is
refused.

A layer that arrives mid-epoch is fused at the same T (a new revision of that
epoch's surface). It is not held back to the next epoch.

The header carries **same_until**: the first epoch start after T at which an
input's activity or effective confidence can change. For each input layer
that could apply (not left out for licence or spacing) the candidates are
`ceil_E(valid_from)`, `ceil_E(valid_until)`, the first epoch after
`fade_from`, and T + E while the layer is fading (`fade_from < T <
valid_until`); same_until is the smallest candidate above T, or "never"
(i64::MAX). For every epoch in `[T, same_until)` the content (section 7) is
the same, so a device keeps its surface until then and re-fuses only the
blocks of the layers that changed (section 9). While a flood model fades, that
is every epoch.

## 4. Contents

Per block of the AOI, in key order:

| Plane | Type | Meaning |
|---|---|---|
| table | list of layer hashes | the grid layers that are some post's source in this block, in fusion order |
| `top` | u8 | the most probable class of the fused distribution (lowest code on a tie) |
| `cert` | u8 | certainty: `round(255 · max P / S)` (`terrain-layers.md` 8) |
| `src` | u8 | 0 base, k = the k-th entry of the block's table (the last grid layer that changed the post), 255 a team overlay item |
| per profile `t` | u16 | time per metre in the profile's unit, 0 = impassable |
| per profile `risk` | u8 | probability (1/255) that the cell is impassable (section 6.2) |
| per profile `why` | u8 | for an impassable cell, `reason << 4 | channel`, else 0 |

Reasons: 1 class mass, 2 steepness, 3 barrier line (channel: river, stream,
railway, fence, cliff), 4 motorway, 5 hazard (channel: hazard slot), 6 level
(channel: level slot), 7 obstruction, 8 washout, 9 team block.

The source index is local to the block, so a block's bytes depend only on the
inputs that touch it. With a global layer index, a late layer that is older
than the others (a satellite pass processed a day late) would renumber the
sources of every block, and a two-block drone update would become a full
rewrite.

The fused state itself (16 probabilities, 8 hazards, 3 levels and the effect
bits per post, about 48 bytes) is not stored: it is reproducible from the
inputs, and the surface holds what queries and confidence need.

## 5. The team overlay

The team overlay is what people on the ground saw: ground they closed, ground
they walked and found clear, a hazard they set. It comes from the host's
overlay entities and goes on top of every layer.

**Items.** An item is an effect, a shape and the time it was observed (UTC
seconds):

| Effect | On each post it covers |
|---|---|
| Block | sets TEAM_BLOCK |
| Clear | clears every hazard and level, and DEBRIS, FALLEN, WASHOUT and TEAM_BLOCK |
| SetHazard(code, p) | sets that hazard's probability to p (1/255); an unknown code does nothing |

Every covered post's source becomes 255 (a team item).

**Shapes.** Vertices are `(lat, lon)` in ticks. A host holds shapes in f64
degrees and brings each vertex onto the lattice with one rule, `ticks =
round(deg · 432,000)`, half away from zero (`ticks_of_deg`); both steps are
exact IEEE operations, so every target gets the same ticks. Latitudes lie
within ±90°, longitudes within ±180°, and shapes do not wrap the
antimeridian. Each shape marks posts by one exact integer rule, so every
device marks the same posts:

- **Box** `[lat0, lat1) × [lon0, lon1)`: the posts `p` with `lat0 ≤ p.lat ·
  s < lat1` and `lon0 ≤ p.lon · s < lon1`. It needs `lat0 < lat1` and
  `lon0 < lon1`.
- **Polygon**: one or more closed rings of at least 3 vertices (the last
  joins the first), filled even-odd. A post is inside when a ray from it
  towards +lon crosses the rings an odd number of times, counting an edge
  `(a, b)` when `min(a.lat, b.lat) ≤ p.lat < max(a.lat, b.lat)` and its
  crossing `x = a.lon + (p.lat − a.lat)(b.lon − a.lon) / (b.lat − a.lat)` is
  at or west of `p.lon` (compared exactly, as products in 128-bit integers).
  A ring inside another is a hole, and a box drawn as a ring marks the box.
- **Line**: a polyline of at least one vertex. It marks the posts whose cell
  `[p − s/2, p + s/2)` a segment touches, by the exact supercover feature
  lines use (`terrain-layers.md`), both neighbours at an exact corner
  crossing. A single vertex marks the post whose cell holds it.

An item holds at most 65,536 vertices. A build refuses an item that breaks
these rules.

**A set, not a list.** Fusion applies the items in canonical order: by
observed time as a number, then by the item's bytes, with duplicates
removed. Where two items cover a post the one observed later wins, whatever
order the items arrived in and from however many devices. Removing an item
means dropping it from the set. The bytes of an item, all little-endian:

| Field | |
|---|---|
| observed_at | i64 |
| effect | tag u8 (1 Block, 2 Clear, 3 SetHazard), hazard code u16, probability u8 (0 for Block and Clear) |
| shape | tag u8 (1 box, 2 polygon, 3 line) |
| box | lat0, lat1, lon0, lon1 (i64) |
| polygon | ring count u32; per ring a vertex count u32 and the vertices |
| line | vertex count u32 and the vertices |

A vertex is lat, lon (i64). The encoding delimits itself. The **overlay
hash** is blake3 of `"wis1/overlay"`, the item count (u64) and the items'
bytes in canonical order.

Items carry no validity window in v1 (section 15).

## 6. Cost rules

### 6.1 One post

Inputs: the fused state (P summing to S = 4096, hazards h, levels with their
confidence, effect bits), the base's steepness class and line mask. Every
step is integer, and `div_round` rounds half away from zero.

1. **Effect bits join the line mask**: an observed barrier line (classes 1-16)
   as a fence, debris and fallen trees as OBSTRUCTION (bit 13), washout as
   WASHOUT (bit 14), observed passage as a path (bit 4), team block as
   TEAM_BLOCK (bit 15).
2. TEAM_BLOCK, then WASHOUT: impassable, risk 255.
3. **Blocking hazards** for the profile are its `haz_block` set, minus a
   hazard whose level is measured on the post: flood when a water depth is
   present, snow cover when a snow depth is present. A measured depth is the
   better evidence, so it decides.
4. **A usable way** (the fastest way kind with a time) sets `t`, as in E5; class,
   steepness and barriers do not apply on it (a bridge crosses the river under
   it). A motorway that the profile cannot use closes the cell.
5. **Off way, class mass**: `Pb` = the probability on classes with no time. If
   `Pb ≥ block_at` the cell is closed. Otherwise `t = div_round(Σ P_k t_k,
   S − Pb)` over the passable classes: the expected time over the passable
   mass. Then E5's steepness and barrier factors: `rel = 1000 · steep / 1000`,
   times each barrier's factor per mille (floor), any zero closes, and `t =
   div_round(t · 1000, rel)`.
6. **Hazards** (on and off ways: a flooded road is flooded): a blocking hazard
   with `h ≥ haz_block_at` closes the cell. Otherwise `t = div_round(t ·
   (255000 + Σ k_s h_s), 255000)` with `k_s` the profile's slowdown per mille at
   probability 1.
7. **Levels**, in slot order, for each level with a confidence: above
   `lev_block` closes the cell; otherwise `t = div_round(t · (1000 + v ·
   lev_k), 1000)`.
8. **OBSTRUCTION**: a profile factor `obst_rel` (0 closes; walker 400 per
   mille), on ways too: fallen trees across a trail slow the trail.
9. `t` is clamped to 1..65535.

With no layers and no overlay the result is E5's surface, cell for cell (E14
checked both profiles on the Tatra base).

### 6.2 Risk

`risk = 255 − round(255 · (1 − Pb/S) · Π (1 − h_s/255))` over the blocking
hazards of step 3 (`Pb` is 0 on a way), in exact integers. A deterministic
closure (steepness, barrier, motorway, level, obstruction, washout, team
block) has risk 255. Risk is reported, not costed: the time per metre stays the
expectation over the passable mass, and a cell is closed only past the
thresholds. It treats hazards as independent, which is a simplification.

### 6.3 Profile tables

A profile is E5's tables (unit, time per metre per class and per way kind,
steepness and barrier factors, pace by grade) plus:

| Field | Walker (E14) | Light 4×4 (E14) |
|---|---|---|
| `block_at` (class mass, in S) | S/2 | S/4 |
| `haz_block_at` (1/255) | 128 | 64 |
| blocking hazards | flood, fire, closure | flood, rubble, windthrow, fire, landslide, closure |
| hazard slowdown at probability 1 (per mille) | flood 500, rubble 1500, windthrow 2000, snow 800, ice 1000, landslide 1500 | snow 1500, ice 2000 |
| level block above | water 50 cm, snow 120 cm | water 30 cm, snow 25 cm |
| level slowdown per unit (per mille) | water 20/cm, snow 8/cm | water 20/cm, snow 30/cm |
| `obst_rel` | 400 | 0 (closed) |

The numbers are E14 assumptions, there to exercise the rules, like E5's
steepness, barrier and 4×4 values. They want a SAR expert and the field trial.
A risk-averse plan (night, a heavier vehicle) is a profile with lower
thresholds, not a different rule.

The tables are embedded in the surface as canonical bytes (`WIP0`, all
integers little-endian, in the order above), and the profile hash is their
blake3. A device can therefore query a received surface without knowing the
profile versions that made it.

## 7. Identity

- **Block hash**: blake3 of the block's canonical bytes: its key (u64), the
  table (count u8, hashes), then every plane decoded, in the order of section 4
  (u8 as bytes, u16 little-endian).
- **Content hash**: blake3 of `"wis1/content"`, the major (u16), the spacing
  (u32), the terrain hash, the profile count (u8) and each profile's hash
  (blake3 of its `WIP0` bytes), the block count (u32), then each block's key
  (u64) and hash in key order. These are everything a route depends on, so the
  same content hash means the same routes and the same confidence.
- **Recipe hash**: blake3 of `"wis1/recipe"`, this contract's major and minor
  and the layer contract's major (u16 each), the header bytes exactly as
  stored (section 8: fuse version, incident id, spacing, epoch, T, same_until,
  terrain, prior, overlay, params, the layers applied and left out, the profile
  tables), the block count (u32) and the AOI keys (u64) in key order. It says
  how the surface was made, for S3 provenance, and it is what devices compare
  to agree that they built the same surface.

Two surfaces can share the content and differ in recipe: the same epoch window
fused at two different T, a surface after a flood model expired and one fused
without that model (E14 checked both), or the same inputs under two incident
ids. Identity is over decoded values, so a different plane coding or
compressor does not change a hash. A received surface is checked by its
hashes, never parsed by the host.

## 8. The file and deltas

**WIS0** (little-endian):

| Field | |
|---|---|
| magic `WIS0`, major u16, minor u16 | |
| header | fuse version u16, incident id (16 bytes), spacing u32, epoch u32, T i64, same_until i64, terrain, prior and overlay hashes, the three params (u8 each), applied layers (varint count; hash, kind u8, observed_at i64, valid_until i64), left-out layers (varint count; hash, reason u8), profiles (u8 count; varint length, `WIP0` bytes) |
| recipe hash, content hash | 32 bytes each |
| block index | varint count; per block key u64, payload length u32, block hash |
| payloads, in key order | table (u8 count, hashes), then each plane as varint length, coding byte, data |

Plane codings: 0 E5's palette-runs (palette of distinct values, then runs of
palette indices), 1 zlib (RFC 1950) of the raw plane. The writer keeps the
shorter, palette-runs on a tie. A reader checks every block hash, the content
hash and the recipe hash, and refuses another major. A reader with another
fuse version still reads the surface and routes on it; it just cannot rebuild
the same bytes.

**WISD**, a delta: magic, versions, the content hash it applies to, the new
header, the new recipe and content hashes, then the changed blocks in key
order, each a key and an op (1 put, with payload length, block hash and
payload; 0 remove). The receiver applies it only to the surface with that
content hash and checks the result's hashes. A delta cannot change the
profile tables. Adding blocks grows the AOI.

## 9. Updates

A block's bytes change only when an input that touches it changes, so an
update re-fuses and re-costs exactly these blocks:

- the grid blocks of a layer that joined or left, and the blocks its feature
  lines cross;
- the blocks of each overlay item that is in only one of the old and the new
  set (by item bytes): the blocks that hold a post it marks;
- after same_until, the blocks of each layer whose activity changed or that is
  fading.

A change of terrain, prior, profiles, params, epoch length or incident id
rebuilds everything. Each post depends only on its own inputs, so the
incremental result equals a full fusion; E14 checked this at every step of its
timeline, and the crate's tests check it for layers, boxes, polygons and lines
added and removed.

**What to send.** A device that has the base fuses for itself in about 0.2 s
on one core (Tatra, 720,896 posts, both profiles, seven layers). Sending it the
layer is cheaper than sending it the result: the E14 flood layers were 166 KB
against a 0.9 MB surface delta, and a fading layer changes the surface every
epoch for days without any new transfer. So devices send each other layers,
overlay items and recipe hashes; a device whose recipe hash differs sees from
the header which layer or item it lacks. A pinned plan's whole surface is the
fallback for a device that cannot fuse. Deltas are optional, for small changes:
a drone survey was a 35 KB delta (3 blocks), a team overlay change 48 KB (4
blocks).

**Memory.** Blocks are costed as soon as they are fused, so fusion state
exists only for the blocks in flight, and layer planes are held at their slot
width. The crate's CI holds the peak heap for the inputs, the build and one
walker route on a synthetic area with a dense land-cover refresh to 64 MB at
451 km², 112 MB at 900 km² and 320 MB at 2,500 km² (measured 45, 89 and
250 MB).

## 10. Queries

The search is E5's and is part of the contract:

- 16 moves; a knight move needs both cells it passes between passable;
- a move of length L (mm, from the lattice and the latitude of its middle)
  costs `round(L · Σ t · unit_us / (n · rel(grade)))` µs over the n cells it
  crosses (the two ends, plus the two passed cells of a knight move), with
  grade the whole percent from the DEM at the two ends, clamped to ±100, and
  rel the profile's pace by grade;
- a route's cost is the optimum and is normative: any correct search (E5's
  A* with its consistent bound, Dijkstra) gives the same number.

**Canonical path.** Among equal-cost routes the reference path is the one
found from the target backwards: the predecessor u with `d(u) + c(u, v) =
d(v)`, smallest cell index first, over exact distances from the source.
Devices that share a route share its geometry; recomputing it gives the same
cost, and the same geometry under this rule.

**AOI edge.** Cells outside the AOI are impassable. A route that touches the
outermost ring of the AOI says so (`at_edge`): a larger AOI might give a
faster route, or reach a target this one cannot.

## 11. Route confidence

Each move's length is shared equally among the cells it crosses, as weights
`4L/n`. A route reports:

- the weighted mean certainty (rounded) and the minimum;
- the per-mille share of its length on cells whose source is a layer, and on
  cells set by a team item;
- the highest risk on it and the per-mille share of its length with any risk;
- the layers it rests on: the applied layers that are the source of any cell
  it crosses, so a commander can see that a leg rests on a two-day-old flood
  model.

## 12. Reproducibility

- Integers only; every division states its rounding.
- Total orders: blocks by key, posts row-major, layers by `(observed_at,
  hash)`, block tables in fusion order, overlay items in set order.
- T is an input, never the wall clock; the host passes its time.
- Positions in f64 degrees reach the lattice by one rule (section 5).
- E14 checked the reference implementation on E5's real Tatra base (176
  blocks, 720,896 posts at 1″) over an eight-step incident timeline: the
  surfaces, deltas, layer files and routes are byte-identical on x86_64
  glibc (release, debug, 1 and 4 threads), x86_64 musl, i686 musl and aarch64
  musl under qemu; incremental equals full fusion at every step; layer input
  order does not matter; same_until holds; expiry equals fusion without the
  layer. A second implementation in plain Python, written from this document,
  verifies all eight files and seven deltas, costs ten sample blocks from E10's
  fused state with the embedded tables to the same bytes, and recomputes two
  routes' cost, path and confidence exactly.
- `wayfarer-terrain` replays E14's timeline: every surface's blocks and every
  delta's block ops are E14's bytes (the headers differ by the v1 fields), and
  its golden table pins content and recipe hashes on x86_64 and i686 in CI.

## 13. Versioning

- **Major** changes the meaning of the bytes: a plane, a header field, a hash,
  the query rules. A reader refuses another major.
- **Fuse version** changes whenever the same inputs would give different
  content: a new fusion or cost rule, a changed rounding or constant, a
  changed default profile. It is in every recipe, so two builds with the same
  fuse version and the same inputs agree whatever their crate versions. The
  field trial's values arrive this way. (Fuse version 0 was E14's rules,
  unversioned; version 1 is the same rules.)
- **Minor** adds optional things a reader may ignore: new header fields at the
  end, new plane codings (a reader that does not know a coding refuses that
  block and says so).
- A new profile field is a major change of the profile bytes (`WIP0` to
  `WIP1`), not of this contract.

## 14. Provenance and privacy

S3 provenance binds the package, the terrain file, the layers and the surface
by their hashes: the recipe hash covers all of them. The attribution strings
of the applied layers are shown with any route that rests on them. Team
overlay items are personal data under the project's rules, so a surface fused
with team items carries them in its sources and is shared like the overlay set
itself.

## 15. Open questions

- **Thresholds and factors** (6.3) are assumptions until the field trial.
- **Epoch length.** 15 minutes keeps re-fusion rare; a fade then steps every
  quarter hour. Coarser fades (an hour) would be a layer-contract change.
- **Dense soft land-cover refreshes** make the surface big: 1.75 MB for the
  Tatra area with a per-post synthetic refresh against 0.64 MB for the base
  alone, mostly the walker's time plane. Rounding times to a 1-4 % ladder saves
  only 7-22 %. Real model output is smoother than the synthetic noise, so this
  is an upper bound; devices that fuse never download it anyway.
- **Risk independence**: hazards are treated as independent within a cell,
  and a route reports its highest cell risk rather than a probability that
  the route is blocked, because neighbouring cells are strongly correlated.
- **AOI growth**: when a route reports `at_edge`, whether the device adds a
  ring of blocks automatically or asks.
- **The DEM** is by reference to the terrain file; a self-contained surface
  for a receiver without the terrain file would embed it (E1's coding).
- **Percentile surfaces** (P10/P90 paces, PLAN.md section 6 item 7) are extra
  profiles in the same file.
- **Overlay items have no validity window** in v1, only the time they were
  observed; an item that expires (a closure until noon) would add one like a
  layer's, as a new item tag.

## Appendix A. The terrain file (sidecar, v1)

Until format 4 fixes the layouts, the base is a file beside the package. No
prebuilt terrain files ship: a host compiles its own from the DEM and land
cover it holds, with `wayfarer-terrain-compiler`, and every host that compiles
the same inputs gets the same file.

| Field | |
|---|---|
| magic `WTR0`, major u16 = 1, minor u16 = 0 | |
| spacing u32 (ticks), block count u32 | |
| sources | varint length (at most 65,535) and UTF-8 text: the inputs, their licences and attribution |
| index | per block key u64, payload length u32, in key order |
| payloads, in key order | the height plane, the steepness plane, the cover plane (`code \| conf << 4`) and the line plane, each a varint length and the plane |
| terrain hash | 32 bytes |

Heights are decimetres (i32, `i32::MIN` where there is none), coded as a
coding byte, then 0 predicted residuals or 1 zlib of those, the shorter kept: a
presence byte (0 all present, 1 a 512-byte presence bitmap follows, 2 none
present), then one zig-zag varint per present post, the residual against the
LOCO-I median edge predictor on (west, south, south-west) along rows from the
south-west. The other planes are coded like a surface's.

The **block hash** is blake3 of `"wtr1/block"`, the key (u64), then the
decoded planes: heights (i32 little-endian), steepness, cover, the confidence
bits (u8 each) and the line mask (u16 little-endian). The **terrain hash** is
blake3 of `"wtr1/terrain"`, the spacing (u32), the block count (u32) and the
block hashes in key order, so a streaming writer needs one pass. The sources
text is not hashed: correcting an attribution changes no route. A cut keeps
the sources. (v0, E14's file, hashed the planes across all blocks.)

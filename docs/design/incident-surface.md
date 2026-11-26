# Incident surface: the contract for the fused per-incident cost surface (draft v0)

Status: draft v0, tested by a reference implementation in the terrain lane's
experiment E14 (`off-road-sar/exp/E14`, branch `exp/terrain-incident`),
nothing in the engine. The off-road plan pins v1 with the terrain tranche
(0.8), together with `terrain-layers.md`.

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

Fusion runs on the command post and on every device with the same code
(decided 2026-11-24), so a device that is offline fuses for itself and gets
the bytes the command post would have sent.

Not stored: access legs, road-graph portals, travel-time fields and any
search state. They are per query (PLAN.md section 9).

## 2. Inputs

A surface is a pure function of these inputs, all named in its header:

- the **terrain** (the terrain file's hash, which covers heights, steepness,
  land cover, the confidence bit and the line mask) and the **prior** table
  that turns the base class into a distribution (`terrain-layers.md` 8);
- the **layers**, each by content hash: the ones fusion applied, in fusion
  order, and the ones it left out, each with a reason code (1 outside its
  validity window, 2 research-only licence, 3 other spacing, 4 no confidence
  anywhere at T, 5 another layer-contract major, refused when read);
- the **team overlay set**, by its hash;
- the **profile tables** (section 5.3), embedded in full;
- the **params**: research layers allowed, feature confidence threshold,
  level confidence threshold;
- the **AOI**: a sorted set of 64 × 64-post blocks at the surface's spacing
  (1″ default);
- the **incident epoch T** and the epoch length E (section 3);
- the versions of this contract and of the layer contract.

## 3. Incident time and epochs

Fade-outs make fusion depend on T to the second, so two devices that each fuse
"now" would never agree. T is therefore an **epoch start**: `T = floor(now /
E) · E` with E = 900 s by default (part of the recipe). Every device that fuses
in the same quarter hour with the same inputs gets the same surface.

A layer that arrives mid-epoch is fused at the same T (a new revision of that
epoch's surface). It is not held back to the next epoch.

The header carries **same_until**: the first epoch start after T at which an
input's activity or effective confidence can change. For each input layer
that could apply (not left out for licence or spacing) the candidates are
`ceil_E(valid_from)`, `ceil_E(valid_until)`, the first epoch after
`fade_from`, and T + E while the layer is fading (`fade_from < T <
valid_until`); same_until is the smallest candidate above T, or "never"
(i64::MAX). For every epoch in `[T, same_until)` the content (section 6) is
the same, so a device keeps its surface until then and re-fuses only the
blocks of the layers that changed (section 8). While a flood model fades, that
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
| per profile `risk` | u8 | probability (1/255) that the cell is impassable (section 5.2) |
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

## 5. Cost rules

### 5.1 One post

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

### 5.2 Risk

`risk = 255 − round(255 · (1 − Pb/S) · Π (1 − h_s/255))` over the blocking
hazards of step 3 (`Pb` is 0 on a way), in exact integers. A deterministic
closure (steepness, barrier, motorway, level, obstruction, washout, team
block) has risk 255. Risk is reported, not costed: the time per metre stays the
expectation over the passable mass, and a cell is closed only past the
thresholds. It treats hazards as independent, which is a simplification.

### 5.3 Profile tables

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

## 6. Identity

- **Block hash**: blake3 of the block's canonical bytes: its key (u64), the
  table (count, hashes), then every plane decoded, in the order of section 4 (u8
  as bytes, u16 little-endian).
- **Content hash**: blake3 of `"wis0/content"`, the major, the spacing, the
  terrain hash, the profile hashes, the block count, then each block's key and
  hash in key order. These are everything a route depends on, so the same
  content hash means the same routes and the same confidence.
- **Recipe hash**: blake3 of `"wis0/recipe"`, the versions, the header (T,
  same_until, terrain, prior, overlay, params, the layers applied and left out,
  the profile tables) and the AOI keys. It says how the surface was made, for
  S3 provenance.

Two surfaces can share the content and differ in recipe: the same epoch window
fused at two different T, or a surface after a flood model expired and one
fused without that model (E14 checked both). Identity is over decoded values,
so a different plane coding or compressor does not change a hash.

## 7. The file and deltas

**WIS0** (little-endian):

| Field | |
|---|---|
| magic `WIS0`, major u16, minor u16 | |
| header | incident id (16 bytes), spacing u32, epoch u32, T i64, same_until i64, terrain, prior and overlay hashes, the three params (u8 each), applied layers (varint count; hash, kind u8, observed_at i64, valid_until i64), left-out layers (varint count; hash, reason u8), profiles (u8 count; varint length, `WIP0` bytes) |
| recipe hash, content hash | 32 bytes each |
| block index | varint count; per block key u64, payload length u32, block hash |
| payloads, in key order | table (u8 count, hashes), then each plane as varint length, coding byte, data |

Plane codings: 0 E5's palette-runs (palette of distinct values, then runs of
palette indices), 1 zlib (RFC 1950) of the raw plane. The writer keeps the
shorter, palette-runs on a tie. A reader checks every block hash, the content
hash and the recipe hash, and refuses another major.

**WISD**, a delta: magic, versions, the content hash it applies to, the new
header, the new recipe and content hashes, then the changed blocks in key
order, each a key and an op (1 put, with payload length, block hash and
payload; 0 remove). The receiver applies it only to the surface with that
content hash and checks the result's hashes. A delta cannot change the
profile tables. Adding blocks grows the AOI.

## 8. Updates

A block's bytes change only when an input that touches it changes, so an
update re-fuses and re-costs exactly these blocks:

- the grid blocks of a layer that joined or left, and the blocks its feature
  lines cross;
- the blocks of every overlay item of the old and the new overlay set, when
  the set changes;
- after same_until, the blocks of each layer whose activity changed or that is
  fading.

Each post depends only on its own inputs, so the incremental result equals a
full fusion; E14 checked this at every step of its timeline.

**What to send.** A device that has the base fuses for itself in about 0.2 s
on one core (Tatra, 720,896 posts, both profiles, seven layers). Sending it the
layer is cheaper than sending it the result: the E14 flood layers were 166 KB
against a 0.9 MB surface delta, and a fading layer changes the surface every
epoch for days without any new transfer. So the command post sends layers and
overlay sets to devices that fuse, plus the recipe and content hashes so the
device can confirm it got the same surface (or see which layer it lacks from
the left-out list). Surface deltas are for receivers that cannot fuse, and for
small changes: a drone survey was a 35 KB delta (3 blocks), a team overlay
change 48 KB (4 blocks).

## 9. Queries

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

## 10. Route confidence

Each move's length is shared equally among the cells it crosses, as weights
`4L/n`. A route reports:

- the weighted mean certainty (rounded) and the minimum;
- the per-mille share of its length on cells whose source is a layer, and on
  cells set by a team item;
- the highest risk on it and the per-mille share of its length with any risk;
- the layers it rests on: the applied layers that are the source of any cell
  it crosses, so a commander can see that a leg rests on a two-day-old flood
  model.

## 11. Reproducibility

- Integers only; every division states its rounding.
- Total orders: blocks by key, posts row-major, layers by `(observed_at,
  hash)`, block tables in fusion order, overlay items in set order.
- T is an input, never the wall clock.
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

## 12. Versioning

- **Major** changes anything that could change a surface or a route for the
  same inputs: a cost rule, a rounding, a plane's meaning, the query rules.
  A reader refuses another major.
- **Minor** adds optional things a reader may ignore: new header fields at the
  end, new plane codings (a reader that does not know a coding refuses that
  block and says so).
- A new profile field is a major change of the profile bytes (`WIP0` to
  `WIP1`), not of this contract.

## 13. Provenance and privacy

S3 provenance binds the package, the terrain file, the layers and the surface
by their hashes: the recipe hash covers all of them. The attribution strings
of the applied layers are shown with any route that rests on them. Team
overlay items are personal data under the project's rules, so a surface fused
with team items carries them in its sources and is shared like the overlay set
itself.

## 14. Open questions

- **Thresholds and factors** (5.3) are assumptions until the field trial.
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
- **Overlay items have no time** in v0; an item that expires (a closure until
  noon) would add a validity window like a layer's.

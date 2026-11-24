# Terrain layers: the contract for model-made terrain data (draft v0)

Status: draft v0, tested by a reference implementation in the terrain lane's
experiment E10 (`off-road-sar/exp/E10`, branch `exp/terrain-layers`), nothing
in the engine. The off-road plan pins v1 with the terrain tranche (0.8).

A terrain layer is newer information about the ground than the compiled
terrain: a flood probability from Sentinel-1, a land-cover refresh from
Sentinel-2, rubble and fallen trees from a drone orthomosaic, a closure drawn
by a host. Models make layers outside the engine (the ML lane trains them on
the RTX rig with Burn). This document says what a layer must look like so that
fusion can combine it with the base terrain deterministically, on the command
post and on every device, with the same bytes out.

How the base is made (DEM, land cover, steepness, linear features) is in
`format-4.md` and the off-road plan (`off-road-sar/PLAN.md`, sections 7 and 9).
The cost surface that reads the fused state is the terrain lane's E5
(`off-road-sar/exp/E5`). Like the analyses format-3.md cites, these are project
notes kept outside the repository.

## 1. Where layers sit

| Stage | Where | Reads | Writes |
|---|---|---|---|
| 1 Base | compile time | DEM, WorldCover, OSM | the terrain file or package sections (format-4.md) |
| 2 Fusion | incident start; again when a layer arrives | base, layers, team overlay items, the incident time T | the fused cell state, then the incident surface (E5) and a fusion record |
| 3 Query | each request | the incident surface | routes, reach, access legs |

Rules that hold throughout:

1. **Everything is on the global lattice** (format-3.md section 9): posts in
   ticks of 1/120″, 64 × 64-post blocks, a post's cell is
   `[p − s/2, p + s/2)` in both axes.
2. **Integer only after the producer.** The one floating-point step is the
   producer's quantiser (section 4.2), and it is exact. Import, fusion and
   costing are integer arithmetic with one stated rounding rule per step.
3. **A layer's identity is the hash of its content**, not of its file bytes
   (section 7). The fusion record and replayable provenance (S3) name layers by
   that hash.
4. **Fusion order is total**: by observation time, then content hash. Any
   input order, thread count or incremental re-fusion gives the same bytes.
5. **The layer file is the artifact.** Inference need not be reproducible
   (GPU kernels are not); the quantised layer is what everyone fuses.

## 2. Kinds

| Kind | Per present post | Fused how | Examples |
|---|---|---|---|
| Class (1) | a probability vector over the 15 land-cover classes plus a KEEP slot, u8, summing to 255 | blended into the cell's class distribution | Sentinel-2 refresh (M1), orthophoto land cover (M3) |
| Hazard (2) | one u8 probability per channel | blended per channel | flood (M2), rubble and windthrow (M4), fire, snow cover |
| Level (3) | one u16 per channel, in the channel's unit | replaced when confident | water depth, snow depth, canopy height |
| Feature (4) | no grid: linear features with a class and a confidence | rasterised into effect bits | fallen trees, washouts, debris lines, observed passages |

Every grid kind also carries a confidence per post (section 5). A sparse
semantic voxel graph is not a kind: it is an import format that projects into
a hazard layer and a feature layer (section 10).

## 3. Grid alignment

- A layer states its spacing `(s_lat, s_lon)` in ticks; both must divide a
  degree (432,000 ticks), as for any lattice grid.
- **v0 fuses only layers at the surface's spacing** (1″, the decided 25 m
  friction cells, or 2″). A layer at another spacing is listed in the fusion
  record as ignored; the importer resamples it first, which makes a new layer
  with its own hash and the original as its parent.
- The footprint is the set of stored blocks and, per block, a presence bitmap.
  A post outside the footprint is untouched by the layer. Inside it, a
  confidence of 0 also leaves the post untouched.
- **Lattice-aligned input is exact.** A GeoTIFF or COG in EPSG:4326 whose pixel
  centres are lattice posts (tie point and scale in whole ticks, as the E1
  reader requires) imports with no arithmetic beyond the quantiser.
- **Anything else is resampled once, at import.** A drone orthomosaic in UTM
  needs a projection, which is floating point. The importer assigns each source
  pixel to the post whose cell holds its centre, averages the probability
  vectors of the pixels in a cell, quantises, and writes a canonical layer. That
  step need not be reproducible across platforms; the canonical layer is, and
  from then on only its hash matters.
- Producers should train and infer on lattice-aligned tiles where they can.
  E11's dataset curation tiles every input onto the lattice, so Sentinel-2 and
  orthophoto models produce lattice-aligned outputs with no resampling at all.

## 4. Values

### 4.1 Classes

Class layers use the Wayfarer land-cover set, the E3 4-bit codes:

| Code | Class | Code | Class |
|---|---|---|---|
| 0 | KEEP in a layer; no data in the fused state | 8 | water |
| 1 | tree | 9 | wetland |
| 2 | shrub | 10 | mangrove |
| 3 | grass | 11 | moss and lichen |
| 4 | crop | 12 | dwarf pine |
| 5 | built-up | 13 | bare rock |
| 6 | bare (sand, quarry) | 14 | scree |
| 7 | snow and ice | 15 | bog, swamp forest |

The KEEP slot is the mass that defers to whatever the cell already is. A model
that only knows "water or not" writes `p` on water and `255 − p` on KEEP, and
fusion keeps the rest of the cell's distribution in proportion. A model with
another class set (Dynamic World's 9 classes, a debris segmenter) is mapped at
import by a fixed table from its classes to these codes, with a row summing to
255 per source class; the table's hash goes into the layer's parents.

### 4.2 The producer's quantiser

From model probabilities (f32, finite, non-negative, not all zero) to u8
summing exactly to 255:

1. `w_k = round_ties_even(f_k × 2^24)` as an integer. An f32 times a power of
   two is exact in f64, and so is the rounding.
2. Largest-remainder rounding of `w` to 255: `a_k = floor(255 w_k / Σw)`; the
   `255 − Σa` leftover units go one each to the largest remainders
   `255 w_k mod Σw`, ties to the lower class code. A zero weight never gets a
   unit.

Every step is exact integer or exactly rounded arithmetic, so a Python, Rust or
Burn implementation gets the same bytes from the same f32 values. The sums are
of integers, so their order doesn't matter, and step 1 is a power-of-two scale,
exact in f32 or f64. The one trap is a rounding function that ties away from
zero: use one that ties to even (Python's `round`, Rust's `round_ties_even`).

### 4.3 Hazard channels (u8, probability in 1/255)

| Code | Channel | Code | Channel |
|---|---|---|---|
| 1 | flood (surface water now) | 5 | snow cover |
| 2 | rubble or debris | 6 | ice |
| 3 | windthrow (fallen trees) | 7 | landslide or mud |
| 4 | fire (active or recent burn) | 8 | closure (a host's no-go area) |

### 4.4 Level channels (u16)

| Code | Channel | Unit |
|---|---|---|
| 1 | water depth | cm |
| 2 | snow depth | cm |
| 3 | canopy height | dm |

### 4.5 Features

Linear features use `BarrierLines`' point encoding (format-4.md section 5):
E7 latitude and longitude, the first point as zig-zag varints and each next one
as zig-zag varint deltas, at least two points. Each feature has a class and a
confidence (u8, 1/255). Classes 1 to 16 are `BarrierLines`' codes; layers add:

| Code | Feature | Effect |
|---|---|---|
| 17 | debris or rubble line (the edge of a pile) | crossing cost; no crossing for vehicles |
| 18 | fallen trees across the way | crossing cost |
| 19 | washout or collapse (a gap in a way) | not crossable; the way on that cell is gone |
| 20 | observed passage (a way through, seen or walked) | followed like a path |

## 5. Confidence

Each present post carries `w`, a u8 in 1/255. The class probabilities say what
the model thinks the cell is; `w` says how far to trust the model at that cell
rather than what the cell already is. Producers lower `w` for haze, cloud
edges, image seams, shadows and inputs far from the training data; they leave
cloud itself out of the footprint. `w = 255` means "replace", `w = 0` means
"no information".

Two rules make `w` comparable between models:

- **Calibration is recorded.** The manifest names the evaluation set's hash and
  the expected calibration error of `w` on it (per mille). An uncalibrated
  model writes 0xFFFF, and a host may choose to cap such a layer's weight.
- **Time fades it.** A layer is active at incident time T when
  `valid_from ≤ T < valid_until`. From `fade_from` on, its weight falls
  linearly: `w_eff = floor(w × (valid_until − T) / (valid_until − fade_from))`.
  A flood observed on Friday can count fully for three days and fade out by
  the next Friday.

## 6. The manifest

| Field | Type | Meaning |
|---|---|---|
| magic | 8 B | `WTLAYER\0` |
| contract major, minor | u16, u16 | this document's version (0.1) |
| kind | u8 | section 2 |
| licence | u8 | 0 open (may ship), 1 research only (section 13) |
| s_lat, s_lon | u32 | spacing in ticks |
| observed_at | i64 | UTC seconds: the time the layer describes (for a mosaic, its median acquisition time). Fusion order |
| valid_from, valid_until, fade_from | i64 | UTC seconds, `valid_from ≤ fade_from ≤ valid_until`, a non-empty window; `fade_from = valid_until` for no fade |
| channels | u16 list | sorted, unique; empty for class and feature layers |
| model id, model version | strings | for example `m2-s1-flood`, `1.0` |
| weights | 32 B | blake3 of the model weights |
| dataset manifest | 32 B | blake3 of the per-model dataset manifest (ML-DATASETS.md) |
| evaluation set, ECE | 32 B, u16 | calibration record (section 5) |
| backend | string | for example `burn-wgpu`, `burn-ndarray`; informational |
| attribution | string | the attribution the inputs require (Copernicus, ESA, GUGiK) |
| inputs | 32 B list | sorted content hashes of the inputs (scenes, tiles) |
| parents | 32 B list | sorted hashes of the layers, voxel graphs or class tables it was derived from |

The footprint is not in the manifest: it is the stored blocks.

## 7. Identity and the file

**Content hash.** blake3 over, in order: the manifest's canonical bytes (the
field order above, little-endian, strings and lists with a varint length); the
block count (u64); for each block in key order its key (u64, format-3.md's
`(i_block + 2^31) << 32 | (j_block + 2^31)`), its 512-byte presence bitmap (64
u64, bit `p` = post `p`, posts row by row from the south-west), and for each
present post in row-major order its confidence then its slot values (u8, or u16
LE for level channels); then the feature count (u64) and the features sorted by
their encoded bytes, duplicates dropped. Class layers have 16 slots in code
order, slot 0 being KEEP.

**File (`WTL0`, v0).** Magic, the manifest bytes, block count (u32), block
keys (u64 each), `n + 1` payload offsets (u64), the payloads, the feature count
(varint) and features, then the content hash (32 B). A block payload is a tag
byte (bit 7: a presence bitmap follows; the low bits are 0) and then one plane
for the confidence and one per slot, each over the present posts only, as a
coding byte and its data:

| Coding | Data |
|---|---|
| 0 | raw values |
| 1 | one value for every present post |
| 3 | runs: varint `length − 1`, then the value |
| 4 | zlib (RFC 1950) of the raw plane: varint length, then the stream |

A writer picks the smallest coding per plane, ties to the lower code. Coding 4
makes the file bytes depend on the compressor, which is allowed: identity is
the decoded content. A reader rejects a file whose class slots don't sum to 255
at a present post, whose blocks or features are not in canonical order, or
whose stored hash differs from the recomputed one.

**Container.** Once the off-road plan settles terrain files (format-4.md
section 3), a layer becomes a container of the package kind holding these
sections in the terrain id range, proposed: 528 `LayerManifest`, 529
`LayerBlocks` (keys and offsets), 530 `LayerPayload`, 531 `LayerFeatures`. The
content hash is unchanged by the move, because it never covered file bytes.

## 8. Fusion

Inputs: the base, the layers, the team overlay items, the incident time T and
the fusion parameters (surface spacing, whether research layers are allowed,
the feature and level confidence thresholds, default 128).

**Which layers apply.** A layer is left out, with the reason in the fusion
record, when T is outside its window, when it is research-only and research
layers are not allowed, when its spacing is not the surface's, or when it has
no weight anywhere at T. The rest are sorted by `(observed_at, content hash)`
and applied oldest first, so a newer layer has the last word. A layer passed
twice counts once.

**Per post, in that order:**

1. **Base.** The cell's distribution `P` (16 u16 summing to `S = 4096`, E5's
   `PSCALE`) comes from the base class and its confidence bit through a prior
   table, one-hot by default. Hazards and levels start at 0, effect bits at 0.
2. **Class layers.** With the layer's `q` (slot 0 = KEEP) and `w = w_eff`:
   `P ← (1 − w) P + w (q + q_KEEP P)`. In integers, per class `k ≥ 0`:
   `raw_k = (255 − w) · 255 · P_k + w · (q_k · S + q_KEEP · P_k)` with `q_0`
   taken as 0, then largest-remainder rounding of `raw` to `S` (as in section
   4.2, ties to the lower code). `Σ raw = 255² S` exactly, so nothing is lost
   before the one rounding. `w = 0` leaves `P` unchanged; `w = 255` with no KEEP
   replaces it by `q`.
3. **Hazard layers.** Per channel, `h ← ((255 − w) h + w q + 127) / 255`
   (integer division).
4. **Level layers.** Per channel, when `w ≥ 128`: the value is replaced and its
   confidence set to `w`.
5. **Features.** Every feature with confidence at or above the threshold sets
   its class's effect bit in every post whose cell a segment touches: an exact
   supercover in integers (E7 × 27 and ticks × 625 share a unit), with both
   neighbours when a segment passes exactly through a cell corner. Effects are
   ORed, so feature order does not matter.
6. **Team overlay items**, last and in the overlay set's own order: they beat
   every raster and feature. v0 effects over a box of posts: block, clear (the
   team walked it: hazards, levels and the debris, fallen-tree, washout and
   block bits are cleared), and set a hazard to a value.

**Out, per post:** `P`, the 8 hazard probabilities, the 3 levels with their
confidence, the effect bits, a certainty `round(255 · max P / S)`, and the
source: 0 base, `k` the k-th applied grid layer, 255 a team item. Each post
depends only on its own inputs, so blocks fuse in parallel and a new layer
re-fuses only the blocks it touches.

**The fusion record** (what S3 provenance stores): T, the base hash, the prior
table's hash, the applied layers in order (hash, kind, observed_at), the left
out layers with reasons, the overlay set's hash, the fused state's hash and the
surface's hash. The same record reproduces the same surface bytes on any
machine.

## 9. Hand-off to the cost surface

E5's `cell_t(profile, dist, steep, lines)` already takes a class distribution
at `PSCALE = 4096`, so the fused `P` goes in unchanged, and with no layers the
surface is exactly E5's. The contract adds four rules for the cost side:

1. **Blocking mass.** A class with no time per metre (water, for a walker) or a
   blocking hazard (flood, closure) blocks the cell when its probability is at
   least the profile's `block_at`, default `S / 2`: the cell is more likely
   than not impassable, which is also what a P50 view of the cell says. Below
   it, the time per metre is the expectation over the passable mass (classes
   renormalised without the blocked ones). E5 today blocks on any mass, which is
   exact for one-hot input but would let a 2 % water probability close a
   meadow. A risk-averse plan (a vehicle, night) sets `block_at` lower.
2. **Hazards** multiply the time per metre by `1 + h · k_channel` per profile
   (rubble slows a walker less than a 4×4), applied after the steepness and
   barrier factors.
3. **Levels** block above a per-profile threshold (wading depth, snow depth)
   and otherwise multiply like a hazard.
4. **Effect bits** join E5's per-post line mask: debris and fallen trees as
   barriers with a crossing cost, washout as not crossable and clearing the
   way-override on that cell, observed passage as a path grade, team block as
   impassable. E5 owns the bit layout.

Route confidence: a route or access leg reports the length-weighted mean and
the minimum of the certainty of the cells it crosses, and the share of its
length on cells whose source is a layer or a team item, so a commander can see
that a leg rests on a two-day-old flood model.

## 10. Voxel graphs

The interchange form of a sparse semantic voxel graph (v0):

- Nodes: id u64 (unique), E7 latitude and longitude, height in decimetres
  (i32), class u8, confidence u8. Classes: 1 ground, 2 rubble, 3 debris, 4
  fallen trunk, 5 water, 6 low vegetation, 7 high vegetation, 8 structure, 9
  drop (a void or a gap).
- Edges: two node ids, kind (1 traversable, 2 blocked), confidence u8.
- Its hash: blake3 over `WTVOXEL\0`, observed_at, nodes sorted by id, edges
  with `a < b` sorted, each field little-endian.

Wayfarer's importer, not the producer, projects a graph into two layers whose
parent is the graph's hash, so the projection is the same everywhere:

- **Hazard layer** (channels flood, rubble, windthrow): the nodes in each
  post's cell give each channel's probability as the confidence-weighted share
  of nodes of its classes (water; rubble and debris; trunks), rounded half up,
  and the post's confidence as `255 · min(n, 8) / 8` for `n` nodes.
- **Feature layer:** each blocked edge becomes a two-point line, a washout if an
  end is a drop, fallen trees if an end is a trunk, a debris line otherwise; each
  traversable edge becomes an observed passage. Confidence is the least of the
  edge's and its ends'.

Connector edges into the road graph (virtual trails, EXP-2b free nodes) are
0.9 work and not in v0.

## 11. Reproducibility

- Integers only after the quantiser; every division states its rounding.
- Total orders everywhere: blocks by key, posts row-major, layers by
  `(observed_at, hash)`, features by bytes, overlay items in set order. No hash
  map iteration reaches an output.
- Time is an explicit input (T), never the wall clock.
- E10 checked the reference implementation on the real Tatra/Podhale base
  (2.36 M posts at 1″) with ten synthetic layers and a 900-node voxel graph:
  layer files, the fused state and a placeholder friction surface are
  byte-identical on x86_64 glibc (release and debug, 1 and 4 threads), x86_64
  musl, i686 musl (32-bit) and aarch64 musl under qemu; fusion results don't
  depend on layer input order; re-fusing only the blocks a new layer touches
  equals a full fusion; a zero-confidence layer is a no-op; an expired layer
  changes nothing. A second implementation in plain Python, written from this
  document, reads the same files, recomputes all ten content hashes and fuses
  eleven sample blocks (45,056 posts, every rule above exercised) to the same
  bytes.

## 12. Versioning

- **Major** changes anything that could change fused results for the same
  inputs (a rule, a rounding, a class code's meaning). A reader refuses a layer
  of another major; the fusion record says so.
- **Minor** adds optional things: new hazard or level channel codes, new
  feature classes, new manifest fields at the end. A reader that does not know a
  channel ignores it and records that it did.
- The class set is part of the major: a new land-cover class is a major bump,
  and its import tables move with it.
- Registries (hazard, level, feature and voxel classes) live in this document;
  codes 128 and up in each are private, for a host's own layers.

## 13. Licences and provenance

- Each layer names its model's weights and dataset manifest. A layer from a
  model trained on non-commercial data (allowed for research models only,
  decided 2026-11-24) is marked research-only, and fusion leaves it out unless
  research layers are explicitly allowed. A shipped build keeps that off.
- Attribution strings travel with the layer and are shown with any result that
  uses it.
- Team tracks and team-drawn items are personal data under the project's rules;
  layers made from them are marked so by their inputs.

## 14. Open questions

- **`block_at` defaults per profile** (section 9): `S / 2` is proposed for
  walkers; vehicles and night plans probably want less. A field-trial question.
- **A base prior that is not one-hot.** E3 measured that its confidence bit
  separates 79 % from 51 % agreement with BDOT10k. A prior table built from
  E3's confusion matrices would let an ML layer outvote the base more where the
  base is weak. Proposed for E13, once a real model layer exists to test it.
- **Layers at a different spacing** fuse after an import resample in v0. Fusing
  a 2″ layer into a 1″ surface directly (each post reads the layer post whose
  cell holds it) is cheap if wanted.
- **Feature effects on the road graph.** A washout or collapsed bridge on a
  road edge should become an overlay item for the road graph too. Whether that
  happens automatically or as a suggestion a team confirms is open.

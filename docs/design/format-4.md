# Package format 4: terrain (draft)

Status: draft layouts, nothing built. Off-road terrain stays out of format 3
(Anton, 2026-11-24): format 3 ships in 0.7 without terrain sections, and
the layouts below, first sketched in format-3.md section 6.1, move here.
Whether this becomes a version bump (`FORMAT_VERSION` 4) or a set of
optional sections added to format 3 after its freeze is still open; the
layouts are the same either way.

How the terrain data is generated (sources, resampling, land-cover fusion)
and how routing uses it (access legs, the incident surface, switching between
road and open ground) is the off-road plan's subject, not this document's:
see the off-road plan (`off-road-sar/PLAN.md`, sections 2 and 6 to 9), the
terrain lane's brief (`off-road-sar/LANE-TERRX.md`) and its experiments
(`off-road-sar/exp/`). Like the analyses format-3.md cites, these are project
notes kept outside the repository.

Newer data that models or hosts make after compile time (flood, rubble or
land-cover rasters, voxel-graph projections) is not stored in these sections.
It arrives as terrain layers on the same lattice and is fused with them per
incident: see `terrain-layers.md`. The fused, costed result per incident
and epoch, which every off-road query reads, is `incident-surface.md`.

## 1. What it builds on from format 3

- **The container.** The tagged section table, the REQUIRED and IDENTITY
  flags, the identity and content hashes and `PackageParams` (format-3.md
  sections 2 to 5.4). A format-3 reader already skips a section it doesn't
  know unless it is REQUIRED.
- **The global lattice.** Posts in ticks of 1/120″ from 0° N, 0° E, in
  64 × 64-post blocks keyed by their signed block indexes (format-3.md
  section 9). Format 3 uses it for the request grid; the terrain grids use
  the same ticks, blocks and keys, so two regions' grids tile exactly.
- **Profile codes.** Terrain rules are keyed by the profile codes of
  `ProfileSlots` (format-3.md section 5.1), never by slot bits.

## 2. Ids

Ids 512 to 639 are for feature content, and format 3 writes none of them
(format-3.md section 6). The terrain sections take these:

| Id | Section | Item | What |
|---|---|---|---|
| 512 | `SegmentElevation` | T1 | heights per shared info, read by `cost_inputs` |
| 513 | `ElevationProfile` | T1 | elevation profiles along shapes |
| 514 | `DemGrid` | T4 | the DEM on the lattice (section 3) |
| 515 | `BarrierLines` | X2 | linear features (section 5) |
| 516 | `LandCover` | X3a | 4-bit land-cover classes on the lattice (section 3) |
| 520 | `SteepnessMax` | T4 | steepest full-resolution slope per DEM post (section 4) |

`SegmentElevation` and `ElevationProfile` get their layouts with T1.

## 3. Containers and grids

The off-road plan stores terrain on the global lattice (format-3.md section
9) and hasn't decided where: in the routing package, or in a separate
terrain file built from DEM and land-cover sources and reused across OSM
refreshes. The ids and layouts below are the same in both, so the choice
changes no id:

- A terrain file, if chosen, uses the package container: the same header,
  section table, flags and hashes, holding `Metadata`, `PackageParams` and
  only sections from this range. Its identity hash covers its own sections
  (the source tile hashes and spacing are in its `PackageParams`), and a
  package that is used with it keeps its own identity. How a container says
  which kind it is (a `PackageParams` key) is settled with that decision.
- In either container these sections are IDENTITY, since they change
  answers, and not REQUIRED: an engine that doesn't know them routes on the
  road graph only, and a request that needs them fails as unsupported.
- Nothing in a terrain section refers to profile slot bits, which only a
  routing package has. Rules are keyed by profile code (format-3.md
  section 5.1), and a reader maps codes to its own profiles.

Grids (`DemGrid` 514, `LandCover` 516, `SteepnessMax` 520) share one
layout:

- Element size 1, `param` = `s_lat << 16 | s_lon` in lattice ticks
  (1/120″, format-3.md section 9.1), so one container can hold the same
  kind of grid at two spacings. The section holds a block count `n` (u64), `n` sorted block
  keys (u64: `(i_block + 2^31) << 32 | (j_block + 2^31)`, so the order of
  the keys is the order of the signed block indexes), `n + 1` u64 byte
  offsets into the payload, and the payload. Format 3's `GridBlocks`
  (138) uses the same key.
- Each block's payload is a tag byte and the block's data, posts row by row
  from the south-west (`i`, then `j`). Bit 7 of the tag says a 512-byte
  presence bitmap (one bit per post, 1 = has data) follows the tag. It is
  written only when some post has no data, and the values of absent posts
  are skipped. The low bits pick the coding:
  - 0: the values uncompressed (i16 for `DemGrid`, two 4-bit values per byte
    with the lower `j` in the low nibble for the others).
  - 1: one value for the whole block.
  - 2 (`DemGrid`): Rice-coded residuals of the MED (LOCO-I) predictor over
    the block's posts, one 4-bit Rice parameter per 64-post row, and an
    escape to the raw value at quotient 24.
  - 3 (4-bit grids): runs along the rows, each a varint
    `(length - 1) << 4 | value`.
  A writer picks the smallest coding per block; the choice is deterministic,
  so rebuilds stay byte-identical. The terrain lane's E1 measured these on
  Małopolskie (GLO-30, whole metres; `off-road-sar/exp/E1`): `DemGrid` at
  1″ is 9.3 MB with Rice against 26.1 MB with the zig-zag varint planned
  before (a varint costs at least a byte a post), 2.9 MB at 2″ and 1.6 MB at
  3″. `LandCover` in runs is 4.3 MB at 1″, a third of packed nibbles.

Grid spacings and values:

| Grid | Spacing | Values |
|---|---|---|
| DEM (`DemGrid`) | 3″ default, 1″ or 6″, ½″ possible | i16 whole metres, absent posts in the block's presence bitmap; Rice-coded MED residuals per block |
| Steepness (`SteepnessMax`) | the DEM's, when coarser than its source | 4-bit classes of the steepest full-resolution slope, in runs (section 4) |
| Land cover (`LandCover`) | 1″ or 2″ (25–50 m) | 4-bit classes (1–11 WorldCover's, 12–15 for OSM-only classes), in runs |
| X4 friction raster | built per request at 1″ | not stored |

DEM sources (SRTM, NASADEM, GLO-30) are resampled onto the posts with
integer arithmetic (the T1 reader's rules); a source with a different
longitude spacing at high latitude (GLO-30) is resampled like any other.

## 4. `SteepnessMax` (520)

`SteepnessMax` (520, T4) is 4 bits per post, at the spacing and on the
blocks of the `DemGrid` it accompanies. Each value is the steepest slope in
the full resolution source within the post's cell (the source pixels whose
centres fall in `[p − s/2, p + s/2)` in both axes), as a class with upper
bounds 2, 5, 8, 11, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60 and 70° (classes
0 to 14), and 15 for anything steeper. Slopes are Horn's on the source's own
grid, with the thresholds compared exactly in integers. Posts the `DemGrid`
has no data for are absent here too. It lets exit legality and the incident
surface catch embankments and cliffs that a coarse DEM averages away (the
off-road plan's section 8): at 3″ on Małopolskie, 3.25 % of cells hold a
source pixel steeper than 30°, but only 0.66 % look that steep from the 3″
grid itself (E1). It is written beside a `DemGrid` coarser than its source
and left out when the spacing equals the source's, where it adds nothing.
About 1.5 MB in runs on Małopolskie at 3″ and 3.0 MB at 2″ (E1).

## 5. `BarrierLines` (515)

`BarrierLines` (515, X2) holds linear features of any kind: streams, rivers
and shorelines, ridges, cliffs and embankments, power lines and cutlines,
fences, walls, hedges, guard rails, railways and fenced roads, and
optionally building outlines. Element size 1, `param` 0. The section holds:

- A 64-byte header: class rule count u32, line count u32, index spacing
  u32 (lattice ticks, 1,440 = 12″ as the request grid), index block count
  u32, then zero.
- Class rules, 8 bytes each, sorted by (class, profile code): class u8,
  zero u8, profile code u16 (0 = any profile not listed), crossing u8
  (0 free, 1 at a cost, 2 not crossable, 3 only from the left of the
  line's direction, for cliffs and drops), along u8 (0 no effect, 1 a
  corridor that is faster to follow), cost u16 (seconds to cross when
  crossing is 1, or the corridor's speed factor in 1/256 when along is 1).
  The rules come with the data, so a class this build doesn't know still
  has rules. A class with no rule for a profile, and none for code 0, is
  crossed freely.
- Line starts: `line count + 1` u64 byte offsets into the line bytes.
- Line bytes, per line: class u8 and flags u8 (bit 0: a closed ring), the
  source OSM way id as a varint (0 for a derived line, such as a ridge from
  the DEM), a point count of at least 2 (varint), then the first point's
  E7 latitude and longitude as zig-zag varints and each next point as
  zig-zag varint deltas.
- A lattice cell index, the request grid's shape (format-3.md section 9.3):
  block keys and bases (16 bytes each), 4,096 + 1 block-relative u32 starts
  per block, then u32 line indexes, sorted, for every line that touches the
  cell.

The class codes are: 1 stream, 2 river or canal, 3 shoreline (lake or
coast), 4 ridge, 5 cliff, 6 embankment or cutting, 7 power line, 8 cutline
or firebreak, 9 fence, 10 wall, 11 hedge, 12 guard rail, 13 railway, 14
fenced road (motorway or motorroad), 15 building outline, and 16 pipeline
above ground. 0 is invalid, 17 to 127 are for later versions, and 128 and
up are private. Like the grids, each of these sections gets its full spec
in the release that first writes it.

## 6. Open questions

- **A version bump or optional format-3 sections.** Under format 3's rules
  these sections are optional and not REQUIRED, so they could be added to
  format-3 packages after the freeze, and every format-3 engine would still
  open them (routing on the road graph only). A version 4 instead makes
  engines that read only format 3 refuse those packages, unless terrain
  ships as a separate terrain file.
- **Terrain in the package or a separate terrain file** (section 3).

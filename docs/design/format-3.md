# Package format 3

Status: being built behind an off-by-default compiler switch (decision F1
of the tranche after 0.4; `PackageFormat::V3`, `wayfarer compile
--package-format 3`, hidden). Done: steps 1 to 3 of section 12 (the tagged
table, both hashes, `PackageParams`, retired reserved sections, implied ids;
`ProfileSlots` and the u16 masks; versioned attribute-class records), draft
revision 3. Written against `main` at `5cb573c` (format 2, tile-ready ids). Format 3 is the one package-format bump left
before 1.0 (roadmap 7.12, release window 0.12 to 0.39) and is frozen at 0.90.
Every 1.x engine must read every package a 1.x compiler writes, so whatever
format 3 leaves out has to arrive in 1.x as an optional section or not at all.

Roadmap items covered: REG-3 (section table, region identity, hashes,
reserved ids), REG-11 (global lattice), REG-13's hook (cell-major
numbering), the format-3 rows of FEAT-P1/P2 (profile slots, wider masks),
FEAT-G4b and W4 (versioned `LegAttrs`), and the width limits found by the
Poland and Germany compiles (item 6 of the compile analysis' summary). Decisions D5 and
D12 of the work plan and the new decisions this design raises are confirmed
as recommended (section 13).

The analyses cited by name (regions, compile, features, profiles, terrain and
performance, November 2026) are project notes kept outside the repository.
Numbers marked *measured* come from a Małopolskie compile of `main` at
`5cb573c` on a 4-core cloud machine for this document (184,954,112 bytes,
1,069,327 nodes, 2,570,724 edges, 1,287,853 shared infos); the rest say what
they are taken from.

## 0. Summary

1. **A tagged section table.** Each entry carries a stable id, a parameter,
   REQUIRED and IDENTITY flags, an element size, offset, length and blake3.
   A reader skips an unknown section unless it is REQUIRED. New optional
   sections (elevation, DEM grid, barrier lines, land cover, portals, cell
   directory, stored potentials) then land in 1.x releases without a format
   bump. The table may sit after the sections, so the compiler can stream
   sections straight into the file.
2. **Two hashes.** The *identity hash* covers the header and the entries
   flagged IDENTITY (not their offsets) and is what overlay sets, results
   and bundle indexes bind to; it keeps the name `datasetHash`. The *content
   hash* is today's root checksum over everything, for downloads and caches.
   Adding an optional section to a package keeps its identity. Any reader can
   compute the identity hash, including over sections it doesn't know.
3. **What else the bump carries** (layout changes and new mandatory
   sections only; content waits for optional sections): a
   profile slot table instead of the six fixed header slots, u16 access and
   restriction masks, `LegAttrs` records that tolerate newer and older
   field lists, a binary `PackageParams` section for everything a query
   reads that lives in the JSON metadata today, a required `RegionInfo`,
   the OSM node id of every node (needed by stable references, API-5), u64
   offsets where u32 runs out at continental scale, a sparse request grid on
   a global lattice, and implied ids. Net size on Małopolskie: **−9.0 MB
   (−4.9 %)** with the numbering of item 4, −4.8 MB without it.
4. **Numbering: node id = CCH rank, with chains eliminated per cell.**
   *Measured* on Małopolskie: ranking each degree ≤ 2 node at the start of
   the dissection subtree that owns it gives **exactly the same CCH arcs
   (3,694,694) and the same elimination tree (height 523)** as today's
   order, so the potentials are unchanged, and every subtree of the
   dissection becomes one contiguous range of ranks. Numbering nodes by that
   rank makes every cell, at every level, a contiguous range in every
   per-node, per-edge and per-rank section, drops `CchRank`, and keeps
   locality (99.83 % of edges join nodes less than 2^15 ids apart; Hilbert
   order today: 99.68 %). The cell size is chosen when a `CellDirectory` is
   written in 1.x, not at the bump. Tile-ready ids stay as the input
   numbering that makes the dissection, and so the final ids, independent of
   build order; a tiled compile forces its tile borders to the top of the
   dissection (section 8).
5. **A global lattice** in ticks of 1/120″ anchored at 0°, 0° for the DEM grid,
   land cover, any raster and the request grid, so two regions' grids tile
   exactly and ties break on global cell indexes (section 9).
6. **Reserved ids** with layouts sketched for portals, node ownership,
   cliques, bounds, component portals, region neighbours, the cell tree, the
   cell directory and stored potentials, and for the feature lane's content
   sections (section 6). `CchCellTree` is written from format 3 on (0.3 MB).
7. **Policy for 1.x:** a 1.x compiler writes no REQUIRED section a 1.0
   engine doesn't know, unless the compile request asks for a feature that
   needs one; such packages say which engine they need. That keeps "every
   1.x engine reads every package a 1.x compiler writes" true by default.

## 1. What format 2 can't do

From `wayfarer-package/src/format.rs`:

- **The table is positional and closed.** `SECTION_COUNT` is 94, the header
  must list exactly that many (`parse_header`, "unexpected section count"),
  and an entry is `offset u64, len u64, blake3 [32]` with no id, element size
  or flags. Every new section is a format bump.
- **The file must end where the table says**, reserved sections must be empty
  or zero (`validate`, `RESERVED_ZERO_SECTIONS`), and `LegAttrs` is decoded
  strictly (`codec::from_slice` rejects trailing bytes), so neither a section
  nor a field can be added.
- **Unknown header layout flags are ignored**, not refused (`parse_header`
  reads only `LAYOUT_NODE_E7_DERIVED`), so a package can't tell an older
  engine that it needs something the engine lacks (bug 18 in
  the features analysis).
- **The dataset hash is the root checksum** over the header and the table, so
  it covers every section: adding a DEM grid to a package would change the
  hash overlay sets are bound to.
- **Six profile slots are fixed in the header** (`PROFILE_SLOTS`,
  `HEADER_PROFILE_COLUMNS`, `HEADER_HEURISTIC`), the per-edge access byte has
  one bit per slot, and slot 0 (`Default`) duplicates `Wheeled`. There is no
  room for a bicycle slot.
- **Things a query reads live in the JSON metadata:** the costing model
  version (`with_runtime` refuses a newer one) and the SAR minimum factors
  (`RouteSarCapability::with_package_data`). The same JSON records build-order
  details such as which ways quality warnings cite first (tile-ready ids PR),
  so it can't be what identifies a package.
- **No OSM node ids.** The road graph has `source_node_id` per node, but no
  section stores it: `NodeIdDelta` holds the road graph's own node ids, which
  since tile-ready ids are `0..n`. A stable reference (`way, from node, to
  node`, roadmap 5.4) can't be resolved from a package today.
- **Twelve reserved sections** carry format 1's landmark slots: six of them
  hold zeros sized by node count (1.2 MB on Małopolskie, *measured*).
- **Ids that are positions are stored anyway.** Since tile-ready ids, `EdgeId`
  is `0..edge_count` and `NodeIdDelta` is `0..node_count` (*measured*:
  identical to the index on Małopolskie): 14.6 MB, 7.9 % of the package.

## 2. File layout

### 2.1 Header (256 bytes, little endian)

| Offset | Size | Field | Format 2 |
|---|---|---|---|
| 0 | 8 | magic `MRRPKG\0\x01` | same |
| 8 | 4 | format version, `3` | same place, so a format-2 engine refuses with `TooNew` before reading anything else |
| 12 | 4 | table entry count | was the fixed section count |
| 16 | 8 | table offset (64-aligned) | new; format 2's table always followed the header |
| 24 | 48 | node, edge, shared info, simple restriction, complex restriction and column counts, u64 each | same counts, moved by 8 bytes |
| 72 | 8 | layout flags: bit 0 node E7 positions derived from `NodeXyzFixed`; every other bit must be zero | an unknown bit now refuses the package (`TooNew`) instead of being ignored |
| 80 | 8 | edge id base | same |
| 88 | 136 | zero | the profile columns, heuristics and format 1's landmark count move out |
| 224 | 32 | root checksum (the content hash, section 4) | was at 152 |

The profile columns and heuristic costs move to `ProfileSlots` (section 5.1). A
reader checks that the zero bytes are zero, so equal packages have equal
bytes.

### 2.2 Section table

`entry count × 64` bytes at the table offset:

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | `id`: a stable section id (section 6) |
| 4 | 4 | `param`: what the section's spec says (a column index for per-column sections, the record revision for `AttrClassBytes`); zero otherwise |
| 8 | 4 | `flags`: bit 0 REQUIRED, bit 1 IDENTITY; other bits zero |
| 12 | 4 | `elem`: element size in bytes (1 for byte pools and JSON) |
| 16 | 8 | offset (64-aligned) |
| 24 | 8 | length (a multiple of `elem`) |
| 32 | 32 | blake3 of the section's bytes |

Rules a reader checks at open:

- Entries are sorted by `(id, param)`, strictly; no duplicates. At most
  4,096 entries (a hostile table can't make open allocate much).
- Sections and the table don't overlap each other or the header, start on
  64-byte boundaries, and lie inside the file. Between them there is only
  zero padding to the next 64-byte boundary, and the file ends at the last
  one's end rounded up to 64. Sections may be in any order in the file; the
  table may follow them.
- For an id the reader knows: `elem` and `flags` equal the reader's
  registry entry for that id (a package can't drop IDENTITY from a graph
  section to make two different graphs share an identity hash), `param` is
  valid for that id, and the length passes the structural checks it passes
  today.
- For an id it doesn't know: REQUIRED refuses the package with `TooNew`
  naming the id ("this package needs section 517, which this build doesn't
  read; update this app"). Otherwise the section is skipped; `wayfarer info`
  lists it under `unknownSections` with its id, param and length.
- Each known id has a presence rule: *mandatory* (every format-3 package
  has it), *conditional* (present exactly when a count or flag says so, as
  `NodeLatE7` today) or *optional* (may be absent). A missing mandatory
  section, or one present against its rule, is invalid.

**Why the table may come last.** Today's writer either keeps every section in
memory until the end or spills them to a scratch file and copies them into
place (`PackageWriter::spilling`), because the table at offset 256 needs
every length first. With a table offset in the header, the compiler can
write each section into the output file as soon as it is built and append the
table, which saves one full copy of the package (2 GB for Poland) and the
spill file (item 6 of the compile analysis' ranked improvements). The order in the
file must still be a deterministic function of the input, so the compiler
writes sections in a fixed program order, never in the order parallel work
finishes. A reader maps the file anyway, so a table at the end costs it
nothing.

### 2.3 Open and verify

`open` reads the header and the table, checks the root checksum, the
entries, and each known section's structure as today; it hashes no section.
`verify` also hashes every section, known or not, and checks the padding.
Lazy, per-cell validation (REG-13, 1.x) changes `open`, not the format.

## 3. Flags: what REQUIRED and IDENTITY mean

**REQUIRED (must understand).** Set when an engine that ignores the section
would misread another section or answer a request wrongly. Examples: the
profile slot table (it says what the access bits mean), `PackageParams`,
`RegionInfo`. Not set when an engine that ignores the section answers every
request exactly as it would for a package compiled without that section's
feature. Examples: a DEM grid (an engine without terrain-aware legs never
reads it), stored potentials (derived data), barrier lines, conditional turn
restrictions (an engine that doesn't know them ignores `restriction:conditional`
anyway, as every engine does today), `NodeOwner` (a bundle member opened on
its own routes over its halo, which holds real roads compiled with the same
rules). A section that changes how a *known* section is read is always
REQUIRED; if a later feature needs a new encoding of an existing section,
it is a new REQUIRED id, not a new meaning of an old one.

**IDENTITY.** Set on every section that defines the graph, its ids, its
costs and what a request that uses no optional feature returns: nodes, edges,
shared infos, shapes, costs, restrictions, the request (snapping) sections,
the attribute classes, `ProfileSlots`, `PackageParams`, `RegionInfo`,
`NodeSourceId`. Not set on the JSON metadata, on derived data (cell tree,
cell directory, stored potentials, in-edge index) and on content sections
that only feed features a request asks for (elevation, DEM, barriers, land
cover). The rule: two packages with the same identity hash give every id the
same meaning and every plain request the same answer; a request that uses an
optional feature may differ, so result caches key on the content hash.

Derived sections must be bit-identical to what an engine would compute from
the identity sections (a stored metric equals the customized one). That is
what lets them stay out of the identity hash. An engine that finds one wrong
(lazily, when it touches it) reports `package_invalid`.

Every mandatory format-3 section is flagged REQUIRED and, where the list
above says so, IDENTITY. Every format-3 reader knows them, so the REQUIRED
flag changes nothing for them, but it keeps the flags truthful for tools. If
`CchRank` stays (section 8.3), it is IDENTITY: the node order decides which of two
equal-cost paths a search returns.

**One id, one layout.** An id means one layout for as long as format 3 is
read. When the bump changes a format-2 section's layout, the section gets a
new id and the old id is retired (never reused).

**Experimental ids** (`0x8000_0000` and up) are for sections behind
`experimental-*` features. They are never REQUIRED and a release compiler
never writes them by default.

## 4. The two hashes

- **Content hash** = today's root checksum: blake3 over the 256-byte header
  with the checksum field zeroed, then the table. It covers every section
  through the entries' blake3 and every offset. Used for downloads, `verify`,
  caches of results, and to tell two files apart.
- **Identity hash** = blake3 over
  `"wayfarer.package.identity.3"` (a domain tag), then the header with the
  table entry count, the table offset and the root checksum zeroed (the
  entry count changes when a non-identity section is added), then, for
  every entry flagged
  IDENTITY in table order, its `id, param, flags, elem, length, blake3`
  (56 bytes, no offset). Offsets are left out so that adding, removing or
  moving a non-identity section (an install step that appends stored
  potentials on a phone, a tool that strips the DEM) keeps the identity.
  Unknown IDENTITY sections are included, so every reader computes the same
  value.

Neither is stored; both come from the header and the table at open, which
open reads anyway.

Names on the wire and in the C ABI: `datasetHash` becomes the identity hash
(it keeps its role: what overlay sets bind to, the `revision` the C ABI
reports), and `contentHash` is added. For a format-2 package both are the
root checksum, so nothing that binds to today's packages breaks. `verify`
and `info` print both.

## 5. Sections the bump adds or changes

### 5.1 `ProfileSlots` (id 128, mandatory, REQUIRED, IDENTITY)

One 32-byte entry per profile slot, at most 16; the entry's index is the
slot's bit in every access and restriction mask.

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | profile code: 1 wheeled, 2 tracked, 3 emergency, 4 on foot, 5 legged; reserved 6 bicycle, 7 HGV, 8 motorcycle, 9 bus; `0x8000` and up private |
| 2 | 2 | flags, zero |
| 4 | 4 | stored column, or `u32::MAX` when the engine derives the profile's costs (as `NO_COLUMN` today) |
| 8 | 8 | heuristic cost per metre (f64), as `HEADER_HEURISTIC` today |
| 16 | 16 | zero |

`Default` is no longer a slot: it is the API's alias of `wheeled` (it is
"routed and costed exactly as `Wheeled`", and Małopolskie stores both on
column 0, *measured*). A request for a profile the package has no slot for is
refused with a typed error. An engine keeps a slot whose profile code it
doesn't know and never matches it, so a 1.x compiler can add a bicycle, HGV
or motorcycle slot and older engines still read the package, as long as the
masks have a free bit: hence u16, with 5 of its 16 bits used today.

As built: `EdgeAccess` also carries format 2's two SAR bits (physically
drivable, against a vehicle one-way), in its top two bits (14 and 15), so a
package has at most 14 slots. The other masks could hold 16, but one slot
count numbers them all. A compile writes the five slots in code order
(wheeled, tracked, emergency, on foot, legged: bits 0 to 4), and the
compiler, which builds every mask in format 2's bits, moves each down one
and drops `Default`'s, which always equals `Wheeled`'s.

### 5.2 u16 masks (new ids, old ids retired)

| Section | Format 2 | Format 3 | Małopolskie cost (*measured* counts) |
|---|---|---|---|
| `EdgeAccess` (was `EdgeProfileAccess`) | u8 per edge | u16 per edge | +2,570,724 B |
| `NodeReachSaturated` | u8 per node | u16 per node | +1,069,327 B |
| `SimpleMask`, `ComplexMask` | u8 per restriction | u16 | +3,454 B |
| `BarrierAccess` | u16: closed u8, gate u8 | u32: closed u16, gate u16 | +37,412 B |

Total +3.7 MB (+2.0 %); about +39 MB on Poland at the same per-edge rate.
This is the feature lane's recommendation (the profiles analysis,
P1); a u8 with bicycle in the freed `Default` bit would cost nothing but
leave no room for HGV, motorcycle or bus access without a format 4.

### 5.3 Versioned `LegAttrs` (id 57 `AttrClassBytes`, `param` = revision)

Each attribute class's record is the correlation part (varint length, then
its bytes) and the leg part (`LegAttrs` in `crate::codec`). Format 3 keeps
that encoding and changes the decode rules:

- The entry's `param` is the `LegAttrs` revision the compiler wrote. Format
  3 starts at revision 1: today's field list, frozen in the format spec as a
  numbered table (field index, name, type), not as "whatever the Rust struct
  is".
- Fields are only ever appended, each with a default (`None`, `false`, empty).
  A reader of revision r decodes its fields from a newer record and ignores
  the bytes after them (each record's end is known from `AttrClassStart`); a
  reader of a newer revision fills the fields an older record lacks with
  their defaults. A record shorter than its revision's field list is invalid,
  so corruption is still caught.
- Enum fields (`road_type`, `directional_access`, `hierarchy_tier`) decode an
  unknown code to a documented fallback (`RoadType::Other`, or the most
  general value of the enum), not an error.
- The correlation part follows the same rule (append only, trailing bytes
  ignored).
- A field that changes the meaning of an existing one can't be appended; it
  needs a REQUIRED section (section 3).

With this, `maxlength`, `hazmat`, cycleway infrastructure, `dismount`
(FEAT-P1, P2) and anything else per attribute class land in 1.x without a
bump, and don't have to be ready for the format-3 release.

As built (draft revision 3, `LEG_ATTRS_REVISION` = 1):

- The `AttrClassBytes` entry's `param` is the revision; 0 is invalid in
  format 3, and the section appears once whatever its revision. A newer
  revision than the reader's is not `TooNew`: it is read as above. A
  format-2 package has no `param` and holds exactly revision 1's fields, and
  its records must still end after them.
- The leg part is encoded as `crate::codec` (bincode 1's varint format)
  writes a struct: the fields in index order, with no framing. An enum is its
  variant index as a varint, `Option` a byte 0 or 1 then the value, a `Vec` a
  varint count then the elements. String fields hold an id in the request
  string table. `request::decode_leg_attrs` reads any revision, and
  `request::LEG_ATTRS_FIELDS` counts each revision's fields; a test fails
  when a field is added without a new revision.
- Fallbacks for unknown enum codes: `road_type` reads `other`,
  `directional_access` reads `both`, and `hierarchy_tier` reads no tier.
- Revision 1's leg fields:

| Index | Field | Type |
|---|---|---|
| 0 | `road_type` | `RoadType` |
| 1 | `directional_access` | `DirectionalAccess` |
| 2 | `highway` | `Option<u32>` |
| 3 | `name` | `Option<u32>` |
| 4 | `ref_name` | `Option<u32>` |
| 5 | `route_refs` | `Vec<u32>` |
| 6 | `access` | `Option<u32>` |
| 7 | `service` | `Option<u32>` |
| 8 | `surface` | `Option<u32>` |
| 9 | `tracktype` | `Option<u32>` |
| 10 | `smoothness` | `Option<u32>` |
| 11 | `maxspeed_kph` | `Option<f32>` |
| 12 | `maxspeed_forward_kph` | `Option<f32>` |
| 13 | `maxspeed_backward_kph` | `Option<f32>` |
| 14 | `hgv_maxspeed_kph` | `Option<f32>` |
| 15 | `max_height_cm` | `Option<u16>` |
| 16 | `max_width_cm` | `Option<u16>` |
| 17 | `physical_width_cm` | `Option<u16>` |
| 18 | `max_weight_kg` | `Option<u32>` |
| 19 | `max_axle_load_kg` | `Option<u32>` |
| 20 | `incline_percent` | `Option<i8>` |
| 21 | `is_4wd_only` | `bool` |
| 22 | `is_motorroad` | `bool` |
| 23 | `is_lit` | `bool` |
| 24 | `sac_scale` | `Option<u32>` |
| 25 | `mtb_scale` | `Option<u32>` |
| 26 | `trail_visibility` | `Option<u32>` |
| 27 | `sidewalk` | `Option<u32>` |
| 28 | `crossing` | `Option<u32>` |
| 29 | `lane_count` | `Option<u8>` |
| 30 | `lanes` | `Option<u32>` |
| 31 | `turn_lanes` | `Option<u32>` |
| 32 | `turn_lanes_forward` | `Option<u32>` |
| 33 | `turn_lanes_backward` | `Option<u32>` |
| 34 | `lane_connectivity_available` | `bool` |
| 35 | `speed_source` | `Option<u32>` |
| 36 | `free_flow_speed_kph` | `Option<f32>` |
| 37 | `constrained_speed_kph` | `Option<f32>` |
| 38 | `admin_country_id` | `Option<u32>` |
| 39 | `admin_region_id` | `Option<u32>` |
| 40 | `timezone_id` | `Option<u32>` |
| 41 | `conditional_restrictions` | `Vec<u32>` |
| 42 | `is_link` | `bool` |
| 43 | `is_roundabout` | `bool` |
| 44 | `is_bridge` | `bool` |
| 45 | `is_tunnel` | `bool` |
| 46 | `has_ford` | `bool` |
| 47 | `is_service_like` | `bool` |
| 48 | `is_destination_only` | `bool` |
| 49 | `osm_forward_mode_mask` | `u32` |
| 50 | `osm_reverse_mode_mask` | `u32` |
| 51 | `hierarchy_tier` | `Option<RoadHierarchyTier>` |
| 52 | `layer` | `Option<i16>` |
| 53 | `level` | `Option<u32>` |
| 54 | `destination` | `Option<u32>` |
| 55 | `destination_ref` | `Option<u32>` |
| 56 | `destination_street` | `Option<u32>` |
| 57 | `junction_ref` | `Option<u32>` |
| 58 | `is_toll` | `bool` |
| 59 | `is_ferry` | `bool` |
| 60 | `has_gate` | `bool` |
| 61 | `is_access_restricted` | `bool` |

- Revision 1's correlation part: `road_type` (u8 code; unknown reads
  `other`), flags (u8: service-like, destination-only, gate, access
  restricted, ferry; unknown bits ignored), `layer` (varint, 0 for none,
  else the zigzagged value plus 1), `level`, `name` and `ref_name` (varint
  string id plus 1, 0 for none), and `route_refs` (varint count, then varint
  string ids). Readers already ignore bytes after these fields.


### 5.4 `PackageParams` (id 129, mandatory, REQUIRED, IDENTITY)

Scalars a query depends on, as sorted 16-byte entries `key u32, flags u32
(bit 0 must understand), value u64`:

| Key | Value | Must understand |
|---|---|---|
| 1 | costing model version | yes |
| 2, 3, 4 | SAR minimum factors: legal access, wrong way, vehicle on paths (f64 bits) | yes |
| 5 | mode mask union | no |
| 6 | default component id (from `RequestMeta`) | yes |
| 7 | reach cap that `NodeReachSaturated` answers for (from `RequestMeta`) | yes |
| 8 | package time zone, as a string id (W0, COMP 2's `--time-zone`) | yes, once W0 reads it |
| 9 | ISO country code, as a string id | no |
| 10 | draft revision (only while format 3 is being built on `main`, section 12) | yes |

An unknown key with bit 0 set refuses the package (`TooNew`); others are
skipped. `RequestMeta` (id 47) is retired: its node and shared-info id bases
are implied (section 5.7). The JSON `Metadata` (id 0) stays mandatory but is
informational only: graph id, road graph hash, quality, provenance (clip
polygon hash, DEM tiles, inputs). No query reads it and it is not IDENTITY,
so a build-order detail in it can't change the identity.

### 5.5 `RegionInfo` (id 256, mandatory, REQUIRED, IDENTITY)

96 bytes, written by every compile; a stand-alone package is a bundle of
one.

| Offset | Size | Field | Stand-alone value |
|---|---|---|---|
| 0 | 16 | region id (assigned by the bundle) | zero |
| 16 | 16 | bundle id | zero |
| 32 | 16 | extent of the owned nodes: min lat, min lng, max lat, max lng (E7, i32) | the nodes' bounding box |
| 48 | 32 | boundary hash: FEAT-D2's canonical polygon hash of the region's boundary | the clip polygon's hash, or zero |
| 80 | 4 | halo width in metres | zero |
| 84 | 4 | cut rule: 0 none, 1 polygon, 2 banded min cut, 3 dissection cell | 0 |
| 88 | 4 | cut parameter (band width in metres, or cell level) | zero |
| 92 | 4 | flags: bit 0 the package holds foreign-owned stub nodes (`NodeOwner` present) | zero |

A reader keeps an unknown cut rule (only region code acts on it, and it
refuses what it doesn't know). Neighbours and the identity hash each portal
table was matched against go in `RegionNeighbours` (section 6), not here, so a
bundle can record a re-match without changing a region's identity.

### 5.6 `NodeSourceId` (ids 140, 141, mandatory, REQUIRED, IDENTITY)

The OSM node id of every node, as `u32` low bits (id 140) and `u16` high
bits (id 141), so 6 bytes per node and ids up to 2^48; `0xFFFF` in the high
part marks a node with no OSM node (a crossing the compiler inserted while
repairing unnoded crossings, 3,062 on Małopolskie in the regions analysis,
section 1.3, or a plaza centre). +6.4 MB
(3.5 %) on Małopolskie. This is what lets the engine resolve a stable
reference `(way, from node, to node, direction, fraction)` (API-5) and what
`PortalRef` names portals by. A node without an OSM id is referenced through
the way: the OSM way, the real node before it along the way, and a
fraction. Plaza ways already have stable synthetic ids
(`SYNTHETIC_PLAZA_WAY_ID_BASE - way id` since tile-ready ids), so a negative
way id in a reference is a plaza of that OSM way. The engine builds the
reverse lookup (OSM id to node) on first use, as it does for edge ids
today.

Stable references are a 1.0 contract (API-5, P0), so this can't be an
optional section added later: a 1.0 package without it couldn't be resolved
by any 1.x engine.

### 5.7 Implied ids and retired sections

- Edge `i` has id `edge id base + i`, node `i` has id `i`, shared info `i`
  has id `i + 1`, as every package has had since tile-ready ids. `EdgeId`
  (6), `NodeIdDelta` (51) and `NodeIdWide` (52) are retired: −14.6 MB
  (−7.9 %, *measured*). The edge id lookup the engine builds on first use
  (`edge_id_lookup`, 4 B per id slot) is no longer needed.
- `Reserved1` to `Reserved12` (28–31, 70, 71, 80–85) are retired: −1.2 MB.
- `CchRank` (93) is retired if nodes are numbered by rank (section 8): −4.3 MB
  (−2.3 %).

### 5.8 Widths (section 10)

`ShapeStart` (34), `AttrClassStart` (56) and `StringStart` (58) become u64
(new ids 135, 136, 137): +0.6 MB on Małopolskie. The dense request grid
(`RequestGrid` 48, `GridCellStart` 49) is replaced by a sparse grid on the
global lattice (ids 138 and 139, section 9.3); `GridEntries` (50) keeps its layout. `SharedInfoCostTuple` becomes u32 under a new id (+2.6 MB on
Małopolskie; section 13).

### 5.9 Net size

On Małopolskie (*measured* section sizes, 184,954,112 bytes today):

| Change | Bytes |
|---|---|
| Implied ids (`EdgeId`, `NodeIdDelta`) | −14,560,204 |
| Retired reserved sections | −1,203,048 |
| `CchRank` retired (numbering by rank) | −4,277,308 |
| u16 masks, u32 barrier access | +3,680,917 |
| u64 offsets | +596,340 |
| `NodeSourceId` | +6,415,962 |
| `CchCellTree` (20 B × 15,748 tree nodes) | +314,960 |
| `ProfileSlots`, `PackageParams`, `RegionInfo`, table growth | under 10,000 |
| **Total** | **−9.0 MB (−4.9 %)**; −4.8 MB (−2.6 %) if `CchRank` stays |

The sparse request grid isn't counted (Małopolskie's bounding box is mostly
road-covered land, so it changes little there; it matters for continents).

## 6. Section id registry

| Ids | What | Presence |
|---|---|---|
| 0–93 | Format 2's slot numbers, for the sections that survive unchanged: `Metadata` 0, the graph, costs, restrictions, shapes, costing tuples, request sections, attribute classes, strings, junctions, barriers, `NodeXyzFixed`, piece bits, twins, lone in-edges | as today's checks |
| retired | 6, 22, 24, 27, 28–31, 34, 47–49, 51, 52, 56, 58, 66, 70, 71, 79–85, and 93 if numbering by rank | never reused |
| 128–191 | Layout changes of the bump: 128 `ProfileSlots`, 129 `PackageParams`, 130 `EdgeAccess` (u16), 131 `SimpleMask` (u16), 132 `ComplexMask` (u16), 133 `BarrierAccess` (u32), 134 `NodeReachSaturated` (u16), 135 `ShapeStart` (u64), 136 `AttrClassStart` (u64), 137 `StringStart` (u64), 138 `GridBlocks`, 139 `GridCellStart` (block-relative u32), 140/141 `NodeSourceId` | mandatory |
| 256–319 | Regions: 256 `RegionInfo` (mandatory); reserved: 257 `RegionNeighbours`, 258 `NodeOwner`, 259 `PortalEdge`, 260 `PortalRef`, 261 `PortalClique` (param = column), 262 `PortalBound` (param = column), 263 `ComponentPortal` | optional |
| 320–383 | Cells and stored potentials: 320 `CchCellTree` (written from format 3 on); reserved: 321 `CellDirectory`, 322 `CchFirstUp`, 323 `CchUpHead`, 324 `CchMetric` (param = column), 325 `InEdgeIndex` (SC-4) | optional |
| 512–639 | Feature content: reserved 512 `SegmentElevation` (T1), 513 `ElevationProfile` (T1), 514 `DemGrid` (T4), 515 `BarrierLines` (X2, linear features, section 6.1), 516 `LandCover` (X3a), 517 `DirectionalDestinations` (G4b), 518 `ConditionalTurnRestrictions` (W4), 519 `SharedInfoLimits` (P2), 520 `SteepnessMax` (T4, section 6.1) | optional |
| `0x8000_0000`+ | Experimental | never REQUIRED |

Reserved layouts, so that the format-3 spec fixes their shape before anyone
writes them (from sections 4.5 and 6.2 of the regions analysis; each gets its full spec
in the release that first writes it):

| Section | Element | Layout |
|---|---|---|
| `RegionNeighbours` | 64 B | neighbour region id [16], the neighbour identity hash the portals were matched against [32], portal count u32, unmatched portal count u32, zero [8] |
| `NodeOwner` | u64 | `[node / 64]` bits set at stub nodes, whose owner is a neighbour |
| `PortalEdge` | 8 B | local edge u32, neighbour index u16 (into `RegionNeighbours`), flags u16 (bit 0 entering) |
| `PortalRef` | 24 B | way id i64, from OSM node id i64, to OSM node id i64, in `PortalEdge` order |
| `PortalClique` | f32 | per column: entering × leaving portal distances, row-major, `f32::INFINITY` for none, rounded down like CCH weights |
| `PortalBound` | f32 | per column: lower bounds between portal nodes (the CCH node metric), same shape |
| `ComponentPortal` | 8 B | (component id u32, portal index u32), sorted |
| `CchCellTree` | 20 B | per dissection tree node in post-order: parent u32 (`u32::MAX` for a root), first rank u32, end of its own chain nodes u32, start of its separator u32, end rank u32 (exclusive) |
| `CellDirectory` | 64 B | per cell: tree node u32, level u16, flags u16, node, edge, shared-info and shape-byte ranges (start u64 each, the ends are the next cell's starts) [32], blake3 of the cell's bytes truncated to 16 bytes [16], zero [8] |
| `CchFirstUp` / `CchUpHead` | u64 / u32 | per rank: first up-arc (n + 1 entries); per arc: the higher end's rank |
| `CchMetric` | 8 B | per arc: up and down weights (f32 each) of one stored column |

`CchFirstUp` is u64 because arc counts reach 2^31 between Europe and the
planet (section 10); the engine's in-memory `UP_BIT` packing is its own business.

### 6.1 Terrain and linear features (512–639)

The off-road plan (`off-road-sar/PLAN.md`, sections 6 to 9) stores terrain
on the global lattice (section 9) and hasn't decided where: in the routing
package, or in a separate terrain file built from DEM and land-cover sources
and reused across OSM refreshes. The ids and layouts below are the same in
both, so the choice changes no id:

- A terrain file, if chosen, is a format-3 container: the same header,
  section table, flags and hashes, holding `Metadata`, `PackageParams` and
  only sections from this range. Its identity hash covers its own sections
  (the source tile hashes and spacing are in its `PackageParams`), and a
  package that is used with it keeps its own identity. How a container says
  which kind it is (a `PackageParams` key) is settled with that decision.
- In either container these sections are IDENTITY, since they change
  answers, and not REQUIRED: an engine that doesn't know them routes on the
  road graph only, and a request that needs them fails as unsupported.
- Nothing in a terrain section refers to profile slot bits, which only a
  routing package has. Rules are keyed by profile code (section 5.1), and a
  reader maps codes to its own profiles.
- Grids (`DemGrid` 514, `LandCover` 516, `SteepnessMax` 520) share one
  layout. Element size 1, `param` = `s_lat << 16 | s_lon` in lattice ticks
  (1/120″, section 9.1), so one container can hold the same kind of grid at
  two spacings. The section holds a block count `n` (u64), `n` sorted block
  keys (u64: `(i_block + 2^31) << 32 | (j_block + 2^31)`, so the order of
  the keys is the order of the signed block indexes), `n + 1` u64 byte
  offsets into the payload, and the payload. `GridBlocks` (138) uses the same
  key.
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

`SteepnessMax` (520, T4) is 4 bits per post, at the spacing and on the blocks
of the `DemGrid` it accompanies. Each value is the steepest slope in the full
resolution source within the post's cell (the source pixels whose centres
fall in `[p − s/2, p + s/2)` in both axes), as a class with upper bounds 2,
5, 8, 11, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60 and 70° (classes 0 to 14),
and 15 for anything steeper. Slopes are Horn's on the source's own grid, with
the thresholds compared exactly in integers. Posts the `DemGrid` has no data
for are absent here too. It lets exit legality and the incident surface catch
embankments and cliffs that a coarse DEM averages away (plan section 8): at
3″ on Małopolskie, 3.25 % of cells hold a source pixel steeper than 30°, but
only 0.66 % look that steep from the 3″ grid itself (E1). It is written
beside a `DemGrid` coarser than its source and left out when the spacing
equals the source's, where it adds nothing. About 1.5 MB in runs on
Małopolskie at 3″ and 3.0 MB at 2″ (E1).

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
- A lattice cell index, the request grid's shape (section 9.3): block keys
  and bases (16 bytes each), 4,096 + 1 block-relative u32 starts per block,
  then u32 line indexes, sorted, for every line that touches the cell.

The class codes are: 1 stream, 2 river or canal, 3 shoreline (lake or
coast), 4 ridge, 5 cliff, 6 embankment or cutting, 7 power line, 8 cutline
or firebreak, 9 fence, 10 wall, 11 hedge, 12 guard rail, 13 railway, 14
fenced road (motorway or motorroad), 15 building outline, and 16 pipeline
above ground. 0 is invalid, 17 to 127 are for later versions, and 128 and
up are private. Like the grids, each of these sections gets its full spec
in the release that first writes it, and this shape is fixed at the freeze.

## 7. Profile slots and masks: what the engine changes

`profile_slot` and `profile_bit` stop being constants: the engine reads the
slot table at open and maps each `MobilityProfile` to a slot (or to none).
Every mask read goes through a `u16`. `BarrierAccess::is_valid` checks bits
against the slot count instead of `PROFILE_SLOTS`. This is mechanical, but it
touches every place a profile bit is built, so it should land as its own PR
(section 12).

## 8. Numbering: tile-ready ids and cell-major numbering

### 8.1 What each gives

- **Tile-ready ids** (on `main`): nodes along a Hilbert curve over one world
  lattice of E7 positions, edges by (way, direction, target position,
  segment, length) within a source node, shared infos by first edge. Ids
  follow from the graph alone, not from the order the compiler built things
  in, and any aligned Hilbert cell is a contiguous id range. Locality is good:
  93.9 % of edges join nodes less than 128 ids apart (*measured*).
- **Cell-major numbering** (REG-13's hook): every dissection cell is one
  contiguous range in every per-node and per-edge section, so a paged
  package can map, validate and evict cells, and a `CellDirectory` can be
  added in 1.x without renumbering. Aligned Hilbert cells don't do this job:
  a straight cut costs 6–11 portal edges per km (regions analysis, section 2.4),
  against 170–220 portal edges for a whole dissection cell of 35–70 k
  nodes, so Hilbert tiles of that size would make the top overlay several
  times larger (*estimated* from those two numbers, not measured).

They are not alternatives. The dissection breaks ties on node ids, so
running it on Hilbert-numbered nodes makes it a function of the graph alone;
the final numbering is then derived from the dissection.

### 8.2 Chains per cell (measured)

Today `cch_order::elimination_order` ranks every degree ≤ 2 node (68.6 % of
Małopolskie's) first, globally, then the core in nested-dissection post-order
(children, then the separator). For cells to be rank ranges, each chain node
must be ranked inside its cell instead. The rule I tested:

- each chain node's *owner* is the deeper of the dissection subtrees that own
  its two neighbours at the time it is eliminated (they are always on one
  root path, because eliminating it adds an arc between them to the core the
  dissection then cuts); a chain node with no core neighbour (a whole
  component of degree ≤ 2) is ranked first;
- a subtree's range is its owned chain nodes (in today's relative order),
  then its children's ranges, then its separator; the core keeps today's
  relative order.

Result on Małopolskie (scratch prototype on the recorded dissection, not in
the repository):

| | Today's order | Chains per cell |
|---|---|---|
| CCH arcs (filled graph, as node pairs) | 3,694,694 | 3,694,694, **identical set** |
| Elimination tree | height 523 | **identical parent of every node** |
| Subtrees that are one contiguous rank range | core only | **every subtree, at every level** |

This is expected: a fill arc joins two nodes through a path of nodes ranked
below both, and moving a chain node never changes its order relative to any
node it can reach without passing a higher one. So the risk the regions analysis lists
(section 7, "per-cell chain elimination makes the order worse") doesn't
arise, and the potentials are unchanged value for value. 6,216 chain
nodes (small isolated components) have no core neighbour; they belong to no
cell and should be ranked by position (Hilbert) at the front, where they
don't affect anything else. The prototype should still be repeated on Poland
in G5, as a check.

### 8.3 Node id = rank

With chains per cell there are two ways to number the package:

1. **Node id = CCH rank** (recommended). Every subtree is a contiguous range of
   node ids, of the edges (grouped by source node), of the shared infos
   (numbered by first edge) and of the shapes. `CchRank` is dropped (−4.3 MB),
   the engine's rank lookups become the identity, and per-rank sections
   (`CchFirstUp`, `CchMetric`) share their ranges with per-node ones, so a
   cell is one range per kind of data.
2. **Cell-major groups, Hilbert inside, `CchRank` kept.** Same contiguity at
   every level, one more indirection, 4.3 MB more.

Locality (*measured*, share of edges whose ends are fewer than 2^7 and 2^15
ids apart): Hilbert today 93.9 % and 99.68 %; either cell-major numbering
90.3 % and 99.83 %. The near-neighbour share drops a little and the
long-range share improves (fewer edges need an exception table if 16-bit
target deltas, OM-10, ever come). Warm query speed should be checked on the
Małopolskie gate in G5 before the choice is final.

In both, every level of the tree is contiguous at once, so the format
doesn't fix a cell size: a 1.x `CellDirectory` chooses its levels (the
regions analysis measured 35 k and 280 k nodes for Poland) from
`CchCellTree`.

### 8.4 How it fits the compiler

1. Build and enhance the road graph, then `renumber_in_package_order` (tile-
   ready ids, unchanged).
2. Compute the elimination order on the renumbered graph, with the tree and
   chains per cell. This moves the order from the package build (where it
   runs on the package's CSR today) to just after the renumbering; it needs
   only `first_out`, `edge_target` and the fixed-point positions.
3. Renumber the road graph a second time by rank with the same
   `permute_nodes`/`permute_edges` machinery. Edges keep their within-node
   order (it uses positions, not ids), shared infos and components follow
   their first edge and node.
4. Build the package sections as today and write `CchCellTree`.

The test `packages_do_not_depend_on_the_order_segments_reach_the_builder`
still holds (the order is a function of the Hilbert-numbered graph). What is
lost is that a node's id no longer depends on its position alone, but on
the whole package's dissection. Nothing depends on that: stable references
use OSM ids, overlay sets bind to the identity hash.

### 8.5 Tiled compiles (1.x)

Tile-ready ids were written so a continent can be compiled as tiles that
number themselves from the counts before them. With rank numbering that
still works, if the tile grid is part of the order's definition: a tiled
compile (and the single compile it must equal) dissects each tile on its own
graph, with the nodes that have edges into other tiles forced to the top
separator, and orders the tiles along the Hilbert curve. A tile's ranks then
depend only on the tile; a second pass resolves edges into neighbour tiles
once their numbering is known. A compile of a single extract is one tile and
keeps the free dissection. This is REG-5a/REG-7's business; format 3 only
needs `RegionInfo`'s cut rule (`dissection-cell` or `polygon`) to record which
was used.

## 9. A global lattice (REG-11)

### 9.1 Definition

Every gridded section uses one angular lattice: posts at
`(i × s_lat, j × s_lon)` ticks from 0° N, 0° E, where a tick is 1/120″,
with `i`, `j` signed 32-bit integers and `s_lat`, `s_lon` whole divisors of
432,000 ticks, a degree (so whole degrees are post lines, where DEM source
tiles meet). 1″ is 120 ticks, 3″ is 360, and ½″ (60) stays possible for a
15 m source such as GEM-15 without a format 4; GLO-30's 1.5″ columns above
50° N (180) and WorldCover's 0.3″ (36) are whole ticks too, so resampling is
exact integer arithmetic. Post indexes still fit an i32 at ½″ (2.6 M posts
around the globe). A grid section states its `(s_lat, s_lon)` and stores
data in blocks of 64 × 64 posts, block
`(floor(i / 64), floor(j / 64))`, as a sorted block index and per-block
offsets, so only covered blocks take space. Global post indexes `(i, j)`
break ties (lexicographically), never a package-local index.

So two regions' grids of the same spacing tile exactly, their overlap in the
halo is byte-identical, and a raster search (X4) gives the same path in
whichever region answers. A bundle fixes one spacing per kind of grid for all
its members; the 1.0 compiler uses `s_lon = s_lat` everywhere.

### 9.2 Users

| Grid | Spacing | Values |
|---|---|---|
| DEM (`DemGrid`, T4) | 3″ default, 1″ or 6″, ½″ possible | i16 whole metres, absent posts in the block's presence bitmap; Rice-coded MED residuals per block (section 6.1) |
| Steepness (`SteepnessMax`, T4) | the DEM's, when coarser than its source | 4-bit classes of the steepest full-resolution slope, in runs (section 6.1) |
| Land cover (`LandCover`, X3a) | 1″ or 2″ (25–50 m) | 4-bit classes (1–11 WorldCover's, 12–15 for OSM-only classes), in runs |
| X4 friction raster | built per request at 1″ | not stored |
| Request grid (section 9.3) | 12″ | edge lists |

DEM sources (SRTM, NASADEM, GLO-30) are resampled onto the posts with
integer arithmetic (the T1 reader's rules); a source with a different
longitude spacing at high latitude (GLO-30) is resampled like any other.

### 9.3 The request grid

Today `RequestGrid` is a dense grid of 0.003° cells over the package's
bounding box with a u32 start per cell (`GridCellStart`, 1.49 MB for 468 ×
794 cells on Małopolskie, *measured*). For Europe's box that is about 1–2 GB
of mostly empty starts, and the planet's 6.8 × 10^9 cells overflow u32
(the compile analysis). Format 3 puts it on the lattice: 12″ cells (0.00333°,
about 370 m by 240 m at 50° N, close to today's 10.8″, which is not a
divisor of a degree), in 64 × 64-cell blocks; `GridBlocks` (id 138) lists the
covered blocks with a u64 key and a u64 base into `GridEntries`, and
`GridCellStart` (id 139) holds 4,096 + 1 block-relative u32 starts per
covered block. Memory follows the road network, not the bounding box, and a
cell directory can page blocks with cells. Candidate order must not depend on
the cell size. If it turns out to, the change shows in the baselines the bump
re-records anyway (section 12).

## 10. Width limits found at scale

From the compile analysis (section 3) and the regions analysis (section 1.1), with
Europe at 330–400 M edges and the planet at 1.0–1.15 G (*estimated* there):

| Limit | Where | Breaks at | Format 3 |
|---|---|---|---|
| `ShapeStart` u32 byte offsets | `format.rs`; the compiler refuses past 4 GiB of shapes | ~880 M edges (planet) | u64 (id 135); +0.3 MB on Małopolskie, since it stores every 16th shape |
| `AttrClassStart`, `StringStart` u32 byte offsets | `format.rs` | byte pools past 4 GiB (planet, *estimated*) | u64 (ids 136, 137) |
| Dense request grid, u32 cell index | `package_build/request.rs` | Europe ~1–2 GB of starts; planet overflows | sparse lattice grid (section 9.3) |
| CCH arc ids, 31 bits (`UP_BIT`) | `package/cch.rs` (engine; it builds no potentials past it) | ~2.1 G arcs: Europe close, planet over | engine change, not format; stored `CchFirstUp` is u64 (section 6) |
| Cost tuple index u16 (`MAX_COST_TUPLES`) | `SharedInfoCostTuple` | 65,536 distinct tuples; Małopolskie has 3,927 (*measured*) | u32 (+2.6 MB on Małopolskie), decided 2026-11-23: Poland has 11,028 and Germany 35,993 (55 % of u16), so a Europe build would likely overflow u16 (section 13) |
| Node and edge counts, u32 | everywhere | ~4.29 G (the planet is ~1.1 G edges) | none; the compiler refuses past it with a typed error |
| `GridEntries` count u32 | request grid | 0.69 entries per edge on Małopolskie (*measured*), ~0.8 G on the planet | none |

Every other `*Start` array counts elements of a section bounded by an u32
count (edges, shared infos, restrictions) and stays u32.

## 11. What stays out of the bump

- **Content sections** (elevation, DEM grid, barrier lines, land cover,
  directional destinations, conditional turn restrictions, truck limits) and
  **stored potentials and the cell directory**: optional sections, one
  release at a time after the bump, on reserved ids.
- **Portals, cliques, bounds, owner bits, neighbours**: not written before
  1.0 (the regions analysis, section 6.7).
- **The OM-10 diet** (durations as a speed index, 16-bit deltas for edge
  targets and twins) and SC-8's correlation boxes. These change core sections,
  so under "one id, one layout" and the 1.x policy (section 13) they ride
  the bump or wait for a format 4. They are hot-path decoding changes (item
  29 of the performance analysis, about −28 MiB, −16 %, *estimated*) and
  need their own measurement.
  Recommended: prototype the edge-target and twin deltas on the rank
  numbering (99.83 % of edges fit 16 bits, *measured*) before the freeze, and
  include them only if the gate shows no slowdown; leave the speed index out.
- **The sparse search state** (REG-12): no format change; it can land
  before the bump.
- **Region-qualified ids in results, overlay sets per region, `region_missing`,
  C ABI 2 open from a source**: wire and ABI (REG-4, API-5, FFI-4), not
  format. Format 3 supplies what they need: `RegionInfo`, `NodeSourceId`,
  the identity hash.

## 12. Migration and implementation

**Reading.** An engine with format 3 reads formats 2 and 3:
`MIN_READ_FORMAT_VERSION` stays 2 for at least one release after the bump,
as the format rule says. The format-2 path maps its positional slots to ids
and synthesizes what format 3 states explicitly (profile slots from the
header, params from the JSON metadata, ids from `EdgeId`), so the engine
above the format crate sees one shape. For a format-2 package
`datasetHash` and `contentHash` are both the root checksum.

**Writing.** The compiler writes only format 3. Recompiling changes edge ids
(rank numbering) and the dataset hash once, so overlay sets bound by edge id
need rebinding (stable references survive), and route dumps may change at
ties, as they did with tile-ready ids. Monaco info, the Kraków and the
Małopolskie baselines are re-recorded once, in the release that ships the
bump.

**Building it on `main`.** Format 3 is several PRs, and packages written by
`main` between them must not be mistaken for the final format. Until
`chore/format-3-release` the writer sits behind a compiler switch that is off
by default, so default compiles stay format 2 and byte-identical. Draft
revision 1 kept format 2's profile columns and heuristic costs at header
offsets 96 and 104; draft revision 2 moves them to `ProfileSlots`, and the
header is zero from 88 to the checksum. Draft revision 3 writes the
attribute-class record revision in `AttrClassBytes`' `param`. The first PR
sets the version to 3 and writes `PackageParams` key 10 (draft revision);
each later PR that changes bytes raises it, and an engine refuses any draft
revision but its own; the last PR removes the key. Suggested order:

1. `feat/format-3-table`: tagged table, flags, table offset, both hashes,
   `PackageParams`, retired reserved sections, implied ids, the format-2
   read path; QA-5 fuzz target for the header and table (duplicate ids,
   unknown REQUIRED ids, element size and flag mismatches, overlaps, a table
   before and after the sections).
2. `feat/format-3-profile-slots`: `ProfileSlots`, u16 masks, u32 barrier
   access (section 7).
3. `feat/format-3-leg-attrs`: revision-tolerant `LegAttrs` and correlation
   records; fuzz the decoder with longer and shorter records.
4. `feat/format-3-identity`: `RegionInfo`, `NodeSourceId`, the wire's
   `contentHash`.
5. `feat/cell-major-numbering` (G5): chains per cell, rank numbering,
   `CchCellTree`, repeated on Poland.
6. `feat/format-3-widths`: u64 offsets, the lattice request grid, the u32
   cost tuple index.
7. `chore/format-3-release`: remove the draft key, re-record baselines,
   release notes.

Before the freeze: EXP-5b step 1 (search across a virtual seam inside one
package) checks that `RegionInfo`, `PortalEdge`/`PortalRef` and the cell
tree carry what a seam needs, so that a 1.x bundle doesn't need a format 4.

## 13. Decisions

Anton confirmed every recommendation below on 2026-11-22.

- **D5, format 3 shape.** Optional sections addable after the bump, a
  per-section REQUIRED flag (no separate header bitset), an identity hash
  separate from the content hash, u16 profile access with a slot table.
- **D12, regions scope.** Poland on a phone offline is a 1.x goal; its
  format-3 hooks (cell-major numbering, reserved cell ids) are in 1.0.
- **1.x policy on REQUIRED sections.** A 1.x compiler writes a REQUIRED
  section that 1.0 doesn't know only when the compile request asks for the
  feature that needs it, and the release notes name the oldest engine that
  reads such packages. (Not chosen: no new REQUIRED sections in 1.x at all,
  which would turn any feature that changes how a core section is read into a
  format 4.)
- **Numbering.** Node id = CCH rank with chains per cell (section 8.3, option
  1); `CchRank` is retired. G5 still repeats the fill check on Poland and
  measures warm queries on the Małopolskie gate; if those show a regression,
  option 2 (cell-major groups keeping `CchRank`) is the fallback.
- **OSM node ids in every package.** `NodeSourceId` is mandatory (+6.4 MB,
  3.5 % on Małopolskie), so API-5's resolver works on every 1.0 package.
- **OM-10 deltas.** 16-bit edge-target and twin deltas go into the bump only
  if a prototype on the rank numbering shows no slowdown on the gate;
  otherwise they wait for a format 4. The duration speed index stays out.
- **Cost tuple index.** u32 (Anton, 2026-11-23). `costTupleCount` is
  11,028 for Poland and 35,993 for Germany (2026-11-23 extracts), and
  Germany's is 55 % of u16's 65,536, too close for a format that must carry
  every 1.x package. It lands with the other widths (`feat/format-3-widths`).

## Appendix: how the numbers were taken

- Package: `wayfarer compile malopolskie-261118.osm.pbf --country PL` from
  `main` at `5cb573c`, release build, 4-core cloud machine (15 s); section
  sizes from `wayfarer verify`. Byte length 184,954,112, as the tile-ready
  ids PR reported.
- Implied ids: `EdgeId[i] == i` for every edge and `NodeIdDelta[i] == i` for
  every node, read from the file.
- Chains per cell and locality: a scratch Rust tool (not committed) that opens
  the package, re-runs the dissection with its tree recorded (a copy of
  `cch_order.rs` made for the regions analysis;
  its order equals the stored `CchRank`), records each chain node's two
  neighbours when it is eliminated, builds the per-cell order, and compares
  the filled graph of both orders by symbolic elimination (higher neighbours
  of each node merged into its lowest one), as sets of node pairs, plus the
  elimination-tree parent of every node. Locality counts directed edges by
  `|id(target) - id(source)|` under each numbering.

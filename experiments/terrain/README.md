# Terrain experiments E1-E3 and E5 (lane TERRX)

Throwaway experiment, not part of the workspace or any default build (it has its own
`[workspace]`). It reads DEM and land-cover rasters, resamples them onto the format-3
global arc-second lattice (docs/design/format-3.md section 9) with integer arithmetic only,
and measures block encodings (E1). E2 scores the DEM sources against GUGiK 1 m LiDAR
(`scripts/run_e2.sh`, `scripts/gugik_lattice.py`). Results and source URLs:
`off-road-sar/exp/E1/README.md` and `off-road-sar/exp/E2/README.md` in the project files.

    cargo build --release
    target/release/terrexp dem  <dem-dir> <blocks.txt> <spacing-ticks> <out-dir> [--steep] [--point] [--threads N]
    target/release/terrexp lc   <worldcover-dir> <blocks.txt> <spacing-ticks> <out-dir>
    target/release/terrexp dump <tile> <row> <col> <n>
    target/release/terrexp bench <out-dir>/dem_m_rice.wte

Spacing is in ticks of 1/120 arc-second (120 = 1", 360 = 3", 60 = 1/2").

- `src/raster.rs`: GeoTIFF/COG via `tiff` 0.11 (GLO-30 Float32, NASADEM Int16), a raw tile
  path for palette images (WorldCover, which `tiff` refuses), and SRTM HGT(.gz). Geographic
  EPSG:4326, north-up, one band; scale and tie point must sit on whole ticks.
- `src/mosaic.rs`: tiles indexed by geotransform; bilinear in exact tick arithmetic across
  tile seams; NoData corners dropped and renormalised; shared HGT edges checked.
- `src/dem.rs`: 1" and finer = bilinear at the post; coarser = mean of the 1" posts over the
  post's cell; 4-bit steepness-max from Horn slope on the source grid.
- `src/landcover.rs`: WorldCover majority per cell with a fixed tie priority.
- `src/encode.rs`: per-block zig-zag varint (delta, MED) and Rice-coded MED residuals; nibble
  layers packed or run-length coded.
- `scripts/`: block lists from the OSM boundary, the run matrix, independent numpy/rasterio
  checks, the byte-identity run across targets, the GUGiK spacing study and the E2
  DEM-source comparison.

## E3 (land cover)

    target/release/terrexp osm <pbf> <blocks.txt> <spacing-ticks> <out-dir> [--threads N]

- `src/osm.rs`: OSM closed ways and multipolygon relations (rings joined by end nodes, even-odd
  fill, smaller area on top) sampled 4x4 per post in exact integer arithmetic; writes
  `posts_fine.u8` (31 fine classes) and `posts_cov.u8` (covered samples 0-16).
- `terrexp lc` also writes `posts_purity.u8` (majority count and valid pixels per post).
- `scripts/e3/`: BDOT10k onto the lattice (reference), the independent OSM check, the evaluation
  against BDOT10k and the fused-layer sizes. Results: `off-road-sar/exp/E3/README.md`.

## E5 (incident cost surface)

    target/release/terrexp lines   <pbf> <blocks.txt> <spacing-ticks> <out-dir>
    target/release/terrexp surface <prep-dir> <spacing-ticks> <E4 pace_table_50.csv> <out-dir> [--queries N] [--threads N]

- `src/lines.rs`: OSM linear features (way kinds by highway/`sac_scale`, rivers, streams, railways, fences, cliffs)
  as a u16 bitmask per post; exact supercover, 4-connected.
- `src/surface.rs`: per-profile friction (u16 time per metre, from a class distribution over E3's codes plus
  steepness-max and barrier multipliers), palette-run block coding, 16-connected integer Dijkstra and A* with the
  E4 pace-by-grade table applied per edge.
- `terrexp dem --steep` also writes `posts_steep.u8` (raw steepness-max, 0xFF = none).
- `scripts/e5/`: stage-1 prep (GEM-15 onto the lattice, E3 fusion), the timing and identity matrix, the independent
  Python check and the pictures. Results: `off-road-sar/exp/E5/README.md`.

## E10 (terrain layer contract)

    python3 -I scripts/e10/base_e3.py <wc-dir> <osm-dir> <dem-dir> <base-dir>
    target/release/terrexp e10 <base-dir> <dem-dir> <out-dir>
    python3 -I scripts/e10/check_fusion.py <base-dir> <out-dir>
    scripts/e10/identity.sh <base-dir> <dem-dir> <work-dir>

The reference implementation of `docs/design/terrain-layers.md` (on branch `docs/terrain-layers`).
- `src/layers.rs`: layer manifest, canonical content hash, the WTL0 file (plane codings raw, one
  value, runs, zlib), the producer's quantiser and largest-remainder rounding.
- `src/fuse.rs`: fusion of the base with class, hazard, level and feature layers and team overlay
  items at an incident time, exact supercover of feature lines, the fusion record.
- `src/voxel.rs`: the v0 voxel graph and its projection into a hazard and a feature layer.
- `src/e10.rs`: synthetic layers on a real base, written, read back, fused and checked (input
  order, threads, incremental re-fusion, zero confidence, expiry); a placeholder friction surface.
- `scripts/e10/`: the E3-rule base builder, an independent Python implementation of the contract
  that re-hashes every layer and re-fuses a sample, and the byte-identity run across targets.
  Results: `off-road-sar/exp/E10/README.md`.

## E14 (incident surface contract)

    target/release/terrexp e14 <E5 prep-dir> <E4 pace_table_50.csv> <out-dir>
    python3 -I scripts/e14/check_surface.py <prep-dir> <out-dir> <e14.json>
    python3 -I scripts/e10/check_fusion.py <out-dir> <out-dir>
    scripts/e14/identity.sh <prep-dir> <pace-csv> <work-dir>

The reference implementation of `docs/design/incident-surface.md` (on branch `docs/incident-surface`).
- `src/incident.rs`: the cost rules from the fused state (blocking mass, hazards, levels, effect bits, risk,
  reasons), the profile tables' canonical bytes, the WIS0 surface file and WISD deltas, block, content and recipe
  hashes, epochs and `same_until`, the blocks an update touches, the canonical path and route confidence.
- `src/e14.rs`: an eight-step incident timeline on E5's Tatra base (layers arriving, a late older layer, a team
  overlay set, a flood fading and expiring), each step built in full and incrementally, written, read back and
  sent as a delta; routes from the incident point with their confidence.
- `scripts/e14/`: the independent Python check (files, hashes, deltas, cost rules, two routes) and the
  byte-identity run across targets. Results: `off-road-sar/exp/E14/README.md`.

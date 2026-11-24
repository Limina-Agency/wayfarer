# Terrain experiment E1 (lane TERRX)

Throwaway experiment, not part of the workspace or any default build (it has its own
`[workspace]`). It reads DEM and land-cover rasters, resamples them onto the format-3
global arc-second lattice (docs/design/format-3.md section 9) with integer arithmetic only,
and measures block encodings. Results and source URLs: `off-road-sar/exp/E1/README.md` in
the project files.

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
  checks, the byte-identity run across targets, and the GUGiK spacing study.

## E4: pace models against OSM GPS traces

`scripts/e4/`: fetch OSM public trackpoints (`fetch_traces.py`), OSM highway ways near the test areas
(`osm_ways.py`, `coverage.py`), clean walking tracks into 50/100 m segments sampled on the `terrexp`
lattices (`segments.py`), GUGiK LiDAR slopes in the E1 boxes (`lidar.py`), and the model comparison and
fits (`fit.py`). `run_e4.sh` runs the whole thing. Results: `off-road-sar/exp/E4/README.md` in the project files.

# wayfarer-terrain-compiler

Stage 1 of Wayfarer's off-road terrain, experimental. It compiles DEM and
land-cover COG tiles into a terrain file (`wayfarer-terrain`'s sidecar format,
v1) on the global arc-second lattice:

- **Heights** from tiers of DEM sources, best first: GEM-15 (Google's 15 m
  DTM in UTM, the squares an app may already hold and draw), then Copernicus
  GLO-30 where GEM-15 has no data. Each post is the mean of 8 × 8 bilinear
  samples over its cell at 1″, in decimetres.
- **Steepness-max** from the same full-resolution pixels: the largest Horn
  slope class (`docs/design/format-4.md` section 4) of the pixels under a
  post's samples.
- **Land cover** from ESA WorldCover: the majority class of the pixels in a
  post's cell, confident when it holds three quarters of them.

The OSM stage (the land-cover override and its confidence, linear features)
comes next and fills the `cover`, `conf` and `lines` planes from an extract.

## Memory and streaming

The compile reads only the tiles a block needs, keeps decoded tiles in a cache
of bounded size (`Options::cache_bytes`, 64 MiB by default) and works one block
row at a time (64 posts, about 2 km at 1″), streaming the file out with
`TerrainWriter`. Its peak heap is the cache plus about 12 MiB per worker thread,
whatever the region's size. `tests/memory_budget.rs` checks this in CI with a
counting allocator: peak heap within the cache, 12 MiB a worker and 8 MiB, and
no growth from 147 to 441 blocks.

Małopolskie's box (49.17-50.53 N, 19.08-21.43 E, 10,374 blocks, 42.5 M posts),
measured on a 4-core cloud x86-64 machine with three GEM-15 squares, eight
GLO-30 tiles and two WorldCover tiles:

| Threads | Cache | Peak heap | Peak RSS | Time | File |
|---|---|---|---|---|---|
| 4 | 16 MiB | 64 MiB | 94 MiB | 50 s | 46.9 MB |
| 4 | 64 MiB | 110 MiB | 168 MiB | 33 s | 46.9 MB |
| 1 | 64 MiB | 81 MiB | 101 MiB | 110 s | 46.9 MB |
| 4 | 256 MiB | 295 MiB | 367 MiB | 24 s | 46.9 MB |

Every run writes the same bytes. A smaller cache only decodes tiles again
(17,191 decodes at 16 MiB against 1,189 at 256 MiB).

## Determinism

Everything after the UTM projection is integer arithmetic. The projection is
Krüger's series in `f64` with `libm`, at anchors every 8 posts with integer
interpolation in between, so the same inputs give the same file on every
target, thread count, cache size and input order. `tests/compile.rs` pins the
hash of a synthetic compile; it matches on x86-64, i686 and aarch64.

## Example

```rust,no_run
use wayfarer_terrain_compiler::{Options, Region, SourceFile, Sources, attribution, compile};

let sources = Sources {
    heights: vec![
        vec![SourceFile::open("gem15/dtm_w19s49.tif")?, SourceFile::open("gem15/dtm_w20s49.tif")?],
        vec![SourceFile::open("glo30/Copernicus_DSM_COG_10_N49_00_E019_00_DEM.tif")?],
    ],
    land_cover: vec![SourceFile::open("worldcover/ESA_WorldCover_10m_2021_v200_N48E018_Map.tif")?],
    attribution: attribution::ALL.join("\n"),
};
let region = Region::Bbox { south: 49.2, west: 19.84, north: 49.38, east: 20.12 };
let out = std::fs::File::create("tatra.wtr")?;
let (report, _) = compile(sources, &region, &Options::default(), out)?;
println!("{} blocks, {} bytes", report.blocks, report.file_bytes);
# Ok::<(), Box<dyn std::error::Error>>(())
```

`examples/compile_terrain.rs` does the same from directories of tiles and
prints the report with the peak heap; `examples/inspect_terrain.rs` prints a
terrain file's plane statistics.

## Sources and licences

The attribution the file's sources text should carry is in
`attribution::{GEM15, GLO30, WORLDCOVER}`. GEM-15 and WorldCover are CC BY 4.0;
GLO-30 is free to use with its Copernicus attribution.

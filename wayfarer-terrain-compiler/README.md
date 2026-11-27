# wayfarer-terrain-compiler

Stage 1 of Wayfarer's off-road terrain, experimental. It compiles DEM and
land-cover COG tiles and an OSM extract into a terrain file
(`wayfarer-terrain`'s sidecar format, v1) on the global arc-second lattice:

- **Heights** from tiers of DEM sources, best first: GEM-15 (Google's 15 m
  DTM in UTM, the squares an app may already hold and draw), then Copernicus
  GLO-30 where GEM-15 has no data. Each post is the mean of 8 × 8 bilinear
  samples over its cell at 1″, in decimetres.
- **Steepness-max** from the same full-resolution pixels: the largest Horn
  slope class (`docs/design/format-4.md` section 4) of the pixels under a
  post's samples.
- **Land cover** from ESA WorldCover: the majority class of the pixels in a
  post's cell, confident when it holds three quarters of them. OSM areas
  (closed ways and multipolygons, rings joined from their member ways)
  override it where they cover half the cell, except forest and lowland
  scrub; OSM scrub from 1,300 m is dwarf pine, and bare rock, scree and bog
  get codes of their own. The confidence bit is then set where OSM and
  WorldCover agree (E3's recommended fusion).
- **Linear features** from OSM ways, in the lines mask
  (`wayfarer_terrain::lines`): road kinds, paths by `sac_scale`, steps,
  rivers, streams, railways, fences and walls, cliffs. Each way marks every
  cell it touches, so a line is 4-connected and an 8-connected mover cannot
  slip across it.

On the Tatra box the OSM classes, their coverage and the lines mask match the
E3/E5 reference cell for cell, and the fused land cover differs on 1 post in
720,896 (the dwarf pine height threshold, read from the post's own height).

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

## Compiling on a host

Terrain files are not shipped: an app compiles one for its region from the
tiles it already holds (the GEM-15 squares it draws, GLO-30 and WorldCover
for the rest), on the device or anywhere else, and gets the same bytes.

From Rust:

```rust,no_run
use wayfarer_terrain_compiler::{CancelToken, Options, Region, Sources, compile_to_file};

// Files or directories of `.tif` tiles; each inner list is one height tier, best first.
let sources = Sources::from_paths(&[vec!["tiles/gem15"], vec!["tiles/glo30"]], &["tiles/worldcover"])?;
let region = Region::Bbox { south: 49.2, west: 19.84, north: 49.38, east: 20.12 };
let cancel = CancelToken::new(); // `cancel.cancel()` from any thread stops it
let report = compile_to_file(sources, &region, &Options::default(), "tatra.wtr", &cancel, |p| {
    eprintln!("{:.0}%", p.fraction() * 100.0)
})?;
println!("{} blocks, {} bytes", report.blocks, report.file_bytes);
# Ok::<(), Box<dyn std::error::Error>>(())
```

- `compile_to_file` writes `<path>.partial` and moves it into place when it
  is complete; an error or a cancel removes it, so the path never holds part
  of a file.
- `compile` writes to any seekable sink, and `SourceFile::new` takes tiles
  from any reader (an app's own tile store, say), not only from paths.
- Progress comes after each block row; a cancel stops the compile within a
  block. The token is `wayfarer-package`'s `CancelToken`, the one the route
  compiler and the engine take.
- `TerrainJob` runs a compile on its own thread, for hosts that poll its
  progress instead of taking a callback (a UI event loop, a C ABI).
- `Options::cache_bytes` and `Options::threads` set the memory and cores it
  uses (see the table above); neither changes the file.

From the command line, `wayfarer terrain` (crate `wayfarer-cli`):

```text
wayfarer terrain compile --heights gem15/ --heights glo30/ --land-cover worldcover/ \
    --osm malopolskie.osm.pbf --bbox 49.17,19.08,50.53,21.43 [--threads 4] -o malopolskie.wtr
wayfarer terrain inspect malopolskie.wtr
```

Each `--heights` is one tier, best first; a tier of several paths joins them
as `PATH` does. `compile` shows progress on stderr, prints the report as JSON
and stops cleanly on Ctrl-C, leaving no file; `inspect` prints a file's plane
statistics as JSON.

## The OSM stage's memory

The extract is read in two passes: pass 1 builds a node-location store (in
memory for a small extract, else in mapped files under `Options::scratch_dir`),
pass 2 keeps only the ways that meet the region, plus the member ways of land-
cover multipolygons. The areas and lines that meet the region stay in memory
for the compile; rows are then rasterized one at a time. With the Małopolskie
extract (204 MB):

| Region | Blocks | Peak heap | Peak RSS | Time (4 threads) |
|---|---|---|---|---|
| Tatra box | 176 | 91 MiB | 297 MiB | 2.5 s |
| Małopolskie box | 10,374 | 1,121 MiB | 1,362 MiB | 52 s (122 s on 1 thread) |

The RSS above the heap is the store's mapped pages, which the system can drop.

## Sources and licences

The attribution the file's sources text should carry is in
`attribution::{GEM15, GLO30, WORLDCOVER, OSM}`. GEM-15 and WorldCover are CC BY 4.0;
GLO-30 is free to use with its Copernicus attribution; OSM data is ODbL 1.0
(`Sources::with_osm` adds its line).

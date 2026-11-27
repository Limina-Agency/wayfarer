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

From the command line, the `wayfarer-terrain` binary (feature `cli`, on by
default; turn it off with `default-features = false` to build without clap):

```text
cargo install --path wayfarer-terrain-compiler
wayfarer-terrain compile --heights gem15/ --heights glo30/ --land-cover worldcover/ \
    --bbox 49.17,19.08,50.53,21.43 [--threads 4] [--cache-mib 64] -o malopolskie.wtr
wayfarer-terrain inspect malopolskie.wtr
```

Each `--heights` is one tier, best first; a tier of several paths joins them
as `PATH` does. `compile` shows progress on stderr, prints the report as JSON
(with the peak heap) and stops cleanly on Ctrl-C; `inspect` prints a file's
plane statistics as JSON. The binary joins the `wayfarer` command line when
the terrain crates are published (0.6).

## Sources and licences

The attribution the file's sources text should carry is in
`attribution::{GEM15, GLO30, WORLDCOVER}`. GEM-15 and WorldCover are CC BY 4.0;
GLO-30 is free to use with its Copernicus attribution.

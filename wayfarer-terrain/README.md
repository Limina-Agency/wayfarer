# wayfarer-terrain

Off-road terrain for Wayfarer, experimental. It holds the three stages of the
off-road plan:

1. **Base.** A terrain file on the global arc-second lattice of format 3. Per post
   it stores the height in decimetres, the steepness-max class, the land-cover code
   with its confidence bit, and the linear-feature mask. Blocks are 64 × 64 posts.
2. **Fusion.** Terrain layers are fused with the base at an incident epoch into an
   incident surface. Layers are model output, hazard rasters or drone features,
   and the team overlay comes on top. The surface holds per post and per profile
   the time per metre, the risk that the cell is closed and why it is closed. It
   carries a content hash (same hash, same routes) and a recipe hash over every
   input, and it can be updated by deltas.
3. **Query.** 16-connected integer Dijkstra and A* run on the surface, with pace by
   grade from the heights, canonical paths and route confidence.

The formats follow the drafts `docs/design/terrain-layers.md` (layer contract v0)
and `docs/design/incident-surface.md` (incident surface v1 draft: a fuse version in
the recipe, an overlay that is a set applied by observed time, an AOI cut). The
terrain file is the sidecar format in `terrain.rs` (v1: a sources and attribution
text, the hash folded per block), used until format 4 fixes the layouts.
Everything is integer arithmetic or `libm`, so the same inputs give the same
bytes on every target. CI checks this with pinned hashes in `tests/incident.rs`.
The profile values are assumptions until the field trial; only the walker's pace
by grade is measured (E4).

## Example

```rust
use wayfarer_terrain::fuse::{HashedLayer, Prior};
use wayfarer_terrain::profile::Rules;
use wayfarer_terrain::search::{Grid, route};
use wayfarer_terrain::surface::{Incident, Surface};
use wayfarer_terrain::terrain::Terrain;

fn plan(terrain_file: &[u8], layer_files: &[Vec<u8>], t: i64) -> wayfarer_terrain::Result<()> {
    let terrain = Terrain::read(terrain_file)?;
    let layers = layer_files.iter().map(|b| HashedLayer::read(b)).collect::<Result<Vec<_>, _>>()?;
    let rules = [Rules::walker_default(), Rules::vehicle()];
    let prior = Prior::default();
    let incident = Incident {
        id: *b"incident-0001\0\0\0",
        terrain: &terrain,
        prior: &prior,
        layers: layers.iter().collect(),
        overlay: vec![],
        rules: &rules,
        epoch_s: 900,
        allow_research: false,
        feature_min_conf: 128,
        level_min_w: 128,
    };
    let surface = incident.surface(t)?;
    // A device that fused the same layers checks it got the same surface.
    let shared = Surface::read(&surface.write())?;
    assert_eq!(shared.content_hash(), surface.content_hash());

    let grid = Grid::new(&terrain, &surface)?;
    let walker = grid.profile("walker").unwrap();
    let (a, b) = (grid.cell_of(177_440, 71_936), grid.cell_of(177_400, 71_990));
    if let (Some(a), Some(b)) = (a, b) {
        if let Some(r) = route(&grid, walker, a, b) {
            println!("{} s, mean certainty {}/255", r.conf.cost_us / 1_000_000, r.conf.mean_cert);
        }
    }
    Ok(())
}
```

## Checking against the experiment

`tests/e14_replay.rs` rebuilds experiment E14's eight incident surfaces, seven
deltas and walker routes on the real Tatra base. It compares them with the
experiment's files byte for byte. Set `WAYFARER_E14_PREP` and `WAYFARER_E14_OUT`
to run it; see the test's header.

## Speed and memory

`examples/bench.rs` builds a surface on a synthetic incident area of any size and reports
build time, surface size, a walker route and peak heap. It needs no data:

```sh
cargo run --release -p wayfarer-terrain --example bench -- --km2 900
RAYON_NUM_THREADS=1 cargo run --release -p wayfarer-terrain --example bench -- --km2 900
```

`tests/memory.rs` holds the budget in CI. It counts peak heap for the inputs, the surface
build and one walker route: at most 96 MB for 451 km² and 128 MB for 900 km² (an IPP ± 15 km
AOI). Blocks are costed as soon as they are fused, so fusion state never exists for the
whole area at once.

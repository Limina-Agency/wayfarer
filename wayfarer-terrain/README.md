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
the recipe, an overlay that is a set applied by observed time, an AOI cut). Overlay
items are boxes, polygons (filled even-odd, holes allowed) or lines (the supercover
feature lines use), each with one exact integer rule for the posts it marks. The
terrain file is the sidecar format in `terrain.rs` (v1: a sources and attribution
text, the hash folded per block), used until format 4 fixes the layouts.
Everything is integer arithmetic or `libm`, so the same inputs give the same
bytes on every target. CI checks this with pinned hashes in `tests/incident.rs`.
The profile values are assumptions until the field trial; only the walker's pace
by grade is measured (E4).

## Example

```rust
use wayfarer_terrain::fuse::{Effect, OverlayItem};
use wayfarer_terrain::incident::{Aoi, Incident, Settings};
use wayfarer_terrain::profile::Rules;
use wayfarer_terrain::surface::Surface;
use wayfarer_terrain::terrain::Terrain;

fn plan(terrain_file: &[u8], layer_files: &[Vec<u8>], now: i64) -> wayfarer_terrain::Result<()> {
    let terrain = Terrain::read(terrain_file)?;
    // The mission's AOI in degrees: the terrain is cut to it once.
    let aoi = Aoi::from_degrees((49.15, 49.30), (19.85, 20.10))?;
    let rules = vec![Rules::walker_default(), Rules::vehicle()];
    let mut incident = Incident::new(*b"mission-0001\0\0\0\0", &terrain, Some(aoi), rules, Settings::default())?;
    for f in layer_files {
        incident.add_layer(f)?;
    }
    // The whole team overlay, in any order.
    incident.set_overlay(vec![OverlayItem::polygon_from_degrees(
        &[vec![(49.22, 19.95), (49.23, 19.95), (49.22, 19.97)]],
        now - 600,
        Effect::Block,
    )?])?;
    // At the host's time; later calls re-fuse only what changed.
    let surface = incident.surface_at(now)?;
    // A device that built from the same inputs has the same recipe hash.
    let shared = Surface::read(&surface.write())?;
    assert_eq!(shared.recipe_hash(), surface.recipe_hash());

    if let Some(r) = incident.route("walker", (49.20, 19.90), (49.25, 20.05))? {
        println!("{} s, mean certainty {}/255", r.route.conf.cost_us / 1_000_000, r.route.conf.mean_cert);
    }
    if let Some(c) = incident.cell(49.21, 19.93)? {
        println!("cover {}, steepness {}, hazards {:?}", c.post.cover, c.post.steep, c.hazards);
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
build and one walker route: at most 64 MB for 451 km², 112 MB for 900 km² (an IPP ± 15 km
AOI) and 320 MB for 2,500 km². Measured: 45, 89 and 250 MB. Blocks are costed as soon as
they are fused, so fusion state never exists for the whole area at once, and layer planes
are held at their slot width, so a class or hazard layer takes one byte per value.

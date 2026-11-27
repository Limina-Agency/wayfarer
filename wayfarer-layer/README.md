# wayfarer-layer

Terrain layer files for Wayfarer, experimental. A layer is what a producer hands to the
engine's fusion: a model's output (land cover, flood, rubble), a hazard raster or drone
features, on the global arc-second lattice, with a manifest that says where it came from and
when it holds.

This crate holds only what a producer needs to write and check a layer: the lattice, the
`WTL0` layer format and the byte codings they share. Its only dependencies are `blake3`,
`flate2` and `thiserror`, so it builds for phones as well as desktops. Fusing layers into an
incident surface and searching it is `wayfarer-terrain`'s job.

A layer's identity is the blake3 hash of its canonical form, not of the file bytes, and the one
float step (`layer::quantise_probs`) is exact in f64. Two devices that write the same layer get
the same hash. The format follows `docs/design/terrain-layers.md` (contract v1, wire version
0.1: the same bytes as the v0 draft, so earlier layers keep their hashes).

See the crate documentation for an example that writes a flood layer and reads it back.

# Format 3 routing package

`short_road.mrt` is the synthetic road of `../short_road.mrt` (one
bidirectional road at latitude 52°, longitude 21°–21.002°, no real-world data)
in package format 3, the format 0.6 first writes: a tagged section table,
implied ids numbered by CCH rank, `PackageParams` and the lattice request
grid. `wayfarer/tests/package_formats.rs` checks that the engine keeps opening
and routing it.

It was built with

```sh
cargo run -p wayfarer --example make_test_archive -- fixtures/routing/format-3/short_road.mrt
```

Never regenerate it: it stands for the format-3 packages in use. A new format
version gets its own `format-<N>/` directory.

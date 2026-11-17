# Format 1 routing package

`short_road.mrt` is the synthetic road of `../short_road.mrt` (one
bidirectional road at latitude 52°, longitude 21°–21.002°, no real-world data)
in package format 1. `wayfarer/tests/package_formats.rs` checks that the engine
keeps opening and routing it.

It was built with

```sh
cargo run -p wayfarer --example make_test_archive -- fixtures/routing/format-1/short_road.mrt
```

Once format 1 is released, never regenerate it: it stands for the packages
already in use. A new format version gets its own `format-<N>/` directory.

# Format 2 routing package

`short_road.mrt` is the synthetic road of `../short_road.mrt` (one
bidirectional road at latitude 52°, longitude 21°–21.002°, no real-world data)
in package format 2: it stores the CCH elimination order (`CchRank`).
`wayfarer/tests/package_formats.rs` checks that the engine keeps opening and
routing it.

It was built with

```sh
cargo run -p wayfarer --example make_test_archive -- fixtures/routing/format-2/short_road.mrt
```

Once format 2 is released, never regenerate it: it stands for the packages
already in use. A new format version gets its own `format-<N>/` directory.

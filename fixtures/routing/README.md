# Routing test package

`short_road.mrt` is a routing package with one synthetic bidirectional road at latitude 52°, longitude 21°–21.002°. It contains no real-world or personal data.

Regenerate from the shared fixture builder:

```sh
cargo run -p wayfarer --example make_test_archive -- fixtures/routing/short_road.mrt
```

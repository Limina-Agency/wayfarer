# Fuzz targets

libFuzzer targets for the inputs `SECURITY.md` names: hostile packages,
request JSON through the C ABI, and overlay sets from other devices; plus route
parity between CCH potentials and plain A*. They run
with [`cargo fuzz`](https://github.com/rust-fuzz/cargo-fuzz) on a nightly
toolchain; this crate is its own workspace and is never published.

| Target | Input | What runs on it |
|---|---|---|
| `package` | A routing package's bytes | `RoutingPackage::from_bytes`, then routes, snaps, a matrix and an isochrone on whatever opens, and routes again after `prepare_potentials`. The root checksum is recomputed after each mutation, as an attacker would, so inputs reach the header, section-table and section checks behind it. |
| `request` | One byte choosing the call, then request JSON | `wayfarer_route`, `_matrix`, `_isochrone`, `_map_match`, `_snap` or `_project` on a Monaco package compiled once per run. Any text a call returns must be JSON. |
| `overlay_set` | Overlay set JSON | `RouteOverlaySet::from_json`; an accepted set must round-trip through `to_json_with_hash`; then `merge`, `workflow_overlays` (bound and unbound) and a route with the overlays on Monaco. |
| `route_parity` | Bytes choosing a profile, two Monaco points, costing options, alternatives and per-edge options on the plain route | The same route on the Monaco package (format 2) bounded by its CCH potentials and by the straight line alone (`internals::set_potentials`). Status, cost and edge path must agree (both are exact); the potentials' via alternatives must be distinct and within the sharing limit. |

```sh
cargo install cargo-fuzz
rustup toolchain install nightly
mkdir -p fuzz/corpus/package fuzz/corpus/request fuzz/corpus/overlay_set
cargo +nightly fuzz run package fuzz/corpus/package fuzz/seeds/package
cargo +nightly fuzz run request fuzz/corpus/request fuzz/seeds/request -- -dict=fuzz/request.dict
cargo +nightly fuzz run overlay_set fuzz/corpus/overlay_set fuzz/seeds/overlay_set -- -dict=fuzz/overlay_set.dict
mkdir -p fuzz/corpus/route_parity
cargo +nightly fuzz run route_parity fuzz/corpus/route_parity
```

Pass `--debug-assertions` (`-a`) to `fuzz run` as well: it turns on
overflow checks, which the release-mode build `cargo fuzz` makes by default
leaves off, so arithmetic on hostile header counts panics instead of wrapping
silently. Embedders that build with overflow checks would hit the same panic.

The nightly workflow (`.github/workflows/nightly.yml`) runs
`scripts/fuzz-nightly.sh`: every target at once, with `--debug-assertions`,
the seeds, dictionaries and a compiled Monaco package seed, for 30 minutes
each, on corpora it keeps in the Actions cache and minimizes after each run.
A crash fails the run and uploads `fuzz/artifacts` and the logs as the
artifact `fuzz-artifacts-<run>`. Locally, `FUZZ_TOOLCHAIN=nightly
scripts/fuzz-nightly.sh 60` runs the same for a minute per target.

Add `--target aarch64-apple-darwin` on Apple silicon if `cargo fuzz` picks
the x86_64 target. Useful libFuzzer options after `--`: `-max_total_time=<s>`,
`-timeout=10` (report inputs slower than 10 s), `-rss_limit_mb=4096`, and
`-jobs=<n>` to run several workers.

`seeds/` holds valid starting inputs, each accepted by its target's code
(the package is `fixtures/routing/short_road.mrt`); new corpus entries go to
`corpus/` and crashes to `artifacts/`, both ignored by git. The dictionaries
list every property name and enum value of `schema/*.json`; regenerate them
when the schemas change. A crash reproduces with
`cargo +nightly fuzz run <target> fuzz/artifacts/<target>/<file>`.

For a richer package seed, compile Monaco into the corpus and raise the input
size limit:

```sh
cargo run --release -p wayfarer-cli -- compile fixtures/monaco.osm.pbf \
  -o fuzz/corpus/package/monaco.mrt --country MC --graph-id monaco
cargo +nightly fuzz run package fuzz/corpus/package fuzz/seeds/package -- -max_len=1100000
```

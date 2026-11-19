# Benchmarks and gates

`gates/gates.sh` compares two builds on a regional extract:

| Command | Checks |
|---|---|
| `gates.sh build <bin_dir>` | Builds the release `wayfarer` binary and the harness examples into `bin_dir` |
| `gates.sh packages <bin_dir> <out_dir>` | Compiles Małopolskie and Monaco; prints sha256, size, time |
| `gates.sh dumps <bin_dir> <pkg_dir> <out_dir>` | Route, parity, matrix and isochrone dumps (deterministic) |
| `gates.sh compare <out_a> <out_b>` | Byte comparison of two dump directories |
| `gates.sh timing <bin_a> <bin_b> <pkg_dir> [runs]` | Alternating warm and cold request timings |
| `gates.sh tests <log>` | `cargo test --workspace` |

Set `CARGO_TARGET_DIR` (outside the repository) and `MALOPOLSKIE_PBF` to the
Geofabrik extract `europe/poland/malopolskie-latest.osm.pbf`. Run timings on a
quiet machine: other builds or simulators skew them by tens of percent.
`malopolskie-corpus.json` and `monaco-corpus.json` are the route corpora;
`normalize.py` drops or renames response keys when a change removes them on
purpose.

## Regional regression gate

`gates/regional.sh <region>` compiles a package from a regional extract,
routes the region's corpus (plain, and again with endpoint hints and
alternatives) and seeded random pairs on it with `route_response_dump`, and
compares every case with `gates/<region>-baseline.json` (`gates/regress.py`).
Unlike `gates.sh compare`, which wants identical dumps, it fails only on

- a case that newly fails, listed by error kind (`NoPath`, `NoCandidate`, …),
- a cost increase beyond the engine's tie window (1e-7 relative,
  `COST_TOLERANCE`),
- visited edges up by more than `VISITED_TOLERANCE` (default 0.25) and
  `VISITED_SLACK` edges (default 500),

and reports the opposite changes as improvements. Corpus cases and random pairs
are reported apart: random points outside the extract give `NoCandidate`, in
the baseline too.

| Region | Extract | Cases | Time | Run by |
|---|---|---|---|---|
| `krakow-centre` | `fixtures/krakow-centre.osm.pbf` (783 kB) | 22 corpus, 60 random | ~3 s | `scripts/check.sh` |
| `malopolskie` | `$MALOPOLSKIE_PBF` | 124 corpus, 200 random | ~1 min | on demand or nightly |

The `malopolskie` baseline is recorded on Geofabrik's
`malopolskie-261118.osm.pbf` (sha256 `777306c5c0bd2898…`, dataset hash
`e1d3a3f9…` with the 0.2 compiler). Map edits move costs and visited edges, so
on another extract the gate reports drift as well as engine changes: compare
two commits on the same extract (`ROUTE_DUMP` and `COMPILER` from
`gates.sh build`), and before re-recording on a new extract, check that the
previous release gives the same costs on it.

`PACKAGE=<routing.mrt>` skips the compile, `ROUTE_DUMP=<route_response_dump>`
(and `COMPILER`) gate binaries built elsewhere, for example by
`gates.sh build <bin_dir>` at another commit, and `UPDATE_BASELINE=1` records
the baseline after an intended change; commit it with the change. A changed
corpus requires a new baseline.

`valhalla/` compares Wayfarer with Valhalla on the same extract: graph
compilation (time, peak memory, size) and routes on the same pairs (path
overlap, length and time ratios, road-type shares, snapping).
`valhalla/bench.sh <region>` runs all of it into one report; see
`valhalla/README.md`.

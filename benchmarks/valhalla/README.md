# Comparison with Valhalla

Routes the same pairs with Wayfarer and with Valhalla (pyvalhalla 3.9, built
from the same extract) and reports where they disagree. Valhalla is the
quality benchmark: it does not decide what is right, but a large, systematic
difference from it is worth explaining.

## One command

```sh
benchmarks/valhalla/bench.sh monaco          # or krakow-centre; both use fixtures
MALOPOLSKIE_PBF=~/.cache/wayfarer-bench/pbf/malopolskie-latest.osm.pbf \
  RUNS=3 benchmarks/valhalla/bench.sh malopolskie
```

`bench.sh` sets up a venv with pyvalhalla (`requirements.txt`; its wheel
carries the `valhalla_build_*` executables, so nothing else to install),
builds Wayfarer in release, then on one extract:

1. **Graph compilation.** Wayfarer's `compile`, then Valhalla's admin database,
   tiles and tar extract (its single-file counterpart of `routing.mrt`; no
   elevation, timezones or traffic), on the same number of threads
   (`THREADS`, default all cores), with the extract already in the page cache.
   `measure.py` records wall time, CPU time and peak resident memory of each
   step. `RUNS=n` repeats the compilation and reports medians.
2. **Route quality.** The scripts below, in order, on the corpus plus seeded
   random pairs.
3. **Report.** `report.py` writes `<BENCH_DIR>/<region>/report.md` and
   `summary.json`: compile table with Wayfarer / Valhalla ratios, route
   outcomes, length and time ratios, path overlap, snap gaps, indicative
   latency, and each script's printed summary.

Everything goes to `BENCH_DIR` (default `~/.cache/wayfarer-bench`), outside
the repository. `SKIP_COMPILE=1` reruns route quality on the previous
compile; `PBF`, `CORPUS` and `COUNTRY` bench another extract. See the header
of `bench.sh` for every option.

## The scripts

| Script | Does |
|---|---|
| `cases.py` | The corpus plus seeded random pairs whose endpoints Valhalla can snap within 30 m (a third of them walks under ~8 km), as corpus-format JSON |
| `valhalla_routes.py` | Routes every case with Valhalla (`auto` or `pedestrian`) |
| `compare.py` | Per case: statuses, length and time ratios, path overlap (share of each route within 25 m of the other) and endpoint snap gaps; a summary and the cases that disagree most |
| `composition.py` | Road-type shares of both engines' routes, both labelled by Valhalla's `trace_attributes` |
| `snapping.py` | How far each engine snaps from the requested points |
| `eta.py` | Valhalla's time on our own route geometry, split into road and junction time |
| `speeds.py` | Our driving speeds against Valhalla's on the same roads, by road type and speed source |
| `measure.py` | Runs one command; records wall time, CPU time and peak memory as JSON |
| `report.py` | `bench.sh`'s report from the compile records and the scripts' outputs |

Run by hand, the scripts need the Python environment pyvalhalla is
installed in (its `valhalla` module), a Valhalla config and tiles of the
extract, and a Wayfarer package of the same extract:

```sh
valhalla_build_config --mjolnir-tile-dir tiles ... > valhalla.json
valhalla_build_tiles -c valhalla.json malopolskie.osm.pbf
python cases.py valhalla.json ../gates/malopolskie-corpus.json cases.json 150
route_response_dump malopolskie.mrt cases.json 0 ours.jsonl 0
python valhalla_routes.py valhalla.json cases.json valhalla.jsonl
python compare.py ours.jsonl valhalla.jsonl compare.csv
python composition.py valhalla.json ours.jsonl valhalla.jsonl compare.csv composition.json
python snapping.py cases.json ours.jsonl valhalla.jsonl
```

(`route_response_dump` is `wayfarer/examples/route_response_dump.rs`;
its `_hinted_alt` copies of each case are ignored.)

Graph compilation, Małopolskie (194 MiB extract), one run on 10 threads
(Apple silicon), Wayfarer 0.1.0 against pyvalhalla 3.9.1 (2026-11-17):

| | Wall | CPU | Peak memory | Output |
|---|---|---|---|---|
| Wayfarer `compile` | 7.1 s | 23.0 s | 1,973 MiB | 259.5 MiB `routing.mrt` |
| Valhalla admins + tiles + tar | 12.4 s | 44.9 s | 2,003 MiB (tiles) | 223.9 MiB (216.7 tar + 7.1 admins) |
| Ratio | 0.57 | 0.51 | 0.98 | 1.16 |

That package (format 1) stored ALT landmark distances, 88.7 MiB of it.
Format 2 replaces them with CCH potentials computed when the package opens
from a stored node order, and Małopolskie's package is now 176 MiB, 0.79×
Valhalla's output (`wayfarer verify` lists the sections).

Route quality, Małopolskie, 212 pairs (147 drives, 65 walks), with junction
delay:

| | Length ratio | Time ratio | Path overlap | Overlap < 0.5 | We snap >20 m farther |
|---|---|---|---|---|---|
| Drives | 0.998 | 0.99 | 0.97 | 9 | 16 of 294 endpoints |
| Walks | 0.999 | 1.01 | 0.97 | 10 | 2 of 130 endpoints |

Medians; ratios are Wayfarer / Valhalla. Before the current costing model walks had
length 1.06, time 1.13 and overlap 0.52 (footways and paths 49% of our walks
against 14% of Valhalla's), with 53 far snaps; drives had overlap 0.93 (23
under 0.5), length p95 1.22 and time 0.92, with 77 far snaps.

Where a road has a posted limit, both engines drive it at that limit. Before
junction delay, our durations left
out the time at junctions, which Valhalla counts (9.5% of drive time in
Kraków centre; on regional drives `eta.py` can't separate it, since
Valhalla's road time there comes out above its total), and the drive time
ratio was 0.94. Now junctions
are 2.9% of our regional drive time and 15.5% in Kraków centre (`eta.py` on
our geometry; we take 8% longer than Valhalla there). The rest of the gap is
tracks, which Valhalla times at about 2 km/h on dirt and we at the compiler's
track speed.
Private and customer roads are destination-only for cars, as in Valhalla
(they were closed to cars, which caused half the far snaps).

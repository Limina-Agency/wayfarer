#!/usr/bin/env bash
# Wayfarer against Valhalla on one extract: graph compilation (wall time, CPU,
# peak memory, output size) and route quality (the comparison scripts in this
# directory), written up in one report.
#
#   benchmarks/valhalla/bench.sh <region>
#
#   monaco          fixtures/monaco.osm.pbf, 30 random pairs        (~1 min)
#   krakow-centre   fixtures/krakow-centre.osm.pbf, 60 random pairs (~1 min)
#   malopolskie     $MALOPOLSKIE_PBF, 150 random pairs              (~15 min)
#
#   BENCH_DIR=<dir>     work directory: venv, packages, tiles, results
#                       (default ~/.cache/wayfarer-bench); each region in <dir>/<region>
#   THREADS=<n>         threads for both compilers (default: all cores)
#   RUNS=<n>            compile each graph n times, report the median (default 1)
#   RANDOM_PAIRS=<n>    random pairs on top of the corpus (default per region)
#   PBF, CORPUS, COUNTRY  another extract: its .osm.pbf, a corpus-format JSON
#                       whose bounding box the random pairs are drawn in, and its
#                       ISO country code (region is then just the output's name)
#   SKIP_COMPILE=1      reuse the region's package and tiles from an earlier run
#   PYTHON=<python>     a Python with pyvalhalla installed, instead of the venv
#
# Valhalla is pyvalhalla (requirements.txt), whose wheel carries the
# valhalla_build_* executables; Valhalla builds its admin database from the
# same extract, then tiles, then a tar extract (its single-file equivalent of
# routing.mrt), with no elevation, timezones or traffic. The report is
# <BENCH_DIR>/<region>/report.md, with summary.json beside it.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
GATES="$ROOT/benchmarks/gates"
region="${1:?usage: bench.sh <monaco|krakow-centre|malopolskie>}"

case "$region" in
  monaco) pbf="$ROOT/fixtures/monaco.osm.pbf" country=MC pairs=30 ;;
  krakow-centre) pbf="$ROOT/fixtures/krakow-centre.osm.pbf" country=PL pairs=60 ;;
  malopolskie) pbf="${MALOPOLSKIE_PBF:-}" country=PL pairs=150 ;;
  *) pbf="" country="" pairs=150 ;;
esac
pbf="${PBF:-$pbf}"
country="${COUNTRY:-$country}"
corpus="${CORPUS:-$GATES/$region-corpus.json}"
pairs="${RANDOM_PAIRS:-$pairs}"
threads="${THREADS:-$(nproc 2>/dev/null || sysctl -n hw.ncpu)}"
runs="${RUNS:-1}"
[[ -n "$pbf" && -f "$pbf" ]] || { echo "no extract for $region: set PBF (or MALOPOLSKIE_PBF)" >&2; exit 2; }
[[ -n "$country" ]] || { echo "set COUNTRY for $region" >&2; exit 2; }
[[ -f "$corpus" ]] || { echo "no corpus $corpus: set CORPUS" >&2; exit 2; }
pbf="$(cd "$(dirname "$pbf")" && pwd)/$(basename "$pbf")"

bench="${BENCH_DIR:-$HOME/.cache/wayfarer-bench}"
work="$bench/$region"
mkdir -p "$work/compile" "$work/quality" "$work/logs"
step() { printf '\n== %s\n' "$*"; }

step "Python environment"
if [[ -n "${PYTHON:-}" ]]; then
  python="$PYTHON"
else
  venv="$bench/venv"
  if [[ ! -x "$venv/bin/python" ]] || ! cmp -s "$HERE/requirements.txt" "$venv/requirements.txt"; then
    rm -rf "$venv"
    if command -v uv >/dev/null; then
      uv venv -q "$venv" && uv pip install -q -p "$venv/bin/python" -r "$HERE/requirements.txt"
    else
      python3 -m venv "$venv" && "$venv/bin/pip" install -q -r "$HERE/requirements.txt"
    fi
    cp "$HERE/requirements.txt" "$venv/requirements.txt"
  fi
  python="$venv/bin/python"
fi
valhalla_bin="$("$python" -c 'from valhalla._scripts import PYVALHALLA_BIN_DIR; print(PYVALHALLA_BIN_DIR)')"
valhalla_version="$("$valhalla_bin/valhalla_build_tiles" --version | awk '{print $NF}')"
echo "pyvalhalla $valhalla_version ($python)"

step "Wayfarer release build"
export RUSTC_WRAPPER=
target="${CARGO_TARGET_DIR:-$ROOT/target}"
(cd "$ROOT" &&
  cargo build --release -q -p wayfarer-cli --bin wayfarer &&
  cargo build --release -q -p wayfarer --example route_response_dump)
compiler="$target/release/wayfarer"
dump="$target/release/examples/route_response_dump"
commit="$(git -C "$ROOT" rev-parse --short HEAD)$(git -C "$ROOT" diff --quiet HEAD || echo -dirty)"
echo "wayfarer $commit"

package="$work/wayfarer/routing.mrt"
tiles="$work/valhalla/tiles"
config="$work/valhalla/valhalla.json"      # builds: logs to the step's log file
query_config="$work/valhalla/query.json"  # route quality: the same, without logging
measure() { "$python" "$HERE/measure.py" "$@"; }

if [[ -z "${SKIP_COMPILE:-}" ]]; then
  rm -rf "$work/compile"/* "$work/wayfarer" "$work/valhalla"
  mkdir -p "$work/wayfarer" "$tiles"
  "$python" -m valhalla.valhalla_build_config \
    --mjolnir-tile-dir "$tiles" --mjolnir-tile-extract "$work/valhalla/tiles.tar" \
    --mjolnir-admin "$work/valhalla/admins.sqlite" --mjolnir-timezone '' \
    --mjolnir-traffic-extract '' --mjolnir-landmarks '' --additional-data-elevation '' \
    --mjolnir-concurrency "$threads" --logging-color false 2>/dev/null >"$config"
  "$python" -c 'import json, sys; c = json.load(open(sys.argv[1])); c["logging"]["type"] = ""; json.dump(c, open(sys.argv[2], "w"), indent=1)' \
    "$config" "$query_config"

  for run in $(seq 1 "$runs"); do
    step "compile $run/$runs on $threads threads: $(basename "$pbf") ($(du -h "$pbf" | cut -f1))"
    cat "$pbf" >/dev/null # both compilers start with the extract in the page cache

    rm -rf "$work/wayfarer/scratch" "$package"
    RAYON_NUM_THREADS="$threads" measure "$work/compile/wayfarer-$run.json" wayfarer \
      --stdout "$work/wayfarer/build.json" --log "$work/logs/wayfarer-build.log" -- \
      "$compiler" compile "$pbf" -o "$package" --country "$country" --graph-id "$region" \
      --scratch-dir "$work/wayfarer/scratch"
    rm -rf "$work/wayfarer/scratch"

    rm -rf "$tiles" "$work/valhalla/tiles.tar" "$work/valhalla/admins.sqlite" && mkdir -p "$tiles"
    measure "$work/compile/valhalla-admins-$run.json" valhalla-admins \
      --log "$work/logs/valhalla-admins.log" -- \
      "$valhalla_bin/valhalla_build_admins" -c "$config" "$pbf"
    measure "$work/compile/valhalla-tiles-$run.json" valhalla-tiles \
      --log "$work/logs/valhalla-tiles.log" -- \
      "$valhalla_bin/valhalla_build_tiles" -c "$config" -j "$threads" "$pbf"
    measure "$work/compile/valhalla-extract-$run.json" valhalla-extract \
      --log "$work/logs/valhalla-extract.log" -- \
      "$python" -m valhalla.valhalla_build_extract -c "$config" -O
  done
  "$compiler" info "$package" >"$work/wayfarer/info.json"
  "$python" - "$work/compile/sizes.json" "$package" "$tiles" "$work/valhalla" <<'EOF'
import json, os, sys
out, package, tiles, valhalla = sys.argv[1:5]
files = [os.path.join(d, f) for d, _, fs in os.walk(tiles) for f in fs if f.endswith(".gph")]
json.dump({
    "wayfarer_package_bytes": os.path.getsize(package),
    "valhalla_tile_count": len(files),
    "valhalla_tiles_bytes": sum(map(os.path.getsize, files)),
    "valhalla_extract_bytes": os.path.getsize(os.path.join(valhalla, "tiles.tar")),
    "valhalla_admins_bytes": os.path.getsize(os.path.join(valhalla, "admins.sqlite")),
}, open(out, "w"), indent=1)
EOF
else
  [[ -f "$package" && -f "$query_config" ]] || { echo "SKIP_COMPILE: no earlier compile in $work" >&2; exit 2; }
  echo "reusing $package and $tiles"
fi

step "route quality: $(basename "$corpus") + $pairs random pairs"
q="$work/quality"
(
  cd "$HERE"
  "$python" cases.py "$query_config" "$corpus" "$q/cases.json" "$pairs"
  "$dump" "$package" "$q/cases.json" 0 "$q/ours.jsonl" 0 | tee "$q/ours.log" | grep '^routes:'
  "$python" valhalla_routes.py "$query_config" "$q/cases.json" "$q/valhalla.jsonl"
  "$python" compare.py "$q/ours.jsonl" "$q/valhalla.jsonl" "$q/compare.csv" | tee "$q/compare.txt"
  "$python" composition.py "$query_config" "$q/ours.jsonl" "$q/valhalla.jsonl" "$q/compare.csv" \
    "$q/composition.json" >"$q/composition.txt"
  "$python" snapping.py "$q/cases.json" "$q/ours.jsonl" "$q/valhalla.jsonl" >"$q/snapping.txt"
  "$python" eta.py "$query_config" "$q/ours.jsonl" "$q/compare.csv" >"$q/eta.txt"
  "$python" speeds.py "$query_config" "$q/ours.jsonl" "$q/compare.csv" >"$q/speeds.txt"
)

step "report"
"$python" "$HERE/report.py" "$work" --region "$region" --pbf "$pbf" --threads "$threads" \
  --wayfarer "$commit" --valhalla "$valhalla_version"
echo "report: $work/report.md"

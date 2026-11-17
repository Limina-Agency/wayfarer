#!/usr/bin/env bash
# Regional regression gate (see benchmarks/README.md): compiles a package from a
# regional extract, routes a corpus and seeded random pairs on it with
# route_response_dump, and compares every case with a checked-in baseline
# (regress.py: newly failing cases by error kind, cost increases, visited-edge
# regressions; improvements are reported and pass).
#
#   regional.sh malopolskie     full tier: $MALOPOLSKIE_PBF, 200 random pairs
#   regional.sh krakow-centre   fast tier: fixtures/krakow-centre.osm.pbf
#                               (scripts/check.sh runs it)
#
#   COMPILER=<path>        `wayfarer` binary to use (default: built in release); an
#                          older `wayfarer-compiler` works too
#   ROUTE_DUMP=<path>      route_response_dump to use (default: built in release),
#                          e.g. from `gates.sh build <bin_dir>` of another commit
#   PACKAGE=<routing.mrt>  route on this package instead of compiling one
#   OUT_DIR=<dir>          keep the package and dump here (default: a temp dir)
#   UPDATE_BASELINE=1      record <region>-baseline.json instead of comparing
#   VISITED_TOLERANCE=0.25, VISITED_SLACK=500, COST_TOLERANCE=1e-7: see regress.py
#
# Uses CARGO_TARGET_DIR when set (default: target/).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HERE="$ROOT/benchmarks/gates"
region="${1:?region: malopolskie or krakow-centre}"
case "$region" in
  malopolskie)
    pbf="${MALOPOLSKIE_PBF:-}" random_pairs=200 ;;
  krakow-centre)
    pbf="$ROOT/fixtures/krakow-centre.osm.pbf" random_pairs=60 ;;
  *) echo "unknown region $region" >&2; exit 2 ;;
esac
corpus="$HERE/$region-corpus.json"
baseline="$HERE/$region-baseline.json"
export RUSTC_WRAPPER=
target="${CARGO_TARGET_DIR:-$ROOT/target}"

if [[ -z "${COMPILER:-}" ]]; then
  (cd "$ROOT" && cargo build --release -q -p wayfarer-cli --bin wayfarer)
fi
if [[ -z "${ROUTE_DUMP:-}" ]]; then
  (cd "$ROOT" && cargo build --release -q -p wayfarer --example route_response_dump)
fi
compiler="${COMPILER:-$target/release/wayfarer}"
dump="${ROUTE_DUMP:-$target/release/examples/route_response_dump}"

if [[ -n "${OUT_DIR:-}" ]]; then
  out="$OUT_DIR" && mkdir -p "$out"
else
  out="$(mktemp -d)" && trap 'rm -rf "$out"' EXIT
fi

package="${PACKAGE:-}"
if [[ -z "$package" ]]; then
  : "${pbf:?set MALOPOLSKIE_PBF to the Malopolskie OSM extract, or PACKAGE to a compiled package}"
  package="$out/routing.mrt"
  started=$SECONDS
  rm -rf "$out/scratch"
  if [[ "$(basename "$compiler")" == wayfarer-compiler* ]]; then
    compile=(build --temp-dir "$out/scratch")
  else
    compile=(compile --scratch-dir "$out/scratch")
  fi
  "$compiler" "${compile[@]}" "$pbf" -o "$package" --country PL --graph-id "$region" \
    >/dev/null 2>"$out/build.log"
  rm -rf "$out/scratch"
  echo "compiled $region in $((SECONDS - started)) s"
fi
dataset_hash="$("$compiler" info "$package" | python3 -c 'import json, sys; print(json.load(sys.stdin)["datasetHash"])')"

started=$SECONDS
"$dump" "$package" "$corpus" "$random_pairs" "$out/routes.jsonl" 0 | grep '^routes:'
echo "routed in $((SECONDS - started)) s"
record=()
[[ -n "${UPDATE_BASELINE:-}" ]] && record=(--record)
python3 "$HERE/regress.py" "$baseline" "$out/routes.jsonl" "$corpus" \
  --dataset-hash "$dataset_hash" --random-pairs "$random_pairs" ${record[@]+"${record[@]}"}

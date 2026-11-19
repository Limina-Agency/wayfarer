#!/usr/bin/env bash
# Release and change gates (see benchmarks/README.md).
#
#   gates.sh build    <bin_dir>                     release `wayfarer` + routing examples
#   gates.sh packages <bin_dir> <out_dir>           G2: compile Malopolskie and Monaco, sha256
#   gates.sh dumps    <bin_dir> <pkg_dir> <out_dir> G3: response, parity and analysis dumps
#   gates.sh compare  <out_a> <out_b>               G3: diff two dump directories
#   gates.sh timing   <bin_a> <bin_b> <pkg_dir> [runs=3]   G4: alternating warm/cold timings
#   gates.sh tests    <log_file>                    G1: workspace tests
#
# Needs CARGO_TARGET_DIR outside the repo; packages and dumps need
# MALOPOLSKIE_PBF (the Geofabrik extract poland/malopolskie-latest.osm.pbf).
# timing routes <pkg_dir>/$REGION/routing.mrt with $REGION-corpus.json
# (REGION=malopolskie by default; krakow-centre for a package compiled from
# fixtures/krakow-centre.osm.pbf). Builds without sccache.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HERE="$ROOT/benchmarks/gates"
RUST="$ROOT"
MONACO_PBF="$RUST/fixtures/monaco.osm.pbf"
REGION="${REGION:-malopolskie}"
case "$REGION" in
  malopolskie) default_cold_case=krakow-tarnow-long-regional ;;
  krakow-centre) default_cold_case=krakow-centre-west-east ;;
  *) echo "unknown REGION $REGION" >&2; exit 2 ;;
esac
COLD_CASE="${COLD_CASE:-$default_cold_case}"
need_malopolskie() {
  : "${MALOPOLSKIE_PBF:?set MALOPOLSKIE_PBF to the Malopolskie OSM extract}"
}
export RUSTC_WRAPPER=
export CARGO_PROFILE_RELEASE_STRIP=none
: "${CARGO_TARGET_DIR:?set CARGO_TARGET_DIR to a directory outside the repo}"

# Linux and macOS: coreutils or shasum, /proc or sysctl.
sha256() {
  if command -v sha256sum >/dev/null; then sha256sum "$1" | cut -d' ' -f1; else shasum -a 256 "$1" | cut -d' ' -f1; fi
}
load_1min() {
  if [[ -r /proc/loadavg ]]; then cut -d' ' -f1 /proc/loadavg; else sysctl -n vm.loadavg | awk '{print $2}'; fi
}

cmd_build() {
  local bin="$1"
  mkdir -p "$bin"
  (cd "$RUST" &&
    cargo build --release -p wayfarer-cli --bin wayfarer &&
    cargo build --release -p wayfarer --examples --all-features)
  cp "$CARGO_TARGET_DIR/release/wayfarer" "$bin/"
  for example in route_response_dump analysis_parity route_parity; do
    [[ -x "$CARGO_TARGET_DIR/release/examples/$example" ]] &&
      cp "$CARGO_TARGET_DIR/release/examples/$example" "$bin/"
  done
  git -C "$ROOT" rev-parse HEAD >"$bin/COMMIT"
  echo "binaries in $bin"
}

compile_package() {
  local bin="$1" out="$2" name="$3" pbf="$4" country="$5"
  local dir="$out/$name"
  rm -rf "$dir" && mkdir -p "$dir/scratch"
  cat >"$dir/request.json" <<EOF
{"inputPbfPath":"$pbf","tempDir":"$dir/scratch","graphId":"$name","countryCode":"$country",
 "buildProfile":"desktop_full","routeArchivePath":"$dir/routing.mrt"}
EOF
  local started=$SECONDS
  # The host protocol's request and events; a bin_dir built before the
  # `wayfarer` binary has the old `wayfarer-compiler` instead.
  if [[ -x "$bin/wayfarer" ]]; then
    "$bin/wayfarer" compile --protocol --request "$dir/request.json" >"$dir/compile.log"
  else
    "$bin/wayfarer-compiler" --request "$dir/request.json" >"$dir/compile.log"
  fi
  rm -rf "$dir/scratch"
  printf '%s %s %s bytes %ss\n' "$name" "$(sha256 "$dir/routing.mrt")" \
    "$(wc -c <"$dir/routing.mrt" | tr -d ' ')" "$((SECONDS - started))"
}

cmd_packages() {
  local bin="$1" out="$2"
  need_malopolskie
  mkdir -p "$out"
  { compile_package "$bin" "$out" malopolskie "$MALOPOLSKIE_PBF" PL
    compile_package "$bin" "$out" monaco "$MONACO_PBF" MC; } | tee "$out/packages.txt"
}

cmd_dumps() {
  local bin="$1" pkg="$2" out="$3"
  local mrt="$pkg/malopolskie/routing.mrt" parity="$bin/route_parity"
  rm -rf "$out" && mkdir -p "$out"
  "$bin/route_response_dump" "$mrt" "$HERE/malopolskie-corpus.json" 200 "$out/routes.jsonl" 2000 >"$out/routes.log"
  "$bin/route_response_dump" "$pkg/monaco/routing.mrt" "$HERE/monaco-corpus.json" 50 "$out/monaco-routes.jsonl" 500 >"$out/monaco-routes.log"
  # Parity harness, package engine only: default options, non-default costing, overlays.
  local skip=(ROUTE_PARITY_SKIP_REFERENCE=1)
  env "${skip[@]}" ROUTE_PARITY_DUMP="$out/parity-default.jsonl" \
    "$parity" "$mrt" "$HERE/malopolskie-corpus.json" 40 1 >"$out/parity-default.log"
  env "${skip[@]}" ROUTE_PARITY_DUMP="$out/parity-costing.jsonl" \
    ROUTE_PARITY_COSTING='{"avoidMotorways":1,"preferPaved":0.5}' ROUTE_PARITY_PROFILES='wheeled=tracked,on_foot=legged' \
    "$parity" "$mrt" "$HERE/malopolskie-corpus.json" 40 1 >"$out/parity-costing.log"
  env "${skip[@]}" ROUTE_PARITY_DUMP="$out/parity-overlay.jsonl" ROUTE_PARITY_OVERLAY=1 \
    "$parity" "$mrt" "$HERE/malopolskie-corpus.json" 40 1 >"$out/parity-overlay.log"
  # Matrices and isochrones, package engine only, plain and with overlays.
  mkdir -p "$out/analysis" "$out/analysis-overlay"
  ANALYSIS_PACKAGE_ONLY=1 ANALYSIS_DUMP="$out/analysis" \
    "$bin/analysis_parity" "$mrt" 1 >"$out/analysis.log"
  ANALYSIS_PACKAGE_ONLY=1 ANALYSIS_OVERLAY=1 ANALYSIS_DUMP="$out/analysis-overlay" \
    "$bin/analysis_parity" "$mrt" 1 >"$out/analysis-overlay.log"
  echo "dumps in $out"
}

cmd_compare() {
  local a="$1" b="$2" status=0
  for file in $(cd "$a" && find . -type f \( -name '*.jsonl' -o -name '*.json' \) | sort); do
    if [[ ! -f "$b/$file" ]]; then echo "missing in $b: $file"; status=1; continue; fi
    if ! cmp -s "$a/$file" "$b/$file"; then
      echo "differs: $file ($(diff "$a/$file" "$b/$file" | grep -c '^<' || true) lines)"
      status=1
    fi
  done
  [[ $status == 0 ]] && echo "identical"
  return $status
}

warm_run() {
  "$1/route_response_dump" "$2/$REGION/routing.mrt" "$HERE/$REGION-corpus.json" 200 |
    sed -nE 's/^routes: .*median request ([0-9.]+) ms, p95 ([0-9.]+) ms/\1 \2/p; s/^correlation: .*median ([0-9.]+) ms/\1/p' |
    paste -sd' ' -
}

cold_run() {
  DUMP_COLD_CASE="$COLD_CASE" "$1/route_response_dump" "$2/$REGION/routing.mrt" "$HERE/$REGION-corpus.json" |
    sed -nE 's/.*first +([0-9.]+) +warm +([0-9.]+).*/\1 \2/p'
}

# Timings are only comparable on a quiet machine: wait until the 1-minute load
# average is below 3 (background compiles and the parallel dump stage skew them),
# then discard one warm-up run per build.
wait_for_quiet() {
  until (( $(load_1min | awk '{printf "%d", $1}') < 3 )); do sleep 15; done
}

cmd_timing() {
  local a="$1" b="$2" pkg="$3" runs="${4:-3}"
  wait_for_quiet
  echo "load: $(load_1min)"
  warm_run "$a" "$pkg" >/dev/null
  warm_run "$b" "$pkg" >/dev/null
  echo "run build  median_ms p95_ms correlation_ms | cold_first_ms cold_warm_ms"
  for run in $(seq 1 "$runs"); do
    for side in a b; do
      local bin="$a"; [[ $side == b ]] && bin="$b"
      echo "$run   $side      $(warm_run "$bin" "$pkg") | $(cold_run "$bin" "$pkg")"
      sleep 20
    done
  done
}

cmd_tests() {
  local log="$1"
  (cd "$RUST" && cargo test --workspace --no-fail-fast) >"$log" 2>&1 || true
  grep -E '^test result:' "$log" | awk '{p+=$4; f+=$6; i+=$8} END {print "passed", p, "failed", f, "ignored", i}'
  grep -E '^test .* FAILED$' "$log" | sort -u || true
}

command="${1:?command}"; shift
"cmd_$command" "$@"

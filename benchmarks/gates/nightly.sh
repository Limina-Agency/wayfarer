#!/usr/bin/env bash
# The gate and timings of the "Gate and fuzz" workflow
# (.github/workflows/nightly.yml, nightly or by hand); runs anywhere
# with binaries from `gates.sh build`:
#
#   nightly.sh <bin_dir> <out_dir> [baseline_bin_dir]
#
# 1. The regional gate: regional.sh $REGION with bin_dir's `wayfarer` and
#    route_response_dump against <region>-baseline.json.
# 2. Latency by class: route_response_dump's phase bench (DUMP_PHASE_BENCH,
#    the minimum and median of PHASE_RUNS runs per corpus case) on the gate's
#    package, summarized per profile and distance bucket (p50/p95 of the
#    per-case medians) by nightly_summary.py.
# 3. With baseline_bin_dir (`gates.sh build` of the last release): gates.sh
#    timing, baseline (a) against this build (b), alternating on the same
#    package. Skipped with a note when the baseline cannot open the package
#    (a newer format).
#
# Writes out_dir/summary.md (Markdown) and out_dir/nightly.json (the numbers),
# with the gate's output, the package and the logs next to them. Exit status
# is the gate's: timings are recorded, never judged here, because shared CI
# runners are too noisy for a latency threshold.
#
#   REGION=krakow-centre   the in-repo fixture (CI); malopolskie needs
#                          MALOPOLSKIE_PBF and runs on cloud machines, not CI
#   PHASE_RUNS=5, TIMING_RUNS=3
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bin="$(cd "${1:?bin_dir}" && pwd)"
mkdir -p "${2:?out_dir}"
out="$(cd "$2" && pwd)"
baseline="${3:-}"
[[ -n "$baseline" ]] && baseline="$(cd "$baseline" && pwd)"
export REGION="${REGION:-krakow-centre}"
corpus="$HERE/$REGION-corpus.json"
case "$REGION" in
  malopolskie) pbf="${MALOPOLSKIE_PBF:?set MALOPOLSKIE_PBF for REGION=malopolskie}" ;;
  krakow-centre) pbf="$HERE/../../fixtures/krakow-centre.osm.pbf" ;;
  *) echo "unknown REGION $REGION" >&2; exit 2 ;;
esac

step() { printf '\n== %s\n' "$*"; }

step "regional gate: $REGION"
gate_status=0
COMPILER="$bin/wayfarer" ROUTE_DUMP="$bin/route_response_dump" OUT_DIR="$out/gate" \
  "$HERE/regional.sh" "$REGION" 2>&1 | tee "$out/gate.txt" || gate_status=$?
mkdir -p "$out/pkg/$REGION"
if [[ -f "$out/gate/routing.mrt" ]]; then
  mv "$out/gate/routing.mrt" "$out/pkg/$REGION/routing.mrt"
fi
package="$out/pkg/$REGION/routing.mrt"

phase_args=()
timing_args=()
if [[ -f "$package" ]]; then
  step "latency by class"
  DUMP_PHASE_BENCH="${PHASE_RUNS:-5}" "$bin/route_response_dump" "$package" "$corpus" \
    | tee "$out/phase.txt"
  phase_args=(--phase "$out/phase.txt")

  if [[ -n "$baseline" ]]; then
    step "timing: $(cat "$baseline/COMMIT" 2>/dev/null || echo baseline) (a) against $(cat "$bin/COMMIT" 2>/dev/null || echo this build) (b)"
    if "$baseline/route_response_dump" "$package" "$corpus" 0 >/dev/null 2>"$out/baseline-open.log"; then
      CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$out/unused-target}" \
        "$HERE/gates.sh" timing "$baseline" "$bin" "$out/pkg" "${TIMING_RUNS:-3}" | tee "$out/timing.txt"
      timing_args=(--timing "$out/timing.txt")
    else
      echo "the baseline build cannot route on this package (see baseline-open.log); timing skipped" |
        tee "$out/timing-skipped.txt"
      timing_args=(--timing-skipped "$out/timing-skipped.txt")
    fi
  fi
fi

step "summary"
python3 "$HERE/nightly_summary.py" --region "$REGION" --corpus "$corpus" --gate "$out/gate.txt" \
  --gate-status "$gate_status" --commit "$(cat "$bin/COMMIT" 2>/dev/null || echo unknown)" \
  --baseline-commit "$(cat "$baseline/COMMIT" 2>/dev/null || true)" \
  --pbf "$pbf" \
  ${phase_args[@]+"${phase_args[@]}"} ${timing_args[@]+"${timing_args[@]}"} \
  --json "$out/nightly.json" >"$out/summary.md"
cat "$out/summary.md"
exit "$gate_status"

#!/usr/bin/env bash
# Time-boxed fuzzing of every target at once, with overflow checks; the
# "Gate and fuzz" workflow runs it (.github/workflows/nightly.yml, nightly or
# by hand) and keeps the corpora between runs.
#
#   scripts/fuzz-nightly.sh [seconds_per_target=1800] [target ...]
#
# Builds the targets with `cargo fuzz build -O --debug-assertions` on
# $FUZZ_TOOLCHAIN (default: nightly), adds a compiled Monaco package to the
# `package` corpus, then runs every target in parallel (one libFuzzer process
# each, so give it as many cores as targets) for the given time, on
# fuzz/corpus/<target> plus its seeds and dictionary. Crashes, timeouts and
# out-of-memory inputs go to fuzz/artifacts/<target>/, logs to
# fuzz/logs/<target>.log. Afterwards each corpus is minimized (`cargo fuzz
# cmin`). Exit status: 0 nothing found, 1 a target failed (see its log and
# artifacts).
#
# Needs `cargo install cargo-fuzz` and the toolchain; uses CARGO_TARGET_DIR for
# the Monaco compile (default target/).
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
export RUSTC_WRAPPER=
seconds="${1:-1800}"
shift || true
toolchain="${FUZZ_TOOLCHAIN:-nightly}"
fuzz=(cargo "+$toolchain" fuzz)
read -r -a targets <<<"${*:-$("${fuzz[@]}" list | tr '\n' ' ')}"
triple="$(rustc "+$toolchain" -vV | sed -n 's/^host: //p')"
mkdir -p fuzz/logs

echo "== build (${targets[*]})"
for target in "${targets[@]}"; do
  "${fuzz[@]}" build -O --debug-assertions "$target"
done

if [[ " ${targets[*]} " == *" package "* ]]; then
  echo "== Monaco package seed"
  mkdir -p fuzz/corpus/package
  cargo run --release -q -p wayfarer-cli -- compile fixtures/monaco.osm.pbf \
    -o fuzz/corpus/package/monaco.mrt --country MC --graph-id monaco >/dev/null
fi

echo "== fuzz ${seconds} s per target"
pids=()
for target in "${targets[@]}"; do
  mkdir -p "fuzz/corpus/$target" "fuzz/artifacts/$target"
  args=("fuzz/corpus/$target")
  [[ -d "fuzz/seeds/$target" ]] && args+=("fuzz/seeds/$target")
  args+=(-max_total_time="$seconds" -timeout=30 -rss_limit_mb=4096 -print_final_stats=1
    -artifact_prefix="fuzz/artifacts/$target/")
  [[ -f "fuzz/$target.dict" ]] && args+=(-dict="fuzz/$target.dict")
  # The Monaco package is ~650 kB; libFuzzer's default limit is 4 kB.
  [[ "$target" == package ]] && args+=(-max_len=1100000)
  "fuzz/target/$triple/release/$target" "${args[@]}" >"fuzz/logs/$target.log" 2>&1 &
  pids+=($!)
done

failed=()
for index in "${!targets[@]}"; do
  target="${targets[$index]}"
  if wait "${pids[$index]}"; then
    echo "$target: no crash ($(ls "fuzz/corpus/$target" | wc -l) corpus inputs," \
      "$(sed -n 's/^stat::number_of_executed_units: *//p' "fuzz/logs/$target.log" | tail -n1) executions)"
  else
    failed+=("$target")
    echo "$target: FAILED; the end of fuzz/logs/$target.log:"
    tail -n 60 "fuzz/logs/$target.log"
    ls -l "fuzz/artifacts/$target"
  fi
done

echo "== minimize corpora"
for target in "${targets[@]}"; do
  before="$(ls "fuzz/corpus/$target" | wc -l)"
  if "${fuzz[@]}" cmin -O --debug-assertions "$target" "fuzz/corpus/$target" -- -timeout=30 \
    >"fuzz/logs/$target-cmin.log" 2>&1; then
    echo "$target: $before -> $(ls "fuzz/corpus/$target" | wc -l) inputs"
  else
    echo "$target: cmin failed (fuzz/logs/$target-cmin.log); corpus kept as is"
  fi
done

if ((${#failed[@]})); then
  echo "error: ${failed[*]} failed; reproduce with" \
    "cargo +nightly fuzz run -O -a <target> fuzz/artifacts/<target>/<file>" >&2
  exit 1
fi

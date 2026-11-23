#!/usr/bin/env bash
# The CI checks, run locally: formatting, clippy, rustdoc warnings, tests, then
# the Monaco package compiled, verified and described with the release
# `wayfarer` binary, a route and
# cancelled routes through the C ABI from the C examples, and the Kraków centre
# regression gate.
#
#   scripts/check.sh
#
# The Monaco package's description must match fixtures/monaco-info.json,
# including its byte length and dataset hash: packages are the same bytes on
# every platform. After an intended change, record it with
#
#   UPDATE_MONACO_INFO=1 scripts/check.sh
#
# The gate (benchmarks/gates/regional.sh krakow-centre) routes a corpus on a
# package compiled from fixtures/krakow-centre.osm.pbf and compares every case
# with benchmarks/gates/krakow-centre-baseline.json. After an intended change,
# record it with
#
#   UPDATE_BASELINE=1 benchmarks/gates/regional.sh krakow-centre
#
# Uses CARGO_TARGET_DIR when set (default: target/). Builds without a rustc
# wrapper: sccache has produced corrupt proc-macro dylibs in release builds.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
export RUSTC_WRAPPER=
target="${CARGO_TARGET_DIR:-target}"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

step() { printf '\n== %s\n' "$*"; }

step "fmt"
cargo fmt --all --check

step "clippy"
cargo clippy --workspace --all-targets -- -D warnings
cargo clippy -p wayfarer-ffi --features compiler --all-targets -- -D warnings

step "rustdoc"
RUSTDOCFLAGS=-Dwarnings cargo doc --workspace --no-deps --all-features

step "tests"
cargo test --workspace
# The C library's compile calls (its `compiler` build).
cargo test -p wayfarer-ffi --features compiler --test compile

step "release builds"
cargo build --release -p wayfarer-cli -p wayfarer-ffi

step "Monaco package: build, verify, info"
wayfarer="$target/release/wayfarer"
"$wayfarer" compile fixtures/monaco.osm.pbf -o "$work/monaco.mrt" --country MC --graph-id monaco \
  >"$work/build.json" 2>"$work/build.log"
"$wayfarer" verify "$work/monaco.mrt" >/dev/null
"$wayfarer" info "$work/monaco.mrt" | python3 -c '
import json, sys
print(json.dumps(json.load(sys.stdin), indent=2, sort_keys=True, ensure_ascii=False))
' >"$work/info.json"
if [[ -n "${UPDATE_MONACO_INFO:-}" ]]; then
  cp "$work/info.json" fixtures/monaco-info.json
fi
if ! diff -u fixtures/monaco-info.json "$work/info.json"; then
  echo "error: the Monaco package differs from fixtures/monaco-info.json; if the" \
    "change is intended, rerun with UPDATE_MONACO_INFO=1 and commit the file" >&2
  exit 1
fi

step "route, matrix, nearest and isochrone through the CLI"
"$wayfarer" route "$work/monaco.mrt" --from 43.7397,7.4275 --via 43.7350,7.4210 \
  --to 43.7290,7.4160 --geojson >/dev/null
"$wayfarer" matrix "$work/monaco.mrt" --source 43.7397,7.4275 --target 43.7290,7.4160 >/dev/null
"$wayfarer" nearest "$work/monaco.mrt" --source 43.7397,7.4275 --target 43.7290,7.4160 \
  --target 43.7350,7.4210 -k 1 >/dev/null
"$wayfarer" isochrone "$work/monaco.mrt" --from 43.7397,7.4275 --minutes 5 \
  --distance-m 1000 >/dev/null

step "route and cancel through the C ABI"
case "$(uname -s)" in
  Darwin) libs=(-framework CoreFoundation) ;;
  *) libs=(-lpthread -ldl -lm) ;;
esac
for example in route cancel; do
  cc -std=c11 -Wall -Wextra -Werror "wayfarer-ffi/examples/$example.c" \
    -Iwayfarer-ffi/include "$target/release/libwayfarer_ffi.a" "${libs[@]}" \
    -o "$work/$example"
  "$work/$example" "$work/monaco.mrt" 43.7397 7.4275 43.7290 7.4160 >/dev/null
done

step "Kraków centre regression gate"
COMPILER="$wayfarer" benchmarks/gates/regional.sh krakow-centre

step "all checks passed"

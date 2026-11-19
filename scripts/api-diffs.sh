#!/usr/bin/env bash
# What a change does to the public surface, as Markdown on stdout. CI writes it
# to the job summary of every pull request (.github/workflows/api-diffs.yml).
#
#   scripts/api-diffs.sh semver  [base_rev]   cargo semver-checks of every
#                                             published library crate
#   scripts/api-diffs.sh surface [base_rev]   JSON Schemas, C header and the
#                                             pinned C ABI response keys
#
# base_rev defaults to the last `wayfarer-v*` tag; `surface` also shows the
# changes since that tag when base_rev is something else. Both are
# informational: they exit 0 whatever they find (a breaking change is allowed
# in 0.x, it just has to be seen), and under GitHub Actions they add a warning
# annotation when something changed. They exit non-zero only when a tool
# cannot run. `cargo install cargo-semver-checks --locked` for `semver`.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
export RUSTC_WRAPPER=
command="${1:?semver or surface}"
last_release="$(git tag --list 'wayfarer-v*' --sort=-v:refname | head -n1)"
base="${2:-$last_release}"
: "${base:?no base revision and no wayfarer-v* tag (fetch tags)}"
base_label="$base"
[[ "$base" =~ ^[0-9a-f]{40}$ ]] && base_label="$(git rev-parse --short "$base")"

warn() {
  if [[ -n "${GITHUB_ACTIONS:-}" ]]; then echo "::warning title=$1::$2" >&2; fi
}

details() { # <summary> <file> <language>
  echo "<details><summary>$1</summary>"
  echo
  echo '```'"$3"
  cat "$2"
  echo '```'
  echo
  echo "</details>"
}

cmd_semver() {
  local out status=0
  out="$(mktemp)"
  # Patch-level rules: a 0.x minor release may break the API, but every break
  # should be on purpose and named in the changelog, so all of them are shown.
  cargo semver-checks --workspace --baseline-rev "$base" --release-type patch \
    --color never >"$out" 2>&1 || status=$?
  echo "## Semver checks against $base_label"
  echo
  if [[ $status == 0 ]]; then
    echo "No breaking change in the published library crates."
  elif grep -qE '^\s*(--- failure|Summary semver requires)' "$out"; then
    echo "**Breaking changes** (allowed in 0.x; name each one in the PR's changelog paragraph):"
    echo
    # One line per failed lint, then the items it failed in (paths without
    # the checkout prefix semver-checks prints).
    awk '
      /^--- failure / { sub(/^--- failure /, ""); sub(/ ---$/, ""); printf "- **%s**\n", $0; items = 0; next }
      /^Failed in:/ { items = 1; next }
      items && /^  [^ ]/ { sub(/^  /, ""); sub(/, previously in file .*/, ""); printf "  - `%s`\n", $0; next }
      items && /^ *$/ { items = 0 }
    ' "$out"
    warn "Breaking API change" "cargo semver-checks reports breaking changes against $base_label; see the job summary"
  else
    echo "cargo semver-checks did not run (exit $status); see the log below."
    echo
    details "cargo semver-checks output" "$out" text
    rm -f "$out"
    return "$status"
  fi
  echo
  details "cargo semver-checks output" "$out" text
  rm -f "$out"
}

surface_paths=(schema wayfarer-ffi/include wayfarer-ffi/tests/abi-keys.json)

surface_against() { # <rev> <label>
  local rev="$1" label="$2" stat diff
  stat="$(git diff --stat "$rev" -- "${surface_paths[@]}")"
  echo "### Against $label"
  echo
  if [[ -z "$stat" ]]; then
    echo "No change to the JSON Schemas, the C header or the C ABI response keys."
    echo
    return 1
  fi
  echo '```text'
  echo "$stat"
  echo '```'
  echo
  diff="$(mktemp)"
  git diff "$rev" -- "${surface_paths[@]}" >"$diff"
  details "Full diff against $label" "$diff" diff
  rm -f "$diff"
}

cmd_surface() {
  echo "## JSON Schemas and C header"
  echo
  if surface_against "$base" "$base_label"; then
    warn "Public surface changed" "schema/, wayfarer.h or the C ABI response keys differ from $base_label; see the job summary"
  fi
  if [[ "$base" != "$last_release" && -n "$last_release" ]]; then
    surface_against "$last_release" "the last release, $last_release" || true
  fi
}

"cmd_$command"

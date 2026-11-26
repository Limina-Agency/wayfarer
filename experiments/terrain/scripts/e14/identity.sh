#!/bin/bash
# E14: byte identity of incident surfaces, deltas, layer files and routes across targets, threads and builds.
#   identity.sh <prep-dir> <pace-csv> <work-dir>
set -u
P=$1; C=$2; W=$3; R=$(cd "$(dirname "$0")/../.." && pwd)/target
declare -A RUN=(
 [x86_64-gnu-t4]="$R/release/terrexp --threads 4"
 [x86_64-gnu-t1]="$R/release/terrexp --threads 1"
 [x86_64-gnu-debug]="$R/debug/terrexp --threads 4"
 [x86_64-musl]="$R/x86_64-unknown-linux-musl/release/terrexp --threads 3"
 [i686-musl]="$R/i686-unknown-linux-musl/release/terrexp --threads 2"
 [aarch64-musl-qemu]="qemu-aarch64-static $R/aarch64-unknown-linux-musl/release/terrexp --threads 4"
)
mkdir -p $W
for k in $(printf '%s\n' "${!RUN[@]}" | sort); do
  rm -rf $W/$k; mkdir -p $W/$k
  ${RUN[$k]} e14 $P $C $W/$k/out > $W/$k/e14.json 2> $W/$k/err.txt || echo "FAIL $k"
  (grep -oE '"h_[a-z_0-9A-Z]+": "[0-9a-f]+"' $W/$k/e14.json; grep -oE '"checks": \{[^}]*\}' $W/$k/e14.json; cd $W/$k/out && sha256sum *.wtl *.wis *.wisd fused_sample.bin) > $W/$k.sums
  echo "$k $(sha256sum < $W/$k.sums | cut -c1-16) lines=$(wc -l < $W/$k.sums)"
done

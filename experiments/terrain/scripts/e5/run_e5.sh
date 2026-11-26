#!/bin/bash
# E5 timing matrix and byte identity. Needs prep_e5.sh output in $W.
set -eu
R=/home/user/wayfarer/experiments/terrain/target
W=${W:-/home/user/e5work}; CSV=${CSV:-/mnt/project-files/off-road-sar/exp/E4/pace_table_50.csv}
cd $W
for area in tatra gorce; do
  for st in 120 240; do
    for th in 1 4; do
      $R/release/terrexp surface $area/$st/prep $st $CSV out/${area}_${st}_t$th --threads $th --queries 200 > json/surface_${area}_${st}_t$th.json
    done
    # byte identity: same 50 queries on every build
    for b in release/terrexp debug/terrexp x86_64-unknown-linux-musl/release/terrexp i686-unknown-linux-musl/release/terrexp; do
      tag=$(echo $b | tr '/' '_')
      $R/$b surface $area/$st/prep $st $CSV out/id_${area}_${st}_$tag --threads 2 --queries 50 > json/id_${area}_${st}_$tag.json
    done
    qemu-aarch64-static $R/aarch64-unknown-linux-musl/release/terrexp surface $area/$st/prep $st $CSV out/id_${area}_${st}_aarch64 --threads 2 --queries 50 > json/id_${area}_${st}_aarch64.json
    echo "$area $st done"
  done
done

#!/bin/bash
# Byte identity of E1 outputs across targets, thread counts and debug/release.
set -u
R=/home/user/wayfarer/experiments/terrain/target
W=/home/user/e1work; T=/home/user/tiles; O=$W/ident; rm -rf $O; mkdir -p $O
declare -A RUN=(
 [x86_64-gnu-t4]="$R/release/terrexp --threads 4"
 [x86_64-gnu-t1]="$R/release/terrexp --threads 1"
 [x86_64-gnu-debug]="$R/debug/terrexp --threads 4"
 [x86_64-musl]="$R/x86_64-unknown-linux-musl/release/terrexp --threads 3"
 [i686-musl]="$R/i686-unknown-linux-musl/release/terrexp --threads 2"
 [aarch64-musl-qemu]="qemu-aarch64-static $R/aarch64-unknown-linux-musl/release/terrexp --threads 4"
)
for k in "${!RUN[@]}"; do
  for job in "tatra_podhale 120" "tatra_podhale 360" "ojcow 240"; do
    set -- $job
    ${RUN[$k]} dem $T/glo30 $W/blocks/blocks_$1_$2.txt $2 $O/$k/dem_$1_$2 --steep > /dev/null || echo FAIL $k dem $job
    ${RUN[$k]} lc $T/worldcover $W/blocks/blocks_$1_$2.txt $2 $O/$k/lc_$1_$2 > /dev/null || echo FAIL $k lc $job
  done
  ${RUN[$k]} dem $T/srtm $W/blocks/blocks_gorce_120.txt 120 $O/$k/srtm_gorce_120 --steep > /dev/null || echo FAIL $k srtm
  ${RUN[$k]} dem $T/nasadem $W/blocks/blocks_gorce_120.txt 120 $O/$k/nasadem_gorce_120 --steep > /dev/null || echo FAIL $k nasadem
done
cd $O && for k in */; do k=${k%/}; (cd $k && find . -type f \( -name '*.wte' -o -name '*.i32' -o -name '*.u8' \) | sort | xargs b3sum 2>/dev/null || find . -type f \( -name '*.wte' -o -name '*.i32' -o -name '*.u8' \) | sort | xargs sha256sum) > $k.sums; done
for k in *.sums; do echo "$k $(sha256sum < $k | cut -c1-16) files=$(wc -l < $k)"; done

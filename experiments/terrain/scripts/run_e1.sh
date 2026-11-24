#!/bin/bash
# E1 matrix: DEM sources x spacing x areas, land cover x spacing x areas.
set -u
B=${B:-/home/user/wayfarer/experiments/terrain/target/release/terrexp}
W=/home/user/e1work; T=/home/user/tiles
mkdir -p $W/json
cd $W
for area in malopolskie tatra_podhale gorce niepolomice ojcow; do
  for src in glo30 srtm nasadem; do
    for st in 120 240 360; do
      $B dem $T/$src blocks/blocks_${area}_$st.txt $st out/${area}_${src}_$st --steep > json/dem_${area}_${src}_$st.json || echo FAIL $area $src $st
    done
  done
  $B dem $T/glo30 blocks/blocks_${area}_360.txt 360 out/${area}_glo30_360p --point > json/dempoint_${area}_glo30_360.json || echo FAIL point $area
  [ $area != malopolskie ] && { $B dem $T/glo30 blocks/blocks_${area}_60.txt 60 out/${area}_glo30_60 > json/dem_${area}_glo30_60.json || echo FAIL half $area; }
  for st in 120 240 360; do
    $B lc $T/worldcover blocks/blocks_${area}_$st.txt $st out/lc_${area}_$st > json/lc_${area}_$st.json || echo FAIL lc $area $st
  done
done
echo ALLDONE

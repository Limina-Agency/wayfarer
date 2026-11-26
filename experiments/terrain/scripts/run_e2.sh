#!/bin/bash
# E2: DEM sources against GUGiK 1 m LiDAR on five boxes in Malopolska.
# Needs the E1 posts at 1" (run_e1.sh) and the tiles from off-road-sar/data/*.tar extracted to $T.
# GUGiK WCS is slow (minutes per 1 km chunk); fetch_gugik.py resumes from cached chunks.
set -eu
B=${B:-/home/user/wayfarer/experiments/terrain/target/release/terrexp}
W=/home/user/e1work; T=/home/user/tiles; S=$(cd "$(dirname "$0")" && pwd)
cd $W
# Babia Gora is not in the E1 matrix: make its 1" posts here (blocks.py writes blocks_babia_120.txt).
for src in glo30 srtm nasadem; do
  $B dem $T/$src blocks/blocks_babia_120.txt 120 out/babia_${src}_120 > json/dem_babia_${src}_120.json
done
#   box          lat0   lon0   lat1   lon1   E1 area holding the 1" posts
while read -r box la0 lo0 la1 lo1 area; do
  python3 -I $S/fetch_gugik.py $box $la0 $lo0 $la1 $lo1
  (cd $S && python3 gugik_lattice.py $box $la0 $lo0 $la1 $lo1 $area > $W/gugik_$box.json 2> $W/gugik_$box.err)
done <<'BOXES'
tatra2      49.237 19.975 49.259 20.008 tatra_podhale
gorce       49.535 20.095 49.555 20.125 gorce
ojcow       50.195 19.815 50.215 19.845 ojcow
niepolomice 50.045 20.330 50.065 20.360 niepolomice
babia       49.565 19.515 49.585 19.545 babia
BOXES
echo ALLDONE

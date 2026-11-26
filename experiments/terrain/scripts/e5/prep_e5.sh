#!/bin/bash
# E5 stage-1 prep for the two 20 x 20 km areas at 1" and 2".
set -eu
B=${B:-/home/user/wayfarer/experiments/terrain/target/release/terrexp}
T=/home/user/tiles; PBF=${PBF:-/mnt/project-files/malopolskie-261118.osm.pbf}
S=$(cd "$(dirname "$0")" && pwd); W=${W:-/home/user/e5work}
#   area  lat0  lon0   lat1  lon1
while read -r area la0 lo0 la1 lo1; do
  for st in 120 240; do
    d=$W/$area/$st; mkdir -p $d
    python3 -c "
import sys; sys.path.insert(0, '$S/..')
from blocks import blocks_for; from shapely.geometry import box
for bi, bj in blocks_for(box($lo0, $la0, $lo1, $la1), $st): print(bi, bj)" > $d/blocks.txt
    $B dem $T/glo30 $d/blocks.txt $st $d/glo30 --steep > $d/glo30.json
    $B lc $T/worldcover $d/blocks.txt $st $d/wc > $d/wc.json
    $B osm $PBF $d/blocks.txt $st $d/osm > $d/osm.json
    $B lines $PBF $d/blocks.txt $st $d/lines > $d/lines.json
    python3 $S/gem15_lattice.py $d/blocks.txt $st $T/gem15 $d/glo30 $d/gem15 > /dev/null
    python3 $S/fuse_e5.py $d $d/prep > /dev/null
    echo "$area $st done"
  done
done <<'AREAS'
tatra 49.20 19.84 49.38 20.12
gorce 49.46 20.01 49.64 20.29
AREAS

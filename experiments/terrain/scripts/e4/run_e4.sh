#!/bin/bash
# E4 pipeline (lane TERRX): pace models against OSM public GPS traces.
# Needs: python3 with numpy scipy rasterio osmium pyproj shapely; terrexp built (cargo build --release).
# RAW = raw trackpoint pages (contain trace names and user URLs: keep out of shared folders).
set -eu
H=$(cd "$(dirname "$0")" && pwd); B=$H/../../target/release/terrexp
RAW=${RAW:?}; W=${W:-/home/user/e4work}; T=${T:?tiles dir with glo30/ srtm/ worldcover/}; G=${G:?GUGiK tiles dir}
OSM=${OSM:?dir with malopolskie-261118.osm.pbf presovsky.osm.pbf zilinsky.osm.pbf presovsky.poly zilinsky.poly malopolskie.wkt}
AREAS="tatra_podhale gorce niepolomice ojcow babia_gora pieniny_sadecki"
python3 $H/fetch_traces.py $RAW tatra_podhale gorce niepolomice ojcow babia_gora pieniny_sadecki  # MAXPAGES=200 for tatra
mkdir -p $W/blocks && cd $W
python3 $H/../blocks.py $OSM/malopolskie.wkt blocks   # four E1 areas; the two E4-only areas: see README (blocks_for on the box)
for a in $AREAS; do
  $B dem $T/glo30 blocks/blocks_${a}_120.txt 120 out/${a}_glo30_120 >/dev/null
  $B dem $T/srtm  blocks/blocks_${a}_120.txt 120 out/${a}_srtm_120 >/dev/null
  $B dem $T/glo30 blocks/blocks_${a}_360.txt 360 out/${a}_glo30_360 >/dev/null
  $B lc  $T/worldcover blocks/blocks_${a}_120.txt 120 out/lc_${a}_120 >/dev/null
done
python3 $H/coverage.py $OSM/malopolskie.wkt $OSM/presovsky.poly $OSM/zilinsky.poly > coverage.wkt
python3 $H/osm_ways.py $OSM/malopolskie-261118.osm.pbf $OSM/presovsky.osm.pbf $OSM/zilinsky.osm.pbf ways.npz
for a in $AREAS; do for L in 50 100; do python3 $H/segments.py $RAW . $a $L; done; done | tee segstats.txt
python3 $H/lidar.py . $G lidar.json
for L in 50 100; do E4_BOOT=60 PYTHONHASHSEED=0 python3 $H/fit.py . res $L; done

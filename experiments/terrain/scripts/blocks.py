# Block lists on the format-3 lattice. Spacing in ticks of 1/120 arc-second.
import sys, math
from shapely import wkt as W
from shapely.geometry import box
from shapely.prepared import prep
TICK_PER_DEG = 3600*120
AREAS = {  # name: (lat0, lon0, lat1, lon1)
 'tatra_podhale': (49.15, 19.70, 49.45, 20.25),
 'gorce':         (49.48, 20.00, 49.64, 20.32),
 'niepolomice':   (49.98, 20.25, 50.13, 20.50),
 'ojcow':         (50.17, 19.76, 50.28, 19.88),
}
def blocks_for(geom, st):
    pg = prep(geom)
    minx, miny, maxx, maxy = geom.bounds
    deg = st/TICK_PER_DEG*64
    out = []
    for bi in range(math.floor(miny/deg)-1, math.floor(maxy/deg)+2):
        for bj in range(math.floor(minx/deg)-1, math.floor(maxx/deg)+2):
            # block covers posts bi*64..bi*64+63; their cells extend half a spacing out
            h = st/TICK_PER_DEG/2
            b = box(bj*deg - h, bi*deg - h, bj*deg + 63*st/TICK_PER_DEG + h, bi*deg + 63*st/TICK_PER_DEG + h)
            if pg.intersects(b): out.append((bi, bj))
    return out
if __name__ == '__main__':
    poly = W.loads(open(sys.argv[1]).read())
    outdir = sys.argv[2]
    print('malopolskie area km2 approx', poly.area * 111.32**2 * math.cos(math.radians(49.8)))
    for st in (60, 120, 240, 360):
        for name, g in [('malopolskie', poly)] + [(k, box(v[1], v[0], v[3], v[2])) for k, v in AREAS.items()]:
            if st == 60 and name == 'malopolskie': pass
            bl = blocks_for(g, st)
            with open(f'{outdir}/blocks_{name}_{st}.txt', 'w') as f:
                for bi, bj in bl: f.write(f'{bi} {bj}\n')
            print(name, st, len(bl))

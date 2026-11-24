# E4: union of the OSM extract coverages used for trail matching: Małopolskie admin boundary (E1 wkt) and the
# openstreetmap.fr clipping polygons of the Slovak regions.   coverage.py <malopolskie.wkt> <x.poly>... > coverage.wkt
import sys
from shapely import wkt
from shapely.geometry import Polygon
from shapely.ops import unary_union
def read_poly(fn):
    rings, cur = [], None
    for line in open(fn).read().split('\n')[1:]:
        t = line.strip()
        if not t: continue
        if t == 'END':
            if cur is not None: rings.append((name, cur)); cur = None
            continue
        if cur is None: name, cur = t, []; continue
        x, y = map(float, t.split()[:2]); cur.append((x, y))
    outer = [Polygon(r) for n, r in rings if not n.startswith('!')]
    holes = [Polygon(r) for n, r in rings if n.startswith('!')]
    return unary_union(outer).difference(unary_union(holes)) if holes else unary_union(outer)
g = [wkt.loads(open(sys.argv[1]).read())] + [read_poly(f) for f in sys.argv[2:]]
print(unary_union(g).wkt)

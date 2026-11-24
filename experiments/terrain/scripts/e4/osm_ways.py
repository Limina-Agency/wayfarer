# E4: highway ways in the test areas from OSM extracts (osm_ways.py <pbf>... <out.npz>), densified to <=5 m vertices in UTM 34N.
# Output per area: npz with x, y (float64 m), cls (u8) per vertex. Classes below.
import sys, numpy as np, osmium, pyproj
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from fetch_traces import AREAS
PAVED = {'motorway','trunk','primary','secondary','tertiary','unclassified','residential','service','living_street',
         'motorway_link','trunk_link','primary_link','secondary_link','tertiary_link','pedestrian','road','busway'}
# 1 paved road, 2 track, 3 path/footway easy (no sac_scale or hiking), 4 mountain path (sac_scale mountain_hiking),
# 5 alpine path (demanding_mountain_hiking and above), 6 steps, 7 other (cycleway, bridleway, via_ferrata, construction)
def cls_of(t):
    h = t.get('highway')
    if h is None: return 0
    if h in PAVED: return 1
    if h == 'track': return 2
    if h == 'steps': return 6
    if h in ('path', 'footway'):
        s = t.get('sac_scale', '')
        if s in ('mountain_hiking',): return 4
        if s in ('demanding_mountain_hiking','alpine_hiking','demanding_alpine_hiking','difficult_alpine_hiking'): return 5
        return 3
    if h in ('cycleway', 'bridleway', 'via_ferrata'): return 7
    return 0
tr = pyproj.Transformer.from_crs(4326, 32634, always_xy=True)
la0 = min(a[0] for a in AREAS.values()) - 0.02; lo0 = min(a[1] for a in AREAS.values()) - 0.03
la1 = max(a[2] for a in AREAS.values()) + 0.02; lo1 = max(a[3] for a in AREAS.values()) + 0.03
class H(osmium.SimpleHandler):
    def __init__(s): super().__init__(); s.out = []
    def way(s, w):
        c = cls_of(w.tags)
        if not c: return
        try: pts = [(n.lon, n.lat) for n in w.nodes]
        except osmium.InvalidLocationError: return
        if not any(lo0 <= x <= lo1 and la0 <= y <= la1 for x, y in pts): return
        s.out.append((c, np.array(pts)))
h = H()
for f in sys.argv[1:-1]:
    h.apply_file(f, locations=True)
X = []; Y = []; C = []
for c, p in h.out:
    x, y = tr.transform(p[:, 0], p[:, 1])
    for i in range(len(x) - 1):
        d = np.hypot(x[i+1]-x[i], y[i+1]-y[i]); n = max(1, int(np.ceil(d / 5.0)))
        t = np.arange(n) / n
        X.append(x[i] + t*(x[i+1]-x[i])); Y.append(y[i] + t*(y[i+1]-y[i])); C.append(np.full(n, c, np.uint8))
    X.append(x[-1:]); Y.append(y[-1:]); C.append(np.array([c], np.uint8))
np.savez_compressed(sys.argv[-1], x=np.concatenate(X), y=np.concatenate(Y), cls=np.concatenate(C))
print(len(h.out), 'ways', sum(len(a) for a in X), 'vertices')

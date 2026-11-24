# Independent check of `terrexp osm`: libosmium area assembly (pyosmium) + shapely, smallest
# area on top, 4x4 samples per post, majority over covered samples with the same tie priority.
import sys, random, numpy as np, osmium, shapely, shapely.wkb
from shapely.strtree import STRtree
pbf, outdir, st, nposts = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
TPD = 432000
PRI = [0,12,20,13,9,8,4,5,10,2,3,3,6,7,30,15,6,25,24,18,23,26,31,22,21,22,19,4,6,5,5,7]
def wet(v): return {'swamp':2,'mangrove':2,'reedbed':24,'bog':25,'string_bog':25,'fen':25,'wet_meadow':26}.get(v,23)
def classify(t):
    g = t.get
    if g('area') == 'no': return 0
    if g('natural') in ('water','bay') or g('waterway') == 'riverbank' or g('landuse') in ('reservoir','basin'): return 22
    if g('natural') == 'wetland': return wet(g('wetland',''))
    c = {'wood':1,'scrub':3,'shrubbery':3,'heath':4,'grassland':7,'fell':8,'tundra':8,'bare_rock':17,'rock':17,'cliff':17,'stone':17,
         'scree':18,'shingle':18,'sand':19,'beach':19,'dune':19,'glacier':21}.get(g('natural'),0)
    if c: return c
    c = {'trees':1,'wood':1,'scrub':3,'grass':6,'gravel':19,'sand':19,'rock':17}.get(g('landcover'),0)
    if c: return c
    if g('man_made') == 'clearcut': return 5
    lu = {'forest':1,'logging':5,'grass':6,'village_green':6,'recreation_ground':6,'flowerbed':6,'greenfield':6,'meadow':7,
          'farmland':9,'plant_nursery':9,'orchard':10,'vineyard':10,'allotments':11,'residential':12,'cemetery':16,'quarry':20,
          'landfill':20,'farmyard':28,'construction':31,'brownfield':31}
    for k in ('industrial','commercial','retail','railway','garages','military','education','religious','depot','port','greenhouse_horticulture'): lu[k]=13
    c = lu.get(g('landuse'),0)
    if c: return c
    if g('amenity') == 'grave_yard': return 16
    c = {'park':29,'garden':27,'pitch':30,'track':30,'golf_course':30,'playground':30,'swimming_pool':22}.get(g('leisure'),0)
    if c: return c
    if g('amenity') == 'parking' or 'area:highway' in t or g('aeroway') in ('apron','runway','taxiway') or \
       (g('highway') == 'pedestrian' and g('area') == 'yes') or g('place') == 'square': return 15
    if 'building' in t and g('building') != 'no': return 14
    return 0
blocks = [tuple(map(int, l.split())) for l in open(f'{outdir}/blocks.txt')]
fine = np.fromfile(f'{outdir}/posts_fine.u8', np.uint8).reshape(len(blocks), 64, 64)
cov = np.fromfile(f'{outdir}/posts_cov.u8', np.uint8).reshape(len(blocks), 64, 64)
bi = [b[0] for b in blocks]; bj = [b[1] for b in blocks]
lat_lo = (min(bi)*64*st - st)/TPD; lat_hi = ((max(bi)+1)*64*st + st)/TPD
lon_lo = (min(bj)*64*st - st)/TPD; lon_hi = ((max(bj)+1)*64*st + st)/TPD
# Closed ways: libosmium areas from ways. Multipolygon relations: even-odd over all member-way
# edges in numpy (independent of the Rust ring joiner; libosmium drops a whole relation on any
# ring error, which is not the rule under test).
geoms, cls, ids, rel_edges, rel_area = [], [], [], {}, {}
fab = osmium.geom.WKBFactory()
rels = {}
class R(osmium.SimpleHandler):
    def relation(s, r):
        t = {k: v for k, v in r.tags}
        if t.get('type') != 'multipolygon': return
        c = classify(t)
        if c: rels[r.id] = (c, [m.ref for m in r.members if m.type == 'w'])
R().apply_file(pbf)
need = {w for _, ms in rels.values() for w in ms}
wl = {}
class H(osmium.SimpleHandler):
    def way(s, w):
        if w.id in need:
            try: wl[w.id] = [(n.lon, n.lat) for n in w.nodes]
            except Exception: pass
    def area(s, a):
        if not a.from_way(): return
        t = {k: v for k, v in a.tags}
        c = classify(t)
        if not c: return
        try: g = shapely.wkb.loads(fab.create_multipolygon(a), hex=True)
        except Exception: return
        x0, y0, x1, y1 = g.bounds
        if x1 < lon_lo or x0 > lon_hi or y1 < lat_lo or y0 > lat_hi: return
        geoms.append(g); cls.append(c); ids.append(a.orig_id())
H().apply_file(pbf, locations=True, idx='flex_mem')
def inside_eo(E, x, y):
    x1, y1, x2, y2 = E
    m = ((y1 <= y) & (y < y2)) | ((y2 <= y) & (y < y1))
    xc = x1[m] + (y - y1[m]) * (x2[m] - x1[m]) / (y2[m] - y1[m])
    return (np.count_nonzero(xc < x) % 2) == 1
for rid, (c, ms) in rels.items():
    lines = [wl[w] for w in ms if w in wl and len(wl[w]) > 1]
    if not lines: continue
    a = np.concatenate([np.array(l[:-1]) for l in lines]); b = np.concatenate([np.array(l[1:]) for l in lines])
    if a[:,0].max() < lon_lo or a[:,0].min() > lon_hi or a[:,1].max() < lat_lo or a[:,1].min() > lat_hi: continue
    E = (a[:,0], a[:,1], b[:,0], b[:,1])
    faces = list(shapely.polygonize([shapely.LineString(l) for l in lines]).geoms)
    area = sum(f.area for f in faces if inside_eo(E, *f.representative_point().coords[0]))
    env = shapely.box(a[:,0].min(), a[:,1].min(), a[:,0].max(), a[:,1].max())
    rel_edges[len(geoms)] = E; rel_area[len(geoms)] = area
    geoms.append(env); cls.append(c); ids.append(-rid)
areas = np.array([g.area for g in geoms])
for m, ar in rel_area.items(): areas[m] = ar

tree = STRtree(geoms)
random.seed(7)
bad = badcov = 0; n = 0; ex = []
for _ in range(nposts):
    k = random.randrange(len(blocks)); r = random.randrange(64); c = random.randrange(64)
    i = blocks[k][0]*64 + r; j = blocks[k][1]*64 + c
    h = [0]*32
    for q in range(4):
        for w in range(4):
            y = (i*st - st/2 + (2*q+1)*st/8)/TPD; x = (j*st - st/2 + (2*w+1)*st/8)/TPD
            pt = shapely.Point(x, y)
            cand = [m for m in tree.query(pt) if (inside_eo(rel_edges[m], x, y) if m in rel_edges else geoms[m].covers(pt))]
            if cand:
                m = min(cand, key=lambda m: (areas[m], -PRI[cls[m]], -ids[m]))
                h[cls[m]] += 1
            else: h[0] += 1
    best = 0
    for kk in range(1, 32):
        hb = h[best] if best else 0
        if h[kk] > hb or (h[kk] == hb and h[kk] > 0 and PRI[kk] > PRI[best]): best = kk
    n += 1
    if best != fine[k, r, c]: bad += 1; ex.append((i, j, best, int(fine[k, r, c]), 16-h[0], int(cov[k, r, c])))
    if 16 - h[0] != cov[k, r, c]: badcov += 1
print('features', len(geoms), 'posts', n, 'class mismatches', bad, 'coverage mismatches', badcov)
for e in ex[:10]: print('  post', e[:2], 'ref', e[2], 'rust', e[3], 'cov ref/rust', e[4], e[5])

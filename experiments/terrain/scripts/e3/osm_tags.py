import osmium, sys, collections
KEYS = ('landuse','natural','leisure','landcover','wetland','water','waterway','amenity','surface','scrub','leaf_type','aeroway','man_made','place','tourism','building')
class H(osmium.SimpleHandler):
    def __init__(s):
        super().__init__(); s.c = collections.Counter(); s.src = collections.Counter(); s.n=0
    def area(s, a):
        s.n += 1
        t = a.tags
        for k in ('landuse','natural','leisure','landcover','waterway','amenity','aeroway','man_made'):
            if k in t:
                s.c[(k, t[k])] += 1
        if 'wetland' in t: s.c[('wetland', t['wetland'])] += 1
        if 'scrub' in t: s.c[('scrub', t['scrub'])] += 1
        if 'landuse' in t or 'natural' in t:
            src = t.get('source','')
            s.src[src.split(';')[0][:40]] += 1
h = H(); h.apply_file(sys.argv[1], locations=True, idx='flex_mem')
print('areas', h.n)
for (k,v),n in h.c.most_common(150): print(f'{n:8d} {k}={v}')
print('--- source on landuse/natural areas')
for s,n in h.src.most_common(25): print(f'{n:8d} {s!r}')

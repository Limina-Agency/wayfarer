import osmium, sys
from osmium.filter import TagFilter
wkt = osmium.geom.WKTFactory()
out = []
fp = osmium.FileProcessor(sys.argv[1]).with_areas(TagFilter(('admin_level','4'))).with_filter(osmium.filter.EntityFilter(osmium.osm.AREA))
for o in fp:
    if o.is_area() and o.tags.get('boundary')=='administrative':
        name=o.tags.get('name','')
        print(o.orig_id(), o.from_way(), name, file=sys.stderr)
        if 'małopolskie' in name.lower():
            out.append(wkt.create_multipolygon(o))
open(sys.argv[2],'w').write(out[0] if out else '')

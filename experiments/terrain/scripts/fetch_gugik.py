# Fetch GUGiK NMT (1 m grid, EPSG:2180) via WCS 2.0.1 for a lat/lon box, in 2000 m chunks.
import sys, os, math, subprocess
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tm2180 import fwd
name, lat0, lon0, lat1, lon1 = sys.argv[1], *map(float, sys.argv[2:6])
out = f'/home/user/tiles/gugik/{name}'; os.makedirs(out, exist_ok=True)
xs, ys = [], []
for la in (lat0, lat1):
    for lo in (lon0, lon1):
        x, y = fwd(la, lo); xs.append(float(x)); ys.append(float(y))
x0 = math.floor(min(xs)/1000)*1000 - 1000; x1 = math.ceil(max(xs)/1000)*1000 + 1000
y0 = math.floor(min(ys)/1000)*1000 - 1000; y1 = math.ceil(max(ys)/1000)*1000 + 1000
B = ("https://mapy.geoportal.gov.pl/wss/service/PZGIK/NMT/GRID1/WCS/DigitalTerrainModelFormatTIFF?"
     "SERVICE=WCS&VERSION=2.0.1&REQUEST=GetCoverage&COVERAGEID=DTM_PL-KRON86-NH_TIFF&FORMAT=image/tiff")
for x in range(x0, x1, 2000):
    for y in range(y0, y1, 2000):
        f = f'{out}/x{x}_y{y}.tif'
        if os.path.exists(f) and os.path.getsize(f) > 1000: continue
        url = f"{B}&SUBSET=x({x},{x+2000})&SUBSET=y({y},{y+2000})"
        r = subprocess.run(['curl', '-s', '--retry', '3', '-m', '600', '-o', f, '-w', '%{http_code} %{size_download}', url], capture_output=True, text=True)
        print(f, r.stdout, flush=True)

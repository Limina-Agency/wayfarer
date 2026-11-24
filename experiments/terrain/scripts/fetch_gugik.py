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
x0 = math.floor(min(xs)/1000)*1000; x1 = math.ceil(max(xs)/1000)*1000
STEP = 1000
y0 = math.floor(min(ys)/1000)*1000; y1 = math.ceil(max(ys)/1000)*1000
from concurrent.futures import ThreadPoolExecutor
B = ("https://mapy.geoportal.gov.pl/wss/service/PZGIK/NMT/GRID1/WCS/DigitalTerrainModelFormatTIFF?"
     "SERVICE=WCS&VERSION=2.0.1&REQUEST=GetCoverage&COVERAGEID=DTM_PL-KRON86-NH_TIFF&FORMAT=image/tiff")
import time
def get(xy):
    x, y = xy
    f = f'{out}/x{x}_y{y}.tif'
    if os.path.exists(f) and os.path.getsize(f) > 1000: return f + ' cached'
    url = f"{B}&SUBSET=x({x},{x+STEP})&SUBSET=y({y},{y+STEP})"
    wait = 60
    for attempt in range(12):   # the service drops connections for minutes at a time
        r = subprocess.run(['curl', '-s', '-m', '600', '-o', f, '-w', '%{http_code} %{size_download} %{time_total}', url], capture_output=True, text=True)
        if r.stdout.startswith('200') and os.path.getsize(f) > 1000:
            return f + ' ' + r.stdout + f' attempt {attempt + 1}'
        if os.path.exists(f): os.remove(f)
        time.sleep(wait); wait = min(2*wait, 600)
    return f + ' FAILED'
jobs = [(x, y) for x in range(x0, x1, STEP) for y in range(y0, y1, STEP)]
print(name, len(jobs), 'chunks', flush=True)
for j in jobs: print(get(j), flush=True)

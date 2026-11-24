# E4: in the three GUGiK LiDAR boxes (E1), slope from the 1 m bare-earth NMT vs the stored 1" GLO-30 grid,
# and the pace residual under each.   lidar.py <e4work> <gugik-dir> <out-json>
import sys, glob, json, math, numpy as np, rasterio, pyproj
work, gdir, outp = sys.argv[1], sys.argv[2], sys.argv[3]
sys.path.insert(0, __file__.rsplit('/', 1)[0])
TR = pyproj.Transformer.from_crs(4326, 2180, always_xy=True)
tiles = {}
for fn in glob.glob(f'{gdir}/*/*.tif'):
    r = rasterio.open(fn); tiles[(int(r.bounds.left) // 1000, int(r.bounds.bottom) // 1000)] = r.read(1).astype(np.float64), r.bounds
def z_at(x, y):
    out = np.full(len(x), np.nan)
    for k in range(len(x)):
        key = (int(x[k] // 1000), int(y[k] // 1000))
        if key not in tiles: continue
        a, b = tiles[key]
        # pixel centres at left + 0.5 .. ; bilinear, 0 = no data
        fx = x[k] - b.left - 0.5; fy = b.top - y[k] - 0.5
        i, j = int(math.floor(fy)), int(math.floor(fx))
        if i < 0 or j < 0 or i >= a.shape[0] - 1 or j >= a.shape[1] - 1: continue
        w = a[i:i + 2, j:j + 2]
        if (w == 0).any(): continue
        di, dj = fy - i, fx - j
        out[k] = w[0, 0] * (1 - di) * (1 - dj) + w[1, 0] * di * (1 - dj) + w[0, 1] * (1 - di) * dj + w[1, 1] * di * dj
    return out
res = {}
for fn in sorted(glob.glob(f'{work}/seg_*_50.npz')):
    d = np.load(fn)
    if 'lat_a' not in d: continue
    xa, ya = TR.transform(d['lon_a'], d['lat_a']); xb, yb = TR.transform(d['lon_b'], d['lat_b'])
    za = z_at(np.asarray(xa), np.asarray(ya)); zb = z_at(np.asarray(xb), np.asarray(yb))
    m = np.isfinite(za) & np.isfinite(zb) & (d['inside'] > 0) & (d['on_share'] >= 0.8) & (d['dmed'] <= 10)
    if m.sum() == 0: continue
    res[fn.split('seg_')[1][:-7]] = dict(za=za[m].tolist(), zb=zb[m].tolist(), idx=np.flatnonzero(m).tolist())
json.dump(res, open(outp, 'w'))
print({k: len(v['idx']) for k, v in res.items()})

# Independent check of DEM lattice posts (GLO-30) with rasterio + numpy: bilinear at 1",
# cell mean of 1" posts at coarser spacings. Only for blocks inside one source tile.
import sys, numpy as np, rasterio, glob
outdir, st = sys.argv[1], int(sys.argv[2])
TPD = 432000
blocks = [tuple(map(int, l.split())) for l in open(f'{outdir}/blocks.txt')]
dm = np.fromfile(f'{outdir}/posts_dm.i32', '<i4').reshape(len(blocks), 64, 64)
tiles = {}
for p in glob.glob('/home/user/tiles/glo30/*.tif'):
    with rasterio.open(p) as d:
        a = d.read(1).astype(np.float64)
        t = d.transform
        r = a*10; v = np.sign(r)*np.floor(np.abs(r)+0.5)
        dlon = round(t.a*TPD); dlat = round(-t.e*TPD)
        lon0 = round(t.c*TPD) + dlon//2; lat0 = round(t.f*TPD) - dlat//2
        tiles[p] = (lat0, lon0, dlat, dlon, v.astype(np.int64))
def rnd(num, den):  # half away from zero
    return np.where(num >= 0, (2*num + den)//(2*den), -((-2*num + den)//(2*den)))
def bil(lat, lon):
    for lat0, lon0, dlat, dlon, v in tiles.values():
        dy = lat0 - lat; dx = lon - lon0
        if dy < 0 or dx < 0 or dy >= v.shape[0]*dlat or dx >= v.shape[1]*dlon: continue
        r, fy, c, fx = dy//dlat, dy % dlat, dx//dlon, dx % dlon
        if r+1 >= v.shape[0] or c+1 >= v.shape[1]: return None
        w = [(dlat-fy)*(dlon-fx), fy*(dlon-fx), (dlat-fy)*fx, fy*fx]
        vals = [v[r,c], v[r+1,c], v[r,c+1], v[r+1,c+1]]
        num = sum(int(a)*b for a, b in zip(vals, w)); den = sum(w)
        return int(rnd(num, den))
    return None
k = st//120; h = k//2
wt = lambda d: min(2, max(0, k + 1 - 2*abs(d)))
rng = np.random.default_rng(1)
bad = n = skipped = 0
for _ in range(400):
    bidx = rng.integers(len(blocks)); r, c = rng.integers(64, size=2)
    bi, bj = blocks[bidx]; i = bi*64 + r; j = bj*64 + c
    if k <= 1:
        want = bil(i*st, j*st)
    else:
        num = den = 0; ok = True
        for dy in range(-h, h+1):
            for dx in range(-h, h+1):
                w = wt(dy)*wt(dx)
                if w == 0: continue
                v = bil((i*k+dy)*120, (j*k+dx)*120)
                if v is None: ok = False; break
                num += v*w; den += w
            if not ok: break
        want = int(rnd(num, den)) if ok else None
    if want is None: skipped += 1; continue
    n += 1; bad += int(want != dm[bidx, r, c])
print(outdir, st, 'checked', n, 'mismatches', bad, 'skipped (tile edge)', skipped)

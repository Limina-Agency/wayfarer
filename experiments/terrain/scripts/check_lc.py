# Independent check of the land-cover lattice majority with rasterio + numpy.
import sys, numpy as np, rasterio
from rasterio.windows import Window
outdir, st = sys.argv[1], int(sys.argv[2])
TPD = 432000
blocks = [tuple(map(int, l.split())) for l in open(f'{outdir}/blocks.txt')]
cls = np.fromfile(f'{outdir}/posts_class.u8', np.uint8).reshape(len(blocks), 64, 64)
MAP = {10:1,20:2,30:3,40:4,50:5,60:6,70:7,80:8,90:9,95:10,100:11}
PRI = [0,6,5,3,2,9,8,7,11,10,1,4]
lut = np.zeros(256, np.int64)
for k, v in MAP.items(): lut[k] = v
f = rasterio.open('/home/user/tiles/worldcover/ESA_WorldCover_10m_2021_v200_N48E018_Map.tif')
lat0 = 51*TPD - 18; lon0 = 18*TPD + 18; d = 36
bad = 0; n = 0
for bidx, (bi, bj) in enumerate(blocks[:12]):
    lat_lo = (bi*64)*st - st//2; lat_hi = (bi*64+63)*st + st//2
    lon_lo = (bj*64)*st - st//2; lon_hi = (bj*64+63)*st + st//2
    r_first = -((lat_hi - 1 - lat0)//d); r_last = (lat0 - lat_lo)//d
    c_first = -((lon0 - lon_lo)//d); c_last = (lon_hi - 1 - lon0)//d
    a = f.read(1, window=Window(c_first, r_first, c_last-c_first+1, r_last-r_first+1))
    lat = lat0 - np.arange(r_first, r_last+1)*d; lon = lon0 + np.arange(c_first, c_last+1)*d
    pr = (lat + st//2)//st - bi*64; pc = (lon + st//2)//st - bj*64
    okr = (lat >= lat_lo) & (lat < lat_hi); okc = (lon >= lon_lo) & (lon < lon_hi)
    h = np.zeros((64, 64, 12), np.int64)
    A = lut[a]
    for ri in np.nonzero(okr)[0]:
        for cl in range(1, 12):
            m = okc & (A[ri] == cl)
            np.add.at(h[pr[ri], :, cl], pc[m], 1)
    for r in range(64):
        for c in range(64):
            hh = h[r, c]; best = 0
            for k in range(1, 12):
                if hh[k] > hh[best] or (hh[k] == hh[best] and hh[k] > 0 and PRI[k] > PRI[best]): best = k
            n += 1; bad += int(best != cls[bidx, r, c])
print('checked posts', n, 'mismatches', bad)

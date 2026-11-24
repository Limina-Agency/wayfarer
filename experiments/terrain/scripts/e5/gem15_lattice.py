# E5 stage-1 prep: GEM-15 DTM (UTM 34N, 15 m, EGM96) onto the lattice posts of a block list.
#   python3 gem15_lattice.py <blocks.txt> <spacing-ticks> <gem15-dir> <glo30-dem-out> <out-dir>
# Height: mean of bilinear samples at 8 x 8 sub-cell centres of the post's cell (at least half
# valid), whole decimetres; GEM-15 gaps take the GLO-30 post (terrexp dem output).
# Steepness-max: largest Horn slope class (E1's 16 classes) of the native GEM-15 pixels under
# those samples; gaps take GLO-30's steepness-max. Floating point (numpy): a prototype of what a
# Rust stage-1 reader would do with an exact projected-grid path.
import sys, os, glob, json, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from tm2180 import fwd
import rasterio

bfile, st, gdir, glo, out = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4], sys.argv[5]
TPD = 432000
BOUNDS = [2, 5, 8, 11, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70]
blocks = [tuple(map(int, l.split())) for l in open(bfile) if l.strip()]
blocks.sort()
assert [tuple(map(int, l.split())) for l in open(f'{glo}/blocks.txt')] == blocks
K = 8
q = (np.arange(K) + 0.5) / K - 0.5           # sub-cell offsets in spacings
# all sample positions, per block (bi, bj): rows r, cols c, sub (a, b)
r = np.arange(64); c = np.arange(64)
def samples(bi, bj):
    lat = ((bi*64 + r)[:, None] + q[None, :]).reshape(-1) * st / TPD      # (64*K,)
    lon = ((bj*64 + c)[:, None] + q[None, :]).reshape(-1) * st / TPD
    LAT = np.repeat(lat[:, None], len(lon), 1); LON = np.repeat(lon[None, :], len(lat), 0)
    return LAT, LON                                                        # (64K, 64K)

# extent in UTM over all blocks, to crop the tiles
la0 = min(b[0] for b in blocks)*64*st/TPD - st/TPD; la1 = (max(b[0] for b in blocks)+1)*64*st/TPD + st/TPD
lo0 = min(b[1] for b in blocks)*64*st/TPD - st/TPD; lo1 = (max(b[1] for b in blocks)+1)*64*st/TPD + st/TPD
cE, cN = fwd(np.array([la0, la0, la1, la1]), np.array([lo0, lo1, lo0, lo1]), 21.0, 0.9996, 500000.0, 0.0)
tiles = []
for p in sorted(glob.glob(f'{gdir}/dtm_*.tif')):
    with rasterio.open(p) as d:
        x0, y1, w, h = d.transform.c, d.transform.f, d.width, d.height
        c0 = max(0, int((cE.min() - x0)//15) - 3); c1 = min(w, int((cE.max() - x0)//15) + 4)
        r0 = max(0, int((y1 - cN.max())//15) - 3); r1 = min(h, int((y1 - cN.min())//15) + 4)
        if c1 - c0 < 3 or r1 - r0 < 3:
            continue
        a = d.read(1, window=((r0, r1), (c0, c1))).astype(np.float64)
    a[~np.isfinite(a)] = np.nan
    # Horn slope on the native grid (north row first), class = number of bounds below
    Z = a
    gx = ((Z[:-2, 2:] + 2*Z[1:-1, 2:] + Z[2:, 2:]) - (Z[:-2, :-2] + 2*Z[1:-1, :-2] + Z[2:, :-2]))/(8*15.0)
    gy = ((Z[:-2, :-2] + 2*Z[:-2, 1:-1] + Z[:-2, 2:]) - (Z[2:, :-2] + 2*Z[2:, 1:-1] + Z[2:, 2:]))/(8*15.0)
    sl = np.degrees(np.arctan(np.hypot(gx, gy)))
    cls = np.full(a.shape, 255, np.uint8)
    inner = np.searchsorted(np.array(BOUNDS, float), sl, side='left').astype(np.uint8)
    inner[~np.isfinite(sl)] = 255
    cls[1:-1, 1:-1] = inner
    tiles.append((x0 + c0*15, y1 - r0*15, a, cls, os.path.basename(p)))
    print(f'{os.path.basename(p)}: window {a.shape}, nan {np.isnan(a).mean():.4f}', file=sys.stderr)

g_dm = np.fromfile(f'{glo}/posts_dm.i32', np.int32).reshape(-1, 64, 64)
g_st = np.fromfile(f'{glo}/posts_steep.u8', np.uint8).reshape(-1, 64, 64)
DM = np.empty_like(g_dm); STP = np.empty_like(g_st)
filled_h = filled_s = 0
for k, (bi, bj) in enumerate(blocks):
    LAT, LON = samples(bi, bj)
    E, N = fwd(LAT, LON, 21.0, 0.9996, 500000.0, 0.0)
    H = np.full(E.shape, np.nan); C = np.full(E.shape, 255, np.uint8)
    for X0, Y1, a, cls, _ in tiles:
        hh, ww = a.shape
        cx = (E - X0)/15 - 0.5; cy = (Y1 - N)/15 - 0.5
        ci = np.floor(cx).astype(int); ri = np.floor(cy).astype(int); fx = cx - ci; fy = cy - ri
        ok = (ci >= 0) & (ri >= 0) & (ci + 1 < ww) & (ri + 1 < hh)
        cc = np.clip(ci, 0, ww - 2); rr = np.clip(ri, 0, hh - 2)
        v = a[rr, cc]*(1-fx)*(1-fy) + a[rr, cc+1]*fx*(1-fy) + a[rr+1, cc]*(1-fx)*fy + a[rr+1, cc+1]*fx*fy
        H = np.where(ok & np.isnan(H), v, H)
        ni = np.clip(np.floor((E - X0)/15).astype(int), 0, ww - 1); nr = np.clip(np.floor((Y1 - N)/15).astype(int), 0, hh - 1)
        inside = ((E - X0) >= 0) & ((E - X0) < ww*15) & ((Y1 - N) >= 0) & ((Y1 - N) < hh*15)
        cv = np.where(inside, cls[nr, ni], 255)
        C = np.where(C == 255, cv, C)
    Hc = H.reshape(64, K, 64, K); Cc = C.reshape(64, K, 64, K)
    valid = np.isfinite(Hc).sum(axis=(1, 3))
    mean = np.nansum(Hc, axis=(1, 3)) / np.maximum(valid, 1)
    dm = np.where(valid >= K*K//2, np.round(mean*10), np.nan)
    gm = np.isnan(dm)
    filled_h += int(gm.sum())
    DM[k] = np.where(gm, g_dm[k], np.nan_to_num(dm)).astype(np.int32)
    Cm = np.where(Cc == 255, 0, Cc.astype(int) + 1).max(axis=(1, 3))     # 0 = no valid class
    sm = Cm == 0
    filled_s += int(sm.sum())
    STP[k] = np.where(sm, g_st[k], Cm - 1).astype(np.uint8)
os.makedirs(out, exist_ok=True)
DM.tofile(f'{out}/posts_dm.i32'); STP.tofile(f'{out}/posts_steep.u8')
n = DM.size
res = {'blocks': len(blocks), 'spacing_ticks': st, 'posts': n, 'gem15_height_gap_share': round(filled_h/n, 5),
       'gem15_steep_gap_share': round(filled_s/n, 5), 'tiles': [t[4] for t in tiles],
       'steep_hist_gem15_filled': np.bincount(STP.reshape(-1), minlength=256)[:16].tolist(),
       'steep_hist_glo30': np.bincount(g_st.reshape(-1), minlength=256)[:16].tolist(),
       'dm_minmax': [int(DM.min()), int(DM.max())]}
json.dump(res, open(f'{out}/gem15.json', 'w'), indent=1)
print(json.dumps(res))

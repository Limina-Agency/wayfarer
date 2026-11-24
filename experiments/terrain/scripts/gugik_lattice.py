# E1 lattice-spacing study on GUGiK NMT 1 m LiDAR (bare earth) as truth.
# Truth is sampled at 1/8" geographic posts (15 ticks), then stored on lattices of 1/2", 1",
# 2", 3" by the E1 cell-mean rule (whole metres), and rebuilt by bilinear interpolation.
# Reports height error, climb error along 1 km lines and steep-area share, plus the same for
# E1's GLO-30 / SRTM / NASADEM 1" posts (source error, after removing the mean datum bias).
import sys, os, glob, json
import numpy as np, rasterio
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tm2180 import fwd

TPD = 432000
name, lat0, lon0, lat1, lon1 = sys.argv[1], *map(float, sys.argv[2:6])
area = sys.argv[6]  # E1 area whose 1" outputs cover the box
FT = 15  # fine spacing, ticks

# 1. mosaic the 2180 tiles
tiles = sorted(glob.glob(f'/home/user/tiles/gugik/{name}/*.tif'))
infos = []
for t in tiles:
    with rasterio.open(t) as d:
        infos.append((d.transform.c, d.transform.f, d.width, d.height, d.read(1).astype(np.float64), d.nodata))
X0 = min(i[0] for i in infos); Y1 = max(i[1] for i in infos)
W = int(round(max(i[0] + i[2] for i in infos) - X0)); H = int(round(Y1 - min(i[1] - i[3] for i in infos)))
big = np.full((H, W), np.nan)
for x, y, w, h, a, nd in infos:
    a = np.where((a <= 0) | (a > 9000) | ((a == nd) if nd is not None else False), np.nan, a)
    big[int(round(Y1 - y)):int(round(Y1 - y)) + h, int(round(x - X0)):int(round(x - X0)) + w] = a

def truth_at(lat, lon):
    E, N = fwd(lat, lon)
    cx = E - X0 - 0.5; cy = Y1 - N - 0.5   # pixel centres
    c = np.floor(cx).astype(int); r = np.floor(cy).astype(int); fx = cx - c; fy = cy - r
    ok = (c >= 0) & (r >= 0) & (c + 1 < W) & (r + 1 < H)
    c = np.clip(c, 0, W - 2); r = np.clip(r, 0, H - 2)
    v = (big[r, c]*(1-fx)*(1-fy) + big[r, c+1]*fx*(1-fy) + big[r+1, c]*(1-fx)*fy + big[r+1, c+1]*fx*fy)
    return np.where(ok, v, np.nan)

# 2. fine truth posts (rows south to north)
i0 = int(np.ceil(lat0*TPD/FT)); i1 = int(np.floor(lat1*TPD/FT))
j0 = int(np.ceil(lon0*TPD/FT)); j1 = int(np.floor(lon1*TPD/FT))
I = np.arange(i0, i1 + 1); J = np.arange(j0, j1 + 1)
LAT, LON = np.meshgrid(I*FT/TPD, J*FT/TPD, indexing='ij')
T = truth_at(LAT, LON)
print(f'{name}: fine posts {T.shape}, nan {np.isnan(T).mean():.4f}, height {np.nanmin(T):.0f}-{np.nanmax(T):.0f} m', file=sys.stderr)

def lattice_from_fine(s):
    """Cell-mean lattice at spacing s ticks from the fine truth (E1 rule), whole metres."""
    k = s // FT; h = k // 2
    wt = np.array([min(2, max(0, k + 1 - 2*abs(d))) for d in range(-h, h + 1)], float)
    pi0 = -(-(i0*FT + h*FT) // s); pi1 = (i1*FT - h*FT) // s
    pj0 = -(-(j0*FT + h*FT) // s); pj1 = (j1*FT - h*FT) // s
    PI = np.arange(pi0, pi1 + 1); PJ = np.arange(pj0, pj1 + 1)
    out = np.zeros((len(PI), len(PJ)))
    W2 = np.outer(wt, wt)
    for a, pi in enumerate(PI):
        ci = pi*s//FT - i0
        rows = T[ci - h:ci + h + 1]
        for b, pj in enumerate(PJ):
            cj = pj*s//FT - j0
            win = rows[:, cj - h:cj + h + 1]
            m = ~np.isnan(win)
            out[a, b] = np.nan if not m.any() else (win[m]*W2[m]).sum()/W2[m].sum()
    return PI, PJ, np.sign(out)*np.floor(np.abs(out) + 0.5)

def bilinear_grid(PI, PJ, V, s, lat, lon):
    """Bilinear on a lattice (rows PI*s, cols PJ*s ticks) at points (degrees)."""
    y = lat*TPD/s - PI[0]; x = lon*TPD/s - PJ[0]
    r = np.floor(y).astype(int); c = np.floor(x).astype(int)
    ok = (r >= 0) & (c >= 0) & (r + 1 < len(PI)) & (c + 1 < len(PJ))
    r = np.clip(r, 0, len(PI) - 2); c = np.clip(c, 0, len(PJ) - 2)
    fy = y - r; fx = x - c
    v = V[r, c]*(1-fx)*(1-fy) + V[r, c+1]*fx*(1-fy) + V[r+1, c]*(1-fx)*fy + V[r+1, c+1]*fx*fy
    return np.where(ok, v, np.nan)

def horn_slope_deg(Z, dy_m, dx_m):
    a = Z[2:, :-2]; b = Z[2:, 1:-1]; c = Z[2:, 2:]   # rows south->north: row+1 is north
    d = Z[1:-1, :-2]; f = Z[1:-1, 2:]
    g = Z[:-2, :-2]; hh = Z[:-2, 1:-1]; i = Z[:-2, 2:]
    gx = ((c + 2*f + i) - (a + 2*d + g))/(8*dx_m)
    gy = ((a + 2*b + c) - (g + 2*hh + i))/(8*dy_m)
    return np.degrees(np.arctan(np.hypot(gx, gy)))

R = 6371008.8
latc = (lat0 + lat1)/2
m_per_tick_y = np.pi/180/TPD*R; m_per_tick_x = m_per_tick_y*np.cos(np.radians(latc))

# 1 km lines for climb
rng = np.random.default_rng(7)
lines = []
while len(lines) < 300:
    la = rng.uniform(lat0 + 0.004, lat1 - 0.004); lo = rng.uniform(lon0 + 0.006, lon1 - 0.006)
    th = rng.uniform(0, 2*np.pi)
    dla = 1000*np.cos(th)/(m_per_tick_y*TPD); dlo = 1000*np.sin(th)/(m_per_tick_x*TPD)
    la2, lo2 = la + dla, lo + dlo
    if lat0 + 0.002 < la2 < lat1 - 0.002 and lon0 + 0.003 < lo2 < lon1 - 0.003:
        lines.append((la, lo, la2, lo2))
tt = np.linspace(0, 1, 201)  # 5 m steps
LL = np.array([[la + (la2 - la)*tt, lo + (lo2 - lo)*tt] for la, lo, la2, lo2 in lines])  # (n,2,201)
truth_lines = truth_at(LL[:, 0], LL[:, 1])
def climb(z):
    dz = np.diff(z, axis=1)
    return np.where(dz > 0, dz, 0).sum(axis=1)
truth_climb = climb(truth_lines)

inner = (slice(40, -40), slice(40, -40))
fine_slope = horn_slope_deg(T, FT*m_per_tick_y, FT*m_per_tick_x)
res = {'box': name, 'bounds': [lat0, lon0, lat1, lon1], 'fine_posts': list(T.shape),
       'truth_relief_m': [float(np.nanmin(T)), float(np.nanmax(T))],
       'truth_climb_median_m_per_km': float(np.median(truth_climb)),
       'fine_slope_over30_pct': float(100*np.nanmean(fine_slope > 30)), 'rows': []}

def evaluate(label, PI, PJ, V, s, bias_fix):
    rec = bilinear_grid(PI, PJ, V, s, LAT, LON)
    err = (rec - T)[inner]
    bias = float(np.nanmean(err)) if bias_fix else 0.0
    err = err - bias
    lrec = bilinear_grid(PI, PJ, V, s, LL[:, 0], LL[:, 1]) - bias
    lc = climb(lrec)
    ok = (truth_climb > 20) & ~np.isnan(truth_lines).any(axis=1) & ~np.isnan(lrec).any(axis=1)
    rel = (lc[ok] - truth_climb[ok])/truth_climb[ok]
    sl = horn_slope_deg(V, s*m_per_tick_y, s*m_per_tick_x)
    row = {'grid': label, 'spacing_ticks': s, 'bias_removed_m': round(bias, 2),
           'rmse_m': round(float(np.sqrt(np.nanmean(err**2))), 2),
           'p95_abs_m': round(float(np.nanpercentile(np.abs(err), 95)), 2),
           'lines': int(ok.sum()),
           'climb_err_median_pct': round(float(100*np.median(rel)), 1),
           'climb_abs_err_p90_pct': round(float(100*np.percentile(np.abs(rel), 90)), 1),
           'slope_over30_pct': round(float(100*np.nanmean(sl > 30)), 2)}
    res['rows'].append(row)
    print(row, file=sys.stderr)

for s in (60, 120, 240, 360):
    PI, PJ, V = lattice_from_fine(s)
    evaluate(f'truth->{s}', PI, PJ, V, s, False)

# Source DEMs at 1" from E1 outputs
for src in ('glo30', 'srtm', 'nasadem'):
    d = f'/home/user/e1work/out/{area}_{src}_120'
    blocks = [tuple(map(int, l.split())) for l in open(f'{d}/blocks.txt')]
    dm = np.fromfile(f'{d}/posts_dm.i32', '<i4').reshape(len(blocks), 64, 64)
    bi0 = min(b[0] for b in blocks); bj0 = min(b[1] for b in blocks)
    nb_i = max(b[0] for b in blocks) - bi0 + 1; nb_j = max(b[1] for b in blocks) - bj0 + 1
    G = np.full((nb_i*64, nb_j*64), np.nan)
    for k, (bi, bj) in enumerate(blocks):
        v = dm[k].astype(float); v[dm[k] == np.iinfo(np.int32).min] = np.nan
        G[(bi - bi0)*64:(bi - bi0 + 1)*64, (bj - bj0)*64:(bj - bj0 + 1)*64] = np.sign(v)*np.floor(np.abs(v)/10 + 0.5)
    PI = np.arange(bi0*64, (bi0 + nb_i)*64); PJ = np.arange(bj0*64, (bj0 + nb_j)*64)
    evaluate(f'{src}@1"', PI, PJ, G, 120, True)
print(json.dumps(res, indent=1))

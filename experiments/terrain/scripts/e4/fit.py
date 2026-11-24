# E4: pace models against OSM GPS trace segments.
#
#   fit.py <e4work-dir> <out-dir> [seglen]
#
# Reads seg_<area>_<seglen>.npz for every area, writes <out-dir>/results_<seglen>.json and tables_<seglen>.md.
import sys, os, glob, json, math
import numpy as np
from scipy.optimize import minimize

work, outd = sys.argv[1], sys.argv[2]
seglen = int(sys.argv[3]) if len(sys.argv) > 3 else 50
os.makedirs(outd, exist_ok=True)
COVER = ["none", "tree", "shrub", "grass", "crop", "built", "bare", "snow", "water", "wetland", "mangrove", "moss"]
WCLS = ["none", "paved road", "track", "path (no/easy sac_scale)", "mountain path (T2)", "alpine path (T3+)", "steps", "cycleway/bridleway"]

# ---------------- data ----------------
D = {}; OFFSET = {}; _n = 0
for fn in sorted(glob.glob(f'{work}/seg_*_{seglen}.npz')):
    area = os.path.basename(fn)[4:-len(f'_{seglen}.npz')]
    d = dict(np.load(fn)); d['area'] = np.full(len(d['dt']), area, dtype=object)
    OFFSET[area] = _n; _n += len(d['dt'])
    for k, v in d.items():
        D.setdefault(k, []).append(v)
D = {k: np.concatenate(v) for k, v in D.items()}
areas = sorted(set(D['area']))
# global track id across areas
tid = np.array([f'{a}:{int(t)}' for a, t in zip(D['area'], D['track'])])
v = D['len'] / D['dt']
month = ((D['t0'] // 86400 + 719468) % 146097)  # placeholder, replaced below
def month_of(ts):
    days = np.floor(ts / 86400).astype(np.int64) + 719468
    era = np.floor_divide(days, 146097); doe = days - era * 146097
    yoe = (doe - doe // 1460 + doe // 36524 - doe // 146096) // 365
    doy = doe - (365 * yoe + yoe // 4 - yoe // 100); mp = (5 * doy + 2) // 153
    return np.where(mp < 10, mp + 3, mp - 9)
month = month_of(D['t0'])
year = 1970 + np.floor(D['t0'] / 86400 / 365.2425).astype(int)
summer = (month >= 5) & (month <= 10)
snow = (month == 12) | (month <= 3)

def slope_from(za, zb):
    s = (zb - za) / D['len']
    return s, np.degrees(np.arctan(s))
S = {}
for k, (a, b) in {'glo30_1': ('z_g1a', 'z_g1b'), 'srtm_1': ('z_s1a', 'z_s1b'), 'glo30_3': ('z_g3a', 'z_g3b')}.items():
    S[k] = slope_from(D[a], D[b])
base_ok = (D['inside'] > 0) & (D['dt'] > 0) & (v > 0.1) & (v < 3.5) & np.isfinite(S['glo30_1'][0]) & np.isfinite(S['glo30_3'][0])
trail = (D['on_share'] >= 0.8) & (D['dmed'] <= 10)
offtrail = (D['on_share'] == 0) & (D['dmed'] > 40)

# ---------------- models (v in m/s; th in degrees, s = tan) ----------------
CAMPBELL = {  # Campbell, Dennison & Thompson 2022, Table A3 (a, b, c, d, e)
    10: (-1.4921, 20.8687, 65.9137, -0.0329, 6.7077e-4),
    50: (-1.4579, 22.0787, 76.3271, 0.0525, -3.2002e-4),
    90: (-1.4109, 19.8424, 46.8318, 0.5561, -1.1847e-3),
}
def lorentz(th, a, b, c, d, e):
    return c * (1.0 / (math.pi * b * (1.0 + ((th - a) / b) ** 2))) + d + e * th
def tobler(s, off=False):
    return 6.0 * np.exp(-3.5 * np.abs(s + 0.05)) / 3.6 * (0.6 if off else 1.0)
def naismith(th):
    t = np.tan(np.radians(th))
    pace = np.where(th >= 0, 0.72 + 6.0 * t, np.where(th > -5, 0.72, np.where(th > -12, 0.72 + 2.0 * t, 0.72 - 2.0 * t)))
    return 1.0 / pace
def ic(s, off=False):
    f = 0.975  # mean of male (1.0) and female (0.95)
    return f * (0.11 + 0.67 * np.exp(-(100 * s + 2) ** 2 / 1800)) if off else f * (0.11 + np.exp(-(100 * s + 5) ** 2 / 1800))
MODELS = {
    'Tobler': lambda s, th, off: tobler(s, off),
    'Naismith-Langmuir': lambda s, th, off: naismith(th) * (0.8 if off else 1.0),  # Aitken: 4 km/h off path
    'Irmischer-Clarke': lambda s, th, off: ic(s, off),
    'Campbell P50': lambda s, th, off: lorentz(th, *CAMPBELL[50]),
    'Wayfarer today (OnFoot)': lambda s, th, off: np.full_like(s, 1.2 if off else 4.8 / 3.6),
}

# ---------------- fitting ----------------
def pinball(q, r):
    return np.mean(np.maximum(q * r, (q - 1) * r))
def fit_lorentz(th, vv, q, x0):
    f = lambda p: pinball(q, vv - lorentz(th, *p)) if p[1] > 1 else 1e9
    r = minimize(f, x0, method='Nelder-Mead', options=dict(maxiter=20000, maxfev=20000, xatol=1e-6, fatol=1e-9))
    r = minimize(f, r.x, method='Nelder-Mead', options=dict(maxiter=20000, maxfev=20000, xatol=1e-7, fatol=1e-10))
    return r.x
def tobler_form(s, a, b, c):
    return a * np.exp(-b * np.abs(s + c))
def fit_tobler(s, vv, q):
    f = lambda p: pinball(q, vv - tobler_form(s, *p))
    return minimize(f, (1.4, 3.5, 0.05), method='Nelder-Mead', options=dict(maxiter=20000, xatol=1e-7, fatol=1e-10)).x

BINS = np.arange(-30, 31, 5)
def bin_table(th, vv, mask, models=None, off=False, s=None):
    rows = []
    for lo in BINS[:-1]:
        m = mask & (th >= lo) & (th < lo + 5)
        if m.sum() < 30:
            rows.append(dict(lo=int(lo), n=int(m.sum()))); continue
        q = np.percentile(vv[m], [10, 50, 90])
        r = dict(lo=int(lo), n=int(m.sum()), tracks=int(len(set(tid[m]))), p10=q[0], p50=q[1], p90=q[2])
        if models:
            for name, fn in models.items():
                r[name] = float(np.median(vv[m] / fn(s[m], th[m], off)))
        rows.append(r)
    return rows

def track_time_error(mask, th, s, fns, off=False, min_m=1000):
    """Per track (>= 1 km of segments in mask): actual moving time / predicted time; percentiles across tracks."""
    out = {}
    tt = tid[mask]; dt = D['dt'][mask]; ln = D['len'][mask]
    ut, inv = np.unique(tt, return_inverse=True)
    cnt = np.bincount(inv)
    keep = cnt * seglen >= min_m
    act = np.bincount(inv, weights=dt)
    for name, fn in fns.items():
        vp = np.maximum(fn(s[mask], th[mask], off), 0.05)
        pred = np.bincount(inv, weights=ln / vp)
        ratio = (act / pred)[keep]
        if len(ratio) == 0:
            out[name] = dict(tracks=0); continue
        out[name] = dict(tracks=int(keep.sum()), p10=float(np.percentile(ratio, 10)), p50=float(np.percentile(ratio, 50)),
                         p90=float(np.percentile(ratio, 90)), mape=float(np.median(np.abs(ratio - 1))))
    return out, (act, cnt, inv, keep)

R = dict(seglen=seglen, areas={})
for a in areas:
    m = base_ok & (D['area'] == a)
    R['areas'][a] = dict(segments=int(m.sum()), km=float(m.sum() * seglen / 1000), tracks=int(len(set(tid[m]))),
                         trail=int((m & trail).sum()), offtrail=int((m & offtrail).sum()),
                         summer=int((m & summer).sum()), snow=int((m & snow).sum()))
R['years'] = {int(y): int(c) for y, c in zip(*np.unique(year[base_ok], return_counts=True))}

s1, th1 = S['glo30_1']
CAL = base_ok & trail & summer & (np.abs(th1) <= 40)
R['cal_segments'] = int(CAL.sum()); R['cal_tracks'] = int(len(set(tid[CAL])))

# 1. observed percentiles by slope bin vs models (summer, on trail, GLO-30 1")
R['bins_trail'] = bin_table(th1, v, CAL, MODELS, False, s1)

# 2. Campbell band coverage
lo10 = lorentz(th1, *CAMPBELL[10]); hi90 = lorentz(th1, *CAMPBELL[90])
R['campbell_coverage'] = dict(below_p10=float(np.mean(v[CAL] < lo10[CAL])), above_p90=float(np.mean(v[CAL] > hi90[CAL])))

# 3. our fits, per DEM
FITS = {}
for dem in ('glo30_1', 'srtm_1', 'glo30_3'):
    s_, th_ = S[dem]
    m = base_ok & trail & summer & (np.abs(th_) <= 40)
    FITS[dem] = {q: fit_lorentz(th_[m], v[m], q / 100, CAMPBELL[q]) for q in (10, 50, 90)}
    FITS[dem]['tobler50'] = fit_tobler(s_[m], v[m], 0.5)
    FITS[dem]['tobler10'] = fit_tobler(s_[m], v[m], 0.1)
    FITS[dem]['tobler90'] = fit_tobler(s_[m], v[m], 0.9)
# mean-pace curve (expected time): least squares on pace (s/m) with the Lorentz speed form
def fit_mean_pace(th, vv, x0):
    pc = 1.0 / vv
    f = lambda p: np.mean((pc - 1.0 / np.maximum(lorentz(th, *p), 0.05)) ** 2) if p[1] > 1 else 1e9
    r = minimize(f, x0, method='Nelder-Mead', options=dict(maxiter=20000, maxfev=20000, xatol=1e-7, fatol=1e-12))
    return minimize(f, r.x, method='Nelder-Mead', options=dict(maxiter=20000, maxfev=20000, xatol=1e-8, fatol=1e-13)).x
for dem in ('glo30_1', 'srtm_1', 'glo30_3'):
    s_, th_ = S[dem]
    m = base_ok & trail & summer & (np.abs(th_) <= 40)
    FITS[dem]['mean'] = fit_mean_pace(th_[m], v[m], FITS[dem][50])
R['fits'] = {dem: {str(k): [float(x) for x in p] for k, p in f.items()} for dem, f in FITS.items()}
F = FITS['glo30_1']
def ours(q, dem='glo30_1'):
    p = FITS[dem][q]
    return lambda s, th, off: lorentz(th, *p)
def ours_tob(q, dem='glo30_1'):
    p = FITS[dem][f'tobler{q}']
    return lambda s, th, off: tobler_form(s, *p)
SPEEDTAB = []
for th in range(-30, 31, 5):
    s = math.tan(math.radians(th))
    r = dict(deg=th, grade=round(100 * s, 1))
    for q in (10, 50, 90):
        r[f'E4 P{q}'] = float(lorentz(th, *F[q])); r[f'Campbell P{q}'] = float(lorentz(th, *CAMPBELL[q]))
    r['E4 Tobler-form P50'] = float(tobler_form(s, *F['tobler50'])); r['E4 mean-pace'] = float(lorentz(th, *F['mean']))
    r['Tobler'] = float(tobler(np.array(s))); r['Irmischer-Clarke on'] = float(ic(np.array(s)))
    r['Naismith-Langmuir'] = float(naismith(np.array(th)))
    SPEEDTAB.append(r)
R['speed_table'] = SPEEDTAB
grid = np.linspace(-30, 30, 601)
R['peak'] = {f'E4 P{q}': float(grid[np.argmax(lorentz(grid, *F[q]))]) for q in (10, 50, 90)}

# 4. track-level time error (summer trail, GLO-30 1")
ALL = dict(MODELS); ALL['E4 P50 (Lorentz)'] = ours(50); ALL['E4 P50 (Tobler form)'] = ours_tob(50); ALL['E4 mean pace'] = ours('mean')
R['track_time_trail'], (act, cnt, inv, keep) = track_time_error(CAL, th1, s1, ALL)
# hold-out check: fit on half the tracks (by hash), test on the other half
ut = np.unique(tid[CAL]); half = np.array([hash(u) % 2 for u in ut])  # PYTHONHASHSEED fixed by the runner
train_tracks = set(ut[half == 0])
mtr = CAL & np.array([t in train_tracks for t in tid]); mte = CAL & ~mtr
p_tr = fit_lorentz(th1[mtr], v[mtr], 0.5, CAMPBELL[50])
R['holdout'] = dict(train_tracks=int(len(train_tracks)), test_tracks=int(len(ut) - len(train_tracks)),
                    fit=[float(x) for x in p_tr])
R['holdout']['test'], _ = track_time_error(mte, th1, s1, {'E4 P50 (train half)': lambda s, th, off: lorentz(th, *p_tr),
                                                          'Campbell P50': MODELS['Campbell P50'], 'Tobler': MODELS['Tobler']})
p_trm = fit_mean_pace(th1[mtr], v[mtr], p_tr)
R['holdout']['test_mean'], _ = track_time_error(mte, th1, s1, {'E4 mean pace (train half)': lambda s, th, off: lorentz(th, *p_trm)})
# person pace factor: per track, median of v / E4 P50 -> spread across tracks
ratio = v[CAL] / lorentz(th1[CAL], *F['mean'])
tt = tid[CAL]; ut2, inv2 = np.unique(tt, return_inverse=True)
per = np.array([np.median(ratio[inv2 == k]) for k in range(len(ut2)) if np.sum(inv2 == k) * seglen >= 1000])
R['person_factor'] = dict(tracks=int(len(per)), p10=float(np.percentile(per, 10)), p50=float(np.percentile(per, 50)),
                          p90=float(np.percentile(per, 90)))
R['segment_factor'] = dict(p10=float(np.percentile(ratio, 10)), p50=float(np.percentile(ratio, 50)), p90=float(np.percentile(ratio, 90)))
# clustered bootstrap (by track) of the E4 P50 speed at a few slopes
rng = np.random.default_rng(20261124)
idx_by_track = [np.flatnonzero(CAL)[inv2 == k] for k in range(len(ut2))]
boot = []
for b in range(int(os.environ.get('E4_BOOT', '60'))):
    pick = rng.integers(0, len(idx_by_track), len(idx_by_track))
    ii = np.concatenate([idx_by_track[k] for k in pick])
    p = fit_lorentz(th1[ii], v[ii], 0.5, F[50])
    boot.append([lorentz(t, *p) for t in (-20, -10, 0, 10, 20)])
boot = np.array(boot)
R['bootstrap_p50'] = {str(t): [float(np.percentile(boot[:, k], 5)), float(np.percentile(boot[:, k], 95))]
                      for k, t in enumerate((-20, -10, 0, 10, 20))}

# 5. trail-class factors: median of v / E4 P50 per OSM class
R['trail_class'] = {}
for c in range(1, 8):
    m = CAL & (D['wcls'] == c)
    if m.sum() >= 50:
        r = v[m] / lorentz(th1[m], *F[50])
        R['trail_class'][WCLS[c]] = dict(n=int(m.sum()), tracks=int(len(set(tid[m]))), p10=float(np.percentile(r, 10)),
                                         p50=float(np.median(r)), p90=float(np.percentile(r, 90)))
# 6. off-trail: factor vs trail P50 at the same slope, by WorldCover class; plus model comparison
OFF = base_ok & offtrail & summer & (np.abs(th1) <= 40)
R['off_segments'] = int(OFF.sum()); R['off_tracks'] = int(len(set(tid[OFF])))
R['off_cover'] = {}
for c in range(0, 12):
    m = OFF & (D['cover'] == c)
    if m.sum() >= 30:
        r = v[m] / lorentz(th1[m], *F[50])
        R['off_cover'][COVER[c]] = dict(n=int(m.sum()), tracks=int(len(set(tid[m]))), p10=float(np.percentile(r, 10)),
                                        p50=float(np.median(r)), p90=float(np.percentile(r, 90)))
# same on trail, by cover (does cover matter on a trail?)
R['trail_cover'] = {}
for c in range(0, 12):
    m = CAL & (D['cover'] == c)
    if m.sum() >= 100:
        r = v[m] / lorentz(th1[m], *F[50])
        R['trail_cover'][COVER[c]] = dict(n=int(m.sum()), p50=float(np.median(r)), p10=float(np.percentile(r, 10)), p90=float(np.percentile(r, 90)))
if OFF.sum() >= 30:
    R['bins_off'] = bin_table(th1, v, OFF, MODELS, True, s1)
    R['track_time_off'], _ = track_time_error(OFF, th1, s1, ALL, True, min_m=300)
# 7. season
WIN = base_ok & trail & snow & (np.abs(th1) <= 40)
if WIN.sum() >= 100:
    r = v[WIN] / lorentz(th1[WIN], *F[50])
    R['snow_season'] = dict(n=int(WIN.sum()), tracks=int(len(set(tid[WIN]))), p10=float(np.percentile(r, 10)),
                            p50=float(np.median(r)), p90=float(np.percentile(r, 90)))
    R['snow_by_area'] = {}
    for a in areas:
        mw = WIN & (D['area'] == a); ms = CAL & (D['area'] == a)
        if mw.sum() >= 100 and ms.sum() >= 100:
            fw = np.median(v[mw] / lorentz(th1[mw], *F[50])); fs = np.median(v[ms] / lorentz(th1[ms], *F[50]))
            R['snow_by_area'][a] = dict(snow_n=int(mw.sum()), snow_tracks=int(len(set(tid[mw]))), summer_n=int(ms.sum()),
                                        factor=float(fw / fs))
# 8. DEM sensitivity: model fitted on one grid, applied to slopes from another
R['dem_cross'] = {}
for fit_dem in ('glo30_1', 'glo30_3', 'srtm_1'):
    for use_dem in ('glo30_1', 'glo30_3', 'srtm_1'):
        s_, th_ = S[use_dem]
        m = base_ok & trail & summer & (np.abs(th_) <= 40) & (np.abs(S['glo30_1'][1]) <= 40)
        e, _ = track_time_error(m, th_, s_, {'x': ours('mean', fit_dem)})
        R['dem_cross'][f'{fit_dem} on {use_dem}'] = e['x']
# slope agreement between grids on trail segments
m = CAL
R['slope_abs_diff_deg'] = {f'{a} vs glo30_1': [float(np.percentile(np.abs(S[a][1][m] - th1[m]), q)) for q in (50, 90)]
                           for a in ('srtm_1', 'glo30_3')}
R['abs_slope_pct'] = {k: [float(np.percentile(np.abs(S[k][1][m]), q)) for q in (50, 90, 99)] for k in S}
# by area
R['by_area'] = {}
for a in areas:
    m = CAL & (D['area'] == a)
    if m.sum() >= 200:
        e, _ = track_time_error(m, th1, s1, {'E4 mean pace': ours('mean'), 'Campbell P50': MODELS['Campbell P50'], 'Tobler': MODELS['Tobler']})
        R['by_area'][a] = e

# 9. LiDAR boxes (GUGiK NMT 1 m): does a bare-earth slope explain pace better than the stored grid?
lj = f'{work}/lidar.json'
if os.path.exists(lj) and seglen == 50:
    L = json.load(open(lj)); gi = []; zl = []
    for a, r in L.items():
        gi += [OFFSET[a] + i for i in r['idx']]; zl += list(zip(r['za'], r['zb']))
    gi = np.array(gi); zl = np.array(zl)
    sl = (zl[:, 1] - zl[:, 0]) / D['len'][gi]; thl = np.degrees(np.arctan(sl))
    ok = base_ok[gi] & summer[gi] & (np.abs(thl) <= 40) & (np.abs(th1[gi]) <= 40)
    gi, sl, thl = gi[ok], sl[ok], thl[ok]
    pl = fit_mean_pace(thl, v[gi], F['mean']); pg = fit_mean_pace(th1[gi], v[gi], F['mean'])
    def mad(x): return float(np.median(np.abs(x - np.median(x))))
    R['lidar'] = dict(segments=int(len(gi)), tracks=int(len(set(tid[gi]))),
                      by_box={a: int(np.sum(D['area'][gi] == a)) for a in L},
                      abs_slope_deg_p50_p90=dict(lidar=[float(np.percentile(np.abs(thl), q)) for q in (50, 90)],
                                                 glo30_1=[float(np.percentile(np.abs(th1[gi]), q)) for q in (50, 90)]),
                      slope_diff_deg_p50_p90=[float(np.percentile(np.abs(thl - th1[gi]), q)) for q in (50, 90)],
                      mad_log_ratio=dict(lidar=mad(np.log(v[gi] / lorentz(thl, *pl))), glo30_1=mad(np.log(v[gi] / lorentz(th1[gi], *pg))),
                                         flat=mad(np.log(v[gi]))),
                      fit_lidar=[float(x) for x in pl], fit_glo30=[float(x) for x in pg],
                      speed_at={str(t): dict(lidar=float(lorentz(t, *pl)), glo30_1=float(lorentz(t, *pg))) for t in (-20, -10, 0, 10, 20)})

# 10. per-mille table of the expected-pace curve normalised to flat (feat-x X3 style: grade -100..+100 % in 1 % steps)
vflat = float(lorentz(0.0, *F['mean']))
with open(f'{outd}/pace_table_{seglen}.csv', 'w') as fh:
    fh.write('grade_pct,deg,mean_pace_permille,p10_permille,p50_permille,p90_permille,tobler_permille\n')
    for g in range(-100, 101):
        th = math.degrees(math.atan(g / 100))
        vals = [lorentz(th, *F['mean']), lorentz(th, *F[10]), lorentz(th, *F[50]), lorentz(th, *F[90])]
        tob = float(tobler(np.array(g / 100))) / float(tobler(np.array(0.0)))
        fh.write(f'{g},{th:.2f},' + ','.join(str(int(round(1000 * x / vflat))) for x in vals) + f',{int(round(1000 * tob))}\n')
R['v_flat_mean_pace'] = vflat

json.dump(R, open(f'{outd}/results_{seglen}.json', 'w'), indent=1, default=float)
print(json.dumps({k: R[k] for k in ('areas', 'cal_segments', 'cal_tracks', 'off_segments', 'off_tracks', 'campbell_coverage', 'peak',
                                     'person_factor', 'segment_factor')}, indent=1, default=float))

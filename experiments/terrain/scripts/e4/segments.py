# E4: OSM GPS trackpoint pages -> cleaned walking tracks -> fixed-length segments with terrain attributes.
#
#   segments.py <raw-dir> <e4work-dir> <area> [seglen_m]
#
# Reads <raw-dir>/<area>/p*.gpx, <e4work>/ways.npz, DEM posts from terrexp (<e4work>/out/<area>_<src>_<st>/)
# and WorldCover classes (<e4work>/out/lc_<area>_120/). Writes <e4work>/seg_<area>_<seglen>.npz and a
# stripped track cache <e4work>/tracks_<area>.npz (anonymous track index, lat/lon E7, unix time; no names, no URLs).
import sys, os, glob, re, hashlib, math
import numpy as np, pyproj
from scipy.spatial import cKDTree

TICKS_PER_DEG = 3600 * 120
UTM = pyproj.Transformer.from_crs(4326, 32634, always_xy=True)

# ---------- GPX pages ----------
TRK = re.compile(rb'<trk>(.*?)</trk>', re.S)
PT = re.compile(rb'<trkpt lat="([-0-9.]+)" lon="([-0-9.]+)">\s*<time>([^<]+)</time>')
URL = re.compile(rb'<url>([^<]*)</url>')

def iso_to_unix(b):
    # 2026-01-02T12:00:02Z or with fractional seconds
    s = b.decode()
    y, mo, d = int(s[0:4]), int(s[5:7]), int(s[8:10])
    hh, mm, ss = int(s[11:13]), int(s[14:16]), float(s[17:].rstrip('Z').split('+')[0] or 0)
    # days from civil (Howard Hinnant)
    y2 = y - (mo <= 2); era = y2 // 400; yoe = y2 - era * 400
    doy = (153 * (mo + (-3 if mo > 2 else 9)) + 2) // 5 + d - 1
    doe = yoe * 365 + yoe // 4 - yoe // 100 + doy
    days = era * 146097 + doe - 719468
    return days * 86400 + hh * 3600 + mm * 60 + ss

def read_pages(d):
    """Yields (key, lat[], lon[], t[]) per <trk> with timestamps. key = sha256 of the trace URL (identifiable
    traces) or of page+index (trackable). Raw names and URLs are not kept."""
    out = []
    for fn in sorted(glob.glob(f'{d}/p*.gpx')):
        data = open(fn, 'rb').read()
        for k, m in enumerate(TRK.finditer(data)):
            body = m.group(1)
            pts = PT.findall(body)
            if len(pts) < 2:
                continue
            u = URL.search(body)
            key = hashlib.sha256(u.group(1) if u else f'{os.path.basename(fn)}#{k}'.encode()).hexdigest()[:16]
            lat = np.array([float(p[0]) for p in pts]); lon = np.array([float(p[1]) for p in pts])
            t = np.array([iso_to_unix(p[2]) for p in pts])
            out.append((key, bool(u), lat, lon, t))
    return out

# ---------- rasters on the lattice ----------
class Posts:
    """Dense array over the stored blocks of a terrexp output; values at posts (i, j) = (lat, lon) * TICKS / s."""
    def __init__(self, d, kind):
        bl = np.loadtxt(f'{d}/blocks.txt', dtype=np.int64, ndmin=2)
        s = int(re.search(r'_(\d+)$', d.rstrip('/')).group(1))
        if kind == 'dem':
            raw = np.fromfile(f'{d}/posts_dm.i32', dtype='<i4').reshape(-1, 64, 64)
            none = np.iinfo(np.int32).min
            # stored grid = whole metres, rounded half away from zero (E1 rule)
            m = np.where(raw == none, np.nan, np.sign(raw) * np.floor((np.abs(raw) + 5) / 10)).astype(np.float64)
        else:
            m = np.fromfile(f'{d}/posts_class.u8', dtype=np.uint8).reshape(-1, 64, 64)
        self.bi0, self.bj0 = bl[:, 0].min(), bl[:, 1].min()
        ni = (bl[:, 0].max() - self.bi0 + 1) * 64; nj = (bl[:, 1].max() - self.bj0 + 1) * 64
        self.a = np.full((ni, nj), np.nan if kind == 'dem' else 0, dtype=m.dtype)
        for k, (bi, bj) in enumerate(bl):
            self.a[(bi - self.bi0) * 64:(bi - self.bi0 + 1) * 64, (bj - self.bj0) * 64:(bj - self.bj0 + 1) * 64] = m[k]
        self.s = s
    def idx(self, lat, lon):
        fi = lat * TICKS_PER_DEG / self.s - self.bi0 * 64
        fj = lon * TICKS_PER_DEG / self.s - self.bj0 * 64
        return fi, fj
    def bilinear(self, lat, lon):
        fi, fj = self.idx(lat, lon)
        i = np.floor(fi).astype(int); j = np.floor(fj).astype(int)
        ok = (i >= 0) & (j >= 0) & (i < self.a.shape[0] - 1) & (j < self.a.shape[1] - 1)
        i = np.clip(i, 0, self.a.shape[0] - 2); j = np.clip(j, 0, self.a.shape[1] - 2)
        di = fi - i; dj = fj - j
        a = self.a
        v = (a[i, j] * (1 - di) * (1 - dj) + a[i + 1, j] * di * (1 - dj) + a[i, j + 1] * (1 - di) * dj + a[i + 1, j + 1] * di * dj)
        return np.where(ok, v, np.nan)
    def nearest(self, lat, lon):
        fi, fj = self.idx(lat, lon)
        i = np.clip(np.rint(fi).astype(int), 0, self.a.shape[0] - 1); j = np.clip(np.rint(fj).astype(int), 0, self.a.shape[1] - 1)
        return self.a[i, j]

# ---------- cleaning ----------
GAP_S, JUMP_M = 120.0, 200.0        # split a track at a time gap or a position jump
STOP_WIN_S, STOP_R_M = 60.0, 12.0   # stationary: stays within 12 m for 60 s
FAST_MPS, FAST_SHARE = 2.5, 0.05    # track is not walking if >5 % of its 60 s windows exceed 9 km/h
MED_MPS = 1.8                       # or if its median 60 s speed exceeds 6.5 km/h (jogging)
MAX_MPS = 4.0                       # any 60 s window above 14.4 km/h: vehicle / bike / ski, drop the track

def split_pieces(x, y, t):
    dt = np.diff(t); dd = np.hypot(np.diff(x), np.diff(y))
    cut = np.where((dt <= 0) | (dt > GAP_S) | (dd > JUMP_M) | (dd / np.maximum(dt, 1e-9) > 15))[0]
    b = np.concatenate([[0], cut + 1, [len(t)]])
    return [(b[k], b[k + 1]) for k in range(len(b) - 1) if b[k + 1] - b[k] >= 10]

def window_speed(x, y, t, w=60.0):
    """Speed over the next ~w seconds from each point (displacement / time)."""
    j = np.searchsorted(t, t + w)
    j = np.clip(j, 0, len(t) - 1)
    dt = t[j] - t
    d = np.hypot(x[j] - x, y[j] - y)
    return np.where(dt >= w * 0.5, d / np.maximum(dt, 1e-9), np.nan), dt

def stationary_mask(x, y, t):
    """A point is stationary if every point within +-STOP_WIN_S/2 lies within STOP_R_M of it."""
    n = len(t); st = np.zeros(n, bool)
    lo = np.searchsorted(t, t - STOP_WIN_S / 2); hi = np.searchsorted(t, t + STOP_WIN_S / 2, side='right')
    for k in range(n):
        a, b = lo[k], hi[k]
        if t[b - 1] - t[a] < STOP_WIN_S * 0.8:
            continue
        if np.max(np.hypot(x[a:b] - x[k], y[a:b] - y[k])) < STOP_R_M:
            st[k] = True
    return st

def smooth(v, k=2):
    """Centred moving average over 2k+1 points (edges shrink)."""
    c = np.concatenate([[0.0], np.cumsum(v)])
    n = len(v); i = np.arange(n); a = np.maximum(0, i - k); b = np.minimum(n, i + k + 1)
    return (c[b] - c[a]) / (b - a)

def main():
    raw, work, area = sys.argv[1], sys.argv[2], sys.argv[3]
    seglen = float(sys.argv[4]) if len(sys.argv) > 4 else 50.0
    trks = read_pages(f'{raw}/{area}')
    W = np.load(f'{work}/ways.npz'); tree = cKDTree(np.c_[W['x'], W['y']]); wcls = W['cls']
    dems = {k: Posts(f'{work}/out/{area}_{k}', 'dem') for k in ('glo30_120', 'srtm_120', 'glo30_360')}
    lc = Posts(f'{work}/out/lc_{area}_120', 'lc')
    stats = dict(trk=len(trks), pts=0, pieces=0, walk_pieces=0, drop_fast=0, drop_short=0, stopped_s=0.0, moving_s=0.0)
    seen = {}
    rows = []; cache = []
    for ti, (key, ident, lat, lon, t) in enumerate(trks):
        stats['pts'] += len(t)
        o = np.argsort(t, kind='stable'); lat, lon, t = lat[o], lon[o], t[o]
        x, y = UTM.transform(lon, lat)
        x = np.asarray(x); y = np.asarray(y)
        for a, b in split_pieces(x, y, t):
            stats['pieces'] += 1
            px, py, pt, pla, plo = x[a:b], y[a:b], t[a:b], lat[a:b], lon[a:b]
            if pt[-1] - pt[0] < 300:
                stats['drop_short'] += 1; continue
            v60, _ = window_speed(px, py, pt)
            vv = v60[~np.isnan(v60)]
            if len(vv) == 0 or np.mean(vv > FAST_MPS) > FAST_SHARE or np.max(vv) > MAX_MPS or np.median(vv) > MED_MPS:
                stats['drop_fast'] += 1; continue
            # same piece fetched twice (a trace uploaded twice, or overlapping pages): drop by start time + position
            sig = (round(pt[0]), round(px[0]), round(py[0]))
            if sig in seen:
                continue
            seen[sig] = 1
            stats['walk_pieces'] += 1
            st = stationary_mask(px, py, pt)
            cache.append((len(cache), pla, plo, pt))
            # moving runs between stationary points
            mv = ~st
            edges = np.flatnonzero(np.diff(np.concatenate([[0], mv.astype(int), [0]])))
            stats['stopped_s'] += float(np.sum(np.diff(pt)[st[:-1]]))
            for r0, r1 in zip(edges[0::2], edges[1::2]):
                if r1 - r0 < 5:
                    continue
                sx, sy, stt = smooth(px[r0:r1]), smooth(py[r0:r1]), pt[r0:r1]
                sla, slo = smooth(pla[r0:r1]), smooth(plo[r0:r1])
                dist = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(sx), np.diff(sy)))])
                stats['moving_s'] += float(stt[-1] - stt[0])
                # cut at whole multiples of seglen along the smoothed path (interpolated positions and times)
                nseg = int(dist[-1] // seglen)
                if nseg == 0:
                    continue
                cuts = np.arange(nseg + 1) * seglen
                cx = np.interp(cuts, dist, sx); cy = np.interp(cuts, dist, sy); ct = np.interp(cuts, dist, stt)
                cla = np.interp(cuts, dist, sla); clo = np.interp(cuts, dist, slo)
                # per-point trail distance, for segment classification
                dpt, ipt = tree.query(np.c_[sx, sy], distance_upper_bound=200.0)
                dpt = np.minimum(dpt, 200.0)
                ptcls = np.where(ipt < len(wcls), wcls[np.minimum(ipt, len(wcls) - 1)], 0)
                seg_of_pt = np.minimum((dist // seglen).astype(int), nseg - 1)
                z = {k: d.bilinear(cla, clo) for k, d in dems.items()}
                mla = (cla[:-1] + cla[1:]) / 2; mlo = (clo[:-1] + clo[1:]) / 2
                cover = lc.nearest(mla, mlo)
                for s in range(nseg):
                    m = seg_of_pt == s
                    if not m.any():
                        dmed = dmax = 200.0; c = 0; on_share = 0.0
                    else:
                        dd = dpt[m]; dmed = float(np.median(dd)); dmax = float(dd.max())
                        on_share = float(np.mean(dd <= 15.0))
                        cc = ptcls[m][dd <= 15.0]
                        c = int(np.bincount(cc, minlength=8).argmax()) if len(cc) else 0
                    dt = ct[s + 1] - ct[s]
                    chord = math.hypot(cx[s + 1] - cx[s], cy[s + 1] - cy[s])
                    rows.append((len(cache) - 1, ti, ident, stt[0], ct[s], dt, seglen, chord,
                                 z['glo30_120'][s], z['glo30_120'][s + 1], z['srtm_120'][s], z['srtm_120'][s + 1],
                                 z['glo30_360'][s], z['glo30_360'][s + 1], dmed, dmax, on_share, c, int(cover[s]),
                                 mla[s], mlo[s], cla[s], clo[s], cla[s + 1], clo[s + 1]))
    # OSM ways come from the Małopolskie, Prešov and Žilina extracts: flag segments at least 150 m inside their union
    from shapely import wkt as W_, contains_xy
    from shapely.ops import transform as tf_
    poly = W_.loads(open(os.environ.get('E4_BOUNDARY', f'{work}/coverage.wkt')).read())
    inner = tf_(lambda a, b: UTM.transform(a, b), poly).buffer(-150.0)
    A0 = np.array(rows, dtype=np.float64) if rows else np.zeros((0, 25))
    if len(A0):
        mx, my = UTM.transform(A0[:, 20], A0[:, 19])
        inside = contains_xy(inner, np.asarray(mx), np.asarray(my)).astype(np.float64)
    else:
        inside = np.zeros(0)
    rows = [tuple(r) + (f,) for r, f in zip(A0, inside)]
    names = ['track', 'trk', 'ident', 't_run0', 't0', 'dt', 'len', 'chord', 'z_g1a', 'z_g1b', 'z_s1a', 'z_s1b',
             'z_g3a', 'z_g3b', 'dmed', 'dmax', 'on_share', 'wcls', 'cover', 'lat', 'lon', 'lat_a', 'lon_a', 'lat_b', 'lon_b', 'inside']
    A = np.array(rows, dtype=np.float64) if rows else np.zeros((0, len(names)))
    np.savez_compressed(f'{work}/seg_{area}_{int(seglen)}.npz', **{n: A[:, k] for k, n in enumerate(names)})
    if seglen == 50.0:
        np.savez_compressed(f'{work}/tracks_{area}.npz',
                            idx=np.concatenate([np.full(len(c[3]), c[0], np.int32) for c in cache]) if cache else np.zeros(0),
                            lat_e7=np.concatenate([np.rint(c[1] * 1e7).astype(np.int64) for c in cache]) if cache else np.zeros(0),
                            lon_e7=np.concatenate([np.rint(c[2] * 1e7).astype(np.int64) for c in cache]) if cache else np.zeros(0),
                            t=np.concatenate([c[3] for c in cache]) if cache else np.zeros(0))
    stats['segments'] = len(rows)
    print(area, seglen, stats)

if __name__ == '__main__':
    main()

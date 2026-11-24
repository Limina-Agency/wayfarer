# E5 independent check: re-implements the cell times, edge costs and the full-field Dijkstra of
# `terrexp surface` in plain Python (heapq) from the prep files and E4's table, and compares the
# field with terrexp's field_<profile>.u32 (deciseconds).
#   python3 check_surface.py <prep-dir> <spacing-ticks> <pace-csv> <surface-out-dir>
import sys, math, heapq, json, numpy as np
P, st, csv, O = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
TPD = 432000; R_MM = 6371008800.0
blocks = sorted(tuple(map(int, l.split())) for l in open(f'{P}/blocks.txt') if l.strip())
bi0 = min(b[0] for b in blocks); bj0 = min(b[1] for b in blocks)
H = (max(b[0] for b in blocks) - bi0 + 1) * 64; W = (max(b[1] for b in blocks) - bj0 + 1) * 64
def dense(a):
    a = a.reshape(-1, 64, 64); g = np.zeros((H, W), a.dtype)
    for k, (bi, bj) in enumerate(blocks):
        g[(bi-bi0)*64:(bi-bi0+1)*64, (bj-bj0)*64:(bj-bj0+1)*64] = a[k]
    return g
dm = dense(np.fromfile(f'{P}/dem_dm.i32', np.int32)).astype(np.int64)
steep = dense(np.fromfile(f'{P}/steep.u8', np.uint8))
lc = dense(np.fromfile(f'{P}/lc4.u8', np.uint8))
ln = dense(np.fromfile(f'{P}/lines.u16', np.uint16)).astype(np.int64)
rows = [l.split(',') for l in open(csv).read().split('\n')[1:] if l]
walker_rel = [int(r[2]) for r in rows]

def rnd(num, den):  # half away from zero
    return (2*num + den)//(2*den) if num >= 0 else -((-2*num + den)//(2*den))
def t_of(v_mmps, unit):
    return 0 if v_mmps <= 0 else int(math.floor(1e9 / v_mmps / unit + 0.5))
prof = {
    'walker': dict(unit=1000,
        cover=[t_of(1040*x, 1000) for x in [0.6, 0.6, 0.35, 0.8, 0.7, 0.5, 0.4, 0.5, 0, 0.25, 0.25, 0.7, 0.15, 0.4, 0.4, 0.2]],
        way=[t_of(1040*x, 1000) for x in [0, 1.08, 1.08, 1.09, 1.00, 0.85, 0.83, 0.92]],
        steep=[1000]*8 + [800, 600, 400, 250, 0, 0, 0, 0], barrier=[0, 600, 500, 500, 0], rel=walker_rel),
    'vehicle': dict(unit=100,
        cover=[t_of(x/3.6*1000, 100) for x in [0, 0, 3, 12, 8, 5, 6, 0, 0, 0, 0, 8, 0, 0, 0, 0]],
        way=[t_of(x/3.6*1000, 100) for x in [0, 60, 40, 20, 8, 0, 0, 0]],
        steep=[1000]*4 + [700, 400] + [0]*10, barrier=[0, 300, 0, 0, 0],
        rel=[1000 if abs(g) <= 10 else max(300, 1000 - 23*(abs(g) - 10)) for g in range(-100, 101)]),
}
MOVES = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1), (1, 2), (2, 1), (2, -1), (1, -2), (-1, -2), (-2, -1), (-2, 1), (-1, 2)]
def crossed(dr, dc):
    if abs(dr) == 1 and abs(dc) == 2: return [(0, dc//2), (dr, dc//2)]
    if abs(dr) == 2 and abs(dc) == 1: return [(dr//2, 0), (dr//2, dc)]
    return []
dy = round(st / TPD * math.pi / 180 * R_MM)
lat0 = bi0 * 64 * st
L = {}
for k in range(-2, 2*H + 2):
    phi = (lat0 + k*st//2) / TPD * math.pi / 180
    dx = round(st / TPD * math.pi / 180 * R_MM * math.cos(phi))
    L[k] = [math.isqrt((dr*dy)**2 + (dc*dx)**2) for dr, dc in MOVES]

res = {}
for name, p in prof.items():
    t = np.zeros((H, W), np.int64)
    for r in range(H):
        for c in range(W):
            w_ = ln[r, c] & 0xFF; v = 0
            ways = [p['way'][k] for k in range(8) if w_ >> k & 1 and p['way'][k] > 0]
            if ways: t[r, c] = min(ways); continue
            if w_ & 1: continue
            tc = p['cover'][lc[r, c] & 15]
            if tc == 0: continue
            rel = 1000
            sr = 1000 if steep[r, c] == 255 else p['steep'][steep[r, c] & 15]
            if sr == 0: continue
            rel = rel * sr // 1000
            blocked = False
            for k in range(5):
                if ln[r, c] >> (8 + k) & 1:
                    if p['barrier'][k] == 0: blocked = True; break
                    rel = rel * p['barrier'][k] // 1000
            if blocked: continue
            t[r, c] = min(65535, max(1, rnd(tc * 1000, max(rel, 1))))
    # IPP as terrexp: passable cell nearest the centre, scanning rings row-major
    cr, cc = H//2, W//2; ipp = None
    for rad in range(H//2):
        for dr in range(-rad, rad+1):
            for dc in range(-rad, rad+1):
                if ipp is None and t[cr+dr, cc+dc] > 0: ipp = (cr+dr)*W + cc+dc
        if ipp is not None: break
    dist = {ipp: 0}; hq = [(0, ipp)]; done = set()
    while hq:
        d, i = heapq.heappop(hq)
        if i in done: continue
        done.add(i)
        r, c = divmod(i, W)
        for m, (dr, dc) in enumerate(MOVES):
            nr, nc = r + dr, c + dc
            if not (0 <= nr < H and 0 <= nc < W): continue
            ta, tz = t[r, c], t[nr, nc]
            if ta == 0 or tz == 0: continue
            s_ = int(ta + tz); n = 2; ok = True
            for er, ec in crossed(dr, dc):
                tx = t[r+er, c+ec]
                if tx == 0: ok = False; break
                s_ += int(tx); n += 1
            if not ok: continue
            l = L[2*r + dr][m]
            g = max(-100, min(100, rnd((int(dm[nr, nc]) - int(dm[r, c])) * 100 * 100, l)))
            cost = max(1, rnd(l * s_ * p['unit'], n * p['rel'][g + 100]))
            j = nr*W + nc; nd = d + cost
            if nd < dist.get(j, 1 << 62):
                dist[j] = nd; heapq.heappush(hq, (nd, j))
    ref = np.full(H*W, 0xFFFFFFFF, np.uint64)
    for i, d in dist.items(): ref[i] = rnd(d, 100000)
    got = np.fromfile(f'{O}/field_{name}.u32', np.uint32).astype(np.uint64)
    reach = ref != 0xFFFFFFFF
    diff = np.abs(ref[reach].astype(np.int64) - got[reach].astype(np.int64))
    res[name] = {'cells': H*W, 'reached_ref': int(reach.sum()), 'reach_agree': bool(((got != 0xFFFFFFFF) == reach).all()),
                 'exact_share': round(float((diff == 0).mean()), 6), 'max_abs_diff_ds': int(diff.max()), 'passable_ref': int((t > 0).sum())}
    print(name, res[name], file=sys.stderr)
print(json.dumps(res))

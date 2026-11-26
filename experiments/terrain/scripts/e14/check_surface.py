# E14: an independent check of the incident surface contract (docs/design/incident-surface.md,
# draft v0), written from the document. It reads every WIS0 surface and WISD delta of the E14 run,
# decodes the planes, recomputes block, content, recipe and terrain hashes, applies each delta, costs
# the sample blocks of the team step from E10's fused state with the profile tables embedded in the
# surface, and recomputes two walker routes (cost, canonical path, route confidence).
#   python3 -I check_surface.py <prep-dir> <e14-out-dir> <e14.json>      (needs numpy and blake3)
import sys, json, zlib, struct, math, heapq, glob, os
import numpy as np
from blake3 import blake3

prep, out, js = sys.argv[1:4]
S, BP, NH, NL = 4096, 4096, 8, 3
run = json.load(open(js))

def varint(b, p):
    v = s = 0
    while True:
        x = b[p]; p += 1
        v |= (x & 0x7F) << s; s += 7
        if x < 0x80:
            return v, p

def rnd(num, den):  # half away from zero
    return (2 * num + den) // (2 * den) if num >= 0 else -((-2 * num + den) // (2 * den))

def palette_runs(b):
    n, p = varint(b, 0)
    pal = []
    for _ in range(n):
        v, p = varint(b, p); pal.append(v)
    k = max(1, (max(n, 2) - 1).bit_length())
    out = []
    while p < len(b):
        x, p = varint(b, p)
        out += [pal[x & ((1 << k) - 1)]] * ((x >> k) + 1)
    assert len(out) == BP
    return out

def plane(b, wide):
    if b[0] == 0:
        v = palette_runs(b[1:])
    else:
        assert b[0] == 1
        raw = zlib.decompress(b[1:])
        v = list(struct.unpack(f'<{BP}H', raw)) if wide else list(raw)
    assert wide or max(v) < 256
    return v

def header(b, p):
    h = {}
    h['incident'] = b[p:p + 16]; p += 16
    h['spacing'], h['epoch'], h['t'], h['same_until'] = struct.unpack_from('<IIqq', b, p); p += 24
    h['terrain'], h['prior'], h['overlay'] = b[p:p + 32], b[p + 32:p + 64], b[p + 64:p + 96]; p += 96
    h['params'] = tuple(b[p:p + 3]); p += 3
    n, p = varint(b, p); h['applied'] = []
    for _ in range(n):
        hs = b[p:p + 32]; kind = b[p + 32]; obs, vu = struct.unpack_from('<qq', b, p + 33); p += 49
        h['applied'].append((hs, kind, obs, vu))
    n, p = varint(b, p); h['left'] = []
    for _ in range(n):
        h['left'].append((b[p:p + 32], b[p + 32])); p += 33
    np_ = b[p]; p += 1; h['profiles'] = []
    for _ in range(np_):
        n, p = varint(b, p); h['profiles'].append(b[p:p + n]); p += n
    return h, p

def profile(b):
    assert b[:4] == b'WIP0'
    n, p = varint(b, 4); name = b[p:p + n].decode(); p += n
    unit, = struct.unpack_from('<Q', b, p); p += 8
    def u32s(k):
        nonlocal p
        v = list(struct.unpack_from(f'<{k}I', b, p)); p += 4 * k; return v
    cover, way, steep, barrier = u32s(16), u32s(8), u32s(16), u32s(5)
    ng, = struct.unpack_from('<H', b, p); p += 2
    grade = u32s(ng)
    block_at, = struct.unpack_from('<I', b, p); p += 4
    hb_at, hb = b[p], b[p + 1]; p += 2
    hk = u32s(8)
    lev_block = list(struct.unpack_from('<3H', b, p)); p += 6
    lev_k = u32s(3)
    obst, = struct.unpack_from('<I', b, p); p += 4
    assert p == len(b)
    return dict(name=name, unit=unit, cover=cover, way=way, steep=steep, barrier=barrier, grade=grade, block_at=block_at,
                hb_at=hb_at, hb=hb, hk=hk, lev_block=lev_block, lev_k=lev_k, obst=obst)

def block(key, b, nprof):
    n = b[0]; p = 1
    table = [b[p + 32 * k:p + 32 * k + 32] for k in range(n)]; p += 32 * n
    planes = []
    for wide in [False] * 3 + [True, False, False] * nprof:
        ln, p = varint(b, p); planes.append(plane(b[p:p + ln], wide)); p += ln
    assert p == len(b)
    canon = struct.pack('<QB', key, n) + b''.join(table)
    for k, (v, wide) in enumerate(zip(planes, [False] * 3 + [True, False, False] * nprof)):
        canon += struct.pack(f'<{BP}H', *v) if wide else bytes(v)
    return dict(table=table, planes=planes, hash=blake3(canon).digest())

def content_hash(h, blocks):
    c = b'wis0/content' + struct.pack('<HI', 0, h['spacing']) + h['terrain'] + bytes([len(h['profiles'])])
    c += b''.join(blake3(p).digest() for p in h['profiles'])
    c += struct.pack('<I', len(blocks)) + b''.join(struct.pack('<Q', k) + blocks[k]['hash'] for k in sorted(blocks))
    return blake3(c).digest()

def recipe_hash(hraw, keys):
    return blake3(b'wis0/recipe' + struct.pack('<HHH', 0, 0, 0) + hraw + struct.pack('<I', len(keys)) +
                  b''.join(struct.pack('<Q', k) for k in sorted(keys))).digest()

def read_index(b, p, nprof):
    n, p = varint(b, p); idx = []
    for _ in range(n):
        k, ln = struct.unpack_from('<QI', b, p); idx.append((k, ln, b[p + 12:p + 44])); p += 44
    blocks = {}
    for k, ln, hs in idx:
        blk = block(k, b[p:p + ln], nprof); p += ln
        assert blk['hash'] == hs, 'block hash'
        blocks[k] = blk
    return blocks, p

def read_surface(path):
    b = open(path, 'rb').read()
    assert b[:4] == b'WIS0' and struct.unpack_from('<HH', b, 4) == (0, 0)
    h, p = header(b, 8)
    hraw = b[8:p]
    recipe, content = b[p:p + 32], b[p + 32:p + 64]; p += 64
    blocks, p = read_index(b, p, len(h['profiles']))
    assert p == len(b)
    assert content_hash(h, blocks) == content, 'content hash'
    assert recipe_hash(hraw, blocks.keys()) == recipe, 'recipe hash'
    return dict(h=h, hraw=hraw, blocks=blocks, content=content, recipe=recipe)

def apply_delta(old, path):
    b = open(path, 'rb').read()
    assert b[:4] == b'WISD' and b[8:40] == old['content']
    h, p = header(b, 40); hraw = b[40:p]
    recipe, content = b[p:p + 32], b[p + 32:p + 64]; p += 64
    blocks = dict(old['blocks'])
    n, p = varint(b, p)
    for _ in range(n):
        k, op = struct.unpack_from('<QB', b, p); p += 9
        if op == 0:
            del blocks[k]; continue
        ln, = struct.unpack_from('<I', b, p); hs = b[p + 4:p + 36]; p += 36
        blk = block(k, b[p:p + ln], len(h['profiles'])); p += ln
        assert blk['hash'] == hs
        blocks[k] = blk
    assert p == len(b)
    assert content_hash(h, blocks) == content and recipe_hash(hraw, blocks.keys()) == recipe
    return dict(h=h, hraw=hraw, blocks=blocks, content=content, recipe=recipe)

def key(bi, bj):
    return ((bi + 2**31) << 32) | (bj + 2**31)

res = {}
# ---- terrain hash from the prep files ----
blist = sorted(tuple(map(int, l.split())) for l in open(f'{prep}/blocks.txt') if l.strip())
th = b'wis0/terrain' + b''.join(struct.pack('<Q', key(*x)) for x in blist)
for f in ['dem_dm.i32', 'steep.u8', 'lc4.u8', 'conf.u8', 'lines.u16']:
    th += open(f'{prep}/{f}', 'rb').read()
terrain = blake3(th).digest()

# ---- every surface and delta ----
files = sorted(glob.glob(f'{out}/step*.wis'), key=lambda f: int(os.path.basename(f)[4:].split('_')[0]))
surf, prev = {}, None
for f in files:
    s = read_surface(f)
    name = os.path.basename(f)[:-4].split('_', 1)[1]
    assert s['h']['terrain'] == terrain, 'terrain hash'
    assert s['content'].hex() == run['h_content_' + name] and s['recipe'].hex() == run['h_recipe_' + name]
    if prev is not None:
        d = apply_delta(prev, f + 'd')
        assert d['content'] == s['content'] and d['recipe'] == s['recipe']
    surf[name] = s; prev = s
res['surfaces_verified'] = len(files)
res['deltas_applied'] = len(files) - 1

# ---- cost rules on the sample blocks of the team step ----
team = surf['team']; H = team['h']
profs = [profile(p) for p in H['profiles']]
def dense_prep(name, dt):
    return np.fromfile(f'{prep}/{name}', dt).reshape(-1, BP)
steep_b, lines_b = dense_prep('steep.u8', np.uint8), dense_prep('lines.u16', np.uint16)
bidx = {key(*x): k for k, x in enumerate(blist)}
PATH, FENCE, MOTORWAY = 1 << 4, 1 << 11, 1
RIVER_ETC = [1 << 8, 1 << 9, 1 << 10, 1 << 11, 1 << 12]
OBST, WASH, TEAM = 1 << 13, 1 << 14, 1 << 15
SUPER = [(0, 0), (4, 1)]

def risk_of(pb, haz, blocking):
    num, den = S - pb, S
    for s_ in range(NH):
        if blocking[s_]:
            num *= 255 - haz[s_]; den *= 255
    return 255 - rnd(255 * num, den)

def cell(pf, P, haz, lev, mask, steep, lines):
    l = lines
    if mask & 1: l |= FENCE
    if mask & (2 | 4): l |= OBST
    if mask & 8: l |= WASH
    if mask & 16: l |= PATH
    if mask & 32: l |= TEAM
    if l & TEAM: return 0, 255, 9 << 4
    if l & WASH: return 0, 255, 8 << 4
    blocking = [bool(pf['hb'] >> s_ & 1) for s_ in range(NH)]
    for h_, lv in SUPER:
        if lev[lv][1] > 0: blocking[h_] = False
    ways = [pf['way'][k] for k in range(8) if l >> k & 1 and pf['way'][k] > 0]
    pb = 0
    if ways:
        t = min(ways)
    elif l & MOTORWAY:
        return 0, 255, 4 << 4
    else:
        acc = 0
        for k in range(16):
            if pf['cover'][k] == 0: pb += P[k]
            else: acc += pf['cover'][k] * P[k]
        if pb >= pf['block_at']: return 0, risk_of(pb, haz, blocking), 1 << 4
        t = rnd(acc, S - pb)
        sr = 1000 if steep == 255 else pf['steep'][steep & 15]
        if sr == 0: return 0, 255, 2 << 4
        rel = sr
        for k, bit in enumerate(RIVER_ETC):
            if l & bit:
                if pf['barrier'][k] == 0: return 0, 255, 3 << 4 | k
                rel = rel * pf['barrier'][k] // 1000
        t = rnd(t * 1000, max(rel, 1))
    for s_ in range(NH):
        if blocking[s_] and haz[s_] >= pf['hb_at']: return 0, risk_of(pb, haz, blocking), 5 << 4 | s_
    hk = sum(pf['hk'][s_] * haz[s_] for s_ in range(NH))
    if hk: t = rnd(t * (255000 + hk), 255000)
    for s_ in range(NL):
        v, c = lev[s_]
        if c == 0: continue
        if v > pf['lev_block'][s_]: return 0, 255, 6 << 4 | s_
        if pf['lev_k'][s_]: t = rnd(t * (1000 + v * pf['lev_k'][s_]), 1000)
    if l & OBST:
        if pf['obst'] == 0: return 0, 255, 7 << 4
        t = rnd(t * 1000, pf['obst'])
    return min(65535, max(1, t)), risk_of(pb, haz, blocking), 0

fs = open(f'{out}/fused_sample.bin', 'rb').read()
REC = 8 + BP * (32 + 8 + 9 + 4)
assert len(fs) % REC == 0
ghash = {k + 1: a[0] for k, a in enumerate(H['applied'])}
bad = posts = 0
from collections import Counter
whys = Counter()
for r in range(len(fs) // REC):
    o = r * REC
    k, = struct.unpack_from('<Q', fs, o); o += 8
    kb = bidx[k]
    got = team['blocks'][k]
    srcs = []
    want = [[] for _ in range(3 + 3 * len(profs))]
    for q in range(BP):
        P = struct.unpack_from('<16H', fs, o); o += 32
        haz = fs[o:o + 8]; o += 8
        lev = [struct.unpack_from('<HB', fs, o + 3 * i) for i in range(3)]; o += 9
        mask, cert, src = struct.unpack_from('<HBB', fs, o); o += 4
        top = max(range(16), key=lambda c: (P[c], -c))
        want[0].append(top); want[1].append(cert); srcs.append(src)
        for n, pf in enumerate(profs):
            t, rk, why = cell(pf, P, haz, lev, mask, int(steep_b[kb, q]), int(lines_b[kb, q]))
            want[3 + 3 * n].append(t); want[4 + 3 * n].append(rk); want[5 + 3 * n].append(why)
            whys[(pf['name'], why >> 4)] += 1
    used = sorted(set(s_ for s_ in srcs if s_ not in (0, 255)))
    table = [ghash[s_] for s_ in used]
    loc = {s_: n + 1 for n, s_ in enumerate(used)}
    want[2] = [loc.get(s_, s_) for s_ in srcs]
    posts += BP
    if table != got['table'] or any(list(a) != list(b) for a, b in zip(want, got['planes'])):
        bad += 1
        print('block', k, 'differs', [n for n, (a, b) in enumerate(zip(want, got['planes'])) if list(a) != list(b)], file=sys.stderr)
res['cost_sample_blocks'] = len(fs) // REC
res['cost_sample_posts'] = posts
res['cost_blocks_differing'] = bad
res['closed_reasons_in_sample'] = {f'{a}:{b}': v for (a, b), v in sorted(whys.items()) if b}

# ---- two walker routes on the team step ----
bi0, bj0 = min(x[0] for x in blist), min(x[1] for x in blist)
Hh = (max(x[0] for x in blist) - bi0 + 1) * 64; W = (max(x[1] for x in blist) - bj0 + 1) * 64
def dense(planes_of):
    g = np.zeros((Hh, W), np.int64)
    for (bi, bj) in blist:
        g[(bi - bi0) * 64:(bi - bi0 + 1) * 64, (bj - bj0) * 64:(bj - bj0 + 1) * 64] = np.array(planes_of(key(bi, bj))).reshape(64, 64)
    return g
t = dense(lambda k: team['blocks'][k]['planes'][3])
cert = dense(lambda k: team['blocks'][k]['planes'][1])
risk = dense(lambda k: team['blocks'][k]['planes'][4])
gpos = {a[0]: n + 1 for n, a in enumerate(H['applied'])}
def src_of(k):
    b = team['blocks'][k]
    return [0 if s_ == 0 else 255 if s_ == 255 else gpos[b['table'][s_ - 1]] for s_ in b['planes'][2]]
src = dense(src_of)
dm = np.fromfile(f'{prep}/dem_dm.i32', np.int32).reshape(-1, 64, 64)
dmg = np.zeros((Hh, W), np.int64)
for k, (bi, bj) in enumerate(blist):
    dmg[(bi - bi0) * 64:(bi - bi0 + 1) * 64, (bj - bj0) * 64:(bj - bj0 + 1) * 64] = dm[k]
pf = profs[0]
TPD = 432000; R_MM = 6371008800.0; st = H['spacing']
MOVES = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1), (1, 2), (2, 1), (2, -1), (1, -2), (-1, -2), (-2, -1), (-2, 1), (-1, 2)]
def crossed(dr, dc):
    if abs(dr) == 1 and abs(dc) == 2: return [(0, dc // 2), (dr, dc // 2)]
    if abs(dr) == 2 and abs(dc) == 1: return [(dr // 2, 0), (dr // 2, dc)]
    return []
dy = round(st / TPD * math.pi / 180 * R_MM)
lat0 = bi0 * 64 * st
L = {}
for kk in range(-2, 2 * Hh + 2):
    phi = (lat0 + kk * st // 2) / TPD * math.pi / 180
    dx = round(st / TPD * math.pi / 180 * R_MM * math.cos(phi))
    L[kk] = [math.isqrt((dr * dy) ** 2 + (dc * dx) ** 2) for dr, dc in MOVES]
tl = t.tolist(); dml = dmg.tolist()
def cost(r, c, m):
    dr, dc = MOVES[m]; nr, nc = r + dr, c + dc
    if not (0 <= nr < Hh and 0 <= nc < W): return None
    ta, tz = tl[r][c], tl[nr][nc]
    if ta == 0 or tz == 0: return None
    s_, n = ta + tz, 2
    for er, ec in crossed(dr, dc):
        tx = tl[r + er][c + ec]
        if tx == 0: return None
        s_ += tx; n += 1
    l = L[2 * r + dr][m]
    g = max(-100, min(100, rnd((dml[nr][nc] - dml[r][c]) * 100 * 100, l)))
    return max(1, rnd(l * s_ * pf['unit'], n * pf['grade'][g + 100]))
inp = json.load(open(f'{out}/fusion_inputs.json'))
ipp = inp['ipp']
routes_checked = []
for tg in [1, 4]:
    want = next(x for x in run['routes'] if x['step'] == 'team' and x['target'] == tg)['walker']
    dst = inp['targets'][tg]
    dist = {ipp: 0}; hq = [(0, ipp)]; done = set()
    while hq:
        d, i = heapq.heappop(hq)
        if i in done: continue
        done.add(i)
        if i == dst: break
        r, c = divmod(i, W)
        for m in range(16):
            x = cost(r, c, m)
            if x is None: continue
            j = (r + MOVES[m][0]) * W + c + MOVES[m][1]
            if d + x < dist.get(j, 1 << 62):
                dist[j] = d + x; heapq.heappush(hq, (d + x, j))
    assert dist[dst] == want['cost_us'], (dist[dst], want['cost_us'])
    # canonical path: from dst back, the predecessor with d(u) + c(u, v) = d(v), smallest index
    path = []; v = dst
    while v != ipp:
        vr, vc = divmod(v, W); best = None
        for m, (dr, dc) in enumerate(MOVES):
            ur, uc = vr - dr, vc - dc
            if not (0 <= ur < Hh and 0 <= uc < W): continue
            u = ur * W + uc
            if u not in dist: continue
            x = cost(ur, uc, m)
            if x is not None and dist[u] + x == dist[v] and (best is None or u < best[0]):
                best = (u, m)
        path.append(best); v = best[0]
    path.reverse()
    wsum = csum = lay = tm = risky = ln = 0; minc, maxr = 255, 0; lays = set(); edge = False
    for u, m in path:
        r, c = divmod(u, W); dr, dc = MOVES[m]
        l = L[2 * r + dr][m]; ln += l
        cells = [(r, c), (r + dr, c + dc)] + [(r + er, c + ec) for er, ec in crossed(dr, dc)]
        wt = 4 * l // len(cells)
        for cr, cc in cells:
            edge |= cr in (0, Hh - 1) or cc in (0, W - 1)
            wsum += wt; csum += wt * int(cert[cr, cc]); minc = min(minc, int(cert[cr, cc]))
            sv = int(src[cr, cc])
            if sv == 255: tm += wt
            elif sv: lay += wt; lays.add(sv)
            rk = int(risk[cr, cc]); maxr = max(maxr, rk)
            if rk: risky += wt
    pm = lambda x: rnd(1000 * x, wsum)
    got = dict(cost_us=dist[dst], moves=len(path), mean_cert=rnd(csum, wsum), min_cert=minc, layer_permille=pm(lay),
               team_permille=pm(tm), max_risk=maxr, risky_permille=pm(risky), layers=sorted(lays), at_edge=edge)
    same = all(got[k] == want[k] for k in got)
    ph = blake3(b''.join(struct.pack('<QQ', u, m) for u, m in path)).hexdigest()
    same &= ph == run[f'h_route_5_{tg}']
    routes_checked.append({'target': tg, 'settled': len(done), 'agree': same, **got})
res['routes'] = routes_checked
ok = res['cost_blocks_differing'] == 0 and all(r['agree'] for r in routes_checked)
res['all_agree'] = ok
print(json.dumps(res, indent=1))
sys.exit(0 if ok else 1)

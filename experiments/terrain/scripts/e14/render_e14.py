# E14 pictures: the planes of a stored incident surface (top class, certainty, source layer, walker time per
# metre with closed cells by reason, walker risk, 4x4 time per metre), and a close-up around the incident point
# with walker routes on the base-only surface and after the team step.
#   python3 -I render_e14.py <prep-dir> <e14-out-dir> <out-dir>      (needs numpy, Pillow)
import sys, struct, zlib, math, heapq, json, os
import numpy as np
from PIL import Image, ImageDraw, ImageFont

prep, out, dst = sys.argv[1:4]
os.makedirs(dst, exist_ok=True)
BP = 4096
F = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
font, small, big = ImageFont.truetype(F, 20), ImageFont.truetype(F, 15), ImageFont.truetype(F, 26)

def varint(b, p):
    v = s = 0
    while True:
        x = b[p]; p += 1; v |= (x & 0x7F) << s; s += 7
        if x < 0x80: return v, p

def plane(b, wide):
    if b[0] == 1:
        raw = zlib.decompress(b[1:])
        return np.frombuffer(raw, '<u2' if wide else 'u1').astype(np.int64)
    n, p = varint(b, 1); pal = []
    for _ in range(n):
        v, p = varint(b, p); pal.append(v)
    k = max(1, (max(n, 2) - 1).bit_length()); o = []
    while p < len(b):
        x, p = varint(b, p); o += [pal[x & ((1 << k) - 1)]] * ((x >> k) + 1)
    return np.array(o, np.int64)

def read(path):
    b = open(path, 'rb').read(); p = 8 + 16 + 24 + 96 + 3
    n, p = varint(b, p); applied = []
    for _ in range(n):
        applied.append(b[p:p + 32]); p += 49
    n, p = varint(b, p); p += 33 * n
    npf = b[p]; p += 1
    profs = []
    for _ in range(npf):
        n, p = varint(b, p); profs.append(b[p:p + n]); p += n
    p += 64
    n, p = varint(b, p); idx = []
    for _ in range(n):
        k, ln = struct.unpack_from('<QI', b, p); idx.append((k, ln)); p += 44
    blocks = {}
    for k, ln in idx:
        q = b[p:p + ln]; p += ln
        nt = q[0]; table = [q[1 + 32 * i:33 + 32 * i] for i in range(nt)]; r = 1 + 32 * nt; pl = []
        for wide in [False] * 3 + [True, False, False] * npf:
            m, r = varint(q, r); pl.append(plane(q[r:r + m], wide)); r += m
        blocks[k] = (table, pl)
    return applied, profs, blocks

blist = sorted(tuple(map(int, l.split())) for l in open(f'{prep}/blocks.txt') if l.strip())
bi0, bj0 = min(x[0] for x in blist), min(x[1] for x in blist)
H = (max(x[0] for x in blist) - bi0 + 1) * 64; W = (max(x[1] for x in blist) - bj0 + 1) * 64
key = lambda bi, bj: ((bi + 2**31) << 32) | (bj + 2**31)

def dense(blocks, f):
    g = np.zeros((H, W), np.int64)
    for bi, bj in blist:
        g[(bi - bi0) * 64:(bi - bi0 + 1) * 64, (bj - bj0) * 64:(bj - bj0 + 1) * 64] = f(blocks[key(bi, bj)]).reshape(64, 64)
    return g

def grids(path):
    applied, profs, blocks = read(path)
    pos = {h: n + 1 for n, h in enumerate(applied)}
    def src(bl):
        table, pl = bl
        return np.array([0 if s == 0 else 255 if s == 255 else pos[table[s - 1]] for s in pl[2]])
    g = {n: dense(blocks, lambda bl, i=i: bl[1][i]) for i, n in enumerate(['top', 'cert', 'src_local', 'wt', 'wrisk', 'wwhy', 'vt', 'vrisk', 'vwhy'])}
    g['src'] = dense(blocks, src)
    return g, profs

base, profs = grids(f'{out}/step0_base.wis')
team, _ = grids(f'{out}/step5_team.wis')
run = json.load(open(f'{out}/fusion_inputs.json'))
ipp, targets = run['ipp'], run['targets']

CLASS = {0: (0, 0, 0), 1: (34, 102, 51), 2: (140, 150, 60), 3: (170, 210, 120), 4: (235, 215, 110), 5: (190, 80, 70),
         6: (205, 180, 140), 7: (250, 250, 250), 8: (50, 110, 200), 9: (90, 170, 170), 10: (60, 140, 110), 11: (200, 220, 170),
         12: (80, 110, 40), 13: (130, 130, 130), 14: (185, 185, 185), 15: (120, 110, 80)}
CNAME = ['none', 'tree', 'shrub', 'grass', 'crop', 'built', 'bare', 'snow', 'water', 'wetland', 'mangrove', 'moss',
         'dwarf pine', 'rock', 'scree', 'bog']
WHY = {1: ((50, 110, 200), 'class (water)'), 2: ((200, 60, 50), 'too steep'), 3: ((20, 40, 120), 'barrier line'),
       4: ((120, 0, 0), 'motorway'), 5: ((0, 200, 230), 'hazard (flood...)'), 6: ((150, 60, 200), 'too deep (water/snow)'),
       7: ((255, 140, 0), 'obstruction'), 8: ((255, 200, 0), 'washout'), 9: ((255, 0, 200), 'team block')}
LAYC = [(200, 200, 200), (230, 170, 60), (110, 170, 230), (40, 90, 200), (120, 60, 180), (230, 90, 40), (200, 30, 30), (255, 220, 0)]
LNAME = ['base', 'late land cover (Nov 15)', 'Sentinel-2 refresh (Nov 20)', 'S1 flood (Nov 23)', 'S1 flood depth',
         'drone land cover', 'drone rubble']

def rgb_classes(top):
    im = np.zeros(top.shape + (3,), np.uint8)
    for c, col in CLASS.items(): im[top == c] = col
    return im

def rgb_time(t, why):
    im = np.zeros(t.shape + (3,), np.uint8)
    ok = t > 0
    lt = np.log(np.clip(t, 700, 20000))
    v = 245 - (lt - math.log(700)) / (math.log(20000) - math.log(700)) * 205
    for k in range(3): im[..., k] = np.where(ok, v, 0).astype(np.uint8)
    for r, (col, _) in WHY.items(): im[(~ok) & ((why >> 4) == r)] = col
    return im

def rgb_grey(x, inv=False):
    v = (255 - x) if inv else x
    return np.repeat(v.astype(np.uint8)[..., None], 3, 2)

def rgb_risk(r):
    im = np.full(r.shape + (3,), 255, np.uint8)
    f = r / 255.0
    im[..., 1] = (255 * (1 - f) ** 0.6).astype(np.uint8); im[..., 2] = (255 * (1 - f) ** 0.6).astype(np.uint8)
    return im

def rgb_src(s):
    im = np.zeros(s.shape + (3,), np.uint8)
    for k, col in enumerate(LAYC[:7]): im[s == k] = col
    im[s == 255] = (255, 0, 200)
    return im

flip = lambda a: a[::-1]

def legend(d, x, y, items, f=small):
    for col, name in items:
        d.rectangle([x, y + 3, x + 16, y + 17], fill=col, outline=(60, 60, 60)); d.text((x + 22, y), name, fill=(20, 20, 20), font=f); y += 21
    return y

# ---- overview: six planes of the team step ----
panels = [('Fused land cover (most probable class)', rgb_classes(team['top'])),
          ('Certainty of that class (darker = less sure)', rgb_grey(team['cert'])),
          ('Source: which layer last changed each cell', rgb_src(team['src'])),
          ('Walker: time per metre (darker = slower), closed cells by reason', rgb_time(team['wt'], team['wwhy'])),
          ('Walker: risk the cell is impassable', rgb_risk(team['wrisk'])),
          ('Light 4x4: time per metre, closed cells by reason', rgb_time(team['vt'], team['vwhy']))]
sc = 0.62
pw, ph = int(W * sc), int(H * sc)
pad, top_h, leg_w = 18, 70, 250
img = Image.new('RGB', (3 * (pw + pad) + pad + leg_w, 2 * (ph + 40 + pad) + top_h), 'white')
d = ImageDraw.Draw(img)
d.text((pad, 14), 'Incident surface, Tatra 20 x 20 km at 1", after the team step (7 layers + team overlay), 2026-11-24 10:15 UTC', fill='black', font=big)
for n, (title, arr) in enumerate(panels):
    x = pad + (n % 3) * (pw + pad); y = top_h + (n // 3) * (ph + 40 + pad)
    d.text((x, y), title, fill='black', font=small)
    img.paste(Image.fromarray(flip(arr)).resize((pw, ph), Image.NEAREST), (x, y + 22))
lx = 3 * (pw + pad) + pad
y = legend(d, lx, top_h, [(CLASS[c], CNAME[c]) for c in [1, 2, 3, 4, 5, 6, 8, 9, 12, 13, 14, 15]])
y = legend(d, lx, y + 20, [(LAYC[k], LNAME[k]) for k in range(7)] + [((255, 0, 200), 'team overlay item')])
legend(d, lx, y + 20, [v for v in WHY.values()])
img.save(f'{dst}/e14_planes.png')

# ---- close-up with routes ----
pf = profs[0]
n_, p_ = varint(pf, 4); p_ += n_
unit = struct.unpack_from('<Q', pf, p_)[0]; p_ += 8 + 4 * (16 + 8 + 16 + 5)
ng = struct.unpack_from('<H', pf, p_)[0]; grade = list(struct.unpack_from(f'<{ng}I', pf, p_ + 2))
dm = np.fromfile(f'{prep}/dem_dm.i32', np.int32).reshape(-1, 64, 64)
dmg = np.zeros((H, W), np.int64)
for k, (bi, bj) in enumerate(blist):
    dmg[(bi - bi0) * 64:(bi - bi0 + 1) * 64, (bj - bj0) * 64:(bj - bj0 + 1) * 64] = dm[k]
dml = dmg.tolist()
MOVES = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1), (1, 2), (2, 1), (2, -1), (1, -2), (-1, -2), (-2, -1), (-2, 1), (-1, 2)]
def crossed(dr, dc):
    if abs(dr) == 1 and abs(dc) == 2: return [(0, dc // 2), (dr, dc // 2)]
    if abs(dr) == 2 and abs(dc) == 1: return [(dr // 2, 0), (dr // 2, dc)]
    return []
rnd = lambda a, b: (2 * a + b) // (2 * b) if a >= 0 else -((-2 * a + b) // (2 * b))
st, TPD, R_MM = 120, 432000, 6371008800.0
dy = round(st / TPD * math.pi / 180 * R_MM); lat0 = bi0 * 64 * st; L = {}
for kk in range(-2, 2 * H + 2):
    dx = round(st / TPD * math.pi / 180 * R_MM * math.cos((lat0 + kk * st // 2) / TPD * math.pi / 180))
    L[kk] = [math.isqrt((a * dy) ** 2 + (b * dx) ** 2) for a, b in MOVES]

def route(t, dst_):
    tl = t.tolist()
    def cost(r, c, m):
        dr, dc = MOVES[m]; nr, nc = r + dr, c + dc
        if not (0 <= nr < H and 0 <= nc < W): return None
        ta, tz = tl[r][c], tl[nr][nc]
        if ta == 0 or tz == 0: return None
        s, n = ta + tz, 2
        for er, ec in crossed(dr, dc):
            tx = tl[r + er][c + ec]
            if tx == 0: return None
            s += tx; n += 1
        l = L[2 * r + dr][m]
        g = max(-100, min(100, rnd((dml[nr][nc] - dml[r][c]) * 10000, l)))
        return max(1, rnd(l * s * unit, n * grade[g + 100]))
    dist = {ipp: 0}; hq = [(0, ipp)]; done = set()
    while hq:
        dd, i = heapq.heappop(hq)
        if i in done: continue
        done.add(i)
        if i == dst_: break
        r, c = divmod(i, W)
        for m in range(16):
            x = cost(r, c, m)
            if x is None: continue
            j = (r + MOVES[m][0]) * W + c + MOVES[m][1]
            if dd + x < dist.get(j, 1 << 62): dist[j] = dd + x; heapq.heappush(hq, (dd + x, j))
    if dst_ not in done: return None, None
    cells = [dst_]; v = dst_
    while v != ipp:
        vr, vc = divmod(v, W); best = None
        for m, (dr, dc) in enumerate(MOVES):
            ur, uc = vr - dr, vc - dc
            u = ur * W + uc
            if not (0 <= ur < H and 0 <= uc < W) or u not in dist: continue
            x = cost(ur, uc, m)
            if x is not None and dist[u] + x == dist[v] and (best is None or u < best): best = u
        cells.append(best); v = best
    return dist[dst_], cells[::-1]

ir, ic = divmod(ipp, W)
R0, R1, C0, C1 = ir - 110, ir + 70, ic - 175, ic + 125
zs = 3
def crop(arr): return flip(arr[R0:R1, C0:C1])
def to_xy(cell):
    r, c = divmod(cell, W)
    return ((c - C0) * zs + zs // 2, (R1 - 1 - r) * zs + zs // 2)
sel = [0, 1, 3, 4]
routes = {}
for name, g in (('base', base), ('team', team)):
    routes[name] = [route(g['wt'], targets[k]) for k in sel]
zw, zh = (C1 - C0) * zs, (R1 - R0) * zs
img = Image.new('RGB', (2 * zw + 3 * pad + leg_w, zh + top_h + 60), 'white')
d = ImageDraw.Draw(img)
d.text((pad, 14), 'Around the incident point, 7 x 4.5 km: walker time per metre and routes, before and after the incident layers', fill='black', font=big)
RC = [(230, 30, 30), (30, 120, 230), (20, 160, 60), (240, 140, 0)]
for n, (name, g, title) in enumerate((('base', base, 'Base only (E5 surface), 09:00'), ('team', team, 'After S2 + flood + drone + late layer + team items, 10:15'))):
    x0 = pad + n * (zw + pad)
    d.text((x0, top_h - 26), title, fill='black', font=font)
    pic = Image.fromarray(crop(rgb_time(g['wt'], g['wwhy']))).resize((zw, zh), Image.NEAREST)
    pd = ImageDraw.Draw(pic)
    for k, (cst, cells) in enumerate(routes[name]):
        if cells is None: continue
        pts = [to_xy(cl) for cl in cells]
        pd.line(pts, fill=RC[k], width=4)
        tx, ty = to_xy(targets[sel[k]])
        pd.ellipse([tx - 7, ty - 7, tx + 7, ty + 7], fill=RC[k], outline='black')
        pd.text((tx + 9, ty - 9), f'{cst / 6e7:.0f} min', fill='black', font=small, stroke_width=2, stroke_fill='white')
    if name == 'team':
        info = json.load(open(f'{out}/../e14.json')) if os.path.exists(f'{out}/../e14.json') else None
        if info:
            cell_of = lambda ij: (ij[0] - bi0 * 64) * W + ij[1] - bj0 * 64
            for (ij, half, label, col) in ((info['team_block'], 1.5, 'team block', (255, 0, 200)),
                                           (info['team_clear'], 6, 'team: walked, dry', (255, 0, 200)),
                                           (info['drone_centre'], 12, 'drone survey', (230, 90, 40))):
                cx, cy = to_xy(cell_of(ij)); hp = half * zs
                if label == 'drone survey':
                    pd.ellipse([cx - hp, cy - hp, cx + hp, cy + hp], outline=col, width=3)
                else:
                    pd.rectangle([cx - hp, cy - hp, cx + hp, cy + hp], outline=col, width=3)
                pd.text((cx + hp + 4, cy - hp - 16), label, fill=col, font=small, stroke_width=2, stroke_fill='white')
    px, py = to_xy(ipp)
    pd.regular_polygon((px, py, 11), 5, fill=(255, 255, 0), outline='black')
    img.paste(pic, (x0, top_h + 4))
lx = 2 * (zw + pad) + pad
y = legend(d, lx, top_h, [((255, 255, 0), 'incident point'), ((245, 245, 245), 'fast (trail, road)'), ((90, 90, 90), 'slow (dwarf pine, bog)')])
legend(d, lx, y + 20, [v for v in WHY.values()])
d.text((pad, zh + top_h + 16), 'Route times from the stored surface; dots are the targets. North is up; 1 pixel block = one 1" cell (~20 x 31 m).',
       fill=(60, 60, 60), font=small)
img.save(f'{dst}/e14_closeup.png')
print(json.dumps({n: [None if c is None else round(c / 6e7, 1) for c, _ in v] for n, v in routes.items()}))

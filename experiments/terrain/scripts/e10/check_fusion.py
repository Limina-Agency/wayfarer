# E10: an independent implementation of terrain-layers.md (contract v0), written from the document,
# not from the Rust code. It reads the WTL0 layer files, recomputes every layer's content hash, fuses
# the sample blocks the Rust run dumped, and compares them byte for byte.
#   python3 -I check_fusion.py <base-dir> <e10-out-dir>      (needs numpy and blake3)
import sys, json, zlib, struct
import numpy as np
from blake3 import blake3

bdir, out = sys.argv[1:3]
S = 4096
BP = 4096

def varint(b, p):
    v = sh = 0
    while True:
        x = b[p]; p += 1
        v |= (x & 0x7F) << sh
        if x < 0x80:
            return v, p
        sh += 7

def unzz(u):
    return (u >> 1) ^ -(u & 1)

def manifest(b, p):
    st = p
    assert b[p:p + 8] == b'WTLAYER\0'; p += 8
    major, minor = struct.unpack_from('<HH', b, p); p += 4
    assert major == 0
    kind, lic = b[p], b[p + 1]; p += 2
    s_lat, s_lon = struct.unpack_from('<II', b, p); p += 8
    obs, vf, vu, ff = struct.unpack_from('<qqqq', b, p); p += 32
    n, p = varint(b, p)
    ch = list(struct.unpack_from(f'<{n}H', b, p)); p += 2 * n
    strs = []
    for _ in range(2):
        n, p = varint(b, p); strs.append(b[p:p + n].decode()); p += n
    p += 96 + 2  # weights, dataset manifest, eval set, ECE
    for _ in range(2):
        n, p = varint(b, p); strs.append(b[p:p + n].decode()); p += n
    for _ in range(2):
        n, p = varint(b, p); p += 32 * n
    return dict(kind=kind, licence=lic, s_lat=s_lat, s_lon=s_lon, obs=obs, vf=vf, vu=vu, ff=ff, ch=ch,
                model=strs[0], raw=b[st:p]), p

def plane(b, p, n, width):
    code = b[p]; p += 1
    if code == 0:
        raw = b[p:p + n * width]; p += n * width
    elif code == 1:
        raw = b[p:p + width] * n; p += width
    elif code == 3:
        raw = bytearray()
        while len(raw) < n * width:
            ln, p = varint(b, p); raw += b[p:p + width] * (ln + 1); p += width
        raw = bytes(raw)
    elif code == 4:
        ln, p = varint(b, p); raw = zlib.decompress(b[p:p + ln]); p += ln
    else:
        raise ValueError(code)
    assert len(raw) == n * width
    return np.frombuffer(raw, np.uint8 if width == 1 else '<u2').astype(np.int64), p

def read_layer(path):
    b = open(path, 'rb').read()
    assert b[:4] == b'WTL0'
    m, p = manifest(b, 4)
    nb, = struct.unpack_from('<I', b, p); p += 4
    keys = struct.unpack_from(f'<{nb}Q', b, p); p += 8 * nb
    offs = struct.unpack_from(f'<{nb + 1}Q', b, p); p += 8 * (nb + 1)
    base = p
    slots, width = {1: (16, 1), 2: (len(m['ch']), 1), 3: (len(m['ch']), 2), 4: (0, 0)}[m['kind']]
    h = blake3(m['raw']); h.update(struct.pack('<Q', nb))
    blocks = {}
    for k in range(nb):
        q = base + offs[k]
        tag = b[q]; q += 1
        if tag & 0x80:
            bm = np.frombuffer(b[q:q + 512], '<u8'); q += 512
        else:
            bm = np.full(64, 2**64 - 1, np.uint64)
        pres = np.array([(int(bm[i >> 6]) >> (i & 63)) & 1 for i in range(BP)], bool)
        idx = np.flatnonzero(pres)
        conf, q = plane(b, q, len(idx), 1)
        vals = []
        for _ in range(slots):
            v, q = plane(b, q, len(idx), width); vals.append(v)
        assert q == base + offs[k + 1]
        vals = np.array(vals).reshape(slots, len(idx))
        if m['kind'] == 1:
            assert (vals.sum(0) == 255).all()
        hb = bytearray(struct.pack('<Q', keys[k])); hb += bm.astype('<u8').tobytes()
        rec = np.zeros((len(idx), 1 + slots * width), np.uint8)
        rec[:, 0] = conf
        for s in range(slots):
            if width == 1:
                rec[:, 1 + s] = vals[s]
            else:
                rec[:, 1 + 2 * s] = vals[s] & 255; rec[:, 2 + 2 * s] = vals[s] >> 8
        hb += rec.tobytes(); h.update(bytes(hb))
        blocks[keys[k]] = (pres, idx, conf, vals)
    p = base + offs[nb]
    nf, p = varint(b, p)
    feats, fb = [], bytearray()
    for _ in range(nf):
        st = p
        cls, cf = b[p], b[p + 1]; p += 2
        n, p = varint(b, p)
        la = lo = 0; pts = []
        for _ in range(n):
            u, p = varint(b, p); la += unzz(u)
            u, p = varint(b, p); lo += unzz(u)
            pts.append((la, lo))
        feats.append((cls, cf, pts)); fb += b[st:p]
    h.update(struct.pack('<Q', nf)); h.update(bytes(fb))
    assert p + 32 == len(b)
    assert h.digest() == b[p:], f'{path}: content hash'
    m['hash'] = b[p:]
    return m, blocks, feats

def largest_remainder(raw, total):
    s = sum(raw)
    out = [r * total // s for r in raw]
    rem = sorted(((r * total % s, -k) for k, r in enumerate(raw)), reverse=True)
    left = total - sum(out)
    for r, k in rem[:left]:
        assert r > 0
        out[-k] += 1
    return out

def w_eff(w, m, t):
    if t < m['ff']:
        return w
    return w * (m['vu'] - t) // (m['vu'] - m['ff'])

def supercover(a, b, s, visit):
    c, half = s * 625, (s // 2) * 625
    ay, ax = a[0] * 27 + half, a[1] * 27 + half
    by, bx = b[0] * 27 + half, b[1] * 27 + half
    cy, cx, ey, ex = ay // c, ax // c, by // c, bx // c
    dy, dx = by - ay, bx - ax
    sy, sx = (dy > 0) - (dy < 0), (dx > 0) - (dx < 0)
    visit(cy, cx)
    while (cy, cx) != (ey, ex):
        nx = (cx + 1) * c - ax if sx > 0 else ax - cx * c
        ny = (cy + 1) * c - ay if sy > 0 else ay - cy * c
        tx = nx * abs(dy) if dx else None
        ty = ny * abs(dx) if dy else None
        if tx is not None and ty is not None and tx == ty:
            visit(cy, cx + sx); visit(cy + sy, cx); cx += sx; cy += sy
        elif tx is not None and (ty is None or tx < ty):
            cx += sx
        else:
            cy += sy
        visit(cy, cx)

def key(bi, bj):
    return ((bi + 2**31) << 32) | (bj + 2**31)

inp = json.load(open(f'{out}/fusion_inputs.json'))
T, SP = inp['t'], inp['spacing']
layers = [read_layer(f'{out}/{f}') for f in inp['layers']]
active = []
for m, blocks, feats in layers:
    if not (m['vf'] <= T < m['vu']) or (m['licence'] == 1 and not inp['allow_research']) or m['s_lat'] != SP:
        continue
    if m['kind'] == 4:
        ok = any(cf >= inp['feature_min_conf'] for _, cf, _ in feats)
    else:
        thr = inp['level_min_w'] if m['kind'] == 3 else 1
        ok = any(w_eff(int(w), m, T) >= thr for (_, _, conf, _) in blocks.values() for w in conf)
    if ok:
        active.append((m['obs'], m['hash'], m, blocks, feats))
active.sort(key=lambda a: (a[0], a[1]))
print('applied:', [(a[2]['model'], a[0]) for a in active])

# effect bits from features (feature classes 1-16 -> bit 0, 17 debris 1, 18 fallen 2, 19 washout 3, 20 passage 4)
BIT = {17: 2, 18: 4, 19: 8, 20: 16}
fbits = {}
for _, _, m, _, feats in active:
    if m['kind'] != 4:
        continue
    for cls, cf, pts in feats:
        bit = 1 if 1 <= cls <= 16 else BIT.get(cls, 0)
        if cf < inp['feature_min_conf'] or not bit:
            continue
        for a, b in zip(pts, pts[1:]):
            def v(i, j):
                k = key(i // 64, j // 64); q = (i % 64) * 64 + j % 64
                fbits.setdefault(k, {}); fbits[k][q] = fbits[k].get(q, 0) | bit
            supercover(a, b, SP, v)

blist = [tuple(map(int, l.split())) for l in open(f'{bdir}/blocks.txt') if l.strip()]
bcls = np.fromfile(f'{bdir}/base_class.u8', np.uint8).reshape(-1, BP)
HAZ = [1, 2, 3, 4, 5, 6, 7, 8]; LEV = [1, 2, 3]
want = open(f'{out}/fused_sample.bin', 'rb').read()
REC = 8 + BP * (32 + 8 + 9 + 2 + 1 + 1)
assert len(want) % REC == 0
grid_layers = [(n + 1, a) for n, a in enumerate(active) if a[2]['kind'] != 4]
bad = 0
for r in range(len(want) // REC):
    k, = struct.unpack_from('<Q', want, r * REC)
    bi, bj = (k >> 32) - 2**31, (k & 0xFFFFFFFF) - 2**31
    kb = blist.index((bi, bj))
    buf = bytearray(struct.pack('<Q', k))
    lmaps = []
    for n, (_, _, m, blocks, _) in grid_layers:
        if k in blocks:
            pres, idx, conf, vals = blocks[k]
            pos = {int(q): x for x, q in enumerate(idx)}
            lmaps.append((n, m, pos, conf, vals))
    for q in range(BP):
        P = [0] * 16; P[int(bcls[kb, q])] = S        # one-hot prior
        haz = [0] * 8; lev = [(0, 0)] * 3; src = 0
        mask = fbits.get(k, {}).get(q, 0)
        for n, m, pos, conf, vals in lmaps:
            if q not in pos:
                continue
            x = pos[q]; w = w_eff(int(conf[x]), m, T)
            if w == 0:
                continue
            if m['kind'] == 1:
                qv = [int(vals[c][x]) for c in range(16)]
                raw = [(255 - w) * 255 * P[c] + w * ((qv[c] if c else 0) * S + qv[0] * P[c]) for c in range(16)]
                assert sum(raw) == 255 * 255 * S
                P = largest_remainder(raw, S)
            elif m['kind'] == 2:
                for ci, code in enumerate(m['ch']):
                    haz[HAZ.index(code)] = ((255 - w) * haz[HAZ.index(code)] + w * int(vals[ci][x]) + 127) // 255
            else:
                if w < inp['level_min_w']:
                    continue
                for ci, code in enumerate(m['ch']):
                    lev[LEV.index(code)] = (int(vals[ci][x]), w)
            src = min(n, 254)
        i, j = bi * 64 + q // 64, bj * 64 + q % 64
        for la0, la1, lo0, lo1, eff in inp['overlay']:
            if not (la0 <= i * SP < la1 and lo0 <= j * SP < lo1):
                continue
            if eff[0] == 'block':
                mask |= 32
            elif eff[0] == 'clear':
                haz = [0] * 8; lev = [(0, 0)] * 3; mask &= ~(2 | 4 | 8 | 32)
            else:
                haz[HAZ.index(eff[1])] = eff[2]
            src = 255
        cert = (max(P) * 255 + S // 2) // S
        buf += struct.pack('<16H', *P) + bytes(haz)
        for v, c in lev:
            buf += struct.pack('<HB', v, c)
        buf += struct.pack('<HBB', mask, cert, src)
    got = bytes(buf)
    exp = want[r * REC:(r + 1) * REC]
    if got != exp:
        bad += 1
        print('block', (bi, bj), 'differs')
print(json.dumps({'layers_hash_checked': len(layers), 'applied': len(active), 'sample_blocks': len(want) // REC,
                  'posts': len(want) // REC * BP, 'blocks_differing': bad}))
sys.exit(1 if bad else 0)

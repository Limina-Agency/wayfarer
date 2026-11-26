# E5 picture: walker friction (grey, darker = slower; blue = impassable) with the walker's
# travel-time bands from the incident point (30 min steps), and the 4x4's reachable area.
#   python3 render_e5.py <prep-dir> <surface-out-dir> <out.png>
import sys, numpy as np
from PIL import Image
P, O, out = sys.argv[1], sys.argv[2], sys.argv[3]
blocks = sorted(tuple(map(int, l.split())) for l in open(f'{P}/blocks.txt') if l.strip())
bi0 = min(b[0] for b in blocks); bj0 = min(b[1] for b in blocks)
H = (max(b[0] for b in blocks) - bi0 + 1) * 64; W = (max(b[1] for b in blocks) - bj0 + 1) * 64
def read_wte(path):
    b = open(path, 'rb').read(); n = int.from_bytes(b[10:14], 'little'); base = 14 + 16*n
    offs = [int.from_bytes(b[14+16*k+8:14+16*k+16], 'little') for k in range(n)] + [len(b) - base]
    g = np.zeros((H, W), np.uint16)
    def varints(buf):
        pos = 0
        while pos < len(buf):
            v = s = 0
            while True:
                x = buf[pos]; pos += 1; v |= (x & 127) << s; s += 7
                if x < 128: break
            yield v
    for k, (bi, bj) in enumerate(blocks):
        it = varints(b[base+offs[k]:base+offs[k+1]])
        npal = next(it); pal = [next(it) for _ in range(npal)]
        kb = max(1, (max(npal, 2) - 1).bit_length()); vals = []
        for x in it:
            vals += [pal[x & ((1 << kb) - 1)]] * ((x >> kb) + 1)
        g[(bi-bi0)*64:(bi-bi0+1)*64, (bj-bj0)*64:(bj-bj0+1)*64] = np.array(vals, np.uint16).reshape(64, 64)
    return g
def panel(name):
    t = read_wte(f'{O}/surface_{name}.wte')
    f = np.fromfile(f'{O}/field_{name}.u32', np.uint32).reshape(H, W)
    grey = np.where(t > 0, 255 - np.clip((np.log(np.maximum(t, 1)) - np.log(t[t > 0].min())) / np.log(25), 0, 1) * 200, 0)
    img = np.stack([grey, grey, grey], -1)
    img[t == 0] = [70, 110, 200]
    reach = f != 0xFFFFFFFF
    step = 18000 if name == 'walker' else 3000                       # 30 min (walker), 5 min (4x4) bands
    band = (f // step) % 2 == 1
    img[reach & band] = img[reach & band] * 0.6 + np.array([255, 140, 0]) * 0.4
    img[reach & ~band] = img[reach & ~band] * 0.85 + np.array([255, 200, 120]) * 0.15
    ipp = np.unravel_index(np.argmin(np.where(reach, f, 0xFFFFFFFF)), f.shape)
    img[max(0, ipp[0]-5):ipp[0]+6, max(0, ipp[1]-5):ipp[1]+6] = [220, 0, 0]
    return img[::-1]                                                  # rows are stored from the south
gap = np.full((H, 12, 3), 255.0)
Image.fromarray(np.concatenate([panel('walker'), gap, panel('vehicle')], 1).astype(np.uint8)).save(out)
print(out, H, W)

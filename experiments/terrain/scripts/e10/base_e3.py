# E10: the stage-1 base land cover by E3's recommended rule (WorldCover first, OSM override where it covers
# at least 8 of 16 samples, no override for forest or for scrub below 1,300 m, which is dwarf pine above),
# with E3's confidence bit. Writes base_class.u8 and base_conf.u8 (one byte per post, block-list order).
#   python3 -I base_e3.py <wc-dir> <osm-dir> <dem-dir> <out-dir>
import sys, os, numpy as np
wcd, osd, dmd, out = sys.argv[1:5]
DP_MIN_DM = 13000
O2W = np.zeros(32, np.uint8)
for k, v in {1:1, 2:15, 4:2, 5:2, 6:3, 7:3, 8:3, 9:4, 10:4, 11:4, 12:5, 13:5, 14:5, 15:5, 16:5, 17:13, 18:14, 19:6, 20:6,
             21:7, 22:8, 23:9, 24:9, 25:15, 26:9, 27:3, 28:5, 29:3, 30:3, 31:5}.items(): O2W[k] = v
W2C = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 1, 3, 2, 6, 6, 9])
wc = np.fromfile(f'{wcd}/posts_class.u8', np.uint8)
pur = np.fromfile(f'{wcd}/posts_purity.u8', np.uint8).reshape(-1, 2)
of = np.fromfile(f'{osd}/posts_fine.u8', np.uint8)
oc = np.fromfile(f'{osd}/posts_cov.u8', np.uint8)
z = np.fromfile(f'{dmd}/posts_dm.i32', np.int32)
ow = O2W[of].copy()
ow[(of == 3) & (z < DP_MIN_DM)] = 0          # lowland scrub: left to WorldCover
ow[(of == 3) & (z >= DP_MIN_DM)] = 12         # dwarf pine
ow[ow == 1] = 0                               # forest: no override (agrees 98 %)
present = (oc >= 8) & (ow > 0)
fused = np.where(present, ow, wc).astype(np.uint8)
agree = present & (W2C[ow] == W2C[wc])
pure = pur[:, 0].astype(int) * 4 >= pur[:, 1].astype(int) * 3
conf = (agree | (~present & pure)).astype(np.uint8)
conf[fused == 0] = 0
os.makedirs(out, exist_ok=True)
fused.tofile(f'{out}/base_class.u8'); conf.tofile(f'{out}/base_conf.u8')
print({'posts': len(fused), 'hist': np.bincount(fused, minlength=16).tolist(), 'conf_share': round(float(conf.mean()), 4)})

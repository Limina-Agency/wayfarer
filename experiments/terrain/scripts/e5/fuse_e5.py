# E5 stage-1 prep: the E3 recommended land-cover fusion (WorldCover base, OSM override where an
# OSM area covers >= 8/16 samples, except forest and scrub below 1,300 m GLO-30; dwarf pine from
# OSM scrub at or above 1,300 m), plus E3's confidence bit, and the prep dir for terrexp surface.
#   python3 fuse_e5.py <work-dir> <prep-dir>
# work-dir holds glo30/, wc/, osm/, lines/, gem15/ (terrexp and gem15_lattice.py outputs).
import sys, os, json, shutil, numpy as np
W, P = sys.argv[1], sys.argv[2]
os.makedirs(P, exist_ok=True)
DP_MIN_DM = 13000
O2W = np.zeros(32, np.uint8)
for k, v in {1:1, 2:15, 4:2, 5:2, 6:3, 7:3, 8:3, 9:4, 10:4, 11:4, 12:5, 13:5, 14:5, 15:5, 16:5, 17:13, 18:14, 19:6, 20:6,
             21:7, 22:8, 23:9, 24:9, 25:15, 26:9, 27:3, 28:5, 29:3, 30:3, 31:5}.items(): O2W[k] = v
W2C = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 1, 3, 2, 6, 6, 9])
for d in ('wc', 'osm'):  # terrexp lines and gem15_lattice.py keep the sorted block order (checked there)
    assert open(f'{W}/{d}/blocks.txt').read() == open(f'{W}/glo30/blocks.txt').read()
wc = np.fromfile(f'{W}/wc/posts_class.u8', np.uint8)
pur = np.fromfile(f'{W}/wc/posts_purity.u8', np.uint8).reshape(-1, 2)
of = np.fromfile(f'{W}/osm/posts_fine.u8', np.uint8)
oc = np.fromfile(f'{W}/osm/posts_cov.u8', np.uint8)
z = np.fromfile(f'{W}/glo30/posts_dm.i32', np.int32)
ow = O2W[of].copy()
ow[(of == 3) & (z >= DP_MIN_DM)] = 12
ow[(of == 1) | ((of == 3) & (z < DP_MIN_DM))] = 0      # forest and lowland scrub: no override
present = (oc >= 8) & (ow > 0)
fused = np.where(present, ow, wc).astype(np.uint8)
agree = present & (W2C[ow] == W2C[wc])
pure = pur[:, 0].astype(int) * 4 >= pur[:, 1].astype(int) * 3
conf = (agree | (~present & pure)).astype(np.uint8)
fused.tofile(f'{P}/lc4.u8'); conf.tofile(f'{P}/conf.u8')
shutil.copy(f'{W}/glo30/blocks.txt', f'{P}/blocks.txt')
shutil.copy(f'{W}/gem15/posts_dm.i32', f'{P}/dem_dm.i32')
shutil.copy(f'{W}/gem15/posts_steep.u8', f'{P}/steep.u8')
shutil.copy(f'{W}/lines/posts_lines.u16', f'{P}/lines.u16')
res = {'posts': int(fused.size), 'osm_override_share': round(float(present.mean()), 4), 'conf_share': round(float(conf.mean()), 4),
       'code_hist': np.bincount(fused, minlength=16).tolist()}
json.dump(res, open(f'{P}/fuse.json', 'w'), indent=1)
print(json.dumps(res))

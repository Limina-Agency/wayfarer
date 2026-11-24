# Fused land-cover layer (WorldCover first, OSM override, dwarf pine from OSM scrub >= 1300 m)
# in the proposed Wayfarer 4-bit table, plus one confidence bit; sizes of run-length encodings.
import sys, json, numpy as np
out, area, st = sys.argv[1], sys.argv[2], int(sys.argv[3])
DP_MIN_DM = 13000
# OSM fine -> Wayfarer 4-bit code (0 = no override)
#  1 tree 2 shrub 3 grass 4 crop 5 built 6 bare 7 snow 8 water 9 wetland 10 mangrove 11 moss (WorldCover)
# 12 dwarf pine 13 rock 14 scree 15 bog / swamp forest (OSM only)
O2W = np.zeros(32, np.uint8)
for k, v in {1:1, 2:15, 4:2, 5:2, 6:3, 7:3, 8:3, 9:4, 10:4, 11:4, 12:5, 13:5, 14:5, 15:5, 16:5, 17:13, 18:14, 19:6, 20:6,
             21:7, 22:8, 23:9, 24:9, 25:15, 26:9, 27:3, 28:5, 29:3, 30:3, 31:5}.items(): O2W[k] = v
# common class of a Wayfarer code (for the agreement bit): WorldCover's own classes; 12 -> shrub, 13/14 -> bare, 15 -> wetland
W2C = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 1, 3, 2, 6, 6, 9])
wc = np.fromfile(f'{out}/wc_{area}_{st}/posts_class.u8', np.uint8)
pur = np.fromfile(f'{out}/wc_{area}_{st}/posts_purity.u8', np.uint8).reshape(-1, 2)
of = np.fromfile(f'{out}/osm_{area}_{st}/posts_fine.u8', np.uint8)
oc = np.fromfile(f'{out}/osm_{area}_{st}/posts_cov.u8', np.uint8)
z = np.fromfile(f'{out}/dem_{area}_{st}/posts_dm.i32', np.int32)
ow = O2W[of].copy()
ow[(of == 3) & (z >= DP_MIN_DM)] = 12  # scrub below 1300 m: no override
present = (oc >= 8) & (ow > 0)
fused = np.where(present, ow, wc).astype(np.uint8)
agree = present & (W2C[ow] == W2C[wc])
pure = pur[:, 0].astype(int) * 4 >= pur[:, 1].astype(int) * 3
conf = (agree | (~present & pure)).astype(np.uint8)
def varint_len(x):
    n = np.ones_like(x)
    for k in range(1, 10): n += (x >> (7 * k)) > 0
    return n
def runs_bytes(v, bits):
    # per block, runs over the 4,096 posts row-major: varint((len - 1) << bits | value)
    v = v.reshape(-1, 4096).astype(np.int64)
    total = 0
    for b in v:
        ch = np.flatnonzero(np.diff(b)) + 1
        starts = np.r_[0, ch]; lens = np.diff(np.r_[starts, 4096])
        total += int(varint_len(((lens - 1) << bits) | b[starts]).sum())
    return total
nb = len(wc) // 4096
idx = 16 * nb
res = {'area': area, 'spacing_ticks': st, 'blocks': nb,
       'wc_runs4': runs_bytes(wc, 4) + idx, 'fused_runs4': runs_bytes(fused, 4) + idx,
       'fused_conf_runs5': runs_bytes(fused * 2 + conf, 5) + idx,
       'fused_runs4_plus_conf_runs1': runs_bytes(fused, 4) + runs_bytes(conf, 1) + 2 * idx,
       'conf_set_share': round(float(conf.mean()), 4), 'osm_override_share': round(float(present.mean()), 4),
       'code_hist': np.bincount(fused, minlength=16).tolist()}
json.dump(res, open(f'{out}/../json/size_{area}_{st}.json', 'w'), indent=1)
print(res)

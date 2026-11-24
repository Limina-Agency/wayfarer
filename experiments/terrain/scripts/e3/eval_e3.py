# E3 evaluation: WorldCover, OSM and fused land cover against BDOT10k on the lattice.
import sys, json, numpy as np
out, area, st = sys.argv[1], sys.argv[2], int(sys.argv[3])
C = ['tree','shrub','dwarf_pine','grass','crop','built','bare','water','wetland','snow']
T,S,DP,G,CR,B,R,W,M,SN = range(10)
U = 10  # unknown
NC = 10
DP_MIN_M = int(sys.argv[4]) if len(sys.argv) > 4 else 1300  # OSM scrub at or above this height (GLO-30) = dwarf pine
# WorldCover 4-bit (E1) -> common
WC = np.array([U, T, S, G, CR, B, R, SN, W, M, T, G] + [U]*4)
# OSM fine (osm.rs FINE_NAMES) -> common
OSM = np.array([U, T, M, S, S, S, G, G, G, CR, CR, CR, B, B, B, B, B, R, R, R, R, SN, W, M, M, M, M, G, B, G, G, B])
# BDOT fine (bdot_lattice.py NAMES) -> common
BD = np.array([U, T, T, T, S, DP, G, CR, CR, CR, B, B, B, B, R, R, R, R, R, G, W, W, M, M, M])
# walker speed factor per common class (feat-x.md X3 starting values; water blocked = 0; unknown 0.7)
# dwarf pine 0.15 is this experiment's assumption (feat-x.md has no class for it)
F = np.array([0.6, 0.35, 0.15, 0.8, 0.7, 0.5, 0.4, 0.0, 0.25, 0.4, 0.7])
def load(d, name, shape=None):
    return np.fromfile(f'{out}/{d}_{area}_{st}/{name}', np.uint8)
bw = open(f'{out}/wc_{area}_{st}/blocks.txt').read(); assert bw == open(f'{out}/osm_{area}_{st}/blocks.txt').read() == open(f'{out}/bdot_{area}_{st}/blocks.txt').read()
wc4 = load('wc', 'posts_class.u8'); pur = load('wc', 'posts_purity.u8').reshape(-1, 2)
of = load('osm', 'posts_fine.u8'); oc = load('osm', 'posts_cov.u8')
bf = load('bdot', 'posts_fine.u8'); bc = load('bdot', 'posts_cov.u8')
z = np.fromfile(f'{out}/dem_{area}_{st}/posts_dm.i32', np.int32)
mask = (bc >= 8) & (wc4 > 0)
wc4, pur, of, oc, bf, z = wc4[mask], pur[mask], of[mask], oc[mask], bf[mask], z[mask]
truth = BD[bf]; wc = WC[wc4]; osm_raw = OSM[of]
osm_raw = np.where((of == 3) & (z >= DP_MIN_M * 10), DP, osm_raw)
n = len(truth)
purity = pur[:, 0] / np.maximum(pur[:, 1], 1)
res = {'area': area, 'spacing_ticks': st, 'posts': int(n)}
def conf(pred):
    m = np.zeros((NC + 1, NC + 1), np.int64)
    np.add.at(m, (truth, pred), 1)
    return m
def summary(pred, name):
    m = conf(pred)
    acc = np.trace(m[:NC, :NC]) / n
    known = pred != U
    acc_known = (pred[known] == truth[known]).mean() if known.any() else 0
    # pace error (walker), water treated separately
    ft, fp = F[truth], F[pred]
    both = (ft > 0) & (fp > 0)
    lr = np.abs(np.log(fp[both] / ft[both]))
    d = {'acc': round(float(acc), 4), 'coverage': round(float(known.mean()), 4), 'acc_known': round(float(acc_known), 4),
         'pace_err_mean': round(float(lr.mean()), 4), 'pace_err_gt1.5x': round(float((lr > np.log(1.5)).mean()), 4),
         'water_missed': int(((truth == W) & (pred != W)).sum()), 'water_false': int(((truth != W) & (pred == W)).sum()),
         'water_truth': int((truth == W).sum()),
         'recall': {C[k]: round(float(m[k, k] / max(m[k].sum(), 1)), 3) for k in range(NC) if m[k].sum()},
         'precision': {C[k]: round(float(m[k, k] / max(m[:, k].sum(), 1)), 3) for k in range(NC) if m[:, k].sum()},
         'confusion': m.tolist()}
    res[name] = d
    return d
osm_present = oc >= 8
osm = np.where(osm_present, osm_raw, U)
summary(wc, 'worldcover')
summary(osm, 'osm')
summary(np.where(osm_present, osm_raw, wc), 'osm_first_wc_fill')
# WorldCover first, OSM overrides only for listed common classes
def wc_override(classes, cov_min=8):
    o = (oc >= cov_min) & np.isin(osm_raw, classes)
    return np.where(o, osm_raw, wc)
VARIANTS = [('wc+osm[water,bare]', [W, R]), ('wc+osm[water,bare,wetland]', [W, R, M]),
            ('wc+osm[water,bare,wetland,dwarf_pine]', [W, R, M, DP]),
            ('wc+osm[water,bare,wetland,dwarf_pine,built]', [W, R, M, DP, B]),
            ('wc+osm[water,bare,wetland,dwarf_pine,built,grass,crop]', [W, R, M, DP, B, G, CR]),
            ('wc+osm[all but shrub]', [T, DP, G, CR, B, R, W, M, SN]),
            ('wc+osm[all]', [T, S, DP, G, CR, B, R, W, M, SN])]
for nm, cl in VARIANTS:
    summary(wc_override(cl), nm)
# OSM-unknown posts: what WorldCover gets right there
unk = ~osm_present
res['osm_unknown_share'] = round(float(unk.mean()), 4)
res['wc_acc_on_osm_unknown'] = round(float((wc[unk] == truth[unk]).mean()), 4) if unk.any() else None
res['osm_unknown_truth_mix'] = {C[k]: round(float((truth[unk] == k).mean()), 3) for k in range(NC) if (truth[unk] == k).any()}
# per OSM class: who is right where OSM says c (vs WorldCover)
res['where_osm_says'] = {}
for k in range(NC):
    sel = osm_present & (osm_raw == k)
    if sel.sum() < 50: continue
    res['where_osm_says'][C[k]] = {'posts': int(sel.sum()), 'osm_right': round(float((truth[sel] == k).mean()), 3),
                                   'wc_right': round(float((wc[sel] == truth[sel]).mean()), 3),
                                   'wc_agrees': round(float((wc[sel] == k).mean()), 3)}
# per OSM fine class (OSM-only candidates)
FN = ["none","forest","swamp_forest","scrub","heath","clearcut","grass","meadow","fell","farmland","orchard","allotments",
      "residential","industrial","building","paved","cemetery","bare_rock","scree","sand","quarry","glacier","water","marsh",
      "reedbed","bog","wet_meadow","garden","farmyard","park","pitch","construction"]
BN = ['none','las','zagajnik','zadrzewienie','krzewy','kosodrzewina','trawiasta','uprawa_orna','sad_plantacja','ogrodki_dzialkowe',
      'zabudowa','komunikacja','plac','przemyslowe','skladowisko','wyrobisko','piasek_zwir','piarg','kamienisty','nieuzytki',
      'woda_plynaca','woda_stojaca','teren_podmokly','bagno','las_podmokly']
res['osm_fine'] = {}
for k in range(1, 32):
    sel = osm_present & (of == k)
    if sel.sum() < 50: continue
    tb = np.bincount(bf[sel], minlength=25); top = np.argsort(-tb)[:3]
    tw = np.bincount(wc4[sel], minlength=12); topw = np.argsort(-tw)[:3]
    res['osm_fine'][FN[k]] = {'posts': int(sel.sum()), 'bdot_top': {BN[t]: round(float(tb[t] / sel.sum()), 3) for t in top if tb[t]},
                              'wc_top': {['none','tree','shrub','grass','crop','built','bare','snow','water','wetland','mangrove','moss'][t]: round(float(tw[t] / sel.sum()), 3) for t in topw if tw[t]}}
res['bdot_fine'] = {}
for k in range(1, 25):
    sel = bf == k
    if sel.sum() < 50: continue
    tw = np.bincount(wc4[sel], minlength=12); topw = np.argsort(-tw)[:3]
    to = np.bincount(of[sel] * osm_present[sel], minlength=32); topo = np.argsort(-to)[:3]
    res['bdot_fine'][BN[k]] = {'posts': int(sel.sum()),
        'wc_top': {['none','tree','shrub','grass','crop','built','bare','snow','water','wetland','mangrove','moss'][t]: round(float(tw[t] / sel.sum()), 3) for t in topw if tw[t]},
        'osm_top': {FN[t]: round(float(to[t] / sel.sum()), 3) for t in topo if to[t]}}
# confidence-bit candidates on the recommended fused layer
REC = [W, R, M, DP, B, G, CR]
fused = wc_override(REC)
agree = osm_present & (osm_raw == wc)
cands = {
  'wc_pure75': purity >= 0.75,
  'wc_pure100': purity >= 1.0,
  'osm_wc_agree': agree,
  'agree_or_osm_specific': agree | (osm_present & np.isin(osm_raw, [W, M, R, DP])),
  'agree_or_(no_osm_and_pure75)': agree | (~osm_present & (purity >= 0.75)),
  'agree_or_specific_or_(no_osm_and_pure75)': agree | (osm_present & np.isin(osm_raw, [W, M, R, DP])) | (~osm_present & (purity >= 0.75)),
}
res['confidence'] = {}
ok = fused == truth
for nm, bit in cands.items():
    res['confidence'][nm] = {'set_share': round(float(bit.mean()), 4), 'acc_set': round(float(ok[bit].mean()), 4) if bit.any() else None,
                             'acc_unset': round(float(ok[~bit].mean()), 4) if (~bit).any() else None}
json.dump(res, open(f'{out}/../json/eval_{area}_{st}.json', 'w'), indent=1)
# grass and crop merged ("open"), the confusion that matters least for friction
res['open_merged_acc'] = {k: round(float(((np.where(np.isin(p, [G, CR]), G, p)) == np.where(np.isin(truth, [G, CR]), G, truth)).mean()), 4)
                          for k, p in [('worldcover', wc), ('osm_first_wc_fill', np.where(osm_present, osm_raw, wc)), ('recommended', fused)]}
json.dump(res, open(f'{out}/../json/eval_{area}_{st}.json', 'w'), indent=1)
print(area, st, n, {k: (res[k]['acc'], res[k]['coverage'], res[k]['pace_err_mean']) for k in res if isinstance(res[k], dict) and 'acc' in res[k]})

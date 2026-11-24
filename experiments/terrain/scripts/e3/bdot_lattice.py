# BDOT10k land cover (PT* layers, a wall-to-wall partition, plus the OIMK wetland overlay)
# onto the lattice: same 4x4 sample centres as `terrexp osm`, majority over covered samples.
# Reference only (floating point, GDAL rasterize at pixel centres).
import sys, glob, warnings, numpy as np, pyogrio, geopandas as gpd, rasterio.features as rf
from affine import Affine
warnings.filterwarnings('ignore')
extdir, bfile, st, outdir = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
import os; os.makedirs(outdir, exist_ok=True)
TPD = 432000
NAMES = ['none','las','zagajnik','zadrzewienie','krzewy','kosodrzewina','trawiasta','uprawa_orna','sad_plantacja',
         'ogrodki_dzialkowe','zabudowa','komunikacja','plac','przemyslowe','skladowisko','wyrobisko','piasek_zwir',
         'piarg','kamienisty','nieuzytki','woda_plynaca','woda_stojaca','teren_podmokly','bagno','las_podmokly']
PRI = [0,12,11,10,13,14,4,2,3,3,6,7,7,7,8,23,18,24,25,9,31,31,22,22,20]
RODZ = {
 'PTLZ': {'las':1,'zagajnik':2,'zadrzewienie':3}, 'PTRK': {'krzewy':4,'kosodrzewina':5},
 'PTTR': {'roślinność trawiasta':6,'uprawa na gruntach ornych':7},
 'PTUT': {'sad':8,'plantacja':8,'szkółka roślin':8,'ogródki działkowe':9},
 'PTGN': {'teren piaszczysty lub żwirowy':16,'piarg, usypisko lub rumowisko skalne':17,'teren kamienisty':18,'pozostałe grunty nieużytkowane':19},
 'PTWP': {'woda płynąca':20,'woda stojąca':21},
}
FIXED = {'PTZB':10,'PTKM':11,'PTPL':12,'PTNZ':13,'PTSO':14,'PTWZ':15}
blocks = [tuple(map(int, l.split())) for l in open(bfile)]
bi0 = min(b[0] for b in blocks); bi1 = max(b[0] for b in blocks)
bj0 = min(b[1] for b in blocks); bj1 = max(b[1] for b in blocks)
sub = st // 4
nrows = (bi1 - bi0 + 1) * 64 * 4; ncols = (bj1 - bj0 + 1) * 64 * 4
west = (bj0*64*st - st/2) / TPD; north = ((bi1+1)*64*st - st/2) / TPD; d = sub / TPD
tr = Affine(d, 0, west, 0, -d, north)
bbox4326 = (west, north - nrows*d, west + ncols*d, north)
def load(layer):
    out = []
    for f in sorted(glob.glob(f'{extdir}/*OT_{layer}_A.gpkg')):
        fields = pyogrio.read_info(f)['fields']
        df = pyogrio.read_dataframe(f, columns=['RODZAJ'] if 'RODZAJ' in fields else [])
        if len(df) == 0: continue
        df = df.to_crs(4326)
        df = df.cx[bbox4326[0]:bbox4326[2], bbox4326[1]:bbox4326[3]]
        if len(df): out.append(df)
    return out
grid = np.zeros((nrows, ncols), np.uint8)
for lay in list(RODZ) + list(FIXED):
    for df in load(lay):
        vals = df.RODZAJ.map(RODZ[lay]).fillna(0).astype(int) if lay in RODZ else np.full(len(df), FIXED[lay])
        shapes = [(g, int(v)) for g, v in zip(df.geometry, vals) if v]
        if shapes: rf.rasterize(shapes, out=grid, transform=tr, merge_alg=rf.MergeAlg.replace)
wet = np.zeros_like(grid)
for df in load('OIMK'):
    vals = df.RODZAJ.map({'teren podmokły':22,'bagno':23}).fillna(22).astype(int)
    rf.rasterize([(g, int(v)) for g, v in zip(df.geometry, vals)], out=wet, transform=tr)
forest = (grid >= 1) & (grid <= 3)
grid = np.where((wet > 0) & forest, 24, np.where(wet > 0, wet, grid)).astype(np.uint8)
del wet, forest
grid = grid[::-1]  # rows from the south, as the posts
fine = open(f'{outdir}/posts_fine.u8', 'wb'); cov = open(f'{outdir}/posts_cov.u8', 'wb')
pri = np.array(PRI)
with open(f'{outdir}/blocks.txt', 'w') as bl:
    for (bi, bj) in sorted(blocks):
        r0 = (bi - bi0) * 256; c0 = (bj - bj0) * 256
        g = grid[r0:r0+256, c0:c0+256].reshape(64, 4, 64, 4).transpose(0, 2, 1, 3).reshape(64, 64, 16)
        h = np.stack([(g == k).sum(-1) for k in range(len(NAMES))], -1)  # 64x64xK
        hc = h.copy(); hc[..., 0] = 0
        score = hc * 64 + pri[None, None, :] * (hc > 0)
        best = score.argmax(-1).astype(np.uint8)
        best[hc.max(-1) == 0] = 0
        fine.write(best.tobytes()); cov.write((16 - h[..., 0]).astype(np.uint8).tobytes())
        bl.write(f'{bi} {bj}\n')
print('done', outdir, nrows, ncols)

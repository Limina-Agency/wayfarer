import json, glob, os, sys
J = '/home/user/e1work/json'
def load(p):
    t = open(p).read()
    return json.loads(t)
MB = lambda b: f'{b/1e6:.2f}'
areas = ['malopolskie','tatra_podhale','gorce','niepolomice','ojcow']
sp = {60:'1/2"',120:'1"',240:'2"',360:'3"'}
print('## DEM sizes (MB, incl. 16 B/block index)\n')
print('| area | source | spacing | blocks | posts (M) | raw i16 | varint delta m | varint MED m | Rice MED m | Rice MED dm | zlib(varint MED m) | steep 4-bit packed | steep runs | steep zlib | t_total s |')
print('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
for a in areas:
    for src in ['glo30','srtm','nasadem']:
        for st in [60,120,240,360]:
            p = f'{J}/dem_{a}_{src}_{st}.json'
            if not os.path.exists(p): continue
            d = load(p); ix = d['index_bytes']
            row = [a, src, sp[st], d['stored_blocks'], f"{d['posts']/1e6:.2f}", MB(d['raw_i16_bytes']+ix), MB(d['m_delta_bytes']+ix), MB(d['m_med_bytes']+ix), MB(d['m_rice_bytes']+ix), MB(d['dm_rice_bytes']+ix), MB(d['m_med_zlib_bytes']+ix)]
            if 'steep_packed_bytes' in d:
                row += [MB(d['steep_packed_bytes']+ix), MB(d['steep_runs_bytes']+ix), MB(d['steep_zlib_bytes']+ix)]
            else: row += ['-','-','-']
            row += [d['t_total_s']]
            print('| ' + ' | '.join(map(str,row)) + ' |')
print('\n## Steepness-max vs slope from the lattice DEM (GLO-30)\n')
print('| area | spacing | cells | steep-max >30° | lattice slope >30° | hidden >30° (max says >30°, lattice ≤30°) | max ≥2 classes above lattice |')
print('|---|---|---|---|---|---|---|')
for a in areas:
    for src in ['glo30']:
        for st in [120,240,360]:
            d = load(f'{J}/dem_{a}_{src}_{st}.json')
            c = d['compared']; h = d['steep_minus_lattice_hist']
            print(f"| {a} | {sp[st]} | {c} | {d['max_over30']} ({100*d['max_over30']/c:.2f}%) | {d['lattice_over30']} ({100*d['lattice_over30']/c:.2f}%) | {d['hidden_over30']} ({100*d['hidden_over30']/c:.2f}%) | {100*sum(h[2:])/c:.1f}% |")
print('\n## Area mean vs point decimation at 3" (GLO-30)\n')
for a in areas:
    d = load(f'{J}/dempoint_{a}_glo30_360.json'); e = load(f'{J}/dem_{a}_glo30_360.json')
    print(f"- {a}: point Rice m {MB(d['m_rice_bytes'])} MB vs area mean {MB(e['m_rice_bytes'])} MB")
print('\n## Land cover (WorldCover 2021 v200)\n')
print('| area | spacing | blocks | packed 4-bit MB | runs MB | zlib(packed) MB | zlib(runs) MB | pure cells | majority <50% |')
print('|---|---|---|---|---|---|---|---|---|')
for a in areas:
    for st in [120,240,360]:
        d = load(f'{J}/lc_{a}_{st}.json'); ix = d['index_bytes']; ph = d['purity_hist']; tot = sum(ph.values())
        print(f"| {a} | {sp[st]} | {d['stored_blocks']} | {MB(d['packed_bytes']+ix)} | {MB(d['runs_bytes']+ix)} | {MB(d['packed_zlib_bytes']+ix)} | {MB(d['runs_zlib_bytes']+ix)} | {100*ph['pure']/tot:.1f}% | {100*ph['lt50']/tot:.1f}% |")
print('\n## Land cover class shares at 1" (posts)\n')
for a in areas:
    d = load(f'{J}/lc_{a}_120.json'); h = d['class_hist']; t = sum(h.values())
    print(f"- {a}: " + ', '.join(f"{k} {100*v/t:.1f}%" for k, v in h.items() if v))

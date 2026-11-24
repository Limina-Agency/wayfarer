# E4: fetch OSM public GPS trackpoints for the test areas (api.openstreetmap.org/api/0.6/trackpoints).
# Raw pages go to <raw-dir> (they carry trace names and user URLs: keep them out of shared folders).
# Stops at the first page whose points have no timestamps (the unordered non-trackable block) or a short page.
import sys, os, time, hashlib, urllib.request
AREAS = {  # name: (lat0, lon0, lat1, lon1), same boxes as E1 blocks.py
 'tatra_podhale': (49.15, 19.70, 49.45, 20.25),
 'gorce':         (49.48, 20.00, 49.64, 20.32),
 'niepolomice':   (49.98, 20.25, 50.13, 20.50),
 'ojcow':         (50.17, 19.76, 50.28, 19.88),
 # E4 only: more mountain trails for sample size
 'babia_gora':    (49.53, 19.42, 49.65, 19.70),
 'pieniny_sadecki': (49.35, 20.30, 49.50, 20.75),
}
UA = 'terrain-pace-experiment/0.1 (research; low rate)'
def get(url):
    for k in range(5):
        try:
            req = urllib.request.Request(url, headers={'User-Agent': UA})
            with urllib.request.urlopen(req, timeout=120) as r:
                return r.read()
        except Exception as e:
            print('retry', k, e, file=sys.stderr); time.sleep(10 * (k + 1))
    raise SystemExit('failed ' + url)
def main():
    raw = sys.argv[1]; names = sys.argv[2:] or list(AREAS)
    maxpages = int(os.environ.get('MAXPAGES', '400'))
    for name in names:
        la0, lo0, la1, lo1 = AREAS[name]
        os.makedirs(f'{raw}/{name}', exist_ok=True)
        for p in range(maxpages):
            fn = f'{raw}/{name}/p{p:04d}.gpx'
            if os.path.exists(fn):
                data = open(fn, 'rb').read()
            else:
                url = f'https://api.openstreetmap.org/api/0.6/trackpoints?bbox={lo0},{la0},{lo1},{la1}&page={p}'
                data = get(url); open(fn, 'wb').write(data); time.sleep(2)
            npts = data.count(b'<trkpt'); ntime = data.count(b'<time>')
            print(name, p, npts, ntime, hashlib.sha256(data).hexdigest()[:16], flush=True)
            if npts < 5000 or ntime == 0: break

if __name__ == '__main__':
    main()

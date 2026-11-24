import sys, numpy as np, rasterio, blake3, gzip
for f in sys.argv[1:]:
    if f.endswith('.gz'):
        a = np.frombuffer(gzip.open(f).read(), '>i2').reshape(3601,3601).astype(np.int64)
        v = np.where(a == -32768, np.iinfo(np.int32).min, a*10)
    else:
        with rasterio.open(f) as d:
            a = d.read(1).astype(np.float64); nd = d.nodata
        bad = ~np.isfinite(a) | (a < -20000) | (a > 20000)
        if nd is not None: bad |= (a == nd)
        v = np.where(bad, np.iinfo(np.int32).min, np.round(np.where(bad, 0, a)*10))  # numpy rounds half to even!
        # half-away-from-zero like Rust f64::round
        r = np.where(bad, 0, a*10); v2 = np.where(bad, np.iinfo(np.int32).min, np.sign(r)*np.floor(np.abs(r)+0.5))
        print('  half-even vs half-away differences:', int((v != v2).sum()))
        v = v2
    print(f.split('/')[-1], blake3.blake3(v.astype('<i4').tobytes()).hexdigest())

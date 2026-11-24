# EPSG:2180 (PUWG 1992): transverse Mercator on GRS80, lon0 19 E, k0 0.9993,
# FE 500000, FN -5300000. Krueger series (as in EPSG guidance note 7-2), numpy.
import numpy as np
a = 6378137.0; f = 1/298.257222101
n = f/(2-f); A = a/(1+n)*(1 + n**2/4 + n**4/64)
al = [n/2 - 2*n**2/3 + 5*n**3/16 + 41*n**4/180,
      13*n**2/48 - 3*n**3/5 + 557*n**4/1440,
      61*n**3/240 - 103*n**4/140,
      49561*n**4/161280]
e = np.sqrt(f*(2-f))
def fwd(lat, lon):
    phi = np.radians(lat); lam = np.radians(lon - 19.0)
    t = np.sinh(np.arctanh(np.sin(phi)) - e*np.arctanh(e*np.sin(phi)))
    xi = np.arctan2(t, np.cos(lam)); eta = np.arctanh(np.sin(lam)/np.sqrt(1+t*t))
    X = xi + sum(al[j]*np.sin(2*(j+1)*xi)*np.cosh(2*(j+1)*eta) for j in range(4))
    Y = eta + sum(al[j]*np.cos(2*(j+1)*xi)*np.sinh(2*(j+1)*eta) for j in range(4))
    k0 = 0.9993
    return 500000 + k0*A*Y, -5300000 + k0*A*X   # easting, northing
if __name__ == '__main__':
    # check against a known point: GUGiK sample t1.tif covers x 570000-571000, y 150000-151000
    print(fwd(49.0, 19.0), fwd(50.0, 20.0))

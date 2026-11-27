//! Coordinate systems of the sources: WGS 84 geographic, and UTM on WGS 84 (or on ETRS89,
//! whose ellipsoid differs from WGS 84's by under 0.1 mm here).
//!
//! The UTM forward projection is Krüger's series to sixth order in `n` (Karney 2011), in
//! `f64` with `libm`, which gives the same bits on every target; accurate to well under a
//! millimetre within a zone. It is the compile's only floating-point step: everything after
//! the projected coordinates is integer arithmetic.

use crate::{Error, Result};

/// A source's coordinate reference system.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Crs {
    /// EPSG:4326.
    Geographic,
    /// UTM zone 1-60, north or south.
    Utm { zone: u8, north: bool },
}

impl Crs {
    /// From an EPSG code: 4326, 326zz / 327zz (WGS 84 / UTM) or 258zz (ETRS89 / UTM, north).
    pub fn from_epsg(code: u16) -> Result<Crs> {
        match code {
            4326 => Ok(Crs::Geographic),
            32601..=32660 => Ok(Crs::Utm {
                zone: (code - 32600) as u8,
                north: true,
            }),
            32701..=32760 => Ok(Crs::Utm {
                zone: (code - 32700) as u8,
                north: false,
            }),
            25801..=25860 => Ok(Crs::Utm {
                zone: (code - 25800) as u8,
                north: true,
            }),
            _ => Err(Error::Source(format!(
                "EPSG:{code} is not supported (EPSG:4326 and UTM only)"
            ))),
        }
    }
}

const A: f64 = 6_378_137.0;
const F: f64 = 1.0 / 298.257_223_563;
const K0: f64 = 0.9996;

/// UTM forward projection for one zone, its constants computed once.
#[derive(Clone, Debug)]
pub struct Utm {
    lon0: f64,
    false_northing: f64,
    e: f64,
    ka: f64,
    alpha: [f64; 6],
}

impl Utm {
    pub fn new(zone: u8, north: bool) -> Utm {
        let n = F / (2.0 - F);
        let (n2, n3) = (n * n, n * n * n);
        let (n4, n5, n6) = (n2 * n2, n2 * n3, n3 * n3);
        let a_rect = A / (1.0 + n) * (1.0 + n2 / 4.0 + n4 / 64.0 + n6 / 256.0);
        let alpha = [
            n / 2.0 - 2.0 * n2 / 3.0 + 5.0 * n3 / 16.0 + 41.0 * n4 / 180.0 - 127.0 * n5 / 288.0
                + 7891.0 * n6 / 37800.0,
            13.0 * n2 / 48.0 - 3.0 * n3 / 5.0 + 557.0 * n4 / 1440.0 + 281.0 * n5 / 630.0
                - 1_983_433.0 * n6 / 1_935_360.0,
            61.0 * n3 / 240.0 - 103.0 * n4 / 140.0
                + 15061.0 * n5 / 26880.0
                + 167_603.0 * n6 / 181_440.0,
            49561.0 * n4 / 161_280.0 - 179.0 * n5 / 168.0 + 6_601_661.0 * n6 / 7_257_600.0,
            34729.0 * n5 / 80640.0 - 3_418_889.0 * n6 / 1_995_840.0,
            212_378_941.0 * n6 / 319_334_400.0,
        ];
        Utm {
            lon0: (zone as f64 * 6.0 - 183.0).to_radians(),
            false_northing: if north { 0.0 } else { 10_000_000.0 },
            e: libm::sqrt(F * (2.0 - F)),
            ka: K0 * a_rect,
            alpha,
        }
    }

    /// (easting, northing) in metres of a latitude and longitude in degrees.
    pub fn forward(&self, lat_deg: f64, lon_deg: f64) -> (f64, f64) {
        let phi = lat_deg.to_radians();
        let lam = lon_deg.to_radians() - self.lon0;
        let s = libm::sin(phi);
        let t = libm::sinh(libm::atanh(s) - self.e * libm::atanh(self.e * s));
        let cl = libm::cos(lam);
        let xi1 = libm::atan2(t, cl);
        let eta1 = libm::atanh(libm::sin(lam) / libm::sqrt(1.0 + t * t));
        let (mut xi, mut eta) = (xi1, eta1);
        for (j, a) in self.alpha.iter().enumerate() {
            let k = 2.0 * (j as f64 + 1.0);
            xi += a * libm::sin(k * xi1) * libm::cosh(k * eta1);
            eta += a * libm::cos(k * xi1) * libm::sinh(k * eta1);
        }
        (
            500_000.0 + self.ka * eta,
            self.false_northing + self.ka * xi,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn utm_reference_points() {
        // On the central meridian at the equator.
        let z = Utm::new(34, true);
        let (e, n) = z.forward(0.0, 21.0);
        assert!((e - 500_000.0).abs() < 1e-6 && n.abs() < 1e-6);
        // Reference values from PROJ (pyproj 3, EPSG:4326 to EPSG:32634 / 32734).
        for (zone, north, lat, lon, ee, nn) in [
            (34, true, 49.5, 20.0, 427_592.989_6, 5_483_521.327_8),
            (34, true, 50.3, 19.0, 357_565.044_6, 5_573_899.991_0),
            (34, false, -10.0, 21.0, 500_000.0, 8_894_587.508_7),
            (34, false, -40.0, 23.5, 713_411.147_9, 5_569_248.980_5),
        ] {
            let (e, n) = Utm::new(zone, north).forward(lat, lon);
            assert!(
                (e - ee).abs() < 0.001 && (n - nn).abs() < 0.001,
                "{lat} {lon}: {e} {n}"
            );
        }
    }

    #[test]
    fn epsg_codes() {
        assert_eq!(
            Crs::from_epsg(32634).unwrap(),
            Crs::Utm {
                zone: 34,
                north: true
            }
        );
        assert_eq!(
            Crs::from_epsg(32733).unwrap(),
            Crs::Utm {
                zone: 33,
                north: false
            }
        );
        assert_eq!(Crs::from_epsg(4326).unwrap(), Crs::Geographic);
        assert!(Crs::from_epsg(2180).is_err());
    }
}

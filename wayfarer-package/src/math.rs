//! Deterministic math for everything that ends up in a package.
//!
//! `f64::sin`, `cos`, `tan`, `asin`, `atan2`, `ln` and the like call the
//! platform's C math library, whose results differ in the last bits between
//! glibc, musl, Apple's libm, Bionic and the MSVC runtime. The compiler derives
//! node unit vectors, edge lengths and headings from them,
//! so the package bytes depended on the platform the compiler was built for.
//! These functions come from the `libm` crate instead, a pure-Rust port of
//! musl's libm: plain IEEE arithmetic, so the same bits on every target and in
//! every build profile (its only hardware paths are `sqrt`, `fma` and `rint`,
//! which IEEE 754 specifies exactly). `+ - * /`, `sqrt`, `round` and float
//! casts are exact by the standard and need no wrapper; `powi` is not (its
//! rounding depends on how LLVM expands it), so write `x * x`.
//!
//! The workspace `clippy.toml` disallows the platform functions; only the
//! engine's query-time code, whose results never reach a package, may use
//! them.

/// Sine of `x` radians.
#[inline]
pub fn sin(x: f64) -> f64 {
    libm::sin(x)
}

/// Cosine of `x` radians.
#[inline]
pub fn cos(x: f64) -> f64 {
    libm::cos(x)
}

/// Tangent of `x` radians.
#[inline]
pub fn tan(x: f64) -> f64 {
    libm::tan(x)
}

/// Arcsine of `x`, in radians.
#[inline]
pub fn asin(x: f64) -> f64 {
    libm::asin(x)
}

/// Four-quadrant arctangent of `y / x`, in radians.
#[inline]
pub fn atan2(y: f64, x: f64) -> f64 {
    libm::atan2(y, x)
}

/// Natural logarithm of `x`.
#[inline]
pub fn ln(x: f64) -> f64 {
    libm::log(x)
}

/// Great-circle distance in metres on a sphere of `radius_m` between two
/// (lat, lng) points in degrees, by the haversine formula as `geo`'s
/// `Haversine` computes it.
#[inline]
pub fn haversine_m(from: (f64, f64), to: (f64, f64), radius_m: f64) -> f64 {
    let (lat1, lat2) = (from.0.to_radians(), to.0.to_radians());
    let half_dlat = sin((to.0 - from.0).to_radians() / 2.0);
    let half_dlng = sin((to.1 - from.1).to_radians() / 2.0);
    let a = half_dlat * half_dlat + cos(lat1) * cos(lat2) * half_dlng * half_dlng;
    radius_m * (2.0 * asin(a.sqrt()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The bits these functions return are part of the package format: a
    /// `libm` upgrade that changes them changes package hashes.
    #[test]
    fn values_are_pinned() {
        let cases = [
            (sin(0.5), 0x3fdeaee8744b05f0_u64),
            (cos(0.5), 0x3fec1528065b7d50),
            (tan(0.5), 0x3fe17b4f5bf3474a),
            (asin(0.5), 0x3fe0c152382d7366),
            (atan2(0.5, -0.25), 0x4000468a8ace4df6),
            (ln(0.5), 0xbfe62e42fefa39ef),
        ];
        for (index, (value, bits)) in cases.into_iter().enumerate() {
            assert_eq!(value.to_bits(), bits, "case {index}: {value}");
        }
    }

    #[test]
    fn haversine_matches_a_known_distance() {
        // One millidegree of longitude at 52 N.
        let metres = haversine_m((52.0, 21.0), (52.0, 21.001), 6_371_008.8);
        assert_eq!(metres.to_bits(), 0x40511d58823d1b7a, "{metres}"); // 68.4585 m
    }
}

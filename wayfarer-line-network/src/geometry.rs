//! Geometry helpers: fixed-point snapping, lengths, Mercator vectors and
//! collinearity.

use super::*;

const SNAP_SCALE: f64 = 1e7;

const MERCATOR_COLLINEAR_EPSILON: f64 = 1e-12;

pub(crate) fn simplify_anchor_points(points: Vec<PointKey>) -> Vec<PointKey> {
    if points.len() < 3 {
        return points;
    }
    let mut simplified = Vec::with_capacity(points.len());
    simplified_indices(
        points.len(),
        |index| points[index],
        |index| {
            simplified.push(points[index]);
        },
    );
    simplified
}

/// The points of a stretch that simplification keeps, by index, in order:
/// its ends, and each inner point that is not collinear (in Mercator) with
/// the last point kept and the next one. `point(index)` is the point at
/// `index` of `count`; each point's Mercator position is computed once.
pub(crate) fn simplified_indices(
    count: usize,
    point: impl Fn(usize) -> PointKey,
    mut keep: impl FnMut(usize),
) {
    if count < 3 {
        (0..count).for_each(keep);
        return;
    }
    keep(0);
    let mut prev = mercator_point_for_key(point(0));
    let mut current = mercator_point_for_key(point(1));
    for index in 1..(count - 1) {
        let next = mercator_point_for_key(point(index + 1));
        if !is_mercator_collinear(prev, current, next) {
            keep(index);
            prev = current;
        }
        current = next;
    }
    keep(count - 1);
}

fn is_mercator_collinear(left: MercatorPoint, middle: MercatorPoint, right: MercatorPoint) -> bool {
    let dx = right.x - left.x;
    let dy = right.y - left.y;
    let length = (dx * dx + dy * dy).sqrt();
    if length == 0.0 {
        return true;
    }
    let area_twice =
        ((middle.x - left.x) * (right.y - left.y)) - ((middle.y - left.y) * (right.x - left.x));
    (area_twice.abs() / length) <= MERCATOR_COLLINEAR_EPSILON
}

/// The haversine length of the polyline, summed segment by segment as geo's
/// `Haversine.length` does.
pub(crate) fn line_length_m(points: &[PointKey]) -> f64 {
    let position = |point: &PointKey| (from_fixed(point.lat), from_fixed(point.lng));
    points.windows(2).fold(0.0, |length, pair| {
        length + haversine_m(position(&pair[0]), position(&pair[1]))
    })
}

/// Great-circle distance in metres on geo's mean Earth radius, by geo's
/// haversine formula but with the `libm` crate's trigonometry, which gives the
/// same bits on every platform (`f64::sin` & co call the platform libm).
fn haversine_m(from: (f64, f64), to: (f64, f64)) -> f64 {
    const MEAN_EARTH_RADIUS_M: f64 = 6_371_008.8;
    let (lat1, lat2) = (from.0.to_radians(), to.0.to_radians());
    let half_dlat = libm::sin((to.0 - from.0).to_radians() / 2.0);
    let half_dlng = libm::sin((to.1 - from.1).to_radians() / 2.0);
    let a = half_dlat * half_dlat + libm::cos(lat1) * libm::cos(lat2) * half_dlng * half_dlng;
    MEAN_EARTH_RADIUS_M * (2.0 * libm::asin(a.sqrt()))
}

#[derive(Debug, Clone, Copy)]
struct MercatorPoint {
    x: f64,
    y: f64,
}

pub(crate) fn mercator_vector(from: PointKey, to: PointKey) -> (f64, f64) {
    let from = mercator_point_for_key(from);
    let to = mercator_point_for_key(to);
    (to.x - from.x, to.y - from.y)
}

fn mercator_point_for_key(point: PointKey) -> MercatorPoint {
    mercator_point(from_fixed(point.lat), from_fixed(point.lng))
}

fn mercator_point(lat: f64, lng: f64) -> MercatorPoint {
    let clamped_lat = lat.clamp(-85.051_128_78, 85.051_128_78);
    let lat_rad = clamped_lat.to_radians();
    MercatorPoint {
        x: (lng + 180.0) / 360.0,
        y: ((1.0 - libm::log(libm::tan(lat_rad) + 1.0 / libm::cos(lat_rad)) / PI) / 2.0),
    }
}

pub(crate) fn to_fixed(value: f64) -> i64 {
    (value * SNAP_SCALE).round() as i64
}

pub(crate) fn from_fixed(value: i64) -> f64 {
    value as f64 / SNAP_SCALE
}

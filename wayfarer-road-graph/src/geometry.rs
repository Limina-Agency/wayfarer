//! Geometry helpers: fixed-point coordinates, path lengths and headings.

use super::*;

const COORD_SCALE: f64 = 1e7;

pub(crate) fn to_fixed(value: f64) -> i32 {
    (value * COORD_SCALE).round() as i32
}

/// The coordinate of a fixed-point value, as the line network computes it.
pub(crate) fn from_fixed(value: i32) -> f64 {
    f64::from(value) / COORD_SCALE
}

/// The haversine length of the polyline (geo's mean Earth radius), summed
/// segment by segment as geo's `Haversine.length` of a `LineString` does, with
/// deterministic trigonometry.
pub(crate) fn calculate_path_distance(points: impl Points) -> f64 {
    let mut length = 0.0;
    for index in 1..points.len() {
        length += math::haversine_m(
            points.at(index - 1),
            points.at(index),
            wayfarer_package::format::EARTH_RADIUS_M,
        );
    }
    length
}

/// A polyline's points by index: a slice, or a shared info's
/// [`InfoGeometry`].
pub(crate) trait Points: Copy {
    fn len(&self) -> usize;
    fn at(&self, index: usize) -> (f64, f64);
}

impl Points for &[(f64, f64)] {
    fn len(&self) -> usize {
        <[(f64, f64)]>::len(self)
    }

    fn at(&self, index: usize) -> (f64, f64) {
        self[index]
    }
}

impl<const N: usize> Points for &[(f64, f64); N] {
    fn len(&self) -> usize {
        N
    }

    fn at(&self, index: usize) -> (f64, f64) {
        self[index]
    }
}

impl Points for &Vec<(f64, f64)> {
    fn len(&self) -> usize {
        Vec::len(self)
    }

    fn at(&self, index: usize) -> (f64, f64) {
        self[index]
    }
}

impl Points for InfoGeometry<'_> {
    fn len(&self) -> usize {
        InfoGeometry::len(self)
    }

    fn at(&self, index: usize) -> (f64, f64) {
        self.get(index)
    }
}

pub(crate) fn edge_headings_deg(
    geometry: impl Points,
    direction: TraversalDirection,
) -> Option<(Option<u16>, Option<u16>)> {
    if geometry.len() < 2 {
        return None;
    }
    let start = match direction {
        TraversalDirection::Forward => heading_from_iter(
            (0..geometry.len() - 1).map(|index| (index, index + 1)),
            geometry,
        ),
        TraversalDirection::Backward => heading_from_iter(
            (1..geometry.len()).rev().map(|index| (index, index - 1)),
            geometry,
        ),
    };
    let end = match direction {
        TraversalDirection::Forward => heading_from_iter(
            (1..geometry.len()).rev().map(|index| (index - 1, index)),
            geometry,
        ),
        TraversalDirection::Backward => heading_from_iter(
            (0..geometry.len() - 1).map(|index| (index + 1, index)),
            geometry,
        ),
    };
    Some((start, end))
}

fn heading_from_iter<I>(pairs: I, geometry: impl Points) -> Option<u16>
where
    I: IntoIterator<Item = (usize, usize)>,
{
    for (from, to) in pairs {
        let Some(heading) = heading_between(geometry.at(from), geometry.at(to)) else {
            continue;
        };
        return Some(heading);
    }
    None
}

fn heading_between(from: (f64, f64), to: (f64, f64)) -> Option<u16> {
    let avg_lat = ((from.0 + to.0) * 0.5).to_radians();
    let dx = (to.1 - from.1) * math::cos(avg_lat);
    let dy = to.0 - from.0;
    if dx.abs() < f64::EPSILON && dy.abs() < f64::EPSILON {
        return None;
    }
    let mut degrees = math::atan2(dx, dy).to_degrees();
    if degrees < 0.0 {
        degrees += 360.0;
    }
    Some((degrees.round() as u16) % 360)
}

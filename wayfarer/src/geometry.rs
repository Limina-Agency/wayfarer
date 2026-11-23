//! Compact route geometry: encoded polylines at precision 6 (the format
//! OSRM and Valhalla use) and GeoJSON `LineString`s.
//!
//! A route request picks the form with [`GeometryFormat`]
//! (`geometryFormat` in JSON). Packages store coordinates at 1e-7 degrees,
//! so both encodings round to 1e-6 degrees: at most about 0.11 m off. They
//! keep every vertex, so shape indexes (`beginShapeIndex`, …) point into the
//! encoded vertices as they point into `geometry`.

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::GeoPoint;

/// How a route result carries its geometry.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "lowercase")]
pub enum GeometryFormat {
    /// `geometry`: an array of `{lat, lng}` points.
    #[default]
    Points,
    /// `polyline6`: an encoded polyline at precision 6; `geometry` is empty.
    Polyline6,
    /// `geojson`: a GeoJSON `LineString` (longitude first, 6 decimals);
    /// `geometry` is empty.
    Geojson,
}

impl GeometryFormat {
    /// Whether this is the default, `points`.
    pub fn is_points(&self) -> bool {
        *self == Self::Points
    }
}

/// The type member of a GeoJSON geometry; always `LineString` here.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub enum GeoJsonLineStringType {
    #[default]
    LineString,
}

/// A GeoJSON `LineString` geometry (RFC 7946): `[longitude, latitude]`
/// positions, rounded to 6 decimals.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct GeoJsonLineString {
    #[serde(rename = "type")]
    pub kind: GeoJsonLineStringType,
    pub coordinates: Vec<[f64; 2]>,
}

impl GeoJsonLineString {
    /// The line through `points`, rounded to 6 decimals.
    pub fn from_points(points: &[GeoPoint]) -> Self {
        Self {
            kind: GeoJsonLineStringType::LineString,
            coordinates: points
                .iter()
                .map(|point| [round6(point.lng), round6(point.lat)])
                .collect(),
        }
    }

    /// The line's positions as points.
    pub fn to_points(&self) -> Vec<GeoPoint> {
        self.coordinates
            .iter()
            .map(|[lng, lat]| GeoPoint::new(*lat, *lng))
            .collect()
    }
}

/// Moves `geometry` into its `format` encoding (`points` leaves it).
pub(crate) fn encode_into(
    format: GeometryFormat,
    geometry: &mut Vec<GeoPoint>,
    polyline6: &mut Option<String>,
    geojson: &mut Option<GeoJsonLineString>,
) {
    match format {
        GeometryFormat::Points => {}
        GeometryFormat::Polyline6 => {
            *polyline6 = Some(encode_polyline6(&std::mem::take(geometry)));
        }
        GeometryFormat::Geojson => {
            *geojson = Some(GeoJsonLineString::from_points(&std::mem::take(geometry)));
        }
    }
}

fn round6(value: f64) -> f64 {
    (value * 1e6).round() / 1e6
}

/// Why an encoded polyline could not be decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum PolylineError {
    #[error("polyline has a character outside '?'..='~' at byte {0}")]
    InvalidCharacter(usize),
    #[error("polyline ends inside a value")]
    Truncated,
    #[error("polyline has a value too long at byte {0}")]
    Overflow(usize),
    #[error("polyline ends after a latitude without its longitude")]
    UnpairedLatitude,
    #[error("polyline has a point outside the valid coordinate range at index {0}")]
    OutOfRange(usize),
}

/// The encoded polyline (precision 6) of `points`, latitude first.
pub fn encode_polyline6(points: &[GeoPoint]) -> String {
    // About 4–6 bytes per point on road shapes.
    let mut encoded = String::with_capacity(points.len() * 6 + 8);
    let (mut previous_lat, mut previous_lng) = (0_i64, 0_i64);
    for point in points {
        let lat = e6(point.lat);
        let lng = e6(point.lng);
        push_value(&mut encoded, lat - previous_lat);
        push_value(&mut encoded, lng - previous_lng);
        previous_lat = lat;
        previous_lng = lng;
    }
    encoded
}

/// The points of an encoded polyline at precision 6.
pub fn decode_polyline6(encoded: &str) -> Result<Vec<GeoPoint>, PolylineError> {
    let bytes = encoded.as_bytes();
    let mut points = Vec::with_capacity(bytes.len() / 4);
    let (mut lat, mut lng) = (0_i64, 0_i64);
    let mut at = 0;
    while at < bytes.len() {
        lat = lat.wrapping_add(read_value(bytes, &mut at)?);
        if at >= bytes.len() {
            return Err(PolylineError::UnpairedLatitude);
        }
        lng = lng.wrapping_add(read_value(bytes, &mut at)?);
        let point = GeoPoint::new(lat as f64 / 1e6, lng as f64 / 1e6);
        if !(-90.0..=90.0).contains(&point.lat) || !(-180.0..=180.0).contains(&point.lng) {
            return Err(PolylineError::OutOfRange(points.len()));
        }
        points.push(point);
    }
    Ok(points)
}

fn e6(degrees: f64) -> i64 {
    (degrees * 1e6).round() as i64
}

fn push_value(out: &mut String, value: i64) {
    let mut rest = ((value << 1) ^ (value >> 63)) as u64;
    while rest >= 0x20 {
        out.push(char::from((0x20 | (rest & 0x1f)) as u8 + 63));
        rest >>= 5;
    }
    out.push(char::from(rest as u8 + 63));
}

fn read_value(bytes: &[u8], at: &mut usize) -> Result<i64, PolylineError> {
    let start = *at;
    let mut value = 0_u64;
    let mut shift = 0;
    loop {
        let Some(&byte) = bytes.get(*at) else {
            return Err(PolylineError::Truncated);
        };
        if !(63..=126).contains(&byte) {
            return Err(PolylineError::InvalidCharacter(*at));
        }
        if shift > 60 {
            return Err(PolylineError::Overflow(start));
        }
        let chunk = u64::from(byte - 63);
        value |= (chunk & 0x1f) << shift;
        shift += 5;
        *at += 1;
        if chunk < 0x20 {
            break;
        }
    }
    let value = (value >> 1) as i64 ^ -((value & 1) as i64);
    Ok(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encodes_the_reference_polyline_at_precision_6() {
        // The Google example line, at precision 6.
        let points = [
            GeoPoint::new(38.5, -120.2),
            GeoPoint::new(40.7, -120.95),
            GeoPoint::new(43.252, -126.453),
        ];
        let encoded = encode_polyline6(&points);
        assert_eq!(encoded, "_izlhA~rlgdF_{geC~ywl@_kwzCn`{nI");
        assert_eq!(decode_polyline6(&encoded).unwrap(), points);
    }

    #[test]
    fn rounds_to_a_millionth_of_a_degree() {
        let points = [
            GeoPoint::new(43.739_712_3, 7.427_549_9),
            GeoPoint::new(43.739_712_3, 7.427_549_9),
            GeoPoint::new(-33.868_820_5, -151.209_295_5),
        ];
        let decoded = decode_polyline6(&encode_polyline6(&points)).unwrap();
        assert_eq!(decoded.len(), 3);
        assert_eq!(decoded[0], GeoPoint::new(43.739712, 7.42755));
        assert_eq!(decoded[1], decoded[0]);
        for (point, back) in points.iter().zip(&decoded) {
            assert!(point.distance_to(*back) < 0.12, "{point:?} {back:?}");
        }
        assert_eq!(encode_polyline6(&[]), "");
        assert_eq!(decode_polyline6("").unwrap(), Vec::new());
    }

    #[test]
    fn rejects_malformed_polylines() {
        assert_eq!(
            decode_polyline6("_izlhA"),
            Err(PolylineError::UnpairedLatitude)
        );
        assert_eq!(decode_polyline6("_izlh"), Err(PolylineError::Truncated));
        assert_eq!(
            decode_polyline6("_iz lhA~rlgdF"),
            Err(PolylineError::InvalidCharacter(3))
        );
        assert_eq!(
            decode_polyline6("~~~~~~~~~~~~~~~~?"),
            Err(PolylineError::Overflow(0))
        );
        // 91 degrees of latitude.
        let far = encode_polyline6(&[GeoPoint::new(91.0, 0.0)]);
        assert_eq!(decode_polyline6(&far), Err(PolylineError::OutOfRange(0)));
    }

    #[test]
    fn geojson_puts_longitude_first() {
        let line = GeoJsonLineString::from_points(&[
            GeoPoint::new(43.739_712_3, 7.427_5),
            GeoPoint::new(43.729, 7.416_000_04),
        ]);
        assert_eq!(
            serde_json::to_string(&line).unwrap(),
            r#"{"type":"LineString","coordinates":[[7.4275,43.739712],[7.416,43.729]]}"#
        );
        assert_eq!(line.to_points()[1], GeoPoint::new(43.729, 7.416));
    }
}

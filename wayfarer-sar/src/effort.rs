//! Search effort a host measured itself (a drone's video coverage grid, an
//! area a team swept and scored), counted as evidence beside team tracks.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use wayfarer::GeoPoint;

use crate::network::{closed, distance_m};
use crate::reach::Edges;
use crate::{SarError, finite_positive, invalid_point, unit_interval};

/// The most efforts of one coverage request.
pub const MAX_EFFORTS: usize = 256;
/// The most cells or edges over all the efforts of one coverage request.
pub const MAX_EFFORT_ITEMS: usize = 1_000_000;
/// The largest effort grid cell, in degrees on either side.
pub const MAX_EFFORT_CELL_DEG: f64 = 1.0;

/// How well one place was searched.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
#[non_exhaustive]
pub enum Detection {
    /// The probability the search would have detected the person there, in
    /// [0, 1].
    Pod(f64),
    /// Seconds the place was seen (a camera's dwell time), at least 0: the
    /// probability of detection is `1 - exp(-seenS / tauS)` with the
    /// effort's `tau_s`.
    SeenS(f64),
}

/// One edge's detection (`edgeId`: either directed id, or the undirected
/// key).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
#[non_exhaustive]
pub struct EdgeEffort {
    pub edge_id: u64,
    #[serde(flatten)]
    pub detection: Detection,
}

impl EdgeEffort {
    pub fn new(edge_id: u64, detection: Detection) -> Self {
        Self { edge_id, detection }
    }
}

/// One grid cell's detection: row 0 is the grid's south row, column 0 its
/// west column.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
#[non_exhaustive]
pub struct CellEffort {
    pub row: u32,
    pub col: u32,
    #[serde(flatten)]
    pub detection: Detection,
}

impl CellEffort {
    pub fn new(row: u32, col: u32, detection: Detection) -> Self {
        Self {
            row,
            col,
            detection,
        }
    }
}

/// Where an effort searched.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case", tag = "kind")]
#[non_exhaustive]
pub enum EffortArea {
    /// Per edge, as a host that already knows the network's edges scores
    /// them.
    Edges { edges: Vec<EdgeEffort> },
    /// A grid of equal cells in degrees from `southWest`, listing only the
    /// cells that were searched (the rest count as not searched). An edge
    /// takes the length-weighted mean of the cells along its shape: the
    /// chance of detecting a person anywhere on it.
    Grid {
        #[serde(rename = "southWest")]
        south_west: GeoPoint,
        #[serde(rename = "cellLatDeg")]
        cell_lat_deg: f64,
        #[serde(rename = "cellLngDeg")]
        cell_lng_deg: f64,
        cells: Vec<CellEffort>,
    },
    /// One detection over a whole area (a sweep the team scored): every edge
    /// inside the polygon takes it.
    Polygon {
        polygon: Vec<GeoPoint>,
        #[serde(flatten)]
        detection: Detection,
    },
}

/// Search effort the host measured: a source (a drone, a camera, a team's
/// own sweep) and where and how well it searched. Efforts and tracks are
/// independent searches: an edge's misses multiply.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
#[non_exhaustive]
pub struct SearchEffort {
    /// Who or what searched, for the report.
    pub source_id: String,
    pub area: EffortArea,
    /// Seconds of viewing that detect with probability `1 - 1/e` (about
    /// 63 %): the time constant of `Detection::SeenS`. Needed when any of the
    /// effort's detections is in seconds.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tau_s: Option<f64>,
}

impl SearchEffort {
    pub fn new(source_id: impl Into<String>, area: EffortArea) -> Self {
        Self {
            source_id: source_id.into(),
            area,
            tau_s: None,
        }
    }

    /// Cells or edges it lists (a polygon counts once).
    pub(crate) fn item_count(&self) -> usize {
        match &self.area {
            EffortArea::Edges { edges } => edges.len(),
            EffortArea::Grid { cells, .. } => cells.len(),
            EffortArea::Polygon { .. } => 1,
        }
    }

    fn detections(&self) -> Box<dyn Iterator<Item = Detection> + '_> {
        match &self.area {
            EffortArea::Edges { edges } => Box::new(edges.iter().map(|edge| edge.detection)),
            EffortArea::Grid { cells, .. } => Box::new(cells.iter().map(|cell| cell.detection)),
            EffortArea::Polygon { detection, .. } => Box::new(std::iter::once(*detection)),
        }
    }

    pub(crate) fn validate(&self) -> Result<(), SarError> {
        let invalid = |what: &str| {
            Err(SarError::invalid(format!(
                "effort {}: {what}",
                self.source_id
            )))
        };
        if self.tau_s.is_some_and(|tau| !finite_positive(tau)) {
            return invalid("tauS must be above 0");
        }
        for detection in self.detections() {
            match detection {
                Detection::Pod(pod) if !unit_interval(pod) => {
                    return invalid("pod values must be in [0, 1]");
                }
                Detection::SeenS(seen) if !(seen.is_finite() && seen >= 0.0) => {
                    return invalid("seenS values must be 0 or more");
                }
                Detection::SeenS(_) if self.tau_s.is_none() => {
                    return invalid("seenS needs the effort's tauS");
                }
                _ => {}
            }
        }
        match &self.area {
            EffortArea::Edges { .. } => {}
            EffortArea::Grid {
                south_west,
                cell_lat_deg,
                cell_lng_deg,
                ..
            } => {
                if invalid_point(*south_west)
                    || ![*cell_lat_deg, *cell_lng_deg]
                        .iter()
                        .all(|size| finite_positive(*size) && *size <= MAX_EFFORT_CELL_DEG)
                {
                    return invalid("grid corner or cell size (above 0, at most 1 degree)");
                }
            }
            EffortArea::Polygon { polygon, .. } => {
                if polygon.len() < 3 || polygon.iter().any(|point| invalid_point(*point)) {
                    return invalid("a polygon of at least three valid points");
                }
            }
        }
        Ok(())
    }

    fn pod(&self, detection: Detection) -> f64 {
        match detection {
            Detection::Pod(pod) => pod,
            Detection::SeenS(seen) => {
                let tau = self.tau_s.unwrap_or(f64::INFINITY);
                1.0 - libm::exp(-seen / tau)
            }
        }
    }

    /// The effort's probability of detection per undirected key, for every
    /// edge it searched (above 0).
    pub(crate) fn edge_pods(&self, edges: &mut Edges) -> BTreeMap<u64, f64> {
        let mut pods = BTreeMap::new();
        match &self.area {
            EffortArea::Edges { edges: listed } => {
                for edge in listed {
                    let key = edges.network().key(edge.edge_id);
                    let miss = 1.0 - self.pod(edge.detection);
                    // The same edge listed twice (both directions): two looks.
                    let entry = pods.entry(key).or_insert(1.0);
                    *entry *= miss;
                }
                pods.values_mut().for_each(|miss| *miss = 1.0 - *miss);
            }
            EffortArea::Polygon { polygon, detection } => {
                let pod = self.pod(*detection);
                for key in edges.network().keys_in_polygon(polygon) {
                    pods.insert(key, pod);
                }
            }
            EffortArea::Grid {
                south_west,
                cell_lat_deg,
                cell_lng_deg,
                cells,
            } => {
                let grid = Grid {
                    south_west: *south_west,
                    cell_lat: *cell_lat_deg,
                    cell_lng: *cell_lng_deg,
                    cells: cells
                        .iter()
                        .map(|cell| ((cell.row, cell.col), self.pod(cell.detection)))
                        .collect(),
                };
                let Some(bounds) = grid.bounds() else {
                    return pods;
                };
                for key in edges.network().keys_in_polygon(&bounds) {
                    let Some(edge) = edges.get(key) else {
                        continue;
                    };
                    let pod = grid.mean_along(&edge.geometry);
                    if pod > 0.0 {
                        pods.insert(key, pod);
                    }
                }
            }
        }
        pods.retain(|_, pod| *pod > 0.0);
        pods
    }
}

struct Grid {
    south_west: GeoPoint,
    cell_lat: f64,
    cell_lng: f64,
    cells: BTreeMap<(u32, u32), f64>,
}

impl Grid {
    /// The box around the listed cells.
    fn bounds(&self) -> Option<Vec<GeoPoint>> {
        let rows = self.cells.keys().map(|(row, _)| *row);
        let cols = self.cells.keys().map(|(_, col)| *col);
        let (south, north) = (rows.clone().min()?, rows.max()?);
        let (west, east) = (cols.clone().min()?, cols.max()?);
        let lat = |row: u32| self.south_west.lat + f64::from(row) * self.cell_lat;
        let lng = |col: u32| self.south_west.lng + f64::from(col) * self.cell_lng;
        Some(closed(&[
            GeoPoint::new(lat(south), lng(west)),
            GeoPoint::new(lat(south), lng(east + 1)),
            GeoPoint::new(lat(north + 1), lng(east + 1)),
            GeoPoint::new(lat(north + 1), lng(west)),
        ]))
    }

    fn pod_at(&self, point: GeoPoint) -> f64 {
        let row = ((point.lat - self.south_west.lat) / self.cell_lat).floor();
        let col = ((point.lng - self.south_west.lng) / self.cell_lng).floor();
        if !(0.0..=f64::from(u32::MAX)).contains(&row)
            || !(0.0..=f64::from(u32::MAX)).contains(&col)
        {
            return 0.0;
        }
        self.cells
            .get(&(row as u32, col as u32))
            .copied()
            .unwrap_or(0.0)
    }

    /// The length-weighted mean of the cells along a polyline, sampled at a
    /// quarter of the smaller cell side (at most 4,096 pieces per segment).
    fn mean_along(&self, shape: &[GeoPoint]) -> f64 {
        let corner = self.south_west;
        let side_m = distance_m(
            corner,
            GeoPoint::new(corner.lat + self.cell_lat, corner.lng),
        )
        .min(distance_m(
            corner,
            GeoPoint::new(corner.lat, corner.lng + self.cell_lng),
        ));
        let step = (side_m / 4.0).max(0.5);
        let (mut total, mut weighted) = (0.0, 0.0);
        for pair in shape.windows(2) {
            let length = distance_m(pair[0], pair[1]);
            if length <= 0.0 {
                continue;
            }
            let pieces = ((length / step).ceil() as usize).clamp(1, 4_096);
            let piece = length / pieces as f64;
            for index in 0..pieces {
                let t = (index as f64 + 0.5) / pieces as f64;
                let point = GeoPoint::new(
                    pair[0].lat + (pair[1].lat - pair[0].lat) * t,
                    pair[0].lng + (pair[1].lng - pair[0].lng) * t,
                );
                weighted += piece * self.pod_at(point);
                total += piece;
            }
        }
        if total > 0.0 {
            weighted / total
        } else {
            shape.first().map_or(0.0, |point| self.pod_at(*point))
        }
    }
}

/// What one effort covered, for the report.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
#[non_exhaustive]
pub struct EffortCoverage {
    pub source_id: String,
    /// Edges it searched (detection above 0).
    pub edge_count: usize,
    pub covered_length_m: f64,
    /// The length-weighted mean detection over those edges.
    pub mean_pod: f64,
}

pub(crate) fn effort_coverage(
    effort: &SearchEffort,
    pods: &BTreeMap<u64, f64>,
    edges: &mut Edges,
) -> EffortCoverage {
    let (mut length, mut weighted) = (0.0, 0.0);
    for (key, pod) in pods {
        let edge_length = edges.length(*key);
        length += edge_length;
        weighted += edge_length * pod;
    }
    EffortCoverage {
        source_id: effort.source_id.clone(),
        edge_count: pods.len(),
        covered_length_m: length,
        mean_pod: if length > 0.0 { weighted / length } else { 0.0 },
    }
}

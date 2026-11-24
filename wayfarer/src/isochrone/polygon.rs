//! Isochrone buckets drawn as polygons (FEAT-A1).
//!
//! Each bucket's reached roads are drawn on one square grid in a local
//! metric frame: a grid node's value is how far it lies beyond the walk-in
//! reach of the nearest reached road (negative within it), and marching
//! squares draws the rings where that value crosses zero. Edges the bucket
//! reaches in full reach `spread` off the road all along; on a boundary
//! fragment the reach shrinks to what remains of the contour at the
//! profile's off-road pace, down to nothing where the contour ends.
//!
//! The polygons are a pure function of the buckets (`edge_ids` and boundary
//! fragments), the package's road shapes and the options: the reference
//! oracle's buckets draw the same polygons. The builder uses only
//! `+ - * / sqrt` and `wayfarer_package::math`, integer grid indexes and a
//! fixed iteration order, so the output is the same on every platform.

use crate::GeoJsonMultiPolygon;
use crate::limits::MAX_ISOCHRONE_POLYGON_GRID_NODES;
use crate::package::SearchGraph;
use crate::package::search::analysis::resolve_edge_id;
use crate::types::{
    GeoPoint, MobilityProfile, RouteError, RouteIsochroneBucket, RouteIsochroneMetric,
    RouteIsochronePolygonOptions, RouteIsochroneRequest, RouteIsochroneResult,
};

/// The default grid spans the reached area's larger side in about this many
/// cells.
const DEFAULT_GRID_CELLS: f64 = 256.0;
/// The default cell size's bounds, in metres.
const DEFAULT_RESOLUTION_M: (f64, f64) = (10.0, 500.0);
/// The bounds of a requested cell size, in metres: crossings stay apart by
/// more than the 6-decimal rounding of the output can move them (see
/// [`CROSSING_MARGIN`]).
const RESOLUTION_M: (f64, f64) = (2.0, 10_000.0);
/// The most a requested spread may be, in metres.
const MAX_SPREAD_M: f64 = 10_000.0;
/// The default spread, in cells.
const DEFAULT_SPREAD_CELLS: f64 = 1.5;
/// The least spread, in cells: no point is farther than 0.71 of a cell
/// from a grid node, so every reached road reaches one.
const MIN_SPREAD_CELLS: f64 = 0.75;
/// How close to a grid node a ring may cross a grid edge, in cells. Two
/// crossings of different edges then lie at least 0.14 of a cell apart
/// (0.28 m at the least cell size), more than rounding to 6 decimals moves
/// two positions together (at most about 0.16 m), so rings that do not
/// touch on the grid do not touch in the output either.
const CROSSING_MARGIN: f64 = 0.1;
/// Mean earth radius of `wayfarer_package::haversine_m`, for the local frame.
const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// `InvalidRequest` for polygon options out of range (see
/// [`RouteIsochronePolygonOptions`]).
pub(crate) fn validate(options: &RouteIsochronePolygonOptions) -> Result<(), RouteError> {
    let within = |value: Option<f64>, (least, most): (f64, f64)| {
        value.is_none_or(|value| value.is_finite() && (least..=most).contains(&value))
    };
    if within(options.resolution_m, RESOLUTION_M)
        && within(options.spread_m, (0.0, MAX_SPREAD_M))
        && within(options.denoise, (0.0, 1.0))
    {
        Ok(())
    } else {
        Err(RouteError::InvalidRequest {
            reason: "isochrone polygon options",
        })
    }
}

/// Draws every bucket of `result` as a polygon when the request asks for
/// polygons, and records the options as applied. Needs the buckets' edge
/// ids, so it runs before `include_edge_ids: false` drops them.
pub(crate) fn draw(
    view: SearchGraph<'_>,
    request: &RouteIsochroneRequest,
    result: &mut RouteIsochroneResult,
) {
    let Some(options) = &request.polygons else {
        return;
    };
    let shapes = Shapes { view };
    let applied = draw_buckets(&shapes, request.profile, options, &mut result.buckets);
    result.polygons = Some(applied);
}

/// Road shapes by edge id.
pub(crate) trait EdgeShapes {
    /// The directed shape of the edge with `edge_id` into `out` (empty when
    /// the package has no such edge).
    fn shape(&self, edge_id: u64, out: &mut Vec<GeoPoint>);
}

struct Shapes<'g> {
    view: SearchGraph<'g>,
}

impl EdgeShapes for Shapes<'_> {
    fn shape(&self, edge_id: u64, out: &mut Vec<GeoPoint>) {
        out.clear();
        if let Some(edge) = resolve_edge_id(self.view, edge_id) {
            self.view.edge_shape(edge, out);
        }
    }
}

/// How far off a road a contour's walk-in reaches per unit of the metric
/// left: metres per second (or per unit of cost, which is close to seconds)
/// at the profile's off-road pace, one metre per metre for distance.
fn walk_rate(metric: RouteIsochroneMetric, profile: MobilityProfile) -> f64 {
    match metric {
        RouteIsochroneMetric::DistanceM => 1.0,
        RouteIsochroneMetric::Cost | RouteIsochroneMetric::TimeSeconds => {
            crate::costing::profile_off_road_speed_mps(profile)
        }
    }
}

/// Draws each bucket on one grid over everything the buckets reach and
/// returns the options as applied.
pub(crate) fn draw_buckets(
    shapes: &impl EdgeShapes,
    profile: MobilityProfile,
    options: &RouteIsochronePolygonOptions,
    buckets: &mut [RouteIsochroneBucket],
) -> RouteIsochronePolygonOptions {
    let denoise = options.denoise.unwrap_or(0.0);
    let mut shape = Vec::new();
    // The reached area: every reached edge's shape and every fragment's.
    let mut bounds = Bounds::default();
    let mut edges = buckets
        .iter()
        .flat_map(|bucket| {
            bucket.edge_ids.iter().copied().chain(
                bucket
                    .boundary_fragments
                    .iter()
                    .map(|fragment| fragment.edge_id),
            )
        })
        .collect::<Vec<_>>();
    edges.sort_unstable();
    edges.dedup();
    for edge_id in edges {
        shapes.shape(edge_id, &mut shape);
        shape.iter().for_each(|point| bounds.add(*point));
    }
    let Some(grid) = Grid::over(&bounds, options) else {
        for bucket in buckets.iter_mut() {
            bucket.polygon = Some(GeoJsonMultiPolygon::from_rings(&[]));
        }
        return RouteIsochronePolygonOptions {
            resolution_m: Some(options.resolution_m.unwrap_or(DEFAULT_RESOLUTION_M.0)),
            spread_m: Some(options.spread_m.unwrap_or(
                DEFAULT_SPREAD_CELLS * options.resolution_m.unwrap_or(DEFAULT_RESOLUTION_M.0),
            )),
            denoise: Some(denoise),
        };
    };
    let mut field = Vec::new();
    for bucket in buckets.iter_mut() {
        grid.fill(
            &mut field,
            shapes,
            bucket,
            walk_rate(bucket.metric, profile),
            &mut shape,
        );
        let polygons = grid.polygons(&field, denoise);
        bucket.polygon = Some(GeoJsonMultiPolygon::from_rings(&polygons));
    }
    RouteIsochronePolygonOptions {
        resolution_m: Some(grid.cell),
        spread_m: Some(grid.spread),
        denoise: Some(denoise),
    }
}

/// A latitude/longitude box.
#[derive(Debug, Clone, Copy)]
struct Bounds {
    lat: (f64, f64),
    lng: (f64, f64),
}

impl Default for Bounds {
    fn default() -> Self {
        Self {
            lat: (f64::INFINITY, f64::NEG_INFINITY),
            lng: (f64::INFINITY, f64::NEG_INFINITY),
        }
    }
}

impl Bounds {
    fn add(&mut self, point: GeoPoint) {
        self.lat = (self.lat.0.min(point.lat), self.lat.1.max(point.lat));
        self.lng = (self.lng.0.min(point.lng), self.lng.1.max(point.lng));
    }

    fn is_empty(&self) -> bool {
        !(self.lat.0 <= self.lat.1 && self.lng.0 <= self.lng.1)
    }
}

/// The grid: an equirectangular frame around the reached area's centre,
/// with nodes `cell` metres apart from `(x0, y0)`, `nx` by `ny`, row by row
/// from the south-west. The outer two cells and the spread around the
/// reached area are margin, so the border nodes are always outside.
#[derive(Debug, Clone)]
struct Grid {
    lat0: f64,
    lng0: f64,
    /// Metres per degree of latitude and of longitude.
    m_per_deg: (f64, f64),
    cell: f64,
    spread: f64,
    x0: f64,
    y0: f64,
    nx: usize,
    ny: usize,
}

impl Grid {
    /// The grid over `bounds`, or none when nothing was reached.
    fn over(bounds: &Bounds, options: &RouteIsochronePolygonOptions) -> Option<Self> {
        if bounds.is_empty() {
            return None;
        }
        let lat0 = (bounds.lat.0 + bounds.lat.1) / 2.0;
        let lng0 = (bounds.lng.0 + bounds.lng.1) / 2.0;
        let m_per_deg_lat = EARTH_RADIUS_M * std::f64::consts::PI / 180.0;
        let m_per_deg = (
            m_per_deg_lat,
            m_per_deg_lat * wayfarer_package::math::cos(lat0.to_radians()),
        );
        let (x_min, y_min) = (
            (bounds.lng.0 - lng0) * m_per_deg.1,
            (bounds.lat.0 - lat0) * m_per_deg.0,
        );
        let (x_max, y_max) = (
            (bounds.lng.1 - lng0) * m_per_deg.1,
            (bounds.lat.1 - lat0) * m_per_deg.0,
        );
        // The default in whole metres.
        let mut cell = options.resolution_m.unwrap_or_else(|| {
            ((x_max - x_min).max(y_max - y_min) / DEFAULT_GRID_CELLS)
                .ceil()
                .clamp(DEFAULT_RESOLUTION_M.0, DEFAULT_RESOLUTION_M.1)
        });
        loop {
            let spread = options
                .spread_m
                .unwrap_or(DEFAULT_SPREAD_CELLS * cell)
                .max(MIN_SPREAD_CELLS * cell);
            let margin = spread + 2.0 * cell;
            let x0 = ((x_min - margin) / cell).floor() * cell;
            let y0 = ((y_min - margin) / cell).floor() * cell;
            let nx = ((x_max + margin - x0) / cell).ceil() as usize + 1;
            let ny = ((y_max + margin - y0) / cell).ceil() as usize + 1;
            let nodes = nx.saturating_mul(ny);
            if nodes <= MAX_ISOCHRONE_POLYGON_GRID_NODES {
                return Some(Self {
                    lat0,
                    lng0,
                    m_per_deg,
                    cell,
                    spread,
                    x0,
                    y0,
                    nx,
                    ny,
                });
            }
            // Larger cells until the grid fits.
            cell *= (nodes as f64 / MAX_ISOCHRONE_POLYGON_GRID_NODES as f64)
                .sqrt()
                .max(1.01);
        }
    }

    fn to_xy(&self, point: GeoPoint) -> (f64, f64) {
        (
            (point.lng - self.lng0) * self.m_per_deg.1,
            (point.lat - self.lat0) * self.m_per_deg.0,
        )
    }

    fn to_geo(&self, (x, y): (f64, f64)) -> GeoPoint {
        GeoPoint::new(
            self.lat0 + y / self.m_per_deg.0,
            self.lng0 + x / self.m_per_deg.1,
        )
    }

    /// The bucket's field: each node's distance beyond the walk-in reach
    /// of the nearest reached road, capped at one cell (negative within it).
    fn fill(
        &self,
        field: &mut Vec<f64>,
        shapes: &impl EdgeShapes,
        bucket: &RouteIsochroneBucket,
        walk_rate: f64,
        shape: &mut Vec<GeoPoint>,
    ) {
        field.clear();
        field.resize(self.nx * self.ny, self.cell);
        let mut line = Vec::<((f64, f64), f64)>::new();
        for edge_id in &bucket.edge_ids {
            shapes.shape(*edge_id, shape);
            line.clear();
            line.extend(shape.iter().map(|point| (self.to_xy(*point), self.spread)));
            self.splat_line(field, &line);
        }
        let contour = bucket.contour;
        for fragment in &bucket.boundary_fragments {
            shapes.shape(fragment.edge_id, shape);
            let points = shape
                .iter()
                .map(|point| self.to_xy(*point))
                .collect::<Vec<_>>();
            let (from, to) = (fragment.start_fraction, fragment.end_fraction);
            // Each vertex's fraction along the edge, by planar length.
            let mut along = Vec::with_capacity(points.len());
            let mut total = 0.0;
            for (index, point) in points.iter().enumerate() {
                if index > 0 {
                    total += distance(points[index - 1], *point);
                }
                along.push(total);
            }
            // The walk-in reach left at fraction `at` of the edge.
            let reach = |at: f64| {
                let metric = if to - from > f64::EPSILON {
                    fragment.start_metric
                        + (fragment.end_metric - fragment.start_metric) * (at - from) / (to - from)
                } else {
                    fragment.end_metric
                };
                ((contour - metric) * walk_rate).clamp(0.0, self.spread)
            };
            line.clear();
            line.push((self.to_xy(fragment.start), reach(from)));
            if total > 0.0 {
                for (point, length) in points.iter().zip(&along) {
                    let at = length / total;
                    if from < at && at < to {
                        line.push((*point, reach(at)));
                    }
                }
            }
            line.push((self.to_xy(fragment.end), reach(to)));
            self.splat_line(field, &line);
        }
    }

    /// Splats points along `line` (positions with their reach) at most half
    /// a cell apart, the reach interpolated between vertices.
    fn splat_line(&self, field: &mut [f64], line: &[((f64, f64), f64)]) {
        let Some(first) = line.first() else {
            return;
        };
        self.splat(field, first.0, first.1);
        for pair in line.windows(2) {
            let (((ax, ay), ar), ((bx, by), br)) = (pair[0], pair[1]);
            let steps = (distance((ax, ay), (bx, by)) / (self.cell / 2.0))
                .ceil()
                .max(1.0) as usize;
            for step in 1..=steps {
                let t = step as f64 / steps as f64;
                self.splat(
                    field,
                    (ax + (bx - ax) * t, ay + (by - ay) * t),
                    ar + (br - ar) * t,
                );
            }
        }
    }

    /// Lowers each node near `(x, y)` to its distance from the point less
    /// `reach`. Nodes more than a cell beyond the reach keep the cap.
    fn splat(&self, field: &mut [f64], (x, y): (f64, f64), reach: f64) {
        let window = reach + self.cell;
        let index = |value: f64, origin: f64, count: usize| {
            ((value - origin) / self.cell).clamp(0.0, (count - 1) as f64)
        };
        let i0 = index(x - window, self.x0, self.nx).ceil() as usize;
        let i1 = index(x + window, self.x0, self.nx).floor() as usize;
        let j0 = index(y - window, self.y0, self.ny).ceil() as usize;
        let j1 = index(y + window, self.y0, self.ny).floor() as usize;
        for j in j0..=j1 {
            let dy = self.y0 + j as f64 * self.cell - y;
            let row = &mut field[j * self.nx..(j + 1) * self.nx];
            for (i, node) in row.iter_mut().enumerate().take(i1 + 1).skip(i0) {
                let dx = self.x0 + i as f64 * self.cell - x;
                let value = (dx * dx + dy * dy).sqrt() - reach;
                if value < *node {
                    *node = value;
                }
            }
        }
    }

    /// The polygons where `field` is at most zero, each its outer ring then
    /// its holes, in geographic coordinates (see [`rings`] and
    /// [`assemble`]).
    fn polygons(&self, field: &[f64], denoise: f64) -> Vec<Vec<Vec<GeoPoint>>> {
        let rings = rings(field, self.nx, self.ny);
        assemble(rings, denoise)
            .into_iter()
            .map(|polygon| {
                polygon
                    .into_iter()
                    .map(|ring| {
                        ring.into_iter()
                            .map(|(i, j)| {
                                self.to_geo((self.x0 + i * self.cell, self.y0 + j * self.cell))
                            })
                            .collect()
                    })
                    .collect()
            })
            .collect()
    }
}

fn distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    (dx * dx + dy * dy).sqrt()
}

/// A ring in grid units (`(i, j)`, fractional along the cell edges), not
/// closed (the last vertex does not repeat the first).
type Ring = Vec<(f64, f64)>;

/// The rings of the region where `field` (`nx` by `ny` nodes, row by row)
/// is at most zero, by marching squares: each with the region on its left,
/// so outer boundaries run counterclockwise and holes clockwise. A saddle
/// cell joins its inside corners when the mean of its four nodes is inside.
/// Every border node must be outside. Rings start at their least crossing
/// and come in the order of their least crossings.
fn rings(field: &[f64], nx: usize, ny: usize) -> Vec<Ring> {
    let inside = |i: usize, j: usize| field[j * nx + i] <= 0.0;
    // A crossing is named by its grid edge: 2 × the lower node's index, plus
    // 1 for an edge going north.
    let mut segments = Vec::<(u64, u64)>::new();
    for j in 0..ny.saturating_sub(1) {
        for i in 0..nx.saturating_sub(1) {
            // Corners counterclockwise from the south-west, and edge k
            // from corner k to corner k + 1.
            let corners = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)];
            let ins = corners.map(|(i, j)| inside(i, j));
            if ins.iter().all(|inside| *inside) || ins.iter().all(|inside| !*inside) {
                continue;
            }
            let edge_id = |k: usize| -> u64 {
                let node = |(i, j): (usize, usize)| (j * nx + i) as u64;
                match k {
                    0 => 2 * node((i, j)),
                    1 => 2 * node((i + 1, j)) + 1,
                    2 => 2 * node((i, j + 1)),
                    _ => 2 * node((i, j)) + 1,
                }
            };
            let saddle = ins[0] == ins[2] && ins[1] == ins[3];
            for k in 0..4 {
                // Leaving the region across edge k: corner k in, k + 1 out.
                if !(ins[k] && !ins[(k + 1) % 4]) {
                    continue;
                }
                let entering = if saddle {
                    let mean = corners.iter().map(|(i, j)| field[j * nx + i]).sum::<f64>() / 4.0;
                    if mean <= 0.0 {
                        // Joined: around the outside corner k + 1.
                        (k + 1) % 4
                    } else {
                        // Apart: around the inside corner k.
                        (k + 3) % 4
                    }
                } else {
                    // The one edge entering the region: corner m out, m + 1 in.
                    (0..4)
                        .find(|m| !ins[*m] && ins[(m + 1) % 4])
                        .expect("a mixed cell enters its region")
                };
                segments.push((edge_id(k), edge_id(entering)));
            }
        }
    }
    segments.sort_unstable();
    let position = |id: u64| {
        let node = (id / 2) as usize;
        let (i, j) = (node % nx, node / nx);
        let (a, b, north) = if id.is_multiple_of(2) {
            (field[node], field[node + 1], false)
        } else {
            (field[node], field[node + nx], true)
        };
        // One end is at most zero and the other above it.
        let t = (a / (a - b)).clamp(CROSSING_MARGIN, 1.0 - CROSSING_MARGIN);
        if north {
            (i as f64, j as f64 + t)
        } else {
            (i as f64 + t, j as f64)
        }
    };
    let next = |id: u64| {
        segments
            .binary_search_by_key(&id, |(start, _)| *start)
            .ok()
            .map(|index| (index, segments[index].1))
    };
    let mut used = vec![false; segments.len()];
    let mut rings = Vec::new();
    for start in 0..segments.len() {
        if used[start] {
            continue;
        }
        let mut ring = Ring::new();
        let mut index = start;
        loop {
            used[index] = true;
            let point = position(segments[index].0);
            if ring.last() != Some(&point) {
                ring.push(point);
            }
            match next(segments[index].1) {
                Some((following, _)) if !used[following] => index = following,
                _ => break,
            }
        }
        while ring.len() > 1 && ring.first() == ring.last() {
            ring.pop();
        }
        if ring.len() >= 3 && signed_area(&ring) != 0.0 {
            rings.push(ring);
        }
    }
    rings
}

/// Twice the ring's signed area: positive when counterclockwise.
fn signed_area(ring: &[(f64, f64)]) -> f64 {
    let mut area = 0.0;
    for (index, (x, y)) in ring.iter().enumerate() {
        let (next_x, next_y) = ring[(index + 1) % ring.len()];
        area += x * next_y - next_x * y;
    }
    area
}

/// Whether `point` is inside `ring` (even-odd rule).
fn contains(ring: &[(f64, f64)], (x, y): (f64, f64)) -> bool {
    let mut inside = false;
    let mut previous = ring[ring.len() - 1];
    for &(xi, yi) in ring {
        let (xj, yj) = previous;
        if (yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi {
            inside = !inside;
        }
        previous = (xi, yi);
    }
    inside
}

/// A point strictly inside the ring's region near its boundary: the middle
/// of its first segment of non-zero length (rings of distinct regions
/// never meet there).
fn probe(ring: &[(f64, f64)]) -> (f64, f64) {
    (0..ring.len())
        .map(|index| (ring[index], ring[(index + 1) % ring.len()]))
        .find(|(a, b)| a != b)
        .map_or(ring[0], |(a, b)| ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0))
}

/// The rings as polygons: each counterclockwise ring an outer boundary, each
/// clockwise one a hole of the smallest outer ring around it. Polygons
/// smaller than `denoise` of the largest are dropped. Polygons come
/// largest first (ties by their least vertex), holes likewise.
fn assemble(rings: Vec<Ring>, denoise: f64) -> Vec<Vec<Ring>> {
    struct Shape {
        ring: Ring,
        area: f64,
        least: (f64, f64),
        bounds: ((f64, f64), (f64, f64)),
    }
    let shape = |ring: Ring| {
        let area = signed_area(&ring);
        let least = ring
            .iter()
            .copied()
            .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)))
            .unwrap_or_default();
        let bounds = ring.iter().fold(
            (
                (f64::INFINITY, f64::INFINITY),
                (f64::NEG_INFINITY, f64::NEG_INFINITY),
            ),
            |((x0, y0), (x1, y1)), (x, y)| ((x0.min(*x), y0.min(*y)), (x1.max(*x), y1.max(*y))),
        );
        Shape {
            ring,
            area,
            least,
            bounds,
        }
    };
    let order = |a: &Shape, b: &Shape| {
        b.area
            .abs()
            .total_cmp(&a.area.abs())
            .then(a.least.0.total_cmp(&b.least.0))
            .then(a.least.1.total_cmp(&b.least.1))
    };
    let (mut outers, mut holes): (Vec<_>, Vec<_>) = rings
        .into_iter()
        .map(shape)
        .partition(|shape| shape.area > 0.0);
    outers.sort_by(order);
    holes.sort_by(order);
    let mut polygons = outers
        .iter()
        .map(|_| Vec::<Shape>::new())
        .collect::<Vec<_>>();
    for hole in holes {
        let point = probe(&hole.ring);
        let ((hx0, hy0), (hx1, hy1)) = hole.bounds;
        // Outers come largest first, so the last that holds it is the
        // smallest.
        let owner = outers.iter().rposition(|outer| {
            let ((x0, y0), (x1, y1)) = outer.bounds;
            x0 <= hx0 && y0 <= hy0 && hx1 <= x1 && hy1 <= y1 && contains(&outer.ring, point)
        });
        if let Some(owner) = owner {
            polygons[owner].push(hole);
        }
    }
    let largest = outers.first().map_or(0.0, |outer| outer.area);
    outers
        .into_iter()
        .zip(polygons)
        .filter(|(outer, _)| outer.area >= denoise * largest)
        .map(|(outer, holes)| {
            std::iter::once(outer.ring)
                .chain(holes.into_iter().map(|hole| hole.ring))
                .collect()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A field of `nx` by `ny` nodes, inside (-1) where `inside(i, j)`.
    fn field(nx: usize, ny: usize, inside: impl Fn(usize, usize) -> bool) -> Vec<f64> {
        (0..ny)
            .flat_map(|j| (0..nx).map(move |i| (i, j)))
            .map(|(i, j)| if inside(i, j) { -1.0 } else { 1.0 })
            .collect()
    }

    fn area(ring: &Ring) -> f64 {
        signed_area(ring) / 2.0
    }

    #[test]
    fn a_block_is_one_counterclockwise_ring_through_the_edge_midpoints() {
        let field = field(6, 6, |i, j| (2..=3).contains(&i) && (2..=3).contains(&j));
        let rings = rings(&field, 6, 6);
        assert_eq!(rings.len(), 1);
        // Halfway between the inside and outside nodes: a 2 × 2 square with
        // its corners cut, 3.5 square cells.
        assert_eq!(area(&rings[0]), 3.5);
        assert!(
            rings[0]
                .iter()
                .all(|(x, y)| { (1.5..=3.5).contains(x) && (1.5..=3.5).contains(y) })
        );
    }

    #[test]
    fn an_unreached_block_is_a_clockwise_hole_of_the_ring_around_it() {
        let field = field(9, 9, |i, j| {
            (1..=7).contains(&i)
                && (1..=7).contains(&j)
                && !((3..=5).contains(&i) && (3..=5).contains(&j))
        });
        let mut rings = rings(&field, 9, 9);
        assert_eq!(rings.len(), 2);
        rings.sort_by(|a, b| area(b).total_cmp(&area(a)));
        assert!(area(&rings[0]) > 0.0);
        assert!(area(&rings[1]) < 0.0);
        let polygons = assemble(rings, 0.0);
        assert_eq!(polygons.len(), 1);
        assert_eq!(polygons[0].len(), 2);
    }

    #[test]
    fn an_island_in_a_hole_is_a_polygon_of_its_own() {
        let field = field(13, 13, |i, j| {
            let ring = |lo: usize, hi: usize| (lo..=hi).contains(&i) && (lo..=hi).contains(&j);
            (ring(1, 11) && !ring(3, 9)) || ring(5, 7)
        });
        let polygons = assemble(rings(&field, 13, 13), 0.0);
        assert_eq!(polygons.len(), 2);
        // The outer ring with its hole, then the island inside the hole.
        assert_eq!(polygons[0].len(), 2);
        assert_eq!(polygons[1].len(), 1);
        assert!(contains(&polygons[0][1], polygons[1][0][0]));
        // Denoise drops the island, not the hole.
        let denoised = assemble(rings(&field, 13, 13), 0.5);
        assert_eq!(denoised.len(), 1);
        assert_eq!(denoised[0].len(), 2);
    }

    #[test]
    fn a_saddle_joins_or_parts_its_corners_by_the_cell_mean() {
        // Two diagonal inside nodes of one cell, at (2, 2) and (3, 3), less
        // deep than their neighbours are high: the cell's mean is outside.
        let mut values = field(6, 6, |i, j| (i, j) == (2, 2) || (i, j) == (3, 3));
        values[2 * 6 + 2] = -0.5;
        values[3 * 6 + 3] = -0.5;
        assert_eq!(rings(&values, 6, 6).len(), 2);
        // Deeper inside, the cell's mean is inside too: one ring.
        values[2 * 6 + 2] = -3.0;
        values[3 * 6 + 3] = -3.0;
        let joined = rings(&values, 6, 6);
        assert_eq!(joined.len(), 1);
        assert!(area(&joined[0]) > 0.0);
    }

    #[test]
    fn crossings_interpolate_the_field() {
        // One inside node at 3 × the depth of its neighbours' height: the
        // crossings sit three quarters of the way out.
        let mut values = vec![1.0; 25];
        values[2 * 5 + 2] = -3.0;
        let rings = rings(&values, 5, 5);
        assert_eq!(rings.len(), 1);
        let mut ring = rings[0].clone();
        ring.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
        assert_eq!(
            ring,
            vec![(1.25, 2.0), (2.0, 1.25), (2.0, 2.75), (2.75, 2.0)]
        );
        // Barely inside, it keeps its crossings a tenth of a cell away, so
        // they stay apart after rounding.
        values[2 * 5 + 2] = -1e-9;
        let mut ring = super::rings(&values, 5, 5)[0].clone();
        ring.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
        assert_eq!(ring, vec![(1.9, 2.0), (2.0, 1.9), (2.0, 2.1), (2.1, 2.0)]);
    }

    #[test]
    fn nothing_inside_is_no_ring() {
        assert!(rings(&[1.0; 16], 4, 4).is_empty());
        assert!(assemble(Vec::new(), 0.0).is_empty());
    }

    #[test]
    fn a_grid_beyond_the_node_cap_takes_larger_cells() {
        let mut bounds = Bounds::default();
        bounds.add(GeoPoint::new(50.0, 19.0));
        bounds.add(GeoPoint::new(50.5, 19.8));
        let fine = RouteIsochronePolygonOptions {
            resolution_m: Some(5.0),
            ..Default::default()
        };
        let grid = Grid::over(&bounds, &fine).unwrap();
        assert!(grid.nx * grid.ny <= MAX_ISOCHRONE_POLYGON_GRID_NODES);
        assert!(grid.cell > 25.0, "{}", grid.cell);
        assert_eq!(grid.spread, 1.5 * grid.cell);
        // By default, 256 cells across the larger side (56.9 km), rounded up:
        // 223 m.
        let grid = Grid::over(&bounds, &Default::default()).unwrap();
        assert_eq!(grid.cell, 223.0);
        // A small area keeps the least default cell, and nothing reached
        // has no grid.
        let mut small = Bounds::default();
        small.add(GeoPoint::new(50.0, 19.0));
        assert_eq!(Grid::over(&small, &Default::default()).unwrap().cell, 10.0);
        assert!(Grid::over(&Bounds::default(), &Default::default()).is_none());
    }

    #[test]
    fn options_are_validated() {
        let ok = |options: RouteIsochronePolygonOptions| validate(&options).is_ok();
        assert!(ok(RouteIsochronePolygonOptions::default()));
        assert!(ok(RouteIsochronePolygonOptions {
            resolution_m: Some(2.0),
            spread_m: Some(0.0),
            denoise: Some(1.0),
        }));
        for bad in [
            RouteIsochronePolygonOptions {
                resolution_m: Some(1.0),
                ..Default::default()
            },
            RouteIsochronePolygonOptions {
                resolution_m: Some(f64::NAN),
                ..Default::default()
            },
            RouteIsochronePolygonOptions {
                spread_m: Some(-1.0),
                ..Default::default()
            },
            RouteIsochronePolygonOptions {
                spread_m: Some(f64::INFINITY),
                ..Default::default()
            },
            RouteIsochronePolygonOptions {
                denoise: Some(1.5),
                ..Default::default()
            },
        ] {
            assert!(!ok(bad.clone()), "{bad:?}");
        }
    }
}

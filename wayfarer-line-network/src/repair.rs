//! Road-crossing repair: unnoded crossings of road segments found on a grid and
//! split at their intersection.

use super::*;

const TOPOLOGY_REPAIR_GRID_SIZE: i64 = 10_000;
const TOPOLOGY_REPAIR_INTERSECTION_EPSILON: f64 = 1e-9;

/// A road segment under test: 24 bytes, as there are millions. Its feature id
/// is its line's, and its ends are E7 coordinates (the node store's `i32`s, or
/// crossings inserted between them), so they fit in `i32`.
#[derive(Debug, Clone, Copy)]
struct RepairSegment {
    line_index: u32,
    segment_index: u32,
    a: [i32; 2],
    b: [i32; 2],
}

impl RepairSegment {
    fn a(&self) -> PointKey {
        from_e7(self.a)
    }

    fn b(&self) -> PointKey {
        from_e7(self.b)
    }
}

fn from_e7(point: [i32; 2]) -> PointKey {
    PointKey {
        lat: i64::from(point[0]),
        lng: i64::from(point[1]),
    }
}

/// A grid cell; cells are E7 coordinates over [`TOPOLOGY_REPAIR_GRID_SIZE`].
type Cell = (i32, i32);

pub(crate) fn repair_road_crossings(raw_lines: &mut [RawLine], stats: &mut SharedLineNetworkStats) {
    stats.topology_repair_input_road_lines += raw_lines.len();
    let line_count = u32::try_from(raw_lines.len()).expect("road line count fits in u32");
    let segments = raw_lines
        .iter()
        .enumerate()
        .flat_map(|(line_index, line)| {
            line.points
                .windows(2)
                .enumerate()
                .filter_map(move |(segment_index, window)| {
                    let a = window[0].key;
                    let b = window[1].key;
                    (a != b).then(|| RepairSegment {
                        line_index: line_index as u32,
                        segment_index: u32::try_from(segment_index)
                            .expect("a line's segment count fits in u32"),
                        a: e7(a),
                        b: e7(b),
                    })
                })
        })
        .collect::<Vec<_>>();
    if segments.len() < 2 {
        return;
    }

    // Every (cell, segment) occupancy, sorted so each cell's segments are
    // contiguous and in segment order: what a map of per-cell vectors held,
    // without an allocation per cell and per segment.
    let mut occupancy = Vec::<(Cell, u32)>::with_capacity(segments.len() + segments.len() / 2);
    for (segment_index, segment) in segments.iter().enumerate() {
        let segment_index = u32::try_from(segment_index).expect("road segment count fits in u32");
        for_each_segment_grid_cell(segment.a(), segment.b(), |cell| {
            occupancy.push((cell, segment_index));
        });
    }
    occupancy.par_sort_unstable();

    // Cells are tested in parallel. Each pair is tested in exactly one cell,
    // the counters only add up, and the insertions land in sets, so neither
    // depends on the order cells are visited in.
    let tally = occupancy
        .par_chunk_by(|left, right| left.0 == right.0)
        .fold(RepairTally::default, |mut tally, cell| {
            tally.test_cell(cell, &segments, raw_lines);
            tally
        })
        .reduce(RepairTally::default, RepairTally::merge);
    drop(segments);
    stats.topology_repair_checked_segment_pairs += tally.checked_segment_pairs;
    stats.topology_repair_skipped_same_feature += tally.skipped_same_feature;
    stats.topology_repair_skipped_incompatible_plane += tally.skipped_incompatible_plane;
    stats.topology_repair_skipped_endpoint_touches += tally.skipped_endpoint_touches;
    stats.topology_repair_repaired_crossings += tally.repaired_crossings;
    drop(occupancy);
    debug_assert!(
        tally
            .insertions
            .iter()
            .all(|(line, _, _)| *line < line_count)
    );

    let mut insertions = BTreeMap::<(usize, usize), BTreeSet<PointKey>>::new();
    for (line_index, segment_index, intersection) in tally.insertions {
        insertions
            .entry((line_index as usize, segment_index as usize))
            .or_default()
            .insert(intersection);
    }

    if insertions.is_empty() {
        return;
    }
    let mut unique_inserted_nodes = HashSet::<(PointKey, SnapCompatKey)>::new();
    for ((line_index, _), points) in &insertions {
        for point in points {
            unique_inserted_nodes.insert((*point, raw_lines[*line_index].snap_compat.clone()));
        }
    }
    stats.topology_repair_inserted_topology_nodes += unique_inserted_nodes.len();

    for (line_index, line) in raw_lines.iter_mut().enumerate() {
        if line.points.len() < 2 {
            continue;
        }
        let mut repaired = Vec::with_capacity(line.points.len());
        repaired.push(line.points[0].clone());
        for segment_index in 0..(line.points.len() - 1) {
            let start = line.points[segment_index].key;
            let end = line.points[segment_index + 1].key;
            if let Some(points) = insertions.get(&(line_index, segment_index)) {
                let mut points = points.iter().copied().collect::<Vec<_>>();
                points.sort_by(|left, right| {
                    point_position_along_segment(start, end, *left)
                        .total_cmp(&point_position_along_segment(start, end, *right))
                });
                for point in points {
                    if repaired.last().map(|last| last.key) != Some(point) {
                        repaired.push(RawPoint {
                            key: point,
                            source_node_id: None,
                        });
                    }
                }
            }
            if repaired.last().map(|last| last.key) != Some(end) {
                repaired.push(line.points[segment_index + 1].clone());
            }
        }
        stats.topology_repair_split_edges += repaired.len().saturating_sub(line.points.len());
        line.points = repaired;
    }
}

/// One cell group's share of [`repair_road_crossings`]: its counters and the
/// crossing points it found, as `(line, segment, point)`.
#[derive(Default)]
struct RepairTally {
    checked_segment_pairs: usize,
    skipped_same_feature: usize,
    skipped_incompatible_plane: usize,
    skipped_endpoint_touches: usize,
    repaired_crossings: usize,
    insertions: Vec<(u32, u32, PointKey)>,
}

impl RepairTally {
    fn test_cell(
        &mut self,
        cell_segments: &[(Cell, u32)],
        segments: &[RepairSegment],
        raw_lines: &[RawLine],
    ) {
        let cell = cell_segments[0].0;
        for left_pos in 0..cell_segments.len() {
            for right_pos in (left_pos + 1)..cell_segments.len() {
                let left_index = cell_segments[left_pos].1 as usize;
                let right_index = cell_segments[right_pos].1 as usize;
                let pair = if left_index < right_index {
                    (left_index, right_index)
                } else {
                    (right_index, left_index)
                };
                // A segment occupies every cell of its bounding box, so a pair
                // surfaces in each cell they share. Testing it only in the
                // lexicographically smallest shared cell keeps the work at one
                // test per pair without the set of every pair already seen —
                // which reached 52M entries, over a gigabyte, on a single
                // region. The shared cells are the intersection of two cell
                // rectangles, so that smallest cell is the corner below, and it
                // is always in both segments' own rectangles.
                if cell != canonical_pair_cell(&segments[pair.0], &segments[pair.1]) {
                    continue;
                }
                self.checked_segment_pairs += 1;

                let left = segments[pair.0];
                let right = segments[pair.1];
                let left_line = &raw_lines[left.line_index as usize];
                let right_line = &raw_lines[right.line_index as usize];
                let (left_a, left_b, right_a, right_b) = (left.a(), left.b(), right.a(), right.b());
                if left_line.feature_id == right_line.feature_id
                    || left.line_index == right.line_index
                {
                    if proper_segment_intersection(left_a, left_b, right_a, right_b).is_some() {
                        self.skipped_same_feature += 1;
                    }
                    continue;
                }
                if left_line.snap_compat != right_line.snap_compat
                    || left_line.snap_compat.brunnel == BrunnelKind::Ferry
                {
                    if proper_segment_intersection(left_a, left_b, right_a, right_b).is_some() {
                        self.skipped_incompatible_plane += 1;
                    }
                    continue;
                }
                let Some(intersection) =
                    proper_segment_intersection(left_a, left_b, right_a, right_b)
                else {
                    if segment_endpoint_touch(left_a, left_b, right_a, right_b) {
                        self.skipped_endpoint_touches += 1;
                    }
                    continue;
                };
                self.insertions
                    .push((left.line_index, left.segment_index, intersection));
                self.insertions
                    .push((right.line_index, right.segment_index, intersection));
                self.repaired_crossings += 1;
            }
        }
    }

    fn merge(mut self, mut other: Self) -> Self {
        self.checked_segment_pairs += other.checked_segment_pairs;
        self.skipped_same_feature += other.skipped_same_feature;
        self.skipped_incompatible_plane += other.skipped_incompatible_plane;
        self.skipped_endpoint_touches += other.skipped_endpoint_touches;
        self.repaired_crossings += other.repaired_crossings;
        self.insertions.append(&mut other.insertions);
        self
    }
}

/// The one cell in which a pair of segments is allowed to be tested: the
/// smallest cell shared by both bounding boxes.
fn canonical_pair_cell(left: &RepairSegment, right: &RepairSegment) -> Cell {
    let (left_lat, left_lng) = segment_min_cell(left.a(), left.b());
    let (right_lat, right_lng) = segment_min_cell(right.a(), right.b());
    (left_lat.max(right_lat), left_lng.max(right_lng))
}

/// Lowest cell of a segment's bounding box, i.e. the first cell
/// [`for_each_segment_grid_cell`] visits.
fn segment_min_cell(a: PointKey, b: PointKey) -> Cell {
    (grid_cell(a.lat.min(b.lat)), grid_cell(a.lng.min(b.lng)))
}

/// Every grid cell of the segment's bounding box, lowest first.
fn for_each_segment_grid_cell(a: PointKey, b: PointKey, mut visit: impl FnMut(Cell)) {
    let min_lat = a.lat.min(b.lat);
    let max_lat = a.lat.max(b.lat);
    let min_lng = a.lng.min(b.lng);
    let max_lng = a.lng.max(b.lng);
    for lat_cell in grid_cell(min_lat)..=grid_cell(max_lat) {
        for lng_cell in grid_cell(min_lng)..=grid_cell(max_lng) {
            visit((lat_cell, lng_cell));
        }
    }
}

/// An E7 coordinate's cell; it fits `i32` as the coordinate does.
fn grid_cell(value: i64) -> i32 {
    value.div_euclid(TOPOLOGY_REPAIR_GRID_SIZE) as i32
}

fn segment_bboxes_overlap(a1: PointKey, a2: PointKey, b1: PointKey, b2: PointKey) -> bool {
    a1.lat.min(a2.lat) <= b1.lat.max(b2.lat)
        && b1.lat.min(b2.lat) <= a1.lat.max(a2.lat)
        && a1.lng.min(a2.lng) <= b1.lng.max(b2.lng)
        && b1.lng.min(b2.lng) <= a1.lng.max(a2.lng)
}

fn segment_endpoint_touch(a1: PointKey, a2: PointKey, b1: PointKey, b2: PointKey) -> bool {
    a1 == b1 || a1 == b2 || a2 == b1 || a2 == b2
}

fn proper_segment_intersection(
    a1: PointKey,
    a2: PointKey,
    b1: PointKey,
    b2: PointKey,
) -> Option<PointKey> {
    if !segment_bboxes_overlap(a1, a2, b1, b2) {
        return None;
    }
    let ax = a1.lng as f64;
    let ay = a1.lat as f64;
    let bx = a2.lng as f64;
    let by = a2.lat as f64;
    let cx = b1.lng as f64;
    let cy = b1.lat as f64;
    let dx = b2.lng as f64;
    let dy = b2.lat as f64;
    let abx = bx - ax;
    let aby = by - ay;
    let cdx = dx - cx;
    let cdy = dy - cy;
    let denom = cross(abx, aby, cdx, cdy);
    if denom.abs() < f64::EPSILON {
        return None;
    }
    let acx = cx - ax;
    let acy = cy - ay;
    let t = cross(acx, acy, cdx, cdy) / denom;
    let u = cross(acx, acy, abx, aby) / denom;
    if t <= TOPOLOGY_REPAIR_INTERSECTION_EPSILON
        || t >= 1.0 - TOPOLOGY_REPAIR_INTERSECTION_EPSILON
        || u <= TOPOLOGY_REPAIR_INTERSECTION_EPSILON
        || u >= 1.0 - TOPOLOGY_REPAIR_INTERSECTION_EPSILON
    {
        return None;
    }
    let lat = (ay + t * aby).round() as i64;
    let lng = (ax + t * abx).round() as i64;
    let intersection = PointKey { lat, lng };
    if intersection == a1 || intersection == a2 || intersection == b1 || intersection == b2 {
        return None;
    }
    Some(intersection)
}

fn cross(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    ax * by - ay * bx
}

fn point_position_along_segment(start: PointKey, end: PointKey, point: PointKey) -> f64 {
    let dx = (end.lng - start.lng) as f64;
    let dy = (end.lat - start.lat) as f64;
    if dx.abs() >= dy.abs() {
        if dx == 0.0 {
            0.0
        } else {
            (point.lng - start.lng) as f64 / dx
        }
    } else if dy == 0.0 {
        0.0
    } else {
        (point.lat - start.lat) as f64 / dy
    }
}

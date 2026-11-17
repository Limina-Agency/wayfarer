//! Synthesized edges: pedestrian plazas (interior anchor stars).

use super::*;

/// Base for synthetic plaza `osm_way_id`s. OSM object ids are non-negative, so
/// the negative range is collision-free; each synthesized plaza claims one id by
/// decrementing from this base. Synthetic ids never match a `source_ways` row, so
/// the way-fact refresh leaves their foot-only masks untouched.
pub(crate) const SYNTHETIC_PLAZA_WAY_ID_BASE: i64 = -9_000_000_000_000;

/// Minimum plaza polygon area (m²) eligible for interior anchor-star edges.
/// Filters out incidental paved patches while admitting real squares.
const MIN_PLAZA_AREA_M2: f64 = 100.0;

/// Minimum perimeter anchors required before an interior star is worthwhile.
const MIN_PLAZA_ANCHORS: usize = 2;

pub(crate) struct PlazaAccumulator {
    pub(crate) highway: Option<String>,
    pub(crate) surface: Option<Arc<str>>,
    pub(crate) allow_bicycle: bool,
    pub(crate) segments: Vec<Vec<(f64, f64)>>,
}

/// A closed way is a routable pedestrian plaza when it is tagged as a filled
/// area over a pedestrian surface. Mirrors the render-side area detection: either
/// `area=yes` (or truthy) on a `highway=pedestrian`/`footway` way, or a bare
/// `area:highway=pedestrian` polygon.
pub(crate) fn is_pedestrian_area(way: &RoutableWay) -> bool {
    let area_truthy = matches!(
        way.area.as_deref().map(str::trim),
        Some("yes" | "true" | "1")
    );
    let highway_is_pedestrian = matches!(way.highway.as_deref(), Some("pedestrian" | "footway"));
    let area_highway_is_pedestrian = matches!(
        way.area_highway.as_deref().map(str::trim),
        Some("pedestrian")
    );
    (area_truthy && highway_is_pedestrian) || area_highway_is_pedestrian
}

pub(crate) fn plaza_allows_bicycle(way: &RoutableWay) -> bool {
    matches!(
        way.bicycle.as_deref().map(str::trim),
        Some("yes" | "designated" | "permissive" | "official")
    )
}

/// Reassemble an ordered, closed polygon ring from one or more polylines that
/// share endpoints. Returns `None` when the segments cannot be chained into a
/// single closed loop (degenerate or open plaza mapping).
fn reassemble_ring(segments: &[Vec<(f64, f64)>]) -> Option<Vec<(f64, f64)>> {
    let mut remaining: Vec<&Vec<(f64, f64)>> = segments
        .iter()
        .filter(|segment| segment.len() >= 2)
        .collect();
    if remaining.is_empty() {
        return None;
    }

    let mut ring = remaining.swap_remove(0).clone();
    while !remaining.is_empty() {
        let tail = *ring.last()?;
        let tail_key = (to_fixed(tail.0), to_fixed(tail.1));
        let mut matched = None;
        for (index, segment) in remaining.iter().enumerate() {
            let start = *segment.first()?;
            let end = *segment.last()?;
            if (to_fixed(start.0), to_fixed(start.1)) == tail_key {
                matched = Some((index, false));
                break;
            }
            if (to_fixed(end.0), to_fixed(end.1)) == tail_key {
                matched = Some((index, true));
                break;
            }
        }
        let (index, reversed) = matched?;
        let segment = remaining.swap_remove(index);
        if reversed {
            ring.extend(segment.iter().rev().skip(1).copied());
        } else {
            ring.extend(segment.iter().skip(1).copied());
        }
    }

    let first = *ring.first()?;
    let last = *ring.last()?;
    if (to_fixed(first.0), to_fixed(first.1)) != (to_fixed(last.0), to_fixed(last.1)) {
        return None;
    }
    // Deduplicated distinct vertices (dropping the closing repeat) must form a
    // polygon.
    if ring.len() < 4 {
        return None;
    }
    Some(ring)
}

/// Locally-projected (equirectangular) polygon area in square metres. Adequate
/// for the plaza-size threshold at city scale.
fn ring_area_m2(ring: &[(f64, f64)]) -> f64 {
    if ring.len() < 4 {
        return 0.0;
    }
    let lat0 = ring.iter().map(|(lat, _)| *lat).sum::<f64>() / ring.len() as f64;
    let m_per_deg_lat = 111_320.0;
    let m_per_deg_lng = 111_320.0 * math::cos(lat0.to_radians());
    let mut area = 0.0;
    for window in ring.windows(2) {
        let (lat_a, lng_a) = window[0];
        let (lat_b, lng_b) = window[1];
        let xa = lng_a * m_per_deg_lng;
        let ya = lat_a * m_per_deg_lat;
        let xb = lng_b * m_per_deg_lng;
        let yb = lat_b * m_per_deg_lat;
        area += xa * yb - xb * ya;
    }
    area.abs() / 2.0
}

/// Representative interior point of the ring: the centroid when it lies inside
/// (natural centre for convex plazas), otherwise a guaranteed-interior label
/// point that copes with concave rings.
fn plaza_interior_point(ring: &[(f64, f64)]) -> Option<(f64, f64)> {
    let polygon = geo::Polygon::new(
        LineString::from(
            ring.iter()
                .map(|(lat, lng)| geo::Coord { x: *lng, y: *lat })
                .collect::<Vec<_>>(),
        ),
        Vec::new(),
    );
    if let Some(centroid) = polygon.centroid()
        && polygon.contains(&centroid)
    {
        return Some((centroid.y(), centroid.x()));
    }
    polygon.interior_point().map(|point| (point.y(), point.x()))
}

/// Synthesize anchor-star interior edges for pedestrian plaza polygons.
///
/// The plaza perimeter ring stays in the graph exactly as ingested; this adds
/// foot-only spokes from each real entry/exit anchor to a virtual centre node so
/// pedestrian routes can cross the open space instead of circling it.
#[allow(clippy::too_many_arguments)]
pub(crate) fn synthesize_pedestrian_plazas(
    graph: &mut RoadGraph,
    node_map: &FxHashMap<(i32, i32), NodeIndex>,
    plaza_accumulators: HashMap<i64, PlazaAccumulator>,
    next_edge_id: &mut u64,
    next_shared_edge_info_id: &mut u64,
    next_synthetic_plaza_way_id: &mut i64,
    report: &mut RoadGraphBuildReport,
) {
    if plaza_accumulators.is_empty() {
        return;
    }

    // The ways at every node a plaza ring can reach (its points come from the
    // plaza's segments), so a ring node can be recognised as an anchor when a
    // *different* way also touches it.
    let mut is_ring_node = vec![false; graph.node_count()];
    for &(lat, lng) in plaza_accumulators
        .values()
        .flat_map(|accumulator| accumulator.segments.iter().flatten())
    {
        if let Some(node_index) = node_map.get(&(to_fixed(lat), to_fixed(lng))) {
            is_ring_node[node_index.index()] = true;
        }
    }
    let info_index_by_id = SharedInfoIndex::new(&graph.shared_edge_infos);
    let mut ways_at_node: HashMap<NodeIndex, BTreeSet<i64>> = HashMap::new();
    for edge_ref in graph.graph.edge_references() {
        let (source, target) = (edge_ref.source(), edge_ref.target());
        if !is_ring_node[source.index()] && !is_ring_node[target.index()] {
            continue;
        }
        if let Some(info_index) = info_index_by_id.get(edge_ref.weight().shared_edge_info_id) {
            let way_id = graph.shared_edge_infos.at(info_index).osm_way_id;
            for node in [source, target] {
                if is_ring_node[node.index()] {
                    ways_at_node.entry(node).or_default().insert(way_id);
                }
            }
        }
    }
    drop(is_ring_node);

    let mut plazas: Vec<(i64, PlazaAccumulator)> = plaza_accumulators.into_iter().collect();
    plazas.sort_by_key(|(way_id, _)| *way_id);

    for (plaza_way_id, accumulator) in plazas {
        let Some(ring) = reassemble_ring(&accumulator.segments) else {
            report.plaza_skipped_open_ring += 1;
            continue;
        };
        if ring_area_m2(&ring) < MIN_PLAZA_AREA_M2 {
            report.plaza_skipped_below_min_area += 1;
            continue;
        }

        // Anchors are distinct ring nodes shared with another ingested way.
        let mut anchors: Vec<NodeIndex> = Vec::new();
        let mut seen: BTreeSet<(i32, i32)> = BTreeSet::new();
        for &(lat, lng) in &ring {
            let key = (to_fixed(lat), to_fixed(lng));
            if !seen.insert(key) {
                continue;
            }
            let Some(&node_index) = node_map.get(&key) else {
                continue;
            };
            let shared_with_other = ways_at_node
                .get(&node_index)
                .map(|ways| ways.iter().any(|&way_id| way_id != plaza_way_id))
                .unwrap_or(false);
            if shared_with_other {
                anchors.push(node_index);
            }
        }

        if anchors.len() < MIN_PLAZA_ANCHORS {
            report.plaza_skipped_insufficient_anchors += 1;
            continue;
        }

        let Some(center) = plaza_interior_point(&ring) else {
            report.plaza_skipped_open_ring += 1;
            continue;
        };

        let synthetic_way_id = *next_synthetic_plaza_way_id;
        *next_synthetic_plaza_way_id -= 1;

        let center_node = graph.add_node(RoadNode {
            id: graph.graph.node_count() as i64,
            lat: center.0,
            lng: center.1,
            source_node_id: None,
            ..Default::default()
        });

        let forward_mask = if accumulator.allow_bicycle {
            OsmMode::FOOT | OsmMode::BICYCLE
        } else {
            OsmMode::FOOT
        };
        let mobility_mask = mobility_mode_mask_from_osm(forward_mask);
        let highway = accumulator
            .highway
            .clone()
            .unwrap_or_else(|| "pedestrian".to_string());

        let mut plaza_edges = 0usize;
        for anchor in anchors.iter().copied() {
            let anchor_point = {
                let node = &graph.graph[anchor];
                (node.lat, node.lng)
            };
            let distance = calculate_path_distance(&[anchor_point, center]);
            if distance <= 0.0 {
                continue;
            }
            let shared_edge_info_id = *next_shared_edge_info_id;
            *next_shared_edge_info_id += 1;
            graph.shared_edge_infos.push(SharedEdgeInfo {
                shared_edge_info_id,
                osm_way_id: synthetic_way_id,
                road_type: RoadType::Path,
                geometry: vec![anchor_point, center],
                directional_access: DirectionalAccess::Both,
                highway: Some(Arc::from(highway.as_str())),
                surface: accumulator.surface.clone(),
                osm_forward_mode_mask: forward_mask,
                osm_reverse_mode_mask: forward_mask,
                hierarchy_tier: Some(RoadHierarchyTier::Local),
                ..Default::default()
            });

            let base_cost = distance as f32 * RoadType::Path.cost_factor();
            for (source, target, traversal_direction) in [
                (anchor, center_node, TraversalDirection::Forward),
                (center_node, anchor, TraversalDirection::Backward),
            ] {
                let edge_id = *next_edge_id;
                *next_edge_id += 1;
                graph.add_edge(
                    source,
                    target,
                    RoadEdge {
                        edge_id,
                        shared_edge_info_id,
                        distance_m: distance as f32,
                        base_cost,
                        mobility_mode_mask: mobility_mask,
                        osm_forward_mode_mask: forward_mask,
                        osm_reverse_mode_mask: forward_mask,
                        hierarchy_tier: Some(RoadHierarchyTier::Local),
                        traversal_direction,
                        source_lineage_ids: vec![synthetic_way_id],
                        ..Default::default()
                    },
                );
                report.created_directed_edge_count += 1;
                report.record_access_mask(forward_mask, mobility_mask);
                plaza_edges += 1;
            }
        }

        if plaza_edges == 0 {
            report.plaza_skipped_insufficient_anchors += 1;
            continue;
        }

        report.synthesized_plaza_count += 1;
        report.plaza_anchor_count += anchors.len();
        report.plaza_interior_edge_count += plaza_edges;
    }
}

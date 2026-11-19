//! Render products: topology edges merged into render lines per layer and road
//! tier, with their zoom ranges.

use super::*;

const STRAIGHT_MERGE_COS_THRESHOLD: f64 = -0.5;

#[derive(Debug, Clone)]
pub(crate) struct RenderSeed {
    pub(crate) feature_id: i64,
    pub(crate) layer: SharedLineLayer,
    pub(crate) points: Vec<PointKey>,
    /// Shared with every other seed carrying the same tags.
    ///
    /// One seed exists per topology edge — over a million for a region's roads —
    /// and each used to own a private clone of its edge's tags and of the
    /// derived merge signature. The signature in particular is at most five
    /// pairs drawn from a five-key vocabulary, so the same handful of values
    /// were being cloned a million times. Merging copies out of the handle once
    /// per emitted line instead.
    pub(crate) attrs: Arc<[(String, String)]>,
    pub(crate) merge_signature: Arc<[(String, String)]>,
    pub(crate) min_zoom: u8,
    pub(crate) max_zoom: u8,
    pub(crate) sort_order: u8,
    pub(crate) start_node: u32,
    pub(crate) end_node: u32,
    pub(crate) source_lineage_ids: Vec<i64>,
    pub(crate) topology_edge_ids: Vec<u32>,
    pub(crate) directional_access: DirectionalAccessModel,
}

#[derive(Debug, Clone)]
struct RenderPair {
    partner: usize,
    next_node: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RoadRenderTier {
    Major,
    Secondary,
    Local,
    Internal,
    PathDetail,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RenderSeedPolicy {
    Generic(SharedLineLayer),
    RoadGeometry(RoadRenderTier),
}

pub(crate) fn build_render_products_for_layer(
    topology_edges: &[SharedTopologyEdge],
    stats: &mut SharedLineNetworkStats,
) -> Vec<SharedRenderLine> {
    let mut attrs = AttrListInterner::default();
    let seeds = topology_edges
        .iter()
        .filter_map(|edge| {
            let signature = attrs.intern(merge_signature(edge.layer, &edge.attrs));
            render_seed_from_topology_edge(edge, signature)
        })
        .collect::<Vec<_>>();
    build_render_lines_from_seeds(
        seeds,
        stats,
        &mut attrs,
        RenderSeedPolicy::Generic(topology_edges[0].layer),
    )
}

pub(crate) const ROAD_RENDER_TIERS: [RoadRenderTier; 5] = [
    RoadRenderTier::Major,
    RoadRenderTier::Secondary,
    RoadRenderTier::Local,
    RoadRenderTier::Internal,
    RoadRenderTier::PathDetail,
];

pub(crate) fn build_road_render_products(
    topology_edges: &[SharedTopologyEdge],
    stats: &mut SharedLineNetworkStats,
) -> Vec<SharedRenderLine> {
    let mut render_lines = Vec::new();
    let mut attrs = AttrListInterner::default();

    // Bucket once. An edge's tier is a pure function of its attrs, and this used
    // to be recomputed by scanning *every* topology edge once per tier for
    // geometry and again for labels — nine full passes over the whole road
    // network. Pushing in edge order keeps each bucket in the order the
    // per-tier scans produced.
    let mut edges_by_tier: [Vec<&SharedTopologyEdge>; ROAD_RENDER_TIERS.len()] =
        std::array::from_fn(|_| Vec::new());
    for edge in topology_edges {
        if let Some(tier) = road_render_tier(&edge.attrs) {
            edges_by_tier[tier as usize].push(edge);
        }
    }

    for (tier_index, tier) in ROAD_RENDER_TIERS.into_iter().enumerate() {
        let tier_edges = &edges_by_tier[tier_index];
        if tier_edges.is_empty() {
            continue;
        }
        let geometry_seeds = tier_edges
            .iter()
            .filter_map(|edge| {
                let signature = attrs.intern(road_geometry_merge_signature(&edge.attrs));
                render_seed_from_topology_edge(edge, signature)
            })
            .collect::<Vec<_>>();
        let geometry_seeds = filter_render_seeds(
            geometry_seeds,
            stats,
            RenderSeedPolicy::RoadGeometry(tier),
            &mut attrs,
        );
        render_lines.extend(build_render_lines_from_filtered_seeds(
            geometry_seeds,
            stats,
        ));
    }

    render_lines
}

fn build_render_lines_from_seeds(
    seeds: Vec<RenderSeed>,
    stats: &mut SharedLineNetworkStats,
    attrs: &mut AttrListInterner,
    policy: RenderSeedPolicy,
) -> Vec<SharedRenderLine> {
    let seeds = filter_render_seeds(seeds, stats, policy, attrs);
    build_render_lines_from_filtered_seeds(seeds, stats)
}

fn filter_render_seeds(
    seeds: Vec<RenderSeed>,
    stats: &mut SharedLineNetworkStats,
    policy: RenderSeedPolicy,
    attrs: &mut AttrListInterner,
) -> Vec<RenderSeed> {
    if seeds.is_empty() {
        return seeds;
    }

    let deduped = dedupe_render_seeds(seeds, stats, attrs);
    let seed_adjacency = build_seed_adjacency(&deduped);
    deduped
        .into_iter()
        .filter_map(|mut seed| {
            if should_drop_render_loop_for_policy(&seed, policy) {
                stats.render_loops_removed += 1;
                return None;
            }
            let adjusted = adjusted_render_min_zoom(&seed, &seed_adjacency, policy);
            if adjusted != seed.min_zoom {
                stats.render_lines_min_zoom_bumped += 1;
                stats.render_stub_edges_bumped += 1;
                seed.min_zoom = adjusted;
            }
            (seed.min_zoom <= seed.max_zoom).then_some(seed)
        })
        .collect()
}

fn build_render_lines_from_filtered_seeds(
    seeds: Vec<RenderSeed>,
    stats: &mut SharedLineNetworkStats,
) -> Vec<SharedRenderLine> {
    if seeds.is_empty() {
        return Vec::new();
    }

    let adjacency = build_seed_adjacency(&seeds);
    let pairings = build_merge_pairings(&seeds, &adjacency, stats);
    merge_render_seeds(&seeds, &pairings)
}

fn render_seed_from_topology_edge(
    edge: &SharedTopologyEdge,
    merge_signature: Arc<[(String, String)]>,
) -> Option<RenderSeed> {
    let points = simplify_anchor_points(
        edge.geometry
            .iter()
            .map(|(lat, lng)| PointKey {
                lat: to_fixed(*lat),
                lng: to_fixed(*lng),
            })
            .collect(),
    );
    if points.len() < 2 || !points.windows(2).any(|pair| pair[0] != pair[1]) {
        return None;
    }
    Some(RenderSeed {
        feature_id: edge.feature_id,
        layer: edge.layer,
        points,
        // The edge already holds a shared handle, so the seed takes a reference
        // to the same list rather than interning a copy.
        attrs: Arc::clone(&edge.attrs),
        merge_signature,
        min_zoom: edge.min_zoom,
        max_zoom: edge.max_zoom,
        sort_order: edge.sort_order,
        start_node: edge.start_node,
        end_node: edge.end_node,
        source_lineage_ids: edge.source_lineage_ids.to_vec(),
        topology_edge_ids: vec![edge.edge_id],
        directional_access: edge
            .routing
            .as_ref()
            .map(|routing| routing.directional_access)
            .unwrap_or_default(),
    })
}

/// Callers reach this through a tier bucket, so the tier gate that used to live
/// here has already been applied.
fn dedupe_render_seeds(
    seeds: Vec<RenderSeed>,
    stats: &mut SharedLineNetworkStats,
    attrs: &mut AttrListInterner,
) -> Vec<RenderSeed> {
    let mut deduped = Vec::<RenderSeed>::new();
    // The signature is a shared handle, so keying by it clones a pointer rather
    // than the tag list.
    let mut seen = BTreeMap::<
        (
            SharedLineLayer,
            Arc<[(String, String)]>,
            u8,
            u8,
            u8,
            u32,
            u32,
            Vec<PointKey>,
        ),
        usize,
    >::new();

    for seed in seeds {
        let canonical_points = canonicalize_points(&seed.points);
        let canonical_nodes = canonicalize_nodes(seed.start_node, seed.end_node);
        let key = (
            seed.layer,
            seed.merge_signature.clone(),
            seed.min_zoom,
            seed.max_zoom,
            seed.sort_order,
            canonical_nodes.0,
            canonical_nodes.1,
            canonical_points,
        );
        if let Some(existing_index) = seen.get(&key).copied() {
            stats.render_duplicate_edges_removed += 1;
            let existing = &mut deduped[existing_index];
            merge_unique_into(&mut existing.source_lineage_ids, seed.source_lineage_ids);
            merge_unique_into(&mut existing.topology_edge_ids, seed.topology_edge_ids);
            existing.attrs = attrs.intern(common_attrs(&existing.attrs, &seed.attrs));
        } else {
            seen.insert(key, deduped.len());
            deduped.push(seed);
        }
    }

    deduped
}

fn build_seed_adjacency(seeds: &[RenderSeed]) -> HashMap<u32, Vec<usize>> {
    let mut adjacency = HashMap::<u32, Vec<usize>>::new();
    for (index, seed) in seeds.iter().enumerate() {
        adjacency.entry(seed.start_node).or_default().push(index);
        adjacency.entry(seed.end_node).or_default().push(index);
    }
    adjacency
}

fn build_merge_pairings(
    seeds: &[RenderSeed],
    adjacency: &HashMap<u32, Vec<usize>>,
    stats: &mut SharedLineNetworkStats,
) -> HashMap<(usize, u32), RenderPair> {
    let mut pairings = HashMap::<(usize, u32), RenderPair>::new();
    for (&node_id, incident) in adjacency {
        let mut candidates = Vec::<(f64, usize, usize)>::new();
        for left_offset in 0..incident.len() {
            let left_index = incident[left_offset];
            for right_offset in (left_offset + 1)..incident.len() {
                let right_index = incident[right_offset];
                if !merge_compatible_at_node(&seeds[left_index], &seeds[right_index], node_id) {
                    continue;
                }
                let Some(score) =
                    merge_straightness_score(&seeds[left_index], &seeds[right_index], node_id)
                else {
                    continue;
                };
                if incident.len() > 2 && score > STRAIGHT_MERGE_COS_THRESHOLD {
                    continue;
                }
                candidates.push((score, left_index, right_index));
            }
        }

        candidates.sort_by(|left, right| {
            left.0
                .partial_cmp(&right.0)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut used = HashSet::<usize>::new();
        for (_, left_index, right_index) in candidates {
            if used.contains(&left_index) || used.contains(&right_index) {
                continue;
            }
            let left_other_node = other_node(&seeds[left_index], node_id);
            let right_other_node = other_node(&seeds[right_index], node_id);
            if let (Some(left_other_node), Some(right_other_node)) =
                (left_other_node, right_other_node)
            {
                pairings.insert(
                    (left_index, node_id),
                    RenderPair {
                        partner: right_index,
                        next_node: right_other_node,
                    },
                );
                pairings.insert(
                    (right_index, node_id),
                    RenderPair {
                        partner: left_index,
                        next_node: left_other_node,
                    },
                );
                used.insert(left_index);
                used.insert(right_index);
                stats.render_stroke_pairs_formed += 1;
            }
        }
    }

    pairings
}

fn merge_render_seeds(
    seeds: &[RenderSeed],
    pairings: &HashMap<(usize, u32), RenderPair>,
) -> Vec<SharedRenderLine> {
    let mut visited = vec![false; seeds.len()];
    let mut merged = Vec::new();

    for seed_index in 0..seeds.len() {
        if visited[seed_index] {
            continue;
        }

        let seed = &seeds[seed_index];
        let mut local_seen = HashSet::from([seed_index]);
        let mut points = seed.points.clone();
        let mut start_node = seed.start_node;
        let mut end_node = seed.end_node;
        // Copied out of the shared handle once per emitted line, rather than
        // once per seed, because chaining mutates it.
        let mut attrs = seed.attrs.to_vec();
        let mut source_lineage_ids = seed.source_lineage_ids.clone();
        let mut topology_edge_ids = seed.topology_edge_ids.clone();

        extend_render_chain(
            seeds,
            pairings,
            &mut local_seen,
            seed_index,
            &mut points,
            &mut start_node,
            &mut attrs,
            &mut source_lineage_ids,
            &mut topology_edge_ids,
            true,
        );
        extend_render_chain(
            seeds,
            pairings,
            &mut local_seen,
            seed_index,
            &mut points,
            &mut end_node,
            &mut attrs,
            &mut source_lineage_ids,
            &mut topology_edge_ids,
            false,
        );

        for index in &local_seen {
            visited[*index] = true;
        }

        source_lineage_ids.sort_unstable();
        source_lineage_ids.dedup();
        topology_edge_ids.sort_unstable();
        topology_edge_ids.dedup();

        merged.push(SharedRenderLine {
            feature_id: *source_lineage_ids.first().unwrap_or(&seed.feature_id),
            layer: seed.layer,
            geometry: points
                .iter()
                .map(|point| (from_fixed(point.lat), from_fixed(point.lng)))
                .collect(),
            attrs,
            min_zoom: seed.min_zoom,
            max_zoom: seed.max_zoom,
            sort_order: seed.sort_order,
            start_node,
            end_node,
            source_lineage_ids,
            topology_edge_ids,
        });
    }

    merged
}

fn extend_render_chain(
    seeds: &[RenderSeed],
    pairings: &HashMap<(usize, u32), RenderPair>,
    local_seen: &mut HashSet<usize>,
    seed_index: usize,
    points: &mut Vec<PointKey>,
    boundary_node: &mut u32,
    attrs: &mut Vec<(String, String)>,
    source_lineage_ids: &mut Vec<i64>,
    topology_edge_ids: &mut Vec<u32>,
    prepend: bool,
) {
    let mut current_index = seed_index;
    let mut current_node = *boundary_node;

    while let Some(pair) = pairings.get(&(current_index, current_node)) {
        if local_seen.contains(&pair.partner) {
            break;
        }
        let next_seed = &seeds[pair.partner];
        let Some(next_points) = oriented_points_for_seed(next_seed, current_node, prepend) else {
            break;
        };

        if prepend {
            let mut merged = next_points;
            merged.pop();
            merged.extend(points.iter().copied());
            *points = merged;
        } else {
            let mut tail = next_points;
            if !tail.is_empty() {
                tail.remove(0);
            }
            points.extend(tail);
        }

        *attrs = common_attrs(attrs, &next_seed.attrs);
        merge_unique_into(source_lineage_ids, next_seed.source_lineage_ids.clone());
        merge_unique_into(topology_edge_ids, next_seed.topology_edge_ids.clone());
        local_seen.insert(pair.partner);
        current_index = pair.partner;
        current_node = pair.next_node;
        *boundary_node = pair.next_node;
    }
}

fn oriented_points_for_seed(
    seed: &RenderSeed,
    boundary_node: u32,
    prepend: bool,
) -> Option<Vec<PointKey>> {
    if seed.start_node == boundary_node {
        if prepend {
            let mut reversed = seed.points.clone();
            reversed.reverse();
            Some(reversed)
        } else {
            Some(seed.points.clone())
        }
    } else if seed.end_node == boundary_node {
        if prepend {
            Some(seed.points.clone())
        } else {
            let mut reversed = seed.points.clone();
            reversed.reverse();
            Some(reversed)
        }
    } else {
        None
    }
}

fn merge_compatible_at_node(left: &RenderSeed, right: &RenderSeed, node_id: u32) -> bool {
    left.layer == right.layer
        && left.merge_signature == right.merge_signature
        && left.min_zoom == right.min_zoom
        && left.max_zoom == right.max_zoom
        && left.sort_order == right.sort_order
        && directional_access_is_continuous(left, right, node_id)
}

fn directional_access_is_continuous(left: &RenderSeed, right: &RenderSeed, node_id: u32) -> bool {
    use DirectionalAccessModel::{Backward, Both, Closed, Forward};

    match (left.directional_access, right.directional_access) {
        (Both, Both) => true,
        (Closed, Closed) => false,
        (Forward | Backward, Forward | Backward) => {
            directional_flow_at_node(left, node_id) != directional_flow_at_node(right, node_id)
        }
        _ => false,
    }
}

fn directional_flow_at_node(seed: &RenderSeed, node_id: u32) -> Option<bool> {
    use DirectionalAccessModel::{Backward, Forward};

    match (seed.directional_access, node_id) {
        (Forward, node) if node == seed.start_node => Some(true),
        (Forward, node) if node == seed.end_node => Some(false),
        (Backward, node) if node == seed.start_node => Some(false),
        (Backward, node) if node == seed.end_node => Some(true),
        _ => None,
    }
}

pub(crate) fn merge_straightness_score(
    left: &RenderSeed,
    right: &RenderSeed,
    node_id: u32,
) -> Option<f64> {
    let left_vector = outgoing_vector(left, node_id)?;
    let right_vector = outgoing_vector(right, node_id)?;
    let left_length = (left_vector.0 * left_vector.0 + left_vector.1 * left_vector.1).sqrt();
    let right_length = (right_vector.0 * right_vector.0 + right_vector.1 * right_vector.1).sqrt();
    if left_length == 0.0 || right_length == 0.0 {
        return None;
    }
    Some(
        (left_vector.0 * right_vector.0 + left_vector.1 * right_vector.1)
            / (left_length * right_length),
    )
}

fn outgoing_vector(seed: &RenderSeed, node_id: u32) -> Option<(f64, f64)> {
    if seed.start_node == node_id {
        seed.points
            .windows(2)
            .find(|window| window[0] != window[1])
            .map(|window| mercator_vector(window[0], window[1]))
    } else if seed.end_node == node_id {
        seed.points
            .windows(2)
            .rev()
            .find(|window| window[0] != window[1])
            .map(|window| mercator_vector(window[1], window[0]))
    } else {
        None
    }
}

fn other_node(seed: &RenderSeed, node_id: u32) -> Option<u32> {
    if seed.start_node == node_id {
        Some(seed.end_node)
    } else if seed.end_node == node_id {
        Some(seed.start_node)
    } else {
        None
    }
}

fn adjusted_render_min_zoom(
    seed: &RenderSeed,
    adjacency: &HashMap<u32, Vec<usize>>,
    policy: RenderSeedPolicy,
) -> u8 {
    match policy {
        RenderSeedPolicy::Generic(_) => bumped_render_min_zoom(seed, adjacency),
        RenderSeedPolicy::RoadGeometry(tier) => road_geometry_min_zoom(seed, tier),
    }
}

fn bumped_render_min_zoom(seed: &RenderSeed, adjacency: &HashMap<u32, Vec<usize>>) -> u8 {
    let class = attr_value(&seed.attrs, "class");
    let length_m = line_length_m(&seed.points);
    let start_degree = adjacency.get(&seed.start_node).map_or(0usize, Vec::len);
    let end_degree = adjacency.get(&seed.end_node).map_or(0usize, Vec::len);
    let dangling = start_degree <= 1 || end_degree <= 1;

    match seed.layer {
        SharedLineLayer::Roads => {
            if !dangling {
                return seed.min_zoom;
            }
            match class {
                Some("service") if length_m < 160.0 => seed.min_zoom.max(14),
                Some(
                    "track" | "path" | "footway" | "cycleway" | "bridleway" | "pedestrian"
                    | "steps",
                ) if length_m < 260.0 => seed.min_zoom.max(14),
                Some("residential" | "living_street" | "unclassified") if length_m < 90.0 => {
                    seed.min_zoom.max(13)
                }
                _ => seed.min_zoom,
            }
        }
        SharedLineLayer::WaterLines => match class {
            Some("ditch" | "drain") if dangling && length_m < 220.0 => seed.min_zoom.max(13),
            Some("stream") if dangling && length_m < 120.0 => seed.min_zoom.max(12),
            _ => seed.min_zoom,
        },
        SharedLineLayer::Railways => seed.min_zoom,
    }
}

fn road_geometry_min_zoom(seed: &RenderSeed, tier: RoadRenderTier) -> u8 {
    // Intersection splitting creates short but essential connectors. Visibility
    // belongs to the class and downstream zoom policy, never fragment length.
    let class = attr_value(&seed.attrs, "class");
    let subclass = road_subclass_value(&seed.attrs);
    match tier {
        RoadRenderTier::Major | RoadRenderTier::Secondary => seed.min_zoom,
        RoadRenderTier::Local => match class {
            Some("pedestrian") => seed.min_zoom.max(12),
            _ => seed.min_zoom.max(11),
        },
        RoadRenderTier::Internal => match subclass.as_deref() {
            Some(
                "parking_aisle" | "driveway" | "drive-through" | "drive_through"
                | "emergency_access" | "private",
            ) => seed.min_zoom.max(14),
            _ => seed.min_zoom.max(13),
        },
        RoadRenderTier::PathDetail => seed.min_zoom.max(match class {
            Some("track" | "pedestrian" | "steps") => 13,
            _ => 14,
        }),
    }
}

fn should_drop_render_loop_for_policy(seed: &RenderSeed, policy: RenderSeedPolicy) -> bool {
    matches!(
        policy,
        RenderSeedPolicy::Generic(SharedLineLayer::WaterLines)
    ) && seed.start_node == seed.end_node
        && line_length_m(&seed.points) < 24.0
}

fn road_render_tier(attrs: &[(String, String)]) -> Option<RoadRenderTier> {
    match attr_value(attrs, "class") {
        Some("motorway" | "trunk" | "primary") => Some(RoadRenderTier::Major),
        Some("secondary" | "tertiary") => Some(RoadRenderTier::Secondary),
        Some(
            "unclassified" | "residential" | "living_street" | "pedestrian" | "busway"
            | "bus_guideway",
        ) => Some(RoadRenderTier::Local),
        Some("service") => Some(RoadRenderTier::Internal),
        Some("track" | "path" | "footway" | "cycleway" | "bridleway" | "steps") => {
            Some(RoadRenderTier::PathDetail)
        }
        Some("other") => Some(RoadRenderTier::Local),
        _ => None,
    }
}

fn road_subclass_value(attrs: &[(String, String)]) -> Option<String> {
    attr_value(attrs, "subclass")
        .or_else(|| attr_value(attrs, "service"))
        .map(str::to_string)
        .or_else(|| (attr_value(attrs, "class") == Some("service")).then(|| "service".to_string()))
}

fn road_geometry_merge_signature(attrs: &[(String, String)]) -> Vec<(String, String)> {
    // Every published road attribute is styleable. Merging across any
    // difference would erase it through `common_attrs`, which is especially
    // visible for physical corridor width, surface, access and direction.
    attrs.to_vec()
}

fn canonicalize_points(points: &[PointKey]) -> Vec<PointKey> {
    let reversed = points.iter().copied().rev().collect::<Vec<_>>();
    if reversed.as_slice() < points {
        reversed
    } else {
        points.to_vec()
    }
}

fn canonicalize_nodes(start_node: u32, end_node: u32) -> (u32, u32) {
    if start_node <= end_node {
        (start_node, end_node)
    } else {
        (end_node, start_node)
    }
}

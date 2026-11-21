//! Tile-ready ids: the package's order of nodes, edges and shared infos,
//! and the road graph ids that follow from it.
//!
//! A compile renumbers its road graph ([`renumber_in_package_order`]) before
//! it writes the package, so every id the package stores is a position in
//! the package's order: node `i` has id `i`, edge `i` id
//! [`PACKAGE_EDGE_ID_BASE`]` + i`, shared info `i` id `i + 1`, and
//! components are numbered by their first node. The order itself follows
//! from what the graph holds (each node's position on a fixed world lattice,
//! each edge's way, direction and ends), not from the order the compiler
//! made things in. So a compile split into tiles along ranges of the node
//! order can number each tile's nodes, edges and shared infos from the
//! counts of the tiles before it and get the ids, and the bytes, a single
//! compile would: the groundwork for compiling continents as tiles.

use rayon::prelude::*;
use rustc_hash::FxHashMap;
use wayfarer_package::TraversalDirection;
use wayfarer_package::format::{RoutingPackageError, edge_id_span_limit, to_e7};
use wayfarer_road_graph::{ResolvedEdgeRestriction, RoadGraph};

/// The package's first edge id: edge `i` (in package order) has id
/// `PACKAGE_EDGE_ID_BASE + i`.
pub(crate) const PACKAGE_EDGE_ID_BASE: u64 = 1;

/// Hilbert index of (x, y) on a 2^32 grid.
fn hilbert_index(mut x: u32, mut y: u32) -> u64 {
    let mut index = 0u64;
    let mut s = 1u32 << 31;
    while s > 0 {
        let rx = u32::from(x & s > 0);
        let ry = u32::from(y & s > 0);
        index += u64::from(s) * u64::from(s) * u64::from((3 * rx) ^ ry);
        if ry == 0 {
            if rx == 1 {
                x = (s - 1).wrapping_sub(x);
                y = (s - 1).wrapping_sub(y);
            }
            std::mem::swap(&mut x, &mut y);
        }
        s >>= 1;
    }
    index
}

/// A node's place along the package's node order: the Hilbert index of its
/// E7 position on one fixed world lattice (longitude + 180° and latitude +
/// 90° in E7 steps, on a 2^32 grid). It depends on the position alone, not
/// on the package's extent or on other nodes, so any range of keys can be a
/// tile (an aligned Hilbert cell is one contiguous range) and a tile orders
/// its nodes as the whole compile would. Distinct E7 positions have distinct
/// keys, and nearby nodes stay near in memory.
pub(crate) fn world_node_key(lat: f64, lng: f64) -> u64 {
    let lattice =
        |e7: i32, offset: i64| (i64::from(e7) + offset).clamp(0, i64::from(u32::MAX)) as u32;
    hilbert_index(
        lattice(to_e7(lng), 1_800_000_000),
        lattice(to_e7(lat), 900_000_000),
    )
}

/// `value`'s bits, ordered as [`f64::total_cmp`] orders the values.
fn ordered_f64(value: f64) -> i64 {
    let bits = value.to_bits() as i64;
    bits ^ (((bits >> 63) as u64) >> 1) as i64
}

/// Where a node goes in the package's order: along the world Hilbert curve
/// ([`world_node_key`]), then nodes at one E7 position (possible only where a
/// position is not an E7 value) by exact position, then by OSM node id. Ties
/// left (synthesized nodes at one exact position) go by the build's order.
pub(crate) fn package_node_key(
    lat: f64,
    lng: f64,
    source_node_id: Option<i64>,
) -> (u64, i64, i64, Option<i64>) {
    (
        world_node_key(lat, lng),
        ordered_f64(lat),
        ordered_f64(lng),
        source_node_id,
    )
}

/// Where an edge goes among the edges of its source node: by its way, its
/// direction along the way, its target's place in the package, its segment
/// of the way and its length. It says what the edge is, not when the builder
/// made it. Ties left (one way running twice between two nodes in one
/// direction, with one length) go by the road graph's edge ids.
pub(crate) fn within_node_edge_key(
    osm_way_id: i64,
    direction: TraversalDirection,
    target: u32,
    segment_index: u32,
    distance_m: f32,
) -> (i64, bool, u32, u32, u32) {
    (
        osm_way_id,
        direction == TraversalDirection::Backward,
        target,
        segment_index,
        distance_m.to_bits(),
    )
}

/// Gives `graph` the ids its package should have, and stores its nodes,
/// edges and shared infos in that order: node, edge and shared-info ids
/// become positions in the package's order (nodes and shared infos no edge
/// uses get the ids after those), components are numbered by their first
/// node in that order, and resolved restrictions and opposing edges are
/// renamed to match.
///
/// The package's order follows from the graph alone: nodes along a Hilbert
/// curve over a fixed world lattice of E7 positions (ties by exact position,
/// then OSM node id); a node's out-edges by their way id, direction along the
/// way, target, segment of the way and length (then the current id); shared
/// infos by their first edge. Node `i` gets id `i`, edge `i` id `i + 1`,
/// shared info `i` id `i + 1`. So a compile split into tiles along ranges of
/// the node order can number each tile from the counts of the tiles before
/// it and get the ids a single compile gets. Since every id is its index
/// (plus one), the id columns take no space, and the package build reads
/// the graph in place. Renumbering a second time changes nothing. A
/// restriction or opposing edge that names no edge of the graph is left
/// naming none (id 0, which no edge has).
pub fn renumber_in_package_order(graph: &mut RoadGraph) -> Result<(), RoutingPackageError> {
    let network = &graph.graph;
    let node_count = network.node_count();
    let edge_count = network.edge_count();
    if edge_count >= u32::MAX as usize || node_count >= u32::MAX as usize {
        return Err(RoutingPackageError::Unsupported(
            "the graph exceeds u32 indexing".into(),
        ));
    }
    let (sources, targets) = (network.sources(), network.targets());

    // Nodes: the endpoints of edges by key, then the others by key.
    let mut node_used = vec![false; node_count];
    for (source, target) in sources.iter().zip(targets) {
        node_used[*source as usize] = true;
        node_used[*target as usize] = true;
    }
    let mut nodes = (0..node_count as u32).collect::<Vec<_>>();
    nodes.par_sort_by_cached_key(|raw| {
        let raw = *raw as usize;
        let (lat, lng) = network.node_lat_lng(raw);
        (
            !node_used[raw],
            package_node_key(lat, lng, network.node_source_node_id(raw)),
            raw,
        )
    });
    drop(node_used);
    // New id (= position) per current node.
    let mut node_position = vec![0u32; node_count];
    for (position, raw) in nodes.iter().enumerate() {
        node_position[*raw as usize] = position as u32;
    }

    // Each edge's shared info (its index in the store).
    let shared_by_id = graph
        .shared_edge_infos
        .edges()
        .iter()
        .enumerate()
        .map(|(index, info)| (info.shared_edge_info_id, index as u32))
        .collect::<FxHashMap<_, _>>();
    let edge_shared_raw = (0..edge_count)
        .into_par_iter()
        .map(|raw| shared_by_id.get(&network.edge_shared_info_id(raw)).copied())
        .collect::<Option<Vec<u32>>>()
        .ok_or_else(|| {
            RoutingPackageError::Unsupported("an edge's shared edge info is missing".into())
        })?;
    drop(shared_by_id);

    // Edges: by source, then by `within_node_edge_key`. That key is read only
    // between edges of one source, so the sort holds 8 bytes per edge.
    let shared_edges = graph.shared_edge_infos.edges();
    let within_node_key = |raw: u32| {
        let raw = raw as usize;
        (
            within_node_edge_key(
                shared_edges[edge_shared_raw[raw] as usize].osm_way_id,
                network.edge_traversal_direction(raw),
                node_position[targets[raw] as usize],
                network.edge_segment_index(raw),
                network.edge_distance_m(raw),
            ),
            network.edge_id(raw),
            raw,
        )
    };
    let mut order = (0..edge_count as u32)
        .into_par_iter()
        .map(|raw| (node_position[sources[raw as usize] as usize], raw))
        .collect::<Vec<_>>();
    order.par_sort_unstable_by(|a, b| {
        a.0.cmp(&b.0)
            .then_with(|| within_node_key(a.1).cmp(&within_node_key(b.1)))
    });
    let edge_raw = order.into_iter().map(|(_, raw)| raw).collect::<Vec<_>>();
    drop(node_position);
    // Position per current edge id (a dense lookup, as the package build's).
    let old_base = (0..edge_count)
        .map(|raw| network.edge_id(raw))
        .min()
        .unwrap_or(0);
    let span = (0..edge_count)
        .map(|raw| network.edge_id(raw) - old_base)
        .max()
        .map_or(0, |span| span as usize + 1);
    if span > edge_id_span_limit(edge_count) {
        return Err(RoutingPackageError::Unsupported(format!(
            "edge ids span {span} values for {edge_count} edges"
        )));
    }
    let mut position_by_old_id = vec![u32::MAX; span];
    for (position, raw) in edge_raw.iter().enumerate() {
        let id = network.edge_id(*raw as usize);
        let slot = &mut position_by_old_id[(id - old_base) as usize];
        if *slot != u32::MAX {
            return Err(RoutingPackageError::Unsupported(format!(
                "edge id {id} is not unique"
            )));
        }
        *slot = position as u32;
    }
    let new_edge_id = |old: u64| {
        old.checked_sub(old_base)
            .and_then(|slot| position_by_old_id.get(slot as usize))
            .filter(|position| **position != u32::MAX)
            .map_or(0, |position| PACKAGE_EDGE_ID_BASE + u64::from(*position))
    };

    // Shared infos: by their first edge, then the unused ones in their
    // current order.
    let shared_count = shared_edges.len();
    let mut shared_ids = vec![0u64; shared_count];
    let mut next = 0u64;
    for raw in &edge_raw {
        let slot = &mut shared_ids[edge_shared_raw[*raw as usize] as usize];
        if *slot == 0 {
            next += 1;
            *slot = next;
        }
    }
    for slot in &mut shared_ids {
        if *slot == 0 {
            next += 1;
            *slot = next;
        }
    }
    // Each edge's new shared-info id, in the new edge order.
    let edge_shared_ids = edge_raw
        .iter()
        .map(|raw| shared_ids[edge_shared_raw[*raw as usize] as usize])
        .collect::<Vec<_>>();
    drop(edge_shared_raw);

    // Components, by their first node.
    let mut components = FxHashMap::<u32, u32>::default();
    for raw in &nodes {
        if let Some(component) = network.node_component_id(*raw as usize) {
            let next = components.len() as u32;
            components.entry(component).or_insert(next);
        }
    }

    // Into the new order, then the new ids: each its index (plus one).
    let network = &mut graph.graph;
    network.permute_edges(&edge_raw);
    drop(edge_raw);
    network.permute_nodes(&nodes);
    if graph.node_road_type_counts.len() == node_count {
        let mut counts = std::mem::take(&mut graph.node_road_type_counts);
        graph.node_road_type_counts = nodes
            .iter()
            .map(|raw| std::mem::take(&mut counts[*raw as usize]))
            .collect();
    }
    drop(nodes);
    network.renumber_edges(
        |_, edge| PACKAGE_EDGE_ID_BASE + edge as u64,
        |_, edge| edge_shared_ids[edge],
        |network, edge| network.edge_opposing_edge_id(edge).map(new_edge_id),
    );
    drop(edge_shared_ids);
    network.renumber_nodes(
        |_, node| node as i64,
        |network, node| {
            network
                .node_component_id(node)
                .map(|component| components[&component])
        },
    );
    // The store moves into the new order too, so that the package build
    // (which reads the shared infos in id order) reads them in place.
    let mut shared_order = vec![0u32; shared_count];
    for (index, id) in shared_ids.into_iter().enumerate() {
        shared_order[(id - 1) as usize] = index as u32;
    }
    graph.shared_edge_infos.reorder(&shared_order, 1);
    for restriction in &mut graph.resolved_restrictions {
        match restriction {
            ResolvedEdgeRestriction::Simple {
                from_edge_id,
                to_edge_id,
                ..
            } => {
                *from_edge_id = new_edge_id(*from_edge_id);
                *to_edge_id = new_edge_id(*to_edge_id);
            }
            ResolvedEdgeRestriction::Complex {
                from_edge_id,
                to_edge_id,
                via_edge_ids,
                ..
            } => {
                *from_edge_id = new_edge_id(*from_edge_id);
                *to_edge_id = new_edge_id(*to_edge_id);
                for via in via_edge_ids {
                    *via = new_edge_id(*via);
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use wayfarer_package::{DirectionalAccess, RoadType};
    use wayfarer_road_graph::{RoadGraphBuilder, RoutableWay, enhance_road_graph};

    /// Streets meeting at shared nodes, one of them one-way, pushed in
    /// `order`.
    fn streets(order: &[usize]) -> RoadGraph {
        let ways = [
            (11, vec![(50.0, 19.9), (50.001, 19.9), (50.002, 19.9)]),
            (7, vec![(50.001, 19.899), (50.001, 19.9), (50.001, 19.901)]),
            (3, vec![(50.002, 19.9), (50.002, 19.901), (50.001, 19.901)]),
            (5, vec![(50.0, 19.9), (50.0, 19.901), (50.001, 19.901)]),
        ];
        let mut builder = RoadGraphBuilder::new("renumber");
        for index in order {
            let (osm_way_id, points) = ways[*index].clone();
            builder.push_way(RoutableWay {
                osm_way_id,
                points,
                road_type: RoadType::Residential,
                directional_access: if osm_way_id == 3 {
                    DirectionalAccess::Forward
                } else {
                    DirectionalAccess::Both
                },
                highway: Some("residential".into()),
                ..Default::default()
            });
        }
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);
        graph
    }

    /// Each edge as (source, target, way, direction, id, shared-info id),
    /// by package position, and each node's (position, id, component).
    fn ids(graph: &RoadGraph) -> (Vec<String>, Vec<String>) {
        let way_of = |id: u64| {
            graph
                .shared_edge_infos
                .iter()
                .find(|info| info.shared_edge_info_id == id)
                .unwrap()
                .osm_way_id
        };
        let mut edges = graph
            .graph
            .edge_references()
            .map(|edge| {
                let (source, target) = (
                    graph.graph.node(edge.source()),
                    graph.graph.node(edge.target()),
                );
                let edge = edge.weight();
                (
                    edge.edge_id,
                    format!(
                        "{:?} {:?} way {} {:?} shared {} opposing {:?}",
                        (source.lat, source.lng),
                        (target.lat, target.lng),
                        way_of(edge.shared_edge_info_id),
                        edge.traversal_direction,
                        edge.shared_edge_info_id,
                        edge.opposing_edge_id,
                    ),
                )
            })
            .collect::<Vec<_>>();
        edges.sort();
        let mut nodes = graph
            .graph
            .node_weights()
            .map(|node| {
                format!(
                    "{} {:?} {:?}",
                    node.id,
                    (node.lat, node.lng),
                    node.component_id
                )
            })
            .collect::<Vec<_>>();
        nodes.sort();
        (
            edges
                .into_iter()
                .map(|(id, edge)| format!("{id}: {edge}"))
                .collect(),
            nodes,
        )
    }

    #[test]
    fn renumbered_ids_follow_from_the_graph_not_the_push_order() {
        let mut first = streets(&[0, 1, 2, 3]);
        let mut second = streets(&[3, 1, 0, 2]);
        assert_ne!(
            ids(&first),
            ids(&second),
            "the builder numbers by push order"
        );
        renumber_in_package_order(&mut first).unwrap();
        renumber_in_package_order(&mut second).unwrap();
        assert_eq!(ids(&first), ids(&second));

        // Positions: edge ids 1..=n, shared-info ids 1..=m, node ids 0..n.
        let mut edge_ids = first
            .graph
            .edge_weights()
            .map(|edge| edge.edge_id)
            .collect::<Vec<_>>();
        edge_ids.sort_unstable();
        assert_eq!(
            edge_ids,
            (1..=first.edge_count() as u64).collect::<Vec<_>>()
        );
        let mut shared_ids = first
            .shared_edge_infos
            .iter()
            .map(|info| info.shared_edge_info_id)
            .collect::<Vec<_>>();
        shared_ids.sort_unstable();
        assert_eq!(
            shared_ids,
            (1..=first.shared_edge_infos.len() as u64).collect::<Vec<_>>()
        );
        let mut node_ids = first
            .graph
            .node_weights()
            .map(|node| node.id)
            .collect::<Vec<_>>();
        node_ids.sort_unstable();
        assert_eq!(node_ids, (0..first.node_count() as i64).collect::<Vec<_>>());

        // Stored in that order: every id is its index (plus one).
        for edge in 0..first.edge_count() {
            assert_eq!(first.graph.edge_id(edge), edge as u64 + 1);
        }
        for node in 0..first.node_count() {
            assert_eq!(first.graph.node_id(node), node as i64);
        }
        for (index, info) in first.shared_edge_infos.iter().enumerate() {
            assert_eq!(info.shared_edge_info_id, index as u64 + 1);
        }

        // A second pass changes nothing.
        let before = ids(&first);
        renumber_in_package_order(&mut first).unwrap();
        assert_eq!(ids(&first), before);
    }

    #[test]
    fn world_keys_follow_the_hilbert_curve_on_one_lattice() {
        // Distinct E7 positions, distinct keys; the key ignores everything
        // but the position.
        let a = world_node_key(50.0, 19.9);
        assert_ne!(a, world_node_key(50.0000001, 19.9));
        assert_ne!(a, world_node_key(50.0, 19.9000001));
        assert_eq!(a, world_node_key(50.00000001, 19.9));
        // The lattice's corners.
        assert_eq!(world_node_key(-90.0, -180.0), 0);
        assert!(world_node_key(90.0, 180.0) > a);
        // A Hilbert curve's first quadrant (x and y below the half) comes
        // first: the south-west of the lattice before its north-east.
        assert!(world_node_key(-60.0, -170.0) < world_node_key(60.0, 170.0));
    }
}

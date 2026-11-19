//! Topology: a layer's lines split at shared and snapped nodes into edges.

use super::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub(crate) struct PointKey {
    pub(crate) lat: i64,
    pub(crate) lng: i64,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub(crate) struct SnapCompatKey {
    layer_value: Option<String>,
    pub(crate) brunnel: BrunnelKind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub(crate) enum BrunnelKind {
    #[default]
    Surface,
    Bridge,
    Tunnel,
    Ford,
    /// A ferry or shuttle-train line (`route=ferry`/`shuttle_train`). It meets
    /// other lines only at the nodes it shares with them, never where it
    /// crosses one, even another ferry.
    Ferry,
}

/// A topology node's identity within one layer: its source node, or its
/// position under an interned [`SnapCompatKey`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum NodeKey {
    Source(i64),
    Snapped(PointKey, u32),
}

/// Whether a point's source node, or its position under the line's snap key,
/// occurs more than once in the layer.
#[derive(Debug, Clone, Copy)]
struct PointRecurrence {
    repeated_source: bool,
    repeated_snapped: bool,
}

/// A line point: its E7 position and source node, 16 bytes (a layer has
/// tens of millions). Line points are the node store's `i32` E7 values, or
/// crossings inserted between them, so they fit.
#[derive(Debug, Clone, Copy)]
pub(crate) struct RawPoint {
    lat: i32,
    lng: i32,
    /// [`NO_SOURCE_NODE`] for none.
    source_node_id: i64,
}

const NO_SOURCE_NODE: i64 = i64::MIN;

impl RawPoint {
    pub(crate) fn new(key: PointKey, source_node_id: Option<i64>) -> Self {
        let [lat, lng] = e7(key);
        assert_ne!(
            source_node_id,
            Some(NO_SOURCE_NODE),
            "source node id out of range"
        );
        Self {
            lat,
            lng,
            source_node_id: source_node_id.unwrap_or(NO_SOURCE_NODE),
        }
    }

    pub(crate) fn key(&self) -> PointKey {
        PointKey {
            lat: i64::from(self.lat),
            lng: i64::from(self.lng),
        }
    }

    pub(crate) fn source_node_id(&self) -> Option<i64> {
        (self.source_node_id != NO_SOURCE_NODE).then_some(self.source_node_id)
    }
}

#[derive(Debug, Clone)]
pub(crate) struct RawLine {
    pub(crate) feature_id: i64,
    layer: SharedLineLayer,
    pub(crate) points: Vec<RawPoint>,
    attrs: Arc<[(String, String)]>,
    compile_inputs: Arc<[(String, String)]>,
    min_zoom: u8,
    max_zoom: u8,
    sort_order: u8,
    routing: Option<Arc<RoadRoutingAttrs>>,
    source_lineage_ids: Arc<[i64]>,
    pub(crate) snap_compat: SnapCompatKey,
}

#[derive(Debug, Default)]
struct LocalNodeRegistry {
    /// Ids are assigned in first-seen order, so the map's own order is moot.
    by_key: FxHashMap<NodeKey, u32>,
    metadata: Vec<(PointKey, Option<i64>)>,
}

impl LocalNodeRegistry {
    fn node_id_for(&mut self, key: NodeKey, point: PointKey, source_node_id: Option<i64>) -> u32 {
        if let Some(id) = self.by_key.get(&key).copied() {
            id
        } else {
            let id = self.metadata.len() as u32;
            self.by_key.insert(key, id);
            self.metadata.push((point, source_node_id));
            id
        }
    }
}

pub(crate) fn append_layer_network(
    network: &mut SharedLineNetwork,
    inputs: Vec<SharedLineInput>,
    observer: &mut dyn FnMut(&str),
    render_products: bool,
) {
    let layer = inputs.first().map(|input| input.layer);
    let mut raw_lines = inputs
        .into_iter()
        .filter_map(raw_line_from_input)
        .collect::<Vec<_>>();
    if raw_lines.is_empty() {
        return;
    }
    let layer_name = match layer {
        Some(SharedLineLayer::Roads) => "roads",
        Some(SharedLineLayer::WaterLines) => "water_lines",
        Some(SharedLineLayer::Railways) => "railways",
        None => "unknown",
    };
    observer(&format!("{layer_name}_raw_lines"));
    if matches!(layer, Some(SharedLineLayer::Roads)) {
        repair_road_crossings(&mut raw_lines, &mut network.stats);
        observer(&format!("{layer_name}_repaired"));
    }

    // Snap-compatibility keys are interned per layer, so a snapped position is
    // counted and identified by (point, small id) rather than by a cloned key
    // carrying an optional string.
    let line_snap_ids = {
        let mut snap_ids = FxHashMap::<&SnapCompatKey, u32>::default();
        raw_lines
            .iter()
            .map(|line| {
                let next = snap_ids.len() as u32;
                *snap_ids.entry(&line.snap_compat).or_insert(next)
            })
            .collect::<Vec<_>>()
    };
    let recurrence = point_recurrence(&raw_lines, &line_snap_ids);

    // Edges go straight into the topology, which is sized once for them: a
    // line splits at most at each of its anchors (see
    // `append_topology_edges`). Built as local edges and then copied, every
    // edge record (~230 bytes) existed twice at the layer's peak.
    let mut point_offset = 0;
    let edge_bound = raw_lines
        .iter()
        .map(|line| {
            let line_recurrence = &recurrence[point_offset..point_offset + line.points.len()];
            point_offset += line.points.len();
            line_anchors(line_recurrence).count().saturating_sub(1)
        })
        .sum::<usize>();
    network.topology.edges.reserve_exact(edge_bound);
    let appended_from = network.topology.edges.len();
    let mut node_registry = LocalNodeRegistry::default();
    let node_offset = network.topology.nodes.len() as u32;
    let mut point_offset = 0;
    // Consumed: each line's points are freed once its edges exist.
    for (raw_line, &snap_id) in raw_lines.into_iter().zip(&line_snap_ids) {
        let line_recurrence = &recurrence[point_offset..point_offset + raw_line.points.len()];
        point_offset += raw_line.points.len();
        append_topology_edges(
            raw_line,
            snap_id,
            line_recurrence,
            &mut node_registry,
            node_offset,
            &mut network.topology.edges,
        );
    }
    observer(&format!("{layer_name}_topology_edges"));
    if network.topology.edges.len() == appended_from {
        return;
    }
    drop(recurrence);
    let LocalNodeRegistry { by_key, metadata } = node_registry;
    drop(by_key);
    network.topology.nodes.reserve_exact(metadata.len());

    for (local_id, (point, source_node_id)) in metadata.iter().enumerate() {
        network.topology.nodes.push(SharedTopologyNode {
            id: node_offset + local_id as u32,
            lat: from_fixed(point.lat),
            lng: from_fixed(point.lng),
            source_node_id: *source_node_id,
        });
    }

    // The render pass reads exactly the edges appended above, so it borrows
    // them back out of the topology instead of keeping a second copy.
    if !render_products {
        return;
    }

    // `stats` is moved out so the render pass can borrow the topology edges
    // immutably while still recording into it.
    let mut stats = std::mem::take(&mut network.stats);
    let appended_edges = &network.topology.edges[appended_from..];
    let render_lines = if matches!(layer, Some(SharedLineLayer::Roads)) {
        build_road_render_products(appended_edges, &mut stats)
    } else {
        build_render_products_for_layer(appended_edges, &mut stats)
    };
    network.stats = stats;
    network.render.render_lines.extend(render_lines);
    observer(&format!("{layer_name}_render_products"));
}

fn raw_line_from_input(input: SharedLineInput) -> Option<RawLine> {
    // Already normalized and interned by `push_input`, and normalization is
    // idempotent — re-running it here would only copy the shared list back out.
    let snap_compat = snap_compat_key(input.layer, &input.attrs);
    let node_chain = input
        .node_chain
        .into_iter()
        .map(Some)
        .chain(std::iter::repeat(None))
        .take(input.geometry.len())
        .collect::<Vec<_>>();

    let mut points = Vec::<RawPoint>::with_capacity(input.geometry.len());
    for ((lat, lng), source_node_id) in input.geometry.into_iter().zip(node_chain) {
        let key = PointKey {
            lat: to_fixed(lat),
            lng: to_fixed(lng),
        };
        if points.last().map(RawPoint::key) != Some(key) {
            points.push(RawPoint::new(key, source_node_id));
        }
    }
    if points.len() < 2 {
        return None;
    }
    points.shrink_to_fit();
    Some(RawLine {
        feature_id: input.feature_id,
        layer: input.layer,
        points,
        attrs: input.attrs,
        compile_inputs: input.compile_inputs,
        min_zoom: input.min_zoom,
        max_zoom: input.max_zoom,
        sort_order: input.sort_order,
        // Shared by `push_input`.
        routing: input.routing,
        source_lineage_ids: input.source_lineage_ids.into(),
        snap_compat,
    })
}

/// The points a line may split at: its ends, and the points whose source
/// node or snapped position recurs in the layer.
fn line_anchors(recurrence: &[PointRecurrence]) -> impl Iterator<Item = usize> + '_ {
    recurrence
        .iter()
        .enumerate()
        .filter_map(move |(index, point)| {
            let is_endpoint = index == 0 || index + 1 == recurrence.len();
            (is_endpoint || point.repeated_source || point.repeated_snapped).then_some(index)
        })
}

/// Appends a line's edges to the topology `output`: one per stretch between
/// consecutive anchors, numbered on from `output`'s length, with their nodes
/// numbered on from `node_offset`.
fn append_topology_edges(
    raw_line: RawLine,
    snap_id: u32,
    recurrence: &[PointRecurrence],
    node_registry: &mut LocalNodeRegistry,
    node_offset: u32,
    output: &mut Vec<SharedTopologyEdge>,
) {
    let anchors = line_anchors(recurrence).collect::<Vec<_>>();
    if anchors.len() < 2 {
        return;
    }

    for window in anchors.windows(2) {
        let start_index = window[0];
        let end_index = window[1];
        if end_index <= start_index {
            continue;
        }

        let segment_points = simplify_anchor_points(
            raw_line.points[start_index..=end_index]
                .iter()
                .map(RawPoint::key)
                .collect(),
        );
        if segment_points.len() < 2 {
            continue;
        }

        let start_point = &raw_line.points[start_index];
        let end_point = &raw_line.points[end_index];
        let start_prefers_snapped = recurrence[start_index].repeated_snapped;
        let end_prefers_snapped = recurrence[end_index].repeated_snapped;
        let start_node = node_registry.node_id_for(
            node_key_for_point(start_point, snap_id, start_prefers_snapped),
            segment_points[0],
            start_point.source_node_id(),
        );
        let end_node = node_registry.node_id_for(
            node_key_for_point(end_point, snap_id, end_prefers_snapped),
            *segment_points.last().unwrap_or(&segment_points[0]),
            end_point.source_node_id(),
        );

        output.push(SharedTopologyEdge {
            edge_id: output.len() as u32,
            feature_id: raw_line.feature_id,
            layer: raw_line.layer,
            geometry: segment_points
                .iter()
                .map(|point| (from_fixed(point.lat), from_fixed(point.lng)))
                .collect(),
            attrs: raw_line.attrs.clone(),
            compile_inputs: raw_line.compile_inputs.clone(),
            min_zoom: raw_line.min_zoom,
            max_zoom: raw_line.max_zoom,
            sort_order: raw_line.sort_order,
            start_node: start_node + node_offset,
            end_node: end_node + node_offset,
            source_lineage_ids: Arc::clone(&raw_line.source_lineage_ids),
            routing: raw_line.routing.clone(),
        });
    }
}

/// Whether each point's source node, and its position under its line's snap
/// key, occurs more than once in the layer, flattened in line order. Counted
/// by sorting `(key, point)` pairs, one key at a time: 16 bytes a point,
/// where hash maps of counts sized to the point count took ~20 times that
/// together.
fn point_recurrence(raw_lines: &[RawLine], line_snap_ids: &[u32]) -> Vec<PointRecurrence> {
    let point_count = raw_lines
        .iter()
        .map(|line| line.points.len())
        .sum::<usize>();
    let point_index = |index: usize| u32::try_from(index).expect("a layer's points fit in u32");
    let mut recurrence = vec![
        PointRecurrence {
            repeated_source: false,
            repeated_snapped: false,
        };
        point_count
    ];

    let mut by_source = raw_lines
        .iter()
        .flat_map(|line| &line.points)
        .enumerate()
        .filter_map(|(index, point)| {
            point
                .source_node_id()
                .map(|node_id| (node_id, point_index(index)))
        })
        .collect::<Vec<_>>();
    by_source.par_sort_unstable();
    for run in by_source.chunk_by(|left, right| left.0 == right.0) {
        if run.len() > 1 {
            for (_, index) in run {
                recurrence[*index as usize].repeated_source = true;
            }
        }
    }
    drop(by_source);

    let mut by_snapped = raw_lines
        .iter()
        .zip(line_snap_ids)
        .flat_map(|(line, &snap_id)| line.points.iter().map(move |point| (point, snap_id)))
        .enumerate()
        .map(|(index, (point, snap_id))| ([point.lat, point.lng], snap_id, point_index(index)))
        .collect::<Vec<_>>();
    by_snapped.par_sort_unstable();
    for run in by_snapped.chunk_by(|left, right| (left.0, left.1) == (right.0, right.1)) {
        if run.len() > 1 {
            for (_, _, index) in run {
                recurrence[*index as usize].repeated_snapped = true;
            }
        }
    }
    recurrence
}

/// A point's E7 coordinates. Line points are the node store's `i32` E7
/// values (or crossings inserted between them), so they fit.
pub(crate) fn e7(point: PointKey) -> [i32; 2] {
    let fit = |value: i64| i32::try_from(value).expect("line coordinates are E7 i32 values");
    [fit(point.lat), fit(point.lng)]
}

fn node_key_for_point(point: &RawPoint, snap_id: u32, prefer_snapped: bool) -> NodeKey {
    if prefer_snapped {
        NodeKey::Snapped(point.key(), snap_id)
    } else {
        point
            .source_node_id()
            .map(NodeKey::Source)
            .unwrap_or(NodeKey::Snapped(point.key(), snap_id))
    }
}

/// The plane a line lies in. `bridge=no`, `tunnel=no` and `ford=no` are the
/// surface, and so is `layer=0`, the default layer.
fn snap_compat_key(_layer: SharedLineLayer, attrs: &[(String, String)]) -> SnapCompatKey {
    let layer_value = attr_value(attrs, "layer")
        .map(str::trim)
        .filter(|value| !value.is_empty() && value.parse::<i16>().ok() != Some(0))
        .map(str::to_string);
    let is_set = |key: &str| attr_value(attrs, key).is_some_and(|value| value != "no");
    let brunnel = if matches!(attr_value(attrs, "route"), Some("ferry" | "shuttle_train")) {
        BrunnelKind::Ferry
    } else if is_set("bridge") {
        BrunnelKind::Bridge
    } else if is_set("tunnel") {
        BrunnelKind::Tunnel
    } else if is_set("ford") {
        BrunnelKind::Ford
    } else {
        BrunnelKind::Surface
    };
    SnapCompatKey {
        layer_value,
        brunnel,
    }
}

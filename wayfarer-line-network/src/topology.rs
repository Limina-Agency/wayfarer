//! Topology: a layer's lines split at shared and snapped nodes into edges.

use super::*;
use std::mem::MaybeUninit;
use std::ops::Range;
use std::sync::atomic::{AtomicU8, AtomicU32, Ordering};

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

/// Whether a point's source node, and its position under its line's snap
/// key, occur more than once in the layer: [`REPEATED_SOURCE`] and
/// [`REPEATED_SNAPPED`] bits.
#[derive(Debug, Clone, Copy, Default)]
struct PointRecurrence(u8);

const REPEATED_SOURCE: u8 = 1;
const REPEATED_SNAPPED: u8 = 2;

impl PointRecurrence {
    fn repeated_source(self) -> bool {
        self.0 & REPEATED_SOURCE != 0
    }

    fn repeated_snapped(self) -> bool {
        self.0 & REPEATED_SNAPPED != 0
    }
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

/// Lines go to the parallel passes over a layer in groups of this many, each
/// group's items written to its own range of a vector sized once.
pub(crate) const LINE_GROUP: usize = 1024;

/// Lines whose edges are built at a time: each batch's raw lines are freed
/// before the next, so the raw lines shrink as the edges grow.
const EDGE_BATCH_LINES: usize = 16 * 1024;

pub(crate) fn line_groups(line_count: usize) -> impl IndexedParallelIterator<Item = Range<usize>> {
    (0..line_count.div_ceil(LINE_GROUP))
        .into_par_iter()
        .map(move |group| group * LINE_GROUP..((group + 1) * LINE_GROUP).min(line_count))
}

/// The items `items` yields for each group of `line_count` lines, group after
/// group, collected in parallel into one vector allocated at its size:
/// `group_lens` holds how many each group yields.
pub(crate) fn collect_by_line_groups<T, I>(
    line_count: usize,
    group_lens: &[usize],
    items: impl Fn(Range<usize>) -> I + Sync,
) -> Vec<T>
where
    T: Send,
    I: Iterator<Item = T>,
{
    let len = group_lens.iter().sum();
    let mut out = Vec::with_capacity(len);
    let mut pieces = Vec::with_capacity(group_lens.len());
    let mut rest = &mut out.spare_capacity_mut()[..len];
    for &group_len in group_lens {
        let (piece, tail) = std::mem::take(&mut rest).split_at_mut(group_len);
        pieces.push(piece);
        rest = tail;
    }
    // The pieces are written in parallel, so the vector's pages are first
    // touched across the pool too.
    line_groups(line_count)
        .zip(pieces)
        .for_each(|(lines, piece)| {
            let mut written = 0;
            for item in items(lines) {
                piece[written].write(item);
                written += 1;
            }
            assert_eq!(written, piece.len(), "a line group yields what it counted");
        });
    // SAFETY: each group wrote every slot of its piece (asserted above), and
    // the pieces tile the first `len` slots.
    unsafe { out.set_len(len) };
    out
}

pub(crate) fn append_layer_network(
    network: &mut SharedLineNetwork,
    inputs: Vec<SharedLineInput>,
    observer: &mut dyn FnMut(&str),
    render_products: bool,
    batches: Option<&mut Vec<Vec<SharedTopologyEdge>>>,
) {
    assert!(
        !(render_products && batches.is_some()),
        "render products read the topology's own edges"
    );
    let layer = inputs.first().map(|input| input.layer);
    // Each input becomes its raw line on its own; the collect keeps them in
    // input order. `filter_map` then drops the empty ones in place (`flatten`
    // would copy them all into a new vector).
    #[allow(clippy::filter_map_identity)]
    let mut raw_lines = inputs
        .into_par_iter()
        .map(raw_line_from_input)
        .collect::<Vec<_>>()
        .into_iter()
        .filter_map(|line| line)
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

    let line_snap_ids = line_snap_ids(&raw_lines);
    let point_offsets = point_offsets(&raw_lines);
    let recurrence = point_recurrence(&raw_lines, &line_snap_ids, &point_offsets);
    let anchors = LayerAnchors::new(&raw_lines, &recurrence, &point_offsets);
    let (anchor_nodes, node_points) = number_anchor_nodes(
        &raw_lines,
        &line_snap_ids,
        &recurrence,
        &point_offsets,
        &anchors,
    );
    drop(recurrence);
    drop(point_offsets);
    drop(line_snap_ids);

    // Edges go straight into the topology, which is sized once for them, or,
    // when `batches` takes them, into one vector per batch of lines below: a
    // line has an edge between each two consecutive anchors. Edge ids run on
    // from the edges of the layers before, wherever those went.
    let appended_from = network.topology.edges.len()
        + batches
            .as_deref()
            .map_or(0, |batches| batches.iter().map(Vec::len).sum());
    let edge_count = anchors.points.len() - raw_lines.len();
    let node_offset = network.topology.nodes.len() as u32;
    if batches.is_none() {
        network.topology.edges.reserve_exact(edge_count);
    }
    let mut layer_batches = Vec::new();
    // The lines' edges are written into their places a batch of lines at a
    // time, from the last batch back: each batch is split off the raw lines
    // and consumed in parallel, so the raw lines shrink as the edges grow,
    // and each line's last edge takes over its shared handles.
    while !raw_lines.is_empty() {
        let first_line = raw_lines.len().saturating_sub(EDGE_BATCH_LINES);
        let batch = raw_lines.split_off(first_line);
        if raw_lines.capacity() > 2 * raw_lines.len() + EDGE_BATCH_LINES {
            raw_lines.shrink_to_fit();
        }
        // Where a line's edges start among the layer's.
        let edge_offset = |line: usize| anchors.offsets[line] - line;
        let range = edge_offset(first_line)..edge_offset(first_line + batch.len());
        let mut own = Vec::new();
        let mut rest = if batches.is_some() {
            own.reserve_exact(range.len());
            &mut own.spare_capacity_mut()[..range.len()]
        } else {
            &mut network.topology.edges.spare_capacity_mut()[range.clone()]
        };
        let mut pieces = Vec::with_capacity(batch.len());
        for line in first_line..first_line + batch.len() {
            let (piece, tail) =
                std::mem::take(&mut rest).split_at_mut(edge_offset(line + 1) - edge_offset(line));
            pieces.push(piece);
            rest = tail;
        }
        batch
            .into_par_iter()
            .zip(pieces)
            .enumerate()
            .for_each(|(offset, (raw_line, piece))| {
                let line = first_line + offset;
                let anchor_range = anchors.offsets[line]..anchors.offsets[line + 1];
                write_line_edges(
                    raw_line,
                    &anchors.points[anchor_range.clone()],
                    &anchor_nodes[anchor_range],
                    node_offset,
                    // Edge ids run on line by line.
                    (appended_from + edge_offset(line)) as u32,
                    piece,
                );
            });
        if batches.is_some() {
            // SAFETY: every line of the batch wrote each of its edges' slots
            // (`write_line_edges` fills its piece), and the lines' pieces
            // tile the vector's first `range.len()` spare slots.
            unsafe { own.set_len(range.len()) };
            layer_batches.push(own);
        }
    }
    match batches {
        Some(batches) => {
            layer_batches.reverse();
            batches.append(&mut layer_batches);
        }
        // SAFETY: every line wrote each of its edges' slots (`write_line_edges`
        // fills its piece), and the lines' pieces tile the first `edge_count`
        // spare slots.
        None => unsafe {
            let edges = &mut network.topology.edges;
            edges.set_len(appended_from + edge_count)
        },
    }
    drop(raw_lines);
    drop(anchors);
    drop(anchor_nodes);
    observer(&format!("{layer_name}_topology_edges"));
    if edge_count == 0 {
        return;
    }
    network
        .topology
        .nodes
        .par_extend(node_points.par_iter().enumerate().map(|(local_id, point)| {
            SharedTopologyNode {
                id: node_offset + local_id as u32,
                lat: from_fixed(point.key().lat),
                lng: from_fixed(point.key().lng),
                source_node_id: point.source_node_id(),
            }
        }));
    drop(node_points);

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

/// Each line's snap-compatibility key as a small id, so a snapped position
/// is counted and identified by (point, id) rather than by a cloned key
/// carrying an optional string. The ids number the layer's distinct keys in
/// their sorted order; only their equality matters.
fn line_snap_ids(raw_lines: &[RawLine]) -> Vec<u32> {
    let mut keys = raw_lines
        .par_iter()
        .fold(FxHashSet::<&SnapCompatKey>::default, |mut keys, line| {
            keys.insert(&line.snap_compat);
            keys
        })
        .reduce(FxHashSet::default, |mut keys, more| {
            keys.extend(more);
            keys
        })
        .into_iter()
        .collect::<Vec<_>>();
    keys.sort_unstable();
    let ids = keys
        .into_iter()
        .enumerate()
        .map(|(id, key)| (key, id as u32))
        .collect::<FxHashMap<_, _>>();
    raw_lines
        .par_iter()
        .map(|line| ids[&line.snap_compat])
        .collect()
}

/// Where each line's points start in the layer's points, flattened in line
/// order; one more entry than lines, the last the point count.
fn point_offsets(raw_lines: &[RawLine]) -> Vec<u32> {
    let mut offsets = Vec::with_capacity(raw_lines.len() + 1);
    let mut total = 0usize;
    offsets.push(0);
    for line in raw_lines {
        total += line.points.len();
        offsets.push(u32::try_from(total).expect("a layer's points fit in u32"));
    }
    offsets
}

/// The points a line may split at: its ends, and the points whose source
/// node or snapped position recurs in the layer.
fn line_anchors(recurrence: &[PointRecurrence]) -> impl Iterator<Item = usize> + '_ {
    recurrence
        .iter()
        .enumerate()
        .filter_map(move |(index, point)| {
            let is_endpoint = index == 0 || index + 1 == recurrence.len();
            (is_endpoint || point.repeated_source() || point.repeated_snapped()).then_some(index)
        })
}

/// Every line's anchors ([`line_anchors`]), flattened in line order. A line
/// has at least two (its ends), and an edge between each two consecutive
/// ones.
struct LayerAnchors {
    /// Where each line's anchors start in `points`; one more entry than
    /// lines, the last the anchor count.
    offsets: Vec<usize>,
    /// Each anchor's point index in its line.
    points: Vec<u32>,
}

impl LayerAnchors {
    fn new(raw_lines: &[RawLine], recurrence: &[PointRecurrence], point_offsets: &[u32]) -> Self {
        let line_recurrence = |line: usize| {
            &recurrence[point_offsets[line] as usize..point_offsets[line + 1] as usize]
        };
        let mut offsets = (0..raw_lines.len())
            .into_par_iter()
            .map(|line| line_anchors(line_recurrence(line)).count())
            .collect::<Vec<_>>();
        let group_lens = line_groups(raw_lines.len())
            .map(|lines| offsets[lines].iter().sum())
            .collect::<Vec<usize>>();
        let mut total = 0;
        for offset in &mut offsets {
            let count = *offset;
            debug_assert!(count >= 2, "a raw line has two distinct ends");
            *offset = total;
            total += count;
        }
        offsets.push(total);
        u32::try_from(total).expect("a layer's anchors fit in u32");
        let points = collect_by_line_groups(raw_lines.len(), &group_lens, |lines| {
            lines.flat_map(|line| line_anchors(line_recurrence(line)).map(|index| index as u32))
        });
        Self { offsets, points }
    }
}

/// A topology node's identity within one layer, as one sortable value: its
/// source node, or its E7 position under its line's snap id.
fn node_key(point: &RawPoint, snap_id: u32, prefer_snapped: bool) -> (u64, u64) {
    let snapped = || {
        (
            1 << 32 | u64::from(snap_id),
            u64::from(point.lat as u32) << 32 | u64::from(point.lng as u32),
        )
    };
    if prefer_snapped {
        snapped()
    } else {
        point
            .source_node_id()
            .map_or_else(snapped, |node_id| (0, node_id as u64))
    }
}

fn node_key_share((high, low): (u64, u64)) -> usize {
    share_of(low ^ high.rotate_left(32))
}

/// Numbers the layer's topology nodes. Each anchor is a node, keyed by
/// [`node_key`]; walking the anchors line by line and along each line, a key
/// gets the next id the first time it comes. That is the order the edges
/// meet their nodes (start, then end, edge by edge), as consecutive edges
/// share their anchor. Found by sorting `(key, anchor)` one share of the
/// keys at a time: each anchor links to its key's first anchor, and one
/// pass in anchor order numbers the first ones and reads the rest from
/// them.
///
/// Returns each anchor's node id, and each node's point by id (its first
/// anchor's).
fn number_anchor_nodes(
    raw_lines: &[RawLine],
    line_snap_ids: &[u32],
    recurrence: &[PointRecurrence],
    point_offsets: &[u32],
    anchors: &LayerAnchors,
) -> (Vec<u32>, Vec<RawPoint>) {
    let line_keys = |line: usize| {
        let (points, snap_id) = (&raw_lines[line].points, line_snap_ids[line]);
        let recurrence = &recurrence[point_offsets[line] as usize..];
        let first_anchor = anchors.offsets[line];
        anchors.points[first_anchor..anchors.offsets[line + 1]]
            .iter()
            .enumerate()
            .map(move |(offset, &index)| {
                let key = node_key(
                    &points[index as usize],
                    snap_id,
                    recurrence[index as usize].repeated_snapped(),
                );
                (key, (first_anchor + offset) as u32)
            })
    };
    let share_lens = line_groups(raw_lines.len())
        .map(|lines| {
            let mut lens = [0usize; RECURRENCE_SHARES];
            for (key, _) in lines.flat_map(line_keys) {
                lens[node_key_share(key)] += 1;
            }
            lens
        })
        .collect::<Vec<_>>();
    let first_anchor = (0..anchors.points.len())
        .into_par_iter()
        .map(|_| AtomicU32::new(0))
        .collect::<Vec<_>>();
    for share in 0..RECURRENCE_SHARES {
        let group_lens = share_lens
            .iter()
            .map(|lens| lens[share])
            .collect::<Vec<_>>();
        let mut keyed = collect_by_line_groups(raw_lines.len(), &group_lens, |lines| {
            lines
                .flat_map(line_keys)
                .filter(|&(key, _)| node_key_share(key) == share)
        });
        keyed.par_sort_unstable();
        keyed
            .par_chunk_by(|left, right| left.0 == right.0)
            .for_each(|run| {
                let first = run[0].1;
                for &(_, anchor) in run {
                    first_anchor[anchor as usize].store(first, Ordering::Relaxed);
                }
            });
    }
    let mut node_of_anchor = first_anchor
        .into_iter()
        .map(AtomicU32::into_inner)
        .collect::<Vec<_>>();
    let mut node_points = Vec::new();
    for (line, raw_line) in raw_lines.iter().enumerate() {
        let line_anchors = anchors.offsets[line]..anchors.offsets[line + 1];
        for (anchor, &index) in line_anchors.clone().zip(&anchors.points[line_anchors]) {
            let first = node_of_anchor[anchor] as usize;
            // An anchor after its key's first reads the id the first got.
            node_of_anchor[anchor] = if first == anchor {
                node_points.push(raw_line.points[index as usize]);
                (node_points.len() - 1) as u32
            } else {
                node_of_anchor[first]
            };
        }
    }
    (node_of_anchor, node_points)
}

/// Writes a line's edges into `out`, one per stretch between consecutive
/// anchors (`anchors`, with their nodes `anchor_nodes`), numbered on from
/// `first_edge_id`, with their nodes numbered on from `node_offset`. The
/// line's last edge takes over its shared handles; the others clone them.
fn write_line_edges(
    raw_line: RawLine,
    anchors: &[u32],
    anchor_nodes: &[u32],
    node_offset: u32,
    first_edge_id: u32,
    out: &mut [MaybeUninit<SharedTopologyEdge>],
) {
    let RawLine {
        feature_id,
        layer,
        points,
        attrs,
        compile_inputs,
        min_zoom,
        max_zoom,
        sort_order,
        routing,
        source_lineage_ids,
        snap_compat: _,
    } = raw_line;
    assert_eq!(
        out.len() + 1,
        anchors.len(),
        "a line has an edge per anchor pair"
    );
    let geometry = |offset: usize| {
        let (start_index, end_index) = (anchors[offset] as usize, anchors[offset + 1] as usize);
        debug_assert!(start_index < end_index);
        let stretch = &points[start_index..=end_index];
        let mut geometry = Vec::with_capacity(stretch.len());
        simplified_indices(
            stretch.len(),
            |index| stretch[index].key(),
            |index| {
                let point = stretch[index].key();
                geometry.push((from_fixed(point.lat), from_fixed(point.lng)));
            },
        );
        geometry.shrink_to_fit();
        geometry
    };
    let edge = |offset: usize,
                attrs: Arc<[(String, String)]>,
                compile_inputs: Arc<[(String, String)]>,
                source_lineage_ids: Arc<[i64]>,
                routing: Option<Arc<RoadRoutingAttrs>>| SharedTopologyEdge {
        edge_id: first_edge_id + offset as u32,
        feature_id,
        layer,
        geometry: geometry(offset),
        attrs,
        compile_inputs,
        min_zoom,
        max_zoom,
        sort_order,
        start_node: anchor_nodes[offset] + node_offset,
        end_node: anchor_nodes[offset + 1] + node_offset,
        source_lineage_ids,
        routing,
    };
    let last = out.len() - 1;
    for (offset, slot) in out[..last].iter_mut().enumerate() {
        slot.write(edge(
            offset,
            Arc::clone(&attrs),
            Arc::clone(&compile_inputs),
            Arc::clone(&source_lineage_ids),
            routing.clone(),
        ));
    }
    out[last].write(edge(
        last,
        attrs,
        compile_inputs,
        source_lineage_ids,
        routing,
    ));
}

/// Whether each point's source node, and its position under its line's snap
/// key, occurs more than once in the layer, flattened in line order. Counted
/// by sorting `(key, point)` pairs, one key at a time and one share of the
/// keys at a time ([`RECURRENCE_SHARES`]; equal keys fall in the same share):
/// 16 bytes for a share's points, where hash maps of counts sized to the
/// point count took ~20 times that for all of them together. Each share's
/// pairs are gathered and their runs marked in parallel.
fn point_recurrence(
    raw_lines: &[RawLine],
    line_snap_ids: &[u32],
    point_offsets: &[u32],
) -> Vec<PointRecurrence> {
    let point_count = *point_offsets.last().unwrap_or(&0) as usize;
    let flags = (0..point_count)
        .into_par_iter()
        .map(|_| AtomicU8::new(0))
        .collect::<Vec<_>>();
    let line_points = |line: usize| {
        let base = point_offsets[line];
        raw_lines[line]
            .points
            .iter()
            .enumerate()
            .map(move |(index, point)| (point, base + index as u32))
    };
    let line_snapped = |line: usize| {
        let snap_id = line_snap_ids[line];
        line_points(line).map(move |(point, index)| (point, snap_id, index))
    };

    // Per line group, its points in each share: by source node, then by
    // snapped position.
    let share_lens = line_groups(raw_lines.len())
        .map(|lines| {
            let mut lens = [[0usize; RECURRENCE_SHARES]; 2];
            for (point, snap_id, _) in lines.flat_map(line_snapped) {
                if let Some(node_id) = point.source_node_id() {
                    lens[0][source_share(node_id)] += 1;
                }
                lens[1][snapped_share(point, snap_id)] += 1;
            }
            lens
        })
        .collect::<Vec<_>>();

    for share in 0..RECURRENCE_SHARES {
        let group_lens = share_lens
            .iter()
            .map(|lens| lens[0][share])
            .collect::<Vec<_>>();
        let mut by_source = collect_by_line_groups(raw_lines.len(), &group_lens, |lines| {
            lines
                .flat_map(line_points)
                .filter_map(move |(point, index)| {
                    point
                        .source_node_id()
                        .filter(|node_id| source_share(*node_id) == share)
                        .map(|node_id| (node_id, index))
                })
        });
        by_source.par_sort_unstable();
        mark_repeated(
            &by_source,
            |entry| entry.0,
            |entry| entry.1,
            &flags,
            REPEATED_SOURCE,
        );
    }

    for share in 0..RECURRENCE_SHARES {
        let group_lens = share_lens
            .iter()
            .map(|lens| lens[1][share])
            .collect::<Vec<_>>();
        let mut by_snapped = collect_by_line_groups(raw_lines.len(), &group_lens, |lines| {
            lines
                .flat_map(line_snapped)
                .filter(move |&(point, snap_id, _)| snapped_share(point, snap_id) == share)
                .map(|(point, snap_id, index)| ([point.lat, point.lng], snap_id, index))
        });
        by_snapped.par_sort_unstable();
        mark_repeated(
            &by_snapped,
            |entry| (entry.0, entry.1),
            |entry| entry.2,
            &flags,
            REPEATED_SNAPPED,
        );
    }
    flags
        .into_iter()
        .map(|flag| PointRecurrence(flag.into_inner()))
        .collect()
}

/// Sets `bit` on the points of every run of more than one equal key in
/// `sorted`.
fn mark_repeated<T: Sync, K: PartialEq>(
    sorted: &[T],
    key: impl Fn(&T) -> K + Sync,
    point: impl Fn(&T) -> u32 + Sync,
    flags: &[AtomicU8],
    bit: u8,
) {
    sorted
        .par_chunk_by(|left, right| key(left) == key(right))
        .filter(|run| run.len() > 1)
        .for_each(|run| {
            for entry in run {
                flags[point(entry) as usize].fetch_or(bit, Ordering::Relaxed);
            }
        });
}

/// The shares [`point_recurrence`] splits each key's points into: the top
/// [`RECURRENCE_SHARE_BITS`] bits of a multiplicative hash of the key.
const RECURRENCE_SHARE_BITS: u32 = 2;
const RECURRENCE_SHARES: usize = 1 << RECURRENCE_SHARE_BITS;

fn share_of(key: u64) -> usize {
    (key.wrapping_mul(0x9E37_79B9_7F4A_7C15) >> (u64::BITS - RECURRENCE_SHARE_BITS)) as usize
}

fn source_share(node_id: i64) -> usize {
    share_of(node_id as u64)
}

fn snapped_share(point: &RawPoint, snap_id: u32) -> usize {
    share_of((u64::from(point.lat as u32) << 32 | u64::from(point.lng as u32)) ^ u64::from(snap_id))
}

/// A point's E7 coordinates. Line points are the node store's `i32` E7
/// values (or crossings inserted between them), so they fit.
pub(crate) fn e7(point: PointKey) -> [i32; 2] {
    let fit = |value: i64| i32::try_from(value).expect("line coordinates are E7 i32 values");
    [fit(point.lat), fit(point.lng)]
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

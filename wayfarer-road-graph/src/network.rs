//! [`RoadNetwork`]: the road graph's nodes and directed edges, stored column
//! by column.
//!
//! The graph used to be a `petgraph::Graph<RoadNode, RoadEdge>`: an array of
//! structs, 88 bytes per edge and about 80 per node with petgraph's adjacency
//! links. A country's compile holds the whole graph from the builder's finish
//! to the end of the package build, so its size is the floor of the compile's
//! memory. Here every field is its own column, and integer columns take the
//! narrowest width their values fit ([`Packed`]): ids that count up with the
//! index take no space at all, masks a byte, ids a `u32`. An edge is about 35
//! bytes and a node about 38, with the same values.
//!
//! Nodes and edges keep their insertion order and indices, so every walk over
//! the graph visits them as it did over petgraph. Edges are read back as
//! [`RoadEdge`] values ([`RoadNetwork::edge`], [`RoadNetwork::edge_references`])
//! and nodes as [`RoadNode`] values; hot loops read single columns. There are
//! no adjacency links: [`RoadNetwork::adjacency`] builds a compact index of a
//! direction's edges when a walk needs one, in petgraph's order.

use super::*;

/// Integer values in the narrowest of five layouts that holds them all:
/// counting up from a base (no storage), or one, two, four or eight bytes
/// each. A value that does not fit widens the column once, copying it.
#[derive(Debug, Clone)]
pub(crate) enum Packed {
    /// `base + index` at every index.
    Iota {
        base: u64,
        len: usize,
    },
    U8(Vec<u8>),
    U16(Vec<u16>),
    U32(Vec<u32>),
    U64(Vec<u64>),
}

impl Default for Packed {
    fn default() -> Self {
        Self::Iota { base: 0, len: 0 }
    }
}

/// The narrowest layout that holds `value`: 0 for one byte … 3 for eight.
fn width_of(value: u64) -> u8 {
    if value <= u64::from(u8::MAX) {
        0
    } else if value <= u64::from(u16::MAX) {
        1
    } else if value <= u64::from(u32::MAX) {
        2
    } else {
        3
    }
}

impl Packed {
    pub(crate) fn len(&self) -> usize {
        match self {
            Self::Iota { len, .. } => *len,
            Self::U8(values) => values.len(),
            Self::U16(values) => values.len(),
            Self::U32(values) => values.len(),
            Self::U64(values) => values.len(),
        }
    }

    #[inline]
    pub(crate) fn get(&self, index: usize) -> u64 {
        match self {
            Self::Iota { base, len } => {
                assert!(index < *len, "index out of bounds");
                base.wrapping_add(index as u64)
            }
            Self::U8(values) => u64::from(values[index]),
            Self::U16(values) => u64::from(values[index]),
            Self::U32(values) => u64::from(values[index]),
            Self::U64(values) => values[index],
        }
    }

    fn width(&self) -> u8 {
        match self {
            Self::Iota { base, len } => {
                if *len == 0 {
                    0
                } else {
                    // The widest value of a run is its largest, unless the
                    // run wraps past u64::MAX (then it holds u64::MAX).
                    let last = base.wrapping_add(*len as u64 - 1);
                    if last < *base { 3 } else { width_of(last) }
                }
            }
            Self::U8(_) => 0,
            Self::U16(_) => 1,
            Self::U32(_) => 2,
            Self::U64(_) => 3,
        }
    }

    /// The column in stored layout `width` (at least its current one), with
    /// room for `additional` more values.
    fn materialize(&mut self, width: u8, additional: usize) {
        let width = width.max(self.width());
        let len = self.len();
        let capacity = self.capacity().max(len + additional);
        let current = matches!(
            (&*self, width),
            (Self::U8(_), 0) | (Self::U16(_), 1) | (Self::U32(_), 2) | (Self::U64(_), 3)
        );
        if current {
            return;
        }
        let this = &*self;
        *self = match width {
            0 => {
                let mut values = Vec::with_capacity(capacity);
                values.extend((0..len).map(|index| this.get(index) as u8));
                Self::U8(values)
            }
            1 => {
                let mut values = Vec::with_capacity(capacity);
                values.extend((0..len).map(|index| this.get(index) as u16));
                Self::U16(values)
            }
            2 => {
                let mut values = Vec::with_capacity(capacity);
                values.extend((0..len).map(|index| this.get(index) as u32));
                Self::U32(values)
            }
            _ => {
                let mut values = Vec::with_capacity(capacity);
                values.extend((0..len).map(|index| this.get(index)));
                Self::U64(values)
            }
        };
    }

    fn capacity(&self) -> usize {
        match self {
            Self::Iota { len, .. } => *len,
            Self::U8(values) => values.capacity(),
            Self::U16(values) => values.capacity(),
            Self::U32(values) => values.capacity(),
            Self::U64(values) => values.capacity(),
        }
    }

    pub(crate) fn reserve_exact(&mut self, additional: usize) {
        match self {
            Self::Iota { .. } => {}
            Self::U8(values) => values.reserve_exact(additional),
            Self::U16(values) => values.reserve_exact(additional),
            Self::U32(values) => values.reserve_exact(additional),
            Self::U64(values) => values.reserve_exact(additional),
        }
    }

    pub(crate) fn push(&mut self, value: u64) {
        if let Self::Iota { base, len } = self {
            if *len == 0 {
                *base = value;
                *len = 1;
                return;
            }
            if base.wrapping_add(*len as u64) == value {
                *len += 1;
                return;
            }
        }
        let width = width_of(value);
        if width > self.width() || matches!(self, Self::Iota { .. }) {
            self.materialize(width, 1);
        }
        match self {
            Self::Iota { .. } => unreachable!("materialized above"),
            Self::U8(values) => values.push(value as u8),
            Self::U16(values) => values.push(value as u16),
            Self::U32(values) => values.push(value as u32),
            Self::U64(values) => values.push(value),
        }
    }

    pub(crate) fn set(&mut self, index: usize, value: u64) {
        if self.get(index) == value {
            return;
        }
        let width = width_of(value);
        if width > self.width() || matches!(self, Self::Iota { .. }) {
            self.materialize(width, 0);
        }
        match self {
            Self::Iota { .. } => unreachable!("materialized above"),
            Self::U8(values) => values[index] = value as u8,
            Self::U16(values) => values[index] = value as u16,
            Self::U32(values) => values[index] = value as u32,
            Self::U64(values) => values[index] = value,
        }
    }

    #[cfg(test)]
    pub(crate) fn iter(&self) -> impl ExactSizeIterator<Item = u64> + '_ {
        (0..self.len()).map(move |index| self.get(index))
    }

    /// Bytes held, for reporting.
    pub(crate) fn heap_bytes(&self) -> usize {
        match self {
            Self::Iota { .. } => 0,
            Self::U8(values) => values.capacity(),
            Self::U16(values) => values.capacity() * 2,
            Self::U32(values) => values.capacity() * 4,
            Self::U64(values) => values.capacity() * 8,
        }
    }

    /// Removes the value at `index`, moving the last value into its place.
    fn swap_remove(&mut self, index: usize) {
        let last = self.len() - 1;
        let value = self.get(last);
        self.set(index, value);
        match self {
            Self::Iota { len, .. } => *len -= 1,
            Self::U8(values) => drop(values.pop()),
            Self::U16(values) => drop(values.pop()),
            Self::U32(values) => drop(values.pop()),
            Self::U64(values) => drop(values.pop()),
        }
    }

    fn shrink_to_fit(&mut self) {
        match self {
            Self::Iota { .. } => {}
            Self::U8(values) => values.shrink_to_fit(),
            Self::U16(values) => values.shrink_to_fit(),
            Self::U32(values) => values.shrink_to_fit(),
            Self::U64(values) => values.shrink_to_fit(),
        }
    }
}

/// `Option<u64>` values: the [`Packed`] values, and a bit per value that is
/// `None` (its packed value is 0).
#[derive(Debug, Clone, Default)]
pub(crate) struct OptionPacked {
    values: Packed,
    none: Vec<u64>,
}

impl OptionPacked {
    #[inline]
    pub(crate) fn get(&self, index: usize) -> Option<u64> {
        let value = self.values.get(index);
        let none = self
            .none
            .get(index / 64)
            .is_some_and(|word| word & (1 << (index % 64)) != 0);
        (!none).then_some(value)
    }

    pub(crate) fn push(&mut self, value: Option<u64>) {
        let index = self.values.len();
        self.values.push(value.unwrap_or(0));
        if value.is_none() {
            self.mark_none(index, true);
        }
    }

    pub(crate) fn set(&mut self, index: usize, value: Option<u64>) {
        self.values.set(index, value.unwrap_or(0));
        self.mark_none(index, value.is_none());
    }

    fn mark_none(&mut self, index: usize, none: bool) {
        let word = index / 64;
        if none {
            if self.none.len() <= word {
                self.none.resize(word + 1, 0);
            }
            self.none[word] |= 1 << (index % 64);
        } else if let Some(word) = self.none.get_mut(word) {
            *word &= !(1 << (index % 64));
        }
    }

    pub(crate) fn reserve_exact(&mut self, additional: usize) {
        self.values.reserve_exact(additional);
    }

    fn swap_remove(&mut self, index: usize) {
        let last = self.values.len() - 1;
        let value = self.get(last);
        self.set(index, value);
        self.mark_none(last, false);
        self.values.swap_remove(index);
    }

    pub(crate) fn heap_bytes(&self) -> usize {
        self.values.heap_bytes() + self.none.capacity() * 8
    }

    fn shrink_to_fit(&mut self) {
        self.values.shrink_to_fit();
        self.none.shrink_to_fit();
    }
}

/// An `Option` of a small integer as a packed value: 0 for `None`, the value
/// plus one otherwise.
#[inline]
fn some_code(value: Option<impl Into<u64>>) -> u64 {
    value.map_or(0, |value| value.into() + 1)
}

#[inline]
fn code_u16(code: u64) -> Option<u16> {
    (code != 0).then(|| (code - 1) as u16)
}

#[inline]
fn code_u32(code: u64) -> Option<u32> {
    (code != 0).then(|| (code - 1) as u32)
}

pub(crate) fn road_type_code(road_type: RoadType) -> u8 {
    match road_type {
        RoadType::Motorway => 0,
        RoadType::Primary => 1,
        RoadType::Secondary => 2,
        RoadType::Tertiary => 3,
        RoadType::Residential => 4,
        RoadType::Track => 5,
        RoadType::Path => 6,
        RoadType::Other => 7,
    }
}

pub(crate) fn road_type_from_code(code: u8) -> RoadType {
    match code {
        0 => RoadType::Motorway,
        1 => RoadType::Primary,
        2 => RoadType::Secondary,
        3 => RoadType::Tertiary,
        4 => RoadType::Residential,
        5 => RoadType::Track,
        6 => RoadType::Path,
        _ => RoadType::Other,
    }
}

fn junction_code(kind: RoadJunctionKind) -> u8 {
    match kind {
        RoadJunctionKind::Unknown => 0,
        RoadJunctionKind::DeadEnd => 1,
        RoadJunctionKind::Through => 2,
        RoadJunctionKind::Intersection => 3,
        RoadJunctionKind::Roundabout => 4,
        RoadJunctionKind::Complex => 5,
    }
}

fn junction_from_code(code: u8) -> RoadJunctionKind {
    match code {
        1 => RoadJunctionKind::DeadEnd,
        2 => RoadJunctionKind::Through,
        3 => RoadJunctionKind::Intersection,
        4 => RoadJunctionKind::Roundabout,
        5 => RoadJunctionKind::Complex,
        _ => RoadJunctionKind::Unknown,
    }
}

fn tier_code(tier: Option<RoadHierarchyTier>) -> u8 {
    match tier {
        None => 0,
        Some(RoadHierarchyTier::Local) => 1,
        Some(RoadHierarchyTier::Collector) => 2,
        Some(RoadHierarchyTier::Arterial) => 3,
        Some(RoadHierarchyTier::Highway) => 4,
    }
}

fn tier_from_code(code: u8) -> Option<RoadHierarchyTier> {
    match code {
        1 => Some(RoadHierarchyTier::Local),
        2 => Some(RoadHierarchyTier::Collector),
        3 => Some(RoadHierarchyTier::Arterial),
        4 => Some(RoadHierarchyTier::Highway),
        _ => None,
    }
}

/// An edge's small fields in one byte: direction (bit 0), minor approach
/// (bit 1), end control (bits 2–4) and hierarchy tier code (bits 5–7).
fn edge_flags_byte(edge: &RoadEdge) -> u8 {
    u8::from(edge.traversal_direction == TraversalDirection::Backward)
        | u8::from(edge.end_minor_approach) << 1
        | (edge.end_control as u8) << 2
        | tier_code(edge.hierarchy_tier) << 5
}

const EDGE_BACKWARD: u8 = 1;
const EDGE_MINOR_APPROACH: u8 = 1 << 1;

/// A node's junction kind (bits 0–2), complex flag (bit 3) and highest
/// incident road type, plus one (bits 4–7, 0 for none).
fn node_kind_byte(node: &RoadNode) -> u8 {
    junction_code(node.junction_kind)
        | u8::from(node.is_complex_junction) << 3
        | node
            .highest_incident_road_type
            .map_or(0, |road_type| road_type_code(road_type) + 1)
            << 4
}

/// The nodes, a column per field.
#[derive(Debug, Clone, Default)]
struct NodeColumns {
    id: Packed,
    lat_lng: Vec<(f64, f64)>,
    source_node_id: OptionPacked,
    /// The component id plus one, 0 for none.
    component: Packed,
    incoming: Packed,
    outgoing: Packed,
    incident: Packed,
    /// [`node_kind_byte`].
    kind: Vec<u8>,
    roundabout_entries: Packed,
    roundabout_exits: Packed,
    barrier_class: Vec<u8>,
    barrier_access: Packed,
}

/// The directed edges, a column per field.
#[derive(Debug, Clone, Default)]
struct EdgeColumns {
    source: Vec<u32>,
    target: Vec<u32>,
    edge_id: Packed,
    shared_edge_info_id: Packed,
    distance_m: Vec<f32>,
    base_cost: Vec<f32>,
    mobility_mode_mask: Packed,
    osm_forward_mode_mask: Packed,
    osm_reverse_mode_mask: Packed,
    /// [`edge_flags_byte`].
    flags: Vec<u8>,
    segment_index: Packed,
    /// Headings and inferred speed plus one, 0 for none.
    start_heading: Packed,
    end_heading: Packed,
    inferred_speed: Packed,
    opposing_edge_id: OptionPacked,
}

/// The graph's nodes and directed edges; see the module documentation.
#[derive(Debug, Clone, Default)]
pub struct RoadNetwork {
    nodes: NodeColumns,
    edges: EdgeColumns,
}

/// An edge read from a [`RoadNetwork`], with its endpoints: what petgraph's
/// edge references were, holding the edge by value.
#[derive(Debug, Clone, PartialEq)]
pub struct RoadEdgeRef {
    index: EdgeIndex,
    source: NodeIndex,
    target: NodeIndex,
    weight: RoadEdge,
}

impl RoadEdgeRef {
    pub fn id(&self) -> EdgeIndex {
        self.index
    }

    pub fn source(&self) -> NodeIndex {
        self.source
    }

    pub fn target(&self) -> NodeIndex {
        self.target
    }

    pub fn weight(&self) -> &RoadEdge {
        &self.weight
    }

    pub fn into_weight(self) -> RoadEdge {
        self.weight
    }
}

impl RoadNetwork {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn node_count(&self) -> usize {
        self.nodes.lat_lng.len()
    }

    pub fn edge_count(&self) -> usize {
        self.edges.source.len()
    }

    pub fn reserve_exact_nodes(&mut self, additional: usize) {
        let nodes = &mut self.nodes;
        nodes.id.reserve_exact(additional);
        nodes.lat_lng.reserve_exact(additional);
        nodes.source_node_id.reserve_exact(additional);
        nodes.kind.reserve_exact(additional);
        nodes.barrier_class.reserve_exact(additional);
    }

    pub fn reserve_exact_edges(&mut self, additional: usize) {
        let edges = &mut self.edges;
        edges.source.reserve_exact(additional);
        edges.target.reserve_exact(additional);
        edges.distance_m.reserve_exact(additional);
        edges.base_cost.reserve_exact(additional);
        edges.flags.reserve_exact(additional);
    }

    /// Releases spare capacity in every column.
    pub fn shrink_to_fit(&mut self) {
        let nodes = &mut self.nodes;
        nodes.id.shrink_to_fit();
        nodes.lat_lng.shrink_to_fit();
        nodes.source_node_id.shrink_to_fit();
        nodes.component.shrink_to_fit();
        nodes.incoming.shrink_to_fit();
        nodes.outgoing.shrink_to_fit();
        nodes.incident.shrink_to_fit();
        nodes.kind.shrink_to_fit();
        nodes.roundabout_entries.shrink_to_fit();
        nodes.roundabout_exits.shrink_to_fit();
        nodes.barrier_class.shrink_to_fit();
        nodes.barrier_access.shrink_to_fit();
        let edges = &mut self.edges;
        edges.source.shrink_to_fit();
        edges.target.shrink_to_fit();
        edges.edge_id.shrink_to_fit();
        edges.shared_edge_info_id.shrink_to_fit();
        edges.distance_m.shrink_to_fit();
        edges.base_cost.shrink_to_fit();
        edges.mobility_mode_mask.shrink_to_fit();
        edges.osm_forward_mode_mask.shrink_to_fit();
        edges.osm_reverse_mode_mask.shrink_to_fit();
        edges.flags.shrink_to_fit();
        edges.segment_index.shrink_to_fit();
        edges.start_heading.shrink_to_fit();
        edges.end_heading.shrink_to_fit();
        edges.inferred_speed.shrink_to_fit();
        edges.opposing_edge_id.shrink_to_fit();
    }

    /// Bytes the columns hold, for reporting.
    pub fn heap_bytes(&self) -> usize {
        let nodes = &self.nodes;
        let edges = &self.edges;
        nodes.id.heap_bytes()
            + nodes.lat_lng.capacity() * 16
            + nodes.source_node_id.heap_bytes()
            + nodes.component.heap_bytes()
            + nodes.incoming.heap_bytes()
            + nodes.outgoing.heap_bytes()
            + nodes.incident.heap_bytes()
            + nodes.kind.capacity()
            + nodes.roundabout_entries.heap_bytes()
            + nodes.roundabout_exits.heap_bytes()
            + nodes.barrier_class.capacity()
            + nodes.barrier_access.heap_bytes()
            + edges.source.capacity() * 4
            + edges.target.capacity() * 4
            + edges.edge_id.heap_bytes()
            + edges.shared_edge_info_id.heap_bytes()
            + edges.distance_m.capacity() * 4
            + edges.base_cost.capacity() * 4
            + edges.mobility_mode_mask.heap_bytes()
            + edges.osm_forward_mode_mask.heap_bytes()
            + edges.osm_reverse_mode_mask.heap_bytes()
            + edges.flags.capacity()
            + edges.segment_index.heap_bytes()
            + edges.start_heading.heap_bytes()
            + edges.end_heading.heap_bytes()
            + edges.inferred_speed.heap_bytes()
            + edges.opposing_edge_id.heap_bytes()
    }

    pub fn add_node(&mut self, node: RoadNode) -> NodeIndex {
        let index = self.node_count();
        assert!(index < u32::MAX as usize, "the graph exceeds u32 indexing");
        let nodes = &mut self.nodes;
        nodes.id.push(node.id as u64);
        nodes.lat_lng.push((node.lat, node.lng));
        nodes
            .source_node_id
            .push(node.source_node_id.map(|id| id as u64));
        nodes.component.push(some_code(node.component_id));
        nodes.incoming.push(u64::from(node.incoming_edge_count));
        nodes.outgoing.push(u64::from(node.outgoing_edge_count));
        nodes.incident.push(u64::from(node.incident_edge_count));
        nodes.kind.push(node_kind_byte(&node));
        nodes
            .roundabout_entries
            .push(u64::from(node.roundabout_entry_count));
        nodes
            .roundabout_exits
            .push(u64::from(node.roundabout_exit_count));
        nodes.barrier_class.push(node.barrier_class);
        nodes.barrier_access.push(u64::from(node.barrier_access));
        NodeIndex::new(index)
    }

    pub fn add_edge(&mut self, source: NodeIndex, target: NodeIndex, edge: RoadEdge) -> EdgeIndex {
        let index = self.edge_count();
        assert!(index < u32::MAX as usize, "the graph exceeds u32 indexing");
        assert!(
            source.index() < self.node_count() && target.index() < self.node_count(),
            "an edge's endpoint is not a node of the graph"
        );
        let edges = &mut self.edges;
        edges.source.push(source.index() as u32);
        edges.target.push(target.index() as u32);
        edges.edge_id.push(edge.edge_id);
        edges.shared_edge_info_id.push(edge.shared_edge_info_id);
        edges.distance_m.push(edge.distance_m);
        edges.base_cost.push(edge.base_cost);
        edges
            .mobility_mode_mask
            .push(u64::from(edge.mobility_mode_mask));
        edges
            .osm_forward_mode_mask
            .push(u64::from(edge.osm_forward_mode_mask));
        edges
            .osm_reverse_mode_mask
            .push(u64::from(edge.osm_reverse_mode_mask));
        edges.flags.push(edge_flags_byte(&edge));
        edges.segment_index.push(u64::from(edge.segment_index));
        edges.start_heading.push(some_code(edge.start_heading_deg));
        edges.end_heading.push(some_code(edge.end_heading_deg));
        edges
            .inferred_speed
            .push(some_code(edge.inferred_speed_kph));
        edges.opposing_edge_id.push(edge.opposing_edge_id);
        EdgeIndex::new(index)
    }

    /// The node at `index`; panics past the end, as petgraph's indexing did.
    pub fn node(&self, index: NodeIndex) -> RoadNode {
        let at = index.index();
        let nodes = &self.nodes;
        let (lat, lng) = nodes.lat_lng[at];
        let kind = nodes.kind[at];
        RoadNode {
            id: nodes.id.get(at) as i64,
            lat,
            lng,
            source_node_id: nodes.source_node_id.get(at).map(|id| id as i64),
            component_id: code_u32(nodes.component.get(at)),
            incoming_edge_count: nodes.incoming.get(at) as u16,
            outgoing_edge_count: nodes.outgoing.get(at) as u16,
            incident_edge_count: nodes.incident.get(at) as u16,
            junction_kind: junction_from_code(kind & 0b111),
            highest_incident_road_type: (kind >> 4 != 0)
                .then(|| road_type_from_code((kind >> 4) - 1)),
            roundabout_entry_count: nodes.roundabout_entries.get(at) as u16,
            roundabout_exit_count: nodes.roundabout_exits.get(at) as u16,
            is_complex_junction: kind & (1 << 3) != 0,
            barrier_class: nodes.barrier_class[at],
            barrier_access: nodes.barrier_access.get(at) as u16,
        }
    }

    pub fn node_weight(&self, index: NodeIndex) -> Option<RoadNode> {
        (index.index() < self.node_count()).then(|| self.node(index))
    }

    /// Replaces the node at `index`.
    pub fn set_node(&mut self, index: NodeIndex, node: &RoadNode) {
        let at = index.index();
        let nodes = &mut self.nodes;
        nodes.id.set(at, node.id as u64);
        nodes.lat_lng[at] = (node.lat, node.lng);
        nodes
            .source_node_id
            .set(at, node.source_node_id.map(|id| id as u64));
        nodes.component.set(at, some_code(node.component_id));
        nodes.incoming.set(at, u64::from(node.incoming_edge_count));
        nodes.outgoing.set(at, u64::from(node.outgoing_edge_count));
        nodes.incident.set(at, u64::from(node.incident_edge_count));
        nodes.kind[at] = node_kind_byte(node);
        nodes
            .roundabout_entries
            .set(at, u64::from(node.roundabout_entry_count));
        nodes
            .roundabout_exits
            .set(at, u64::from(node.roundabout_exit_count));
        nodes.barrier_class[at] = node.barrier_class;
        nodes.barrier_access.set(at, u64::from(node.barrier_access));
    }

    /// Changes the node at `index` in place: read, changed by `update`, and
    /// written back.
    pub fn update_node<T>(
        &mut self,
        index: NodeIndex,
        update: impl FnOnce(&mut RoadNode) -> T,
    ) -> T {
        let mut node = self.node(index);
        let result = update(&mut node);
        self.set_node(index, &node);
        result
    }

    /// The edge at `index`; panics past the end.
    pub fn edge(&self, index: EdgeIndex) -> RoadEdge {
        let at = index.index();
        let edges = &self.edges;
        let flags = edges.flags[at];
        RoadEdge {
            edge_id: edges.edge_id.get(at),
            shared_edge_info_id: edges.shared_edge_info_id.get(at),
            distance_m: edges.distance_m[at],
            base_cost: edges.base_cost[at],
            mobility_mode_mask: edges.mobility_mode_mask.get(at) as ModeMask,
            osm_forward_mode_mask: edges.osm_forward_mode_mask.get(at) as ModeMask,
            osm_reverse_mode_mask: edges.osm_reverse_mode_mask.get(at) as ModeMask,
            traversal_direction: if flags & EDGE_BACKWARD != 0 {
                TraversalDirection::Backward
            } else {
                TraversalDirection::Forward
            },
            segment_index: edges.segment_index.get(at) as u32,
            start_heading_deg: code_u16(edges.start_heading.get(at)),
            end_heading_deg: code_u16(edges.end_heading.get(at)),
            inferred_speed_kph: code_u16(edges.inferred_speed.get(at)),
            opposing_edge_id: edges.opposing_edge_id.get(at),
            hierarchy_tier: tier_from_code(flags >> 5),
            end_control: TrafficControl::from_code((flags >> 2) & 0b111),
            end_minor_approach: flags & EDGE_MINOR_APPROACH != 0,
        }
    }

    pub fn edge_weight(&self, index: EdgeIndex) -> Option<RoadEdge> {
        (index.index() < self.edge_count()).then(|| self.edge(index))
    }

    /// Replaces the edge at `index` (its endpoints stay).
    pub fn set_edge(&mut self, index: EdgeIndex, edge: &RoadEdge) {
        let at = index.index();
        let edges = &mut self.edges;
        edges.edge_id.set(at, edge.edge_id);
        edges.shared_edge_info_id.set(at, edge.shared_edge_info_id);
        edges.distance_m[at] = edge.distance_m;
        edges.base_cost[at] = edge.base_cost;
        edges
            .mobility_mode_mask
            .set(at, u64::from(edge.mobility_mode_mask));
        edges
            .osm_forward_mode_mask
            .set(at, u64::from(edge.osm_forward_mode_mask));
        edges
            .osm_reverse_mode_mask
            .set(at, u64::from(edge.osm_reverse_mode_mask));
        edges.flags[at] = edge_flags_byte(edge);
        edges.segment_index.set(at, u64::from(edge.segment_index));
        edges
            .start_heading
            .set(at, some_code(edge.start_heading_deg));
        edges.end_heading.set(at, some_code(edge.end_heading_deg));
        edges
            .inferred_speed
            .set(at, some_code(edge.inferred_speed_kph));
        edges.opposing_edge_id.set(at, edge.opposing_edge_id);
    }

    /// Changes the edge at `index` in place: read, changed by `update`, and
    /// written back.
    pub fn update_edge<T>(
        &mut self,
        index: EdgeIndex,
        update: impl FnOnce(&mut RoadEdge) -> T,
    ) -> T {
        let mut edge = self.edge(index);
        let result = update(&mut edge);
        self.set_edge(index, &edge);
        result
    }

    /// Removes the edge at `index` and returns it. As petgraph did, the last
    /// edge takes its index.
    pub fn remove_edge(&mut self, index: EdgeIndex) -> Option<RoadEdge> {
        let at = index.index();
        let removed = self.edge_weight(index)?;
        let edges = &mut self.edges;
        edges.source.swap_remove(at);
        edges.target.swap_remove(at);
        edges.edge_id.swap_remove(at);
        edges.shared_edge_info_id.swap_remove(at);
        edges.distance_m.swap_remove(at);
        edges.base_cost.swap_remove(at);
        edges.mobility_mode_mask.swap_remove(at);
        edges.osm_forward_mode_mask.swap_remove(at);
        edges.osm_reverse_mode_mask.swap_remove(at);
        edges.flags.swap_remove(at);
        edges.segment_index.swap_remove(at);
        edges.start_heading.swap_remove(at);
        edges.end_heading.swap_remove(at);
        edges.inferred_speed.swap_remove(at);
        edges.opposing_edge_id.swap_remove(at);
        Some(removed)
    }

    pub fn edge_endpoints(&self, index: EdgeIndex) -> Option<(NodeIndex, NodeIndex)> {
        let at = index.index();
        (at < self.edge_count()).then(|| {
            (
                NodeIndex::new(self.edges.source[at] as usize),
                NodeIndex::new(self.edges.target[at] as usize),
            )
        })
    }

    pub fn node_indices(&self) -> impl ExactSizeIterator<Item = NodeIndex> + use<> {
        (0..self.node_count()).map(NodeIndex::new)
    }

    pub fn edge_indices(&self) -> impl ExactSizeIterator<Item = EdgeIndex> + use<> {
        (0..self.edge_count()).map(EdgeIndex::new)
    }

    pub fn node_weights(&self) -> impl ExactSizeIterator<Item = RoadNode> + '_ {
        self.node_indices().map(|index| self.node(index))
    }

    pub fn edge_weights(&self) -> impl ExactSizeIterator<Item = RoadEdge> + '_ {
        self.edge_indices().map(|index| self.edge(index))
    }

    /// The edge at `index` with its endpoints.
    pub fn edge_ref(&self, index: EdgeIndex) -> RoadEdgeRef {
        let at = index.index();
        RoadEdgeRef {
            index,
            source: NodeIndex::new(self.edges.source[at] as usize),
            target: NodeIndex::new(self.edges.target[at] as usize),
            weight: self.edge(index),
        }
    }

    /// Every edge with its endpoints, in index order.
    pub fn edge_references(&self) -> impl ExactSizeIterator<Item = RoadEdgeRef> + '_ {
        self.edge_indices().map(|index| self.edge_ref(index))
    }

    /// An index of every node's edges in `direction` (outgoing: those leaving
    /// it), each node's in descending edge index: the order petgraph's
    /// `edges_directed` walked its links in.
    pub fn adjacency(&self, direction: Direction) -> Adjacency {
        let ends = match direction {
            Direction::Outgoing => &self.edges.source,
            Direction::Incoming => &self.edges.target,
        };
        let node_count = self.node_count();
        let mut start = vec![0u32; node_count + 1];
        for end in ends {
            start[*end as usize + 1] += 1;
        }
        for node in 0..node_count {
            start[node + 1] += start[node];
        }
        let mut cursor = start.clone();
        let mut edges = vec![0u32; ends.len()];
        for (edge, end) in ends.iter().enumerate().rev() {
            let slot = &mut cursor[*end as usize];
            edges[*slot as usize] = edge as u32;
            *slot += 1;
        }
        Adjacency { start, edges }
    }

    /// Edge sources by edge index.
    pub fn sources(&self) -> &[u32] {
        &self.edges.source
    }

    /// Edge targets by edge index.
    pub fn targets(&self) -> &[u32] {
        &self.edges.target
    }

    #[inline]
    pub fn source(&self, edge: usize) -> NodeIndex {
        NodeIndex::new(self.edges.source[edge] as usize)
    }

    #[inline]
    pub fn target(&self, edge: usize) -> NodeIndex {
        NodeIndex::new(self.edges.target[edge] as usize)
    }

    #[inline]
    pub fn edge_id(&self, edge: usize) -> u64 {
        self.edges.edge_id.get(edge)
    }

    #[inline]
    pub fn edge_shared_info_id(&self, edge: usize) -> u64 {
        self.edges.shared_edge_info_id.get(edge)
    }

    #[inline]
    pub fn edge_distance_m(&self, edge: usize) -> f32 {
        self.edges.distance_m[edge]
    }

    #[inline]
    pub fn edge_mobility_mode_mask(&self, edge: usize) -> ModeMask {
        self.edges.mobility_mode_mask.get(edge) as ModeMask
    }

    #[inline]
    pub fn edge_traversal_direction(&self, edge: usize) -> TraversalDirection {
        if self.edges.flags[edge] & EDGE_BACKWARD != 0 {
            TraversalDirection::Backward
        } else {
            TraversalDirection::Forward
        }
    }

    #[inline]
    pub fn edge_segment_index(&self, edge: usize) -> u32 {
        self.edges.segment_index.get(edge) as u32
    }

    /// The OSM modes that may use the edge in its direction
    /// ([`RoadEdge::direction_osm_mode_mask`]).
    #[inline]
    pub fn edge_direction_osm_mode_mask(&self, edge: usize) -> ModeMask {
        match self.edge_traversal_direction(edge) {
            TraversalDirection::Forward => self.edges.osm_forward_mode_mask.get(edge) as ModeMask,
            TraversalDirection::Backward => self.edges.osm_reverse_mode_mask.get(edge) as ModeMask,
        }
    }

    #[inline]
    pub fn edge_opposing_edge_id(&self, edge: usize) -> Option<u64> {
        self.edges.opposing_edge_id.get(edge)
    }

    /// Sets an edge's OSM masks, and its mobility mask from the mask of its
    /// direction.
    pub fn set_edge_mode_masks(
        &mut self,
        edge: usize,
        osm_forward_mode_mask: ModeMask,
        osm_reverse_mode_mask: ModeMask,
        mobility_mode_mask: ModeMask,
    ) {
        let edges = &mut self.edges;
        edges
            .osm_forward_mode_mask
            .set(edge, u64::from(osm_forward_mode_mask));
        edges
            .osm_reverse_mode_mask
            .set(edge, u64::from(osm_reverse_mode_mask));
        edges
            .mobility_mode_mask
            .set(edge, u64::from(mobility_mode_mask));
    }

    /// Sets the values enhancement derives for an edge.
    pub fn set_edge_enhancement(
        &mut self,
        edge: usize,
        start_heading_deg: Option<u16>,
        end_heading_deg: Option<u16>,
        inferred_speed_kph: Option<u16>,
        hierarchy_tier: Option<RoadHierarchyTier>,
        end_minor_approach: bool,
    ) {
        let edges = &mut self.edges;
        edges.start_heading.set(edge, some_code(start_heading_deg));
        edges.end_heading.set(edge, some_code(end_heading_deg));
        edges
            .inferred_speed
            .set(edge, some_code(inferred_speed_kph));
        let flags = &mut edges.flags[edge];
        *flags = (*flags & !(EDGE_MINOR_APPROACH | 0b111 << 5))
            | u8::from(end_minor_approach) << 1
            | tier_code(hierarchy_tier) << 5;
    }

    pub fn set_edge_opposing_edge_id(&mut self, edge: usize, opposing_edge_id: Option<u64>) {
        self.edges.opposing_edge_id.set(edge, opposing_edge_id);
    }

    #[inline]
    pub fn node_lat_lng(&self, node: usize) -> (f64, f64) {
        self.nodes.lat_lng[node]
    }

    #[inline]
    pub fn node_id(&self, node: usize) -> i64 {
        self.nodes.id.get(node) as i64
    }

    #[inline]
    pub fn node_source_node_id(&self, node: usize) -> Option<i64> {
        self.nodes.source_node_id.get(node).map(|id| id as i64)
    }

    pub fn set_node_source_node_id(&mut self, node: usize, source_node_id: Option<i64>) {
        self.nodes
            .source_node_id
            .set(node, source_node_id.map(|id| id as u64));
    }

    #[inline]
    pub fn node_component_id(&self, node: usize) -> Option<u32> {
        code_u32(self.nodes.component.get(node))
    }

    #[inline]
    pub fn node_barrier(&self, node: usize) -> (u8, u16) {
        (
            self.nodes.barrier_class[node],
            self.nodes.barrier_access.get(node) as u16,
        )
    }

    pub fn set_node_barrier(&mut self, node: usize, barrier_class: u8, barrier_access: u16) {
        self.nodes.barrier_class[node] = barrier_class;
        self.nodes
            .barrier_access
            .set(node, u64::from(barrier_access));
    }

    #[inline]
    pub fn node_roundabout_exit_count(&self, node: usize) -> u16 {
        self.nodes.roundabout_exits.get(node) as u16
    }
}

/// A direction's edges per node ([`RoadNetwork::adjacency`]).
#[derive(Debug, Clone, Default)]
pub struct Adjacency {
    start: Vec<u32>,
    edges: Vec<u32>,
}

impl Adjacency {
    /// The edge indices of `node`'s edges in the index's direction.
    pub fn edges(&self, node: NodeIndex) -> &[u32] {
        let at = node.index();
        match (self.start.get(at), self.start.get(at + 1)) {
            (Some(start), Some(end)) => &self.edges[*start as usize..*end as usize],
            _ => &[],
        }
    }

    /// `node`'s edges in the index's direction, read from `network`.
    pub fn edge_refs<'a>(
        &'a self,
        network: &'a RoadNetwork,
        node: NodeIndex,
    ) -> impl Iterator<Item = RoadEdgeRef> + 'a {
        self.edges(node)
            .iter()
            .map(|&edge| network.edge_ref(EdgeIndex::new(edge as usize)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packed_widens_and_keeps_values() {
        let mut column = Packed::default();
        let values = [5u64, 6, 7, 7, 300, 2, 70_000, u64::MAX, 0];
        for (count, value) in values.iter().enumerate() {
            column.push(*value);
            for (index, expected) in values[..=count].iter().enumerate() {
                assert_eq!(column.get(index), *expected);
            }
        }
        column.set(1, 1 << 40);
        assert_eq!(column.get(1), 1 << 40);
        assert_eq!(column.get(8), 0);
        let mut iota = Packed::default();
        for value in 10..20 {
            iota.push(value);
        }
        assert!(matches!(iota, Packed::Iota { base: 10, len: 10 }));
        iota.set(3, 13);
        assert!(matches!(iota, Packed::Iota { .. }));
        iota.set(3, 4);
        assert_eq!(
            iota.iter().collect::<Vec<_>>(),
            [10, 11, 12, 4, 14, 15, 16, 17, 18, 19]
        );
        let mut wrapping = Packed::default();
        wrapping.push(u64::MAX);
        wrapping.push(0);
        assert_eq!((wrapping.get(0), wrapping.get(1)), (u64::MAX, 0));
    }

    #[test]
    fn option_packed_keeps_none_and_zero_apart() {
        let mut column = OptionPacked::default();
        let values = [Some(0), None, Some(u64::MAX), None, Some(3)];
        for value in values {
            column.push(value);
        }
        for (index, value) in values.iter().enumerate() {
            assert_eq!(column.get(index), *value);
        }
        column.set(1, Some(9));
        column.set(0, None);
        assert_eq!(column.get(1), Some(9));
        assert_eq!(column.get(0), None);
    }

    #[test]
    fn nodes_and_edges_read_back_as_written() {
        let mut network = RoadNetwork::new();
        let nodes = [
            RoadNode {
                id: 7,
                lat: 50.1,
                lng: 19.9,
                source_node_id: Some(12_345_678_901),
                component_id: Some(0),
                incoming_edge_count: 300,
                outgoing_edge_count: 1,
                incident_edge_count: 301,
                junction_kind: RoadJunctionKind::Complex,
                highest_incident_road_type: Some(RoadType::Other),
                roundabout_entry_count: 2,
                roundabout_exit_count: 3,
                is_complex_junction: true,
                barrier_class: 4,
                barrier_access: 0x1ff,
            },
            RoadNode {
                id: -1,
                lat: f64::from_bits(0x7ff8_0000_0000_0001),
                lng: -0.0,
                ..RoadNode::default()
            },
        ];
        for node in &nodes {
            network.add_node(node.clone());
        }
        let edge = RoadEdge {
            edge_id: 1 << 40,
            shared_edge_info_id: 3,
            distance_m: 12.5,
            base_cost: f32::NAN,
            mobility_mode_mask: u32::MAX,
            osm_forward_mode_mask: 5,
            osm_reverse_mode_mask: 0,
            traversal_direction: TraversalDirection::Backward,
            segment_index: 70_000,
            start_heading_deg: Some(0),
            end_heading_deg: Some(u16::MAX),
            inferred_speed_kph: None,
            opposing_edge_id: Some(0),
            hierarchy_tier: Some(RoadHierarchyTier::Highway),
            end_control: TrafficControl::TrafficSignals,
            end_minor_approach: true,
        };
        network.add_edge(NodeIndex::new(1), NodeIndex::new(0), edge.clone());
        network.add_edge(NodeIndex::new(0), NodeIndex::new(1), RoadEdge::default());
        let read = network.edge(EdgeIndex::new(0));
        assert_eq!(read.base_cost.to_bits(), edge.base_cost.to_bits());
        assert_eq!(
            RoadEdge {
                base_cost: 0.0,
                ..read
            },
            RoadEdge {
                base_cost: 0.0,
                ..edge.clone()
            }
        );
        assert_eq!(network.edge(EdgeIndex::new(1)), RoadEdge::default());
        assert_eq!(network.node(NodeIndex::new(0)), nodes[0]);
        let second = network.node(NodeIndex::new(1));
        assert_eq!(second.lat.to_bits(), nodes[1].lat.to_bits());
        assert_eq!(second.lng.to_bits(), nodes[1].lng.to_bits());
        assert_eq!(second.id, -1);
        network.update_edge(EdgeIndex::new(1), |edge| {
            edge.opposing_edge_id = Some(1 << 40)
        });
        assert_eq!(network.edge_opposing_edge_id(1), Some(1 << 40));
        let out = network.adjacency(Direction::Outgoing);
        assert_eq!(out.edges(NodeIndex::new(0)), [1]);
        assert_eq!(out.edges(NodeIndex::new(1)), [0]);
    }

    #[test]
    fn adjacency_lists_newest_edge_first() {
        let mut network = RoadNetwork::new();
        for _ in 0..3 {
            network.add_node(RoadNode::default());
        }
        let (a, b, c) = (NodeIndex::new(0), NodeIndex::new(1), NodeIndex::new(2));
        for (from, to) in [(a, b), (a, c), (b, a), (a, a)] {
            network.add_edge(from, to, RoadEdge::default());
        }
        let mut petgraph = Graph::<(), ()>::new();
        for _ in 0..3 {
            petgraph.add_node(());
        }
        for (from, to) in [(a, b), (a, c), (b, a), (a, a)] {
            petgraph.add_edge(from, to, ());
        }
        for direction in [Direction::Outgoing, Direction::Incoming] {
            let adjacency = network.adjacency(direction);
            for node in [a, b, c] {
                let expected = petgraph
                    .edges_directed(node, direction)
                    .map(|edge| edge.id().index() as u32)
                    .collect::<Vec<_>>();
                assert_eq!(adjacency.edges(node), expected, "{direction:?} {node:?}");
            }
        }
    }
}

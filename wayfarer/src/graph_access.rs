use std::collections::{BTreeMap, HashMap};
#[cfg(test)]
use std::collections::{BTreeSet, VecDeque};
use std::sync::{Arc, OnceLock};

#[cfg(test)]
use petgraph::Direction;
use petgraph::graph::{EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use rstar::{AABB, RTree, RTreeObject};
use serde::{Deserialize, Serialize};
use wayfarer_package::{
    MobilityMode, ModeMask, RoadEdge, RoadNode, SharedEdgeInfo, TraversalDirection,
};
use wayfarer_road_graph::{ResolvedEdgeRestriction, RoadGraph};

#[cfg(test)]
use crate::spatial::{directed_geometry, lat_lng_envelope_for_radius};
use crate::types::{GeoPoint, MobilityProfile};

#[derive(Debug, Clone, Copy)]
pub struct EdgeRecord<'a> {
    pub edge: &'a RoadEdge,
    pub source: &'a RoadNode,
    pub target: &'a RoadNode,
    pub source_index: NodeIndex,
    pub target_index: NodeIndex,
    pub raw_edge_index: usize,
    pub shared_info: &'a SharedEdgeInfo,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub(crate) struct IndexedSegment {
    pub edge_id: u64,
    start: GeoPoint,
    end: GeoPoint,
}

impl RTreeObject for IndexedSegment {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(
            [
                self.start.lng.min(self.end.lng),
                self.start.lat.min(self.end.lat),
            ],
            [
                self.start.lng.max(self.end.lng),
                self.start.lat.max(self.end.lat),
            ],
        )
    }
}

impl EdgeRecord<'_> {
    /// An owned copy, as routing reads records.
    pub fn to_view_edge(&self) -> crate::graph_view::GraphViewEdge {
        crate::graph_view::GraphViewEdge {
            edge: self.edge.clone(),
            source: self.source.clone(),
            target: self.target.clone(),
            source_index: self.source_index.index() as u32,
            target_index: self.target_index.index() as u32,
            raw_edge_index: self.raw_edge_index,
            shared_info: self.shared_info.clone(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct RouteGraphIndex {
    node_by_id: Vec<(i64, usize)>,
    edge_by_id: Vec<(u64, usize)>,
    edge_metadata_by_raw_edge_index: Vec<Option<RouteGraphIndexedEdge>>,
    #[cfg_attr(not(test), allow(dead_code))]
    opposing_edge_by_id: HashMap<u64, u64>,
    shared_info_by_id: HashMap<u64, usize>,
    /// The graph's shared infos whole: the reference costs read them as one
    /// `SharedEdgeInfo` each.
    shared_infos: Vec<SharedEdgeInfo>,
    #[cfg_attr(not(test), allow(dead_code))]
    component_size_by_id: HashMap<u32, usize>,
    /// Spatial index over every geometry window of every directed edge, used
    /// only to snap a point or envelope onto candidate edges.
    ///
    /// Built on first use rather than in [`Self::build`]. A package build never
    /// queries it — it reads pages and adjacency, then drops the index — so
    /// eagerly bulk-loading roughly 10M segments cost about
    /// 400 MB of a region build's peak for nothing. It is derived from the road
    /// graph and is rebuilt on demand.
    #[cfg_attr(not(test), allow(dead_code))]
    segment_index: OnceLock<RTree<IndexedSegment>>,
    simple_restrictions: BTreeMap<(u64, u64), Vec<ResolvedSimpleRestriction>>,
    complex_restrictions_by_to_edge: BTreeMap<u64, Vec<ResolvedComplexRestriction>>,
    outgoing_raw_adjacency: RouteGraphRawAdjacency,
    incoming_raw_adjacency: RouteGraphRawAdjacency,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
struct RouteGraphIndexedEdge {
    edge_id: u64,
    source_raw_node_index: usize,
    target_raw_node_index: usize,
    shared_info_raw_index: usize,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub(crate) struct RouteGraphRawAdjacency {
    pub(crate) offsets: Vec<usize>,
    pub(crate) edge_indexes: Vec<usize>,
}

impl RouteGraphRawAdjacency {
    fn get(&self, raw_node_index: usize) -> Option<&[usize]> {
        let start = *self.offsets.get(raw_node_index)?;
        let end = *self.offsets.get(raw_node_index + 1)?;
        self.edge_indexes.get(start..end)
    }

    /// Outgoing and incoming adjacency, packed in one pass over the edge list.
    ///
    /// Both come out together because they need the same two walks. The shape
    /// this replaces was a `Vec<Vec<usize>>` per direction — two heap
    /// allocations per node, grown by push — that was flattened into this form
    /// and then discarded, and each node's list was sorted by a comparator that
    /// read the sort key back out of the graph on *every comparison* rather
    /// than once per edge.
    ///
    /// Every edge is placed, including edges whose shared info did not resolve:
    /// adjacency is the graph's own topology, and dropping an edge here would
    /// silently disconnect it from the search.
    fn both_directions(graph: &RoadGraph) -> (Self, Self) {
        let mut edge_id_by_raw_edge_index = Vec::<u64>::new();
        let mut node_slot_count = graph.graph.node_count();
        for edge_ref in graph.graph.edge_references() {
            let raw_edge_index = edge_ref.id().index();
            if edge_id_by_raw_edge_index.len() <= raw_edge_index {
                edge_id_by_raw_edge_index.resize(raw_edge_index + 1, 0);
            }
            edge_id_by_raw_edge_index[raw_edge_index] = edge_ref.weight().edge_id;
            node_slot_count = node_slot_count
                .max(edge_ref.source().index() + 1)
                .max(edge_ref.target().index() + 1);
        }

        let mut outgoing_offsets = vec![0usize; node_slot_count + 1];
        let mut incoming_offsets = vec![0usize; node_slot_count + 1];
        for edge_ref in graph.graph.edge_references() {
            outgoing_offsets[edge_ref.source().index() + 1] += 1;
            incoming_offsets[edge_ref.target().index() + 1] += 1;
        }
        for slot in 1..=node_slot_count {
            outgoing_offsets[slot] += outgoing_offsets[slot - 1];
            incoming_offsets[slot] += incoming_offsets[slot - 1];
        }

        let edge_count = outgoing_offsets[node_slot_count];
        let mut outgoing_edge_indexes = vec![0usize; edge_count];
        let mut incoming_edge_indexes = vec![0usize; incoming_offsets[node_slot_count]];
        let mut outgoing_cursor = outgoing_offsets.clone();
        let mut incoming_cursor = incoming_offsets.clone();
        for edge_ref in graph.graph.edge_references() {
            let raw_edge_index = edge_ref.id().index();
            let source = edge_ref.source().index();
            outgoing_edge_indexes[outgoing_cursor[source]] = raw_edge_index;
            outgoing_cursor[source] += 1;
            let target = edge_ref.target().index();
            incoming_edge_indexes[incoming_cursor[target]] = raw_edge_index;
            incoming_cursor[target] += 1;
        }

        for slot in 0..node_slot_count {
            let sort_by_edge_id = |indexes: &mut Vec<usize>, offsets: &[usize]| {
                indexes[offsets[slot]..offsets[slot + 1]].sort_unstable_by_key(|raw_edge_index| {
                    edge_id_by_raw_edge_index[*raw_edge_index]
                });
            };
            sort_by_edge_id(&mut outgoing_edge_indexes, &outgoing_offsets);
            sort_by_edge_id(&mut incoming_edge_indexes, &incoming_offsets);
        }

        (
            Self {
                offsets: outgoing_offsets,
                edge_indexes: outgoing_edge_indexes,
            },
            Self {
                offsets: incoming_offsets,
                edge_indexes: incoming_edge_indexes,
            },
        )
    }
}

fn node_id_index_from_graph(graph: &RoadGraph) -> Vec<(i64, usize)> {
    let mut index = graph
        .graph
        .node_indices()
        .map(|node_index| (graph.graph[node_index].id, node_index.index()))
        .collect::<Vec<_>>();
    index.sort_unstable_by_key(|(node_id, _)| *node_id);
    index
}

fn edge_id_index_from_graph(graph: &RoadGraph) -> Vec<(u64, usize)> {
    let mut index = graph
        .graph
        .edge_references()
        .map(|edge_ref| (edge_ref.weight().edge_id, edge_ref.id().index()))
        .collect::<Vec<_>>();
    index.sort_unstable_by_key(|(edge_id, _)| *edge_id);
    index
}

fn lookup_raw_node_index(index: &[(i64, usize)], node_id: i64) -> Option<usize> {
    index
        .binary_search_by_key(&node_id, |(candidate_id, _)| *candidate_id)
        .ok()
        .map(|position| index[position].1)
}

fn lookup_raw_edge_index(index: &[(u64, usize)], edge_id: u64) -> Option<usize> {
    index
        .binary_search_by_key(&edge_id, |(candidate_id, _)| *candidate_id)
        .ok()
        .map(|position| index[position].1)
}

impl RouteGraphIndex {
    pub fn build(graph: &RoadGraph) -> Self {
        let shared_info_by_id = graph
            .shared_edge_infos
            .iter()
            .enumerate()
            .map(|(index, info)| (info.shared_edge_info_id, index))
            .collect::<HashMap<_, _>>();
        let edge_by_id = edge_id_index_from_graph(graph);
        let mut edge_metadata_by_raw_edge_index = Vec::<Option<RouteGraphIndexedEdge>>::new();
        for edge_ref in graph.graph.edge_references() {
            let raw_edge_index = edge_ref.id().index();
            if edge_metadata_by_raw_edge_index.len() <= raw_edge_index {
                edge_metadata_by_raw_edge_index.resize(raw_edge_index + 1, None);
            }
            let edge = edge_ref.weight();
            let Some(shared_info_raw_index) =
                shared_info_by_id.get(&edge.shared_edge_info_id).copied()
            else {
                continue;
            };
            edge_metadata_by_raw_edge_index[raw_edge_index] = Some(RouteGraphIndexedEdge {
                edge_id: edge.edge_id,
                source_raw_node_index: edge_ref.source().index(),
                target_raw_node_index: edge_ref.target().index(),
                shared_info_raw_index,
            });
        }
        let component_size_by_id = graph
            .graph
            .node_weights()
            .filter_map(|node| node.component_id)
            .fold(HashMap::<u32, usize>::new(), |mut counts, component_id| {
                *counts.entry(component_id).or_default() += 1;
                counts
            });

        let node_by_id = node_id_index_from_graph(graph);

        // Adjacency is built straight into its packed form, sorted by edge id
        // per node with the sort key read once per edge.
        let (outgoing_raw_adjacency, incoming_raw_adjacency) =
            RouteGraphRawAdjacency::both_directions(graph);
        let opposing_edge_by_id = opposing_edge_by_id(graph);

        Self {
            node_by_id,
            edge_by_id,
            edge_metadata_by_raw_edge_index,
            opposing_edge_by_id,
            shared_info_by_id,
            shared_infos: graph.shared_edge_infos.to_vec(),
            component_size_by_id,
            segment_index: OnceLock::new(),
            simple_restrictions: simple_restrictions(graph),
            complex_restrictions_by_to_edge: complex_restrictions_by_to_edge(graph),
            outgoing_raw_adjacency,
            incoming_raw_adjacency,
        }
    }

    pub fn raw_edge_index_for_edge_id(&self, edge_id: u64) -> Option<usize> {
        lookup_raw_edge_index(&self.edge_by_id, edge_id)
    }

    /// The segment index, building it from `graph` the first time it is asked
    /// for. `graph` must be the graph this index was built against.
    #[cfg(test)]
    pub(crate) fn segment_index(&self, graph: &RoadGraph) -> &RTree<IndexedSegment> {
        self.segment_index
            .get_or_init(|| build_segment_index(graph, &self.shared_info_by_id, &self.shared_infos))
    }
}

pub struct GraphAccess<'a> {
    graph: GraphAccessGraph<'a>,
    index: Arc<RouteGraphIndex>,
}

enum GraphAccessGraph<'a> {
    Borrowed(&'a RoadGraph),
    Owned(Arc<RoadGraph>),
}

impl<'a> GraphAccessGraph<'a> {
    fn as_ref(&self) -> &RoadGraph {
        match self {
            Self::Borrowed(graph) => graph,
            Self::Owned(graph) => graph.as_ref(),
        }
    }
}

impl<'a> GraphAccess<'a> {
    pub fn new(graph: &'a RoadGraph) -> Self {
        Self::from_index(graph, Arc::new(RouteGraphIndex::build(graph)))
    }

    pub fn from_index(graph: &'a RoadGraph, index: Arc<RouteGraphIndex>) -> Self {
        Self {
            graph: GraphAccessGraph::Borrowed(graph),
            index,
        }
    }

    pub fn graph(&self) -> &RoadGraph {
        self.graph.as_ref()
    }

    pub fn graph_id(&self) -> &str {
        &self.graph().graph_id
    }

    pub fn node_count(&self) -> usize {
        self.graph().node_count()
    }

    pub fn edge_count(&self) -> usize {
        self.graph().edge_count()
    }

    pub fn raw_node_count(&self) -> usize {
        self.graph().graph.node_count()
    }

    pub fn raw_edge_count(&self) -> usize {
        self.graph().graph.edge_count()
    }

    pub fn raw_node_index_for_node_id(&self, node_id: i64) -> Option<usize> {
        lookup_raw_node_index(&self.index.node_by_id, node_id)
    }

    pub fn node_by_raw_node_index(&self, raw_node_index: usize) -> Option<&RoadNode> {
        self.graph()
            .graph
            .node_weight(NodeIndex::new(raw_node_index))
    }

    pub fn node_id_for_raw_node_index(&self, raw_node_index: usize) -> Option<i64> {
        self.node_by_raw_node_index(raw_node_index)
            .map(|node| node.id)
    }

    pub fn raw_edge_index_for_edge_id(&self, edge_id: u64) -> Option<usize> {
        lookup_raw_edge_index(&self.index.edge_by_id, edge_id)
    }

    pub fn edge_record_by_raw_edge_index(&self, raw_edge_index: usize) -> Option<EdgeRecord<'_>> {
        self.edge_record(EdgeIndex::new(raw_edge_index))
    }

    pub fn raw_node_indexes_for_raw_edge_index(
        &self,
        raw_edge_index: usize,
    ) -> Option<(usize, usize)> {
        let metadata = self
            .index
            .edge_metadata_by_raw_edge_index
            .get(raw_edge_index)
            .and_then(|metadata| *metadata)?;
        Some((
            metadata.source_raw_node_index,
            metadata.target_raw_node_index,
        ))
    }

    pub fn outgoing_raw_edge_indexes_from_node_id(&self, node_id: i64) -> Option<Vec<usize>> {
        let node_index = lookup_raw_node_index(&self.index.node_by_id, node_id)?;
        Some(
            self.outgoing_raw_edge_indexes_from_raw_node_index(node_index)?
                .to_vec(),
        )
    }

    pub fn incoming_raw_edge_indexes_to_node_id(&self, node_id: i64) -> Option<Vec<usize>> {
        let node_index = lookup_raw_node_index(&self.index.node_by_id, node_id)?;
        Some(
            self.incoming_raw_edge_indexes_to_raw_node_index(node_index)?
                .to_vec(),
        )
    }

    pub fn is_ready(&self) -> bool {
        self.node_count() > 0
            && self.edge_count() > 0
            && self.has_supported_route_edges()
            && self.graph().graph.edge_weights().all(|edge| {
                edge.distance_m.is_finite()
                    && edge.distance_m > 0.0
                    && self
                        .index
                        .shared_info_by_id
                        .contains_key(&edge.shared_edge_info_id)
            })
    }

    pub fn has_supported_route_edges(&self) -> bool {
        self.graph()
            .graph
            .edge_weights()
            .any(|edge| edge.mobility_mode_mask & MobilityMode::ALL_GROUND != 0)
    }

    pub fn has_route_edges_for_profile(&self, profile: MobilityProfile) -> bool {
        let profile_mask = profile.mobility_mode_mask();
        self.graph()
            .graph
            .edge_weights()
            .any(|edge| edge.mobility_mode_mask & profile_mask != 0)
    }

    pub fn node_by_id(&self, node_id: i64) -> Option<&RoadNode> {
        let raw_node_index = lookup_raw_node_index(&self.index.node_by_id, node_id)?;
        self.graph()
            .graph
            .node_weight(NodeIndex::new(raw_node_index))
    }

    pub fn edge_by_id(&self, edge_id: u64) -> Option<EdgeRecord<'_>> {
        let raw_edge_index = lookup_raw_edge_index(&self.index.edge_by_id, edge_id)?;
        self.edge_record(EdgeIndex::new(raw_edge_index))
    }

    #[cfg(test)]
    pub(crate) fn opposing_edge_id(&self, edge_id: u64) -> Option<u64> {
        self.index.opposing_edge_by_id.get(&edge_id).copied()
    }

    pub fn outgoing_edges_from_node_id(&self, node_id: i64) -> Option<Vec<EdgeRecord<'_>>> {
        let node_index = lookup_raw_node_index(&self.index.node_by_id, node_id)?;
        Some(
            self.outgoing_raw_edge_indexes_from_raw_node_index(node_index)?
                .iter()
                .filter_map(|raw_edge_index| self.edge_record(EdgeIndex::new(*raw_edge_index)))
                .collect(),
        )
    }

    pub fn incoming_edges_to_node_id(&self, node_id: i64) -> Option<Vec<EdgeRecord<'_>>> {
        let node_index = lookup_raw_node_index(&self.index.node_by_id, node_id)?;
        Some(
            self.incoming_raw_edge_indexes_to_raw_node_index(node_index)?
                .iter()
                .filter_map(|raw_edge_index| self.edge_record(EdgeIndex::new(*raw_edge_index)))
                .collect(),
        )
    }
    pub(crate) fn outgoing_raw_edge_indexes_from_raw_node_index(
        &self,
        raw_node_index: usize,
    ) -> Option<&[usize]> {
        if raw_node_index >= self.raw_node_count() {
            return None;
        }
        Some(
            self.index
                .outgoing_raw_adjacency
                .get(raw_node_index)
                .unwrap_or(&[]),
        )
    }

    pub fn incoming_raw_edge_indexes_to_raw_node_index(
        &self,
        raw_node_index: usize,
    ) -> Option<&[usize]> {
        if raw_node_index >= self.raw_node_count() {
            return None;
        }
        Some(
            self.index
                .incoming_raw_adjacency
                .get(raw_node_index)
                .unwrap_or(&[]),
        )
    }

    #[cfg(test)]
    pub(crate) fn candidate_edges_near(
        &self,
        point: GeoPoint,
        max_distance_m: u32,
    ) -> Vec<EdgeRecord<'_>> {
        let (min_lat, min_lng, max_lat, max_lng) =
            lat_lng_envelope_for_radius(point, max_distance_m);
        let envelope = AABB::from_corners([min_lng, min_lat], [max_lng, max_lat]);
        let edge_ids = self
            .index
            .segment_index(self.graph.as_ref())
            .locate_in_envelope_intersecting(envelope)
            .map(|segment| segment.edge_id)
            .collect::<BTreeSet<_>>();
        edge_ids
            .into_iter()
            .filter_map(|edge_id| self.edge_by_id(edge_id))
            .collect()
    }

    #[cfg(test)]
    pub(crate) fn component_id_for_node(&self, node_id: i64) -> Option<u32> {
        self.node_by_id(node_id).and_then(|node| node.component_id)
    }

    #[cfg(test)]
    pub(crate) fn component_size_for_node(&self, node_id: i64) -> Option<usize> {
        let component_id = self.component_id_for_node(node_id)?;
        self.index.component_size_by_id.get(&component_id).copied()
    }

    #[cfg(test)]
    pub(crate) fn outbound_reach_edge_count(
        &self,
        node_id: i64,
        profile: MobilityProfile,
        max_edge_count: usize,
    ) -> Option<usize> {
        self.bounded_reach_edge_count(node_id, profile, max_edge_count, Direction::Outgoing)
    }

    #[cfg(test)]
    pub(crate) fn inbound_reach_edge_count(
        &self,
        node_id: i64,
        profile: MobilityProfile,
        max_edge_count: usize,
    ) -> Option<usize> {
        self.bounded_reach_edge_count(node_id, profile, max_edge_count, Direction::Incoming)
    }

    pub(crate) fn transition_restricted(
        &self,
        from_edge_id: u64,
        to_edge_id: u64,
        _via_node_id: i64,
        profile: MobilityProfile,
    ) -> bool {
        self.index
            .simple_restrictions
            .get(&(from_edge_id, to_edge_id))
            .is_some_and(|restrictions| {
                restrictions.iter().any(|restriction| {
                    // `via_node_id` holds the OSM via-node id while search node ids are
                    // graph indexes; the (from, to) edge pair already fixes the via node
                    // (the end of `from`), so the ids are not compared.
                    restriction.applies_to_mode(profile.mobility_mode_mask())
                })
            })
    }

    pub(crate) fn has_complex_restrictions_to_edge(
        &self,
        to_edge_id: u64,
        profile: MobilityProfile,
    ) -> bool {
        let profile_mask = profile.mobility_mode_mask();
        self.index
            .complex_restrictions_by_to_edge
            .get(&to_edge_id)
            .is_some_and(|restrictions| {
                restrictions
                    .iter()
                    .any(|restriction| restriction.applies_to_mode(profile_mask))
            })
    }

    pub(crate) fn complex_transition_restricted_for_path(
        &self,
        edge_path: &[u64],
        to_edge_id: u64,
        profile: MobilityProfile,
    ) -> bool {
        self.index
            .complex_restrictions_by_to_edge
            .get(&to_edge_id)
            .is_some_and(|restrictions| {
                restrictions.iter().any(|restriction| {
                    restriction.applies_to_mode(profile.mobility_mode_mask())
                        && edge_path.ends_with(&restriction.required_prefix)
                })
            })
    }

    /// How many distinct complex restrictions towards `to_edge_id` that apply
    /// to `profile` have a required prefix `edge_path` ends with (the SAR
    /// `turn_restrictions` penalty count).
    #[cfg(feature = "reference")]
    pub(crate) fn complex_restriction_count_for_path(
        &self,
        edge_path: &[u64],
        to_edge_id: u64,
        profile: MobilityProfile,
    ) -> usize {
        let Some(restrictions) = self.index.complex_restrictions_by_to_edge.get(&to_edge_id) else {
            return 0;
        };
        let mut prefixes = restrictions
            .iter()
            .filter(|restriction| {
                restriction.applies_to_mode(profile.mobility_mode_mask())
                    && edge_path.ends_with(&restriction.required_prefix)
            })
            .map(|restriction| restriction.required_prefix.as_slice())
            .collect::<Vec<_>>();
        prefixes.sort_unstable();
        prefixes.dedup();
        prefixes.len()
    }

    /// The required prefixes of every complex restriction that applies to
    /// `profile`.
    #[cfg(feature = "reference")]
    pub(crate) fn complex_restriction_prefixes(&self, profile: MobilityProfile) -> Vec<Vec<u64>> {
        let profile_mask = profile.mobility_mode_mask();
        self.index
            .complex_restrictions_by_to_edge
            .values()
            .flatten()
            .filter(|restriction| restriction.applies_to_mode(profile_mask))
            .map(|restriction| restriction.required_prefix.clone())
            .collect()
    }

    #[cfg(test)]
    fn bounded_reach_edge_count(
        &self,
        node_id: i64,
        profile: MobilityProfile,
        max_edge_count: usize,
        direction: Direction,
    ) -> Option<usize> {
        let start = NodeIndex::new(lookup_raw_node_index(&self.index.node_by_id, node_id)?);
        let profile_mask = profile.mobility_mode_mask();
        let mut queue = VecDeque::from([start]);
        let mut seen_nodes = BTreeSet::from([start]);
        let mut seen_edges = BTreeSet::<u64>::new();

        while let Some(node) = queue.pop_front() {
            for edge_ref in self.graph().graph.edges_directed(node, direction) {
                let edge = edge_ref.weight();
                if edge.mobility_mode_mask & profile_mask == 0 {
                    continue;
                }
                if seen_edges.insert(edge.edge_id) && seen_edges.len() >= max_edge_count {
                    return Some(seen_edges.len());
                }
                let next_node = match direction {
                    Direction::Outgoing => edge_ref.target(),
                    Direction::Incoming => edge_ref.source(),
                };
                if seen_nodes.insert(next_node) {
                    queue.push_back(next_node);
                }
            }
        }

        Some(seen_edges.len())
    }

    fn edge_record(&self, edge_index: EdgeIndex) -> Option<EdgeRecord<'_>> {
        let metadata = self
            .index
            .edge_metadata_by_raw_edge_index
            .get(edge_index.index())
            .and_then(|metadata| *metadata)?;
        let graph = self.graph();
        let edge = graph.graph.edge_weight(edge_index)?;
        if metadata.edge_id != edge.edge_id {
            return None;
        }
        let source_index = NodeIndex::new(metadata.source_raw_node_index);
        let target_index = NodeIndex::new(metadata.target_raw_node_index);
        let source = graph.graph.node_weight(source_index)?;
        let target = graph.graph.node_weight(target_index)?;
        let shared_info = self
            .index
            .shared_infos
            .get(metadata.shared_info_raw_index)?;
        if shared_info.shared_edge_info_id != edge.shared_edge_info_id {
            return None;
        }
        Some(EdgeRecord {
            edge,
            source,
            target,
            source_index,
            target_index,
            raw_edge_index: edge_index.index(),
            shared_info,
        })
    }
}

impl GraphAccess<'static> {
    pub fn from_owned(graph: Arc<RoadGraph>, index: Arc<RouteGraphIndex>) -> Self {
        Self {
            graph: GraphAccessGraph::Owned(graph),
            index,
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
struct ResolvedSimpleRestriction {
    via_node_id: i64,
    mobility_mode_mask: ModeMask,
}

impl ResolvedSimpleRestriction {
    fn applies_to_mode(self, profile_mask: ModeMask) -> bool {
        self.mobility_mode_mask == 0 || self.mobility_mode_mask & profile_mask != 0
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
struct ResolvedComplexRestriction {
    required_prefix: Vec<u64>,
    mobility_mode_mask: ModeMask,
}

impl ResolvedComplexRestriction {
    fn applies_to_mode(&self, profile_mask: ModeMask) -> bool {
        self.mobility_mode_mask == 0 || self.mobility_mode_mask & profile_mask != 0
    }
}

fn simple_restrictions(graph: &RoadGraph) -> BTreeMap<(u64, u64), Vec<ResolvedSimpleRestriction>> {
    let mut restrictions = BTreeMap::<(u64, u64), Vec<ResolvedSimpleRestriction>>::new();
    for restriction in &graph.resolved_restrictions {
        if let ResolvedEdgeRestriction::Simple {
            from_edge_id,
            to_edge_id,
            via_node_id,
            mobility_mode_mask,
            ..
        } = restriction
        {
            restrictions
                .entry((*from_edge_id, *to_edge_id))
                .or_default()
                .push(ResolvedSimpleRestriction {
                    via_node_id: *via_node_id,
                    mobility_mode_mask: *mobility_mode_mask,
                });
        }
    }
    restrictions
}

fn complex_restrictions_by_to_edge(
    graph: &RoadGraph,
) -> BTreeMap<u64, Vec<ResolvedComplexRestriction>> {
    let mut restrictions = BTreeMap::<u64, Vec<ResolvedComplexRestriction>>::new();
    for restriction in &graph.resolved_restrictions {
        if let ResolvedEdgeRestriction::Complex {
            from_edge_id,
            to_edge_id,
            via_edge_ids,
            mobility_mode_mask,
            ..
        } = restriction
        {
            let mut required_prefix = Vec::with_capacity(via_edge_ids.len() + 1);
            required_prefix.push(*from_edge_id);
            required_prefix.extend(via_edge_ids.iter().copied());
            restrictions
                .entry(*to_edge_id)
                .or_default()
                .push(ResolvedComplexRestriction {
                    required_prefix,
                    mobility_mode_mask: *mobility_mode_mask,
                });
        }
    }
    restrictions
}

/// Borrowed from the shared info: built for every edge and every successor
/// candidate, so owning its strings meant three allocations per comparison.
#[cfg(test)]
fn build_segment_index(
    graph: &RoadGraph,
    shared_info_by_id: &HashMap<u64, usize>,
    shared_infos: &[SharedEdgeInfo],
) -> RTree<IndexedSegment> {
    let mut segments = Vec::with_capacity(graph.edge_count());
    for edge in graph.graph.edge_weights() {
        let Some(info_index) = shared_info_by_id.get(&edge.shared_edge_info_id).copied() else {
            continue;
        };
        segments.extend(indexed_segments_from_parts(
            edge.edge_id,
            edge.traversal_direction,
            &shared_infos[info_index],
        ));
    }
    RTree::bulk_load(segments)
}

#[cfg(test)]
fn indexed_segments_from_parts(
    edge_id: u64,
    traversal_direction: TraversalDirection,
    info: &SharedEdgeInfo,
) -> Vec<IndexedSegment> {
    let geometry = directed_geometry(info, traversal_direction);
    geometry
        .windows(2)
        .map(|window| IndexedSegment {
            edge_id,
            start: window[0],
            end: window[1],
        })
        .collect()
}

fn opposing_edge_by_id(graph: &RoadGraph) -> HashMap<u64, u64> {
    let edge_keys = graph
        .graph
        .edge_weights()
        .map(|edge| {
            (
                (
                    edge.shared_edge_info_id,
                    edge.segment_index,
                    traversal_direction_code(edge.traversal_direction),
                ),
                edge.edge_id,
            )
        })
        .collect::<HashMap<_, _>>();

    graph
        .graph
        .edge_weights()
        .filter_map(|edge| {
            let opposite_key = (
                edge.shared_edge_info_id,
                edge.segment_index,
                traversal_direction_code(opposite_traversal_direction(edge.traversal_direction)),
            );
            edge_keys
                .get(&opposite_key)
                .copied()
                .map(|opposing_edge_id| (edge.edge_id, opposing_edge_id))
        })
        .collect()
}

fn opposite_traversal_direction(direction: TraversalDirection) -> TraversalDirection {
    match direction {
        TraversalDirection::Forward => TraversalDirection::Backward,
        TraversalDirection::Backward => TraversalDirection::Forward,
    }
}

fn traversal_direction_code(direction: TraversalDirection) -> u8 {
    match direction {
        TraversalDirection::Forward => 0,
        TraversalDirection::Backward => 1,
    }
}

#[cfg(test)]
mod tests {
    use wayfarer_package::{DirectionalAccess, MobilityMode, RoadType};
    use wayfarer_road_graph::{ResolvedEdgeRestriction, RoadGraphBuilder, RoutableWay};

    use super::*;
    use crate::types::MobilityProfile;

    #[test]
    fn complex_restriction_presence_is_profile_aware() {
        let mut graph = profile_restriction_graph();
        graph.set_resolved_restrictions(vec![ResolvedEdgeRestriction::Complex {
            relation_id: 42,
            restriction: "no_straight_on".to_string(),
            from_edge_id: 1,
            to_edge_id: 3,
            via_edge_ids: vec![2],
            osm_mode_mask: 0,
            mobility_mode_mask: MobilityMode::WHEELED,
        }]);

        let access = GraphAccess::new(&graph);

        assert!(access.has_complex_restrictions_to_edge(3, MobilityProfile::Wheeled));
        assert!(!access.has_complex_restrictions_to_edge(3, MobilityProfile::OnFoot));
        assert!(access.complex_transition_restricted_for_path(
            &[1, 2],
            3,
            MobilityProfile::Wheeled
        ));
        assert!(!access.complex_transition_restricted_for_path(
            &[1, 2],
            3,
            MobilityProfile::OnFoot
        ));
    }

    fn profile_restriction_graph() -> wayfarer_road_graph::RoadGraph {
        let mut builder = RoadGraphBuilder::new("restriction-profile");
        builder.push_way(test_way(1, &[(0.0, 0.0), (0.0, 0.001)]));
        builder.push_way(test_way(2, &[(0.0, 0.001), (0.0, 0.002)]));
        builder.push_way(test_way(3, &[(0.0, 0.002), (0.0, 0.003)]));
        builder.finish()
    }

    fn test_way(osm_way_id: i64, points: &[(f64, f64)]) -> RoutableWay {
        RoutableWay {
            osm_way_id,
            points: points.to_vec(),
            source_node_ids: Vec::new(),
            road_type: RoadType::Residential,
            directional_access: DirectionalAccess::Forward,
            name: None,
            ref_name: None,
            highway: Some("residential".into()),
            access: None,
            vehicle: None,
            motor_vehicle: None,
            motorcar: None,
            bicycle: None,
            foot: None,
            oneway_foot: None,
            oneway_bicycle: None,
            motorroad: None,
            service: None,
            surface: None,
            tracktype: None,
            smoothness: None,
            maxspeed: None,
            lanes: None,
            has_bridge: false,
            has_tunnel: false,
            has_ford: false,
            is_roundabout: false,
            is_link: false,
            is_destination_only: false,
            access_restricted: false,
            source_lineage_ids: Vec::new(),
            coverage_id: None,
            area: None,
            area_highway: None,
            layer: None,
            level: None,
            destination: None,
            destination_ref: None,
            destination_street: None,
            junction_ref: None,
            is_toll: false,
            is_ferry: false,
            has_gate: false,
            is_access_restricted: false,
            conditional_restrictions: Vec::new(),
            ..Default::default()
        }
    }
}

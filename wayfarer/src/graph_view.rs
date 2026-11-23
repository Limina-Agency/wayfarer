use std::borrow::Cow;
use std::sync::{Arc, OnceLock};

pub use wayfarer_package::GraphViewEdge;
use wayfarer_package::RoadNode;

use crate::package::RoutingPackage;
use crate::spatial::lat_lng_envelope_for_radius;
use crate::types::{CorrelatedLocation, GeoPoint, MobilityProfile};

/// The road network as the router reads it from a routing package: every
/// record, correlation candidate, junction count and restriction comes from
/// the package's sections. "Raw" indexes are the package's dense node and
/// edge indexes; ids are the road graph's.
pub struct PackageGraphView {
    graph: Arc<RoutingPackage>,
    /// Compact nodes sorted by node id, built on the first lookup by id.
    nodes_by_id: OnceLock<Vec<u32>>,
}

impl PackageGraphView {
    pub(crate) fn new(graph: Arc<RoutingPackage>) -> Self {
        Self {
            graph,
            nodes_by_id: OnceLock::new(),
        }
    }

    pub(crate) fn package(&self) -> &Arc<RoutingPackage> {
        &self.graph
    }

    pub(crate) fn request(&self) -> crate::package::request::RequestView<'_> {
        self.graph
            .file()
            .request_view()
            .expect("an opened package has request sections")
    }

    fn node_for_id(&self, node_id: i64) -> Option<u32> {
        let request = self.request();
        let sorted = self.nodes_by_id.get_or_init(|| {
            let mut nodes = (0..self.graph.counts().node_count as u32).collect::<Vec<_>>();
            nodes.sort_unstable_by_key(|node| (request.node_id(*node), *node));
            nodes
        });
        let at = sorted.partition_point(|node| request.node_id(*node) < node_id);
        sorted
            .get(at)
            .copied()
            .filter(|node| request.node_id(*node) == node_id)
    }

    fn edges_of_node(&self, node_id: i64, outgoing: bool) -> Option<Vec<GraphViewEdge>> {
        let node = self.node_for_id(node_id)? as usize;
        let view = self.graph.view();
        let request = self.request();
        let mut classes = crate::package::request::LegClassCache::default();
        let edges: Vec<u32> = if outgoing {
            (view.first_out[node]..view.first_out[node + 1]).collect()
        } else {
            view.in_edges(node).map(|(edge, _)| edge).collect()
        };
        Some(
            edges
                .into_iter()
                .filter_map(|edge| request.leg_record_cached(edge, &mut classes))
                .collect(),
        )
    }

    fn profile_mask_applies(&self, mask: u16, profile: MobilityProfile) -> bool {
        mask & self.graph.profile_bit(profile) != 0
    }
}

impl PackageGraphView {
    pub fn search_edge_by_id(&self, edge_id: u64) -> Option<GraphViewEdge> {
        self.edge_by_id(edge_id)
    }

    pub fn overlay_projection_backend(&self) -> &'static str {
        // The label of the former archive engines, kept so overlay
        // diagnostics stay unchanged.
        "package"
    }

    pub fn routing_package(&self) -> &RoutingPackage {
        &self.graph
    }

    pub fn graph_id(&self) -> &str {
        self.graph.graph_id()
    }

    pub fn raw_node_count(&self) -> usize {
        self.graph.counts().node_count
    }

    pub fn raw_edge_count(&self) -> usize {
        self.graph.counts().edge_count
    }

    pub fn node_by_raw_node_index(&self, raw_node_index: usize) -> Option<RoadNode> {
        (raw_node_index < self.graph.counts().node_count)
            .then(|| self.request().road_node(raw_node_index as u32))
    }

    pub fn raw_edge_index_for_edge_id(&self, edge_id: u64) -> Option<usize> {
        self.graph
            .view()
            .edge_for_id(edge_id)
            .map(|edge| edge as usize)
    }

    pub fn edge_record_by_raw_edge_index(&self, raw_edge_index: usize) -> Option<GraphViewEdge> {
        let edge = u32::try_from(raw_edge_index).ok()?;
        self.request().leg_record(edge)
    }

    pub fn edge_records_for_path(
        &self,
        edge_indexes: &[Option<usize>],
        edge_ids: &[u64],
    ) -> Vec<Option<GraphViewEdge>> {
        // Each attribute class is decoded once per path (consecutive edges
        // usually share a street's class).
        let request = self.request();
        let view = self.graph.view();
        let mut classes = crate::package::request::LegClassCache::default();
        edge_ids
            .iter()
            .enumerate()
            .map(|(index, edge_id)| {
                let edge = match edge_indexes.get(index).copied().flatten() {
                    Some(raw) => u32::try_from(raw).ok()?,
                    None => view.edge_for_id(*edge_id)?,
                };
                request.leg_record_cached(edge, &mut classes)
            })
            .collect()
    }

    pub fn outgoing_raw_edge_indexes_from_raw_node_index(
        &self,
        raw_node_index: usize,
    ) -> Option<Cow<'_, [usize]>> {
        let view = self.graph.view();
        (raw_node_index < view.node_count).then(|| {
            Cow::Owned(
                (view.first_out[raw_node_index]..view.first_out[raw_node_index + 1])
                    .map(|edge| edge as usize)
                    .collect(),
            )
        })
    }

    pub fn is_ready(&self) -> bool {
        self.graph.is_ready()
    }

    pub fn has_route_edges_for_profile(&self, profile: MobilityProfile) -> bool {
        self.graph.file().has_route_edges_for_profile(profile)
    }

    pub fn node_by_id(&self, node_id: i64) -> Option<RoadNode> {
        let node = self.node_for_id(node_id)?;
        Some(self.request().road_node(node))
    }

    pub fn edge_by_id(&self, edge_id: u64) -> Option<GraphViewEdge> {
        let edge = self.graph.view().edge_for_id(edge_id)?;
        self.request().leg_record(edge)
    }

    pub fn opposing_edge_id(&self, edge_id: u64) -> Option<u64> {
        let view = self.graph.view();
        let edge = view.edge_for_id(edge_id)?;
        self.request()
            .opposing_edge(edge)
            .map(|opposing| view.edge_id(opposing as usize))
    }

    pub fn outgoing_edges_from_node_id(&self, node_id: i64) -> Option<Vec<GraphViewEdge>> {
        self.edges_of_node(node_id, true)
    }

    pub fn incoming_edges_to_node_id(&self, node_id: i64) -> Option<Vec<GraphViewEdge>> {
        self.edges_of_node(node_id, false)
    }

    pub fn outgoing_shared_info_count_excluding(
        &self,
        node_id: i64,
        excluded_shared_info_ids: &[u64],
    ) -> Option<usize> {
        let node = self.node_for_id(node_id)?;
        Some(
            self.request()
                .node_edge_count(node, excluded_shared_info_ids, true),
        )
    }

    pub fn incoming_shared_info_count_excluding(
        &self,
        node_id: i64,
        excluded_shared_info_ids: &[u64],
    ) -> Option<usize> {
        let node = self.node_for_id(node_id)?;
        Some(
            self.request()
                .node_edge_count(node, excluded_shared_info_ids, false),
        )
    }

    pub fn junction_shared_info_counts(
        &self,
        raw_edge_index: Option<usize>,
        source_node_id: i64,
        outgoing_excluded_shared_info_ids: &[u64],
        incoming_excluded_shared_info_ids: &[u64],
    ) -> (Option<usize>, Option<usize>) {
        self.junction_shared_info_counts_in(
            &self.request(),
            raw_edge_index,
            source_node_id,
            outgoing_excluded_shared_info_ids,
            incoming_excluded_shared_info_ids,
        )
    }

    /// [`Self::junction_shared_info_counts`] with the request view built once
    /// by the caller (leg building calls it once per edge).
    pub(crate) fn junction_shared_info_counts_in(
        &self,
        request: &crate::package::request::RequestView<'_>,
        raw_edge_index: Option<usize>,
        source_node_id: i64,
        outgoing_excluded_shared_info_ids: &[u64],
        incoming_excluded_shared_info_ids: &[u64],
    ) -> (Option<usize>, Option<usize>) {
        let edge = raw_edge_index
            .and_then(|raw| u32::try_from(raw).ok())
            .filter(|edge| (*edge as usize) < self.graph.counts().edge_count);
        match edge {
            Some(edge) => (
                Some(request.junction_edge_count(edge, outgoing_excluded_shared_info_ids, true)),
                Some(request.junction_edge_count(edge, incoming_excluded_shared_info_ids, false)),
            ),
            None => (
                self.outgoing_shared_info_count_excluding(
                    source_node_id,
                    outgoing_excluded_shared_info_ids,
                ),
                self.incoming_shared_info_count_excluding(
                    source_node_id,
                    incoming_excluded_shared_info_ids,
                ),
            ),
        }
    }

    /// The branches guidance weighs at the junction between `previous` and
    /// `edge`: the signed angles (degrees, negative to the left) from the
    /// approach to every other outgoing edge `profile` may take, U-turns onto
    /// `previous`' road excluded, and from the arrival of every other incoming
    /// edge `profile` may use (not `edge`'s own road) to the departure along
    /// `edge`. Angles come from the edges' stored end headings. `view` is the
    /// package's sections, built once per leg by the caller.
    pub(crate) fn junction_branch_angles_in(
        &self,
        view: &wayfarer_package::format::Sections<'_>,
        previous: &GraphViewEdge,
        edge: &GraphViewEdge,
        profile: MobilityProfile,
    ) -> (Vec<i16>, Vec<i16>) {
        let node = edge.source_index as usize;
        let (Some(approach), Some(departure)) =
            (previous.edge.end_heading_deg, edge.edge.start_heading_deg)
        else {
            return (Vec::new(), Vec::new());
        };
        if node + 1 >= view.first_out.len() || node >= view.node_count {
            return (Vec::new(), Vec::new());
        }
        let bit = view.profile_bit(profile);
        let shared_of = |raw: usize| view.edge_shared_info[raw];
        let (previous_shared, edge_shared) = (
            shared_of(previous.raw_edge_index),
            shared_of(edge.raw_edge_index),
        );
        let usable = |raw: usize| view.edge_access.profiles(raw) & bit != 0;
        let branches = (view.first_out[node]..view.first_out[node + 1])
            .map(|raw| raw as usize)
            .filter(|raw| *raw != edge.raw_edge_index && shared_of(*raw) != previous_shared)
            .filter(|raw| usable(*raw))
            .filter_map(|raw| {
                let heading = view.heading_start[raw];
                (heading != crate::package::NO_HEADING).then(|| signed_angle(approach, heading))
            })
            .collect();
        let joining = view
            .in_edges(node)
            .map(|(raw, _)| raw as usize)
            .filter(|raw| *raw != previous.raw_edge_index && shared_of(*raw) != edge_shared)
            .filter(|raw| usable(*raw))
            .filter_map(|raw| {
                let heading = view.heading_end(raw);
                (heading != crate::package::NO_HEADING).then(|| signed_angle(heading, departure))
            })
            .collect();
        (branches, joining)
    }

    pub fn candidate_edges_near(&self, point: GeoPoint, max_distance_m: u32) -> Vec<GraphViewEdge> {
        let (min_lat, min_lng, max_lat, max_lng) =
            lat_lng_envelope_for_radius(point, max_distance_m);
        self.candidate_edges_in_envelope(min_lat, min_lng, max_lat, max_lng)
    }

    pub fn candidate_edges_in_envelope(
        &self,
        min_lat: f64,
        min_lng: f64,
        max_lat: f64,
        max_lng: f64,
    ) -> Vec<GraphViewEdge> {
        let request = self.request();
        let mut classes = crate::package::request::LegClassCache::default();
        request
            .correlation_edges_in_envelope(
                min_lat,
                min_lng,
                max_lat,
                max_lng,
                MobilityProfile::Wheeled,
            )
            .into_iter()
            .filter_map(|candidate| {
                let edge = self.graph.view().edge_for_id(candidate.edge_id)?;
                request.leg_record_cached(edge, &mut classes)
            })
            .collect()
    }

    pub fn component_id_for_node(&self, node_id: i64) -> Option<u32> {
        let node = self.node_for_id(node_id)?;
        self.request().component_id(node)
    }

    pub fn component_size_for_node(&self, node_id: i64) -> Option<usize> {
        let node = self.node_for_id(node_id)?;
        self.request().component_size(node)
    }

    /// The compact edge of a correlated location (see
    /// [`crate::types::CompactEdgeHint`]).
    pub(crate) fn candidate_edge(&self, candidate: &CorrelatedLocation) -> Option<u32> {
        self.graph
            .view()
            .edge_for_id_hinted(candidate.compact_edge.0, candidate.edge_id)
    }

    pub fn candidate_component_id(
        &self,
        candidate: &CorrelatedLocation,
        at_target: bool,
    ) -> Option<u32> {
        let request = self.request();
        match self.candidate_edge(candidate) {
            Some(edge) => request.component_id(if at_target {
                request.edge_target(edge)
            } else {
                request.edge_source(edge)
            }),
            None => self.component_id_for_node(if at_target {
                candidate.target_node_id
            } else {
                candidate.source_node_id
            }),
        }
    }

    pub fn outbound_reach_edge_count(
        &self,
        node_id: i64,
        profile: MobilityProfile,
        max_edge_count: usize,
    ) -> Option<usize> {
        let node = self.node_for_id(node_id)?;
        crate::package::request::ReachCounter::new(self.graph.file(), profile, max_edge_count)
            .count_from_node(node, true)
    }

    pub fn inbound_reach_edge_count(
        &self,
        node_id: i64,
        profile: MobilityProfile,
        max_edge_count: usize,
    ) -> Option<usize> {
        let node = self.node_for_id(node_id)?;
        crate::package::request::ReachCounter::new(self.graph.file(), profile, max_edge_count)
            .count_from_node(node, false)
    }

    pub fn transition_restricted(
        &self,
        from_edge_id: u64,
        to_edge_id: u64,
        _via_node_id: i64,
        profile: MobilityProfile,
    ) -> bool {
        let view = self.graph.view();
        let (Some(from), Some(to)) = (view.edge_for_id(from_edge_id), view.edge_for_id(to_edge_id))
        else {
            return false;
        };
        let start = view.simple_from.partition_point(|edge| *edge < from);
        let end = view.simple_from.partition_point(|edge| *edge <= from);
        (start..end).any(|row| {
            view.simple_to[row] == to
                && self.profile_mask_applies(view.simple_mask.get(row), profile)
        })
    }

    pub fn has_complex_restrictions_to_edge(
        &self,
        to_edge_id: u64,
        profile: MobilityProfile,
    ) -> bool {
        let view = self.graph.view();
        let Some(to) = view.edge_for_id(to_edge_id) else {
            return false;
        };
        let start = view.complex_to.partition_point(|edge| *edge < to);
        let end = view.complex_to.partition_point(|edge| *edge <= to);
        (start..end).any(|row| self.profile_mask_applies(view.complex_mask.get(row), profile))
    }

    pub fn complex_transition_restricted_for_path(
        &self,
        edge_path: &[u64],
        to_edge_id: u64,
        profile: MobilityProfile,
    ) -> bool {
        let view = self.graph.view();
        let Some(to) = view.edge_for_id(to_edge_id) else {
            return false;
        };
        let start = view.complex_to.partition_point(|edge| *edge < to);
        let end = view.complex_to.partition_point(|edge| *edge <= to);
        (start..end).any(|row| {
            if !self.profile_mask_applies(view.complex_mask.get(row), profile) {
                return false;
            }
            let prefix = &view.complex_prefix_pool[view.complex_prefix_start[row] as usize
                ..view.complex_prefix_start[row + 1] as usize];
            prefix.len() <= edge_path.len()
                && prefix
                    .iter()
                    .zip(&edge_path[edge_path.len() - prefix.len()..])
                    .all(|(edge, id)| view.edge_id(*edge as usize) == *id)
        })
    }
}

/// The signed turn (degrees in -180..=180, negative to the left) from heading
/// `from` to heading `to`.
fn signed_angle(from: u16, to: u16) -> i16 {
    let delta = (i32::from(to) - i32::from(from)).rem_euclid(360);
    (if delta > 180 { delta - 360 } else { delta }) as i16
}

//! The network's public vocabulary: inputs, topology, render lines, stats and
//! the built network.

use super::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SharedLineLayer {
    Roads,
    WaterLines,
    Railways,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum DirectionalAccessModel {
    #[default]
    Both,
    Forward,
    Backward,
    Closed,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RoadRoutingAttrs {
    pub road_class: String,
    #[serde(default)]
    pub directional_access: DirectionalAccessModel,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub access: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub surface: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tracktype: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub smoothness: Option<String>,
    #[serde(default)]
    pub has_ford: bool,
    #[serde(default)]
    pub has_tunnel: bool,
    #[serde(default)]
    pub access_restricted: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SharedLineInput {
    pub feature_id: i64,
    pub layer: SharedLineLayer,
    pub geometry: Vec<(f64, f64)>,
    #[serde(default)]
    pub node_chain: Vec<i64>,
    #[serde(default)]
    pub source_lineage_ids: Vec<i64>,
    /// Shared with every other edge carrying the same tags.
    ///
    /// A region's road network is over a million topology edges, and most of
    /// them repeat one of a few thousand tag lists. Owning a private `Vec` per
    /// edge made this the largest retained structure in a package build; a
    /// shared handle costs a pointer and lets render seeds clone it for free.
    #[serde(default)]
    pub attrs: Arc<[(String, String)]>,
    /// Raw OSM tags kept for *compile-time* consumers only.
    ///
    /// The road-surface buffer generator needs `highway`/`width`/`lanes` to size
    /// a carriageway, and the road-label derivation needs `name`/`ref`, but none
    /// of those belong in a rendered tile: they are recoverable from the source
    /// store by OSM id. Before this channel existed both read `attrs`, which
    /// silently made the render contract load-bearing for geometry generation —
    /// pruning a tile attribute emptied road labels and disabled carriageway
    /// buffering.
    #[serde(default)]
    pub compile_inputs: Arc<[(String, String)]>,
    pub min_zoom: u8,
    pub max_zoom: u8,
    pub sort_order: u8,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub routing: Option<Arc<RoadRoutingAttrs>>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SharedTopologyNode {
    pub id: u32,
    pub lat: f64,
    pub lng: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source_node_id: Option<i64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SharedTopologyEdge {
    pub edge_id: u32,
    pub feature_id: i64,
    pub layer: SharedLineLayer,
    pub geometry: Vec<(f64, f64)>,
    #[serde(default)]
    pub attrs: Arc<[(String, String)]>,
    /// Raw OSM tags kept for *compile-time* consumers only.
    ///
    /// The road-surface buffer generator needs `highway`/`width`/`lanes` to size
    /// a carriageway, and the road-label derivation needs `name`/`ref`, but none
    /// of those belong in a rendered tile: they are recoverable from the source
    /// store by OSM id. Before this channel existed both read `attrs`, which
    /// silently made the render contract load-bearing for geometry generation —
    /// pruning a tile attribute emptied road labels and disabled carriageway
    /// buffering.
    #[serde(default)]
    pub compile_inputs: Arc<[(String, String)]>,
    pub min_zoom: u8,
    pub max_zoom: u8,
    pub sort_order: u8,
    pub start_node: u32,
    pub end_node: u32,
    /// Shared by the edges of one input line.
    #[serde(default)]
    pub source_lineage_ids: Arc<[i64]>,
    /// Shared by every edge with the same routing attributes: a region's
    /// million road edges carry a few thousand distinct ones.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub routing: Option<Arc<RoadRoutingAttrs>>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SharedLineTopology {
    #[serde(default)]
    pub nodes: Vec<SharedTopologyNode>,
    #[serde(default)]
    pub edges: Vec<SharedTopologyEdge>,
}

impl SharedLineTopology {
    pub fn edges_for_layer(
        &self,
        layer: SharedLineLayer,
    ) -> impl Iterator<Item = &SharedTopologyEdge> {
        self.edges.iter().filter(move |edge| edge.layer == layer)
    }

    pub fn road_segments(&self) -> impl Iterator<Item = &SharedTopologyEdge> {
        self.edges
            .iter()
            .filter(|edge| edge.layer == SharedLineLayer::Roads && edge.routing.is_some())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SharedRenderLine {
    pub feature_id: i64,
    pub layer: SharedLineLayer,
    pub geometry: Vec<(f64, f64)>,
    #[serde(default)]
    pub attrs: Vec<(String, String)>,
    pub min_zoom: u8,
    pub max_zoom: u8,
    pub sort_order: u8,
    pub start_node: u32,
    pub end_node: u32,
    #[serde(default)]
    pub source_lineage_ids: Vec<i64>,
    #[serde(default)]
    pub topology_edge_ids: Vec<u32>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RenderNetworkProducts {
    #[serde(default)]
    pub render_lines: Vec<SharedRenderLine>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SharedLineNetworkStats {
    pub input_lines: usize,
    pub topology_nodes: usize,
    pub topology_edges: usize,
    #[serde(default)]
    pub topology_repair_input_road_lines: usize,
    #[serde(default)]
    pub topology_repair_checked_segment_pairs: usize,
    #[serde(default)]
    pub topology_repair_repaired_crossings: usize,
    #[serde(default)]
    pub topology_repair_inserted_topology_nodes: usize,
    #[serde(default)]
    pub topology_repair_split_edges: usize,
    #[serde(default)]
    pub topology_repair_skipped_endpoint_touches: usize,
    #[serde(default)]
    pub topology_repair_skipped_same_feature: usize,
    #[serde(default)]
    pub topology_repair_skipped_incompatible_plane: usize,
    pub render_lines: usize,
    pub render_lines_min_zoom_bumped: usize,
    pub render_duplicate_edges_removed: usize,
    pub render_stub_edges_bumped: usize,
    pub render_loops_removed: usize,
    pub render_stroke_pairs_formed: usize,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SharedLineNetwork {
    pub topology: SharedLineTopology,
    pub render: RenderNetworkProducts,
    #[serde(default)]
    pub stats: SharedLineNetworkStats,
}

impl SharedLineNetwork {
    pub fn render_lines_for_layer(
        &self,
        layer: SharedLineLayer,
    ) -> impl Iterator<Item = &SharedRenderLine> {
        self.render
            .render_lines
            .iter()
            .filter(move |line| line.layer == layer)
    }

    pub fn road_segments(&self) -> impl Iterator<Item = &SharedTopologyEdge> {
        self.topology.road_segments()
    }

    pub fn discard_render_products(&mut self) {
        self.render.render_lines.clear();
        self.render.render_lines.shrink_to_fit();
        self.stats.render_lines = 0;
    }
}

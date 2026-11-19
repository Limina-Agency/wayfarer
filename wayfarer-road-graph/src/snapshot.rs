//! Road graph snapshot files (`.mrg`): the graph, encoded with
//! [`wayfarer_package::codec`] after a little-endian
//! [`ROAD_GRAPH_SNAPSHOT_VERSION`], in a zstd container with a BLAKE3 checksum.
//! They cache graphs for the reference oracle and test fixtures; the route
//! compiler does not write them. Behind the `snapshot` feature, so builds
//! without it link neither zstd nor BLAKE3.

use super::*;
use std::io::{BufReader, Write};
use std::path::Path;
use wayfarer_package::codec;

const ROAD_GRAPH_ZSTD_LEVEL: i32 = 6;

#[derive(Debug, Clone, Serialize, Deserialize)]
struct RoadGraphSnapshotEdge {
    source: usize,
    target: usize,
    edge: SnapshotEdge,
}

/// A [`RoadEdge`] as snapshots store it: with the source lineage and coverage
/// id the graph now keeps on the edge's shared info, so the format stays
/// [`ROAD_GRAPH_SNAPSHOT_VERSION`] 1. Fields in snapshot order.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct SnapshotEdge {
    edge_id: u64,
    shared_edge_info_id: u64,
    distance_m: f32,
    base_cost: f32,
    mobility_mode_mask: ModeMask,
    osm_forward_mode_mask: ModeMask,
    osm_reverse_mode_mask: ModeMask,
    traversal_direction: TraversalDirection,
    segment_index: u32,
    source_lineage_ids: Vec<i64>,
    coverage_id: Option<String>,
    component_id: Option<u32>,
    start_heading_deg: Option<u16>,
    end_heading_deg: Option<u16>,
    inferred_speed_kph: Option<u16>,
    estimated_travel_time_s: Option<f32>,
    opposing_edge_id: Option<u64>,
    hierarchy_tier: Option<RoadHierarchyTier>,
    end_control: TrafficControl,
    end_minor_approach: bool,
}

impl SnapshotEdge {
    fn new(edge: &RoadEdge, info: Option<SharedInfoRef<'_>>) -> Self {
        Self {
            edge_id: edge.edge_id,
            shared_edge_info_id: edge.shared_edge_info_id,
            distance_m: edge.distance_m,
            base_cost: edge.base_cost,
            mobility_mode_mask: edge.mobility_mode_mask,
            osm_forward_mode_mask: edge.osm_forward_mode_mask,
            osm_reverse_mode_mask: edge.osm_reverse_mode_mask,
            traversal_direction: edge.traversal_direction,
            segment_index: edge.segment_index,
            source_lineage_ids: info.map_or_else(Vec::new, |info| info.source_lineage_ids.to_vec()),
            coverage_id: info.and_then(|info| info.coverage_id.map(|id| id.to_string())),
            component_id: edge.component_id,
            start_heading_deg: edge.start_heading_deg,
            end_heading_deg: edge.end_heading_deg,
            inferred_speed_kph: edge.inferred_speed_kph,
            estimated_travel_time_s: edge.estimated_travel_time_s,
            opposing_edge_id: edge.opposing_edge_id,
            hierarchy_tier: edge.hierarchy_tier,
            end_control: edge.end_control,
            end_minor_approach: edge.end_minor_approach,
        }
    }

    fn into_edge(self) -> RoadEdge {
        RoadEdge {
            edge_id: self.edge_id,
            shared_edge_info_id: self.shared_edge_info_id,
            distance_m: self.distance_m,
            base_cost: self.base_cost,
            mobility_mode_mask: self.mobility_mode_mask,
            osm_forward_mode_mask: self.osm_forward_mode_mask,
            osm_reverse_mode_mask: self.osm_reverse_mode_mask,
            traversal_direction: self.traversal_direction,
            segment_index: self.segment_index,
            component_id: self.component_id,
            start_heading_deg: self.start_heading_deg,
            end_heading_deg: self.end_heading_deg,
            inferred_speed_kph: self.inferred_speed_kph,
            estimated_travel_time_s: self.estimated_travel_time_s,
            opposing_edge_id: self.opposing_edge_id,
            hierarchy_tier: self.hierarchy_tier,
            end_control: self.end_control,
            end_minor_approach: self.end_minor_approach,
        }
    }
}

/// A [`RoadNode`] as snapshots store it: with its road type counts (the
/// graph's [`RoadGraph::node_road_type_counts`]). Fields in snapshot order.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct SnapshotNode {
    id: i64,
    lat: f64,
    lng: f64,
    source_node_id: Option<i64>,
    component_id: Option<u32>,
    incoming_edge_count: u16,
    outgoing_edge_count: u16,
    incident_edge_count: u16,
    junction_kind: RoadJunctionKind,
    highest_incident_road_type: Option<RoadType>,
    incoming_road_type_counts: Vec<RoadGraphNamedCount>,
    outgoing_road_type_counts: Vec<RoadGraphNamedCount>,
    roundabout_entry_count: u16,
    roundabout_exit_count: u16,
    is_complex_junction: bool,
    barrier_class: u8,
    barrier_access: u16,
}

impl SnapshotNode {
    fn new(node: &RoadNode, counts: Option<&NodeRoadTypeCounts>) -> Self {
        Self {
            id: node.id,
            lat: node.lat,
            lng: node.lng,
            source_node_id: node.source_node_id,
            component_id: node.component_id,
            incoming_edge_count: node.incoming_edge_count,
            outgoing_edge_count: node.outgoing_edge_count,
            incident_edge_count: node.incident_edge_count,
            junction_kind: node.junction_kind,
            highest_incident_road_type: node.highest_incident_road_type,
            incoming_road_type_counts: counts.map_or_else(Vec::new, |c| c.incoming.clone()),
            outgoing_road_type_counts: counts.map_or_else(Vec::new, |c| c.outgoing.clone()),
            roundabout_entry_count: node.roundabout_entry_count,
            roundabout_exit_count: node.roundabout_exit_count,
            is_complex_junction: node.is_complex_junction,
            barrier_class: node.barrier_class,
            barrier_access: node.barrier_access,
        }
    }

    fn into_parts(self) -> (RoadNode, NodeRoadTypeCounts) {
        let counts = NodeRoadTypeCounts {
            incoming: self.incoming_road_type_counts,
            outgoing: self.outgoing_road_type_counts,
        };
        let node = RoadNode {
            id: self.id,
            lat: self.lat,
            lng: self.lng,
            source_node_id: self.source_node_id,
            component_id: self.component_id,
            incoming_edge_count: self.incoming_edge_count,
            outgoing_edge_count: self.outgoing_edge_count,
            incident_edge_count: self.incident_edge_count,
            junction_kind: self.junction_kind,
            highest_incident_road_type: self.highest_incident_road_type,
            roundabout_entry_count: self.roundabout_entry_count,
            roundabout_exit_count: self.roundabout_exit_count,
            is_complex_junction: self.is_complex_junction,
            barrier_class: self.barrier_class,
            barrier_access: self.barrier_access,
        };
        (node, counts)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct RoadGraphSnapshot {
    graph_id: String,
    nodes: Vec<SnapshotNode>,
    edges: Vec<RoadGraphSnapshotEdge>,
    #[serde(default)]
    shared_edge_infos: Vec<SharedEdgeInfo>,
    #[serde(default)]
    way_facts: Vec<RoadWayFacts>,
    #[serde(default)]
    route_memberships: Vec<RoadRouteMembership>,
    #[serde(default)]
    turn_restrictions: Vec<RoadTurnRestriction>,
    #[serde(default)]
    resolved_restrictions: Vec<ResolvedEdgeRestriction>,
}

fn road_graph_snapshot(graph: &RoadGraph) -> RoadGraphSnapshot {
    let mut node_lookup = HashMap::new();
    let mut nodes = Vec::new();
    for node_index in graph.graph.node_indices() {
        node_lookup.insert(node_index, nodes.len());
        nodes.push(SnapshotNode::new(
            &graph.graph[node_index],
            graph.node_road_type_counts.get(node_index.index()),
        ));
    }
    let info_index = SharedInfoIndex::new(&graph.shared_edge_infos);

    let edges = graph
        .graph
        .edge_references()
        .filter_map(|edge_ref| {
            let source = node_lookup.get(&edge_ref.source()).copied()?;
            let target = node_lookup.get(&edge_ref.target()).copied()?;
            Some(RoadGraphSnapshotEdge {
                source,
                target,
                edge: SnapshotEdge::new(
                    edge_ref.weight(),
                    shared_edge_info(graph, &info_index, edge_ref.weight().shared_edge_info_id),
                ),
            })
        })
        .collect::<Vec<_>>();

    RoadGraphSnapshot {
        graph_id: graph.graph_id.clone(),
        nodes,
        edges,
        shared_edge_infos: graph.shared_edge_infos.to_vec(),
        way_facts: graph.way_facts.clone(),
        route_memberships: graph.route_memberships.clone(),
        turn_restrictions: graph.turn_restrictions.clone(),
        resolved_restrictions: graph.resolved_restrictions.clone(),
    }
}

pub(crate) fn serialize_road_graph_snapshot(graph: &RoadGraph) -> Result<Vec<u8>, String> {
    let mut bytes = ROAD_GRAPH_SNAPSHOT_VERSION.to_le_bytes().to_vec();
    codec::to_writer(&mut bytes, &road_graph_snapshot(graph))
        .map_err(|err| format!("Failed to serialize road graph: {}", err))?;
    Ok(bytes)
}

fn deserialize_road_graph_snapshot(bytes: &[u8]) -> Result<RoadGraph, String> {
    let version = decode_road_graph_snapshot_version(bytes)?;
    if version != ROAD_GRAPH_SNAPSHOT_VERSION {
        return Err(format!(
            "Unsupported road graph snapshot version {}",
            version
        ));
    }
    let snapshot: RoadGraphSnapshot = codec::from_slice(&bytes[4..])
        .map_err(|err| format!("Failed to decode road graph: {}", err))?;

    let mut graph = RoadGraph::new(snapshot.graph_id);
    graph.shared_edge_infos = snapshot.shared_edge_infos.into();
    graph.set_way_facts(snapshot.way_facts);
    graph.set_route_memberships(snapshot.route_memberships);
    graph.set_turn_restrictions(snapshot.turn_restrictions);
    graph.set_resolved_restrictions(snapshot.resolved_restrictions);

    let mut node_indices = Vec::with_capacity(snapshot.nodes.len());
    let mut node_road_type_counts = Vec::with_capacity(snapshot.nodes.len());
    for node in snapshot.nodes {
        let (node, counts) = node.into_parts();
        node_indices.push(graph.add_node(node));
        node_road_type_counts.push(counts);
    }
    // Counts are all empty in a snapshot of a graph without them.
    if node_road_type_counts
        .iter()
        .any(|counts| !counts.incoming.is_empty() || !counts.outgoing.is_empty())
    {
        graph.node_road_type_counts = node_road_type_counts;
    }
    // Each shared info takes the lineage and coverage id of its first edge.
    let info_index = SharedInfoIndex::new(&graph.shared_edge_infos);
    let mut info_seen = vec![false; graph.shared_edge_infos.len()];
    for edge in snapshot.edges {
        let source = *node_indices
            .get(edge.source)
            .ok_or_else(|| format!("Missing source node {}", edge.source))?;
        let target = *node_indices
            .get(edge.target)
            .ok_or_else(|| format!("Missing target node {}", edge.target))?;
        let SnapshotEdge {
            ref source_lineage_ids,
            ref coverage_id,
            shared_edge_info_id,
            ..
        } = edge.edge;
        if let Some(index) = info_index.get(shared_edge_info_id)
            && !std::mem::replace(&mut info_seen[index], true)
        {
            let infos = &mut graph.shared_edge_infos;
            infos.set_source_lineage_ids(index, source_lineage_ids.clone());
            infos.set_coverage_id(index, coverage_id.as_deref().map(Arc::from));
        }
        graph.add_edge(source, target, edge.edge.into_edge());
    }

    Ok(graph)
}

pub fn serialize_road_graph(graph: &RoadGraph) -> Result<Vec<u8>, String> {
    let raw = serialize_road_graph_snapshot(graph)?;
    encode_road_graph_container(&raw)
}

pub fn deserialize_road_graph(bytes: &[u8]) -> Result<RoadGraph, String> {
    if is_road_graph_container(bytes) {
        let raw = decode_road_graph_container(bytes)?;
        deserialize_road_graph_snapshot(&raw)
    } else {
        deserialize_road_graph_snapshot(bytes)
    }
}

fn encode_road_graph_container(raw: &[u8]) -> Result<Vec<u8>, String> {
    let compressed = zstd::encode_all(raw, ROAD_GRAPH_ZSTD_LEVEL)
        .map_err(|err| format!("Failed to compress road graph: {}", err))?;
    let hash = blake3::hash(raw);
    let mut bytes = Vec::with_capacity(ROAD_GRAPH_CONTAINER_HEADER_LEN + compressed.len());
    bytes.extend_from_slice(ROAD_GRAPH_CONTAINER_MAGIC);
    bytes.extend_from_slice(&ROAD_GRAPH_CONTAINER_VERSION.to_le_bytes());
    bytes.push(ROAD_GRAPH_CONTAINER_CODEC_ZSTD);
    bytes.push(0);
    bytes.extend_from_slice(&ROAD_GRAPH_SNAPSHOT_VERSION.to_le_bytes());
    bytes.extend_from_slice(&(raw.len() as u64).to_le_bytes());
    bytes.extend_from_slice(&(compressed.len() as u64).to_le_bytes());
    bytes.extend_from_slice(hash.as_bytes());
    bytes.extend_from_slice(&compressed);
    Ok(bytes)
}

struct EncodedRoadGraphPayload {
    uncompressed_bytes: u64,
    compressed_bytes: u64,
    raw_hash: blake3::Hash,
}

struct CountingWriter<W> {
    inner: W,
    bytes_written: u64,
}

impl<W> CountingWriter<W> {
    fn new(inner: W) -> Self {
        Self {
            inner,
            bytes_written: 0,
        }
    }
}

impl<W: Write> Write for CountingWriter<W> {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let written = self.inner.write(buf)?;
        self.bytes_written += written as u64;
        Ok(written)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.inner.flush()
    }
}

struct HashingWriter<W> {
    inner: W,
    hasher: blake3::Hasher,
    bytes_written: u64,
}

impl<W> HashingWriter<W> {
    fn new(inner: W) -> Self {
        Self {
            inner,
            hasher: blake3::Hasher::new(),
            bytes_written: 0,
        }
    }

    fn into_inner(self) -> W {
        self.inner
    }

    fn raw_hash(&self) -> blake3::Hash {
        self.hasher.finalize()
    }
}

impl<W: Write> Write for HashingWriter<W> {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let written = self.inner.write(buf)?;
        self.hasher.update(&buf[..written]);
        self.bytes_written += written as u64;
        Ok(written)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.inner.flush()
    }
}

fn encode_road_graph_snapshot_payload(
    snapshot: &RoadGraphSnapshot,
    payload_path: &Path,
) -> Result<EncodedRoadGraphPayload, String> {
    let payload_file = std::fs::File::create(payload_path)
        .map_err(|err| format!("Failed to create {:?}: {}", payload_path, err))?;
    let counting_payload = CountingWriter::new(payload_file);
    let encoder = zstd::Encoder::new(counting_payload, ROAD_GRAPH_ZSTD_LEVEL)
        .map_err(|err| format!("Failed to initialize road graph zstd encoder: {}", err))?;
    let mut hashing_encoder = HashingWriter::new(encoder);
    hashing_encoder
        .write_all(&ROAD_GRAPH_SNAPSHOT_VERSION.to_le_bytes())
        .map_err(|err| format!("Failed to serialize road graph: {}", err))?;
    codec::to_writer(&mut hashing_encoder, snapshot)
        .map_err(|err| format!("Failed to serialize road graph: {}", err))?;
    let uncompressed_bytes = hashing_encoder.bytes_written;
    let raw_hash = hashing_encoder.raw_hash();
    let encoder = hashing_encoder.into_inner();
    let compressed_payload = encoder
        .finish()
        .map_err(|err| format!("Failed to compress road graph: {}", err))?;

    Ok(EncodedRoadGraphPayload {
        uncompressed_bytes,
        compressed_bytes: compressed_payload.bytes_written,
        raw_hash,
    })
}

fn road_graph_container_header(payload: &EncodedRoadGraphPayload) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(ROAD_GRAPH_CONTAINER_HEADER_LEN);
    bytes.extend_from_slice(ROAD_GRAPH_CONTAINER_MAGIC);
    bytes.extend_from_slice(&ROAD_GRAPH_CONTAINER_VERSION.to_le_bytes());
    bytes.push(ROAD_GRAPH_CONTAINER_CODEC_ZSTD);
    bytes.push(0);
    bytes.extend_from_slice(&ROAD_GRAPH_SNAPSHOT_VERSION.to_le_bytes());
    bytes.extend_from_slice(&payload.uncompressed_bytes.to_le_bytes());
    bytes.extend_from_slice(&payload.compressed_bytes.to_le_bytes());
    bytes.extend_from_slice(payload.raw_hash.as_bytes());
    bytes
}

fn write_road_graph_container_from_payload(
    path: &Path,
    payload_path: &Path,
    header: &[u8],
) -> Result<(), String> {
    let mut file = std::fs::File::create(path)
        .map_err(|err| format!("Failed to create {:?}: {}", path, err))?;
    file.write_all(header)
        .map_err(|err| format!("Failed to write {:?}: {}", path, err))?;
    let payload_file = std::fs::File::open(payload_path)
        .map_err(|err| format!("Failed to open {:?}: {}", payload_path, err))?;
    let mut payload = BufReader::new(payload_file);
    std::io::copy(&mut payload, &mut file)
        .map_err(|err| format!("Failed to write {:?}: {}", path, err))?;
    Ok(())
}

fn decode_road_graph_container(bytes: &[u8]) -> Result<Vec<u8>, String> {
    let header = parse_road_graph_container_header(bytes)?;
    let compressed_payload = &bytes[ROAD_GRAPH_CONTAINER_HEADER_LEN..];
    if compressed_payload.len() as u64 != header.compressed_bytes {
        return Err(format!(
            "Road graph compressed payload length mismatch: header {} B, actual {} B",
            header.compressed_bytes,
            compressed_payload.len()
        ));
    }
    let raw = zstd::decode_all(compressed_payload)
        .map_err(|err| format!("Failed to decompress road graph: {}", err))?;
    if raw.len() as u64 != header.uncompressed_bytes {
        return Err(format!(
            "Road graph uncompressed length mismatch: header {} B, actual {} B",
            header.uncompressed_bytes,
            raw.len()
        ));
    }
    let actual_hash = blake3::hash(&raw).to_hex().to_string();
    if actual_hash != header.blake3_hash_hex {
        return Err("Road graph checksum mismatch".to_string());
    }
    Ok(raw)
}

fn decode_road_graph_snapshot_version(bytes: &[u8]) -> Result<u32, String> {
    if bytes.len() < 4 {
        return Err(format!(
            "Road graph snapshot is too short: {} B",
            bytes.len()
        ));
    }
    Ok(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
}

pub fn load_road_graph_file(path: &Path) -> Result<RoadGraph, String> {
    let bytes = std::fs::read(path).map_err(|err| format!("Failed to read {:?}: {}", path, err))?;
    deserialize_road_graph(&bytes)
}

pub fn persist_road_graph_file(path: &Path, graph: &RoadGraph) -> Result<(), String> {
    persist_road_graph_file_with_info(path, graph).map(|_| ())
}

pub fn persist_road_graph_file_with_info(
    path: &Path,
    graph: &RoadGraph,
) -> Result<RoadGraphContainerInfo, String> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)
            .map_err(|err| format!("Failed to create {:?}: {}", parent, err))?;
    }
    let payload_path = path.with_extension("mrg.payload.tmp");
    let _ = std::fs::remove_file(&payload_path);
    let snapshot = road_graph_snapshot(graph);
    let payload = encode_road_graph_snapshot_payload(&snapshot, &payload_path)?;
    drop(snapshot);
    let header = road_graph_container_header(&payload);
    write_road_graph_container_from_payload(path, &payload_path, &header)?;
    let _ = std::fs::remove_file(&payload_path);

    Ok(RoadGraphContainerInfo {
        container_version: ROAD_GRAPH_CONTAINER_VERSION,
        codec: ROAD_GRAPH_CONTAINER_CODEC.to_string(),
        graph_schema_version: ROAD_GRAPH_SNAPSHOT_VERSION,
        uncompressed_bytes: payload.uncompressed_bytes,
        compressed_bytes: payload.compressed_bytes,
        blake3_hash_hex: payload.raw_hash.to_hex().to_string(),
    })
}

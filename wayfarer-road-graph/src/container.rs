//! The road graph container header (`.mrg`): versions, magic and inspection.

pub const ROAD_GRAPH_SNAPSHOT_VERSION: u32 = 1;
/// The snapshot inside is encoded with `wayfarer_package::codec`
/// (variable-length integers) after a 4-byte version.
pub const ROAD_GRAPH_CONTAINER_VERSION: u16 = 1;
pub const ROAD_GRAPH_CONTAINER_CODEC: &str = "zstd";

pub(crate) const ROAD_GRAPH_CONTAINER_MAGIC: &[u8; 4] = b"MRG\0";
pub(crate) const ROAD_GRAPH_CONTAINER_CODEC_ZSTD: u8 = 1;
pub(crate) const ROAD_GRAPH_CONTAINER_HEADER_LEN: usize = 60;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RoadGraphContainerInfo {
    pub container_version: u16,
    pub codec: String,
    pub graph_schema_version: u32,
    pub uncompressed_bytes: u64,
    pub compressed_bytes: u64,
    pub blake3_hash_hex: String,
}

pub fn inspect_road_graph_container(bytes: &[u8]) -> Result<RoadGraphContainerInfo, String> {
    let header = parse_road_graph_container_header(bytes)?;
    let compressed_payload = &bytes[ROAD_GRAPH_CONTAINER_HEADER_LEN..];
    if compressed_payload.len() as u64 != header.compressed_bytes {
        return Err(format!(
            "Road graph compressed payload length mismatch: header {} B, actual {} B",
            header.compressed_bytes,
            compressed_payload.len()
        ));
    }
    Ok(header)
}

pub(crate) fn is_road_graph_container(bytes: &[u8]) -> bool {
    bytes.starts_with(ROAD_GRAPH_CONTAINER_MAGIC)
}

pub(crate) fn parse_road_graph_container_header(
    bytes: &[u8],
) -> Result<RoadGraphContainerInfo, String> {
    if bytes.len() < ROAD_GRAPH_CONTAINER_HEADER_LEN {
        return Err(format!(
            "Road graph container is too short: {} B",
            bytes.len()
        ));
    }
    if !is_road_graph_container(bytes) {
        return Err("Road graph container magic is invalid".to_string());
    }

    let container_version = u16::from_le_bytes([bytes[4], bytes[5]]);
    if container_version != ROAD_GRAPH_CONTAINER_VERSION {
        return Err(format!(
            "Unsupported road graph container version {}",
            container_version
        ));
    }

    let codec = bytes[6];
    if codec != ROAD_GRAPH_CONTAINER_CODEC_ZSTD {
        return Err(format!("Unsupported road graph container codec {}", codec));
    }

    let graph_schema_version = u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);
    if graph_schema_version != ROAD_GRAPH_SNAPSHOT_VERSION {
        return Err(format!(
            "Unsupported road graph snapshot version {}",
            graph_schema_version
        ));
    }

    let uncompressed_bytes = u64::from_le_bytes([
        bytes[12], bytes[13], bytes[14], bytes[15], bytes[16], bytes[17], bytes[18], bytes[19],
    ]);
    let compressed_bytes = u64::from_le_bytes([
        bytes[20], bytes[21], bytes[22], bytes[23], bytes[24], bytes[25], bytes[26], bytes[27],
    ]);
    let hash = &bytes[28..60];

    Ok(RoadGraphContainerInfo {
        container_version,
        codec: ROAD_GRAPH_CONTAINER_CODEC.to_string(),
        graph_schema_version,
        uncompressed_bytes,
        compressed_bytes,
        blake3_hash_hex: hash
            .iter()
            .map(|byte| format!("{:02x}", byte))
            .collect::<String>(),
    })
}

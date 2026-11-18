//! Every package format this build reads opens and routes: one frozen package
//! per format version under `fixtures/routing/format-<N>/`.

use std::path::Path;
use std::sync::Arc;

use wayfarer::{GeoPoint, LegRequest, MobilityProfile, RouteStatus, RoutingEngine, RoutingPackage};
use wayfarer_compiler::package_build::{PackageBuildOptions, write_package};
use wayfarer_package::format::{
    HEADER_COUNTS, HEADER_FIXED_BYTES, HEADER_ROOT_CHECKSUM, HEADER_VERSION,
    MIN_READ_FORMAT_VERSION, PackageFile, ROUTING_PACKAGE_FORMAT_VERSION, Section,
    TABLE_ENTRY_BYTES, section_count,
};
use wayfarer_road_graph::{
    DirectionalAccess, RoadGraphBuilder, RoadType, RoutableWay, build_road_graph_manifest,
    enhance_road_graph, inspect_road_graph_container, serialize_road_graph,
};

#[test]
fn every_readable_format_opens_and_routes() {
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/routing");
    // The same synthetic road, regenerated with today's format and model.
    let current = RoutingEngine::from_package(Arc::new(
        RoutingPackage::open(&fixtures.join("short_road.mrt")).unwrap(),
    ));
    for version in MIN_READ_FORMAT_VERSION..=ROUTING_PACKAGE_FORMAT_VERSION {
        let path = fixtures
            .join(format!("format-{version}"))
            .join("short_road.mrt");
        assert!(
            path.exists(),
            "format {version} is readable but has no fixture at {}",
            path.display()
        );
        let package = RoutingPackage::open(&path).unwrap();
        assert_eq!(package.format_version(), version);
        // A frozen package of an older costing model is re-costed with this
        // build's model from its stored costing inputs. A walk costs exactly
        // what it does on a package built today; a drive on this unposted
        // road keeps the speed its compiler inferred (an input), costed with
        // today's road factor.
        assert_eq!(
            package.is_recosted(),
            PackageFile::open(&path)
                .unwrap()
                .metadata()
                .costing_model_version
                < wayfarer_costing::COSTING_MODEL_VERSION,
            "format {version}"
        );
        let engine = RoutingEngine::from_package(Arc::new(package));
        for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
            let route = engine.route_leg(short_road_request(profile)).unwrap();
            assert_eq!(route.status, RouteStatus::Routed, "format {version}");
            assert!(route.distance_m > 100.0, "format {version}");
            let today = current.route_leg(short_road_request(profile)).unwrap();
            let (factor, today_factor) =
                (route.cost / route.duration_s, today.cost / today.duration_s);
            assert!(
                (factor - today_factor).abs() <= 1e-4 * today_factor,
                "format {version} {profile:?}: cost per second {factor} vs today's {today_factor}"
            );
            if profile == MobilityProfile::OnFoot {
                assert!(
                    (route.cost - today.cost).abs() <= 1e-4 * today.cost,
                    "format {version} {profile:?}: {} vs today's {}",
                    route.cost,
                    today.cost
                );
                assert!(
                    (route.duration_s - today.duration_s).abs() <= 1e-4 * today.duration_s,
                    "format {version} {profile:?}"
                );
            }
        }
    }
}

fn short_road_request(profile: MobilityProfile) -> LegRequest {
    LegRequest::new(
        GeoPoint::new(52.0, 21.0002),
        GeoPoint::new(52.0, 21.0018),
        profile,
    )
}

/// `bytes` with a root checksum that matches its header and section table, as
/// a hostile package would carry: the checksum guards against corruption only.
fn resigned(mut bytes: Vec<u8>) -> Vec<u8> {
    let version = u32::from_le_bytes(
        bytes[HEADER_VERSION..HEADER_VERSION + 4]
            .try_into()
            .unwrap(),
    );
    let table_end = HEADER_FIXED_BYTES + section_count(version) * TABLE_ENTRY_BYTES;
    let mut header = bytes[..HEADER_FIXED_BYTES].to_vec();
    header[HEADER_ROOT_CHECKSUM..HEADER_ROOT_CHECKSUM + 32].fill(0);
    let mut root = blake3::Hasher::new();
    root.update(&header);
    root.update(&bytes[HEADER_FIXED_BYTES..table_end]);
    bytes[HEADER_ROOT_CHECKSUM..HEADER_ROOT_CHECKSUM + 32]
        .copy_from_slice(root.finalize().as_bytes());
    bytes
}

/// The byte range of `section` in package `bytes`.
fn section_range(bytes: &[u8], section: Section) -> std::ops::Range<usize> {
    let entry = HEADER_FIXED_BYTES + section as usize * TABLE_ENTRY_BYTES;
    let offset = u64::from_le_bytes(bytes[entry..entry + 8].try_into().unwrap()) as usize;
    let len = u64::from_le_bytes(bytes[entry + 8..entry + 16].try_into().unwrap()) as usize;
    offset..offset + len
}

/// A package whose landmark names a node it doesn't have is rejected when it
/// opens; engines used to index node arrays with it at query time (found by
/// the `package` fuzz target).
#[test]
fn a_landmark_node_out_of_range_is_rejected() {
    // Format 1 stores landmarks; format 2 packages, by default, do not.
    let path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/routing/format-1/short_road.mrt");
    let mut bytes = std::fs::read(path).unwrap();
    let entry = HEADER_FIXED_BYTES + Section::LandmarkNode as usize * TABLE_ENTRY_BYTES;
    let offset = u64::from_le_bytes(bytes[entry..entry + 8].try_into().unwrap()) as usize;
    let len = u64::from_le_bytes(bytes[entry + 8..entry + 16].try_into().unwrap()) as usize;
    assert!(len >= 4, "the fixture stores landmark nodes");
    bytes[offset..offset + 4].copy_from_slice(&3_788_965_444u32.to_le_bytes());
    let error = RoutingPackage::from_bytes(&resigned(bytes)).unwrap_err();
    assert!(
        error.to_string().contains("landmark node out of range"),
        "{error}"
    );
}

/// A package with a negative edge cost is rejected when it opens: searches
/// assume non-negative costs, and one used to loop forever rebuilding a path
/// (found by the `package` fuzz target).
#[test]
fn a_negative_edge_cost_is_rejected() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/routing/short_road.mrt");
    let mut bytes = std::fs::read(path).unwrap();
    let entry = HEADER_FIXED_BYTES + Section::EdgeCost as usize * TABLE_ENTRY_BYTES;
    let offset = u64::from_le_bytes(bytes[entry..entry + 8].try_into().unwrap()) as usize;
    for value in [-3_332_946_788_352.0f32, f32::NAN] {
        bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        let error = RoutingPackage::from_bytes(&resigned(bytes.clone())).unwrap_err();
        assert!(error.to_string().contains("edge costs"), "{value}: {error}");
    }
}

/// A package whose header counts overflow when multiplied by element sizes is
/// rejected, not a panic with overflow checks on (found by the `package` fuzz
/// target with `--debug-assertions`).
#[test]
fn overflowing_header_counts_are_rejected() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/routing/short_road.mrt");
    let bytes = std::fs::read(path).unwrap();
    // Node, edge, shared-info, restriction, column and landmark counts.
    for slot in 0..7 {
        for count in [
            u64::MAX,
            u64::MAX / 2,
            u64::from(u32::MAX),
            (1 << 62) + 1,
            1 << 40,
        ] {
            let mut bytes = bytes.clone();
            let at = HEADER_COUNTS + slot * 8;
            bytes[at..at + 8].copy_from_slice(&count.to_le_bytes());
            assert!(
                RoutingPackage::from_bytes(&resigned(bytes)).is_err(),
                "count slot {slot} = {count}"
            );
        }
    }
}

/// A stored CCH order that is not a permutation of the nodes is rejected
/// when the package opens: the potentials index by it.
#[test]
fn a_cch_order_that_is_not_a_permutation_is_rejected() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/routing/short_road.mrt");
    let mut bytes = std::fs::read(path).unwrap();
    let entry = HEADER_FIXED_BYTES + Section::CchRank as usize * TABLE_ENTRY_BYTES;
    let offset = u64::from_le_bytes(bytes[entry..entry + 8].try_into().unwrap()) as usize;
    let len = u64::from_le_bytes(bytes[entry + 8..entry + 16].try_into().unwrap()) as usize;
    assert!(len >= 8, "the fixture stores a CCH order");
    // Two nodes of the same rank, then a rank out of range.
    let first = bytes[offset..offset + 4].to_vec();
    bytes[offset + 4..offset + 8].copy_from_slice(&first);
    assert!(RoutingPackage::from_bytes(&resigned(bytes.clone())).is_err());
    bytes[offset..offset + 4].copy_from_slice(&u32::MAX.to_le_bytes());
    assert!(RoutingPackage::from_bytes(&resigned(bytes)).is_err());
}

/// A package with an edge heading of 360 degrees or more is rejected when it
/// opens; costing folded heading differences with `360 - delta`, which
/// overflowed (found by the `package` fuzz target with debug assertions).
#[test]
fn an_out_of_range_heading_is_rejected() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/routing/short_road.mrt");
    let bytes = std::fs::read(path).unwrap();
    let headings = section_range(&bytes, Section::EdgeHeadingStart);
    assert!(headings.len() >= 2, "the fixture stores edge headings");
    for heading in [360u16, 400, u16::MAX - 1] {
        let mut bytes = bytes.clone();
        bytes[headings.start..headings.start + 2].copy_from_slice(&heading.to_le_bytes());
        let error = RoutingPackage::from_bytes(&resigned(bytes)).unwrap_err();
        assert!(
            error.to_string().contains("edge headings out of range"),
            "{heading}: {error}"
        );
    }
    // No heading at all stays valid.
    let mut bytes = bytes;
    bytes[headings.start..headings.start + 2].copy_from_slice(&u16::MAX.to_le_bytes());
    RoutingPackage::from_bytes(&resigned(bytes)).unwrap();
}

/// A package that marks a junction as a chain node is rejected when it
/// opens: the route search walks a chain node's single way on without
/// queueing labels, and asserted "a chain node continues one way" (found by
/// the `package` fuzz target with debug assertions).
#[test]
fn a_chain_mark_at_a_junction_is_rejected() {
    // Four roads meeting at (52.0, 21.0).
    let mut builder = RoadGraphBuilder::new("dataset-junction");
    let arms = [(0.001, 0.0), (-0.001, 0.0), (0.0, 0.001), (0.0, -0.001)];
    for (id, (lat, lng)) in (1..).zip(arms) {
        builder.push_way(RoutableWay {
            osm_way_id: id,
            points: vec![(52.0, 21.0), (52.0 + lat, 21.0 + lng)],
            road_type: RoadType::Primary,
            directional_access: DirectionalAccess::Both,
            highway: Some("primary".into()),
            access: Some("yes".into()),
            ..RoutableWay::default()
        });
    }
    let mut graph = builder.finish();
    enhance_road_graph(&mut graph);
    let container = inspect_road_graph_container(&serialize_road_graph(&graph).unwrap()).unwrap();
    let manifest = build_road_graph_manifest(&graph, &container);
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("junction.mrt");
    write_package(
        &graph,
        Some(&manifest),
        &path,
        &PackageBuildOptions::default(),
    )
    .unwrap();
    let bytes = std::fs::read(&path).unwrap();
    RoutingPackage::from_bytes(&bytes).unwrap();

    let first_out = section_range(&bytes, Section::NodeFirstOut);
    let first_out = bytes[first_out]
        .as_chunks::<4>()
        .0
        .iter()
        .map(|word| u32::from_le_bytes(*word))
        .collect::<Vec<_>>();
    let junction = first_out
        .windows(2)
        .position(|pair| pair[1] - pair[0] == 4)
        .expect("the junction has four out-edges");
    let chain = section_range(&bytes, Section::PieceChain);
    let tree = section_range(&bytes, Section::PieceTree);
    assert!(!chain.is_empty(), "the package classifies its columns");
    let mut bytes = bytes;
    // No tree marks (the arms count as the junction's core neighbours), and
    // the junction marked a chain node in the first column.
    bytes[tree].fill(0);
    let word = chain.start + junction / 64 * 8;
    let mut bits = u64::from_le_bytes(bytes[word..word + 8].try_into().unwrap());
    bits |= 1 << (junction % 64);
    bytes[word..word + 8].copy_from_slice(&bits.to_le_bytes());
    let error = RoutingPackage::from_bytes(&resigned(bytes)).unwrap_err();
    assert!(
        error.to_string().contains("piece kinds are invalid"),
        "{error}"
    );
}

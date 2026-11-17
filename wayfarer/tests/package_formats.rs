//! Every package format this build reads opens and routes: one frozen package
//! per format version under `fixtures/routing/format-<N>/`.

use std::path::Path;
use std::sync::Arc;

use wayfarer::{GeoPoint, LegRequest, MobilityProfile, RouteStatus, RoutingEngine, RoutingPackage};
use wayfarer_package::format::{
    HEADER_COUNTS, HEADER_FIXED_BYTES, HEADER_ROOT_CHECKSUM, MIN_READ_FORMAT_VERSION, PackageFile,
    ROUTING_PACKAGE_FORMAT_VERSION, SECTION_COUNT, Section, TABLE_ENTRY_BYTES,
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
    let table_end = HEADER_FIXED_BYTES + SECTION_COUNT * TABLE_ENTRY_BYTES;
    let mut header = bytes[..HEADER_FIXED_BYTES].to_vec();
    header[HEADER_ROOT_CHECKSUM..HEADER_ROOT_CHECKSUM + 32].fill(0);
    let mut root = blake3::Hasher::new();
    root.update(&header);
    root.update(&bytes[HEADER_FIXED_BYTES..table_end]);
    bytes[HEADER_ROOT_CHECKSUM..HEADER_ROOT_CHECKSUM + 32]
        .copy_from_slice(root.finalize().as_bytes());
    bytes
}

/// A package whose landmark names a node it doesn't have is rejected when it
/// opens; engines used to index node arrays with it at query time (found by
/// the `package` fuzz target).
#[test]
fn a_landmark_node_out_of_range_is_rejected() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/routing/short_road.mrt");
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

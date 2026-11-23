//! Format 3's section table (`docs/design/format-3.md`, sections 2 to 5):
//! a format-3 compile routes as format 2 does, the identity and content
//! hashes, and the table checks against hostile packages.

use std::path::Path;
use std::sync::{Arc, OnceLock};

use wayfarer::{
    GeoPoint, MobilityProfile, RouteMatrixRequest, RouteRequest, RoutingEngine, RoutingPackage,
};
use wayfarer_compiler::{CancelToken, CompileRequest, compile};
use wayfarer_package::format::{
    FORMAT_3_DRAFT_REVISION, HEADER_V3_LAYOUT_FLAGS, PARAM_MUST_UNDERSTAND, PackageFile,
    PackageFormat, PackageParams, RoutingPackageError, SECTION_IDENTITY, SECTION_REQUIRED, Section,
    param_key,
};
use wayfarer_package::raw::{RawPackage, RawSection, resign};

/// Monaco compiled in both formats, once.
fn monaco() -> &'static (Vec<u8>, Vec<u8>) {
    static PACKAGES: OnceLock<(Vec<u8>, Vec<u8>)> = OnceLock::new();
    PACKAGES.get_or_init(|| {
        let dir = tempfile::tempdir().unwrap();
        let pbf = Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/monaco.osm.pbf");
        let mut packages = [PackageFormat::V2, PackageFormat::V3].map(|format| {
            let output = dir.path().join(format!("monaco-{}.mrt", format.version()));
            let mut request = CompileRequest::new(&pbf, &output).country("MC");
            request.package_format = format;
            let result = compile(&request, &CancelToken::new(), |_| {}).unwrap();
            assert_eq!(result.format_version, format.version());
            std::fs::read(&output).unwrap()
        });
        let v3 = std::mem::take(&mut packages[1]);
        (std::mem::take(&mut packages[0]), v3)
    })
}

fn format_3() -> RawPackage {
    RawPackage::parse(&monaco().1).unwrap()
}

fn open(bytes: &[u8]) -> Result<PackageFile, RoutingPackageError> {
    PackageFile::from_bytes(bytes)
}

fn open_error(raw: &RawPackage) -> String {
    match open(&raw.encode()) {
        Ok(_) => panic!("the package opened"),
        Err(error) => error.to_string(),
    }
}

fn points() -> Vec<GeoPoint> {
    [
        (43.7397, 7.4275),
        (43.7290, 7.4160),
        (43.7316, 7.4199),
        (43.7350, 7.4210),
        (43.7480, 7.4380),
        (43.7250, 7.4130),
    ]
    .into_iter()
    .map(|(lat, lng)| GeoPoint::new(lat, lng))
    .collect()
}

/// `text` with the numbers after `_ns: ` and `_ms: ` (timings) zeroed.
fn without_timings(text: String) -> String {
    let mut out = String::with_capacity(text.len());
    let mut rest = text.as_str();
    while let Some(at) = ["_ns: ", "_ms: "]
        .iter()
        .filter_map(|marker| rest.find(marker).map(|at| at + marker.len()))
        .min()
    {
        out.push_str(&rest[..at]);
        rest = rest[at..].trim_start_matches(|c: char| c.is_ascii_digit());
        out.push('0');
    }
    out.push_str(rest);
    out
}

/// Every route, and a matrix, give the same answer on both formats: the
/// same graph, ids and costs, read through the other table.
#[test]
fn a_format_3_package_routes_as_format_2_does() {
    let (v2, v3) = monaco();
    let engines = [v2, v3].map(|bytes| {
        RoutingEngine::from_package(Arc::new(RoutingPackage::from_bytes(bytes).unwrap()))
    });
    let points = points();
    // Blocked edges are named by id: the engine maps them to edges through
    // the stored ids (format 2) or the implied ones (format 3).
    let base = open(v2).unwrap().view().edge_id_base;
    let blocked: std::collections::BTreeSet<u64> =
        (0..4_000).step_by(7).map(|slot| base + slot).collect();
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        for from in &points {
            for to in &points {
                for block in [false, true] {
                    let [a, b] = engines.each_ref().map(|engine| {
                        let mut request = RouteRequest::new(*from, *to, profile);
                        if block {
                            request.options.blocked_edge_ids =
                                std::collections::BTreeSet::clone(&blocked);
                        }
                        without_timings(format!("{:?}", engine.route(request)))
                    });
                    assert_eq!(a, b, "{profile:?} {from:?} -> {to:?} blocked {block}");
                }
            }
        }
    }
    let matrix = |engine: &RoutingEngine| {
        let request: RouteMatrixRequest = serde_json::from_value(serde_json::json!({
            "sources": points.iter().map(|p| serde_json::json!({"lat": p.lat, "lng": p.lng})).collect::<Vec<_>>(),
            "targets": points.iter().map(|p| serde_json::json!({"lat": p.lat, "lng": p.lng})).collect::<Vec<_>>(),
            "profile": "wheeled",
        }))
        .unwrap();
        without_timings(format!("{:?}", engine.matrix(request)))
    };
    assert_eq!(matrix(&engines[0]), matrix(&engines[1]));
}

#[test]
fn format_3_drops_stored_ids_and_reserved_sections_and_states_its_params() {
    let (v2, v3) = monaco();
    let (v2, v3) = (open(v2).unwrap(), open(v3).unwrap());
    assert_eq!((v2.format_version(), v3.format_version()), (2, 3));
    assert_eq!(v2.counts(), v3.counts());
    assert!(v3.byte_len() < v2.byte_len());
    let names = v3
        .section_sizes()
        .into_iter()
        .map(|(name, _)| name)
        .collect::<Vec<_>>();
    for retired in [
        "EdgeId",
        "NodeIdDelta",
        "NodeIdWide",
        "RequestMeta",
        "Reserved1",
        "Reserved12",
    ] {
        assert!(!names.contains(&retired), "{retired}");
    }
    assert!(names.contains(&"PackageParams"));
    // Positions are derived from the fixed-point vectors here.
    assert!(!names.contains(&"NodeLatE7"));
    let (a, b) = (v2.view(), v3.view());
    for edge in 0..a.edge_count {
        assert_eq!(a.edge_id(edge), b.edge_id(edge));
        assert_eq!(b.edge_for_id(a.edge_id(edge)), Some(edge as u32));
    }
    assert_eq!(b.edge_for_id(b.edge_id_base + b.edge_count as u64), None);
    let (ra, rb) = (v2.request_view().unwrap(), v3.request_view().unwrap());
    for node in 0..a.node_count as u32 {
        assert_eq!(ra.node_id(node), rb.node_id(node));
        assert_eq!(ra.component_id(node), rb.component_id(node));
    }
    for shared in 0..v2.counts().shared_info_count {
        assert_eq!(ra.shared_info_id(shared), rb.shared_info_id(shared));
    }
    let (pa, pb) = (v2.params(), v3.params());
    assert_eq!(pb.draft_revision, Some(FORMAT_3_DRAFT_REVISION));
    assert_eq!(
        PackageParams {
            draft_revision: None,
            ..*pb
        },
        *pa
    );
    assert_eq!(v2.metadata(), v3.metadata());
}

#[test]
fn format_3_has_an_identity_hash_apart_from_its_content_hash() {
    let (v2, v3) = monaco();
    let (v2, v3) = (open(v2).unwrap(), open(v3).unwrap());
    assert_eq!(v2.dataset_hash(), v2.content_hash());
    assert_ne!(v3.dataset_hash(), v3.content_hash());

    // An unknown optional section, in the table or anywhere in the file,
    // keeps the identity; the content hash changes.
    let mut raw = format_3();
    raw.sections.push(RawSection {
        id: 600,
        param: 7,
        flags: 0,
        element_bytes: 4,
        bytes: vec![1; 40],
    });
    let extended = open(&raw.encode()).unwrap();
    assert_eq!(extended.dataset_hash(), v3.dataset_hash());
    assert_ne!(extended.content_hash(), v3.content_hash());
    assert_eq!(extended.unknown_sections().len(), 1);
    assert_eq!(extended.unknown_sections()[0].id, 600);
    assert_eq!(extended.unknown_sections()[0].byte_len, 40);
    raw.table_first = true;
    let moved = open(&raw.encode()).unwrap();
    assert_eq!(moved.dataset_hash(), v3.dataset_hash());

    // An unknown IDENTITY section is hashed by every reader.
    let mut raw = format_3();
    raw.sections.push(RawSection {
        id: 601,
        param: 0,
        flags: SECTION_IDENTITY,
        element_bytes: 1,
        bytes: vec![2; 3],
    });
    assert_ne!(
        open(&raw.encode()).unwrap().dataset_hash(),
        v3.dataset_hash()
    );

    // So does any change to an identity section, and the metadata does not.
    let mut raw = format_3();
    let at = raw.position(Section::EdgeLength.id()).unwrap();
    raw.sections[at].bytes[0] ^= 1;
    assert_ne!(
        open(&raw.encode()).unwrap().dataset_hash(),
        v3.dataset_hash()
    );
    let mut raw = format_3();
    let at = raw.position(Section::Metadata.id()).unwrap();
    raw.sections[at].bytes.extend_from_slice(b"  ");
    assert_eq!(
        open(&raw.encode()).unwrap().dataset_hash(),
        v3.dataset_hash()
    );
}

#[test]
fn verify_hashes_every_section_and_the_table_wherever_it_is() {
    let dir = tempfile::tempdir().unwrap();
    let mut raw = format_3();
    raw.sections.push(RawSection {
        id: 600,
        param: 0,
        flags: 0,
        element_bytes: 1,
        bytes: vec![5; 3],
    });
    for table_first in [false, true] {
        raw.table_first = table_first;
        let path = dir.path().join(format!("routing-{table_first}.mrt"));
        std::fs::write(&path, raw.encode()).unwrap();
        let verified = PackageFile::verify(&path).unwrap();
        let package = PackageFile::open(&path).unwrap();
        assert_eq!(verified.dataset_hash, package.dataset_hash());
        assert_eq!(verified.content_hash, package.content_hash());
        assert_eq!(verified.sections.len(), raw.sections.len());
        assert!(
            verified
                .sections
                .iter()
                .any(|section| section.name == "unknown" && section.id == 600)
        );
        // A flipped byte in a section, or in the padding, is caught.
        let mut bytes = std::fs::read(&path).unwrap();
        let last = bytes.len() - 1;
        bytes[last] ^= 1;
        std::fs::write(&path, &bytes).unwrap();
        assert!(PackageFile::verify(&path).is_err());
    }
}

#[test]
fn the_table_must_be_sorted_without_repeats() {
    let mut raw = format_3();
    raw.sections.swap(1, 2);
    assert!(
        open_error(&raw).contains("not sorted"),
        "{}",
        open_error(&raw)
    );
    let mut raw = format_3();
    let copy = raw.sections[3].clone();
    raw.sections.insert(3, copy);
    assert!(open_error(&raw).contains("not sorted"));
    // A known section never takes a parameter.
    let mut raw = format_3();
    raw.sections[3].param = 1;
    assert!(open_error(&raw).contains("parameter"));
}

#[test]
fn known_sections_keep_their_element_size_and_flags() {
    let at = |raw: &RawPackage| raw.position(Section::EdgeTarget.id()).unwrap();
    let mut raw = format_3();
    let edge_target = at(&raw);
    raw.sections[edge_target].element_bytes = 8;
    assert!(open_error(&raw).contains("element size"));
    // Dropping IDENTITY would let two graphs share an identity hash.
    let mut raw = format_3();
    raw.sections[edge_target].flags = SECTION_REQUIRED;
    assert!(open_error(&raw).contains("flags"));
    let mut raw = format_3();
    raw.sections[edge_target].flags |= 1 << 5;
    assert!(open_error(&raw).contains("unknown flags"));
}

#[test]
fn an_unknown_required_section_needs_a_newer_build() {
    let mut raw = format_3();
    raw.sections.push(RawSection {
        id: 517,
        param: 0,
        flags: SECTION_REQUIRED,
        element_bytes: 4,
        bytes: vec![0; 8],
    });
    let error = open(&raw.encode()).unwrap_err();
    assert!(matches!(error, RoutingPackageError::TooNew(_)), "{error}");
    assert!(error.to_string().contains("section 517"), "{error}");
    // Experimental sections are never REQUIRED.
    let mut raw = format_3();
    raw.sections.push(RawSection {
        id: 0x8000_0001,
        param: 0,
        flags: SECTION_REQUIRED,
        element_bytes: 1,
        bytes: Vec::new(),
    });
    assert!(open_error(&raw).contains("experimental"));
}

#[test]
fn mandatory_sections_are_present_and_retired_ones_absent() {
    let mut raw = format_3();
    let at = raw.position(Section::EdgeTwin.id()).unwrap();
    raw.sections.remove(at);
    assert!(open_error(&raw).contains("missing"), "{}", open_error(&raw));
    // A retired id is never read again, nor reused.
    let mut raw = format_3();
    let at = raw.position(Section::EdgeLength.id()).unwrap();
    raw.sections.insert(
        at,
        RawSection {
            id: Section::EdgeId.id(),
            param: 0,
            flags: SECTION_REQUIRED | SECTION_IDENTITY,
            element_bytes: 4,
            bytes: Vec::new(),
        },
    );
    assert!(open_error(&raw).contains("retired"));
    // A conditional section present against its rule.
    let mut raw = format_3();
    let at = raw
        .position(Section::NodeLngE7.id())
        .unwrap_or_else(|| raw.position(Section::NodeLatLng.id()).unwrap());
    raw.sections.insert(
        at,
        RawSection {
            id: Section::NodeLatE7.id(),
            param: 0,
            flags: SECTION_REQUIRED | SECTION_IDENTITY,
            element_bytes: 4,
            bytes: Vec::new(),
        },
    );
    assert!(open_error(&raw).contains("against its rule"));
}

#[test]
fn sections_may_not_overlap_or_leave_gaps() {
    let raw = format_3();
    let bytes = raw.encode();
    let table_offset = u64::from_le_bytes(bytes[16..24].try_into().unwrap()) as usize;
    let entry_offset = |index: usize| table_offset + index * 64 + 16;
    // The second section moved onto the first.
    let mut overlapping = bytes.clone();
    let first = overlapping[entry_offset(0)..entry_offset(0) + 8].to_vec();
    overlapping[entry_offset(1)..entry_offset(1) + 8].copy_from_slice(&first);
    resign(&mut overlapping);
    // A section moved 64 bytes on, into the next one or the table.
    let mut gap = bytes.clone();
    let at = entry_offset(raw.sections.len() / 2);
    let moved = u64::from_le_bytes(gap[at..at + 8].try_into().unwrap()) + 64;
    gap[at..at + 8].copy_from_slice(&moved.to_le_bytes());
    resign(&mut gap);
    // Trailing bytes after the table.
    let mut longer = bytes.clone();
    longer.extend_from_slice(&[0; 64]);
    // A misaligned table.
    let mut misaligned = bytes.clone();
    misaligned[16..24].copy_from_slice(&(table_offset as u64 + 8).to_le_bytes());
    resign(&mut misaligned);
    for (case, bytes) in [
        ("overlap", overlapping),
        ("gap", gap),
        ("trailing bytes", longer),
        ("misaligned table", misaligned),
    ] {
        let error = open(&bytes).unwrap_err();
        assert!(
            matches!(error, RoutingPackageError::Invalid(_)),
            "{case}: {error}"
        );
    }
}

#[test]
fn unknown_layout_flags_and_header_bytes_are_refused() {
    let mut raw = format_3();
    raw.header[HEADER_V3_LAYOUT_FLAGS + 1] |= 1;
    let error = open(&raw.encode()).unwrap_err();
    assert!(matches!(error, RoutingPackageError::TooNew(_)), "{error}");
    let mut raw = format_3();
    raw.header[200] = 1;
    assert!(open_error(&raw).contains("reserved header bytes"));
    // Too many entries for any package.
    let mut bytes = monaco().1.clone();
    bytes[12..16].copy_from_slice(&5000u32.to_le_bytes());
    resign(&mut bytes);
    assert!(open(&bytes).unwrap_err().to_string().contains("entries"));
}

/// [`Section::PackageParams`] with `edit` applied to its (key, flags,
/// value) entries.
fn with_params(edit: impl FnOnce(&mut Vec<(u32, u32, u64)>)) -> RawPackage {
    let mut raw = format_3();
    let at = raw.position(Section::PackageParams.id()).unwrap();
    let mut entries = raw.sections[at]
        .bytes
        .as_chunks::<16>()
        .0
        .iter()
        .map(|entry| {
            (
                u32::from_le_bytes(entry[0..4].try_into().unwrap()),
                u32::from_le_bytes(entry[4..8].try_into().unwrap()),
                u64::from_le_bytes(entry[8..16].try_into().unwrap()),
            )
        })
        .collect::<Vec<_>>();
    edit(&mut entries);
    raw.sections[at].bytes = entries
        .into_iter()
        .flat_map(|(key, flags, value)| {
            let mut entry = key.to_le_bytes().to_vec();
            entry.extend_from_slice(&flags.to_le_bytes());
            entry.extend_from_slice(&value.to_le_bytes());
            entry
        })
        .collect();
    raw
}

#[test]
fn package_params_are_checked_and_the_draft_revision_must_match() {
    let set_draft = |revision: u64| {
        with_params(|entries| {
            for entry in entries.iter_mut() {
                if entry.0 == param_key::DRAFT_REVISION {
                    entry.2 = revision;
                }
            }
        })
    };
    let newer = open(&set_draft(FORMAT_3_DRAFT_REVISION + 1).encode()).unwrap_err();
    assert!(matches!(newer, RoutingPackageError::TooNew(_)), "{newer}");
    let older = open(&set_draft(FORMAT_3_DRAFT_REVISION - 1).encode()).unwrap_err();
    assert!(matches!(older, RoutingPackageError::Outdated(_)), "{older}");
    // Without a draft revision it claims to be the released format 3.
    let released =
        with_params(|entries| entries.retain(|entry| entry.0 != param_key::DRAFT_REVISION));
    let error = open(&released.encode()).unwrap_err();
    assert!(matches!(error, RoutingPackageError::TooNew(_)), "{error}");

    // An unknown key is skipped unless it must be understood.
    let skipped = with_params(|entries| entries.push((40, 0, 9)));
    assert!(open(&skipped.encode()).is_ok());
    let needed = with_params(|entries| entries.push((40, PARAM_MUST_UNDERSTAND, 9)));
    let error = open(&needed.encode()).unwrap_err();
    assert!(matches!(error, RoutingPackageError::TooNew(_)), "{error}");

    let missing = with_params(|entries| entries.retain(|entry| entry.0 != param_key::REACH_CAP));
    assert!(open_error(&missing).contains("missing"));
    let unsorted = with_params(|entries| entries.swap(0, 1));
    assert!(open_error(&unsorted).contains("sorted"));
    let wrong_flags = with_params(|entries| entries[0].1 = 0);
    assert!(open_error(&wrong_flags).contains("flags"));
    // Values are checked as the JSON's were.
    let bad_sar = with_params(|entries| {
        for entry in entries.iter_mut() {
            if entry.0 == param_key::SAR_WRONG_WAY_MIN_FACTOR {
                entry.2 = 0.5f64.to_bits();
            }
        }
    });
    assert!(open_error(&bad_sar).contains("sar"));
    // The costing model comes from the parameters, not the JSON.
    let newer_model = with_params(|entries| {
        for entry in entries.iter_mut() {
            if entry.0 == param_key::COSTING_MODEL_VERSION {
                entry.2 = 9_999;
            }
        }
    });
    assert_eq!(
        open(&newer_model.encode())
            .unwrap()
            .metadata()
            .costing_model_version,
        9_999
    );
    assert!(RoutingPackage::from_bytes(&newer_model.encode()).is_err());
}

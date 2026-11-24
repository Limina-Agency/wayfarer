//! Format 3's section table and profile slots (`docs/design/format-3.md`,
//! sections 2 to 5 and 7): a format-3 compile routes as format 2 does, the
//! identity and content hashes, the slot table and u16 masks, versioned
//! attribute-class records, the region info and OSM node ids, and the
//! checks against hostile packages.

use std::path::Path;
use std::sync::{Arc, OnceLock};

use wayfarer::{
    GeoPoint, MobilityProfile, RouteMatrixRequest, RouteRequest, RoutingEngine, RoutingPackage,
};
use wayfarer_compiler::{CancelToken, CompileRequest, compile};
use wayfarer_package::format::{
    FORMAT_3_DRAFT_REVISION, HEADER_V3_LAYOUT_FLAGS, LEG_ATTRS_REVISION, NO_SLOT_COLUMN,
    NO_SOURCE_NODE_HIGH, NODE_OWNER_SECTION_ID, PARAM_MUST_UNDERSTAND, PROFILE_SLOT_BYTES,
    PackageFile, PackageFormat, PackageParams, ProfileSlot, REGION_HAS_NODE_OWNER,
    REGION_INFO_BYTES, RegionInfo, RoutingPackageError, SECTION_IDENTITY, SECTION_REQUIRED,
    Section, param_key, profile_code,
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
    // Every profile: format 3 numbers their slots differently.
    for profile in ALL_PROFILES {
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
        // A flipped byte in a section, or in the padding, is caught. It goes
        // to another file: Windows refuses to rewrite one `package` maps.
        let mut bytes = std::fs::read(&path).unwrap();
        let last = bytes.len() - 1;
        bytes[last] ^= 1;
        let flipped = dir.path().join(format!("flipped-{table_first}.mrt"));
        std::fs::write(&flipped, &bytes).unwrap();
        assert!(PackageFile::verify(&flipped).is_err());
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

const ALL_PROFILES: [MobilityProfile; 6] = [
    MobilityProfile::Default,
    MobilityProfile::Wheeled,
    MobilityProfile::Tracked,
    MobilityProfile::Emergency,
    MobilityProfile::OnFoot,
    MobilityProfile::Legged,
];

/// Every mask of both packages admits the same profiles, through each
/// package's own bits: format 3 numbers five slots (`Default` shares
/// `Wheeled`'s) where format 2 has six fixed ones.
#[test]
fn format_3_numbers_its_profile_slots_and_widens_the_masks() {
    let (v2, v3) = monaco();
    let (v2, v3) = (open(v2).unwrap(), open(v3).unwrap());
    assert_eq!(v2.profiles().slot_count(), 6);
    assert_eq!(v3.profiles().slot_count(), 5);
    assert_eq!(
        ALL_PROFILES.map(|profile| v3.profile_bit(profile)),
        [1, 1, 2, 4, 8, 16]
    );
    assert_eq!(
        ALL_PROFILES.map(|profile| v2.profile_bit(profile)),
        [1, 2, 4, 8, 16, 32]
    );
    for profile in ALL_PROFILES {
        assert_eq!(
            v2.column_for(profile),
            v3.column_for(profile),
            "{profile:?}"
        );
        assert_eq!(
            v2.heuristic_cost_per_m(profile).to_bits(),
            v3.heuristic_cost_per_m(profile).to_bits(),
            "{profile:?}"
        );
    }
    let names = v3
        .section_sizes()
        .into_iter()
        .map(|(name, _)| name)
        .collect::<Vec<_>>();
    for wide in [
        "ProfileSlots",
        "EdgeAccess",
        "SimpleMask16",
        "ComplexMask16",
        "BarrierAccess32",
        "NodeReachSaturated16",
    ] {
        assert!(names.contains(&wide), "{wide}");
    }
    for narrow in [
        "EdgeProfileAccess",
        "SimpleMask",
        "ComplexMask",
        "BarrierAccess",
        "NodeReachSaturated",
    ] {
        assert!(!names.contains(&narrow), "{narrow}");
    }
    // The header no longer holds profile columns or heuristic costs.
    let header = &monaco().1[..256];
    assert!(header[88..224].iter().all(|byte| *byte == 0));
    let (a, b) = (v2.view(), v3.view());
    let same = |what: &str, count: usize, mask: &dyn Fn(bool, usize) -> u16| {
        for index in 0..count {
            for profile in ALL_PROFILES {
                assert_eq!(
                    mask(false, index) & v2.profile_bit(profile) != 0,
                    mask(true, index) & v3.profile_bit(profile) != 0,
                    "{what} {index} {profile:?}"
                );
            }
        }
    };
    same("edge", a.edge_count, &|wide, edge| {
        if wide {
            b.edge_access.profiles(edge)
        } else {
            a.edge_access.profiles(edge)
        }
    });
    for edge in 0..a.edge_count {
        assert_eq!(
            a.edge_access.sar(edge),
            b.edge_access.sar(edge),
            "edge {edge}"
        );
    }
    same("simple restriction", a.simple_mask.len(), &|wide, row| {
        if wide {
            b.simple_mask.get(row)
        } else {
            a.simple_mask.get(row)
        }
    });
    same("complex restriction", a.complex_mask.len(), &|wide, row| {
        if wide {
            b.complex_mask.get(row)
        } else {
            a.complex_mask.get(row)
        }
    });
    let saturated = [&v2, &v3].map(|package| {
        package.profile_masks(Section::NodeReachSaturated, Section::NodeReachSaturated16)
    });
    same("node", a.node_count, &|wide, node| {
        saturated[usize::from(wide)].get(node)
    });
    assert_eq!(a.barrier_access.len(), b.barrier_access.len());
    for at in 0..a.barrier_access.len() {
        for profile in ALL_PROFILES {
            assert_eq!(
                a.barrier_access.get(at).effect(v2.profile_bit(profile)),
                b.barrier_access.get(at).effect(v3.profile_bit(profile)),
                "barrier {at} {profile:?}"
            );
        }
    }
}

/// The format-3 package with its slots replaced by `edit`'s.
fn with_slots(edit: impl FnOnce(&mut Vec<ProfileSlot>)) -> RawPackage {
    let mut raw = format_3();
    let at = raw.position(Section::ProfileSlots.id()).unwrap();
    let mut slots = raw.sections[at]
        .bytes
        .as_chunks::<PROFILE_SLOT_BYTES>()
        .0
        .iter()
        .map(|bytes| ProfileSlot::from_bytes(bytes).unwrap())
        .collect::<Vec<_>>();
    edit(&mut slots);
    raw.sections[at].bytes = slots.into_iter().flat_map(ProfileSlot::to_bytes).collect();
    raw
}

#[test]
fn profile_slots_are_checked() {
    // Flags and reserved bytes are zero.
    for byte in [2, 3, 16, 31] {
        let mut raw = format_3();
        let at = raw.position(Section::ProfileSlots.id()).unwrap();
        raw.sections[at].bytes[byte] = 1;
        assert!(open_error(&raw).contains("profile slots"), "byte {byte}");
    }
    type Edit = fn(&mut Vec<ProfileSlot>);
    let cases: [(&str, Edit); 6] = [
        ("profile slot codes", |slots| slots[1].code = slots[0].code),
        ("profile slot codes", |slots| slots[2].code = 0),
        ("too many profile slots", |slots| {
            for code in 0..10 {
                slots.push(ProfileSlot {
                    code: profile_code::PRIVATE + code,
                    column: NO_SLOT_COLUMN,
                    heuristic_cost_per_m: 0.0,
                });
            }
        }),
        ("profile column out of range", |slots| slots[0].column = 7),
        ("heuristic", |slots| slots[3].heuristic_cost_per_m = -1.0),
        ("heuristic", |slots| {
            slots[3].heuristic_cost_per_m = f64::NAN
        }),
    ];
    for (expected, edit) in cases {
        let error = open_error(&with_slots(edit));
        assert!(error.contains(expected), "{expected}: {error}");
    }
}

/// A 1.x compiler may add a slot this build doesn't know (a bicycle, say):
/// the package still opens, its bit matches no profile, and every route is
/// the same.
#[test]
fn a_slot_of_an_unknown_profile_is_kept_and_matches_no_profile() {
    let mut raw = with_slots(|slots| {
        slots.push(ProfileSlot {
            code: profile_code::BICYCLE,
            column: NO_SLOT_COLUMN,
            heuristic_cost_per_m: 0.25,
        })
    });
    // Bicycles may use every edge.
    let at = raw.position(Section::EdgeAccess.id()).unwrap();
    for pair in raw.sections[at].bytes.as_chunks_mut::<2>().0 {
        pair[0] |= 1 << 5;
    }
    let bytes = raw.encode();
    let package = open(&bytes).unwrap();
    assert_eq!(package.profiles().slot_count(), 6);
    assert_eq!(package.profile_bit(MobilityProfile::Wheeled), 1);
    assert_eq!(package.profile_bit(MobilityProfile::Legged), 16);
    let engines = [&monaco().1, &bytes].map(|bytes| {
        RoutingEngine::from_package(Arc::new(RoutingPackage::from_bytes(bytes).unwrap()))
    });
    let points = points();
    for profile in [MobilityProfile::Tracked, MobilityProfile::Legged] {
        for to in &points[1..] {
            let [a, b] = engines.each_ref().map(|engine| {
                without_timings(format!(
                    "{:?}",
                    engine.route(RouteRequest::new(points[0], *to, profile))
                ))
            });
            assert_eq!(a, b, "{profile:?} -> {to:?}");
        }
    }
}

/// A request for a profile the package has no slot for is refused with a
/// typed error.
#[test]
fn a_profile_without_a_slot_is_refused() {
    let raw = with_slots(|slots| {
        let tracked = slots
            .iter_mut()
            .find(|slot| slot.code == profile_code::TRACKED)
            .unwrap();
        tracked.code = profile_code::PRIVATE;
    });
    let bytes = raw.encode();
    assert_eq!(
        open(&bytes).unwrap().profile_bit(MobilityProfile::Tracked),
        0
    );
    let engine = RoutingEngine::from_package(Arc::new(RoutingPackage::from_bytes(&bytes).unwrap()));
    let points = points();
    let route = |profile| engine.route(RouteRequest::new(points[0], points[1], profile));
    let refused = format!("{:?}", route(MobilityProfile::Tracked).unwrap_err().error);
    assert!(refused.contains("ImpossibleMobilityProfile"), "{refused}");
    assert!(route(MobilityProfile::Wheeled).is_ok());
}

/// Barrier access names only bits of the package's slots.
#[test]
fn barrier_access_is_checked_against_the_slot_count() {
    let mut raw = format_3();
    let at = raw.position(Section::BarrierAccess32.id()).unwrap();
    assert!(
        !raw.sections[at].bytes.is_empty(),
        "Monaco has barrier nodes"
    );
    // A gate for a sixth slot.
    raw.sections[at].bytes[2] |= 1 << 5;
    assert!(open_error(&raw).contains("barrier"));
}

/// Every shared info's leg attributes, as legs report them.
fn leg_infos(bytes: &[u8]) -> Vec<wayfarer_package::SharedEdgeInfo> {
    let file = open(bytes).unwrap();
    let view = file.request_view().unwrap();
    let infos: Vec<_> = (0..)
        .map_while(|shared| view.class_parts(shared).map(|_| shared))
        .map(|shared| view.leg_shared_info(shared).unwrap())
        .collect();
    assert!(!infos.is_empty());
    infos
}

/// The attribute-class records carry their revision in the table entry's
/// `param` (design section 5.3), and a newer revision's records, with
/// fields this build doesn't know appended, still read.
#[test]
fn attribute_class_records_carry_their_revision() {
    let raw = format_3();
    let at = raw.position(Section::AttrClassBytes.id()).unwrap();
    assert_eq!(raw.sections[at].param, LEG_ATTRS_REVISION);
    let mut bad = raw.clone();
    bad.sections[at].param = 0;
    assert!(
        open_error(&bad).contains("parameter"),
        "{}",
        open_error(&bad)
    );
    // Two revisions of the records sort apart but are still one section.
    let mut twice = raw.clone();
    let mut copy = twice.sections[at].clone();
    copy.param = LEG_ATTRS_REVISION + 1;
    twice.sections.insert(at + 1, copy);
    assert!(
        open_error(&twice).contains("more than once"),
        "{}",
        open_error(&twice)
    );

    // A newer revision: three more bytes after each record's leg part.
    let before = leg_infos(&monaco().1);
    let starts_at = raw.position(Section::AttrClassStart.id()).unwrap();
    let starts: Vec<u32> = raw.sections[starts_at]
        .bytes
        .as_chunks::<4>()
        .0
        .iter()
        .map(|bytes| u32::from_le_bytes(*bytes))
        .collect();
    let old_bytes = &raw.sections[at].bytes;
    let (mut new_starts, mut new_bytes) = (vec![0u32], Vec::new());
    for pair in starts.windows(2) {
        new_bytes.extend_from_slice(&old_bytes[pair[0] as usize..pair[1] as usize]);
        new_bytes.extend_from_slice(&[1, 9, 0]);
        new_starts.push(new_bytes.len() as u32);
    }
    let mut newer = raw.clone();
    newer.sections[starts_at].bytes = new_starts.iter().flat_map(|s| s.to_le_bytes()).collect();
    newer.sections[at].bytes = new_bytes;
    newer.sections[at].param = LEG_ATTRS_REVISION + 1;
    assert_eq!(leg_infos(&newer.encode()), before);
    // The same bytes claiming this revision have bytes after their fields.
    newer.sections[at].param = LEG_ATTRS_REVISION;
    let file = open(&newer.encode()).unwrap();
    assert!(file.request_view().unwrap().leg_shared_info(0).is_none());
}

/// Every node states its OSM node id, the reverse lookup finds it, and the
/// region info is a stand-alone package's over the nodes' extent.
#[test]
fn format_3_states_osm_node_ids_and_a_stand_alone_region() {
    let file = open(&monaco().1).unwrap();
    let view = file.view();
    let mut with_id = 0;
    let mut extent = [i32::MAX, i32::MAX, i32::MIN, i32::MIN];
    for node in 0..view.node_count as u32 {
        if let Some(id) = view.osm_node_id(node) {
            with_id += 1;
            assert!(id > 0);
            assert!(view.nodes_with_osm_id(id).contains(&node), "node {node}");
        }
        let (lat, lng) = view.node_e7(node);
        extent = [
            extent[0].min(lat),
            extent[1].min(lng),
            extent[2].max(lat),
            extent[3].max(lng),
        ];
    }
    // Monaco's only nodes without one are crossings the compiler inserted.
    assert!(with_id * 100 >= view.node_count * 99, "{with_id}");
    assert_eq!(file.region_info(), Some(&RegionInfo::stand_alone(extent)));
    let v2 = open(&monaco().0).unwrap();
    assert_eq!(v2.region_info(), None);
    assert!(
        v2.view()
            .nodes_with_osm_id(view.osm_node_id(0).unwrap())
            .is_empty()
    );
    for section in [
        Section::RegionInfo,
        Section::NodeSourceIdLow,
        Section::NodeSourceIdHigh,
    ] {
        assert_eq!(
            section.format_3_flags(),
            SECTION_REQUIRED | SECTION_IDENTITY
        );
        assert!(!v2.has_section(section));
    }
}

/// The format-3 package with its region info replaced by `edit`'s.
fn with_region(edit: impl FnOnce(&mut RegionInfo)) -> RawPackage {
    let mut raw = format_3();
    let at = raw.position(Section::RegionInfo.id()).unwrap();
    let mut region = RegionInfo::from_bytes(raw.sections[at].bytes.as_slice().try_into().unwrap());
    edit(&mut region);
    raw.sections[at].bytes = region.to_bytes().to_vec();
    raw
}

#[test]
fn region_info_is_checked() {
    // A bundle member's ids, boundary, halo and an unknown cut rule are kept.
    let raw = with_region(|region| {
        region.region_id = [7; 16];
        region.bundle_id = [9; 16];
        region.boundary_hash = [3; 32];
        region.halo_m = 2_000;
        region.cut_rule = 99;
        region.cut_param = 12;
    });
    let file = open(&raw.encode()).unwrap();
    let region = file.region_info().unwrap();
    assert_eq!((region.region_id, region.cut_rule), ([7; 16], 99));
    // Its identity changes with them.
    assert_ne!(
        file.dataset_hash(),
        open(&monaco().1).unwrap().dataset_hash()
    );
    // A flag this build doesn't know needs a newer build.
    let error = open(&with_region(|region| region.flags = 1 << 4).encode()).unwrap_err();
    assert!(matches!(error, RoutingPackageError::TooNew(_)), "{error}");
    // Stub nodes without the `NodeOwner` section that lists them.
    let error = open_error(&with_region(|region| region.flags = REGION_HAS_NODE_OWNER));
    assert!(error.contains("node owner"), "{error}");
    let mut owner = format_3();
    owner.sections.push(RawSection {
        id: NODE_OWNER_SECTION_ID,
        param: 0,
        flags: SECTION_IDENTITY,
        element_bytes: 8,
        bytes: vec![0; 8],
    });
    owner.sections.sort_by_key(|section| section.id);
    let error = open_error(&owner);
    assert!(error.contains("node owner"), "{error}");
    let at = owner.position(Section::RegionInfo.id()).unwrap();
    owner.sections[at].bytes[92] = REGION_HAS_NODE_OWNER as u8;
    assert!(open(&owner.encode()).is_ok());
    // The extent is a box on the globe.
    type Edit = fn(&mut RegionInfo);
    let cases: [Edit; 3] = [
        |region| region.extent_e7.swap(0, 2),
        |region| region.extent_e7[2] = 900_000_001,
        |region| region.extent_e7[1] = -1_800_000_001,
    ];
    for edit in cases {
        assert!(open_error(&with_region(edit)).contains("region extent"));
    }
    // Exactly one region info.
    let mut raw = format_3();
    let at = raw.position(Section::RegionInfo.id()).unwrap();
    raw.sections[at].bytes.extend([0; REGION_INFO_BYTES]);
    assert!(open_error(&raw).contains("one region info"));
}

#[test]
fn osm_node_ids_are_checked() {
    // A node without an OSM node has a zero low part.
    let mut raw = format_3();
    let high = raw.position(Section::NodeSourceIdHigh.id()).unwrap();
    raw.sections[high].bytes[0..2].copy_from_slice(&NO_SOURCE_NODE_HIGH.to_le_bytes());
    let low = raw.position(Section::NodeSourceIdLow.id()).unwrap();
    raw.sections[low].bytes[0..4].copy_from_slice(&5u32.to_le_bytes());
    assert!(open_error(&raw).contains("node source ids"));
    raw.sections[low].bytes[0..4].fill(0);
    let file = open(&raw.encode()).unwrap();
    assert_eq!(file.view().osm_node_id(0), None);
    // One id per node.
    for section in [Section::NodeSourceIdLow, Section::NodeSourceIdHigh] {
        let mut raw = format_3();
        let at = raw.position(section.id()).unwrap();
        let element = raw.sections[at].element_bytes as usize;
        let len = raw.sections[at].bytes.len();
        raw.sections[at].bytes.truncate(len - element);
        assert!(open_error(&raw).contains("header counts"), "{section:?}");
    }
}

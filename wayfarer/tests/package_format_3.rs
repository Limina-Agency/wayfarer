//! Format 3's section table and profile slots (`docs/design/format-3.md`,
//! sections 2 to 5 and 7): a format-3 compile routes as format 2 does, the
//! identity and content hashes, the slot table and u16 masks, versioned
//! attribute-class records, the region info and OSM node ids, rank
//! numbering and the cell tree, the wide offsets and the lattice request
//! grid, and the checks against hostile packages.

use std::path::Path;
use std::sync::{Arc, OnceLock};

use wayfarer::{
    GeoPoint, MobilityProfile, RouteMatrixRequest, RouteRequest, RoutingEngine, RoutingPackage,
};
use wayfarer_compiler::{CancelToken, CompileRequest, compile};
use wayfarer_package::cch_order::{CELL_TREE_ENTRY_BYTES, CellTreeEntry, NO_PARENT};
use wayfarer_package::format::{
    HEADER_V3_LAYOUT_FLAGS, LAYOUT_NODES_BY_RANK, LEG_ATTRS_REVISION, NO_SLOT_COLUMN,
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

/// Format 3 numbers nodes by CCH rank, so its nodes, edges and shared infos
/// are format 2's in another order: each format-3 element's format-2
/// counterpart, found by position (nodes) and by ends, length and way
/// (edges).
struct Correspondence {
    node: Vec<u32>,
    edge: Vec<u32>,
    shared: Vec<u32>,
}

impl Correspondence {
    fn of(v2: &PackageFile, v3: &PackageFile) -> Self {
        let (a, b) = (v2.view(), v3.view());
        let position = |view: &wayfarer_package::format::Sections<'_>, node: u32| {
            let (lat, lng) = view.node_lat_lng(node);
            (lat.to_bits(), lng.to_bits())
        };
        let by_position: std::collections::HashMap<_, u32> = (0..a.node_count as u32)
            .map(|node| (position(&a, node), node))
            .collect();
        assert_eq!(by_position.len(), a.node_count, "node positions repeat");
        let node = (0..b.node_count as u32)
            .map(|node| by_position[&position(&b, node)])
            .collect::<Vec<_>>();
        let source = |view: &wayfarer_package::format::Sections<'_>| {
            let mut source = vec![0u32; view.edge_count];
            for n in 0..view.node_count {
                for edge in view.first_out[n]..view.first_out[n + 1] {
                    source[edge as usize] = n as u32;
                }
            }
            source
        };
        let (source_a, source_b) = (source(&a), source(&b));
        let key = |view: &wayfarer_package::format::Sections<'_>, source: u32, edge: usize| {
            (
                source,
                view.edge_target[edge],
                view.edge_length[edge].to_bits(),
                view.si_way.get(view.edge_shared_info[edge] as usize),
            )
        };
        // Parallel edges of one way with one length (a way that doubles
        // back) share a key: they are taken in order, as both formats keep
        // a node's edges in one order.
        let mut by_key: std::collections::HashMap<_, std::collections::VecDeque<u32>> =
            std::collections::HashMap::new();
        for (edge, source) in source_a.iter().enumerate() {
            by_key
                .entry(key(&a, *source, edge))
                .or_default()
                .push_back(edge as u32);
        }
        let edge = (0..b.edge_count)
            .map(|edge| {
                let (from, to, length, way) = key(&b, source_b[edge], edge);
                by_key
                    .get_mut(&(node[from as usize], node[to as usize], length, way))
                    .and_then(|edges| edges.pop_front())
                    .unwrap_or_else(|| panic!("edge {edge} has no format-2 counterpart"))
            })
            .collect::<Vec<_>>();
        assert!(
            by_key.values().all(|edges| edges.is_empty()),
            "format 2 has more edges"
        );
        let mut shared = vec![u32::MAX; v3.counts().shared_info_count];
        for (at, edge) in edge.iter().enumerate() {
            let mapped = a.edge_shared_info[*edge as usize];
            let slot = &mut shared[b.edge_shared_info[at] as usize];
            assert!(
                *slot == u32::MAX || *slot == mapped,
                "edge {at}'s shared info"
            );
            *slot = mapped;
        }
        Self { node, edge, shared }
    }

    /// Format 3's edge id of format 2's `edge_id`.
    fn edge_id_in_3(&self, v2: &PackageFile, v3: &PackageFile, edge_id: u64) -> u64 {
        let edge = v2.view().edge_for_id(edge_id).unwrap();
        let at = self.edge.iter().position(|e| *e == edge).unwrap();
        v3.view().edge_id(at)
    }

    /// `value` (a format-3 result as JSON) with its ids made format 2's.
    fn to_format_2(&self, v2: &PackageFile, v3: &PackageFile, value: &mut serde_json::Value) {
        let (a, b) = (v2.view(), v3.view());
        let edge_id = |id: u64| {
            b.edge_for_id(id)
                .map_or(id, |edge| a.edge_id(self.edge[edge as usize] as usize))
        };
        match value {
            serde_json::Value::Object(map) => {
                for (key, item) in map.iter_mut() {
                    let map_number = |item: &mut serde_json::Value, f: &dyn Fn(u64) -> u64| {
                        if let Some(id) = item.as_u64() {
                            *item = f(id).into();
                        }
                    };
                    match key.as_str() {
                        "edgeId" | "opposingEdgeId" | "sourceEdgeId" | "targetEdgeId" => {
                            map_number(item, &edge_id)
                        }
                        "edgeIds" => {
                            for id in item.as_array_mut().into_iter().flatten() {
                                map_number(id, &edge_id);
                            }
                        }
                        "sharedEdgeInfoId" => {
                            map_number(item, &|id| u64::from(self.shared[(id - 1) as usize]) + 1)
                        }
                        "sourceNodeId" | "targetNodeId" => map_number(item, &|id| {
                            let (ra, rb) = (v2.request_view().unwrap(), v3.request_view().unwrap());
                            let node = (id as i64 - rb.node_id(0)) as usize;
                            ra.node_id(self.node[node]) as u64
                        }),
                        _ => self.to_format_2(v2, v3, item),
                    }
                }
            }
            serde_json::Value::Array(items) => {
                for item in items {
                    self.to_format_2(v2, v3, item);
                }
            }
            _ => {}
        }
    }
}

fn monaco_correspondence() -> &'static Correspondence {
    static MAP: OnceLock<Correspondence> = OnceLock::new();
    MAP.get_or_init(|| Correspondence::of(&open(&monaco().0).unwrap(), &open(&monaco().1).unwrap()))
}

/// `value` with the lists whose order follows node or edge ids sorted:
/// joining edges come in their source nodes' order, rejected candidates in
/// edge id order. Guidance reads the angles as a set.
fn sort_id_ordered(value: &mut serde_json::Value) {
    match value {
        serde_json::Value::Object(map) => {
            for (key, item) in map.iter_mut() {
                sort_id_ordered(item);
                let Some(items) = item.as_array_mut() else {
                    continue;
                };
                if key == "junctionJoiningAnglesDeg" {
                    items.sort_by_key(|item| item.as_i64());
                } else if key.ends_with("RejectedCandidates") {
                    items.sort_by_key(|item| item["edgeId"].as_u64());
                }
            }
        }
        serde_json::Value::Array(items) => items.iter_mut().for_each(sort_id_ordered),
        _ => {}
    }
}

/// `value` with every key ending in `Ns`, `Ms` or `_ns`/`_ms` (timings)
/// zeroed.
fn zero_timings(value: &mut serde_json::Value) {
    match value {
        serde_json::Value::Object(map) => {
            for (key, item) in map.iter_mut() {
                if ["Ns", "Ms", "_ns", "_ms"]
                    .iter()
                    .any(|end| key.ends_with(end))
                    && item.is_number()
                {
                    *item = 0.into();
                } else {
                    zero_timings(item);
                }
            }
        }
        serde_json::Value::Array(items) => items.iter_mut().for_each(zero_timings),
        _ => {}
    }
}

/// Every route, and a matrix, give the same answer on both formats: the
/// same graph and costs, read through the other table and numbered by rank,
/// so the same paths under format 3's ids.
#[test]
fn a_format_3_package_routes_as_format_2_does() {
    let (v2, v3) = monaco();
    let files = [open(v2).unwrap(), open(v3).unwrap()];
    let map = monaco_correspondence();
    let engines = [v2, v3].map(|bytes| {
        RoutingEngine::from_package(Arc::new(RoutingPackage::from_bytes(bytes).unwrap()))
    });
    let points = points();
    // Blocked edges are named by id: the engine maps them to edges through
    // the stored ids (format 2) or the implied ones (format 3).
    let base = files[0].view().edge_id_base;
    let blocked: std::collections::BTreeSet<u64> =
        (0..4_000).step_by(7).map(|slot| base + slot).collect();
    let blocked_3: std::collections::BTreeSet<u64> = blocked
        .iter()
        .map(|id| map.edge_id_in_3(&files[0], &files[1], *id))
        .collect();
    let as_json = |result: &dyn erased::Json, format_3: bool| {
        let mut value = result.json();
        zero_timings(&mut value);
        if format_3 {
            map.to_format_2(&files[0], &files[1], &mut value);
        }
        sort_id_ordered(&mut value);
        value
    };
    // Every profile: format 3 numbers their slots differently.
    for profile in ALL_PROFILES {
        for from in &points {
            for to in &points {
                // A route from a point to itself ties between an edge and
                // its twin, and ties go to the lower edge id.
                if from == to {
                    continue;
                }
                for block in [false, true] {
                    let [a, b] = [0, 1].map(|at| {
                        let mut request = RouteRequest::new(*from, *to, profile);
                        if block {
                            request.options.blocked_edge_ids =
                                std::collections::BTreeSet::clone(if at == 0 {
                                    &blocked
                                } else {
                                    &blocked_3
                                });
                        }
                        let route = engines[at].route(request);
                        match route {
                            Ok(route) => as_json(&route, at == 1),
                            Err(failure) => {
                                let mut value = as_json(&*failure.diagnostics, at == 1);
                                value["error"] = format!("{:?}", failure.error).into();
                                value["segmentIndex"] = failure.segment_index.into();
                                value
                            }
                        }
                    });
                    assert_same(
                        &a,
                        &b,
                        &format!("{profile:?} {from:?} -> {to:?} blocked {block}"),
                    );
                }
            }
        }
    }
    let matrix = |at: usize| {
        let request: RouteMatrixRequest = serde_json::from_value(serde_json::json!({
            "sources": points.iter().map(|p| serde_json::json!({"lat": p.lat, "lng": p.lng})).collect::<Vec<_>>(),
            "targets": points.iter().map(|p| serde_json::json!({"lat": p.lat, "lng": p.lng})).collect::<Vec<_>>(),
            "profile": "wheeled",
        }))
        .unwrap();
        as_json(&engines[at].matrix(request), at == 1)
    };
    // The diagonal ties as a route from a point to itself does.
    let off_diagonal = |mut value: serde_json::Value| {
        let cells = value["cells"].as_array_mut().unwrap();
        for at in 0..points.len() {
            cells[at * points.len() + at]["edgeIds"] = serde_json::Value::Null;
        }
        value
    };
    assert_same(
        &off_diagonal(matrix(0)),
        &off_diagonal(matrix(1)),
        "the matrix",
    );
}

/// Panics naming the first few paths where `a` and `b` differ.
fn assert_same(a: &serde_json::Value, b: &serde_json::Value, what: &str) {
    fn walk(a: &serde_json::Value, b: &serde_json::Value, path: String, out: &mut Vec<String>) {
        use serde_json::Value::{Array, Object};
        if out.len() >= 8 || a == b {
            return;
        }
        match (a, b) {
            (Object(x), Object(y)) if x.len() == y.len() => {
                for (key, item) in x {
                    match y.get(key) {
                        Some(other) => walk(item, other, format!("{path}.{key}"), out),
                        None => out.push(format!("{path}.{key}: missing")),
                    }
                }
            }
            (Array(x), Array(y)) if x.len() == y.len() => {
                for (at, (item, other)) in x.iter().zip(y).enumerate() {
                    walk(item, other, format!("{path}[{at}]"), out);
                }
            }
            _ => out.push(format!("{path}: {a} vs {b}")),
        }
    }
    let mut out = Vec::new();
    walk(a, b, String::new(), &mut out);
    assert!(out.is_empty(), "{what}:\n{}", out.join("\n"));
}

/// Results as JSON, whatever their type.
mod erased {
    pub trait Json {
        fn json(&self) -> serde_json::Value;
    }
    impl<T: serde::Serialize> Json for T {
        fn json(&self) -> serde_json::Value {
            serde_json::to_value(self).unwrap()
        }
    }
}

#[test]
fn format_3_drops_stored_ids_and_reserved_sections_and_states_its_params() {
    let (v2, v3) = monaco();
    let (v2, v3) = (open(v2).unwrap(), open(v3).unwrap());
    assert_eq!((v2.format_version(), v3.format_version()), (2, 3));
    assert_eq!(v2.counts(), v3.counts());
    // Smaller but for the request grid: a lattice block is 16 KiB of starts,
    // more than Monaco's whole dense grid.
    let grid_bytes = |file: &PackageFile| {
        file.section_sizes()
            .into_iter()
            .filter(|(name, _)| {
                [
                    "RequestGrid",
                    "GridCellStart",
                    "GridBlocks",
                    "GridBlockCellStart",
                ]
                .contains(name)
            })
            .map(|(_, len)| len)
            .sum::<usize>()
    };
    assert!(v3.byte_len() - grid_bytes(&v3) < v2.byte_len() - grid_bytes(&v2));
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
    // Ids are implied by position: edges from the base, nodes in rank
    // order, shared infos from 1. Format 2 stores its own.
    let (a, b) = (v2.view(), v3.view());
    let map = monaco_correspondence();
    assert_eq!(a.edge_id_base, b.edge_id_base);
    for edge in 0..b.edge_count {
        assert_eq!(b.edge_id(edge), b.edge_id_base + edge as u64);
        assert_eq!(b.edge_for_id(b.edge_id(edge)), Some(edge as u32));
        let twin = map.edge[edge] as usize;
        assert_eq!(a.edge_for_id(a.edge_id(twin)), Some(twin as u32));
    }
    assert_eq!(b.edge_for_id(b.edge_id_base + b.edge_count as u64), None);
    // A node's edges keep their order (by way, direction, target position,
    // segment and length), and shared infos are numbered by first edge.
    for node in 0..b.node_count {
        let twin = map.node[node] as usize;
        assert_eq!(
            (b.first_out[node]..b.first_out[node + 1])
                .map(|edge| map.edge[edge as usize])
                .collect::<Vec<_>>(),
            (a.first_out[twin]..a.first_out[twin + 1]).collect::<Vec<_>>(),
            "node {node}"
        );
    }
    let mut next_shared = 0;
    for edge in 0..b.edge_count {
        let shared = b.edge_shared_info[edge];
        assert!(shared <= next_shared, "edge {edge}");
        next_shared = next_shared.max(shared + 1);
    }
    let (ra, rb) = (v2.request_view().unwrap(), v3.request_view().unwrap());
    let mut components = std::collections::HashMap::new();
    let mut back = std::collections::HashMap::new();
    for node in 0..b.node_count as u32 {
        assert_eq!(rb.node_id(node), rb.node_id(0) + i64::from(node));
        let (in_2, in_3) = (
            ra.component_id(map.node[node as usize]),
            rb.component_id(node),
        );
        assert_eq!(*components.entry(in_3).or_insert(in_2), in_2, "node {node}");
        assert_eq!(*back.entry(in_2).or_insert(in_3), in_3, "node {node}");
        assert_eq!(
            ra.component_size(map.node[node as usize]),
            rb.component_size(node)
        );
    }
    let mut seen = vec![false; v2.counts().shared_info_count];
    for shared in 0..v3.counts().shared_info_count {
        assert_eq!(rb.shared_info_id(shared), shared as u64 + 1);
        assert!(!std::mem::replace(
            &mut seen[map.shared[shared] as usize],
            true
        ));
    }
    let (pa, pb) = (v2.params(), v3.params());
    // Components are numbered in node order, so the default one is the
    // same component under another id.
    assert_eq!(
        back[&Some(pa.default_component)],
        Some(pb.default_component)
    );
    assert_eq!(
        PackageParams {
            default_component: pa.default_component,
            request_grid_spacing: None,
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
fn package_params_are_checked_and_drafts_are_refused() {
    // Released format 3 writes no draft revision.
    assert!(
        !with_params(|_| {}).sections.iter().any(|section| section.id
            == Section::PackageParams.id()
            && section.bytes.as_chunks::<16>().0.iter().any(|entry| {
                u32::from_le_bytes(entry[0..4].try_into().unwrap()) == param_key::DRAFT_REVISION
            }))
    );
    // A development build's draft, of any revision, is refused as outdated.
    for revision in [1, 6, 7] {
        let draft = with_params(|entries| {
            let at = entries.partition_point(|entry| entry.0 < param_key::DRAFT_REVISION);
            entries.insert(
                at,
                (param_key::DRAFT_REVISION, PARAM_MUST_UNDERSTAND, revision),
            );
        });
        let error = open(&draft.encode()).unwrap_err();
        assert!(
            matches!(error, RoutingPackageError::Outdated(_))
                && error.to_string().contains("draft"),
            "{error}"
        );
    }

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
    // Format 3 numbers nodes and edges by rank: each element is compared
    // with its format-2 counterpart, and restriction rows as multisets.
    let (a, b) = (v2.view(), v3.view());
    let map = monaco_correspondence();
    let profiles = |package: &PackageFile, mask: u16| {
        ALL_PROFILES.map(|profile| mask & package.profile_bit(profile) != 0)
    };
    for edge in 0..b.edge_count {
        let twin = map.edge[edge] as usize;
        assert_eq!(
            profiles(&v2, a.edge_access.profiles(twin)),
            profiles(&v3, b.edge_access.profiles(edge)),
            "edge {edge}"
        );
        assert_eq!(
            a.edge_access.sar(twin),
            b.edge_access.sar(edge),
            "edge {edge}"
        );
    }
    let simple =
        |package: &PackageFile, view: &wayfarer_package::format::Sections<'_>, mapped: bool| {
            let edge = |e: u32| if mapped { map.edge[e as usize] } else { e };
            let mut rows = (0..view.simple_from.len())
                .map(|row| {
                    (
                        edge(view.simple_from[row]),
                        edge(view.simple_to[row]),
                        profiles(package, view.simple_mask.get(row)),
                    )
                })
                .collect::<Vec<_>>();
            rows.sort();
            rows
        };
    assert_eq!(simple(&v2, &a, false), simple(&v3, &b, true));
    let complex =
        |package: &PackageFile, view: &wayfarer_package::format::Sections<'_>, mapped: bool| {
            let edge = |e: u32| if mapped { map.edge[e as usize] } else { e };
            let mut rows = (0..view.complex_to.len())
                .map(|row| {
                    let prefix = &view.complex_prefix_pool[view.complex_prefix_start[row] as usize
                        ..view.complex_prefix_start[row + 1] as usize];
                    (
                        prefix.iter().map(|e| edge(*e)).collect::<Vec<_>>(),
                        edge(view.complex_to[row]),
                        profiles(package, view.complex_mask.get(row)),
                    )
                })
                .collect::<Vec<_>>();
            rows.sort();
            rows
        };
    assert_eq!(complex(&v2, &a, false), complex(&v3, &b, true));
    let saturated = [&v2, &v3].map(|package| {
        package.profile_masks(Section::NodeReachSaturated, Section::NodeReachSaturated16)
    });
    for node in 0..b.node_count {
        let twin = map.node[node] as usize;
        assert_eq!(
            profiles(&v2, saturated[0].get(twin)),
            profiles(&v3, saturated[1].get(node)),
            "node {node}"
        );
    }
    assert_eq!(a.barrier_node.len(), b.barrier_node.len());
    let barriers =
        |package: &PackageFile, view: &wayfarer_package::format::Sections<'_>, mapped: bool| {
            let mut rows = (0..view.barrier_node.len())
                .map(|at| {
                    let node = view.barrier_node[at];
                    (
                        if mapped {
                            map.node[node as usize]
                        } else {
                            node
                        },
                        view.barrier_class[at],
                        ALL_PROFILES.map(|profile| {
                            format!(
                                "{:?}",
                                view.barrier_access
                                    .get(at)
                                    .effect(package.profile_bit(profile))
                            )
                        }),
                    )
                })
                .collect::<Vec<_>>();
            rows.sort();
            rows
        };
    assert_eq!(barriers(&v2, &a, false), barriers(&v3, &b, true));
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
    let starts_at = raw.position(Section::AttrClassStart64.id()).unwrap();
    let starts: Vec<u64> = raw.sections[starts_at]
        .bytes
        .as_chunks::<8>()
        .0
        .iter()
        .map(|bytes| u64::from_le_bytes(*bytes))
        .collect();
    let old_bytes = &raw.sections[at].bytes;
    let (mut new_starts, mut new_bytes) = (vec![0u64], Vec::new());
    for pair in starts.windows(2) {
        new_bytes.extend_from_slice(&old_bytes[pair[0] as usize..pair[1] as usize]);
        new_bytes.extend_from_slice(&[1, 9, 0]);
        new_starts.push(new_bytes.len() as u64);
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

/// Format 3 numbers nodes by CCH rank (`docs/design/format-3.md`, section
/// 8): the layout says so, the ranks are implied, and the cell tree states
/// each dissection cell's range of ranks.
#[test]
fn format_3_numbers_nodes_by_rank_and_states_the_cell_tree() {
    let (v2, v3) = monaco();
    let (v2, v3) = (open(v2).unwrap(), open(v3).unwrap());
    assert!(v3.layout().nodes_by_rank);
    assert!(!v2.layout().nodes_by_rank);
    let flags = u64::from_le_bytes(
        monaco().1[HEADER_V3_LAYOUT_FLAGS..HEADER_V3_LAYOUT_FLAGS + 8]
            .try_into()
            .unwrap(),
    );
    assert_ne!(flags & u64::from(LAYOUT_NODES_BY_RANK), 0);
    let names = v3
        .section_sizes()
        .into_iter()
        .map(|(name, _)| name)
        .collect::<Vec<_>>();
    assert!(!names.contains(&"CchRank"));
    assert!(names.contains(&"CchCellTree"));
    let view = v3.view();
    let ranks = view.cch_ranks().unwrap();
    assert!(
        ranks
            .iter()
            .enumerate()
            .all(|(node, rank)| node as u32 == *rank)
    );
    // The tree: post-order, every cell one range, the root ending at the
    // last rank, and the cells' ranges nested in their parents'.
    let tree = v3.cell_tree();
    assert!(tree.len() > 1, "{}", tree.len());
    let root = tree.last().unwrap();
    assert_eq!(root.parent, NO_PARENT);
    assert_eq!(root.end_rank as usize, view.node_count);
    for entry in &tree[..tree.len() - 1] {
        let parent = &tree[entry.parent as usize];
        assert!(parent.chains_end <= entry.first_rank && entry.end_rank <= parent.separator_start);
    }
    // Format 2 keeps its stored ranks.
    assert!(v2.cell_tree().is_empty());
    assert_eq!(v2.view().cch_ranks().unwrap().len(), v2.view().node_count);
}

fn with_cell_tree(edit: impl FnOnce(&mut Vec<CellTreeEntry>)) -> RawPackage {
    let mut raw = format_3();
    let at = raw.position(Section::CchCellTree.id()).unwrap();
    let mut tree = raw.sections[at]
        .bytes
        .as_chunks::<CELL_TREE_ENTRY_BYTES>()
        .0
        .iter()
        .map(CellTreeEntry::from_bytes)
        .collect::<Vec<_>>();
    edit(&mut tree);
    raw.sections[at].bytes = tree.into_iter().flat_map(CellTreeEntry::to_bytes).collect();
    raw
}

#[test]
fn the_cell_tree_is_checked() {
    let invalid = |raw: &RawPackage| {
        let error = open_error(raw);
        assert!(error.contains("cell tree is invalid"), "{error}");
    };
    // The unchanged tree opens.
    open(&with_cell_tree(|_| {}).encode()).unwrap();
    // A root that stops short of the last rank.
    invalid(&with_cell_tree(|tree| {
        tree.last_mut().unwrap().end_rank -= 1
    }));
    // A root with a parent.
    invalid(&with_cell_tree(|tree| tree.last_mut().unwrap().parent = 0));
    // A child that names itself, or a cell after it, or none, as parent.
    invalid(&with_cell_tree(|tree| tree[0].parent = 0));
    invalid(&with_cell_tree(|tree| tree[0].parent = NO_PARENT));
    invalid(&with_cell_tree(|tree| {
        let past = tree.len() as u32;
        tree[0].parent = past;
    }));
    // Ranges out of order, or leaving a gap between siblings.
    invalid(&with_cell_tree(|tree| {
        let entry = &mut tree[0];
        entry.chains_end = entry.separator_start + 1;
    }));
    invalid(&with_cell_tree(|tree| tree[0].end_rank += 1));
    invalid(&with_cell_tree(|tree| tree[0].first_rank += 1));
    // Children out of post-order.
    invalid(&with_cell_tree(|tree| tree.swap(0, 1)));
    // A ragged section is refused by its element size.
    let mut raw = format_3();
    let at = raw.position(Section::CchCellTree.id()).unwrap();
    raw.sections[at].bytes.pop();
    assert!(open(&raw.encode()).is_err());
}

#[test]
fn the_rank_layout_and_the_cell_tree_come_together() {
    // The layout without its tree.
    let mut raw = format_3();
    let at = raw.position(Section::CchCellTree.id()).unwrap();
    raw.sections.remove(at);
    assert!(open_error(&raw).contains("missing"), "{}", open_error(&raw));
    // The tree without the layout.
    let mut raw = format_3();
    raw.header[HEADER_V3_LAYOUT_FLAGS] &= !(LAYOUT_NODES_BY_RANK as u8);
    assert!(
        open_error(&raw).contains("against its rule"),
        "{}",
        open_error(&raw)
    );
    // A stored rank vector is retired in format 3.
    let mut raw = format_3();
    let at = raw
        .sections
        .iter()
        .position(|section| section.id > Section::CchRank.id())
        .unwrap();
    let node_count = open(&monaco().1).unwrap().view().node_count;
    raw.sections.insert(
        at,
        RawSection {
            id: Section::CchRank.id(),
            param: 0,
            flags: SECTION_IDENTITY,
            element_bytes: 4,
            bytes: (0..node_count as u32).flat_map(u32::to_le_bytes).collect(),
        },
    );
    assert!(open_error(&raw).contains("retired"), "{}", open_error(&raw));
}

/// The u64 words of section `id`.
fn words(raw: &RawPackage, section: Section) -> Vec<u64> {
    let at = raw.position(section.id()).unwrap();
    raw.sections[at]
        .bytes
        .as_chunks::<8>()
        .0
        .iter()
        .map(|bytes| u64::from_le_bytes(*bytes))
        .collect()
}

fn with_words(section: Section, edit: impl FnOnce(&mut Vec<u64>)) -> RawPackage {
    let mut raw = format_3();
    let mut values = words(&raw, section);
    edit(&mut values);
    let at = raw.position(section.id()).unwrap();
    raw.sections[at].bytes = values.into_iter().flat_map(u64::to_le_bytes).collect();
    raw
}

fn with_u32s(section: Section, edit: impl FnOnce(&mut Vec<u32>)) -> RawPackage {
    let mut raw = format_3();
    let at = raw.position(section.id()).unwrap();
    let mut values = raw.sections[at]
        .bytes
        .as_chunks::<4>()
        .0
        .iter()
        .map(|bytes| u32::from_le_bytes(*bytes))
        .collect::<Vec<_>>();
    edit(&mut values);
    raw.sections[at].bytes = values.into_iter().flat_map(u32::to_le_bytes).collect();
    raw
}

#[test]
fn format_3_widens_offsets_and_cost_tuples_and_puts_the_grid_on_the_lattice() {
    let (v2, v3) = (open(&monaco().0).unwrap(), open(&monaco().1).unwrap());
    let names = |file: &PackageFile| {
        file.section_sizes()
            .into_iter()
            .map(|(name, _)| name)
            .collect::<Vec<_>>()
    };
    let (names_2, names_3) = (names(&v2), names(&v3));
    for (narrow, wide) in [
        ("ShapeStart", "ShapeStart64"),
        ("AttrClassStart", "AttrClassStart64"),
        ("StringStart", "StringStart64"),
        ("SharedInfoCostTuple", "SharedInfoCostTuple32"),
        ("RequestGrid", "GridBlocks"),
        ("GridCellStart", "GridBlockCellStart"),
    ] {
        assert!(
            names_2.contains(&narrow) && !names_2.contains(&wide),
            "{narrow}"
        );
        assert!(
            names_3.contains(&wide) && !names_3.contains(&narrow),
            "{wide}"
        );
    }
    assert_eq!(v3.params().request_grid_spacing, Some([1_440, 1_440]));

    // Shapes and cost tuples read the same through the correspondence.
    let map = monaco_correspondence();
    let (a, b) = (v2.view(), v3.view());
    let (mut shape_2, mut shape_3) = (Vec::new(), Vec::new());
    for edge in 0..b.edge_count {
        shape_2.clear();
        shape_3.clear();
        a.edge_shape(map.edge[edge], &mut shape_2);
        b.edge_shape(edge as u32, &mut shape_3);
        assert_eq!(shape_2, shape_3, "edge {edge}");
    }
    let mut tuple_in_2 = std::collections::HashMap::new();
    for shared in 0..v3.counts().shared_info_count {
        let tuple = b.cost_tuple(shared);
        let twin = a.cost_tuple(map.shared[shared] as usize);
        assert_eq!(*tuple_in_2.entry(tuple).or_insert(twin), twin, "{shared}");
        assert_eq!(b.si_cost_flags[tuple], a.si_cost_flags[twin], "{shared}");
        assert_eq!(
            b.si_cost_classes[tuple], a.si_cost_classes[twin],
            "{shared}"
        );
    }

    // Monaco fits one lattice block; every edge with a shape is an entry.
    let blocks = words(&format_3(), Section::GridBlocks);
    assert!(!blocks.is_empty() && blocks.len() <= 8, "{blocks:?}");
    assert_eq!(blocks[1], 0);
}

#[test]
fn wide_offsets_are_checked() {
    let refused = |raw: RawPackage| assert!(open(&raw.encode()).is_err());
    open(&with_words(Section::ShapeStart64, |_| {}).encode()).unwrap();
    for section in [
        Section::ShapeStart64,
        Section::AttrClassStart64,
        Section::StringStart64,
    ] {
        // Past the end, past 4 GiB, out of order, or not from 0.
        refused(with_words(section, |starts| {
            *starts.last_mut().unwrap() += 1
        }));
        refused(with_words(section, |starts| {
            *starts.last_mut().unwrap() = 1 << 32
        }));
        refused(with_words(section, |starts| starts[1] = u64::MAX));
        refused(with_words(section, |starts| starts[0] = 1));
    }
    assert!(
        open_error(&with_words(Section::ShapeStart64, |starts| {
            *starts.last_mut().unwrap() += 1
        }))
        .contains("shape offsets")
    );
    // A cost tuple row past the tuples.
    let error = open_error(&with_u32s(Section::SharedInfoCostTuple32, |rows| {
        rows[0] = 1 << 20
    }));
    assert!(error.contains("costing tuples"), "{error}");
}

#[test]
fn the_lattice_grid_is_checked() {
    let inconsistent = |raw: RawPackage| {
        let error = open_error(&raw);
        assert!(error.contains("request sections"), "{error}");
    };
    let blocks = words(&format_3(), Section::GridBlocks);
    let entries = with_u32s(Section::GridEntries, |_| {});
    let entry_count = entries.sections[entries.position(Section::GridEntries.id()).unwrap()]
        .bytes
        .len() as u64
        / 4;
    // An empty block after the last, with its starts, opens.
    let with_block = |key: u64, base: u64| {
        let mut raw = with_words(Section::GridBlocks, |blocks| blocks.extend([key, base]));
        let at = raw.position(Section::GridBlockCellStart.id()).unwrap();
        raw.sections[at].bytes.extend(vec![0u8; 4097 * 4]);
        raw
    };
    let last_key = blocks[blocks.len() - 2];
    open(&with_block(last_key + 1, entry_count).encode()).unwrap();
    // The same block twice, or out of key order.
    inconsistent(with_block(last_key, entry_count));
    inconsistent(with_block(blocks[0] - 1, entry_count));
    // A base that leaves a gap or overlaps.
    inconsistent(with_block(last_key + 1, entry_count + 1));
    inconsistent(with_words(Section::GridBlocks, |blocks| blocks[1] = 1));
    // Starts that don't end at the entry count, fall back, or don't come
    // 4,097 to a block.
    inconsistent(with_u32s(Section::GridBlockCellStart, |starts| {
        *starts.last_mut().unwrap() += 1
    }));
    inconsistent(with_u32s(Section::GridBlockCellStart, |starts| {
        starts[1] = u32::MAX
    }));
    inconsistent(with_u32s(Section::GridBlockCellStart, |starts| {
        starts[0] = 1
    }));
    inconsistent(with_u32s(Section::GridBlockCellStart, |starts| {
        starts.pop();
    }));
    // A half block entry is refused by its element size.
    let mut raw = format_3();
    let at = raw.position(Section::GridBlocks.id()).unwrap();
    let len = raw.sections[at].bytes.len();
    raw.sections[at].bytes.truncate(len - 8);
    assert!(open(&raw.encode()).is_err());

    // The spacing must be stated and divide a degree.
    let set_spacing = |value: Option<u64>| {
        with_params(|entries| {
            entries.retain(|entry| entry.0 != param_key::REQUEST_GRID_SPACING);
            if let Some(value) = value {
                let at = entries.partition_point(|entry| entry.0 < param_key::REQUEST_GRID_SPACING);
                entries.insert(
                    at,
                    (
                        param_key::REQUEST_GRID_SPACING,
                        PARAM_MUST_UNDERSTAND,
                        value,
                    ),
                );
            }
        })
    };
    open(&set_spacing(Some(1_440 | 1_440 << 32)).encode()).unwrap();
    assert!(open_error(&set_spacing(None)).contains("missing"));
    for bad in [0, 1_440, 1_440 << 32, 7 | 1_440 << 32] {
        let error = open_error(&set_spacing(Some(bad)));
        assert!(error.contains("spacing"), "{bad}: {error}");
    }
}

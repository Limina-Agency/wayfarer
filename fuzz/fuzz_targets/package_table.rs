//! A format-3 package's header and section table, rebuilt from a valid one
//! by a list of edits (QA-5): entries swapped, repeated, dropped or retagged
//! (id, param, flags, element size), sections resized, unknown sections
//! added (REQUIRED, IDENTITY, experimental or neither), the table before or
//! after the sections, header bytes changed, and raw bytes patched after
//! layout (offsets that overlap or leave gaps). Each edit is 8 bytes: kind,
//! two small operands, a byte and a u32.
//!
//! Whatever opens must route without panicking, and must keep its identity
//! hash when its unknown non-IDENTITY sections are stripped and its table
//! moved to the other end of the file.

#![no_main]

use std::sync::{Arc, OnceLock};

use libfuzzer_sys::fuzz_target;
use wayfarer::{GeoPoint, MobilityProfile, RouteRequest, RoutingEngine, RoutingPackage};
use wayfarer_compiler::package_build::{
    PackageBuildOptions, package_bytes, renumber_in_package_order,
};
use wayfarer_package::format::{PackageFile, PackageFormat, SECTION_IDENTITY, Section};
use wayfarer_package::raw::{RawPackage, RawSection, resign};
use wayfarer_road_graph::{
    DirectionalAccess, RoadGraphBuilder, RoadType, RoutableWay, enhance_road_graph,
};

/// A small format-3 package: a square of streets with a diagonal.
fn seed() -> &'static RawPackage {
    static SEED: OnceLock<RawPackage> = OnceLock::new();
    SEED.get_or_init(|| {
        let mut builder = RoadGraphBuilder::new("package-table-fuzz");
        let ways = [
            vec![(52.0, 21.0), (52.0, 21.001), (52.0, 21.002)],
            vec![(52.0, 21.002), (52.001, 21.002)],
            vec![(52.001, 21.002), (52.001, 21.0)],
            vec![(52.001, 21.0), (52.0, 21.0)],
            vec![(52.0, 21.001), (52.001, 21.002)],
        ];
        for (index, points) in ways.into_iter().enumerate() {
            builder.push_way(RoutableWay {
                osm_way_id: index as i64 + 1,
                points,
                road_type: RoadType::Residential,
                directional_access: if index == 4 {
                    DirectionalAccess::Forward
                } else {
                    DirectionalAccess::Both
                },
                highway: Some("residential".into()),
                ..RoutableWay::default()
            });
        }
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);
        // Format 3 implies ids: the graph needs a compile's numbering.
        renumber_in_package_order(&mut graph).expect("the seed renumbers");
        let options = PackageBuildOptions {
            package_format: PackageFormat::V3,
            ..PackageBuildOptions::default()
        };
        let (bytes, _) = package_bytes(&graph, &options).expect("the seed compiles");
        RawPackage::parse(&bytes).expect("the seed parses")
    })
}

fuzz_target!(|data: &[u8]| {
    let mut raw = seed().clone();
    let mut patches = Vec::new();
    for edit in data.chunks_exact(8) {
        let (kind, a, b, byte) = (edit[0] % 12, edit[1] as usize, edit[2] as usize, edit[3]);
        let value = u32::from_le_bytes(edit[4..8].try_into().unwrap());
        let count = raw.sections.len();
        match kind {
            0 if count > 0 => raw.sections.swap(a % count, b % count),
            1 if count > 0 => {
                let copy = raw.sections[a % count].clone();
                raw.sections.insert(b % (count + 1), copy);
            }
            2 if count > 0 => {
                raw.sections.remove(a % count);
            }
            3 if count > 0 => raw.sections[a % count].id = value,
            4 if count > 0 => raw.sections[a % count].param = value,
            5 if count > 0 => raw.sections[a % count].flags = value % 8,
            6 if count > 0 => raw.sections[a % count].element_bytes = value % 33,
            7 => {
                let element_bytes = 1 + (a % 8) as u32;
                raw.sections.insert(
                    b % (count + 1),
                    RawSection {
                        id: value,
                        param: u32::from(byte),
                        flags: u32::from(byte) % 4,
                        element_bytes,
                        bytes: vec![byte; (b % 5) * element_bytes as usize],
                    },
                );
            }
            8 => raw.table_first = a % 2 == 1,
            9 => {
                let len = raw.header.len();
                raw.header[a % len] = byte;
            }
            10 if count > 0 => {
                let section = &mut raw.sections[a % count];
                let len = section.bytes.len();
                section.bytes.resize(
                    if b % 2 == 0 {
                        len + b % 17
                    } else {
                        len.saturating_sub(b % 17)
                    },
                    byte,
                );
            }
            11 => patches.push((value as usize, byte)),
            _ => {}
        }
    }
    let mut bytes = raw.encode();
    for (at, byte) in &patches {
        let len = bytes.len();
        bytes[at % len] = *byte;
    }
    resign(&mut bytes);
    let Ok(package) = PackageFile::from_bytes(&bytes) else {
        return;
    };
    let reopened = PackageFile::from_bytes(&bytes).expect("a package that opened reopens");
    assert_eq!(package.dataset_hash(), reopened.dataset_hash());
    assert_eq!(package.content_hash(), reopened.content_hash());
    if patches.is_empty() {
        // The identity hash covers the header and the IDENTITY entries only.
        let mut stripped = raw.clone();
        stripped.sections.retain(|section| {
            Section::from_id(section.id).is_some() || section.flags & SECTION_IDENTITY != 0
        });
        stripped.table_first = !stripped.table_first;
        let stripped = PackageFile::from_bytes(&stripped.encode())
            .expect("a package without its unknown optional sections opens");
        assert_eq!(stripped.dataset_hash(), package.dataset_hash());
    }
    let Ok(package) = RoutingPackage::from_bytes(&bytes) else {
        return;
    };
    let engine = RoutingEngine::from_package(Arc::new(package));
    for profile in [MobilityProfile::Wheeled, MobilityProfile::OnFoot] {
        let _ = engine.route(RouteRequest::new(
            GeoPoint::new(52.0, 21.0002),
            GeoPoint::new(52.001, 21.0015),
            profile,
        ));
    }
});

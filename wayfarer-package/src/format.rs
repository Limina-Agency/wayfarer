//! The routing package format: header, section table, sections, the
//! section view queries read, the shape codec and the writer.

use std::fs::File;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

use memmap2::Mmap;
use serde::{Deserialize, Serialize};

use crate::{GeoPoint, MobilityProfile, RoadType, TrafficControl, math};

#[cfg(not(target_endian = "little"))]
compile_error!("the routing package format assumes a little-endian target");

pub const MAGIC: &[u8; 8] = b"MRRPKG\0\x01";
/// The format the compiler writes by default. Bump whenever the layout or
/// the meaning of any section changes.
///
/// Format 2 added [`Section::CchRank`], the node order of CCH potentials, at
/// the end of the section table, and stores no landmarks: format 1's
/// landmark sections are [`Section::Reserved1`] to [`Section::Reserved12`],
/// as a package without landmarks wrote them.
///
/// Format 3 (`docs/design/format-3.md`) is being built: a compile writes it
/// only when asked ([`PackageFormat::V3`]), and those packages carry a draft
/// revision ([`FORMAT_3_DRAFT_REVISION`]) that only a build of the same
/// revision reads.
pub const ROUTING_PACKAGE_FORMAT_VERSION: u32 = 2;
/// The oldest format this build still opens. A layout change bumps
/// [`ROUTING_PACKAGE_FORMAT_VERSION`] and keeps reading the previous format
/// for at least one release once packages are in use; raising this drops a
/// format, which the release notes must say. Format 1 (landmark distances,
/// no CCH order) is no longer read.
pub const MIN_READ_FORMAT_VERSION: u32 = 2;
/// The newest format this build opens: format 3, of its own draft revision
/// only, while format 3 is being built.
pub const MAX_READ_FORMAT_VERSION: u32 = 3;
/// The draft revision of format 3 this build writes and reads
/// ([`param_key::DRAFT_REVISION`]). Every change to format 3's bytes before
/// its release raises it; the release removes the key.
pub const FORMAT_3_DRAFT_REVISION: u64 = 1;
pub const HEADER_FIXED_BYTES: usize = 256;
/// A format-2 section table entry: offset u64, length u64, blake3.
pub const TABLE_ENTRY_BYTES: usize = 48;
/// A format-3 section table entry: id u32, param u32, flags u32, element
/// size u32, offset u64, length u64, blake3.
pub const TABLE_ENTRY_BYTES_V3: usize = 64;
/// The most entries a format-3 section table may have, so a hostile table
/// can't make open allocate much.
pub const MAX_TABLE_ENTRIES: usize = 4096;
/// Format-3 entry flag: an engine that doesn't know the section must refuse
/// the package (`TooNew`).
pub const SECTION_REQUIRED: u32 = 1 << 0;
/// Format-3 entry flag: the section is covered by the identity hash.
pub const SECTION_IDENTITY: u32 = 1 << 1;
/// Section ids from here up are experimental: never REQUIRED.
pub const EXPERIMENTAL_SECTION_ID: u32 = 0x8000_0000;
/// The domain tag the identity hash starts with.
pub const IDENTITY_HASH_TAG: &[u8] = b"wayfarer.package.identity.3";

/// The package format a writer produces.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum PackageFormat {
    /// Format 2: a positional section table after the header (the default).
    #[default]
    V2,
    /// Format 3, draft [`FORMAT_3_DRAFT_REVISION`]: a tagged section table
    /// after the sections, implied ids, [`Section::PackageParams`].
    V3,
}

impl PackageFormat {
    /// The version number in the header.
    pub fn version(self) -> u32 {
        match self {
            Self::V2 => 2,
            Self::V3 => 3,
        }
    }

    /// The format of a version number, if this build writes it.
    pub fn from_version(version: u32) -> Option<Self> {
        match version {
            2 => Some(Self::V2),
            3 => Some(Self::V3),
            _ => None,
        }
    }
}
pub const SECTION_ALIGN: usize = 64;
pub const PROFILE_SLOTS: usize = 6;
pub const NO_COLUMN: u8 = u8::MAX;

/// [`Section::EdgeTwin`] of an edge without a twin.
pub const NO_TWIN: u32 = u32::MAX;

/// The heading of the reverse direction of travel: `heading` plus 180
/// degrees, modulo 360 ([`NO_HEADING`] stays). A twin's end heading is its
/// edge's start heading reversed, and the reverse.
#[inline]
pub fn reverse_heading(heading: u16) -> u16 {
    if heading == NO_HEADING {
        NO_HEADING
    } else {
        ((u32::from(heading) + 180) % 360) as u16
    }
}
/// The [`Section::SharedInfoWay`] value of a way id stored in
/// [`Section::SharedInfoWayWide`]: negative (synthetic) ids and ids from
/// `u32::MAX` up.
pub const WAY_ID_WIDE: u32 = u32::MAX;
/// Distinct costing tuples a package can index ([`Section::SharedInfoCostTuple`]).
pub const MAX_COST_TUPLES: usize = u16::MAX as usize + 1;
/// [`Section::ShapeStart`] stores the byte offset of shapes `0, k, 2k, …`
/// (k = this) and the end of `ShapeBytes`: the shapes between are found by
/// skipping their encodings ([`skip_shape`]).
pub const SHAPE_START_STRIDE: usize = 16;
pub const NO_HEADING: u16 = u16::MAX;
pub const NO_TOKEN: u32 = u32::MAX;

pub const EDGE_FLAG_LINK: u8 = 1 << 0;
pub const EDGE_FLAG_SIMPLE_RESTRICTION_FROM: u8 = 1 << 1;
pub const EDGE_FLAG_COMPLEX_RESTRICTION_TO: u8 = 1 << 2;
/// The edge is the first edge of some complex restriction's forbidden sequence.
pub const EDGE_FLAG_COMPLEX_RESTRICTION_FIRST: u8 = 1 << 3;
/// The edge runs against its shared-info shape (the road graph's `Backward`
/// traversal direction), so its polyline is the shape reversed.
pub const EDGE_FLAG_SHAPE_REVERSED: u8 = 1 << 4;
/// The edge's target is a barrier node ([`Section::BarrierNode`]), so a
/// transition through it may pay or be closed by the barrier.
pub const EDGE_FLAG_BARRIER_AT_TARGET: u8 = 1 << 5;

/// Section identifiers; the numeric value is the slot in the section table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(usize)]
pub enum Section {
    /// JSON [`RoutingPackageMetadata`].
    Metadata,
    NodeFirstOut,
    /// Empty when E7 positions derive from `NodeXyzFixed` (see
    /// [`PackageLayout`]).
    NodeLatE7,
    NodeLngE7,
    NodeLatLng,
    EdgeTarget,
    EdgeId,
    EdgeLength,
    EdgeSharedInfo,
    EdgeHeadingStart,
    EdgeFlags,
    EdgeRoadRank,
    EdgeCost,
    EdgeDuration,
    SharedInfoWay,
    /// One name token per attribute class ([`Section::SharedAttrClass`]): a
    /// shared info's name and ref are part of its class's key, so the class
    /// determines them. Likewise `SharedInfoRef`, and the route refs
    /// (`SharedInfoRouteRefStart` offsets per class into
    /// `SharedInfoRouteRefPool`).
    SharedInfoName,
    SharedInfoRef,
    SharedInfoRouteRefStart,
    SharedInfoRouteRefPool,
    SimpleFrom,
    SimpleTo,
    SimpleVia,
    SimpleMask,
    ComplexTo,
    ComplexMask,
    ComplexPrefixStart,
    ComplexPrefixPool,
    EdgeProfileAccess,
    /// Reserved: held format 1's landmark distances. In format 2,
    /// [`RESERVED_ZERO_SECTIONS`] hold `column_count * ceil(node_count / 64)`
    /// zeros (the row bits and ranks of a package without landmarks) and the
    /// other reserved sections are empty. A later format drops the slots.
    Reserved1,
    /// Reserved, empty (see [`Section::Reserved1`]).
    Reserved2,
    /// Reserved, empty (see [`Section::Reserved1`]).
    Reserved3,
    /// Reserved, empty (see [`Section::Reserved1`]).
    Reserved4,
    ComplexFirst,
    ComplexByFirst,
    ShapeStart,
    ShapeBytes,
    ExactShapeShared,
    ExactShapeStart,
    ExactShapePoints,
    SharedInfoCostFlags,
    SharedInfoCostClasses,
    SharedInfoSpeeds,
    EdgeInferredSpeed,
    ConditionalEdges,
    ConditionalShared,
    ConditionalStart,
    ConditionalPool,
    // Request-path sections (see `request.rs`).
    RequestMeta,
    RequestGrid,
    GridCellStart,
    GridEntries,
    NodeIdDelta,
    NodeIdWide,
    ComponentExceptions,
    SharedInfoIdGaps,
    SharedAttrClass,
    AttrClassStart,
    AttrClassBytes,
    StringStart,
    StringBytes,
    OpposingExceptionEdge,
    OpposingExceptionValue,
    LegOpposingExceptionEdge,
    LegOpposingExceptionValue,
    NodeRoundaboutExits,
    ComponentSizes,
    NodeReachSaturated,
    ModeMaskExceptions,
    TierExceptions,
    /// Per edge, [`junction::encode`] of what a vehicle meets at the edge's
    /// end (traffic controls and priority).
    EdgeJunction,
    /// Reserved, zeros (see [`Section::Reserved1`]).
    Reserved5,
    /// Reserved, zeros (see [`Section::Reserved1`]).
    Reserved6,
    /// The sorted nodes whose exact position `NodeLatLng` holds (the nodes
    /// that are not their E7 value).
    NodeLatLngNodes,
    /// Per shared info, its row of the costing tuple table
    /// (`SharedInfoCostFlags`, `SharedInfoCostClasses`, `SharedInfoSpeeds`).
    SharedInfoCostTuple,
    /// The sorted shared infos whose way id is [`WAY_ID_WIDE`] in
    /// `SharedInfoWay`.
    SharedInfoWayWideShared,
    /// Their way ids.
    SharedInfoWayWide,
    /// Per shared info, its [`crate::sar::LegalReason`] byte.
    SharedInfoLegalReason,
    /// The sorted nodes with a barrier that affects some profile or vehicles
    /// under SAR (or of unknown type).
    BarrierNode,
    /// Their [`crate::sar::SarBarrierClass`] codes, 0 for a barrier with no
    /// effect on vehicles.
    BarrierClass,
    /// Their [`crate::barrier::BarrierAccess`] bits.
    BarrierAccess,
    /// Reserved, zeros (see [`Section::Reserved1`]).
    Reserved7,
    /// Reserved, zeros (see [`Section::Reserved1`]).
    Reserved8,
    /// Reserved, empty (see [`Section::Reserved1`]).
    Reserved9,
    /// Reserved, zeros (see [`Section::Reserved1`]).
    Reserved10,
    /// Reserved, zeros (see [`Section::Reserved1`]).
    Reserved11,
    /// Reserved, empty (see [`Section::Reserved1`]).
    Reserved12,
    /// Per node, its unit vector in fixed point ([`fixed_xyz`]).
    NodeXyzFixed,
    /// `[column][node / 64]` bits, set at the stored column's marked chain
    /// nodes (see [`PieceKinds`]).
    PieceChain,
    /// `[column][node / 64]` bits, set at the stored column's marked tree
    /// nodes.
    PieceTree,
    /// Per edge, its twin (the reverse edge of the same shared info), or
    /// [`NO_TWIN`]. A node's in-edges are derived from it: the twins of its
    /// out-edges, plus the edges without a twin ([`Section::LoneInEdge`]);
    /// so are edges' sources and end headings. Read through
    /// [`Sections::in_edges`], [`Sections::edge_source`] and
    /// [`Sections::heading_end`].
    EdgeTwin,
    /// `[node / 64]` bits, set at the targets of the edges without a twin.
    LoneInPresent,
    /// The edges without a twin, sorted by (target, edge).
    LoneInEdge,
    /// The end headings of the `LoneInEdge` edges, in its order.
    LoneHeadingEnd,
    /// Per node, its rank in the CCH elimination order
    /// ([`crate::cch_order`]); empty when the package was written without
    /// one (searches then have no potentials).
    CchRank,
    /// Format 3 (id 129): the scalars a query reads, as sorted 16-byte
    /// entries `key u32, flags u32 (bit 0: must understand), value u64`
    /// ([`PackageParams`]). Format 2 keeps them in the JSON metadata and
    /// `RequestMeta`.
    PackageParams,
}

/// Every section this build knows, of any format; the internal index of
/// [`Section`] (not its format-3 id, see [`Section::id`]).
pub const SECTION_COUNT: usize = Section::PackageParams as usize + 1;
/// The sections a format-2 table lists, in slot order: `Metadata` to
/// `CchRank`.
pub const FORMAT2_SECTION_COUNT: usize = Section::CchRank as usize + 1;

/// Every [`Section`], by internal index.
pub const SECTIONS: [Section; SECTION_COUNT] = [
    Section::Metadata,
    Section::NodeFirstOut,
    Section::NodeLatE7,
    Section::NodeLngE7,
    Section::NodeLatLng,
    Section::EdgeTarget,
    Section::EdgeId,
    Section::EdgeLength,
    Section::EdgeSharedInfo,
    Section::EdgeHeadingStart,
    Section::EdgeFlags,
    Section::EdgeRoadRank,
    Section::EdgeCost,
    Section::EdgeDuration,
    Section::SharedInfoWay,
    Section::SharedInfoName,
    Section::SharedInfoRef,
    Section::SharedInfoRouteRefStart,
    Section::SharedInfoRouteRefPool,
    Section::SimpleFrom,
    Section::SimpleTo,
    Section::SimpleVia,
    Section::SimpleMask,
    Section::ComplexTo,
    Section::ComplexMask,
    Section::ComplexPrefixStart,
    Section::ComplexPrefixPool,
    Section::EdgeProfileAccess,
    Section::Reserved1,
    Section::Reserved2,
    Section::Reserved3,
    Section::Reserved4,
    Section::ComplexFirst,
    Section::ComplexByFirst,
    Section::ShapeStart,
    Section::ShapeBytes,
    Section::ExactShapeShared,
    Section::ExactShapeStart,
    Section::ExactShapePoints,
    Section::SharedInfoCostFlags,
    Section::SharedInfoCostClasses,
    Section::SharedInfoSpeeds,
    Section::EdgeInferredSpeed,
    Section::ConditionalEdges,
    Section::ConditionalShared,
    Section::ConditionalStart,
    Section::ConditionalPool,
    Section::RequestMeta,
    Section::RequestGrid,
    Section::GridCellStart,
    Section::GridEntries,
    Section::NodeIdDelta,
    Section::NodeIdWide,
    Section::ComponentExceptions,
    Section::SharedInfoIdGaps,
    Section::SharedAttrClass,
    Section::AttrClassStart,
    Section::AttrClassBytes,
    Section::StringStart,
    Section::StringBytes,
    Section::OpposingExceptionEdge,
    Section::OpposingExceptionValue,
    Section::LegOpposingExceptionEdge,
    Section::LegOpposingExceptionValue,
    Section::NodeRoundaboutExits,
    Section::ComponentSizes,
    Section::NodeReachSaturated,
    Section::ModeMaskExceptions,
    Section::TierExceptions,
    Section::EdgeJunction,
    Section::Reserved5,
    Section::Reserved6,
    Section::NodeLatLngNodes,
    Section::SharedInfoCostTuple,
    Section::SharedInfoWayWideShared,
    Section::SharedInfoWayWide,
    Section::SharedInfoLegalReason,
    Section::BarrierNode,
    Section::BarrierClass,
    Section::BarrierAccess,
    Section::Reserved7,
    Section::Reserved8,
    Section::Reserved9,
    Section::Reserved10,
    Section::Reserved11,
    Section::Reserved12,
    Section::NodeXyzFixed,
    Section::PieceChain,
    Section::PieceTree,
    Section::EdgeTwin,
    Section::LoneInPresent,
    Section::LoneInEdge,
    Section::LoneHeadingEnd,
    Section::CchRank,
    Section::PackageParams,
];

/// Whether a section is in format-3 packages.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Presence {
    /// In every format-3 package.
    Mandatory,
    /// In a format-3 package exactly when its header says so
    /// (`NodeLatE7`, `NodeLngE7`: when positions are not derived).
    Conditional,
    /// Not in format 3; its id is never reused.
    Retired,
}

impl Section {
    /// The section with internal index `index`.
    pub fn from_index(index: usize) -> Option<Self> {
        SECTIONS.get(index).copied()
    }

    /// The format-3 section id: format 2's slot number for the sections
    /// that keep their layout, ids from 128 up for the bump's new layouts
    /// (`docs/design/format-3.md`, section 6).
    pub const fn id(self) -> u32 {
        match self {
            Self::PackageParams => 129,
            other => other as u32,
        }
    }

    /// The known section with format-3 id `id`, retired or not.
    pub fn from_id(id: u32) -> Option<Self> {
        match id {
            129 => Some(Self::PackageParams),
            id if (id as usize) < FORMAT2_SECTION_COUNT => Self::from_index(id as usize),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        SECTION_NAMES[self as usize]
    }

    /// The element size in bytes.
    pub fn element_bytes(self) -> usize {
        SECTION_ELEMENT_BYTES[self as usize]
    }

    /// Whether a format-2 table lists the section.
    pub fn in_format_2(self) -> bool {
        (self as usize) < FORMAT2_SECTION_COUNT
    }

    /// Whether, and when, format-3 packages hold the section.
    pub fn format_3_presence(self) -> Presence {
        match self {
            Self::EdgeId
            | Self::Reserved1
            | Self::Reserved2
            | Self::Reserved3
            | Self::Reserved4
            | Self::RequestMeta
            | Self::NodeIdDelta
            | Self::NodeIdWide
            | Self::Reserved5
            | Self::Reserved6
            | Self::Reserved7
            | Self::Reserved8
            | Self::Reserved9
            | Self::Reserved10
            | Self::Reserved11
            | Self::Reserved12 => Presence::Retired,
            Self::NodeLatE7 | Self::NodeLngE7 => Presence::Conditional,
            _ => Presence::Mandatory,
        }
    }

    /// The flags a format-3 entry of the section carries: every section a
    /// format-3 package holds is REQUIRED; all but the JSON metadata (which
    /// no query reads) are IDENTITY.
    pub fn format_3_flags(self) -> u32 {
        match self {
            Self::Metadata => SECTION_REQUIRED,
            _ => SECTION_REQUIRED | SECTION_IDENTITY,
        }
    }

    /// Whether a package of `format` with `layout` holds the section.
    pub fn is_written(self, format: PackageFormat, layout: PackageLayout) -> bool {
        match format {
            PackageFormat::V2 => self.in_format_2(),
            PackageFormat::V3 => match self.format_3_presence() {
                Presence::Mandatory => true,
                Presence::Conditional => !layout.node_e7_derived,
                Presence::Retired => false,
            },
        }
    }
}

/// The reserved sections format 2 fills with `column_count *
/// ceil(node_count / 64)` zeros (see [`Section::Reserved1`]).
pub const RESERVED_ZERO_SECTIONS: [Section; 6] = [
    Section::Reserved5,
    Section::Reserved6,
    Section::Reserved7,
    Section::Reserved8,
    Section::Reserved10,
    Section::Reserved11,
];

/// The [`Section::EdgeJunction`] byte: bits 0-2 the [`TrafficControl`] a
/// vehicle meets on the edge before leaving it at its end, bit 3 set when the
/// edge ends at a junction where it yields to a road of a higher class or
/// enters a roundabout (a minor approach). Bits 4-7 are zero.
pub mod junction {
    use crate::TrafficControl;

    pub const CONTROL_MASK: u8 = 0b111;
    pub const MINOR_APPROACH: u8 = 1 << 3;

    pub fn encode(control: TrafficControl, minor_approach: bool) -> u8 {
        control as u8 | if minor_approach { MINOR_APPROACH } else { 0 }
    }

    pub fn control(byte: u8) -> TrafficControl {
        TrafficControl::from_code(byte & CONTROL_MASK)
    }

    pub fn minor_approach(byte: u8) -> bool {
        byte & MINOR_APPROACH != 0
    }
}

pub const SECTION_ELEMENT_BYTES: [usize; SECTION_COUNT] = [
    1,  // Metadata (JSON)
    4,  // NodeFirstOut
    4,  // NodeLatE7 (empty when derived from NodeXyzFixed, see `PackageLayout`)
    4,  // NodeLngE7 (likewise)
    8,  // NodeLatLng: (lat, lng) f64 pairs of the NodeLatLngNodes
    4,  // EdgeTarget
    4,  // EdgeId: edge id - base (the header's edge id base)
    4,  // EdgeLength
    4,  // EdgeSharedInfo
    2,  // EdgeHeadingStart
    1,  // EdgeFlags
    1,  // EdgeRoadRank
    4,  // EdgeCost (f32, column-major)
    4,  // EdgeDuration (f32, column-major)
    4,  // SharedInfoWay: way id, or WAY_ID_WIDE
    4,  // SharedInfoName: name token per attribute class
    4,  // SharedInfoRef: ref token, likewise
    4,  // SharedInfoRouteRefStart: offsets into SharedInfoRouteRefPool per attribute class + 1
    4,  // SharedInfoRouteRefPool
    4,  // SimpleFrom
    4,  // SimpleTo
    4,  // SimpleVia
    1,  // SimpleMask
    4,  // ComplexTo
    1,  // ComplexMask
    4,  // ComplexPrefixStart
    4,  // ComplexPrefixPool
    1,  // EdgeProfileAccess (bit per profile slot: mobility mode mask overlap)
    4,  // Reserved1 (empty)
    4,  // Reserved2 (empty)
    4,  // Reserved3 (empty)
    4,  // Reserved4 (empty)
    4,  // ComplexFirst: first prefix edge per row of ComplexByFirst order
    4,  // ComplexByFirst: complex rows sorted by first prefix edge
    4, // ShapeStart: byte offset of every SHAPE_START_STRIDEth shared-info shape in ShapeBytes, then its length
    1, // ShapeBytes: varint-encoded E7 polylines (see `encode_shape`)
    4, // ExactShapeShared: sorted shared infos whose shape is not exact E7
    4, // ExactShapeStart: point offsets into ExactShapePoints (len + 1)
    8, // ExactShapePoints: (lat, lng) f64 pairs
    1, // SharedInfoCostFlags (`costing::EdgeCostFlags`), per costing tuple
    4, // SharedInfoCostClasses (`costing::EdgeCostClasses::pack`), likewise
    4, // SharedInfoSpeeds: (maxspeed, free flow, constrained) km/h, NaN = none, likewise
    2, // EdgeInferredSpeed: km/h + 1, 0 = none
    4, // ConditionalEdges: sorted edges whose shared info has conditional tags
    4, // ConditionalShared: sorted shared infos with conditional tags
    4, // ConditionalStart: offsets into ConditionalPool (len + 1)
    4, // ConditionalPool: string ids (request string table)
    8, // RequestMeta: see `request::Meta`
    8, // RequestGrid: [min lat, min lng, cell degrees, rows, cols]
    4, // GridCellStart
    4, // GridEntries: representative compact edges per grid cell
    4, // NodeIdDelta: node id - base (empty when the ids do not fit)
    8, // NodeIdWide: node ids (only when NodeIdDelta cannot hold them)
    4, // ComponentExceptions: (node, component id) pairs differing from the default
    8, // SharedInfoIdGaps: (shared info, ids skipped up to it) where ids skip values
    4, // SharedAttrClass
    4, // AttrClassStart
    1, // AttrClassBytes: varint-encoded correlation attributes + leg shared info
    4, // StringStart
    1, // StringBytes
    4, // OpposingExceptionEdge: sorted compact edges
    4, // OpposingExceptionValue: compact edge (u32::MAX: none)
    4, // LegOpposingExceptionEdge: sorted compact edges
    4, // LegOpposingExceptionValue: compact edge (u32::MAX: none)
    4, // NodeRoundaboutExits: (node, count) pairs for nodes with exits
    4, // ComponentSizes: (component id, size) pairs
    1, // NodeReachSaturated: profile bits whose reach count saturates at the node
    4, // ModeMaskExceptions: (edge, mode mask) where profile bits do not rebuild it
    4, // TierExceptions: (edge, tier code) where the edge's tier is not its shared info's
    1, // EdgeJunction: `junction::encode` per edge
    8, // Reserved5 (zeros)
    4, // Reserved6 (zeros)
    4, // NodeLatLngNodes: sorted nodes with an exact position in NodeLatLng
    2, // SharedInfoCostTuple: costing tuple per shared info
    4, // SharedInfoWayWideShared: sorted shared infos with a wide way id
    8, // SharedInfoWayWide: their way ids
    1, // SharedInfoLegalReason: `sar::LegalReason` per shared info
    4, // BarrierNode: sorted barrier nodes
    1, // BarrierClass: their `sar::SarBarrierClass` codes, 0 for none
    2, // BarrierAccess: their `barrier::BarrierAccess` bits
    8, // Reserved7 (zeros)
    4, // Reserved8 (zeros)
    4, // Reserved9 (empty)
    8, // Reserved10 (zeros)
    4, // Reserved11 (zeros)
    4, // Reserved12 (empty)
    4, // NodeXyzFixed: [node][x, y, z] unit vector times 2^31
    8, // PieceChain: [column][node / 64] marked chain nodes
    8, // PieceTree: [column][node / 64] marked tree nodes
    4, // EdgeTwin: the reverse edge of the same shared info, NO_TWIN for none
    8, // LoneInPresent: [node / 64] targets of edges without a twin
    4, // LoneInEdge: edges without a twin, sorted by (target, edge)
    2, // LoneHeadingEnd: their end headings
    4, // CchRank: [node] rank in the CCH elimination order (format 2)
    16, // PackageParams: (key u32, flags u32, value u64) entries (format 3)
];

pub const SECTION_NAMES: [&str; SECTION_COUNT] = [
    "Metadata",
    "NodeFirstOut",
    "NodeLatE7",
    "NodeLngE7",
    "NodeLatLng",
    "EdgeTarget",
    "EdgeId",
    "EdgeLength",
    "EdgeSharedInfo",
    "EdgeHeadingStart",
    "EdgeFlags",
    "EdgeRoadRank",
    "EdgeCost",
    "EdgeDuration",
    "SharedInfoWay",
    "SharedInfoName",
    "SharedInfoRef",
    "SharedInfoRouteRefStart",
    "SharedInfoRouteRefPool",
    "SimpleFrom",
    "SimpleTo",
    "SimpleVia",
    "SimpleMask",
    "ComplexTo",
    "ComplexMask",
    "ComplexPrefixStart",
    "ComplexPrefixPool",
    "EdgeProfileAccess",
    "Reserved1",
    "Reserved2",
    "Reserved3",
    "Reserved4",
    "ComplexFirst",
    "ComplexByFirst",
    "ShapeStart",
    "ShapeBytes",
    "ExactShapeShared",
    "ExactShapeStart",
    "ExactShapePoints",
    "SharedInfoCostFlags",
    "SharedInfoCostClasses",
    "SharedInfoSpeeds",
    "EdgeInferredSpeed",
    "ConditionalEdges",
    "ConditionalShared",
    "ConditionalStart",
    "ConditionalPool",
    "RequestMeta",
    "RequestGrid",
    "GridCellStart",
    "GridEntries",
    "NodeIdDelta",
    "NodeIdWide",
    "ComponentExceptions",
    "SharedInfoIdGaps",
    "SharedAttrClass",
    "AttrClassStart",
    "AttrClassBytes",
    "StringStart",
    "StringBytes",
    "OpposingExceptionEdge",
    "OpposingExceptionValue",
    "LegOpposingExceptionEdge",
    "LegOpposingExceptionValue",
    "NodeRoundaboutExits",
    "ComponentSizes",
    "NodeReachSaturated",
    "ModeMaskExceptions",
    "TierExceptions",
    "EdgeJunction",
    "Reserved5",
    "Reserved6",
    "NodeLatLngNodes",
    "SharedInfoCostTuple",
    "SharedInfoWayWideShared",
    "SharedInfoWayWide",
    "SharedInfoLegalReason",
    "BarrierNode",
    "BarrierClass",
    "BarrierAccess",
    "Reserved7",
    "Reserved8",
    "Reserved9",
    "Reserved10",
    "Reserved11",
    "Reserved12",
    "NodeXyzFixed",
    "PieceChain",
    "PieceTree",
    "EdgeTwin",
    "LoneInPresent",
    "LoneInEdge",
    "LoneHeadingEnd",
    "CchRank",
    "PackageParams",
];

#[derive(Debug, thiserror::Error)]
pub enum RoutingPackageError {
    #[error("routing package i/o failed: {0}")]
    Io(#[from] std::io::Error),
    #[error("routing package is invalid: {0}")]
    Invalid(String),
    /// The file is a routing package of a format older than this build reads.
    #[error("routing package uses an outdated format ({0}); recompile this package")]
    Outdated(String),
    #[error("routing package cannot be built from this graph: {0}")]
    Unsupported(String),
    /// The file is a routing package of a format newer than this build reads.
    #[error("routing package uses a newer format ({0}); update this app")]
    TooNew(String),
}

impl RoutingPackageError {
    /// Whether the file is missing.
    pub fn is_missing(&self) -> bool {
        matches!(self, Self::Io(error) if error.kind() == std::io::ErrorKind::NotFound)
    }
}

/// Graph-level facts about the package, stored as its JSON metadata section.
///
/// The compiler writes the road graph's full quality manifest as `quality`;
/// readers take the [`PackageQuality`] view of it (unknown fields are skipped),
/// so the format crate needs no road graph types.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct RoutingPackageMetadata<Q = PackageQuality> {
    pub graph_id: String,
    pub country_code: Option<String>,
    pub road_graph_schema_version: u32,
    pub road_graph_blake3_hash_hex: String,
    /// Road graph counts (the package holds the nodes that edges use).
    pub node_count: usize,
    pub directed_edge_count: usize,
    pub shared_edge_info_count: usize,
    /// Union of every edge's mobility mode mask.
    pub mode_mask_union: u32,
    pub quality: Q,
    /// The costing model the stored costs were baked with
    /// (`wayfarer_costing::COSTING_MODEL_VERSION`).
    pub costing_model_version: u32,
    /// The package's SAR data (the physical graph, legal reasons and barrier
    /// nodes, see [`crate::sar`]): the minimum factors of the SAR
    /// relaxations, which the bound metrics of SAR columns use.
    pub sar: crate::sar::SarPackageInfo,
}

/// Keys of [`Section::PackageParams`] (`docs/design/format-3.md`, section
/// 5.4). Keys 8 (time zone) and 9 (country code) are reserved and not
/// written yet.
pub mod param_key {
    /// The costing model the stored costs were baked with. Must understand.
    pub const COSTING_MODEL_VERSION: u32 = 1;
    /// The SAR minimum factors (f64 bits): legal access, wrong way, vehicle
    /// on paths. Must understand.
    pub const SAR_LEGAL_ACCESS_MIN_FACTOR: u32 = 2;
    pub const SAR_WRONG_WAY_MIN_FACTOR: u32 = 3;
    pub const SAR_VEHICLE_ON_PATHS_MIN_FACTOR: u32 = 4;
    /// The union of every edge's mobility mode mask.
    pub const MODE_MASK_UNION: u32 = 5;
    /// The component most nodes belong to (`u32::MAX`: none). Must
    /// understand.
    pub const DEFAULT_COMPONENT: u32 = 6;
    /// The reach cap `NodeReachSaturated` answers for. Must understand.
    pub const REACH_CAP: u32 = 7;
    /// The draft revision of format 3, only while format 3 is being built
    /// ([`super::FORMAT_3_DRAFT_REVISION`]). Must understand.
    pub const DRAFT_REVISION: u32 = 10;
}

/// [`Section::PackageParams`] entry flag: an engine that doesn't know the
/// key must refuse the package.
pub const PARAM_MUST_UNDERSTAND: u32 = 1 << 0;

/// The scalars a query reads beyond the sections: [`Section::PackageParams`]
/// in format 3; in format 2 the JSON metadata's costing model, SAR factors
/// and mode mask union and `RequestMeta`'s default component and reach cap.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PackageParams {
    pub costing_model_version: u32,
    pub sar: crate::sar::SarPackageInfo,
    pub mode_mask_union: u32,
    /// The component most nodes belong to (`u32::MAX`: none).
    pub default_component: u32,
    /// The reach cap `NodeReachSaturated` answers for.
    pub reach_cap: u64,
    /// Format 3's draft revision; `None` in format 2 (and in released
    /// format 3).
    pub draft_revision: Option<u64>,
}

impl PackageParams {
    /// Whether key `key` must be understood, for the keys this build knows.
    fn known_key_flags(key: u32) -> Option<u32> {
        use param_key::*;
        match key {
            COSTING_MODEL_VERSION
            | SAR_LEGAL_ACCESS_MIN_FACTOR
            | SAR_WRONG_WAY_MIN_FACTOR
            | SAR_VEHICLE_ON_PATHS_MIN_FACTOR
            | DEFAULT_COMPONENT
            | REACH_CAP
            | DRAFT_REVISION => Some(PARAM_MUST_UNDERSTAND),
            MODE_MASK_UNION => Some(0),
            _ => None,
        }
    }

    /// The section's words: two per entry (`key | flags << 32`, value),
    /// sorted by key.
    pub fn to_section(&self) -> Vec<u64> {
        use param_key::*;
        let mut entries = vec![
            (COSTING_MODEL_VERSION, u64::from(self.costing_model_version)),
            (
                SAR_LEGAL_ACCESS_MIN_FACTOR,
                self.sar.legal_access_min_factor.to_bits(),
            ),
            (
                SAR_WRONG_WAY_MIN_FACTOR,
                self.sar.wrong_way_min_factor.to_bits(),
            ),
            (
                SAR_VEHICLE_ON_PATHS_MIN_FACTOR,
                self.sar.vehicle_on_paths_min_factor.to_bits(),
            ),
            (MODE_MASK_UNION, u64::from(self.mode_mask_union)),
            (DEFAULT_COMPONENT, u64::from(self.default_component)),
            (REACH_CAP, self.reach_cap),
        ];
        if let Some(revision) = self.draft_revision {
            entries.push((DRAFT_REVISION, revision));
        }
        entries
            .into_iter()
            .flat_map(|(key, value)| {
                let flags = Self::known_key_flags(key).unwrap_or(0);
                [u64::from(key) | (u64::from(flags) << 32), value]
            })
            .collect()
    }

    /// Reads a [`Section::PackageParams`] section: entries sorted by key,
    /// known keys with their flags, every known key but the draft revision
    /// present, an unknown must-understand key `TooNew`, and the draft
    /// revision this build's ([`FORMAT_3_DRAFT_REVISION`]).
    pub fn from_section(bytes: &[u8]) -> Result<Self, RoutingPackageError> {
        use param_key::*;
        let invalid = |message: String| RoutingPackageError::Invalid(message);
        if !bytes.len().is_multiple_of(16) {
            return Err(invalid("package parameters are malformed".into()));
        }
        let mut values = std::collections::BTreeMap::new();
        let mut previous = None;
        for entry in bytes.as_chunks::<16>().0 {
            let key = u32::from_le_bytes(entry[0..4].try_into().unwrap());
            let flags = u32::from_le_bytes(entry[4..8].try_into().unwrap());
            let value = u64::from_le_bytes(entry[8..16].try_into().unwrap());
            if previous.is_some_and(|previous| previous >= key) {
                return Err(invalid("package parameters are not sorted by key".into()));
            }
            previous = Some(key);
            if flags & !PARAM_MUST_UNDERSTAND != 0 {
                return Err(invalid(format!(
                    "package parameter {key} has unknown flags"
                )));
            }
            match Self::known_key_flags(key) {
                Some(expected) if expected != flags => {
                    return Err(invalid(format!(
                        "package parameter {key} has the wrong flags"
                    )));
                }
                Some(_) => {
                    values.insert(key, value);
                }
                None if flags & PARAM_MUST_UNDERSTAND != 0 => {
                    return Err(RoutingPackageError::TooNew(format!(
                        "the package needs parameter {key}, which this build doesn't read"
                    )));
                }
                None => {}
            }
        }
        // A released format 3 has no draft revision: it is newer than any
        // draft.
        match values.get(&DRAFT_REVISION) {
            Some(revision) if *revision < FORMAT_3_DRAFT_REVISION => {
                return Err(RoutingPackageError::Outdated(format!(
                    "format 3 draft revision {revision}, this build reads draft revision {FORMAT_3_DRAFT_REVISION}"
                )));
            }
            Some(revision) if *revision > FORMAT_3_DRAFT_REVISION => {
                return Err(RoutingPackageError::TooNew(format!(
                    "format 3 draft revision {revision}, this build reads draft revision {FORMAT_3_DRAFT_REVISION}"
                )));
            }
            Some(_) => {}
            None => {
                return Err(RoutingPackageError::TooNew(format!(
                    "released format 3, this build reads format 3 draft revision {FORMAT_3_DRAFT_REVISION} only"
                )));
            }
        }
        let get = |key: u32| {
            values
                .get(&key)
                .copied()
                .ok_or_else(|| invalid(format!("package parameter {key} is missing")))
        };
        let get_u32 = |key: u32| {
            u32::try_from(get(key)?)
                .map_err(|_| invalid(format!("package parameter {key} is out of range")))
        };
        Ok(Self {
            costing_model_version: get_u32(COSTING_MODEL_VERSION)?,
            sar: crate::sar::SarPackageInfo {
                legal_access_min_factor: f64::from_bits(get(SAR_LEGAL_ACCESS_MIN_FACTOR)?),
                wrong_way_min_factor: f64::from_bits(get(SAR_WRONG_WAY_MIN_FACTOR)?),
                vehicle_on_paths_min_factor: f64::from_bits(get(SAR_VEHICLE_ON_PATHS_MIN_FACTOR)?),
            },
            mode_mask_union: get_u32(MODE_MASK_UNION)?,
            default_component: get_u32(DEFAULT_COMPONENT)?,
            reach_cap: get(REACH_CAP)?,
            draft_revision: values.get(&DRAFT_REVISION).copied(),
        })
    }
}

/// A section of a format-3 package this build doesn't know (and that isn't
/// REQUIRED, so the package opened without it).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownSection {
    pub id: u32,
    pub param: u32,
    pub flags: u32,
    pub element_bytes: u32,
    pub byte_len: u64,
}

/// What readers use of the road graph's quality manifest.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", default)]
pub struct PackageQuality {
    pub status: PackageQualityStatus,
    pub reasons: Vec<String>,
}

/// The road graph's quality verdict, as its manifest stores it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum PackageQualityStatus {
    #[default]
    Ready,
    Warning,
    Stale,
}

/// Plain-old-data element types that may be viewed in place: the primitive
/// integers and floats a section stores.
///
/// Sealed: [`PackageFile::slice`] reinterprets mapped bytes as `T`, which is
/// sound only for types with no padding, no invalid bit patterns and an
/// alignment of at most 8, so outside crates cannot add implementations.
pub trait Pod: sealed::Sealed + Copy + 'static {}

mod sealed {
    pub trait Sealed {}
}

macro_rules! pod {
    ($($t:ty),*) => {$(
        impl sealed::Sealed for $t {}
        impl Pod for $t {}
    )*};
}
pod!(u8, u16, u32, i32, u64, i64, f32, f64);

pub(crate) enum Storage {
    Mapped(Mmap),
    Owned(Vec<u64>, usize),
}

impl Storage {
    pub(crate) fn bytes(&self) -> &[u8] {
        match self {
            Self::Mapped(map) => map,
            Self::Owned(words, len) => {
                // SAFETY: a `Vec<u64>` is a valid, initialised byte buffer of
                // `8 * words.len()` bytes; `len` never exceeds that.
                unsafe { std::slice::from_raw_parts(words.as_ptr().cast::<u8>(), *len) }
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PackageCounts {
    pub node_count: usize,
    pub edge_count: usize,
    pub shared_info_count: usize,
    pub simple_restriction_count: usize,
    pub complex_restriction_count: usize,
    pub column_count: usize,
}

/// How derivable per-element data is stored; kept in the header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PackageLayout {
    /// Node E7 positions are recomputed from `NodeXyzFixed` and the exact
    /// positions (checked at build), so `NodeLatE7`/`NodeLngE7` are not
    /// stored.
    pub node_e7_derived: bool,
    /// The smallest edge id: `EdgeId` stores `id - base`.
    pub edge_id_base: u64,
}

// Header layout (little endian).
pub const HEADER_VERSION: usize = 8;
pub const HEADER_SECTION_COUNT: usize = 12;
/// Six counts as u64 (see [`PackageCounts`]), then a reserved u64 (format
/// 1's landmark count), zero.
pub const HEADER_COUNTS: usize = 16; // 7 x u64
pub const HEADER_LAYOUT_FLAGS: usize = 72;
pub const HEADER_EDGE_ID_BASE: usize = 80;
pub const HEADER_PROFILE_COLUMNS: usize = 96; // 6 x u8
pub const HEADER_HEURISTIC: usize = 104; // 6 x f64
pub const HEADER_ROOT_CHECKSUM: usize = 152; // 32 bytes
pub const LAYOUT_NODE_E7_DERIVED: u32 = 1 << 0;

// Format 3 header (little endian; `docs/design/format-3.md`, section 2.1).
// The magic and version are where format 2 has them, and the table entry
// count is where format 2 has its section count.
pub const HEADER_V3_TABLE_OFFSET: usize = 16; // u64, 64-aligned
pub const HEADER_V3_COUNTS: usize = 24; // 6 x u64 (see [`PackageCounts`])
/// u64; bit 0 [`LAYOUT_NODE_E7_DERIVED`], every other bit must be zero.
pub const HEADER_V3_LAYOUT_FLAGS: usize = 72;
pub const HEADER_V3_EDGE_ID_BASE: usize = 80;
/// Draft revision 1 keeps format 2's profile columns (6 x u8) and heuristic
/// costs (6 x f64) at format 2's offsets, inside the header's zero area,
/// until `ProfileSlots` replaces them.
pub const HEADER_V3_PROFILE_COLUMNS: usize = HEADER_PROFILE_COLUMNS;
pub const HEADER_V3_HEURISTIC: usize = HEADER_HEURISTIC;
pub const HEADER_V3_ROOT_CHECKSUM: usize = 224; // 32 bytes
/// The format-3 header bytes that must be zero.
const HEADER_V3_ZERO: [std::ops::Range<usize>; 3] = [
    88..HEADER_V3_PROFILE_COLUMNS,
    HEADER_V3_PROFILE_COLUMNS + PROFILE_SLOTS..HEADER_V3_HEURISTIC,
    HEADER_V3_HEURISTIC + PROFILE_SLOTS * 8..HEADER_V3_ROOT_CHECKSUM,
];

/// A validated routing package file: its storage, header and section table.
/// The engine wraps it with its query-time state.
pub struct PackageFile {
    pub(crate) storage: Storage,
    pub(crate) counts: PackageCounts,
    pub(crate) layout: PackageLayout,
    /// Each known section's (offset, length); (0, 0) for one the package
    /// doesn't hold.
    pub(crate) sections: [(usize, usize); SECTION_COUNT],
    /// The identity hash (format 3), or the root checksum (format 2).
    pub(crate) dataset_hash: String,
    /// The root checksum.
    pub(crate) content_hash: String,
    pub(crate) unknown_sections: Vec<UnknownSection>,
    /// In format 3 the costing model, SAR factors and mode mask union come
    /// from [`Section::PackageParams`] (see [`Self::params`]), not the JSON.
    pub(crate) metadata: RoutingPackageMetadata,
    pub(crate) params: PackageParams,
    pub(crate) profile_columns: [u8; PROFILE_SLOTS],
    pub(crate) heuristic_cost_per_m: [f64; PROFILE_SLOTS],
    pub(crate) format_version: u32,
    /// The edge id lookup, built on first use (see [`Sections::edge_for_id`]).
    pub(crate) edge_id_lookup: OnceLock<Vec<u32>>,
}

impl std::fmt::Debug for PackageFile {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PackageFile")
            .field("counts", &self.counts)
            .field("dataset_hash", &self.dataset_hash)
            .field("graph_id", &self.metadata.graph_id)
            .field("profile_columns", &self.profile_columns)
            .finish()
    }
}

pub fn profile_slot(profile: MobilityProfile) -> usize {
    match profile {
        MobilityProfile::Default => 0,
        MobilityProfile::Wheeled => 1,
        MobilityProfile::Tracked => 2,
        MobilityProfile::Emergency => 3,
        MobilityProfile::OnFoot => 4,
        MobilityProfile::Legged => 5,
    }
}

pub fn profile_bit(profile: MobilityProfile) -> u8 {
    1 << profile_slot(profile)
}

/// What [`PackageFile::move_into_place`] puts before the unique part of the
/// name of a package it moved aside.
const REPLACED_INFIX: &str = ".replaced-";

/// [`PackageFile::move_into_place`] when a rename over `path` failed: moves
/// the package at `path` aside, `temp` into its place, and deletes the old
/// one if nothing has it open any more. Puts the old one back if the second
/// rename fails.
fn move_into_place_aside(temp: &Path, path: &Path) -> std::io::Result<()> {
    use std::sync::atomic::{AtomicU64, Ordering};
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |since| since.as_nanos());
    let mut aside = path.as_os_str().to_owned();
    aside.push(format!(
        "{REPLACED_INFIX}{}-{nanos}-{}",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    let aside = PathBuf::from(aside);
    std::fs::rename(path, &aside)?;
    if let Err(error) = std::fs::rename(temp, path) {
        let _ = std::fs::rename(&aside, path);
        return Err(error);
    }
    // Fails while an engine still has it mapped; a later call retries.
    let _ = std::fs::remove_file(&aside);
    Ok(())
}

/// Deletes the packages [`move_into_place_aside`] moved aside from `path`
/// that nothing has open any more. Best effort: errors are ignored.
fn remove_replaced(path: &Path) {
    let Some(name) = path.file_name().and_then(|name| name.to_str()) else {
        return;
    };
    let prefix = format!("{name}{REPLACED_INFIX}");
    let dir = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        let file_name = entry.file_name();
        let Some(unique) = file_name
            .to_str()
            .and_then(|file_name| file_name.strip_prefix(&prefix))
        else {
            continue;
        };
        if !unique.is_empty()
            && unique
                .bytes()
                .all(|byte| byte.is_ascii_digit() || byte == b'-')
        {
            let _ = std::fs::remove_file(entry.path());
        }
    }
}

/// One section's size and hash, as the section table records it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RoutingPackageSection {
    /// The section's name; `"unknown"` for a format-3 section this build
    /// doesn't know.
    pub name: &'static str,
    /// The format-3 section id (format 2: the slot number).
    pub id: u32,
    pub byte_len: u64,
    pub blake3_hex: String,
}

/// What [`PackageFile::verify`] checked.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RoutingPackageVerification {
    /// The package's dataset hash: the identity hash in format 3, the root
    /// checksum in format 2.
    pub dataset_hash: String,
    /// The root checksum.
    pub content_hash: String,
    pub byte_len: u64,
    /// In file order.
    pub sections: Vec<RoutingPackageSection>,
}

impl PackageFile {
    /// Whether the edge id lookup has been built (by a request that names an
    /// edge id: blocks, overlays, multipliers, opened edges, hints).
    pub fn edge_id_lookup_built(&self) -> bool {
        self.edge_id_lookup.get().is_some()
    }

    /// Opens a package: checks the header, the root checksum over the header
    /// and section table, and every section's bounds and structure. Sections
    /// are not hashed (see [`Self::verify`]). An older package format is
    /// `Outdated`.
    pub fn open(path: &Path) -> Result<Self, RoutingPackageError> {
        let file = File::open(path)?;
        // SAFETY: packages are written atomically (temp file + rename, see
        // `move_into_place`) and never modified in place; the mapping is
        // validated before any use. `File::open` shares delete access on
        // Windows, which lets a newer package be moved into place while this
        // one stays mapped.
        let map = unsafe { Mmap::map(&file)? };
        Self::from_storage(Storage::Mapped(map))
    }

    /// Opens a package held in memory.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, RoutingPackageError> {
        let mut words = vec![0u64; bytes.len().div_ceil(8)];
        // SAFETY: `words` holds at least `bytes.len()` bytes.
        unsafe {
            std::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                words.as_mut_ptr().cast::<u8>(),
                bytes.len(),
            );
        }
        Self::from_storage(Storage::Owned(words, bytes.len()))
    }

    /// Checks a package file end to end: the header, the root checksum, every
    /// section's hash against the section table, zero padding, the length,
    /// and the sections' structure. Reads the whole file once; meant for the
    /// install-time check of a package that came over a network.
    pub fn verify(path: &Path) -> Result<RoutingPackageVerification, RoutingPackageError> {
        use std::io::{Seek, SeekFrom};
        let mut file = File::open(path)?;
        let byte_len = file.metadata()?.len();
        let file_len = usize::try_from(byte_len)
            .map_err(|_| RoutingPackageError::Invalid("file exceeds the address space".into()))?;
        let mut prefix = vec![0u8; HEADER_FIXED_BYTES.min(file_len)];
        file.read_exact(&mut prefix)?;
        let header = parse_header(&prefix, file_len, |offset, len| {
            let mut table = vec![0u8; len];
            file.seek(SeekFrom::Start(offset as u64))?;
            file.read_exact(&mut table)?;
            Ok(table)
        })?;
        // The table and the sections in file order, with the padding between
        // them; the table itself is covered by the root checksum.
        let mut regions = header
            .entries
            .iter()
            .map(|entry| (entry.offset, entry.len, Some(entry)))
            .chain(std::iter::once((header.table.0, header.table.1, None)))
            .collect::<Vec<_>>();
        regions.sort_by_key(|(offset, len, _)| (*offset, *offset + *len));
        file.seek(SeekFrom::Start(HEADER_FIXED_BYTES as u64))?;
        let mut sections = Vec::with_capacity(header.entries.len());
        let mut at = HEADER_FIXED_BYTES;
        let mut buffer = vec![0u8; 1 << 20];
        let skip_padding = |file: &mut File, at: &mut usize, to: usize| {
            let mut padding = vec![0u8; to - *at];
            file.read_exact(&mut padding)?;
            if padding.iter().any(|byte| *byte != 0) {
                return Err(RoutingPackageError::Invalid(
                    "section padding is not zero".into(),
                ));
            }
            *at = to;
            Ok(())
        };
        for (offset, len, entry) in regions {
            skip_padding(&mut file, &mut at, offset)?;
            let mut hasher = blake3::Hasher::new();
            let mut remaining = len;
            while remaining > 0 {
                let chunk = remaining.min(buffer.len());
                file.read_exact(&mut buffer[..chunk])?;
                hasher.update(&buffer[..chunk]);
                remaining -= chunk;
            }
            at += len;
            let Some(entry) = entry else {
                continue;
            };
            let hash = hasher.finalize();
            let name = entry.section.map_or("unknown", Section::name);
            if hash.as_bytes() != &entry.hash {
                return Err(RoutingPackageError::Invalid(format!(
                    "section {name} (id {}) does not match its checksum",
                    entry.id
                )));
            }
            sections.push(RoutingPackageSection {
                name,
                id: entry.id,
                byte_len: len as u64,
                blake3_hex: hash.to_hex().to_string(),
            });
        }
        skip_padding(&mut file, &mut at, file_len)?;
        // Structure (bounds, monotone offsets, index ranges) on the hashed bytes.
        let graph = Self::open(path)?;
        Ok(RoutingPackageVerification {
            dataset_hash: graph.dataset_hash,
            content_hash: graph.content_hash,
            byte_len,
            sections,
        })
    }

    /// Writes `bytes` to `path` atomically (tests).
    #[doc(hidden)]
    pub fn write_file(path: &Path, bytes: &[u8]) -> Result<(), RoutingPackageError> {
        Self::write_atomically(path, |file| file.write_all(bytes))
    }

    /// Streams `writer`'s encoding to `path` atomically (temp file + rename).
    pub fn write_encoded(path: &Path, writer: &PackageWriter) -> Result<(), RoutingPackageError> {
        Self::write_atomically(path, |file| {
            let mut buffered = std::io::BufWriter::with_capacity(1 << 20, file);
            writer.write_to(&mut buffered)?;
            buffered.flush()
        })
    }

    fn write_atomically(
        path: &Path,
        write: impl FnOnce(&mut File) -> std::io::Result<()>,
    ) -> Result<(), RoutingPackageError> {
        if let Some(parent) = path
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
        {
            std::fs::create_dir_all(parent)?;
        }
        let mut temp = path.as_os_str().to_owned();
        temp.push(".tmp");
        let temp = PathBuf::from(temp);
        let result = (|| {
            let mut file = File::create(&temp)?;
            write(&mut file)?;
            file.sync_all()?;
            Self::move_into_place(&temp, path)
        })();
        if result.is_err() {
            let _ = std::fs::remove_file(&temp);
        }
        Ok(result?)
    }

    /// Moves a finished package from `temp` onto `path`, replacing the file
    /// there even while it is open.
    ///
    /// A rename does that on Linux, macOS, and Windows on NTFS and ReFS
    /// (std renames with POSIX semantics there): an engine that has the old
    /// file open keeps reading it until it closes. Where Windows can't rename
    /// with POSIX semantics (FAT32, exFAT, some network shares), a rename over
    /// a memory-mapped package fails with "Access is denied". There the open
    /// package is renamed aside instead to
    /// `<name>.replaced-<unique>` (allowed: packages are opened with delete
    /// sharing, see [`Self::open`]), the new one takes its place, and the old
    /// one is deleted as soon as nothing has it open: at once if it can be,
    /// otherwise by the next call for the same `path`. Between the two
    /// renames `path` is briefly missing, so an open at that moment fails
    /// with not found. A process that opened the package without delete
    /// sharing (outside Wayfarer) still blocks the replacement.
    pub fn move_into_place(temp: &Path, path: &Path) -> std::io::Result<()> {
        remove_replaced(path);
        match std::fs::rename(temp, path) {
            Err(error)
                if cfg!(windows)
                    && error.kind() == std::io::ErrorKind::PermissionDenied
                    && path.is_file() =>
            {
                move_into_place_aside(temp, path)
            }
            result => result,
        }
    }

    fn from_storage(storage: Storage) -> Result<Self, RoutingPackageError> {
        let bytes = storage.bytes();
        if !(bytes.as_ptr() as usize).is_multiple_of(8) {
            return Err(RoutingPackageError::Invalid(
                "storage is not 8-byte aligned".into(),
            ));
        }
        let header = parse_header(bytes, bytes.len(), |offset, len| {
            Ok(bytes[offset..offset + len].to_vec())
        })?;
        let section_bytes = |section: Section| {
            let (offset, len) = header.sections[section as usize];
            &bytes[offset..offset + len]
        };
        let mut metadata =
            serde_json::from_slice::<RoutingPackageMetadata>(section_bytes(Section::Metadata))
                .map_err(|error| RoutingPackageError::Invalid(format!("metadata: {error}")))?;
        let params = if header.version >= 3 {
            // The JSON is informational: queries read the parameters.
            let params = PackageParams::from_section(section_bytes(Section::PackageParams))?;
            metadata.costing_model_version = params.costing_model_version;
            metadata.sar = params.sar;
            metadata.mode_mask_union = params.mode_mask_union;
            params
        } else {
            let meta = section_bytes(Section::RequestMeta);
            let word = |at: usize| {
                meta.get(at * 8..at * 8 + 8)
                    .map(|word| u64::from_le_bytes(word.try_into().unwrap()))
            };
            if meta.len() != crate::request::META_LEN * 8
                || word(0) != Some(crate::request::REQUEST_LAYOUT)
            {
                return Err(RoutingPackageError::Invalid(
                    "request metadata is missing or of another layout".into(),
                ));
            }
            PackageParams {
                costing_model_version: metadata.costing_model_version,
                sar: metadata.sar,
                mode_mask_union: metadata.mode_mask_union,
                default_component: u32::try_from(word(3).unwrap_or_default()).map_err(|_| {
                    RoutingPackageError::Invalid("request sections are inconsistent".into())
                })?,
                reach_cap: word(5).unwrap_or_default(),
                draft_revision: None,
            }
        };
        let graph = Self {
            storage,
            counts: header.counts,
            layout: header.layout,
            sections: header.sections,
            dataset_hash: header.identity_hash_hex,
            content_hash: header.root_checksum_hex,
            unknown_sections: header.unknown_sections,
            metadata,
            params,
            profile_columns: header.profile_columns,
            heuristic_cost_per_m: header.heuristic_cost_per_m,
            format_version: header.version,
            edge_id_lookup: OnceLock::new(),
        };
        graph.validate()?;
        graph.validate_request_sections()?;
        Ok(graph)
    }

    pub fn slice<T: Pod>(&self, section: Section) -> &[T] {
        let (offset, len) = self.sections[section as usize];
        debug_assert_eq!(
            std::mem::size_of::<T>(),
            SECTION_ELEMENT_BYTES[section as usize]
        );
        let bytes = &self.storage.bytes()[offset..offset + len];
        // SAFETY: offsets are 64-byte aligned within 8-byte-aligned storage,
        // lengths are multiples of the element size (checked at open), and every
        // `Pod` type accepts any bit pattern.
        unsafe {
            std::slice::from_raw_parts(bytes.as_ptr().cast::<T>(), len / std::mem::size_of::<T>())
        }
    }

    fn validate(&self) -> Result<(), RoutingPackageError> {
        let invalid = |message: &str| Err(RoutingPackageError::Invalid(message.to_string()));
        let c = self.counts;
        // Counts come straight from the header: every size derived from them
        // is checked arithmetic, and one that overflows matches no section.
        let expect = |section: Section, elements: Option<usize>| {
            elements
                .and_then(|elements| elements.checked_mul(SECTION_ELEMENT_BYTES[section as usize]))
                == Some(self.sections[section as usize].1)
        };
        let mul = |a: usize, b: usize| a.checked_mul(b);
        let plus_one = |a: usize| a.checked_add(1);
        let stored_nodes = if self.layout.node_e7_derived {
            0
        } else {
            c.node_count
        };
        let lat_lng = self.slice::<f64>(Section::NodeLatLng).len();
        self.check_piece_kinds()?;
        // Costing inputs per costing tuple.
        let cost_rows = self.slice::<u8>(Section::SharedInfoCostFlags).len();
        // Name and ref tokens per attribute class (classes are checked
        // against the shared infos with the request sections).
        let token_rows = self
            .slice::<u32>(Section::AttrClassStart)
            .len()
            .saturating_sub(1);
        // Format 3 implies the edge ids and drops format 1's reserved slots.
        let format_3 = self.format_version >= 3;
        let reserved_words = if format_3 {
            Some(0)
        } else {
            c.column_count.checked_mul(c.node_count.div_ceil(64))
        };
        let stored_edge_ids = if format_3 { 0 } else { c.edge_count };
        let fixed = [
            (Section::NodeFirstOut, plus_one(c.node_count)),
            (Section::NodeXyzFixed, mul(c.node_count, 3)),
            (Section::NodeLatE7, Some(stored_nodes)),
            (Section::NodeLngE7, Some(stored_nodes)),
            (Section::EdgeTarget, Some(c.edge_count)),
            (Section::EdgeId, Some(stored_edge_ids)),
            (Section::EdgeLength, Some(c.edge_count)),
            (Section::EdgeSharedInfo, Some(c.edge_count)),
            (Section::EdgeHeadingStart, Some(c.edge_count)),
            (Section::EdgeFlags, Some(c.edge_count)),
            (Section::EdgeRoadRank, Some(c.edge_count)),
            (Section::EdgeCost, mul(c.edge_count, c.column_count)),
            (Section::EdgeDuration, mul(c.edge_count, c.column_count)),
            (Section::SharedInfoWay, Some(c.shared_info_count)),
            (Section::SharedInfoName, Some(token_rows)),
            (Section::SharedInfoRef, Some(token_rows)),
            (Section::SharedInfoRouteRefStart, plus_one(token_rows)),
            (Section::SimpleFrom, Some(c.simple_restriction_count)),
            (Section::SimpleTo, Some(c.simple_restriction_count)),
            (Section::SimpleVia, Some(c.simple_restriction_count)),
            (Section::SimpleMask, Some(c.simple_restriction_count)),
            (Section::ComplexTo, Some(c.complex_restriction_count)),
            (Section::ComplexMask, Some(c.complex_restriction_count)),
            (
                Section::ComplexPrefixStart,
                plus_one(c.complex_restriction_count),
            ),
            (Section::EdgeTwin, Some(c.edge_count)),
            (Section::EdgeProfileAccess, Some(c.edge_count)),
            (Section::Reserved1, Some(0)),
            (Section::Reserved2, Some(0)),
            (Section::Reserved3, Some(0)),
            (Section::Reserved4, Some(0)),
            (Section::Reserved5, reserved_words),
            (Section::Reserved6, reserved_words),
            (Section::Reserved7, reserved_words),
            (Section::Reserved8, reserved_words),
            (Section::Reserved9, Some(0)),
            (Section::Reserved10, reserved_words),
            (Section::Reserved11, reserved_words),
            (Section::Reserved12, Some(0)),
            (Section::ComplexFirst, Some(c.complex_restriction_count)),
            (Section::ComplexByFirst, Some(c.complex_restriction_count)),
            (
                Section::ShapeStart,
                plus_one(c.shared_info_count.div_ceil(SHAPE_START_STRIDE)),
            ),
            (Section::SharedInfoCostFlags, Some(cost_rows)),
            (Section::SharedInfoCostClasses, Some(cost_rows)),
            (Section::SharedInfoSpeeds, mul(cost_rows, 3)),
            (Section::EdgeInferredSpeed, Some(c.edge_count)),
            (Section::SharedInfoCostTuple, Some(c.shared_info_count)),
            (Section::SharedInfoLegalReason, Some(c.shared_info_count)),
            (Section::EdgeJunction, Some(c.edge_count)),
        ];
        if fixed
            .iter()
            .any(|(section, count)| !expect(*section, *count))
        {
            return invalid("section length does not match header counts");
        }
        if RESERVED_ZERO_SECTIONS.iter().any(|section| {
            let (offset, len) = self.sections[*section as usize];
            self.storage.bytes()[offset..offset + len]
                .iter()
                .any(|byte| *byte != 0)
        }) {
            return invalid("reserved section is not zero");
        }
        let nodes = self.slice::<u32>(Section::NodeLatLngNodes);
        if lat_lng != nodes.len() * 2
            || nodes.windows(2).any(|pair| pair[0] >= pair[1])
            || nodes
                .last()
                .is_some_and(|node| *node as usize >= c.node_count)
        {
            return invalid("exact node positions are invalid");
        }
        let tuples = cost_rows;
        if tuples > MAX_COST_TUPLES
            || (c.shared_info_count > 0 && tuples == 0)
            || self
                .slice::<u16>(Section::SharedInfoCostTuple)
                .iter()
                .any(|tuple| usize::from(*tuple) >= tuples)
        {
            return invalid("costing tuples are invalid");
        }
        let wide_shared = self.slice::<u32>(Section::SharedInfoWayWideShared);
        if wide_shared.len() != self.slice::<i64>(Section::SharedInfoWayWide).len()
            || wide_shared.windows(2).any(|pair| pair[0] >= pair[1])
            || wide_shared
                .last()
                .is_some_and(|shared| *shared as usize >= c.shared_info_count)
        {
            return invalid("wide way ids are invalid");
        }
        if !self.metadata.sar.is_valid() {
            return invalid("sar minimum factors are invalid");
        }
        if self
            .slice::<u8>(Section::SharedInfoLegalReason)
            .iter()
            .any(|reason| !crate::sar::LegalReason(*reason).is_valid())
        {
            return invalid("legal reasons are invalid");
        }
        let barrier_nodes = self.slice::<u32>(Section::BarrierNode);
        let barrier_classes = self.slice::<u8>(Section::BarrierClass);
        let barrier_access = self.slice::<u16>(Section::BarrierAccess);
        if barrier_nodes.len() != barrier_classes.len()
            || barrier_nodes.len() != barrier_access.len()
            || barrier_nodes.windows(2).any(|pair| pair[0] >= pair[1])
            || barrier_nodes
                .last()
                .is_some_and(|node| *node as usize >= c.node_count)
            || barrier_classes
                .iter()
                .zip(barrier_access)
                .any(|(class, access)| {
                    let access = crate::barrier::BarrierAccess(*access);
                    (*class != 0 && crate::sar::SarBarrierClass::from_code(*class).is_none())
                        || !access.is_valid()
                        || (*class == 0 && !access.has_effect())
                })
        {
            return invalid("barrier nodes are invalid");
        }
        if self.slice::<u8>(Section::EdgeJunction).iter().any(|byte| {
            byte & !(junction::CONTROL_MASK | junction::MINOR_APPROACH) != 0
                || byte & junction::CONTROL_MASK > TrafficControl::MAX_CODE
        }) {
            return invalid("edge junction section is malformed");
        }
        // Headings are whole degrees below 360 (costing folds their
        // differences into 0..=180 and would overflow past it), or none.
        let heading_ok = |heading: &u16| *heading < 360 || *heading == NO_HEADING;
        if !self
            .slice::<u16>(Section::EdgeHeadingStart)
            .iter()
            .all(heading_ok)
            || !self
                .slice::<u16>(Section::LoneHeadingEnd)
                .iter()
                .all(heading_ok)
        {
            return invalid("edge headings out of range");
        }
        if c.node_count >= u32::MAX as usize || c.edge_count >= u32::MAX as usize {
            return invalid("graph exceeds u32 indexing");
        }
        if self
            .profile_columns
            .iter()
            .any(|column| *column != NO_COLUMN && usize::from(*column) >= c.column_count)
        {
            return invalid("profile column out of range");
        }
        let first_out = self.slice::<u32>(Section::NodeFirstOut);
        if first_out.first() != Some(&0)
            || first_out.last().map(|last| *last as usize) != Some(c.edge_count)
            || first_out.windows(2).any(|pair| pair[0] > pair[1])
        {
            return invalid("node adjacency offsets are not monotonic");
        }
        let n = c.node_count as u32;
        let e = c.edge_count as u32;
        let s = c.shared_info_count as u32;
        if self
            .slice::<u32>(Section::EdgeTarget)
            .iter()
            .any(|v| *v >= n)
            || self
                .slice::<u32>(Section::EdgeSharedInfo)
                .iter()
                .any(|v| *v >= s)
        {
            return invalid("edge reference out of range");
        }
        // The searches need non-negative arc costs: a negative or NaN one
        // breaks the label order (a path's predecessors can then form a
        // cycle). Infinity marks an edge a column can't use.
        let non_negative = |section: Section| self.slice::<f32>(section).iter().all(|v| *v >= 0.0);
        let finite = |section: Section| {
            self.slice::<f32>(section)
                .iter()
                .all(|v| v.is_finite() && *v >= 0.0)
        };
        if !non_negative(Section::EdgeCost)
            || !non_negative(Section::EdgeDuration)
            || !finite(Section::EdgeLength)
            || !self.heuristic_cost_per_m.iter().all(|v| *v >= 0.0)
        {
            return invalid("edge costs, durations, lengths or bound factors are invalid");
        }
        // The lookup is built on first use: the ids must be distinct, and
        // span no more than the compiler allows.
        let span_limit = edge_id_span_limit(c.edge_count);
        // Ids are the base plus a slot below the span limit, in u64.
        if self
            .layout
            .edge_id_base
            .checked_add(span_limit as u64)
            .is_none()
        {
            return invalid("edge id base overflows");
        }
        let mut seen = vec![0u64; span_limit.div_ceil(64)];
        for slot in self.slice::<u32>(Section::EdgeId) {
            let slot = *slot as usize;
            if slot >= span_limit || seen[slot / 64] & (1 << (slot % 64)) != 0 {
                return invalid("edge ids are not distinct or span too many values");
            }
            seen[slot / 64] |= 1 << (slot % 64);
        }
        self.check_twins()?;
        let pool_len = self.slice::<u32>(Section::SharedInfoRouteRefPool).len();
        let starts = self.slice::<u32>(Section::SharedInfoRouteRefStart);
        if starts.first().is_some_and(|first| *first != 0)
            || starts.windows(2).any(|pair| pair[0] > pair[1])
            || starts.last().is_some_and(|last| *last as usize != pool_len)
        {
            return invalid("route-ref token offsets are invalid");
        }
        let simple_from = self.slice::<u32>(Section::SimpleFrom);
        if simple_from.windows(2).any(|pair| pair[0] > pair[1])
            || simple_from.iter().any(|v| *v >= e)
            || self.slice::<u32>(Section::SimpleTo).iter().any(|v| *v >= e)
            || self
                .slice::<u32>(Section::SimpleVia)
                .iter()
                .any(|v| *v >= n)
        {
            return invalid("simple restrictions are invalid");
        }
        let complex_to = self.slice::<u32>(Section::ComplexTo);
        let prefix_pool = self.slice::<u32>(Section::ComplexPrefixPool);
        let prefix_starts = self.slice::<u32>(Section::ComplexPrefixStart);
        let complex_first = self.slice::<u32>(Section::ComplexFirst);
        let complex_by_first = self.slice::<u32>(Section::ComplexByFirst);
        if complex_first.windows(2).any(|pair| pair[0] > pair[1])
            || complex_by_first
                .iter()
                .any(|row| *row as usize >= c.complex_restriction_count)
        {
            return invalid("complex restriction first-edge index is invalid");
        }
        if complex_to.windows(2).any(|pair| pair[0] > pair[1])
            || complex_to.iter().any(|v| *v >= e)
            || prefix_pool.iter().any(|v| *v >= e)
            || prefix_starts.windows(2).any(|pair| pair[0] >= pair[1])
            || prefix_starts.first().is_some_and(|first| *first != 0)
            || prefix_starts
                .last()
                .is_some_and(|last| *last as usize != prefix_pool.len())
        {
            return invalid("complex restrictions are invalid");
        }
        if self
            .slice::<u8>(Section::EdgeRoadRank)
            .iter()
            .any(|rank| road_type_from_rank(*rank).is_none())
        {
            return invalid("edge road rank out of range");
        }
        let conditional = self.slice::<u32>(Section::ConditionalEdges);
        if conditional.windows(2).any(|pair| pair[0] >= pair[1])
            || conditional.iter().any(|edge| *edge >= e)
        {
            return invalid("conditional edge list is invalid");
        }
        let conditional_shared = self.slice::<u32>(Section::ConditionalShared);
        let conditional_start = self.slice::<u32>(Section::ConditionalStart);
        if conditional_shared.windows(2).any(|pair| pair[0] >= pair[1])
            || conditional_shared.iter().any(|shared| *shared >= s)
            || conditional_start.len() != conditional_shared.len() + 1
            || conditional_start.first() != Some(&0)
            || conditional_start.windows(2).any(|pair| pair[0] > pair[1])
            || conditional_start.last().map(|last| *last as usize)
                != Some(self.slice::<u32>(Section::ConditionalPool).len())
        {
            return invalid("conditional tag lists are invalid");
        }
        let shape_starts = self.slice::<u32>(Section::ShapeStart);
        if shape_starts.first().is_some_and(|first| *first != 0)
            || shape_starts.windows(2).any(|pair| pair[0] > pair[1])
            || shape_starts
                .last()
                .is_some_and(|last| *last as usize != self.sections[Section::ShapeBytes as usize].1)
        {
            return invalid("shape offsets are invalid");
        }
        let exact_shared = self.slice::<u32>(Section::ExactShapeShared);
        let exact_start = self.slice::<u32>(Section::ExactShapeStart);
        if exact_shared.windows(2).any(|pair| pair[0] >= pair[1])
            || exact_shared.iter().any(|shared| *shared >= s)
            || exact_start.len() != exact_shared.len() + 1
            || exact_start.first() != Some(&0)
            || exact_start.windows(2).any(|pair| pair[0] > pair[1])
            || exact_start.last().map(|last| *last as usize * 2)
                != Some(self.slice::<f64>(Section::ExactShapePoints).len())
        {
            return invalid("exact shapes are invalid");
        }
        self.check_chain_marks()?;
        Ok(())
    }

    /// Marked chain nodes see what the route search's walks rely on (see
    /// [`PieceKinds`]): over the stored column's usable arcs, leaving out
    /// its marked tree nodes, at most two neighbours, no self-loop, and at
    /// most one arc each way to each. A walk then has at most one way to
    /// continue at every chain node. Needs the adjacency, twins and costs
    /// checked.
    fn check_chain_marks(&self) -> Result<(), RoutingPackageError> {
        use rayon::prelude::*;
        let view = self.view();
        let chain = self.slice::<u64>(Section::PieceChain);
        if chain.is_empty() {
            return Ok(());
        }
        let c = self.counts;
        let words = c.node_count.div_ceil(64);
        let valid = (0..c.column_count).into_par_iter().all(|column| {
            let Some(kinds) = view.piece_kinds.column(column) else {
                return false;
            };
            let cost = &view.edge_cost[column * c.edge_count..(column + 1) * c.edge_count];
            chain[column * words..(column + 1) * words]
                .par_iter()
                .enumerate()
                .all(|(word, bits)| {
                    let mut bits = *bits;
                    while bits != 0 {
                        let node = (word * 64) as u32 + bits.trailing_zeros();
                        bits &= bits - 1;
                        if !chain_node_is_valid(&view, kinds, cost, node) {
                            return false;
                        }
                    }
                    true
                })
        });
        if valid {
            Ok(())
        } else {
            Err(RoutingPackageError::Invalid(
                "piece kinds are invalid".into(),
            ))
        }
    }

    /// Twins pair edges
    /// of one shared info between the same two nodes in opposite directions
    /// (an involution without fixed points), and `LoneInEdge` lists exactly
    /// the edges without a twin, by (target, edge), with their targets'
    /// bits in `LoneInPresent`. Then the twins of a node's out-edges and its
    /// lone in-edges are exactly the edges into it.
    fn check_twins(&self) -> Result<(), RoutingPackageError> {
        let c = self.counts;
        let invalid = || RoutingPackageError::Invalid("edge twins are invalid".into());
        let first_out = self.slice::<u32>(Section::NodeFirstOut);
        let targets = self.slice::<u32>(Section::EdgeTarget);
        let shared = self.slice::<u32>(Section::EdgeSharedInfo);
        let twins = self.slice::<u32>(Section::EdgeTwin);
        let mut lone = 0;
        for node in 0..c.node_count {
            for edge in first_out[node]..first_out[node + 1] {
                let twin = twins[edge as usize];
                if twin == NO_TWIN {
                    lone += 1;
                    continue;
                }
                let at = twin as usize;
                if at >= c.edge_count
                    || twin == edge
                    || twins[at] != edge
                    || targets[at] as usize != node
                    || shared[at] != shared[edge as usize]
                {
                    return Err(invalid());
                }
            }
        }
        let rank = self.slice::<u32>(Section::CchRank);
        if !rank.is_empty()
            && (rank.len() != c.node_count || !crate::cch_order::is_permutation(rank))
        {
            return Err(invalid());
        }
        let present = self.slice::<u64>(Section::LoneInPresent);
        let list = self.slice::<u32>(Section::LoneInEdge);
        if present.len() != c.node_count.div_ceil(64)
            || list.len() != lone
            || self.slice::<u16>(Section::LoneHeadingEnd).len() != lone
        {
            return Err(invalid());
        }
        let mut bits = vec![0u64; present.len()];
        let mut previous = None;
        for edge in list {
            if *edge as usize >= c.edge_count || twins[*edge as usize] != NO_TWIN {
                return Err(invalid());
            }
            let target = targets[*edge as usize];
            if previous.is_some_and(|key| key >= (target, *edge)) {
                return Err(invalid());
            }
            previous = Some((target, *edge));
            bits[target as usize / 64] |= 1 << (target % 64);
        }
        if bits != present {
            return Err(invalid());
        }
        Ok(())
    }

    /// Piece kinds: one word per (column, 64 nodes) in each section, or none
    /// at all; a node is not both; no bits past the last node.
    fn check_piece_kinds(&self) -> Result<(), RoutingPackageError> {
        let c = self.counts;
        let invalid = || RoutingPackageError::Invalid("piece kinds are invalid".into());
        let words = c.node_count.div_ceil(64);
        let tail = c.node_count % 64;
        let chain = self.slice::<u64>(Section::PieceChain);
        let tree = self.slice::<u64>(Section::PieceTree);
        if chain.len() != tree.len()
            || (!chain.is_empty() && Some(chain.len()) != c.column_count.checked_mul(words))
        {
            return Err(invalid());
        }
        for (index, (a, b)) in chain.iter().zip(tree).enumerate() {
            let last = index % words == words - 1;
            if a & b != 0 || (last && tail != 0 && (a | b) >> tail != 0) {
                return Err(invalid());
            }
        }
        Ok(())
    }

    /// The format version in the file's header.
    pub fn format_version(&self) -> u32 {
        self.format_version
    }

    pub fn layout(&self) -> PackageLayout {
        self.layout
    }

    /// A section's (offset, length) in the file.
    pub fn section_range(&self, section: Section) -> (usize, usize) {
        self.sections[section as usize]
    }

    pub fn counts(&self) -> PackageCounts {
        self.counts
    }

    /// The package's dataset hash, in hex: what overlay sets, results and
    /// bundle indexes bind to. In format 3 the identity hash, over the header
    /// and the sections flagged IDENTITY (adding or moving any other section
    /// keeps it); in format 2 the root checksum ([`Self::content_hash`]).
    pub fn dataset_hash(&self) -> &str {
        &self.dataset_hash
    }

    /// The package's root checksum, in hex: covers every byte of the file.
    pub fn content_hash(&self) -> &str {
        &self.content_hash
    }

    /// The format a writer would write this package in.
    pub fn package_format(&self) -> PackageFormat {
        PackageFormat::from_version(self.format_version).unwrap_or_default()
    }

    /// The scalars queries read beyond the sections.
    pub fn params(&self) -> &PackageParams {
        &self.params
    }

    /// The format-3 sections this build doesn't know, which it skipped.
    pub fn unknown_sections(&self) -> &[UnknownSection] {
        &self.unknown_sections
    }

    /// Whether the package holds `section` (in format 3 retired sections and
    /// a conditional section its header leaves out are absent).
    pub fn has_section(&self, section: Section) -> bool {
        section.is_written(self.package_format(), self.layout)
    }

    pub fn metadata(&self) -> &RoutingPackageMetadata {
        &self.metadata
    }

    pub fn graph_id(&self) -> &str {
        &self.metadata.graph_id
    }

    pub fn supports_profile(&self, profile: MobilityProfile) -> bool {
        self.profile_columns[profile_slot(profile)] != NO_COLUMN
    }

    pub fn byte_len(&self) -> usize {
        self.storage.bytes().len()
    }

    /// The name and size of every known section the package holds, for size
    /// reports.
    pub fn section_sizes(&self) -> Vec<(&'static str, usize)> {
        SECTIONS
            .iter()
            .filter(|section| self.has_section(**section))
            .map(|section| (section.name(), self.sections[*section as usize].1))
            .collect()
    }

    /// Ready to route: nodes, edges, and some ground-mode edge.
    pub fn is_ready(&self) -> bool {
        self.counts.node_count > 0
            && self.counts.edge_count > 0
            && self.metadata.mode_mask_union & crate::MobilityMode::ALL_GROUND != 0
    }

    pub fn has_route_edges_for_profile(&self, profile: MobilityProfile) -> bool {
        self.metadata.mode_mask_union & profile.mobility_mode_mask() != 0
    }

    pub fn view(&self) -> Sections<'_> {
        Sections {
            first_out: self.slice(Section::NodeFirstOut),
            node_xyz_fixed: self.slice(Section::NodeXyzFixed),
            node_lat_e7: self.slice(Section::NodeLatE7),
            node_lng_e7: self.slice(Section::NodeLngE7),
            node_lat_lng: self.slice(Section::NodeLatLng),
            node_lat_lng_nodes: self.slice(Section::NodeLatLngNodes),
            edge_target: self.slice(Section::EdgeTarget),
            edge_id_delta: self.slice(Section::EdgeId),
            edge_ids_implied: self.format_version >= 3,
            built_edge_id_lookup: &self.edge_id_lookup,
            edge_id_base: self.layout.edge_id_base,
            edge_length: self.slice(Section::EdgeLength),
            edge_shared_info: self.slice(Section::EdgeSharedInfo),
            heading_start: self.slice(Section::EdgeHeadingStart),
            lone_heading_end: self.slice(Section::LoneHeadingEnd),
            cch_rank: self.slice(Section::CchRank),
            edge_flags: self.slice(Section::EdgeFlags),
            road_rank: self.slice(Section::EdgeRoadRank),
            edge_junction: self.slice(Section::EdgeJunction),
            edge_cost: self.slice(Section::EdgeCost),
            edge_duration: self.slice(Section::EdgeDuration),
            si_way: WayIds {
                ids: self.slice(Section::SharedInfoWay),
                wide_shared: self.slice(Section::SharedInfoWayWideShared),
                wide: self.slice(Section::SharedInfoWayWide),
            },
            si_name: self.class_tokens(Section::SharedInfoName),
            si_ref: self.class_tokens(Section::SharedInfoRef),
            si_route_refs: RouteRefs {
                start: self.slice(Section::SharedInfoRouteRefStart),
                pool: self.slice(Section::SharedInfoRouteRefPool),
                class: self.slice(Section::SharedAttrClass),
            },
            simple_from: self.slice(Section::SimpleFrom),
            simple_to: self.slice(Section::SimpleTo),
            simple_mask: self.slice(Section::SimpleMask),
            complex_to: self.slice(Section::ComplexTo),
            complex_mask: self.slice(Section::ComplexMask),
            complex_prefix_start: self.slice(Section::ComplexPrefixStart),
            complex_prefix_pool: self.slice(Section::ComplexPrefixPool),
            edge_twin: self.slice(Section::EdgeTwin),
            lone_in_present: self.slice(Section::LoneInPresent),
            lone_in_edge: self.slice(Section::LoneInEdge),
            edge_profile_access: self.slice(Section::EdgeProfileAccess),
            shape_start: self.slice(Section::ShapeStart),
            shape_bytes: self.slice(Section::ShapeBytes),
            exact_shape_shared: self.slice(Section::ExactShapeShared),
            exact_shape_start: self.slice(Section::ExactShapeStart),
            exact_shape_points: self.slice(Section::ExactShapePoints),
            si_cost_flags: self.slice(Section::SharedInfoCostFlags),
            si_cost_classes: self.slice(Section::SharedInfoCostClasses),
            si_speeds: self.slice(Section::SharedInfoSpeeds),
            si_cost_tuple: self.slice(Section::SharedInfoCostTuple),
            si_legal_reason: self.slice(Section::SharedInfoLegalReason),
            barrier_node: self.slice(Section::BarrierNode),
            barrier_class: self.slice(Section::BarrierClass),
            barrier_access: self.slice(Section::BarrierAccess),
            piece_kinds: PieceKinds {
                chain: self.slice(Section::PieceChain),
                tree: self.slice(Section::PieceTree),
                words: self.counts.node_count.div_ceil(64),
            },
            sar: self.metadata.sar,
            edge_inferred_speed: self.slice(Section::EdgeInferredSpeed),
            conditional_edges: self.slice(Section::ConditionalEdges),
            conditional_shared: self.slice(Section::ConditionalShared),
            conditional_start: self.slice(Section::ConditionalStart),
            conditional_pool: self.slice(Section::ConditionalPool),
            string_start: self.slice(Section::StringStart),
            string_bytes: self.slice(Section::StringBytes),
            node_count: self.counts.node_count,
            edge_count: self.counts.edge_count,
        }
    }

    fn class_tokens(&self, section: Section) -> ClassTokens<'_> {
        ClassTokens {
            tokens: self.slice(section),
            class: self.slice(Section::SharedAttrClass),
        }
    }

    pub fn column_for(&self, profile: MobilityProfile) -> Option<usize> {
        let column = self.profile_columns[profile_slot(profile)];
        (column != NO_COLUMN).then_some(usize::from(column))
    }

    pub fn heuristic_cost_per_m(&self, profile: MobilityProfile) -> f64 {
        self.heuristic_cost_per_m[profile_slot(profile)]
    }
}

pub struct ParsedHeader {
    version: u32,
    counts: PackageCounts,
    layout: PackageLayout,
    profile_columns: [u8; PROFILE_SLOTS],
    heuristic_cost_per_m: [f64; PROFILE_SLOTS],
    /// Each known section's (offset, length); (0, 0) when absent.
    sections: [(usize, usize); SECTION_COUNT],
    /// Every table entry, in table order.
    entries: Vec<TableEntry>,
    /// The section table's (offset, length).
    table: (usize, usize),
    unknown_sections: Vec<UnknownSection>,
    root_checksum_hex: String,
    identity_hash_hex: String,
}

/// One section table entry, as read.
struct TableEntry {
    id: u32,
    /// The known section of that id.
    section: Option<Section>,
    offset: usize,
    len: usize,
    hash: [u8; 32],
}

/// Parses and checks the header and section table of a `file_len`-byte
/// package whose first bytes are `header`; `read_table(offset, len)` returns
/// the section table's bytes.
pub fn parse_header(
    header: &[u8],
    file_len: usize,
    read_table: impl FnOnce(usize, usize) -> Result<Vec<u8>, RoutingPackageError>,
) -> Result<ParsedHeader, RoutingPackageError> {
    if header.len() < HEADER_FIXED_BYTES || &header[0..8] != MAGIC {
        return Err(RoutingPackageError::Invalid(
            "not a routing package (missing header magic)".into(),
        ));
    }
    let version = u32::from_le_bytes(
        header[HEADER_VERSION..HEADER_VERSION + 4]
            .try_into()
            .unwrap(),
    );
    let readable = MIN_READ_FORMAT_VERSION..=MAX_READ_FORMAT_VERSION;
    if version < MIN_READ_FORMAT_VERSION {
        return Err(RoutingPackageError::Outdated(format!(
            "routing package format {version}, this build reads formats {readable:?}"
        )));
    }
    if version > MAX_READ_FORMAT_VERSION {
        return Err(RoutingPackageError::TooNew(format!(
            "routing package format {version}, this build reads formats {readable:?}"
        )));
    }
    if version == 2 {
        parse_header_v2(header, file_len, read_table)
    } else {
        parse_header_v3(header, file_len, read_table)
    }
}

fn read_u32(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap())
}

fn read_u64(bytes: &[u8], at: usize) -> u64 {
    u64::from_le_bytes(bytes[at..at + 8].try_into().unwrap())
}

/// The root checksum: blake3 over the header with its checksum field (at
/// `checksum_at`) zeroed, then the table.
fn root_checksum(header: &[u8], checksum_at: usize, table: &[u8]) -> blake3::Hash {
    let mut zeroed = header[..HEADER_FIXED_BYTES].to_vec();
    zeroed[checksum_at..checksum_at + 32].fill(0);
    let mut root = blake3::Hasher::new();
    root.update(&zeroed);
    root.update(table);
    root.finalize()
}

/// The six counts at `at` (u64 each).
fn read_counts(header: &[u8], at: usize) -> Result<PackageCounts, RoutingPackageError> {
    // `as usize` would truncate on 32-bit targets.
    let count = |slot: usize| {
        usize::try_from(read_u64(header, at + slot * 8)).map_err(|_| {
            RoutingPackageError::Invalid("header count exceeds this target's address space".into())
        })
    };
    Ok(PackageCounts {
        node_count: count(0)?,
        edge_count: count(1)?,
        shared_info_count: count(2)?,
        simple_restriction_count: count(3)?,
        complex_restriction_count: count(4)?,
        column_count: count(5)?,
    })
}

fn read_profiles(
    header: &[u8],
    columns_at: usize,
    heuristic_at: usize,
) -> ([u8; PROFILE_SLOTS], [f64; PROFILE_SLOTS]) {
    let mut profile_columns = [NO_COLUMN; PROFILE_SLOTS];
    profile_columns.copy_from_slice(&header[columns_at..columns_at + PROFILE_SLOTS]);
    let mut heuristic_cost_per_m = [0.0; PROFILE_SLOTS];
    for (slot, value) in heuristic_cost_per_m.iter_mut().enumerate() {
        *value = f64::from_bits(read_u64(header, heuristic_at + slot * 8));
    }
    (profile_columns, heuristic_cost_per_m)
}

/// Format 2: a positional table of [`FORMAT2_SECTION_COUNT`] entries right
/// after the header, sections in table order.
fn parse_header_v2(
    header: &[u8],
    file_len: usize,
    read_table: impl FnOnce(usize, usize) -> Result<Vec<u8>, RoutingPackageError>,
) -> Result<ParsedHeader, RoutingPackageError> {
    let invalid = |message: &str| RoutingPackageError::Invalid(message.to_string());
    let listed = FORMAT2_SECTION_COUNT;
    if read_u32(header, HEADER_SECTION_COUNT) as usize != listed {
        return Err(invalid("unexpected section count"));
    }
    let table_len = listed * TABLE_ENTRY_BYTES;
    let table_end = HEADER_FIXED_BYTES + table_len;
    if file_len < table_end {
        return Err(invalid("truncated section table"));
    }
    let table = read_table(HEADER_FIXED_BYTES, table_len)?;
    let root = root_checksum(header, HEADER_ROOT_CHECKSUM, &table);
    if root.as_bytes()[..] != header[HEADER_ROOT_CHECKSUM..HEADER_ROOT_CHECKSUM + 32] {
        return Err(invalid(
            "header or section table does not match the root checksum",
        ));
    }
    let counts = read_counts(header, HEADER_COUNTS)?;
    if read_u64(header, HEADER_COUNTS + 6 * 8) != 0 {
        return Err(invalid("reserved header count is not zero"));
    }
    let layout = PackageLayout {
        node_e7_derived: read_u32(header, HEADER_LAYOUT_FLAGS) & LAYOUT_NODE_E7_DERIVED != 0,
        edge_id_base: read_u64(header, HEADER_EDGE_ID_BASE),
    };
    let (profile_columns, heuristic_cost_per_m) =
        read_profiles(header, HEADER_PROFILE_COLUMNS, HEADER_HEURISTIC);
    let mut sections = [(0usize, 0usize); SECTION_COUNT];
    let mut entries = Vec::with_capacity(listed);
    let mut previous_end = table_end;
    for (index, section) in sections.iter_mut().enumerate().take(listed) {
        let entry = &table[index * TABLE_ENTRY_BYTES..(index + 1) * TABLE_ENTRY_BYTES];
        let read = |at: usize| usize::try_from(read_u64(entry, at)).ok();
        let (Some(offset), Some(len)) = (read(0), read(8)) else {
            return Err(invalid("section out of bounds or misaligned"));
        };
        // Sections are laid out in table order, each after the previous one.
        if !offset.is_multiple_of(SECTION_ALIGN)
            || offset < previous_end
            || offset.checked_add(len).is_none_or(|end| end > file_len)
            || !len.is_multiple_of(SECTION_ELEMENT_BYTES[index])
        {
            return Err(invalid("section out of bounds or misaligned"));
        }
        previous_end = offset + len;
        *section = (offset, len);
        entries.push(TableEntry {
            id: index as u32,
            section: Section::from_index(index),
            offset,
            len,
            hash: entry[16..48].try_into().unwrap(),
        });
    }
    if previous_end.next_multiple_of(SECTION_ALIGN) != file_len {
        return Err(invalid("file length does not match the section table"));
    }
    let root_checksum_hex = root.to_hex().to_string();
    Ok(ParsedHeader {
        version: 2,
        counts,
        layout,
        profile_columns,
        heuristic_cost_per_m,
        sections,
        entries,
        table: (HEADER_FIXED_BYTES, table_len),
        unknown_sections: Vec::new(),
        identity_hash_hex: root_checksum_hex.clone(),
        root_checksum_hex,
    })
}

/// Format 3 (`docs/design/format-3.md`, section 2): a tagged table anywhere
/// after the header (its offset is in the header), entries sorted by (id,
/// param), sections in any order, packed with zero padding to 64 bytes.
fn parse_header_v3(
    header: &[u8],
    file_len: usize,
    read_table: impl FnOnce(usize, usize) -> Result<Vec<u8>, RoutingPackageError>,
) -> Result<ParsedHeader, RoutingPackageError> {
    let invalid = |message: String| RoutingPackageError::Invalid(message);
    let entry_count = read_u32(header, HEADER_SECTION_COUNT) as usize;
    if entry_count > MAX_TABLE_ENTRIES {
        return Err(invalid(format!(
            "the section table has {entry_count} entries, more than {MAX_TABLE_ENTRIES}"
        )));
    }
    let table_len = entry_count * TABLE_ENTRY_BYTES_V3;
    let table_offset = usize::try_from(read_u64(header, HEADER_V3_TABLE_OFFSET))
        .ok()
        .filter(|offset| {
            offset.is_multiple_of(SECTION_ALIGN)
                && *offset >= HEADER_FIXED_BYTES
                && offset
                    .checked_add(table_len)
                    .is_some_and(|end| end <= file_len)
        })
        .ok_or_else(|| invalid("section table out of bounds or misaligned".into()))?;
    let table = read_table(table_offset, table_len)?;
    let root = root_checksum(header, HEADER_V3_ROOT_CHECKSUM, &table);
    if root.as_bytes()[..] != header[HEADER_V3_ROOT_CHECKSUM..HEADER_V3_ROOT_CHECKSUM + 32] {
        return Err(invalid(
            "header or section table does not match the root checksum".into(),
        ));
    }
    let counts = read_counts(header, HEADER_V3_COUNTS)?;
    let flags = read_u64(header, HEADER_V3_LAYOUT_FLAGS);
    let unknown_flags = flags & !u64::from(LAYOUT_NODE_E7_DERIVED);
    if unknown_flags != 0 {
        return Err(RoutingPackageError::TooNew(format!(
            "the package needs layout flags {unknown_flags:#x}, which this build doesn't read"
        )));
    }
    let layout = PackageLayout {
        node_e7_derived: flags & u64::from(LAYOUT_NODE_E7_DERIVED) != 0,
        edge_id_base: read_u64(header, HEADER_V3_EDGE_ID_BASE),
    };
    if HEADER_V3_ZERO
        .iter()
        .any(|range| header[range.clone()].iter().any(|byte| *byte != 0))
    {
        return Err(invalid("reserved header bytes are not zero".into()));
    }
    let (profile_columns, heuristic_cost_per_m) =
        read_profiles(header, HEADER_V3_PROFILE_COLUMNS, HEADER_V3_HEURISTIC);

    let mut sections = [(0usize, 0usize); SECTION_COUNT];
    let mut present = [false; SECTION_COUNT];
    let mut entries = Vec::with_capacity(entry_count);
    let mut unknown_sections = Vec::new();
    let mut identity = blake3::Hasher::new();
    identity.update(IDENTITY_HASH_TAG);
    let mut identity_header = header[..HEADER_FIXED_BYTES].to_vec();
    // The entry count and table offset change when a non-identity section is
    // added or moved; the checksum covers those.
    identity_header[HEADER_SECTION_COUNT..HEADER_V3_COUNTS].fill(0);
    identity_header[HEADER_V3_ROOT_CHECKSUM..].fill(0);
    identity.update(&identity_header);
    let mut previous_key = None;
    for raw in table.as_chunks::<TABLE_ENTRY_BYTES_V3>().0 {
        let (id, param, flags, elem) = (
            read_u32(raw, 0),
            read_u32(raw, 4),
            read_u32(raw, 8),
            read_u32(raw, 12),
        );
        if previous_key.is_some_and(|key| key >= (id, param)) {
            return Err(invalid(
                "section table entries are not sorted by (id, param) or repeat one".into(),
            ));
        }
        previous_key = Some((id, param));
        let (Some(offset), Some(len)) = (
            usize::try_from(read_u64(raw, 16)).ok(),
            usize::try_from(read_u64(raw, 24)).ok(),
        ) else {
            return Err(invalid(format!("section {id} out of bounds or misaligned")));
        };
        if flags & !(SECTION_REQUIRED | SECTION_IDENTITY) != 0 {
            return Err(invalid(format!(
                "section {id} has unknown flags {flags:#x}"
            )));
        }
        if elem == 0
            || !offset.is_multiple_of(SECTION_ALIGN)
            || offset < HEADER_FIXED_BYTES
            || offset.checked_add(len).is_none_or(|end| end > file_len)
            || !len.is_multiple_of(elem as usize)
        {
            return Err(invalid(format!("section {id} out of bounds or misaligned")));
        }
        let section = Section::from_id(id);
        match section {
            Some(known) => {
                if known.format_3_presence() == Presence::Retired {
                    return Err(invalid(format!(
                        "section {id} ({}) is retired in format 3",
                        known.name()
                    )));
                }
                if elem as usize != known.element_bytes()
                    || flags != known.format_3_flags()
                    || param != 0
                {
                    return Err(invalid(format!(
                        "section {id} ({}) has the wrong element size, flags or parameter",
                        known.name()
                    )));
                }
                // (id, param) are distinct and param is 0: one entry per id.
                sections[known as usize] = (offset, len);
                present[known as usize] = true;
            }
            None if flags & SECTION_REQUIRED != 0 && id >= EXPERIMENTAL_SECTION_ID => {
                return Err(invalid(format!(
                    "experimental section {id:#x} is flagged REQUIRED"
                )));
            }
            None if flags & SECTION_REQUIRED != 0 => {
                return Err(RoutingPackageError::TooNew(format!(
                    "the package needs section {id}, which this build doesn't read"
                )));
            }
            None => unknown_sections.push(UnknownSection {
                id,
                param,
                flags,
                element_bytes: elem,
                byte_len: len as u64,
            }),
        }
        let hash: [u8; 32] = raw[32..64].try_into().unwrap();
        if flags & SECTION_IDENTITY != 0 {
            identity.update(&raw[0..16]);
            identity.update(&raw[24..32]);
            identity.update(&hash);
        }
        entries.push(TableEntry {
            id,
            section,
            offset,
            len,
            hash,
        });
    }
    // The header, the table and the sections, in file order: each starts
    // where the one before ends, rounded up to 64 bytes, so they don't
    // overlap and only padding lies between them; the file ends at the last
    // end rounded up.
    let mut regions = entries
        .iter()
        .map(|entry| (entry.offset, entry.len))
        .chain([(0, HEADER_FIXED_BYTES), (table_offset, table_len)])
        .collect::<Vec<_>>();
    regions.sort_unstable_by_key(|(offset, len)| (*offset, *offset + *len));
    let mut end = 0usize;
    for (offset, len) in regions {
        if offset != end.next_multiple_of(SECTION_ALIGN) {
            return Err(invalid(
                "sections overlap, or leave more than padding between them".into(),
            ));
        }
        end = offset + len;
    }
    if end.next_multiple_of(SECTION_ALIGN) != file_len {
        return Err(invalid(
            "file length does not match the section table".into(),
        ));
    }
    for section in SECTIONS {
        let expected = section.is_written(PackageFormat::V3, layout);
        if present[section as usize] != expected {
            return Err(invalid(format!(
                "section {} ({}) is {}",
                section.id(),
                section.name(),
                if expected {
                    "missing"
                } else {
                    "present against its rule"
                }
            )));
        }
    }
    Ok(ParsedHeader {
        version: 3,
        counts,
        layout,
        profile_columns,
        heuristic_cost_per_m,
        sections,
        entries,
        table: (table_offset, table_len),
        unknown_sections,
        root_checksum_hex: root.to_hex().to_string(),
        identity_hash_hex: identity.finalize().to_hex().to_string(),
    })
}

/// Borrowed, pre-sliced access to every section; built once per query.
#[derive(Clone, Copy)]
pub struct Sections<'g> {
    pub first_out: &'g [u32],
    /// Per node, its rank in the CCH elimination order; empty without one
    /// (format 1). Checked to be a permutation at open.
    pub cch_rank: &'g [u32],
    /// Per node its unit vector in fixed point ([`fixed_xyz`]); read through
    /// [`Self::xyz`].
    pub node_xyz_fixed: &'g [i32],
    /// Empty when E7 positions derive from `node_xyz_fixed`.
    pub node_lat_e7: &'g [i32],
    pub node_lng_e7: &'g [i32],
    /// Exact (lat, lng) pairs of `node_lat_lng_nodes`.
    pub node_lat_lng: &'g [f64],
    /// The sorted nodes that are not their E7 position.
    pub node_lat_lng_nodes: &'g [u32],
    pub edge_target: &'g [u32],
    /// Edge id - `edge_id_base` per edge (see [`Self::edge_id`]); empty
    /// when the ids are implied.
    pub edge_id_delta: &'g [u32],
    /// Edge `i` has id `edge_id_base + i` (format 3).
    pub edge_ids_implied: bool,
    /// The id lookup, built on first use (see [`Self::edge_id_lookup`]).
    pub built_edge_id_lookup: &'g OnceLock<Vec<u32>>,
    pub edge_id_base: u64,
    pub edge_length: &'g [f32],
    pub edge_shared_info: &'g [u32],
    pub heading_start: &'g [u16],
    pub edge_flags: &'g [u8],
    pub road_rank: &'g [u8],
    /// [`junction::encode`] per edge.
    pub edge_junction: &'g [u8],
    pub edge_cost: &'g [f32],
    pub edge_duration: &'g [f32],
    pub si_way: WayIds<'g>,
    /// Name and ref tokens of a shared info (`si_name[shared]`).
    pub si_name: ClassTokens<'g>,
    pub si_ref: ClassTokens<'g>,
    /// Route-ref tokens of a shared info (`si_route_refs.get(shared)`).
    pub si_route_refs: RouteRefs<'g>,
    pub simple_from: &'g [u32],
    pub simple_to: &'g [u32],
    pub simple_mask: &'g [u8],
    pub complex_to: &'g [u32],
    pub complex_mask: &'g [u8],
    pub complex_prefix_start: &'g [u32],
    pub complex_prefix_pool: &'g [u32],
    /// Per edge its twin or [`NO_TWIN`] (see [`Section::EdgeTwin`]).
    pub edge_twin: &'g [u32],
    /// `[node / 64]` bits at the targets of `lone_in_edge`.
    pub lone_in_present: &'g [u64],
    /// The edges without a twin, sorted by (target, edge).
    pub lone_in_edge: &'g [u32],
    /// Their end headings.
    pub lone_heading_end: &'g [u16],
    pub edge_profile_access: &'g [u8],
    /// Byte offsets of every [`SHAPE_START_STRIDE`]th shape in
    /// `shape_bytes`, then its length.
    pub shape_start: &'g [u32],
    pub shape_bytes: &'g [u8],
    pub exact_shape_shared: &'g [u32],
    pub exact_shape_start: &'g [u32],
    pub exact_shape_points: &'g [f64],
    /// Costing inputs per costing tuple: index them with
    /// [`Self::cost_tuple`].
    pub si_cost_flags: &'g [u8],
    pub si_cost_classes: &'g [u32],
    pub si_speeds: &'g [f32],
    /// Each shared info's costing tuple.
    pub si_cost_tuple: &'g [u16],
    /// Each shared info's [`crate::sar::LegalReason`] byte.
    pub si_legal_reason: &'g [u8],
    /// The sorted barrier nodes, their SAR classes and their access in
    /// normal routing.
    pub barrier_node: &'g [u32],
    pub barrier_class: &'g [u8],
    pub barrier_access: &'g [u16],
    /// Each stored column's marked chain and tree nodes.
    pub piece_kinds: PieceKinds<'g>,
    /// The package's SAR data.
    pub sar: crate::sar::SarPackageInfo,
    pub edge_inferred_speed: &'g [u16],
    pub conditional_edges: &'g [u32],
    pub conditional_shared: &'g [u32],
    pub conditional_start: &'g [u32],
    pub conditional_pool: &'g [u32],
    pub string_start: &'g [u32],
    pub string_bytes: &'g [u8],
    pub node_count: usize,
    pub edge_count: usize,
}

/// In-edges a node holds without allocating ([`InEdges`]).
const IN_EDGES_INLINE: usize = 8;

/// Collects a node's (in-edge, source) pairs, inline up to
/// [`IN_EDGES_INLINE`].
#[derive(Default)]
struct InEdgeBuffer {
    inline: [(u32, u32); IN_EDGES_INLINE],
    len: usize,
    spilled: Vec<(u32, u32)>,
}

impl InEdgeBuffer {
    #[inline]
    fn push(&mut self, item: (u32, u32)) {
        if self.len < IN_EDGES_INLINE {
            self.inline[self.len] = item;
            self.len += 1;
        } else {
            if self.spilled.is_empty() {
                self.spilled.extend_from_slice(&self.inline);
            }
            self.spilled.push(item);
        }
    }

    /// The pairs in ascending edge order (edges are distinct).
    #[inline]
    fn into_sorted(mut self) -> InEdges {
        if !self.spilled.is_empty() {
            self.spilled.sort_unstable();
            return InEdges::Spilled(self.spilled.into_iter());
        }
        // Insertion sort: a node has a few in-edges, mostly in order already.
        let items = &mut self.inline[..self.len];
        for at in 1..items.len() {
            let item = items[at];
            let mut to = at;
            while to > 0 && items[to - 1].0 > item.0 {
                items[to] = items[to - 1];
                to -= 1;
            }
            items[to] = item;
        }
        InEdges::Inline {
            items: self.inline,
            at: 0,
            len: self.len,
        }
    }
}

/// The edges into a node, each with its source, in ascending edge order
/// ([`Sections::in_edges`]).
pub enum InEdges {
    /// At most `IN_EDGES_INLINE` (8) in-edges.
    Inline {
        items: [(u32, u32); IN_EDGES_INLINE],
        at: usize,
        len: usize,
    },
    /// Derived, for a node with more than `IN_EDGES_INLINE` (8) in-edges.
    Spilled(std::vec::IntoIter<(u32, u32)>),
}

impl Iterator for InEdges {
    /// (edge, its source)
    type Item = (u32, u32);

    #[inline]
    fn next(&mut self) -> Option<(u32, u32)> {
        match self {
            InEdges::Inline { items, at, len } => {
                let item = *items[..*len].get(*at)?;
                *at += 1;
                Some(item)
            }
            InEdges::Spilled(items) => items.next(),
        }
    }
}

/// What a node is in a stored column's piece classification (see
/// [`PieceKinds`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PieceKind {
    /// A junction, an unmarked node of a piece, or no data.
    Other,
    /// A marked degree-2 node of the 2-core.
    Chain,
    /// A marked node off the 2-core (in a dead-end tree).
    Tree,
}

/// Per stored column, the nodes the route search may walk through without
/// queueing labels. On the undirected simple graph
/// of the column's usable arcs, peeling nodes of degree at most one leaves
/// the 2-core, whose nodes of degree three or more are junctions; the other
/// nodes form pieces (a dead-end tree off one junction, or a chain of
/// degree-2 core nodes with the trees hanging off it between one or two
/// junctions). A node of a piece with a junction neighbour is marked, as a
/// chain node (in the core) or a tree node, unless it or a node of the trees
/// below it is a turn restriction's via node or can turn round without a
/// U-turn (a usable self-loop, two segments or two same-direction edges to
/// one neighbour).
#[derive(Clone, Copy)]
pub struct PieceKinds<'g> {
    /// `[column][node / 64]`.
    chain: &'g [u64],
    tree: &'g [u64],
    words: usize,
}

impl<'g> PieceKinds<'g> {
    /// Builds kinds from `[column][node / 64]` bit words (tests, in-memory
    /// packages).
    pub fn new(chain: &'g [u64], tree: &'g [u64], node_count: usize) -> Self {
        Self {
            chain,
            tree,
            words: node_count.div_ceil(64),
        }
    }

    /// Whether the package classifies its columns.
    pub fn is_present(&self) -> bool {
        !self.chain.is_empty()
    }

    /// Column `column`'s bits, if the package has them.
    pub fn column(&self, column: usize) -> Option<ColumnPieceKinds<'g>> {
        let range = column * self.words..(column + 1) * self.words;
        Some(ColumnPieceKinds {
            chain: self.chain.get(range.clone())?,
            tree: self.tree.get(range)?,
        })
    }
}

/// One stored column's [`PieceKinds`].
#[derive(Clone, Copy)]
pub struct ColumnPieceKinds<'g> {
    chain: &'g [u64],
    tree: &'g [u64],
}

impl ColumnPieceKinds<'_> {
    #[inline]
    pub fn kind(&self, node: u32) -> PieceKind {
        let (word, bit) = (node as usize / 64, 1u64 << (node % 64));
        if self.chain[word] & bit != 0 {
            PieceKind::Chain
        } else if self.tree[word] & bit != 0 {
            PieceKind::Tree
        } else {
            PieceKind::Other
        }
    }
}

/// Whether chain node `node` of a column (`kinds`, arc costs `cost`) sees
/// at most two neighbours over the usable arcs, leaving out tree nodes, no
/// self-loop, and at most one arc each way to each (see
/// `PackageFile::check_chain_marks`).
fn chain_node_is_valid(
    view: &Sections<'_>,
    kinds: ColumnPieceKinds<'_>,
    cost: &[f32],
    node: u32,
) -> bool {
    let n = node as usize;
    let mut neighbours = [u32::MAX; 2];
    // Per direction (out, in), the neighbours an arc already reaches.
    let mut seen = [[false; 2]; 2];
    let mut visit = |direction: usize, edge: u32, other: u32| {
        if !cost[edge as usize].is_finite() || kinds.kind(other) == PieceKind::Tree {
            return true;
        }
        if other == node {
            return false;
        }
        let slot = match neighbours.iter().position(|seen| *seen == other) {
            Some(slot) => slot,
            None => match neighbours.iter().position(|seen| *seen == u32::MAX) {
                Some(slot) => {
                    neighbours[slot] = other;
                    slot
                }
                None => return false,
            },
        };
        !std::mem::replace(&mut seen[direction][slot], true)
    };
    // The in-edges are the out-edges' twins and the lone in-edges (see
    // `Sections::in_edges`), in any order.
    for edge in view.first_out[n]..view.first_out[n + 1] {
        let target = view.edge_target[edge as usize];
        if !visit(0, edge, target) {
            return false;
        }
        let twin = view.edge_twin[edge as usize];
        if twin != NO_TWIN && !visit(1, twin, target) {
            return false;
        }
    }
    if view.lone_in_present[n / 64] & (1 << (n % 64)) != 0 {
        let target = |edge: &u32| view.edge_target[*edge as usize];
        let start = view
            .lone_in_edge
            .partition_point(|edge| target(edge) < node);
        for edge in &view.lone_in_edge[start..] {
            if target(edge) != node {
                break;
            }
            if !visit(1, *edge, view.edge_source(*edge as usize)) {
                return false;
            }
        }
    }
    true
}

/// A token per shared info, read as `tokens[shared]`: stored per attribute
/// class (a shared info's name and ref are part of its class's key, so the
/// class determines them).
#[derive(Clone, Copy)]
pub struct ClassTokens<'g> {
    tokens: &'g [u32],
    /// The attribute class per shared info.
    class: &'g [u32],
}

impl std::ops::Index<usize> for ClassTokens<'_> {
    type Output = u32;

    #[inline]
    fn index(&self, shared: usize) -> &u32 {
        &self.tokens[self.class[shared] as usize]
    }
}

/// Sorted route-ref tokens per shared info, stored per attribute class (the
/// class key holds the route refs).
#[derive(Clone, Copy)]
pub struct RouteRefs<'g> {
    /// Offsets into `pool`, one per class + 1.
    start: &'g [u32],
    pool: &'g [u32],
    /// The attribute class per shared info.
    class: &'g [u32],
}

impl<'g> RouteRefs<'g> {
    /// Shared info `shared`'s route-ref tokens, sorted and distinct.
    #[inline]
    pub fn get(&self, shared: usize) -> &'g [u32] {
        let row = self.class[shared] as usize;
        let (start, end) = (self.start[row] as usize, self.start[row + 1] as usize);
        &self.pool[start..end]
    }
}

/// Way ids per shared info: `u32`, with [`WAY_ID_WIDE`] marking the ids
/// kept in a sorted exception list (synthetic negative ids, and ids from
/// `u32::MAX` up).
#[derive(Clone, Copy)]
pub struct WayIds<'g> {
    pub ids: &'g [u32],
    pub wide_shared: &'g [u32],
    pub wide: &'g [i64],
}

impl WayIds<'_> {
    /// The OSM way id of shared info `shared` (0: none).
    #[inline]
    pub fn get(&self, shared: usize) -> i64 {
        match self.ids[shared] {
            WAY_ID_WIDE => self
                .wide_shared
                .binary_search(&(shared as u32))
                .map_or(0, |at| self.wide[at]),
            id => i64::from(id),
        }
    }

    /// Whether two shared infos belong to the same (known) way.
    #[inline]
    pub fn same_way(&self, left: usize, right: usize) -> bool {
        let (a, b) = (self.ids[left], self.ids[right]);
        if a != WAY_ID_WIDE && b != WAY_ID_WIDE {
            return a != 0 && a == b;
        }
        let way = self.get(left);
        way != 0 && way == self.get(right)
    }

    pub fn len(&self) -> usize {
        self.ids.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// The largest span of edge ids (largest id - smallest + 1) a package may
/// have for `edge_count` edges: the lookup from ids to edges is that long.
pub fn edge_id_span_limit(edge_count: usize) -> usize {
    edge_count
        .saturating_mul(4)
        .max(1 << 20)
        .min(u32::MAX as usize)
}

/// Mean Earth radius used for chord distances; any constant works as long as
/// the heuristic factor was derived with the same metric.
pub const EARTH_RADIUS_M: f64 = 6_371_008.8;

/// The unit vector of a position in degrees. Deterministic (see
/// [`crate::math`]): the compiler stores it in fixed point as
/// `NodeXyzFixed` ([`fixed_xyz`]).
pub fn unit_xyz(lat_deg: f64, lng_deg: f64) -> [f64; 3] {
    let (lat, lng) = (lat_deg.to_radians(), lng_deg.to_radians());
    let cos_lat = math::cos(lat);
    [
        cos_lat * math::cos(lng),
        cos_lat * math::sin(lng),
        math::sin(lat),
    ]
}

/// Straight 3-D chord between two unit vectors, in metres. Never exceeds the
/// great-circle distance, so it is a valid lower bound on any road path.
#[inline]
pub fn chord_m(a: [f64; 3], b: [f64; 3]) -> f64 {
    let (dx, dy, dz) = (a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    (dx * dx + dy * dy + dz * dz).sqrt() * EARTH_RADIUS_M
}

impl<'g> Sections<'g> {
    /// The edges into `node` with their sources, in ascending edge order:
    /// the twins of the node's out-edges, whose sources are those out-edges'
    /// targets, and the node's edges without a twin (see
    /// [`Section::EdgeTwin`]).
    #[inline]
    pub fn in_edges(&self, node: usize) -> InEdges {
        let mut items = InEdgeBuffer::default();
        for out in self.first_out[node]..self.first_out[node + 1] {
            let twin = self.edge_twin[out as usize];
            if twin != NO_TWIN {
                items.push((twin, self.edge_target[out as usize]));
            }
        }
        if self.lone_in_present[node / 64] & (1 << (node % 64)) != 0 {
            let target = |edge: &u32| self.edge_target[*edge as usize] as usize;
            let start = self
                .lone_in_edge
                .partition_point(|edge| target(edge) < node);
            for edge in &self.lone_in_edge[start..] {
                if target(edge) != node {
                    break;
                }
                items.push((*edge, self.edge_source(*edge as usize)));
            }
        }
        items.into_sorted()
    }

    /// The node `edge` leaves: its twin's target, or for an edge without a
    /// twin the node whose out-edges hold it (a binary search of
    /// `first_out`).
    #[inline]
    pub fn edge_source(&self, edge: usize) -> u32 {
        match self.edge_twin[edge] {
            NO_TWIN => self.lone_edge_source(edge),
            twin => self.edge_target[twin as usize],
        }
    }

    /// The heading in which `edge` ends ([`NO_HEADING`] for none): its
    /// twin's start heading reversed ([`reverse_heading`]; the compiler pairs
    /// edges only where that holds), or for an edge without a twin from
    /// `LoneHeadingEnd`.
    #[inline]
    pub fn heading_end(&self, edge: usize) -> u16 {
        match self.edge_twin[edge] {
            NO_TWIN => self.lone_heading_end(edge),
            twin => reverse_heading(self.heading_start[twin as usize]),
        }
    }

    /// `edge`'s row of `LoneHeadingEnd`, found by (target, edge).
    #[cold]
    fn lone_heading_end(&self, edge: usize) -> u16 {
        let key = (self.edge_target[edge], edge as u32);
        let at = self
            .lone_in_edge
            .partition_point(|lone| (self.edge_target[*lone as usize], *lone) < key);
        self.lone_heading_end[at]
    }

    /// The node whose out-edges hold `edge`: the last with its first
    /// out-edge at or before it.
    #[cold]
    fn lone_edge_source(&self, edge: usize) -> u32 {
        (self
            .first_out
            .partition_point(|first| *first as usize <= edge)
            - 1) as u32
    }

    /// The edge's [`junction`] byte.
    #[inline]
    pub fn junction(&self, edge: usize) -> u8 {
        self.edge_junction[edge]
    }

    /// The [`crate::sar::LegalReason`] of shared info `shared`.
    #[inline]
    pub fn legal_reason(&self, shared: usize) -> crate::sar::LegalReason {
        crate::sar::LegalReason(self.si_legal_reason[shared])
    }

    /// The barrier class of `node`, when it has a barrier that affects
    /// vehicles.
    #[inline]
    pub fn barrier(&self, node: u32) -> Option<crate::sar::SarBarrierClass> {
        let at = self.barrier_node.binary_search(&node).ok()?;
        crate::sar::SarBarrierClass::from_code(self.barrier_class[at])
    }

    /// The SAR class and normal-routing access of `node`, when it is a
    /// barrier node.
    #[inline]
    pub fn barrier_at(
        &self,
        node: u32,
    ) -> Option<(
        Option<crate::sar::SarBarrierClass>,
        crate::barrier::BarrierAccess,
    )> {
        let at = self.barrier_node.binary_search(&node).ok()?;
        Some((
            crate::sar::SarBarrierClass::from_code(self.barrier_class[at]),
            crate::barrier::BarrierAccess(self.barrier_access[at]),
        ))
    }

    /// The complex-restriction automata of the profile with `profile_bit`,
    /// built on first use.
    #[inline]
    pub fn xyz(&self, node: u32) -> [f64; 3] {
        let at = node as usize * 3;
        let fixed = &self.node_xyz_fixed[at..at + 3];
        xyz_from_fixed([fixed[0], fixed[1], fixed[2]])
    }

    /// How far, in metres of chord, [`Self::xyz`] may lie from the node's
    /// exact unit vector ([`FIXED_XYZ_MARGIN_M`]). A straight-line bound to
    /// an exact point (a snapped location) subtracts it to stay admissible.
    #[inline]
    pub fn xyz_margin_m(&self) -> f64 {
        FIXED_XYZ_MARGIN_M
    }

    /// The id of compact edge `edge`.
    #[inline]
    pub fn edge_id(&self, edge: usize) -> u64 {
        if self.edge_ids_implied {
            return self.edge_id_base + edge as u64;
        }
        self.edge_id_base + u64::from(self.edge_id_delta[edge])
    }

    /// `hint` when it is the compact edge with `edge_id`, else the compact
    /// edge with `edge_id` through the lookup ([`Self::edge_for_id`]). Callers
    /// that carry compact edges pass them as hints and never build the lookup.
    #[inline]
    pub fn edge_for_id_hinted(&self, hint: Option<u32>, edge_id: u64) -> Option<u32> {
        if self.edge_ids_implied {
            return self.edge_for_id(edge_id);
        }
        hint.filter(|edge| {
            self.edge_id_delta
                .get(*edge as usize)
                .is_some_and(|delta| self.edge_id_base + u64::from(*delta) == edge_id)
        })
        .or_else(|| self.edge_for_id(edge_id))
    }

    /// The compact edge with `edge_id`.
    #[inline]
    pub fn edge_for_id(&self, edge_id: u64) -> Option<u32> {
        let slot = edge_id.checked_sub(self.edge_id_base)?;
        if self.edge_ids_implied {
            return u32::try_from(slot)
                .ok()
                .filter(|edge| (*edge as usize) < self.edge_count);
        }
        self.edge_id_lookup()
            .get(usize::try_from(slot).ok()?)
            .copied()
            .filter(|edge| *edge != u32::MAX)
    }

    /// The compact edge per (edge id - base), `u32::MAX` for none: built
    /// from the edge ids on first use (one pass over them, under a lock that
    /// other threads wait on). Empty when the ids are implied (and so is the
    /// lookup).
    pub fn edge_id_lookup(&self) -> &'g [u32] {
        if self.edge_id_delta.is_empty() {
            return &[];
        }
        self.built_edge_id_lookup.get_or_init(|| {
            // Open checked that the ids are distinct and their span bounded.
            let span = self
                .edge_id_delta
                .iter()
                .max()
                .map_or(0, |max| *max as usize + 1);
            let mut lookup = vec![u32::MAX; span];
            for (edge, slot) in self.edge_id_delta.iter().enumerate() {
                lookup[*slot as usize] = edge as u32;
            }
            lookup
        })
    }

    /// Shared info `shared`'s row of the costing inputs (`si_cost_flags`,
    /// `si_cost_classes`, `si_speeds`).
    #[inline]
    pub fn cost_tuple(&self, shared: usize) -> usize {
        usize::from(self.si_cost_tuple[shared])
    }

    /// The node's position in E7 (rounded when it is not an exact E7 value;
    /// see [`Self::node_lat_lng`]).
    #[inline]
    pub fn node_e7(&self, node: u32) -> (i32, i32) {
        match (
            self.node_lat_e7.get(node as usize),
            self.node_lng_e7.get(node as usize),
        ) {
            (Some(lat), Some(lng)) => (*lat, *lng),
            _ => {
                // A node whose exact position is not an E7 value rounds it;
                // the others round-trip through the fixed-point vector.
                if let Ok(at) = self.node_lat_lng_nodes.binary_search(&node)
                    && let Some(pair) = self.node_lat_lng.get(at * 2..at * 2 + 2)
                {
                    return (to_e7(pair[0]), to_e7(pair[1]));
                }
                let at = node as usize * 3;
                let fixed = &self.node_xyz_fixed[at..at + 3];
                e7_from_fixed_xyz([fixed[0], fixed[1], fixed[2]])
            }
        }
    }

    /// The node's exact position in degrees.
    #[inline]
    pub fn node_lat_lng(&self, node: u32) -> (f64, f64) {
        let at = if self.node_lat_lng_nodes.is_empty() {
            None
        } else {
            self.node_lat_lng_nodes.binary_search(&node).ok()
        };
        match at.and_then(|at| self.node_lat_lng.get(at * 2..at * 2 + 2)) {
            Some(pair) => (pair[0], pair[1]),
            None => {
                let (lat, lng) = self.node_e7(node);
                (from_e7(lat), from_e7(lng))
            }
        }
    }

    /// String `id` of the package's string table.
    pub fn string(&self, id: u32) -> Option<&'g str> {
        let id = id as usize;
        let (start, end) = (
            *self.string_start.get(id)? as usize,
            *self.string_start.get(id + 1)? as usize,
        );
        std::str::from_utf8(self.string_bytes.get(start..end)?).ok()
    }

    /// The `*:conditional` tags of shared info `shared` (empty for most).
    pub fn conditional_restrictions(&self, shared: u32) -> Vec<String> {
        let Ok(at) = self.conditional_shared.binary_search(&shared) else {
            return Vec::new();
        };
        let (start, end) = (
            self.conditional_start[at] as usize,
            self.conditional_start[at + 1] as usize,
        );
        self.conditional_pool[start..end]
            .iter()
            .filter_map(|id| self.string(*id).map(str::to_string))
            .collect()
    }

    /// The exact points of shared info `shared`'s shape when it is not an
    /// exact E7 polyline (stored apart from `ShapeBytes`).
    fn exact_shape(&self, shared: u32) -> Option<&'g [f64]> {
        if self.exact_shape_shared.is_empty() {
            return None;
        }
        let at = self.exact_shape_shared.binary_search(&shared).ok()?;
        let (start, end) = (
            self.exact_shape_start[at] as usize,
            self.exact_shape_start[at + 1] as usize,
        );
        self.exact_shape_points.get(start * 2..end * 2)
    }

    /// Shared info `shared`'s shape in its stored direction, its first point
    /// at node `from` and its last at `to` (the edge's nodes, in shape order).
    pub fn shared_shape(&self, shared: u32, from: u32, to: u32, out: &mut Vec<GeoPoint>) {
        out.clear();
        if let Some(points) = self.exact_shape(shared) {
            out.extend(
                points
                    .as_chunks::<2>()
                    .0
                    .iter()
                    .map(|pair| GeoPoint::new(pair[0], pair[1])),
            );
            return;
        }
        let shared = shared as usize;
        let block = shared / SHAPE_START_STRIDE;
        let (Some(start), Some(end)) =
            (self.shape_start.get(block), self.shape_start.get(block + 1))
        else {
            return;
        };
        let Some(bytes) = self.shape_bytes.get(*start as usize..*end as usize) else {
            return;
        };
        // The shapes before it in its block.
        let mut at = 0;
        for _ in 0..shared % SHAPE_START_STRIDE {
            match skip_shape(bytes, at) {
                Some(next) => at = next,
                None => return,
            }
        }
        let decoded = skip_shape(bytes, at).and_then(|next| {
            decode_shape(&bytes[at..next], self.node_e7(from), self.node_e7(to), out)
        });
        if decoded.is_none() {
            out.clear();
        }
    }

    /// The edge's polyline in travel direction: the road graph geometry of
    /// its shared info (reversed for edges that run against it), as
    /// `directed_geometry` returns it. Clears `out` and leaves it empty when
    /// the stored shape is malformed.
    pub fn edge_shape(&self, edge: u32, out: &mut Vec<GeoPoint>) {
        let index = edge as usize;
        let shared = self.edge_shared_info[index];
        let reversed = self.edge_flags[index] & EDGE_FLAG_SHAPE_REVERSED != 0;
        let (from, to) = if reversed {
            (self.edge_target[index], self.edge_source(index))
        } else {
            (self.edge_source(index), self.edge_target[index])
        };
        self.shared_shape(shared, from, to, out);
        if reversed {
            out.reverse();
        }
    }
}

/// Scale of [`fixed_xyz`]: a unit vector component `v` is stored as
/// `round(v * 2^31)`, clamped to `i32`.
pub const FIXED_XYZ_SCALE: f64 = 2_147_483_648.0;

/// A bound, in metres of chord, on the distance between a unit vector and
/// its fixed-point value decoded ([`xyz_from_fixed`]): each component is off
/// by at most 2^-32 (2^-31 where 1.0 is clamped), so the vector by at most
/// `sqrt(3) * 2^-31 * EARTH_RADIUS_M` = 5.2 mm. Rounded up.
pub const FIXED_XYZ_MARGIN_M: f64 = 0.01;

/// A unit vector in fixed point, as `NodeXyzFixed` stores it. Exact IEEE
/// operations only (a power-of-two scale, `round`), so every platform gives
/// the same values.
pub fn fixed_xyz(xyz: [f64; 3]) -> [i32; 3] {
    xyz.map(|v| {
        (v * FIXED_XYZ_SCALE)
            .round()
            .clamp(f64::from(i32::MIN), f64::from(i32::MAX)) as i32
    })
}

/// The unit vector a [`fixed_xyz`] value stands for (exact: a power-of-two
/// scale).
#[inline]
pub fn xyz_from_fixed(fixed: [i32; 3]) -> [f64; 3] {
    fixed.map(|v| f64::from(v) / FIXED_XYZ_SCALE)
}

/// E7 position of a [`fixed_xyz`] vector. The latitude comes from
/// `atan2(z, |(x, y)|)`, well conditioned at every latitude; the longitude
/// loses precision towards the poles, which the builder detects by
/// round-tripping every node whose position is an E7 value (others store
/// their exact position). Deterministic (see [`crate::math`]).
pub fn e7_from_fixed_xyz(fixed: [i32; 3]) -> (i32, i32) {
    let [x, y, z] = xyz_from_fixed(fixed);
    let lat = math::atan2(z, (x * x + y * y).sqrt()).to_degrees();
    let lng = math::atan2(y, x).to_degrees();
    (to_e7(lat), to_e7(lng))
}

/// E7 fixed point to degrees. Division (not `* 1e-7`) reproduces OSM
/// coordinates bit for bit.
#[inline]
pub fn from_e7(value: i32) -> f64 {
    f64::from(value) / 1e7
}

/// Degrees to E7 fixed point.
#[inline]
pub fn to_e7(value: f64) -> i32 {
    (value * 1e7).round() as i32
}

/// Whether `value` degrees round-trip exactly through E7.
#[inline]
pub fn is_exact_e7(value: f64) -> bool {
    let e7 = (value * 1e7).round();
    (-2e9..=2e9).contains(&e7) && (e7 / 1e7).to_bits() == value.to_bits()
}

pub fn put_varint(out: &mut Vec<u8>, mut value: u64) {
    while value >= 0x80 {
        out.push((value as u8) | 0x80);
        value >>= 7;
    }
    out.push(value as u8);
}

#[inline]
pub fn get_varint(bytes: &[u8], at: &mut usize) -> Option<u64> {
    let mut value = 0u64;
    for shift in (0..64).step_by(7) {
        let byte = *bytes.get(*at)?;
        *at += 1;
        value |= u64::from(byte & 0x7f) << shift;
        if byte & 0x80 == 0 {
            return Some(value);
        }
    }
    None
}

#[inline]
pub fn zigzag(value: i64) -> u64 {
    ((value << 1) ^ (value >> 63)) as u64
}

#[inline]
pub fn unzigzag(value: u64) -> i64 {
    ((value >> 1) as i64) ^ -((value & 1) as i64)
}

/// Appends one shape: a header varint `(coded_points << 1) | explicit`, then
/// zigzag lat/lng deltas. With `endpoints` (the E7 positions of the shape's
/// start and end nodes, which then equal its first and last points) only
/// the interior points are coded, as deltas from the start node; otherwise
/// every point is coded, starting from (0, 0).
pub fn encode_shape(
    out: &mut Vec<u8>,
    points: &[(i32, i32)],
    endpoints: Option<((i32, i32), (i32, i32))>,
) {
    let implicit = endpoints.filter(|(start, end)| {
        points.len() >= 2 && points[0] == *start && points[points.len() - 1] == *end
    });
    let (coded, mut previous) = match implicit {
        Some((start, _)) => (&points[1..points.len() - 1], start),
        None => (points, (0, 0)),
    };
    put_varint(
        out,
        ((coded.len() as u64) << 1) | u64::from(implicit.is_none()),
    );
    for point in coded {
        put_varint(out, zigzag(i64::from(point.0) - i64::from(previous.0)));
        put_varint(out, zigzag(i64::from(point.1) - i64::from(previous.1)));
        previous = *point;
    }
}

/// The end of the shape [`encode_shape`] wrote at `at` in `bytes`, without
/// decoding its points: its header, then two varints per coded point (a
/// varint ends at a byte below 0x80). `None` when `bytes` ends first.
#[inline]
pub fn skip_shape(bytes: &[u8], at: usize) -> Option<usize> {
    let mut at = at;
    let header = get_varint(bytes, &mut at)?;
    let mut varints = usize::try_from(header >> 1).ok()?.checked_mul(2)?;
    while varints > 0 {
        let byte = *bytes.get(at)?;
        at += 1;
        if byte < 0x80 {
            varints -= 1;
        }
    }
    Some(at)
}

pub fn decode_shape(
    bytes: &[u8],
    start: (i32, i32),
    end: (i32, i32),
    out: &mut Vec<GeoPoint>,
) -> Option<()> {
    let mut at = 0;
    let header = get_varint(bytes, &mut at)?;
    let explicit = header & 1 != 0;
    let count = usize::try_from(header >> 1).ok()?;
    // Every coded point takes at least two bytes.
    if count > bytes.len() / 2 {
        return None;
    }
    let point = |(lat, lng): (i32, i32)| GeoPoint::new(from_e7(lat), from_e7(lng));
    out.reserve(count + 2);
    let mut previous = if explicit { (0, 0) } else { start };
    if !explicit {
        out.push(point(start));
    }
    for _ in 0..count {
        let lat = i64::from(previous.0) + unzigzag(get_varint(bytes, &mut at)?);
        let lng = i64::from(previous.1) + unzigzag(get_varint(bytes, &mut at)?);
        previous = (i32::try_from(lat).ok()?, i32::try_from(lng).ok()?);
        out.push(point(previous));
    }
    if !explicit {
        out.push(point(end));
    }
    (at == bytes.len()).then_some(())
}

/// A polyline with its haversine segment lengths, for interpolating several
/// points along it. Interpolation matches the record geometry the reference
/// oracle (`crate::reference`) interpolates:
/// the same segment lengths summed in the same order, then linear
/// interpolation in degrees within the segment.
#[derive(Default)]
pub struct MeasuredPolyline {
    pub points: Vec<GeoPoint>,
    segments_m: Vec<f64>,
    total_m: f64,
}

impl MeasuredPolyline {
    /// Loads `edge`'s directed shape and measures it.
    pub fn load_edge(&mut self, view: &Sections<'_>, edge: u32) {
        view.edge_shape(edge, &mut self.points);
        self.segments_m.clear();
        self.segments_m.extend(
            self.points
                .windows(2)
                .map(|window| window[0].distance_to(window[1])),
        );
        self.total_m = self.segments_m.iter().sum::<f64>();
    }

    pub fn point_at_fraction(&self, fraction: f64) -> Option<GeoPoint> {
        let geometry = &self.points;
        if geometry.len() < 2 {
            return None;
        }
        let fraction = fraction.clamp(0.0, 1.0);
        if self.total_m <= f64::EPSILON {
            return geometry.first().copied();
        }
        let target_distance_m = self.total_m * fraction;
        let mut distance_before_m = 0.0;
        for (window, segment_distance_m) in geometry.windows(2).zip(&self.segments_m) {
            let segment_distance_m = *segment_distance_m;
            if segment_distance_m <= f64::EPSILON {
                continue;
            }
            if distance_before_m + segment_distance_m >= target_distance_m {
                let segment_fraction =
                    ((target_distance_m - distance_before_m) / segment_distance_m).clamp(0.0, 1.0);
                return Some(GeoPoint::new(
                    window[0].lat + (window[1].lat - window[0].lat) * segment_fraction,
                    window[0].lng + (window[1].lng - window[0].lng) * segment_fraction,
                ));
            }
            distance_before_m += segment_distance_m;
        }
        geometry.last().copied()
    }
}

/// A section's typed storage, viewable as little-endian bytes.
pub trait SectionBytes {
    fn bytes(&self) -> &[u8];
}

impl<T: Pod> SectionBytes for Vec<T> {
    fn bytes(&self) -> &[u8] {
        // SAFETY: `Pod` types have no padding and any value is a valid byte
        // sequence on the (little-endian) target.
        unsafe {
            std::slice::from_raw_parts(
                self.as_ptr().cast::<u8>(),
                std::mem::size_of_val(self.as_slice()),
            )
        }
    }
}

/// Where a section's bytes are until the package is written.
enum SectionData {
    /// The typed vector, moved in.
    Memory(Box<dyn SectionBytes + Send + Sync>),
    /// Written to the writer's spill file at `offset` when it was put, and
    /// hashed then.
    Spilled {
        offset: u64,
        len: usize,
        hash: [u8; 32],
    },
}

/// Lays sections out at aligned offsets. Sections keep their typed vectors
/// (moved in, not copied) until they are hashed and streamed to the
/// destination, or, for a writer made with [`PackageWriter::spilling`], go
/// to a spill file as they are put, so a build holds only the sections it
/// still reads.
///
/// A writer writes format 2 unless [`Self::with_format`] says otherwise.
/// Format 2 puts the table after the header and the sections in slot order;
/// format 3 puts the sections in id order after the header and the table
/// after them. A section the format doesn't hold is dropped when it is put.
pub struct PackageWriter {
    format: PackageFormat,
    header: Vec<u8>,
    /// Whether node E7 positions are derived (from the header's layout):
    /// format 3 then leaves `NodeLatE7` and `NodeLngE7` out.
    node_e7_derived: bool,
    sections: Vec<Option<SectionData>>,
    /// The spill file and its length so far.
    spill: Option<(File, u64)>,
    /// The first failed spill write, reported by [`Self::write_to`].
    spill_error: Option<std::io::Error>,
    /// Section hashes and the final header, computed once by `seal`.
    sealed: OnceLock<Sealed>,
}

/// A writer's final header, table and layout.
struct Sealed {
    header: Vec<u8>,
    table: Vec<u8>,
    /// The sections the file holds, in file order, with their offsets.
    order: Vec<(Section, usize)>,
    table_offset: usize,
    byte_len: usize,
}

impl PackageWriter {
    pub fn new(
        counts: PackageCounts,
        layout: PackageLayout,
        profile_columns: [u8; PROFILE_SLOTS],
        heuristic_cost_per_m: [f64; PROFILE_SLOTS],
    ) -> Self {
        let mut writer = Self::with_spill(None);
        writer.set_header(counts, layout, profile_columns, heuristic_cost_per_m);
        writer
    }

    /// A writer that writes each section to `spill` (an empty file, readable
    /// and writable) as it is put, and copies them from there in file order
    /// when the package is written. Call [`Self::set_header`] before
    /// sealing or writing it.
    pub fn spilling(spill: File) -> Self {
        Self::with_spill(Some(spill))
    }

    /// A writer that keeps its sections in memory, before its header is
    /// known: call [`Self::set_header`] before sealing or writing it.
    pub fn in_memory() -> Self {
        Self::with_spill(None)
    }

    /// The writer, writing `format`. Call it before putting any section.
    pub fn with_format(mut self, format: PackageFormat) -> Self {
        assert!(
            self.header.is_empty() && self.sections.iter().all(Option::is_none),
            "the package format is set before the header and the sections"
        );
        self.format = format;
        self
    }

    pub fn format(&self) -> PackageFormat {
        self.format
    }

    fn with_spill(spill: Option<File>) -> Self {
        Self {
            format: PackageFormat::V2,
            header: Vec::new(),
            node_e7_derived: false,
            sections: (0..SECTION_COUNT).map(|_| None).collect(),
            spill: spill.map(|file| (file, 0)),
            spill_error: None,
            sealed: OnceLock::new(),
        }
    }

    /// Sets the header, and in format 2 puts the [`RESERVED_ZERO_SECTIONS`]
    /// its counts size. [`Self::new`] does this.
    pub fn set_header(
        &mut self,
        counts: PackageCounts,
        layout: PackageLayout,
        profile_columns: [u8; PROFILE_SLOTS],
        heuristic_cost_per_m: [f64; PROFILE_SLOTS],
    ) {
        let (counts_at, flags_at, base_at, columns_at, heuristic_at) = match self.format {
            PackageFormat::V2 => (
                HEADER_COUNTS,
                HEADER_LAYOUT_FLAGS,
                HEADER_EDGE_ID_BASE,
                HEADER_PROFILE_COLUMNS,
                HEADER_HEURISTIC,
            ),
            PackageFormat::V3 => (
                HEADER_V3_COUNTS,
                HEADER_V3_LAYOUT_FLAGS,
                HEADER_V3_EDGE_ID_BASE,
                HEADER_V3_PROFILE_COLUMNS,
                HEADER_V3_HEURISTIC,
            ),
        };
        let mut header = vec![0u8; HEADER_FIXED_BYTES];
        header[0..8].copy_from_slice(MAGIC);
        header[HEADER_VERSION..HEADER_VERSION + 4]
            .copy_from_slice(&self.format.version().to_le_bytes());
        // Format 3's entry count and table offset are set when sealing.
        if self.format == PackageFormat::V2 {
            header[HEADER_SECTION_COUNT..HEADER_SECTION_COUNT + 4]
                .copy_from_slice(&(FORMAT2_SECTION_COUNT as u32).to_le_bytes());
        }
        // Format 2 has a seventh, reserved count (format 1's landmark
        // count), zero.
        let values = [
            counts.node_count,
            counts.edge_count,
            counts.shared_info_count,
            counts.simple_restriction_count,
            counts.complex_restriction_count,
            counts.column_count,
        ];
        for (index, value) in values.iter().enumerate() {
            let at = counts_at + index * 8;
            header[at..at + 8].copy_from_slice(&(*value as u64).to_le_bytes());
        }
        let flags = if layout.node_e7_derived {
            LAYOUT_NODE_E7_DERIVED
        } else {
            0
        };
        header[flags_at..flags_at + 4].copy_from_slice(&flags.to_le_bytes());
        header[base_at..base_at + 8].copy_from_slice(&layout.edge_id_base.to_le_bytes());
        header[columns_at..columns_at + PROFILE_SLOTS].copy_from_slice(&profile_columns);
        for (slot, value) in heuristic_cost_per_m.iter().enumerate() {
            let at = heuristic_at + slot * 8;
            header[at..at + 8].copy_from_slice(&value.to_le_bytes());
        }
        self.header = header;
        self.node_e7_derived = layout.node_e7_derived;
        if self.format == PackageFormat::V2 {
            let words = counts.column_count * counts.node_count.div_ceil(64);
            for section in RESERVED_ZERO_SECTIONS {
                match SECTION_ELEMENT_BYTES[section as usize] {
                    8 => self.put(section, vec![0u64; words]),
                    _ => self.put(section, vec![0u32; words]),
                }
            }
        }
    }

    /// Whether the package holds `section`: in the writer's format, and,
    /// for `NodeLatE7` and `NodeLngE7` in format 3, not derived.
    fn writes(&self, section: Section) -> bool {
        section.is_written(
            self.format,
            PackageLayout {
                node_e7_derived: self.node_e7_derived,
                edge_id_base: 0,
            },
        )
    }

    /// Puts a section. One the writer's format doesn't hold is dropped.
    pub fn put<T: Pod + Send + Sync>(&mut self, section: Section, data: Vec<T>) {
        debug_assert!(
            SECTION_ELEMENT_BYTES[section as usize].is_multiple_of(std::mem::size_of::<T>())
        );
        let held = match self.format {
            PackageFormat::V2 => section.in_format_2(),
            PackageFormat::V3 => section.format_3_presence() != Presence::Retired,
        };
        if !held {
            return;
        }
        let Some((file, end)) = &mut self.spill else {
            self.sections[section as usize] = Some(SectionData::Memory(Box::new(data)));
            return;
        };
        let bytes = data.bytes();
        let offset = *end;
        if self.spill_error.is_none() {
            use std::io::{Seek, SeekFrom};
            let written = file
                .seek(SeekFrom::Start(offset))
                .and_then(|_| file.write_all(bytes));
            if let Err(error) = written {
                self.spill_error = Some(error);
            }
        }
        *end += bytes.len() as u64;
        let hash = *hash_section(bytes).as_bytes();
        self.sections[section as usize] = Some(SectionData::Spilled {
            offset,
            len: bytes.len(),
            hash,
        });
    }

    pub fn put_metadata<Q: Serialize>(&mut self, metadata: &RoutingPackageMetadata<Q>) {
        let bytes = serde_json::to_vec(metadata).expect("metadata serializes");
        self.put(Section::Metadata, bytes);
    }

    /// The bytes of a section held in memory (empty if it was not put);
    /// `None` for a section in the spill file.
    pub fn section_bytes(&self, index: usize) -> Option<&[u8]> {
        match &self.sections[index] {
            None => Some(&[]),
            Some(SectionData::Memory(section)) => Some(section.bytes()),
            Some(SectionData::Spilled { .. }) => None,
        }
    }

    /// The byte length of a section (0 if it was not put).
    pub fn section_len(&self, index: usize) -> usize {
        match &self.sections[index] {
            None => 0,
            Some(SectionData::Memory(section)) => section.bytes().len(),
            Some(SectionData::Spilled { len, .. }) => *len,
        }
    }

    /// Where a format-2 package's first section starts.
    pub fn first_section_offset() -> usize {
        (HEADER_FIXED_BYTES + FORMAT2_SECTION_COUNT * TABLE_ENTRY_BYTES)
            .next_multiple_of(SECTION_ALIGN)
    }

    /// The sections the package holds, in file order (by id: format 2's slot
    /// order, then format 3's new ids), each with its offset; the table's
    /// offset; the file's length.
    fn layout(&self) -> (Vec<(Section, usize)>, usize, usize) {
        let mut sections = SECTIONS
            .into_iter()
            .filter(|section| self.writes(*section))
            .collect::<Vec<_>>();
        sections.sort_by_key(|section| section.id());
        let (table_len, mut offset) = match self.format {
            PackageFormat::V2 => (
                FORMAT2_SECTION_COUNT * TABLE_ENTRY_BYTES,
                Self::first_section_offset(),
            ),
            PackageFormat::V3 => (sections.len() * TABLE_ENTRY_BYTES_V3, HEADER_FIXED_BYTES),
        };
        let mut order = Vec::with_capacity(sections.len());
        for section in sections {
            order.push((section, offset));
            offset = (offset + self.section_len(section as usize)).next_multiple_of(SECTION_ALIGN);
        }
        match self.format {
            PackageFormat::V2 => (order, HEADER_FIXED_BYTES, offset),
            PackageFormat::V3 => (
                order,
                offset,
                (offset + table_len).next_multiple_of(SECTION_ALIGN),
            ),
        }
    }

    /// Total encoded size.
    pub fn byte_len(&self) -> usize {
        self.layout().2
    }

    /// The final header and section table (computed once: every section
    /// hashed, then the root checksum).
    pub fn seal(&self) -> (&[u8], &[u8]) {
        let sealed = self.sealed();
        (&sealed.header, &sealed.table)
    }

    /// Every section in memory hashed (in parallel; spilled ones were hashed
    /// when put), then the table, then the root checksum over the header and
    /// table.
    fn sealed(&self) -> &Sealed {
        assert!(!self.header.is_empty(), "the package header is not set");
        self.sealed.get_or_init(|| {
            use rayon::prelude::*;
            let (order, table_offset, byte_len) = self.layout();
            let hashes = order
                .par_iter()
                .map(|(section, _)| match &self.sections[*section as usize] {
                    Some(SectionData::Spilled { hash, .. }) => *hash,
                    _ => *hash_section(self.section_bytes(*section as usize).unwrap_or_default())
                        .as_bytes(),
                })
                .collect::<Vec<_>>();
            let mut header = self.header.clone();
            let mut table = Vec::new();
            let checksum_at = match self.format {
                PackageFormat::V2 => {
                    for ((section, offset), hash) in order.iter().zip(&hashes) {
                        table.extend_from_slice(&(*offset as u64).to_le_bytes());
                        table.extend_from_slice(
                            &(self.section_len(*section as usize) as u64).to_le_bytes(),
                        );
                        table.extend_from_slice(hash);
                    }
                    HEADER_ROOT_CHECKSUM
                }
                PackageFormat::V3 => {
                    for ((section, offset), hash) in order.iter().zip(&hashes) {
                        table.extend_from_slice(&section.id().to_le_bytes());
                        table.extend_from_slice(&0u32.to_le_bytes());
                        table.extend_from_slice(&section.format_3_flags().to_le_bytes());
                        table.extend_from_slice(&(section.element_bytes() as u32).to_le_bytes());
                        table.extend_from_slice(&(*offset as u64).to_le_bytes());
                        table.extend_from_slice(
                            &(self.section_len(*section as usize) as u64).to_le_bytes(),
                        );
                        table.extend_from_slice(hash);
                    }
                    header[HEADER_SECTION_COUNT..HEADER_SECTION_COUNT + 4]
                        .copy_from_slice(&(order.len() as u32).to_le_bytes());
                    header[HEADER_V3_TABLE_OFFSET..HEADER_V3_TABLE_OFFSET + 8]
                        .copy_from_slice(&(table_offset as u64).to_le_bytes());
                    HEADER_V3_ROOT_CHECKSUM
                }
            };
            let root = root_checksum(&header, checksum_at, &table);
            header[checksum_at..checksum_at + 32].copy_from_slice(root.as_bytes());
            Sealed {
                header,
                table,
                order,
                table_offset,
                byte_len,
            }
        })
    }

    /// Streams the encoded package to `out`.
    pub fn write_to(&self, out: &mut impl Write) -> std::io::Result<()> {
        if let Some(error) = &self.spill_error {
            return Err(std::io::Error::new(
                error.kind(),
                format!("writing a package section to the spill file failed: {error}"),
            ));
        }
        let sealed = self.sealed();
        fn emit(out: &mut impl Write, written: &mut usize, bytes: &[u8]) -> std::io::Result<()> {
            out.write_all(bytes)?;
            *written += bytes.len();
            Ok(())
        }
        fn pad_to(out: &mut impl Write, written: &mut usize, to: usize) -> std::io::Result<()> {
            let zeros = [0u8; SECTION_ALIGN];
            while *written < to {
                let count = (to - *written).min(SECTION_ALIGN);
                emit(out, written, &zeros[..count])?;
            }
            Ok(())
        }
        let mut written = 0usize;
        emit(out, &mut written, &sealed.header)?;
        if self.format == PackageFormat::V2 {
            emit(out, &mut written, &sealed.table)?;
        }
        for (section, offset) in &sealed.order {
            pad_to(out, &mut written, *offset)?;
            let index = *section as usize;
            match (&self.sections[index], &self.spill) {
                (Some(SectionData::Spilled { offset, len, .. }), Some((file, _))) => {
                    use std::io::{Seek, SeekFrom};
                    let mut file = file;
                    file.seek(SeekFrom::Start(*offset))?;
                    let copied = std::io::copy(&mut file.take(*len as u64), out)?;
                    if copied != *len as u64 {
                        return Err(std::io::Error::new(
                            std::io::ErrorKind::UnexpectedEof,
                            "the package spill file is shorter than its sections",
                        ));
                    }
                    written += *len;
                }
                _ => emit(
                    out,
                    &mut written,
                    self.section_bytes(index).unwrap_or_default(),
                )?,
            }
        }
        if self.format == PackageFormat::V3 {
            pad_to(out, &mut written, sealed.table_offset)?;
            emit(out, &mut written, &sealed.table)?;
        }
        pad_to(out, &mut written, sealed.byte_len)
    }

    /// Encodes into an owned buffer (in-memory engines and tests).
    pub fn finish(self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.byte_len());
        self.write_to(&mut out)
            .expect("writing to a Vec cannot fail");
        out
    }
}

/// A section's checksum: BLAKE3 over its bytes.
fn hash_section(bytes: &[u8]) -> blake3::Hash {
    blake3::hash(bytes)
}

/// The road class rank stored in `EdgeRoadRank` (higher is a bigger road).
pub fn road_type_rank(road_type: RoadType) -> u8 {
    match road_type {
        RoadType::Path => 1,
        RoadType::Track => 2,
        RoadType::Other => 3,
        RoadType::Residential => 4,
        RoadType::Tertiary => 5,
        RoadType::Secondary => 6,
        RoadType::Primary => 7,
        RoadType::Motorway => 8,
    }
}

/// Inverse of [`road_type_rank`].
pub fn road_type_from_rank(rank: u8) -> Option<RoadType> {
    Some(match rank {
        1 => RoadType::Path,
        2 => RoadType::Track,
        3 => RoadType::Other,
        4 => RoadType::Residential,
        5 => RoadType::Tertiary,
        6 => RoadType::Secondary,
        7 => RoadType::Primary,
        8 => RoadType::Motorway,
        _ => return None,
    })
}

#[cfg(test)]
mod shape_codec_tests {
    use super::*;

    fn decoded(bytes: &[u8], start: (i32, i32), end: (i32, i32)) -> Option<Vec<(i32, i32)>> {
        let mut out = Vec::new();
        decode_shape(bytes, start, end, &mut out)?;
        Some(out.iter().map(|p| (to_e7(p.lat), to_e7(p.lng))).collect())
    }

    #[test]
    fn skip_shape_finds_each_shape_end() {
        // Implicit and explicit shapes, with short and multi-byte deltas.
        let shapes: Vec<Vec<(i32, i32)>> = vec![
            vec![(10, 20), (11, 21)],
            vec![(10, 20), (500_000_000, -1_000_000), (11, 21)],
            vec![(0, 0)],
            vec![(1, 2), (3, 4), (-5_000, 6_000), (7, 8)],
        ];
        let mut bytes = Vec::new();
        let mut ends = Vec::new();
        for points in &shapes {
            let endpoints = (points.len() >= 2).then(|| (points[0], points[points.len() - 1]));
            encode_shape(&mut bytes, points, endpoints);
            ends.push(bytes.len());
        }
        let mut at = 0;
        for end in &ends {
            at = skip_shape(&bytes, at).unwrap();
            assert_eq!(at, *end);
        }
        assert_eq!(skip_shape(&bytes, at), None);
        assert_eq!(skip_shape(&bytes[..bytes.len() - 1], ends[2]), None);
    }

    #[test]
    fn shapes_round_trip_with_implicit_and_explicit_endpoints() {
        let points = [
            (500_647_000, 199_448_000),
            (500_647_123, 199_447_544),
            (500_640_000, 199_460_000),
            (-899_999_999, 1_799_999_999),
            (500_650_000, 199_450_000),
        ];
        let (start, end) = (points[0], points[4]);

        let mut implicit = Vec::new();
        encode_shape(&mut implicit, &points, Some((start, end)));
        assert_eq!(implicit[0] & 1, 0, "endpoints match the nodes");
        assert_eq!(decoded(&implicit, start, end).unwrap(), points);

        // Nodes elsewhere: the endpoints are stored and the nodes ignored.
        let mut explicit = Vec::new();
        encode_shape(&mut explicit, &points, Some(((1, 1), end)));
        assert_eq!(explicit[0] & 1, 1);
        assert_eq!(decoded(&explicit, (7, 7), (8, 8)).unwrap(), points);

        // A degenerate shape stays degenerate (no boundary point).
        let mut single = Vec::new();
        encode_shape(&mut single, &points[..1], Some((start, start)));
        assert_eq!(decoded(&single, start, start).unwrap(), &points[..1]);

        // Truncated or padded streams are rejected, never read past.
        assert!(decoded(&implicit[..implicit.len() - 1], start, end).is_none());
        let mut padded = implicit.clone();
        padded.push(0);
        assert!(decoded(&padded, start, end).is_none());
        assert!(decoded(&[0xff; 12], start, end).is_none());
    }

    #[test]
    fn e7_conversion_reproduces_osm_degrees() {
        for value in [
            0,
            1,
            -1,
            500_647_123,
            -899_999_999,
            1_799_999_999,
            199_448_001,
        ] {
            let degrees = f64::from(value) / 1e7;
            assert_eq!(to_e7(degrees), value);
            assert_eq!(from_e7(to_e7(degrees)), degrees);
            assert!(is_exact_e7(degrees));
        }
        assert!(!is_exact_e7(0.1 + 0.2));
    }

    /// Fixed-point unit vectors lie within `FIXED_XYZ_MARGIN_M` of the exact
    /// ones, and E7 positions round-trip through them up to 60° of latitude
    /// (a deterministic sample of E7 values, plus the axes and poles).
    #[test]
    fn fixed_point_vectors_are_close_and_round_trip_e7() {
        let mut state = 0x9e37_79b9_7f4a_7c15u64;
        let mut next = |range: i64| {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state % (2 * range as u64 + 1)) as i64 - range
        };
        let mut worst = 0.0f64;
        let mut points = vec![
            (0, 0),
            (900_000_000, 0),
            (-900_000_000, 0),
            (0, 1_800_000_000),
        ];
        points.extend((0..200_000).map(|_| (next(600_000_000) as i32, next(1_800_000_000) as i32)));
        for (index, (lat, lng)) in points.into_iter().enumerate() {
            let exact = unit_xyz(from_e7(lat), from_e7(lng));
            let fixed = fixed_xyz(exact);
            worst = worst.max(chord_m(exact, xyz_from_fixed(fixed)));
            if index >= 3 {
                // Not at the poles, where every longitude is the same point.
                assert_eq!(e7_from_fixed_xyz(fixed), (lat, lng), "{lat} {lng}");
            }
        }
        assert!(worst <= FIXED_XYZ_MARGIN_M / 1.5, "{worst}");
        assert_eq!(fixed_xyz([1.0, -1.0, 0.0]), [i32::MAX, i32::MIN, 0]);
    }
}

#[cfg(test)]
mod replace_tests {
    use super::*;

    fn names(dir: &Path) -> Vec<String> {
        let mut names: Vec<String> = std::fs::read_dir(dir)
            .unwrap()
            .map(|entry| entry.unwrap().file_name().into_string().unwrap())
            .collect();
        names.sort();
        names
    }

    /// The new package takes the path while the old one stays mapped and
    /// readable; once it is unmapped, the next replacement leaves no file
    /// behind. Through the rename on Linux and macOS, through the move aside
    /// on Windows.
    #[test]
    fn a_mapped_package_is_replaced_and_keeps_its_bytes() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("routing.mrt");
        let temp = dir.path().join("routing.mrt.tmp");
        std::fs::write(&path, b"old package").unwrap();
        let file = File::open(&path).unwrap();
        // SAFETY: the test changes the file only by renames.
        let map = unsafe { Mmap::map(&file) }.unwrap();
        drop(file);

        std::fs::write(&temp, b"new package").unwrap();
        PackageFile::move_into_place(&temp, &path).unwrap();
        assert_eq!(std::fs::read(&path).unwrap(), b"new package");
        assert_eq!(&map[..], b"old package");

        drop(map);
        std::fs::write(&temp, b"newer package").unwrap();
        PackageFile::move_into_place(&temp, &path).unwrap();
        assert_eq!(std::fs::read(&path).unwrap(), b"newer package");
        assert_eq!(names(dir.path()), ["routing.mrt"]);
    }

    /// The Windows fallback on its own, on any platform: the old file is
    /// moved aside and deleted, or left for the next call while it is open.
    #[test]
    fn moving_aside_replaces_the_package_and_cleans_up() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("routing.mrt");
        let temp = dir.path().join("routing.mrt.tmp");
        std::fs::write(&path, b"old package").unwrap();
        let file = File::open(&path).unwrap();
        // SAFETY: the test changes the file only by renames.
        let map = unsafe { Mmap::map(&file) }.unwrap();

        std::fs::write(&temp, b"new package").unwrap();
        move_into_place_aside(&temp, &path).unwrap();
        assert_eq!(std::fs::read(&path).unwrap(), b"new package");
        assert_eq!(&map[..], b"old package");
        assert!(!temp.exists());

        drop(map);
        drop(file);
        // Whatever the first call could not delete goes now.
        std::fs::write(&temp, b"newer package").unwrap();
        move_into_place_aside(&temp, &path).unwrap();
        remove_replaced(&path);
        assert_eq!(std::fs::read(&path).unwrap(), b"newer package");
        assert_eq!(names(dir.path()), ["routing.mrt"]);
    }

    /// Only names this module made for this path are deleted.
    #[test]
    fn cleanup_deletes_only_packages_moved_aside_from_the_path() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("routing.mrt");
        let kept = [
            "other.mrt.replaced-12-34-0",
            "routing.mrt.replaced-",
            "routing.mrt.replaced-notes",
            "routing.mrt.tmp",
            "routing.mrt",
        ];
        for name in kept.iter().chain(&["routing.mrt.replaced-12-34-0"]) {
            std::fs::write(dir.path().join(name), b"").unwrap();
        }
        remove_replaced(&path);
        let mut kept = kept.map(str::to_string).to_vec();
        kept.sort();
        assert_eq!(names(dir.path()), kept);
    }
}

//! The routing package (`routing.mrt`) as a file format, shared by the route
//! compiler that writes it and the engine that reads it:
//!
//! - the road vocabulary it stores (nodes, directed edges, the attributes they
//!   share, mobility profiles, and the enums and mode masks those use), which
//!   the road graph builder speaks too;
//! - [`format`](mod@format): the header, section table and sections, opening and
//!   verifying a file ([`format::PackageFile`]), the section view queries
//!   read, the shape codec and the writer;
//! - [`request`]: the request sections endpoint correlation and route legs
//!   read, and the codecs the compiler encodes them with.
//!
//! A package is one memory-mappable file holding the road network of an
//! OpenStreetMap extract, with per-profile edge costs, turn restrictions and
//! the node order of CCH potentials, in checksummed sections. Compiling one from OSM data is
//! the route compiler's job and answering routes is the engine's; this crate
//! opens, verifies, reads and writes the file.
//!
//! ```no_run
//! use wayfarer_package::format::{PackageFile, ROUTING_PACKAGE_FORMAT_VERSION};
//! use std::path::Path;
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let path = Path::new("routing.mrt");
//!
//! // Hashes every section against the section table. Reads the whole file,
//! // so do it once, when a package is installed.
//! let verification = PackageFile::verify(path)?;
//! println!("{} bytes, dataset hash {}", verification.byte_len, verification.dataset_hash);
//!
//! // Opening maps the file and checks the header and each section's bounds,
//! // without hashing the sections.
//! let package = PackageFile::open(path)?;
//! let counts = package.counts();
//! println!(
//!     "{}: format {} (this build writes {}), {} nodes, {} directed edges",
//!     package.graph_id(),
//!     package.format_version(),
//!     ROUTING_PACKAGE_FORMAT_VERSION,
//!     counts.node_count,
//!     counts.edge_count,
//! );
//! # Ok(())
//! # }
//! ```
//!
//! # Stability
//!
//! The layout is versioned by [`format::ROUTING_PACKAGE_FORMAT_VERSION`]. A
//! build opens the formats [`format::MIN_READ_FORMAT_VERSION`] through that
//! version: a layout change keeps reading the previous format for at least one
//! release, an older format is reported as `Outdated` and a newer
//! format as `TooNew`. A package also records the costing model its stored
//! costs were baked with, which the engine must match. The format is
//! little-endian only, and the [`request`], [`codec`] and [`raw`] modules are
//! doc-hidden and not part of the supported API.
//!
//! Format 3 (`docs/design/format-3.md`: a tagged section table, an identity
//! hash apart from the content hash, implied ids numbered by CCH rank,
//! `PackageParams`) is the format a 0.5 compile writes. This build still
//! opens format 2, and writes it when asked ([`format::PackageFormat::V2`])
//! for engines older than 0.5. The drafts of format 3 that development
//! builds wrote are refused as `Outdated`.

pub mod barrier;
mod cancel;
pub mod cch_order;
#[doc(hidden)]
pub mod codec;
pub mod format;
pub mod math;
#[doc(hidden)]
pub mod raw;
#[doc(hidden)]
pub mod request;
pub mod sar;

pub use cancel::CancelToken;

use serde::{Deserialize, Serialize};
use std::sync::Arc;

pub type ModeMask = u32;

#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct RoadNode {
    pub id: i64,
    pub lat: f64,
    pub lng: f64,
    #[serde(default)]
    pub source_node_id: Option<i64>,
    #[serde(default)]
    pub component_id: Option<u32>,
    #[serde(default)]
    pub incoming_edge_count: u16,
    #[serde(default)]
    pub outgoing_edge_count: u16,
    #[serde(default)]
    pub incident_edge_count: u16,
    #[serde(default)]
    pub junction_kind: RoadJunctionKind,
    #[serde(default)]
    pub highest_incident_road_type: Option<RoadType>,
    #[serde(default)]
    pub roundabout_entry_count: u16,
    #[serde(default)]
    pub roundabout_exit_count: u16,
    #[serde(default)]
    pub is_complex_junction: bool,
    /// The node's [`sar::SarBarrierClass`] code, 0 for none: barriers that
    /// affect vehicles, for the SAR `barriers` relaxation.
    #[serde(default)]
    pub barrier_class: u8,
    /// The node's [`barrier::BarrierAccess`] bits, 0 for none: the profiles
    /// its barrier closes or gates in normal routing.
    #[serde(default)]
    pub barrier_access: u16,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct RoadEdge {
    pub edge_id: u64,
    pub shared_edge_info_id: u64,
    pub distance_m: f32,
    pub base_cost: f32,
    pub mobility_mode_mask: ModeMask,
    pub osm_forward_mode_mask: ModeMask,
    pub osm_reverse_mode_mask: ModeMask,
    #[serde(default)]
    pub traversal_direction: TraversalDirection,
    #[serde(default)]
    pub segment_index: u32,
    #[serde(default)]
    pub start_heading_deg: Option<u16>,
    #[serde(default)]
    pub end_heading_deg: Option<u16>,
    #[serde(default)]
    pub inferred_speed_kph: Option<u16>,
    #[serde(default)]
    pub opposing_edge_id: Option<u64>,
    #[serde(default)]
    pub hierarchy_tier: Option<RoadHierarchyTier>,
    /// The traffic control a vehicle meets on this edge before leaving it at
    /// its end: one tagged at the end node, or on the edge's own stretch for
    /// this direction (see `wayfarer-road-graph`'s builder).
    #[serde(default)]
    pub end_control: TrafficControl,
    /// The edge ends at a junction where it yields: a road of a higher class
    /// meets it there, or it enters a roundabout.
    #[serde(default)]
    pub end_minor_approach: bool,
}

impl RoadEdge {
    /// The OSM modes that may use the edge in its direction.
    pub fn direction_osm_mode_mask(&self) -> ModeMask {
        match self.traversal_direction {
            TraversalDirection::Forward => self.osm_forward_mode_mask,
            TraversalDirection::Backward => self.osm_reverse_mode_mask,
        }
    }

    /// An edge no mode may legally use: kept for SAR only (a drivable way
    /// the law closes to every mode, see [`sar`]). Normal routing never
    /// uses it, and graph statistics leave it out.
    pub fn is_sar_only(&self) -> bool {
        self.direction_osm_mode_mask() == 0
    }

    /// The time to drive the edge at its inferred speed, when it has one.
    pub fn estimated_travel_time_s(&self) -> Option<f32> {
        self.inferred_speed_kph.and_then(|speed| {
            let speed_mps = speed as f32 * 1000.0 / 3600.0;
            (speed_mps > 0.0).then_some(self.distance_m / speed_mps)
        })
    }
}

/// A traffic control on a vehicle's way through a junction (OSM
/// `highway=traffic_signals|stop|give_way`, `crossing=traffic_signals`).
/// Stronger controls have larger codes; an edge keeps the strongest it meets.
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
#[repr(u8)]
pub enum TrafficControl {
    #[default]
    None = 0,
    GiveWay = 1,
    /// Signals of a pedestrian crossing away from a junction.
    CrossingSignals = 2,
    Stop = 3,
    TrafficSignals = 4,
}

impl TrafficControl {
    pub const MAX_CODE: u8 = TrafficControl::TrafficSignals as u8;

    /// Inverse of `as u8`; unknown codes read as `None`.
    pub fn from_code(code: u8) -> Self {
        match code {
            1 => Self::GiveWay,
            2 => Self::CrossingSignals,
            3 => Self::Stop,
            4 => Self::TrafficSignals,
            _ => Self::None,
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct SharedEdgeInfo {
    pub shared_edge_info_id: u64,
    pub osm_way_id: i64,
    pub road_type: RoadType,
    pub geometry: Vec<(f64, f64)>,
    #[serde(default)]
    pub directional_access: DirectionalAccess,
    #[serde(default)]
    pub highway: Option<Arc<str>>,
    #[serde(default)]
    pub name: Option<Arc<str>>,
    #[serde(default)]
    pub ref_name: Option<Arc<str>>,
    #[serde(default)]
    pub route_refs: Vec<String>,
    #[serde(default)]
    pub access: Option<Arc<str>>,
    #[serde(default)]
    pub service: Option<Arc<str>>,
    #[serde(default)]
    pub surface: Option<Arc<str>>,
    #[serde(default)]
    pub tracktype: Option<Arc<str>>,
    #[serde(default)]
    pub smoothness: Option<Arc<str>>,
    #[serde(default)]
    pub maxspeed_kph: Option<f32>,
    /// Direction-specific limits from `maxspeed:forward` / `maxspeed:backward`.
    /// `maxspeed_kph` stays the way-level value; these override it for a
    /// directed edge whose traversal matches the way's digitisation direction
    /// (`forward`) or opposes it (`backward`). Both `None` is the common case.
    #[serde(default)]
    pub maxspeed_forward_kph: Option<f32>,
    #[serde(default)]
    pub maxspeed_backward_kph: Option<f32>,
    /// `maxspeed:hgv` — a lower limit that applies only to heavy goods
    /// vehicles, so it cannot be folded into `maxspeed_kph` without slowing
    /// every other profile.
    #[serde(default)]
    pub hgv_maxspeed_kph: Option<f32>,
    /// Physical and legal limits, as hard gates for a vehicle profile.
    /// Centimetres and kilograms so a threshold comparison is integral; `None`
    /// means unconstrained, which is also what an unparseable tag yields — a
    /// limit that cannot be read must not become a limit that blocks.
    #[serde(default)]
    pub max_height_cm: Option<u16>,
    #[serde(default)]
    pub max_width_cm: Option<u16>,
    /// `width` — the carriageway's physical width, distinct from the posted
    /// `maxwidth` limit and roughly two orders of magnitude more common.
    #[serde(default)]
    pub physical_width_cm: Option<u16>,
    #[serde(default)]
    pub max_weight_kg: Option<u32>,
    /// `maxaxleload` — measured ~3x more frequent than `maxweight`, so an HGV
    /// profile that reads only the latter misses most real restrictions.
    #[serde(default)]
    pub max_axle_load_kg: Option<u32>,
    /// Signed percent, positive uphill along the way's digitisation direction.
    #[serde(default)]
    pub incline_percent: Option<i8>,
    #[serde(default)]
    pub is_4wd_only: bool,
    #[serde(default)]
    pub is_motorroad: bool,
    #[serde(default)]
    pub is_lit: bool,
    /// Raw difficulty and pedestrian-context tags. Kept as strings here on the
    /// same rule as `surface`/`tracktype`/`smoothness`: the tile build reduces
    /// them to cost classes, and the string never reaches the search payload.
    #[serde(default)]
    pub sac_scale: Option<Arc<str>>,
    #[serde(default)]
    pub mtb_scale: Option<Arc<str>>,
    #[serde(default)]
    pub trail_visibility: Option<Arc<str>>,
    #[serde(default)]
    pub sidewalk: Option<Arc<str>>,
    #[serde(default)]
    pub crossing: Option<Arc<str>>,
    #[serde(default)]
    pub lane_count: Option<u8>,
    #[serde(default)]
    pub lanes: Option<Arc<str>>,
    #[serde(default)]
    pub turn_lanes: Option<Arc<str>>,
    #[serde(default)]
    pub turn_lanes_forward: Option<Arc<str>>,
    #[serde(default)]
    pub turn_lanes_backward: Option<Arc<str>>,
    #[serde(default)]
    pub lane_connectivity_available: bool,
    #[serde(default)]
    pub speed_source: Option<Arc<str>>,
    #[serde(default)]
    pub free_flow_speed_kph: Option<f32>,
    #[serde(default)]
    pub constrained_speed_kph: Option<f32>,
    #[serde(default)]
    pub admin_country_id: Option<Arc<str>>,
    #[serde(default)]
    pub admin_region_id: Option<Arc<str>>,
    #[serde(default)]
    pub timezone_id: Option<Arc<str>>,
    #[serde(default)]
    pub conditional_restrictions: Vec<String>,
    #[serde(default)]
    pub is_link: bool,
    #[serde(default)]
    pub is_roundabout: bool,
    #[serde(default)]
    pub is_bridge: bool,
    #[serde(default)]
    pub is_tunnel: bool,
    #[serde(default)]
    pub has_ford: bool,
    #[serde(default)]
    pub is_service_like: bool,
    #[serde(default)]
    pub is_destination_only: bool,
    #[serde(default)]
    pub osm_forward_mode_mask: ModeMask,
    #[serde(default)]
    pub osm_reverse_mode_mask: ModeMask,
    #[serde(default)]
    pub hierarchy_tier: Option<RoadHierarchyTier>,
    #[serde(default)]
    pub layer: Option<i16>,
    #[serde(default)]
    pub level: Option<Arc<str>>,
    #[serde(default)]
    pub destination: Option<Arc<str>>,
    #[serde(default)]
    pub destination_ref: Option<Arc<str>>,
    #[serde(default)]
    pub destination_street: Option<Arc<str>>,
    #[serde(default)]
    pub junction_ref: Option<Arc<str>>,
    #[serde(default)]
    pub is_toll: bool,
    #[serde(default)]
    pub is_ferry: bool,
    #[serde(default)]
    pub has_gate: bool,
    #[serde(default)]
    pub is_access_restricted: bool,
    /// The [`sar::LegalReason`] byte of the way: the tag deciding vehicle
    /// access and whether vehicles are legally denied on a drivable way.
    #[serde(default)]
    pub legal_reason: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RoadType {
    Motorway,
    Primary,
    Secondary,
    Tertiary,
    Residential,
    Track,
    Path,
    #[default]
    Other,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RoadHierarchyTier {
    #[default]
    Local,
    Collector,
    Arterial,
    Highway,
}

impl RoadType {
    pub fn cost_factor(self) -> f32 {
        match self {
            Self::Motorway => 1.0,
            Self::Primary => 1.0,
            Self::Secondary => 1.2,
            Self::Tertiary => 1.5,
            Self::Residential => 2.0,
            Self::Track => 5.0,
            Self::Path => 8.0,
            Self::Other => 3.0,
        }
    }

    pub fn from_highway(highway: Option<&str>) -> Self {
        match highway {
            Some("motorway" | "motorway_link" | "trunk" | "trunk_link") => Self::Motorway,
            Some("primary" | "primary_link") => Self::Primary,
            Some("secondary" | "secondary_link") => Self::Secondary,
            Some("tertiary" | "tertiary_link") => Self::Tertiary,
            Some("residential" | "living_street" | "service" | "unclassified") => Self::Residential,
            Some("track") => Self::Track,
            Some("path" | "footway" | "cycleway" | "bridleway" | "steps" | "pedestrian") => {
                Self::Path
            }
            _ => Self::Other,
        }
    }
}

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize, Default,
)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum TraversalDirection {
    #[default]
    Forward,
    Backward,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RoadJunctionKind {
    #[default]
    Unknown,
    DeadEnd,
    Through,
    Intersection,
    Roundabout,
    Complex,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum DirectionalAccess {
    #[default]
    Both,
    Forward,
    Backward,
    Closed,
}

impl DirectionalAccess {
    pub fn allows_forward(self) -> bool {
        matches!(self, Self::Both | Self::Forward)
    }

    pub fn allows_backward(self) -> bool {
        matches!(self, Self::Both | Self::Backward)
    }
}

pub struct OsmMode;

impl OsmMode {
    pub const MOTOR_VEHICLE: ModeMask = 1 << 0;
    pub const BICYCLE: ModeMask = 1 << 1;
    pub const FOOT: ModeMask = 1 << 2;
    pub const BUS: ModeMask = 1 << 3;
    pub const HGV: ModeMask = 1 << 4;
    pub const EMERGENCY: ModeMask = 1 << 5;
    pub const ALL: ModeMask =
        Self::MOTOR_VEHICLE | Self::BICYCLE | Self::FOOT | Self::BUS | Self::HGV | Self::EMERGENCY;
}

pub struct MobilityMode;

impl MobilityMode {
    pub const WHEELED: ModeMask = 1 << 0;
    pub const TRACKED: ModeMask = 1 << 1;
    pub const ON_FOOT: ModeMask = 1 << 2;
    pub const LEGGED: ModeMask = 1 << 3;
    pub const AERIAL: ModeMask = 1 << 4;
    /// Emergency vehicles: the ways motor vehicles, buses or HGVs may use,
    /// busways included. Every edge open to `WHEELED` has it too.
    pub const EMERGENCY: ModeMask = 1 << 5;
    pub const ALL_GROUND: ModeMask =
        Self::WHEELED | Self::TRACKED | Self::ON_FOOT | Self::LEGGED | Self::EMERGENCY;
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", default)]
pub struct RoadGraphNamedCount {
    pub name: String,
    pub count: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct GeoPoint {
    pub lat: f64,
    pub lng: f64,
}

impl GeoPoint {
    pub fn new(lat: f64, lng: f64) -> Self {
        Self { lat, lng }
    }

    pub fn distance_to(self, other: Self) -> f64 {
        haversine_m(self, other)
    }
}

impl From<(f64, f64)> for GeoPoint {
    fn from(value: (f64, f64)) -> Self {
        Self::new(value.0, value.1)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum MobilityProfile {
    /// Routed and costed exactly as `Wheeled`.
    Default,
    /// A road vehicle (car, van, truck), costed close to driving time.
    /// Private and destination-only roads only at a route's ends.
    Wheeled,
    /// A tracked or off-road vehicle on the vehicle network: tracks are
    /// strongly preferred, and speeds are capped (30 km/h on tracks, 42 km/h
    /// on main roads). Private and destination-only roads only at a route's
    /// ends.
    Tracked,
    /// An emergency vehicle: the roads cars may use and busways, and not
    /// confined to through roads (it may cross private and destination-only
    /// roads).
    Emergency,
    /// A person walking, costed close to walking time (about 5 km/h), as
    /// Valhalla's pedestrian costing: footways, paths and streets alike.
    OnFoot,
    /// Travel on foot that keeps to tracks and paths: the ways `OnFoot` may
    /// use, and bikeable ways `foot=no` closes to it (a cycleway with
    /// `foot=no`, a track with `access=no, bicycle=yes`). Tracks and paths cost
    /// half as much as streets, main roads are avoided more strongly, and the
    /// pace on tracks and paths is faster (6–6.5 km/h).
    Legged,
}

impl MobilityProfile {
    pub fn mobility_mode_mask(self) -> ModeMask {
        match self {
            Self::Default | Self::Wheeled => MobilityMode::WHEELED,
            Self::Tracked => MobilityMode::TRACKED,
            // Edges built before the emergency mode existed (and road graphs
            // of tests) carry only the wheeled and tracked bits.
            Self::Emergency => {
                MobilityMode::WHEELED | MobilityMode::TRACKED | MobilityMode::EMERGENCY
            }
            Self::OnFoot => MobilityMode::ON_FOOT,
            Self::Legged => MobilityMode::LEGGED,
        }
    }
}

/// A directed edge with its end nodes and shared attributes, as routing reads
/// it from a package (or a road graph, in tests).
#[derive(Debug, Clone, PartialEq)]
pub struct GraphViewEdge {
    pub edge: RoadEdge,
    pub source: RoadNode,
    pub target: RoadNode,
    /// The raw (dense) indexes of the edge's end nodes in the graph it was read from.
    pub source_index: u32,
    pub target_index: u32,
    pub raw_edge_index: usize,
    pub shared_info: SharedEdgeInfo,
}

/// Great-circle distance in meters. Query time only (snapping, correlation):
/// it uses the platform libm and must not feed a package; the compiler uses
/// [`math::haversine_m`].
#[allow(clippy::disallowed_methods)]
pub fn haversine_m(a: GeoPoint, b: GeoPoint) -> f64 {
    let lat1 = a.lat.to_radians();
    let lat2 = b.lat.to_radians();
    let dlat = (b.lat - a.lat).to_radians();
    let dlng = (b.lng - a.lng).to_radians();
    let h = (dlat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (dlng / 2.0).sin().powi(2);
    2.0 * EARTH_RADIUS_M * h.sqrt().asin()
}

const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// Query time only, like [`haversine_m`].
#[allow(clippy::disallowed_methods)]
pub fn lat_lng_envelope_for_radius(point: GeoPoint, radius_m: u32) -> (f64, f64, f64, f64) {
    let lat_delta = f64::from(radius_m) / 111_320.0;
    let lng_delta =
        f64::from(radius_m) / (111_320.0 * point.lat.to_radians().cos().abs().max(0.01));
    (
        point.lat - lat_delta,
        point.lng - lng_delta,
        point.lat + lat_delta,
        point.lng + lng_delta,
    )
}

pub fn directed_geometry(info: &SharedEdgeInfo, direction: TraversalDirection) -> Vec<GeoPoint> {
    let mut points = info
        .geometry
        .iter()
        .map(|(lat, lng)| GeoPoint::new(*lat, *lng))
        .collect::<Vec<_>>();
    if direction == TraversalDirection::Backward {
        points.reverse();
    }
    points
}

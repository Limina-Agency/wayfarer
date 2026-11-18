//! Request-path sections: what endpoint correlation and route legs read, so
//! a route request needs nothing beyond the package's flat sections.
//!
//! - a uniform grid over shared-info shapes (`GridCellStart`/`GridEntries`),
//!   from which the correlation window test selects candidate edges: a shape
//!   segment is a candidate when its bounding box meets the query envelope
//!   and it crosses a 0.02° detail cell inside the envelope's cells (the rule
//!   of the former tile archive, kept so responses stay identical);
//! - per shared info, the attributes correlation scores and reports (road
//!   class, access flags, layer, level, name, ref, route refs) and the full
//!   shared info route legs report, as deduplicated attribute classes over a
//!   string table (shared infos are numbered in id order, so their ids
//!   follow from the index and a short list of skipped ids);
//! - per node, the node id, component and roundabout exit count;
//! - opposing edges, derived from the adjacency with stored exceptions;
//! - per edge, the exceptions a leg record needs beyond the shared sections
//!   (a mode mask the profile bits do not rebuild, a tier other than its
//!   shared info's), so every edge's record is rebuilt exactly.

use std::sync::Arc;

use crate::{RoadEdge, RoadHierarchyTier, RoadNode, RoadType, SharedEdgeInfo, TraversalDirection};
use rustc_hash::FxHashMap;

use crate::format::{
    EDGE_FLAG_SHAPE_REVERSED, NO_HEADING, PackageFile, RoutingPackageError, Section, Sections,
    get_varint, profile_bit, put_varint, unzigzag, zigzag,
};
use crate::{GeoPoint, GraphViewEdge, MobilityProfile, lat_lng_envelope_for_radius};

/// Layout revision of the request sections, stored in `RequestMeta[0]`.
pub const REQUEST_LAYOUT: u64 = 2;
/// `RequestMeta` = [layout, node id base, shared-info id base, default
/// component id (`NONE_U32`: none), reserved (0), reach cap
/// `NodeReachSaturated` answers for].
pub const META_LEN: usize = 6;
/// The correlator's reach cap (`DEFAULT_REACH_EDGE_LIMIT`).
pub const SATURATION_CAP: usize = 64;
pub const NONE_U32: u32 = u32::MAX;
pub const NONE_U64: u64 = u64::MAX;
/// Grid cell edge in degrees (about 330 m by 215 m at 50° N).
pub const GRID_CELL_DEG: f64 = 0.003;
/// Correlation windows are filed under 0.02° detail cells.
pub const DETAIL_TILE_DEG: f64 = 0.02;

pub const ATTR_SERVICE_LIKE: u8 = 1 << 0;
pub const ATTR_DESTINATION_ONLY: u8 = 1 << 1;
pub const ATTR_GATE: u8 = 1 << 2;
pub const ATTR_ACCESS_RESTRICTED: u8 = 1 << 3;
pub const ATTR_FERRY: u8 = 1 << 4;

pub fn road_type_code(road_type: RoadType) -> u8 {
    match road_type {
        RoadType::Motorway => 0,
        RoadType::Primary => 1,
        RoadType::Secondary => 2,
        RoadType::Tertiary => 3,
        RoadType::Residential => 4,
        RoadType::Track => 5,
        RoadType::Path => 6,
        RoadType::Other => 7,
    }
}

pub fn road_type_from_code(code: u8) -> RoadType {
    match code {
        0 => RoadType::Motorway,
        1 => RoadType::Primary,
        2 => RoadType::Secondary,
        3 => RoadType::Tertiary,
        4 => RoadType::Residential,
        5 => RoadType::Track,
        6 => RoadType::Path,
        _ => RoadType::Other,
    }
}

pub fn tier_code(tier: Option<RoadHierarchyTier>) -> u32 {
    match tier {
        None => 0,
        Some(RoadHierarchyTier::Local) => 1,
        Some(RoadHierarchyTier::Collector) => 2,
        Some(RoadHierarchyTier::Arterial) => 3,
        Some(RoadHierarchyTier::Highway) => 4,
    }
}

pub fn tier_from_code(code: u32) -> Option<RoadHierarchyTier> {
    match code {
        1 => Some(RoadHierarchyTier::Local),
        2 => Some(RoadHierarchyTier::Collector),
        3 => Some(RoadHierarchyTier::Arterial),
        4 => Some(RoadHierarchyTier::Highway),
        _ => None,
    }
}

/// The detail cell of a point (0.02° cells, as the former archive's tiles).
pub fn detail_tile_xy(lat: f64, lng: f64) -> (i32, i32) {
    (
        ((lng + 180.0) / DETAIL_TILE_DEG).floor() as i32,
        ((lat + 90.0) / DETAIL_TILE_DEG).floor() as i32,
    )
}

/// Whether a segment crosses detail cell (x, y) (Liang–Barsky clipping).
pub fn segment_intersects_detail_tile(start: GeoPoint, end: GeoPoint, x: i32, y: i32) -> bool {
    let size = DETAIL_TILE_DEG;
    let min_lng = f64::from(x) * size - 180.0;
    let min_lat = f64::from(y) * size - 90.0;
    let max_lng = min_lng + size;
    let max_lat = min_lat + size;
    let dx = end.lng - start.lng;
    let dy = end.lat - start.lat;
    let mut lower = 0.0f64;
    let mut upper = 1.0f64;
    for (p, q) in [
        (-dx, start.lng - min_lng),
        (dx, max_lng - start.lng),
        (-dy, start.lat - min_lat),
        (dy, max_lat - start.lat),
    ] {
        if p.abs() <= f64::EPSILON {
            if q < 0.0 {
                return false;
            }
            continue;
        }
        let ratio = q / p;
        if p < 0.0 {
            lower = lower.max(ratio);
        } else {
            upper = upper.min(ratio);
        }
        if lower > upper {
            return false;
        }
    }
    true
}

/// A correlation query envelope and the detail cells it touches.
#[derive(Clone, Copy)]
pub struct Envelope {
    pub min_lat: f64,
    pub min_lng: f64,
    pub max_lat: f64,
    pub max_lng: f64,
    pub tile_min: (i32, i32),
    pub tile_max: (i32, i32),
}

impl Envelope {
    pub fn new(min_lat: f64, min_lng: f64, max_lat: f64, max_lng: f64) -> Self {
        Self {
            min_lat,
            min_lng,
            max_lat,
            max_lng,
            tile_min: detail_tile_xy(min_lat, min_lng),
            tile_max: detail_tile_xy(max_lat, max_lng),
        }
    }

    /// Whether the correlation window `start -> end` is a candidate for the
    /// envelope: its bounding box meets the envelope and it crosses one of
    /// the envelope's detail cells.
    pub fn selects(&self, start: GeoPoint, end: GeoPoint) -> bool {
        let (seg_min_lat, seg_max_lat) = (start.lat.min(end.lat), start.lat.max(end.lat));
        let (seg_min_lng, seg_max_lng) = (start.lng.min(end.lng), start.lng.max(end.lng));
        if seg_max_lat < self.min_lat
            || seg_min_lat > self.max_lat
            || seg_max_lng < self.min_lng
            || seg_min_lng > self.max_lng
        {
            return false;
        }
        let seg_tile_min = detail_tile_xy(seg_min_lat, seg_min_lng);
        let seg_tile_max = detail_tile_xy(seg_max_lat, seg_max_lng);
        let (x0, x1) = (
            seg_tile_min.0.max(self.tile_min.0),
            seg_tile_max.0.min(self.tile_max.0),
        );
        let (y0, y1) = (
            seg_tile_min.1.max(self.tile_min.1),
            seg_tile_max.1.min(self.tile_max.1),
        );
        (y0..=y1).any(|y| (x0..=x1).any(|x| segment_intersects_detail_tile(start, end, x, y)))
    }
}

/// Grid geometry, stored as `RequestGrid = [min_lat, min_lng, cell, rows, cols]`.
#[derive(Clone, Copy)]
pub struct Grid {
    pub min_lat: f64,
    pub min_lng: f64,
    pub cell: f64,
    pub rows: usize,
    pub cols: usize,
}

impl Grid {
    pub fn from_section(values: &[f64]) -> Option<Self> {
        let [min_lat, min_lng, cell, rows, cols] = values.try_into().ok()?;
        (cell > 0.0 && rows >= 1.0 && cols >= 1.0).then_some(Self {
            min_lat,
            min_lng,
            cell,
            rows: rows as usize,
            cols: cols as usize,
        })
    }

    pub fn row(&self, lat: f64) -> usize {
        (((lat - self.min_lat) / self.cell).floor().max(0.0) as usize).min(self.rows - 1)
    }

    pub fn col(&self, lng: f64) -> usize {
        (((lng - self.min_lng) / self.cell).floor().max(0.0) as usize).min(self.cols - 1)
    }

    /// Cells whose span meets the box, as (row range, col range). Monotone in
    /// the box, so any box that meets another shares at least one cell.
    pub fn cells(
        &self,
        min_lat: f64,
        min_lng: f64,
        max_lat: f64,
        max_lng: f64,
    ) -> (
        std::ops::RangeInclusive<usize>,
        std::ops::RangeInclusive<usize>,
    ) {
        (
            self.row(min_lat)..=self.row(max_lat),
            self.col(min_lng)..=self.col(max_lng),
        )
    }
}

/// The adjacency rule for opposing edges: the first edge out of `edge`'s
/// target that returns to its source over the same shared info, against it.
/// The road graph pairs edges by (shared info, segment, direction); where
/// the rule disagrees the build stores an exception.
pub fn derived_opposing(
    first_out: &[u32],
    edge_source: &[u32],
    edge_target: &[u32],
    edge_shared_info: &[u32],
    edge_flags: &[u8],
    edge: u32,
) -> Option<u32> {
    opposing_by_rule(
        first_out,
        edge_target,
        edge_shared_info,
        edge_flags,
        edge,
        edge_source[edge as usize],
    )
}

/// [`derived_opposing`] of a package's edge.
fn package_opposing(g: &Sections<'_>, edge: u32) -> Option<u32> {
    opposing_by_rule(
        g.first_out,
        g.edge_target,
        g.edge_shared_info,
        g.edge_flags,
        edge,
        g.edge_source(edge as usize),
    )
}

/// [`derived_opposing`] of `edge`, which leaves `source`.
fn opposing_by_rule(
    first_out: &[u32],
    edge_target: &[u32],
    edge_shared_info: &[u32],
    edge_flags: &[u8],
    edge: u32,
    source: u32,
) -> Option<u32> {
    let index = edge as usize;
    let target = edge_target[index];
    let reversed = edge_flags[index] & EDGE_FLAG_SHAPE_REVERSED;
    (first_out[target as usize]..first_out[target as usize + 1]).find(|candidate| {
        let at = *candidate as usize;
        *candidate != edge
            && edge_target[at] == source
            && edge_shared_info[at] == edge_shared_info[index]
            && edge_flags[at] & EDGE_FLAG_SHAPE_REVERSED != reversed
    })
}

// ---------------------------------------------------------------------------
// Build
// ---------------------------------------------------------------------------

/// Rich shared-info attributes correlation reads, as an interning key.
#[derive(Clone, PartialEq, Eq, Hash)]
pub struct SharedAttrs {
    pub road_type: u8,
    pub flags: u8,
    pub layer: Option<i16>,
    pub level: Option<Arc<str>>,
    pub name: Option<Arc<str>>,
    pub ref_name: Option<Arc<str>>,
    pub route_refs: Vec<String>,
}

impl SharedAttrs {
    pub fn from_info(info: &SharedEdgeInfo) -> Self {
        let mut flags = 0;
        for (set, bit) in [
            (info.is_service_like, ATTR_SERVICE_LIKE),
            (info.is_destination_only, ATTR_DESTINATION_ONLY),
            (info.has_gate, ATTR_GATE),
            (info.is_access_restricted, ATTR_ACCESS_RESTRICTED),
            (info.is_ferry, ATTR_FERRY),
        ] {
            if set {
                flags |= bit;
            }
        }
        Self {
            road_type: road_type_code(info.road_type),
            flags,
            layer: info.layer,
            level: info.level.clone(),
            name: info.name.clone(),
            ref_name: info.ref_name.clone(),
            route_refs: info.route_refs.clone(),
        }
    }
}

/// An attribute class: what correlation reads, plus the full shared info a
/// route leg reports (without id, way id and geometry, which are stored per
/// shared info).
#[derive(Clone, PartialEq, Eq, Hash)]
pub struct ClassKey {
    pub attrs: SharedAttrs,
    /// `SharedEdgeInfo` in [`crate::codec`] encoding.
    pub leg: Vec<u8>,
}

/// A shared info as a class stores it: without its id, way id and geometry
/// (stored per shared info), and with every string replaced by its id in the
/// request string table. Both conversions destructure and rebuild every field
/// by name, so a field added to `SharedEdgeInfo` fails to compile here
/// instead of being dropped.
#[derive(serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct LegAttrs {
    pub road_type: RoadType,
    pub directional_access: crate::DirectionalAccess,
    pub highway: Option<u32>,
    pub name: Option<u32>,
    pub ref_name: Option<u32>,
    pub route_refs: Vec<u32>,
    pub access: Option<u32>,
    pub service: Option<u32>,
    pub surface: Option<u32>,
    pub tracktype: Option<u32>,
    pub smoothness: Option<u32>,
    pub maxspeed_kph: Option<f32>,
    pub maxspeed_forward_kph: Option<f32>,
    pub maxspeed_backward_kph: Option<f32>,
    pub hgv_maxspeed_kph: Option<f32>,
    pub max_height_cm: Option<u16>,
    pub max_width_cm: Option<u16>,
    pub physical_width_cm: Option<u16>,
    pub max_weight_kg: Option<u32>,
    pub max_axle_load_kg: Option<u32>,
    pub incline_percent: Option<i8>,
    pub is_4wd_only: bool,
    pub is_motorroad: bool,
    pub is_lit: bool,
    pub sac_scale: Option<u32>,
    pub mtb_scale: Option<u32>,
    pub trail_visibility: Option<u32>,
    pub sidewalk: Option<u32>,
    pub crossing: Option<u32>,
    pub lane_count: Option<u8>,
    pub lanes: Option<u32>,
    pub turn_lanes: Option<u32>,
    pub turn_lanes_forward: Option<u32>,
    pub turn_lanes_backward: Option<u32>,
    pub lane_connectivity_available: bool,
    pub speed_source: Option<u32>,
    pub free_flow_speed_kph: Option<f32>,
    pub constrained_speed_kph: Option<f32>,
    pub admin_country_id: Option<u32>,
    pub admin_region_id: Option<u32>,
    pub timezone_id: Option<u32>,
    pub conditional_restrictions: Vec<u32>,
    pub is_link: bool,
    pub is_roundabout: bool,
    pub is_bridge: bool,
    pub is_tunnel: bool,
    pub has_ford: bool,
    pub is_service_like: bool,
    pub is_destination_only: bool,
    pub osm_forward_mode_mask: u32,
    pub osm_reverse_mode_mask: u32,
    pub hierarchy_tier: Option<crate::RoadHierarchyTier>,
    pub layer: Option<i16>,
    pub level: Option<u32>,
    pub destination: Option<u32>,
    pub destination_ref: Option<u32>,
    pub destination_street: Option<u32>,
    pub junction_ref: Option<u32>,
    pub is_toll: bool,
    pub is_ferry: bool,
    pub has_gate: bool,
    pub is_access_restricted: bool,
}

impl LegAttrs {
    pub fn from_info(info: SharedEdgeInfo, intern: &mut dyn FnMut(&str) -> u32) -> Self {
        let SharedEdgeInfo {
            shared_edge_info_id: _,
            osm_way_id: _,
            geometry: _,
            road_type,
            directional_access,
            highway,
            name,
            ref_name,
            route_refs,
            access,
            service,
            surface,
            tracktype,
            smoothness,
            maxspeed_kph,
            maxspeed_forward_kph,
            maxspeed_backward_kph,
            hgv_maxspeed_kph,
            max_height_cm,
            max_width_cm,
            physical_width_cm,
            max_weight_kg,
            max_axle_load_kg,
            incline_percent,
            is_4wd_only,
            is_motorroad,
            is_lit,
            sac_scale,
            mtb_scale,
            trail_visibility,
            sidewalk,
            crossing,
            lane_count,
            lanes,
            turn_lanes,
            turn_lanes_forward,
            turn_lanes_backward,
            lane_connectivity_available,
            speed_source,
            free_flow_speed_kph,
            constrained_speed_kph,
            admin_country_id,
            admin_region_id,
            timezone_id,
            conditional_restrictions,
            is_link,
            is_roundabout,
            is_bridge,
            is_tunnel,
            has_ford,
            is_service_like,
            is_destination_only,
            osm_forward_mode_mask,
            osm_reverse_mode_mask,
            hierarchy_tier,
            layer,
            level,
            destination,
            destination_ref,
            destination_street,
            junction_ref,
            is_toll,
            is_ferry,
            has_gate,
            is_access_restricted,
            // Stored per shared info in its own section.
            legal_reason: _,
        } = info;
        let mut text = |value: Option<Arc<str>>| value.map(|value| intern(&value));
        let highway = text(highway);
        let name = text(name);
        let ref_name = text(ref_name);
        let access = text(access);
        let service = text(service);
        let surface = text(surface);
        let tracktype = text(tracktype);
        let smoothness = text(smoothness);
        let sac_scale = text(sac_scale);
        let mtb_scale = text(mtb_scale);
        let trail_visibility = text(trail_visibility);
        let sidewalk = text(sidewalk);
        let crossing = text(crossing);
        let lanes = text(lanes);
        let turn_lanes = text(turn_lanes);
        let turn_lanes_forward = text(turn_lanes_forward);
        let turn_lanes_backward = text(turn_lanes_backward);
        let speed_source = text(speed_source);
        let admin_country_id = text(admin_country_id);
        let admin_region_id = text(admin_region_id);
        let timezone_id = text(timezone_id);
        let level = text(level);
        let destination = text(destination);
        let destination_ref = text(destination_ref);
        let destination_street = text(destination_street);
        let junction_ref = text(junction_ref);
        Self {
            road_type,
            directional_access,
            highway,
            name,
            ref_name,
            route_refs: route_refs.iter().map(|value| intern(value)).collect(),
            access,
            service,
            surface,
            tracktype,
            smoothness,
            maxspeed_kph,
            maxspeed_forward_kph,
            maxspeed_backward_kph,
            hgv_maxspeed_kph,
            max_height_cm,
            max_width_cm,
            physical_width_cm,
            max_weight_kg,
            max_axle_load_kg,
            incline_percent,
            is_4wd_only,
            is_motorroad,
            is_lit,
            sac_scale,
            mtb_scale,
            trail_visibility,
            sidewalk,
            crossing,
            lane_count,
            lanes,
            turn_lanes,
            turn_lanes_forward,
            turn_lanes_backward,
            lane_connectivity_available,
            speed_source,
            free_flow_speed_kph,
            constrained_speed_kph,
            admin_country_id,
            admin_region_id,
            timezone_id,
            conditional_restrictions: conditional_restrictions
                .iter()
                .map(|value| intern(value))
                .collect(),
            is_link,
            is_roundabout,
            is_bridge,
            is_tunnel,
            has_ford,
            is_service_like,
            is_destination_only,
            osm_forward_mode_mask,
            osm_reverse_mode_mask,
            hierarchy_tier,
            layer,
            level,
            destination,
            destination_ref,
            destination_street,
            junction_ref,
            is_toll,
            is_ferry,
            has_gate,
            is_access_restricted,
        }
    }

    /// The shared info with id 0, way id 0 and no geometry; `None` when a
    /// string id does not resolve.
    pub fn into_info(self, string: &dyn Fn(u32) -> Option<Arc<str>>) -> Option<SharedEdgeInfo> {
        let text = |id: Option<u32>| -> Option<Option<Arc<str>>> {
            match id {
                Some(id) => Some(Some(string(id)?)),
                None => Some(None),
            }
        };
        let list = |ids: Vec<u32>| -> Option<Vec<String>> {
            ids.into_iter()
                .map(|id| string(id).map(|value| value.to_string()))
                .collect()
        };
        Some(SharedEdgeInfo {
            shared_edge_info_id: 0,
            osm_way_id: 0,
            geometry: Vec::new(),
            road_type: self.road_type,
            directional_access: self.directional_access,
            highway: text(self.highway)?,
            name: text(self.name)?,
            ref_name: text(self.ref_name)?,
            route_refs: list(self.route_refs)?,
            access: text(self.access)?,
            service: text(self.service)?,
            surface: text(self.surface)?,
            tracktype: text(self.tracktype)?,
            smoothness: text(self.smoothness)?,
            maxspeed_kph: self.maxspeed_kph,
            maxspeed_forward_kph: self.maxspeed_forward_kph,
            maxspeed_backward_kph: self.maxspeed_backward_kph,
            hgv_maxspeed_kph: self.hgv_maxspeed_kph,
            max_height_cm: self.max_height_cm,
            max_width_cm: self.max_width_cm,
            physical_width_cm: self.physical_width_cm,
            max_weight_kg: self.max_weight_kg,
            max_axle_load_kg: self.max_axle_load_kg,
            incline_percent: self.incline_percent,
            is_4wd_only: self.is_4wd_only,
            is_motorroad: self.is_motorroad,
            is_lit: self.is_lit,
            sac_scale: text(self.sac_scale)?,
            mtb_scale: text(self.mtb_scale)?,
            trail_visibility: text(self.trail_visibility)?,
            sidewalk: text(self.sidewalk)?,
            crossing: text(self.crossing)?,
            lane_count: self.lane_count,
            lanes: text(self.lanes)?,
            turn_lanes: text(self.turn_lanes)?,
            turn_lanes_forward: text(self.turn_lanes_forward)?,
            turn_lanes_backward: text(self.turn_lanes_backward)?,
            lane_connectivity_available: self.lane_connectivity_available,
            speed_source: text(self.speed_source)?,
            free_flow_speed_kph: self.free_flow_speed_kph,
            constrained_speed_kph: self.constrained_speed_kph,
            admin_country_id: text(self.admin_country_id)?,
            admin_region_id: text(self.admin_region_id)?,
            timezone_id: text(self.timezone_id)?,
            conditional_restrictions: list(self.conditional_restrictions)?,
            is_link: self.is_link,
            is_roundabout: self.is_roundabout,
            is_bridge: self.is_bridge,
            is_tunnel: self.is_tunnel,
            has_ford: self.has_ford,
            is_service_like: self.is_service_like,
            is_destination_only: self.is_destination_only,
            osm_forward_mode_mask: self.osm_forward_mode_mask,
            osm_reverse_mode_mask: self.osm_reverse_mode_mask,
            hierarchy_tier: self.hierarchy_tier,
            layer: self.layer,
            level: text(self.level)?,
            destination: text(self.destination)?,
            destination_ref: text(self.destination_ref)?,
            destination_street: text(self.destination_street)?,
            junction_ref: text(self.junction_ref)?,
            is_toll: self.is_toll,
            is_ferry: self.is_ferry,
            has_gate: self.has_gate,
            is_access_restricted: self.is_access_restricted,
            legal_reason: 0,
        })
    }
}

/// The mode mask legs rebuild from the profile access bits: each atomic mode
/// whose profile admits the edge. Every profile's access test (`mask &
/// profile mask != 0`) gives the same answer on it as on the true mask when
/// `exact_mode_mask` holds; other edges store their mask as an exception.
pub fn rebuilt_mode_mask(profile_access: u8) -> u32 {
    [
        MobilityProfile::Wheeled,
        MobilityProfile::Tracked,
        MobilityProfile::OnFoot,
        MobilityProfile::Legged,
    ]
    .into_iter()
    .filter(|profile| profile_access & profile_bit(*profile) != 0)
    .fold(0, |mask, profile| mask | profile.mobility_mode_mask())
}

pub fn exact_mode_mask(mask: u32) -> bool {
    let access = PROFILES
        .iter()
        .filter(|profile| mask & profile.mobility_mode_mask() != 0)
        .fold(0u8, |bits, profile| bits | profile_bit(*profile));
    let rebuilt = rebuilt_mode_mask(access);
    PROFILES.iter().all(|profile| {
        (mask & profile.mobility_mode_mask() != 0) == (rebuilt & profile.mobility_mode_mask() != 0)
    })
}

pub const PROFILES: [MobilityProfile; 6] = [
    MobilityProfile::Default,
    MobilityProfile::Wheeled,
    MobilityProfile::Tracked,
    MobilityProfile::Emergency,
    MobilityProfile::OnFoot,
    MobilityProfile::Legged,
];

/// Interns attribute classes and strings.
#[derive(Default)]
pub struct StringsAndClasses {
    pub classes: FxHashMap<ClassKey, u32>,
    pub class_start: Vec<u32>,
    pub class_bytes: Vec<u8>,
    pub strings: FxHashMap<Arc<str>, u32>,
    pub string_start: Vec<u32>,
    pub string_bytes: Vec<u8>,
}

impl StringsAndClasses {
    pub fn string(&mut self, value: &str) -> u32 {
        if let Some(id) = self.strings.get(value) {
            return *id;
        }
        if self.string_start.is_empty() {
            self.string_start.push(0);
        }
        let id = (self.string_start.len() - 1) as u32;
        self.string_bytes.extend_from_slice(value.as_bytes());
        self.string_start.push(self.string_bytes.len() as u32);
        self.strings.insert(Arc::from(value), id);
        id
    }

    pub fn optional_string(&mut self, value: Option<&str>) -> u64 {
        value.map_or(0, |value| u64::from(self.string(value)) + 1)
    }

    /// Encodes a class as `[varint correlation length][correlation part]
    /// [leg part]`.
    pub fn class(&mut self, key: ClassKey) -> Result<u32, RoutingPackageError> {
        if let Some(id) = self.classes.get(&key) {
            return Ok(*id);
        }
        if self.class_start.is_empty() {
            self.class_start.push(0);
        }
        let attrs = &key.attrs;
        let mut bytes = vec![attrs.road_type, attrs.flags];
        put_varint(
            &mut bytes,
            attrs.layer.map_or(0, |layer| zigzag(i64::from(layer)) + 1),
        );
        for value in [&attrs.level, &attrs.name, &attrs.ref_name] {
            let id = self.optional_string(value.as_deref());
            put_varint(&mut bytes, id);
        }
        put_varint(&mut bytes, attrs.route_refs.len() as u64);
        for route_ref in &attrs.route_refs.clone() {
            let id = self.string(route_ref);
            put_varint(&mut bytes, u64::from(id));
        }
        let id = (self.class_start.len() - 1) as u32;
        put_varint(&mut self.class_bytes, bytes.len() as u64);
        self.class_bytes.extend_from_slice(&bytes);
        // The leg part with its strings interned.
        let leg = crate::codec::from_slice::<SharedEdgeInfo>(&key.leg)
            .ok()
            .and_then(|info| {
                let attrs = LegAttrs::from_info(info, &mut |value| self.string(value));
                crate::codec::to_vec(&attrs).ok()
            })
            .ok_or_else(|| {
                RoutingPackageError::Unsupported(
                    "a shared info could not be encoded for legs".into(),
                )
            })?;
        self.class_bytes.extend_from_slice(&leg);
        self.class_start
            .push(u32::try_from(self.class_bytes.len()).map_err(|_| {
                RoutingPackageError::Unsupported("attribute classes exceed 4 GiB".into())
            })?);
        self.classes.insert(key, id);
        Ok(id)
    }
}

/// Attribute classes decoded for leg records, by class id.
#[derive(Default)]
pub struct LegClassCache(FxHashMap<u32, SharedEdgeInfo>);

/// One correlation candidate: what the correlator's first checks read
/// (access, component, layer, level, polyline in travel direction).
/// `RequestView::correlation_record` builds the full record for candidates
/// that pass them.
pub struct CorrelationEdge {
    edge: u32,
    pub edge_id: u64,
    pub accessible: bool,
    pub component_size: Option<usize>,
    pub layer: Option<i16>,
    pub level: Option<Arc<str>>,
    pub geometry: Vec<GeoPoint>,
}

/// Borrowed request sections.
pub struct RequestView<'g> {
    graph: Sections<'g>,
    grid: Grid,
    node_base: i64,
    shared_base: u64,
    default_component: u32,
    component_sizes: &'g [u32],
    cell_start: &'g [u32],
    entries: &'g [u32],
    node_id_delta: &'g [u32],
    node_id_wide: &'g [i64],
    component_exceptions: &'g [u32],
    shared_id_gaps: &'g [u64],
    shared_attr_class: &'g [u32],
    class_start: &'g [u32],
    class_bytes: &'g [u8],
    opposing_edge: &'g [u32],
    opposing_value: &'g [u32],
    leg_opposing_edge: &'g [u32],
    leg_opposing_value: &'g [u32],
    roundabout_exits: &'g [u32],
    mode_mask_exceptions: &'g [u32],
    tier_exceptions: &'g [u32],
}

impl PackageFile {
    /// Faults the whole package into memory, so the first requests do not
    /// pay a page fault per touched page (adjacency, costs, shapes).
    /// Returns the bytes touched. Call it from a background thread after
    /// opening; it is safe to route concurrently.
    pub fn prefetch_pages(&self) -> usize {
        let bytes = self.storage.bytes();
        // A read-ahead hint where the platform has one (`madvise`); the loop
        // below touches every page either way.
        #[cfg(unix)]
        if let crate::format::Storage::Mapped(map) = &self.storage {
            let _ = map.advise(memmap2::Advice::WillNeed);
        }
        let mut checksum = 0u8;
        for at in (0..bytes.len()).step_by(4096) {
            // SAFETY: `at` is in bounds; a volatile read keeps the touch.
            checksum ^= unsafe { std::ptr::read_volatile(bytes.as_ptr().add(at)) };
        }
        std::hint::black_box(checksum);
        bytes.len()
    }

    /// The request sections.
    pub fn request_view(&self) -> Option<RequestView<'_>> {
        let meta = self.slice::<u64>(Section::RequestMeta);
        if meta.first() != Some(&REQUEST_LAYOUT) || meta.len() < META_LEN {
            return None;
        }
        Some(RequestView {
            graph: self.view(),
            grid: Grid::from_section(self.slice::<f64>(Section::RequestGrid))?,
            node_base: meta[1] as i64,
            shared_base: meta[2],
            default_component: u32::try_from(meta[3]).ok()?,
            component_sizes: self.slice(Section::ComponentSizes),
            cell_start: self.slice(Section::GridCellStart),
            entries: self.slice(Section::GridEntries),
            node_id_delta: self.slice(Section::NodeIdDelta),
            node_id_wide: self.slice(Section::NodeIdWide),
            component_exceptions: self.slice(Section::ComponentExceptions),
            shared_id_gaps: self.slice(Section::SharedInfoIdGaps),
            shared_attr_class: self.slice(Section::SharedAttrClass),
            class_start: self.slice(Section::AttrClassStart),
            class_bytes: self.slice(Section::AttrClassBytes),
            opposing_edge: self.slice(Section::OpposingExceptionEdge),
            opposing_value: self.slice(Section::OpposingExceptionValue),
            leg_opposing_edge: self.slice(Section::LegOpposingExceptionEdge),
            leg_opposing_value: self.slice(Section::LegOpposingExceptionValue),
            roundabout_exits: self.slice(Section::NodeRoundaboutExits),
            mode_mask_exceptions: self.slice(Section::ModeMaskExceptions),
            tier_exceptions: self.slice(Section::TierExceptions),
        })
    }

    /// Validates the request sections against the header counts.
    pub fn validate_request_sections(&self) -> Result<(), RoutingPackageError> {
        let invalid = |message: &str| Err(RoutingPackageError::Invalid(message.to_string()));
        let meta = self.slice::<u64>(Section::RequestMeta);
        if meta.len() != META_LEN || meta[0] != REQUEST_LAYOUT {
            return invalid("request metadata is missing or of another layout");
        }
        let c = self.counts;
        let Some(grid) = Grid::from_section(self.slice::<f64>(Section::RequestGrid)) else {
            return invalid("request grid is invalid");
        };
        let cell_start = self.slice::<u32>(Section::GridCellStart);
        let entries = self.slice::<u32>(Section::GridEntries);
        let class_start = self.slice::<u32>(Section::AttrClassStart);
        let string_start = self.slice::<u32>(Section::StringStart);
        let offsets_ok = |starts: &[u32], len: usize| {
            starts.first() == Some(&0)
                && starts.windows(2).all(|pair| pair[0] <= pair[1])
                && starts.last().is_some_and(|last| *last as usize == len)
        };
        let class_count = class_start.len().saturating_sub(1) as u32;
        let string_count = string_start.len().saturating_sub(1) as u32;
        let opposing_edge = self.slice::<u32>(Section::OpposingExceptionEdge);
        let e = c.edge_count as u32;
        let exceptions = self.slice::<u32>(Section::ComponentExceptions);
        let gaps = self.slice::<u64>(Section::SharedInfoIdGaps);
        let shared_ids_ok = gaps.len().is_multiple_of(2)
            && gaps
                .as_chunks::<2>()
                .0
                .iter()
                .all(|pair| (pair[0] as usize) < c.shared_info_count)
            && gaps
                .as_chunks::<2>()
                .0
                .iter()
                .zip(gaps.as_chunks::<2>().0.iter().skip(1))
                .all(|(left, right)| left[0] < right[0] && left[1] < right[1]);
        let node_ids_ok = self.slice::<u32>(Section::NodeIdDelta).len()
            + self.slice::<i64>(Section::NodeIdWide).len()
            == c.node_count;
        if !shared_ids_ok
            || !node_ids_ok
            || grid
                .rows
                .checked_mul(grid.cols)
                .and_then(|cells| cells.checked_add(1))
                != Some(cell_start.len())
            || !offsets_ok(cell_start, entries.len())
            || entries.iter().any(|edge| *edge >= e)
            || !exceptions.len().is_multiple_of(2)
            || exceptions
                .as_chunks::<2>()
                .0
                .iter()
                .any(|pair| pair[0] as usize >= c.node_count)
            || exceptions
                .as_chunks::<2>()
                .0
                .iter()
                .zip(exceptions.as_chunks::<2>().0.iter().skip(1))
                .any(|(left, right)| left[0] >= right[0])
            || self.slice::<u32>(Section::SharedAttrClass).len() != c.shared_info_count
            || self
                .slice::<u32>(Section::SharedAttrClass)
                .iter()
                .any(|class| *class >= class_count)
            || !offsets_ok(class_start, self.slice::<u8>(Section::AttrClassBytes).len())
            || !offsets_ok(string_start, self.slice::<u8>(Section::StringBytes).len())
            || self
                .slice::<u32>(Section::ConditionalPool)
                .iter()
                .any(|id| *id >= string_count)
            || !sorted_exceptions_ok(
                opposing_edge,
                self.slice(Section::OpposingExceptionValue),
                c.edge_count,
            )
            || !sorted_exceptions_ok(
                self.slice(Section::LegOpposingExceptionEdge),
                self.slice(Section::LegOpposingExceptionValue),
                c.edge_count,
            )
            || !sorted_pairs_ok(self.slice(Section::NodeRoundaboutExits), c.node_count)
            || !sorted_pairs_ok(self.slice(Section::ComponentSizes), usize::MAX)
            || !sorted_pairs_ok(self.slice(Section::ModeMaskExceptions), c.edge_count)
            || !sorted_pairs_ok(self.slice(Section::TierExceptions), c.edge_count)
            || u32::try_from(meta[3]).is_err()
            || self.slice::<u8>(Section::NodeReachSaturated).len() != c.node_count
        {
            return invalid("request sections are inconsistent");
        }
        Ok(())
    }
}

/// Exception lists: sorted, in-range edges with in-range (or none) values.
pub fn sorted_exceptions_ok(edges: &[u32], values: &[u32], edge_count: usize) -> bool {
    edges.len() == values.len()
        && edges.windows(2).all(|pair| pair[0] < pair[1])
        && edges.iter().all(|edge| (*edge as usize) < edge_count)
        && values
            .iter()
            .all(|value| *value == NONE_U32 || (*value as usize) < edge_count)
}

/// (key, value) pairs sorted by an in-range key. A `key_count` of
/// `usize::MAX` accepts every key: on 32-bit targets `u32::MAX as usize`
/// equals it, so `<` alone would reject a `u32::MAX` key there.
pub fn sorted_pairs_ok(pairs: &[u32], key_count: usize) -> bool {
    pairs.len().is_multiple_of(2)
        && pairs
            .as_chunks::<2>()
            .0
            .iter()
            .all(|pair| key_count == usize::MAX || (pair[0] as usize) < key_count)
        && pairs
            .as_chunks::<2>()
            .0
            .iter()
            .zip(pairs.as_chunks::<2>().0.iter().skip(1))
            .all(|(left, right)| left[0] < right[0])
}

/// The value of `key` in sorted (key, value) pairs.
pub fn pair_value(pairs: &[u32], key: u32) -> Option<u32> {
    let count = pairs.len() / 2;
    let (mut low, mut high) = (0, count);
    while low < high {
        let mid = (low + high) / 2;
        match pairs[mid * 2].cmp(&key) {
            std::cmp::Ordering::Less => low = mid + 1,
            std::cmp::Ordering::Greater => high = mid,
            std::cmp::Ordering::Equal => return Some(pairs[mid * 2 + 1]),
        }
    }
    None
}

impl RequestView<'_> {
    pub fn string(&self, id: u64) -> Option<Arc<str>> {
        let id = u32::try_from(id.checked_sub(1)?).ok()?;
        self.graph.string(id).map(Arc::from)
    }

    /// Shared info `shared`'s class, split into its correlation part and its
    /// leg part.
    pub fn class_parts(&self, shared: usize) -> Option<(&[u8], &[u8])> {
        let class = *self.shared_attr_class.get(shared)? as usize;
        let (start, end) = (
            *self.class_start.get(class)? as usize,
            *self.class_start.get(class + 1)? as usize,
        );
        let bytes = self.class_bytes.get(start..end)?;
        let mut at = 0;
        let correlation_len = usize::try_from(get_varint(bytes, &mut at)?).ok()?;
        let split = at.checked_add(correlation_len)?;
        Some((bytes.get(at..split)?, bytes.get(split..)?))
    }

    /// The full shared info legs report (without id, way id and geometry).
    pub fn leg_shared_info(&self, shared: usize) -> Option<SharedEdgeInfo> {
        let (_, leg) = self.class_parts(shared)?;
        crate::codec::from_slice::<LegAttrs>(leg)
            .ok()?
            .into_info(&|id| self.string(u64::from(id) + 1))
    }

    /// Fills `info` with shared info `shared`'s correlation attributes.
    pub fn fill_attributes(&self, shared: usize, info: &mut SharedEdgeInfo) -> Option<()> {
        let (bytes, _) = self.class_parts(shared)?;
        let (&road_type, &flags) = (bytes.first()?, bytes.get(1)?);
        info.road_type = road_type_from_code(road_type);
        info.is_service_like = flags & ATTR_SERVICE_LIKE != 0;
        info.is_destination_only = flags & ATTR_DESTINATION_ONLY != 0;
        info.has_gate = flags & ATTR_GATE != 0;
        info.is_access_restricted = flags & ATTR_ACCESS_RESTRICTED != 0;
        info.is_ferry = flags & ATTR_FERRY != 0;
        let mut at = 2;
        let layer = get_varint(bytes, &mut at)?;
        info.layer = match layer {
            0 => None,
            value => Some(i16::try_from(unzigzag(value - 1)).ok()?),
        };
        info.level = self.string(get_varint(bytes, &mut at)?);
        info.name = self.string(get_varint(bytes, &mut at)?);
        info.ref_name = self.string(get_varint(bytes, &mut at)?);
        let count = get_varint(bytes, &mut at)?;
        info.route_refs = (0..count)
            .map(|_| {
                let id = get_varint(bytes, &mut at)?;
                self.string(id + 1).map(|value| value.to_string())
            })
            .collect::<Option<Vec<_>>>()?;
        Some(())
    }

    pub fn node_id(&self, node: u32) -> i64 {
        match self.node_id_delta.get(node as usize) {
            Some(delta) => self.node_base + i64::from(*delta),
            None => self.node_id_wide[node as usize],
        }
    }

    /// The road graph's component id for the node.
    pub fn component_id(&self, node: u32) -> Option<u32> {
        Some(pair_value(self.component_exceptions, node).unwrap_or(self.default_component))
            .filter(|component| *component != NONE_U32)
    }

    /// The node count of the node's component.
    pub fn component_size(&self, node: u32) -> Option<usize> {
        let component =
            pair_value(self.component_exceptions, node).unwrap_or(self.default_component);
        pair_value(self.component_sizes, component)
            .filter(|size| *size != NONE_U32)
            .map(|size| size as usize)
    }

    /// The compact edge with `edge_id`.
    pub fn edge_for_id(&self, edge_id: u64) -> Option<u32> {
        self.graph.edge_for_id(edge_id)
    }

    pub fn edge_target(&self, edge: u32) -> u32 {
        self.graph.edge_target[edge as usize]
    }

    pub fn edge_source(&self, edge: u32) -> u32 {
        self.graph.edge_source(edge as usize)
    }

    /// The `shared_edge_info_id` of shared info `shared`: shared infos are
    /// numbered in id order, so it is the base plus the index plus the ids
    /// skipped up to it.
    pub fn shared_info_id(&self, shared: usize) -> u64 {
        // The last gap entry at or before `shared`.
        let gaps = self.shared_id_gaps.len() / 2;
        let (mut low, mut high) = (0, gaps);
        while low < high {
            let mid = (low + high) / 2;
            if self.shared_id_gaps[mid * 2] as usize <= shared {
                low = mid + 1;
            } else {
                high = mid;
            }
        }
        let skipped = if low == 0 {
            0
        } else {
            self.shared_id_gaps[(low - 1) * 2 + 1]
        };
        self.shared_base + shared as u64 + skipped
    }

    /// The opposing edge correlation reports (the road graph's pairing).
    pub fn opposing_edge(&self, edge: u32) -> Option<u32> {
        match self.opposing_edge.binary_search(&edge) {
            Ok(at) => Some(self.opposing_value[at]).filter(|value| *value != NONE_U32),
            Err(_) => package_opposing(&self.graph, edge),
        }
    }

    /// The edges whose correlation windows the envelope around `point`
    /// selects, in edge id order. `profile` only sets the records' mode
    /// mask, which the correlator reads for its access check.
    pub fn correlation_edges(
        &self,
        point: GeoPoint,
        max_distance_m: u32,
        profile: MobilityProfile,
    ) -> Vec<CorrelationEdge> {
        self.correlation_edges_sar(point, max_distance_m, profile, None)
    }

    /// [`Self::correlation_edges`] for a SAR request: an edge its relaxations
    /// open (`sar`) is accessible too. Without SAR an edge the profile may not
    /// legally use is not, so normal endpoints never snap onto the physical
    /// graph's SAR-only roads.
    pub fn correlation_edges_sar(
        &self,
        point: GeoPoint,
        max_distance_m: u32,
        profile: MobilityProfile,
        sar: Option<crate::sar::SarEdgeRelaxations>,
    ) -> Vec<CorrelationEdge> {
        let (min_lat, min_lng, max_lat, max_lng) =
            lat_lng_envelope_for_radius(point, max_distance_m);
        self.correlation_edges_within(min_lat, min_lng, max_lat, max_lng, profile, sar)
    }

    pub fn correlation_edges_in_envelope(
        &self,
        min_lat: f64,
        min_lng: f64,
        max_lat: f64,
        max_lng: f64,
        profile: MobilityProfile,
    ) -> Vec<CorrelationEdge> {
        self.correlation_edges_within(min_lat, min_lng, max_lat, max_lng, profile, None)
    }

    fn correlation_edges_within(
        &self,
        min_lat: f64,
        min_lng: f64,
        max_lat: f64,
        max_lng: f64,
        profile: MobilityProfile,
        sar: Option<crate::sar::SarEdgeRelaxations>,
    ) -> Vec<CorrelationEdge> {
        let g = &self.graph;
        let envelope = Envelope::new(min_lat, min_lng, max_lat, max_lng);
        let (rows, cols) = self.grid.cells(min_lat, min_lng, max_lat, max_lng);
        let mut entries = Vec::new();
        for row in rows {
            let first = row * self.grid.cols;
            let (start, end) = (
                self.cell_start[first + cols.start()] as usize,
                self.cell_start[first + cols.end() + 1] as usize,
            );
            entries.extend_from_slice(&self.entries[start..end]);
        }
        entries.sort_unstable();
        entries.dedup();
        // Every directed edge over each entry's shared info, tested on the
        // shared shape decoded once (in its stored direction). A directed
        // edge's windows are the shape's, swapped when it runs against it.
        let mut group = Vec::new();
        let mut shape = Vec::new();
        let mut shapes = Vec::<Vec<GeoPoint>>::new();
        let mut selected = Vec::new(); // (edge id, edge, shape slot)
        for entry in entries {
            let shared = g.edge_shared_info[entry as usize];
            let (a, b) = (g.edge_source(entry as usize), g.edge_target[entry as usize]);
            let ends = if a == b { &[a][..] } else { &[a, b][..] };
            group.clear();
            for node in ends {
                let node = *node as usize;
                group.extend(
                    (g.first_out[node]..g.first_out[node + 1])
                        .filter(|edge| g.edge_shared_info[*edge as usize] == shared),
                );
            }
            g.edge_shape(entry, &mut shape);
            if g.edge_flags[entry as usize] & EDGE_FLAG_SHAPE_REVERSED != 0 {
                shape.reverse();
            }
            let mut slot = None;
            for edge in &group {
                let reversed = g.edge_flags[*edge as usize] & EDGE_FLAG_SHAPE_REVERSED != 0;
                let hit = shape.windows(2).any(|window| {
                    if reversed {
                        envelope.selects(window[1], window[0])
                    } else {
                        envelope.selects(window[0], window[1])
                    }
                });
                if !hit {
                    continue;
                }
                let slot = *slot.get_or_insert_with(|| {
                    shapes.push(shape.clone());
                    shapes.len() - 1
                });
                selected.push((g.edge_id(*edge as usize), *edge, slot));
            }
        }
        // An edge reached from two entries (a representative and an extra
        // entry of the same shared info) is listed once.
        selected.sort_unstable_by_key(|(edge_id, _, _)| *edge_id);
        selected.dedup_by_key(|(edge_id, _, _)| *edge_id);
        let bit = profile_bit(profile);
        let mut candidates = Vec::with_capacity(selected.len());
        for (edge_id, edge, slot) in selected {
            let mut geometry = shapes[slot].clone();
            if g.edge_flags[edge as usize] & EDGE_FLAG_SHAPE_REVERSED != 0 {
                geometry.reverse();
            }
            let shared = g.edge_shared_info[edge as usize] as usize;
            let (layer, level) = self.layer_and_level(shared).unwrap_or_default();
            candidates.push(CorrelationEdge {
                edge,
                edge_id,
                accessible: edge_accessible(g, edge, bit, sar.as_ref()),
                component_size: self.component_size(g.edge_source(edge as usize)),
                layer,
                level,
                geometry,
            });
        }
        candidates
    }

    /// The full record of a correlation candidate (for those that pass the
    /// access and distance checks) and its opposing edge id. `profile` only
    /// sets the record's mode mask, which the correlator's access check reads.
    pub fn correlation_record(
        &self,
        candidate: CorrelationEdge,
        profile: MobilityProfile,
    ) -> Option<(GraphViewEdge, Option<u64>)> {
        let g = &self.graph;
        let edge = candidate.edge;
        let index = edge as usize;
        let shared = g.edge_shared_info[index] as usize;
        let (source, target) = (g.edge_source(index), g.edge_target[index]);
        let shared_edge_info_id = self.shared_info_id(shared);
        let mut info = SharedEdgeInfo {
            shared_edge_info_id,
            osm_way_id: g.si_way.get(shared),
            legal_reason: g.legal_reason(shared).0,
            geometry: candidate
                .geometry
                .iter()
                .map(|point| (point.lat, point.lng))
                .collect(),
            ..Default::default()
        };
        self.fill_attributes(shared, &mut info)?;
        // Correlation reads node ids only (positions come from the shape).
        let node = |node: u32| RoadNode {
            id: self.node_id(node),
            ..Default::default()
        };
        let heading = g.heading_start[index];
        let record = GraphViewEdge {
            edge: RoadEdge {
                edge_id: candidate.edge_id,
                shared_edge_info_id,
                distance_m: g.edge_length[index],
                mobility_mode_mask: if candidate.accessible {
                    profile.mobility_mode_mask()
                } else {
                    0
                },
                // The shape above is already in travel direction.
                traversal_direction: TraversalDirection::Forward,
                start_heading_deg: (heading != NO_HEADING).then_some(heading),
                ..Default::default()
            },
            source: node(source),
            target: node(target),
            source_index: source,
            target_index: target,
            raw_edge_index: index,
            shared_info: info,
        };
        let opposing = self
            .opposing_edge(edge)
            .map(|opposing| g.edge_id(opposing as usize));
        Some((record, opposing))
    }

    /// Shared info `shared`'s layer and level (the start of its class).
    pub fn layer_and_level(&self, shared: usize) -> Option<(Option<i16>, Option<Arc<str>>)> {
        let (bytes, _) = self.class_parts(shared)?;
        let mut at = 2;
        let layer = match get_varint(bytes, &mut at)? {
            0 => None,
            value => Some(i16::try_from(unzigzag(value - 1)).ok()?),
        };
        let level = self.string(get_varint(bytes, &mut at)?);
        Some((layer, level))
    }

    /// The road graph's edge record of `edge` as a route leg reads it: the
    /// edge's own fields, its nodes, and the full shared info with its
    /// polyline. Fields no leg, costing or guidance code reads keep their
    /// defaults.
    pub fn leg_record(&self, edge: u32) -> Option<GraphViewEdge> {
        self.leg_record_cached(edge, &mut LegClassCache::default())
    }

    /// `leg_record`, decoding each attribute class once across calls sharing
    /// `classes`.
    pub fn leg_record_cached(
        &self,
        edge: u32,
        classes: &mut LegClassCache,
    ) -> Option<GraphViewEdge> {
        let g = &self.graph;
        let index = edge as usize;
        if index >= g.edge_count {
            return None;
        }
        let shared = g.edge_shared_info[index] as usize;
        let (source, target) = (g.edge_source(index), g.edge_target[index]);
        let reversed = g.edge_flags[index] & EDGE_FLAG_SHAPE_REVERSED != 0;
        let class = *self.shared_attr_class.get(shared)?;
        let mut info = match classes.0.get(&class) {
            Some(decoded) => decoded.clone(),
            None => {
                let decoded = self.leg_shared_info(shared)?;
                classes.0.insert(class, decoded.clone());
                decoded
            }
        };
        let shared_edge_info_id = self.shared_info_id(shared);
        info.shared_edge_info_id = shared_edge_info_id;
        info.osm_way_id = g.si_way.get(shared);
        info.legal_reason = g.legal_reason(shared).0;
        // The shared shape in its stored direction.
        let (shape_from, shape_to) = if reversed {
            (target, source)
        } else {
            (source, target)
        };
        let mut shape = Vec::new();
        g.shared_shape(shared as u32, shape_from, shape_to, &mut shape);
        info.geometry = shape.iter().map(|point| (point.lat, point.lng)).collect();
        let node = |node: u32| {
            let (lat, lng) = g.node_lat_lng(node);
            RoadNode {
                id: self.node_id(node),
                lat,
                lng,
                roundabout_exit_count: pair_value(self.roundabout_exits, node)
                    .and_then(|count| u16::try_from(count).ok())
                    .unwrap_or(0),
                ..Default::default()
            }
        };
        let heading = |value: u16| (value != NO_HEADING).then_some(value);
        let opposing = match self.leg_opposing_edge.binary_search(&edge) {
            Ok(at) => Some(self.leg_opposing_value[at]).filter(|value| *value != NONE_U32),
            Err(_) => package_opposing(g, edge),
        };
        let inferred = g.edge_inferred_speed[index];
        let mobility_mode_mask = pair_value(self.mode_mask_exceptions, edge)
            .unwrap_or_else(|| rebuilt_mode_mask(g.edge_profile_access[index]));
        let hierarchy_tier = match pair_value(self.tier_exceptions, edge) {
            Some(code) => tier_from_code(code),
            None => info.hierarchy_tier,
        };
        Some(GraphViewEdge {
            edge: RoadEdge {
                edge_id: g.edge_id(index),
                shared_edge_info_id,
                distance_m: g.edge_length[index],
                mobility_mode_mask,
                traversal_direction: if reversed {
                    TraversalDirection::Backward
                } else {
                    TraversalDirection::Forward
                },
                start_heading_deg: heading(g.heading_start[index]),
                end_heading_deg: heading(g.heading_end(index)),
                inferred_speed_kph: (inferred != 0).then(|| inferred - 1),
                opposing_edge_id: opposing.map(|opposing| g.edge_id(opposing as usize)),
                hierarchy_tier,
                end_control: crate::format::junction::control(g.junction(index)),
                end_minor_approach: crate::format::junction::minor_approach(g.junction(index)),
                ..Default::default()
            },
            source: node(source),
            target: node(target),
            source_index: source,
            target_index: target,
            raw_edge_index: index,
            shared_info: info,
        })
    }

    /// The road node of `node`: id, position and roundabout exits (as legs
    /// read it) plus its component.
    pub fn road_node(&self, node: u32) -> RoadNode {
        let (lat, lng) = self.graph.node_lat_lng(node);
        RoadNode {
            id: self.node_id(node),
            lat,
            lng,
            component_id: self.component_id(node),
            roundabout_exit_count: pair_value(self.roundabout_exits, node)
                .and_then(|count| u16::try_from(count).ok())
                .unwrap_or(0),
            ..Default::default()
        }
    }

    /// Edges leaving (or entering) the source node of `edge` whose shared
    /// info is not excluded: the junction counts of route legs.
    pub fn junction_edge_count(
        &self,
        edge: u32,
        excluded_shared_info_ids: &[u64],
        outgoing: bool,
    ) -> usize {
        let node = self.graph.edge_source(edge as usize);
        self.node_edge_count(node, excluded_shared_info_ids, outgoing)
    }

    /// Edges leaving (or entering) `node` whose shared info is not excluded.
    pub fn node_edge_count(
        &self,
        node: u32,
        excluded_shared_info_ids: &[u64],
        outgoing: bool,
    ) -> usize {
        let g = &self.graph;
        let node = node as usize;
        let count = |candidate: u32| {
            let shared = g.edge_shared_info[candidate as usize] as usize;
            !excluded_shared_info_ids.contains(&self.shared_info_id(shared))
        };
        if outgoing {
            (g.first_out[node]..g.first_out[node + 1])
                .filter(|candidate| count(*candidate))
                .count()
        } else {
            g.in_edges(node)
                .filter(|(candidate, _)| count(*candidate))
                .count()
        }
    }
}

/// Open-addressing set of u32 keys for the small, bounded sets of a reach
/// count (at most a few dozen keys), cleared in time proportional to its use.
pub struct SmallSet {
    slots: Vec<u32>,
    used: Vec<u32>,
}

impl SmallSet {
    pub fn new(capacity: usize) -> Self {
        Self {
            slots: vec![NONE_U32; (capacity * 2).next_power_of_two().max(16)],
            used: Vec::with_capacity(capacity),
        }
    }

    /// Inserts `key`; false when it was present.
    pub fn insert(&mut self, key: u32) -> bool {
        if (self.used.len() + 1) * 2 > self.slots.len() {
            let keys = self
                .used
                .drain(..)
                .map(|at| self.slots[at as usize])
                .collect::<Vec<_>>();
            self.slots = vec![NONE_U32; self.slots.len() * 2];
            for old in keys {
                self.insert(old);
            }
        }
        let mask = self.slots.len() - 1;
        let mut at = (key.wrapping_mul(0x9E37_79B1) as usize) & mask;
        loop {
            match self.slots[at] {
                NONE_U32 => {
                    self.slots[at] = key;
                    self.used.push(at as u32);
                    return true;
                }
                existing if existing == key => return false,
                _ => at = (at + 1) & mask,
            }
        }
    }

    // An `is_empty` would be new public API with no caller; not worth adding
    // just for the lint.
    #[allow(clippy::len_without_is_empty)]
    pub fn len(&self) -> usize {
        self.used.len()
    }

    pub fn clear(&mut self) {
        for at in self.used.drain(..) {
            self.slots[at as usize] = NONE_U32;
        }
    }
}

/// The correlator's reach metric (`PackageFile::reach_edge_count`) for
/// the many candidates of one request: the same breadth-first count, with
/// hashed visited sets and a memo per start node, since candidates share
/// nodes. The count is min(reachable, cap), so it depends only on the start.
pub struct ReachCounter<'g> {
    graph: Sections<'g>,
    bit: u8,
    /// A SAR request's R1 and R2: the edges they open count too.
    sar: Option<crate::sar::SarEdgeRelaxations>,
    cap: usize,
    memo: FxHashMap<(u32, bool), usize>,
    /// `NodeReachSaturated` when it answers for `cap`, else empty.
    saturated: &'g [u8],
    edges: SmallSet,
    nodes: SmallSet,
    queue: Vec<u32>,
}

impl<'g> ReachCounter<'g> {
    pub fn new(graph: &'g PackageFile, profile: MobilityProfile, cap: usize) -> Self {
        let meta = graph.slice::<u64>(Section::RequestMeta);
        let saturated = if meta.first() == Some(&REQUEST_LAYOUT)
            && meta
                .get(5)
                .is_some_and(|saturation_cap| *saturation_cap == cap as u64)
        {
            graph.slice::<u8>(Section::NodeReachSaturated)
        } else {
            &[]
        };
        Self {
            graph: graph.view(),
            bit: profile_bit(profile),
            sar: None,
            cap,
            memo: FxHashMap::default(),
            saturated,
            edges: SmallSet::new(cap),
            nodes: SmallSet::new(cap + 1),
            queue: Vec::with_capacity(cap + 1),
        }
    }

    /// This counter for a SAR request: the edges `sar` opens count too.
    pub fn with_sar(mut self, sar: Option<crate::sar::SarEdgeRelaxations>) -> Self {
        self.sar = sar;
        self
    }

    /// Distinct edges admitting the profile reachable forward from the end of
    /// compact edge `edge` (or backward from its start), capped at `cap`.
    pub fn count(&mut self, edge: usize, outgoing: bool) -> Option<usize> {
        let g = self.graph;
        if edge >= g.edge_count {
            return None;
        }
        let start = if outgoing {
            g.edge_target[edge]
        } else {
            g.edge_source(edge)
        };
        self.count_from_node(start, outgoing)
    }

    /// Distinct edges admitting the profile reachable forward (or backward)
    /// from compact node `start`, capped at `cap`.
    pub fn count_from_node(&mut self, start: u32, outgoing: bool) -> Option<usize> {
        if self.cap == 0 {
            return Some(0);
        }
        let g = self.graph;
        if start as usize >= g.node_count {
            return None;
        }
        if self
            .saturated
            .get(start as usize)
            .is_some_and(|bits| bits & self.bit != 0)
        {
            return Some(self.cap);
        }
        if let Some(count) = self.memo.get(&(start, outgoing)) {
            return Some(*count);
        }
        self.edges.clear();
        self.nodes.clear();
        self.queue.clear();
        self.queue.push(start);
        self.nodes.insert(start);
        let mut cursor = 0;
        let count = 'search: loop {
            let Some(node) = self.queue.get(cursor).copied() else {
                break self.edges.len();
            };
            cursor += 1;
            let node = node as usize;
            let (incoming, out) = if outgoing {
                (None, g.first_out[node]..g.first_out[node + 1])
            } else {
                (Some(g.in_edges(node)), 0..0)
            };
            for candidate in incoming
                .into_iter()
                .flatten()
                .map(|(edge, _)| edge)
                .chain(out)
            {
                if !edge_accessible(&g, candidate, self.bit, self.sar.as_ref())
                    || !self.edges.insert(candidate)
                {
                    continue;
                }
                if self.edges.len() >= self.cap {
                    break 'search self.edges.len();
                }
                let next = if outgoing {
                    g.edge_target[candidate as usize]
                } else {
                    g.edge_source(candidate as usize)
                };
                if self.nodes.insert(next) {
                    self.queue.push(next);
                }
            }
        };
        self.memo.insert((start, outgoing), count);
        Some(count)
    }
}

/// Whether compact `edge` admits the profile with `bit`, or a SAR request's
/// relaxations open it.
#[inline]
fn edge_accessible(
    g: &Sections<'_>,
    edge: u32,
    bit: u8,
    sar: Option<&crate::sar::SarEdgeRelaxations>,
) -> bool {
    let access = g.edge_profile_access[edge as usize];
    access & bit != 0
        || sar.is_some_and(|sar| {
            let shared = g.edge_shared_info[edge as usize] as usize;
            sar.factor(crate::sar::SarEdgeFacts::from_cost_classes(
                access,
                g.legal_reason(shared),
                g.si_cost_classes[g.cost_tuple(shared)],
            ))
            .is_some()
        })
}

#[cfg(test)]
mod tests {
    use super::sorted_pairs_ok;

    #[test]
    fn sorted_pairs_ok_accepts_every_key_without_a_bound() {
        // `ComponentSizes` is checked with `usize::MAX`; a `u32::MAX` key must
        // pass on 32-bit targets too, where it equals `usize::MAX`.
        assert!(sorted_pairs_ok(&[0, 7, u32::MAX, 3], usize::MAX));
    }

    #[test]
    fn sorted_pairs_ok_checks_bounds_order_and_pairing() {
        assert!(sorted_pairs_ok(&[1, 0, 4, 0], 5));
        assert!(!sorted_pairs_ok(&[1, 0, 5, 0], 5));
        assert!(!sorted_pairs_ok(&[4, 0, 1, 0], 5));
        assert!(!sorted_pairs_ok(&[1, 0, 1, 0], 5));
        assert!(!sorted_pairs_ok(&[1, 0, 2], 5));
    }
}

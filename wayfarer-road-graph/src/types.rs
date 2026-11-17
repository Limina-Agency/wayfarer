//! The graph's input and fact vocabulary: routable ways, mobility categories,
//! way facts, route memberships and turn restrictions (raw and resolved).

use super::*;

/// Mobility category represented by the mobility mode mask (`MobilityMode`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum MobilityCategory {
    #[default]
    Wheeled,
    Tracked,
    OnFoot,
    Legged,
    Aerial,
}

pub fn mobility_mode_bit(category: MobilityCategory) -> ModeMask {
    match category {
        MobilityCategory::Wheeled => MobilityMode::WHEELED,
        MobilityCategory::Tracked => MobilityMode::TRACKED,
        MobilityCategory::OnFoot => MobilityMode::ON_FOOT,
        MobilityCategory::Legged => MobilityMode::LEGGED,
        MobilityCategory::Aerial => MobilityMode::AERIAL,
    }
}

pub fn mobility_mode_mask_from_osm(osm_mode_mask: ModeMask) -> ModeMask {
    let mut mask = 0u32;
    if osm_mode_mask & (OsmMode::MOTOR_VEHICLE | OsmMode::BUS | OsmMode::HGV) != 0 {
        mask |= mobility_mode_bit(MobilityCategory::Wheeled);
        mask |= mobility_mode_bit(MobilityCategory::Tracked);
    }
    if osm_mode_mask & (OsmMode::FOOT | OsmMode::BICYCLE) != 0 {
        mask |= mobility_mode_bit(MobilityCategory::OnFoot);
        mask |= mobility_mode_bit(MobilityCategory::Legged);
    }
    mask
}

/// Deduplicates the tag values buffered ways carry.
///
/// A region's road ways draw their `highway`, `surface`, `access` and similar
/// values from a vocabulary of a few hundred strings, but each way used to own
/// a separate `String` for every one it set — millions of small allocations for
/// a handful of distinct values. Handing out a shared [`Arc<str>`] collapses
/// those to one allocation per distinct value, and shrinks each field from a
/// 24-byte `Option<String>` to a 16-byte `Option<Arc<str>>`.
#[derive(Debug, Default)]
pub struct TagInterner {
    values: HashSet<Arc<str>>,
}

impl TagInterner {
    pub fn new() -> Self {
        Self::default()
    }

    /// The shared handle for `value`, allocating only on first sight.
    pub fn intern(&mut self, value: &str) -> Arc<str> {
        if let Some(existing) = self.values.get(value) {
            return Arc::clone(existing);
        }
        let shared: Arc<str> = Arc::from(value);
        self.values.insert(Arc::clone(&shared));
        shared
    }

    /// Convenience for the many optional tag fields.
    pub fn intern_opt(&mut self, value: Option<&str>) -> Option<Arc<str>> {
        value.map(|value| self.intern(value))
    }

    /// Distinct values held, for reporting.
    pub fn len(&self) -> usize {
        self.values.len()
    }

    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RoutableWay {
    pub osm_way_id: i64,
    pub points: Vec<(f64, f64)>,
    #[serde(default)]
    pub source_node_ids: Vec<Option<i64>>,
    pub road_type: RoadType,
    #[serde(default)]
    pub directional_access: DirectionalAccess,
    #[serde(default)]
    pub name: Option<Arc<str>>,
    #[serde(default)]
    pub ref_name: Option<Arc<str>>,
    #[serde(default)]
    pub highway: Option<Arc<str>>,
    #[serde(default)]
    pub access: Option<Arc<str>>,
    #[serde(default)]
    pub vehicle: Option<Arc<str>>,
    #[serde(default)]
    pub motor_vehicle: Option<Arc<str>>,
    #[serde(default)]
    pub motorcar: Option<Arc<str>>,
    #[serde(default)]
    pub bicycle: Option<Arc<str>>,
    #[serde(default)]
    pub foot: Option<Arc<str>>,
    #[serde(default)]
    pub oneway_foot: Option<Arc<str>>,
    #[serde(default)]
    pub oneway_bicycle: Option<Arc<str>>,
    #[serde(default)]
    pub motorroad: Option<Arc<str>>,
    #[serde(default)]
    pub service: Option<Arc<str>>,
    #[serde(default)]
    pub surface: Option<Arc<str>>,
    #[serde(default)]
    pub tracktype: Option<Arc<str>>,
    #[serde(default)]
    pub smoothness: Option<Arc<str>>,
    #[serde(default)]
    pub maxspeed: Option<Arc<str>>,
    #[serde(default)]
    pub maxspeed_forward: Option<Arc<str>>,
    #[serde(default)]
    pub maxspeed_backward: Option<Arc<str>>,
    #[serde(default)]
    pub maxspeed_hgv: Option<Arc<str>>,
    #[serde(default)]
    pub maxheight: Option<Arc<str>>,
    #[serde(default)]
    pub maxwidth: Option<Arc<str>>,
    #[serde(default)]
    pub width: Option<Arc<str>>,
    #[serde(default)]
    pub maxweight: Option<Arc<str>>,
    #[serde(default)]
    pub maxaxleload: Option<Arc<str>>,
    #[serde(default)]
    pub incline: Option<Arc<str>>,
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
    pub is_4wd_only: bool,
    #[serde(default)]
    pub is_lit: bool,
    #[serde(default)]
    pub lanes: Option<Arc<str>>,
    #[serde(default)]
    pub has_bridge: bool,
    #[serde(default)]
    pub has_tunnel: bool,
    #[serde(default)]
    pub has_ford: bool,
    #[serde(default)]
    pub is_roundabout: bool,
    #[serde(default)]
    pub is_link: bool,
    #[serde(default)]
    pub is_destination_only: bool,
    #[serde(default)]
    pub access_restricted: bool,
    #[serde(default)]
    pub source_lineage_ids: Vec<i64>,
    #[serde(default)]
    pub coverage_id: Option<Arc<str>>,
    #[serde(default)]
    pub layer: Option<Arc<str>>,
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
    #[serde(default)]
    pub conditional_restrictions: Vec<Arc<str>>,
    /// Raw `area=*` tag. Combined with `highway`/`area_highway`, a truthy value
    /// marks a way as a pedestrian plaza polygon eligible for interior routing.
    #[serde(default)]
    pub area: Option<Arc<str>>,
    /// Raw `area:highway=*` tag, used for plazas mapped as bare area polygons
    /// without a `highway` tag.
    #[serde(default)]
    pub area_highway: Option<Arc<str>>,
}

impl RoutableWay {
    pub fn is_routable(&self) -> bool {
        self.directional_access != DirectionalAccess::Closed
            && (!self.access_restricted
                || matches!(self.access.as_deref(), Some("permit" | "destination")))
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RoadWayFacts {
    pub osm_way_id: i64,
    pub road_type: RoadType,
    #[serde(default)]
    pub directional_access: DirectionalAccess,
    #[serde(default)]
    pub name: Option<String>,
    #[serde(default)]
    pub ref_name: Option<String>,
    #[serde(default)]
    pub highway: Option<String>,
    #[serde(default)]
    pub access: Option<String>,
    #[serde(default)]
    pub vehicle: Option<String>,
    #[serde(default)]
    pub motor_vehicle: Option<String>,
    #[serde(default)]
    pub motorcar: Option<String>,
    #[serde(default)]
    pub bicycle: Option<String>,
    #[serde(default)]
    pub foot: Option<String>,
    #[serde(default)]
    pub oneway_foot: Option<String>,
    #[serde(default)]
    pub oneway_bicycle: Option<String>,
    #[serde(default)]
    pub motorroad: Option<String>,
    #[serde(default)]
    pub service: Option<String>,
    #[serde(default)]
    pub surface: Option<String>,
    #[serde(default)]
    pub tracktype: Option<String>,
    #[serde(default)]
    pub smoothness: Option<String>,
    #[serde(default)]
    pub maxspeed: Option<String>,
    #[serde(default)]
    pub maxspeed_forward: Option<String>,
    #[serde(default)]
    pub maxspeed_backward: Option<String>,
    #[serde(default)]
    pub maxspeed_hgv: Option<String>,
    #[serde(default)]
    pub maxheight: Option<String>,
    #[serde(default)]
    pub maxwidth: Option<String>,
    #[serde(default)]
    pub width: Option<String>,
    #[serde(default)]
    pub maxweight: Option<String>,
    #[serde(default)]
    pub maxaxleload: Option<String>,
    #[serde(default)]
    pub incline: Option<String>,
    #[serde(default)]
    pub sac_scale: Option<String>,
    #[serde(default)]
    pub mtb_scale: Option<String>,
    #[serde(default)]
    pub trail_visibility: Option<String>,
    #[serde(default)]
    pub sidewalk: Option<String>,
    #[serde(default)]
    pub crossing: Option<String>,
    /// Every `*:conditional` tag of the way as `key=value`, sorted, for
    /// example `access:conditional=no @ (Mo-Fr 07:00-09:00)`. Costing parses
    /// them against a request's `conditional_restriction_eval_at_ms`.
    #[serde(default)]
    pub conditional_restrictions: Vec<String>,
    #[serde(default)]
    pub is_4wd_only: bool,
    #[serde(default)]
    pub is_lit: bool,
    #[serde(default)]
    pub lanes: Option<String>,
    #[serde(default)]
    pub turn_lanes: Option<String>,
    #[serde(default)]
    pub turn_lanes_forward: Option<String>,
    #[serde(default)]
    pub turn_lanes_backward: Option<String>,
    #[serde(default)]
    pub lane_connectivity_available: bool,
    #[serde(default)]
    pub admin_country_id: Option<String>,
    #[serde(default)]
    pub admin_region_id: Option<String>,
    #[serde(default)]
    pub timezone_id: Option<String>,
    #[serde(default)]
    pub has_bridge: bool,
    #[serde(default)]
    pub has_tunnel: bool,
    #[serde(default)]
    pub has_ford: bool,
    #[serde(default)]
    pub is_roundabout: bool,
    #[serde(default)]
    pub is_link: bool,
    #[serde(default)]
    pub is_destination_only: bool,
    #[serde(default)]
    pub access_restricted: bool,
    #[serde(default)]
    pub layer: Option<String>,
    #[serde(default)]
    pub level: Option<String>,
    #[serde(default)]
    pub destination: Option<String>,
    #[serde(default)]
    pub destination_ref: Option<String>,
    #[serde(default)]
    pub destination_street: Option<String>,
    #[serde(default)]
    pub junction_ref: Option<String>,
    #[serde(default)]
    pub is_toll: bool,
    #[serde(default)]
    pub is_ferry: bool,
    #[serde(default)]
    pub has_gate: bool,
    #[serde(default)]
    pub is_access_restricted: bool,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RelationMemberRefType {
    #[default]
    Node,
    Way,
    Relation,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RoadTurnRestriction {
    pub relation_id: i64,
    /// `no_*` forbids the turn from `from_way_id` to `to_way_id`; `only_*`
    /// forbids every other turn at the via node or the via ways' end.
    pub restriction: String,
    pub from_way_id: i64,
    pub to_way_id: i64,
    pub via_member_type: RelationMemberRefType,
    /// The via node, or the first via way.
    pub via_member_id: i64,
    /// Every via way in member order, for a restriction over several; empty
    /// means `via_member_id` alone.
    #[serde(default)]
    pub via_way_ids: Vec<i64>,
    /// The modes of a `restriction:<mode>` value; empty for the plain
    /// `restriction`, which binds every vehicle.
    #[serde(default)]
    pub modes: Vec<String>,
    #[serde(default)]
    pub except_modes: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RoadRouteMembership {
    pub relation_id: i64,
    pub way_id: i64,
    pub route: String,
    #[serde(default)]
    pub network: Option<String>,
    #[serde(default)]
    pub ref_name: Option<String>,
    #[serde(default)]
    pub relation_name: Option<String>,
    #[serde(default)]
    pub member_role: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ResolvedEdgeRestriction {
    Simple {
        relation_id: i64,
        restriction: String,
        from_edge_id: u64,
        to_edge_id: u64,
        via_node_id: i64,
        osm_mode_mask: ModeMask,
        mobility_mode_mask: ModeMask,
    },
    Complex {
        relation_id: i64,
        restriction: String,
        from_edge_id: u64,
        to_edge_id: u64,
        via_edge_ids: Vec<u64>,
        osm_mode_mask: ModeMask,
        mobility_mode_mask: ModeMask,
    },
}

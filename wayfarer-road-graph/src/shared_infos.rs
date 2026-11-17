//! [`SharedEdgeInfos`]: the graph's shared edge infos, each stored as its own
//! id, way and geometry plus a handle to its attributes, which are kept once
//! per distinct value.
//!
//! A region has ~1.3M shared infos (one per way segment) but only ~60k
//! distinct attribute sets once the id, way id and geometry are set aside,
//! and a `SharedEdgeInfo` is ~600 bytes. Stored whole, the infos were the
//! largest part of the compile's peak (~0.8 GiB on a region).

use super::*;
use rustc_hash::FxHasher;
use std::hash::{Hash, Hasher};

/// One shared info as the graph stores it: what is its own, and its
/// attribute class.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SharedEdge {
    pub shared_edge_info_id: u64,
    pub osm_way_id: i64,
    pub geometry: Vec<(f64, f64)>,
    /// Index into the classes of the [`SharedEdgeInfos`] that holds it.
    pub class: u32,
}

/// A shared info read from [`SharedEdgeInfos`]: the segment's own fields, and
/// its attributes. `attrs` has no id, way id or geometry (they are zero and
/// empty there): read those from the view.
#[derive(Debug, Clone, Copy)]
pub struct SharedInfoRef<'a> {
    pub shared_edge_info_id: u64,
    pub osm_way_id: i64,
    pub geometry: &'a [(f64, f64)],
    pub attrs: &'a SharedEdgeInfo,
    pub class: u32,
}

impl SharedInfoRef<'_> {
    /// The whole `SharedEdgeInfo`, as it was stored before the split.
    pub fn to_info(&self) -> SharedEdgeInfo {
        SharedEdgeInfo {
            shared_edge_info_id: self.shared_edge_info_id,
            osm_way_id: self.osm_way_id,
            geometry: self.geometry.to_vec(),
            ..self.attrs.clone()
        }
    }
}

/// Distinct attribute sets, numbered in the order they were first added.
#[derive(Debug, Clone, Default)]
pub struct SharedInfoClasses {
    classes: Vec<SharedEdgeInfo>,
    /// Class content hash to the newest class with that hash; `next` chains
    /// the older ones (`u32::MAX` ends a chain).
    by_hash: FxHashMap<u64, u32>,
    next: Vec<u32>,
}

impl SharedInfoClasses {
    pub fn len(&self) -> usize {
        self.classes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.classes.is_empty()
    }

    pub fn get(&self, class: u32) -> &SharedEdgeInfo {
        &self.classes[class as usize]
    }

    /// The classes, by number.
    pub(crate) fn with_capacity(capacity: usize) -> Self {
        Self {
            classes: Vec::with_capacity(capacity),
            by_hash: FxHashMap::with_capacity_and_hasher(capacity, Default::default()),
            next: Vec::with_capacity(capacity),
        }
    }

    pub(crate) fn into_vec(self) -> Vec<SharedEdgeInfo> {
        self.classes
    }

    /// The class of `attrs` (its id, way id and geometry are dropped), added
    /// if new.
    pub fn intern(&mut self, mut attrs: SharedEdgeInfo) -> u32 {
        strip_segment_fields(&mut attrs);
        let hash = class_hash(&attrs);
        let mut at = self.by_hash.get(&hash).copied().unwrap_or(u32::MAX);
        let head = at;
        while at != u32::MAX {
            if class_eq(&self.classes[at as usize], &attrs) {
                return at;
            }
            at = self.next[at as usize];
        }
        let class = u32::try_from(self.classes.len()).expect("fewer than 2^32 classes");
        self.classes.push(attrs);
        self.next.push(head);
        self.by_hash.insert(hash, class);
        class
    }
}

/// The graph's shared infos, in the order they were added.
#[derive(Debug, Clone, Default)]
pub struct SharedEdgeInfos {
    edges: Vec<SharedEdge>,
    classes: SharedInfoClasses,
}

impl SharedEdgeInfos {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn len(&self) -> usize {
        self.edges.len()
    }

    pub fn is_empty(&self) -> bool {
        self.edges.is_empty()
    }

    pub fn reserve_exact(&mut self, additional: usize) {
        self.edges.reserve_exact(additional);
    }

    /// The number of distinct attribute sets.
    pub fn class_count(&self) -> usize {
        self.classes.len()
    }

    pub fn classes(&self) -> &SharedInfoClasses {
        &self.classes
    }

    pub fn edges(&self) -> &[SharedEdge] {
        &self.edges
    }

    pub fn get(&self, index: usize) -> Option<SharedInfoRef<'_>> {
        self.edges.get(index).map(|edge| self.view(edge))
    }

    /// The info at `index`; panics past the end, as slice indexing does.
    pub fn at(&self, index: usize) -> SharedInfoRef<'_> {
        self.view(&self.edges[index])
    }

    pub fn iter(&self) -> impl ExactSizeIterator<Item = SharedInfoRef<'_>> + Clone + '_ {
        self.edges.iter().map(|edge| self.view(edge))
    }

    pub fn par_iter(&self) -> impl IndexedParallelIterator<Item = SharedInfoRef<'_>> + '_ {
        self.edges.par_iter().map(|edge| self.view(edge))
    }

    fn view<'a>(&'a self, edge: &'a SharedEdge) -> SharedInfoRef<'a> {
        SharedInfoRef {
            shared_edge_info_id: edge.shared_edge_info_id,
            osm_way_id: edge.osm_way_id,
            geometry: &edge.geometry,
            attrs: self.classes.get(edge.class),
            class: edge.class,
        }
    }

    /// Appends `info`, sharing its attributes with an equal earlier info's.
    pub fn push(&mut self, mut info: SharedEdgeInfo) {
        let shared_edge_info_id = info.shared_edge_info_id;
        let osm_way_id = info.osm_way_id;
        let geometry = std::mem::take(&mut info.geometry);
        let class = self.classes.intern(info);
        self.edges.push(SharedEdge {
            shared_edge_info_id,
            osm_way_id,
            geometry,
            class,
        });
    }

    /// Appends a segment of an attribute class already added with
    /// [`Self::intern`].
    pub fn push_with_class(
        &mut self,
        shared_edge_info_id: u64,
        osm_way_id: i64,
        geometry: Vec<(f64, f64)>,
        class: u32,
    ) {
        assert!((class as usize) < self.classes.len(), "unknown class");
        self.edges.push(SharedEdge {
            shared_edge_info_id,
            osm_way_id,
            geometry,
            class,
        });
    }

    /// The class of `attrs`, added if new (see [`SharedInfoClasses::intern`]).
    pub fn intern(&mut self, attrs: SharedEdgeInfo) -> u32 {
        self.classes.intern(attrs)
    }

    /// The geometry of the info at `index`.
    pub fn geometry_mut(&mut self, index: usize) -> &mut Vec<(f64, f64)> {
        &mut self.edges[index].geometry
    }

    /// Gives the info at `index` a class already added with [`Self::intern`].
    pub fn set_class(&mut self, index: usize, class: u32) {
        assert!((class as usize) < self.classes.len(), "unknown class");
        self.edges[index].class = class;
    }

    /// Changes the attributes of the info at `index` (its id, way id and
    /// geometry stay).
    pub fn update(&mut self, index: usize, change: impl FnOnce(&mut SharedEdgeInfo)) {
        let mut attrs = self.classes.get(self.edges[index].class).clone();
        change(&mut attrs);
        self.edges[index].class = self.classes.intern(attrs);
    }

    /// Gives info `i` class `classes_of[i]` of `classes`, which replace the
    /// old classes.
    pub(crate) fn replace_classes(&mut self, classes: SharedInfoClasses, classes_of: Vec<u32>) {
        assert_eq!(classes_of.len(), self.edges.len());
        for (edge, class) in self.edges.iter_mut().zip(classes_of) {
            assert!((class as usize) < classes.len(), "unknown class");
            edge.class = class;
        }
        self.classes = classes;
    }

    /// Drops the classes no info uses and numbers the rest in order of first
    /// use. The infos' attributes do not change.
    pub fn compact(&mut self) {
        let mut renumbered = vec![u32::MAX; self.classes.len()];
        let mut used = 0u32;
        for edge in &self.edges {
            let slot = &mut renumbered[edge.class as usize];
            if *slot == u32::MAX {
                *slot = used;
                used += 1;
            }
        }
        if used as usize == self.classes.len()
            && renumbered.iter().enumerate().all(|(at, n)| *n == at as u32)
        {
            return;
        }
        // The used classes move to an exactly sized table, in their new order.
        let mut by_new = vec![0u32; used as usize];
        for (old_class, &new) in renumbered.iter().enumerate() {
            if new != u32::MAX {
                by_new[new as usize] = old_class as u32;
            }
        }
        let mut old = std::mem::take(&mut self.classes.classes);
        let mut classes = SharedInfoClasses::with_capacity(by_new.len());
        for old_class in by_new {
            let interned = classes.intern(std::mem::take(&mut old[old_class as usize]));
            debug_assert_eq!(interned as usize, classes.len() - 1);
        }
        drop(old);
        for edge in &mut self.edges {
            edge.class = renumbered[edge.class as usize];
        }
        self.classes = classes;
    }

    /// Every info whole, in order (for snapshots and tests).
    pub fn to_vec(&self) -> Vec<SharedEdgeInfo> {
        self.iter().map(|info| info.to_info()).collect()
    }
}

impl FromIterator<SharedEdgeInfo> for SharedEdgeInfos {
    fn from_iter<I: IntoIterator<Item = SharedEdgeInfo>>(infos: I) -> Self {
        let mut shared = Self::new();
        for info in infos {
            shared.push(info);
        }
        shared
    }
}

impl From<Vec<SharedEdgeInfo>> for SharedEdgeInfos {
    fn from(infos: Vec<SharedEdgeInfo>) -> Self {
        infos.into_iter().collect()
    }
}

fn strip_segment_fields(attrs: &mut SharedEdgeInfo) {
    attrs.shared_edge_info_id = 0;
    attrs.osm_way_id = 0;
    attrs.geometry = Vec::new();
}

fn f32_bits(value: Option<f32>) -> Option<u32> {
    value.map(f32::to_bits)
}

/// Hashes every attribute (floats by their bits). The destructuring names
/// every field, so a new field fails to compile until it is added here and to
/// [`class_eq`].
fn class_hash(info: &SharedEdgeInfo) -> u64 {
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
        legal_reason,
    } = info;
    let mut hasher = FxHasher::default();
    let h = &mut hasher;
    (*road_type as u8).hash(h);
    (*directional_access as u8).hash(h);
    hierarchy_tier.map(|tier| tier as u8).hash(h);
    for value in [
        highway,
        name,
        ref_name,
        access,
        service,
        surface,
        tracktype,
        smoothness,
        sac_scale,
        mtb_scale,
        trail_visibility,
        sidewalk,
        crossing,
        lanes,
        turn_lanes,
        turn_lanes_forward,
        turn_lanes_backward,
        speed_source,
        admin_country_id,
        admin_region_id,
        timezone_id,
        level,
        destination,
        destination_ref,
        destination_street,
        junction_ref,
    ] {
        value.as_deref().hash(h);
    }
    route_refs.hash(h);
    conditional_restrictions.hash(h);
    for value in [
        maxspeed_kph,
        maxspeed_forward_kph,
        maxspeed_backward_kph,
        hgv_maxspeed_kph,
        free_flow_speed_kph,
        constrained_speed_kph,
    ] {
        f32_bits(*value).hash(h);
    }
    (max_height_cm, max_width_cm, physical_width_cm).hash(h);
    (max_weight_kg, max_axle_load_kg, incline_percent, lane_count).hash(h);
    (
        layer,
        osm_forward_mode_mask,
        osm_reverse_mode_mask,
        legal_reason,
    )
        .hash(h);
    [
        is_4wd_only,
        is_motorroad,
        is_lit,
        lane_connectivity_available,
        is_link,
        is_roundabout,
        is_bridge,
        is_tunnel,
        has_ford,
        is_service_like,
        is_destination_only,
        is_toll,
        is_ferry,
        has_gate,
        is_access_restricted,
    ]
    .hash(h);
    hasher.finish()
}

/// Attribute equality, floats by their bits (so `0.0` and `-0.0` differ and a
/// NaN equals itself): two infos share a class only if every attribute would
/// encode the same. Ids, way ids and geometry are not compared.
fn class_eq(left: &SharedEdgeInfo, right: &SharedEdgeInfo) -> bool {
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
        legal_reason,
    } = left;
    let r = right;
    *road_type == r.road_type
        && *directional_access == r.directional_access
        && *hierarchy_tier == r.hierarchy_tier
        && highway == &r.highway
        && name == &r.name
        && ref_name == &r.ref_name
        && access == &r.access
        && service == &r.service
        && surface == &r.surface
        && tracktype == &r.tracktype
        && smoothness == &r.smoothness
        && sac_scale == &r.sac_scale
        && mtb_scale == &r.mtb_scale
        && trail_visibility == &r.trail_visibility
        && sidewalk == &r.sidewalk
        && crossing == &r.crossing
        && lanes == &r.lanes
        && turn_lanes == &r.turn_lanes
        && turn_lanes_forward == &r.turn_lanes_forward
        && turn_lanes_backward == &r.turn_lanes_backward
        && speed_source == &r.speed_source
        && admin_country_id == &r.admin_country_id
        && admin_region_id == &r.admin_region_id
        && timezone_id == &r.timezone_id
        && level == &r.level
        && destination == &r.destination
        && destination_ref == &r.destination_ref
        && destination_street == &r.destination_street
        && junction_ref == &r.junction_ref
        && route_refs == &r.route_refs
        && conditional_restrictions == &r.conditional_restrictions
        && f32_bits(*maxspeed_kph) == f32_bits(r.maxspeed_kph)
        && f32_bits(*maxspeed_forward_kph) == f32_bits(r.maxspeed_forward_kph)
        && f32_bits(*maxspeed_backward_kph) == f32_bits(r.maxspeed_backward_kph)
        && f32_bits(*hgv_maxspeed_kph) == f32_bits(r.hgv_maxspeed_kph)
        && f32_bits(*free_flow_speed_kph) == f32_bits(r.free_flow_speed_kph)
        && f32_bits(*constrained_speed_kph) == f32_bits(r.constrained_speed_kph)
        && *max_height_cm == r.max_height_cm
        && *max_width_cm == r.max_width_cm
        && *physical_width_cm == r.physical_width_cm
        && *max_weight_kg == r.max_weight_kg
        && *max_axle_load_kg == r.max_axle_load_kg
        && *incline_percent == r.incline_percent
        && *lane_count == r.lane_count
        && *layer == r.layer
        && *osm_forward_mode_mask == r.osm_forward_mode_mask
        && *osm_reverse_mode_mask == r.osm_reverse_mode_mask
        && *legal_reason == r.legal_reason
        && *is_4wd_only == r.is_4wd_only
        && *is_motorroad == r.is_motorroad
        && *is_lit == r.is_lit
        && *lane_connectivity_available == r.lane_connectivity_available
        && *is_link == r.is_link
        && *is_roundabout == r.is_roundabout
        && *is_bridge == r.is_bridge
        && *is_tunnel == r.is_tunnel
        && *has_ford == r.has_ford
        && *is_service_like == r.is_service_like
        && *is_destination_only == r.is_destination_only
        && *is_toll == r.is_toll
        && *is_ferry == r.is_ferry
        && *has_gate == r.has_gate
        && *is_access_restricted == r.is_access_restricted
}

#[cfg(test)]
mod class_tests {
    use super::*;

    fn info(id: u64, way: i64, name: &str) -> SharedEdgeInfo {
        SharedEdgeInfo {
            shared_edge_info_id: id,
            osm_way_id: way,
            geometry: vec![(id as f64, 0.0), (id as f64, 1.0)],
            name: Some(Arc::from(name)),
            maxspeed_kph: Some(50.0),
            ..Default::default()
        }
    }

    #[test]
    fn equal_attributes_share_a_class_and_round_trip() {
        let infos = vec![
            info(1, 10, "a"),
            info(2, 10, "a"),
            info(3, 11, "b"),
            info(4, 12, "a"),
        ];
        let shared = SharedEdgeInfos::from(infos.clone());
        assert_eq!(shared.len(), 4);
        assert_eq!(shared.class_count(), 2);
        assert_eq!(shared.to_vec(), infos);
        assert_eq!(shared.at(3).osm_way_id, 12);
        assert_eq!(shared.at(3).attrs.osm_way_id, 0);
    }

    #[test]
    fn floats_compare_by_bits() {
        let mut negative = info(2, 10, "a");
        negative.maxspeed_kph = Some(-0.0);
        let mut positive = info(1, 10, "a");
        positive.maxspeed_kph = Some(0.0);
        let shared = SharedEdgeInfos::from(vec![positive, negative]);
        assert_eq!(shared.class_count(), 2);
    }

    #[test]
    fn update_and_compact_keep_attributes() {
        let infos = vec![info(1, 10, "a"), info(2, 10, "a"), info(3, 11, "b")];
        let mut shared = SharedEdgeInfos::from(infos.clone());
        shared.update(0, |attrs| attrs.name = Some(Arc::from("b")));
        shared.update(1, |attrs| attrs.name = Some(Arc::from("b")));
        assert_eq!(shared.class_count(), 2);
        shared.compact();
        assert_eq!(shared.class_count(), 1);
        let mut expected = infos;
        expected[0].name = Some(Arc::from("b"));
        expected[1].name = Some(Arc::from("b"));
        assert_eq!(shared.to_vec(), expected);
    }
}

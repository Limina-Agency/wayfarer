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
/// attribute class. Its geometry is a range of the [`SharedEdgeInfos`]'
/// point pool (read it from [`SharedInfoRef::geometry`]).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SharedEdge {
    pub shared_edge_info_id: u64,
    pub osm_way_id: i64,
    /// Index into the classes of the [`SharedEdgeInfos`] that holds it.
    pub class: u32,
    /// The end of its points in the pool (the start is the previous info's).
    geometry_end: u64,
}

/// A shared info read from [`SharedEdgeInfos`]: the segment's own fields, and
/// its attributes. `attrs` has no id, way id or geometry (they are zero and
/// empty there): read those from the view.
#[derive(Debug, Clone, Copy)]
pub struct SharedInfoRef<'a> {
    pub shared_edge_info_id: u64,
    pub osm_way_id: i64,
    pub geometry: &'a [(f64, f64)],
    /// The source ways the info's edges come from: `[osm_way_id]` unless set
    /// otherwise with [`SharedEdgeInfos::set_source_lineage_ids`].
    pub source_lineage_ids: &'a [i64],
    /// The way's `coverage_id` tag, if any.
    pub coverage_id: Option<&'a Arc<str>>,
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
    /// Every info's points, in info order: one allocation instead of one per
    /// info.
    points: Vec<(f64, f64)>,
    classes: SharedInfoClasses,
    /// `(index, lineage)` of the infos whose lineage is not `[osm_way_id]`,
    /// by index: the rare exceptions, instead of a lineage per edge.
    lineage: Vec<(u32, Box<[i64]>)>,
    /// `(index, coverage id)` of the infos that have one, by index.
    coverage: Vec<(u32, Arc<str>)>,
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

    /// Reserves room for `additional` more geometry points.
    pub fn reserve_points_exact(&mut self, additional: usize) {
        self.points.reserve_exact(additional);
    }

    /// The geometry points of every info, in info order.
    pub fn point_count(&self) -> usize {
        self.points.len()
    }

    fn geometry_range(&self, index: usize) -> std::ops::Range<usize> {
        let start = match index {
            0 => 0,
            _ => self.edges[index - 1].geometry_end as usize,
        };
        start..self.edges[index].geometry_end as usize
    }

    fn push_edge(
        &mut self,
        shared_edge_info_id: u64,
        osm_way_id: i64,
        geometry: &[(f64, f64)],
        class: u32,
    ) {
        self.points.extend_from_slice(geometry);
        self.edges.push(SharedEdge {
            shared_edge_info_id,
            osm_way_id,
            class,
            geometry_end: self.points.len() as u64,
        });
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
        (index < self.edges.len()).then(|| self.view(index))
    }

    /// The info at `index`; panics past the end, as slice indexing does.
    pub fn at(&self, index: usize) -> SharedInfoRef<'_> {
        self.view(index)
    }

    pub fn iter(&self) -> impl ExactSizeIterator<Item = SharedInfoRef<'_>> + Clone + '_ {
        (0..self.edges.len()).map(|index| self.view(index))
    }

    pub fn par_iter(&self) -> impl IndexedParallelIterator<Item = SharedInfoRef<'_>> + '_ {
        (0..self.edges.len())
            .into_par_iter()
            .map(|index| self.view(index))
    }

    fn view(&self, index: usize) -> SharedInfoRef<'_> {
        let edge = &self.edges[index];
        SharedInfoRef {
            shared_edge_info_id: edge.shared_edge_info_id,
            osm_way_id: edge.osm_way_id,
            geometry: &self.points[self.geometry_range(index)],
            source_lineage_ids: sparse_get(&self.lineage, index)
                .map_or(std::slice::from_ref(&edge.osm_way_id), |lineage| lineage),
            coverage_id: sparse_get(&self.coverage, index),
            attrs: self.classes.get(edge.class),
            class: edge.class,
        }
    }

    /// Sets the source lineage of the info at `index` (`[osm_way_id]` when not
    /// set).
    pub fn set_source_lineage_ids(&mut self, index: usize, lineage: Vec<i64>) {
        assert!(index < self.edges.len(), "no shared info {index}");
        let lineage = (lineage != [self.edges[index].osm_way_id]).then(|| lineage.into());
        sparse_set(&mut self.lineage, index, lineage);
    }

    /// Sets the `coverage_id` of the info at `index`.
    pub fn set_coverage_id(&mut self, index: usize, coverage_id: Option<Arc<str>>) {
        assert!(index < self.edges.len(), "no shared info {index}");
        sparse_set(&mut self.coverage, index, coverage_id);
    }

    /// Appends `info`, sharing its attributes with an equal earlier info's.
    pub fn push(&mut self, mut info: SharedEdgeInfo) {
        let shared_edge_info_id = info.shared_edge_info_id;
        let osm_way_id = info.osm_way_id;
        let geometry = std::mem::take(&mut info.geometry);
        let class = self.classes.intern(info);
        self.push_edge(shared_edge_info_id, osm_way_id, &geometry, class);
    }

    /// Appends a segment of an attribute class already added with
    /// [`Self::intern`].
    pub fn push_with_class(
        &mut self,
        shared_edge_info_id: u64,
        osm_way_id: i64,
        geometry: &[(f64, f64)],
        class: u32,
    ) {
        assert!((class as usize) < self.classes.len(), "unknown class");
        self.push_edge(shared_edge_info_id, osm_way_id, geometry, class);
    }

    /// The class of `attrs`, added if new (see [`SharedInfoClasses::intern`]).
    pub fn intern(&mut self, attrs: SharedEdgeInfo) -> u32 {
        self.classes.intern(attrs)
    }

    /// Replaces the geometry of the info at `index` (moves the later infos'
    /// points when the length changes).
    pub fn set_geometry(&mut self, index: usize, geometry: &[(f64, f64)]) {
        let range = self.geometry_range(index);
        let delta = geometry.len() as i64 - range.len() as i64;
        self.points.splice(range, geometry.iter().copied());
        for edge in &mut self.edges[index..] {
            edge.geometry_end = (edge.geometry_end as i64 + delta) as u64;
        }
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

/// The value at `index` of a sparse `(index, value)` list sorted by index.
fn sparse_get<T>(entries: &[(u32, T)], index: usize) -> Option<&T> {
    if entries.is_empty() {
        return None;
    }
    entries
        .binary_search_by_key(&index, |(at, _)| *at as usize)
        .ok()
        .map(|at| &entries[at].1)
}

/// Sets (`Some`) or clears (`None`) the value at `index` of a sparse list;
/// appending past the last index is the common case.
fn sparse_set<T>(entries: &mut Vec<(u32, T)>, index: usize, value: Option<T>) {
    let key = u32::try_from(index).expect("fewer than 2^32 shared infos");
    match (entries.binary_search_by_key(&key, |(at, _)| *at), value) {
        (Ok(at), Some(value)) => entries[at].1 = value,
        (Ok(at), None) => {
            entries.remove(at);
        }
        (Err(at), Some(value)) => entries.insert(at, (key, value)),
        (Err(_), None) => {}
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

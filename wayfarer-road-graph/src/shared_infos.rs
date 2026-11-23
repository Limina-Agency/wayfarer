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
    geometry_end: u32,
}

/// A shared info read from [`SharedEdgeInfos`]: the segment's own fields, and
/// its attributes. `attrs` has no id, way id or geometry (they are zero and
/// empty there): read those from the view.
#[derive(Debug, Clone, Copy)]
pub struct SharedInfoRef<'a> {
    pub shared_edge_info_id: u64,
    pub osm_way_id: i64,
    pub geometry: InfoGeometry<'a>,
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

/// A shared info's points, read from the [`SharedEdgeInfos`]' pool, where
/// they are kept as E7 pairs (half the size of `f64` pairs). The rare point
/// that is not the `f64` of its E7 value is kept exactly beside them, so every
/// point reads back as it was pushed.
#[derive(Debug, Clone, Copy)]
pub struct InfoGeometry<'a> {
    fixed: &'a [(i32, i32)],
    /// The exact points among `fixed`, by their pool index.
    exact: &'a [(u32, (f64, f64))],
    /// The pool index of `fixed[0]`.
    start: u32,
}

impl<'a> InfoGeometry<'a> {
    pub fn len(&self) -> usize {
        self.fixed.len()
    }

    pub fn is_empty(&self) -> bool {
        self.fixed.is_empty()
    }

    /// The point at `index`; panics past the end, as slice indexing does.
    pub fn get(&self, index: usize) -> (f64, f64) {
        let (lat, lng) = self.fixed[index];
        if !self.exact.is_empty()
            && let Ok(at) = self
                .exact
                .binary_search_by_key(&(self.start + index as u32), |(at, _)| *at)
        {
            return self.exact[at].1;
        }
        (from_fixed(lat), from_fixed(lng))
    }

    pub fn first(&self) -> Option<(f64, f64)> {
        (!self.is_empty()).then(|| self.get(0))
    }

    pub fn last(&self) -> Option<(f64, f64)> {
        (!self.is_empty()).then(|| self.get(self.len() - 1))
    }

    pub fn iter(
        &self,
    ) -> impl ExactSizeIterator<Item = (f64, f64)> + DoubleEndedIterator + Clone + 'a {
        let geometry = *self;
        (0..self.len()).map(move |index| geometry.get(index))
    }

    pub fn to_vec(&self) -> Vec<(f64, f64)> {
        self.iter().collect()
    }

    /// The points into `buffer`, which is cleared first.
    pub fn read_into<'b>(&self, buffer: &'b mut Vec<(f64, f64)>) -> &'b [(f64, f64)] {
        buffer.clear();
        buffer.extend(self.iter());
        buffer
    }
}

impl PartialEq<[(f64, f64)]> for InfoGeometry<'_> {
    fn eq(&self, other: &[(f64, f64)]) -> bool {
        self.len() == other.len() && self.iter().zip(other).all(|(a, b)| a == *b)
    }
}

/// Distinct attribute sets, numbered in the order they were first added.
///
/// Each set is boxed: a table that is rebuilt (merged, finished, compacted)
/// moves its sets' boxes instead of copying the sets (616 bytes each, and
/// millions of them on a country) into a second table beside the first.
#[derive(Debug, Clone, Default)]
pub struct SharedInfoClasses {
    // Boxed on purpose, see above.
    #[allow(clippy::vec_box)]
    classes: Vec<Box<SharedEdgeInfo>>,
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

    /// Rebuilds the hash chains for the classes in their current order, as
    /// interning them in that order would (the classes are distinct).
    fn reindex(&mut self) {
        self.by_hash.clear();
        self.next.clear();
        for (class, attrs) in self.classes.iter().enumerate() {
            let head = self
                .by_hash
                .insert(class_hash(attrs), class as u32)
                .unwrap_or(u32::MAX);
            self.next.push(head);
        }
        self.by_hash.shrink_to_fit();
        self.next.shrink_to_fit();
    }

    /// The classes' attribute sets, by number.
    #[allow(clippy::vec_box)]
    pub(crate) fn into_boxes(self) -> Vec<Box<SharedEdgeInfo>> {
        self.classes
    }

    /// The class of `attrs` (its id, way id and geometry are dropped), added
    /// if new.
    pub fn intern(&mut self, mut attrs: SharedEdgeInfo) -> u32 {
        strip_segment_fields(&mut attrs);
        let hash = class_hash(&attrs);
        match self.find(hash, &attrs) {
            Ok(class) => class,
            Err(head) => self.push(hash, head, Box::new(attrs)),
        }
    }

    /// [`Self::intern`] for attributes already boxed: a new class keeps the
    /// box.
    pub(crate) fn intern_boxed(&mut self, mut attrs: Box<SharedEdgeInfo>) -> u32 {
        strip_segment_fields(&mut attrs);
        let hash = class_hash(&attrs);
        match self.find(hash, &attrs) {
            Ok(class) => class,
            Err(head) => self.push(hash, head, attrs),
        }
    }

    /// The class equal to `attrs`, or else the head of its hash's chain.
    fn find(&self, hash: u64, attrs: &SharedEdgeInfo) -> Result<u32, u32> {
        let head = self.by_hash.get(&hash).copied().unwrap_or(u32::MAX);
        let mut at = head;
        while at != u32::MAX {
            if class_eq(&self.classes[at as usize], attrs) {
                return Ok(at);
            }
            at = self.next[at as usize];
        }
        Err(head)
    }

    fn push(&mut self, hash: u64, head: u32, attrs: Box<SharedEdgeInfo>) -> u32 {
        let class = u32::try_from(self.classes.len()).expect("fewer than 2^32 classes");
        self.classes.push(attrs);
        self.next.push(head);
        self.by_hash.insert(hash, class);
        class
    }
}

/// Shared infos to append to a [`SharedEdgeInfos`] at once
/// ([`SharedEdgeInfos::append`]): built apart from it (across a thread pool,
/// a batch each), then appended in order. Each info carries a key that names
/// its class when it is appended, and its geometry ends and sparse entries
/// count from the batch's start.
#[derive(Debug, Default)]
pub(crate) struct SharedInfoBatch {
    edges: Vec<SharedEdge>,
    points: Vec<(i32, i32)>,
    exact_points: Vec<(u32, (f64, f64))>,
    lineage: Vec<(u32, Box<[i64]>)>,
    coverage: Vec<(u32, Arc<str>)>,
}

impl SharedInfoBatch {
    pub(crate) fn len(&self) -> usize {
        self.edges.len()
    }

    /// Adds an info, as [`SharedEdgeInfos::push_with_class`] would, with
    /// `key` for its class; returns its index in the batch.
    pub(crate) fn push(
        &mut self,
        shared_edge_info_id: u64,
        osm_way_id: i64,
        geometry: &[(f64, f64)],
        key: u32,
    ) -> usize {
        for &(lat, lng) in geometry {
            let fixed = (to_fixed(lat), to_fixed(lng));
            if !is_exact_fixed((lat, lng), fixed) {
                self.exact_points
                    .push((pool_index(self.points.len()), (lat, lng)));
            }
            self.points.push(fixed);
        }
        self.edges.push(SharedEdge {
            shared_edge_info_id,
            osm_way_id,
            class: key,
            geometry_end: pool_index(self.points.len()),
        });
        self.edges.len() - 1
    }

    /// [`SharedEdgeInfos::set_source_lineage_ids`] of the batch's info at
    /// `index`, which has no lineage set yet.
    pub(crate) fn set_source_lineage_ids(&mut self, index: usize, lineage: &[i64]) {
        if lineage != [self.edges[index].osm_way_id] {
            self.lineage.push((pool_index(index), lineage.into()));
        }
    }

    /// [`SharedEdgeInfos::set_coverage_id`] of the batch's info at `index`,
    /// which has none set yet.
    pub(crate) fn set_coverage_id(&mut self, index: usize, coverage_id: Arc<str>) {
        self.coverage.push((pool_index(index), coverage_id));
    }
}

/// The graph's shared infos, in the order they were added.
#[derive(Debug, Clone, Default)]
pub struct SharedEdgeInfos {
    edges: Vec<SharedEdge>,
    /// Every info's points as E7 pairs, in info order: one allocation instead
    /// of one per info (see [`InfoGeometry`]).
    points: Vec<(i32, i32)>,
    /// `(pool index, point)` of the points that are not the `f64` of their E7
    /// pair, by index.
    exact_points: Vec<(u32, (f64, f64))>,
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
        self.push_points(geometry);
        self.edges.push(SharedEdge {
            shared_edge_info_id,
            osm_way_id,
            class,
            geometry_end: pool_index(self.points.len()),
        });
    }

    /// Appends `geometry` to the pool.
    fn push_points(&mut self, geometry: &[(f64, f64)]) {
        for &(lat, lng) in geometry {
            let fixed = (to_fixed(lat), to_fixed(lng));
            if !is_exact_fixed((lat, lng), fixed) {
                self.exact_points
                    .push((pool_index(self.points.len()), (lat, lng)));
            }
            self.points.push(fixed);
        }
    }

    fn geometry_at(&self, range: std::ops::Range<usize>) -> InfoGeometry<'_> {
        let exact_start = self
            .exact_points
            .partition_point(|(at, _)| (*at as usize) < range.start);
        let exact_end = exact_start
            + self.exact_points[exact_start..]
                .partition_point(|(at, _)| (*at as usize) < range.end);
        InfoGeometry {
            fixed: &self.points[range.clone()],
            exact: &self.exact_points[exact_start..exact_end],
            start: range.start as u32,
        }
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
            geometry: self.geometry_at(self.geometry_range(index)),
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

    /// Appends a batch's infos in its order, as pushing each with
    /// [`Self::push_with_class`] (class `class_of(key)`) and then setting its
    /// lineage and coverage would.
    pub(crate) fn append(&mut self, batch: SharedInfoBatch, class_of: impl Fn(u32) -> u32) {
        let first = self.edges.len();
        let first_point = self.points.len();
        pool_index(first_point + batch.points.len());
        pool_index(first + batch.edges.len());
        self.points.extend_from_slice(&batch.points);
        self.exact_points.extend(
            batch
                .exact_points
                .into_iter()
                .map(|(at, point)| (at + first_point as u32, point)),
        );
        let classes = self.classes.len();
        self.edges.extend(batch.edges.into_iter().map(|edge| {
            let class = class_of(edge.class);
            assert!((class as usize) < classes, "unknown class");
            SharedEdge {
                class,
                geometry_end: edge.geometry_end + first_point as u32,
                ..edge
            }
        }));
        debug_assert!(
            self.lineage
                .last()
                .is_none_or(|(at, _)| (*at as usize) < first)
        );
        self.lineage.extend(
            batch
                .lineage
                .into_iter()
                .map(|(at, lineage)| (at + first as u32, lineage)),
        );
        debug_assert!(
            self.coverage
                .last()
                .is_none_or(|(at, _)| (*at as usize) < first)
        );
        self.coverage.extend(
            batch
                .coverage
                .into_iter()
                .map(|(at, coverage)| (at + first as u32, coverage)),
        );
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
        // The later points move to the end, the new ones go in their place.
        let tail = self.points.split_off(range.end);
        let exact_from = self
            .exact_points
            .partition_point(|(at, _)| (*at as usize) < range.start);
        let exact_tail = self
            .exact_points
            .split_off(exact_from)
            .into_iter()
            .filter(|(at, _)| *at as usize >= range.end)
            .collect::<Vec<_>>();
        self.points.truncate(range.start);
        self.push_points(geometry);
        self.points.extend(tail);
        self.exact_points.extend(
            exact_tail
                .into_iter()
                .map(|(at, point)| (pool_index((at as i64 + delta) as usize), point)),
        );
        for edge in &mut self.edges[index..] {
            edge.geometry_end = pool_index((edge.geometry_end as i64 + delta) as usize);
        }
    }

    /// Moves the infos into a new order: info `i` becomes the one that was at
    /// `order[i]`, with id `first_id + i`. `order` must be a permutation of
    /// the indices. Classes are then numbered in order of first use (see
    /// [`Self::compact`]), so reading the infos in order reads the pool and the
    /// classes in order too.
    pub fn reorder(&mut self, order: &[u32], first_id: u64) {
        assert_eq!(order.len(), self.edges.len(), "not a permutation");
        let mut seen = vec![false; order.len()];
        for old in order {
            let seen = &mut seen[*old as usize];
            assert!(!*seen, "not a permutation");
            *seen = true;
        }
        drop(seen);
        let mut new_at = vec![0u32; order.len()];
        for (new, old) in order.iter().enumerate() {
            new_at[*old as usize] = new as u32;
        }
        let mut points = Vec::with_capacity(self.points.len());
        let mut exact_points = Vec::with_capacity(self.exact_points.len());
        let mut edges = Vec::with_capacity(self.edges.len());
        for (new, old) in order.iter().enumerate() {
            let old = *old as usize;
            let range = self.geometry_range(old);
            let exact_start = self
                .exact_points
                .partition_point(|(at, _)| (*at as usize) < range.start);
            for (at, point) in self.exact_points[exact_start..]
                .iter()
                .take_while(|(at, _)| (*at as usize) < range.end)
            {
                let moved = points.len() + (*at as usize - range.start);
                exact_points.push((pool_index(moved), *point));
            }
            points.extend_from_slice(&self.points[range]);
            edges.push(SharedEdge {
                shared_edge_info_id: first_id + new as u64,
                geometry_end: pool_index(points.len()),
                ..self.edges[old]
            });
        }
        self.edges = edges;
        self.points = points;
        self.exact_points = exact_points;
        fn remap<T>(entries: &mut [(u32, T)], new_at: &[u32]) {
            for (at, _) in entries.iter_mut() {
                *at = new_at[*at as usize];
            }
            entries.sort_unstable_by_key(|(at, _)| *at);
        }
        remap(&mut self.lineage, &new_at);
        remap(&mut self.coverage, &new_at);
        self.compact();
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

    /// Takes the classes out, to be replaced with [`Self::replace_classes`];
    /// until then the infos' classes are not there.
    pub(crate) fn take_classes(&mut self) -> SharedInfoClasses {
        std::mem::take(&mut self.classes)
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
        for edge in &mut self.edges {
            edge.class = renumbered[edge.class as usize];
        }
        // The classes move to their new places within the table, which is then
        // cut to the used ones: a second table, built while the first was still
        // held, was the class table's size again (3M classes of 616 bytes on
        // Germany). The unused classes go after the used ones.
        let classes = &mut self.classes.classes;
        let mut place = renumbered;
        let mut next_unused = used;
        for slot in &mut place {
            if *slot == u32::MAX {
                *slot = next_unused;
                next_unused += 1;
            }
        }
        // Each swap puts one class in its final place.
        for at in 0..classes.len() {
            while place[at] as usize != at {
                let to = place[at] as usize;
                classes.swap(at, to);
                place.swap(at, to);
            }
        }
        classes.truncate(used as usize);
        classes.shrink_to_fit();
        self.classes.reindex();
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

/// A pool position as stored; a pool holds fewer than 2^32 points.
fn pool_index(index: usize) -> u32 {
    u32::try_from(index).expect("fewer than 2^32 shared info points")
}

/// Whether `point` reads back from its E7 pair `fixed` bit for bit.
fn is_exact_fixed((lat, lng): (f64, f64), (lat7, lng7): (i32, i32)) -> bool {
    from_fixed(lat7).to_bits() == lat.to_bits() && from_fixed(lng7).to_bits() == lng.to_bits()
}

#[cfg(test)]
mod class_tests {
    use super::*;

    #[test]
    fn reorder_moves_every_part_of_an_info_with_it() {
        let mut infos = SharedEdgeInfos::new();
        let mut first = info(10, 100, "a");
        // A point that is not the f64 of its E7 pair, kept exactly.
        first.geometry = vec![(0.123_456_789_123, 1.0), (2.0, 3.0), (4.0, 5.0)];
        infos.push(first);
        infos.push(info(20, 200, "b"));
        infos.push(info(30, 300, "a"));
        infos.set_source_lineage_ids(0, vec![100, 101]);
        infos.set_coverage_id(2, Some(Arc::from("c")));
        let before = infos.iter().map(|info| info.to_info()).collect::<Vec<_>>();
        let lineage = |infos: &SharedEdgeInfos, at| infos.at(at).source_lineage_ids.to_vec();
        let coverage = |infos: &SharedEdgeInfos, at| infos.at(at).coverage_id.cloned();
        infos.reorder(&[2, 0, 1], 1);
        for (new, old) in [2usize, 0, 1].into_iter().enumerate() {
            let moved = infos.at(new).to_info();
            assert_eq!(moved.shared_edge_info_id, new as u64 + 1);
            assert_eq!(
                SharedEdgeInfo {
                    shared_edge_info_id: before[old].shared_edge_info_id,
                    ..moved
                },
                before[old]
            );
        }
        assert_eq!(lineage(&infos, 1), vec![100, 101]);
        assert_eq!(lineage(&infos, 0), vec![300]);
        assert_eq!(coverage(&infos, 0).as_deref(), Some("c"));
        assert_eq!(coverage(&infos, 1), None);
        // Classes by first use: "a" (the info now first) is class 0.
        assert_eq!(infos.at(0).class, 0);
        assert_eq!(infos.at(1).class, 0);
        assert_eq!(infos.at(2).class, 1);
    }

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

    #[test]
    fn compact_numbers_classes_by_first_use_and_still_interns() {
        let infos = vec![
            info(1, 10, "a"),
            info(2, 11, "b"),
            info(3, 12, "c"),
            info(4, 13, "d"),
        ];
        let mut shared = SharedEdgeInfos::from(infos.clone());
        // Classes a, b, c, d; the infos come to use d, c, d, a.
        let (a, c, d) = (shared.at(0).class, shared.at(2).class, shared.at(3).class);
        shared.set_class(0, d);
        shared.set_class(1, c);
        shared.set_class(2, d);
        shared.set_class(3, a);
        shared.compact();
        assert_eq!(shared.class_count(), 3);
        let names = (0..4)
            .map(|index| (shared.at(index).class, shared.at(index).attrs.name.clone()))
            .collect::<Vec<_>>();
        assert_eq!(
            names,
            [
                (0, Some(Arc::from("d"))),
                (1, Some(Arc::from("c"))),
                (0, Some(Arc::from("d"))),
                (2, Some(Arc::from("a"))),
            ]
        );
        // The rebuilt chains find each kept class, and a dropped one is new.
        assert_eq!(shared.intern(info(9, 20, "a")), 2);
        assert_eq!(shared.intern(info(9, 20, "d")), 0);
        assert_eq!(shared.intern(info(9, 20, "b")), 3);
    }

    #[test]
    fn points_read_back_exactly_with_and_without_e7_values() {
        let mut infos = SharedEdgeInfos::new();
        let class = infos.intern(SharedEdgeInfo::default());
        let fixed = [(50.0612345, 19.9384567), (50.0612346, 19.9384568)];
        let inexact = [(50.06123456789, 19.0), (50.0, 19.00000001234), (50.1, 19.1)];
        infos.push_with_class(1, 10, &fixed, class);
        infos.push_with_class(2, 20, &inexact, class);
        infos.push_with_class(3, 30, &fixed, class);
        assert_eq!(infos.at(0).geometry.to_vec(), fixed);
        assert_eq!(infos.at(1).geometry.to_vec(), inexact);
        assert_eq!(infos.at(2).geometry.to_vec(), fixed);
        assert_eq!(infos.at(1).geometry.last(), Some((50.1, 19.1)));

        // A geometry replaced in the middle moves the later exact points.
        let replaced = [
            (50.2, 19.2),
            (50.20000000001, 19.2),
            (50.3, 19.3),
            (50.4, 19.4),
        ];
        infos.set_geometry(0, &replaced);
        assert_eq!(infos.at(0).geometry.to_vec(), replaced);
        assert_eq!(infos.at(1).geometry.to_vec(), inexact);
        assert_eq!(infos.at(2).geometry.to_vec(), fixed);
        infos.set_geometry(1, &fixed);
        assert_eq!(infos.at(0).geometry.to_vec(), replaced);
        assert_eq!(infos.at(1).geometry.to_vec(), fixed);
        assert_eq!(infos.at(2).geometry.to_vec(), fixed);
    }
}

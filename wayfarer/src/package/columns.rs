//! Cost columns for requests whose costing differs from the stored
//! default-option columns: non-default `RouteCostingOptions`, the strict
//! destination-only pass, and profiles the package has no column for.
//!
//! A derived column is computed once per (profile, normalized options,
//! destination-only policy) by running the costing function
//! ([`CostingPolicy::edge_cost_inputs`]) over every edge's stored attributes,
//! so its weights are the record weights cast to f32, exactly like the stored
//! columns. Columns ignore conditional tags and are keyed without the
//! evaluation time: a request that evaluates them costs the few edges that
//! carry them at its own time, as sparse cost and duration overrides on the
//! column (`analysis::conditional_overrides`), so requests at different times
//! share one column. Columns are kept in a small LRU (its capacity is set by
//! the package's memory profile).
//!
//! Landmarks stay lower bounds by scaling: with `r = min_e derived_e / base_e`
//! over the edges the derived column can use, every derived path costs at
//! least `r` times its base-column cost, so `r` times a base landmark bound
//! is admissible (and consistent, since turn costs are unchanged). The bound
//! is dropped when some edge is usable in the derived column but not in the
//! base column, because landmark reachability would then be wrong.
//!
//! Where that ratio is poor (Emergency and Tracked, whose tracks cost a
//! fraction of Wheeled's), or the bound is dropped because a SAR column
//! with R5 can use paths the stored landmarks were not built on, the
//! column instead gets landmarks of its own metric, measured at first use
//! from the same landmark nodes (`profile_landmarks`) and shared by every
//! column of the same profile and costing options (the strict
//! destination-only pass, SAR columns; R5 columns share a metric of their
//! own, with every path at R5's package minimum factor).

use std::sync::{Arc, Mutex};
use std::time::Instant;

use rayon::prelude::*;
use rustc_hash::FxHashMap;

use super::profile_landmarks::ProfileLandmarks;
use super::{RoutingPackage, SearchGraph, chord_m, profile_bit, road_type_from_rank};
use crate::costing::{
    CostingPolicy, EdgeCostClasses, EdgeCostDecision, EdgeCostFlags, EdgeCostInputs, JunctionDelay,
};
use crate::types::{MobilityProfile, RouteCostingOptions, RouteOptions};
use wayfarer_package::RoadType;
use wayfarer_package::format::{ColumnPieceKinds, LandmarkTable, PieceKind, PieceKinds};
use wayfarer_package::sar::{
    ACCESS_VEHICLE_PHYSICAL, ACCESS_WRONG_WAY, LegalValue, SarEdgeRelaxations, SarPathClass,
    SarPathRelaxation,
};

/// The SAR bits of an `EdgeProfileAccess` byte.
const SAR_ACCESS_BITS: u8 = ACCESS_VEHICLE_PHYSICAL | ACCESS_WRONG_WAY;

/// Safety margin on the landmark scale against rounding in f32 weights.
const LANDMARK_SCALE_MARGIN: f64 = 1.0 - 1e-6;

/// A derived column whose stored landmarks would be scaled below this gets
/// landmarks of its own metric (see `profile_landmarks`).
const OWN_LANDMARKS_BELOW_SCALE: f64 = 0.9;

/// Landmarks measured for a derived metric: the first of the stored column's
/// (they are kept in maxCover order), as many as a query uses.
const PROFILE_LANDMARK_COUNT: usize = super::landmarks::ACTIVE_LANDMARKS;

/// Borrowed view of one cost column, as the searches read it.
#[derive(Clone, Copy)]
pub(crate) struct CostColumn<'c> {
    pub cost: &'c [f32],
    pub duration: &'c [f32],
    /// Per-request durations replacing the column's (conditional tags
    /// evaluated at the request's time; costs go through the searches' cost
    /// overrides).
    pub duration_overrides: Option<&'c FxHashMap<u32, f32>>,
    /// Landmark tables usable as lower bounds for this column, if any.
    pub landmarks: Option<LandmarkColumn<'c>>,
    /// Admissible cost per metre of straight-line distance.
    pub heuristic_cost_per_m: f64,
    /// The profile's junction delay model; `None` for walking profiles (see
    /// `SearchGraph::junction_delay`).
    pub junction: Option<JunctionDelay>,
    /// PERF-2 spike: the stored column whose CCH metric bounds this column,
    /// and the factor to scale it by (every edge this column can use is
    /// usable there and costs at least `factor` times as much).
    pub potential: Option<(usize, f64)>,
}

/// A landmark column: its tables (the package's, or a derived metric's over
/// the package's rows of `column`) and per-(landmark, direction) quanta,
/// already multiplied by the column's landmark scale.
#[derive(Clone, Copy)]
pub(crate) struct LandmarkColumn<'c> {
    pub column: usize,
    pub scales: &'c [f32],
    pub from: LandmarkTable<'c>,
    pub to: LandmarkTable<'c>,
    /// Landmarks per row.
    pub count: usize,
}

impl<'c> CostColumn<'c> {
    #[inline]
    pub(crate) fn cost(&self, edge: u32) -> f32 {
        self.cost[edge as usize]
    }

    #[inline]
    pub(crate) fn duration(&self, edge: u32) -> f32 {
        if let Some(overrides) = self.duration_overrides
            && let Some(duration) = overrides.get(&edge)
        {
            return *duration;
        }
        self.duration[edge as usize]
    }

    /// This column with `overrides` replacing its durations.
    pub(crate) fn with_durations(self, overrides: &'c FxHashMap<u32, f32>) -> Self {
        Self {
            duration_overrides: (!overrides.is_empty()).then_some(overrides),
            ..self
        }
    }
}

pub(crate) struct DerivedColumn {
    cost: Vec<f32>,
    duration: Vec<f32>,
    landmark_column: Option<usize>,
    landmark_scales: Vec<f32>,
    /// Landmarks of the column's own metric, replacing the stored column's.
    own_landmarks: Option<Arc<ProfileLandmarks>>,
    heuristic_cost_per_m: f64,
    junction: Option<JunctionDelay>,
    /// Ratio by which landmark bounds are scaled (diagnostics).
    pub landmark_scale: Option<f64>,
    /// Time spent deriving the column (diagnostics).
    pub build_ms: f64,
    /// The piece kinds the route search walks with (see [`ColumnMarks`]).
    marks: Option<ColumnMarks>,
    /// PERF-2 spike: see [`CostColumn::potential`].
    potential: Option<(usize, f64)>,
}

/// The piece kinds valid for a derived column: those of the stored column
/// its landmarks come from (wheeled for vehicles, on-foot for walkers), less
/// every piece that an edge the derived column opens touches.
///
/// The walks (`search::Run::walk_forward`) rely only on what a marked node
/// sees: at most two usable core neighbours (a chain node) or a dead-end
/// tree below it of single segments, no turn round without a U-turn, no
/// restriction via it (any profile). A column that can use only edges the
/// stored column can use keeps every one of these at every marked node: it
/// is the stored column with some edges blocked, which the walks handle as
/// they handle a request's blocks. An edge the stored column cannot use
/// changes what its end nodes see, and every marked node of their pieces
/// (the flood over marked nodes along the derived column's usable arcs,
/// both ways, closed as the walks move) loses its mark.
enum ColumnMarks {
    /// The derived column uses no edge the stored column cannot: its marks.
    Stored(usize),
    /// `[node / 64]` chain and tree bits of the stored column, less the
    /// pieces opened edges touch.
    Own { chain: Vec<u64>, tree: Vec<u64> },
}

impl ColumnMarks {
    /// The marks valid for `cost`, a column derived beside stored column
    /// `column`; `None` when the package has no piece kinds.
    fn new(view: &SearchGraph<'_>, cost: &[f32], column: usize) -> Option<Self> {
        let kinds = view.piece_kinds.column(column)?;
        let edge_count = view.edge_count;
        let stored = &view.edge_cost[column * edge_count..(column + 1) * edge_count];
        let opened = cost
            .par_iter()
            .zip(stored)
            .enumerate()
            .filter(|(_, (cost, stored))| cost.is_finite() && !stored.is_finite())
            .map(|(edge, _)| edge as u32)
            .collect::<Vec<_>>();
        if opened.is_empty() {
            return Some(Self::Stored(column));
        }
        let words = view.node_count.div_ceil(64);
        let (mut chain, mut tree) = (vec![0u64; words], vec![0u64; words]);
        for node in 0..view.node_count as u32 {
            let (word, bit) = (node as usize / 64, 1u64 << (node % 64));
            match kinds.kind(node) {
                PieceKind::Chain => chain[word] |= bit,
                PieceKind::Tree => tree[word] |= bit,
                PieceKind::Other => {}
            }
        }
        let mut queue = opened
            .iter()
            .flat_map(|edge| {
                [
                    view.edge_source(*edge as usize),
                    view.edge_target[*edge as usize],
                ]
            })
            .collect::<Vec<_>>();
        while let Some(node) = queue.pop() {
            let (word, bit) = (node as usize / 64, 1u64 << (node % 64));
            if (chain[word] | tree[word]) & bit == 0 {
                continue;
            }
            chain[word] &= !bit;
            tree[word] &= !bit;
            let node = node as usize;
            let out = (view.first_out[node]..view.first_out[node + 1])
                .filter(|edge| cost[*edge as usize].is_finite())
                .map(|edge| view.edge_target[edge as usize]);
            let into = view
                .in_edges(node)
                .filter(|(edge, _)| cost[*edge as usize].is_finite())
                .map(|(_, source)| source);
            queue.extend(out.chain(into));
        }
        Some(Self::Own { chain, tree })
    }
}

/// A request's cost column: a stored one or a shared derived one.
#[derive(Clone)]
pub(crate) enum ColumnHandle {
    Stored {
        column: usize,
        heuristic_cost_per_m: f64,
        junction: Option<JunctionDelay>,
    },
    Derived(Arc<DerivedColumn>),
}

impl ColumnHandle {
    pub(crate) fn column<'a>(&'a self, view: &SearchGraph<'a>) -> CostColumn<'a> {
        match self {
            Self::Stored {
                column,
                heuristic_cost_per_m,
                junction,
            } => {
                let range = column * view.edge_count..(column + 1) * view.edge_count;
                CostColumn {
                    cost: &view.edge_cost[range.clone()],
                    duration: &view.edge_duration[range],
                    duration_overrides: None,
                    landmarks: (view.landmark_count > 0).then_some(LandmarkColumn {
                        column: *column,
                        scales: view.landmark_scale,
                        from: view.landmark_from,
                        to: view.landmark_to,
                        count: view.landmark_count,
                    }),
                    heuristic_cost_per_m: *heuristic_cost_per_m,
                    junction: *junction,
                    potential: Some((*column, 1.0)),
                }
            }
            Self::Derived(derived) => CostColumn {
                cost: &derived.cost,
                duration: &derived.duration,
                duration_overrides: None,
                landmarks: derived.landmark_column.map(|column| {
                    let (from, to, count) = match &derived.own_landmarks {
                        Some(own) => {
                            let (from, to) = own.tables();
                            (from, to, own.count)
                        }
                        None => (view.landmark_from, view.landmark_to, view.landmark_count),
                    };
                    LandmarkColumn {
                        column,
                        scales: &derived.landmark_scales,
                        from,
                        to,
                        count,
                    }
                }),
                heuristic_cost_per_m: derived.heuristic_cost_per_m,
                junction: derived.junction,
                potential: derived.potential,
            },
        }
    }
}

impl ColumnHandle {
    /// Whether the stored column this column's piece kinds were classified
    /// on (see [`ColumnMarks`]) can use `edge`: an override opening such an
    /// edge keeps the walks exact, the request still using only edges that
    /// column can (a strict pass's destination-only pockets). False for a
    /// derived column with its own marks.
    pub(crate) fn classified_usable(&self, view: &SearchGraph<'_>, edge: u32) -> bool {
        let column = match self {
            Self::Stored { column, .. } => *column,
            Self::Derived(derived) => match derived.marks.as_ref() {
                Some(ColumnMarks::Stored(column)) => *column,
                _ => return false,
            },
        };
        view.edge_cost[column * view.edge_count + edge as usize].is_finite()
    }

    /// The piece kinds the route search may walk with on this column (see
    /// [`ColumnMarks`]), if the package has them.
    pub(crate) fn piece_kinds<'a>(
        &'a self,
        view: &SearchGraph<'a>,
    ) -> Option<ColumnPieceKinds<'a>> {
        match self {
            Self::Stored { column, .. } => view.piece_kinds.column(*column),
            Self::Derived(derived) => match derived.marks.as_ref()? {
                ColumnMarks::Stored(column) => view.piece_kinds.column(*column),
                ColumnMarks::Own { chain, tree } => {
                    PieceKinds::new(chain, tree, view.node_count).column(0)
                }
            },
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct ColumnKey {
    pub(super) profile: MobilityProfile,
    pub(super) costing: RouteCostingOptions,
    pub(super) allow_destination_only: bool,
    /// A SAR request's R1 and R2: the edges they open, at their factors.
    pub(super) sar: Option<SarEdgeRelaxations>,
}

pub(crate) struct DerivedColumnCache {
    /// Least recently used first.
    entries: Mutex<Vec<(ColumnKey, Arc<DerivedColumn>)>>,
    /// Columns kept (each is 8 bytes per edge, ~20 MB regionally).
    capacity: usize,
}

impl DerivedColumnCache {
    pub(crate) fn with_capacity(capacity: usize) -> Self {
        Self {
            entries: Mutex::new(Vec::new()),
            capacity,
        }
    }

    fn get(&self, key: &ColumnKey) -> Option<Arc<DerivedColumn>> {
        let mut entries = self.entries.lock().ok()?;
        let position = entries.iter().position(|(entry, _)| entry == key)?;
        let entry = entries.remove(position);
        let column = entry.1.clone();
        entries.push(entry);
        Some(column)
    }

    fn insert(&self, key: ColumnKey, column: Arc<DerivedColumn>) -> Arc<DerivedColumn> {
        let Ok(mut entries) = self.entries.lock() else {
            return column;
        };
        // Another thread may have derived the same column meanwhile.
        if let Some(existing) = entries.iter().find(|(entry, _)| *entry == key) {
            return existing.1.clone();
        }
        if self.capacity == 0 {
            return column;
        }
        if entries.len() >= self.capacity {
            entries.remove(0);
        }
        entries.push((key, column.clone()));
        column
    }
}

/// The metric a set of [`ProfileLandmarks`] was measured on: a profile under
/// normalized costing options (without the evaluation time), with or without
/// the paths R5 opens.
#[derive(Debug, Clone, Copy, PartialEq)]
struct MetricKey {
    profile: MobilityProfile,
    costing: RouteCostingOptions,
    /// Every path R5 can open, at its package minimum factor: the metric of
    /// R5 columns, whatever classes and widths they open.
    paths: bool,
}

impl ColumnKey {
    fn metric(&self) -> MetricKey {
        MetricKey {
            profile: self.profile,
            costing: self.costing,
            paths: self.sar.is_some_and(|sar| sar.vehicle_on_paths.is_some()),
        }
    }
}

/// Landmarks of derived metrics, least recently used first. A failed build
/// (`None`) is kept too, so that it is not retried on every column.
pub(crate) struct ProfileLandmarkCache {
    entries: Mutex<Vec<(MetricKey, Option<Arc<ProfileLandmarks>>)>>,
    capacity: usize,
}

impl ProfileLandmarkCache {
    pub(crate) fn with_capacity(capacity: usize) -> Self {
        Self {
            entries: Mutex::new(Vec::new()),
            capacity,
        }
    }

    fn get(&self, key: &MetricKey) -> Option<Option<Arc<ProfileLandmarks>>> {
        let mut entries = self.entries.lock().ok()?;
        let position = entries.iter().position(|(entry, _)| entry == key)?;
        let entry = entries.remove(position);
        let landmarks = entry.1.clone();
        entries.push(entry);
        Some(landmarks)
    }

    fn insert(
        &self,
        key: MetricKey,
        landmarks: Option<Arc<ProfileLandmarks>>,
    ) -> Option<Arc<ProfileLandmarks>> {
        let Ok(mut entries) = self.entries.lock() else {
            return landmarks;
        };
        if let Some(existing) = entries.iter().find(|(entry, _)| *entry == key) {
            return existing.1.clone();
        }
        if self.capacity == 0 {
            return landmarks;
        }
        if entries.len() >= self.capacity {
            entries.remove(0);
        }
        entries.push((key, landmarks.clone()));
        landmarks
    }
}

fn canonical(profile: MobilityProfile) -> MobilityProfile {
    match profile {
        MobilityProfile::Default => MobilityProfile::Wheeled,
        other => other,
    }
}

/// The stored column whose landmarks a derived column of `profile` borrows.
fn landmark_profile(profile: MobilityProfile) -> MobilityProfile {
    match canonical(profile) {
        MobilityProfile::OnFoot | MobilityProfile::Legged => MobilityProfile::OnFoot,
        _ => MobilityProfile::Wheeled,
    }
}

impl RoutingPackage {
    /// The cost column for `profile` under `costing` (and the destination-only
    /// policy): the stored column when the options are the defaults it was
    /// built with (and the package's costing model is this build's), else a
    /// derived column from the LRU, derived on a miss.
    /// Every profile and option set has a column.
    #[cfg(test)]
    pub(crate) fn request_column(
        &self,
        profile: MobilityProfile,
        costing: &RouteCostingOptions,
        allow_destination_only: bool,
    ) -> ColumnHandle {
        self.request_column_sar(profile, costing, allow_destination_only, None)
    }

    /// [`Self::request_column`] for a request whose SAR relaxations open
    /// edges the profile may not legally use (`sar`, R1 and R2): always a
    /// derived column, keyed by them too.
    pub(crate) fn request_column_sar(
        &self,
        profile: MobilityProfile,
        costing: &RouteCostingOptions,
        allow_destination_only: bool,
        sar: Option<SarEdgeRelaxations>,
    ) -> ColumnHandle {
        let view = self.view();
        let mut costing = costing.normalized();
        // Conditional tags are the request's sparse overrides, not the
        // column's: requests at different times share one column.
        costing.conditional_restriction_eval_at_ms = None;
        let key = ColumnKey {
            profile: canonical(profile),
            costing,
            allow_destination_only,
            sar,
        };
        if allow_destination_only
            && sar.is_none()
            && !self.recost
            && costing == RouteCostingOptions::default().normalized()
            && let Some(column) = self.file().column_for(profile)
        {
            return ColumnHandle::Stored {
                column,
                heuristic_cost_per_m: self.file().heuristic_cost_per_m(profile),
                junction: JunctionDelay::for_profile(profile),
            };
        }
        if let Some(column) = self.derived_columns.get(&key) {
            return ColumnHandle::Derived(column);
        }
        let mut derived = derive_column(
            &view,
            key,
            self.file().column_for(landmark_profile(profile)),
        );
        if wants_own_landmarks(&derived, key)
            && let Some(own) = self.profile_landmarks(&view, key)
        {
            derived.adopt(&view, key, own);
        }
        if crate::debug::trace() {
            eprintln!(
                "compact: derived {:?} column in {:.1} ms (landmark scale {:?}, own landmarks {}, k={:.4})",
                key.profile,
                derived.build_ms,
                derived.landmark_scale,
                derived.own_landmarks.is_some(),
                derived.heuristic_cost_per_m
            );
        }
        ColumnHandle::Derived(self.derived_columns.insert(key, Arc::new(derived)))
    }

    /// The landmarks of `key`'s metric, measured on a miss.
    fn profile_landmarks(
        &self,
        view: &SearchGraph<'_>,
        key: ColumnKey,
    ) -> Option<Arc<ProfileLandmarks>> {
        let metric = key.metric();
        if let Some(landmarks) = self.profile_landmarks.get(&metric) {
            return landmarks;
        }
        let column = self.file().column_for(landmark_profile(key.profile))?;
        let costs = metric_costs(view, metric);
        let landmarks =
            ProfileLandmarks::build(view, &costs, column, PROFILE_LANDMARK_COUNT).map(Arc::new);
        if crate::debug::trace()
            && let Some(landmarks) = &landmarks
        {
            eprintln!(
                "compact: {:?} landmarks ({}) in {:.1} ms, {:.1} MiB",
                key.profile,
                landmarks.count,
                landmarks.build_ms,
                landmarks.byte_len() as f64 / (1 << 20) as f64
            );
        }
        self.profile_landmarks.insert(metric, landmarks)
    }

    /// Diagnostics for parity harnesses: derives the column for `profile`
    /// under `costing` from the attribute sections (bypassing the cache and
    /// the stored columns) and reports its build time, its landmark scale
    /// and, when the profile has a stored column for these options, how many
    /// edges' f32 cost or duration bits differ from it.
    #[doc(hidden)]
    pub fn derived_column_probe(
        &self,
        profile: MobilityProfile,
        costing: &RouteCostingOptions,
    ) -> Option<DerivedColumnProbe> {
        let view = self.view();
        let key = ColumnKey {
            profile: canonical(profile),
            costing: costing.normalized(),
            allow_destination_only: true,
            sar: None,
        };
        let mut derived = derive_column(
            &view,
            key,
            self.file().column_for(landmark_profile(profile)),
        );
        let stored_scale = derived.landmark_scale;
        let mut own = None;
        if wants_own_landmarks(&derived, key)
            && let Some(landmarks) = self.profile_landmarks(&view, key)
        {
            own = Some((landmarks.count, landmarks.build_ms, landmarks.byte_len()));
            derived.adopt(&view, key, landmarks);
        }
        let stored = (key.costing == RouteCostingOptions::default().normalized())
            .then(|| self.file().column_for(profile))
            .flatten()
            .map(|column| {
                let range = column * view.edge_count..(column + 1) * view.edge_count;
                let differs = |stored: &[f32], derived: &[f32]| {
                    stored
                        .iter()
                        .zip(derived)
                        .filter(|(a, b)| a.to_bits() != b.to_bits())
                        .count()
                };
                differs(&view.edge_cost[range.clone()], &derived.cost)
                    + differs(&view.edge_duration[range], &derived.duration)
            });
        Some(DerivedColumnProbe {
            build_ms: derived.build_ms,
            landmark_scale: derived.landmark_scale,
            stored_landmark_scale: stored_scale,
            own_landmarks: own,
            heuristic_cost_per_m: derived.heuristic_cost_per_m,
            usable_edge_count: derived.cost.iter().filter(|cost| cost.is_finite()).count(),
            stored_mismatch_count: stored,
        })
    }
}

/// See [`RoutingPackage::derived_column_probe`].
#[doc(hidden)]
#[derive(Debug, Clone)]
pub struct DerivedColumnProbe {
    pub build_ms: f64,
    pub landmark_scale: Option<f64>,
    /// The scale of the stored column's landmarks, before any own landmarks.
    pub stored_landmark_scale: Option<f64>,
    /// Own landmarks (`profile_landmarks`): count, the time their first use
    /// took to build them (ms), and their heap bytes.
    pub own_landmarks: Option<(usize, f64, usize)>,
    pub heuristic_cost_per_m: f64,
    pub usable_edge_count: usize,
    /// Cost or duration values differing from the stored column (default
    /// options on a profile with a stored column only).
    pub stored_mismatch_count: Option<usize>,
}

impl SearchGraph<'_> {
    /// The costing inputs of `edge` from the attribute sections, as seen by
    /// `profile` (whose mode mask is `profile_mask`), without its conditional
    /// tags (edges carrying them are listed in `conditional_edges`; see
    /// [`Self::conditional_restrictions`]).
    #[inline]
    pub(crate) fn cost_inputs(
        &self,
        edge: u32,
        profile_bit: u8,
        profile_mask: wayfarer_package::ModeMask,
    ) -> EdgeCostInputs<'static> {
        let index = edge as usize;
        let tuple = self.cost_tuple(self.edge_shared_info[index] as usize);
        let speed = |value: f32| (!value.is_nan()).then_some(value);
        let inferred = self.edge_inferred_speed[index];
        EdgeCostInputs {
            edge_id: self.edge_id(index),
            distance_m: self.edge_length[index],
            // Only `mask & profile_mask != 0` is ever tested, which the
            // stored access bit records exactly.
            mobility_mode_mask: if self.edge_profile_access[index] & profile_bit != 0 {
                profile_mask
            } else {
                0
            },
            inferred_speed_kph: (inferred != 0).then(|| inferred - 1),
            road_type: road_type_from_rank(self.road_rank[index]).unwrap_or(RoadType::Other),
            flags: EdgeCostFlags(self.si_cost_flags[tuple]),
            classes: EdgeCostClasses::unpack(self.si_cost_classes[tuple]),
            maxspeed_kph: speed(self.si_speeds[tuple * 3]),
            free_flow_speed_kph: speed(self.si_speeds[tuple * 3 + 1]),
            constrained_speed_kph: speed(self.si_speeds[tuple * 3 + 2]),
            conditional_restrictions: &[],
            sar_access: self.edge_profile_access[index] & SAR_ACCESS_BITS,
            legal_reason: self.legal_reason(self.edge_shared_info[index] as usize).0,
        }
    }

    /// The Wheeled landmark metric's cost of `edge` where the stored column
    /// cannot use it (`SarPackageInfo`): its default cost with legal access
    /// times the minimum factors of the relaxations it needs, as the
    /// compiler computed it. `None` when the edge is not physically
    /// drivable.
    pub(crate) fn sar_metric_cost(&self, edge: u32, policy: &CostingPolicy<'_>) -> Option<f64> {
        let sar = self.sar;
        let mask = MobilityProfile::Wheeled.mobility_mode_mask();
        let mut inputs = self.cost_inputs(edge, profile_bit(MobilityProfile::Wheeled), mask);
        let factor = sar.landmark_factor(inputs.sar_facts())?;
        inputs.mobility_mode_mask = mask;
        match policy.edge_cost_inputs(&inputs) {
            EdgeCostDecision::Allowed(costed) => Some(f64::from(
                wayfarer_package::sar::sar_metric_cost(costed.cost, factor),
            )),
            _ => None,
        }
    }

    /// The request's costing of `edge` from the package's raw costing inputs,
    /// conditional tags included: exactly the costing of the road graph's
    /// record under `policy` (for a profile with `profile_bit` and mode mask
    /// `profile_mask`).
    pub(crate) fn edge_cost_decision(
        &self,
        policy: &CostingPolicy<'_>,
        edge: u32,
        profile_bit: u8,
        profile_mask: wayfarer_package::ModeMask,
    ) -> EdgeCostDecision {
        let conditional = if self.conditional_edges.binary_search(&edge).is_ok() {
            self.conditional_restrictions(self.edge_shared_info[edge as usize])
        } else {
            Vec::new()
        };
        let mut inputs: EdgeCostInputs<'_> = self.cost_inputs(edge, profile_bit, profile_mask);
        inputs.conditional_restrictions = &conditional;
        policy.edge_cost_inputs(&inputs)
    }
}

#[derive(Clone, Copy)]
struct ColumnStats {
    /// min derived / base over edges usable in the derived column.
    ratio: f64,
    /// Every edge usable in the derived column is usable in the base column.
    subset: bool,
    /// min cost per metre of span.
    cost_per_m: f64,
}

impl ColumnStats {
    const IDENTITY: Self = Self {
        ratio: f64::INFINITY,
        subset: true,
        cost_per_m: f64::INFINITY,
    };

    fn merge(self, other: Self) -> Self {
        Self {
            ratio: self.ratio.min(other.ratio),
            subset: self.subset && other.subset,
            cost_per_m: self.cost_per_m.min(other.cost_per_m),
        }
    }
}

pub(super) fn derive_column(
    view: &SearchGraph<'_>,
    key: ColumnKey,
    landmark_column: Option<usize>,
) -> DerivedColumn {
    let started = Instant::now();
    let options = RouteOptions {
        costing: key.costing,
        ..RouteOptions::default()
    };
    let policy = CostingPolicy::with_destination_only_policy(
        key.profile,
        &options,
        key.allow_destination_only,
    )
    .with_sar_relaxations(key.sar);
    let bit = profile_bit(key.profile);
    let mask = key.profile.mobility_mode_mask();
    let edge_count = view.edge_count;
    let mut cost = vec![f32::INFINITY; edge_count];
    let mut duration = vec![f32::INFINITY; edge_count];
    const CHUNK: usize = 1 << 14;
    cost.par_chunks_mut(CHUNK)
        .zip(duration.par_chunks_mut(CHUNK))
        .enumerate()
        .for_each(|(chunk, (costs, durations))| {
            let first = chunk * CHUNK;
            for (offset, (cost, duration)) in costs.iter_mut().zip(durations).enumerate() {
                let inputs = view.cost_inputs((first + offset) as u32, bit, mask);
                if let EdgeCostDecision::Allowed(costed) = policy.edge_cost_inputs(&inputs) {
                    *cost = costed.cost as f32;
                    *duration = costed.duration_s as f32;
                }
            }
        });
    if key.costing.conditional_restriction_eval_at_ms.is_some() {
        for edge in view.conditional_edges {
            let (edge_cost, edge_duration) = match view
                .edge_cost_decision(&policy, *edge, bit, mask)
            {
                EdgeCostDecision::Allowed(costed) => (costed.cost as f32, costed.duration_s as f32),
                _ => (f32::INFINITY, f32::INFINITY),
            };
            cost[*edge as usize] = edge_cost;
            duration[*edge as usize] = edge_duration;
        }
    }

    let base = landmark_column
        .filter(|_| view.landmark_count > 0)
        .map(|column| &view.edge_cost[column * edge_count..(column + 1) * edge_count]);
    // The Wheeled landmarks were built on the landmark metric (`SarPackageInfo`):
    // where the stored column cannot use an edge, the metric's cost is the
    // bound's base, so a SAR column keeps its landmarks.
    let metric_options = RouteOptions::default();
    let metric_policy = CostingPolicy::new(MobilityProfile::Wheeled, &metric_options);
    let metric =
        (landmark_profile(key.profile) == MobilityProfile::Wheeled).then_some(&metric_policy);
    let stats = cost
        .par_chunks(CHUNK)
        .enumerate()
        .map(|(chunk, costs)| {
            let mut stats = ColumnStats::IDENTITY;
            for (offset, cost) in costs.iter().enumerate() {
                if !cost.is_finite() {
                    continue;
                }
                let edge = chunk * CHUNK + offset;
                let cost = f64::from(*cost);
                if let Some(base) = base {
                    let mut base = f64::from(base[edge]);
                    if !base.is_finite()
                        && let Some(policy) = metric
                    {
                        base = view
                            .sar_metric_cost(edge as u32, policy)
                            .unwrap_or(f64::INFINITY);
                    }
                    if !base.is_finite() {
                        stats.subset = false;
                    } else if base > 0.0 {
                        stats.ratio = stats.ratio.min(cost / base);
                    }
                }
                let span = f64::from(view.edge_length[edge]).max(chord_m(
                    view.xyz(view.edge_source(edge)),
                    view.xyz(view.edge_target[edge]),
                ));
                if span > 0.0 {
                    stats.cost_per_m = stats.cost_per_m.min(cost / span);
                }
            }
            stats
        })
        .reduce(|| ColumnStats::IDENTITY, ColumnStats::merge);

    // Same rule as the stored columns' factor (see `build_package`).
    let heuristic_cost_per_m = if stats.cost_per_m.is_finite() {
        stats.cost_per_m * 0.999
    } else {
        0.0
    };
    let landmark_scale = base
        .filter(|_| stats.subset)
        .map(|_| {
            if stats.ratio.is_finite() {
                stats.ratio * LANDMARK_SCALE_MARGIN
            } else {
                // No usable edge: nothing is reachable, any scale is valid.
                1.0
            }
        })
        .filter(|scale| *scale > 0.0);
    let landmark_scales = landmark_scale
        .map(|scale| {
            view.landmark_scale
                .iter()
                .map(|quantum| (f64::from(*quantum) * scale) as f32)
                .collect()
        })
        .unwrap_or_default();
    // PERF-2 spike: the stored column itself (not the landmark metric) as
    // the CCH potential's base: valid when every usable edge is usable there.
    let potential = landmark_column.zip(base).and_then(|(column, base)| {
        let (ratio, subset) = cost
            .par_iter()
            .zip(base)
            .map(|(cost, base)| {
                if !cost.is_finite() {
                    (f64::INFINITY, true)
                } else if !base.is_finite() {
                    (f64::INFINITY, false)
                } else if *base > 0.0 {
                    (f64::from(*cost) / f64::from(*base), true)
                } else {
                    (f64::INFINITY, true)
                }
            })
            .reduce(|| (f64::INFINITY, true), |a, b| (a.0.min(b.0), a.1 && b.1));
        let factor = if ratio.is_finite() {
            ratio * LANDMARK_SCALE_MARGIN
        } else {
            1.0
        };
        (subset && factor > 0.0).then_some((column, factor))
    });
    DerivedColumn {
        potential,
        duration,
        landmark_column: landmark_scale.and(landmark_column),
        landmark_scales,
        own_landmarks: None,
        heuristic_cost_per_m,
        junction: JunctionDelay::for_profile(key.profile),
        landmark_scale,
        marks: landmark_column.and_then(|column| ColumnMarks::new(view, &cost, column)),
        build_ms: started.elapsed().as_secs_f64() * 1e3,
        cost,
    }
}

/// Whether `derived` should get landmarks of its own metric: its stored
/// landmarks would be scaled below [`OWN_LANDMARKS_BELOW_SCALE`], or it is an
/// R5 column, which the stored landmarks cannot bound at all (paths are not
/// in their metric).
fn wants_own_landmarks(derived: &DerivedColumn, key: ColumnKey) -> bool {
    match derived.landmark_scale {
        Some(scale) => scale < OWN_LANDMARKS_BELOW_SCALE,
        None => key.metric().paths,
    }
}

/// The SAR relaxations of a metric (see [`metric_costs`]): R1 with every
/// denying value and R2, at the package's minimum factors, for a vehicle
/// profile; and for an R5 metric, every path
/// class at every width at R5's minimum factor, so that any R5 column costs
/// each edge at least this. Without the paths, R5 stays out, as it does of
/// the stored landmarks.
fn metric_policy_sar(view: &SearchGraph<'_>, metric: MetricKey) -> Option<SarEdgeRelaxations> {
    let sar = view.sar;
    let profile = metric.profile;
    (landmark_profile(profile) == MobilityProfile::Wheeled).then(|| SarEdgeRelaxations {
        legal_access: Some((
            LegalValue::ALL
                .into_iter()
                .filter(|value| value.denies())
                .fold(0u16, |bits, value| bits | value.bit()),
            sar.legal_access_min_factor,
        )),
        wrong_way: Some(sar.wrong_way_min_factor),
        vehicle_on_paths: metric.paths.then(|| SarPathRelaxation {
            classes: SarPathClass::ALL
                .into_iter()
                .fold(0, |bits, class| bits | class.bit()),
            widths: SarPathRelaxation::width_bits(0.0, true),
            factor: sar.vehicle_on_paths_min_factor,
        }),
    })
}

/// The metric of a profile's own landmarks (see `profile_landmarks`): the
/// profile's costs under its options with destination-only edges allowed,
/// and, for a vehicle profile, every edge the
/// SAR relaxations R1 and R2 can open at its cost times their package
/// minimum factors (and for an R5 metric every path too, at R5's). Every
/// column of the profile and options costs each edge at least this.
fn metric_costs(view: &SearchGraph<'_>, metric: MetricKey) -> Vec<f32> {
    let options = RouteOptions {
        costing: metric.costing,
        ..RouteOptions::default()
    };
    let policy = CostingPolicy::with_destination_only_policy(metric.profile, &options, true)
        .with_sar_relaxations(metric_policy_sar(view, metric));
    let bit = profile_bit(metric.profile);
    let mask = metric.profile.mobility_mode_mask();
    (0..view.edge_count as u32)
        .into_par_iter()
        .map(
            |edge| match policy.edge_cost_inputs(&view.cost_inputs(edge, bit, mask)) {
                EdgeCostDecision::Allowed(costed) => costed.cost as f32,
                _ => f32::INFINITY,
            },
        )
        .collect()
}

impl DerivedColumn {
    /// Whether the column is bounded by landmarks of its own metric.
    #[cfg(test)]
    pub(crate) fn has_own_landmarks(&self) -> bool {
        self.own_landmarks.is_some()
    }

    /// Bounds this column with `own`, its metric's landmarks, when they are
    /// valid for it (every edge it can use is in the metric) and scale less.
    fn adopt(&mut self, view: &SearchGraph<'_>, key: ColumnKey, own: Arc<ProfileLandmarks>) {
        let metric = metric_costs(view, key.metric());
        let (ratio, subset) = self
            .cost
            .par_iter()
            .zip(&metric)
            .map(|(cost, base)| {
                if !cost.is_finite() {
                    (f64::INFINITY, true)
                } else if !base.is_finite() {
                    (f64::INFINITY, false)
                } else if *base > 0.0 {
                    (f64::from(*cost) / f64::from(*base), true)
                } else {
                    (f64::INFINITY, true)
                }
            })
            .reduce(|| (f64::INFINITY, true), |a, b| (a.0.min(b.0), a.1 && b.1));
        if !subset {
            return;
        }
        let scale = if ratio.is_finite() {
            ratio * LANDMARK_SCALE_MARGIN
        } else {
            1.0
        };
        if scale <= self.landmark_scale.unwrap_or(0.0) {
            return;
        }
        self.landmark_scales = own
            .scales
            .iter()
            .map(|quantum| (f64::from(*quantum) * scale) as f32)
            .collect();
        self.landmark_column = Some(own.column);
        self.landmark_scale = Some(scale);
        self.own_landmarks = Some(own);
    }
}

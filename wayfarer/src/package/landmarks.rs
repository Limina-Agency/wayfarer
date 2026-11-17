//! ALT (A*, landmarks, triangle inequality) lower bounds.
//!
//! For each profile column a small set of landmark nodes is chosen near the
//! edge of the graph, and node-level travel costs from and to every landmark
//! are stored (turn costs ignored, which keeps them lower bounds). Per-request
//! blocks and penalties only increase costs, so the bounds stay admissible for
//! them as well.
//!
//! Distances are quantized per (column, landmark, direction) with a scale
//! `q`: `u32` whole quanta over arcs of `floor(c / q)` (see the compiler's
//! `package_build::landmarks`), clamped at `LANDMARK_SATURATED`. Every term
//! `q * (D(L, t) - D(L, n))` or `q * (D(n, L) - D(t, L))` is then admissible
//! and consistent as it stands, saturated values included, so the
//! bidirectional stop is exact. A node outside a column's landmark network
//! has no stored row (see `LandmarkRows`) and reads as unreachable.

use wayfarer_package::format::{LANDMARK_INFINITY, LandmarkRow, LandmarkTable};

/// Landmarks a walking query bounds with (best bound at the origin).
/// Vehicle queries use every stored landmark: on Małopolskie all 16 cut a
/// plain drive's visited edges by about a third (the corpus's p95 from 81k
/// to 45k), while walks got slower with them (p95 69 to 102 ms).
pub(crate) const WALKING_LANDMARKS: usize = 6;

/// How many of a column's `count` landmarks a query bounds with.
pub(crate) fn active_landmarks(walking: bool, count: usize) -> usize {
    if walking {
        WALKING_LANDMARKS.min(count)
    } else {
        count
    }
}

/// Per-query ALT bound towards a set of target nodes: the minimum over
/// targets of the best landmark bound for that target. (Aggregating targets
/// per landmark first lets one awkward candidate - e.g. on the far carriageway
/// - erase every landmark's information.)
pub(crate) struct AltBound<'g> {
    from: LandmarkTable<'g>,
    to: LandmarkTable<'g>,
    column: usize,
    /// Active landmarks: (landmark, from scale, to scale).
    active: Vec<(usize, f64, f64)>,
    /// `[active][target]` (d(L, t), d(t, L)) in stored units.
    target_values: Vec<(u32, u32)>,
    target_count: usize,
}

impl<'g> AltBound<'g> {
    /// `reversed` bounds d(targets -> node) instead of d(node -> targets):
    /// the same formulas with the from/to tables (and their scales) swapped.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new_directed(
        from: LandmarkTable<'g>,
        to: LandmarkTable<'g>,
        scales: &[f32],
        landmark_count: usize,
        active: usize,
        node_count: usize,
        column: usize,
        targets: &[u32],
        origins: &[u32],
        reversed: bool,
    ) -> Option<Self> {
        if !reversed {
            return Self::new(
                from,
                to,
                scales,
                landmark_count,
                active,
                node_count,
                column,
                targets,
                origins,
            );
        }
        let swapped = scales
            .as_chunks::<2>()
            .0
            .iter()
            .flat_map(|pair| [pair[1], pair[0]])
            .collect::<Vec<_>>();
        Self::new(
            to,
            from,
            &swapped,
            landmark_count,
            active,
            node_count,
            column,
            targets,
            origins,
        )
    }

    /// Bounds with the `active` landmarks that bound `origins` best.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        from: LandmarkTable<'g>,
        to: LandmarkTable<'g>,
        scales: &[f32],
        landmark_count: usize,
        active: usize,
        node_count: usize,
        column: usize,
        targets: &[u32],
        origins: &[u32],
    ) -> Option<Self> {
        // Per-target accumulators are fixed-size; a bound over a subset of
        // targets would not be admissible, so give up rather than truncate.
        if landmark_count == 0 || targets.is_empty() || targets.len() > 16 {
            return None;
        }
        // The tables know their own row layout; `node_count` only checks the
        // targets.
        debug_assert!(targets.iter().all(|target| (*target as usize) < node_count));
        let all = Self::with_landmarks(
            from,
            to,
            scales,
            landmark_count,
            column,
            targets,
            (0..landmark_count).collect(),
        );
        // Keep the landmarks that bound the origin best.
        let mut scored = (0..landmark_count)
            .map(|index| {
                let score = origins
                    .iter()
                    .map(|origin| all.bound_with(&[index], *origin))
                    .fold(f64::INFINITY, f64::min);
                (score, index)
            })
            .collect::<Vec<_>>();
        scored.sort_by(|a, b| b.0.total_cmp(&a.0));
        let chosen = scored
            .into_iter()
            .take(active)
            .map(|(_, index)| index)
            .collect::<Vec<_>>();
        Some(Self::with_landmarks(
            from,
            to,
            scales,
            landmark_count,
            column,
            targets,
            chosen,
        ))
    }

    fn with_landmarks(
        from: LandmarkTable<'g>,
        to: LandmarkTable<'g>,
        scales: &[f32],
        landmark_count: usize,
        column: usize,
        targets: &[u32],
        landmarks: Vec<usize>,
    ) -> Self {
        let rows = from.rows();
        let target_rows = targets
            .iter()
            .map(|target| rows.row(column, *target))
            .collect::<Vec<_>>();
        let active = landmarks
            .iter()
            .map(|landmark| {
                let scale_index = (column * landmark_count + landmark) * 2;
                (
                    *landmark,
                    f64::from(scales[scale_index]),
                    f64::from(scales[scale_index + 1]),
                )
            })
            .collect::<Vec<_>>();
        let target_values = landmarks
            .iter()
            .flat_map(|landmark| {
                target_rows
                    .iter()
                    .map(move |row| (from.get(*row, *landmark), to.get(*row, *landmark)))
            })
            .collect();
        Self {
            from,
            to,
            column,
            active,
            target_values,
            target_count: targets.len(),
        }
    }

    /// Lower bound from `node` to the closest target using the active
    /// landmarks at positions `which` (all when empty slice is not used).
    fn bound_with(&self, which: &[usize], node: u32) -> f64 {
        let mut per_target = [0.0f64; 16];
        let targets = self.target_count.min(per_target.len());
        let row = self.from.rows().row(self.column, node);
        for position in which {
            self.accumulate(*position, row, &mut per_target[..targets]);
        }
        per_target[..targets]
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min)
    }

    #[inline]
    fn accumulate(&self, position: usize, row: LandmarkRow, per_target: &mut [f64]) {
        let (landmark, from_scale, to_scale) = self.active[position];
        let (d_ln, d_nl) = (self.from.get(row, landmark), self.to.get(row, landmark));
        let values = &self.target_values[position * self.target_count..];
        for (slot, (d_lt, d_tl)) in per_target.iter_mut().zip(values) {
            let mut term = 0.0f64;
            // d(n, t) >= d(L, t) - d(L, n)
            if *d_lt == LANDMARK_INFINITY {
                if d_ln != LANDMARK_INFINITY {
                    // L reaches n but not t: n cannot reach t.
                    term = f64::INFINITY;
                }
            } else if d_ln != LANDMARK_INFINITY && *d_lt > d_ln {
                term = f64::from(*d_lt - d_ln) * from_scale;
            }
            // d(n, t) >= d(n, L) - d(t, L)
            if *d_tl != LANDMARK_INFINITY {
                if d_nl == LANDMARK_INFINITY {
                    // t reaches L but n does not: n cannot reach t.
                    term = f64::INFINITY;
                } else if d_nl > *d_tl {
                    term = term.max(f64::from(d_nl - *d_tl) * to_scale);
                }
            }
            if term > *slot {
                *slot = term;
            }
        }
    }

    #[inline]
    pub(crate) fn bound(&self, node: u32) -> f64 {
        let mut per_target = [0.0f64; 16];
        let targets = self.target_count.min(per_target.len());
        let row = self.from.rows().row(self.column, node);
        for position in 0..self.active.len() {
            self.accumulate(position, row, &mut per_target[..targets]);
        }
        per_target[..targets]
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min)
    }
}

/// One landmark's distances aggregated over a group of nodes.
#[derive(Clone, Copy)]
enum Aggregate {
    /// For `d(t, L)`: some node does not reach the landmark, so the group
    /// gives no term.
    NoInfo,
    /// No node of the group is reachable from the landmark.
    Unreachable,
    Value(u32),
}

/// Lower bounds towards groups of target nodes, each group with an offset:
/// `min over groups g of bound(n, g) + offset(g)`, where `bound(n, g)` uses,
/// per active landmark, the least `d(L, t)` and the greatest `d(t, L)` over
/// the group's nodes `t`. `q * (min d(L, t) - d(L, n))` is below every
/// node's term `q * (d(L, t) - d(L, n))`, and likewise for the other
/// direction, so the bound is admissible for every node of the group; each
/// term is consistent as the per-target terms are (see [`AltBound`]). A
/// group costs one term per landmark whatever its size, and loses about its
/// diameter against per-node bounds: for spatially tight groups of many
/// targets (the discount gates of `search::gates`).
pub(crate) struct GroupBound<'g> {
    from: LandmarkTable<'g>,
    to: LandmarkTable<'g>,
    column: usize,
    active: Vec<(usize, f64, f64)>,
    /// `[group][active]`: (aggregated d(L, t), aggregated d(t, L)).
    values: Vec<(Aggregate, Aggregate)>,
    group_count: usize,
    /// Per group; groups with a finite offset are searched in `order`.
    offsets: Vec<f64>,
    order: Vec<usize>,
}

impl<'g> GroupBound<'g> {
    /// `reversed` bounds d(groups -> node), as [`AltBound::new_directed`].
    /// The `active` landmarks are the ones that bound `origins` best towards
    /// the closest group; offsets start at 0 (see [`Self::set_offsets`]).
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        from: LandmarkTable<'g>,
        to: LandmarkTable<'g>,
        scales: &[f32],
        landmark_count: usize,
        active: usize,
        column: usize,
        groups: &[Vec<u32>],
        origins: &[u32],
        reversed: bool,
    ) -> Option<Self> {
        if landmark_count == 0 || groups.is_empty() || groups.iter().any(Vec::is_empty) {
            return None;
        }
        let (from, to, scales) = if reversed {
            let swapped = scales
                .as_chunks::<2>()
                .0
                .iter()
                .flat_map(|pair| [pair[1], pair[0]])
                .collect::<Vec<_>>();
            (to, from, swapped)
        } else {
            (from, to, scales.to_vec())
        };
        let all = Self::with_landmarks(
            from,
            to,
            &scales,
            landmark_count,
            column,
            groups,
            (0..landmark_count).collect(),
        );
        let mut scored = (0..landmark_count)
            .map(|index| {
                let score = origins
                    .iter()
                    .map(|origin| {
                        let row = all.from.rows().row(column, *origin);
                        (0..groups.len())
                            .map(|group| all.term(group, index, row))
                            .fold(f64::INFINITY, f64::min)
                    })
                    .fold(f64::INFINITY, f64::min);
                (score, index)
            })
            .collect::<Vec<_>>();
        scored.sort_by(|a, b| b.0.total_cmp(&a.0));
        let chosen = scored
            .into_iter()
            .take(active)
            .map(|(_, index)| index)
            .collect();
        Some(Self::with_landmarks(
            from,
            to,
            &scales,
            landmark_count,
            column,
            groups,
            chosen,
        ))
    }

    fn with_landmarks(
        from: LandmarkTable<'g>,
        to: LandmarkTable<'g>,
        scales: &[f32],
        landmark_count: usize,
        column: usize,
        groups: &[Vec<u32>],
        landmarks: Vec<usize>,
    ) -> Self {
        let rows = from.rows();
        let active = landmarks
            .iter()
            .map(|landmark| {
                let scale_index = (column * landmark_count + landmark) * 2;
                (
                    *landmark,
                    f64::from(scales[scale_index]),
                    f64::from(scales[scale_index + 1]),
                )
            })
            .collect::<Vec<_>>();
        let mut values = Vec::with_capacity(groups.len() * landmarks.len());
        for group in groups {
            let group_rows = group
                .iter()
                .map(|node| rows.row(column, *node))
                .collect::<Vec<_>>();
            for landmark in &landmarks {
                // Least d(L, t): unreachable nodes drop out (their own term
                // is infinite wherever L reaches n).
                let mut least = Aggregate::Unreachable;
                // Greatest d(t, L): every node must reach L with a value.
                let mut greatest = Aggregate::Value(0);
                for row in &group_rows {
                    let (d_lt, d_tl) = (from.get(*row, *landmark), to.get(*row, *landmark));
                    least = match least {
                        Aggregate::NoInfo => Aggregate::NoInfo,
                        _ if d_lt == LANDMARK_INFINITY => least,
                        Aggregate::Unreachable => Aggregate::Value(d_lt),
                        Aggregate::Value(value) => Aggregate::Value(value.min(d_lt)),
                    };
                    greatest = match greatest {
                        _ if d_tl == LANDMARK_INFINITY => Aggregate::NoInfo,
                        Aggregate::Value(value) => Aggregate::Value(value.max(d_tl)),
                        other => other,
                    };
                }
                values.push((least, greatest));
            }
        }
        Self {
            from,
            to,
            column,
            active,
            values,
            group_count: groups.len(),
            offsets: vec![0.0; groups.len()],
            order: (0..groups.len()).collect(),
        }
    }

    /// The term of active landmark `position` for `group` at a node's row.
    #[inline]
    fn term(&self, group: usize, position: usize, row: LandmarkRow) -> f64 {
        let (landmark, from_scale, to_scale) = self.active[position];
        let (d_ln, d_nl) = (self.from.get(row, landmark), self.to.get(row, landmark));
        let (least, greatest) = self.values[group * self.active.len() + position];
        let mut term = 0.0f64;
        match least {
            Aggregate::Unreachable if d_ln != LANDMARK_INFINITY => term = f64::INFINITY,
            Aggregate::Value(d_lt) if d_ln != LANDMARK_INFINITY && d_lt > d_ln => {
                term = f64::from(d_lt - d_ln) * from_scale;
            }
            _ => {}
        }
        if let Aggregate::Value(d_tl) = greatest {
            if d_nl == LANDMARK_INFINITY {
                term = f64::INFINITY;
            } else if d_nl > d_tl {
                term = term.max(f64::from(d_nl - d_tl) * to_scale);
            }
        }
        term
    }

    /// The bound from `node` to `group`, without its offset.
    pub(crate) fn group_bound(&self, group: usize, node: u32) -> f64 {
        let row = self.from.rows().row(self.column, node);
        (0..self.active.len())
            .map(|position| self.term(group, position, row))
            .fold(0.0, f64::max)
    }

    pub(crate) fn group_count(&self) -> usize {
        self.group_count
    }

    /// Sets each group's offset; groups with an infinite one are dropped.
    pub(crate) fn set_offsets(&mut self, offsets: Vec<f64>) {
        let mut order = (0..self.group_count)
            .filter(|group| offsets[*group].is_finite())
            .collect::<Vec<_>>();
        order.sort_by(|a, b| offsets[*a].total_cmp(&offsets[*b]).then(a.cmp(b)));
        self.offsets = offsets;
        self.order = order;
    }

    /// `min(cap, min over groups of bound(node, group) + offset(group))`, or
    /// any value at or below `enough` once the minimum reaches it (the
    /// caller takes the larger of the result and `enough`).
    #[inline]
    pub(crate) fn bound_below(&self, node: u32, cap: f64, enough: f64) -> f64 {
        let row = self.from.rows().row(self.column, node);
        let mut value = cap;
        for group in &self.order {
            let offset = self.offsets[*group];
            // Bounds are never negative: no later group can do better.
            if offset >= value || value <= enough {
                break;
            }
            let bound = (0..self.active.len())
                .map(|position| self.term(*group, position, row))
                .fold(0.0, f64::max);
            value = value.min(bound + offset);
        }
        value
    }
}

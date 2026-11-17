//! Discount gates: exact landmark bounds for requests that make edges cheaper
//! than the column the landmarks were computed on (discounts, fast speed
//! overrides, opened edges, discounted overlay fragments).
//!
//! The landmark bound `ALT(v, T)` is a lower bound on the column cost from `v`
//! to the targets `T`, and it stays one for any path whose edges cost at
//! least their column cost. A request edge `e = (u, w)` that costs `c(e)`
//! below its column cost (or is usable where the column is not) is a *gate*:
//! every path to the targets either uses no gate, and costs at least
//! `ALT(v, T)`, or reaches a first gate `e` at its source `u` through
//! column-costed edges. So
//!
//! `pi(v) = min(ALT(v, T), min over gates e of ALT(v, u) + off(e))`
//!
//! with `off(e) = c(e) + pi(w)` is a lower bound. It is consistent: each term
//! is a consistent landmark bound plus a constant (the minimum of consistent
//! bounds is consistent), and on a gate `pi(u) <= ALT(u, u) + off(e) =
//! c(e) + pi(w)`. The offsets solve that system as a shortest path problem
//! over gate clusters (Bellman-Ford, as for the overlay patch's connector
//! gates, see `patch::Bounds`, which takes discount gates the same way).
//!
//! Gate nodes are grouped into clusters of up to [`GATE_CLUSTER_NODES`] nodes
//! in compact (Hilbert, so spatial) order. A cluster's term is a landmark
//! bound towards all its nodes at once, from the least `d(L, t)` and the
//! greatest `d(t, L)` over them (`landmarks::GroupBound`), plus the least
//! offset of its gates: below every member's term, so the bound stays a
//! consistent lower bound, flatter by about a cluster's extent. A cluster
//! costs one term per active landmark, so a bound evaluation grows with the
//! cluster count, not the gate count. More than [`MAX_GATE_CLUSTERS`]
//! clusters on a side (a long discounted corridor, a whole region) fall back
//! to scaling every bound by the least cost ratio
//! (`analysis::override_bounds`).
//!
//! When every gate is an edge the column can use, each edge costs at least
//! `r` times its column cost (`r` the least gate cost ratio), so `r * ALT`
//! is a consistent lower bound as well, and [`GatedAlt`] takes the larger of
//! the two (the maximum of consistent bounds is consistent): a mild discount
//! on the route itself is bounded as well as by scaling.
//!
//! The backward side (bounds on the cost from the origins) mirrors this with
//! reversed landmarks: a path's *last* gate `e` is left at its target `w`,
//! with `off(e) = pi_b(u) + c(e)`.

use rustc_hash::FxHashMap;

use crate::package::SearchGraph;
use crate::package::columns::{CostColumn, LandmarkColumn};
use crate::package::landmarks::{AltBound, GroupBound};

/// Gate nodes per cluster (measured: 8 or 4 cost more time for no fewer
/// visited edges on the corpus drives).
pub(crate) const GATE_CLUSTER_NODES: usize = 16;
/// Clusters per side beyond which they grow past [`GATE_CLUSTER_NODES`]
/// nodes: a bound evaluation costs one term per cluster and landmark.
pub(crate) const TARGET_GATE_CLUSTERS: usize = 48;
/// Clusters per side before a request falls back to scaled bounds.
pub(crate) const MAX_GATE_CLUSTERS: usize = 128;
/// Gate nodes per side those clusters hold.
pub(crate) const MAX_GATE_NODES: usize = GATE_CLUSTER_NODES * MAX_GATE_CLUSTERS;

#[cfg(test)]
thread_local! {
    static GATE_LIMIT: std::cell::Cell<Option<usize>> = const { std::cell::Cell::new(None) };
}

/// Forces the gate node limit on this thread (tests: `Some(0)` sends every
/// discount to the scaled fallback).
#[cfg(test)]
pub(crate) fn force_gate_limit(limit: Option<usize>) {
    GATE_LIMIT.with(|cell| cell.set(limit));
}

/// The gate node limit per side: [`MAX_GATE_NODES`], or a test's.
pub(crate) fn gate_limit() -> usize {
    #[cfg(test)]
    if let Some(limit) = GATE_LIMIT.with(std::cell::Cell::get) {
        return limit;
    }
    MAX_GATE_NODES
}

/// An edge the request makes cheaper than its column: `cost` is its request
/// cost over the whole edge (a split edge's fragment cost per unit fraction).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct DiscountGate {
    pub edge: u32,
    pub source: u32,
    pub target: u32,
    pub cost: f64,
    /// `cost` over the column cost; 0 where the column cannot use the edge.
    pub ratio: f64,
}

/// The request's gates: edges whose override (`overrides`) or split fragment
/// cost (`split_cost`) is finite and below the column cost, in edge order.
/// `None` when either side has more than `max_nodes` gate nodes (the default,
/// [`MAX_GATE_NODES`], fills [`MAX_GATE_CLUSTERS`] clusters).
pub(crate) fn discount_gates(
    graph: SearchGraph<'_>,
    column: CostColumn<'_>,
    overrides: &FxHashMap<u32, f32>,
    split_cost: &FxHashMap<u32, f32>,
    max_nodes: usize,
) -> Option<Vec<DiscountGate>> {
    let mut gates = overrides
        .iter()
        .chain(split_cost)
        .filter(|(edge, cost)| cost.is_finite() && **cost < column.cost(**edge))
        .map(|(edge, cost)| DiscountGate {
            edge: *edge,
            source: graph.edge_source(*edge as usize),
            target: graph.edge_target[*edge as usize],
            cost: f64::from(*cost),
            ratio: match f64::from(column.cost(*edge)) {
                base if base.is_finite() && base > 0.0 => f64::from(*cost) / base,
                _ => 0.0,
            },
        })
        .collect::<Vec<_>>();
    gates.sort_unstable_by(|a, b| a.edge.cmp(&b.edge).then(a.cost.total_cmp(&b.cost)));
    // A split edge has an infinite override, so it is listed once.
    gates.dedup_by_key(|gate| gate.edge);
    let fits = |node: fn(&DiscountGate) -> u32| {
        let mut nodes = gates.iter().map(node).collect::<Vec<_>>();
        nodes.sort_unstable();
        nodes.dedup();
        nodes.len() <= max_nodes
    };
    (fits(|gate| gate.source) && fits(|gate| gate.target)).then_some(gates)
}

/// The gate node of `gate` on one side, and the node its offset continues
/// from: forward, its source and target; backward, its target and source.
#[inline]
pub(crate) fn gate_ends(gate: &DiscountGate, reversed: bool) -> (u32, u32) {
    if reversed {
        (gate.target, gate.source)
    } else {
        (gate.source, gate.target)
    }
}

/// The gate nodes of one side in compact order, grouped into clusters.
pub(crate) fn gate_clusters(gates: &[DiscountGate], reversed: bool) -> Vec<Vec<u32>> {
    let mut nodes = gates
        .iter()
        .map(|gate| gate_ends(gate, reversed).0)
        .collect::<Vec<_>>();
    nodes.sort_unstable();
    nodes.dedup();
    let size = GATE_CLUSTER_NODES.max(nodes.len().div_ceil(TARGET_GATE_CLUSTERS));
    nodes.chunks(size).map(<[u32]>::to_vec).collect()
}

/// Landmark bounds towards a set of targets (forward) or from a set of
/// origins (backward, `reversed`), with the request's gates.
pub(crate) struct GatedAlt<'g> {
    base: Vec<AltBound<'g>>,
    /// The gate clusters with their offsets.
    clusters: Option<GroupBound<'g>>,
    /// The least gate cost ratio when every gate is an edge the column can
    /// use (else 0): `floor * base` is a lower bound too (every edge costs at
    /// least `floor` times its column cost), tighter than the gates where a
    /// mild discount lies on the path itself.
    floor: f64,
}

impl<'g> GatedAlt<'g> {
    /// `base` bounds the gate-free paths (its minimum over the bounds is
    /// used); `seeds` pick the clusters' active landmarks, as for the base.
    /// `None` when the clusters have no landmark bound.
    pub(crate) fn new(
        landmarks: LandmarkColumn<'g>,
        base: Vec<AltBound<'g>>,
        seeds: &[u32],
        reversed: bool,
        gates: &[DiscountGate],
    ) -> Option<Self> {
        if gates.is_empty() {
            return Some(Self {
                base,
                clusters: None,
                floor: 0.0,
            });
        }
        /// Safety margin on the floor against rounding in f32 weights.
        const MARGIN: f64 = 1.0 - 1e-6;
        let floor = gates
            .iter()
            .map(|gate| gate.ratio)
            .fold(f64::INFINITY, f64::min)
            .min(1.0)
            * MARGIN;
        let clusters = gate_clusters(gates, reversed);
        let mut bound = GroupBound::new(
            landmarks.from,
            landmarks.to,
            landmarks.scales,
            landmarks.count,
            landmarks.active,
            landmarks.column,
            &clusters,
            seeds,
            reversed,
        )?;
        let offsets = gate_offsets(&clusters, gates, reversed, |node| {
            let base = base
                .iter()
                .map(|bound| bound.bound(node))
                .fold(f64::INFINITY, f64::min);
            let terms = (0..bound.group_count())
                .map(|group| bound.group_bound(group, node))
                .collect();
            (base, terms)
        });
        bound.set_offsets(offsets);
        Some(Self {
            base,
            clusters: Some(bound),
            floor,
        })
    }

    /// The lower bound at `node` (infinite: no target is reachable).
    #[inline]
    pub(crate) fn bound(&self, node: u32) -> f64 {
        let base = self
            .base
            .iter()
            .map(|bound| bound.bound(node))
            .fold(f64::INFINITY, f64::min);
        let Some(clusters) = &self.clusters else {
            return base;
        };
        // An infinite base proves the node useless only without opened
        // edges, and then the floor is positive.
        if self.floor > 0.0 {
            let floor = self.floor * base;
            clusters.bound_below(node, base, floor).max(floor)
        } else {
            clusters.bound_below(node, base, 0.0)
        }
    }
}

/// The least offset of each cluster: `off(c) = min over c's gates e of
/// cost(e) + pi(end of e)`, where `pi(n) = min(base(n), min over c' of
/// term(n, c') + off(c'))` and `terms(n)` gives `(base(n), term(n, c') per
/// cluster)`. Every term is non-negative, so a dense Dijkstra over clusters
/// settles them in order of their offsets; unreachable clusters stay
/// infinite.
pub(crate) fn gate_offsets(
    clusters: &[Vec<u32>],
    gates: &[DiscountGate],
    reversed: bool,
    terms: impl Fn(u32) -> (f64, Vec<f64>),
) -> Vec<f64> {
    let slot_of = clusters
        .iter()
        .enumerate()
        .flat_map(|(slot, nodes)| nodes.iter().map(move |node| (*node, slot)))
        .collect::<FxHashMap<_, _>>();
    let mut ends = FxHashMap::<u32, (f64, Vec<f64>)>::default();
    let rows = gates
        .iter()
        .map(|gate| {
            let (node, end) = gate_ends(gate, reversed);
            ends.entry(end).or_insert_with(|| terms(end));
            (slot_of[&node], gate.cost, end)
        })
        .collect::<Vec<_>>();
    let mut offsets = vec![f64::INFINITY; clusters.len()];
    for (slot, cost, end) in &rows {
        offsets[*slot] = offsets[*slot].min(cost + ends[end].0);
    }
    let mut settled = vec![false; clusters.len()];
    while let Some(next) = (0..clusters.len())
        .filter(|slot| !settled[*slot] && offsets[*slot].is_finite())
        .min_by(|a, b| offsets[*a].total_cmp(&offsets[*b]).then(a.cmp(b)))
    {
        settled[next] = true;
        let through = offsets[next];
        for (slot, cost, end) in &rows {
            if !settled[*slot] {
                let value = cost + ends[end].1[next] + through;
                if value < offsets[*slot] {
                    offsets[*slot] = value;
                }
            }
        }
    }
    offsets
}

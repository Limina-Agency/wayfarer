//! Discount gates: edges a request makes cheaper than the column its lower
//! bounds were computed on (discounts, fast speed overrides, opened edges,
//! discounted overlay fragments).
//!
//! A bound `pi(v)` on the column cost from `v` to the targets stays one for
//! any path whose edges cost at least their column cost. A request edge
//! `e = (u, w)` that costs `c(e)` below its column cost (or is usable where
//! the column is not) is a *gate*: every path to the targets either uses no
//! gate, or reaches a first gate at its source `u` through column-costed
//! edges, so the bound becomes the least of `pi(v)` and `d(v, u) + off(e)`
//! over the gates. The CCH potentials take the gates that way
//! (`cch::GatedPotential`), and so do the overlay patch's bounds at virtual
//! points (`patch::Bounds`).
//!
//! The backward side (bounds on the cost from the origins) mirrors this: a
//! path's *last* gate `e` is left at its target `w`.
//!
//! More than [`MAX_GATE_NODES`] gate nodes on a side (a long discounted
//! corridor, a whole region) fall back to scaling every bound by the least
//! cost ratio (`analysis::override_bounds`).

use rustc_hash::FxHashMap;

use crate::package::SearchGraph;
use crate::package::columns::CostColumn;

/// Gate nodes per side before a request falls back to scaled bounds.
pub(crate) const MAX_GATE_NODES: usize = 2048;

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
/// `None` when either side has more than `max_nodes` gate nodes (by default
/// [`MAX_GATE_NODES`]).
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

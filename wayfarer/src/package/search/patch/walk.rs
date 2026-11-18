//! Walks of the patch search through the stored column's clean pieces (see
//! `search::Run::walk_forward`).
//!
//! The patch changes the graph only at the real nodes its arcs start or end
//! at (fragment stops at an edge's ends, connector ends, the seeds' first
//! and the finishes' last stops), and blocks the edges it splits. The pieces
//! holding one of those nodes keep the per-edge search; elsewhere the
//! patched graph is the static one with some edges blocked, which the walks
//! handle, so the static search's walks are exact there too.

use rustc_hash::FxHashSet;
use wayfarer_package::format::{ColumnPieceKinds, PieceKind};

use super::super::{U_TURN_COST, chain_continuation, chain_predecessor, opposing_edge};
use super::{BackwardLabel, Label, NO_LABEL, NO_NODE, PatchCosts, PatchOutcome, PatchRun, RunArcs};
use crate::package::SearchGraph;

/// A patch search's walkable nodes: the column's marked nodes, less the
/// pieces the patch or an endpoint touches.
pub(super) struct PatchWalk<'g> {
    pub kinds: ColumnPieceKinds<'g>,
    /// Marked nodes of the touched pieces.
    pub touched: FxHashSet<u32>,
    /// Real nodes; virtual nodes are never walked.
    node_count: u32,
}

impl<'g> PatchWalk<'g> {
    pub(super) fn new(
        graph: SearchGraph<'_>,
        kinds: ColumnPieceKinds<'g>,
        costs: &PatchCosts<'_>,
        arcs: &RunArcs<'_, '_>,
    ) -> Self {
        let g = &graph;
        let patch = arcs.patch;
        // Every arc's real ends: fragment stops at an edge's ends, connector
        // ends, seeds' first stops and finishes' last ones. Nothing else
        // changes at a real node: a split edge's own arc is blocked, and a
        // virtual point is reached only through arcs.
        let starts = arcs
            .all()
            .flat_map(|arc| [arc.from, arc.to])
            .filter(|node| *node != NO_NODE && *node < patch.node_count)
            .collect::<Vec<_>>();
        // The arcs the column or the request's overrides can use (a strict
        // pass's destination-only pockets).
        let usable =
            |edge: u32| costs.column.cost(edge).is_finite() || (costs.edge_cost)(edge).is_finite();
        let mut touched = FxHashSet::default();
        let mut queue = Vec::new();
        for node in starts {
            if kinds.kind(node) == PieceKind::Other || !touched.insert(node) {
                continue;
            }
            queue.push(node);
            while let Some(node) = queue.pop() {
                let node = node as usize;
                let out = (g.first_out[node]..g.first_out[node + 1])
                    .filter(|edge| usable(*edge))
                    .map(|edge| g.edge_target[edge as usize]);
                let into = g
                    .in_edges(node)
                    .filter(|(edge, _)| usable(*edge))
                    .map(|(_, source)| source);
                for next in out.chain(into) {
                    if kinds.kind(next) != PieceKind::Other && touched.insert(next) {
                        queue.push(next);
                    }
                }
            }
        }
        Self {
            kinds,
            touched,
            node_count: patch.node_count,
        }
    }

    /// Whether a walk may enter `node`: a marked node of an untouched piece.
    #[inline]
    pub(super) fn walkable(&self, node: u32) -> bool {
        node < self.node_count
            && self.kinds.kind(node) != PieceKind::Other
            && !self.touched.contains(&node)
    }
}

/// How a walk into a piece ended (see `search::WalkEnd`).
pub(super) enum WalkEnd {
    Done,
    /// The visited-edge budget ran out (or the request was cancelled).
    Exhausted(PatchOutcome),
    /// The turn back at the first node costs more than a U-turn (a barrier
    /// there): the per-edge search relaxes the first edge instead.
    PerEdge,
}

impl PatchRun<'_, '_, '_, '_, '_> {
    /// Whether the search walks into real edge `arc` (its head on the
    /// forward side, `node`): a marked node of an untouched piece.
    #[inline]
    pub(super) fn walks_into(&self, arc: u32, node: u32) -> bool {
        arc < self.edge_count() && self.walk.as_ref().is_some_and(|walk| walk.walkable(node))
    }

    #[inline]
    fn is_kind(&self, node: u32, kind: PieceKind) -> bool {
        self.walk
            .as_ref()
            .is_some_and(|walk| walk.kinds.kind(node) == kind)
    }

    #[inline]
    fn edge_cost(&self, edge: u32) -> f64 {
        f64::from((self.query.costs.edge_cost)(edge))
    }

    /// See [`chain_continuation`].
    fn continuation(&self, edge: u32) -> Option<u32> {
        chain_continuation(
            &self.g,
            edge,
            |node| self.is_kind(node, PieceKind::Tree),
            |edge| self.edge_cost(edge).is_finite(),
        )
    }

    /// See [`chain_predecessor`].
    fn chain_predecessor(&self, edge: u32) -> Option<u32> {
        chain_predecessor(
            &self.g,
            edge,
            |node| self.is_kind(node, PieceKind::Tree),
            |edge| self.edge_cost(edge).is_finite(),
        )
    }

    /// See [`opposing_edge`].
    fn opposing(&self, edge: u32) -> Option<u32> {
        opposing_edge(&self.g, edge, |edge| self.edge_cost(edge).is_finite())
    }

    /// Turn cost and junction delay from real edge `previous` onto real edge
    /// `next` inside a piece, as the per-edge search adds them; infinite
    /// where a restriction or barrier closes the turn (none inside a piece:
    /// no marked node is a via node).
    #[inline]
    fn walk_transition(&self, previous: u32, next: u32) -> (f64, f64) {
        self.transition(previous, true, &self.info(next))
            .unwrap_or((f64::INFINITY, 0.0))
    }

    /// Relaxes real edge `first` out of forward label `pred` (`cost`,
    /// `duration_s`, `distance_m` at `first`'s end, restriction `state`)
    /// into a marked node of an untouched piece: the static search's
    /// [`walk`](crate::package::search) with the patch's labels. Queues the
    /// U-turn back onto `first`'s opposing edge and, on a chain, the edge
    /// entering the next node that is not a marked chain node (or before a
    /// turn dearer than a U-turn), summing costs, durations and junction
    /// delays edge by edge as the per-edge search does.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn walk_forward(
        &mut self,
        pred: u32,
        first: u32,
        (cost, duration_s, distance_m): (f64, f64, f64),
        state: u32,
        potential: &mut dyn FnMut(u32) -> f64,
    ) -> WalkEnd {
        let g = self.g;
        let back = self
            .opposing(first)
            .map(|back| (back, self.walk_transition(first, back)));
        if back.is_some_and(|(_, (turn, _))| turn.is_nan() || turn > U_TURN_COST) {
            return WalkEnd::PerEdge;
        }
        let origin = self.labels[pred as usize].origin;
        if let Some((back, (turn, delay))) = back {
            if let Err(outcome) = self.count_visit() {
                return WalkEnd::Exhausted(outcome);
            }
            let info = self.info(back);
            let back_cost = cost + turn + info.cost;
            let bound = potential(g.edge_source(first as usize));
            if back_cost.is_finite() && bound.is_finite() {
                self.push_label(
                    Label {
                        arc: back,
                        pred,
                        cost: back_cost,
                        duration_s: duration_s + delay + info.duration_s,
                        distance_m: distance_m + info.distance_m,
                        last_edge: back,
                        traversed: true,
                        origin,
                        state: self.automata.forward_next(&g, state, back),
                        via: first,
                        back: true,
                    },
                    back_cost + bound,
                );
            }
        }
        if !self.is_kind(g.edge_target[first as usize], PieceKind::Chain) {
            return WalkEnd::Done;
        }
        let (mut edge, mut cost, mut duration_s, mut distance_m, mut state) =
            (first, cost, duration_s, distance_m, state);
        let mut walked = 0;
        let end = loop {
            let at_end = !self.is_kind(g.edge_target[edge as usize], PieceKind::Chain);
            let next = if at_end {
                None
            } else {
                self.continuation(edge)
            };
            let (turn, delay) = match next {
                Some(next) => {
                    walked += 1;
                    self.walk_transition(edge, next)
                }
                None => (f64::INFINITY, 0.0),
            };
            if let (Some(next), true) = (next, turn <= U_TURN_COST) {
                let info = self.info(next);
                cost = cost + turn + info.cost;
                duration_s = duration_s + delay + info.duration_s;
                distance_m += info.distance_m;
                state = self.automata.forward_next(&g, state, next);
                edge = next;
                continue;
            }
            // A dead end (the way back is the U-turn above), or the edge
            // before a junction or a dear turn.
            if next.is_none() && !at_end {
                break WalkEnd::Done;
            }
            let bound = potential(g.edge_target[edge as usize]);
            if bound.is_finite() {
                self.push_label(
                    Label {
                        arc: edge,
                        pred,
                        cost,
                        duration_s,
                        distance_m,
                        last_edge: edge,
                        traversed: true,
                        origin,
                        state,
                        via: if edge == first { NO_LABEL } else { first },
                        back: false,
                    },
                    cost + bound,
                );
            }
            break WalkEnd::Done;
        };
        crate::package::search::walks::record(walked);
        end
    }

    /// [`Self::walk_forward`] for the backward search: relaxes real edge
    /// `first` (whose source is the marked node) before backward label
    /// `next_label`, with `cost`, `duration_s`, `distance_m` from entering
    /// `first`, in backward restriction `state`.
    pub(super) fn walk_backward(
        &mut self,
        next_label: u32,
        first: u32,
        (cost, duration_s, distance_m): (f64, f64, f64),
        state: u32,
        potential: &mut dyn FnMut(u32) -> f64,
    ) -> WalkEnd {
        let g = self.g;
        let back = self
            .opposing(first)
            .map(|back| (back, self.walk_transition(back, first)));
        if back.is_some_and(|(_, (turn, _))| turn.is_nan() || turn > U_TURN_COST) {
            return WalkEnd::PerEdge;
        }
        if let Some((back, (turn, delay))) = back {
            if let Err(outcome) = self.count_visit() {
                return WalkEnd::Exhausted(outcome);
            }
            let info = self.info(back);
            let back_cost = cost + turn + info.cost;
            let bound = potential(g.edge_source(back as usize));
            if let Some(back_state) = self.automata.backward_prepend(&g, state, back)
                && back_cost.is_finite()
                && bound.is_finite()
            {
                self.push_backward_label(
                    BackwardLabel {
                        arc: back,
                        next: next_label,
                        cost: back_cost,
                        duration_s: duration_s + delay + info.duration_s,
                        distance_m: distance_m + info.distance_m,
                        state: back_state,
                        via: first,
                        back: true,
                    },
                    back_cost + bound,
                );
            }
        }
        if !self.is_kind(g.edge_source(first as usize), PieceKind::Chain) {
            return WalkEnd::Done;
        }
        let (mut edge, mut cost, mut duration_s, mut distance_m, mut state) =
            (first, cost, duration_s, distance_m, state);
        let mut walked = 0;
        let end = loop {
            let at_end = !self.is_kind(g.edge_source(edge as usize), PieceKind::Chain);
            let previous = if at_end {
                None
            } else {
                self.chain_predecessor(edge)
            };
            let (turn, delay) = match previous {
                Some(previous) => {
                    walked += 1;
                    self.walk_transition(previous, edge)
                }
                None => (f64::INFINITY, 0.0),
            };
            if let (Some(previous), true) = (previous, turn <= U_TURN_COST) {
                let Some(next_state) = self.automata.backward_prepend(&g, state, previous) else {
                    // No marked node is a restriction's via node.
                    break WalkEnd::Done;
                };
                let info = self.info(previous);
                cost = cost + turn + info.cost;
                duration_s = duration_s + delay + info.duration_s;
                distance_m += info.distance_m;
                state = next_state;
                edge = previous;
                continue;
            }
            if previous.is_none() && !at_end {
                break WalkEnd::Done;
            }
            let bound = potential(g.edge_source(edge as usize));
            if bound.is_finite() {
                self.push_backward_label(
                    BackwardLabel {
                        arc: edge,
                        next: next_label,
                        cost,
                        duration_s,
                        distance_m,
                        state,
                        via: if edge == first { NO_LABEL } else { first },
                        back: false,
                    },
                    cost + bound,
                );
            }
            break WalkEnd::Done;
        };
        crate::package::search::walks::record(walked);
        end
    }

    /// Pushes the arcs a forward label covers, last first: its arc alone, or
    /// its walk from `via`.
    pub(super) fn push_label_arcs_reversed(&self, label: &Label, out: &mut Vec<u32>) {
        if label.via == NO_LABEL {
            out.push(label.arc);
        } else if label.back {
            out.extend([label.arc, label.via]);
        } else {
            let start = out.len();
            self.push_walk_edges(label.via, label.arc, out);
            out[start..].reverse();
        }
    }

    /// Pushes the arcs a backward label covers, in path order: its arc
    /// alone, or its walk to `via`.
    pub(super) fn push_backward_label_arcs(&self, label: &BackwardLabel, out: &mut Vec<u32>) {
        if label.via == NO_LABEL {
            out.push(label.arc);
        } else if label.back {
            out.extend([label.arc, label.via]);
        } else {
            self.push_walk_edges(label.arc, label.via, out);
        }
    }

    /// Pushes the walk from `first` to `last` along a piece's chain, both
    /// included.
    fn push_walk_edges(&self, first: u32, last: u32, out: &mut Vec<u32>) {
        let mut edge = first;
        out.push(edge);
        while edge != last {
            edge = self
                .continuation(edge)
                .expect("a walk follows its piece's chain to its last edge");
            out.push(edge);
        }
    }
}

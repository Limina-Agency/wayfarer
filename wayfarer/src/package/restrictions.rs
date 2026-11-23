//! Complex (via-way) restrictions as automata over edge sequences, so the
//! searches can key labels by restriction state and stay exact.
//!
//! A complex restriction forbids entering edge `t` right after the
//! traversed sequence `e1..ek` (its required prefix). One label per edge is
//! not exact under it: when the cheapest arrival on some `ei` comes through
//! the forbidden prefix, a dearer arrival that may continue to `t` is
//! dropped. A label is therefore keyed by (edge, state), where the state is
//! how far the path's suffix has progressed through the prefixes:
//!
//! - **Forward** (labels from the origin): an Aho–Corasick automaton over
//!   every required prefix of the profile's complex restrictions. A path's
//!   state is the longest suffix of its traversed edges that starts some
//!   prefix; it fixes exactly which next edges are forbidden, now and after
//!   any continuation, so labels with equal states are interchangeable and
//!   the cheaper dominates. Overlapping prefixes (one restriction's prefix
//!   inside or overlapping another's) are handled by the automaton's
//!   suffix links.
//! - **Backward** (labels towards the destination, which grow by prepending
//!   edges): the same construction over every whole forbidden sequence
//!   `e1..ek t`, reversed. A chain's state is the longest prefix of the
//!   chain that ends some forbidden sequence; prepending an edge that
//!   completes a sequence is forbidden.
//!
//! Joining a forward path and a backward chain is legal exactly when neither
//! side holds a forbidden sequence and none spans the join, which depends on
//! the two states alone (a sequence spanning the join splits into a suffix of
//! the forward path that starts a prefix, hence lies within the forward
//! state, and a prefix of the chain that ends a sequence, hence lies within
//! the backward state). The searches still check each join on the concrete
//! path.
//!
//! The root state (no partial match) is state 0. An edge that is in no
//! required prefix only ever leads to the root forward, and an edge that is
//! the restricted edge of no restriction only ever leads to the root
//! backward from the root; both are tested with the graph's edge flags, so
//! unrestricted traffic never looks the automata up and keeps one label per
//! edge in the dense slots. Simple (from, to) restrictions need no state.

use rustc_hash::FxHashMap;

use super::{EDGE_FLAG_COMPLEX_RESTRICTION_FIRST, EDGE_FLAG_COMPLEX_RESTRICTION_TO, SearchGraph};

/// The state of a path with no partial match.
pub(crate) const ROOT: u32 = 0;

/// One Aho–Corasick automaton over edge sequences.
#[derive(Debug, Default)]
struct Automaton {
    goto: FxHashMap<(u32, u32), u32>,
    fail: Vec<u32>,
    /// States in breadth-first order: every suffix link points earlier.
    order: Vec<u32>,
}

impl Automaton {
    fn step(&self, mut state: u32, edge: u32) -> u32 {
        loop {
            if let Some(next) = self.goto.get(&(state, edge)) {
                return *next;
            }
            if state == ROOT {
                return ROOT;
            }
            state = self.fail[state as usize];
        }
    }

    /// Builds the trie of `patterns`, returning each pattern's end state,
    /// then the suffix links (breadth first, so every link is computed before
    /// the states that use it).
    fn build<'p>(patterns: impl Iterator<Item = &'p [u32]>) -> (Self, Vec<u32>) {
        let mut automaton = Self {
            goto: FxHashMap::default(),
            fail: vec![ROOT],
            order: Vec::new(),
        };
        let mut children = vec![Vec::<(u32, u32)>::new()];
        let mut ends = Vec::new();
        for pattern in patterns {
            let mut state = ROOT;
            for edge in pattern {
                state = match automaton.goto.get(&(state, *edge)) {
                    Some(next) => *next,
                    None => {
                        let next = automaton.fail.len() as u32;
                        automaton.goto.insert((state, *edge), next);
                        automaton.fail.push(ROOT);
                        children.push(Vec::new());
                        children[state as usize].push((*edge, next));
                        next
                    }
                };
            }
            ends.push(state);
        }
        // Children of the root link to the root (the default); every other
        // state links to the step of its parent's link.
        let mut order = vec![ROOT];
        let mut cursor = 0;
        while cursor < order.len() {
            let state = order[cursor];
            cursor += 1;
            for (edge, child) in children[state as usize].clone() {
                if state != ROOT {
                    automaton.fail[child as usize] =
                        automaton.step(automaton.fail[state as usize], edge);
                }
                order.push(child);
            }
        }
        automaton.order = order;
        (automaton, ends)
    }

    fn state_count(&self) -> usize {
        self.fail.len()
    }
}

/// Forward and backward automata of one profile's complex restrictions.
#[derive(Debug, Default)]
pub(crate) struct ComplexAutomata {
    forward: Automaton,
    /// Per forward state, the sorted edges it may not enter next (its own
    /// restrictions and those of every suffix state).
    forbidden: Vec<Vec<u32>>,
    /// Per forward state, `(edge, n)` sorted by edge: entering `edge` next
    /// breaks `n` distinct restrictions (its own and its suffix states'), for
    /// the SAR `turn_restrictions` penalty.
    forbidden_counts: Vec<Vec<(u32, u32)>>,
    backward: Automaton,
    /// Per backward state, whether its chain starts with a whole forbidden
    /// sequence (it or a suffix state ends one).
    complete: Vec<bool>,
    /// Per backward state, how many distinct forbidden sequences its chain
    /// starts with.
    complete_counts: Vec<u32>,
    /// Bit per edge: the edge is in some forbidden sequence, so labels on
    /// it may be in a state other than the root (either direction).
    member: Vec<u64>,
    restriction_count: usize,
}

impl ComplexAutomata {
    pub(crate) fn new(graph: &SearchGraph<'_>, profile_bit: u16) -> Self {
        let restrictions = (0..graph.complex_to.len())
            .filter(|row| graph.complex_mask.get(*row) & profile_bit != 0)
            .map(|row| {
                (
                    &graph.complex_prefix_pool[graph.complex_prefix_start[row] as usize
                        ..graph.complex_prefix_start[row + 1] as usize],
                    graph.complex_to[row],
                )
            })
            .collect::<Vec<_>>();
        Self::from_restrictions(&restrictions, graph.edge_count)
    }

    /// Automata over `(required prefix, restricted edge)` pairs of edges
    /// below `edge_count`.
    fn from_restrictions(restrictions: &[(&[u32], u32)], edge_count: usize) -> Self {
        if restrictions.is_empty() {
            return Self {
                forbidden: vec![Vec::new()],
                forbidden_counts: vec![Vec::new()],
                complete: vec![false],
                complete_counts: vec![0],
                ..Self::default()
            };
        }
        let (forward, ends) = Automaton::build(restrictions.iter().map(|(prefix, _)| *prefix));
        let mut forbidden = vec![Vec::new(); forward.state_count()];
        for ((_, to), end) in restrictions.iter().zip(&ends) {
            forbidden[*end as usize].push(*to);
        }
        // A state is one prefix, so its own entries, deduplicated, are the
        // distinct restrictions ending there; its suffix states are other
        // prefixes, whose counts add.
        let own_counts = forbidden
            .iter()
            .map(|own| {
                let mut own = own.clone();
                own.sort_unstable();
                own.dedup();
                own.into_iter().map(|to| (to, 1)).collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        let forbidden_counts = Self::close_over_suffixes(&forward, own_counts, |own, inherited| {
            for (to, count) in inherited {
                match own.binary_search_by_key(to, |(edge, _)| *edge) {
                    Ok(index) => own[index].1 += count,
                    Err(index) => own.insert(index, (*to, *count)),
                }
            }
        });
        let forbidden = Self::close_over_suffixes(&forward, forbidden, |own, inherited| {
            own.extend_from_slice(inherited);
            own.sort_unstable();
            own.dedup();
        });

        let sequences = restrictions
            .iter()
            .map(|(prefix, to)| {
                std::iter::once(*to)
                    .chain(prefix.iter().rev().copied())
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        let (backward, ends) = Automaton::build(sequences.iter().map(Vec::as_slice));
        let mut complete = vec![false; backward.state_count()];
        for end in ends {
            complete[end as usize] = true;
        }
        let complete_counts = Self::close_over_suffixes(
            &backward,
            complete.iter().map(|own| u32::from(*own)).collect(),
            |own, inherited| *own += *inherited,
        );
        let complete = Self::close_over_suffixes(&backward, complete, |own, inherited| {
            *own |= *inherited;
        });
        let mut member = vec![0u64; edge_count.div_ceil(64)];
        for sequence in &sequences {
            for edge in sequence {
                member[*edge as usize / 64] |= 1 << (*edge % 64);
            }
        }
        Self {
            forward,
            forbidden,
            forbidden_counts,
            backward,
            complete,
            complete_counts,
            member,
            restriction_count: restrictions.len(),
        }
    }

    /// Folds each state's suffix-link target's value into its own, shorter
    /// suffixes first.
    fn close_over_suffixes<T: Clone>(
        automaton: &Automaton,
        mut values: Vec<T>,
        merge: impl Fn(&mut T, &T),
    ) -> Vec<T> {
        for state in automaton.order.iter().skip(1) {
            let inherited = values[automaton.fail[*state as usize] as usize].clone();
            merge(&mut values[*state as usize], &inherited);
        }
        values
    }

    /// Complex restrictions of this profile.
    #[cfg(test)]
    pub(crate) fn restriction_count(&self) -> usize {
        self.restriction_count
    }

    /// Forward and backward state counts, roots included.
    #[cfg(test)]
    pub(crate) fn state_counts(&self) -> (usize, usize) {
        (self.forward.state_count(), self.backward.state_count())
    }

    /// Forward and backward states whose suffix link is not the root: one
    /// sequence's suffix starts another's (overlapping sequences).
    #[cfg(test)]
    pub(crate) fn overlapping_state_counts(&self) -> (usize, usize) {
        let count = |automaton: &Automaton| automaton.fail.iter().filter(|f| **f != ROOT).count();
        (count(&self.forward), count(&self.backward))
    }

    /// Edges that may carry a state other than the root.
    #[cfg(test)]
    pub(crate) fn member_count(&self) -> usize {
        self.member
            .iter()
            .map(|word| word.count_ones() as usize)
            .sum()
    }

    /// Whether labels on `edge` may be in a state other than the root.
    #[inline]
    pub(crate) fn has_states(&self, edge: u32) -> bool {
        self.member
            .get(edge as usize / 64)
            .is_some_and(|word| word & (1 << (edge % 64)) != 0)
    }

    /// The forward state after traversing `edge` from `state`.
    #[inline]
    pub(crate) fn forward_next(&self, graph: &SearchGraph<'_>, state: u32, edge: u32) -> u32 {
        if state == ROOT
            && (self.restriction_count == 0
                || graph.edge_flags[edge as usize] & EDGE_FLAG_COMPLEX_RESTRICTION_FIRST == 0)
        {
            return ROOT;
        }
        self.forward.step(state, edge)
    }

    /// Whether a path in forward `state` may not enter `next`.
    #[inline]
    pub(crate) fn forbids(&self, state: u32, next: u32) -> bool {
        state != ROOT && self.forbidden[state as usize].binary_search(&next).is_ok()
    }

    /// How many distinct complex restrictions a path in forward `state`
    /// breaks by entering `next` (the SAR penalty count; [`Self::forbids`]
    /// is whether it is at least one).
    #[inline]
    pub(crate) fn forbidden_count(&self, state: u32, next: u32) -> u32 {
        if state == ROOT {
            return 0;
        }
        let counts = &self.forbidden_counts[state as usize];
        counts
            .binary_search_by_key(&next, |(edge, _)| *edge)
            .map_or(0, |index| counts[index].1)
    }

    /// The backward state of `edge` prepended to a chain in `state`, and how
    /// many distinct forbidden sequences the longer chain starts with: a
    /// banned prepend when that is not zero (see [`Self::backward_prepend`]),
    /// a penalised one under the SAR `turn_restrictions` relaxation.
    #[inline]
    pub(crate) fn backward_step(
        &self,
        graph: &SearchGraph<'_>,
        state: u32,
        edge: u32,
    ) -> (u32, u32) {
        if state == ROOT
            && (self.restriction_count == 0
                || graph.edge_flags[edge as usize] & EDGE_FLAG_COMPLEX_RESTRICTION_TO == 0)
        {
            return (ROOT, 0);
        }
        let next = self.backward.step(state, edge);
        (next, self.complete_counts[next as usize])
    }

    /// The backward state of `edge` prepended to a chain in `state`, or
    /// `None` when the longer chain starts with a forbidden sequence.
    #[inline]
    pub(crate) fn backward_prepend(
        &self,
        graph: &SearchGraph<'_>,
        state: u32,
        edge: u32,
    ) -> Option<u32> {
        if state == ROOT
            && (self.restriction_count == 0
                || graph.edge_flags[edge as usize] & EDGE_FLAG_COMPLEX_RESTRICTION_TO == 0)
        {
            return Some(ROOT);
        }
        let next = self.backward.step(state, edge);
        (!self.complete[next as usize]).then_some(next)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Small deterministic generator (xorshift).
    struct Rng(u64);

    impl Rng {
        fn below(&mut self, bound: u32) -> u32 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            (self.0 % u64::from(bound)) as u32
        }
    }

    /// The automata against the definitions, on random restriction sets over
    /// a few edges (so prefixes overlap, nest, repeat edges and share
    /// restricted edges) and random paths: a path may not continue onto `t`
    /// exactly when it ends with a required prefix of `t`, and a chain may
    /// not be prepended with `e` exactly when `e` + chain starts with a whole
    /// forbidden sequence.
    #[test]
    fn automata_match_restriction_definitions() {
        let mut rng = Rng(0x2545_f491_4f6c_dd1d);
        let alphabet = 5;
        let mut overlapping = 0;
        for _ in 0..300 {
            let restrictions = (0..1 + rng.below(6))
                .map(|_| {
                    let prefix = (0..1 + rng.below(4))
                        .map(|_| rng.below(alphabet))
                        .collect::<Vec<_>>();
                    (prefix, rng.below(alphabet))
                })
                .collect::<Vec<_>>();
            let borrowed = restrictions
                .iter()
                .map(|(prefix, to)| (prefix.as_slice(), *to))
                .collect::<Vec<_>>();
            let automata = ComplexAutomata::from_restrictions(&borrowed, alphabet as usize);
            overlapping += automata.forward.fail.iter().filter(|f| **f != ROOT).count();
            for _ in 0..40 {
                let path = (0..rng.below(9))
                    .map(|_| rng.below(alphabet))
                    .collect::<Vec<_>>();
                let mut state = ROOT;
                for edge in &path {
                    state = automata.forward.step(state, *edge);
                }
                for next in 0..alphabet {
                    let expected = restrictions
                        .iter()
                        .any(|(prefix, to)| *to == next && path.ends_with(prefix));
                    assert_eq!(
                        automata.forbids(state, next),
                        expected,
                        "{restrictions:?} {path:?} -> {next}"
                    );
                }
                // The chain is `path`, built by prepending from its end; once
                // a prefix of it is forbidden, longer ones are never built.
                let mut state = ROOT;
                for (index, edge) in path.iter().enumerate().rev() {
                    let chain = &path[index..];
                    let expected = restrictions.iter().any(|(prefix, to)| {
                        chain.len() > prefix.len()
                            && chain.starts_with(prefix)
                            && chain[prefix.len()] == *to
                    });
                    let next = automata.backward.step(state, *edge);
                    assert_eq!(
                        automata.complete[next as usize], expected,
                        "{restrictions:?} {chain:?}"
                    );
                    if expected {
                        break;
                    }
                    state = next;
                }
            }
        }
        assert!(overlapping > 100, "{overlapping}");
    }

    /// The SAR penalty counts against the definitions: entering `next`
    /// breaks as many restrictions as there are distinct (prefix, `next`)
    /// pairs the path ends with, and prepending to a chain (which, penalised,
    /// continues past a forbidden prefix) completes as many as there are
    /// distinct whole sequences the longer chain starts with.
    #[test]
    fn penalty_counts_match_restriction_definitions() {
        let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
        let alphabet = 4;
        let mut multiple = 0;
        for _ in 0..300 {
            let restrictions = (0..1 + rng.below(7))
                .map(|_| {
                    let prefix = (0..1 + rng.below(3))
                        .map(|_| rng.below(alphabet))
                        .collect::<Vec<_>>();
                    (prefix, rng.below(alphabet))
                })
                .collect::<Vec<_>>();
            let mut distinct = restrictions.clone();
            distinct.sort();
            distinct.dedup();
            let borrowed = restrictions
                .iter()
                .map(|(prefix, to)| (prefix.as_slice(), *to))
                .collect::<Vec<_>>();
            let automata = ComplexAutomata::from_restrictions(&borrowed, alphabet as usize);
            for _ in 0..40 {
                let path = (0..rng.below(9))
                    .map(|_| rng.below(alphabet))
                    .collect::<Vec<_>>();
                let mut state = ROOT;
                for edge in &path {
                    state = automata.forward.step(state, *edge);
                }
                for next in 0..alphabet {
                    let expected = distinct
                        .iter()
                        .filter(|(prefix, to)| *to == next && path.ends_with(prefix))
                        .count() as u32;
                    multiple += usize::from(expected > 1);
                    assert_eq!(
                        automata.forbidden_count(state, next),
                        expected,
                        "{restrictions:?} {path:?} -> {next}"
                    );
                }
                let mut state = ROOT;
                for (index, edge) in path.iter().enumerate().rev() {
                    let chain = &path[index..];
                    let expected = distinct
                        .iter()
                        .filter(|(prefix, to)| {
                            chain.len() > prefix.len()
                                && chain.starts_with(prefix)
                                && chain[prefix.len()] == *to
                        })
                        .count() as u32;
                    let next = automata.backward.step(state, *edge);
                    assert_eq!(
                        automata.complete_counts[next as usize], expected,
                        "{restrictions:?} {chain:?}"
                    );
                    state = next;
                }
            }
        }
        assert!(multiple > 50, "{multiple}");
    }
}

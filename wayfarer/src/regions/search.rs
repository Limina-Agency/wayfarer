//! The edge-based A* across the seam: the package search's semantics (one
//! label per directed edge and complex-restriction state, seeds on partly
//! traversed origin edges, turn costs, simple and complex restrictions,
//! destinations finished when the source node of a destination edge is
//! settled, the same tie-break), forward only, guided by any consistent
//! potential. It runs over the whole package, as a search over the regions
//! a corridor touches would; only the potential knows about the seam.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use rustc_hash::FxHashMap;

use crate::package::SearchGraph;
use crate::package::restrictions::{ComplexAutomata, ROOT};
use crate::package::search::{SearchPath, SearchQuery, approach_costs, tie_window};

const NO_LABEL: u32 = u32::MAX;
/// As the package search: a seed this close to its edge's end traverses
/// nothing.
const SEED_TRAVERSED_FRACTION_EPSILON: f64 = 1e-9;

#[derive(Clone, Copy)]
struct Label {
    edge: u32,
    pred: u32,
    cost: f64,
    origin: u16,
    traversed: bool,
    state: u32,
}

#[derive(Clone, Copy)]
struct Entry {
    priority: f64,
    seq: u32,
    label: u32,
}

impl PartialEq for Entry {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}
impl Eq for Entry {}
impl PartialOrd for Entry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Entry {
    // Min-heap on (priority, insertion order).
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .priority
            .total_cmp(&self.priority)
            .then_with(|| other.seq.cmp(&self.seq))
    }
}

struct Destination {
    edge: u32,
    source: u32,
    fraction: f64,
}

struct Finish {
    label: u32,
    destination: usize,
    cost: f64,
}

/// What the search did.
#[derive(Debug, Default, Clone, Copy)]
pub(crate) struct Counters {
    pub settled: usize,
    pub visited: usize,
    pub labels: usize,
}

struct Run<'a, 'g, P> {
    g: SearchGraph<'g>,
    q: &'a SearchQuery<'a>,
    automata: &'g ComplexAutomata,
    potential: P,
    labels: Vec<Label>,
    slots: FxHashMap<(u32, u32), u32>,
    heap: BinaryHeap<Entry>,
    seq: u32,
    origin_approach: Vec<f64>,
    destination_approach: Vec<f64>,
    counters: Counters,
}

/// Runs `query` (no cost overrides, budget, gates or restriction penalty:
/// the caller checks) guided by `potential` (infinite: no target reachable).
pub(crate) fn run<P: Fn(u32) -> f64>(
    g: SearchGraph<'_>,
    query: &SearchQuery<'_>,
    potential: P,
) -> (Option<SearchPath>, Counters) {
    let mut search = Run {
        g,
        q: query,
        automata: g.complex_automata(query.profile_bit),
        potential,
        labels: Vec::new(),
        slots: FxHashMap::default(),
        heap: BinaryHeap::new(),
        seq: 0,
        origin_approach: approach_costs(query.origins),
        destination_approach: approach_costs(query.destinations),
        counters: Counters::default(),
    };
    let path = search.execute();
    (path, search.counters)
}

impl<P: Fn(u32) -> f64> Run<'_, '_, P> {
    fn edge_cost(&self, edge: u32) -> f64 {
        f64::from(self.q.column.cost(edge))
    }

    fn execute(&mut self) -> Option<SearchPath> {
        let g = self.g;
        let destinations = self
            .q
            .destinations
            .iter()
            .zip(self.q.destination_edges)
            .map(|(destination, edge)| Destination {
                edge: *edge,
                source: g.edge_source(*edge as usize),
                fraction: destination.fraction_along_edge.clamp(0.0, 1.0),
            })
            .collect::<Vec<_>>();
        for (origin_index, (origin, edge)) in
            self.q.origins.iter().zip(self.q.origin_edges).enumerate()
        {
            let cost = self.edge_cost(*edge);
            if !cost.is_finite() {
                continue;
            }
            let remaining = (1.0 - origin.fraction_along_edge).clamp(0.0, 1.0);
            let start = cost * remaining + self.origin_approach[origin_index];
            let potential = (self.potential)(g.edge_target[*edge as usize]);
            if potential.is_infinite() {
                continue;
            }
            let traversed = remaining > SEED_TRAVERSED_FRACTION_EPSILON;
            let state = if traversed {
                self.automata.forward_next(&g, ROOT, *edge)
            } else {
                ROOT
            };
            self.push(
                *edge,
                NO_LABEL,
                start,
                start + potential,
                origin_index as u16,
                traversed,
                state,
            );
        }

        let mut best: Option<Finish> = None;
        while let Some(entry) = self.heap.pop() {
            if best
                .as_ref()
                .is_some_and(|best| entry.priority > best.cost + tie_window(best.cost))
            {
                break;
            }
            let label = self.labels[entry.label as usize];
            if self.slots.get(&(label.edge, label.state)) != Some(&entry.label) {
                continue;
            }
            self.counters.settled += 1;
            let node = g.edge_target[label.edge as usize];
            let last_traversed = label.traversed.then_some(label.edge);

            for (index, destination) in destinations.iter().enumerate() {
                if destination.source != node {
                    continue;
                }
                self.counters.visited += 1;
                if self.restricted(label.state, last_traversed, destination.edge) {
                    continue;
                }
                let edge_cost = self.edge_cost(destination.edge);
                if !edge_cost.is_finite() || self.finish_repeats(entry.label, destination) {
                    continue;
                }
                let turn = last_traversed.map_or(0.0, |last| g.turn_cost(last, destination.edge));
                if !turn.is_finite() {
                    continue;
                }
                let cost = label.cost
                    + turn
                    + edge_cost * destination.fraction
                    + self.destination_approach[index];
                let better = match &best {
                    None => true,
                    Some(current) => {
                        cost < current.cost - tie_window(current.cost)
                            || ((cost - current.cost).abs() <= tie_window(current.cost)
                                && self.path_key(entry.label, destination.edge)
                                    < self.path_key(
                                        current.label,
                                        destinations[current.destination].edge,
                                    ))
                    }
                };
                if better {
                    best = Some(Finish {
                        label: entry.label,
                        destination: index,
                        cost,
                    });
                }
            }

            for next in g.first_out[node as usize]..g.first_out[node as usize + 1] {
                self.counters.visited += 1;
                let edge_cost = self.edge_cost(next);
                if !edge_cost.is_finite() || self.restricted(label.state, last_traversed, next) {
                    continue;
                }
                let turn = g.turn_cost(label.edge, next);
                if !turn.is_finite() {
                    continue;
                }
                let target = g.edge_target[next as usize];
                let potential = (self.potential)(target);
                if potential.is_infinite() {
                    continue;
                }
                let cost = label.cost + edge_cost + turn;
                let state = self.automata.forward_next(&g, label.state, next);
                self.push(
                    next,
                    entry.label,
                    cost,
                    cost + potential,
                    label.origin,
                    true,
                    state,
                );
            }
        }
        best.map(|best| self.reconstruct(&best, &destinations))
    }

    #[allow(clippy::too_many_arguments)]
    fn push(
        &mut self,
        edge: u32,
        pred: u32,
        cost: f64,
        priority: f64,
        origin: u16,
        traversed: bool,
        state: u32,
    ) {
        if let Some(slot) = self.slots.get(&(edge, state))
            && self.labels[*slot as usize].cost <= cost
        {
            return;
        }
        let index = self.labels.len() as u32;
        self.labels.push(Label {
            edge,
            pred,
            cost,
            origin,
            traversed,
            state,
        });
        self.slots.insert((edge, state), index);
        self.seq = self.seq.wrapping_add(1);
        self.heap.push(Entry {
            priority,
            seq: self.seq,
            label: index,
        });
        self.counters.labels += 1;
    }

    /// Whether a path in `state`, whose last traversed edge is
    /// `last_traversed`, may not enter `next`.
    fn restricted(&self, state: u32, last_traversed: Option<u32>, next: u32) -> bool {
        last_traversed
            .is_some_and(|previous| self.g.simple_restricted(previous, next, self.q.profile_bit))
            || self.automata.forbids(state, next)
    }

    /// As the package search: whether finishing on `destination` after
    /// `label` repeats an edge already on the path.
    fn finish_repeats(&self, label: u32, destination: &Destination) -> bool {
        let mut current = label;
        while current != NO_LABEL {
            let entry = self.labels[current as usize];
            if entry.traversed && entry.edge == destination.edge {
                return entry.pred != NO_LABEL
                    || self.q.origins[usize::from(entry.origin)]
                        .fraction_along_edge
                        .clamp(0.0, 1.0)
                        <= destination.fraction;
            }
            current = entry.pred;
        }
        false
    }

    fn traversed_edges(&self, label: u32) -> Vec<u32> {
        let mut edges = Vec::new();
        let mut current = label;
        while current != NO_LABEL {
            let entry = self.labels[current as usize];
            if entry.traversed {
                edges.push(entry.edge);
            }
            current = entry.pred;
        }
        edges.reverse();
        edges
    }

    /// The package search's tie-break key: (edge count, edge ids).
    fn path_key(&self, label: u32, destination_edge: u32) -> (usize, Vec<u64>) {
        let mut ids = self
            .traversed_edges(label)
            .into_iter()
            .map(|edge| self.g.edge_id(edge as usize))
            .collect::<Vec<_>>();
        ids.push(self.g.edge_id(destination_edge as usize));
        (ids.len(), ids)
    }

    fn reconstruct(&self, finish: &Finish, destinations: &[Destination]) -> SearchPath {
        let g = &self.g;
        let column = self.q.column;
        let (mut distance_m, mut duration_s) = (0.0, 0.0);
        let mut seed = finish.label;
        let mut current = finish.label;
        while current != NO_LABEL {
            let entry = self.labels[current as usize];
            if entry.pred == NO_LABEL {
                seed = current;
            } else {
                distance_m += f64::from(g.edge_length[entry.edge as usize]);
                duration_s += f64::from(column.duration(entry.edge));
                let pred = self.labels[entry.pred as usize].edge;
                duration_s += g.junction_delay(&column, pred, entry.edge);
            }
            current = entry.pred;
        }
        let seed_label = self.labels[seed as usize];
        let origin_index = usize::from(seed_label.origin);
        let remaining = (1.0 - self.q.origins[origin_index].fraction_along_edge).clamp(0.0, 1.0);
        distance_m += f64::from(g.edge_length[seed_label.edge as usize]) * remaining;
        duration_s += f64::from(column.duration(seed_label.edge)) * remaining;
        let destination = &destinations[finish.destination];
        distance_m += f64::from(g.edge_length[destination.edge as usize]) * destination.fraction;
        duration_s += f64::from(column.duration(destination.edge)) * destination.fraction;
        let last = self.labels[finish.label as usize];
        if last.traversed {
            duration_s += g.junction_delay(&column, last.edge, destination.edge);
        }
        let mut edges = self.traversed_edges(finish.label);
        edges.push(destination.edge);
        SearchPath {
            edges,
            origin_index,
            destination_index: finish.destination,
            cost: finish.cost
                - self.origin_approach[origin_index]
                - self.destination_approach[finish.destination],
            distance_m,
            duration_s,
        }
    }
}

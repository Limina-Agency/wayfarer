//! Portal bounds: each region's own CCH metric, the overlay of lower bounds
//! between portal nodes, and the potential a search across both regions is
//! guided by (search (b) of the regions analysis, 4.2).
//!
//! Region `R`'s metric is the package's CCH customized with every edge that
//! does not have both ends in `R` made unusable. CCH distances are exact for
//! any metric on the topology, so its distances `d_R` are those of a CCH
//! built over `R` alone: paths that leave `R` and come back are not seen,
//! exactly as in a package compiled for `R` only.
//!
//! The overlay's vertices are the portal nodes: the ends of the usable edges
//! whose ends are owned by different regions. Its arcs are, per region,
//! `d_R(x, y)` from each of its portal nodes `x` to each of its exits `y`
//! (tails of portal edges; the "cliques" of lower bounds, cheap: elimination
//! tree queries, no turn costs) and each portal edge at its metric cost.
//! For a set of targets, `D(x)` is the overlay distance from `x` to them,
//! starting from `d_R(x, targets in R)` (a Dijkstra backwards over the
//! overlay). Every path to a target splits into stretches inside one region
//! and portal edges, so `D` is a lower bound, and region `R`'s potential is
//!
//! `pi_R(v) = min(d_R(v, targets in R), min over exits u of R of d_R(v, u) + D(u))`,
//!
//! one `CchPotential::with_offsets` query with the exits as seeds at their
//! offsets. Inside `R` it is a minimum of distances plus constants, hence
//! consistent; across a portal edge `(u, w)` it holds because `pi_R(u) <=
//! D(u) <= w(u, w) + D(w)` and `pi_R'(w)` evaluates to `D(w)`. The offsets
//! are lowered by a hair so that rounding in the two ways of summing `D(w)`
//! (overlay and CCH) can't break that.
//!
//! The cliques are exact region distances, so the overlay is exact for the
//! node-level metric: `pi_R` is the package's own potential, up to the f32
//! rounding of arc weights (the tests check both bounds node by node). The
//! cliques are lower bounds only for the edge-based costs (turns), as the
//! package's potentials are.

use std::cell::RefCell;
use std::cmp::Reverse;
use std::collections::BinaryHeap;

use wayfarer_package::format::Sections;

use super::seam::VirtualSeam;
use crate::package::cch::{
    CchMetric, CchPotential, CchTopology, GateDistances, POTENTIAL_MARGIN, PotentialScratch,
};

/// Offsets are lowered by this share against rounding (see the module docs).
const OFFSET_MARGIN: f64 = 1.0 - 1e-9;

/// One region's metric and its part of the overlay.
struct Region {
    metric: CchMetric,
    /// Overlay vertices (indexes into [`RegionBounds::nodes`]) of this
    /// region's portal nodes, and their package nodes.
    members: Vec<u32>,
    member_nodes: Vec<u32>,
    /// Overlay vertices of this region's exits, and their package nodes.
    exits: Vec<u32>,
    exit_nodes: Vec<u32>,
}

/// Per stored column: the regions' metrics and the overlay, independent of
/// the targets.
pub(crate) struct RegionBounds {
    regions: [Region; 2],
    /// Overlay vertices: package nodes, sorted.
    nodes: Vec<u32>,
    /// By overlay vertex: arcs into it, as (tail vertex, metric cost).
    into: Vec<Vec<(u32, f64)>>,
    pub(crate) stats: OverlayStats,
}

/// The overlay of a column (see `RegionBounds`), and what one query added.
#[derive(Debug, Clone, Default, PartialEq)]
#[non_exhaustive]
pub struct OverlayStats {
    /// Portal nodes, both regions.
    pub vertex_count: usize,
    /// Usable portal edges (overlay arcs between the regions).
    pub portal_arc_count: usize,
    /// Finite lower bounds between portal nodes inside a region.
    pub clique_arc_count: usize,
    /// Regions' metrics and cliques, milliseconds (once per column).
    pub build_ms: f64,
    /// For one query: target distances, the overlay Dijkstra and the
    /// regions' potentials, microseconds.
    pub query_us: f64,
    /// For one query: exits seeded at a finite offset.
    pub seeded_exit_count: usize,
}

impl RegionBounds {
    /// The bounds of the metric customized from `costs` (one per package
    /// edge, infinite = unusable).
    pub(crate) fn build(
        topology: &CchTopology,
        g: &Sections<'_>,
        seam: &VirtualSeam,
        costs: &[f32],
    ) -> Self {
        let started = std::time::Instant::now();
        let metric = |region: u8| {
            let restricted = (0..g.edge_count)
                .map(|edge| {
                    if seam.inside(edge as u32, g.edge_target[edge], region) {
                        costs[edge]
                    } else {
                        f32::INFINITY
                    }
                })
                .collect::<Vec<_>>();
            topology.customize(&restricted)
        };
        let metrics = [metric(0), metric(1)];

        // Usable portal edges, between and within regions' portal nodes.
        let mut portal_arcs = Vec::new();
        for edge in 0..g.edge_count as u32 {
            let target = g.edge_target[edge as usize];
            if seam.is_portal(edge, target) && costs[edge as usize].is_finite() {
                portal_arcs.push((seam.source(edge), target, f64::from(costs[edge as usize])));
            }
        }
        let mut nodes = portal_arcs
            .iter()
            .flat_map(|(tail, head, _)| [*tail, *head])
            .collect::<Vec<_>>();
        nodes.sort_unstable();
        nodes.dedup();
        let vertex = |node: u32| nodes.binary_search(&node).expect("a portal node") as u32;
        let mut into = vec![Vec::new(); nodes.len()];
        for (tail, head, cost) in &portal_arcs {
            into[vertex(*head) as usize].push((vertex(*tail), *cost));
        }
        let mut exit_nodes = [Vec::new(), Vec::new()];
        for (tail, _, _) in &portal_arcs {
            exit_nodes[usize::from(seam.owner(*tail))].push(*tail);
        }
        let mut clique_arc_count = 0;
        let mut built = Vec::with_capacity(2);
        for (region, metric) in metrics.into_iter().enumerate() {
            let mut exits = std::mem::take(&mut exit_nodes[region]);
            exits.sort_unstable();
            exits.dedup();
            let member_nodes = nodes
                .iter()
                .copied()
                .filter(|node| usize::from(seam.owner(*node)) == region)
                .collect::<Vec<_>>();
            if !exits.is_empty() {
                let distances = GateDistances::new(topology, &metric, false, &exits);
                for member in &member_nodes {
                    for (exit, distance) in exits.iter().zip(distances.from(*member)) {
                        if distance.is_finite() && exit != member {
                            into[vertex(*exit) as usize].push((vertex(*member), distance));
                            clique_arc_count += 1;
                        }
                    }
                }
            }
            built.push(Region {
                metric,
                members: member_nodes.iter().map(|node| vertex(*node)).collect(),
                member_nodes,
                exits: exits.iter().map(|node| vertex(*node)).collect(),
                exit_nodes: exits,
            });
        }
        let regions: [Region; 2] = built.try_into().ok().expect("two regions");
        let stats = OverlayStats {
            vertex_count: nodes.len(),
            portal_arc_count: portal_arcs.len(),
            clique_arc_count,
            build_ms: started.elapsed().as_secs_f64() * 1e3,
            ..Default::default()
        };
        Self {
            regions,
            nodes,
            into,
            stats,
        }
    }

    /// The potential towards `targets` (package nodes), with the column's
    /// `factor` (see `search::potential_plan`).
    pub(crate) fn potential<'a>(
        &'a self,
        topology: &'a CchTopology,
        seam: &'a VirtualSeam,
        targets: &[u32],
        factor: f64,
        scratch: &'a [RefCell<PotentialScratch>; 2],
    ) -> (SeamPotential<'a>, OverlayStats) {
        let started = std::time::Instant::now();
        let mut by_region = [Vec::new(), Vec::new()];
        for target in targets {
            by_region[usize::from(seam.owner(*target))].push(*target);
        }
        // D starts at each portal node's distance to its region's targets.
        let mut offset = vec![f64::INFINITY; self.nodes.len()];
        for (region, targets) in self.regions.iter().zip(&by_region) {
            if targets.is_empty() || region.member_nodes.is_empty() {
                continue;
            }
            let distances = GateDistances::new(topology, &region.metric, false, targets);
            for (vertex, node) in region.members.iter().zip(&region.member_nodes) {
                offset[*vertex as usize] = distances
                    .from(*node)
                    .into_iter()
                    .fold(f64::INFINITY, f64::min);
            }
        }
        // Backwards over the overlay.
        let mut heap = BinaryHeap::new();
        for (vertex, value) in offset.iter().enumerate() {
            if value.is_finite() {
                heap.push((Reverse(Ordered(*value)), vertex as u32));
            }
        }
        while let Some((Reverse(Ordered(value)), vertex)) = heap.pop() {
            if value > offset[vertex as usize] {
                continue;
            }
            for (tail, cost) in &self.into[vertex as usize] {
                let candidate = value + cost;
                if candidate < offset[*tail as usize] {
                    offset[*tail as usize] = candidate;
                    heap.push((Reverse(Ordered(candidate)), *tail));
                }
            }
        }
        // Seeds in request costs, as `with_offsets` takes them.
        let scale = factor * POTENTIAL_MARGIN;
        let mut seeded_exit_count = 0;
        let mut potential = |region: usize| {
            let r = &self.regions[region];
            let seeds = r
                .exits
                .iter()
                .zip(&r.exit_nodes)
                .filter(|(vertex, _)| offset[**vertex as usize].is_finite())
                .map(|(vertex, node)| (*node, offset[*vertex as usize] * OFFSET_MARGIN * scale))
                .collect::<Vec<_>>();
            seeded_exit_count += seeds.len();
            CchPotential::with_offsets(
                topology,
                &r.metric,
                &by_region[region],
                &seeds,
                false,
                factor,
                &scratch[region],
            )
        };
        let potentials = [potential(0), potential(1)];
        let stats = OverlayStats {
            query_us: started.elapsed().as_secs_f64() * 1e6,
            seeded_exit_count,
            ..self.stats.clone()
        };
        (
            SeamPotential {
                owner: seam.owners(),
                potentials,
            },
            stats,
        )
    }
}

/// The potential of a search across the seam: each node's region's
/// potential (see the module docs), in request costs.
pub(crate) struct SeamPotential<'a> {
    owner: &'a [u8],
    potentials: [CchPotential<'a>; 2],
}

impl SeamPotential<'_> {
    #[inline]
    pub(crate) fn get(&self, node: u32) -> f64 {
        self.potentials[usize::from(self.owner[node as usize])].get(node)
    }
}

/// An f64 ordered by `total_cmp`, for the overlay's heap.
#[derive(Clone, Copy, PartialEq)]
struct Ordered(f64);
impl Eq for Ordered {}
impl PartialOrd for Ordered {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Ordered {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.total_cmp(&other.0)
    }
}

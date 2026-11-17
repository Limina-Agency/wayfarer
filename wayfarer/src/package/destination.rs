//! Destination-only roads: private, customer, destination, permit and
//! delivery roads a vehicle may use to reach somewhere on them, never to pass
//! through (the roads Valhalla marks `destination_only`).
//!
//! A strict search runs on a column that blocks every destination-only edge,
//! and unblocks only the *pockets* around its endpoints: the connected sets of
//! destination-only edges the profile may use that an origin or destination
//! candidate lies on. So a path starts or ends inside a pocket and crosses no
//! other. An endpoint that cannot leave without crossing one (a square ringed
//! by private streets) finds no path, and the search retries on the relaxed
//! column.
//!
//! Unblocking goes through the request's sparse cost overrides, so the search
//! loops know nothing of it: the landmark bounds take the unblocked edges as
//! discount gates (`search::gates`), and the walks stay exact, the strict
//! column with a pocket unblocked using no edge the stored column (which
//! classified the pieces) cannot.

use rustc_hash::{FxHashMap, FxHashSet};

use wayfarer_costing::{EdgeCostDecision, EdgeCostFlags};

use super::{RoutingPackage, SearchGraph, profile_bit, profile_slot};
use crate::costing::CostingPolicy;
use crate::types::{MobilityProfile, RouteOptions};

/// The package's destination-only edges a profile may use, grouped into
/// pockets.
#[derive(Debug, Default)]
pub(crate) struct DestinationPockets {
    pocket_of: FxHashMap<u32, u32>,
    /// `edges[first[pocket]..first[pocket + 1]]` are the pocket's edges.
    first: Vec<u32>,
    edges: Vec<u32>,
}

impl DestinationPockets {
    pub(crate) fn build(graph: SearchGraph<'_>, profile: MobilityProfile) -> Self {
        let bit = profile_bit(profile);
        let destination_only = (0..graph.edge_count as u32)
            .filter(|edge| {
                let index = *edge as usize;
                let tuple = graph.cost_tuple(graph.edge_shared_info[index] as usize);
                graph.edge_profile_access[index] & bit != 0
                    && EdgeCostFlags(graph.si_cost_flags[tuple])
                        .has(EdgeCostFlags::DESTINATION_ONLY)
            })
            .collect::<Vec<_>>();
        if destination_only.is_empty() {
            return Self::default();
        }
        let mut parent = (0..graph.node_count as u32).collect::<Vec<_>>();
        fn find(parent: &mut [u32], mut node: u32) -> u32 {
            while parent[node as usize] != node {
                parent[node as usize] = parent[parent[node as usize] as usize];
                node = parent[node as usize];
            }
            node
        }
        for edge in &destination_only {
            let source = find(&mut parent, graph.edge_source(*edge as usize));
            let target = find(&mut parent, graph.edge_target[*edge as usize]);
            parent[source.max(target) as usize] = source.min(target);
        }
        let mut pocket_of_root = FxHashMap::<u32, u32>::default();
        let mut members = Vec::<Vec<u32>>::new();
        let mut pocket_of = FxHashMap::default();
        for edge in destination_only {
            let root = find(&mut parent, graph.edge_source(edge as usize));
            let pocket = *pocket_of_root.entry(root).or_insert_with(|| {
                members.push(Vec::new());
                (members.len() - 1) as u32
            });
            members[pocket as usize].push(edge);
            pocket_of.insert(edge, pocket);
        }
        let mut first = vec![0u32];
        let mut edges = Vec::new();
        for pocket in members {
            edges.extend(pocket);
            first.push(edges.len() as u32);
        }
        Self {
            pocket_of,
            first,
            edges,
        }
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.edges.is_empty()
    }

    fn pocket_edges(&self, pocket: u32) -> &[u32] {
        &self.edges[self.first[pocket as usize] as usize..self.first[pocket as usize + 1] as usize]
    }
}

/// Whether a request runs its strict pass first: the vehicles destination-only
/// roads confine (emergency vehicles have business on them, walkers are not
/// through traffic, and a SAR request whose R1 names `private` or
/// `customers` may cross them, see `sar::lifts_destination_only`), and any
/// profile with `avoid_destination_only >= 1`.
pub(crate) fn strict_destination_only(options: &RouteOptions, profile: MobilityProfile) -> bool {
    let confined = matches!(
        profile,
        MobilityProfile::Default | MobilityProfile::Wheeled | MobilityProfile::Tracked
    ) && !crate::sar::lifts_destination_only(options, profile);
    confined
        || (options.costing.avoid_destination_only.is_finite()
            && options.costing.avoid_destination_only >= 1.0)
}

impl RoutingPackage {
    /// The pockets of `profile`, grouped on first use.
    pub(crate) fn destination_pockets(&self, profile: MobilityProfile) -> &DestinationPockets {
        self.destination_pockets[profile_slot(profile)]
            .get_or_init(|| DestinationPockets::build(self.view(), profile))
    }

    /// [`strict_destination_only`] where the package has anything to confine:
    /// without destination-only edges the strict column is the relaxed one.
    pub(crate) fn confines_destination_only(
        &self,
        options: &RouteOptions,
        profile: MobilityProfile,
    ) -> bool {
        strict_destination_only(options, profile) && !self.destination_pockets(profile).is_empty()
    }

    /// Unblocks, for a search on the strict column, the pockets around
    /// `endpoints` (candidate edges of its origins and destinations): each of
    /// their edges is costed by the relaxed policy, in `cost_overrides` and
    /// (the strict column has no duration for it) `durations`. Returns those
    /// costs, which an overlay's split edges among them take as their
    /// fragments' (see [`relax_split_costs`]).
    pub(crate) fn unblock_destination_pockets(
        &self,
        options: &RouteOptions,
        profile: MobilityProfile,
        endpoints: impl IntoIterator<Item = u32>,
        cost_overrides: &mut FxHashMap<u32, f32>,
        durations: &mut FxHashMap<u32, f32>,
    ) -> FxHashMap<u32, f32> {
        let mut unblocked = FxHashMap::default();
        let graph = self.view();
        let pockets = self.destination_pockets(profile);
        let relaxed = CostingPolicy::new(profile, options);
        let (bit, mask) = (profile_bit(profile), profile.mobility_mode_mask());
        let mut seen = FxHashSet::default();
        for endpoint in endpoints {
            let Some(pocket) = pockets.pocket_of.get(&endpoint) else {
                continue;
            };
            if !seen.insert(*pocket) {
                continue;
            }
            for edge in pockets.pocket_edges(*pocket) {
                if let EdgeCostDecision::Allowed(costed) =
                    graph.edge_cost_decision(&relaxed, *edge, bit, mask)
                {
                    cost_overrides.insert(*edge, costed.cost as f32);
                    durations.insert(*edge, costed.duration_s as f32);
                    unblocked.insert(*edge, costed.cost as f32);
                }
            }
        }
        unblocked
    }
}

/// Costs an overlay's split edges that lie in an unblocked pocket
/// (`unblocked`, from `unblock_destination_pockets`) as the pocket's other
/// edges: `split_cost` came from the strict policy, which refuses them.
pub(crate) fn relax_split_costs(
    unblocked: &FxHashMap<u32, f32>,
    split_cost: &mut FxHashMap<u32, f32>,
) {
    for (edge, cost) in split_cost.iter_mut() {
        if let Some(relaxed) = unblocked.get(edge) {
            *cost = *relaxed;
        }
    }
}

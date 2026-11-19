//! A virtual seam: one package cut into two "regions" at the top separator
//! of its stored node order.
//!
//! The compiler orders nodes by nested dissection (`cch_order`) and stores
//! only the ranks, but the CCH's elimination tree keeps the dissection's
//! shape: every edge joins a node to one of its ancestors, so the subtrees
//! below a node share no edge, and the nodes above the first balanced
//! branching (the top separator) are all that join its subtrees. The cut
//! walks down from the root of the largest component while one child holds
//! nearly everything (degree ≤ 2 chains, eliminated first, hang off
//! separator nodes as small subtrees), then gives the subtrees found on the
//! way to the two regions, largest first to the smaller one. Separator nodes
//! join the region they share most edges with (lowest rank first), as the
//! scratch tool of the regions analysis did. The cut is as deterministic as
//! the stored order.

use wayfarer_package::format::Sections;

use super::{RegionEdge, SeamError};
use crate::package::cch::CchTopology;

/// Below the separator, the heaviest child subtree may hold at most this
/// share of the nodes for the branching to count as the split.
const BALANCE: f64 = 0.85;

/// The two regions of a seam (see `SeamRouter`), and what joins them.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct SeamSummary {
    /// Nodes owned by each region.
    pub node_count: [usize; 2],
    /// Edges owned by each region (an edge belongs to its source's region).
    pub edge_count: [usize; 2],
    /// Nodes of the top separator (each owned by one region).
    pub separator_node_count: usize,
    /// Directed edges whose ends are owned by different regions, for any
    /// profile.
    pub portal_edge_count: usize,
    /// Connected components of the node graph (elimination-tree roots).
    pub component_count: usize,
    /// Subtrees given to the regions (below the separator, other
    /// components).
    pub subtree_count: usize,
}

/// One package cut into two regions (see the module docs).
pub(crate) struct VirtualSeam {
    /// By package node: its region.
    owner: Vec<u8>,
    /// By package edge: its index among its region's edges.
    local: Vec<u32>,
    /// By region: its edges, by local index.
    region_edges: [Vec<u32>; 2],
    /// By package edge: its source node.
    source: Vec<u32>,
    /// Package nodes of the top separator, top down.
    separator: Vec<u32>,
    summary: SeamSummary,
}

impl VirtualSeam {
    pub(crate) fn cut(topology: &CchTopology, g: &Sections<'_>) -> Result<Self, SeamError> {
        let n = topology.node_count();
        if n != g.node_count || n < 2 {
            return Err(SeamError::NotSplittable);
        }
        // Children by rank (a parent's rank is above its children's).
        let mut first_child = vec![0u32; n + 1];
        for rank in 0..n as u32 {
            if let Some(parent) = topology.parent_of(rank) {
                first_child[parent as usize + 1] += 1;
            }
        }
        for rank in 0..n {
            first_child[rank + 1] += first_child[rank];
        }
        let mut children = vec![0u32; n];
        let mut fill = first_child.clone();
        let mut size = vec![1u32; n];
        let mut roots = Vec::new();
        for rank in 0..n as u32 {
            match topology.parent_of(rank) {
                Some(parent) => {
                    children[fill[parent as usize] as usize] = rank;
                    fill[parent as usize] += 1;
                    size[parent as usize] += size[rank as usize];
                }
                None => roots.push(rank),
            }
        }
        let children_of = |rank: u32| {
            &children[first_child[rank as usize] as usize..first_child[rank as usize + 1] as usize]
        };
        let main = *roots
            .iter()
            .max_by_key(|rank| (size[**rank as usize], **rank))
            .ok_or(SeamError::NotSplittable)?;

        // Walk down to the first balanced branching.
        let mut separator_ranks = Vec::new();
        let mut parts = roots
            .iter()
            .copied()
            .filter(|rank| *rank != main)
            .collect::<Vec<_>>();
        let mut x = main;
        loop {
            separator_ranks.push(x);
            let mut kids = children_of(x).to_vec();
            kids.sort_unstable_by_key(|rank| (std::cmp::Reverse(size[*rank as usize]), *rank));
            let Some(&heaviest) = kids.first() else {
                break;
            };
            let below = f64::from(size[x as usize] - 1);
            if kids.len() >= 2 && f64::from(size[heaviest as usize]) <= BALANCE * below {
                parts.extend(kids);
                break;
            }
            parts.extend(&kids[1..]);
            x = heaviest;
        }
        if separator_ranks.len() > n / 4 {
            return Err(SeamError::NotSplittable);
        }

        // Subtrees to the regions, largest first to the smaller.
        parts.sort_unstable_by_key(|rank| (std::cmp::Reverse(size[*rank as usize]), *rank));
        const NONE: u8 = u8::MAX;
        let mut owner_by_rank = vec![NONE; n];
        let mut load = [0u64; 2];
        for part in &parts {
            let region = usize::from(load[1] < load[0]);
            owner_by_rank[*part as usize] = region as u8;
            load[region] += u64::from(size[*part as usize]);
        }
        let mut is_separator = vec![false; n];
        for rank in &separator_ranks {
            is_separator[*rank as usize] = true;
        }
        // Down the tree: a node joins its parent's region.
        for rank in (0..n as u32).rev() {
            if owner_by_rank[rank as usize] != NONE || is_separator[rank as usize] {
                continue;
            }
            if let Some(parent) = topology.parent_of(rank)
                && !is_separator[parent as usize]
            {
                owner_by_rank[rank as usize] = owner_by_rank[parent as usize];
            }
        }
        let mut owner = vec![NONE; n];
        for (rank, region) in owner_by_rank.iter().enumerate() {
            owner[topology.node_at(rank as u32) as usize] = *region;
        }

        // Edge sources, and each separator node to the region it shares most
        // edges with, lowest rank first.
        let mut source = vec![0u32; g.edge_count];
        let mut incident = vec![Vec::<u32>::new(); separator_ranks.len()];
        let separator = separator_ranks
            .iter()
            .map(|rank| topology.node_at(*rank))
            .collect::<Vec<_>>();
        let mut separator_position = vec![u32::MAX; n];
        for (index, node) in separator.iter().enumerate() {
            separator_position[*node as usize] = index as u32;
        }
        let separator_index = |node: u32| {
            let index = separator_position[node as usize];
            (index != u32::MAX).then_some(index as usize)
        };
        for node in 0..n {
            for edge in g.first_out[node]..g.first_out[node + 1] {
                source[edge as usize] = node as u32;
                let target = g.edge_target[edge as usize];
                if let Some(index) = separator_index(node as u32) {
                    incident[index].push(target);
                }
                if let Some(index) = separator_index(target) {
                    incident[index].push(node as u32);
                }
            }
        }
        for index in (0..separator.len()).rev() {
            let mut count = [0usize; 2];
            for other in &incident[index] {
                if let Some(region) = [0u8, 1].into_iter().find(|r| owner[*other as usize] == *r) {
                    count[usize::from(region)] += 1;
                }
            }
            let region = if count[0] != count[1] {
                usize::from(count[1] > count[0])
            } else {
                usize::from(load[1] < load[0])
            };
            owner[separator[index] as usize] = region as u8;
            load[region] += 1;
        }
        debug_assert!(owner.iter().all(|region| *region < 2));

        let mut local = vec![0u32; g.edge_count];
        let mut region_edges = [Vec::new(), Vec::new()];
        let mut portal_edge_count = 0;
        for edge in 0..g.edge_count {
            let region = usize::from(owner[source[edge] as usize]);
            local[edge] = region_edges[region].len() as u32;
            region_edges[region].push(edge as u32);
            if owner[source[edge] as usize] != owner[g.edge_target[edge] as usize] {
                portal_edge_count += 1;
            }
        }
        let mut node_count = [0usize; 2];
        for region in &owner {
            node_count[usize::from(*region)] += 1;
        }
        if node_count.contains(&0) {
            return Err(SeamError::NotSplittable);
        }
        let summary = SeamSummary {
            node_count,
            edge_count: [region_edges[0].len(), region_edges[1].len()],
            separator_node_count: separator.len(),
            portal_edge_count,
            component_count: roots.len(),
            subtree_count: parts.len(),
        };
        Ok(Self {
            owner,
            local,
            region_edges,
            source,
            separator,
            summary,
        })
    }

    pub(crate) fn summary(&self) -> &SeamSummary {
        &self.summary
    }

    /// By package node: its region (0 or 1).
    pub(crate) fn owners(&self) -> &[u8] {
        &self.owner
    }

    #[inline]
    pub(crate) fn owner(&self, node: u32) -> u8 {
        self.owner[node as usize]
    }

    #[inline]
    pub(crate) fn source(&self, edge: u32) -> u32 {
        self.source[edge as usize]
    }

    /// Whether `edge` joins the two regions.
    #[inline]
    pub(crate) fn is_portal(&self, edge: u32, target: u32) -> bool {
        self.owner(self.source(edge)) != self.owner(target)
    }

    /// Whether both ends of `edge` are owned by `region`.
    #[inline]
    pub(crate) fn inside(&self, edge: u32, target: u32, region: u8) -> bool {
        self.owner(self.source(edge)) == region && self.owner(target) == region
    }

    pub(crate) fn separator(&self) -> &[u32] {
        &self.separator
    }

    /// The region-qualified id of package edge `edge`.
    pub(crate) fn region_edge(&self, edge: u32) -> RegionEdge {
        RegionEdge {
            region: self.owner(self.source(edge)),
            edge: self.local[edge as usize],
        }
    }

    /// The package edge of a region-qualified id.
    pub(crate) fn package_edge(&self, edge: RegionEdge) -> Option<u32> {
        self.region_edges
            .get(usize::from(edge.region))?
            .get(edge.edge as usize)
            .copied()
    }
}

//! Walkable pieces of each stored column (`Section::PieceChain`,
//! `Section::PieceTree`), which let the route search walk through nodes that
//! are not decision points without queueing a label at each
//!.
//!
//! On the undirected simple graph of a column's usable arcs (finite,
//! non-negative cost), peeling nodes of degree at most one leaves the 2-core;
//! its nodes of degree three or more are *junctions*. Removing the junctions
//! splits the other nodes into *pieces*: a dead-end tree hanging off one
//! junction, or a chain of degree-2 core nodes with the trees hanging off it,
//! whose ends attach to one or two junctions (its *anchors*), over the stored
//! column's own arcs.
//!
//! The search walks through a piece without queueing labels inside, which
//! is exact where turning back inside it costs at least a U-turn and nothing
//! inside restricts a turn. So a node is *marked* (a chain node or a tree
//! node) when its piece has an anchor, and neither it nor any node of the
//! dead-end trees below it (away from the 2-core) is
//!
//! - the via node of a turn restriction (any profile: the end of a simple
//!   restriction's `from` edge, or of any edge of a complex restriction's
//!   prefix), or
//! - the end of a usable self-loop, or of a pair of adjacent nodes joined by
//!   more than one segment (shared info) or by two usable edges in one
//!   direction, where a path could turn round without a U-turn.
//!
//! Every other node (junctions, such nodes, the tree nodes above them and
//! the chain nodes they hang off, nodes of pieces without an anchor) is
//! unmarked, and the search treats it as it always did. Barriers need no
//! mark: their costs are transition costs, which the walks add and check
//! like turn costs.

/// Inputs of [`piece_bits`]: the package's adjacency, costs per column, and
/// what unmarks a node.
pub(crate) struct PieceInput<'a> {
    pub node_count: usize,
    pub edge_count: usize,
    pub first_out: &'a [u32],
    pub first_in: &'a [u32],
    pub in_edges: &'a [u32],
    pub edge_source: &'a [u32],
    pub edge_target: &'a [u32],
    /// `[column][edge]`.
    pub edge_cost: &'a [f32],
    pub edge_shared_info: &'a [u32],
    /// Per node: the via node of some simple or complex restriction.
    pub restricted: &'a [bool],
}

/// Per column `[column][node / 64]` bits of marked chain nodes and tree
/// nodes, and the counts of each per column.
pub(crate) struct PieceBits {
    pub chain: Vec<u64>,
    pub tree: Vec<u64>,
    pub counts: Vec<(usize, usize)>,
}

const NONE: u32 = u32::MAX;

pub(crate) fn piece_bits(input: &PieceInput<'_>, column_count: usize) -> PieceBits {
    let n = input.node_count;
    let words = n.div_ceil(64);
    let mut bits = PieceBits {
        chain: vec![0; column_count * words],
        tree: vec![0; column_count * words],
        counts: Vec::with_capacity(column_count),
    };
    for column in 0..column_count {
        let costs = &input.edge_cost[column * input.edge_count..(column + 1) * input.edge_count];
        let (chain, tree) = classify(input, costs);
        let mut count = (0, 0);
        for node in 0..n {
            let (word, bit) = (column * words + node / 64, 1u64 << (node % 64));
            if chain[node] {
                bits.chain[word] |= bit;
                count.0 += 1;
            } else if tree[node] {
                bits.tree[word] |= bit;
                count.1 += 1;
            }
        }
        bits.counts.push(count);
    }
    bits
}

/// The distinct neighbours of `node` over usable edges in either direction,
/// itself excluded, into `out` (sorted).
fn neighbours(input: &PieceInput<'_>, usable: &[bool], node: usize, out: &mut Vec<u32>) {
    out.clear();
    for edge in input.first_out[node]..input.first_out[node + 1] {
        if usable[edge as usize] {
            out.push(input.edge_target[edge as usize]);
        }
    }
    for slot in input.first_in[node]..input.first_in[node + 1] {
        let edge = input.in_edges[slot as usize];
        if usable[edge as usize] {
            out.push(input.edge_source[edge as usize]);
        }
    }
    out.retain(|next| *next as usize != node);
    out.sort_unstable();
    out.dedup();
}

/// Whether `node`'s own usable edges allow turning round only by a U-turn:
/// no self-loop, and one segment, at most one edge per direction, to each
/// neighbour.
fn turns_by_u_turn(
    input: &PieceInput<'_>,
    usable: &[bool],
    node: usize,
    scratch: &mut Vec<Link>,
) -> bool {
    scratch.clear();
    for edge in input.first_out[node]..input.first_out[node + 1] {
        let edge = edge as usize;
        if usable[edge] {
            let target = input.edge_target[edge];
            if target as usize == node {
                return false;
            }
            scratch.push((target, true, input.edge_shared_info[edge]));
        }
    }
    for slot in input.first_in[node]..input.first_in[node + 1] {
        let edge = input.in_edges[slot as usize] as usize;
        if usable[edge] {
            scratch.push((input.edge_source[edge], false, input.edge_shared_info[edge]));
        }
    }
    scratch.sort_unstable();
    scratch.chunk_by(|a, b| a.0 == b.0).all(|links| {
        let outgoing = links.iter().filter(|link| link.1).count();
        outgoing <= 1
            && links.len() - outgoing <= 1
            && links.iter().all(|link| link.2 == links[0].2)
    })
}

/// A usable edge at a node: (other node, outgoing, shared info).
type Link = (u32, bool, u32);

/// Per node: (marked chain node, marked tree node).
fn classify(input: &PieceInput<'_>, costs: &[f32]) -> (Vec<bool>, Vec<bool>) {
    let n = input.node_count;
    let usable = costs
        .iter()
        .map(|cost| cost.is_finite() && *cost >= 0.0)
        .collect::<Vec<_>>();
    let mut scratch = Vec::new();
    // Peel to the 2-core, noting each peeled node's parent: its one
    // neighbour left when it was peeled (towards the core).
    let mut degree = (0..n)
        .map(|node| {
            neighbours(input, &usable, node, &mut scratch);
            scratch.len() as u32
        })
        .collect::<Vec<_>>();
    let mut core = vec![true; n];
    let mut parent = vec![NONE; n];
    let mut peeled = Vec::new();
    let mut stack = (0..n as u32)
        .filter(|node| degree[*node as usize] <= 1)
        .collect::<Vec<_>>();
    while let Some(node) = stack.pop() {
        if !core[node as usize] {
            continue;
        }
        core[node as usize] = false;
        peeled.push(node);
        neighbours(input, &usable, node as usize, &mut scratch);
        for next in &scratch {
            let next = *next as usize;
            if core[next] {
                parent[node as usize] = next as u32;
                degree[next] -= 1;
                if degree[next] == 1 {
                    stack.push(next as u32);
                }
            }
        }
    }
    let junction = (0..n)
        .map(|node| core[node] && degree[node] >= 3)
        .collect::<Vec<_>>();
    drop(degree);

    // A node is unmarked when it, or a node of the trees below it, is a
    // restriction's via node or can turn round without a U-turn. Trees
    // were peeled leaves first, so children come before their parents.
    let mut links = Vec::new();
    let mut unmarked = (0..n)
        .map(|node| input.restricted[node] || !turns_by_u_turn(input, &usable, node, &mut links))
        .collect::<Vec<_>>();
    for node in &peeled {
        let node = *node as usize;
        if unmarked[node] && parent[node] != NONE {
            unmarked[parent[node] as usize] = true;
        }
    }
    drop(parent);
    drop(peeled);

    // Pieces: components of the other nodes; those without an anchor stay
    // unmarked.
    let mut chain = vec![false; n];
    let mut tree = vec![false; n];
    let mut seen = vec![false; n];
    let mut members = Vec::new();
    for start in 0..n {
        if junction[start] || seen[start] {
            continue;
        }
        members.clear();
        seen[start] = true;
        members.push(start as u32);
        let mut anchored = false;
        let mut at = 0;
        while at < members.len() {
            let node = members[at] as usize;
            at += 1;
            neighbours(input, &usable, node, &mut scratch);
            for next in &scratch {
                let next = *next as usize;
                if junction[next] {
                    anchored = true;
                } else if !seen[next] {
                    seen[next] = true;
                    members.push(next as u32);
                }
            }
        }
        if !anchored {
            continue;
        }
        for node in &members {
            let node = *node as usize;
            if unmarked[node] {
                continue;
            }
            if core[node] {
                chain[node] = true;
            } else {
                tree[node] = true;
            }
        }
    }
    (chain, tree)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Directed edges `(source, target, shared info)` as package adjacency,
    /// grouped by source.
    struct Graph {
        n: usize,
        first_out: Vec<u32>,
        first_in: Vec<u32>,
        in_edges: Vec<u32>,
        source: Vec<u32>,
        target: Vec<u32>,
        shared: Vec<u32>,
    }

    fn graph(n: usize, mut edges: Vec<(u32, u32, u32)>) -> Graph {
        edges.sort();
        let mut first_out = vec![0u32; n + 1];
        let mut first_in = vec![0u32; n + 1];
        for (s, t, _) in &edges {
            first_out[*s as usize + 1] += 1;
            first_in[*t as usize + 1] += 1;
        }
        for i in 1..=n {
            first_out[i] += first_out[i - 1];
            first_in[i] += first_in[i - 1];
        }
        let mut at = first_in.clone();
        let mut in_edges = vec![0u32; edges.len()];
        for (index, (_, t, _)) in edges.iter().enumerate() {
            in_edges[at[*t as usize] as usize] = index as u32;
            at[*t as usize] += 1;
        }
        Graph {
            n,
            first_out,
            first_in,
            in_edges,
            source: edges.iter().map(|e| e.0).collect(),
            target: edges.iter().map(|e| e.1).collect(),
            shared: edges.iter().map(|e| e.2).collect(),
        }
    }

    /// Two-way segments `(a, b)`, each its own shared info.
    fn two_way(pairs: &[(u32, u32)]) -> Vec<(u32, u32, u32)> {
        pairs
            .iter()
            .enumerate()
            .flat_map(|(si, (a, b))| [(*a, *b, si as u32), (*b, *a, si as u32)])
            .collect()
    }

    fn kinds(g: &Graph, restricted: &[usize]) -> (Vec<bool>, Vec<bool>) {
        let costs = vec![1.0f32; g.source.len()];
        let mut flags = vec![false; g.n];
        for node in restricted {
            flags[*node] = true;
        }
        let input = PieceInput {
            node_count: g.n,
            edge_count: g.source.len(),
            first_out: &g.first_out,
            first_in: &g.first_in,
            in_edges: &g.in_edges,
            edge_source: &g.source,
            edge_target: &g.target,
            edge_cost: &costs,
            edge_shared_info: &g.shared,
            restricted: &flags,
        };
        classify(&input, &costs)
    }

    /// Junctions 0 and 1 joined directly and by two chains (2-3 and 4), with
    /// a tree 5-6 hanging off chain node 3 and a tree 7 off junction 0.
    fn sample() -> Vec<(u32, u32, u32)> {
        two_way(&[
            (0, 1),
            (0, 2),
            (2, 3),
            (3, 1),
            (0, 4),
            (4, 1),
            (3, 5),
            (5, 6),
            (0, 7),
        ])
    }

    #[test]
    fn chains_and_trees_are_marked() {
        let (chain, tree) = kinds(&graph(8, sample()), &[]);
        assert_eq!(chain, [false, false, true, true, true, false, false, false]);
        assert_eq!(tree, [false, false, false, false, false, true, true, true]);
    }

    #[test]
    fn restrictions_and_turns_without_a_u_turn_unmark_their_tree_and_chain_node() {
        let g = graph(8, sample());
        let marked = |(chain, tree): (Vec<bool>, Vec<bool>)| {
            (0..8)
                .filter(|node| chain[*node] || tree[*node])
                .collect::<Vec<_>>()
        };
        // A restriction via 6 unmarks 6, its parent 5 and chain node 3.
        assert_eq!(marked(kinds(&g, &[6])), [2, 4, 7]);
        // Via 2 unmarks 2 only; via junction 0 nothing.
        assert_eq!(marked(kinds(&g, &[2])), [3, 4, 5, 6, 7]);
        assert_eq!(marked(kinds(&g, &[0])), [2, 3, 4, 5, 6, 7]);
        // A second segment between 0 and 7 (a loop at the dead end).
        let mut edges = sample();
        edges.push((0, 7, 99));
        assert_eq!(marked(kinds(&graph(8, edges), &[])), [2, 3, 4, 5, 6]);
        // A self-loop at 5.
        let mut edges = sample();
        edges.push((5, 5, 98));
        assert_eq!(marked(kinds(&graph(8, edges), &[])), [2, 4, 6, 7]);
        // A second segment between chain nodes 2 and 3.
        let mut edges = sample();
        edges.push((2, 3, 97));
        assert_eq!(marked(kinds(&graph(8, edges), &[])), [4, 5, 6, 7]);
    }

    #[test]
    fn a_cycle_without_a_junction_is_not_marked() {
        let (chain, tree) = kinds(&graph(4, two_way(&[(0, 1), (1, 2), (2, 0), (2, 3)])), &[]);
        assert!(chain.iter().chain(&tree).all(|marked| !marked));
    }
}

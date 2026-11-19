//! Builds the request sections (see `wayfarer_package::request`)
//! from the road graph.

use petgraph::graph::NodeIndex;
use rayon::prelude::*;
use rustc_hash::FxHashMap;
use wayfarer_package::RoadEdge;
use wayfarer_package::codec;
use wayfarer_package::request::*;
use wayfarer_road_graph::{RoadGraph, opposing_edge_indices};

use wayfarer_package::format::{PackageWriter, RoutingPackageError, Section, profile_bit};

/// Inputs from the package build, in compact order.
pub(crate) struct RequestBuildInput<'a> {
    pub graph: &'a RoadGraph,
    /// Road graph node index per compact node.
    pub node_raw: &'a [u32],
    pub first_out: &'a [u32],
    pub edge_source: &'a [u32],
    pub edge_target: &'a [u32],
    pub edge_ids: &'a [u64],
    /// Compact edge per edge id less `edge_id_base` (`u32::MAX` for none).
    pub edge_id_base: u64,
    pub edge_id_lookup: &'a [u32],
    /// Road graph edge index per compact edge (build time only).
    pub edge_raw: &'a [u32],
    pub edge_shared_info: &'a [u32],
    pub edge_flags: &'a [u8],
    pub edge_profile_access: &'a [u8],
    /// Road graph shared-info index per compact shared info (in id order).
    pub shared_raw: &'a [u32],
}

/// Built request sections, ready for the writer.
#[derive(Default)]
pub(crate) struct RequestSections {
    meta: Vec<u64>,
    grid: Vec<f64>,
    cell_start: Vec<u32>,
    entries: Vec<u32>,
    node_id_delta: Vec<u32>,
    node_id_wide: Vec<i64>,
    component_exceptions: Vec<u32>,
    component_sizes: Vec<u32>,
    shared_id_gaps: Vec<u64>,
    shared_attr_class: Vec<u32>,
    class_start: Vec<u32>,
    class_bytes: Vec<u8>,
    string_start: Vec<u32>,
    string_bytes: Vec<u8>,
    opposing_edge: Vec<u32>,
    opposing_value: Vec<u32>,
    leg_opposing_edge: Vec<u32>,
    leg_opposing_value: Vec<u32>,
    roundabout_exits: Vec<u32>,
    reach_saturated: Vec<u8>,
    mode_mask_exceptions: Vec<u32>,
    tier_exceptions: Vec<u32>,
    conditional_shared: Vec<u32>,
    conditional_start: Vec<u32>,
    conditional_pool: Vec<u32>,
}

impl RequestSections {
    pub(crate) fn byte_len(&self) -> usize {
        (self.meta.len() + self.grid.len() + self.node_id_wide.len() + self.shared_id_gaps.len())
            * 8
            + self.reach_saturated.len()
            + (self.cell_start.len()
                + self.entries.len()
                + self.node_id_delta.len()
                + self.component_exceptions.len()
                + self.component_sizes.len()
                + self.shared_attr_class.len()
                + self.class_start.len()
                + self.string_start.len()
                + self.opposing_edge.len()
                + self.opposing_value.len()
                + self.leg_opposing_edge.len()
                + self.leg_opposing_value.len()
                + self.roundabout_exits.len()
                + self.mode_mask_exceptions.len()
                + self.tier_exceptions.len()
                + self.conditional_shared.len()
                + self.conditional_start.len()
                + self.conditional_pool.len())
                * 4
            + self.class_bytes.len()
            + self.string_bytes.len()
    }

    /// The attribute class of each shared info.
    pub(crate) fn shared_attr_class(&self) -> &[u32] {
        &self.shared_attr_class
    }

    pub(crate) fn class_count(&self) -> usize {
        self.class_start.len().saturating_sub(1)
    }

    pub(crate) fn opposing_exception_count(&self) -> usize {
        self.opposing_edge.len()
    }

    pub(crate) fn mode_mask_exception_count(&self) -> usize {
        self.mode_mask_exceptions.len() / 2
    }

    pub(crate) fn tier_exception_count(&self) -> usize {
        self.tier_exceptions.len() / 2
    }

    pub(crate) fn put(self, writer: &mut PackageWriter) {
        writer.put(Section::RequestMeta, self.meta);
        writer.put(Section::RequestGrid, self.grid);
        writer.put(Section::GridCellStart, self.cell_start);
        writer.put(Section::GridEntries, self.entries);
        writer.put(Section::NodeIdDelta, self.node_id_delta);
        writer.put(Section::NodeIdWide, self.node_id_wide);
        writer.put(Section::ComponentExceptions, self.component_exceptions);
        writer.put(Section::ComponentSizes, self.component_sizes);
        writer.put(Section::NodeReachSaturated, self.reach_saturated);
        writer.put(Section::SharedInfoIdGaps, self.shared_id_gaps);
        writer.put(Section::SharedAttrClass, self.shared_attr_class);
        writer.put(Section::AttrClassStart, self.class_start);
        writer.put(Section::AttrClassBytes, self.class_bytes);
        writer.put(Section::StringStart, self.string_start);
        writer.put(Section::StringBytes, self.string_bytes);
        writer.put(Section::OpposingExceptionEdge, self.opposing_edge);
        writer.put(Section::OpposingExceptionValue, self.opposing_value);
        writer.put(Section::LegOpposingExceptionEdge, self.leg_opposing_edge);
        writer.put(Section::LegOpposingExceptionValue, self.leg_opposing_value);
        writer.put(Section::NodeRoundaboutExits, self.roundabout_exits);
        writer.put(Section::ModeMaskExceptions, self.mode_mask_exceptions);
        writer.put(Section::TierExceptions, self.tier_exceptions);
        writer.put(Section::ConditionalShared, self.conditional_shared);
        writer.put(Section::ConditionalStart, self.conditional_start);
        writer.put(Section::ConditionalPool, self.conditional_pool);
    }
}

/// Fits ids into u32 deltas from their minimum.
fn deltas<T: Copy + Into<i128>>(ids: &[T]) -> Option<(i128, Vec<u32>)> {
    let base = ids.iter().map(|id| (*id).into()).min().unwrap_or(0);
    let deltas = ids
        .iter()
        .map(|id| {
            u32::try_from((*id).into() - base)
                .ok()
                .filter(|v| *v != NONE_U32)
        })
        .collect::<Option<Vec<_>>>()?;
    Some((base, deltas))
}

pub(crate) fn build_request_sections(
    input: &RequestBuildInput<'_>,
) -> Result<RequestSections, RoutingPackageError> {
    let graph = input.graph;
    let edge_count = input.edge_ids.len();
    let shared_count = input.shared_raw.len();
    let node_count = input.node_raw.len();
    let node = |compact: usize| &graph.graph[NodeIndex::new(input.node_raw[compact] as usize)];
    let road_edge = |compact: usize| -> &RoadEdge {
        &graph.graph.raw_edges()[input.edge_raw[compact] as usize].weight
    };
    let shared_info = |shared: usize| {
        graph
            .shared_edge_infos
            .at(input.shared_raw[shared] as usize)
    };

    // Node ids (u32 deltas when they fit) and roundabout exits.
    let node_ids = (0..node_count).map(|at| node(at).id).collect::<Vec<_>>();
    let (node_base, node_id_delta, node_id_wide) = match deltas(&node_ids) {
        Some((base, deltas)) => (base as i64, deltas, Vec::new()),
        None => (0, Vec::new(), node_ids.clone()),
    };
    let roundabout_exits = (0..node_count)
        .filter(|at| node(*at).roundabout_exit_count != 0)
        .flat_map(|at| [at as u32, u32::from(node(at).roundabout_exit_count)])
        .collect::<Vec<_>>();

    // Components: the road graph's component ids, and each component's node
    // count over the whole graph.
    let mut size_by_component = FxHashMap::<u32, usize>::default();
    for road_node in graph.graph.node_weights() {
        if let Some(component) = road_node.component_id {
            *size_by_component.entry(component).or_default() += 1;
        }
    }
    let components = (0..node_count)
        .map(|at| {
            node(at)
                .component_id
                .filter(|component| *component != NONE_U32)
                .unwrap_or(NONE_U32)
        })
        .collect::<Vec<_>>();
    let mut id_counts = FxHashMap::<u32, usize>::default();
    let mut component_sizes = FxHashMap::<u32, u32>::default();
    for component in &components {
        *id_counts.entry(*component).or_default() += 1;
        let size = (*component != NONE_U32)
            .then(|| size_by_component.get(component).copied())
            .flatten()
            .and_then(|size| u32::try_from(size).ok())
            .filter(|size| *size != NONE_U32)
            .unwrap_or(NONE_U32);
        component_sizes.insert(*component, size);
    }
    // Most nodes share one component (the main network): store its id once
    // and the other nodes as exceptions.
    let default_component = id_counts
        .into_iter()
        .max_by_key(|(component, count)| (*count, *component))
        .map_or(NONE_U32, |(component, _)| component);
    let component_exceptions = components
        .iter()
        .enumerate()
        .filter(|(_, component)| **component != default_component)
        .flat_map(|(node, component)| [node as u32, *component])
        .collect::<Vec<_>>();
    let mut component_sizes = component_sizes.into_iter().collect::<Vec<_>>();
    component_sizes.sort_unstable();
    let component_sizes = component_sizes
        .into_iter()
        .flat_map(|(component, size)| [component, size])
        .collect::<Vec<_>>();

    // Shared-info ids: shared infos are numbered in id order, so shared info
    // `i` has id base + i + the ids skipped up to it (the gaps list).
    let shared_ids = (0..shared_count)
        .map(|shared| shared_info(shared).shared_edge_info_id)
        .collect::<Vec<_>>();
    if shared_ids.windows(2).any(|pair| pair[0] >= pair[1]) {
        return Err(RoutingPackageError::Invalid(
            "shared infos are not in increasing id order".into(),
        ));
    }
    let shared_base = shared_ids.first().copied().unwrap_or(0);
    let mut shared_id_gaps = Vec::new();
    let mut skipped = 0u64;
    for shared in 1..shared_count {
        let step = shared_ids[shared] - shared_ids[shared - 1];
        if step > 1 {
            skipped += step - 1;
            shared_id_gaps.extend([shared as u64, skipped]);
        }
    }
    drop(shared_ids);

    // Attribute classes per shared info, in compact shared-info order, and
    // the conditional tags (sharing the string table). A package class is a
    // function of the graph's attribute class alone, so each graph class is
    // encoded once, at its first use: `table.class` of a key it has seen adds
    // nothing, so classes and strings keep their first-use numbering.
    let mut table = StringsAndClasses::default();
    let mut shared_attr_class = Vec::with_capacity(shared_count);
    let mut package_class_of = vec![u32::MAX; graph.shared_edge_infos.class_count()];
    let mut conditional_shared = Vec::new();
    let mut conditional_start = vec![0u32];
    let mut conditional_pool = Vec::new();
    for shared in 0..shared_count {
        let info = shared_info(shared);
        let mut class = package_class_of[info.class as usize];
        if class == u32::MAX {
            // `attrs` has no id, way id or geometry. Routing reads no road
            // hierarchy (since 2026-10-08), so none is stored.
            let mut stripped = info.attrs.clone();
            stripped.hierarchy_tier = None;
            let leg = codec::to_vec(&stripped).map_err(|error| {
                RoutingPackageError::Unsupported(format!("shared info encoding: {error}"))
            })?;
            class = table.class(ClassKey {
                attrs: SharedAttrs::from_info(info.attrs),
                leg,
            })?;
            package_class_of[info.class as usize] = class;
        }
        shared_attr_class.push(class);
        let info = info.attrs;
        if !info.conditional_restrictions.is_empty() {
            conditional_shared.push(shared as u32);
            for tag in &info.conditional_restrictions {
                let id = table.string(tag);
                conditional_pool.push(id);
            }
            conditional_start.push(conditional_pool.len() as u32);
        }
    }

    // Representative edge per shared info (its lowest compact edge).
    let mut representative = vec![NONE_U32; shared_count];
    for edge in 0..edge_count {
        let shared = input.edge_shared_info[edge] as usize;
        if representative[shared] == NONE_U32 {
            representative[shared] = edge as u32;
        }
    }
    // Grid entries: each shared info's representative, plus any edge the
    // representative's endpoints do not reach (it is then its own entry).
    let mut entry_edges = Vec::<(u32, u32)>::new(); // (edge, shared)
    for edge in 0..edge_count {
        let shared = input.edge_shared_info[edge];
        let rep = representative[shared as usize] as usize;
        let (a, b) = (input.edge_source[rep], input.edge_target[rep]);
        let source = input.edge_source[edge];
        if edge == rep || (source != a && source != b) {
            entry_edges.push((edge as u32, shared));
        }
    }

    // Grid over the shapes' extent.
    let (mut min_lat, mut min_lng, mut max_lat, mut max_lng) = (
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    );
    for shared in 0..shared_count {
        for (lat, lng) in shared_info(shared).geometry.iter() {
            min_lat = min_lat.min(lat);
            max_lat = max_lat.max(lat);
            min_lng = min_lng.min(lng);
            max_lng = max_lng.max(lng);
        }
    }
    if min_lat > max_lat {
        (min_lat, min_lng, max_lat, max_lng) = (0.0, 0.0, 0.0, 0.0);
    }
    let grid = Grid {
        min_lat,
        min_lng,
        cell: GRID_CELL_DEG,
        rows: (((max_lat - min_lat) / GRID_CELL_DEG).floor() as usize + 1).max(1),
        cols: (((max_lng - min_lng) / GRID_CELL_DEG).floor() as usize + 1).max(1),
    };
    let mut pairs = entry_edges
        .par_iter()
        .flat_map_iter(|(edge, shared)| {
            let mut cells = Vec::new();
            let points = shared_info(*shared as usize).geometry;
            for index in 1..points.len() {
                let (a, b) = (points.get(index - 1), points.get(index));
                let (rows, cols) =
                    grid.cells(a.0.min(b.0), a.1.min(b.1), a.0.max(b.0), a.1.max(b.1));
                for row in rows {
                    for col in cols.clone() {
                        cells.push((row * grid.cols + col) as u32);
                    }
                }
            }
            cells.sort_unstable();
            cells.dedup();
            cells.into_iter().map(move |cell| (cell, *edge))
        })
        .collect::<Vec<_>>();
    pairs.par_sort_unstable();
    let cell_count = grid.rows * grid.cols;
    let mut cell_start = vec![0u32; cell_count + 1];
    for (cell, _) in &pairs {
        cell_start[*cell as usize + 1] += 1;
    }
    for cell in 0..cell_count {
        cell_start[cell + 1] += cell_start[cell];
    }
    let entries = pairs.into_iter().map(|(_, edge)| edge).collect::<Vec<_>>();

    // Opposing edges. Correlation pairs edges as the road graph does, by
    // (shared info, segment, direction) (the last edge of a key wins); legs
    // report the edge's own `opposing_edge_id`. Both are stored as
    // exceptions to the adjacency rule.
    // By road graph edge index.
    let correlation_opposing = opposing_edge_indices(graph);
    let derived = (0..edge_count as u32)
        .into_par_iter()
        .map(|edge| {
            derived_opposing(
                input.first_out,
                input.edge_source,
                input.edge_target,
                input.edge_shared_info,
                input.edge_flags,
                edge,
            )
            .map_or(NONE_U64, |opposing| input.edge_ids[opposing as usize])
        })
        .collect::<Vec<_>>();
    // (edges, values) where `actual` differs from the rule; an opposing edge
    // the graph does not hold reads as none.
    let exceptions = |actual: &(dyn Fn(usize) -> u64 + Sync)| -> (Vec<u32>, Vec<u32>) {
        (0..edge_count as u32)
            .into_par_iter()
            .filter_map(|edge| {
                let actual = actual(edge as usize);
                let value = if actual == NONE_U64 {
                    NONE_U32
                } else {
                    actual
                        .checked_sub(input.edge_id_base)
                        .and_then(|slot| input.edge_id_lookup.get(slot as usize))
                        .copied()
                        .unwrap_or(NONE_U32)
                };
                let actual = if value == NONE_U32 { NONE_U64 } else { actual };
                (derived[edge as usize] != actual).then_some((edge, value))
            })
            .collect::<Vec<_>>()
            .into_iter()
            .unzip()
    };
    let (opposing_edge, opposing_value) = exceptions(&|edge| match correlation_opposing
        [input.edge_raw[edge] as usize]
    {
        u32::MAX => NONE_U64,
        raw => graph.graph.raw_edges()[raw as usize].weight.edge_id,
    });
    let (leg_opposing_edge, leg_opposing_value) =
        exceptions(&|edge| road_edge(edge).opposing_edge_id.unwrap_or(NONE_U64));
    drop(derived);
    drop(correlation_opposing);

    // Leg record exceptions: masks the profile bits do not rebuild. Tier
    // exceptions stay empty: no hierarchy tier is stored.
    let mut mode_mask_exceptions = Vec::new();
    let tier_exceptions = Vec::new();
    for edge in 0..edge_count {
        let road = road_edge(edge);
        if !exact_mode_mask(road.mobility_mode_mask) {
            mode_mask_exceptions.extend([edge as u32, road.mobility_mode_mask]);
        }
    }

    let string_start = if table.string_start.is_empty() {
        vec![0]
    } else {
        table.string_start
    };
    let class_start = if table.class_start.is_empty() {
        vec![0]
    } else {
        table.class_start
    };
    if super::trace() {
        eprintln!(
            "request sections: grid {}x{} cells, {} entries; {} classes ({} B); {} strings ({} B); \
             {} component exceptions; {} shared-id gaps; {} leg opposing exceptions, \
             {} roundabout nodes, {} mode-mask and {} tier exceptions",
            grid.rows,
            grid.cols,
            entries.len(),
            class_start.len() - 1,
            table.class_bytes.len(),
            string_start.len() - 1,
            table.string_bytes.len(),
            component_exceptions.len() / 2,
            shared_id_gaps.len() / 2,
            leg_opposing_edge.len(),
            roundabout_exits.len() / 2,
            mode_mask_exceptions.len() / 2,
            tier_exceptions.len() / 2,
        );
    }
    Ok(RequestSections {
        meta: vec![
            REQUEST_LAYOUT,
            node_base as u64,
            shared_base,
            u64::from(default_component),
            0,
            SATURATION_CAP as u64,
        ],
        grid: vec![
            grid.min_lat,
            grid.min_lng,
            grid.cell,
            grid.rows as f64,
            grid.cols as f64,
        ],
        cell_start,
        entries,
        node_id_delta,
        node_id_wide,
        component_exceptions,
        component_sizes,
        shared_id_gaps,
        shared_attr_class,
        class_start,
        class_bytes: table.class_bytes,
        string_start,
        string_bytes: table.string_bytes,
        opposing_edge,
        opposing_value,
        leg_opposing_edge,
        leg_opposing_value,
        roundabout_exits,
        reach_saturated: reach_saturated(input, SATURATION_CAP),
        mode_mask_exceptions,
        tier_exceptions,
        conditional_shared,
        conditional_start,
        conditional_pool,
    })
}

/// Per node, a bit per profile slot: the node lies in a strongly connected
/// component (over edges the profile may use) with at least `cap` edges, so
/// the correlator's reach count from it is `cap` in both directions (the
/// breadth-first search reaches every edge of the component).
fn reach_saturated(input: &RequestBuildInput<'_>, cap: usize) -> Vec<u8> {
    let node_count = input.first_out.len() - 1;
    let slots = PROFILES
        .iter()
        .map(|profile| profile_bit(*profile))
        .collect::<Vec<_>>();
    // Profiles whose access bits agree on every edge have the same answer,
    // so each distinct bit column is searched once (its first profile's).
    let access = input.edge_profile_access;
    let same_column = |a: u8, b: u8| {
        access
            .par_iter()
            .all(|bits| (bits & a == 0) == (bits & b == 0))
    };
    let first_alike = slots
        .iter()
        .enumerate()
        .map(|(at, bit)| {
            slots[..at]
                .iter()
                .position(|earlier| same_column(*earlier, *bit))
                .unwrap_or(at)
        })
        .collect::<Vec<_>>();
    let per_slot = slots
        .par_iter()
        .enumerate()
        .map(|(at, bit)| (first_alike[at] == at).then(|| large_component_nodes(input, *bit, cap)))
        .collect::<Vec<_>>();
    (0..node_count)
        .map(|node| {
            slots
                .iter()
                .zip(&first_alike)
                .filter(|(_, first)| per_slot[**first].as_ref().is_some_and(|large| large[node]))
                .fold(0u8, |bits, (bit, _)| bits | bit)
        })
        .collect()
}

/// Nodes of strongly connected components with at least `min_edges` edges,
/// over edges whose access bits include `bit` (iterative Tarjan).
fn large_component_nodes(input: &RequestBuildInput<'_>, bit: u8, min_edges: usize) -> Vec<bool> {
    let (first_out, targets, access) = (
        input.first_out,
        input.edge_target,
        input.edge_profile_access,
    );
    let node_count = first_out.len() - 1;
    let mut index = vec![NONE_U32; node_count];
    let mut low = vec![0u32; node_count];
    let mut on_stack = vec![false; node_count];
    let mut stack = Vec::<u32>::new();
    let mut calls = Vec::<(u32, u32)>::new(); // (node, next edge to scan)
    let (mut next_index, mut component_count) = (0u32, 0u32);
    for root in 0..node_count as u32 {
        if index[root as usize] != NONE_U32 {
            continue;
        }
        index[root as usize] = next_index;
        low[root as usize] = next_index;
        next_index += 1;
        stack.push(root);
        on_stack[root as usize] = true;
        calls.push((root, first_out[root as usize]));
        while let Some(&(node, mut cursor)) = calls.last() {
            let end = first_out[node as usize + 1];
            let mut descended = false;
            while cursor < end {
                let edge = cursor as usize;
                cursor += 1;
                if access[edge] & bit == 0 {
                    continue;
                }
                let next = targets[edge];
                if index[next as usize] == NONE_U32 {
                    index[next as usize] = next_index;
                    low[next as usize] = next_index;
                    next_index += 1;
                    stack.push(next);
                    on_stack[next as usize] = true;
                    descended = true;
                    if let Some(top) = calls.last_mut() {
                        top.1 = cursor;
                    }
                    calls.push((next, first_out[next as usize]));
                    break;
                } else if on_stack[next as usize] {
                    low[node as usize] = low[node as usize].min(index[next as usize]);
                }
            }
            if descended {
                continue;
            }
            calls.pop();
            if let Some(&(parent, _)) = calls.last() {
                low[parent as usize] = low[parent as usize].min(low[node as usize]);
            }
            if low[node as usize] == index[node as usize] {
                // A popped member's `low` is never read again, so it holds
                // the member's component from here.
                while let Some(member) = stack.pop() {
                    on_stack[member as usize] = false;
                    low[member as usize] = component_count;
                    if member == node {
                        break;
                    }
                }
                component_count += 1;
            }
        }
    }
    drop((index, on_stack, stack, calls));
    let component = low;
    let mut internal_edges = vec![0u32; component_count as usize];
    for node in 0..node_count {
        for edge in first_out[node] as usize..first_out[node + 1] as usize {
            if access[edge] & bit != 0 && component[targets[edge] as usize] == component[node] {
                internal_edges[component[node] as usize] += 1;
            }
        }
    }
    component
        .iter()
        .map(|component| internal_edges[*component as usize] as usize >= min_edges)
        .collect()
}

// ---------------------------------------------------------------------------
// Query
// ---------------------------------------------------------------------------

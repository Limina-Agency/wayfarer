use std::collections::BTreeSet;

use rayon::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};

use crate::cancellation::RouteCancellation;
use crate::correlator::{CandidateSearch, Correlator};
use crate::costing::CostingPolicy;
use crate::graph_view::PackageGraphView;
use crate::limits::MAX_MATRIX_CELLS;
use crate::package::destination::relax_split_costs;
use crate::package::search::SearchState;
use crate::package::search::analysis::{
    AnalysisPath, AnalysisPoint, AnalysisQuery, MatrixTargetCandidate, matrix_cell, matrix_row,
    overlay_split_costs, override_bounds, request_cost_overrides, request_duration_overrides,
    resolve_edge_id, resolve_location,
};
use crate::package::search::patch::{
    PatchCosts, PatchCounters, PatchGraph, PatchOutcome, PatchPath, PatchQuery, analysis_overlay,
    matrix_row as patch_row, run as run_patch,
};
use crate::package::{SearchGraph, profile_bit};
use crate::search::RoutingEngine;
use crate::trip::{
    apply_query_overlays, invalid_point, validate_query_options, validate_query_overlays,
};
use crate::types::{
    CorrelatedLocation, CorrelationPolicy, MobilityProfile, RouteError, RouteMatrixCell,
    RouteMatrixCellStatus, RouteMatrixRequest, RouteMatrixResult,
};

impl RouteMatrixRequest {
    /// Checks the request against [`crate::limits`]: at most
    /// [`MAX_MATRIX_CELLS`] cells, valid coordinates, the overlays and the
    /// options a route accepts (candidate limit, correlation distance,
    /// costing values, edge constraints and dynamic overlay caps). [`RoutingEngine::matrix`] runs
    /// it first; a host that wants a request-level error instead of failed
    /// cells calls it before.
    pub fn validate(&self) -> Result<(), RouteError> {
        if self
            .sources
            .len()
            .checked_mul(self.targets.len())
            .is_none_or(|cells| cells > MAX_MATRIX_CELLS)
        {
            return Err(RouteError::RequestTooLarge {
                what: "matrix cells",
            });
        }
        if self
            .sources
            .iter()
            .chain(&self.targets)
            .copied()
            .any(invalid_point)
        {
            return Err(RouteError::InvalidRequest {
                reason: "location coordinates",
            });
        }
        validate_query_overlays(&self.overlays)?;
        validate_query_options(&self.options)
    }
}

impl RoutingEngine {
    /// Travel between every source and every target, one cell per pair
    /// (source-major). A request that fails [`RouteMatrixRequest::validate`]
    /// fails every cell with the reason, except one with more than
    /// [`MAX_MATRIX_CELLS`] cells, which gets no cells at all.
    pub fn matrix(&self, mut request: RouteMatrixRequest) -> RouteMatrixResult {
        let source_count = request.sources.len();
        let target_count = request.targets.len();
        let request_error = request.validate().err();
        if source_count.saturating_mul(target_count) > MAX_MATRIX_CELLS {
            return RouteMatrixResult {
                source_count,
                target_count,
                cells: Vec::new(),
                source_expansion_count: 0,
                visited_edge_count: 0,
                settled_node_count: 0,
                source_expanded_correlation_distances_m: Vec::new(),
                target_expanded_correlation_distances_m: Vec::new(),
            };
        }
        let mut cells = matrix_cell_grid(source_count, target_count);

        let graph_error = if request_error.is_some() {
            request_error
        } else if !self.route_graph_view().is_ready() {
            Some(RouteError::GraphNotReady)
        } else if !self
            .route_graph_view()
            .has_route_edges_for_profile(request.profile)
        {
            Some(RouteError::ImpossibleMobilityProfile {
                profile: request.profile,
            })
        } else if let Err(error) = self.validate_sar(&request.options, request.profile) {
            Some(error)
        } else if let Err(error) = apply_query_overlays(
            self,
            &request.overlays,
            request.profile,
            request.departure_at_ms,
            &mut request.options,
        ) {
            Some(error)
        } else {
            request.options.cancellation.check().err()
        };
        if let Some(error) = graph_error {
            for cell in &mut cells {
                apply_matrix_error(cell, &error);
            }
            return RouteMatrixResult {
                source_count,
                target_count,
                cells,
                source_expansion_count: 0,
                visited_edge_count: 0,
                settled_node_count: 0,
                source_expanded_correlation_distances_m: Vec::new(),
                target_expanded_correlation_distances_m: Vec::new(),
            };
        }

        let policy = CorrelationPolicy::from(&request.options);
        let correlator = Correlator::new(self.route_graph_view());
        // Endpoints correlate in parallel, on the matrix pool: 1,500 Krakow
        // points take about 0.28 s on one thread, as long as their 1x1,500
        // matrix's cells.
        let (source_searches, target_searches) = self.package().in_matrix_pool(|| {
            let correlate = |points: &[crate::types::GeoPoint]| {
                points
                    .par_iter()
                    .map(|point| {
                        correlator.search_expanding(
                            *point,
                            request.profile,
                            &policy,
                            &Default::default(),
                        )
                    })
                    .collect::<Vec<_>>()
            };
            (correlate(&request.sources), correlate(&request.targets))
        });
        let source_expanded_correlation_distances_m =
            expanded_correlation_distances(&source_searches);
        let target_expanded_correlation_distances_m =
            expanded_correlation_distances(&target_searches);

        for (source_index, search) in source_searches.iter().enumerate() {
            if search.candidates.is_empty() {
                for target_index in 0..target_count {
                    apply_matrix_error(
                        cell_mut(&mut cells, target_count, source_index, target_index),
                        &RouteError::NoCandidate { endpoint: "origin" },
                    );
                }
            }
        }
        for (target_index, search) in target_searches.iter().enumerate() {
            if search.candidates.is_empty() {
                for source_index in 0..source_count {
                    let cell = cell_mut(&mut cells, target_count, source_index, target_index);
                    if cell.failure_reason.is_none() {
                        apply_matrix_error(
                            cell,
                            &RouteError::NoCandidate {
                                endpoint: "destination",
                            },
                        );
                    }
                }
            }
        }

        let source_candidates = source_searches
            .iter()
            .map(|search| search.candidates.as_slice())
            .collect::<Vec<_>>();
        let target_candidates = target_searches
            .iter()
            .map(|search| search.candidates.as_slice())
            .collect::<Vec<_>>();
        // Cells and rows search in parallel, on the package's matrix pool
        // (bounded by its memory profile).
        let strict = self
            .package()
            .confines_destination_only(&request.options, request.profile);
        // Tests compare with re-running the whole matrix relaxed.
        #[cfg(test)]
        let only_retried = !test_mode::full_relaxed_pass();
        #[cfg(not(test))]
        let only_retried = true;
        let outcome = self.package().in_matrix_pool(|| {
            let mut outcome = self.matrix_cells(
                &request,
                &source_candidates,
                &target_candidates,
                &mut cells,
                !strict,
                None,
                None,
                None,
            );
            // The strict pass opens the pockets of every source and target
            // for every cell; a cell whose path crossed the pocket of
            // another point is searched again with its own two only.
            if let Ok((expansions, visited, settled)) = &mut outcome
                && strict
            {
                match self.confine_to_own_pockets(
                    &request,
                    &source_candidates,
                    &target_candidates,
                    &mut cells,
                ) {
                    Ok((more_expansions, more_visited, more_settled)) => {
                        *expansions += more_expansions;
                        *visited += more_visited;
                        *settled += more_settled;
                    }
                    Err(error) => outcome = Err(error),
                }
            }
            // Cells that cannot connect without crossing a destination-only
            // road get the relaxed search, as routes do. Only those cells are
            // searched again (see `matrix_cells`' `retry`).
            if let Ok((expansions, visited, settled)) = &mut outcome
                && strict
                && cells.iter().any(retries_relaxed)
            {
                let retry = cells.iter().map(retries_relaxed).collect::<Vec<_>>();
                let mut relaxed = cells.clone();
                match self.matrix_cells(
                    &request,
                    &source_candidates,
                    &target_candidates,
                    &mut relaxed,
                    true,
                    only_retried.then_some(retry.as_slice()),
                    None,
                    None,
                ) {
                    Ok((more_expansions, more_visited, more_settled)) => {
                        *expansions += more_expansions;
                        *visited += more_visited;
                        *settled += more_settled;
                        take_relaxed_cells(&mut cells, relaxed);
                    }
                    Err(error) => outcome = Err(error),
                }
            }
            outcome
        });
        if !request.include_edge_ids {
            for cell in &mut cells {
                cell.edge_ids = Vec::new();
            }
        }
        match outcome {
            Ok((source_expansion_count, visited_edge_count, settled_node_count)) => {
                RouteMatrixResult {
                    source_count,
                    target_count,
                    cells,
                    source_expansion_count,
                    visited_edge_count,
                    settled_node_count,
                    source_expanded_correlation_distances_m,
                    target_expanded_correlation_distances_m,
                }
            }
            Err(error) => {
                for cell in &mut cells {
                    if cell.failure_reason.is_none() {
                        apply_matrix_error(cell, &error);
                    }
                }
                RouteMatrixResult {
                    source_count,
                    target_count,
                    cells,
                    source_expansion_count: 0,
                    visited_edge_count: 0,
                    settled_node_count: 0,
                    source_expanded_correlation_distances_m,
                    target_expanded_correlation_distances_m,
                }
            }
        }
    }
}

/// Each point's widened correlation distance, or nothing when none widened
/// (`RouteMatrixResult::source_expanded_correlation_distances_m`).
pub(crate) fn expanded_correlation_distances(searches: &[CandidateSearch]) -> Vec<Option<u32>> {
    if searches
        .iter()
        .all(|search| search.expanded_max_distance_m.is_none())
    {
        return Vec::new();
    }
    searches
        .iter()
        .map(|search| search.expanded_max_distance_m)
        .collect()
}

impl RoutingEngine {
    /// Fills every searchable cell from searches over the routing package, with
    /// per-edge block/avoid/multiplier options applied as exact cost
    /// overrides and a dynamic overlay as a patch (see
    /// `package::search::patch`; endpoint connectors belong to routes and are
    /// ignored). Non-default costing options run on a derived column (see
    /// `package::columns`). The error, for every searchable cell, is
    /// `GraphNotReady` without a package and `MissingEdgeMetadata` for a
    /// candidate edge the package does not hold.
    ///
    /// `retry` (row-major, one flag per cell) limits a relaxed pass to the
    /// cells the strict pass could not connect; the other cells keep whatever
    /// `cells` held. The relaxed query does not depend on the endpoints (no
    /// pockets are unblocked): pairwise searches only the flagged cells,
    /// which are independent. Rows search only the sources with a flagged
    /// cell and, without a visited-edge budget (which applies to a whole
    /// row), only towards their flagged targets: each target's path is exact
    /// whatever the others, though between paths of exactly equal cost a
    /// differently focused expansion could label another first (none did in
    /// the Krakow and Malopolskie matrices measured). The mode is chosen
    /// from the whole matrix either way.
    ///
    /// `confined` (strict passes only): the candidate edges whose
    /// destination-only pockets the pass unblocks, in place of every
    /// source's and target's; the query then does not depend on the
    /// endpoints either, so `retry` may limit such a pass too.
    ///
    /// `stop` (no dynamic overlay patch): rows, each stopping once
    /// `stop.after` of its targets have their final path (see
    /// [`matrix_row`]). The cells of the targets a row left open keep what
    /// `cells` held; their lower bounds go to `stop.bounds`.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn matrix_cells(
        &self,
        request: &RouteMatrixRequest,
        source_candidates: &[&[CorrelatedLocation]],
        target_candidates: &[&[CorrelatedLocation]],
        cells: &mut [RouteMatrixCell],
        allow_destination_only: bool,
        retry: Option<&[bool]>,
        confined: Option<&[u32]>,
        mut stop: Option<&mut RowStop>,
    ) -> Result<(usize, usize, usize), RouteError> {
        debug_assert!(retry.is_none() || allow_destination_only || confined.is_some());
        let stop_after = stop.as_ref().map(|stop| stop.after);
        let target_count = target_candidates.len();
        let retries = |source: usize, target: usize| {
            retry.is_none_or(|retry| retry[source * target_count + target])
        };
        let source_retries =
            |source: usize| retry.is_none_or(|_| (0..target_count).any(|t| retries(source, t)));
        // A relaxed row also searches only towards its retried targets, unless
        // a visited-edge budget makes the row's outcome depend on all of them.
        let retried_targets_only =
            retry.is_some() && request.options.max_visited_edge_count.is_none();
        let access = self.route_graph_view();
        let graph = access.routing_package();
        let overlay = request
            .options
            .dynamic_overlay
            .as_ref()
            .filter(|overlay| {
                !overlay.split_edge_ranges.is_empty() || !overlay.virtual_edges.is_empty()
            })
            .map(analysis_overlay);
        // SAR R1 and R2 open edges in a derived column; R2's entry and R4's
        // barriers are transition costs the view adds to turn costs.
        let handle = graph.request_column_sar(
            request.profile,
            &request.options.costing,
            allow_destination_only,
            crate::sar::edge_relaxations(&request.options, request.profile),
        );
        let view = graph
            .view()
            .with_transitions(crate::transitions::Transitions::of(
                &request.options,
                request.profile,
            ));
        let column = handle.column(&view);
        let resolve = |candidates: &&[CorrelatedLocation]| {
            let edges = candidates
                .iter()
                .map(|candidate| {
                    resolve_location(view, candidate).ok_or(RouteError::MissingEdgeMetadata {
                        edge_id: candidate.edge_id,
                    })
                })
                .collect::<Result<Vec<_>, _>>()?;
            let mut points = AnalysisPoint::of_candidates(candidates, edges);
            // Rows that stop early bound every candidate pair alike.
            if stop_after.is_some() {
                for point in &mut points {
                    point.approach = 0.0;
                }
            }
            Ok::<_, RouteError>(points)
        };
        let sources = source_candidates
            .iter()
            .map(resolve)
            .collect::<Result<Vec<_>, _>>()?;
        let targets = target_candidates
            .iter()
            .map(resolve)
            .collect::<Result<Vec<_>, _>>()?;
        let edges_of =
            |points: &Vec<AnalysisPoint>| points.iter().map(|point| point.edge).collect::<Vec<_>>();
        let source_edges = sources.iter().map(edges_of).collect::<Vec<_>>();
        let target_edges = targets.iter().map(edges_of).collect::<Vec<_>>();
        let candidates = targets
            .iter()
            .enumerate()
            .flat_map(|(target, points)| {
                points.iter().map(move |point| MatrixTargetCandidate {
                    target,
                    point: *point,
                })
            })
            .collect::<Vec<_>>();
        let costing = CostingPolicy::with_destination_only_policy(
            request.profile,
            &request.options,
            allow_destination_only,
        );
        let mut cost_overrides =
            request_cost_overrides(view, &request.options, &costing, request.profile);
        let mut durations = request_duration_overrides(
            view,
            column,
            &request.options,
            &costing,
            request.profile,
            &mut cost_overrides,
        );
        // A strict pass may start or end in the destination-only pockets of
        // its sources and targets (see `package::destination`).
        let unblocked = if allow_destination_only {
            FxHashMap::default()
        } else {
            let endpoints = match confined {
                Some(edges) => edges.to_vec(),
                None => source_edges
                    .iter()
                    .chain(&target_edges)
                    .flatten()
                    .copied()
                    .collect(),
            };
            graph.unblock_destination_pockets(
                &request.options,
                request.profile,
                endpoints,
                &mut cost_overrides,
                &mut durations,
            )
        };
        let column = column.with_durations(&durations);
        let split_cost = match &overlay {
            Some(overlay) => {
                let mut split_cost = overlay_split_costs(
                    view,
                    overlay,
                    &costing,
                    request.profile,
                    &mut cost_overrides,
                );
                relax_split_costs(&unblocked, &mut split_cost);
                split_cost
            }
            None => FxHashMap::default(),
        };
        // Discounted edges and fragments become gates of the potentials.
        let bounds = override_bounds(view, column, &cost_overrides, &split_cost);
        let heuristic_cost_per_m = bounds.heuristic_cost_per_m;
        let query = AnalysisQuery {
            column,
            profile_bit: profile_bit(request.profile),
            max_visited_edge_count: request.options.max_visited_edge_count,
            cancellation: &request.options.cancellation,
            cost_overrides,
            gates: bounds.gates.clone(),
            restriction_penalty: crate::sar::restriction_penalty(&request.options),
        };
        let edge_cost = |edge: u32| query.edge_cost(view, edge);
        let costs = PatchCosts {
            column,
            edge_cost: &edge_cost,
            split_cost: &split_cost,
        };
        let resolve_id = |edge_id: u64| resolve_edge_id(view, edge_id);
        let patch = overlay
            .as_ref()
            .map(|overlay| PatchGraph::new(view, overlay, &costs, &resolve_id))
            .filter(|patch| !patch.is_empty());
        let patch_template = PatchQuery {
            costs: PatchCosts {
                column,
                edge_cost: &edge_cost,
                split_cost: &split_cost,
            },
            // The scaled fallback does not cover the patch's bounds at
            // virtual points, which go along edges at column cost.
            potentials: !bounds.fell_back(),
            gates: &bounds.gates,
            profile_bit: profile_bit(request.profile),
            origins: &[],
            destinations: &[],
            origin_edges: &[],
            destination_edges: &[],
            origin_targets: &[],
            destination_targets: &[],
            max_visited_edge_count: request.options.max_visited_edge_count,
            cancellation: &request.options.cancellation,
            heuristic_cost_per_m,
            bidirectional: true,
            restriction_penalty: crate::sar::restriction_penalty(&request.options),
            pieces: None,
        };
        let patch_path = |patch: &PatchGraph<'_>, path: PatchPath| MatrixPath {
            edge_ids: patch.step_edge_ids(view, &path.steps),
            distance_m: path.distance_m,
            duration_s: path.duration_s,
            cost: path.cost,
        };
        let compact_path = |path: AnalysisPath| MatrixPath {
            edge_ids: path
                .edges
                .iter()
                .map(|edge| view.edge_id(*edge as usize))
                .collect(),
            distance_m: path.distance_m,
            duration_s: path.duration_s,
            cost: path.cost,
        };

        let searchable_targets = targets.iter().filter(|points| !points.is_empty()).count();
        // A cell across two weak components has no path: it is not searched
        // (an unreachable target would otherwise keep a row's expansion going
        // until it had flooded the whole component) and is reported
        // `Disconnected` below, as before. Overlay connectors may join
        // components, so with them every cell is searched.
        // Roads a SAR request opens may join components too.
        let connected = if patch.as_ref().is_some_and(PatchGraph::has_connectors)
            || crate::sar::SarOpenEdges::of(&request.options).is_some()
        {
            vec![vec![true; target_count]; source_candidates.len()]
        } else {
            cell_connectivity(access, source_candidates, target_candidates)
        };
        // One point-to-point cell search, on `state`.
        let search_cell = |state: &mut SearchState, source: usize, target: usize| {
            match &patch {
                Some(patch) => {
                    let query = patch_template.between(
                        source_candidates[source],
                        &source_edges[source],
                        target_candidates[target],
                        &target_edges[target],
                    );
                    let mut counters = PatchCounters::default();
                    let outcome = run_patch(view, state, patch, &query, &mut counters);
                    let (path, budget_exceeded) = match outcome {
                        PatchOutcome::Found(path) => (Some(patch_path(patch, path)), false),
                        PatchOutcome::NoPath => (None, false),
                        // Out of budget, a cell keeps its best path
                        // so far, as a row does.
                        PatchOutcome::BudgetExceeded { best, .. } => {
                            (best.map(|(path, _)| patch_path(patch, path)), true)
                        }
                    };
                    (
                        path,
                        counters.visited_edge_count,
                        counters.settled_edge_count,
                        budget_exceeded,
                    )
                }
                None => {
                    let cell = matrix_cell(
                        view,
                        state,
                        &query,
                        heuristic_cost_per_m,
                        source_candidates[source],
                        &sources[source],
                        target_candidates[target],
                        &targets[target],
                    );
                    (
                        cell.path.map(compact_path),
                        cell.counters.visited_edge_count,
                        cell.counters.settled_edge_count,
                        cell.budget_exceeded,
                    )
                }
            }
        };
        // Cells search in parallel, in runs of cells of one source on one
        // state, so that each run computes the potential from its source
        // once and the following cells reuse its memo (see
        // `package::cch::PotentialScratch`). `pairs` come by source.
        let search_cells = |pairs: &[(usize, usize)]| {
            let run = pairs
                .len()
                .div_ceil(MATRIX_RUNS_PER_THREAD * rayon::current_num_threads())
                .clamp(1, MATRIX_MAX_RUN);
            let mut runs = Vec::new();
            let mut start = 0;
            for end in 1..=pairs.len() {
                if end == pairs.len() || end - start == run || pairs[end].0 != pairs[start].0 {
                    runs.push(start..end);
                    start = end;
                }
            }
            runs.into_par_iter()
                .map(|run| {
                    let mut state = graph.take_state();
                    let cells = pairs[run]
                        .iter()
                        .map(|(source, target)| {
                            let cell = search_cell(&mut state, *source, *target);
                            state.reset();
                            cell
                        })
                        .collect::<Vec<_>>();
                    graph.return_state(state);
                    cells
                })
                .flatten_iter()
                .collect::<Vec<_>>()
        };
        debug_assert!(stop_after.is_none() || patch.is_none());
        let mode = if stop_after.is_some() {
            MatrixMode::Rows
        } else {
            matrix_mode(
                request.profile,
                searchable_targets,
                view.cch.is_some() && query.column.potential.is_some(),
            )
        };
        let rows = if mode == MatrixMode::Pairwise {
            // Few targets: one bidirectional A* search per cell. Each explores
            // little beyond the corridor between its two points, where a shared
            // one-to-many expansion has to cover a disc as wide as the costliest
            // target. Cells run in parallel.
            let pairs = (0..sources.len())
                .filter(|source| !sources[*source].is_empty())
                .flat_map(|source| {
                    (0..target_count)
                        .filter(|target| !targets[*target].is_empty())
                        .map(move |target| (source, target))
                })
                .filter(|(source, target)| connected[*source][*target] && retries(*source, *target))
                .collect::<Vec<_>>();
            let cells = search_cells(&pairs);
            let mut rows = sources
                .iter()
                .enumerate()
                .map(|(source, origins)| {
                    (!origins.is_empty() && searchable_targets > 0 && source_retries(source)).then(
                        || MatrixRowCells {
                            paths: vec![None; target_count],
                            budget_exceeded: vec![false; target_count],
                            visited_edge_count: 0,
                            settled_edge_count: 0,
                            open_bounds: Vec::new(),
                        },
                    )
                })
                .collect::<Vec<_>>();
            for ((source, target), (path, visited, settled, budget_exceeded)) in
                pairs.into_iter().zip(cells)
            {
                if let Some(row) = rows[source].as_mut() {
                    row.paths[target] = path;
                    row.budget_exceeded[target] = budget_exceeded;
                    row.visited_edge_count += visited;
                    row.settled_edge_count += settled;
                }
            }
            rows
        } else {
            // Many targets: one shared expansion per source row, in parallel.
            // A row runs until its last target is final, so one target no
            // source can reach (in a strict pass, one ringed by
            // destination-only roads) floods the whole component from every
            // source. Targets whose few reaching edges are known leave the
            // rows of the sources outside them (their cells find no path, as
            // the flood did). A visited-edge budget applies to a whole row,
            // so with one the rows keep every target.
            #[cfg(test)]
            let prune = !test_mode::keep_unreachable_targets();
            #[cfg(not(test))]
            let prune = true;
            // Rows that stop early stop before any flood of theirs (weak
            // components aside, which `connected` rules out) has a target to
            // wait for once they have enough, so they skip the pruning.
            let reaching = if prune
                && stop_after.is_none()
                && patch.is_none()
                && request.options.max_visited_edge_count.is_none()
            {
                // A pass limited to retried targets needs their sets only.
                let wanted = |target: usize| {
                    !retried_targets_only
                        || (0..sources.len()).any(|source| retries(source, target))
                };
                target_edges
                    .par_iter()
                    .enumerate()
                    .map(|(target, edges)| {
                        wanted(target)
                            .then(|| {
                                reaching_edges(
                                    view,
                                    edges,
                                    |edge| query.edge_cost(view, edge).is_finite(),
                                    &request.options.cancellation,
                                )
                            })
                            .flatten()
                    })
                    .collect::<Vec<_>>()
            } else {
                vec![None; target_count]
            };
            let may_reach = |source: usize, target: usize| {
                reaching[target].as_ref().is_none_or(|reaching| {
                    source_edges[source]
                        .iter()
                        .any(|edge| reaching.contains(edge))
                })
            };
            sources
                .par_iter()
                .enumerate()
                .zip(&connected)
                .map(|((source, origins), connected)| {
                    if origins.is_empty() || candidates.is_empty() || !source_retries(source) {
                        return None;
                    }
                    let mut state = graph.take_state();
                    let row = match &patch {
                        Some(patch) => {
                            let mut locations = Vec::new();
                            let mut edges = Vec::new();
                            let mut target_of = Vec::new();
                            for (target, points) in targets.iter().enumerate() {
                                if !connected[target] {
                                    continue;
                                }
                                for (location, point) in
                                    target_candidates[target].iter().zip(points)
                                {
                                    locations.push(location.clone());
                                    edges.push(point.edge);
                                    target_of.push(target);
                                }
                            }
                            let mut query = patch_template.between(
                                source_candidates[source],
                                &source_edges[source],
                                &locations,
                                &edges,
                            );
                            // Approach costs per target (`PatchQuery::approaches`).
                            query.destination_targets = &target_of;
                            let row = patch_row(
                                view,
                                &mut state,
                                patch,
                                &query,
                                &target_of,
                                target_count,
                            );
                            MatrixRowCells {
                                paths: row
                                    .paths
                                    .into_iter()
                                    .map(|path| path.map(|path| patch_path(patch, path)))
                                    .collect(),
                                budget_exceeded: vec![row.budget_exceeded; target_count],
                                visited_edge_count: row.counters.visited_edge_count,
                                settled_edge_count: row.counters.settled_edge_count,
                                open_bounds: Vec::new(),
                            }
                        }
                        None => {
                            let candidates = candidates
                                .iter()
                                .filter(|candidate| {
                                    connected[candidate.target]
                                        && (!retried_targets_only
                                            || retries(source, candidate.target))
                                        && may_reach(source, candidate.target)
                                })
                                .copied()
                                .collect::<Vec<_>>();
                            let row = matrix_row(
                                view,
                                &mut state,
                                &query,
                                origins,
                                &candidates,
                                target_count,
                                stop_after,
                            );
                            MatrixRowCells {
                                paths: row
                                    .paths
                                    .into_iter()
                                    .map(|path| path.map(compact_path))
                                    .collect(),
                                budget_exceeded: vec![row.budget_exceeded; target_count],
                                visited_edge_count: row.counters.visited_edge_count,
                                settled_edge_count: row.counters.settled_edge_count,
                                open_bounds: row.open_bounds,
                            }
                        }
                    };
                    graph.return_state(state);
                    Some(row)
                })
                .collect::<Vec<_>>()
        };
        // Cancelled searches stop as if out of budget: every searched cell
        // fails instead.
        request.options.cancellation.check()?;
        let (mut expansions, mut visited, mut settled) = (0, 0, 0);
        for (source_index, row) in rows.into_iter().enumerate() {
            let Some(row) = row else {
                continue;
            };
            expansions += 1;
            visited += row.visited_edge_count;
            settled += row.settled_edge_count;
            let mut left_open = vec![false; target_count];
            for (target, _) in &row.open_bounds {
                left_open[*target] = true;
            }
            if let Some(stop) = stop.as_mut() {
                stop.bounds.extend(&row.open_bounds);
            }
            for (target_index, path) in row.paths.into_iter().enumerate() {
                let budget_exceeded = row.budget_exceeded[target_index];
                // Cells a pass does not retry keep what `cells` held.
                if target_candidates[target_index].is_empty()
                    || left_open[target_index]
                    || !retries(source_index, target_index)
                {
                    continue;
                }
                let cell = cell_mut(cells, target_count, source_index, target_index);
                if let Some(path) = path {
                    apply_matrix_path(cell, &path);
                } else if components_are_disconnected(
                    access,
                    source_candidates[source_index],
                    target_candidates[target_index],
                ) {
                    apply_matrix_error(cell, &RouteError::Disconnected);
                } else if budget_exceeded {
                    apply_matrix_error(
                        cell,
                        &RouteError::SearchBudgetExceeded {
                            visited_edge_count: row.visited_edge_count,
                            max_visited_edge_count: request
                                .options
                                .max_visited_edge_count
                                .unwrap_or(0),
                        },
                    );
                } else {
                    apply_matrix_error(
                        cell,
                        &RouteError::NoPath {
                            visited_edge_count: row.visited_edge_count,
                        },
                    );
                }
            }
        }
        Ok((expansions, visited, settled))
    }
}

impl RoutingEngine {
    /// Searches again, on the strict column, the found cells whose path
    /// crossed a destination-only pocket that neither of their own endpoints
    /// lies in, with only their own endpoints' pockets unblocked, as a route
    /// between the two points has them (see `package::destination`). Every
    /// other found cell stands: its path is feasible with its own pockets
    /// alone, and the cheapest even with every pocket unblocked. Cells that
    /// then find no path are left for the relaxed pass. Returns the
    /// searches' (expansions, visited, settled).
    fn confine_to_own_pockets(
        &self,
        request: &RouteMatrixRequest,
        source_candidates: &[&[CorrelatedLocation]],
        target_candidates: &[&[CorrelatedLocation]],
        cells: &mut [RouteMatrixCell],
    ) -> Result<(usize, usize, usize), RouteError> {
        let graph = self.package();
        let view = graph.view();
        let pockets = graph.destination_pockets(request.profile);
        let own = |candidates: &&[CorrelatedLocation]| {
            let mut own = candidates
                .iter()
                .filter_map(|candidate| resolve_location(view, candidate))
                .filter_map(|edge| pockets.pocket_of(edge))
                .collect::<Vec<_>>();
            own.sort_unstable();
            own.dedup();
            own
        };
        let source_pockets = source_candidates.iter().map(own).collect::<Vec<_>>();
        let target_pockets = target_candidates.iter().map(own).collect::<Vec<_>>();
        // The pockets the strict pass unblocked, by edge id.
        let mut unblocked = FxHashMap::default();
        for pocket in source_pockets.iter().chain(&target_pockets).flatten() {
            for edge in pockets.pocket_edges(*pocket) {
                unblocked.insert(view.edge_id(*edge as usize), *pocket);
            }
        }
        if unblocked.is_empty() {
            return Ok((0, 0, 0));
        }
        // Cells to search again, by the pockets their endpoints lie in.
        let target_count = target_candidates.len();
        let mut groups = std::collections::BTreeMap::<Vec<u32>, Vec<bool>>::new();
        for (index, cell) in cells.iter().enumerate() {
            if cell.status != RouteMatrixCellStatus::Reachable {
                continue;
            }
            let (source, target) = (
                &source_pockets[cell.source_index],
                &target_pockets[cell.target_index],
            );
            let crosses = cell.edge_ids.iter().any(|edge| {
                unblocked
                    .get(edge)
                    .is_some_and(|pocket| !source.contains(pocket) && !target.contains(pocket))
            });
            if crosses {
                let mut key = source.iter().chain(target).copied().collect::<Vec<_>>();
                key.sort_unstable();
                key.dedup();
                groups
                    .entry(key)
                    .or_insert_with(|| vec![false; cells.len()])[index] = true;
            }
        }
        let mut counts = (0, 0, 0);
        for (key, retry) in groups {
            // One edge of each pocket names it.
            let endpoints = key
                .iter()
                .filter_map(|pocket| pockets.pocket_edges(*pocket).first().copied())
                .collect::<Vec<_>>();
            let mut confined = matrix_cell_grid(source_candidates.len(), target_count);
            let (expansions, visited, settled) = self.matrix_cells(
                request,
                source_candidates,
                target_candidates,
                &mut confined,
                false,
                Some(&retry),
                Some(&endpoints),
                None,
            )?;
            counts.0 += expansions;
            counts.1 += visited;
            counts.2 += settled;
            for ((cell, searched), retried) in cells.iter_mut().zip(confined).zip(&retry) {
                if *retried {
                    *cell = searched;
                }
            }
        }
        Ok(counts)
    }
}

/// Pairwise cells per thread: about this many runs each (see `search_cells`
/// in `RoutingEngine::matrix_cells`), so that the threads stay balanced.
const MATRIX_RUNS_PER_THREAD: usize = 4;
/// The longest run of cells of one source on one search state.
const MATRIX_MAX_RUN: usize = 64;

/// Up to this many searchable targets, cells are searched pairwise; above it,
/// each source row shares one expansion.
///
/// A row has to settle everything cheaper than its costliest target. On foot
/// that is a modest disc, so rows win early. For vehicles one far or
/// penalised target (a car point in a pedestrian zone) floods most of the
/// region, so a row costs about the same whatever its targets, while a cell
/// costs more the farther apart its points are. Measured on Malopolskie
/// (`examples/analysis_parity.rs`, process CPU, 10 source rows): on foot in
/// Krakow, pairwise wins at 8 targets and rows from 16.
///
/// Driving, a strict pass's cells also slow down as the matrix grows: every
/// endpoint's destination-only pocket is unblocked for every cell, and each
/// unblocked edge is a discount gate weakening the bounds. Measured
/// 2026-11-17, before CCH potentials replaced landmark bounds (square
/// matrices of random road points, 16 threads, visited edges, rows with
/// unreachable targets skipped): across Malopolskie rows
/// and pairwise visit about as much at 50 points (305M and 292M) and rows a
/// quarter as much at 64 (384M and 1,486M; 23 s and 62 s) and 80 (33 s and
/// 110 s); on the Krakow centre package rows win from 10 points (0.24 s and
/// 4.3 s at 50, 0.47 s and 26 s at 100). Rows and pairwise cells have the
/// same paths there; their costs, distances and durations differ only in
/// the last bits (the two halves of a bidirectional search add up in
/// another order).
const MATRIX_PAIRWISE_MAX_TARGETS_ON_FOOT: usize = 12;
/// The on-foot cut-off when CCH potentials bound the cells: a cell then
/// searches a corridor, while a row settles everything up to its farthest
/// target. Measured 2026-11-19 on Malopolskie (4 threads, potentials
/// prepared, process CPU, 12 rows of Krakow points): pairwise against rows
/// 2.3 s and 2.9 s at 50 targets, 2.8 s and 3.3 s at 64, 3.4 s and 3.4 s at
/// 80, 4.3 s and 3.8 s at 100; the whole Krakow 50x50 walk 8.1 s and 12.0 s.
const MATRIX_PAIRWISE_MAX_TARGETS_ON_FOOT_POTENTIALS: usize = 64;
const MATRIX_PAIRWISE_MAX_TARGETS_VEHICLE: usize = 50;
/// The vehicle cut-off when CCH potentials bound the cells
/// (`RoutingPackage::prepare_potentials`): none. A cell then searches a
/// corridor, while a row floods towards its farthest target, and with
/// hundreds of targets one of them is far (or ringed by destination-only
/// roads), so a row settles most of the region whatever its targets. Rows
/// also pay per target: the reaching sets that prune unreachable targets,
/// and a pocket pass (`confine_to_own_pockets`) for each group of cells
/// whose path crossed another point's destination-only pocket, which redoes
/// the setup for the whole matrix.
///
/// Measured 2026-11-18 on Malopolskie (16 threads, potentials prepared):
/// pairwise against rows 0.66 s and 30 s at 50x50, 2.2 s and 16 s at
/// 100x100, 15.6 s and about 90 s at 250x250, and with Krakow points 0.53 s
/// and 4.1 s at 100x100. Measured 2026-11-23 on Malopolskie (4 threads,
/// potentials prepared, `examples/analysis_parity.rs`, wall and process
/// CPU): Krakow 1x1,500 1.0 s (1.9 s CPU) pairwise against 14.5 s (42.7 s)
/// in rows, region 8x1,000 3.4 s (12.1 s) against 6.8 s (21.9 s), region
/// 2x4,000 3.7 s (12.9 s) against 16.0 s (40.8 s). On the small Krakow
/// centre package rows were a little cheaper at 100 and 250 targets (0.17 s
/// and 0.22 s at 100, 1.1 s and 1.4 s at 250), a fraction of a second either
/// way.
const MATRIX_PAIRWISE_MAX_TARGETS_VEHICLE_POTENTIALS: usize = usize::MAX;

/// The pairwise cut-off for `profile` (`potentials`: CCH potentials bound
/// the cells); [`crate::debug::matrix_pairwise_max_targets`] overrides it
/// for every profile.
fn matrix_pairwise_max_targets(profile: MobilityProfile, potentials: bool) -> usize {
    crate::debug::matrix_pairwise_max_targets().unwrap_or(match profile {
        MobilityProfile::OnFoot | MobilityProfile::Legged => {
            if potentials {
                MATRIX_PAIRWISE_MAX_TARGETS_ON_FOOT_POTENTIALS
            } else {
                MATRIX_PAIRWISE_MAX_TARGETS_ON_FOOT
            }
        }
        MobilityProfile::Default
        | MobilityProfile::Wheeled
        | MobilityProfile::Tracked
        | MobilityProfile::Emergency => {
            if potentials {
                MATRIX_PAIRWISE_MAX_TARGETS_VEHICLE_POTENTIALS
            } else {
                MATRIX_PAIRWISE_MAX_TARGETS_VEHICLE
            }
        }
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MatrixMode {
    /// One bidirectional search per cell.
    Pairwise,
    /// One shared expansion per source row.
    Rows,
}

/// The search mode for a matrix of `targets` searchable targets
/// (`potentials`: CCH potentials bound the cells);
/// [`crate::debug::matrix_mode`] forces one.
fn matrix_mode(profile: MobilityProfile, targets: usize, potentials: bool) -> MatrixMode {
    let forced = crate::debug::matrix_mode().and_then(|name| match name.as_str() {
        "pairwise" => Some(MatrixMode::Pairwise),
        "rows" => Some(MatrixMode::Rows),
        _ => None,
    });
    #[cfg(test)]
    let forced = test_mode::get().or(forced);
    forced.unwrap_or(
        if targets <= matrix_pairwise_max_targets(profile, potentials) {
            MatrixMode::Pairwise
        } else {
            MatrixMode::Rows
        },
    )
}

/// Tests: force a matrix mode on the current thread.
#[cfg(test)]
pub(crate) mod test_mode {
    use super::MatrixMode;
    use std::cell::Cell;

    thread_local! {
        static MODE: Cell<Option<MatrixMode>> = const { Cell::new(None) };
        static FULL_RELAXED_PASS: Cell<bool> = const { Cell::new(false) };
        static KEEP_UNREACHABLE_TARGETS: Cell<bool> = const { Cell::new(false) };
    }

    /// Makes the relaxed pass search every cell again, as it used to.
    pub(crate) fn set_full_relaxed_pass(full: bool) {
        FULL_RELAXED_PASS.with(|cell| cell.set(full));
    }

    pub(crate) fn full_relaxed_pass() -> bool {
        FULL_RELAXED_PASS.with(Cell::get)
    }

    /// Keeps targets no source can reach in every row, as rows used to.
    pub(crate) fn set_keep_unreachable_targets(keep: bool) {
        KEEP_UNREACHABLE_TARGETS.with(|cell| cell.set(keep));
    }

    pub(crate) fn keep_unreachable_targets() -> bool {
        KEEP_UNREACHABLE_TARGETS.with(Cell::get)
    }

    pub(crate) fn set(mode: Option<MatrixMode>) {
        MODE.with(|cell| cell.set(mode));
    }

    pub(crate) fn get() -> Option<MatrixMode> {
        MODE.with(Cell::get)
    }
}

/// One source row of compact results, however it was searched.
struct MatrixRowCells {
    paths: Vec<Option<MatrixPath>>,
    budget_exceeded: Vec<bool>,
    visited_edge_count: usize,
    settled_edge_count: usize,
    /// The targets a stopped row left open, with lower bounds on their costs.
    open_bounds: Vec<(usize, f64)>,
}

/// Rows that stop early ([`RoutingEngine::matrix_cells`]' `stop`): after
/// `after` targets each, with the open targets' lower bounds (target index,
/// least cost) collected in `bounds`. They search without approach costs:
/// a cell is then the cheapest path between any of its two points'
/// candidates, and a bound holds for every pair, so both bound the matrix's
/// cell between the same points on any pass whose edge costs are no lower.
#[derive(Debug, Default)]
pub(crate) struct RowStop {
    pub(crate) after: usize,
    pub(crate) bounds: Vec<(usize, f64)>,
}

#[derive(Debug, Clone)]
pub(crate) struct MatrixPath {
    pub(crate) edge_ids: Vec<u64>,
    pub(crate) distance_m: f64,
    pub(crate) duration_s: f64,
    pub(crate) cost: f64,
}

pub(crate) fn matrix_cell_grid(source_count: usize, target_count: usize) -> Vec<RouteMatrixCell> {
    let mut cells = Vec::with_capacity(source_count.saturating_mul(target_count));
    for source_index in 0..source_count {
        for target_index in 0..target_count {
            cells.push(RouteMatrixCell {
                source_index,
                target_index,
                status: RouteMatrixCellStatus::Failed,
                distance_m: None,
                duration_s: None,
                cost: None,
                edge_ids: Vec::new(),
                failure_reason: None,
            });
        }
    }
    cells
}

pub(crate) fn cell_mut(
    cells: &mut [RouteMatrixCell],
    target_count: usize,
    source_index: usize,
    target_index: usize,
) -> &mut RouteMatrixCell {
    &mut cells[source_index * target_count + target_index]
}

pub(crate) fn apply_matrix_path(cell: &mut RouteMatrixCell, path: &MatrixPath) {
    cell.status = RouteMatrixCellStatus::Reachable;
    cell.distance_m = Some(path.distance_m);
    cell.duration_s = Some(path.duration_s);
    cell.cost = Some(path.cost);
    cell.edge_ids = path.edge_ids.clone();
    cell.failure_reason = None;
}

/// Whether a strict pass's cell gets the relaxed one's: unreachable, and not
/// across weak components (which the relaxed pass cannot join either).
pub(crate) fn retries_relaxed(cell: &RouteMatrixCell) -> bool {
    cell.status == RouteMatrixCellStatus::Unreachable
        && cell.failure_reason.as_deref() != Some("disconnected")
}

/// Replaces the strict pass's `cells` that [`retries_relaxed`] by the
/// relaxed pass's.
pub(crate) fn take_relaxed_cells(cells: &mut [RouteMatrixCell], relaxed: Vec<RouteMatrixCell>) {
    for (cell, retried) in cells.iter_mut().zip(relaxed) {
        if retries_relaxed(cell) {
            *cell = retried;
        }
    }
}

pub(crate) fn apply_matrix_error(cell: &mut RouteMatrixCell, error: &RouteError) {
    cell.status = matrix_status_for_error(error);
    cell.distance_m = None;
    cell.duration_s = None;
    cell.cost = None;
    cell.edge_ids.clear();
    cell.failure_reason = Some(route_error_code(error));
}

/// Reaching sets larger than this are not collected: the target is taken to
/// be reachable from everywhere.
const MATRIX_REACHING_EDGES_LIMIT: usize = 16_384;

/// Every edge from which a path over edges that are `usable` can reach one of
/// `target_edges` (which are included), or `None` when there are more than
/// [`MATRIX_REACHING_EDGES_LIMIT`]. Turn restrictions and turn costs are
/// ignored, so a search from an edge outside the set cannot reach the
/// targets. Also `None` once `cancellation` stops the request, checked as the
/// search loops check it: the rows' searches then stop on their first visit.
fn reaching_edges(
    graph: SearchGraph<'_>,
    target_edges: &[u32],
    usable: impl Fn(u32) -> bool,
    cancellation: &RouteCancellation,
) -> Option<FxHashSet<u32>> {
    let mut reaching = target_edges.iter().copied().collect::<FxHashSet<_>>();
    let mut pending = target_edges.to_vec();
    let mut visits = 0usize;
    while let Some(edge) = pending.pop() {
        visits += 1;
        if cancellation.interrupted_at(visits) {
            return None;
        }
        for (previous, _) in graph.in_edges(graph.edge_source(edge as usize) as usize) {
            if usable(previous) && reaching.insert(previous) {
                if reaching.len() > MATRIX_REACHING_EDGES_LIMIT {
                    return None;
                }
                pending.push(previous);
            }
        }
    }
    Some(reaching)
}

/// `[source][target]`: whether the cell's candidates may share a weak
/// component (the negation of `components_are_disconnected`).
fn cell_connectivity(
    access: &PackageGraphView,
    sources: &[&[CorrelatedLocation]],
    targets: &[&[CorrelatedLocation]],
) -> Vec<Vec<bool>> {
    let source_components = sources
        .iter()
        .map(|candidates| {
            candidates
                .iter()
                .filter_map(|candidate| access.candidate_component_id(candidate, true))
                .collect::<BTreeSet<_>>()
        })
        .collect::<Vec<_>>();
    let target_components = targets
        .iter()
        .map(|candidates| {
            candidates
                .iter()
                .filter_map(|candidate| access.candidate_component_id(candidate, false))
                .collect::<BTreeSet<_>>()
        })
        .collect::<Vec<_>>();
    source_components
        .iter()
        .map(|sources| {
            target_components
                .iter()
                .map(|targets| {
                    sources.is_empty() || targets.is_empty() || !sources.is_disjoint(targets)
                })
                .collect()
        })
        .collect()
}

pub(crate) fn components_are_disconnected(
    access: &PackageGraphView,
    sources: &[CorrelatedLocation],
    targets: &[CorrelatedLocation],
) -> bool {
    let source_components = sources
        .iter()
        .filter_map(|candidate| access.candidate_component_id(candidate, true))
        .collect::<BTreeSet<_>>();
    let target_components = targets
        .iter()
        .filter_map(|candidate| access.candidate_component_id(candidate, false))
        .collect::<BTreeSet<_>>();
    !source_components.is_empty()
        && !target_components.is_empty()
        && source_components.is_disjoint(&target_components)
}

fn matrix_status_for_error(error: &RouteError) -> RouteMatrixCellStatus {
    match error {
        RouteError::NoCandidate { .. } | RouteError::ImpossibleMobilityProfile { .. } => {
            RouteMatrixCellStatus::NoCandidate
        }
        RouteError::Disconnected
        | RouteError::NoPath { .. }
        | RouteError::BlockedCorridor { .. } => RouteMatrixCellStatus::Unreachable,
        RouteError::GraphNotReady
        | RouteError::SearchBudgetExceeded { .. }
        | RouteError::MissingEdgeMetadata { .. }
        | RouteError::Cancelled { .. }
        | RouteError::SarUnavailable { .. }
        | RouteError::InvalidRequest { .. }
        | RouteError::RequestTooLarge { .. }
        | RouteError::NoMatch => RouteMatrixCellStatus::Failed,
    }
}

fn route_error_code(error: &RouteError) -> String {
    match error {
        RouteError::GraphNotReady => "graph_not_ready".to_string(),
        RouteError::NoCandidate { endpoint } => format!("no_candidate:{endpoint}"),
        RouteError::ImpossibleMobilityProfile { .. } => "impossible_mobility_profile".to_string(),
        RouteError::Disconnected => "disconnected".to_string(),
        RouteError::BlockedCorridor { .. } => "blocked_corridor".to_string(),
        RouteError::NoPath { .. } => "no_path".to_string(),
        RouteError::SearchBudgetExceeded { .. } => "search_budget_exceeded".to_string(),
        RouteError::MissingEdgeMetadata { .. } => "missing_edge_metadata".to_string(),
        RouteError::Cancelled {
            deadline_exceeded: false,
        } => "cancelled".to_string(),
        RouteError::Cancelled {
            deadline_exceeded: true,
        } => "deadline_exceeded".to_string(),
        RouteError::SarUnavailable { relaxation } => format!("sar_unavailable:{relaxation}"),
        RouteError::InvalidRequest { reason } => format!("invalid_request:{reason}"),
        RouteError::RequestTooLarge { what } => format!("request_too_large:{what}"),
        RouteError::NoMatch => "no_match".to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vehicle_matrices_with_potentials_stay_pairwise() {
        test_mode::set(None);
        for profile in [
            MobilityProfile::Default,
            MobilityProfile::Wheeled,
            MobilityProfile::Tracked,
            MobilityProfile::Emergency,
        ] {
            // However many targets: rows lost even at 4,000 (region).
            assert_eq!(
                matrix_mode(profile, MAX_MATRIX_CELLS, true),
                MatrixMode::Pairwise
            );
            assert_eq!(matrix_mode(profile, 50, false), MatrixMode::Pairwise);
            assert_eq!(matrix_mode(profile, 51, false), MatrixMode::Rows);
        }
        for profile in [MobilityProfile::OnFoot, MobilityProfile::Legged] {
            assert_eq!(matrix_mode(profile, 64, true), MatrixMode::Pairwise);
            assert_eq!(matrix_mode(profile, 65, true), MatrixMode::Rows);
            assert_eq!(matrix_mode(profile, 13, false), MatrixMode::Rows);
        }
    }
}

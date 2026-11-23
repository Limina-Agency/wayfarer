//! Nearest targets: the `k` targets cheapest to reach from a source (or to
//! reach it from), by route cost.
//!
//! The answer is defined by the matrix: the `k` cheapest reachable cells of
//! the matrix between the source and every target, ties by target index.
//! Towards the source it is that matrix's column, searched in full. From
//! the source, with more targets than `k`, a row expansion stops early
//! instead of flooding to the farthest target:
//!
//! 1. A bounding row (destination-only roads open, no approach costs) stops
//!    once `after` targets (first `2k`) have their path. Without approach
//!    costs and with the cheapest edge costs, its paths and the lower bounds
//!    it reports for the targets it left open bound the matrix's cell to
//!    the same target on every pass (strict, confined to the cell's own
//!    pockets, or relaxed), whichever candidates that cell then uses.
//! 2. The settled targets get their matrix cells: a matrix of the source
//!    and those targets alone, since a cell does not depend on the other
//!    targets (each is confined to its own two points' destination-only
//!    pockets).
//! 3. With `k` cells found, the open targets whose bound does not exceed the
//!    k-th cost get their cells too, and the `k` cheapest of all are the
//!    answer: every other target's cell costs more. With fewer, `after`
//!    doubles and the row runs again.
//!
//! With a dynamic overlay's split or virtual edges, or a visited-edge
//! budget (which applies to whole rows), the matrix row is searched in full.
//! Overlays are applied once, into the options of every search.

use crate::correlator::Correlator;
use crate::limits::MAX_NEAREST_TARGETS;
use crate::matrix::{RowStop, matrix_cell_grid};
use crate::search::RoutingEngine;
use crate::trip::{apply_query_overlays, invalid_point};
use crate::types::{
    CorrelatedLocation, CorrelationPolicy, NearestDirection, NearestHit, NearestRequest,
    NearestResult, RouteError, RouteMatrixCell, RouteMatrixCellStatus, RouteMatrixRequest,
};

impl NearestRequest {
    /// Checks the request against [`crate::limits`]: `k` between 1 and the
    /// target count, at most [`MAX_NEAREST_TARGETS`] targets, valid
    /// coordinates, and the overlays and options a matrix accepts.
    /// [`RoutingEngine::nearest`] runs it first.
    pub fn validate(&self) -> Result<(), RouteError> {
        if self.targets.len() > MAX_NEAREST_TARGETS {
            return Err(RouteError::RequestTooLarge {
                what: "nearest targets",
            });
        }
        if self.k == 0 || self.k > self.targets.len() {
            return Err(RouteError::InvalidRequest {
                reason: "k must be between 1 and the target count",
            });
        }
        if invalid_point(self.source) || self.targets.iter().copied().any(invalid_point) {
            return Err(RouteError::InvalidRequest {
                reason: "location coordinates",
            });
        }
        self.matrix(vec![self.source], Vec::new()).validate()
    }

    fn matrix(
        &self,
        sources: Vec<crate::GeoPoint>,
        targets: Vec<crate::GeoPoint>,
    ) -> RouteMatrixRequest {
        RouteMatrixRequest {
            sources,
            targets,
            profile: self.profile,
            options: self.options.clone(),
            overlays: self.overlays.clone(),
            departure_at_ms: self.departure_at_ms,
            include_edge_ids: self.include_edge_ids,
        }
    }
}

/// What the searches of one request add up to.
#[derive(Default)]
struct Searched {
    rows: usize,
    visited_edges: usize,
}

impl RoutingEngine {
    /// The `request.k` targets cheapest to travel to from `request.source`
    /// (or, `to_source`, from which to travel to it), cheapest first: the
    /// `k` cheapest reachable cells of the matrix between the source and
    /// every target, with their costs, durations, distances and paths. A
    /// request that fails [`NearestRequest::validate`] fails with its
    /// reason; so does one whose source has no road within reach
    /// (`no_candidate`) or that is cancelled. Unreachable targets are left
    /// out, so there may be fewer than `k` hits.
    pub fn nearest(&self, mut request: NearestRequest) -> Result<NearestResult, RouteError> {
        request.validate()?;
        let access = self.route_graph_view();
        if !access.is_ready() {
            return Err(RouteError::GraphNotReady);
        }
        if !access.has_route_edges_for_profile(request.profile) {
            return Err(RouteError::ImpossibleMobilityProfile {
                profile: request.profile,
            });
        }
        self.validate_sar(&request.options, request.profile)?;
        // Overlays are applied once, into the options every search shares.
        apply_query_overlays(
            self,
            &request.overlays,
            request.profile,
            request.departure_at_ms,
            &mut request.options,
        )?;
        request.overlays.clear();
        request.options.cancellation.check()?;

        let policy = CorrelationPolicy::from(&request.options);
        let correlator = Correlator::new(access);
        let correlate = |point| {
            correlator
                .search_expanding(point, request.profile, &policy, &Default::default())
                .candidates
        };
        let source = correlate(request.source);
        if source.is_empty() {
            return Err(RouteError::NoCandidate {
                endpoint: match request.direction {
                    NearestDirection::FromSource => "origin",
                    NearestDirection::ToSource => "destination",
                },
            });
        }
        let targets = request
            .targets
            .iter()
            .map(|target| correlate(*target))
            .collect::<Vec<_>>();
        let no_candidate_target_indexes = (0..targets.len())
            .filter(|target| targets[*target].is_empty())
            .collect::<Vec<_>>();

        let mut searched = Searched::default();
        let hits = match request.direction {
            NearestDirection::ToSource => {
                let column =
                    self.matrix(request.matrix(request.targets.clone(), vec![request.source]));
                searched.add(&column);
                request.options.cancellation.check()?;
                cheapest(
                    column.cells.iter().map(|cell| (cell.source_index, cell)),
                    request.k,
                )
            }
            NearestDirection::FromSource if self.searches_rows_in_full(&request) => {
                let row =
                    self.matrix(request.matrix(vec![request.source], request.targets.clone()));
                searched.add(&row);
                request.options.cancellation.check()?;
                cheapest(
                    row.cells.iter().map(|cell| (cell.target_index, cell)),
                    request.k,
                )
            }
            NearestDirection::FromSource => {
                self.nearest_from_source(&request, &source, &targets, &mut searched)?
            }
        };
        Ok(NearestResult {
            target_count: request.targets.len(),
            hits,
            no_candidate_target_indexes,
            row_count: searched.rows,
            visited_edge_count: searched.visited_edges,
        })
    }

    /// Whether the row from the source is searched in full (not stopped
    /// early): no more targets than `k`, a visited-edge budget, or a dynamic
    /// overlay patch.
    fn searches_rows_in_full(&self, request: &NearestRequest) -> bool {
        request.targets.len() <= request.k
            || request.options.max_visited_edge_count.is_some()
            || request
                .options
                .dynamic_overlay
                .as_ref()
                .is_some_and(|overlay| {
                    !overlay.split_edge_ranges.is_empty() || !overlay.virtual_edges.is_empty()
                })
    }

    /// The early-stopping search of the module docs.
    fn nearest_from_source(
        &self,
        request: &NearestRequest,
        source: &[CorrelatedLocation],
        targets: &[Vec<CorrelatedLocation>],
        searched: &mut Searched,
    ) -> Result<Vec<NearestHit>, RouteError> {
        let matrix_request = request.matrix(vec![request.source], request.targets.clone());
        let source_candidates = [source];
        let target_candidates = targets.iter().map(Vec::as_slice).collect::<Vec<_>>();
        let searchable = targets.iter().filter(|target| !target.is_empty()).count();
        // The matrix's cells, by target, as they are searched.
        let mut cells = vec![None::<RouteMatrixCell>; request.targets.len()];
        let evaluate = |cells: &mut Vec<Option<RouteMatrixCell>>,
                        wanted: &[usize],
                        searched: &mut Searched| {
            let wanted = wanted
                .iter()
                .copied()
                .filter(|target| cells[*target].is_none())
                .collect::<Vec<_>>();
            if wanted.is_empty() {
                return Ok(());
            }
            let sub = self.matrix(
                request.matrix(
                    vec![request.source],
                    wanted
                        .iter()
                        .map(|target| request.targets[*target])
                        .collect(),
                ),
            );
            searched.add(&sub);
            request.options.cancellation.check()?;
            for cell in sub.cells {
                let target = wanted[cell.target_index];
                cells[target] = Some(cell);
            }
            Ok::<_, RouteError>(())
        };
        let mut after = request.k.saturating_mul(2).min(searchable).max(1);
        loop {
            let mut stop = RowStop {
                after,
                bounds: Vec::new(),
            };
            let mut bounding = matrix_cell_grid(1, request.targets.len());
            let (expansions, visited, _) = self.package().in_matrix_pool(|| {
                self.matrix_cells(
                    &matrix_request,
                    &source_candidates,
                    &target_candidates,
                    &mut bounding,
                    true,
                    None,
                    None,
                    Some(&mut stop),
                )
            })?;
            searched.rows += expansions;
            searched.visited_edges += visited;
            request.options.cancellation.check()?;

            let mut open = vec![false; request.targets.len()];
            for (target, _) in &stop.bounds {
                open[*target] = true;
            }
            let settled = (0..request.targets.len())
                .filter(|target| !open[*target] && !targets[*target].is_empty())
                .collect::<Vec<_>>();
            evaluate(&mut cells, &settled, searched)?;
            let hits = |cells: &[Option<RouteMatrixCell>]| {
                cheapest(
                    cells
                        .iter()
                        .enumerate()
                        .filter_map(|(target, cell)| Some((target, cell.as_ref()?))),
                    request.k,
                )
            };
            let found = hits(&cells);
            if stop.bounds.is_empty() {
                return Ok(found);
            }
            if found.len() == request.k {
                // Every open target whose bound does not exceed the k-th cost
                // may still beat it (or tie it at a lower index): search
                // those cells too. The k-th cost can only fall, so the others
                // stay out.
                let kth = found[request.k - 1].cost;
                let contenders = stop
                    .bounds
                    .iter()
                    .filter(|(_, bound)| *bound <= kth)
                    .map(|(target, _)| *target)
                    .collect::<Vec<_>>();
                evaluate(&mut cells, &contenders, searched)?;
                return Ok(hits(&cells));
            }
            // Fewer than k reachable among the settled: settle more. A row
            // asked for every target stops only once it has them all.
            if after >= searchable {
                return Ok(found);
            }
            after = after.saturating_mul(2).min(searchable);
        }
    }
}

impl Searched {
    fn add(&mut self, result: &crate::RouteMatrixResult) {
        self.rows += result.source_expansion_count;
        self.visited_edges += result.visited_edge_count;
    }
}

/// The `k` cheapest reachable cells, as hits for their targets, ties by
/// target index.
fn cheapest<'c>(
    cells: impl Iterator<Item = (usize, &'c RouteMatrixCell)>,
    k: usize,
) -> Vec<NearestHit> {
    let mut hits = cells
        .filter(|(_, cell)| cell.status == RouteMatrixCellStatus::Reachable)
        .filter_map(|(target_index, cell)| {
            Some(NearestHit {
                target_index,
                cost: cell.cost?,
                distance_m: cell.distance_m?,
                duration_s: cell.duration_s?,
                edge_ids: cell.edge_ids.clone(),
            })
        })
        .collect::<Vec<_>>();
    hits.sort_by(|a, b| {
        a.cost
            .total_cmp(&b.cost)
            .then(a.target_index.cmp(&b.target_index))
    });
    hits.truncate(k);
    hits
}

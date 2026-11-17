//! Reference searches: the parity oracle for tests and harnesses.
//!
//! Test support, not a production path. [`ReferenceEngine`] answers route,
//! matrix and isochrone requests with plain Dijkstra over the in-memory road
//! graph's own records ([`GraphAccess`]): edge costs and durations from
//! `CostingPolicy` (so every costing option and per-edge
//! block/avoid/multiplier applies exactly as the package's columns and
//! overrides are meant to), turn costs from
//! `CostingPolicy::transition_cost_for_records`, and simple and complex
//! restrictions from the road graph's resolved restrictions. The searches never
//! read a routing package; correlation and route legs run on one built in
//! memory from the same road graph, as the engine's do.
//!
//! The search semantics are the routing package's (see `package/search.rs`,
//! `package/search/analysis.rs` and `package/search/patch.rs`), written out
//! independently and without goal direction:
//!
//! - one label per directed edge (per arc with an overlay), or per edge and
//!   path context with [`ReferenceEngine::with_path_context`], which makes
//!   the oracle exact under complex restrictions; seeds on the partially
//!   traversed origin edges, destinations finished from the source node of
//!   the destination edge (never onto an edge already on the path);
//! - a direct same-edge path when an origin lies behind a destination on one
//!   edge (routes take the first such pair, matrices the cheapest);
//! - dynamic overlays as a patch: split edges leave the graph and their usable
//!   ranges become fragments between stops (range ends and connector
//!   attachments), connectors are virtual arcs, a connector starts a new
//!   restriction context, and turn costs and restrictions apply where an arc
//!   enters an edge at its start node;
//! - isochrones as one bounded expansion that records reach per relaxation
//!   (a U-turn onto the opposing edge pays its turn cost, as on routes) and
//!   reports boundary crossings per contour. Isochrone overlays are limited
//!   to full blocks.
//!
//! Budgets (`max_visited_edge_count`) are ignored by routes and matrices: the
//! oracle always searches to completion. Isochrones honour
//! `max_expansion_edge_count` with the package expansion's counting.

use std::cell::RefCell;
use std::cmp::Ordering;
use std::collections::{BTreeSet, BinaryHeap};
use std::rc::Rc;

use rustc_hash::FxHashMap;

use crate::correlator::{CandidateSearch, Correlator};
use crate::costing::{CostingPolicy, EdgeCostDecision};
use crate::graph_access::GraphAccess;
use crate::graph_view::{GraphViewEdge, PackageGraphView};
use crate::isochrone::{IsochroneBoundaryCandidate, boundary_fragments_for_contour};
use crate::matrix::components_are_disconnected as matrix_components_are_disconnected;
use crate::matrix::{
    MatrixPath, apply_matrix_error, apply_matrix_path, cell_mut, matrix_cell_grid, retries_relaxed,
    take_relaxed_cells,
};
use crate::package::destination::strict_destination_only;
use crate::package::search::approach_cost_of;
use crate::route_leg::build_route_result;
use crate::search::{
    RouteSearchStep, SearchRoute, components_are_disconnected, relaxed_correlation_retry,
};
use crate::spatial::directed_geometry;
use crate::types::{
    CorrelatedLocation, CorrelationPolicy, GeoPoint, LegRequest, LegResult, MobilityProfile,
    RouteDiagnostics, RouteDynamicEndpointKind, RouteDynamicOverlay, RouteError,
    RouteIsochroneBucket, RouteIsochroneMetric, RouteIsochroneRequest, RouteIsochroneResult,
    RouteMatrixCell, RouteMatrixRequest, RouteMatrixResult, RouteOptions, RouteShapeQualityFlag,
};

const NONE: u32 = u32::MAX;
const FRACTION_EPSILON: f64 = f64::EPSILON;
/// A seed shorter than this traverses nothing.
const TRAVERSED_EPSILON: f64 = 1e-9;

/// Approach costs (`package::search::approach_costs`), written out: each
/// `(target, candidate)`'s snap distance beyond the nearest candidate of the
/// same target, priced by `approach_cost_of`.
fn approach_costs(candidates: &[(usize, &CorrelatedLocation)]) -> Vec<f64> {
    let distance = |candidate: &CorrelatedLocation| {
        if candidate.distance_m.is_finite() {
            candidate.distance_m.max(0.0)
        } else {
            0.0
        }
    };
    let mut nearest = FxHashMap::<usize, f64>::default();
    for (target, candidate) in candidates {
        let entry = nearest.entry(*target).or_insert(f64::INFINITY);
        *entry = entry.min(distance(candidate));
    }
    candidates
        .iter()
        .map(|(target, candidate)| approach_cost_of(distance(candidate) - nearest[target]))
        .collect()
}

/// The approach costs of one endpoint's candidates.
fn endpoint_approach(candidates: &[CorrelatedLocation]) -> Vec<f64> {
    approach_costs(
        &candidates
            .iter()
            .map(|candidate| (0, candidate))
            .collect::<Vec<_>>(),
    )
}
/// Records kept per query before the cache starts over (bounds memory on
/// regional searches).
const RECORD_CACHE_LIMIT: usize = 400_000;

/// The reference engine over one road graph. See the module docs.
pub struct ReferenceEngine<'a> {
    access: GraphAccess<'a>,
    /// The same road graph as a routing package, for correlation and route
    /// legs.
    view: PackageGraphView,
    path_context: usize,
    /// Destination-only edges grouped into pockets per profile slot, on
    /// first use.
    destination_pockets:
        [std::cell::OnceCell<DestinationPockets>; wayfarer_package::format::PROFILE_SLOTS],
}

/// The road graph's destination-only edges a profile may use, by connected
/// pocket (`package::destination`, written out over the graph's own
/// records).
struct DestinationPockets {
    pocket_of: FxHashMap<u64, usize>,
    members: Vec<Vec<u64>>,
}

impl<'a> ReferenceEngine<'a> {
    /// The reference engine over `graph`. `package` must be the routing
    /// package built from this same graph (with the default build options):
    /// correlation and route legs read it.
    pub fn new(
        graph: &'a wayfarer_road_graph::RoadGraph,
        package: std::sync::Arc<crate::RoutingPackage>,
    ) -> Self {
        Self::from_access(GraphAccess::new(graph), package)
    }

    /// As [`ReferenceEngine::new`], over a prepared graph access.
    pub fn from_access(
        access: GraphAccess<'a>,
        package: std::sync::Arc<crate::RoutingPackage>,
    ) -> Self {
        Self {
            access,
            view: PackageGraphView::new(package),
            path_context: 0,
            destination_pockets: Default::default(),
        }
    }

    fn destination_pockets(&self, profile: MobilityProfile) -> &DestinationPockets {
        self.destination_pockets[wayfarer_package::format::profile_slot(profile)].get_or_init(
            || {
                let mask = profile.mobility_mode_mask();
                let records = (0..self.access.raw_edge_count())
                    .filter_map(|raw| self.access.edge_record_by_raw_edge_index(raw))
                    .filter(|record| {
                        record.shared_info.is_destination_only
                            && record.edge.mobility_mode_mask & mask != 0
                    })
                    .map(|record| {
                        (
                            record.edge.edge_id,
                            record.source_index.index(),
                            record.target_index.index(),
                        )
                    })
                    .collect::<Vec<_>>();
                // Merge pockets until no edge joins two: slow, plain and exact.
                let mut pocket_of_node = FxHashMap::<usize, usize>::default();
                let mut members = Vec::<Vec<(u64, usize, usize)>>::new();
                for record in records {
                    let (_, source, target) = record;
                    let mut pocket = None;
                    for node in [source, target] {
                        if let Some(found) = pocket_of_node.get(&node).copied() {
                            match pocket {
                                None => pocket = Some(found),
                                Some(kept) if kept != found => {
                                    let moved = std::mem::take(&mut members[found]);
                                    for (_, source, target) in &moved {
                                        pocket_of_node.insert(*source, kept);
                                        pocket_of_node.insert(*target, kept);
                                    }
                                    members[kept].extend(moved);
                                }
                                Some(_) => {}
                            }
                        }
                    }
                    let pocket = pocket.unwrap_or_else(|| {
                        members.push(Vec::new());
                        members.len() - 1
                    });
                    members[pocket].push(record);
                    pocket_of_node.insert(source, pocket);
                    pocket_of_node.insert(target, pocket);
                }
                let members = members
                    .into_iter()
                    .filter(|pocket| !pocket.is_empty())
                    .map(|pocket| pocket.into_iter().map(|(edge, ..)| edge).collect())
                    .collect::<Vec<Vec<u64>>>();
                let pocket_of = members
                    .iter()
                    .enumerate()
                    .flat_map(|(index, pocket)| pocket.iter().map(move |edge| (*edge, index)))
                    .collect();
                DestinationPockets { pocket_of, members }
            },
        )
    }

    /// Records for one pass: strict (destination-only edges blocked but for
    /// the pockets around `endpoints`) or relaxed.
    fn records<'r>(
        &'r self,
        profile: MobilityProfile,
        options: &'r RouteOptions,
        allow_destination_only: bool,
        endpoints: impl IntoIterator<Item = u64>,
    ) -> Records<'r> {
        let mut records = Records::new(&self.access, profile, options, allow_destination_only);
        if !allow_destination_only {
            let pockets = self.destination_pockets(profile);
            let unblocked = endpoints
                .into_iter()
                .filter_map(|edge| pockets.pocket_of.get(&edge))
                .flat_map(|pocket| pockets.members[*pocket].iter().copied())
                .collect();
            records.unblocked = Some((CostingPolicy::new(profile, options), unblocked));
        }
        records
    }

    /// Routes and matrices (plain and overlay) keep one label per (edge or
    /// arc, last `edges` traversed edges) instead of one per edge. With
    /// `edges` at least the longest required prefix of a complex
    /// restriction, they are exact under complex restrictions too: one label
    /// per edge can lose a legal continuation when the cheapest way onto an
    /// edge is the start of a forbidden sequence and a dearer way onto it is
    /// not.
    ///
    /// Isochrones report a boundary crossing per relaxation, so a finer key
    /// would add crossings rather than change reach: with any `edges > 0`
    /// they key labels by the longest suffix of the path that starts some
    /// required prefix (found by comparing suffixes with every prefix), the
    /// coarsest key that is still exact.
    pub fn with_path_context(mut self, edges: usize) -> Self {
        self.path_context = edges;
        self
    }

    /// As [`ReferenceEngine::new`], owning the graph (a loaded snapshot, for
    /// whole-region comparisons).
    pub fn from_owned_graph(
        graph: std::sync::Arc<wayfarer_road_graph::RoadGraph>,
        package: std::sync::Arc<crate::RoutingPackage>,
    ) -> ReferenceEngine<'static> {
        let index = std::sync::Arc::new(crate::graph_access::RouteGraphIndex::build(&graph));
        ReferenceEngine::from_access(GraphAccess::from_owned(graph, index), package)
    }

    fn ready(&self, profile: MobilityProfile) -> Result<(), RouteError> {
        if !self.access.is_ready() {
            return Err(RouteError::GraphNotReady);
        }
        if !self.access.has_route_edges_for_profile(profile) {
            return Err(RouteError::ImpossibleMobilityProfile { profile });
        }
        Ok(())
    }

    fn correlate(
        &self,
        point: GeoPoint,
        profile: MobilityProfile,
        policy: &CorrelationPolicy,
        options: &crate::types::RouteEndpointOptions,
    ) -> CandidateSearch {
        let correlator = Correlator::new(&self.view);
        let search = correlator.search_expanding(point, profile, policy, options);
        if search.candidates.is_empty()
            && options.has_scoring_hints()
            && let Some(retry) = relaxed_correlation_retry(
                &correlator,
                point,
                profile,
                policy,
                options,
                search.clone(),
            )
        {
            return retry;
        }
        search
    }

    /// The shortest route, with legs built from the road graph's records.
    /// Alternatives are not searched.
    pub fn route_leg(&self, request: LegRequest) -> Result<LegResult, RouteError> {
        self.ready(request.profile)?;
        let policy = CorrelationPolicy::from(&request.options);
        let origins = self
            .correlate(
                request.origin,
                request.profile,
                &policy,
                &request.origin_options,
            )
            .candidates;
        if origins.is_empty() {
            return Err(RouteError::NoCandidate { endpoint: "origin" });
        }
        let destinations = self
            .correlate(
                request.destination,
                request.profile,
                &policy,
                &request.destination_options,
            )
            .candidates;
        if destinations.is_empty() {
            return Err(RouteError::NoCandidate {
                endpoint: "destination",
            });
        }
        // Components are the legal network's (see the engine's route).
        if crate::sar::SarOpenEdges::of(&request.options).is_none()
            && components_are_disconnected(&self.view, &origins, &destinations)
        {
            return Err(RouteError::Disconnected);
        }
        let strict = strict_destination_only(&request.options, request.profile);
        let route = match self.route_search(&request, &origins, &destinations, !strict) {
            Err(error) if strict && error.is_no_path() => {
                self.route_search(&request, &origins, &destinations, true)?
            }
            result => result?,
        };
        let mut result = build_route_result(&self.view, route, request.profile, &request.options)?;
        crate::sar::add_off_network_uses(
            &mut result,
            request.origin,
            request.destination,
            request.profile,
            &request.options,
        );
        Ok(result)
    }

    fn route_search(
        &self,
        request: &LegRequest,
        origins: &[CorrelatedLocation],
        destinations: &[CorrelatedLocation],
        allow_destination_only: bool,
    ) -> Result<SearchRoute, RouteError> {
        let records = self.records(
            request.profile,
            &request.options,
            allow_destination_only,
            origins
                .iter()
                .chain(destinations)
                .map(|location| location.edge_id),
        );
        if let Some(model) = request
            .options
            .dynamic_overlay
            .as_ref()
            .map(|overlay| OverlayModel::new(&records, overlay))
            .filter(|model| !model.is_empty())
        {
            let targets = destinations.iter().map(|location| (0, location));
            let path = overlay_search(
                &records,
                &model,
                origins,
                targets.collect(),
                1,
                self.path_context,
            )
            .pop()
            .flatten()
            .ok_or(RouteError::NoPath {
                visited_edge_count: 0,
            })?;
            return Ok(overlay_route(
                &self.view,
                request,
                origins,
                destinations,
                path,
            ));
        }
        // The cheapest origin behind a destination on the same edge competes
        // with the searched path.
        let direct = cheapest_direct(&records, origins, destinations);
        let targets = destinations.iter().map(|location| (0, location)).collect();
        let searched = plain_search(&records, origins, targets, 1, self.path_context)
            .pop()
            .flatten();
        let path = match (searched, direct) {
            (Some(searched), Some(direct))
                if direct.total <= searched.total + 1e-7 * searched.total.max(1.0) =>
            {
                direct
            }
            (Some(searched), _) => searched,
            (None, direct) => direct.ok_or(RouteError::NoPath {
                visited_edge_count: 0,
            })?,
        };
        Ok(plain_route(
            &self.view,
            &origins[path.origin],
            &destinations[path.destination],
            path,
        ))
    }

    /// Every cell, one expansion per source row. Dynamic overlays drop their
    /// endpoint connectors, which belong to a route's own endpoints.
    pub fn matrix(&self, request: RouteMatrixRequest) -> RouteMatrixResult {
        let source_count = request.sources.len();
        let target_count = request.targets.len();
        let mut cells = matrix_cell_grid(source_count, target_count);
        let result = |cells, source_expansion_count| RouteMatrixResult {
            source_count,
            target_count,
            cells,
            source_expansion_count,
            visited_edge_count: 0,
            settled_node_count: 0,
        };
        if let Err(error) = self.ready(request.profile) {
            for cell in &mut cells {
                apply_matrix_error(cell, &error);
            }
            return result(cells, 0);
        }
        let policy = CorrelationPolicy::from(&request.options);
        let correlate = |point| {
            Correlator::new(&self.view)
                .search_expanding(point, request.profile, &policy, &Default::default())
                .candidates
        };
        let sources = request
            .sources
            .iter()
            .copied()
            .map(correlate)
            .collect::<Vec<_>>();
        let targets = request
            .targets
            .iter()
            .copied()
            .map(correlate)
            .collect::<Vec<_>>();
        for (source, candidates) in sources.iter().enumerate() {
            if candidates.is_empty() {
                for target in 0..target_count {
                    apply_matrix_error(
                        cell_mut(&mut cells, target_count, source, target),
                        &RouteError::NoCandidate { endpoint: "origin" },
                    );
                }
            }
        }
        for (target, candidates) in targets.iter().enumerate() {
            if candidates.is_empty() {
                for source in 0..source_count {
                    let cell = cell_mut(&mut cells, target_count, source, target);
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
        let strict = strict_destination_only(&request.options, request.profile);
        let mut expansions = self.matrix_pass(&request, &sources, &targets, &mut cells, !strict);
        // Cells that cannot connect without crossing a destination-only road
        // get the relaxed search, as routes do.
        if strict && cells.iter().any(retries_relaxed) {
            let mut relaxed = cells.clone();
            expansions += self.matrix_pass(&request, &sources, &targets, &mut relaxed, true);
            take_relaxed_cells(&mut cells, relaxed);
        }
        result(cells, expansions)
    }

    /// One pass over every row, strict or relaxed (see `records`); returns
    /// the rows expanded.
    fn matrix_pass(
        &self,
        request: &RouteMatrixRequest,
        sources: &[Vec<CorrelatedLocation>],
        targets: &[Vec<CorrelatedLocation>],
        cells: &mut [RouteMatrixCell],
        allow_destination_only: bool,
    ) -> usize {
        let target_count = targets.len();
        let records = self.records(
            request.profile,
            &request.options,
            allow_destination_only,
            sources
                .iter()
                .chain(targets)
                .flatten()
                .map(|location| location.edge_id),
        );
        let overlay = request
            .options
            .dynamic_overlay
            .as_ref()
            .map(crate::package::search::patch::analysis_overlay);
        let model = overlay
            .as_ref()
            .map(|overlay| OverlayModel::new(&records, overlay))
            .filter(|model| !model.is_empty());
        // Connectors, and roads a SAR request opens, may join weak components.
        let has_connectors = model.as_ref().is_some_and(|model| model.has_connectors())
            || crate::sar::SarOpenEdges::of(&request.options).is_some();
        let mut expansions = 0;
        for (source, origins) in sources.iter().enumerate() {
            if origins.is_empty() {
                continue;
            }
            // Cells across weak components are not searched (without
            // connectors, which may join them).
            let open = (0..target_count)
                .filter(|target| {
                    !targets[*target].is_empty()
                        && (has_connectors
                            || !matrix_components_are_disconnected(
                                &self.view,
                                origins,
                                &targets[*target],
                            ))
                })
                .collect::<Vec<_>>();
            if open.is_empty() && targets.iter().all(Vec::is_empty) {
                continue;
            }
            expansions += 1;
            let candidates = open
                .iter()
                .flat_map(|target| {
                    targets[*target]
                        .iter()
                        .map(move |location| (*target, location))
                })
                .collect::<Vec<_>>();
            let paths: Vec<Option<MatrixPath>> = match &model {
                Some(model) => overlay_search(
                    &records,
                    model,
                    origins,
                    candidates,
                    target_count,
                    self.path_context,
                )
                .into_iter()
                .map(|path| path.map(OverlayPath::matrix_path))
                .collect(),
                None => {
                    let mut paths = plain_search(
                        &records,
                        origins,
                        candidates,
                        target_count,
                        self.path_context,
                    );
                    // A target ahead of an origin on the same edge may be
                    // reached directly, at the cheapest such pair.
                    for target in &open {
                        if let Some(direct) = cheapest_direct(&records, origins, &targets[*target])
                            && paths[*target].as_ref().is_none_or(|searched| {
                                direct.total <= searched.total + 1e-7 * searched.total.max(1.0)
                            })
                        {
                            paths[*target] = Some(direct);
                        }
                    }
                    paths
                        .into_iter()
                        .map(|path| {
                            path.map(|path| MatrixPath {
                                edge_ids: path.edge_ids,
                                distance_m: path.distance_m,
                                duration_s: path.duration_s,
                                cost: path.cost,
                            })
                        })
                        .collect()
                }
            };
            for (target, path) in paths.into_iter().enumerate() {
                if targets[target].is_empty() {
                    continue;
                }
                let cell = cell_mut(cells, target_count, source, target);
                if let Some(path) = path {
                    apply_matrix_path(cell, &path);
                } else if matrix_components_are_disconnected(&self.view, origins, &targets[target])
                {
                    apply_matrix_error(cell, &RouteError::Disconnected);
                } else {
                    apply_matrix_error(
                        cell,
                        &RouteError::NoPath {
                            visited_edge_count: 0,
                        },
                    );
                }
            }
        }
        expansions
    }

    /// One bounded expansion from the origin's candidates. Overlays may only
    /// fully block edges (the costing policy applies those).
    pub fn isochrone(
        &self,
        request: RouteIsochroneRequest,
    ) -> Result<RouteIsochroneResult, RouteError> {
        self.ready(request.profile)?;
        if let Some(overlay) = &request.options.dynamic_overlay {
            assert!(
                overlay.virtual_edges.is_empty()
                    && overlay
                        .split_edge_ranges
                        .iter()
                        .all(|split| split.usable_ranges.is_empty()),
                "the reference isochrone models overlays of full blocks only"
            );
        }
        let policy = CorrelationPolicy::from(&request.options);
        let candidates = Correlator::new(&self.view)
            .search(
                request.origin,
                request.profile,
                &policy,
                &Default::default(),
            )
            .candidates;
        let origin = candidates
            .first()
            .cloned()
            .ok_or(RouteError::NoCandidate { endpoint: "origin" })?;
        let context = self.path_context > 0;
        let pass = |allow_destination_only: bool| {
            let records = self.records(
                request.profile,
                &request.options,
                allow_destination_only,
                candidates.iter().map(|location| location.edge_id),
            );
            isochrone(&records, &request, &candidates, origin.clone(), context)
        };
        if !strict_destination_only(&request.options, request.profile) {
            return Ok(pass(true));
        }
        let strict = pass(false);
        if !crate::isochrone::trapped(&strict) {
            return Ok(strict);
        }
        Ok(crate::isochrone::pick_relaxed(strict, pass(true)))
    }
}

// ---------------------------------------------------------------------------
// Records
// ---------------------------------------------------------------------------

/// A directed edge's search record and its request cost and duration (`None`
/// when the costing policy blocks it or the profile cannot use it).
struct Edge {
    record: GraphViewEdge,
    costed: Option<(f64, f64)>,
}

impl Edge {
    fn id(&self) -> u64 {
        self.record.edge.edge_id
    }

    fn length(&self) -> f64 {
        f64::from(self.record.edge.distance_m)
    }

    fn source(&self) -> usize {
        self.record.source_index as usize
    }

    fn target(&self) -> usize {
        self.record.target_index as usize
    }

    fn cost(&self) -> f64 {
        self.costed.map_or(f64::INFINITY, |(cost, _)| cost)
    }

    fn duration(&self) -> f64 {
        self.costed.map_or(f64::INFINITY, |(_, duration)| duration)
    }
}

/// Road graph records, costed for one request, cached per query.
struct Records<'a> {
    access: &'a GraphAccess<'a>,
    profile: MobilityProfile,
    costing: CostingPolicy<'a>,
    /// On a strict pass: the relaxed policy, and the destination-only edges
    /// it costs because they lie in a pocket around the endpoints.
    unblocked: Option<(CostingPolicy<'a>, rustc_hash::FxHashSet<u64>)>,
    /// The SAR `turn_restrictions` penalty per violated restriction (`None`:
    /// restrictions ban).
    restriction_penalty: Option<f64>,
    /// The request's transition costs (barriers, SAR R2 entry).
    transitions: crate::transitions::Transitions,
    edges: RefCell<FxHashMap<usize, Rc<Edge>>>,
    /// Every non-empty start of a required prefix of the profile's complex
    /// restrictions, read on first use.
    prefix_starts: std::cell::OnceCell<rustc_hash::FxHashSet<Vec<u64>>>,
}

impl<'a> Records<'a> {
    fn new(
        access: &'a GraphAccess<'a>,
        profile: MobilityProfile,
        options: &'a RouteOptions,
        allow_destination_only: bool,
    ) -> Self {
        Self {
            access,
            profile,
            costing: CostingPolicy::with_destination_only_policy(
                profile,
                options,
                allow_destination_only,
            ),
            unblocked: None,
            restriction_penalty: crate::sar::restriction_penalty(options),
            transitions: crate::transitions::Transitions::of(options, profile),
            edges: RefCell::default(),
            prefix_starts: std::cell::OnceCell::new(),
        }
    }

    /// The restriction context of a path whose previous context was
    /// `previous` and which then traversed `edge_id`: the longest suffix that
    /// starts some complex restriction's required prefix (empty if none).
    fn restriction_context(&self, previous: &[u64], edge_id: u64) -> Vec<u64> {
        let starts = self.prefix_starts.get_or_init(|| {
            let mut starts = rustc_hash::FxHashSet::default();
            for prefix in self.access.complex_restriction_prefixes(self.profile) {
                for end in 1..=prefix.len() {
                    starts.insert(prefix[..end].to_vec());
                }
            }
            starts
        });
        let mut path = previous.to_vec();
        path.push(edge_id);
        (0..path.len())
            .map(|start| &path[start..])
            .find(|suffix| starts.contains(*suffix))
            .map(<[u64]>::to_vec)
            .unwrap_or_default()
    }

    fn edge(&self, raw: usize) -> Option<Rc<Edge>> {
        if let Some(edge) = self.edges.borrow().get(&raw) {
            return Some(edge.clone());
        }
        let record = self
            .access
            .edge_record_by_raw_edge_index(raw)?
            .to_view_edge();
        let costing = match &self.unblocked {
            Some((relaxed, edges)) if edges.contains(&record.edge.edge_id) => relaxed,
            _ => &self.costing,
        };
        let costed = match costing.edge_cost(&record) {
            EdgeCostDecision::Allowed(costed) => Some((costed.cost, costed.duration_s)),
            EdgeCostDecision::Blocked | EdgeCostDecision::NotAccessible => None,
        };
        let edge = Rc::new(Edge { record, costed });
        let mut edges = self.edges.borrow_mut();
        if edges.len() >= RECORD_CACHE_LIMIT {
            edges.clear();
        }
        edges.insert(raw, edge.clone());
        Some(edge)
    }

    fn edge_by_id(&self, edge_id: u64) -> Option<Rc<Edge>> {
        self.edge(self.access.raw_edge_index_for_edge_id(edge_id)?)
            .filter(|edge| edge.id() == edge_id)
    }

    fn out_edges(&self, node: usize) -> Vec<usize> {
        self.access
            .outgoing_raw_edge_indexes_from_raw_node_index(node)
            .map(<[usize]>::to_vec)
            .unwrap_or_default()
    }

    fn turn(&self, previous: &Edge, next: &Edge) -> f64 {
        let turn = self
            .costing
            .transition_cost_for_records(&previous.record, &next.record);
        turn + self.transition(previous, next)
    }

    /// The request's cost moving from `previous` onto `next` on the road
    /// graph's records: a SAR request's R2 entry into a wrong-way stretch,
    /// and the barrier at the node (see
    /// `crate::sar::barrier_applies`), costed as `crate::transitions` does.
    fn transition(&self, previous: &Edge, next: &Edge) -> f64 {
        use wayfarer_package::barrier::BarrierAccess;
        use wayfarer_package::sar::{ACCESS_WRONG_WAY, SarBarrierClass, edge_sar_bits};
        let mut cost = 0.0;
        if let Some(entry_s) = self.transitions.sar.and_then(|sar| sar.wrong_way_entry_s) {
            let wrong = |edge: &Edge| {
                edge_sar_bits(
                    &edge.record.shared_info,
                    edge.record.edge.traversal_direction,
                    edge.record.edge.mobility_mode_mask,
                ) & ACCESS_WRONG_WAY
                    != 0
            };
            if wrong(next) && !wrong(previous) {
                cost += entry_s;
            }
        }
        let barrier = &next.record.source;
        let barrier_cost = self.transitions.barrier_cost_of(
            SarBarrierClass::from_code(barrier.barrier_class),
            BarrierAccess(barrier.barrier_access),
        );
        // A U-turn on one segment does not pass the barrier.
        if barrier_cost == 0.0
            || previous.record.shared_info.shared_edge_info_id
                == next.record.shared_info.shared_edge_info_id
        {
            return cost;
        }
        let node = next.record.source_index as usize;
        let incident = self
            .access
            .outgoing_raw_edge_indexes_from_raw_node_index(node)
            .unwrap_or(&[])
            .iter()
            .chain(
                self.access
                    .incoming_raw_edge_indexes_to_raw_node_index(node)
                    .unwrap_or(&[]),
            )
            .filter_map(|raw| self.access.edge_record_by_raw_edge_index(*raw))
            .map(|record| {
                (
                    record.shared_info.shared_edge_info_id,
                    record.shared_info.osm_way_id,
                )
            })
            .collect::<Vec<_>>();
        let ends = |shared: u64, way: i64| {
            !incident
                .iter()
                .any(|(other, other_way)| *other != shared && way != 0 && *other_way == way)
        };
        let ends_edge = |edge: &Edge| {
            ends(
                edge.record.shared_info.shared_edge_info_id,
                edge.record.shared_info.osm_way_id,
            )
        };
        if ends_edge(previous)
            || ends_edge(next)
            || incident.iter().all(|(shared, way)| !ends(*shared, *way))
        {
            cost += barrier_cost;
        }
        cost
    }

    /// The junction delay added to durations wherever [`Self::turn`] is
    /// added to costs.
    fn delay(&self, previous: &Edge, next: &Edge) -> f64 {
        self.costing
            .junction_delay_for_records(&previous.record, &next.record)
    }

    fn simple_restricted(&self, previous: &Edge, next: &Edge) -> bool {
        self.access.transition_restricted(
            previous.id(),
            next.id(),
            next.record.source.id,
            self.profile,
        )
    }

    /// The restriction cost of entering `next` from `previous` (`traversed`:
    /// else an untraversed seed, outside every simple restriction) after
    /// `path`: `None` when a restriction bans it, else the SAR penalty for
    /// each restriction it breaks (each simple one, and each distinct complex
    /// one whose required prefix `path` ends with).
    fn restriction_penalty(
        &self,
        previous: &Edge,
        traversed: bool,
        path: impl FnOnce() -> Vec<u64>,
        next: &Edge,
    ) -> Option<f64> {
        let Some(penalty) = self.restriction_penalty else {
            let restricted = (traversed && self.simple_restricted(previous, next))
                || self.complex_restricted(path, next);
            return (!restricted).then_some(0.0);
        };
        let simple = traversed && self.simple_restricted(previous, next);
        let complex = if self
            .access
            .has_complex_restrictions_to_edge(next.id(), self.profile)
        {
            self.access
                .complex_restriction_count_for_path(&path(), next.id(), self.profile)
        } else {
            0
        };
        Some(penalty * (usize::from(simple) + complex) as f64)
    }

    /// Whether `path` (traversed edge ids, oldest first) ends with a complex
    /// restriction's required prefix towards `next`.
    fn complex_restricted(&self, path: impl FnOnce() -> Vec<u64>, next: &Edge) -> bool {
        self.access
            .has_complex_restrictions_to_edge(next.id(), self.profile)
            && self
                .access
                .complex_transition_restricted_for_path(&path(), next.id(), self.profile)
    }
}

// ---------------------------------------------------------------------------
// Queue
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq)]
enum Item {
    Label(u32),
    Finish(u32),
}

struct Entry {
    cost: f64,
    seq: u64,
    item: Item,
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
    // Min-heap on (cost, insertion order).
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .total_cmp(&self.cost)
            .then_with(|| other.seq.cmp(&self.seq))
    }
}

#[derive(Default)]
struct Queue {
    heap: BinaryHeap<Entry>,
    seq: u64,
}

impl Queue {
    fn push(&mut self, cost: f64, item: Item) {
        // A closed barrier closes a transition with an infinite cost.
        if !cost.is_finite() {
            return;
        }
        self.seq += 1;
        self.heap.push(Entry {
            cost,
            seq: self.seq,
            item,
        });
    }

    fn pop(&mut self) -> Option<(f64, Item)> {
        self.heap.pop().map(|entry| (entry.cost, entry.item))
    }
}

/// Whether two path costs tie: equal up to floating-point summation order.
fn tie(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * a.abs().max(b.abs()).max(1.0)
}

/// Settles each target at its cheapest finish. Finishes that tie go to the
/// path with the fewest edges, then the smallest edge ids, as the compact
/// search breaks ties, so equal-cost paths compare equal more often.
struct Settler<P> {
    open: BTreeSet<usize>,
    pending: FxHashMap<usize, (f64, Vec<u64>, P)>,
    paths: Vec<Option<P>>,
}

impl<P> Settler<P> {
    fn new(open: BTreeSet<usize>, target_count: usize) -> Self {
        Self {
            open,
            pending: FxHashMap::default(),
            paths: std::iter::repeat_with(|| None).take(target_count).collect(),
        }
    }

    fn done(&self) -> bool {
        self.open.is_empty()
    }

    /// The queue has reached `cost`: every pending finish below it, and not
    /// tied with it, is final.
    fn advance(&mut self, cost: f64) {
        let settled = self
            .pending
            .iter()
            .filter(|(_, (pending, _, _))| cost > *pending && !tie(cost, *pending))
            .map(|(target, _)| *target)
            .collect::<Vec<_>>();
        for target in settled {
            let (_, _, path) = self.pending.remove(&target).expect("pending target");
            self.paths[target] = Some(path);
            self.open.remove(&target);
        }
    }

    fn offer(&mut self, target: usize, cost: f64, edge_ids: Vec<u64>, path: impl FnOnce() -> P) {
        if !self.open.contains(&target) {
            return;
        }
        let better = self.pending.get(&target).is_none_or(|(best, ids, _)| {
            tie(cost, *best) && (edge_ids.len(), &edge_ids) < (ids.len(), ids)
        });
        if better {
            self.pending.insert(target, (cost, edge_ids, path()));
        }
    }

    fn finish(mut self) -> Vec<Option<P>> {
        for (target, (_, _, path)) in self.pending.drain() {
            self.paths[target] = Some(path);
        }
        self.paths
    }
}

// ---------------------------------------------------------------------------
// Plain search (no overlay patch)
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct Label {
    raw: usize,
    pred: u32,
    cost: f64,
    duration_s: f64,
    distance_m: f64,
    origin: usize,
    traversed: bool,
}

struct PlainDestination {
    target: usize,
    location: usize,
    edge: Rc<Edge>,
    fraction: f64,
    approach: f64,
}

struct PlainFinish {
    label: u32,
    destination: usize,
    cost: f64,
    /// The junction delay of turning onto the destination edge.
    delay_s: f64,
}

#[derive(Debug, Clone)]
struct PlainPath {
    /// Traversed edges, ending with the destination edge.
    edge_ids: Vec<u64>,
    origin: usize,
    /// Index into the destination candidates given to the search.
    destination: usize,
    cost: f64,
    /// The cost with both endpoints' approach costs, which paths compete on.
    total: f64,
    duration_s: f64,
    distance_m: f64,
}

/// One label per (edge, context), the context being the last `context`
/// traversed edge ids (empty with no path context).
struct PlainLabels {
    labels: Vec<Label>,
    contexts: Vec<Vec<u64>>,
    slot: FxHashMap<(usize, Vec<u64>), u32>,
    context: usize,
}

impl PlainLabels {
    fn new(context: usize) -> Self {
        Self {
            labels: Vec::new(),
            contexts: Vec::new(),
            slot: FxHashMap::default(),
            context,
        }
    }

    fn push(&mut self, records: &Records<'_>, queue: &mut Queue, label: Label) {
        let mut context = match label.pred {
            NONE => Vec::new(),
            pred => self.contexts[pred as usize].clone(),
        };
        if self.context > 0 && label.traversed {
            if let Some(edge) = records.edge(label.raw) {
                context.push(edge.id());
            }
            if context.len() > self.context {
                context.remove(0);
            }
        }
        let key = (label.raw, context);
        if let Some(slot) = self.slot.get(&key)
            && self.labels[*slot as usize].cost <= label.cost
        {
            return;
        }
        let index = self.labels.len() as u32;
        self.labels.push(label);
        self.contexts.push(key.1.clone());
        self.slot.insert(key, index);
        queue.push(label.cost, Item::Label(index));
    }

    fn is_current(&self, index: u32) -> bool {
        let label = &self.labels[index as usize];
        self.slot
            .get(&(label.raw, self.contexts[index as usize].clone()))
            == Some(&index)
    }

    /// Traversed edges from the seed to `label`, oldest first.
    fn traversed_path(&self, records: &Records<'_>, label: u32) -> Vec<u64> {
        let mut path = Vec::new();
        let mut current = label;
        while current != NONE {
            let entry = self.labels[current as usize];
            if entry.traversed
                && let Some(edge) = records.edge(entry.raw)
            {
                path.push(edge.id());
            }
            current = entry.pred;
        }
        path.reverse();
        path
    }

    /// Whether finishing on `destination` after `label` repeats an edge on
    /// the path; the origin seed's piece of the destination edge counts only
    /// where the two overlap (a destination behind the origin is reached
    /// around a loop).
    fn finish_repeats(
        &self,
        label: u32,
        destination: &PlainDestination,
        origins: &[CorrelatedLocation],
    ) -> bool {
        let raw = destination.edge.record.raw_edge_index;
        let mut current = label;
        while current != NONE {
            let entry = self.labels[current as usize];
            if entry.traversed && entry.raw == raw {
                return entry.pred != NONE
                    || origins[entry.origin].fraction_along_edge.clamp(0.0, 1.0)
                        <= destination.fraction;
            }
            current = entry.pred;
        }
        false
    }

    /// See [`Records::restriction_penalty`].
    fn restriction_penalty(
        &self,
        records: &Records<'_>,
        label: u32,
        from: &Edge,
        next: &Edge,
    ) -> Option<f64> {
        records.restriction_penalty(
            from,
            self.labels[label as usize].traversed,
            || self.traversed_path(records, label),
            next,
        )
    }
}

/// Shortest paths from `origins` to each target, where `destinations` lists
/// every target's candidates as `(target, candidate)`. One label per edge;
/// a target is final when its first finish leaves the queue.
fn plain_search(
    records: &Records<'_>,
    origins: &[CorrelatedLocation],
    destinations: Vec<(usize, &CorrelatedLocation)>,
    target_count: usize,
    context: usize,
) -> Vec<Option<PlainPath>> {
    let approach = approach_costs(&destinations);
    let origin_approach = endpoint_approach(origins);
    let destinations = destinations
        .into_iter()
        .enumerate()
        .filter_map(|(location, (target, candidate))| {
            Some(PlainDestination {
                target,
                location,
                edge: records.edge_by_id(candidate.edge_id)?,
                fraction: candidate.fraction_along_edge.clamp(0.0, 1.0),
                approach: approach[location],
            })
        })
        .collect::<Vec<_>>();
    let mut settler = Settler::new(
        destinations
            .iter()
            .map(|destination| destination.target)
            .collect(),
        target_count,
    );
    let mut at_node = FxHashMap::<usize, Vec<usize>>::default();
    for (index, destination) in destinations.iter().enumerate() {
        at_node
            .entry(destination.edge.source())
            .or_default()
            .push(index);
    }
    let mut labels = PlainLabels::new(context);
    let mut finishes = Vec::<PlainFinish>::new();
    let mut queue = Queue::default();
    for (index, origin) in origins.iter().enumerate() {
        let Some(edge) = records.edge_by_id(origin.edge_id) else {
            continue;
        };
        let Some((cost, duration_s)) = edge.costed else {
            continue;
        };
        let remaining = (1.0 - origin.fraction_along_edge).clamp(0.0, 1.0);
        labels.push(
            records,
            &mut queue,
            Label {
                raw: edge.record.raw_edge_index,
                pred: NONE,
                cost: cost * remaining + origin_approach[index],
                duration_s: duration_s * remaining,
                distance_m: edge.length() * remaining,
                origin: index,
                traversed: remaining > TRAVERSED_EPSILON,
            },
        );
    }
    while let Some((cost, item)) = queue.pop() {
        settler.advance(cost);
        if settler.done() {
            break;
        }
        let index = match item {
            Item::Finish(index) => {
                let finish = &finishes[index as usize];
                let destination = &destinations[finish.destination];
                let path = plain_path(records, &labels, finish, destination, &origin_approach);
                settler.offer(
                    destination.target,
                    finish.cost,
                    path.edge_ids.clone(),
                    || path,
                );
                continue;
            }
            Item::Label(index) => index,
        };
        let label = labels.labels[index as usize];
        if !labels.is_current(index) {
            continue;
        }
        let Some(edge) = records.edge(label.raw) else {
            continue;
        };
        let node = edge.target();
        for destination_index in at_node.get(&node).into_iter().flatten() {
            let destination = &destinations[*destination_index];
            let Some(penalty) =
                labels.restriction_penalty(records, index, &edge, &destination.edge)
            else {
                continue;
            };
            let Some((cost, _)) = destination.edge.costed else {
                continue;
            };
            if labels.finish_repeats(index, destination, origins) {
                continue;
            }
            let (turn, delay_s) = if label.traversed {
                (
                    records.turn(&edge, &destination.edge),
                    records.delay(&edge, &destination.edge),
                )
            } else {
                (0.0, 0.0)
            };
            let total =
                label.cost + turn + cost * destination.fraction + destination.approach + penalty;
            finishes.push(PlainFinish {
                label: index,
                destination: *destination_index,
                cost: total,
                delay_s,
            });
            queue.push(total, Item::Finish(finishes.len() as u32 - 1));
        }
        for raw in records.out_edges(node) {
            let Some(next) = records.edge(raw) else {
                continue;
            };
            let Some((cost, duration_s)) = next.costed else {
                continue;
            };
            let Some(penalty) = labels.restriction_penalty(records, index, &edge, &next) else {
                continue;
            };
            labels.push(
                records,
                &mut queue,
                Label {
                    raw,
                    pred: index,
                    cost: label.cost + cost + records.turn(&edge, &next) + penalty,
                    duration_s: label.duration_s + records.delay(&edge, &next) + duration_s,
                    distance_m: label.distance_m + next.length(),
                    origin: label.origin,
                    traversed: true,
                },
            );
        }
    }
    settler.finish()
}

fn plain_path(
    records: &Records<'_>,
    labels: &PlainLabels,
    finish: &PlainFinish,
    destination: &PlainDestination,
    origin_approach: &[f64],
) -> PlainPath {
    let label = labels.labels[finish.label as usize];
    let mut seed = finish.label;
    while labels.labels[seed as usize].pred != NONE {
        seed = labels.labels[seed as usize].pred;
    }
    let mut edge_ids = labels.traversed_path(records, finish.label);
    edge_ids.push(destination.edge.id());
    let origin = labels.labels[seed as usize].origin;
    PlainPath {
        edge_ids,
        origin,
        destination: destination.location,
        cost: finish.cost - origin_approach[origin] - destination.approach,
        total: finish.cost,
        duration_s: label.duration_s
            + finish.delay_s
            + destination.edge.duration() * destination.fraction,
        distance_m: label.distance_m + destination.edge.length() * destination.fraction,
    }
}

/// The cheapest direct path from an origin to a destination ahead of it on
/// the same edge, approach costs included in its total (not in its cost). It
/// competes with the searched path and wins ties.
fn cheapest_direct(
    records: &Records<'_>,
    origins: &[CorrelatedLocation],
    destinations: &[CorrelatedLocation],
) -> Option<PlainPath> {
    let mut best = None::<(f64, PlainPath)>;
    let (origin_approach, destination_approach) =
        (endpoint_approach(origins), endpoint_approach(destinations));
    for (origin_index, origin) in origins.iter().enumerate() {
        for (destination_index, destination) in destinations.iter().enumerate() {
            if origin.edge_id != destination.edge_id
                || origin.fraction_along_edge > destination.fraction_along_edge
            {
                continue;
            }
            let Some(edge) = records.edge_by_id(origin.edge_id) else {
                continue;
            };
            let Some((cost, duration_s)) = edge.costed else {
                continue;
            };
            let fraction =
                (destination.fraction_along_edge - origin.fraction_along_edge).clamp(0.0, 1.0);
            let total = cost * fraction
                + origin_approach[origin_index]
                + destination_approach[destination_index];
            if best.as_ref().is_none_or(|(best, _)| total < *best) {
                best = Some((
                    total,
                    PlainPath {
                        edge_ids: vec![origin.edge_id],
                        origin: origin_index,
                        destination: destination_index,
                        cost: cost * fraction,
                        total,
                        duration_s: duration_s * fraction,
                        distance_m: edge.length() * fraction,
                    },
                ));
            }
        }
    }
    best.map(|(_, path)| path)
}

fn plain_route(
    access: &PackageGraphView,
    origin: &CorrelatedLocation,
    destination: &CorrelatedLocation,
    path: PlainPath,
) -> SearchRoute {
    SearchRoute {
        origin: origin.clone(),
        destination: destination.clone(),
        edge_indexes: path
            .edge_ids
            .iter()
            .map(|id| access.raw_edge_index_for_edge_id(*id))
            .collect(),
        edge_ids: path.edge_ids,
        steps: Vec::new(),
        distance_m: path.distance_m,
        duration_s: path.duration_s,
        cost: path.cost,
        diagnostics: RouteDiagnostics::default(),
    }
}

// ---------------------------------------------------------------------------
// Overlay patch search
// ---------------------------------------------------------------------------

/// A node of the patched graph.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum Place {
    Real(usize),
    /// A point inside an edge: (edge id, fraction key).
    Point(u64, i64),
    Origin,
    Destination,
}

#[derive(Debug, Clone, Copy)]
enum ArcKind {
    /// Part of an edge between two stops (or, from no place, a seed from an
    /// origin point to the next stop).
    Fragment {
        start: f64,
        end: f64,
        split: bool,
    },
    Connector(usize),
    /// Into destination candidate `destination`.
    Finish {
        destination: usize,
        start: f64,
        end: f64,
        split: bool,
    },
    /// Out of the destination endpoint.
    End,
}

#[derive(Clone)]
struct PatchArc {
    from: Option<Place>,
    to: Option<Place>,
    kind: ArcKind,
    edge: Option<Rc<Edge>>,
    cost: f64,
    duration_s: f64,
    distance_m: f64,
}

impl PatchArc {
    fn enters(&self) -> bool {
        matches!(
            self.kind,
            ArcKind::Fragment { start, .. } | ArcKind::Finish { start, .. }
                if start <= FRACTION_EPSILON
        )
    }

    fn traversed(&self) -> bool {
        match self.kind {
            ArcKind::Fragment { start, end, .. } => {
                self.from.is_some() || end - start > TRAVERSED_EPSILON
            }
            ArcKind::Finish { start, end, .. } => end - start > TRAVERSED_EPSILON,
            ArcKind::Connector(_) | ArcKind::End => false,
        }
    }

    fn terminal(&self) -> bool {
        matches!(self.kind, ArcKind::Finish { .. } | ArcKind::End)
    }
}

fn fraction_key(fraction: f64) -> i64 {
    (fraction.clamp(0.0, 1.0) * 1_000_000.0).round() as i64
}

/// The overlay as a patch over the road graph.
struct OverlayModel<'o> {
    overlay: &'o RouteDynamicOverlay,
    split: FxHashMap<u64, Vec<(f64, f64)>>,
    stops: FxHashMap<u64, Vec<(f64, Place)>>,
    origin_endpoint: bool,
    destination_endpoint: bool,
    arcs: Vec<PatchArc>,
    out: FxHashMap<Place, Vec<usize>>,
}

impl<'o> OverlayModel<'o> {
    fn new(records: &Records<'_>, overlay: &'o RouteDynamicOverlay) -> Self {
        let mut model = Self {
            overlay,
            split: FxHashMap::default(),
            stops: FxHashMap::default(),
            origin_endpoint: false,
            destination_endpoint: false,
            arcs: Vec::new(),
            out: FxHashMap::default(),
        };
        let mut attachments = Vec::<(u64, f64)>::new();
        for split in &overlay.split_edge_ranges {
            if records.edge_by_id(split.edge_id).is_none() {
                continue;
            }
            let ranges = split
                .usable_ranges
                .iter()
                .map(|range| {
                    (
                        range.start_fraction.clamp(0.0, 1.0),
                        range.end_fraction.clamp(0.0, 1.0),
                    )
                })
                .filter(|(start, end)| end > start)
                .collect::<Vec<_>>();
            for (start, end) in &ranges {
                attachments.push((split.edge_id, *start));
                attachments.push((split.edge_id, *end));
            }
            model.split.insert(split.edge_id, ranges);
        }
        let mut ends = Vec::with_capacity(overlay.virtual_edges.len());
        for connector in &overlay.virtual_edges {
            let end = |model: &mut Self,
                       attachments: &mut Vec<(u64, f64)>,
                       endpoint: Option<RouteDynamicEndpointKind>,
                       point: Option<GeoPoint>,
                       edge_id: Option<u64>,
                       fraction: Option<f64>,
                       default: f64| {
                match endpoint {
                    Some(kind) => {
                        point?;
                        Some(match kind {
                            RouteDynamicEndpointKind::Origin => {
                                model.origin_endpoint = true;
                                Place::Origin
                            }
                            RouteDynamicEndpointKind::Destination => {
                                model.destination_endpoint = true;
                                Place::Destination
                            }
                        })
                    }
                    None => {
                        let edge = records.edge_by_id(edge_id?)?;
                        let at = fraction.unwrap_or(default).clamp(0.0, 1.0);
                        if fraction.is_some() {
                            attachments.push((edge.id(), at));
                        }
                        Some(place_at(&edge, at))
                    }
                }
            };
            let source = end(
                &mut model,
                &mut attachments,
                connector.source_endpoint,
                connector.source_point,
                connector.source_edge_id,
                connector.source_fraction,
                1.0,
            );
            let target = end(
                &mut model,
                &mut attachments,
                connector.target_endpoint,
                connector.target_point,
                connector.target_edge_id,
                connector.target_fraction,
                0.0,
            );
            ends.push(source.zip(target));
        }
        // Stops: every attachment inside a usable range of its edge.
        let mut by_edge = FxHashMap::<u64, Vec<f64>>::default();
        for (edge, fraction) in attachments {
            by_edge.entry(edge).or_default().push(fraction);
        }
        let mut by_edge = by_edge.into_iter().collect::<Vec<_>>();
        by_edge.sort_by_key(|(edge, _)| *edge);
        for (edge_id, mut fractions) in by_edge {
            let Some(edge) = records.edge_by_id(edge_id) else {
                continue;
            };
            fractions.sort_by(f64::total_cmp);
            fractions.dedup_by_key(|fraction| fraction_key(*fraction));
            let split = model.split.contains_key(&edge_id);
            let ranges = model
                .split
                .get(&edge_id)
                .cloned()
                .unwrap_or_else(|| vec![(0.0, 1.0)]);
            let mut stops = Vec::new();
            for (start, end) in ranges {
                let inside = fractions
                    .iter()
                    .copied()
                    .filter(|fraction| {
                        *fraction + FRACTION_EPSILON >= start && *fraction <= end + FRACTION_EPSILON
                    })
                    .collect::<Vec<_>>();
                if !split
                    && inside.iter().all(|fraction| {
                        *fraction <= FRACTION_EPSILON || *fraction >= 1.0 - FRACTION_EPSILON
                    })
                {
                    continue;
                }
                let mut range_stops = std::iter::once(start)
                    .chain(inside)
                    .chain(std::iter::once(end))
                    .map(|fraction| fraction.clamp(start, end))
                    .collect::<Vec<_>>();
                range_stops.sort_by(f64::total_cmp);
                range_stops.dedup_by_key(|fraction| fraction_key(*fraction));
                for pair in range_stops.windows(2) {
                    let (from, to) = (place_at(&edge, pair[0]), place_at(&edge, pair[1]));
                    if from == to {
                        continue;
                    }
                    let fraction = pair[1] - pair[0];
                    model.push(PatchArc {
                        from: Some(from),
                        to: Some(to),
                        kind: ArcKind::Fragment {
                            start: pair[0],
                            end: pair[1],
                            split: true,
                        },
                        edge: Some(edge.clone()),
                        cost: edge.cost() * fraction,
                        duration_s: edge.duration() * fraction,
                        distance_m: edge.length() * fraction,
                    });
                }
                for fraction in range_stops {
                    stops.push((fraction, place_at(&edge, fraction)));
                }
            }
            if !stops.is_empty() {
                stops.sort_by(|a, b| a.0.total_cmp(&b.0));
                model.stops.insert(edge_id, stops);
            }
        }
        for (index, (connector, ends)) in overlay.virtual_edges.iter().zip(ends).enumerate() {
            let Some((from, to)) = ends else {
                continue;
            };
            if from == to || !connector.cost.is_finite() || connector.cost < 0.0 {
                continue;
            }
            model.push(PatchArc {
                from: Some(from),
                to: Some(to),
                kind: ArcKind::Connector(index),
                edge: None,
                cost: connector.cost,
                duration_s: connector.cost,
                distance_m: connector.distance_m,
            });
        }
        model
    }

    fn push(&mut self, arc: PatchArc) {
        if let Some(from) = arc.from {
            self.out.entry(from).or_default().push(self.arcs.len());
        }
        self.arcs.push(arc);
    }

    /// Nothing but full blocks, which the costing policy applies.
    fn is_empty(&self) -> bool {
        self.arcs.is_empty()
            && !self.origin_endpoint
            && !self.destination_endpoint
            && self.split.values().all(Vec::is_empty)
    }

    fn has_connectors(&self) -> bool {
        self.arcs
            .iter()
            .any(|arc| matches!(arc.kind, ArcKind::Connector(_)))
    }

    fn usable_range(&self, edge_id: u64, from: f64, to: f64) -> Option<(f64, f64)> {
        match self.split.get(&edge_id) {
            Some(ranges) => ranges.iter().copied().find(|(start, end)| {
                from + FRACTION_EPSILON >= *start && to <= *end + FRACTION_EPSILON
            }),
            None => Some((0.0, 1.0)),
        }
    }

    /// Seed arcs `(arc, origin)`: the origin endpoint's connectors when the
    /// overlay has one, else one per origin candidate to its next stop.
    fn seeds(
        &self,
        records: &Records<'_>,
        origins: &[CorrelatedLocation],
        arcs: &mut Vec<PatchArc>,
    ) -> Vec<(usize, usize)> {
        if self.origin_endpoint {
            return self
                .out
                .get(&Place::Origin)
                .into_iter()
                .flatten()
                .map(|arc| (*arc, usize::MAX))
                .collect();
        }
        let mut seeds = Vec::new();
        let approach = endpoint_approach(origins);
        for (index, location) in origins.iter().enumerate() {
            let Some(edge) = records.edge_by_id(location.edge_id) else {
                continue;
            };
            let from = location.fraction_along_edge.clamp(0.0, 1.0);
            if !edge.cost().is_finite() {
                continue;
            }
            let split = self.split.contains_key(&edge.id());
            let next = match self.stops.get(&edge.id()) {
                Some(stops) => {
                    let Some(range) = self.usable_range(edge.id(), from, from) else {
                        continue;
                    };
                    let within = |fraction: f64| fraction <= range.1 + FRACTION_EPSILON;
                    stops
                        .iter()
                        .copied()
                        .find(|(fraction, _)| *fraction > from + 1e-9 && within(*fraction))
                        .or_else(|| {
                            stops
                                .iter()
                                .copied()
                                .find(|(fraction, _)| *fraction + 1e-9 >= from && within(*fraction))
                        })
                        .map(|(fraction, place)| (fraction.max(from), place, true))
                }
                None if split => None,
                None => Some((1.0, Place::Real(edge.target()), false)),
            };
            let Some((to, place, fragment)) = next else {
                continue;
            };
            let fraction = (to - from).max(0.0);
            arcs.push(PatchArc {
                from: None,
                to: Some(place),
                kind: ArcKind::Fragment {
                    start: from,
                    end: to,
                    split: fragment && (split || to < 1.0 - FRACTION_EPSILON),
                },
                cost: edge.cost() * fraction + approach[index],
                duration_s: edge.duration() * fraction,
                distance_m: edge.length() * fraction,
                edge: Some(edge),
            });
            seeds.push((arcs.len() - 1, index));
        }
        seeds
    }

    /// Finish arcs into the destination candidates (or the end arc out of the
    /// destination endpoint), keyed by their start place.
    fn finishes(
        &self,
        records: &Records<'_>,
        destinations: &[(usize, &CorrelatedLocation)],
        arcs: &mut Vec<PatchArc>,
    ) -> FxHashMap<Place, Vec<usize>> {
        let mut out = FxHashMap::<Place, Vec<usize>>::default();
        let mut push = |arcs: &mut Vec<PatchArc>, arc: PatchArc| {
            if let Some(from) = arc.from {
                out.entry(from).or_default().push(arcs.len());
            }
            arcs.push(arc);
        };
        if self.destination_endpoint {
            push(
                arcs,
                PatchArc {
                    from: Some(Place::Destination),
                    to: None,
                    kind: ArcKind::End,
                    edge: None,
                    cost: 0.0,
                    duration_s: 0.0,
                    distance_m: 0.0,
                },
            );
            return out;
        }
        let approach = approach_costs(destinations);
        for (index, (_, location)) in destinations.iter().enumerate() {
            let Some(edge) = records.edge_by_id(location.edge_id) else {
                continue;
            };
            let end = location.fraction_along_edge.clamp(0.0, 1.0);
            let split = self.split.contains_key(&edge.id());
            if !edge.cost().is_finite() {
                continue;
            }
            let mut options = Vec::new();
            match self.stops.get(&edge.id()) {
                Some(stops) => {
                    let range = self.usable_range(edge.id(), end, end);
                    if split && range.is_none() {
                        continue;
                    }
                    let (start, _) = range.unwrap_or((0.0, 1.0));
                    for (fraction, place) in stops {
                        if *fraction + FRACTION_EPSILON >= start && *fraction <= end + 1e-9 {
                            options.push((*fraction, *place, true));
                        }
                    }
                }
                None if split => continue,
                None => {}
            }
            if !split {
                options.push((0.0, Place::Real(edge.source()), false));
            }
            for (start, place, fragment) in options {
                let start = start.min(end);
                let fraction = end - start;
                push(
                    arcs,
                    PatchArc {
                        from: Some(place),
                        to: None,
                        kind: ArcKind::Finish {
                            destination: index,
                            start,
                            end,
                            split: fragment && (split || start > FRACTION_EPSILON),
                        },
                        edge: Some(edge.clone()),
                        cost: edge.cost() * fraction + approach[index],
                        duration_s: edge.duration() * fraction,
                        distance_m: edge.length() * fraction,
                    },
                );
            }
        }
        out
    }

    /// Direct same-edge paths as finish arcs `(origin, arc)`.
    fn directs(
        &self,
        records: &Records<'_>,
        origins: &[CorrelatedLocation],
        destinations: &[(usize, &CorrelatedLocation)],
        arcs: &mut Vec<PatchArc>,
    ) -> Vec<(usize, usize)> {
        let mut directs = Vec::new();
        if self.origin_endpoint || self.destination_endpoint {
            return directs;
        }
        let (origin_approach, destination_approach) =
            (endpoint_approach(origins), approach_costs(destinations));
        for (o, origin) in origins.iter().enumerate() {
            for (d, (_, destination)) in destinations.iter().enumerate() {
                let (from, to) = (
                    origin.fraction_along_edge.clamp(0.0, 1.0),
                    destination.fraction_along_edge.clamp(0.0, 1.0),
                );
                if origin.edge_id != destination.edge_id || from > to {
                    continue;
                }
                let Some(edge) = records.edge_by_id(origin.edge_id) else {
                    continue;
                };
                let cost = edge.cost() * (to - from) + origin_approach[o] + destination_approach[d];
                if self.usable_range(edge.id(), from, to).is_none() || !cost.is_finite() {
                    continue;
                }
                arcs.push(PatchArc {
                    from: None,
                    to: None,
                    kind: ArcKind::Finish {
                        destination: d,
                        start: from,
                        end: to,
                        split: self.split.contains_key(&edge.id()),
                    },
                    cost,
                    duration_s: edge.duration() * (to - from),
                    distance_m: edge.length() * (to - from),
                    edge: Some(edge),
                });
                directs.push((o, arcs.len() - 1));
            }
        }
        directs
    }
}

/// The place at `fraction` along `edge`: its end nodes at the ends, else a
/// point inside it.
fn place_at(edge: &Edge, fraction: f64) -> Place {
    if fraction <= FRACTION_EPSILON {
        Place::Real(edge.source())
    } else if fraction >= 1.0 - FRACTION_EPSILON {
        Place::Real(edge.target())
    } else {
        Place::Point(edge.id(), fraction_key(fraction))
    }
}

/// An arc of one overlay search: a real edge (unsplit), or a patch arc.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum ArcKey {
    Edge(usize),
    Patch(usize),
}

#[derive(Clone)]
struct ArcInfo {
    head: Option<Place>,
    edge: Option<Rc<Edge>>,
    enters: bool,
    traversed: bool,
    terminal: bool,
    cost: f64,
    duration_s: f64,
    distance_m: f64,
}

#[derive(Clone)]
struct OverlayLabel {
    arc: ArcKey,
    pred: u32,
    cost: f64,
    duration_s: f64,
    distance_m: f64,
    last_edge: Option<Rc<Edge>>,
    traversed: bool,
    origin: usize,
}

struct OverlayFinish {
    /// `NONE` for a direct path.
    label: u32,
    arc: usize,
    origin: usize,
    cost: f64,
    /// The junction delay of entering the finish arc.
    delay_s: f64,
}

#[derive(Debug, Clone)]
struct OverlayPath {
    steps: Vec<RouteSearchStep>,
    origin: Option<usize>,
    destination: Option<usize>,
    cost: f64,
    duration_s: f64,
    distance_m: f64,
}

impl OverlayPath {
    fn matrix_path(self) -> MatrixPath {
        MatrixPath {
            edge_ids: self.steps.iter().map(RouteSearchStep::edge_id).collect(),
            distance_m: self.distance_m,
            duration_s: self.duration_s,
            cost: self.cost,
        }
    }
}

struct OverlaySearch<'r, 'm, 'o> {
    records: &'r Records<'r>,
    model: &'m OverlayModel<'o>,
    arcs: Vec<PatchArc>,
    finish_out: FxHashMap<Place, Vec<usize>>,
    labels: Vec<OverlayLabel>,
    /// Per label, the last `context_edges` edges of its restriction context.
    contexts: Vec<Vec<u64>>,
    slot: FxHashMap<(ArcKey, Vec<u64>), u32>,
    context_edges: usize,
}

impl OverlaySearch<'_, '_, '_> {
    fn info(&self, key: ArcKey) -> Option<ArcInfo> {
        match key {
            ArcKey::Edge(raw) => {
                let edge = self.records.edge(raw)?;
                // Split edges leave the static graph.
                let cost = if self.model.split.contains_key(&edge.id()) {
                    f64::INFINITY
                } else {
                    edge.cost()
                };
                Some(ArcInfo {
                    head: Some(Place::Real(edge.target())),
                    enters: true,
                    traversed: true,
                    terminal: false,
                    cost,
                    duration_s: edge.duration(),
                    distance_m: edge.length(),
                    edge: Some(edge),
                })
            }
            ArcKey::Patch(index) => {
                let arc = &self.arcs[index];
                Some(ArcInfo {
                    head: arc.to,
                    edge: arc.edge.clone(),
                    enters: arc.enters(),
                    traversed: arc.traversed(),
                    terminal: arc.terminal(),
                    cost: arc.cost,
                    duration_s: arc.duration_s,
                    distance_m: arc.distance_m,
                })
            }
        }
    }

    fn out_arcs(&self, place: Place) -> Vec<ArcKey> {
        let mut out = Vec::new();
        if let Place::Real(node) = place {
            out.extend(self.records.out_edges(node).into_iter().map(ArcKey::Edge));
        }
        for arc in self.model.out.get(&place).into_iter().flatten() {
            out.push(ArcKey::Patch(*arc));
        }
        for arc in self.finish_out.get(&place).into_iter().flatten() {
            out.push(ArcKey::Patch(*arc));
        }
        out
    }

    fn push(&mut self, queue: &mut Queue, label: OverlayLabel) {
        // The last `context_edges` edges of `context()` after this label.
        let mut context = Vec::new();
        if self.context_edges > 0
            && let Some(edge) = &label.last_edge
        {
            if label.pred != NONE {
                context = self.contexts[label.pred as usize].clone();
            }
            if label.traversed && context.last() != Some(&edge.id()) {
                context.push(edge.id());
            }
            if context.len() > self.context_edges {
                context.remove(0);
            }
        }
        let key = (label.arc, context);
        if let Some(slot) = self.slot.get(&key)
            && self.labels[*slot as usize].cost <= label.cost
        {
            return;
        }
        let index = self.labels.len() as u32;
        self.contexts.push(key.1.clone());
        self.slot.insert(key, index);
        queue.push(label.cost, Item::Label(index));
        self.labels.push(label);
    }

    fn is_current(&self, index: u32) -> bool {
        let key = (
            self.labels[index as usize].arc,
            self.contexts[index as usize].clone(),
        );
        self.slot.get(&key) == Some(&index)
    }

    /// Real edges traversed since the last connector, each run once, oldest
    /// first.
    fn context(&self, label: u32) -> Vec<u64> {
        let mut context = Vec::new();
        let mut current = label;
        while current != NONE {
            let entry = &self.labels[current as usize];
            current = entry.pred;
            let Some(edge) = &entry.last_edge else {
                break;
            };
            if !entry.traversed || context.last() == Some(&edge.id()) {
                continue;
            }
            context.push(edge.id());
        }
        context.reverse();
        context
    }

    /// Turn cost and junction delay onto `next` after `label`, or `None`
    /// when a restriction forbids it.
    fn transition(&self, label: u32, next: &ArcInfo) -> Option<(f64, f64)> {
        let entry = &self.labels[label as usize];
        let (Some(last), Some(edge)) = (&entry.last_edge, &next.edge) else {
            return Some((0.0, 0.0));
        };
        if !next.enters {
            return Some((0.0, 0.0));
        }
        let penalty = self.records.restriction_penalty(
            last,
            entry.traversed,
            || self.context(label),
            edge,
        )?;
        // A SAR restriction penalty is paid with the turn; a barrier
        // may close the transition.
        let turn = self.records.turn(last, edge);
        if !turn.is_finite() {
            return None;
        }
        Some((turn + penalty, self.records.delay(last, edge)))
    }

    fn steps(&self, finish: &OverlayFinish) -> Vec<RouteSearchStep> {
        let mut keys = vec![ArcKey::Patch(finish.arc)];
        let mut current = finish.label;
        while current != NONE {
            let entry = &self.labels[current as usize];
            keys.push(entry.arc);
            current = entry.pred;
        }
        keys.reverse();
        let mut steps = Vec::new();
        for key in keys {
            match key {
                ArcKey::Edge(raw) => {
                    if let Some(edge) = self.records.edge(raw) {
                        steps.push(graph_step(edge.id(), 0.0, 1.0, false));
                    }
                }
                ArcKey::Patch(index) => {
                    let arc = &self.arcs[index];
                    match arc.kind {
                        ArcKind::Fragment { start, end, split }
                        | ArcKind::Finish {
                            start, end, split, ..
                        } => {
                            let empty = end - start <= FRACTION_EPSILON;
                            let keep = match arc.kind {
                                ArcKind::Fragment { .. } => !(arc.from.is_none() && empty),
                                _ => !empty,
                            };
                            if keep && let Some(edge) = &arc.edge {
                                steps.push(graph_step(edge.id(), start, end, split));
                            }
                        }
                        ArcKind::Connector(index) => {
                            steps.push(connector_step(&self.model.overlay.virtual_edges[index]));
                        }
                        ArcKind::End => {}
                    }
                }
            }
        }
        steps
    }
}

fn graph_step(edge_id: u64, start: f64, end: f64, split: bool) -> RouteSearchStep {
    RouteSearchStep::GraphEdge {
        edge_id,
        start_fraction: start,
        end_fraction: end,
        quality_flags: if split {
            vec![RouteShapeQualityFlag::SsvgPartialEdgeSplit]
        } else {
            Vec::new()
        },
    }
}

fn connector_step(edge: &crate::types::RouteDynamicVirtualEdge) -> RouteSearchStep {
    let edge = edge.clone();
    RouteSearchStep::VirtualConnector {
        edge_id: edge.edge_id,
        connector_id: edge.connector_id,
        connector_type: edge.connector_type,
        geometry: edge.geometry,
        distance_m: edge.distance_m,
        cost: edge.cost,
        source_chunk_ids: edge.source_chunk_ids,
        source_endpoint: edge.source_endpoint,
        source_point: edge.source_point,
        source_edge_id: edge.source_edge_id,
        source_fraction: edge.source_fraction,
        target_endpoint: edge.target_endpoint,
        target_point: edge.target_point,
        target_edge_id: edge.target_edge_id,
        target_fraction: edge.target_fraction,
    }
}

/// Shortest patched paths from `origins` to each target (see
/// [`plain_search`] for the target bookkeeping).
fn overlay_search(
    records: &Records<'_>,
    model: &OverlayModel<'_>,
    origins: &[CorrelatedLocation],
    destinations: Vec<(usize, &CorrelatedLocation)>,
    target_count: usize,
    context_edges: usize,
) -> Vec<Option<OverlayPath>> {
    let mut arcs = model.arcs.clone();
    let (origin_approach, destination_approach) =
        (endpoint_approach(origins), approach_costs(&destinations));
    let seeds = model.seeds(records, origins, &mut arcs);
    let finish_out = model.finishes(records, &destinations, &mut arcs);
    let directs = model.directs(records, origins, &destinations, &mut arcs);
    let mut search = OverlaySearch {
        records,
        model,
        arcs,
        finish_out,
        labels: Vec::new(),
        contexts: Vec::new(),
        slot: FxHashMap::default(),
        context_edges,
    };
    let target_of = |arc: &PatchArc| match arc.kind {
        ArcKind::Finish { destination, .. } => Some(destinations[destination].0),
        _ => None,
    };
    let mut settler = Settler::new(
        if model.destination_endpoint {
            (0..target_count).collect()
        } else {
            destinations.iter().map(|(target, _)| *target).collect()
        },
        target_count,
    );
    let mut finishes = Vec::<OverlayFinish>::new();
    let mut queue = Queue::default();
    for (origin, arc) in directs {
        finishes.push(OverlayFinish {
            label: NONE,
            arc,
            origin,
            cost: search.arcs[arc].cost,
            delay_s: 0.0,
        });
        queue.push(
            search.arcs[arc].cost,
            Item::Finish(finishes.len() as u32 - 1),
        );
    }
    for (arc, origin) in seeds {
        let Some(info) = search.info(ArcKey::Patch(arc)) else {
            continue;
        };
        if !info.cost.is_finite() {
            continue;
        }
        search.push(
            &mut queue,
            OverlayLabel {
                arc: ArcKey::Patch(arc),
                pred: NONE,
                cost: info.cost,
                duration_s: info.duration_s,
                distance_m: info.distance_m,
                last_edge: info.edge,
                traversed: info.traversed,
                origin,
            },
        );
    }
    while let Some((cost, item)) = queue.pop() {
        settler.advance(cost);
        if settler.done() {
            break;
        }
        let index = match item {
            Item::Finish(index) => {
                let finish = &finishes[index as usize];
                let arc = &search.arcs[finish.arc];
                let (duration_s, distance_m) = match finish.label {
                    NONE => (0.0, 0.0),
                    label => {
                        let label = &search.labels[label as usize];
                        (label.duration_s, label.distance_m)
                    }
                };
                let origin = (finish.origin != usize::MAX).then_some(finish.origin);
                let destination = match arc.kind {
                    ArcKind::Finish { destination, .. } => Some(destination),
                    _ => None,
                };
                // Seeds and finishes carry their approach costs; the path's
                // cost does not.
                let approach = origin.map_or(0.0, |origin| origin_approach[origin])
                    + destination.map_or(0.0, |destination| destination_approach[destination]);
                let path = OverlayPath {
                    steps: search.steps(finish),
                    origin,
                    destination,
                    cost: finish.cost - approach,
                    duration_s: duration_s + finish.delay_s + arc.duration_s,
                    distance_m: distance_m + arc.distance_m,
                };
                let edge_ids = path
                    .steps
                    .iter()
                    .map(RouteSearchStep::edge_id)
                    .collect::<Vec<_>>();
                // The destination endpoint ends every target's path at once.
                let targets = match target_of(arc) {
                    Some(target) => vec![target],
                    None => (0..target_count).collect(),
                };
                for target in targets {
                    settler.offer(target, finish.cost, edge_ids.clone(), || path.clone());
                }
                continue;
            }
            Item::Label(index) => index,
        };
        let label = search.labels[index as usize].clone();
        if !search.is_current(index) {
            continue;
        }
        let Some(head) = search.info(label.arc).and_then(|info| info.head) else {
            continue;
        };
        for next in search.out_arcs(head) {
            let Some(info) = search.info(next) else {
                continue;
            };
            if !info.cost.is_finite() {
                continue;
            }
            let Some((turn, delay_s)) = search.transition(index, &info) else {
                continue;
            };
            let total = label.cost + turn + info.cost;
            if info.terminal {
                let ArcKey::Patch(arc) = next else {
                    continue;
                };
                finishes.push(OverlayFinish {
                    label: index,
                    arc,
                    origin: label.origin,
                    cost: total,
                    delay_s,
                });
                queue.push(total, Item::Finish(finishes.len() as u32 - 1));
                continue;
            }
            let traversed = info.edge.is_some();
            search.push(
                &mut queue,
                OverlayLabel {
                    arc: next,
                    pred: index,
                    cost: total,
                    duration_s: label.duration_s + delay_s + info.duration_s,
                    distance_m: label.distance_m + info.distance_m,
                    last_edge: info.edge,
                    traversed,
                    origin: label.origin,
                },
            );
        }
    }
    settler.finish()
}

fn overlay_route(
    access: &PackageGraphView,
    request: &LegRequest,
    origins: &[CorrelatedLocation],
    destinations: &[CorrelatedLocation],
    path: OverlayPath,
) -> SearchRoute {
    // Endpoint connectors start or end at the request's own points.
    let endpoint = |kind, fallback: &CorrelatedLocation| {
        let mut location = fallback.clone();
        location.snapped_point = match kind {
            RouteDynamicEndpointKind::Origin => request.origin,
            RouteDynamicEndpointKind::Destination => request.destination,
        };
        location.distance_m = 0.0;
        location.confidence = 1.0;
        location
    };
    let origin = match path.origin {
        Some(index) => origins[index].clone(),
        None => endpoint(RouteDynamicEndpointKind::Origin, &origins[0]),
    };
    let destination = match path.destination {
        Some(index) => destinations[index].clone(),
        None => endpoint(RouteDynamicEndpointKind::Destination, &destinations[0]),
    };
    let edge_ids = path
        .steps
        .iter()
        .map(RouteSearchStep::edge_id)
        .collect::<Vec<_>>();
    let edge_indexes = path
        .steps
        .iter()
        .map(|step| match step {
            RouteSearchStep::GraphEdge { edge_id, .. } => {
                access.raw_edge_index_for_edge_id(*edge_id)
            }
            RouteSearchStep::VirtualConnector { .. } => None,
        })
        .collect();
    let diagnostics = RouteDiagnostics {
        final_path_edge_count: edge_ids.len(),
        ..Default::default()
    };
    SearchRoute {
        origin,
        destination,
        edge_ids,
        edge_indexes,
        steps: path.steps,
        distance_m: path.distance_m,
        duration_s: path.duration_s,
        cost: path.cost,
        diagnostics,
    }
}

// ---------------------------------------------------------------------------
// Isochrone
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct IsochroneLabel {
    raw: usize,
    pred: u32,
    cost: f64,
    duration_s: f64,
}

struct Crossing {
    contour: usize,
    raw: usize,
    start_fraction: f64,
    end_fraction: f64,
    start_metric: f64,
}

fn isochrone(
    records: &Records<'_>,
    request: &RouteIsochroneRequest,
    candidates: &[CorrelatedLocation],
    origin: CorrelatedLocation,
    restriction_contexts: bool,
) -> RouteIsochroneResult {
    let contours = &request.contours;
    let limit = |metric| {
        contours
            .iter()
            .filter(|contour| contour.metric == metric)
            .map(|contour| contour.contour)
            .fold(0.0, f64::max)
    };
    let (max_cost, max_duration) = (
        limit(RouteIsochroneMetric::Cost),
        limit(RouteIsochroneMetric::TimeSeconds),
    );
    let within = |cost: f64, duration_s: f64| {
        (max_cost > 0.0 && cost <= max_cost) || (max_duration > 0.0 && duration_s <= max_duration)
    };
    let mut crossings = Vec::<Crossing>::new();
    let cross = |crossings: &mut Vec<Crossing>,
                 raw: usize,
                 start_fraction: f64,
                 (start_cost, end_cost): (f64, f64),
                 (start_duration, end_duration): (f64, f64)| {
        for (index, contour) in contours.iter().enumerate() {
            let (start, end) = match contour.metric {
                RouteIsochroneMetric::TimeSeconds => (start_duration, end_duration),
                RouteIsochroneMetric::Cost => (start_cost, end_cost),
            };
            if !(start < contour.contour && contour.contour < end) || end - start <= f64::EPSILON {
                continue;
            }
            let ratio = ((contour.contour - start) / (end - start)).clamp(0.0, 1.0);
            crossings.push(Crossing {
                contour: index,
                raw,
                start_fraction,
                end_fraction: start_fraction + (1.0 - start_fraction) * ratio,
                start_metric: start,
            });
        }
    };
    // One label per (edge, restriction context); the context stays empty
    // without `restriction_contexts`.
    let mut labels = Vec::<IsochroneLabel>::new();
    let mut contexts = Vec::<Vec<u64>>::new();
    let mut slot = FxHashMap::<(usize, Vec<u64>), u32>::default();
    let mut reached = FxHashMap::<usize, (f64, f64)>::default();
    let mut reached_order = Vec::<usize>::new();
    let mut queue = Queue::default();
    let context_of = |contexts: &[Vec<u64>], label: &IsochroneLabel| {
        if !restriction_contexts {
            return Vec::new();
        }
        let previous = match label.pred {
            NONE => &[][..],
            pred => &contexts[pred as usize][..],
        };
        records
            .edge(label.raw)
            .map(|edge| records.restriction_context(previous, edge.id()))
            .unwrap_or_default()
    };
    let push = |labels: &mut Vec<IsochroneLabel>,
                contexts: &mut Vec<Vec<u64>>,
                slot: &mut FxHashMap<(usize, Vec<u64>), u32>,
                queue: &mut Queue,
                label: IsochroneLabel| {
        let key = (label.raw, context_of(contexts, &label));
        if let Some(existing) = slot.get(&key)
            && labels[*existing as usize].cost <= label.cost
        {
            return;
        }
        let index = labels.len() as u32;
        contexts.push(key.1.clone());
        slot.insert(key, index);
        labels.push(label);
        queue.push(label.cost, Item::Label(index));
    };
    let mut reach = |raw: usize, cost: f64, duration_s: f64| {
        let entry = reached.entry(raw).or_insert_with(|| {
            reached_order.push(raw);
            (cost, duration_s)
        });
        entry.0 = entry.0.min(cost);
        entry.1 = entry.1.min(duration_s);
    };
    // A farther candidate starts at its approach cost, on both metrics.
    for (candidate, approach) in candidates.iter().zip(endpoint_approach(candidates)) {
        let Some(edge) = records.edge_by_id(candidate.edge_id) else {
            continue;
        };
        let Some((edge_cost, edge_duration)) = edge.costed else {
            continue;
        };
        let remaining = (1.0 - candidate.fraction_along_edge).clamp(0.0, 1.0);
        let (cost, duration_s) = (
            approach + edge_cost * remaining,
            approach + edge_duration * remaining,
        );
        let raw = edge.record.raw_edge_index;
        cross(
            &mut crossings,
            raw,
            candidate.fraction_along_edge,
            (approach, cost),
            (approach, duration_s),
        );
        push(
            &mut labels,
            &mut contexts,
            &mut slot,
            &mut queue,
            IsochroneLabel {
                raw,
                pred: NONE,
                cost,
                duration_s,
            },
        );
        if within(cost, duration_s) {
            reach(raw, cost, duration_s);
        }
    }
    let path_contains = |labels: &[IsochroneLabel], label: u32, raw: usize| {
        let mut current = label;
        while current != NONE {
            let entry = labels[current as usize];
            if entry.raw == raw {
                return true;
            }
            current = entry.pred;
        }
        false
    };
    let path = |labels: &[IsochroneLabel], label: u32| {
        let mut path = Vec::new();
        let mut current = label;
        while current != NONE {
            let entry = labels[current as usize];
            if let Some(edge) = records.edge(entry.raw) {
                path.push(edge.id());
            }
            current = entry.pred;
        }
        path.reverse();
        path
    };
    let (mut visited, mut settled, mut truncated) = (0usize, 0usize, false);
    'search: while let Some((_, item)) = queue.pop() {
        let Item::Label(index) = item else {
            continue;
        };
        let label = labels[index as usize];
        if slot.get(&(label.raw, contexts[index as usize].clone())) != Some(&index) {
            continue;
        }
        settled += 1;
        let Some(edge) = records.edge(label.raw) else {
            continue;
        };
        for raw in records.out_edges(edge.target()) {
            visited += 1;
            if request
                .max_expansion_edge_count
                .is_some_and(|max| visited > max)
            {
                truncated = true;
                break 'search;
            }
            let Some(next) = records.edge(raw) else {
                continue;
            };
            let Some(penalty) =
                records.restriction_penalty(&edge, true, || path(&labels, index), &next)
            else {
                continue;
            };
            let Some((edge_cost, edge_duration)) = next.costed else {
                continue;
            };
            let turn = records.turn(&edge, &next) + penalty;
            if !turn.is_finite() {
                continue;
            }
            let cost = label.cost + edge_cost + turn;
            let entered_s = label.duration_s + records.delay(&edge, &next);
            let duration_s = entered_s + edge_duration;
            let crosses = contours.iter().any(|contour| {
                let (start, end) = match contour.metric {
                    RouteIsochroneMetric::TimeSeconds => (entered_s, duration_s),
                    RouteIsochroneMetric::Cost => (label.cost + turn, cost),
                };
                start < contour.contour && contour.contour < end && end - start > f64::EPSILON
            });
            if crosses && path_contains(&labels, index, raw) {
                continue;
            }
            cross(
                &mut crossings,
                raw,
                0.0,
                (label.cost + turn, cost),
                (entered_s, duration_s),
            );
            if !within(cost, duration_s) {
                continue;
            }
            push(
                &mut labels,
                &mut contexts,
                &mut slot,
                &mut queue,
                IsochroneLabel {
                    raw,
                    pred: index,
                    cost,
                    duration_s,
                },
            );
            reach(raw, cost, duration_s);
        }
    }

    let mut fragments = Vec::with_capacity(crossings.len());
    for crossing in &crossings {
        let Some(record) = records
            .access
            .edge_record_by_raw_edge_index(crossing.raw)
            .map(|record| record.to_view_edge())
        else {
            continue;
        };
        let (Some(start), Some(end)) = (
            point_at_fraction(&record, crossing.start_fraction),
            point_at_fraction(&record, crossing.end_fraction),
        ) else {
            continue;
        };
        let contour = &contours[crossing.contour];
        fragments.push(IsochroneBoundaryCandidate {
            contour: contour.contour,
            metric: contour.metric,
            fragment: crate::types::RouteIsochroneBoundaryFragment {
                edge_id: record.edge.edge_id,
                start,
                end,
                start_fraction: crossing.start_fraction.clamp(0.0, 1.0),
                end_fraction: crossing.end_fraction.clamp(0.0, 1.0),
                start_metric: crossing.start_metric,
                end_metric: contour.contour,
            },
        });
    }
    let buckets = contours
        .iter()
        .map(|contour| RouteIsochroneBucket {
            contour: contour.contour,
            metric: contour.metric,
            edge_ids: reached_order
                .iter()
                .filter(|raw| {
                    let (cost, duration_s) = reached[*raw];
                    match contour.metric {
                        RouteIsochroneMetric::Cost => cost <= contour.contour,
                        RouteIsochroneMetric::TimeSeconds => duration_s <= contour.contour,
                    }
                })
                .filter_map(|raw| records.edge(*raw).map(|edge| edge.id()))
                .collect::<BTreeSet<_>>()
                .into_iter()
                .collect(),
            boundary_fragments: boundary_fragments_for_contour(&fragments, contour),
        })
        .collect();
    RouteIsochroneResult {
        origin,
        buckets,
        visited_edge_count: visited,
        settled_node_count: settled,
        truncated,
    }
}

/// The point at `fraction` of the edge's length along its directed geometry.
fn point_at_fraction(record: &GraphViewEdge, fraction: f64) -> Option<GeoPoint> {
    let geometry = directed_geometry(&record.shared_info, record.edge.traversal_direction);
    if geometry.len() < 2 {
        return None;
    }
    let total_m = geometry
        .windows(2)
        .map(|window| window[0].distance_to(window[1]))
        .sum::<f64>();
    if total_m <= f64::EPSILON {
        return geometry.first().copied();
    }
    let target_m = total_m * fraction.clamp(0.0, 1.0);
    let mut before_m = 0.0;
    for window in geometry.windows(2) {
        let segment_m = window[0].distance_to(window[1]);
        if segment_m <= f64::EPSILON {
            continue;
        }
        if before_m + segment_m >= target_m {
            let ratio = ((target_m - before_m) / segment_m).clamp(0.0, 1.0);
            return Some(GeoPoint::new(
                window[0].lat + (window[1].lat - window[0].lat) * ratio,
                window[0].lng + (window[1].lng - window[0].lng) * ratio,
            ));
        }
        before_m += segment_m;
    }
    geometry.last().copied()
}

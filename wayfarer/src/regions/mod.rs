//! **Prototype** (feature `experimental-regions`; not a supported API): a
//! bounds-guided search across a *virtual seam* inside one package, step 1
//! of EXP-5b in the regions analysis (`analysis/regions.md`, 4.2 and 6.5).
//!
//! A [`SeamRouter`] cuts its package into two regions at the top separator
//! of the stored node order (module `seam`) and routes over both with an
//! edge-based A* whose potential uses only what a bundle of two separately
//! compiled regions would hold: each region's own CCH distances, and the
//! lower bounds between its portal nodes (module `bounds`). That is search
//! (b) of the analysis: potentials stay admissible and consistent, so routes
//! are exact for anything the package search handles exactly, and the
//! parity checks compare them route for route with the package's own search
//! (and, in the tests, the reference oracle). The overlay is exact for the
//! node-level metric the potentials use, so the seam bounds equal the
//! package's (up to f32 rounding): crossing a seam costs no guidance, only
//! the overlay query.
//!
//! Nothing here changes a package, the format or a default code path. The
//! prototype handles plain requests: a profile's stored column with the
//! default costing, no overlays, overrides, SAR options or budget; the
//! search runs forward only. Results name edges by region
//! ([`RegionEdge`]), the form a bundle would need; package edge ids are kept
//! next to them for the comparisons.

mod bounds;
mod seam;
mod search;
#[cfg(test)]
mod tests;

use std::cell::RefCell;
use std::sync::{Arc, Mutex};

use rustc_hash::FxHashMap;

pub use bounds::OverlayStats;
pub use seam::SeamSummary;

use crate::package::SearchGraph;
use crate::package::cch::{PotentialScratch, PotentialStats};
use crate::package::columns::{ColumnHandle, PotentialBase};
use crate::package::search::analysis::{override_bounds, resolve_location};
use crate::package::search::{
    SearchCounters, SearchOutcome, SearchPath, SearchQuery, approach_costs, potential_plan,
    run as run_package,
};
use crate::types::{CorrelatedLocation, MobilityProfile, RouteCostingOptions, RouteOptions};
use crate::{RouteCancellation, RoutingPackage};

use bounds::RegionBounds;
use seam::VirtualSeam;

/// Why a seam search could not run.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum SeamError {
    /// The package stores no node order (or the potentials are off).
    #[error("the package has no CCH potentials")]
    NoPotentials,
    /// The order's elimination tree has no usable top separator.
    #[error("the package cannot be cut into two regions")]
    NotSplittable,
    /// The profile has no stored column bounded by the potentials.
    #[error("the profile has no stored column with potentials")]
    UnsupportedProfile,
    /// A location's edge is not in the package.
    #[error("edge {0} is not in the package")]
    MissingEdge(u64),
}

/// An edge named by its region: the region's index and the edge's index
/// among the region's edges (an edge belongs to its source node's region).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct RegionEdge {
    pub region: u8,
    pub edge: u32,
}

/// What guides a [`SeamRouter::route`] search.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeamBound {
    /// The regions' own CCHs and the portal overlay (the prototype).
    Regions,
    /// The package's CCH, as the package search uses it: the same A*, for
    /// comparing the bounds.
    Package,
}

/// A route found by a seam search or by the package search.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct SeamPath {
    /// Without approach costs (`LegResult::cost`).
    pub cost: f64,
    /// With the approach costs of the chosen candidates, which the searches
    /// minimise: compare these.
    pub total_cost: f64,
    pub distance_m: f64,
    pub duration_s: f64,
    /// Package edge ids (`LegResult::edge_ids`), destination edge last.
    pub edge_ids: Vec<u64>,
    /// The same edges, region-qualified.
    pub region_edges: Vec<RegionEdge>,
    pub origin_index: usize,
    pub destination_index: usize,
    /// Consecutive edges in different regions.
    pub portal_crossings: usize,
    pub settled_edge_count: usize,
    pub visited_edge_count: usize,
    /// The overlay and this query's part of it ([`SeamBound::Regions`]).
    pub overlay: Option<OverlayStats>,
}

/// A package cut into two regions, and searches across them.
pub struct SeamRouter {
    package: Arc<RoutingPackage>,
    seam: VirtualSeam,
    /// By stored column: the regions' metrics and overlay.
    bounds: Mutex<FxHashMap<usize, Arc<RegionBounds>>>,
}

impl SeamRouter {
    /// Cuts `package` (preparing its potentials first).
    pub fn new(package: Arc<RoutingPackage>) -> Result<Self, SeamError> {
        let seam = {
            let view = package.view();
            let potentials = view.cch.ok_or(SeamError::NoPotentials)?;
            VirtualSeam::cut(&potentials.topology, &view)?
        };
        Ok(Self {
            package,
            seam,
            bounds: Mutex::new(FxHashMap::default()),
        })
    }

    pub fn package(&self) -> &Arc<RoutingPackage> {
        &self.package
    }

    pub fn summary(&self) -> &SeamSummary {
        self.seam.summary()
    }

    /// The package's potentials (built by [`Self::new`]).
    pub fn potential_stats(&self) -> Option<PotentialStats> {
        self.package.prepare_potentials()
    }

    /// Package nodes of the top separator, top down.
    pub fn separator_nodes(&self) -> &[u32] {
        self.seam.separator()
    }

    /// The stored position of package node `node` (for harnesses picking
    /// points).
    pub fn node_point(&self, node: u32) -> crate::GeoPoint {
        let [x, y, z] = self.package.view().xyz(node);
        crate::GeoPoint::new(z.asin().to_degrees(), y.atan2(x).to_degrees())
    }

    /// The region of package node `node`.
    pub fn region_of_node(&self, node: u32) -> u8 {
        self.seam.owner(node)
    }

    /// The region-qualified id of package edge index `edge`.
    pub fn region_edge(&self, edge: u32) -> RegionEdge {
        self.seam.region_edge(edge)
    }

    /// The package edge index of a region-qualified id.
    pub fn package_edge(&self, edge: RegionEdge) -> Option<u32> {
        self.seam.package_edge(edge)
    }

    /// The overlay of `profile`'s stored column, built on first use.
    pub fn overlay(&self, profile: MobilityProfile) -> Result<OverlayStats, SeamError> {
        self.with_query(profile, &[], &[], |prepared| {
            Ok(prepared.bounds.stats.clone())
        })
    }

    /// The best route from one of `origins` to one of `destinations` (as a
    /// route correlates them, e.g. [`crate::RoutingEngine::snap`]) with the
    /// forward A* across the seam guided by `bound`; `None`: no path.
    pub fn route(
        &self,
        profile: MobilityProfile,
        origins: &[CorrelatedLocation],
        destinations: &[CorrelatedLocation],
        bound: SeamBound,
    ) -> Result<Option<SeamPath>, SeamError> {
        self.with_query(profile, origins, destinations, |prepared| {
            let g = prepared.view;
            let query = &prepared.query;
            let targets = query
                .destination_edges
                .iter()
                .map(|edge| g.edge_source(*edge as usize))
                .collect::<Vec<_>>();
            let scratch: [RefCell<PotentialScratch>; 2] = Default::default();
            let (path, counters, overlay) = match bound {
                SeamBound::Regions => {
                    let (potential, stats) = prepared.bounds.potential(
                        prepared.topology,
                        &self.seam,
                        &targets,
                        prepared.factor,
                        &scratch,
                    );
                    let (path, counters) = search::run(g, query, |node| potential.get(node));
                    (path, counters, Some(stats))
                }
                SeamBound::Package => {
                    let plan = potential_plan(g, &query.column, query.cost_overrides, false)
                        .ok_or(SeamError::UnsupportedProfile)?;
                    let potential = plan.potential(&targets, false, &[], &scratch[0], &scratch[1]);
                    let (path, counters) = search::run(g, query, |node| potential.get(node));
                    (path, counters, None)
                }
            };
            Ok(path.map(|path| {
                self.seam_path(g, query, path, counters.settled, counters.visited, overlay)
            }))
        })
    }

    /// The package's own search on the same query (bidirectional, walking
    /// pieces, as a plain route runs it), for the parity checks.
    pub fn package_route(
        &self,
        profile: MobilityProfile,
        origins: &[CorrelatedLocation],
        destinations: &[CorrelatedLocation],
    ) -> Result<Option<SeamPath>, SeamError> {
        self.with_query(profile, origins, destinations, |prepared| {
            let mut counters = SearchCounters::default();
            let mut state = self.package.take_state();
            let outcome = run_package(prepared.view, &mut state, &prepared.query, &mut counters);
            self.package.return_state(state);
            Ok(match outcome {
                SearchOutcome::Found(path) => Some(self.seam_path(
                    prepared.view,
                    &prepared.query,
                    path,
                    counters.settled_edge_count,
                    counters.visited_edge_count,
                    None,
                )),
                _ => None,
            })
        })
    }

    fn seam_path(
        &self,
        g: SearchGraph<'_>,
        query: &SearchQuery<'_>,
        path: SearchPath,
        settled_edge_count: usize,
        visited_edge_count: usize,
        overlay: Option<OverlayStats>,
    ) -> SeamPath {
        let region_edges = path
            .edges
            .iter()
            .map(|edge| self.seam.region_edge(*edge))
            .collect::<Vec<_>>();
        let portal_crossings = region_edges
            .windows(2)
            .filter(|pair| pair[0].region != pair[1].region)
            .count();
        let total_cost = path.cost
            + approach_costs(query.origins)[path.origin_index]
            + approach_costs(query.destinations)[path.destination_index];
        SeamPath {
            cost: path.cost,
            total_cost,
            distance_m: path.distance_m,
            duration_s: path.duration_s,
            edge_ids: path
                .edges
                .iter()
                .map(|edge| g.edge_id(*edge as usize))
                .collect(),
            region_edges,
            origin_index: path.origin_index,
            destination_index: path.destination_index,
            portal_crossings,
            settled_edge_count,
            visited_edge_count,
            overlay,
        }
    }

    /// Runs `work` on the plain query of `profile` between the locations.
    fn with_query<T>(
        &self,
        profile: MobilityProfile,
        origins: &[CorrelatedLocation],
        destinations: &[CorrelatedLocation],
        work: impl FnOnce(&Prepared<'_>) -> Result<T, SeamError>,
    ) -> Result<T, SeamError> {
        let graph = &self.package;
        let options = RouteOptions::default();
        let handle = graph.request_column_sar(profile, &RouteCostingOptions::default(), true, None);
        let view = graph
            .view()
            .with_transitions(crate::transitions::Transitions::of(
                &options,
                profile,
                graph.profile_bit(profile),
            ));
        let topology = &view.cch.ok_or(SeamError::NoPotentials)?.topology;
        let column = handle.column(&view);
        let stored = match (&handle, column.potential) {
            (ColumnHandle::Stored { .. }, Some((PotentialBase::Stored(stored), _))) => Some(stored),
            _ => None,
        };
        let stored = stored.ok_or(SeamError::UnsupportedProfile)?;
        let resolve = |location: &CorrelatedLocation| {
            resolve_location(view, location).ok_or(SeamError::MissingEdge(location.edge_id))
        };
        let origin_edges = origins.iter().map(resolve).collect::<Result<Vec<_>, _>>()?;
        let destination_edges = destinations
            .iter()
            .map(resolve)
            .collect::<Result<Vec<_>, _>>()?;
        let overrides = FxHashMap::default();
        let plan = potential_plan(view, &column, &overrides, false)
            .ok_or(SeamError::UnsupportedProfile)?;
        let bounds = {
            let cached = self
                .bounds
                .lock()
                .ok()
                .and_then(|cache| cache.get(&stored).cloned());
            match cached {
                Some(bounds) => bounds,
                None => {
                    let built = Arc::new(RegionBounds::build(
                        topology,
                        &view,
                        &self.seam,
                        plan.metric.costs(),
                    ));
                    if let Ok(mut cache) = self.bounds.lock() {
                        cache.entry(stored).or_insert_with(|| built.clone());
                    }
                    built
                }
            }
        };
        let limits = override_bounds(view, column, &overrides, &Default::default());
        let cancellation = RouteCancellation::default();
        let query = SearchQuery {
            column,
            profile_bit: view.profile_bit(profile),
            heuristic_cost_per_m: limits.heuristic_cost_per_m,
            origins,
            destinations,
            origin_edges: &origin_edges,
            destination_edges: &destination_edges,
            cost_overrides: &overrides,
            max_visited_edge_count: None,
            cancellation: &cancellation,
            gates: &limits.gates,
            bidirectional: true,
            restriction_penalty: None,
            pieces: handle.piece_kinds(&view),
        };
        work(&Prepared {
            view,
            topology,
            query,
            bounds: &bounds,
            factor: plan.factor,
        })
    }
}

/// A plain query and what its seam search needs.
struct Prepared<'a> {
    view: SearchGraph<'a>,
    topology: &'a crate::package::cch::CchTopology,
    query: SearchQuery<'a>,
    bounds: &'a RegionBounds,
    /// Scales the column's metric to the query's costs.
    factor: f64,
}

//! Portals: off-network access priced by an incident surface
//! (`docs/design/portals.md`, feature `terrain`).
//!
//! A [`TerrainPricer`] is an [`AccessPricer`]. Put it on a request's
//! `offNetwork` options and each endpoint that lies on open ground is priced by
//! a least-cost search over the surface instead of X1's straight line:
//!
//! 1. One bounded search over the surface per endpoint, from an origin and
//!    towards a destination, so that each move costs what it costs walked in
//!    the direction of travel.
//! 2. For each directed edge the correlator found within the radius, every
//!    lattice post the edge's shape crosses is a possible portal: the shape
//!    point closest to the post's centre. A post is a portal only where the
//!    profile may leave the road there (see [`TerrainPricer::exit_allowed`]).
//! 3. Each edge keeps the portal with the least key, the access time plus the
//!    switch cost plus the rest of the edge, and the edges are ranked by that
//!    key. The endpoint keeps the first `maxCandidates`, by default all of
//!    them ([`crate::access::ACCESS_PRICED_DEFAULT_CANDIDATES`]).
//!
//! Each kept stretch reports its surface, its confidence and what the
//! endpoint cut (`RouteAccessPricing`, on the leg's `offNetwork` summary).
//!
//! The searches seed each kept candidate with its priced access, so routes,
//! matrices, isochrones and trips are exact over the candidate set, as X1's
//! are. An endpoint within the snap radius of a road it may use, or outside
//! the surface, keeps X1's straight-line candidates.

use std::sync::{Arc, Mutex};

use wayfarer_package::{GeoPoint, MobilityProfile, RoadType, directed_geometry, haversine_m};
use wayfarer_terrain::lattice::{BlockId, TICKS_PER_DEG, div_round_i64, ticks_of_deg};
use wayfarer_terrain::search::{Graph, Grid, Search, Toward, path, path_to, route_conf};
use wayfarer_terrain::surface::Surface;
use wayfarer_terrain::terrain::{STEEP_NONE, Terrain};
use wayfarer_terrain::{fuse, lines};

use crate::CorrelatedLocation;
use crate::access::{
    AccessPricer, AccessQuery, AccessRole, CandidateAccess, PricedAccess, RouteAccessConfidence,
    RouteAccessPricing, RouteAccessTruncation,
};
use crate::graph_view::GraphViewEdge;

/// Line bits no exit may cross on its portal post: rivers, streams, rails,
/// fences, cliffs and a team's block.
const EXIT_BARRIERS: u16 =
    lines::RIVER | lines::STREAM | lines::RAIL | lines::FENCE | lines::CLIFF | lines::TEAM_BLOCK;

/// Units of a fraction along an edge in a portal's key: 2^32 per edge.
const FRACTION_ONE: f64 = 4_294_967_296.0;

/// How one class of profile (walkers or vehicles) leaves the road.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PortalClass {
    /// The longest access stretch searched, in seconds.
    pub max_access_s: f64,
    /// Added once per stretch, in time and cost: leaving or joining the road.
    pub switch_cost_s: f64,
    /// The steepest steepness-max class a portal post may have.
    pub max_steep: u8,
    /// The closure risk (1/255) at or above which a post is no portal.
    pub max_risk: u8,
    /// The pace on the road, metres per second, for the rest of an edge in a
    /// portal's key when the edge states no speed. Correlation runs before
    /// costing, so the key reads the edge's time at this pace or its own speed
    /// in place of its cost: it only chooses the portal on each edge and ranks
    /// the edges, and the searches then price the kept candidates exactly.
    pub road_pace_mps: f64,
}

impl PortalClass {
    /// Walkers: 30 minutes of access, 10 s to switch, steepness class 9.
    pub const WALKER: PortalClass = PortalClass {
        max_access_s: 1_800.0,
        switch_cost_s: 10.0,
        max_steep: 9,
        max_risk: 128,
        road_pace_mps: 1.4,
    };
    /// Vehicles: 10 minutes of access, 45 s to switch, steepness class 5.
    pub const VEHICLE: PortalClass = PortalClass {
        max_access_s: 600.0,
        switch_cost_s: 45.0,
        max_steep: 5,
        max_risk: 128,
        road_pace_mps: 30.0 / 3.6,
    };
}

/// Prices off-network access by least-cost searches over an incident
/// surface (see the [module docs](self)).
pub struct TerrainPricer {
    terrain: Arc<Terrain>,
    surface: Arc<Surface>,
    grid: Arc<Grid>,
    profiles: Vec<(MobilityProfile, usize)>,
    /// Walkers ([`MobilityProfile::OnFoot`], [`MobilityProfile::Legged`]).
    pub walker: PortalClass,
    /// Every other profile.
    pub vehicle: PortalClass,
    /// An endpoint this close to a road its profile may use (metres) keeps
    /// X1's candidates: it is on the road, not on open ground.
    pub snap_radius_m: f64,
    /// Price every endpoint on the surface by the terrain, however close it
    /// is to a road (SAR's last known position).
    pub force: bool,
    searches: Mutex<Vec<Search>>,
}

impl std::fmt::Debug for TerrainPricer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TerrainPricer")
            .field(
                "surface",
                &wayfarer_terrain::hex(&self.surface.content_hash()),
            )
            .field("profiles", &self.profiles)
            .field("walker", &self.walker)
            .field("vehicle", &self.vehicle)
            .field("snap_radius_m", &self.snap_radius_m)
            .field("force", &self.force)
            .finish()
    }
}

/// A stretch's drawn line, its length on the lattice (metres) and its steps
/// (from cell, move).
struct Stretch {
    line: Vec<GeoPoint>,
    metres: f64,
    steps: Vec<(usize, usize)>,
}

/// What a portal on one edge costs and where it is.
struct Portal {
    /// Access plus switch plus the rest of the edge, µs.
    key: u64,
    cell: usize,
    fraction: f64,
    snapped: GeoPoint,
    /// Access time, µs, with the offsets to the requested point and the join.
    access_us: u64,
}

impl TerrainPricer {
    /// A pricer over `surface`, built on `terrain` (the incident's terrain,
    /// cut to its AOI), searching `grid` (the surface's grid). Walkers use the
    /// surface's profile named `walker` and vehicles the one named `vehicle`,
    /// where the surface has them; [`TerrainPricer::with_profile`] maps others.
    pub fn new(terrain: Arc<Terrain>, surface: Arc<Surface>, grid: Arc<Grid>) -> TerrainPricer {
        let walker = grid.profile("walker");
        let vehicle = grid.profile("vehicle");
        let mut profiles = Vec::new();
        for profile in [MobilityProfile::OnFoot, MobilityProfile::Legged] {
            if let Some(index) = walker {
                profiles.push((profile, index));
            }
        }
        for profile in [
            MobilityProfile::Default,
            MobilityProfile::Wheeled,
            MobilityProfile::Tracked,
            MobilityProfile::Emergency,
        ] {
            if let Some(index) = vehicle {
                profiles.push((profile, index));
            }
        }
        TerrainPricer {
            terrain,
            surface,
            grid,
            profiles,
            walker: PortalClass::WALKER,
            vehicle: PortalClass::VEHICLE,
            snap_radius_m: 25.0,
            force: false,
            searches: Mutex::new(Vec::new()),
        }
    }

    /// A pricer over an incident's last surface.
    pub fn from_incident(
        incident: &wayfarer_terrain::incident::Incident,
    ) -> wayfarer_terrain::Result<TerrainPricer> {
        let surface = incident.surface().ok_or_else(|| {
            wayfarer_terrain::Error::invalid("no surface yet: call surface_at first")
        })?;
        let grid = incident.grid()?;
        Ok(TerrainPricer::new(
            Arc::new(incident.terrain().clone()),
            surface,
            grid,
        ))
    }

    /// Prices `profile` with the surface's profile `index`, or not at all.
    pub fn with_profile(mut self, profile: MobilityProfile, index: Option<usize>) -> Self {
        self.profiles.retain(|(known, _)| *known != profile);
        if let Some(index) = index.filter(|&index| index < self.grid.rules.len()) {
            self.profiles.push((profile, index));
        }
        self
    }

    /// The surface the pricer searches.
    pub fn surface(&self) -> &Arc<Surface> {
        &self.surface
    }

    fn class(&self, profile: MobilityProfile) -> &PortalClass {
        if is_walker(profile) {
            &self.walker
        } else {
            &self.vehicle
        }
    }

    /// The cell of the post nearest `point`, if the grid holds it.
    fn cell_of(&self, point: GeoPoint) -> Option<usize> {
        let s = self.grid.spacing;
        let (lat, lng) = (ticks_of_deg(point.lat)?, ticks_of_deg(point.lng)?);
        self.grid
            .cell_of(div_round_i64(lat, s), div_round_i64(lng, s))
    }

    /// The position of a cell's post.
    fn point_of(&self, cell: usize) -> GeoPoint {
        let (i, j) = self.grid.post_of(cell);
        let s = self.grid.spacing as f64 / TICKS_PER_DEG as f64;
        GeoPoint::new(i as f64 * s, j as f64 * s)
    }

    /// Whether `profile` may leave or join the road at post (i, j), cell
    /// `cell` of the grid: the surface lets it stand there (a time per metre,
    /// closure risk below its class's limit), no barrier line or team block
    /// lies on the post, and the post is no steeper than the class allows.
    pub fn exit_allowed(&self, prof: usize, class: &PortalClass, cell: usize) -> bool {
        if self.grid.t[prof][cell] == 0 || self.grid.risk[prof][cell] >= class.max_risk {
            return false;
        }
        let (i, j) = self.grid.post_of(cell);
        let (id, q) = BlockId::of_post(i, j);
        let Some(k) = self.terrain.block_index(id) else {
            return false;
        };
        let steep = self.terrain.steep(k)[q];
        self.terrain.lines(k)[q] & EXIT_BARRIERS == 0
            && (steep == STEEP_NONE || steep <= class.max_steep)
    }

    /// Microseconds to cross `metres` at a cell's own time per metre.
    fn offset_us(&self, prof: usize, cell: usize, metres: f64) -> u64 {
        let per_m = self.grid.t[prof][cell] as f64 * self.grid.rules[prof].p.unit_us as f64;
        (metres * per_m).round() as u64
    }

    /// The best portal on one directed edge, if it has any the search reached.
    #[allow(clippy::too_many_arguments)]
    fn portal_on(
        &self,
        search: &Search,
        prof: usize,
        class: &PortalClass,
        walker: bool,
        role: AccessRole,
        point: GeoPoint,
        at: usize,
        edge: &GraphViewEdge,
    ) -> Option<Portal> {
        let shape = directed_geometry(&edge.shared_info, edge.edge.traversal_direction);
        if shape.len() < 2 {
            return None;
        }
        let s = self.grid.spacing;
        let ticks = |p: GeoPoint| Some((ticks_of_deg(p.lat)?, ticks_of_deg(p.lng)?));
        let mut posts = Vec::new();
        for pair in shape.windows(2) {
            let (a, b) = (ticks(pair[0])?, ticks(pair[1])?);
            fuse::supercover_ticks(a, b, s, &mut |i, j| posts.push((i, j)));
        }
        posts.sort_unstable();
        posts.dedup();
        let edge_us = edge_time_us(edge, class, walker);
        let switch_us = (class.switch_cost_s * 1e6).round() as u64;
        let mut best: Option<Portal> = None;
        for (i, j) in posts {
            let Some(cell) = self.grid.cell_of(i, j) else {
                continue;
            };
            let reached = search.d(cell);
            if reached == u64::MAX || !self.exit_allowed(prof, class, cell) {
                continue;
            }
            let centre = self.point_of(cell);
            let Some(snap) = crate::correlator::snap_to_geometry(centre, &shape) else {
                continue;
            };
            let units = (snap.fraction.clamp(0.0, 1.0) * FRACTION_ONE).round();
            let fraction = units / FRACTION_ONE;
            let rest = match role {
                AccessRole::Origin => 1.0 - fraction,
                AccessRole::Destination => fraction,
            };
            let access_us = reached
                + self.offset_us(prof, at, haversine_m(point, self.point_of(at)))
                + self.offset_us(prof, cell, snap.distance_m);
            let key = access_us + switch_us + (rest * edge_us as f64).round() as u64;
            let better = best.as_ref().is_none_or(|current| {
                (key, cell, fraction.to_bits())
                    < (current.key, current.cell, current.fraction.to_bits())
            });
            if better {
                best = Some(Portal {
                    key,
                    cell,
                    fraction,
                    snapped: snap.point,
                    access_us,
                });
            }
        }
        best
    }

    /// The drawn path of a stretch: from the requested point's post to the
    /// portal's and on to the join (origin), or back (destination).
    fn stretch_path(
        &self,
        graph: &Graph,
        search: &Search,
        role: AccessRole,
        at: usize,
        portal: &Portal,
    ) -> Option<Stretch> {
        let steps = match role {
            AccessRole::Origin => path(graph, search, at, portal.cell)?,
            AccessRole::Destination => path_to(graph, search, portal.cell, at)?,
        };
        let w = self.grid.w;
        let metres: f64 = steps
            .iter()
            .map(|&(cell, m)| graph.move_len(cell / w, m) as f64 / 1000.0)
            .sum();
        let mut cells: Vec<usize> = steps.iter().map(|&(cell, _)| cell).collect();
        cells.push(match role {
            AccessRole::Origin => portal.cell,
            AccessRole::Destination => at,
        });
        let mut line: Vec<GeoPoint> = cells.into_iter().map(|cell| self.point_of(cell)).collect();
        match role {
            AccessRole::Origin => line.push(portal.snapped),
            AccessRole::Destination => line.insert(0, portal.snapped),
        }
        Some(Stretch {
            line,
            metres,
            steps,
        })
    }

    fn take_search(&self) -> Search {
        let n = self.grid.h * self.grid.w;
        self.searches
            .lock()
            .ok()
            .and_then(|mut pool| pool.pop())
            .unwrap_or_else(|| Search::new(n))
    }

    fn give_search(&self, search: Search) {
        if let Ok(mut pool) = self.searches.lock()
            && pool.len() < 8
        {
            pool.push(search);
        }
    }
}

impl AccessPricer for TerrainPricer {
    fn price(&self, query: AccessQuery<'_>) -> Vec<CorrelatedLocation> {
        let AccessQuery {
            point,
            profile,
            role,
            policy,
            candidates,
            view,
        } = query;
        let Some(prof) = self
            .profiles
            .iter()
            .find(|(known, _)| *known == profile)
            .map(|&(_, index)| index)
        else {
            return candidates;
        };
        let class = self.class(profile);
        let walker = is_walker(profile);
        if !self.force
            && candidates
                .iter()
                .any(|candidate| candidate.distance_m <= self.snap_radius_m)
        {
            return candidates;
        }
        let Some(at) = self.cell_of(point) else {
            return candidates;
        };
        if self.grid.t[prof][at] == 0 {
            return candidates;
        }
        let graph = Graph::new(&self.grid, prof);
        let mut search = self.take_search();
        let toward = match role {
            AccessRole::Origin => Toward::From,
            AccessRole::Destination => Toward::To,
        };
        let limit_us = (class.max_access_s * 1e6).round() as u64;
        search.run_field(&graph, at, toward, limit_us);
        let surface_hash = wayfarer_terrain::hex(&self.surface.content_hash());
        let mut priced = Vec::new();
        for candidate in candidates {
            let Some(edge) = view.edge_by_id(candidate.edge_id) else {
                continue;
            };
            if !edge_allows_exit(&edge, walker) {
                continue;
            }
            let Some(portal) = self.portal_on(&search, prof, class, walker, role, point, at, &edge)
            else {
                continue;
            };
            let Some(Stretch {
                line,
                metres,
                steps,
            }) = self.stretch_path(&graph, &search, role, at, &portal)
            else {
                continue;
            };
            let conf = route_conf(&graph, prof, &steps, portal.access_us);
            let ends = haversine_m(point, self.point_of(at))
                + haversine_m(self.point_of(portal.cell), portal.snapped);
            let duration_s = portal.access_us as f64 / 1e6 + class.switch_cost_s;
            let mut location = candidate;
            location.snapped_point = portal.snapped;
            location.fraction_along_edge = portal.fraction;
            location.distance_m = haversine_m(point, portal.snapped);
            location.access = Some(CandidateAccess {
                point,
                pace_mps: policy.pace_mps,
                cost_factor: 1.0,
                priced: Some(Arc::new(PricedAccess {
                    cost: duration_s,
                    duration_s,
                    distance_m: metres + ends,
                    path: line,
                    pricing: Some(RouteAccessPricing {
                        source: "terrain".into(),
                        surface_hash: Some(surface_hash.clone()),
                        confidence: Some(RouteAccessConfidence {
                            mean_certainty: conf.mean_cert,
                            min_certainty: conf.min_cert,
                            layer_permille: conf.layer_permille,
                            team_permille: conf.team_permille,
                            max_risk: conf.max_risk,
                            risky_permille: conf.risky_permille,
                            at_edge: conf.at_edge,
                        }),
                        truncated: None,
                    }),
                })),
            });
            priced.push((portal.key, location));
        }
        self.give_search(search);
        priced.sort_by_key(|(key, location)| (*key, location.edge_id));
        let kept = policy.candidate_limit.min(priced.len());
        let truncated = (kept < priced.len() && kept > 0).then(|| RouteAccessTruncation {
            cut: priced.len() - kept,
            best_cut_key_s: priced[kept].0 as f64 / 1e6,
            worst_kept_key_s: priced[kept - 1].0 as f64 / 1e6,
        });
        priced.truncate(kept);
        priced
            .into_iter()
            .map(|(_, mut location)| {
                if let Some(truncated) = &truncated
                    && let Some(access) = location.access.as_mut()
                    && let Some(priced) = access.priced.as_mut()
                {
                    let priced = Arc::make_mut(priced);
                    if let Some(pricing) = priced.pricing.as_mut() {
                        pricing.truncated = Some(truncated.clone());
                    }
                }
                location
            })
            .collect()
    }
}

fn is_walker(profile: MobilityProfile) -> bool {
    matches!(profile, MobilityProfile::OnFoot | MobilityProfile::Legged)
}

/// Whether a profile may leave or join the road anywhere on `edge`: never
/// from a motorway, motorroad, bridge, tunnel, ferry or a layer other than 0,
/// and vehicles only from tracks, unpaved roads and minor roads.
pub fn edge_allows_exit(edge: &GraphViewEdge, walker: bool) -> bool {
    let info = &edge.shared_info;
    let highway = info.highway.as_deref().unwrap_or("");
    if info.road_type == RoadType::Motorway
        || highway.starts_with("motorway")
        || info.is_motorroad
        || info.is_bridge
        || info.is_tunnel
        || info.is_ferry
        || info.layer.unwrap_or(0) != 0
    {
        return false;
    }
    walker
        || matches!(
            info.road_type,
            RoadType::Residential | RoadType::Track | RoadType::Path | RoadType::Other
        )
        || is_unpaved(info.surface.as_deref(), info.tracktype.as_deref())
}

fn is_unpaved(surface: Option<&str>, tracktype: Option<&str>) -> bool {
    const UNPAVED: [&str; 14] = [
        "unpaved",
        "compacted",
        "gravel",
        "fine_gravel",
        "pebblestone",
        "rock",
        "ground",
        "dirt",
        "earth",
        "grass",
        "mud",
        "sand",
        "woodchips",
        "grass_paver",
    ];
    surface.is_some_and(|surface| UNPAVED.contains(&surface))
        || tracktype.is_some_and(|grade| grade != "grade1")
}

/// The edge's time in µs: a vehicle's at the edge's own speed where it states
/// one, a walker's (and a vehicle's otherwise) at the class's road pace.
fn edge_time_us(edge: &GraphViewEdge, class: &PortalClass, walker: bool) -> u64 {
    let pace = match edge.edge.inferred_speed_kph {
        Some(kph) if kph > 0 && !walker => kph as f64 / 3.6,
        _ => class.road_pace_mps,
    };
    (edge.edge.distance_m as f64 / pace * 1e6).round() as u64
}

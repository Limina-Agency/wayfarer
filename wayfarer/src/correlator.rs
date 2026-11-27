use std::cmp::Ordering;
use std::collections::HashMap;

use wayfarer_package::{RoadType, SharedEdgeInfo};

use crate::graph_view::PackageGraphView;
use crate::package::request::{CorrelationEdge, ReachCounter};
use crate::types::{
    CorrelatedLocation, CorrelationMode, CorrelationPolicy, CorrelationRejectReason,
    CorrelationScoreBreakdown, CorrelationSide, GeoPoint, MobilityProfile, PreferredRoadSide,
    RejectedCorrelationCandidate, RouteEndpointOptions, RouteError,
};

const MAX_CONFIDENCE: f64 = 1.0;
const MIN_CONFIDENCE: f64 = 0.0;
const DEFAULT_REACH_EDGE_LIMIT: usize = 64;
/// The radii `search_expanding` widens to, in turn, past the policy's.
pub const EXPANDED_CORRELATION_DISTANCES_M: [u32; 2] = [300, 1_000];
/// The smallest component (nodes) `search_expanding` takes for connected to
/// the network; smaller ones are islands.
pub const EXPANDED_CORRELATION_MIN_COMPONENT_SIZE: usize = 50;

pub struct Correlator<'a> {
    access: &'a PackageGraphView,
}

impl<'a> Correlator<'a> {
    pub fn new(access: &'a PackageGraphView) -> Self {
        Self { access }
    }

    pub fn candidates(
        &self,
        point: GeoPoint,
        profile: MobilityProfile,
        policy: &CorrelationPolicy,
    ) -> Result<Vec<CorrelatedLocation>, RouteError> {
        Ok(self
            .search(point, profile, policy, &RouteEndpointOptions::default())
            .candidates)
    }

    /// The candidates for `point`, best first, and every rejected edge in the
    /// radius with its reason.
    ///
    /// Two phases. Every edge in the radius gets the checks that need no edge
    /// record (access, snap distance, component, heading, layer, level and,
    /// when a minimum asks for it, reach), so every rejection keeps its
    /// diagnostics. Then only the survivors that can still make the candidate
    /// limit get a record, a reach count and a score (`ranked_survivors`); the
    /// others only add their road group key to the same-road counts. The
    /// result is the one scoring every survivor gives.
    pub fn search(
        &self,
        point: GeoPoint,
        profile: MobilityProfile,
        policy: &CorrelationPolicy,
        endpoint_options: &RouteEndpointOptions,
    ) -> CandidateSearch {
        let request_view = self.access.request();
        let file = self.access.routing_package().file();
        let sections = file.view();
        let sar = policy.sar_open.map(crate::sar::SarOpenEdges::relaxations);
        let edges = request_view.correlation_edges_sar(point, policy.max_distance_m, profile, sar);
        let mut reach = ReachCounter::new(file, profile, DEFAULT_REACH_EDGE_LIMIT).with_sar(sar);
        let mut reach_counts = |edge: u32| -> ReachCounts {
            // The routing package answers the reach question without
            // materializing edge records (min(reachable, cap) is order-free).
            let index = edge as usize;
            match reach
                .count(index, true)
                .and_then(|outbound| Some((outbound, reach.count(index, false)?)))
            {
                Some((outbound, inbound)) => (Some(outbound), Some(inbound)),
                None => (
                    self.access.outbound_reach_edge_count(
                        request_view.node_id(request_view.edge_target(edge)),
                        profile,
                        DEFAULT_REACH_EDGE_LIMIT,
                    ),
                    self.access.inbound_reach_edge_count(
                        request_view.node_id(request_view.edge_source(edge)),
                        profile,
                        DEFAULT_REACH_EDGE_LIMIT,
                    ),
                ),
            }
        };

        // Phase 1: the record-free checks, in edge id order.
        let max_distance_m = f64::from(policy.max_distance_m);
        let checks_reach = endpoint_options.min_outbound_reach_edge_count.is_some()
            || endpoint_options.min_inbound_reach_edge_count.is_some();
        let mut rejected_candidates = Vec::new();
        let mut survivors = Vec::new();
        for edge in edges {
            let mut reject = |reason, distance_m, heading_delta_deg, reach: ReachCounts| {
                rejected_candidates.push(rejected_candidate(
                    Some(edge.edge_id),
                    reason,
                    distance_m,
                    heading_delta_deg,
                    edge.component_size,
                    reach.0,
                    reach.1,
                    edge.layer,
                    edge.level.as_deref().map(str::to_string),
                ));
            };
            if !edge.accessible {
                reject(
                    CorrelationRejectReason::InaccessibleProfile,
                    None,
                    None,
                    (None, None),
                );
                continue;
            }
            let Some(snap) = snap_to_geometry(point, &edge.geometry) else {
                reject(
                    CorrelationRejectReason::SnapUnavailable,
                    None,
                    None,
                    (None, None),
                );
                continue;
            };
            let distance_m = Some(snap.distance_m);
            if snap.distance_m > max_distance_m {
                reject(
                    CorrelationRejectReason::BeyondMaxDistance,
                    distance_m,
                    None,
                    (None, None),
                );
                continue;
            }
            if endpoint_options
                .min_component_size
                .is_some_and(|minimum| edge.component_size.unwrap_or_default() < minimum)
            {
                reject(
                    CorrelationRejectReason::BelowMinComponentSize,
                    distance_m,
                    None,
                    (None, None),
                );
                continue;
            }

            // The edge's direction where the point snaps, not where it
            // starts: the two differ on any curved edge.
            let heading_deg = snap.bearing_deg.or_else(|| {
                let heading = sections.heading_start[edge.edge() as usize];
                (heading != crate::package::NO_HEADING).then_some(heading)
            });
            let heading_delta_deg = endpoint_options
                .heading_deg
                .and_then(|heading| heading_deg.map(|candidate| heading_delta(heading, candidate)));
            if endpoint_options
                .heading_tolerance_deg
                .zip(heading_delta_deg)
                .is_some_and(|(tolerance, delta)| delta > tolerance.min(180))
            {
                reject(
                    CorrelationRejectReason::HeadingOutsideTolerance,
                    distance_m,
                    heading_delta_deg,
                    (None, None),
                );
                continue;
            }
            if endpoint_options.require_layer_match
                && endpoint_options
                    .preferred_layer
                    .is_some_and(|layer| edge.layer != Some(layer))
            {
                reject(
                    CorrelationRejectReason::LayerMismatch,
                    distance_m,
                    heading_delta_deg,
                    (None, None),
                );
                continue;
            }
            if endpoint_options.require_level_match
                && endpoint_options
                    .preferred_level
                    .as_ref()
                    .is_some_and(|level| {
                        edge.level
                            .as_deref()
                            .is_none_or(|candidate| !level_matches(level, candidate))
                    })
            {
                reject(
                    CorrelationRejectReason::LevelMismatch,
                    distance_m,
                    heading_delta_deg,
                    (None, None),
                );
                continue;
            }

            // Reach BFS (up to 2x64 edges) runs here only when a minimum
            // rejects on it; otherwise only for the survivors that get scored.
            let reach = checks_reach.then(|| reach_counts(edge.edge()));
            if let Some(counts @ (outbound, inbound)) = reach
                && (endpoint_options
                    .min_outbound_reach_edge_count
                    .is_some_and(|minimum| outbound.unwrap_or_default() < minimum)
                    || endpoint_options
                        .min_inbound_reach_edge_count
                        .is_some_and(|minimum| inbound.unwrap_or_default() < minimum))
            {
                reject(
                    CorrelationRejectReason::BelowMinReach,
                    distance_m,
                    heading_delta_deg,
                    counts,
                );
                continue;
            }
            survivors.push(Survivor {
                edge,
                snap,
                heading_deg,
                heading_delta_deg,
                reach,
            });
        }
        let rejected_candidate_count = rejected_candidates.len();

        // Phase 2: records and scores for the survivors that can make the
        // limit.
        let hinted = endpoint_options.has_scoring_hints();
        let mut score = |survivor: Survivor| -> Option<CorrelatedLocation> {
            let Survivor {
                edge,
                snap,
                heading_deg,
                heading_delta_deg,
                reach,
            } = survivor;
            let component_size = edge.component_size;
            let compact_edge = edge.edge();
            let (record, opposing_edge_id) = request_view.correlation_record(edge, profile)?;
            let (outbound_reach_edge_count, inbound_reach_edge_count) =
                reach.unwrap_or_else(|| reach_counts(compact_edge));
            let road_group_key = road_group_key(&record.shared_info);
            let confidence = confidence(snap.distance_m, max_distance_m);
            let score_breakdown = score_candidate_breakdown(
                snap.distance_m,
                max_distance_m,
                heading_delta_deg,
                snap.side,
                endpoint_options.preferred_side,
                component_size,
                &record.shared_info,
                profile,
                outbound_reach_edge_count,
                inbound_reach_edge_count,
                endpoint_options.preferred_layer,
                record.shared_info.layer,
                endpoint_options.preferred_level.as_deref(),
                record.shared_info.level.as_deref(),
                endpoint_options,
            );
            let score = score_breakdown.total_score;
            Some(CorrelatedLocation {
                edge_id: record.edge.edge_id,
                compact_edge: crate::types::CompactEdgeHint::of(record.raw_edge_index),
                shared_edge_info_id: record.edge.shared_edge_info_id,
                opposing_edge_id,
                road_group_key,
                same_road_candidate_count: 0,
                source_node_id: record.source.id,
                target_node_id: record.target.id,
                snapped_point: snap.point,
                distance_m: snap.distance_m,
                fraction_along_edge: snap.fraction,
                heading_deg,
                heading_delta_deg,
                side: snap.side,
                component_size,
                outbound_reach_edge_count,
                inbound_reach_edge_count,
                layer: record.shared_info.layer,
                level: record.shared_info.level.as_deref().map(str::to_string),
                score,
                score_breakdown,
                confidence,
                access: None,
            })
        };
        #[cfg(test)]
        let score_all = test_mode::score_all();
        #[cfg(not(test))]
        let score_all = false;
        let (mut candidates, unscored) = if score_all {
            (
                survivors.into_iter().filter_map(&mut score).collect(),
                Vec::new(),
            )
        } else if hinted {
            scored_by_bound(
                survivors,
                policy.candidate_limit,
                endpoint_options,
                max_distance_m,
                &mut score,
            )
        } else {
            scored_by_bucket(survivors, policy.candidate_limit, &mut score)
        };

        // Unscored survivors still count towards the same-road counts, by
        // their road group key alone (both directions of a segment share it).
        let mut unscored_keys = HashMap::<u32, Option<String>>::new();
        if !candidates.is_empty() {
            for edge in &unscored {
                let shared = sections.edge_shared_info[edge.edge() as usize];
                unscored_keys.entry(shared).or_insert_with(|| {
                    let mut info = SharedEdgeInfo {
                        osm_way_id: sections.si_way.get(shared as usize),
                        ..Default::default()
                    };
                    request_view.fill_attributes(shared as usize, &mut info)?;
                    road_group_key(&info)
                });
            }
        }
        let extra_keys = unscored.iter().filter_map(|edge| {
            unscored_keys
                .get(&sections.edge_shared_info[edge.edge() as usize])?
                .as_deref()
        });
        attach_same_road_candidate_counts(&mut candidates, extra_keys);

        // Both orders end on the (distinct) edge id, so they are total and an
        // unstable sort of indexes gives the same list without moving the
        // large candidate records around.
        let mut order = (0..candidates.len()).collect::<Vec<_>>();
        order.sort_unstable_by(|a, b| candidate_order(hinted, &candidates[*a], &candidates[*b]));
        order.truncate(policy.candidate_limit);
        let mut slots = candidates.into_iter().map(Some).collect::<Vec<_>>();
        let candidates = order
            .into_iter()
            .filter_map(|index| slots[index].take())
            .collect::<Vec<_>>();

        CandidateSearch {
            candidates,
            rejected_candidate_count,
            rejected_candidates,
            mode: CorrelationMode::Indexed,
            expanded_max_distance_m: None,
        }
    }

    /// [`Self::search`], widening the radius in turn to each of
    /// [`EXPANDED_CORRELATION_DISTANCES_M`] beyond the policy's while no
    /// candidate is usable or connected: when every candidate was rejected or
    /// lies on an island (a component under
    /// [`EXPANDED_CORRELATION_MIN_COMPONENT_SIZE`] nodes, or a reach under
    /// the reach cap either way). A wider search keeps only connected
    /// candidates, after up to half a candidate limit of the nearer island ones
    /// (an endpoint on the same island still needs those), and records its
    /// radius.
    ///
    /// `role` says which end of the search the point is; only a request's
    /// [`AccessPricer`](crate::access::AccessPricer) reads it.
    pub fn search_expanding(
        &self,
        point: GeoPoint,
        profile: MobilityProfile,
        role: crate::access::AccessRole,
        policy: &CorrelationPolicy,
        endpoint_options: &RouteEndpointOptions,
    ) -> CandidateSearch {
        if let Some(access) = &policy.access {
            let mut search = self.search_access(
                point,
                profile,
                policy,
                endpoint_options,
                access.resolve(profile),
            );
            if let Some(pricer) = &access.pricer {
                let candidates = std::mem::take(&mut search.candidates);
                search.candidates = pricer.0.price(crate::access::AccessQuery {
                    point,
                    profile,
                    role,
                    policy: access.resolve(profile),
                    candidates,
                    view: self.access,
                });
                search
                    .candidates
                    .truncate(crate::access::ACCESS_MAX_CANDIDATES);
            }
            return search;
        }
        let nearby = self.search(point, profile, policy, endpoint_options);
        if nearby.candidates.iter().any(connected) {
            return nearby;
        }
        let expanded_options = connected_options(endpoint_options);
        // The SAR `off_network` reach extends the radii beyond the last one.
        let off_network = policy
            .off_network_max_distance_m
            .filter(|distance| *distance > EXPANDED_CORRELATION_DISTANCES_M[1]);
        for max_distance_m in EXPANDED_CORRELATION_DISTANCES_M
            .into_iter()
            .chain(off_network)
        {
            if max_distance_m <= policy.max_distance_m {
                continue;
            }
            let wider = CorrelationPolicy {
                max_distance_m,
                ..policy.clone()
            };
            let expanded = self.search(point, profile, &wider, &expanded_options);
            if expanded.candidates.is_empty() {
                continue;
            }
            let mut expanded = with_islands(nearby, expanded, policy.candidate_limit);
            expanded.expanded_max_distance_m = Some(max_distance_m);
            return expanded;
        }
        nearby
    }

    /// The candidates of a request with costed access legs
    /// (`crate::access`): one search within the access radius, for up to
    /// its candidate count, each candidate priced by its stretch from
    /// `point`. The connectivity rule of [`Self::search_expanding`] holds
    /// within the radius: when no candidate is connected, the connected
    /// ones within it are searched for and take at least half the slots.
    /// Nothing widens beyond the radius.
    fn search_access(
        &self,
        point: GeoPoint,
        profile: MobilityProfile,
        policy: &CorrelationPolicy,
        endpoint_options: &RouteEndpointOptions,
        access: crate::access::AccessPolicy,
    ) -> CandidateSearch {
        let within = CorrelationPolicy {
            max_distance_m: access.radius_whole_m(),
            candidate_limit: access.candidate_limit,
            off_network_max_distance_m: None,
            sar_open: policy.sar_open,
            access: None,
        };
        let mut search = self.search(point, profile, &within, endpoint_options);
        if !search.candidates.is_empty() && !search.candidates.iter().any(connected) {
            let reachable = self.search(
                point,
                profile,
                &within,
                &connected_options(endpoint_options),
            );
            if !reachable.candidates.is_empty() {
                search = with_islands(search, reachable, within.candidate_limit);
            }
        }
        let priced = access.access_from(point);
        for candidate in &mut search.candidates {
            candidate.access = Some(priced.clone());
        }
        search
    }
}

/// Whether a candidate is connected to the network: a component of at least
/// [`EXPANDED_CORRELATION_MIN_COMPONENT_SIZE`] nodes and a saturated reach
/// both ways. Components count every mode's edges, so the profile's reach
/// must saturate too: a square open to cars but ringed by private streets
/// lies in the city's component and is still an island for cars.
fn connected(candidate: &CorrelatedLocation) -> bool {
    candidate
        .component_size
        .is_some_and(|size| size >= EXPANDED_CORRELATION_MIN_COMPONENT_SIZE)
        && [
            candidate.outbound_reach_edge_count,
            candidate.inbound_reach_edge_count,
        ]
        .iter()
        .all(|reach| reach.is_some_and(|reach| reach >= DEFAULT_REACH_EDGE_LIMIT))
}

/// `endpoint_options` asking for connected candidates only (see
/// [`connected`]).
fn connected_options(endpoint_options: &RouteEndpointOptions) -> RouteEndpointOptions {
    let mut options = endpoint_options.clone();
    options.min_component_size = Some(
        endpoint_options
            .min_component_size
            .unwrap_or_default()
            .max(EXPANDED_CORRELATION_MIN_COMPONENT_SIZE),
    );
    options.min_outbound_reach_edge_count = Some(
        endpoint_options
            .min_outbound_reach_edge_count
            .unwrap_or_default()
            .max(DEFAULT_REACH_EDGE_LIMIT),
    );
    options.min_inbound_reach_edge_count = Some(
        endpoint_options
            .min_inbound_reach_edge_count
            .unwrap_or_default()
            .max(DEFAULT_REACH_EDGE_LIMIT),
    );
    options
}

/// The connected candidates of `connected_search`, after at most half the
/// slots of `nearby`'s nearer island candidates, so that the connected ones
/// are always seeded; the rejections of both.
fn with_islands(
    nearby: CandidateSearch,
    mut connected_search: CandidateSearch,
    candidate_limit: usize,
) -> CandidateSearch {
    let mut candidates = nearby.candidates;
    candidates.truncate(candidate_limit / 2);
    for candidate in connected_search.candidates {
        if !candidates
            .iter()
            .any(|nearer| nearer.edge_id == candidate.edge_id)
        {
            candidates.push(candidate);
        }
    }
    candidates.truncate(candidate_limit.max(1));
    connected_search.candidates = candidates;
    connected_search.rejected_candidate_count += nearby.rejected_candidate_count;
    connected_search
        .rejected_candidates
        .splice(0..0, nearby.rejected_candidates);
    connected_search
}

#[derive(Debug, Clone)]
pub struct CandidateSearch {
    pub candidates: Vec<CorrelatedLocation>,
    pub rejected_candidate_count: usize,
    pub rejected_candidates: Vec<RejectedCorrelationCandidate>,
    pub mode: CorrelationMode,
    /// The radius `Correlator::search_expanding` widened to, if it did.
    pub expanded_max_distance_m: Option<u32>,
}

/// What phase 1 of `Correlator::search` keeps of a survivor.
struct Survivor {
    edge: CorrelationEdge,
    snap: Snap,
    heading_deg: Option<u16>,
    heading_delta_deg: Option<u16>,
    /// Counted in phase 1 when a minimum rejects on reach.
    reach: Option<ReachCounts>,
}

/// Outbound and inbound reach, capped at [`DEFAULT_REACH_EDGE_LIMIT`].
type ReachCounts = (Option<usize>, Option<usize>);

/// How `Correlator::search` ranks candidates: by score with scoring hints,
/// else by 2 m distance bucket first. Both end on the (distinct) edge id.
fn candidate_order(hinted: bool, a: &CorrelatedLocation, b: &CorrelatedLocation) -> Ordering {
    if hinted {
        b.score
            .total_cmp(&a.score)
            .then_with(|| a.distance_m.total_cmp(&b.distance_m))
            .then_with(|| a.edge_id.cmp(&b.edge_id))
    } else {
        default_distance_bucket(a.distance_m)
            .cmp(&default_distance_bucket(b.distance_m))
            .then_with(|| b.score.total_cmp(&a.score))
            .then_with(|| a.distance_m.total_cmp(&b.distance_m))
            .then_with(|| b.confidence.total_cmp(&a.confidence))
            .then_with(|| a.edge_id.cmp(&b.edge_id))
    }
}

/// Phase 2 without scoring hints: the order starts with the distance bucket
/// and nothing rejects a survivor any more, so only those in the buckets up
/// to the `limit`-th one's can make the limit. Returns the scored candidates
/// and the survivors left unscored.
fn scored_by_bucket(
    survivors: Vec<Survivor>,
    limit: usize,
    score: &mut impl FnMut(Survivor) -> Option<CorrelatedLocation>,
) -> (Vec<CorrelatedLocation>, Vec<CorrelationEdge>) {
    if survivors.len() <= limit {
        return (
            survivors.into_iter().filter_map(score).collect(),
            Vec::new(),
        );
    }
    if limit == 0 {
        return (Vec::new(), Vec::new());
    }
    let bucket = |survivor: &Survivor| default_distance_bucket(survivor.snap.distance_m);
    let mut buckets = survivors.iter().map(bucket).collect::<Vec<_>>();
    let threshold = *buckets.select_nth_unstable(limit - 1).1;
    let (kept, dropped): (Vec<_>, Vec<_>) = survivors
        .into_iter()
        .partition(|survivor| bucket(survivor) <= threshold);
    let mut candidates = kept.into_iter().filter_map(&mut *score).collect::<Vec<_>>();
    if candidates.len() < limit {
        // A kept survivor without a record (none in a valid package) lets a
        // dropped one in: score them all, as without the cut.
        candidates.extend(dropped.into_iter().filter_map(score));
        return (candidates, Vec::new());
    }
    (
        candidates,
        dropped.into_iter().map(|survivor| survivor.edge).collect(),
    )
}

/// Bounds and exact scores may sum the same terms in another order.
const SCORE_BOUND_SLACK: f64 = 1e-9;

/// Phase 2 with scoring hints: the order is by score. Survivors are scored in
/// the order of an upper bound on their score from what phase 1 knows, until
/// the next bound falls below the `limit`-th best score so far. Returns the
/// scored candidates and the survivors left unscored.
fn scored_by_bound(
    survivors: Vec<Survivor>,
    limit: usize,
    endpoint_options: &RouteEndpointOptions,
    max_distance_m: f64,
    score: &mut impl FnMut(Survivor) -> Option<CorrelatedLocation>,
) -> (Vec<CorrelatedLocation>, Vec<CorrelationEdge>) {
    if survivors.len() <= limit {
        return (
            survivors.into_iter().filter_map(score).collect(),
            Vec::new(),
        );
    }
    if limit == 0 {
        return (Vec::new(), Vec::new());
    }
    let mut bounded = survivors
        .into_iter()
        .map(|survivor| {
            (
                score_upper_bound(&survivor, endpoint_options, max_distance_m),
                survivor,
            )
        })
        .collect::<Vec<_>>();
    bounded.sort_unstable_by(|a, b| b.0.total_cmp(&a.0));
    let mut candidates = Vec::with_capacity(limit);
    // The `limit` best scores so far, best first.
    let mut best = Vec::<f64>::with_capacity(limit + 1);
    let mut unscored = Vec::new();
    let mut rest = bounded.into_iter();
    for (bound, survivor) in rest.by_ref() {
        if best.len() == limit && bound + SCORE_BOUND_SLACK < best[limit - 1] {
            // This and every later survivor score below `limit` candidates.
            unscored.push(survivor.edge);
            break;
        }
        if let Some(candidate) = score(survivor) {
            let at = best.partition_point(|better| better.total_cmp(&candidate.score).is_ge());
            best.insert(at, candidate.score);
            best.truncate(limit);
            candidates.push(candidate);
        }
    }
    unscored.extend(rest.map(|(_, survivor)| survivor.edge));
    (candidates, unscored)
}

/// The most `score_candidate_breakdown` can give a survivor: its exact
/// distance, heading, component, side, layer and level terms (and reach, when
/// phase 1 counted it), plus the largest road class, access, candidate type
/// (and reach) terms, which need the record.
fn score_upper_bound(
    survivor: &Survivor,
    endpoint_options: &RouteEndpointOptions,
    max_distance_m: f64,
) -> f64 {
    let reach = survivor.reach.map_or(REACH_WEIGHT, |(outbound, inbound)| {
        reach_score(outbound, inbound)
    });
    let candidate_type = if has_candidate_type_hints(endpoint_options) {
        0.5 * CANDIDATE_TYPE_WEIGHT
    } else {
        0.0
    };
    distance_score(survivor.snap.distance_m, max_distance_m)
        + heading_score(survivor.heading_delta_deg)
        + ROAD_CLASS_WEIGHT
        + ACCESS_WEIGHT
        + component_score(survivor.edge.component_size)
        + reach
        + side_score(survivor.snap.side, endpoint_options.preferred_side)
        + layer_match_score(endpoint_options.preferred_layer, survivor.edge.layer) * LAYER_WEIGHT
        + level_match_score(
            endpoint_options.preferred_level.as_deref(),
            survivor.edge.level.as_deref(),
        ) * LEVEL_WEIGHT
        + candidate_type
}

fn attach_same_road_candidate_counts<'k>(
    candidates: &mut [CorrelatedLocation],
    extra_keys: impl Iterator<Item = &'k str>,
) {
    let mut counts = HashMap::<&str, usize>::new();
    for key in candidates
        .iter()
        .filter_map(|candidate| candidate.road_group_key.as_deref())
    {
        *counts.entry(key).or_default() += 1;
    }
    for key in extra_keys {
        *counts.entry(key).or_default() += 1;
    }
    let counts = candidates
        .iter()
        .map(|candidate| {
            candidate
                .road_group_key
                .as_deref()
                .and_then(|key| counts.get(key))
                .copied()
                .unwrap_or(1)
        })
        .collect::<Vec<_>>();
    for (candidate, count) in candidates.iter_mut().zip(counts) {
        candidate.same_road_candidate_count = count;
    }
}

fn road_group_key(info: &SharedEdgeInfo) -> Option<String> {
    normalized_label(info.ref_name.as_deref())
        .map(|label| format!("ref:{label}"))
        .or_else(|| {
            info.route_refs
                .iter()
                .find_map(|route_ref| normalized_label(Some(route_ref)))
                .map(|label| format!("ref:{label}"))
        })
        .or_else(|| normalized_label(info.name.as_deref()).map(|label| format!("name:{label}")))
        .or_else(|| Some(format!("way:{}", info.osm_way_id)))
}

fn normalized_label(value: Option<&str>) -> Option<String> {
    let value = value?;
    // Most labels are already normalized: single spaces between words, none
    // at the ends. Copy those; collapse whitespace otherwise.
    let mut previous_space = true;
    let normalized_already = value.chars().all(|character| {
        let ok = if character.is_whitespace() {
            character == ' ' && !previous_space
        } else {
            true
        };
        previous_space = character.is_whitespace();
        ok
    }) && !previous_space;
    let normalized = if normalized_already {
        value.to_string()
    } else {
        value.split_whitespace().collect::<Vec<_>>().join(" ")
    };
    (!normalized.is_empty()).then_some(normalized)
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct Snap {
    pub(crate) point: GeoPoint,
    pub(crate) distance_m: f64,
    pub(crate) fraction: f64,
    side: CorrelationSide,
    /// The bearing of the geometry's segment at the snapped point.
    pub(crate) bearing_deg: Option<u16>,
}

/// The distance from `point` to `geometry` as correlation measures it.
#[cfg(test)]
pub(crate) fn snap_distance_m(point: GeoPoint, geometry: &[GeoPoint]) -> Option<f64> {
    snap_to_geometry(point, geometry).map(|snap| snap.distance_m)
}

pub(crate) fn snap_to_geometry(point: GeoPoint, geometry: &[GeoPoint]) -> Option<Snap> {
    if geometry.len() < 2 {
        return None;
    }
    let origin_lat = point.lat.to_radians();
    let meters_per_lat = 111_320.0;
    let meters_per_lng = 111_320.0 * origin_lat.cos().abs().max(0.01);
    let to_xy = |p: GeoPoint| -> (f64, f64) {
        (
            (p.lng - point.lng) * meters_per_lng,
            (p.lat - point.lat) * meters_per_lat,
        )
    };

    // Two passes over the windows (the same arithmetic both times) instead of
    // a buffer of segments: this runs for every candidate edge.
    let segments = || {
        geometry.windows(2).map(|segment| {
            let a = segment[0];
            let b = segment[1];
            let (ax, ay) = to_xy(a);
            let (bx, by) = to_xy(b);
            let vx = bx - ax;
            let vy = by - ay;
            let length = (vx * vx + vy * vy).sqrt();
            (a, b, ax, ay, vx, vy, length)
        })
    };
    let mut total_len = 0.0;
    for (.., length) in segments() {
        total_len += length;
    }
    if total_len <= f64::EPSILON {
        return None;
    }

    let mut best = None;
    let mut distance_before = 0.0;
    for (a, b, ax, ay, vx, vy, length) in segments() {
        if length <= f64::EPSILON {
            continue;
        }
        let t = (-(ax * vx + ay * vy) / (length * length)).clamp(0.0, 1.0);
        let x = ax + vx * t;
        let y = ay + vy * t;
        let distance_m = (x * x + y * y).sqrt();
        let snapped = GeoPoint::new(a.lat + (b.lat - a.lat) * t, a.lng + (b.lng - a.lng) * t);
        let fraction = ((distance_before + length * t) / total_len).clamp(0.0, 1.0);
        let snap = Snap {
            point: snapped,
            distance_m,
            fraction,
            side: side_of_segment(ax, ay, vx, vy),
            bearing_deg: Some(vx.atan2(vy).to_degrees().rem_euclid(360.0).round() as u16 % 360),
        };
        if best
            .as_ref()
            .is_none_or(|current: &Snap| snap.distance_m < current.distance_m)
        {
            best = Some(snap);
        }
        distance_before += length;
    }
    best
}

fn confidence(distance_m: f64, max_distance_m: f64) -> f64 {
    if max_distance_m <= f64::EPSILON {
        return MIN_CONFIDENCE;
    }
    (MAX_CONFIDENCE - distance_m / max_distance_m).clamp(MIN_CONFIDENCE, MAX_CONFIDENCE)
}

fn rejected_candidate(
    edge_id: Option<u64>,
    reason: CorrelationRejectReason,
    distance_m: Option<f64>,
    heading_delta_deg: Option<u16>,
    component_size: Option<usize>,
    outbound_reach_edge_count: Option<usize>,
    inbound_reach_edge_count: Option<usize>,
    layer: Option<i16>,
    level: Option<String>,
) -> RejectedCorrelationCandidate {
    RejectedCorrelationCandidate {
        edge_id,
        reason,
        distance_millimeters: distance_m
            .map(|distance| (distance * 1000.0).round().max(0.0) as u64),
        heading_delta_deg,
        component_size,
        outbound_reach_edge_count,
        inbound_reach_edge_count,
        layer,
        level,
    }
}

fn side_of_segment(ax: f64, ay: f64, vx: f64, vy: f64) -> CorrelationSide {
    let cross = vx * -ay - vy * -ax;
    if cross.abs() < 0.01 {
        CorrelationSide::OnEdge
    } else if cross > 0.0 {
        CorrelationSide::Left
    } else {
        CorrelationSide::Right
    }
}

fn heading_delta(a: u16, b: u16) -> u16 {
    let a = i32::from(a % 360);
    let b = i32::from(b % 360);
    let diff = (a - b).abs();
    diff.min(360 - diff) as u16
}

fn score_candidate_breakdown(
    distance_m: f64,
    max_distance_m: f64,
    heading_delta_deg: Option<u16>,
    side: CorrelationSide,
    preferred_side: PreferredRoadSide,
    component_size: Option<usize>,
    shared_info: &SharedEdgeInfo,
    profile: MobilityProfile,
    outbound_reach_edge_count: Option<usize>,
    inbound_reach_edge_count: Option<usize>,
    preferred_layer: Option<i16>,
    candidate_layer: Option<i16>,
    preferred_level: Option<&str>,
    candidate_level: Option<&str>,
    endpoint_options: &RouteEndpointOptions,
) -> CorrelationScoreBreakdown {
    let distance_score = distance_score(distance_m, max_distance_m);
    let heading_score = heading_score(heading_delta_deg);
    let road_score = profile_road_score(profile, shared_info.road_type) * ROAD_CLASS_WEIGHT;
    let access_score = access_preference_score(shared_info) * ACCESS_WEIGHT;
    let component_score = component_score(component_size);
    let reach_score = reach_score(outbound_reach_edge_count, inbound_reach_edge_count);
    let side_score = side_score(side, preferred_side);
    let layer_score = layer_match_score(preferred_layer, candidate_layer) * LAYER_WEIGHT;
    let level_score = level_match_score(preferred_level, candidate_level) * LEVEL_WEIGHT;
    let candidate_type_score = (candidate_type_preference_score(shared_info, endpoint_options)
        - 0.5)
        * CANDIDATE_TYPE_WEIGHT;
    let total_score = distance_score
        + heading_score
        + road_score
        + access_score
        + component_score
        + reach_score
        + side_score
        + layer_score
        + level_score
        + candidate_type_score;
    CorrelationScoreBreakdown {
        distance_score,
        heading_score,
        road_class_score: road_score,
        access_score,
        component_score,
        reach_score,
        side_score,
        layer_score,
        level_score,
        candidate_type_score,
        total_score,
    }
}

// The weights of the score's terms; each term is its weight times a value in
// [0, 1] (the candidate type's value is shifted by -0.5).
const DISTANCE_WEIGHT: f64 = 0.31;
const HEADING_WEIGHT: f64 = 0.16;
const ROAD_CLASS_WEIGHT: f64 = 0.13;
const ACCESS_WEIGHT: f64 = 0.10;
const COMPONENT_WEIGHT: f64 = 0.07;
const REACH_WEIGHT: f64 = 0.10;
const SIDE_WEIGHT: f64 = 0.05;
const LAYER_WEIGHT: f64 = 0.07;
const LEVEL_WEIGHT: f64 = 0.05;
const CANDIDATE_TYPE_WEIGHT: f64 = 0.20;

fn distance_score(distance_m: f64, max_distance_m: f64) -> f64 {
    confidence(distance_m, max_distance_m) * DISTANCE_WEIGHT
}

fn heading_score(heading_delta_deg: Option<u16>) -> f64 {
    heading_delta_deg
        .map(|delta| (1.0 - f64::from(delta.min(180)) / 180.0).clamp(0.0, 1.0))
        .unwrap_or(0.5)
        * HEADING_WEIGHT
}

fn component_score(component_size: Option<usize>) -> f64 {
    component_size
        .map(|size| ((size as f64).ln_1p() / 8.0).clamp(0.0, 1.0))
        .unwrap_or(0.0)
        * COMPONENT_WEIGHT
}

fn reach_score(
    outbound_reach_edge_count: Option<usize>,
    inbound_reach_edge_count: Option<usize>,
) -> f64 {
    outbound_reach_edge_count
        .into_iter()
        .chain(inbound_reach_edge_count)
        .max()
        .map(|count| ((count as f64).ln_1p() / 5.0).clamp(0.0, 1.0))
        .unwrap_or(0.0)
        * REACH_WEIGHT
}

fn side_score(side: CorrelationSide, preferred_side: PreferredRoadSide) -> f64 {
    let matches = |wanted| side == wanted || side == CorrelationSide::OnEdge;
    let value = match preferred_side {
        PreferredRoadSide::Either => 0.5,
        PreferredRoadSide::Left => f64::from(u8::from(matches(CorrelationSide::Left))),
        PreferredRoadSide::Right => f64::from(u8::from(matches(CorrelationSide::Right))),
    };
    value * SIDE_WEIGHT
}

fn default_distance_bucket(distance_m: f64) -> u64 {
    (distance_m.max(0.0) / 2.0).floor() as u64
}

fn access_preference_score(info: &SharedEdgeInfo) -> f64 {
    let mut score: f64 = 1.0;
    if info.is_service_like {
        score -= 0.20;
    }
    if info.is_destination_only {
        score -= 0.20;
    }
    if info.has_gate {
        score -= 0.25;
    }
    if info.is_access_restricted {
        score -= 0.30;
    }
    if info.is_ferry {
        score -= 0.15;
    }
    score.clamp(0.0, 1.0)
}

fn candidate_type_preference_score(
    info: &SharedEdgeInfo,
    endpoint_options: &RouteEndpointOptions,
) -> f64 {
    if !has_candidate_type_hints(endpoint_options) {
        return 0.5;
    }

    let mut score: f64 = 0.5;
    if !endpoint_options.preferred_road_classes.is_empty() {
        if endpoint_options
            .preferred_road_classes
            .iter()
            .any(|preference| preference.matches(info.road_type))
        {
            score += 0.25;
        } else {
            score -= 0.15;
        }
    }
    if endpoint_options
        .avoided_road_classes
        .iter()
        .any(|preference| preference.matches(info.road_type))
    {
        score -= 0.25;
    }

    let has_known_route_ref = info
        .ref_name
        .as_ref()
        .is_some_and(|value| !value.trim().is_empty())
        || info.route_refs.iter().any(|value| !value.trim().is_empty());
    if endpoint_options.prefer_any_route_ref {
        if has_known_route_ref {
            score += 0.20;
        } else {
            score -= 0.05;
        }
    }
    if !endpoint_options.preferred_route_refs.is_empty() {
        if preferred_route_ref_matches(info, &endpoint_options.preferred_route_refs) {
            score += 0.25;
        } else {
            score -= 0.10;
        }
    }

    if endpoint_options.avoid_service_roads && info.is_service_like {
        score -= 0.25;
    }
    if endpoint_options.avoid_private_access && private_like_access(info) {
        score -= 0.25;
    }

    score.clamp(0.0, 1.0)
}

fn has_candidate_type_hints(endpoint_options: &RouteEndpointOptions) -> bool {
    !endpoint_options.preferred_road_classes.is_empty()
        || !endpoint_options.avoided_road_classes.is_empty()
        || !endpoint_options.preferred_route_refs.is_empty()
        || endpoint_options.prefer_any_route_ref
        || endpoint_options.avoid_service_roads
        || endpoint_options.avoid_private_access
}

fn preferred_route_ref_matches(info: &SharedEdgeInfo, preferred_route_refs: &[String]) -> bool {
    let preferred = preferred_route_refs
        .iter()
        .filter_map(|route_ref| normalized_label(Some(route_ref)))
        .collect::<Vec<_>>();
    if preferred.is_empty() {
        return false;
    }

    normalized_label(info.ref_name.as_deref())
        .into_iter()
        .chain(
            info.route_refs
                .iter()
                .filter_map(|route_ref| normalized_label(Some(route_ref))),
        )
        .any(|candidate| {
            preferred
                .iter()
                .any(|requested| candidate.eq_ignore_ascii_case(requested))
        })
}

fn private_like_access(info: &SharedEdgeInfo) -> bool {
    info.is_access_restricted || info.has_gate || info.is_destination_only
}

fn layer_match_score(preferred_layer: Option<i16>, candidate_layer: Option<i16>) -> f64 {
    let Some(preferred_layer) = preferred_layer else {
        return 0.5;
    };
    let Some(candidate_layer) = candidate_layer else {
        return 0.2;
    };
    let delta = preferred_layer.abs_diff(candidate_layer);
    if delta == 0 {
        1.0
    } else if delta == 1 {
        0.55
    } else {
        0.0
    }
}

fn level_match_score(preferred_level: Option<&str>, candidate_level: Option<&str>) -> f64 {
    let Some(preferred_level) = preferred_level else {
        return 0.5;
    };
    let Some(candidate_level) = candidate_level else {
        return 0.2;
    };
    if level_matches(preferred_level, candidate_level) {
        1.0
    } else {
        0.0
    }
}

fn level_matches(preferred_level: &str, candidate_level: &str) -> bool {
    preferred_level
        .trim()
        .eq_ignore_ascii_case(candidate_level.trim())
}

fn profile_road_score(profile: MobilityProfile, road_type: RoadType) -> f64 {
    match profile {
        MobilityProfile::Default | MobilityProfile::Wheeled | MobilityProfile::Emergency => {
            match road_type {
                RoadType::Motorway | RoadType::Primary | RoadType::Secondary => 1.0,
                RoadType::Tertiary | RoadType::Residential => 0.8,
                RoadType::Track => 0.45,
                RoadType::Path => 0.15,
                RoadType::Other => 0.5,
            }
        }
        MobilityProfile::Tracked => match road_type {
            RoadType::Track => 1.0,
            RoadType::Residential | RoadType::Tertiary | RoadType::Other => 0.75,
            RoadType::Primary | RoadType::Secondary => 0.6,
            RoadType::Motorway => 0.35,
            RoadType::Path => 0.3,
        },
        MobilityProfile::OnFoot | MobilityProfile::Legged => match road_type {
            RoadType::Path | RoadType::Track => 1.0,
            RoadType::Residential | RoadType::Other => 0.7,
            RoadType::Tertiary => 0.45,
            RoadType::Primary | RoadType::Secondary => 0.25,
            RoadType::Motorway => 0.05,
        },
    }
}

#[cfg(test)]
pub(crate) mod test_mode {
    use std::cell::Cell;

    thread_local! {
        static SCORE_ALL: Cell<bool> = const { Cell::new(false) };
    }

    /// Makes `Correlator::search` score every survivor, as it used to.
    pub(crate) fn set_score_all(all: bool) {
        SCORE_ALL.with(|cell| cell.set(all));
    }

    pub(crate) fn score_all() -> bool {
        SCORE_ALL.with(Cell::get)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::access::AccessRole;
    use crate::types::CorrelationRoadClassPreference;
    use wayfarer_package::DirectionalAccess;
    use wayfarer_road_graph::{RoadGraph, RoadGraphBuilder, RoutableWay, enhance_road_graph};

    fn correlation_test_graph() -> RoadGraph {
        let mut builder = RoadGraphBuilder::new("correlation-test");
        builder.push_way(RoutableWay {
            osm_way_id: 1,
            points: vec![(50.0, 20.0), (50.0, 20.002)],
            source_node_ids: Vec::new(),
            road_type: RoadType::Primary,
            directional_access: DirectionalAccess::Both,
            name: Some("Opolska".into()),
            ref_name: None,
            highway: None,
            access: None,
            vehicle: None,
            motor_vehicle: None,
            motorcar: None,
            bicycle: None,
            foot: None,
            oneway_foot: None,
            oneway_bicycle: None,
            motorroad: None,
            service: None,
            surface: None,
            tracktype: None,
            smoothness: None,
            maxspeed: None,
            lanes: None,
            has_bridge: false,
            has_tunnel: false,
            has_ford: false,
            is_roundabout: false,
            is_link: false,
            is_destination_only: false,
            access_restricted: false,
            area: None,
            area_highway: None,
            layer: None,
            level: None,
            destination: None,
            destination_ref: None,
            destination_street: None,
            junction_ref: None,
            is_toll: false,
            is_ferry: false,
            has_gate: false,
            is_access_restricted: false,
            conditional_restrictions: Vec::new(),
            ..Default::default()
        });
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);
        graph
    }

    #[test]
    fn heading_tolerance_rejects_wrong_direction_candidates() {
        let graph = correlation_test_graph();
        let engine = crate::test_support::engine(&graph);
        let correlator = Correlator::new(engine.route_graph_view());
        let search = correlator.search(
            GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
            &CorrelationPolicy::default(),
            &RouteEndpointOptions::default()
                .with_heading(90)
                .with_heading_tolerance(35),
        );

        assert!(!search.candidates.is_empty());
        assert!(
            search
                .candidates
                .iter()
                .all(|candidate| candidate.heading_delta_deg.is_some_and(|delta| delta <= 35)),
            "all accepted candidates should match the requested heading"
        );
        let best = search.candidates.first().expect("accepted candidate");
        assert_eq!(best.score, best.score_breakdown.total_score);
        assert!(
            best.score_breakdown.heading_score > 0.0,
            "accepted candidate should expose heading contribution"
        );
        assert!(search.rejected_candidates.iter().any(|candidate| {
            candidate.reason == CorrelationRejectReason::HeadingOutsideTolerance
                && candidate.heading_delta_deg.is_some_and(|delta| delta > 35)
        }));
    }

    #[test]
    fn preferred_side_changes_candidate_order_without_rejecting_candidates() {
        let graph = correlation_test_graph();
        let engine = crate::test_support::engine(&graph);
        let correlator = Correlator::new(engine.route_graph_view());
        let point = GeoPoint::new(50.00005, 20.001);
        let policy = CorrelationPolicy::default();

        let left_search = correlator.search(
            point,
            MobilityProfile::Wheeled,
            &policy,
            &RouteEndpointOptions::default().with_preferred_side(PreferredRoadSide::Left),
        );
        let right_search = correlator.search(
            point,
            MobilityProfile::Wheeled,
            &policy,
            &RouteEndpointOptions::default().with_preferred_side(PreferredRoadSide::Right),
        );

        assert_eq!(left_search.rejected_candidate_count, 0);
        assert_eq!(right_search.rejected_candidate_count, 0);
        assert_eq!(
            left_search
                .candidates
                .first()
                .map(|candidate| candidate.side),
            Some(CorrelationSide::Left)
        );
        assert_eq!(
            right_search
                .candidates
                .first()
                .map(|candidate| candidate.side),
            Some(CorrelationSide::Right)
        );
        assert_ne!(
            left_search
                .candidates
                .first()
                .map(|candidate| candidate.edge_id),
            right_search
                .candidates
                .first()
                .map(|candidate| candidate.edge_id)
        );
        assert!(left_search.candidates[0].score_breakdown.side_score > 0.0);
        assert!(right_search.candidates[0].score_breakdown.side_score > 0.0);
    }

    #[test]
    fn default_correlation_prefers_unrestricted_road_in_near_distance_tie() {
        let mut restricted_track = RoutableWay {
            osm_way_id: 1,
            points: vec![(50.0, 20.0), (50.0, 20.002)],
            source_node_ids: Vec::new(),
            road_type: RoadType::Track,
            directional_access: DirectionalAccess::Both,
            name: Some("Restricted track".into()),
            ref_name: None,
            highway: None,
            access: Some("yes".into()),
            vehicle: None,
            motor_vehicle: None,
            motorcar: None,
            bicycle: None,
            foot: None,
            oneway_foot: None,
            oneway_bicycle: None,
            motorroad: None,
            service: Some("driveway".into()),
            surface: None,
            tracktype: None,
            smoothness: None,
            maxspeed: None,
            lanes: None,
            has_bridge: false,
            has_tunnel: false,
            has_ford: false,
            is_roundabout: false,
            is_link: false,
            is_destination_only: true,
            access_restricted: false,
            area: None,
            area_highway: None,
            layer: None,
            level: None,
            destination: None,
            destination_ref: None,
            destination_street: None,
            junction_ref: None,
            is_toll: false,
            is_ferry: false,
            has_gate: false,
            is_access_restricted: false,
            conditional_restrictions: Vec::new(),
            ..Default::default()
        };
        restricted_track.tracktype = Some("grade3".into());
        let public_primary = RoutableWay {
            osm_way_id: 2,
            points: vec![(50.000012, 20.0), (50.000012, 20.002)],
            road_type: RoadType::Primary,
            name: Some("Public road".into()),
            access: Some("yes".into()),
            service: None,
            tracktype: None,
            is_destination_only: false,
            access_restricted: false,
            has_gate: false,
            is_access_restricted: false,
            ..restricted_track.clone()
        };
        let mut builder = RoadGraphBuilder::new("near-tie-correlation");
        builder.push_way(restricted_track);
        builder.push_way(public_primary);
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);

        let engine = crate::test_support::engine(&graph);
        let candidates = Correlator::new(engine.route_graph_view())
            .candidates(
                GeoPoint::new(50.000003, 20.001),
                MobilityProfile::Wheeled,
                &CorrelationPolicy {
                    max_distance_m: 30,
                    candidate_limit: 4,
                    ..CorrelationPolicy::default()
                },
            )
            .expect("candidate search succeeds");

        assert_eq!(
            candidates
                .first()
                .and_then(|candidate| candidate.road_group_key.as_deref()),
            Some("name:Public road"),
            "public road should outrank a slightly closer restricted track"
        );
    }

    #[test]
    fn default_correlation_keeps_much_closer_restricted_candidate_first() {
        let mut restricted_track = RoutableWay {
            osm_way_id: 1,
            points: vec![(50.0, 20.0), (50.0, 20.002)],
            source_node_ids: Vec::new(),
            road_type: RoadType::Track,
            directional_access: DirectionalAccess::Both,
            name: Some("Restricted track".into()),
            ref_name: None,
            highway: None,
            access: Some("yes".into()),
            vehicle: None,
            motor_vehicle: None,
            motorcar: None,
            bicycle: None,
            foot: None,
            oneway_foot: None,
            oneway_bicycle: None,
            motorroad: None,
            service: Some("driveway".into()),
            surface: None,
            tracktype: None,
            smoothness: None,
            maxspeed: None,
            lanes: None,
            has_bridge: false,
            has_tunnel: false,
            has_ford: false,
            is_roundabout: false,
            is_link: false,
            is_destination_only: true,
            access_restricted: false,
            area: None,
            area_highway: None,
            layer: None,
            level: None,
            destination: None,
            destination_ref: None,
            destination_street: None,
            junction_ref: None,
            is_toll: false,
            is_ferry: false,
            has_gate: false,
            is_access_restricted: false,
            conditional_restrictions: Vec::new(),
            ..Default::default()
        };
        restricted_track.tracktype = Some("grade3".into());
        let public_primary = RoutableWay {
            osm_way_id: 2,
            points: vec![(50.00005, 20.0), (50.00005, 20.002)],
            road_type: RoadType::Primary,
            name: Some("Public road".into()),
            access: Some("yes".into()),
            service: None,
            tracktype: None,
            is_destination_only: false,
            access_restricted: false,
            has_gate: false,
            is_access_restricted: false,
            ..restricted_track.clone()
        };
        let mut builder = RoadGraphBuilder::new("clear-distance-correlation");
        builder.push_way(restricted_track);
        builder.push_way(public_primary);
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);

        let engine = crate::test_support::engine(&graph);
        let candidates = Correlator::new(engine.route_graph_view())
            .candidates(
                GeoPoint::new(50.000001, 20.001),
                MobilityProfile::Wheeled,
                &CorrelationPolicy {
                    max_distance_m: 30,
                    candidate_limit: 4,
                    ..CorrelationPolicy::default()
                },
            )
            .expect("candidate search succeeds");

        assert_eq!(
            candidates
                .first()
                .and_then(|candidate| candidate.road_group_key.as_deref()),
            Some("name:Restricted track"),
            "large distance difference should still dominate access preference"
        );
    }

    #[test]
    fn candidate_type_preferences_steer_ambiguous_endpoint_to_signed_road() {
        let service_driveway = RoutableWay {
            osm_way_id: 1,
            points: vec![(50.0, 20.0), (50.0, 20.002)],
            source_node_ids: Vec::new(),
            road_type: RoadType::Residential,
            directional_access: DirectionalAccess::Both,
            name: Some("Depot driveway".into()),
            ref_name: None,
            highway: None,
            access: Some("yes".into()),
            vehicle: None,
            motor_vehicle: None,
            motorcar: None,
            bicycle: None,
            foot: None,
            oneway_foot: None,
            oneway_bicycle: None,
            motorroad: None,
            service: Some("driveway".into()),
            surface: None,
            tracktype: None,
            smoothness: None,
            maxspeed: None,
            lanes: None,
            has_bridge: false,
            has_tunnel: false,
            has_ford: false,
            is_roundabout: false,
            is_link: false,
            is_destination_only: false,
            access_restricted: false,
            area: None,
            area_highway: None,
            layer: None,
            level: None,
            destination: None,
            destination_ref: None,
            destination_street: None,
            junction_ref: None,
            is_toll: false,
            is_ferry: false,
            has_gate: false,
            is_access_restricted: false,
            conditional_restrictions: Vec::new(),
            ..Default::default()
        };
        let signed_primary = RoutableWay {
            osm_way_id: 2,
            points: vec![(50.00005, 20.0), (50.00005, 20.002)],
            road_type: RoadType::Primary,
            name: Some("Public road".into()),
            ref_name: Some("MSR-1".into()),
            service: None,
            ..service_driveway.clone()
        };
        let mut builder = RoadGraphBuilder::new("candidate-type-preference-correlation");
        builder.push_way(service_driveway);
        builder.push_way(signed_primary);
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);

        let engine = crate::test_support::engine(&graph);
        let correlator = Correlator::new(engine.route_graph_view());
        let point = GeoPoint::new(50.000001, 20.001);
        let policy = CorrelationPolicy {
            max_distance_m: 30,
            candidate_limit: 4,
            ..CorrelationPolicy::default()
        };

        let default_candidates = correlator
            .candidates(point, MobilityProfile::Wheeled, &policy)
            .expect("default candidate search succeeds");
        assert_eq!(
            default_candidates
                .first()
                .and_then(|candidate| candidate.road_group_key.as_deref()),
            Some("name:Depot driveway"),
            "without endpoint type hints, the much closer driveway should remain first"
        );

        let preferred_search = correlator.search(
            point,
            MobilityProfile::Wheeled,
            &policy,
            &RouteEndpointOptions::default()
                .with_preferred_road_class(CorrelationRoadClassPreference::Primary)
                .with_prefer_any_route_ref()
                .with_preferred_route_ref("MSR-1")
                .with_avoid_service_roads(),
        );

        assert_eq!(preferred_search.rejected_candidate_count, 0);
        assert_eq!(
            preferred_search
                .candidates
                .first()
                .and_then(|candidate| candidate.road_group_key.as_deref()),
            Some("ref:MSR-1"),
            "candidate type preferences should steer ambiguous endpoints toward the signed primary"
        );
        let preferred = preferred_search.candidates.first().unwrap();
        assert!(
            preferred.score_breakdown.candidate_type_score > 0.0,
            "accepted signed primary should expose positive candidate-type score"
        );
    }

    fn plain_way(osm_way_id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
        RoutableWay {
            osm_way_id,
            points,
            road_type: RoadType::Primary,
            directional_access: DirectionalAccess::Both,
            name: Some(format!("Way {osm_way_id}").into()),
            highway: Some("primary".into()),
            access: Some("yes".into()),
            ..Default::default()
        }
    }

    fn graph_of(ways: Vec<RoutableWay>) -> RoadGraph {
        let mut builder = RoadGraphBuilder::new("correlation-shapes");
        for way in ways {
            builder.push_way(way);
        }
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);
        graph
    }

    #[test]
    fn heading_is_read_where_the_point_snaps_on_a_curved_edge() {
        // One edge: east for 72 m, then north for 111 m.
        let graph = graph_of(vec![plain_way(
            1,
            vec![(50.0, 20.0), (50.0, 20.001), (50.001, 20.001)],
        )]);
        let engine = crate::test_support::engine(&graph);
        let search = Correlator::new(engine.route_graph_view()).search(
            GeoPoint::new(50.0008, 20.00102),
            MobilityProfile::Wheeled,
            &CorrelationPolicy {
                max_distance_m: 25,
                ..CorrelationPolicy::default()
            },
            &RouteEndpointOptions::default()
                .with_heading(0)
                .with_heading_tolerance(30),
        );

        // The northbound direction is kept although the edge starts eastward;
        // the southbound one is rejected.
        assert_eq!(
            search.candidates.len(),
            1,
            "{:?}",
            search.rejected_candidates
        );
        let candidate = &search.candidates[0];
        assert!(candidate.fraction_along_edge > 0.5);
        assert_eq!(candidate.heading_deg, Some(0));
        assert_eq!(candidate.heading_delta_deg, Some(0));
        assert!(search.rejected_candidates.iter().any(|rejected| {
            rejected.reason == CorrelationRejectReason::HeadingOutsideTolerance
                && rejected.heading_delta_deg == Some(180)
        }));
    }

    #[test]
    fn expanding_search_looks_past_an_island_for_the_network() {
        // A network of 60 chained ways (61 nodes) and, 200 m north of it, a
        // lone 70 m way.
        let mut ways = (0..60)
            .map(|index| {
                let lng = 20.0 + f64::from(index) * 0.0005;
                plain_way(
                    i64::from(index) + 1,
                    vec![(50.0, lng), (50.0, lng + 0.0005)],
                )
            })
            .collect::<Vec<_>>();
        ways.push(plain_way(100, vec![(50.0018, 20.0), (50.0018, 20.001)]));
        let graph = graph_of(ways);
        let engine = crate::test_support::engine(&graph);
        let correlator = Correlator::new(engine.route_graph_view());
        let point = GeoPoint::new(50.0019, 20.0005);
        let policy = CorrelationPolicy::default();
        let options = RouteEndpointOptions::default();

        let nearby = correlator.search(point, MobilityProfile::Wheeled, &policy, &options);
        assert!(!nearby.candidates.is_empty());
        assert!(nearby.candidates.iter().all(|candidate| {
            candidate.component_size.unwrap_or_default() < EXPANDED_CORRELATION_MIN_COMPONENT_SIZE
        }));
        assert_eq!(nearby.expanded_max_distance_m, None);

        let expanded = correlator.search_expanding(
            point,
            MobilityProfile::Wheeled,
            AccessRole::Origin,
            &policy,
            &options,
        );
        assert_eq!(expanded.expanded_max_distance_m, Some(300));
        // The island's candidates stay first, then the network's.
        assert_eq!(expanded.candidates[0].edge_id, nearby.candidates[0].edge_id);
        assert!(expanded.candidates.iter().any(|candidate| {
            candidate.component_size.unwrap_or_default() >= EXPANDED_CORRELATION_MIN_COMPONENT_SIZE
                && candidate.distance_m > 150.0
        }));
        assert!(expanded.candidates.len() <= policy.candidate_limit);

        // Next to the network, nothing is widened.
        let connected = correlator.search_expanding(
            GeoPoint::new(50.0001, 20.01),
            MobilityProfile::Wheeled,
            AccessRole::Origin,
            &policy,
            &options,
        );
        assert_eq!(connected.expanded_max_distance_m, None);

        // Routes and matrices report the widened radius per endpoint.
        let far = GeoPoint::new(50.0001, 20.02);
        let route = engine
            .route_leg(crate::LegRequest::new(point, far, MobilityProfile::Wheeled))
            .expect("route from the widened origin");
        assert_eq!(
            route.diagnostics.origin_expanded_correlation_distance_m,
            Some(300)
        );
        assert_eq!(
            route
                .diagnostics
                .destination_expanded_correlation_distance_m,
            None
        );
        let json = serde_json::to_value(&route.diagnostics).unwrap();
        assert_eq!(json["originExpandedCorrelationDistanceM"], 300);
        assert!(
            json.get("destinationExpandedCorrelationDistanceM")
                .is_none()
        );
        let matrix = engine.matrix(crate::RouteMatrixRequest::new(
            vec![far, point],
            vec![far],
            MobilityProfile::Wheeled,
        ));
        assert_eq!(
            matrix.source_expanded_correlation_distances_m,
            vec![None, Some(300)]
        );
        assert!(matrix.target_expanded_correlation_distances_m.is_empty());
        let plain = engine.matrix(crate::RouteMatrixRequest::new(
            vec![far],
            vec![far],
            MobilityProfile::Wheeled,
        ));
        let json = serde_json::to_value(&plain).unwrap();
        assert!(json.get("sourceExpandedCorrelationDistancesM").is_none());
    }

    #[test]
    fn expanding_search_looks_past_a_profile_island_in_a_large_component() {
        // The 60-way network and, 200 m north of it, a lone 70 m road joined
        // to it by a footway: one component of 63 nodes, but a car on the
        // road reaches only the road.
        let mut ways = (0..60)
            .map(|index| {
                let lng = 20.0 + f64::from(index) * 0.0005;
                plain_way(
                    i64::from(index) + 1,
                    vec![(50.0, lng), (50.0, lng + 0.0005)],
                )
            })
            .collect::<Vec<_>>();
        ways.push(plain_way(100, vec![(50.0018, 20.0), (50.0018, 20.001)]));
        ways.push(RoutableWay {
            osm_way_id: 101,
            points: vec![(50.0018, 20.0), (50.0, 20.0)],
            road_type: RoadType::Path,
            highway: Some("footway".into()),
            ..Default::default()
        });
        let graph = graph_of(ways);
        let engine = crate::test_support::engine(&graph);
        let correlator = Correlator::new(engine.route_graph_view());
        let point = GeoPoint::new(50.0019, 20.0005);
        let policy = CorrelationPolicy::default();
        let options = RouteEndpointOptions::default();

        let nearby = correlator.search(point, MobilityProfile::Wheeled, &policy, &options);
        assert!(!nearby.candidates.is_empty());
        assert!(nearby.candidates.iter().all(|candidate| {
            candidate.component_size.unwrap_or_default() >= EXPANDED_CORRELATION_MIN_COMPONENT_SIZE
                && candidate.outbound_reach_edge_count.unwrap_or_default()
                    < DEFAULT_REACH_EDGE_LIMIT
        }));

        let expanded = correlator.search_expanding(
            point,
            MobilityProfile::Wheeled,
            AccessRole::Origin,
            &policy,
            &options,
        );
        assert_eq!(expanded.expanded_max_distance_m, Some(300));
        assert!(
            expanded
                .candidates
                .iter()
                .any(|candidate| candidate.distance_m > 150.0)
        );
        let route = engine.route_leg(crate::LegRequest::new(
            point,
            GeoPoint::new(50.0001, 20.02),
            MobilityProfile::Wheeled,
        ));
        assert!(route.is_ok(), "{route:?}");
    }

    /// The Monaco fixture, compiled as `scripts/check.sh` does.
    fn monaco() -> (tempfile::TempDir, crate::RoutingEngine) {
        let pbf =
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../fixtures/monaco.osm.pbf");
        let dir = tempfile::tempdir().unwrap();
        let output = dir.path().join("monaco.mrt");
        wayfarer_compiler::compile(
            &wayfarer_compiler::CompileRequest::new(pbf, output.clone())
                .scratch_dir(dir.path().join("scratch"))
                .graph_id("monaco")
                .country("MC"),
            &wayfarer_compiler::CancelToken::new(),
            |_| {},
        )
        .unwrap();
        let package = std::sync::Arc::new(crate::RoutingPackage::open(&output).unwrap());
        (dir, crate::RoutingEngine::from_package(package))
    }

    /// Scoring only the survivors that can make the limit gives what scoring
    /// all of them gave: candidates, same-road counts and rejections, with and
    /// without scoring hints, for every profile and through the widening
    /// search.
    #[test]
    fn two_phase_search_equals_scoring_every_survivor_on_monaco() {
        let (_dir, engine) = monaco();
        let correlator = Correlator::new(engine.route_graph_view());
        let hinted = RouteEndpointOptions {
            heading_deg: Some(17),
            heading_tolerance_deg: Some(120),
            preferred_side: PreferredRoadSide::Right,
            preferred_layer: Some(0),
            avoid_private_access: true,
            preferred_road_classes: vec![
                CorrelationRoadClassPreference::Primary,
                CorrelationRoadClassPreference::Secondary,
            ],
            avoid_service_roads: true,
            prefer_any_route_ref: true,
            min_outbound_reach_edge_count: Some(8),
            ..RouteEndpointOptions::default()
        };
        let option_sets = [
            RouteEndpointOptions::default(),
            hinted,
            RouteEndpointOptions {
                heading_deg: Some(250),
                ..RouteEndpointOptions::default()
            },
            RouteEndpointOptions {
                preferred_side: PreferredRoadSide::Left,
                preferred_level: Some("0".into()),
                ..RouteEndpointOptions::default()
            },
            RouteEndpointOptions {
                min_component_size: Some(EXPANDED_CORRELATION_MIN_COMPONENT_SIZE),
                min_outbound_reach_edge_count: Some(DEFAULT_REACH_EDGE_LIMIT),
                min_inbound_reach_edge_count: Some(DEFAULT_REACH_EDGE_LIMIT),
                ..RouteEndpointOptions::default()
            },
            RouteEndpointOptions {
                preferred_layer: Some(1),
                require_layer_match: true,
                avoided_road_classes: vec![CorrelationRoadClassPreference::Residential],
                ..RouteEndpointOptions::default()
            },
        ];
        let policies = [(150, 8), (40, 1), (300, 3), (1_000, 8), (75, 0)].map(
            |(max_distance_m, candidate_limit)| CorrelationPolicy {
                max_distance_m,
                candidate_limit,
                ..CorrelationPolicy::default()
            },
        );
        let profiles = [
            MobilityProfile::Wheeled,
            MobilityProfile::OnFoot,
            MobilityProfile::Tracked,
        ];
        let both = |run: &dyn Fn() -> CandidateSearch| {
            test_mode::set_score_all(true);
            let all = format!("{:?}", run());
            test_mode::set_score_all(false);
            (all, format!("{:?}", run()))
        };
        let mut compared = 0;
        let mut full = 0;
        for row in 0..9u8 {
            for col in 0..9u8 {
                let point = GeoPoint::new(
                    43.723 + f64::from(row) * 0.0037,
                    7.405 + f64::from(col) * 0.0043,
                );
                for profile in profiles {
                    for (index, options) in option_sets.iter().enumerate() {
                        // Fewer combinations at the wide radius.
                        for policy in policies.iter().filter(|policy| {
                            policy.max_distance_m < 1_000
                                || (usize::from(row + col) + index) % 4 == 0
                        }) {
                            let (all, two_phase) =
                                both(&|| correlator.search(point, profile, policy, options));
                            assert_eq!(
                                all, two_phase,
                                "{point:?} {profile:?} options {index} {policy:?}"
                            );
                            compared += 1;
                            let kept = all.matches("CorrelatedLocation {").count();
                            full += usize::from(kept > 0 && kept == policy.candidate_limit);
                        }
                    }
                    let policy = &policies[1];
                    let (all, two_phase) = both(&|| {
                        correlator.search_expanding(
                            point,
                            profile,
                            AccessRole::Origin,
                            policy,
                            &option_sets[0],
                        )
                    });
                    assert_eq!(all, two_phase, "{point:?} {profile:?} expanding");
                }
            }
        }
        assert!(compared > 3_000 && full > 600, "{compared} {full}");
    }
}

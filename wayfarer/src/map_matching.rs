//! Map matching: a trace of position samples to a connected path on the
//! package graph, with a hidden Markov model decoded by Viterbi (Newson &
//! Krumm 2009, as in OSRM and Valhalla's Meili).
//!
//! - **Candidates.** Each sample is correlated once, with a cheap search:
//!   the profile's accessible directed edges within the search radius,
//!   snapped, the `candidate_limit` most likely kept. No reach searches.
//! - **Emission** (negative log-likelihood): `0.5 * (d / sigma)^2` for the
//!   snap distance `d`, with `sigma` the configured GPS sigma, or the
//!   sample's reported accuracy when larger; plus
//!   `heading_weight * (1 - cos delta)` when the sample has a heading (and
//!   moves at 1 m/s or more, when it reports a speed), `delta` the angle to
//!   the edge's bearing at the snap point.
//! - **Transition**: `|route - great circle| / beta`, the route distance
//!   between the candidates of consecutive samples from a bounded
//!   one-to-many Dijkstra over directed edges (edge lengths, the profile's
//!   access, so oneways, simple and complex turn restrictions, partial
//!   endpoint edges). Its bound is `route_distance_factor * great circle +
//!   route_distance_slack_m` and, between timestamped samples,
//!   `max_speed_mps` times their time apart plus twice their sigmas; a pair
//!   beyond either is infeasible. A U-turn (onto the opposing edge at a
//!   node, or mid-edge where the source candidate is) costs
//!   `backtrack_penalty`; a mid-edge U-turn is only allowed when the trace
//!   reverses: the next sample's heading points against the edge, or the
//!   trace moves backwards along it. Moving backwards along the same
//!   directed edge by up to twice the target sample's sigma is GPS jitter,
//!   charged its length once more.
//! - **Outliers.** Up to `max_consecutive_outliers` samples in a row
//!   (including the first or last ones) may be skipped, at `outlier_cost`
//!   each, by transitions over them.
//! - **Trace breaks.** A time gap over `max_time_gap_s` or a jump over
//!   `breakage_distance_m` between consecutive matchable samples starts a new
//!   sub-trace. Otherwise a break before a candidate costs
//!   `(max_consecutive_outliers + 1) * outlier_cost` and is taken where it
//!   beats every transition into it, always where none is feasible (so a
//!   short burst of outliers is skipped rather than broken at).
//!
//! The result is one connected path per sub-trace (edge ids in travel order
//! and the geometry rebuilt from the edges), the matched position and edge of
//! every matched sample, and the samples rejected (no candidates, or skipped
//! as outliers).

use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::ops::Range;

use rustc_hash::FxHashMap;
use serde::{Deserialize, Serialize};

use crate::cancellation::RouteCancellation;
use crate::correlator::snap_to_geometry;
use crate::costing::heading_delta;
use crate::graph_view::PackageGraphView;
use crate::navigation::NavigationPositionSample;
use crate::package::restrictions::{ComplexAutomata, ROOT};
use crate::package::{EDGE_FLAG_SHAPE_REVERSED, NO_HEADING, SearchGraph, profile_bit};
use crate::types::{CorrelationScoreBreakdown, GeoPoint, MobilityProfile};

const CANCELLED: &str = "map matching was cancelled";

const DEFAULT_TRACE_MAX_CORRELATION_DISTANCE_M: u32 = 75;
const DEFAULT_TRACE_CANDIDATE_LIMIT: usize = 8;
const DEFAULT_HEADING_WEIGHT: f64 = 1.4;
const DEFAULT_TRANSITION_WEIGHT: f64 = 0.65;
const DEFAULT_BACKTRACK_PENALTY: f64 = 8.0;
const DEFAULT_TRACE_ALTERNATIVE_COUNT: usize = 2;
const MAX_TRACE_ALTERNATIVE_COUNT: usize = 8;
const DEFAULT_GPS_SIGMA_M: f64 = 5.0;
const DEFAULT_BETA_M: f64 = 10.0;
const DEFAULT_ROUTE_DISTANCE_FACTOR: f64 = 2.0;
const DEFAULT_ROUTE_DISTANCE_SLACK_M: f64 = 100.0;
const DEFAULT_MAX_TIME_GAP_S: f64 = 60.0;
const DEFAULT_BREAKAGE_DISTANCE_M: f64 = 2_000.0;
const DEFAULT_MAX_CONSECUTIVE_OUTLIERS: usize = 2;
const DEFAULT_OUTLIER_COST: f64 = 8.0;
const DEFAULT_MAX_SPEED_MPS: f64 = 50.0;
const MIN_SIGMA_M: f64 = 0.5;
const MIN_BETA_M: f64 = 0.1;
/// Below this reported speed a sample's heading is noise and is ignored.
const MIN_HEADING_SPEED_MPS: f64 = 1.0;
/// Moving backwards along an edge by up to this many sigmas is GPS jitter.
const BACKWARD_TOLERANCE_SIGMAS: f64 = 2.0;
/// A heading further than this from an edge's bearing points against it.
const REVERSED_HEADING_DELTA_DEG: u16 = 90;
const METERS_PER_DEGREE: f64 = 111_320.0;

const NO_CANDIDATES: &str = "no_candidates";
const OUTLIER: &str = "outlier";
const TIME_GAP: &str = "time_gap";
const DISTANCE_JUMP: &str = "distance_jump";
const NO_TRANSITION: &str = "no_transition";

/// Map-matching parameters. Fields missing from JSON take their defaults.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", default)]
pub struct MapMatchOptions {
    /// Candidate search radius around each sample (m), widened to twice the
    /// sample's reported accuracy.
    #[serde(default = "default_trace_max_correlation_distance_m")]
    pub max_correlation_distance_m: u32,
    /// Candidates kept per sample: the most likely by emission.
    #[serde(default = "default_trace_candidate_limit")]
    pub candidate_limit: usize,
    /// Weight of the emission's heading term, `heading_weight * (1 - cos
    /// delta)`.
    #[serde(default = "default_heading_weight")]
    pub heading_weight: f64,
    /// Unused since the HMM matcher (`beta_m` weighs transitions); kept for
    /// source compatibility.
    #[serde(default = "default_transition_weight")]
    pub transition_weight: f64,
    /// Cost (negative log-likelihood) of a U-turn onto the opposing edge, at a
    /// node or mid-edge. Formerly the penalty for moving backwards along one
    /// edge id.
    #[serde(default = "default_backtrack_penalty")]
    pub backtrack_penalty: f64,
    /// Alternative endings of the match reported, at most 8.
    #[serde(default = "default_trace_alternative_count")]
    pub alternative_count: usize,
    /// GPS noise standard deviation (m) of the Gaussian emission.
    pub gps_sigma_m: f64,
    /// Whether a sample's reported accuracy, when larger than `gps_sigma_m`,
    /// is its sigma.
    pub use_reported_accuracy: bool,
    /// Scale (m) of the exponential transition in `|route - great circle|`.
    pub beta_m: f64,
    /// Route searches between samples stop at this multiple of their great
    /// circle distance, plus `route_distance_slack_m`.
    pub route_distance_factor: f64,
    pub route_distance_slack_m: f64,
    /// A time gap over this between consecutive matchable samples breaks the
    /// trace (s).
    pub max_time_gap_s: f64,
    /// A jump over this between consecutive matchable samples breaks the
    /// trace (m).
    pub breakage_distance_m: f64,
    /// Samples in a row the match may skip as outliers.
    pub max_consecutive_outliers: usize,
    /// Cost of skipping one sample as an outlier.
    pub outlier_cost: f64,
    /// Route distance between timestamped samples is at most this speed
    /// times their time apart, plus twice their sigmas (m/s).
    pub max_speed_mps: f64,
}

impl Default for MapMatchOptions {
    fn default() -> Self {
        Self {
            max_correlation_distance_m: default_trace_max_correlation_distance_m(),
            candidate_limit: default_trace_candidate_limit(),
            heading_weight: default_heading_weight(),
            transition_weight: default_transition_weight(),
            backtrack_penalty: default_backtrack_penalty(),
            alternative_count: default_trace_alternative_count(),
            gps_sigma_m: DEFAULT_GPS_SIGMA_M,
            use_reported_accuracy: true,
            beta_m: DEFAULT_BETA_M,
            route_distance_factor: DEFAULT_ROUTE_DISTANCE_FACTOR,
            route_distance_slack_m: DEFAULT_ROUTE_DISTANCE_SLACK_M,
            max_time_gap_s: DEFAULT_MAX_TIME_GAP_S,
            breakage_distance_m: DEFAULT_BREAKAGE_DISTANCE_M,
            max_consecutive_outliers: DEFAULT_MAX_CONSECUTIVE_OUTLIERS,
            outlier_cost: DEFAULT_OUTLIER_COST,
            max_speed_mps: DEFAULT_MAX_SPEED_MPS,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct MatchedTraceSample {
    pub sample_index: usize,
    pub raw_position: GeoPoint,
    pub matched_edge_id: u64,
    pub snapped_position: GeoPoint,
    pub distance_meters: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub heading_delta_degrees: Option<u16>,
    /// `1 - distance / search radius`, as the correlator reports it.
    pub confidence: f64,
    /// The sample's emission likelihood: 1 on the road along its heading,
    /// `score_breakdown.distance_score * score_breakdown.heading_score`.
    pub score: f64,
    /// The emission's factors: `distance_score` (Gaussian in distance) and
    /// `heading_score` (1 without a usable heading); `total_score` is their
    /// product and the correlator's other terms are 0.
    #[serde(default)]
    pub score_breakdown: CorrelationScoreBreakdown,
    /// Index of the sample's sub-path in `MapMatchResult::sub_paths`.
    #[serde(default)]
    pub sub_path_index: usize,
    #[serde(default)]
    pub fraction_along_edge: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RejectedTraceSample {
    pub sample_index: usize,
    pub raw_position: GeoPoint,
    /// `no_candidates` (no accessible edge within the radius) or `outlier`
    /// (skipped by the match).
    pub reason: String,
    pub max_correlation_distance_meters: u32,
    pub candidate_limit: usize,
}

/// One connected stretch of the match, between trace breaks.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct MapMatchSubPath {
    pub first_sample_index: usize,
    pub last_sample_index: usize,
    pub matched_sample_count: usize,
    /// Traversed directed edges in order, from the first matched sample's
    /// edge to the last's; consecutive edges connect.
    pub edge_ids: Vec<u64>,
    /// The path's geometry from the first matched position to the last.
    pub geometry: Vec<GeoPoint>,
    pub distance_meters: f64,
    /// Why the trace broke before this sub-path: `time_gap`,
    /// `distance_jump` or `no_transition` (none for the first).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub break_reason: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct MapMatchResult {
    pub sample_count: usize,
    pub matched_sample_count: usize,
    pub rejected_sample_count: usize,
    /// The sub-paths' edge ids, concatenated (connected within each
    /// sub-path, not across breaks).
    pub matched_edge_ids: Vec<u64>,
    pub samples: Vec<MatchedTraceSample>,
    /// Other endings of the match (another candidate of its last matched
    /// sample) with a different edge sequence, cheapest first.
    #[serde(default)]
    pub alternatives: Vec<MapMatchAlternative>,
    #[serde(default)]
    pub rejected_samples: Vec<RejectedTraceSample>,
    /// The match's negative log-likelihood (emissions, transitions, skipped
    /// outliers and breaks).
    pub total_cost: f64,
    #[serde(default)]
    pub sub_paths: Vec<MapMatchSubPath>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct MapMatchAlternative {
    pub rank: usize,
    pub matched_sample_count: usize,
    pub matched_edge_ids: Vec<u64>,
    pub total_cost: f64,
    pub cost_delta: f64,
}

pub fn match_navigation_trace(
    access: &PackageGraphView,
    samples: &[NavigationPositionSample],
    profile: MobilityProfile,
    options: MapMatchOptions,
) -> Result<MapMatchResult, String> {
    match_navigation_trace_cancellable(
        access,
        samples,
        profile,
        options,
        &RouteCancellation::default(),
    )
}

/// [`match_navigation_trace`], stopping with an error once `cancellation`
/// fires; it is checked before each sample's candidates and each Viterbi step,
/// so a caller tells a cancelled match from a failed one by
/// [`RouteCancellation::check`].
pub fn match_navigation_trace_cancellable(
    access: &PackageGraphView,
    samples: &[NavigationPositionSample],
    profile: MobilityProfile,
    options: MapMatchOptions,
    cancellation: &RouteCancellation,
) -> Result<MapMatchResult, String> {
    if samples.is_empty() {
        return Err("trace must contain at least one sample".to_string());
    }

    let g = access.package().view();
    let bit = profile_bit(profile);
    let network = Network {
        g,
        automata: g.complex_automata(bit),
        profile_bit: bit,
    };
    let params = Params::new(options);
    let candidate_limit = options.candidate_limit.max(1);

    // Candidates, once per sample.
    let request = access.request();
    let mut steps = Vec::<Step>::new();
    let mut candidates = Vec::<Candidate>::new();
    let mut rejected_samples = Vec::<RejectedTraceSample>::new();
    let mut found = Vec::<Candidate>::new();
    for (sample_index, sample) in samples.iter().enumerate() {
        if cancellation.is_cancelled() {
            return Err(CANCELLED.to_string());
        }
        let max_distance_m = effective_max_distance_m(sample, options.max_correlation_distance_m);
        let sigma_m = sample_sigma_m(sample, options);
        let heading_deg = usable_heading_deg(sample);
        found.clear();
        for edge in request.correlation_edges(sample.position, max_distance_m, profile) {
            if !edge.accessible {
                continue;
            }
            let Some(compact) = g.edge_for_id(edge.edge_id) else {
                continue;
            };
            let Some(snap) = snap_to_geometry(sample.position, &edge.geometry) else {
                continue;
            };
            if snap.distance_m > f64::from(max_distance_m) {
                continue;
            }
            let start_heading = g.heading_start[compact as usize];
            let bearing_deg = snap
                .bearing_deg
                .or((start_heading != NO_HEADING).then_some(start_heading));
            let heading_delta_deg = heading_deg
                .zip(bearing_deg)
                .map(|(heading, bearing)| heading_delta(heading, bearing));
            let z = snap.distance_m / sigma_m;
            found.push(Candidate {
                step: steps.len(),
                edge: compact,
                point: snap.point,
                distance_m: snap.distance_m,
                fraction: snap.fraction,
                bearing_deg,
                heading_delta_deg,
                distance_cost: 0.5 * z * z,
                heading_cost: heading_delta_deg.map_or(0.0, |delta| {
                    options.heading_weight.max(0.0) * (1.0 - f64::from(delta).to_radians().cos())
                }),
            });
        }
        if found.is_empty() {
            rejected_samples.push(rejected(
                sample_index,
                sample,
                NO_CANDIDATES,
                max_distance_m,
                candidate_limit,
            ));
            continue;
        }
        found.sort_by(|a, b| {
            a.emission()
                .total_cmp(&b.emission())
                .then_with(|| g.edge_id(a.edge as usize).cmp(&g.edge_id(b.edge as usize)))
        });
        found.truncate(candidate_limit);
        let start = candidates.len();
        candidates.append(&mut found);
        let break_before = steps.last().and_then(|previous| {
            hard_break(previous, sample.position, sample.sampled_at_ms, options)
        });
        steps.push(Step {
            sample_index,
            position: sample.position,
            sampled_at_ms: sample.sampled_at_ms,
            heading_deg,
            sigma_m,
            max_distance_m,
            candidates: start..candidates.len(),
            break_before,
        });
    }
    if steps.is_empty() {
        return Err("trace produced no matchable samples".to_string());
    }

    // Viterbi, pushing each sample's states forward to the next
    // `max_consecutive_outliers + 1` samples. States sit beside their
    // candidates; nothing is cloned per step.
    let mut states = vec![State::UNREACHED; candidates.len()];
    let mut pool = Vec::<u32>::new();
    let mut search = RouteSearch::default();
    let mut targets = Vec::<Target>::new();
    // The current chain (the trace, or its part after the last hard break):
    // its first step, and the cost, predecessor and link a match starting
    // in it takes. A match may start at any of its first `max_skipped + 1`
    // steps, skipping the ones before as outliers.
    let mut chain_floor = 0;
    let mut chain_start = (0.0, NO_STATE, Link::Start);
    for t in 0..steps.len() {
        if cancellation.is_cancelled() {
            return Err(CANCELLED.to_string());
        }
        let range = steps[t].candidates.clone();
        if let Some(reason) = steps[t].break_before {
            let (cost, back) =
                best_recent(&steps, &states, chain_floor, t, &params).unwrap_or((0.0, NO_STATE));
            chain_floor = t;
            chain_start = (cost, back, Link::Break(reason));
        }
        // Each candidate takes the cheapest of its transitions, a start of
        // the chain, and a break (always open, so taken where no transition
        // reaches it).
        let start = (t - chain_floor <= params.max_skipped).then(|| {
            let (base, back, link) = chain_start;
            let skipped = params.outlier_cost * (t - chain_floor) as f64;
            (base + skipped, back, link)
        });
        let soft_break = best_recent(&steps, &states, chain_floor, t, &params)
            .map(|(cost, back)| (cost + params.break_cost, back, Link::Break(NO_TRANSITION)));
        for (base, back, link) in [start, soft_break].into_iter().flatten() {
            for index in range.clone() {
                let cost = base + candidates[index].emission();
                if cost < states[index].cost {
                    states[index] = State {
                        cost,
                        back,
                        link,
                        path: 0..0,
                        start_fraction: candidates[index].fraction,
                    };
                }
            }
        }
        // States costlier than the best by more than a break cannot lead to
        // a cheaper match than a break from the best: no search from them.
        let prune_above = states[range.clone()]
            .iter()
            .map(|state| state.cost)
            .fold(f64::INFINITY, f64::min)
            + params.break_cost;

        targets.clear();
        let last = (t + params.max_skipped + 1).min(steps.len() - 1);
        for u in t + 1..=last {
            if steps[u].break_before.is_some() {
                break;
            }
            let great_circle_m = steps[t].position.distance_to(steps[u].position);
            let bound_m =
                great_circle_m * params.route_distance_factor + params.route_distance_slack_m;
            // No faster than `max_speed_mps` between timestamped samples, give
            // or take the noise of both positions.
            let max_route_m = match (steps[t].sampled_at_ms, steps[u].sampled_at_ms) {
                (Some(from), Some(to)) if to > from => {
                    params.max_speed_mps * (to - from) as f64 / 1000.0
                        + 2.0 * (steps[t].sigma_m + steps[u].sigma_m)
                }
                _ => f64::INFINITY,
            };
            for index in steps[u].candidates.clone() {
                let candidate = &candidates[index];
                targets.push(Target {
                    candidate: index,
                    skipped: u - t - 1,
                    edge: candidate.edge,
                    source: g.edge_source(candidate.edge as usize),
                    fraction: candidate.fraction,
                    length_m: network.length_m(candidate.edge),
                    great_circle_m,
                    bound_m,
                    max_route_m,
                    backward_tolerance_m: BACKWARD_TOLERANCE_SIGMAS * steps[u].sigma_m,
                    position: steps[u].position,
                    heading_deg: steps[u].heading_deg,
                });
            }
        }
        if targets.is_empty() {
            continue;
        }
        for index in range {
            let from_cost = states[index].cost;
            if !from_cost.is_finite() || from_cost > prune_above {
                continue;
            }
            let source = &candidates[index];
            let reverses = trace_reverses(source, steps[t].position, &targets);
            search.run(&network, &params, source, reverses, &targets);
            for (at, target) in targets.iter().enumerate() {
                let Some(reach) = search.reached[at] else {
                    continue;
                };
                let cost = from_cost
                    + reach.cost
                    + params.outlier_cost * target.skipped as f64
                    + candidates[target.candidate].emission();
                if cost < states[target.candidate].cost {
                    let start = pool.len();
                    let start_fraction = search.path(reach.via, target.edge, &mut pool);
                    states[target.candidate] = State {
                        cost,
                        back: index as u32,
                        link: Link::Transition,
                        path: start as u32..pool.len() as u32,
                        start_fraction,
                    };
                }
            }
        }
    }

    let trace = Trace {
        g,
        steps: &steps,
        candidates: &candidates,
        states: &states,
        pool: &pool,
    };
    let Some((total_cost, terminal)) =
        best_recent(&steps, &states, chain_floor, steps.len(), &params)
    else {
        return Err("trace produced no matchable samples".to_string());
    };
    let terminal = terminal as usize;
    let best = trace.assemble(terminal, true);

    let mut on_path = vec![false; steps.len()];
    for sample in &best.samples {
        if let Ok(step) = steps.binary_search_by_key(&sample.sample_index, |step| step.sample_index)
        {
            on_path[step] = true;
        }
    }
    for (step, matched) in steps.iter().zip(on_path) {
        if !matched {
            rejected_samples.push(rejected(
                step.sample_index,
                &samples[step.sample_index],
                OUTLIER,
                step.max_distance_m,
                candidate_limit,
            ));
        }
    }
    rejected_samples.sort_by_key(|sample| sample.sample_index);

    // Alternative endings: the terminal sample's other candidates, where
    // their match keeps the same samples.
    let terminal_step = candidates[terminal].step;
    let trailing_cost = params.outlier_cost * (steps.len() - 1 - terminal_step) as f64;
    let mut endings = steps[terminal_step]
        .candidates
        .clone()
        .filter(|index| *index != terminal && states[*index].cost.is_finite())
        .collect::<Vec<_>>();
    endings.sort_by(|a, b| {
        states[*a]
            .cost
            .total_cmp(&states[*b].cost)
            .then_with(|| candidates[*a].edge.cmp(&candidates[*b].edge))
    });
    let limit = options.alternative_count.min(MAX_TRACE_ALTERNATIVE_COUNT);
    let mut alternatives = Vec::new();
    let mut seen_edge_sequences = vec![best.edge_ids.clone()];
    for ending in endings {
        if alternatives.len() >= limit {
            break;
        }
        let alternative = trace.assemble(ending, false);
        let same_samples = alternative.samples.len() == best.samples.len()
            && alternative
                .samples
                .iter()
                .zip(&best.samples)
                .all(|(left, right)| left.sample_index == right.sample_index);
        if !same_samples || seen_edge_sequences.contains(&alternative.edge_ids) {
            continue;
        }
        seen_edge_sequences.push(alternative.edge_ids.clone());
        let cost = states[ending].cost + trailing_cost;
        alternatives.push(MapMatchAlternative {
            rank: alternatives.len() + 1,
            matched_sample_count: alternative.samples.len(),
            matched_edge_ids: alternative.edge_ids,
            total_cost: cost,
            cost_delta: cost - total_cost,
        });
    }

    Ok(MapMatchResult {
        sample_count: samples.len(),
        matched_sample_count: best.samples.len(),
        rejected_sample_count: rejected_samples.len(),
        matched_edge_ids: best.edge_ids,
        samples: best.samples,
        alternatives,
        rejected_samples,
        total_cost,
        sub_paths: best.sub_paths,
    })
}

/// The matcher's derived parameters.
struct Params {
    beta_m: f64,
    /// A U-turn's cost as route metres (`backtrack_penalty * beta`).
    u_turn_m: f64,
    route_distance_factor: f64,
    route_distance_slack_m: f64,
    outlier_cost: f64,
    break_cost: f64,
    max_skipped: usize,
    max_speed_mps: f64,
}

impl Params {
    fn new(options: MapMatchOptions) -> Self {
        let beta_m = options.beta_m.max(MIN_BETA_M);
        let outlier_cost = options.outlier_cost.max(0.0);
        Self {
            beta_m,
            u_turn_m: options.backtrack_penalty.max(0.0) * beta_m,
            route_distance_factor: options.route_distance_factor.max(1.0),
            route_distance_slack_m: options.route_distance_slack_m.max(0.0),
            outlier_cost,
            break_cost: outlier_cost * (options.max_consecutive_outliers as f64 + 1.0),
            max_skipped: options.max_consecutive_outliers,
            max_speed_mps: if options.max_speed_mps > 0.0 {
                options.max_speed_mps
            } else {
                f64::INFINITY
            },
        }
    }
}

/// A matchable sample (one with candidates).
struct Step {
    sample_index: usize,
    position: GeoPoint,
    sampled_at_ms: Option<i64>,
    heading_deg: Option<u16>,
    sigma_m: f64,
    max_distance_m: u32,
    candidates: Range<usize>,
    /// Why the trace breaks between the previous step and this one.
    break_before: Option<&'static str>,
}

#[derive(Debug, Clone, Copy)]
struct Candidate {
    step: usize,
    edge: u32,
    point: GeoPoint,
    distance_m: f64,
    fraction: f64,
    bearing_deg: Option<u16>,
    heading_delta_deg: Option<u16>,
    distance_cost: f64,
    heading_cost: f64,
}

impl Candidate {
    fn emission(&self) -> f64 {
        self.distance_cost + self.heading_cost
    }
}

const NO_STATE: u32 = u32::MAX;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Link {
    /// The trace starts here.
    Start,
    /// From the previous state over a route (its edges in `State::path`).
    Transition,
    /// A new sub-trace, after the previous state.
    Break(&'static str),
}

/// The Viterbi state of one candidate: the cheapest match ending there.
#[derive(Debug, Clone)]
struct State {
    cost: f64,
    back: u32,
    link: Link,
    /// The transition's route in the edge pool, from the previous state's
    /// edge (or its opposing edge after a U-turn) to this candidate's.
    path: Range<u32>,
    /// Where the route starts on its first edge.
    start_fraction: f64,
}

impl State {
    const UNREACHED: Self = Self {
        cost: f64::INFINITY,
        back: NO_STATE,
        link: Link::Start,
        path: 0..0,
        start_fraction: 0.0,
    };
}

/// The cheapest state of the `max_skipped + 1` steps before `t` (none before
/// `floor`), charged the outlier cost of every step after it.
fn best_recent(
    steps: &[Step],
    states: &[State],
    floor: usize,
    t: usize,
    params: &Params,
) -> Option<(f64, u32)> {
    let first = t.saturating_sub(params.max_skipped + 1).max(floor);
    (first..t)
        .flat_map(|s| {
            let skipped = params.outlier_cost * (t - 1 - s) as f64;
            steps[s]
                .candidates
                .clone()
                .map(move |index| (states[index].cost + skipped, index as u32))
        })
        .filter(|(cost, _)| cost.is_finite())
        .min_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.cmp(&b.1)))
}

fn hard_break(
    previous: &Step,
    position: GeoPoint,
    sampled_at_ms: Option<i64>,
    options: MapMatchOptions,
) -> Option<&'static str> {
    if let (Some(before), Some(at)) = (previous.sampled_at_ms, sampled_at_ms)
        && (at - before) as f64 > options.max_time_gap_s * 1000.0
    {
        return Some(TIME_GAP);
    }
    (previous.position.distance_to(position) > options.breakage_distance_m).then_some(DISTANCE_JUMP)
}

/// Whether the trace reverses against the source candidate's edge on the way
/// to any target sample: a heading pointing against the edge's bearing, or a
/// displacement backwards along it.
fn trace_reverses(source: &Candidate, from: GeoPoint, targets: &[Target]) -> bool {
    let Some(bearing) = source.bearing_deg else {
        return false;
    };
    let (sin, cos) = f64::from(bearing).to_radians().sin_cos();
    let meters_per_lng = METERS_PER_DEGREE * from.lat.to_radians().cos().abs().max(0.01);
    targets.iter().any(|target| {
        target
            .heading_deg
            .is_some_and(|heading| heading_delta(heading, bearing) > REVERSED_HEADING_DELTA_DEG)
            || {
                let east = (target.position.lng - from.lng) * meters_per_lng;
                let north = (target.position.lat - from.lat) * METERS_PER_DEGREE;
                east * sin + north * cos < 0.0
            }
    })
}

fn effective_max_distance_m(sample: &NavigationPositionSample, base: u32) -> u32 {
    let accuracy = sample
        .accuracy_meters
        .filter(|value| value.is_finite() && *value > 0.0)
        .unwrap_or(0.0);
    f64::from(base)
        .max(accuracy * 2.0)
        .round()
        .clamp(1.0, f64::from(u32::MAX)) as u32
}

fn sample_sigma_m(sample: &NavigationPositionSample, options: MapMatchOptions) -> f64 {
    let sigma = options.gps_sigma_m.max(MIN_SIGMA_M);
    match sample
        .accuracy_meters
        .filter(|value| options.use_reported_accuracy && value.is_finite() && *value > 0.0)
    {
        Some(accuracy) => accuracy.max(sigma),
        None => sigma,
    }
}

fn usable_heading_deg(sample: &NavigationPositionSample) -> Option<u16> {
    let moving = sample
        .speed_meters_per_second
        .is_none_or(|speed| !speed.is_finite() || speed >= MIN_HEADING_SPEED_MPS);
    sample
        .heading_degrees
        .filter(|heading| heading.is_finite() && moving)
        .map(|heading| heading.rem_euclid(360.0).round() as u16 % 360)
}

fn rejected(
    sample_index: usize,
    sample: &NavigationPositionSample,
    reason: &str,
    max_distance_m: u32,
    candidate_limit: usize,
) -> RejectedTraceSample {
    RejectedTraceSample {
        sample_index,
        raw_position: sample.position,
        reason: reason.to_string(),
        max_correlation_distance_meters: max_distance_m,
        candidate_limit,
    }
}

/// The package graph as the matcher's route searches read it.
struct Network<'g> {
    g: SearchGraph<'g>,
    automata: &'g ComplexAutomata,
    profile_bit: u8,
}

impl Network<'_> {
    fn accessible(&self, edge: u32) -> bool {
        self.g.edge_profile_access[edge as usize] & self.profile_bit != 0
    }

    fn length_m(&self, edge: u32) -> f64 {
        f64::from(self.g.edge_length[edge as usize])
    }

    /// Whether a path in restriction `state` whose last edge is `previous`
    /// may not enter `next`.
    fn restricted(&self, state: u32, previous: u32, next: u32) -> bool {
        self.g.simple_restricted(previous, next, self.profile_bit)
            || self.automata.forbids(state, next)
    }

    /// Whether `next` runs back along `previous`' road segment.
    fn is_u_turn(&self, previous: u32, next: u32) -> bool {
        let (previous, next) = (previous as usize, next as usize);
        self.g.edge_shared_info[previous] == self.g.edge_shared_info[next]
            && (self.g.edge_flags[previous] ^ self.g.edge_flags[next]) & EDGE_FLAG_SHAPE_REVERSED
                != 0
    }

    /// The accessible edge running back along `edge`'s road segment.
    fn opposing(&self, edge: u32) -> Option<u32> {
        let g = self.g;
        let (source, target) = (g.edge_source(edge as usize), g.edge_target[edge as usize]);
        (g.first_out[target as usize]..g.first_out[target as usize + 1]).find(|other| {
            g.edge_target[*other as usize] == source
                && self.is_u_turn(edge, *other)
                && self.accessible(*other)
        })
    }
}

/// A candidate of a later sample, as a route search's destination.
struct Target {
    candidate: usize,
    /// Samples skipped (as outliers) to reach this one.
    skipped: usize,
    edge: u32,
    source: u32,
    fraction: f64,
    length_m: f64,
    great_circle_m: f64,
    /// Bound on route distance plus penalties (from the great circle).
    bound_m: f64,
    /// Bound on route distance (from the time apart).
    max_route_m: f64,
    backward_tolerance_m: f64,
    position: GeoPoint,
    heading_deg: Option<u16>,
}

const NO_LABEL: u32 = u32::MAX;

#[derive(Debug, Clone, Copy)]
struct RouteLabel {
    edge: u32,
    state: u32,
    /// Route distance to the end of `edge`.
    distance_m: f64,
    /// U-turn penalties and backward jitter on the way, in metres.
    penalty_m: f64,
    pred: u32,
    /// Where a seed starts on its edge.
    start_fraction: f64,
}

#[derive(Debug, Clone, Copy)]
struct QueueEntry {
    priority: f64,
    label: u32,
}

impl PartialEq for QueueEntry {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for QueueEntry {}

impl PartialOrd for QueueEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for QueueEntry {
    /// Reversed, so the heap pops the cheapest first.
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .priority
            .total_cmp(&self.priority)
            .then_with(|| other.label.cmp(&self.label))
    }
}

#[derive(Debug, Clone, Copy)]
enum Via {
    /// Along one edge from `start_fraction`, without leaving it.
    Direct { edge: u32, start_fraction: f64 },
    /// Through the network, finishing onto the target edge after `label`.
    Label(u32),
}

#[derive(Debug, Clone, Copy)]
struct Reach {
    /// The transition cost.
    cost: f64,
    via: Via,
}

/// A bounded one-to-many Dijkstra over directed edges and complex-restriction
/// states, from one candidate to the candidates of the next samples. Its
/// buffers are kept between searches.
#[derive(Default)]
struct RouteSearch {
    labels: Vec<RouteLabel>,
    best: FxHashMap<(u32, u32), u32>,
    heap: BinaryHeap<QueueEntry>,
    reached: Vec<Option<Reach>>,
    /// The search's bounds: the targets' farthest.
    bound_m: f64,
    max_route_m: f64,
}

impl RouteSearch {
    fn run(
        &mut self,
        network: &Network<'_>,
        params: &Params,
        source: &Candidate,
        reverses: bool,
        targets: &[Target],
    ) {
        self.labels.clear();
        self.best.clear();
        self.heap.clear();
        self.reached.clear();
        self.reached.resize(targets.len(), None);
        let g = network.g;
        let edge = source.edge;
        let length_m = network.length_m(edge);
        let opposing = if reverses {
            network.opposing(edge)
        } else {
            None
        };

        // Targets on the source's own segment.
        for (at, target) in targets.iter().enumerate() {
            if target.edge == edge {
                let ahead_m = (target.fraction - source.fraction) * length_m;
                let via = Via::Direct {
                    edge,
                    start_fraction: source.fraction,
                };
                if ahead_m >= 0.0 {
                    self.reach(params, at, target, ahead_m, 0.0, via);
                } else if -ahead_m <= target.backward_tolerance_m {
                    self.reach(params, at, target, -ahead_m, -ahead_m, via);
                }
            } else if opposing == Some(target.edge) {
                let start_fraction = 1.0 - source.fraction;
                let ahead_m = (target.fraction - start_fraction) * length_m;
                if ahead_m >= 0.0 {
                    let via = Via::Direct {
                        edge: target.edge,
                        start_fraction,
                    };
                    self.reach(params, at, target, ahead_m, params.u_turn_m, via);
                }
            }
        }

        let farthest = |bound: fn(&Target) -> f64| targets.iter().map(bound).fold(0.0, f64::max);
        self.bound_m = farthest(|target| target.bound_m);
        self.max_route_m = farthest(|target| target.max_route_m);
        self.push(RouteLabel {
            edge,
            state: network.automata.forward_next(&g, ROOT, edge),
            distance_m: (1.0 - source.fraction) * length_m,
            penalty_m: 0.0,
            pred: NO_LABEL,
            start_fraction: source.fraction,
        });
        if let Some(opposing) = opposing {
            self.push(RouteLabel {
                edge: opposing,
                state: network.automata.forward_next(&g, ROOT, opposing),
                distance_m: source.fraction * length_m,
                penalty_m: params.u_turn_m,
                pred: NO_LABEL,
                start_fraction: 1.0 - source.fraction,
            });
        }

        while let Some(entry) = self.heap.pop() {
            if entry.priority > self.bound_m {
                break;
            }
            let label = self.labels[entry.label as usize];
            if self.best.get(&(label.edge, label.state)) != Some(&entry.label) {
                continue;
            }
            let node = g.edge_target[label.edge as usize];
            let u_turn = |next: u32| {
                if network.is_u_turn(label.edge, next) {
                    params.u_turn_m
                } else {
                    0.0
                }
            };
            for (at, target) in targets.iter().enumerate() {
                if target.source != node || network.restricted(label.state, label.edge, target.edge)
                {
                    continue;
                }
                let distance_m = label.distance_m + target.fraction * target.length_m;
                let penalty_m = label.penalty_m + u_turn(target.edge);
                self.reach(
                    params,
                    at,
                    target,
                    distance_m,
                    penalty_m,
                    Via::Label(entry.label),
                );
            }
            for next in g.first_out[node as usize]..g.first_out[node as usize + 1] {
                if !network.accessible(next) || network.restricted(label.state, label.edge, next) {
                    continue;
                }
                self.push(RouteLabel {
                    edge: next,
                    state: network.automata.forward_next(&g, label.state, next),
                    distance_m: label.distance_m + network.length_m(next),
                    penalty_m: label.penalty_m + u_turn(next),
                    pred: entry.label,
                    start_fraction: 0.0,
                });
            }
        }
    }

    fn push(&mut self, label: RouteLabel) {
        let priority = label.distance_m + label.penalty_m;
        if priority > self.bound_m || label.distance_m > self.max_route_m {
            return;
        }
        let key = (label.edge, label.state);
        if let Some(existing) = self.best.get(&key) {
            let existing = self.labels[*existing as usize];
            if existing.distance_m + existing.penalty_m <= priority {
                return;
            }
        }
        let index = self.labels.len() as u32;
        self.labels.push(label);
        self.best.insert(key, index);
        self.heap.push(QueueEntry {
            priority,
            label: index,
        });
    }

    fn reach(
        &mut self,
        params: &Params,
        at: usize,
        target: &Target,
        distance_m: f64,
        penalty_m: f64,
        via: Via,
    ) {
        if distance_m + penalty_m > target.bound_m || distance_m > target.max_route_m {
            return;
        }
        let cost = ((distance_m - target.great_circle_m).abs() + penalty_m) / params.beta_m;
        if self.reached[at].is_none_or(|reached| cost < reached.cost) {
            self.reached[at] = Some(Reach { cost, via });
        }
    }

    /// Appends the edges of the route `via` (ending on `target_edge`) to
    /// `pool`; returns where it starts on its first edge.
    fn path(&self, via: Via, target_edge: u32, pool: &mut Vec<u32>) -> f64 {
        match via {
            Via::Direct {
                edge,
                start_fraction,
            } => {
                pool.push(edge);
                start_fraction
            }
            Via::Label(label) => {
                let start = pool.len();
                let mut start_fraction = 0.0;
                let mut current = label;
                while current != NO_LABEL {
                    let label = self.labels[current as usize];
                    pool.push(label.edge);
                    start_fraction = label.start_fraction;
                    current = label.pred;
                }
                pool[start..].reverse();
                pool.push(target_edge);
                start_fraction
            }
        }
    }
}

/// The decoded trace, for reconstructing matches.
struct Trace<'a, 'g> {
    g: SearchGraph<'g>,
    steps: &'a [Step],
    candidates: &'a [Candidate],
    states: &'a [State],
    pool: &'a [u32],
}

struct Assembled {
    samples: Vec<MatchedTraceSample>,
    sub_paths: Vec<MapMatchSubPath>,
    edge_ids: Vec<u64>,
}

impl Trace<'_, '_> {
    /// The match ending at candidate `terminal`: its samples and sub-paths.
    fn assemble(&self, terminal: usize, with_geometry: bool) -> Assembled {
        let mut path = Vec::new();
        let mut current = terminal as u32;
        while current != NO_STATE {
            path.push(current as usize);
            current = self.states[current as usize].back;
        }
        path.reverse();

        let mut samples = Vec::with_capacity(path.len());
        let mut sub_paths = Vec::new();
        let mut builder: Option<SubPathBuilder<'_>> = None;
        for index in path {
            let state = &self.states[index];
            let candidate = &self.candidates[index];
            let step = &self.steps[candidate.step];
            match (state.link, builder.as_mut()) {
                (Link::Transition, Some(builder)) => builder.follow(
                    &self.pool[state.path.start as usize..state.path.end as usize],
                    state.start_fraction,
                    candidate.fraction,
                    step.sample_index,
                ),
                (link, _) => {
                    sub_paths.extend(builder.take().map(SubPathBuilder::finish));
                    let reason = match link {
                        Link::Break(reason) => Some(reason),
                        _ => None,
                    };
                    builder = Some(SubPathBuilder::new(
                        self.g,
                        candidate,
                        step.sample_index,
                        reason,
                        with_geometry,
                    ));
                }
            }
            samples.push(self.matched_sample(candidate, step, sub_paths.len()));
        }
        sub_paths.extend(builder.map(SubPathBuilder::finish));
        let edge_ids = compact_edge_sequence(
            sub_paths
                .iter()
                .flat_map(|sub_path| sub_path.edge_ids.iter().copied()),
        );
        Assembled {
            samples,
            sub_paths,
            edge_ids,
        }
    }

    fn matched_sample(
        &self,
        candidate: &Candidate,
        step: &Step,
        sub_path_index: usize,
    ) -> MatchedTraceSample {
        let distance_score = (-candidate.distance_cost).exp();
        let heading_score = (-candidate.heading_cost).exp();
        let total_score = distance_score * heading_score;
        let max_distance_m = f64::from(step.max_distance_m).max(f64::EPSILON);
        MatchedTraceSample {
            sample_index: step.sample_index,
            raw_position: step.position,
            matched_edge_id: self.g.edge_id(candidate.edge as usize),
            snapped_position: candidate.point,
            distance_meters: candidate.distance_m,
            heading_delta_degrees: candidate.heading_delta_deg,
            confidence: (1.0 - candidate.distance_m / max_distance_m).clamp(0.0, 1.0),
            score: total_score,
            score_breakdown: CorrelationScoreBreakdown {
                distance_score,
                heading_score,
                total_score,
                ..CorrelationScoreBreakdown::default()
            },
            sub_path_index,
            fraction_along_edge: candidate.fraction,
        }
    }
}

/// Builds one sub-path from its first candidate and the transitions after.
struct SubPathBuilder<'g> {
    g: SearchGraph<'g>,
    with_geometry: bool,
    edges: Vec<u32>,
    /// The furthest fraction reached on the last edge (jitter backwards
    /// along it does not move the path).
    cursor: f64,
    geometry: Vec<GeoPoint>,
    distance_m: f64,
    shape: Vec<GeoPoint>,
    first_sample_index: usize,
    last_sample_index: usize,
    matched_sample_count: usize,
    break_reason: Option<&'static str>,
}

impl<'g> SubPathBuilder<'g> {
    fn new(
        g: SearchGraph<'g>,
        candidate: &Candidate,
        sample_index: usize,
        break_reason: Option<&'static str>,
        with_geometry: bool,
    ) -> Self {
        Self {
            g,
            with_geometry,
            edges: vec![candidate.edge],
            cursor: candidate.fraction,
            geometry: if with_geometry {
                vec![candidate.point]
            } else {
                Vec::new()
            },
            distance_m: 0.0,
            shape: Vec::new(),
            first_sample_index: sample_index,
            last_sample_index: sample_index,
            matched_sample_count: 1,
            break_reason,
        }
    }

    fn follow(
        &mut self,
        edges: &[u32],
        start_fraction: f64,
        end_fraction: f64,
        sample_index: usize,
    ) {
        let last = edges.len().saturating_sub(1);
        for (at, edge) in edges.iter().enumerate() {
            let from = if at == 0 { start_fraction } else { 0.0 };
            let to = if at == last { end_fraction } else { 1.0 };
            self.piece(*edge, from, to);
        }
        self.last_sample_index = sample_index;
        self.matched_sample_count += 1;
    }

    fn piece(&mut self, edge: u32, from: f64, to: f64) {
        let from = if self.edges.last() == Some(&edge) {
            from.max(self.cursor)
        } else {
            self.edges.push(edge);
            self.cursor = from;
            from
        };
        if to <= from {
            return;
        }
        self.distance_m += (to - from) * f64::from(self.g.edge_length[edge as usize]);
        self.cursor = to;
        if self.with_geometry {
            self.g.edge_shape(edge, &mut self.shape);
            append_clipped(&self.shape, from, to, &mut self.geometry);
        }
    }

    fn finish(self) -> MapMatchSubPath {
        MapMatchSubPath {
            first_sample_index: self.first_sample_index,
            last_sample_index: self.last_sample_index,
            matched_sample_count: self.matched_sample_count,
            edge_ids: self
                .edges
                .iter()
                .map(|edge| self.g.edge_id(*edge as usize))
                .collect(),
            geometry: self.geometry,
            distance_meters: self.distance_m,
            break_reason: self.break_reason.map(str::to_string),
        }
    }
}

/// Appends the stretch of `shape` between fractions `from` and `to` (of its
/// length) to `out`, skipping a first point `out` already ends with.
fn append_clipped(shape: &[GeoPoint], from: f64, to: f64, out: &mut Vec<GeoPoint>) {
    let total_m = shape
        .windows(2)
        .map(|segment| segment[0].distance_to(segment[1]))
        .sum::<f64>();
    if shape.len() < 2 || total_m <= f64::EPSILON {
        return;
    }
    let (start_m, end_m) = (from.clamp(0.0, 1.0) * total_m, to.clamp(0.0, 1.0) * total_m);
    push_point(out, point_at_distance(shape, start_m));
    let mut cumulative_m = 0.0;
    for segment in shape.windows(2) {
        cumulative_m += segment[0].distance_to(segment[1]);
        if cumulative_m > start_m && cumulative_m < end_m {
            push_point(out, segment[1]);
        }
    }
    push_point(out, point_at_distance(shape, end_m));
}

fn push_point(out: &mut Vec<GeoPoint>, point: GeoPoint) {
    if out.last().is_none_or(|last| last.distance_to(point) >= 0.1) {
        out.push(point);
    }
}

fn point_at_distance(points: &[GeoPoint], distance_m: f64) -> GeoPoint {
    let mut cumulative_m = 0.0;
    for segment in points.windows(2) {
        let segment_m = segment[0].distance_to(segment[1]);
        if segment_m <= f64::EPSILON {
            continue;
        }
        if cumulative_m + segment_m >= distance_m {
            let t = ((distance_m - cumulative_m) / segment_m).clamp(0.0, 1.0);
            return GeoPoint::new(
                segment[0].lat + (segment[1].lat - segment[0].lat) * t,
                segment[0].lng + (segment[1].lng - segment[0].lng) * t,
            );
        }
        cumulative_m += segment_m;
    }
    points.last().copied().unwrap_or(GeoPoint::new(0.0, 0.0))
}

fn compact_edge_sequence(edge_ids: impl Iterator<Item = u64>) -> Vec<u64> {
    let mut compacted = Vec::new();
    for edge_id in edge_ids {
        if compacted.last().copied() != Some(edge_id) {
            compacted.push(edge_id);
        }
    }
    compacted
}

fn default_trace_max_correlation_distance_m() -> u32 {
    DEFAULT_TRACE_MAX_CORRELATION_DISTANCE_M
}

fn default_trace_candidate_limit() -> usize {
    DEFAULT_TRACE_CANDIDATE_LIMIT
}

fn default_heading_weight() -> f64 {
    DEFAULT_HEADING_WEIGHT
}

fn default_transition_weight() -> f64 {
    DEFAULT_TRANSITION_WEIGHT
}

fn default_backtrack_penalty() -> f64 {
    DEFAULT_BACKTRACK_PENALTY
}

fn default_trace_alternative_count() -> usize {
    DEFAULT_TRACE_ALTERNATIVE_COUNT
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::RoutingEngine;
    use petgraph::visit::EdgeRef;
    use wayfarer_package::{DirectionalAccess, RoadType};
    use wayfarer_road_graph::{RoadGraph, RoadGraphBuilder, RoutableWay, enhance_road_graph};

    fn way(id: i64, lat: f64, name: &str) -> RoutableWay {
        RoutableWay {
            osm_way_id: id,
            points: vec![(lat, 20.0), (lat, 20.003)],
            source_node_ids: Vec::new(),
            road_type: RoadType::Primary,
            directional_access: DirectionalAccess::Both,
            name: Some(name.into()),
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
            source_lineage_ids: Vec::new(),
            coverage_id: None,
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
        }
    }

    fn parallel_road_graph() -> RoadGraph {
        let mut builder = RoadGraphBuilder::new("map-matching-test");
        builder.push_way(way(1, 50.0, "South Road"));
        builder.push_way(way(2, 50.00025, "North Road"));
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);
        graph
    }

    fn sample(lat: f64, lng: f64, heading: Option<f64>) -> NavigationPositionSample {
        NavigationPositionSample {
            position: GeoPoint::new(lat, lng),
            heading_degrees: heading,
            speed_meters_per_second: None,
            sampled_at_ms: None,
            accuracy_meters: Some(8.0),
            provider: Some("test_trace".to_string()),
        }
    }

    #[test]
    fn trace_match_uses_heading_to_choose_directed_edge() {
        let graph = parallel_road_graph();
        let engine = crate::test_support::engine(&graph);
        let samples = vec![
            sample(50.0, 20.0004, Some(90.0)),
            sample(50.0, 20.0014, Some(90.0)),
            sample(50.0, 20.0024, Some(90.0)),
        ];

        let result = match_navigation_trace(
            engine.route_graph_view(),
            &samples,
            MobilityProfile::Wheeled,
            MapMatchOptions::default(),
        )
        .expect("trace match");

        assert_eq!(result.sample_count, 3);
        assert_eq!(result.matched_sample_count, 3);
        assert_eq!(result.rejected_sample_count, 0);
        assert_eq!(
            result.matched_edge_ids.len(),
            1,
            "samples along the same directed edge should compact to one edge id"
        );
        assert!(result.samples.iter().all(|sample| {
            sample
                .heading_delta_degrees
                .is_some_and(|delta| delta <= 45)
        }));
        assert!(result.samples.iter().all(|sample| {
            sample.score == sample.score_breakdown.total_score
                && sample.score_breakdown.heading_score > 0.0
        }));
    }

    #[test]
    fn trace_match_keeps_noisy_samples_on_nearest_parallel_road() {
        let graph = parallel_road_graph();
        let engine = crate::test_support::engine(&graph);
        let samples = vec![
            sample(50.00023, 20.0004, Some(90.0)),
            sample(50.00028, 20.0014, Some(90.0)),
            sample(50.00022, 20.0024, Some(90.0)),
        ];

        let result = match_navigation_trace(
            engine.route_graph_view(),
            &samples,
            MobilityProfile::Wheeled,
            MapMatchOptions::default(),
        )
        .expect("trace match");

        assert_eq!(result.matched_sample_count, 3);
        assert_eq!(result.matched_edge_ids.len(), 1);
        for matched in &result.samples {
            assert!(
                (matched.snapped_position.lat - 50.00025).abs() < 0.00002,
                "sample {} should snap to the north road, got {}",
                matched.sample_index,
                matched.snapped_position.lat
            );
        }
    }

    #[test]
    fn trace_match_reports_bounded_alternative_edge_sequences() {
        let graph = parallel_road_graph();
        let engine = crate::test_support::engine(&graph);
        let samples = vec![
            sample(50.00012, 20.0004, Some(90.0)),
            sample(50.00013, 20.0014, Some(90.0)),
            sample(50.00012, 20.0024, Some(90.0)),
        ];
        let options = MapMatchOptions {
            alternative_count: 2,
            candidate_limit: 8,
            ..MapMatchOptions::default()
        };

        let result = match_navigation_trace(
            engine.route_graph_view(),
            &samples,
            MobilityProfile::Wheeled,
            options,
        )
        .expect("trace match");

        assert!(!result.alternatives.is_empty());
        assert!(result.alternatives.len() <= 2);
        let mut previous_cost_delta = 0.0;
        for (index, alternative) in result.alternatives.iter().enumerate() {
            assert_eq!(alternative.rank, index + 1);
            assert_ne!(alternative.matched_edge_ids, result.matched_edge_ids);
            assert_eq!(
                alternative.matched_sample_count,
                result.matched_sample_count
            );
            assert!(alternative.total_cost >= result.total_cost);
            assert!(alternative.cost_delta >= 0.0);
            assert!(alternative.cost_delta >= previous_cost_delta);
            previous_cost_delta = alternative.cost_delta;
        }
    }

    #[test]
    fn trace_match_keeps_samples_before_a_mid_trace_rejection() {
        let graph = parallel_road_graph();
        let engine = crate::test_support::engine(&graph);
        let samples = vec![
            sample(50.0, 20.0004, Some(90.0)),
            sample(50.0, 20.0010, Some(90.0)),
            sample(51.0, 20.0014, Some(90.0)),
            sample(50.0, 20.0018, Some(90.0)),
            sample(50.0, 20.0024, Some(90.0)),
        ];

        let result = match_navigation_trace(
            engine.route_graph_view(),
            &samples,
            MobilityProfile::Wheeled,
            MapMatchOptions::default(),
        )
        .expect("trace match");

        assert_eq!(result.sample_count, 5);
        assert_eq!(result.rejected_sample_count, 1);
        assert_eq!(result.rejected_samples[0].sample_index, 2);
        assert_eq!(result.matched_sample_count, 4);
        assert_eq!(
            result
                .samples
                .iter()
                .map(|sample| sample.sample_index)
                .collect::<Vec<_>>(),
            vec![0, 1, 3, 4],
            "samples before the rejected one must stay in the match"
        );
        assert_eq!(result.matched_edge_ids.len(), 1);
        for alternative in &result.alternatives {
            assert_eq!(alternative.matched_sample_count, 4);
        }
    }

    #[test]
    fn trace_match_survives_rejected_first_and_consecutive_samples() {
        let graph = parallel_road_graph();
        let engine = crate::test_support::engine(&graph);
        let samples = vec![
            sample(51.0, 20.0004, Some(90.0)),
            sample(50.0, 20.0008, Some(90.0)),
            sample(51.0, 20.0012, Some(90.0)),
            sample(51.0, 20.0016, Some(90.0)),
            sample(50.0, 20.0020, Some(90.0)),
        ];

        let result = match_navigation_trace(
            engine.route_graph_view(),
            &samples,
            MobilityProfile::Wheeled,
            MapMatchOptions::default(),
        )
        .expect("trace match");

        assert_eq!(result.rejected_sample_count, 3);
        assert_eq!(
            result
                .samples
                .iter()
                .map(|sample| sample.sample_index)
                .collect::<Vec<_>>(),
            vec![1, 4]
        );
    }

    #[test]
    fn trace_match_reports_rejected_samples() {
        let graph = parallel_road_graph();
        let engine = crate::test_support::engine(&graph);
        let samples = vec![
            sample(50.0, 20.0004, Some(90.0)),
            sample(50.0, 20.0014, Some(90.0)),
            sample(51.0, 20.0014, Some(90.0)),
        ];

        let result = match_navigation_trace(
            engine.route_graph_view(),
            &samples,
            MobilityProfile::Wheeled,
            MapMatchOptions::default(),
        )
        .expect("trace match");

        assert_eq!(result.sample_count, 3);
        assert_eq!(result.matched_sample_count, 2);
        assert_eq!(result.rejected_sample_count, 1);
        assert_eq!(result.rejected_samples.len(), 1);
        assert_eq!(result.rejected_samples[0].sample_index, 2);
        assert_eq!(result.rejected_samples[0].reason, "no_candidates");
        assert_eq!(
            result.rejected_samples[0].max_correlation_distance_meters,
            DEFAULT_TRACE_MAX_CORRELATION_DISTANCE_M
        );
        assert_eq!(
            result.rejected_samples[0].candidate_limit,
            DEFAULT_TRACE_CANDIDATE_LIMIT
        );
    }

    /// Metres north as degrees of latitude.
    const LAT_PER_M: f64 = 1.0 / 111_320.0;
    /// About 8 m of longitude at 50 degrees north (8 m/s at 1 Hz).
    const STEP_LNG: f64 = 0.000112;

    fn road(id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
        RoutableWay {
            points,
            ..way(id, 0.0, "Road")
        }
    }

    fn one_way(id: i64, points: Vec<(f64, f64)>) -> RoutableWay {
        RoutableWay {
            directional_access: DirectionalAccess::Forward,
            ..road(id, points)
        }
    }

    fn graph_of(ways: Vec<RoutableWay>) -> RoadGraph {
        let mut builder = RoadGraphBuilder::new("map-matching-test");
        for way in ways {
            builder.push_way(way);
        }
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);
        graph
    }

    /// A fix without heading or accuracy (so sigma is the default 5 m).
    fn fix(lat: f64, lng: f64, at_s: i64) -> NavigationPositionSample {
        NavigationPositionSample {
            position: GeoPoint::new(lat, lng),
            heading_degrees: None,
            speed_meters_per_second: None,
            sampled_at_ms: Some(at_s * 1000),
            accuracy_meters: None,
            provider: Some("test_trace".to_string()),
        }
    }

    fn match_trace(engine: &RoutingEngine, samples: &[NavigationPositionSample]) -> MapMatchResult {
        match_navigation_trace(
            engine.route_graph_view(),
            samples,
            MobilityProfile::Wheeled,
            MapMatchOptions::default(),
        )
        .expect("trace match")
    }

    /// The directed edges running along latitude `lat`, eastward or
    /// westward, ordered in that direction.
    fn edges_along(graph: &RoadGraph, lat: f64, eastward: bool) -> Vec<u64> {
        let mut edges = graph
            .graph
            .edge_references()
            .filter_map(|edge| {
                let (source, target) = (&graph.graph[edge.source()], &graph.graph[edge.target()]);
                ((source.lat - lat).abs() < 1e-9
                    && (target.lat - lat).abs() < 1e-9
                    && (target.lng > source.lng) == eastward)
                    .then_some((source.lng, edge.weight().edge_id))
            })
            .collect::<Vec<_>>();
        edges.sort_by(|a, b| a.0.total_cmp(&b.0));
        if !eastward {
            edges.reverse();
        }
        edges.into_iter().map(|(_, edge_id)| edge_id).collect()
    }

    fn assert_connected(engine: &RoutingEngine, edge_ids: &[u64]) {
        let view = engine.route_graph_view();
        for pair in edge_ids.windows(2) {
            let (from, to) = (
                view.edge_by_id(pair[0]).expect("edge"),
                view.edge_by_id(pair[1]).expect("edge"),
            );
            assert_eq!(
                from.target.id, to.source.id,
                "edges {} and {} of {edge_ids:?} do not connect",
                pair[0], pair[1]
            );
        }
    }

    #[test]
    fn trace_match_stays_on_the_connected_road_beside_a_parallel_one() {
        // Main road A (four segments) and road B 22 m north, joined only at
        // their ends. Samples 10..19 drift 13 m north (9 m from B).
        let lat_b = 50.0002;
        let mut ways = (0..4)
            .map(|at| {
                let lng = 20.0 + 0.001 * f64::from(at);
                road(i64::from(at) + 1, vec![(50.0, lng), (50.0, lng + 0.001)])
            })
            .collect::<Vec<_>>();
        ways.push(road(5, vec![(lat_b, 20.0), (lat_b, 20.004)]));
        ways.push(road(6, vec![(50.0, 20.0), (lat_b, 20.0)]));
        ways.push(road(7, vec![(50.0, 20.004), (lat_b, 20.004)]));
        let graph = graph_of(ways);
        let engine = crate::test_support::engine(&graph);
        let samples = (0..30)
            .map(|at| {
                let drift = if (10..19).contains(&at) { 13.0 } else { 0.0 };
                fix(
                    50.0 + drift * LAT_PER_M,
                    20.0003 + STEP_LNG * f64::from(at),
                    i64::from(at),
                )
            })
            .collect::<Vec<_>>();

        let result = match_trace(&engine, &samples);

        assert_eq!(result.matched_sample_count, 30);
        assert_eq!(result.sub_paths.len(), 1);
        assert_eq!(result.matched_edge_ids, edges_along(&graph, 50.0, true));
        assert_connected(&engine, &result.matched_edge_ids);
        assert!(
            result
                .samples
                .iter()
                .all(|sample| (sample.snapped_position.lat - 50.0).abs() < 1e-7),
            "every sample should snap to road A"
        );
        let sub_path = &result.sub_paths[0];
        assert!(
            sub_path
                .geometry
                .iter()
                .all(|point| (point.lat - 50.0).abs() < 1e-7)
        );
        assert!((sub_path.distance_meters - 29.0 * 8.0).abs() < 5.0);
    }

    #[test]
    fn trace_match_keeps_each_carriageway_direction() {
        // Eastbound carriageway on 50.0, westbound 16.7 m north, joined at
        // the ends. No headings: only the transitions tell them apart.
        let lat_north = 50.00015;
        let graph = graph_of(vec![
            one_way(1, vec![(50.0, 20.0), (50.0, 20.002)]),
            one_way(2, vec![(50.0, 20.002), (50.0, 20.004)]),
            one_way(3, vec![(lat_north, 20.004), (lat_north, 20.002)]),
            one_way(4, vec![(lat_north, 20.002), (lat_north, 20.0)]),
            road(5, vec![(50.0, 20.0), (lat_north, 20.0)]),
            road(6, vec![(50.0, 20.004), (lat_north, 20.004)]),
        ]);
        let engine = crate::test_support::engine(&graph);
        // Samples 8..16 lean 10 m towards the other carriageway.
        let offset = |at: i32| {
            if (8..16).contains(&at) {
                10.0
            } else {
                [0.0, 2.0, -2.0, 1.0][at as usize % 4]
            }
        };
        let eastbound = (0..30)
            .map(|at| {
                fix(
                    50.0 + offset(at) * LAT_PER_M,
                    20.0003 + STEP_LNG * f64::from(at),
                    i64::from(at),
                )
            })
            .collect::<Vec<_>>();
        let westbound = (0..30)
            .map(|at| {
                fix(
                    lat_north - offset(at) * LAT_PER_M,
                    20.0037 - STEP_LNG * f64::from(at),
                    i64::from(at),
                )
            })
            .collect::<Vec<_>>();

        for (samples, lat, eastward) in [(eastbound, 50.0, true), (westbound, lat_north, false)] {
            let result = match_trace(&engine, &samples);
            assert_eq!(result.matched_sample_count, 30);
            assert_eq!(result.sub_paths.len(), 1);
            assert_eq!(result.matched_edge_ids, edges_along(&graph, lat, eastward));
            assert!(
                result
                    .samples
                    .iter()
                    .all(|sample| (sample.snapped_position.lat - lat).abs() < 1e-7),
                "every sample should match the carriageway driven"
            );
        }
    }

    #[test]
    fn trace_match_follows_a_u_turn_where_the_trace_reverses() {
        // A two-way road; the trace drives east at 5 m/s, turns around at
        // 20.00253 (mid-edge) and drives back west.
        let graph = graph_of(vec![
            road(1, vec![(50.0, 20.0), (50.0, 20.002)]),
            road(2, vec![(50.0, 20.002), (50.0, 20.004)]),
        ]);
        let engine = crate::test_support::engine(&graph);
        let step = 0.00007;
        let jitter = |at: usize| [0.0, 1.5, -1.0, 0.5][at % 4] * LAT_PER_M;
        let mut lngs = (0..30)
            .map(|at| 20.0005 + step * at as f64)
            .collect::<Vec<_>>();
        let turn = *lngs.last().unwrap();
        lngs.extend((1..=25).map(|at| turn - step * at as f64));
        let samples = lngs
            .iter()
            .enumerate()
            .map(|(at, lng)| fix(50.0 + jitter(at), *lng, at as i64))
            .collect::<Vec<_>>();

        let result = match_trace(&engine, &samples);

        let east = edges_along(&graph, 50.0, true);
        let west = edges_along(&graph, 50.0, false);
        assert_eq!(result.sub_paths.len(), 1);
        assert_eq!(result.rejected_sample_count, 0);
        assert_eq!(
            result.matched_edge_ids,
            vec![east[0], east[1], west[0], west[1]]
        );
        assert_connected(&engine, &result.matched_edge_ids);
        for sample in &result.samples {
            let lng = samples[sample.sample_index].position.lng;
            let expected = if sample.sample_index < 30 {
                &east
            } else {
                &west
            };
            if lng < 20.0022 {
                assert!(
                    expected.contains(&sample.matched_edge_id),
                    "sample {} should match the direction driven",
                    sample.sample_index
                );
            }
        }
        // The path turns where the trace does, not at the end of the road.
        let furthest = result.sub_paths[0]
            .geometry
            .iter()
            .map(|point| point.lng)
            .fold(f64::MIN, f64::max);
        assert!((furthest - turn).abs() < 0.0001, "turned at {furthest}");
    }

    #[test]
    fn trace_match_flips_direction_only_when_the_trace_reverses() {
        // Noisy samples straddling a two-way road while driving east: the
        // path stays on the eastbound edge (no free flip to the westbound).
        let graph = graph_of(vec![road(1, vec![(50.0, 20.0), (50.0, 20.004)])]);
        let engine = crate::test_support::engine(&graph);
        let samples = (0..30)
            .map(|at| {
                let along = [0.0, -3.0, 2.0, -4.0, 1.0][at as usize % 5] * 0.0000140;
                let across = [3.0, -3.0, 4.0, -2.0][at as usize % 4] * LAT_PER_M;
                fix(
                    50.0 + across,
                    20.0003 + STEP_LNG * f64::from(at) + along,
                    i64::from(at),
                )
            })
            .collect::<Vec<_>>();

        let result = match_trace(&engine, &samples);

        assert_eq!(result.matched_edge_ids, edges_along(&graph, 50.0, true));
        assert_eq!(result.rejected_sample_count, 0);
    }

    #[test]
    fn trace_match_breaks_the_trace_at_a_time_gap() {
        let graph = graph_of(vec![
            road(1, vec![(50.0, 20.0), (50.0, 20.002)]),
            road(2, vec![(50.0, 20.002), (50.0, 20.004)]),
        ]);
        let engine = crate::test_support::engine(&graph);
        let samples = (0..20)
            .map(|at| {
                let at_s = if at < 10 { at } else { 300 + at };
                fix(50.0, 20.0003 + STEP_LNG * f64::from(at), i64::from(at_s))
            })
            .collect::<Vec<_>>();

        let result = match_trace(&engine, &samples);

        assert_eq!(result.matched_sample_count, 20);
        assert_eq!(result.sub_paths.len(), 2);
        let (first, second) = (&result.sub_paths[0], &result.sub_paths[1]);
        assert_eq!((first.first_sample_index, first.last_sample_index), (0, 9));
        assert_eq!(
            (second.first_sample_index, second.last_sample_index),
            (10, 19)
        );
        assert_eq!(first.break_reason, None);
        assert_eq!(second.break_reason.as_deref(), Some(TIME_GAP));
        assert_eq!(first.matched_sample_count + second.matched_sample_count, 20);
        assert!(
            result.samples[..10]
                .iter()
                .all(|sample| sample.sub_path_index == 0)
        );
        assert!(
            result.samples[10..]
                .iter()
                .all(|sample| sample.sub_path_index == 1)
        );
    }

    #[test]
    fn trace_match_stops_when_cancelled() {
        let graph = graph_of(vec![road(1, vec![(50.0, 20.0), (50.0, 20.002)])]);
        let engine = crate::test_support::engine(&graph);
        let samples = (0..5)
            .map(|at| fix(50.0, 20.0003 + STEP_LNG * f64::from(at), i64::from(at)))
            .collect::<Vec<_>>();
        let token = crate::CancelToken::new();
        let cancellation = RouteCancellation::default().with_token(token.clone());
        let options = MapMatchOptions::default();
        let profile = MobilityProfile::Wheeled;

        let view = engine.route_graph_view();
        let live =
            match_navigation_trace_cancellable(view, &samples, profile, options, &cancellation);
        assert!(live.is_ok(), "{live:?}");

        token.cancel();
        let cancelled =
            match_navigation_trace_cancellable(view, &samples, profile, options, &cancellation);
        assert_eq!(cancelled, Err(CANCELLED.to_string()));
        assert!(cancellation.check().is_err());
    }

    #[test]
    fn trace_match_breaks_the_trace_where_no_route_connects_the_samples() {
        // The parallel test roads do not connect: a trace jumping from one to
        // the other is two sub-paths.
        let graph = parallel_road_graph();
        let engine = crate::test_support::engine(&graph);
        let samples = (0..10)
            .map(|at| {
                let lat = if at < 5 { 50.0 } else { 50.00025 };
                sample(lat, 20.0003 + 0.0002 * f64::from(at), Some(90.0))
            })
            .collect::<Vec<_>>();

        let result = match_trace(&engine, &samples);

        assert_eq!(result.matched_sample_count, 10);
        assert_eq!(result.sub_paths.len(), 2);
        assert_eq!(result.sub_paths[1].first_sample_index, 5);
        assert_eq!(
            result.sub_paths[1].break_reason.as_deref(),
            Some(NO_TRANSITION)
        );
        assert_eq!(
            result.sub_paths[0].edge_ids,
            edges_along(&graph, 50.0, true)
        );
        assert_eq!(
            result.sub_paths[1].edge_ids,
            edges_along(&graph, 50.00025, true)
        );
    }

    #[test]
    fn trace_match_skips_outliers_instead_of_detouring_to_them() {
        // Main road with a side street north at 20.002. Sample 15 jumps 44 m
        // up the side street, sample 22 40 m south of the road (no road).
        let mut ways = (0..4)
            .map(|at| {
                let lng = 20.0 + 0.001 * f64::from(at);
                road(i64::from(at) + 1, vec![(50.0, lng), (50.0, lng + 0.001)])
            })
            .collect::<Vec<_>>();
        ways.push(road(5, vec![(50.0, 20.002), (50.001, 20.002)]));
        let graph = graph_of(ways);
        let engine = crate::test_support::engine(&graph);
        let mut samples = (0..30)
            .map(|at| {
                let jitter = [0.0, 1.0, -1.0][at as usize % 3] * LAT_PER_M;
                fix(
                    50.0 + jitter,
                    20.0003 + STEP_LNG * f64::from(at),
                    i64::from(at),
                )
            })
            .collect::<Vec<_>>();
        samples[15].position = GeoPoint::new(50.0004, 20.00201);
        samples[22].position.lat = 50.0 - 40.0 * LAT_PER_M;

        let result = match_trace(&engine, &samples);

        assert_eq!(
            result
                .rejected_samples
                .iter()
                .map(|sample| (sample.sample_index, sample.reason.as_str()))
                .collect::<Vec<_>>(),
            vec![(15, OUTLIER), (22, OUTLIER)]
        );
        assert_eq!(result.matched_sample_count, 28);
        assert_eq!(result.sub_paths.len(), 1);
        assert_eq!(result.matched_edge_ids, edges_along(&graph, 50.0, true));
    }
}

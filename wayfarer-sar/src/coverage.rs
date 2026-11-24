//! Track coverage (S6): what teams searched, and the probability after it.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use wayfarer::{
    MapMatchRequest, MobilityProfile, NavigationPositionSample, RouteError, RoutingEngine,
};

use crate::reach::{EdgeProbability, Edges, ProbabilityMap};
use crate::teams::{SearchSegment, unique};
use crate::{SarError, unit_interval};

/// The most tracks of one coverage request.
pub const MAX_TRACKS: usize = 256;

/// One team's GPS track of a search assignment.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct TeamTrack {
    pub team_id: String,
    pub samples: Vec<NavigationPositionSample>,
    #[serde(default = "on_foot")]
    pub profile: MobilityProfile,
    /// The probability one pass along a path detects the person on it, in
    /// [0, 1]; by default the request's `default_pod`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pod: Option<f64>,
}

/// Searched tracks against a probability map. See
/// `docs/design/sar-planning.md` 2.6.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct CoverageRequest {
    /// 1 to 256 tracks.
    pub tracks: Vec<TeamTrack>,
    /// The probability before the search (`ReachResult::probability`).
    pub probability: ProbabilityMap,
    /// Segments to report coverage for (ids unique); may be empty.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub segments: Vec<SearchSegment>,
    /// Per-pass detection probability of tracks that set none (default 0.8,
    /// a hasty search along a path: a starting value, not calibrated).
    #[serde(default = "default_pod")]
    pub default_pod: f64,
}

impl CoverageRequest {
    pub fn new(tracks: Vec<TeamTrack>, probability: ProbabilityMap) -> Self {
        Self {
            tracks,
            probability,
            segments: Vec::new(),
            default_pod: default_pod(),
        }
    }

    fn validate(&self) -> Result<(), SarError> {
        if self.tracks.is_empty() || self.tracks.len() > MAX_TRACKS {
            return Err(SarError::invalid("coverage needs 1 to 256 tracks"));
        }
        if !unit_interval(self.default_pod)
            || self
                .tracks
                .iter()
                .any(|track| track.pod.is_some_and(|pod| !unit_interval(pod)))
        {
            return Err(SarError::invalid("pod values must be in [0, 1]"));
        }
        unique(
            self.segments.iter().map(|segment| segment.id.as_str()),
            "segment",
        )?;
        for segment in &self.segments {
            segment.validate()?;
        }
        self.probability.validate()
    }
}

fn on_foot() -> MobilityProfile {
    MobilityProfile::OnFoot
}
fn default_pod() -> f64 {
    0.8
}

/// An edge a team searched.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct CoveredEdge {
    /// The undirected key.
    pub edge_id: u64,
    /// Traversals over all tracks (out and back counts twice).
    pub passes: u32,
    /// `1 - product((1 - pod) ^ passes)` over the tracks.
    pub pod: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct TrackCoverage {
    pub team_id: String,
    pub matched_sample_count: usize,
    pub sample_count: usize,
    pub covered_length_m: f64,
    /// Why nothing matched, when nothing did.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub failure: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct SegmentCoverage {
    pub segment_id: String,
    pub network_length_m: f64,
    pub covered_length_m: f64,
    /// `covered_length_m / network_length_m` (0 for a segment with no path).
    pub coverage: f64,
    /// Probability-weighted detection: `sum(POA_e POD_e) / sum(POA_e)`
    /// (length-weighted where the segment holds no probability).
    pub pod: f64,
    pub probability_before: f64,
    pub probability_after: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct CoverageResult {
    pub tracks: Vec<TrackCoverage>,
    /// Sorted by edge key.
    pub covered_edges: Vec<CoveredEdge>,
    pub segments: Vec<SegmentCoverage>,
    /// The probability after an unsuccessful search (Bayes): searched edges
    /// lose, everything else (rest of world included) gains.
    pub probability: ProbabilityMap,
    /// `sum(POA_e POD_e)`: the chance the search so far would have found the
    /// person (cumulative probability of success).
    pub probability_of_success: f64,
}

/// Turns team tracks into coverage and updates the probability (see
/// [`CoverageRequest`]). A track that matches no path is reported, not an
/// error.
pub fn apply_coverage(
    engine: &RoutingEngine,
    request: &CoverageRequest,
) -> Result<CoverageResult, SarError> {
    request.validate()?;
    let mut edges = Edges::new(engine);
    // Per key: passes and the product of misses.
    let mut covered = BTreeMap::<u64, (u32, f64)>::new();
    let mut tracks = Vec::with_capacity(request.tracks.len());
    for track in &request.tracks {
        let pod = track.pod.unwrap_or(request.default_pod);
        let mut report = TrackCoverage {
            team_id: track.team_id.clone(),
            matched_sample_count: 0,
            sample_count: track.samples.len(),
            covered_length_m: 0.0,
            failure: None,
        };
        if track.samples.is_empty() {
            report.failure = Some("no samples".into());
            tracks.push(report);
            continue;
        }
        let matched =
            match engine.map_match(MapMatchRequest::new(track.samples.clone(), track.profile)) {
                Ok(matched) => matched,
                Err(error @ RouteError::Cancelled { .. }) => return Err(error.into()),
                Err(error) => {
                    report.failure = Some(error.to_string());
                    tracks.push(report);
                    continue;
                }
            };
        report.matched_sample_count = matched.matched_sample_count;
        let mut track_keys = std::collections::BTreeSet::new();
        for sub_path in &matched.sub_paths {
            let mut previous = None;
            for edge_id in &sub_path.edge_ids {
                if previous == Some(*edge_id) {
                    continue;
                }
                previous = Some(*edge_id);
                let key = edges.network().key(*edge_id);
                track_keys.insert(key);
                let entry = covered.entry(key).or_insert((0, 1.0));
                entry.0 += 1;
                entry.1 *= 1.0 - pod;
            }
        }
        report.covered_length_m = track_keys.iter().map(|key| edges.length(*key)).sum();
        tracks.push(report);
    }

    let pod_of = covered
        .iter()
        .map(|(key, (_, miss))| (*key, 1.0 - miss))
        .collect::<BTreeMap<_, _>>();
    let before = request.probability.by_edge();
    let success: f64 = before
        .iter()
        .map(|(key, probability)| probability * pod_of.get(key).copied().unwrap_or(0.0))
        .sum();
    let remaining = 1.0 - success;
    let scale = if remaining > 0.0 {
        1.0 / remaining
    } else {
        0.0
    };
    let probability = ProbabilityMap {
        edges: request
            .probability
            .edges
            .iter()
            .map(|edge| EdgeProbability {
                probability: edge.probability
                    * (1.0 - pod_of.get(&edge.edge_id).copied().unwrap_or(0.0))
                    * scale,
                ..edge.clone()
            })
            .filter(|edge| edge.probability > 0.0)
            .collect(),
        rest_of_world: request.probability.rest_of_world * scale,
        off_network: request.probability.off_network * scale,
    };
    let after = probability.by_edge();

    let mut segments = Vec::with_capacity(request.segments.len());
    for segment in &request.segments {
        let keys = edges.network().keys_in_polygon(&segment.polygon);
        let (mut length, mut covered_length) = (0.0, 0.0);
        let (mut poa, mut poa_pod, mut length_pod, mut poa_after) = (0.0, 0.0, 0.0, 0.0);
        for key in &keys {
            let edge_length = edges.length(*key);
            let pod = pod_of.get(key).copied().unwrap_or(0.0);
            let probability = before.get(key).copied().unwrap_or(0.0);
            length += edge_length;
            if pod > 0.0 {
                covered_length += edge_length;
            }
            poa += probability;
            poa_pod += probability * pod;
            length_pod += edge_length * pod;
            poa_after += after.get(key).copied().unwrap_or(0.0);
        }
        segments.push(SegmentCoverage {
            segment_id: segment.id.clone(),
            network_length_m: length,
            covered_length_m: covered_length,
            coverage: if length > 0.0 {
                covered_length / length
            } else {
                0.0
            },
            pod: if poa > 0.0 {
                poa_pod / poa
            } else if length > 0.0 {
                length_pod / length
            } else {
                0.0
            },
            probability_before: poa,
            probability_after: poa_after,
        });
    }

    Ok(CoverageResult {
        tracks,
        covered_edges: covered
            .into_iter()
            .map(|(edge_id, (passes, miss))| CoveredEdge {
                edge_id,
                passes,
                pod: 1.0 - miss,
            })
            .collect(),
        segments,
        probability,
        probability_of_success: success,
    })
}

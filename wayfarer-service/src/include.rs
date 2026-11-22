//! The `include` list of a route request: which optional parts of the
//! result to keep. Without one, a result carries every part, as before;
//! with one, only the core (geometry, distance, duration, cost, maneuvers,
//! edge ids, location matches, alternatives, summaries, explanations and
//! diagnostics) and the listed parts. The core is a small fraction of a
//! full result: edge attributions and the per-leg copies are most of it.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};
use wayfarer::{RouteAlternative, RouteAlternatives, RouteLegMode, RouteResult, RouteSegment};

/// An optional part of a route result, named in a request's `include`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub enum RouteResultPart {
    /// `segments`: each leg's full single-leg result. Left out, `segments`
    /// is empty and `segmentSummaries` has each leg's distance, duration and
    /// cost.
    Segments,
    /// The `legs` of each segment and alternative: their geometry, edges
    /// and shape segments split at waypoints.
    Legs,
    /// `edgeAttributions`, and every leg's `edges`.
    EdgeAttributions,
    /// The `costBreakdown` of every edge attribution that is kept.
    CostBreakdown,
    /// `shapeSegments`, and every leg's `shapeSegments`.
    ShapeSegments,
    /// `recosts`, and each segment's.
    Recosts,
}

/// One leg of a route in brief, for a result without `segments`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteSegmentSummary {
    pub mode: RouteLegMode,
    pub distance_m: f64,
    pub duration_s: f64,
    pub cost: f64,
}

impl RouteSegmentSummary {
    pub fn of(segment: &RouteSegment) -> Self {
        Self {
            mode: match segment {
                RouteSegment::Routed(_) => RouteLegMode::Routed,
                RouteSegment::Direct(_) => RouteLegMode::Direct,
            },
            distance_m: segment.distance_m(),
            duration_s: segment.duration_s(),
            cost: segment.cost(),
        }
    }
}

/// Empties the parts of `route` that `include` leaves out, and returns the
/// segment summaries when it leaves out `segments`. Edge attributions'
/// `costBreakdown`s stay in the Rust result; serialization leaves them out
/// ([`crate::RouteWorkflowResult`]).
pub(crate) fn strip(
    route: &mut RouteResult,
    include: &BTreeSet<RouteResultPart>,
) -> Option<Vec<RouteSegmentSummary>> {
    let keeps = |part| include.contains(&part);
    let summaries = (!keeps(RouteResultPart::Segments)).then(|| {
        let summaries = route.segments.iter().map(RouteSegmentSummary::of).collect();
        route.segments = Vec::new();
        summaries
    });
    if !keeps(RouteResultPart::EdgeAttributions) {
        route.edge_attributions = Vec::new();
    }
    if !keeps(RouteResultPart::ShapeSegments) {
        route.shape_segments = Vec::new();
    }
    if !keeps(RouteResultPart::Recosts) {
        route.recosts = Vec::new();
    }
    let alternative = |alternative: &mut RouteAlternative| {
        if !keeps(RouteResultPart::Legs) {
            alternative.legs = Vec::new();
        }
        for leg in &mut alternative.legs {
            if !keeps(RouteResultPart::EdgeAttributions) {
                leg.edges = Vec::new();
            }
            if !keeps(RouteResultPart::ShapeSegments) {
                leg.shape_segments = Vec::new();
            }
        }
    };
    for segment in &mut route.segments {
        let RouteSegment::Routed(leg_result) = segment else {
            continue;
        };
        if !keeps(RouteResultPart::Legs) {
            leg_result.legs = Vec::new();
        }
        if !keeps(RouteResultPart::Recosts) {
            leg_result.recosts = Vec::new();
        }
        for leg in &mut leg_result.legs {
            if !keeps(RouteResultPart::EdgeAttributions) {
                leg.edges = Vec::new();
            }
            if !keeps(RouteResultPart::ShapeSegments) {
                leg.shape_segments = Vec::new();
            }
        }
        leg_result.alternatives.iter_mut().for_each(alternative);
    }
    if let RouteAlternatives::Computed(alternatives) = &mut route.alternatives {
        alternatives.iter_mut().for_each(alternative);
    }
    summaries
}

/// Removes `costBreakdown` from every edge attribution in `value` (a
/// serialized `RouteResult`): the top-level `edgeAttributions` and every
/// leg's `edges`.
pub(crate) fn strip_cost_breakdowns(value: &mut serde_json::Value) {
    use serde_json::Value;
    match value {
        Value::Object(map) => {
            for (key, child) in map.iter_mut() {
                if key == "edgeAttributions" || key == "edges" {
                    if let Value::Array(edges) = child {
                        for edge in edges {
                            if let Value::Object(edge) = edge {
                                edge.remove("costBreakdown");
                            }
                        }
                    }
                } else {
                    strip_cost_breakdowns(child);
                }
            }
        }
        Value::Array(items) => items.iter_mut().for_each(strip_cost_breakdowns),
        _ => {}
    }
}

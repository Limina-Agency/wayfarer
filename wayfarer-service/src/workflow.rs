//! Route requests as a host sends them to a routing source: the engine's
//! [`RouteRequest`] plus the overlay sets ([`crate::overlay_set`]) and the
//! precompiled mobility input that join it, routed against one source and
//! reported with the source's id and revision.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use thiserror::Error;
use wayfarer::limits::{
    MAX_ROUTE_PROVENANCE_VALUES, MAX_ROUTE_STRING_BYTES, MAX_ROUTE_TOTAL_STRING_BYTES,
};
use wayfarer::{
    RouteDynamicOverlay, RouteEdgeCostMultiplierRange, RouteEdgeRange, RouteExplanation,
    RouteExplanationKind, RouteExplanationSeverity, RouteFailure, RouteOptions, RouteRequest,
    RouteResult, RoutingEngine,
};

use crate::RoutingSourceId;
use crate::overlay_set::{RouteOverlaySet, RouteOverlaySetBinding};

/// The most mobility adjustments (edge ids, ranges, connectors) one request
/// may carry.
pub const MAX_WORKFLOW_MOBILITY_ADJUSTMENTS: usize = 100_000;
/// The most overlay sets one workflow request may carry.
pub const MAX_WORKFLOW_OVERLAY_SETS: usize = 16;

/// The source a workflow is routed against, as its result reports it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RouteWorkflowSourceContext {
    pub source_id: RoutingSourceId,
    pub revision: String,
    pub cache_hit: bool,
}

/// Portable, precompiled mobility influence.
///
/// Loading patches and sampling an elevation source are deliberately outside
/// the route kernel. The hybrid-routing compiler turns those artifacts into
/// these edge/range adjustments and an optional dynamic overlay; therefore DEM
/// slope still changes route costs without giving this service package paths,
/// raster formats, or renderer imagery.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteWorkflowMobilityInput {
    #[serde(default)]
    pub blocked_edge_ids: BTreeSet<u64>,
    #[serde(default)]
    pub blocked_edge_ranges: Vec<RouteEdgeRange>,
    #[serde(default)]
    pub warning_edge_ids: BTreeSet<u64>,
    #[serde(default)]
    pub warning_edge_ranges: Vec<RouteEdgeRange>,
    #[serde(default)]
    pub edge_cost_multipliers: BTreeMap<u64, f64>,
    #[serde(default)]
    pub edge_cost_multiplier_ranges: Vec<RouteEdgeCostMultiplierRange>,
    #[serde(default)]
    pub dynamic_overlay: Option<RouteDynamicOverlay>,
    /// Connector overlays keyed by their stable workflow leg index. These are
    /// merged only for the corresponding routed leg, preventing endpoint nodes
    /// from exposing another leg's origin/destination connectors.
    #[serde(default)]
    pub leg_dynamic_overlays: BTreeMap<usize, RouteDynamicOverlay>,
    #[serde(default)]
    pub provenance: RouteWorkflowMobilityProvenance,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteWorkflowMobilityProvenance {
    #[serde(default)]
    pub policy_version: Option<String>,
    #[serde(default)]
    pub patch_ids: Vec<String>,
    #[serde(default)]
    pub source_ids: Vec<String>,
    #[serde(default)]
    pub model_versions: Vec<String>,
    /// Identifies the elevation/cost model already folded into range costs.
    #[serde(default)]
    pub elevation_model_version: Option<String>,
    #[serde(default)]
    pub impact_tier_counts: BTreeMap<String, usize>,
    #[serde(default)]
    pub connector_ids: Vec<String>,
    #[serde(default)]
    pub connector_source_chunk_ids: Vec<String>,
    #[serde(default)]
    pub partial_edge_split_count: usize,
    #[serde(default)]
    pub connector_count: usize,
    #[serde(default)]
    pub endpoint_connector_count: usize,
    #[serde(default)]
    pub road_bypass_connector_count: usize,
    #[serde(default)]
    pub cross_country_connector_count: usize,
    #[serde(default)]
    pub cross_country_no_coverage: bool,
    #[serde(default)]
    pub slope_applied: bool,
}

/// A route request to a routing source: the engine's request (its fields at
/// the top level of the JSON) plus overlay sets and mobility input.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteWorkflowRequest {
    #[serde(flatten)]
    pub route: RouteRequest,
    #[serde(default)]
    pub overlay_provenance: RouteWorkflowOverlayProvenance,
    #[serde(default)]
    pub mobility: Option<RouteWorkflowMobilityInput>,
    /// Overlay sets (`crate::overlay_set`) whose live items join the
    /// request's overlays, bound edges where a set is bound to this package,
    /// re-projected geometry otherwise. Sets with one `setId` are merged
    /// first.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub overlay_sets: Vec<RouteOverlaySet>,
}

impl From<RouteRequest> for RouteWorkflowRequest {
    fn from(route: RouteRequest) -> Self {
        Self {
            route,
            overlay_provenance: RouteWorkflowOverlayProvenance::default(),
            mobility: None,
            overlay_sets: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteWorkflowOverlayProvenance {
    #[serde(default)]
    pub policy_version: Option<String>,
    #[serde(default)]
    pub source_ids: Vec<String>,
}

/// The engine's route (its fields at the top level of the JSON) with the
/// source it ran on and what the service added to the request.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteWorkflowResult {
    pub source_id: RoutingSourceId,
    pub revision: String,
    pub cache_hit: bool,
    #[serde(flatten)]
    pub route: RouteResult,
    pub overlay_provenance: RouteWorkflowOverlayProvenance,
    pub mobility_provenance: Option<RouteWorkflowMobilityProvenance>,
    /// How each overlay set reached the request.
    pub overlay_sets: Vec<RouteOverlaySetBinding>,
}

#[derive(Debug, Error)]
pub enum RouteWorkflowError {
    #[error("invalid route workflow request: {0}")]
    InvalidRequest(&'static str),
    #[error("route workflow request exceeds service bounds: {0}")]
    RequestTooLarge(&'static str),
    #[error("overlay set rejected: {0}")]
    OverlaySet(#[from] crate::overlay_set::OverlaySetError),
    /// The engine refused or failed the route (`segment_index` names a failed
    /// leg).
    #[error("{0}")]
    Route(Box<RouteFailure>),
}

impl From<RouteFailure> for RouteWorkflowError {
    fn from(failure: RouteFailure) -> Self {
        Self::Route(Box::new(failure))
    }
}

/// Routes `request` on `engine` (the source's): overlay sets join its
/// overlays and mobility input its options, then the engine routes it.
pub fn compile_route_workflow(
    engine: &RoutingEngine,
    source: RouteWorkflowSourceContext,
    request: RouteWorkflowRequest,
) -> Result<RouteWorkflowResult, RouteWorkflowError> {
    if source.revision.trim().is_empty()
        || source.revision.len() > crate::MAX_SOURCE_REVISION_BYTES
        || source.revision.chars().any(char::is_control)
    {
        return Err(RouteWorkflowError::InvalidRequest("source revision"));
    }
    validate_workflow(&request)?;
    let RouteWorkflowRequest {
        mut route,
        overlay_provenance,
        mobility,
        overlay_sets,
    } = request;
    // Overlay sets join the request's overlays, merged per set id.
    let dataset_hash = engine.package().dataset_hash().to_string();
    let mut set_bindings = Vec::new();
    let mut by_set = BTreeMap::<String, Vec<RouteOverlaySet>>::new();
    for set in overlay_sets {
        set.validate()?;
        by_set.entry(set.set_id.clone()).or_default().push(set);
    }
    for sets in by_set.into_values() {
        let Some(set) = RouteOverlaySet::merge(&sets)? else {
            continue;
        };
        let (set_overlays, binding) = set.workflow_overlays(&dataset_hash);
        route.overlays.extend(set_overlays);
        set_bindings.push(binding);
    }
    let mobility_adjustment_count = mobility
        .as_ref()
        .map(mobility_adjustment_count_of)
        .unwrap_or(0);
    let mobility_provenance = mobility.as_ref().map(|input| input.provenance.clone());
    if let Some(mobility) = mobility {
        for (leg_index, overlay) in apply_mobility(mobility, &mut route.options) {
            let target = route.leg_dynamic_overlays.remove(&leg_index);
            let mut merged = target;
            RouteDynamicOverlay::merge_into(&mut merged, overlay);
            route
                .leg_dynamic_overlays
                .insert(leg_index, merged.expect("merged overlay"));
        }
    }
    let mut route = engine.route(route)?;
    route.diagnostics.mobility_adjustment_count = mobility_adjustment_count;
    if mobility_provenance
        .as_ref()
        .is_some_and(|value| !value.patch_ids.is_empty() || !value.source_ids.is_empty())
    {
        // After the cost driver and dynamic-overlay explanations.
        let at = route
            .explanations
            .iter()
            .position(|explanation| {
                !matches!(
                    explanation.kind,
                    RouteExplanationKind::DominantCostDriver(_)
                        | RouteExplanationKind::DynamicOverlay
                )
            })
            .unwrap_or(route.explanations.len());
        route.explanations.insert(
            at,
            RouteExplanation {
                kind: RouteExplanationKind::MobilityPolicy,
                severity: RouteExplanationSeverity::Info,
                edge_count: None,
                value: mobility_provenance
                    .as_ref()
                    .map(|value| value.patch_ids.len() as f64),
            },
        );
        route.explanations.truncate(6);
    }
    Ok(RouteWorkflowResult {
        source_id: source.source_id,
        revision: source.revision,
        cache_hit: source.cache_hit,
        route,
        overlay_provenance,
        mobility_provenance,
        overlay_sets: set_bindings,
    })
}

/// The parts the engine does not check: overlay sets, mobility input and
/// provenance.
fn validate_workflow(request: &RouteWorkflowRequest) -> Result<(), RouteWorkflowError> {
    if request.overlay_sets.len() > MAX_WORKFLOW_OVERLAY_SETS {
        return Err(RouteWorkflowError::RequestTooLarge("overlay sets"));
    }
    if let Some(mobility) = &request.mobility {
        if mobility_adjustment_count_of(mobility) > MAX_WORKFLOW_MOBILITY_ADJUSTMENTS {
            return Err(RouteWorkflowError::RequestTooLarge("mobility adjustments"));
        }
        if mobility
            .edge_cost_multipliers
            .values()
            .any(|value| !value.is_finite() || *value <= 0.0)
            || mobility
                .edge_cost_multiplier_ranges
                .iter()
                .any(|range| !valid_multiplier_range(*range))
            || mobility
                .blocked_edge_ranges
                .iter()
                .chain(&mobility.warning_edge_ranges)
                .any(|range| !valid_range(*range))
        {
            return Err(RouteWorkflowError::InvalidRequest(
                "mobility edge adjustment",
            ));
        }
        validate_provenance(&mobility.provenance)?;
        let leg_count = request.route.locations.len().saturating_sub(1);
        for leg_index in mobility.leg_dynamic_overlays.keys() {
            if *leg_index >= leg_count
                || request.route.leg_mode(*leg_index) != wayfarer::RouteLegMode::Routed
            {
                return Err(RouteWorkflowError::InvalidRequest(
                    "mobility leg dynamic overlay",
                ));
            }
        }
    }
    validate_overlay_provenance(&request.overlay_provenance)?;
    Ok(())
}

fn apply_mobility(
    input: RouteWorkflowMobilityInput,
    options: &mut RouteOptions,
) -> BTreeMap<usize, RouteDynamicOverlay> {
    let RouteWorkflowMobilityInput {
        blocked_edge_ids,
        blocked_edge_ranges,
        warning_edge_ids,
        warning_edge_ranges,
        edge_cost_multipliers,
        edge_cost_multiplier_ranges,
        dynamic_overlay,
        leg_dynamic_overlays,
        provenance: _,
    } = input;
    options.blocked_edge_ids.extend(blocked_edge_ids);
    options.blocked_edge_ranges.extend(blocked_edge_ranges);
    options.warning_edge_ids.extend(warning_edge_ids);
    options.warning_edge_ranges.extend(warning_edge_ranges);
    for (edge_id, multiplier) in edge_cost_multipliers {
        if multiplier < 1.0 {
            // A discount: as a whole-edge range, so it multiplies with other
            // discounts on the edge instead of losing to a slow-go factor.
            options
                .edge_cost_multiplier_ranges
                .push(RouteEdgeCostMultiplierRange {
                    edge_id,
                    start_fraction: 0.0,
                    end_fraction: 1.0,
                    multiplier,
                });
            continue;
        }
        options
            .edge_cost_multipliers
            .entry(edge_id)
            .and_modify(|current| *current = current.max(multiplier))
            .or_insert(multiplier);
    }
    options
        .edge_cost_multiplier_ranges
        .extend(edge_cost_multiplier_ranges);
    if let Some(incoming) = dynamic_overlay {
        RouteDynamicOverlay::merge_into(&mut options.dynamic_overlay, incoming);
    }
    leg_dynamic_overlays
}

fn validate_overlay_provenance(
    provenance: &RouteWorkflowOverlayProvenance,
) -> Result<(), RouteWorkflowError> {
    let values = provenance.source_ids.len() + usize::from(provenance.policy_version.is_some());
    if values > MAX_ROUTE_PROVENANCE_VALUES {
        return Err(RouteWorkflowError::RequestTooLarge("overlay provenance"));
    }
    let mut strings = provenance
        .source_ids
        .iter()
        .map(String::as_str)
        .chain(provenance.policy_version.as_deref());
    if strings.clone().map(str::len).sum::<usize>() > MAX_ROUTE_TOTAL_STRING_BYTES {
        return Err(RouteWorkflowError::RequestTooLarge(
            "overlay provenance strings",
        ));
    }
    if strings.any(invalid_bounded_string) {
        return Err(RouteWorkflowError::InvalidRequest("overlay provenance"));
    }
    Ok(())
}

fn invalid_bounded_string(value: &str) -> bool {
    value.is_empty() || value.len() > MAX_ROUTE_STRING_BYTES || value.chars().any(char::is_control)
}

fn valid_range(range: RouteEdgeRange) -> bool {
    range.start_fraction.is_finite()
        && range.end_fraction.is_finite()
        && range.start_fraction >= 0.0
        && range.end_fraction <= 1.0
        && range.end_fraction > range.start_fraction
}

fn valid_multiplier_range(range: RouteEdgeCostMultiplierRange) -> bool {
    valid_range(RouteEdgeRange {
        edge_id: range.edge_id,
        start_fraction: range.start_fraction,
        end_fraction: range.end_fraction,
    }) && range.multiplier.is_finite()
        && range.multiplier > 0.0
}

fn mobility_adjustment_count_of(input: &RouteWorkflowMobilityInput) -> usize {
    input.blocked_edge_ids.len()
        + input.blocked_edge_ranges.len()
        + input.warning_edge_ids.len()
        + input.warning_edge_ranges.len()
        + input.edge_cost_multipliers.len()
        + input.edge_cost_multiplier_ranges.len()
        + input
            .dynamic_overlay
            .as_ref()
            .map(|overlay| overlay.split_edge_ranges.len() + overlay.virtual_edges.len())
            .unwrap_or(0)
        + input
            .leg_dynamic_overlays
            .values()
            .map(|overlay| overlay.split_edge_ranges.len() + overlay.virtual_edges.len())
            .sum::<usize>()
}

fn validate_provenance(
    provenance: &RouteWorkflowMobilityProvenance,
) -> Result<(), RouteWorkflowError> {
    let values = provenance.patch_ids.len()
        + provenance.source_ids.len()
        + provenance.model_versions.len()
        + provenance.impact_tier_counts.len()
        + provenance.connector_ids.len()
        + provenance.connector_source_chunk_ids.len()
        + usize::from(provenance.policy_version.is_some())
        + usize::from(provenance.elevation_model_version.is_some());
    if values > MAX_ROUTE_PROVENANCE_VALUES {
        return Err(RouteWorkflowError::RequestTooLarge("mobility provenance"));
    }
    if provenance
        .partial_edge_split_count
        .saturating_add(provenance.connector_count)
        > MAX_WORKFLOW_MOBILITY_ADJUSTMENTS
        || provenance.connector_ids.len() > provenance.connector_count
        || provenance
            .endpoint_connector_count
            .saturating_add(provenance.road_bypass_connector_count)
            .saturating_add(provenance.cross_country_connector_count)
            > provenance.connector_count
        || (provenance.cross_country_no_coverage && provenance.slope_applied)
    {
        return Err(RouteWorkflowError::InvalidRequest(
            "mobility provenance counts",
        ));
    }
    let mut strings = provenance
        .patch_ids
        .iter()
        .chain(&provenance.source_ids)
        .chain(&provenance.model_versions)
        .chain(provenance.impact_tier_counts.keys())
        .chain(&provenance.connector_ids)
        .chain(&provenance.connector_source_chunk_ids)
        .map(String::as_str)
        .chain(provenance.policy_version.as_deref())
        .chain(provenance.elevation_model_version.as_deref());
    if strings.clone().map(str::len).sum::<usize>() > MAX_ROUTE_TOTAL_STRING_BYTES {
        return Err(RouteWorkflowError::RequestTooLarge(
            "mobility provenance strings",
        ));
    }
    if strings.any(invalid_bounded_string) {
        return Err(RouteWorkflowError::InvalidRequest("mobility provenance"));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use wayfarer::{GeoPoint, MobilityProfile, RouteLegMode};

    use super::*;

    fn request(legs: Vec<RouteLegMode>) -> RouteWorkflowRequest {
        let mut route = RouteRequest::new(
            GeoPoint::new(52.0, 21.0),
            GeoPoint::new(52.0, 21.1),
            MobilityProfile::OnFoot,
        );
        route.legs = legs;
        RouteWorkflowRequest {
            mobility: Some(RouteWorkflowMobilityInput::default()),
            ..route.into()
        }
    }

    #[test]
    fn mobility_rejects_non_finite_cost_adjustments() {
        let mut request = request(vec![RouteLegMode::Direct]);
        request
            .mobility
            .as_mut()
            .unwrap()
            .edge_cost_multipliers
            .insert(1, f64::NAN);
        assert!(matches!(
            validate_workflow(&request),
            Err(RouteWorkflowError::InvalidRequest(_))
        ));
    }

    #[test]
    fn mobility_leg_overlay_is_not_applied_to_global_options() {
        let mobility = RouteWorkflowMobilityInput {
            dynamic_overlay: Some(RouteDynamicOverlay::from_blocked_ranges(&[
                RouteEdgeRange {
                    edge_id: 7,
                    start_fraction: 0.1,
                    end_fraction: 0.2,
                },
            ])),
            leg_dynamic_overlays: BTreeMap::from([(
                1,
                RouteDynamicOverlay::from_blocked_ranges(&[RouteEdgeRange {
                    edge_id: 42,
                    start_fraction: 0.25,
                    end_fraction: 0.75,
                }]),
            )]),
            ..RouteWorkflowMobilityInput::default()
        };
        let mut options = RouteOptions::default();

        let leg_overlays = apply_mobility(mobility, &mut options);

        assert_eq!(
            options.dynamic_overlay.as_ref().unwrap().split_edge_ranges[0].edge_id,
            7
        );
        assert_eq!(leg_overlays[&1].split_edge_ranges[0].edge_id, 42);
    }

    #[test]
    fn mobility_leg_overlay_rejects_direct_leg_assignment() {
        let mut request = request(vec![RouteLegMode::Direct]);
        request
            .mobility
            .as_mut()
            .unwrap()
            .leg_dynamic_overlays
            .insert(0, RouteDynamicOverlay::default());
        assert!(matches!(
            validate_workflow(&request),
            Err(RouteWorkflowError::InvalidRequest(
                "mobility leg dynamic overlay"
            ))
        ));
    }

    #[test]
    fn mobility_provenance_rejects_impossible_cross_country_state() {
        let provenance = RouteWorkflowMobilityProvenance {
            cross_country_no_coverage: true,
            slope_applied: true,
            ..RouteWorkflowMobilityProvenance::default()
        };
        assert!(matches!(
            validate_provenance(&provenance),
            Err(RouteWorkflowError::InvalidRequest(
                "mobility provenance counts"
            ))
        ));
    }
}

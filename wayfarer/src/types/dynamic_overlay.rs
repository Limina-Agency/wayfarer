//! Dynamic overlays: edges split into usable and blocked stretches and virtual connectors, costed per request.

use super::*;

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteDynamicOverlay {
    #[serde(default)]
    pub split_edge_ranges: Vec<RouteDynamicSplitEdgeRange>,
    #[serde(default)]
    pub virtual_edges: Vec<RouteDynamicVirtualEdge>,
    #[serde(default)]
    pub diagnostics: RouteDynamicOverlayDiagnostics,
}

impl RouteDynamicOverlay {
    pub fn from_blocked_ranges(blocked_ranges: &[RouteEdgeRange]) -> Self {
        let mut by_edge = BTreeMap::<u64, Vec<RouteEdgeRange>>::new();
        for range in blocked_ranges.iter().filter_map(valid_edge_range) {
            by_edge.entry(range.edge_id).or_default().push(range);
        }

        let mut split_edge_ranges = Vec::new();
        for (edge_id, mut ranges) in by_edge {
            ranges.sort_by(|left, right| {
                left.start_fraction
                    .total_cmp(&right.start_fraction)
                    .then_with(|| left.end_fraction.total_cmp(&right.end_fraction))
            });
            let mut merged = Vec::<RouteEdgeRange>::new();
            for range in ranges {
                if let Some(last) = merged.last_mut()
                    && range.start_fraction <= last.end_fraction + f64::EPSILON
                {
                    last.end_fraction = last.end_fraction.max(range.end_fraction).clamp(0.0, 1.0);
                    continue;
                }
                merged.push(range);
            }

            let mut cursor: f64 = 0.0;
            let mut usable_ranges = Vec::new();
            for range in &merged {
                if range.start_fraction > cursor + f64::EPSILON {
                    usable_ranges.push(RouteEdgeRange {
                        edge_id,
                        start_fraction: cursor,
                        end_fraction: range.start_fraction,
                    });
                }
                cursor = cursor.max(range.end_fraction);
            }
            if cursor < 1.0 - f64::EPSILON {
                usable_ranges.push(RouteEdgeRange {
                    edge_id,
                    start_fraction: cursor,
                    end_fraction: 1.0,
                });
            }

            split_edge_ranges.push(RouteDynamicSplitEdgeRange {
                edge_id,
                blocked_ranges: merged,
                usable_ranges,
            });
        }

        let mut overlay = Self {
            split_edge_ranges,
            virtual_edges: Vec::new(),
            diagnostics: RouteDynamicOverlayDiagnostics::default(),
        };
        overlay.refresh_diagnostics();
        overlay
    }

    pub fn refresh_diagnostics(&mut self) {
        self.diagnostics.split_edge_count = self.split_edge_ranges.len();
        self.diagnostics.partial_split_edge_count = self
            .split_edge_ranges
            .iter()
            .filter(|split| split.is_partial())
            .count();
        self.diagnostics.full_blocked_edge_count = self
            .split_edge_ranges
            .iter()
            .filter(|split| split.is_fully_blocked())
            .count();
        self.diagnostics.virtual_connector_count = self.virtual_edges.len();
        self.diagnostics.endpoint_connector_count = self
            .virtual_edges
            .iter()
            .filter(|edge| edge.connector_type == RouteDynamicVirtualEdgeKind::Endpoint)
            .count();
        self.diagnostics.road_bypass_connector_count = self
            .virtual_edges
            .iter()
            .filter(|edge| edge.connector_type == RouteDynamicVirtualEdgeKind::RoadToRoadBypass)
            .count();
        self.diagnostics.cross_country_connector_count = self
            .virtual_edges
            .iter()
            .filter(|edge| edge.connector_type == RouteDynamicVirtualEdgeKind::CrossCountry)
            .count();
    }

    pub fn edge_fully_blocked(&self, edge_id: u64) -> bool {
        self.split_edge_ranges
            .iter()
            .find(|split| split.edge_id == edge_id)
            .is_some_and(RouteDynamicSplitEdgeRange::is_fully_blocked)
    }

    pub fn partial_blocked_coverage(&self, edge_id: u64) -> Option<f64> {
        self.split_edge_ranges
            .iter()
            .find(|split| split.edge_id == edge_id && split.is_partial())
            .map(RouteDynamicSplitEdgeRange::blocked_coverage)
    }
}

fn valid_edge_range(range: &RouteEdgeRange) -> Option<RouteEdgeRange> {
    let start = range.start_fraction.clamp(0.0, 1.0);
    let end = range.end_fraction.clamp(0.0, 1.0);
    (start.is_finite() && end.is_finite() && end > start).then_some(RouteEdgeRange {
        edge_id: range.edge_id,
        start_fraction: start,
        end_fraction: end,
    })
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteDynamicSplitEdgeRange {
    pub edge_id: u64,
    #[serde(default)]
    pub blocked_ranges: Vec<RouteEdgeRange>,
    #[serde(default)]
    pub usable_ranges: Vec<RouteEdgeRange>,
}

impl RouteDynamicSplitEdgeRange {
    pub fn blocked_coverage(&self) -> f64 {
        self.blocked_ranges
            .iter()
            .map(|range| {
                (range.end_fraction.clamp(0.0, 1.0) - range.start_fraction.clamp(0.0, 1.0)).max(0.0)
            })
            .sum::<f64>()
            .clamp(0.0, 1.0)
    }

    pub fn is_fully_blocked(&self) -> bool {
        self.blocked_coverage() >= 1.0 - 1e-6
    }

    pub fn is_partial(&self) -> bool {
        !self.is_fully_blocked() && self.blocked_coverage() > f64::EPSILON
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteDynamicVirtualEdgeKind {
    Endpoint,
    RoadEntry,
    RoadToRoadBypass,
    CrossCountry,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteDynamicEndpointKind {
    Origin,
    Destination,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteDynamicVirtualEdge {
    pub edge_id: u64,
    pub connector_id: String,
    pub connector_type: RouteDynamicVirtualEdgeKind,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source_endpoint: Option<RouteDynamicEndpointKind>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source_point: Option<GeoPoint>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source_edge_id: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source_fraction: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target_endpoint: Option<RouteDynamicEndpointKind>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target_point: Option<GeoPoint>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target_edge_id: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target_fraction: Option<f64>,
    #[serde(default)]
    pub source_chunk_ids: Vec<String>,
    #[serde(default)]
    pub geometry: Vec<GeoPoint>,
    pub distance_m: f64,
    pub cost: f64,
    #[serde(default)]
    pub provenance: String,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteDynamicOverlayDiagnostics {
    pub split_edge_count: usize,
    pub partial_split_edge_count: usize,
    pub full_blocked_edge_count: usize,
    pub virtual_connector_count: usize,
    pub endpoint_connector_count: usize,
    pub road_bypass_connector_count: usize,
    #[serde(default)]
    pub cross_country_connector_count: usize,
    #[serde(default)]
    pub rejected_connector_count: usize,
}

#[cfg(test)]
mod dynamic_overlay_tests {
    use super::*;

    #[test]
    fn dynamic_overlay_splits_partial_blocked_range() {
        let overlay = RouteDynamicOverlay::from_blocked_ranges(&[RouteEdgeRange {
            edge_id: 10,
            start_fraction: 0.25,
            end_fraction: 0.5,
        }]);

        assert_eq!(overlay.diagnostics.split_edge_count, 1);
        assert_eq!(overlay.diagnostics.partial_split_edge_count, 1);
        assert_eq!(overlay.diagnostics.full_blocked_edge_count, 0);
        let split = &overlay.split_edge_ranges[0];
        assert_eq!(split.usable_ranges.len(), 2);
        assert_eq!(split.usable_ranges[0].start_fraction, 0.0);
        assert_eq!(split.usable_ranges[0].end_fraction, 0.25);
        assert_eq!(split.usable_ranges[1].start_fraction, 0.5);
        assert_eq!(split.usable_ranges[1].end_fraction, 1.0);
    }

    #[test]
    fn dynamic_overlay_merges_ranges_and_detects_full_block() {
        let overlay = RouteDynamicOverlay::from_blocked_ranges(&[
            RouteEdgeRange {
                edge_id: 22,
                start_fraction: 0.0,
                end_fraction: 0.4,
            },
            RouteEdgeRange {
                edge_id: 22,
                start_fraction: 0.4,
                end_fraction: 1.0,
            },
        ]);

        assert_eq!(overlay.diagnostics.split_edge_count, 1);
        assert_eq!(overlay.diagnostics.partial_split_edge_count, 0);
        assert_eq!(overlay.diagnostics.full_blocked_edge_count, 1);
        assert!(overlay.edge_fully_blocked(22));
        assert_eq!(overlay.split_edge_ranges[0].usable_ranges, Vec::new());
    }

    #[test]
    fn dynamic_overlay_ignores_empty_or_invalid_ranges() {
        let overlay = RouteDynamicOverlay::from_blocked_ranges(&[
            RouteEdgeRange {
                edge_id: 1,
                start_fraction: 0.6,
                end_fraction: 0.6,
            },
            RouteEdgeRange {
                edge_id: 2,
                start_fraction: f64::NAN,
                end_fraction: 0.8,
            },
        ]);

        assert!(overlay.split_edge_ranges.is_empty());
        assert_eq!(overlay.diagnostics.split_edge_count, 0);
    }
}

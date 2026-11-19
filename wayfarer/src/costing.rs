//! Request costing: the edge-cost model (`wayfarer_costing`) with the
//! request's blocked edges, avoidances, cost multipliers and dynamic overlay
//! layered on.

pub(crate) use wayfarer_costing::*;
#[cfg(test)]
use wayfarer_package::RoadEdge;

use crate::graph_view::GraphViewEdge;
use crate::types::{MobilityProfile, RouteOptions};
use wayfarer_package::sar::SarEdgeRelaxations;

#[derive(Debug, Clone, Copy)]
pub(crate) struct CostingPolicy<'a> {
    model: CostModel<'a>,
    options: &'a RouteOptions,
    /// The request's SAR R1 and R2 (Wheeled requests only).
    sar: Option<SarEdgeRelaxations>,
}

impl<'a> CostingPolicy<'a> {
    pub fn new(profile: MobilityProfile, options: &'a RouteOptions) -> Self {
        Self::with_destination_only_policy(profile, options, true)
    }

    pub fn with_destination_only_policy(
        profile: MobilityProfile,
        options: &'a RouteOptions,
        allow_destination_only: bool,
    ) -> Self {
        Self {
            model: CostModel::with_destination_only_policy(
                profile,
                &options.costing,
                allow_destination_only,
            ),
            options,
            sar: crate::sar::edge_relaxations(options, profile),
        }
    }

    /// This policy with `sar` as its SAR relaxations (derived columns).
    pub(crate) fn with_sar_relaxations(self, sar: Option<SarEdgeRelaxations>) -> Self {
        Self { sar, ..self }
    }

    /// The request's SAR R1 and R2, when it enables either.
    pub(crate) fn sar_relaxations(&self) -> Option<SarEdgeRelaxations> {
        self.sar
    }

    pub fn edge_cost(&self, record: &GraphViewEdge) -> EdgeCostDecision {
        self.model.edge_cost(record, self)
    }

    /// [`Self::edge_cost`] with the record's cost classes already derived.
    pub(crate) fn edge_cost_with_classes(
        &self,
        record: &GraphViewEdge,
        classes: EdgeCostClasses,
    ) -> EdgeCostDecision {
        self.model.edge_cost_inputs(
            &EdgeCostInputs::from_parts(&record.edge, &record.shared_info, classes),
            self,
        )
    }

    /// [`Self::edge_duration_s`] with the record's cost classes already derived.
    pub(crate) fn edge_duration_s_with_classes(
        &self,
        record: &GraphViewEdge,
        classes: EdgeCostClasses,
    ) -> f64 {
        let edge_id = record.edge.edge_id;
        self.overridden_duration_s(
            edge_id,
            f64::from(record.edge.distance_m).max(0.0),
            self.model.edge_duration_s_with_classes(record, classes),
        ) * self.duration_factor(edge_id)
    }

    /// [`Self::junction_delay_for_records`] remembering the
    /// records' junction speeds (see `CostModel::junction_delay_for_records_memo`).
    pub(crate) fn junction_delay_for_records_memo(
        &self,
        previous: (&GraphViewEdge, EdgeCostClasses, &mut Option<f64>),
        next: (&GraphViewEdge, EdgeCostClasses, &mut Option<f64>),
    ) -> f64 {
        self.model
            .junction_delay_for_records_memo(previous, next, self)
    }

    /// Whether destination-only edges are usable (false on the strict pass of
    /// a request that avoids them fully).
    pub(crate) fn allows_destination_only(&self) -> bool {
        self.model.allows_destination_only()
    }

    /// The edge costing function shared by record costing and the package's
    /// derived columns.
    pub(crate) fn edge_cost_inputs(&self, record: &EdgeCostInputs<'_>) -> EdgeCostDecision {
        self.model.edge_cost_inputs(record, self)
    }

    #[cfg(test)]
    pub fn transition_cost(&self, previous: &RoadEdge, next: &RoadEdge) -> f64 {
        self.model.transition_cost(previous, next)
    }

    pub fn transition_cost_for_records(
        &self,
        previous: &GraphViewEdge,
        next: &GraphViewEdge,
    ) -> f64 {
        self.model.transition_cost_for_records(previous, next)
    }

    #[cfg(test)]
    fn transition_cost_with_shared_info(
        &self,
        previous: &RoadEdge,
        previous_info: &wayfarer_package::SharedEdgeInfo,
        next: &RoadEdge,
        next_info: &wayfarer_package::SharedEdgeInfo,
        same_corridor: &dyn Fn() -> bool,
    ) -> f64 {
        self.model.transition_cost_with_shared_info(
            previous,
            previous_info,
            next,
            next_info,
            same_corridor,
        )
    }

    /// The edge's travel time under the request: the profile's, with the
    /// request's speed overrides and discounts.
    #[cfg(test)]
    pub(crate) fn edge_duration_s(&self, record: &GraphViewEdge) -> f64 {
        let edge_id = record.edge.edge_id;
        self.overridden_duration_s(
            edge_id,
            f64::from(record.edge.distance_m).max(0.0),
            self.model.edge_duration_s(record),
        ) * self.duration_factor(edge_id)
    }

    /// Seconds lost at the junction from `previous` onto `next` (see
    /// `wayfarer_costing::junction`).
    #[cfg(any(test, feature = "reference"))]
    pub(crate) fn junction_delay_for_records(
        &self,
        previous: &GraphViewEdge,
        next: &GraphViewEdge,
    ) -> f64 {
        self.model
            .junction_delay_for_records_with(previous, next, self)
    }

    /// Whether the request blocks `edge_id` outright (blocked ids or ranges,
    /// or an overlay's full blocks).
    pub(crate) fn edge_blocked(&self, edge_id: u64) -> bool {
        if self.options.blocked_edge_ids.contains(&edge_id) {
            return true;
        }
        if let Some(overlay) = &self.options.dynamic_overlay {
            return overlay.edge_fully_blocked(edge_id);
        }
        self.options
            .blocked_edge_ranges
            .iter()
            .filter(|range| range.edge_id == edge_id)
            .any(|range| {
                let start = range.start_fraction.clamp(0.0, 1.0);
                let end = range.end_fraction.clamp(0.0, 1.0);
                start.is_finite() && end.is_finite() && end > start
            })
    }

    /// The edge's cost multiplier: the largest slow-go factor (≥ 1: whole
    /// edge, ranges, partial blocks) times every discount (factors in
    /// (0, 1)), so overlapping slow-go factors combine by maximum and
    /// discounts by product. A ranged factor
    /// counts by the share of the edge it covers.
    pub(crate) fn overlay_multiplier(&self, edge_id: u64) -> f64 {
        let (slow, discount) = self.multiplier_parts(edge_id);
        slow * discount
    }

    /// (largest slow-go factor, product of discounts) of the edge.
    fn multiplier_parts(&self, edge_id: u64) -> (f64, f64) {
        let mut discount = 1.0;
        let whole_edge_multiplier = self
            .options
            .edge_cost_multipliers
            .get(&edge_id)
            .copied()
            .filter(|multiplier| multiplier.is_finite() && *multiplier > 0.0)
            .unwrap_or(1.0);
        if whole_edge_multiplier < 1.0 {
            discount *= whole_edge_multiplier;
        }
        let mut ranged_multiplier = 1.0f64;
        for range in self
            .options
            .edge_cost_multiplier_ranges
            .iter()
            .filter(|range| range.edge_id == edge_id)
            .filter(|range| range.multiplier.is_finite() && range.multiplier > 0.0)
        {
            let coverage = (range.end_fraction.clamp(0.0, 1.0)
                - range.start_fraction.clamp(0.0, 1.0))
            .max(0.0);
            let factor = 1.0 + (range.multiplier - 1.0) * coverage;
            if range.multiplier < 1.0 {
                discount *= factor;
            } else {
                ranged_multiplier = ranged_multiplier.max(factor);
            }
        }
        let dynamic_partial_block_multiplier = self
            .options
            .dynamic_overlay
            .as_ref()
            .and_then(|overlay| overlay.partial_blocked_coverage(edge_id))
            .map(|coverage| 1.0 + (50.0 - 1.0) * coverage)
            .unwrap_or(1.0);
        let slow = whole_edge_multiplier
            .max(ranged_multiplier)
            .max(dynamic_partial_block_multiplier);
        (slow, discount)
    }

    /// A discount lowers the edge's duration by its factor too; a slow-go
    /// factor changes cost only (as it always has).
    pub(crate) fn duration_factor(&self, edge_id: u64) -> f64 {
        if self.options.edge_cost_multipliers.is_empty()
            && self.options.edge_cost_multiplier_ranges.is_empty()
        {
            return 1.0;
        }
        self.multiplier_parts(edge_id).1
    }

    /// The edge's time under the request's speed overrides: a whole-edge
    /// override sets the whole edge's speed; ranges set their stretches'
    /// (their coverage summed, at most the whole edge), the rest keeping
    /// `duration_s`'s pace.
    pub(crate) fn overridden_duration_s(
        &self,
        edge_id: u64,
        distance_m: f64,
        duration_s: f64,
    ) -> f64 {
        let valid = |speed_kph: f64| speed_kph.is_finite() && speed_kph > 0.0;
        if let Some(speed_kph) = self
            .options
            .edge_speed_overrides
            .get(&edge_id)
            .copied()
            .filter(|speed| valid(*speed))
        {
            return distance_m / (speed_kph / 3.6);
        }
        if self.options.edge_speed_override_ranges.is_empty() {
            return duration_s;
        }
        let mut covered = 0.0f64;
        let mut overridden_s = 0.0;
        for range in self
            .options
            .edge_speed_override_ranges
            .iter()
            .filter(|range| range.edge_id == edge_id && valid(range.speed_kph))
        {
            let coverage = (range.end_fraction.clamp(0.0, 1.0)
                - range.start_fraction.clamp(0.0, 1.0))
            .max(0.0)
            .min(1.0 - covered);
            covered += coverage;
            overridden_s += distance_m * coverage / (range.speed_kph / 3.6);
        }
        if covered <= 0.0 {
            return duration_s;
        }
        duration_s * (1.0 - covered) + overridden_s
    }

    /// Whether the request opens the edge for its profile.
    pub(crate) fn edge_opened(&self, edge_id: u64) -> bool {
        self.options.opened_edge_ids.contains(&edge_id)
    }

    /// Whether the SAR `conditional` relaxation ignores conditional closures.
    pub(crate) fn ignores_conditional_access(&self) -> bool {
        self.options
            .sar
            .as_ref()
            .is_some_and(crate::sar::RouteSarOptions::ignores_conditional)
    }

    pub(crate) fn avoidance_penalty(&self, edge_id: u64) -> f64 {
        let whole_edge_penalty: f64 = if self.options.avoided_edge_ids.contains(&edge_id) {
            6.0
        } else {
            1.0
        };
        let ranged_penalty = self
            .options
            .avoided_edge_ranges
            .iter()
            .filter(|range| range.edge_id == edge_id)
            .map(|range| {
                let coverage = (range.end_fraction.clamp(0.0, 1.0)
                    - range.start_fraction.clamp(0.0, 1.0))
                .max(0.0);
                1.0 + (6.0 - 1.0) * coverage
            })
            .fold(1.0, f64::max);
        whole_edge_penalty.max(ranged_penalty)
    }
}

impl EdgeOverrides for CostingPolicy<'_> {
    fn edge_blocked(&self, edge_id: u64) -> bool {
        CostingPolicy::edge_blocked(self, edge_id)
    }

    fn avoidance_penalty(&self, edge_id: u64) -> f64 {
        CostingPolicy::avoidance_penalty(self, edge_id)
    }

    fn overlay_multiplier(&self, edge_id: u64) -> f64 {
        CostingPolicy::overlay_multiplier(self, edge_id)
    }

    fn duration_factor(&self, edge_id: u64) -> f64 {
        CostingPolicy::duration_factor(self, edge_id)
    }

    fn overridden_duration_s(&self, edge_id: u64, distance_m: f64, duration_s: f64) -> f64 {
        CostingPolicy::overridden_duration_s(self, edge_id, distance_m, duration_s)
    }

    fn edge_opened(&self, edge_id: u64) -> bool {
        CostingPolicy::edge_opened(self, edge_id)
    }

    fn ignores_conditional_access(&self) -> bool {
        CostingPolicy::ignores_conditional_access(self)
    }

    fn sar_relaxations(&self) -> Option<SarEdgeRelaxations> {
        self.sar
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LegRequest;
    use wayfarer_package::{
        DirectionalAccess, RoadEdge, RoadType, SharedEdgeInfo, TraversalDirection,
    };
    use wayfarer_road_graph::{RoadGraphBuilder, RoutableWay, enhance_road_graph};

    fn test_way(id: i64, surface: Option<&str>) -> RoutableWay {
        RoutableWay {
            osm_way_id: id,
            points: vec![(50.0, 20.0), (50.0, 20.001)],
            source_node_ids: Vec::new(),
            road_type: RoadType::Residential,
            directional_access: DirectionalAccess::Both,
            name: None,
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
            surface: surface.map(Into::into),
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
        }
    }

    fn first_edge_record(surface: Option<&str>) -> GraphViewEdge {
        let mut builder = RoadGraphBuilder::new("costing-test");
        builder.push_way(test_way(1, surface));
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);
        let engine = crate::test_support::engine(Box::leak(Box::new(graph)));
        let edge_id = engine
            .route_graph_view()
            .candidate_edges_near(crate::GeoPoint::new(50.0, 20.0005), 20)
            .first()
            .map(|record| record.edge.edge_id)
            .expect("edge");
        engine
            .route_graph_view()
            .search_edge_by_id(edge_id)
            .expect("edge record")
    }

    fn allowed_cost(policy: &CostingPolicy<'_>, record: &GraphViewEdge) -> CostedEdge {
        match policy.edge_cost(record) {
            EdgeCostDecision::Allowed(cost) => cost,
            other => panic!("expected allowed edge cost, got {other:?}"),
        }
    }

    fn typed_edge_record(
        highway: &str,
        road_type: RoadType,
        maxspeed: Option<&str>,
    ) -> GraphViewEdge {
        let mut way = test_way(1, Some("asphalt"));
        way.highway = Some(highway.into());
        way.road_type = road_type;
        way.maxspeed = maxspeed.map(Into::into);
        let mut builder = RoadGraphBuilder::new("costing-metric-test");
        builder.push_way(way);
        let mut graph = builder.finish();
        enhance_road_graph(&mut graph);
        let engine = crate::test_support::engine(Box::leak(Box::new(graph)));
        let edge_id = engine
            .route_graph_view()
            .candidate_edges_near(crate::GeoPoint::new(50.0, 20.0005), 20)
            .first()
            .map(|record| record.edge.edge_id)
            .expect("edge");
        engine
            .route_graph_view()
            .search_edge_by_id(edge_id)
            .expect("edge record")
    }

    #[test]
    fn walkway_cost_class_treats_bridleway_and_cycleway_as_footpaths() {
        assert_eq!(
            walkway_cost_class(Some("bridleway")),
            WalkwayCostClass::Footpath
        );
        assert_eq!(
            walkway_cost_class(Some("cycleway")),
            WalkwayCostClass::Footpath
        );
        assert_eq!(
            walkway_cost_class(Some("footway")),
            WalkwayCostClass::Footpath
        );
        assert_eq!(walkway_cost_class(Some("service")), WalkwayCostClass::Other);
    }

    #[test]
    fn faster_edge_costs_strictly_less_per_meter_at_equal_road_class() {
        // Phase 0 time metric: at the same road class (identical road_type_factor
        // and surface), a faster edge must cost strictly less per meter than a
        // slower one of equal length. Under the old distance-dominated metric the
        // profile_time_factor was clamped to a 1.0 floor, so every edge >= 30 km/h
        // cost exactly the same per meter regardless of speed. Both fixtures are
        // motorways (road_type_factor == 1.0) so only the speed term differs.
        let request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );
        let policy = CostingPolicy::new(MobilityProfile::Wheeled, &request.options);

        let fast = typed_edge_record("motorway", RoadType::Motorway, Some("130"));
        let slow = typed_edge_record("motorway", RoadType::Motorway, Some("35"));

        let fast_distance = f64::from(fast.edge.distance_m);
        let slow_distance = f64::from(slow.edge.distance_m);
        assert!(
            (fast_distance - slow_distance).abs() < 1e-6,
            "fixtures must be equal length: {fast_distance} vs {slow_distance}"
        );

        let fast_breakdown = allowed_cost(&policy, &fast).breakdown;
        let slow_breakdown = allowed_cost(&policy, &slow).breakdown;
        // Guard: the two fixtures differ ONLY in speed, not other factors.
        assert!((fast_breakdown.road_type_factor - slow_breakdown.road_type_factor).abs() < 1e-9);
        assert!((fast_breakdown.surface_factor - slow_breakdown.surface_factor).abs() < 1e-9);

        let fast_per_meter = fast_breakdown.total_cost / fast_distance;
        let slow_per_meter = slow_breakdown.total_cost / slow_distance;

        assert!(
            fast_per_meter < slow_per_meter,
            "fast motorway {fast_per_meter} should cost less per meter than slow {slow_per_meter}"
        );
    }

    fn transition_edge(id: u64, shared_id: u64, start: u16, end: u16) -> RoadEdge {
        RoadEdge {
            edge_id: id,
            shared_edge_info_id: shared_id,
            distance_m: 100.0,
            base_cost: 100.0,
            mobility_mode_mask: MobilityProfile::Wheeled.mobility_mode_mask(),
            osm_forward_mode_mask: MobilityProfile::Wheeled.mobility_mode_mask(),
            osm_reverse_mode_mask: MobilityProfile::Wheeled.mobility_mode_mask(),
            traversal_direction: TraversalDirection::Forward,
            segment_index: 0,
            component_id: None,
            start_heading_deg: Some(start),
            end_heading_deg: Some(end),
            inferred_speed_kph: None,
            estimated_travel_time_s: None,
            opposing_edge_id: None,
            hierarchy_tier: None,
            end_control: Default::default(),
            end_minor_approach: false,
        }
    }

    fn transition_info(shared_id: u64, is_link: bool) -> SharedEdgeInfo {
        transition_info_with_type(shared_id, is_link, RoadType::Primary)
    }

    fn transition_info_with_type(
        shared_id: u64,
        is_link: bool,
        road_type: RoadType,
    ) -> SharedEdgeInfo {
        SharedEdgeInfo {
            shared_edge_info_id: shared_id,
            road_type,
            is_link,
            ..SharedEdgeInfo::default()
        }
    }

    #[test]
    fn transition_cost_reduces_non_uturn_link_penalty() {
        let request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );
        let policy = CostingPolicy::new(MobilityProfile::Wheeled, &request.options);
        let previous = transition_edge(1, 10, 0, 120);
        let next = transition_edge(2, 20, 0, 0);

        // `transition_cost` probes the raw (legacy distance-domain) heading cost.
        assert_eq!(policy.transition_cost(&previous, &next), 90.0);
        // Phase 0: the composed transition cost is converted to seconds
        // (legacy 45.0 * 0.12 s/unit = 5.4 s).
        let previous_info = transition_info(10, false);
        let next_info = transition_info(20, true);
        assert_eq!(
            policy.transition_cost_with_shared_info(
                &previous,
                &previous_info,
                &next,
                &next_info,
                &|| { same_signed_corridor(&previous_info, &next_info) }
            ),
            45.0 * TURN_COST_SECONDS_PER_LEGACY_UNIT
        );
    }

    #[test]
    fn transition_cost_charges_a_u_turn_onto_the_opposing_edge() {
        let request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );
        let policy = CostingPolicy::new(MobilityProfile::Wheeled, &request.options);
        let info = transition_info(10, false);
        let forward = transition_edge(1, 10, 90, 90);
        let mut backward = transition_edge(2, 10, 270, 270);
        backward.traversal_direction = TraversalDirection::Backward;
        let cost = |previous: &RoadEdge, next: &RoadEdge| {
            policy.transition_cost_with_shared_info(previous, &info, next, &info, &|| true)
        };
        // Straight on along one segment (a closed loop) is free; reversing
        // onto its opposing edge costs what any U-turn does.
        assert_eq!(cost(&forward, &forward), 0.0);
        assert_eq!(
            cost(&forward, &backward),
            U_TURN_LEGACY_COST * TURN_COST_SECONDS_PER_LEGACY_UNIT
        );
        assert_eq!(cost(&backward, &forward), cost(&forward, &backward));
    }

    #[test]
    fn transition_cost_keeps_link_uturn_penalty_authoritative() {
        let request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );
        let policy = CostingPolicy::new(MobilityProfile::Wheeled, &request.options);
        let previous = transition_edge(1, 10, 0, 180);
        let next = transition_edge(2, 20, 0, 0);

        let previous_info = transition_info(10, true);
        let next_info = transition_info(20, false);
        assert_eq!(
            policy.transition_cost_with_shared_info(
                &previous,
                &previous_info,
                &next,
                &next_info,
                &|| { same_signed_corridor(&previous_info, &next_info) }
            ),
            // Phase 0: legacy 180.0 converted to seconds.
            180.0 * TURN_COST_SECONDS_PER_LEGACY_UNIT
        );
    }

    #[test]
    fn transition_cost_penalizes_non_link_turn_from_major_to_minor_road() {
        let request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );
        let policy = CostingPolicy::new(MobilityProfile::Wheeled, &request.options);
        let previous = transition_edge(1, 10, 0, 120);
        let next = transition_edge(2, 20, 0, 0);

        let previous_info = transition_info_with_type(10, false, RoadType::Primary);
        let next_info = transition_info_with_type(20, false, RoadType::Residential);
        assert_eq!(
            policy.transition_cost_with_shared_info(
                &previous,
                &previous_info,
                &next,
                &next_info,
                &|| { same_signed_corridor(&previous_info, &next_info) }
            ),
            // Phase 0: legacy 144.0 converted to seconds.
            144.0 * TURN_COST_SECONDS_PER_LEGACY_UNIT
        );
    }

    #[test]
    fn transition_cost_relief_for_same_signed_corridor() {
        let request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );
        let policy = CostingPolicy::new(MobilityProfile::Wheeled, &request.options);
        let previous = transition_edge(1, 10, 0, 70);
        let next = transition_edge(2, 20, 0, 0);
        let mut previous_info = transition_info(10, false);
        previous_info.ref_name = Some("DK7".into());
        let mut next_info = transition_info(20, false);
        next_info.ref_name = Some("dk7".into());

        assert_eq!(
            policy.transition_cost_with_shared_info(
                &previous,
                &previous_info,
                &next,
                &next_info,
                &|| { same_signed_corridor(&previous_info, &next_info) }
            ),
            // Phase 0: legacy 24.5 converted to seconds.
            24.5 * TURN_COST_SECONDS_PER_LEGACY_UNIT
        );
    }

    #[test]
    fn costing_penalizes_destination_gate_and_access_metadata() {
        let mut record = first_edge_record(Some("asphalt"));
        record.shared_info.is_destination_only = true;
        record.shared_info.has_gate = true;
        record.shared_info.is_access_restricted = true;

        let mut request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );
        request.options.costing.avoid_destination_only = 1.0;
        request.options.costing.avoid_gates = 1.0;
        request.options.costing.avoid_access_restricted = 1.0;

        let strict_policy =
            CostingPolicy::with_destination_only_policy(request.profile, &request.options, false);
        assert_eq!(
            strict_policy.edge_cost(&record),
            EdgeCostDecision::NotAccessible
        );

        let relaxed_policy =
            CostingPolicy::with_destination_only_policy(request.profile, &request.options, true);
        let cost = allowed_cost(&relaxed_policy, &record);
        assert!(cost.is_destination_only_penalized);
        assert!(cost.has_gate_penalty);
        assert!(cost.has_access_restriction_penalty);
        assert!(cost.breakdown.destination_only_penalty > 1.0);
        assert!(cost.breakdown.gate_penalty > 1.0);
        assert!(cost.breakdown.access_restriction_penalty > 1.0);
    }

    #[test]
    fn costing_uses_surface_and_profile_preferences() {
        let paved = first_edge_record(Some("asphalt"));
        let muddy = first_edge_record(Some("mud"));
        let mut request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );
        request.options.costing.prefer_paved = 1.0;

        let wheeled_policy = CostingPolicy::new(MobilityProfile::Wheeled, &request.options);
        let tracked_policy = CostingPolicy::new(MobilityProfile::Tracked, &request.options);
        let wheeled_paved = allowed_cost(&wheeled_policy, &paved);
        let wheeled_muddy = allowed_cost(&wheeled_policy, &muddy);
        let tracked_muddy = allowed_cost(&tracked_policy, &muddy);

        assert!(wheeled_muddy.cost > wheeled_paved.cost);
        assert!(wheeled_muddy.duration_s > wheeled_paved.duration_s);
        assert!(
            tracked_muddy.duration_s < wheeled_muddy.duration_s,
            "tracked profile should be less damaged by mud than wheeled"
        );
    }

    #[test]
    fn speed_preference_selects_requested_vehicle_speed_source() {
        let mut record = first_edge_record(Some("asphalt"));
        record.shared_info.maxspeed_kph = Some(60.0);
        record.edge.inferred_speed_kph = Some(45);
        record.shared_info.free_flow_speed_kph = Some(90.0);
        record.shared_info.constrained_speed_kph = Some(30.0);

        assert_eq!(
            selected_vehicle_speed_source(
                &record.edge,
                &record.shared_info,
                &RouteCostingOptions {
                    speed_preference: RouteSpeedPreference::PostedOrInferred,
                    ..RouteCostingOptions::default()
                },
            ),
            crate::types::RouteSpeedSource::Posted
        );
        assert_eq!(
            selected_vehicle_speed_source(
                &record.edge,
                &record.shared_info,
                &RouteCostingOptions {
                    speed_preference: RouteSpeedPreference::FreeFlow,
                    ..RouteCostingOptions::default()
                },
            ),
            crate::types::RouteSpeedSource::FreeFlow
        );
        assert_eq!(
            selected_vehicle_speed_source(
                &record.edge,
                &record.shared_info,
                &RouteCostingOptions {
                    speed_preference: RouteSpeedPreference::Constrained,
                    ..RouteCostingOptions::default()
                },
            ),
            crate::types::RouteSpeedSource::Constrained
        );
        assert_eq!(
            selected_vehicle_speed_source(
                &record.edge,
                &record.shared_info,
                &RouteCostingOptions {
                    speed_preference: RouteSpeedPreference::RoadTypeDefault,
                    ..RouteCostingOptions::default()
                },
            ),
            crate::types::RouteSpeedSource::Default
        );

        let mut request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );
        let posted_duration =
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_duration_s(&record);

        request.options.costing.speed_preference = RouteSpeedPreference::FreeFlow;
        let free_flow_duration =
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_duration_s(&record);

        request.options.costing.speed_preference = RouteSpeedPreference::Constrained;
        let constrained_duration =
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_duration_s(&record);

        assert!(free_flow_duration < posted_duration);
        assert!(constrained_duration > posted_duration);
    }

    #[test]
    fn fixed_and_top_speed_overrides_bound_vehicle_duration() {
        let mut record = first_edge_record(Some("asphalt"));
        record.shared_info.maxspeed_kph = Some(100.0);
        record.edge.inferred_speed_kph = Some(80);
        record.shared_info.free_flow_speed_kph = Some(120.0);

        let mut request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );
        request.options.costing.speed_preference = RouteSpeedPreference::FreeFlow;
        let free_flow_duration =
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_duration_s(&record);

        request.options.costing.top_speed_kph = Some(50.0);
        let capped_duration =
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_duration_s(&record);
        assert!(capped_duration > free_flow_duration);
        assert_eq!(
            selected_vehicle_speed_source(
                &record.edge,
                &record.shared_info,
                &request.options.costing
            ),
            crate::types::RouteSpeedSource::FreeFlow
        );

        request.options.costing.fixed_speed_kph = Some(30.0);
        let fixed_duration =
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_duration_s(&record);
        assert!(fixed_duration > capped_duration);
        assert_eq!(
            selected_vehicle_speed_source(
                &record.edge,
                &record.shared_info,
                &request.options.costing
            ),
            crate::types::RouteSpeedSource::Fixed
        );
    }

    #[test]
    fn active_conditional_access_blocks_matching_profile() {
        let mut record = first_edge_record(Some("asphalt"));
        record.shared_info.conditional_restrictions =
            vec!["access:conditional=no @ (2026 May 01-2026 Jun 30)".to_string()];
        let mut request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );

        assert!(matches!(
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_cost(&record),
            EdgeCostDecision::Allowed(_)
        ));

        request.options.costing.conditional_restriction_eval_at_ms = Some(1_778_832_000_000);
        assert_eq!(
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_cost(&record),
            EdgeCostDecision::NotAccessible
        );

        request.options.costing.conditional_restriction_eval_at_ms = Some(1_782_892_800_000);
        assert!(matches!(
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_cost(&record),
            EdgeCostDecision::Allowed(_)
        ));
    }

    #[test]
    fn active_conditional_destination_access_makes_the_edge_destination_only() {
        let mut record = first_edge_record(Some("asphalt"));
        record.shared_info.conditional_restrictions =
            vec!["motor_vehicle:conditional=destination @ (2026 May 01-2026 Jun 30)".to_string()];
        let mut request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );
        request.options.costing.conditional_restriction_eval_at_ms = Some(1_778_832_000_000);
        // Usable to reach somewhere on it, not closed ...
        let EdgeCostDecision::Allowed(plain) =
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_cost(&record)
        else {
            panic!("a destination-only edge was closed");
        };
        // ... but not on the strict pass, and penalized when avoided.
        assert_eq!(
            CostingPolicy::with_destination_only_policy(
                MobilityProfile::Wheeled,
                &request.options,
                false
            )
            .edge_cost(&record),
            EdgeCostDecision::NotAccessible
        );
        request.options.costing.avoid_destination_only = 1.0;
        let EdgeCostDecision::Allowed(avoided) =
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_cost(&record)
        else {
            panic!("a destination-only edge was closed");
        };
        assert!(avoided.is_destination_only_penalized);
        assert!(avoided.cost > plain.cost);
        // Outside the window the edge is a plain one again.
        request.options.costing.conditional_restriction_eval_at_ms = Some(1_782_892_800_000);
        let EdgeCostDecision::Allowed(outside) =
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_cost(&record)
        else {
            panic!("the edge was closed outside the window");
        };
        assert!(!outside.is_destination_only_penalized);
    }

    #[test]
    fn active_conditional_access_supports_yearless_seasonal_date_ranges() {
        let mut record = first_edge_record(Some("asphalt"));
        record.shared_info.conditional_restrictions =
            vec!["access:conditional=no @ (May 01-Jun 30)".to_string()];
        let mut request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );

        request.options.costing.conditional_restriction_eval_at_ms = Some(1_778_832_000_000);
        assert_eq!(
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_cost(&record),
            EdgeCostDecision::NotAccessible
        );

        request.options.costing.conditional_restriction_eval_at_ms = Some(1_782_892_800_000);
        assert!(matches!(
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_cost(&record),
            EdgeCostDecision::Allowed(_)
        ));
    }

    #[test]
    fn active_conditional_access_supports_wrapping_seasonal_date_ranges() {
        let mut record = first_edge_record(Some("asphalt"));
        record.shared_info.conditional_restrictions =
            vec!["access:conditional=no @ (Nov 01-Mar 31)".to_string()];
        let mut request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );

        // 2026-01-15T08:00:00Z is inside the winter wrapping range.
        request.options.costing.conditional_restriction_eval_at_ms = Some(1_768_464_000_000);
        assert_eq!(
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_cost(&record),
            EdgeCostDecision::NotAccessible
        );

        // 2026-07-01T08:00:00Z is outside the winter wrapping range.
        request.options.costing.conditional_restriction_eval_at_ms = Some(1_782_892_800_000);
        assert!(matches!(
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_cost(&record),
            EdgeCostDecision::Allowed(_)
        ));
    }

    #[test]
    fn active_conditional_access_supports_weekday_lists() {
        let mut record = first_edge_record(Some("asphalt"));
        record.shared_info.conditional_restrictions =
            vec!["access:conditional=no @ (Mo,We,Fr 07:00-09:00)".to_string()];
        let mut request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );

        // 2026-05-20T08:00:00Z is Wednesday inside the listed weekday/time window.
        request.options.costing.conditional_restriction_eval_at_ms = Some(1_779_264_000_000);
        assert_eq!(
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_cost(&record),
            EdgeCostDecision::NotAccessible
        );

        // 2026-05-19T08:00:00Z is Tuesday and should not match the listed days.
        request.options.costing.conditional_restriction_eval_at_ms = Some(1_779_177_600_000);
        assert!(matches!(
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_cost(&record),
            EdgeCostDecision::Allowed(_)
        ));
    }

    #[test]
    fn active_conditional_access_supports_multiple_time_windows() {
        let mut record = first_edge_record(Some("asphalt"));
        record.shared_info.conditional_restrictions =
            vec!["access:conditional=no @ (Mo-Fr 07:00-09:00,16:00-18:00)".to_string()];
        let mut request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );

        // 2026-05-19T08:30:00Z is Tuesday inside the morning window.
        request.options.costing.conditional_restriction_eval_at_ms = Some(1_779_179_400_000);
        assert_eq!(
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_cost(&record),
            EdgeCostDecision::NotAccessible
        );

        // 2026-05-19T16:30:00Z is Tuesday inside the afternoon window.
        request.options.costing.conditional_restriction_eval_at_ms = Some(1_779_208_200_000);
        assert_eq!(
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_cost(&record),
            EdgeCostDecision::NotAccessible
        );

        // 2026-05-19T12:00:00Z is Tuesday but outside both listed windows.
        request.options.costing.conditional_restriction_eval_at_ms = Some(1_779_192_000_000);
        assert!(matches!(
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_cost(&record),
            EdgeCostDecision::Allowed(_)
        ));
    }

    #[test]
    fn active_conditional_access_supports_semicolon_clauses() {
        let mut record = first_edge_record(Some("asphalt"));
        record.shared_info.conditional_restrictions = vec![
            "access:conditional=yes @ (Mo-Fr 00:00-23:59); no @ (Sa-Su 00:00-23:59)".to_string(),
        ];
        let mut request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );

        // 2026-05-23T08:00:00Z is Saturday and should match the second clause.
        request.options.costing.conditional_restriction_eval_at_ms = Some(1_779_523_200_000);
        assert_eq!(
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_cost(&record),
            EdgeCostDecision::NotAccessible
        );

        // 2026-05-20T08:00:00Z is Wednesday and should match only the explicit yes clause.
        request.options.costing.conditional_restriction_eval_at_ms = Some(1_779_264_000_000);
        assert!(matches!(
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_cost(&record),
            EdgeCostDecision::Allowed(_)
        ));
    }

    #[test]
    fn active_conditional_maxspeed_overrides_vehicle_duration() {
        let mut record = first_edge_record(Some("asphalt"));
        record.shared_info.maxspeed_kph = Some(80.0);
        record.shared_info.conditional_restrictions =
            vec!["maxspeed:conditional=30 @ (Mo-Fr 07:00-09:00)".to_string()];
        let mut request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );
        let normal_duration =
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_duration_s(&record);

        request.options.costing.conditional_restriction_eval_at_ms = Some(1_779_091_200_000);
        let active_policy = CostingPolicy::new(MobilityProfile::Wheeled, &request.options);
        let conditional_duration = active_policy.edge_duration_s(&record);
        assert!(conditional_duration > normal_duration);
        assert_eq!(
            selected_vehicle_speed_source(
                &record.edge,
                &record.shared_info,
                &request.options.costing
            ),
            crate::types::RouteSpeedSource::Conditional
        );

        request.options.costing.conditional_restriction_eval_at_ms = Some(1_779_098_400_000);
        let inactive_duration =
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_duration_s(&record);
        assert_eq!(inactive_duration, normal_duration);
    }

    #[test]
    fn active_conditional_maxspeed_supports_semicolon_clauses() {
        let mut record = first_edge_record(Some("asphalt"));
        record.shared_info.maxspeed_kph = Some(80.0);
        record.shared_info.conditional_restrictions = vec![
            "maxspeed:conditional=30 @ (Mo-Fr 07:00-09:00); 50 @ (Mo-Fr 16:00-18:00)".to_string(),
        ];
        let mut request = LegRequest::new(
            crate::GeoPoint::new(50.0, 20.0),
            crate::GeoPoint::new(50.0, 20.001),
            MobilityProfile::Wheeled,
        );
        let normal_duration =
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_duration_s(&record);

        // 2026-05-19T16:30:00Z is Tuesday inside the second semicolon clause.
        request.options.costing.conditional_restriction_eval_at_ms = Some(1_779_208_200_000);
        let afternoon_duration =
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_duration_s(&record);
        assert!(afternoon_duration > normal_duration);

        // 2026-05-19T12:00:00Z is Tuesday but outside both semicolon clauses.
        request.options.costing.conditional_restriction_eval_at_ms = Some(1_779_192_000_000);
        let inactive_duration =
            CostingPolicy::new(MobilityProfile::Wheeled, &request.options).edge_duration_s(&record);
        assert_eq!(inactive_duration, normal_duration);
    }
}

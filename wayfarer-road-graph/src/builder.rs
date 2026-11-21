//! [`RoadGraphBuilder`]: pushed ways and traffic-control points to a
//! [`RoadGraph`].

use super::*;

const ROUTABLE_WAY_CHUNK_LEN: usize = 16 * 1024;

/// A pushed way as `finish` reads it. A `RoutableWay` is ~780 bytes of
/// attributes, and the compiler pushes one per topology segment, so a region
/// held ~1M copies of mostly the same attributes. Pushes with equal attributes
/// (once the way id and lineage are set aside) share one copy: ~60k distinct
/// sets for ~1M pushes on a region.
#[derive(Debug)]
struct PendingWay {
    /// The way without its id, lineage, `points` and `source_node_ids`.
    attrs: Arc<RoutableWay>,
    osm_way_id: i64,
    /// The way's sorted, distinct lineage; empty for the common `[osm_way_id]`.
    source_lineage_ids: Box<[i64]>,
    points: PendingPoints,
    /// `(point index, node id)` for the points that have a source node id, in
    /// point order: usually only the two endpoints.
    source_node_ids: Box<[(u32, i64)]>,
}

/// A pending way's points: E7 pairs (half the size) when every point is the
/// `f64` of its E7 value, as the compiler's are; the points as given
/// otherwise.
#[derive(Debug)]
enum PendingPoints {
    Fixed(Box<[(i32, i32)]>),
    Exact(Box<[(f64, f64)]>),
}

impl PendingPoints {
    fn new(points: Vec<(f64, f64)>) -> Self {
        let fixed = points
            .iter()
            .map(|&(lat, lng)| (to_fixed(lat), to_fixed(lng)))
            .collect::<Vec<_>>();
        let exact = points
            .iter()
            .zip(&fixed)
            .all(|(&(lat, lng), &(lat7, lng7))| {
                from_fixed(lat7).to_bits() == lat.to_bits()
                    && from_fixed(lng7).to_bits() == lng.to_bits()
            });
        if exact {
            Self::Fixed(fixed.into_boxed_slice())
        } else {
            Self::Exact(points.into_boxed_slice())
        }
    }

    fn len(&self) -> usize {
        match self {
            Self::Fixed(points) => points.len(),
            Self::Exact(points) => points.len(),
        }
    }

    /// The point at `index` in fixed point (`to_fixed` of each coordinate).
    fn fixed(&self, index: usize) -> (i32, i32) {
        match self {
            Self::Fixed(points) => points[index],
            Self::Exact(points) => (to_fixed(points[index].0), to_fixed(points[index].1)),
        }
    }

    /// The points as given, into `buffer`.
    fn read_into<'a>(&self, buffer: &'a mut Vec<(f64, f64)>) -> &'a [(f64, f64)] {
        buffer.clear();
        match self {
            Self::Fixed(points) => buffer.extend(
                points
                    .iter()
                    .map(|&(lat, lng)| (from_fixed(lat), from_fixed(lng))),
            ),
            Self::Exact(points) => buffer.extend_from_slice(points),
        }
        buffer
    }
}

impl PendingWay {
    /// `source_node_ids` expanded to one entry per point, into `buffer`.
    fn expand_source_node_ids(&self, buffer: &mut Vec<Option<i64>>) {
        buffer.clear();
        buffer.resize(self.points.len(), None);
        for &(index, node_id) in &self.source_node_ids {
            buffer[index as usize] = Some(node_id);
        }
    }
}

#[derive(Debug, Default)]
pub struct RoadGraphBuilder {
    graph_id: String,
    country_code: Option<String>,
    /// Pushed ways in fixed-size chunks: `finish` consumes them in order and
    /// frees each chunk as it goes, instead of holding every (wide) way until
    /// the whole graph has been built.
    ways: Vec<Vec<PendingWay>>,
    /// Every pushed way point's coordinate ([`point_key`]), one entry per
    /// point. `finish` sorts them to find the coordinates more than one way
    /// point shares: 8 bytes a point here, where a map counting every
    /// coordinate as the ways arrived held about 25 and doubled as it grew.
    point_keys: Vec<u64>,
    /// The distinct pushed attributes (see [`PendingWay::attrs`]), by
    /// [`way_attrs_hash`].
    way_attrs: FxHashMap<u64, Vec<Arc<RoutableWay>>>,
    next_edge_id: u64,
    next_shared_edge_info_id: u64,
    next_synthetic_plaza_way_id: i64,
    barrier_nodes: HashMap<i64, NodeBarrierTags>,
    traffic_controls: FxHashMap<(i32, i32), TrafficControlPoint>,
    report: RoadGraphBuildReport,
}

#[derive(Debug, Clone, Copy, Default)]
struct PointCounts {
    points: u32,
    motor_arms: u32,
}

/// Which way along its way a traffic control faces (OSM
/// `traffic_signals:direction` or `direction`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TrafficControlDirection {
    #[default]
    Both,
    Forward,
    Backward,
}

/// A traffic control node: signals, a stop or give-way sign, or a crossing
/// with signals, at a position (an OSM node of some way).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrafficControlPoint {
    pub lat: f64,
    pub lng: f64,
    pub control: TrafficControl,
    pub direction: TrafficControlDirection,
}

/// A control on a way's stretch within this distance of a junction of motor
/// roads belongs to that junction: it controls the traffic approaching the
/// junction, not the traffic leaving it (stop lines and crossings are a few
/// metres before the junction node). An edge this short between two
/// signalised junction nodes is inside one junction and is not signalled
/// again.
pub const JUNCTION_CONTROL_RANGE_M: f64 = 35.0;

impl RoadGraphBuilder {
    pub fn new(graph_id: impl Into<String>) -> Self {
        Self {
            graph_id: graph_id.into(),
            country_code: None,
            ways: Vec::new(),
            point_keys: Vec::new(),
            way_attrs: FxHashMap::default(),
            next_edge_id: 1,
            next_shared_edge_info_id: 1,
            next_synthetic_plaza_way_id: SYNTHETIC_PLAZA_WAY_ID_BASE,
            barrier_nodes: HashMap::new(),
            traffic_controls: FxHashMap::default(),
            report: RoadGraphBuildReport::default(),
        }
    }

    /// Register traffic control nodes. During [`RoadGraphBuilder::finish`]
    /// each directed edge takes the strongest control a vehicle meets before
    /// leaving it at its end: one at its end node (unless it faces the other
    /// way), or one on its stretch that faces its direction. A control on the
    /// stretch that faces both ways goes to the direction approaching the
    /// junction of motor roads it is near (within
    /// [`JUNCTION_CONTROL_RANGE_M`]), and to both directions elsewhere.
    /// Crossing signals stay crossing signals near a junction: where the
    /// junction is signalised its own signals are mapped too (a stronger
    /// control on the same edge), and where it is not they are a pedestrian
    /// crossing beside a side street.
    pub fn set_traffic_control_points(&mut self, points: Vec<TrafficControlPoint>) {
        self.traffic_controls = points
            .into_iter()
            .map(|point| ((to_fixed(point.lat), to_fixed(point.lng)), point))
            .collect();
    }

    /// Register barrier node tags keyed by OSM node id. During [`RoadGraphBuilder::finish`], any
    /// registered node that threads through a road way's interior is turned into
    /// a mode-aware crossing constraint (see [`derive_node_barrier`]).
    pub fn set_barrier_node_tags(&mut self, barrier_nodes: HashMap<i64, NodeBarrierTags>) {
        self.barrier_nodes = barrier_nodes;
    }

    /// Set the ISO 3166-1 alpha-2 country used to resolve implicit `maxspeed`
    /// values while baking edge speeds. Defaults to `None` (countryless
    /// keywords only).
    pub fn with_country_code(mut self, country_code: Option<String>) -> Self {
        self.country_code = country_code;
        self
    }

    pub fn push_way(&mut self, mut way: RoutableWay) {
        if way.points.len() < 2 {
            self.report.dropped_short_way_count += 1;
            self.report.record_rejected_way("short");
            return;
        }
        if way.directional_access == DirectionalAccess::Closed {
            self.report.dropped_closed_way_count += 1;
            self.report.record_rejected_way("closed");
            return;
        }
        let mode_mask = way_mode_mask(&way);
        // A drivable way the law closes to every mode is kept for SAR (the
        // physical graph): its edges have no
        // legal access, so normal routing never uses them.
        let sar_only =
            mode_mask == 0 && !way.is_ferry && vehicle_drivable_class(way.highway.as_deref());
        if way.access_restricted
            && !matches!(way.access.as_deref(), Some("permit" | "destination"))
            && mode_mask == 0
            && !sar_only
        {
            self.report.dropped_restricted_way_count += 1;
            self.report.record_rejected_way("restricted");
            return;
        }
        // Construction, proposed and other ways no mode may use by default.
        if mode_mask == 0 && !sar_only {
            self.report.record_rejected_way("no_mode");
            return;
        }
        if sar_only {
            self.report.sar_only_way_count += 1;
        } else {
            self.report.record_accepted_way(way.road_type);
        }

        if way.source_lineage_ids.is_empty() {
            way.source_lineage_ids.push(way.osm_way_id);
        }
        way.source_lineage_ids.sort_unstable();
        way.source_lineage_ids.dedup();
        // Ids that do not match the points one to one are ignored.
        let source_node_ids = if way.source_node_ids.len() == way.points.len() {
            let mut ids = Vec::with_capacity(way.source_node_ids.iter().flatten().count());
            ids.extend(
                way.source_node_ids
                    .iter()
                    .enumerate()
                    .filter_map(|(index, node_id)| node_id.map(|node_id| (index as u32, node_id))),
            );
            ids
        } else {
            Vec::new()
        };
        way.source_node_ids = Vec::new();
        let points = PendingPoints::new(std::mem::take(&mut way.points));

        self.point_keys
            .extend((0..points.len()).map(|index| point_key(points.fixed(index))));

        let osm_way_id = way.osm_way_id;
        let lineage = std::mem::take(&mut way.source_lineage_ids);
        let source_lineage_ids = if lineage == [osm_way_id] {
            Box::default()
        } else {
            lineage.into_boxed_slice()
        };
        way.osm_way_id = 0;
        let bucket = self.way_attrs.entry(way_attrs_hash(&way)).or_default();
        let attrs = match bucket.iter().find(|attrs| ***attrs == way) {
            Some(attrs) => Arc::clone(attrs),
            None => {
                let attrs = Arc::new(way);
                bucket.push(Arc::clone(&attrs));
                attrs
            }
        };
        let way = PendingWay {
            attrs,
            osm_way_id,
            source_lineage_ids,
            points,
            source_node_ids: source_node_ids.into_boxed_slice(),
        };

        match self.ways.last_mut() {
            Some(chunk) if chunk.len() < ROUTABLE_WAY_CHUNK_LEN => chunk.push(way),
            _ => {
                let mut chunk = Vec::with_capacity(ROUTABLE_WAY_CHUNK_LEN);
                chunk.push(way);
                self.ways.push(chunk);
            }
        }
    }

    pub fn finish(self) -> RoadGraph {
        self.finish_with_report().0
    }

    pub fn finish_with_report(mut self) -> (RoadGraph, RoadGraphBuildReport) {
        let country_code = self.country_code.take();
        // Only the pending ways hold the attributes from here, so each is
        // freed with its last way's chunk.
        self.way_attrs = FxHashMap::default();
        let mut graph = RoadGraph::new(self.graph_id);
        graph.country_code = country_code.clone();
        if self.ways.iter().all(Vec::is_empty) {
            return (graph, self.report);
        }

        let mut node_map: FxHashMap<(i32, i32), NodeIndex> = FxHashMap::default();
        let mut get_or_create_node =
            |lat: f64, lng: f64, source_node_id: Option<i64>, graph: &mut RoadGraph| -> NodeIndex {
                let key = (to_fixed(lat), to_fixed(lng));
                if let Some(index) = node_map.get(&key).copied() {
                    if graph.graph.node_source_node_id(index.index()).is_none() {
                        graph
                            .graph
                            .set_node_source_node_id(index.index(), source_node_id);
                    }
                    index
                } else {
                    let index = graph.add_node(RoadNode {
                        id: graph.graph.node_count() as i64,
                        lat,
                        lng,
                        source_node_id,
                        ..Default::default()
                    });
                    node_map.insert(key, index);
                    index
                }
            };

        let mut plaza_accumulators: HashMap<i64, PlazaAccumulator> = HashMap::new();
        // Which way points close a segment because another way point shares
        // their coordinate. Resolved up front so the coordinate map is freed
        // before the graph grows, and so the graph's two largest vectors can be
        // sized once: grown by doubling, the shared-info vector held its old
        // and new buffers at once and ended up to half unused (~0.5 GiB on a
        // region), and the edge vector likewise.
        // Junctions of motor roads, per way point; only read to place traffic
        // controls.
        let place_controls = !self.traffic_controls.is_empty();
        let coord_counts = shared_coord_counts(
            std::mem::take(&mut self.point_keys),
            &self.ways,
            place_controls,
        );
        let mut shared_point = Vec::<bool>::new();
        let mut motor_junction_point = Vec::<bool>::new();
        let mut segment_estimate = 0usize;
        // Nodes sit at segment ends: each shared coordinate once, plus the
        // unshared way ends and barrier points (an upper bound).
        let mut node_estimate = coord_counts.len();
        let mut point_estimate = 0usize;
        for way in self.ways.iter().flatten() {
            let last = way.points.len().saturating_sub(1);
            if way.points.len() >= 2 {
                segment_estimate += 1;
                // The first segment's points; each further one adds its own.
                point_estimate += way.points.len() - 1;
            }
            for index in 0..way.points.len() {
                let counts = coord_counts
                    .get(&way.points.fixed(index))
                    .copied()
                    .unwrap_or_default();
                let shared = counts.points > 1;
                shared_point.push(shared);
                if place_controls {
                    motor_junction_point.push(counts.motor_arms >= 3);
                }
                if index == 0 || index >= last {
                    node_estimate += usize::from(!shared);
                    continue;
                }
                if shared {
                    segment_estimate += 1;
                } else if way
                    .source_node_ids
                    .binary_search_by_key(&(index as u32), |&(point, _)| point)
                    .is_ok_and(|position| {
                        self.barrier_nodes
                            .contains_key(&way.source_node_ids[position].1)
                    })
                {
                    // The barrier closes a segment.
                    segment_estimate += 1;
                    node_estimate += 1;
                }
            }
        }
        drop(coord_counts);
        // Plazas add a few more; the slack keeps them from forcing a doubling.
        let shared_info_estimate = segment_estimate + segment_estimate / 64 + 64;
        graph.shared_edge_infos.reserve_exact(shared_info_estimate);
        // A way's segments share their end points: each segment has one more
        // point than it adds to the way (plazas: two per spoke).
        graph.shared_edge_infos.reserve_points_exact(
            point_estimate + segment_estimate + 2 * (shared_info_estimate - segment_estimate),
        );
        graph.graph.reserve_exact_edges(2 * shared_info_estimate);
        // Grown by doubling, the node vector ended up to half unused.
        graph
            .graph
            .reserve_exact_nodes(node_estimate + node_estimate / 64 + 64);

        let mut point_cursor = 0usize;
        let mut source_node_ids = Vec::<Option<i64>>::new();
        let mut point_buffer = Vec::<(f64, f64)>::new();
        // The class of the last way's template. Consecutive pushes of one way
        // share their attributes (`PendingWay::attrs`), so its template is
        // interned once; the `Arc` is held so its address is not reused.
        let mut last_template = None::<(Arc<RoutableWay>, u32)>;
        for pending in self.ways.into_iter().flatten() {
            let way_point_start = point_cursor;
            point_cursor += pending.points.len();
            let points = pending.points.read_into(&mut point_buffer);
            if points.len() < 2 {
                continue;
            }
            pending.expand_source_node_ids(&mut source_node_ids);
            let way = &*pending.attrs;

            if is_pedestrian_area(way) {
                let accumulator =
                    plaza_accumulators
                        .entry(pending.osm_way_id)
                        .or_insert_with(|| PlazaAccumulator {
                            highway: way.highway.as_deref().map(str::to_string),
                            surface: way.surface.clone(),
                            allow_bicycle: plaza_allows_bicycle(way),
                            segments: Vec::new(),
                        });
                accumulator.segments.push(points.to_vec());
            }

            let road_factor = road_surface_factor(way);
            // Everything a segment's shared info takes from its way, parsed
            // once per way rather than once per segment.
            let (osm_forward_mode_mask, osm_reverse_mode_mask) =
                derive_osm_mode_masks_from_way(way);
            // Vehicles legally denied on a drivable way: its vehicle
            // directions get edges anyway (SAR only where no mode is left).
            let vehicle_denied = vehicle_legally_denied(
                way.is_ferry,
                way.highway.as_deref(),
                osm_forward_mode_mask | osm_reverse_mode_mask,
            );
            // Paths R5 can open (footways, cycleways, …: not steps) are
            // physically drivable in the directions their one-way allows,
            // also where no mode may legally use that direction.
            let physical = vehicle_denied
                || (path_physically_drivable(way.is_ferry, way.highway.as_deref())
                    && mobility_mode_mask_from_osm(osm_forward_mode_mask | osm_reverse_mode_mask)
                        & MobilityMode::WHEELED
                        == 0);
            let physical_forward = physical && way.directional_access.allows_forward();
            let physical_backward = physical && way.directional_access.allows_backward();
            let legal_reason = LegalReason::from_tags(
                way.access.as_deref(),
                way.vehicle.as_deref(),
                way.motor_vehicle.as_deref(),
                way.motorcar.as_deref(),
                &[],
                vehicle_denied,
            )
            .0;
            let hierarchy_tier = Some(base_hierarchy_tier(way.road_type));
            let maxspeed_kph =
                parse_maxspeed_kph_with_country(way.maxspeed.as_deref(), country_code.as_deref());
            let inferred_speed_kph = infer_speed_kph_for_facts(
                way.road_type,
                maxspeed_kph,
                way.surface.as_deref(),
                way.tracktype.as_deref(),
                way.smoothness.as_deref(),
                way.has_ford,
            );
            let shared_info_template = SharedEdgeInfo {
                // Set per segment.
                shared_edge_info_id: 0,
                osm_way_id: pending.osm_way_id,
                road_type: way.road_type,
                geometry: Vec::new(),
                directional_access: way.directional_access,
                highway: way.highway.clone(),
                name: way.name.clone(),
                ref_name: way.ref_name.clone(),
                route_refs: Vec::new(),
                access: way.access.clone(),
                service: way.service.clone(),
                surface: way.surface.clone(),
                tracktype: way.tracktype.clone(),
                smoothness: way.smoothness.clone(),
                maxspeed_kph,
                maxspeed_forward_kph: parse_maxspeed_kph_with_country(
                    way.maxspeed_forward.as_deref(),
                    country_code.as_deref(),
                ),
                maxspeed_backward_kph: parse_maxspeed_kph_with_country(
                    way.maxspeed_backward.as_deref(),
                    country_code.as_deref(),
                ),
                hgv_maxspeed_kph: parse_maxspeed_kph_with_country(
                    way.maxspeed_hgv.as_deref(),
                    country_code.as_deref(),
                ),
                max_height_cm: parse_length_cm(way.maxheight.as_deref()),
                max_width_cm: parse_length_cm(way.maxwidth.as_deref()),
                physical_width_cm: parse_length_cm(way.width.as_deref()),
                max_weight_kg: parse_mass_kg(way.maxweight.as_deref()),
                max_axle_load_kg: parse_mass_kg(way.maxaxleload.as_deref()),
                incline_percent: parse_incline_percent(way.incline.as_deref()),
                is_4wd_only: way.is_4wd_only,
                is_motorroad: oneway_value_is_yes(way.motorroad.as_deref()),
                is_lit: way.is_lit,
                sac_scale: way.sac_scale.clone(),
                mtb_scale: way.mtb_scale.clone(),
                trail_visibility: way.trail_visibility.clone(),
                sidewalk: way.sidewalk.clone(),
                crossing: way.crossing.clone(),
                lane_count: parse_lane_count(way.lanes.as_deref()),
                lanes: way.lanes.clone(),
                turn_lanes: None,
                turn_lanes_forward: None,
                turn_lanes_backward: None,
                lane_connectivity_available: false,
                speed_source: Some(speed_source_for_values(maxspeed_kph)),
                free_flow_speed_kph: Some(inferred_speed_kph as f32),
                constrained_speed_kph: Some(constrained_speed_kph(inferred_speed_kph)),
                admin_country_id: None,
                admin_region_id: None,
                timezone_id: None,
                conditional_restrictions: way
                    .conditional_restrictions
                    .iter()
                    .map(|value| value.to_string())
                    .collect(),
                layer: parse_layer(way.layer.as_deref()),
                level: way.level.clone(),
                destination: way.destination.clone(),
                destination_ref: way.destination_ref.clone(),
                destination_street: way.destination_street.clone(),
                junction_ref: way.junction_ref.clone(),
                is_toll: way.is_toll,
                is_ferry: way.is_ferry,
                has_gate: way.has_gate,
                is_access_restricted: way.is_access_restricted,
                is_link: way.is_link,
                is_roundabout: way.is_roundabout,
                is_bridge: way.has_bridge,
                is_tunnel: way.has_tunnel,
                has_ford: way.has_ford,
                is_service_like: way.service.is_some()
                    || matches!(way.highway.as_deref(), Some("service")),
                is_destination_only: way.is_destination_only,
                osm_forward_mode_mask,
                osm_reverse_mode_mask,
                hierarchy_tier,
                legal_reason,
            };
            let mut current_segment_points = Vec::with_capacity(points.len());
            current_segment_points.push(points[0]);
            let mut segment_start = 0usize;
            let mut start_node_idx =
                get_or_create_node(points[0].0, points[0].1, source_node_ids[0], &mut graph);
            let mut segment_index = 0u32;

            for (index, &(lat, lng)) in points.iter().enumerate().skip(1) {
                current_segment_points.push((lat, lng));

                let is_last = index == points.len() - 1;
                let is_intersection = shared_point[way_point_start + index];

                // A barrier with an effect inside a way becomes a graph node
                // (the way is split there), so transitions through it can
                // apply it; see the barrier pass below.
                let barrier_split = !is_last
                    && !is_intersection
                    && source_node_ids[index]
                        .and_then(|node_id| self.barrier_nodes.get(&node_id))
                        .is_some_and(barrier_node_applies);

                if !is_last && !is_intersection && !barrier_split {
                    continue;
                }

                let end_source_node_id = source_node_ids[index];
                let end_node_idx = get_or_create_node(lat, lng, end_source_node_id, &mut graph);
                let distance = calculate_path_distance(&current_segment_points);
                let (forward_control, backward_control) = if place_controls {
                    segment_end_controls(
                        &current_segment_points,
                        &self.traffic_controls,
                        motor_junction_point[way_point_start + segment_start],
                        motor_junction_point[way_point_start + index],
                    )
                } else {
                    (TrafficControl::None, TrafficControl::None)
                };
                self.report.traffic_controlled_edge_count += usize::from(
                    osm_forward_mode_mask != 0 && forward_control != TrafficControl::None,
                ) + usize::from(
                    osm_reverse_mode_mask != 0 && backward_control != TrafficControl::None,
                );
                let shared_edge_info_id = self.next_shared_edge_info_id;
                self.next_shared_edge_info_id += 1;
                let template_class = match &last_template {
                    Some((attrs, class)) if Arc::ptr_eq(attrs, &pending.attrs) => *class,
                    _ => {
                        let class = graph.shared_edge_infos.intern(shared_info_template.clone());
                        last_template = Some((pending.attrs.clone(), class));
                        class
                    }
                };
                graph.shared_edge_infos.push_with_class(
                    shared_edge_info_id,
                    shared_info_template.osm_way_id,
                    &current_segment_points,
                    template_class,
                );
                let info_index = graph.shared_edge_infos.len() - 1;
                if !pending.source_lineage_ids.is_empty() {
                    graph
                        .shared_edge_infos
                        .set_source_lineage_ids(info_index, pending.source_lineage_ids.to_vec());
                }
                if way.coverage_id.is_some() {
                    graph
                        .shared_edge_infos
                        .set_coverage_id(info_index, way.coverage_id.clone());
                }
                self.report.created_shared_segment_count += 1;

                if osm_forward_mode_mask != 0 || physical_forward {
                    let edge_id = self.next_edge_id;
                    self.next_edge_id += 1;
                    let mobility_mode_mask = mobility_mode_mask_from_osm(osm_forward_mode_mask);
                    graph.add_edge(
                        start_node_idx,
                        end_node_idx,
                        RoadEdge {
                            edge_id,
                            shared_edge_info_id,
                            distance_m: distance as f32,
                            base_cost: distance as f32 * way.road_type.cost_factor() * road_factor,
                            mobility_mode_mask,
                            osm_forward_mode_mask,
                            osm_reverse_mode_mask,
                            hierarchy_tier,
                            traversal_direction: TraversalDirection::Forward,
                            segment_index,
                            end_control: forward_control,
                            ..Default::default()
                        },
                    );
                    self.report.created_directed_edge_count += 1;
                    if osm_forward_mode_mask == 0 {
                        self.report.sar_only_edge_count += 1;
                    } else {
                        self.report
                            .record_access_mask(osm_forward_mode_mask, mobility_mode_mask);
                        if mobility_mode_mask == 0 {
                            self.report.zero_access_edge_count += 1;
                        }
                    }
                }
                if osm_reverse_mode_mask != 0 || physical_backward {
                    let edge_id = self.next_edge_id;
                    self.next_edge_id += 1;
                    let mobility_mode_mask = mobility_mode_mask_from_osm(osm_reverse_mode_mask);
                    graph.add_edge(
                        end_node_idx,
                        start_node_idx,
                        RoadEdge {
                            edge_id,
                            shared_edge_info_id,
                            distance_m: distance as f32,
                            base_cost: distance as f32 * way.road_type.cost_factor() * road_factor,
                            mobility_mode_mask,
                            osm_forward_mode_mask,
                            osm_reverse_mode_mask,
                            hierarchy_tier,
                            traversal_direction: TraversalDirection::Backward,
                            segment_index,
                            end_control: backward_control,
                            ..Default::default()
                        },
                    );
                    self.report.created_directed_edge_count += 1;
                    if osm_reverse_mode_mask == 0 {
                        self.report.sar_only_edge_count += 1;
                    } else {
                        self.report
                            .record_access_mask(osm_reverse_mode_mask, mobility_mode_mask);
                        if mobility_mode_mask == 0 {
                            self.report.zero_access_edge_count += 1;
                        }
                    }
                }

                start_node_idx = end_node_idx;
                let continuation = { graph.graph.node_lat_lng(start_node_idx.index()) };
                current_segment_points.clear();
                current_segment_points.push(continuation);
                segment_start = index;
                segment_index += 1;
            }
        }
        drop(shared_point);
        drop(motor_junction_point);

        synthesize_pedestrian_plazas(
            &mut graph,
            &node_map,
            plaza_accumulators,
            &mut self.next_edge_id,
            &mut self.next_shared_edge_info_id,
            &mut self.next_synthetic_plaza_way_id,
            &mut self.report,
        );

        // Barriers, on every graph node that is a barrier node: the profiles
        // each closes or gates in normal routing, and its class
        // for the SAR `barriers` relaxation. The engine applies them at
        // transitions through the node.
        if !self.barrier_nodes.is_empty() {
            for node in 0..graph.graph.node_count() {
                let Some(tags) = graph
                    .graph
                    .node_source_node_id(node)
                    .and_then(|id| self.barrier_nodes.get(&id))
                else {
                    continue;
                };
                let (mut barrier_class, mut barrier_access) = graph.graph.node_barrier(node);
                self.report.barrier_nodes_seen += 1;
                match derive_node_barrier(tags) {
                    None => self.report.barrier_unknown_value_count += 1,
                    Some(barrier) if barrier.has_effect() => {
                        let access = barrier.profile_access();
                        barrier_access = access.0;
                        self.report.barrier_constraints_applied += 1;
                        self.report.barrier_hard_block_count += usize::from(access.closed() != 0);
                        self.report.barrier_conditional_count += usize::from(access.gate() != 0);
                    }
                    Some(_) => {}
                }
                if let Some(class) = sar_barrier_class(tags) {
                    barrier_class = class as u8;
                    self.report.sar_barrier_node_count += 1;
                }
                graph
                    .graph
                    .set_node_barrier(node, barrier_class, barrier_access);
            }
        }

        self.report.created_node_count = graph.node_count();
        (graph, self.report)
    }
}

/// The controls a vehicle meets on one segment before leaving it, (forward,
/// backward): see [`RoadGraphBuilder::set_traffic_control_points`].
/// `start_junction` and `end_junction` tell whether the segment's ends are
/// junctions of motor roads.
fn segment_end_controls(
    points: &[(f64, f64)],
    controls: &FxHashMap<(i32, i32), TrafficControlPoint>,
    start_junction: bool,
    end_junction: bool,
) -> (TrafficControl, TrafficControl) {
    use TrafficControlDirection as D;
    let last = points.len() - 1;
    let control_at = |index: usize| {
        let (lat, lng) = points[index];
        controls.get(&(to_fixed(lat), to_fixed(lng)))
    };
    let length_m = calculate_path_distance(points);
    let signalled_node = |index: usize| {
        control_at(index).is_some_and(|point| point.control == TrafficControl::TrafficSignals)
    };
    // An edge inside one signalised junction is not signalled again.
    let internal =
        length_m <= JUNCTION_CONTROL_RANGE_M && signalled_node(0) && signalled_node(last);
    let (mut forward, mut backward) = (TrafficControl::None, TrafficControl::None);
    if let Some(point) = control_at(last)
        && point.direction != D::Backward
        && !internal
    {
        forward = forward.max(point.control);
    }
    if let Some(point) = control_at(0)
        && point.direction != D::Forward
        && !internal
    {
        backward = backward.max(point.control);
    }
    let mut along_m = 0.0;
    for index in 1..last {
        along_m += calculate_path_distance(&points[index - 1..=index]);
        let Some(point) = control_at(index) else {
            continue;
        };
        let near_start = start_junction && along_m <= JUNCTION_CONTROL_RANGE_M;
        let near_end = end_junction && length_m - along_m <= JUNCTION_CONTROL_RANGE_M;
        let (to_end, to_start) = match point.direction {
            D::Forward => (true, false),
            D::Backward => (false, true),
            D::Both => match (near_start, near_end) {
                (true, false) => (false, true),
                (false, true) => (true, false),
                (true, true) => (length_m - along_m <= along_m, along_m <= length_m - along_m),
                (false, false) => (true, true),
            },
        };
        if to_end {
            forward = forward.max(point.control);
        }
        if to_start {
            backward = backward.max(point.control);
        }
    }
    (forward, backward)
}

/// A hash of the attributes pushed ways are most often told apart by. Equal
/// attributes hash equally; the full comparison is `RoutableWay`'s `==`.
fn way_attrs_hash(way: &RoutableWay) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut hasher = rustc_hash::FxHasher::default();
    (way.road_type as u8, way.directional_access as u8).hash(&mut hasher);
    for value in [
        &way.highway,
        &way.name,
        &way.ref_name,
        &way.access,
        &way.service,
        &way.surface,
        &way.maxspeed,
        &way.foot,
        &way.bicycle,
        &way.lanes,
        &way.layer,
        &way.coverage_id,
    ] {
        value.as_deref().hash(&mut hasher);
    }
    (
        way.is_roundabout,
        way.is_link,
        way.has_bridge,
        way.has_tunnel,
    )
        .hash(&mut hasher);
    hasher.finish()
}

/// The modes OSM tagging allows on `way`, before any direction.
fn way_mode_mask(way: &RoutableWay) -> ModeMask {
    derive_osm_mode_mask(
        way.highway.as_deref(),
        way.access.as_deref(),
        way.vehicle.as_deref(),
        way.motor_vehicle.as_deref(),
        way.motorcar.as_deref(),
        way.bicycle.as_deref(),
        way.foot.as_deref(),
        way.motorroad.as_deref(),
    )
}

/// A fixed-point coordinate as one sortable key.
fn point_key((lat, lng): (i32, i32)) -> u64 {
    (u64::from(lat as u32) << 32) | u64::from(lng as u32)
}

fn point_of_key(key: u64) -> (i32, i32) {
    ((key >> 32) as u32 as i32, key as u32 as i32)
}

/// The coordinates more than one way point shares, with how many points
/// are there and, when `motor_arms`, the arms of motor roads meeting there
/// (a way's interior point is two arms, an end one). A coordinate left
/// out has a single point and at most two arms. `point_keys` holds every
/// way point's [`point_key`].
fn shared_coord_counts(
    mut point_keys: Vec<u64>,
    ways: &[Vec<PendingWay>],
    motor_arms: bool,
) -> FxHashMap<(i32, i32), PointCounts> {
    point_keys.par_sort_unstable();
    let shared_runs = || {
        point_keys
            .chunk_by(|left, right| left == right)
            .filter(|run| run.len() > 1)
    };
    let mut counts = FxHashMap::default();
    counts.reserve(shared_runs().count());
    for run in shared_runs() {
        counts.insert(
            point_of_key(run[0]),
            PointCounts {
                points: run.len() as u32,
                motor_arms: 0,
            },
        );
    }
    drop(point_keys);
    if motor_arms {
        for way in ways.iter().flatten() {
            if way_mode_mask(&way.attrs) & MOTOR_MODES == 0 {
                continue;
            }
            let last = way.points.len() - 1;
            for index in 0..way.points.len() {
                if let Some(point) = counts.get_mut(&way.points.fixed(index)) {
                    point.motor_arms += u32::from(index > 0) + u32::from(index < last);
                }
            }
        }
    }
    counts
}

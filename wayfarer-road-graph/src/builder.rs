//! [`RoadGraphBuilder`]: pushed ways and traffic-control points to a
//! [`RoadGraph`].

use super::*;
use rustc_hash::FxHashSet;
use std::ops::Range;
use std::sync::atomic::{AtomicU8, AtomicU64, Ordering};

const ROUTABLE_WAY_CHUNK_LEN: usize = 16 * 1024;

/// A pushed way as `finish` reads it. A `RoutableWay` is ~780 bytes of
/// attributes, and the compiler pushes one per topology segment, so a region
/// held ~1M copies of mostly the same attributes. Pushes with equal attributes
/// (once the way id and lineage are set aside) share one template: ~60k
/// distinct sets for ~1M pushes on a region.
#[derive(Debug)]
struct PendingWay {
    /// The way without its id, lineage, `points` and `source_node_ids`: an
    /// index into [`RoadGraphBuilder::templates`].
    template: u32,
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

    /// The point at `index` as given.
    fn point(&self, index: usize) -> (f64, f64) {
        match self {
            Self::Fixed(points) => (from_fixed(points[index].0), from_fixed(points[index].1)),
            Self::Exact(points) => points[index],
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

/// Attributes ways can share, as [`RoadGraphBuilder::way_template`] holds
/// them for [`RoadGraphBuilder::push_template_ways`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct WayTemplate(u32);

/// A way pushed with a [`WayTemplate`] for its attributes: what
/// [`RoadGraphBuilder::push_way`] takes from a [`RoutableWay`] besides them.
#[derive(Debug, Clone)]
pub struct TemplateWay {
    pub template: WayTemplate,
    pub osm_way_id: i64,
    pub points: Vec<(f64, f64)>,
    /// `(point index, node id)` for the points that have a source node id,
    /// in point order. Indices past the points are ignored.
    pub source_node_ids: Vec<(u32, i64)>,
    /// Empty for `[osm_way_id]`.
    pub source_lineage_ids: Vec<i64>,
}

/// A template's attributes and what pushing a way with them does.
#[derive(Debug)]
struct Template {
    way: RoutableWay,
    verdict: WayVerdict,
}

/// What [`RoadGraphBuilder::push_way`] does with a way of two points or
/// more, by its attributes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WayVerdict {
    Closed,
    Restricted,
    NoMode,
    /// Kept for SAR only: no mode may legally use it.
    SarOnly,
    Accepted,
}

impl WayVerdict {
    fn of(way: &RoutableWay) -> Self {
        if way.directional_access == DirectionalAccess::Closed {
            return Self::Closed;
        }
        let mode_mask = way_mode_mask(way);
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
            return Self::Restricted;
        }
        // Construction, proposed and other ways no mode may use by default.
        if mode_mask == 0 && !sar_only {
            return Self::NoMode;
        }
        if sar_only {
            Self::SarOnly
        } else {
            Self::Accepted
        }
    }

    fn keeps(self) -> bool {
        matches!(self, Self::SarOnly | Self::Accepted)
    }
}

/// A pushed way's outcome, for the report: its verdict, or `None` for a way
/// of fewer than two points.
type PushOutcome = (Option<WayVerdict>, RoadType);

#[derive(Debug, Default)]
pub struct RoadGraphBuilder {
    graph_id: String,
    country_code: Option<String>,
    /// Pushed ways in fixed-size chunks: `finish` consumes them in order and
    /// frees each chunk as it goes, instead of holding every (wide) way until
    /// the whole graph has been built.
    ways: Vec<Vec<PendingWay>>,
    /// The distinct pushed attributes (see [`PendingWay::template`]), and
    /// their indices by [`way_attrs_hash`].
    templates: Vec<Template>,
    way_attrs: FxHashMap<u64, Vec<u32>>,
    next_edge_id: u64,
    next_shared_edge_info_id: u64,
    next_synthetic_plaza_way_id: i64,
    barrier_nodes: HashMap<i64, NodeBarrierTags>,
    traffic_controls: FxHashMap<(i32, i32), TrafficControlPoint>,
    report: RoadGraphBuildReport,
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
            templates: Vec::new(),
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
            self.record_outcomes([(None, way.road_type)]);
            return;
        }
        // Ids that do not match the points one to one are ignored.
        let source_node_ids = if way.source_node_ids.len() == way.points.len() {
            way.source_node_ids
                .iter()
                .enumerate()
                .filter_map(|(index, node_id)| node_id.map(|node_id| (index as u32, node_id)))
                .collect()
        } else {
            Vec::new()
        };
        let pushed = TemplateWay {
            template: WayTemplate(0),
            osm_way_id: way.osm_way_id,
            points: std::mem::take(&mut way.points),
            source_node_ids,
            source_lineage_ids: std::mem::take(&mut way.source_lineage_ids),
        };
        let template = self.way_template(way);
        self.push_template_ways(vec![TemplateWay { template, ..pushed }]);
    }

    /// The template for `way`'s attributes (all but its id, lineage, points
    /// and source node ids, which are ignored), the same for equal ones.
    pub fn way_template(&mut self, mut way: RoutableWay) -> WayTemplate {
        way.osm_way_id = 0;
        way.points = Vec::new();
        way.source_node_ids = Vec::new();
        way.source_lineage_ids = Vec::new();
        let bucket = self.way_attrs.entry(way_attrs_hash(&way)).or_default();
        if let Some(&index) = bucket
            .iter()
            .find(|&&index| self.templates[index as usize].way == way)
        {
            return WayTemplate(index);
        }
        let index = u32::try_from(self.templates.len()).expect("fewer than 2^32 way templates");
        bucket.push(index);
        let verdict = WayVerdict::of(&way);
        self.templates.push(Template { way, verdict });
        WayTemplate(index)
    }

    /// Pushes ways in order, each as [`push_way`](Self::push_way) of it with
    /// its template's attributes would; they are prepared in parallel.
    pub fn push_template_ways(&mut self, ways: Vec<TemplateWay>) {
        let templates = &self.templates;
        let prepared = ways
            .into_par_iter()
            .map(|way| pending_way(templates, way))
            .collect::<Vec<_>>();
        self.record_outcomes(prepared.iter().map(|(outcome, _)| *outcome));
        for way in prepared.into_iter().filter_map(|(_, way)| way) {
            match self.ways.last_mut() {
                Some(chunk) if chunk.len() < ROUTABLE_WAY_CHUNK_LEN => chunk.push(way),
                _ => {
                    let mut chunk = Vec::with_capacity(ROUTABLE_WAY_CHUNK_LEN);
                    chunk.push(way);
                    self.ways.push(chunk);
                }
            }
        }
    }

    /// Counts pushed ways' outcomes into the report.
    fn record_outcomes(&mut self, outcomes: impl IntoIterator<Item = PushOutcome>) {
        let mut accepted = Vec::<(RoadType, usize)>::new();
        let mut rejected = [0usize; 4];
        for (verdict, road_type) in outcomes {
            match verdict {
                None => rejected[0] += 1,
                Some(WayVerdict::Closed) => rejected[1] += 1,
                Some(WayVerdict::Restricted) => rejected[2] += 1,
                Some(WayVerdict::NoMode) => rejected[3] += 1,
                Some(WayVerdict::SarOnly) => self.report.sar_only_way_count += 1,
                Some(WayVerdict::Accepted) => {
                    match accepted.iter_mut().find(|(known, _)| *known == road_type) {
                        Some((_, count)) => *count += 1,
                        None => accepted.push((road_type, 1)),
                    }
                }
            }
        }
        self.report.dropped_short_way_count += rejected[0];
        self.report.dropped_closed_way_count += rejected[1];
        self.report.dropped_restricted_way_count += rejected[2];
        for (reason, count) in ["short", "closed", "restricted", "no_mode"]
            .into_iter()
            .zip(rejected)
        {
            self.report.record_rejected_ways(reason, count);
        }
        for (road_type, count) in accepted {
            self.report.record_accepted_ways(road_type, count);
        }
    }

    pub fn finish(self) -> RoadGraph {
        self.finish_with_report().0
    }

    pub fn finish_with_report(mut self) -> (RoadGraph, RoadGraphBuildReport) {
        let country_code = self.country_code.take();
        self.way_attrs = FxHashMap::default();
        let mut graph = RoadGraph::new(self.graph_id);
        graph.country_code = country_code.clone();
        if self.ways.iter().all(Vec::is_empty) {
            return (graph, self.report);
        }
        let ways = std::mem::take(&mut self.ways);
        let templates = std::mem::take(&mut self.templates);
        let layout = ChunkLayout::new(&ways);

        // Which way points close a segment, and where motor roads meet:
        // found by sorting the points by coordinate, then each way point that
        // starts or ends a segment (a visit) is numbered into a node.
        let (flags, links) = point_flags(&ways, &layout, &templates, &self.barrier_nodes);
        let numbering = number_nodes(&ways, &layout, &flags, links);
        // Where each chunk's visits start.
        let mut visit_starts = (0..ways.len())
            .into_par_iter()
            .map(|chunk| visit_count(&flags[layout.chunk_points(chunk)]))
            .collect::<Vec<_>>();
        let mut visits = 0;
        for start in &mut visit_starts {
            (*start, visits) = (visits, visits + *start);
        }

        // Grown by doubling, the node vector ended up to half unused.
        let node_count = numbering.nodes.len();
        graph
            .graph
            .reserve_exact_nodes(node_count + node_count / 64 + 64);
        // Sized once: grown by doubling, the shared-info vector held its old
        // and new buffers at once and ended up to half unused (~0.5 GiB on a
        // region), and the edge vector likewise. Plazas add a few more; the
        // slack keeps them from forcing a doubling.
        let segment_count = numbering.visit_nodes.len() - layout.way_count;
        let shared_info_estimate = segment_count + segment_count / 64 + 64;
        graph.shared_edge_infos.reserve_exact(shared_info_estimate);
        // A way's segments share their end points: each segment has one more
        // point than it adds to the way (plazas: two per spoke).
        graph.shared_edge_infos.reserve_points_exact(
            layout.point_count - layout.way_count
                + segment_count
                + 2 * (shared_info_estimate - segment_count),
        );
        graph.graph.reserve_exact_edges(2 * shared_info_estimate);
        // A segment continues from its start node's position: the way's own
        // point unless a way of exact (not E7) points made the node, which
        // only a caller other than the compiler pushes.
        let node_positions = ways
            .iter()
            .flatten()
            .any(|way| matches!(way.points, PendingPoints::Exact(_)))
            .then(|| {
                numbering
                    .nodes
                    .iter()
                    .map(|node| (node.lat, node.lng))
                    .collect::<Vec<_>>()
            });
        for (id, node) in numbering.nodes.into_iter().enumerate() {
            graph.add_node(RoadNode {
                id: id as i64,
                lat: node.lat,
                lng: node.lng,
                source_node_id: node.source_node_id,
                ..Default::default()
            });
        }
        let visit_nodes = numbering.visit_nodes;

        // What each template's segments take from it, once per template.
        let segment_templates = templates
            .par_iter()
            .map(|template| SegmentTemplate::new(&template.way, country_code.as_deref()))
            .collect::<Vec<_>>();

        // A chunk of ways is planned (segments, lengths, controls) across the
        // pool while the one before it is added to the graph in order.
        let planner = SegmentPlanner {
            flags: &flags,
            visit_nodes: &visit_nodes,
            node_positions: node_positions.as_deref(),
            traffic_controls: &self.traffic_controls,
        };
        let plan = |(index, chunk): (usize, Vec<PendingWay>)| {
            let plans = planner.plan(&chunk, layout.point_starts[index], visit_starts[index]);
            (chunk, plans)
        };
        let mut emitter = SegmentEmitter {
            templates: &templates,
            segment_templates: &segment_templates,
            template_classes: vec![u32::MAX; templates.len()],
            access_masks: Vec::new(),
            plaza_accumulators: HashMap::new(),
        };
        let mut chunks = ways.into_iter().enumerate();
        let mut current = chunks.next().map(plan);
        while let Some((chunk, plans)) = current.take() {
            let (next, ()) = rayon::join(
                || chunks.next().map(plan),
                || {
                    emitter.emit(
                        &chunk,
                        plans,
                        &mut graph,
                        &mut self.next_edge_id,
                        &mut self.next_shared_edge_info_id,
                        &mut self.report,
                    )
                },
            );
            current = next;
        }
        drop(flags);
        drop(visit_nodes);
        drop(node_positions);
        for (osm_mode_mask, mobility_mode_mask, count) in emitter.access_masks {
            self.report
                .record_access_masks(osm_mode_mask, mobility_mode_mask, count);
        }
        let plaza_accumulators = emitter.plaza_accumulators;

        // The plazas look nodes up by position: only their own ring points'.
        let node_map = plaza_node_map(&graph, &plaza_accumulators);
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

/// A pushed way's template verdict and, when it is kept, the way as `finish`
/// reads it.
fn pending_way(templates: &[Template], way: TemplateWay) -> (PushOutcome, Option<PendingWay>) {
    let template = &templates[way.template.0 as usize];
    let road_type = template.way.road_type;
    if way.points.len() < 2 {
        return ((None, road_type), None);
    }
    if !template.verdict.keeps() {
        return ((Some(template.verdict), road_type), None);
    }
    let mut lineage = way.source_lineage_ids;
    if lineage.is_empty() {
        lineage.push(way.osm_way_id);
    }
    lineage.sort_unstable();
    lineage.dedup();
    let source_lineage_ids = if lineage == [way.osm_way_id] {
        Box::default()
    } else {
        lineage.into_boxed_slice()
    };
    let point_count = way.points.len();
    let mut source_node_ids = way.source_node_ids;
    debug_assert!(source_node_ids.is_sorted_by_key(|&(index, _)| index));
    source_node_ids.retain(|&(index, _)| (index as usize) < point_count);
    let pending = PendingWay {
        template: way.template.0,
        osm_way_id: way.osm_way_id,
        source_lineage_ids,
        points: PendingPoints::new(way.points),
        source_node_ids: source_node_ids.into_boxed_slice(),
    };
    ((Some(template.verdict), road_type), Some(pending))
}

/// Where each chunk of pending ways starts in the ways' points, flattened in
/// push order.
struct ChunkLayout {
    /// One more entry than chunks, the last the point count.
    point_starts: Vec<usize>,
    point_count: usize,
    way_count: usize,
}

impl ChunkLayout {
    fn new(ways: &[Vec<PendingWay>]) -> Self {
        let lens = ways
            .par_iter()
            .map(|chunk| chunk.iter().map(|way| way.points.len()).sum::<usize>())
            .collect::<Vec<_>>();
        let mut point_starts = Vec::with_capacity(ways.len() + 1);
        let mut total = 0;
        point_starts.push(0);
        for len in lens {
            total += len;
            point_starts.push(total);
        }
        Self {
            point_starts,
            point_count: total,
            way_count: ways.iter().map(Vec::len).sum(),
        }
    }

    fn chunk_points(&self, chunk: usize) -> Range<usize> {
        self.point_starts[chunk]..self.point_starts[chunk + 1]
    }
}

/// [`point_flags`] bits of a way point.
const FIRST_POINT: u8 = 1;
const LAST_POINT: u8 = 2;
/// On a way some motor mode may use.
const MOTOR_WAY_POINT: u8 = 4;
/// A barrier node with an effect: it closes a segment inside a way.
const BARRIER_POINT: u8 = 8;
/// Another way point shares its coordinate: a junction, which closes a
/// segment.
const SHARED_POINT: u8 = 16;
/// A junction of three or more motor road arms (a way's interior point is
/// two arms, an end one), where traffic controls are placed.
const MOTOR_JUNCTION_POINT: u8 = 32;
/// A point that starts or ends a segment.
const VISIT_POINT: u8 = FIRST_POINT | LAST_POINT | BARRIER_POINT | SHARED_POINT;

/// The shares [`point_flags`] sorts the points in, by a hash of their
/// coordinate: a share's `(coordinate, point)` pairs at a time.
const POINT_SHARE_BITS: u32 = 2;
const POINT_SHARES: usize = 1 << POINT_SHARE_BITS;

fn point_share(key: u64) -> usize {
    (key.wrapping_mul(0x9E37_79B9_7F4A_7C15) >> (u64::BITS - POINT_SHARE_BITS)) as usize
}

/// How many of `flags`' points start or end a segment.
fn visit_count(flags: &[u8]) -> usize {
    flags
        .iter()
        .filter(|&&flag| flag & VISIT_POINT != 0)
        .count()
}

/// Every way point's coordinate key, with its index among all points, chunk
/// by chunk.
fn chunk_point_keys<'a>(
    ways: &'a [Vec<PendingWay>],
    layout: &'a ChunkLayout,
    chunk: usize,
) -> impl Iterator<Item = (u64, u64)> + 'a {
    let mut index = layout.point_starts[chunk] as u64;
    ways[chunk].iter().flat_map(move |way| {
        let start = index;
        index += way.points.len() as u64;
        (0..way.points.len()).map(move |at| (point_key(way.points.fixed(at)), start + at as u64))
    })
}

/// Flags every way point ([`FIRST_POINT`] … [`MOTOR_JUNCTION_POINT`]). The
/// coordinates are sorted one share at a time with their points: a run of
/// more than one is a shared coordinate, and its motor arms are summed. Each
/// run's points that start or end a segment are linked to the first of them:
/// returns the flags and, for those points, the links, flattened in push
/// order.
fn point_flags(
    ways: &[Vec<PendingWay>],
    layout: &ChunkLayout,
    templates: &[Template],
    barrier_nodes: &HashMap<i64, NodeBarrierTags>,
) -> (Vec<u8>, Vec<u64>) {
    let flags = (0..layout.point_count)
        .into_par_iter()
        .map(|_| AtomicU8::new(0))
        .collect::<Vec<_>>();
    let motor_templates = templates
        .par_iter()
        .map(|template| way_mode_mask(&template.way) & MOTOR_MODES != 0)
        .collect::<Vec<_>>();
    ways.par_iter().enumerate().for_each(|(chunk, chunk_ways)| {
        let mut index = layout.point_starts[chunk];
        for way in chunk_ways {
            let last = way.points.len() - 1;
            let motor = if motor_templates[way.template as usize] {
                MOTOR_WAY_POINT
            } else {
                0
            };
            for at in 0..=last {
                let first = if at == 0 { FIRST_POINT } else { 0 };
                let end = if at == last { LAST_POINT } else { 0 };
                flags[index + at].store(first | end | motor, Ordering::Relaxed);
            }
            for &(at, node_id) in &way.source_node_ids {
                if barrier_nodes
                    .get(&node_id)
                    .is_some_and(barrier_node_applies)
                {
                    flags[index + at as usize].fetch_or(BARRIER_POINT, Ordering::Relaxed);
                }
            }
            index += way.points.len();
        }
    });
    drop(motor_templates);

    let links = (0..layout.point_count)
        .into_par_iter()
        .map(|_| AtomicU64::new(0))
        .collect::<Vec<_>>();
    let share_lens = (0..ways.len())
        .into_par_iter()
        .map(|chunk| {
            let mut lens = [0usize; POINT_SHARES];
            for (key, _) in chunk_point_keys(ways, layout, chunk) {
                lens[point_share(key)] += 1;
            }
            lens
        })
        .collect::<Vec<_>>();
    for share in 0..POINT_SHARES {
        let chunk_lens = share_lens
            .iter()
            .map(|lens| lens[share])
            .collect::<Vec<_>>();
        let mut keyed = collect_by_chunks(&chunk_lens, |chunk| {
            chunk_point_keys(ways, layout, chunk).filter(move |&(key, _)| point_share(key) == share)
        });
        keyed.par_sort_unstable();
        keyed
            .par_chunk_by(|left, right| left.0 == right.0)
            .for_each(|run| {
                let flag = |point: u64| flags[point as usize].load(Ordering::Relaxed);
                let shared = if run.len() > 1 {
                    let motor_arms = run
                        .iter()
                        .map(|&(_, point)| flag(point))
                        .filter(|flag| flag & MOTOR_WAY_POINT != 0)
                        .map(|flag| {
                            u32::from(flag & FIRST_POINT == 0) + u32::from(flag & LAST_POINT == 0)
                        })
                        .sum::<u32>();
                    if motor_arms >= 3 {
                        SHARED_POINT | MOTOR_JUNCTION_POINT
                    } else {
                        SHARED_POINT
                    }
                } else {
                    0
                };
                let mut first_visit = None;
                for &(_, point) in run {
                    let flag = flags[point as usize].fetch_or(shared, Ordering::Relaxed) | shared;
                    if flag & VISIT_POINT != 0 {
                        let first = *first_visit.get_or_insert(point);
                        links[point as usize].store(first, Ordering::Relaxed);
                    }
                }
            });
    }
    (
        flags.into_iter().map(AtomicU8::into_inner).collect(),
        links.into_iter().map(AtomicU64::into_inner).collect(),
    )
}

/// A graph node before it is added: its first visit's position and the first
/// source node id its visits have.
struct NodeSeed {
    lat: f64,
    lng: f64,
    source_node_id: Option<i64>,
}

/// The graph's nodes, and the node of each visit (a way point that starts
/// or ends a segment), in push order.
struct NodeNumbering {
    nodes: Vec<NodeSeed>,
    visit_nodes: Vec<u32>,
}

/// A visit: a way point that starts or ends a segment.
struct Visit<'a> {
    /// Its index among all way points.
    point: usize,
    way: &'a PendingWay,
    /// Its index in its way.
    at: usize,
    source_node_id: Option<i64>,
}

/// A chunk's visits, in push order.
fn chunk_visits<'a>(
    ways: &'a [Vec<PendingWay>],
    layout: &ChunkLayout,
    flags: &'a [u8],
    chunk: usize,
) -> impl Iterator<Item = Visit<'a>> + 'a {
    let mut start = layout.point_starts[chunk];
    ways[chunk].iter().flat_map(move |way| {
        let first = start;
        start += way.points.len();
        let mut sources = way.source_node_ids.iter().peekable();
        (0..way.points.len())
            .filter(move |&at| flags[first + at] & VISIT_POINT != 0)
            .map(move |at| {
                while sources
                    .next_if(|&&(source_at, _)| (source_at as usize) < at)
                    .is_some()
                {}
                let source_node_id = sources
                    .next_if(|&&(source_at, _)| source_at as usize == at)
                    .map(|&(_, node_id)| node_id);
                Visit {
                    point: first + at,
                    way,
                    at,
                    source_node_id,
                }
            })
    })
}

/// Marks a link that holds its visit's node id rather than a point.
const LINKED_NODE: u64 = 1 << 63;

/// Numbers the nodes in the order the ways first visit their coordinates
/// (a way's start, then each segment's end), as the serial loop did. A
/// visit linked to itself ([`point_flags`]' `links`) is a node's first: the
/// first visits are numbered in push order, chunk by chunk in parallel from
/// each chunk's count, and every other visit reads its first visit's node. A
/// node takes its first visit's position and the first source node id its
/// visits have.
fn number_nodes(
    ways: &[Vec<PendingWay>],
    layout: &ChunkLayout,
    flags: &[u8],
    mut links: Vec<u64>,
) -> NodeNumbering {
    let is_first = |links: &[u64], visit: &Visit<'_>| links[visit.point] == visit.point as u64;
    let counts = (0..ways.len())
        .into_par_iter()
        .map(|chunk| {
            chunk_visits(ways, layout, flags, chunk).fold((0, 0), |(firsts, visits), visit| {
                (firsts + usize::from(is_first(&links, &visit)), visits + 1)
            })
        })
        .collect::<Vec<(usize, usize)>>();
    let first_lens = counts.iter().map(|&(firsts, _)| firsts).collect::<Vec<_>>();
    let visit_lens = counts.iter().map(|&(_, visits)| visits).collect::<Vec<_>>();
    let mut nodes = collect_by_chunks(&first_lens, |chunk| {
        chunk_visits(ways, layout, flags, chunk)
            .filter(|visit| is_first(&links, visit))
            .map(|visit| {
                let (lat, lng) = visit.way.points.point(visit.at);
                NodeSeed {
                    lat,
                    lng,
                    source_node_id: visit.source_node_id,
                }
            })
    });
    u32::try_from(nodes.len()).expect("fewer than 2^32 graph nodes");

    // Each first visit's link becomes its node id.
    let mut chunk_links = Vec::with_capacity(ways.len());
    let mut rest = links.as_mut_slice();
    for chunk in 0..ways.len() {
        let (piece, tail) =
            std::mem::take(&mut rest).split_at_mut(layout.chunk_points(chunk).len());
        chunk_links.push(piece);
        rest = tail;
    }
    let mut first_node = 0u64;
    let chunk_first_nodes = first_lens
        .iter()
        .map(|&firsts| {
            let start = first_node;
            first_node += firsts as u64;
            start
        })
        .collect::<Vec<_>>();
    chunk_links
        .into_par_iter()
        .enumerate()
        .for_each(|(chunk, chunk_links)| {
            let start = layout.point_starts[chunk];
            let mut node = chunk_first_nodes[chunk];
            for visit in chunk_visits(ways, layout, flags, chunk) {
                let link = &mut chunk_links[visit.point - start];
                if *link == visit.point as u64 {
                    *link = node | LINKED_NODE;
                    node += 1;
                }
            }
        });
    let node_of = |visit: &Visit<'_>| {
        let link = links[visit.point];
        let link = if link & LINKED_NODE != 0 {
            link
        } else {
            links[link as usize]
        };
        (link & !LINKED_NODE) as u32
    };
    let visit_nodes = collect_by_chunks(&visit_lens, |chunk| {
        chunk_visits(ways, layout, flags, chunk).map(|visit| node_of(&visit))
    });

    // A node whose first visit has no source node takes the first one a later
    // visit has.
    let mut late_sources = (0..ways.len())
        .into_par_iter()
        .flat_map_iter(|chunk| {
            chunk_visits(ways, layout, flags, chunk).filter_map(|visit| {
                let node = node_of(&visit);
                let source_node_id = visit.source_node_id?;
                nodes[node as usize].source_node_id.is_none().then_some((
                    visit.point,
                    node,
                    source_node_id,
                ))
            })
        })
        .collect::<Vec<_>>();
    late_sources.par_sort_unstable();
    for (_, node, source_node_id) in late_sources {
        nodes[node as usize]
            .source_node_id
            .get_or_insert(source_node_id);
    }
    NodeNumbering { nodes, visit_nodes }
}

/// The items `items(chunk)` yields for each chunk, chunk after chunk,
/// collected in parallel into one vector allocated at its size: `chunk_lens`
/// holds how many each chunk yields.
fn collect_by_chunks<T, I>(chunk_lens: &[usize], items: impl Fn(usize) -> I + Sync) -> Vec<T>
where
    T: Send,
    I: Iterator<Item = T>,
{
    let len = chunk_lens.iter().sum();
    let mut out = Vec::with_capacity(len);
    let mut pieces = Vec::with_capacity(chunk_lens.len());
    let mut rest = &mut out.spare_capacity_mut()[..len];
    for &chunk_len in chunk_lens {
        let (piece, tail) = std::mem::take(&mut rest).split_at_mut(chunk_len);
        pieces.push(piece);
        rest = tail;
    }
    pieces
        .into_par_iter()
        .enumerate()
        .for_each(|(chunk, piece)| {
            let mut written = 0;
            for item in items(chunk) {
                piece[written].write(item);
                written += 1;
            }
            assert_eq!(written, piece.len(), "a chunk yields what it counted");
        });
    // SAFETY: each chunk wrote every slot of its piece (asserted above), and
    // the pieces tile the first `len` slots.
    unsafe { out.set_len(len) };
    out
}

/// Everything a segment's shared info and edges take from its way's
/// attributes, derived once per template.
struct SegmentTemplate {
    /// The shared info of each segment, but its id, way id and geometry.
    info: SharedEdgeInfo,
    osm_forward_mode_mask: ModeMask,
    osm_reverse_mode_mask: ModeMask,
    physical_forward: bool,
    physical_backward: bool,
    road_factor: f32,
    hierarchy_tier: Option<RoadHierarchyTier>,
    pedestrian_area: bool,
}

impl SegmentTemplate {
    fn new(way: &RoutableWay, country_code: Option<&str>) -> Self {
        let road_factor = road_surface_factor(way);
        let (osm_forward_mode_mask, osm_reverse_mode_mask) = derive_osm_mode_masks_from_way(way);
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
        let maxspeed_kph = parse_maxspeed_kph_with_country(way.maxspeed.as_deref(), country_code);
        let inferred_speed_kph = infer_speed_kph_for_facts(
            way.road_type,
            maxspeed_kph,
            way.surface.as_deref(),
            way.tracktype.as_deref(),
            way.smoothness.as_deref(),
            way.has_ford,
        );
        let info = SharedEdgeInfo {
            // Set per segment.
            shared_edge_info_id: 0,
            // Set per way.
            osm_way_id: 0,
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
                country_code,
            ),
            maxspeed_backward_kph: parse_maxspeed_kph_with_country(
                way.maxspeed_backward.as_deref(),
                country_code,
            ),
            hgv_maxspeed_kph: parse_maxspeed_kph_with_country(
                way.maxspeed_hgv.as_deref(),
                country_code,
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
        Self {
            info,
            osm_forward_mode_mask,
            osm_reverse_mode_mask,
            physical_forward: physical && way.directional_access.allows_forward(),
            physical_backward: physical && way.directional_access.allows_backward(),
            road_factor,
            hierarchy_tier,
            pedestrian_area: is_pedestrian_area(way),
        }
    }
}

/// One segment of a way, planned: its nodes, the end of its points in its
/// way's [`WayPlan::geometry`], its length and its end controls.
struct SegmentPlan {
    start_node: u32,
    end_node: u32,
    geometry_end: u32,
    distance: f64,
    forward_control: TrafficControl,
    backward_control: TrafficControl,
}

/// A way's segments, in order, and their points one after the other.
#[derive(Default)]
struct WayPlan {
    segments: Vec<SegmentPlan>,
    geometry: Vec<(f64, f64)>,
}

/// Splits ways into segments, independently of every other way.
struct SegmentPlanner<'a> {
    flags: &'a [u8],
    visit_nodes: &'a [u32],
    /// Node positions, when a segment can't continue from its way's own
    /// point (see `finish_with_report`).
    node_positions: Option<&'a [(f64, f64)]>,
    traffic_controls: &'a FxHashMap<(i32, i32), TrafficControlPoint>,
}

impl SegmentPlanner<'_> {
    /// The chunk's ways' plans, in order, built in parallel: the chunk's
    /// points start at `first_point` and its visits at `first_visit`.
    fn plan(&self, chunk: &[PendingWay], first_point: usize, first_visit: usize) -> Vec<WayPlan> {
        // Where each way's points and visits start.
        let mut starts = Vec::with_capacity(chunk.len());
        let (mut point, mut visit) = (first_point, first_visit);
        for way in chunk {
            starts.push((point, visit));
            visit += visit_count(&self.flags[point..point + way.points.len()]);
            point += way.points.len();
        }
        chunk
            .par_iter()
            .zip(starts)
            .map(|(way, (point, visit))| self.plan_way(way, point, visit))
            .collect()
    }

    fn plan_way(&self, way: &PendingWay, first_point: usize, first_visit: usize) -> WayPlan {
        let flags = &self.flags[first_point..first_point + way.points.len()];
        let mut visit_nodes = self.visit_nodes[first_visit..].iter().copied();
        let mut points = Vec::new();
        let points = way.points.read_into(&mut points);
        let mut plan = WayPlan {
            segments: Vec::new(),
            geometry: Vec::with_capacity(points.len()),
        };
        let place_controls = !self.traffic_controls.is_empty();
        let mut current_segment_points = Vec::with_capacity(points.len());
        current_segment_points.push(points[0]);
        let mut segment_start = 0usize;
        let mut start_node = visit_nodes.next().expect("a way starts at a node");
        for (index, &point) in points.iter().enumerate().skip(1) {
            current_segment_points.push(point);
            // A shared point (a junction), a barrier with an effect, or the
            // way's end closes a segment.
            if flags[index] & VISIT_POINT == 0 {
                continue;
            }
            let end_node = visit_nodes.next().expect("a segment ends at a node");
            let distance = calculate_path_distance(&current_segment_points);
            let (forward_control, backward_control) = if place_controls {
                segment_end_controls(
                    &current_segment_points,
                    self.traffic_controls,
                    flags[segment_start] & MOTOR_JUNCTION_POINT != 0,
                    flags[index] & MOTOR_JUNCTION_POINT != 0,
                )
            } else {
                (TrafficControl::None, TrafficControl::None)
            };
            plan.geometry.extend_from_slice(&current_segment_points);
            plan.segments.push(SegmentPlan {
                start_node,
                end_node,
                geometry_end: plan.geometry.len() as u32,
                distance,
                forward_control,
                backward_control,
            });
            start_node = end_node;
            let continuation = match self.node_positions {
                Some(positions) => positions[start_node as usize],
                None => point,
            };
            current_segment_points.clear();
            current_segment_points.push(continuation);
            segment_start = index;
        }
        plan
    }
}

/// Adds planned segments to the graph, in push order: their shared infos
/// (classes interned as they come), their directed edges and their counts.
struct SegmentEmitter<'a> {
    templates: &'a [Template],
    segment_templates: &'a [SegmentTemplate],
    /// Each template's shared-info class, once a segment of it has been
    /// added ([`u32::MAX`] before): interned the first time, so the classes
    /// are numbered in the order segments first use them.
    template_classes: Vec<u32>,
    /// How many directed edges have each (OSM, mobility) mode mask pair, in
    /// the order the pairs come: added to the report at the end.
    access_masks: Vec<(ModeMask, ModeMask, usize)>,
    plaza_accumulators: HashMap<i64, PlazaAccumulator>,
}

impl SegmentEmitter<'_> {
    fn count_access_mask(&mut self, osm_mode_mask: ModeMask, mobility_mode_mask: ModeMask) {
        match self
            .access_masks
            .iter_mut()
            .find(|(osm, mobility, _)| (*osm, *mobility) == (osm_mode_mask, mobility_mode_mask))
        {
            Some((_, _, count)) => *count += 1,
            None => self
                .access_masks
                .push((osm_mode_mask, mobility_mode_mask, 1)),
        }
    }

    fn emit(
        &mut self,
        chunk: &[PendingWay],
        plans: Vec<WayPlan>,
        graph: &mut RoadGraph,
        next_edge_id: &mut u64,
        next_shared_edge_info_id: &mut u64,
        report: &mut RoadGraphBuildReport,
    ) {
        for (pending, plan) in chunk.iter().zip(plans) {
            let way = &self.templates[pending.template as usize].way;
            let template = &self.segment_templates[pending.template as usize];
            if template.pedestrian_area {
                let mut points = Vec::new();
                pending.points.read_into(&mut points);
                self.plaza_accumulators
                    .entry(pending.osm_way_id)
                    .or_insert_with(|| PlazaAccumulator {
                        highway: way.highway.as_deref().map(str::to_string),
                        surface: way.surface.clone(),
                        allow_bicycle: plaza_allows_bicycle(way),
                        segments: Vec::new(),
                    })
                    .segments
                    .push(points);
            }
            let osm_forward_mode_mask = template.osm_forward_mode_mask;
            let osm_reverse_mode_mask = template.osm_reverse_mode_mask;
            let mut geometry_start = 0usize;
            for (segment_index, segment) in plan.segments.iter().enumerate() {
                let geometry = &plan.geometry[geometry_start..segment.geometry_end as usize];
                geometry_start = segment.geometry_end as usize;
                let (start_node_idx, end_node_idx) = (
                    NodeIndex::new(segment.start_node as usize),
                    NodeIndex::new(segment.end_node as usize),
                );
                let distance = segment.distance;
                let (forward_control, backward_control) =
                    (segment.forward_control, segment.backward_control);
                let segment_index = segment_index as u32;
                report.traffic_controlled_edge_count += usize::from(
                    osm_forward_mode_mask != 0 && forward_control != TrafficControl::None,
                ) + usize::from(
                    osm_reverse_mode_mask != 0 && backward_control != TrafficControl::None,
                );
                let shared_edge_info_id = *next_shared_edge_info_id;
                *next_shared_edge_info_id += 1;
                let template_class = &mut self.template_classes[pending.template as usize];
                if *template_class == u32::MAX {
                    *template_class = graph.shared_edge_infos.intern(template.info.clone());
                }
                let template_class = *template_class;
                graph.shared_edge_infos.push_with_class(
                    shared_edge_info_id,
                    pending.osm_way_id,
                    geometry,
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
                report.created_shared_segment_count += 1;

                if osm_forward_mode_mask != 0 || template.physical_forward {
                    let edge_id = *next_edge_id;
                    *next_edge_id += 1;
                    let mobility_mode_mask = mobility_mode_mask_from_osm(osm_forward_mode_mask);
                    graph.add_edge(
                        start_node_idx,
                        end_node_idx,
                        RoadEdge {
                            edge_id,
                            shared_edge_info_id,
                            distance_m: distance as f32,
                            base_cost: distance as f32
                                * way.road_type.cost_factor()
                                * template.road_factor,
                            mobility_mode_mask,
                            osm_forward_mode_mask,
                            osm_reverse_mode_mask,
                            hierarchy_tier: template.hierarchy_tier,
                            traversal_direction: TraversalDirection::Forward,
                            segment_index,
                            end_control: forward_control,
                            ..Default::default()
                        },
                    );
                    report.created_directed_edge_count += 1;
                    if osm_forward_mode_mask == 0 {
                        report.sar_only_edge_count += 1;
                    } else {
                        self.count_access_mask(osm_forward_mode_mask, mobility_mode_mask);
                        if mobility_mode_mask == 0 {
                            report.zero_access_edge_count += 1;
                        }
                    }
                }
                if osm_reverse_mode_mask != 0 || template.physical_backward {
                    let edge_id = *next_edge_id;
                    *next_edge_id += 1;
                    let mobility_mode_mask = mobility_mode_mask_from_osm(osm_reverse_mode_mask);
                    graph.add_edge(
                        end_node_idx,
                        start_node_idx,
                        RoadEdge {
                            edge_id,
                            shared_edge_info_id,
                            distance_m: distance as f32,
                            base_cost: distance as f32
                                * way.road_type.cost_factor()
                                * template.road_factor,
                            mobility_mode_mask,
                            osm_forward_mode_mask,
                            osm_reverse_mode_mask,
                            hierarchy_tier: template.hierarchy_tier,
                            traversal_direction: TraversalDirection::Backward,
                            segment_index,
                            end_control: backward_control,
                            ..Default::default()
                        },
                    );
                    report.created_directed_edge_count += 1;
                    if osm_reverse_mode_mask == 0 {
                        report.sar_only_edge_count += 1;
                    } else {
                        self.count_access_mask(osm_reverse_mode_mask, mobility_mode_mask);
                        if mobility_mode_mask == 0 {
                            report.zero_access_edge_count += 1;
                        }
                    }
                }
            }
        }
    }
}

/// The graph's nodes at the plazas' ring points, by position: the lookups
/// [`synthesize_pedestrian_plazas`] makes.
fn plaza_node_map(
    graph: &RoadGraph,
    plaza_accumulators: &HashMap<i64, PlazaAccumulator>,
) -> FxHashMap<(i32, i32), NodeIndex> {
    let keys = plaza_accumulators
        .values()
        .flat_map(|accumulator| accumulator.segments.iter().flatten())
        .map(|&(lat, lng)| (to_fixed(lat), to_fixed(lng)))
        .collect::<FxHashSet<_>>();
    if keys.is_empty() {
        return FxHashMap::default();
    }
    (0..graph.graph.node_count())
        .into_par_iter()
        .filter_map(|index| {
            let (lat, lng) = graph.graph.node_lat_lng(index);
            let key = (to_fixed(lat), to_fixed(lng));
            keys.contains(&key).then_some((key, NodeIndex::new(index)))
        })
        .collect()
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

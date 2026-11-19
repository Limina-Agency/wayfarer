//! [`RoadGraph`]: the directed graph with its shared edge infos and source
//! facts, the metadata refreshes over them, and shared-info lookup.

use super::*;

#[derive(Debug, Clone, Default)]
pub struct RoadGraph {
    pub graph: RoadNetwork,
    pub graph_id: String,
    pub shared_edge_infos: SharedEdgeInfos,
    pub way_facts: Vec<RoadWayFacts>,
    pub route_memberships: Vec<RoadRouteMembership>,
    pub turn_restrictions: Vec<RoadTurnRestriction>,
    pub resolved_restrictions: Vec<ResolvedEdgeRestriction>,
    /// Each node's edge counts by road type, by node index, when enhancement
    /// derived them ([`EnhanceOptions::node_road_type_counts`]); empty
    /// otherwise.
    pub node_road_type_counts: Vec<NodeRoadTypeCounts>,
    /// ISO 3166-1 alpha-2 country of the build region, used at build time to
    /// resolve implicit `maxspeed` values. Not persisted in snapshots: it only
    /// influences the baked edge speeds, never the serialized format.
    pub country_code: Option<String>,
}

/// A node's incoming and outgoing edge counts by road type label.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct NodeRoadTypeCounts {
    pub incoming: Vec<RoadGraphNamedCount>,
    pub outgoing: Vec<RoadGraphNamedCount>,
}

impl RoadGraph {
    pub fn new(graph_id: impl Into<String>) -> Self {
        Self {
            graph: Graph::new(),
            graph_id: graph_id.into(),
            shared_edge_infos: SharedEdgeInfos::new(),
            way_facts: Vec::new(),
            route_memberships: Vec::new(),
            turn_restrictions: Vec::new(),
            resolved_restrictions: Vec::new(),
            node_road_type_counts: Vec::new(),
            country_code: None,
        }
    }

    pub fn set_country_code(&mut self, country_code: Option<String>) {
        self.country_code = country_code;
    }

    pub fn add_node(&mut self, node: RoadNode) -> NodeIndex {
        self.graph.add_node(node)
    }

    pub fn add_edge(&mut self, a: NodeIndex, b: NodeIndex, edge: RoadEdge) {
        self.graph.add_edge(a, b, edge);
    }

    pub fn node_count(&self) -> usize {
        self.graph.node_count()
    }

    pub fn edge_count(&self) -> usize {
        self.graph.edge_count()
    }

    pub fn set_way_facts(&mut self, way_facts: Vec<RoadWayFacts>) {
        self.way_facts = dedupe_way_facts(way_facts);
    }

    pub fn set_route_memberships(&mut self, route_memberships: Vec<RoadRouteMembership>) {
        self.route_memberships = dedupe_route_memberships(route_memberships);
    }

    pub fn set_turn_restrictions(&mut self, turn_restrictions: Vec<RoadTurnRestriction>) {
        self.turn_restrictions = dedupe_turn_restrictions(turn_restrictions);
    }

    pub fn set_resolved_restrictions(
        &mut self,
        resolved_restrictions: Vec<ResolvedEdgeRestriction>,
    ) {
        self.resolved_restrictions = dedupe_resolved_restrictions(resolved_restrictions);
    }

    pub fn discard_source_enrichment_inputs(&mut self) {
        self.way_facts.clear();
        self.way_facts.shrink_to_fit();
        self.route_memberships.clear();
        self.route_memberships.shrink_to_fit();
        self.turn_restrictions.clear();
        self.turn_restrictions.shrink_to_fit();
    }

    pub fn shared_edge_info(&self, shared_edge_info_id: u64) -> Option<SharedInfoRef<'_>> {
        self.shared_edge_infos
            .iter()
            .find(|info| info.shared_edge_info_id == shared_edge_info_id)
    }

    pub fn refresh_shared_edge_metadata(&mut self) {
        let facts_by_way = self
            .way_facts
            .iter()
            .map(|facts| (facts.osm_way_id, facts.clone()))
            .collect::<HashMap<_, _>>();
        self.refresh_shared_edge_way_facts_with(|way_id| {
            Ok::<_, std::convert::Infallible>(facts_by_way.get(&way_id).cloned())
        })
        .expect("infallible way fact lookup");
        self.refresh_route_membership_metadata();
    }

    pub fn refresh_shared_edge_way_facts_with<F, E>(&mut self, mut load_fact: F) -> Result<(), E>
    where
        F: FnMut(i64) -> Result<Option<RoadWayFacts>, E>,
    {
        let country_code = self.country_code.clone();
        let len = self.shared_edge_infos.len();
        let run = way_facts_run(
            self.shared_edge_infos.edges(),
            self.shared_edge_infos.classes(),
            0,
            country_code.as_deref(),
            &mut load_fact,
        )?;
        self.finish_way_facts(vec![run], len, country_code.as_deref(), load_fact)
    }

    /// [`Self::refresh_shared_edge_way_facts_with`] across threads, with the
    /// same result. The infos are split into runs that end at a way boundary,
    /// so each way's facts are still loaded once; each run collects its
    /// attribute classes, and the runs' classes are merged in run order, so
    /// classes are numbered by first use whatever the thread count.
    pub fn refresh_shared_edge_way_facts_par_with<F, E>(&mut self, load_fact: F) -> Result<(), E>
    where
        F: Fn(i64) -> Result<Option<RoadWayFacts>, E> + Sync,
        E: Send,
    {
        let country_code = self.country_code.clone();
        let edges = self.shared_edge_infos.edges();
        let classes = self.shared_edge_infos.classes();
        let run_len = (edges.len() / (rayon::current_num_threads() * 8)).max(4096);
        let mut runs = Vec::new();
        let mut offset = 0usize;
        while offset < edges.len() {
            let mut end = (offset + run_len).min(edges.len());
            while end < edges.len() && edges[end].osm_way_id == edges[end - 1].osm_way_id {
                end += 1;
            }
            runs.push(offset..end);
            offset = end;
        }
        let runs = runs
            .into_par_iter()
            .map(|range| {
                let offset = range.start;
                way_facts_run(
                    &edges[range],
                    classes,
                    offset,
                    country_code.as_deref(),
                    &mut |way_id| load_fact(way_id),
                )
            })
            .collect::<Result<Vec<_>, E>>()?;
        let len = edges.len();
        self.finish_way_facts(runs, len, country_code.as_deref(), load_fact)
    }

    /// Merges the runs' classes in order, applies the lineage fallback to the
    /// infos whose own way had no facts, and installs the new classes.
    fn finish_way_facts<F, E>(
        &mut self,
        runs: Vec<WayFactsRun>,
        len: usize,
        country_code: Option<&str>,
        mut load_fact: F,
    ) -> Result<(), E>
    where
        F: FnMut(i64) -> Result<Option<RoadWayFacts>, E>,
    {
        let mut classes = SharedInfoClasses::default();
        let mut classes_of = Vec::with_capacity(len);
        let mut fallback = Vec::new();
        for run in runs {
            let global = run
                .classes
                .into_vec()
                .into_iter()
                .map(|attrs| classes.intern(attrs))
                .collect::<Vec<_>>();
            classes_of.extend(run.classes_of.iter().map(|local| global[*local as usize]));
            fallback.extend(run.without_own_facts);
        }
        if !fallback.is_empty() {
            let mut interner = TagInterner::new();
            let wanted = fallback
                .iter()
                .map(|&info_index| self.shared_edge_infos.at(info_index).shared_edge_info_id)
                .collect::<HashSet<_>>();
            let lineage = SharedInfoLineage::new(self, |shared_edge_info_id| {
                wanted.contains(&shared_edge_info_id)
            });
            for info_index in fallback {
                let shared_edge_info_id = self.shared_edge_infos.at(info_index).shared_edge_info_id;
                for &way_id in lineage.way_ids(shared_edge_info_id) {
                    if let Some(fact) = load_fact(way_id)? {
                        let mut attrs = classes.get(classes_of[info_index]).clone();
                        apply_way_facts(&mut attrs, &fact, country_code, &mut interner);
                        classes_of[info_index] = classes.intern(attrs);
                        break;
                    }
                }
            }
        }
        self.shared_edge_infos.replace_classes(classes, classes_of);
        self.shared_edge_infos.compact();
        self.refresh_edge_access_masks();
        Ok(())
    }

    pub fn refresh_route_membership_metadata(&mut self) {
        // Only road routes name the corridor a way belongs to: bus, hiking or
        // cycle routes would earn their ways the same-corridor turn discount.
        let mut route_refs_by_way = HashMap::<i64, BTreeSet<String>>::new();
        for membership in self
            .route_memberships
            .iter()
            .filter(|membership| membership.route == "road")
        {
            let entry = route_refs_by_way.entry(membership.way_id).or_default();
            if let Some(value) = membership
                .ref_name
                .as_deref()
                .filter(|value| !value.is_empty())
            {
                entry.insert(value.to_string());
            } else if let Some(value) = membership
                .relation_name
                .as_deref()
                .filter(|value| !value.is_empty())
            {
                entry.insert(value.to_string());
            } else if !membership.route.is_empty() {
                entry.insert(membership.route.clone());
            }
        }

        // With one info per id, an info's lineage is its own, so the table of
        // every info's lineage (about 16 bytes a lineage way, sorted) is only
        // built for any other numbering.
        let info_ids = SharedInfoIndex::new(&self.shared_edge_infos);
        let (used, lineage) = if info_ids.is_dense() {
            (used_infos(self, &info_ids, |_| true), None)
        } else {
            (Vec::new(), Some(SharedInfoLineage::new(self, |_| true)))
        };
        drop(info_ids);
        // A class gets each distinct route-ref list once.
        let mut changed = FxHashMap::<(u32, Vec<String>), u32>::default();
        for info_index in 0..self.shared_edge_infos.len() {
            let info = self.shared_edge_infos.at(info_index);
            let lineage_ids = match &lineage {
                Some(lineage) => lineage.way_ids(info.shared_edge_info_id),
                None if used.get(info_index) == Some(&true) => info.source_lineage_ids,
                None => &[],
            };
            let mut route_refs = BTreeSet::new();
            for way_id in std::iter::once(info.osm_way_id).chain(lineage_ids.iter().copied()) {
                if let Some(values) = route_refs_by_way.get(&way_id) {
                    route_refs.extend(values.iter().cloned());
                }
            }
            if route_refs.iter().eq(info.attrs.route_refs.iter()) {
                continue;
            }
            let key = (info.class, route_refs.into_iter().collect::<Vec<_>>());
            let class = match changed.get(&key) {
                Some(class) => *class,
                None => {
                    let mut attrs = info.attrs.clone();
                    attrs.route_refs = key.1.clone();
                    let class = self.shared_edge_infos.intern(attrs);
                    changed.insert(key, class);
                    class
                }
            };
            self.shared_edge_infos.set_class(info_index, class);
        }
        self.shared_edge_infos.compact();
    }

    fn refresh_edge_access_masks(&mut self) {
        let info_index = SharedInfoIndex::new(&self.shared_edge_infos);
        let masks = self
            .shared_edge_infos
            .iter()
            .map(|info| {
                (
                    info.attrs.osm_forward_mode_mask,
                    info.attrs.osm_reverse_mode_mask,
                )
            })
            .collect::<Vec<_>>();
        for edge in self.graph.edge_weights_mut() {
            if let Some((forward_mask, reverse_mask)) = info_index
                .get(edge.shared_edge_info_id)
                .map(|info_position| masks[info_position])
            {
                edge.osm_forward_mode_mask = forward_mask;
                edge.osm_reverse_mode_mask = reverse_mask;
                edge.mobility_mode_mask =
                    mobility_mode_mask_from_osm(match edge.traversal_direction {
                        TraversalDirection::Forward => forward_mask,
                        TraversalDirection::Backward => reverse_mask,
                    });
            }
        }
    }
}

/// Every way id in the lineage of each shared info's directed edges, ascending
/// and distinct per info: what a map of per-info sets held, as one sorted
/// list of `(shared info id, way id)` pairs instead of a set per info.
struct SharedInfoLineage {
    info_ids: Vec<u64>,
    way_ids: Vec<i64>,
}

impl SharedInfoLineage {
    fn new(graph: &RoadGraph, include: impl Fn(u64) -> bool) -> Self {
        let info_index = SharedInfoIndex::new(&graph.shared_edge_infos);
        let used = used_infos(graph, &info_index, include);
        drop(info_index);
        let mut pairs = graph
            .shared_edge_infos
            .iter()
            .zip(used)
            .filter(|(_, used)| *used)
            .flat_map(|(info, _)| {
                info.source_lineage_ids
                    .iter()
                    .map(move |way_id| (info.shared_edge_info_id, *way_id))
            })
            .collect::<Vec<_>>();
        pairs.par_sort_unstable();
        pairs.dedup();
        let (info_ids, way_ids) = pairs.into_iter().unzip();
        Self { info_ids, way_ids }
    }

    fn way_ids(&self, shared_edge_info_id: u64) -> &[i64] {
        let start = self
            .info_ids
            .partition_point(|info_id| *info_id < shared_edge_info_id);
        let end = start
            + self.info_ids[start..].partition_point(|info_id| *info_id == shared_edge_info_id);
        &self.way_ids[start..end]
    }
}

/// Whether some directed edge uses each shared info, by position, among the
/// infos whose id `include` accepts. The lineage is the info's: an info counts
/// once some edge uses it.
fn used_infos(
    graph: &RoadGraph,
    info_index: &SharedInfoIndex,
    include: impl Fn(u64) -> bool,
) -> Vec<bool> {
    let mut used = vec![false; graph.shared_edge_infos.len()];
    for edge in graph.graph.edge_weights() {
        if let Some(index) = info_index.get(edge.shared_edge_info_id)
            && include(edge.shared_edge_info_id)
        {
            used[index] = true;
        }
    }
    used
}

pub(crate) fn shared_edge_info<'a>(
    graph: &'a RoadGraph,
    info_index_by_id: &SharedInfoIndex,
    shared_edge_info_id: u64,
) -> Option<SharedInfoRef<'a>> {
    info_index_by_id
        .get(shared_edge_info_id)
        .and_then(|index| graph.shared_edge_infos.get(index))
}

/// Shared-info position by id.
///
/// The builder numbers shared infos 1, 2, 3… in order, so a lookup is usually
/// arithmetic; any other numbering falls back to a map with the same answers
/// (for a repeated id, its last position). Enhancement looks an info up several
/// times per directed edge, and hashing was a measurable share of it.
pub struct SharedInfoIndex {
    dense_len: Option<usize>,
    by_id: HashMap<u64, usize>,
}

impl SharedInfoIndex {
    pub fn new(infos: &SharedEdgeInfos) -> Self {
        let dense = infos
            .iter()
            .enumerate()
            .all(|(index, info)| info.shared_edge_info_id == index as u64 + 1);
        if dense {
            Self {
                dense_len: Some(infos.len()),
                by_id: HashMap::new(),
            }
        } else {
            Self {
                dense_len: None,
                by_id: infos
                    .iter()
                    .enumerate()
                    .map(|(index, info)| (info.shared_edge_info_id, index))
                    .collect(),
            }
        }
    }

    pub fn get(&self, shared_edge_info_id: u64) -> Option<usize> {
        match self.dense_len {
            Some(len) => (shared_edge_info_id >= 1 && shared_edge_info_id <= len as u64)
                .then(|| (shared_edge_info_id - 1) as usize),
            None => self.by_id.get(&shared_edge_info_id).copied(),
        }
    }

    /// Whether the infos are numbered 1, 2, 3… in order, so each id names
    /// exactly one info.
    pub fn is_dense(&self) -> bool {
        self.dense_len.is_some()
    }

    pub fn contains_key(&self, shared_edge_info_id: &u64) -> bool {
        self.get(*shared_edge_info_id).is_some()
    }
}

fn dedupe_way_facts(way_facts: Vec<RoadWayFacts>) -> Vec<RoadWayFacts> {
    let mut by_way = BTreeMap::new();
    for fact in way_facts {
        by_way.insert(fact.osm_way_id, fact);
    }
    by_way.into_values().collect()
}

fn dedupe_route_memberships(
    route_memberships: Vec<RoadRouteMembership>,
) -> Vec<RoadRouteMembership> {
    // First occurrence wins. The keys borrow from the memberships rather than
    // cloning five strings each into an ordered set.
    let mut seen = HashSet::with_capacity(route_memberships.len());
    let first_occurrence = route_memberships
        .iter()
        .map(|membership| {
            seen.insert((
                membership.relation_id,
                membership.way_id,
                &membership.route,
                &membership.network,
                &membership.ref_name,
                &membership.relation_name,
                &membership.member_role,
            ))
        })
        .collect::<Vec<_>>();
    drop(seen);
    // A fresh vector, so the input's spare capacity is not carried along.
    let mut deduped = Vec::with_capacity(first_occurrence.iter().filter(|first| **first).count());
    deduped.extend(
        route_memberships
            .into_iter()
            .zip(first_occurrence)
            .filter_map(|(membership, first)| first.then_some(membership)),
    );
    deduped
}

fn dedupe_turn_restrictions(
    turn_restrictions: Vec<RoadTurnRestriction>,
) -> Vec<RoadTurnRestriction> {
    let mut seen = BTreeSet::new();
    let mut deduped = Vec::new();
    for restriction in turn_restrictions {
        let key = (
            restriction.relation_id,
            restriction.from_way_id,
            restriction.to_way_id,
            restriction.via_member_type,
            restriction.via_member_id,
            restriction.via_way_ids.clone(),
            restriction.restriction.clone(),
            restriction.modes.clone(),
            restriction.except_modes.clone(),
        );
        if seen.insert(key) {
            deduped.push(restriction);
        }
    }
    deduped
}

fn dedupe_resolved_restrictions(
    resolved_restrictions: Vec<ResolvedEdgeRestriction>,
) -> Vec<ResolvedEdgeRestriction> {
    let mut seen = BTreeSet::new();
    let mut deduped = Vec::new();
    for restriction in resolved_restrictions {
        let key = serde_json::to_string(&restriction).unwrap_or_default();
        if seen.insert(key) {
            deduped.push(restriction);
        }
    }
    deduped
}

/// One run of infos with its way facts applied: the run's classes, each
/// info's class among them, and the infos whose own way has no facts (by
/// position in the graph).
struct WayFactsRun {
    classes: SharedInfoClasses,
    classes_of: Vec<u32>,
    without_own_facts: Vec<usize>,
}

/// Applies each way's facts to its infos in `edges` (which start at info
/// `offset`). A way's infos are consecutive, so its facts are loaded once,
/// and applied once per distinct class among its infos.
fn way_facts_run<E>(
    edges: &[SharedEdge],
    old: &SharedInfoClasses,
    offset: usize,
    country_code: Option<&str>,
    load_fact: &mut dyn FnMut(i64) -> Result<Option<RoadWayFacts>, E>,
) -> Result<WayFactsRun, E> {
    // Interned so the run's classes share one allocation per distinct string.
    let mut interner = TagInterner::new();
    let mut classes = SharedInfoClasses::default();
    let mut classes_of = Vec::with_capacity(edges.len());
    let mut without_own_facts = Vec::new();
    let mut loaded = None::<(i64, Option<RoadWayFacts>)>;
    // Old class to run class, for the current way.
    let mut applied = Vec::<(u32, u32)>::new();
    for (at, edge) in edges.iter().enumerate() {
        if loaded
            .as_ref()
            .is_none_or(|(way_id, _)| *way_id != edge.osm_way_id)
        {
            loaded = Some((edge.osm_way_id, load_fact(edge.osm_way_id)?));
            applied.clear();
        }
        let fact = loaded.as_ref().and_then(|(_, fact)| fact.as_ref());
        if fact.is_none() {
            without_own_facts.push(offset + at);
        }
        let class = match applied.iter().find(|(from, _)| *from == edge.class) {
            Some((_, to)) => *to,
            None => {
                let mut attrs = old.get(edge.class).clone();
                if let Some(fact) = fact {
                    apply_way_facts(&mut attrs, fact, country_code, &mut interner);
                }
                let to = classes.intern(attrs);
                applied.push((edge.class, to));
                to
            }
        };
        classes_of.push(class);
    }
    Ok(WayFactsRun {
        classes,
        classes_of,
        without_own_facts,
    })
}

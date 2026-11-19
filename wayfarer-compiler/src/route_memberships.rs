//! Route relation memberships, held compactly from collection to the graph.
//!
//! A compile reads the memberships of `route=road` relations (they name the
//! corridor a way belongs to) and counts all of them for the graph manifest.
//! Bus, hiking and cycle routes make up nearly all rows (Poland: 3.3 M rows,
//! 0.4 M of them road routes), so collection keeps each relation's tags once
//! and each way member as a 16-byte row instead of a `RoadRouteMembership`
//! (136 bytes and up to five strings each).

use std::collections::HashMap;

use osm_pbf_ingest::{NormalizedRelation, RelationMemberType};
use rayon::prelude::*;
use rustc_hash::FxHashMap;
use wayfarer_road_graph::RoadRouteMembership;

/// The tags a membership copies from its route relation.
#[derive(Debug, PartialEq, Eq, Hash)]
struct RouteRelation {
    relation_id: i64,
    route: Box<str>,
    network: Option<Box<str>>,
    ref_name: Option<Box<str>>,
    relation_name: Option<Box<str>>,
}

/// A way member of a route relation: indices into the relations and roles.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct MembershipRow {
    way_id: i64,
    relation: u32,
    /// 0 for no role, else 1 + the index into `roles`.
    role: u32,
}

#[derive(Debug, Default)]
pub(crate) struct CollectedRouteMemberships {
    relations: Vec<RouteRelation>,
    roles: Vec<Box<str>>,
    role_ids: FxHashMap<Box<str>, u32>,
    rows: Vec<MembershipRow>,
}

impl CollectedRouteMemberships {
    /// Adds a relation's way members if it is a route (`type=route` with a
    /// non-empty `route` tag).
    pub(crate) fn push_relation(&mut self, relation: &NormalizedRelation) {
        if relation.tags.get("type") != Some("route") {
            return;
        }
        let Some(route) = relation
            .tags
            .get("route")
            .map(str::trim)
            .filter(|value| !value.is_empty())
        else {
            return;
        };
        let mut members = relation
            .members
            .iter()
            .filter(|member| member.member_type == RelationMemberType::Way)
            .peekable();
        if members.peek().is_none() {
            return;
        }
        let index = u32::try_from(self.relations.len()).expect("route relation count fits u32");
        self.relations.push(RouteRelation {
            relation_id: relation.relation_id,
            route: route.into(),
            network: relation.tags.get("network").map(Into::into),
            ref_name: relation.tags.get("ref").map(Into::into),
            relation_name: relation.tags.get("name").map(Into::into),
        });
        for member in members {
            let role = if member.role.is_empty() {
                0
            } else if let Some(id) = self.role_ids.get(member.role.as_str()) {
                *id
            } else {
                self.roles.push(member.role.as_str().into());
                let id = u32::try_from(self.roles.len()).expect("role count fits u32");
                self.role_ids.insert(member.role.as_str().into(), id);
                id
            };
            self.rows.push(MembershipRow {
                way_id: member.member_id,
                relation: index,
                role,
            });
        }
    }

    /// The memberships of ways in `graph_way_ids` (sorted): how many distinct
    /// ones there are (equal in every field counts once), and the road
    /// routes' memberships as records, by way id, relation id and role.
    pub(crate) fn finish(self, graph_way_ids: &[i64]) -> (usize, Vec<RoadRouteMembership>) {
        let Self {
            relations,
            roles,
            rows,
            ..
        } = self;
        // Relations with the same id and tags are one relation for the
        // count (an id listed twice in the input).
        let mut first_of = HashMap::<&RouteRelation, u32>::with_capacity(relations.len());
        let canonical = relations
            .iter()
            .enumerate()
            .map(|(index, relation)| *first_of.entry(relation).or_insert(index as u32))
            .collect::<Vec<_>>();
        drop(first_of);
        let mut kept = rows
            .into_par_iter()
            .filter(|row| graph_way_ids.binary_search(&row.way_id).is_ok())
            .map(|row| MembershipRow {
                relation: canonical[row.relation as usize],
                ..row
            })
            .collect::<Vec<_>>();
        kept.par_sort_unstable();
        kept.dedup();
        let count = kept.len();
        let text = |value: &Option<Box<str>>| value.as_deref().map(str::to_string);
        let mut road = kept
            .iter()
            .filter(|row| &*relations[row.relation as usize].route == "road")
            .map(|row| {
                let relation = &relations[row.relation as usize];
                RoadRouteMembership {
                    relation_id: relation.relation_id,
                    way_id: row.way_id,
                    route: relation.route.to_string(),
                    network: text(&relation.network),
                    ref_name: text(&relation.ref_name),
                    relation_name: text(&relation.relation_name),
                    member_role: (row.role != 0).then(|| roles[row.role as usize - 1].to_string()),
                }
            })
            .collect::<Vec<_>>();
        road.sort_by(|left, right| {
            left.way_id
                .cmp(&right.way_id)
                .then(left.relation_id.cmp(&right.relation_id))
                .then(left.member_role.cmp(&right.member_role))
        });
        (count, road)
    }
}

//! Overlay sets: small versioned files of overlay items that devices share
//! offline, and how a workflow applies them.
//!
//! ```json
//! {
//!   "format": "wayfarer-overlay-set", "formatVersion": 1,
//!   "setId": "6f1c…", "version": 14, "createdAtMs": 1762128000000,
//!   "author": "device-or-team id",
//!   "package": { "graphId": "malopolskie", "datasetHash": "238403ef…", "formatVersion": 1 },
//!   "items": [
//!     { "itemId": "a91e…", "version": 3, "kind": "open", "profiles": ["wheeled"],
//!       "validFromMs": 1762128000000, "validToMs": null,
//!       "geometry": { "kind": "line", "points": [[50.06, 19.94], [50.07, 19.95]],
//!                     "projectionDistanceM": 20 },
//!       "edges": [ { "edgeId": 123456, "startFraction": 0.0, "endFraction": 1.0 } ],
//!       "speedKph": 30, "note": "Forest road opened", "provenance": "radio 14:05" },
//!     { "itemId": "c07b…", "version": 1, "kind": "block", "deleted": true }
//!   ],
//!   "contentHash": "blake3 of the canonical JSON without this field"
//! }
//! ```
//!
//! - **Binding.** An item's `edges` hold only on the package whose dataset
//!   hash the set names. On any other package the item is re-projected from
//!   its geometry, which is always kept, with the workflow's projection; an
//!   item with neither usable edges nor geometry is skipped and reported. A
//!   set never applies stale edge ids.
//! - **Merging.** Items have stable ids and versions: the higher version
//!   wins, and deleted items stay as tombstones so a deletion propagates.
//!   Between two different items of equal version, the tombstone wins, else
//!   the item whose canonical JSON is greater byte for byte. That is a total
//!   order on the items' own content, so merging is commutative and
//!   associative: every device that has seen the same items holds the same
//!   set, in whatever order it received them. An item without an `author`
//!   takes its set's when merged, so provenance survives merges. The merged
//!   set's version is its largest item version.
//! - **Trust (stage 0).** Sets are accepted as given: no
//!   signatures and no author allow-list. The author and versions are
//!   recorded and reported with every applied item. [`OverlaySetTrust`] is
//!   the extension point: a host that signs sets checks them there before a
//!   set reaches a workflow ([`RouteOverlaySet::from_json_trusted`]).
//! - **Content hash.** `contentHash`, when present, is the BLAKE3 hash (hex)
//!   of the set's canonical JSON without that field: object keys sorted by
//!   their UTF-8 bytes, no whitespace, numbers as `serde_json` writes them. A
//!   set whose hash does not match is rejected.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use thiserror::Error;
use wayfarer::{GeoPoint, MobilityProfile, RouteEdgeRange};

use wayfarer::{RouteOverlay, RouteOverlayGeometryKind, RouteOverlayPolicy, RouteOverlaySource};

/// The `format` of an overlay set file.
pub const OVERLAY_SET_FORMAT: &str = "wayfarer-overlay-set";
/// The overlay set file format version this crate reads and writes.
pub const OVERLAY_SET_FORMAT_VERSION: u32 = 1;
/// The most items one set may hold.
pub const MAX_OVERLAY_SET_ITEMS: usize = 4_096;
const MAX_OVERLAY_SET_STRING_BYTES: usize = 1_024;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteOverlaySet {
    pub format: String,
    pub format_version: u32,
    pub set_id: String,
    pub version: u64,
    #[serde(default)]
    pub created_at_ms: Option<i64>,
    #[serde(default)]
    pub author: Option<String>,
    /// The package the items' `edges` are bound to.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub package: Option<RouteOverlaySetPackage>,
    #[serde(default)]
    pub items: Vec<RouteOverlaySetItem>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub content_hash: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteOverlaySetPackage {
    #[serde(default)]
    pub graph_id: Option<String>,
    pub dataset_hash: String,
    #[serde(default)]
    pub format_version: Option<u32>,
}

/// What an item does.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum RouteOverlaySetItemKind {
    /// No-go.
    Block,
    /// Slow-go: a cost multiplier above 1.
    Slow,
    /// A hazard warning.
    Warn,
    /// Cheaper: a multiplier in (0, 1) or a speed override.
    Discount,
    /// Usable for the listed profiles whatever their legal access.
    Open,
}

impl From<RouteOverlaySetItemKind> for RouteOverlayPolicy {
    fn from(kind: RouteOverlaySetItemKind) -> Self {
        match kind {
            RouteOverlaySetItemKind::Block => Self::NoGo,
            RouteOverlaySetItemKind::Slow => Self::SlowGo,
            RouteOverlaySetItemKind::Warn => Self::Warning,
            RouteOverlaySetItemKind::Discount => Self::Discount,
            RouteOverlaySetItemKind::Open => Self::Open,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteOverlaySetItem {
    pub item_id: String,
    pub version: u64,
    /// Absent only on a tombstone.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub kind: Option<RouteOverlaySetItemKind>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub deleted: bool,
    /// The profiles the item applies to; empty: every profile.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub profiles: Vec<MobilityProfile>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub valid_from_ms: Option<i64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub valid_to_ms: Option<i64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub geometry: Option<RouteOverlaySetGeometry>,
    /// Edge ids of the bound package (`RouteOverlaySet::package`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub edges: Vec<RouteOverlaySetEdge>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cost_multiplier: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed_kph: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub provenance: Option<String>,
    /// Who wrote the item; a merge fills it from the item's set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub author: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteOverlaySetGeometry {
    pub kind: RouteOverlayGeometryKind,
    /// `[lat, lng]` pairs.
    pub points: Vec<[f64; 2]>,
    #[serde(default = "default_projection_distance_m")]
    pub projection_distance_m: u32,
}

fn default_projection_distance_m() -> u32 {
    20
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteOverlaySetEdge {
    pub edge_id: u64,
    #[serde(default)]
    pub start_fraction: f64,
    #[serde(default = "one")]
    pub end_fraction: f64,
}

fn one() -> f64 {
    1.0
}

#[derive(Debug, Error, PartialEq)]
pub enum OverlaySetError {
    #[error("not an overlay set: {0}")]
    Parse(String),
    #[error("invalid overlay set: {0}")]
    Invalid(&'static str),
    #[error("overlay set content hash does not match")]
    ContentHash,
    #[error("overlay sets {0} and {1} cannot be merged: different set ids")]
    DifferentSets(String, String),
    #[error("overlay set rejected: {0}")]
    Untrusted(String),
}

/// Decides whether a device accepts an overlay set, after it parsed and
/// validated. Stage 0 has no signing: [`AcceptAllOverlaySets`]
/// accepts every set. A host that signs sets (or keeps an author allow-list)
/// implements this and reads sets with [`RouteOverlaySet::from_json_trusted`];
/// a signature travels as an extra top-level field, which the parser ignores
/// and the content hash covers.
pub trait OverlaySetTrust {
    /// `raw` is the set's JSON as received, for checking a signature over it.
    fn accept(&self, set: &RouteOverlaySet, raw: &serde_json::Value) -> Result<(), String>;
}

/// Accepts every overlay set as given (stage 0).
#[derive(Debug, Clone, Copy, Default)]
pub struct AcceptAllOverlaySets;

impl OverlaySetTrust for AcceptAllOverlaySets {
    fn accept(&self, _set: &RouteOverlaySet, _raw: &serde_json::Value) -> Result<(), String> {
        Ok(())
    }
}

/// `value` with every object's keys sorted, written without whitespace.
fn canonical_json(value: &serde_json::Value) -> String {
    fn write(value: &serde_json::Value, out: &mut String) {
        match value {
            serde_json::Value::Object(map) => {
                let mut entries = map.iter().collect::<Vec<_>>();
                entries.sort_by(|left, right| left.0.as_bytes().cmp(right.0.as_bytes()));
                out.push('{');
                for (index, (key, value)) in entries.into_iter().enumerate() {
                    if index > 0 {
                        out.push(',');
                    }
                    out.push_str(&serde_json::Value::String(key.clone()).to_string());
                    out.push(':');
                    write(value, out);
                }
                out.push('}');
            }
            serde_json::Value::Array(values) => {
                out.push('[');
                for (index, value) in values.iter().enumerate() {
                    if index > 0 {
                        out.push(',');
                    }
                    write(value, out);
                }
                out.push(']');
            }
            other => out.push_str(&other.to_string()),
        }
    }
    let mut out = String::new();
    write(value, &mut out);
    out
}

/// The content hash of a set's JSON (`contentHash` itself left out).
pub fn overlay_set_content_hash(value: &serde_json::Value) -> String {
    let mut value = value.clone();
    if let Some(map) = value.as_object_mut() {
        map.remove("contentHash");
    }
    blake3::hash(canonical_json(&value).as_bytes())
        .to_hex()
        .to_string()
}

fn item_canonical_json(item: &RouteOverlaySetItem) -> String {
    canonical_json(&serde_json::to_value(item).expect("an overlay item serializes"))
}

fn bounded(text: &str) -> bool {
    !text.trim().is_empty()
        && text.len() <= MAX_OVERLAY_SET_STRING_BYTES
        && !text.chars().any(char::is_control)
}

impl RouteOverlaySet {
    /// Parses and validates a set, accepting it as given (stage 0 trust).
    pub fn from_json(text: &str) -> Result<Self, OverlaySetError> {
        Self::from_json_trusted(text, &AcceptAllOverlaySets)
    }

    /// Parses and validates a set, checks its content hash when it has one,
    /// then asks `trust` whether to accept it.
    pub fn from_json_trusted(
        text: &str,
        trust: &dyn OverlaySetTrust,
    ) -> Result<Self, OverlaySetError> {
        let raw: serde_json::Value = serde_json::from_str(text)
            .map_err(|error| OverlaySetError::Parse(error.to_string()))?;
        let set: Self = serde_json::from_value(raw.clone())
            .map_err(|error| OverlaySetError::Parse(error.to_string()))?;
        set.validate()?;
        if let Some(hash) = &set.content_hash
            && !hash.eq_ignore_ascii_case(&overlay_set_content_hash(&raw))
        {
            return Err(OverlaySetError::ContentHash);
        }
        trust
            .accept(&set, &raw)
            .map_err(OverlaySetError::Untrusted)?;
        Ok(set)
    }

    /// The set as JSON with a fresh `contentHash`.
    pub fn to_json_with_hash(&self) -> String {
        let mut set = self.clone();
        set.content_hash = None;
        let mut value = serde_json::to_value(&set).expect("an overlay set serializes");
        let hash = overlay_set_content_hash(&value);
        value
            .as_object_mut()
            .expect("a set is an object")
            .insert("contentHash".to_string(), serde_json::Value::String(hash));
        value.to_string()
    }

    pub fn validate(&self) -> Result<(), OverlaySetError> {
        use OverlaySetError::Invalid;
        if self.format != OVERLAY_SET_FORMAT {
            return Err(Invalid("format"));
        }
        if self.format_version != OVERLAY_SET_FORMAT_VERSION {
            return Err(Invalid("format version"));
        }
        if !bounded(&self.set_id) {
            return Err(Invalid("set id"));
        }
        if self
            .author
            .as_deref()
            .is_some_and(|author| !bounded(author))
        {
            return Err(Invalid("author"));
        }
        if self.items.len() > MAX_OVERLAY_SET_ITEMS {
            return Err(Invalid("too many items"));
        }
        let mut ids = std::collections::BTreeSet::new();
        for item in &self.items {
            if !bounded(&item.item_id) || !ids.insert(item.item_id.as_str()) {
                return Err(Invalid("item id"));
            }
            for text in [&item.note, &item.provenance, &item.author]
                .into_iter()
                .flatten()
            {
                if text.len() > MAX_OVERLAY_SET_STRING_BYTES || text.chars().any(char::is_control) {
                    return Err(Invalid("item text"));
                }
            }
            if item.deleted {
                continue;
            }
            let Some(kind) = item.kind else {
                return Err(Invalid("item kind"));
            };
            if let (Some(from), Some(to)) = (item.valid_from_ms, item.valid_to_ms)
                && to <= from
            {
                return Err(Invalid("item time window"));
            }
            if item.geometry.is_none() && item.edges.is_empty() {
                return Err(Invalid("item has neither geometry nor edges"));
            }
            if let Some(geometry) = &item.geometry
                && geometry.points.iter().any(|[lat, lng]| {
                    !lat.is_finite()
                        || !lng.is_finite()
                        || !(-90.0..=90.0).contains(lat)
                        || !(-180.0..=180.0).contains(lng)
                })
            {
                return Err(Invalid("item geometry"));
            }
            if item.edges.iter().any(|edge| {
                !(edge.start_fraction.is_finite()
                    && edge.end_fraction.is_finite()
                    && (0.0..=1.0).contains(&edge.start_fraction)
                    && (0.0..=1.0).contains(&edge.end_fraction)
                    && edge.end_fraction > edge.start_fraction)
            }) {
                return Err(Invalid("item edges"));
            }
            let multiplier_ok = |range: std::ops::RangeInclusive<f64>| {
                item.cost_multiplier
                    .is_none_or(|value| value.is_finite() && range.contains(&value))
            };
            let speed_ok = item
                .speed_kph
                .is_none_or(|speed| speed.is_finite() && speed > 0.0 && speed <= 300.0);
            let valid = match kind {
                RouteOverlaySetItemKind::Slow => item
                    .cost_multiplier
                    .is_none_or(|m| m.is_finite() && m > 1.0),
                RouteOverlaySetItemKind::Discount => {
                    multiplier_ok(f64::MIN_POSITIVE..=1.0 - f64::EPSILON)
                        && speed_ok
                        && (item.cost_multiplier.is_some() || item.speed_kph.is_some())
                }
                RouteOverlaySetItemKind::Open => speed_ok && item.cost_multiplier.is_none(),
                RouteOverlaySetItemKind::Block | RouteOverlaySetItemKind::Warn => true,
            };
            if !valid {
                return Err(Invalid("item cost multiplier or speed"));
            }
        }
        Ok(())
    }

    /// Merges sets of one `setId` (see the module docs): per item the higher
    /// version, at equal versions the tombstone, else the greater canonical
    /// JSON. The result's version is its largest item version, its author
    /// and creation time the first set's.
    pub fn merge(sets: &[Self]) -> Result<Option<Self>, OverlaySetError> {
        let Some(first) = sets.first() else {
            return Ok(None);
        };
        let mut items = BTreeMap::<String, (RouteOverlaySetItem, String)>::new();
        for set in sets {
            if set.set_id != first.set_id {
                return Err(OverlaySetError::DifferentSets(
                    first.set_id.clone(),
                    set.set_id.clone(),
                ));
            }
            for item in &set.items {
                let mut item = item.clone();
                if item.author.is_none() {
                    item.author.clone_from(&set.author);
                }
                let key = item_canonical_json(&item);
                let wins = |current: &(RouteOverlaySetItem, String)| {
                    (item.version, item.deleted, key.as_bytes())
                        > (current.0.version, current.0.deleted, current.1.as_bytes())
                };
                match items.get(&item.item_id) {
                    Some(current) if !wins(current) => {}
                    _ => {
                        items.insert(item.item_id.clone(), (item, key));
                    }
                }
            }
        }
        let items = items
            .into_values()
            .map(|(item, _)| item)
            .collect::<Vec<_>>();
        Ok(Some(Self {
            format: OVERLAY_SET_FORMAT.to_string(),
            format_version: OVERLAY_SET_FORMAT_VERSION,
            set_id: first.set_id.clone(),
            version: items.iter().map(|item| item.version).max().unwrap_or(0),
            created_at_ms: first.created_at_ms,
            author: first.author.clone(),
            package: sets.iter().rev().find_map(|set| set.package.clone()),
            items,
            content_hash: None,
        }))
    }

    /// The set's live items as workflow overlays for a package with
    /// `dataset_hash`: bound edges where the set is bound to that package,
    /// else the geometry, re-projected. Tombstones and items with neither are
    /// left out and reported.
    pub fn workflow_overlays(
        &self,
        dataset_hash: &str,
    ) -> (Vec<RouteOverlay>, RouteOverlaySetBinding) {
        let bound = self
            .package
            .as_ref()
            .is_some_and(|package| package.dataset_hash.eq_ignore_ascii_case(dataset_hash));
        let mut binding = RouteOverlaySetBinding {
            set_id: self.set_id.clone(),
            set_version: self.version,
            author: self.author.clone(),
            bound_to_package: bound,
            ..RouteOverlaySetBinding::default()
        };
        let mut overlays = Vec::new();
        for item in &self.items {
            if item.deleted {
                binding.tombstone_count += 1;
                continue;
            }
            let Some(kind) = item.kind else {
                continue;
            };
            let edges = if bound {
                item.edges
                    .iter()
                    .map(|edge| RouteEdgeRange {
                        edge_id: edge.edge_id,
                        start_fraction: edge.start_fraction,
                        end_fraction: edge.end_fraction,
                    })
                    .collect::<Vec<_>>()
            } else {
                Vec::new()
            };
            let (geometry_kind, points, projection_distance_m) = match &item.geometry {
                Some(geometry) => (
                    geometry.kind,
                    geometry
                        .points
                        .iter()
                        .map(|[lat, lng]| GeoPoint::new(*lat, *lng))
                        .collect::<Vec<_>>(),
                    geometry.projection_distance_m,
                ),
                None if !edges.is_empty() => (RouteOverlayGeometryKind::Point, Vec::new(), 0),
                None => {
                    binding.skipped_item_ids.push(item.item_id.clone());
                    continue;
                }
            };
            if edges.is_empty() {
                binding.reprojected_item_count += 1;
            } else {
                binding.bound_item_count += 1;
            }
            overlays.push(RouteOverlay {
                geometry_kind,
                points,
                policy: kind.into(),
                projection_distance_m,
                cost_multiplier: item.cost_multiplier,
                speed_kph: item.speed_kph,
                profiles: item.profiles.clone(),
                valid_from_ms: item.valid_from_ms,
                valid_to_ms: item.valid_to_ms,
                edges,
                source: Some(RouteOverlaySource {
                    set_id: self.set_id.clone(),
                    set_version: self.version,
                    item_id: item.item_id.clone(),
                    item_version: item.version,
                    author: item.author.clone().or_else(|| self.author.clone()),
                }),
            });
        }
        (overlays, binding)
    }
}

/// How a set's items reached a workflow (`RouteWorkflowResult::overlay_sets`).
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase")]
pub struct RouteOverlaySetBinding {
    pub set_id: String,
    pub set_version: u64,
    pub author: Option<String>,
    /// The set's `package.datasetHash` is the routing package's: its edge ids
    /// are used as they stand.
    pub bound_to_package: bool,
    /// Items applied by their bound edge ids.
    pub bound_item_count: usize,
    /// Items applied by projecting their geometry.
    pub reprojected_item_count: usize,
    pub tombstone_count: usize,
    /// Items with no geometry whose edges are bound to another package.
    pub skipped_item_ids: Vec<String>,
}

//! The routing road graph: directed edges over shared way attributes, built
//! from normalized OSM ways (`RoadGraphBuilder`), enhanced (components,
//! junctions, opposing edges, turn restrictions) and checked for quality
//! (`RoadGraphQualityManifest`). The route compiler turns it into a routing
//! package; its vocabulary (`RoadEdge`, `SharedEdgeInfo`, ...) lives in the
//! routing package format crate.
//!
//! ```
//! use wayfarer_road_graph::{
//!     DirectionalAccess, RoadGraphBuilder, RoadType, RoutableWay, enhance_road_graph,
//! };
//!
//! let mut builder = RoadGraphBuilder::new("example");
//! builder.push_way(RoutableWay {
//!     osm_way_id: 1,
//!     points: vec![(52.0, 21.0), (52.0, 21.001), (52.0, 21.002)], // (lat, lng)
//!     road_type: RoadType::Primary,
//!     directional_access: DirectionalAccess::Both,
//!     name: Some("Example road".into()),
//!     highway: Some("primary".into()),
//!     access: Some("yes".into()),
//!     ..RoutableWay::default()
//! });
//!
//! let mut graph = builder.finish();
//! enhance_road_graph(&mut graph);
//! println!("{} nodes, {} directed edges", graph.node_count(), graph.edge_count());
//! ```
//!
//! # Stability
//!
//! Version 0.x: the API may change between minor releases. The snapshot written
//! by `serialize_road_graph` has its own versions ([`ROAD_GRAPH_SNAPSHOT_VERSION`]
//! and [`ROAD_GRAPH_CONTAINER_VERSION`]); it is an intermediate form, not the
//! distribution format, which is the routing package.

use geo::{Centroid, Contains, InteriorPoint, LineString};
use petgraph::Direction;
#[cfg(test)]
use petgraph::graph::Graph;
use petgraph::graph::{EdgeIndex, NodeIndex};
#[cfg(test)]
use petgraph::visit::EdgeRef;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque};
use std::sync::Arc;
use wayfarer_package::math;

use rayon::prelude::*;
use rustc_hash::FxHashMap;
use wayfarer_package::sar::{
    LegalReason, SarBarrierClass, path_physically_drivable, vehicle_drivable_class,
};

pub use wayfarer_package::{
    DirectionalAccess, MobilityMode, ModeMask, OsmMode, RoadEdge, RoadGraphNamedCount,
    RoadHierarchyTier, RoadJunctionKind, RoadNode, RoadType, SharedEdgeInfo, TrafficControl,
    TraversalDirection,
};

mod builder;
mod container;
mod enhance;
mod geometry;
mod graph;
mod network;
mod plazas;
mod quality;
mod report;
mod shared_infos;
mod speed;
mod tags;
mod types;
mod warnings;

pub use builder::*;
pub use container::*;
pub use enhance::*;
use geometry::*;
pub use graph::*;
pub use network::{Adjacency, RoadEdgeRef, RoadNetwork};
use plazas::*;
pub use quality::*;
pub use report::*;
pub use shared_infos::{
    InfoGeometry, SharedEdge, SharedEdgeInfos, SharedInfoClasses, SharedInfoRef,
};
pub use speed::*;
pub use tags::*;
pub use types::*;
use warnings::*;

#[cfg(feature = "snapshot")]
mod snapshot;
#[cfg(all(test, feature = "snapshot"))]
use snapshot::serialize_road_graph_snapshot;
#[cfg(feature = "snapshot")]
pub use snapshot::{
    deserialize_road_graph, load_road_graph_file, persist_road_graph_file,
    persist_road_graph_file_with_info, serialize_road_graph,
};

mod restriction_resolution;
#[cfg(test)]
mod tests;
pub use restriction_resolution::resolve_turn_restrictions_with_report;

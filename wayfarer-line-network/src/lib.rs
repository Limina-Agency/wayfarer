//! Line networks for the compilers: OSM ways split at shared nodes into a
//! topology of segments with merged attributes, so routing and rendering see
//! the same junctions.
//!
//! ```
//! use wayfarer_line_network::{
//!     DirectionalAccessModel, RoadRoutingAttrs, SharedLineInput, SharedLineLayer,
//!     SharedLineNetworkBuilder,
//! };
//!
//! fn road(feature_id: i64, geometry: Vec<(f64, f64)>, node_chain: Vec<i64>) -> SharedLineInput {
//!     SharedLineInput {
//!         feature_id,
//!         layer: SharedLineLayer::Roads,
//!         geometry, // (lat, lng) per vertex
//!         node_chain, // the OSM node id of each vertex
//!         source_lineage_ids: vec![feature_id],
//!         attrs: Default::default(),
//!         compile_inputs: Default::default(),
//!         min_zoom: 0,
//!         max_zoom: 14,
//!         sort_order: 0,
//!         routing: Some(RoadRoutingAttrs {
//!             road_class: "residential".into(),
//!             directional_access: DirectionalAccessModel::Both,
//!             access: None,
//!             surface: None,
//!             tracktype: None,
//!             smoothness: None,
//!             has_ford: false,
//!             has_tunnel: false,
//!             access_restricted: false,
//!         }),
//!     }
//! }
//!
//! // Two ways that cross at OSM node 2.
//! let mut builder = SharedLineNetworkBuilder::default();
//! builder.push_input(road(10, vec![(52.000, 21.000), (52.001, 21.001), (52.002, 21.002)], vec![1, 2, 3]));
//! builder.push_input(road(11, vec![(52.002, 21.000), (52.001, 21.001), (52.000, 21.002)], vec![4, 2, 5]));
//!
//! // Topology only: no render products, which a routing build would discard.
//! let network = builder.finish_topology_only();
//!
//! // Each way is split at the shared node: four segments meeting at one junction.
//! assert_eq!(network.topology.edges.len(), 4);
//! assert_eq!(network.road_segments().count(), 4);
//! ```
//!
//! # Stability
//!
//! Version 0.x: the API may change between minor releases. The render-oriented
//! fields (`min_zoom`, `max_zoom`, `sort_order`, `compile_inputs`) serve the
//! vector basemap compiler and are ignored by routing.

use rayon::prelude::*;
use rustc_hash::FxHashMap;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::f64::consts::PI;
use std::sync::Arc;

mod attrs;
mod builder;
mod geometry;
mod render;
mod repair;
mod topology;
mod types;

use attrs::*;
pub use builder::*;
use geometry::*;
use render::*;
use repair::*;
use topology::*;
pub use types::*;

#[cfg(test)]
mod tests;

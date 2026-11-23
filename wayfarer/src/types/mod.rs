//! The engine's request and result types, by concern.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use thiserror::Error;
pub use wayfarer_costing::{
    RouteCostBreakdown, RouteCostingOptions, RouteSpeedPreference, RouteSpeedSource,
};
pub(crate) use wayfarer_package::haversine_m;
pub use wayfarer_package::{GeoPoint, MobilityProfile};
use wayfarer_package::{RoadType, TraversalDirection};

use crate::cancellation::RouteCancellation;
use crate::geometry::GeoJsonLineString;

mod analysis;
mod correlation;
mod dynamic_overlay;
mod leg;
mod options;

pub use analysis::*;
pub use correlation::*;
pub use dynamic_overlay::*;
pub use leg::*;
pub use options::*;

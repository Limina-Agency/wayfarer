//! Packages built in memory with the route compiler (a dev-dependency only)
//! for the unit tests.

use std::sync::Arc;

pub(crate) use wayfarer_compiler::package_build::{PackageBuildOptions, PackageBuildReport};
use wayfarer_road_graph::RoadGraph;

use crate::{RoutingEngine, RoutingPackage, RoutingPackageError};

/// The package of `graph`, built in memory.
pub(crate) fn package(
    graph: &RoadGraph,
    options: &PackageBuildOptions,
) -> Result<(RoutingPackage, PackageBuildReport), RoutingPackageError> {
    let (bytes, report) = wayfarer_compiler::package_build::package_bytes(graph, options)?;
    Ok((RoutingPackage::from_bytes(&bytes)?, report))
}

/// The unit tests' package options: the compiler's default (with the CCH
/// order, so searches bound with the potentials).
pub(crate) fn test_options() -> PackageBuildOptions {
    PackageBuildOptions::default()
}

/// An engine over the package of `graph`, built with [`test_options`].
pub(crate) fn engine(graph: &RoadGraph) -> RoutingEngine {
    let (package, _) =
        package(graph, &test_options()).expect("the road graph builds a routing package");
    RoutingEngine::from_package(Arc::new(package))
}

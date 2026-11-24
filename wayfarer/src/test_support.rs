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
/// order, so searches bound with the potentials), in format 2, which keeps
/// the road graph's node, edge and shared-info ids. The reference oracle
/// and the tests' fixtures name elements by those ids; format 3 renumbers
/// them by CCH rank (`wayfarer/tests/package_format_3.rs` checks that it
/// routes as format 2 does).
pub(crate) fn test_options() -> PackageBuildOptions {
    PackageBuildOptions {
        package_format: wayfarer_package::format::PackageFormat::V2,
        ..PackageBuildOptions::default()
    }
}

/// An engine over the package of `graph`, built with [`test_options`].
pub(crate) fn engine(graph: &RoadGraph) -> RoutingEngine {
    let (package, _) =
        package(graph, &test_options()).expect("the road graph builds a routing package");
    RoutingEngine::from_package(Arc::new(package))
}

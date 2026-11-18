//! `wayfarer info` and `wayfarer verify`: a package's description and its
//! checksums, as JSON on stdout.

use std::path::PathBuf;

use clap::Args;
use serde_json::json;
use wayfarer_package::format::{
    MIN_READ_FORMAT_VERSION, PackageFile, ROUTING_PACKAGE_FORMAT_VERSION,
};

use crate::{Failure, print_json};

/// Print a package's format, costing model, counts and dataset hash as JSON.
#[derive(Debug, Args)]
pub(crate) struct InfoArgs {
    /// The routing package.
    #[arg(value_name = "routing.mrt")]
    package: PathBuf,
}

/// Check every section of a package against its checksum; print the sections
/// as JSON.
#[derive(Debug, Args)]
pub(crate) struct VerifyArgs {
    /// The routing package.
    #[arg(value_name = "routing.mrt")]
    package: PathBuf,
}

pub(crate) fn info(args: InfoArgs) -> Result<(), Failure> {
    let path = &args.package;
    let package = PackageFile::open(path)
        .map_err(|error| Failure::new(format!("{}: {error}", path.display())))?;
    let metadata = package.metadata();
    let counts = package.counts();
    print_json(&json!({
        "formatVersion": package.format_version(),
        "readableFormats": (MIN_READ_FORMAT_VERSION..=ROUTING_PACKAGE_FORMAT_VERSION).collect::<Vec<_>>(),
        "costingModelVersion": metadata.costing_model_version,
        "graphId": metadata.graph_id,
        "countryCode": metadata.country_code,
        "datasetHash": package.dataset_hash(),
        "byteLen": package.byte_len(),
        "nodeCount": counts.node_count,
        "edgeCount": counts.edge_count,
        "sharedInfoCount": counts.shared_info_count,
        "profileColumnCount": counts.column_count,
        "quality": {
            "status": metadata.quality.status,
            "reasons": metadata.quality.reasons,
        },
    }))
}

pub(crate) fn verify(args: VerifyArgs) -> Result<(), Failure> {
    let path = &args.package;
    let verification = PackageFile::verify(path)
        .map_err(|error| Failure::new(format!("{}: {error}", path.display())))?;
    print_json(&json!({
        "datasetHash": verification.dataset_hash,
        "byteLen": verification.byte_len,
        "sections": verification.sections.iter().map(|section| json!({
            "name": section.name,
            "byteLen": section.byte_len,
            "blake3": section.blake3_hex,
        })).collect::<Vec<_>>(),
    }))
}

//! Prints a routing package's identity, open time, and section table, and
//! with `--verify` checks every section against its checksum.
//!
//! ```sh
//! cargo run --release -p wayfarer --example inspect_routing_package -- \
//!   <routing.mrt> [--verify]
//! ```
use serde::Serialize;
use std::path::PathBuf;
use std::time::Instant;
use wayfarer::{ROUTING_PACKAGE_FORMAT_VERSION, RoutingPackage};

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct PackageInspection {
    format_version: u32,
    dataset_hash: String,
    graph_id: String,
    byte_len: usize,
    node_count: usize,
    edge_count: usize,
    open_ms: f64,
    verify_ms: Option<f64>,
    sections: Vec<(String, usize)>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args_os().skip(1);
    let path = args
        .next()
        .map(PathBuf::from)
        .ok_or("usage: inspect_routing_package <routing.mrt> [--verify]")?;
    let verify = args.next().is_some_and(|flag| flag == "--verify");
    let started = Instant::now();
    let package = RoutingPackage::open(&path)?;
    let open_ms = started.elapsed().as_secs_f64() * 1e3;
    let verify_ms = verify
        .then(|| {
            let started = Instant::now();
            RoutingPackage::verify(&path).map(|_| started.elapsed().as_secs_f64() * 1e3)
        })
        .transpose()?;
    println!(
        "{}",
        serde_json::to_string_pretty(&PackageInspection {
            format_version: ROUTING_PACKAGE_FORMAT_VERSION,
            dataset_hash: package.dataset_hash().to_string(),
            graph_id: package.graph_id().to_string(),
            byte_len: package.byte_len(),
            node_count: package.counts().node_count,
            edge_count: package.counts().edge_count,
            open_ms,
            verify_ms,
            sections: package
                .section_sizes()
                .into_iter()
                .map(|(name, len)| (name.to_string(), len))
                .collect(),
        })?
    );
    Ok(())
}

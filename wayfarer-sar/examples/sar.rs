//! Runs a planning step on a package from JSON files.
//!
//! ```text
//! cargo run --release -p wayfarer-sar --example sar -- reach routing.mrt request.json result.json [map.geojson]
//! cargo run --release -p wayfarer-sar --example sar -- teams routing.mrt request.json result.json
//! cargo run --release -p wayfarer-sar --example sar -- coverage routing.mrt request.json result.json
//! ```

use std::time::Instant;

use wayfarer::RoutingEngine;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = std::env::args().collect::<Vec<_>>();
    let [_, step, package, request, result, rest @ ..] = args.as_slice() else {
        return Err(
            "usage: sar reach|teams|coverage PACKAGE REQUEST.json RESULT.json [MAP.geojson]".into(),
        );
    };
    let engine = RoutingEngine::open(package.as_ref())?;
    let input = std::fs::read_to_string(request)?;
    let started = Instant::now();
    let output = match step.as_str() {
        "reach" => {
            let reach = wayfarer_sar::predict_reach(&engine, &serde_json::from_str(&input)?)?;
            eprintln!("reach: {:.1} ms", started.elapsed().as_secs_f64() * 1e3);
            if let Some(map) = rest.first() {
                let collection = wayfarer_sar::geojson::reach_feature_collection(&engine, &reach);
                std::fs::write(map, serde_json::to_string(&collection)?)?;
            }
            serde_json::to_string_pretty(&reach)?
        }
        "teams" => {
            let plan = wayfarer_sar::plan_teams(&engine, &serde_json::from_str(&input)?)?;
            eprintln!("teams: {:.1} ms", started.elapsed().as_secs_f64() * 1e3);
            serde_json::to_string_pretty(&plan)?
        }
        "coverage" => {
            let coverage = wayfarer_sar::apply_coverage(&engine, &serde_json::from_str(&input)?)?;
            eprintln!("coverage: {:.1} ms", started.elapsed().as_secs_f64() * 1e3);
            serde_json::to_string_pretty(&coverage)?
        }
        other => return Err(format!("unknown step {other}").into()),
    };
    std::fs::write(result, output)?;
    Ok(())
}

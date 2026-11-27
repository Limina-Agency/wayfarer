//! `wayfarer sar reach | teams | coverage`: the search and rescue planning
//! steps of `wayfarer-sar` on a routing package, from the request JSON (the
//! C ABI's `wayfarer_sar_*` take the same), with the result as JSON (or, for
//! a reach, GeoJSON) on stdout.

use std::path::{Path, PathBuf};

use clap::{Args, Subcommand};
use serde::de::DeserializeOwned;
use wayfarer::{GeoPoint, RouteIsochronePolygonOptions, RoutingEngine};
use wayfarer_sar::{
    CoverageRequest, ReachRequest, TeamPlanRequest, apply_coverage, geojson, plan_teams,
    predict_reach,
};

use crate::query::{parse_point, read_text};
use crate::{Failure, print_json};

/// Search and rescue planning: where a missing person could be, which team
/// goes where, and what a search has covered.
#[derive(Debug, Args)]
pub(crate) struct SarArgs {
    #[command(subcommand)]
    command: SarCommand,
}

#[derive(Debug, Subcommand)]
enum SarCommand {
    /// Predict the reach, edge probabilities and likely routes from a last
    /// known position and the time since.
    Reach(ReachArgs),
    /// Plan teams onto search segments: arrival times, a first assignment and
    /// each team's territory within a shift.
    Teams(StepArgs),
    /// Map team GPS tracks onto the paths they covered and update the
    /// probabilities after an unsuccessful search.
    Coverage(StepArgs),
}

#[derive(Debug, Args)]
struct StepArgs {
    /// The routing package.
    #[arg(value_name = "routing.mrt")]
    package: PathBuf,
    /// The request as JSON (a file, or `-` for stdin).
    #[arg(long, value_name = "FILE")]
    request: PathBuf,
}

#[derive(Debug, Args)]
struct ReachArgs {
    /// The routing package.
    #[arg(value_name = "routing.mrt")]
    package: PathBuf,
    /// The reach request as JSON (a file, or `-` for stdin), instead of the
    /// flags.
    #[arg(long, value_name = "FILE", required_unless_present = "last_known")]
    request: Option<PathBuf>,
    /// The last known position, `lat,lng`.
    #[arg(long, value_parser = parse_point, conflicts_with = "request", requires = "hours", allow_hyphen_values = true)]
    last_known: Option<GeoPoint>,
    /// Hours since the person was last seen.
    #[arg(long, conflicts_with = "request")]
    hours: Option<f64>,
    /// How far the last known position may be off, in metres.
    #[arg(long, default_value_t = 0.0, conflicts_with = "request")]
    uncertainty_m: f64,
    /// List at most this many edges in the probability map, the most
    /// probable first (overrides the request's `maxEdges`).
    #[arg(long)]
    max_edges: Option<usize>,
    /// Draw each person's reach as a polygon (`areas[].polygon`; with
    /// `--geojson`, a `reach` feature each).
    #[arg(long)]
    polygons: bool,
    /// Snap an off-path last known position to the paths for free instead of
    /// pricing the walk to them (overrides the request's `offTrail`).
    #[arg(long)]
    no_off_trail: bool,
    /// Print the reach as a GeoJSON FeatureCollection (last known position,
    /// radius circles, reach polygons, edge probabilities, routes, corridors).
    #[arg(long)]
    geojson: bool,
}

pub(crate) fn run(args: SarArgs) -> Result<(), Failure> {
    match args.command {
        SarCommand::Reach(args) => reach(args),
        SarCommand::Teams(args) => {
            let engine = open(&args.package)?;
            let request: TeamPlanRequest = read(&args.request)?;
            print_json(&plan_teams(&engine, &request).map_err(failure)?)
        }
        SarCommand::Coverage(args) => {
            let engine = open(&args.package)?;
            let request: CoverageRequest = read(&args.request)?;
            print_json(&apply_coverage(&engine, &request).map_err(failure)?)
        }
    }
}

fn reach(args: ReachArgs) -> Result<(), Failure> {
    let engine = open(&args.package)?;
    let mut request = match (&args.request, args.last_known) {
        (Some(path), _) => read::<ReachRequest>(path)?,
        (None, Some(last_known)) => {
            let mut request = ReachRequest::new(last_known);
            request.elapsed_s = args.hours.map(|hours| hours * 3_600.0);
            request.uncertainty_m = args.uncertainty_m;
            request
        }
        (None, None) => unreachable!("clap requires --request or --last-known"),
    };
    if args.max_edges.is_some() {
        request.max_edges = args.max_edges;
    }
    if args.polygons && request.polygons.is_none() {
        request.polygons = Some(RouteIsochronePolygonOptions::default());
    }
    if args.no_off_trail {
        request.off_trail = false;
    }
    let result = predict_reach(&engine, &request).map_err(failure)?;
    if args.geojson {
        print_json(&geojson::reach_feature_collection(&engine, &result))
    } else {
        print_json(&result)
    }
}

fn open(package: &Path) -> Result<RoutingEngine, Failure> {
    RoutingEngine::open(package)
        .map_err(|error| Failure::new(format!("{}: {error}", package.display())))
}

fn read<T: DeserializeOwned>(path: &Path) -> Result<T, Failure> {
    serde_json::from_str(&read_text(path)?)
        .map_err(|error| Failure::new(format!("{}: {error}", path.display())))
}

fn failure(error: wayfarer_sar::SarError) -> Failure {
    Failure::new(error.to_string())
}

#[cfg(test)]
mod tests {
    use clap::Parser;

    use super::*;

    #[derive(Parser)]
    struct Wrapper {
        #[command(subcommand)]
        command: SarCommand,
    }

    fn parse(args: &[&str]) -> Result<SarCommand, clap::Error> {
        Wrapper::try_parse_from(std::iter::once("sar").chain(args.iter().copied()))
            .map(|wrapper| wrapper.command)
    }

    #[test]
    fn reach_takes_a_request_or_the_flags() {
        let Ok(SarCommand::Reach(args)) = parse(&[
            "reach",
            "a.mrt",
            "--last-known",
            "49.24,-20.0",
            "--hours",
            "3",
            "--uncertainty-m",
            "300",
            "--max-edges",
            "500",
            "--polygons",
            "--no-off-trail",
            "--geojson",
        ]) else {
            panic!("reach flags");
        };
        assert_eq!(args.last_known, Some(GeoPoint::new(49.24, -20.0)));
        assert_eq!(args.hours, Some(3.0));
        assert_eq!(args.max_edges, Some(500));
        assert!(args.geojson && args.polygons && args.no_off_trail && args.request.is_none());
        assert!(parse(&["reach", "a.mrt", "--request", "r.json"]).is_ok());
        // A position needs the time since; the flags and a request exclude
        // each other; one of them is needed.
        assert!(parse(&["reach", "a.mrt", "--last-known", "1,1"]).is_err());
        assert!(parse(&["reach", "a.mrt", "--request", "r.json", "--hours", "1"]).is_err());
        assert!(parse(&["reach", "a.mrt"]).is_err());
        assert!(parse(&["teams", "a.mrt", "--request", "t.json"]).is_ok());
        assert!(parse(&["coverage", "a.mrt"]).is_err());
    }
}

//! `wayfarer route | matrix | isochrone | match | snap`: requests on a
//! routing package, from flags or from the request JSON the C ABI takes
//! (`--request`), with the result as JSON (or GeoJSON) on stdout.

use std::io::Read;
use std::path::{Path, PathBuf};

use clap::Args;
use serde::de::DeserializeOwned;
use serde_json::{Value, json};
use wayfarer::{
    GeoPoint, MapMatchRequest, MobilityProfile, RouteIsochroneContour, RouteIsochroneMetric,
    RouteIsochroneRequest, RouteMatrixRequest, RouteRequest, RoutingEngine, SnapRequest,
};
use wayfarer_service::{
    RouteWorkflowRequest, RouteWorkflowResult, RouteWorkflowSourceContext, RoutingSourceId,
    compile_route_workflow,
};

use crate::{Failure, print_json};

/// `lat,lng` in degrees.
fn parse_point(text: &str) -> Result<GeoPoint, String> {
    let (lat, lng) = text
        .split_once(',')
        .ok_or_else(|| format!("`{text}` is not `lat,lng`"))?;
    let parse = |value: &str| {
        value
            .trim()
            .parse::<f64>()
            .map_err(|_| format!("`{text}` is not `lat,lng`"))
    };
    Ok(GeoPoint::new(parse(lat)?, parse(lng)?))
}

/// A mobility profile by its JSON name (`wheeled`, `on_foot`, …).
fn parse_profile(text: &str) -> Result<MobilityProfile, String> {
    serde_json::from_value(Value::String(text.to_string())).map_err(|_| {
        format!("unknown profile `{text}` (wheeled, on_foot, tracked, emergency, legged)")
    })
}

/// The package and the request source every query takes.
#[derive(Debug, Args)]
pub(crate) struct Common {
    /// The routing package.
    #[arg(value_name = "routing.mrt")]
    package: PathBuf,
    /// The mobility profile: wheeled, on_foot, tracked, emergency or legged.
    #[arg(long, default_value = "wheeled", value_parser = parse_profile)]
    profile: MobilityProfile,
    /// The request as JSON (a file, or `-` for stdin), as the C ABI takes it,
    /// instead of the point flags.
    #[arg(long, value_name = "FILE")]
    request: Option<PathBuf>,
}

impl Common {
    fn engine(&self) -> Result<RoutingEngine, Failure> {
        RoutingEngine::open(&self.package)
            .map_err(|error| Failure::new(format!("{}: {error}", self.package.display())))
    }

    /// The `--request` JSON, if given.
    fn request<T: DeserializeOwned>(&self) -> Result<Option<T>, Failure> {
        let Some(path) = &self.request else {
            return Ok(None);
        };
        let text = read_text(path)?;
        serde_json::from_str(&text)
            .map(Some)
            .map_err(|error| Failure::new(format!("{}: {error}", path.display())))
    }
}

fn read_text(path: &Path) -> Result<String, Failure> {
    if path == Path::new("-") {
        let mut text = String::new();
        std::io::stdin()
            .read_to_string(&mut text)
            .map_err(|error| Failure::new(format!("stdin: {error}")))?;
        Ok(text)
    } else {
        std::fs::read_to_string(path)
            .map_err(|error| Failure::new(format!("{}: {error}", path.display())))
    }
}

/// Route through two or more points (`--from`, any `--via`, `--to`), or a
/// route request with overlays (`--request`).
#[derive(Debug, Args)]
pub(crate) struct RouteArgs {
    #[command(flatten)]
    common: Common,
    /// The origin, `lat,lng`.
    #[arg(long, value_parser = parse_point, required_unless_present = "request", allow_hyphen_values = true)]
    from: Option<GeoPoint>,
    /// A waypoint, `lat,lng` (repeatable, in order).
    #[arg(long, value_parser = parse_point, allow_hyphen_values = true)]
    via: Vec<GeoPoint>,
    /// The destination, `lat,lng`.
    #[arg(long, value_parser = parse_point, required_unless_present = "request", allow_hyphen_values = true)]
    to: Option<GeoPoint>,
    /// Print the route as a GeoJSON FeatureCollection (the line and its
    /// locations) instead of the route JSON.
    #[arg(long)]
    geojson: bool,
}

pub(crate) fn route(args: RouteArgs) -> Result<(), Failure> {
    let engine = args.common.engine()?;
    let request = match args.common.request::<RouteWorkflowRequest>()? {
        Some(request) => request,
        None => {
            let points = args
                .from
                .into_iter()
                .chain(args.via.iter().copied())
                .chain(args.to);
            RouteRequest::through(points, args.common.profile).into()
        }
    };
    let package = engine.package();
    let source = RouteWorkflowSourceContext {
        source_id: RoutingSourceId::new(package.graph_id())
            .or_else(|_| RoutingSourceId::new("package"))
            .map_err(|error| Failure::new(error.to_string()))?,
        revision: package.dataset_hash().to_string(),
        cache_hit: false,
    };
    let result = compile_route_workflow(&engine, source, request)
        .map_err(|error| Failure::new(error.to_string()))?;
    if args.geojson {
        print_json(&route_geojson(&result))
    } else {
        print_json(&result)
    }
}

fn coordinates(point: GeoPoint) -> Value {
    json!([point.lng, point.lat])
}

/// The route's line and its locations as GeoJSON (longitude first).
fn route_geojson(result: &RouteWorkflowResult) -> Value {
    let route = &result.route;
    let mut features = vec![json!({
        "type": "Feature",
        "geometry": {
            "type": "LineString",
            "coordinates": route.geometry.iter().copied().map(coordinates).collect::<Vec<_>>(),
        },
        "properties": {
            "distanceM": route.distance_m,
            "durationS": route.duration_s,
            "cost": route.cost,
            "legs": route.segments.len(),
        },
    })];
    for (index, matched) in route.location_matches.iter().enumerate() {
        if let Some(location) = matched.preferred() {
            features.push(json!({
                "type": "Feature",
                "geometry": { "type": "Point", "coordinates": coordinates(location.snapped_point) },
                "properties": { "location": index, "edgeId": location.edge_id, "snapDistanceM": location.distance_m },
            }));
        }
    }
    json!({ "type": "FeatureCollection", "features": features })
}

/// Distances and durations from every `--source` to every `--target`.
#[derive(Debug, Args)]
pub(crate) struct MatrixArgs {
    #[command(flatten)]
    common: Common,
    /// A source, `lat,lng` (repeatable).
    #[arg(long = "source", value_parser = parse_point, required_unless_present = "request", allow_hyphen_values = true)]
    sources: Vec<GeoPoint>,
    /// A target, `lat,lng` (repeatable).
    #[arg(long = "target", value_parser = parse_point, required_unless_present = "request", allow_hyphen_values = true)]
    targets: Vec<GeoPoint>,
}

pub(crate) fn matrix(args: MatrixArgs) -> Result<(), Failure> {
    let engine = args.common.engine()?;
    let request = match args.common.request::<RouteMatrixRequest>()? {
        Some(request) => request,
        None => RouteMatrixRequest::new(args.sources, args.targets, args.common.profile),
    };
    print_json(&engine.matrix(request))
}

/// What is reachable from `--from` within `--minutes` (or a request).
#[derive(Debug, Args)]
pub(crate) struct IsochroneArgs {
    #[command(flatten)]
    common: Common,
    /// The origin, `lat,lng`.
    #[arg(long, value_parser = parse_point, required_unless_present = "request", allow_hyphen_values = true)]
    from: Option<GeoPoint>,
    /// A travel-time contour in minutes (repeatable).
    #[arg(long, required_unless_present = "request")]
    minutes: Vec<f64>,
}

pub(crate) fn isochrone(args: IsochroneArgs) -> Result<(), Failure> {
    let engine = args.common.engine()?;
    let request = match args.common.request::<RouteIsochroneRequest>()? {
        Some(request) => request,
        None => RouteIsochroneRequest::new(
            args.from.expect("required without --request"),
            args.common.profile,
            args.minutes
                .iter()
                .map(|minutes| RouteIsochroneContour {
                    contour: minutes * 60.0,
                    metric: RouteIsochroneMetric::TimeSeconds,
                })
                .collect(),
        ),
    };
    let result = engine
        .isochrone(request)
        .map_err(|error| Failure::new(error.to_string()))?;
    print_json(&result)
}

/// Match a GPS trace (a map-match request, `--request`) onto the roads.
#[derive(Debug, Args)]
pub(crate) struct MatchArgs {
    /// The routing package.
    #[arg(value_name = "routing.mrt")]
    package: PathBuf,
    /// The map-match request as JSON (a file, or `-` for stdin):
    /// `{"samples": [{"position": {"lat", "lng"}, "sampledAtMs"}], "profile"}`.
    #[arg(long, value_name = "FILE")]
    request: PathBuf,
}

pub(crate) fn map_match(args: MatchArgs) -> Result<(), Failure> {
    let engine = RoutingEngine::open(&args.package)
        .map_err(|error| Failure::new(format!("{}: {error}", args.package.display())))?;
    let text = read_text(&args.request)?;
    let request: MapMatchRequest = serde_json::from_str(&text)
        .map_err(|error| Failure::new(format!("{}: {error}", args.request.display())))?;
    let result = engine
        .map_match(request)
        .map_err(|error| Failure::new(error.to_string()))?;
    print_json(&result)
}

/// Where a point meets the roads the profile may use.
#[derive(Debug, Args)]
pub(crate) struct SnapArgs {
    #[command(flatten)]
    common: Common,
    /// The point, `lat,lng`.
    #[arg(long, value_parser = parse_point, required_unless_present = "request", allow_hyphen_values = true)]
    at: Option<GeoPoint>,
    /// How far from the point candidates may lie, in metres.
    #[arg(long)]
    max_distance_m: Option<u32>,
}

pub(crate) fn snap(args: SnapArgs) -> Result<(), Failure> {
    let engine = args.common.engine()?;
    let request = match args.common.request::<SnapRequest>()? {
        Some(request) => request,
        None => SnapRequest {
            max_distance_m: args.max_distance_m,
            ..SnapRequest::new(
                args.at.expect("required without --request"),
                args.common.profile,
            )
        },
    };
    print_json(&engine.snap(request))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn points_and_profiles_parse() {
        assert_eq!(
            parse_point("50.06, 19.94").unwrap(),
            GeoPoint::new(50.06, 19.94)
        );
        assert_eq!(
            parse_point("-33.9,18.4").unwrap(),
            GeoPoint::new(-33.9, 18.4)
        );
        assert!(parse_point("50.06").is_err());
        assert_eq!(parse_profile("on_foot").unwrap(), MobilityProfile::OnFoot);
        assert!(parse_profile("car").is_err());
    }
}

//! Routes through two or more points on a routing package.
//!
//! ```sh
//! cargo run --release -p wayfarer --example route -- \
//!   routing.mrt 43.7397 7.4275 43.7290 7.4160
//! ```
//!
//! More `<lat> <lng>` pairs add waypoints.

use wayfarer::{GeoPoint, MobilityProfile, RouteRequest, RoutingEngine};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let usage = "usage: route <routing.mrt> <lat> <lng> <lat> <lng> [<lat> <lng> ...]";
    let (Some(path), coordinates) = (args.get(1), args.get(2..).unwrap_or_default()) else {
        return Err(usage.into());
    };
    if coordinates.len() < 4 || coordinates.len() % 2 != 0 {
        return Err(usage.into());
    }
    let points = coordinates
        .chunks(2)
        .map(|pair| Ok(GeoPoint::new(pair[0].parse()?, pair[1].parse()?)))
        .collect::<Result<Vec<_>, Box<dyn std::error::Error>>>()?;
    let engine = RoutingEngine::open(path.as_ref())?;
    let route = engine.route(RouteRequest::through(points, MobilityProfile::Wheeled))?;
    println!(
        "{:.0} m, {:.0} s, {} legs, {} maneuvers",
        route.distance_m,
        route.duration_s,
        route.segments.len(),
        route.maneuvers.len()
    );
    Ok(())
}

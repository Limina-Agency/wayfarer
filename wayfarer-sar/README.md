# wayfarer-sar

Search and rescue planning over the offline routing engine. Every step is a
pure function of the routing package, the request and its explicit times, so
a plan replays the same on a command post and on a phone.

- **`predict_reach`**: from an approximate last known position (with an
  uncertainty radius) and the time since the person was last seen, the paths a
  fast, a typical and a slow person could have reached and how far that is in
  a straight line; a probability per edge (probability of area on the
  network, with rest of world); likely routes to the edge of that area and to
  named destinations, with arrival times; and the corridors those routes
  share. Zones the person cannot enter (`no_go`), tends to avoid (`avoid`) or
  is drawn to (`weight`) shape all of it. Optional lost-person distance
  statistics (supplied by the host) are mixed in.
- **`plan_teams`**: each search segment's probability, every team's arrival
  time at every segment (mixed profiles, pace factors, start offsets), a first
  assignment, and each team's territory within a shift.
- **`apply_coverage`**: team GPS tracks map-matched into covered paths, the
  probability of detection per segment, and the probabilities after an
  unsuccessful search.
- **`geojson`**: the results as GeoJSON for maps and GIS tools.

On foot, pace comes from the walking pace model (`wayfarer::pace`): the
built-in E4 calibration, or a team's own (`wayfarer pace calibrate` on its GPX
tracks) passed as `pace` in a reach request, on a team, or on a plan. Its P10,
P50 and P90 times its ratio to the engine's time on the reached paths give the
fast, typical and slow person. This release works on the road and trail network; reach into open terrain comes
with the terrain packages. The model is described in
`docs/design/sar-planning.md`.

```text
cargo run --release -p wayfarer-sar --example sar -- reach routing.mrt request.json result.json map.geojson
```

Version 0.x: the API may change between minor releases.

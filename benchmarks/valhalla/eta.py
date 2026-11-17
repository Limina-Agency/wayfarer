"""Where the driving ETA differs: road speeds by class, and junction time.

    python eta.py <valhalla.json> <ours.jsonl> <compare.csv>

Valhalla times our own route geometry (trace_route), and its edges' speeds
(trace_attributes) give its time on the road itself; the rest of its time is
spent at junctions (turns, signals, stops). Our route's edges give our time
on the road; the rest of our duration is ours at junctions. Prints, over the
drives both engines route, both splits and the mean speed per road class.
"""

import csv
import json
import sys
from collections import defaultdict

import valhalla

from composition import encode
import re


def snake_keys(value):
    """`value` with camelCase keys in snake_case: route dumps are camelCase
    since the 0.1.0 API, older ones snake_case."""
    if isinstance(value, dict):
        return {re.sub(r"(?<=[a-z0-9])([A-Z])", lambda m: "_" + m.group(1).lower(), k): snake_keys(v)
                for k, v in value.items()}
    if isinstance(value, list):
        return [snake_keys(v) for v in value]
    return value


def main():
    config, ours_path, compare_path = sys.argv[1:4]
    actor = valhalla.Actor(config)
    ours = {}
    for line in open(ours_path):
        row = json.loads(line)
        if "case" in row and not row["case"].endswith("_hinted_alt") and "error" not in row["response"]:
            ours[row["case"]] = snake_keys(row["response"])
    labels = [row["label"] for row in csv.DictReader(open(compare_path)) if row.get("overlap") and row["profile"] == "wheeled"]
    totals = defaultdict(float)
    ours_by_class = defaultdict(lambda: [0.0, 0.0])
    theirs_by_class = defaultdict(lambda: [0.0, 0.0])
    speed_sources = defaultdict(float)
    count = 0
    for label in labels:
        route = ours[label]
        shape = encode([[p["lat"], p["lng"]] for p in route["geometry"]])
        request = {"encoded_polyline": shape, "shape_match": "map_snap", "costing": "auto"}
        try:
            trip = json.loads(actor.trace_route(json.dumps(request)))["trip"]
            attributes = json.loads(actor.trace_attributes(json.dumps({
                **request,
                "filters": {"attributes": ["edge.speed", "edge.length", "edge.road_class", "edge.use"], "action": "include"},
            })))["edges"]
        except RuntimeError:
            continue
        count += 1
        edges = [edge for leg in route["legs"] for edge in leg["edges"]]
        our_edge_time = sum(edge["duration_s"] for edge in edges)
        totals["ours_total"] += route["duration_s"]
        totals["ours_edges"] += our_edge_time
        for edge in edges:
            entry = ours_by_class[edge["road_type"]]
            entry[0] += edge["distance_m"]
            entry[1] += edge["duration_s"]
            speed_sources[edge.get("speed_source") or "none"] += edge["distance_m"]
        their_edge_time = 0.0
        for edge in attributes:
            metres, kph = edge.get("length", 0.0) * 1000.0, edge.get("speed") or 0
            if kph <= 0:
                continue
            seconds = metres / (kph / 3.6)
            their_edge_time += seconds
            label_ = edge.get("use") if edge.get("use") not in ("road", "ramp", "turn_channel") else edge.get("road_class")
            entry = theirs_by_class[label_]
            entry[0] += metres
            entry[1] += seconds
        totals["valhalla_total"] += trip["summary"]["time"]
        totals["valhalla_edges"] += their_edge_time
    print(f"{count} drives timed by Valhalla on our geometry")
    for engine in ("ours", "valhalla"):
        total, on_road = totals[f"{engine}_total"], totals[f"{engine}_edges"]
        print(f"  {engine:8} total {total / 3600:7.2f} h = on the road {on_road / 3600:7.2f} h + at junctions {(total - on_road) / 3600:6.2f} h ({100 * (total - on_road) / total:4.1f}%)")
    print("mean speed by road class (km/h; ours by our class, Valhalla's by its own):")
    for name, table in (("ours", ours_by_class), ("valhalla", theirs_by_class)):
        for road_class, (metres, seconds) in sorted(table.items(), key=lambda item: -item[1][0]):
            print(f"  {name:8} {road_class:14} {metres / 1000:8.1f} km  {3.6 * metres / seconds:6.1f} km/h")
    print("our speed sources (share of length):")
    total = sum(speed_sources.values())
    for source, metres in sorted(speed_sources.items(), key=lambda item: -item[1]):
        print(f"  {source:24} {100 * metres / total:5.1f}%")


if __name__ == "__main__":
    main()

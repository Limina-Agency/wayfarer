"""Our driving speeds against Valhalla's on the same roads.

    python speeds.py <valhalla.json> <ours.jsonl> <compare.csv>

Valhalla's trace_attributes labels our own route geometry with its edges'
speeds; both are sampled every 50 m by distance along the route (the path is
the same, so positions line up), and grouped by our road type and speed
source. Prints the median ratio of our speed to Valhalla's per group, and
for posted speeds per posted value.
"""

import bisect
import csv
import json
import statistics
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

STEP_M = 50.0


def main():
    config, ours_path, compare_path = sys.argv[1:4]
    actor = valhalla.Actor(config)
    ours = {}
    for line in open(ours_path):
        row = json.loads(line)
        if "case" in row and not row["case"].endswith("_hinted_alt") and "error" not in row["response"]:
            ours[row["case"]] = snake_keys(row["response"])
    labels = [row["label"] for row in csv.DictReader(open(compare_path)) if row.get("overlap") and row["profile"] == "wheeled"]
    ratios = defaultdict(list)
    by_posted = defaultdict(list)
    for label in labels:
        route = ours[label]
        request = {
            "encoded_polyline": encode([[p["lat"], p["lng"]] for p in route["geometry"]]),
            "shape_match": "map_snap",
            "costing": "auto",
            "filters": {"attributes": ["edge.speed", "edge.length"], "action": "include"},
        }
        try:
            theirs = json.loads(actor.trace_attributes(json.dumps(request)))["edges"]
        except RuntimeError:
            continue
        their_ends, their_speeds, at = [], [], 0.0
        for edge in theirs:
            at += edge.get("length", 0.0) * 1000.0
            their_ends.append(at)
            their_speeds.append(edge.get("speed") or 0)
        edges = [edge for leg in route["legs"] for edge in leg["edges"]]
        at = 0.0
        for edge in edges:
            start, at = at, at + edge["distance_m"]
            if edge["duration_s"] <= 0 or edge["distance_m"] < STEP_M:
                continue
            ours_kph = 3.6 * edge["distance_m"] / edge["duration_s"]
            position = start + STEP_M / 2
            while position < at:
                index = bisect.bisect_left(their_ends, position)
                if index < len(their_speeds) and their_speeds[index] > 0:
                    key = (edge["road_type"], edge.get("speed_source") or "none")
                    ratios[key].append(ours_kph / their_speeds[index])
                    if edge.get("speed_source") == "posted" and edge.get("maxspeed_kph"):
                        by_posted[(edge["road_type"], round(edge["maxspeed_kph"]))].append(their_speeds[index])
                position += STEP_M
    print("our speed / Valhalla's, by our road type and speed source (samples every 50 m):")
    for key, values in sorted(ratios.items(), key=lambda item: -len(item[1])):
        if len(values) >= 20:
            print(f"  {key[0]:12} {key[1]:10} n={len(values):6}  median {statistics.median(values):.3f}"
                  f"  p25 {statistics.quantiles(values, n=4)[0]:.3f}  p75 {statistics.quantiles(values, n=4)[2]:.3f}")
    print("Valhalla's speed where ours is a posted limit (road type, posted km/h -> Valhalla median):")
    for key, values in sorted(by_posted.items(), key=lambda item: -len(item[1])):
        if len(values) >= 50:
            print(f"  {key[0]:12} {key[1]:4} km/h  n={len(values):6}  valhalla median {statistics.median(values):5.1f} km/h")


if __name__ == "__main__":
    main()

"""Endpoint snapping: how far each engine snaps from the requested points.

    python snapping.py <cases.json> <ours.jsonl> <valhalla.jsonl>

Valhalla's snapped point is the first or last point of its shape; ours is the
correlated location's own distance. Prints, per profile, how often each
engine snaps more than 20 m farther than the other, the medians, and the
worst cases of ours snapping farther.
"""

import json
import math
import statistics
import sys
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
    cases_path, ours_path, valhalla_path = sys.argv[1:4]
    cases = {case["label"]: case for case in json.load(open(cases_path))}
    ours = {}
    for line in open(ours_path):
        row = json.loads(line)
        if "case" in row and not row["case"].endswith("_hinted_alt") and "error" not in row["response"]:
            ours[row["case"]] = snake_keys(row["response"])
    theirs = {row["label"]: row for row in map(json.loads, open(valhalla_path)) if row["status"] == "ok"}
    for profile in ("wheeled", "on_foot"):
        ends = []
        for label in sorted(set(ours) & set(theirs)):
            case = cases[label]
            if case["profile"] != profile:
                continue
            kx = 111_320.0 * math.cos(math.radians(case["origin"][1]))
            for end, index in (("origin", 0), ("destination", -1)):
                lat, lng = theirs[label]["shape"][index]
                requested = case[end]
                valhalla_m = math.hypot((lng - requested[0]) * kx, (lat - requested[1]) * 110_574.0)
                ends.append((label, end, ours[label][end]["distance_m"], valhalla_m))
        if not ends:
            continue
        farther = [e for e in ends if e[2] > e[3] + 20]
        print(
            f"{profile}: {len(ends)} endpoints; ours >20 m farther in {len(farther)},"
            f" valhalla >20 m farther in {sum(e[3] > e[2] + 20 for e in ends)};"
            f" median ours {statistics.median(e[2] for e in ends):.1f} m, valhalla {statistics.median(e[3] for e in ends):.1f} m"
        )
        for label, end, mine, other in sorted(farther, key=lambda e: e[3] - e[2])[:8]:
            print(f"  {label:34} {end:11} ours {mine:6.1f} m  valhalla {other:6.1f} m")


if __name__ == "__main__":
    main()

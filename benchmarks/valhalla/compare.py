"""Compares Wayfarer routes with Valhalla's on the same cases.

    python compare.py <ours.jsonl> <valhalla.jsonl> <out.csv>

ours.jsonl is route_response_dump's output on cases.json (its `_hinted_alt`
copies are ignored). Per case: both statuses, length and time ratios
(Wayfarer / Valhalla), path overlap (the share of each route within
OVERLAP_M of the other; `overlap` is the smaller of the two), and how far
apart the two engines snapped each endpoint. Prints a summary and the cases
that disagree most.
"""

import csv
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

OVERLAP_M = 25.0
SAMPLE_M = 10.0
CELL_M = 50.0


class Plane:
    """Equirectangular metres around a reference latitude."""

    def __init__(self, lat):
        self.kx = 111_320.0 * math.cos(math.radians(lat))
        self.ky = 110_574.0

    def xy(self, point):
        return (point[1] * self.kx, point[0] * self.ky)


def segment_distance(p, a, b):
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def covered_share(route, other):
    """Share of `route`'s length within OVERLAP_M of `other`."""
    if len(route) < 2 or len(other) < 2:
        return 0.0
    grid = {}
    for a, b in zip(other, other[1:]):
        for gx in range(int(min(a[0], b[0]) // CELL_M) - 1, int(max(a[0], b[0]) // CELL_M) + 2):
            for gy in range(int(min(a[1], b[1]) // CELL_M) - 1, int(max(a[1], b[1]) // CELL_M) + 2):
                grid.setdefault((gx, gy), []).append((a, b))
    total = near = 0.0
    for a, b in zip(route, route[1:]):
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(1, int(length // SAMPLE_M))
        for step in range(steps):
            t = (step + 0.5) / steps
            p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            segments = grid.get((int(p[0] // CELL_M), int(p[1] // CELL_M)), ())
            if any(segment_distance(p, s, e) <= OVERLAP_M for s, e in segments):
                near += length / steps
            total += length / steps
    return near / total if total else 0.0


def main():
    ours_path, valhalla_path, out_path = sys.argv[1:4]
    ours = {}
    for line in open(ours_path):
        row = json.loads(line)
        if "case" in row and not row["case"].endswith("_hinted_alt"):
            ours[row["case"]] = snake_keys(row["response"])
    theirs = {row["label"]: row for row in map(json.loads, open(valhalla_path))}
    rows = []
    for label, valhalla in theirs.items():
        mine = ours.get(label)
        if mine is None:
            continue
        row = {"label": label, "profile": valhalla["profile"]}
        row["ours"] = "error:" + mine["error"].split("{")[0].split("(")[0] if "error" in mine else "ok"
        row["valhalla"] = valhalla["status"]
        if row["ours"] == "ok" and valhalla["status"] == "ok":
            shape_ours = [[p["lat"], p["lng"]] for p in mine["geometry"]]
            shape_theirs = valhalla["shape"]
            plane = Plane(shape_theirs[0][0])
            a = [plane.xy(p) for p in shape_ours]
            b = [plane.xy(p) for p in shape_theirs]
            ours_in_theirs = covered_share(a, b)
            theirs_in_ours = covered_share(b, a)
            row.update(
                length_ours_m=round(mine["distance_m"], 1),
                length_valhalla_m=round(valhalla["length_m"], 1),
                time_ours_s=round(mine["duration_s"], 1),
                time_valhalla_s=round(valhalla["time_s"], 1),
                length_ratio=round(mine["distance_m"] / max(valhalla["length_m"], 1.0), 3),
                time_ratio=round(mine["duration_s"] / max(valhalla["time_s"], 1.0), 3),
                overlap=round(min(ours_in_theirs, theirs_in_ours), 3),
                ours_in_valhalla=round(ours_in_theirs, 3),
                valhalla_in_ours=round(theirs_in_ours, 3),
                origin_snap_gap_m=round(math.dist(a[0], b[0]), 1),
                destination_snap_gap_m=round(math.dist(a[-1], b[-1]), 1),
                maneuvers_ours=len(mine.get("maneuvers", [])),
                maneuvers_valhalla=valhalla["maneuvers"],
            )
        rows.append(row)
    fields = sorted({key for row in rows for key in row}, key=lambda k: (k != "label", k != "profile", k))
    with open(out_path, "w", newline="") as out:
        writer = csv.DictWriter(out, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    both = [row for row in rows if "overlap" in row]
    print(f"{len(rows)} cases: {len(both)} routed by both")
    for status in sorted({(r['ours'], r['valhalla']) for r in rows if (r['ours'], r['valhalla']) != ('ok', 'ok')}):
        labels = [r["label"] for r in rows if (r["ours"], r["valhalla"]) == status]
        print(f"  ours {status[0]}, valhalla {status[1]}: {len(labels)} ({', '.join(labels[:6])})")
    for profile in ("wheeled", "on_foot"):
        subset = [row for row in both if row["profile"] == profile]
        if not subset:
            continue
        quant = lambda key, q: statistics.quantiles([r[key] for r in subset], n=20)[q]
        print(
            f"{profile:8} n={len(subset):3}  length ratio median {statistics.median(r['length_ratio'] for r in subset):.3f}"
            f" (p5 {quant('length_ratio', 0):.3f}, p95 {quant('length_ratio', 18):.3f})"
            f"  time ratio median {statistics.median(r['time_ratio'] for r in subset):.3f}"
            f" (p5 {quant('time_ratio', 0):.3f}, p95 {quant('time_ratio', 18):.3f})"
            f"  overlap median {statistics.median(r['overlap'] for r in subset):.3f},"
            f" <0.9: {sum(r['overlap'] < 0.9 for r in subset)}, <0.5: {sum(r['overlap'] < 0.5 for r in subset)}"
            f"  snap gap >30 m: {sum(max(r['origin_snap_gap_m'], r['destination_snap_gap_m']) > 30 for r in subset)}"
        )
    print("lowest overlap:")
    for row in sorted(both, key=lambda r: r["overlap"])[:15]:
        print(
            f"  {row['label']:40} {row['profile']:8} overlap {row['overlap']:.2f}"
            f"  length {row['length_ours_m']:8.0f} / {row['length_valhalla_m']:8.0f}"
            f"  time {row['time_ours_s']:6.0f} / {row['time_valhalla_s']:6.0f}"
            f"  snap gaps {row['origin_snap_gap_m']:.0f} / {row['destination_snap_gap_m']:.0f} m"
        )


if __name__ == "__main__":
    main()

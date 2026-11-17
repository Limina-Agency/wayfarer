"""Road-type composition of both engines' routes, labelled by Valhalla.

    python composition.py <valhalla.json> <ours.jsonl> <valhalla.jsonl> <compare.csv> <out.json>

Valhalla's trace_attributes (map_snap) labels each route's shape with its
edges' road class and use, so both routes are classified by one labeller. The
output keeps, per case, the metres of each label on our route and on
Valhalla's; the summary prints, per profile, the labels on which the two
engines spend the most different shares of their length, over all cases and
over those whose paths overlap least.
"""

import csv
import json
import sys
from collections import defaultdict

import valhalla

from valhalla_routes import COSTING


def encode(points, precision=6):
    factor, out, last = 10**precision, [], (0, 0)
    for lat, lng in points:
        value = (round(lat * factor), round(lng * factor))
        for delta in (value[0] - last[0], value[1] - last[1]):
            delta = ~(delta << 1) if delta < 0 else delta << 1
            while delta >= 0x20:
                out.append(chr((0x20 | (delta & 0x1F)) + 63))
                delta >>= 5
            out.append(chr(delta + 63))
        last = value
    return "".join(out)


def composition(actor, shape, profile):
    request = {
        "encoded_polyline": encode(shape),
        "shape_match": "map_snap",
        "costing": COSTING[profile],
        "filters": {"attributes": ["edge.road_class", "edge.use", "edge.length"], "action": "include"},
    }
    try:
        edges = json.loads(actor.trace_attributes(json.dumps(request)))["edges"]
    except (RuntimeError, KeyError):
        return None
    metres = defaultdict(float)
    for edge in edges:
        use = edge.get("use", "?")
        label = use if use not in ("road", "ramp", "turn_channel") else edge.get("road_class", "?")
        metres[label] += edge.get("length", 0.0) * 1000.0
    return dict(metres)


def main():
    config, ours_path, valhalla_path, compare_path, out_path = sys.argv[1:6]
    actor = valhalla.Actor(config)
    ours = {}
    for line in open(ours_path):
        row = json.loads(line)
        if "case" in row and "error" not in row["response"]:
            ours[row["case"]] = [[p["lat"], p["lng"]] for p in row["response"]["geometry"]]
    theirs = {row["label"]: row for row in map(json.loads, open(valhalla_path))}
    compared = {row["label"]: row for row in csv.DictReader(open(compare_path)) if row.get("overlap")}
    result = {}
    for label, row in compared.items():
        mine = composition(actor, ours[label], row["profile"])
        other = composition(actor, theirs[label]["shape"], row["profile"])
        if mine is not None and other is not None:
            result[label] = {"profile": row["profile"], "overlap": float(row["overlap"]), "ours": mine, "valhalla": other}
    json.dump(result, open(out_path, "w"), indent=1)

    for profile in ("wheeled", "on_foot"):
        for title, keep in (("all cases", lambda r: True), ("overlap < 0.5", lambda r: r["overlap"] < 0.5)):
            cases = [r for r in result.values() if r["profile"] == profile and keep(r)]
            if not cases:
                continue
            totals = {"ours": defaultdict(float), "valhalla": defaultdict(float)}
            for case in cases:
                for engine in totals:
                    for label, metres in case[engine].items():
                        totals[engine][label] += metres
            sums = {engine: sum(values.values()) or 1.0 for engine, values in totals.items()}
            labels = set(totals["ours"]) | set(totals["valhalla"])
            share = lambda engine, label: 100.0 * totals[engine][label] / sums[engine]
            print(f"{profile} — {title} (n={len(cases)}): share of route length, ours vs valhalla")
            for label in sorted(labels, key=lambda l: -abs(share("ours", l) - share("valhalla", l)))[:10]:
                print(f"  {label:16} {share('ours', label):6.1f}% vs {share('valhalla', label):6.1f}%"
                      f"  ({share('ours', label) - share('valhalla', label):+.1f})")


if __name__ == "__main__":
    main()

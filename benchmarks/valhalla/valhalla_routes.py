"""Routes every case with Valhalla.

    python valhalla_routes.py <valhalla.json> <cases.json> <out.jsonl>

One JSON line per case: label, profile, status, length_m, time_s, shape
([[lat, lng], ...]) and maneuver count, or the error.
"""

import json
import sys
import time

import valhalla

COSTING = {"wheeled": "auto", "on_foot": "pedestrian"}


def decode(encoded, precision=6):
    points, index, lat, lng = [], 0, 0, 0
    factor = 10.0**precision
    while index < len(encoded):
        for axis in range(2):
            shift = result = 0
            while True:
                byte = ord(encoded[index]) - 63
                index += 1
                result |= (byte & 0x1F) << shift
                shift += 5
                if byte < 0x20:
                    break
            delta = ~(result >> 1) if result & 1 else result >> 1
            if axis == 0:
                lat += delta
            else:
                lng += delta
        points.append([lat / factor, lng / factor])
    return points


def main():
    config, cases_path, out_path = sys.argv[1:4]
    actor = valhalla.Actor(config)
    with open(out_path, "w") as out:
        for case in json.load(open(cases_path)):
            request = {
                "locations": [
                    {"lat": case["origin"][1], "lon": case["origin"][0], "type": "break"},
                    {"lat": case["destination"][1], "lon": case["destination"][0], "type": "break"},
                ],
                "costing": COSTING[case["profile"]],
                "units": "kilometers",
            }
            row = {"label": case["label"], "profile": case["profile"]}
            started = time.perf_counter()
            try:
                trip = json.loads(actor.route(json.dumps(request)))["trip"]
                shape = [point for leg in trip["legs"] for point in decode(leg["shape"])]
                row.update(
                    status="ok",
                    length_m=trip["summary"]["length"] * 1000.0,
                    time_s=trip["summary"]["time"],
                    shape=shape,
                    maneuvers=sum(len(leg["maneuvers"]) for leg in trip["legs"]),
                )
            except RuntimeError as error:
                row.update(status="error", error=str(error)[:200])
            row["ms"] = (time.perf_counter() - started) * 1000.0
            out.write(json.dumps(row) + "\n")


if __name__ == "__main__":
    main()

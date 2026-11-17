"""Route pairs for the Valhalla comparison: the corpus plus seeded random pairs.

    python cases.py <valhalla.json> <corpus.json> <out.json> [random_pairs=150]

Random endpoints are drawn in the corpus bounding box and kept only where
Valhalla's locate finds a road for the profile within 30 m, so both engines
start and end on a road. The output is a corpus-format JSON array
(label, origin [lng, lat], destination, profile), which route_response_dump
reads as its corpus.
"""

import json
import math
import random
import sys

import valhalla

SNAP_LIMIT_M = 30.0
COSTING = {"wheeled": "auto", "on_foot": "pedestrian"}


def snaps(actor, lng, lat, profile):
    request = {"locations": [{"lat": lat, "lon": lng}], "costing": COSTING[profile], "verbose": True}
    try:
        located = json.loads(actor.locate(json.dumps(request)))
    except RuntimeError:
        return False
    edges = (located[0] or {}).get("edges") or []
    for edge in edges:
        dlat = (edge["correlated_lat"] - lat) * 111_320.0
        dlng = (edge["correlated_lon"] - lng) * 111_320.0 * math.cos(math.radians(lat))
        if math.hypot(dlat, dlng) <= SNAP_LIMIT_M:
            return True
    return False


def main():
    config, corpus_path, out_path = sys.argv[1:4]
    random_pairs = int(sys.argv[4]) if len(sys.argv) > 4 else 150
    corpus = json.load(open(corpus_path))
    cases = [
        {key: case[key] for key in ("label", "origin", "destination", "profile")}
        for case in corpus
    ]
    lngs = [p[0] for case in corpus for p in (case["origin"], case["destination"])]
    lats = [p[1] for case in corpus for p in (case["origin"], case["destination"])]
    actor = valhalla.Actor(config)
    rng = random.Random(20261029)

    def point(profile):
        while True:
            lng = rng.uniform(min(lngs), max(lngs))
            lat = rng.uniform(min(lats), max(lats))
            if snaps(actor, lng, lat, profile):
                return [round(lng, 6), round(lat, 6)]

    for index in range(random_pairs):
        profile = "on_foot" if index % 3 == 2 else "wheeled"
        origin = point(profile)
        if profile == "on_foot":
            # Walks within ~8 km: a random offset from the origin.
            while True:
                destination = [
                    round(origin[0] + rng.uniform(-0.08, 0.08), 6),
                    round(origin[1] + rng.uniform(-0.05, 0.05), 6),
                ]
                if snaps(actor, destination[0], destination[1], profile):
                    break
        else:
            destination = point(profile)
        cases.append(
            {"label": f"vrandom_{index:03}", "origin": origin, "destination": destination, "profile": profile}
        )
    json.dump(cases, open(out_path, "w"), indent=1)
    print(f"{len(cases)} cases ({len(corpus)} corpus, {random_pairs} random)")


if __name__ == "__main__":
    main()

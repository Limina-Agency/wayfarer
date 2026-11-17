"""Writes bench.sh's report: graph compilation and route quality, both engines.

    python report.py <work_dir> --region R --pbf P --threads N --wayfarer SHA --valhalla V

Reads <work_dir>/compile (measure.py records, median over runs; sizes.json),
<work_dir>/wayfarer/info.json and <work_dir>/quality (compare.csv and the
comparison scripts' printed summaries), and writes <work_dir>/report.md and
<work_dir>/summary.json. Ratios are Wayfarer / Valhalla.
"""

import argparse
import csv
import datetime
import glob
import json
import os
import platform
import re
import statistics

VALHALLA_STEPS = ("valhalla-admins", "valhalla-tiles", "valhalla-extract")


def cpu_model():
    try:
        for line in open("/proc/cpuinfo"):
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor() or platform.machine()


def mib(size):
    return f"{size / 2**20:,.1f} MiB"


def compile_steps(directory):
    records = {}
    for path in glob.glob(os.path.join(directory, "*-[0-9]*.json")):
        record = json.load(open(path))
        records.setdefault(record["label"], []).append(record)
    steps = {}
    for label, runs in records.items():
        if any(run["exit"] != 0 for run in runs):
            raise SystemExit(f"{label} failed in {directory}; see the logs")
        steps[label] = {
            "runs": len(runs),
            "wall_s": statistics.median(r["wall_s"] for r in runs),
            "cpu_s": statistics.median(r["user_s"] + r["sys_s"] for r in runs),
            "max_rss_mib": max(r["max_rss_mib"] for r in runs),
        }
    return steps


def quantile(values, q):
    values = sorted(values)
    if len(values) == 1:
        return values[0]
    return statistics.quantiles(values, n=20)[q]


def quality(directory):
    rows = list(csv.DictReader(open(os.path.join(directory, "compare.csv"))))
    outcome = lambda r: ("ok" if r["ours"] == "ok" else "fail", "ok" if r["valhalla"] == "ok" else "fail")
    statuses = {
        "both_ok": sum(outcome(r) == ("ok", "ok") for r in rows),
        "only_ours_failed": [r["label"] for r in rows if outcome(r) == ("fail", "ok")],
        "only_valhalla_failed": [r["label"] for r in rows if outcome(r) == ("ok", "fail")],
        "both_failed": [r["label"] for r in rows if outcome(r) == ("fail", "fail")],
    }
    profiles = {}
    for profile in ("wheeled", "on_foot"):
        subset = [r for r in rows if r["profile"] == profile and r.get("overlap")]
        if not subset:
            continue
        number = lambda key: [float(r[key]) for r in subset]
        snap = [max(float(r["origin_snap_gap_m"]), float(r["destination_snap_gap_m"])) for r in subset]
        profiles[profile] = {
            "cases": sum(r["profile"] == profile for r in rows),
            "routed_by_both": len(subset),
            "length_ratio": {k: round(quantile(number("length_ratio"), q), 3) for k, q in (("p5", 0), ("median", 9), ("p95", 18))},
            "time_ratio": {k: round(quantile(number("time_ratio"), q), 3) for k, q in (("p5", 0), ("median", 9), ("p95", 18))},
            "overlap_median": round(statistics.median(number("overlap")), 3),
            "overlap_mean": round(statistics.fmean(number("overlap")), 3),
            "overlap_below_0_9": sum(v < 0.9 for v in number("overlap")),
            "overlap_below_0_5": sum(v < 0.5 for v in number("overlap")),
            "snap_gap_over_30m": sum(v > 30 for v in snap),
        }
    latency = {}
    match = re.search(r"median request ([0-9.]+) ms, p95 ([0-9.]+) ms", open(os.path.join(directory, "ours.log")).read())
    if match:
        latency["wayfarer"] = {"median_ms": float(match[1]), "p95_ms": float(match[2])}
    valhalla_ms = [
        row["ms"] for row in map(json.loads, open(os.path.join(directory, "valhalla.jsonl"))) if row["status"] == "ok"
    ]
    if valhalla_ms:
        latency["valhalla"] = {
            "median_ms": round(statistics.median(valhalla_ms), 2),
            "p95_ms": round(quantile(valhalla_ms, 18), 2),
        }
    return {"cases": len(rows), "statuses": statuses, "profiles": profiles, "latency": latency}


def section(directory, name):
    path = os.path.join(directory, name)
    return open(path).read().rstrip() if os.path.exists(path) else "(not run)"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("work")
    for flag in ("region", "pbf", "threads", "wayfarer", "valhalla"):
        parser.add_argument(f"--{flag}", required=True)
    args = parser.parse_args()
    work = args.work

    steps = compile_steps(os.path.join(work, "compile"))
    sizes = json.load(open(os.path.join(work, "compile", "sizes.json")))
    info = json.load(open(os.path.join(work, "wayfarer", "info.json")))
    wayfarer = steps["wayfarer"]
    valhalla = {
        "wall_s": sum(steps[s]["wall_s"] for s in VALHALLA_STEPS),
        "cpu_s": sum(steps[s]["cpu_s"] for s in VALHALLA_STEPS),
        "max_rss_mib": max(steps[s]["max_rss_mib"] for s in VALHALLA_STEPS),
        "bytes": sizes["valhalla_extract_bytes"] + sizes["valhalla_admins_bytes"],
    }
    ratios = {
        "wall": round(wayfarer["wall_s"] / valhalla["wall_s"], 3),
        "cpu": round(wayfarer["cpu_s"] / valhalla["cpu_s"], 3),
        "peak_memory": round(wayfarer["max_rss_mib"] / valhalla["max_rss_mib"], 3),
        "size": round(sizes["wayfarer_package_bytes"] / valhalla["bytes"], 3),
    }
    routes = quality(os.path.join(work, "quality"))
    setup = {
        "region": args.region,
        "extract": os.path.basename(args.pbf),
        "extract_bytes": os.path.getsize(args.pbf),
        "threads": int(args.threads),
        "cpu": cpu_model(),
        "wayfarer": args.wayfarer,
        "valhalla": f"pyvalhalla {args.valhalla}",
        "date": datetime.date.today().isoformat(),
    }
    summary = {
        "setup": setup,
        "compile": {"steps": steps, "valhalla_total": valhalla, "sizes": sizes, "ratios": ratios,
                    "wayfarer_graph": {k: info[k] for k in ("graphNodeCount", "directedEdgeCount") if k in info}},
        "quality": routes,
    }
    json.dump(summary, open(os.path.join(work, "summary.json"), "w"), indent=1)

    runs = wayfarer["runs"]
    lines = [
        f"# Wayfarer vs Valhalla: {args.region}",
        "",
        f"{setup['extract']} ({mib(setup['extract_bytes'])}), {setup['threads']} threads on {setup['cpu']},"
        f" {setup['date']}. Wayfarer {setup['wayfarer']}, {setup['valhalla']}. Ratios are Wayfarer / Valhalla.",
        "",
        "## Graph compilation",
        "",
        f"Median of {runs} run{'s' if runs > 1 else ''}; peak memory is the largest resident set of each step.",
        "",
        "| Step | Wall | CPU | Peak memory | Output |",
        "|---|---:|---:|---:|---|",
        f"| Wayfarer `compile` | {wayfarer['wall_s']:.1f} s | {wayfarer['cpu_s']:.1f} s | {wayfarer['max_rss_mib']:,.0f} MiB"
        f" | routing.mrt {mib(sizes['wayfarer_package_bytes'])} |",
        f"| Valhalla admins | {steps['valhalla-admins']['wall_s']:.1f} s | {steps['valhalla-admins']['cpu_s']:.1f} s"
        f" | {steps['valhalla-admins']['max_rss_mib']:,.0f} MiB | admins.sqlite {mib(sizes['valhalla_admins_bytes'])} |",
        f"| Valhalla tiles | {steps['valhalla-tiles']['wall_s']:.1f} s | {steps['valhalla-tiles']['cpu_s']:.1f} s"
        f" | {steps['valhalla-tiles']['max_rss_mib']:,.0f} MiB"
        f" | {sizes['valhalla_tile_count']} tiles, {mib(sizes['valhalla_tiles_bytes'])} |",
        f"| Valhalla tar extract | {steps['valhalla-extract']['wall_s']:.1f} s | {steps['valhalla-extract']['cpu_s']:.1f} s"
        f" | {steps['valhalla-extract']['max_rss_mib']:,.0f} MiB | tiles.tar {mib(sizes['valhalla_extract_bytes'])} |",
        f"| **Valhalla total** | **{valhalla['wall_s']:.1f} s** | **{valhalla['cpu_s']:.1f} s**"
        f" | **{valhalla['max_rss_mib']:,.0f} MiB** | **{mib(valhalla['bytes'])}** (tar + admins) |",
        f"| **Ratio** | **{ratios['wall']:.2f}** | **{ratios['cpu']:.2f}** | **{ratios['peak_memory']:.2f}**"
        f" | **{ratios['size']:.2f}** |",
        "",
    ]
    graph = summary["compile"]["wayfarer_graph"]
    if graph:
        lines += [f"Wayfarer graph: {graph.get('graphNodeCount', 0):,} nodes, {graph.get('directedEdgeCount', 0):,} directed edges.", ""]

    statuses = routes["statuses"]
    listed = lambda labels: f"{len(labels)}" + (f" ({', '.join(labels[:8])}{', …' if len(labels) > 8 else ''})" if labels else "")
    lines += [
        "## Route quality",
        "",
        f"{routes['cases']} cases: both routed {statuses['both_ok']}; only Wayfarer failed {listed(statuses['only_ours_failed'])};"
        f" only Valhalla failed {listed(statuses['only_valhalla_failed'])}; both failed {listed(statuses['both_failed'])}.",
        "",
        "| Profile | Routed by both | Length ratio p5 / median / p95 | Time ratio p5 / median / p95 | Overlap median | < 0.9 | < 0.5 | Snap gap > 30 m |",
        "|---|---:|---|---|---:|---:|---:|---:|",
    ]
    for profile, p in routes["profiles"].items():
        lr, tr = p["length_ratio"], p["time_ratio"]
        lines.append(
            f"| {profile} | {p['routed_by_both']} of {p['cases']} | {lr['p5']:.3f} / {lr['median']:.3f} / {lr['p95']:.3f}"
            f" | {tr['p5']:.3f} / {tr['median']:.3f} / {tr['p95']:.3f} | {p['overlap_median']:.3f}"
            f" | {p['overlap_below_0_9']} | {p['overlap_below_0_5']} | {p['snap_gap_over_30m']} |"
        )
    lines += [
        "",
        "Overlap is the smaller of the two shares of each route within 25 m of the other; the snap gap is how far"
        " apart the two engines' routes start or end.",
        "",
    ]
    latency = routes["latency"]
    if len(latency) == 2:
        lines += [
            f"Request latency (single thread, in process, warm): Wayfarer median {latency['wayfarer']['median_ms']:.2f} ms,"
            f" p95 {latency['wayfarer']['p95_ms']:.2f} ms; Valhalla median {latency['valhalla']['median_ms']:.2f} ms,"
            f" p95 {latency['valhalla']['p95_ms']:.2f} ms. Indicative only: Wayfarer's figure is"
            " `route_response_dump`'s (it also routes `_hinted_alt` copies with alternatives), Valhalla's includes"
            " pyvalhalla's JSON round trip.",
            "",
        ]
    quality_dir = os.path.join(work, "quality")
    for title, name in (
        ("Disagreements", "compare.txt"),
        ("Road-type composition", "composition.txt"),
        ("Endpoint snapping", "snapping.txt"),
        ("Driving time: road and junction time", "eta.txt"),
        ("Driving speeds on the same roads", "speeds.txt"),
    ):
        lines += [f"### {title}", "", "```", section(quality_dir, name), "```", ""]
    open(os.path.join(work, "report.md"), "w").write("\n".join(lines))


if __name__ == "__main__":
    main()

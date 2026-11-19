"""Summarize a nightly gate run (nightly.sh) as Markdown and JSON.

usage: nightly_summary.py --region R --corpus C --gate GATE_TXT --gate-status N --commit SHA
                          [--baseline-commit SHA] [--pbf PATH] [--phase PHASE_TXT]
                          [--timing TIMING_TXT | --timing-skipped NOTE_TXT]
                          --json OUT_JSON

Markdown goes to stdout. Latency classes come from the corpus: profile
(`wheeled` drives, `on_foot` walks) and, for drives, the distance bucket. Each
class's p50 and p95 are taken over its cases' median request times from the
phase bench (nearest rank, so with six cases p95 is the slowest case).
"""
import argparse
import hashlib
import json
import math
import re
import sys
from collections import defaultdict


def percentile(values, fraction):
    ordered = sorted(values)
    rank = max(1, math.ceil(fraction * len(ordered)))
    return ordered[rank - 1]


def parse_phase(path):
    # "<label> correlation min 0.120 med 0.130 | request min 1.234 med 1.300"
    pattern = re.compile(
        r'^(\S+)\s+correlation min\s+([\d.]+) med\s+([\d.]+) \| request min\s+([\d.]+) med\s+([\d.]+)')
    cases = {}
    for line in open(path, encoding='utf-8'):
        match = pattern.match(line)
        if match:
            label, _, correlation_med, request_min, request_med = match.groups()
            cases[label] = {'correlationMedMs': float(correlation_med),
                            'requestMinMs': float(request_min),
                            'requestMedMs': float(request_med)}
    return cases


def classes(corpus, phase):
    groups = defaultdict(list)
    for entry in corpus:
        timing = phase.get(entry['label'])
        if timing is None:
            continue
        kind = 'walk' if entry.get('profile') in ('on_foot', 'walking') else 'drive'
        groups[kind].append(timing['requestMedMs'])
        if kind == 'drive':
            groups[f"drive/{entry.get('bucket', 'other')}"].append(timing['requestMedMs'])
    order = ['drive', 'walk'] + sorted(k for k in groups if k.startswith('drive/'))
    return {name: {'cases': len(groups[name]),
                   'p50Ms': percentile(groups[name], 0.5),
                   'p95Ms': percentile(groups[name], 0.95),
                   'maxMs': max(groups[name])}
            for name in order if groups.get(name)}


def parse_gate(path):
    text = open(path, encoding='utf-8', errors='replace').read()
    gate = {}
    if match := re.search(r'compiled \S+ in (\d+) s', text):
        gate['compileS'] = int(match.group(1))
    if match := re.search(r'routes: (\d+) cases, median request ([\d.]+) ms, p95 ([\d.]+) ms', text):
        gate['routes'] = {'cases': int(match.group(1)), 'medianMs': float(match.group(2)),
                          'p95Ms': float(match.group(3))}
    if match := re.search(r'routed in (\d+) s', text):
        gate['routeS'] = int(match.group(1))
    return gate, text


def parse_timing(path):
    # "<run>   <a|b>      <median> <p95> <correlation> | <cold_first> <cold_warm>"
    rows = defaultdict(list)
    for line in open(path, encoding='utf-8'):
        fields = line.replace('|', ' ').split()
        if len(fields) >= 2 and fields[0].isdigit() and fields[1] in ('a', 'b'):
            numbers = [float(value) for value in fields[2:]]
            rows[fields[1]].append(numbers)
    names = ['medianMs', 'p95Ms', 'correlationMs', 'coldFirstMs', 'coldWarmMs']
    result = {}
    for side, runs in rows.items():
        # Median over runs of each column (runs alternate a and b).
        columns = list(zip(*runs))
        result[side] = {name: sorted(column)[len(column) // 2]
                        for name, column in zip(names, columns)}
        result[side]['runs'] = len(runs)
    return result


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as file:
        for block in iter(lambda: file.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--region', default='malopolskie')
    parser.add_argument('--corpus', required=True)
    parser.add_argument('--gate', required=True)
    parser.add_argument('--gate-status', type=int, required=True)
    parser.add_argument('--commit', required=True)
    parser.add_argument('--baseline-commit', default='')
    parser.add_argument('--pbf')
    parser.add_argument('--phase')
    parser.add_argument('--timing')
    parser.add_argument('--timing-skipped')
    parser.add_argument('--json', required=True)
    args = parser.parse_args()

    corpus = json.load(open(args.corpus, encoding='utf-8'))
    gate, gate_text = parse_gate(args.gate)
    result = {'region': args.region, 'commit': args.commit, 'gatePassed': args.gate_status == 0, 'gate': gate}
    if args.pbf:
        result['pbfSha256'] = sha256(args.pbf)
    if args.phase:
        result['latencyByClass'] = classes(corpus, parse_phase(args.phase))
    if args.timing:
        result['timing'] = {'baselineCommit': args.baseline_commit, **parse_timing(args.timing)}
    with open(args.json, 'w', encoding='utf-8') as file:
        json.dump(result, file, indent=2)
        file.write('\n')

    out = sys.stdout
    verdict = 'passed' if args.gate_status == 0 else f'**failed** (exit {args.gate_status})'
    out.write(f'## Nightly {args.region} gate: {verdict}\n\n')
    out.write(f'Commit `{args.commit[:12]}`')
    if 'pbfSha256' in result:
        out.write(f", extract sha256 `{result['pbfSha256'][:16]}…`")
    out.write('.')
    if 'compileS' in gate:
        out.write(f" Compiled in {gate['compileS']} s.")
    if 'routes' in gate:
        routes = gate['routes']
        out.write(f" {routes['cases']} gate cases: median {routes['medianMs']:.2f} ms, "
                  f"p95 {routes['p95Ms']:.2f} ms.")
    out.write('\n\n')
    report = [line for line in gate_text.splitlines()
              if not line.startswith(('compiled ', 'routes:', 'routed in '))]
    if report:
        out.write('<details><summary>Gate report</summary>\n\n```text\n')
        out.write('\n'.join(report[-80:]) + '\n```\n\n</details>\n\n')
    if result.get('latencyByClass'):
        out.write('### Latency by class\n\n')
        out.write('Median request time per corpus case (phase bench), then p50/p95 over the '
                  'cases of each class. Shared runners are noisy: compare trends, not single nights.\n\n')
        out.write('| Class | Cases | p50 ms | p95 ms | max ms |\n|---|---:|---:|---:|---:|\n')
        for name, row in result['latencyByClass'].items():
            out.write(f"| {name} | {row['cases']} | {row['p50Ms']:.2f} | {row['p95Ms']:.2f} | "
                      f"{row['maxMs']:.2f} |\n")
        out.write('\n')
    if result.get('timing'):
        timing = result['timing']
        out.write(f"### Against the last release (`{args.baseline_commit[:12]}`)\n\n")
        out.write('gates.sh timing, a and b alternating on the same package; medians over runs.\n\n')
        out.write('| Build | warm median ms | warm p95 ms | correlation ms | cold first ms | cold warm ms |\n')
        out.write('|---|---:|---:|---:|---:|---:|\n')
        for side, label in (('a', 'release'), ('b', 'this commit')):
            if side in timing:
                row = timing[side]
                cells = ' | '.join(f'{row.get(name, float("nan")):.{3 if name == "correlationMs" else 2}f}'
                                   for name in ('medianMs', 'p95Ms', 'correlationMs', 'coldFirstMs',
                                                'coldWarmMs'))
                out.write(f'| {label} | {cells} |\n')
        out.write('\n')
    elif args.timing_skipped:
        out.write('### Against the last release\n\n')
        out.write(open(args.timing_skipped, encoding='utf-8').read().strip() + '\n')


if __name__ == '__main__':
    main()

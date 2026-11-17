"""Compare a route_response_dump output with a recorded baseline, case by case.

usage: regress.py <baseline.json> <routes.jsonl> <corpus.json> [--record]
                  [--dataset-hash H] [--random-pairs N]

The dump's corpus cases (and their `_hinted_alt` variants) and its `random_*`
pairs are reported apart. A case fails the gate when it newly fails (an error,
or an error of another kind, where the baseline has none or a different one),
when its cost rises beyond the engine's tie window (`COST_TOLERANCE`, relative,
default 1e-7: f32 edge weights summed in another order), or when its visited
edges rise by more than `VISITED_TOLERANCE` (default 0.25) and `VISITED_SLACK`
edges (default 500). The opposite changes are reported as improvements and do
not fail. Random points outside the extract give NoCandidate in the baseline
too, so they only fail if their outcome changes.

With --record the baseline is written from the dump instead. Exit status: 0
passed, 1 regressions, 2 the baseline does not match the corpus or the dump.
"""
import argparse
import hashlib
import json
import os
import sys
from collections import defaultdict


def visited_edge_count(diagnostics):
    for key in ('visitedEdgeCount', 'visited_edge_count'):
        if key in diagnostics:
            return diagnostics[key]
    raise KeyError(f'no visited edge count in the dump\'s diagnostics: {sorted(diagnostics)}')


def summarize(routes_path):
    cases = {}
    for line in open(routes_path):
        entry = json.loads(line)
        if 'case' not in entry:
            continue  # correlation lines
        response = entry['response']
        if 'error' in response:
            status = response['error'].replace('{', ' ').split()[0]
            cost = None
        else:
            status, cost = 'ok', response['cost']
        cases[entry['case']] = {
            'status': status,
            'cost': cost,
            # camelCase since the 0.1.0 API (snake_case in older dumps); a
            # dump without either is an error, not zero visited edges.
            'visited': visited_edge_count(response['diagnostics']),
        }
    return cases


def group(label):
    return 'random' if label.startswith('random_') else 'corpus'


def status_counts(cases):
    counts = defaultdict(int)
    for case in cases.values():
        counts[case['status']] += 1
    return ', '.join(f'{n} {status}' for status, n in sorted(counts.items(), key=lambda kv: -kv[1]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('baseline')
    parser.add_argument('routes')
    parser.add_argument('corpus')
    parser.add_argument('--record', action='store_true')
    parser.add_argument('--dataset-hash')
    parser.add_argument('--random-pairs', type=int)
    args = parser.parse_args()
    cost_tolerance = float(os.environ.get('COST_TOLERANCE', '1e-7'))
    visited_tolerance = float(os.environ.get('VISITED_TOLERANCE', '0.25'))
    visited_slack = int(os.environ.get('VISITED_SLACK', '500'))

    corpus_sha = hashlib.sha256(open(args.corpus, 'rb').read()).hexdigest()
    current = summarize(args.routes)
    if args.record:
        header = {
            'corpus': os.path.basename(args.corpus),
            'corpusSha256': corpus_sha,
            'randomPairs': args.random_pairs,
            'datasetHash': args.dataset_hash,
        }
        # One case per line, in dump order, so a re-recorded baseline diffs cleanly.
        with open(args.baseline, 'w') as out:
            out.write(json.dumps(header, sort_keys=True)[:-1] + ', "cases": {\n')
            out.write(',\n'.join(f'{json.dumps(label)}: {json.dumps(case, sort_keys=True)}'
                                 for label, case in current.items()))
            out.write('\n}}\n')
        for name in ('corpus', 'random'):
            part = {k: v for k, v in current.items() if group(k) == name}
            print(f'{name}: {len(part)} cases, {status_counts(part)}')
        print(f'recorded {args.baseline}')
        return 0

    baseline = json.load(open(args.baseline))
    if baseline['corpusSha256'] != corpus_sha:
        print(f'error: {args.corpus} changed since {args.baseline} was recorded; '
              'rerun with UPDATE_BASELINE=1 and commit the baseline', file=sys.stderr)
        return 2
    if args.random_pairs is not None and baseline.get('randomPairs') != args.random_pairs:
        print(f'error: {args.random_pairs} random pairs, the baseline has '
              f'{baseline.get("randomPairs")}', file=sys.stderr)
        return 2
    if args.dataset_hash and baseline.get('datasetHash') != args.dataset_hash:
        print(f'note: package dataset hash {args.dataset_hash} differs from the '
              f'baseline\'s {baseline.get("datasetHash")} (compiler or extract changed)')
    base = baseline['cases']

    failed = False
    for name in ('corpus', 'random'):
        labels = sorted(k for k in set(base) | set(current) if group(k) == name)
        regressions, improvements = defaultdict(list), defaultdict(list)
        for label in labels:
            b, c = base.get(label), current.get(label)
            if c is None:
                regressions['missing from the dump'].append(label)
                continue
            if b is None:
                improvements['new, not in the baseline'].append(label)
                continue
            if c['status'] != b['status']:
                if c['status'] == 'ok':
                    improvements[f'newly ok (was {b["status"]})'].append(label)
                else:
                    regressions[f'newly fails: {c["status"]}'].append(
                        label if b['status'] == 'ok' else f'{label} (was {b["status"]})')
                continue
            if c['status'] == 'ok':
                window = cost_tolerance * max(abs(b['cost']), 1.0)
                change = f'{label} {b["cost"]:.3f} -> {c["cost"]:.3f} ' \
                         f'({(c["cost"] - b["cost"]) / max(b["cost"], 1e-9) * 100:+.4f}%)'
                if c['cost'] > b['cost'] + window:
                    regressions['cost increase'].append(change)
                elif c['cost'] < b['cost'] - window:
                    improvements['cheaper'].append(change)
            delta = c['visited'] - b['visited']
            change = f'{label} {b["visited"]} -> {c["visited"]} ' \
                     f'({delta / max(b["visited"], 1) * 100:+.0f}%)'
            if abs(delta) > visited_slack and abs(delta) > visited_tolerance * b['visited']:
                (regressions if delta > 0 else improvements)['visited edges'].append(change)
        part_base = {k: v for k, v in base.items() if group(k) == name}
        part_now = {k: v for k, v in current.items() if group(k) == name}
        visited = [sum(v['visited'] for v in part.values()) for part in (part_base, part_now)]
        print(f'{name}: {len(part_now)} cases; baseline {status_counts(part_base)}; '
              f'now {status_counts(part_now)}; visited edges {visited[0]} -> {visited[1]}')
        for title, items in sorted(regressions.items()):
            failed = True
            print(f'  REGRESSION {title} ({len(items)}):')
            for item in items:
                print(f'    {item}')
        for title, items in sorted(improvements.items()):
            print(f'  improvement {title} ({len(items)}):')
            for item in items:
                print(f'    {item}')
    print(f'visited-edge tolerance +{visited_tolerance * 100:.0f}% and {visited_slack} edges; '
          f'cost tolerance {cost_tolerance:g} relative')
    print('FAILED' if failed else 'passed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())

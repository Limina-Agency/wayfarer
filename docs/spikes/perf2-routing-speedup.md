# PERF-2 spike: routing speed-up

Status: spike, 2026-11-17, against 0.1.3 (c0ff0c3). Prototype code is on the
branch `perf/routing-spike`, behind the `debug-env` switches; nothing
here is meant to be merged as it stands.

## Recommendation

**Adopt CCH lower-bound potentials (prototype A) as the PERF-3/4/5 path; do
not start an edge-based CCH (B) or MLD (C).** On Małopolskie, exact CCH
potentials under today's edge-based search give **0 cost mismatches** and cut
plain-drive visited edges 4-7x (random drives median 0.15x of ALT6). Plain
drives go from p95 31.6 / p99 52.4 ms (ALT6) and 20.9 / 49.6 ms (ALT16, the
0.2 quick fix) to **11.1 / 16.0 ms**, and walks from p95 69.5 to **28.1 ms**.
That run was taken in the less loaded of two windows; neither window was
quiet. Search is now a quarter of drive request time: leg building is the
next limit (PERF-Q1), not the search. Package format 2 should store the
order (and the topology, ~23 MB) and customize at open (~0.11 s per column
on 16 threads). The landmarks (88.9 MiB) go only once the potential also
covers discount gates, matrices and own-metric columns.

**Alternatives are not fixed by a better potential.** The penalty method's
penalised optimum is far from the base metric, so CCH on the base metric
does nothing for them (p95 903 ms vs 908). Re-customizing the CCH per attempt
keeps today's output exactly (95/95 alternatives, identical) and cuts p95 to
277 ms, but each partial re-customization costs 11-21 ms. Only a per-attempt
budget reaches the 60 ms target (31-41 ms), and it loses a third of the
alternatives (59-62 of 95). Ship the bounded variant as the 0.x safety net
(PERF-A1). Build via-node/plateau alternatives on the CCH for 1.0 (PERF-7):
a node-level one-to-all PHAST sweep costs 9.3 ms here.

## Machine, data, method

- `tony-pc`: Ryzen 7 5800H (8c/16t), 16 GB, Windows 11, Rust 1.99.1,
  `--release` (workspace profile), 16 rayon threads. Shared with other
  agents; **every timing below was taken under load** and is marked with the
  CPU load sampled every 5 s during the run.
- Package: the 0.1.3 Małopolskie package the PERF research compiled from the
  same extract (203,820,142 B, sha256 34773f9a…155a; 272,769,856 B package;
  1,069,065 nodes, 2,570,035 edges).
- Harness: `wayfarer/examples/perf2_bench.rs`. Same requests as
  `route_response_dump` (150 m correlation, 2 M visited-edge budget). Classes:
  the 62-case corpus (47 drives, 15 walks), the corpus again with 2
  alternatives, and 200 seeded random pairs in the corpus bounding box (every
  4th on foot). Arms alternate inside one process, round by round
  (A B C … A B C …), each case 3 times back to back per round, 2 rounds, so
  6 runs per case per arm; a case's time is its minimum over the 6, and the
  percentiles are over cases. A fixed integer loop at the start of each round
  measures contention.
- Exactness: every arm's primary cost is compared with ALT6's per case
  (tie window 1e-9 relative).

```sh
CARGO_TARGET_DIR=<outside the repo> cargo build --release -p wayfarer --example perf2_bench
perf2_bench <malopolskie.mrt> benchmarks/gates/malopolskie-corpus.json 200 3 2 \
  alt6,alt16,cch,cch:recustom,cch:bound,alt6:budget4,cch:budget4,cch:bound+recustom out.jsonl
# PERF2_CHECK=6: CCH distances vs Dijkstra; PERF2_PHAST=1: one-to-all sweep timing
```

## Results

### Plain routes (ms, p50 / p95 / p99 over cases of min-of-6)

Run 1: 22:57-23:12, CPU load mean 81 % (max 100 %), calibration 141 / 101 ms
(quiet machine ~95 ms).

| Arm | Corpus drives (47) | Random drives (117) | **All drives (164)** | **Walks (54)** | Drive visited p50 / p95 | Walk visited p50 / p95 | Cost mismatches |
|---|---|---|---|---|---|---|---|
| ALT6 (0.1.3) | 5.7 / 15.5 / 17.2 | 9.5 / 43.7 / 52.4 | 7.8 / 31.6 / 52.4 | 9.7 / 69.5 / 164 | 12.2k / 111k | 31.9k / 357k | – |
| ALT16 (0.2 fix) | 6.0 / 12.6 / 17.3 | 8.4 / 25.2 / 49.6 | 7.3 / 20.9 / 49.6 | 9.7 / 74.4 / 142 | 10.2k / 54k | 31.9k / 357k | 0 |
| **CCH** | 4.1 / 11.3 / 15.4 | 5.6 / 11.1 / 16.0 | **5.4 / 11.1 / 16.0** | **5.8 / 28.1 / 43.7** | **1.9k / 8.9k** | **16.7k / 110k** | **0** |

Run 2 (23:13-23:33, load mean 96 %, calibration 118 / 154 ms) has the same
order with everything ~2x slower: all drives ALT6 13.6 / 62.3 / 154, ALT16
14.8 / 61.5 / 164, CCH 12.2 / 31.4 / 46.9; walks ALT6 15.8 / 151, ALT16
17.8 / 247, CCH 13.6 / 97.7; 0 cost mismatches. Visited edges do not depend
on load and are identical in both runs.

Per case, CCH visits a median 0.15x of ALT6's edges on random drives (p90
0.60x), 0.40x on random walks, 0.54x on corpus drives. A visited edge costs
more with CCH (≈350 vs ≈210 ns: memoised potential evaluation along the
elimination tree). Summed over the drives, search time falls 4.2x (243 vs
1,022 ms) while leg building stays (657 vs 702 ms). **Leg building is now
69 % of drive request time.**

### Alternatives (corpus with 2 alternatives, 62 cases)

Run 1 (load mean 81 %). "Identical": same set of accepted alternatives
(edge paths) as ALT6 penalty.

| Arm | p50 | p95 | p99 / max | Accepted | Identical | Cost ratio | Shared ratio | Attempts |
|---|---|---|---|---|---|---|---|---|
| ALT6 penalty (0.1.3) | 33.6 | 908 | 974 | 95 | 62 | 1.198 | 0.288 | 217 |
| ALT16 penalty | 34.3 | 986 | 1137 | 95 | 62 | 1.198 | 0.288 | 217 |
| CCH penalty | 37.7 | 903 | 1577 | 95 | 62 | 1.198 | 0.288 | 217 |
| **CCH re-customized per attempt** | 50.3 | **277** | 508 | **95** | **62** | 1.198 | 0.288 | 217 |
| CCH, cost bound | 13.4 | 322 | 378 | 80 | 46 | 1.185 | 0.285 | 249 |
| ALT6, cost bound | 14.8 | 260 | 599 | 80 | 46 | 1.185 | 0.285 | 249 |
| CCH, cost bound + re-customized | 39.5 | 100 | 176 | 80 | 46 | 1.185 | 0.285 | 249 |
| ALT6, budget 4x | 13.0 | 41.3 | 66.6 | 62 | 35 | 1.164 | 0.348 | 143 |
| **CCH, budget 4x** | 11.3 | **31.3** | 36.3 | 59 | 33 | 1.157 | 0.362 | 141 |

- *Penalty method with a better base potential* (ALT16, CCH): no gain. The
  penalised optimum climbs to ~3x the primary's cost, far outside what any
  base-metric bound sees; CCH's dearer visits make it slightly slower.
- *Re-customized per attempt* (`recustom`): the potential is exact for the
  penalised metric, so the result is unchanged and the searches are short,
  but 801 partial re-customizations took 16.6 s in total: 20.8 ms each
  (3,780 node relaxations each, mostly the top separators' long downward
  lists; 11 ms with the cost bound, 1,280 nodes). The re-customization is
  in place with an undo log and runs level by level in parallel; a quieter
  single-run measurement gave 21 ms including a 30 MB metric copy, now
  avoided. Triangle-precise partial updates (recompute only arcs whose old
  weight came through a changed triangle) are the known fix, *estimated* at
  3-5x. Even then the floor is ~6 attempts × (a few ms + search + leg) per
  request.
- *Cost bound* (an attempt stops once its penalised lower bound passes
  `max_cost_ratio` × the primary; skipped instead of ending the loop): drops
  candidates whose real cost qualifies but whose penalised cost does not
  (80 of 95 kept).
- *Budget* (4x the primary's visited edges, min 20k; an attempt out of
  budget ends the loop, as in the PERF research): meets the target, keeps 59-62
  alternatives. With CCH the primary visits fewer edges, so the cap is
  tighter and slightly fewer alternatives survive.

### Prototype A: build, exactness, memory

| | Value |
|---|---|
| Node graph | 1,069,065 nodes, 1,278,932 undirected arcs |
| Degree ≤ 2 eliminated first | 733,663 (68.6 %); core 335,402 nodes |
| Nested dissection (inertial flow, 4 directions, Dinic), 16 threads | 3.1 s (load 96 %) to 8.8 s (run 1, load spike); top separator 58 nodes |
| Topology (fill, elimination tree, edge → arc maps) | 0.5-1.2 s |
| CCH arcs | 3,694,425 (2.9 per undirected input arc); 23.2 M lower triangles |
| Elimination tree height (= max upward search space) | 524; mean upward search space 309 nodes |
| Customization per stored column | 109-118 ms parallel (16 threads), 326-358 ms single thread (run 2); 181 ms / 389 ms (smoke) |
| Exactness vs node-level Dijkstra | 3,000,002 (node, root) pairs over both columns and both directions: **0 mismatches**, worst relative error 7.7e-7 (below: weights are rounded down) |
| PHAST one-to-all sweep | 9.3 ms per source, 0 mismatches vs Dijkstra (all 1.07 M nodes) |
| Topology in memory (spike layout) | 105 MB, of which a stored format needs rank + `first_up` + `up_head` ≈ 23 MB; the rest (downward lists, arc → edge maps, levels) is derived at open in ~0.3 s or only needed for re-customization |
| Metric in memory | 29.6 MB per column (2 × f32 per arc) |
| Query scratch | 2 × 24 B per node ≈ 51 MB per pooled search state (epoch arrays; a hash map for the touched part would cut it to ~1 MB) |

### Where the potential applies

The potential is the node-level distance (turn costs left out; turn costs
are never negative) under a **stored** column, scaled by `factor`. Per
search it is used when all of these hold, otherwise the search keeps ALT and
a counter records why:

1. the query uses landmarks today (`query.landmarks()` is `Some`: every
   override only raises the column);
2. the column is stored (`factor = 1`), or derived with every usable edge
   usable in the stored column, `factor` = least derived/stored cost ratio
   × (1 − 1e-6). This covers the strict destination-only pass every default
   drive starts with, and costing options;
3. every override is finite only where the stored column is, and at least
   `factor` × the stored cost. The strict pass's unblocked pockets (gates
   relative to the strict column) pass: they cost exactly the stored cost.
   Real discounts, opened edges and SAR openings do not; they keep ALT with
   gates. Gates with a CCH potential are a direct extension, since the
   elimination-tree walk takes initial distances:
   `pot = min over targets and gates of d(v, ·) + offset`.

In the benchmark, every one of the 1,719 searches of the CCH arm (round 0) used
the potential; none fell back.

### Parity

- **CCH distances vs node-level Dijkstra:** 0 mismatches over 3.0 M pairs
  (table above), PHAST 0 mismatches over all nodes.
- **`cargo test -p wayfarer` with `WAYFARER_POTENTIAL=cch`:** 178 unit
  (including the package tests against the `reference` oracle on their
  fixture graphs), 3 + 5 + 121 + 1 integration tests: all pass. The test build
  has `debug-env` on (checked), so the switch was in effect.
- **`route_parity`, package engine ALT6 vs CCH** (corpus + 60 random pairs,
  dumps compared case by case; `ROUTE_PARITY_SKIP_REFERENCE=1`):

| Option set | Cases routed | Cost mismatches | Path differences | Status differences | Visited CCH/ALT6 | Bound used |
|---|---|---|---|---|---|---|
| plain | 101 | 0 | 0 | 2: ALT6 out of the 250k budget, CCH routes | 0.23x | CCH 116 |
| plain + `WAYFARER_VERIFY` | 101 | 0 | 0 | same 2 | 0.23x | CCH 116, 0 verification failures |
| costing options (`avoidMotorways`, `preferPaved`) | 89 | 0 | 0 | 5, all ALT6 out of budget | 0.79x | CCH 116 (derived column, scaled) |
| overlay (block + bypass connector) | 101 common | 0 | 0 | 0 (2 more cases exist only under CCH: their plain route needs it) | – | patch engine, unchanged |
| discount 0.5 on the route | 103 | 0 | 0 | 0 | 1.00x | ALT with gates (114 fallbacks, by design) |
| SAR turn-restriction relaxation | 73 | 0 | 0 | 4, all ALT6 out of budget | 0.15x | CCH 90 |

  Costs agree to 5e-16 relative and every edge path is identical, ties
  included. The `reference` oracle itself was not run on Małopolskie (no
  road-graph snapshot tooling; see the open questions).

## Decision criteria (Małopolskie)

| Criterion | Target | Prototype A (CCH) | Verdict |
|---|---|---|---|
| Plain-drive p95 / p99 | ≤ 10 / ≤ 20 ms | 11.1 / 16.0 ms under 81 % load (ALT6 31.6 / 52.4, ALT16 20.9 / 49.6, same run) | p99 met; p95 within 11 % under load, likely met quiet; the rest is leg building |
| Walk p95 | ≤ 30 ms | 28.1 ms under load (ALT6 69.5) | met |
| 2-alternative p95 | ≤ 60 ms | 903 ms with today's method; 277 ms re-customized (same output); 31 ms with a budget (59 of 95 alternatives) | not met with today's quality; needs PERF-7 |
| Cost mismatches vs `reference` | 0 | 0 vs ALT6 on every class and arm; `cargo test -p wayfarer` under CCH passes (308 tests); `route_parity` dumps identical to ALT6 on plain, verified, costing, overlay, discount and SAR sets; `reference` itself not run on Małopolskie | met as far as tested |
| Package size | ≤ 260 MiB | no format change in the spike; storing order + topology adds ~23 MB; dropping the landmarks saves 88.9 MiB once they have no other use | ~194 MiB reachable |
| Compile | ≤ +30 % wall, ≤ 2 GiB | order + topology 3.6-4.3 s on 16 threads (*loaded*) vs the 3.0 s landmark stage it would replace: about +1 s on 17-18 s (+4-7 %) if the landmarks go; memory far below the 1.2 GiB peak | met (*estimated*) |
| Open to first route incl. customization | ≤ 1 s | open 0.17 s (warm) + one customization 0.11 s = ~0.3 s with order and topology stored; ~0.8 s with only the order stored; 4.5-20 s with the spike's build from scratch | met with a stored order |
| Overlay request p95 | ≤ 2x plain | overlays go through the patch engine, which keeps its own bounds; unchanged by the spike | not affected |

## Other candidates (on paper)

**Core-ALT.** The degree-2/tree contraction is shipped (pieces and walks).
ALT16 is ALT's bound-quality ceiling on this graph with no format change,
and the CCH arm visits 5-6x fewer drive edges than ALT16 at p95 (8.9k vs
54k). Core-ALT would only move landmark bytes to the core. Not worth a
prototype.

**Full edge-based CCH query (B).** Not prototyped: after A, drive search is
~1.5 ms per request on average, so B would save at most that while leg
building stays. B needs turn-aware customization (turn-expanded graph ~2.4x
the nodes), path unpacking, re-customization or patch searches for every
overlay, discount and SAR request, and parity on costs only. Revisit for
matrices (PERF-8) and country scale.

**MLD / partitions (C).** Estimated from prototype A's dissection (core
nodes, after degree-2 elimination; boundary nodes = the separators of all
cells larger than the bound; clique entries assume each boundary node on two
cells):

| Cell bound (core nodes) | Cells | Boundary nodes | Clique entries |
|---|---|---|---|
| 256 | 2,140 | 15,093 | 426k |
| 1,024 | 557 | 6,298 | 285k |
| 4,096 | 166 | 2,490 | 149k |
| 16,384 | 67 | 1,006 | 60k |
| 65,536 | 39 | 302 | 9k |

A 2-3 level MLD has clique matrices of a few MB per metric here, customizable
in milliseconds, and a query comparable to A's search time. It wins at
country scale and for per-partition rebuilds (EXP-5/6), not on regional
latency, and needs the same turn and overlay work as B. On paper only.

**Via-node / plateau alternatives (PERF-7).** Not prototyped. Cost basis
measured: a node-level one-to-all PHAST sweep is 9.3 ms (two per request:
from the origins, to the destinations), then via-node candidates
`d(s, v) + d(v, t) ≤ (1 + ε) d(s, t)`, ranked by sharing and local
optimality, each finished by two short edge-based searches with CCH
potentials (~1-3 ms each). *Estimated* 30-50 ms per 2-alternative request,
with results that differ from 0.1's.

## Open questions

1. Are alternatives allowed to change for 1.0 (via-node/plateau instead of
   penalties)? Only that, or a budget with fewer alternatives, meets 60 ms.
2. Store the order only (smaller, +0.5 s at open) or order and topology
   (+23 MB, ~0.3 s to first route)? Store metrics too for mobile (29.6 MB per
   column)?
3. Drop the landmarks? It needs the potential with gate offsets (discounts,
   opened edges, SAR openings), potentials for matrices and isochrone-free
   analysis paths, and own-metric derived columns (Emergency, Tracked, R5)
   customized at first use like their landmarks today.
4. A quiet-machine re-run of the plain classes, to confirm the drive p95 is
   under 10 ms (both windows today were loaded), and the `reference` oracle
   on Małopolskie: the compiler has no public way to write the road-graph
   snapshot `route_parity` needs, so the parity here is against ALT6.
5. Nested dissection uses floating-point projections (deterministic on one
   machine, mul/add only). A production order must be platform-independent:
   fixed-point coordinates from `NodeXyzFixed` and integer tie-breaks.
6. Per-state scratch for potentials is 51 MB in the spike (16 pooled states
   on desktop); the format-2 version needs a compact touched-node map.

## Artifacts

- Code: `wayfarer/src/package/cch.rs`, hooks in `package/search.rs`,
  `package/columns.rs`, `package/landmarks.rs`, `search.rs`, switches in
  `debug.rs`, harness `examples/perf2_bench.rs`.
- Raw outputs (not committed, on `tony-pc`): `E:\repos\wayfarer-wt\bench-out\`
  (`full1.*` run 1, `quiet2.*` run 2, `parity\` route_parity dumps,
  `test-cch.txt`).

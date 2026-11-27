# Stored potentials: the CCH topology and metrics in the package

Status: design (POT 4), for review. Nothing here is built yet. It fixes what
POT 5 (`feat/stored-cch-topology`) and POT 6 (`feat/stored-cch-metrics`) write
behind the format-3 switch, and what FMT 5 (cell-major numbering) has to leave
room for. Stored potentials go on by default with the format-3 release (0.7).

Inputs: `wayfarer/src/package/cch.rs` (the potentials as built today),
`docs/design/format-3.md` (sections 3, 6 and 8, which reserve the ids), the
open-memory analysis (OM-6, OM-7, OM-12) and the tranche plan's defaults
(section 6: topology plus the default column stored, more columns opt-in,
`--no-stored-potentials` to opt out).

## 1. Why

Every search bounds itself with CCH potentials. Today an engine builds them
from the package after `open`: the topology on the first search, then each
stored column's metric on the first search that uses that column. On
Małopolskie (1.07 M nodes, 3.69 M arcs, 4 threads, *measured* in POT 1) the
first route spends about 100–115 ms on the topology and about 85 ms
customizing one column, out of about 290 ms. On Poland (10.56 M nodes,
47.8 M arcs, a 1.84 GiB package) the topology takes about 1.4 s and each
column about 2.8 s, and the potentials hold 1.58 GiB of heap with both
columns customized (*measured*, POT 1 branch, 4 threads). Stored sections
remove both steps from the first route and move the arrays from anonymous
heap to file pages the OS can share and evict, which matters most on
Mobile.

Target: first route after `open` costs about one search (Małopolskie about
50 ms, *estimated* from the search's own 30–50 ms).

## 2. What is stored

Four arrays are worth storing. The rest are cheap to derive or are only
needed for customization.

| Array today | Stored as | Why |
|---|---|---|
| `first_up` (n + 1) | `CchFirstUp` (322), u64 | read by every potential |
| `up_head` (arcs) | `CchUpHead` (323), u32 | read by every potential |
| the metric of stored column k: `up`, `down` (arcs each) | `CchMetric` (324, `param` = k), 8 B per arc | read by every potential; the customization it replaces is the largest open cost |
| `rank` | `CchRank` (already stored), or nothing once node id = rank (format-3 section 8.3) | |
| `node` (inverse of `rank`) | not stored: one parallel pass at first use, or the identity once node id = rank | |
| `parent` | not stored: it is the first head of each up list (`up_head[first_up[x]]`, none for an empty list) | |
| customization arrays (`first_down`, `down_tail`, `down_arc`, `level_order`, `level_start`, `edge_arc`, `edge_blocks`) | not stored | only customization reads them; built on demand (section 5) |

Layouts are format-3 section 6's: `CchFirstUp` is u64 so a planet-scale arc
count fits; the engine narrows it to its in-memory u32 when the count is
below 2^31 (today's `UP_BIT` limit) and otherwise falls back to the straight
line, as it does now. `CchMetric` interleaves each arc's up and down weight,
so one potential step reads one 8-byte pair. Weights are f32, rounded down
exactly as customization rounds them (section 7).

Not stored: the elimination-tree levels (derived in one pass), and the
potentials' per-search scratch.

## 3. Which columns

Default (the tranche plan's section 6 default): the topology plus the
metric of the column that `Wheeled` (and `Default`) routes on. On today's
packages that is column 0. More columns are opt-in:

- `--stored-potentials default` (the default): topology + Wheeled's column.
- `--stored-potentials all`: topology + every stored column.
- `--stored-potentials <profile>,...`: topology + the named profiles' columns.
- `--no-stored-potentials`: none; the engine builds everything as today.

`CompileRequest` gets the same choice (`storedPotentials`). Columns derived
at request time (non-default costing options, Tracked's and Emergency's own
metrics, SAR relaxations) are never stored: their costs come from the
request, and they keep customizing on first use from the stored topology.

## 4. Size and compile cost

Małopolskie (*measured* array sizes; the package is 176.4 MiB today):

| Section | Size | Share of the package |
|---|---|---|
| `CchFirstUp` | 8.2 MiB | +4.6 % |
| `CchUpHead` | 14.1 MiB | +8.0 % |
| `CchMetric`, one column | 28.2 MiB | +16.0 % |
| Default (topology + one column) | 50.5 MiB | +28.6 % |
| All (two columns) | 78.7 MiB | +44.6 % |

With node id = rank (FMT 5, recommended) `CchRank` goes (−4.1 MiB), so the
default costs about +46 MiB net. f32 weights barely compress (OM-7), so
downloads grow about as much as the file.

Poland (*computed* from its measured counts; the package is 1.84 GiB):

| Section | Size | Share of the package |
|---|---|---|
| `CchFirstUp` | 80.6 MiB | +4.3 % |
| `CchUpHead` | 182.4 MiB | +9.7 % |
| `CchMetric`, one column | 364.9 MiB | +19.4 % |
| Default (topology + one column) | 627.8 MiB | +33.4 % |
| All (two columns) | 992.7 MiB | +52.8 % |

The share grows with the region because the arcs per node grow with the
separators (3.5 per node on Małopolskie, 4.5 on Poland).

Compile time: the compiler builds the topology (POT 1's parallel build) and
customizes each stored column once. On Małopolskie that is about 0.1 s for
the topology and about 0.15 s per column on 4 threads, against a 17–23 s
compile. On Poland it is about 1.4 s for the topology and 2.8 s per column
against a 250 s compile (*measured*). The transient memory (about 105 MiB on
Małopolskie for the topology and one metric, *measured* as
`PotentialStats.heap_bytes`, about 1.2 GiB on Poland) stays below the
compile's peak, which comes from the line network.

## 5. Engine behaviour

**Open.** Unchanged: `open` checks the table and each known section's
lengths, and with lazy validation (POT 2) it defers the per-element checks.
The stored sections are mapped, not read.

**First use.** `CchCache::prepare` finds `CchFirstUp` and `CchUpHead` and
builds a `CchTopology` that borrows them instead of building the arrays.
`stored_metric(k)` borrows `CchMetric` with `param` k when present and
customizes otherwise. The types change from owned `Vec`s to a borrowed or
owned slice (`Cow`-like), so every search path reads the same type either
way.

**Fallbacks**, each to today's behaviour:

| Package has | Engine does |
|---|---|
| no stored sections (format 2, or `--no-stored-potentials`) | builds topology and metrics as today |
| topology, no metric for column k | borrows the topology, customizes column k on first use |
| metric for column k, no topology | ignores the metric (its arc ids are only defined by the stored topology) and builds as today |
| a stored section that fails its checks | `package_invalid` from the first request that needs it (format-3 section 3), as for any section |

**Customization from a stored topology.** Own metrics and unstored columns
still need the customization arrays. They are built from the stored topology
on first need (the downward lists, levels and edge arcs, about 40 ms on
Małopolskie at 4 threads after POT 1) and dropped afterwards on Mobile, as
OM-6 does today. A session that only routes on the stored column never
builds them.

**Memory.** With the default sections a Małopolskie session that only drives
keeps no potentials heap beyond the per-search scratch: about 50 MiB moves
from anonymous memory to mapped pages, which the first searches fault in as
they touch them. On Poland that is about 0.6 GiB of the 1.58 GiB heap; most
of the rest is the customization arrays (dropped on Mobile, never built by a
session that only drives) and the walking column's metric (unless stored).

**Warm-up (POT 3).** With stored potentials there is little left to warm up:
`prefetch` then derives the drive's strict column, pools a search state and
faults the file in, the stored sections with it. The background warm-up
option (`warmUp`) keeps its meaning (do the first-use work now) and becomes
cheap: about 0.3 s of work on Małopolskie today, most of it the topology and
the drive metric, which stored sections remove.

## 6. Checks

The stored sections are derived data: they must equal what the engine would
compute from the identity sections, bit for bit (format-3 section 3). That
is what keeps them out of the identity hash, so a package with and without
them has the same identity, and an overlay set or a result cache bound to
one works with the other.

- **Open / first use (cheap, structure only).** `CchFirstUp` has n + 1
  entries, starts at 0, never decreases and ends at `CchUpHead`'s length;
  each up list is strictly ascending and its heads are above its own rank and
  below n; `CchMetric`'s length is twice the arc count in f32; weights are
  not NaN and not negative (infinity marks an unusable arc). These are what
  the searches and the customization's unchecked writes rely on, so they
  run before anything reads the arrays (with POT 2, on first use rather than
  at open). They are one pass over the arrays, in parallel.
- **`verify` (full).** Rebuild the topology and customize every stored
  column, then compare bitwise. This is the install-time check of a
  downloaded package and costs what an open without stored potentials costs
  today (about 0.4 s on Małopolskie and 7 s on Poland, both columns).
- **What a wrong stored metric can do.** Structure checks keep every index in
  range, so a corrupt or hostile metric cannot make the engine read out of
  bounds or panic. It can make potentials overestimate, and then a search
  can return a longer route than the best one. That is the same trust a
  package's costs get. Corruption is caught by the section hashes (`verify`),
  and the engine never trusts a metric without the topology it was built on
  (the metric is dropped when the topology section is absent).

## 7. Determinism

A stored metric must be the bits an engine computes, on any platform and
thread count. Customization already is (POT 1 and #86 test this): weights
are the minimum of input weights and lower triangles, sums in f64 rounded
down to f32, no fused multiply-add and no libm. To keep one implementation,
the topology build and customization move from `wayfarer::package::cch` to
`wayfarer_package` (next to `cch_order`, which the compiler already runs),
and the compiler and engine call the same code. The Monaco info and Kraków
checks then cover the new sections like any other package bytes.

## 8. Interactions

- **FMT 5 (cell-major numbering, node id = rank).** Recommended before POT 5:
  `CchFirstUp` and `CchMetric` are then indexed by node id, `CchRank` and
  the `node` array go away, and a cell is one range in every per-node and
  per-arc section, so a future `CellDirectory` (REG-13) can page stored
  potentials with the rest of a cell. FMT 5 should place the stored sections
  after the per-node and per-edge ones and not interleave them, so that a
  package without them is the same bytes up to the table.
- **POT 2 (lazy validation).** The structure checks of section 6 are
  per-section checks on first use, the same mechanism.
- **Terrain virtual trails** (TERR lane). Edges the compiler adds to the
  graph change the topology, so a package with them stores potentials
  built with them: nothing special, as long as they are added before the
  order is computed.
- **Overlays and patches.** Unchanged: they bound their searches with the
  base potentials plus gate offsets (connectors, discount gates), whether
  the potentials were built or stored. An overlay never writes to them.
- **Regions (REG-10).** Stored potentials are per package. A region bundle
  stores one set per member.

## 9. Decisions

1. **Topology plus the Wheeled column by default** (the tranche plan's
   section 6, recommended). Alternative: every stored column (+28 MiB per
   extra column here); or the metric only, rebuilding the topology at open
   (saves 22 MiB, costs about 100 ms per first route).
2. **`CchFirstUp` stays u64** as format 3 reserves it (recommended), rather
   than u32 with a planet-scale exception: 4 MiB more on Małopolskie.
3. **`verify` recomputes and compares** (recommended). Alternative: hashes
   only, and trust the compiler.
4. **Move topology and customization to `wayfarer_package`** so the compiler
   and the engine share one implementation (recommended), rather than a
   second copy in the compiler.

## 10. Plan

- POT 5 `feat/stored-cch-topology` (after FMT 5): move the topology build to
  `wayfarer_package`; compiler writes `CchFirstUp`/`CchUpHead` behind the
  format-3 switch; the engine borrows them; structure checks; `verify`
  recompares; Monaco format-3 goldens re-recorded.
- POT 6 `feat/stored-cch-metrics`: move customization; compiler writes
  `CchMetric` for the chosen columns; `--stored-potentials` and
  `CompileRequest.storedPotentials`; customization arrays on demand.
- Format-3 release (0.7): stored potentials on by default; Poland and Mobile
  measurement (first route, heap, package size).

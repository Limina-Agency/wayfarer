# wayfarer-cli

The `wayfarer` command line: compiles OpenStreetMap `.osm.pbf` extracts into
offline routing packages (`routing.mrt`), inspects packages and routes on
them. The compiler
itself is the `wayfarer-compiler` library; this crate is its command
line.

```sh
cargo install wayfarer-cli

wayfarer compile monaco.osm.pbf -o routing.mrt --country MC
wayfarer info routing.mrt
wayfarer verify routing.mrt
wayfarer route routing.mrt --from 43.7397,7.4275 --to 43.7290,7.4160 --geojson > route.geojson
```

`wayfarer help <command>` describes every flag.

## `wayfarer compile`

```sh
wayfarer compile <input.osm.pbf> -o <routing.mrt> [--country CC] [--graph-id ID]
                 [--scratch-dir DIR] [--low-memory] [--threads N] [--profiles P,...]
wayfarer compile --request <request.json | ->
```

Compiles a package. Progress goes to stderr; the result goes to stdout as
JSON (`CompileResult`: the package's path, `byteLen`, `datasetHash`,
`formatVersion`, `costingModelVersion`, `graphId`, `country`, `nodeCount`,
`edgeCount` and the build `report`).

- `--country CC`: the ISO country code recorded in the package.
- `--graph-id ID`: the package's graph id (default: the input's file name
  without `.osm.pbf`).
- `--scratch-dir DIR`: where scratch files go. The compile works in a fresh
  directory inside `DIR` and removes it when it ends (default: the output's
  directory).
- `--low-memory`: smaller ingest batches and queues, for memory-constrained
  machines; slower. The package's bytes are the same.
- `--threads N`: the compile's threads and the ingest's decode workers
  (default: all cores, at most two decode workers with `--low-memory`). The
  package's bytes are the same for any count.
- `--profiles P,...`: the profiles whose costs the package stores, a column
  each (`wheeled`, `on_foot`, `tracked`, `emergency`, `legged`; default
  `wheeled,on_foot`). The engine derives the others' costs when a profile is
  first used, so every profile routes on every package; storing one costs
  about 8 bytes an edge and its customization when the package opens, and
  saves computing and holding it on first use. The order doesn't matter.
- `--request FILE` (or `-` for stdin): the compile as `CompileRequest` JSON
  instead of paths and flags, the same JSON the C ABI takes:

  ```json
  {"input": "monaco.osm.pbf", "output": "routing.mrt", "scratchDir": "/tmp",
   "graphId": "monaco", "country": "MC", "memory": "low", "threads": 2,
   "profiles": ["wheeled", "on_foot"]}
  ```

  Only `input` and `output` are required; `memory` is `"standard"` (default)
  or `"low"`. `schema/compile-request.json` in the repository describes it.

A package already at the output path is replaced only by a complete one.
Ctrl-C (or SIGTERM) cancels: the compile removes its scratch and partial files
and exits with status 130; a second signal exits at once. Other failures exit
with status 1 and a message on stderr.

## `wayfarer info` and `wayfarer verify`

`wayfarer info <routing.mrt>` prints the package's format (and the formats
this build reads), costing model, graph id, country, dataset hash, byte
length, counts and quality as JSON. `wayfarer verify <routing.mrt>` checks
every section against its checksum and prints the sections (name, byte length,
BLAKE3) as JSON.

## Routing on a package

```sh
wayfarer route <routing.mrt> --from LAT,LNG [--via LAT,LNG ...] --to LAT,LNG
               [--profile wheeled|on_foot|tracked|emergency|legged] [--geojson]
wayfarer matrix <routing.mrt> --source LAT,LNG ... --target LAT,LNG ... [--profile P]
wayfarer nearest <routing.mrt> --source LAT,LNG --target LAT,LNG ... [-k N]
                 [--direction from_source|to_source] [--profile P]
wayfarer isochrone <routing.mrt> --from LAT,LNG [--minutes N ...] [--distance-m M ...]
                   [--profile P]
wayfarer snap <routing.mrt> --at LAT,LNG [--max-distance-m M] [--profile P]
wayfarer match <routing.mrt> --request <trace.json | ->
```

Each prints the engine's result as JSON on stdout (keys camelCase, as the C
ABI returns them). `route --geojson` prints a GeoJSON FeatureCollection
instead: the route's line and where each location snapped, ready for QGIS or
geojson.io. `nearest` returns the `k` targets (1 by default) cheapest to reach
from the source by route cost, cheapest first, each named by its index among
the `--target`s; with `--direction to_source`, the targets that reach the
source soonest. `isochrone` takes travel-time contours (`--minutes`), distance
contours in metres along the cheapest path (`--distance-m`), or both, and
returns one bucket per contour. Every command except `match` also takes `--request <file | ->`
with the request JSON the C ABI takes (`schema/` in the repository describes
each), which reaches what the flags don't: closures and other overlays,
overlay sets, costing options, alternatives. For example, a route around a
closed area:

```json
{"locations": [{"point": {"lat": 43.7397, "lng": 7.4275}},
               {"point": {"lat": 43.7290, "lng": 7.4160}}],
 "profile": "wheeled",
 "overlays": [{"geometryKind": "polygon", "policy": "no_go", "projectionDistanceM": 5,
               "points": [{"lat": 43.734, "lng": 7.418}, {"lat": 43.734, "lng": 7.424},
                          {"lat": 43.737, "lng": 7.424}, {"lat": 43.737, "lng": 7.418}]}]}
```

A request that cannot be answered (no road near a point, no path) exits with
status 1 and the error on stderr.

## The host protocol

An application that runs the compiler as a child process, rather than linking
the library, uses the host protocol (version 1):

```sh
wayfarer compile --protocol --request <request.json> [--parent-stdin]
```

The request is a file of JSON, a `CompileRequest` (above), the same JSON the
C ABI takes:

```json
{"input": "/data/monaco.osm.pbf", "output": "/data/routing.mrt",
 "scratchDir": "/tmp/wayfarer", "graphId": "monaco", "country": "MC", "threads": 4}
```

The protocol's own request is deprecated and will be removed before 1.0. It is
still read, with a warning on stderr, but takes none of the options added since
(`profiles`, ...):

```json
{"inputPbfPath": "/data/monaco.osm.pbf", "tempDir": "/tmp/wayfarer",
 "graphId": "monaco", "countryCode": "MC", "buildProfile": "desktop_full",
 "routeArchivePath": "/data/routing.mrt", "threads": 4}
```

Its fields map to the `CompileRequest`'s: `inputPbfPath` is `input`,
`routeArchivePath` `output`, `tempDir` `scratchDir`, `countryCode` `country`,
and `buildProfile` `"desktop_full"` / `"mobile_operational"` is `memory`
`"standard"` / `"low"`.

With `--parent-stdin` the host holds the child's stdin open for the life of
the job; when it closes (the host quit or crashed) the compile is cancelled,
cleans up and exits with status 130.

The child writes one JSON object per line on stdout; every event carries
`"protocolVersion": 1` and an `"event"`:

| `event` | Fields | When |
|---|---|---|
| `ingest` | `stage` (the ingest pass: `ingest_pass1_index`, `ingest_pass2_ways`, `ingest_pass2_relations`), `batchesRead`, `objectsRead`, `message`, `fraction` (of the pass, or `null`), `percent` | While reading the extract |
| `graph_progress` | `stage` (`finalize_graph`, `build_package`, `cch_order`, `write_package`), `rowsWritten`, `message`, `fraction` (of the stage, or `null`), `percent` | While building the graph and the package |
| `complete` | `graphNodeCount`, `directedEdgeCount`, `packageBytes`, `packageDatasetHash` | Last, on success (exit status 0) |
| `error` | `message` | Last, on failure (exit status 1, or 130 when cancelled) |

`percent` is the share of the whole compile done, from 0 to 1; it never
decreases.

## License

Licensed under the Apache License, Version 2.0 (`LICENSE`). A package compiled
from OpenStreetMap data is a derived database under the Open Database License
1.0; see the `wayfarer-compiler` README.

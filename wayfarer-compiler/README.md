# wayfarer-compiler

Compiles an OpenStreetMap `.osm.pbf` extract into an offline routing package
(`routing.mrt`): a single memory-mappable file that the routing engine opens
and queries. It ingests the PBF, builds and enhances the road graph, bakes the
default edge costs for the wheeled and on-foot profiles (the engine derives
the others), computes landmark distances for
A* search, and writes the package. This crate is the library; the command
line is the `wayfarer` binary (crate `wayfarer-cli`):

```sh
wayfarer compile monaco.osm.pbf -o routing.mrt --country MC
```

## Library

```rust,no_run
use wayfarer_compiler::{CancelToken, CompileRequest, MemoryProfile, compile};

fn main() -> Result<(), wayfarer_compiler::CompileError> {
    let request = CompileRequest::new("monaco.osm.pbf", "routing.mrt")
        .country("MC")
        .memory(MemoryProfile::Standard)
        .threads(2);
    let cancel = CancelToken::new(); // clone it; `cancel()` from any thread
    let result = compile(&request, &cancel, |progress| {
        eprintln!("{} {:.0}%", progress.stage.as_str(), progress.overall * 100.0)
    })?;
    println!("{} bytes, dataset hash {}", result.byte_len, result.dataset_hash);
    Ok(())
}
```

`CompileRequest` is also JSON (camelCase, unknown fields rejected), the form
the `wayfarer compile --request` command and the C ABI take; only `input` and
`output` are required:

```json
{"input": "monaco.osm.pbf", "output": "routing.mrt", "scratchDir": "/tmp",
 "graphId": "monaco", "country": "MC", "memory": "low", "threads": 2}
```

A host that cannot take callbacks (a C ABI, a UI event loop) starts a
`CompileJob` and polls it:

```rust,no_run
use wayfarer_compiler::{CancelToken, CompileJob, CompileRequest};

let cancel = CancelToken::new();
let job = CompileJob::start(CompileRequest::new("monaco.osm.pbf", "routing.mrt"), cancel.clone());
while !job.is_finished() {
    let progress = job.progress(); // the latest report; cheap, any thread
    std::thread::sleep(std::time::Duration::from_millis(200));
}
let result = job.wait();
```

Dropping a job that has not finished cancels it and waits for it to stop.

### In an app's process

The compiler is made to run as a background job inside an app (on iOS no
worker process can be started):

- **Progress.** `on_progress` gets a `CompileProgress` per step, the ingest's
  included: a stage (`CompileStage`: `index_nodes`, `collect_roads`,
  `finalize_graph`, `build_package`, `landmarks`, `write_package`, in that
  order; the ids are stable), the stage's `stage_fraction` where its size is
  known (blobs read, build steps, landmark Dijkstras, bytes written), an
  `overall` share of the compile and an English diagnostic `message` (objects
  read so far, in the ingest). Fractions never go back within a stage; a
  successful compile ends with `write_package` at 1.
- **Cancel.** The `CancelToken` is checked in every stage, inside the
  landmark Dijkstras and between 4 MiB writes; once it is cancelled the call
  returns `CompileError::Cancelled`. A request that cannot run is
  `CompileError::InvalidRequest`, any other failure `CompileError::Failed`;
  `code()` gives a stable id for each (`invalid_request`, `cancelled`,
  `failed`), and the error serializes as `{"code", "message"}`.
- **Files.** The package is written to `<output>.tmp` and renamed into place:
  a package already at `output` is replaced only by a complete one and kept
  when the compile fails or is cancelled. Scratch files go to a fresh
  directory inside `scratch_dir` (default: the output's directory), removed
  whatever the outcome; on cancel and on error the `.tmp` file and any
  directory the compile created are removed too.
- **Replacing an open package on Windows.** Linux and macOS let the rename
  replace a package an engine has open (the engine keeps reading the old
  one). Windows doesn't allow replacing a memory-mapped file, in any
  process: the compile fails with access denied and the old package stays
  intact. An app that recompiles in the field should write to a new path
  and then switch its engine over.
- **Threads.** `threads: Some(n)` runs the compile on its own rayon pool of
  `n` threads (never the global pool) with `n` ingest decode workers plus a
  reader thread. `None` runs on the current pool: call `compile` inside your
  own `ThreadPool::install` to choose thread names, stack sizes or
  priorities. The callback is called from the compile's threads, hence
  `Send`.
- **Nothing global.** The library prints nothing (`WAYFARER_DEBUG` is
  read only with the `debug-env` feature), never exits the process, reads no
  environment or working directory and installs no allocator.
- **Memory.** `MemoryProfile::Low` (JSON `"low"`) uses smaller ingest batches
  and queues and at most two default decode workers; `Standard` (the default)
  is faster. The package's bytes are the same with either profile and any
  thread count.
- **Result.** `CompileResult` gives the package's path, byte length, dataset
  hash, format and costing-model versions, graph id, country, node and edge
  counts and the package build report (`PackageBuildReport`); the road
  graph's own manifest (`graph_manifest`) is there in Rust but not in the
  JSON.

Peak RSS of the release CLI (`ru_maxrss`, one run each, 0.1.0 on an
Apple-silicon Mac with 10 cores, 2026-11-17):

| Extract | Memory | Threads | Peak RSS | Package |
|---|---|---|---|---|
| Monaco (`fixtures/monaco.osm.pbf`, 0.6 MB) | `Standard` | 2 | 32.1 MiB | `7c476af5…`, 995,392 B |
| Monaco | `Low` | 2 | 34.2 MiB | identical |
| Monaco | `Standard` | default (10) | 41.0 MiB | identical |
| Małopolskie (194 MiB PBF, 1.07M nodes, 2.56M edges) | `Standard` | 4 | 1,746 MiB | `3e28f62f…`, 272,076,864 B |
| Małopolskie | `Low` | 4 | 1,717 MiB | identical |

At Monaco's size both profiles keep the node store in RAM and the peak is
mostly the binary, its threads and buffers; the profiles differ in batch and
queue sizes. On a region the peak is the road graph's data, not buffers, so
the two profiles end up close. Inside an app the host's allocator decides how
much freed memory stays resident: on Linux (2026-11-12) the same compile's
live heap peaked at 1,072 MiB, and glibc malloc kept up to 1,513 MiB
resident.

## Status

Version 0.x: the library API may change between minor releases; the
`wayfarer` command line is the stable way to run a build.

- The package layout is versioned (`ROUTING_PACKAGE_FORMAT_VERSION`, 1 today);
  see the routing package format crate for which formats an engine opens.
- Packages record the costing model they were baked with
  (`COSTING_MODEL_VERSION`, 1 today). An engine opens packages of its own
  model and of older ones, re-costing an older model's package from its
  stored costing inputs (slower until it is recompiled); a package of a newer
  model than the engine's fails to open as too new.
- Output is deterministic: a compiler of one release writes the same bytes for
  the same input on every platform, in debug and release builds and at any
  thread count (trigonometry comes from the pure-Rust `libm` crate, not the
  platform's C library).
- The `benchmark-profiling` feature prints phase timings on stderr and
  installs a counting global allocator, for benchmark harnesses; leave it off
  otherwise.

## Data

A package compiled from OpenStreetMap data is a derived database under the Open
Database License 1.0. If you distribute one, credit "© OpenStreetMap
contributors" and offer it under the ODbL; see
<https://www.openstreetmap.org/copyright>. The compiler's own code is
Apache-2.0 and does not change the terms of the data.

## License

Licensed under the Apache License, Version 2.0 (`LICENSE`). Its dependencies
carry their own licenses (Apache-2.0, MIT, zlib and Unicode-3.0).

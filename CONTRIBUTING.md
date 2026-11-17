# Contributing

Issues and pull requests are welcome.

- `scripts/check.sh` passes (formatting, clippy with warnings as errors,
  tests, the Monaco package and the C example).
- A change to search, costing or the package build states its effect: run
  `benchmarks/gates/gates.sh` and report package hashes, dump differences and
  timings against `main`.
- A change to the package layout or to stored costs follows `RELEASING.md`
  (format and costing-model versions).
- Contributions are licensed under the Apache License, Version 2.0, as the
  license's section 5 describes.

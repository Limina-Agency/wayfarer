# JSON Schemas

JSON Schemas (draft 2020-12) of the C ABI's requests and results,
generated from the engine's types by
`wayfarer-ffi/tests/schema.rs`. Hosts can generate their request and
result types from them (Swift, Kotlin, TypeScript, …).

| File | Used by |
|---|---|
| `route-request.json`, `route-result.json` | `wayfarer_route` |
| `matrix-request.json`, `matrix-result.json` | `wayfarer_matrix` |
| `isochrone-request.json`, `isochrone-result.json` | `wayfarer_isochrone` |
| `map-match-request.json`, `map-match-result.json` | `wayfarer_map_match` |
| `nearest-request.json`, `nearest-result.json` | `wayfarer_nearest` |
| `snap-request.json`, `snap-result.json` | `wayfarer_snap` |
| `project-request.json`, `project-result.json` | `wayfarer_project` |
| `navigation-input.json` | `wayfarer_navigation_route_new` (`"input"`) |
| `navigation-sample.json`, `navigation-progress-options.json`, `navigation-progress.json` | `wayfarer_navigation_progress` |
| `overlay-set.json` | overlay sets (`"overlaySets"` in a route request) |
| `compile-request.json`, `compile-progress.json`, `compile-result.json` | `wayfarer_compile_*` (compiler build) and `wayfarer compile --request` |
| `sar-reach-request.json`, `sar-reach-result.json` | `wayfarer_sar_reach` (sar build) and `wayfarer sar reach --request` |
| `sar-teams-request.json`, `sar-teams-result.json` | `wayfarer_sar_plan_teams` (sar build) and `wayfarer sar teams` |
| `sar-coverage-request.json`, `sar-coverage-result.json` | `wayfarer_sar_coverage` (sar build) and `wayfarer sar coverage` |

Every object key is camelCase; enum values are snake_case. The test fails when
a type changes without its schema; after an intended change run

```sh
UPDATE_SCHEMA=1 cargo test -p wayfarer-ffi --test schema
```

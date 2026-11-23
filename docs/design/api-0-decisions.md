# API-0: decisions before the API window

Status: decided on 2026-11-23, against 0.4.0. Each section gives the
decision and the alternative that was not chosen; Anton confirmed every
recommendation (section 8).

This record settles the questions the API lane needs answered before the
redesign window (0.10 – 0.39) opens: who 1.0 is for, which crates are public,
how many release lines there are, how the wire format is versioned, what 1.x
promises, and how deprecations are announced. It changes no code. The
additive preparation that follows from it (builders, `#[deprecated]` names,
`apiVersion` and `warnings` on results) is API-0b, by 0.9.

Inputs: roadmap section 4 and open questions 1, 4, 5 and 6; the API lane
report (`API.md`, written against 0.1.3, its counts still hold unless noted);
and the format-3 decisions of 2026-11-22 (`docs/design/format-3.md`,
section 13), which already fix the package side of the 1.x promise.

## 1. Audience

**Recommendation: field teams first, with a general-purpose API.** 1.0 is
for teams that route offline on laptops, tablets and phones, search and
rescue first among them, and the README already says so. When two items
compete for the same release, the one those teams need wins: elevation,
off-network legs, cross-country movement, Poland on a phone (a 1.x goal,
D12) and SAR relaxations rank above bicycle and truck profiles, transit
stays a non-goal, and an HTTP server stays after 1.0. The API itself is not
SAR-shaped: SAR is one options group on route requests (it is today), and
nothing in the core request, result or error types names SAR, so a
general-purpose host never has to learn it.

Not chosen: a general-purpose offline router as the primary target. It would
put bicycle and truck profiles, turn-by-turn polish and a server ahead of
elevation and off-network legs, which the field teams need and no other
offline router offers.

## 2. Public crate set

**Recommendation:** five public crates, four internal ones, and
`wayfarer-service` folded into the engine.

| Crate | At 1.0 | Contract |
|---|---|---|
| `wayfarer` | public | Rust API (semver-checked), the request and result JSON |
| `wayfarer-compiler` | public | Rust API, the compile request and result JSON |
| `wayfarer-ffi` | public | `wayfarer.h` and the JSON; its Rust items are only the `extern "C"` functions |
| `wayfarer-cli` | public | commands, flags, exit codes and the host protocol; no library |
| `wayfarer-package` | public, cut down | the vocabulary both the engine and the compiler use: `GeoPoint`, the profile type, `CancelToken`, package metadata, verification, errors and format constants |
| `wayfarer-costing` | internal | |
| `wayfarer-road-graph` | internal | |
| `wayfarer-line-network` | internal, or merged into the compiler | |
| `wayfarer-service` | folded into `wayfarer`, then not published | |

- **Why a vocabulary crate.** The engine re-exports `GeoPoint`,
  `MobilityProfile`, `CancelToken`, `GraphViewEdge`, the format constants
  and the costing request types from `wayfarer-package` and
  `wayfarer-costing`, and the compiler re-exports `CancelToken` and
  `MobilityProfile`. Those types have to live in a crate both can depend on,
  and the compiler does not (and should not) depend on the engine. So
  `wayfarer-package` stays public but shrinks to that vocabulary; its
  request, codec, writer and graph record types (`RoadEdge`, `RoadNode`,
  `SharedEdgeInfo`) move to an internal crate. The engine and the compiler
  re-export the vocabulary, so most hosts never name it.
- **Costing becomes internal** once its public request types
  (`RouteCostingOptions`, `RouteCostBreakdown`, `RouteSpeedPreference`,
  `RouteSpeedSource`) are engine-owned, which API-7b does; until then it is
  public in fact. The C library uses it only for `COSTING_MODEL_VERSION`.
- **Road graph becomes internal** once the compiler stops exposing it:
  `CompileResult::graph_manifest` is a `RoadGraphPackageManifest` today
  (API-2b, API-9).
- **Line network** has one user, the compiler, and leaks nothing. It merges
  into the compiler unless the vector basemap compiler (roadmap question 9)
  still needs its render fields; until that is answered it stays a separate
  internal crate.
- **`wayfarer-service`** folds into `wayfarer`: the route workflow and
  overlay sets become `wayfarer::overlay`. Nothing in the workspace uses
  `RoutingService` (the C library and the CLI use only
  `compile_route_workflow` and overlay sets), and no external host is known
  to. It is deprecated in the release that folds the crate (0.12 in the
  API lane plan) and removed one deprecation window later; if a host needs
  its source cache and readiness check, they come back behind a `service`
  feature of `wayfarer`.
- **Internal crates** stay published (cargo requires it), say "internal, no
  stability promise" in their crate docs and README, are depended on with
  exact `=x.y.z` requirements by the public crates, and are excluded from
  `cargo semver-checks`. Exact pins need one release line (section 3).
- **`internals`** (the engine's hidden module) goes behind a non-default
  `unstable-internals` feature (API-2a); the gate harnesses and tests turn it
  on.

Not chosen: keeping all nine crates public (every internal refactor, such as
0.3's columnar road graph, would be a breaking release of a public crate),
or defining the vocabulary in `wayfarer` itself (the compiler would have to
depend on the engine).

## 3. Release lines

**Recommendation: one release line.** Every release gives every crate the
same version, as every release so far has (0.1.3, 0.2.0, 0.3.0 and 0.4.0
were each tagged on both lines at the same commit, from one workspace
version). From the release that changes the release workflow (API-0b, by
0.9), a release is one tag, `v<version>`, and the workflow attaches the
`wayfarer` binaries and the C libraries to the same GitHub release. The
`wayfarer-v*` and `wayfarer-compiler-v*` tags stop there; old tags stay.

A compiler-only fix becomes a patch release of the whole workspace: every
crate is published at `x.y.z+1`, and the changelog says which crates
changed. Republishing an unchanged crate costs a minute of `cargo publish`
and nothing for users.

Why: internal crates shared by the engine and the compiler (package,
costing) can only be pinned exactly by both if they release together. With
two lines a compiler fix would need a new `wayfarer-package` that the engine
line also pins, which makes the "internal" crates public again in practice.
One line is also what users see: one version number, one changelog
section, one set of release notes.

Not chosen: keeping two lines for compiler fixes without engine releases.
In 0.x that never happened, and with exact pins it cannot.

## 4. Wire versioning

**Recommendation: one wire version per incompatible revision, not per
request type and not per release.**

- `apiVersion` is an integer. Requests may send it from API-0b (by 0.9) and
  must send it from API-4a (0.15 in the API lane plan); every result echoes
  the version the engine answered in.
- During the window it increases only in the releases that break the wire
  (API-4, API-8, API-7a, …). At 1.0 it is `1` and stays `1` through 1.x,
  because 1.x changes the wire only additively.
- An engine answers exactly one version. A request with another version
  fails with `unsupported_api_version`, naming the version the engine
  speaks. There is no multi-version serving: within 1.x it is not needed,
  and in 0.x the migration notes cover each step.
- The C ABI keeps its own version (`WAYFARER_ABI_VERSION`, 1 today, 2 once
  with FFI-4), and so do the package format and the costing model; none of
  them follows `apiVersion`.
- Schemas are published per `apiVersion` under `schema/v<N>/` from API-4a.
- Unknown keys: a `warnings` entry in the result from API-0b (soft deny),
  rejected at every level of every request from API-4c. Additive 1.x fields
  are then refused cleanly by older 1.x engines, which is what a host
  sending a new option wants. Result objects stay open: clients ignore
  unknown keys and treat unknown enum values as `unknown` (API-4d).

Not chosen: `apiVersion` per request type (every request type would carry
its own history, and hosts would track a table of versions), or one version
per library release (a version bump every week that says nothing about
compatibility).

## 5. The 1.x promise

**Recommendation:** within 1.x,

| Surface | Promise |
|---|---|
| Rust API of the public crates | semver: no breaking change before 2.0; `cargo semver-checks` blocking from the 0.90 freeze |
| Request and result JSON | additive only: new optional request fields, new result fields and enum values (clients tolerate unknowns, section 4); `apiVersion` stays `1` |
| C ABI | additive only: new functions, never a changed signature or struct layout; `WAYFARER_ABI_VERSION` stays 2 |
| Packages | every 1.x engine reads every package a 1.x compiler wrote; format 3 frozen at 1.0; a new REQUIRED section only when the compile request asks for the feature that needs it, with the oldest engine that reads it named in the release notes (format-3 decision) |
| Costing model | versioned separately; a new model never stops an older package from opening (it is re-costed, as today) |
| Determinism | the same input and options give the same package bytes on every platform; a 1.x minor that changes the bytes (a compiler fix) says so in its notes |
| MSRV | raised only in a minor release, to a Rust release at least six months old, and named in the changelog |
| CLI | commands, flags and exit codes additive only; the host protocol request is the `CompileRequest` JSON |

Results may change within 1.x when a fix makes them more correct (a route
that ignored a restriction, an isochrone that stopped early); the release
notes say which requests change. Diagnostics under the unstable `debug` key
(API-8) carry no promise.

Not chosen: an N-1 package window (each engine reads only the previous
format). It would make every format change a forced recompile for teams in
the field, and format 3's optional sections already let 1.x add data
without a new format.

## 6. Deprecations

**Recommendation:** a deprecated item keeps working, warns, and is removed
no earlier than **two releases and four weeks** after the release that
deprecated it, whichever is later. Releases come days apart now (0.2.0 to
0.4.0 took four days), so a one-release window could be a weekend.

How each surface warns:

| Surface | Warning | Removal |
|---|---|---|
| Rust | `#[deprecated(since, note)]` on the item; renamed types as deprecated type aliases, since a deprecated `pub use` does not warn | after the window |
| Request JSON | a `warnings` entry in the result: `{code: "deprecated", path, message}` (also `alias` for an alias spelling and `unknown_key` before API-4c) | after the window; the key or spelling is then rejected as unknown |
| C ABI | the function stays exported and documented as deprecated in `wayfarer.h` | with the next ABI version only |
| CLI and host protocol | a line on stderr (stdout carries the protocol), as #76 does for the old protocol request | after the window |

Every deprecation is listed under "Deprecated" in the changelog with the
release that removes it, and every removal under "Removed" with what replaces
it. Within 1.x nothing public is removed: deprecated items stay until 2.0.

The `warnings` list ships in API-0b, together with `apiVersion` on results,
so hosts can start logging it before the first wire deprecation.

Not chosen: one release (too short at this release rate), or a fixed number
of weeks alone (a slow month would keep dead names around for one release
only in practice).

## 7. What follows

- **API-0b (by 0.9):** builders and constructors for every request struct;
  `#[deprecated]` on the legacy Rust names (`MobilityProfile::Default`,
  `ssvg*`, the legacy constants); `apiVersion` and `warnings` on results;
  `RouteError::code()` as the one code table; the release workflow on one
  `v<version>` tag.
- **QA-1** already runs `cargo semver-checks`, the schema diff and the C
  header checks on every pull request (informational until 0.90).
- **The window (0.10 – 0.39)** follows the API lane plan: non-exhaustive
  types (0.10), legacy names (0.11), the crate set (0.12), modules (0.13),
  errors (0.14), wire rules (0.15), lean results (0.16), time (0.17), then
  the overlay and profile items after stable edge references.

## 8. Decisions

Anton confirmed every recommendation above on 2026-11-23.

- **Audience:** field teams first, search and rescue among them, with a
  general-purpose API; SAR is one options group (section 1).
- **Crates:** public `wayfarer`, `wayfarer-compiler`, `wayfarer-ffi`,
  `wayfarer-cli` and a cut-down `wayfarer-package`; costing, road graph and
  line network internal and pinned exactly; `wayfarer-service` folded into
  `wayfarer` (section 2).
- **One release line**, one `v<version>` tag per release from API-0b
  (section 3).
- **Wire:** one `apiVersion` per incompatible revision, `1` through 1.x, one
  version per engine (section 4).
- **1.x promise** as in section 5's table.
- **Deprecations:** two releases and four weeks, whichever is later; the
  warning per surface as in section 6.

/*
 * Wayfarer C ABI, version 1.
 *
 * Open a routing package (`routing.mrt`), then send route (two or more
 * locations, with overlays), matrix, isochrone, map matching, snap and
 * projection requests as JSON and get JSON back; navigation progress runs on
 * a prepared-route handle. The request and result shapes are the engine's
 * types, named below; `schema/` in the source tree and release archives has
 * a JSON Schema for each.
 *
 * Status codes. Every call returns one. On success the out string holds the
 * result; on failure it holds {"code": "...", "message": "..."} (route
 * failures add "diagnostics"). Out-string pointers may be NULL when the
 * caller does not want the text.
 *
 * Memory ownership.
 *   - Input strings (paths, JSON) are borrowed for the call only; the library
 *     keeps no pointer to them.
 *   - Every string the library returns (out strings, wayfarer_versions)
 *     belongs to the caller, who frees it once with wayfarer_string_free
 *     (never with free()).
 *   - Handles (WayfarerPackage, WayfarerCancelToken,
 *     WayfarerNavigationRoute) belong to the caller, who frees each once
 *     with its own function: wayfarer_close,
 *     wayfarer_cancel_token_free, wayfarer_navigation_route_free.
 *     Free functions ignore NULL. On failure no handle is written.
 *
 * Paths and text. Every char string in and out is UTF-8, paths included.
 *   On Windows a C program's argv and the Win32 "A" functions use the ANSI
 *   code page instead, which cannot spell every path: take the arguments
 *   from wmain and convert them with WideCharToMultiByte(CP_UTF8, ...) (as
 *   examples/route.c does), or pass UTF-16 paths to wayfarer_open_w and
 *   wayfarer_verify_w.
 *
 * Replacing a package. Packages are written to "<path>.tmp" and moved over
 *   the old file, so a package can be recompiled or downloaded onto the path
 *   of one that is open: the open handle keeps reading the old package, and
 *   the next wayfarer_open gets the new one. Where Windows can't rename
 *   with POSIX semantics (FAT32, exFAT, some network shares), a rename over
 *   an open, memory-mapped package fails, so there the old file is renamed
 *   aside to "<path>.replaced-<unique>" and deleted once no handle has it
 *   open, at the latest by the next replacement of the same path. A host
 *   that writes packages itself does the same, or writes each version to a
 *   new path and opens that.
 *
 * Thread safety.
 *   - Every function may be called from any thread.
 *   - A package handle, a cancel token and a navigation route may each be used
 *     by several calls at once on different threads; none of them is changed
 *     by a request. Requests on one package run in parallel.
 *   - wayfarer_cancel_token_cancel may be called from any thread while
 *     requests using the token run; that is its purpose.
 *   - Free or close a handle only after every call using it has returned.
 *
 * Cancellation. A cancel token stops every request it is passed
 * to: a request in flight stops within one cancellation check (every 1024
 * edges a search visits, tens of microseconds), and a request started after
 * cancellation fails at once. The call then returns WAYFARER_CANCELLED
 * with "code": "cancelled", or "deadline_exceeded" once a token's timeout has
 * passed. Cancelling cannot be undone: use a new token per request (or per
 * batch of requests to stop together). The cancel parameter may be NULL.
 *
 * JSON casing. Every object key is camelCase ("distanceM",
 *   "maxVisitedEdgeCount", "locationMatches"); enum values are snake_case
 *   ("wheeled", "time_seconds", "no_go", "routed"). Request options
 *   (RouteOptions, RouteEndpointOptions, RouteSarOptions) also accept their
 *   keys in snake_case. GeoPoint is {"lat", "lng"}. A maneuver's
 *   "instructionParameters" is a map of template parameter names, which are
 *   snake_case like enum values ("road_name", "turn_angle_deg").
 *
 * Licensed under the Apache License, Version 2.0.
 */
#ifndef WAYFARER_H
#define WAYFARER_H

#include <stdint.h>
#ifdef _WIN32
#include <wchar.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define WAYFARER_ABI_VERSION 1

/* Success: the out string holds the result. */
#define WAYFARER_OK 0
/* A NULL pointer, a string that is not UTF-8, JSON that does not parse as
 * the request ("code": "invalid_argument"), options the engine rejects
 * ("invalid_request", e.g. a SAR surcharge below the package minimum or SAR
 * for another profile than wheeled), or an overlay set that does not
 * validate or match its content hash ("invalid_overlay_set"). */
#define WAYFARER_INVALID_ARGUMENT 1
/* The package could not be opened or verified ("code": "not_found", "io",
 * "invalid_package", "outdated_package", "package_too_new", "unsupported"). */
#define WAYFARER_PACKAGE_ERROR 2
/* The request was valid but has no answer ("code": "no_candidate", "no_path",
 * "disconnected", "search_budget_exceeded", ...), or asks for a SAR
 * relaxation the package has no data for ("sar_unavailable", with
 * "relaxation": "legal_access", "wrong_way", "barriers" or
 * "vehicle_on_paths"; see the info's "sarCapability"). */
#define WAYFARER_ROUTE_ERROR 3
/* A bug in the library (a caught panic). */
#define WAYFARER_INTERNAL_ERROR 4
/* Compiler build: a compile failed, e.g. the input could not be
 * read or the output written ("code": "failed"). */
#define WAYFARER_COMPILE_ERROR 6
/* The request's cancel token was cancelled ("code": "cancelled")
 * or its timeout passed ("deadline_exceeded"). Only calls taking a token
 * return it. */
#define WAYFARER_CANCELLED 5

/* An opened routing package. */
typedef struct WayfarerPackage WayfarerPackage;
/* A cancel token, with an optional deadline. */
typedef struct WayfarerCancelToken WayfarerCancelToken;
/* A route prepared for navigation progress. */
typedef struct WayfarerNavigationRoute WayfarerNavigationRoute;

/* The version of this ABI (WAYFARER_ABI_VERSION). */
uint32_t wayfarer_abi_version(void);

/* The library's versions as JSON: {"library", "abi", "readableFormats":
 * [min, max], "costingModel"}. Free with wayfarer_string_free. */
char *wayfarer_versions(void);

/* Opens the package at `path` (UTF-8, also on Windows: see "Paths and
 * text"). `options_json` is NULL or {"memoryProfile": "desktop" |
 * "mobile"}. On success `*package` is the handle and `*out_json` the package
 * info. */
int32_t wayfarer_open(const char *path,
                              const char *options_json,
                              WayfarerPackage **package,
                              char **out_json);

#ifdef _WIN32
/* Windows only: wayfarer_open with a UTF-16 path, as wmain and the Win32 "W"
 * functions give it. */
int32_t wayfarer_open_w(const wchar_t *path,
                                const char *options_json,
                                WayfarerPackage **package,
                                char **out_json);
#endif

/* Closes a handle. NULL is ignored. */
void wayfarer_close(WayfarerPackage *package);

/* The package's info: format and costing versions, counts, dataset hash,
 * quality, and "sarCapability" (the SAR relaxations "available" and
 * "unavailable" on it, and on a package with SAR data the least
 * "legalAccessMinFactor", "wrongWayMinFactor" and
 * "vehicleOnPathsMinFactor" a request may set). */
int32_t wayfarer_info(const WayfarerPackage *package, char **out_json);

/* Checks a package file end to end (every section against its checksum).
 * `path` is UTF-8. */
int32_t wayfarer_verify(const char *path, char **out_json);

#ifdef _WIN32
/* Windows only: wayfarer_verify with a UTF-16 path. */
int32_t wayfarer_verify_w(const wchar_t *path, char **out_json);
#endif

/* Routes a RouteRequest: {"locations": [{"point": GeoPoint, "options":
 * RouteEndpointOptions (optional)}, ...] (2 to 34), "profile", "options":
 * RouteOptions, "legs": ["routed" | "direct", ...] (optional, one per pair
 * of locations), "overlays": [RouteOverlay] (closures and other overlays as
 * geometry or edges), "departureAtMs", "overlaySets", "mobility"} gives a
 * RouteResult: "segments" (one per leg), "locationMatches", "geometry",
 * "maneuvers", "distanceM", "durationS", "cost", "overlayUses",
 * "explanations", ..., with "sourceId" (the graph id) and "revision" (the
 * dataset hash). A failed leg is a route error with its "segmentIndex" and
 * "diagnostics". */
int32_t wayfarer_route(const WayfarerPackage *package,
                               const char *request_json,
                               char **out_json);

/* Answers a RouteMatrixRequest (unreachable cells are reported per cell). A
 * request of more than 250,000 cells, or with invalid coordinates or
 * options, fails with "request_too_large" or "invalid_request". */
int32_t wayfarer_matrix(const WayfarerPackage *package,
                                const char *request_json,
                                char **out_json);

/* Answers a RouteIsochroneRequest. */
int32_t wayfarer_isochrone(const WayfarerPackage *package,
                                   const char *request_json,
                                   char **out_json);

/* Frees a string the library returned. NULL is ignored. */
void wayfarer_string_free(char *text);

/* ---- Cancellable requests, map matching, snapping, navigation ----------- */

/* A new cancel token (never NULL). With a non-zero `timeout_ms` it also stops
 * requests once that many milliseconds have passed from now. Free with
 * wayfarer_cancel_token_free. */
WayfarerCancelToken *wayfarer_cancel_token_new(uint64_t timeout_ms);

/* Cancels the token: requests using it stop with WAYFARER_CANCELLED.
 * Callable from any thread, any number of times. NULL is ignored. */
void wayfarer_cancel_token_cancel(const WayfarerCancelToken *cancel);

/* Frees a token once no call using it is in flight. NULL is ignored. */
void wayfarer_cancel_token_free(WayfarerCancelToken *cancel);

/* wayfarer_route, wayfarer_matrix and
 * wayfarer_isochrone, stopped by `cancel` (may be NULL). A cancelled
 * matrix is an error, not a result of failed cells. */
int32_t wayfarer_route_with_cancel(const WayfarerPackage *package,
                                           const char *request_json,
                                           const WayfarerCancelToken *cancel,
                                           char **out_json);
int32_t wayfarer_matrix_with_cancel(const WayfarerPackage *package,
                                            const char *request_json,
                                            const WayfarerCancelToken *cancel,
                                            char **out_json);
int32_t wayfarer_isochrone_with_cancel(const WayfarerPackage *package,
                                               const char *request_json,
                                               const WayfarerCancelToken *cancel,
                                               char **out_json);

/* Matches a GPS trace, {"samples": [NavigationPositionSample], "profile",
 * "options": MapMatchOptions (optional)}, to the road network with a
 * MapMatchResult; "code": "no_match" when no sample matches. `cancel` is
 * checked before each sample's candidates and each matching step. */
int32_t wayfarer_map_match(const WayfarerPackage *package,
                                   const char *request_json,
                                   const WayfarerCancelToken *cancel,
                                   char **out_json);

/* Snaps a point onto the roads a profile may use: {"point", "profile",
 * "maxDistanceM" (optional), "candidateLimit" (optional), "options":
 * RouteEndpointOptions (optional)} gives a SnapResult, {"candidates",
 * "rejectedCandidateCount", "mode"}, best candidate first. No candidate is
 * not an error; "maxDistanceM" above 10,000 or "candidateLimit" above 64 is
 * ("request_too_large"). */
int32_t wayfarer_snap(const WayfarerPackage *package,
                              const char *request_json,
                              char **out_json);

/* Projects geometry onto the package's edges, as a route overlay with that
 * geometry would be: {"geometryKind": "point" | "line" | "polygon",
 * "points", "projectionDistanceM"} gives a RouteOverlayGeometryProjection
 * ({"edgeIds", "edgeRanges", ...}), for closures a host stores by edge.
 * "projectionDistanceM" is at most 100,000, as for route overlays. */
int32_t wayfarer_project(const WayfarerPackage *package,
                                 const char *request_json,
                                 char **out_json);

/* Prepares a route for navigation progress from {"route": RouteResult} (as
 * wayfarer_route returns it) or {"input": RouteNavigationInput}. On
 * success `*route` is the handle and `*out_json` {"geometryPointCount",
 * "maneuverCount"}. It needs no package. */
int32_t wayfarer_navigation_route_new(const char *route_json,
                                              WayfarerNavigationRoute **route,
                                              char **out_json);

/* Progress along a prepared route: {"sample": NavigationPositionSample,
 * "previous": RouteNavigationProgress (the last result on this route;
 * omitted for the first sample), "options": RouteNavigationProgressOptions
 * (optional)} gives a RouteNavigationProgress. The handle keeps no state
 * between samples. */
int32_t wayfarer_navigation_progress(const WayfarerNavigationRoute *route,
                                             const char *request_json,
                                             char **out_json);

/* Frees a prepared route once no call on it is in flight. NULL is ignored. */
void wayfarer_navigation_route_free(WayfarerNavigationRoute *route);

/* ---- Compiling on the device (compiler build only) ---------------------- */

/* The calls below exist only in the library's compiler build
 * (libwayfarer_ffi built with the `compiler` feature, released as
 * wayfarer-ffi-compiler-*); wayfarer_versions reports
 * "compiler": true there. A compile runs on a thread the job owns; the host
 * polls its progress (from a UI timer, say) and waits for the result. */

/* A compile in progress. */
typedef struct WayfarerCompileJob WayfarerCompileJob;

/* Starts compiling a CompileRequest: {"input": "<extract.osm.pbf>",
 * "output": "<routing.mrt>", "scratchDir" (optional; a fresh subdirectory is
 * made and removed), "graphId" (default: from the input's name), "country"
 * (ISO code), "memory": "standard" | "low" (memory use and speed, never the
 * package's bytes), "threads", "profiles" (the profiles whose costs are
 * stored, default ["wheeled", "on_foot"]; see schema/compile-request.json)}.
 * `cancel` (may be NULL) stops it, its timeout
 * included; a cancelled or failed compile removes its scratch and partial
 * output and keeps any package that was at "output" before. A package open
 * at "output" is replaced (see "Replacing a package"). On success
 * `*job` is the handle and `*out_json` the first progress. */
int32_t wayfarer_compile_start(const char *request_json,
                                       const WayfarerCancelToken *cancel,
                                       WayfarerCompileJob **job,
                                       char **out_json);

/* The latest progress: {"stage": "collect_roads" | "index_nodes" |
 * "finalize_graph" | "build_package" | "cch_order" | "write_package",
 * "stageFraction" (0 to 1, when known), "overall" (0 to 1), "message"}. */
int32_t wayfarer_compile_progress(const WayfarerCompileJob *job,
                                          char **out_json);

/* 1 once the job has ended (succeeded, failed or been cancelled), else 0. */
int32_t wayfarer_compile_is_finished(const WayfarerCompileJob *job);

/* Waits for the job to end. On success `*out_json` is the CompileResult
 * ({"package", "byteLen", "datasetHash", "formatVersion",
 * "costingModelVersion", "graphId", "country", "nodeCount", "edgeCount",
 * "report"}); otherwise an error with WAYFARER_CANCELLED,
 * WAYFARER_INVALID_ARGUMENT or WAYFARER_COMPILE_ERROR.
 * Calling it again returns the same outcome. */
int32_t wayfarer_compile_wait(WayfarerCompileJob *job, char **out_json);

/* Frees a job once no other call on it is in flight; a running compile is
 * cancelled and waited for first. NULL is ignored. */
void wayfarer_compile_free(WayfarerCompileJob *job);

#ifdef __cplusplus
}
#endif

#endif /* WAYFARER_H */

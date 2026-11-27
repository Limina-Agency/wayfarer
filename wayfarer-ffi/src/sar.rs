//! Search and rescue planning (the `sar` feature): the three steps of
//! `wayfarer-sar` on an open package, JSON in and JSON out like every other
//! request.

use std::ffi::c_char;

use wayfarer_sar::{CoverageRequest, ReachRequest, SarError, TeamPlanRequest};

use super::{
    Failure, WayfarerCancelToken, WayfarerPackage, cancellation, engine, read_json, respond,
};

impl Failure {
    fn sar(error: &SarError) -> Self {
        match error {
            SarError::InvalidRequest { .. } => Self::new(
                super::WAYFARER_INVALID_ARGUMENT,
                "invalid_request",
                error.to_string(),
            ),
            SarError::Route(route) => Self::route(route),
            _ => Self::new(
                super::WAYFARER_INTERNAL_ERROR,
                "internal",
                error.to_string(),
            ),
        }
    }
}

/// Predicts where a missing person could be from `request_json`, a
/// `ReachRequest` (`{"lastKnown", "uncertaintyM", "elapsedS" |
/// "lastSeenAtMs" + "atMs", "profile", "pace", "zones", "destinations",
/// "maxEdges", ...}`; see `schema/sar-reach-request.json`). `*out_json` is
/// the `ReachResult`: the fast, typical and slow person's reach, the
/// probability per edge, likely routes and corridors. `cancel` (null:
/// never) stops it with `WAYFARER_CANCELLED`.
///
/// # Safety
/// As for `wayfarer_route`; `cancel` is null or a live token.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_sar_reach(
    package: *const WayfarerPackage,
    request_json: *const c_char,
    cancel: *const WayfarerCancelToken,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe {
        respond(out_json, || {
            let engine = engine(package)?;
            let mut request: ReachRequest = read_json(request_json, "request")?;
            request.options.cancellation = cancellation(cancel);
            wayfarer_sar::predict_reach(engine, &request).map_err(|error| Failure::sar(&error))
        })
    }
}

/// Plans teams onto search segments from `request_json`, a
/// `TeamPlanRequest` (`{"teams", "segments", "probability", "shiftS",
/// ...}`; see `schema/sar-teams-request.json`). `*out_json` is the
/// `TeamPlanResult`: arrival times, a first assignment and each team's
/// territory. `cancel` (null: never) stops it with `WAYFARER_CANCELLED`.
///
/// # Safety
/// As for `wayfarer_route`; `cancel` is null or a live token.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_sar_plan_teams(
    package: *const WayfarerPackage,
    request_json: *const c_char,
    cancel: *const WayfarerCancelToken,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe {
        respond(out_json, || {
            let engine = engine(package)?;
            let mut request: TeamPlanRequest = read_json(request_json, "request")?;
            request.options.cancellation = cancellation(cancel);
            wayfarer_sar::plan_teams(engine, &request).map_err(|error| Failure::sar(&error))
        })
    }
}

/// Applies team tracks and host-measured search effort to a probability map
/// from `request_json`, a `CoverageRequest` (`{"tracks", "efforts",
/// "probability", "segments", "defaultPod"}`;
/// see `schema/sar-coverage-request.json`). `*out_json` is the
/// `CoverageResult`: the covered edges, per-segment coverage and detection,
/// and the updated probability. A track that matches no road is reported in
/// the result, not as an error.
///
/// # Safety
/// As for `wayfarer_route`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_sar_coverage(
    package: *const WayfarerPackage,
    request_json: *const c_char,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe {
        respond(out_json, || {
            let engine = engine(package)?;
            let request: CoverageRequest = read_json(request_json, "request")?;
            wayfarer_sar::apply_coverage(engine, &request).map_err(|error| Failure::sar(&error))
        })
    }
}

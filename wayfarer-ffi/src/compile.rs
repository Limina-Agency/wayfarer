//! Compiling a routing package from an `.osm.pbf` extract on a job the host
//! polls (the `compiler` feature): start, read progress, wait, free.

use std::ffi::c_char;
use std::sync::{Mutex, PoisonError};
use std::time::Instant;

use wayfarer_compiler::{CancelToken, CompileError, CompileJob, CompileProgress, CompileRequest};

use super::{
    Failure, WAYFARER_CANCELLED, WAYFARER_INVALID_ARGUMENT, WayfarerCancelToken, read_json, respond,
};

/// The compile failed: the input could not be read or parsed, the output
/// could not be written ("code": "failed").
pub const WAYFARER_COMPILE_ERROR: i32 = 6;

/// A compile running on a thread of its own. Opaque to C.
pub struct WayfarerCompileJob {
    job: Mutex<Option<CompileJob>>,
    /// The last progress, kept when `wait` takes the job.
    last: Mutex<Option<CompileProgress>>,
    /// The outcome, as JSON with its status, once `wait` returned it.
    outcome: Mutex<Option<(i32, serde_json::Value)>>,
    /// Stops the deadline watcher when the job ends.
    done: CancelToken,
}

impl Failure {
    fn compile(error: &CompileError) -> Self {
        let status = match error {
            CompileError::InvalidRequest(_) => WAYFARER_INVALID_ARGUMENT,
            CompileError::Cancelled => WAYFARER_CANCELLED,
            CompileError::Failed(_) => WAYFARER_COMPILE_ERROR,
        };
        Self::new(status, error.code(), error.to_string())
    }
}

/// Starts compiling `request_json`, a `CompileRequest` (`{"input",
/// "output", "scratchDir", "graphId", "country", "memory": "standard" |
/// "low", "threads"}`), on a thread the job owns. `cancel` (may be null)
/// stops it, its timeout included. On success `*job` is the handle and
/// `*out_json` (if not null) the first progress.
///
/// # Safety
/// `request_json` is null or a NUL-terminated string; `cancel` is null or a
/// live token; `job` is null or valid for one write; `out_json` is null or
/// valid for one write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_compile_start(
    request_json: *const c_char,
    cancel: *const WayfarerCancelToken,
    job: *mut *mut WayfarerCompileJob,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe {
        respond(out_json, || {
            if job.is_null() {
                return Err(Failure::invalid("job out-pointer is null"));
            }
            let request: CompileRequest = read_json(request_json, "request")?;
            let (token, deadline) = if cancel.is_null() {
                (CancelToken::new(), None)
            } else {
                // SAFETY: a live token per the contract.
                let cancel = &*cancel;
                (cancel.token.clone(), cancel.cancellation.deadline())
            };
            if token.is_cancelled() {
                return Err(Failure::compile(&CompileError::Cancelled));
            }
            let done = CancelToken::new();
            if let Some(deadline) = deadline {
                watch_deadline(deadline, token.clone(), done.clone());
            }
            let started = CompileJob::start(request, token);
            let progress = started.progress();
            // SAFETY: checked non-null; valid for one write per the contract.
            *job = Box::into_raw(Box::new(WayfarerCompileJob {
                job: Mutex::new(Some(started)),
                last: Mutex::new(None),
                outcome: Mutex::new(None),
                done,
            }));
            Ok(progress)
        })
    }
}

/// Cancels `token` at `deadline`, unless `done` is cancelled first.
fn watch_deadline(deadline: Instant, token: CancelToken, done: CancelToken) {
    std::thread::spawn(move || {
        while !done.is_cancelled() {
            let now = Instant::now();
            if now >= deadline {
                token.cancel();
                return;
            }
            std::thread::sleep((deadline - now).min(std::time::Duration::from_millis(100)));
        }
    });
}

/// The job's latest progress, `{"stage", "stageFraction", "overall",
/// "message"}`, cheap enough for a UI timer. Callable from any thread while
/// another waits.
///
/// # Safety
/// `job` is a live handle; `out_json` is null or valid for one write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_compile_progress(
    job: *const WayfarerCompileJob,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe {
        respond(out_json, || {
            let job = job
                .as_ref()
                .ok_or_else(|| Failure::invalid("job is null"))?;
            let running = job.job.lock().unwrap_or_else(PoisonError::into_inner);
            let progress = match running.as_ref() {
                Some(compile) => compile.progress(),
                None => job
                    .last
                    .lock()
                    .unwrap_or_else(PoisonError::into_inner)
                    .clone()
                    .unwrap_or_else(CompileProgress::starting),
            };
            Ok(progress)
        })
    }
}

/// Whether the job has ended (succeeded, failed or been cancelled): 1 or 0.
/// A null handle is 0.
///
/// # Safety
/// `job` is null or a live handle.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_compile_is_finished(job: *const WayfarerCompileJob) -> i32 {
    let Some(job) = (unsafe { job.as_ref() }) else {
        return 0;
    };
    let running = job.job.lock().unwrap_or_else(PoisonError::into_inner);
    i32::from(running.as_ref().is_none_or(CompileJob::is_finished))
}

/// Waits for the job to end. On success `*out_json` is the `CompileResult`
/// (the package's path, size, dataset hash, versions, counts and build
/// report); otherwise an error: "cancelled" (WAYFARER_CANCELLED),
/// "invalid_request" (WAYFARER_INVALID_ARGUMENT) or "failed"
/// (WAYFARER_COMPILE_ERROR). Calling it again returns the same
/// outcome.
///
/// # Safety
/// `job` is a live handle; `out_json` is null or valid for one write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_compile_wait(
    job: *mut WayfarerCompileJob,
    out_json: *mut *mut c_char,
) -> i32 {
    unsafe {
        let Some(handle) = job.as_ref() else {
            return respond(out_json, || -> Result<(), Failure> {
                Err(Failure::invalid("job is null"))
            });
        };
        let mut outcome = handle
            .outcome
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        if outcome.is_none() {
            let running = handle
                .job
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
                .take();
            if let Some(compile) = &running {
                // The final progress, kept for `wayfarer_compile_progress`
                // once `wait` has taken the job.
                while !compile.is_finished() {
                    std::thread::sleep(std::time::Duration::from_millis(20));
                }
                *handle.last.lock().unwrap_or_else(PoisonError::into_inner) =
                    Some(compile.progress());
            }
            let result = running.map(CompileJob::wait);
            handle.done.cancel();
            *outcome = Some(match result {
                Some(Ok(result)) => (
                    super::WAYFARER_OK,
                    serde_json::to_value(result).expect("serializes"),
                ),
                Some(Err(error)) => {
                    let failure = Failure::compile(&error);
                    (failure.status, failure.error)
                }
                None => {
                    let failure = Failure::new(
                        super::WAYFARER_INTERNAL_ERROR,
                        "internal",
                        "the job's result is gone",
                    );
                    (failure.status, failure.error)
                }
            });
        }
        let (status, value) = outcome.clone().expect("set above");
        respond(out_json, move || {
            if status == super::WAYFARER_OK {
                Ok(value)
            } else {
                Err(Failure {
                    status,
                    error: value,
                })
            }
        })
    }
}

/// Frees a job. A job that has not ended is cancelled first and waited for
/// (it removes its scratch and partial output). NULL is ignored.
///
/// # Safety
/// `job` is null or a live handle no other call is using.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wayfarer_compile_free(job: *mut WayfarerCompileJob) {
    if job.is_null() {
        return;
    }
    // SAFETY: a live handle per the contract, freed once.
    let job = unsafe { Box::from_raw(job) };
    job.done.cancel();
    drop(job);
}

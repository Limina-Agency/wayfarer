//! A compile on a thread of its own, for hosts that poll instead of taking
//! callbacks (the C ABI, UI event loops).

use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;

use wayfarer_package::CancelToken;

use crate::{CompileError, CompileProgress, CompileRequest, CompileResult, compile};

/// A compile running on a thread the job owns.
///
/// ```no_run
/// use wayfarer_compiler::{CancelToken, CompileJob, CompileRequest};
///
/// let cancel = CancelToken::new();
/// let job = CompileJob::start(CompileRequest::new("monaco.osm.pbf", "routing.mrt"), cancel.clone());
/// while !job.is_finished() {
///     let progress = job.progress();
///     println!("{} {:.0}%", progress.stage.as_str(), progress.overall * 100.0);
///     std::thread::sleep(std::time::Duration::from_millis(200));
/// }
/// let result = job.wait();
/// ```
///
/// Dropping a job that has not finished cancels it and waits for it to stop
/// (within about a second; it removes its scratch and partial files first).
pub struct CompileJob {
    cancel: CancelToken,
    progress: Arc<Mutex<CompileProgress>>,
    state: State,
}

enum State {
    Running(JoinHandle<Result<CompileResult, CompileError>>),
    /// The thread could not be started, or `wait` took the result.
    Done(Option<Box<Result<CompileResult, CompileError>>>),
}

impl CompileJob {
    /// Starts compiling `request` on a new thread. `cancel` (or a clone of it)
    /// stops the compile; [`CompileJob::wait`] then returns
    /// [`CompileError::Cancelled`].
    pub fn start(request: CompileRequest, cancel: CancelToken) -> CompileJob {
        let progress = Arc::new(Mutex::new(CompileProgress::starting()));
        let shared = Arc::clone(&progress);
        let token = cancel.clone();
        let spawned = std::thread::Builder::new()
            .name("route-compile-job".into())
            .spawn(move || {
                compile(&request, &token, |report| {
                    *shared.lock().unwrap_or_else(PoisonError::into_inner) = report.clone();
                })
            });
        let state = match spawned {
            Ok(handle) => State::Running(handle),
            Err(error) => State::Done(Some(Box::new(Err(CompileError::Failed(format!(
                "Failed to start the compile's thread: {error}"
            )))))),
        };
        CompileJob {
            cancel,
            progress,
            state,
        }
    }

    /// The latest progress report (cheap; any thread).
    pub fn progress(&self) -> CompileProgress {
        self.progress
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    /// Whether the compile has ended (succeeded, failed or stopped after a
    /// cancel), so that [`CompileJob::wait`] returns at once.
    pub fn is_finished(&self) -> bool {
        match &self.state {
            State::Running(handle) => handle.is_finished(),
            State::Done(_) => true,
        }
    }

    /// Cancels the compile (as cancelling its token does).
    pub fn cancel(&self) {
        self.cancel.cancel();
    }

    /// Waits for the compile to end and returns its outcome.
    pub fn wait(mut self) -> Result<CompileResult, CompileError> {
        self.take_result()
    }

    fn take_result(&mut self) -> Result<CompileResult, CompileError> {
        let state = std::mem::replace(&mut self.state, State::Done(None));
        match state {
            State::Running(handle) => handle.join().unwrap_or_else(|panic| {
                let message = panic
                    .downcast_ref::<&str>()
                    .map(|text| text.to_string())
                    .or_else(|| panic.downcast_ref::<String>().cloned())
                    .unwrap_or_else(|| "unknown panic".into());
                Err(CompileError::Failed(format!(
                    "The compile stopped on an internal error: {message}"
                )))
            }),
            State::Done(result) => result.map(|result| *result).unwrap_or_else(|| {
                Err(CompileError::Failed(
                    "The compile's result was taken".into(),
                ))
            }),
        }
    }
}

impl Drop for CompileJob {
    fn drop(&mut self) {
        if let State::Running(handle) = &self.state {
            if !handle.is_finished() {
                self.cancel.cancel();
            }
            let _ = self.take_result();
        }
    }
}

impl std::fmt::Debug for CompileJob {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CompileJob")
            .field("progress", &self.progress())
            .field("finished", &self.is_finished())
            .finish()
    }
}

//! A terrain compile on a thread of its own, for hosts that poll instead of taking callbacks
//! (UI event loops, a C ABI).

use crate::{CancelToken, Error, Options, Progress, Region, Report, Result, Sources};
use std::path::PathBuf;
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;

/// A [`compile_to_file`](crate::compile_to_file) running on a thread the job owns.
///
/// ```no_run
/// use wayfarer_terrain_compiler::{CancelToken, Options, Region, Sources, TerrainJob};
///
/// let sources = Sources::from_paths(&[vec!["tiles/gem15"], vec!["tiles/glo30"]], &["tiles/worldcover"])?;
/// let region = Region::Bbox { south: 49.2, west: 19.84, north: 49.38, east: 20.12 };
/// let job = TerrainJob::start(sources, region, Options::default(), "tatra.wtr", CancelToken::new());
/// while !job.is_finished() {
///     println!("{:.0}%", job.progress().fraction() * 100.0);
///     std::thread::sleep(std::time::Duration::from_millis(200));
/// }
/// let report = job.wait()?;
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
///
/// Dropping a job that has not finished cancels it and waits for it to stop (within a block;
/// it removes its partial file first).
pub struct TerrainJob {
    cancel: CancelToken,
    progress: Arc<Mutex<Progress>>,
    handle: Option<JoinHandle<Result<Report>>>,
}

impl TerrainJob {
    /// Starts compiling `region` into the file at `path` on a new thread. `cancel` (or a clone
    /// of it) stops the compile; [`TerrainJob::wait`] then returns [`Error::Cancelled`].
    pub fn start(
        sources: Sources,
        region: Region,
        options: Options,
        path: impl Into<PathBuf>,
        cancel: CancelToken,
    ) -> TerrainJob {
        let progress = Arc::new(Mutex::new(Progress::default()));
        let (shared, token, path) = (Arc::clone(&progress), cancel.clone(), path.into());
        let handle = std::thread::Builder::new()
            .name("terrain-compile-job".into())
            .spawn(move || {
                crate::compile_to_file(sources, &region, &options, &path, &token, |p| {
                    *shared.lock().unwrap_or_else(PoisonError::into_inner) = p;
                })
            })
            .unwrap_or_else(|e| {
                let e = Error::Output(format!("could not start the compile's thread: {e}"));
                std::thread::spawn(move || Err(e))
            });
        TerrainJob {
            cancel,
            progress,
            handle: Some(handle),
        }
    }

    /// The latest progress report (cheap; any thread).
    pub fn progress(&self) -> Progress {
        *self.progress.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Whether the compile has ended (succeeded, failed or stopped after a cancel), so that
    /// [`TerrainJob::wait`] returns at once.
    pub fn is_finished(&self) -> bool {
        self.handle.as_ref().is_none_or(|h| h.is_finished())
    }

    /// Cancels the compile (as cancelling its token does).
    pub fn cancel(&self) {
        self.cancel.cancel();
    }

    /// Waits for the compile to end and returns its outcome.
    pub fn wait(mut self) -> Result<Report> {
        self.join()
    }

    fn join(&mut self) -> Result<Report> {
        let Some(handle) = self.handle.take() else {
            return Err(Error::Output("the compile's result was taken".into()));
        };
        handle.join().unwrap_or_else(|panic| {
            let message = panic
                .downcast_ref::<&str>()
                .map(|t| t.to_string())
                .or_else(|| panic.downcast_ref::<String>().cloned())
                .unwrap_or_else(|| "unknown panic".into());
            Err(Error::Output(format!(
                "the compile stopped on an internal error: {message}"
            )))
        })
    }
}

impl Drop for TerrainJob {
    fn drop(&mut self) {
        if self.handle.is_some() {
            self.cancel.cancel();
            let _ = self.join();
        }
    }
}

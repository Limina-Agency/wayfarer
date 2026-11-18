//! Progress and cancellation shared by every stage of one compile.
//!
//! Stages run on the caller's thread, on the compile's rayon pool and on the
//! ingest's decode threads, so the job is `Sync`: cancellation is a sticky
//! flag over the caller's `should_cancel`, and progress goes through a lock to
//! the caller's callback, which therefore must be `Send`.

use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};

use osm_pbf_ingest::SharedIngestProgress;

use crate::CompileStage;

/// `should_cancel` returned true.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Cancelled;

/// One report, as the stages make it; `crate::api` turns it into the public
/// `CompileProgress` (and the host protocol's details).
#[derive(Debug, Clone)]
pub(crate) struct Report {
    pub(crate) stage: CompileStage,
    /// The share of the stage done, 0 to 1, when its size is known.
    pub(crate) fraction: Option<f64>,
    /// The share of the whole compile done, 0 to 1.
    pub(crate) overall: f64,
    pub(crate) message: String,
    /// Objects read (ingest), ways pushed (graph) or bytes written (write);
    /// 0 where the stage counts nothing.
    pub(crate) rows_written: usize,
    /// The ingest's own report this one forwards, if any.
    pub(crate) ingest: Option<SharedIngestProgress>,
}

pub(crate) type ProgressFn<'a> = dyn FnMut(&Report) + Send + 'a;

pub(crate) struct Job<'a> {
    should_cancel: &'a (dyn Fn() -> bool + Sync),
    cancelled: AtomicBool,
    reporter: Mutex<Reporter<'a>>,
}

struct Reporter<'a> {
    on_progress: &'a mut ProgressFn<'a>,
    stage: Option<CompileStage>,
    fraction: f64,
}

impl<'a> Job<'a> {
    pub(crate) fn new(
        should_cancel: &'a (dyn Fn() -> bool + Sync),
        on_progress: &'a mut ProgressFn<'a>,
    ) -> Self {
        Self {
            should_cancel,
            cancelled: AtomicBool::new(false),
            reporter: Mutex::new(Reporter {
                on_progress,
                stage: None,
                fraction: 0.0,
            }),
        }
    }

    /// Whether the caller asked to stop. Once true it stays true, so a stage
    /// that stopped early (and holds partial results) is always followed by a
    /// [`Job::check`] that fails.
    pub(crate) fn cancelled(&self) -> bool {
        if self.cancelled.load(Ordering::Relaxed) {
            return true;
        }
        if (self.should_cancel)() {
            self.cancelled.store(true, Ordering::Relaxed);
            return true;
        }
        false
    }

    pub(crate) fn check(&self) -> Result<(), Cancelled> {
        if self.cancelled() {
            Err(Cancelled)
        } else {
            Ok(())
        }
    }

    /// Reports `fraction` (0 to 1, `None` when the stage's size is unknown)
    /// of `stage`. Fractions never go back within a stage: a report behind
    /// the last one (from a slower parallel worker) is dropped.
    pub(crate) fn report(
        &self,
        stage: CompileStage,
        fraction: Option<f64>,
        rows_written: usize,
        message: impl Into<String>,
    ) {
        self.send(stage, fraction, rows_written, message.into(), None);
    }

    /// Forwards one of the ingest's own reports as a report of `stage`.
    pub(crate) fn report_ingest(&self, stage: CompileStage, ingest: SharedIngestProgress) {
        let (fraction, objects) = (ingest.fraction, ingest.objects_read);
        let message = ingest.message.clone();
        self.send(stage, fraction, objects, message, Some(ingest));
    }

    fn send(
        &self,
        stage: CompileStage,
        fraction: Option<f64>,
        rows_written: usize,
        message: String,
        ingest: Option<SharedIngestProgress>,
    ) {
        let fraction = fraction.map(|fraction| fraction.clamp(0.0, 1.0));
        let mut reporter = self
            .reporter
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        if reporter.stage == Some(stage) {
            if let Some(fraction) = fraction {
                if fraction < reporter.fraction {
                    return;
                }
                reporter.fraction = fraction;
            }
        } else {
            reporter.stage = Some(stage);
            reporter.fraction = fraction.unwrap_or(0.0);
        }
        let report = Report {
            stage,
            fraction,
            overall: stage.overall(fraction),
            message,
            rows_written,
            ingest,
        };
        (reporter.on_progress)(&report);
    }
}

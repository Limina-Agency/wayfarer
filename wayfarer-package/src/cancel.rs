//! The stop flag shared by the engine's requests and the compiler's jobs.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

/// A shared stop flag. Clones share it; [`CancelToken::cancel`] from any
/// thread stops every request or compile carrying a clone, and cannot be
/// undone.
#[derive(Debug, Clone, Default)]
pub struct CancelToken(Arc<AtomicBool>);

impl CancelToken {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn cancel(&self) {
        self.0.store(true, Ordering::Relaxed);
    }

    pub fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::Relaxed)
    }

    /// Whether `other` is a clone of this token (shares its flag).
    pub fn same_as(&self, other: &CancelToken) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }
}

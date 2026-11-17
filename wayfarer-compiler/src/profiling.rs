//! Opt-in measurement only; no phase events or clocks in ordinary builds.

pub(crate) struct Phase {
    #[cfg(feature = "benchmark-profiling")]
    name: &'static str,
    #[cfg(feature = "benchmark-profiling")]
    started: std::time::Instant,
}

impl Phase {
    #[inline]
    pub(crate) fn start(_name: &'static str) -> Self {
        #[cfg(feature = "benchmark-profiling")]
        {
            let started = std::time::Instant::now();
            heap::reset_peak();
            emit(_name, "begin", None);
            Self {
                name: _name,
                started,
            }
        }
        #[cfg(not(feature = "benchmark-profiling"))]
        Self {}
    }
}

/// Live/peak heap at a named point inside a phase; resets the peak.
#[cfg(feature = "benchmark-profiling")]
pub(crate) fn checkpoint(name: &str) {
    emit(name, "checkpoint", None);
    heap::reset_segment_peak();
}

#[cfg(feature = "benchmark-profiling")]
fn emit(name: &str, event: &str, elapsed_ms: Option<f64>) {
    use std::io::Write;
    static START: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
    let start = START.get_or_init(std::time::Instant::now);
    let _ = writeln!(
        std::io::stderr().lock(),
        "{}",
        serde_json::json!({
            "benchmark_phase": name,
            "event": event,
            "since_first_phase_ms": start.elapsed().as_secs_f64() * 1000.0,
            "elapsed_ms": elapsed_ms,
            "live_bytes": heap::live(),
            "peak_live_bytes": heap::peak(),
            "segment_peak_live_bytes": heap::segment_peak(),
        })
    );
}

impl Drop for Phase {
    #[inline]
    fn drop(&mut self) {
        #[cfg(feature = "benchmark-profiling")]
        emit(
            self.name,
            "end",
            Some(self.started.elapsed().as_secs_f64() * 1000.0),
        );
    }
}

/// Counting allocator: exact live heap bytes and the peak since the last phase
/// began. RSS cannot attribute memory to phases because freed pages are often
/// retained by the allocator.
#[cfg(feature = "benchmark-profiling")]
mod heap {
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::sync::atomic::{AtomicUsize, Ordering};

    static LIVE: AtomicUsize = AtomicUsize::new(0);
    static PEAK: AtomicUsize = AtomicUsize::new(0);
    static SEGMENT_PEAK: AtomicUsize = AtomicUsize::new(0);

    struct Counting;

    unsafe impl GlobalAlloc for Counting {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            let pointer = unsafe { System.alloc(layout) };
            if !pointer.is_null() {
                grow(layout.size());
            }
            pointer
        }

        unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
            let pointer = unsafe { System.alloc_zeroed(layout) };
            if !pointer.is_null() {
                grow(layout.size());
            }
            pointer
        }

        unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
            unsafe { System.dealloc(pointer, layout) };
            LIVE.fetch_sub(layout.size(), Ordering::Relaxed);
        }

        unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
            let moved = unsafe { System.realloc(pointer, layout, new_size) };
            if !moved.is_null() {
                if new_size >= layout.size() {
                    grow(new_size - layout.size());
                } else {
                    LIVE.fetch_sub(layout.size() - new_size, Ordering::Relaxed);
                }
            }
            moved
        }
    }

    fn grow(bytes: usize) {
        let live = LIVE.fetch_add(bytes, Ordering::Relaxed) + bytes;
        PEAK.fetch_max(live, Ordering::Relaxed);
        SEGMENT_PEAK.fetch_max(live, Ordering::Relaxed);
    }

    #[global_allocator]
    static ALLOCATOR: Counting = Counting;

    pub(super) fn live() -> usize {
        LIVE.load(Ordering::Relaxed)
    }

    pub(super) fn peak() -> usize {
        PEAK.load(Ordering::Relaxed)
    }

    pub(super) fn segment_peak() -> usize {
        SEGMENT_PEAK.load(Ordering::Relaxed)
    }

    pub(super) fn reset_peak() {
        PEAK.store(LIVE.load(Ordering::Relaxed), Ordering::Relaxed);
        reset_segment_peak();
    }

    pub(super) fn reset_segment_peak() {
        SEGMENT_PEAK.store(LIVE.load(Ordering::Relaxed), Ordering::Relaxed);
    }
}

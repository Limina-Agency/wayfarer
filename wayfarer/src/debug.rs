//! Debug switches for the parity harnesses and profiling, read from the
//! environment only with the `debug-env` feature. Without it every switch is
//! off and the checks behind it compile away.

/// `WAYFARER_DEBUG`: print search, column and request-section stats.
pub(crate) fn trace() -> bool {
    switch("WAYFARER_DEBUG")
}

/// `WAYFARER_VERIFY`: re-derive every route's legality and cost from
/// the package's raw attributes.
pub(crate) fn verify() -> bool {
    switch("WAYFARER_VERIFY")
}

/// `WAYFARER_UNIDIRECTIONAL`: run route searches forward only.
pub(crate) fn unidirectional() -> bool {
    switch("WAYFARER_UNIDIRECTIONAL")
}

/// `WAYFARER_EXACT_STOP`: end bidirectional route searches on the
/// symmetric stop, exact under merely admissible bounds (about 5x the
/// visited edges of the default stop on regional routes). Package landmark
/// bounds are consistent, so the default stop is exact too.
pub(crate) fn exact_stop() -> bool {
    #[cfg(test)]
    if EXACT_STOP.with(std::cell::Cell::get) {
        return true;
    }
    switch("WAYFARER_EXACT_STOP")
}

#[cfg(test)]
thread_local! {
    static EXACT_STOP: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Forces [`exact_stop`] on this thread (tests).
#[cfg(test)]
pub(crate) fn force_exact_stop(value: bool) {
    EXACT_STOP.with(|flag| flag.set(value));
}

#[cfg(feature = "debug-env")]
fn switch(name: &str) -> bool {
    std::env::var_os(name).is_some()
}

#[cfg(not(feature = "debug-env"))]
fn switch(_name: &str) -> bool {
    false
}

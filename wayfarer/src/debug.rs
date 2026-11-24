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
/// visited edges of the default stop on regional routes). CCH potentials
/// are consistent, so the default stop is exact too.
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

const POTENTIALS_UNSET: u8 = u8::MAX;
static POTENTIALS: std::sync::atomic::AtomicU8 = std::sync::atomic::AtomicU8::new(POTENTIALS_UNSET);

/// `on` (or empty) → 0, `off` → 1.
fn potentials_code(name: &str) -> Option<u8> {
    match name {
        "on" | "" => Some(0),
        "off" => Some(1),
        _ => None,
    }
}

/// `WAYFARER_POTENTIALS=off`: searches bound with the straight line alone,
/// not the CCH potentials (or what [`set_potentials`] set).
pub(crate) fn potentials_off() -> bool {
    use std::sync::atomic::Ordering;
    let mut code = POTENTIALS.load(Ordering::Relaxed);
    if code == POTENTIALS_UNSET {
        code = env_value("WAYFARER_POTENTIALS")
            .and_then(|value| potentials_code(&value))
            .unwrap_or(0);
        POTENTIALS.store(code, Ordering::Relaxed);
    }
    code == 1
}

/// Sets [`potentials_off`] for the whole process (harnesses): `on` or
/// `off`; false for another name.
pub(crate) fn set_potentials(name: &str) -> bool {
    match potentials_code(name) {
        Some(code) => {
            POTENTIALS.store(code, std::sync::atomic::Ordering::Relaxed);
            true
        }
        None => false,
    }
}

/// `WAYFARER_MATRIX_MODE` (`pairwise` or `rows`): the search mode every
/// matrix uses (benchmarking).
pub(crate) fn matrix_mode() -> Option<String> {
    static VALUE: std::sync::OnceLock<Option<String>> = std::sync::OnceLock::new();
    VALUE
        .get_or_init(|| env_value("WAYFARER_MATRIX_MODE"))
        .clone()
}

/// `WAYFARER_REQUEST_METRIC` (`on` or `off`): whether every matrix without
/// an overlay bounds with its own costs customized, whatever its gates
/// (benchmarking).
pub(crate) fn request_metric() -> Option<bool> {
    static VALUE: std::sync::OnceLock<Option<bool>> = std::sync::OnceLock::new();
    *VALUE.get_or_init(|| match env_value("WAYFARER_REQUEST_METRIC")?.as_str() {
        "on" => Some(true),
        "off" => Some(false),
        _ => None,
    })
}

/// `WAYFARER_MATRIX_PAIRWISE_MAX_TARGETS`: the most targets a matrix
/// searches pairwise, for every profile (benchmarking).
pub(crate) fn matrix_pairwise_max_targets() -> Option<usize> {
    static VALUE: std::sync::OnceLock<Option<usize>> = std::sync::OnceLock::new();
    *VALUE.get_or_init(|| parsed("WAYFARER_MATRIX_PAIRWISE_MAX_TARGETS"))
}

/// `WAYFARER_CCH_ROW_TARGETS`: the most open target nodes a matrix row
/// bounds with CCH potentials (benchmarking).
pub(crate) fn cch_row_targets() -> Option<usize> {
    static VALUE: std::sync::OnceLock<Option<usize>> = std::sync::OnceLock::new();
    *VALUE.get_or_init(|| parsed("WAYFARER_CCH_ROW_TARGETS"))
}

fn parsed(name: &str) -> Option<usize> {
    env_value(name).and_then(|value| value.parse().ok())
}

#[cfg(feature = "debug-env")]
fn env_value(name: &str) -> Option<String> {
    std::env::var(name).ok()
}

#[cfg(not(feature = "debug-env"))]
fn env_value(_name: &str) -> Option<String> {
    None
}

#[cfg(feature = "debug-env")]
fn switch(name: &str) -> bool {
    std::env::var_os(name).is_some()
}

#[cfg(not(feature = "debug-env"))]
fn switch(_name: &str) -> bool {
    false
}

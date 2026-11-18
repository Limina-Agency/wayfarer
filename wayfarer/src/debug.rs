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

/// Which lower bound route searches use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Potential {
    /// CCH potentials once `RoutingPackage::prepare_potentials` built them,
    /// landmarks before (the default).
    Auto,
    /// Landmarks only, even with potentials prepared.
    Alt,
    /// CCH potentials, prepared on the first search that wants them.
    Cch,
}

const POTENTIAL_UNSET: u8 = u8::MAX;
static POTENTIAL: std::sync::atomic::AtomicU8 = std::sync::atomic::AtomicU8::new(POTENTIAL_UNSET);

fn potential_code(name: &str) -> Option<u8> {
    match name {
        "auto" | "" => Some(0),
        "alt" => Some(1),
        "cch" => Some(2),
        _ => None,
    }
}

/// `WAYFARER_POTENTIAL=auto|alt|cch` (default `auto`), or what
/// [`set_potential`] set.
pub(crate) fn potential() -> Potential {
    use std::sync::atomic::Ordering;
    let mut code = POTENTIAL.load(Ordering::Relaxed);
    if code == POTENTIAL_UNSET {
        code = env_value("WAYFARER_POTENTIAL")
            .and_then(|value| potential_code(&value))
            .unwrap_or(0);
        POTENTIAL.store(code, Ordering::Relaxed);
    }
    match code {
        1 => Potential::Alt,
        2 => Potential::Cch,
        _ => Potential::Auto,
    }
}

/// Sets [`potential`] for the whole process (harnesses); false for an
/// unknown name.
pub(crate) fn set_potential(name: &str) -> bool {
    match potential_code(name) {
        Some(code) => {
            POTENTIAL.store(code, std::sync::atomic::Ordering::Relaxed);
            true
        }
        None => false,
    }
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

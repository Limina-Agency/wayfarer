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

// ---------------------------------------------------------------------------
// PERF-2 spike switches (`WAYFARER_POTENTIAL`, `WAYFARER_ALTERNATIVES`).
// Read from the environment once with `debug-env`, and settable in-process
// (`set_potential`, `set_alternatives`) so one harness can alternate arms.
// Without `debug-env` they stay at today's behaviour.
// ---------------------------------------------------------------------------

use std::sync::atomic::{AtomicU32, Ordering};

/// The lower bound the route search uses (PERF-2 spike).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Potential {
    /// ALT with [`crate::package::landmarks::ACTIVE_LANDMARKS`] active (today).
    Alt6,
    /// ALT with every stored landmark active.
    Alt16,
    /// CCH lower-bound potentials (`package::cch`), ALT where they do not apply.
    Cch,
}

const UNSET: u32 = u32::MAX;
static POTENTIAL: AtomicU32 = AtomicU32::new(UNSET);
static ALTERNATIVES: AtomicU32 = AtomicU32::new(UNSET);

fn potential_code(name: &str) -> Option<u32> {
    match name {
        "alt6" | "alt" | "" => Some(0),
        "alt16" => Some(1),
        "cch" => Some(2),
        _ => None,
    }
}

/// `WAYFARER_POTENTIAL=alt6|alt16|cch` (default `alt6`).
pub(crate) fn potential() -> Potential {
    let mut code = POTENTIAL.load(Ordering::Relaxed);
    if code == UNSET {
        code = env_value("WAYFARER_POTENTIAL")
            .and_then(|value| potential_code(&value))
            .unwrap_or(0);
        POTENTIAL.store(code, Ordering::Relaxed);
    }
    match code {
        1 => Potential::Alt16,
        2 => Potential::Cch,
        _ => Potential::Alt6,
    }
}

/// Sets the potential in-process (harness); false for an unknown name.
pub(crate) fn set_potential(name: &str) -> bool {
    match potential_code(name) {
        Some(code) => {
            POTENTIAL.store(code, Ordering::Relaxed);
            true
        }
        None => false,
    }
}

/// How alternatives are searched (PERF-2 spike).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) struct AlternativesMode {
    /// Cap each attempt at this many times the primary's visited edges
    /// (min 20k); an attempt out of budget ends the loop (as in the PERF
    /// research's scratch experiment).
    pub budget_factor: Option<u32>,
    /// Stop each attempt once its penalised lower bound passes
    /// `max_cost_ratio` times the primary's cost; such an attempt is skipped,
    /// not the end of the loop.
    pub cost_bound: bool,
    /// Re-customize the CCH metric with each attempt's penalties, so that the
    /// potential is exact for the penalised metric (with `WAYFARER_POTENTIAL=cch`).
    pub recustomize: bool,
}

impl AlternativesMode {
    fn encode(self) -> u32 {
        self.budget_factor.unwrap_or(0).min(255)
            | (u32::from(self.cost_bound) << 8)
            | (u32::from(self.recustomize) << 9)
    }

    fn decode(code: u32) -> Self {
        Self {
            budget_factor: Some(code & 255).filter(|factor| *factor > 0),
            cost_bound: code & (1 << 8) != 0,
            recustomize: code & (1 << 9) != 0,
        }
    }

    /// `penalty` (today), or `+`-joined parts: `budget<k>`, `bound`,
    /// `recustom`, e.g. `bound+recustom`.
    fn parse(value: &str) -> Option<Self> {
        let mut mode = Self::default();
        for part in value.split('+') {
            match part {
                "penalty" | "" => {}
                "bound" => mode.cost_bound = true,
                "recustom" => mode.recustomize = true,
                budget if budget.starts_with("budget") => {
                    mode.budget_factor = Some(budget["budget".len()..].parse().ok()?);
                }
                _ => return None,
            }
        }
        Some(mode)
    }
}

/// `WAYFARER_ALTERNATIVES=penalty|budget<k>|bound|recustom` (`+`-joined).
pub(crate) fn alternatives_mode() -> AlternativesMode {
    let mut code = ALTERNATIVES.load(Ordering::Relaxed);
    if code == UNSET {
        code = env_value("WAYFARER_ALTERNATIVES")
            .and_then(|value| AlternativesMode::parse(&value))
            .unwrap_or_default()
            .encode();
        ALTERNATIVES.store(code, Ordering::Relaxed);
    }
    AlternativesMode::decode(code)
}

/// Sets the alternatives mode in-process (harness); false when unparsable.
pub(crate) fn set_alternatives(value: &str) -> bool {
    match AlternativesMode::parse(value) {
        Some(mode) => {
            ALTERNATIVES.store(mode.encode(), Ordering::Relaxed);
            true
        }
        None => false,
    }
}

thread_local! {
    /// A bound on the penalised cost of the current alternative attempt
    /// (`AlternativesMode::cost_bound`), and whether the search hit it.
    static COST_BOUND: std::cell::Cell<Option<f64>> = const { std::cell::Cell::new(None) };
    static COST_BOUND_HIT: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    /// Whether the current search should re-customize the CCH metric with
    /// its cost overrides (`AlternativesMode::recustomize`).
    static RECUSTOMIZE: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

pub(crate) fn set_cost_bound(bound: Option<f64>) {
    COST_BOUND.with(|cell| cell.set(bound));
    COST_BOUND_HIT.with(|cell| cell.set(false));
}

pub(crate) fn cost_bound() -> Option<f64> {
    COST_BOUND.with(std::cell::Cell::get)
}

pub(crate) fn note_cost_bound_hit() {
    COST_BOUND_HIT.with(|cell| cell.set(true));
}

pub(crate) fn cost_bound_hit() -> bool {
    COST_BOUND_HIT.with(std::cell::Cell::get)
}

pub(crate) fn set_recustomize(value: bool) {
    RECUSTOMIZE.with(|cell| cell.set(value));
}

pub(crate) fn recustomize() -> bool {
    RECUSTOMIZE.with(std::cell::Cell::get)
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

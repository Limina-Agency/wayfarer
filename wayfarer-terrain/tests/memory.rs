//! The memory budget for building a surface and searching it, counted as peak heap by a global
//! allocator: what a phone has to spare beside a map renderer. One test, so nothing else runs
//! in this process while it counts.
//!
//! The area is the synthetic one of `examples/synth`: a dense land-cover refresh everywhere,
//! flood and depth layers along every river, rubble patches, a drone land-cover patch, drone
//! features and a team overlay, costed for the walker and the vehicle. The budget covers the
//! inputs the host holds (terrain and layers), the surface build and one walker route.

#[path = "../examples/synth/mod.rs"]
#[allow(dead_code)]
mod synth;

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
use wayfarer_terrain::fuse::Prior;
use wayfarer_terrain::profile::Rules;
use wayfarer_terrain::search::{Grid, route};
use wayfarer_terrain::surface::Incident;

struct Counting;
static NOW: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        let p = unsafe { System.alloc(l) };
        if !p.is_null() {
            let n = NOW.fetch_add(l.size(), Relaxed) + l.size();
            PEAK.fetch_max(n, Relaxed);
        }
        p
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        unsafe { System.dealloc(p, l) };
        NOW.fetch_sub(l.size(), Relaxed);
    }
}

#[global_allocator]
static A: Counting = Counting;

const MB: usize = 1 << 20;

/// Peak heap, in bytes above what was allocated before, of making the area, building its
/// surface and routing a walker across it.
fn peak(km2: f64) -> usize {
    let base = NOW.load(Relaxed);
    PEAK.store(base, Relaxed);
    let a = synth::area(km2);
    let rules = [Rules::walker_default(), Rules::vehicle()];
    let prior = Prior::default();
    let inc = Incident {
        id: *b"wayfarer-memory\0",
        terrain: &a.terrain,
        prior: &prior,
        layers: a.layers.iter().collect(),
        overlay: a.overlay.clone(),
        rules: &rules,
        epoch_s: 900,
        allow_research: false,
        feature_min_conf: 128,
        level_min_w: 128,
    };
    let s = inc.surface(synth::T0).unwrap();
    let g = Grid::new(&a.terrain, &s).unwrap();
    let w = g.profile("walker").unwrap();
    let col = 300.min(g.w - 1);
    let r = route(&g, w, g.h / 8 * g.w + col, 7 * g.h / 8 * g.w + col);
    assert!(r.is_some(), "{km2} km²: the walker gets across");
    PEAK.load(Relaxed) - base
}

#[test]
fn peak_heap_within_budget() {
    for (km2, limit_mb) in [(451.0, 96), (900.0, 128)] {
        let p = peak(km2);
        eprintln!("{km2} km²: peak heap {} MB (limit {limit_mb} MB)", p / MB);
        assert!(
            p <= limit_mb * MB,
            "{km2} km²: peak heap {} MB over {limit_mb} MB",
            p / MB
        );
    }
}

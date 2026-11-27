//! Surface build bench on a synthetic incident area: build time, surface size, a walker route
//! and peak heap, for any area. No data is needed, so anyone can run it on their own machine:
//!
//! ```text
//! cargo run --release -p wayfarer-terrain --example bench -- --km2 900
//! RAYON_NUM_THREADS=1 cargo run --release -p wayfarer-terrain --example bench -- --km2 900
//! ```
//!
//! With no `--km2` it runs 451, 900 and 2,500 km². The build time is the median of three
//! builds after a warm-up. Peak heap is counted by a global allocator:
//! "inputs" is the terrain and the layers the host holds, "build" is the peak above them while
//! the surface is built, and "route" the peak above the surface while a route is searched.

#[allow(dead_code)]
mod synth;

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering::Relaxed};
use std::time::Instant;
use wayfarer_terrain::fuse::Prior;
use wayfarer_terrain::profile::Rules;
use wayfarer_terrain::search::{Grid, route};
use wayfarer_terrain::surface::Incident;

struct Counting;
static NOW: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);
/// Off while timing: the shared counters would serialise the threads' allocations. The timed
/// builds free everything they allocate, so the count stays right.
static COUNT: AtomicBool = AtomicBool::new(true);

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        let p = unsafe { System.alloc(l) };
        if !p.is_null() && COUNT.load(Relaxed) {
            let n = NOW.fetch_add(l.size(), Relaxed) + l.size();
            PEAK.fetch_max(n, Relaxed);
        }
        p
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        unsafe { System.dealloc(p, l) };
        if COUNT.load(Relaxed) {
            NOW.fetch_sub(l.size(), Relaxed);
        }
    }
}

#[global_allocator]
static A: Counting = Counting;

fn mb(b: usize) -> f64 {
    b as f64 / 1_048_576.0
}

/// Peak heap above the current level while `f` runs.
fn peak_above<T>(f: impl FnOnce() -> T) -> (T, usize) {
    let base = NOW.load(Relaxed);
    PEAK.store(base, Relaxed);
    let t = f();
    (t, PEAK.load(Relaxed) - base)
}

fn run(km2: f64) {
    let base = NOW.load(Relaxed);
    let t0 = Instant::now();
    let a = synth::area(km2);
    let made = t0.elapsed();
    let inputs = NOW.load(Relaxed) - base;
    let nb = a.terrain.blocks().len();
    let rules = [Rules::walker_default(), Rules::vehicle()];
    let prior = Prior::default();
    let inc = Incident {
        id: *b"wayfarer-bench\0\0",
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
    // The first build counts the peak heap and warms up; the median of three more is the time.
    let (s, build) = peak_above(|| inc.surface(synth::T0).unwrap());
    COUNT.store(false, Relaxed);
    let mut times: Vec<f64> = (0..3)
        .map(|_| {
            let t1 = Instant::now();
            std::hint::black_box(inc.surface(synth::T0).unwrap());
            t1.elapsed().as_secs_f64()
        })
        .collect();
    times.sort_by(f64::total_cmp);
    let built = times[1];
    COUNT.store(true, Relaxed);
    let file = s.write().len();
    let ((g, r), search) = peak_above(|| {
        let g = Grid::new(&a.terrain, &s).unwrap();
        let w = g.profile("walker").unwrap();
        // From an eighth of the way up to seven eighths, in the valley between two rivers:
        // the rivers carry flood layers and close the ground beside them.
        let col = 300.min(g.w - 1);
        let (src, dst) = (Some(g.h / 8 * g.w + col), Some(7 * g.h / 8 * g.w + col));
        let t2 = Instant::now();
        let r = route(&g, w, src.unwrap(), dst.unwrap());
        (g, (r, t2.elapsed()))
    });
    let (route, rt) = r;
    println!(
        "{km2:>6.0} km² ({nb} blocks, {} x {} posts): build {:>7.1} ms, surface {:.2} MB, route {:>6.1} ms ({}); \
peak heap: inputs {:.0} MB, build +{:.0} MB, grid and route +{:.0} MB  [generated in {:.1} s]",
        g.h,
        g.w,
        built * 1e3,
        file as f64 / 1e6,
        rt.as_secs_f64() * 1e3,
        if route.is_some() { "found" } else { "none" },
        mb(inputs),
        mb(build),
        mb(search),
        made.as_secs_f64(),
    );
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let sizes: Vec<f64> = match args.iter().position(|a| a == "--km2") {
        Some(i) => vec![args[i + 1].parse().expect("--km2 takes a number")],
        None => vec![451.0, 900.0, 2_500.0],
    };
    println!("threads: {}", rayon::current_num_threads());
    for km2 in sizes {
        run(km2);
    }
}

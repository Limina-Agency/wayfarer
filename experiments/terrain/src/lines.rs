//! E5: OSM linear features onto the lattice, integer only.
//!
//! Each post gets a u16 bitmask of the linear features whose line passes through its cell
//! `[p - s/2, p + s/2)`: way kinds (bits 0-7) and barriers (bits 8-12). A line marks every
//! cell it touches, walked cell by cell with exact rational crossings (supercover), and a
//! step through a cell corner marks the cell on the x side first, so a marked line is always
//! 4-connected and an 8-connected mover cannot slip across it. Coordinates use E3's common
//! unit: OSM 1e-7 degrees x 27 = lattice ticks x 625.
//!
//! Bridges and tunnels: a motorway or railway on a bridge or in a tunnel is not a barrier
//! (the ground under or over it stays open), so it is left out; a waterway in a culvert is
//! left out. Ways of every other kind are kept, also on bridges, since a bridge is how a
//! walker crosses the barrier under it.

use crate::lattice::{BLOCK, BLOCK_POSTS, BlockId, Spacing};
use osmpbf::{Element, ElementReader};
use std::collections::HashMap;

pub const MOTORWAY: u16 = 1 << 0;
pub const MAJOR: u16 = 1 << 1;
pub const MINOR: u16 = 1 << 2;
pub const TRACK: u16 = 1 << 3;
pub const PATH: u16 = 1 << 4;
pub const PATH_MOUNTAIN: u16 = 1 << 5;
pub const PATH_DEMANDING: u16 = 1 << 6;
pub const STEPS: u16 = 1 << 7;
pub const RIVER: u16 = 1 << 8;
pub const STREAM: u16 = 1 << 9;
pub const RAIL: u16 = 1 << 10;
pub const FENCE: u16 = 1 << 11;
pub const CLIFF: u16 = 1 << 12;
pub const WAYS: u16 = 0xFF;
pub const BIT_NAMES: [&str; 13] =
    ["motorway", "major", "minor", "track", "path", "path_mountain", "path_demanding", "steps", "river", "stream", "rail", "fence", "cliff"];

const OSM_SCALE: i64 = 27;
const TICK_SCALE: i64 = 625;

/// Feature bits of a way from its tags (0 = not a linear feature E5 uses).
pub fn classify<'a>(tags: impl Iterator<Item = (&'a str, &'a str)>) -> u16 {
    let t: HashMap<&str, &str> = tags.collect();
    let g = |k: &str| t.get(k).copied();
    let raised = g("bridge").is_some_and(|v| v != "no") || g("tunnel").is_some_and(|v| v != "no");
    let mut bits = 0u16;
    if let Some(h) = g("highway") {
        bits |= match h {
            "motorway" | "motorway_link" | "trunk" | "trunk_link" => {
                if raised { 0 } else { MOTORWAY }
            }
            "primary" | "primary_link" | "secondary" | "secondary_link" => MAJOR,
            "tertiary" | "tertiary_link" | "unclassified" | "residential" | "living_street" | "service" | "road" => MINOR,
            "track" => TRACK,
            "path" | "footway" | "bridleway" | "cycleway" | "pedestrian" => match g("sac_scale") {
                Some("mountain_hiking") => PATH_MOUNTAIN,
                Some("demanding_mountain_hiking" | "alpine_hiking" | "demanding_alpine_hiking" | "difficult_alpine_hiking") => PATH_DEMANDING,
                _ => PATH,
            },
            "steps" => STEPS,
            _ => 0,
        };
    }
    if let Some(w) = g("waterway") {
        if g("tunnel").is_none_or(|v| v == "no") {
            bits |= match w {
                "river" | "canal" => RIVER,
                "stream" | "ditch" | "drain" => STREAM,
                _ => 0,
            };
        }
    }
    if let Some(r) = g("railway") {
        if !raised && matches!(r, "rail" | "light_rail" | "narrow_gauge" | "tram") {
            bits |= RAIL;
        }
    }
    if let Some(b) = g("barrier") {
        if matches!(b, "fence" | "wall" | "city_wall" | "hedge" | "retaining_wall" | "guard_rail") {
            bits |= FENCE;
        }
    }
    if g("natural") == Some("cliff") {
        bits |= CLIFF;
    }
    bits
}

#[derive(Default, Debug)]
pub struct LineStats {
    pub ways: u64,
    pub ways_in_area: u64,
    pub segments: u64,
    pub cells_marked: u64,
}

/// Mark the cells a segment touches. Grid: `ncol x nrow` cells of `cs` units, cell (0, 0)
/// starting at (`x0`, `y0`). Exact: crossings are compared as rationals in i128.
fn walk(a: (i64, i64), b: (i64, i64), x0: i64, y0: i64, cs: i64, ncol: i64, nrow: i64, mut mark: impl FnMut(i64, i64)) {
    let cell = |v: i64, o: i64| (v - o).div_euclid(cs);
    let (mut cx, mut cy) = (cell(a.0, x0), cell(a.1, y0));
    let (ex, ey) = (cell(b.0, x0), cell(b.1, y0));
    let (dx, dy) = ((b.0 - a.0) as i128, (b.1 - a.1) as i128);
    let (sx, sy) = (dx.signum() as i64, dy.signum() as i64);
    let inside = |x: i64, y: i64| x >= 0 && y >= 0 && x < ncol && y < nrow;
    if inside(cx, cy) {
        mark(cx, cy);
    }
    let mut guard = 0u64;
    while (cx, cy) != (ex, ey) {
        guard += 1;
        assert!(guard < 1 << 32);
        // Next boundary on each axis, as parameter t = num / den along the segment.
        let bx = x0 as i128 + (cx + if sx > 0 { 1 } else { 0 }) as i128 * cs as i128;
        let by = y0 as i128 + (cy + if sy > 0 { 1 } else { 0 }) as i128 * cs as i128;
        // tx = (bx - ax)/dx, ty = (by - ay)/dy; compare tx vs ty without division.
        let step_x = if sx == 0 {
            false
        } else if sy == 0 {
            true
        } else {
            let lx = (bx - a.0 as i128) * dy.abs() * sx as i128;
            let ly = (by - a.1 as i128) * dx.abs() * sy as i128;
            lx <= ly // tie (corner): x first, then y on the next turn
        };
        if step_x {
            cx += sx;
        } else {
            cy += sy;
        }
        if inside(cx, cy) {
            mark(cx, cy);
        }
    }
}

/// Rasterize the linear features onto the posts of `blocks` (which must form a rectangle).
pub fn rasterize(pbf: &str, s: Spacing, blocks: &[BlockId]) -> (Vec<(BlockId, Vec<u16>)>, LineStats) {
    let mut stats = LineStats::default();
    let mut ways: Vec<(i64, u16, Vec<i64>)> = ElementReader::from_path(pbf)
        .unwrap()
        .par_map_reduce(
            |e| match e {
                Element::Way(w) => {
                    let c = classify(w.tags());
                    if c == 0 { vec![] } else { vec![(w.id(), c, w.refs().collect())] }
                }
                _ => vec![],
            },
            Vec::new,
            |mut a, b| {
                a.extend(b);
                a
            },
        )
        .unwrap();
    ways.sort_by_key(|w| w.0);
    stats.ways = ways.len() as u64;
    let mut need: Vec<i64> = ways.iter().flat_map(|w| w.2.iter().copied()).collect();
    need.sort_unstable();
    need.dedup();
    let mut nodes: Vec<(i64, i32, i32)> = ElementReader::from_path(pbf)
        .unwrap()
        .par_map_reduce(
            |e| match e {
                Element::DenseNode(n) if need.binary_search(&n.id()).is_ok() => vec![(n.id(), n.decimicro_lon(), n.decimicro_lat())],
                Element::Node(n) if need.binary_search(&n.id()).is_ok() => vec![(n.id(), n.decimicro_lon(), n.decimicro_lat())],
                _ => vec![],
            },
            Vec::new,
            |mut a, b| {
                a.extend(b);
                a
            },
        )
        .unwrap();
    nodes.sort_unstable_by_key(|n| n.0);
    let loc = |id: i64| nodes.binary_search_by_key(&id, |n| n.0).ok().map(|k| (nodes[k].1 as i64 * OSM_SCALE, nodes[k].2 as i64 * OSM_SCALE));

    let bi0 = blocks.iter().map(|b| b.bi).min().unwrap() as i64;
    let bi1 = blocks.iter().map(|b| b.bi).max().unwrap() as i64;
    let bj0 = blocks.iter().map(|b| b.bj).min().unwrap() as i64;
    let bj1 = blocks.iter().map(|b| b.bj).max().unwrap() as i64;
    assert_eq!(blocks.len() as i64, (bi1 - bi0 + 1) * (bj1 - bj0 + 1), "lines: blocks must form a rectangle");
    let nrow = (bi1 - bi0 + 1) * BLOCK as i64;
    let ncol = (bj1 - bj0 + 1) * BLOCK as i64;
    let cs = s.0 * TICK_SCALE;
    let x0 = (bj0 * BLOCK as i64 * s.0 - s.0 / 2) * TICK_SCALE;
    let y0 = (bi0 * BLOCK as i64 * s.0 - s.0 / 2) * TICK_SCALE;
    let (x1, y1) = (x0 + ncol * cs, y0 + nrow * cs);
    let mut grid = vec![0u16; (nrow * ncol) as usize];
    for (_, bits, refs) in &ways {
        let pts: Vec<(i64, i64)> = refs.iter().filter_map(|&r| loc(r)).collect();
        if pts.len() < 2 {
            continue;
        }
        let (mnx, mxx) = (pts.iter().map(|p| p.0).min().unwrap(), pts.iter().map(|p| p.0).max().unwrap());
        let (mny, mxy) = (pts.iter().map(|p| p.1).min().unwrap(), pts.iter().map(|p| p.1).max().unwrap());
        if mxx < x0 || mnx >= x1 || mxy < y0 || mny >= y1 {
            continue;
        }
        stats.ways_in_area += 1;
        for w in pts.windows(2) {
            stats.segments += 1;
            walk(w[0], w[1], x0, y0, cs, ncol, nrow, |cx, cy| grid[(cy * ncol + cx) as usize] |= bits);
        }
    }
    stats.cells_marked = grid.iter().filter(|&&v| v != 0).count() as u64;
    let mut out = Vec::new();
    for &b in blocks {
        let mut v = vec![0u16; BLOCK_POSTS];
        let (r0, c0) = ((b.bi as i64 - bi0) * BLOCK as i64, (b.bj as i64 - bj0) * BLOCK as i64);
        for r in 0..BLOCK {
            for c in 0..BLOCK {
                v[r * BLOCK + c] = grid[((r0 + r as i64) * ncol + c0 + c as i64) as usize];
            }
        }
        out.push((b, v));
    }
    out.sort_by_key(|(b, _)| *b);
    (out, stats)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn supercover_is_4_connected() {
        for &(a, b) in &[((5, 5), (95, 37)), ((95, 5), (5, 95)), ((0, 0), (40, 40)), ((33, 3), (33, 77)), ((7, 70), (91, 12))] {
            let mut cells = Vec::new();
            walk(a, b, 0, 0, 10, 100, 100, |x, y| cells.push((x, y)));
            for w in cells.windows(2) {
                assert_eq!((w[0].0 - w[1].0).abs() + (w[0].1 - w[1].1).abs(), 1, "{a:?} {b:?} {w:?}");
            }
            assert_eq!(*cells.first().unwrap(), (a.0 / 10, a.1 / 10));
            assert_eq!(*cells.last().unwrap(), (b.0 / 10, b.1 / 10));
        }
    }
    #[test]
    fn classes() {
        assert_eq!(classify([("highway", "path"), ("sac_scale", "mountain_hiking")].into_iter()), PATH_MOUNTAIN);
        assert_eq!(classify([("highway", "motorway"), ("bridge", "yes")].into_iter()), 0);
        assert_eq!(classify([("waterway", "river")].into_iter()), RIVER);
        assert_eq!(classify([("highway", "footway"), ("bridge", "yes")].into_iter()), PATH);
    }
}

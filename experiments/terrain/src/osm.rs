//! E3: OSM land-cover areas onto the lattice, integer only.
//!
//! Features are closed ways and `type=multipolygon` relations whose tags map to a fine class
//! (`classify`). Relation rings are joined from member ways by shared end nodes, in member
//! order; every ring of a feature is filled with the even-odd rule, so outer/inner roles are
//! not needed for the fill. Overlaps: the feature with the smaller area is on top (a pond in a
//! forest, a wood in a residential area); ties by fine-class priority, then by OSM id.
//!
//! Each post's cell `[p - s/2, p + s/2)` is sampled at 4 x 4 sub-cell centres
//! (`p - s/2 + (2q + 1) s/8`, whole ticks for s = 1", 2", 3"). A sample is inside a ring when
//! the number of ring edges crossing its row strictly to its left is odd (edges half-open in
//! y). Coordinates are compared exactly: OSM 1e-7 degrees x 27 and lattice ticks x 625 share
//! one unit (1 degree = 2.7e8 units). Per post: the majority fine class over its covered
//! samples (ties by priority) and the number of covered samples (0-16).

use crate::lattice::{BLOCK, BLOCK_POSTS, BlockId, Spacing};
use osmpbf::{Element, ElementReader, RelMemberType};
use rayon::prelude::*;
use std::collections::HashMap;

pub const SUB: i64 = 4;

/// Fine OSM classes (u8). 0 = no OSM area.
pub const FINE_NAMES: [&str; 32] = [
    "none", "forest", "swamp_forest", "scrub", "heath", "clearcut", "grass", "meadow", "fell", "farmland",
    "orchard", "allotments", "residential", "industrial", "building", "paved", "cemetery", "bare_rock",
    "scree", "sand", "quarry", "glacier", "water", "marsh", "reedbed", "bog", "wet_meadow", "garden",
    "farmyard", "park", "pitch", "construction",
];
/// Tie priority per fine class (higher wins): hazards and barriers first, as in E1.
const FINE_PRI: [u8; 32] = [
    0, 12, 20, 13, 9, 8, 4, 5, 10, 2, 3, 3, 6, 7, 30, 15, 6, 25, 24, 18, 23, 26, 31, 22, 21, 22, 19, 4, 6, 5, 5, 7,
];

fn wetland(v: &str) -> u8 {
    match v {
        "swamp" | "mangrove" => 2,
        "reedbed" => 24,
        "bog" | "string_bog" | "fen" => 25,
        "wet_meadow" => 26,
        _ => 23, // marsh, saltmarsh, tidalflat, unspecified
    }
}

/// Fine class of an area feature from its tags, or 0. The first matching rule wins, in this
/// order: water, wetland, natural, landcover, landuse, leisure, paved areas, building.
pub fn classify<'a>(tags: impl Iterator<Item = (&'a str, &'a str)>) -> u8 {
    let mut t: HashMap<&str, &str> = HashMap::new();
    for (k, v) in tags {
        t.insert(k, v);
    }
    let g = |k: &str| t.get(k).copied();
    if g("area") == Some("no") {
        return 0;
    }
    if g("natural") == Some("water") || g("waterway") == Some("riverbank") || matches!(g("landuse"), Some("reservoir" | "basin"))
        || g("natural") == Some("bay")
    {
        return 22;
    }
    if g("natural") == Some("wetland") {
        return wetland(g("wetland").unwrap_or(""));
    }
    if let Some(v) = g("natural") {
        let c = match v {
            "wood" => 1,
            "scrub" | "shrubbery" => 3,
            "heath" => 4,
            "grassland" => 7,
            "fell" | "tundra" => 8,
            "bare_rock" | "rock" | "cliff" | "stone" => 17,
            "scree" | "shingle" => 18,
            "sand" | "beach" | "dune" => 19,
            "glacier" => 21,
            _ => 0,
        };
        if c != 0 {
            return c;
        }
    }
    if let Some(v) = g("landcover") {
        let c = match v {
            "trees" | "wood" => 1,
            "scrub" => 3,
            "grass" => 6,
            "gravel" | "sand" => 19,
            "rock" => 17,
            _ => 0,
        };
        if c != 0 {
            return c;
        }
    }
    if g("man_made") == Some("clearcut") {
        return 5;
    }
    if let Some(v) = g("landuse") {
        let c = match v {
            "forest" => 1,
            "logging" => 5,
            "grass" | "village_green" | "recreation_ground" | "flowerbed" | "greenfield" => 6,
            "meadow" => 7,
            "farmland" | "plant_nursery" => 9,
            "orchard" | "vineyard" => 10,
            "allotments" => 11,
            "residential" => 12,
            "industrial" | "commercial" | "retail" | "railway" | "garages" | "military" | "education" | "religious"
            | "depot" | "port" => 13,
            "greenhouse_horticulture" => 13,
            "cemetery" => 16,
            "quarry" | "landfill" => 20,
            "farmyard" => 28,
            "construction" | "brownfield" => 31,
            _ => 0,
        };
        if c != 0 {
            return c;
        }
    }
    if g("amenity") == Some("grave_yard") {
        return 16;
    }
    if let Some(v) = g("leisure") {
        let c = match v {
            "park" => 29,
            "garden" => 27,
            "pitch" | "track" | "golf_course" | "playground" => 30,
            "swimming_pool" => 22,
            _ => 0,
        };
        if c != 0 {
            return c;
        }
    }
    if matches!(g("amenity"), Some("parking")) || g("area:highway").is_some() || matches!(g("aeroway"), Some("apron" | "runway" | "taxiway"))
        || (g("highway") == Some("pedestrian") && g("area") == Some("yes"))
        || g("place") == Some("square")
    {
        return 15;
    }
    if let Some(v) = g("building") {
        if v != "no" {
            return 14;
        }
    }
    0
}

/// Scaled coordinate unit: 1 degree = 2.7e8 (lcm of 1e7 and 432,000 ticks).
const OSM_SCALE: i64 = 27;
const TICK_SCALE: i64 = 625;

struct Feature {
    id: i64, // ways positive, relations negative
    class: u8,
    area2: i128,             // twice the area, scaled units^2 (outer rings minus inner rings)
    rings: Vec<Vec<(i64, i64)>>, // (x = lon, y = lat), scaled, closed (first == last)
}

#[derive(Default, Debug)]
pub struct OsmStats {
    pub ways_closed: u64,
    pub relations: u64,
    pub rings_joined: u64,
    pub rings_open_dropped: u64,
    pub rings_missing_nodes: u64,
    pub features: u64,
    pub edges: u64,
}

fn ring_area2(r: &[(i64, i64)]) -> i128 {
    let mut a: i128 = 0;
    for w in r.windows(2) {
        a += w[0].0 as i128 * w[1].1 as i128 - w[1].0 as i128 * w[0].1 as i128;
    }
    a
}

/// Join member ways into closed rings, in member order (deterministic). Returns rings of
/// node ids with the role of the first way in each ring (true = inner).
fn join_rings(ways: &[(Vec<i64>, bool)], stats: &mut OsmStats) -> Vec<(Vec<i64>, bool)> {
    let mut out = Vec::new();
    let mut used = vec![false; ways.len()];
    for i in 0..ways.len() {
        if used[i] || ways[i].0.len() < 2 {
            continue;
        }
        used[i] = true;
        let mut ring = ways[i].0.clone();
        let inner = ways[i].1;
        loop {
            if ring.len() >= 4 && ring.first() == ring.last() {
                break;
            }
            let end = *ring.last().unwrap();
            let mut found = false;
            for j in 0..ways.len() {
                if used[j] || ways[j].0.len() < 2 {
                    continue;
                }
                let w = &ways[j].0;
                if w[0] == end {
                    ring.extend_from_slice(&w[1..]);
                } else if *w.last().unwrap() == end {
                    ring.extend(w.iter().rev().skip(1));
                } else {
                    continue;
                }
                used[j] = true;
                found = true;
                break;
            }
            if !found {
                break;
            }
        }
        if ring.len() >= 4 && ring.first() == ring.last() {
            if ways[i].0.first() != ways[i].0.last() {
                stats.rings_joined += 1;
            }
            out.push((ring, inner));
        } else {
            stats.rings_open_dropped += 1;
        }
    }
    out
}

/// Read every land-cover feature in the PBF (whole file; the window only filters edges later).
fn read_features(pbf: &str, stats: &mut OsmStats) -> Vec<Feature> {
    // Pass 1: multipolygon relations with a class.
    let mut rels: Vec<(i64, u8, Vec<(i64, bool)>)> = ElementReader::from_path(pbf)
        .unwrap()
        .par_map_reduce(
            |e| match e {
                Element::Relation(r) => {
                    let tags: Vec<(&str, &str)> = r.tags().collect();
                    if !tags.iter().any(|&(k, v)| k == "type" && v == "multipolygon") {
                        return vec![];
                    }
                    let c = classify(tags.into_iter());
                    if c == 0 {
                        return vec![];
                    }
                    let mem: Vec<(i64, bool)> = r
                        .members()
                        .filter(|m| m.member_type == RelMemberType::Way)
                        .map(|m| (m.member_id, m.role().unwrap_or("") == "inner"))
                        .collect();
                    vec![(r.id(), c, mem)]
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
    rels.sort_by_key(|r| r.0);
    let mut member_ids: Vec<i64> = rels.iter().flat_map(|r| r.2.iter().map(|m| m.0)).collect();
    member_ids.sort_unstable();
    member_ids.dedup();
    stats.relations = rels.len() as u64;

    // Pass 2: ways (classified closed ways, and relation members).
    let mut ways: Vec<(i64, u8, Vec<i64>)> = ElementReader::from_path(pbf)
        .unwrap()
        .par_map_reduce(
            |e| match e {
                Element::Way(w) => {
                    let c = classify(w.tags());
                    let member = member_ids.binary_search(&w.id()).is_ok();
                    if c == 0 && !member {
                        return vec![];
                    }
                    let refs: Vec<i64> = w.refs().collect();
                    let closed = c != 0 && refs.len() >= 4 && refs.first() == refs.last();
                    if closed || member {
                        vec![(w.id(), if closed { c } else { 0 }, refs)]
                    } else {
                        vec![]
                    }
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
    let mut need: Vec<i64> = ways.iter().flat_map(|w| w.2.iter().copied()).collect();
    need.sort_unstable();
    need.dedup();

    // Pass 3: node locations.
    let mut nodes: Vec<(i64, i32, i32)> = ElementReader::from_path(pbf)
        .unwrap()
        .par_map_reduce(
            |e| match e {
                Element::DenseNode(n) if need.binary_search(&n.id()).is_ok() => {
                    vec![(n.id(), n.decimicro_lon(), n.decimicro_lat())]
                }
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
    let loc = |id: i64| -> Option<(i64, i64)> {
        nodes.binary_search_by_key(&id, |n| n.0).ok().map(|k| (nodes[k].1 as i64 * OSM_SCALE, nodes[k].2 as i64 * OSM_SCALE))
    };
    let to_coords = |refs: &[i64], stats: &mut OsmStats| -> Option<Vec<(i64, i64)>> {
        let mut v = Vec::with_capacity(refs.len());
        for &r in refs {
            match loc(r) {
                Some(p) => v.push(p),
                None => {
                    stats.rings_missing_nodes += 1;
                    return None;
                }
            }
        }
        Some(v)
    };

    let mut feats = Vec::new();
    for (id, c, refs) in &ways {
        if *c == 0 {
            continue;
        }
        stats.ways_closed += 1;
        if let Some(r) = to_coords(refs, stats) {
            let a = ring_area2(&r).abs();
            feats.push(Feature { id: *id, class: *c, area2: a, rings: vec![r] });
        }
    }
    let way_refs = |id: i64| ways.binary_search_by_key(&id, |w| w.0).ok().map(|k| &ways[k].2);
    for (rid, c, mem) in &rels {
        let mut mw: Vec<(Vec<i64>, bool)> = Vec::new();
        for &(wid, inner) in mem {
            if let Some(r) = way_refs(wid) {
                mw.push((r.clone(), inner));
            }
        }
        let rings = join_rings(&mw, stats);
        let mut rr = Vec::new();
        let mut area: i128 = 0;
        for (ring, inner) in rings {
            if let Some(r) = to_coords(&ring, stats) {
                let a = ring_area2(&r).abs();
                area += if inner { -a } else { a };
                rr.push(r);
            }
        }
        if !rr.is_empty() {
            feats.push(Feature { id: -*rid, class: *c, area2: area.max(0), rings: rr });
        }
    }
    stats.features = feats.len() as u64;
    feats
}

pub struct OsmBlock {
    pub class: Vec<u8>,
    pub cov: Vec<u8>,
}

/// Rasterize all features onto the posts of `blocks` at spacing `s`.
pub fn rasterize(pbf: &str, s: Spacing, blocks: &[BlockId]) -> (Vec<(BlockId, OsmBlock)>, OsmStats) {
    let mut stats = OsmStats::default();
    let mut feats = read_features(pbf, &mut stats);
    // Paint order: larger area first, so smaller features end on top.
    feats.sort_by(|a, b| b.area2.cmp(&a.area2).then(FINE_PRI[a.class as usize].cmp(&FINE_PRI[b.class as usize])).then(a.id.cmp(&b.id)));
    let class_of: Vec<u8> = feats.iter().map(|f| f.class).collect();

    let bi0 = blocks.iter().map(|b| b.bi).min().unwrap() as i64;
    let bi1 = blocks.iter().map(|b| b.bi).max().unwrap() as i64;
    let bj0 = blocks.iter().map(|b| b.bj).min().unwrap() as i64;
    let bj1 = blocks.iter().map(|b| b.bj).max().unwrap() as i64;
    let sub = s.0 / SUB; // sample spacing, ticks
    assert!(s.0 % (2 * SUB) == 0);
    // Sample (row, col) centre in ticks; rows from the south, cols from the west.
    let y0 = (bi0 * BLOCK as i64) * s.0 - s.0 / 2 + sub / 2;
    let x0 = (bj0 * BLOCK as i64) * s.0 - s.0 / 2 + sub / 2;
    let nbands = (bi1 - bi0 + 1) as usize;
    let rows_per_band = BLOCK as i64 * SUB;
    let ncols = ((bj1 - bj0 + 1) * BLOCK as i64 * SUB) as usize;
    let (y0s, x0s, dys) = (y0 * TICK_SCALE, x0 * TICK_SCALE, sub * TICK_SCALE);

    // Edges bucketed by band: (rank, x1, y1, x2, y2), y1 < y2, scaled units.
    let mut buckets: Vec<Vec<(u32, i64, i64, i64, i64)>> = vec![Vec::new(); nbands];
    for (rank, f) in feats.iter().enumerate() {
        for r in &f.rings {
            for w in r.windows(2) {
                let ((xa, ya), (xb, yb)) = (w[0], w[1]);
                if ya == yb {
                    continue;
                }
                let (x1, y1, x2, y2) = if ya < yb { (xa, ya, xb, yb) } else { (xb, yb, xa, ya) };
                // sample rows with y1 <= Y < y2
                let r_first = (y1 - y0s + dys - 1).div_euclid(dys);
                let r_last = (y2 - y0s + dys - 1).div_euclid(dys) - 1;
                if r_last < r_first || r_last < 0 || r_first >= nbands as i64 * rows_per_band {
                    continue;
                }
                let b_first = (r_first.max(0) / rows_per_band) as usize;
                let b_last = ((r_last.min(nbands as i64 * rows_per_band - 1)) / rows_per_band) as usize;
                for b in b_first..=b_last {
                    buckets[b].push((rank as u32, x1, y1, x2, y2));
                }
                stats.edges += 1;
            }
        }
    }
    let want: std::collections::HashSet<BlockId> = blocks.iter().copied().collect();
    let dxs = sub * TICK_SCALE;
    let mut out: Vec<(BlockId, OsmBlock)> = buckets
        .par_iter()
        .enumerate()
        .flat_map_iter(|(band, edges)| {
            let bi = bi0 + band as i64;
            let cols_in: Vec<i64> = (bj0..=bj1).filter(|&bj| want.contains(&BlockId { bi: bi as i32, bj: bj as i32 })).collect();
            if cols_in.is_empty() {
                return Vec::new().into_iter();
            }
            let row0 = band as i64 * rows_per_band;
            let mut per_row: Vec<Vec<(u32, i32)>> = vec![Vec::new(); rows_per_band as usize];
            for &(rank, x1, y1, x2, y2) in edges {
                let r_first = ((y1 - y0s + dys - 1).div_euclid(dys)).max(row0);
                let r_last = ((y2 - y0s + dys - 1).div_euclid(dys) - 1).min(row0 + rows_per_band - 1);
                for row in r_first..=r_last {
                    let ys = y0s + row * dys;
                    // crossing x = x1 + (ys - y1)(x2 - x1)/(y2 - y1); first col k with x0s + k dxs > x
                    let den = (y2 - y1) as i128;
                    let num = x1 as i128 * den + (ys - y1) as i128 * (x2 - x1) as i128;
                    let k = (num - x0s as i128 * den).div_euclid(dxs as i128 * den) + 1;
                    let k = k.clamp(0, ncols as i128) as i32;
                    per_row[(row - row0) as usize].push((rank, k));
                }
            }
            let mut grid = vec![0u8; rows_per_band as usize * ncols];
            for (r, v) in per_row.iter_mut().enumerate() {
                v.sort_unstable();
                let line = &mut grid[r * ncols..(r + 1) * ncols];
                let mut i = 0;
                while i < v.len() {
                    let rank = v[i].0;
                    let mut j = i;
                    while j < v.len() && v[j].0 == rank {
                        j += 1;
                    }
                    let c = class_of[rank as usize];
                    let mut q = i;
                    while q + 1 < j {
                        let (a, b) = (v[q].1 as usize, v[q + 1].1 as usize);
                        line[a..b].fill(c);
                        q += 2;
                    }
                    i = j;
                }
            }
            let mut res = Vec::new();
            for bj in cols_in {
                let mut ob = OsmBlock { class: vec![0; BLOCK_POSTS], cov: vec![0; BLOCK_POSTS] };
                let c0 = ((bj - bj0) * BLOCK as i64 * SUB) as usize;
                for pr in 0..BLOCK {
                    for pc in 0..BLOCK {
                        let mut h = [0u8; 32];
                        for sr in 0..SUB as usize {
                            let line = &grid[(pr * SUB as usize + sr) * ncols..];
                            for sc in 0..SUB as usize {
                                h[line[c0 + pc * SUB as usize + sc] as usize] += 1;
                            }
                        }
                        let mut best = 0usize; // majority over covered samples only
                        for k in 1..32 {
                            let hb = if best == 0 { 0 } else { h[best] };
                            if h[k] > hb || (h[k] == hb && h[k] > 0 && FINE_PRI[k] > FINE_PRI[best]) {
                                best = k;
                            }
                        }
                        ob.class[pr * BLOCK + pc] = best as u8;
                        ob.cov[pr * BLOCK + pc] = (SUB * SUB) as u8 - h[0];
                    }
                }
                res.push((BlockId { bi: bi as i32, bj: bj as i32 }, ob));
            }
            res.into_iter()
        })
        .collect();
    out.sort_by_key(|(b, _)| *b);
    (out, stats)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn classes() {
        assert_eq!(classify([("landuse", "forest")].into_iter()), 1);
        assert_eq!(classify([("natural", "wetland"), ("wetland", "swamp")].into_iter()), 2);
        assert_eq!(classify([("building", "yes")].into_iter()), 14);
        assert_eq!(classify([("building", "no")].into_iter()), 0);
        assert_eq!(classify([("natural", "water"), ("landuse", "forest")].into_iter()), 22);
    }
    #[test]
    fn join() {
        let mut st = OsmStats::default();
        let w = vec![(vec![1, 2, 3], false), (vec![5, 4, 3], false), (vec![5, 6, 1], false), (vec![7, 8, 9, 7], true)];
        let r = join_rings(&w, &mut st);
        assert_eq!(r.len(), 2);
        assert_eq!(r[0].0, vec![1, 2, 3, 4, 5, 6, 1]);
        assert!(r[1].1);
    }
}

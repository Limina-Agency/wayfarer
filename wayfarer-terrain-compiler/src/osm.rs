//! The OSM stage: land-cover areas and linear features from an OSM extract, onto the lattice,
//! in integers.
//!
//! ## Areas
//!
//! Area features are closed ways and `type=multipolygon` relations whose tags map to one of
//! 31 fine classes ([`area_class`], first rule wins: water, wetland, natural, landcover,
//! clearcut, landuse, leisure, paved, building). A relation's rings are joined from its member
//! ways by shared end nodes, in member order; a ring that does not close is dropped, and the
//! rest of the relation kept. Every ring of a feature fills by the even-odd rule, so roles only
//! set its area. Overlaps: the feature with the smaller area is on top, then the higher class
//! priority, then the lower OSM id (ways before relations).
//!
//! Each post's cell is sampled at 4 × 4 sub-cell centres. A sample is inside a ring when the
//! ring's edges cross its row strictly to its left an odd number of times (edges half-open in
//! latitude). OSM's 1e-7° times 216 and an eighth of a tick times 625 are one unit (1° is
//! 2.16 × 10⁹), so every comparison is exact. A post gets the majority fine class of its
//! covered samples (ties to the higher priority) and how many samples are covered (0-16).
//!
//! ## Linear features
//!
//! A way's tags give bits of the lines mask ([`line_bits`], `wayfarer_terrain::lines`): road
//! kinds, paths by `sac_scale`, steps, rivers, streams, railways, fences and walls, cliffs.
//! A way marks every cell its segments touch (an exact supercover, stepping through a cell
//! corner along longitude first), so a marked line is 4-connected and an 8-connected mover
//! cannot slip across it. A motorway or railway on a bridge or in a tunnel is not a barrier
//! and is left out, as is a waterway in a tunnel; every other way is kept on bridges too,
//! since a bridge is how a walker crosses what is under it.
//!
//! These are E3's and E5's rules (`off-road-sar/exp/E3`, `E5`), ported.

use crate::{CancelToken, Error, Result};
use osm_pbf_ingest::node_store::{DecimicroBox, NodeCursor};
use osm_pbf_ingest::{
    IngestBuildProfile, NodeBatch, Pass2Options, Pass2Sink, PrepareOptions, RelationBatch,
    RelationMemberType, WayBatch, prepare_ingest_index_with_options, run_pass2_with_options,
};
use rayon::prelude::*;
use std::path::Path;
use std::sync::Arc;
use wayfarer_terrain::lattice::{BLOCK, BLOCK_POSTS, BlockId, Spacing};
use wayfarer_terrain::lines as L;

/// Units per 1e-7°.
const OSM_UNIT: i64 = 216;
/// Units per tick (an eighth of a tick is 625).
const TICK_UNIT: i64 = 5_000;
/// Samples per post side for the areas.
pub(crate) const SUB: usize = 4;

/// Fine OSM classes. 0 = no OSM area.
pub const FINE_NAMES: [&str; 32] = [
    "none",
    "forest",
    "swamp_forest",
    "scrub",
    "heath",
    "clearcut",
    "grass",
    "meadow",
    "fell",
    "farmland",
    "orchard",
    "allotments",
    "residential",
    "industrial",
    "building",
    "paved",
    "cemetery",
    "bare_rock",
    "scree",
    "sand",
    "quarry",
    "glacier",
    "water",
    "marsh",
    "reedbed",
    "bog",
    "wet_meadow",
    "garden",
    "farmyard",
    "park",
    "pitch",
    "construction",
];

/// Tie priority per fine class (higher wins): hazards and barriers first.
const FINE_PRI: [u8; 32] = [
    0, 12, 20, 13, 9, 8, 4, 5, 10, 2, 3, 3, 6, 7, 30, 15, 6, 25, 24, 18, 23, 26, 31, 22, 21, 22,
    19, 4, 6, 5, 5, 7,
];

pub(crate) const FOREST: u8 = 1;
pub(crate) const SCRUB: u8 = 3;

/// The 4-bit land-cover code of each fine class (`wayfarer_terrain::profile`), before the
/// forest and scrub rules of [`crate::compile`].
pub(crate) const FINE_TO_COVER: [u8; 32] = [
    0, 1, 15, 0, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5, 5, 13, 14, 6, 6, 7, 8, 9, 9, 15, 9, 3, 5, 3,
    3, 5,
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

/// The fine class of an area feature from its tags (`get` looks a key up), or 0.
pub fn area_class<'a>(get: impl Fn(&str) -> Option<&'a str>) -> u8 {
    let g = get;
    if g("area") == Some("no") {
        return 0;
    }
    if g("natural") == Some("water")
        || g("waterway") == Some("riverbank")
        || matches!(g("landuse"), Some("reservoir" | "basin"))
        || g("natural") == Some("bay")
    {
        return 22;
    }
    if g("natural") == Some("wetland") {
        return wetland(g("wetland").unwrap_or(""));
    }
    let natural = match g("natural") {
        Some("wood") => 1,
        Some("scrub" | "shrubbery") => 3,
        Some("heath") => 4,
        Some("grassland") => 7,
        Some("fell" | "tundra") => 8,
        Some("bare_rock" | "rock" | "cliff" | "stone") => 17,
        Some("scree" | "shingle") => 18,
        Some("sand" | "beach" | "dune") => 19,
        Some("glacier") => 21,
        _ => 0,
    };
    if natural != 0 {
        return natural;
    }
    let landcover = match g("landcover") {
        Some("trees" | "wood") => 1,
        Some("scrub") => 3,
        Some("grass") => 6,
        Some("gravel" | "sand") => 19,
        Some("rock") => 17,
        _ => 0,
    };
    if landcover != 0 {
        return landcover;
    }
    if g("man_made") == Some("clearcut") {
        return 5;
    }
    let landuse = match g("landuse") {
        Some("forest") => 1,
        Some("logging") => 5,
        Some("grass" | "village_green" | "recreation_ground" | "flowerbed" | "greenfield") => 6,
        Some("meadow") => 7,
        Some("farmland" | "plant_nursery") => 9,
        Some("orchard" | "vineyard") => 10,
        Some("allotments") => 11,
        Some("residential") => 12,
        Some(
            "industrial"
            | "commercial"
            | "retail"
            | "railway"
            | "garages"
            | "military"
            | "education"
            | "religious"
            | "depot"
            | "port"
            | "greenhouse_horticulture",
        ) => 13,
        Some("cemetery") => 16,
        Some("quarry" | "landfill") => 20,
        Some("farmyard") => 28,
        Some("construction" | "brownfield") => 31,
        _ => 0,
    };
    if landuse != 0 {
        return landuse;
    }
    if g("amenity") == Some("grave_yard") {
        return 16;
    }
    let leisure = match g("leisure") {
        Some("park") => 29,
        Some("garden") => 27,
        Some("pitch" | "track" | "golf_course" | "playground") => 30,
        Some("swimming_pool") => 22,
        _ => 0,
    };
    if leisure != 0 {
        return leisure;
    }
    if g("amenity") == Some("parking")
        || g("area:highway").is_some()
        || matches!(g("aeroway"), Some("apron" | "runway" | "taxiway"))
        || (g("highway") == Some("pedestrian") && g("area") == Some("yes"))
        || g("place") == Some("square")
    {
        return 15;
    }
    if g("building").is_some_and(|v| v != "no") {
        return 14;
    }
    0
}

/// The lines-mask bits of a way from its tags, or 0.
pub fn line_bits<'a>(get: impl Fn(&str) -> Option<&'a str>) -> u16 {
    let g = get;
    let raised = g("bridge").is_some_and(|v| v != "no") || g("tunnel").is_some_and(|v| v != "no");
    let mut bits = match g("highway") {
        Some("motorway" | "motorway_link" | "trunk" | "trunk_link") if !raised => L::MOTORWAY,
        Some("primary" | "primary_link" | "secondary" | "secondary_link") => L::MAJOR,
        Some(
            "tertiary" | "tertiary_link" | "unclassified" | "residential" | "living_street"
            | "service" | "road",
        ) => L::MINOR,
        Some("track") => L::TRACK,
        Some("path" | "footway" | "bridleway" | "cycleway" | "pedestrian") => {
            match g("sac_scale") {
                Some("mountain_hiking") => L::PATH_MOUNTAIN,
                Some(
                    "demanding_mountain_hiking"
                    | "alpine_hiking"
                    | "demanding_alpine_hiking"
                    | "difficult_alpine_hiking",
                ) => L::PATH_DEMANDING,
                _ => L::PATH,
            }
        }
        Some("steps") => L::STEPS,
        _ => 0,
    };
    if g("tunnel").is_none_or(|v| v == "no") {
        bits |= match g("waterway") {
            Some("river" | "canal") => L::RIVER,
            Some("stream" | "ditch" | "drain") => L::STREAM,
            _ => 0,
        };
    }
    if !raised
        && matches!(
            g("railway"),
            Some("rail" | "light_rail" | "narrow_gauge" | "tram")
        )
    {
        bits |= L::RAIL;
    }
    if matches!(
        g("barrier"),
        Some("fence" | "wall" | "city_wall" | "hedge" | "retaining_wall" | "guard_rail")
    ) {
        bits |= L::FENCE;
    }
    if g("natural") == Some("cliff") {
        bits |= L::CLIFF;
    }
    bits
}

/// A box in 1e-7° (longitude `x`, latitude `y`), bounds included.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Rect {
    pub x0: i32,
    pub y0: i32,
    pub x1: i32,
    pub y1: i32,
}

impl Rect {
    fn of(pts: &[(i32, i32)]) -> Rect {
        let mut r = Rect {
            x0: i32::MAX,
            y0: i32::MAX,
            x1: i32::MIN,
            y1: i32::MIN,
        };
        for &(x, y) in pts {
            r.x0 = r.x0.min(x);
            r.x1 = r.x1.max(x);
            r.y0 = r.y0.min(y);
            r.y1 = r.y1.max(y);
        }
        r
    }
    fn meets(&self, o: &Rect) -> bool {
        self.x0 <= o.x1 && o.x0 <= self.x1 && self.y0 <= o.y1 && o.y0 <= self.y1
    }
    fn join(&self, o: &Rect) -> Rect {
        Rect {
            x0: self.x0.min(o.x0),
            y0: self.y0.min(o.y0),
            x1: self.x1.max(o.x1),
            y1: self.y1.max(o.y1),
        }
    }
    /// The box of these blocks' cells at a spacing, widened by a unit of 1e-7°.
    pub fn of_blocks(blocks: &[BlockId], s: Spacing) -> Rect {
        let s = s.ticks();
        let b = BLOCK as i64;
        let (i0, i1) = (
            blocks.iter().map(|x| x.bi).min().unwrap_or(0) as i64 * b,
            (blocks.iter().map(|x| x.bi).max().unwrap_or(0) as i64 + 1) * b,
        );
        let (j0, j1) = (
            blocks.iter().map(|x| x.bj).min().unwrap_or(0) as i64 * b,
            (blocks.iter().map(|x| x.bj).max().unwrap_or(0) as i64 + 1) * b,
        );
        // 1 tick = 1e7 / 432,000 = 625 / 27 units of 1e-7°.
        let lo = |p: i64| ((p * s - s / 2) * 625).div_euclid(27) as i32 - 1;
        let hi = |p: i64| -(((p * s - s / 2) * -625).div_euclid(27)) as i32 + 1;
        Rect {
            x0: lo(j0).max(-1_800_000_000),
            y0: lo(i0).max(-900_000_000),
            x1: hi(j1).min(1_800_000_000),
            y1: hi(i1).min(900_000_000),
        }
    }
}

struct Ring {
    /// (longitude, latitude) in 1e-7°, closed.
    pts: Vec<(i32, i32)>,
    bbox: Rect,
}

struct Area {
    class: u8,
    rings: Vec<Ring>,
    bbox: Rect,
}

struct Line {
    bits: u16,
    pts: Vec<(i32, i32)>,
    bbox: Rect,
}

/// What the OSM read found.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct OsmStats {
    /// Area features meeting the region (closed ways and multipolygons).
    pub areas: u64,
    pub multipolygons: u64,
    /// Rings joined from more than one way.
    pub rings_joined: u64,
    /// Rings dropped: not closed, or with a node the extract lacks.
    pub rings_dropped: u64,
    /// Ways with line bits meeting the region.
    pub lines: u64,
}

/// An OSM extract's areas and lines that meet a region, in paint order.
pub(crate) struct Osm {
    areas: Vec<Area>,
    lines: Vec<Line>,
    pub stats: OsmStats,
}

/// One block's OSM planes.
pub(crate) struct OsmPlanes {
    pub fine: Vec<u8>,
    pub cov: Vec<u8>,
    pub lines: Vec<u16>,
}

/// A classified multipolygon: its id, fine class and member ways (id, inner).
type Multipolygon = (i64, u8, Vec<(i64, bool)>);

/// A way the stage keeps: its area class when it is a closed area, its line bits, its nodes.
struct KeptWay {
    id: i64,
    class: u8,
    bits: u16,
    refs: Vec<i64>,
}

#[derive(Default)]
struct Ways(Vec<KeptWay>);

impl Pass2Sink for Ways {
    fn on_relation_batch(&mut self, _: RelationBatch) -> Result<(), String> {
        Ok(())
    }
    fn on_way_batch(&mut self, batch: WayBatch) -> Result<(), String> {
        for w in batch.ways {
            let c = area_class(|k| w.tags.get(k));
            let closed = c != 0 && w.node_ids.len() >= 4 && w.node_ids.first() == w.node_ids.last();
            self.0.push(KeptWay {
                id: w.way_id,
                class: if closed { c } else { 0 },
                bits: line_bits(|k| w.tags.get(k)),
                refs: w.node_ids,
            });
        }
        Ok(())
    }
    fn on_node_batch(&mut self, _: NodeBatch) -> Result<(), String> {
        Ok(())
    }
}

/// A node's location in 1e-7°, (longitude, latitude).
fn at(c: &mut NodeCursor<'_>, id: i64) -> Option<(i32, i32)> {
    c.get(id)
        .map(|(lat, lon)| (libm::round(lon * 1e7) as i32, libm::round(lat * 1e7) as i32))
}

fn area2(r: &[(i32, i32)]) -> i128 {
    r.windows(2)
        .map(|w| w[0].0 as i128 * w[1].1 as i128 - w[1].0 as i128 * w[0].1 as i128)
        .sum()
}

/// Joins member ways into closed rings, in member order. Returns the rings' node ids, each
/// with the role of its first way (true = inner), and how many rings were joined from more
/// than one way and how many were dropped open.
fn join_rings(ways: &[(&[i64], bool)]) -> (Vec<(Vec<i64>, bool)>, u64, u64) {
    let (mut out, mut joined, mut dropped) = (Vec::new(), 0, 0);
    let mut used = vec![false; ways.len()];
    for i in 0..ways.len() {
        if used[i] || ways[i].0.len() < 2 {
            continue;
        }
        used[i] = true;
        let mut ring = ways[i].0.to_vec();
        loop {
            if ring.len() >= 4 && ring.first() == ring.last() {
                break;
            }
            let end = *ring.last().unwrap();
            let mut found = false;
            for j in 0..ways.len() {
                let w = ways[j].0;
                if used[j] || w.len() < 2 {
                    continue;
                }
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
                joined += 1;
            }
            out.push((ring, ways[i].1));
        } else {
            dropped += 1;
        }
    }
    (out, joined, dropped)
}

fn ingest(e: String) -> Error {
    if e == osm_pbf_ingest::INGEST_CANCELLED {
        Error::Cancelled
    } else {
        Error::Source(format!("OSM: {e}"))
    }
}

impl Osm {
    /// Reads the areas and lines of `pbf` that meet `region`, in two passes: the node
    /// locations and relations (into a node store under `scratch`, in memory for a small
    /// extract), then the ways that meet the region, and every member way of a classified
    /// multipolygon.
    pub fn read(
        pbf: &Path,
        region: Rect,
        threads: Option<usize>,
        scratch: &Path,
        cancel: &CancelToken,
    ) -> Result<Osm> {
        let profile = IngestBuildProfile::DesktopFull;
        let workers = threads.unwrap_or_else(|| profile.worker_threads()).max(1);
        let stop = || cancel.is_cancelled();
        let index = prepare_ingest_index_with_options(
            pbf,
            profile,
            scratch,
            PrepareOptions::new().worker_threads(workers),
            |_| {},
            stop,
        )
        .map_err(ingest)?;
        let mut rels: Vec<Multipolygon> = index
            .supported_relations
            .iter()
            .filter(|r| r.tags.get("type") == Some("multipolygon"))
            .filter_map(|r| {
                let c = area_class(|k| r.tags.get(k));
                (c != 0).then(|| {
                    let m = r
                        .members
                        .iter()
                        .filter(|m| m.member_type == RelationMemberType::Way)
                        .map(|m| (m.member_id, m.role == "inner"))
                        .collect();
                    (r.relation_id, c, m)
                })
            })
            .collect();
        drop(index.supported_relations);
        rels.sort_by_key(|r| r.0);
        let mut members: Vec<i64> = rels.iter().flat_map(|r| r.2.iter().map(|m| m.0)).collect();
        members.sort_unstable();
        members.dedup();
        let members = Arc::new(members);
        let store = Arc::new(index.node_store);
        let (wanted, nodes) = (Arc::clone(&members), Arc::clone(&store));
        let area = [DecimicroBox {
            min_lat: region.y0,
            min_lon: region.x0,
            max_lat: region.y1,
            max_lon: region.x1,
        }];
        let mut ways = Ways::default();
        run_pass2_with_options(
            pbf,
            profile,
            Pass2Options::new()
                .worker_threads(workers)
                .skip_nodes()
                .skip_relations()
                .keep_ways_where(Arc::new(move |w| {
                    if wanted.binary_search(&w.way_id).is_ok() {
                        return true;
                    }
                    if area_class(|k| w.tags.get(k)) == 0 && line_bits(|k| w.tags.get(k)) == 0 {
                        return false;
                    }
                    if !nodes.may_touch(w.node_ids, &area) {
                        return false;
                    }
                    let mut c = nodes.cursor();
                    let pts: Vec<(i32, i32)> =
                        w.node_ids.iter().filter_map(|&id| at(&mut c, id)).collect();
                    !pts.is_empty() && Rect::of(&pts).meets(&region)
                })),
            &mut ways,
            |_| {},
            stop,
        )
        .map_err(ingest)?;
        let mut ways = ways.0;
        ways.sort_by_key(|w| w.id);
        let mut cursor = store.cursor();
        let mut coords = |refs: &[i64]| -> Option<Vec<(i32, i32)>> {
            refs.iter().map(|&r| at(&mut cursor, r)).collect()
        };
        let mut stats = OsmStats::default();
        // (area2, priority, id, area) for the paint order.
        let mut areas: Vec<(i128, u8, i64, Area)> = Vec::new();
        let mut keep_area =
            |id: i64, class: u8, a2: i128, rings: Vec<Ring>, stats: &mut OsmStats| {
                let rings: Vec<Ring> = rings
                    .into_iter()
                    .filter(|r| r.bbox.meets(&region))
                    .collect();
                if let Some(first) = rings.first() {
                    let bbox = rings.iter().fold(first.bbox, |b, r| b.join(&r.bbox));
                    stats.areas += 1;
                    areas.push((
                        a2,
                        FINE_PRI[class as usize],
                        id,
                        Area { class, rings, bbox },
                    ));
                }
            };
        let refs_of = |id: i64| {
            ways.binary_search_by_key(&id, |w| w.id)
                .ok()
                .map(|k| ways[k].refs.as_slice())
        };
        for (rid, class, mem) in &rels {
            let mw: Vec<(&[i64], bool)> = mem
                .iter()
                .filter_map(|&(wid, inner)| refs_of(wid).map(|r| (r, inner)))
                .collect();
            let (rings, joined, dropped) = join_rings(&mw);
            stats.rings_joined += joined;
            stats.rings_dropped += dropped;
            let (mut rr, mut a2) = (Vec::new(), 0i128);
            for (ring, inner) in rings {
                match coords(&ring) {
                    Some(pts) => {
                        let a = area2(&pts).abs();
                        a2 += if inner { -a } else { a };
                        rr.push(Ring {
                            bbox: Rect::of(&pts),
                            pts,
                        });
                    }
                    None => stats.rings_dropped += 1,
                }
            }
            if !rr.is_empty() {
                let before = stats.areas;
                keep_area(-rid, *class, a2.max(0), rr, &mut stats);
                stats.multipolygons += stats.areas - before;
            }
        }
        let mut lines = Vec::new();
        // The ways last, each freed as its features are made.
        for w in ways {
            if w.class != 0 {
                match coords(&w.refs) {
                    Some(pts) => {
                        let ring = Ring {
                            bbox: Rect::of(&pts),
                            pts,
                        };
                        keep_area(
                            w.id,
                            w.class,
                            area2(&ring.pts).abs(),
                            vec![ring],
                            &mut stats,
                        );
                    }
                    None => stats.rings_dropped += 1,
                }
            }
            if w.bits != 0 {
                let mut c = store.cursor();
                let pts: Vec<(i32, i32)> = w.refs.iter().filter_map(|&r| at(&mut c, r)).collect();
                if pts.len() >= 2 {
                    let bbox = Rect::of(&pts);
                    if bbox.meets(&region) {
                        stats.lines += 1;
                        lines.push(Line {
                            bits: w.bits,
                            pts,
                            bbox,
                        });
                    }
                }
            }
        }
        // Paint order: larger areas first, so smaller ones end on top.
        areas.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
        Ok(Osm {
            areas: areas.into_iter().map(|a| a.3).collect(),
            lines,
            stats,
        })
    }

    /// The OSM planes of the blocks `bjs` (sorted) of block row `bi`, in that order.
    pub fn band(&self, s: Spacing, bi: i32, bjs: &[i32]) -> Vec<OsmPlanes> {
        let mut out = Vec::with_capacity(bjs.len());
        let mut at = 0;
        while at < bjs.len() {
            let mut end = at + 1;
            while end < bjs.len() && bjs[end] == bjs[end - 1] + 1 {
                end += 1;
            }
            out.extend(self.run(s, bi, bjs[at], end - at));
            at = end;
        }
        out
    }

    /// The planes of `w` consecutive blocks from `(bi, bj)`.
    fn run(&self, s: Spacing, bi: i32, bj: i32, w: usize) -> Vec<OsmPlanes> {
        let s = s.ticks();
        let b = BLOCK as i64;
        // Cell (0, 0)'s south-west corner, in units; half a sample step is `h`.
        let x0 = (bj as i64 * b * s - s / 2) * TICK_UNIT;
        let y0 = (bi as i64 * b * s - s / 2) * TICK_UNIT;
        let h = s * TICK_UNIT / (2 * SUB as i64);
        let (rows, cols) = (BLOCK * SUB, w * BLOCK * SUB);
        let cell = s * TICK_UNIT;
        let (x1, y1) = (x0 + (w * BLOCK) as i64 * cell, y0 + b * cell);
        // The run's box in 1e-7° (widened by one), to skip what cannot meet it.
        let deg = |u: i64, up: bool| {
            (if up {
                -((-u).div_euclid(OSM_UNIT))
            } else {
                u.div_euclid(OSM_UNIT)
            }) as i32
        };
        let rect = Rect {
            x0: deg(x0, false) - 1,
            y0: deg(y0, false) - 1,
            x1: deg(x1, true) + 1,
            y1: deg(y1, true) + 1,
        };

        // Areas: each edge's crossings of the sample rows, as (row, rank, first column right
        // of the crossing).
        let mut cross: Vec<(u16, u32, i32)> = self
            .areas
            .par_iter()
            .enumerate()
            // A ring wholly left of the run crosses each row an even number of times, all
            // before column 0, so it changes no parity and is skipped like one to the right.
            .filter(|(_, a)| a.bbox.meets(&rect))
            .flat_map_iter(|(rank, a)| {
                let mut v = Vec::new();
                for r in &a.rings {
                    if !r.bbox.meets(&rect) {
                        continue;
                    }
                    for e in r.pts.windows(2) {
                        let (xa, ya) = (e[0].0 as i64 * OSM_UNIT, e[0].1 as i64 * OSM_UNIT);
                        let (xb, yb) = (e[1].0 as i64 * OSM_UNIT, e[1].1 as i64 * OSM_UNIT);
                        if ya == yb {
                            continue;
                        }
                        let (ex1, ey1, ex2, ey2) = if ya < yb {
                            (xa, ya, xb, yb)
                        } else {
                            (xb, yb, xa, ya)
                        };
                        // Rows with ey1 <= y0 + (2r + 1) h < ey2.
                        let first = (ey1 - y0 - h).div_euclid(2 * h)
                            + ((ey1 - y0 - h).rem_euclid(2 * h) != 0) as i64;
                        let last = (ey2 - y0 - h).div_euclid(2 * h)
                            + ((ey2 - y0 - h).rem_euclid(2 * h) != 0) as i64
                            - 1;
                        let (first, last) = (first.max(0), last.min(rows as i64 - 1));
                        if first > last || ex1.min(ex2) >= x1 {
                            continue;
                        }
                        let den = (ey2 - ey1) as i128;
                        for row in first..=last {
                            let ys = y0 + (2 * row + 1) * h;
                            let num = ex1 as i128 * den + (ys - ey1) as i128 * (ex2 - ex1) as i128;
                            let k =
                                (num - (x0 + h) as i128 * den).div_euclid(2 * h as i128 * den) + 1;
                            v.push((row as u16, rank as u32, k.clamp(0, cols as i128) as i32));
                        }
                    }
                }
                v
            })
            .collect();
        cross.par_sort_unstable();
        let mut grid = vec![0u8; rows * cols];
        let mut starts = vec![0usize; rows + 1];
        for &(r, _, _) in &cross {
            starts[r as usize + 1] += 1;
        }
        for r in 0..rows {
            starts[r + 1] += starts[r];
        }
        grid.par_chunks_mut(cols).enumerate().for_each(|(r, line)| {
            let v = &cross[starts[r]..starts[r + 1]];
            let mut i = 0;
            while i < v.len() {
                let rank = v[i].1;
                let mut j = i;
                while j < v.len() && v[j].1 == rank {
                    j += 1;
                }
                let c = self.areas[rank as usize].class;
                let mut q = i;
                while q < j {
                    // Edges wholly right of the run were skipped: their crossings would all
                    // sit at the last column, so an unpaired crossing fills to the end.
                    let end = if q + 1 < j { v[q + 1].2 as usize } else { cols };
                    line[v[q].2 as usize..end].fill(c);
                    q += 2;
                }
                i = j;
            }
        });
        drop(cross);

        // Lines: every cell each segment touches.
        let ncol = (w * BLOCK) as i64;
        let marks = self
            .lines
            .par_iter()
            .filter(|l| l.bbox.meets(&rect))
            .fold(
                || vec![0u16; BLOCK * w * BLOCK],
                |mut g, l| {
                    for e in l.pts.windows(2) {
                        let a = (e[0].0 as i64 * OSM_UNIT, e[0].1 as i64 * OSM_UNIT);
                        let z = (e[1].0 as i64 * OSM_UNIT, e[1].1 as i64 * OSM_UNIT);
                        if a.0.max(z.0) < x0
                            || a.0.min(z.0) >= x1
                            || a.1.max(z.1) < y0
                            || a.1.min(z.1) >= y1
                        {
                            continue;
                        }
                        walk(a, z, x0, y0, cell, ncol, b, |cx, cy| {
                            g[(cy * ncol + cx) as usize] |= l.bits
                        });
                    }
                    g
                },
            )
            .reduce(
                || vec![0u16; BLOCK * w * BLOCK],
                |mut a, b| {
                    a.iter_mut().zip(&b).for_each(|(x, y)| *x |= y);
                    a
                },
            );

        (0..w)
            .map(|k| {
                let mut p = OsmPlanes {
                    fine: vec![0; BLOCK_POSTS],
                    cov: vec![0; BLOCK_POSTS],
                    lines: vec![0; BLOCK_POSTS],
                };
                for pr in 0..BLOCK {
                    for pc in 0..BLOCK {
                        let q = pr * BLOCK + pc;
                        p.lines[q] = marks[pr * w * BLOCK + k * BLOCK + pc];
                        let mut hist = [0u8; 32];
                        for sr in 0..SUB {
                            let line = &grid[(pr * SUB + sr) * cols..];
                            for sc in 0..SUB {
                                hist[line[(k * BLOCK + pc) * SUB + sc] as usize] += 1;
                            }
                        }
                        let mut best = 0usize;
                        for c in 1..32 {
                            let hb = if best == 0 { 0 } else { hist[best] };
                            if hist[c] > hb
                                || (hist[c] == hb && hist[c] > 0 && FINE_PRI[c] > FINE_PRI[best])
                            {
                                best = c;
                            }
                        }
                        p.fine[q] = best as u8;
                        p.cov[q] = (SUB * SUB) as u8 - hist[0];
                    }
                }
                p
            })
            .collect()
    }
}

/// Marks the cells a segment touches, on a grid of `ncol × nrow` cells of `cs` units whose
/// cell (0, 0) starts at (`x0`, `y0`). Crossings are compared exactly in `i128`; through a
/// corner it steps along x first.
#[allow(clippy::too_many_arguments)]
fn walk(
    a: (i64, i64),
    b: (i64, i64),
    x0: i64,
    y0: i64,
    cs: i64,
    ncol: i64,
    nrow: i64,
    mut mark: impl FnMut(i64, i64),
) {
    let cell = |v: i64, o: i64| (v - o).div_euclid(cs);
    let (mut cx, mut cy) = (cell(a.0, x0), cell(a.1, y0));
    let (ex, ey) = (cell(b.0, x0), cell(b.1, y0));
    let (dx, dy) = ((b.0 - a.0) as i128, (b.1 - a.1) as i128);
    let (sx, sy) = (dx.signum() as i64, dy.signum() as i64);
    let inside = |x: i64, y: i64| x >= 0 && y >= 0 && x < ncol && y < nrow;
    if inside(cx, cy) {
        mark(cx, cy);
    }
    while (cx, cy) != (ex, ey) {
        let bx = x0 as i128 + (cx + (sx > 0) as i64) as i128 * cs as i128;
        let by = y0 as i128 + (cy + (sy > 0) as i64) as i128 * cs as i128;
        let step_x = if sx == 0 {
            false
        } else if sy == 0 {
            true
        } else {
            let lx = (bx - a.0 as i128) * dy.abs() * sx as i128;
            let ly = (by - a.1 as i128) * dx.abs() * sy as i128;
            lx <= ly
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

#[cfg(test)]
mod tests {
    use super::*;

    fn tags<'a>(t: &'a [(&'a str, &'a str)]) -> impl Fn(&str) -> Option<&'a str> {
        move |k| t.iter().find(|(a, _)| *a == k).map(|(_, v)| *v)
    }

    #[test]
    fn classes() {
        assert_eq!(area_class(tags(&[("landuse", "forest")])), FOREST);
        assert_eq!(
            area_class(tags(&[("natural", "wetland"), ("wetland", "swamp")])),
            2
        );
        assert_eq!(area_class(tags(&[("building", "yes")])), 14);
        assert_eq!(area_class(tags(&[("building", "no")])), 0);
        assert_eq!(
            area_class(tags(&[("natural", "water"), ("landuse", "forest")])),
            22
        );
        assert_eq!(
            line_bits(tags(&[
                ("highway", "path"),
                ("sac_scale", "mountain_hiking")
            ])),
            L::PATH_MOUNTAIN
        );
        assert_eq!(
            line_bits(tags(&[("highway", "motorway"), ("bridge", "yes")])),
            0
        );
        assert_eq!(line_bits(tags(&[("waterway", "river")])), L::RIVER);
        assert_eq!(
            line_bits(tags(&[("waterway", "river"), ("tunnel", "culvert")])),
            0
        );
        assert_eq!(
            line_bits(tags(&[("highway", "footway"), ("bridge", "yes")])),
            L::PATH
        );
        assert_eq!(
            line_bits(tags(&[("natural", "cliff"), ("barrier", "wall")])),
            L::CLIFF | L::FENCE
        );
        for (fine, name) in FINE_NAMES.iter().enumerate() {
            assert!(FINE_TO_COVER[fine] <= 15, "{name}");
        }
    }

    #[test]
    fn joins_rings_in_member_order() {
        let w: Vec<(&[i64], bool)> = vec![
            (&[1, 2, 3], false),
            (&[5, 4, 3], false),
            (&[5, 6, 1], false),
            (&[7, 8, 9, 7], true),
            (&[10, 11], false),
        ];
        let (r, joined, dropped) = join_rings(&w);
        assert_eq!(r.len(), 2);
        assert_eq!(r[0].0, vec![1, 2, 3, 4, 5, 6, 1]);
        assert!(r[1].1);
        assert_eq!((joined, dropped), (1, 1));
    }

    #[test]
    fn supercover_is_4_connected() {
        for &(a, b) in &[
            ((5, 5), (95, 37)),
            ((95, 5), (5, 95)),
            ((0, 0), (40, 40)),
            ((33, 3), (33, 77)),
            ((7, 70), (91, 12)),
        ] {
            let mut cells = Vec::new();
            walk(a, b, 0, 0, 10, 100, 100, |x, y| cells.push((x, y)));
            for w in cells.windows(2) {
                assert_eq!(
                    (w[0].0 - w[1].0).abs() + (w[0].1 - w[1].1).abs(),
                    1,
                    "{a:?} {b:?}"
                );
            }
            assert_eq!(cells[0], (a.0 / 10, a.1 / 10));
            assert_eq!(*cells.last().unwrap(), (b.0 / 10, b.1 / 10));
        }
    }
}

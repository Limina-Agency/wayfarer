//! The OSM stage end to end, on a synthetic extract: the land-cover override and its
//! exceptions, multipolygon assembly, the confidence bit, linear features, and the same bytes
//! on any thread count and for any subset of blocks.

mod common;

use common::{Place, Sample, Spec, tiff};
use std::io::Cursor;
use std::path::Path;
use wayfarer_terrain::lattice::BLOCK;
use wayfarer_terrain::lines;
use wayfarer_terrain::terrain::Terrain;
use wayfarer_terrain_compiler::{
    CancelToken, Options, Progress, Region, SourceFile, Sources, Stage, compile,
};

const REGION: Region = Region::Bbox {
    south: 49.50,
    west: 20.00,
    north: 49.53,
    east: 20.05,
};

// A minimal PBF writer: one block each of nodes, ways and relations.

fn varint(mut v: u64, out: &mut Vec<u8>) {
    while v >= 0x80 {
        out.push(v as u8 | 0x80);
        v >>= 7;
    }
    out.push(v as u8);
}
fn bytes(field: u32, b: &[u8], out: &mut Vec<u8>) {
    varint((field << 3 | 2) as u64, out);
    varint(b.len() as u64, out);
    out.extend_from_slice(b);
}
fn uint(field: u32, v: u64, out: &mut Vec<u8>) {
    varint((field << 3) as u64, out);
    varint(v, out);
}
fn zigzag(v: i64) -> u64 {
    ((v << 1) ^ (v >> 63)) as u64
}
fn blob(kind: &str, payload: &[u8], out: &mut Vec<u8>) {
    let mut b = Vec::new();
    bytes(1, payload, &mut b);
    let mut h = Vec::new();
    bytes(1, kind.as_bytes(), &mut h);
    uint(3, b.len() as u64, &mut h);
    out.extend_from_slice(&(h.len() as u32).to_be_bytes());
    out.extend_from_slice(&h);
    out.extend_from_slice(&b);
}

type Tags<'a> = &'a [(&'a str, &'a str)];

struct Way<'a> {
    id: i64,
    nodes: Vec<i64>,
    tags: Tags<'a>,
}

struct Rel<'a> {
    id: i64,
    tags: Tags<'a>,
    /// (role, way id)
    members: Vec<(&'a str, i64)>,
}

fn write_pbf(path: &Path, nodes: &[(i64, f64, f64)], ways: &[Way], rels: &[Rel]) {
    let mut file = Vec::new();
    let mut header = Vec::new();
    bytes(4, b"OsmSchema-V0.6", &mut header);
    blob("OSMHeader", &header, &mut file);
    let mut strings = vec![String::new()];
    let mut sid = |s: &str| match strings.iter().position(|c| c == s) {
        Some(i) => i as u64,
        None => {
            strings.push(s.to_string());
            strings.len() as u64 - 1
        }
    };
    let tag_fields = |tags: Tags, out: &mut Vec<u8>, sid: &mut dyn FnMut(&str) -> u64| {
        let (mut k, mut v) = (Vec::new(), Vec::new());
        for (a, b) in tags {
            varint(sid(a), &mut k);
            varint(sid(b), &mut v);
        }
        bytes(2, &k, out);
        bytes(3, &v, out);
    };
    let mut ng = Vec::new();
    for &(id, lat, lon) in nodes {
        let mut n = Vec::new();
        uint(1, zigzag(id), &mut n);
        uint(8, zigzag((lat * 1e7).round() as i64), &mut n);
        uint(9, zigzag((lon * 1e7).round() as i64), &mut n);
        bytes(1, &n, &mut ng);
    }
    let mut wg = Vec::new();
    for w in ways {
        let mut e = Vec::new();
        uint(1, w.id as u64, &mut e);
        tag_fields(w.tags, &mut e, &mut sid);
        let (mut refs, mut prev) = (Vec::new(), 0);
        for &n in &w.nodes {
            varint(zigzag(n - prev), &mut refs);
            prev = n;
        }
        bytes(8, &refs, &mut e);
        bytes(3, &e, &mut wg);
    }
    let mut rg = Vec::new();
    for r in rels {
        let mut e = Vec::new();
        uint(1, r.id as u64, &mut e);
        tag_fields(r.tags, &mut e, &mut sid);
        let (mut roles, mut ids, mut types, mut prev) = (Vec::new(), Vec::new(), Vec::new(), 0);
        for &(role, id) in &r.members {
            varint(sid(role), &mut roles);
            varint(zigzag(id - prev), &mut ids);
            prev = id;
            varint(1, &mut types);
        }
        bytes(8, &roles, &mut e);
        bytes(9, &ids, &mut e);
        bytes(10, &types, &mut e);
        bytes(4, &e, &mut rg);
    }
    let mut table = Vec::new();
    for s in &strings {
        bytes(1, s.as_bytes(), &mut table);
    }
    for g in [ng, wg, rg] {
        if !g.is_empty() {
            let mut b = Vec::new();
            bytes(1, &table, &mut b);
            bytes(2, &g, &mut b);
            blob("OSMData", &b, &mut file);
        }
    }
    std::fs::write(path, file).unwrap();
}

/// The extract: a forest with a lake in it, a scrub multipolygon (two ways joined into the
/// outer ring, an inner hole, an open member) across the 1,300 m line, bare rock, a building
/// far away, and lines: a mountain path, a fence, a river in a culvert, a railway on a bridge.
fn extract(path: &Path) {
    let mut nodes = Vec::new();
    let mut ways = Vec::new();
    let mut id = 0i64;
    let mut node = |lat: f64, lon: f64, nodes: &mut Vec<(i64, f64, f64)>| {
        id += 1;
        nodes.push((id, lat, lon));
        id
    };
    let mut rect = |s: f64, w: f64, n: f64, e: f64, nodes: &mut Vec<_>| {
        let a = node(s, w, nodes);
        let b = node(s, e, nodes);
        let c = node(n, e, nodes);
        let d = node(n, w, nodes);
        vec![a, b, c, d, a]
    };
    let forest = rect(49.505, 20.005, 49.515, 20.015, &mut nodes);
    let lake = rect(49.508, 20.008, 49.511, 20.011, &mut nodes);
    let outer = rect(49.517, 20.010, 49.525, 20.035, &mut nodes);
    let hole = rect(49.519, 20.012, 49.522, 20.016, &mut nodes);
    let rock = rect(49.5005, 20.030, 49.5035, 20.040, &mut nodes);
    let far = rect(50.5, 21.5, 50.501, 21.501, &mut nodes);
    let mut line = |pts: &[(f64, f64)], nodes: &mut Vec<_>| {
        pts.iter()
            .map(|&(a, b)| node(a, b, nodes))
            .collect::<Vec<i64>>()
    };
    let trail = line(&[(49.527, 20.001), (49.527, 20.049)], &mut nodes);
    let fence = line(&[(49.501, 20.002), (49.529, 20.002)], &mut nodes);
    let culvert = line(&[(49.501, 20.045), (49.529, 20.045)], &mut nodes);
    let bridge = line(&[(49.5015, 20.001), (49.5015, 20.049)], &mut nodes);
    let open = line(&[(49.526, 20.040), (49.5265, 20.041)], &mut nodes);
    ways.push(Way {
        id: 10,
        nodes: forest,
        tags: &[("landuse", "forest")],
    });
    ways.push(Way {
        id: 11,
        nodes: lake,
        tags: &[("natural", "water")],
    });
    // The outer ring in two halves, SW-SE-NE and NE-NW-SW.
    ways.push(Way {
        id: 12,
        nodes: outer[..3].to_vec(),
        tags: &[],
    });
    ways.push(Way {
        id: 13,
        nodes: outer[2..].to_vec(),
        tags: &[],
    });
    ways.push(Way {
        id: 14,
        nodes: hole,
        tags: &[],
    });
    ways.push(Way {
        id: 15,
        nodes: open,
        tags: &[],
    });
    ways.push(Way {
        id: 16,
        nodes: rock,
        tags: &[("natural", "bare_rock")],
    });
    ways.push(Way {
        id: 17,
        nodes: far,
        tags: &[("building", "yes")],
    });
    ways.push(Way {
        id: 20,
        nodes: trail,
        tags: &[("highway", "path"), ("sac_scale", "mountain_hiking")],
    });
    ways.push(Way {
        id: 21,
        nodes: fence,
        tags: &[("barrier", "fence")],
    });
    ways.push(Way {
        id: 22,
        nodes: culvert,
        tags: &[("waterway", "river"), ("tunnel", "culvert")],
    });
    ways.push(Way {
        id: 23,
        nodes: bridge,
        tags: &[("railway", "rail"), ("bridge", "yes")],
    });
    let rels = [Rel {
        id: 30,
        tags: &[("type", "multipolygon"), ("natural", "scrub")],
        members: vec![("outer", 12), ("outer", 13), ("inner", 14), ("outer", 15)],
    }];
    write_pbf(path, &nodes, &ways, &rels);
}

/// Heights: 1,400 m west of 20.02° E and 1,000 m east of it. WorldCover: grass everywhere.
fn sources(pbf: &Path) -> Sources {
    let glo = Spec {
        width: 600,
        height: 600,
        tile: 256,
        sample: Sample::F32,
        compression: 8,
        predictor: 1,
        big_endian: false,
        bigtiff: false,
        place: Place::Geo {
            lon: 19.95,
            lat: 49.6,
            dx: 1.0 / 3600.0,
            dy: 1.0 / 3600.0,
            point: true,
        },
        nodata: None,
    };
    let glo = tiff(&glo, &|_, c| {
        if 19.95 + c as f64 / 3600.0 < 20.02 {
            1400.0
        } else {
            1000.0
        }
    });
    let wc = Spec {
        width: 1200,
        height: 1200,
        tile: 256,
        sample: Sample::U8,
        compression: 8,
        predictor: 1,
        big_endian: false,
        bigtiff: false,
        place: Place::Geo {
            lon: 19.95,
            lat: 49.6,
            dx: 1.0 / 12000.0,
            dy: 1.0 / 12000.0,
            point: false,
        },
        nodata: Some("0"),
    };
    let wc = tiff(&wc, &|_, _| 30.0);
    Sources {
        heights: vec![vec![SourceFile::new("glo.tif", Cursor::new(glo))]],
        land_cover: vec![SourceFile::new("wc.tif", Cursor::new(wc))],
        osm: None,
        attribution: "synthetic".into(),
    }
    .with_osm(pbf)
}

fn run(
    pbf: &Path,
    region: &Region,
    threads: usize,
) -> (wayfarer_terrain_compiler::Report, Terrain, Vec<u8>) {
    let opts = Options {
        threads: Some(threads),
        scratch_dir: Some(pbf.parent().unwrap().to_path_buf()),
        ..Options::default()
    };
    let mut stages = Vec::new();
    let (r, out) = compile(
        sources(pbf),
        region,
        &opts,
        Cursor::new(Vec::new()),
        &CancelToken::new(),
        |p: Progress| stages.push(p.stage),
    )
    .unwrap();
    assert_eq!(stages[0], Stage::ReadingOsm);
    assert!(stages[1..].iter().all(|&s| s == Stage::Blocks));
    let bytes = out.into_inner();
    (r, Terrain::read(&bytes).unwrap(), bytes)
}

/// The block index and post of the post nearest a point, at 1″.
fn post(t: &Terrain, lat: f64, lon: f64) -> (usize, usize) {
    let (i, j) = ((lat * 3600.0).round() as i64, (lon * 3600.0).round() as i64);
    let b = BLOCK as i64;
    let k = t
        .blocks()
        .iter()
        .position(|x| x.bi as i64 == i.div_euclid(b) && x.bj as i64 == j.div_euclid(b))
        .unwrap();
    (k, (i.rem_euclid(b) * b + j.rem_euclid(b)) as usize)
}

#[test]
fn overrides_land_cover_and_marks_lines() {
    let dir = tempfile::tempdir().unwrap();
    let pbf = dir.path().join("t.osm.pbf");
    extract(&pbf);
    let (r, t, bytes) = run(&pbf, &REGION, 2);
    let cover = |lat, lon| {
        let (k, q) = post(&t, lat, lon);
        (t.cover(k)[q], t.conf(k)[q])
    };
    assert_eq!(cover(49.512, 20.025), (3, 1), "WorldCover, pure");
    assert_eq!(cover(49.506, 20.006), (3, 1), "forest does not override");
    assert_eq!(
        cover(49.5095, 20.0095),
        (8, 0),
        "the lake on top, against WorldCover"
    );
    assert_eq!(
        cover(49.518, 20.013),
        (12, 0),
        "scrub at 1,400 m is dwarf pine"
    );
    assert_eq!(cover(49.5205, 20.014), (3, 1), "the hole");
    assert_eq!(
        cover(49.521, 20.030),
        (3, 1),
        "scrub at 1,000 m does not override"
    );
    assert_eq!(cover(49.502, 20.035), (13, 0), "bare rock");

    let lines_at = |lat, lon| {
        let (k, q) = post(&t, lat, lon);
        t.lines(k)[q]
    };
    assert_eq!(lines_at(49.527, 20.03), lines::PATH_MOUNTAIN);
    assert_eq!(lines_at(49.51, 20.002), lines::FENCE);
    assert_eq!(
        lines_at(49.527, 20.002),
        lines::FENCE | lines::PATH_MOUNTAIN
    );
    assert_eq!(lines_at(49.51, 20.03), 0);
    let all = (0..t.blocks().len()).fold(0u16, |a, k| t.lines(k).iter().fold(a, |a, &l| a | l));
    assert_eq!(
        all,
        lines::PATH_MOUNTAIN | lines::FENCE,
        "no culverted river, no bridged railway"
    );
    // The path runs along one row of posts, one post per column it spans.
    let path_posts: i64 = (0..t.blocks().len())
        .map(|k| {
            t.lines(k)
                .iter()
                .filter(|&&l| l & lines::PATH_MOUNTAIN != 0)
                .count() as i64
        })
        .sum();
    let col = |lon: f64| (lon * 3600.0).round() as i64;
    assert_eq!(path_posts, col(20.049) - col(20.001) + 1);

    let o = r.osm.as_ref().unwrap();
    assert_eq!(
        (
            o.areas,
            o.multipolygons,
            o.rings_joined,
            o.rings_dropped,
            o.lines
        ),
        (4, 1, 1, 1, 2)
    );
    assert!(r.posts_with_osm_cover > 0 && r.posts_with_lines > 0);
    assert!(t.sources().ends_with("\nosm: t.osm.pbf"), "{}", t.sources());
    assert!(t.sources().contains("OpenStreetMap contributors, ODbL"));

    // The same bytes on another thread count, and the same blocks from a subset.
    let (_, _, again) = run(&pbf, &REGION, 1);
    assert_eq!(again, bytes);
    let pick = vec![t.blocks()[4], t.blocks()[0]];
    let (_, sub, _) = run(&pbf, &Region::Blocks(pick.clone()), 3);
    for (n, b) in sub.blocks().iter().enumerate() {
        let k = t.blocks().iter().position(|x| x == b).unwrap();
        assert_eq!(sub.cover(n), t.cover(k));
        assert_eq!(sub.conf(n), t.conf(k));
        assert_eq!(sub.lines(n), t.lines(k));
    }
    // The scratch directory is cleaned up.
    let left: Vec<_> = std::fs::read_dir(dir.path())
        .unwrap()
        .map(|e| e.unwrap().file_name())
        .collect();
    assert_eq!(left.len(), 1, "{left:?}");
}

#[test]
fn refuses_an_extract_it_cannot_read() {
    let dir = tempfile::tempdir().unwrap();
    let pbf = dir.path().join("junk.osm.pbf");
    std::fs::write(&pbf, b"not a pbf").unwrap();
    let r = compile(
        sources(&pbf),
        &REGION,
        &Options::default(),
        Cursor::new(Vec::new()),
        &CancelToken::new(),
        |_| {},
    );
    assert!(
        matches!(r, Err(wayfarer_terrain_compiler::Error::Source(_))),
        "{:?}",
        r.err()
    );
    let missing = dir.path().join("missing.osm.pbf");
    assert!(
        compile(
            sources(&missing),
            &REGION,
            &Options::default(),
            Cursor::new(Vec::new()),
            &CancelToken::new(),
            |_| {}
        )
        .is_err()
    );
}

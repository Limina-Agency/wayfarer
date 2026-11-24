//! E10: a sparse semantic voxel graph (contract v0 interchange) and its deterministic projection
//! into two ordinary layers: a hazard grid (rubble, windthrow, flood per lattice cell) and a
//! feature layer (blocked edges as debris, fallen-tree or washout lines; traversable edges as
//! observed passages). Fusion only ever reads layers; the projection is the importer, and each
//! output names the graph's hash as its parent.

use crate::lattice::{BLOCK, BlockId};
use crate::layers::{FEAT_DEBRIS, FEAT_FALLEN, FEAT_PASSAGE, FEAT_WASHOUT, Feature, GridBlock, Kind, Layer, Manifest};

// Voxel classes
pub const V_GROUND: u8 = 1;
pub const V_RUBBLE: u8 = 2;
pub const V_DEBRIS: u8 = 3;
pub const V_TRUNK: u8 = 4;
pub const V_WATER: u8 = 5;
pub const V_VEG_LOW: u8 = 6;
#[allow(dead_code)]
pub const V_VEG_HIGH: u8 = 7;
#[allow(dead_code)]
pub const V_STRUCTURE: u8 = 8;
pub const V_DROP: u8 = 9;

pub const E_TRAVERSABLE: u8 = 1;
pub const E_BLOCKED: u8 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Node {
    pub id: u64,
    pub lat: i32,
    pub lon: i32,
    pub h_dm: i32,
    pub class: u8,
    pub conf: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Edge {
    pub a: u64,
    pub b: u64,
    pub kind: u8,
    pub conf: u8,
}

pub struct Graph {
    pub nodes: Vec<Node>,
    pub edges: Vec<Edge>,
    pub observed_at: i64,
}

/// Number of voxels in a cell for full weight; fewer scale the cell's confidence down linearly.
pub const N_FULL: u32 = 8;

impl Graph {
    /// Canonical: nodes by id (unique), edges with a < b, sorted, unique, endpoints present.
    pub fn canonicalise(&mut self) -> Result<(), String> {
        self.nodes.sort();
        if self.nodes.windows(2).any(|w| w[0].id == w[1].id) {
            return Err("duplicate node id".into());
        }
        for e in self.edges.iter_mut() {
            if e.a > e.b {
                std::mem::swap(&mut e.a, &mut e.b);
            }
        }
        self.edges.sort();
        self.edges.dedup();
        for e in &self.edges {
            if e.a == e.b || self.node(e.a).is_none() || self.node(e.b).is_none() {
                return Err("edge with a missing or repeated endpoint".into());
            }
        }
        Ok(())
    }
    pub fn node(&self, id: u64) -> Option<&Node> {
        self.nodes.binary_search_by_key(&id, |n| n.id).ok().map(|k| &self.nodes[k])
    }
    pub fn hash(&self) -> [u8; 32] {
        let mut h = blake3::Hasher::new();
        h.update(b"WTVOXEL\0");
        h.update(&self.observed_at.to_le_bytes());
        h.update(&(self.nodes.len() as u64).to_le_bytes());
        for n in &self.nodes {
            h.update(&n.id.to_le_bytes());
            h.update(&n.lat.to_le_bytes());
            h.update(&n.lon.to_le_bytes());
            h.update(&n.h_dm.to_le_bytes());
            h.update(&[n.class, n.conf]);
        }
        h.update(&(self.edges.len() as u64).to_le_bytes());
        for e in &self.edges {
            h.update(&e.a.to_le_bytes());
            h.update(&e.b.to_le_bytes());
            h.update(&[e.kind, e.conf]);
        }
        *h.finalize().as_bytes()
    }
}

/// Post of an E7 coordinate at spacing s ticks: the cell [p - s/2, p + s/2) holding it, exactly.
pub fn post_of_e7(e7: i32, s: i64) -> i64 {
    (e7 as i64 * 27 + (s / 2) * 625).div_euclid(s * 625)
}

fn round_div(n: u64, d: u64) -> u64 {
    (2 * n + d) / (2 * d)
}

/// Project into (hazard layer with channels flood 1, rubble 2, windthrow 3; feature layer).
/// `template` carries the producer fields (model id, times, licence); kind and channels are set here.
pub fn project(g: &Graph, s: i64, template: &Manifest) -> (Layer, Layer) {
    let parent = g.hash();
    // per post: (sum conf all, sum conf flood, rubble, windthrow, count)
    let mut cells: std::collections::BTreeMap<(i64, i64), [u64; 5]> = Default::default();
    for n in &g.nodes {
        let e = cells.entry((post_of_e7(n.lat, s), post_of_e7(n.lon, s))).or_default();
        let c = n.conf as u64;
        e[0] += c;
        match n.class {
            V_WATER => e[1] += c,
            V_RUBBLE | V_DEBRIS => e[2] += c,
            V_TRUNK => e[3] += c,
            _ => {}
        }
        e[4] += 1;
    }
    let mut blocks: std::collections::BTreeMap<BlockId, GridBlock> = Default::default();
    for (&(i, j), v) in &cells {
        if v[0] == 0 {
            continue;
        }
        let id = BlockId { bi: i.div_euclid(BLOCK as i64) as i32, bj: j.div_euclid(BLOCK as i64) as i32 };
        let q = (i.rem_euclid(BLOCK as i64) * BLOCK as i64 + j.rem_euclid(BLOCK as i64)) as usize;
        let b = blocks.entry(id).or_insert_with(|| GridBlock::new(id, 3));
        b.set_present(q);
        b.conf[q] = (255 * v[4].min(N_FULL as u64) / N_FULL as u64) as u8;
        for ch in 0..3 {
            b.vals[ch][q] = round_div(255 * v[ch + 1], v[0]) as u16;
        }
    }
    let mut hm = template.clone();
    hm.kind = Kind::Hazard;
    hm.channels = vec![1, 2, 3];
    hm.parents = vec![parent];
    hm.observed_at = g.observed_at;
    let mut haz = Layer { m: hm, blocks: blocks.into_values().collect(), features: vec![] };
    haz.canonicalise().expect("projected hazard layer");

    let mut feats = Vec::new();
    for e in &g.edges {
        let (a, b) = (g.node(e.a).unwrap(), g.node(e.b).unwrap());
        let class = match e.kind {
            E_BLOCKED => {
                if a.class == V_DROP || b.class == V_DROP {
                    FEAT_WASHOUT
                } else if a.class == V_TRUNK || b.class == V_TRUNK {
                    FEAT_FALLEN
                } else {
                    FEAT_DEBRIS
                }
            }
            E_TRAVERSABLE => FEAT_PASSAGE,
            _ => continue,
        };
        feats.push(Feature { class, conf: e.conf.min(a.conf).min(b.conf), pts: vec![(a.lat, a.lon), (b.lat, b.lon)] });
    }
    let mut fm = template.clone();
    fm.kind = Kind::Feature;
    fm.channels = vec![];
    fm.parents = vec![parent];
    fm.observed_at = g.observed_at;
    let mut fl = Layer { m: fm, blocks: vec![], features: feats };
    fl.canonicalise().expect("projected feature layer");
    (haz, fl)
}

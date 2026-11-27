//! Terrain layer files (`docs/design/terrain-layers.md`, contract v0).
//!
//! A layer is what a producer (a model, a host, a drone survey, a voxel-graph projection) hands
//! to fusion: values per lattice post with a per-post confidence, or a list of linear features,
//! plus a manifest that says where it came from and when it holds. Its identity is the blake3
//! of its canonical form (manifest, then every present post's values in block-key order, then
//! the features sorted by their bytes), not of the file bytes, so a writer may pick any plane
//! coding and the hash stays.
//!
//! Producers build a [`Layer`], call [`Layer::canonicalise`] (which validates it), and write it
//! with [`Layer::write`]; [`Layer::read`] checks a file completely. The one float step is the
//! producer's quantiser [`quantise_probs`], whose result is exact in f64 and so the same in any
//! language that follows its steps.

use crate::codec::{self, Reader, put_str, put_varint, unzigzag, zigzag};
use crate::lattice::{BLOCK_POSTS, BlockId, TICKS_PER_DEG};
use crate::{Error, Result};

pub const MAJOR: u16 = 0;
pub const MINOR: u16 = 1;
/// Magic of the manifest (and of the hashed canonical form).
pub const MAGIC: &[u8; 8] = b"WTLAYER\0";
/// Magic of a layer file.
pub const FILE_MAGIC: &[u8; 4] = b"WTL0";

/// Land-cover classes: the 4-bit codes 1-15. In a class layer, slot 0 is KEEP (the mass that
/// defers to whatever the cell already is); in the fused state slot 0 is "no data".
pub const NCLASS: usize = 16;
/// The class probabilities in a layer sum to this.
pub const Q: u32 = 255;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Kind {
    Class = 1,
    Hazard = 2,
    Level = 3,
    Feature = 4,
}

impl Kind {
    pub fn from_u8(v: u8) -> Option<Kind> {
        Some(match v {
            1 => Kind::Class,
            2 => Kind::Hazard,
            3 => Kind::Level,
            4 => Kind::Feature,
            _ => return None,
        })
    }
}

/// Hazard channels: probability in 1/255 that the hazard is present in the cell. The slot of
/// a hazard is its index here.
pub const HAZARDS: [(u16, &str); 8] = [
    (1, "flood"),
    (2, "rubble"),
    (3, "windthrow"),
    (4, "fire"),
    (5, "snow_cover"),
    (6, "ice"),
    (7, "landslide"),
    (8, "closure"),
];
pub const NHAZ: usize = HAZARDS.len();
/// Level channels: u16 in a fixed unit per code.
pub const LEVELS: [(u16, &str); 3] = [
    (1, "water_depth_cm"),
    (2, "snow_depth_cm"),
    (3, "canopy_height_dm"),
];
pub const NLEV: usize = LEVELS.len();

/// Feature classes: format-4.md's `BarrierLines` codes 1-16, plus the ones surveys add.
/// Debris or rubble line (edge of a pile), not crossable by vehicles.
pub const FEAT_DEBRIS: u8 = 17;
/// Fallen trees across the way.
pub const FEAT_FALLEN: u8 = 18;
/// Washout or collapse: a gap in a way, not crossable.
pub const FEAT_WASHOUT: u8 = 19;
/// An observed passage (a way through), followed like a path.
pub const FEAT_PASSAGE: u8 = 20;

/// Licence of a layer: may ship, or research only (refused unless research layers are allowed).
pub const LICENCE_OPEN: u8 = 0;
pub const LICENCE_RESEARCH: u8 = 1;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Manifest {
    pub kind: Kind,
    /// [`LICENCE_OPEN`] or [`LICENCE_RESEARCH`].
    pub licence: u8,
    /// Spacing in ticks; fusion refuses a layer whose spacing differs from the surface's.
    pub s_lat: u32,
    pub s_lon: u32,
    /// UTC seconds. Fusion order is (observed_at, content hash).
    pub observed_at: i64,
    pub valid_from: i64,
    pub valid_until: i64,
    /// Confidence fades linearly to 0 between `fade_from` and `valid_until`; = valid_until for no fade.
    pub fade_from: i64,
    /// Sorted, unique. Class and feature layers: empty.
    pub channels: Vec<u16>,
    pub model_id: String,
    pub model_version: String,
    pub weights: [u8; 32],
    pub dataset_manifest: [u8; 32],
    pub eval_set: [u8; 32],
    /// Expected calibration error of the confidence on `eval_set`, per mille; 0xFFFF = not measured.
    pub ece_permille: u16,
    pub backend: String,
    pub attribution: String,
    /// Content hashes of the inputs (imagery, DEM tiles), sorted.
    pub inputs: Vec<[u8; 32]>,
    /// Layers or graphs this one was derived from, sorted.
    pub parents: Vec<[u8; 32]>,
}

impl Manifest {
    pub fn encode(&self) -> Vec<u8> {
        let mut o = Vec::new();
        o.extend_from_slice(MAGIC);
        o.extend_from_slice(&MAJOR.to_le_bytes());
        o.extend_from_slice(&MINOR.to_le_bytes());
        o.push(self.kind as u8);
        o.push(self.licence);
        o.extend_from_slice(&self.s_lat.to_le_bytes());
        o.extend_from_slice(&self.s_lon.to_le_bytes());
        for t in [
            self.observed_at,
            self.valid_from,
            self.valid_until,
            self.fade_from,
        ] {
            o.extend_from_slice(&t.to_le_bytes());
        }
        put_varint(&mut o, self.channels.len() as u64);
        for c in &self.channels {
            o.extend_from_slice(&c.to_le_bytes());
        }
        put_str(&mut o, &self.model_id);
        put_str(&mut o, &self.model_version);
        o.extend_from_slice(&self.weights);
        o.extend_from_slice(&self.dataset_manifest);
        o.extend_from_slice(&self.eval_set);
        o.extend_from_slice(&self.ece_permille.to_le_bytes());
        put_str(&mut o, &self.backend);
        put_str(&mut o, &self.attribution);
        for list in [&self.inputs, &self.parents] {
            put_varint(&mut o, list.len() as u64);
            for h in list.iter() {
                o.extend_from_slice(h);
            }
        }
        o
    }

    fn decode(r: &mut Reader) -> Result<Manifest> {
        if r.take(8)? != MAGIC {
            return Err(Error::Magic("terrain layer manifest"));
        }
        let major = r.u16()?;
        let _minor = r.u16()?;
        if major != MAJOR {
            return Err(Error::Version {
                what: "layer contract",
                found: major,
                known: MAJOR,
            });
        }
        let kind = Kind::from_u8(r.u8()?).ok_or_else(|| Error::invalid("unknown layer kind"))?;
        let licence = r.u8()?;
        let (s_lat, s_lon) = (r.u32()?, r.u32()?);
        let (observed_at, valid_from, valid_until, fade_from) =
            (r.i64()?, r.i64()?, r.i64()?, r.i64()?);
        let nch = r.len()?;
        let channels = (0..nch).map(|_| r.u16()).collect::<Result<Vec<_>>>()?;
        let model_id = r.str()?;
        let model_version = r.str()?;
        let (weights, dataset_manifest, eval_set) = (r.h32()?, r.h32()?, r.h32()?);
        let ece_permille = r.u16()?;
        let backend = r.str()?;
        let attribution = r.str()?;
        let mut lists = [Vec::new(), Vec::new()];
        for l in lists.iter_mut() {
            let n = r.len()?;
            for _ in 0..n {
                l.push(r.h32()?);
            }
        }
        let [inputs, parents] = lists;
        let m = Manifest {
            kind,
            licence,
            s_lat,
            s_lon,
            observed_at,
            valid_from,
            valid_until,
            fade_from,
            channels,
            model_id,
            model_version,
            weights,
            dataset_manifest,
            eval_set,
            ece_permille,
            backend,
            attribution,
            inputs,
            parents,
        };
        m.check()?;
        Ok(m)
    }

    pub fn check(&self) -> Result<()> {
        let d = TICKS_PER_DEG as u32;
        if self.s_lat == 0
            || self.s_lon == 0
            || !d.is_multiple_of(self.s_lat)
            || !d.is_multiple_of(self.s_lon)
        {
            return Err(Error::invalid(
                "spacing must divide a degree (432,000 ticks)",
            ));
        }
        if self.licence > LICENCE_RESEARCH {
            return Err(Error::invalid("unknown licence code"));
        }
        if !(self.valid_from <= self.fade_from
            && self.fade_from <= self.valid_until
            && self.valid_from < self.valid_until)
        {
            return Err(Error::invalid(
                "need valid_from <= fade_from <= valid_until and a non-empty window",
            ));
        }
        if self.channels.windows(2).any(|w| w[0] >= w[1]) {
            return Err(Error::invalid("channels must be sorted and unique"));
        }
        if self.inputs.windows(2).any(|w| w[0] >= w[1])
            || self.parents.windows(2).any(|w| w[0] >= w[1])
        {
            return Err(Error::invalid(
                "inputs and parents must be sorted and unique",
            ));
        }
        match self.kind {
            Kind::Class | Kind::Feature if !self.channels.is_empty() => {
                Err(Error::invalid("class and feature layers have no channels"))
            }
            Kind::Hazard | Kind::Level if self.channels.is_empty() => {
                Err(Error::invalid("hazard and level layers need channels"))
            }
            _ => Ok(()),
        }
    }

    /// Value slots per present post, and bytes per slot.
    pub fn slots(&self) -> (usize, usize) {
        match self.kind {
            Kind::Class => (NCLASS, 1),
            Kind::Hazard => (self.channels.len(), 1),
            Kind::Level => (self.channels.len(), 2),
            Kind::Feature => (0, 0),
        }
    }
}

/// One 64 × 64 block of a grid layer. `vals[slot][post]`; absent posts hold zeros.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GridBlock {
    pub id: BlockId,
    pub present: [u64; 64],
    pub conf: Vec<u8>,
    pub vals: Vec<Vec<u16>>,
}

impl GridBlock {
    pub fn new(id: BlockId, slots: usize) -> Self {
        GridBlock {
            id,
            present: [0; 64],
            conf: vec![0; BLOCK_POSTS],
            vals: vec![vec![0; BLOCK_POSTS]; slots],
        }
    }
    pub fn has(&self, p: usize) -> bool {
        self.present[p >> 6] >> (p & 63) & 1 == 1
    }
    pub fn set_present(&mut self, p: usize) {
        self.present[p >> 6] |= 1 << (p & 63);
    }
    pub fn count(&self) -> usize {
        self.present.iter().map(|w| w.count_ones() as usize).sum()
    }
}

/// A linear feature: class, confidence (1/255), points in E7 degrees (lat, lon).
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Feature {
    pub class: u8,
    pub conf: u8,
    pub pts: Vec<(i32, i32)>,
}

impl Feature {
    pub fn encode(&self, o: &mut Vec<u8>) {
        o.push(self.class);
        o.push(self.conf);
        put_varint(o, self.pts.len() as u64);
        let (mut la, mut lo) = (0i64, 0i64);
        for &(a, b) in &self.pts {
            put_varint(o, zigzag(a as i64 - la));
            put_varint(o, zigzag(b as i64 - lo));
            (la, lo) = (a as i64, b as i64);
        }
    }

    fn decode(r: &mut Reader) -> Result<Feature> {
        let (class, conf) = (r.u8()?, r.u8()?);
        let n = r.len()?;
        if n < 2 {
            return Err(Error::invalid("a feature needs two points"));
        }
        let (mut la, mut lo) = (0i64, 0i64);
        let mut pts = Vec::with_capacity(n);
        let bad = || Error::invalid("feature point outside E7 degrees");
        for _ in 0..n {
            la = la.checked_add(unzigzag(r.varint()?)).ok_or_else(bad)?;
            lo = lo.checked_add(unzigzag(r.varint()?)).ok_or_else(bad)?;
            pts.push((
                i32::try_from(la).map_err(|_| bad())?,
                i32::try_from(lo).map_err(|_| bad())?,
            ));
        }
        Ok(Feature { class, conf, pts })
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Layer {
    pub m: Manifest,
    /// Sorted by block key; blocks with no present post are not stored.
    pub blocks: Vec<GridBlock>,
    /// Canonical order: sorted by encoded bytes, unique.
    pub features: Vec<Feature>,
}

impl Layer {
    /// Sorts and validates; class probabilities must sum to [`Q`] at every present post.
    pub fn canonicalise(&mut self) -> Result<()> {
        self.m.check()?;
        self.blocks.retain(|b| b.count() > 0);
        self.blocks.sort_by_key(|b| b.id.key());
        if self.blocks.windows(2).any(|w| w[0].id == w[1].id) {
            return Err(Error::invalid("duplicate block"));
        }
        let (slots, width) = self.m.slots();
        for b in self.blocks.iter_mut() {
            if b.vals.len() != slots
                || b.conf.len() != BLOCK_POSTS
                || b.vals.iter().any(|v| v.len() != BLOCK_POSTS)
            {
                return Err(Error::invalid("block planes do not match the manifest"));
            }
            for p in 0..BLOCK_POSTS {
                if !b.has(p) {
                    b.conf[p] = 0;
                    for v in b.vals.iter_mut() {
                        v[p] = 0;
                    }
                    continue;
                }
                if width == 1 && b.vals.iter().any(|v| v[p] > 255) {
                    return Err(Error::invalid("u8 slot above 255"));
                }
                if self.m.kind == Kind::Class {
                    let s: u32 = b.vals.iter().map(|v| v[p] as u32).sum();
                    if s != Q {
                        return Err(Error::invalid(format!(
                            "class probabilities sum to {s} at block {:?} post {p}",
                            b.id
                        )));
                    }
                }
            }
        }
        if self.m.kind == Kind::Feature {
            if !self.blocks.is_empty() {
                return Err(Error::invalid("a feature layer has no grid"));
            }
            if self.features.iter().any(|f| f.pts.len() < 2) {
                return Err(Error::invalid("a feature needs two points"));
            }
            let mut keyed: Vec<(Vec<u8>, Feature)> = self
                .features
                .drain(..)
                .map(|f| {
                    let mut o = Vec::new();
                    f.encode(&mut o);
                    (o, f)
                })
                .collect();
            keyed.sort();
            keyed.dedup_by(|a, b| a.0 == b.0);
            self.features = keyed.into_iter().map(|k| k.1).collect();
        } else if !self.features.is_empty() {
            return Err(Error::invalid("a grid layer has no features"));
        }
        Ok(())
    }

    /// The layer's identity: blake3 of manifest, blocks (key, presence, then per present post
    /// confidence and slot values), and features, in canonical order.
    pub fn content_hash(&self) -> [u8; 32] {
        let mut h = blake3::Hasher::new();
        h.update(&self.m.encode());
        let (_, width) = self.m.slots();
        h.update(&(self.blocks.len() as u64).to_le_bytes());
        let mut buf = Vec::with_capacity(BLOCK_POSTS * 34);
        for b in &self.blocks {
            buf.clear();
            buf.extend_from_slice(&b.id.key().to_le_bytes());
            for w in b.present {
                buf.extend_from_slice(&w.to_le_bytes());
            }
            for p in 0..BLOCK_POSTS {
                if b.has(p) {
                    buf.push(b.conf[p]);
                    for v in &b.vals {
                        if width == 1 {
                            buf.push(v[p] as u8)
                        } else {
                            buf.extend_from_slice(&v[p].to_le_bytes())
                        }
                    }
                }
            }
            h.update(&buf);
        }
        h.update(&(self.features.len() as u64).to_le_bytes());
        let mut o = Vec::new();
        for f in &self.features {
            f.encode(&mut o);
        }
        h.update(&o);
        *h.finalize().as_bytes()
    }

    /// The file: "WTL0", manifest, block count u32, block keys u64, n + 1 u64 payload offsets,
    /// payloads, feature count varint and features, then the content hash (32 bytes). The layer
    /// must be canonical.
    pub fn write(&self) -> Vec<u8> {
        self.write_with(None)
    }

    /// [`Layer::write`] with one plane coding forced (tests that hashes do not depend on it).
    #[doc(hidden)]
    pub fn write_with(&self, force: Option<u8>) -> Vec<u8> {
        let mut o = Vec::new();
        o.extend_from_slice(FILE_MAGIC);
        o.extend_from_slice(&self.m.encode());
        o.extend_from_slice(&(self.blocks.len() as u32).to_le_bytes());
        let (_, width) = self.m.slots();
        let payloads: Vec<Vec<u8>> = self
            .blocks
            .iter()
            .map(|b| encode_block(b, width, force))
            .collect();
        for b in &self.blocks {
            o.extend_from_slice(&b.id.key().to_le_bytes());
        }
        let mut off = 0u64;
        for p in &payloads {
            o.extend_from_slice(&off.to_le_bytes());
            off += p.len() as u64;
        }
        o.extend_from_slice(&off.to_le_bytes());
        for p in &payloads {
            o.extend_from_slice(p);
        }
        put_varint(&mut o, self.features.len() as u64);
        for f in &self.features {
            f.encode(&mut o);
        }
        o.extend_from_slice(&self.content_hash());
        o
    }

    /// Reads a layer file and checks it completely: canonical order, every value, the hash.
    pub fn read(buf: &[u8]) -> Result<Layer> {
        let mut r = Reader::new(buf, 0, "layer file");
        if r.take(4).map_err(|_| Error::Magic("layer file"))? != FILE_MAGIC {
            return Err(Error::Magic("layer file"));
        }
        let m = Manifest::decode(&mut r)?;
        let n = r.u32()? as usize;
        if n > buf.len() / 16 {
            return Err(Error::Truncated("layer file"));
        }
        let keys = (0..n).map(|_| r.u64()).collect::<Result<Vec<_>>>()?;
        let offs = (0..=n).map(|_| r.u64()).collect::<Result<Vec<_>>>()?;
        if offs[0] != 0 || offs.windows(2).any(|w| w[0] > w[1]) {
            return Err(Error::invalid("layer payload offsets out of order"));
        }
        let base = r.pos;
        let (slots, width) = m.slots();
        let mut blocks = Vec::with_capacity(n);
        for k in 0..n {
            let id = BlockId::from_key(keys[k]);
            let (a, z) = (base as u64 + offs[k], base as u64 + offs[k + 1]);
            let pl = buf
                .get(a as usize..z as usize)
                .ok_or(Error::Truncated("layer file"))?;
            blocks.push(
                decode_block(pl, id, slots, width)
                    .ok_or_else(|| Error::invalid(format!("bad payload in layer block {id:?}")))?,
            );
        }
        r.pos = base + offs[n] as usize;
        let nf = r.len()?;
        let features = (0..nf)
            .map(|_| Feature::decode(&mut r))
            .collect::<Result<Vec<_>>>()?;
        let stored = r.h32()?;
        r.end()?;
        let mut l = Layer {
            m,
            blocks,
            features,
        };
        let before = l.content_hash();
        l.canonicalise()?;
        let h = l.content_hash();
        if h != before {
            return Err(Error::invalid("layer file is not in canonical order"));
        }
        if h != stored {
            return Err(Error::Hash("layer content"));
        }
        Ok(l)
    }
}

/// Block payload: tag byte (bit 7: a 512 B presence bitmap follows), then the confidence plane
/// and each slot plane over the present posts only, each a coding byte and its data: 0 raw,
/// 1 one value, 3 runs (varint run length - 1, then the value), 4 zlib of the raw plane
/// (varint length, then the stream). The smallest coding wins, ties to the lower code.
fn encode_block(b: &GridBlock, width: usize, force: Option<u8>) -> Vec<u8> {
    let full = b.count() == BLOCK_POSTS;
    let mut o = vec![if full { 0u8 } else { 0x80 }];
    if !full {
        for w in b.present {
            o.extend_from_slice(&w.to_le_bytes());
        }
    }
    let idx: Vec<usize> = (0..BLOCK_POSTS).filter(|&p| b.has(p)).collect();
    let conf: Vec<u16> = idx.iter().map(|&p| b.conf[p] as u16).collect();
    encode_plane(&mut o, &conf, 1, force);
    for v in &b.vals {
        let vv: Vec<u16> = idx.iter().map(|&p| v[p]).collect();
        encode_plane(&mut o, &vv, width, force);
    }
    o
}

fn put_val(o: &mut Vec<u8>, v: u16, width: usize) {
    if width == 1 {
        o.push(v as u8)
    } else {
        o.extend_from_slice(&v.to_le_bytes())
    }
}

fn encode_plane(o: &mut Vec<u8>, v: &[u16], width: usize, force: Option<u8>) {
    let mut cands: Vec<(u8, Vec<u8>)> = Vec::new();
    let mut raw = Vec::with_capacity(v.len() * width);
    for &x in v {
        put_val(&mut raw, x, width);
    }
    if v.iter().all(|&x| x == v[0]) {
        let mut c = Vec::new();
        put_val(&mut c, v[0], width);
        cands.push((1, c));
    }
    let mut runs = Vec::new();
    let mut i = 0;
    while i < v.len() {
        let mut j = i + 1;
        while j < v.len() && v[j] == v[i] {
            j += 1;
        }
        put_varint(&mut runs, (j - i - 1) as u64);
        put_val(&mut runs, v[i], width);
        i = j;
    }
    cands.push((3, runs));
    let z = codec::zlib(&raw);
    let mut zl = Vec::with_capacity(z.len() + 3);
    put_varint(&mut zl, z.len() as u64);
    zl.extend_from_slice(&z);
    cands.push((4, zl));
    cands.push((0, raw));
    let (code, data) = match force {
        Some(f) => cands
            .into_iter()
            .find(|c| c.0 == f)
            .unwrap_or_else(|| panic!("plane coding {f} is not possible here")),
        None => cands
            .into_iter()
            .min_by_key(|c| (c.1.len(), c.0))
            .expect("raw is always a candidate"),
    };
    o.push(code);
    o.extend_from_slice(&data);
}

fn get_val(r: &mut Reader, width: usize) -> Option<u16> {
    if width == 1 {
        r.u8().ok().map(u16::from)
    } else {
        r.u16().ok()
    }
}

fn decode_plane(r: &mut Reader, n: usize, width: usize) -> Option<Vec<u16>> {
    let code = r.u8().ok()?;
    let mut out = Vec::with_capacity(n);
    match code {
        0 => {
            for _ in 0..n {
                out.push(get_val(r, width)?);
            }
        }
        1 => {
            let v = get_val(r, width)?;
            out.resize(n, v);
        }
        3 => {
            while out.len() < n {
                let len = r.varint().ok()? as usize + 1;
                let v = get_val(r, width)?;
                if out.len() + len > n {
                    return None;
                }
                out.extend(std::iter::repeat_n(v, len));
            }
        }
        4 => {
            let len = r.len().ok()?;
            let raw = codec::unzlib(r.take(len).ok()?, n * width)?;
            let mut q = Reader::new(&raw, 0, "plane");
            for _ in 0..n {
                out.push(get_val(&mut q, width)?);
            }
        }
        _ => return None,
    }
    Some(out)
}

fn decode_block(b: &[u8], id: BlockId, slots: usize, width: usize) -> Option<GridBlock> {
    let mut g = GridBlock::new(id, slots);
    let mut r = Reader::new(b, 0, "layer block");
    let tag = r.u8().ok()?;
    if tag & 0x7F != 0 {
        return None;
    }
    if tag & 0x80 != 0 {
        for k in 0..64 {
            g.present[k] = r.u64().ok()?;
        }
    } else {
        g.present = [u64::MAX; 64];
    }
    let idx: Vec<usize> = (0..BLOCK_POSTS).filter(|&q| g.has(q)).collect();
    let conf = decode_plane(&mut r, idx.len(), 1)?;
    for (k, &q) in idx.iter().enumerate() {
        g.conf[q] = conf[k] as u8;
    }
    for s in 0..slots {
        let v = decode_plane(&mut r, idx.len(), width)?;
        for (k, &q) in idx.iter().enumerate() {
            g.vals[s][q] = v[k];
        }
    }
    r.at_end().then_some(g)
}

/// Largest-remainder rounding of non-negative weights to integers summing exactly to `total`.
/// Shares are floor(w * total / sum); the leftover units go to the largest remainders, ties to
/// the lower index. Weights of 0 never receive a unit.
///
/// # Panics
/// If the weights sum to 0 or `out` has another length.
pub fn normalise(w: &[u64], total: u64, out: &mut [u64]) {
    let sum: u128 = w.iter().map(|&x| x as u128).sum();
    assert!(
        sum > 0 && w.len() == out.len(),
        "normalise needs a positive sum and matching lengths"
    );
    let mut rem: Vec<(u128, usize)> = Vec::with_capacity(w.len());
    let mut given = 0u64;
    for (k, &x) in w.iter().enumerate() {
        let num = x as u128 * total as u128;
        out[k] = (num / sum) as u64;
        given += out[k];
        rem.push((num % sum, k));
    }
    let mut left = total - given;
    if left > 0 {
        rem.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
        for &(r, k) in &rem {
            if left == 0 || r == 0 {
                break;
            }
            out[k] += 1;
            left -= 1;
        }
    }
    debug_assert_eq!(left, 0);
}

/// The producer's quantiser: f32 probabilities (finite, >= 0, sum > 0) to u8 summing to 255.
/// Weights are the f32 values scaled by 2^24 exactly (f32 to f64 is exact and so is the
/// product), rounded to whole units with ties to even, then [`normalise`]d: any language that
/// follows these steps gets the same bytes.
pub fn quantise_probs(f: &[f32]) -> Option<Vec<u8>> {
    let mut w = Vec::with_capacity(f.len());
    for &x in f {
        if !x.is_finite() || x < 0.0 {
            return None;
        }
        w.push(((x as f64) * 16_777_216.0).round_ties_even() as u64);
    }
    if w.iter().sum::<u64>() == 0 {
        return None;
    }
    let mut out = vec![0u64; f.len()];
    normalise(&w, Q as u64, &mut out);
    Some(out.into_iter().map(|v| v as u8).collect())
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub fn manifest(kind: Kind, channels: Vec<u16>) -> Manifest {
        Manifest {
            kind,
            licence: 0,
            s_lat: 120,
            s_lon: 120,
            observed_at: 1_000,
            valid_from: 0,
            valid_until: 10_000,
            fade_from: 10_000,
            channels,
            model_id: "test".into(),
            model_version: "0".into(),
            weights: [1; 32],
            dataset_manifest: [2; 32],
            eval_set: [3; 32],
            ece_permille: 0xFFFF,
            backend: "none".into(),
            attribution: String::new(),
            inputs: vec![],
            parents: vec![],
        }
    }

    #[test]
    fn normalise_exact() {
        let mut o = [0u64; 3];
        normalise(&[1, 1, 1], 255, &mut o);
        assert_eq!(o, [85, 85, 85]);
        normalise(&[1, 1, 1], 256, &mut o);
        assert_eq!(o, [86, 85, 85]);
        normalise(&[0, 5, 0], 7, &mut o);
        assert_eq!(o, [0, 7, 0]);
        normalise(&[2, 1, 0], 2, &mut o);
        assert_eq!(o, [1, 1, 0]);
    }

    #[test]
    fn quantiser() {
        assert_eq!(quantise_probs(&[0.5, 0.5]).unwrap(), vec![128, 127]);
        assert_eq!(
            quantise_probs(&[0.2, 0.3, 0.5])
                .unwrap()
                .iter()
                .map(|&v| v as u32)
                .sum::<u32>(),
            255
        );
        assert_eq!(quantise_probs(&[1.0, 0.0, 1e-9]).unwrap(), vec![255, 0, 0]);
        assert!(quantise_probs(&[f32::NAN, 1.0]).is_none());
        assert!(quantise_probs(&[0.0, 0.0]).is_none());
    }

    #[test]
    fn round_trip_and_hash_independent_of_coding() {
        let mut b = GridBlock::new(BlockId { bi: 3, bj: -2 }, 2);
        for p in 0..BLOCK_POSTS {
            if p % 7 != 0 {
                b.set_present(p);
                b.conf[p] = (p % 200) as u8;
                b.vals[0][p] = if p < 2000 { 9 } else { (p % 3) as u16 };
                b.vals[1][p] = 4;
            }
        }
        let mut l = Layer {
            m: manifest(Kind::Hazard, vec![1, 2]),
            blocks: vec![b],
            features: vec![],
        };
        l.canonicalise().unwrap();
        let h = l.content_hash();
        for force in [None, Some(0), Some(3), Some(4)] {
            let f = l.write_with(force);
            let r = Layer::read(&f).unwrap();
            assert_eq!(r.content_hash(), h);
            assert_eq!(r.blocks, l.blocks);
        }
        let mut f = l.write();
        let n = f.len();
        f[n - 40] ^= 1;
        assert!(Layer::read(&f).is_err());
        assert!(Layer::read(&f[..n / 2]).is_err());
    }

    #[test]
    fn features_canonical() {
        let a = Feature {
            class: 18,
            conf: 200,
            pts: vec![(1, 2), (3, 4)],
        };
        let b = Feature {
            class: 17,
            conf: 10,
            pts: vec![(5, 6), (-7, 8), (9, 9)],
        };
        let mut l1 = Layer {
            m: manifest(Kind::Feature, vec![]),
            blocks: vec![],
            features: vec![a.clone(), b.clone()],
        };
        let mut l2 = Layer {
            m: manifest(Kind::Feature, vec![]),
            blocks: vec![],
            features: vec![b, a.clone(), a],
        };
        l1.canonicalise().unwrap();
        l2.canonicalise().unwrap();
        assert_eq!(l1.content_hash(), l2.content_hash());
        assert_eq!(Layer::read(&l1.write()).unwrap().features, l1.features);
    }

    #[test]
    fn class_sum_checked() {
        let mut b = GridBlock::new(BlockId { bi: 0, bj: 0 }, NCLASS);
        b.set_present(5);
        b.vals[3][5] = 254;
        let mut l = Layer {
            m: manifest(Kind::Class, vec![]),
            blocks: vec![b],
            features: vec![],
        };
        assert!(l.canonicalise().is_err());
    }
}

//! The incident surface (`docs/design/incident-surface.md`, contract v1 draft).
//!
//! The incident surface is the stored result of fusion: the base, the layers and the team
//! overlay active at an incident epoch T, costed per profile. A query reads only this surface
//! and the terrain's heights.
//!
//! Per block it stores the fused top class, its certainty, the post's source (a block-local
//! index into a table of layer hashes, so a block depends only on its own inputs), and per
//! profile the time per metre (u16, 0 = impassable), the risk that the cell is impassable
//! (1/255) and why a closed cell is closed.
//!
//! Identity: a block hash over each block's canonical decoded bytes; the **content hash** over
//! what routes depend on (spacing, terrain, profile tables) and the block hashes; the **recipe
//! hash** over every input (fuse version, terrain, prior, layers applied and left out, the
//! canonical overlay, profiles, params, T). The same content hash means the same routes. A delta carries only the changed
//! blocks and is checked by the content hash it produces.

use crate::codec::{self, Reader, put_varint};
use crate::cost::cell;
use crate::fuse::{self, FusedBlock, HashedLayer, LeftOut, OverlayItem, Params, Prior, Record};
use crate::lattice::{BLOCK_POSTS, BlockId, ceil_div};
use crate::layer::{Kind, LICENCE_RESEARCH, Layer, Manifest};
use crate::profile::Rules;
use crate::terrain::Terrain;
use crate::{Error, Result};
use rayon::prelude::*;
use std::collections::{BTreeMap, BTreeSet};

pub const MAJOR: u16 = 1;
pub const MINOR: u16 = 0;
/// The fusion and cost rules' version, in every recipe. It changes whenever the same inputs
/// would give different surface content: a new rule, a changed constant, a changed default
/// profile. Two builds with the same fuse version and the same inputs agree, whatever their
/// crate versions. (0 was experiment E14's rules, unversioned.)
pub const FUSE_VERSION: u16 = 1;
pub const MAGIC: &[u8; 4] = b"WIS0";
pub const DELTA_MAGIC: &[u8; 4] = b"WISD";
/// The incident id of a surface that belongs to no incident: all zeros.
pub const NO_INCIDENT: [u8; 16] = [0; 16];
/// Source of a post: the base.
pub const SRC_BASE: u8 = 0;
/// Source of a post: the team overlay.
pub const SRC_TEAM: u8 = 255;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProfPlanes {
    pub t: Vec<u16>,
    pub risk: Vec<u8>,
    pub why: Vec<u8>,
}

/// One block of a surface.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SBlock {
    /// Hashes of the grid layers that are some post's source here, in fusion order.
    pub table: Vec<[u8; 32]>,
    pub top: Vec<u8>,
    pub cert: Vec<u8>,
    /// [`SRC_BASE`], k = `table[k - 1]`, or [`SRC_TEAM`].
    pub src: Vec<u8>,
    pub prof: Vec<ProfPlanes>,
    hash: [u8; 32],
}

impl SBlock {
    pub fn hash(&self) -> [u8; 32] {
        self.hash
    }

    /// Canonical decoded bytes: key, table, then the planes (u8 as bytes, u16 little-endian).
    pub fn canonical(&self, id: BlockId) -> Vec<u8> {
        let mut o =
            Vec::with_capacity(BLOCK_POSTS * (3 + 4 * self.prof.len()) + 9 + 32 * self.table.len());
        o.extend_from_slice(&id.key().to_le_bytes());
        o.push(self.table.len() as u8);
        for h in &self.table {
            o.extend_from_slice(h);
        }
        o.extend_from_slice(&self.top);
        o.extend_from_slice(&self.cert);
        o.extend_from_slice(&self.src);
        for pp in &self.prof {
            for v in &pp.t {
                o.extend_from_slice(&v.to_le_bytes());
            }
            o.extend_from_slice(&pp.risk);
            o.extend_from_slice(&pp.why);
        }
        o
    }

    fn rehash(&mut self, id: BlockId) {
        self.hash = *blake3::hash(&self.canonical(id)).as_bytes();
    }

    /// Stored payload: the table, then each plane as a varint length and a coded plane.
    pub fn payload(&self) -> Vec<u8> {
        let mut o = Vec::new();
        o.push(self.table.len() as u8);
        for h in &self.table {
            o.extend_from_slice(h);
        }
        let mut plane = |v: Vec<u16>, wide: bool| {
            let b = codec::plane_encode(&v, wide);
            put_varint(&mut o, b.len() as u64);
            o.extend_from_slice(&b);
        };
        plane(u8s(&self.top), false);
        plane(u8s(&self.cert), false);
        plane(u8s(&self.src), false);
        for pp in &self.prof {
            plane(pp.t.clone(), true);
            plane(u8s(&pp.risk), false);
            plane(u8s(&pp.why), false);
        }
        o
    }

    pub fn from_payload(id: BlockId, b: &[u8], nprof: usize) -> Result<SBlock> {
        let mut r = Reader::new(b, 0, "surface block");
        let n = r.u8()? as usize;
        let table = (0..n).map(|_| r.h32()).collect::<Result<Vec<_>>>()?;
        let mut plane = |wide: bool| -> Result<Vec<u16>> {
            let len = r.len()?;
            codec::plane_decode(r.take(len)?, wide)
                .ok_or_else(|| Error::invalid(format!("bad plane in surface block {id:?}")))
        };
        let n8 = |v: Vec<u16>| v.into_iter().map(|x| x as u8).collect::<Vec<u8>>();
        let top = n8(plane(false)?);
        let cert = n8(plane(false)?);
        let src = n8(plane(false)?);
        let mut prof = Vec::with_capacity(nprof);
        for _ in 0..nprof {
            let t = plane(true)?;
            let risk = n8(plane(false)?);
            let why = n8(plane(false)?);
            prof.push(ProfPlanes { t, risk, why });
        }
        r.end()?;
        if src
            .iter()
            .any(|&s| s != SRC_BASE && s != SRC_TEAM && s as usize > table.len())
        {
            return Err(Error::invalid("source index outside the block table"));
        }
        let mut sb = SBlock {
            table,
            top,
            cert,
            src,
            prof,
            hash: [0; 32],
        };
        sb.rehash(id);
        Ok(sb)
    }
}

fn u8s(v: &[u8]) -> Vec<u16> {
    v.iter().map(|&x| x as u16).collect()
}

/// A surface block from a fused block: top class, certainty, block-local source, and per
/// profile the cost rules. `k` is the block's index in the terrain.
pub fn cost_block(
    ter: &Terrain,
    k: usize,
    fb: &FusedBlock,
    applied: &[fuse::Applied],
    rules: &[Rules],
) -> SBlock {
    let id = ter.blocks()[k];
    let (steep, lines) = (ter.steep(k), ter.lines(k));
    let used: BTreeSet<u8> = fb
        .src
        .iter()
        .copied()
        .filter(|&s| s != SRC_BASE && s != SRC_TEAM)
        .collect();
    let table: Vec<[u8; 32]> = used.iter().map(|&s| applied[s as usize - 1].hash).collect();
    let local: BTreeMap<u8, u8> = used
        .iter()
        .enumerate()
        .map(|(n, &s)| (s, n as u8 + 1))
        .collect();
    let src = fb.src.iter().map(|s| *local.get(s).unwrap_or(s)).collect();
    let top =
        fb.p.iter()
            .map(|p| {
                let mx = *p.iter().max().expect("16 classes");
                p.iter()
                    .position(|&v| v == mx)
                    .expect("the maximum is in the array") as u8
            })
            .collect();
    let prof = rules
        .iter()
        .map(|r| {
            let mut pp = ProfPlanes {
                t: vec![0; BLOCK_POSTS],
                risk: vec![0; BLOCK_POSTS],
                why: vec![0; BLOCK_POSTS],
            };
            for q in 0..BLOCK_POSTS {
                let c = cell(
                    r, &fb.p[q], &fb.haz[q], &fb.lev[q], fb.mask[q], steep[q], lines[q],
                );
                (pp.t[q], pp.risk[q], pp.why[q]) = (c.t, c.risk, c.why);
            }
            pp
        })
        .collect();
    let mut sb = SBlock {
        table,
        top,
        cert: fb.cert.clone(),
        src,
        prof,
        hash: [0; 32],
    };
    sb.rehash(id);
    sb
}

/// Surface blocks from fused blocks ([`cost_block`] for each).
pub fn build_blocks(
    ter: &Terrain,
    fused: &[(BlockId, FusedBlock)],
    rec: &Record,
    rules: &[Rules],
) -> Vec<(BlockId, SBlock)> {
    fused
        .par_iter()
        .map(|(id, fb)| {
            let k = ter
                .block_index(*id)
                .expect("fused blocks come from the terrain");
            (*id, cost_block(ter, k, fb, &rec.applied, rules))
        })
        .collect()
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AppliedRef {
    pub hash: [u8; 32],
    pub kind: u8,
    pub observed_at: i64,
    pub valid_until: i64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Header {
    /// [`FUSE_VERSION`] of the build that made the surface. A reader with another fuse version
    /// still reads and routes on it; it just cannot rebuild the same surface.
    pub fuse_version: u16,
    /// [`Incident::id`]; [`NO_INCIDENT`] when the surface belongs to no incident.
    pub incident: [u8; 16],
    pub spacing: u32,
    pub epoch_s: u32,
    pub t: i64,
    /// The first epoch start after T at which the content can change; `i64::MAX` if none.
    pub same_until: i64,
    pub terrain: [u8; 32],
    /// Hash of the land-cover prior ([`Prior`]), a fixed input; not a previous surface.
    pub prior: [u8; 32],
    pub overlay: [u8; 32],
    pub allow_research: u8,
    pub feature_min_conf: u8,
    pub level_min_w: u8,
    /// The layers applied, in fusion order; a block's table refers to these.
    pub applied: Vec<AppliedRef>,
    /// The layers left out and the [`LeftOut`] code, sorted by hash.
    pub left_out: Vec<([u8; 32], u8)>,
    /// The profile tables ([`Rules::bytes`]), in plane order.
    pub profiles: Vec<Vec<u8>>,
}

impl Header {
    fn encode(&self) -> Vec<u8> {
        let mut o = Vec::new();
        o.extend_from_slice(&self.fuse_version.to_le_bytes());
        o.extend_from_slice(&self.incident);
        o.extend_from_slice(&self.spacing.to_le_bytes());
        o.extend_from_slice(&self.epoch_s.to_le_bytes());
        o.extend_from_slice(&self.t.to_le_bytes());
        o.extend_from_slice(&self.same_until.to_le_bytes());
        o.extend_from_slice(&self.terrain);
        o.extend_from_slice(&self.prior);
        o.extend_from_slice(&self.overlay);
        o.extend_from_slice(&[self.allow_research, self.feature_min_conf, self.level_min_w]);
        put_varint(&mut o, self.applied.len() as u64);
        for a in &self.applied {
            o.extend_from_slice(&a.hash);
            o.push(a.kind);
            o.extend_from_slice(&a.observed_at.to_le_bytes());
            o.extend_from_slice(&a.valid_until.to_le_bytes());
        }
        put_varint(&mut o, self.left_out.len() as u64);
        for (h, r) in &self.left_out {
            o.extend_from_slice(h);
            o.push(*r);
        }
        o.push(self.profiles.len() as u8);
        for p in &self.profiles {
            put_varint(&mut o, p.len() as u64);
            o.extend_from_slice(p);
        }
        o
    }

    fn decode(r: &mut Reader) -> Result<Header> {
        let fuse_version = r.u16()?;
        let incident = r.take(16)?.try_into().unwrap();
        let (spacing, epoch_s) = (r.u32()?, r.u32()?);
        let (t, same_until) = (r.i64()?, r.i64()?);
        let (terrain, prior, overlay) = (r.h32()?, r.h32()?, r.h32()?);
        let (allow_research, feature_min_conf, level_min_w) = (r.u8()?, r.u8()?, r.u8()?);
        let n = r.len()?;
        let mut applied = Vec::with_capacity(n);
        for _ in 0..n {
            let hash = r.h32()?;
            let kind = r.u8()?;
            applied.push(AppliedRef {
                hash,
                kind,
                observed_at: r.i64()?,
                valid_until: r.i64()?,
            });
        }
        let n = r.len()?;
        let mut left_out = Vec::with_capacity(n);
        for _ in 0..n {
            let h = r.h32()?;
            left_out.push((h, r.u8()?));
        }
        let np = r.u8()? as usize;
        let mut profiles = Vec::with_capacity(np);
        for _ in 0..np {
            let n = r.len()?;
            let b = r.take(n)?.to_vec();
            Rules::from_bytes(&b)?;
            profiles.push(b);
        }
        if epoch_s == 0 || applied.len() > 253 {
            return Err(Error::invalid("bad surface header"));
        }
        Ok(Header {
            fuse_version,
            incident,
            spacing,
            epoch_s,
            t,
            same_until,
            terrain,
            prior,
            overlay,
            allow_research,
            feature_min_conf,
            level_min_w,
            applied,
            left_out,
            profiles,
        })
    }

    /// What routes depend on besides the blocks: spacing, terrain (the heights), profile tables.
    fn content_prefix(&self, h: &mut blake3::Hasher) {
        h.update(b"wis1/content");
        h.update(&MAJOR.to_le_bytes());
        h.update(&self.spacing.to_le_bytes());
        h.update(&self.terrain);
        h.update(&[self.profiles.len() as u8]);
        for p in &self.profiles {
            h.update(blake3::hash(p).as_bytes());
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Surface {
    pub h: Header,
    /// By block key.
    pub blocks: BTreeMap<u64, (BlockId, SBlock)>,
}

impl Surface {
    pub fn content_hash(&self) -> [u8; 32] {
        let mut h = blake3::Hasher::new();
        self.h.content_prefix(&mut h);
        h.update(&(self.blocks.len() as u32).to_le_bytes());
        for (k, (_, b)) in &self.blocks {
            h.update(&k.to_le_bytes());
            h.update(&b.hash);
        }
        *h.finalize().as_bytes()
    }

    /// Every input: the header (T, the layers applied and left out, the overlay, the params
    /// and the profile tables), the contract versions and the AOI.
    pub fn recipe_hash(&self) -> [u8; 32] {
        let mut h = blake3::Hasher::new();
        h.update(b"wis1/recipe");
        h.update(&MAJOR.to_le_bytes());
        h.update(&MINOR.to_le_bytes());
        h.update(&crate::layer::MAJOR.to_le_bytes());
        h.update(&self.h.encode());
        h.update(&(self.blocks.len() as u32).to_le_bytes());
        for k in self.blocks.keys() {
            h.update(&k.to_le_bytes());
        }
        *h.finalize().as_bytes()
    }

    /// The profile tables the surface was costed with.
    pub fn rules(&self) -> Vec<Rules> {
        self.h
            .profiles
            .iter()
            .map(|b| Rules::from_bytes(b).expect("checked when built or read"))
            .collect()
    }

    /// The file: magic, versions, header, recipe and content hashes, block index (key, length,
    /// block hash), payloads in key order.
    pub fn write(&self) -> Vec<u8> {
        let mut o = Vec::new();
        o.extend_from_slice(MAGIC);
        o.extend_from_slice(&MAJOR.to_le_bytes());
        o.extend_from_slice(&MINOR.to_le_bytes());
        o.extend_from_slice(&self.h.encode());
        o.extend_from_slice(&self.recipe_hash());
        o.extend_from_slice(&self.content_hash());
        o.extend_from_slice(&self.block_section());
        o
    }

    /// The end of the file after the hashes: block count, block index and payloads. It is
    /// the same bytes whenever the blocks are, whatever the header holds.
    pub fn block_section(&self) -> Vec<u8> {
        let mut o = Vec::new();
        put_varint(&mut o, self.blocks.len() as u64);
        let pays: Vec<Vec<u8>> = self.blocks.values().map(|(_, b)| b.payload()).collect();
        for ((k, (_, b)), p) in self.blocks.iter().zip(&pays) {
            o.extend_from_slice(&k.to_le_bytes());
            o.extend_from_slice(&(p.len() as u32).to_le_bytes());
            o.extend_from_slice(&b.hash);
        }
        for p in pays {
            o.extend_from_slice(&p);
        }
        o
    }

    /// Reads a surface file and verifies every block hash, the content and the recipe hash.
    pub fn read(b: &[u8]) -> Result<Surface> {
        let mut r = Reader::new(b, 0, "incident surface");
        if r.take(4).map_err(|_| Error::Magic("incident surface"))? != MAGIC {
            return Err(Error::Magic("incident surface"));
        }
        let major = r.u16()?;
        if major != MAJOR {
            return Err(Error::Version {
                what: "incident surface",
                found: major,
                known: MAJOR,
            });
        }
        let _minor = r.u16()?;
        let h = Header::decode(&mut r)?;
        let (recipe, content) = (r.h32()?, r.h32()?);
        let blocks = read_blocks(&mut r, h.profiles.len())?;
        r.end()?;
        let s = Surface { h, blocks };
        if s.content_hash() != content {
            return Err(Error::Hash("surface content"));
        }
        if s.recipe_hash() != recipe {
            return Err(Error::Hash("surface recipe"));
        }
        Ok(s)
    }
}

fn read_blocks(r: &mut Reader, nprof: usize) -> Result<BTreeMap<u64, (BlockId, SBlock)>> {
    let n = r.len()?;
    let mut idx = Vec::with_capacity(n);
    for _ in 0..n {
        idx.push((r.u64()?, r.u32()? as usize, r.h32()?));
    }
    if idx.windows(2).any(|w| w[0].0 >= w[1].0) {
        return Err(Error::invalid("block index not sorted"));
    }
    let mut out = BTreeMap::new();
    for (key, len, hash) in idx {
        let id = BlockId::from_key(key);
        let sb = SBlock::from_payload(id, r.take(len)?, nprof)?;
        if sb.hash != hash {
            return Err(Error::Hash("surface block"));
        }
        out.insert(key, (id, sb));
    }
    Ok(out)
}

/// A delta from `old` to `new`: the old content hash, the new header and hashes, then the
/// changed or added blocks (op 1, with payload) and the removed ones (op 0), by key.
pub fn delta(old: &Surface, new: &Surface) -> Vec<u8> {
    let mut o = Vec::new();
    o.extend_from_slice(DELTA_MAGIC);
    o.extend_from_slice(&MAJOR.to_le_bytes());
    o.extend_from_slice(&MINOR.to_le_bytes());
    o.extend_from_slice(&old.content_hash());
    o.extend_from_slice(&new.h.encode());
    o.extend_from_slice(&new.recipe_hash());
    o.extend_from_slice(&new.content_hash());
    o.extend_from_slice(&delta_ops(old, new));
    o
}

/// The end of a delta after the hashes: the block ops, the same bytes whenever the blocks are.
pub fn delta_ops(old: &Surface, new: &Surface) -> Vec<u8> {
    let mut o = Vec::new();
    let mut ch: Vec<(u64, Option<&SBlock>)> = Vec::new();
    for (k, (_, b)) in &new.blocks {
        if old.blocks.get(k).map(|x| x.1.hash) != Some(b.hash) {
            ch.push((*k, Some(b)));
        }
    }
    for k in old.blocks.keys() {
        if !new.blocks.contains_key(k) {
            ch.push((*k, None));
        }
    }
    ch.sort_by_key(|c| c.0);
    put_varint(&mut o, ch.len() as u64);
    for (k, b) in ch {
        o.extend_from_slice(&k.to_le_bytes());
        match b {
            None => o.push(0),
            Some(b) => {
                o.push(1);
                let p = b.payload();
                o.extend_from_slice(&(p.len() as u32).to_le_bytes());
                o.extend_from_slice(&b.hash);
                o.extend_from_slice(&p);
            }
        }
    }
    o
}

/// Applies a delta made from `old`; the result is checked by its content and recipe hash.
pub fn apply_delta(old: &Surface, b: &[u8]) -> Result<Surface> {
    let mut r = Reader::new(b, 0, "surface delta");
    if r.take(4).map_err(|_| Error::Magic("surface delta"))? != DELTA_MAGIC {
        return Err(Error::Magic("surface delta"));
    }
    let major = r.u16()?;
    if major != MAJOR {
        return Err(Error::Version {
            what: "surface delta",
            found: major,
            known: MAJOR,
        });
    }
    let _minor = r.u16()?;
    if r.h32()? != old.content_hash() {
        return Err(Error::invalid("the delta is not from this surface"));
    }
    let h = Header::decode(&mut r)?;
    let (recipe, content) = (r.h32()?, r.h32()?);
    if h.profiles != old.h.profiles {
        return Err(Error::invalid("a delta cannot change the profile tables"));
    }
    let mut blocks = old.blocks.clone();
    let n = r.len()?;
    for _ in 0..n {
        let key = r.u64()?;
        let id = BlockId::from_key(key);
        match r.u8()? {
            0 => {
                blocks
                    .remove(&key)
                    .ok_or_else(|| Error::invalid("the delta removes a block that is not there"))?;
            }
            1 => {
                let len = r.u32()? as usize;
                let hash = r.h32()?;
                let sb = SBlock::from_payload(id, r.take(len)?, h.profiles.len())?;
                if sb.hash != hash {
                    return Err(Error::Hash("delta block"));
                }
                blocks.insert(key, (id, sb));
            }
            _ => return Err(Error::invalid("unknown delta op")),
        }
    }
    r.end()?;
    let s = Surface { h, blocks };
    if s.content_hash() != content || s.recipe_hash() != recipe {
        return Err(Error::Hash("delta result"));
    }
    Ok(s)
}

// ---- epochs ------------------------------------------------------------------------------------

/// The start of the epoch holding `now`.
pub fn epoch_of(now: i64, epoch_s: i64) -> i64 {
    now.div_euclid(epoch_s) * epoch_s
}

fn ceil_epoch(x: i64, e: i64) -> i64 {
    ceil_div(x, e) * e
}

/// The first epoch start after T at which some input's activity or effective confidence can
/// change (`i64::MAX` if none): each layer's valid_from and valid_until rounded up to an epoch,
/// the first epoch after fade_from, and T + E while a layer is fading.
pub fn same_until(t: i64, e: i64, ms: &[&Manifest]) -> i64 {
    let mut best = i64::MAX;
    for m in ms {
        let mut c = vec![ceil_epoch(m.valid_from, e), ceil_epoch(m.valid_until, e)];
        if m.fade_from < m.valid_until {
            c.push(m.fade_from.div_euclid(e) * e + e);
            if m.fade_from < t && t < m.valid_until {
                c.push(t + e);
            }
        }
        for x in c {
            if x > t {
                best = best.min(x);
            }
        }
    }
    best
}

/// Blocks a layer can change: its grid blocks and the blocks its feature lines cross.
pub fn touched_blocks(l: &Layer, spacing: i64) -> BTreeSet<BlockId> {
    let mut s: BTreeSet<BlockId> = l.blocks.iter().map(|b| b.id).collect();
    for f in &l.features {
        for seg in f.pts.windows(2) {
            fuse::supercover(seg[0], seg[1], spacing, &mut |i, j| {
                s.insert(BlockId::of_post(i, j).0);
            });
        }
    }
    s
}

/// Blocks an overlay item covers.
pub fn overlay_blocks(it: &OverlayItem, spacing: i64) -> BTreeSet<BlockId> {
    it.blocks(spacing)
}

pub fn kind_u8(k: Kind) -> u8 {
    k as u8
}

// ---- building a surface ------------------------------------------------------------------------

/// Everything a surface is built from.
#[derive(Clone, Debug)]
pub struct Incident<'a> {
    /// The incident (mission) the surface belongs to, chosen by the host, or [`NO_INCIDENT`]
    /// for a surface that belongs to none (a route's own analysis). It enters the recipe hash
    /// but not the content hash, so it never changes a route: two hosts that build from the
    /// same inputs with the same id, or both with none, get the same recipe hash.
    pub id: [u8; 16],
    pub terrain: &'a Terrain,
    pub prior: &'a Prior,
    /// In any order: fusion orders them.
    pub layers: Vec<&'a HashedLayer>,
    /// In any order: fusion applies them by observed time ([`fuse::canonical_overlay`]).
    pub overlay: Vec<OverlayItem>,
    pub rules: &'a [Rules],
    /// Epoch length in seconds (900 = 15 minutes).
    pub epoch_s: i64,
    pub allow_research: bool,
    pub feature_min_conf: u8,
    pub level_min_w: u8,
}

impl Incident<'_> {
    fn params(&self, t: i64) -> Params {
        Params {
            t,
            allow_research: self.allow_research,
            feature_min_conf: self.feature_min_conf,
            level_min_w: self.level_min_w,
        }
    }

    /// The surface at epoch start T, built in full.
    pub fn surface(&self, t: i64) -> Result<Surface> {
        self.build(t, None, None)
    }

    /// The surface for the host's time `now` (UTC seconds): at the start of the epoch that
    /// holds `now`. A device that was suspended for hours calls this once on waking; a surface
    /// depends on its inputs and T only, so there is nothing to replay.
    pub fn surface_at(&self, now: i64) -> Result<Surface> {
        if self.epoch_s <= 0 {
            return Err(Error::invalid("the epoch length must be positive"));
        }
        self.surface(epoch_of(now, self.epoch_s))
    }

    /// The surface at epoch start T from `prev`, re-fusing only the blocks whose inputs changed
    /// between `before` (the inputs `prev` was built from) and `self`. Equal to
    /// [`Incident::surface`] at T, which tests check.
    pub fn update(&self, t: i64, prev: &Surface, before: &Incident) -> Result<Surface> {
        match self.dirty_blocks(t, prev, before) {
            None => self.surface(t),
            Some(only) => self.build(t, Some(prev), Some(&only)),
        }
    }

    /// The blocks to re-fuse, or None when everything must be rebuilt.
    fn dirty_blocks(&self, t: i64, prev: &Surface, before: &Incident) -> Option<Vec<BlockId>> {
        let same_base = self.terrain.hash() == before.terrain.hash()
            && prev.h.terrain == self.terrain.hash()
            && self.prior == before.prior
            && self.rules == before.rules
            && self.epoch_s == before.epoch_s
            && self.params(0) == before.params(0)
            && self.id == before.id;
        if !same_base {
            return None;
        }
        let sp = self.terrain.spacing().ticks();
        fn by_hash<'b>(v: &[&'b HashedLayer]) -> BTreeMap<[u8; 32], &'b Layer> {
            v.iter().map(|l| (l.hash(), l.layer())).collect()
        }
        let (a, b) = (by_hash(&before.layers), by_hash(&self.layers));
        let mut changed = BTreeSet::new();
        let only_a = a.iter().filter(|(h, _)| !b.contains_key(*h));
        let only_b = b.iter().filter(|(h, _)| !a.contains_key(*h));
        for (_, l) in only_a.chain(only_b) {
            changed.extend(touched_blocks(l, sp));
        }
        let t0 = prev.h.t;
        if t >= prev.h.same_until || t < t0 {
            for (h, l) in &b {
                if !a.contains_key(h) {
                    continue;
                }
                let m = &l.m;
                let act = |t: i64| m.valid_from <= t && t < m.valid_until;
                let fading = |t: i64| m.fade_from < t && t < m.valid_until;
                if act(t0) != act(t) || fading(t0) || fading(t) {
                    changed.extend(touched_blocks(l, sp));
                }
            }
        }
        let (oa, ob): (BTreeSet<_>, BTreeSet<_>) = (
            before.overlay.iter().map(|it| it.bytes()).collect(),
            self.overlay.iter().map(|it| it.bytes()).collect(),
        );
        for it in before.overlay.iter().chain(&self.overlay) {
            if oa.contains(&it.bytes()) != ob.contains(&it.bytes()) {
                changed.extend(overlay_blocks(it, sp));
            }
        }
        Some(
            changed
                .into_iter()
                .filter(|b| self.terrain.block_index(*b).is_some())
                .collect(),
        )
    }

    fn build(&self, t: i64, prev: Option<&Surface>, only: Option<&[BlockId]>) -> Result<Surface> {
        if self.epoch_s <= 0 || self.epoch_s > u32::MAX as i64 || t != epoch_of(t, self.epoch_s) {
            return Err(Error::invalid("T must be the start of an epoch"));
        }
        if self.rules.is_empty() || self.rules.len() > 255 {
            return Err(Error::invalid("a surface has 1-255 profiles"));
        }
        for r in self.rules {
            r.check()?;
        }
        for it in &self.overlay {
            it.check()?;
        }
        // Each block is costed as soon as it is fused, so only the blocks in flight hold
        // fusion state: peak memory grows with the surface, not with the fusion state.
        let (built, rec) = fuse::fuse_with(
            self.terrain,
            self.prior,
            &self.layers,
            &self.overlay,
            &self.params(t),
            only,
            |k, applied, fb| cost_block(self.terrain, k, &fb, applied, self.rules),
        );
        if rec.applied.len() > 253 {
            return Err(Error::invalid("a surface sources at most 253 layers"));
        }
        let mut blocks = match (prev, only) {
            (Some(p), Some(_)) => p.blocks.clone(),
            _ => BTreeMap::new(),
        };
        for (id, sb) in built {
            blocks.insert(id.key(), (id, sb));
        }
        // Layers that can never apply to this surface do not bound how long it stays the same.
        let permanent: BTreeSet<[u8; 32]> = rec
            .left_out
            .iter()
            .filter(|(_, why)| matches!(why, LeftOut::ResearchOnly | LeftOut::Spacing))
            .map(|(h, _)| *h)
            .collect();
        let ms: Vec<&Manifest> = self
            .layers
            .iter()
            .filter(|l| {
                !permanent.contains(&l.hash())
                    && (l.layer().m.licence != LICENCE_RESEARCH || self.allow_research)
            })
            .map(|l| &l.layer().m)
            .collect();
        let h = Header {
            fuse_version: FUSE_VERSION,
            incident: self.id,
            spacing: self.terrain.spacing().ticks() as u32,
            epoch_s: self.epoch_s as u32,
            t,
            same_until: same_until(t, self.epoch_s, &ms),
            terrain: self.terrain.hash(),
            prior: rec.prior,
            overlay: rec.overlay,
            allow_research: self.allow_research as u8,
            feature_min_conf: self.feature_min_conf,
            level_min_w: self.level_min_w,
            applied: rec
                .applied
                .iter()
                .map(|a| AppliedRef {
                    hash: a.hash,
                    kind: kind_u8(a.kind),
                    observed_at: a.observed_at,
                    valid_until: a.valid_until,
                })
                .collect(),
            left_out: rec
                .left_out
                .iter()
                .map(|(h, why)| (*h, why.code()))
                .collect(),
            profiles: self.rules.iter().map(|r| r.bytes()).collect(),
        };
        Ok(Surface { h, blocks })
    }
}

// ---- reading one post ----------------------------------------------------------------------------

/// Everything the base and the surface say about one post, for a host's own ranking (landing
/// zones, say): the terrain's values, the fused top class and its certainty, where that came
/// from, and per profile the time per metre, the risk that the cell is closed and why.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Post {
    /// Height in decimetres, or [`crate::terrain::DEM_NONE`].
    pub dm: i32,
    /// Steepness-max class 0-15, or [`crate::terrain::STEEP_NONE`].
    pub steep: u8,
    /// The base land-cover code and its confidence bit.
    pub cover: u8,
    pub conf: u8,
    /// The base linear-feature bits ([`crate::lines`]).
    pub lines: u16,
    /// The fused top land-cover class and its certainty (1/255).
    pub top: u8,
    pub cert: u8,
    /// The layer that decided the post (its hash), `None` for the base or the team overlay.
    pub layer: Option<[u8; 32]>,
    /// Whether a team overlay item decided the post.
    pub team: bool,
    /// Per profile, in the surface's order: (time per metre in the profile's unit, 0 =
    /// impassable; risk the cell is closed, 1/255; why it is closed, `cost::WHY_*`).
    pub profiles: Vec<(u16, u8, u8)>,
}

/// The [`Post`] at global post (i, j), if both the terrain and the surface hold its block.
pub fn post(ter: &Terrain, s: &Surface, i: i64, j: i64) -> Option<Post> {
    let (id, q) = BlockId::of_post(i, j);
    let k = ter.block_index(id)?;
    let (_, b) = s.blocks.get(&id.key())?;
    let src = b.src[q];
    Some(Post {
        dm: ter.dm(k)[q],
        steep: ter.steep(k)[q],
        cover: ter.cover(k)[q],
        conf: ter.conf(k)[q],
        lines: ter.lines(k)[q],
        top: b.top[q],
        cert: b.cert[q],
        layer: (src != SRC_BASE && src != SRC_TEAM).then(|| b.table[src as usize - 1]),
        team: src == SRC_TEAM,
        profiles: b
            .prof
            .iter()
            .map(|p| (p.t[q], p.risk[q], p.why[q]))
            .collect(),
    })
}

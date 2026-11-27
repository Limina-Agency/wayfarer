//! The stage-1 base: per post of the global lattice, the height, the steepness-max class, the
//! land-cover class with its confidence bit, and the linear-feature mask, for a sorted set of
//! 64 × 64 blocks. Compiled once per region and reused across incidents and OSM refreshes.
//!
//! Planes, block-major (block k's posts at `k * 4096 ..`, row-major from the south-west):
//! - `dm`: height in decimetres (i32), [`DEM_NONE`] where there is no height;
//! - `steep`: steepness-max class 0-15 (the largest Horn slope class of any source pixel in
//!   the cell), [`STEEP_NONE`] where none;
//! - `cover`: land-cover code 0-15 (0 = none; see [`crate::profile`] for the codes);
//! - `conf`: the land-cover confidence bit (1 = two sources agree);
//! - `lines`: the linear-feature mask, base bits only ([`crate::lines::BASE_BITS`]).
//!
//! ## Terrain file (sidecar, v0)
//!
//! Until format 4 fixes the layouts, the base ships as a file beside the package:
//!
//! ```text
//! "WTR0", major u16 = 0, minor u16 = 0, spacing u32 (ticks), block count u32,
//! index: per block (key u64, payload length u32), in key order,
//! payloads in key order, then the terrain hash (32 bytes).
//! payload: dem plane, steep plane, cover plane (code | conf << 4), lines plane,
//!          each a varint length and the plane.
//! ```
//!
//! Every plane but the heights is a [`crate::codec::plane_encode`] plane. The height plane is
//! a coding byte, then 0 = predicted residuals, or 1 = zlib of those, the shorter kept:
//! a presence byte (0 all present, 1 a 512-byte presence bitmap follows, 2 none present), then
//! one zig-zag varint per present post: the residual against the LOCO-I median edge predictor
//! on (west, south, south-west), along rows from the south-west; the first post of a row
//! predicts from the first post of the row below, the very first from 0. Absent posts take the
//! predicted value for later predictions.
//!
//! The terrain hash is over the decoded planes (blake3 of `"wis0/terrain"`, the block keys,
//! `dm` little-endian, `steep`, `cover`, `conf`, `lines` little-endian), so it does not depend
//! on the codings a writer picked.

use crate::codec::{self, Reader, put_varint, unzigzag, zigzag};
use crate::lattice::{BLOCK, BLOCK_POSTS, BlockId, Spacing};
use crate::lines::BASE_BITS;
use crate::{Error, Result};

pub const MAGIC: &[u8; 4] = b"WTR0";
pub const MAJOR: u16 = 0;
pub const MINOR: u16 = 0;
/// No height at this post.
pub const DEM_NONE: i32 = i32::MIN;
/// No steepness class at this post.
pub const STEEP_NONE: u8 = 0xFF;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Terrain {
    spacing: Spacing,
    blocks: Vec<BlockId>,
    pub(crate) dm: Vec<i32>,
    pub(crate) steep: Vec<u8>,
    pub(crate) cover: Vec<u8>,
    pub(crate) conf: Vec<u8>,
    pub(crate) lines: Vec<u16>,
    hash: [u8; 32],
}

impl Terrain {
    /// Builds a base from block-major planes; `blocks` must be sorted and unique.
    pub fn new(
        spacing: Spacing,
        blocks: Vec<BlockId>,
        dm: Vec<i32>,
        steep: Vec<u8>,
        cover: Vec<u8>,
        conf: Vec<u8>,
        lines: Vec<u16>,
    ) -> Result<Terrain> {
        if blocks.windows(2).any(|w| w[0] >= w[1]) {
            return Err(Error::invalid("terrain blocks must be sorted and unique"));
        }
        let n = blocks.len() * BLOCK_POSTS;
        if [dm.len(), steep.len(), cover.len(), conf.len(), lines.len()]
            .iter()
            .any(|&l| l != n)
        {
            return Err(Error::invalid(format!(
                "terrain planes must hold {n} posts"
            )));
        }
        if steep.iter().any(|&s| s > 15 && s != STEEP_NONE) {
            return Err(Error::invalid("steepness class above 15"));
        }
        if cover.iter().any(|&c| c > 15) || conf.iter().any(|&c| c > 1) {
            return Err(Error::invalid(
                "land cover is a 4-bit code and a confidence bit",
            ));
        }
        if lines.iter().any(|&l| l & !BASE_BITS != 0) {
            return Err(Error::invalid(
                "line mask uses bits reserved for surface effects",
            ));
        }
        let mut t = Terrain {
            spacing,
            blocks,
            dm,
            steep,
            cover,
            conf,
            lines,
            hash: [0; 32],
        };
        t.hash = t.compute_hash();
        Ok(t)
    }

    fn compute_hash(&self) -> [u8; 32] {
        let mut h = blake3::Hasher::new();
        h.update(b"wis0/terrain");
        for b in &self.blocks {
            h.update(&b.key().to_le_bytes());
        }
        let mut buf = Vec::with_capacity(self.dm.len() * 4);
        for v in &self.dm {
            buf.extend_from_slice(&v.to_le_bytes());
        }
        h.update(&buf);
        h.update(&self.steep);
        h.update(&self.cover);
        h.update(&self.conf);
        buf.clear();
        for v in &self.lines {
            buf.extend_from_slice(&v.to_le_bytes());
        }
        h.update(&buf);
        *h.finalize().as_bytes()
    }

    pub fn spacing(&self) -> Spacing {
        self.spacing
    }
    /// The blocks, sorted.
    pub fn blocks(&self) -> &[BlockId] {
        &self.blocks
    }
    pub fn hash(&self) -> [u8; 32] {
        self.hash
    }
    /// Index of a block in [`Terrain::blocks`].
    pub fn block_index(&self, b: BlockId) -> Option<usize> {
        self.blocks.binary_search(&b).ok()
    }
    fn range(k: usize) -> std::ops::Range<usize> {
        k * BLOCK_POSTS..(k + 1) * BLOCK_POSTS
    }
    /// Heights (dm) of block `k`.
    pub fn dm(&self, k: usize) -> &[i32] {
        &self.dm[Self::range(k)]
    }
    pub fn steep(&self, k: usize) -> &[u8] {
        &self.steep[Self::range(k)]
    }
    pub fn cover(&self, k: usize) -> &[u8] {
        &self.cover[Self::range(k)]
    }
    pub fn conf(&self, k: usize) -> &[u8] {
        &self.conf[Self::range(k)]
    }
    pub fn lines(&self, k: usize) -> &[u16] {
        &self.lines[Self::range(k)]
    }

    pub fn write(&self) -> Vec<u8> {
        let pays: Vec<Vec<u8>> = (0..self.blocks.len()).map(|k| self.payload(k)).collect();
        let mut o = Vec::with_capacity(
            16 + 12 * pays.len() + pays.iter().map(Vec::len).sum::<usize>() + 32,
        );
        o.extend_from_slice(MAGIC);
        o.extend_from_slice(&MAJOR.to_le_bytes());
        o.extend_from_slice(&MINOR.to_le_bytes());
        o.extend_from_slice(&(self.spacing.ticks() as u32).to_le_bytes());
        o.extend_from_slice(&(self.blocks.len() as u32).to_le_bytes());
        for (b, p) in self.blocks.iter().zip(&pays) {
            o.extend_from_slice(&b.key().to_le_bytes());
            o.extend_from_slice(&(p.len() as u32).to_le_bytes());
        }
        for p in &pays {
            o.extend_from_slice(p);
        }
        o.extend_from_slice(&self.hash);
        o
    }

    fn payload(&self, k: usize) -> Vec<u8> {
        let mut o = Vec::new();
        let mut put = |p: Vec<u8>| {
            put_varint(&mut o, p.len() as u64);
            o.extend_from_slice(&p);
        };
        put(dem_encode(self.dm(k)));
        put(codec::plane_encode(
            &self.steep(k).iter().map(|&x| x as u16).collect::<Vec<_>>(),
            false,
        ));
        put(codec::plane_encode(
            &self
                .cover(k)
                .iter()
                .zip(self.conf(k))
                .map(|(&c, &b)| (c | b << 4) as u16)
                .collect::<Vec<_>>(),
            false,
        ));
        put(codec::plane_encode(self.lines(k), true));
        o
    }

    /// Reads and verifies a terrain file (every plane decoded, the hash recomputed).
    pub fn read(b: &[u8]) -> Result<Terrain> {
        let mut r = Reader::new(b, 0, "terrain file");
        if r.take(4).map_err(|_| Error::Magic("terrain file"))? != MAGIC {
            return Err(Error::Magic("terrain file"));
        }
        let major = r.u16()?;
        if major != MAJOR {
            return Err(Error::Version {
                what: "terrain file",
                found: major,
                known: MAJOR,
            });
        }
        let _minor = r.u16()?;
        let spacing = Spacing::new(r.u32()? as i64)?;
        let n = r.u32()? as usize;
        if n > b.len() / 12 {
            return Err(Error::Truncated("terrain file"));
        }
        let mut index = Vec::with_capacity(n);
        for _ in 0..n {
            index.push((BlockId::from_key(r.u64()?), r.u32()? as usize));
        }
        let posts = n * BLOCK_POSTS;
        let (mut dm, mut steep, mut cover, mut conf, mut lines) = (
            Vec::with_capacity(posts),
            Vec::with_capacity(posts),
            Vec::with_capacity(posts),
            Vec::with_capacity(posts),
            Vec::with_capacity(posts),
        );
        let mut blocks = Vec::with_capacity(n);
        for (id, len) in index {
            let p = r.take(len)?;
            let bad = || Error::invalid(format!("bad payload in terrain block {id:?}"));
            let mut pr = Reader::new(p, 0, "terrain block");
            let mut plane = || -> Result<&[u8]> {
                let l = pr.len()?;
                pr.take(l)
            };
            dm.extend(dem_decode(plane()?).ok_or_else(bad)?);
            steep.extend(
                codec::plane_decode(plane()?, false)
                    .ok_or_else(bad)?
                    .into_iter()
                    .map(|x| x as u8),
            );
            for x in codec::plane_decode(plane()?, false).ok_or_else(bad)? {
                cover.push((x & 15) as u8);
                conf.push((x >> 4) as u8);
            }
            lines.extend(codec::plane_decode(plane()?, true).ok_or_else(bad)?);
            pr.end()?;
            blocks.push(id);
        }
        let stored = r.h32()?;
        r.end()?;
        let t = Terrain::new(spacing, blocks, dm, steep, cover, conf, lines)?;
        if t.hash != stored {
            return Err(Error::Hash("terrain"));
        }
        Ok(t)
    }
}

fn predict(v: &[i64], r: usize, c: usize) -> i64 {
    let at = |rr: usize, cc: usize| v[rr * BLOCK + cc];
    match (r, c) {
        (0, 0) => 0,
        (0, _) => at(0, c - 1),
        (_, 0) => at(r - 1, 0),
        _ => {
            let (a, b, d) = (at(r, c - 1), at(r - 1, c), at(r - 1, c - 1));
            if d >= a.max(b) {
                a.min(b)
            } else if d <= a.min(b) {
                a.max(b)
            } else {
                a + b - d
            }
        }
    }
}

fn dem_residuals(dm: &[i32]) -> Vec<u8> {
    let present = dm.iter().filter(|&&v| v != DEM_NONE).count();
    let mut out = Vec::new();
    if present == 0 {
        out.push(2);
        return out;
    }
    if present == BLOCK_POSTS {
        out.push(0);
    } else {
        out.push(1);
        let mut bm = [0u8; BLOCK_POSTS / 8];
        for (i, &v) in dm.iter().enumerate() {
            if v != DEM_NONE {
                bm[i / 8] |= 1 << (i % 8);
            }
        }
        out.extend_from_slice(&bm);
    }
    let mut rec = vec![0i64; BLOCK_POSTS];
    for r in 0..BLOCK {
        for c in 0..BLOCK {
            let p = predict(&rec, r, c);
            let v = dm[r * BLOCK + c];
            if v == DEM_NONE {
                rec[r * BLOCK + c] = p;
            } else {
                put_varint(&mut out, zigzag(v as i64 - p));
                rec[r * BLOCK + c] = v as i64;
            }
        }
    }
    out
}

fn dem_encode(dm: &[i32]) -> Vec<u8> {
    let res = dem_residuals(dm);
    let z = codec::zlib(&res);
    let mut o = Vec::with_capacity(1 + res.len().min(z.len() + 5));
    if z.len() + 5 < res.len() {
        o.push(1);
        put_varint(&mut o, res.len() as u64);
        o.extend_from_slice(&z);
    } else {
        o.push(0);
        o.extend_from_slice(&res);
    }
    o
}

fn dem_decode(b: &[u8]) -> Option<Vec<i32>> {
    let owned;
    let res: &[u8] = match *b.first()? {
        0 => &b[1..],
        1 => {
            let mut r = Reader::new(b, 1, "heights");
            let n = r.varint().ok()? as usize;
            if n > 1 + BLOCK_POSTS / 8 + BLOCK_POSTS * 10 {
                return None;
            }
            owned = codec::unzlib(&b[r.pos..], n)?;
            &owned
        }
        _ => return None,
    };
    let mut r = Reader::new(res, 1, "heights");
    let present: Option<&[u8]> = match *res.first()? {
        0 => None,
        1 => Some(r.take(BLOCK_POSTS / 8).ok()?),
        2 => return r.at_end().then(|| vec![DEM_NONE; BLOCK_POSTS]),
        _ => return None,
    };
    let mut rec = vec![0i64; BLOCK_POSTS];
    let mut out = vec![DEM_NONE; BLOCK_POSTS];
    for row in 0..BLOCK {
        for c in 0..BLOCK {
            let i = row * BLOCK + c;
            let p = predict(&rec, row, c);
            if present.is_none_or(|bm| bm[i / 8] >> (i % 8) & 1 == 1) {
                let v = p.checked_add(unzigzag(r.varint().ok()?))?;
                let v32 = i32::try_from(v).ok().filter(|&x| x != DEM_NONE)?;
                out[i] = v32;
                rec[i] = v;
            } else {
                rec[i] = p;
            }
        }
    }
    r.at_end().then_some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(blocks: &[BlockId]) -> Terrain {
        let n = blocks.len() * BLOCK_POSTS;
        let dm: Vec<i32> = (0..n)
            .map(|i| {
                if i % 997 == 5 {
                    DEM_NONE
                } else {
                    8_000 + (i % 64) as i32 * 3 - (i / 64 % 64) as i32 * 7
                }
            })
            .collect();
        let steep = (0..n)
            .map(|i| {
                if i % 1013 == 0 {
                    STEEP_NONE
                } else {
                    (i % 16) as u8
                }
            })
            .collect();
        let cover = (0..n).map(|i| (i / 100 % 16) as u8).collect();
        let conf = (0..n).map(|i| (i / 300 % 2) as u8).collect();
        let lines = (0..n)
            .map(|i| if i % 64 == 10 { crate::lines::PATH } else { 0 })
            .collect();
        Terrain::new(
            Spacing::new(120).unwrap(),
            blocks.to_vec(),
            dm,
            steep,
            cover,
            conf,
            lines,
        )
        .unwrap()
    }

    #[test]
    fn file_round_trip() {
        let t = sample(&[BlockId { bi: -3, bj: 7 }, BlockId { bi: 2, bj: 7 }]);
        let f = t.write();
        let back = Terrain::read(&f).unwrap();
        assert_eq!(back, t);
        let mut bad = f.clone();
        bad[30] ^= 0x40;
        assert!(Terrain::read(&bad).is_err());
        assert!(Terrain::read(&f[..f.len() - 1]).is_err());
    }

    #[test]
    fn heights_codings() {
        let all: Vec<i32> = (0..BLOCK_POSTS as i32)
            .map(|i| 12_345 + i % 64 - i / 64)
            .collect();
        let none = vec![DEM_NONE; BLOCK_POSTS];
        let mut some = all.clone();
        some[7] = DEM_NONE;
        for v in [all, none, some] {
            assert_eq!(dem_decode(&dem_encode(&v)).unwrap(), v);
        }
    }

    #[test]
    fn checks_planes() {
        let s = Spacing::new(120).unwrap();
        let b = vec![BlockId { bi: 0, bj: 0 }];
        let n = BLOCK_POSTS;
        assert!(
            Terrain::new(
                s,
                b.clone(),
                vec![0; n],
                vec![16; n],
                vec![0; n],
                vec![0; n],
                vec![0; n]
            )
            .is_err()
        );
        assert!(
            Terrain::new(
                s,
                b.clone(),
                vec![0; n],
                vec![0; n],
                vec![0; n],
                vec![0; n],
                vec![crate::lines::TEAM_BLOCK; n]
            )
            .is_err()
        );
        assert!(
            Terrain::new(
                s,
                b,
                vec![0; n - 1],
                vec![0; n],
                vec![0; n],
                vec![0; n],
                vec![0; n]
            )
            .is_err()
        );
    }
}

//! Byte-level codings shared by the terrain, layer and surface files: LEB128 varints, zig-zag,
//! a bounds-checked reader, palette-runs over a block plane, and zlib.
//!
//! zlib output depends on the compressor, which is fine: every identity in this crate is a
//! hash of decoded values, never of compressed bytes. Writers built with the workspace's
//! `flate2` (zlib-rs, level 9) give the same bytes on every target.

use crate::lattice::BLOCK_POSTS;
use crate::{Error, Result};
use std::io::{Read, Write};

pub fn put_varint(out: &mut Vec<u8>, mut v: u64) {
    while v >= 0x80 {
        out.push((v as u8) | 0x80);
        v >>= 7;
    }
    out.push(v as u8);
}

pub fn zigzag(v: i64) -> u64 {
    ((v << 1) ^ (v >> 63)) as u64
}

pub fn unzigzag(u: u64) -> i64 {
    ((u >> 1) as i64) ^ -((u & 1) as i64)
}

pub(crate) fn put_str(o: &mut Vec<u8>, s: &str) {
    put_varint(o, s.len() as u64);
    o.extend_from_slice(s.as_bytes());
}

/// A bounds-checked little-endian reader; every short read is `Error::Truncated(what)`.
pub(crate) struct Reader<'a> {
    pub b: &'a [u8],
    pub pos: usize,
    what: &'static str,
}

impl<'a> Reader<'a> {
    pub fn new(b: &'a [u8], pos: usize, what: &'static str) -> Self {
        Reader { b, pos, what }
    }
    pub fn take(&mut self, n: usize) -> Result<&'a [u8]> {
        let end = self.pos.checked_add(n).ok_or(Error::Truncated(self.what))?;
        let s = self
            .b
            .get(self.pos..end)
            .ok_or(Error::Truncated(self.what))?;
        self.pos = end;
        Ok(s)
    }
    pub fn u8(&mut self) -> Result<u8> {
        Ok(self.take(1)?[0])
    }
    pub fn u16(&mut self) -> Result<u16> {
        Ok(u16::from_le_bytes(self.take(2)?.try_into().unwrap()))
    }
    pub fn u32(&mut self) -> Result<u32> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }
    pub fn u64(&mut self) -> Result<u64> {
        Ok(u64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }
    pub fn i64(&mut self) -> Result<i64> {
        Ok(i64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }
    pub fn h32(&mut self) -> Result<[u8; 32]> {
        Ok(self.take(32)?.try_into().unwrap())
    }
    pub fn varint(&mut self) -> Result<u64> {
        let mut v = 0u64;
        let mut shift = 0;
        loop {
            let b = self.u8()?;
            v |= ((b & 0x7F) as u64) << shift;
            if b & 0x80 == 0 {
                return Ok(v);
            }
            shift += 7;
            if shift > 63 {
                return Err(Error::invalid(format!("varint too long in {}", self.what)));
            }
        }
    }
    /// A varint used as a length or count: at most what the remaining bytes could hold.
    pub fn len(&mut self) -> Result<usize> {
        let n = self.varint()?;
        if n > (self.b.len() - self.pos) as u64 {
            return Err(Error::Truncated(self.what));
        }
        Ok(n as usize)
    }
    pub fn str(&mut self) -> Result<String> {
        let n = self.len()?;
        String::from_utf8(self.take(n)?.to_vec())
            .map_err(|_| Error::invalid(format!("text in {} is not UTF-8", self.what)))
    }
    pub fn at_end(&self) -> bool {
        self.pos == self.b.len()
    }
    pub fn end(&self) -> Result<()> {
        if self.at_end() {
            Ok(())
        } else {
            Err(Error::invalid(format!("trailing bytes in {}", self.what)))
        }
    }
}

/// Palette-runs over one plane: the palette size and the distinct values in first-seen order
/// (varints), then runs `varint((len - 1) << k | index)` with `k` the palette index width
/// (at least 1 bit).
pub fn palette_runs_encode(v: &[u16]) -> Vec<u8> {
    let mut pal: Vec<u16> = Vec::new();
    let idx: Vec<usize> = v
        .iter()
        .map(|x| match pal.iter().position(|p| p == x) {
            Some(k) => k,
            None => {
                pal.push(*x);
                pal.len() - 1
            }
        })
        .collect();
    let k = index_bits(pal.len());
    let mut buf = Vec::new();
    put_varint(&mut buf, pal.len() as u64);
    for &p in &pal {
        put_varint(&mut buf, p as u64);
    }
    let mut i = 0;
    while i < idx.len() {
        let mut j = i + 1;
        while j < idx.len() && idx[j] == idx[i] {
            j += 1;
        }
        put_varint(&mut buf, (((j - i - 1) as u64) << k) | idx[i] as u64);
        i = j;
    }
    buf
}

fn index_bits(n: usize) -> u64 {
    (usize::BITS - (n.max(2) - 1).leading_zeros()) as u64
}

/// Decodes a palette-runs plane of exactly `n` values.
pub fn palette_runs_decode(buf: &[u8], n: usize) -> Option<Vec<u16>> {
    let mut r = Reader::new(buf, 0, "plane");
    let np = r.len().ok()?;
    let mut pal = Vec::with_capacity(np);
    for _ in 0..np {
        pal.push(u16::try_from(r.varint().ok()?).ok()?);
    }
    let k = index_bits(np);
    let mut out = Vec::with_capacity(n);
    while !r.at_end() {
        let x = r.varint().ok()?;
        let len = (x >> k) as usize + 1;
        if out.len() + len > n {
            return None;
        }
        out.extend(std::iter::repeat_n(
            *pal.get((x & ((1 << k) - 1)) as usize)?,
            len,
        ));
    }
    (out.len() == n).then_some(out)
}

pub fn zlib(raw: &[u8]) -> Vec<u8> {
    let mut e = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::new(9));
    e.write_all(raw).expect("writing to a Vec");
    e.finish().expect("writing to a Vec")
}

/// Inflates a zlib stream that must hold exactly `n` bytes.
pub fn unzlib(z: &[u8], n: usize) -> Option<Vec<u8>> {
    let mut raw = Vec::with_capacity(n);
    flate2::read::ZlibDecoder::new(z)
        .take(n as u64 + 1)
        .read_to_end(&mut raw)
        .ok()?;
    (raw.len() == n).then_some(raw)
}

/// A block plane of `BLOCK_POSTS` values as stored in a surface or terrain file: a coding byte,
/// then 0 palette-runs, or 1 zlib of the raw plane (u8, or u16 little-endian when `wide`). The
/// writer keeps the shorter, palette-runs on a tie.
pub fn plane_encode(v: &[u16], wide: bool) -> Vec<u8> {
    debug_assert_eq!(v.len(), BLOCK_POSTS);
    let pr = palette_runs_encode(v);
    let raw: Vec<u8> = if wide {
        v.iter().flat_map(|x| x.to_le_bytes()).collect()
    } else {
        v.iter().map(|&x| x as u8).collect()
    };
    let z = zlib(&raw);
    let mut o = Vec::with_capacity(1 + pr.len().min(z.len()));
    if z.len() < pr.len() {
        o.push(1);
        o.extend_from_slice(&z);
    } else {
        o.push(0);
        o.extend_from_slice(&pr);
    }
    o
}

/// Decodes a block plane; a narrow plane's values are checked to fit in a byte.
pub fn plane_decode(b: &[u8], wide: bool) -> Option<Vec<u16>> {
    let v = match *b.first()? {
        0 => palette_runs_decode(&b[1..], BLOCK_POSTS)?,
        1 => {
            let raw = unzlib(&b[1..], BLOCK_POSTS * if wide { 2 } else { 1 })?;
            if wide {
                raw.as_chunks::<2>()
                    .0
                    .iter()
                    .map(|c| u16::from_le_bytes(*c))
                    .collect()
            } else {
                raw.into_iter().map(|x| x as u16).collect()
            }
        }
        _ => return None,
    };
    (wide || v.iter().all(|&x| x <= 255)).then_some(v)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn varints_and_zigzag() {
        for v in [0u64, 1, 127, 128, 300, u64::MAX] {
            let mut o = Vec::new();
            put_varint(&mut o, v);
            assert_eq!(Reader::new(&o, 0, "t").varint().unwrap(), v);
        }
        for v in [0i64, -1, 1, i64::MIN, i64::MAX] {
            assert_eq!(unzigzag(zigzag(v)), v);
        }
        assert!(Reader::new(&[0x80], 0, "t").varint().is_err());
        assert!(Reader::new(&[5, 1], 0, "t").len().is_err());
    }

    #[test]
    fn planes_round_trip() {
        let a: Vec<u16> = (0..BLOCK_POSTS)
            .map(|i| ((i / 7) % 5 * 300 + i % 3) as u16)
            .collect();
        let b: Vec<u16> = vec![42; BLOCK_POSTS];
        let c: Vec<u16> = (0..BLOCK_POSTS).map(|i| (i * 7919 % 256) as u16).collect();
        for (v, wide) in [(&a, true), (&b, false), (&c, false)] {
            let e = plane_encode(v, wide);
            assert_eq!(&plane_decode(&e, wide).unwrap(), v);
        }
        assert_eq!(plane_encode(&b, false)[0], 0);
        assert!(plane_decode(&plane_encode(&a, true), false).is_none());
        assert!(palette_runs_decode(&palette_runs_encode(&b), BLOCK_POSTS - 1).is_none());
    }
}

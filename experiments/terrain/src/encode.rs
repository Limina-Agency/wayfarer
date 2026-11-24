//! Block encodings measured in E1. Every block is 64 x 64 posts, row-major from the
//! south-west; all-none blocks are not stored.
//!
//! DEM: a flag byte (0 = all present, 1 = a 512-byte presence bitmap follows), then one
//! zig-zag LEB128 varint per present post: the residual against a predictor. NoData posts
//! are skipped and take the predicted value for later predictions.
//! - `Delta`: west neighbour; the first post of a row predicts from the first post of the
//!   row below; the very first post from 0 (format-3.md 9.2's "zig-zag delta varint").
//! - `Med`: the LOCO-I median edge detector on (west, south, south-west).
//!
//! Nibble layers (steepness-max, land cover): `Packed` 2048 bytes, or `Runs`: varint
//! `(run_len - 1) << 4 | value` over the row-major scan.

use crate::lattice::{BLOCK, BLOCK_POSTS, div_round};
use crate::raster::DEM_NONE;
use std::io::Write;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pred {
    Delta,
    Med,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Units {
    Metre,
    Decimetre,
}

pub fn put_varint(out: &mut Vec<u8>, mut v: u64) {
    while v >= 0x80 {
        out.push((v as u8) | 0x80);
        v >>= 7;
    }
    out.push(v as u8);
}

pub fn get_varint(buf: &[u8], pos: &mut usize) -> Option<u64> {
    let mut v = 0u64;
    let mut shift = 0;
    loop {
        let b = *buf.get(*pos)?;
        *pos += 1;
        v |= ((b & 0x7F) as u64) << shift;
        if b & 0x80 == 0 {
            return Some(v);
        }
        shift += 7;
        if shift > 63 {
            return None;
        }
    }
}

fn zz(v: i64) -> u64 {
    ((v << 1) ^ (v >> 63)) as u64
}
fn unzz(u: u64) -> i64 {
    ((u >> 1) as i64) ^ -((u & 1) as i64)
}

fn predict(v: &[i64], r: usize, c: usize, pred: Pred) -> i64 {
    let at = |rr: usize, cc: usize| v[rr * BLOCK + cc];
    match (r, c) {
        (0, 0) => 0,
        (0, _) => at(0, c - 1),
        (_, 0) => at(r - 1, 0),
        _ => match pred {
            Pred::Delta => at(r, c - 1),
            Pred::Med => {
                let (a, b, cc) = (at(r, c - 1), at(r - 1, c), at(r - 1, c - 1));
                if cc >= a.max(b) {
                    a.min(b)
                } else if cc <= a.min(b) {
                    a.max(b)
                } else {
                    a + b - cc
                }
            }
        },
    }
}

/// Values in the stored unit (metres rounded half away from zero, or decimetres).
pub fn to_units(dm: &[i32], units: Units) -> Vec<Option<i64>> {
    dm.iter()
        .map(|&v| {
            if v == DEM_NONE {
                None
            } else {
                Some(match units {
                    Units::Decimetre => v as i64,
                    Units::Metre => div_round(v as i128, 10) as i64,
                })
            }
        })
        .collect()
}

pub fn encode_dem(vals: &[Option<i64>], pred: Pred) -> Option<Vec<u8>> {
    if vals.iter().all(|v| v.is_none()) {
        return None;
    }
    let mut out = Vec::new();
    let full = vals.iter().all(|v| v.is_some());
    if full {
        out.push(0);
    } else {
        out.push(1);
        let mut bm = [0u8; BLOCK_POSTS / 8];
        for (i, v) in vals.iter().enumerate() {
            if v.is_some() {
                bm[i / 8] |= 1 << (i % 8);
            }
        }
        out.extend_from_slice(&bm);
    }
    let mut rec = vec![0i64; BLOCK_POSTS];
    for r in 0..BLOCK {
        for c in 0..BLOCK {
            let p = predict(&rec, r, c, pred);
            match vals[r * BLOCK + c] {
                Some(v) => {
                    put_varint(&mut out, zz(v - p));
                    rec[r * BLOCK + c] = v;
                }
                None => rec[r * BLOCK + c] = p,
            }
        }
    }
    Some(out)
}

pub fn decode_dem(buf: &[u8], pred: Pred) -> Option<Vec<Option<i64>>> {
    let mut pos = 1usize;
    let bm: Option<&[u8]> = match *buf.first()? {
        0 => None,
        1 => {
            pos += BLOCK_POSTS / 8;
            Some(buf.get(1..1 + BLOCK_POSTS / 8)?)
        }
        _ => return None,
    };
    let mut rec = vec![0i64; BLOCK_POSTS];
    let mut out = vec![None; BLOCK_POSTS];
    for r in 0..BLOCK {
        for c in 0..BLOCK {
            let i = r * BLOCK + c;
            let p = predict(&rec, r, c, pred);
            let present = bm.is_none_or(|b| b[i / 8] & (1 << (i % 8)) != 0);
            if present {
                let v = p + unzz(get_varint(buf, &mut pos)?);
                rec[i] = v;
                out[i] = Some(v);
            } else {
                rec[i] = p;
            }
        }
    }
    (pos == buf.len()).then_some(out)
}


struct BitW {
    out: Vec<u8>,
    acc: u64,
    n: u32,
}
impl BitW {
    fn put(&mut self, v: u64, bits: u32) {
        for i in (0..bits).rev() {
            self.acc = (self.acc << 1) | ((v >> i) & 1);
            self.n += 1;
            if self.n == 8 {
                self.out.push(self.acc as u8);
                self.acc = 0;
                self.n = 0;
            }
        }
    }
    fn finish(mut self) -> Vec<u8> {
        if self.n > 0 {
            self.out.push((self.acc << (8 - self.n)) as u8);
        }
        self.out
    }
}
struct BitR<'a> {
    buf: &'a [u8],
    pos: usize,
}
impl BitR<'_> {
    fn get(&mut self, bits: u32) -> Option<u64> {
        let mut v = 0u64;
        for _ in 0..bits {
            let b = *self.buf.get(self.pos / 8)?;
            v = (v << 1) | ((b >> (7 - self.pos % 8)) & 1) as u64;
            self.pos += 1;
        }
        Some(v)
    }
}

const RICE_ESC: u64 = 24;

fn rice_cost(u: u64, k: u32) -> u64 {
    let q = u >> k;
    if q < RICE_ESC { q + 1 + k as u64 } else { RICE_ESC + 32 }
}

/// Like `encode_dem`, but the residuals are Rice-coded with one parameter k (4 bits) per
/// 64-post row, chosen to minimise the row's bits (ties: smaller k). A quotient of 24 or
/// more is an escape: 24 one-bits, then the zig-zag residual in 32 bits.
pub fn encode_dem_rice(vals: &[Option<i64>], pred: Pred) -> Option<Vec<u8>> {
    if vals.iter().all(|v| v.is_none()) {
        return None;
    }
    let mut head = Vec::new();
    let full = vals.iter().all(|v| v.is_some());
    if full {
        head.push(0);
    } else {
        head.push(1);
        let mut bm = [0u8; BLOCK_POSTS / 8];
        for (i, v) in vals.iter().enumerate() {
            if v.is_some() {
                bm[i / 8] |= 1 << (i % 8);
            }
        }
        head.extend_from_slice(&bm);
    }
    let mut rec = vec![0i64; BLOCK_POSTS];
    let mut w = BitW { out: Vec::new(), acc: 0, n: 0 };
    for r in 0..BLOCK {
        let mut us = Vec::with_capacity(BLOCK);
        for c in 0..BLOCK {
            let p = predict(&rec, r, c, pred);
            match vals[r * BLOCK + c] {
                Some(v) => {
                    us.push(zz(v - p));
                    rec[r * BLOCK + c] = v;
                }
                None => rec[r * BLOCK + c] = p,
            }
        }
        let k = (0..16u32).min_by_key(|&k| (us.iter().map(|&u| rice_cost(u, k)).sum::<u64>(), k)).unwrap();
        w.put(k as u64, 4);
        for u in us {
            let q = u >> k;
            if q < RICE_ESC {
                w.put((1u64 << q) - 1, q as u32);
                w.put(0, 1);
                w.put(u & ((1u64 << k) - 1), k);
            } else {
                w.put((1u64 << RICE_ESC) - 1, RICE_ESC as u32);
                w.put(u, 32);
            }
        }
    }
    head.extend_from_slice(&w.finish());
    Some(head)
}

pub fn decode_dem_rice(buf: &[u8], pred: Pred) -> Option<Vec<Option<i64>>> {
    let (bm, start): (Option<&[u8]>, usize) = match *buf.first()? {
        0 => (None, 1),
        1 => (Some(buf.get(1..1 + BLOCK_POSTS / 8)?), 1 + BLOCK_POSTS / 8),
        _ => return None,
    };
    let mut rd = BitR { buf: &buf[start..], pos: 0 };
    let mut rec = vec![0i64; BLOCK_POSTS];
    let mut out = vec![None; BLOCK_POSTS];
    for r in 0..BLOCK {
        let k = rd.get(4)? as u32;
        for c in 0..BLOCK {
            let i = r * BLOCK + c;
            let p = predict(&rec, r, c, pred);
            if bm.is_none_or(|b| b[i / 8] & (1 << (i % 8)) != 0) {
                let mut q = 0u64;
                while q < RICE_ESC && rd.get(1)? == 1 {
                    q += 1;
                }
                let u = if q == RICE_ESC { rd.get(32)? } else { (q << k) | rd.get(k)? };
                let v = p + unzz(u);
                rec[i] = v;
                out[i] = Some(v);
            } else {
                rec[i] = p;
            }
        }
    }
    (rd.pos.div_ceil(8) == buf.len() - start).then_some(out)
}

pub fn packed_nibbles(v: &[u8]) -> Vec<u8> {
    v.chunks(2).map(|p| (p[0] & 15) | ((p.get(1).copied().unwrap_or(0) & 15) << 4)).collect()
}

pub fn runs_nibbles(v: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < v.len() {
        let mut j = i + 1;
        while j < v.len() && v[j] == v[i] {
            j += 1;
        }
        put_varint(&mut out, (((j - i - 1) as u64) << 4) | (v[i] & 15) as u64);
        i = j;
    }
    out
}

pub fn decode_runs(buf: &[u8]) -> Option<Vec<u8>> {
    let mut out = Vec::with_capacity(BLOCK_POSTS);
    let mut pos = 0;
    while pos < buf.len() {
        let x = get_varint(buf, &mut pos)?;
        let n = (x >> 4) as usize + 1;
        out.extend(std::iter::repeat_n((x & 15) as u8, n));
    }
    (out.len() == BLOCK_POSTS).then_some(out)
}

/// zlib level 9 of a payload: a reference for what a general-purpose coder would save.
pub fn zlib_len(b: &[u8]) -> usize {
    let mut e = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::new(9));
    e.write_all(b).unwrap();
    e.finish().unwrap().len()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn dem_roundtrip() {
        let mut vals: Vec<Option<i64>> = (0..BLOCK_POSTS as i64).map(|i| Some((i * 37) % 900 - 400)).collect();
        vals[5] = None;
        vals[4095] = None;
        for pred in [Pred::Delta, Pred::Med] {
            let b = encode_dem(&vals, pred).unwrap();
            assert_eq!(decode_dem(&b, pred).unwrap(), vals);
            let b = encode_dem_rice(&vals, pred).unwrap();
            assert_eq!(decode_dem_rice(&b, pred).unwrap(), vals);
        }
        let mut big = vals.clone();
        big[7] = Some(30_000);
        big[8] = Some(-30_000);
        let b = encode_dem_rice(&big, Pred::Med).unwrap();
        assert_eq!(decode_dem_rice(&b, Pred::Med).unwrap(), big);
        let full: Vec<Option<i64>> = (0..BLOCK_POSTS as i64).map(|i| Some(i % 64 + i / 64)).collect();
        let b = encode_dem(&full, Pred::Med).unwrap();
        assert_eq!(b.len(), 1 + BLOCK_POSTS); // a plane: one byte per residual
        assert_eq!(decode_dem(&b, Pred::Med).unwrap(), full);
    }
    #[test]
    fn runs_roundtrip() {
        let v: Vec<u8> = (0..BLOCK_POSTS).map(|i| ((i / 100) % 16) as u8).collect();
        assert_eq!(decode_runs(&runs_nibbles(&v)).unwrap(), v);
    }
}

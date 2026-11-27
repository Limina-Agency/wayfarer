//! A small reader of tiled or stripped GeoTIFFs and COGs: the first image only (overviews are
//! never read), one band, tiles read on demand from any `Read + Seek` source.
//!
//! It covers what the terrain sources use: classic TIFF and BigTIFF in either byte order,
//! no compression or Deflate, predictors 1 (none), 2 (horizontal) and 3 (floating point),
//! samples u8, i8, u16, i16, u32, i32, f32 and f64, the GeoTIFF scale, tie point and
//! GeoKeys, and GDAL's NoData tag. Anything else is refused with a message, never guessed.

use crate::{Error, Result};
use std::io::{Read, Seek, SeekFrom};

/// A sample type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SampleType {
    U8,
    I8,
    U16,
    I16,
    U32,
    I32,
    F32,
    F64,
}

impl SampleType {
    pub fn bytes(self) -> usize {
        match self {
            SampleType::U8 | SampleType::I8 => 1,
            SampleType::U16 | SampleType::I16 => 2,
            SampleType::U32 | SampleType::I32 | SampleType::F32 => 4,
            SampleType::F64 => 8,
        }
    }
}

/// What a source's first image header says.
#[derive(Clone, Debug)]
pub struct TiffInfo {
    pub width: u32,
    pub height: u32,
    pub sample: SampleType,
    pub compression: u16,
    pub predictor: u16,
    /// Chunk (tile or strip) size in pixels; strips are as wide as the image.
    pub chunk_w: u32,
    pub chunk_h: u32,
    pub offsets: Vec<u64>,
    pub byte_counts: Vec<u64>,
    pub big_endian: bool,
    /// ModelPixelScale (x, y) and the first ModelTiepoint (i, j, x, y).
    pub scale: Option<[f64; 2]>,
    pub tiepoint: Option<[f64; 4]>,
    pub has_transformation: bool,
    /// GeoKeys with a value held inline (id, value).
    pub geo_keys: Vec<(u16, u16)>,
    pub nodata: Option<f64>,
}

impl TiffInfo {
    pub fn chunks_across(&self) -> u32 {
        self.width.div_ceil(self.chunk_w)
    }
    pub fn chunks_down(&self) -> u32 {
        self.height.div_ceil(self.chunk_h)
    }
    pub fn geo_key(&self, id: u16) -> Option<u16> {
        self.geo_keys.iter().find(|k| k.0 == id).map(|k| k.1)
    }
}

struct Bytes {
    be: bool,
}

impl Bytes {
    fn u16(&self, b: &[u8]) -> u16 {
        let a = [b[0], b[1]];
        if self.be {
            u16::from_be_bytes(a)
        } else {
            u16::from_le_bytes(a)
        }
    }
    fn u32(&self, b: &[u8]) -> u32 {
        let a = [b[0], b[1], b[2], b[3]];
        if self.be {
            u32::from_be_bytes(a)
        } else {
            u32::from_le_bytes(a)
        }
    }
    fn u64(&self, b: &[u8]) -> u64 {
        let a: [u8; 8] = b[..8].try_into().unwrap();
        if self.be {
            u64::from_be_bytes(a)
        } else {
            u64::from_le_bytes(a)
        }
    }
}

fn bad(name: &str, msg: impl std::fmt::Display) -> Error {
    Error::Source(format!("{name}: {msg}"))
}

/// Largest tag array read (tile tables of a planet-wide COG stay well below).
const MAX_TAG_BYTES: u64 = 64 << 20;

/// Reads the first image's header.
pub fn read_info<R: Read + Seek>(r: &mut R, name: &str) -> Result<TiffInfo> {
    let mut h = [0u8; 16];
    r.seek(SeekFrom::Start(0)).map_err(|e| bad(name, e))?;
    r.read_exact(&mut h[..8]).map_err(|e| bad(name, e))?;
    let be = match &h[..2] {
        b"II" => false,
        b"MM" => true,
        _ => return Err(bad(name, "not a TIFF")),
    };
    let bo = Bytes { be };
    let (big, ifd) = match bo.u16(&h[2..4]) {
        42 => (false, bo.u32(&h[4..8]) as u64),
        43 => {
            r.read_exact(&mut h[8..16]).map_err(|e| bad(name, e))?;
            if bo.u16(&h[4..6]) != 8 {
                return Err(bad(name, "BigTIFF offset size is not 8"));
            }
            (true, bo.u64(&h[8..16]))
        }
        _ => return Err(bad(name, "not a TIFF")),
    };
    r.seek(SeekFrom::Start(ifd)).map_err(|e| bad(name, e))?;
    let count = if big {
        let mut b = [0u8; 8];
        r.read_exact(&mut b).map_err(|e| bad(name, e))?;
        bo.u64(&b)
    } else {
        let mut b = [0u8; 2];
        r.read_exact(&mut b).map_err(|e| bad(name, e))?;
        bo.u16(&b) as u64
    };
    if count > 4096 {
        return Err(bad(name, "implausible IFD entry count"));
    }
    let entry_len = if big { 20 } else { 12 };
    let mut raw = vec![0u8; count as usize * entry_len];
    r.read_exact(&mut raw).map_err(|e| bad(name, e))?;
    // (tag, type, count, inline value bytes)
    let mut entries = Vec::with_capacity(count as usize);
    for e in raw.chunks_exact(entry_len) {
        let tag = bo.u16(&e[0..2]);
        let ty = bo.u16(&e[2..4]);
        let (n, val) = if big {
            (bo.u64(&e[4..12]), e[12..20].to_vec())
        } else {
            (bo.u32(&e[4..8]) as u64, e[8..12].to_vec())
        };
        entries.push((tag, ty, n, val));
    }
    let type_size = |ty: u16| -> Option<u64> {
        Some(match ty {
            1 | 2 | 6 | 7 => 1,
            3 | 8 => 2,
            4 | 9 | 11 => 4,
            5 | 10 | 12 | 16 | 17 => 8,
            _ => return None,
        })
    };
    // Reads a tag's values as f64 (numbers) or bytes (ASCII).
    let values = |tag: u16, r: &mut R| -> Result<Option<(u16, Vec<u8>, u64)>> {
        let Some((_, ty, n, val)) = entries.iter().find(|e| e.0 == tag) else {
            return Ok(None);
        };
        let size = type_size(*ty).ok_or_else(|| bad(name, format!("tag {tag} type {ty}")))?;
        let total = size
            .checked_mul(*n)
            .filter(|&t| t <= MAX_TAG_BYTES)
            .ok_or_else(|| bad(name, format!("tag {tag} too large")))?;
        let inline = if big { 8 } else { 4 };
        let bytes = if total <= inline {
            val[..total as usize].to_vec()
        } else {
            let off = if big { bo.u64(val) } else { bo.u32(val) as u64 };
            let mut b = vec![0u8; total as usize];
            r.seek(SeekFrom::Start(off)).map_err(|e| bad(name, e))?;
            r.read_exact(&mut b).map_err(|e| bad(name, e))?;
            b
        };
        Ok(Some((*ty, bytes, *n)))
    };
    let num = |ty: u16, b: &[u8], i: usize| -> f64 {
        match ty {
            1 => b[i] as f64,
            6 => b[i] as i8 as f64,
            3 => bo.u16(&b[2 * i..]) as f64,
            8 => bo.u16(&b[2 * i..]) as i16 as f64,
            4 => bo.u32(&b[4 * i..]) as f64,
            9 => bo.u32(&b[4 * i..]) as i32 as f64,
            11 => f32::from_bits(bo.u32(&b[4 * i..])) as f64,
            12 => f64::from_bits(bo.u64(&b[8 * i..])),
            16 => bo.u64(&b[8 * i..]) as f64,
            17 => bo.u64(&b[8 * i..]) as i64 as f64,
            5 => bo.u32(&b[8 * i..]) as f64 / bo.u32(&b[8 * i + 4..]) as f64,
            10 => bo.u32(&b[8 * i..]) as i32 as f64 / bo.u32(&b[8 * i + 4..]) as i32 as f64,
            _ => f64::NAN,
        }
    };
    let nums = |tag: u16, r: &mut R| -> Result<Option<Vec<f64>>> {
        Ok(values(tag, r)?.map(|(ty, b, n)| (0..n as usize).map(|i| num(ty, &b, i)).collect()))
    };
    let int = |tag: u16, r: &mut R| -> Result<Option<u64>> {
        Ok(nums(tag, r)?
            .and_then(|v| v.first().copied())
            .map(|x| x as u64))
    };
    let width = int(256, r)?.ok_or_else(|| bad(name, "no ImageWidth"))? as u32;
    let height = int(257, r)?.ok_or_else(|| bad(name, "no ImageLength"))? as u32;
    let bits = int(258, r)?.unwrap_or(1);
    let compression = int(259, r)?.unwrap_or(1) as u16;
    let spp = int(277, r)?.unwrap_or(1);
    let planar = int(284, r)?.unwrap_or(1);
    let predictor = int(317, r)?.unwrap_or(1) as u16;
    let format = int(339, r)?.unwrap_or(1);
    if spp != 1 {
        return Err(bad(name, format!("{spp} samples per pixel; one band only")));
    }
    let _ = planar;
    let sample = match (format, bits) {
        (1, 8) => SampleType::U8,
        (2, 8) => SampleType::I8,
        (1, 16) => SampleType::U16,
        (2, 16) => SampleType::I16,
        (1, 32) => SampleType::U32,
        (2, 32) => SampleType::I32,
        (3, 32) => SampleType::F32,
        (3, 64) => SampleType::F64,
        _ => {
            return Err(bad(
                name,
                format!("sample format {format} with {bits} bits"),
            ));
        }
    };
    if !matches!(compression, 1 | 8 | 32946) {
        return Err(bad(
            name,
            format!("compression {compression}; only none and Deflate are read"),
        ));
    }
    let float = matches!(sample, SampleType::F32 | SampleType::F64);
    let predictor_ok = match predictor {
        1 => true,
        2 => !float,
        3 => float,
        _ => false,
    };
    if !predictor_ok {
        return Err(bad(name, format!("predictor {predictor} with {sample:?}")));
    }
    let (chunk_w, chunk_h, offsets, byte_counts) = match int(322, r)? {
        Some(tw) => {
            let th = int(323, r)?.ok_or_else(|| bad(name, "TileWidth without TileLength"))?;
            let o = nums(324, r)?.ok_or_else(|| bad(name, "no TileOffsets"))?;
            let c = nums(325, r)?.ok_or_else(|| bad(name, "no TileByteCounts"))?;
            (tw as u32, th as u32, o, c)
        }
        None => {
            let rps = int(278, r)?.unwrap_or(height as u64).min(height as u64);
            let o = nums(273, r)?.ok_or_else(|| bad(name, "no StripOffsets"))?;
            let c = nums(279, r)?.ok_or_else(|| bad(name, "no StripByteCounts"))?;
            (width, rps as u32, o, c)
        }
    };
    if chunk_w == 0 || chunk_h == 0 || width == 0 || height == 0 {
        return Err(bad(name, "empty image or chunk"));
    }
    if (chunk_w as u64) * (chunk_h as u64) * sample.bytes() as u64 > 256 << 20 {
        return Err(bad(name, "chunks over 256 MiB"));
    }
    let info = TiffInfo {
        width,
        height,
        sample,
        compression,
        predictor,
        chunk_w,
        chunk_h,
        offsets: offsets.iter().map(|&x| x as u64).collect(),
        byte_counts: byte_counts.iter().map(|&x| x as u64).collect(),
        big_endian: be,
        scale: nums(33550, r)?
            .filter(|v| v.len() >= 2)
            .map(|v| [v[0], v[1]]),
        tiepoint: nums(33922, r)?
            .filter(|v| v.len() >= 6)
            .map(|v| [v[0], v[1], v[3], v[4]]),
        has_transformation: entries.iter().any(|e| e.0 == 34264),
        geo_keys: match nums(34735, r)? {
            Some(k) if k.len() >= 4 => {
                let n = k[3] as usize;
                (0..n)
                    .filter_map(|i| {
                        let e = k.get(4 + 4 * i..8 + 4 * i)?;
                        (e[1] == 0.0).then_some((e[0] as u16, e[3] as u16))
                    })
                    .collect()
            }
            _ => Vec::new(),
        },
        nodata: match values(42113, r)? {
            Some((2, b, _)) => {
                let s = String::from_utf8_lossy(&b);
                let s = s.trim_matches(char::from(0)).trim();
                Some(if s.eq_ignore_ascii_case("nan") {
                    f64::NAN
                } else {
                    s.parse::<f64>()
                        .map_err(|_| bad(name, format!("NoData {s:?}")))?
                })
            }
            _ => None,
        },
    };
    let n = info.chunks_across() as usize * info.chunks_down() as usize;
    if info.offsets.len() != n || info.byte_counts.len() != n {
        return Err(bad(name, "chunk table does not match the image"));
    }
    Ok(info)
}

/// Reads chunk `idx`'s stored (compressed) bytes, reusing `comp`'s allocation.
pub fn read_chunk<R: Read + Seek>(
    r: &mut R,
    info: &TiffInfo,
    name: &str,
    idx: usize,
    comp: &mut Vec<u8>,
) -> Result<Vec<u8>> {
    let len = info.byte_counts[idx];
    if len > 512 << 20 {
        return Err(bad(name, format!("chunk {idx} too large")));
    }
    comp.resize(len as usize, 0);
    r.seek(SeekFrom::Start(info.offsets[idx]))
        .map_err(|e| bad(name, e))?;
    r.read_exact(comp).map_err(|e| bad(name, e))?;
    Ok(std::mem::take(comp))
}

/// Decompresses a chunk's bytes, undoes the predictor and converts each sample with `conv`
/// (row-major, `chunk_w * chunk_h` values; rows a short last strip lacks are `fill`).
pub fn decode_chunk<T: Copy>(
    info: &TiffInfo,
    name: &str,
    idx: usize,
    comp: &[u8],
    fill: T,
    conv: impl Fn(f64) -> T,
) -> Result<Vec<T>> {
    let (w, ch) = (info.chunk_w as usize, info.chunk_h as usize);
    // Rows actually stored: a strip at the bottom may be short.
    let rows = if info.chunk_w == info.width && info.offsets.len() == info.chunks_down() as usize {
        let r0 = (idx / info.chunks_across() as usize) * ch;
        ch.min(info.height as usize - r0)
    } else {
        ch
    };
    let bps = info.sample.bytes();
    let want = w * rows * bps;
    let mut raw = if info.compression == 1 {
        comp.to_vec()
    } else {
        let mut out = Vec::with_capacity(want);
        flate2::read::ZlibDecoder::new(comp)
            .take(want as u64)
            .read_to_end(&mut out)
            .map_err(|e| bad(name, format!("chunk {idx}: {e}")))?;
        out
    };
    if raw.len() < want {
        return Err(bad(name, format!("chunk {idx} is short")));
    }
    raw.truncate(want);
    let be = info.big_endian;
    let mut vals = vec![fill; w * ch];
    match info.predictor {
        3 => {
            // Floating-point predictor: bytes differenced along the row, then split into
            // planes by significance (most significant first).
            let rowb = w * bps;
            for row in raw.chunks_exact_mut(rowb) {
                for i in 1..rowb {
                    row[i] = row[i].wrapping_add(row[i - 1]);
                }
            }
            for (y, row) in raw.chunks_exact(rowb).enumerate() {
                for x in 0..w {
                    let mut b = [0u8; 8];
                    for (k, v) in b.iter_mut().take(bps).enumerate() {
                        *v = row[k * w + x];
                    }
                    vals[y * w + x] = conv(match info.sample {
                        SampleType::F32 => f32::from_be_bytes([b[0], b[1], b[2], b[3]]) as f64,
                        _ => f64::from_be_bytes(b),
                    });
                }
            }
        }
        p => {
            let get = |b: &[u8]| -> f64 {
                macro_rules! rd {
                    ($t:ty, $n:expr) => {{
                        let a: [u8; $n] = b[..$n].try_into().unwrap();
                        if be {
                            <$t>::from_be_bytes(a)
                        } else {
                            <$t>::from_le_bytes(a)
                        }
                    }};
                }
                match info.sample {
                    SampleType::U8 => b[0] as f64,
                    SampleType::I8 => b[0] as i8 as f64,
                    SampleType::U16 => rd!(u16, 2) as f64,
                    SampleType::I16 => rd!(i16, 2) as f64,
                    SampleType::U32 => rd!(u32, 4) as f64,
                    SampleType::I32 => rd!(i32, 4) as f64,
                    SampleType::F32 => rd!(f32, 4) as f64,
                    SampleType::F64 => rd!(f64, 8),
                }
            };
            if p == 2 {
                // Horizontal differencing of integers, wrapping in the sample's width.
                for row in raw.chunks_exact_mut(w * bps) {
                    for x in 1..w {
                        let (a, b) = row.split_at_mut(x * bps);
                        let prev = &a[(x - 1) * bps..];
                        let cur = &mut b[..bps];
                        add_wrapping(cur, prev, bps, be);
                    }
                }
            }
            for (i, b) in raw.chunks_exact(bps).enumerate() {
                vals[i] = conv(get(b));
            }
        }
    }
    Ok(vals)
}

fn add_wrapping(cur: &mut [u8], prev: &[u8], n: usize, be: bool) {
    let mut carry = 0u16;
    for k in 0..n {
        let i = if be { n - 1 - k } else { k };
        let s = cur[i] as u16 + prev[i] as u16 + carry;
        cur[i] = s as u8;
        carry = s >> 8;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrapping_add_both_orders() {
        let mut a = [0xFFu8, 0x00];
        add_wrapping(&mut a, &[0x01, 0x00], 2, false);
        assert_eq!(a, [0x00, 0x01]);
        let mut b = [0x00u8, 0xFF];
        add_wrapping(&mut b, &[0x00, 0x01], 2, true);
        assert_eq!(b, [0x01, 0x00]);
    }
}

//! Synthetic GeoTIFF sources for the tests: small files written in memory with any of the
//! reader's codings, and large virtual files whose tiles are computed when read, so a test can
//! compile a wide region without holding (or counting) the source bytes.

#![allow(dead_code)]

use std::io::{Read, Seek, SeekFrom, Write};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Sample {
    U8,
    I16,
    F32,
}

impl Sample {
    fn bytes(self) -> usize {
        match self {
            Sample::U8 => 1,
            Sample::I16 => 2,
            Sample::F32 => 4,
        }
    }
    fn tags(self) -> (u16, u16) {
        // (bits, sample format)
        match self {
            Sample::U8 => (8, 1),
            Sample::I16 => (16, 2),
            Sample::F32 => (32, 3),
        }
    }
}

/// Where a raster sits.
#[derive(Clone, Copy, Debug)]
pub enum Place {
    /// UTM zone (north): top-left corner easting and northing (m), pixel size (m).
    Utm { zone: u16, x: f64, y: f64, px: f64 },
    /// Geographic: tie point (lon, lat) of pixel (0, 0), pixel size (degrees) in x and y, and
    /// whether the tie is the pixel's centre (PixelIsPoint) or its corner.
    Geo {
        lon: f64,
        lat: f64,
        dx: f64,
        dy: f64,
        point: bool,
    },
}

#[derive(Clone)]
pub struct Spec {
    pub width: u32,
    pub height: u32,
    pub tile: u32,
    pub sample: Sample,
    /// 1 none or 8 Deflate.
    pub compression: u16,
    /// 1, 2 (integers) or 3 (floats).
    pub predictor: u16,
    pub big_endian: bool,
    pub bigtiff: bool,
    pub place: Place,
    pub nodata: Option<&'static str>,
}

impl Spec {
    fn across(&self) -> u32 {
        self.width.div_ceil(self.tile)
    }
    fn down(&self) -> u32 {
        self.height.div_ceil(self.tile)
    }
    fn tile_bytes(&self) -> usize {
        (self.tile * self.tile) as usize * self.sample.bytes()
    }
}

struct Out {
    be: bool,
    b: Vec<u8>,
}

impl Out {
    fn u16(&mut self, v: u16) {
        self.b.extend_from_slice(&if self.be {
            v.to_be_bytes()
        } else {
            v.to_le_bytes()
        });
    }
    fn u32(&mut self, v: u32) {
        self.b.extend_from_slice(&if self.be {
            v.to_be_bytes()
        } else {
            v.to_le_bytes()
        });
    }
    fn u64(&mut self, v: u64) {
        self.b.extend_from_slice(&if self.be {
            v.to_be_bytes()
        } else {
            v.to_le_bytes()
        });
    }
    fn f64(&mut self, v: f64) {
        self.u64(v.to_bits());
    }
}

/// The header (everything before the tile data) for tiles of these lengths at `data_at`.
fn header(spec: &Spec, lens: &[u64], data_at: u64) -> Vec<u8> {
    let be = spec.big_endian;
    let big = spec.bigtiff;
    let mut o = Out { be, b: Vec::new() };
    o.b.extend_from_slice(if be { b"MM" } else { b"II" });
    if big {
        o.u16(43);
        o.u16(8);
        o.u16(0);
        o.u64(16);
    } else {
        o.u16(42);
        o.u32(8);
    }
    let (bits, fmt) = spec.sample.tags();
    let mut offsets = Vec::new();
    let mut at = data_at;
    for &l in lens {
        offsets.push(at);
        at += l;
    }
    let (scale, tie, keys): ([f64; 3], [f64; 6], Vec<u16>) = match spec.place {
        Place::Utm { zone, x, y, px } => (
            [px, px, 0.0],
            [0.0, 0.0, 0.0, x, y, 0.0],
            vec![
                1,
                1,
                0,
                4,
                1024,
                0,
                1,
                1,
                1025,
                0,
                1,
                1,
                3072,
                0,
                1,
                32600 + zone,
                3076,
                0,
                1,
                9001,
            ],
        ),
        Place::Geo {
            lon,
            lat,
            dx,
            dy,
            point,
        } => (
            [dx, dy, 0.0],
            [0.0, 0.0, 0.0, lon, lat, 0.0],
            vec![
                1,
                1,
                0,
                3,
                1024,
                0,
                1,
                2,
                1025,
                0,
                1,
                if point { 2 } else { 1 },
                2048,
                0,
                1,
                4326,
            ],
        ),
    };
    // (tag, type, values as u64 or f64 bytes)
    enum V {
        S(Vec<u16>),
        L(Vec<u64>),
        D(Vec<f64>),
        A(Vec<u8>),
    }
    let n = lens.len();
    let mut tags: Vec<(u16, V)> = vec![
        (256, V::S(vec![spec.width as u16])),
        (257, V::S(vec![spec.height as u16])),
        (258, V::S(vec![bits])),
        (259, V::S(vec![spec.compression])),
        (262, V::S(vec![1])),
        (277, V::S(vec![1])),
        (284, V::S(vec![1])),
        (317, V::S(vec![spec.predictor])),
        (322, V::S(vec![spec.tile as u16])),
        (323, V::S(vec![spec.tile as u16])),
        (324, V::L(offsets)),
        (325, V::L(lens.to_vec())),
        (339, V::S(vec![fmt])),
        (33550, V::D(scale.to_vec())),
        (33922, V::D(tie.to_vec())),
        (34735, V::S(keys)),
    ];
    if spec.width > 65535 || spec.height > 65535 {
        tags[0].1 = V::L(vec![spec.width as u64]);
        tags[1].1 = V::L(vec![spec.height as u64]);
    }
    if let Some(nd) = spec.nodata {
        let mut a = nd.as_bytes().to_vec();
        a.push(0);
        tags.push((42113, V::A(a)));
    }
    let _ = n;
    let entry = if big { 20 } else { 12 };
    let ifd_at = o.b.len();
    let count_len = if big { 8 } else { 2 };
    let mut extra_at = ifd_at + count_len + tags.len() * entry + if big { 8 } else { 4 };
    let mut extra = Out { be, b: Vec::new() };
    let mut entries = Out { be, b: Vec::new() };
    for (tag, v) in &tags {
        let mut data = Out { be, b: Vec::new() };
        let (ty, count) = match v {
            V::S(x) => {
                x.iter().for_each(|&e| data.u16(e));
                (3u16, x.len())
            }
            V::L(x) => {
                if big {
                    x.iter().for_each(|&e| data.u64(e));
                    (16, x.len())
                } else {
                    x.iter().for_each(|&e| data.u32(e as u32));
                    (4, x.len())
                }
            }
            V::D(x) => {
                x.iter().for_each(|&e| data.f64(e));
                (12, x.len())
            }
            V::A(x) => {
                data.b.extend_from_slice(x);
                (2, x.len())
            }
        };
        entries.u16(*tag);
        entries.u16(ty);
        if big {
            entries.u64(count as u64);
        } else {
            entries.u32(count as u32);
        }
        let inline = if big { 8 } else { 4 };
        if data.b.len() <= inline {
            let mut d = data.b.clone();
            d.resize(inline, 0);
            entries.b.extend_from_slice(&d);
        } else {
            if big {
                entries.u64(extra_at as u64);
            } else {
                entries.u32(extra_at as u32);
            }
            extra_at += data.b.len();
            extra.b.extend_from_slice(&data.b);
        }
    }
    if big {
        o.u64(tags.len() as u64);
    } else {
        o.u16(tags.len() as u16);
    }
    o.b.extend_from_slice(&entries.b);
    if big {
        o.u64(0);
    } else {
        o.u32(0);
    }
    o.b.extend_from_slice(&extra.b);
    o.b
}

fn put_sample(spec: &Spec, v: f64, out: &mut [u8]) {
    let be = spec.big_endian;
    match spec.sample {
        Sample::U8 => out[0] = v as u8,
        Sample::I16 => out.copy_from_slice(&if be {
            (v as i16).to_be_bytes()
        } else {
            (v as i16).to_le_bytes()
        }),
        Sample::F32 => out.copy_from_slice(&if be {
            (v as f32).to_be_bytes()
        } else {
            (v as f32).to_le_bytes()
        }),
    }
}

/// One tile's stored bytes (before compression), with the predictor applied.
fn tile_raw(spec: &Spec, ti: u32, f: &dyn Fn(u32, u32) -> f64) -> Vec<u8> {
    let (t, bps) = (spec.tile as usize, spec.sample.bytes());
    let (tr, tc) = (ti / spec.across(), ti % spec.across());
    let mut raw = vec![0u8; t * t * bps];
    for y in 0..t {
        for x in 0..t {
            let (r, c) = (tr * spec.tile + y as u32, tc * spec.tile + x as u32);
            let v = if r < spec.height && c < spec.width {
                f(r, c)
            } else {
                0.0
            };
            put_sample(spec, v, &mut raw[(y * t + x) * bps..(y * t + x + 1) * bps]);
        }
    }
    match spec.predictor {
        2 => {
            // Integers: difference each sample from its west neighbour, wrapping.
            for row in raw.chunks_exact_mut(t * bps) {
                let vals: Vec<i64> = row
                    .chunks_exact(bps)
                    .map(|b| match (bps, spec.big_endian) {
                        (1, _) => b[0] as i64,
                        (2, true) => u16::from_be_bytes([b[0], b[1]]) as i64,
                        (2, false) => u16::from_le_bytes([b[0], b[1]]) as i64,
                        _ => unreachable!(),
                    })
                    .collect();
                for x in (1..t).rev() {
                    let d = (vals[x] - vals[x - 1]).rem_euclid(1 << (8 * bps));
                    let cell = &mut row[x * bps..(x + 1) * bps];
                    match (bps, spec.big_endian) {
                        (1, _) => cell[0] = d as u8,
                        (2, true) => cell.copy_from_slice(&(d as u16).to_be_bytes()),
                        _ => cell.copy_from_slice(&(d as u16).to_le_bytes()),
                    }
                }
            }
        }
        3 if spec.sample == Sample::F32 => {
            // Floats: bytes split into planes by significance (big-endian order), then the
            // row's bytes differenced.
            for row in raw.chunks_exact_mut(t * bps) {
                let mut planes = vec![0u8; t * bps];
                for x in 0..t {
                    let mut b = [0u8; 4];
                    b.copy_from_slice(&row[x * 4..x * 4 + 4]);
                    let v = if spec.big_endian {
                        f32::from_be_bytes(b)
                    } else {
                        f32::from_le_bytes(b)
                    };
                    for (k, byte) in v.to_be_bytes().iter().enumerate() {
                        planes[k * t + x] = *byte;
                    }
                }
                for i in (1..planes.len()).rev() {
                    planes[i] = planes[i].wrapping_sub(planes[i - 1]);
                }
                row.copy_from_slice(&planes);
            }
        }
        _ => {}
    }
    raw
}

/// A whole file in memory.
pub fn tiff(spec: &Spec, f: &dyn Fn(u32, u32) -> f64) -> Vec<u8> {
    let n = spec.across() * spec.down();
    let tiles: Vec<Vec<u8>> = (0..n)
        .map(|ti| {
            let raw = tile_raw(spec, ti, f);
            if spec.compression == 8 {
                let mut z =
                    flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::new(6));
                z.write_all(&raw).unwrap();
                z.finish().unwrap()
            } else {
                raw
            }
        })
        .collect();
    let lens: Vec<u64> = tiles.iter().map(|t| t.len() as u64).collect();
    // The header's length doesn't depend on the offsets' values.
    let probe = header(spec, &lens, 0);
    let mut out = header(spec, &lens, probe.len() as u64);
    for t in tiles {
        out.extend_from_slice(&t);
    }
    out
}

/// A file whose uncompressed tiles are computed as they are read: no bytes are held.
pub struct Virtual {
    spec: Spec,
    head: Vec<u8>,
    f: Box<dyn Fn(u32, u32) -> f64 + Send>,
    pos: u64,
}

impl Virtual {
    pub fn new(spec: Spec, f: impl Fn(u32, u32) -> f64 + Send + 'static) -> Virtual {
        assert_eq!(spec.compression, 1);
        assert_eq!(spec.predictor, 1);
        let n = (spec.across() * spec.down()) as usize;
        let lens = vec![spec.tile_bytes() as u64; n];
        let probe = header(&spec, &lens, 0);
        let head = header(&spec, &lens, probe.len() as u64);
        Virtual {
            spec,
            head,
            f: Box::new(f),
            pos: 0,
        }
    }
    fn len(&self) -> u64 {
        self.head.len() as u64
            + (self.spec.across() * self.spec.down()) as u64 * self.spec.tile_bytes() as u64
    }
}

impl Read for Virtual {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let hl = self.head.len() as u64;
        if self.pos >= self.len() {
            return Ok(0);
        }
        if self.pos < hl {
            let n = buf.len().min((hl - self.pos) as usize);
            buf[..n].copy_from_slice(&self.head[self.pos as usize..self.pos as usize + n]);
            self.pos += n as u64;
            return Ok(n);
        }
        let s = &self.spec;
        let (bps, t, tb) = (
            s.sample.bytes() as u64,
            s.tile as u64,
            s.tile_bytes() as u64,
        );
        let off = self.pos - hl;
        let ti = off / tb;
        let n = buf.len().min((tb - off % tb) as usize);
        let (tr, tc) = (ti as u32 / s.across(), ti as u32 % s.across());
        let mut i = 0;
        while i < n {
            let o = off % tb + i as u64;
            let px = o / bps;
            let (y, x) = ((px / t) as u32, (px % t) as u32);
            let (r, c) = (tr * s.tile + y, tc * s.tile + x);
            let v = if r < s.height && c < s.width {
                (self.f)(r, c)
            } else {
                0.0
            };
            let mut b = [0u8; 4];
            put_sample(s, v, &mut b[..bps as usize]);
            let k = (o % bps) as usize;
            let take = (bps as usize - k).min(n - i);
            buf[i..i + take].copy_from_slice(&b[k..k + take]);
            i += take;
        }
        self.pos += n as u64;
        Ok(n)
    }
}

impl Seek for Virtual {
    fn seek(&mut self, p: SeekFrom) -> std::io::Result<u64> {
        self.pos = match p {
            SeekFrom::Start(x) => x,
            SeekFrom::End(x) => (self.len() as i64 + x) as u64,
            SeekFrom::Current(x) => (self.pos as i64 + x) as u64,
        };
        Ok(self.pos)
    }
}

/// Smooth hills with a cliff: metres at a UTM position.
pub fn hills(e: f64, n: f64) -> f64 {
    // libm, not the platform library: the golden hash depends on these bits.
    let base =
        600.0 + 250.0 * libm::sin(e / 2300.0) * libm::cos(n / 3100.0) + 0.004 * (e - 400_000.0);
    if (e - 424_000.0).abs() < 40.0 {
        base + 30.0
    } else {
        base
    }
}

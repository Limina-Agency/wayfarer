//! Source rasters as the compile reads them: opened files grouped into mosaics of one pixel
//! grid, decoded chunks shared through a cache of bounded size, and pixel windows.
//!
//! A mosaic is every file of one kind on one grid: the same coordinate system, pixel size and
//! pixel phase, so its pixels have global indexes. Where files of a mosaic overlap (GEM-15's
//! squares share a margin of about 2 km), a pixel's height is the mean of the files' valid
//! values, and a class pixel the largest code: neither depends on the order the files came in.

use crate::geo::Crs;
use crate::tiff::{self, TiffInfo};
use crate::{Error, Result};
use std::collections::HashMap;
use std::io::{Read, Seek};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use wayfarer_terrain::lattice::div_round;

/// No height at a pixel (millimetres) or a sample.
pub const MM_NONE: i32 = i32::MIN;

/// What the compile reads from a source.
pub trait ReadSeek: Read + Seek + Send {}
impl<T: Read + Seek + Send> ReadSeek for T {}

/// One source file: a name for messages and the report, and its bytes.
pub struct SourceFile {
    pub name: String,
    pub reader: Box<dyn ReadSeek>,
}

impl SourceFile {
    pub fn new(name: impl Into<String>, reader: impl ReadSeek + 'static) -> SourceFile {
        SourceFile {
            name: name.into(),
            reader: Box::new(reader),
        }
    }
    /// Opens a file from disk.
    pub fn open(path: impl AsRef<std::path::Path>) -> Result<SourceFile> {
        let p = path.as_ref();
        let f =
            std::fs::File::open(p).map_err(|e| Error::Source(format!("{}: {e}", p.display())))?;
        let name = p
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default();
        Ok(SourceFile::new(
            name,
            std::io::BufReader::with_capacity(1 << 16, f),
        ))
    }
}

/// Heights in millimetres or 8-bit class codes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Heights,
    Classes,
}

/// Fixed-point unit of projected coordinates: 1/1024 m.
pub const MM_FP: f64 = 1024.0;

/// A pixel grid: the centre of global pixel (0, 0) and the step, in the CRS's integer units
/// (half-ticks of 1/240 arc-second for geographic grids, 1/1024 m for UTM). Rows run south
/// (geographic: latitude falls; UTM: northing falls), columns east.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Grid {
    pub crs: Crs,
    /// Phase of the pixel centres: x of column 0's centre, y of row 0's centre.
    pub x0: i64,
    pub y0: i64,
    pub dx: i64,
    pub dy: i64,
}

pub(crate) struct Raster {
    pub name: String,
    pub info: TiffInfo,
    /// Global pixel index of the file's pixel (0, 0).
    pub row0: i64,
    pub col0: i64,
    pub nodata: Option<f64>,
    pub reader: Mutex<Box<dyn ReadSeek>>,
}

pub(crate) struct Mosaic {
    pub kind: Kind,
    pub grid: Grid,
    pub rasters: Vec<Raster>,
    /// The mosaic's id in the cache's keys.
    pub id: u32,
}

/// A decoded chunk.
pub(crate) enum Chunk {
    Heights(Vec<i32>),
    Classes(Vec<u8>),
}

impl Chunk {
    fn bytes(&self) -> usize {
        match self {
            Chunk::Heights(v) => v.len() * 4,
            Chunk::Classes(v) => v.len(),
        }
    }
}

/// Decoded chunks shared by every block of the compile, least recently used out first.
pub(crate) struct ChunkCache {
    cap: usize,
    inner: Mutex<CacheInner>,
    pub decoded: AtomicU64,
    pub peak: AtomicU64,
}

struct CacheInner {
    map: HashMap<(u32, u32, u32), (Arc<Chunk>, u64)>,
    bytes: usize,
    clock: u64,
}

impl ChunkCache {
    pub fn new(cap: usize) -> ChunkCache {
        ChunkCache {
            cap,
            inner: Mutex::new(CacheInner {
                map: HashMap::new(),
                bytes: 0,
                clock: 0,
            }),
            decoded: AtomicU64::new(0),
            peak: AtomicU64::new(0),
        }
    }

    fn get(&self, m: &Mosaic, ri: usize, idx: usize) -> Result<Arc<Chunk>> {
        let key = (m.id, ri as u32, idx as u32);
        {
            let mut g = self.inner.lock().unwrap();
            g.clock += 1;
            let now = g.clock;
            if let Some(e) = g.map.get_mut(&key) {
                e.1 = now;
                return Ok(e.0.clone());
            }
        }
        let ras = &m.rasters[ri];
        let comp = {
            let mut r = ras.reader.lock().unwrap();
            let mut buf = Vec::new();
            tiff::read_chunk(&mut *r, &ras.info, &ras.name, idx, &mut buf)?
        };
        let nodata = ras.nodata;
        let chunk = Arc::new(match m.kind {
            Kind::Heights => Chunk::Heights(tiff::decode_chunk(
                &ras.info,
                &ras.name,
                idx,
                &comp,
                MM_NONE,
                |v| height_mm(v, nodata),
            )?),
            Kind::Classes => Chunk::Classes(tiff::decode_chunk(
                &ras.info,
                &ras.name,
                idx,
                &comp,
                0,
                |v| {
                    if !v.is_finite() || nodata == Some(v) || !(0.0..=255.0).contains(&v) {
                        0
                    } else {
                        v as u8
                    }
                },
            )?),
        });
        drop(comp);
        self.decoded.fetch_add(1, Ordering::Relaxed);
        let mut g = self.inner.lock().unwrap();
        g.clock += 1;
        let now = g.clock;
        if let Some(e) = g.map.get(&key) {
            return Ok(e.0.clone());
        }
        g.bytes += chunk.bytes();
        g.map.insert(key, (chunk.clone(), now));
        while g.bytes > self.cap && g.map.len() > 1 {
            let (&old, _) = g
                .map
                .iter()
                .filter(|e| *e.0 != key)
                .min_by_key(|e| e.1.1)
                .unwrap();
            let (c, _) = g.map.remove(&old).unwrap();
            g.bytes -= c.bytes();
        }
        self.peak.fetch_max(g.bytes as u64, Ordering::Relaxed);
        Ok(chunk)
    }
}

/// A source value in metres as whole millimetres, half away from zero; NaN, NoData and
/// values beyond ±20 km are no height.
pub fn height_mm(v: f64, nodata: Option<f64>) -> i32 {
    if !v.is_finite() || nodata.is_some_and(|n| n == v) || !(-20_000.0..=20_000.0).contains(&v) {
        MM_NONE
    } else {
        libm::round(v * 1000.0) as i32
    }
}

fn whole(v: f64, unit: f64, what: &str, name: &str) -> Result<i64> {
    let t = v * unit;
    let r = libm::round(t);
    if !r.is_finite() || (t - r).abs() > 1e-3 {
        return Err(Error::Source(format!(
            "{name}: {what} {v} is not a whole number of grid units"
        )));
    }
    Ok(r as i64)
}

/// Half-ticks (1/240″) per degree.
pub const HALF_TICKS_PER_DEG: f64 = 2.0 * wayfarer_terrain::lattice::TICKS_PER_DEG as f64;

/// Opens a file's header and places it on a grid.
pub(crate) fn open(mut f: SourceFile) -> Result<(Grid, Raster)> {
    let name = f.name.clone();
    let info = tiff::read_info(&mut f.reader, &name)?;
    if info.has_transformation {
        return Err(Error::Source(format!(
            "{name}: rotated rasters are not supported"
        )));
    }
    let (Some(scale), Some(tie)) = (info.scale, info.tiepoint) else {
        return Err(Error::Source(format!(
            "{name}: no GeoTIFF scale and tie point"
        )));
    };
    let model = info.geo_key(1024);
    let crs = match model {
        Some(1) => Crs::from_epsg(
            info.geo_key(3072)
                .ok_or_else(|| Error::Source(format!("{name}: projected without an EPSG code")))?,
        )?,
        Some(2) => Crs::from_epsg(info.geo_key(2048).unwrap_or(4326))?,
        m => return Err(Error::Source(format!("{name}: GTModelType {m:?}"))),
    };
    if crs != Crs::Geographic && info.geo_key(3076).is_some_and(|u| u != 9001) {
        return Err(Error::Source(format!(
            "{name}: linear units are not metres"
        )));
    }
    let point = match info.geo_key(1025) {
        None | Some(1) => false,
        Some(2) => true,
        Some(x) => return Err(Error::Source(format!("{name}: GTRasterType {x}"))),
    };
    let unit = if crs == Crs::Geographic {
        HALF_TICKS_PER_DEG
    } else {
        MM_FP
    };
    let dx = whole(scale[0], unit, "x scale", &name)?;
    let dy = whole(scale[1], unit, "y scale", &name)?;
    if dx <= 0 || dy <= 0 {
        return Err(Error::Source(format!("{name}: non-positive pixel scale")));
    }
    let (ti, tj) = (tie[0], tie[1]);
    let tx = whole(tie[2], unit, "tie x", &name)?;
    let ty = whole(tie[3], unit, "tie y", &name)?;
    // Centre of the file's pixel (0, 0); tie (i, j) is a pixel corner unless PixelIsPoint.
    let ti2 = whole(ti, 2.0, "tie column", &name)?;
    let tj2 = whole(tj, 2.0, "tie row", &name)?;
    let corner = if point { 0 } else { 1 };
    if ((ti2 - corner) * dx) % 2 != 0 || ((tj2 - corner) * dy) % 2 != 0 {
        return Err(Error::Source(format!(
            "{name}: pixel centres between grid units"
        )));
    }
    let cx = tx - (ti2 - corner) * dx / 2;
    let cy = ty + (tj2 - corner) * dy / 2;
    let grid = Grid {
        crs,
        x0: cx.rem_euclid(dx),
        y0: cy.rem_euclid(dy),
        dx,
        dy,
    };
    let col0 = (cx - grid.x0) / dx;
    // Rows run south: global row r has centre y0 - r * dy.
    let row0 = (grid.y0 - cy) / dy;
    let nodata = info.nodata;
    Ok((
        grid,
        Raster {
            name,
            info,
            row0,
            col0,
            nodata,
            reader: Mutex::new(f.reader),
        },
    ))
}

/// Groups files of one kind into mosaics by grid, files sorted by position (never by name).
pub(crate) fn mosaics(files: Vec<SourceFile>, kind: Kind, first_id: u32) -> Result<Vec<Mosaic>> {
    let mut by_grid: Vec<(Grid, Vec<Raster>)> = Vec::new();
    for f in files {
        let (g, r) = open(f)?;
        match by_grid.iter_mut().find(|e| e.0 == g) {
            Some(e) => e.1.push(r),
            None => by_grid.push((g, vec![r])),
        }
    }
    by_grid.sort_by_key(|e| {
        let g = e.0;
        let c = match g.crs {
            Crs::Geographic => (0u8, 0u8, false),
            Crs::Utm { zone, north } => (1, zone, north),
        };
        (c, g.dx, g.dy, g.x0, g.y0)
    });
    Ok(by_grid
        .into_iter()
        .enumerate()
        .map(|(k, (grid, mut rasters))| {
            rasters.sort_by_key(|r| (r.row0, r.col0, r.info.height, r.info.width));
            Mosaic {
                kind,
                grid,
                rasters,
                id: first_id + k as u32,
            }
        })
        .collect())
}

/// Global pixels `rows × cols` from `(r0, c0)`.
pub(crate) struct Window<T> {
    pub r0: i64,
    pub c0: i64,
    pub rows: usize,
    pub cols: usize,
    pub v: Vec<T>,
}

impl<T: Copy> Window<T> {
    #[inline]
    pub fn at(&self, r: i64, c: i64) -> Option<T> {
        let (y, x) = (r - self.r0, c - self.c0);
        if y < 0 || x < 0 || y as usize >= self.rows || x as usize >= self.cols {
            return None;
        }
        Some(self.v[y as usize * self.cols + x as usize])
    }
}

impl Mosaic {
    /// Whether any file has a pixel in the global range.
    pub fn touches(&self, r0: i64, r1: i64, c0: i64, c1: i64) -> bool {
        self.rasters.iter().any(|ras| {
            ras.row0 < r1
                && r0 < ras.row0 + ras.info.height as i64
                && ras.col0 < c1
                && c0 < ras.col0 + ras.info.width as i64
        })
    }

    /// Calls `f(window index, chunk, index in chunk)` for every file pixel in the window.
    fn each_pixel(
        &self,
        cache: &ChunkCache,
        (r0, r1, c0, c1): (i64, i64, i64, i64),
        mut f: impl FnMut(usize, &Chunk, usize),
    ) -> Result<()> {
        let cols = (c1 - c0) as usize;
        for (ri, ras) in self.rasters.iter().enumerate() {
            let (h, w) = (ras.info.height as i64, ras.info.width as i64);
            // The window in the file's own pixels.
            let fr0 = (r0 - ras.row0).max(0);
            let fr1 = (r1 - ras.row0).min(h);
            let fc0 = (c0 - ras.col0).max(0);
            let fc1 = (c1 - ras.col0).min(w);
            if fr0 >= fr1 || fc0 >= fc1 {
                continue;
            }
            let (cw, ch) = (ras.info.chunk_w as i64, ras.info.chunk_h as i64);
            let across = ras.info.chunks_across() as i64;
            for tr in fr0 / ch..=(fr1 - 1) / ch {
                for tc in fc0 / cw..=(fc1 - 1) / cw {
                    let idx = (tr * across + tc) as usize;
                    let chunk = cache.get(self, ri, idx)?;
                    let (y0, y1) = ((tr * ch).max(fr0), ((tr + 1) * ch).min(fr1));
                    let (x0, x1) = ((tc * cw).max(fc0), ((tc + 1) * cw).min(fc1));
                    for y in y0..y1 {
                        let wrow = (y + ras.row0 - r0) as usize * cols;
                        let crow = ((y - tr * ch) * cw) as usize;
                        for x in x0..x1 {
                            f(
                                wrow + (x + ras.col0 - c0) as usize,
                                &chunk,
                                crow + (x - tc * cw) as usize,
                            );
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// Heights (mm) of the global pixels `[r0, r1) × [c0, c1)`; overlapping files' valid
    /// values are averaged.
    pub fn heights(
        &self,
        cache: &ChunkCache,
        r0: i64,
        r1: i64,
        c0: i64,
        c1: i64,
    ) -> Result<Window<i32>> {
        let (rows, cols) = ((r1 - r0) as usize, (c1 - c0) as usize);
        let mut sum = vec![0i64; rows * cols];
        let mut n = vec![0u8; rows * cols];
        self.each_pixel(cache, (r0, r1, c0, c1), |w, chunk, i| {
            if let Chunk::Heights(v) = chunk
                && v[i] != MM_NONE
            {
                sum[w] += v[i] as i64;
                n[w] += 1;
            }
        })?;
        let v = sum
            .iter()
            .zip(&n)
            .map(|(&s, &k)| match k {
                0 => MM_NONE,
                1 => s as i32,
                _ => div_round(s as i128, k as i128) as i32,
            })
            .collect();
        Ok(Window {
            r0,
            c0,
            rows,
            cols,
            v,
        })
    }

    /// Class codes (0 = none) of the global pixels; overlapping files give the largest code.
    pub fn classes(
        &self,
        cache: &ChunkCache,
        r0: i64,
        r1: i64,
        c0: i64,
        c1: i64,
    ) -> Result<Window<u8>> {
        let (rows, cols) = ((r1 - r0) as usize, (c1 - c0) as usize);
        let mut v = vec![0u8; rows * cols];
        self.each_pixel(cache, (r0, r1, c0, c1), |w, chunk, i| {
            if let Chunk::Classes(c) = chunk {
                v[w] = v[w].max(c[i]);
            }
        })?;
        Ok(Window {
            r0,
            c0,
            rows,
            cols,
            v,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heights_round_half_away() {
        assert_eq!(height_mm(1.0625, None), 1063);
        assert_eq!(height_mm(-1.0625, None), -1063);
        assert_eq!(height_mm(212.25, None), 212_250);
        assert_eq!(height_mm(f64::NAN, None), MM_NONE);
        assert_eq!(height_mm(-32767.0, Some(-32767.0)), MM_NONE);
        assert_eq!(height_mm(25_000.0, None), MM_NONE);
    }
}

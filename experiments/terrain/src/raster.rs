//! Source rasters: GeoTIFF/COG (GLO-30, NASADEM, WorldCover) and SRTM HGT, read into
//! integer grids whose pixel centres sit on whole ticks.
//!
//! Rules (feat-td.md, T1): geographic EPSG:4326 only, north-up, one band, no rotation;
//! scale and tie point must land on whole ticks (1/120"), else the file is refused (warp a
//! projected source onto an aligned geographic grid first). Heights become i32 decimetres,
//! `(f32 as f64 * 10).round()` (exact per IEEE, half away from zero).

use crate::lattice::TICKS_PER_DEG;
use std::fs::File;
use std::io::{BufReader, Read};
use tiff::decoder::{Decoder, DecodingResult, Limits};
use tiff::tags::Tag;

pub const DEM_NONE: i32 = i32::MIN;

/// Pixel `(r, c)` has its centre at latitude `lat0 - r * dlat`, longitude `lon0 + c * dlon`
/// (ticks). Rows run north to south as in the files.
#[derive(Clone, Debug)]
pub struct Grid {
    pub lat0: i64,
    pub lon0: i64,
    pub dlat: i64,
    pub dlon: i64,
    pub width: usize,
    pub height: usize,
}

pub struct Raster<T> {
    pub name: String,
    pub grid: Grid,
    pub data: Vec<T>,
}

impl<T: Copy> Raster<T> {
    pub fn at(&self, r: usize, c: usize) -> T {
        self.data[r * self.grid.width + c]
    }
}

/// A latitude/longitude window in ticks, inclusive; pixels whose centres fall in it (plus a
/// one-pixel margin) are decoded.
#[derive(Clone, Copy, Debug)]
pub struct Window {
    pub lat_min: i64,
    pub lat_max: i64,
    pub lon_min: i64,
    pub lon_max: i64,
}

fn to_ticks(deg: f64, what: &str) -> Result<i64, String> {
    let t = deg * TICKS_PER_DEG as f64;
    let r = t.round();
    if !(r.is_finite()) || (t - r).abs() > 1e-3 {
        return Err(format!("{what} {deg} is not on a whole 1/120-arc-second tick"));
    }
    Ok(r as i64)
}

struct GeoInfo {
    grid: Grid,
    nodata: Option<f64>,
}

fn geo_info<R: Read + std::io::Seek>(d: &mut Decoder<R>, name: &str) -> Result<GeoInfo, String> {
    let e = |x: tiff::TiffError| format!("{name}: {x}");
    let (w, h) = d.dimensions().map_err(e)?;
    let scale = d.get_tag_f64_vec(Tag::ModelPixelScaleTag).map_err(e)?;
    let tie = d.get_tag_f64_vec(Tag::ModelTiepointTag).map_err(e)?;
    let keys = d.get_tag_u16_vec(Tag::GeoKeyDirectoryTag).map_err(e)?;
    if d.find_tag(Tag::Unknown(34264)).map_err(e)?.is_some() {
        return Err(format!("{name}: ModelTransformation (rotation) not supported"));
    }
    let spp = d.find_tag_unsigned::<u16>(Tag::SamplesPerPixel).map_err(e)?.unwrap_or(1);
    if spp != 1 {
        return Err(format!("{name}: {spp} bands; one band only"));
    }
    let nodata = match d.find_tag(Tag::GdalNodata).map_err(e)? {
        Some(_) => {
            let s = d.get_tag_ascii_string(Tag::GdalNodata).map_err(e)?;
            Some(s.trim_matches(char::from(0)).trim().parse::<f64>().map_err(|x| format!("{name}: nodata {s:?}: {x}"))?)
        }
        None => None,
    };
    // GeoKeyDirectory: header (4 u16) then (id, location, count, value) entries.
    if keys.len() < 4 {
        return Err(format!("{name}: short GeoKeyDirectory"));
    }
    let n = keys[3] as usize;
    let (mut model, mut raster_type, mut gcs) = (None, None, None);
    for k in 0..n {
        let base = 4 + 4 * k;
        if base + 3 >= keys.len() {
            return Err(format!("{name}: truncated GeoKeyDirectory"));
        }
        let (id, loc, val) = (keys[base], keys[base + 1], keys[base + 3]);
        if loc != 0 {
            continue;
        }
        match id {
            1024 => model = Some(val),
            1025 => raster_type = Some(val),
            2048 => gcs = Some(val),
            _ => {}
        }
    }
    if model != Some(2) {
        return Err(format!("{name}: GTModelType {model:?}, want 2 (geographic)"));
    }
    if gcs != Some(4326) {
        return Err(format!("{name}: GeographicType {gcs:?}, want 4326"));
    }
    let is_point = match raster_type {
        Some(1) | None => false,
        Some(2) => true,
        Some(x) => return Err(format!("{name}: GTRasterType {x}")),
    };
    if scale.len() < 2 || tie.len() < 6 {
        return Err(format!("{name}: short scale or tie point"));
    }
    let dlon = to_ticks(scale[0], "x scale")?;
    let dlat = to_ticks(scale[1], "y scale")?;
    if dlon <= 0 || dlat <= 0 {
        return Err(format!("{name}: non-positive scale"));
    }
    let (ti, tj) = (tie[0], tie[1]);
    if ti.fract() != 0.0 || tj.fract() != 0.0 {
        return Err(format!("{name}: fractional tie point pixel"));
    }
    let x = to_ticks(tie[3], "tie x")?;
    let y = to_ticks(tie[4], "tie y")?;
    // Centre of pixel (0, 0).
    let (mut lon0, mut lat0) = (x - ti as i64 * dlon, y + tj as i64 * dlat);
    if !is_point {
        if dlon % 2 != 0 || dlat % 2 != 0 {
            return Err(format!("{name}: PixelIsArea with odd tick scale"));
        }
        lon0 += dlon / 2;
        lat0 -= dlat / 2;
    }
    if (w as u64) * (h as u64) > (1 << 31) {
        return Err(format!("{name}: {w}x{h} too large"));
    }
    Ok(GeoInfo { grid: Grid { lat0, lon0, dlat, dlon, width: w as usize, height: h as usize }, nodata })
}

/// Pixel row/column range covering a window (centres inside, plus one pixel each side).
fn window_range(g: &Grid, win: Option<Window>) -> (usize, usize, usize, usize) {
    let Some(w) = win else { return (0, g.height, 0, g.width) };
    let clamp = |v: i64, n: usize| v.clamp(0, n as i64) as usize;
    let r0 = clamp((g.lat0 - w.lat_max).div_euclid(g.dlat) - 1, g.height);
    let r1 = clamp((g.lat0 - w.lat_min).div_euclid(g.dlat) + 2, g.height);
    let c0 = clamp((w.lon_min - g.lon0).div_euclid(g.dlon) - 1, g.width);
    let c1 = clamp((w.lon_max - g.lon0).div_euclid(g.dlon) + 2, g.width);
    (r0, r1, c0, c1)
}

/// Decode the chunks (tiles or strips) of a window, converting each sample with `conv`.
fn read_window<T: Copy, F: Fn(&DecodingResult, usize) -> Result<T, String>>(
    path: &str,
    win: Option<Window>,
    fill: T,
    conv: F,
) -> Result<Option<Raster<T>>, String> {
    let name = path.rsplit('/').next().unwrap_or(path).to_string();
    let f = File::open(path).map_err(|e| format!("{path}: {e}"))?;
    let mut limits = Limits::default();
    limits.decoding_buffer_size = 64 << 20;
    let mut d = Decoder::new(BufReader::new(f)).map_err(|e| format!("{name}: {e}"))?.with_limits(limits);
    let info = geo_info(&mut d, &name)?;
    let g = info.grid.clone();
    let (r0, r1, c0, c1) = window_range(&g, win);
    if r0 >= r1 || c0 >= c1 {
        return Ok(None);
    }
    let (cw, ch) = d.chunk_dimensions();
    let (cw, ch) = (cw as usize, ch as usize);
    let across = g.width.div_ceil(cw);
    let (ow, oh) = (c1 - c0, r1 - r0);
    let mut data = vec![fill; ow * oh];
    for tr in r0 / ch..r1.div_ceil(ch) {
        for tc in c0 / cw..c1.div_ceil(cw) {
            let idx = (tr * across + tc) as u32;
            let (dw, dh) = d.chunk_data_dimensions(idx);
            let res = d.read_chunk(idx).map_err(|e| format!("{name} chunk {idx}: {e}"))?;
            for y in 0..dh as usize {
                let r = tr * ch + y;
                if r < r0 || r >= r1 {
                    continue;
                }
                for x in 0..dw as usize {
                    let c = tc * cw + x;
                    if c < c0 || c >= c1 {
                        continue;
                    }
                    data[(r - r0) * ow + (c - c0)] = conv(&res, y * dw as usize + x)?;
                }
            }
        }
    }
    let _ = info.nodata;
    let grid = Grid {
        lat0: g.lat0 - r0 as i64 * g.dlat,
        lon0: g.lon0 + c0 as i64 * g.dlon,
        dlat: g.dlat,
        dlon: g.dlon,
        width: ow,
        height: oh,
    };
    Ok(Some(Raster { name, grid, data }))
}

fn nodata_of(path: &str) -> Result<Option<f64>, String> {
    let f = File::open(path).map_err(|e| format!("{path}: {e}"))?;
    let mut d = Decoder::new(BufReader::new(f)).map_err(|e| format!("{path}: {e}"))?;
    Ok(geo_info(&mut d, path)?.nodata)
}

/// A DEM GeoTIFF (Float32 or Int16) as decimetres.
pub fn read_dem_geotiff(path: &str, win: Option<Window>) -> Result<Option<Raster<i32>>, String> {
    let nodata = nodata_of(path)?;
    read_window(path, win, DEM_NONE, |res, k| {
        let v: f64 = match res {
            DecodingResult::F32(v) => v[k] as f64,
            DecodingResult::F64(v) => v[k],
            DecodingResult::I16(v) => v[k] as f64,
            DecodingResult::U16(v) => v[k] as f64,
            DecodingResult::I32(v) => v[k] as f64,
            _ => return Err("unsupported DEM sample type".into()),
        };
        if !v.is_finite() || nodata == Some(v) || v < -20_000.0 || v > 20_000.0 {
            return Ok(DEM_NONE);
        }
        Ok((v * 10.0).round() as i32)
    })
}

/// A u8 class GeoTIFF (WorldCover). WorldCover is a palette image, which the `tiff` crate
/// refuses to decode, so tiles are read here: deflate (8/32946) or none, no predictor, one
/// u8 band, tiled. Values are the class codes (the palette is display only).
pub fn read_class_geotiff(path: &str, win: Option<Window>) -> Result<Option<Raster<u8>>, String> {
    use std::io::{Seek, SeekFrom};
    let name = path.rsplit('/').next().unwrap_or(path).to_string();
    let e = |x: tiff::TiffError| format!("{name}: {x}");
    let f = File::open(path).map_err(|x| format!("{path}: {x}"))?;
    let mut d = Decoder::new(BufReader::new(f)).map_err(e)?;
    let info = geo_info(&mut d, &name)?;
    let g = info.grid.clone();
    let bps = d.find_tag_unsigned::<u16>(Tag::BitsPerSample).map_err(e)?.unwrap_or(1);
    let comp = d.find_tag_unsigned::<u16>(Tag::Compression).map_err(e)?.unwrap_or(1);
    let pred = d.find_tag_unsigned::<u16>(Tag::Predictor).map_err(e)?.unwrap_or(1);
    let tw = d.find_tag_unsigned::<u32>(Tag::TileWidth).map_err(e)?;
    let th = d.find_tag_unsigned::<u32>(Tag::TileLength).map_err(e)?;
    let (Some(tw), Some(th)) = (tw, th) else { return Err(format!("{name}: not tiled")) };
    if bps != 8 || pred != 1 || !(comp == 1 || comp == 8 || comp == 32946) {
        return Err(format!("{name}: bits {bps}, predictor {pred}, compression {comp} not supported"));
    }
    let offs = d.get_tag_u64_vec(Tag::TileOffsets).map_err(e)?;
    let lens = d.get_tag_u64_vec(Tag::TileByteCounts).map_err(e)?;
    let (tw, th) = (tw as usize, th as usize);
    let across = g.width.div_ceil(tw);
    let down = g.height.div_ceil(th);
    if offs.len() != across * down || lens.len() != offs.len() || tw * th > 1 << 24 {
        return Err(format!("{name}: tile table does not match the image"));
    }
    let (r0, r1, c0, c1) = window_range(&g, win);
    if r0 >= r1 || c0 >= c1 {
        return Ok(None);
    }
    let (ow, oh) = (c1 - c0, r1 - r0);
    let mut data = vec![0u8; ow * oh];
    let mut file = File::open(path).map_err(|x| format!("{path}: {x}"))?;
    let mut comp_buf = Vec::new();
    let mut tile = vec![0u8; tw * th];
    for tr in r0 / th..r1.div_ceil(th) {
        for tc in c0 / tw..c1.div_ceil(tw) {
            let idx = tr * across + tc;
            if lens[idx] > (64 << 20) {
                return Err(format!("{name}: tile {idx} too large"));
            }
            comp_buf.resize(lens[idx] as usize, 0);
            file.seek(SeekFrom::Start(offs[idx])).map_err(|x| format!("{name}: {x}"))?;
            file.read_exact(&mut comp_buf).map_err(|x| format!("{name}: {x}"))?;
            if comp == 1 {
                if comp_buf.len() != tile.len() {
                    return Err(format!("{name}: tile {idx} size"));
                }
                tile.copy_from_slice(&comp_buf);
            } else {
                let mut z = flate2::read::ZlibDecoder::new(&comp_buf[..]);
                z.read_exact(&mut tile).map_err(|x| format!("{name}: tile {idx}: {x}"))?;
            }
            for y in 0..th {
                let r = tr * th + y;
                if r < r0 || r >= r1 {
                    continue;
                }
                for x in 0..tw {
                    let c = tc * tw + x;
                    if c < c0 || c >= c1 {
                        continue;
                    }
                    data[(r - r0) * ow + (c - c0)] = tile[y * tw + x];
                }
            }
        }
    }
    let grid = Grid { lat0: g.lat0 - r0 as i64 * g.dlat, lon0: g.lon0 + c0 as i64 * g.dlon, dlat: g.dlat, dlon: g.dlon, width: ow, height: oh };
    Ok(Some(Raster { name, grid, data }))
}

/// SRTM HGT (optionally gzip): big-endian i16, 3601 or 1201 square, edge rows shared with
/// neighbours, -32768 void; the name gives the south-west corner (N49E019).
pub fn read_hgt(path: &str) -> Result<Raster<i32>, String> {
    let name = path.rsplit('/').next().unwrap_or(path).to_string();
    let stem = &name[..7.min(name.len())];
    let b = stem.as_bytes();
    if b.len() < 7 || !(b[0] == b'N' || b[0] == b'S') || !(b[3] == b'E' || b[3] == b'W') {
        return Err(format!("{name}: not an HGT name"));
    }
    let lat: i64 = stem[1..3].parse().map_err(|_| format!("{name}: bad latitude"))?;
    let lon: i64 = stem[4..7].parse().map_err(|_| format!("{name}: bad longitude"))?;
    let lat = if b[0] == b'S' { -lat } else { lat };
    let lon = if b[3] == b'W' { -lon } else { lon };
    let mut raw = Vec::new();
    let f = File::open(path).map_err(|e| format!("{path}: {e}"))?;
    if name.ends_with(".gz") {
        flate2::read::GzDecoder::new(BufReader::new(f)).read_to_end(&mut raw).map_err(|e| format!("{name}: {e}"))?;
    } else {
        BufReader::new(f).read_to_end(&mut raw).map_err(|e| format!("{name}: {e}"))?;
    }
    let n = match raw.len() {
        x if x == 2 * 3601 * 3601 => 3601,
        x if x == 2 * 1201 * 1201 => 1201,
        x => return Err(format!("{name}: {x} bytes is not a 1\" or 3\" HGT")),
    };
    let data = raw
        .chunks_exact(2)
        .map(|p| {
            let v = i16::from_be_bytes([p[0], p[1]]);
            if v == i16::MIN { DEM_NONE } else { v as i32 * 10 }
        })
        .collect();
    let step = TICKS_PER_DEG / (n as i64 - 1);
    Ok(Raster {
        name,
        grid: Grid { lat0: (lat + 1) * TICKS_PER_DEG, lon0: lon * TICKS_PER_DEG, dlat: step, dlon: step, width: n, height: n },
        data,
    })
}

/// Every DEM file in a directory (sorted by name; the mosaic indexes by geotransform).
pub fn read_dem_dir(dir: &str, win: Option<Window>) -> Result<Vec<Raster<i32>>, String> {
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .map_err(|e| format!("{dir}: {e}"))?
        .filter_map(|e| e.ok().map(|e| e.path().to_string_lossy().into_owned()))
        .filter(|p| p.ends_with(".tif") || p.ends_with(".hgt") || p.ends_with(".hgt.gz"))
        .collect();
    names.sort();
    let mut out = Vec::new();
    for p in names {
        if p.ends_with(".tif") {
            if let Some(r) = read_dem_geotiff(&p, win)? {
                out.push(r);
            }
        } else {
            out.push(read_hgt(&p)?);
        }
    }
    Ok(out)
}

pub fn read_class_dir(dir: &str, win: Option<Window>) -> Result<Vec<Raster<u8>>, String> {
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .map_err(|e| format!("{dir}: {e}"))?
        .filter_map(|e| e.ok().map(|e| e.path().to_string_lossy().into_owned()))
        .filter(|p| p.ends_with(".tif"))
        .collect();
    names.sort();
    let mut out = Vec::new();
    for p in names {
        if let Some(r) = read_class_geotiff(&p, win)? {
            out.push(r);
        }
    }
    Ok(out)
}

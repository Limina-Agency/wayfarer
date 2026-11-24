//! Calibrating the walking pace model ([`wayfarer_costing::pace`]) from a
//! team's own GPS tracks.
//!
//! The built-in [`PaceModel::default`] describes OpenStreetMap hikers. A
//! search team, a dog handler or a team carrying kit walks at its own pace, so
//! [`calibrate_pace`] fits a model to the team's GPX tracks, and the team
//! passes that model wherever a pace model is taken. It runs the cleaning of
//! lane TERRX experiment E4:
//!
//! 1. Each track is split at a gap over 120 s or a jump over 200 m. Pieces
//!    under 5 minutes are dropped, and so are pieces that are not walking:
//!    more than 5 % of their 60 s windows over 2.5 m/s, any window over
//!    4 m/s, or a median 60 s speed over 1.8 m/s.
//! 2. Breaks are cut out: a point is stationary when every point within
//!    +-30 s stays within 12 m of it. What is left is moving time.
//! 3. Positions and heights are smoothed over 5 points. The path is cut
//!    every 50 m, and each 50 m segment's time and climb are compared with
//!    the base model's expected time.
//!
//! The fit then
//!
//! - scales the flat speed so the team's total moving time matches (the
//!   team factor);
//! - with `refit_curve`, reshapes the slope curve per 5 % grade bin from
//!   -40 % to +40 %, each bin pulled toward the base curve by
//!   `prior_segments` pseudo-segments, so sparse bins barely move;
//! - with at least `min_tracks_for_spread` pieces of 1 km or more, replaces
//!   the P10/P50/P90 time multipliers with the spread of the team's own walks.
//!   This is narrower than the default's spread, which is between strangers.
//!
//! Heights come from the GPX `<ele>` values unless the caller passes an
//! elevation source (a DEM). GPS heights are noisy. A barometric altimeter
//! or a DEM gives a better curve, and the time factor is robust either way.
//! Trail types are not detected: `class` says what the tracks were walked on,
//! and the calibrated model keeps the base model's ratios between trail types.
//!
//! Everything is IEEE arithmetic plus [`wayfarer_package::math`], so the same
//! tracks give the same model on every platform.

use serde::Serialize;
use wayfarer_costing::pace::{
    PACE_MODEL_VERSION, PACE_SLOPE_STEPS, PaceModel, PacePercentile, PaceSource,
    PaceTimePercentiles, PaceTrailClass,
};
use wayfarer_package::math;

/// One GPS fix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpsPoint {
    pub lat: f64,
    pub lon: f64,
    /// Height above the datum in metres, if the device logged one.
    pub ele_m: Option<f64>,
    /// Unix time, milliseconds.
    pub time_ms: i64,
}

/// One recorded track: the points of a GPX `<trkseg>` in file order.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GpsTrack {
    pub points: Vec<GpsPoint>,
}

/// Why a GPX file could not be read.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum GpxError {
    #[error("no <gpx> element")]
    NotGpx,
    #[error("track point {0}: {1}")]
    Point(usize, String),
}

/// Reads the track segments of a GPX 1.0 or 1.1 file. Points without a time
/// are skipped (they can't give a pace), and so are routes and waypoints.
pub fn parse_gpx(text: &str) -> Result<Vec<GpsTrack>, GpxError> {
    if !text.contains("<gpx") {
        return Err(GpxError::NotGpx);
    }
    let mut tracks = Vec::new();
    let mut index = 0usize;
    for seg in elements(text, "trkseg") {
        let mut track = GpsTrack::default();
        for (attrs, body) in elements_with_attrs(seg, "trkpt") {
            index += 1;
            let lat = attr(attrs, "lat")
                .and_then(|v| v.trim().parse::<f64>().ok())
                .ok_or_else(|| GpxError::Point(index, "missing or bad lat".into()))?;
            let lon = attr(attrs, "lon")
                .and_then(|v| v.trim().parse::<f64>().ok())
                .ok_or_else(|| GpxError::Point(index, "missing or bad lon".into()))?;
            if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
                return Err(GpxError::Point(index, "lat or lon out of range".into()));
            }
            let Some(time_ms) = child_text(body, "time").and_then(parse_rfc3339_ms) else {
                continue;
            };
            let ele_m = child_text(body, "ele")
                .and_then(|v| v.trim().parse::<f64>().ok())
                .filter(|v| v.is_finite());
            track.points.push(GpsPoint {
                lat,
                lon,
                ele_m,
                time_ms,
            });
        }
        if !track.points.is_empty() {
            tracks.push(track);
        }
    }
    Ok(tracks)
}

/// The bodies of `<name ...>...</name>` elements (no nesting of the same name).
fn elements<'a>(text: &'a str, name: &str) -> impl Iterator<Item = &'a str> {
    elements_with_attrs(text, name)
        .into_iter()
        .map(|(_, body)| body)
}

/// `(attributes, body)` of each `<name ...>...</name>` or `<name .../>`.
fn elements_with_attrs<'a>(text: &'a str, name: &str) -> Vec<(&'a str, &'a str)> {
    let open = format!("<{name}");
    let close = format!("</{name}>");
    let mut out = Vec::new();
    let mut rest = text;
    while let Some(start) = rest.find(&open) {
        let after = &rest[start + open.len()..];
        // `<trkpt` must not match `<trkptx`.
        if !after.starts_with([' ', '>', '/', '\t', '\n', '\r']) {
            rest = after;
            continue;
        }
        let Some(tag_end) = after.find('>') else {
            break;
        };
        let attrs = &after[..tag_end];
        if let Some(attrs) = attrs.strip_suffix('/') {
            out.push((attrs, ""));
            rest = &after[tag_end + 1..];
            continue;
        }
        let body_start = &after[tag_end + 1..];
        let Some(end) = body_start.find(&close) else {
            break;
        };
        out.push((attrs, &body_start[..end]));
        rest = &body_start[end + close.len()..];
    }
    out
}

fn attr<'a>(attrs: &'a str, name: &str) -> Option<&'a str> {
    let mut rest = attrs;
    while let Some(pos) = rest.find(name) {
        let before_ok = pos == 0 || rest.as_bytes()[pos - 1].is_ascii_whitespace();
        let after = rest[pos + name.len()..].trim_start();
        if before_ok && let Some(after) = after.strip_prefix('=') {
            let after = after.trim_start();
            let quote = after.chars().next()?;
            if quote == '"' || quote == '\'' {
                let value = &after[1..];
                return value.find(quote).map(|end| &value[..end]);
            }
        }
        rest = &rest[pos + name.len()..];
    }
    None
}

fn child_text<'a>(body: &'a str, name: &str) -> Option<&'a str> {
    elements(body, name).next()
}

/// `YYYY-MM-DDTHH:MM:SS[.fff](Z|+HH:MM|-HH:MM)` to Unix milliseconds.
fn parse_rfc3339_ms(text: &str) -> Option<i64> {
    let s = text.trim();
    let b = s.as_bytes();
    if b.len() < 19 || b[4] != b'-' || b[7] != b'-' || !(b[10] == b'T' || b[10] == b' ') {
        return None;
    }
    let num = |r: std::ops::Range<usize>| s.get(r)?.parse::<i64>().ok();
    let (y, mo, d) = (num(0..4)?, num(5..7)?, num(8..10)?);
    let (h, mi, sec) = (num(11..13)?, num(14..16)?, num(17..19)?);
    if !(1..=12).contains(&mo) || !(1..=31).contains(&d) || h > 23 || mi > 59 || sec > 60 {
        return None;
    }
    let mut i = 19;
    let mut ms = 0i64;
    if b.get(i) == Some(&b'.') {
        i += 1;
        let start = i;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
        }
        let frac = &s[start..i];
        let digits: String = frac.chars().chain("000".chars()).take(3).collect();
        ms = digits.parse().ok()?;
    }
    let offset_min = match b.get(i) {
        None | Some(b'Z') | Some(b'z') => 0,
        Some(&sign @ (b'+' | b'-')) => {
            let oh = num(i + 1..i + 3)?;
            let om = num(i + 4..i + 6)?;
            let o = oh * 60 + om;
            if sign == b'+' { o } else { -o }
        }
        _ => return None,
    };
    // days from civil (proleptic Gregorian)
    let y2 = if mo <= 2 { y - 1 } else { y };
    let era = y2.div_euclid(400);
    let yoe = y2 - era * 400;
    let mp = (mo + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let days = era * 146_097 + doe - 719_468;
    Some(((days * 24 + h) * 60 + mi - offset_min) * 60_000 + sec * 1_000 + ms)
}

/// Options of [`calibrate_pace`].
#[derive(Debug, Clone)]
pub struct PaceCalibrationOptions {
    /// Recorded in the model's [`PaceSource::Team`].
    pub label: String,
    /// What the tracks were walked on.
    pub class: PaceTrailClass,
    /// Reshape the slope curve as well as the flat speed.
    pub refit_curve: bool,
    /// Pseudo-segments of the base curve added to every grade bin.
    pub prior_segments: u32,
    /// Pieces of at least 1 km needed to replace the time percentiles.
    pub min_tracks_for_spread: u32,
}

impl Default for PaceCalibrationOptions {
    fn default() -> Self {
        Self {
            label: String::new(),
            class: PaceTrailClass::Unknown,
            refit_curve: true,
            prior_segments: 100,
            min_tracks_for_spread: 5,
        }
    }
}

/// One 5 % grade bin of the slope refit.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct PaceGradeBin {
    /// Lower and upper grade, percent. The end bins also take everything beyond.
    pub grade_from_pct: i32,
    pub grade_to_pct: i32,
    pub segments: u32,
    /// The team's time over the base model's in this bin, over the team
    /// factor, per mille (above 1,000: slower here than the curve says).
    pub observed_time_permille: u32,
    /// The same after the pull toward the base curve: what was applied.
    pub applied_time_permille: u32,
}

/// What a calibration used and found.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct PaceCalibrationReport {
    pub tracks_in: u32,
    pub pieces: u32,
    pub pieces_used: u32,
    pub pieces_short: u32,
    pub pieces_not_walking: u32,
    pub pieces_no_elevation: u32,
    pub segments: u32,
    pub distance_m: u64,
    pub moving_s: u64,
    pub stopped_s: u64,
    /// The team's moving time over the base model's expected time, per mille.
    pub team_time_permille: u32,
    /// The same over the calibrated model (about 1,000 by construction).
    pub calibrated_time_permille: u32,
    /// Per piece of 1 km or more: its time over the calibrated model's, per mille.
    pub piece_time_permille: Vec<u32>,
    pub spread_from_team: bool,
    pub bins: Vec<PaceGradeBin>,
}

/// Why a calibration failed.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum PaceCalibrationError {
    #[error("base pace model: {0}")]
    BaseModel(String),
    #[error("not enough walking: {segments} segments of 50 m (at least {needed} needed)")]
    NotEnoughData {
        segments: u32,
        needed: u32,
        report: Box<PaceCalibrationReport>,
    },
}

const SEGMENT_M: f64 = 50.0;
const MIN_SEGMENTS: u32 = 20;
const GAP_S: f64 = 120.0;
const JUMP_M: f64 = 200.0;
const MIN_PIECE_S: f64 = 300.0;
const FAST_MPS: f64 = 2.5;
const FAST_SHARE: f64 = 0.05;
const MAX_MPS: f64 = 4.0;
const MEDIAN_MAX_MPS: f64 = 1.8;
const STOP_WINDOW_S: f64 = 60.0;
const STOP_RADIUS_M: f64 = 12.0;
const BIN_PCT: i32 = 5;
const BIN_LIMIT_PCT: i32 = 40;
const SPREAD_PIECE_M: f64 = 1_000.0;
const EARTH_RADIUS_M: f64 = 6_371_008.8;

struct Segment {
    piece: usize,
    rise_mm: i64,
    time_ms: u64,
}

/// Fits a pace model to `tracks` starting from `base`. `elevation`, when
/// given, replaces the tracks' own heights: (lat, lon) to metres, or `None`
/// where it has no data.
pub fn calibrate_pace(
    tracks: &[GpsTrack],
    base: &PaceModel,
    elevation: Option<&dyn Fn(f64, f64) -> Option<f64>>,
    options: &PaceCalibrationOptions,
) -> Result<(PaceModel, PaceCalibrationReport), PaceCalibrationError> {
    base.validate()
        .map_err(|e| PaceCalibrationError::BaseModel(e.to_string()))?;
    let mut report = PaceCalibrationReport {
        tracks_in: tracks.len() as u32,
        pieces: 0,
        pieces_used: 0,
        pieces_short: 0,
        pieces_not_walking: 0,
        pieces_no_elevation: 0,
        segments: 0,
        distance_m: 0,
        moving_s: 0,
        stopped_s: 0,
        team_time_permille: 0,
        calibrated_time_permille: 0,
        piece_time_permille: Vec::new(),
        spread_from_team: false,
        bins: Vec::new(),
    };
    let mut segments = Vec::new();
    let mut moving_s = 0.0;
    let mut stopped_s = 0.0;
    let mut piece_id = 0usize;
    for track in tracks {
        let mut pts: Vec<GpsPoint> = track
            .points
            .iter()
            .copied()
            .filter(|p| p.lat.is_finite() && p.lon.is_finite())
            .collect();
        pts.sort_by_key(|p| p.time_ms);
        pts.dedup_by_key(|p| p.time_ms);
        if pts.len() < 2 {
            continue;
        }
        let (lat0, lon0) = (pts[0].lat, pts[0].lon);
        let ky = EARTH_RADIUS_M * core::f64::consts::PI / 180.0;
        let kx = ky * math::cos(lat0.to_radians());
        let xs: Vec<f64> = pts.iter().map(|p| (p.lon - lon0) * kx).collect();
        let ys: Vec<f64> = pts.iter().map(|p| (p.lat - lat0) * ky).collect();
        let ts: Vec<f64> = pts
            .iter()
            .map(|p| (p.time_ms - pts[0].time_ms) as f64 / 1_000.0)
            .collect();
        let zs: Vec<Option<f64>> = pts
            .iter()
            .map(|p| match elevation {
                Some(f) => f(p.lat, p.lon),
                None => p.ele_m,
            })
            .collect();
        for (a, b) in split_pieces(&xs, &ys, &ts) {
            report.pieces += 1;
            let (x, y, t) = (&xs[a..b], &ys[a..b], &ts[a..b]);
            if t[t.len() - 1] - t[0] < MIN_PIECE_S {
                report.pieces_short += 1;
                continue;
            }
            if !is_walking(x, y, t) {
                report.pieces_not_walking += 1;
                continue;
            }
            let Some(z) = fill_heights(&zs[a..b]) else {
                report.pieces_no_elevation += 1;
                continue;
            };
            report.pieces_used += 1;
            let still = stationary(x, y, t);
            for k in 0..t.len() - 1 {
                if still[k] {
                    stopped_s += t[k + 1] - t[k];
                }
            }
            let mut k = 0;
            while k < t.len() {
                if still[k] {
                    k += 1;
                    continue;
                }
                let start = k;
                while k < t.len() && !still[k] {
                    k += 1;
                }
                if k - start >= 5 {
                    moving_s += t[k - 1] - t[start];
                    cut_segments(
                        &smooth(&x[start..k]),
                        &smooth(&y[start..k]),
                        &smooth(&z[start..k]),
                        &t[start..k],
                        piece_id,
                        &mut segments,
                    );
                }
            }
            piece_id += 1;
        }
    }
    report.segments = segments.len() as u32;
    report.distance_m = (segments.len() as f64 * SEGMENT_M) as u64;
    report.moving_s = moving_s.round() as u64;
    report.stopped_s = stopped_s.round() as u64;
    if report.segments < MIN_SEGMENTS {
        return Err(PaceCalibrationError::NotEnoughData {
            segments: report.segments,
            needed: MIN_SEGMENTS,
            report: Box::new(report),
        });
    }

    let seg_mm = (SEGMENT_M * 1_000.0) as u64;
    let predict = |model: &PaceModel, s: &Segment| {
        model.duration_ms(seg_mm, s.rise_mm, options.class, PacePercentile::Expected) as f64
    };
    let actual: f64 = segments.iter().map(|s| s.time_ms as f64).sum();
    let predicted: f64 = segments.iter().map(|s| predict(base, s)).sum();
    let team = actual / predicted;
    report.team_time_permille = permille(team);

    // Per grade bin: time ratio over the team factor, pulled toward 1.
    let nbins = (2 * BIN_LIMIT_PCT / BIN_PCT) as usize;
    let mut bin_act = vec![0.0; nbins];
    let mut bin_pred = vec![0.0; nbins];
    let mut bin_n = vec![0u32; nbins];
    for s in &segments {
        let grade_pct = s.rise_mm as f64 / (SEGMENT_M * 10.0);
        let b = (((grade_pct + BIN_LIMIT_PCT as f64) / BIN_PCT as f64).floor() as i64)
            .clamp(0, nbins as i64 - 1) as usize;
        bin_act[b] += s.time_ms as f64;
        bin_pred[b] += predict(base, s);
        bin_n[b] += 1;
    }
    let prior = f64::from(options.prior_segments);
    let mut applied = vec![1.0; nbins];
    for b in 0..nbins {
        let observed = if bin_n[b] > 0 {
            bin_act[b] / bin_pred[b] / team
        } else {
            1.0
        };
        let n = f64::from(bin_n[b]);
        if options.refit_curve {
            applied[b] = (n * observed + prior) / (n + prior);
        }
        let from = -BIN_LIMIT_PCT + b as i32 * BIN_PCT;
        report.bins.push(PaceGradeBin {
            grade_from_pct: from,
            grade_to_pct: from + BIN_PCT,
            segments: bin_n[b],
            observed_time_permille: permille(observed),
            applied_time_permille: permille(applied[b]),
        });
    }

    // Speed multiplier per table step: 1 / time ratio, linear between bin
    // centres, flat beyond the outer centres.
    let centre = |b: usize| -BIN_LIMIT_PCT as f64 + (b as f64 + 0.5) * BIN_PCT as f64;
    let speed_mult = |grade_pct: f64| {
        let t = if grade_pct <= centre(0) {
            applied[0]
        } else if grade_pct >= centre(nbins - 1) {
            applied[nbins - 1]
        } else {
            let f = (grade_pct - centre(0)) / BIN_PCT as f64;
            let i = f.floor() as usize;
            let w = f - i as f64;
            applied[i] * (1.0 - w) + applied[i + 1] * w
        };
        1.0 / t
    };
    let m0 = speed_mult(0.0);
    let slope: Vec<u16> = (0..PACE_SLOPE_STEPS)
        .map(|i| {
            let g = i as f64 - 100.0;
            let v = f64::from(base.slope_permille[i]) * speed_mult(g) / m0;
            v.round().clamp(1.0, 5_000.0) as u16
        })
        .collect();
    let flat = (f64::from(base.flat_speed_mm_s) * m0 / team)
        .round()
        .clamp(1.0, 20_000.0) as u32;
    let mut model = PaceModel {
        version: PACE_MODEL_VERSION,
        source: PaceSource::Team {
            label: options.label.clone(),
            tracks: report.pieces_used,
            distance_m: report.distance_m,
            moving_s: report.moving_s,
            curve_refitted: options.refit_curve,
        },
        flat_speed_mm_s: flat,
        slope_permille: slope,
        trail_permille: base.trail_permille,
        time_percentiles_permille: base.time_percentiles_permille,
    };

    // The team's own spread, over the calibrated model.
    let mut piece_act = vec![0.0; piece_id];
    let mut piece_pred = vec![0.0; piece_id];
    let mut piece_n = vec![0u32; piece_id];
    for s in &segments {
        piece_act[s.piece] += s.time_ms as f64;
        piece_pred[s.piece] += predict(&model, s);
        piece_n[s.piece] += 1;
    }
    let total_pred: f64 = piece_pred.iter().sum();
    report.calibrated_time_permille = permille(actual / total_pred);
    let mut ratios: Vec<f64> = (0..piece_id)
        .filter(|&p| f64::from(piece_n[p]) * SEGMENT_M >= SPREAD_PIECE_M)
        .map(|p| piece_act[p] / piece_pred[p])
        .collect();
    report.piece_time_permille = ratios.iter().map(|&r| permille(r)).collect();
    if ratios.len() as u32 >= options.min_tracks_for_spread.max(1) {
        ratios.sort_by(f64::total_cmp);
        let rank =
            |p: f64| ratios[((p * ratios.len() as f64).ceil() as usize).clamp(1, ratios.len()) - 1];
        let (p10, p50, p90) = (
            permille(rank(0.1)),
            permille(rank(0.5)),
            permille(rank(0.9)),
        );
        let clamp = |v: u32| v.clamp(1, 5_000) as u16;
        model.time_percentiles_permille = PaceTimePercentiles {
            p10: clamp(p10),
            p50: clamp(p50.max(p10)),
            p90: clamp(p90.max(p50).max(p10)),
        };
        report.spread_from_team = true;
    }
    model
        .validate()
        .map_err(|e| PaceCalibrationError::BaseModel(format!("calibrated model invalid: {e}")))?;
    Ok((model, report))
}

fn permille(x: f64) -> u32 {
    (x * 1_000.0).round().clamp(0.0, f64::from(u32::MAX)) as u32
}

fn dist(x: &[f64], y: &[f64], i: usize, j: usize) -> f64 {
    let (dx, dy) = (x[j] - x[i], y[j] - y[i]);
    (dx * dx + dy * dy).sqrt()
}

fn split_pieces(x: &[f64], y: &[f64], t: &[f64]) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut start = 0;
    for i in 1..=t.len() {
        let cut = i == t.len() || {
            let dt = t[i] - t[i - 1];
            let dd = dist(x, y, i - 1, i);
            dt > GAP_S || dd > JUMP_M || dd > 15.0 * dt
        };
        if cut {
            if i - start >= 10 {
                out.push((start, i));
            }
            start = i;
        }
    }
    out
}

/// Speed over the next 60 s from each point with at least 30 s of track ahead.
fn window_speeds(x: &[f64], y: &[f64], t: &[f64]) -> Vec<f64> {
    let mut out = Vec::new();
    let mut j = 0;
    for i in 0..t.len() {
        j = j.max(i);
        while j + 1 < t.len() && t[j] < t[i] + STOP_WINDOW_S {
            j += 1;
        }
        let dt = t[j] - t[i];
        if dt >= STOP_WINDOW_S / 2.0 {
            out.push(dist(x, y, i, j) / dt);
        }
    }
    out
}

fn is_walking(x: &[f64], y: &[f64], t: &[f64]) -> bool {
    let mut v = window_speeds(x, y, t);
    if v.is_empty() {
        return false;
    }
    let fast = v.iter().filter(|&&s| s > FAST_MPS).count() as f64 / v.len() as f64;
    if fast > FAST_SHARE || v.iter().any(|&s| s > MAX_MPS) {
        return false;
    }
    v.sort_by(f64::total_cmp);
    let n = v.len();
    let median = if n % 2 == 1 {
        v[n / 2]
    } else {
        (v[n / 2 - 1] + v[n / 2]) / 2.0
    };
    median <= MEDIAN_MAX_MPS
}

fn stationary(x: &[f64], y: &[f64], t: &[f64]) -> Vec<bool> {
    let n = t.len();
    let mut out = vec![false; n];
    let (mut lo, mut hi) = (0usize, 0usize);
    for k in 0..n {
        while t[lo] < t[k] - STOP_WINDOW_S / 2.0 {
            lo += 1;
        }
        while hi + 1 < n && t[hi + 1] <= t[k] + STOP_WINDOW_S / 2.0 {
            hi += 1;
        }
        if t[hi] - t[lo] < STOP_WINDOW_S * 0.8 {
            continue;
        }
        out[k] = (lo..=hi).all(|i| dist(x, y, k, i) < STOP_RADIUS_M);
    }
    out
}

/// Missing heights filled linearly from their neighbours; `None` when more
/// than a fifth are missing.
fn fill_heights(z: &[Option<f64>]) -> Option<Vec<f64>> {
    let known: Vec<usize> = (0..z.len()).filter(|&i| z[i].is_some()).collect();
    if known.is_empty() || (z.len() - known.len()) * 5 > z.len() {
        return None;
    }
    let mut out = vec![0.0; z.len()];
    let mut next = 0usize;
    for (i, value) in out.iter_mut().enumerate() {
        while next < known.len() && known[next] < i {
            next += 1;
        }
        *value = match z[i] {
            Some(v) => v,
            None if next == 0 => z[known[0]].unwrap_or(0.0),
            None if next >= known.len() => z[known[known.len() - 1]].unwrap_or(0.0),
            None => {
                let (a, b) = (known[next - 1], known[next]);
                let (za, zb) = (z[a].unwrap_or(0.0), z[b].unwrap_or(0.0));
                za + (zb - za) * (i - a) as f64 / (b - a) as f64
            }
        };
    }
    Some(out)
}

fn smooth(v: &[f64]) -> Vec<f64> {
    let n = v.len();
    (0..n)
        .map(|i| {
            let (a, b) = (i.saturating_sub(2), (i + 3).min(n));
            v[a..b].iter().sum::<f64>() / (b - a) as f64
        })
        .collect()
}

fn cut_segments(x: &[f64], y: &[f64], z: &[f64], t: &[f64], piece: usize, out: &mut Vec<Segment>) {
    let mut cum = vec![0.0; x.len()];
    for i in 1..x.len() {
        cum[i] = cum[i - 1] + dist(x, y, i - 1, i);
    }
    let at = |d: f64, v: &[f64]| {
        let i = cum.partition_point(|&c| c <= d).clamp(1, cum.len() - 1);
        let (c0, c1) = (cum[i - 1], cum[i]);
        let w = if c1 > c0 { (d - c0) / (c1 - c0) } else { 0.0 };
        v[i - 1] + (v[i] - v[i - 1]) * w
    };
    let total = cum[cum.len() - 1];
    let n = (total / SEGMENT_M).floor() as usize;
    let mut prev = (at(0.0, t), at(0.0, z));
    for s in 1..=n {
        let d = s as f64 * SEGMENT_M;
        let cur = (at(d, t), at(d, z));
        let dt = cur.0 - prev.0;
        if dt > 0.0 {
            let speed = SEGMENT_M / dt;
            if (0.1..3.5).contains(&speed) {
                out.push(Segment {
                    piece,
                    rise_mm: ((cur.1 - prev.1) * 1_000.0).round() as i64,
                    time_ms: (dt * 1_000.0).round() as u64,
                });
            }
        }
        prev = cur;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A synthetic walk: `n` points 3 s apart heading north at `speed` m/s,
    /// climbing `grade` (rise per metre), from `t0`.
    fn walk(n: usize, speed: f64, grade: f64, t0: i64) -> GpsTrack {
        let ky = EARTH_RADIUS_M * core::f64::consts::PI / 180.0;
        GpsTrack {
            points: (0..n)
                .map(|i| {
                    let d = i as f64 * 3.0 * speed;
                    GpsPoint {
                        lat: 49.2 + d / ky,
                        lon: 19.9,
                        ele_m: Some(1_000.0 + d * grade),
                        time_ms: t0 + i as i64 * 3_000,
                    }
                })
                .collect(),
        }
    }

    #[test]
    fn parses_gpx() {
        let gpx = r#"<?xml version="1.0"?>
<gpx version="1.1" creator="x" xmlns="http://www.topografix.com/GPX/1/1">
 <wpt lat="1" lon="2"><name>skip</name></wpt>
 <trk><name>a</name><trkseg>
  <trkpt lat="49.25" lon='20.10'><ele>1200.5</ele><time>2026-01-02T12:00:02Z</time></trkpt>
  <trkpt lon="20.1001" lat="49.2501"><time>2026-01-02T13:00:05.250+01:00</time></trkpt>
  <trkpt lat="49.2502" lon="20.1002"/>
 </trkseg><trkseg>
  <trkpt lat="49.3" lon="20.2"><time>2026-01-02T12:10:00Z</time></trkpt>
 </trkseg></trk></gpx>"#;
        let tracks = parse_gpx(gpx).unwrap();
        assert_eq!(tracks.len(), 2);
        assert_eq!(tracks[0].points.len(), 2);
        let p = tracks[0].points[0];
        assert_eq!((p.lat, p.lon, p.ele_m), (49.25, 20.10, Some(1200.5)));
        assert_eq!(p.time_ms, 1_767_355_202_000);
        assert_eq!(tracks[0].points[1].time_ms, 1_767_355_205_250);
        assert_eq!(tracks[0].points[1].ele_m, None);
        assert_eq!(parse_gpx("<kml/>"), Err(GpxError::NotGpx));
        assert!(
            parse_gpx(r#"<gpx><trkseg><trkpt lat="x" lon="1"></trkpt></trkseg></gpx>"#).is_err()
        );
    }

    #[test]
    fn rfc3339() {
        assert_eq!(parse_rfc3339_ms("1970-01-01T00:00:00Z"), Some(0));
        assert_eq!(
            parse_rfc3339_ms("2000-03-01T00:00:00Z"),
            Some(951_868_800_000)
        );
        assert_eq!(
            parse_rfc3339_ms("2026-11-24T10:00:00-02:30"),
            parse_rfc3339_ms("2026-11-24T12:30:00Z")
        );
        assert_eq!(parse_rfc3339_ms("2026-13-01T00:00:00Z"), None);
        assert_eq!(parse_rfc3339_ms("yesterday"), None);
    }

    /// A team walking 20 % slower than the base model everywhere gets a
    /// 1.25x time factor, and the calibrated model predicts its time.
    #[test]
    fn recovers_a_slower_team() {
        let base = PaceModel::default();
        let class = PaceTrailClass::Path;
        let mut tracks = Vec::new();
        for (k, grade) in [0.0, 0.1, -0.1, 0.2, -0.2, 0.05].into_iter().enumerate() {
            let speed = base.speed_mm_s((grade * 1_000.0) as i64, class) as f64 / 1_000.0 * 0.8;
            tracks.push(walk(1_000, speed, grade, k as i64 * 10_000_000));
        }
        let options = PaceCalibrationOptions {
            class,
            label: "test".into(),
            ..Default::default()
        };
        let (model, report) = calibrate_pace(&tracks, &base, None, &options).unwrap();
        assert_eq!(report.pieces_used, 6);
        assert!(
            (1_230..=1_270).contains(&report.team_time_permille),
            "{report:?}"
        );
        assert!(
            (985..=1_015).contains(&report.calibrated_time_permille),
            "{report:?}"
        );
        assert!(
            (820..=850).contains(&model.flat_speed_mm_s),
            "{}",
            model.flat_speed_mm_s
        );
        assert!(report.spread_from_team);
        model.validate().unwrap();
        // Same input, same model.
        let (again, _) = calibrate_pace(&tracks, &base, None, &options).unwrap();
        assert_eq!(model, again);
        let text = serde_json::to_string(&model).unwrap();
        assert_eq!(PaceModel::from_json(&text).unwrap(), model);
    }

    /// A team that is slow only uphill gets a steeper curve there.
    #[test]
    fn reshapes_the_curve() {
        let base = PaceModel::default();
        let class = PaceTrailClass::Path;
        let mut tracks = Vec::new();
        for (k, grade) in [0.025, 0.225, 0.225, 0.225, -0.225, 0.025]
            .into_iter()
            .enumerate()
        {
            let slow = if grade > 0.1 { 0.6 } else { 1.0 };
            let speed = base.speed_mm_s((grade * 1_000.0) as i64, class) as f64 / 1_000.0 * slow;
            tracks.push(walk(600, speed, grade, k as i64 * 10_000_000));
        }
        let options = PaceCalibrationOptions {
            class,
            prior_segments: 10,
            ..Default::default()
        };
        let (model, _) = calibrate_pace(&tracks, &base, None, &options).unwrap();
        assert!(model.slope_permille[122] < base.slope_permille[122] * 8 / 10);
        assert!(model.slope_permille[78] >= base.slope_permille[78] * 9 / 10);
        let fixed = PaceCalibrationOptions {
            refit_curve: false,
            ..options
        };
        let (scaled, _) = calibrate_pace(&tracks, &base, None, &fixed).unwrap();
        assert_eq!(scaled.slope_permille, base.slope_permille);
    }

    #[test]
    fn drops_breaks_vehicles_and_short_pieces() {
        let base = PaceModel::default();
        // A 10-minute stop in the middle does not count as walking time.
        let mut with_stop = walk(300, 1.2, 0.0, 0);
        let last = *with_stop.points.last().unwrap();
        for i in 1..200 {
            with_stop.points.push(GpsPoint {
                time_ms: last.time_ms + i * 3_000,
                ..last
            });
        }
        let tail = walk(300, 1.2, 0.0, 0);
        let lat_shift = last.lat - tail.points[0].lat;
        let t_shift = last.time_ms + 200 * 3_000;
        with_stop
            .points
            .extend(tail.points.iter().map(|p| GpsPoint {
                lat: p.lat + lat_shift,
                time_ms: p.time_ms + t_shift,
                ..*p
            }));
        let car = walk(300, 12.0, 0.0, 50_000_000);
        let short = walk(50, 1.2, 0.0, 90_000_000);
        let (_, report) = calibrate_pace(
            &[with_stop, car, short],
            &base,
            None,
            &PaceCalibrationOptions::default(),
        )
        .unwrap();
        assert_eq!(report.pieces_not_walking, 1);
        assert_eq!(report.pieces_short, 1);
        assert_eq!(report.pieces_used, 1);
        assert!(report.stopped_s >= 500, "{report:?}");
        assert!(report.moving_s <= 600 * 3 + 60, "{report:?}");
    }

    #[test]
    fn needs_heights_and_data() {
        let base = PaceModel::default();
        let mut track = walk(300, 1.2, 0.0, 0);
        for p in &mut track.points {
            p.ele_m = None;
        }
        let err = calibrate_pace(
            std::slice::from_ref(&track),
            &base,
            None,
            &PaceCalibrationOptions::default(),
        );
        assert!(matches!(
            err,
            Err(PaceCalibrationError::NotEnoughData { .. })
        ));
        // A DEM supplies them.
        let dem = |_lat: f64, _lon: f64| Some(500.0);
        let ok = calibrate_pace(
            &[track],
            &base,
            Some(&dem),
            &PaceCalibrationOptions::default(),
        );
        assert!(ok.is_ok());
    }
}

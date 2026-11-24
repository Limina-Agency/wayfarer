//! Walking pace on terrain: a slope curve, trail-type factors and travel-time
//! percentiles, in integer fixed point.
//!
//! The built-in model ([`PaceModel::default`]) is calibrated on 2,945 km of
//! OpenStreetMap public GPS traces in southern Poland and northern Slovakia
//! (lane TERRX experiment E4, 2026-11-24). It predicted the moving time of 522
//! summer walks on trails with a median 0.98x the actual time and 15 % median
//! error, against 18-20 % for Tobler's hiking function and the Campbell 2022 median.
//! A team replaces it with its own calibration, fitted from its GPS tracks by
//! `wayfarer::pace_calibration`, and passes that model instead.
//!
//! The model gives a moving pace: the time spent walking, including slow
//! stretches and stops under a minute, but not rests. Time is
//!
//! ```text
//! duration = length / (flat_speed * slope(grade) * trail(class)) * percentile
//! ```
//!
//! with `slope` from a 201-entry table in per mille of the flat speed (grade
//! -100 % to +100 % in 1 % steps, linear between steps, clamped outside), a
//! per-mille trail factor and a per-mille time multiplier for the percentile.
//! Every step is integer arithmetic, so results are the same on every target.
//!
//! No route uses the model yet: it waits for terrain heights (format 4). The
//! SAR reach work and the terrain-aware access legs (X3) read it.
//!
//! ```
//! use wayfarer_costing::pace::{PaceModel, PacePercentile, PaceTrailClass};
//!
//! let model = PaceModel::default();
//! // 1 km on a 10 % climb along a marked mountain path, expected time:
//! let ms = model.duration_ms(1_000_000, 100_000, PaceTrailClass::MountainPath, PacePercentile::Expected);
//! assert_eq!(ms, 1_302_202);
//! // The slow end (90th percentile of walkers) takes about 1.32 times as long.
//! let slow = model.duration_ms(1_000_000, 100_000, PaceTrailClass::MountainPath, PacePercentile::P90);
//! assert!(slow > ms * 13 / 10);
//! ```

use serde::{Deserialize, Serialize};

/// Version of the pace model's layout and semantics. A calibration file
/// records it, and [`PaceModel::validate`] refuses another version.
pub const PACE_MODEL_VERSION: u32 = 1;

/// Entries in the slope table: grade -100 % to +100 % in 1 % steps.
pub const PACE_SLOPE_STEPS: usize = 201;

/// The built-in flat moving speed, mm/s (3.75 km/h).
pub const DEFAULT_FLAT_SPEED_MM_S: u32 = 1_041;

/// The built-in slope curve, per mille of the flat speed, for grade -100 %
/// (index 0) to +100 % (index 200).
///
/// Generated once from the E4 expected-pace fit, Campbell's Lorentz form
/// `v = c / (pi b (1 + ((t - a) / b)^2)) + d + e t` (t the slope in degrees,
/// v in m/s) with a = -1.5465052590943267, b = 16.666136335269947,
/// c = 44.01668451933821, d = 0.2079187381950021, e = -0.0009320187126748538:
/// `round(1000 v(atan(grade)) / v(0))`. A test regenerates it. The fit
/// covers slopes to about +-30 degrees (+-58 %); steeper entries extrapolate.
/// Peak 1,008 at -3 % to -2 %.
pub const DEFAULT_SLOPE_PERMILLE: [u16; PACE_SLOPE_STEPS] = [
    343, 344, 345, 346, 347, 348, 350, 351, 352, 353, 354, 356, 357, 358, 360, 361, 363, 365, 366,
    368, 370, 372, 373, 375, 377, 380, 382, 384, 387, 389, 392, 394, 397, 400, 403, 406, 409, 413,
    416, 420, 424, 427, 432, 436, 440, 445, 450, 455, 460, 466, 472, 478, 484, 490, 497, 504, 512,
    520, 528, 536, 545, 554, 564, 574, 584, 595, 607, 618, 631, 643, 657, 670, 684, 699, 714, 729,
    745, 761, 777, 794, 811, 828, 844, 861, 878, 894, 910, 925, 939, 952, 965, 976, 985, 993, 1000,
    1004, 1007, 1008, 1007, 1005, 1000, 994, 985, 976, 964, 952, 938, 923, 907, 890, 873, 856, 838,
    819, 801, 783, 765, 747, 730, 712, 696, 679, 663, 648, 632, 618, 604, 590, 577, 564, 552, 540,
    529, 518, 507, 497, 488, 478, 469, 461, 453, 445, 437, 430, 423, 416, 409, 403, 397, 391, 386,
    380, 375, 370, 365, 361, 356, 352, 348, 344, 340, 336, 333, 329, 326, 322, 319, 316, 313, 310,
    307, 305, 302, 299, 297, 295, 292, 290, 288, 286, 283, 281, 279, 277, 276, 274, 272, 270, 269,
    267, 265, 264, 262, 261, 259, 258, 256, 255, 254, 252, 251,
];

/// The kind of way a stretch is walked on, for its trail factor.
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PaceTrailClass {
    /// Roads and paved ways (`highway` residential, service, tertiary, ...).
    Paved,
    /// `highway=track`, cycleways and bridleways.
    Track,
    /// `footway` or `path` without `sac_scale`, or `sac_scale=hiking`.
    Path,
    /// `sac_scale=mountain_hiking`.
    MountainPath,
    /// `sac_scale=demanding_mountain_hiking` and harder.
    AlpinePath,
    /// `highway=steps`.
    Steps,
    /// Unknown way type: the calibration mix itself (factor 1).
    Unknown,
}

impl PaceTrailClass {
    /// Classifies OSM `highway` and `sac_scale` values. `None` when the way
    /// is not walkable ground the model covers (no `highway`, or a motorway).
    pub fn from_osm(highway: Option<&str>, sac_scale: Option<&str>) -> Option<Self> {
        let highway = highway?;
        Some(match highway {
            "motorway" | "motorway_link" | "trunk" | "trunk_link" | "construction" | "proposed"
            | "raceway" | "bus_guideway" => return None,
            "track" | "cycleway" | "bridleway" => Self::Track,
            "steps" => Self::Steps,
            "footway" | "path" => match sac_scale {
                Some("mountain_hiking") => Self::MountainPath,
                Some(
                    "demanding_mountain_hiking"
                    | "alpine_hiking"
                    | "demanding_alpine_hiking"
                    | "difficult_alpine_hiking",
                ) => Self::AlpinePath,
                _ => Self::Path,
            },
            _ => Self::Paved,
        })
    }
}

/// Trail factors, per mille of the slope curve's speed.
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct PaceTrailFactors {
    pub paved: u16,
    pub track: u16,
    pub path: u16,
    pub mountain_path: u16,
    pub alpine_path: u16,
    pub steps: u16,
}

impl Default for PaceTrailFactors {
    /// E4: median speed on each way type over the median curve at the same
    /// slope (5,286 to 10,828 segments per class; steps 275).
    fn default() -> Self {
        Self {
            paved: 1_083,
            track: 1_092,
            path: 996,
            mountain_path: 845,
            alpine_path: 832,
            steps: 922,
        }
    }
}

impl PaceTrailFactors {
    /// The factor for `class`, per mille.
    pub fn get(&self, class: PaceTrailClass) -> u16 {
        match class {
            PaceTrailClass::Paved => self.paved,
            PaceTrailClass::Track => self.track,
            PaceTrailClass::Path => self.path,
            PaceTrailClass::MountainPath => self.mountain_path,
            PaceTrailClass::AlpinePath => self.alpine_path,
            PaceTrailClass::Steps => self.steps,
            PaceTrailClass::Unknown => 1_000,
        }
    }
}

/// Which travel time to give: the expected time, or a percentile over
/// walkers (P10 the fast end, P90 the slow end).
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PacePercentile {
    /// The model's own time (multiplier 1).
    Expected,
    P10,
    P50,
    P90,
}

/// Travel-time multipliers per percentile, per mille of the expected time.
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct PaceTimePercentiles {
    pub p10: u16,
    pub p50: u16,
    pub p90: u16,
}

impl Default for PaceTimePercentiles {
    /// E4: actual / predicted moving time over 522 walks of at least 1 km.
    fn default() -> Self {
        Self {
            p10: 728,
            p50: 982,
            p90: 1_321,
        }
    }
}

/// Where a model's numbers come from.
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", tag = "kind", deny_unknown_fields)]
pub enum PaceSource {
    /// The built-in E4 calibration.
    Default,
    /// Calibrated from a team's GPS tracks.
    #[serde(rename_all = "camelCase")]
    Team {
        /// A label the caller gave (team name, date).
        label: String,
        /// Track pieces used.
        tracks: u32,
        /// Moving distance used, metres.
        distance_m: u64,
        /// Moving time used, seconds.
        moving_s: u64,
        /// Whether the slope curve was refitted (or only scaled).
        curve_refitted: bool,
    },
}

/// A walking pace model. Serialized as JSON for calibration files.
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct PaceModel {
    /// [`PACE_MODEL_VERSION`].
    pub version: u32,
    pub source: PaceSource,
    /// Moving speed on flat ground on the calibration mix of ways, mm/s.
    pub flat_speed_mm_s: u32,
    /// [`PACE_SLOPE_STEPS`] entries, per mille of the flat speed, for grade
    /// -100 % to +100 % in 1 % steps.
    pub slope_permille: Vec<u16>,
    pub trail_permille: PaceTrailFactors,
    pub time_percentiles_permille: PaceTimePercentiles,
}

impl Default for PaceModel {
    fn default() -> Self {
        Self {
            version: PACE_MODEL_VERSION,
            source: PaceSource::Default,
            flat_speed_mm_s: DEFAULT_FLAT_SPEED_MM_S,
            slope_permille: DEFAULT_SLOPE_PERMILLE.to_vec(),
            trail_permille: PaceTrailFactors::default(),
            time_percentiles_permille: PaceTimePercentiles::default(),
        }
    }
}

/// Why a pace model was refused.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum PaceModelError {
    #[error("pace model version {0} is not supported (expected {PACE_MODEL_VERSION})")]
    Version(u32),
    #[error("slope table has {0} entries (expected {PACE_SLOPE_STEPS})")]
    SlopeTableLength(usize),
    #[error("{0} must be between 1 and {1}")]
    OutOfRange(&'static str, u32),
    #[error("time percentiles must satisfy p10 <= p50 <= p90")]
    PercentileOrder,
}

/// Upper bounds that keep [`PaceModel::duration_ms`] far from overflow and
/// refuse absurd files: 20 m/s flat, 5x per table or factor entry.
const MAX_FLAT_SPEED_MM_S: u32 = 20_000;
const MAX_PERMILLE: u32 = 5_000;

impl PaceModel {
    /// Checks the version and every value's range.
    pub fn validate(&self) -> Result<(), PaceModelError> {
        if self.version != PACE_MODEL_VERSION {
            return Err(PaceModelError::Version(self.version));
        }
        if self.slope_permille.len() != PACE_SLOPE_STEPS {
            return Err(PaceModelError::SlopeTableLength(self.slope_permille.len()));
        }
        let check = |name, value: u32, max: u32| {
            if value == 0 || value > max {
                Err(PaceModelError::OutOfRange(name, max))
            } else {
                Ok(())
            }
        };
        check("flatSpeedMmS", self.flat_speed_mm_s, MAX_FLAT_SPEED_MM_S)?;
        for &value in &self.slope_permille {
            check("slopePermille", u32::from(value), MAX_PERMILLE)?;
        }
        let t = &self.trail_permille;
        for value in [
            t.paved,
            t.track,
            t.path,
            t.mountain_path,
            t.alpine_path,
            t.steps,
        ] {
            check("trailPermille", u32::from(value), MAX_PERMILLE)?;
        }
        let p = &self.time_percentiles_permille;
        for value in [p.p10, p.p50, p.p90] {
            check("timePercentilesPermille", u32::from(value), MAX_PERMILLE)?;
        }
        if !(p.p10 <= p.p50 && p.p50 <= p.p90) {
            return Err(PaceModelError::PercentileOrder);
        }
        Ok(())
    }

    /// Parses and validates a calibration file.
    pub fn from_json(text: &str) -> Result<Self, String> {
        let model: Self = serde_json::from_str(text).map_err(|e| e.to_string())?;
        model.validate().map_err(|e| e.to_string())?;
        Ok(model)
    }

    /// The slope factor at `grade_permille` (rise per 1,000 of horizontal
    /// length), in units of 1/10,000 of the flat speed: linear between the
    /// table's 1 % steps, clamped at +-100 %.
    pub fn slope_factor_e4(&self, grade_permille: i64) -> u32 {
        let g = grade_permille.clamp(-1_000, 1_000) + 1_000;
        let i = (g / 10) as usize;
        let frac = (g % 10) as u32;
        let a = u32::from(self.slope_permille[i]);
        if frac == 0 {
            return a * 10;
        }
        let b = u32::from(self.slope_permille[i + 1]);
        a * (10 - frac) + b * frac
    }

    /// Grade of a stretch in per mille, rounded toward zero; 0 for a zero
    /// length.
    pub fn grade_permille(length_mm: u64, rise_mm: i64) -> i64 {
        if length_mm == 0 {
            return 0;
        }
        let g = i128::from(rise_mm) * 1_000 / i128::from(length_mm);
        g.clamp(-1_000_000, 1_000_000) as i64
    }

    /// The time multiplier for `percentile`, per mille.
    pub fn percentile_permille(&self, percentile: PacePercentile) -> u32 {
        let p = &self.time_percentiles_permille;
        u32::from(match percentile {
            PacePercentile::Expected => 1_000,
            PacePercentile::P10 => p.p10,
            PacePercentile::P50 => p.p50,
            PacePercentile::P90 => p.p90,
        })
    }

    /// Moving speed in mm/s on a stretch of `grade_permille` and `class`,
    /// rounded to nearest.
    pub fn speed_mm_s(&self, grade_permille: i64, class: PaceTrailClass) -> u32 {
        let num = u64::from(self.flat_speed_mm_s)
            * u64::from(self.slope_factor_e4(grade_permille))
            * u64::from(self.trail_permille.get(class));
        ((num + 5_000_000) / 10_000_000) as u32
    }

    /// Walking time in milliseconds over `length_mm` of horizontal length
    /// that rises `rise_mm` (negative going down), rounded to nearest.
    ///
    /// `length_mm * 10^7 * percentile / (flat * slope_e4 * trail)` in u128,
    /// so any length up to `u64::MAX` is exact. A model that failed
    /// [`PaceModel::validate`] may panic on a zero entry.
    pub fn duration_ms(
        &self,
        length_mm: u64,
        rise_mm: i64,
        class: PaceTrailClass,
        percentile: PacePercentile,
    ) -> u64 {
        let grade = Self::grade_permille(length_mm, rise_mm);
        let den = u128::from(self.flat_speed_mm_s)
            * u128::from(self.slope_factor_e4(grade))
            * u128::from(self.trail_permille.get(class));
        let num =
            u128::from(length_mm) * 10_000_000 * u128::from(self.percentile_permille(percentile));
        let ms = (num + den / 2) / den;
        u64::try_from(ms).unwrap_or(u64::MAX)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The checked-in table equals the E4 fit evaluated with `libm`.
    #[test]
    fn default_table_matches_the_fit() {
        let (a, b, c, d, e) = (
            -1.5465052590943267_f64,
            16.666136335269947_f64,
            44.01668451933821_f64,
            0.2079187381950021_f64,
            -0.0009320187126748538_f64,
        );
        let v = |deg: f64| {
            let z = (deg - a) / b;
            c / (core::f64::consts::PI * b * (1.0 + z * z)) + d + e * deg
        };
        let flat = v(0.0);
        for (i, &entry) in DEFAULT_SLOPE_PERMILLE.iter().enumerate() {
            let grade = i as f64 - 100.0;
            let deg = libm::atan2(grade, 100.0).to_degrees();
            let expected = (1_000.0 * v(deg) / flat).round() as i64;
            assert!(
                (i64::from(entry) - expected).abs() <= 1,
                "grade {grade}: table {entry}, fit {expected}"
            );
        }
        assert_eq!(DEFAULT_SLOPE_PERMILLE[100], 1_000);
        assert_eq!((flat * 1_000.0).round() as u32, DEFAULT_FLAT_SPEED_MM_S);
    }

    #[test]
    fn default_model_is_valid_and_round_trips() {
        let model = PaceModel::default();
        model.validate().unwrap();
        let text = serde_json::to_string(&model).unwrap();
        assert_eq!(PaceModel::from_json(&text).unwrap(), model);
    }

    #[test]
    fn interpolates_between_steps_and_clamps() {
        let model = PaceModel::default();
        assert_eq!(model.slope_factor_e4(0), 10_000);
        assert_eq!(model.slope_factor_e4(100), 8_730);
        // halfway between +10 % (873) and +11 % (856)
        assert_eq!(model.slope_factor_e4(105), 8_645);
        assert_eq!(model.slope_factor_e4(-5), (1_000 * 5 + 1_005 * 5));
        assert_eq!(model.slope_factor_e4(5_000), 2_510);
        assert_eq!(model.slope_factor_e4(-5_000), 3_430);
    }

    #[test]
    fn durations() {
        let model = PaceModel::default();
        // 1,041 m flat on the calibration mix: 1,000 s.
        assert_eq!(
            model.duration_ms(
                1_041_000,
                0,
                PaceTrailClass::Unknown,
                PacePercentile::Expected
            ),
            1_000_000
        );
        let up = model.duration_ms(
            1_000_000,
            200_000,
            PaceTrailClass::Path,
            PacePercentile::Expected,
        );
        let down = model.duration_ms(
            1_000_000,
            -200_000,
            PaceTrailClass::Path,
            PacePercentile::Expected,
        );
        let flat = model.duration_ms(1_000_000, 0, PaceTrailClass::Path, PacePercentile::Expected);
        assert!(up > down && down > flat);
        let p10 = model.duration_ms(1_000_000, 0, PaceTrailClass::Path, PacePercentile::P10);
        let p90 = model.duration_ms(1_000_000, 0, PaceTrailClass::Path, PacePercentile::P90);
        assert!(p10 < flat && flat < p90);
        assert_eq!(
            model.duration_ms(0, 0, PaceTrailClass::Path, PacePercentile::P90),
            0
        );
        // No overflow at the extremes.
        let _ = model.duration_ms(
            u64::MAX,
            i64::MIN,
            PaceTrailClass::Steps,
            PacePercentile::P90,
        );
        assert_eq!(model.speed_mm_s(0, PaceTrailClass::Unknown), 1_041);
    }

    #[test]
    fn classifies_osm_ways() {
        use PaceTrailClass as C;
        assert_eq!(C::from_osm(Some("residential"), None), Some(C::Paved));
        assert_eq!(C::from_osm(Some("track"), None), Some(C::Track));
        assert_eq!(C::from_osm(Some("path"), Some("hiking")), Some(C::Path));
        assert_eq!(
            C::from_osm(Some("path"), Some("mountain_hiking")),
            Some(C::MountainPath)
        );
        assert_eq!(
            C::from_osm(Some("footway"), Some("alpine_hiking")),
            Some(C::AlpinePath)
        );
        assert_eq!(C::from_osm(Some("steps"), None), Some(C::Steps));
        assert_eq!(C::from_osm(Some("motorway"), None), None);
        assert_eq!(C::from_osm(None, None), None);
    }

    #[test]
    fn refuses_bad_models() {
        let model = PaceModel {
            version: 2,
            ..PaceModel::default()
        };
        assert_eq!(model.validate(), Err(PaceModelError::Version(2)));
        let mut model = PaceModel::default();
        model.slope_permille.pop();
        assert!(matches!(
            model.validate(),
            Err(PaceModelError::SlopeTableLength(200))
        ));
        let mut model = PaceModel::default();
        model.slope_permille[3] = 0;
        assert!(matches!(
            model.validate(),
            Err(PaceModelError::OutOfRange(..))
        ));
        let mut model = PaceModel::default();
        model.time_percentiles_permille.p10 = 2_000;
        assert_eq!(model.validate(), Err(PaceModelError::PercentileOrder));
        assert!(PaceModel::from_json("{\"version\":1}").is_err());
    }
}

//! The person factor: actual travel time over the engine's model time.

use serde::{Deserialize, Serialize};
use wayfarer::pace::{PaceModel, PacePercentile, PaceTrailClass};
use wayfarer::{MobilityProfile, RoutingEngine};
use wayfarer_costing::CostModel;

use crate::SarError;

/// The 90th percentile of the standard normal distribution.
const Z90: f64 = 1.281_551_565_544_600_4;

/// A person's travel time over the engine's model time, as three percentiles
/// of a log-normal distribution: `p10` for a fast person (10 % are faster),
/// `p50` typical, `p90` slow. The reach of a person with factor `F` is the
/// isochrone at `budget / F`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
#[non_exhaustive]
pub struct PersonFactors {
    pub p10: f64,
    pub p50: f64,
    pub p90: f64,
}

impl PersonFactors {
    /// On today's flat OnFoot pace (4.8 km/h on paths, no slope): E4's
    /// measured ratios of actual moving time to the flat-pace prediction on
    /// 522 summer on-trail tracks in Małopolskie's mountains
    /// (`off-road-sar/exp/E4`, "Wayfarer today"). The default until OnFoot
    /// costs slope.
    pub const FLAT_PACE: Self = Self {
        p10: 1.03,
        p50: 1.42,
        p90: 2.10,
    };

    /// On E4's fitted expected-pace curve (slope-aware): the factors to use
    /// once OnFoot costing applies it (T2).
    pub const E4_CURVE: Self = Self {
        p10: 0.73,
        p50: 0.98,
        p90: 1.32,
    };

    pub fn validate(&self) -> Result<(), SarError> {
        let valid = [self.p10, self.p50, self.p90]
            .iter()
            .all(|value| crate::finite_positive(*value))
            && self.p10 <= self.p50
            && self.p50 <= self.p90
            && self.p10 < self.p90;
        if valid {
            Ok(())
        } else {
            Err(SarError::invalid(
                "person factors must be finite, above 0, with p10 <= p50 <= p90 and p10 < p90",
            ))
        }
    }

    fn mu(&self) -> f64 {
        libm::log(self.p50)
    }

    fn sigma(&self) -> f64 {
        (libm::log(self.p90) - libm::log(self.p10)) / (2.0 * Z90)
    }

    /// The share of people whose factor is at most `factor`.
    pub fn cdf(&self, factor: f64) -> f64 {
        if factor <= 0.0 {
            return 0.0;
        }
        if factor.is_infinite() {
            return 1.0;
        }
        let z = (libm::log(factor) - self.mu()) / (self.sigma() * std::f64::consts::SQRT_2);
        (0.5 * (1.0 + libm::erf(z))).clamp(0.0, 1.0)
    }

    /// The factor below which a share `q` (0 < q < 1) of people fall.
    pub fn quantile(&self, q: f64) -> f64 {
        libm::exp(self.mu() + self.sigma() * inverse_normal(q))
    }

    pub(crate) fn of(&self, person: PersonPercentile) -> f64 {
        match person {
            PersonPercentile::Fast => self.p10,
            PersonPercentile::Typical => self.p50,
            PersonPercentile::Slow => self.p90,
        }
    }
}

impl Default for PersonFactors {
    fn default() -> Self {
        Self::FLAT_PACE
    }
}

/// Which person of [`PersonFactors`] a reach area or arrival is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum PersonPercentile {
    /// `p10`: 10 % of people are faster. The outer reach.
    Fast,
    /// `p50`.
    Typical,
    /// `p90`: 10 % of people are slower.
    Slow,
}

impl PersonPercentile {
    pub(crate) const ALL: [Self; 3] = [Self::Fast, Self::Typical, Self::Slow];
}

/// The standard normal quantile (Acklam's rational approximation, relative
/// error under 1.2e-9), for `0 < p < 1`.
fn inverse_normal(p: f64) -> f64 {
    const A: [f64; 6] = [
        -3.969_683_028_665_376e1,
        2.209_460_984_245_205e2,
        -2.759_285_104_469_687e2,
        1.383_577_518_672_69e2,
        -3.066_479_806_614_716e1,
        2.506_628_277_459_239,
    ];
    const B: [f64; 5] = [
        -5.447_609_879_822_406e1,
        1.615_858_368_580_409e2,
        -1.556_989_798_598_866e2,
        6.680_131_188_771_972e1,
        -1.328_068_155_288_572e1,
    ];
    const C: [f64; 6] = [
        -7.784_894_002_430_293e-3,
        -3.223_964_580_411_365e-1,
        -2.400_758_277_161_838,
        -2.549_732_539_343_734,
        4.374_664_141_464_968,
        2.938_163_982_698_783,
    ];
    const D: [f64; 4] = [
        7.784_695_709_041_462e-3,
        3.224_671_290_700_398e-1,
        2.445_134_137_142_996,
        3.754_408_661_907_416,
    ];
    let p = p.clamp(1e-12, 1.0 - 1e-12);
    let tail = |q: f64| {
        (((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0)
    };
    if p < 0.02425 {
        tail((-2.0 * libm::log(p)).sqrt())
    } else if p > 1.0 - 0.02425 {
        -tail((-2.0 * libm::log(1.0 - p)).sqrt())
    } else {
        let q = p - 0.5;
        let r = q * q;
        (((((A[0] * r + A[1]) * r + A[2]) * r + A[3]) * r + A[4]) * r + A[5]) * q
            / (((((B[0] * r + B[1]) * r + B[2]) * r + B[3]) * r + B[4]) * r + 1.0)
    }
}

/// The walking pace model's time against the engine's on a set of edges:
/// `sum(pace expected time) / sum(engine time)` over the edges the model
/// covers. Heights are not in packages yet, so every stretch is flat (grade
/// 0) and only the flat speed and trail factors apply; the slope curve
/// applies once packages carry heights (format 4).
pub(crate) fn pace_ratio(
    engine: &RoutingEngine,
    pace: &PaceModel,
    profile: MobilityProfile,
    costing: &wayfarer::RouteCostingOptions,
    keys: impl IntoIterator<Item = u64>,
) -> Option<f64> {
    let model = CostModel::new(profile, costing);
    let view = engine.route_graph_view();
    let (mut pace_s, mut engine_s) = (0.0, 0.0);
    for key in keys {
        let Some(record) = view.edge_by_id(key) else {
            continue;
        };
        let info = &record.shared_info;
        let Some(class) =
            PaceTrailClass::from_osm(info.highway.as_deref(), info.sac_scale.as_deref())
        else {
            continue;
        };
        let duration = model.edge_duration_s(&record);
        if !duration.is_finite() || duration <= 0.0 {
            continue;
        }
        let length_mm = (f64::from(record.edge.distance_m).max(0.0) * 1_000.0).round() as u64;
        pace_s += pace.duration_ms(length_mm, 0, class, PacePercentile::Expected) as f64 / 1_000.0;
        engine_s += duration;
    }
    (engine_s > 0.0 && pace_s > 0.0).then(|| pace_s / engine_s)
}

/// The pace model's time against the engine's on an unclassified footpath
/// (the engine's 5.1 km/h): the first guess before the reached network is
/// known.
pub(crate) fn footpath_ratio(pace: &PaceModel) -> f64 {
    const ENGINE_FOOTPATH_MM_S: f64 = 5_100_000.0 / 3_600.0;
    let speed = f64::from(pace.speed_mm_s(0, PaceTrailClass::Path)).max(1.0);
    ENGINE_FOOTPATH_MM_S / speed
}

impl PersonFactors {
    /// Fast (P10), typical (P50) and slow (P90) multipliers of the engine's
    /// time; [`PersonFactors::validate`] checks them.
    pub fn new(p10: f64, p50: f64, p90: f64) -> Self {
        Self { p10, p50, p90 }
    }

    /// The person factors (engine time multipliers) of a pace model whose
    /// time is `ratio` times the engine's: its P10, P50 and P90 over the
    /// expected time, times `ratio`.
    pub(crate) fn from_pace(pace: &PaceModel, ratio: f64) -> Self {
        let percentile =
            |p: PacePercentile| ratio * f64::from(pace.percentile_permille(p)) / 1_000.0;
        Self {
            p10: percentile(PacePercentile::P10),
            p50: percentile(PacePercentile::P50),
            p90: percentile(PacePercentile::P90),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percentiles_round_trip() {
        for factors in [PersonFactors::FLAT_PACE, PersonFactors::E4_CURVE] {
            assert!((factors.cdf(factors.p50) - 0.5).abs() < 1e-9);
            // The fitted sigma puts p10 and p90 symmetric in log space
            // around the median; E4's are close to that.
            let spread = factors.cdf(factors.p90) - factors.cdf(factors.p10);
            assert!((spread - 0.8).abs() < 0.05, "{spread}");
            for q in [0.02, 0.1, 0.5, 0.9, 0.98] {
                assert!((factors.cdf(factors.quantile(q)) - q).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn cdf_edges() {
        let factors = PersonFactors::default();
        assert_eq!(factors.cdf(0.0), 0.0);
        assert_eq!(factors.cdf(f64::INFINITY), 1.0);
        assert!(factors.validate().is_ok());
        let bad = PersonFactors {
            p10: 1.5,
            p50: 1.0,
            p90: 2.0,
        };
        assert!(bad.validate().is_err());
    }
}

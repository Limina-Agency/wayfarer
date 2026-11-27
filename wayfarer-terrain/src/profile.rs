//! Profiles: the tables that turn the fused state into a time per metre per cell.
//!
//! Land-cover codes (4 bits): 0 none, 1 tree, 2 shrub, 3 grass, 4 crop, 5 built, 6 bare,
//! 7 snow, 8 water, 9 wetland, 10 mangrove, 11 moss, 12 dwarf pine, 13 rock, 14 scree,
//! 15 bog or swamp forest. Hazard and level slots are those of [`crate::layer::HAZARDS`] and
//! [`crate::layer::LEVELS`].
//!
//! The values of [`Rules::walker`] and [`Rules::vehicle`] are the experiment values (E4 pace by
//! grade, E5 cover, steepness and barrier factors, E14 hazard, level and obstruction rules).
//! Apart from the pace by grade they are assumptions until the field trial; a surface embeds
//! the full tables it was built with, so changing them changes its content hash, never
//! silently its meaning.

use crate::codec::{Reader, put_varint};
use crate::fuse::S;
use crate::layer::{NHAZ, NLEV};
use crate::{Error, Result};

/// Grade steps of a pace-by-grade table: -100 % to +100 % in 1 % steps.
pub const GRADE_STEPS: usize = 201;

/// The base tables of a profile.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Profile {
    pub name: String,
    /// Microseconds per stored time unit (walker 1 ms/m: 1000; 4×4 0.1 ms/m: 100).
    pub unit_us: u64,
    /// Flat time per metre per land-cover code, in units (0 = impassable).
    pub cover_t: [u32; 16],
    /// Flat time per metre on a way of each kind (line bits 0-7; 0 = not usable).
    pub way_t: [u32; 8],
    /// Speed per mille per steepness-max class (0 = impassable), off ways only.
    pub steep_rel: [u32; 16],
    /// Speed per mille across river, stream, rail, fence, cliff (0 = impassable), off ways only.
    pub barrier_rel: [u32; 5],
    /// Speed per mille by grade, -100 %..+100 % (index grade + 100).
    pub grade_rel: Vec<u32>,
}

/// The hazard, level and obstruction rules of a profile.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Ext {
    /// Class mass (in S) on classes with no time per metre at or above which the cell blocks.
    pub block_at: u32,
    /// Blocking-hazard probability (1/255) at or above which the cell blocks.
    pub haz_block_at: u8,
    /// Bit per hazard slot: the hazard blocks this profile.
    pub haz_block: u8,
    /// Slowdown per hazard slot at probability 1, per mille: time × (1 + h k / 255000).
    pub haz_k: [u32; NHAZ],
    /// Per level slot: block above this value (u16::MAX = never).
    pub lev_block: [u16; NLEV],
    /// Per level slot: slowdown per unit of the level, per mille.
    pub lev_k: [u32; NLEV],
    /// Speed per mille across debris or fallen trees (0 = impassable), also on ways.
    pub obst_rel: u32,
}

/// A complete profile: what a surface embeds and a search reads.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Rules {
    pub p: Profile,
    pub x: Ext,
}

fn t_of(speed_mmps: f64, unit_us: u64) -> u32 {
    if speed_mmps <= 0.0 {
        0
    } else {
        libm::round(1e9 / speed_mmps / unit_us as f64) as u32
    }
}

impl Rules {
    /// Walker: pace by grade from `grade_rel` (per mille, 201 steps), 1.04 m/s on the flat,
    /// trail factors from E4, cover factors from feat-x.md X3 and E3; flood, fire and closure
    /// block at one half, rubble, windthrow and landslide slow; water over 50 cm and snow over
    /// 120 cm block; fallen trees and debris cost 2.5×.
    pub fn walker(grade_rel: &[u32]) -> Result<Rules> {
        if grade_rel.len() != GRADE_STEPS || grade_rel.contains(&0) {
            return Err(Error::invalid(
                "a pace-by-grade table has 201 positive entries",
            ));
        }
        let v0 = 1040.0;
        let f = [
            0.6, 0.6, 0.35, 0.8, 0.7, 0.5, 0.4, 0.5, 0.0, 0.25, 0.25, 0.7, 0.15, 0.4, 0.4, 0.2,
        ];
        let w = [0.0, 1.08, 1.08, 1.09, 1.00, 0.85, 0.83, 0.92];
        Ok(Rules {
            p: Profile {
                name: "walker".into(),
                unit_us: 1000,
                cover_t: f.map(|x| t_of(v0 * x, 1000)),
                way_t: w.map(|x| t_of(v0 * x, 1000)),
                steep_rel: [
                    1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 800, 600, 400, 250, 0, 0, 0, 0,
                ],
                barrier_rel: [0, 600, 500, 500, 0],
                grade_rel: grade_rel.to_vec(),
            },
            x: Ext {
                block_at: S / 2,
                haz_block_at: 128,
                haz_block: 1 << 0 | 1 << 3 | 1 << 7,
                haz_k: [500, 1500, 2000, 0, 800, 1000, 1500, 0],
                lev_block: [50, 120, u16::MAX],
                lev_k: [20, 8, 0],
                obst_rel: 400,
            },
        })
    }

    /// The walker with the engine's built-in pace by grade
    /// (`wayfarer_costing::pace::DEFAULT_SLOPE_PERMILLE`, the E4 fit).
    pub fn walker_default() -> Rules {
        let g: Vec<u32> = wayfarer_costing::pace::DEFAULT_SLOPE_PERMILLE
            .iter()
            .map(|&v| v as u32)
            .collect();
        Rules::walker(&g).expect("the built-in pace table is valid")
    }

    /// Light 4×4: roads and tracks at fixed speeds, open ground slow, forest and wet ground
    /// no-go (go / slow-go / no-go); any ground hazard blocks at one quarter except snow cover
    /// and ice, which slow; water over 30 cm and snow over 25 cm block; debris blocks.
    pub fn vehicle() -> Rules {
        let kmh = |x: f64| t_of(x / 3.6 * 1000.0, 100);
        let f = [
            0.0, 0.0, 3.0, 12.0, 8.0, 5.0, 6.0, 0.0, 0.0, 0.0, 0.0, 8.0, 0.0, 0.0, 0.0, 0.0,
        ];
        let w = [0.0, 60.0, 40.0, 20.0, 8.0, 0.0, 0.0, 0.0];
        let grade_rel = (-100..=100)
            .map(|g: i32| {
                if g.abs() <= 10 {
                    1000
                } else {
                    (1000 - 23 * (g.abs() - 10)).max(300) as u32
                }
            })
            .collect();
        Rules {
            p: Profile {
                name: "vehicle".into(),
                unit_us: 100,
                cover_t: f.map(kmh),
                way_t: w.map(kmh),
                steep_rel: [
                    1000, 1000, 1000, 1000, 700, 400, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                barrier_rel: [0, 300, 0, 0, 0],
                grade_rel,
            },
            x: Ext {
                block_at: S / 4,
                haz_block_at: 64,
                haz_block: 1 << 0 | 1 << 1 | 1 << 2 | 1 << 3 | 1 << 6 | 1 << 7,
                haz_k: [0, 0, 0, 0, 1500, 2000, 0, 0],
                lev_block: [30, 25, u16::MAX],
                lev_k: [20, 30, 0],
                obst_rel: 0,
            },
        }
    }

    /// Checks what the cost rules and the search rely on.
    pub fn check(&self) -> Result<()> {
        let p = &self.p;
        if p.name.is_empty() || p.name.len() > 255 {
            return Err(Error::invalid("a profile name has 1-255 bytes"));
        }
        if p.unit_us == 0 || p.unit_us > 1_000_000 {
            return Err(Error::invalid("unit_us must be 1..=1,000,000"));
        }
        if p.grade_rel.len() != GRADE_STEPS || p.grade_rel.iter().any(|&g| g == 0 || g > 10_000) {
            return Err(Error::invalid("grade_rel has 201 entries in 1..=10,000"));
        }
        if p.cover_t.iter().chain(&p.way_t).any(|&t| t > 65_535) {
            return Err(Error::invalid("times per metre fit in u16"));
        }
        if p.steep_rel.iter().chain(&p.barrier_rel).any(|&r| r > 1000) || self.x.obst_rel > 1000 {
            return Err(Error::invalid("speed factors are at most 1000 per mille"));
        }
        if self.x.block_at == 0 || self.x.block_at > S {
            return Err(Error::invalid("block_at is in 1..=4096"));
        }
        Ok(())
    }

    /// Canonical bytes of the tables ("WIP0"; all integers little-endian). A surface embeds
    /// them; the profile hash is their blake3.
    pub fn bytes(&self) -> Vec<u8> {
        let (p, x) = (&self.p, &self.x);
        let mut o = Vec::new();
        o.extend_from_slice(b"WIP0");
        put_varint(&mut o, p.name.len() as u64);
        o.extend_from_slice(p.name.as_bytes());
        o.extend_from_slice(&p.unit_us.to_le_bytes());
        for v in p
            .cover_t
            .iter()
            .chain(&p.way_t)
            .chain(&p.steep_rel)
            .chain(&p.barrier_rel)
        {
            o.extend_from_slice(&v.to_le_bytes());
        }
        o.extend_from_slice(&(p.grade_rel.len() as u16).to_le_bytes());
        for v in &p.grade_rel {
            o.extend_from_slice(&v.to_le_bytes());
        }
        o.extend_from_slice(&x.block_at.to_le_bytes());
        o.push(x.haz_block_at);
        o.push(x.haz_block);
        for v in &x.haz_k {
            o.extend_from_slice(&v.to_le_bytes());
        }
        for v in &x.lev_block {
            o.extend_from_slice(&v.to_le_bytes());
        }
        for v in &x.lev_k {
            o.extend_from_slice(&v.to_le_bytes());
        }
        o.extend_from_slice(&x.obst_rel.to_le_bytes());
        o
    }

    pub fn from_bytes(b: &[u8]) -> Result<Rules> {
        let mut r = Reader::new(b, 0, "profile tables");
        if r.take(4)? != b"WIP0" {
            return Err(Error::Magic("profile tables"));
        }
        let n = r.len()?;
        let name = String::from_utf8(r.take(n)?.to_vec())
            .map_err(|_| Error::invalid("profile name is not UTF-8"))?;
        let unit_us = r.u64()?;
        fn arr<const K: usize>(r: &mut Reader) -> Result<[u32; K]> {
            let mut a = [0u32; K];
            for v in a.iter_mut() {
                *v = r.u32()?;
            }
            Ok(a)
        }
        let cover_t = arr::<16>(&mut r)?;
        let way_t = arr::<8>(&mut r)?;
        let steep_rel = arr::<16>(&mut r)?;
        let barrier_rel = arr::<5>(&mut r)?;
        let ng = r.u16()? as usize;
        let grade_rel = (0..ng).map(|_| r.u32()).collect::<Result<Vec<_>>>()?;
        let block_at = r.u32()?;
        let (haz_block_at, haz_block) = (r.u8()?, r.u8()?);
        let haz_k = arr::<NHAZ>(&mut r)?;
        let mut lev_block = [0u16; NLEV];
        for v in lev_block.iter_mut() {
            *v = r.u16()?;
        }
        let lev_k = arr::<NLEV>(&mut r)?;
        let obst_rel = r.u32()?;
        r.end()?;
        let rules = Rules {
            p: Profile {
                name,
                unit_us,
                cover_t,
                way_t,
                steep_rel,
                barrier_rel,
                grade_rel,
            },
            x: Ext {
                block_at,
                haz_block_at,
                haz_block,
                haz_k,
                lev_block,
                lev_k,
                obst_rel,
            },
        };
        rules.check()?;
        Ok(rules)
    }

    pub fn hash(&self) -> [u8; 32] {
        *blake3::hash(&self.bytes()).as_bytes()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tables_round_trip() {
        for r in [Rules::walker_default(), Rules::vehicle()] {
            r.check().unwrap();
            let b = r.bytes();
            assert_eq!(Rules::from_bytes(&b).unwrap(), r);
            assert!(Rules::from_bytes(&b[..b.len() - 1]).is_err());
        }
        assert!(Rules::walker(&[1000; 200]).is_err());
    }
}

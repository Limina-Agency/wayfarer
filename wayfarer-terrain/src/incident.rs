//! An open incident on one device: the owned API a host keeps for a mission.
//!
//! [`Incident`] holds the terrain cut to the incident's AOI, the profiles, the layers the device
//! has, the team overlay and the last surface it built. The host adds and removes layers, sets
//! the overlay, and asks for the surface at its own time; the incident re-fuses only the blocks
//! whose inputs changed since the last build. Every value is owned, so a host can keep an
//! incident behind a lock or wrap it for another language without lifetimes.
//!
//! ```
//! use wayfarer_terrain::incident::{Aoi, Incident, Settings};
//! use wayfarer_terrain::lattice::{BlockId, Spacing};
//! use wayfarer_terrain::profile::Rules;
//! use wayfarer_terrain::surface::NO_INCIDENT;
//! use wayfarer_terrain::terrain::Terrain;
//!
//! // Four blocks of flat meadow at 1" near 49.25 N, 20 E.
//! let n = 4 * 4096;
//! let blocks = vec![
//!     BlockId { bi: 2770, bj: 1125 }, BlockId { bi: 2770, bj: 1126 },
//!     BlockId { bi: 2771, bj: 1125 }, BlockId { bi: 2771, bj: 1126 },
//! ];
//! let base = Terrain::new(
//!     Spacing::new(120).unwrap(), blocks,
//!     vec![10_000; n], vec![0; n], vec![4; n], vec![1; n], vec![0; n],
//! ).unwrap();
//!
//! let aoi = Aoi::from_degrees((49.2, 49.3), (20.0, 20.1)).unwrap();
//! let mut inc = Incident::new(
//!     NO_INCIDENT, &base, Some(aoi), vec![Rules::walker_default()], Settings::default(),
//! ).unwrap();
//! let s = inc.surface_at(1_795_510_800).unwrap();
//! assert_eq!(s.blocks.len(), 4);
//! let r = inc.route("walker", (49.250, 20.005), (49.270, 20.030)).unwrap().unwrap();
//! assert!(r.route.conf.cost_us > 0 && r.points.len() > 2);
//! let c = inc.cell(49.26, 20.01).unwrap().unwrap();
//! assert_eq!(c.post.cover, 4);
//! ```

use crate::fuse::{self, HashedLayer, OverlayItem, Params, Prior, canonical_overlay};
use crate::lattice::{BlockId, TICKS_PER_DEG, div_round_i64, ticks_of_deg};
use crate::layer::{HAZARDS, LEVELS};
use crate::profile::Rules;
use crate::search::{Grid, Route, route};
use crate::surface::{Inputs, Post, Surface, epoch_of, post};
use crate::terrain::Terrain;
use crate::{Error, Result};
use std::collections::BTreeMap;
use std::sync::{Arc, OnceLock};

/// An area of interest: a box of positions in ticks, `[lat0, lat1) x [lon0, lon1)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Aoi {
    pub lat0: i64,
    pub lat1: i64,
    pub lon0: i64,
    pub lon1: i64,
}

impl Aoi {
    /// A box from f64 degrees, its corners in any order, each edge rounded onto the lattice by
    /// [`ticks_of_deg`].
    pub fn from_degrees(lat: (f64, f64), lon: (f64, f64)) -> Result<Aoi> {
        let t = |d: f64| ticks_of_deg(d).ok_or_else(|| Error::invalid("AOI corner outside ±180°"));
        let (a, b, c, d) = (t(lat.0)?, t(lat.1)?, t(lon.0)?, t(lon.1)?);
        if a.abs().max(b.abs()) > 90 * TICKS_PER_DEG {
            return Err(Error::invalid("AOI latitude outside ±90°"));
        }
        Ok(Aoi {
            lat0: a.min(b),
            lat1: a.max(b),
            lon0: c.min(d),
            lon1: c.max(d),
        })
    }
}

/// The incident's fusion settings: part of every recipe.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Settings {
    /// Epoch length in seconds (900 = 15 minutes).
    pub epoch_s: i64,
    /// Whether layers under a research-only licence may be fused.
    pub allow_research: bool,
    /// Feature lines below this confidence (1/255) have no effect.
    pub feature_min_conf: u8,
    /// Level values below this confidence (1/255) are not applied.
    pub level_min_w: u8,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            epoch_s: 900,
            allow_research: false,
            feature_min_conf: 128,
            level_min_w: 128,
        }
    }
}

/// A route on the incident's surface, with its points in degrees.
#[derive(Clone, Debug, PartialEq)]
pub struct RouteOut {
    pub route: Route,
    /// The posts the route passes, `(lat, lon)` in degrees, from the start to the end.
    pub points: Vec<(f64, f64)>,
}

/// What the incident knows about one post: the base and surface values of [`Post`], and the
/// fused hazards, levels and effect bits that the surface does not store.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Cell {
    /// Global post (i, j).
    pub i: i64,
    pub j: i64,
    pub post: Post,
    /// Fused hazard probabilities above 0, as (hazard code, probability in 1/255).
    pub hazards: Vec<(u16, u8)>,
    /// Fused levels with a confidence, as (level code, value, confidence in 1/255).
    pub levels: Vec<(u16, u16, u8)>,
    /// The effect bits features and team items set ([`fuse::M_BARRIER`] and the others).
    pub effects: u16,
}

/// The inputs of the last build and what it made.
struct Built {
    surface: Arc<Surface>,
    layers: Vec<Arc<HashedLayer>>,
    overlay: Vec<OverlayItem>,
    grid: OnceLock<Arc<Grid>>,
}

/// An open incident: see the [module docs](self).
pub struct Incident {
    id: [u8; 16],
    terrain: Terrain,
    prior: Prior,
    rules: Vec<Rules>,
    settings: Settings,
    layers: BTreeMap<[u8; 32], Arc<HashedLayer>>,
    overlay: Vec<OverlayItem>,
    built: Option<Built>,
}

impl Incident {
    /// Opens an incident with the host's id ([`crate::surface::NO_INCIDENT`] for none), on
    /// `terrain` cut to `aoi` (the whole terrain when `None`), for 1-255 profiles.
    pub fn new(
        id: [u8; 16],
        terrain: &Terrain,
        aoi: Option<Aoi>,
        rules: Vec<Rules>,
        settings: Settings,
    ) -> Result<Incident> {
        if rules.is_empty() || rules.len() > 255 {
            return Err(Error::invalid("an incident has 1-255 profiles"));
        }
        for r in &rules {
            r.check()?;
        }
        if settings.epoch_s <= 0 || settings.epoch_s > u32::MAX as i64 {
            return Err(Error::invalid("the epoch length is 1 to 2^32 - 1 seconds"));
        }
        let terrain = match aoi {
            Some(a) => terrain.cut(a.lat0, a.lat1, a.lon0, a.lon1)?,
            None => terrain.clone(),
        };
        if terrain.blocks().is_empty() {
            return Err(Error::invalid("the AOI holds no terrain"));
        }
        Ok(Incident {
            id,
            terrain,
            prior: Prior::default(),
            rules,
            settings,
            layers: BTreeMap::new(),
            overlay: Vec::new(),
            built: None,
        })
    }

    /// Replaces the land-cover prior (one-hot by default).
    pub fn set_prior(&mut self, prior: Prior) {
        self.prior = prior;
    }

    /// The terrain cut to the AOI: what surfaces are built on.
    pub fn terrain(&self) -> &Terrain {
        &self.terrain
    }
    pub fn id(&self) -> [u8; 16] {
        self.id
    }
    pub fn rules(&self) -> &[Rules] {
        &self.rules
    }
    pub fn settings(&self) -> Settings {
        self.settings
    }

    /// Reads and checks a layer file and adds it; returns its content hash. Adding a layer the
    /// incident holds changes nothing. Whether fusion applies it, and why not, is in each
    /// surface's header ([`crate::surface::Header::left_out`]).
    pub fn add_layer(&mut self, bytes: &[u8]) -> Result<[u8; 32]> {
        Ok(self.add_hashed(HashedLayer::read(bytes)?))
    }

    /// Adds a layer that is already read and hashed; returns its hash.
    pub fn add_hashed(&mut self, layer: HashedLayer) -> [u8; 32] {
        let h = layer.hash();
        self.layers.entry(h).or_insert_with(|| Arc::new(layer));
        h
    }

    /// Removes a layer by hash; false if the incident did not hold it.
    pub fn remove_layer(&mut self, hash: &[u8; 32]) -> bool {
        self.layers.remove(hash).is_some()
    }

    /// The hashes of the layers the incident holds, sorted.
    pub fn layer_hashes(&self) -> Vec<[u8; 32]> {
        self.layers.keys().copied().collect()
    }

    /// Sets the whole team overlay, in any order: each item is checked, and the set is kept in
    /// canonical order. To remove an item, set the overlay without it.
    pub fn set_overlay(&mut self, items: Vec<OverlayItem>) -> Result<()> {
        for it in &items {
            it.check()?;
        }
        self.overlay = canonical_overlay(&items);
        Ok(())
    }

    /// The overlay in canonical order.
    pub fn overlay(&self) -> &[OverlayItem] {
        &self.overlay
    }

    /// The surface built inputs for the current state: what [`Incident::surface_at`] builds from.
    pub fn inputs(&self) -> Inputs<'_> {
        self.inputs_of(
            self.layers.values().map(|l| &**l).collect(),
            self.overlay.clone(),
        )
    }

    fn inputs_of<'a>(
        &'a self,
        layers: Vec<&'a HashedLayer>,
        overlay: Vec<OverlayItem>,
    ) -> Inputs<'a> {
        Inputs {
            id: self.id,
            terrain: &self.terrain,
            prior: &self.prior,
            layers,
            overlay,
            rules: &self.rules,
            epoch_s: self.settings.epoch_s,
            allow_research: self.settings.allow_research,
            feature_min_conf: self.settings.feature_min_conf,
            level_min_w: self.settings.level_min_w,
        }
    }

    /// The surface for the host's time `now` (UTC seconds), at the start of the epoch that holds
    /// it. The same inputs at the same epoch return the last surface; otherwise the incident
    /// re-fuses only the blocks whose inputs changed since the last build. Equal to a full
    /// build from the same inputs.
    pub fn surface_at(&mut self, now: i64) -> Result<Arc<Surface>> {
        let t = epoch_of(now, self.settings.epoch_s);
        let layers: Vec<Arc<HashedLayer>> = self.layers.values().cloned().collect();
        let same_inputs = |b: &Built| {
            b.surface.h.t == t
                && b.overlay == self.overlay
                && b.layers.len() == layers.len()
                && b.layers
                    .iter()
                    .zip(&layers)
                    .all(|(x, y)| x.hash() == y.hash())
        };
        if let Some(b) = &self.built
            && same_inputs(b)
        {
            return Ok(b.surface.clone());
        }
        let now_in = self.inputs_of(layers.iter().map(|l| &**l).collect(), self.overlay.clone());
        let s = match &self.built {
            Some(b) => {
                let before =
                    self.inputs_of(b.layers.iter().map(|l| &**l).collect(), b.overlay.clone());
                now_in.update(t, &b.surface, &before)?
            }
            None => now_in.surface(t)?,
        };
        let s = Arc::new(s);
        self.built = Some(Built {
            surface: s.clone(),
            layers,
            overlay: self.overlay.clone(),
            grid: OnceLock::new(),
        });
        Ok(s)
    }

    /// The last surface built, if any.
    pub fn surface(&self) -> Option<Arc<Surface>> {
        self.built.as_ref().map(|b| b.surface.clone())
    }

    fn built(&self) -> Result<&Built> {
        self.built
            .as_ref()
            .ok_or_else(|| Error::invalid("no surface yet: call surface_at first"))
    }

    /// The search grid of the last surface, built once per surface and shared: what
    /// [`Incident::route`] and a host's own searches (portals, say) run on. An error when
    /// there is no surface yet.
    pub fn grid(&self) -> Result<Arc<Grid>> {
        let b = self.built()?;
        if let Some(g) = b.grid.get() {
            return Ok(g.clone());
        }
        let g = Arc::new(Grid::new(&self.terrain, &b.surface)?);
        Ok(b.grid.get_or_init(|| g).clone())
    }

    /// The global post (i, j) nearest a position in degrees.
    pub fn post_of_degrees(&self, lat: f64, lon: f64) -> Result<(i64, i64)> {
        let t = |d: f64| ticks_of_deg(d).ok_or_else(|| Error::invalid("position outside ±180°"));
        let s = self.terrain.spacing().ticks();
        Ok((div_round_i64(t(lat)?, s), div_round_i64(t(lon)?, s)))
    }

    /// The least-cost route for the named profile on the last surface, between the posts
    /// nearest two positions in degrees, with its confidence; `None` when the end cannot be
    /// reached. An error when there is no surface yet, the profile is unknown, or a position
    /// lies outside the AOI.
    pub fn route(
        &self,
        profile: &str,
        from: (f64, f64),
        to: (f64, f64),
    ) -> Result<Option<RouteOut>> {
        let g = self.grid()?;
        let p = g
            .profile(profile)
            .ok_or_else(|| Error::invalid(format!("no profile named {profile}")))?;
        let cell = |(lat, lon): (f64, f64)| -> Result<usize> {
            let (i, j) = self.post_of_degrees(lat, lon)?;
            g.cell_of(i, j)
                .ok_or_else(|| Error::invalid("a route end lies outside the AOI"))
        };
        let (a, b) = (cell(from)?, cell(to)?);
        let Some(r) = route(&g, p, a, b) else {
            return Ok(None);
        };
        let s = g.spacing as f64 / TICKS_PER_DEG as f64;
        let deg = |c: usize| {
            let (i, j) = g.post_of(c);
            (i as f64 * s, j as f64 * s)
        };
        let mut points: Vec<(f64, f64)> = r.steps.iter().map(|&(c, _)| deg(c)).collect();
        points.push(deg(b));
        Ok(Some(RouteOut { route: r, points }))
    }

    /// What the incident knows about the post nearest a position in degrees, from the last
    /// surface: `None` outside the AOI. The fused hazards, levels and effect bits come from
    /// fusing that post's block again with the last build's inputs, so they match the surface.
    pub fn cell(&self, lat: f64, lon: f64) -> Result<Option<Cell>> {
        let (i, j) = self.post_of_degrees(lat, lon)?;
        self.cell_at(i, j)
    }

    /// [`Incident::cell`] at global post (i, j).
    pub fn cell_at(&self, i: i64, j: i64) -> Result<Option<Cell>> {
        let b = self.built()?;
        let Some(p) = post(&self.terrain, &b.surface, i, j) else {
            return Ok(None);
        };
        let (id, q): (BlockId, usize) = BlockId::of_post(i, j);
        let layers: Vec<&HashedLayer> = b.layers.iter().map(|l| &**l).collect();
        let prm = Params {
            t: b.surface.h.t,
            allow_research: self.settings.allow_research,
            feature_min_conf: self.settings.feature_min_conf,
            level_min_w: self.settings.level_min_w,
        };
        let (mut fused, _) = fuse::fuse(
            &self.terrain,
            &self.prior,
            &layers,
            &b.overlay,
            &prm,
            Some(&[id]),
        );
        let (_, fb) = fused
            .pop()
            .ok_or_else(|| Error::invalid("the surface block is not in the terrain"))?;
        Ok(Some(Cell {
            i,
            j,
            post: p,
            hazards: HAZARDS
                .iter()
                .zip(fb.haz[q])
                .filter(|(_, v)| *v > 0)
                .map(|(h, v)| (h.0, v))
                .collect(),
            levels: LEVELS
                .iter()
                .zip(fb.lev[q])
                .filter(|(_, (_, w))| *w > 0)
                .map(|(l, (v, w))| (l.0, v, w))
                .collect(),
            effects: fb.mask[q],
        }))
    }
}

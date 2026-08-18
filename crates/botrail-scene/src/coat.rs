//! Spray coating: film thickness accumulated on a target surface, walked
//! over a baked timeline.
//!
//! The mirror of [`crate::carve`] — that one subtracts volume from a
//! voxel grid, this one adds thickness to surface patches — and it makes
//! the same claim about what it is: *presentation and bookkeeping*. The
//! deposition model here is **calibrated geometry, not fluid dynamics**.
//! An applicator carries a footprint profile measured on a reference
//! plane at a known standoff; the integrator projects that profile onto
//! the surface along the spray axis, scales it for range and incidence,
//! and integrates over the time the gun was enabled. No air flow, no
//! electrostatics, no droplet physics — see `design/design-painting.md`
//! §5 for what that forecloses (the electrostatic wrap around edges is
//! the big one).
//!
//! What it buys, and what nothing else in botrail's stack can say: the
//! integration runs on the *baked* trajectory, so the film reflects the
//! speed the robot could actually hold, ramps and joint-limited
//! slowdowns included. Film thickness goes as `flow / (speed x pitch)`,
//! so a stroke that lost speed is a stroke that laid on too much paint.
//!
//! Deterministic by construction: a fixed tessellation of the target in
//! its own frame, a fixed sub-step schedule keyed to the patch size, and
//! midpoint-rule integration. Same input, same film, bit for bit.

use nalgebra::{Isometry3, Point3, Vector3};
use std::f64::consts::FRAC_PI_2;
use thiserror::Error;

use botrail_collide::ObstacleCollider;
use botrail_model::Geometry;

use crate::rollout::SequenceTimeline;
use crate::Scene;

#[derive(Debug, Error)]
pub enum CoatError {
    #[error("unknown obstacle `{0}`")]
    UnknownTarget(String),
    #[error("target `{0}` has no geometry to coat")]
    NoGeometry(String),
    #[error("no signal named `{0}` in this timeline")]
    UnknownGate(String),
    #[error(
        "patch size {patch} m over `{target}` needs {patches} patches (cap {cap}); \
         use a coarser patch"
    )]
    TooFine {
        target: String,
        patch: f64,
        patches: usize,
        cap: usize,
    },
    #[error("patch size must be positive, got {0}")]
    BadPatch(f64),
    #[error("applicator: {0}")]
    BadApplicator(String),
    #[error("{0}")]
    BadBrush(String),
    #[error("unknown applicator `{0}`")]
    UnknownApplicator(String),
    #[error("unknown brush `{0}`")]
    UnknownBrush(String),
    #[error(
        "the program names no brush and no applicator was given: pass `applicator=` \
         to spray_coat, or author the strokes with a brush (`scene.define_brush`)"
    )]
    NoApplicator,
    #[error("could not read the target mesh: {0}")]
    Mesh(String),
    #[error("{0}")]
    Toolpath(#[from] crate::toolpath::ToolpathError),
}

// ------------------------------------------------------------- applicator

/// The footprint an applicator lays on its reference plane, as a *shape*:
/// the integrator normalizes whatever this returns so it integrates to
/// one over the plane, and [`Applicator::flow`] supplies the magnitude.
/// Coordinates are meters on the reference plane, origin on the spray
/// axis, `u` across the fan and `w` along it.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub enum Pattern {
    /// Elliptic dual-beta — the literature's standard fit for a flat-fan
    /// air gun: thick down the middle, thin at the edges, and the extent
    /// along `w` narrows toward the ends of the fan. `beta = 1` is a flat
    /// top hat, larger is more peaked.
    DualBeta {
        /// Full fan width across (`u` extent) [m].
        width: f64,
        /// Full fan height along (`w` extent at mid-fan) [m].
        height: f64,
        beta_across: f64,
        beta_along: f64,
    },
    /// Axisymmetric cone — a rotary bell or a round nozzle. `beta = 1` is
    /// a flat disc, 2 is parabolic.
    Round { diameter: f64, beta: f64 },
    /// A measured static-pattern profile, sampled radially: what a shop
    /// actually has after spraying a coupon for a fixed time at a fixed
    /// distance. `radii` ascending from 0, `weight` the film measured
    /// there (any unit — only the shape survives normalization).
    Measured { radii: Vec<f64>, weight: Vec<f64> },
}

impl Pattern {
    /// Lateral reach on the reference plane [m] — the AABB the integrator
    /// culls with.
    fn radius(&self) -> f64 {
        match self {
            Pattern::DualBeta { width, height, .. } => (width.max(*height)) / 2.0,
            Pattern::Round { diameter, .. } => diameter / 2.0,
            Pattern::Measured { radii, .. } => radii.last().copied().unwrap_or(0.0),
        }
    }

    /// Unnormalized shape at `(u, w)` on the reference plane.
    fn shape(&self, u: f64, w: f64) -> f64 {
        match *self {
            Pattern::DualBeta {
                width,
                height,
                beta_across,
                beta_along,
            } => {
                let a = width / 2.0;
                let b = height / 2.0;
                let x = u / a;
                let across = 1.0 - x * x;
                if across <= 0.0 {
                    return 0.0;
                }
                // The fan's along-extent shrinks toward its ends, which is
                // what makes this *elliptic* dual-beta rather than a
                // separable product — the footprint boundary is an ellipse.
                let half = b * across.sqrt();
                let y = w / half;
                let along = 1.0 - y * y;
                if along <= 0.0 {
                    return 0.0;
                }
                across.powf(beta_across - 1.0) * along.powf(beta_along - 1.0)
            }
            Pattern::Round { diameter, beta } => {
                let r = (u * u + w * w).sqrt() / (diameter / 2.0);
                let s = 1.0 - r * r;
                if s <= 0.0 {
                    return 0.0;
                }
                s.powf(beta - 1.0)
            }
            Pattern::Measured {
                ref radii,
                ref weight,
            } => {
                let r = (u * u + w * w).sqrt();
                match radii.iter().position(|x| *x >= r) {
                    None => 0.0,
                    Some(0) => weight[0],
                    Some(i) => {
                        let (r0, r1) = (radii[i - 1], radii[i]);
                        let t = if r1 > r0 { (r - r0) / (r1 - r0) } else { 0.0 };
                        weight[i - 1] + (weight[i] - weight[i - 1]) * t
                    }
                }
            }
        }
    }

    fn validate(&self) -> Result<(), CoatError> {
        let bad = |m: &str| Err(CoatError::BadApplicator(m.to_string()));
        match self {
            Pattern::DualBeta {
                width,
                height,
                beta_across,
                beta_along,
            } => {
                if !(*width > 0.0 && *height > 0.0) {
                    return bad("fan width and height must be positive");
                }
                if *beta_across < 1.0 || *beta_along < 1.0 {
                    // Below 1 the profile diverges at the footprint edge,
                    // which no measured pattern does and which would make
                    // the normalization integral resolution-dependent.
                    return bad("beta must be >= 1 (1 is a flat top hat)");
                }
            }
            Pattern::Round { diameter, beta } => {
                if !(diameter.is_finite() && *diameter > 0.0) {
                    return bad("pattern diameter must be positive");
                }
                if *beta < 1.0 {
                    return bad("beta must be >= 1 (1 is a flat disc)");
                }
            }
            Pattern::Measured { radii, weight } => {
                if radii.len() < 2 || radii.len() != weight.len() {
                    return bad("a measured profile needs >= 2 aligned radius/weight samples");
                }
                if radii.windows(2).any(|p| p[1] <= p[0]) || radii[0] < 0.0 {
                    return bad("measured radii must be non-negative and strictly ascending");
                }
                if weight.iter().any(|v| *v < 0.0) || weight.iter().all(|v| *v <= 0.0) {
                    return bad("measured weights must be non-negative and not all zero");
                }
            }
        }
        Ok(())
    }
}

/// A process setting a stroke runs with — ABB's *brush*: which applicator,
/// at what fraction of its flow, and how the trigger is timed around the
/// stroke. Named on the scene, referenced from a toolpath's feed moves.
/// Analog process values live here, in the authoring, because signals are
/// bool: the PLC enables the gun, the program picks the brush.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct Brush {
    pub name: String,
    /// A [`Scene::applicator`] name.
    pub applicator: String,
    /// Multiplier on the applicator's flow. `1.0` is the flow it was
    /// calibrated at; a primer at half flow is `0.5`.
    #[serde(default = "one")]
    pub flow: f64,
    /// Seconds the paint starts flowing *before* a stroke with this brush
    /// begins — the programmed lead that compensates the fluid system's
    /// delay, and coats the run-in. Zero is "opens exactly at the
    /// stroke's first point".
    #[serde(default)]
    pub lead: f64,
    /// Seconds it keeps flowing after the stroke ends.
    #[serde(default)]
    pub lag: f64,
}

fn one() -> f64 {
    1.0
}

impl Brush {
    pub fn validate(&self) -> Result<(), CoatError> {
        let bad = |m: String| Err(CoatError::BadBrush(m));
        if self.name.is_empty() {
            return bad("a brush needs a name".into());
        }
        if !(self.flow.is_finite() && self.flow >= 0.0) {
            return bad(format!(
                "brush `{}`: flow must be >= 0, got {}",
                self.name, self.flow
            ));
        }
        if !(self.lead.is_finite() && self.lead >= 0.0 && self.lag.is_finite() && self.lag >= 0.0) {
            return bad(format!(
                "brush `{}`: lead and lag are non-negative seconds, got {} / {}",
                self.name, self.lead, self.lag
            ));
        }
        Ok(())
    }
}

/// A spray applicator: where its footprint was measured, what shape it
/// has, and how much paint it delivers.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct Applicator {
    /// Distance the pattern was measured at [m]. The whole model is a
    /// projection of that plane, so this is the number the authored
    /// standoff should match.
    pub standoff: f64,
    pub pattern: Pattern,
    /// Paint delivered at the nozzle [m^3/s].
    pub flow: f64,
    /// Fraction of `flow` that reaches the reference plane. The rest is
    /// overspray before the paint ever gets to the part.
    pub transfer_efficiency: f64,
    /// Nothing lands past this axial distance [m].
    pub max_range: f64,
}

impl Applicator {
    /// The checks [`Self::prepare`] runs, without the quadrature — for
    /// declaring an applicator on a scene ahead of any bake.
    pub fn validate(&self) -> Result<(), CoatError> {
        let bad = |m: &str| CoatError::BadApplicator(m.to_string());
        if !(self.standoff > 0.0 && self.standoff.is_finite()) {
            return Err(bad("standoff must be positive"));
        }
        if !(self.flow > 0.0 && self.flow.is_finite()) {
            return Err(bad("flow must be positive"));
        }
        if !(self.transfer_efficiency > 0.0 && self.transfer_efficiency <= 1.0) {
            return Err(bad("transfer_efficiency must be in (0, 1]"));
        }
        if self.max_range <= self.standoff {
            return Err(bad("max_range must exceed the reference standoff"));
        }
        self.pattern.validate()
    }

    fn prepare(&self) -> Result<Prepared, CoatError> {
        let bad = |m: &str| CoatError::BadApplicator(m.to_string());
        self.validate()?;

        // Normalize numerically rather than in closed form: the same code
        // then serves the dual-beta (whose closed form needs a gamma
        // function), the round cone, and a measured profile that has no
        // closed form at all. One midpoint quadrature at construction,
        // and mass conservation holds by construction afterwards.
        const N: usize = 512;
        let radius = self.pattern.radius();
        let step = 2.0 * radius / N as f64;
        let cell = step * step;
        let mut sum = 0.0;
        for i in 0..N {
            let u = -radius + (i as f64 + 0.5) * step;
            for j in 0..N {
                let w = -radius + (j as f64 + 0.5) * step;
                sum += self.pattern.shape(u, w);
            }
        }
        let integral = sum * cell;
        if !(integral.is_finite() && integral > 0.0) {
            return Err(bad("the pattern integrates to zero over its own footprint"));
        }
        // A coarser grid than the normalization's, weighted by the shape:
        // one ray per cell that carries paint. Ten across the footprint
        // keeps a stamp's overspray query under a hundred casts.
        const K: usize = 10;
        let cell_k = 2.0 * radius / K as f64;
        let mut rays = Vec::new();
        for i in 0..K {
            let u = -radius + (i as f64 + 0.5) * cell_k;
            for j in 0..K {
                let w = -radius + (j as f64 + 0.5) * cell_k;
                let v = self.pattern.shape(u, w);
                if v > 0.0 {
                    rays.push((u, w, v));
                }
            }
        }
        let total: f64 = rays.iter().map(|r| r.2).sum();
        for r in &mut rays {
            r.2 /= total;
        }
        Ok(Prepared {
            pattern: self.pattern.clone(),
            standoff: self.standoff,
            radius,
            // Deposition volume rate on the reference plane [m^3/s],
            // divided by the shape integral so `rate * shape` is a film
            // growth rate in m/s.
            rate: self.flow * self.transfer_efficiency / integral,
            flow: self.flow,
            landed: self.flow * self.transfer_efficiency,
            max_range: self.max_range,
            // Closer than a fifth of the reference standoff the inverse
            // square blows up and the measurement it came from says
            // nothing. Reported rather than silently skipped.
            min_range: self.standoff * 0.2,
            rays,
        })
    }
}

struct Prepared {
    pattern: Pattern,
    standoff: f64,
    radius: f64,
    rate: f64,
    flow: f64,
    /// Volume rate reaching the reference plane [m^3/s]: `flow *
    /// transfer_efficiency`. The mass the footprint's rays share out.
    landed: f64,
    max_range: f64,
    min_range: f64,
    /// A fixed quadrature of the footprint on the reference plane —
    /// `(u, w, weight)` with the weights summing to one — for asking where
    /// the paint that misses the target goes. Same shape the film uses,
    /// so the two accountings agree.
    rays: Vec<(f64, f64, f64)>,
}

// ---------------------------------------------------------------- options

#[derive(Debug, Clone)]
pub struct CoatOptions {
    /// Target edge length of a surface patch [m]. The film map quantizes
    /// to this, and so does the smallest holiday it can see — keep it
    /// well under the pattern width (a twentieth is a good default).
    pub patch_size: f64,
    /// Timeline sampling period [s]; sub-stepped further so the gun never
    /// travels more than half a patch between stamps.
    pub dt: f64,
    /// Signal that gates the gun. `None` sprays for the whole timeline.
    pub gate: Option<String>,
    /// Acceptable film band [m]; drives `in_spec_ratio` and the thin/thick
    /// split. `None` reports the distribution without judging it.
    pub spec: Option<(f64, f64)>,
    /// Steepest incidence [rad] at which a patch still counts as one the
    /// gun *addressed*. Statistics run over that surface only.
    ///
    /// This is a reporting mask, not a physical cutoff — deposition is
    /// unaffected, so paint is still conserved. It exists because a part
    /// is a solid: spray a panel from above and its rim faces the gun at
    /// a grazing angle the whole time, taking almost nothing. Averaged in,
    /// that one narrow band swamps everything the film map is trying to
    /// say about lap uniformity, while telling you nothing the standoff
    /// check would not say better. Past roughly 60 degrees the coupon the
    /// pattern came from has no opinion anyway. Raise it if a steep face
    /// really is part of the job.
    pub max_incidence: f64,
    /// The job, named by the way it faces: a world direction, and only
    /// patches whose outward normal lies within `facing_tolerance` of it
    /// count as the surface being coated. `None` = every face the gun
    /// addressed.
    ///
    /// `max_incidence` alone leaves the addressed set *path-dependent*: a
    /// panel's rim is out of the mask while the gun runs over the panel,
    /// then swings into it as the gun turns around past the edge, so
    /// lengthening the overtravel quietly changes the denominator of
    /// every statistic. Naming the face is the modeling input that
    /// removes the dependence — "the top", "the outside of the hood" —
    /// which is what a paint engineer means by the surface anyway.
    pub facing: Option<Vector3<f64>>,
    /// Half-angle [rad] of the normal cone around `facing`. Wide enough
    /// for the curvature of the face in question: a hood curving 15
    /// degrees each way wants at least that; a flat top wants next to
    /// nothing. Default 60 degrees, matching `max_incidence`.
    pub facing_tolerance: f64,
    /// Shadow-test each patch against the target's own body. Skipped for
    /// convex primitives, where facing the gun is the whole of visibility.
    pub occlusion: bool,
    /// How the film map is coloured — see [`FilmStyle`].
    pub style: FilmStyle,
    /// The paint's own colour (linear RGB) for the amount ramp: the map
    /// then runs from a light wash of it to the full colour, so a film
    /// building up looks like paint going on. `None` uses the palette's
    /// sequential blue.
    pub paint_color: Option<[f32; 3]>,
    /// What bare, never-sprayed patches wear (linear RGB). `None` takes
    /// the target obstacle's own colour when it has one — the part as it
    /// looks unpainted — else a dark neutral.
    pub substrate: Option<[f32; 3]>,
}

impl Default for CoatOptions {
    fn default() -> Self {
        CoatOptions {
            patch_size: 0.005,
            dt: 0.01,
            gate: None,
            spec: None,
            max_incidence: std::f64::consts::FRAC_PI_3,
            facing: None,
            facing_tolerance: std::f64::consts::FRAC_PI_3,
            occlusion: true,
            style: FilmStyle::Auto,
            paint_color: None,
            substrate: None,
        }
    }
}

/// How a film map is coloured. Two readings of the same numbers, for two
/// questions: *how much paint is there* (a sequential ramp, one hue,
/// light to dark — what a film building up looks like) and *is it on
/// target* (a diverging map over the spec band: neutral on target, blue
/// thin, red thick — the verdict).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FilmStyle {
    /// `Spec` when a spec band was given, else `Amount`.
    #[default]
    Auto,
    Amount,
    Spec,
}

/// The colours a film map was drawn with — what its legend is built from.
#[derive(Debug, Clone, PartialEq)]
pub struct FilmPalette {
    /// The resolved style (`Amount` or `Spec`, never `Auto`).
    pub style: FilmStyle,
    /// The amount ramp, linear RGB, lightest first (unused by `Spec`).
    pub ramp: Vec<[f32; 3]>,
    /// Top of the amount ramp [m]: the spec's high edge when one was
    /// given (so a finished film reaches the full colour and more is
    /// clipped), else the film's maximum.
    pub top: f64,
    /// The bare-substrate colour, linear RGB.
    pub uncoated: [f32; 3],
    pub spec: Option<(f64, f64)>,
}

/// The coated target: a film map as a colored mesh plus the bookkeeping.
#[derive(Debug, Clone)]
pub struct FilmCoat {
    /// The obstacle that was coated.
    pub target: String,
    /// Heatmap of the film, target-local coordinates (place it at
    /// [`FilmCoat::pose`]).
    pub mesh: botrail_mesh::MeshData,
    /// World pose of the target at coat time.
    pub pose: Isometry3<f64>,
    pub patch_size: f64,
    pub patch_count: usize,
    /// Film thickness per patch [m], aligned with `mesh.indices`.
    pub thickness: Vec<f64>,
    /// Whether the gun ever worked over each patch, aligned with
    /// `mesh.indices`. Every statistic below is over this subset.
    pub exposed: Vec<bool>,
    /// Whole tessellated area of the target [m^2], worked or not.
    pub surface_area: f64,
    /// Area the gun worked over [m^2] — the denominator for everything
    /// else here.
    pub total_area: f64,
    /// Area-weighted film statistics [m].
    pub mean: f64,
    pub min: f64,
    pub max: f64,
    pub sigma: f64,
    /// Area that never received any paint [m^2] — holidays.
    pub uncoated_area: f64,
    /// Area under / over `spec` [m^2]; zero when no spec was given.
    pub thin_area: f64,
    pub thick_area: f64,
    /// Fraction of the area inside `spec`; `None` without a spec.
    pub in_spec_ratio: Option<f64>,
    /// Paint delivered while the gun was on [m^3].
    pub sprayed_volume: f64,
    /// Paint that landed on this target [m^3] — anywhere on it, including
    /// the grazing faces `max_incidence` keeps out of the statistics, so
    /// a shade more than `mean * total_area`.
    pub deposited_volume: f64,
    pub gun_on_time: f64,
    /// Time the gun spent closer than the model's validity floor [s] —
    /// nothing was deposited for it, so a nonzero value means the film is
    /// under-reported and the standoff needs looking at.
    pub too_close_time: f64,
    /// The spec band the film was judged against, if any.
    pub spec: Option<(f64, f64)>,
    /// The colours the film map (and every stage of a progressive coat)
    /// was drawn with — see [`film_legend`].
    pub palette: FilmPalette,
    /// Paint that landed on *other* obstacles [m^3], by name — the
    /// overspray, and where a masking leak shows: a fixture that took
    /// paint is a fixture that was not masked. Enabled obstacles only
    /// (display-only ones are not physical). From a ray quadrature of the
    /// footprint, so approximate at the percent level; the target's own
    /// share is the patch integral in `deposited_volume`.
    pub overspray: Vec<(String, f64)>,
    /// Paint that landed nowhere in the scene [m^3]: past every obstacle,
    /// plus the atomization loss (`1 - transfer_efficiency`) that never
    /// reaches any surface. `sprayed - deposited - overspray`.
    pub lost_volume: f64,
    /// Paint sprayed [m^3] and landed on the target [m^3] per brush, in
    /// order of first use; a program without brushes reports one entry
    /// named `""`.
    pub sprayed_by_brush: Vec<(String, f64)>,
    pub deposited_by_brush: Vec<(String, f64)>,
}

impl FilmCoat {
    /// Deposited over sprayed: what fraction of the paint that left the
    /// gun ended up on this target. Below the applicator's nominal
    /// transfer efficiency by whatever missed, overshot, or landed on
    /// something else.
    pub fn effective_transfer_efficiency(&self) -> f64 {
        if self.sprayed_volume > 0.0 {
            self.deposited_volume / self.sprayed_volume
        } else {
            0.0
        }
    }
}

// ----------------------------------------------------------------- patches

/// The tessellated target: one entry per triangle, all in target-local
/// coordinates.
struct Patches {
    vertices: Vec<[f64; 3]>,
    indices: Vec<[u32; 3]>,
    centroid: Vec<Vector3<f64>>,
    normal: Vec<Vector3<f64>>,
    area: Vec<f64>,
    film: Vec<f64>,
    /// Whether the gun ever had this patch in range and facing it — the
    /// surface it worked over. Everything the film reports is per this
    /// mask, because a part's back face is not a holiday.
    exposed: Vec<bool>,
    /// Whether the patch belongs to the named face (`CoatOptions::facing`);
    /// all true when no face was named. Gates `exposed`.
    job: Vec<bool>,
}

/// Patch centroids bucketed into a uniform grid, CSR-style: `starts[c]
/// .. starts[c + 1]` indexes `items`. Deterministic (no hashing) and
/// enough to keep a stamp looking only at the patches under the cone.
struct Buckets {
    origin: Vector3<f64>,
    cell: f64,
    dims: [usize; 3],
    starts: Vec<u32>,
    items: Vec<u32>,
}

impl Buckets {
    fn build(centroids: &[Vector3<f64>], cell: f64) -> Buckets {
        let mut lo = Vector3::repeat(f64::INFINITY);
        let mut hi = Vector3::repeat(f64::NEG_INFINITY);
        for c in centroids {
            lo = lo.inf(c);
            hi = hi.sup(c);
        }
        if centroids.is_empty() {
            lo = Vector3::zeros();
            hi = Vector3::zeros();
        }
        let dims = [0, 1, 2].map(|i| (((hi[i] - lo[i]) / cell).ceil() as usize).max(1));
        let count = dims[0] * dims[1] * dims[2];
        let index = |c: &Vector3<f64>| -> usize {
            let g = [0, 1, 2].map(|i| {
                (((c[i] - lo[i]) / cell).floor() as isize).clamp(0, dims[i] as isize - 1) as usize
            });
            (g[2] * dims[1] + g[1]) * dims[0] + g[0]
        };
        let mut starts = vec![0u32; count + 1];
        for c in centroids {
            starts[index(c) + 1] += 1;
        }
        for i in 0..count {
            starts[i + 1] += starts[i];
        }
        let mut cursor = starts.clone();
        let mut items = vec![0u32; centroids.len()];
        for (i, c) in centroids.iter().enumerate() {
            let cell_index = index(c);
            items[cursor[cell_index] as usize] = i as u32;
            cursor[cell_index] += 1;
        }
        Buckets {
            origin: lo,
            cell,
            dims,
            starts,
            items,
        }
    }

    /// Visits every patch whose centroid falls in a cell overlapping the
    /// world-axis-aligned box `lo..hi`.
    fn for_each_in(&self, lo: &Vector3<f64>, hi: &Vector3<f64>, mut f: impl FnMut(usize)) {
        let range = |i: usize| -> (usize, usize) {
            let a = ((lo[i] - self.origin[i]) / self.cell).floor() as isize;
            let b = ((hi[i] - self.origin[i]) / self.cell).floor() as isize;
            let n = self.dims[i] as isize;
            (a.clamp(0, n - 1) as usize, b.clamp(0, n - 1) as usize)
        };
        if (0..3).any(|i| {
            hi[i] < self.origin[i] || lo[i] > self.origin[i] + self.dims[i] as f64 * self.cell
        }) {
            return;
        }
        let (x0, x1) = range(0);
        let (y0, y1) = range(1);
        let (z0, z1) = range(2);
        for z in z0..=z1 {
            for y in y0..=y1 {
                for x in x0..=x1 {
                    let c = (z * self.dims[1] + y) * self.dims[0] + x;
                    for k in self.starts[c]..self.starts[c + 1] {
                        f(self.items[k as usize] as usize);
                    }
                }
            }
        }
    }
}

/// The target's exact surface for ray probes (see
/// [`ObstacleCollider::exact_surface`]).
fn exact_surface(target: &str, geometry: &Geometry) -> Result<ObstacleCollider, CoatError> {
    ObstacleCollider::exact_surface(geometry)
        .map_err(|e| CoatError::Mesh(format!("`{target}`: {e}")))
}

/// Patch cap: past this the film map stops being something a viewer can
/// load, long before it stops being something the integrator can hold.
const PATCH_CAP: usize = 2_000_000;

/// Target-local vertices and the triangles over them — the raw
/// tessellation, before [`Patches`] derives centroids and normals.
type Tessellation = (Vec<[f64; 3]>, Vec<[u32; 3]>);

fn tessellate(target: &str, geometry: &Geometry, patch: f64) -> Result<Tessellation, CoatError> {
    let divisions = |len: f64| -> usize { ((len / patch).ceil() as usize).max(1) };
    let mut vertices: Vec<[f64; 3]> = Vec::new();
    let mut indices: Vec<[u32; 3]> = Vec::new();

    // A quad grid spanning `origin + s*su + t*tv`, wound so the normal
    // comes out along `su x tv`.
    let grid = |origin: Vector3<f64>,
                su: Vector3<f64>,
                tv: Vector3<f64>,
                ns: usize,
                nt: usize,
                vs: &mut Vec<[f64; 3]>,
                is: &mut Vec<[u32; 3]>| {
        let base = vs.len() as u32;
        for i in 0..=ns {
            for j in 0..=nt {
                let p = origin + su * (i as f64 / ns as f64) + tv * (j as f64 / nt as f64);
                vs.push([p.x, p.y, p.z]);
            }
        }
        let at = |i: usize, j: usize| base + (i * (nt + 1) + j) as u32;
        for i in 0..ns {
            for j in 0..nt {
                is.push([at(i, j), at(i + 1, j), at(i + 1, j + 1)]);
                is.push([at(i, j), at(i + 1, j + 1), at(i, j + 1)]);
            }
        }
    };

    match geometry {
        Geometry::Box { size } => {
            let h = size / 2.0;
            // Six faces, each a regular grid. Winding is outward: the
            // spanning pair is ordered so `su x tv` points out of the box.
            let axes = [
                (
                    Vector3::x(),
                    Vector3::y(),
                    Vector3::z(),
                    h.z,
                    size.x,
                    size.y,
                ),
                (
                    Vector3::y(),
                    Vector3::z(),
                    Vector3::x(),
                    h.x,
                    size.y,
                    size.z,
                ),
                (
                    Vector3::z(),
                    Vector3::x(),
                    Vector3::y(),
                    h.y,
                    size.z,
                    size.x,
                ),
            ];
            for (su, tv, n, off, ls, lt) in axes {
                for sign in [1.0f64, -1.0] {
                    let (su, tv) = if sign > 0.0 { (su, tv) } else { (tv, su) };
                    let (ls, lt) = if sign > 0.0 { (ls, lt) } else { (lt, ls) };
                    let origin = n * (off * sign) - su * (ls / 2.0) - tv * (lt / 2.0);
                    grid(
                        origin,
                        su * ls,
                        tv * lt,
                        divisions(ls),
                        divisions(lt),
                        &mut vertices,
                        &mut indices,
                    );
                }
            }
        }
        Geometry::Cylinder { radius, length } => {
            // botrail cylinders run along +z (URDF convention).
            let (r, l) = (*radius, *length);
            let around = divisions(std::f64::consts::TAU * r).max(8);
            let along = divisions(l);
            let ring = |i: usize| -> (f64, f64) {
                let a = std::f64::consts::TAU * i as f64 / around as f64;
                (a.cos(), a.sin())
            };
            let base = vertices.len() as u32;
            for i in 0..around {
                let (c, s) = ring(i);
                for j in 0..=along {
                    let z = -l / 2.0 + l * j as f64 / along as f64;
                    vertices.push([r * c, r * s, z]);
                }
            }
            let at = |i: usize, j: usize| base + ((i % around) * (along + 1) + j) as u32;
            for i in 0..around {
                for j in 0..along {
                    indices.push([at(i, j), at(i + 1, j), at(i + 1, j + 1)]);
                    indices.push([at(i, j), at(i + 1, j + 1), at(i, j + 1)]);
                }
            }
            // Caps: a fan each, wound outward.
            for (z, sign) in [(l / 2.0, 1.0f64), (-l / 2.0, -1.0)] {
                let hub = vertices.len() as u32;
                vertices.push([0.0, 0.0, z]);
                let rim = vertices.len() as u32;
                let rings = divisions(r);
                for i in 0..around {
                    let (c, s) = ring(i);
                    for k in 1..=rings {
                        let rr = r * k as f64 / rings as f64;
                        vertices.push([rr * c, rr * s, z]);
                    }
                }
                let spoke = |i: usize, k: usize| rim + ((i % around) * rings + (k - 1)) as u32;
                for i in 0..around {
                    let (a, b) = if sign > 0.0 {
                        (spoke(i, 1), spoke(i + 1, 1))
                    } else {
                        (spoke(i + 1, 1), spoke(i, 1))
                    };
                    indices.push([hub, a, b]);
                    for k in 1..rings {
                        let (p0, p1) = (spoke(i, k), spoke(i, k + 1));
                        let (q0, q1) = (spoke(i + 1, k), spoke(i + 1, k + 1));
                        if sign > 0.0 {
                            indices.push([p0, p1, q1]);
                            indices.push([p0, q1, q0]);
                        } else {
                            indices.push([p0, q1, p1]);
                            indices.push([p0, q0, q1]);
                        }
                    }
                }
            }
        }
        Geometry::Sphere { radius } => {
            let r = *radius;
            let around = divisions(std::f64::consts::TAU * r).max(8);
            let down = divisions(std::f64::consts::PI * r).max(4);
            let base = vertices.len() as u32;
            for i in 0..=down {
                let phi = std::f64::consts::PI * i as f64 / down as f64;
                let (sp, cp) = phi.sin_cos();
                for j in 0..around {
                    let th = std::f64::consts::TAU * j as f64 / around as f64;
                    vertices.push([r * sp * th.cos(), r * sp * th.sin(), r * cp]);
                }
            }
            let at = |i: usize, j: usize| base + (i * around + (j % around)) as u32;
            for i in 0..down {
                for j in 0..around {
                    indices.push([at(i, j), at(i, j + 1), at(i + 1, j + 1)]);
                    indices.push([at(i, j), at(i + 1, j + 1), at(i + 1, j)]);
                }
            }
        }
        Geometry::Mesh { path, scale } => {
            let data = botrail_collide::mesh::load_mesh_data(path, scale)
                .map_err(|e| CoatError::Mesh(e.to_string()))?;
            // One global subdivision level rather than per-triangle: every
            // edge midpoint is then shared by both triangles that own it,
            // so the tessellation stays conforming and the film map has no
            // cracks. A file whose triangles vary wildly in size pays for
            // it in patch count, which the cap catches.
            let longest = data
                .indices
                .iter()
                .map(|t| {
                    let p = t.map(|i| Vector3::from(data.vertices[i as usize]));
                    (p[0] - p[1])
                        .norm()
                        .max((p[1] - p[2]).norm())
                        .max((p[2] - p[0]).norm())
                })
                .fold(0.0f64, f64::max);
            let level = if longest <= patch {
                0
            } else {
                (longest / patch).log2().ceil().max(0.0) as usize
            };
            vertices = data.vertices;
            indices = data.indices;
            // Deposition needs outward normals, and a file's winding is
            // whatever its exporter did. A closed body tells: its signed
            // volume (divergence theorem over the triangles) is negative
            // when wound inside out, so flip it. An open surface has no
            // volume to speak of and is taken as authored.
            let signed_volume: f64 = indices
                .iter()
                .map(|t| {
                    let p = t.map(|i| Vector3::from(vertices[i as usize]));
                    p[0].dot(&p[1].cross(&p[2])) / 6.0
                })
                .sum();
            if signed_volume < 0.0 {
                for t in &mut indices {
                    t.swap(1, 2);
                }
            }
            for _ in 0..level {
                if indices.len() * 4 > PATCH_CAP {
                    break;
                }
                let mut midpoints = std::collections::HashMap::new();
                let mut split = Vec::with_capacity(indices.len() * 4);
                for tri in &indices {
                    let mid = |a: u32,
                               b: u32,
                               vs: &mut Vec<[f64; 3]>,
                               m: &mut std::collections::HashMap<(u32, u32), u32>|
                     -> u32 {
                        let key = if a < b { (a, b) } else { (b, a) };
                        *m.entry(key).or_insert_with(|| {
                            let (p, q) = (vs[a as usize], vs[b as usize]);
                            vs.push([
                                (p[0] + q[0]) / 2.0,
                                (p[1] + q[1]) / 2.0,
                                (p[2] + q[2]) / 2.0,
                            ]);
                            (vs.len() - 1) as u32
                        })
                    };
                    let (a, b, c) = (tri[0], tri[1], tri[2]);
                    let ab = mid(a, b, &mut vertices, &mut midpoints);
                    let bc = mid(b, c, &mut vertices, &mut midpoints);
                    let ca = mid(c, a, &mut vertices, &mut midpoints);
                    split.push([a, ab, ca]);
                    split.push([ab, b, bc]);
                    split.push([ca, bc, c]);
                    split.push([ab, bc, ca]);
                }
                indices = split;
            }
        }
    }

    if indices.len() > PATCH_CAP {
        return Err(CoatError::TooFine {
            target: target.to_string(),
            patch,
            patches: indices.len(),
            cap: PATCH_CAP,
        });
    }
    Ok((vertices, indices))
}

impl Patches {
    fn build(vertices: Vec<[f64; 3]>, indices: Vec<[u32; 3]>) -> Patches {
        let n = indices.len();
        let mut centroid = Vec::with_capacity(n);
        let mut normal = Vec::with_capacity(n);
        let mut area = Vec::with_capacity(n);
        for tri in &indices {
            let p = tri.map(|i| Vector3::from(vertices[i as usize]));
            let cross = (p[1] - p[0]).cross(&(p[2] - p[0]));
            let mag = cross.norm();
            centroid.push((p[0] + p[1] + p[2]) / 3.0);
            normal.push(if mag > 0.0 { cross / mag } else { Vector3::z() });
            area.push(mag / 2.0);
        }
        Patches {
            vertices,
            indices,
            centroid,
            normal,
            area,
            film: vec![0.0; n],
            exposed: vec![false; n],
            job: vec![true; n],
        }
    }

    /// Restricts the job to patches whose outward normal lies within
    /// `tolerance` of `facing` (both target-local).
    fn select_facing(&mut self, facing: &Vector3<f64>, tolerance: f64) {
        let Some(dir) = facing.try_normalize(1e-12) else {
            return;
        };
        let min_cos = tolerance.clamp(0.0, std::f64::consts::PI).cos();
        for (job, n) in self.job.iter_mut().zip(&self.normal) {
            *job = n.dot(&dir) >= min_cos;
        }
    }
}

// --------------------------------------------------------------- the walk

/// Coats `target` with the gun carried on `robot`'s `tcp` link along
/// `timeline`. `scene` must be the pre-rollout snapshot the timeline was
/// baked against (same contract as [`crate::carve::carve_stock`]).
///
/// What sprays, and when, comes from the program: a toolpath whose
/// strokes name brushes sprays each with that brush's applicator, flow
/// and trigger timing (declared on the scene); one that names none
/// sprays every feed move with `applicator`, which is then required. In
/// both cases the PLC's enable (`options.gate`) must agree.
///
/// The tool-frame convention matches the toolpath solver's: the TCP's
/// local `+Z` runs from the nozzle tip toward the gun body, so paint
/// travels along **-Z**. A fan's width lies along the TCP's local `+X`.
pub fn spray_coat(
    scene: &Scene,
    timeline: &SequenceTimeline,
    target: &str,
    robot: usize,
    tcp: usize,
    applicator: Option<&Applicator>,
    options: &CoatOptions,
) -> Result<FilmCoat, CoatError> {
    spray_coat_staged(scene, timeline, target, robot, tcp, applicator, options, 1)
        .map(|(film, _)| film)
}

/// One frame of a progressive coat: the film as of `time`, coloured on
/// the *final* film's scale so the stages read as one animation (a
/// half-built film is pale, not full-scale). Snapshots are only taken
/// where the walk deposited something since the previous one, so idle
/// stretches produce no duplicate meshes.
#[derive(Debug, Clone)]
pub struct FilmStage {
    /// Timeline time this state is current from.
    pub time: f64,
    pub mesh: botrail_mesh::MeshData,
    /// Cumulative paint on the target as of this stage [m^3].
    pub deposited_volume: f64,
}

/// [`spray_coat`] plus intermediate snapshots at `stages` equal time
/// boundaries — the raw material of progressive-build-up display (each
/// stage shown for its window via [`crate::carve::staged_timeline`]).
#[allow(clippy::too_many_arguments)]
pub fn spray_coat_staged(
    scene: &Scene,
    timeline: &SequenceTimeline,
    target: &str,
    robot: usize,
    tcp: usize,
    applicator: Option<&Applicator>,
    options: &CoatOptions,
    stages: usize,
) -> Result<(FilmCoat, Vec<FilmStage>), CoatError> {
    if !(options.patch_size.is_finite() && options.patch_size > 0.0) {
        return Err(CoatError::BadPatch(options.patch_size));
    }
    let (obstacle, _) = scene
        .obstacle_with_collider(target)
        .ok_or_else(|| CoatError::UnknownTarget(target.to_string()))?;
    // Shadow tests run against the real surface, not the collision hulls:
    // the hulls of a thin shell would shadow its own outer face.
    let surface = exact_surface(target, &obstacle.geometry)?;
    let collider = &surface;

    let trigger = Trigger::resolve(scene, timeline, robot, options.gate.as_deref(), applicator)?;
    // Everything else physical in the cell: what shadows the part, and
    // what the overspray lands on.
    let fixtures = if options.occlusion {
        Fixture::gather(scene, target, &obstacle.pose)
    } else {
        Vec::new()
    };

    let (vertices, indices) = tessellate(target, &obstacle.geometry, options.patch_size)?;
    if indices.is_empty() {
        return Err(CoatError::NoGeometry(target.to_string()));
    }
    let mut patches = Patches::build(vertices, indices);
    if let Some(facing) = options.facing {
        // Named in world; the patches live in the target's frame.
        let local = obstacle.pose.rotation.inverse() * facing;
        patches.select_facing(&local, options.facing_tolerance);
    }

    // A cone at max range spans this much laterally; sizing cells to a
    // third of it keeps a stamp's cell scan in the tens. Sized for the
    // widest gun in play.
    let far_radius = trigger
        .guns
        .iter()
        .map(|g| g.radius * g.max_range / g.standoff)
        .fold(0.0f64, f64::max);
    let buckets = Buckets::build(
        &patches.centroid,
        (far_radius / 3.0).max(options.patch_size * 2.0),
    );

    // Convex primitives cannot shadow themselves: a patch either faces the
    // gun or it does not, and the facing test already ran. Only a mesh
    // needs the ray.
    let self_shadow = options.occlusion && matches!(obstacle.geometry, Geometry::Mesh { .. });
    let cos_incidence_limit = options.max_incidence.clamp(0.0, FRAC_PI_2).cos().powi(2);

    let to_local = obstacle.pose.inverse();
    let track = &timeline.robots[robot].trajectory;
    let tcp_pose = |q: &[f64]| -> Option<Isometry3<f64>> {
        let poses = scene.fk_for(robot, q).ok()?;
        Some(to_local * poses[tcp])
    };

    let mut tally = Tally::new(&trigger, &fixtures);
    let stages = stages.max(1);
    // Raw snapshots (film arrays and the deposition so far); coloured
    // once the final scale is known.
    let mut raw_stages: Vec<(f64, Vec<f64>, f64)> = Vec::new();
    let mut last_deposited = 0.0;
    let mut t = 0.0;
    let mut prev = tcp_pose(&track.sample(0.0));
    for k in 1..=stages {
        let boundary = timeline.duration * k as f64 / stages as f64;
        let mut change_t: Option<f64> = None;
        while t < boundary - 1e-9 {
            let next_t = (t + options.dt).min(boundary);
            let next = tcp_pose(&track.sample(next_t));
            if let (Some(a), Some(b)) = (&prev, &next) {
                let dist = (b.translation.vector - a.translation.vector).norm();
                let steps = ((dist / (options.patch_size * 0.5)).ceil() as usize).max(1);
                let sub_dt = (next_t - t) / steps as f64;
                for s in 0..steps {
                    // Midpoint rule, not the endpoint sampling the carve
                    // uses: this is an integral, and sampling the ends
                    // would count every interior sub-step twice.
                    let u = (s as f64 + 0.5) / steps as f64;
                    let mid_t = t + sub_dt * (s as f64 + 0.5);
                    let Some(active) = trigger.active(mid_t) else {
                        continue;
                    };
                    let gun = &trigger.guns[active.gun];
                    let tip = a.translation.vector.lerp(&b.translation.vector, u);
                    let rot = a.rotation.slerp(&b.rotation, u);
                    let frame = SprayFrame {
                        tip,
                        dir: -(rot * Vector3::z()),
                        e1: rot * Vector3::x(),
                        e2: rot * Vector3::y(),
                    };
                    let stamped = stamp(
                        &mut patches,
                        &buckets,
                        gun,
                        active.scale,
                        &frame,
                        sub_dt,
                        cos_incidence_limit,
                        self_shadow,
                        options.patch_size,
                        collider,
                        &fixtures,
                    );
                    if stamped.deposited > 0.0 {
                        change_t = Some(next_t);
                    }
                    let missed = overspray(gun, active.scale, &frame, sub_dt, collider, &fixtures);
                    tally.add(&active, gun, sub_dt, stamped, &missed);
                }
            }
            prev = next;
            t = next_t;
        }
        // Snapshot only where paint went on: idle windows extend the
        // previous stage. Stamped with the *last change* time, not the
        // window boundary — the state is identical at both, but this
        // keeps the display exact from the moment it switches (see the
        // carve's identical rule).
        if stages > 1 {
            let deposited: f64 = tally.deposited_by_brush.iter().sum();
            if deposited != last_deposited {
                raw_stages.push((
                    change_t.unwrap_or(boundary),
                    patches.film.clone(),
                    deposited,
                ));
                last_deposited = deposited;
            }
        }
    }

    tally.name_overspray(&fixtures);
    let film = finish(
        target,
        patches,
        obstacle.pose,
        obstacle.color,
        options,
        tally,
        &trigger,
    );
    // Colour every stage on the final film's palette, and share the final
    // film's vertices: only the face colours differ between frames.
    let stage_list = raw_stages
        .into_iter()
        .map(|(time, film_at, deposited)| FilmStage {
            time,
            mesh: botrail_mesh::MeshData {
                vertices: film.mesh.vertices.clone(),
                indices: film.mesh.indices.clone(),
                face_colors: film_colors(&film_at, &film.palette),
            },
            deposited_volume: deposited,
        })
        .collect();
    Ok((film, stage_list))
}

/// The gun's frame at one stamp, target-local: tip, spray direction, and
/// the reference plane's axes.
struct SprayFrame {
    tip: Vector3<f64>,
    dir: Vector3<f64>,
    e1: Vector3<f64>,
    e2: Vector3<f64>,
}

/// What one stamp did: film volume laid on the target, and whether the
/// gun was inside the model's validity floor.
struct Stamped {
    deposited: f64,
    too_close: bool,
}

/// The walk's running accounts.
struct Tally {
    on_time: f64,
    too_close_time: f64,
    sprayed: f64,
    /// Per brush index (see [`Trigger::brush_names`]).
    sprayed_by_brush: Vec<f64>,
    deposited_by_brush: Vec<f64>,
    /// Per fixture index.
    overspray: Vec<f64>,
    /// `overspray` by fixture name, fixtures that took none dropped —
    /// filled by [`Self::name_overspray`] once the walk is done.
    overspray_named: Vec<(String, f64)>,
    overspray_total: f64,
}

impl Tally {
    fn new(trigger: &Trigger, fixtures: &[Fixture]) -> Tally {
        Tally {
            on_time: 0.0,
            too_close_time: 0.0,
            sprayed: 0.0,
            sprayed_by_brush: vec![0.0; trigger.brush_names.len()],
            deposited_by_brush: vec![0.0; trigger.brush_names.len()],
            overspray: vec![0.0; fixtures.len()],
            overspray_named: Vec::new(),
            overspray_total: 0.0,
        }
    }

    fn name_overspray(&mut self, fixtures: &[Fixture]) {
        self.overspray_total = self.overspray.iter().sum();
        self.overspray_named = fixtures
            .iter()
            .zip(&self.overspray)
            .filter(|(_, v)| **v > 0.0)
            .map(|(f, v)| (f.name.clone(), *v))
            .collect();
    }

    fn add(&mut self, active: &Active, gun: &Prepared, dt: f64, stamped: Stamped, missed: &[f64]) {
        self.on_time += dt;
        if stamped.too_close {
            self.too_close_time += dt;
        }
        let sprayed = gun.flow * active.scale * dt;
        self.sprayed += sprayed;
        self.sprayed_by_brush[active.brush] += sprayed;
        self.deposited_by_brush[active.brush] += stamped.deposited;
        for (acc, m) in self.overspray.iter_mut().zip(missed) {
            *acc += m;
        }
    }
}

/// Deposits one sub-step's worth of paint.
#[allow(clippy::too_many_arguments)]
fn stamp(
    patches: &mut Patches,
    buckets: &Buckets,
    gun: &Prepared,
    scale_flow: f64,
    frame: &SprayFrame,
    dt: f64,
    // `cos(max_incidence)^2` — the reporting mask's threshold, compared
    // squared so the inner loop needs no square root.
    cos_incidence_limit: f64,
    self_shadow: bool,
    // How far short of a patch's centroid a shadow ray stops [m].
    shadow_margin: f64,
    collider: &ObstacleCollider,
    fixtures: &[Fixture],
) -> Stamped {
    let (tip, dir, e1, e2) = (&frame.tip, &frame.dir, &frame.e1, &frame.e2);
    let far = tip + dir * gun.max_range;
    let lo = tip.inf(&far) - Vector3::repeat(far_pad(gun));
    let hi = tip.sup(&far) + Vector3::repeat(far_pad(gun));
    let mut too_close = false;
    let mut deposited = 0.0;
    // Split the borrow: the closure needs the accumulator mutably while
    // the geometry stays shared.
    let (centroid, normal, area, film, exposed, job) = (
        &patches.centroid,
        &patches.normal,
        &patches.area,
        &mut patches.film,
        &mut patches.exposed,
        &patches.job,
    );
    buckets.for_each_in(&lo, &hi, |i| {
        let v = centroid[i] - tip;
        let r = v.dot(dir);
        if r > gun.max_range {
            return;
        }
        if r < gun.min_range {
            // Only counts as "too close" if the patch is in front of the
            // gun at all — patches behind it are simply not being sprayed.
            if r > 0.0 {
                too_close = true;
            }
            return;
        }
        // Obliquity, relative to the plane the pattern was measured on.
        //
        // A ray tube carrying `dQ` cuts area `dA` out of the surface. Its
        // cross-section on a plane perpendicular to the spray axis (which
        // is what the reference measurement and the `scale^2` term above
        // are expressed in) is `dA_perp`. Projecting both along the ray
        // direction `v_hat` gives `dA*|n.v_hat| = dA_perp*|axis.v_hat|`,
        // so the film rate picks up `|n.v_hat| / cos(off-axis angle)`.
        // With `cos(off-axis) = r/|v|` the norms cancel and the whole
        // factor is just `-n.v / r` — which is exactly 1 for a surface
        // square on to the gun, as it must be: there the target plane
        // *is* the reference plane. (Using `-n.v/|v|` alone would be
        // the same obliquity counted twice, and quietly loses a couple
        // of percent of the paint off-axis.)
        let obliquity = -normal[i].dot(&v) / r;
        if obliquity <= 0.0 {
            return;
        }
        // A shadow ray runs tip -> centroid and stops a patch short of it:
        // the surface is exact, so the patch's own triangle sits at toi = 1
        // to within rounding, and a ray allowed to reach it would shadow
        // every patch with itself. Anything the shortened ray still meets
        // is genuinely in front.
        let stop = 1.0 - (shadow_margin / v.norm()).min(0.5);
        // A fixture in the way — a mask, a clamp — hides the patch from
        // this stamp entirely: it neither takes paint nor counts as
        // addressed. (Bounds first; the ray is rare.)
        if fixtures.iter().any(|f| f.blocks(tip, &v, stop)) {
            return;
        }
        // In range and squarely enough addressed: the gun worked over this
        // patch, whether or not its pattern reached this far out. That is
        // what makes a gap between two laps a holiday rather than
        // something off-target. Compared squared to keep the norm out of
        // the inner loop: `cos(incidence) = -n.v/|v|`, and both sides are
        // known non-negative here.
        let facing = -normal[i].dot(&v);
        if job[i] && facing * facing >= cos_incidence_limit * v.norm_squared() {
            exposed[i] = true;
        }
        // Pull the hit point back to the reference plane: the footprint
        // grows as r/standoff, so the coordinates shrink by its inverse
        // and the intensity by its square. That pairing is what conserves
        // paint across range.
        let scale = gun.standoff / r;
        let lateral = v - dir * r;
        let s = gun
            .pattern
            .shape(lateral.dot(e1) * scale, lateral.dot(e2) * scale);
        if s <= 0.0 {
            return;
        }
        if self_shadow
            && collider
                .cast_local_ray(&Point3::from(*tip), &v, stop)
                .is_some()
        {
            return;
        }
        let dfilm = gun.rate * scale_flow * s * scale * scale * obliquity * dt;
        film[i] += dfilm;
        deposited += dfilm * area[i];
    });
    Stamped {
        deposited,
        too_close,
    }
}

/// Where the paint that is *not* the target's goes, for one sub-step: the
/// footprint's rays cast into the cell, each carrying its share of the
/// deposition rate, attributed to the first fixture it meets. Rays that
/// meet the target first are its (already in the patch integral); rays
/// that meet nothing are lost. Returns paint per fixture [m^3].
fn overspray(
    gun: &Prepared,
    scale_flow: f64,
    frame: &SprayFrame,
    dt: f64,
    target: &ObstacleCollider,
    fixtures: &[Fixture],
) -> Vec<f64> {
    let mut out = vec![0.0; fixtures.len()];
    if fixtures.is_empty() {
        return out;
    }
    let mass = gun.landed * scale_flow * dt;
    let origin = Point3::from(frame.tip);
    for &(u, w, weight) in &gun.rays {
        // The ray through this reference-plane sample, in units of the
        // standoff: `toi = 1` is the reference plane, `max_range /
        // standoff` the far limit along the axis.
        let d = frame.dir * gun.standoff + frame.e1 * u + frame.e2 * w;
        let max_toi = gun.max_range / gun.standoff;
        let on_target = target.cast_local_ray(&origin, &d, max_toi);
        let mut best: Option<(usize, f64)> = None;
        for (k, f) in fixtures.iter().enumerate() {
            if let Some(toi) = f.hit(&frame.tip, &d, max_toi) {
                if best.is_none_or(|(_, b)| toi < b) {
                    best = Some((k, toi));
                }
            }
        }
        if let Some((k, toi)) = best {
            if on_target.is_none_or(|t| toi < t) {
                out[k] += mass * weight;
            }
        }
    }
    out
}

fn far_pad(gun: &Prepared) -> f64 {
    gun.radius * gun.max_range / gun.standoff
}

#[allow(clippy::too_many_arguments)]
fn finish(
    target: &str,
    patches: Patches,
    pose: Isometry3<f64>,
    target_color: Option<[f32; 3]>,
    options: &CoatOptions,
    tally: Tally,
    trigger: &Trigger,
) -> FilmCoat {
    let sprayed = tally.sprayed;
    let on_time = tally.on_time;
    let too_close_time = tally.too_close_time;
    // Every statistic is over the *worked* surface — the patches the gun
    // had in range and facing it at some point. Averaging a part's back
    // face into its film would report a beautifully painted panel as half
    // bare, and would make the numbers depend on how much unrelated
    // geometry the target obstacle happens to carry.
    let surface_area: f64 = patches.area.iter().sum();
    let worked = |i: usize| patches.exposed[i];
    let total_area: f64 = (0..patches.area.len())
        .filter(|i| worked(*i))
        .map(|i| patches.area[i])
        .sum();
    // Deposition is counted everywhere, though: paint that landed on a
    // face the gun only grazed is still paint that left the gun.
    let deposited: f64 = patches
        .film
        .iter()
        .zip(&patches.area)
        .map(|(t, a)| t * a)
        .sum();
    let on_worked: f64 = (0..patches.area.len())
        .filter(|i| worked(*i))
        .map(|i| patches.film[i] * patches.area[i])
        .sum();
    let mean = if total_area > 0.0 {
        on_worked / total_area
    } else {
        0.0
    };
    let variance = if total_area > 0.0 {
        (0..patches.area.len())
            .filter(|i| worked(*i))
            .map(|i| patches.area[i] * (patches.film[i] - mean).powi(2))
            .sum::<f64>()
            / total_area
    } else {
        0.0
    };
    let mut min = f64::INFINITY;
    let mut max: f64 = 0.0;
    let mut uncoated_area = 0.0;
    let (mut thin_area, mut thick_area) = (0.0, 0.0);
    for i in 0..patches.area.len() {
        if !worked(i) {
            continue;
        }
        let (t, a) = (patches.film[i], patches.area[i]);
        min = min.min(t);
        max = max.max(t);
        if t <= 0.0 {
            uncoated_area += a;
        }
        if let Some((lo, hi)) = options.spec {
            if t < lo {
                thin_area += a;
            } else if t > hi {
                thick_area += a;
            }
        }
    }
    let in_spec_ratio = options.spec.map(|_| {
        if total_area > 0.0 {
            1.0 - (thin_area + thick_area) / total_area
        } else {
            0.0
        }
    });

    let palette = FilmPalette::resolve(options, max, target_color);
    let face_colors = film_colors(&patches.film, &palette);
    FilmCoat {
        target: target.to_string(),
        mesh: botrail_mesh::MeshData {
            vertices: patches.vertices,
            indices: patches.indices,
            face_colors,
        },
        pose,
        patch_size: options.patch_size,
        patch_count: patches.film.len(),
        thickness: patches.film,
        exposed: patches.exposed,
        surface_area,
        total_area,
        mean,
        min: if min.is_finite() { min } else { 0.0 },
        max,
        sigma: variance.sqrt(),
        uncoated_area,
        thin_area,
        thick_area,
        in_spec_ratio,
        sprayed_volume: sprayed,
        deposited_volume: deposited,
        gun_on_time: on_time,
        too_close_time,
        spec: options.spec,
        palette,
        overspray: tally.overspray_named,
        lost_volume: (sprayed - deposited - tally.overspray_total).max(0.0),
        sprayed_by_brush: trigger
            .brush_names
            .iter()
            .cloned()
            .zip(tally.sprayed_by_brush.iter().copied())
            .filter(|(_, v)| *v > 0.0)
            .collect(),
        deposited_by_brush: trigger
            .brush_names
            .iter()
            .cloned()
            .zip(tally.deposited_by_brush.iter().copied())
            .filter(|(name, _)| {
                tally.sprayed_by_brush[trigger
                    .brush_names
                    .iter()
                    .position(|n| n == name)
                    .unwrap_or(0)]
                    > 0.0
            })
            .collect(),
    }
}

// --------------------------------------------------------- standoff check

/// When the gun is spraying, as far as the timeline can tell — and with
/// what: the PLC's enable signal (`gate`, if one was named) *and* the
/// program's own trigger — the spraying strokes of whatever toolpath the
/// robot was running, each with its brush. Rapids, gun-off moves, and the
/// approach planned in from wherever the robot stood never spray, however
/// the enable was authored: a gun opened by the sequence in the same step
/// the toolpath starts must not paint the part on the way in. A brush's
/// lead and lag widen its strokes' spans. A timeline that ran no toolpath
/// at all (a hand-built or motion-only one) has no program to say when
/// the process was on, so there the enable alone decides, with the
/// default applicator.
struct Trigger<'a> {
    gate: Option<&'a crate::rollout::BoolTrack>,
    /// `None` = no toolpath ran: the whole timeline is process time.
    spans: Option<Vec<TriggerSpan>>,
    /// Every applicator the walk can spray with, prepared once.
    guns: Vec<Prepared>,
    /// Which of `guns` a span without a brush uses (the applicator handed
    /// in), if any.
    default_gun: Option<usize>,
    /// Brush names, indexed by [`Active::brush`]; `""` is "no brush".
    brush_names: Vec<String>,
}

struct TriggerSpan {
    start: f64,
    end: f64,
    gun: usize,
    scale: f64,
    brush: usize,
}

/// What is spraying at one instant.
#[derive(Clone, Copy)]
struct Active {
    gun: usize,
    /// Multiplier on the gun's flow (the brush's).
    scale: f64,
    /// Index into [`Trigger::brush_names`].
    brush: usize,
}

impl<'a> Trigger<'a> {
    fn resolve(
        scene: &Scene,
        timeline: &'a SequenceTimeline,
        robot: usize,
        gate: Option<&str>,
        default: Option<&Applicator>,
    ) -> Result<Self, CoatError> {
        let gate = match gate {
            None => None,
            Some(name) => Some(
                timeline
                    .signals
                    .iter()
                    .find(|s| s.name == name)
                    .ok_or_else(|| CoatError::UnknownGate(name.to_string()))?,
            ),
        };
        let mut guns: Vec<Prepared> = Vec::new();
        let mut gun_names: Vec<String> = Vec::new();
        let mut brush_names: Vec<String> = vec![String::new()];
        let default_gun = match default {
            Some(a) => {
                guns.push(a.prepare()?);
                gun_names.push(String::new());
                Some(0)
            }
            None => None,
        };
        let spans = match timeline.process_spans(robot) {
            None => {
                if default_gun.is_none() {
                    return Err(CoatError::NoApplicator);
                }
                None
            }
            Some(process) => {
                let mut out = Vec::with_capacity(process.len());
                for span in process {
                    match &span.brush {
                        None => {
                            let Some(gun) = default_gun else {
                                return Err(CoatError::NoApplicator);
                            };
                            out.push(TriggerSpan {
                                start: span.start,
                                end: span.end,
                                gun,
                                scale: 1.0,
                                brush: 0,
                            });
                        }
                        Some(name) => {
                            let brush = scene
                                .brush(name)
                                .ok_or_else(|| CoatError::UnknownBrush(name.clone()))?;
                            let applicator =
                                scene.applicator(&brush.applicator).ok_or_else(|| {
                                    CoatError::UnknownApplicator(brush.applicator.clone())
                                })?;
                            let gun = match gun_names.iter().position(|n| *n == brush.applicator) {
                                Some(i) => i,
                                None => {
                                    guns.push(applicator.prepare()?);
                                    gun_names.push(brush.applicator.clone());
                                    guns.len() - 1
                                }
                            };
                            let brush_index = match brush_names.iter().position(|n| n == name) {
                                Some(i) => i,
                                None => {
                                    brush_names.push(name.clone());
                                    brush_names.len() - 1
                                }
                            };
                            out.push(TriggerSpan {
                                start: span.start - brush.lead,
                                end: span.end + brush.lag,
                                gun,
                                scale: brush.flow,
                                brush: brush_index,
                            });
                        }
                    }
                }
                Some(out)
            }
        };
        Ok(Trigger {
            gate,
            spans,
            guns,
            default_gun,
            brush_names,
        })
    }

    /// The gun spraying at `t`, if any.
    fn active(&self, t: f64) -> Option<Active> {
        if self.gate.is_some_and(|g| !g.value_at(t)) {
            return None;
        }
        match &self.spans {
            None => self.default_gun.map(|gun| Active {
                gun,
                scale: 1.0,
                brush: 0,
            }),
            Some(spans) => spans
                .iter()
                .find(|s| s.start <= t && t <= s.end)
                .map(|s| Active {
                    gun: s.gun,
                    scale: s.scale,
                    brush: s.brush,
                }),
        }
    }

    fn on(&self, t: f64) -> bool {
        self.active(t).is_some()
    }
}

/// A fixture the spray can be shadowed by, or land on: any enabled
/// obstacle other than the target, with its exact surface and the
/// transform from target-local coordinates into its own.
struct Fixture {
    name: String,
    surface: ObstacleCollider,
    /// target-local -> fixture-local.
    to_fixture: Isometry3<f64>,
    /// The fixture's bounds in target-local coordinates, for culling.
    lo: Vector3<f64>,
    hi: Vector3<f64>,
}

impl Fixture {
    fn gather(scene: &Scene, target: &str, target_pose: &Isometry3<f64>) -> Vec<Fixture> {
        let mut out = Vec::new();
        for (obstacle, collider) in scene.obstacles().iter().zip(scene.obstacle_colliders()) {
            if obstacle.name == target || !obstacle.enabled {
                continue;
            }
            let Ok(surface) = ObstacleCollider::exact_surface(&obstacle.geometry) else {
                continue;
            };
            let Some((mins, maxs)) = collider.aabb(&Isometry3::identity()) else {
                continue;
            };
            // Bounds in target-local coordinates: transform the corners.
            let fixture_to_target = target_pose.inverse() * obstacle.pose;
            let mut lo = Vector3::repeat(f64::INFINITY);
            let mut hi = Vector3::repeat(f64::NEG_INFINITY);
            for k in 0..8 {
                let c = Point3::new(
                    if k & 1 == 0 { mins[0] } else { maxs[0] },
                    if k & 2 == 0 { mins[1] } else { maxs[1] },
                    if k & 4 == 0 { mins[2] } else { maxs[2] },
                );
                let p = fixture_to_target * c;
                lo = lo.inf(&p.coords);
                hi = hi.sup(&p.coords);
            }
            out.push(Fixture {
                name: obstacle.name.clone(),
                surface,
                to_fixture: obstacle.pose.inverse() * target_pose,
                lo,
                hi,
            });
        }
        out
    }

    /// Whether the segment `tip -> tip + v * stop` (target-local) meets
    /// this fixture. Bounds first, then the ray.
    fn blocks(&self, tip: &Vector3<f64>, v: &Vector3<f64>, stop: f64) -> bool {
        let end = tip + v * stop;
        let seg_lo = tip.inf(&end);
        let seg_hi = tip.sup(&end);
        if (0..3).any(|i| seg_hi[i] < self.lo[i] || seg_lo[i] > self.hi[i]) {
            return false;
        }
        let origin = self.to_fixture * Point3::from(*tip);
        let dir = self.to_fixture.rotation * v;
        self.surface.cast_local_ray(&origin, &dir, stop).is_some()
    }

    /// Distance (in units of `dir`) to this fixture along a ray, if hit
    /// within `max_toi`.
    fn hit(&self, tip: &Vector3<f64>, dir: &Vector3<f64>, max_toi: f64) -> Option<f64> {
        let end = tip + dir * max_toi;
        let seg_lo = tip.inf(&end);
        let seg_hi = tip.sup(&end);
        if (0..3).any(|i| seg_hi[i] < self.lo[i] || seg_lo[i] > self.hi[i]) {
            return None;
        }
        let origin = self.to_fixture * Point3::from(*tip);
        let d = self.to_fixture.rotation * dir;
        self.surface.cast_local_ray(&origin, &d, max_toi)
    }
}

/// The effective trigger — enable signal AND program, brush lead/lag
/// included — as a signal track named `name`, sampled every `dt`: what a
/// timing chart or a spray-cone effect binds to, so the picture follows
/// what actually sprayed rather than the enable alone. A program that
/// names no brush needs no applicator here (only the timing is read).
pub fn trigger_track(
    scene: &Scene,
    timeline: &SequenceTimeline,
    robot: usize,
    gate: Option<&str>,
    name: &str,
    dt: f64,
) -> Result<crate::rollout::BoolTrack, CoatError> {
    let stand_in = Applicator {
        standoff: 0.25,
        pattern: Pattern::Round {
            diameter: 0.1,
            beta: 1.0,
        },
        flow: 1e-6,
        transfer_efficiency: 1.0,
        max_range: 1.0,
    };
    let trigger = Trigger::resolve(scene, timeline, robot, gate, Some(&stand_in))?;
    let dt = if dt > 0.0 { dt } else { 0.01 };
    let mut edges: Vec<(f64, bool)> = vec![(0.0, false)];
    let mut on = false;
    let mut t = 0.0;
    while t <= timeline.duration + 1e-9 {
        let now = trigger.on(t.min(timeline.duration));
        if now != on {
            if t == 0.0 {
                edges[0] = (0.0, now);
            } else {
                edges.push((t, now));
            }
            on = now;
        }
        if t >= timeline.duration {
            break;
        }
        t += dt;
    }
    Ok(crate::rollout::BoolTrack {
        name: name.to_string(),
        edges,
        kind: crate::rollout::LaneKind::Signal,
    })
}

/// The teaching rules a spray program is checked against. Painting is
/// non-contact, so the questions are the mirror of machining's contact
/// exemption: not "may this touch" but "is this the right distance, and
/// square enough on".
#[derive(Debug, Clone)]
pub struct PaintLimits {
    /// Acceptable gun-to-surface distance [m], measured along the spray
    /// axis. `None` reports the distances without judging them.
    pub standoff: Option<(f64, f64)>,
    /// Steepest acceptable angle [rad] between the spray axis and the
    /// surface normal.
    pub max_incidence: f64,
    /// How far to look for the surface [m]; a probe finding nothing
    /// within it is `NoTarget`.
    pub max_range: f64,
}

impl Default for PaintLimits {
    fn default() -> Self {
        PaintLimits {
            standoff: None,
            max_incidence: std::f64::consts::FRAC_PI_4,
            max_range: 1.0,
        }
    }
}

/// One look from the gun to the surface.
#[derive(Debug, Clone)]
pub struct PaintProbe {
    /// Where along the program: arc length [m] from the first sample for
    /// an authored path, seconds for a baked timeline.
    pub at: f64,
    /// Index into the toolpath's moves (authored path only).
    pub move_index: Option<usize>,
    /// World position of the gun tip.
    pub position: Point3<f64>,
    /// Distance along the spray axis to the target, if it was hit.
    pub standoff: Option<f64>,
    /// Angle [rad] between the spray axis and the surface normal at the
    /// hit; `None` when nothing was hit.
    pub incidence: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PaintIssueKind {
    /// The spray axis does not meet the target within `max_range`.
    NoTarget,
    /// Standoff above the band.
    TooFar,
    /// Standoff below the band.
    TooClose,
    /// Incidence above `max_incidence`.
    Oblique,
}

impl PaintIssueKind {
    /// Stable snake_case name, the same one the wire and Python use.
    pub fn as_str(&self) -> &'static str {
        match self {
            PaintIssueKind::NoTarget => "no_target",
            PaintIssueKind::TooFar => "too_far",
            PaintIssueKind::TooClose => "too_close",
            PaintIssueKind::Oblique => "oblique",
        }
    }
}

#[derive(Debug, Clone)]
pub struct PaintIssue {
    /// Index into [`PaintReport::probes`].
    pub sample: usize,
    pub at: f64,
    pub move_index: Option<usize>,
    pub position: Point3<f64>,
    pub kind: PaintIssueKind,
    /// The offending number: standoff [m] for the distance kinds,
    /// incidence [rad] for `Oblique`, `max_range` for `NoTarget`.
    pub value: f64,
}

/// Face diagnosis of a spray program against a target: every spraying
/// sample probed, every finding collected. The same shape whether the
/// program is an authored path (`at` in meters along it) or a baked
/// timeline (`at` in seconds).
///
/// Two kinds of finding, judged differently. Wherever the gun *was* over
/// the part, the distance and angle rules apply and a violation fails
/// the check. Where it was not — `NoTarget` — nothing failed: a raster's
/// overtravel is supposed to run past the part, and whether the gun
/// should be closed there is a triggering question, not a teaching one.
/// Off-target samples are still reported (marks, spans, the on-target
/// ratio) because they are the overspray.
#[derive(Debug, Clone)]
pub struct PaintReport {
    pub probes: Vec<PaintProbe>,
    pub issues: Vec<PaintIssue>,
    /// Probes that met the target.
    pub hits: usize,
    /// Over the hits; zero when there were none.
    pub standoff_min: f64,
    pub standoff_max: f64,
    pub standoff_mean: f64,
    pub incidence_max: f64,
    /// Of the probes that met the target, the fraction inside every rule
    /// — the program's adherence to its teaching rules where they apply.
    pub in_band_ratio: f64,
    /// Fraction of all probes that met the target at all — how much of
    /// the spraying was pointed at the part.
    pub on_target_ratio: f64,
}

impl PaintReport {
    /// The program met its target somewhere, and everywhere it did, it
    /// kept the distance and angle rules. Off-target stretches do not
    /// count against it (see the type docs).
    pub fn ok(&self) -> bool {
        self.hits > 0
            && !self
                .issues
                .iter()
                .any(|i| i.kind != PaintIssueKind::NoTarget)
    }

    /// Runs of consecutive issue samples of one kind, as `(at, at)`
    /// ranges — the stretches of the program to look at.
    pub fn spans(&self, kind: PaintIssueKind) -> Vec<(f64, f64)> {
        let mut out: Vec<(f64, f64)> = Vec::new();
        let mut last_sample: Option<usize> = None;
        for issue in self.issues.iter().filter(|i| i.kind == kind) {
            match (last_sample, out.last_mut()) {
                (Some(prev), Some(span)) if issue.sample == prev + 1 => span.1 = issue.at,
                _ => out.push((issue.at, issue.at)),
            }
            last_sample = Some(issue.sample);
        }
        out
    }
}

/// Casts one probe from `tip` along `dir` (world) at the target and
/// reads standoff and incidence off the hit.
fn look(
    collider: &ObstacleCollider,
    to_local: &Isometry3<f64>,
    tip: &Point3<f64>,
    dir: &Vector3<f64>,
    max_range: f64,
) -> (Option<f64>, Option<f64>) {
    let origin = to_local * tip;
    let local_dir = to_local.rotation * dir;
    match collider.cast_local_ray_with_normal(&origin, &local_dir, max_range) {
        None => (None, None),
        Some((toi, normal)) => {
            // Outward normal against the spray direction: square on is
            // zero, grazing is a right angle. A hit from behind (a mesh
            // whose winding is inside out) clamps to a right angle rather
            // than reading as a good hit.
            let cos = (-normal.dot(&local_dir)).clamp(-1.0, 1.0);
            let incidence = if cos <= 0.0 { FRAC_PI_2 } else { cos.acos() };
            (Some(toi), Some(incidence))
        }
    }
}

fn assess(probes: Vec<PaintProbe>, limits: &PaintLimits) -> PaintReport {
    let mut issues = Vec::new();
    let mut hits = 0usize;
    let mut min = f64::INFINITY;
    let mut max = 0.0f64;
    let mut sum = 0.0;
    let mut incidence_max = 0.0f64;
    let mut clean = 0usize;
    for (i, p) in probes.iter().enumerate() {
        let mut issue = |kind: PaintIssueKind, value: f64| {
            issues.push(PaintIssue {
                sample: i,
                at: p.at,
                move_index: p.move_index,
                position: p.position,
                kind,
                value,
            })
        };
        match (p.standoff, p.incidence) {
            (Some(d), Some(a)) => {
                hits += 1;
                min = min.min(d);
                max = max.max(d);
                sum += d;
                incidence_max = incidence_max.max(a);
                let mut bad = false;
                if let Some((lo, hi)) = limits.standoff {
                    if d < lo {
                        issue(PaintIssueKind::TooClose, d);
                        bad = true;
                    } else if d > hi {
                        issue(PaintIssueKind::TooFar, d);
                        bad = true;
                    }
                }
                if a > limits.max_incidence {
                    issue(PaintIssueKind::Oblique, a);
                    bad = true;
                }
                if !bad {
                    clean += 1;
                }
            }
            _ => issue(PaintIssueKind::NoTarget, limits.max_range),
        }
    }
    let n = probes.len();
    PaintReport {
        hits,
        standoff_min: if hits > 0 { min } else { 0.0 },
        standoff_max: max,
        standoff_mean: if hits > 0 { sum / hits as f64 } else { 0.0 },
        incidence_max,
        in_band_ratio: if hits > 0 {
            clean as f64 / hits as f64
        } else {
            0.0
        },
        on_target_ratio: if n > 0 { hits as f64 / n as f64 } else { 0.0 },
        probes,
        issues,
    }
}

/// Checks an authored spray program against `target` before anything is
/// baked: every *feed* sample of `toolpath` (rapids are not spraying)
/// looks along its spray axis and reports standoff and incidence. Pure
/// geometry — no robot is involved, so this runs before one is chosen,
/// and it is the same check whether the path was typed, generated, or
/// imported.
pub fn check_paint(
    scene: &Scene,
    toolpath: &crate::toolpath::Toolpath,
    target: &str,
    limits: &PaintLimits,
    options: &crate::toolpath::ToolpathOptions,
) -> Result<PaintReport, CoatError> {
    let (obstacle, _) = scene
        .obstacle_with_collider(target)
        .ok_or_else(|| CoatError::UnknownTarget(target.to_string()))?;
    let surface = exact_surface(target, &obstacle.geometry)?;
    let collider = &surface;
    let samples = crate::toolpath::resolve_and_sample(scene, toolpath, options)?;
    let to_local = obstacle.pose.inverse();
    let mut probes = Vec::new();
    let mut arc = 0.0;
    for s in &samples {
        arc += s.chord;
        if s.feed.is_none() {
            continue;
        }
        // Spray runs against the tool axis (tip -> body is +Z).
        let dir = -s.tool_axis.into_inner();
        let (standoff, incidence) = look(collider, &to_local, &s.position, &dir, limits.max_range);
        probes.push(PaintProbe {
            at: arc,
            move_index: Some(s.move_index),
            position: s.position,
            standoff,
            incidence,
        });
    }
    Ok(assess(probes, limits))
}

/// The same check over a baked timeline: what the robot actually did,
/// sampled every `dt` while `gate` (if any) was high, with the spray
/// axis read off the TCP's forward kinematics. `scene` is the pre-rollout
/// snapshot the timeline was baked against.
#[allow(clippy::too_many_arguments)]
pub fn timeline_paint_report(
    scene: &Scene,
    timeline: &SequenceTimeline,
    target: &str,
    robot: usize,
    tcp: usize,
    gate: Option<&str>,
    dt: f64,
    limits: &PaintLimits,
) -> Result<PaintReport, CoatError> {
    let (obstacle, _) = scene
        .obstacle_with_collider(target)
        .ok_or_else(|| CoatError::UnknownTarget(target.to_string()))?;
    let surface = exact_surface(target, &obstacle.geometry)?;
    let collider = &surface;
    // The standoff check needs the trigger's timing, not its guns; a
    // program that names no brush is checked with a stand-in.
    let stand_in = Applicator {
        standoff: 0.25,
        pattern: Pattern::Round {
            diameter: 0.1,
            beta: 1.0,
        },
        flow: 1e-6,
        transfer_efficiency: 1.0,
        max_range: 1.0,
    };
    let trigger = Trigger::resolve(scene, timeline, robot, gate, Some(&stand_in))?;
    let to_local = obstacle.pose.inverse();
    let track = &timeline.robots[robot].trajectory;
    let dt = if dt > 0.0 { dt } else { 0.01 };
    let mut probes = Vec::new();
    let mut t = 0.0;
    while t <= timeline.duration + 1e-9 {
        let t_now = t.min(timeline.duration);
        if trigger.on(t_now) {
            if let Ok(poses) = scene.fk_for(robot, &track.sample(t_now)) {
                let pose = poses[tcp];
                let tip = Point3::from(pose.translation.vector);
                let dir = -(pose.rotation * Vector3::z());
                let (standoff, incidence) = look(collider, &to_local, &tip, &dir, limits.max_range);
                probes.push(PaintProbe {
                    at: t_now,
                    move_index: None,
                    position: tip,
                    standoff,
                    incidence,
                });
            }
        }
        if t >= timeline.duration {
            break;
        }
        t += dt;
    }
    Ok(assess(probes, limits))
}

// ------------------------------------------------------------------ color

/// Sequential blue ramp, light -> dark, sRGB — steps 250..700 of the
/// documented scale. Magnitude gets one hue: a rainbow would invent
/// structure the film does not have. It starts at 250 rather than 100 so
/// the first paint stands off a light substrate.
const AMOUNT_RAMP: [u32; 10] = [
    0x86b6ef, 0x6da7ec, 0x5598e7, 0x3987e5, 0x2a78d6, 0x256abf, 0x1c5cab, 0x184f95, 0x104281,
    0x0d366b,
];

/// Bare substrate when the target has no colour of its own. Not a ramp
/// step: "never sprayed" is a category, not a small amount of paint, and
/// the dark neutral reads as uncoated metal against every step.
const UNCOATED: u32 = 0x383835;

/// The bare-substrate colour used when the target has none, linear RGB.
pub fn uncoated_color() -> [f32; 3] {
    srgb_to_linear(UNCOATED)
}

fn srgb_to_linear(hex: u32) -> [f32; 3] {
    let channel = |shift: u32| {
        let c = ((hex >> shift) & 0xff) as f32 / 255.0;
        if c <= 0.04045 {
            c / 12.92
        } else {
            ((c + 0.055) / 1.055).powf(2.4)
        }
    };
    [channel(16), channel(8), channel(0)]
}

/// The diverging map a film wears when it is judged against a spec:
/// thin is one hue, thick the other, on target is neutral. Five steps
/// inside each half of the band and one saturated step past it, so
/// in-spec ripple reads as gradation and out-of-spec as a hard colour.
/// Blue for thin (the sequential hue, so the two maps agree on what
/// "less" looks like), red for thick, lightest first.
const THIN_ARM: [u32; 6] = [0xcde2fb, 0x9ec5f4, 0x6da7ec, 0x3987e5, 0x1c5cab, 0x0d366b];
const THICK_ARM: [u32; 6] = [0xf9d3d2, 0xf1aaa9, 0xe97b7a, 0xe34948, 0xa52a29, 0x631615];
/// The midpoint: neutral, and light, so it reads as "nothing to see".
const ON_TARGET: u32 = 0xf0efec;

impl FilmPalette {
    fn resolve(options: &CoatOptions, max: f64, target_color: Option<[f32; 3]>) -> FilmPalette {
        let style = match options.style {
            FilmStyle::Auto if options.spec.is_some() => FilmStyle::Spec,
            FilmStyle::Auto => FilmStyle::Amount,
            other => other,
        };
        let ramp = match options.paint_color {
            // A wash of the paint's colour, thin to full: linear-space
            // interpolation from a light tint toward the colour itself, so
            // "more paint" is "more of that colour" — the way a coat looks
            // as it goes on.
            Some(paint) => (1..=AMOUNT_RAMP.len())
                .map(|k| {
                    let s = 0.25 + 0.75 * (k as f32 / AMOUNT_RAMP.len() as f32);
                    [
                        1.0 - (1.0 - paint[0]) * s,
                        1.0 - (1.0 - paint[1]) * s,
                        1.0 - (1.0 - paint[2]) * s,
                    ]
                })
                .collect(),
            None => AMOUNT_RAMP.iter().map(|c| srgb_to_linear(*c)).collect(),
        };
        FilmPalette {
            style,
            ramp,
            top: options.spec.map(|(_, hi)| hi).unwrap_or(max),
            uncoated: options
                .substrate
                .or(target_color)
                .unwrap_or_else(uncoated_color),
            spec: options.spec,
        }
    }
}

/// Quantizes the film onto the palette. Banding is deliberate: discrete
/// steps read as contours, which is how a lap-streak becomes visible at a
/// glance — and it keeps the OBJ to a dozen materials instead of one per
/// patch.
///
/// `Amount`: the ramp over `0..top` (how much paint), the top step for
/// anything at or past it. `Spec`: diverging over the band (which side of
/// target, and whether inside it) — a uniform film reads as flat neutral
/// instead of flat dark, and striping shows as alternating tints; the
/// polarity is the information, not the magnitude.
fn film_colors(film: &[f64], palette: &FilmPalette) -> Vec<[f32; 3]> {
    let bare = palette.uncoated;
    match palette.style {
        FilmStyle::Amount | FilmStyle::Auto => {
            let n = palette.ramp.len();
            film.iter()
                .map(|t| {
                    if *t <= 0.0 || palette.top <= 0.0 {
                        bare
                    } else {
                        let step = ((t / palette.top) * n as f64).ceil() as usize;
                        palette.ramp[step.clamp(1, n) - 1]
                    }
                })
                .collect()
        }
        FilmStyle::Spec => {
            let (lo, hi) = palette.spec.unwrap_or((0.0, palette.top.max(1e-12)));
            let thin: Vec<[f32; 3]> = THIN_ARM.iter().map(|c| srgb_to_linear(*c)).collect();
            let thick: Vec<[f32; 3]> = THICK_ARM.iter().map(|c| srgb_to_linear(*c)).collect();
            let neutral = srgb_to_linear(ON_TARGET);
            let mid = (lo + hi) / 2.0;
            let half = ((hi - lo) / 2.0).max(1e-12);
            film.iter()
                .map(|t| {
                    if *t <= 0.0 {
                        return bare;
                    }
                    let d = (t - mid) / half;
                    let arm = if d < 0.0 { &thin } else { &thick };
                    // Five bands inside the half-band, the sixth is out.
                    let step = (d.abs() * 5.0).ceil() as usize;
                    if step == 0 {
                        neutral
                    } else {
                        arm[step.min(6) - 1]
                    }
                })
                .collect()
        }
    }
}

/// The colour key of a film map, top to bottom, as `(linear RGB, label)`
/// pairs; empty labels are swatches without text. Amount: the ramp's
/// steps with microns at the top, middle and bottom (and the spec edges,
/// when the ramp was scaled to a spec). Spec: the out-of-spec ends, the
/// spec edges, and target, in microns.
pub fn film_legend(film: &FilmCoat) -> Vec<([f32; 3], String)> {
    let um = |t: f64| format!("{:.0}", t * 1e6);
    let palette = &film.palette;
    let mut stops: Vec<([f32; 3], String)> = match palette.style {
        FilmStyle::Amount | FilmStyle::Auto => {
            let n = palette.ramp.len();
            palette
                .ramp
                .iter()
                .enumerate()
                .rev()
                .map(|(i, color)| {
                    let upper = palette.top * (i + 1) as f64 / n as f64;
                    let label = if i + 1 == n {
                        match palette.spec {
                            Some((_, hi)) => format!("{}+", um(hi)),
                            None => um(upper),
                        }
                    } else if i == 0 || i == n / 2 {
                        um(upper)
                    } else {
                        String::new()
                    };
                    (*color, label)
                })
                .collect()
        }
        FilmStyle::Spec => {
            let (lo, hi) = palette.spec.unwrap_or((0.0, palette.top));
            let mid = (lo + hi) / 2.0;
            let mut out = Vec::with_capacity(13);
            out.push((srgb_to_linear(THICK_ARM[5]), format!("> {}", um(hi))));
            for (i, c) in THICK_ARM[..5].iter().enumerate().rev() {
                let label = if i == 4 { um(hi) } else { String::new() };
                out.push((srgb_to_linear(*c), label));
            }
            out.push((srgb_to_linear(ON_TARGET), um(mid)));
            for (i, c) in THIN_ARM[..5].iter().enumerate() {
                let label = if i == 4 { um(lo) } else { String::new() };
                out.push((srgb_to_linear(*c), label));
            }
            out.push((srgb_to_linear(THIN_ARM[5]), format!("< {}", um(lo))));
            out
        }
    };
    stops.push((palette.uncoated, "uncoated".to_string()));
    stops
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rollout::BoolTrack;
    use botrail_model::RobotModel;
    use botrail_traj::JointTrajectory;
    use std::sync::Arc;

    const ARM: &str = include_str!("../../../examples/simple_arm.urdf");
    const SPINDLE: &str = include_str!("../../../examples/assets/spindle.urdf");

    fn round_gun() -> Applicator {
        Applicator {
            standoff: 0.25,
            pattern: Pattern::Round {
                diameter: 0.20,
                beta: 2.0,
            },
            flow: 200e-6,
            transfer_efficiency: 0.80,
            max_range: 0.60,
        }
    }

    /// Arm + spindle: the spindle ships the tool-frame convention the
    /// coater wants (TCP `+Z` runs tip toward body, so paint goes `-Z`),
    /// which is exactly what a real applicator would carry. Standing in
    /// for an applicator asset until the catalog has one.
    fn gun_robot() -> RobotModel {
        let arm = RobotModel::from_urdf_str(ARM).unwrap();
        let tool = RobotModel::from_urdf_str(SPINDLE).unwrap();
        arm.attach_tool(
            &tool,
            Some("tool0"),
            None,
            Isometry3::identity(),
            None,
            None,
        )
        .unwrap()
    }

    /// A plate parked `standoff` below the TCP, square on to it: the
    /// canonical calibration geometry.
    fn plate_under_gun(scene: &mut Scene, q: &[f64], standoff: f64, size: [f64; 3]) {
        scene.set_joint_positions(q.to_vec()).unwrap();
        let tcp = scene.robot().default_tcp_link();
        let tip = scene.link_poses()[tcp].translation.vector;
        scene
            .add_obstacle(
                "plate",
                Geometry::Box {
                    size: Vector3::new(size[0], size[1], size[2]),
                },
                Isometry3::translation(tip.x, tip.y, tip.z - standoff - size[2] / 2.0),
            )
            .unwrap();
    }

    /// Flange-down: the TCP's `-Z` (the spray direction) points at the
    /// floor, so a plate below it takes the whole cone square on.
    const DOWN_Q: [f64; 6] = [0.0, 0.5, 0.9, std::f64::consts::PI - 1.4, 0.0, 0.0];

    fn hold_timeline(
        scene: &Scene,
        q: Vec<f64>,
        duration: f64,
    ) -> crate::rollout::SequenceTimeline {
        scene.timeline_from_trajectory(
            0,
            &JointTrajectory {
                times: vec![0.0, duration],
                positions: vec![q.clone(), q],
                velocities: vec![vec![0.0; 6], vec![0.0; 6]],
            },
            "hold",
        )
    }

    #[test]
    fn pattern_normalization_conserves_paint() {
        // A stationary gun at the reference standoff, square on to a
        // plate wide enough to catch the whole cone: everything the
        // applicator delivers (times its transfer efficiency) has to end
        // up as film.
        let gun = round_gun().prepare().unwrap();
        // Integrate the normalized shape over the plane; must be 1.
        let n = 400;
        let r = gun.radius;
        let step = 2.0 * r / n as f64;
        let mut sum = 0.0;
        for i in 0..n {
            for j in 0..n {
                let u = -r + (i as f64 + 0.5) * step;
                let w = -r + (j as f64 + 0.5) * step;
                sum += gun.rate * gun.pattern.shape(u, w);
            }
        }
        let volume_rate = sum * step * step;
        let expected = 200e-6 * 0.80;
        assert!(
            (volume_rate - expected).abs() / expected < 0.005,
            "normalized pattern delivers {volume_rate} m^3/s, expected {expected}"
        );
    }

    #[test]
    fn dual_beta_footprint_is_elliptic() {
        let p = Pattern::DualBeta {
            width: 0.30,
            height: 0.08,
            beta_across: 2.0,
            beta_along: 2.0,
        };
        assert!(p.shape(0.0, 0.0) > 0.0);
        // On the fan's long axis, inside and outside the ends.
        assert!(p.shape(0.14, 0.0) > 0.0);
        assert_eq!(p.shape(0.16, 0.0), 0.0);
        // The along-extent narrows toward the ends: a point that is
        // inside at mid-fan is outside near the tip.
        assert!(p.shape(0.0, 0.035) > 0.0);
        assert_eq!(p.shape(0.145, 0.035), 0.0);
    }

    #[test]
    fn beta_below_one_is_rejected() {
        let mut gun = round_gun();
        gun.pattern = Pattern::Round {
            diameter: 0.2,
            beta: 0.5,
        };
        assert!(matches!(gun.prepare(), Err(CoatError::BadApplicator(_))));
    }

    #[test]
    fn box_tessellation_is_outward_wound() {
        let (vertices, indices) = tessellate(
            "plate",
            &Geometry::Box {
                size: Vector3::new(0.2, 0.1, 0.02),
            },
            0.01,
        )
        .unwrap();
        let patches = Patches::build(vertices, indices);
        // Every face of a box points away from its center.
        for (c, n) in patches.centroid.iter().zip(&patches.normal) {
            assert!(
                c.dot(n) > 0.0,
                "inward-facing patch at {c:?} with normal {n:?}"
            );
        }
        // And the areas add up to the box's surface.
        let area: f64 = patches.area.iter().sum();
        let expected = 2.0 * (0.2 * 0.1 + 0.2 * 0.02 + 0.1 * 0.02);
        assert!(
            (area - expected).abs() < 1e-9,
            "area {area}, expected {expected}"
        );
    }

    #[test]
    fn cylinder_and_sphere_tessellate_to_their_area() {
        let (v, i) = tessellate(
            "drum",
            &Geometry::Cylinder {
                radius: 0.1,
                length: 0.3,
            },
            0.01,
        )
        .unwrap();
        let area: f64 = Patches::build(v, i).area.iter().sum();
        let exact = std::f64::consts::TAU * 0.1 * 0.3 + 2.0 * std::f64::consts::PI * 0.01;
        assert!(
            (area / exact - 1.0).abs() < 0.02,
            "cylinder area {area}, exact {exact}"
        );

        let (v, i) = tessellate("ball", &Geometry::Sphere { radius: 0.1 }, 0.01).unwrap();
        let area: f64 = Patches::build(v, i).area.iter().sum();
        let exact = 4.0 * std::f64::consts::PI * 0.01;
        assert!(
            (area / exact - 1.0).abs() < 0.02,
            "sphere area {area}, exact {exact}"
        );
    }

    #[test]
    fn patch_cap_is_enforced() {
        let err = tessellate(
            "plate",
            &Geometry::Box {
                size: Vector3::new(4.0, 4.0, 0.01),
            },
            0.0005,
        )
        .unwrap_err();
        assert!(matches!(err, CoatError::TooFine { .. }), "{err}");
    }

    /// The acceptance test for the whole model: park the gun at its
    /// reference standoff over a plate big enough to catch the cone, and
    /// every drop it delivers (times its transfer efficiency) has to show
    /// up as film. Everything else the integrator does is a perturbation
    /// of this.
    #[test]
    fn a_parked_gun_deposits_what_it_sprays() {
        let mut scene = Scene::new(Arc::new(gun_robot()));
        plate_under_gun(&mut scene, &DOWN_Q, 0.25, [0.3, 0.3, 0.01]);
        let timeline = hold_timeline(&scene, DOWN_Q.to_vec(), 2.0);
        let tcp = scene.robot().default_tcp_link();
        let film = spray_coat(
            &scene,
            &timeline,
            "plate",
            0,
            tcp,
            Some(&round_gun()),
            &CoatOptions {
                patch_size: 0.004,
                ..CoatOptions::default()
            },
        )
        .unwrap();

        let expected = 200e-6 * 0.80 * 2.0;
        let err = (film.deposited_volume - expected).abs() / expected;
        assert!(
            err < 0.005,
            "deposited {:.4e} m^3, sprayed*TE {:.4e} ({:.1}% off)",
            film.deposited_volume,
            expected,
            err * 100.0
        );
        assert!((film.sprayed_volume - 200e-6 * 2.0).abs() < 1e-12);
        assert!((film.gun_on_time - 2.0).abs() < 1e-9);
        assert_eq!(film.too_close_time, 0.0);
        // A parked round gun leaves a disc: paint in the middle, bare
        // plate everywhere else (both faces plus the rim).
        assert!(film.max > 0.0);
        assert!(film.uncoated_area > film.total_area / 2.0);
    }

    /// Range scaling is the other half of conservation: pull the gun back
    /// to twice its reference standoff and the footprint doubles across
    /// while the film quarters, so the same paint lands.
    #[test]
    fn backing_off_spreads_the_same_paint() {
        let coat_at = |standoff: f64| {
            let mut scene = Scene::new(Arc::new(gun_robot()));
            plate_under_gun(&mut scene, &DOWN_Q, standoff, [0.8, 0.8, 0.01]);
            let timeline = hold_timeline(&scene, DOWN_Q.to_vec(), 1.0);
            let tcp = scene.robot().default_tcp_link();
            spray_coat(
                &scene,
                &timeline,
                "plate",
                0,
                tcp,
                Some(&round_gun()),
                &CoatOptions {
                    patch_size: 0.006,
                    ..CoatOptions::default()
                },
            )
            .unwrap()
        };
        let near = coat_at(0.25);
        let far = coat_at(0.50);
        let ratio = far.deposited_volume / near.deposited_volume;
        assert!(
            (ratio - 1.0).abs() < 0.02,
            "same paint should land at either range, got {ratio:.4}"
        );
        // ...spread four times as thin over four times the area.
        let peak = far.max / near.max;
        assert!(
            (peak - 0.25).abs() < 0.03,
            "peak film should quarter at double range, got {peak:.4}"
        );
    }

    /// The obliquity term, checked where it actually bites. Tilt the
    /// plate and every ray still lands on it, so the paint is conserved —
    /// it is only spread thinner over the longer footprint. Counting the
    /// obliquity against the ray instead of against the reference plane
    /// fails this by several percent.
    #[test]
    fn a_tilted_surface_takes_all_the_paint_thinner() {
        let coat_tilted = |tilt: f64| {
            let mut scene = Scene::new(Arc::new(gun_robot()));
            scene.set_joint_positions(DOWN_Q.to_vec()).unwrap();
            let tcp = scene.robot().default_tcp_link();
            let tip = scene.link_poses()[tcp].translation.vector;
            scene
                .add_obstacle(
                    "plate",
                    Geometry::Box {
                        size: Vector3::new(0.8, 0.8, 0.01),
                    },
                    Isometry3::from_parts(
                        nalgebra::Translation3::new(tip.x, tip.y, tip.z - 0.255),
                        nalgebra::UnitQuaternion::from_axis_angle(&Vector3::y_axis(), tilt),
                    ),
                )
                .unwrap();
            let timeline = hold_timeline(&scene, DOWN_Q.to_vec(), 1.0);
            spray_coat(
                &scene,
                &timeline,
                "plate",
                0,
                tcp,
                Some(&round_gun()),
                &CoatOptions {
                    patch_size: 0.005,
                    ..CoatOptions::default()
                },
            )
            .unwrap()
        };
        let expected = 200e-6 * 0.80;
        for tilt in [0.0, 0.3, 0.6] {
            let film = coat_tilted(tilt);
            let err = (film.deposited_volume - expected).abs() / expected;
            assert!(
                err < 0.01,
                "tilt {tilt} rad deposited {:.4e}, expected {expected:.4e} ({:.1}% off)",
                film.deposited_volume,
                err * 100.0
            );
        }
        // Thinner where it lands: same paint over a longer footprint.
        assert!(coat_tilted(0.6).max < coat_tilted(0.0).max);
    }

    #[test]
    fn the_film_is_deterministic() {
        let mut scene = Scene::new(Arc::new(gun_robot()));
        plate_under_gun(&mut scene, &DOWN_Q, 0.25, [0.3, 0.3, 0.01]);
        let timeline = hold_timeline(&scene, DOWN_Q.to_vec(), 1.0);
        let tcp = scene.robot().default_tcp_link();
        let run = || {
            spray_coat(
                &scene,
                &timeline,
                "plate",
                0,
                tcp,
                Some(&round_gun()),
                &CoatOptions::default(),
            )
            .unwrap()
        };
        let (a, b) = (run(), run());
        assert_eq!(a.thickness, b.thickness, "the film must bake bit for bit");
        assert_eq!(a.mesh.face_colors, b.mesh.face_colors);
    }

    #[test]
    fn the_gate_signal_stops_the_gun() {
        let mut scene = Scene::new(Arc::new(gun_robot()));
        plate_under_gun(&mut scene, &DOWN_Q, 0.25, [0.3, 0.3, 0.01]);
        let mut timeline = hold_timeline(&scene, DOWN_Q.to_vec(), 2.0);
        // On for the first second, off for the second.
        timeline.signals.push(BoolTrack {
            name: "gun".into(),
            edges: vec![(0.0, true), (1.0, false)],
            kind: crate::rollout::LaneKind::Signal,
        });
        let tcp = scene.robot().default_tcp_link();
        let film = spray_coat(
            &scene,
            &timeline,
            "plate",
            0,
            tcp,
            Some(&round_gun()),
            &CoatOptions {
                patch_size: 0.004,
                gate: Some("gun".into()),
                ..CoatOptions::default()
            },
        )
        .unwrap();
        assert!(
            (film.gun_on_time - 1.0).abs() < 0.02,
            "on for {}",
            film.gun_on_time
        );
        let expected = 200e-6 * 0.80;
        assert!((film.deposited_volume - expected).abs() / expected < 0.03);

        let missing = spray_coat(
            &scene,
            &timeline,
            "plate",
            0,
            tcp,
            Some(&round_gun()),
            &CoatOptions {
                gate: Some("nope".into()),
                ..CoatOptions::default()
            },
        );
        assert!(matches!(missing, Err(CoatError::UnknownGate(_))));
    }

    /// Getting too close is reported, not silently absorbed: the inverse
    /// square is outside the measurement's validity there, and a quiet
    /// zero would read as "this stretch laid down nothing".
    #[test]
    fn crowding_the_surface_is_reported() {
        let mut scene = Scene::new(Arc::new(gun_robot()));
        plate_under_gun(&mut scene, &DOWN_Q, 0.02, [0.3, 0.3, 0.01]);
        let timeline = hold_timeline(&scene, DOWN_Q.to_vec(), 1.0);
        let tcp = scene.robot().default_tcp_link();
        let film = spray_coat(
            &scene,
            &timeline,
            "plate",
            0,
            tcp,
            Some(&round_gun()),
            &CoatOptions::default(),
        )
        .unwrap();
        assert!(
            film.too_close_time > 0.9,
            "expected the whole second flagged, got {}",
            film.too_close_time
        );
    }

    /// A part's back face is not a holiday. Statistics run over the
    /// surface the gun actually worked over, so a plate sprayed from
    /// above reports on its top face and nothing else — otherwise every
    /// number would depend on how much unrelated geometry the target
    /// obstacle happens to carry.
    #[test]
    fn the_back_of_the_part_is_not_counted() {
        let mut scene = Scene::new(Arc::new(gun_robot()));
        plate_under_gun(&mut scene, &DOWN_Q, 0.25, [0.3, 0.3, 0.02]);
        let timeline = hold_timeline(&scene, DOWN_Q.to_vec(), 1.0);
        let tcp = scene.robot().default_tcp_link();
        let film = spray_coat(
            &scene,
            &timeline,
            "plate",
            0,
            tcp,
            Some(&round_gun()),
            &CoatOptions {
                patch_size: 0.004,
                ..CoatOptions::default()
            },
        )
        .unwrap();
        let top = 0.3 * 0.3;
        let whole = 2.0 * (0.3 * 0.3 + 2.0 * 0.3 * 0.02);
        assert!((film.surface_area - whole).abs() < 1e-9);
        assert!(
            (film.total_area - top).abs() < 1e-9,
            "worked area {} should be the top face {top}",
            film.total_area
        );
        // The mean is over the top face only, so it is the deposited
        // volume spread over that face — not over both faces and the rim.
        assert!((film.mean - film.deposited_volume / top).abs() / film.mean < 1e-6);
    }

    #[test]
    fn spec_splits_the_area_three_ways() {
        let mut scene = Scene::new(Arc::new(gun_robot()));
        plate_under_gun(&mut scene, &DOWN_Q, 0.25, [0.3, 0.3, 0.01]);
        let timeline = hold_timeline(&scene, DOWN_Q.to_vec(), 1.0);
        let tcp = scene.robot().default_tcp_link();
        let film = spray_coat(
            &scene,
            &timeline,
            "plate",
            0,
            tcp,
            Some(&round_gun()),
            &CoatOptions {
                patch_size: 0.004,
                spec: Some((20e-6, 30e-6)),
                ..CoatOptions::default()
            },
        )
        .unwrap();
        let ratio = film.in_spec_ratio.unwrap();
        let bands = film.thin_area + film.thick_area;
        assert!((1.0 - bands / film.total_area - ratio).abs() < 1e-9);
        // A parked gun is the worst possible film: a peak in the middle
        // and bare plate around it, so almost nothing is in band.
        assert!(ratio < 0.2, "in-spec ratio {ratio}");
        assert!(film.thin_area > 0.0 && film.thick_area > 0.0);
    }
}

#[cfg(test)]
mod standoff_tests {
    use super::*;
    use crate::toolpath::{PathTarget, ToolMove, ToolMoveKind, Toolpath, ToolpathOptions};
    use botrail_model::RobotModel;
    use nalgebra::Unit;
    use std::sync::Arc;

    const ARM: &str = include_str!("../../../examples/simple_arm.urdf");

    fn scene_with_plate(size: [f64; 3], pose: Isometry3<f64>) -> Scene {
        let mut scene = Scene::new(Arc::new(RobotModel::from_urdf_str(ARM).unwrap()));
        scene
            .add_obstacle(
                "plate",
                Geometry::Box {
                    size: Vector3::new(size[0], size[1], size[2]),
                },
                pose,
            )
            .unwrap();
        scene
    }

    fn down(x: f64, y: f64, z: f64) -> PathTarget {
        PathTarget {
            position: Point3::new(x, y, z),
            tool_axis: Unit::new_normalize(Vector3::z()),
            spin: None,
        }
    }

    /// One rapid in, one feed stroke across the plate at height `z`.
    fn stroke(z: f64, x0: f64, x1: f64) -> Toolpath {
        Toolpath {
            name: "stroke".into(),
            frame: None,
            moves: vec![
                ToolMove {
                    kind: ToolMoveKind::Rapid,
                    targets: vec![down(x0, 0.0, z)],
                    brush: None,
                },
                ToolMove {
                    kind: ToolMoveKind::Feed(0.2),
                    targets: vec![down(x1, 0.0, z)],
                    brush: None,
                },
            ],
        }
    }

    #[test]
    fn a_square_stroke_at_the_right_height_is_clean() {
        let scene = scene_with_plate([0.4, 0.4, 0.01], Isometry3::translation(0.0, 0.0, -0.005));
        let report = check_paint(
            &scene,
            &stroke(0.25, -0.15, 0.15),
            "plate",
            &PaintLimits {
                standoff: Some((0.20, 0.30)),
                ..PaintLimits::default()
            },
            &ToolpathOptions::default(),
        )
        .unwrap();
        assert!(report.ok(), "{:?}", report.issues.first());
        assert_eq!(report.hits, report.probes.len());
        assert!((report.standoff_min - 0.25).abs() < 1e-9);
        assert!((report.standoff_max - 0.25).abs() < 1e-9);
        assert!(report.incidence_max < 1e-9);
        assert_eq!(report.in_band_ratio, 1.0);
        // Rapids are not spraying: only the feed samples were probed, and
        // `at` counts arc length from the path's first sample.
        assert!(report.probes.iter().all(|p| p.move_index == Some(1)));
        assert!(report.probes.last().unwrap().at > 0.29);
    }

    #[test]
    fn a_stroke_that_runs_off_the_plate_loses_its_target() {
        let scene = scene_with_plate([0.4, 0.4, 0.01], Isometry3::translation(0.0, 0.0, -0.005));
        let report = check_paint(
            &scene,
            &stroke(0.25, -0.15, 0.40),
            "plate",
            &PaintLimits {
                standoff: Some((0.20, 0.30)),
                ..PaintLimits::default()
            },
            &ToolpathOptions::default(),
        )
        .unwrap();
        // Running past the part is not a violation — every sample that met
        // the plate kept the rules — but it is reported: as issues (for
        // the marks), as one contiguous span past x = 0.2, and in the
        // on-target ratio.
        assert!(report.ok());
        assert!(!report.issues.is_empty());
        assert!(report
            .issues
            .iter()
            .all(|i| i.kind == PaintIssueKind::NoTarget));
        let spans = report.spans(PaintIssueKind::NoTarget);
        assert_eq!(spans.len(), 1, "{spans:?}");
        assert!(report.hits > 0 && report.hits < report.probes.len());
        assert_eq!(report.in_band_ratio, 1.0);
        assert!(report.on_target_ratio < 1.0 && report.on_target_ratio > 0.5);
    }

    #[test]
    fn a_program_that_never_meets_the_part_is_not_ok() {
        let scene = scene_with_plate([0.4, 0.4, 0.01], Isometry3::translation(2.0, 0.0, -0.005));
        let report = check_paint(
            &scene,
            &stroke(0.25, -0.15, 0.15),
            "plate",
            &PaintLimits::default(),
            &ToolpathOptions::default(),
        )
        .unwrap();
        assert!(!report.ok());
        assert_eq!(report.hits, 0);
        assert_eq!(report.on_target_ratio, 0.0);
    }

    #[test]
    fn distance_and_angle_are_judged_by_kind() {
        // Too far: plate 0.4 m down against a 0.2-0.3 band.
        let far = scene_with_plate([0.4, 0.4, 0.01], Isometry3::translation(0.0, 0.0, -0.155));
        let limits = PaintLimits {
            standoff: Some((0.20, 0.30)),
            max_incidence: 0.3,
            ..PaintLimits::default()
        };
        let report = check_paint(
            &far,
            &stroke(0.25, -0.1, 0.1),
            "plate",
            &limits,
            &ToolpathOptions::default(),
        )
        .unwrap();
        assert!(report
            .issues
            .iter()
            .all(|i| i.kind == PaintIssueKind::TooFar));
        assert!((report.issues[0].value - 0.40).abs() < 1e-6);

        // Too close: plate 0.1 m down.
        let near = scene_with_plate([0.4, 0.4, 0.01], Isometry3::translation(0.0, 0.0, 0.145));
        let report = check_paint(
            &near,
            &stroke(0.25, -0.1, 0.1),
            "plate",
            &limits,
            &ToolpathOptions::default(),
        )
        .unwrap();
        assert!(report
            .issues
            .iter()
            .all(|i| i.kind == PaintIssueKind::TooClose));

        // Oblique: the plate tilted 35 degrees under a vertical gun.
        let tilted = scene_with_plate(
            [0.6, 0.6, 0.01],
            Isometry3::from_parts(
                nalgebra::Translation3::new(0.0, 0.0, -0.005),
                nalgebra::UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.6),
            ),
        );
        let report = check_paint(
            &tilted,
            &stroke(0.25, -0.05, 0.05),
            "plate",
            &PaintLimits {
                standoff: None,
                max_incidence: 0.3,
                ..PaintLimits::default()
            },
            &ToolpathOptions::default(),
        )
        .unwrap();
        assert!(!report.issues.is_empty());
        assert!(report
            .issues
            .iter()
            .all(|i| i.kind == PaintIssueKind::Oblique));
        assert!(
            (report.incidence_max - 0.6).abs() < 0.02,
            "{}",
            report.incidence_max
        );
    }

    #[test]
    fn the_baked_check_reads_the_same_geometry() {
        // A parked gun over the plate, checked off a hand-built timeline:
        // the FK-driven probe has to agree with the authored one.
        let arm = RobotModel::from_urdf_str(ARM).unwrap();
        let tool = RobotModel::from_urdf_str(include_str!("../../../examples/assets/spindle.urdf"))
            .unwrap();
        let robot = arm
            .attach_tool(
                &tool,
                Some("tool0"),
                None,
                Isometry3::identity(),
                None,
                None,
            )
            .unwrap();
        let mut scene = Scene::new(Arc::new(robot));
        let q = vec![0.0, 0.5, 0.9, std::f64::consts::PI - 1.4, 0.0, 0.0];
        scene.set_joint_positions(q.clone()).unwrap();
        let tcp = scene.robot().default_tcp_link();
        let tip = scene.link_poses()[tcp].translation.vector;
        scene
            .add_obstacle(
                "plate",
                Geometry::Box {
                    size: Vector3::new(0.3, 0.3, 0.01),
                },
                Isometry3::translation(tip.x, tip.y, tip.z - 0.25 - 0.005),
            )
            .unwrap();
        let mut timeline = scene.timeline_from_trajectory(
            0,
            &botrail_traj::JointTrajectory {
                times: vec![0.0, 1.0],
                positions: vec![q.clone(), q],
                velocities: vec![vec![0.0; 6], vec![0.0; 6]],
            },
            "hold",
        );
        timeline.signals.push(crate::rollout::BoolTrack {
            name: "gun".into(),
            edges: vec![(0.0, true), (0.5, false)],
            kind: crate::rollout::LaneKind::Signal,
        });
        let limits = PaintLimits {
            standoff: Some((0.20, 0.30)),
            ..PaintLimits::default()
        };
        let report = timeline_paint_report(
            &scene,
            &timeline,
            "plate",
            0,
            tcp,
            Some("gun"),
            0.1,
            &limits,
        )
        .unwrap();
        assert!(report.ok(), "{:?}", report.issues.first());
        assert!(
            (report.standoff_mean - 0.25).abs() < 1e-6,
            "{}",
            report.standoff_mean
        );
        // Gated: only the first half second was probed.
        assert!(report.probes.iter().all(|p| p.at <= 0.5 + 1e-9));
        assert!(report.probes.len() >= 5);
        assert!(matches!(
            timeline_paint_report(
                &scene,
                &timeline,
                "plate",
                0,
                tcp,
                Some("nope"),
                0.1,
                &limits
            ),
            Err(CoatError::UnknownGate(_))
        ));
    }

    /// Naming the face fixes the denominator: with `facing` set to the
    /// top, the rim can never enter the addressed set however far the gun
    /// overtravels, and the film statistics stop depending on the path.
    #[test]
    fn naming_the_face_makes_the_statistics_path_independent() {
        let arm = RobotModel::from_urdf_str(ARM).unwrap();
        let tool = RobotModel::from_urdf_str(include_str!("../../../examples/assets/spindle.urdf"))
            .unwrap();
        let robot = arm
            .attach_tool(
                &tool,
                Some("tool0"),
                None,
                Isometry3::identity(),
                None,
                None,
            )
            .unwrap();
        let mut scene = Scene::new(Arc::new(robot));
        let q = vec![0.0, 0.5, 0.9, std::f64::consts::PI - 1.4, 0.0, 0.0];
        scene.set_joint_positions(q.clone()).unwrap();
        let tcp = scene.robot().default_tcp_link();
        let tip = scene.link_poses()[tcp].translation.vector;
        // A thick plate, so the rim is a real face, parked well off to
        // one side so the gun addresses it at a shallow angle.
        scene
            .add_obstacle(
                "plate",
                Geometry::Box {
                    size: Vector3::new(0.2, 0.2, 0.06),
                },
                Isometry3::translation(tip.x + 0.25, tip.y, tip.z - 0.25 - 0.03),
            )
            .unwrap();
        let timeline = scene.timeline_from_trajectory(
            0,
            &botrail_traj::JointTrajectory {
                times: vec![0.0, 1.0],
                positions: vec![q.clone(), q],
                velocities: vec![vec![0.0; 6], vec![0.0; 6]],
            },
            "hold",
        );
        let gun = Applicator {
            standoff: 0.25,
            pattern: Pattern::Round {
                diameter: 0.20,
                beta: 2.0,
            },
            flow: 200e-6,
            transfer_efficiency: 0.80,
            max_range: 0.80,
        };
        let coat = |facing: Option<Vector3<f64>>| {
            spray_coat(
                &scene,
                &timeline,
                "plate",
                0,
                tcp,
                Some(&gun),
                &CoatOptions {
                    patch_size: 0.005,
                    facing,
                    facing_tolerance: 0.2,
                    ..CoatOptions::default()
                },
            )
            .unwrap()
        };
        let any = coat(None);
        let top = coat(Some(Vector3::z()));
        // Unnamed, the near rim (facing the gun at ~45 degrees) is in the
        // addressed set alongside the top; named, only the top is.
        assert!(
            any.total_area > 0.2 * 0.2 + 1e-6,
            "rim not addressed: {}",
            any.total_area
        );
        assert!(
            (top.total_area - 0.2 * 0.2).abs() < 1e-9,
            "top only: {}",
            top.total_area
        );
        // Same paint either way — the mask is reporting, not physics.
        assert_eq!(any.deposited_volume, top.deposited_volume);
    }
}

#[cfg(test)]
mod mesh_target_tests {
    use super::*;
    use botrail_model::RobotModel;
    use std::sync::Arc;

    const ARM: &str = include_str!("../../../examples/simple_arm.urdf");
    const SPINDLE: &str = include_str!("../../../examples/assets/spindle.urdf");

    /// A mesh target must take paint like a primitive one does — the
    /// self-shadow ray has to know the patch's own triangle from something
    /// in front of it. (The gap that let an exact-surface probe zero out
    /// every film on a mesh: with the surface exact, the patch sits at the
    /// ray's end to within rounding.)
    #[test]
    fn a_mesh_plate_takes_the_same_paint_as_a_box() {
        let arm = RobotModel::from_urdf_str(ARM).unwrap();
        let tool = RobotModel::from_urdf_str(SPINDLE).unwrap();
        let robot = Arc::new(
            arm.attach_tool(
                &tool,
                Some("tool0"),
                None,
                Isometry3::identity(),
                None,
                None,
            )
            .unwrap(),
        );
        let q = vec![0.0, 0.5, 0.9, std::f64::consts::PI - 1.4, 0.0, 0.0];
        let gun = Applicator {
            standoff: 0.25,
            pattern: Pattern::Round {
                diameter: 0.20,
                beta: 2.0,
            },
            flow: 200e-6,
            transfer_efficiency: 0.80,
            max_range: 0.60,
        };
        let coat = |geometry: Geometry| {
            let mut scene = Scene::new(Arc::clone(&robot));
            scene.set_joint_positions(q.clone()).unwrap();
            let tcp = scene.robot().default_tcp_link();
            let tip = scene.link_poses()[tcp].translation.vector;
            scene
                .add_obstacle(
                    "plate",
                    geometry,
                    Isometry3::translation(tip.x, tip.y, tip.z - 0.25 - 0.005),
                )
                .unwrap();
            let timeline = scene.timeline_from_trajectory(
                0,
                &botrail_traj::JointTrajectory {
                    times: vec![0.0, 1.0],
                    positions: vec![q.clone(), q.clone()],
                    velocities: vec![vec![0.0; 6], vec![0.0; 6]],
                },
                "hold",
            );
            spray_coat(
                &scene,
                &timeline,
                "plate",
                0,
                tcp,
                Some(&gun),
                &CoatOptions {
                    patch_size: 0.005,
                    ..CoatOptions::default()
                },
            )
            .unwrap()
        };
        // The same plate twice: as a box, and as that box's mesh written
        // to disk and read back through the mesh path (occlusion on).
        let size = Vector3::new(0.3, 0.3, 0.01);
        let as_box = coat(Geometry::Box { size });
        let dir = std::env::temp_dir().join("botrail-coat-mesh-test");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("plate.obj");
        let (obj, _) =
            botrail_mesh::to_obj_with_mtl(&botrail_mesh::box_mesh([0.3, 0.3, 0.01]), "plate.mtl");
        std::fs::write(&path, obj).unwrap();
        let as_mesh = coat(Geometry::Mesh {
            path,
            scale: Vector3::new(1.0, 1.0, 1.0),
        });
        assert!(
            as_mesh.deposited_volume > 0.0,
            "the mesh plate took no paint"
        );
        let ratio = as_mesh.deposited_volume / as_box.deposited_volume;
        assert!(
            (ratio - 1.0).abs() < 0.02,
            "mesh {:.4e} vs box {:.4e} ({ratio:.4})",
            as_mesh.deposited_volume,
            as_box.deposited_volume
        );
        assert!((as_mesh.total_area - as_box.total_area).abs() < 1e-6);
    }
}

#[cfg(test)]
mod trigger_tests {
    use super::*;
    use crate::rollout::RolloutOptions;
    use crate::seq::{Action, Condition, Sequence, Step};
    use crate::toolpath::{PathTarget, ToolMove, ToolMoveKind, Toolpath};
    use botrail_model::RobotModel;
    use nalgebra::Unit;
    use std::sync::Arc;

    const ARM: &str = include_str!("../../../examples/simple_arm.urdf");
    const SPINDLE: &str = include_str!("../../../examples/assets/spindle.urdf");

    /// The authoring trap: a sequence that opens the gun in the same step
    /// it starts the toolpath. The rollout plans a joint-space approach in
    /// from wherever the robot stood — at joint speed, straight across the
    /// part — and a film that followed the signal alone would paint the
    /// approach. It must follow the program's feed strokes too.
    #[test]
    fn the_approach_into_a_toolpath_does_not_spray() {
        let arm = RobotModel::from_urdf_str(ARM).unwrap();
        let tool = RobotModel::from_urdf_str(SPINDLE).unwrap();
        let robot = Arc::new(
            arm.attach_tool(
                &tool,
                Some("tool0"),
                None,
                Isometry3::identity(),
                None,
                None,
            )
            .unwrap(),
        );
        let mut scene = Scene::new(robot);
        // A stroke authored around a flange-down pose over a plate.
        let work_q = vec![0.0, 0.5, 0.9, std::f64::consts::PI - 1.4, 0.0, 0.0];
        scene.set_joint_positions(work_q.clone()).unwrap();
        let tcp = scene.robot().default_tcp_link();
        let tip = scene.link_poses()[tcp].translation.vector;
        scene
            .add_obstacle(
                "plate",
                Geometry::Box {
                    size: Vector3::new(0.30, 0.30, 0.01),
                },
                Isometry3::translation(tip.x, tip.y, tip.z - 0.25 - 0.005),
            )
            .unwrap();
        let target = |x: f64| PathTarget {
            position: Point3::new(x, tip.y, tip.z),
            tool_axis: Unit::new_normalize(Vector3::z()),
            spin: None,
        };
        scene.add_toolpath(Toolpath {
            name: "stroke".into(),
            frame: None,
            moves: vec![
                ToolMove {
                    kind: ToolMoveKind::Rapid,
                    targets: vec![target(tip.x - 0.05)],
                    brush: None,
                },
                ToolMove {
                    kind: ToolMoveKind::Feed(0.05),
                    targets: vec![target(tip.x + 0.05)],
                    brush: None,
                },
            ],
        });
        // Park the robot elsewhere, so the fire has to plan an approach.
        let park = vec![0.4, 0.4, 1.0, std::f64::consts::PI - 1.4, 0.0, 0.0];
        scene.set_joint_positions(park).unwrap();
        scene.define_signal("gun_on", false);
        let step = |name: &str, actions: Vec<Action>, transition: Condition| Step {
            name: name.to_string(),
            actions,
            transition,
            select: Vec::new(),
        };
        scene.upsert_sequence(Sequence {
            name: "cycle".into(),
            steps: vec![
                step(
                    "spray",
                    vec![
                        Action::Set {
                            signal: "gun_on".into(),
                            value: true,
                        },
                        Action::StartToolpath {
                            robot: None,
                            toolpath: "stroke".into(),
                        },
                    ],
                    Condition::Done,
                ),
                step(
                    "close",
                    vec![Action::Set {
                        signal: "gun_on".into(),
                        value: false,
                    }],
                    Condition::Elapsed { seconds: 0.2 },
                ),
            ],
        });
        let timeline = scene
            .simulate_sequence("cycle", &RolloutOptions::default())
            .unwrap();
        let spans = timeline.process_spans(0).expect("a toolpath ran");
        assert_eq!(spans.len(), 1, "{spans:?}");
        // The feed span is the 0.1 m stroke at 0.05 m/s: two seconds,
        // starting after the approach.
        assert!(spans[0].start > 0.5, "the approach takes time: {spans:?}");
        assert!(
            (spans[0].end - spans[0].start - 2.0).abs() < 0.3,
            "{spans:?}"
        );
        let on = timeline
            .signals
            .iter()
            .find(|s| s.name == "gun_on")
            .unwrap();
        assert!(
            on.value_at(0.1),
            "the gun was opened with the start command"
        );

        let gun = Applicator {
            standoff: 0.25,
            pattern: Pattern::Round {
                diameter: 0.10,
                beta: 2.0,
            },
            flow: 100e-6,
            transfer_efficiency: 0.8,
            max_range: 0.6,
        };
        let film = spray_coat(
            &scene,
            &timeline,
            "plate",
            0,
            tcp,
            Some(&gun),
            &CoatOptions {
                patch_size: 0.005,
                gate: Some("gun_on".into()),
                ..CoatOptions::default()
            },
        )
        .unwrap();
        // Paint is charged for the feed stroke only, not the approach.
        assert!(
            (film.gun_on_time - (spans[0].end - spans[0].start)).abs() < 0.05,
            "gun on {}s, feed span {:?}",
            film.gun_on_time,
            spans[0]
        );
        // And the baked standoff check probes the same interval.
        let report = timeline_paint_report(
            &scene,
            &timeline,
            "plate",
            0,
            tcp,
            Some("gun_on"),
            0.05,
            &PaintLimits {
                standoff: Some((0.2, 0.3)),
                ..PaintLimits::default()
            },
        )
        .unwrap();
        assert!(report
            .probes
            .iter()
            .all(|p| p.at >= spans[0].start - 1e-9 && p.at <= spans[0].end + 1e-9));
        assert!(report.ok(), "{:?}", report.issues.first());
    }
}

#[cfg(test)]
mod brush_tests {
    use super::*;
    use crate::rollout::RolloutOptions;
    use crate::seq::{Action, Condition, Sequence, Step};
    use crate::toolpath::{PathTarget, ToolMove, ToolMoveKind, Toolpath};
    use botrail_model::RobotModel;
    use nalgebra::Unit;
    use std::sync::Arc;

    const ARM: &str = include_str!("../../../examples/simple_arm.urdf");
    const SPINDLE: &str = include_str!("../../../examples/assets/spindle.urdf");

    fn gun_robot() -> Arc<RobotModel> {
        let arm = RobotModel::from_urdf_str(ARM).unwrap();
        let tool = RobotModel::from_urdf_str(SPINDLE).unwrap();
        Arc::new(
            arm.attach_tool(
                &tool,
                Some("tool0"),
                None,
                Isometry3::identity(),
                None,
                None,
            )
            .unwrap(),
        )
    }

    const DOWN_Q: [f64; 6] = [0.0, 0.5, 0.9, std::f64::consts::PI - 1.4, 0.0, 0.0];

    fn round(flow: f64) -> Applicator {
        Applicator {
            standoff: 0.25,
            pattern: Pattern::Round {
                diameter: 0.10,
                beta: 2.0,
            },
            flow,
            transfer_efficiency: 0.8,
            max_range: 0.6,
        }
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    /// A plate under the parked gun, and a two-stroke program over it:
    /// stroke, side-step, stroke back. `brushes`: the brush of each of the
    /// three feed moves (`None` = gun off / legacy).
    fn cell(brushes: [Option<&str>; 3]) -> (Scene, usize) {
        let mut scene = Scene::new(gun_robot());
        scene.set_joint_positions(DOWN_Q.to_vec()).unwrap();
        let tcp = scene.robot().default_tcp_link();
        let tip = scene.link_poses()[tcp].translation.vector;
        scene
            .add_obstacle(
                "plate",
                Geometry::Box {
                    size: Vector3::new(0.30, 0.30, 0.01),
                },
                Isometry3::translation(tip.x, tip.y, tip.z - 0.25 - 0.005),
            )
            .unwrap();
        let target = |x: f64, y: f64| PathTarget {
            position: Point3::new(tip.x + x, tip.y + y, tip.z),
            tool_axis: Unit::new_normalize(Vector3::z()),
            spin: None,
        };
        let feed = |b: Option<&str>, targets: Vec<PathTarget>| ToolMove {
            kind: ToolMoveKind::Feed(0.05),
            targets,
            brush: b.map(str::to_string),
        };
        scene.add_toolpath(Toolpath {
            name: "raster".into(),
            frame: None,
            moves: vec![
                ToolMove {
                    kind: ToolMoveKind::Rapid,
                    targets: vec![target(-0.05, -0.03)],
                    brush: None,
                },
                feed(brushes[0], vec![target(0.05, -0.03)]),
                feed(brushes[1], vec![target(0.05, 0.03)]),
                feed(brushes[2], vec![target(-0.05, 0.03)]),
            ],
        });
        scene.define_signal("gun_on", false);
        scene.upsert_sequence(Sequence {
            name: "cycle".into(),
            steps: vec![
                step(
                    "spray",
                    vec![
                        Action::Set {
                            signal: "gun_on".into(),
                            value: true,
                        },
                        Action::StartToolpath {
                            robot: None,
                            toolpath: "raster".into(),
                        },
                    ],
                    Condition::Done,
                ),
                step(
                    "close",
                    vec![Action::Set {
                        signal: "gun_on".into(),
                        value: false,
                    }],
                    Condition::Elapsed { seconds: 0.2 },
                ),
            ],
        });
        (scene, tcp)
    }

    fn coat(scene: &Scene, tcp: usize, applicator: Option<&Applicator>) -> FilmCoat {
        let timeline = scene
            .simulate_sequence("cycle", &RolloutOptions::default())
            .unwrap();
        spray_coat(
            scene,
            &timeline,
            "plate",
            0,
            tcp,
            applicator,
            &CoatOptions {
                patch_size: 0.005,
                gate: Some("gun_on".into()),
                ..CoatOptions::default()
            },
        )
        .unwrap()
    }

    #[test]
    fn brushes_trigger_per_stroke_and_the_side_step_runs_dry() {
        // Legacy: no brushes, everything sprays with the applicator given.
        let (scene, tcp) = cell([None, None, None]);
        let all_on = coat(&scene, tcp, Some(&round(100e-6)));
        // Brushed: strokes spray, the side-step (no brush) does not.
        let (mut scene, tcp) = cell([Some("base"), None, Some("base")]);
        scene.define_applicator("bell", round(100e-6)).unwrap();
        scene
            .define_brush(Brush {
                name: "base".into(),
                applicator: "bell".into(),
                flow: 1.0,
                lead: 0.0,
                lag: 0.0,
            })
            .unwrap();
        let strokes_only = coat(&scene, tcp, None);
        // The side-step is 6 cm of 26: about a quarter less gun-on time.
        let ratio = strokes_only.gun_on_time / all_on.gun_on_time;
        assert!((ratio - 20.0 / 26.0).abs() < 0.05, "gun on ratio {ratio}");
        assert!(strokes_only.sprayed_volume < all_on.sprayed_volume);
        assert_eq!(strokes_only.sprayed_by_brush.len(), 1);
        assert_eq!(strokes_only.sprayed_by_brush[0].0, "base");
        assert!((strokes_only.sprayed_by_brush[0].1 - strokes_only.sprayed_volume).abs() < 1e-15);
        // Without a brush anywhere and no applicator handed in, there is
        // nothing to spray with.
        let (scene, tcp) = cell([None, None, None]);
        let timeline = scene
            .simulate_sequence("cycle", &RolloutOptions::default())
            .unwrap();
        assert!(matches!(
            spray_coat(
                &scene,
                &timeline,
                "plate",
                0,
                tcp,
                None,
                &CoatOptions::default()
            ),
            Err(CoatError::NoApplicator)
        ));
    }

    #[test]
    fn two_brushes_are_accounted_separately_and_flow_scales() {
        let (mut scene, tcp) = cell([Some("primer"), None, Some("top")]);
        scene.define_applicator("bell", round(100e-6)).unwrap();
        for (name, flow) in [("primer", 0.5), ("top", 1.0)] {
            scene
                .define_brush(Brush {
                    name: name.into(),
                    applicator: "bell".into(),
                    flow,
                    lead: 0.0,
                    lag: 0.0,
                })
                .unwrap();
        }
        let film = coat(&scene, tcp, None);
        let by: std::collections::HashMap<_, _> = film.sprayed_by_brush.iter().cloned().collect();
        // Same stroke length, half the flow: half the paint.
        let ratio = by["primer"] / by["top"];
        assert!((ratio - 0.5).abs() < 0.02, "primer/top sprayed {ratio}");
        let dep: std::collections::HashMap<_, _> =
            film.deposited_by_brush.iter().cloned().collect();
        assert!((dep["primer"] / dep["top"] - 0.5).abs() < 0.03);
        let total: f64 = film.deposited_by_brush.iter().map(|(_, v)| v).sum();
        assert!((total - film.deposited_volume).abs() / film.deposited_volume < 1e-9);
    }

    #[test]
    fn lead_and_lag_widen_the_strokes() {
        let (mut scene, tcp) = cell([Some("b"), None, Some("b")]);
        scene.define_applicator("bell", round(100e-6)).unwrap();
        let brush = |lead: f64, lag: f64| Brush {
            name: "b".into(),
            applicator: "bell".into(),
            flow: 1.0,
            lead,
            lag,
        };
        scene.define_brush(brush(0.0, 0.0)).unwrap();
        let tight = coat(&scene, tcp, None);
        scene.define_brush(brush(0.2, 0.3)).unwrap();
        let wide = coat(&scene, tcp, None);
        // Two strokes, each 0.5 s wider — except that the second stroke's
        // lag runs into the `close` step, where the enable drops: the PLC
        // still has the last word, so 0.2 + 0.3 + 0.2 and not a full
        // second. The first stroke's lead reaches back into the approach
        // (enable high, so it sprays: that is what lead is for).
        let extra = wide.gun_on_time - tight.gun_on_time;
        assert!((extra - 0.7).abs() < 0.1, "lead+lag added {extra}s");
        assert!(wide.deposited_volume > tight.deposited_volume);
        // Bad timings are refused at declaration.
        assert!(scene.define_brush(brush(-0.1, 0.0)).is_err());
        assert!(scene
            .define_brush(Brush {
                name: "x".into(),
                applicator: "nope".into(),
                flow: 1.0,
                lead: 0.0,
                lag: 0.0,
            })
            .is_err());
    }

    /// A mask over part of the plate: the masked patches take no paint
    /// and are not holidays (they were never addressed), the mask itself
    /// shows up in the overspray by name, and the accounting closes.
    #[test]
    fn a_mask_shadows_the_part_and_takes_the_overspray() {
        let (mut scene, tcp) = cell([None, None, None]);
        let unmasked = coat(&scene, tcp, Some(&round(100e-6)));
        let tip_z = scene.link_poses()[tcp].translation.vector.z;
        let tip = scene.link_poses()[tcp].translation.vector;
        // A strip 3 cm above the plate, across the middle of both strokes.
        scene
            .add_obstacle(
                "mask",
                Geometry::Box {
                    size: Vector3::new(0.03, 0.30, 0.004),
                },
                Isometry3::translation(tip.x, tip.y, tip_z - 0.22),
            )
            .unwrap();
        let masked = coat(&scene, tcp, Some(&round(100e-6)));
        // Less on the part, the difference on the mask (roughly — the
        // strip's shadow is a little narrower than its projection at
        // grazing incidence, and the ray quadrature is coarse).
        assert!(masked.deposited_volume < unmasked.deposited_volume * 0.95);
        let on_mask = masked
            .overspray
            .iter()
            .find(|(n, _)| n == "mask")
            .map(|(_, v)| *v)
            .unwrap_or(0.0);
        assert!(
            on_mask > 0.0,
            "the mask took no paint: {:?}",
            masked.overspray
        );
        let lost_delta = unmasked.deposited_volume - masked.deposited_volume;
        assert!(
            (on_mask - lost_delta).abs() < 0.3 * lost_delta,
            "mask took {on_mask:.3e}, part lost {lost_delta:.3e}"
        );
        // Masked patches are not holidays: they leave the addressed set
        // instead — and since this small program leaves much of the plate
        // addressed-but-bare, the strip takes some of that with it too.
        assert!(masked.total_area < unmasked.total_area);
        assert!(masked.uncoated_area <= unmasked.uncoated_area);
        assert!(masked.uncoated_area > 0.9 * unmasked.uncoated_area);
        // The books close: sprayed = on part + overspray + lost.
        let overspray: f64 = masked.overspray.iter().map(|(_, v)| v).sum();
        assert!(
            (masked.sprayed_volume - masked.deposited_volume - overspray - masked.lost_volume)
                .abs()
                < 1e-12
        );
        // And the atomization loss alone is a fifth of what was sprayed.
        assert!(masked.lost_volume >= 0.2 * masked.sprayed_volume - 1e-12);
    }
}

#[cfg(test)]
mod palette_tests {
    use super::*;
    use botrail_model::RobotModel;
    use std::sync::Arc;

    const ARM: &str = include_str!("../../../examples/simple_arm.urdf");
    const SPINDLE: &str = include_str!("../../../examples/assets/spindle.urdf");

    fn coat_with(options: CoatOptions, plate_color: Option<[f32; 3]>) -> FilmCoat {
        let arm = RobotModel::from_urdf_str(ARM).unwrap();
        let tool = RobotModel::from_urdf_str(SPINDLE).unwrap();
        let robot = arm
            .attach_tool(
                &tool,
                Some("tool0"),
                None,
                Isometry3::identity(),
                None,
                None,
            )
            .unwrap();
        let mut scene = Scene::new(Arc::new(robot));
        let q = vec![0.0, 0.5, 0.9, std::f64::consts::PI - 1.4, 0.0, 0.0];
        scene.set_joint_positions(q.clone()).unwrap();
        let tcp = scene.robot().default_tcp_link();
        let tip = scene.link_poses()[tcp].translation.vector;
        scene
            .add_obstacle(
                "plate",
                Geometry::Box {
                    size: Vector3::new(0.3, 0.3, 0.01),
                },
                Isometry3::translation(tip.x, tip.y, tip.z - 0.25 - 0.005),
            )
            .unwrap();
        scene.set_obstacle_color("plate", plate_color).unwrap();
        let timeline = scene.timeline_from_trajectory(
            0,
            &botrail_traj::JointTrajectory {
                times: vec![0.0, 1.0],
                positions: vec![q.clone(), q],
                velocities: vec![vec![0.0; 6], vec![0.0; 6]],
            },
            "hold",
        );
        let gun = Applicator {
            standoff: 0.25,
            pattern: Pattern::Round {
                diameter: 0.20,
                beta: 2.0,
            },
            flow: 200e-6,
            transfer_efficiency: 0.8,
            max_range: 0.6,
        };
        spray_coat(&scene, &timeline, "plate", 0, tcp, Some(&gun), &options).unwrap()
    }

    /// The two readings of one film: by amount (one hue, light to dark,
    /// or a wash of the paint's own colour) and against spec (diverging).
    /// Bare patches wear the part's own colour when it has one.
    #[test]
    fn styles_paint_colour_and_substrate() {
        let grey = [0.62f32, 0.63, 0.66];
        let plain = coat_with(
            CoatOptions {
                patch_size: 0.005,
                ..CoatOptions::default()
            },
            Some(grey),
        );
        assert_eq!(plain.palette.style, FilmStyle::Amount);
        assert_eq!(
            plain.palette.uncoated, grey,
            "bare patches wear the part's colour"
        );
        assert!((plain.palette.top - plain.max).abs() < 1e-15);
        // A parked round gun: a disc of paint in a sea of substrate.
        let bare = plain
            .mesh
            .face_colors
            .iter()
            .filter(|c| **c == grey)
            .count();
        assert!(bare > plain.mesh.face_colors.len() / 2);
        assert!(plain.mesh.face_colors.iter().any(|c| *c != grey));

        // With a spec, `Auto` is the verdict map: on-target neutral, thin
        // blue, thick red, and the ramp's top is the spec's high edge
        // whichever style is asked for.
        let judged = coat_with(
            CoatOptions {
                patch_size: 0.005,
                spec: Some((20e-6, 30e-6)),
                ..CoatOptions::default()
            },
            None,
        );
        assert_eq!(judged.palette.style, FilmStyle::Spec);
        assert_eq!(judged.palette.uncoated, uncoated_color());
        let by_amount = coat_with(
            CoatOptions {
                patch_size: 0.005,
                spec: Some((20e-6, 30e-6)),
                style: FilmStyle::Amount,
                ..CoatOptions::default()
            },
            None,
        );
        assert_eq!(by_amount.palette.style, FilmStyle::Amount);
        assert!((by_amount.palette.top - 30e-6).abs() < 1e-15);
        assert_ne!(judged.mesh.face_colors, by_amount.mesh.face_colors);

        // A paint colour: a wash from a light tint to the colour itself,
        // ten steps, monotone in every channel.
        let red = [0.72f32, 0.10, 0.06];
        let painted = coat_with(
            CoatOptions {
                patch_size: 0.005,
                paint_color: Some(red),
                ..CoatOptions::default()
            },
            Some(grey),
        );
        let ramp = &painted.palette.ramp;
        assert_eq!(ramp.len(), 10);
        for (got, want) in ramp.last().unwrap().iter().zip(&red) {
            assert!((got - want).abs() < 1e-6);
        }
        for w in ramp.windows(2) {
            for (next, prev) in w[1].iter().zip(&w[0]) {
                assert!(*next <= prev + 1e-6, "wash darkens step by step");
            }
        }
        assert!(
            ramp[0][1] > 0.6,
            "the first step is a light tint: {:?}",
            ramp[0]
        );
        // And the legend follows the palette: top label, uncoated last.
        let legend = film_legend(&painted);
        assert_eq!(legend.last().unwrap().1, "uncoated");
        assert_eq!(legend.last().unwrap().0, grey);
        assert_eq!(legend.len(), 11);
        let with_spec = film_legend(&by_amount);
        assert!(with_spec[0].1.ends_with('+'), "{:?}", with_spec[0]);
    }
}

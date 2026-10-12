//! The rope's shape as an optimization, not a simulation.
//!
//! At every frame of the bake the line is solved to rest: an inextensible
//! rod held where the pass holds it (a position — and the direction, where
//! two or more samples are held), of least bending and gravitational
//! energy — a flexural rigidity and a weight per metre, not spring
//! frequencies — and clear of the bodies around it. Nothing carries from
//! one frame to the next but the shape the solve starts from, so the shape
//! at a frame does not depend on how fast the cycle ran to it: what a
//! harness configurator draws, in place of the replay's dynamics, on the
//! same track and under the same checks. The solve is a dynamic
//! relaxation run to a tolerance — the bending forces of the rod's
//! rigidity and its weight stepped explicitly, its length and its
//! contacts projected, the motion damped kinetically — so the rigidity
//! sets the balance of bend against weight, the only material figure the
//! picture needs.
use crate::{
    pass::{bake, Bake, RopePass},
    shape_bridge, Error, RopeLocation, RopeTrack,
};
use botrail_scene::{rollout::SequenceTimeline, Scene};
use rapier_rope::rapier::parry::{
    bounding_volume::{Aabb, BoundingVolume},
    shape::SharedShape,
};
use rapier_rope::rapier::prelude::{Pose, Vector};
use std::collections::BTreeMap;

/// The solve's settings: the rod's rigidity and how far it is run.
#[derive(Debug, Clone, PartialEq)]
pub struct StaticSettings {
    /// Flexural rigidity EI, N·m² — what resists the bend against the
    /// line's weight. A 6 × 4 mm polyurethane tube is about 1.5e-3; it
    /// goes with the fourth power of the diameter, so a 10 mm robot cable
    /// is about ten times that.
    pub rigidity_nm2: f64,
    /// Frames a second on the track.
    pub record_hz: f64,
    /// A frame is at rest when no sample moved farther than this in one
    /// round of the solve.
    pub tolerance_m: f64,
    /// Rounds of the relaxation (1/120 s of its own time each) a frame may
    /// take at most; the first frame, which starts from the laid polyline,
    /// may take twice as many.
    pub rounds: usize,
    /// Torsional rigidity GJ, N·m² — what resists the twist between two
    /// holders that hold the line's roll. `None`: `rigidity_nm2 / 1.4` (a
    /// round section of a material with Poisson's ratio 0.4).
    pub torsion_nm2: Option<f64>,
}

impl StaticSettings {
    /// A polyurethane tube's rigidity for a line of this radius: 1.5e-3
    /// N·m² at 6 mm, with the fourth power of the diameter.
    pub fn tube_rigidity(radius_m: f64) -> f64 {
        1.5e-3 * (radius_m / 0.003).powi(4)
    }
    pub fn new(rigidity_nm2: f64) -> Self {
        Self {
            rigidity_nm2,
            record_hz: 30.0,
            tolerance_m: 1e-5,
            rounds: 1000,
            torsion_nm2: None,
        }
    }
    /// The torsional rigidity in force.
    pub fn torsion(&self) -> f64 {
        self.torsion_nm2.unwrap_or(self.rigidity_nm2 / 1.4)
    }
}

/// A round of the relaxation, in its own time (s) — not the cycle's.
const ROUND_S: f64 = 1.0 / 120.0;
const GRAVITY: f64 = 9.81;
/// How far around a sample the bodies are looked up: a sample that has
/// moved half this far since is looked up again.
const REACH: f64 = 0.015;
/// How far a sample may move before the surface under it is asked again
/// rather than taken as the plane found last time.
const CONTACT_M: f64 = 3e-4;
/// Rounds without the motion slowing by a twentieth before the first
/// frame is taken as settled as it gets (a third of that for the frames
/// after it, which start from the one before).
const STALL_ROUNDS: usize = 150;
/// The velocities' factor a step, beside the kinetic damping: a bulge
/// sliding along a frictionless line at steady speed would otherwise
/// never slow.
const DAMPING: f64 = 0.999;
/// How far a body may move between two placings of the line on the way
/// to a frame (m), and the rounds the line gets to follow each placing.
const SWEEP_M: f64 = 0.002;
const SWEEP_ROUNDS: usize = 3;
/// Rounds a line settled against the others' new places is given: the
/// contacts are projected at once, the bend near them relaxes over these.
const SETTLE_ROUNDS: usize = 12;
/// Frames a line is settled on after another last pushed it, to find its
/// own rest again once the other has gone.
const SETTLE_AFTER: u8 = 3;

fn invalid(message: impl Into<String>) -> Error {
    Error::Input(message.into())
}

enum Hold {
    Body {
        proxy: usize,
        local: Vector,
    },
    Pin(Vector),
    /// A sample of a carriage on a rail: `offset` from the rail's start,
    /// in the body's frame.
    Carriage {
        carriage: usize,
        offset: Vector,
    },
}

/// A retracting holder: a carriage on a rail of a body, holding its
/// samples as a rigid cluster, free to run along the rail within its
/// stroke and pulled toward the start. The retraction is taken as strong
/// against the line's weight and stiffness (a retraction unit is): the
/// carriage creeps toward the start every step, and the line's length,
/// through the samples (which take part in the length constraints as the
/// line's own), pulls it out as far as the line can reach — the line
/// between the carriage and the next holder stays taut.
struct Carriage {
    proxy: usize,
    /// The rail's start and axis in the body's frame.
    base: Vector,
    axis: Vector,
    stroke: f64,
    /// Where along the rail the carriage is (m from the start).
    s: f64,
    samples: Vec<(usize, Vector)>,
}

/// How far a carriage creeps toward its rail's start in a step (m).
const RETRACT_M: f64 = 2e-5;

/// An end whose holder holds the line's roll: the material normal there,
/// in the holder body's frame.
struct RollEnd {
    proxy: usize,
    normal: Vector,
}

/// The twist of the rod between two roll-holding ends: the angle between
/// the start's normal carried along the centerline (parallel transport)
/// and the end's, about the end's tangent — `phi0` the angle the rod was
/// installed with (twist-free by definition), `phi` the twist now, on the
/// branch continuous with the last.
struct Twist {
    a: RollEnd,
    b: RollEnd,
    phi0: Option<f64>,
    phi: f64,
}

/// `angle` on the branch (± turns) nearest `near`.
fn unwrap_near(angle: f64, near: f64) -> f64 {
    let turn = std::f64::consts::TAU;
    angle + ((near - angle) / turn).round() * turn
}

/// `normal` carried from the unit direction `from` to `to` by the least
/// rotation between them.
fn transport(normal: Vector, from: Vector, to: Vector) -> Vector {
    let axis = from.cross(to);
    let s = axis.length();
    let c = from.dot(to);
    if s < 1e-12 {
        return normal;
    }
    let k = axis / s;
    normal * c + k.cross(normal) * s + k * (k.dot(normal) * (1.0 - c))
}

/// The signed angle, about the last edge, from the start's normal carried
/// along `x` to the end's normal (both given in the world).
fn holonomy(x: &[Vector], n_a: Vector, n_b: Vector) -> f64 {
    let edges: Vec<Vector> = x
        .windows(2)
        .map(|w| (w[1] - w[0]).normalize_or_zero())
        .collect();
    if edges.len() < 2 {
        return 0.0;
    }
    let project = |n: Vector, e: Vector| (n - e * n.dot(e)).normalize_or_zero();
    let mut n = project(n_a, edges[0]);
    for w in edges.windows(2) {
        n = transport(n, w[0], w[1]);
    }
    let last = edges[edges.len() - 1];
    let nb = project(n_b, last);
    n.cross(nb).dot(last).atan2(n.dot(nb))
}

/// ∂/∂x of the holonomy, by the discrete elastic rod's curvature binormal
/// (Bergou et al. 2008): at an interior sample, with the edges either side.
fn holonomy_gradient(x: &[Vector], grad: &mut [Vector]) {
    grad.iter_mut().for_each(|g| *g = Vector::ZERO);
    for i in 1..x.len().saturating_sub(1) {
        let e0 = x[i] - x[i - 1];
        let e1 = x[i + 1] - x[i];
        let (l0, l1) = (e0.length(), e1.length());
        let denom = l0 * l1 + e0.dot(e1);
        if l0 < 1e-12 || l1 < 1e-12 || denom.abs() < 1e-12 {
            continue;
        }
        // The carried normal turns with the holonomy; the angle read from
        // it to the end's normal shrinks by as much: the sign is theirs
        // reversed.
        let kb = e0.cross(e1) * (-2.0 / denom);
        let g_prev = kb / (2.0 * l0);
        let g_next = -kb / (2.0 * l1);
        grad[i - 1] += g_prev;
        grad[i + 1] += g_next;
        grad[i] -= g_prev + g_next;
    }
}

/// The pass's line shaped against the bake, frame by frame.
pub fn animate(
    scene: &Scene,
    timeline: &SequenceTimeline,
    pass: &RopePass,
    settings: &StaticSettings,
) -> Result<RopeTrack, Error> {
    shape(&bake(scene, timeline, pass)?, pass, settings)
}

/// One line's solve, kept across the frames: what `shape` sets up from
/// the bake and the pass, and the state the frames carry on.
struct Solve<'a> {
    bake: &'a Bake,
    pass: &'a RopePass,
    settings: &'a StaticSettings,
    holds: Vec<Option<Hold>>,
    carriages: Vec<Carriage>,
    shapes: Vec<SharedShape>,
    colliders: Vec<(Option<usize>, Pose)>,
    radius: f64,
    rod: Rod,
    times: Vec<f64>,
    track: RopeTrack,
    twist: Option<Twist>,
    still: Vec<bool>,
    holds_still: bool,
    /// The bodies' poses at the last frame solved: a frame is reached by
    /// sweeping the holders and the bodies from there in steps small
    /// enough that a ring the line runs through cannot jump past it.
    last_data: Option<Vec<crate::PoseData>>,
    /// Whether the last frame was solved rather than copied from the one
    /// before (a still line no moving body came near).
    moved_frame: bool,
    /// Frames left on which to settle this line again after another pushed
    /// on it, to find its own rest once the other has gone.
    settling_left: u8,
    rounds_taken: usize,
    debug: bool,
    started: std::time::Instant,
}

impl<'a> Solve<'a> {
    fn new(
        bake: &'a Bake,
        pass: &'a RopePass,
        settings: &'a StaticSettings,
    ) -> Result<Self, Error> {
        let duration = bake.duration_s;
        if !duration.is_finite() || duration < 0.0 {
            return Err(invalid("duration must be finite and nonnegative"));
        }
        if !(settings.rigidity_nm2.is_finite() && settings.rigidity_nm2 > 0.0) {
            return Err(invalid("rigidity_nm2 must be finite and positive"));
        }
        if !(settings.record_hz.is_finite() && settings.record_hz > 0.0)
            || !(settings.tolerance_m.is_finite() && settings.tolerance_m > 0.0)
            || settings.rounds == 0
        {
            return Err(invalid(
                "the solve needs a positive record rate, tolerance and rounds",
            ));
        }
        let mut proxies: BTreeMap<&str, usize> = BTreeMap::new();
        for (i, p) in bake.proxies.iter().enumerate() {
            if p.name.is_empty()
                || proxies.insert(p.name.as_str(), i).is_some()
                || p.times.is_empty()
                || p.times.len() != p.poses.len()
                || p.times[0] != 0.0
                || *p.times.last().unwrap() != duration
                || p.times.iter().any(|t| !t.is_finite())
                || p.times.windows(2).any(|w| w[0] >= w[1])
            {
                return Err(invalid(format!("invalid proxy clock/name {}", p.name)));
            }
            for pose in &p.poses {
                pose.native()?;
            }
        }

        // The laid line, sampled evenly along its length.
        let spec = &pass.spec;
        let line: Vec<Vector> = spec
            .reference_centerline_m
            .iter()
            .map(|p| Vector::from_array(*p))
            .collect();
        if line.len() < 2 || line.iter().any(|p| !p.is_finite()) {
            return Err(invalid("the line needs two or more finite points"));
        }
        let mut cumulative = vec![0.0];
        for w in line.windows(2) {
            cumulative.push(cumulative.last().unwrap() + (w[1] - w[0]).length());
        }
        let length = *cumulative.last().unwrap();
        let spacing = spec.sampling.max_segment_length_m;
        if !(length > 0.0) || !(spacing.is_finite() && spacing > 0.0) {
            return Err(invalid(
                "the line needs a positive length and sample spacing",
            ));
        }
        let n = ((length / spacing).ceil() as usize).max(1);
        let ds = length / n as f64;
        let at = |s: f64| -> Vector {
            let s = s.clamp(0.0, length);
            let k = cumulative
                .partition_point(|c| *c <= s)
                .saturating_sub(1)
                .min(line.len() - 2);
            let seg = cumulative[k + 1] - cumulative[k];
            let u = if seg > 0.0 {
                ((s - cumulative[k]) / seg).clamp(0.0, 1.0)
            } else {
                0.0
            };
            line[k] + (line[k + 1] - line[k]) * u
        };
        let x0: Vec<Vector> = (0..=n).map(|i| at(i as f64 * ds)).collect();
        let arc = |location: &RopeLocation| -> Result<f64, Error> {
            Ok(match location {
                RopeLocation::Start => 0.0,
                RopeLocation::End => length,
                RopeLocation::ArcLength { arc_length_m, .. } => *arc_length_m,
                RopeLocation::Named(name) => spec
                    .named_locations
                    .iter()
                    .find(|l| l.name == *name)
                    .map(|l| l.arc_length_m)
                    .ok_or_else(|| invalid(format!("unknown named location {name}")))?,
            })
        };
        let sample_at = |s: f64| ((s / ds).round().max(0.0) as usize).min(n);

        // What holds the line: an anchor holds every sample inside its span
        // where it lies at time zero (its offset in the body's frame), a pin
        // holds one sample in the world.
        let mut holds: Vec<Option<Hold>> = (0..=n).map(|_| None).collect();
        let mut carriages: Vec<Carriage> = Vec::new();
        let slack = 1e-9 * length.max(1.0);
        for a in &pass.anchors {
            let key = a.body.key();
            let &pi = proxies
                .get(key.as_str())
                .ok_or_else(|| invalid(format!("missing proxy {key}")))?;
            if !a.length_m.is_finite() || a.length_m < 0.0 {
                return Err(invalid(format!(
                    "anchor on {key} needs a finite, nonnegative length"
                )));
            }
            let s = arc(&a.location)?;
            let (lo, hi) = match a.location {
                RopeLocation::Start => (s, s + a.length_m),
                RopeLocation::End => (s - a.length_m, s),
                _ => (s - a.length_m / 2.0, s + a.length_m / 2.0),
            };
            let pose0 = bake.proxies[pi].poses[0].native()?;
            let mut members: Vec<usize> = (0..=n)
                .filter(|&i| {
                    let si = i as f64 * ds;
                    si >= lo - slack && si <= hi + slack
                })
                .collect();
            if members.is_empty() {
                members.push(sample_at(s));
            }
            match a.rail {
                None => {
                    for i in members {
                        holds[i] = Some(Hold::Body {
                            proxy: pi,
                            local: pose0.inverse_transform_point(x0[i]),
                        });
                    }
                }
                Some(rail) => {
                    let axis = pose0.inverse_transform_vector(Vector::from_array(rail.axis));
                    if axis.length() < 1e-9
                        || !rail.stroke_m.is_finite()
                        || !rail.force_n.is_finite()
                    {
                        return Err(invalid(format!("anchor on {key}: an invalid rail")));
                    }
                    let base = pose0.inverse_transform_point(x0[sample_at(s)]);
                    let index = carriages.len();
                    carriages.push(Carriage {
                        proxy: pi,
                        base,
                        axis: axis.normalize(),
                        stroke: rail.stroke_m.max(0.0),
                        s: 0.0,
                        samples: members
                            .iter()
                            .map(|&i| (i, pose0.inverse_transform_point(x0[i]) - base))
                            .collect(),
                    });
                    for &i in &members {
                        holds[i] = Some(Hold::Carriage {
                            carriage: index,
                            offset: pose0.inverse_transform_point(x0[i]) - base,
                        });
                    }
                }
            }
        }
        for (location, p) in &pass.pins {
            let p = Vector::from_array(*p);
            if !p.is_finite() {
                return Err(invalid("a pin needs a finite position"));
            }
            holds[sample_at(arc(location)?)] = Some(Hold::Pin(p));
        }
        // The twist, when both ends hold the roll: the material normal at each
        // end, square to the line there, kept in the holder body's frame.
        let roll_end = |location: &RopeLocation| -> Result<Option<RollEnd>, Error> {
            let Some(a) = pass
                .anchors
                .iter()
                .find(|a| a.roll && &a.location == location)
            else {
                return Ok(None);
            };
            let pi = proxies[a.body.key().as_str()];
            let pose0 = bake.proxies[pi].poses[0].native()?;
            let tangent = match location {
                RopeLocation::Start => x0[1.min(n)] - x0[0],
                _ => x0[n] - x0[n.saturating_sub(1)],
            };
            let t = tangent.normalize_or_zero();
            let seed = if t.x.abs() < 0.9 {
                Vector::X
            } else {
                Vector::Y
            };
            let normal = t.cross(seed).normalize_or_zero();
            Ok(Some(RollEnd {
                proxy: pi,
                normal: pose0.inverse_transform_vector(normal),
            }))
        };
        let twist = match (
            roll_end(&RopeLocation::Start)?,
            roll_end(&RopeLocation::End)?,
        ) {
            (Some(a), Some(b)) if n >= 2 => Some(Twist {
                a,
                b,
                phi0: None,
                phi: 0.0,
            }),
            _ => None,
        };

        // The bodies around: the proxies' shapes at their poses, the pass's
        // own static shapes, and the floor.
        let mut shapes: Vec<SharedShape> = Vec::new();
        let mut colliders: Vec<(Option<usize>, Pose)> = Vec::new();
        for (pi, p) in bake.proxies.iter().enumerate() {
            for (offset, shape) in &p.shapes {
                shapes.push(shape_bridge::reconstruct(shape)?);
                colliders.push((Some(pi), offset.native()?));
            }
        }
        for (pose, shape) in &pass.extra_colliders {
            shapes.push(shape_bridge::reconstruct(shape)?);
            colliders.push((None, pose.native()?));
        }
        if let Some(z) = pass.ground_z {
            if !z.is_finite() {
                return Err(invalid("ground_z must be finite"));
            }
        }

        let radius = spec.collision.radius_m;
        let mass = spec.material.linear_density_kg_m * ds;
        if !(radius.is_finite() && radius > 0.0) || !(mass.is_finite() && mass > 0.0) {
            return Err(invalid("the line needs a positive radius and density"));
        }
        // The explicit step must stay under the stiffest bending mode's
        // (about 4 √(EI/ρ) / ds²): a quarter of that limit.
        let density = spec.material.linear_density_kg_m;
        let step = (0.25 * ds * ds * (density / settings.rigidity_nm2).sqrt()).min(1e-4);
        let rod = Rod {
            x: x0,
            v: vec![Vector::ZERO; n + 1],
            // A carriage's samples take part in the constraints as the line's
            // own: the line's length pulls the carriage along its rail.
            w: holds
                .iter()
                .map(|h| match h {
                    None | Some(Hold::Carriage { .. }) => 1.0 / mass,
                    Some(_) => 0.0,
                })
                .collect(),
            ds,
            radius,
            stiffness: settings.rigidity_nm2 / ds,
            torsion: settings.torsion() / length,
            step,
            ground: pass.ground_z,
            pushes: 0,
        };

        let frames = (duration * settings.record_hz).ceil().max(0.0) as usize;
        let mut times: Vec<f64> = (0..frames).map(|k| k as f64 / settings.record_hz).collect();
        if times.last().is_none_or(|t| duration - t > 1e-9) {
            times.push(duration);
        }
        let track = RopeTrack {
            name: spec.name.clone(),
            segments: (0..n as u32).map(|i| [i, i + 1]).collect(),
            radius_m: radius,
            times: Vec::with_capacity(times.len()),
            points: Vec::with_capacity(times.len()),
            held: Vec::with_capacity(times.len()),
            events: Vec::new(),
            connectors: Vec::new(),
            coupling: "baked_one_way_no_source_reaction".into(),
            coordinate_system: "right-handed Z-up SI".into(),
            solver: "botrail static rod (least bending and gravitational energy)".into(),
            color: pass.color,
            twist_deg: twist.as_ref().map(|_| Vec::with_capacity(times.len())),
        };
        // A span whose holders stand still is solved again only when a body
        // that moves comes within reach of it.
        let still: Vec<bool> = bake
            .proxies
            .iter()
            .map(|p| {
                p.poses.iter().all(|q| {
                    (0..3).all(|i| (q.position[i] - p.poses[0].position[i]).abs() <= 1e-9)
                        && (0..4)
                            .all(|i| (q.quaternion[i] - p.poses[0].quaternion[i]).abs() <= 1e-9)
                })
            })
            .collect();
        let holds_still = holds.iter().all(|h| match h {
            Some(Hold::Body { proxy, .. }) => still[*proxy],
            Some(Hold::Carriage { carriage, .. }) => still[carriages[*carriage].proxy],
            _ => true,
        });
        Ok(Solve {
            bake,
            pass,
            settings,
            holds,
            carriages,
            shapes,
            colliders,
            radius,
            rod,
            times,
            track,
            twist,
            still,
            holds_still,
            last_data: None,
            moved_frame: true,
            settling_left: 0,
            rounds_taken: 0,
            debug: std::env::var_os("BOTRAIL_STATIC_DEBUG").is_some(),
            started: std::time::Instant::now(),
        })
    }

    /// Frame `k` at `t`: the bodies placed, the line relaxed to rest, the
    /// frame recorded. `others` are the other lines of the group as they
    /// stand (their samples and radius): this line keeps off them.
    fn frame(&mut self, k: usize, t: f64, others: &[(&[Vector], f64)]) -> Result<(), Error> {
        let Solve {
            bake,
            pass,
            settings,
            holds,
            carriages,
            shapes,
            colliders,
            radius,
            rod,
            track,
            twist,
            still,
            holds_still,
            last_data,
            moved_frame,
            rounds_taken,
            debug,
            ..
        } = self;
        let mut world: Vec<(Pose, &SharedShape, Aabb)> = Vec::with_capacity(colliders.len());
        let data: Vec<crate::PoseData> = bake.proxies.iter().map(|p| p.pose_at(t)).collect();
        let poses = data
            .iter()
            .map(|p| p.native())
            .collect::<Result<Vec<_>, _>>()?;
        if *holds_still && k > 0 {
            let lo = rod
                .x
                .iter()
                .fold(Vector::splat(f64::INFINITY), |a, p| a.min(*p));
            let hi = rod
                .x
                .iter()
                .fold(Vector::splat(f64::NEG_INFINITY), |a, p| a.max(*p));
            let span = Aabb::new(lo, hi).loosened(*radius + REACH);
            let touched = colliders
                .iter()
                .zip(shapes.iter())
                .any(|(&(pi, offset), shape)| {
                    pi.is_some_and(|pi| {
                        !still[pi] && shape.compute_aabb(&(poses[pi] * offset)).intersects(&span)
                    })
                });
            if !touched {
                track.times.push(t);
                track.points.push(track.points[k - 1].clone());
                track.held.push(Vec::new());
                if let Some(v) = track.twist_deg.as_mut() {
                    let last = v.last().copied().unwrap_or(0.0);
                    v.push(last);
                }
                *last_data = Some(data);
                *moved_frame = false;
                return Ok(());
            }
        }
        // The way from the last frame's poses, in steps of at most
        // `SWEEP_M` of any body's motion near the line.
        if let Some(last) = last_data.as_ref() {
            let far = data
                .iter()
                .zip(last)
                .map(|(a, b)| {
                    (0..3)
                        .map(|i| (a.position[i] - b.position[i]).abs())
                        .fold(0.0_f64, f64::max)
                        + (0..4)
                            .map(|i| (a.quaternion[i] - b.quaternion[i]).abs())
                            .fold(0.0_f64, f64::max)
                            * 0.5
                })
                .fold(0.0_f64, f64::max);
            let steps = ((far / SWEEP_M).ceil() as usize).clamp(1, 20);
            for j in 1..steps {
                let u = j as f64 / steps as f64;
                let between = last
                    .iter()
                    .zip(&data)
                    .map(|(a, b)| a.interpolate(*b, u).native())
                    .collect::<Result<Vec<_>, _>>()?;
                place(
                    holds, carriages, colliders, shapes, *radius, rod, &mut world, &between,
                );
                let (taken, _) = rod.relax(
                    &world,
                    settings,
                    SWEEP_ROUNDS,
                    SWEEP_ROUNDS,
                    carriages,
                    &between,
                    twist.as_mut(),
                    others,
                );
                *rounds_taken += taken;
            }
        }
        place(
            holds, carriages, colliders, shapes, *radius, rod, &mut world, &poses,
        );
        // The first frame starts from the laid polyline, not a rest shape:
        // it may take longer, and is given longer to settle.
        let (rounds, stall) = if k == 0 {
            (settings.rounds * 2, STALL_ROUNDS)
        } else {
            (settings.rounds, STALL_ROUNDS / 3)
        };
        let (taken, moved) = rod.relax(
            &world,
            settings,
            rounds,
            stall,
            carriages,
            &poses,
            twist.as_mut(),
            others,
        );
        *rounds_taken += taken;
        // The rod is installed twist-free in its first rest shape; from
        // then on the twist is measured against that.
        if let Some(tw) = twist.as_mut() {
            let raw = holonomy(
                &rod.x,
                poses[tw.a.proxy].transform_vector(tw.a.normal),
                poses[tw.b.proxy].transform_vector(tw.b.normal),
            );
            if tw.phi0.is_none() {
                tw.phi0 = Some(raw);
                tw.phi = 0.0;
            } else {
                tw.phi = unwrap_near(raw - tw.phi0.unwrap_or(0.0), tw.phi);
            }
            if let Some(v) = track.twist_deg.as_mut() {
                v.push(tw.phi.to_degrees());
            }
        }
        *last_data = Some(data);
        *moved_frame = true;
        if *debug {
            eprintln!(
                "static {} frame {k} t={t:.3}: {taken}/{rounds} rounds, last move {moved:.2e}",
                pass.spec.name
            );
        }
        track.times.push(t);
        track
            .points
            .push(rod.x.iter().map(|p| p.to_array()).collect());
        track.held.push(Vec::new());

        Ok(())
    }

    /// The line settled again at the frame just recorded, against the
    /// other lines where they came to rest this frame (`others`): the
    /// frame's record is replaced. Lines solved side by side each see the
    /// others as they stood at the last frame; this is where one that was
    /// pushed on finds its rest against the other's new place.
    fn settle(&mut self, others: &[(&[Vector], f64)]) -> Result<(), Error> {
        let Solve {
            settings,
            holds,
            carriages,
            shapes,
            colliders,
            radius,
            rod,
            track,
            twist,
            last_data,
            settling_left,
            rounds_taken,
            ..
        } = self;
        let Some(last) = last_data.as_ref() else {
            return Ok(());
        };
        let poses = last
            .iter()
            .map(|p| p.native())
            .collect::<Result<Vec<_>, _>>()?;
        let mut world: Vec<(Pose, &SharedShape, Aabb)> = Vec::with_capacity(colliders.len());
        place(
            holds, carriages, colliders, shapes, *radius, rod, &mut world, &poses,
        );
        let (taken, _) = rod.relax(
            &world,
            settings,
            SETTLE_ROUNDS,
            SETTLE_ROUNDS,
            carriages,
            &poses,
            twist.as_mut(),
            others,
        );
        *rounds_taken += taken;
        *settling_left = if rod.pushes > 0 {
            SETTLE_AFTER
        } else {
            settling_left.saturating_sub(1)
        };
        if let Some(tw) = twist.as_mut() {
            let raw = holonomy(
                &rod.x,
                poses[tw.a.proxy].transform_vector(tw.a.normal),
                poses[tw.b.proxy].transform_vector(tw.b.normal),
            );
            tw.phi = unwrap_near(raw - tw.phi0.unwrap_or(raw), tw.phi);
            if let Some(v) = track.twist_deg.as_mut().and_then(|v| v.last_mut()) {
                *v = tw.phi.to_degrees();
            }
        }
        if let Some(points) = track.points.last_mut() {
            *points = rod.x.iter().map(|p| p.to_array()).collect();
        }
        Ok(())
    }

    /// The box around the line's samples.
    fn bounds(&self) -> (Vector, Vector) {
        self.rod.x.iter().fold(
            (
                Vector::splat(f64::INFINITY),
                Vector::splat(f64::NEG_INFINITY),
            ),
            |(lo, hi), p| (lo.min(*p), hi.max(*p)),
        )
    }

    /// Whether the box around this line comes within reach of the one
    /// around `other`: the two could meet. (Settling a line that then turns
    /// out to touch nothing costs a round or two; telling the two apart
    /// finely costs about as much, and leaves lines that do touch to the
    /// luck of which of them is checked.)
    fn within_reach(&self, other: &Solve) -> bool {
        let reach = self.radius + other.radius + REACH;
        let (lo, hi) = self.bounds();
        let (lo2, hi2) = other.bounds();
        (0..3).all(|c| lo[c] - reach <= hi2[c] && lo2[c] <= hi[c] + reach)
    }

    fn finish(self) -> RopeTrack {
        if self.debug {
            eprintln!(
                "static {}: {} samples, {} frames, {} rounds, {:.2} s",
                self.pass.spec.name,
                self.rod.x.len(),
                self.times.len(),
                self.rounds_taken,
                self.started.elapsed().as_secs_f64()
            );
        }
        self.track
    }
}

/// The static shapes over a bake: one frame every `1 / record_hz` and the
/// final time, each the rod at rest between what holds it then.
pub fn shape(bake: &Bake, pass: &RopePass, settings: &StaticSettings) -> Result<RopeTrack, Error> {
    let mut solve = Solve::new(bake, pass, settings)?;
    let times = solve.times.clone();
    for (k, &t) in times.iter().enumerate() {
        solve.frame(k, t, &[])?;
    }
    Ok(solve.finish())
}

/// Several lines' static shapes over their bakes, solved together frame
/// by frame: each line keeps off the others as they stood at the last
/// frame (all side by side), and the ones within reach of another are
/// then settled in turn against the others at rest, so lines laid side by
/// side do not pass through one another. The bakes share one clock.
pub fn shape_many(
    bakes: &[Bake],
    passes: &[&RopePass],
    settings: &[&StaticSettings],
) -> Result<Vec<RopeTrack>, Error> {
    if bakes.len() != passes.len() || bakes.len() != settings.len() {
        return Err(invalid("shape_many takes one pass and settings per bake"));
    }
    let mut solves: Vec<Solve> = bakes
        .iter()
        .zip(passes)
        .zip(settings)
        .map(|((b, p), s)| Solve::new(b, p, s))
        .collect::<Result<_, _>>()?;
    let Some(first) = solves.first() else {
        return Ok(Vec::new());
    };
    let times = first.times.clone();
    if solves.iter().any(|s| s.times != times) {
        return Err(invalid(
            "lines solved together share one clock (duration and record rate)",
        ));
    }
    if std::env::var_os("BOTRAIL_STATIC_DEBUG").is_some() {
        eprintln!(
            "static together: {}",
            solves
                .iter()
                .map(|s| s.pass.spec.name.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    let workers = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1)
        .clamp(1, solves.len());
    let debug = std::env::var_os("BOTRAIL_STATIC_DEBUG").is_some();
    let (mut side_by_side_s, mut settling_s, mut settles) = (0.0_f64, 0.0_f64, 0usize);
    for (k, &t) in times.iter().enumerate() {
        let started = std::time::Instant::now();
        // Every line to its rest, side by side, each keeping off the others
        // where they stood at the last frame ...
        let snapshot: Vec<(Vec<Vector>, f64)> =
            solves.iter().map(|s| (s.rod.x.clone(), s.radius)).collect();
        let queue = std::sync::Mutex::new(solves.iter_mut().enumerate());
        let outcomes: Vec<Result<(), Error>> = std::thread::scope(|scope| {
            let handles: Vec<_> = (0..workers)
                .map(|_| {
                    let (queue, snapshot) = (&queue, &snapshot);
                    scope.spawn(move || -> Result<(), Error> {
                        loop {
                            let next = queue
                                .lock()
                                .map_err(|_| invalid("a line's solve panicked"))?
                                .next();
                            let Some((i, solve)) = next else {
                                return Ok(());
                            };
                            let others: Vec<(&[Vector], f64)> = snapshot
                                .iter()
                                .enumerate()
                                .filter(|(j, _)| *j != i)
                                .map(|(_, (x, r))| (x.as_slice(), *r))
                                .collect();
                            solve.frame(k, t, &others)?;
                        }
                    })
                })
                .collect();
            handles
                .into_iter()
                .map(|h| {
                    h.join()
                        .unwrap_or_else(|_| Err(invalid("a line's solve panicked")))
                })
                .collect()
        });
        for outcome in outcomes {
            outcome?;
        }
        // ... then each line within reach of one that moved this frame
        // (or pushed on at a frame just past) settled in turn against the
        // others where they came to rest: a line pushed on by one that
        // moved finds its rest off the other's new place, not its old one.
        // A line that kept its frame is where the others already saw it.
        side_by_side_s += started.elapsed().as_secs_f64();
        let started = std::time::Instant::now();
        let moved: Vec<bool> = solves.iter().map(|s| s.moved_frame).collect();
        for i in 0..solves.len() {
            let near =
                (0..solves.len()).any(|j| j != i && moved[j] && solves[i].within_reach(&solves[j]));
            if !near && solves[i].settling_left == 0 {
                continue;
            }
            let others: Vec<(Vec<Vector>, f64)> = solves
                .iter()
                .enumerate()
                .filter(|(j, _)| *j != i)
                .map(|(_, s)| (s.rod.x.clone(), s.radius))
                .collect();
            let refs: Vec<(&[Vector], f64)> =
                others.iter().map(|(x, r)| (x.as_slice(), *r)).collect();
            solves[i].settle(&refs)?;
            settles += 1;
        }
        settling_s += started.elapsed().as_secs_f64();
    }
    if debug {
        eprintln!(
            "static together: {} lines, {} frames: side by side {side_by_side_s:.2} s, {settles} settles {settling_s:.2} s",
            solves.len(),
            times.len()
        );
    }
    Ok(solves.into_iter().map(Solve::finish).collect())
}

/// The passes' lines shaped together against the bake (see `shape_many`).
pub fn animate_many(
    scene: &Scene,
    timeline: &SequenceTimeline,
    passes: &[(&RopePass, &StaticSettings)],
) -> Result<Vec<RopeTrack>, Error> {
    let bakes: Vec<Bake> = passes
        .iter()
        .map(|(p, _)| bake(scene, timeline, p))
        .collect::<Result<_, _>>()?;
    let ps: Vec<&RopePass> = passes.iter().map(|(p, _)| *p).collect();
    let ss: Vec<&StaticSettings> = passes.iter().map(|(_, s)| *s).collect();
    shape_many(&bakes, &ps, &ss)
}

/// The held samples at the bodies' poses, and the bodies' shapes where the
/// poses put them.
fn place<'a>(
    holds: &[Option<Hold>],
    carriages: &[Carriage],
    colliders: &[(Option<usize>, Pose)],
    shapes: &'a [SharedShape],
    radius: f64,
    rod: &mut Rod,
    world: &mut Vec<(Pose, &'a SharedShape, Aabb)>,
    poses: &[Pose],
) {
    for (i, h) in holds.iter().enumerate() {
        match h {
            Some(Hold::Body { proxy, local }) => rod.x[i] = poses[*proxy].transform_point(*local),
            Some(Hold::Pin(p)) => rod.x[i] = *p,
            Some(Hold::Carriage { carriage, offset }) => {
                let c = &carriages[*carriage];
                rod.x[i] = poses[c.proxy].transform_point(c.base + c.axis * c.s + *offset);
            }
            None => {}
        }
    }
    world.clear();
    for (&(pi, offset), shape) in colliders.iter().zip(shapes) {
        let pose = match pi {
            Some(pi) => poses[pi] * offset,
            None => offset,
        };
        let aabb = shape.compute_aabb(&pose).loosened(radius + 0.002);
        world.push((pose, shape, aabb));
    }
}

/// The rod between its holds: positions, the relaxation's velocities, the
/// inverse masses (zero where held), and the rod's figures.
struct Rod {
    x: Vec<Vector>,
    v: Vec<Vector>,
    w: Vec<f64>,
    ds: f64,
    radius: f64,
    /// `EI / ds`: the moment per radian of turn at a sample — the energy
    /// of a turn θ there is (EI / ds)(1 − cos θ).
    stiffness: f64,
    /// `GJ / L`: the moment per radian of twist between the ends — the
    /// energy of a twist Φ is (GJ / 2L) Φ².
    torsion: f64,
    /// The explicit step (s) in the relaxation's own time.
    step: f64,
    ground: Option<f64>,
    /// How often another line (or the line itself) pushed on a sample in
    /// the last relaxation.
    pushes: usize,
}

impl Rod {
    /// Relax to rest: at most `rounds` rounds of explicit steps — the
    /// bending forces and gravity, then the length and the bodies around
    /// projected — the velocities zeroed whenever the motion starts to slow
    /// (kinetic damping), given up after `stall` rounds without the motion
    /// slowing further. Returns the rounds taken and the farthest any
    /// sample moved in the last of them.
    fn relax(
        &mut self,
        world: &[(Pose, &SharedShape, Aabb)],
        s: &StaticSettings,
        rounds: usize,
        stall: usize,
        carriages: &mut [Carriage],
        poses: &[Pose],
        mut twist: Option<&mut Twist>,
        others: &[(&[Vector], f64)],
    ) -> (usize, f64) {
        let n = self.x.len();
        self.pushes = 0;
        let mut twist_grad = vec![Vector::ZERO; n];
        // The other lines' segments an own segment could meet (and its
        // own, two or more segments away), looked up as the bodies are.
        let mut near_lines: Vec<Vec<(usize, usize)>> = vec![Vec::new(); n.saturating_sub(1)];
        let mut relook = vec![false; n.saturating_sub(1)];
        let segments: Vec<(&[Vector], f64)> = others.iter().copied().collect();
        let boxes: Vec<(Vector, Vector)> = segments
            .iter()
            .map(|(line, _)| {
                line.iter().fold(
                    (
                        Vector::splat(f64::INFINITY),
                        Vector::splat(f64::NEG_INFINITY),
                    ),
                    |(lo, hi), p| (lo.min(*p), hi.max(*p)),
                )
            })
            .collect();
        // Own segments close enough along the line to lie within two radii
        // of each other when it is straight are not a contact.
        let gap = 1 + ((2.0 * self.radius / self.ds).ceil() as usize).max(1);
        let lookup_lines = |e: usize,
                            own: &[Vector],
                            own_r: f64,
                            near: &mut Vec<(usize, usize)>| {
            near.clear();
            let (a, b) = (own[e], own[e + 1]);
            for (l, (line, r)) in segments.iter().enumerate() {
                let reach = own_r + r + REACH;
                let (lo, hi) = boxes[l];
                if (0..3).any(|k| a[k].max(b[k]) < lo[k] - reach || a[k].min(b[k]) > hi[k] + reach)
                {
                    continue;
                }
                for (k, w) in line.windows(2).enumerate() {
                    if segments_box_meet(a, b, w[0], w[1], reach) {
                        near.push((l, k));
                    }
                }
            }
            let reach = 2.0 * own_r + REACH;
            for (k, w) in own.windows(2).enumerate() {
                if (k + gap <= e || k >= e + gap) && segments_box_meet(a, b, w[0], w[1], reach) {
                    near.push((usize::MAX, k));
                }
            }
        };
        let h = self.step;
        let substeps = (ROUND_S / h).ceil().max(1.0) as usize;
        let g = Vector::new(0.0, 0.0, -GRAVITY);
        let mut force = vec![Vector::ZERO; n];
        let mut prev = self.x.clone();
        let mut before = self.x.clone();
        let mut energy_prev = 0.0;
        // A span its holds and the bodies around leave no rest to (a line
        // a hair too short for the corner it is pushed off) jitters instead
        // of settling: give up once the motion has not slowed for a while.
        let (mut best, mut since_best) = (f64::INFINITY, 0usize);
        // The bodies a sample could meet: those within `REACH` of it when
        // it was last looked up (their boxes first, then the surface itself
        // — a hull's box takes in far more than its hull), looked up again
        // once it has moved half that far.
        let mut near: Vec<Vec<(usize, Contact)>> = vec![Vec::new(); n];
        let mut looked = vec![Vector::splat(f64::INFINITY); n];
        let radius = self.radius;
        let lookup = |x: Vector, near: &mut Vec<(usize, Contact)>| {
            near.clear();
            for (j, (pose, shape, aabb)) in world.iter().enumerate() {
                if !aabb.loosened(REACH).contains_local_point(x) {
                    continue;
                }
                let proj = shape.project_point(pose, x, false);
                if proj.is_inside || (x - proj.point).length() <= radius + REACH {
                    near.push((j, Contact::default()));
                }
            }
        };
        for round in 0..rounds {
            before.copy_from_slice(&self.x);
            for _ in 0..substeps {
                prev.copy_from_slice(&self.x);
                force.iter_mut().for_each(|f| *f = Vector::ZERO);
                for i in 1..n - 1 {
                    self.bend(i, &mut force);
                }
                // The twist between the roll-holding ends pulls the rod
                // toward the shape that carries less of it (writhe): the
                // holonomy's gradient, scaled by the twist and GJ / L.
                if let Some(tw) = twist.as_deref_mut() {
                    if let Some(phi0) = tw.phi0 {
                        let raw = holonomy(
                            &self.x,
                            poses[tw.a.proxy].transform_vector(tw.a.normal),
                            poses[tw.b.proxy].transform_vector(tw.b.normal),
                        );
                        tw.phi = unwrap_near(raw - phi0, tw.phi);
                        holonomy_gradient(&self.x, &mut twist_grad);
                        let moment = self.torsion * tw.phi;
                        for i in 0..n {
                            force[i] -= twist_grad[i] * moment;
                        }
                    }
                }
                for i in 0..n {
                    if self.w[i] > 0.0 {
                        self.v[i] = (self.v[i] + (force[i] * self.w[i] + g) * h) * DAMPING;
                        self.x[i] += self.v[i] * h;
                    }
                }
                for e in 0..n - 1 {
                    self.stretch(e);
                }
                for e in (0..n - 1).rev() {
                    self.stretch(e);
                }
                for i in 0..n {
                    if self.w[i] > 0.0 && (self.x[i] - looked[i]).length() > REACH / 2.0 {
                        lookup(self.x[i], &mut near[i]);
                        looked[i] = self.x[i];
                        if i > 0 {
                            relook[i - 1] = true;
                        }
                        if i + 1 < n {
                            relook[i] = true;
                        }
                    }
                }
                for e in 0..n.saturating_sub(1) {
                    if std::mem::take(&mut relook[e]) {
                        lookup_lines(e, &self.x, self.radius, &mut near_lines[e]);
                    }
                }
                self.collide(world, &mut near);
                self.collide_lines(&segments, &near_lines);
                // The carriage runs along its rail: where the line's length
                // took its samples (on average, along the rail), less the
                // retraction's creep toward the start, within the stroke;
                // its samples ride with it as one.
                for c in carriages.iter_mut() {
                    let pose = &poses[c.proxy];
                    let axis = pose.transform_vector(c.axis);
                    let along: f64 = c
                        .samples
                        .iter()
                        .map(|&(i, offset)| {
                            (self.x[i] - pose.transform_point(c.base + offset)).dot(axis)
                        })
                        .sum::<f64>()
                        / c.samples.len() as f64;
                    c.s = (along - RETRACT_M).clamp(0.0, c.stroke);
                    for &(i, offset) in &c.samples {
                        self.x[i] = pose.transform_point(c.base + c.axis * c.s + offset);
                    }
                }
                let mut energy = 0.0;
                for i in 0..n {
                    if self.w[i] > 0.0 {
                        let d = self.x[i] - prev[i];
                        self.v[i] = d / h;
                        energy += d.length_squared();
                    }
                }
                if energy < energy_prev {
                    self.v.iter_mut().for_each(|v| *v = Vector::ZERO);
                    energy = 0.0;
                }
                energy_prev = energy;
            }
            let moved = (0..n)
                .map(|i| (self.x[i] - before[i]).length())
                .fold(0.0_f64, f64::max);
            if moved < 0.95 * best {
                (best, since_best) = (moved, 0);
            } else {
                since_best += 1;
            }
            if moved < s.tolerance_m || round + 1 == rounds || since_best >= stall {
                // The length tightened once more before the frame is kept,
                // the carriages' samples riding where their carriages are.
                for _ in 0..3 {
                    for e in 0..n - 1 {
                        self.stretch(e);
                    }
                    for e in (0..n - 1).rev() {
                        self.stretch(e);
                    }
                }
                for c in carriages.iter() {
                    let pose = &poses[c.proxy];
                    for &(i, offset) in &c.samples {
                        self.x[i] = pose.transform_point(c.base + c.axis * c.s + offset);
                    }
                }
                // ... and off the bodies and the other lines again.
                self.collide(world, &mut near);
                self.collide_lines(&segments, &near_lines);
                return (round + 1, moved);
            }
        }
        (rounds, f64::NAN)
    }

    /// Segment `e` back to its length.
    fn stretch(&mut self, e: usize) {
        let (i, j) = (e, e + 1);
        let wsum = self.w[i] + self.w[j];
        if wsum <= 0.0 {
            return;
        }
        let d = self.x[j] - self.x[i];
        let len = d.length();
        if len < 1e-12 {
            return;
        }
        let corr = d * ((len - self.ds) / (len * wsum));
        self.x[i] += corr * self.w[i];
        self.x[j] -= corr * self.w[j];
    }

    /// The bending forces of the turn at sample `i` on it and its two
    /// neighbours: −∂/∂x of (EI / ds)(1 − cos θ), θ the angle between
    /// the segments either side.
    fn bend(&self, i: usize, force: &mut [Vector]) {
        let d1 = self.x[i] - self.x[i - 1];
        let d2 = self.x[i + 1] - self.x[i];
        let (l1, l2) = (d1.length(), d2.length());
        if l1 < 1e-12 || l2 < 1e-12 {
            return;
        }
        let (e1, e2) = (d1 / l1, d2 / l2);
        let c = e1.dot(e2).clamp(-1.0, 1.0);
        // sin θ · ∂θ/∂d1 = −a, sin θ · ∂θ/∂d2 = −b.
        let a = (e2 - e1 * c) / l1;
        let b = (e1 - e2 * c) / l2;
        force[i - 1] -= a * self.stiffness;
        force[i] += (a - b) * self.stiffness;
        force[i + 1] += b * self.stiffness;
    }

    /// Every free sample out of the bodies near it, to the line's radius
    /// off their surfaces, and off the floor. A sample that has hardly
    /// moved since a body's surface was last found under it is kept off
    /// the plane found then (a hull's surface is flat at that scale);
    /// the surface itself is asked again once it has moved.
    fn collide(
        &mut self,
        world: &[(Pose, &SharedShape, Aabb)],
        near: &mut [Vec<(usize, Contact)>],
    ) {
        for i in 0..self.x.len() {
            if self.w[i] <= 0.0 {
                continue;
            }
            if let Some(z) = self.ground {
                if self.x[i].z < z + self.radius {
                    self.x[i].z = z + self.radius;
                }
            }
            for (j, contact) in near[i].iter_mut() {
                let (pose, shape, aabb) = &world[*j];
                let p = self.x[i];
                if contact.valid && (p - contact.at).length_squared() < CONTACT_M * CONTACT_M {
                    let off = (p - contact.point).dot(contact.normal);
                    if off < self.radius {
                        self.x[i] = p + contact.normal * (self.radius - off);
                    }
                    continue;
                }
                if !aabb.contains_local_point(p) {
                    contact.valid = false;
                    continue;
                }
                let proj = shape.project_point(pose, p, false);
                let d = p - proj.point;
                let dist = d.length();
                if dist <= 1e-12 {
                    contact.valid = false;
                    continue;
                }
                let normal = if proj.is_inside { -d / dist } else { d / dist };
                *contact = Contact {
                    at: p,
                    point: proj.point,
                    normal,
                    valid: true,
                };
                if proj.is_inside || dist < self.radius {
                    self.x[i] = proj.point + normal * self.radius;
                }
            }
        }
    }
}

impl Rod {
    /// Every segment with a free end off the other lines' segments (and
    /// its own, two or more segments away): two radii between the
    /// centrelines, the segment's ends moved by their share of the
    /// nearest point (a held end stays).
    fn collide_lines(&mut self, lines: &[(&[Vector], f64)], near: &[Vec<(usize, usize)>]) {
        for e in 0..self.x.len().saturating_sub(1) {
            let (wa, wb) = (self.w[e], self.w[e + 1]);
            if wa <= 0.0 && wb <= 0.0 {
                continue;
            }
            for &(l, k) in &near[e] {
                let (c, d, need) = if l == usize::MAX {
                    (self.x[k], self.x[k + 1], 2.0 * self.radius)
                } else {
                    let (line, r) = lines[l];
                    (line[k], line[k + 1], self.radius + r)
                };
                let (a, b) = (self.x[e], self.x[e + 1]);
                let (s, t) = closest_segments(a, b, c, d);
                let diff = (a + (b - a) * s) - (c + (d - c) * t);
                let dist = diff.length();
                if dist >= need {
                    continue;
                }
                let dir = if dist > 1e-9 {
                    diff / dist
                } else {
                    let cross = (b - a).cross(d - c);
                    if cross.length() < 1e-12 {
                        continue;
                    }
                    cross.normalize()
                };
                let denom = wa * (1.0 - s) * (1.0 - s) + wb * s * s;
                if denom <= 0.0 {
                    continue;
                }
                // The ends' shares of the way out, each no farther than the
                // way out itself: with one end held and the nearest point
                // close to it, the other's share grows as the lever, and
                // a sample flung by a few centimetres kinks the line.
                let push = (need - dist) / denom;
                let (mut m_a, mut m_b) = (push * wa * (1.0 - s), push * wb * s);
                let most = m_a.max(m_b);
                if most > need - dist {
                    let scale = (need - dist) / most;
                    m_a *= scale;
                    m_b *= scale;
                }
                self.x[e] += dir * m_a;
                self.x[e + 1] += dir * m_b;
                self.pushes += 1;
            }
        }
    }
}

/// The parameters of the nearest points of the segments `p0`–`p1` and
/// `q0`–`q1` (Ericson, Real-Time Collision Detection 5.1.9).
fn closest_segments(p0: Vector, p1: Vector, q0: Vector, q1: Vector) -> (f64, f64) {
    const EPS: f64 = 1e-18;
    let d1 = p1 - p0;
    let d2 = q1 - q0;
    let r = p0 - q0;
    let a = d1.length_squared();
    let e = d2.length_squared();
    let f = d2.dot(r);
    if a <= EPS && e <= EPS {
        return (0.0, 0.0);
    }
    if a <= EPS {
        return (0.0, (f / e).clamp(0.0, 1.0));
    }
    let c = d1.dot(r);
    if e <= EPS {
        return ((-c / a).clamp(0.0, 1.0), 0.0);
    }
    let b = d1.dot(d2);
    let denom = a * e - b * b;
    let mut s = if denom.abs() > EPS {
        ((b * f - c * e) / denom).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let mut t = (b * s + f) / e;
    if t < 0.0 {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - c) / a).clamp(0.0, 1.0);
    }
    (s, t)
}

/// Whether the boxes around the segments `a`–`b` and `c`–`d` come within
/// `reach` of each other.
fn segments_box_meet(a: Vector, b: Vector, c: Vector, d: Vector, reach: f64) -> bool {
    (0..3).all(|i| {
        a[i].max(b[i]) + reach >= c[i].min(d[i]) && a[i].min(b[i]) - reach <= c[i].max(d[i])
    })
}

/// Where a body's surface was last found under a sample: the sample's
/// position then, the nearest surface point and its outward normal.
#[derive(Clone, Copy, Default)]
struct Contact {
    at: Vector,
    point: Vector,
    normal: Vector,
    valid: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_holonomy_gradient_matches_a_finite_difference() {
        // A bent, twisted polyline; its holonomy's derivative by each
        // coordinate of an interior sample against the analytic gradient.
        let x: Vec<Vector> = (0..7)
            .map(|i| {
                let t = i as f64 * 0.3;
                Vector::new(t, 0.2 * (t * 1.7).sin(), 0.15 * (t * 1.1).cos())
            })
            .collect();
        let n_a = Vector::new(0.0, 1.0, 0.3).normalize();
        let n_b = Vector::new(0.2, 0.9, -0.4).normalize();
        let mut grad = vec![Vector::ZERO; x.len()];
        holonomy_gradient(&x, &mut grad);
        let base = holonomy(&x, n_a, n_b);
        // Away from the ends: the samples next to an end are held with it
        // (the end's normal is projected square to the first and the last
        // edge, which the gradient does not follow).
        for i in 2..x.len() - 2 {
            for axis in 0..3 {
                let h = 1e-6;
                let mut y = x.clone();
                y[i][axis] += h;
                let fd = (holonomy(&y, n_a, n_b) - base) / h;
                assert!(
                    (fd - grad[i][axis]).abs() < 1e-3 * (1.0 + fd.abs()),
                    "sample {i} axis {axis}: finite difference {fd} vs gradient {}",
                    grad[i][axis]
                );
            }
        }
    }

    #[test]
    fn a_turn_of_an_end_is_read_as_twist_and_unwraps() {
        let x: Vec<Vector> = (0..5)
            .map(|i| Vector::new(0.0, 0.0, i as f64 * 0.1))
            .collect();
        let n_a = Vector::X;
        for deg in [10.0_f64, 90.0, 170.0, -45.0] {
            let r = deg.to_radians();
            let n_b = Vector::new(r.cos(), r.sin(), 0.0);
            assert!((holonomy(&x, n_a, n_b) - r).abs() < 1e-9, "{deg}");
        }
        assert!(
            (unwrap_near(-170.0_f64.to_radians(), 180.0_f64.to_radians()) - 190.0_f64.to_radians())
                .abs()
                < 1e-9
        );
    }
}

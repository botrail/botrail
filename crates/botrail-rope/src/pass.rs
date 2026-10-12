//! Explicit event-boundary replay. Standard Rapier steps are never retried.
use crate::{
    shape_bridge::{self, ShapeData},
    track::Replay,
    Error, PoseData, RopeConnectorTrack, RopeEvent, RopeLocation, RopeSpec, RopeTrack,
};
use botrail_scene::{rollout::SequenceTimeline, Scene};
use rapier_rope::{rapier::prelude::*, *};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct LinkBinding {
    pub robot: String,
    pub link: String,
}
impl LinkBinding {
    pub fn key(&self) -> String {
        format!("{}/{}", self.robot, self.link)
    }
}

#[derive(Debug, Clone)]
pub struct GripperBinding {
    pub link: LinkBinding,
    pub signal: String,
    /// An explicit material point, not collision-inferred gripping.
    pub location: RopeLocation,
}
/// What an [`Anchor`] holds the rope to.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum AnchorBody {
    Link(LinkBinding),
    /// A scene obstacle, at its baked pose: one a robot carries moves.
    Obstacle(String),
}
impl AnchorBody {
    /// The proxy name: `robot/link`, or `obstacle/<name>`.
    pub fn key(&self) -> String {
        match self {
            Self::Link(link) => link.key(),
            Self::Obstacle(name) => format!("obstacle/{name}"),
        }
    }
}
/// A material span held by a body for the whole replay — the crimp and
/// boot inside a connector housing, the clip on a harness. The rope
/// follows the body and never pulls it back. The anchor captures each
/// held particle's offset from the body at time zero; nothing teleports.
#[derive(Debug, Clone)]
pub struct Anchor {
    pub body: AnchorBody,
    pub location: RopeLocation,
    /// Reference arc length held: inward from `Start`/`End`, centred on
    /// any other location. Zero holds one sample, which stays free to
    /// turn; a span holding two or more also holds the rope's direction.
    pub length_m: f64,
    /// The anchor is a carriage on a rail of the body — a retraction unit
    /// (the static shape runs it; the replay holds the carriage at the
    /// rail's start).
    pub rail: Option<Rail>,
    /// The holder holds the rope's roll about its own axis too (a rigid
    /// clamp, a connector): the static rod carries the twist between two
    /// such holders. The replay has no torsion.
    pub roll: bool,
}

/// A retracting anchor's rail: from the held place along `axis` (world,
/// unit, at time zero) for `stroke_m`, the carriage pulled back toward
/// the start with `force_n`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rail {
    pub axis: [f64; 3],
    pub stroke_m: f64,
    pub force_n: f64,
}
/// A guide the line runs through: a ring on a body — a sliding holder —
/// the line slides and rolls in. A collider on the body, not an anchor.
#[derive(Debug, Clone)]
pub struct Guide {
    pub body: AnchorBody,
    /// The ring's centre at time zero (world, m) and its axis (world, unit).
    pub position: [f64; 3],
    pub axis: [f64; 3],
    /// The ring's length along its axis (m) and its inner radius (m).
    pub length_m: f64,
    pub inner_radius_m: f64,
}

/// The ring's wall: plates this thick around the bore — thick enough that a
/// line pulled taut cannot jump through one between two steps.
const GUIDE_WALL_M: f64 = 0.01;
const GUIDE_PLATES: usize = 8;

impl Guide {
    /// The ring as plates around its bore, in the body's frame at time zero.
    fn plates(&self, body_at_zero: &Pose) -> Result<Vec<(PoseData, ShapeData)>, Error> {
        use ::nalgebra::{Matrix3, Point3, Rotation3, UnitQuaternion, Vector3};
        if !self.length_m.is_finite()
            || self.length_m <= 0.0
            || !self.inner_radius_m.is_finite()
            || self.inner_radius_m <= 0.0
            || self
                .position
                .iter()
                .chain(&self.axis)
                .any(|v| !v.is_finite())
        {
            return Err(invalid(
                "a guide needs a finite position and axis, a positive length and inner radius",
            ));
        }
        let centre = body_at_zero.inverse_transform_point(Vector::from_array(self.position));
        let axis = body_at_zero.inverse_transform_vector(Vector::from_array(self.axis));
        let axis = Vector3::new(axis.x, axis.y, axis.z);
        if axis.norm() < 1e-9 {
            return Err(invalid("a guide's axis must not be zero"));
        }
        let axis = axis.normalize();
        let seed = if axis.x.abs() < 0.9 {
            Vector3::x()
        } else {
            Vector3::y()
        };
        let a = axis.cross(&seed).normalize();
        let b = axis.cross(&a);
        let centre = Point3::new(centre.x, centre.y, centre.z);
        let half_wall = GUIDE_WALL_M / 2.0;
        let radius = self.inner_radius_m + half_wall;
        // Each plate spans its share of the circle, with a little overlap.
        let half_width = radius * (std::f64::consts::PI / GUIDE_PLATES as f64).tan() * 1.15;
        let mut plates = Vec::with_capacity(GUIDE_PLATES);
        for k in 0..GUIDE_PLATES {
            let phi = 2.0 * std::f64::consts::PI * k as f64 / GUIDE_PLATES as f64;
            let radial = a * phi.cos() + b * phi.sin();
            let tangential = axis.cross(&radial);
            let rotation = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[
                radial, tangential, axis,
            ]));
            let q = UnitQuaternion::from_rotation_matrix(&rotation);
            let p = centre + radial * radius;
            plates.push((
                PoseData {
                    position: [p.x, p.y, p.z],
                    quaternion: [q.i, q.j, q.k, q.w],
                },
                ShapeData::Cuboid([half_wall, half_width, self.length_m / 2.0]),
            ));
        }
        Ok(plates)
    }
}

#[derive(Debug, Clone)]
pub struct Connector {
    pub name: String,
    pub location: RopeLocation,
    /// Dynamic connectors currently display and simulate exact balls/boxes.
    pub shape: ShapeData,
    pub pose: PoseData,
    pub mass_kg: f64,
}
#[derive(Debug, Clone)]
pub struct RopePass {
    pub spec: RopeSpec,
    pub grippers: Vec<GripperBinding>,
    /// Links whose collision geometry participates; gripper frames are
    /// included even without collision geometry. Geometry is never inferred.
    pub collision_links: Vec<LinkBinding>,
    /// Explicit scene obstacle names; disabled obstacles are rejected.
    pub obstacles: Vec<String>,
    pub connectors: Vec<Connector>,
    pub step_s: f64,
    /// Optional constant pins throughout replay (e.g. a fixed harness end).
    pub pins: Vec<(RopeLocation, [f64; 3])>,
    /// Spans held by links or obstacles throughout replay. Their bodies
    /// get frame proxies; geometry collides only when also selected.
    pub anchors: Vec<Anchor>,
    /// Rings on bodies the line runs through (sliding holders): colliders
    /// on their bodies, which the line slides and rolls in.
    pub guides: Vec<Guide>,
    /// Display colour carried to the track (linear RGB).
    pub color: Option<[f32; 3]>,
    /// A floor the rope rests on: a static half-space at this height
    /// (`None` = no floor — the cell's obstacles are all there is).
    pub ground_z: Option<f64>,
    /// Settling time before the clock starts: the rope finds its hanging
    /// shape under gravity while everything that holds it stays at its
    /// time-zero pose. Nothing of it is recorded; zero = none.
    pub preroll_s: f64,
    /// Static colliders of the pass's own (world pose, shape): the floor and
    /// walls of a duct the rope lies in, which the cell models as one box.
    pub extra_colliders: Vec<(PoseData, ShapeData)>,
    /// The bending springs rest straight — a tube or a cable off the reel,
    /// bent only by what holds it and by its weight — instead of in the
    /// shape the rope is laid in (the default: a pre-formed harness keeps
    /// its bends, and a rope laid with a corner keeps the corner).
    pub straight_rest: bool,
    /// The rope as a rod: windows of a few particles along it, each drawn
    /// toward its own rest shape (straight, or as laid) by a spring of this
    /// natural frequency — zero for none. The two-away bending springs
    /// resist a fold and little else; this is what makes a tube hold a
    /// round loop between its holders instead of hanging in a V. A
    /// spring, not a calibrated flexural rigidity.
    pub rod_hz: f64,
}

/// A rod's window: seven particles, every other particle — a bend is
/// felt by three or four windows at once, and the windows overlap by
/// more than half.
const ROD_WINDOW: u32 = 7;
const ROD_STRIDE: u32 = 2;

#[derive(Debug, Clone)]
pub struct Proxy {
    pub name: String,
    pub times: Vec<f64>,
    pub poses: Vec<PoseData>,
    /// Local offsets and neutral shapes; dimensions stay in body-local SI.
    pub shapes: Vec<(PoseData, ShapeData)>,
}
impl Proxy {
    pub fn pose_at(&self, t: f64) -> PoseData {
        let k = self.times.partition_point(|s| *s <= t).saturating_sub(1);
        let j = (k + 1).min(self.times.len() - 1);
        let u = if k == j {
            0.0
        } else {
            ((t - self.times[k]) / (self.times[j] - self.times[k])).clamp(0.0, 1.0)
        };
        self.poses[k].interpolate(self.poses[j], u)
    }
}
#[derive(Debug, Clone)]
pub struct Signal {
    pub name: String,
    pub edges: Vec<(f64, bool)>,
}
/// Neutral replay inputs can also be provided by another baker. No source
/// Rapier/Parry object can be stored here.
#[derive(Debug, Clone)]
pub struct Bake {
    pub duration_s: f64,
    pub proxies: Vec<Proxy>,
    pub signals: Vec<Signal>,
}

/// The playback track's sampling: the physics steps every `step_s`, but
/// the track keeps every event (signal edges, the first and final
/// samples), at most `RECORD_HZ` frames a second otherwise, and only the two
/// ends of a hold — frames within `HOLD_M` of the last kept one. Linear
/// blending between kept frames (studio, `positions_at`) is then within
/// `HOLD_M` of the rope during a hold. The native track keeps every step.
pub const RECORD_HZ: f64 = 60.0;
pub const HOLD_M: f64 = 2e-5;

struct Frame {
    time: f64,
    points: Vec<[f64; 3]>,
    held: Vec<u32>,
    connectors: Vec<botrail_scene::wire::PoseMsg>,
}

#[derive(Default)]
struct Recorder {
    /// The latest frame not kept because the rope was at rest: the end of
    /// a hold, kept when the rope moves again.
    hold: Option<Frame>,
}
impl Recorder {
    fn offer(&mut self, track: &mut RopeTrack, frame: Frame, event: bool) {
        let Some(&last) = track.times.last() else {
            Self::keep(track, frame);
            return;
        };
        let moved = Self::moved(track, &frame);
        let due = frame.time - last >= 1.0 / RECORD_HZ - 1e-12;
        if event || (due && moved) {
            if let Some(hold) = self.hold.take() {
                if hold.time > last && hold.time < frame.time {
                    Self::keep(track, hold);
                }
            }
            Self::keep(track, frame);
        } else if !moved {
            self.hold = Some(frame);
        }
    }
    fn moved(track: &RopeTrack, frame: &Frame) -> bool {
        let points = track.points.last().expect("a kept frame");
        let far = |a: &[f64; 3], b: &[f64; 3]| (0..3).any(|i| (a[i] - b[i]).abs() > HOLD_M);
        track.held.last() != Some(&frame.held)
            || points.iter().zip(&frame.points).any(|(a, b)| far(a, b))
            || track
                .connectors
                .iter()
                .zip(&frame.connectors)
                .any(|(c, p)| {
                    c.poses.last().is_some_and(|q| {
                        far(&q.position, &p.position)
                            || (0..4).any(|i| (q.quaternion[i] - p.quaternion[i]).abs() > 1e-4)
                    })
                })
    }
    fn keep(track: &mut RopeTrack, frame: Frame) {
        track.times.push(frame.time);
        track.points.push(frame.points);
        track.held.push(frame.held);
        for (c, pose) in track.connectors.iter_mut().zip(frame.connectors) {
            c.poses.push(pose);
        }
    }
}

fn invalid(message: impl Into<String>) -> Error {
    Error::Input(message.into())
}

/// A track ending at the cycle's duration up to roundoff: the rollout sums
/// the duration and each robot's knot times separately, so they can differ
/// in the last bit.
/// A rope whose springs rest straight, laid in the shape its spec starts
/// in. The native rest lengths come from the shape a rope is built in, so
/// it is built along a straight line of the same length, and each particle
/// is then moved to its arc length on the starting polyline — before
/// anything holds it, so every hold captures the laid shape.
fn insert_straight(
    ropes: &mut RopeSet,
    id: WorldId,
    world: &mut PhysicsWorld,
    spec: &RopeSpec,
) -> Result<RopeHandle, Error> {
    if spec.placement != RigidPlacement::default() {
        return Err(invalid(
            "a straight rest takes a rope laid in world coordinates (no placement)",
        ));
    }
    let pts = &spec.reference_centerline_m;
    let dist = |a: [f64; 3], b: [f64; 3]| {
        ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt()
    };
    let mut arcs = vec![0.0];
    for w in pts.windows(2) {
        arcs.push(arcs[arcs.len() - 1] + dist(w[0], w[1]));
    }
    let total = arcs[arcs.len() - 1];
    if pts.len() < 2 || total <= 0.0 || !total.is_finite() {
        return Err(invalid(
            "a straight rest needs a starting line of positive length",
        ));
    }
    let (first, last) = (pts[0], pts[pts.len() - 1]);
    let chord = dist(first, last);
    let dir = if chord > 1e-9 {
        [
            (last[0] - first[0]) / chord,
            (last[1] - first[1]) / chord,
            (last[2] - first[2]) / chord,
        ]
    } else {
        [1.0, 0.0, 0.0]
    };
    let mut straight = spec.clone();
    straight.reference_centerline_m = vec![
        first,
        [
            first[0] + dir[0] * total,
            first[1] + dir[1] * total,
            first[2] + dir[2] * total,
        ],
    ];
    let rope = ropes.insert(id, world, &straight)?;
    let (native, at) = {
        let view = ropes.get(id, world, rope)?;
        (view.native_handle, view.samples.arc_lengths_m().to_vec())
    };
    let body = world
        .soft_bodies
        .get_mut(native)
        .ok_or_else(|| invalid("the rope's soft body is missing"))?;
    for (i, &s) in at.iter().enumerate() {
        // The point at arc length `s` on the starting polyline.
        let k = arcs.partition_point(|a| *a <= s).clamp(1, pts.len() - 1);
        let (a, b) = (pts[k - 1], pts[k]);
        let run = arcs[k] - arcs[k - 1];
        let u = if run > 0.0 {
            ((s - arcs[k - 1]) / run).clamp(0.0, 1.0)
        } else {
            0.0
        };
        body.set_particle_position(
            i,
            Vector::new(
                (a[0] + (b[0] - a[0]) * u) as Real,
                (a[1] + (b[1] - a[1]) * u) as Real,
                (a[2] + (b[2] - a[2]) * u) as Real,
            ),
        );
    }
    Ok(rope)
}

fn ends_at(last: f64, duration: f64) -> bool {
    (last - duration).abs() <= 1e-9 * duration.abs().max(1.0)
}

/// A wire pose lattice on the replay's clock: its last knot is the
/// duration exactly (it may differ from it in the last bit, see `ends_at`).
fn on_clock(times: &[f64], duration: f64) -> Vec<f64> {
    let mut times = times.to_vec();
    if let Some(last) = times.last_mut() {
        if ends_at(*last, duration) {
            *last = duration;
        }
    }
    times
}

/// Capture the same 30 Hz pose lattice sent to Studio. FK is evaluated at
/// those baked samples, including moving bases, then poses are interpolated.
pub fn bake(scene: &Scene, timeline: &SequenceTimeline, pass: &RopePass) -> Result<Bake, Error> {
    if timeline.robots.len() != scene.robots().len()
        || timeline
            .robots
            .iter()
            .zip(scene.robots())
            .any(|(t, r)| t.name != r.name)
    {
        return Err(invalid("robot track names/order do not match the scene"));
    }
    // Validate before trajectory sampling (empty/malformed tracks otherwise panic).
    for (track, robot) in timeline.robots.iter().zip(scene.robots()) {
        let t = &track.trajectory;
        if t.times.is_empty()
            || t.times.len() != t.positions.len()
            || t.times.len() != t.velocities.len()
            || t.times[0] != 0.0
            || !ends_at(*t.times.last().unwrap(), timeline.duration)
            || t.times.iter().any(|t| !t.is_finite())
            || t.times.windows(2).any(|w| w[0] >= w[1])
            || t.positions
                .iter()
                .chain(&t.velocities)
                .any(|v| v.len() != robot.model.dof() || v.iter().any(|v| !v.is_finite()))
        {
            return Err(invalid(format!(
                "invalid trajectory for robot {}",
                robot.name
            )));
        }
    }
    let wire = botrail_session::timeline_msg(scene, timeline);
    let selected: BTreeSet<_> = pass
        .collision_links
        .iter()
        .chain(pass.grippers.iter().map(|g| &g.link))
        .chain(pass.anchors.iter().filter_map(|a| match &a.body {
            AnchorBody::Link(link) => Some(link),
            AnchorBody::Obstacle(_) => None,
        }))
        .chain(pass.guides.iter().filter_map(|g| match &g.body {
            AnchorBody::Link(link) => Some(link),
            AnchorBody::Obstacle(_) => None,
        }))
        .cloned()
        .collect();
    let mut proxies = Vec::new();
    for binding in selected {
        let ri = scene
            .robots()
            .iter()
            .position(|r| r.name == binding.robot)
            .ok_or_else(|| invalid(format!("unknown robot {}", binding.robot)))?;
        let robot = &scene.robots()[ri];
        let li = robot
            .model
            .links
            .iter()
            .position(|l| l.name == binding.link)
            .ok_or_else(|| invalid(format!("unknown link {}", binding.key())))?;
        let times = on_clock(&wire.robots[ri].trajectory.times, timeline.duration);
        let poses = times
            .iter()
            .map(|&t| {
                let base = SequenceTimeline::base_pose(&timeline.robots[ri], t)
                    .unwrap_or(*robot.base_pose());
                let links = botrail_kin::forward_kinematics_with_base(
                    &robot.model,
                    &timeline.robots[ri].trajectory.sample(t),
                    &base,
                )
                .map_err(|e| invalid(e.to_string()))?;
                Ok(PoseData::from_iso(&links[li]))
            })
            .collect::<Result<Vec<_>, Error>>()?;
        let mut shapes = Vec::new();
        if pass.collision_links.contains(&binding) {
            let link = &robot.model.links[li];
            let source_count = if link.collisions.is_empty() {
                link.visuals.len()
            } else {
                link.collisions.len()
            };
            let parts = robot.collider().link_parts(li);
            if parts.len() != source_count {
                return Err(invalid(format!(
                    "{} has omitted source collision geometry",
                    binding.key()
                )));
            }
            for (p, s) in parts {
                shapes.push((PoseData::from_old(p), shape_bridge::extract(s)?));
            }
        }
        proxies.push(Proxy {
            name: binding.key(),
            times,
            poses,
            shapes,
        });
    }
    // Moving obstacles use the same baked object pose samples as the viewer.
    // An anchor's obstacle that is not also selected for collision is a
    // frame only, and may be one the cell does not collision-check.
    let mut selected_obstacles: Vec<(&String, bool)> =
        pass.obstacles.iter().map(|name| (name, true)).collect();
    for body in pass
        .anchors
        .iter()
        .map(|a| &a.body)
        .chain(pass.guides.iter().map(|g| &g.body))
    {
        if let AnchorBody::Obstacle(name) = body {
            if !selected_obstacles.iter().any(|(n, _)| *n == name) {
                selected_obstacles.push((name, false));
            }
        }
    }
    for (name, collides) in selected_obstacles {
        let i = scene
            .obstacles()
            .iter()
            .position(|o| o.name == *name)
            .ok_or_else(|| invalid(format!("unknown obstacle {name}")))?;
        let obstacle = &scene.obstacles()[i];
        if collides && !obstacle.enabled {
            return Err(invalid(format!("disabled obstacle {name}")));
        }
        let object = wire.objects.iter().find(|o| o.name == *name);
        if object.is_some_and(|o| o.visible.iter().any(|v| !v)) {
            return Err(invalid(format!(
                "obstacle {name} has visibility/stow transitions; not supported"
            )));
        }
        let times = wire
            .robots
            .first()
            .map(|r| on_clock(&r.trajectory.times, timeline.duration))
            .unwrap_or_else(|| {
                let n = (timeline.duration * 30.0).ceil().max(1.0) as usize;
                (0..=n)
                    .map(|k| timeline.duration * k as f64 / n as f64)
                    .collect()
            });
        let poses = if let Some(o) = object {
            if o.poses.len() == 1 {
                vec![
                    PoseData {
                        position: o.poses[0].position,
                        quaternion: o.poses[0].quaternion
                    };
                    times.len()
                ]
            } else {
                o.poses
                    .iter()
                    .map(|p| PoseData {
                        position: p.position,
                        quaternion: p.quaternion,
                    })
                    .collect()
            }
        } else {
            vec![PoseData::from_iso(&obstacle.pose); times.len()]
        };
        let shapes = if collides {
            scene.obstacle_colliders()[i]
                .parts()
                .iter()
                .map(|(p, s)| Ok((PoseData::from_old(p), shape_bridge::extract(s)?)))
                .collect::<Result<_, Error>>()?
        } else {
            Vec::new()
        };
        proxies.push(Proxy {
            name: format!("obstacle/{name}"),
            times,
            poses,
            shapes,
        });
    }
    // The guides' rings, on their bodies.
    for guide in &pass.guides {
        let key = guide.body.key();
        let proxy = proxies
            .iter_mut()
            .find(|p| p.name == key)
            .ok_or_else(|| invalid(format!("missing proxy {key} for a guide")))?;
        let at_zero = proxy.poses[0].native()?;
        proxy.shapes.extend(guide.plates(&at_zero)?);
    }
    Ok(Bake {
        duration_s: timeline.duration,
        proxies,
        signals: timeline
            .signals
            .iter()
            .map(|s| Signal {
                name: s.name.clone(),
                edges: s.edges.clone(),
            })
            .collect(),
    })
}

pub fn animate(
    scene: &Scene,
    timeline: &SequenceTimeline,
    pass: &RopePass,
) -> Result<Replay, Error> {
    drive(&bake(scene, timeline, pass)?, pass)
}

/// Nominal step boundaries plus all signal edges, all pose knots and the
/// exact final time. Short pulses are simulated, not sampled away. Each
/// event's source and applied times are equal and stored independently.
pub fn drive(bake: &Bake, pass: &RopePass) -> Result<Replay, Error> {
    let duration = bake.duration_s;
    if !pass.preroll_s.is_finite() || pass.preroll_s < 0.0 {
        return Err(invalid("preroll_s must be finite and nonnegative"));
    }
    if !duration.is_finite() || duration < 0.0 || !pass.step_s.is_finite() || pass.step_s <= 0.0 {
        return Err(invalid(
            "duration must be finite/nonnegative and step positive",
        ));
    }
    let n = (duration / pass.step_s).ceil();
    if n > 10_000_000.0 {
        return Err(invalid("replay exceeds 10000000 nominal steps"));
    }
    let mut times: Vec<_> = (0..n as usize).map(|k| k as f64 * pass.step_s).collect();
    times.extend([0.0, duration]);
    let mut names = BTreeSet::new();
    for p in &bake.proxies {
        if p.name.is_empty()
            || !names.insert(p.name.clone())
            || p.times.is_empty()
            || p.times.len() != p.poses.len()
            || p.times[0] != 0.0
            || *p.times.last().unwrap() != duration
            || p.times.iter().any(|t| !t.is_finite())
            || p.times.windows(2).any(|w| w[0] >= w[1])
        {
            return Err(invalid(format!("invalid proxy clock/name {}", p.name)));
        }
        for p in &p.poses {
            p.native()?;
        }
        times.extend(&p.times);
    }
    let proxies: BTreeMap<_, _> = bake.proxies.iter().map(|p| (p.name.clone(), p)).collect();
    let mut signal_names = BTreeSet::new();
    for s in &bake.signals {
        if !signal_names.insert(&s.name)
            || s.edges.is_empty()
            || s.edges[0].0 != 0.0
            || s.edges
                .iter()
                .any(|(t, _)| !t.is_finite() || *t < 0.0 || *t > duration)
            || s.edges.windows(2).any(|w| w[0].0 >= w[1].0)
        {
            return Err(invalid(format!("invalid signal {}", s.name)));
        }
    }
    let mut signals = Vec::new();
    for g in &pass.grippers {
        if !proxies.contains_key(&g.link.key()) {
            return Err(invalid(format!("missing proxy {}", g.link.key())));
        }
        let s = bake
            .signals
            .iter()
            .find(|s| s.name == g.signal)
            .ok_or_else(|| invalid(format!("unknown signal {}", g.signal)))?;
        times.extend(s.edges.iter().map(|(t, _)| *t));
        signals.push(s);
    }
    for a in &pass.anchors {
        if !proxies.contains_key(&a.body.key()) {
            return Err(invalid(format!("missing proxy {}", a.body.key())));
        }
        if !a.length_m.is_finite() || a.length_m < 0.0 {
            return Err(invalid(format!(
                "anchor on {} needs a finite, nonnegative length",
                a.body.key()
            )));
        }
    }
    // Signal edges and the final time are authoritative. Remove only
    // redundant numerical-grid knots within roundoff of those edges;
    // distinct event times, however close, are never merged.
    let mut boundaries = vec![0.0, duration];
    for signal in &signals {
        boundaries.extend(signal.edges.iter().map(|(t, _)| *t));
    }
    boundaries.sort_by(f64::total_cmp);
    boundaries.dedup();
    let near =
        |a: f64, b: f64| (a - b).abs() <= 16.0 * f64::EPSILON * a.abs().max(b.abs()).max(1.0);
    for t in times {
        let i = boundaries.partition_point(|s| *s < t);
        if i < boundaries.len() && near(boundaries[i], t) || i > 0 && near(boundaries[i - 1], t) {
            continue;
        }
        boundaries.insert(i, t);
    }
    let times = boundaries;
    let id = WorldId(6);
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, 0.0, -9.81);
    world.integration_parameters.dt = pass.step_s;
    let mut ropes = RopeSet::new(id)?;
    if !pass.rod_hz.is_finite() || pass.rod_hz < 0.0 {
        return Err(invalid("rod_hz must be finite and nonnegative"));
    }
    let mut spec = pass.spec.clone();
    if pass.rod_hz > 0.0 {
        spec.rod = Some(RodSettings::new(
            ROD_WINDOW,
            ROD_STRIDE,
            SpringSettings::new(pass.rod_hz, 1.0),
        ));
    }
    let rope = if pass.straight_rest {
        insert_straight(&mut ropes, id, &mut world, &spec)?
    } else {
        ropes.insert(id, &mut world, &spec)?
    };
    let mut native = rapier_rope::RopeTrack::new(
        &pass.spec.name,
        "right-handed Z-up SI; baked_one_way_no_source_reaction",
        &world,
    )?;
    native.register_rope(&ropes.centerline(id, &world, rope)?)?;
    let mut bodies = BTreeMap::new();
    for p in &bake.proxies {
        let body = world
            .insert_body(RigidBodyBuilder::kinematic_position_based().pose(p.poses[0].native()?));
        native.add_display_object(DisplayObject {
            name: p.name.clone(),
            role: "kinematic_proxy_marker".into(),
            body: Some(body.into()),
            shape: DisplayShape::Sphere {
                radius_m: FiniteScalar::new(0.003).expect("finite marker"),
            },
            translation_m: p.poses[0]
                .position
                .map(|x| FiniteScalar::new(x).expect("validated pose")),
            rotation_xyzw: p.poses[0]
                .quaternion
                .map(|x| FiniteScalar::new(x).expect("validated pose")),
        })?;
        for (offset, shape) in &p.shapes {
            world.insert_collider(
                ColliderBuilder::new(shape_bridge::reconstruct(shape)?)
                    .position(offset.native()?)
                    .friction(pass.spec.collision.friction),
                Some(body),
            );
        }
        bodies.insert(p.name.clone(), body);
    }
    for (pose, shape) in &pass.extra_colliders {
        world.insert_collider(
            ColliderBuilder::new(shape_bridge::reconstruct(shape)?)
                .position(pose.native()?)
                .friction(pass.spec.collision.friction),
            None,
        );
    }
    if let Some(z) = pass.ground_z {
        // A floor: a wide slab whose top face is at `z` (a half-space
        // would do, but the slab needs no unit-vector plumbing).
        world.insert_collider(
            ColliderBuilder::cuboid(100.0, 100.0, 0.5)
                .translation(Vector::new(0.0, 0.0, z - 0.5))
                .friction(pass.spec.collision.friction),
            None,
        );
    }
    let mut track = RopeTrack {
        name: pass.spec.name.clone(),
        segments: ropes.centerline(id, &world, rope)?.segments().to_vec(),
        radius_m: pass.spec.collision.radius_m,
        times: Vec::new(),
        points: Vec::new(),
        held: Vec::new(),
        events: Vec::new(),
        connectors: Vec::new(),
        coupling: "baked_one_way_no_source_reaction".into(),
        coordinate_system: "right-handed Z-up SI".into(),
        solver: "rapier3d-f64 0.36.0".into(),
        color: pass.color,
        twist_deg: None,
    };
    let mut connector_bodies = Vec::new();
    let mut initial = pass
        .pins
        .iter()
        .map(|(location, position_m)| AttachmentCommand::Pin {
            rope,
            location: location.clone(),
            position_m: *position_m,
        })
        .collect::<Vec<_>>();
    for c in &pass.connectors {
        if c.name.is_empty()
            || bodies.contains_key(&c.name)
            || !names.insert(c.name.clone())
            || !c.mass_kg.is_finite()
            || c.mass_kg <= 0.0
        {
            return Err(invalid("duplicate/empty connector name or invalid mass"));
        }
        let (shape, dimensions) = match &c.shape {
            ShapeData::Ball(r) => ("sphere", vec![*r]),
            ShapeData::Cuboid(h) => ("cuboid", h.to_vec()),
            _ => {
                return Err(invalid(
                    "dynamic connector display supports only exact sphere/cuboid",
                ))
            }
        };
        let (body, _) = world.insert(
            RigidBodyBuilder::dynamic()
                .pose(c.pose.native()?)
                .can_sleep(false),
            ColliderBuilder::new(shape_bridge::reconstruct(&c.shape)?)
                .mass(c.mass_kg)
                .friction(pass.spec.collision.friction),
        );
        initial.push(AttachmentCommand::Attach {
            rope,
            location: c.location.clone(),
            body,
        });
        connector_bodies.push(body);
        track.connectors.push(RopeConnectorTrack {
            name: c.name.clone(),
            shape: shape.into(),
            dimensions_m: dimensions,
            mass_kg: c.mass_kg,
            poses: Vec::new(),
        });
    }
    // An anchor holds every sample inside its span where it lies at time
    // zero; two or more held samples also hold the rope's direction.
    let samples = ropes.get(id, &world, rope)?.samples.clone();
    let slack = 1e-9 * samples.reference_length_m().max(1.0);
    for a in &pass.anchors {
        let at = samples
            .resolve_location(&a.location)
            .map_err(|e| invalid(e.to_string()))?
            .actual_arc_length_m;
        let (lo, hi) = match a.location {
            RopeLocation::Start => (at, at + a.length_m),
            RopeLocation::End => (at - a.length_m, at),
            _ => (at - a.length_m / 2.0, at + a.length_m / 2.0),
        };
        for &s in samples.arc_lengths_m() {
            if s >= lo - slack && s <= hi + slack {
                initial.push(AttachmentCommand::Attach {
                    rope,
                    location: RopeLocation::ArcLength {
                        arc_length_m: s,
                        tolerance_m: slack,
                    },
                    body: bodies[&a.body.key()],
                });
            }
        }
    }
    let mut held = vec![None; pass.grippers.len()];
    // Every signal edge is kept in the playback track; between them, at most
    // `RECORD_HZ` frames a second, and a rope at rest costs its hold's ends.
    let mut events: Vec<f64> = signals
        .iter()
        .flat_map(|s| s.edges.iter().map(|(t, _)| *t))
        .collect();
    events.sort_by(f64::total_cmp);
    events.dedup();
    let mut recorder = Recorder::default();
    for (k, &time) in times.iter().enumerate() {
        let dt = times.get(k + 1).map_or(pass.step_s, |t| t - time);
        world.integration_parameters.dt = dt;
        for p in &bake.proxies {
            let current = p.pose_at(time).native()?;
            // Current pose is already reached by the previous step; at t=0
            // park it explicitly, then set the next target after recording.
            if k == 0 {
                world.bodies[bodies[&p.name]].set_position(current, true);
            }
        }
        let mut commands = if k == 0 {
            std::mem::take(&mut initial)
        } else {
            Vec::new()
        };
        let mut acquiring = Vec::new();
        let mut releasing = Vec::new();
        // All releases precede acquisitions, so a same-time handoff is atomic.
        for (gi, g) in pass.grippers.iter().enumerate() {
            if let Some(&(_, closed)) = signals[gi].edges.iter().find(|(t, _)| *t == time) {
                if !closed {
                    if let Some(h) = held[gi].take() {
                        commands.push(AttachmentCommand::Detach { attachment: h });
                        releasing.push(gi);
                    }
                } else if held[gi].is_none() {
                    acquiring.push(gi);
                } else {
                    return Err(invalid("signal repeats a closed level without a release"));
                }
                if !closed && !releasing.contains(&gi) {
                    let particle = ropes
                        .get(id, &world, rope)?
                        .samples
                        .resolve_location(&g.location)
                        .map_err(|e| invalid(e.to_string()))?
                        .particle_index;
                    track.events.push(RopeEvent {
                        signal: g.signal.clone(),
                        body: g.link.key(),
                        closed: false,
                        source_time_s: time,
                        applied_time_s: time,
                        particle,
                        local_anchor_m: None,
                    });
                }
            }
        }
        let mut acquire_commands = Vec::new();
        for &gi in &acquiring {
            acquire_commands.push((commands.len(), gi));
            let g = &pass.grippers[gi];
            commands.push(AttachmentCommand::Attach {
                rope,
                location: g.location.clone(),
                body: bodies[&g.link.key()],
            });
        }
        let detached = commands
            .iter()
            .filter_map(|c| {
                if let AttachmentCommand::Detach { attachment } = c {
                    Some(ropes.get_attachment(id, &world, *attachment))
                } else {
                    None
                }
            })
            .collect::<Result<Vec<_>, _>>()?;
        let identity = ropes.centerline(id, &world, rope)?.identity();
        let prepared = ropes.prepare_attachments(id, &mut world, k as u64, dt, &commands)?;
        for a in detached {
            native.record_event(TrackEvent {
                step: k as u64,
                time_s: FiniteScalar::new(time).expect("validated time"),
                operation: TrackEventKind::Detach {
                    rope: identity,
                    particle: a.particle,
                    attachment: a.handle.into(),
                    body: a.body.into(),
                },
            })?;
        }
        for location in &prepared.locations {
            if matches!(
                commands[location.command_index],
                AttachmentCommand::Pin { .. }
            ) {
                native.record_event(TrackEvent {
                    step: k as u64,
                    time_s: FiniteScalar::new(time).expect("validated time"),
                    operation: TrackEventKind::Pin {
                        rope: identity,
                        particle: location.location.particle_index,
                    },
                })?;
            }
        }
        for a in &prepared.prepared.created_attachments {
            native.record_event(TrackEvent {
                step: k as u64,
                time_s: FiniteScalar::new(time).expect("validated time"),
                operation: TrackEventKind::Attach {
                    rope: identity,
                    particle: a.particle,
                    attachment: a.handle.into(),
                    body: a.body.into(),
                    local_anchor_m: a
                        .local_anchor_m
                        .map(|x| FiniteScalar::new(x).expect("validated anchor")),
                },
            })?;
        }
        for gi in releasing {
            let g = &pass.grippers[gi];
            let particle = ropes
                .get(id, &world, rope)?
                .samples
                .resolve_location(&g.location)
                .map_err(|e| invalid(e.to_string()))?
                .particle_index;
            track.events.push(RopeEvent {
                signal: g.signal.clone(),
                body: g.link.key(),
                closed: false,
                source_time_s: time,
                applied_time_s: time,
                particle,
                local_anchor_m: None,
            });
        }
        for (ci, gi) in acquire_commands {
            let a = prepared
                .prepared
                .created_attachments
                .iter()
                .find(|a| a.command_index == ci)
                .expect("prepared acquisition");
            held[gi] = Some(a.handle);
            let g = &pass.grippers[gi];
            track.events.push(RopeEvent {
                signal: g.signal.clone(),
                body: g.link.key(),
                closed: true,
                source_time_s: time,
                applied_time_s: time,
                particle: a.particle,
                local_anchor_m: Some(a.local_anchor_m),
            });
        }
        if k == 0 && pass.preroll_s > 0.0 {
            // Settle before the clock starts: the rope was laid along a
            // design polyline, not a hanging shape, and the first moments
            // of a replay would otherwise be its fall. The holders stay at
            // their time-zero poses; none of this is recorded.
            let steps = (pass.preroll_s / pass.step_s).ceil().max(1.0) as usize;
            world.integration_parameters.dt = pass.step_s;
            for p in &bake.proxies {
                world.bodies[bodies[&p.name]].set_next_kinematic_position(p.pose_at(0.0).native()?);
            }
            for _ in 0..steps {
                world.step();
            }
            world.integration_parameters.dt = dt;
        }
        let mut particles = Vec::new();
        for h in held.iter().flatten() {
            particles.push(ropes.get_attachment(id, &world, *h)?.particle);
        }
        particles.sort_unstable();
        recorder.offer(
            &mut track,
            Frame {
                time,
                points: ropes.centerline(id, &world, rope)?.positions_m().collect(),
                held: particles,
                connectors: connector_bodies
                    .iter()
                    .map(|&b| PoseData::from_native(world.bodies[b].position()).message())
                    .collect(),
            },
            k == 0
                || k == times.len() - 1
                || events.binary_search_by(|e| e.total_cmp(&time)).is_ok(),
        );
        let stamp = CaptureStamp::new(CapturePhase::BeforeStep, Some(k as u64), time, dt)?;
        native.push_frame(TrackFrame {
            capture: stamp.clone(),
            ropes: vec![ropes.centerline(id, &world, rope)?.snapshot(stamp)?],
            bodies: bodies
                .values()
                .chain(&connector_bodies)
                .map(|&b| BodyPoseSnapshot::capture(&world, b))
                .collect::<Result<_, _>>()?,
        })?;
        // Final-time events still apply to the final frame. No extra physical
        // step is performed; this private world is consumed by the pass.
        if k == times.len() - 1 {
            break;
        }
        for p in &bake.proxies {
            world.bodies[bodies[&p.name]]
                .set_next_kinematic_position(p.pose_at(times[k + 1]).native()?);
        }
        world.step();
        ropes
            .inspect(id, &world, k as u64)
            .map_err(|e| Error::Advanced {
                time_s: times[k + 1],
                reason: e.to_string(),
            })?;
        let stamp = CaptureStamp::new(CapturePhase::AfterStep, Some(k as u64), times[k + 1], dt)?;
        native.push_frame(TrackFrame {
            capture: stamp.clone(),
            ropes: vec![ropes.centerline(id, &world, rope)?.snapshot(stamp)?],
            bodies: bodies
                .values()
                .chain(&connector_bodies)
                .map(|&b| BodyPoseSnapshot::capture(&world, b))
                .collect::<Result<_, _>>()?,
        })?;
    }
    track
        .events
        .sort_by(|a, b| a.source_time_s.total_cmp(&b.source_time_s));
    Ok(Replay { track, native })
}

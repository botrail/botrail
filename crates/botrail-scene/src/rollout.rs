//! The sequence scan engine: evaluates a [`crate::seq::Sequence`] against a
//! scene snapshot with a fixed-Δt scan loop (PLC-style: transitions are
//! checked once per tick, so step boundaries quantize to the scan period)
//! and bakes the result into a [`SequenceTimeline`].
//!
//! Everything is deterministic: motions plan with the seeded planner
//! against the world state *at their step* (obstacle poses and grasps
//! included), so the same scene + sequence always bakes the same timeline.

use botrail_traj::JointTrajectory;
use nalgebra::Isometry3;
use thiserror::Error;

use crate::seq::{
    vehicle_frame, Action, Condition, DeviceCommand, DeviceKind, SensorKind, SensorWatch, Sequence,
};
use crate::Scene;
use botrail_collide::ObstacleCollider;
use nalgebra::Vector3;

#[derive(Debug, Error)]
pub enum SeqError {
    #[error("unknown sequence `{0}`")]
    UnknownSequence(String),
    #[error("step {}: {message}", step.map(|s| s.to_string()).unwrap_or_else(|| "-".into()))]
    Validation {
        step: Option<usize>,
        message: String,
    },
    #[error("step {step} (`{name}`): planning failed: {message}")]
    PlanFailed {
        step: usize,
        name: String,
        message: String,
    },
    #[error("step {step} (`{name}`): {message}")]
    Action {
        step: usize,
        name: String,
        message: String,
    },
    #[error("timed out after {limit}s waiting in step {step} (`{name}`)")]
    Timeout {
        step: usize,
        name: String,
        limit: f64,
    },
    /// The parallel-program timeout names every stuck cursor: with two or
    /// more programs, "where is everybody waiting" *is* the deadlock
    /// diagnosis (a gate watching a signal nobody sets shows up here).
    #[error("timed out after {limit}s; programs waiting at: {at}")]
    ProgramsTimeout { at: String, limit: f64 },
    #[error(
        "step {step} (`{name}`): >{limit} instantaneous steps in one scan tick (immediate loop?)"
    )]
    ImmediateLoop {
        step: usize,
        name: String,
        limit: usize,
    },
    /// Two robots met mid-cycle. Plans freeze the other robots at their
    /// start-of-motion pose, so this is not a planning bug: the cycle needs
    /// an interlock (§design-multi-robot 3.2) making one arm wait.
    #[error(
        "robots `{a}` and `{b}` collide at t = {t:.3}s ({link_a} × {link_b}); \
         add an interlock (zone sensor / robot_done) so one waits for the other"
    )]
    RobotCollision {
        t: f64,
        a: String,
        b: String,
        link_a: String,
        link_b: String,
    },
    /// A travelling vehicle's body met the environment. Travel is authored,
    /// not planned, so this is the aisle check failing: widen the aisle,
    /// move the shelf, or re-teach the path.
    #[error(
        "vehicle `{vehicle}` collides with `{obstacle}` at t = {t:.3}s \
         (body part `{body}`); widen the aisle or re-teach the path"
    )]
    VehicleCollision {
        t: f64,
        vehicle: String,
        body: String,
        obstacle: String,
    },
}

#[derive(Debug, Clone)]
pub struct RolloutOptions {
    /// Scan period in seconds — transition timing quantizes to this.
    pub dt: f64,
    /// Hard wall-clock cap; exceeded waits are authoring errors.
    pub max_duration: f64,
    pub plan: botrail_plan::PlanOptions,
    /// Instantaneous steps allowed within one scan tick.
    pub immediate_chain_limit: usize,
}

impl Default for RolloutOptions {
    fn default() -> Self {
        RolloutOptions {
            dt: 0.01,
            max_duration: 120.0,
            plan: botrail_plan::PlanOptions::default(),
            immediate_chain_limit: 64,
        }
    }
}

/// One step's active interval on the baked timeline.
#[derive(Debug, Clone)]
pub struct StepSpan {
    pub name: String,
    pub start: f64,
    pub end: f64,
}

/// A boolean signal as a step function: `(time, new_value)` edges,
/// starting with `(0, initial)`.
#[derive(Debug, Clone)]
pub struct BoolTrack {
    pub name: String,
    pub edges: Vec<(f64, bool)>,
}

impl BoolTrack {
    pub fn value_at(&self, t: f64) -> bool {
        self.edges
            .iter()
            .take_while(|(edge_t, _)| *edge_t <= t + 1e-12)
            .last()
            .map(|(_, v)| *v)
            .unwrap_or(false)
    }
}

/// Piecewise world motion of one tracked object. Spans tile `[0, duration]`.
#[derive(Debug, Clone)]
pub enum TrackSpan {
    /// At rest at a fixed world pose.
    Hold {
        t0: f64,
        t1: f64,
        pose: Isometry3<f64>,
    },
    /// Rigidly attached to a robot: `world = link_pose(t) ∘ offset`.
    Follow {
        t0: f64,
        t1: f64,
        /// Carrying robot (scene index).
        robot: usize,
        link: usize,
        offset: Isometry3<f64>,
    },
    /// Held, and out of sight: waiting in a magazine, or consumed at the
    /// end of a line. The pose is still defined (collision and queries see
    /// it), but nothing should draw it — a carrier queueing off-line is
    /// stock, not scenery, and watching it teleport onto the belt is the
    /// one thing that gives a recirculating line away.
    Stowed {
        t0: f64,
        t1: f64,
        pose: Isometry3<f64>,
    },
    /// Conveyed at constant velocity from `from` (rotation unchanged).
    Linear {
        t0: f64,
        t1: f64,
        from: Isometry3<f64>,
        velocity: nalgebra::Vector3<f64>,
    },
    /// Turning in place with a vehicle: rotated about the vertical axis
    /// through `center` at constant `omega` (rad/s, +Z right-hand). The
    /// first rotating rigid motion a device produces — closed form, so any
    /// resample rate is exact.
    Pivot {
        t0: f64,
        t1: f64,
        from: Isometry3<f64>,
        /// Floor point of the pivot axis (its z is irrelevant).
        center: nalgebra::Point3<f64>,
        omega: f64,
    },
}

/// Advances `from` by one vehicle motion piece lasting `dt`.
fn apply_piece(from: &Isometry3<f64>, piece: &VehiclePiece, dt: f64) -> Isometry3<f64> {
    match piece {
        VehiclePiece::Lin { velocity } => {
            let mut next = *from;
            next.translation.vector += velocity * dt;
            next
        }
        VehiclePiece::Piv { center, omega } => pivot_pose(from, center, omega * dt),
    }
}

/// Appends one vehicle motion piece to a span list, merging with the open
/// span when it continues it — so a whole leg bakes to a single span.
/// Seeds a leading rest span when the list is empty and the motion starts
/// after t = 0, keeping the tiling from zero.
fn push_vehicle_span(
    spans: &mut Vec<TrackSpan>,
    from: Isometry3<f64>,
    tau0: f64,
    tau1: f64,
    piece: &VehiclePiece,
) {
    if tau1 - tau0 < 1e-12 {
        return;
    }
    if spans.is_empty() && tau0 > 0.0 {
        spans.push(TrackSpan::Hold {
            t0: 0.0,
            t1: tau0,
            pose: from,
        });
    }
    let merged = match (spans.last_mut(), piece) {
        (Some(TrackSpan::Linear { t1, velocity, .. }), VehiclePiece::Lin { velocity: v })
            if (*velocity - v).norm() < 1e-12 && (*t1 - tau0).abs() < 1e-9 =>
        {
            *t1 = tau1;
            true
        }
        (
            Some(TrackSpan::Pivot {
                t1, center, omega, ..
            }),
            VehiclePiece::Piv {
                center: c,
                omega: o,
            },
        ) if (center.coords - c.coords).norm() < 1e-12
            && (*omega - o).abs() < 1e-12
            && (*t1 - tau0).abs() < 1e-9 =>
        {
            *t1 = tau1;
            true
        }
        _ => false,
    };
    if merged {
        return;
    }
    // A gap means the vehicle stood still in between. Stretching the
    // previous span across it would keep *travelling* through the stop —
    // the rest has to be recorded as a rest.
    if let Some(open) = spans.last() {
        let (_, end) = open.range();
        if end < tau0 - 1e-12 {
            spans.push(TrackSpan::Hold {
                t0: end,
                t1: tau0,
                pose: from,
            });
        }
    }
    spans.push(match piece {
        VehiclePiece::Lin { velocity } => TrackSpan::Linear {
            t0: tau0,
            t1: tau1,
            from,
            velocity: *velocity,
        },
        VehiclePiece::Piv { center, omega } => TrackSpan::Pivot {
            t0: tau0,
            t1: tau1,
            from,
            center: *center,
            omega: *omega,
        },
    });
}

/// Rigid rotation of `from` about the vertical line through `center` by
/// `phi` radians.
pub(crate) fn pivot_pose(
    from: &Isometry3<f64>,
    center: &nalgebra::Point3<f64>,
    phi: f64,
) -> Isometry3<f64> {
    let rot = nalgebra::UnitQuaternion::from_axis_angle(&Vector3::z_axis(), phi);
    let arm = from.translation.vector - center.coords;
    Isometry3::from_parts(
        nalgebra::Translation3::from(center.coords + rot * arm),
        rot * from.rotation,
    )
}

impl TrackSpan {
    fn end_mut(&mut self) -> &mut f64 {
        match self {
            TrackSpan::Hold { t1, .. }
            | TrackSpan::Stowed { t1, .. }
            | TrackSpan::Follow { t1, .. }
            | TrackSpan::Linear { t1, .. }
            | TrackSpan::Pivot { t1, .. } => t1,
        }
    }

    pub fn range(&self) -> (f64, f64) {
        match self {
            TrackSpan::Hold { t0, t1, .. }
            | TrackSpan::Stowed { t0, t1, .. }
            | TrackSpan::Follow { t0, t1, .. }
            | TrackSpan::Linear { t0, t1, .. }
            | TrackSpan::Pivot { t0, t1, .. } => (*t0, *t1),
        }
    }
}

#[derive(Debug, Clone)]
pub struct ObjectTrack {
    /// Obstacle name.
    pub name: String,
    pub spans: Vec<TrackSpan>,
}

/// One robot's share of a baked timeline.
#[derive(Debug, Clone)]
pub struct RobotTrack {
    /// Instance name.
    pub name: String,
    /// Whole-sequence joint track (holds still during waits).
    pub trajectory: JointTrajectory,
    /// Intervals a motion/ramp drove this robot (timeline robot lanes),
    /// labelled with the motion name (or `ramp`).
    pub moves: Vec<StepSpan>,
    /// Where the robot's base was over time — `Some` only for a robot that
    /// rides a vehicle. Spans tile `[0, duration]` in the same vocabulary
    /// the load uses, because it is the same rigid motion.
    pub base: Option<Vec<TrackSpan>>,
}

/// The baked result of a sequence rollout — what playback, USD export, and
/// the timing chart consume. `duration` is the cycle time.
#[derive(Debug, Clone)]
pub struct SequenceTimeline {
    pub duration: f64,
    /// One track per robot, in scene order.
    pub robots: Vec<RobotTrack>,
    /// Objects that were grasped at some point (everything else is static).
    pub objects: Vec<ObjectTrack>,
    pub signals: Vec<BoolTrack>,
    pub step_spans: Vec<StepSpan>,
}

impl SequenceTimeline {
    /// Whether a tracked object should be drawn at `t`. False only while it
    /// is stowed — waiting in a magazine or taken off the line.
    pub fn object_visible(track: &ObjectTrack, t: f64) -> bool {
        let span = track
            .spans
            .iter()
            .find(|s| {
                let (t0, t1) = s.range();
                t >= t0 - 1e-9 && t <= t1 + 1e-9
            })
            .or(track.spans.last());
        !matches!(span, Some(TrackSpan::Stowed { .. }))
    }

    /// World pose of a tracked object at `t`; `link_poses[robot]` must be
    /// the FK world poses of that robot at the same instant.
    pub fn object_pose(
        track: &ObjectTrack,
        link_poses: &[Vec<Isometry3<f64>>],
        t: f64,
    ) -> Option<Isometry3<f64>> {
        Self::span_pose(&track.spans, link_poses, t)
    }

    /// Where a mounted robot's base was at `t`.
    pub fn base_pose(track: &RobotTrack, t: f64) -> Option<Isometry3<f64>> {
        Self::span_pose(track.base.as_deref()?, &[], t)
    }

    /// Pose from a span list at `t` (spans tile `[0, duration]`; the last
    /// one extends past its end).
    pub fn span_pose(
        spans: &[TrackSpan],
        link_poses: &[Vec<Isometry3<f64>>],
        t: f64,
    ) -> Option<Isometry3<f64>> {
        let span = spans
            .iter()
            .find(|s| {
                let (t0, t1) = s.range();
                t >= t0 - 1e-9 && t <= t1 + 1e-9
            })
            .or(spans.last())?;
        Some(match span {
            TrackSpan::Hold { pose, .. } | TrackSpan::Stowed { pose, .. } => *pose,
            TrackSpan::Follow {
                robot,
                link,
                offset,
                ..
            } => link_poses[*robot][*link] * offset,
            TrackSpan::Linear {
                t0,
                t1,
                from,
                velocity,
            } => {
                let dt = t.clamp(*t0, *t1) - t0;
                let mut pose = *from;
                pose.translation.vector += velocity * dt;
                pose
            }
            TrackSpan::Pivot {
                t0,
                t1,
                from,
                center,
                omega,
            } => pivot_pose(from, center, omega * (t.clamp(*t0, *t1) - t0)),
        })
    }

    /// The track of the robot instance named `name`.
    pub fn robot_track(&self, name: &str) -> Option<&RobotTrack> {
        self.robots.iter().find(|r| r.name == name)
    }

    /// Seconds `name` spent in motion, with overlapping move intervals
    /// merged (a robot driven by a motion and a ramp in the same breath
    /// is busy once, not twice).
    pub fn busy_seconds(&self, name: &str) -> Option<f64> {
        let track = self.robot_track(name)?;
        let mut spans: Vec<(f64, f64)> = track.moves.iter().map(|s| (s.start, s.end)).collect();
        spans.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut total = 0.0;
        let mut open: Option<(f64, f64)> = None;
        for (start, end) in spans {
            match open {
                Some((s, e)) if start <= e + 1e-12 => open = Some((s, e.max(end))),
                Some((s, e)) => {
                    total += e - s;
                    open = Some((start, end));
                }
                None => open = Some((start, end)),
            }
        }
        if let Some((s, e)) = open {
            total += e - s;
        }
        Some(total)
    }

    /// Fraction of the cycle `name` spent moving, 0..1.
    ///
    /// The line-balancing number: the bottleneck station is the one whose
    /// arms sit near 1, and moving a spot off it is the edit whose effect
    /// on takt this figure predicts. `None` for an unknown instance; 0
    /// for a zero-length cycle.
    pub fn utilization(&self, name: &str) -> Option<f64> {
        let busy = self.busy_seconds(name)?;
        Some(if self.duration > 0.0 {
            busy / self.duration
        } else {
            0.0
        })
    }
}

impl Scene {
    /// Runs the scan loop for `name` against a clone of this scene (the
    /// live scene is untouched). Planned motions are time-parameterized
    /// with their owning robot's URDF-derived limits
    /// ([`crate::motion::traj_limits`]).
    pub fn simulate_sequence(
        &self,
        name: &str,
        options: &RolloutOptions,
    ) -> Result<SequenceTimeline, SeqError> {
        self.simulate_sequences(&[name], options)
    }

    /// Runs several sequences *concurrently* over one shared world — one
    /// scan-loop tick advances every program, in the order given here.
    ///
    /// This is the PLC picture of a line: one POU per station plus a
    /// transfer POU, each a plain serial SFC, synchronized only through
    /// signals and sensors. Concurrency changes nothing about determinism:
    /// programs are scanned in declaration order every tick, so the bake
    /// stays bit-identical run to run.
    ///
    /// Every robot, device, and written signal must be driven by at most
    /// one of the programs (validated up front). Two programs commanding
    /// one robot is not a scheduling problem to referee at runtime — it is
    /// an authoring error, the same as two PLC programs writing one coil.
    pub fn simulate_sequences(
        &self,
        names: &[&str],
        options: &RolloutOptions,
    ) -> Result<SequenceTimeline, SeqError> {
        if names.is_empty() {
            return Err(SeqError::Validation {
                step: None,
                message: "no sequences to simulate".to_string(),
            });
        }
        let mut sequences = Vec::with_capacity(names.len());
        for name in names {
            if sequences.iter().any(|s: &Sequence| &s.name == name) {
                return Err(SeqError::Validation {
                    step: None,
                    message: format!("sequence `{name}` is listed twice"),
                });
            }
            let sequence = self
                .sequence(name)
                .ok_or_else(|| SeqError::UnknownSequence(name.to_string()))?
                .clone();
            self.validate_sequence(&sequence)
                .map_err(|(step, message)| SeqError::Validation {
                    step,
                    message: if names.len() == 1 {
                        message
                    } else {
                        format!("sequence `{name}`: {message}")
                    },
                })?;
            sequences.push(sequence);
        }
        if sequences.len() > 1 {
            self.validate_program_ownership(&sequences)
                .map_err(|message| SeqError::Validation {
                    step: None,
                    message,
                })?;
        }
        Rollout::new(self.clone(), sequences, options.clone()).run()
    }
}

/// Per-robot scan-loop state: the commanded joints, the in-flight move,
/// the tracking latch, and the accumulating baked track.
struct RobotRuntime {
    /// Commanded joints (what the robot actually does).
    q: Vec<f64>,
    /// Joints as the motion/ramp asks for them, before any tracking offset.
    /// Equal to `q` unless a track is active.
    q_nom: Vec<f64>,
    /// Previous tick's nominal, so a tracked solve can warm-start from the
    /// previous *command* plus the nominal increment.
    q_nom_prev: Vec<f64>,
    /// The in-flight motion/ramp, for per-tick joint sampling. One slot —
    /// matching the "one driver per robot per step" rule.
    active: Option<ActiveMove>,
    /// Conveyor tracking: the latched part and the offset it has built up.
    tracking: Option<TrackLatch>,
    // Accumulating baked track.
    times: Vec<f64>,
    positions: Vec<Vec<f64>>,
    velocities: Vec<Vec<f64>>,
    /// Intervals a move drove this robot (the timeline's robot lanes).
    moves: Vec<StepSpan>,
    /// Base motion, for a robot riding a vehicle.
    base: Option<Vec<TrackSpan>>,
}

impl RobotRuntime {
    fn append_waypoint(&mut self, t: f64, q: Vec<f64>, v: Vec<f64>) {
        let last = *self.times.last().expect("seeded with t = 0");
        if t <= last + 1e-9 {
            return;
        }
        self.times.push(t);
        self.positions.push(q);
        self.velocities.push(v);
    }
}

/// One program's scan-loop cursor. Several of these advancing over one
/// world is what "parallel sequences" means: each station is its own SFC,
/// the world is shared, and the only coupling is through what the world
/// carries (signals, sensors, robot/device state).
struct Program {
    sequence: Sequence,
    step: usize,
    entered_at: f64,
    /// Absolute end times of the moves started by the active step (`Done`
    /// waits for all of them).
    move_ends: Vec<f64>,
    /// Index into `step_spans` of the active step's span. Programs
    /// interleave their spans in one list, so "the last one" stopped
    /// meaning "mine" the moment there were two cursors.
    open_span: usize,
}

impl Program {
    fn finished(&self) -> bool {
        self.step >= self.sequence.steps.len()
    }
}

struct Rollout {
    world: Scene,
    /// The concurrently-running programs, in the order given to
    /// `simulate_sequences` — which is the deterministic scan order: every
    /// tick each program gets one transition scan, first to last, so a
    /// signal written by an earlier program is seen by a later one in the
    /// same tick, and the whole bake stays bit-identical.
    programs: Vec<Program>,
    /// The program currently being scanned (actions fire, transitions
    /// evaluate against this cursor).
    current: usize,
    options: RolloutOptions,

    t: f64,
    /// Per-robot runtimes, in scene order (the deterministic advance order).
    robots: Vec<RobotRuntime>,
    sensors: Vec<SensorRuntime>,
    devices: Vec<DeviceRuntime>,

    // Accumulating outputs.
    objects: Vec<ObjectTrack>,
    signals: Vec<BoolTrack>,
    step_spans: Vec<StepSpan>,
}

/// The motion currently driving the joints, sampled per scan tick.
enum ActiveMove {
    Traj {
        start: f64,
        traj: JointTrajectory,
    },
    Ramp {
        start: f64,
        duration: f64,
        from: Vec<f64>,
        to: Vec<f64>,
    },
}

impl ActiveMove {
    fn sample(&self, t: f64) -> Vec<f64> {
        match self {
            ActiveMove::Traj { start, traj } => traj.sample(t - start),
            ActiveMove::Ramp {
                start,
                duration,
                from,
                to,
            } => {
                let u = ((t - start) / duration).clamp(0.0, 1.0);
                // Rest-to-rest cubic (matches the baked Hermite waypoints).
                let s = u * u * (3.0 - 2.0 * u);
                from.iter().zip(to).map(|(a, b)| a + (b - a) * s).collect()
            }
        }
    }

    /// Absolute end time of this move.
    fn end(&self) -> f64 {
        match self {
            ActiveMove::Traj { start, traj } => start + traj.duration(),
            ActiveMove::Ramp {
                start, duration, ..
            } => start + duration,
        }
    }
}

/// Per-tick tracking solve: warm-started from the nominal configuration,
/// so it only has to absorb one scan period of part motion. No restarts —
/// a restart could land on another solution branch and teleport the arm
/// mid-track. Null-space centering is off for the same reason: the track
/// must reproduce the taught posture carried by the offset, not add its
/// own self-motion drift on top (a deliberate bias would belong to the
/// authoring layer, not the per-tick follow).
const TRACK_IK: botrail_kin::IkOptions = botrail_kin::IkOptions {
    mode: botrail_kin::IkMode::Pose,
    max_iters: 100,
    tol_pos: 1e-7,
    tol_rot: 1e-6,
    damping: 0.05,
    orientation_weight: 0.5,
    max_step: 0.5,
    restarts: 0,
    null_space_gain: 0.0,
};

/// An active conveyor track: taught poses are carried by `offset`, which is
/// the part's rigid motion since the latch — recomputed every tick until the
/// part is grasped, then frozen (from then on it is the robot moving it).
struct TrackLatch {
    object: String,
    link: usize,
    origin: Isometry3<f64>,
    offset: Isometry3<f64>,
    frozen: bool,
    /// The program that latched the track — mid-track failures surface
    /// during the world advance, when no program is being scanned, and
    /// still have to be pinned on the right one.
    program: usize,
}

struct SensorRuntime {
    collider: ObstacleCollider,
    /// Authored pose — world for a fixture, vehicle-frame for a mounted one.
    pose: Isometry3<f64>,
    /// Device index of the vehicle this rides on, if any.
    mount: Option<usize>,
    watch: SensorWatch,
    /// Index of this sensor's lane in the signal tracks.
    lane: usize,
}

/// Is `point` inside the box centred on `pose` with half-extents `half`?
fn inside_zone(pose: &Isometry3<f64>, half: &Vector3<f64>, point: &Isometry3<f64>) -> bool {
    let local = pose
        .inverse()
        .transform_point(&nalgebra::Point3::from(point.translation.vector));
    local.x.abs() <= half.x && local.y.abs() <= half.y && local.z.abs() <= half.z
}

enum DeviceRuntime {
    Conveyor {
        name: String,
        zone_pose: Isometry3<f64>,
        zone_half: Vector3<f64>,
        velocity: Vector3<f64>,
        running: bool,
        /// Metres left of a fixed `Advance`; `None` when none is pending.
        /// While pending the belt runs regardless of `running`, and the
        /// final tick consumes exactly the remainder.
        remaining: Option<f64>,
        lane: usize,
    },
    Axis {
        name: String,
        objects: Vec<String>,
        axis: Vector3<f64>,
        speed: f64,
        position: f64,
        target: f64,
        lane: usize,
    },
    Source {
        name: String,
        pool: Vec<String>,
        park: Isometry3<f64>,
        pitch: Vector3<f64>,
        pose: Isometry3<f64>,
        interval: f64,
        running: bool,
        /// Seconds until the next release is due.
        due: f64,
        /// Per pool member: waiting in the magazine rather than out on the
        /// line. Seeded from where the scene put it, so a belt authored
        /// already-loaded stays loaded.
        parked: Vec<bool>,
        lane: usize,
    },
    Sink {
        name: String,
        zone_pose: Isometry3<f64>,
        zone_half: Vector3<f64>,
        /// Index into `devices` of the source this returns members to.
        source: usize,
    },
    Vehicle {
        name: String,
        waypoints: Vec<nalgebra::Point2<f64>>,
        stations: Vec<(String, usize)>,
        ring: bool,
        body: Vec<String>,
        speed: f64,
        turn_speed: f64,
        allow_reverse: bool,
        /// Load deck in the vehicle frame (pose, half-extents).
        tray: Option<(Isometry3<f64>, Vector3<f64>)>,
        /// SE(2) reference frame: floor position + yaw. The body rides this
        /// frame's rigid motion; its own z never changes.
        position: nalgebra::Point2<f64>,
        heading: f64,
        /// Waypoint index the vehicle is parked at (`None` while travelling).
        at: Option<usize>,
        /// Commanded station's waypoint index (`DeviceDone` = parked there).
        target: usize,
        /// Remaining legs of the active goto, front first.
        legs: std::collections::VecDeque<Leg>,
        lane: usize,
    },
}

/// One piece of a vehicle's route.
enum Leg {
    /// Pivot in place to the absolute heading `to` at signed rate `omega`.
    Turn { to: f64, omega: f64 },
    /// Drive straight to `to` along unit `dir`.
    Straight {
        to: nalgebra::Point2<f64>,
        dir: nalgebra::Vector2<f64>,
    },
}

/// The net rigid motion a vehicle applied over one sub-interval of a tick.
enum VehiclePiece {
    Lin {
        velocity: Vector3<f64>,
    },
    Piv {
        center: nalgebra::Point3<f64>,
        omega: f64,
    },
}

/// One travelling vehicle's tick worth of motion: its name, body members,
/// the load deck as it stood at the *start* of the tick (whatever was on it
/// then rides), and the exact sub-tick `(τ0, τ1, piece)` intervals.
struct VehicleMove {
    name: String,
    body: Vec<String>,
    tray: Option<(Isometry3<f64>, Vector3<f64>)>,
    pieces: Vec<(f64, f64, VehiclePiece)>,
}

/// Wraps to `(-π, π]` — the exact-π case turns counter-clockwise, fixed, so
/// a 180° about-face is deterministic.
fn wrap_angle(a: f64) -> f64 {
    let mut a = a % std::f64::consts::TAU;
    if a <= -std::f64::consts::PI {
        a += std::f64::consts::TAU;
    } else if a > std::f64::consts::PI {
        a -= std::f64::consts::TAU;
    }
    a
}

/// Waypoint indices from `from` to `to` (exclusive of `from`, inclusive of
/// `to`): the straight index walk on an open path; on a ring, whichever way
/// around is shorter by distance (ties go forward).
fn vehicle_route(
    waypoints: &[nalgebra::Point2<f64>],
    ring: bool,
    from: usize,
    to: usize,
) -> Vec<usize> {
    if from == to {
        return Vec::new();
    }
    if !ring {
        return if from < to {
            ((from + 1)..=to).collect()
        } else {
            (to..from).rev().collect()
        };
    }
    let n = waypoints.len();
    let walk_by = |step: usize| {
        let mut walk = Vec::new();
        let mut i = from;
        while i != to {
            i = (i + step) % n;
            walk.push(i);
        }
        walk
    };
    let forward = walk_by(1);
    let backward = walk_by(n - 1);
    let length = |walk: &[usize]| {
        let mut prev = from;
        let mut total = 0.0;
        for &i in walk {
            total += (waypoints[i] - waypoints[prev]).norm();
            prev = i;
        }
        total
    };
    if length(&backward) + 1e-12 < length(&forward) {
        backward
    } else {
        forward
    }
}

/// Expands a route into turn/straight legs from the vehicle's current
/// heading. Coincident waypoints contribute no leg.
fn build_legs(
    waypoints: &[nalgebra::Point2<f64>],
    route: &[usize],
    start: nalgebra::Point2<f64>,
    heading: f64,
    turn_speed: f64,
    allow_reverse: bool,
) -> std::collections::VecDeque<Leg> {
    let mut legs = std::collections::VecDeque::new();
    let mut position = start;
    let mut heading = heading;
    for &i in route {
        let to = waypoints[i];
        let d = to - position;
        if d.norm() < 1e-9 {
            continue;
        }
        let travel = d.y.atan2(d.x);
        // Backing up is just facing the other way while travelling the same
        // direction — worth it whenever it is the shorter turn, which is
        // exactly when a machine would reverse rather than turn around.
        let leg_heading =
            if allow_reverse && wrap_angle(travel - heading).abs() > std::f64::consts::FRAC_PI_2 {
                wrap_angle(travel + std::f64::consts::PI)
            } else {
                travel
            };
        let dphi = wrap_angle(leg_heading - heading);
        if dphi.abs() > 1e-9 {
            legs.push_back(Leg::Turn {
                to: leg_heading,
                omega: dphi.signum() * turn_speed,
            });
        }
        legs.push_back(Leg::Straight {
            to,
            dir: d / d.norm(),
        });
        heading = leg_heading;
        position = to;
    }
    legs
}

impl Rollout {
    fn new(world: Scene, sequences: Vec<Sequence>, options: RolloutOptions) -> Self {
        // Signal lanes: internal relays, then sensor inputs, then device
        // outputs — all recorded as edge tracks for the timing chart.
        let mut signals: Vec<BoolTrack> = world
            .signals()
            .iter()
            .map(|s| BoolTrack {
                name: s.name.clone(),
                edges: vec![(0.0, s.initial)],
            })
            .collect();
        let sensors: Vec<SensorRuntime> = world
            .sensors()
            .iter()
            .map(|sensor| {
                let (collider, pose) = match &sensor.kind {
                    SensorKind::Zone { pose, size } => {
                        (ObstacleCollider::cuboid(size / 2.0), *pose)
                    }
                    SensorKind::Beam { from, to, radius } => (
                        ObstacleCollider::capsule(*from, *to, *radius),
                        Isometry3::identity(),
                    ),
                };
                let lane = signals.len();
                signals.push(BoolTrack {
                    name: sensor.name.clone(),
                    edges: vec![(0.0, false)],
                });
                SensorRuntime {
                    collider,
                    pose,
                    mount: sensor
                        .mount
                        .as_ref()
                        .and_then(|name| world.devices().iter().position(|d| &d.name == name)),
                    watch: sensor.watch.clone(),
                    lane,
                }
            })
            .collect();
        let devices: Vec<DeviceRuntime> = world
            .devices()
            .iter()
            .map(|device| {
                let lane = signals.len();
                match &device.kind {
                    DeviceKind::Conveyor {
                        zone_pose,
                        zone_size,
                        velocity,
                        running,
                    } => {
                        signals.push(BoolTrack {
                            name: device.name.clone(),
                            edges: vec![(0.0, *running)],
                        });
                        DeviceRuntime::Conveyor {
                            name: device.name.clone(),
                            zone_pose: *zone_pose,
                            zone_half: zone_size / 2.0,
                            velocity: *velocity,
                            running: *running,
                            remaining: None,
                            lane,
                        }
                    }
                    DeviceKind::LinearAxis {
                        objects,
                        axis,
                        speed,
                        position,
                        ..
                    } => {
                        signals.push(BoolTrack {
                            name: device.name.clone(),
                            edges: vec![(0.0, false)],
                        });
                        DeviceRuntime::Axis {
                            name: device.name.clone(),
                            objects: objects.clone(),
                            axis: axis.into_inner(),
                            speed: *speed,
                            position: *position,
                            target: *position,
                            lane,
                        }
                    }
                    DeviceKind::Source {
                        pool,
                        park,
                        pitch,
                        pose,
                        interval,
                        running,
                    } => {
                        signals.push(BoolTrack {
                            name: device.name.clone(),
                            edges: vec![(0.0, *running)],
                        });
                        // Seeded from the world: a member sitting on its
                        // parking slot is waiting, anything else is already
                        // out on the line.
                        let parked = pool
                            .iter()
                            .enumerate()
                            .map(|(i, name)| {
                                world
                                    .obstacles()
                                    .iter()
                                    .find(|o| &o.name == name)
                                    .is_some_and(|o| {
                                        let slot = park.translation.vector + pitch * i as f64;
                                        (o.pose.translation.vector - slot).norm() < 1e-6
                                    })
                            })
                            .collect();
                        DeviceRuntime::Source {
                            name: device.name.clone(),
                            pool: pool.clone(),
                            park: *park,
                            pitch: *pitch,
                            pose: *pose,
                            interval: *interval,
                            running: *running,
                            due: 0.0,
                            parked,
                            lane,
                        }
                    }
                    DeviceKind::Sink {
                        zone_pose,
                        zone_size,
                        source,
                    } => {
                        signals.push(BoolTrack {
                            name: device.name.clone(),
                            edges: vec![(0.0, false)],
                        });
                        DeviceRuntime::Sink {
                            name: device.name.clone(),
                            zone_pose: *zone_pose,
                            zone_half: zone_size / 2.0,
                            // Resolved after the pass: sources may come later
                            // in the list.
                            source: world
                                .devices()
                                .iter()
                                .position(|d| &d.name == source)
                                .unwrap_or(usize::MAX),
                        }
                    }
                    DeviceKind::Vehicle {
                        path,
                        body,
                        speed,
                        turn_speed,
                        start,
                        allow_reverse,
                        tray,
                    } => {
                        signals.push(BoolTrack {
                            name: device.name.clone(),
                            edges: vec![(0.0, false)],
                        });
                        // Validation vetted the station; a missing one only
                        // happens on unvalidated direct use, and parks at 0.
                        let at = path.station(start).unwrap_or(0);
                        let position = path
                            .waypoints
                            .get(at)
                            .copied()
                            .unwrap_or_else(nalgebra::Point2::origin);
                        let heading = path.heading_at(at);
                        DeviceRuntime::Vehicle {
                            name: device.name.clone(),
                            waypoints: path.waypoints.clone(),
                            stations: path.stations.clone(),
                            ring: path.ring,
                            body: body.clone(),
                            speed: *speed,
                            turn_speed: *turn_speed,
                            allow_reverse: *allow_reverse,
                            tray: tray.map(|(pose, size)| (pose, size / 2.0)),
                            position,
                            heading,
                            at: Some(at),
                            target: at,
                            legs: std::collections::VecDeque::new(),
                            lane,
                        }
                    }
                }
            })
            .collect();
        // Carriers waiting in a magazine at t = 0 are stock: stowed from
        // the start, so a run never opens on a pile of parts that then
        // teleport onto the line.
        let mut stowed: Vec<ObjectTrack> = Vec::new();
        for device in &devices {
            if let DeviceRuntime::Source {
                pool,
                parked,
                park,
                pitch,
                ..
            } = device
            {
                for (i, name) in pool.iter().enumerate() {
                    if !parked[i] {
                        continue;
                    }
                    let slot = Isometry3::from_parts(
                        nalgebra::Translation3::from(park.translation.vector + pitch * i as f64),
                        park.rotation,
                    );
                    stowed.push(ObjectTrack {
                        name: name.clone(),
                        spans: vec![TrackSpan::Stowed {
                            t0: 0.0,
                            t1: 0.0,
                            pose: slot,
                        }],
                    });
                }
            }
        }

        // Objects grasped before the sequence starts follow from t = 0.
        let objects = world
            .attachments()
            .iter()
            .map(|a| ObjectTrack {
                name: a.object.clone(),
                spans: vec![TrackSpan::Follow {
                    t0: 0.0,
                    t1: 0.0,
                    robot: a.robot,
                    link: a.link,
                    offset: a.grasp,
                }],
            })
            .chain(stowed)
            .collect();
        let robots = world
            .robots()
            .iter()
            .map(|sr| {
                let q = sr.joint_positions().to_vec();
                RobotRuntime {
                    times: vec![0.0],
                    positions: vec![q.clone()],
                    velocities: vec![vec![0.0; q.len()]],
                    q_nom: q.clone(),
                    q_nom_prev: q.clone(),
                    q,
                    active: None,
                    tracking: None,
                    moves: Vec::new(),
                    base: sr.mount.as_ref().map(|_| Vec::new()),
                }
            })
            .collect();
        Rollout {
            world,
            programs: sequences
                .into_iter()
                .map(|sequence| Program {
                    sequence,
                    step: 0,
                    entered_at: 0.0,
                    move_ends: Vec::new(),
                    open_span: 0,
                })
                .collect(),
            current: 0,
            options,
            t: 0.0,
            robots,
            sensors,
            devices,
            objects,
            signals,
            step_spans: Vec::new(),
        }
    }

    /// The step's display name, qualified by its program when several run
    /// — two stations both have a `feed`, and a timeline (or an error)
    /// that just says `feed` names neither.
    fn step_name_in(&self, program: usize, index: usize) -> String {
        let p = &self.programs[program];
        let step = p
            .sequence
            .steps
            .get(index)
            .map(|s| s.name.clone())
            .unwrap_or_default();
        if self.programs.len() == 1 {
            step
        } else {
            format!("{}/{step}", p.sequence.name)
        }
    }

    fn cur_step(&self) -> usize {
        self.programs[self.current].step
    }

    fn cur_step_name(&self) -> String {
        self.step_name_in(self.current, self.cur_step())
    }

    fn finished(&self) -> bool {
        self.programs.iter().all(Program::finished)
    }

    fn run(mut self) -> Result<SequenceTimeline, SeqError> {
        self.update_sensors();
        // First scan: every program enters its first step (actions fire in
        // declaration order), then every program chains through whatever
        // is instantaneously ready — so a signal set by program 1's first
        // step is already visible to program 2's first transition.
        for p in 0..self.programs.len() {
            self.current = p;
            self.enter_step()?;
        }
        for p in 0..self.programs.len() {
            self.current = p;
            self.advance_through_ready_steps()?;
        }

        let mut tick = 0u64;
        while !self.finished() {
            tick += 1;
            self.t = tick as f64 * self.options.dt;
            if self.t > self.options.max_duration {
                return Err(self.timeout());
            }
            // PLC scan: outputs advance the world through this tick, then
            // inputs are read, then transitions fire — for each program in
            // declaration order.
            self.advance_world()?;
            self.update_sensors();
            for p in 0..self.programs.len() {
                self.current = p;
                self.advance_through_ready_steps()?;
            }
        }
        Ok(self.finish())
    }

    /// The timeout error, naming where every unfinished program is stuck —
    /// with several programs that list is the deadlock diagnosis (A waits
    /// on a signal B never sets), so it has to name all of them.
    fn timeout(&self) -> SeqError {
        let waiting: Vec<usize> = (0..self.programs.len())
            .filter(|&p| !self.programs[p].finished())
            .collect();
        if self.programs.len() == 1 {
            let step = self.programs[0].step;
            return SeqError::Timeout {
                step,
                name: self.step_name_in(0, step),
                limit: self.options.max_duration,
            };
        }
        SeqError::ProgramsTimeout {
            at: waiting
                .iter()
                .map(|&p| self.step_name_in(p, self.programs[p].step))
                .collect::<Vec<_>>()
                .join(", "),
            limit: self.options.max_duration,
        }
    }

    /// Advances every robot's joints and every device by one scan period,
    /// then verifies the robots stayed clear of each other.
    fn advance_world(&mut self) -> Result<(), SeqError> {
        let t = self.t;
        let dt = self.options.dt;
        // Joints follow each robot's in-flight motion/ramp, in scene order
        // (attached obstacles are re-synced by set_joint_positions_for).
        for (r, rt) in self.robots.iter_mut().enumerate() {
            let Some(active) = &rt.active else { continue };
            rt.q_nom = active.sample(t);
            // Under a track the commanded joints are solved in
            // `follow_tracked_parts`, once this tick's part motion is known.
            if rt.tracking.is_none() {
                rt.q = rt.q_nom.clone();
                self.world
                    .set_joint_positions_for(r, rt.q.clone())
                    .expect("sampled q has robot DOF");
            }
            if t >= active.end() - 1e-9 {
                rt.active = None;
            }
        }

        // Devices move obstacles kinematically; grasped objects are bound
        // to the robot and never advected.
        let attached: Vec<String> = self
            .world
            .attachments()
            .iter()
            .map(|a| a.object.clone())
            .collect();
        let mut moved: Vec<(String, Isometry3<f64>, Vector3<f64>)> = Vec::new();
        // (source device index, object) pairs a sink caught this tick.
        let mut returned: Vec<(usize, String)> = Vec::new();
        let mut lane_updates: Vec<(usize, bool)> = Vec::new();
        // Per travelling vehicle: its exact sub-tick motion pieces, applied
        // to the body after this loop (span recording needs `&mut self`).
        let mut vehicle_moves: Vec<VehicleMove> = Vec::new();
        for device in &mut self.devices {
            match device {
                DeviceRuntime::Conveyor {
                    zone_pose,
                    zone_half,
                    velocity,
                    running,
                    remaining,
                    lane,
                    ..
                } => {
                    let speed = velocity.norm();
                    // A pending fixed advance runs the belt whatever the
                    // running flag says; the last tick moves exactly the
                    // remainder so the pitch never picks up a scan-period
                    // fraction. Otherwise plain free-running.
                    let tick_velocity = if let Some(rem) = remaining.take() {
                        if speed < 1e-12 {
                            // Speed zeroed mid-advance: nothing can move.
                            // The command stays pending (device_done stays
                            // false) rather than dividing by zero here.
                            *remaining = Some(rem);
                            None
                        } else {
                            let step = (speed * dt).min(rem);
                            let left = rem - step;
                            if left > 0.0 {
                                *remaining = Some(left);
                            } else {
                                lane_updates.push((*lane, false));
                            }
                            // Scaled so `v * dt` is exactly the (partial) step.
                            Some(*velocity * (step / (speed * dt)))
                        }
                    } else if *running && speed >= 1e-12 {
                        Some(*velocity)
                    } else {
                        None
                    };
                    let Some(tick_velocity) = tick_velocity else {
                        continue;
                    };
                    for obstacle in self.world.obstacles() {
                        if attached.iter().any(|a| a == &obstacle.name) {
                            continue;
                        }
                        let local = zone_pose.inverse().transform_point(&nalgebra::Point3::from(
                            obstacle.pose.translation.vector,
                        ));
                        if local.x.abs() <= zone_half.x
                            && local.y.abs() <= zone_half.y
                            && local.z.abs() <= zone_half.z
                        {
                            let mut pose = obstacle.pose;
                            pose.translation.vector += tick_velocity * dt;
                            moved.push((obstacle.name.clone(), pose, tick_velocity));
                        }
                    }
                }
                DeviceRuntime::Axis {
                    objects,
                    axis,
                    speed,
                    position,
                    target,
                    ..
                } => {
                    let remaining = *target - *position;
                    if remaining.abs() < 1e-12 || *speed <= 0.0 {
                        continue;
                    }
                    let step = remaining.abs().min(*speed * dt) * remaining.signum();
                    *position += step;
                    let delta = *axis * step;
                    // Span velocity from the actual per-tick displacement so
                    // the (partial) arrival tick samples exactly.
                    let velocity = *axis * (step / dt);
                    for name in objects.iter() {
                        if attached.iter().any(|a| a == name) {
                            continue;
                        }
                        if let Some(o) = self.world.obstacles().iter().find(|o| &o.name == name) {
                            let mut pose = o.pose;
                            pose.translation.vector += delta;
                            moved.push((name.clone(), pose, velocity));
                        }
                    }
                    // Arrival closes the axis's moving lane on this tick.
                    if (*target - *position).abs() < 1e-12 {
                        lane_updates.push((
                            match device {
                                DeviceRuntime::Axis { lane, .. } => *lane,
                                _ => unreachable!(),
                            },
                            false,
                        ));
                    }
                }
                DeviceRuntime::Sink {
                    zone_pose,
                    zone_half,
                    source,
                    ..
                } => {
                    for obstacle in self.world.obstacles() {
                        if attached.iter().any(|a| a == &obstacle.name) {
                            continue;
                        }
                        let local = zone_pose.inverse().transform_point(&nalgebra::Point3::from(
                            obstacle.pose.translation.vector,
                        ));
                        if local.x.abs() <= zone_half.x
                            && local.y.abs() <= zone_half.y
                            && local.z.abs() <= zone_half.z
                        {
                            returned.push((*source, obstacle.name.clone()));
                        }
                    }
                }
                DeviceRuntime::Vehicle {
                    name,
                    body,
                    speed,
                    tray,
                    position,
                    heading,
                    at,
                    target,
                    legs,
                    lane,
                    ..
                } => {
                    if legs.is_empty() {
                        continue;
                    }
                    // The deck as it stands *now*: what is on it at the top
                    // of the tick is what travels this tick. Capturing it
                    // before the walk also settles the ordering question —
                    // a part placed mid-tick joins on the next one.
                    let deck =
                        tray.map(|(pose, half)| (vehicle_frame(position, *heading) * pose, half));
                    // Walk the legs through this tick's time budget. Leg
                    // boundaries land at exact sub-tick instants, so the
                    // recorded spans (and any resample of them) are exact;
                    // world poses advance at tick boundaries like every
                    // other device.
                    let mut remaining = dt;
                    let mut pieces: Vec<(f64, f64, VehiclePiece)> = Vec::new();
                    while remaining > 1e-12 {
                        let Some(leg) = legs.front() else { break };
                        let tau0 = t - remaining;
                        match leg {
                            Leg::Turn { to, omega } => {
                                let left = wrap_angle(to - *heading);
                                let need = left / omega; // same sign ⇒ ≥ 0
                                if need <= 1e-12 {
                                    *heading = *to;
                                    legs.pop_front();
                                    continue;
                                }
                                let step = need.min(remaining);
                                pieces.push((
                                    tau0,
                                    tau0 + step,
                                    VehiclePiece::Piv {
                                        center: nalgebra::Point3::new(position.x, position.y, 0.0),
                                        omega: *omega,
                                    },
                                ));
                                if step >= need - 1e-12 {
                                    *heading = *to;
                                    legs.pop_front();
                                } else {
                                    *heading = wrap_angle(*heading + omega * step);
                                }
                                remaining -= step;
                            }
                            Leg::Straight { to, dir } => {
                                let need = (to - *position).norm() / *speed;
                                if need <= 1e-12 {
                                    *position = *to;
                                    legs.pop_front();
                                    continue;
                                }
                                let step = need.min(remaining);
                                pieces.push((
                                    tau0,
                                    tau0 + step,
                                    VehiclePiece::Lin {
                                        velocity: Vector3::new(dir.x * *speed, dir.y * *speed, 0.0),
                                    },
                                ));
                                if step >= need - 1e-12 {
                                    *position = *to;
                                    legs.pop_front();
                                } else {
                                    *position += *dir * (*speed * step);
                                }
                                remaining -= step;
                            }
                        }
                    }
                    // A tick fully consumed ends exactly at t (float dust in
                    // `t - remaining + step` would otherwise leave the span
                    // an ulp short and trip the settle check mid-route).
                    if remaining <= 1e-12 {
                        if let Some(last) = pieces.last_mut() {
                            last.1 = t;
                        }
                    }
                    if legs.is_empty() {
                        *at = Some(*target);
                        lane_updates.push((*lane, false));
                    }
                    if !pieces.is_empty() {
                        vehicle_moves.push(VehicleMove {
                            name: name.clone(),
                            body: body.clone(),
                            tray: deck,
                            pieces,
                        });
                    }
                }
                // Releases are a second pass: a member returned this tick
                // should be feedable this tick, which needs the returns in
                // before any source looks at its magazine.
                DeviceRuntime::Source { .. } => {}
            }
        }
        for (lane, value) in lane_updates {
            self.set_lane(lane, t, value);
        }
        for (name, pose, velocity) in &moved {
            let rest = self
                .world
                .obstacles()
                .iter()
                .find(|o| &o.name == name)
                .map(|o| o.pose)
                .expect("moved obstacle exists");
            self.world
                .set_obstacle_pose(name, *pose)
                .expect("moved obstacle exists");
            self.extend_linear_span(name, rest, *velocity);
        }
        // A part a conveyor already moved this tick is that conveyor's; one
        // device per part per tick keeps the bake single-valued.
        let advected: Vec<String> = moved.iter().map(|(n, _, _)| n.clone()).collect();
        let riders = self.apply_vehicle_moves(vehicle_moves, &advected)?;
        // Objects that stopped riding (zone exit / device stop / vehicle
        // arrival) settle into a hold at their current pose.
        let moved_names: Vec<&String> = moved
            .iter()
            .map(|(n, _, _)| n)
            .chain(riders.iter())
            .collect();
        let settled: Vec<(String, Isometry3<f64>)> = self
            .objects
            .iter()
            .filter(|track| {
                !moved_names.iter().any(|n| **n == track.name)
                    && matches!(
                        track.spans.last(),
                        Some(TrackSpan::Linear { t1, .. } | TrackSpan::Pivot { t1, .. }) if *t1 < t
                    )
            })
            .map(|track| track.name.clone())
            .filter_map(|name| {
                self.world
                    .obstacles()
                    .iter()
                    .find(|o| o.name == name)
                    .map(|o| (name.clone(), o.pose))
            })
            .collect();
        for (name, pose) in settled {
            let t_stop = match self
                .objects
                .iter()
                .find(|tr| tr.name == name)
                .and_then(|tr| tr.spans.last())
            {
                Some(TrackSpan::Linear { t1, .. } | TrackSpan::Pivot { t1, .. }) => *t1,
                _ => t,
            };
            let track = self.object_track_at(&name, pose, t_stop);
            track.spans.push(TrackSpan::Hold {
                t0: t_stop,
                t1: t_stop,
                pose,
            });
        }
        self.return_to_magazines(returned);
        self.feed_from_sources(dt);
        self.follow_tracked_parts()?;
        self.check_robot_collisions()?;
        Ok(())
    }

    /// Sinks: put what reached them back in the magazine it came from.
    fn return_to_magazines(&mut self, returned: Vec<(usize, String)>) {
        for (source, object) in returned {
            let Some(DeviceRuntime::Source {
                pool,
                park,
                pitch,
                parked,
                ..
            }) = self.devices.get_mut(source)
            else {
                continue;
            };
            let Some(i) = pool.iter().position(|n| *n == object) else {
                // Something else drifted through the sink; a sink only owns
                // its source's carriers.
                continue;
            };
            if parked[i] {
                continue;
            }
            parked[i] = true;
            let slot = Isometry3::from_parts(
                nalgebra::Translation3::from(park.translation.vector + *pitch * i as f64),
                park.rotation,
            );
            self.teleport_object(&object, slot, true);
        }
    }

    /// Sources: release the next waiting carrier when one is due.
    fn feed_from_sources(&mut self, dt: f64) {
        for i in 0..self.devices.len() {
            let (object, pose, lane_off) = {
                let DeviceRuntime::Source {
                    pool,
                    pose,
                    interval,
                    running,
                    due,
                    parked,
                    lane,
                    ..
                } = &mut self.devices[i]
                else {
                    continue;
                };
                if !*running {
                    continue;
                }
                *due -= dt;
                if *due > 1e-12 {
                    continue;
                }
                let Some(next) = parked.iter().position(|p| *p) else {
                    // Magazine empty: stay due, so the next return feeds
                    // immediately rather than waiting out another interval.
                    *due = 0.0;
                    continue;
                };
                parked[next] = false;
                let lane = *lane;
                let one_shot = *interval <= 0.0;
                if one_shot {
                    // An indexing feeder: one carrier per Start. Sequences
                    // that name the carrier each step takes need supply to
                    // be demand-driven — on a timer, a carrier released
                    // while the cell is busy elsewhere goes past unclaimed
                    // and every later step is off by one.
                    *running = false;
                } else {
                    *due = *interval;
                }
                (pool[next].clone(), *pose, one_shot.then_some(lane))
            };
            self.teleport_object(&object, pose, false);
            if let Some(lane) = lane_off {
                let t = self.t;
                self.set_lane(lane, t, false);
            }
        }
    }

    /// Records an instantaneous jump: whatever span was open closes here and
    /// a hold at the new pose begins. Teleports are how a magazine takes a
    /// carrier back and how a source puts one on the line — the alternative,
    /// creating and destroying objects, has no place in a timeline whose
    /// object tracks are a fixed named set.
    fn teleport_object(&mut self, name: &str, to: Isometry3<f64>, stowed: bool) {
        let t = self.t;
        let from = self
            .world
            .obstacles()
            .iter()
            .find(|o| o.name == name)
            .map(|o| o.pose);
        let Some(from) = from else { return };
        self.world
            .set_obstacle_pose(name, to)
            .expect("teleported obstacle exists");
        let track = self.object_track_at(name, from, t);
        if let Some(open) = track.spans.last_mut() {
            let end = open.end_mut();
            if *end < t {
                *end = t;
            }
        }
        track.spans.push(if stowed {
            TrackSpan::Stowed {
                t0: t,
                t1: t,
                pose: to,
            }
        } else {
            TrackSpan::Hold {
                t0: t,
                t1: t,
                pose: to,
            }
        });
    }

    /// Conveyor tracking: re-solve each latched arm so this tick's
    /// commanded pose is the nominal one carried by the part's motion since
    /// the latch. Runs after the devices have moved the world, so the
    /// robots see the parts where they are *now*; robots resolve in scene
    /// order (deterministic).
    fn follow_tracked_parts(&mut self) -> Result<(), SeqError> {
        for r in 0..self.robots.len() {
            self.follow_tracked_part(r)?;
        }
        Ok(())
    }

    fn follow_tracked_part(&mut self, r: usize) -> Result<(), SeqError> {
        let Some(latch) = &self.robots[r].tracking else {
            return Ok(());
        };
        let (object, link, origin, frozen) =
            (latch.object.clone(), latch.link, latch.origin, latch.frozen);
        // Failures here happen during the world advance, between program
        // scans — attribute them to the program that latched the track.
        self.current = latch.program;
        // A grasped part is carried by the robot itself, so following it
        // would chase its own tail: the offset it had at the grasp stands.
        let offset = if frozen {
            latch.offset
        } else {
            let pose = self
                .world
                .obstacles()
                .iter()
                .find(|o| o.name == object)
                .map(|o| o.pose)
                .ok_or_else(|| SeqError::Action {
                    step: self.cur_step(),
                    name: self.cur_step_name(),
                    message: format!("tracked obstacle `{object}` disappeared"),
                })?;
            let offset = pose * origin.inverse();
            if let Some(latch) = &mut self.robots[r].tracking {
                latch.offset = offset;
            }
            offset
        };

        let rt = &self.robots[r];
        let nominal = self
            .world
            .fk_for(r, &rt.q_nom)
            .expect("q_nom has robot DOF")[link];
        let target = offset * nominal;
        // Warm start from what the robot did last tick plus this tick's
        // nominal increment: the solve then only absorbs one scan period of
        // part motion (and joints the offset cannot touch — the gripper —
        // follow the nominal exactly).
        let seed: Vec<f64> =
            rt.q.iter()
                .zip(&rt.q_nom)
                .zip(&rt.q_nom_prev)
                .map(|((commanded, nominal), previous)| commanded + (nominal - previous))
                .collect();
        let result = self
            .world
            .solve_ik_world_for(r, link, &target, &seed, &TRACK_IK)
            .expect("seed has robot DOF");
        if !result.converged {
            return Err(SeqError::Action {
                step: self.cur_step(),
                name: self.cur_step_name(),
                message: format!(
                    "tracking `{object}`: the part ran out of reach at t = {:.2}s \
                     ({:.3} mm / {:.4} rad short after {} iterations)",
                    self.t,
                    result.pos_error * 1e3,
                    result.rot_error,
                    result.iters
                ),
            });
        }
        let rt = &mut self.robots[r];
        let previous = rt.q.clone();
        rt.q = result.q;
        rt.q_nom_prev = rt.q_nom.clone();
        self.world
            .set_joint_positions_for(r, rt.q.clone())
            .expect("solved q has robot DOF");
        // The move's own waypoints know nothing about the offset, so a
        // tracked tick bakes itself (velocities by difference).
        let dt = self.options.dt;
        let velocity =
            rt.q.iter()
                .zip(&previous)
                .map(|(now, before)| (now - before) / dt)
                .collect();
        let (t, q) = (self.t, rt.q.clone());
        rt.append_waypoint(t, q, velocity);
        Ok(())
    }

    /// After every robot advanced: no two robots (or the objects they
    /// carry) may touch. Only pairs spanning two different robots count —
    /// self-collisions and static-obstacle contacts stay the planner's
    /// concern.
    fn check_robot_collisions(&self) -> Result<(), SeqError> {
        if self.world.robots().len() < 2 {
            return Ok(());
        }
        // Which robot a collider belongs to: its own links, or the robot
        // carrying it (attached ids are remapped to obstacles by Scene).
        let side = |id: botrail_collide::ColliderId| -> Option<(usize, String)> {
            match id {
                botrail_collide::ColliderId::Link { robot, link } => Some((
                    robot,
                    self.world.robots()[robot].model.links[link].name.clone(),
                )),
                botrail_collide::ColliderId::Obstacle(k) => {
                    let name = &self.world.obstacles()[k].name;
                    self.world.attachment(name).map(|a| (a.robot, name.clone()))
                }
                botrail_collide::ColliderId::Attached(_) => {
                    unreachable!("attached ids are remapped by Scene")
                }
            }
        };
        // The dedicated cross-robot path: a full scene check also prices
        // every self-collision and every obstacle contact — all discarded
        // here — and at line scale that bill *was* the tick.
        for pair in self.world.check_cross_robot_collisions() {
            if let (Some((ra, link_a)), Some((rb, link_b))) = (side(pair.a), side(pair.b)) {
                if ra != rb {
                    return Err(SeqError::RobotCollision {
                        t: self.t,
                        a: self.world.robots()[ra].name.clone(),
                        b: self.world.robots()[rb].name.clone(),
                        link_a,
                        link_b,
                    });
                }
            }
        }
        Ok(())
    }

    /// Latches robot `r` onto `object`: from here its nominal poses ride
    /// the part's motion.
    fn latch_track(&mut self, r: usize, object: &str, link: Option<&str>) -> Result<(), SeqError> {
        let err = |message: String| SeqError::Action {
            step: self.cur_step(),
            name: self.cur_step_name(),
            message,
        };
        let model = &self.world.robots()[r].model;
        let link = match link {
            Some(name) => model
                .link_index(name)
                .ok_or_else(|| err(format!("unknown link `{name}`")))?,
            // The wrist, not the fingertip: a pose says nothing about the
            // grip, so the solver must not be able to spend it.
            None => model.tool_mount_link(),
        };
        let origin = self
            .world
            .obstacles()
            .iter()
            .find(|o| o.name == object)
            .map(|o| o.pose)
            .ok_or_else(|| err(format!("unknown obstacle `{object}`")))?;
        let rt = &mut self.robots[r];
        rt.q_nom = rt.q.clone();
        rt.q_nom_prev = rt.q.clone();
        let program = self.current;
        rt.tracking = Some(TrackLatch {
            program,
            object: object.to_string(),
            link,
            origin,
            offset: Isometry3::identity(),
            frozen: false,
        });
        Ok(())
    }

    /// Drops robot `r`'s track; the robot keeps the configuration it is in,
    /// so the nominal frame is re-based onto it (releasing never moves the
    /// robot).
    fn release_track(&mut self, r: usize) {
        let rt = &mut self.robots[r];
        rt.tracking = None;
        rt.q_nom = rt.q.clone();
    }

    /// Extends (or opens) a constant-velocity span covering this tick.
    fn extend_linear_span(
        &mut self,
        name: &str,
        rest_pose: Isometry3<f64>,
        velocity: Vector3<f64>,
    ) {
        let t = self.t;
        let dt = self.options.dt;
        let track = self.object_track_at(name, rest_pose, t - dt);
        match track.spans.last_mut() {
            Some(TrackSpan::Linear {
                t1, velocity: v, ..
            }) if (*v - velocity).norm() < 1e-12 && (*t1 - (t - dt)).abs() < 1e-9 => {
                *t1 = t;
            }
            _ => {
                if let Some(open) = track.spans.last_mut() {
                    let end = open.end_mut();
                    if *end < t - dt {
                        *end = t - dt;
                    }
                }
                track.spans.push(TrackSpan::Linear {
                    t0: t - dt,
                    t1: t,
                    from: rest_pose,
                    velocity,
                });
            }
        }
    }

    /// Applies each travelling vehicle's tick motion to its body obstacles
    /// — composing the exact sub-tick pieces onto the world poses and
    /// recording them as Linear/Pivot spans — then runs the aisle check.
    /// Returns the names that rode a deck this tick (they settle like any
    /// other carried part when the vehicle stops).
    fn apply_vehicle_moves(
        &mut self,
        moves: Vec<VehicleMove>,
        advected: &[String],
    ) -> Result<Vec<String>, SeqError> {
        // Whatever sits on a deck travels exactly like the body does, so the
        // two are carried by the same code — the only difference is that the
        // load is worked out per tick and the body is fixed.
        let moves: Vec<(VehicleMove, Vec<String>)> = moves
            .into_iter()
            .map(|mv| {
                let riders = match &mv.tray {
                    None => Vec::new(),
                    Some((zone, half)) => self
                        .world
                        .obstacles()
                        .iter()
                        .filter(|o| {
                            !mv.body.iter().any(|b| b == &o.name)
                                && self.world.attachment(&o.name).is_none()
                                && !advected.iter().any(|n| n == &o.name)
                                && inside_zone(zone, half, &o.pose)
                        })
                        .map(|o| o.name.clone())
                        .collect(),
                };
                (mv, riders)
            })
            .collect();
        for (mv, riders) in &moves {
            let (body, pieces) = (&mv.body, &mv.pieces);
            for member in body.iter().chain(riders) {
                if self.world.attachment(member).is_some() {
                    // Grasped body parts are the robot's problem, like any
                    // other advection exclusion.
                    continue;
                }
                let Some(mut pose) = self
                    .world
                    .obstacles()
                    .iter()
                    .find(|o| &o.name == member)
                    .map(|o| o.pose)
                else {
                    continue;
                };
                for (tau0, tau1, piece) in pieces {
                    let from = pose;
                    pose = apply_piece(&from, piece, tau1 - tau0);
                    self.extend_vehicle_span(member, from, *tau0, *tau1, piece);
                }
                self.world
                    .set_obstacle_pose(member, pose)
                    .expect("body member exists");
            }
            // A robot riding this vehicle moves by the same pieces. Deriving
            // its base this way rather than recomposing `frame ∘ offset`
            // keeps it *exactly* rigid with the body it is bolted to, which
            // is what any check of the pair will ask.
            for r in 0..self.world.robots().len() {
                let rides = self.world.robots()[r]
                    .mount
                    .as_ref()
                    .is_some_and(|m| m.device == mv.name);
                if !rides {
                    continue;
                }
                let mut base = *self.world.robots()[r].base_pose();
                for (tau0, tau1, piece) in pieces {
                    let from = base;
                    base = apply_piece(&from, piece, tau1 - tau0);
                    if let Some(spans) = self.robots[r].base.as_mut() {
                        push_vehicle_span(spans, from, *tau0, *tau1, piece);
                    }
                }
                self.world.set_robot_base_pose_for(r, base);
            }
        }
        self.check_vehicle_collisions(&moves)?;
        Ok(moves.into_iter().flat_map(|(_, riders)| riders).collect())
    }

    /// Extends (or opens) the exact sub-tick span covering one vehicle
    /// piece for a carried obstacle.
    fn extend_vehicle_span(
        &mut self,
        name: &str,
        from: Isometry3<f64>,
        tau0: f64,
        tau1: f64,
        piece: &VehiclePiece,
    ) {
        if tau1 - tau0 < 1e-12 {
            return;
        }
        let track = self.object_track_at(name, from, tau0);
        push_vehicle_span(&mut track.spans, from, tau0, tau1, piece);
    }

    /// A travelling vehicle's body must clear everything it does not carry
    /// — the aisle check. Only vehicles that moved this tick are checked,
    /// so a parked vehicle touching its dock guide is legitimate authoring.
    /// The load is checked too, and counts as carried: a part on the deck
    /// may touch the body it rides on, but not the shelf it passes.
    fn check_vehicle_collisions(
        &self,
        moves: &[(VehicleMove, Vec<String>)],
    ) -> Result<(), SeqError> {
        for (mv, riders) in moves {
            let (vehicle, body) = (&mv.name, &mv.body);
            let carried = |name: &String| body.contains(name) || riders.contains(name);
            for member in body.iter().chain(riders) {
                let Some((k, obstacle)) = self
                    .world
                    .obstacles()
                    .iter()
                    .enumerate()
                    .find(|(_, o)| &o.name == member)
                else {
                    continue;
                };
                if !obstacle.enabled || self.world.attachment(member).is_some() {
                    continue;
                }
                let collider = &self.world.obstacle_colliders[k];
                for (j, other) in self.world.obstacles().iter().enumerate() {
                    if j == k
                        || !other.enabled
                        || carried(&other.name)
                        || self.world.attachment(&other.name).is_some()
                    {
                        continue;
                    }
                    if collider.intersects(
                        &obstacle.pose,
                        &self.world.obstacle_colliders[j],
                        &other.pose,
                    ) {
                        return Err(SeqError::VehicleCollision {
                            t: self.t,
                            vehicle: vehicle.clone(),
                            body: member.clone(),
                            obstacle: other.name.clone(),
                        });
                    }
                }
            }
        }
        Ok(())
    }

    /// Evaluates every pseudo-sensor at the current world state and records
    /// edges on its input lane.
    fn update_sensors(&mut self) {
        if self.sensors.is_empty() {
            return;
        }
        let needs_robot = self.sensors.iter().any(|s| {
            matches!(
                s.watch,
                SensorWatch::Robot | SensorWatch::Robots(_) | SensorWatch::All
            )
        });
        let link_poses = needs_robot.then(|| self.world.all_link_poses());
        let t = self.t;
        let mut edges = Vec::new();
        for sensor in &self.sensors {
            // A mounted sensor's geometry is authored in its vehicle's
            // frame, so its world pose is re-resolved every tick — that is
            // the whole difference between a fixture and one that travels.
            let pose = match sensor.mount.and_then(|d| self.devices.get(d)) {
                Some(DeviceRuntime::Vehicle {
                    position, heading, ..
                }) => vehicle_frame(position, *heading) * sensor.pose,
                _ => sensor.pose,
            };
            let mut value = false;
            let watch_objects: Option<&[String]> = match &sensor.watch {
                SensorWatch::Objects(names) => Some(names),
                SensorWatch::AllObjects | SensorWatch::All => None,
                SensorWatch::Robot | SensorWatch::Robots(_) => Some(&[]),
            };
            if !matches!(sensor.watch, SensorWatch::Robot | SensorWatch::Robots(_)) {
                for (obstacle, collider) in self
                    .world
                    .obstacles()
                    .iter()
                    .zip(self.world.obstacle_colliders.iter())
                {
                    if !obstacle.enabled {
                        continue;
                    }
                    if let Some(names) = watch_objects {
                        if !names.iter().any(|n| n == &obstacle.name) {
                            continue;
                        }
                    }
                    if sensor.collider.intersects(&pose, collider, &obstacle.pose) {
                        value = true;
                        break;
                    }
                }
            }
            if !value {
                if let (
                    Some(poses),
                    SensorWatch::Robot | SensorWatch::Robots(_) | SensorWatch::All,
                ) = (&link_poses, &sensor.watch)
                {
                    let watched = |name: &str| match &sensor.watch {
                        SensorWatch::Robots(names) => names.iter().any(|n| n == name),
                        _ => true,
                    };
                    value = self.world.robots().iter().zip(poses).any(|(r, lp)| {
                        watched(&r.name)
                            && botrail_collide::robot_intersects(
                                r.collider(),
                                lp,
                                &sensor.collider,
                                &pose,
                            )
                    });
                }
            }
            edges.push((sensor.lane, value));
        }
        for (lane, value) in edges {
            self.set_lane(lane, t, value);
        }
    }

    /// Records an edge on a signal lane when the value changes.
    fn set_lane(&mut self, lane: usize, t: f64, value: bool) {
        let track = &mut self.signals[lane];
        let current = track.edges.last().map(|(_, v)| *v).unwrap_or(false);
        if current != value {
            track.edges.push((t, value));
        }
    }

    /// Fires transitions that hold at the current time for the current
    /// program, chaining through instantaneous steps (bounded per tick).
    fn advance_through_ready_steps(&mut self) -> Result<(), SeqError> {
        let mut chain = 0usize;
        while !self.programs[self.current].finished() {
            let p = &self.programs[self.current];
            let condition = p.sequence.steps[p.step].transition.clone();
            if !self.condition_holds(&condition) {
                return Ok(());
            }
            self.exit_step();
            let p = &mut self.programs[self.current];
            p.step += 1;
            if p.finished() {
                return Ok(());
            }
            chain += 1;
            if chain > self.options.immediate_chain_limit {
                return Err(SeqError::ImmediateLoop {
                    step: self.cur_step(),
                    name: self.cur_step_name(),
                    limit: self.options.immediate_chain_limit,
                });
            }
            self.enter_step()?;
        }
        Ok(())
    }

    fn condition_holds(&self, condition: &Condition) -> bool {
        match condition {
            Condition::Immediately => true,
            Condition::Done => self.programs[self.current]
                .move_ends
                .iter()
                .all(|end| self.t >= end - 1e-9),
            Condition::RobotDone { robot } => self
                .world
                .robot_index(robot)
                .map(|r| self.robots[r].active.is_none())
                .unwrap_or(true),
            Condition::Elapsed { seconds } => {
                self.t - self.programs[self.current].entered_at >= seconds - 1e-9
            }
            Condition::Signal { name, value } => {
                let current = self
                    .signals
                    .iter()
                    .find(|s| &s.name == name)
                    .map(|s| s.edges.last().map(|(_, v)| *v).unwrap_or(false))
                    .unwrap_or(false);
                current == *value
            }
            Condition::DeviceDone { device } => self.devices.iter().any(|d| match d {
                DeviceRuntime::Axis {
                    name,
                    position,
                    target,
                    ..
                } => name == device && (target - position).abs() < 1e-9,
                // Parked at the commanded station (in-position).
                DeviceRuntime::Vehicle { name, legs, .. } => name == device && legs.is_empty(),
                // In-position for a conveyor means "no fixed advance
                // pending" — the await for `Advance`.
                DeviceRuntime::Conveyor {
                    name, remaining, ..
                } => name == device && remaining.is_none(),
                // Nothing to be "done" with: these run continuously.
                DeviceRuntime::Source { .. } | DeviceRuntime::Sink { .. } => false,
            }),
            Condition::All(cs) => cs.iter().all(|c| self.condition_holds(c)),
            Condition::Any(cs) => cs.iter().any(|c| self.condition_holds(c)),
        }
    }

    fn enter_step(&mut self) -> Result<(), SeqError> {
        let name = self.cur_step_name();
        let program = &mut self.programs[self.current];
        program.entered_at = self.t;
        program.move_ends.clear();
        program.open_span = self.step_spans.len();
        self.step_spans.push(StepSpan {
            name,
            start: self.t,
            end: self.t,
        });
        let step = self.programs[self.current].step;
        for action in self.programs[self.current].sequence.steps[step]
            .actions
            .clone()
        {
            self.fire(&action)?;
        }
        Ok(())
    }

    fn exit_step(&mut self) {
        // Close *this program's* span: another program may have opened one
        // since, so "the last span" stopped being ours.
        let span = self.programs[self.current].open_span;
        if let Some(span) = self.step_spans.get_mut(span) {
            span.end = self.t;
        }
        // Hold every robot's last configuration up to the transition
        // instant, so the baked tracks stay exact through waits.
        let t = self.t;
        for rt in &mut self.robots {
            let (q, zeros) = (rt.q.clone(), vec![0.0; rt.q.len()]);
            rt.append_waypoint(t, q, zeros);
        }
    }

    /// Resolves an action's robot reference; validation already vetted it,
    /// so failures here are defensive.
    fn action_robot(&self, robot: &Option<String>) -> Result<usize, SeqError> {
        self.world
            .resolve_seq_robot(robot)
            .map_err(|message| SeqError::Action {
                step: self.cur_step(),
                name: self.cur_step_name(),
                message,
            })
    }

    fn fire(&mut self, action: &Action) -> Result<(), SeqError> {
        let step_index = self.cur_step();
        let step_name = self.cur_step_name();
        let err = move |message: String| SeqError::Action {
            step: step_index,
            name: step_name.clone(),
            message,
        };
        match action {
            Action::StartMotion { motion } => {
                let owner = self
                    .world
                    .motions()
                    .iter()
                    .find(|m| &m.name == motion)
                    .map(|m| m.robot)
                    .ok_or_else(|| err(format!("unknown motion `{motion}`")))?;
                // A plan is baked in world coordinates when it starts, so a
                // base that moves underneath it invalidates every waypoint.
                // Same rule as planning under a conveyor track, and the same
                // escape: ramps are re-evaluated per tick, so those may run
                // while the machine drives (stow the arm and go).
                if let Some(mount) = self.world.robots()[owner].mount.clone() {
                    let travelling = self.devices.iter().any(|d| match d {
                        DeviceRuntime::Vehicle { name, legs, .. } => {
                            *name == mount.device && !legs.is_empty()
                        }
                        _ => false,
                    });
                    if travelling {
                        return Err(err(format!(
                            "motion `{motion}` cannot start while `{}` is driving: plans are \
                             baked in world coordinates, so wait for device_done first \
                             (a ramp may run while travelling)",
                            mount.device
                        )));
                    }
                }
                // Plan against the world as it stands *now*: current q,
                // moved obstacles, live grasps — the other robots are
                // frozen collision bodies at their current configuration.
                self.world
                    .set_joint_positions_for(owner, self.robots[owner].q.clone())
                    .map_err(|e| err(e.to_string()))?;
                let limits = crate::motion::traj_limits(&self.world.robots()[owner].model);
                let planned = self
                    .world
                    .plan_motion(motion, &self.options.plan, &limits)
                    .map_err(|e| SeqError::PlanFailed {
                        step: self.cur_step(),
                        name: self.cur_step_name(),
                        message: e.to_string(),
                    })?;
                let traj = planned.trajectory;
                let rt = &mut self.robots[owner];
                for i in 0..traj.times.len() {
                    rt.append_waypoint(
                        self.t + traj.times[i],
                        traj.positions[i].clone(),
                        traj.velocities[i].clone(),
                    );
                }
                let end = self.t + traj.duration();
                self.programs[self.current].move_ends.push(end);
                rt.moves.push(StepSpan {
                    name: motion.clone(),
                    start: self.t,
                    end,
                });
                // Joints follow the trajectory tick by tick (advance_world),
                // so mid-motion sensors see the true robot state.
                rt.active = Some(ActiveMove::Traj {
                    start: self.t,
                    traj,
                });
            }
            Action::StartRamp {
                robot,
                targets,
                duration,
            } => {
                let r = self.action_robot(robot)?;
                let model = self.world.robots()[r].model.clone();
                let rt = &mut self.robots[r];
                let mut goal = rt.q_nom.clone();
                for (joint, value) in targets {
                    let ji = model
                        .joint_index(joint)
                        .ok_or_else(|| err(format!("unknown joint `{joint}`")))?;
                    let qi = model.joints[ji]
                        .q_index
                        .ok_or_else(|| err(format!("joint `{joint}` is not actuated")))?;
                    goal[qi] = *value;
                }
                // Two rest-to-rest waypoints: cubic Hermite eases in/out.
                // A tracked ramp cannot bake ahead — its poses are carried
                // by a part that has not moved yet — so it bakes per tick.
                if rt.tracking.is_none() {
                    rt.append_waypoint(self.t + duration, goal.clone(), vec![0.0; goal.len()]);
                }
                let end = self.t + duration;
                self.programs[self.current].move_ends.push(end);
                rt.moves.push(StepSpan {
                    name: "ramp".to_string(),
                    start: self.t,
                    end,
                });
                rt.active = Some(ActiveMove::Ramp {
                    start: self.t,
                    duration: *duration,
                    from: rt.q_nom.clone(),
                    to: goal,
                });
            }
            Action::Attach {
                robot,
                object,
                link,
                touch_links,
            } => {
                let r = self.action_robot(robot)?;
                self.world
                    .set_joint_positions_for(r, self.robots[r].q.clone())
                    .map_err(|e| err(e.to_string()))?;
                // The pose the object rested at until this instant — a
                // freshly created track must tile [0, duration], so the
                // pre-grasp interval becomes a Hold at this pose.
                let rest_pose = self
                    .world
                    .obstacles()
                    .iter()
                    .find(|o| &o.name == object)
                    .map(|o| o.pose);
                self.world
                    .attach_obstacle_to(r, object, link.as_deref(), touch_links.as_deref())
                    .map_err(|e| err(e.to_string()))?;
                let attachment = self
                    .world
                    .attachment(object)
                    .expect("attach_obstacle just succeeded")
                    .clone();
                let t = self.t;
                let rest = rest_pose.expect("attach_obstacle validated the obstacle");
                let track = self.object_track_at(object, rest, t);
                if let Some(open) = track.spans.last_mut() {
                    let end = open.end_mut();
                    if *end < t {
                        *end = t;
                    }
                }
                track.spans.push(TrackSpan::Follow {
                    t0: t,
                    t1: t,
                    robot: attachment.robot,
                    link: attachment.link,
                    offset: attachment.grasp,
                });
                // Grasping the tracked part ends the chase: it moves with
                // the robot now, so the offset it had at the grasp stands
                // (which is what keeps the lift straight).
                if let Some(latch) = &mut self.robots[r].tracking {
                    if &latch.object == object {
                        latch.frozen = true;
                    }
                }
            }
            Action::Detach { object } => {
                // Sync the carrier so the object freezes where it truly is.
                let carrier = self
                    .world
                    .attachment(object)
                    .map(|a| a.robot)
                    .ok_or_else(|| err(format!("`{object}` is not attached")))?;
                self.world
                    .set_joint_positions_for(carrier, self.robots[carrier].q.clone())
                    .map_err(|e| err(e.to_string()))?;
                self.world
                    .detach_obstacle(object)
                    .map_err(|e| err(e.to_string()))?;
                let pose = self
                    .world
                    .obstacles()
                    .iter()
                    .find(|o| &o.name == object)
                    .map(|o| o.pose)
                    .ok_or_else(|| err(format!("unknown obstacle `{object}`")))?;
                let t = self.t;
                let track = self.object_track_at(object, pose, t);
                if let Some(open) = track.spans.last_mut() {
                    let end = open.end_mut();
                    if *end < t {
                        *end = t;
                    }
                }
                track.spans.push(TrackSpan::Hold { t0: t, t1: t, pose });
            }
            Action::Track {
                robot,
                object,
                link,
            } => {
                let r = self.action_robot(robot)?;
                self.world
                    .set_joint_positions_for(r, self.robots[r].q.clone())
                    .map_err(|e| err(e.to_string()))?;
                self.latch_track(r, object, link.as_deref())?;
            }
            Action::Untrack { robot } => {
                let r = self.action_robot(robot)?;
                self.release_track(r);
            }
            Action::Set { signal, value } => {
                let t = self.t;
                let track = self
                    .signals
                    .iter_mut()
                    .find(|s| &s.name == signal)
                    .ok_or_else(|| err(format!("unknown signal `{signal}`")))?;
                let current = track.edges.last().map(|(_, v)| *v).unwrap_or(false);
                if current != *value {
                    track.edges.push((t, *value));
                }
            }
            Action::Device { device, command } => {
                let t = self.t;
                let mut lane_update = None;
                let found = self.devices.iter_mut().find(|d| match d {
                    DeviceRuntime::Conveyor { name, .. }
                    | DeviceRuntime::Axis { name, .. }
                    | DeviceRuntime::Source { name, .. }
                    | DeviceRuntime::Sink { name, .. }
                    | DeviceRuntime::Vehicle { name, .. } => name == device,
                });
                let Some(found) = found else {
                    return Err(err(format!("unknown device `{device}`")));
                };
                match (found, command) {
                    (DeviceRuntime::Conveyor { running, lane, .. }, DeviceCommand::Start) => {
                        *running = true;
                        lane_update = Some((*lane, true));
                    }
                    (DeviceRuntime::Conveyor { running, lane, .. }, DeviceCommand::Stop) => {
                        *running = false;
                        lane_update = Some((*lane, false));
                    }
                    (
                        DeviceRuntime::Source {
                            running, due, lane, ..
                        },
                        DeviceCommand::Start,
                    ) => {
                        *running = true;
                        // Re-starting re-triggers: the feed is due now, so a
                        // Start always yields a carrier rather than landing
                        // mid-interval.
                        *due = 0.0;
                        lane_update = Some((*lane, true));
                    }
                    (DeviceRuntime::Source { running, lane, .. }, DeviceCommand::Stop) => {
                        *running = false;
                        lane_update = Some((*lane, false));
                    }
                    (DeviceRuntime::Source { interval, .. }, DeviceCommand::SetSpeed(seconds)) => {
                        // A source's "speed" is its feed period.
                        *interval = *seconds;
                    }
                    (DeviceRuntime::Conveyor { velocity, .. }, DeviceCommand::SetSpeed(speed)) => {
                        let norm = velocity.norm();
                        if norm > 1e-12 {
                            *velocity = *velocity / norm * *speed;
                        }
                    }
                    (
                        DeviceRuntime::Conveyor {
                            running,
                            remaining,
                            lane,
                            ..
                        },
                        DeviceCommand::Advance(distance),
                    ) => {
                        // Indexed transfer is a mode, not an overlay: a
                        // free-running belt has no defined "advance by",
                        // and a second advance while one is pending is a
                        // missing device_done in the program.
                        if *running {
                            return Err(err(format!(
                                "conveyor `{device}` is free-running; stop it before a \
                                 fixed advance"
                            )));
                        }
                        if remaining.is_some() {
                            return Err(err(format!(
                                "conveyor `{device}` is still advancing; wait for \
                                 device_done before the next advance"
                            )));
                        }
                        if *distance > 0.0 {
                            *remaining = Some(*distance);
                            lane_update = Some((*lane, true));
                        }
                    }
                    (
                        DeviceRuntime::Axis {
                            position,
                            target,
                            lane,
                            ..
                        },
                        DeviceCommand::MoveTo(goal),
                    ) => {
                        *target = *goal;
                        if (*target - *position).abs() > 1e-9 {
                            lane_update = Some((*lane, true));
                        }
                    }
                    (
                        DeviceRuntime::Vehicle {
                            waypoints,
                            stations,
                            ring,
                            turn_speed,
                            allow_reverse,
                            position,
                            heading,
                            at,
                            target,
                            legs,
                            lane,
                            ..
                        },
                        DeviceCommand::Goto { station },
                    ) => {
                        // A real dispatcher can amend an order in flight;
                        // deterministic v1 keeps travel uninterruptible.
                        if !legs.is_empty() {
                            return Err(err(format!(
                                "vehicle `{device}` is still travelling; wait for \
                                 device_done before the next goto"
                            )));
                        }
                        let to = stations
                            .iter()
                            .find(|(n, _)| n == station)
                            .map(|(_, i)| *i)
                            .ok_or_else(|| {
                                err(format!("vehicle `{device}` has no station `{station}`"))
                            })?;
                        let from = at.unwrap_or(*target);
                        let route = vehicle_route(waypoints, *ring, from, to);
                        *legs = build_legs(
                            waypoints,
                            &route,
                            *position,
                            *heading,
                            *turn_speed,
                            *allow_reverse,
                        );
                        *target = to;
                        if legs.is_empty() {
                            // Already there (or a zero-length route).
                            *at = Some(to);
                        } else {
                            *at = None;
                            lane_update = Some((*lane, true));
                        }
                    }
                    // Kind/command mismatches are rejected by validation.
                    _ => return Err(err(format!("invalid command for device `{device}`"))),
                }
                if let Some((lane, value)) = lane_update {
                    self.set_lane(lane, t, value);
                }
            }
        }
        Ok(())
    }

    /// The track for `object`, created lazily with a rest hold covering
    /// `[0, since]` so spans always tile from t = 0.
    fn object_track_at(
        &mut self,
        object: &str,
        rest_pose: Isometry3<f64>,
        since: f64,
    ) -> &mut ObjectTrack {
        let index = match self.objects.iter().position(|o| o.name == object) {
            Some(i) => i,
            None => {
                let mut spans = Vec::new();
                if since > 0.0 {
                    spans.push(TrackSpan::Hold {
                        t0: 0.0,
                        t1: since,
                        pose: rest_pose,
                    });
                }
                self.objects.push(ObjectTrack {
                    name: object.to_string(),
                    spans,
                });
                self.objects.len() - 1
            }
        };
        &mut self.objects[index]
    }

    fn finish(mut self) -> SequenceTimeline {
        let duration = self.t;
        let names: Vec<String> = self.world.robots().iter().map(|r| r.name.clone()).collect();
        let robots = self
            .robots
            .into_iter()
            .zip(names)
            .map(|(mut rt, name)| {
                let (q, zeros) = (rt.q.clone(), vec![0.0; rt.q.len()]);
                rt.append_waypoint(duration, q, zeros);
                RobotTrack {
                    name,
                    trajectory: JointTrajectory {
                        times: rt.times,
                        positions: rt.positions,
                        velocities: rt.velocities,
                    },
                    moves: rt.moves,
                    // The cycle usually ends parked: close a travelling span
                    // at its own end and rest there, rather than extending it
                    // to the horn and driving off the timeline.
                    base: rt.base.map(|mut spans| {
                        match spans.last() {
                            Some(span @ (TrackSpan::Linear { .. } | TrackSpan::Pivot { .. })) => {
                                let (_, end) = span.range();
                                if end < duration - 1e-12 {
                                    let pose = SequenceTimeline::span_pose(&spans, &[], end)
                                        .expect("non-empty base track");
                                    spans.push(TrackSpan::Hold {
                                        t0: end,
                                        t1: duration,
                                        pose,
                                    });
                                }
                            }
                            Some(_) => {
                                if let Some(open) = spans.last_mut() {
                                    *open.end_mut() = duration;
                                }
                            }
                            None => {}
                        }
                        spans
                    }),
                }
            })
            .collect();
        for track in &mut self.objects {
            if let Some(open) = track.spans.last_mut() {
                *open.end_mut() = duration;
            }
        }
        SequenceTimeline {
            duration,
            robots,
            objects: self.objects,
            signals: self.signals,
            step_spans: self.step_spans,
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::motion::{Segment, SegmentKind};
    use crate::seq::Step;
    use botrail_model::Geometry;
    use nalgebra::{Translation3, UnitQuaternion, Vector3};
    use std::sync::Arc;

    /// 1-DOF arm: revolute Z at z = 0.5 (limits ±1), two 0.1 cubes.
    pub(crate) fn sample_scene() -> Scene {
        let urdf = r#"
        <robot name="r">
          <link name="a">
            <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
          </link>
          <link name="b">
            <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
          </link>
          <joint name="j" type="revolute">
            <parent link="a"/><child link="b"/>
            <origin xyz="0 0 0.5"/>
            <axis xyz="0 0 1"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/>
          </joint>
        </robot>"#;
        Scene::new(Arc::new(
            botrail_model::RobotModel::from_urdf_str(urdf).unwrap(),
        ))
    }

    pub(crate) fn joint_motion(scene: &mut Scene, name: &str, goal: f64) {
        scene
            .add_segment(
                name,
                Segment {
                    kind: SegmentKind::Joint,
                    goal_positions: vec![goal],
                    constraints: vec![],
                },
            )
            .unwrap();
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
        }
    }

    #[test]
    fn motions_waits_and_step_spans_line_up() {
        let mut scene = sample_scene();
        joint_motion(&mut scene, "go", 0.8);
        joint_motion(&mut scene, "back", 0.0);
        scene.upsert_sequence(Sequence {
            name: "cycle".into(),
            steps: vec![
                step(
                    "run",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::Done,
                ),
                step("wait", vec![], Condition::Elapsed { seconds: 0.5 }),
                step(
                    "return",
                    vec![Action::StartMotion {
                        motion: "back".into(),
                    }],
                    Condition::Done,
                ),
            ],
        });
        let options = RolloutOptions::default();
        let tl = scene.simulate_sequence("cycle", &options).unwrap();

        assert_eq!(tl.step_spans.len(), 3);
        let run = &tl.step_spans[0];
        let wait = &tl.step_spans[1];
        let ret = &tl.step_spans[2];
        // Step boundaries quantize up to the scan period.
        assert!(run.start == 0.0 && run.end > 0.2);
        assert!((tl.robots[0].trajectory.sample(run.end)[0] - 0.8).abs() < 1e-9);
        let wait_len = wait.end - wait.start;
        assert!(
            (0.5 - 1e-9..=0.5 + options.dt + 1e-9).contains(&wait_len),
            "wait_len = {wait_len}"
        );
        // The robot holds still through the wait.
        assert!((tl.robots[0].trajectory.sample(wait.start + 0.25)[0] - 0.8).abs() < 1e-9);
        // The return motion starts where the previous ended and comes home.
        assert!((tl.robots[0].trajectory.sample(tl.duration)[0]).abs() < 1e-9);
        assert!((ret.end - tl.duration).abs() < 1e-12);
        // Cycle time covers both motions plus the wait.
        assert!(tl.duration > run.end + 0.5);
    }

    #[test]
    fn ramp_signals_and_instant_chains() {
        let mut scene = sample_scene();
        scene.define_signal("flag", false);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step(
                    "ramp",
                    vec![Action::StartRamp {
                        robot: None,
                        targets: vec![("j".into(), 0.6)],
                        duration: 0.3,
                    }],
                    Condition::Done,
                ),
                step(
                    "mark",
                    vec![Action::Set {
                        signal: "flag".into(),
                        value: true,
                    }],
                    Condition::Immediately,
                ),
                step(
                    "gate",
                    vec![],
                    Condition::Signal {
                        name: "flag".into(),
                        value: true,
                    },
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();

        // Cubic rest-to-rest ramp: exact midpoint halfway through.
        assert!((tl.robots[0].trajectory.sample(0.15)[0] - 0.3).abs() < 1e-9);
        assert!((tl.robots[0].trajectory.sample(0.3)[0] - 0.6).abs() < 1e-9);
        // mark + gate resolve in the same scan tick as the ramp's Done.
        let flag = &tl.signals[0];
        assert_eq!(flag.edges.first(), Some(&(0.0, false)));
        assert_eq!(flag.edges.len(), 2);
        let (edge_t, v) = flag.edges[1];
        assert!(v && (edge_t - tl.duration).abs() < 1e-12);
        assert!(!flag.value_at(0.1) && flag.value_at(tl.duration));
        // Instantaneous steps leave zero-width spans at the end.
        assert_eq!(tl.step_spans[1].start, tl.step_spans[1].end);
        assert!((tl.duration - 0.3).abs() < 0.011);
    }

    #[test]
    fn attach_detach_object_tracks() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "box",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.04, 0.04),
                },
                nalgebra::Isometry3::from_parts(
                    Translation3::new(0.1, 0.0, 0.5),
                    UnitQuaternion::identity(),
                ),
            )
            .unwrap();
        joint_motion(&mut scene, "go", 0.8);
        scene.upsert_sequence(Sequence {
            name: "pick".into(),
            steps: vec![
                step(
                    "grasp",
                    vec![Action::Attach {
                        robot: None,
                        object: "box".into(),
                        link: None,
                        touch_links: None,
                    }],
                    Condition::Immediately,
                ),
                step(
                    "move",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::Done,
                ),
                step(
                    "drop",
                    vec![Action::Detach {
                        object: "box".into(),
                    }],
                    Condition::Immediately,
                ),
                step("settle", vec![], Condition::Elapsed { seconds: 0.2 }),
            ],
        });
        let tl = scene
            .simulate_sequence("pick", &RolloutOptions::default())
            .unwrap();

        assert_eq!(tl.objects.len(), 1);
        let track = &tl.objects[0];
        assert_eq!(track.name, "box");
        assert_eq!(track.spans.len(), 2);
        let TrackSpan::Follow {
            t0,
            t1,
            robot,
            link,
            offset,
        } = &track.spans[0]
        else {
            panic!("expected follow span, got {track:?}");
        };
        assert!(*t0 == 0.0 && *t1 > 0.2 && *robot == 0 && *link == 1);
        // Mid-motion the box rides FK ∘ grasp.
        let t_mid = (t0 + t1) / 2.0;
        let q = tl.robots[0].trajectory.sample(t_mid);
        let poses = scene.fk(&q).unwrap();
        let expected = poses[*link] * offset;
        let via_track =
            SequenceTimeline::object_pose(track, std::slice::from_ref(&poses), t_mid).unwrap();
        assert!((via_track.translation.vector - expected.translation.vector).norm() < 1e-12);
        // After detach it holds at the rotated position for the rest.
        let TrackSpan::Hold {
            t1: hold_end, pose, ..
        } = &track.spans[1]
        else {
            panic!("expected hold span");
        };
        assert!((hold_end - tl.duration).abs() < 1e-12);
        let expected = Vector3::new(0.1 * 0.8f64.cos(), 0.1 * 0.8f64.sin(), 0.5);
        assert!((pose.translation.vector - expected).norm() < 1e-9);
        // The live scene is untouched by the rollout.
        assert!(scene.attachments().is_empty());
        assert!((scene.obstacles()[0].pose.translation.x - 0.1).abs() < 1e-12);
    }

    #[test]
    fn mid_sequence_attach_prepends_a_rest_hold() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "box",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.04, 0.04),
                },
                nalgebra::Isometry3::from_parts(
                    Translation3::new(0.1, 0.0, 0.5),
                    UnitQuaternion::identity(),
                ),
            )
            .unwrap();
        joint_motion(&mut scene, "go", 0.8);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step("wait", vec![], Condition::Elapsed { seconds: 0.4 }),
                step(
                    "grasp",
                    vec![Action::Attach {
                        robot: None,
                        object: "box".into(),
                        link: None,
                        touch_links: None,
                    }],
                    Condition::Immediately,
                ),
                step(
                    "move",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::Done,
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        // Before the grasp the object rests at its scene pose (the track
        // tiles [0, duration] with a leading Hold).
        let track = &tl.objects[0];
        let TrackSpan::Hold { t0, t1, pose } = &track.spans[0] else {
            panic!("expected leading hold, got {track:?}");
        };
        assert!(*t0 == 0.0 && (*t1 - 0.4).abs() < 0.011);
        assert!((pose.translation.vector - Vector3::new(0.1, 0.0, 0.5)).norm() < 1e-12);
        let poses = scene.fk(&tl.robots[0].trajectory.sample(0.2)).unwrap();
        let early =
            SequenceTimeline::object_pose(track, std::slice::from_ref(&poses), 0.2).unwrap();
        assert!((early.translation.vector - Vector3::new(0.1, 0.0, 0.5)).norm() < 1e-12);
        assert!(matches!(track.spans[1], TrackSpan::Follow { .. }));
    }

    #[test]
    fn validation_catches_authoring_mistakes() {
        let mut scene = sample_scene();
        joint_motion(&mut scene, "go", 0.8);
        let check = |scene: &Scene, steps: Vec<Step>, needle: &str| {
            let mut s = scene.clone();
            s.upsert_sequence(Sequence {
                name: "bad".into(),
                steps,
            });
            let err = s
                .simulate_sequence("bad", &RolloutOptions::default())
                .unwrap_err();
            let msg = err.to_string();
            assert!(
                matches!(err, SeqError::Validation { .. }) && msg.contains(needle),
                "expected `{needle}` in `{msg}`"
            );
        };
        check(&scene, vec![], "no steps");
        check(
            &scene,
            vec![step(
                "x",
                vec![Action::StartMotion {
                    motion: "nope".into(),
                }],
                Condition::Done,
            )],
            "unknown motion",
        );
        check(
            &scene,
            vec![step("x", vec![], Condition::Done)],
            "starts none",
        );
        check(
            &scene,
            vec![step(
                "x",
                vec![],
                Condition::Signal {
                    name: "ghost".into(),
                    value: true,
                },
            )],
            "unknown signal",
        );
        check(
            &scene,
            vec![step(
                "x",
                vec![Action::StartRamp {
                    robot: None,
                    targets: vec![("j".into(), 5.0)],
                    duration: 0.2,
                }],
                Condition::Done,
            )],
            "outside",
        );
        check(
            &scene,
            vec![step(
                "x",
                vec![
                    Action::StartMotion {
                        motion: "go".into(),
                    },
                    Action::StartRamp {
                        robot: None,
                        targets: vec![("j".into(), 0.1)],
                        duration: 0.2,
                    },
                ],
                Condition::Done,
            )],
            "at most one",
        );
    }

    #[test]
    fn timeout_and_immediate_loop_guards() {
        let mut scene = sample_scene();
        scene.define_signal("never", false);
        scene.upsert_sequence(Sequence {
            name: "stuck".into(),
            steps: vec![step(
                "wait forever",
                vec![],
                Condition::Signal {
                    name: "never".into(),
                    value: true,
                },
            )],
        });
        let options = RolloutOptions {
            max_duration: 0.5,
            ..RolloutOptions::default()
        };
        assert!(matches!(
            scene.simulate_sequence("stuck", &options),
            Err(SeqError::Timeout { .. })
        ));

        let many: Vec<Step> = (0..10)
            .map(|i| step(&format!("s{i}"), vec![], Condition::Immediately))
            .collect();
        scene.upsert_sequence(Sequence {
            name: "chain".into(),
            steps: many,
        });
        let tight = RolloutOptions {
            immediate_chain_limit: 5,
            ..RolloutOptions::default()
        };
        assert!(matches!(
            scene.simulate_sequence("chain", &tight),
            Err(SeqError::ImmediateLoop { .. })
        ));
        // Under the limit the chain is fine (zero-duration sequence).
        let ok = scene
            .simulate_sequence("chain", &RolloutOptions::default())
            .unwrap();
        assert_eq!(ok.duration, 0.0);
        assert_eq!(ok.step_spans.len(), 10);
    }

    #[test]
    fn rollouts_are_deterministic() {
        let mut scene = sample_scene();
        scene.define_signal("flag", true);
        joint_motion(&mut scene, "go", 0.7);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step(
                    "run",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::All(vec![
                        Condition::Done,
                        Condition::Signal {
                            name: "flag".into(),
                            value: true,
                        },
                    ]),
                ),
                step("wait", vec![], Condition::Elapsed { seconds: 0.25 }),
            ],
        });
        let a = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        let b = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        assert_eq!(a.duration, b.duration);
        assert_eq!(a.robots[0].trajectory.times, b.robots[0].trajectory.times);
        assert_eq!(
            a.robots[0].trajectory.positions,
            b.robots[0].trajectory.positions
        );
        assert_eq!(a.step_spans.len(), b.step_spans.len());
    }

    #[test]
    fn signal_defs_shape_the_initial_state() {
        let mut scene = sample_scene();
        scene.define_signal("armed", true);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![step(
                "gate",
                vec![],
                Condition::Signal {
                    name: "armed".into(),
                    value: true,
                },
            )],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        // Initially-true signal lets the gate pass at t = 0.
        assert_eq!(tl.duration, 0.0);
        assert_eq!(tl.signals[0].edges, vec![(0.0, true)]);
    }
}

#[cfg(test)]
mod device_tests {
    use super::tests::*;
    use super::*;
    use crate::seq::{Device, DeviceCommand, DeviceKind, Sensor, SensorKind, SensorWatch, Step};
    use botrail_model::Geometry;
    use nalgebra::{Point3, Translation3, Unit, UnitQuaternion, Vector3};

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity())
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
        }
    }

    /// Conveyor feed with a photoelectric beam: the box trips the beam at
    /// the analytic time and the conveyor stops on the next scan.
    /// A magazine + a belt + a sink is an endless line built out of a
    /// finite pool: every carrier goes round more than once, and the object
    /// tracks stay continuous across the jumps.
    #[test]
    fn a_source_and_sink_recirculate_a_finite_pool() {
        let mut scene = sample_scene();
        let pool: Vec<String> = (0..3).map(|i| format!("c{i}")).collect();
        for (i, name) in pool.iter().enumerate() {
            scene
                .add_obstacle(
                    name,
                    Geometry::Box {
                        size: Vector3::new(0.04, 0.04, 0.04),
                    },
                    iso(-1.5, 0.5 - 0.1 * i as f64, 0.5),
                )
                .unwrap();
        }
        scene.upsert_device(Device {
            name: "feed".into(),
            kind: DeviceKind::Source {
                pool: pool.clone(),
                park: iso(-1.5, 0.5, 0.5),
                pitch: Vector3::new(0.0, -0.1, 0.0),
                pose: iso(-0.8, 0.5, 0.5),
                interval: 1.0,
                running: true,
            },
        });
        scene.upsert_device(Device {
            name: "belt".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: iso(0.0, 0.5, 0.5),
                zone_size: Vector3::new(2.0, 0.3, 0.3),
                velocity: Vector3::new(0.4, 0.0, 0.0),
                running: true,
            },
        });
        scene.upsert_device(Device {
            name: "out".into(),
            kind: DeviceKind::Sink {
                zone_pose: iso(0.9, 0.5, 0.5),
                zone_size: Vector3::new(0.2, 0.3, 0.3),
                source: "feed".into(),
            },
        });
        scene.upsert_sequence(Sequence {
            name: "run".into(),
            steps: vec![Step {
                name: "run".into(),
                actions: vec![],
                transition: Condition::Elapsed { seconds: 12.0 },
            }],
        });

        let tl = scene
            .simulate_sequence("run", &RolloutOptions::default())
            .unwrap();
        let track = |name: &str| {
            tl.objects
                .iter()
                .find(|o| o.name == name)
                .unwrap_or_else(|| panic!("no track for {name}"))
        };

        for name in &pool {
            let t = track(name);
            // Spans tile the whole run with no gap — a teleport must close
            // the span it interrupts, not leave a hole in the timeline.
            let mut cursor = 0.0;
            for span in &t.spans {
                let (t0, t1) = span.range();
                assert!(
                    (t0 - cursor).abs() < 1e-9,
                    "{name}: gap before {t0} (cursor {cursor})"
                );
                cursor = t1;
            }
            assert!(cursor >= 12.0 - 1e-6, "{name}: track ends at {cursor}");

            // Went round more than once: each lap is one jump upstream.
            let laps = (0..1200)
                .map(|i| {
                    SequenceTimeline::object_pose(t, &[], i as f64 * 0.01)
                        .unwrap()
                        .translation
                        .x
                })
                .collect::<Vec<_>>()
                .windows(2)
                .filter(|w| w[1] < w[0] - 0.5)
                .count();
            assert!(laps >= 2, "{name} only went round {laps} time(s)");
        }
    }

    /// A carrier the robot is holding must not be swept up by the sink it
    /// happens to be carried over.
    #[test]
    fn a_sink_leaves_grasped_carriers_alone() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "c0",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.04, 0.04),
                },
                iso(0.9, 0.5, 0.5),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "feed".into(),
            kind: DeviceKind::Source {
                pool: vec!["c0".into()],
                park: iso(-1.5, 0.5, 0.5),
                pitch: Vector3::zeros(),
                pose: iso(-0.8, 0.5, 0.5),
                interval: 1.0,
                running: false,
            },
        });
        scene.upsert_device(Device {
            name: "out".into(),
            kind: DeviceKind::Sink {
                zone_pose: iso(0.9, 0.5, 0.5),
                zone_size: Vector3::new(0.4, 0.4, 0.4),
                source: "feed".into(),
            },
        });
        scene.attach_obstacle("c0", None, None).unwrap();
        scene.upsert_sequence(Sequence {
            name: "hold".into(),
            steps: vec![Step {
                name: "hold".into(),
                actions: vec![],
                transition: Condition::Elapsed { seconds: 1.0 },
            }],
        });
        let tl = scene
            .simulate_sequence("hold", &RolloutOptions::default())
            .unwrap();
        let track = tl.objects.iter().find(|o| o.name == "c0").unwrap();
        assert!(
            track
                .spans
                .iter()
                .all(|s| matches!(s, TrackSpan::Follow { .. })),
            "the sink took a carrier out of the gripper: {:?}",
            track.spans
        );
    }

    #[test]
    fn conveyor_feed_trips_the_beam_on_time() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "box",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.04, 0.04),
                },
                iso(-0.5, 0.5, 0.5),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "conv".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: iso(-0.2, 0.5, 0.5),
                zone_size: Vector3::new(1.2, 0.3, 0.3),
                velocity: Vector3::new(0.25, 0.0, 0.0),
                running: false,
            },
        });
        scene.upsert_sensor(Sensor {
            name: "beam".into(),
            kind: SensorKind::Beam {
                from: Point3::new(0.0, 0.3, 0.5),
                to: Point3::new(0.0, 0.7, 0.5),
                radius: 0.005,
            },
            watch: SensorWatch::AllObjects,
            mount: None,
        });
        scene.upsert_sequence(Sequence {
            name: "feed".into(),
            steps: vec![
                step(
                    "feed",
                    vec![Action::Device {
                        device: "conv".into(),
                        command: DeviceCommand::Start,
                    }],
                    Condition::Signal {
                        name: "beam".into(),
                        value: true,
                    },
                ),
                step(
                    "stop",
                    vec![Action::Device {
                        device: "conv".into(),
                        command: DeviceCommand::Stop,
                    }],
                    Condition::Elapsed { seconds: 0.1 },
                ),
            ],
        });
        let options = RolloutOptions::default();
        let tl = scene.simulate_sequence("feed", &options).unwrap();

        // Contact when the box face (half 0.02) meets the beam surface
        // (radius 0.005): origin at -0.025, i.e. 0.475 m / 0.25 m/s = 1.9 s.
        let feed_end = tl.step_spans[0].end;
        assert!(
            (feed_end - 1.9).abs() <= options.dt + 1e-9,
            "feed_end = {feed_end}"
        );
        // Lanes: the beam input edge and the conveyor output edges line up.
        let beam = tl.signals.iter().find(|s| s.name == "beam").unwrap();
        assert_eq!(beam.edges.len(), 2);
        assert!((beam.edges[1].0 - feed_end).abs() < 1e-9 && beam.edges[1].1);
        let conv = tl.signals.iter().find(|s| s.name == "conv").unwrap();
        assert!(conv.value_at(1.0));
        assert!(!conv.value_at(tl.duration));
        // The box rode a linear span then settled; total travel matches.
        let track = &tl.objects[0];
        assert!(matches!(track.spans[0], TrackSpan::Linear { .. }));
        assert!(matches!(track.spans.last(), Some(TrackSpan::Hold { .. })));
        let poses = scene
            .fk(&tl.robots[0].trajectory.sample(tl.duration))
            .unwrap();
        let end_pose =
            SequenceTimeline::object_pose(track, std::slice::from_ref(&poses), tl.duration)
                .unwrap();
        let travelled = end_pose.translation.x - (-0.5);
        assert!(
            (travelled - 0.25 * feed_end).abs() < 1e-9,
            "travelled = {travelled}"
        );
        // The live scene's box stays where it was authored.
        assert!((scene.obstacles()[0].pose.translation.x - (-0.5)).abs() < 1e-12);
    }

    /// A linear axis positions its payload exactly and reports in-position.
    #[test]
    fn linear_axis_moves_exactly_and_reports_done() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "door",
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                iso(0.6, 0.0, 0.2),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "lift".into(),
            kind: DeviceKind::LinearAxis {
                objects: vec!["door".into()],
                axis: Unit::new_normalize(Vector3::z()),
                speed: 0.5,
                position: 0.0,
                range: (0.0, 0.4),
            },
        });
        scene.upsert_sequence(Sequence {
            name: "open".into(),
            steps: vec![
                step(
                    "raise",
                    vec![Action::Device {
                        device: "lift".into(),
                        command: DeviceCommand::MoveTo(0.3),
                    }],
                    Condition::DeviceDone {
                        device: "lift".into(),
                    },
                ),
                step("hold", vec![], Condition::Elapsed { seconds: 0.1 }),
            ],
        });
        let options = RolloutOptions::default();
        let tl = scene.simulate_sequence("open", &options).unwrap();

        // 0.3 m at 0.5 m/s = 0.6 s (+ scan quantization).
        let raise_end = tl.step_spans[0].end;
        assert!((raise_end - 0.6).abs() <= options.dt + 1e-9, "{raise_end}");
        // The door lands exactly 0.3 above its rest height.
        let track = &tl.objects[0];
        let poses = scene
            .fk(&tl.robots[0].trajectory.sample(tl.duration))
            .unwrap();
        let end = SequenceTimeline::object_pose(track, std::slice::from_ref(&poses), tl.duration)
            .unwrap();
        assert!(
            (end.translation.z - 0.5).abs() < 1e-12,
            "{}",
            end.translation.z
        );
        // Mid-travel sampling is linear (exact at 0.3 s: half way).
        let mid = SequenceTimeline::object_pose(track, std::slice::from_ref(&poses), 0.3).unwrap();
        assert!(
            (mid.translation.z - 0.35).abs() < 1e-9,
            "{}",
            mid.translation.z
        );
        // The moving lane pulses during travel only.
        let lane = tl.signals.iter().find(|s| s.name == "lift").unwrap();
        assert!(lane.value_at(0.3) && !lane.value_at(tl.duration));
    }

    /// A zone sensor watching the robot (light curtain) fires when a ramp
    /// swings the arm into it.
    #[test]
    fn robot_watch_zone_fires_on_intrusion() {
        let mut scene = sample_scene();
        // Link b (cube at z = 0.5) swings to +y at q = pi/2; park the zone
        // there.
        scene.upsert_sensor(Sensor {
            name: "curtain".into(),
            kind: SensorKind::Zone {
                pose: iso(0.0, 0.0, 0.5),
                size: Vector3::new(0.4, 0.4, 0.4),
            },
            watch: SensorWatch::Robot,
            mount: None,
        });
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![step(
                "wait for the arm",
                vec![],
                Condition::Signal {
                    name: "curtain".into(),
                    value: true,
                },
            )],
        });
        // The arm is already inside the curtain at rest: fires at t = 0.
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        assert_eq!(tl.duration, 0.0);
        let lane = tl.signals.iter().find(|s| s.name == "curtain").unwrap();
        assert!(lane.value_at(0.0));
    }

    #[test]
    fn device_and_sensor_validation() {
        let mut scene = sample_scene();
        scene.upsert_device(Device {
            name: "conv".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: iso(0.0, 0.0, 0.0),
                zone_size: Vector3::new(1.0, 1.0, 1.0),
                velocity: Vector3::new(0.1, 0.0, 0.0),
                running: true,
            },
        });
        // A source really has no in-position (a conveyor now does: a
        // consumed `advance`).
        scene.upsert_device(Device {
            name: "feed".into(),
            kind: DeviceKind::Source {
                pool: vec![],
                park: iso(0.0, 0.0, 0.0),
                pitch: Vector3::zeros(),
                pose: iso(0.0, 0.0, 0.0),
                interval: 1.0,
                running: false,
            },
        });
        scene.upsert_sensor(Sensor {
            name: "eye".into(),
            kind: SensorKind::Zone {
                pose: iso(0.0, 0.0, 0.0),
                size: Vector3::new(0.1, 0.1, 0.1),
            },
            watch: SensorWatch::AllObjects,
            mount: None,
        });
        let check = |scene: &Scene, steps: Vec<Step>, needle: &str| {
            let mut s = scene.clone();
            s.upsert_sequence(Sequence {
                name: "bad".into(),
                steps,
            });
            let err = s
                .simulate_sequence("bad", &RolloutOptions::default())
                .unwrap_err()
                .to_string();
            assert!(err.contains(needle), "expected `{needle}` in `{err}`");
        };
        // Sensors are read-only inputs.
        check(
            &scene,
            vec![step(
                "x",
                vec![Action::Set {
                    signal: "eye".into(),
                    value: true,
                }],
                Condition::Immediately,
            )],
            "read-only",
        );
        // device_done needs a positioning device (axis or vehicle).
        check(
            &scene,
            vec![step(
                "x",
                vec![],
                Condition::DeviceDone {
                    device: "feed".into(),
                },
            )],
            "no in-position",
        );
        // Axis commands must stay in range; conveyors have no position.
        check(
            &scene,
            vec![step(
                "x",
                vec![Action::Device {
                    device: "conv".into(),
                    command: DeviceCommand::MoveTo(0.5),
                }],
                Condition::Immediately,
            )],
            "start/stop",
        );
        // Sensor names may not shadow internal signals.
        let mut shadowed = scene.clone();
        shadowed.define_signal("eye", false);
        check(
            &shadowed,
            vec![step("x", vec![], Condition::Immediately)],
            "collides",
        );
    }
}

#[cfg(test)]
mod multi_actor_tests {
    use super::*;
    use crate::motion::{Segment, SegmentKind};
    use crate::seq::{Sensor, SensorKind, SensorWatch, Step};
    use botrail_model::Geometry;
    use nalgebra::{Translation3, UnitQuaternion, Vector3};
    use std::sync::Arc;

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity())
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
        }
    }

    /// §1 in miniature: two 1-DOF sliders face each other across a shared
    /// middle band. Fully extended rods overlap; one at a time clears.
    const SLIDER: &str = r#"
        <robot name="slider">
          <link name="base"/>
          <link name="rod">
            <visual>
              <origin xyz="0 0.25 0"/>
              <geometry><box size="0.08 0.5 0.08"/></geometry>
            </visual>
          </link>
          <joint name="s" type="prismatic">
            <parent link="base"/><child link="rod"/>
            <origin xyz="0 0 0.3"/>
            <axis xyz="0 1 0"/>
            <limit lower="0" upper="0.6" effort="1" velocity="1"/>
          </joint>
        </robot>"#;

    /// Robots `a` (pushing +y from y = -0.75) and `b` (mirrored at +0.75),
    /// with the interlock zone over the shared band y ∈ [-0.2, 0.2] watching
    /// only `a`. Motions: `<r>_in` extends into the band, `<r>_out` retreats.
    fn dual_cell() -> Scene {
        let model = Arc::new(botrail_model::RobotModel::from_urdf_str(SLIDER).unwrap());
        let mut scene = crate::Scene::with_base(model.clone(), iso(0.0, -0.75, 0.0));
        scene.rename_robot(0, "a");
        let flipped = Isometry3::from_parts(
            Translation3::new(0.0, 0.75, 0.0),
            UnitQuaternion::from_axis_angle(&nalgebra::Vector3::z_axis(), std::f64::consts::PI),
        );
        scene.add_robot(model, Some("b"), flipped);
        for (robot, name, goal) in [
            (0, "a_in", 0.45),
            (0, "a_out", 0.0),
            (1, "b_in", 0.45),
            (1, "b_out", 0.0),
        ] {
            scene
                .add_segment_for(
                    robot,
                    name,
                    Segment {
                        kind: SegmentKind::Joint,
                        goal_positions: vec![goal],
                        constraints: vec![],
                    },
                )
                .unwrap();
        }
        scene.upsert_sensor(Sensor {
            name: "zone".into(),
            kind: SensorKind::Zone {
                pose: iso(0.0, 0.0, 0.3),
                size: Vector3::new(0.4, 0.4, 0.4),
            },
            watch: SensorWatch::Robots(vec!["a".into()]),
            mount: None,
        });
        scene
    }

    /// The §1 scenario with the interlock in place: A works the shared band
    /// first, B waits for the zone to clear, both retreats run concurrently,
    /// and the whole cycle bakes deterministically.
    #[test]
    fn zone_interlock_serializes_the_shared_band() {
        let mut scene = dual_cell();
        scene.upsert_sequence(Sequence {
            name: "cell".into(),
            steps: vec![
                step(
                    "A enter",
                    vec![Action::StartMotion {
                        motion: "a_in".into(),
                    }],
                    Condition::Done,
                ),
                // Async: A retreats while B already waits on the interlock.
                step(
                    "A retreat",
                    vec![Action::StartMotion {
                        motion: "a_out".into(),
                    }],
                    Condition::Immediately,
                ),
                step(
                    "B interlock",
                    vec![],
                    Condition::Signal {
                        name: "zone".into(),
                        value: false,
                    },
                ),
                step(
                    "B enter",
                    vec![Action::StartMotion {
                        motion: "b_in".into(),
                    }],
                    Condition::Done,
                ),
                step(
                    "B retreat",
                    vec![Action::StartMotion {
                        motion: "b_out".into(),
                    }],
                    Condition::Immediately,
                ),
                step(
                    "cycle end",
                    vec![],
                    Condition::All(vec![
                        Condition::RobotDone { robot: "a".into() },
                        Condition::RobotDone { robot: "b".into() },
                    ]),
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("cell", &RolloutOptions::default())
            .unwrap();

        assert_eq!(tl.robots.len(), 2);
        assert_eq!(tl.robots[0].name, "a");
        assert_eq!(tl.robots[1].name, "b");
        // Both arms actually ran their strokes.
        let a_moves = &tl.robots[0].moves;
        let b_moves = &tl.robots[1].moves;
        assert_eq!(
            a_moves.iter().map(|m| m.name.as_str()).collect::<Vec<_>>(),
            ["a_in", "a_out"]
        );
        assert_eq!(
            b_moves.iter().map(|m| m.name.as_str()).collect::<Vec<_>>(),
            ["b_in", "b_out"]
        );
        // Concurrency: B enters while A is still retreating (the interlock
        // released on zone exit, not on A's completion)...
        assert!(
            b_moves[0].start < a_moves[1].end - 1e-9,
            "B started at {} but A finished retreating at {}",
            b_moves[0].start,
            a_moves[1].end
        );
        // ...but never before A cleared the shared band (the zone lane's
        // falling edge).
        let zone = tl.signals.iter().find(|s| s.name == "zone").unwrap();
        let cleared = zone
            .edges
            .iter()
            .rev()
            .find(|(_, v)| !*v)
            .map(|(t, _)| *t)
            .unwrap();
        assert!(b_moves[0].start >= cleared - 1e-9);
        // The cycle ends when both robots are idle (RobotDone × 2).
        let last_end = a_moves[1].end.max(b_moves[1].end);
        assert!((tl.duration - last_end).abs() <= 0.011, "{}", tl.duration);

        // Deterministic: an identical run bakes bit-identical tracks.
        let again = scene
            .simulate_sequence("cell", &RolloutOptions::default())
            .unwrap();
        assert_eq!(tl.duration, again.duration);
        for (x, y) in tl.robots.iter().zip(&again.robots) {
            assert_eq!(x.trajectory.times, y.trajectory.times);
            assert_eq!(x.trajectory.positions, y.trajectory.positions);
        }
    }

    /// Without the interlock both rods sweep into the band mid-flight —
    /// each plan was valid against the other's *frozen* pose — and the tick
    /// verification reports the collision with its time.
    #[test]
    fn dropping_the_interlock_reports_a_robot_collision() {
        let mut scene = dual_cell();
        scene.upsert_sequence(Sequence {
            name: "clash".into(),
            steps: vec![step(
                "both enter",
                vec![
                    Action::StartMotion {
                        motion: "a_in".into(),
                    },
                    Action::StartMotion {
                        motion: "b_in".into(),
                    },
                ],
                Condition::Done,
            )],
        });
        let err = scene
            .simulate_sequence("clash", &RolloutOptions::default())
            .unwrap_err();
        let SeqError::RobotCollision {
            t, a, b, link_a, ..
        } = &err
        else {
            panic!("expected RobotCollision, got {err}");
        };
        assert_eq!((a.as_str(), b.as_str()), ("a", "b"));
        assert!(*t > 0.1 && *t < 1.0, "t = {t}");
        assert!(link_a.contains("rod"));
        let msg = err.to_string();
        assert!(msg.contains("interlock"), "{msg}");
    }

    /// A handover writes as detach → attach: the object's track follows
    /// robot `a`, holds, then follows robot `b`.
    #[test]
    fn handover_switches_the_carrier() {
        let mut scene = dual_cell();
        scene
            .add_obstacle(
                "box",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.04, 0.04),
                },
                iso(0.0, 0.0, 0.3),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "pass".into(),
            steps: vec![
                step(
                    "A grasp",
                    vec![Action::Attach {
                        robot: Some("a".into()),
                        object: "box".into(),
                        link: None,
                        touch_links: None,
                    }],
                    Condition::Elapsed { seconds: 0.2 },
                ),
                step(
                    "A place",
                    vec![Action::Detach {
                        object: "box".into(),
                    }],
                    Condition::Elapsed { seconds: 0.2 },
                ),
                step(
                    "B grasp",
                    vec![Action::Attach {
                        robot: Some("b".into()),
                        object: "box".into(),
                        link: None,
                        touch_links: None,
                    }],
                    Condition::Elapsed { seconds: 0.2 },
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("pass", &RolloutOptions::default())
            .unwrap();
        let track = &tl.objects[0];
        let carriers: Vec<Option<usize>> = track
            .spans
            .iter()
            .map(|s| match s {
                TrackSpan::Follow { robot, .. } => Some(*robot),
                _ => None,
            })
            .collect();
        assert_eq!(carriers, [Some(0), None, Some(1)], "{track:?}");
        // The live scene is untouched.
        assert!(scene.attachments().is_empty());
    }

    /// Multi-robot authoring rules: ambiguous `robot=None`, double attach,
    /// two drivers on one arm, and unknown robots in `robot_done`.
    #[test]
    fn multi_actor_validation_rules() {
        let scene = dual_cell();
        let check = |steps: Vec<Step>, needle: &str| {
            let mut s = scene.clone();
            s.add_obstacle(
                "box",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.04, 0.04),
                },
                iso(0.0, 0.0, 0.3),
            )
            .unwrap();
            s.upsert_sequence(Sequence {
                name: "bad".into(),
                steps,
            });
            let err = s
                .simulate_sequence("bad", &RolloutOptions::default())
                .unwrap_err();
            let msg = err.to_string();
            assert!(
                matches!(err, SeqError::Validation { .. }) && msg.contains(needle),
                "expected `{needle}` in `{msg}`"
            );
        };
        // With two robots, an unaddressed ramp is ambiguous.
        check(
            vec![step(
                "x",
                vec![Action::StartRamp {
                    robot: None,
                    targets: vec![("s".into(), 0.1)],
                    duration: 0.2,
                }],
                Condition::Done,
            )],
            "give the action a robot",
        );
        // One driver per robot per step; a second on the same arm is out.
        check(
            vec![step(
                "x",
                vec![
                    Action::StartMotion {
                        motion: "a_in".into(),
                    },
                    Action::StartRamp {
                        robot: Some("a".into()),
                        targets: vec![("s".into(), 0.1)],
                        duration: 0.2,
                    },
                ],
                Condition::Done,
            )],
            "per robot",
        );
        // One carrier at a time — a handover needs the detach in between.
        check(
            vec![
                step(
                    "a grabs",
                    vec![Action::Attach {
                        robot: Some("a".into()),
                        object: "box".into(),
                        link: None,
                        touch_links: None,
                    }],
                    Condition::Immediately,
                ),
                step(
                    "b grabs too",
                    vec![Action::Attach {
                        robot: Some("b".into()),
                        object: "box".into(),
                        link: None,
                        touch_links: None,
                    }],
                    Condition::Immediately,
                ),
            ],
            "already attached",
        );
        // robot_done must name a robot that exists.
        check(
            vec![step(
                "x",
                vec![],
                Condition::RobotDone {
                    robot: "ghost".into(),
                },
            )],
            "unknown robot",
        );
    }
}

#[cfg(test)]
mod tracking_tests {
    use super::*;
    use crate::seq::{Action, Condition, Device, DeviceKind, Sequence, Step};
    use botrail_model::Geometry;
    use nalgebra::{Translation3, UnitQuaternion, Vector3};

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity())
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
        }
    }

    /// A 3-axis gantry: enough DOF to hold a pose while a part slides by.
    fn gantry() -> Scene {
        let urdf = r#"
        <robot name="gantry">
          <link name="base"/>
          <link name="bridge"/>
          <link name="carriage"/>
          <link name="tool">
            <visual><geometry><box size="0.05 0.05 0.05"/></geometry></visual>
          </link>
          <joint name="jx" type="prismatic">
            <parent link="base"/><child link="bridge"/>
            <axis xyz="1 0 0"/>
            <limit lower="-2" upper="2" effort="1" velocity="1"/>
          </joint>
          <joint name="jy" type="prismatic">
            <parent link="bridge"/><child link="carriage"/>
            <axis xyz="0 1 0"/>
            <limit lower="-2" upper="2" effort="1" velocity="1"/>
          </joint>
          <joint name="jz" type="prismatic">
            <parent link="carriage"/><child link="tool"/>
            <axis xyz="0 0 1"/>
            <limit lower="-2" upper="2" effort="1" velocity="1"/>
          </joint>
        </robot>"#;
        Scene::new(std::sync::Arc::new(
            botrail_model::RobotModel::from_urdf_str(urdf).unwrap(),
        ))
    }

    /// Belt at 0.2 m/s with the part starting under the taught pose.
    fn cell() -> Scene {
        let mut scene = gantry();
        scene.set_joint_positions(vec![0.0, 0.0, 0.4]).unwrap();
        scene
            .add_obstacle(
                "part",
                Geometry::Box {
                    size: Vector3::new(0.05, 0.05, 0.05),
                },
                iso(0.0, 0.0, 0.0),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "belt".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: iso(0.5, 0.0, 0.0),
                zone_size: Vector3::new(3.0, 0.4, 0.4),
                velocity: Vector3::new(0.2, 0.0, 0.0),
                running: true,
            },
        });
        scene
    }

    fn tool_pose(scene: &Scene, q: &[f64]) -> Isometry3<f64> {
        let link = scene.robot().link_index("tool").unwrap();
        scene.fk(q).unwrap()[link]
    }

    /// The taught descent is carried by the belt: the tool meets the part
    /// where the part *is*, not where it was taught.
    #[test]
    fn tracked_ramp_follows_the_moving_part() {
        let mut scene = cell();
        scene.upsert_sequence(Sequence {
            name: "pick".into(),
            steps: vec![
                step(
                    "latch",
                    vec![Action::Track {
                        robot: None,
                        object: "part".into(),
                        link: Some("tool".into()),
                    }],
                    Condition::Immediately,
                ),
                step(
                    "descend",
                    vec![Action::StartRamp {
                        robot: None,
                        // taught: straight down onto the part at x = 0
                        targets: vec![("jz".into(), 0.05)],
                        duration: 0.5,
                    }],
                    Condition::Done,
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("pick", &RolloutOptions::default())
            .unwrap();
        assert!((tl.duration - 0.5).abs() < 1e-9, "{}", tl.duration);

        // The part has travelled 0.2 * 0.5 = 0.1 m; so has the tool, which
        // also completed the taught 0.35 m descent.
        let end = tool_pose(&scene, &tl.robots[0].trajectory.sample(tl.duration));
        assert!(
            (end.translation.x - 0.1).abs() < 1e-4,
            "{}",
            end.translation.x
        );
        assert!(
            (end.translation.z - 0.05).abs() < 1e-4,
            "{}",
            end.translation.z
        );
        // Mid-ramp the tool sits over the part throughout, not behind it.
        for i in 0..=10 {
            let t = tl.duration * f64::from(i) / 10.0;
            let pose = tool_pose(&scene, &tl.robots[0].trajectory.sample(t));
            assert!(
                (pose.translation.x - 0.2 * t).abs() < 1e-3,
                "t={t}: tool x {} vs part {}",
                pose.translation.x,
                0.2 * t
            );
        }
    }

    /// Grasping the tracked part freezes the offset, so the lift after it is
    /// straight up — the part is not dragged back to the taught station.
    #[test]
    fn grasp_freezes_the_offset_and_the_lift_stays_put() {
        let mut scene = cell();
        scene.upsert_sequence(Sequence {
            name: "pick".into(),
            steps: vec![
                step(
                    "latch",
                    vec![Action::Track {
                        robot: None,
                        object: "part".into(),
                        link: Some("tool".into()),
                    }],
                    Condition::Immediately,
                ),
                step(
                    "descend",
                    vec![Action::StartRamp {
                        robot: None,
                        targets: vec![("jz".into(), 0.05)],
                        duration: 0.5,
                    }],
                    Condition::Done,
                ),
                step(
                    "grasp",
                    vec![Action::Attach {
                        robot: None,
                        object: "part".into(),
                        link: Some("tool".into()),
                        touch_links: Some(vec!["tool".into()]),
                    }],
                    Condition::Immediately,
                ),
                step(
                    "lift",
                    vec![Action::StartRamp {
                        robot: None,
                        targets: vec![("jz".into(), 0.4)],
                        duration: 0.5,
                    }],
                    Condition::Done,
                ),
                step(
                    "release",
                    vec![Action::Untrack { robot: None }],
                    Condition::Immediately,
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("pick", &RolloutOptions::default())
            .unwrap();
        let track = tl
            .objects
            .iter()
            .find(|o| o.name == "part")
            .expect("the grasped part is tracked");
        let poses = |t: f64| {
            let q = tl.robots[0].trajectory.sample(t);
            let link_poses = scene.fk(&q).unwrap();
            (
                tool_pose(&scene, &q),
                SequenceTimeline::object_pose(track, std::slice::from_ref(&link_poses), t).unwrap(),
            )
        };
        let (_, at_grasp) = poses(0.5);
        let (tool_end, at_end) = poses(tl.duration);
        // Lifted straight up from where it was caught.
        assert!((at_end.translation.x - at_grasp.translation.x).abs() < 1e-6);
        assert!((at_end.translation.z - at_grasp.translation.z - 0.35).abs() < 1e-4);
        // ... and it is still in the gripper.
        assert!((tool_end.translation.x - at_end.translation.x).abs() < 1e-5);
    }

    /// Releasing a track never moves the robot: the world-frame moves after
    /// it start from wherever the tracked part left the arm.
    #[test]
    fn untrack_holds_the_pose_it_reached() {
        let mut scene = cell();
        scene.upsert_sequence(Sequence {
            name: "pick".into(),
            steps: vec![
                step(
                    "latch",
                    vec![Action::Track {
                        robot: None,
                        object: "part".into(),
                        link: Some("tool".into()),
                    }],
                    Condition::Immediately,
                ),
                step("follow", vec![], Condition::Elapsed { seconds: 0.5 }),
                step(
                    "release",
                    vec![Action::Untrack { robot: None }],
                    Condition::Immediately,
                ),
                step("settle", vec![], Condition::Elapsed { seconds: 0.2 }),
            ],
        });
        let tl = scene
            .simulate_sequence("pick", &RolloutOptions::default())
            .unwrap();
        let at_release = tool_pose(&scene, &tl.robots[0].trajectory.sample(0.5));
        let at_end = tool_pose(&scene, &tl.robots[0].trajectory.sample(tl.duration));
        assert!((at_release.translation.x - 0.1).abs() < 1e-4);
        assert!(
            (at_end.translation.vector - at_release.translation.vector).norm() < 1e-9,
            "the robot moved after the release"
        );
    }

    /// Servoing a fingertip while the gripper ramps is rejected: the solve
    /// could spend the grip chasing the part (the default link — the tool
    /// mount — is what keeps this from happening in the first place).
    #[test]
    fn ramping_the_tracked_gripper_is_rejected() {
        let urdf = r#"
        <robot name="arm">
          <link name="base"/>
          <link name="wrist"/>
          <link name="left"/>
          <link name="right"/>
          <joint name="elbow" type="prismatic">
            <parent link="base"/><child link="wrist"/>
            <axis xyz="1 0 0"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/>
          </joint>
          <joint name="finger_left" type="prismatic">
            <parent link="wrist"/><child link="left"/>
            <axis xyz="0 1 0"/>
            <limit lower="0" upper="0.04" effort="1" velocity="1"/>
          </joint>
          <joint name="finger_right" type="prismatic">
            <parent link="wrist"/><child link="right"/>
            <axis xyz="0 -1 0"/>
            <limit lower="0" upper="0.04" effort="1" velocity="1"/>
          </joint>
        </robot>"#;
        let mut scene = Scene::new(std::sync::Arc::new(
            botrail_model::RobotModel::from_urdf_str(urdf).unwrap(),
        ));
        scene
            .add_obstacle(
                "part",
                Geometry::Box {
                    size: Vector3::new(0.05, 0.05, 0.05),
                },
                iso(0.0, 0.0, 0.0),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step(
                    "latch",
                    vec![Action::Track {
                        robot: None,
                        object: "part".into(),
                        link: Some("left".into()),
                    }],
                    Condition::Immediately,
                ),
                step(
                    "close",
                    vec![Action::StartRamp {
                        robot: None,
                        targets: vec![("finger_left".into(), 0.02)],
                        duration: 0.4,
                    }],
                    Condition::Done,
                ),
            ],
        });
        let err = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .expect_err("the gripper joint moves the servoed link")
            .to_string();
        assert!(err.contains("fights the track"), "{err}");
        assert!(err.contains("wrist"), "{err}");
    }

    /// A mimic joint is commandable, just not directly — the rejection has
    /// to name the joint that actually drives it.
    #[test]
    fn ramping_a_mimic_joint_points_at_its_source() {
        let urdf = r#"
        <robot name="arm">
          <link name="base"/><link name="left"/><link name="right"/>
          <joint name="finger_left" type="prismatic">
            <parent link="base"/><child link="left"/>
            <origin xyz="0 0.02 0"/>
            <axis xyz="0 1 0"/>
            <limit lower="0" upper="0.04" effort="1" velocity="1"/>
          </joint>
          <joint name="finger_right" type="prismatic">
            <parent link="base"/><child link="right"/>
            <origin xyz="0 -0.02 0"/>
            <axis xyz="0 1 0"/>
            <limit lower="-0.04" upper="0" effort="1" velocity="1"/>
            <mimic joint="finger_left" multiplier="-1"/>
          </joint>
        </robot>"#;
        let mut scene = Scene::new(std::sync::Arc::new(
            botrail_model::RobotModel::from_urdf_str(urdf).unwrap(),
        ));
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![step(
                "close",
                vec![Action::StartRamp {
                    robot: None,
                    targets: vec![("finger_right".into(), -0.02)],
                    duration: 0.4,
                }],
                Condition::Done,
            )],
        });
        let err = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .expect_err("a driven joint cannot be ramped on its own")
            .to_string();
        assert!(err.contains("follows `finger_left`"), "{err}");
    }

    /// Authoring-time rules: one track at a time, no stray release, and no
    /// planned motions while the frame is moving.
    #[test]
    fn tracking_rules_are_validated() {
        let base = cell();
        let check = |steps: Vec<Step>, needle: &str| {
            let mut scene = base.clone();
            scene
                .add_segment(
                    "go",
                    crate::motion::Segment {
                        kind: crate::motion::SegmentKind::Joint,
                        goal_positions: vec![0.1, 0.0, 0.4],
                        constraints: vec![],
                    },
                )
                .unwrap();
            scene.upsert_sequence(Sequence {
                name: "s".into(),
                steps,
            });
            let err = scene
                .simulate_sequence("s", &RolloutOptions::default())
                .expect_err("expected `{needle}`")
                .to_string();
            assert!(err.contains(needle), "expected `{needle}` in `{err}`");
        };
        let track = || Action::Track {
            robot: None,
            object: "part".into(),
            link: None,
        };
        check(
            vec![step(
                "twice",
                vec![track(), track()],
                Condition::Immediately,
            )],
            "already tracking",
        );
        check(
            vec![step(
                "loose",
                vec![Action::Untrack { robot: None }],
                Condition::Immediately,
            )],
            "without an active track",
        );
        check(
            vec![
                step("latch", vec![track()], Condition::Immediately),
                step(
                    "move",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::Done,
                ),
            ],
            "release the track first",
        );
        check(
            vec![step(
                "ghost",
                vec![Action::Track {
                    robot: None,
                    object: "nope".into(),
                    link: None,
                }],
                Condition::Immediately,
            )],
            "unknown obstacle",
        );
    }
}

#[cfg(test)]
mod vehicle_tests {
    use super::tests::*;
    use super::*;
    use crate::seq::{Device, DeviceKind, Step, VehiclePath};
    use botrail_model::Geometry;
    use nalgebra::{Point2, Translation3, UnitQuaternion};
    use std::f64::consts::{FRAC_PI_2, PI};

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity())
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
        }
    }

    /// An L: 2 m along +x, then 1 m along +y. Stations at both ends.
    fn l_path() -> VehiclePath {
        VehiclePath {
            waypoints: vec![
                Point2::new(0.0, 0.0),
                Point2::new(2.0, 0.0),
                Point2::new(2.0, 1.0),
            ],
            stations: vec![("a".into(), 0), ("c".into(), 2)],
            ring: false,
        }
    }

    /// 0.5 m/s cruise, 90°/s pivot: the L takes 4 + 1 + 2 = 7 s.
    fn agv(body: Vec<String>) -> Device {
        Device {
            name: "agv".into(),
            kind: DeviceKind::Vehicle {
                path: l_path(),
                body,
                speed: 0.5,
                turn_speed: FRAC_PI_2,
                start: "a".into(),
                allow_reverse: false,
                tray: None,
            },
        }
    }

    fn goto(station: &str) -> Action {
        Action::Device {
            device: "agv".into(),
            command: DeviceCommand::Goto {
                station: station.into(),
            },
        }
    }

    fn device_done() -> Condition {
        Condition::DeviceDone {
            device: "agv".into(),
        }
    }

    fn chassis_scene() -> Scene {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "chassis",
                Geometry::Box {
                    size: Vector3::new(0.2, 0.2, 0.2),
                },
                iso(0.3, 0.2, 0.1),
            )
            .unwrap();
        scene.upsert_device(agv(vec!["chassis".into()]));
        scene
    }

    /// No robot drives, so `object_pose` never needs FK.
    fn no_fk(scene: &Scene) -> Vec<Vec<Isometry3<f64>>> {
        scene
            .robots()
            .iter()
            .map(|r| vec![Isometry3::identity(); r.model.links.len()])
            .collect()
    }

    #[test]
    fn vehicle_arrives_at_the_analytic_time_with_the_body_carried() {
        let mut scene = chassis_scene();
        scene.upsert_sequence(Sequence {
            name: "out".into(),
            steps: vec![
                step("go", vec![goto("c")], device_done()),
                step("dwell", vec![], Condition::Elapsed { seconds: 0.2 }),
            ],
        });
        let tl = scene
            .simulate_sequence("out", &RolloutOptions::default())
            .unwrap();

        // 2 m at 0.5 + 90° at 90°/s + 1 m at 0.5, plus the dwell.
        assert!(
            (tl.duration - 7.2).abs() < 0.021,
            "duration = {}",
            tl.duration
        );
        let arrival = tl.step_spans[0].end;
        assert!((arrival - 7.0).abs() < 0.011, "arrival = {arrival}");

        // The moving lane goes on at dispatch, off at arrival.
        let lane = tl.signals.iter().find(|s| s.name == "agv").unwrap();
        assert_eq!(lane.edges.len(), 3, "{:?}", lane.edges);
        assert_eq!(lane.edges[1], (0.0, true));
        assert!(!lane.edges[2].1 && (lane.edges[2].0 - 7.0).abs() < 0.011);

        // Net rigid motion: +2 x, pivot +90° about (2, 0), +1 y.
        let track = tl.objects.iter().find(|o| o.name == "chassis").unwrap();
        let end = SequenceTimeline::object_pose(track, &no_fk(&scene), tl.duration).unwrap();
        assert!(
            (end.translation.vector - Vector3::new(1.8, 1.3, 0.1)).norm() < 1e-9,
            "end = {:?}",
            end.translation.vector
        );
        let quarter = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
        assert!(end.rotation.angle_to(&quarter) < 1e-9);
        // After arrival the body settles into a hold at the parked pose.
        assert!(matches!(track.spans.last(), Some(TrackSpan::Hold { .. })));
    }

    #[test]
    fn vehicle_spans_bake_one_span_per_leg_and_resample_exactly() {
        let mut scene = chassis_scene();
        scene.upsert_sequence(Sequence {
            name: "out".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let tl = scene
            .simulate_sequence("out", &RolloutOptions::default())
            .unwrap();
        let track = tl.objects.iter().find(|o| o.name == "chassis").unwrap();

        // Whole legs merge: straight, pivot, straight — three spans, not 700.
        assert_eq!(track.spans.len(), 3, "{:?}", track.spans);
        assert!(matches!(track.spans[0], TrackSpan::Linear { .. }));
        assert!(matches!(track.spans[1], TrackSpan::Pivot { .. }));
        assert!(matches!(track.spans[2], TrackSpan::Linear { .. }));

        let fk = no_fk(&scene);
        // Mid-straight: 1 s at 0.5 m/s along +x.
        let mid = SequenceTimeline::object_pose(track, &fk, 1.0).unwrap();
        assert!((mid.translation.vector - Vector3::new(0.8, 0.2, 0.1)).norm() < 1e-9);
        // Mid-turn (t = 4.5 ⇒ 45° about (2, 0)): closed form, no resample grid.
        let mid = SequenceTimeline::object_pose(track, &fk, 4.5).unwrap();
        let phi = FRAC_PI_2 * 0.5;
        let expected = Vector3::new(
            2.0 + 0.3 * phi.cos() - 0.2 * phi.sin(),
            0.3 * phi.sin() + 0.2 * phi.cos(),
            0.1,
        );
        assert!(
            (mid.translation.vector - expected).norm() < 1e-9,
            "mid = {:?}, expected = {expected:?}",
            mid.translation.vector
        );
        let eighth = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), phi);
        assert!(mid.rotation.angle_to(&eighth) < 1e-9);
    }

    #[test]
    fn vehicle_round_trip_composes_to_a_half_turn_about_home() {
        let mut scene = chassis_scene();
        scene.upsert_sequence(Sequence {
            name: "cycle".into(),
            steps: vec![
                step("out", vec![goto("c")], device_done()),
                step("back", vec![goto("a")], device_done()),
            ],
        });
        let tl = scene
            .simulate_sequence("cycle", &RolloutOptions::default())
            .unwrap();
        // Return: about-face (2 s) + 1 m (2 s) + 90° (1 s) + 2 m (4 s) = 9 s.
        assert!(
            (tl.duration - 16.0).abs() < 0.031,
            "duration = {}",
            tl.duration
        );
        // The vehicle is back at `a` facing −x: the cycle's net rigid motion
        // is a half turn about home, and the body carries it exactly.
        let track = tl.objects.iter().find(|o| o.name == "chassis").unwrap();
        let end = SequenceTimeline::object_pose(track, &no_fk(&scene), tl.duration).unwrap();
        assert!(
            (end.translation.vector - Vector3::new(-0.3, -0.2, 0.1)).norm() < 1e-9,
            "end = {:?}",
            end.translation.vector
        );
        let half = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI);
        assert!(end.rotation.angle_to(&half) < 1e-9);
    }

    #[test]
    fn vehicle_rollout_is_deterministic() {
        let mut scene = chassis_scene();
        scene.upsert_sequence(Sequence {
            name: "out".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let options = RolloutOptions::default();
        let a = scene.simulate_sequence("out", &options).unwrap();
        let b = scene.simulate_sequence("out", &options).unwrap();
        assert_eq!(a.duration.to_bits(), b.duration.to_bits());
        let fk = no_fk(&scene);
        let track = |tl: &SequenceTimeline| {
            tl.objects
                .iter()
                .find(|o| o.name == "chassis")
                .unwrap()
                .clone()
        };
        let (ta, tb) = (track(&a), track(&b));
        assert_eq!(ta.spans.len(), tb.spans.len());
        for k in 0..=72 {
            let t = k as f64 * 0.1;
            let pa = SequenceTimeline::object_pose(&ta, &fk, t).unwrap();
            let pb = SequenceTimeline::object_pose(&tb, &fk, t).unwrap();
            for i in 0..3 {
                assert_eq!(
                    pa.translation.vector[i].to_bits(),
                    pb.translation.vector[i].to_bits()
                );
            }
            assert_eq!(
                pa.rotation
                    .coords
                    .iter()
                    .map(|c| c.to_bits())
                    .collect::<Vec<_>>(),
                pb.rotation
                    .coords
                    .iter()
                    .map(|c| c.to_bits())
                    .collect::<Vec<_>>()
            );
        }
    }

    #[test]
    fn a_travelling_vehicle_reports_the_aisle_collision() {
        let mut scene = chassis_scene();
        scene
            .add_obstacle(
                "wall",
                Geometry::Box {
                    size: Vector3::new(0.2, 0.2, 0.2),
                },
                iso(1.0, 0.2, 0.1),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "out".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let err = scene
            .simulate_sequence("out", &RolloutOptions::default())
            .unwrap_err();
        match err {
            SeqError::VehicleCollision {
                t,
                vehicle,
                body,
                obstacle,
            } => {
                // Faces touch after 0.5 m of travel at 0.5 m/s.
                assert!((0.99..=1.05).contains(&t), "t = {t}");
                assert_eq!(vehicle, "agv");
                assert_eq!(body, "chassis");
                assert_eq!(obstacle, "wall");
            }
            other => panic!("expected VehicleCollision, got {other:?}"),
        }
    }

    #[test]
    fn goto_while_travelling_is_rejected() {
        let mut scene = chassis_scene();
        scene.upsert_sequence(Sequence {
            name: "out".into(),
            steps: vec![
                step("go", vec![goto("c")], Condition::Elapsed { seconds: 0.5 }),
                step("amend", vec![goto("a")], device_done()),
            ],
        });
        let err = scene
            .simulate_sequence("out", &RolloutOptions::default())
            .unwrap_err();
        assert!(
            err.to_string().contains("still travelling"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn vehicle_validation_names_the_problem() {
        let check = |scene: &Scene, sequence: &str, needle: &str| {
            let err = scene
                .simulate_sequence(sequence, &RolloutOptions::default())
                .unwrap_err();
            assert!(
                matches!(err, SeqError::Validation { .. }) && err.to_string().contains(needle),
                "expected `{needle}` in: {err}"
            );
        };
        // Unknown goto station.
        let mut scene = chassis_scene();
        scene.upsert_sequence(Sequence {
            name: "bad".into(),
            steps: vec![step("go", vec![goto("nowhere")], device_done())],
        });
        check(&scene, "bad", "no station `nowhere`");
        // Goto on a non-vehicle.
        let mut scene = chassis_scene();
        scene.upsert_device(Device {
            name: "belt".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: iso(0.0, 0.0, 0.5),
                zone_size: Vector3::new(1.0, 1.0, 0.2),
                velocity: Vector3::new(0.1, 0.0, 0.0),
                running: false,
            },
        });
        scene.upsert_sequence(Sequence {
            name: "bad".into(),
            steps: vec![step(
                "go",
                vec![Action::Device {
                    device: "belt".into(),
                    command: DeviceCommand::Goto {
                        station: "a".into(),
                    },
                }],
                Condition::Immediately,
            )],
        });
        check(&scene, "bad", "not a vehicle");
        // A station pointing past the path.
        let mut scene = chassis_scene();
        scene.upsert_device(Device {
            name: "agv".into(),
            kind: DeviceKind::Vehicle {
                path: VehiclePath {
                    waypoints: vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)],
                    stations: vec![("a".into(), 0), ("ghost".into(), 9)],
                    ring: false,
                },
                body: vec!["chassis".into()],
                speed: 0.5,
                turn_speed: FRAC_PI_2,
                start: "a".into(),
                allow_reverse: false,
                tray: None,
            },
        });
        scene.upsert_sequence(Sequence {
            name: "any".into(),
            steps: vec![step("noop", vec![], Condition::Immediately)],
        });
        check(&scene, "any", "points at waypoint 9");
    }

    #[test]
    fn a_ring_walks_the_shorter_way_around() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "cart",
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                iso(0.0, 0.0, 1.5),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "agv".into(),
            kind: DeviceKind::Vehicle {
                path: VehiclePath {
                    waypoints: vec![
                        Point2::new(0.0, 0.0),
                        Point2::new(2.0, 0.0),
                        Point2::new(2.0, 2.0),
                        Point2::new(0.0, 2.0),
                    ],
                    stations: vec![("a".into(), 0), ("d".into(), 3)],
                    ring: true,
                },
                body: vec!["cart".into()],
                speed: 0.5,
                turn_speed: FRAC_PI_2,
                start: "a".into(),
                allow_reverse: false,
                tray: None,
            },
        });
        scene.upsert_sequence(Sequence {
            name: "out".into(),
            steps: vec![step("go", vec![goto("d")], device_done())],
        });
        let tl = scene
            .simulate_sequence("out", &RolloutOptions::default())
            .unwrap();
        // Backward around the ring: 90° turn (1 s) + one 2 m side (4 s),
        // not the three-side walk (6 m).
        assert!(
            (tl.duration - 5.0).abs() < 0.011,
            "duration = {}",
            tl.duration
        );
    }

    #[test]
    fn vehicle_device_round_trips_through_wire_and_project() {
        let scene = {
            let mut scene = chassis_scene();
            scene.upsert_sequence(Sequence {
                name: "out".into(),
                steps: vec![step("go", vec![goto("c")], device_done())],
            });
            scene
        };
        let msg = crate::wire::device_msg(&scene.devices()[0]);
        let json = serde_json::to_string(&msg).unwrap();
        let back: crate::wire::DeviceMsg = serde_json::from_str(&json).unwrap();
        assert_eq!(msg, back);
        let rebuilt = crate::wire::device_msg(&crate::wire::device_from_msg(&back));
        assert_eq!(msg, rebuilt);

        // The generated script re-authors the vehicle and the dispatch.
        let code = crate::project::generate_python(&scene.to_project());
        assert!(code.contains("scene.add_vehicle(\"agv\""), "{code}");
        assert!(code.contains("stations={\"a\": 0, \"c\": 2}"), "{code}");
        assert!(code.contains("bt.seq.goto(\"agv\", \"c\")"), "{code}");
        assert!(code.contains("bt.seq.device_done(\"agv\")"), "{code}");
    }
}

#[cfg(test)]
mod tray_tests {
    use super::tests::*;
    use super::*;
    use crate::seq::{Device, DeviceKind, Sensor, SensorKind, SensorWatch, Step, VehiclePath};
    use botrail_model::Geometry;
    use nalgebra::{Point2, Translation3, UnitQuaternion};
    use std::f64::consts::FRAC_PI_2;

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity())
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
        }
    }

    fn goto(station: &str) -> Action {
        Action::Device {
            device: "agv".into(),
            command: DeviceCommand::Goto {
                station: station.into(),
            },
        }
    }

    fn device_done() -> Condition {
        Condition::DeviceDone {
            device: "agv".into(),
        }
    }

    /// An L with a deck: 2 m along +x, pivot, 1 m along +y. The deck is a
    /// 0.4 m box centred over the frame, 0.2 m up.
    fn deck_scene() -> Scene {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "chassis",
                Geometry::Box {
                    size: Vector3::new(0.4, 0.3, 0.2),
                },
                iso(0.0, 0.0, 0.1),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "agv".into(),
            kind: DeviceKind::Vehicle {
                path: VehiclePath {
                    waypoints: vec![
                        Point2::new(0.0, 0.0),
                        Point2::new(2.0, 0.0),
                        Point2::new(2.0, 1.0),
                    ],
                    stations: vec![("a".into(), 0), ("c".into(), 2)],
                    ring: false,
                },
                body: vec!["chassis".into()],
                speed: 0.5,
                turn_speed: FRAC_PI_2,
                start: "a".into(),
                allow_reverse: false,
                tray: Some((iso(0.0, 0.0, 0.2), Vector3::new(0.4, 0.3, 0.2))),
            },
        });
        scene
    }

    fn no_fk(scene: &Scene) -> Vec<Vec<Isometry3<f64>>> {
        scene
            .robots()
            .iter()
            .map(|r| vec![Isometry3::identity(); r.model.links.len()])
            .collect()
    }

    #[test]
    fn a_part_on_the_deck_rides_the_whole_route() {
        let mut scene = deck_scene();
        // Resting on the deck at the start: inside the tray zone.
        scene
            .add_obstacle(
                "carton",
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                iso(0.05, 0.0, 0.25),
            )
            .unwrap();
        // Standing beside the vehicle, outside the zone: must not move.
        scene
            .add_obstacle(
                "bystander",
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                iso(0.0, 0.6, 0.05),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "haul".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let tl = scene
            .simulate_sequence("haul", &RolloutOptions::default())
            .unwrap();

        let fk = no_fk(&scene);
        let track = |name: &str| tl.objects.iter().find(|o| o.name == name).cloned();
        let carton = track("carton").expect("the load has a track");
        let end = SequenceTimeline::object_pose(&carton, &fk, tl.duration).unwrap();
        // Same rigid motion as the body: +2 x, pivot +90° about (2, 0), +1 y.
        // The load sat 0.05 ahead of the frame, so the turn swings it to +y.
        assert!(
            (end.translation.vector - Vector3::new(2.0, 1.05, 0.25)).norm() < 1e-9,
            "end = {:?}",
            end.translation.vector
        );
        let quarter = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
        assert!(end.rotation.angle_to(&quarter) < 1e-9);
        // It stayed on the deck the whole way: the offset to the body is
        // constant, which is what "rides" has to mean.
        let chassis = track("chassis").expect("the body has a track");
        for k in 0..=70 {
            let t = k as f64 * 0.1;
            let c = SequenceTimeline::object_pose(&carton, &fk, t).unwrap();
            let b = SequenceTimeline::object_pose(&chassis, &fk, t).unwrap();
            let offset = b.inverse() * c;
            assert!(
                (offset.translation.vector - Vector3::new(0.05, 0.0, 0.15)).norm() < 1e-9,
                "t = {t}: {:?}",
                offset.translation.vector
            );
        }
        // Nothing was picked up that was not on the deck.
        assert!(track("bystander").is_none(), "the bystander must not move");
    }

    #[test]
    fn a_part_the_deck_drives_under_joins_the_load() {
        let mut scene = deck_scene();
        // Hovering over the second leg, above the chassis but inside the
        // deck zone: the vehicle drives under it and it becomes cargo. A
        // part only has to *be* on the deck — there is no load action.
        scene
            .add_obstacle(
                "carton",
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                iso(2.0, 0.6, 0.28),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "haul".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let tl = scene
            .simulate_sequence("haul", &RolloutOptions::default())
            .unwrap();
        let fk = no_fk(&scene);
        let carton = tl.objects.iter().find(|o| o.name == "carton").unwrap();

        // Still where it was left while the vehicle is on the first leg.
        let early = SequenceTimeline::object_pose(carton, &fk, 2.0).unwrap();
        assert!(
            (early.translation.vector - Vector3::new(2.0, 0.6, 0.28)).norm() < 1e-9,
            "early = {:?}",
            early.translation.vector
        );
        // The deck's front edge reaches it at vy = 0.4, i.e. t = 5.8 s, and
        // from there it travels the remaining 0.6 m with the vehicle.
        let end = SequenceTimeline::object_pose(carton, &fk, tl.duration).unwrap();
        assert!(
            (end.translation.vector - Vector3::new(2.0, 1.2, 0.28)).norm() < 1e-9,
            "end = {:?}",
            end.translation.vector
        );
        // Its first span is the rest before it was picked up.
        assert!(matches!(carton.spans.first(), Some(TrackSpan::Hold { .. })));
    }

    #[test]
    fn a_grasped_part_is_the_robots_not_the_decks() {
        let mut scene = deck_scene();
        scene
            .add_obstacle(
                "carton",
                Geometry::Box {
                    size: Vector3::new(0.05, 0.05, 0.05),
                },
                iso(0.0, 0.0, 0.25),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "hold".into(),
            steps: vec![
                step(
                    "grasp",
                    vec![Action::Attach {
                        robot: None,
                        object: "carton".into(),
                        link: None,
                        touch_links: None,
                    }],
                    Condition::Immediately,
                ),
                step("go", vec![goto("c")], device_done()),
            ],
        });
        let tl = scene
            .simulate_sequence("hold", &RolloutOptions::default())
            .unwrap();
        let carton = tl.objects.iter().find(|o| o.name == "carton").unwrap();
        // Grasp wins: every span is a Follow, never a vehicle span.
        assert!(
            carton
                .spans
                .iter()
                .all(|s| matches!(s, TrackSpan::Follow { .. })),
            "{:?}",
            carton.spans
        );
    }

    #[test]
    fn a_mounted_sensor_travels_with_its_vehicle() {
        let mut scene = deck_scene();
        scene
            .add_obstacle(
                "carton",
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                iso(0.05, 0.0, 0.25),
            )
            .unwrap();
        // A load-present eye over the deck, authored in the vehicle frame.
        scene.upsert_sensor(Sensor {
            name: "loaded".into(),
            kind: SensorKind::Zone {
                pose: iso(0.0, 0.0, 0.25),
                size: Vector3::new(0.4, 0.3, 0.2),
            },
            watch: SensorWatch::Objects(vec!["carton".into()]),
            mount: Some("agv".into()),
        });
        // The same zone bolted to the floor, for contrast.
        scene.upsert_sensor(Sensor {
            name: "fixture".into(),
            kind: SensorKind::Zone {
                pose: iso(0.0, 0.0, 0.25),
                size: Vector3::new(0.4, 0.3, 0.2),
            },
            watch: SensorWatch::Objects(vec!["carton".into()]),
            mount: None,
        });
        scene.upsert_sequence(Sequence {
            name: "haul".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let tl = scene
            .simulate_sequence("haul", &RolloutOptions::default())
            .unwrap();

        let lane = |name: &str| tl.signals.iter().find(|s| s.name == name).unwrap();
        // The mounted eye sees its load for the whole run — it moves with
        // it. (Every lane seeds false and records its first reading at t=0.)
        let loaded = lane("loaded");
        assert_eq!(
            loaded.edges,
            vec![(0.0, false), (0.0, true)],
            "{:?}",
            loaded.edges
        );
        // The floor fixture sees the same load at t = 0 and loses it as
        // soon as the vehicle pulls away — the mounted/fixed distinction in
        // one pair of lanes.
        let fixture = lane("fixture");
        assert_eq!(fixture.edges.len(), 3, "{:?}", fixture.edges);
        assert_eq!(&fixture.edges[..2], &[(0.0, false), (0.0, true)]);
        let (dropped_at, value) = fixture.edges[2];
        assert!(!value && dropped_at < 1.0, "dropped at {dropped_at}");
    }

    #[test]
    fn the_load_is_checked_against_the_aisle_too() {
        let mut scene = deck_scene();
        // A tall load on the deck, and a bridge the *body* clears (its top
        // is 0.2) but the load does not.
        scene
            .add_obstacle(
                "carton",
                Geometry::Box {
                    size: Vector3::new(0.2, 0.2, 0.4),
                },
                iso(0.0, 0.0, 0.25),
            )
            .unwrap();
        scene
            .add_obstacle(
                "bridge",
                Geometry::Box {
                    size: Vector3::new(0.1, 1.0, 0.1),
                },
                iso(1.0, 0.0, 0.45),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "haul".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let err = scene
            .simulate_sequence("haul", &RolloutOptions::default())
            .unwrap_err();
        match err {
            SeqError::VehicleCollision { body, obstacle, .. } => {
                // The *load* is what hits it — the body passes under.
                assert_eq!(body, "carton");
                assert_eq!(obstacle, "bridge");
            }
            other => panic!("expected VehicleCollision, got {other:?}"),
        }
    }

    #[test]
    fn tray_and_mount_round_trip_through_wire() {
        let scene = deck_scene();
        let msg = crate::wire::device_msg(&scene.devices()[0]);
        let json = serde_json::to_string(&msg).unwrap();
        let back: crate::wire::DeviceMsg = serde_json::from_str(&json).unwrap();
        assert_eq!(msg, back);
        assert_eq!(
            crate::wire::device_msg(&crate::wire::device_from_msg(&back)),
            msg
        );
        // A vehicle without a deck keeps its pre-V1 JSON shape.
        let mut plain = scene.devices()[0].clone();
        if let DeviceKind::Vehicle { tray, .. } = &mut plain.kind {
            *tray = None;
        }
        let json = serde_json::to_string(&crate::wire::device_msg(&plain)).unwrap();
        assert!(!json.contains("tray"), "{json}");

        let sensor = Sensor {
            name: "loaded".into(),
            kind: SensorKind::Zone {
                pose: iso(0.0, 0.0, 0.25),
                size: Vector3::new(0.4, 0.3, 0.2),
            },
            watch: SensorWatch::AllObjects,
            mount: Some("agv".into()),
        };
        let msg = crate::wire::sensor_msg(&sensor);
        let back: crate::wire::SensorMsg =
            serde_json::from_str(&serde_json::to_string(&msg).unwrap()).unwrap();
        assert_eq!(back.mount.as_deref(), Some("agv"));
        assert_eq!(crate::wire::sensor_from_msg(&back).mount, sensor.mount);
    }

    #[test]
    fn mount_validation_names_the_problem() {
        let check = |scene: &Scene, needle: &str| {
            let err = scene
                .simulate_sequence("haul", &RolloutOptions::default())
                .unwrap_err();
            assert!(
                matches!(err, SeqError::Validation { .. }) && err.to_string().contains(needle),
                "expected `{needle}` in: {err}"
            );
        };
        let mut scene = deck_scene();
        scene.upsert_sequence(Sequence {
            name: "haul".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let mut ghost = scene.clone();
        ghost.upsert_sensor(Sensor {
            name: "eye".into(),
            kind: SensorKind::Zone {
                pose: iso(0.0, 0.0, 0.2),
                size: Vector3::new(0.1, 0.1, 0.1),
            },
            watch: SensorWatch::AllObjects,
            mount: Some("nowhere".into()),
        });
        check(&ghost, "unknown device `nowhere`");

        let mut belt = scene.clone();
        belt.upsert_device(Device {
            name: "belt".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: iso(0.0, 0.0, 0.5),
                zone_size: Vector3::new(1.0, 1.0, 0.2),
                velocity: Vector3::new(0.1, 0.0, 0.0),
                running: false,
            },
        });
        belt.upsert_sensor(Sensor {
            name: "eye".into(),
            kind: SensorKind::Zone {
                pose: iso(0.0, 0.0, 0.2),
                size: Vector3::new(0.1, 0.1, 0.1),
            },
            watch: SensorWatch::AllObjects,
            mount: Some("belt".into()),
        });
        check(&belt, "not a vehicle");
    }
}

#[cfg(test)]
mod mount_tests {
    use super::tests::*;
    use super::*;
    use crate::seq::{Device, DeviceKind, Step, VehiclePath};
    use botrail_model::Geometry;
    use nalgebra::{Point2, Translation3, UnitQuaternion};
    use std::f64::consts::FRAC_PI_2;

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity())
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
        }
    }

    fn goto(station: &str) -> Action {
        Action::Device {
            device: "amr".into(),
            command: DeviceCommand::Goto {
                station: station.into(),
            },
        }
    }

    fn device_done() -> Condition {
        Condition::DeviceDone {
            device: "amr".into(),
        }
    }

    /// The 1-DOF arm of `sample_scene`, riding a chassis that runs 2 m
    /// along +x then 1 m along +y. Mount 0.3 m up.
    fn amr_scene() -> Scene {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "chassis",
                Geometry::Box {
                    size: Vector3::new(0.4, 0.3, 0.2),
                },
                iso(0.0, 0.0, 0.1),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "amr".into(),
            kind: DeviceKind::Vehicle {
                path: VehiclePath {
                    waypoints: vec![
                        Point2::new(0.0, 0.0),
                        Point2::new(2.0, 0.0),
                        Point2::new(2.0, 1.0),
                    ],
                    stations: vec![("a".into(), 0), ("c".into(), 2)],
                    ring: false,
                },
                body: vec!["chassis".into()],
                speed: 0.5,
                turn_speed: FRAC_PI_2,
                start: "a".into(),
                allow_reverse: false,
                tray: None,
            },
        });
        scene.mount_robot(0, "amr", iso(0.0, 0.0, 0.3)).unwrap();
        scene
    }

    #[test]
    fn mounting_places_the_base_on_the_parked_vehicle() {
        let scene = amr_scene();
        // Parked at (0, 0) facing +x, so the base is straight up from it.
        let base = scene.robots()[0].base_pose();
        assert!((base.translation.vector - Vector3::new(0.0, 0.0, 0.3)).norm() < 1e-12);
        assert!(scene.robot_mount(0).is_some());
    }

    #[test]
    fn the_base_rides_the_vehicle_and_rests_when_it_stops() {
        let mut scene = amr_scene();
        scene.upsert_sequence(Sequence {
            name: "go".into(),
            steps: vec![
                step("drive", vec![goto("c")], device_done()),
                step("dwell", vec![], Condition::Elapsed { seconds: 2.0 }),
            ],
        });
        let tl = scene
            .simulate_sequence("go", &RolloutOptions::default())
            .unwrap();
        let track = &tl.robots[0];
        let base = track
            .base
            .as_ref()
            .expect("a mounted robot has a base track");
        assert!(!base.is_empty());

        // Mid-straight, mid-turn, and after arrival.
        let at = |t: f64| SequenceTimeline::base_pose(track, t).unwrap();
        assert!((at(1.0).translation.vector - Vector3::new(0.5, 0.0, 0.3)).norm() < 1e-9);
        // The turn is about the vehicle frame, which the base sits directly
        // above — so it turns on the spot and only its orientation changes.
        let mid = at(4.5);
        assert!((mid.translation.vector - Vector3::new(2.0, 0.0, 0.3)).norm() < 1e-9);
        let eighth = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2 * 0.5);
        assert!(mid.rotation.angle_to(&eighth) < 1e-9);
        // Parked: the base rests instead of coasting on past the timeline —
        // the last travelling span must not simply be stretched to the end.
        let arrived = at(7.05);
        for t in (71..=90).map(|k| k as f64 * 0.1) {
            let p = at(t);
            assert!(
                (p.translation.vector - arrived.translation.vector).norm() < 1e-9,
                "base drifted at t = {t}: {:?}",
                p.translation.vector
            );
        }
        // ...and it agrees with the body it is bolted to, exactly.
        let chassis = tl.objects.iter().find(|o| o.name == "chassis").unwrap();
        let fk = vec![vec![
            Isometry3::identity();
            scene.robots()[0].model.links.len()
        ]];
        for k in 0..=90 {
            let t = k as f64 * 0.1;
            let body = SequenceTimeline::object_pose(chassis, &fk, t).unwrap();
            let offset = body.inverse() * at(t);
            assert!(
                (offset.translation.vector - Vector3::new(0.0, 0.0, 0.2)).norm() < 1e-9,
                "t = {t}: {:?}",
                offset.translation.vector
            );
        }
    }

    #[test]
    fn a_ramp_may_run_while_driving_but_a_plan_may_not() {
        let mut scene = amr_scene();
        joint_motion(&mut scene, "reach", 0.5);
        // A ramp alongside the drive: allowed, and the arm ends up moved.
        scene.upsert_sequence(Sequence {
            name: "stow".into(),
            steps: vec![step(
                "drive",
                vec![
                    goto("c"),
                    Action::StartRamp {
                        robot: None,
                        targets: vec![("j".into(), 0.4)],
                        duration: 1.0,
                    },
                ],
                device_done(),
            )],
        });
        let tl = scene
            .simulate_sequence("stow", &RolloutOptions::default())
            .unwrap();
        assert!((tl.robots[0].trajectory.sample(tl.duration)[0] - 0.4).abs() < 1e-9);

        // A planned motion in the same step: rejected by name.
        scene.upsert_sequence(Sequence {
            name: "bad".into(),
            steps: vec![step(
                "drive",
                vec![
                    goto("c"),
                    Action::StartMotion {
                        motion: "reach".into(),
                    },
                ],
                device_done(),
            )],
        });
        let err = scene
            .simulate_sequence("bad", &RolloutOptions::default())
            .unwrap_err();
        assert!(
            err.to_string()
                .contains("cannot start while `amr` is driving"),
            "unexpected: {err}"
        );
    }

    #[test]
    fn mounting_rejects_a_non_vehicle_and_an_unknown_device() {
        let mut scene = amr_scene();
        scene.upsert_device(Device {
            name: "belt".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: iso(0.0, 0.0, 0.5),
                zone_size: Vector3::new(1.0, 1.0, 0.2),
                velocity: Vector3::new(0.1, 0.0, 0.0),
                running: false,
            },
        });
        let err = scene
            .mount_robot(0, "belt", Isometry3::identity())
            .unwrap_err();
        assert!(err.to_string().contains("not a vehicle"), "{err}");
        let err = scene
            .mount_robot(0, "nowhere", Isometry3::identity())
            .unwrap_err();
        assert!(err.to_string().contains("nowhere"), "{err}");
    }
}

#[cfg(test)]
mod parallel_program_tests {
    use super::tests::{joint_motion, sample_scene};
    use super::*;
    use crate::seq::{Device, DeviceKind, Step};
    use botrail_model::Geometry;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity())
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
        }
    }

    fn ramp(joint: &str, value: f64, duration: f64) -> Action {
        Action::StartRamp {
            robot: None,
            targets: vec![(joint.to_string(), value)],
            duration,
        }
    }

    /// A scene with the 1-DOF arm, a box parked on a belt, and the belt.
    fn belt_scene(speed: f64) -> Scene {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "box",
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                iso(0.0, 2.0, 0.5),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "belt".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: iso(5.0, 2.0, 0.5),
                zone_size: Vector3::new(20.0, 1.0, 1.0),
                velocity: Vector3::new(speed, 0.0, 0.0),
                running: false,
            },
        });
        scene
    }

    #[test]
    fn advance_moves_exactly_the_asked_distance() {
        // 3.2 m at 0.4 m/s is the weld line's pitch: 800 full scans. And
        // 0.123 m is deliberately *not* a multiple of v·dt — the final
        // partial tick has to make up the fraction that the old
        // start/elapsed/stop pattern always lost.
        for (distance, speed) in [(3.2, 0.4), (0.123, 0.4), (0.05, 1.3)] {
            let mut scene = belt_scene(speed);
            scene.upsert_sequence(Sequence {
                name: "index".into(),
                steps: vec![step(
                    "advance",
                    vec![Action::Device {
                        device: "belt".into(),
                        command: DeviceCommand::Advance(distance),
                    }],
                    Condition::DeviceDone {
                        device: "belt".into(),
                    },
                )],
            });
            let tl = scene
                .simulate_sequence("index", &RolloutOptions::default())
                .unwrap();
            let no_fk: Vec<Vec<Isometry3<f64>>> = scene
                .robots()
                .iter()
                .map(|r| vec![Isometry3::identity(); r.model.links.len()])
                .collect();
            let track = tl.objects.iter().find(|o| o.name == "box").unwrap();
            let pose = SequenceTimeline::object_pose(track, &no_fk, tl.duration).unwrap();
            let moved = pose.translation.vector.x;
            assert!(
                (moved - distance).abs() < 1e-12,
                "asked {distance}, moved {moved} (err {:+.3e})",
                moved - distance
            );
            // The belt lane pulses on for the advance and off at the end.
            let lane = tl.signals.iter().find(|s| s.name == "belt").unwrap();
            assert_eq!(lane.edges.first().map(|(_, v)| *v), Some(false));
            assert_eq!(lane.edges.last().map(|(_, v)| *v), Some(false));
            assert!(lane.edges.iter().any(|(_, v)| *v));
        }
    }

    #[test]
    fn advance_rejects_free_running_and_double_commands() {
        let mut scene = belt_scene(0.4);
        scene.upsert_sequence(Sequence {
            name: "bad_running".into(),
            steps: vec![step(
                "go",
                vec![
                    Action::Device {
                        device: "belt".into(),
                        command: DeviceCommand::Start,
                    },
                    Action::Device {
                        device: "belt".into(),
                        command: DeviceCommand::Advance(1.0),
                    },
                ],
                Condition::DeviceDone {
                    device: "belt".into(),
                },
            )],
        });
        let err = scene
            .simulate_sequence("bad_running", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("free-running"), "{err}");

        let mut scene = belt_scene(0.4);
        scene.upsert_sequence(Sequence {
            name: "bad_double".into(),
            steps: vec![step(
                "go",
                vec![
                    Action::Device {
                        device: "belt".into(),
                        command: DeviceCommand::Advance(1.0),
                    },
                    Action::Device {
                        device: "belt".into(),
                        command: DeviceCommand::Advance(1.0),
                    },
                ],
                Condition::DeviceDone {
                    device: "belt".into(),
                },
            )],
        });
        let err = scene
            .simulate_sequence("bad_double", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("still advancing"), "{err}");

        // Statically: advance on a non-conveyor is a validation error.
        let mut scene = sample_scene();
        scene.upsert_device(Device {
            name: "lift".into(),
            kind: DeviceKind::LinearAxis {
                objects: vec![],
                axis: nalgebra::Unit::new_normalize(Vector3::z()),
                speed: 0.1,
                position: 0.0,
                range: (0.0, 1.0),
            },
        });
        scene.upsert_sequence(Sequence {
            name: "bad_kind".into(),
            steps: vec![step(
                "go",
                vec![Action::Device {
                    device: "lift".into(),
                    command: DeviceCommand::Advance(0.5),
                }],
                Condition::Immediately,
            )],
        });
        let err = scene
            .simulate_sequence("bad_kind", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("not a conveyor"), "{err}");
    }

    /// Two programs, one world: the belt program indexes while the robot
    /// program works, and the whole bake lasts as long as the *slower*
    /// of the two — which is the entire point of parallel programs.
    #[test]
    fn programs_run_concurrently_and_sync_through_signals() {
        let mut scene = belt_scene(0.4);
        scene.define_signal("welded", false);
        scene.upsert_sequence(Sequence {
            name: "station".into(),
            steps: vec![
                step("work", vec![ramp("j", 0.8, 1.0)], Condition::Done),
                step(
                    "done",
                    vec![Action::Set {
                        signal: "welded".into(),
                        value: true,
                    }],
                    Condition::Immediately,
                ),
            ],
        });
        scene.upsert_sequence(Sequence {
            name: "transfer".into(),
            steps: vec![
                step(
                    "wait_station",
                    vec![],
                    Condition::Signal {
                        name: "welded".into(),
                        value: true,
                    },
                ),
                step(
                    "index",
                    vec![Action::Device {
                        device: "belt".into(),
                        command: DeviceCommand::Advance(0.2),
                    }],
                    Condition::DeviceDone {
                        device: "belt".into(),
                    },
                ),
            ],
        });
        let tl = scene
            .simulate_sequences(&["station", "transfer"], &RolloutOptions::default())
            .unwrap();
        // Station: 1.0 s of ramp. Transfer: gated on the weld, then 0.5 s
        // of indexing. Serial would be impossible to even express without
        // weaving them by hand; parallel comes out at the sum only because
        // the transfer *waits* — and the step spans prove which is which.
        let spans: std::collections::HashMap<String, (f64, f64)> = tl
            .step_spans
            .iter()
            .map(|s| (s.name.clone(), (s.start, s.end)))
            .collect();
        assert!(spans.contains_key("station/work"), "{spans:?}");
        assert!(spans.contains_key("transfer/index"), "{spans:?}");
        let work_end = spans["station/work"].1;
        let index_start = spans["transfer/index"].0;
        assert!(
            (index_start - work_end).abs() < 0.011,
            "transfer moved at {index_start}, station finished at {work_end}"
        );
        assert!(
            (tl.duration - 1.51).abs() < 0.02,
            "duration {}",
            tl.duration
        );

        // And with no dependency, the two programs overlap: total is the
        // max of the two, not the sum.
        let mut scene = belt_scene(0.4);
        scene.upsert_sequence(Sequence {
            name: "station".into(),
            steps: vec![step("work", vec![ramp("j", 0.8, 1.0)], Condition::Done)],
        });
        scene.upsert_sequence(Sequence {
            name: "transfer".into(),
            steps: vec![step(
                "index",
                vec![Action::Device {
                    device: "belt".into(),
                    command: DeviceCommand::Advance(0.2),
                }],
                Condition::DeviceDone {
                    device: "belt".into(),
                },
            )],
        });
        let tl = scene
            .simulate_sequences(&["station", "transfer"], &RolloutOptions::default())
            .unwrap();
        assert!(
            (tl.duration - 1.0).abs() < 0.02,
            "expected max(1.0, 0.5), got {}",
            tl.duration
        );
    }

    #[test]
    fn rebaking_parallel_programs_is_bit_identical() {
        let bake = || {
            let mut scene = belt_scene(0.4);
            scene.define_signal("welded", false);
            scene.upsert_sequence(Sequence {
                name: "a".into(),
                steps: vec![
                    step("work", vec![ramp("j", 0.7, 0.6)], Condition::Done),
                    step(
                        "flag",
                        vec![Action::Set {
                            signal: "welded".into(),
                            value: true,
                        }],
                        Condition::Immediately,
                    ),
                ],
            });
            scene.upsert_sequence(Sequence {
                name: "b".into(),
                steps: vec![
                    step(
                        "wait",
                        vec![],
                        Condition::Signal {
                            name: "welded".into(),
                            value: true,
                        },
                    ),
                    step(
                        "index",
                        vec![Action::Device {
                            device: "belt".into(),
                            command: DeviceCommand::Advance(0.3),
                        }],
                        Condition::DeviceDone {
                            device: "belt".into(),
                        },
                    ),
                ],
            });
            scene
                .simulate_sequences(&["a", "b"], &RolloutOptions::default())
                .unwrap()
        };
        let (one, two) = (bake(), bake());
        assert_eq!(one.duration, two.duration);
        assert_eq!(
            one.robots[0].trajectory.positions,
            two.robots[0].trajectory.positions
        );
        let x = |tl: &SequenceTimeline| {
            tl.signals
                .iter()
                .map(|s| s.edges.clone())
                .collect::<Vec<_>>()
        };
        assert_eq!(x(&one), x(&two));
    }

    #[test]
    fn ownership_is_validated_up_front() {
        // Robot commanded by two programs.
        let mut scene = sample_scene();
        joint_motion(&mut scene, "go", 0.8);
        scene.upsert_sequence(Sequence {
            name: "a".into(),
            steps: vec![step("x", vec![ramp("j", 0.5, 0.2)], Condition::Done)],
        });
        scene.upsert_sequence(Sequence {
            name: "b".into(),
            steps: vec![step(
                "y",
                vec![Action::StartMotion {
                    motion: "go".into(),
                }],
                Condition::Done,
            )],
        });
        let err = scene
            .simulate_sequences(&["a", "b"], &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(
            err.contains("robot `r` is commanded by both `a` and `b`"),
            "{err}"
        );

        // Signal written by two programs.
        let mut scene = sample_scene();
        scene.define_signal("flag", false);
        let writer = |name: &str| Sequence {
            name: name.into(),
            steps: vec![step(
                "w",
                vec![Action::Set {
                    signal: "flag".into(),
                    value: true,
                }],
                Condition::Immediately,
            )],
        };
        scene.upsert_sequence(writer("a"));
        scene.upsert_sequence(writer("b"));
        let err = scene
            .simulate_sequences(&["a", "b"], &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("signal `flag` is commanded by both"), "{err}");

        // Device driven by two programs.
        let mut scene = belt_scene(0.4);
        let starter = |name: &str| Sequence {
            name: name.into(),
            steps: vec![step(
                "s",
                vec![Action::Device {
                    device: "belt".into(),
                    command: DeviceCommand::Start,
                }],
                Condition::Immediately,
            )],
        };
        scene.upsert_sequence(starter("a"));
        scene.upsert_sequence(starter("b"));
        let err = scene
            .simulate_sequences(&["a", "b"], &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("device `belt` is commanded by both"), "{err}");

        // The same list twice is its own mistake.
        let mut scene = sample_scene();
        scene.upsert_sequence(Sequence {
            name: "a".into(),
            steps: vec![step("x", vec![], Condition::Immediately)],
        });
        let err = scene
            .simulate_sequences(&["a", "a"], &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("listed twice"), "{err}");
    }

    #[test]
    fn timeout_names_every_stuck_program() {
        let mut scene = sample_scene();
        scene.define_signal("never", false);
        scene.define_signal("also_never", false);
        let stuck = |name: &str, signal: &str| Sequence {
            name: name.into(),
            steps: vec![step(
                "gate",
                vec![],
                Condition::Signal {
                    name: signal.into(),
                    value: true,
                },
            )],
        };
        scene.upsert_sequence(stuck("st1", "never"));
        scene.upsert_sequence(stuck("st2", "also_never"));
        let err = scene
            .simulate_sequences(
                &["st1", "st2"],
                &RolloutOptions {
                    max_duration: 0.5,
                    ..Default::default()
                },
            )
            .unwrap_err()
            .to_string();
        assert!(
            err.contains("st1/gate") && err.contains("st2/gate"),
            "{err}"
        );

        // A finished program is not on the list.
        let mut scene = sample_scene();
        scene.define_signal("never", false);
        scene.upsert_sequence(Sequence {
            name: "quick".into(),
            steps: vec![step("done", vec![], Condition::Immediately)],
        });
        scene.upsert_sequence(stuck("st2", "never"));
        let err = scene
            .simulate_sequences(
                &["quick", "st2"],
                &RolloutOptions {
                    max_duration: 0.5,
                    ..Default::default()
                },
            )
            .unwrap_err()
            .to_string();
        assert!(err.contains("st2/gate") && !err.contains("quick"), "{err}");
    }
}

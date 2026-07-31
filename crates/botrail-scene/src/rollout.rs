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

use crate::seq::{Action, Condition, DeviceCommand, DeviceKind, SensorKind, SensorWatch, Sequence};
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
    #[error(
        "step {step} (`{name}`): >{limit} instantaneous steps in one scan tick (immediate loop?)"
    )]
    ImmediateLoop {
        step: usize,
        name: String,
        limit: usize,
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
    /// Rigidly attached: `world = link_pose(t) ∘ offset`.
    Follow {
        t0: f64,
        t1: f64,
        link: usize,
        offset: Isometry3<f64>,
    },
    /// Conveyed at constant velocity from `from` (rotation unchanged).
    Linear {
        t0: f64,
        t1: f64,
        from: Isometry3<f64>,
        velocity: nalgebra::Vector3<f64>,
    },
}

impl TrackSpan {
    fn end_mut(&mut self) -> &mut f64 {
        match self {
            TrackSpan::Hold { t1, .. }
            | TrackSpan::Follow { t1, .. }
            | TrackSpan::Linear { t1, .. } => t1,
        }
    }

    pub fn range(&self) -> (f64, f64) {
        match self {
            TrackSpan::Hold { t0, t1, .. }
            | TrackSpan::Follow { t0, t1, .. }
            | TrackSpan::Linear { t0, t1, .. } => (*t0, *t1),
        }
    }
}

#[derive(Debug, Clone)]
pub struct ObjectTrack {
    /// Obstacle name.
    pub name: String,
    pub spans: Vec<TrackSpan>,
}

/// The baked result of a sequence rollout — what playback, USD export, and
/// the timing chart consume. `duration` is the cycle time.
#[derive(Debug, Clone)]
pub struct SequenceTimeline {
    pub duration: f64,
    /// Whole-sequence joint track (holds still during waits).
    pub robot: JointTrajectory,
    /// Objects that were grasped at some point (everything else is static).
    pub objects: Vec<ObjectTrack>,
    pub signals: Vec<BoolTrack>,
    pub step_spans: Vec<StepSpan>,
}

impl SequenceTimeline {
    /// World pose of a tracked object at `t`; `link_poses` must be the FK
    /// world poses at the same instant.
    pub fn object_pose(
        track: &ObjectTrack,
        link_poses: &[Isometry3<f64>],
        t: f64,
    ) -> Option<Isometry3<f64>> {
        let span = track
            .spans
            .iter()
            .find(|s| {
                let (t0, t1) = s.range();
                t >= t0 - 1e-9 && t <= t1 + 1e-9
            })
            .or(track.spans.last())?;
        Some(match span {
            TrackSpan::Hold { pose, .. } => *pose,
            TrackSpan::Follow { link, offset, .. } => link_poses[*link] * offset,
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
        })
    }
}

impl Scene {
    /// Runs the scan loop for `name` against a clone of this scene (the
    /// live scene is untouched). `limits` time-parameterizes the planned
    /// motions, as in [`Scene::plan_motion`].
    pub fn simulate_sequence(
        &self,
        name: &str,
        options: &RolloutOptions,
        limits: &botrail_traj::Limits,
    ) -> Result<SequenceTimeline, SeqError> {
        let sequence = self
            .sequence(name)
            .ok_or_else(|| SeqError::UnknownSequence(name.to_string()))?
            .clone();
        self.validate_sequence(&sequence)
            .map_err(|(step, message)| SeqError::Validation { step, message })?;
        Rollout::new(self.clone(), sequence, options.clone(), limits.clone()).run()
    }
}

struct Rollout {
    world: Scene,
    sequence: Sequence,
    options: RolloutOptions,
    limits: botrail_traj::Limits,

    t: f64,
    step: usize,
    step_entered_at: f64,
    /// Absolute end time of the motion/ramp started by the active step.
    motion_end: Option<f64>,
    /// The in-flight motion/ramp, for per-tick joint sampling.
    active: Option<ActiveMove>,
    /// Commanded joints (what the robot actually does).
    q: Vec<f64>,
    /// Joints as the motion/ramp asks for them, before any tracking offset.
    /// Equal to `q` unless a track is active.
    q_nom: Vec<f64>,
    /// Previous tick's nominal, so a tracked solve can warm-start from the
    /// previous *command* plus the nominal increment.
    q_nom_prev: Vec<f64>,
    /// Conveyor tracking: the latched part and the offset it has built up.
    tracking: Option<TrackLatch>,
    sensors: Vec<SensorRuntime>,
    devices: Vec<DeviceRuntime>,

    // Accumulating outputs.
    times: Vec<f64>,
    positions: Vec<Vec<f64>>,
    velocities: Vec<Vec<f64>>,
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
}

/// Per-tick tracking solve: warm-started from the nominal configuration,
/// so it only has to absorb one scan period of part motion.
const TRACK_IK: botrail_kin::IkOptions = botrail_kin::IkOptions {
    mode: botrail_kin::IkMode::Pose,
    max_iters: 100,
    tol_pos: 1e-7,
    tol_rot: 1e-6,
    damping: 0.05,
    orientation_weight: 0.5,
    max_step: 0.5,
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
}

struct SensorRuntime {
    collider: ObstacleCollider,
    pose: Isometry3<f64>,
    watch: SensorWatch,
    /// Index of this sensor's lane in the signal tracks.
    lane: usize,
}

enum DeviceRuntime {
    Conveyor {
        name: String,
        zone_pose: Isometry3<f64>,
        zone_half: Vector3<f64>,
        velocity: Vector3<f64>,
        running: bool,
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
}

impl Rollout {
    fn new(
        world: Scene,
        sequence: Sequence,
        options: RolloutOptions,
        limits: botrail_traj::Limits,
    ) -> Self {
        let q = world.joint_positions().to_vec();
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
                }
            })
            .collect();
        // Objects grasped before the sequence starts follow from t = 0.
        let objects = world
            .attachments()
            .iter()
            .map(|a| ObjectTrack {
                name: a.object.clone(),
                spans: vec![TrackSpan::Follow {
                    t0: 0.0,
                    t1: 0.0,
                    link: a.link,
                    offset: a.grasp,
                }],
            })
            .collect();
        Rollout {
            world,
            sequence,
            options,
            limits,
            t: 0.0,
            step: 0,
            step_entered_at: 0.0,
            motion_end: None,
            active: None,
            tracking: None,
            sensors,
            devices,
            times: vec![0.0],
            positions: vec![q.clone()],
            velocities: Vec::new(),
            q_nom: q.clone(),
            q_nom_prev: q.clone(),
            q,
            objects,
            signals,
            step_spans: Vec::new(),
        }
    }

    fn step_name(&self, index: usize) -> String {
        self.sequence
            .steps
            .get(index)
            .map(|s| s.name.clone())
            .unwrap_or_default()
    }

    fn run(mut self) -> Result<SequenceTimeline, SeqError> {
        self.velocities.push(vec![0.0; self.q.len()]);
        self.update_sensors();
        self.enter_step()?;
        // Instantaneous steps may complete before the first tick.
        self.advance_through_ready_steps()?;

        let mut tick = 0u64;
        while self.step < self.sequence.steps.len() {
            tick += 1;
            self.t = tick as f64 * self.options.dt;
            if self.t > self.options.max_duration {
                return Err(SeqError::Timeout {
                    step: self.step,
                    name: self.step_name(self.step),
                    limit: self.options.max_duration,
                });
            }
            // PLC scan: outputs advance the world through this tick, then
            // inputs are read, then transitions fire.
            self.advance_world()?;
            self.update_sensors();
            self.advance_through_ready_steps()?;
        }
        Ok(self.finish())
    }

    /// Advances the robot joints and every device by one scan period.
    fn advance_world(&mut self) -> Result<(), SeqError> {
        let t = self.t;
        let dt = self.options.dt;
        // Joints follow the in-flight motion/ramp (attached obstacles are
        // re-synced by set_joint_positions).
        if let Some(active) = &self.active {
            self.q_nom = active.sample(t);
            // Under a track the commanded joints are solved in
            // `follow_tracked_part`, once this tick's part motion is known.
            if self.tracking.is_none() {
                self.q = self.q_nom.clone();
                self.world
                    .set_joint_positions(self.q.clone())
                    .expect("sampled q has scene DOF");
            }
            if self.motion_end.map(|end| t >= end - 1e-9).unwrap_or(false) {
                self.active = None;
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
        let mut lane_updates: Vec<(usize, bool)> = Vec::new();
        for device in &mut self.devices {
            match device {
                DeviceRuntime::Conveyor {
                    zone_pose,
                    zone_half,
                    velocity,
                    running,
                    ..
                } => {
                    if !*running || velocity.norm() < 1e-12 {
                        continue;
                    }
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
                            pose.translation.vector += *velocity * dt;
                            moved.push((obstacle.name.clone(), pose, *velocity));
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
        // Objects that stopped riding (zone exit / device stop) settle into
        // a hold at their current pose.
        let moved_names: Vec<&String> = moved.iter().map(|(n, _, _)| n).collect();
        let settled: Vec<(String, Isometry3<f64>)> = self
            .objects
            .iter()
            .filter(|track| {
                !moved_names.iter().any(|n| **n == track.name)
                    && matches!(track.spans.last(), Some(TrackSpan::Linear { t1, .. }) if *t1 < t)
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
                Some(TrackSpan::Linear { t1, .. }) => *t1,
                _ => t,
            };
            let track = self.object_track_at(&name, pose, t_stop);
            track.spans.push(TrackSpan::Hold {
                t0: t_stop,
                t1: t_stop,
                pose,
            });
        }
        self.follow_tracked_part()?;
        Ok(())
    }

    /// Conveyor tracking: re-solve the arm so this tick's commanded pose is
    /// the nominal one carried by the part's motion since the latch. Runs
    /// after the devices have moved the world, so the robot sees the part
    /// where it is *now*.
    fn follow_tracked_part(&mut self) -> Result<(), SeqError> {
        let Some(latch) = &self.tracking else {
            return Ok(());
        };
        let (object, link, origin, frozen) =
            (latch.object.clone(), latch.link, latch.origin, latch.frozen);
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
                    step: self.step,
                    name: self.step_name(self.step),
                    message: format!("tracked obstacle `{object}` disappeared"),
                })?;
            let offset = pose * origin.inverse();
            if let Some(latch) = &mut self.tracking {
                latch.offset = offset;
            }
            offset
        };

        let nominal = self.world.fk(&self.q_nom).expect("q_nom has scene DOF")[link];
        let target = offset * nominal;
        // Warm start from what the robot did last tick plus this tick's
        // nominal increment: the solve then only absorbs one scan period of
        // part motion (and joints the offset cannot touch — the gripper —
        // follow the nominal exactly).
        let seed: Vec<f64> = self
            .q
            .iter()
            .zip(&self.q_nom)
            .zip(&self.q_nom_prev)
            .map(|((commanded, nominal), previous)| commanded + (nominal - previous))
            .collect();
        let result = self
            .world
            .solve_ik_world(link, &target, &seed, &TRACK_IK)
            .expect("seed has scene DOF");
        if !result.converged {
            return Err(SeqError::Action {
                step: self.step,
                name: self.step_name(self.step),
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
        let previous = self.q.clone();
        self.q = result.q;
        self.q_nom_prev = self.q_nom.clone();
        self.world
            .set_joint_positions(self.q.clone())
            .expect("solved q has scene DOF");
        // The move's own waypoints know nothing about the offset, so a
        // tracked tick bakes itself (velocities by difference).
        let dt = self.options.dt;
        let velocity = self
            .q
            .iter()
            .zip(&previous)
            .map(|(now, before)| (now - before) / dt)
            .collect();
        let (t, q) = (self.t, self.q.clone());
        self.append_waypoint(t, q, velocity);
        Ok(())
    }

    /// Latches onto `object`: from here the nominal poses ride its motion.
    fn latch_track(&mut self, object: &str, link: Option<&str>) -> Result<(), SeqError> {
        let err = |message: String| SeqError::Action {
            step: self.step,
            name: self.step_name(self.step),
            message,
        };
        let link = match link {
            Some(name) => self
                .world
                .robot
                .link_index(name)
                .ok_or_else(|| err(format!("unknown link `{name}`")))?,
            // The wrist, not the fingertip: a pose says nothing about the
            // grip, so the solver must not be able to spend it.
            None => self.world.robot.tool_mount_link(),
        };
        let origin = self
            .world
            .obstacles()
            .iter()
            .find(|o| o.name == object)
            .map(|o| o.pose)
            .ok_or_else(|| err(format!("unknown obstacle `{object}`")))?;
        self.q_nom = self.q.clone();
        self.q_nom_prev = self.q.clone();
        self.tracking = Some(TrackLatch {
            object: object.to_string(),
            link,
            origin,
            offset: Isometry3::identity(),
            frozen: false,
        });
        Ok(())
    }

    /// Drops the track; the robot keeps the configuration it is in, so the
    /// nominal frame is re-based onto it (releasing never moves the robot).
    fn release_track(&mut self) {
        self.tracking = None;
        self.q_nom = self.q.clone();
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

    /// Evaluates every pseudo-sensor at the current world state and records
    /// edges on its input lane.
    fn update_sensors(&mut self) {
        if self.sensors.is_empty() {
            return;
        }
        let needs_robot = self
            .sensors
            .iter()
            .any(|s| matches!(s.watch, SensorWatch::Robot | SensorWatch::All));
        let link_poses = needs_robot.then(|| self.world.link_poses());
        let t = self.t;
        let mut edges = Vec::new();
        for sensor in &self.sensors {
            let mut value = false;
            let watch_objects: Option<&[String]> = match &sensor.watch {
                SensorWatch::Objects(names) => Some(names),
                SensorWatch::AllObjects | SensorWatch::All => None,
                SensorWatch::Robot => Some(&[]),
            };
            if !matches!(sensor.watch, SensorWatch::Robot) {
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
                    if sensor
                        .collider
                        .intersects(&sensor.pose, collider, &obstacle.pose)
                    {
                        value = true;
                        break;
                    }
                }
            }
            if !value {
                if let (Some(poses), SensorWatch::Robot | SensorWatch::All) =
                    (&link_poses, &sensor.watch)
                {
                    value = botrail_collide::robot_intersects(
                        &self.world.robot_collider,
                        poses,
                        &sensor.collider,
                        &sensor.pose,
                    );
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

    /// Fires transitions that hold at the current time, chaining through
    /// instantaneous steps (bounded per tick).
    fn advance_through_ready_steps(&mut self) -> Result<(), SeqError> {
        let mut chain = 0usize;
        while self.step < self.sequence.steps.len() {
            let condition = self.sequence.steps[self.step].transition.clone();
            if !self.condition_holds(&condition) {
                return Ok(());
            }
            self.exit_step();
            self.step += 1;
            if self.step == self.sequence.steps.len() {
                return Ok(());
            }
            chain += 1;
            if chain > self.options.immediate_chain_limit {
                return Err(SeqError::ImmediateLoop {
                    step: self.step,
                    name: self.step_name(self.step),
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
            Condition::Done => self
                .motion_end
                .map(|end| self.t >= end - 1e-9)
                .unwrap_or(true),
            Condition::Elapsed { seconds } => self.t - self.step_entered_at >= seconds - 1e-9,
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
                DeviceRuntime::Conveyor { .. } => false,
            }),
            Condition::All(cs) => cs.iter().all(|c| self.condition_holds(c)),
            Condition::Any(cs) => cs.iter().any(|c| self.condition_holds(c)),
        }
    }

    fn enter_step(&mut self) -> Result<(), SeqError> {
        self.step_entered_at = self.t;
        self.motion_end = None;
        self.step_spans.push(StepSpan {
            name: self.step_name(self.step),
            start: self.t,
            end: self.t,
        });
        for action in self.sequence.steps[self.step].actions.clone() {
            self.fire(&action)?;
        }
        Ok(())
    }

    fn exit_step(&mut self) {
        if let Some(span) = self.step_spans.last_mut() {
            span.end = self.t;
        }
        // Hold the last configuration up to the transition instant, so the
        // baked track stays exact through waits.
        self.append_waypoint(self.t, self.q.clone(), vec![0.0; self.q.len()]);
    }

    fn fire(&mut self, action: &Action) -> Result<(), SeqError> {
        let step_index = self.step;
        let step_name = self.step_name(self.step);
        let err = move |message: String| SeqError::Action {
            step: step_index,
            name: step_name.clone(),
            message,
        };
        match action {
            Action::StartMotion { motion } => {
                // Plan against the world as it stands *now*: current q,
                // moved obstacles, live grasps.
                self.world
                    .set_joint_positions(self.q.clone())
                    .map_err(|e| err(e.to_string()))?;
                let planned = self
                    .world
                    .plan_motion(motion, &self.options.plan, &self.limits)
                    .map_err(|e| SeqError::PlanFailed {
                        step: self.step,
                        name: self.step_name(self.step),
                        message: e.to_string(),
                    })?;
                let traj = planned.trajectory;
                for i in 0..traj.times.len() {
                    self.append_waypoint(
                        self.t + traj.times[i],
                        traj.positions[i].clone(),
                        traj.velocities[i].clone(),
                    );
                }
                self.motion_end = Some(self.t + traj.duration());
                // Joints follow the trajectory tick by tick (advance_world),
                // so mid-motion sensors see the true robot state.
                self.active = Some(ActiveMove::Traj {
                    start: self.t,
                    traj,
                });
            }
            Action::StartRamp { targets, duration } => {
                let mut goal = self.q_nom.clone();
                for (joint, value) in targets {
                    let ji = self
                        .world
                        .robot
                        .joint_index(joint)
                        .ok_or_else(|| err(format!("unknown joint `{joint}`")))?;
                    let qi = self.world.robot.joints[ji]
                        .q_index
                        .ok_or_else(|| err(format!("joint `{joint}` is not actuated")))?;
                    goal[qi] = *value;
                }
                // Two rest-to-rest waypoints: cubic Hermite eases in/out.
                // A tracked ramp cannot bake ahead — its poses are carried
                // by a part that has not moved yet — so it bakes per tick.
                if self.tracking.is_none() {
                    self.append_waypoint(self.t + duration, goal.clone(), vec![0.0; goal.len()]);
                }
                self.motion_end = Some(self.t + duration);
                self.active = Some(ActiveMove::Ramp {
                    start: self.t,
                    duration: *duration,
                    from: self.q_nom.clone(),
                    to: goal,
                });
            }
            Action::Attach {
                object,
                link,
                touch_links,
            } => {
                self.world
                    .set_joint_positions(self.q.clone())
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
                    .attach_obstacle(object, link.as_deref(), touch_links.as_deref())
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
                    link: attachment.link,
                    offset: attachment.grasp,
                });
                // Grasping the tracked part ends the chase: it moves with
                // the robot now, so the offset it had at the grasp stands
                // (which is what keeps the lift straight).
                if let Some(latch) = &mut self.tracking {
                    if &latch.object == object {
                        latch.frozen = true;
                    }
                }
            }
            Action::Detach { object } => {
                self.world
                    .set_joint_positions(self.q.clone())
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
            Action::Track { object, link } => {
                self.world
                    .set_joint_positions(self.q.clone())
                    .map_err(|e| err(e.to_string()))?;
                self.latch_track(object, link.as_deref())?;
            }
            Action::Untrack => self.release_track(),
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
                    DeviceRuntime::Conveyor { name, .. } | DeviceRuntime::Axis { name, .. } => {
                        name == device
                    }
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
                    (DeviceRuntime::Conveyor { velocity, .. }, DeviceCommand::SetSpeed(speed)) => {
                        let norm = velocity.norm();
                        if norm > 1e-12 {
                            *velocity = *velocity / norm * *speed;
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

    fn append_waypoint(&mut self, t: f64, q: Vec<f64>, v: Vec<f64>) {
        let last = *self.times.last().expect("seeded with t = 0");
        if t <= last + 1e-9 {
            return;
        }
        self.times.push(t);
        self.positions.push(q);
        self.velocities.push(v);
    }

    fn finish(mut self) -> SequenceTimeline {
        let duration = self.t;
        self.append_waypoint(duration, self.q.clone(), vec![0.0; self.q.len()]);
        for track in &mut self.objects {
            if let Some(open) = track.spans.last_mut() {
                *open.end_mut() = duration;
            }
        }
        SequenceTimeline {
            duration,
            robot: JointTrajectory {
                times: self.times,
                positions: self.positions,
                velocities: self.velocities,
            },
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

    pub(crate) fn limits() -> botrail_traj::Limits {
        botrail_traj::Limits::uniform(1, 1.0, 2.0)
    }

    fn joint_motion(scene: &mut Scene, name: &str, goal: f64) {
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
        let tl = scene
            .simulate_sequence("cycle", &options, &limits())
            .unwrap();

        assert_eq!(tl.step_spans.len(), 3);
        let run = &tl.step_spans[0];
        let wait = &tl.step_spans[1];
        let ret = &tl.step_spans[2];
        // Step boundaries quantize up to the scan period.
        assert!(run.start == 0.0 && run.end > 0.2);
        assert!((tl.robot.sample(run.end)[0] - 0.8).abs() < 1e-9);
        let wait_len = wait.end - wait.start;
        assert!(
            (0.5 - 1e-9..=0.5 + options.dt + 1e-9).contains(&wait_len),
            "wait_len = {wait_len}"
        );
        // The robot holds still through the wait.
        assert!((tl.robot.sample(wait.start + 0.25)[0] - 0.8).abs() < 1e-9);
        // The return motion starts where the previous ended and comes home.
        assert!((tl.robot.sample(tl.duration)[0]).abs() < 1e-9);
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
            .simulate_sequence("s", &RolloutOptions::default(), &limits())
            .unwrap();

        // Cubic rest-to-rest ramp: exact midpoint halfway through.
        assert!((tl.robot.sample(0.15)[0] - 0.3).abs() < 1e-9);
        assert!((tl.robot.sample(0.3)[0] - 0.6).abs() < 1e-9);
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
            .simulate_sequence("pick", &RolloutOptions::default(), &limits())
            .unwrap();

        assert_eq!(tl.objects.len(), 1);
        let track = &tl.objects[0];
        assert_eq!(track.name, "box");
        assert_eq!(track.spans.len(), 2);
        let TrackSpan::Follow {
            t0,
            t1,
            link,
            offset,
        } = &track.spans[0]
        else {
            panic!("expected follow span, got {track:?}");
        };
        assert!(*t0 == 0.0 && *t1 > 0.2 && *link == 1);
        // Mid-motion the box rides FK ∘ grasp.
        let t_mid = (t0 + t1) / 2.0;
        let q = tl.robot.sample(t_mid);
        let poses = scene.fk(&q).unwrap();
        let expected = poses[*link] * offset;
        let via_track = SequenceTimeline::object_pose(track, &poses, t_mid).unwrap();
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
            .simulate_sequence("s", &RolloutOptions::default(), &limits())
            .unwrap();
        // Before the grasp the object rests at its scene pose (the track
        // tiles [0, duration] with a leading Hold).
        let track = &tl.objects[0];
        let TrackSpan::Hold { t0, t1, pose } = &track.spans[0] else {
            panic!("expected leading hold, got {track:?}");
        };
        assert!(*t0 == 0.0 && (*t1 - 0.4).abs() < 0.011);
        assert!((pose.translation.vector - Vector3::new(0.1, 0.0, 0.5)).norm() < 1e-12);
        let poses = scene.fk(&tl.robot.sample(0.2)).unwrap();
        let early = SequenceTimeline::object_pose(track, &poses, 0.2).unwrap();
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
                .simulate_sequence("bad", &RolloutOptions::default(), &limits())
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
            scene.simulate_sequence("stuck", &options, &limits()),
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
            scene.simulate_sequence("chain", &tight, &limits()),
            Err(SeqError::ImmediateLoop { .. })
        ));
        // Under the limit the chain is fine (zero-duration sequence).
        let ok = scene
            .simulate_sequence("chain", &RolloutOptions::default(), &limits())
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
            .simulate_sequence("s", &RolloutOptions::default(), &limits())
            .unwrap();
        let b = scene
            .simulate_sequence("s", &RolloutOptions::default(), &limits())
            .unwrap();
        assert_eq!(a.duration, b.duration);
        assert_eq!(a.robot.times, b.robot.times);
        assert_eq!(a.robot.positions, b.robot.positions);
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
            .simulate_sequence("s", &RolloutOptions::default(), &limits())
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
        let tl = scene
            .simulate_sequence("feed", &options, &limits())
            .unwrap();

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
        let poses = scene.fk(&tl.robot.sample(tl.duration)).unwrap();
        let end_pose = SequenceTimeline::object_pose(track, &poses, tl.duration).unwrap();
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
        let tl = scene
            .simulate_sequence("open", &options, &limits())
            .unwrap();

        // 0.3 m at 0.5 m/s = 0.6 s (+ scan quantization).
        let raise_end = tl.step_spans[0].end;
        assert!((raise_end - 0.6).abs() <= options.dt + 1e-9, "{raise_end}");
        // The door lands exactly 0.3 above its rest height.
        let track = &tl.objects[0];
        let poses = scene.fk(&tl.robot.sample(tl.duration)).unwrap();
        let end = SequenceTimeline::object_pose(track, &poses, tl.duration).unwrap();
        assert!(
            (end.translation.z - 0.5).abs() < 1e-12,
            "{}",
            end.translation.z
        );
        // Mid-travel sampling is linear (exact at 0.3 s: half way).
        let mid = SequenceTimeline::object_pose(track, &poses, 0.3).unwrap();
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
            .simulate_sequence("s", &RolloutOptions::default(), &limits())
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
        scene.upsert_sensor(Sensor {
            name: "eye".into(),
            kind: SensorKind::Zone {
                pose: iso(0.0, 0.0, 0.0),
                size: Vector3::new(0.1, 0.1, 0.1),
            },
            watch: SensorWatch::AllObjects,
        });
        let check = |scene: &Scene, steps: Vec<Step>, needle: &str| {
            let mut s = scene.clone();
            s.upsert_sequence(Sequence {
                name: "bad".into(),
                steps,
            });
            let err = s
                .simulate_sequence("bad", &RolloutOptions::default(), &limits())
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
        // device_done needs an axis.
        check(
            &scene,
            vec![step(
                "x",
                vec![],
                Condition::DeviceDone {
                    device: "conv".into(),
                },
            )],
            "conveyor",
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

    fn limits() -> botrail_traj::Limits {
        botrail_traj::Limits::uniform(3, 1.0, 2.0)
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
        let link = scene.robot.link_index("tool").unwrap();
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
                        object: "part".into(),
                        link: Some("tool".into()),
                    }],
                    Condition::Immediately,
                ),
                step(
                    "descend",
                    vec![Action::StartRamp {
                        // taught: straight down onto the part at x = 0
                        targets: vec![("jz".into(), 0.05)],
                        duration: 0.5,
                    }],
                    Condition::Done,
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("pick", &RolloutOptions::default(), &limits())
            .unwrap();
        assert!((tl.duration - 0.5).abs() < 1e-9, "{}", tl.duration);

        // The part has travelled 0.2 * 0.5 = 0.1 m; so has the tool, which
        // also completed the taught 0.35 m descent.
        let end = tool_pose(&scene, &tl.robot.sample(tl.duration));
        assert!((end.translation.x - 0.1).abs() < 1e-4, "{}", end.translation.x);
        assert!((end.translation.z - 0.05).abs() < 1e-4, "{}", end.translation.z);
        // Mid-ramp the tool sits over the part throughout, not behind it.
        for i in 0..=10 {
            let t = tl.duration * f64::from(i) / 10.0;
            let pose = tool_pose(&scene, &tl.robot.sample(t));
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
                        object: "part".into(),
                        link: Some("tool".into()),
                    }],
                    Condition::Immediately,
                ),
                step(
                    "descend",
                    vec![Action::StartRamp {
                        targets: vec![("jz".into(), 0.05)],
                        duration: 0.5,
                    }],
                    Condition::Done,
                ),
                step(
                    "grasp",
                    vec![Action::Attach {
                        object: "part".into(),
                        link: Some("tool".into()),
                        touch_links: Some(vec!["tool".into()]),
                    }],
                    Condition::Immediately,
                ),
                step(
                    "lift",
                    vec![Action::StartRamp {
                        targets: vec![("jz".into(), 0.4)],
                        duration: 0.5,
                    }],
                    Condition::Done,
                ),
                step("release", vec![Action::Untrack], Condition::Immediately),
            ],
        });
        let tl = scene
            .simulate_sequence("pick", &RolloutOptions::default(), &limits())
            .unwrap();
        let track = tl
            .objects
            .iter()
            .find(|o| o.name == "part")
            .expect("the grasped part is tracked");
        let poses = |t: f64| {
            let q = tl.robot.sample(t);
            let link_poses = scene.fk(&q).unwrap();
            (
                tool_pose(&scene, &q),
                SequenceTimeline::object_pose(track, &link_poses, t).unwrap(),
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
                        object: "part".into(),
                        link: Some("tool".into()),
                    }],
                    Condition::Immediately,
                ),
                step("follow", vec![], Condition::Elapsed { seconds: 0.5 }),
                step("release", vec![Action::Untrack], Condition::Immediately),
                step("settle", vec![], Condition::Elapsed { seconds: 0.2 }),
            ],
        });
        let tl = scene
            .simulate_sequence("pick", &RolloutOptions::default(), &limits())
            .unwrap();
        let at_release = tool_pose(&scene, &tl.robot.sample(0.5));
        let at_end = tool_pose(&scene, &tl.robot.sample(tl.duration));
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
                        object: "part".into(),
                        link: Some("left".into()),
                    }],
                    Condition::Immediately,
                ),
                step(
                    "close",
                    vec![Action::StartRamp {
                        targets: vec![("finger_left".into(), 0.02)],
                        duration: 0.4,
                    }],
                    Condition::Done,
                ),
            ],
        });
        let err = scene
            .simulate_sequence(
                "s",
                &RolloutOptions::default(),
                &botrail_traj::Limits::uniform(3, 1.0, 2.0),
            )
            .expect_err("the gripper joint moves the servoed link")
            .to_string();
        assert!(err.contains("fights the track"), "{err}");
        assert!(err.contains("wrist"), "{err}");
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
                .simulate_sequence("s", &RolloutOptions::default(), &limits())
                .expect_err("expected `{needle}`")
                .to_string();
            assert!(err.contains(needle), "expected `{needle}` in `{err}`");
        };
        let track = || Action::Track {
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
            vec![step("loose", vec![Action::Untrack], Condition::Immediately)],
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
                    object: "nope".into(),
                    link: None,
                }],
                Condition::Immediately,
            )],
            "unknown obstacle",
        );
    }
}

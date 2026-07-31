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

use crate::seq::{Action, Condition, Sequence};
use crate::Scene;

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
}

impl TrackSpan {
    fn end_mut(&mut self) -> &mut f64 {
        match self {
            TrackSpan::Hold { t1, .. } | TrackSpan::Follow { t1, .. } => t1,
        }
    }

    pub fn range(&self) -> (f64, f64) {
        match self {
            TrackSpan::Hold { t0, t1, .. } | TrackSpan::Follow { t0, t1, .. } => (*t0, *t1),
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
    q: Vec<f64>,

    // Accumulating outputs.
    times: Vec<f64>,
    positions: Vec<Vec<f64>>,
    velocities: Vec<Vec<f64>>,
    objects: Vec<ObjectTrack>,
    signals: Vec<BoolTrack>,
    step_spans: Vec<StepSpan>,
}

impl Rollout {
    fn new(
        world: Scene,
        sequence: Sequence,
        options: RolloutOptions,
        limits: botrail_traj::Limits,
    ) -> Self {
        let q = world.joint_positions().to_vec();
        // Signals start at their declared initial values.
        let signals = world
            .signals()
            .iter()
            .map(|s| BoolTrack {
                name: s.name.clone(),
                edges: vec![(0.0, s.initial)],
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
            times: vec![0.0],
            positions: vec![q.clone()],
            velocities: Vec::new(),
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
            self.advance_through_ready_steps()?;
        }
        Ok(self.finish())
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
                let traj = &planned.trajectory;
                for i in 0..traj.times.len() {
                    self.append_waypoint(
                        self.t + traj.times[i],
                        traj.positions[i].clone(),
                        traj.velocities[i].clone(),
                    );
                }
                self.q = traj
                    .positions
                    .last()
                    .cloned()
                    .unwrap_or_else(|| self.q.clone());
                self.motion_end = Some(self.t + traj.duration());
            }
            Action::StartRamp { targets, duration } => {
                let mut goal = self.q.clone();
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
                self.append_waypoint(self.t + duration, goal.clone(), vec![0.0; goal.len()]);
                self.q = goal;
                self.motion_end = Some(self.t + duration);
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
                let track = self.object_track(object);
                if track.spans.is_empty() && t > 0.0 {
                    track.spans.push(TrackSpan::Hold {
                        t0: 0.0,
                        t1: t,
                        pose: rest_pose.expect("attach_obstacle validated the obstacle"),
                    });
                }
                track.spans.push(TrackSpan::Follow {
                    t0: t,
                    t1: t,
                    link: attachment.link,
                    offset: attachment.grasp,
                });
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
                let track = self.object_track(object);
                track.spans.push(TrackSpan::Hold { t0: t, t1: t, pose });
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
        }
        Ok(())
    }

    /// The track for `object`, closing the previous span at the current
    /// time. Creates the track on first use (object grasped mid-sequence).
    fn object_track(&mut self, object: &str) -> &mut ObjectTrack {
        let t = self.t;
        let index = match self.objects.iter().position(|o| o.name == object) {
            Some(i) => i,
            None => {
                self.objects.push(ObjectTrack {
                    name: object.to_string(),
                    spans: Vec::new(),
                });
                self.objects.len() - 1
            }
        };
        if let Some(open) = self.objects[index].spans.last_mut() {
            *open.end_mut() = t;
        }
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
mod tests {
    use super::*;
    use crate::motion::{Segment, SegmentKind};
    use crate::seq::{SignalDef, Step};
    use botrail_model::Geometry;
    use nalgebra::{Translation3, UnitQuaternion, Vector3};
    use std::sync::Arc;

    /// 1-DOF arm: revolute Z at z = 0.5 (limits ±1), two 0.1 cubes.
    fn sample_scene() -> Scene {
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

    fn limits() -> botrail_traj::Limits {
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

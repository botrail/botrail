//! Grasp authoring: derive a gripper's closing values from geometry, and
//! read grasp episodes back out of a baked timeline.
//!
//! [`Scene::grasp_close`] is a teaching aid, not a planner
//! (design-grasping.md D2): with the scene posed at the grasp, the finger
//! values that bring each drive group to a chosen signed clearance from
//! the part are solved by bisection on the collision distance — then baked
//! into ordinary ramps. The bake stays deterministic; whether the taught
//! close actually *touched* is what [`SequenceTimeline::grasp_episodes`]
//! (and `tl.grasp_report()` above it) read back from the contact record.

use nalgebra::Isometry3;

use crate::rollout::{RobotTrack, SequenceTimeline, TrackSpan};
use crate::{Scene, SceneError};

/// Default signed clearance `grasp_close` closes to: half a millimetre of
/// overtravel. Measured against rapier 0.34 (the adapter's
/// `kinematic_finger_squeeze_touches_without_ejecting`): a squeeze this
/// size reports contact on every pad reliably while the opposing pushes
/// cancel, so the part stays put for the few ticks until the attach takes
/// ownership.
pub const DEFAULT_CLEARANCE: f64 = -0.0005;

/// `ancestor` is `link` or an ancestor of it, walking parent joints.
/// (`RobotModel` keeps its own copy private; the walk is three lines.)
fn is_ancestor_or_self(model: &botrail_model::RobotModel, ancestor: usize, link: usize) -> bool {
    let mut current = Some(link);
    while let Some(index) = current {
        if index == ancestor {
            return true;
        }
        current = model.links[index]
            .parent_joint
            .map(|ji| model.joints[ji].parent_link);
    }
    false
}

/// A force-limited drive on a gripper's joints (design-grasping.md G3):
/// under a physics bake the driven joints' finger links become dynamic
/// bodies moved by force-capped position motors, so holding a part is
/// friction — a too-heavy or too-fast carry slips for real. Without a
/// physics backend the declaration is inert and the fingers move exactly
/// as planned.
#[derive(Debug, Clone)]
pub struct GripperDrive {
    /// The driven actuated joints (model joint indices); their mimic
    /// followers are driven with them.
    pub joints: Vec<usize>,
    /// Resolved motor per entry of `joints`.
    pub motors: Vec<botrail_physics::JointMotor>,
    /// The declaration, kept for project persistence: explicitly named
    /// joints (empty = derived from the tool mount) and the overrides.
    pub named: Vec<String>,
    pub max_force: Option<f64>,
    pub stiffness: Option<f64>,
    pub damping: Option<f64>,
    /// Mass floor per driven finger body, kg. Rapier's contact stiffness
    /// scales with the pair's masses, so a mesh-derived few-gram finger
    /// cannot develop a newton-scale clamp no matter the motor cap —
    /// measured boundary ≈0.05 kg against a 0.2 kg part; the default
    /// 0.2 kg is a real finger-plus-carriage moving mass.
    pub finger_mass: f64,
}

/// Default driven-finger mass floor (kg); see [`GripperDrive::finger_mass`].
pub const DEFAULT_FINGER_MASS: f64 = 0.2;

/// Default motor stiffness: the same for both joint kinds, and high on
/// purpose. Rapier's position motor pulls with
/// `erp/dt = k / (dt·k + c)` — the position term must dominate the
/// damping (`dt·k ≳ c`, dt = 2.5 ms) or the motor degenerates into a
/// damper that gravity visibly bends (measured on the 2F-85: k = 1e3
/// against c = 250 rested 0.28 rad off target under nothing but the
/// fingers' own weight). 1e5 also puts the cap-saturation error
/// (`max_force / k`) at real scales: 0.3 mm for a 30 N gripper,
/// 10 mrad for a 1000 N·m knuckle.
fn default_stiffness(_kind: botrail_physics::JointKind) -> f64 {
    1e5
}

fn default_damping(kind: botrail_physics::JointKind, max_force: f64) -> f64 {
    match kind {
        // Free speed = max_force / damping: 0.1 m/s, 4 rad/s.
        botrail_physics::JointKind::Prismatic => max_force / 0.1,
        _ => max_force / 4.0,
    }
}

pub(crate) fn joint_kind(model: &botrail_model::RobotModel, ji: usize) -> botrail_physics::JointKind {
    match model.joints[ji].joint_type {
        botrail_model::JointType::Prismatic => botrail_physics::JointKind::Prismatic,
        _ => botrail_physics::JointKind::Revolute,
    }
}

impl Scene {
    /// The actuated joints a gripper command drives: named explicitly, or
    /// every actuated joint below the tool mount. Shared by `grasp_close`
    /// and `set_gripper_drive` so the two always agree on what "the
    /// gripper" is.
    fn tool_drive_joints(
        &self,
        robot: usize,
        joints: Option<&[String]>,
    ) -> Result<Vec<usize>, SceneError> {
        let bad = |m: String| Err(SceneError::BadGrasp(m));
        let model = &self.robots[robot].model;
        match joints {
            Some(names) => {
                let mut out = Vec::with_capacity(names.len());
                for name in names {
                    let Some(ji) = model.joints.iter().position(|j| &j.name == name) else {
                        return bad(format!("unknown joint `{name}`"));
                    };
                    if let Some(m) = model.joints[ji].mimic {
                        return bad(format!(
                            "joint `{name}` follows `{}`; close that joint instead",
                            model.joints[m.source_joint].name
                        ));
                    }
                    if model.joints[ji].q_index.is_none() {
                        return bad(format!("joint `{name}` is fixed"));
                    }
                    out.push(ji);
                }
                Ok(out)
            }
            None => {
                let mount = model.tool_mount_link();
                let tool = self.link_subtree(robot, mount);
                // Joints hanging OFF the mount (parent inside its
                // subtree): the joint whose child IS the mount is the
                // arm's last axis, not a finger — deriving it as a drive
                // once made a "close" hoist the whole hand and shelf the
                // part on the palm (G3 実装記録).
                let derived: Vec<usize> = model
                    .joints
                    .iter()
                    .enumerate()
                    .filter(|(_, j)| j.q_index.is_some() && tool.contains(&j.parent_link))
                    .map(|(ji, _)| ji)
                    .collect();
                if derived.is_empty() {
                    return bad(format!(
                        "no drive joints under the tool mount `{}` — is a gripper attached? \
                         (name the joints to close explicitly with joints=)",
                        model.links[mount].name
                    ));
                }
                Ok(derived)
            }
        }
    }

    /// Declares a force-limited drive on `robot`'s gripper joints. Under
    /// a physics bake the driven fingers become dynamic bodies moved by
    /// force-capped position motors and a grasped part is held by
    /// friction alone; without a physics backend the declaration is
    /// inert. Re-declaring replaces the previous drive.
    ///
    /// * `joints` — the driven actuated joints; `None` derives every
    ///   actuated joint below the tool mount (mimic followers are driven
    ///   with their sources either way).
    /// * `max_force` — the per-joint force cap (N / N·m); `None` takes
    ///   each joint's own URDF effort limit.
    /// * `stiffness` / `damping` — motor gains; the defaults saturate the
    ///   cap within a millimetre of error and bound the cap-limited free
    ///   speed to a real finger's range.
    pub fn set_gripper_drive(
        &mut self,
        robot: usize,
        joints: Option<&[String]>,
        max_force: Option<f64>,
        stiffness: Option<f64>,
        damping: Option<f64>,
        finger_mass: Option<f64>,
    ) -> Result<(), SceneError> {
        let drives = self.tool_drive_joints(robot, joints)?;
        let model = self.robots[robot].model.clone();
        let mut motors = Vec::with_capacity(drives.len());
        for &ji in &drives {
            let kind = joint_kind(&model, ji);
            let cap = match max_force {
                Some(f) if f.is_finite() && f > 0.0 => f,
                Some(f) => {
                    return Err(SceneError::BadGrasp(format!(
                        "max_force must be positive, got {f}"
                    )))
                }
                None => {
                    let effort = model.joints[ji].limits.as_ref().map_or(0.0, |l| l.effort);
                    if !(effort.is_finite() && effort > 0.0) {
                        return Err(SceneError::BadGrasp(format!(
                            "`{}` declares no effort limit — pass max_force=",
                            model.joints[ji].name
                        )));
                    }
                    effort
                }
            };
            motors.push(botrail_physics::JointMotor {
                stiffness: stiffness.unwrap_or_else(|| default_stiffness(kind)),
                damping: damping.unwrap_or_else(|| default_damping(kind, cap)),
                max_force: cap,
            });
        }
        let finger_mass = finger_mass.unwrap_or(DEFAULT_FINGER_MASS);
        if !(finger_mass.is_finite() && finger_mass > 0.0) {
            return Err(SceneError::BadGrasp(format!(
                "finger_mass must be positive, got {finger_mass}"
            )));
        }
        self.robots[robot].gripper_drive = Some(GripperDrive {
            joints: drives,
            motors,
            named: joints.map(|n| n.to_vec()).unwrap_or_default(),
            max_force,
            stiffness,
            damping,
            finger_mass,
        });
        Ok(())
    }

    /// The declared gripper drive of a robot, if any.
    pub fn gripper_drive(&self, robot: usize) -> Option<&GripperDrive> {
        self.robots[robot].gripper_drive.as_ref()
    }

    /// Solves the joint values that close `robot`'s tool on the obstacle
    /// `object`, to a signed `clearance` (negative = overtravel into the
    /// surface, the [`DEFAULT_CLEARANCE`]; positive = stop short). Returns
    /// `(joint name, value)` for every drive joint it solved — ready to
    /// hand to a ramp.
    ///
    /// Pose the grasp first: the solve runs at the current configuration
    /// and the part's current pose, the same contract as `attach`.
    ///
    /// * `joints` — the drive joints to close. `None` derives them: every
    ///   actuated joint below the tool mount (mimic followers close with
    ///   their drivers automatically).
    /// * `closed` — per-joint fully-closed values. Absent entries use the
    ///   position-limit end that approaches the part (measured, not
    ///   guessed); a limitless drive joint must be given one.
    ///
    /// Joints on one serial chain close together (one scalar sweep);
    /// independent branches — a multi-finger hand — close one branch at a
    /// time, each stopping at its own first touch.
    pub fn grasp_close(
        &self,
        robot: usize,
        object: &str,
        joints: Option<&[String]>,
        closed: Option<&[(String, f64)]>,
        clearance: f64,
    ) -> Result<Vec<(String, f64)>, SceneError> {
        let bad = |m: String| Err(SceneError::BadGrasp(m));
        let index = self.obstacle_index(object)?;
        if self.is_attached(object) {
            return bad(format!(
                "`{object}` is already attached — grasp_close teaches the close on a free part"
            ));
        }
        let sr = &self.robots[robot];
        let model = sr.model.clone();

        // The drive joints: named explicitly, or every actuated joint
        // below the tool mount.
        let drives = self.tool_drive_joints(robot, joints)?;

        // Joints on one serial chain sweep together; independent branches
        // (a multi-finger hand) solve one at a time.
        let mut groups: Vec<Vec<usize>> = Vec::new();
        for &ji in &drives {
            let child = model.joints[ji].child_link;
            let chained = groups.iter_mut().find(|g| {
                g.iter().any(|&other| {
                    let oc = model.joints[other].child_link;
                    is_ancestor_or_self(&model, oc, child) || is_ancestor_or_self(&model, child, oc)
                })
            });
            match chained {
                Some(g) => g.push(ji),
                None => groups.push(vec![ji]),
            }
        }

        let obstacle_pose = self.obstacles[index].pose;
        let q0 = sr.joint_positions().to_vec();
        let mut result = Vec::new();
        for group in &groups {
            // Every link this group's motion carries: the subtrees below
            // its joints and below every joint that mimics one of them
            // (the second finger of a coupled gripper).
            let mut moved: Vec<usize> = Vec::new();
            for (ji, j) in model.joints.iter().enumerate() {
                let driven = group.contains(&ji)
                    || j.mimic.is_some_and(|m| group.contains(&m.source_joint));
                if driven {
                    moved.extend(self.link_subtree(robot, j.child_link));
                }
            }
            moved.sort_unstable();
            moved.dedup();
            moved.retain(|&l| !sr.collider().link_parts(l).is_empty());
            let names: Vec<&str> = group.iter().map(|&ji| model.joints[ji].name.as_str()).collect();
            if moved.is_empty() {
                return bad(format!(
                    "the links driven by `{}` carry no collision geometry",
                    names.join("`, `")
                ));
            }

            // Signed clearance between the group's links and the part at a
            // trial configuration.
            let distance = |q: &[f64]| -> f64 {
                let poses = self.fk_for(robot, q).expect("q0 length is the model's dof");
                let mut min = f64::INFINITY;
                for &l in &moved {
                    let parts = sr.collider().link_parts(l);
                    if let Some(d) = botrail_collide::parts_signed_distance(
                        &poses[l],
                        parts,
                        &obstacle_pose,
                        self.obstacle_colliders()[index].parts(),
                    ) {
                        min = min.min(d);
                    }
                }
                min
            };
            let at = |s: f64, targets: &[f64]| -> Vec<f64> {
                let mut q = q0.clone();
                for (&ji, &target) in group.iter().zip(targets) {
                    let qi = model.joints[ji].q_index.expect("drives are actuated");
                    q[qi] += s * (target - q[qi]);
                }
                q
            };

            // Fully-closed targets: explicit values win; otherwise the
            // position-limit end that approaches the part — measured by
            // trying both, not guessed from a sign convention.
            let explicit = |ji: usize| -> Option<f64> {
                closed.and_then(|m| {
                    m.iter()
                        .find(|(n, _)| n == &model.joints[ji].name)
                        .map(|(_, v)| *v)
                })
            };
            let end = |pick_upper: bool| -> Result<Vec<f64>, SceneError> {
                group
                    .iter()
                    .map(|&ji| {
                        if let Some(v) = explicit(ji) {
                            return Ok(v);
                        }
                        match &model.joints[ji].limits {
                            Some(l) => Ok(if pick_upper { l.upper } else { l.lower }),
                            None => Err(SceneError::BadGrasp(format!(
                                "`{}` has no position limits — pass closed={{\"{}\": value}}",
                                model.joints[ji].name, model.joints[ji].name
                            ))),
                        }
                    })
                    .collect()
            };
            let d0 = distance(&at(0.0, &end(false)?));
            if d0 <= clearance {
                // Already at (or inside) the asked clearance: nothing to
                // close; the current values are the answer.
                for &ji in group {
                    let qi = model.joints[ji].q_index.expect("drives are actuated");
                    result.push((model.joints[ji].name.clone(), q0[qi]));
                }
                continue;
            }

            // A curling finger *approaches and leaves again* — at full curl
            // it has folded past the part, so the closed-end distance says
            // nothing. Scan each candidate sweep forward for its FIRST
            // touch (the closing direction is whichever limit end's sweep
            // actually reaches contact), then sharpen the bracketing step
            // by bisection.
            const STEPS: usize = 256;
            let candidates: Vec<Vec<f64>> = if group.iter().all(|&ji| explicit(ji).is_some()) {
                vec![end(false)?]
            } else {
                vec![end(false)?, end(true)?]
            };
            let mut bracket: Option<(f64, f64, usize)> = None;
            let mut closest = f64::INFINITY;
            for (c, targets) in candidates.iter().enumerate() {
                let mut prev = 0.0f64;
                for k in 1..=STEPS {
                    let s = k as f64 / STEPS as f64;
                    let d = distance(&at(s, targets));
                    if d.is_finite() {
                        closest = closest.min(d);
                    }
                    if d <= clearance {
                        // The earlier first touch wins between directions —
                        // less travel to the same contact.
                        if bracket.is_none_or(|(_, s_hi, _)| s < s_hi) {
                            bracket = Some((prev, s, c));
                        }
                        break;
                    }
                    prev = s;
                }
            }
            let Some((mut lo, mut hi, c)) = bracket else {
                let closest = if closest.is_finite() {
                    format!("closest {:.1} mm", closest * 1000.0)
                } else {
                    "nothing within range".to_string()
                };
                return bad(format!(
                    "closing `{}` never reaches `{object}` anywhere along its \
                     sweep ({closest}) — pose the grasp over the part first",
                    names.join("`, `")
                ));
            };
            let targets = &candidates[c];
            for _ in 0..40 {
                if hi - lo < 1e-9 {
                    break;
                }
                let mid = 0.5 * (lo + hi);
                if distance(&at(mid, targets)) > clearance {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            // The touching side of the bracket, so the derived close is
            // guaranteed at (or a hair inside) the clearance.
            let q = at(hi, targets);
            for &ji in group {
                let qi = model.joints[ji].q_index.expect("drives are actuated");
                result.push((model.joints[ji].name.clone(), q[qi]));
            }
        }
        Ok(result)
    }
}

/// One grasp as a baked timeline recorded it: an object's `Follow` stretch,
/// annotated with what the physics contact record says about how it began
/// and ended. On a kinematic bake the contact fields are simply empty —
/// the episode structure itself is engine-independent.
#[derive(Debug, Clone)]
pub struct GraspEpisode {
    pub object: String,
    /// Carrying robot's instance name and anchor link.
    pub robot: String,
    pub link: String,
    /// The `Follow` span: attach time to release (or the horn).
    pub start: f64,
    pub end: f64,
    /// The object was still held when the bake ended (no release).
    pub held_to_end: bool,
    /// Tool-subtree links touching the object around the attach, with the
    /// peak contact force (N) their episode recorded.
    pub touched: Vec<(String, f64)>,
    /// The release happened while fingers were still on the part — the
    /// authored order should be "open, then detach" (design-grasping.md:
    /// a body returned to dynamic inside a squeeze gets kicked).
    pub released_touching: bool,
    /// The mass the physics bake used: authored/part-identified, else the
    /// collision volume at the default density. `None` when the object has
    /// no physics authoring at all.
    pub mass_kg: Option<f64>,
    /// Largest carry acceleration (m/s², translation only), sampled over
    /// the follow span — what the grip must hold against, on top of g.
    pub max_accel: f64,
    /// Friction holds only: the largest distance between where the hold's
    /// intent says the object should be (carrier link ∘ attach offset)
    /// and where physics actually put it. Millimetres of slip; a dropped
    /// part reads as metres. `None` on a welded (attachment) grasp.
    pub slip_max: Option<f64>,
}

/// Half-width of the window contacts are matched in around the attach and
/// release instants. Contacts end *at* the attach (the part turns
/// kinematic, and kinematic-kinematic pairs stop reporting), so the window
/// only needs to absorb tick granularity.
const MATCH_WINDOW: f64 = 0.06;

impl SequenceTimeline {
    /// Every grasp this bake performed, in `Follow`-span order, annotated
    /// from the contact record. `scene` must be the scene the timeline was
    /// baked from (the pre-rollout snapshot).
    pub fn grasp_episodes(&self, scene: &Scene) -> Vec<GraspEpisode> {
        let mut out = Vec::new();
        // Welded grasps: the object rode a Follow span.
        let mut welds: Vec<(String, f64, f64, usize, usize, Isometry3<f64>)> = Vec::new();
        for track in &self.objects {
            for span in &track.spans {
                if let &TrackSpan::Follow {
                    t0,
                    t1,
                    robot,
                    link,
                    offset,
                } = span
                {
                    welds.push((track.name.clone(), t0, t1, robot, link, offset));
                }
            }
        }
        // Friction holds: physics owned the object; the declaration is
        // the intent, and slip is measured against it.
        let holds: Vec<(String, f64, f64, usize, usize, Isometry3<f64>)> = self
            .grasps
            .iter()
            .map(|h| (h.object.clone(), h.start, h.end, h.robot, h.link, h.offset))
            .collect();
        for (kind, list) in [(false, welds), (true, holds)] {
            for (name, t0, t1, robot, link, offset) in list {
                let friction = kind;
                let track = self.objects.iter().find(|tr| tr.name == name);
                let Some(sr) = scene.robots().get(robot) else {
                    continue;
                };
                let model = &sr.model;
                let mount = model.tool_mount_link();
                // Contact names are `"{instance}/{link}"`; collect the
                // tool subtree's, mapped back to bare link names.
                let tool: Vec<(String, String)> = scene
                    .link_subtree(robot, mount)
                    .into_iter()
                    .map(|l| {
                        let bare = model.links[l].name.clone();
                        (format!("{}/{}", sr.name, bare), bare)
                    })
                    .collect();
                let part_side = |a: &str, b: &str| -> Option<String> {
                    let other = if a == name {
                        b
                    } else if b == name {
                        a
                    } else {
                        return None;
                    };
                    tool.iter()
                        .find(|(qualified, _)| qualified == other)
                        .map(|(_, bare)| bare.clone())
                };

                let mut touched: Vec<(String, f64)> = Vec::new();
                let mut released_touching = false;
                let held_to_end = t1 >= self.duration - 1e-9;
                for c in &self.contacts {
                    let Some(bare) = part_side(&c.a, &c.b) else {
                        continue;
                    };
                    // Open across the attach instant (the close that led
                    // into this grasp).
                    if c.start <= t0 + MATCH_WINDOW && c.end >= t0 - MATCH_WINDOW {
                        match touched.iter_mut().find(|(n, _)| n == &bare) {
                            Some((_, f)) => *f = f.max(c.peak_force),
                            None => touched.push((bare.clone(), c.peak_force)),
                        }
                    }
                    // Opening right at the release: the part came back
                    // dynamic while still inside the fingers.
                    if !held_to_end && c.start <= t1 + MATCH_WINDOW && c.end >= t1 {
                        released_touching = true;
                    }
                }
                touched.sort_by(|a, b| a.0.cmp(&b.0));

                let mass_kg = scene.resolved_body_props(&name).map(|props| {
                    props.mass.unwrap_or_else(|| {
                        scene
                            .obstacle_index(&name)
                            .map(|i| {
                                botrail_collide::parts_volume(
                                    scene.obstacle_colliders()[i].parts(),
                                ) * botrail_physics::DEFAULT_DENSITY
                            })
                            .unwrap_or(0.0)
                    })
                });

                let max_accel = carry_accel(self, scene, robot, link, &offset, t0, t1);
                // Slip: intent (carrier ∘ offset) vs where physics put it.
                let slip_max = if friction {
                    let track_spans = track.map(|tr| tr.spans.as_slice()).unwrap_or(&[]);
                    let rt = self.robots.get(robot);
                    let mut slip: f64 = 0.0;
                    let mut t = t0;
                    while t <= t1 {
                        let expected = rt.and_then(|rt| {
                            let q = rt.trajectory.sample(t);
                            let base = base_at(rt, sr, t);
                            botrail_kin::forward_kinematics_with_base(&sr.model, &q, &base)
                                .ok()
                                .map(|poses| (poses[link] * offset).translation.vector)
                        });
                        let actual = SequenceTimeline::span_pose(track_spans, &[], t)
                            .map(|p| p.translation.vector);
                        if let (Some(e), Some(a)) = (expected, actual) {
                            slip = slip.max((e - a).norm());
                        }
                        t += 0.02;
                    }
                    Some(slip)
                } else {
                    None
                };
                out.push(GraspEpisode {
                    object: name.clone(),
                    robot: sr.name.clone(),
                    link: model.links[link].name.clone(),
                    start: t0,
                    end: t1,
                    held_to_end,
                    touched,
                    released_touching,
                    mass_kg,
                    max_accel,
                    slip_max,
                });
            }
        }
        out.sort_by(|a, b| a.start.total_cmp(&b.start));
        out
    }
}

/// Numeric catalog specs of every gripper in a robot's source tree — the
/// tool welded on by `attach_tool` keeps its `RobotSource::Catalog` record,
/// category and specs included, inside the composite. What lets
/// `grasp_report` default its holding checks to the mounted gripper's own
/// numbers (`grip_force_min_n`, `payload_kg`) instead of asking the author
/// to retype the datasheet.
pub fn gripper_tool_specs(scene: &Scene, robot: usize) -> Vec<(String, f64)> {
    fn walk(source: &botrail_model::RobotSource, out: &mut Vec<(String, f64)>) {
        match source {
            botrail_model::RobotSource::Catalog { meta, inner, .. } => {
                if meta
                    .category
                    .as_deref()
                    .is_some_and(|c| c.starts_with("gripper."))
                {
                    out.extend(meta.specs.iter().cloned());
                }
                walk(inner, out);
            }
            botrail_model::RobotSource::Composite { base, tool, .. } => {
                walk(base, out);
                walk(tool, out);
            }
            _ => {}
        }
    }
    let mut out = Vec::new();
    if let Some(sr) = scene.robots().get(robot) {
        walk(&sr.model.source, &mut out);
    }
    out
}

/// Largest translational acceleration of `link ∘ offset` over `[t0, t1]`,
/// by central differences on the baked joint track.
fn carry_accel(
    tl: &SequenceTimeline,
    scene: &Scene,
    robot: usize,
    link: usize,
    offset: &Isometry3<f64>,
    t0: f64,
    t1: f64,
) -> f64 {
    const H: f64 = 0.02;
    let Some(rt) = tl.robots.get(robot) else {
        return 0.0;
    };
    let sr = &scene.robots()[robot];
    let pos = |t: f64| -> Option<nalgebra::Vector3<f64>> {
        let q = rt.trajectory.sample(t);
        let base = base_at(rt, sr, t);
        let poses = botrail_kin::forward_kinematics_with_base(&sr.model, &q, &base).ok()?;
        Some((poses[link] * offset).translation.vector)
    };
    let mut max = 0.0f64;
    let mut t = t0 + H;
    while t <= t1 - H {
        if let (Some(a), Some(b), Some(c)) = (pos(t - H), pos(t), pos(t + H)) {
            max = max.max((a - 2.0 * b + c).norm() / (H * H));
        }
        t += H;
    }
    max
}

/// The carrying robot's base at `t`: its ride track if it drove, else the
/// scene's static base.
fn base_at(rt: &RobotTrack, sr: &crate::SceneRobot, t: f64) -> Isometry3<f64> {
    SequenceTimeline::base_pose(rt, t).unwrap_or_else(|| *sr.base_pose())
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use botrail_model::{Geometry, RobotModel};
    use nalgebra::Vector3;

    use super::*;
    use crate::rollout::RolloutOptions;
    use crate::seq::{Action, Condition, Sequence, Step};

    const ARM: &str = include_str!("../../../examples/assets/simple_arm.urdf");
    const GRIPPER: &str = crate::testdata::GRIPPER_URDF;

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::translation(x, y, z)
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.into(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    /// A bare gripper standing at the origin with a 40 mm part centred
    /// between its pads (60 mm opening).
    fn gripper_scene() -> Scene {
        let mut scene = Scene::new(Arc::new(RobotModel::from_urdf_str(GRIPPER).unwrap()));
        scene
            .add_obstacle(
                "part",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.02, 0.04),
                },
                iso(0.0, 0.0, 0.07),
            )
            .unwrap();
        scene
    }

    /// The fixture's pads touch a 40 mm part at drive = 0.010 exactly, so
    /// the default half-millimetre overtravel solves to 0.0105 — pinned
    /// geometrically, not against the solver's own distance function.
    #[test]
    fn grasp_close_derives_the_touch_width() {
        let scene = gripper_scene();
        let solved = scene
            .grasp_close(0, "part", None, None, DEFAULT_CLEARANCE)
            .unwrap();
        assert_eq!(solved.len(), 1, "one drive joint: {solved:?}");
        assert_eq!(solved[0].0, "drive");
        assert!(
            (solved[0].1 - 0.0105).abs() < 1e-4,
            "drive = {}, expected 0.0105",
            solved[0].1
        );
        // The mimic finger closes with its driver — it never appears as
        // its own output row.
        assert!(solved.iter().all(|(n, _)| n != "follow"));

        // An explicit fully-closed value takes the same path to the same
        // touch (the limit end merely loses the vote).
        let explicit = scene
            .grasp_close(
                0,
                "part",
                None,
                Some(&[("drive".to_string(), 0.028)]),
                DEFAULT_CLEARANCE,
            )
            .unwrap();
        assert!((explicit[0].1 - solved[0].1).abs() < 1e-6);
    }

    /// A revolute finger sweeps *past* the part: it touches mid-arc and is
    /// far away again at its closed limit. The solve must find the first
    /// touch along the sweep — requiring contact at the closed end is the
    /// bug this test pins (a Dex3-1 curl exposed it: design-grasping.md
    /// G2 実装記録).
    #[test]
    fn grasp_close_finds_the_first_touch_of_a_curling_finger() {
        const CURL: &str = r#"<?xml version="1.0"?>
        <robot name="curl">
          <link name="palm">
            <collision><origin xyz="0 0 -0.02"/><geometry><box size="0.06 0.06 0.02"/></geometry></collision>
          </link>
          <link name="finger">
            <collision><origin xyz="0.08 0 0"/><geometry><box size="0.16 0.01 0.01"/></geometry></collision>
          </link>
          <joint name="curl" type="revolute">
            <parent link="palm"/><child link="finger"/>
            <origin xyz="0 0 0.02"/><axis xyz="0 0 1"/>
            <limit lower="0" upper="3.1" effort="1" velocity="1"/>
          </joint>
        </robot>"#;
        let mut scene = Scene::new(Arc::new(RobotModel::from_urdf_str(CURL).unwrap()));
        // The part sits at 90° on the finger's arc: touched mid-sweep,
        // far behind the finger at the 178° closed limit.
        scene
            .add_obstacle(
                "part",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.04, 0.04),
                },
                iso(0.0, 0.12, 0.02),
            )
            .unwrap();
        // A single-chain model's tool mount is its tip, so the drive is
        // named explicitly — the derivation error says exactly that.
        let solved = scene
            .grasp_close(0, "part", Some(&["curl".to_string()]), None, DEFAULT_CLEARANCE)
            .unwrap();
        assert_eq!(solved[0].0, "curl");
        // First touch: the 10 mm-thick finger meets the box's -x face
        // region near 60-80°, well before sweeping past it.
        assert!(
            solved[0].1 > 0.6 && solved[0].1 < 1.5,
            "curl = {} rad, expected the first touch on the approach side",
            solved[0].1
        );
    }

    /// A part the close can never reach is an authoring error, said with
    /// the measured miss distance, not a silent full close.
    #[test]
    fn grasp_close_reports_an_unreachable_part() {
        let mut scene = Scene::new(Arc::new(RobotModel::from_urdf_str(GRIPPER).unwrap()));
        scene
            .add_obstacle(
                "part",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.02, 0.04),
                },
                iso(0.4, 0.0, 0.07),
            )
            .unwrap();
        let err = scene
            .grasp_close(0, "part", None, None, DEFAULT_CLEARANCE)
            .expect_err("part is 40 cm away")
            .to_string();
        assert!(err.contains("never reaches"), "{err}");
    }

    /// An arm with no gripper has no drive joints below its tool mount —
    /// the error says so instead of solving nothing.
    #[test]
    fn grasp_close_requires_a_gripper() {
        let mut scene = Scene::new(Arc::new(RobotModel::from_urdf_str(ARM).unwrap()));
        scene
            .add_obstacle(
                "part",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.04, 0.04),
                },
                iso(0.5, 0.0, 0.3),
            )
            .unwrap();
        let err = scene
            .grasp_close(0, "part", None, None, DEFAULT_CLEARANCE)
            .expect_err("no gripper attached")
            .to_string();
        assert!(err.contains("no drive joints"), "{err}");
    }

    /// `touch_links = ["tool"]` exempts the whole tool subtree — palm and
    /// both fingers — not just the anchor's own chain.
    #[test]
    fn attach_touch_links_tool_expands_the_subtree() {
        let arm = RobotModel::from_urdf_str(ARM).unwrap();
        let gripper = RobotModel::from_urdf_str(GRIPPER).unwrap();
        let robot = arm
            .attach_tool(
                &gripper,
                Some("tool0"),
                None,
                Isometry3::identity(),
                None,
                None,
            )
            .unwrap();
        let mut scene = Scene::new(Arc::new(robot));
        let (px, py, pz) = {
            let poses = scene.link_poses_for(0);
            let tcp = scene.robots()[0].model.default_tcp_link();
            let t = poses[tcp].translation;
            (t.x, t.y, t.z)
        };
        scene
            .add_obstacle(
                "part",
                Geometry::Box {
                    size: Vector3::new(0.03, 0.015, 0.03),
                },
                iso(px, py, pz),
            )
            .unwrap();
        scene
            .attach_obstacle_to(0, "part", None, Some(&["tool".to_string()]))
            .unwrap();
        let attachment = scene.attachment("part").unwrap().clone();
        let model = scene.robots()[0].model.clone();
        let named: Vec<&str> = attachment
            .touch_links
            .iter()
            .map(|&l| model.links[l].name.as_str())
            .collect();
        for expected in ["palm", "finger_l", "finger_r", "grasp_center"] {
            assert!(named.contains(&expected), "missing {expected}: {named:?}");
        }
        // A literal link name still resolves as itself (plus the anchor,
        // when it differs) — no group expansion.
        scene.detach_obstacle("part").unwrap();
        scene
            .attach_obstacle_to(0, "part", None, Some(&["finger_l".to_string()]))
            .unwrap();
        let literal = scene.attachment("part").unwrap().clone();
        let named: Vec<&str> = literal
            .touch_links
            .iter()
            .map(|&l| model.links[l].name.as_str())
            .collect();
        assert!(named.contains(&"finger_l"), "{named:?}");
        assert!(!named.contains(&"palm"), "no expansion: {named:?}");
    }

    /// Link materials survive a project round-trip, and an unknown link is
    /// a load error, not a silent drop.
    #[test]
    fn link_material_round_trips_through_a_project() {
        let mut scene = Scene::new(Arc::new(RobotModel::from_urdf_str(GRIPPER).unwrap()));
        scene
            .set_link_material(0, "finger_l", Some(1.1), None)
            .unwrap();
        scene
            .set_link_material(0, "finger_r", Some(1.1), Some(0.1))
            .unwrap();
        assert!(scene.set_link_material(0, "nope", Some(1.0), None).is_err());

        let project = scene.to_project();
        let json = serde_json::to_string(&project).unwrap();
        let reloaded =
            Scene::from_project(&crate::project::ProjectFile::from_json(&json).unwrap()).unwrap();
        let model = reloaded.robots()[0].model.clone();
        let l = model.link_index("finger_l").unwrap();
        let r = model.link_index("finger_r").unwrap();
        let ml = reloaded.link_material(0, l).unwrap();
        assert!((ml.friction - 1.1).abs() < 1e-12);
        assert!((ml.restitution - 0.0).abs() < 1e-12);
        assert!((reloaded.link_material(0, r).unwrap().restitution - 0.1).abs() < 1e-12);
    }

    /// The physics grasp cycle end-to-end: the derived close touches, the
    /// episode names both pads, and a release *inside* the squeeze — the
    /// wrong authored order — is called out.
    #[test]
    fn a_physics_bake_records_which_pads_touched() {
        let mut scene = gripper_scene();
        // Seat the part: a pedestal whose top is the part's bottom.
        scene
            .add_obstacle(
                "pedestal",
                Geometry::Box {
                    size: Vector3::new(0.12, 0.12, 0.05),
                },
                iso(0.0, 0.0, 0.025),
            )
            .unwrap();
        scene
            .set_obstacle_physics(
                "part",
                Some(botrail_physics::BodyProps {
                    mass: Some(0.1),
                    ..botrail_physics::BodyProps::dynamic()
                }),
            )
            .unwrap();
        let solved = scene
            .grasp_close(0, "part", None, None, DEFAULT_CLEARANCE)
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "cycle".into(),
            steps: vec![
                step(
                    "close",
                    vec![Action::StartRamp {
                        robot: None,
                        targets: solved.clone(),
                        duration: 0.4,
                    }],
                    Condition::Done,
                ),
                step(
                    "grab",
                    vec![Action::Attach {
                        robot: None,
                        object: "part".into(),
                        link: None,
                        touch_links: Some(vec!["tool".into()]),
                    }],
                    Condition::Elapsed { seconds: 0.5 },
                ),
                step(
                    "drop",
                    vec![Action::Detach {
                        object: "part".into(),
                    }],
                    Condition::Elapsed { seconds: 0.5 },
                ),
            ],
        });
        let timeline = scene
            .simulate_sequences_with(
                &["cycle"],
                &RolloutOptions::default(),
                Some(Box::new(botrail_physics_rapier::RapierBackend::new())),
            )
            .unwrap();
        let episodes = timeline.grasp_episodes(&scene);
        assert_eq!(episodes.len(), 1, "episodes: {episodes:?}");
        let ep = &episodes[0];
        assert_eq!(ep.object, "part");
        // The anchor is the model's default TCP (the bare URDF declares
        // none, so the deepest-leaf heuristic picks a finger).
        let model = &scene.robots()[0].model;
        assert_eq!(ep.link, model.links[model.default_tcp_link()].name);
        let touched: Vec<&str> = ep.touched.iter().map(|(n, _)| n.as_str()).collect();
        assert!(
            touched.contains(&"finger_l") && touched.contains(&"finger_r"),
            "touched: {touched:?}"
        );
        assert!(ep.touched.iter().all(|(_, f)| *f > 0.0));
        assert_eq!(ep.mass_kg, Some(0.1));
        assert!(!ep.held_to_end);
        // The sequence detaches while the pads still squeeze — exactly the
        // authored order the report is meant to flag.
        assert!(ep.released_touching, "release inside the squeeze");
    }

    /// The G3 friction grasp end to end: a lift-jointed gripper closes on
    /// the part with a force-capped drive, ATTACH IS ONLY A DECLARATION —
    /// physics owns the part throughout — and the lift carries it up by
    /// friction alone. The same authoring with a feeble cap slips: the
    /// part stays behind, and the report's slip says so in metres.
    const LIFTER: &str = r#"<?xml version="1.0"?>
    <robot name="lifter">
      <link name="base">
        <!-- Off to the side: the part hoists straight up past the base,
             and a box in that corridor jams the ride at part-top = box-
             bottom (measured: a centered box stopped the part 44 mm
             short and read as slip). -->
        <collision><origin xyz="0 0.08 0.05"/><geometry><box size="0.05 0.05 0.02"/></geometry></collision>
      </link>
      <link name="palm">
        <collision><origin xyz="0 0 -0.02"/><geometry><box size="0.08 0.06 0.04"/></geometry></collision>
      </link>
      <link name="finger_l">
        <collision><origin xyz="0 0 -0.03"/><geometry><box size="0.01 0.02 0.06"/></geometry></collision>
      </link>
      <link name="finger_r">
        <collision><origin xyz="0 0 -0.03"/><geometry><box size="0.01 0.02 0.06"/></geometry></collision>
      </link>
      <joint name="lift" type="prismatic">
        <parent link="base"/><child link="palm"/>
        <axis xyz="0 0 1"/>
        <limit lower="-0.3" upper="0.3" effort="200" velocity="0.5"/>
      </joint>
      <joint name="drive" type="prismatic">
        <parent link="palm"/><child link="finger_l"/>
        <origin xyz="-0.035 0 -0.04"/><axis xyz="1 0 0"/>
        <limit lower="0" upper="0.028" effort="30" velocity="0.1"/>
      </joint>
      <joint name="follow" type="prismatic">
        <parent link="palm"/><child link="finger_r"/>
        <origin xyz="0.035 0 -0.04"/><axis xyz="1 0 0"/>
        <limit lower="-0.028" upper="0" effort="30" velocity="0.1"/>
        <mimic joint="drive" multiplier="-1" offset="0"/>
      </joint>
    </robot>"#;

    /// Builds the lifter cell: the palm hangs fingers-DOWN over the part
    /// (so the rising palm can never scoop it up like a tray — the first
    /// cut of this fixture did exactly that and faked a hold), pads
    /// straddling a part on a pedestal, drive declared at `max_force`.
    fn lifter_scene(max_force: f64) -> Scene {
        // Base high enough that the fingers straddle the part's UPPER
        // half — a finger reaching below the part's bottom is buried in
        // the pedestal (the first cut of this fixture measured exactly
        // that drag and called it a grip).
        let mut scene = Scene::with_base(
            Arc::new(RobotModel::from_urdf_str(LIFTER).unwrap()),
            iso(0.0, 0.0, 0.315),
        );
        scene
            .add_obstacle(
                "part",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.02, 0.04),
                },
                iso(0.0, 0.0, 0.23),
            )
            .unwrap();
        scene
            .add_obstacle(
                "pedestal",
                Geometry::Box {
                    size: Vector3::new(0.12, 0.12, 0.21),
                },
                iso(0.0, 0.0, 0.105),
            )
            .unwrap();
        scene
            .set_obstacle_physics(
                "part",
                Some(botrail_physics::BodyProps {
                    mass: Some(0.2),
                    material: botrail_physics::PhysicsMaterial {
                        friction: 0.6,
                        ..Default::default()
                    },
                    ..botrail_physics::BodyProps::dynamic()
                }),
            )
            .unwrap();
        // Rubber pads: the friction the hold lives on.
        scene.set_link_material(0, "finger_l", Some(0.9), None).unwrap();
        scene.set_link_material(0, "finger_r", Some(0.9), None).unwrap();
        scene
            .set_gripper_drive(0, Some(&["drive".to_string()]), Some(max_force), None, None, None)
            .unwrap();
        // Friction-drive close: ~2 mm of overtravel is what the contact
        // needs to develop a multi-newton clamp (the rapier spike's
        // measured law); the motor cap is the ceiling.
        let q_close = scene.grasp_close(0, "part", None, None, -0.002).unwrap();
        scene.upsert_sequence(Sequence {
            name: "cycle".into(),
            steps: vec![
                step(
                    "close",
                    vec![Action::StartRamp {
                        robot: None,
                        targets: q_close,
                        duration: 0.5,
                    }],
                    Condition::Done,
                ),
                step(
                    "grab",
                    vec![Action::Attach {
                        robot: None,
                        object: "part".into(),
                        link: Some("palm".into()),
                        touch_links: Some(vec!["tool".into()]),
                    }],
                    Condition::Elapsed { seconds: 0.3 },
                ),
                step(
                    "hoist",
                    vec![Action::StartRamp {
                        robot: None,
                        targets: vec![("lift".into(), 0.15)],
                        duration: 1.0,
                    }],
                    Condition::Done,
                ),
                step("hold", vec![], Condition::Elapsed { seconds: 0.5 }),
            ],
        });
        scene
    }

    #[test]
    fn a_friction_drive_lifts_the_part_without_a_weld() {
        let scene = lifter_scene(30.0);
        let timeline = scene
            .simulate_sequences_with(
                &["cycle"],
                &RolloutOptions::default(),
                Some(Box::new(botrail_physics_rapier::RapierBackend::new())),
            )
            .unwrap();
        // The declaration is on the timeline; the object track is
        // physics-sampled, never a Follow span.
        assert_eq!(timeline.grasps.len(), 1);
        let track = timeline
            .objects
            .iter()
            .find(|tr| tr.name == "part")
            .expect("part has a track");
        assert!(
            !track
                .spans
                .iter()
                .any(|s| matches!(s, TrackSpan::Follow { .. })),
            "a friction hold never welds"
        );
        let end = SequenceTimeline::span_pose(&track.spans, &[], timeline.duration)
            .expect("part pose at the horn");
        assert!(
            end.translation.z > 0.23 + 0.10,
            "friction should carry the part up ~0.15 m, ended at z = {}",
            end.translation.z
        );
        let episodes = timeline.grasp_episodes(&scene);
        let ep = episodes
            .iter()
            .find(|e| e.slip_max.is_some())
            .expect("the hold has an episode");
        assert!(
            ep.slip_max.unwrap() < 0.015,
            "held part slipped {} m",
            ep.slip_max.unwrap()
        );
        assert!(!ep.touched.is_empty(), "the pads touched");
    }

    #[test]
    fn a_feeble_cap_slips_and_the_report_says_so() {
        let scene = lifter_scene(0.3);

        let timeline = scene
            .simulate_sequences_with(
                &["cycle"],
                &RolloutOptions::default(),
                Some(Box::new(botrail_physics_rapier::RapierBackend::new())),
            )
            .unwrap();
        let track = timeline
            .objects
            .iter()
            .find(|tr| tr.name == "part")
            .expect("part has a track");
        let end = SequenceTimeline::span_pose(&track.spans, &[], timeline.duration)
            .expect("part pose at the horn");
        assert!(
            end.translation.z < 0.28,
            "0.3 N should slip on a 200 g part, but it rose to z = {}",
            end.translation.z
        );
        let episodes = timeline.grasp_episodes(&scene);
        let ep = episodes
            .iter()
            .find(|e| e.slip_max.is_some())
            .expect("the hold has an episode");
        assert!(
            ep.slip_max.unwrap() > 0.05,
            "the slip should be obvious, got {} m",
            ep.slip_max.unwrap()
        );
    }

    /// A kinematic bake still yields the episode structure — with empty
    /// contact fields, not a lie about touching.
    #[test]
    fn a_kinematic_bake_yields_bare_episodes() {
        let mut scene = gripper_scene();
        scene.upsert_sequence(Sequence {
            name: "cycle".into(),
            steps: vec![
                step(
                    "grab",
                    vec![Action::Attach {
                        robot: None,
                        object: "part".into(),
                        link: None,
                        touch_links: Some(vec!["tool".into()]),
                    }],
                    Condition::Elapsed { seconds: 0.3 },
                ),
                step(
                    "drop",
                    vec![Action::Detach {
                        object: "part".into(),
                    }],
                    Condition::Elapsed { seconds: 0.2 },
                ),
            ],
        });
        let timeline = scene
            .simulate_sequences_with(&["cycle"], &RolloutOptions::default(), None)
            .unwrap();
        let episodes = timeline.grasp_episodes(&scene);
        assert_eq!(episodes.len(), 1);
        assert!(episodes[0].touched.is_empty());
        assert!(!episodes[0].released_touching);
        assert!(episodes[0].mass_kg.is_none(), "no physics authoring");
    }
}

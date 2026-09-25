//! Dynamic robots (design-rl-dynamics.md): a robot declared dynamic is
//! simulated, under a physics bake, as an articulated body — every link
//! with geometry a rigid body weighing what its model says, every joint
//! a force-capped servo following the planned motion, the base a
//! kinematic mirror on its stand or vehicle. The declaration is the
//! gripper drive of G3 widened to the whole robot; without a physics
//! backend it is inert and the robot moves exactly as planned.
//!
//! The servo is the industrial cascade: a position loop in the rollout
//! turns the command into an approach velocity — the command's own rate
//! fed forward, plus [`APPROACH_GAIN`] per second times the command
//! error, clamped to the joint's rated speed — and the engine's motor
//! closes the velocity loop under the force cap (`damping` is its gain).
//! No position spring: a spring stiff enough to hold a load to
//! sub-milliradians asks for approach speeds the cap cannot brake, and a
//! joint knocked off its target — a collision, the hand-back after a
//! torque episode — then oscillates around it for good. The velocity
//! loop brakes at the cap and comes back at the rated speed instead. The
//! feedforward is what keeps a move's lag to the residual, not the whole
//! approach: a real drive's velocity feedforward, with no model
//! knowledge. What remains under load is the impulse solver's sag (a
//! couple of milliradians on a loaded shoulder, whatever the loop gains),
//! and a bounded integral trim takes that out: [`TRIM_GAIN`] on the
//! error, with [`TRIM_AUTHORITY_REVOLUTE`] (or the prismatic authority)
//! to spend — a few milliradians of reach, so it can neither wind up
//! against a stall (a
//! finger on its part, a joint on its stop) nor overshoot on release.
//! An unbounded integrator did both (measured: the Franka's fingers rang
//! ±5 mm around a close, and its arm overshot every ramp). Gravity
//! compensation is
//! not a simulator feature: it is an opt-in of the `Torque` control (a
//! torque interface whose firmware compensates), and the model's gravity
//! torques are readable (`gravity_torques`) for any controller a user
//! builds.

use crate::{Scene, SceneError};

/// One joint's servo: the force cap (N·m, N), the velocity-loop gain the
/// engine's motor runs with, the rated speed the approach is clamped
/// to, and the reflected drive inertia.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointServo {
    pub max_force: f64,
    pub damping: f64,
    pub max_velocity: f64,
    /// Reflected drive inertia added to the driven link about the joint
    /// axis (kg·m²; kg for a prismatic joint) — a geared motor's rotor
    /// seen through the gear ratio squared, and what keeps a light wrist
    /// link from drooping under an impulse-solved motor (the solver's
    /// position error scales with `torque · dt / inertia`).
    pub armature: f64,
}

/// The declaration (`set_robot_dynamics`): the overrides and the mass
/// floor, plus the servo resolved per actuated joint.
#[derive(Debug, Clone)]
pub struct RobotDynamics {
    pub max_force: Option<f64>,
    pub damping: Option<f64>,
    pub armature: Option<f64>,
    /// Mass floor (kg) for a link without stated inertials, whose mass
    /// then comes from its collision shape at the default density — a
    /// mesh-less frame link must still weigh something for the joint
    /// solver.
    pub mass_floor: f64,
    /// Resolved servo per entry of the model's `actuated_joints`.
    pub servos: Vec<JointServo>,
    /// Whether the base is a free rigid body under physics (a legged
    /// machine standing on its feet, a multirotor) rather than a
    /// kinematic mirror on its stand or vehicle. `None` derives it: a
    /// world-scope bake floats walkers and aircraft, the declared scope
    /// floats nothing (design-world-physics.md §3.3).
    pub floating: Option<bool>,
}

/// Default mass floor per dynamic link (kg); see [`RobotDynamics::mass_floor`].
pub const DEFAULT_MASS_FLOOR: f64 = 0.2;

/// Force cap a world-scope bake gives a joint whose model states no
/// effort limit (N·m revolute, N prismatic): generous enough to hold any
/// arm the catalog carries, and flagged in the plan so nobody mistakes it
/// for a datasheet figure. The declared scope refuses such a model.
pub const DEFAULT_WORLD_CAP_REVOLUTE: f64 = 200.0;
pub const DEFAULT_WORLD_CAP_PRISMATIC: f64 = 2000.0;

/// Default reflected drive inertia: 0.1 kg·m² about a revolute joint
/// (a small geared servo), 1 kg along a prismatic one (a belt or screw
/// driven slide; a gantry's ball screw reflects tens of kilograms —
/// declare it). The revolute one is added to a link's tensor
/// isotropically, the prismatic one to its mass (rollout.rs).
pub const DEFAULT_ARMATURE_REVOLUTE: f64 = 0.1;
pub const DEFAULT_ARMATURE_PRISMATIC: f64 = 1.0;

/// Approach gain (1/s): command error → approach velocity, before the
/// rated-speed clamp. 20/s is a 50 ms time constant, inside a 100 Hz
/// scan's stability margin (`dt · gain < 1`) and brake-feasible for an
/// arm whose cap decelerates it at a few tens of rad/s² — 40/s made a
/// light wrist under a stepwise command (0.05 rad every 50 ms) overshoot
/// what its cap could brake, step after step.
pub const APPROACH_GAIN: f64 = 20.0;

/// Gain of the integral trim (1/s²): critically damped against the
/// approach gain (`gain² / 4`), a 0.1 s time constant for the trim to
/// take the solver's sag over.
pub const TRIM_GAIN: f64 = APPROACH_GAIN * APPROACH_GAIN / 4.0;

/// The integral trim's authority: the most it may add to the approach
/// velocity — 5 mrad (0.5 mm) of standing error's worth at the approach
/// gain, so a stall can wind it no further than that.
pub const TRIM_AUTHORITY_REVOLUTE: f64 = 0.1;
pub const TRIM_AUTHORITY_PRISMATIC: f64 = 0.01;

/// A stated effort limit above this many times the model's weight (N·m
/// per kg·m/s², or N per kg·m/s²) is no datasheet figure but a "no
/// limit" convention (the Isaac Franka states 100 kN·m on 18 kg): the
/// world scope defaults the cap as if none were stated, the declared
/// scope asks for `max_force`. A hundredfold: the largest arms lift a
/// few times their own weight at a few metres.
pub const IMPLAUSIBLE_EFFORT_PER_WEIGHT: f64 = 100.0;

/// Below this fraction of the link's shape-derived tensor (at its stated
/// mass), a stated inertia is taken for a modelling slip and the shape's
/// is used instead. A hundredth: a real link's tensor is within a small
/// factor of its bounding shape's; the Isaac Franka's are four orders
/// under.
pub const INERTIA_FLOOR: f64 = 0.01;

fn default_armature(kind: botrail_physics::JointKind) -> f64 {
    match kind {
        botrail_physics::JointKind::Prismatic => DEFAULT_ARMATURE_PRISMATIC,
        _ => DEFAULT_ARMATURE_REVOLUTE,
    }
}

/// Default velocity-loop gain: the cap is reached at a velocity error
/// of 2 mrad/s (revolute) or 0.2 mm/s (prismatic) — a stiff loop, which
/// is what keeps a load's droop under a milliradian with no integral
/// term in the position loop (a joint creeps at `torque / damping`, and
/// the approach gain turns that creep into a standing error).
fn default_damping(kind: botrail_physics::JointKind, max_force: f64) -> f64 {
    match kind {
        botrail_physics::JointKind::Prismatic => max_force / 0.0002,
        _ => max_force / 0.002,
    }
}

/// Rated speed when the model states none: 4 rad/s, 0.1 m/s.
fn default_max_velocity(kind: botrail_physics::JointKind) -> f64 {
    match kind {
        botrail_physics::JointKind::Prismatic => 0.1,
        _ => 4.0,
    }
}

impl Scene {
    /// Declares robot `robot` dynamic under physics bakes (or not, with
    /// `dynamic = false`). Every actuated joint gets a force-capped servo:
    /// `max_force` (N·m / N; default each joint's URDF effort limit, which
    /// must then exist) and `damping`, the velocity loop's gain (default
    /// the cap per 0.02 rad/s or 2 mm/s of error); the joint's velocity
    /// limit is its rated speed. `mass_floor` is the least mass a link
    /// without inertials weighs; `armature` the reflected drive inertia
    /// per joint (default 0.1 kg·m² revolute, 10 kg prismatic).
    pub fn set_robot_dynamics(
        &mut self,
        robot: usize,
        dynamic: bool,
        max_force: Option<f64>,
        damping: Option<f64>,
        mass_floor: Option<f64>,
        armature: Option<f64>,
    ) -> Result<(), SceneError> {
        self.set_robot_dynamics_with(
            robot, dynamic, max_force, damping, mass_floor, armature, None,
        )
    }

    /// [`set_robot_dynamics`](Self::set_robot_dynamics) with the base's
    /// kind stated: `floating = Some(true)` makes the base a free rigid
    /// body under physics (it falls, tips, collapses with the rest),
    /// `Some(false)` keeps it a mirror on its stand or vehicle whatever
    /// the scope, `None` lets the bake derive it.
    #[allow(clippy::too_many_arguments)]
    pub fn set_robot_dynamics_with(
        &mut self,
        robot: usize,
        dynamic: bool,
        max_force: Option<f64>,
        damping: Option<f64>,
        mass_floor: Option<f64>,
        armature: Option<f64>,
        floating: Option<bool>,
    ) -> Result<(), SceneError> {
        let Some(sr) = self.robots.get(robot) else {
            return Err(SceneError::UnknownRobot(robot.to_string()));
        };
        if !dynamic {
            self.robots[robot].dynamics = None;
            return Ok(());
        }
        let (mut dynamics, _) =
            resolve_dynamics(&sr.model, max_force, damping, mass_floor, armature, false)?;
        dynamics.floating = floating;
        self.robots[robot].dynamics = Some(dynamics);
        Ok(())
    }

    /// Installs an already-resolved declaration (a world-scope bake's
    /// defaults for an undeclared robot, on the rollout's snapshot).
    pub(crate) fn install_robot_dynamics(&mut self, robot: usize, dynamics: RobotDynamics) {
        self.robots[robot].dynamics = Some(dynamics);
    }

    /// The dynamics declaration of a robot, if any.
    pub fn robot_dynamics(&self, robot: usize) -> Option<&RobotDynamics> {
        self.robots.get(robot).and_then(|r| r.dynamics.as_ref())
    }
}

/// Resolves a declaration against a model: validates the overrides and
/// builds one servo per actuated joint. `lenient` is the world scope's
/// reading — a joint without an effort limit gets
/// [`DEFAULT_WORLD_CAP_REVOLUTE`] / [`DEFAULT_WORLD_CAP_PRISMATIC`]
/// instead of an error; the returned flag says whether that happened.
pub(crate) fn resolve_dynamics(
    model: &botrail_model::RobotModel,
    max_force: Option<f64>,
    damping: Option<f64>,
    mass_floor: Option<f64>,
    armature: Option<f64>,
    lenient: bool,
) -> Result<(RobotDynamics, bool), SceneError> {
    for (label, value) in [
        ("max_force", max_force),
        ("damping", damping),
        ("mass_floor", mass_floor),
        ("armature", armature),
    ] {
        if let Some(v) = value {
            if !(v.is_finite() && v >= 0.0) || (label != "armature" && v == 0.0) {
                return Err(SceneError::BadDynamics(format!(
                    "{label} must be positive, got {v}"
                )));
            }
        }
    }
    let mut defaulted = false;
    let mut servos = Vec::with_capacity(model.actuated_joints.len());
    // What the model weighs, for the effort plausibility bound (a link
    // without inertials counts the floor).
    let weight = 9.81
        * model
            .links
            .iter()
            .map(|l| {
                l.inertial
                    .as_ref()
                    .map_or(mass_floor.unwrap_or(DEFAULT_MASS_FLOOR), |i| i.mass)
            })
            .sum::<f64>();
    for &ji in &model.actuated_joints {
        let kind = crate::grasp::joint_kind(model, ji);
        let limits = model.joints[ji].limits.as_ref();
        let cap = match max_force {
            Some(f) => f,
            None => {
                let effort = limits.map_or(0.0, |l| l.effort);
                if effort.is_finite()
                    && effort > 0.0
                    && effort <= IMPLAUSIBLE_EFFORT_PER_WEIGHT * weight
                {
                    effort
                } else if lenient {
                    defaulted = true;
                    match kind {
                        botrail_physics::JointKind::Prismatic => DEFAULT_WORLD_CAP_PRISMATIC,
                        _ => DEFAULT_WORLD_CAP_REVOLUTE,
                    }
                } else if effort > 0.0 {
                    return Err(SceneError::BadDynamics(format!(
                        "joint `{}` states an effort limit of {effort} that is no datasheet \
                         figure ({:.0}× the model's weight) — pass max_force=",
                        model.joints[ji].name,
                        effort / weight.max(1e-9)
                    )));
                } else {
                    return Err(SceneError::BadDynamics(format!(
                        "joint `{}` declares no effort limit — pass max_force=",
                        model.joints[ji].name
                    )));
                }
            }
        };
        let rated = limits
            .map(|l| l.velocity)
            .filter(|v| v.is_finite() && *v > 0.0)
            .unwrap_or_else(|| default_max_velocity(kind));
        servos.push(JointServo {
            max_force: cap,
            damping: damping.unwrap_or_else(|| default_damping(kind, cap)),
            max_velocity: rated,
            armature: armature.unwrap_or_else(|| default_armature(kind)),
        });
    }
    Ok((
        RobotDynamics {
            max_force,
            damping,
            armature,
            mass_floor: mass_floor.unwrap_or(DEFAULT_MASS_FLOOR),
            servos,
            floating: None,
        },
        defaulted,
    ))
}

#[cfg(test)]
pub(crate) mod tests_support {
    use crate::seq::{Action, Condition, Sequence, Step};
    use crate::Scene;
    use botrail_model::{Geometry, RobotModel};
    use nalgebra::{Isometry3, Vector3};
    use std::sync::Arc;

    pub const ARM6: &str = include_str!("../../../examples/assets/simple_arm.urdf");
    pub const READY: [f64; 6] = [0.0, 0.6, 0.8, 0.0, 0.5, 0.0];

    pub fn rapier() -> Option<Box<dyn botrail_physics::PhysicsBackend>> {
        Some(Box::new(botrail_physics_rapier::RapierBackend::new()))
    }

    pub fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    /// The six-axis arm at READY over a floor, declared dynamic, with a
    /// `hold` wait and a `lift` ramp of the shoulder.
    pub fn arm(dynamic: bool) -> Scene {
        let mut scene = Scene::new(Arc::new(RobotModel::from_urdf_str(ARM6).unwrap()));
        scene.set_joint_positions(READY.to_vec()).unwrap();
        scene
            .add_obstacle(
                "floor",
                Geometry::Box {
                    size: Vector3::new(3.0, 3.0, 0.1),
                },
                Isometry3::translation(0.0, 0.0, -0.06),
            )
            .unwrap();
        if dynamic {
            scene
                .set_robot_dynamics(0, true, None, None, None, None)
                .unwrap();
        }
        scene.upsert_sequence(Sequence {
            name: "hold".into(),
            steps: vec![step("wait", vec![], Condition::Elapsed { seconds: 1.0 })],
        });
        scene.upsert_sequence(Sequence {
            name: "lift".into(),
            steps: vec![
                step(
                    "up",
                    vec![Action::StartRamp {
                        robot: None,
                        targets: vec![("shoulder_lift".into(), 1.0)],
                        duration: 1.0,
                    }],
                    Condition::Done,
                ),
                step("settle", vec![], Condition::Elapsed { seconds: 0.5 }),
            ],
        });
        scene
    }
}

#[cfg(test)]
mod tests {
    use super::tests_support::*;
    use super::*;
    use crate::rollout::RolloutOptions;
    use botrail_model::RobotModel;
    use std::sync::Arc;

    #[test]
    fn the_declaration_resolves_motors_from_the_effort_limits() {
        let scene = arm(true);
        let d = scene.robot_dynamics(0).unwrap();
        assert_eq!(d.servos.len(), 6);
        // Caps and rated speeds straight from the URDF limits.
        assert_eq!(d.servos[0].max_force, 50.0);
        assert_eq!(d.servos[2].max_force, 30.0);
        assert_eq!(d.servos[5].max_force, 10.0);
        assert_eq!(d.servos[0].max_velocity, 2.0);
        assert_eq!(d.servos[0].damping, 25000.0);
        assert_eq!(d.servos[0].armature, DEFAULT_ARMATURE_REVOLUTE);
        assert_eq!(d.mass_floor, DEFAULT_MASS_FLOOR);
        let mut scene = arm(false);
        assert!(scene.robot_dynamics(0).is_none());
        let err = scene
            .set_robot_dynamics(0, true, None, Some(-1.0), None, None)
            .unwrap_err()
            .to_string();
        assert!(err.contains("damping must be positive"), "{err}");
        scene
            .set_robot_dynamics(0, true, Some(20.0), None, Some(0.5), Some(0.0))
            .unwrap();
        assert!(scene
            .robot_dynamics(0)
            .unwrap()
            .servos
            .iter()
            .all(|s| s.armature == 0.0));
        assert!(scene
            .robot_dynamics(0)
            .unwrap()
            .servos
            .iter()
            .all(|s| s.max_force == 20.0));
        scene
            .set_robot_dynamics(0, false, None, None, None, None)
            .unwrap();
        assert!(scene.robot_dynamics(0).is_none());
        // A joint without an effort limit needs an explicit cap.
        let bare = r#"<robot name="bare">
            <link name="base"><visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link>
            <link name="arm"><visual><origin xyz="0 0 0.1"/><geometry><box size="0.05 0.05 0.2"/></geometry></visual></link>
            <joint name="j" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" velocity="1"/></joint>
        </robot>"#;
        let mut scene = Scene::new(Arc::new(RobotModel::from_urdf_str(bare).unwrap()));
        let err = scene
            .set_robot_dynamics(0, true, None, None, None, None)
            .unwrap_err()
            .to_string();
        assert!(err.contains("no effort limit"), "{err}");
        scene
            .set_robot_dynamics(0, true, Some(5.0), None, None, None)
            .unwrap();
        // And the project carries the declaration.
        let project = scene.to_project();
        let back = Scene::from_project(&project).unwrap();
        let d = back.robot_dynamics(0).unwrap();
        assert_eq!((d.max_force, d.mass_floor), (Some(5.0), DEFAULT_MASS_FLOOR));
    }

    /// The world's tail: a part still falling when the program ends lands
    /// and comes to rest before the bake does, the cap — the programs'
    /// time — not cutting it; without a tail the bake ends mid-flight;
    /// and a program still waiting at the cap times out, tail or not.
    #[test]
    fn a_settle_tail_runs_the_world_to_rest_after_the_programs_end() {
        let drop = |wait: f64| {
            let mut scene = Scene::empty();
            scene
                .add_obstacle(
                    "part",
                    Geometry::Box {
                        size: Vector3::new(0.1, 0.1, 0.1),
                    },
                    Isometry3::translation(0.0, 0.0, 0.55),
                )
                .unwrap();
            scene
                .set_obstacle_physics(
                    "part",
                    Some(botrail_physics::BodyProps {
                        mass: Some(0.5),
                        ..botrail_physics::BodyProps::dynamic()
                    }),
                )
                .unwrap();
            scene.upsert_sequence(Sequence {
                name: "drop".into(),
                steps: vec![step("wait", vec![], Condition::Elapsed { seconds: wait })],
            });
            scene
        };
        let options = |settle: f64| RolloutOptions {
            max_duration: 0.2,
            physics: Some(crate::rollout::PhysicsOptions {
                settle,
                ..crate::rollout::PhysicsOptions::world()
            }),
            ..Default::default()
        };
        let cut = drop(0.1)
            .simulate_sequences_with(&["drop"], &options(0.0), rapier())
            .unwrap();
        assert!((cut.duration - 0.1).abs() < 1e-6, "{}", cut.duration);
        let at = |tl: &crate::rollout::SequenceTimeline, t: f64| {
            let track = tl.objects.iter().find(|o| o.name == "part").unwrap();
            crate::rollout::SequenceTimeline::object_pose(track, &[], t)
                .unwrap()
                .translation
                .z
        };
        let mid = at(&cut, cut.duration);
        assert!(mid > 0.45, "still falling at the program's end: {mid}");

        let tailed = drop(0.1)
            .simulate_sequences_with(&["drop"], &options(3.0), rapier())
            .unwrap();
        assert!(
            tailed.duration > 0.4 && tailed.duration < 3.1,
            "landed and rested within the tail: {}",
            tailed.duration
        );
        let end = at(&tailed, tailed.duration);
        assert!((end - 0.05).abs() < 0.01, "at rest on the ground: {end}");

        let stalled = drop(60.0).simulate_sequences_with(&["drop"], &options(3.0), rapier());
        assert!(
            matches!(stalled, Err(crate::rollout::SeqError::Timeout { .. })),
            "{stalled:?}"
        );
    }

    #[test]
    fn a_dynamic_arm_holds_ready_under_gravity_and_bakes_per_tick() {
        let scene = arm(true);
        let options = RolloutOptions::default();
        let tl = scene
            .simulate_sequences_with(&["hold"], &options, rapier())
            .unwrap();
        let track = &tl.robots[0].trajectory;
        // Tick by tick: a sample every scan.
        assert!(track.times.len() >= 99, "{} samples", track.times.len());
        let worst = track
            .positions
            .iter()
            .flat_map(|q| q.iter().zip(READY.iter()).map(|(a, b)| (a - b).abs()))
            .fold(0.0, f64::max);
        assert!(
            worst < 0.01,
            "the motors hold READY to within {worst:.4} rad"
        );
        // The integral trim takes the solver's sag over: the second half
        // of the hold sits within a fraction of a milliradian.
        let settled = track
            .times
            .iter()
            .zip(&track.positions)
            .filter(|(t, _)| **t >= 0.5)
            .flat_map(|(_, q)| q.iter().zip(READY.iter()).map(|(a, b)| (a - b).abs()))
            .fold(0.0, f64::max);
        assert!(settled < 5e-4, "settled droop {settled:.5} rad");
        // The lane is the engine's state, not the plan: before the trim
        // took the sag out, the arm measurably rested below the command.
        assert!(
            track.positions.iter().any(|q| q
                .iter()
                .zip(READY.iter())
                .any(|(a, b)| (a - b).abs() > 1e-5)),
            "no sag ever read back"
        );
        // Without the declaration the same bake is the kinematic one:
        // exactly READY, one pre-baked span.
        let plain = arm(false)
            .simulate_sequences_with(&["hold"], &options, rapier())
            .unwrap();
        assert!(plain.robots[0]
            .trajectory
            .positions
            .iter()
            .all(|q| q.iter().zip(READY.iter()).all(|(a, b)| a == b)));
    }

    #[test]
    fn a_dynamic_arm_follows_a_ramp() {
        let scene = arm(true);
        let options = RolloutOptions::default();
        let tl = scene
            .simulate_sequences_with(&["lift"], &options, rapier())
            .unwrap();
        let track = &tl.robots[0].trajectory;
        // During the ramp the shoulder trails its command by less than
        // 0.1 rad; settled, it sits within 0.02 rad of the target.
        for (t, q) in track.times.iter().zip(&track.positions) {
            let command = 0.6 + 0.4 * (t / 1.0).min(1.0);
            assert!(
                (q[1] - command).abs() < 0.1,
                "t={t:.2}: q1={:.3} vs {command:.3}",
                q[1]
            );
        }
        let last = track.positions.last().unwrap();
        assert!((last[1] - 1.0).abs() < 0.02, "settled at {:.4}", last[1]);
        assert!(
            (tl.duration - 1.5).abs() < 0.011,
            "duration {}",
            tl.duration
        );
    }

    #[test]
    fn dynamic_bakes_are_bit_identical() {
        let scene = arm(true);
        let options = RolloutOptions::default();
        let a = scene
            .simulate_sequences_with(&["lift"], &options, rapier())
            .unwrap();
        let b = scene
            .simulate_sequences_with(&["lift"], &options, rapier())
            .unwrap();
        assert_eq!(
            a.robots[0].trajectory.positions,
            b.robots[0].trajectory.positions
        );
        // The live rollout ticks the same physics.
        let mut live = scene.open_rollout(&["lift"], &options, rapier()).unwrap();
        while !live.finished() {
            live.tick().unwrap();
        }
        let live_tl = live.finish();
        assert_eq!(
            a.robots[0].trajectory.positions,
            live_tl.robots[0].trajectory.positions
        );
    }

    #[test]
    fn a_torque_moves_a_driven_joint_and_a_command_takes_it_back() {
        let scene = arm(true);
        let options = RolloutOptions::default();
        let mut live = scene.open_rollout(&["hold"], &options, rapier()).unwrap();
        assert_eq!(live.view().is_dynamic(0), Some(true));
        live.drive(0, None, None).unwrap();
        // Positive torque on the base yaw (no gravity about z): it turns.
        live.command_torque(0, &[(0, 20.0)], false).unwrap();
        for _ in 0..20 {
            live.tick().unwrap();
        }
        let q = live.joint_positions(0).unwrap().to_vec();
        let v = live.joint_velocities(0).unwrap();
        assert!(q[0] > 0.01 && v[0] > 0.1, "q0={:.4} v0={:.3}", q[0], v[0]);
        // A position command hands the joint back to its motor: it holds.
        let mut hold = q.clone();
        hold[0] = q[0];
        live.command(0, &hold).unwrap();
        for _ in 0..50 {
            live.tick().unwrap();
        }
        let v = live.joint_velocities(0).unwrap();
        let q_end = live.joint_positions(0).unwrap().to_vec();
        assert!(v[0].abs() < 0.05, "settled, v0={:.3}", v[0]);
        assert!(
            (q_end[0] - hold[0]).abs() < 0.05,
            "held at {:.4} vs {:.4}",
            q_end[0],
            hold[0]
        );
        // Errors: a joint the drive does not own, and a kinematic robot.
        live.undrive(0);
        assert!(live.command_torque(0, &[(0, 1.0)], false).is_err());
        let mut plain = arm(false)
            .open_rollout(&["hold"], &options, rapier())
            .unwrap();
        plain.drive(0, None, None).unwrap();
        let err = plain
            .command_torque(0, &[(0, 1.0)], false)
            .unwrap_err()
            .to_string();
        assert!(err.contains("not dynamic"), "{err}");
        assert!(plain.gravity_torques(0).is_none());
    }

    /// The world scope powers a robot a program drives; an external drive
    /// is a driver too (design-rl-tabletop.md G12). The arm a `hold`
    /// program leaves alone is unpowered and folds — unless a driver
    /// takes it: its servos switch on where it stands, and it holds; taken
    /// after it fell, it holds where it fell (no snap back to the
    /// authored pose).
    #[test]
    fn an_external_drive_powers_an_unpowered_arm_under_the_world_scope() {
        use crate::rollout::PhysicsOptions;
        let scene = arm(true);
        let options = RolloutOptions {
            physics: Some(PhysicsOptions::world()),
            ..Default::default()
        };
        let mut idle = scene.open_rollout(&["hold"], &options, rapier()).unwrap();
        for _ in 0..150 {
            idle.tick().unwrap();
        }
        let fallen = idle.joint_positions(0).unwrap().to_vec();
        let drop = (fallen[1] - READY[1]).abs();
        assert!(
            drop > 0.1,
            "an unpowered shoulder should fold, moved {drop:.3}"
        );

        let mut live = scene.open_rollout(&["hold"], &options, rapier()).unwrap();
        live.drive(0, None, None).unwrap();
        live.command(0, &READY).unwrap();
        for _ in 0..150 {
            live.tick().unwrap();
        }
        let q = live.joint_positions(0).unwrap().to_vec();
        let err = q
            .iter()
            .zip(READY.iter())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max);
        assert!(
            err < 0.02,
            "the driven arm drifted {err:.4} rad from READY: {q:?}"
        );

        let mut late = scene.open_rollout(&["hold"], &options, rapier()).unwrap();
        for _ in 0..150 {
            late.tick().unwrap();
        }
        let before = late.joint_positions(0).unwrap().to_vec();
        late.drive(0, None, None).unwrap();
        for _ in 0..30 {
            late.tick().unwrap();
        }
        let after = late.joint_positions(0).unwrap().to_vec();
        let moved = after
            .iter()
            .zip(before.iter())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max);
        assert!(
            moved < 0.05,
            "a late drive should hold where the arm fell, moved {moved:.3}"
        );
    }

    /// The gravity-torque read-out is exact against the engine: applied
    /// as raw torques with the motors off, it holds the arm where it is
    /// for a second; the same through the control's compensation with a
    /// zero action.
    #[test]
    fn gravity_torques_hold_the_arm_with_the_motors_off() {
        let scene = arm(true);
        let options = RolloutOptions::default();
        let mut live = scene.open_rollout(&["hold"], &options, rapier()).unwrap();
        live.drive(0, None, None).unwrap();
        let g = live.gravity_torques(0).unwrap();
        assert_eq!(g.len(), 6);
        // A base yaw about the vertical carries no gravity load; the
        // shoulder carries the arm's weight.
        assert!(g[0].abs() < 1e-9, "{g:?}");
        assert!(g[1].abs() > 1.0, "{g:?}");
        let start = live.joint_positions(0).unwrap().to_vec();
        let torques: Vec<(usize, f64)> = g.iter().copied().enumerate().collect();
        for _ in 0..100 {
            live.command_torque(0, &torques, false).unwrap();
            live.tick().unwrap();
        }
        let q = live.joint_positions(0).unwrap();
        let drift = q
            .iter()
            .zip(&start)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max);
        assert!(
            drift < 0.02,
            "held by its own gravity torques: drift {drift:.4} rad"
        );
        // Without them the arm falls.
        let mut free = scene.open_rollout(&["hold"], &options, rapier()).unwrap();
        free.drive(0, None, None).unwrap();
        free.command_torque(0, &[(1, 0.0), (2, 0.0)], false)
            .unwrap();
        for _ in 0..100 {
            free.tick().unwrap();
        }
        let fell = free.joint_positions(0).unwrap()[1] - start[1];
        assert!(
            fell.abs() > 0.1,
            "motors off, no torque: the shoulder moved {fell:+.3}"
        );
        // The opt-in: zero raw torques on top of the compensation hold.
        let mut comp = scene.open_rollout(&["hold"], &options, rapier()).unwrap();
        comp.drive(0, None, None).unwrap();
        let zeros: Vec<(usize, f64)> = (0..6).map(|qi| (qi, 0.0)).collect();
        comp.command_torque(0, &zeros, true).unwrap();
        for _ in 0..100 {
            comp.tick().unwrap();
        }
        let q = comp.joint_positions(0).unwrap();
        let drift = q
            .iter()
            .zip(&start)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max);
        assert!(drift < 0.02, "compensated: drift {drift:.4} rad");
    }

    use crate::rollout::{PhysicsOptions, PhysicsScope, SequenceTimeline, TrackSpan};
    use crate::seq::{Action, Condition, Sequence};
    use botrail_model::Geometry;
    use nalgebra::{Isometry3, Vector3};

    /// A three-axis gantry, with the effort and speed limits a dynamic
    /// declaration needs, and a box on every link so each is a body.
    fn gantry(dynamic: bool) -> Scene {
        let urdf = r#"
        <robot name="gantry">
          <link name="base"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
          <link name="bridge"><collision><geometry><box size="0.05 0.05 0.05"/></geometry></collision></link>
          <link name="carriage"><collision><geometry><box size="0.05 0.05 0.05"/></geometry></collision></link>
          <link name="tool"><collision><geometry><box size="0.05 0.05 0.05"/></geometry></collision></link>
          <joint name="jx" type="prismatic">
            <parent link="base"/><child link="bridge"/>
            <axis xyz="1 0 0"/>
            <limit lower="-2" upper="2" effort="100" velocity="1"/>
          </joint>
          <joint name="jy" type="prismatic">
            <parent link="bridge"/><child link="carriage"/>
            <axis xyz="0 1 0"/>
            <limit lower="-2" upper="2" effort="100" velocity="1"/>
          </joint>
          <joint name="jz" type="prismatic">
            <parent link="carriage"/><child link="tool"/>
            <axis xyz="0 0 1"/>
            <limit lower="-2" upper="2" effort="100" velocity="1"/>
          </joint>
        </robot>"#;
        let mut scene = Scene::new(Arc::new(RobotModel::from_urdf_str(urdf).unwrap()));
        scene.set_joint_positions(vec![0.0, 0.0, 0.4]).unwrap();
        if dynamic {
            scene
                .set_robot_dynamics(0, true, None, None, None, None)
                .unwrap();
        }
        scene
    }

    /// A dynamic robot tracks a conveyed part: the per-tick solve is its
    /// motors' command, the servo follows, and the tool rides the belt as
    /// closely as the kinematic robot's does.
    #[test]
    fn a_dynamic_robot_tracks_a_conveyed_part() {
        fn chase_scene(dynamic: bool) -> Scene {
            let mut scene = gantry(dynamic);
            scene
                .add_obstacle(
                    "part",
                    Geometry::Box {
                        size: Vector3::new(0.05, 0.05, 0.05),
                    },
                    // Clear of the gantry's own links: a dynamic link
                    // touching the part inside the belt zone would be
                    // belt-driven itself.
                    Isometry3::translation(0.3, 0.6, 0.0),
                )
                .unwrap();
            scene.upsert_device(crate::seq::Device {
                name: "belt".into(),
                kind: crate::seq::DeviceKind::Conveyor {
                    zone_pose: Isometry3::translation(0.8, 0.6, 0.0),
                    zone_size: Vector3::new(3.0, 0.4, 0.4),
                    velocity: Vector3::new(0.2, 0.0, 0.0),
                    running: false,
                },
            });
            scene.upsert_sequence(Sequence {
                name: "chase".into(),
                steps: vec![
                    step(
                        "latch",
                        vec![
                            Action::Device {
                                device: "belt".into(),
                                command: crate::seq::DeviceCommand::Start,
                            },
                            Action::Track {
                                robot: None,
                                object: "part".into(),
                                link: Some("tool".into()),
                                group: None,
                            },
                        ],
                        Condition::Elapsed { seconds: 2.0 },
                    ),
                    step(
                        "let_go",
                        vec![Action::Untrack {
                            robot: None,
                            group: None,
                        }],
                        Condition::Elapsed { seconds: 0.5 },
                    ),
                ],
            });
            scene
        }
        // The tool's position at `t`, from the baked joints.
        fn tool(scene: &Scene, tl: &SequenceTimeline, t: f64) -> Vector3<f64> {
            let track = &tl.robots[0].trajectory;
            let k = track
                .times
                .iter()
                .position(|&s| s >= t - 1e-9)
                .expect("sample at t");
            let tip = scene.robot().link_index("tool").unwrap();
            scene.fk_for(0, &track.positions[k]).unwrap()[tip]
                .translation
                .vector
        }
        let kinematic = chase_scene(false);
        let plain = kinematic
            .simulate_sequences_with(&["chase"], &RolloutOptions::default(), rapier())
            .unwrap();
        let dynamic = chase_scene(true);
        let servoed = dynamic
            .simulate_sequences_with(&["chase"], &RolloutOptions::default(), rapier())
            .unwrap();
        assert!(
            (servoed.duration - 2.5).abs() < 0.011,
            "{}",
            servoed.duration
        );
        // Baked per tick: the servo's read-back, every scan.
        assert!(servoed.robots[0].trajectory.times.len() > 250);
        // Both tools rode the belt: 0.2 m/s for the tracked stretch.
        let ride =
            |scene: &Scene, tl: &SequenceTimeline| tool(scene, tl, 2.0) - tool(scene, tl, 0.5);
        let planned = ride(&kinematic, &plain);
        let actual = ride(&dynamic, &servoed);
        assert!((planned.x - 0.3).abs() < 1e-3, "kinematic ride {planned:?}");
        assert!(
            (actual - planned).norm() < 0.01,
            "servoed ride {actual:?} vs planned {planned:?}"
        );
        // And rode it in step: the servoed tool is never far behind the
        // planned one.
        for t in [1.0, 1.5, 2.0] {
            let gap = tool(&dynamic, &servoed, t) - tool(&kinematic, &plain, t);
            assert!(gap.norm() < 0.01, "at t = {t}: {gap:?}");
        }
        // Released, the tool stays where the chase left it.
        let after = tool(&dynamic, &servoed, servoed.duration) - tool(&dynamic, &servoed, 2.0);
        assert!(after.norm() < 0.005, "drift after untrack {after:?}");
    }

    /// A stated inertia four orders under the link's shape (the Isaac
    /// Franka's) is floored to the shape's: a finger on such a hand then
    /// follows its close instead of crawling, and a sane model's tensor
    /// is left alone.
    #[test]
    fn a_degenerate_stated_inertia_is_floored_to_the_shape() {
        let box_inertia = |m: f64, a: f64, b: f64, c: f64| {
            nalgebra::Matrix3::from_diagonal(&Vector3::new(
                m / 12.0 * (b * b + c * c),
                m / 12.0 * (a * a + c * c),
                m / 12.0 * (a * a + b * b),
            ))
        };
        let parts = vec![(
            parry3d_f64::math::Pose::identity(),
            parry3d_f64::shape::SharedShape::cuboid(0.05, 0.05, 0.05),
        )];
        let sane = box_inertia(0.5, 0.1, 0.1, 0.1);
        let (kept, floored) = super::super::rollout::floored_inertia_for_test(&sane, 0.5, &parts);
        assert!(!floored);
        assert_eq!(kept, sane);
        let degenerate = sane * 1e-4;
        let (fixed, floored) =
            super::super::rollout::floored_inertia_for_test(&degenerate, 0.5, &parts);
        assert!(floored);
        assert!((fixed.trace() - sane.trace()).abs() < 1e-9, "{fixed}");
        // The Franka's hand: a finger closing on a hand that states a
        // 1e-7 tensor follows its ramp only with the floor.
        let urdf = |hand_inertia: f64| {
            format!(
                r#"
        <robot name="hand">
          <link name="base"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
          <link name="hand">
            <inertial><mass value="0.56"/><inertia ixx="{i}" iyy="{i}" izz="{i}" ixy="0" ixz="0" iyz="0"/></inertial>
            <collision><geometry><box size="0.2 0.06 0.06"/></geometry></collision>
          </link>
          <link name="finger">
            <inertial><mass value="0.014"/><inertia ixx="4e-10" iyy="4e-10" izz="1e-10" ixy="0" ixz="0" iyz="0"/></inertial>
            <collision><origin xyz="0 0 -0.03"/><geometry><box size="0.02 0.015 0.05"/></geometry></collision>
          </link>
          <joint name="wrist" type="revolute">
            <parent link="base"/><child link="hand"/>
            <origin xyz="0 0 0.3"/><axis xyz="0 0 1"/>
            <limit lower="-3" upper="3" effort="12" velocity="2"/>
          </joint>
          <joint name="finger_joint" type="prismatic">
            <parent link="hand"/><child link="finger"/>
            <origin xyz="0.08 0 -0.03"/><axis xyz="0 1 0"/>
            <limit lower="0" upper="0.04" effort="20" velocity="0.2"/>
          </joint>
        </robot>"#,
                i = hand_inertia
            )
        };
        for hand_inertia in [2e-7, 1e-3] {
            let mut scene = Scene::new(Arc::new(
                RobotModel::from_urdf_str(&urdf(hand_inertia)).unwrap(),
            ));
            scene.set_joint_positions(vec![0.0, 0.039]).unwrap();
            scene
                .set_robot_dynamics(0, true, None, None, None, None)
                .unwrap();
            scene.upsert_sequence(Sequence {
                name: "close".into(),
                steps: vec![
                    step(
                        "close",
                        vec![Action::StartRamp {
                            robot: None,
                            targets: vec![("finger_joint".into(), 0.029)],
                            duration: 0.4,
                        }],
                        Condition::Done,
                    ),
                    step("hold", vec![], Condition::Elapsed { seconds: 0.4 }),
                ],
            });
            let tl = scene
                .simulate_sequences_with(&["close"], &RolloutOptions::default(), rapier())
                .unwrap();
            let track = &tl.robots[0].trajectory;
            let at = |t: f64| {
                let k = track.times.iter().position(|&s| s >= t - 1e-9).unwrap();
                track.positions[k][1]
            };
            // Within a millimetre of the ramp at its end, and settled.
            assert!(
                (at(0.4) - 0.029).abs() < 1e-3,
                "hand inertia {hand_inertia}: {}",
                at(0.4)
            );
            assert!(
                (at(0.8) - 0.029).abs() < 2e-4,
                "hand inertia {hand_inertia}: {}",
                at(0.8)
            );
        }
    }

    /// A grasp fired by a program on a dynamic robot with no gripper drive
    /// means what it says: the part rides the hand (a mirror the FK
    /// places) until detach hands it back to physics, where it falls.
    #[test]
    fn a_grasp_mid_program_rides_the_hand_and_lets_go_under_physics() {
        let mut scene = arm(false);
        let tip = scene.robots()[0].model.links.len() - 1;
        let tcp = scene.link_poses_for(0)[tip];
        scene
            .add_obstacle(
                "part",
                Geometry::Box {
                    size: Vector3::new(0.05, 0.05, 0.05),
                },
                tcp * Isometry3::translation(0.0, 0.0, 0.05),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "pick".into(),
            steps: vec![
                step(
                    "grasp",
                    vec![Action::Attach {
                        robot: None,
                        object: "part".into(),
                        link: None,
                        touch_links: None,
                        group: None,
                    }],
                    Condition::Immediately,
                ),
                step(
                    "up",
                    vec![Action::StartRamp {
                        robot: None,
                        targets: vec![("shoulder_lift".into(), 1.0)],
                        duration: 1.0,
                    }],
                    Condition::Done,
                ),
                step(
                    "let_go",
                    vec![Action::Detach {
                        object: "part".into(),
                    }],
                    Condition::Elapsed { seconds: 1.0 },
                ),
            ],
        });
        let tl = scene
            .simulate_sequences_with(&["pick"], &world_unpowered(), rapier())
            .unwrap();
        assert_eq!(tl.physics_scope, Some(PhysicsScope::World));
        let track = tl.objects.iter().find(|o| o.name == "part").unwrap();
        let fk = |t: f64| {
            let traj = &tl.robots[0].trajectory;
            let k = traj.times.iter().position(|&s| s >= t - 1e-9).unwrap();
            scene.fk_for(0, &traj.positions[k]).unwrap()
        };
        let grasp = scene.attachment("part").map(|a| a.grasp);
        assert!(grasp.is_none(), "the bake leaves the scene as authored");
        // Through the lift the part is where the hand carries it.
        let at = |t: f64| SequenceTimeline::object_pose(track, &[fk(t)], t).unwrap();
        let held = at(1.0);
        let hand = fk(1.0)[tip];
        let offset = hand.inverse() * held;
        assert!(
            (offset.translation.vector - Vector3::new(0.0, 0.0, 0.05)).norm() < 0.01,
            "part rode the hand at {:?}",
            offset.translation.vector
        );
        let moved = (held.translation.vector - at(0.05).translation.vector).norm();
        assert!(moved > 0.05, "the lift carried it {moved} m");
        // Let go, it is physics' again: a second later it has fallen.
        let dropped = at(tl.duration);
        assert!(
            dropped.translation.z < held.translation.z - 0.1,
            "fell from {} to {}",
            held.translation.z,
            dropped.translation.z
        );
    }

    /// A hand on a loose body (design-physics-pick.md): the drag spring
    /// pulls it to the target and lets go; a mirror is not held; an
    /// unpowered arm's link swings when pulled.
    #[test]
    fn a_dragged_box_follows_the_hand_and_a_mirror_is_not_held() {
        let mut scene = arm(false);
        scene
            .add_obstacle(
                "box",
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                Isometry3::translation(0.8, 0.0, 0.05),
            )
            .unwrap();
        let mut live = scene
            .open_physics_rollout(5.0, &world_unpowered(), rapier())
            .unwrap();
        // The floor slab is bolted (buried): no hand moves it.
        assert!(!live
            .drag("floor", Vector3::zeros(), Vector3::zeros())
            .unwrap());
        assert!(live
            .drag("nothing", Vector3::zeros(), Vector3::zeros())
            .is_err());
        // The box, by its top: pulled 0.3 m sideways at its own height.
        let grip = Vector3::new(0.0, 0.0, 0.05);
        let target = Vector3::new(0.8, 0.3, 0.1);
        assert!(live.drag("box", grip, target).unwrap());
        assert_eq!(live.dragging(), Some("box"));
        for _ in 0..120 {
            live.tick().unwrap();
        }
        let anchor = live.obstacle_pose("box").unwrap() * nalgebra::Point3::from(grip);
        let gap = (anchor.coords - target).norm();
        assert!(gap < 0.03, "the box's top sits {gap} m from the hand");
        // Let go: it stays put on the floor, at rest.
        live.release();
        assert_eq!(live.dragging(), None);
        for _ in 0..120 {
            live.tick().unwrap();
        }
        let pose = live.obstacle_pose("box").unwrap();
        assert!(
            (pose.translation.z - 0.05).abs() < 0.01,
            "z = {}",
            pose.translation.z
        );
        let speed = live.obstacle_velocity("box").unwrap().linear.norm();
        assert!(speed < 0.05, "still moving at {speed} m/s");
        // The unpowered arm has folded onto the floor by now; pulled up
        // and sideways by its wrist, the link comes along.
        let model = &scene.robots()[0].model;
        let wrist = model.link_index("wrist_3_link").unwrap();
        let name = format!("{}/{}", scene.robots()[0].name, model.links[wrist].name);
        let before = live.link_poses(0).unwrap()[wrist].translation.vector;
        assert!(live
            .drag(
                &name,
                Vector3::zeros(),
                before + Vector3::new(0.0, 0.3, 0.3)
            )
            .unwrap());
        for _ in 0..120 {
            live.tick().unwrap();
        }
        let after = live.link_poses(0).unwrap()[wrist].translation.vector;
        assert!(
            (after - before).norm() > 0.05,
            "the wrist moved {} m",
            (after - before).norm()
        );
        live.release();
    }

    // ============ world scope robots (design-world-physics.md W1) ============

    fn world_unpowered() -> RolloutOptions {
        RolloutOptions {
            physics: Some(PhysicsOptions::world()),
            ..Default::default()
        }
    }

    fn world_powered(powered: bool) -> RolloutOptions {
        RolloutOptions {
            physics: Some(PhysicsOptions {
                powered: Some(powered),
                ..PhysicsOptions::world()
            }),
            ..Default::default()
        }
    }

    fn within_limits(model: &RobotModel, q: &[f64], slack: f64) -> bool {
        model.actuated_joints.iter().enumerate().all(|(k, &ji)| {
            model.joints[ji]
                .limits
                .as_ref()
                .is_none_or(|l| q[k] >= l.lower - slack && q[k] <= l.upper + slack)
        })
    }

    /// A UR axis may turn a full circle: its ±2π limits are no stop to
    /// the engine (rapier's half-angle-sine limit would read them as a
    /// stop at zero and snap every pose back). The servo still holds.
    #[test]
    fn full_turn_joint_limits_do_not_snap_the_arm_to_zero() {
        let urdf = ARM6
            .replace(
                r#"lower="-2.2" upper="2.2""#,
                r#"lower="-6.2832" upper="6.2832""#,
            )
            .replace(
                r#"lower="-2.6" upper="2.6""#,
                r#"lower="-6.2832" upper="6.2832""#,
            );
        assert_ne!(urdf, ARM6);
        let mut scene = Scene::new(Arc::new(RobotModel::from_urdf_str(&urdf).unwrap()));
        scene.set_joint_positions(READY.to_vec()).unwrap();
        scene
            .set_robot_dynamics(0, true, None, None, None, None)
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "hold".into(),
            steps: vec![step("wait", vec![], Condition::Elapsed { seconds: 1.0 })],
        });
        let options = RolloutOptions {
            physics: Some(PhysicsOptions {
                ground: Some(0.0),
                ..Default::default()
            }),
            ..Default::default()
        };
        let tl = scene
            .simulate_sequences_with(&["hold"], &options, rapier())
            .unwrap();
        let worst = tl.robots[0]
            .trajectory
            .positions
            .iter()
            .flat_map(|q| q.iter().zip(READY.iter()).map(|(a, b)| (a - b).abs()))
            .fold(0.0, f64::max);
        assert!(worst < 0.02, "servo droop {worst}");
        // Unpowered it folds like any arm — no snap, no spin.
        let tl = scene
            .simulate_physics_with(2.0, &world_unpowered(), rapier())
            .unwrap();
        let q1 = &tl.robots[0].trajectory.positions[1];
        assert!(
            q1.iter()
                .zip(READY.iter())
                .all(|(a, b)| (a - b).abs() < 0.05),
            "first tick {q1:?}"
        );
        let q_end = tl.robots[0].trajectory.positions.last().unwrap();
        assert!(q_end.iter().all(|v| v.abs() < 7.0), "spun: {q_end:?}");
    }

    #[test]
    fn an_undeclared_arm_collapses_in_an_unpowered_world_bake() {
        let scene = arm(false);
        let tl = scene
            .simulate_physics_with(3.0, &world_unpowered(), rapier())
            .unwrap();
        assert_eq!(tl.physics_scope, Some(PhysicsScope::World));
        let track = &tl.robots[0];
        // The robot lane is the engine's, tick by tick.
        assert!(
            track.trajectory.times.len() > 200,
            "{}",
            track.trajectory.times.len()
        );
        let q_end = track.trajectory.positions.last().unwrap();
        // The shoulder folded well away from READY …
        assert!(
            (q_end[1] - READY[1]).abs() > 0.5,
            "shoulder at {}",
            q_end[1]
        );
        // … and every joint stayed inside its travel.
        let model = &scene.robots()[0].model;
        for q in &track.trajectory.positions {
            assert!(within_limits(model, q, 0.02), "{q:?}");
        }
        // It came to rest: the last tenth of a second barely moves.
        let n = track.trajectory.positions.len();
        let drift: f64 = (0..6)
            .map(|k| {
                (track.trajectory.positions[n - 1][k] - track.trajectory.positions[n - 11][k]).abs()
            })
            .fold(0.0, f64::max);
        assert!(drift < 0.05, "still swinging: {drift}");
        // The base stayed on its stand: no base track.
        assert!(track.base.is_none());
        // And the plan said so.
        let plan = scene.physics_plan(&PhysicsOptions::world());
        assert!(plan
            .rows
            .iter()
            .any(|r| r.name == "simple_arm" && r.kind == crate::physics_world::PlanKind::Robot));
    }

    #[test]
    fn a_powered_world_bake_holds_the_pose_the_declared_one_holds() {
        let scene = arm(false);
        let tl = scene
            .simulate_physics_with(1.0, &world_powered(true), rapier())
            .unwrap();
        let worst = tl.robots[0]
            .trajectory
            .positions
            .iter()
            .flat_map(|q| q.iter().zip(READY.iter()).map(|(a, b)| (a - b).abs()))
            .fold(0.0, f64::max);
        assert!(worst < 0.02, "servo droop {worst}");
        // A declared robot is powered by the program that drives it: the
        // ramp lands where it was told. A program that never moves it
        // (`hold` only waits) leaves it unpowered — it folds — as does no
        // program at all.
        let declared = arm(true);
        let driven = declared
            .simulate_sequences_with(&["lift"], &world_unpowered(), rapier())
            .unwrap();
        let q_end = driven.robots[0].trajectory.positions.last().unwrap();
        assert!((q_end[1] - 1.0).abs() < 0.02, "ramp landed at {}", q_end[1]);
        for (k, target) in READY.iter().enumerate().filter(|(k, _)| *k != 1) {
            assert!(
                (q_end[k] - target).abs() < 0.02,
                "joint {k} drooped to {}",
                q_end[k]
            );
        }
        let idle = declared
            .simulate_sequences_with(&["hold"], &world_unpowered(), rapier())
            .unwrap();
        let q_end = idle.robots[0].trajectory.positions.last().unwrap();
        assert!(
            (q_end[1] - READY[1]).abs() > 0.3,
            "undriven under a program, shoulder at {}",
            q_end[1]
        );
        let dropped = declared
            .simulate_physics_with(1.0, &world_unpowered(), rapier())
            .unwrap();
        let q_end = dropped.robots[0].trajectory.positions.last().unwrap();
        assert!(
            (q_end[1] - READY[1]).abs() > 0.3,
            "shoulder at {}",
            q_end[1]
        );
        // Powered on purpose, the idle program holds; unpowered on
        // purpose, the driving one folds.
        let held = declared
            .simulate_sequences_with(&["hold"], &world_powered(true), rapier())
            .unwrap();
        let worst = held.robots[0]
            .trajectory
            .positions
            .iter()
            .flat_map(|q| q.iter().zip(READY.iter()).map(|(a, b)| (a - b).abs()))
            .fold(0.0, f64::max);
        assert!(worst < 0.02, "powered droop {worst}");
        let cut = declared
            .simulate_sequences_with(&["lift"], &world_powered(false), rapier())
            .unwrap();
        let q_end = cut.robots[0].trajectory.positions.last().unwrap();
        assert!(
            (q_end[1] - READY[1]).abs() > 0.3,
            "shoulder at {}",
            q_end[1]
        );
    }

    #[test]
    fn an_explicitly_floating_arm_falls_off_its_stand() {
        let mut scene = arm(false);
        scene.set_robot_base_pose_for(0, Isometry3::translation(0.0, 0.0, 0.5));
        scene
            .set_robot_dynamics_with(0, true, None, None, None, None, Some(true))
            .unwrap();
        // Declared scope, with a ground: the declaration alone floats it.
        let options = RolloutOptions {
            physics: Some(PhysicsOptions {
                ground: Some(0.0),
                ..Default::default()
            }),
            ..Default::default()
        };
        let tl = scene
            .simulate_physics_with(2.0, &options, rapier())
            .unwrap();
        assert_eq!(tl.physics_scope, Some(PhysicsScope::Declared));
        let track = &tl.robots[0];
        let spans = track.base.as_ref().expect("a floating base has a track");
        assert!(spans.iter().any(|s| matches!(s, TrackSpan::Sampled { .. })));
        let base = SequenceTimeline::base_pose(track, tl.duration).unwrap();
        assert!(
            base.translation.z < 0.2,
            "base fell to z = {}",
            base.translation.z
        );
        assert!(
            base.translation.z > -0.05,
            "base under the ground: {}",
            base.translation.z
        );
        // Bit-identical run to run, base included.
        let again = scene
            .simulate_physics_with(2.0, &options, rapier())
            .unwrap();
        let b2 = SequenceTimeline::base_pose(&again.robots[0], tl.duration).unwrap();
        assert_eq!(base.translation.vector, b2.translation.vector);
        // Told not to float, it stays put.
        scene
            .set_robot_dynamics_with(0, true, None, None, None, None, Some(false))
            .unwrap();
        let held = scene
            .simulate_physics_with(1.0, &options, rapier())
            .unwrap();
        assert!(held.robots[0].base.is_none());
    }

    #[test]
    fn an_object_in_hand_at_the_start_rides_the_collapse_on_a_weld() {
        let mut scene = arm(false);
        let tip = scene.robots()[0].model.links.len() - 1;
        let tcp = scene.link_poses_for(0)[tip];
        scene
            .add_obstacle(
                "part",
                Geometry::Box {
                    size: Vector3::new(0.05, 0.05, 0.05),
                },
                tcp * Isometry3::translation(0.0, 0.0, 0.05),
            )
            .unwrap();
        let tip_name = scene.robots()[0].model.links[tip].name.clone();
        scene
            .attach_obstacle("part", Some(tip_name.as_str()), None)
            .unwrap();
        let grasp = scene.attachment("part").unwrap().grasp;
        scene.upsert_sequence(Sequence {
            name: "hold5".into(),
            steps: vec![step("wait", vec![], Condition::Elapsed { seconds: 5.0 })],
        });
        let mut live = scene
            .open_rollout(&["hold5"], &world_powered(false), rapier())
            .unwrap();
        for _ in 0..300 {
            live.tick().unwrap();
        }
        let link = live.link_poses(0).unwrap()[tip];
        let part = live
            .scene()
            .obstacles()
            .iter()
            .find(|o| o.name == "part")
            .unwrap()
            .pose;
        // The arm folded, and the part is still where the grasp says,
        // in the collapsed link's frame.
        let q = live.view().joint_positions(0).unwrap().to_vec();
        assert!((q[1] - READY[1]).abs() > 0.3, "shoulder at {}", q[1]);
        let expect = link * grasp;
        assert!(
            (expect.translation.vector - part.translation.vector).norm() < 0.01,
            "part drifted {:?} from its grasp",
            (expect.translation.vector - part.translation.vector)
        );
        // Under the declared scope the part is the FK-supplied mirror it
        // always was, on a kinematic arm.
        let plain = scene
            .simulate_physics_with(0.5, &RolloutOptions::default(), rapier())
            .unwrap();
        assert!(plain.objects.iter().any(|o| o.name == "part"));
        // A program that lets it go is refused up front.
        scene.upsert_sequence(Sequence {
            name: "drop".into(),
            steps: vec![step(
                "let_go",
                vec![Action::Detach {
                    object: "part".into(),
                }],
                Condition::Elapsed { seconds: 0.5 },
            )],
        });
        let err = scene
            .simulate_sequences_with(&["drop"], &world_unpowered(), rapier())
            .unwrap_err();
        assert!(err.to_string().contains("welded"), "{err}");
    }
}

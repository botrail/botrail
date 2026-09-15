//! Dynamic robots (design-rl-dynamics.md): a robot declared dynamic is
//! simulated, under a physics bake, as an articulated body — every link
//! with geometry a rigid body weighing what its model says, every joint
//! a force-capped servo following the planned motion, the base a
//! kinematic mirror on its stand or vehicle. The declaration is the
//! gripper drive of G3 widened to the whole robot; without a physics
//! backend it is inert and the robot moves exactly as planned.
//!
//! The servo is the industrial cascade: a PI position loop in the
//! rollout turns the command error into an approach velocity
//! ([`APPROACH_GAIN`] per second plus [`INTEGRAL_GAIN`] times its
//! integral, clamped to the joint's rated speed), and the engine's motor
//! closes the velocity loop under the force cap (`damping` is its gain).
//! No position spring: a spring stiff enough to hold a load to
//! sub-milliradians asks for approach speeds the cap cannot brake, and a
//! joint knocked off its target — a collision, the hand-back after a
//! torque episode — then oscillates around it for good. The velocity
//! loop brakes at the cap and comes back at the rated speed instead. The
//! integral term is what removes the static droop under load the way a
//! real drive's does — with no model knowledge; the loop is wound back
//! whenever the rated-speed clamp holds, and cleared while a raw torque
//! owns the joint. Gravity compensation is not a simulator feature: it
//! is an opt-in of the `Torque` control (a torque interface whose
//! firmware compensates), and the model's gravity torques are readable
//! (`gravity_torques`) for any controller a user builds.

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
}

/// Default mass floor per dynamic link (kg); see [`RobotDynamics::mass_floor`].
pub const DEFAULT_MASS_FLOOR: f64 = 0.2;

/// Default reflected drive inertia: 0.1 kg·m² about a revolute joint
/// (a small geared servo), 10 kg along a prismatic one (a ball screw).
pub const DEFAULT_ARMATURE_REVOLUTE: f64 = 0.1;
pub const DEFAULT_ARMATURE_PRISMATIC: f64 = 10.0;

/// Approach gain (1/s): command error → approach velocity, before the
/// rated-speed clamp. 20/s is a 50 ms time constant, inside a 100 Hz
/// scan's stability margin (`dt · gain < 1`) and brake-feasible for an
/// arm whose cap decelerates it at a few tens of rad/s².
pub const APPROACH_GAIN: f64 = 20.0;

/// Integral gain of the position loop (1/s²): critically damped against
/// the approach gain (`gain² / 4`), a 0.1 s time constant for the
/// integral to take a static load over.
pub const INTEGRAL_GAIN: f64 = APPROACH_GAIN * APPROACH_GAIN / 4.0;

fn default_armature(kind: botrail_physics::JointKind) -> f64 {
    match kind {
        botrail_physics::JointKind::Prismatic => DEFAULT_ARMATURE_PRISMATIC,
        _ => DEFAULT_ARMATURE_REVOLUTE,
    }
}

/// Default velocity-loop gain: the cap is reached at a velocity error
/// of 0.02 rad/s (revolute) or 2 mm/s (prismatic) — a stiff loop.
fn default_damping(kind: botrail_physics::JointKind, max_force: f64) -> f64 {
    match kind {
        botrail_physics::JointKind::Prismatic => max_force / 0.002,
        _ => max_force / 0.02,
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
        let Some(sr) = self.robots.get(robot) else {
            return Err(SceneError::UnknownRobot(robot.to_string()));
        };
        if !dynamic {
            self.robots[robot].dynamics = None;
            return Ok(());
        }
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
        let model = sr.model.clone();
        let mut servos = Vec::with_capacity(model.actuated_joints.len());
        for &ji in &model.actuated_joints {
            let kind = crate::grasp::joint_kind(&model, ji);
            let limits = model.joints[ji].limits.as_ref();
            let cap = match max_force {
                Some(f) => f,
                None => {
                    let effort = limits.map_or(0.0, |l| l.effort);
                    if !(effort.is_finite() && effort > 0.0) {
                        return Err(SceneError::BadDynamics(format!(
                            "joint `{}` declares no effort limit — pass max_force=",
                            model.joints[ji].name
                        )));
                    }
                    effort
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
        self.robots[robot].dynamics = Some(RobotDynamics {
            max_force,
            damping,
            armature,
            mass_floor: mass_floor.unwrap_or(DEFAULT_MASS_FLOOR),
            servos,
        });
        Ok(())
    }

    /// The dynamics declaration of a robot, if any.
    pub fn robot_dynamics(&self, robot: usize) -> Option<&RobotDynamics> {
        self.robots.get(robot).and_then(|r| r.dynamics.as_ref())
    }
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
        assert_eq!(d.servos[0].damping, 2500.0);
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
        assert!(scene.robot_dynamics(0).unwrap().servos.iter().all(|s| s.armature == 0.0));
        assert!(scene.robot_dynamics(0).unwrap().servos.iter().all(|s| s.max_force == 20.0));
        scene.set_robot_dynamics(0, false, None, None, None, None).unwrap();
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
        assert!(worst < 0.01, "the motors hold READY to within {worst:.4} rad");
        // The integral term takes the load over: the second half of the
        // hold sits within a fraction of a milliradian.
        let settled = track
            .times
            .iter()
            .zip(&track.positions)
            .filter(|(t, _)| **t >= 0.5)
            .flat_map(|(_, q)| q.iter().zip(READY.iter()).map(|(a, b)| (a - b).abs()))
            .fold(0.0, f64::max);
        assert!(settled < 5e-4, "settled droop {settled:.5} rad");
        // The physical arm rests a little below the command: the
        // read-back is the engine's state, not the plan.
        let last = track.positions.last().unwrap();
        assert!(last.iter().zip(READY.iter()).any(|(a, b)| (a - b).abs() > 1e-6), "{last:?}");
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
            assert!((q[1] - command).abs() < 0.1, "t={t:.2}: q1={:.3} vs {command:.3}", q[1]);
        }
        let last = track.positions.last().unwrap();
        assert!((last[1] - 1.0).abs() < 0.02, "settled at {:.4}", last[1]);
        assert!((tl.duration - 1.5).abs() < 0.011, "duration {}", tl.duration);
    }

    #[test]
    fn dynamic_bakes_are_bit_identical() {
        let scene = arm(true);
        let options = RolloutOptions::default();
        let a = scene.simulate_sequences_with(&["lift"], &options, rapier()).unwrap();
        let b = scene.simulate_sequences_with(&["lift"], &options, rapier()).unwrap();
        assert_eq!(a.robots[0].trajectory.positions, b.robots[0].trajectory.positions);
        // The live rollout ticks the same physics.
        let mut live = scene.open_rollout(&["lift"], &options, rapier()).unwrap();
        while !live.finished() {
            live.tick().unwrap();
        }
        let live_tl = live.finish();
        assert_eq!(a.robots[0].trajectory.positions, live_tl.robots[0].trajectory.positions);
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
        assert!((q_end[0] - hold[0]).abs() < 0.05, "held at {:.4} vs {:.4}", q_end[0], hold[0]);
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
        let drift = q.iter().zip(&start).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
        assert!(drift < 0.02, "held by its own gravity torques: drift {drift:.4} rad");
        // Without them the arm falls.
        let mut free = scene.open_rollout(&["hold"], &options, rapier()).unwrap();
        free.drive(0, None, None).unwrap();
        free.command_torque(0, &[(1, 0.0), (2, 0.0)], false).unwrap();
        for _ in 0..100 {
            free.tick().unwrap();
        }
        let fell = free.joint_positions(0).unwrap()[1] - start[1];
        assert!(fell.abs() > 0.1, "motors off, no torque: the shoulder moved {fell:+.3}");
        // The opt-in: zero raw torques on top of the compensation hold.
        let mut comp = scene.open_rollout(&["hold"], &options, rapier()).unwrap();
        comp.drive(0, None, None).unwrap();
        let zeros: Vec<(usize, f64)> = (0..6).map(|qi| (qi, 0.0)).collect();
        comp.command_torque(0, &zeros, true).unwrap();
        for _ in 0..100 {
            comp.tick().unwrap();
        }
        let q = comp.joint_positions(0).unwrap();
        let drift = q.iter().zip(&start).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
        assert!(drift < 0.02, "compensated: drift {drift:.4} rad");
    }
}

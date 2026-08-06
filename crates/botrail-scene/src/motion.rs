//! Motions: named sequences of segments, matching the waypoint-teaching
//! mental model. Each segment moves from wherever the previous one ended to
//! its goal configuration, either via a collision-free joint-space plan or
//! via a straight Cartesian line of the TCP.
//!
//! Constraints are enforced as validity filters (rejection): fine for wide
//! cones/regions; narrow constraints will need projection-based planning
//! (post-M4, see DESIGN).

use botrail_traj::JointTrajectory;
use nalgebra::{Isometry3, Vector3};
use thiserror::Error;

use crate::Scene;

#[derive(Debug, Error)]
pub enum MotionError {
    #[error("unknown motion `{0}`")]
    UnknownMotion(String),
    #[error("motion `{0}` has no segments")]
    EmptyMotion(String),
    #[error("segment {index} goal expects {expected} joint values, got {got}")]
    WrongDof {
        index: usize,
        expected: usize,
        got: usize,
    },
    #[error("segment {0} is out of range")]
    BadSegmentIndex(usize),
    #[error("segment {index}: start violates its constraints")]
    StartViolatesConstraints { index: usize },
    #[error("segment {index}: planning failed: {source}")]
    PlanFailed {
        index: usize,
        source: botrail_plan::PlanError,
    },
    #[error("segment {index}: cartesian line failed at {fraction:.0}%: {reason}")]
    CartesianFailed {
        index: usize,
        fraction: f64,
        reason: String,
    },
    #[error("time parameterization failed: {0}")]
    Timing(#[from] botrail_traj::TrajError),
}

/// How a segment reaches its goal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SegmentKind {
    /// Collision-free joint-space plan (RRT-Connect + shortcut).
    Joint,
    /// Straight TCP line, followed with seed-continuous IK.
    CartesianLine,
}

/// Path constraint on the TCP, enforced along the whole segment.
#[derive(Debug, Clone)]
pub enum Constraint {
    /// The tool axis (`axis_local` in the TCP frame) must stay within
    /// `angle` radians of `axis_world`.
    OrientationCone {
        axis_local: Vector3<f64>,
        axis_world: Vector3<f64>,
        angle: f64,
    },
    /// The TCP origin must stay inside the world-aligned box.
    PositionBox {
        min: Vector3<f64>,
        max: Vector3<f64>,
    },
}

#[derive(Debug, Clone)]
pub struct Segment {
    pub kind: SegmentKind,
    /// Goal configuration (captured robot pose), DOF order.
    pub goal_positions: Vec<f64>,
    pub constraints: Vec<Constraint>,
}

#[derive(Debug, Clone)]
pub struct Motion {
    pub name: String,
    /// Owning robot (index into `Scene::robots`): the goal configurations
    /// are in this robot's DOF order and planning drives this robot (every
    /// other robot is a frozen collision body).
    pub robot: usize,
    pub segments: Vec<Segment>,
}

/// The joint path a single segment was planned through, before
/// densification and time parameterization. For `Joint` segments this is
/// the shortcut-smoothed RRT output (sparse; consecutive waypoints are
/// connected by valid straight joint-space segments), for `CartesianLine`
/// the seed-continuous IK follow points along the TCP line. Script
/// exporters consume this to emit per-vendor move commands instead of
/// reconstructing waypoints from the dense trajectory.
#[derive(Debug, Clone)]
pub struct PlannedSegment {
    pub kind: SegmentKind,
    /// Waypoints in DOF order, both endpoints included. The first equals
    /// the previous segment's last; the last is the reached goal.
    pub waypoints: Vec<Vec<f64>>,
}

/// A planned motion: one concatenated trajectory plus the time at which
/// each segment ends (for UI markers and per-segment inspection) and the
/// per-segment sparse paths (for script export).
#[derive(Debug, Clone)]
pub struct PlannedMotion {
    pub trajectory: JointTrajectory,
    pub segment_ends: Vec<f64>,
    pub segments: Vec<PlannedSegment>,
}

#[derive(Debug, Clone)]
pub struct CartesianOptions {
    /// Translation step between IK follow points (m).
    pub step_pos: f64,
    /// Rotation step between IK follow points (rad).
    pub step_rot: f64,
    /// Maximum joint-space jump between consecutive follow points; larger
    /// jumps indicate an IK branch change and abort the segment.
    pub jump_threshold: f64,
}

impl Default for CartesianOptions {
    fn default() -> Self {
        CartesianOptions {
            step_pos: 0.01,
            step_rot: 0.05,
            jump_threshold: 0.5,
        }
    }
}

fn constraints_ok(
    scene: &Scene,
    robot: usize,
    q: &[f64],
    constraints: &[Constraint],
    tcp: usize,
) -> bool {
    if constraints.is_empty() {
        return true;
    }
    let Ok(poses) = scene.fk_for(robot, q) else {
        return false;
    };
    let pose = &poses[tcp];
    constraints.iter().all(|c| match c {
        Constraint::OrientationCone {
            axis_local,
            axis_world,
            angle,
        } => {
            let world = pose.rotation * axis_local;
            let cos = world.normalize().dot(&axis_world.normalize());
            cos.clamp(-1.0, 1.0).acos() <= *angle + 1e-9
        }
        Constraint::PositionBox { min, max } => {
            let p = pose.translation.vector;
            (0..3).all(|i| p[i] >= min[i] - 1e-9 && p[i] <= max[i] + 1e-9)
        }
    })
}

fn joint_distance(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y) * (x - y))
        .sum::<f64>()
        .sqrt()
}

/// Follows a straight TCP line from `start_q` toward the TCP pose of
/// `goal_q` with seed-continuous IK. Returns the joint path (both endpoints
/// included; the final configuration is the IK-followed one).
#[allow(clippy::too_many_arguments)]
fn cartesian_line(
    scene: &Scene,
    robot: usize,
    start_q: &[f64],
    goal_q: &[f64],
    constraints: &[Constraint],
    tcp: usize,
    options: &CartesianOptions,
    index: usize,
) -> Result<Vec<Vec<f64>>, MotionError> {
    let fail = |fraction: f64, reason: &str| MotionError::CartesianFailed {
        index,
        fraction: fraction * 100.0,
        reason: reason.to_string(),
    };
    // World-frame TCP poses; the base pose cancels out of the interpolated
    // targets only through solve_ik_world's re-expression.
    let start_pose = scene
        .fk_for(robot, start_q)
        .map_err(|e| fail(0.0, &e.to_string()))?[tcp];
    let goal_pose = scene
        .fk_for(robot, goal_q)
        .map_err(|e| fail(0.0, &e.to_string()))?[tcp];

    let dist = (goal_pose.translation.vector - start_pose.translation.vector).norm();
    let angle = start_pose.rotation.angle_to(&goal_pose.rotation);
    let steps = ((dist / options.step_pos).max(angle / options.step_rot))
        .ceil()
        .max(1.0) as usize;

    let ik_options = botrail_kin::IkOptions {
        max_iters: 50,
        tol_pos: 1e-5,
        tol_rot: 1e-4,
        ..botrail_kin::IkOptions::default()
    };

    let mut q = start_q.to_vec();
    let mut path = vec![q.clone()];
    for k in 1..=steps {
        let u = k as f64 / steps as f64;
        let target = Isometry3::from_parts(
            (start_pose.translation.vector
                + (goal_pose.translation.vector - start_pose.translation.vector) * u)
                .into(),
            start_pose.rotation.slerp(&goal_pose.rotation, u),
        );
        let ik = scene
            .solve_ik_world_for(robot, tcp, &target, &q, &ik_options)
            .map_err(|e| fail(u, &e.to_string()))?;
        if !ik.converged {
            return Err(fail(u, "IK did not converge (unreachable along the line)"));
        }
        if joint_distance(&ik.q, &q) > options.jump_threshold {
            return Err(fail(u, "configuration jump (IK branch change)"));
        }
        if !scene.is_state_valid_for(robot, &ik.q) {
            return Err(fail(u, "collision or joint limit violation"));
        }
        if !constraints_ok(scene, robot, &ik.q, constraints, tcp) {
            return Err(fail(u, "constraint violation"));
        }
        q = ik.q.clone();
        path.push(ik.q);
    }
    Ok(path)
}

/// Plans one segment of robot `robot` from `start_q`; returns the joint
/// path.
fn plan_segment(
    scene: &Scene,
    robot: usize,
    start_q: &[f64],
    segment: &Segment,
    tcp: usize,
    index: usize,
    plan_options: &botrail_plan::PlanOptions,
) -> Result<Vec<Vec<f64>>, MotionError> {
    let model = &scene.robots()[robot].model;
    if segment.goal_positions.len() != model.dof() {
        return Err(MotionError::WrongDof {
            index,
            expected: model.dof(),
            got: segment.goal_positions.len(),
        });
    }
    if !constraints_ok(scene, robot, start_q, &segment.constraints, tcp) {
        return Err(MotionError::StartViolatesConstraints { index });
    }
    match segment.kind {
        SegmentKind::Joint => {
            let (lower, upper) = model.sampling_bounds();
            let space = botrail_plan::JointSpace { lower, upper };
            let mut is_valid = |q: &[f64]| {
                scene.is_state_valid_for(robot, q)
                    && constraints_ok(scene, robot, q, &segment.constraints, tcp)
            };
            botrail_plan::plan(
                &space,
                start_q,
                &segment.goal_positions,
                &mut is_valid,
                plan_options,
            )
            .map_err(|source| MotionError::PlanFailed { index, source })
        }
        SegmentKind::CartesianLine => cartesian_line(
            scene,
            robot,
            start_q,
            &segment.goal_positions,
            &segment.constraints,
            tcp,
            &CartesianOptions::default(),
            index,
        ),
    }
}

/// Trajectory limits from the model: joint velocity limits (defaulting to
/// 1 rad/s where unspecified) and acceleration at twice the velocity bound
/// (URDF has no acceleration field; reaches peak speed in 0.5s).
pub fn traj_limits(model: &botrail_model::RobotModel) -> botrail_traj::Limits {
    let velocity: Vec<f64> = model
        .actuated_joints
        .iter()
        .map(|&ji| match model.joints[ji].limits {
            Some(l) if l.velocity > 0.0 => l.velocity,
            _ => 1.0,
        })
        .collect();
    let acceleration = velocity.iter().map(|v| 2.0 * v).collect();
    botrail_traj::Limits {
        velocity,
        acceleration,
    }
}

/// Plans every segment of `motion` — driving its owning robot, with every
/// other robot as a frozen collision body — starting from that robot's
/// current configuration, time-parameterizes each segment (rest-to-rest at
/// segment boundaries, teach-pendant style), and concatenates the results.
pub fn plan_motion(
    scene: &Scene,
    motion: &Motion,
    plan_options: &botrail_plan::PlanOptions,
    limits: &botrail_traj::Limits,
) -> Result<PlannedMotion, MotionError> {
    if motion.segments.is_empty() {
        return Err(MotionError::EmptyMotion(motion.name.clone()));
    }
    let robot = motion.robot;
    let tcp = scene.robots()[robot].model.default_tcp_link();
    let timing = botrail_traj::TimingOptions::default();

    let mut current = scene.robots()[robot].joint_positions().to_vec();
    let mut combined: Option<JointTrajectory> = None;
    let mut segment_ends = Vec::with_capacity(motion.segments.len());
    let mut segments = Vec::with_capacity(motion.segments.len());

    for (index, segment) in motion.segments.iter().enumerate() {
        let path = plan_segment(scene, robot, &current, segment, tcp, index, plan_options)?;
        current = path.last().expect("paths are non-empty").clone();
        let traj = botrail_traj::time_parameterize(&path, limits, &timing)?;
        combined = Some(match combined {
            None => traj,
            Some(head) => concatenate(head, traj),
        });
        segment_ends.push(combined.as_ref().expect("just set").duration());
        segments.push(PlannedSegment {
            kind: segment.kind,
            waypoints: path,
        });
    }

    Ok(PlannedMotion {
        trajectory: combined.expect("at least one segment"),
        segment_ends,
        segments,
    })
}

/// Appends `tail` to `head`, shifting its timestamps. The duplicated
/// boundary waypoint (tail start == head end) is dropped.
fn concatenate(mut head: JointTrajectory, tail: JointTrajectory) -> JointTrajectory {
    let offset = head.duration();
    let skip = usize::from(
        tail.positions
            .first()
            .zip(head.positions.last())
            .is_some_and(|(a, b)| joint_distance(a, b) < 1e-9),
    );
    for (i, t) in tail.times.iter().enumerate().skip(skip) {
        head.times.push(offset + t);
        head.positions.push(tail.positions[i].clone());
        head.velocities.push(tail.velocities[i].clone());
    }
    head
}

#[cfg(test)]
mod tests {
    use super::*;
    use botrail_model::RobotModel;
    use std::f64::consts::FRAC_PI_2;
    use std::sync::Arc;

    const ARM: &str = include_str!("../../../examples/simple_arm.urdf");

    fn scene() -> Scene {
        Scene::new(Arc::new(RobotModel::from_urdf_str(ARM).unwrap()))
    }

    fn seg(kind: SegmentKind, goal: Vec<f64>) -> Segment {
        Segment {
            kind,
            goal_positions: goal,
            constraints: vec![],
        }
    }

    fn limits() -> botrail_traj::Limits {
        botrail_traj::Limits::uniform(6, 2.0, 4.0)
    }

    /// xurdf 0.6 defaults an omitted `velocity` attribute to 0 (and an
    /// absent `<limit>` to no limits at all); neither may reach the time
    /// parameterizer, which rejects non-positive velocity bounds.
    #[test]
    fn traj_limits_fall_back_when_velocity_is_unspecified() {
        let urdf = r#"
        <robot name="r">
          <link name="base"/><link name="l1"/><link name="l2"/><link name="l3"/>
          <joint name="no_velocity" type="revolute">
            <parent link="base"/><child link="l1"/>
            <axis xyz="0 0 1"/><limit lower="-1" upper="1"/>
          </joint>
          <joint name="no_limit" type="revolute">
            <parent link="l1"/><child link="l2"/>
            <origin xyz="1 0 0"/><axis xyz="0 0 1"/>
          </joint>
          <joint name="specified" type="revolute">
            <parent link="l2"/><child link="l3"/>
            <origin xyz="1 0 0"/><axis xyz="0 0 1"/>
            <limit lower="-1" upper="1" effort="1" velocity="2"/>
          </joint>
        </robot>"#;
        let model = RobotModel::from_urdf_str(urdf).unwrap();
        let limits = traj_limits(&model);
        assert_eq!(limits.velocity, vec![1.0, 1.0, 2.0]);
        assert_eq!(limits.acceleration, vec![2.0, 2.0, 4.0]);
    }

    #[test]
    fn two_segment_motion_concatenates() {
        let scene = scene();
        let motion = Motion {
            name: "m".into(),
            robot: 0,
            segments: vec![
                seg(SegmentKind::Joint, vec![0.6, 0.4, -0.5, 0.2, 0.0, 0.0]),
                seg(SegmentKind::Joint, vec![-0.4, 0.8, -1.0, 0.0, 0.3, 0.0]),
            ],
        };
        let planned = plan_motion(
            &scene,
            &motion,
            &botrail_plan::PlanOptions::default(),
            &limits(),
        )
        .unwrap();
        let traj = &planned.trajectory;
        assert_eq!(planned.segment_ends.len(), 2);
        assert_eq!(planned.segment_ends[1], traj.duration());
        assert!(planned.segment_ends[0] < planned.segment_ends[1]);
        // Passes through the first goal at the recorded segment end.
        let at_boundary = traj.sample(planned.segment_ends[0]);
        for (a, b) in at_boundary.iter().zip(&motion.segments[0].goal_positions) {
            assert!((a - b).abs() < 1e-6, "{a} vs {b}");
        }
        // Ends at the final goal, times strictly increasing.
        let end = traj.sample(traj.duration());
        for (a, b) in end.iter().zip(&motion.segments[1].goal_positions) {
            assert!((a - b).abs() < 1e-6);
        }
        assert!(traj.times.windows(2).all(|w| w[1] > w[0]));
    }

    #[test]
    fn planned_motion_retains_sparse_segment_paths() {
        let scene = scene();
        let g1 = vec![0.6, 0.4, -0.5, 0.2, 0.0, 0.0];
        let g2 = vec![-0.4, 0.8, -1.0, 0.0, 0.3, 0.0];
        let motion = Motion {
            name: "m".into(),
            robot: 0,
            segments: vec![
                seg(SegmentKind::Joint, g1.clone()),
                seg(SegmentKind::Joint, g2.clone()),
            ],
        };
        let planned = plan_motion(
            &scene,
            &motion,
            &botrail_plan::PlanOptions::default(),
            &limits(),
        )
        .unwrap();

        assert_eq!(planned.segments.len(), 2);
        assert!(planned
            .segments
            .iter()
            .all(|s| s.kind == SegmentKind::Joint));
        // Chained endpoints: start at the scene configuration, pass exactly
        // through each goal.
        assert_eq!(
            planned.segments[0].waypoints.first().unwrap().as_slice(),
            scene.joint_positions()
        );
        assert_eq!(planned.segments[0].waypoints.last().unwrap(), &g1);
        assert_eq!(planned.segments[1].waypoints.first().unwrap(), &g1);
        assert_eq!(planned.segments[1].waypoints.last().unwrap(), &g2);
        // Sparse: fewer waypoints than the densified trajectory.
        let sparse: usize = planned.segments.iter().map(|s| s.waypoints.len()).sum();
        assert!(
            sparse < planned.trajectory.positions.len(),
            "{sparse} sparse vs {} dense",
            planned.trajectory.positions.len()
        );
    }

    #[test]
    fn cartesian_line_keeps_tcp_on_the_line() {
        let scene = scene();
        // Start folded horizontally, goal: same orientation, shifted down in z
        // via a joint-space capture: fold slightly differently.
        let start = vec![0.0, 1.1, -0.6, -0.5, 0.0, 0.0];
        let mut scene = scene;
        scene.set_joint_positions(start.clone()).unwrap();
        let tcp = scene.robot().default_tcp_link();
        let start_pose = scene.link_poses()[tcp];

        // Goal configuration: IK to a pose 8cm lower, same orientation.
        let target = Isometry3::from_parts(
            (start_pose.translation.vector + Vector3::new(0.0, 0.0, -0.08)).into(),
            start_pose.rotation,
        );
        let ik = botrail_kin::solve_ik(
            scene.robot(),
            tcp,
            &target,
            &start,
            &botrail_kin::IkOptions::default(),
        )
        .unwrap();
        assert!(ik.converged);

        let path = cartesian_line(
            &scene,
            0,
            &start,
            &ik.q,
            &[],
            tcp,
            &CartesianOptions::default(),
            0,
        )
        .unwrap();
        // Every follow point's TCP lies on the straight segment (within tol).
        for q in &path {
            let pose = botrail_kin::forward_kinematics(scene.robot(), q).unwrap()[tcp];
            let p = pose.translation.vector;
            let a = start_pose.translation.vector;
            let b = target.translation.vector;
            let ab = b - a;
            let t = (p - a).dot(&ab) / ab.norm_squared();
            let closest = a + ab * t.clamp(0.0, 1.0);
            assert!(
                (p - closest).norm() < 2e-3,
                "off line by {}",
                (p - closest).norm()
            );
        }
        let final_pose =
            botrail_kin::forward_kinematics(scene.robot(), path.last().unwrap()).unwrap()[tcp];
        assert!((final_pose.translation.vector - target.translation.vector).norm() < 1e-3);

        // Planned as a motion, the segment retains the IK follow path
        // (deterministic, so it matches the direct call above).
        let motion = Motion {
            name: "descend".into(),
            robot: 0,
            segments: vec![seg(SegmentKind::CartesianLine, ik.q.clone())],
        };
        let planned = plan_motion(
            &scene,
            &motion,
            &botrail_plan::PlanOptions::default(),
            &limits(),
        )
        .unwrap();
        assert_eq!(planned.segments.len(), 1);
        assert_eq!(planned.segments[0].kind, SegmentKind::CartesianLine);
        assert_eq!(planned.segments[0].waypoints, path);
    }

    #[test]
    fn cartesian_line_follows_world_line_with_moved_base() {
        let mut scene = scene();
        let base = Isometry3::from_parts(
            nalgebra::Translation3::new(0.7, -0.4, 0.2),
            nalgebra::UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.9),
        );
        scene.set_robot_base_pose(base);
        let start = vec![0.0, 1.1, -0.6, -0.5, 0.0, 0.0];
        scene.set_joint_positions(start.clone()).unwrap();
        let tcp = scene.robot().default_tcp_link();
        let start_pose = scene.link_poses()[tcp];

        // World-frame goal: 8cm straight down from the current TCP.
        let target = Isometry3::from_parts(
            (start_pose.translation.vector + Vector3::new(0.0, 0.0, -0.08)).into(),
            start_pose.rotation,
        );
        let ik = scene
            .solve_ik_world(tcp, &target, &start, &botrail_kin::IkOptions::default())
            .unwrap();
        assert!(ik.converged);

        let path = cartesian_line(
            &scene,
            0,
            &start,
            &ik.q,
            &[],
            tcp,
            &CartesianOptions::default(),
            0,
        )
        .unwrap();
        // Every follow point's WORLD TCP lies on the straight world segment.
        for q in &path {
            let pose = scene.fk(q).unwrap()[tcp];
            let p = pose.translation.vector;
            let a = start_pose.translation.vector;
            let b = target.translation.vector;
            let ab = b - a;
            let t = (p - a).dot(&ab) / ab.norm_squared();
            let closest = a + ab * t.clamp(0.0, 1.0);
            assert!(
                (p - closest).norm() < 2e-3,
                "off line by {}",
                (p - closest).norm()
            );
        }
        let final_pose = scene.fk(path.last().unwrap()).unwrap()[tcp];
        assert!((final_pose.translation.vector - target.translation.vector).norm() < 1e-3);
    }

    #[test]
    fn orientation_cone_filters_plans() {
        let mut scene = scene();
        // Start folded horizontally: tool +z points along world +x.
        scene
            .set_joint_positions(vec![0.0, FRAC_PI_2, 0.0, 0.0, 0.0, 0.0])
            .unwrap();
        // Goal: same fold, panned 90 degrees; the tool stays horizontal all
        // the way if the cone demands it.
        let cone = Constraint::OrientationCone {
            axis_local: Vector3::z(),
            axis_world: Vector3::new(1.0, 1.0, 0.0),
            angle: 1.0,
        };
        let motion = Motion {
            name: "m".into(),
            robot: 0,
            segments: vec![Segment {
                kind: SegmentKind::Joint,
                goal_positions: vec![FRAC_PI_2, FRAC_PI_2, 0.0, 0.0, 0.0, 0.0],
                constraints: vec![cone.clone()],
            }],
        };
        let planned = plan_motion(
            &scene,
            &motion,
            &botrail_plan::PlanOptions::default(),
            &limits(),
        )
        .unwrap();
        // Constraint holds along the sampled trajectory.
        let tcp = scene.robot().default_tcp_link();
        let mut t = 0.0;
        while t <= planned.trajectory.duration() {
            let q = planned.trajectory.sample(t);
            assert!(constraints_ok(
                &scene,
                0,
                &q,
                std::slice::from_ref(&cone),
                tcp
            ));
            t += 0.1;
        }
    }

    #[test]
    fn constraint_violating_start_is_rejected() {
        let scene = scene();
        // Upright start: tool +z points up, but the cone demands +x.
        let motion = Motion {
            name: "m".into(),
            robot: 0,
            segments: vec![Segment {
                kind: SegmentKind::Joint,
                goal_positions: vec![0.0; 6],
                constraints: vec![Constraint::OrientationCone {
                    axis_local: Vector3::z(),
                    axis_world: Vector3::x(),
                    angle: 0.3,
                }],
            }],
        };
        assert!(matches!(
            plan_motion(
                &scene,
                &motion,
                &botrail_plan::PlanOptions::default(),
                &limits()
            ),
            Err(MotionError::StartViolatesConstraints { .. })
        ));
    }

    #[test]
    fn plan_detours_around_a_second_robot() {
        let mut scene = scene();
        // Robot 0 leans 1.2 rad toward +x; the goal is the same lean panned
        // to -x. The straight joint interpolation sweeps the leaning arm
        // through +y — straight at a second arm standing upright there.
        let lean = vec![0.0, 1.2, 0.0, 0.0, 0.0, 0.0];
        scene.set_joint_positions(lean.clone()).unwrap();
        let blocker = Arc::new(RobotModel::from_urdf_str(ARM).unwrap());
        scene.add_robot(
            blocker,
            Some("blocker"),
            nalgebra::Isometry3::from_parts(
                nalgebra::Translation3::new(0.0, 0.55, 0.0),
                nalgebra::UnitQuaternion::identity(),
            ),
        );
        let goal = vec![std::f64::consts::PI, 1.2, 0.0, 0.0, 0.0, 0.0];
        let mid = vec![std::f64::consts::FRAC_PI_2, 1.2, 0.0, 0.0, 0.0, 0.0];
        assert!(scene.is_state_valid_for(0, &lean));
        assert!(scene.is_state_valid_for(0, &goal));
        // The naive sweep collides with the second robot…
        assert!(!scene.is_state_valid_for(0, &mid));
        // …so the planner must detour, and every densified sample of the
        // result stays clear of it.
        let motion = Motion {
            name: "swing".into(),
            robot: 0,
            segments: vec![seg(SegmentKind::Joint, goal.clone())],
        };
        let planned = plan_motion(
            &scene,
            &motion,
            &botrail_plan::PlanOptions::default(),
            &limits(),
        )
        .unwrap();
        let traj = &planned.trajectory;
        let end = traj.sample(traj.duration());
        for (a, b) in end.iter().zip(&goal) {
            assert!((a - b).abs() < 1e-6);
        }
        let mut t = 0.0;
        while t <= traj.duration() {
            assert!(
                scene.is_state_valid_for(0, &traj.sample(t)),
                "collides with the second robot at t = {t}"
            );
            t += 0.05;
        }
    }

    #[test]
    fn empty_motion_is_rejected() {
        let scene = scene();
        let motion = Motion {
            name: "empty".into(),
            robot: 0,
            segments: vec![],
        };
        assert!(matches!(
            plan_motion(
                &scene,
                &motion,
                &botrail_plan::PlanOptions::default(),
                &limits()
            ),
            Err(MotionError::EmptyMotion(_))
        ));
    }
}

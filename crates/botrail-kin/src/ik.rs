//! Damped-least-squares inverse kinematics.

use botrail_model::{JointType, RobotModel};
use nalgebra::{DMatrix, DVector, Isometry3, Vector3};

use crate::{forward_kinematics, KinError};

/// What the solver tries to match.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IkMode {
    /// Position only (3 DOF task).
    Position,
    /// Full pose: position + orientation (6 DOF task).
    Pose,
}

#[derive(Debug, Clone)]
pub struct IkOptions {
    pub mode: IkMode,
    pub max_iters: usize,
    /// Convergence threshold on position error (m).
    pub tol_pos: f64,
    /// Convergence threshold on orientation error (rad). Ignored in position mode.
    pub tol_rot: f64,
    /// DLS damping factor lambda.
    pub damping: f64,
    /// Relative weight of orientation error vs position error.
    pub orientation_weight: f64,
    /// Maximum joint-space step norm per iteration (rad / m).
    pub max_step: f64,
}

impl Default for IkOptions {
    fn default() -> Self {
        IkOptions {
            mode: IkMode::Pose,
            max_iters: 100,
            tol_pos: 1e-5,
            tol_rot: 1e-4,
            damping: 0.05,
            orientation_weight: 0.5,
            max_step: 0.5,
        }
    }
}

impl IkOptions {
    /// Settings for interactive (per-frame, warm-seeded) solving: looser
    /// tolerances, but enough iterations to settle within a single message
    /// (a 6-DOF solve iteration is microseconds; the badge shown to the
    /// user reflects the final solve of a drag, so it must not stop short).
    pub fn streaming() -> Self {
        IkOptions {
            max_iters: 100,
            tol_pos: 1e-4,
            tol_rot: 1e-3,
            ..IkOptions::default()
        }
    }
}

#[derive(Debug, Clone)]
pub struct IkResult {
    /// Best joint configuration found (always within limits).
    pub q: Vec<f64>,
    /// Whether the error tolerances were met.
    pub converged: bool,
    /// Remaining position error (m).
    pub pos_error: f64,
    /// Remaining orientation error (rad); 0 in position mode.
    pub rot_error: f64,
    pub iters: usize,
}

/// Geometric Jacobian of `link`'s origin w.r.t. the actuated joints,
/// expressed in the world frame: rows 0-2 linear, rows 3-5 angular.
/// `poses` must be the FK result for the configuration of interest.
pub fn jacobian(model: &RobotModel, poses: &[Isometry3<f64>], link: usize) -> DMatrix<f64> {
    let mut on_chain = vec![false; model.joints.len()];
    let mut li = link;
    while let Some(ji) = model.links[li].parent_joint {
        on_chain[ji] = true;
        li = model.joints[ji].parent_link;
    }

    let p_end = poses[link].translation.vector;
    let mut jac = DMatrix::zeros(6, model.dof());
    for (col, &ji) in model.actuated_joints.iter().enumerate() {
        if !on_chain[ji] {
            continue;
        }
        let joint = &model.joints[ji];
        // The joint frame coincides with the child link frame; rotation about
        // its own axis leaves the axis invariant, so the child pose rotation
        // maps the local axis to world coordinates.
        let world_axis = poses[joint.child_link].rotation * joint.axis.into_inner();
        match joint.joint_type {
            JointType::Revolute | JointType::Continuous => {
                let p_joint = poses[joint.child_link].translation.vector;
                let linear = world_axis.cross(&(p_end - p_joint));
                jac.fixed_view_mut::<3, 1>(0, col).copy_from(&linear);
                jac.fixed_view_mut::<3, 1>(3, col).copy_from(&world_axis);
            }
            JointType::Prismatic => {
                jac.fixed_view_mut::<3, 1>(0, col).copy_from(&world_axis);
            }
            JointType::Fixed => unreachable!("fixed joints are not actuated"),
        }
    }
    jac
}

fn clamp_to_limits(model: &RobotModel, q: &mut [f64]) {
    for (qi, &ji) in q.iter_mut().zip(&model.actuated_joints) {
        if let Some(l) = model.joints[ji].limits {
            *qi = qi.clamp(l.lower, l.upper);
        }
    }
}

/// xorshift64* mapped to [-1, 1]; deterministic so solves are reproducible.
fn jitter_unit(state: &mut u64) -> f64 {
    *state ^= *state >> 12;
    *state ^= *state << 25;
    *state ^= *state >> 27;
    let bits = state.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11;
    bits as f64 / (1u64 << 53) as f64 * 2.0 - 1.0
}

fn pose_error(current: &Isometry3<f64>, target: &Isometry3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let e_pos = target.translation.vector - current.translation.vector;
    let r_err = target.rotation * current.rotation.inverse();
    let e_rot = r_err.scaled_axis();
    (e_pos, e_rot)
}

/// Solves for a configuration placing `link` at `target`, starting from
/// `seed`. Always returns the best configuration found; check `converged`.
pub fn solve_ik(
    model: &RobotModel,
    link: usize,
    target: &Isometry3<f64>,
    seed: &[f64],
    options: &IkOptions,
) -> Result<IkResult, KinError> {
    if seed.len() != model.dof() {
        return Err(KinError::WrongDof {
            expected: model.dof(),
            got: seed.len(),
        });
    }
    let task_dim = match options.mode {
        IkMode::Position => 3,
        IkMode::Pose => 6,
    };
    let lambda2 = options.damping * options.damping;

    let mut q = seed.to_vec();
    clamp_to_limits(model, &mut q);
    let mut rng: u64 = 0x9E37_79B9_7F4A_7C15;

    let mut best: Option<IkResult> = None;
    for iter in 0..=options.max_iters {
        let poses = forward_kinematics(model, &q)?;
        let (e_pos, e_rot) = pose_error(&poses[link], target);
        let pos_error = e_pos.norm();
        let rot_error = match options.mode {
            IkMode::Position => 0.0,
            IkMode::Pose => e_rot.norm(),
        };
        let converged = pos_error < options.tol_pos
            && (options.mode == IkMode::Position || rot_error < options.tol_rot);

        // Track the best configuration by weighted error.
        let score = pos_error + options.orientation_weight * rot_error;
        if best
            .as_ref()
            .map(|b| score < b.pos_error + options.orientation_weight * b.rot_error)
            .unwrap_or(true)
        {
            best = Some(IkResult {
                q: q.clone(),
                converged,
                pos_error,
                rot_error,
                iters: iter,
            });
        }
        if converged || iter == options.max_iters {
            break;
        }

        let jac_full = jacobian(model, &poses, link);
        let mut e = DVector::zeros(task_dim);
        e.fixed_rows_mut::<3>(0).copy_from(&e_pos);
        let jac = if options.mode == IkMode::Pose {
            e.fixed_rows_mut::<3>(3)
                .copy_from(&(options.orientation_weight * e_rot));
            let mut j = jac_full;
            j.rows_mut(3, 3).scale_mut(options.orientation_weight);
            j
        } else {
            jac_full.rows(0, 3).into_owned()
        };

        // dq = J^T (J J^T + lambda^2 I)^-1 e
        let jjt = &jac * jac.transpose() + DMatrix::identity(task_dim, task_dim) * lambda2;
        let Some(chol) = jjt.cholesky() else { break };
        let mut dq = jac.transpose() * chol.solve(&e);
        let step = dq.norm();
        if step < 1e-9 {
            // Stalled at a singularity (e.g. a fully extended arm asked to
            // move along its own axis produces dq = 0): kick the
            // configuration to break the symmetry, then keep iterating.
            for qi in q.iter_mut() {
                *qi += 0.05 * jitter_unit(&mut rng);
            }
            clamp_to_limits(model, &mut q);
            continue;
        }
        if step > options.max_step {
            dq *= options.max_step / step;
        }
        for (qi, dqi) in q.iter_mut().zip(dq.iter()) {
            *qi += dqi;
        }
        clamp_to_limits(model, &mut q);
    }

    Ok(best.expect("loop always records at least one result"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Translation3, UnitQuaternion};

    const SIX_DOF: &str = include_str!("../../../examples/simple_arm.urdf");

    fn six_dof() -> RobotModel {
        RobotModel::from_urdf_str(SIX_DOF).unwrap()
    }

    #[test]
    fn jacobian_matches_finite_differences() {
        let model = six_dof();
        let tool = model.link_index("tool0").unwrap();
        let q = [0.3, -0.5, 0.8, 0.4, -0.2, 0.6];
        let poses = forward_kinematics(&model, &q).unwrap();
        let jac = jacobian(&model, &poses, tool);

        let h = 1e-7;
        for col in 0..model.dof() {
            let mut qp = q;
            qp[col] += h;
            let poses_p = forward_kinematics(&model, &qp).unwrap();
            // linear part
            let dp = (poses_p[tool].translation.vector - poses[tool].translation.vector) / h;
            // angular part: rotation vector of R(q+h) * R(q)^-1, scaled by 1/h
            let dr = (poses_p[tool].rotation * poses[tool].rotation.inverse()).scaled_axis() / h;
            for row in 0..3 {
                assert!(
                    (jac[(row, col)] - dp[row]).abs() < 1e-5,
                    "linear jacobian mismatch at ({row},{col}): {} vs {}",
                    jac[(row, col)],
                    dp[row]
                );
                assert!(
                    (jac[(row + 3, col)] - dr[row]).abs() < 1e-5,
                    "angular jacobian mismatch at ({},{col}): {} vs {}",
                    row + 3,
                    jac[(row + 3, col)],
                    dr[row]
                );
            }
        }
    }

    #[test]
    fn ik_reaches_fk_poses() {
        let model = six_dof();
        let tool = model.link_index("tool0").unwrap();
        let targets = [
            [0.4, -0.9, 1.2, 0.3, 0.8, -0.5],
            [-1.2, 0.6, -0.7, 1.0, -1.4, 0.2],
            [2.0, 1.1, 0.4, -2.0, 0.9, 1.5],
        ];
        for q_true in targets {
            let target = forward_kinematics(&model, &q_true).unwrap()[tool];
            let seed = model.neutral_positions();
            let result = solve_ik(&model, tool, &target, &seed, &IkOptions::default()).unwrap();
            assert!(
                result.converged,
                "IK failed for {q_true:?}: pos_err={}, rot_err={}",
                result.pos_error, result.rot_error
            );
            // Verify with FK: the found q must realize the target pose
            // (it need not equal q_true).
            let reached = forward_kinematics(&model, &result.q).unwrap()[tool];
            let (e_pos, e_rot) = pose_error(&reached, &target);
            assert!(e_pos.norm() < 1e-4 && e_rot.norm() < 1e-3);
        }
    }

    #[test]
    fn escapes_fully_extended_singularity() {
        // At q = 0 the arm points straight up and every linear Jacobian
        // column is horizontal, so a straight-down target used to stall the
        // solver with dq = 0. The stall kick must break the symmetry.
        let model = six_dof();
        let tool = model.link_index("tool0").unwrap();
        let target =
            Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.6), UnitQuaternion::identity());
        let result = solve_ik(
            &model,
            tool,
            &target,
            &model.neutral_positions(),
            &IkOptions::default(),
        )
        .unwrap();
        assert!(
            result.converged,
            "stalled: pos_err={}, rot_err={}, iters={}",
            result.pos_error, result.rot_error, result.iters
        );
    }

    #[test]
    fn ik_respects_joint_limits() {
        let model = six_dof();
        let tool = model.link_index("tool0").unwrap();
        let target =
            Isometry3::from_parts(Translation3::new(0.3, 0.3, 0.5), UnitQuaternion::identity());
        let result = solve_ik(
            &model,
            tool,
            &target,
            &model.neutral_positions(),
            &IkOptions::default(),
        )
        .unwrap();
        for (qi, limits) in result.q.iter().zip(model.actuated_joint_limits()) {
            if let Some((lo, hi)) = limits {
                assert!(*qi >= lo - 1e-12 && *qi <= hi + 1e-12);
            }
        }
    }

    #[test]
    fn unreachable_target_reports_best_effort() {
        let model = six_dof();
        let tool = model.link_index("tool0").unwrap();
        // Total reach is ~0.85m; ask for 2m away.
        let target =
            Isometry3::from_parts(Translation3::new(2.0, 0.0, 0.2), UnitQuaternion::identity());
        let options = IkOptions {
            mode: IkMode::Position,
            ..IkOptions::default()
        };
        let result = solve_ik(&model, tool, &target, &model.neutral_positions(), &options).unwrap();
        assert!(!result.converged);
        // Best effort: closer than the neutral configuration was.
        assert!(result.pos_error < 1.6, "pos_error = {}", result.pos_error);
        assert!(result.pos_error > 0.9, "pos_error = {}", result.pos_error);
    }

    #[test]
    fn position_only_mode_on_planar_arm() {
        let urdf = r#"
        <robot name="planar">
          <link name="base"/><link name="l1"/><link name="l2"/><link name="tip"/>
          <joint name="q1" type="revolute">
            <parent link="base"/><child link="l1"/>
            <axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
          </joint>
          <joint name="q2" type="revolute">
            <parent link="l1"/><child link="l2"/>
            <origin xyz="1 0 0"/><axis xyz="0 0 1"/>
            <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
          </joint>
          <joint name="t" type="fixed">
            <parent link="l2"/><child link="tip"/><origin xyz="1 0 0"/>
          </joint>
        </robot>"#;
        let model = RobotModel::from_urdf_str(urdf).unwrap();
        let tip = model.link_index("tip").unwrap();
        let target =
            Isometry3::from_parts(Translation3::new(1.0, 1.0, 0.0), UnitQuaternion::identity());
        let options = IkOptions {
            mode: IkMode::Position,
            ..IkOptions::default()
        };
        // Seed away from the straight-out singularity at q = 0.
        let result = solve_ik(&model, tip, &target, &[0.3, 0.3], &options).unwrap();
        assert!(result.converged, "pos_error = {}", result.pos_error);
        let reached = forward_kinematics(&model, &result.q).unwrap()[tip];
        assert!((reached.translation.vector - target.translation.vector).norm() < 1e-4);
    }
}

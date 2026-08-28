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
    /// Position + tool-axis alignment (5 DOF task): the link frame's local
    /// +Z is driven onto the target frame's +Z; rotation about that axis is
    /// left free. The task rows span only the plane perpendicular to the
    /// axis, so the spin direction genuinely lives in the task null space —
    /// the joint-centering secondary objective places it, which is the whole
    /// point for axis-symmetric tools (milling cutters, dispensing nozzles).
    Axis,
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
    /// Additional attempts when the seeded solve does not converge — the
    /// escape hatch for seeds pinned against joint limits or parked on a
    /// singularity (a robot whose limits exclude zero starts exactly
    /// there). The first restart seeds from the limits midpoint, later
    /// ones from uniform samples within the sampling bounds, all drawn
    /// from a fixed-seed generator: the same call returns the same answer,
    /// every time. `0` disables restarts — the streaming/tracking
    /// configuration, where a warm-seeded solve jumping to a different
    /// solution branch would teleport the arm.
    pub restarts: usize,
    /// Gain of the joint-centering secondary objective, applied through an
    /// exact (SVD-based) projector onto the task null space each iteration.
    /// Redundant arms drift toward mid-range along their self-motion
    /// manifold instead of camping on joint limits; on a full-rank 6-DOF
    /// task the null space is empty and the term vanishes. The projection
    /// is exact, so the secondary objective cannot disturb the task even
    /// near singularities. `0` disables.
    pub null_space_gain: f64,
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
            restarts: 4,
            null_space_gain: 0.1,
        }
    }
}

impl IkOptions {
    /// Settings for interactive (per-frame, warm-seeded) solving: looser
    /// tolerances, but enough iterations to settle within a single message
    /// (a 6-DOF solve iteration is microseconds; the badge shown to the
    /// user reflects the final solve of a drag, so it must not stop short).
    /// No restarts: a per-frame solve must stay on its solution branch.
    pub fn streaming() -> Self {
        IkOptions {
            max_iters: 100,
            tol_pos: 1e-4,
            tol_rot: 1e-3,
            restarts: 0,
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
    for (ji, joint) in model.joints.iter().enumerate() {
        if !on_chain[ji] {
            continue;
        }
        // A mimic joint moves with its source at `multiplier` times the
        // rate, so its twist belongs in the source's column — the chain
        // rule, not a column of its own (it has no DOF to own one).
        let (driver, scale) = match joint.mimic {
            Some(m) => (m.source_joint, m.multiplier),
            None => (ji, 1.0),
        };
        let Some(col) = model.joints[driver].q_index else {
            continue;
        };
        // The joint frame coincides with the child link frame; rotation about
        // its own axis leaves the axis invariant, so the child pose rotation
        // maps the local axis to world coordinates.
        let world_axis = poses[joint.child_link].rotation * joint.axis.into_inner();
        match joint.joint_type {
            JointType::Revolute | JointType::Continuous => {
                let p_joint = poses[joint.child_link].translation.vector;
                let linear = world_axis.cross(&(p_end - p_joint)) * scale;
                let angular = world_axis * scale;
                let mut lin = jac.fixed_view_mut::<3, 1>(0, col);
                lin += linear;
                let mut ang = jac.fixed_view_mut::<3, 1>(3, col);
                ang += angular;
            }
            JointType::Prismatic => {
                let mut lin = jac.fixed_view_mut::<3, 1>(0, col);
                lin += world_axis * scale;
            }
            JointType::Fixed => {}
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

/// Normalized pull toward each limited joint's mid-range, in [-1, 1] per
/// joint; joints without limits (continuous) contribute nothing. `None`
/// when no joint has limits — there is no center to steer toward.
fn centering_direction(model: &RobotModel, q: &[f64]) -> Option<DVector<f64>> {
    let mut z = DVector::zeros(q.len());
    let mut any = false;
    for (i, &ji) in model.actuated_joints.iter().enumerate() {
        if let Some(l) = model.joints[ji].limits {
            let half = 0.5 * (l.upper - l.lower);
            if half > 1e-9 {
                let mid = 0.5 * (l.upper + l.lower);
                z[i] = ((mid - q[i]) / half).clamp(-1.0, 1.0);
                any = true;
            }
        }
    }
    any.then_some(z)
}

/// The objective the null-space term descends: sum of squared normalized
/// offsets from mid-range over the limited joints.
fn centering_measure(model: &RobotModel, q: &[f64]) -> f64 {
    q.iter()
        .zip(&model.actuated_joints)
        .map(|(qi, &ji)| match model.joints[ji].limits {
            Some(l) => {
                let half = 0.5 * (l.upper - l.lower);
                if half > 1e-9 {
                    let mid = 0.5 * (l.upper + l.lower);
                    ((qi - mid) / half).powi(2)
                } else {
                    0.0
                }
            }
            None => 0.0,
        })
        .sum()
}

/// Component of `z` in the null space of `jac`: z minus its projection onto
/// the row space, built from the right singular vectors. Exact (orthogonal)
/// projection — unlike the damped pseudo-inverse, nothing leaks into the
/// task even when singular values approach zero, because near-null
/// directions are simply *kept* rather than divided by sigma.
fn project_to_null_space(jac: DMatrix<f64>, z: &DVector<f64>) -> DVector<f64> {
    let svd = jac.svd(false, true);
    let mut out = z.clone();
    if let Some(vt) = svd.v_t {
        for (i, sigma) in svd.singular_values.iter().enumerate() {
            // Rows with vanishing sigma span (numerically) the null space;
            // only genuine row-space directions are subtracted.
            if *sigma > 1e-6 {
                let row = vt.row(i);
                let coeff = (row * z)[(0, 0)];
                out.axpy(-coeff, &row.transpose(), 1.0);
            }
        }
    }
    out
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

/// Rotation vector (axis · angle) that carries `a_cur` onto `a_tgt` along
/// the geodesic. Perpendicular to `a_cur` by construction. The antiparallel
/// case has no preferred geodesic; a deterministic perpendicular is chosen
/// so the solver still makes progress instead of stalling on a zero cross
/// product.
fn axis_alignment_error(a_cur: &Vector3<f64>, a_tgt: &Vector3<f64>) -> Vector3<f64> {
    let cross = a_cur.cross(a_tgt);
    let dot = a_cur.dot(a_tgt).clamp(-1.0, 1.0);
    let angle = cross.norm().atan2(dot);
    if angle < 1e-12 {
        return Vector3::zeros();
    }
    let n = if cross.norm() > 1e-9 {
        cross / cross.norm()
    } else {
        perpendicular_to(a_cur)
    };
    n * angle
}

/// A deterministic unit vector perpendicular to `a`, from the world basis
/// vector least aligned with it.
fn perpendicular_to(a: &Vector3<f64>) -> Vector3<f64> {
    let pick = if a.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    (pick - a * pick.dot(a)).normalize()
}

/// Solves for a configuration placing `link` at `target`, starting from
/// `seed`. When the seeded solve does not converge, up to
/// [`IkOptions::restarts`] deterministically-seeded attempts follow.
/// Always returns the best configuration found; check `converged`.
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
    let mut rng: u64 = 0x9E37_79B9_7F4A_7C15;
    let mut best = solve_attempt(model, link, target, seed, options, &mut rng)?;
    if best.converged {
        return Ok(best);
    }
    let score = |r: &IkResult| r.pos_error + options.orientation_weight * r.rot_error;
    let (lower, upper) = model.sampling_bounds();
    for restart in 0..options.restarts {
        let seed: Vec<f64> = lower
            .iter()
            .zip(&upper)
            .map(|(lo, hi)| {
                if restart == 0 {
                    // The centered configuration first: it is the analytic
                    // antidote to a seed clamped against its limits.
                    0.5 * (lo + hi)
                } else {
                    lo + 0.5 * (jitter_unit(&mut rng) + 1.0) * (hi - lo)
                }
            })
            .collect();
        let result = solve_attempt(model, link, target, &seed, options, &mut rng)?;
        if result.converged || score(&result) < score(&best) {
            best = result;
        }
        if best.converged {
            break;
        }
    }
    Ok(best)
}

/// One damped-least-squares descent from `seed`; `rng` feeds the
/// singularity-stall kick and is threaded through so successive attempts
/// stay deterministic as a sequence.
fn solve_attempt(
    model: &RobotModel,
    link: usize,
    target: &Isometry3<f64>,
    seed: &[f64],
    options: &IkOptions,
    rng: &mut u64,
) -> Result<IkResult, KinError> {
    let task_dim = match options.mode {
        IkMode::Position => 3,
        IkMode::Pose => 6,
        IkMode::Axis => 5,
    };
    let lambda2 = options.damping * options.damping;

    let mut q = seed.to_vec();
    clamp_to_limits(model, &mut q);

    let mut best: Option<IkResult> = None;
    for iter in 0..=options.max_iters {
        let poses = forward_kinematics(model, &q)?;
        let (e_pos, e_rot) = pose_error(&poses[link], target);
        // In axis mode the rotational task is the geodesic carrying the
        // link's local +Z onto the target's +Z; spin about it is not error.
        let a_cur = poses[link].rotation * Vector3::z();
        let e_rot = match options.mode {
            IkMode::Axis => axis_alignment_error(&a_cur, &(target.rotation * Vector3::z())),
            _ => e_rot,
        };
        let pos_error = e_pos.norm();
        let rot_error = match options.mode {
            IkMode::Position => 0.0,
            IkMode::Pose | IkMode::Axis => e_rot.norm(),
        };
        let converged = pos_error < options.tol_pos
            && (options.mode == IkMode::Position || rot_error < options.tol_rot);

        // Track the best configuration: weighted task error decides; among
        // converged iterates (the task is met either way) the better-centered
        // one wins — that is what the settling iterations below produce.
        let score = pos_error + options.orientation_weight * rot_error;
        let replace = match &best {
            None => true,
            Some(b) if converged && b.converged => {
                centering_measure(model, &q) < centering_measure(model, &b.q)
            }
            Some(b) if converged != b.converged => converged,
            Some(b) => score < b.pos_error + options.orientation_weight * b.rot_error,
        };
        if replace {
            best = Some(IkResult {
                q: q.clone(),
                converged,
                pos_error,
                rot_error,
                iters: iter,
            });
        }
        if iter == options.max_iters || (converged && options.null_space_gain <= 0.0) {
            break;
        }

        let jac_full = jacobian(model, &poses, link);
        let mut e = DVector::zeros(task_dim);
        e.fixed_rows_mut::<3>(0).copy_from(&e_pos);
        let jac = match options.mode {
            IkMode::Pose => {
                e.fixed_rows_mut::<3>(3)
                    .copy_from(&(options.orientation_weight * e_rot));
                let mut j = jac_full;
                j.rows_mut(3, 3).scale_mut(options.orientation_weight);
                j
            }
            IkMode::Axis => {
                // Two angular rows spanning the plane perpendicular to the
                // current axis. The spin direction (angular velocity along
                // the axis) maps to zero here, so it stays in the null
                // space for the centering term instead of being pinned by
                // damping as a full 3-row task would do.
                let u = perpendicular_to(&a_cur);
                let v = a_cur.cross(&u);
                let w = options.orientation_weight;
                e[3] = w * u.dot(&e_rot);
                e[4] = w * v.dot(&e_rot);
                let jw = jac_full.rows(3, 3);
                let mut j = DMatrix::zeros(5, jac_full.ncols());
                j.rows_mut(0, 3).copy_from(&jac_full.rows(0, 3));
                for col in 0..jac_full.ncols() {
                    let wcol = Vector3::new(jw[(0, col)], jw[(1, col)], jw[(2, col)]);
                    j[(3, col)] = w * u.dot(&wcol);
                    j[(4, col)] = w * v.dot(&wcol);
                }
                j
            }
            IkMode::Position => jac_full.rows(0, 3).into_owned(),
        };

        // dq = J^T (J J^T + lambda^2 I)^-1 e
        let jjt = &jac * jac.transpose() + DMatrix::identity(task_dim, task_dim) * lambda2;
        let Some(chol) = jjt.cholesky() else { break };
        let mut dq = jac.transpose() * chol.solve(&e);
        if !converged && dq.norm() < 1e-9 {
            // The *task* step stalled at a singularity (e.g. a fully
            // extended arm asked to move along its own axis produces
            // dq = 0): kick the configuration to break the symmetry, then
            // keep iterating. Checked before the null-space term so the
            // secondary objective cannot mask a stalled task.
            for qi in q.iter_mut() {
                *qi += 0.05 * jitter_unit(rng);
            }
            clamp_to_limits(model, &mut q);
            continue;
        }
        // Secondary objective: joint centering through the task null space.
        // The row scaling above (orientation weight) does not change the
        // row space, so the projector is unaffected by it. Once the task
        // has converged, iterations continue on this term alone (the task
        // rows keep the pose pinned) until the centering pull has no
        // null-space component left — the settling phase that actually
        // moves a redundant arm toward mid-range.
        let mut ns_step = 0.0;
        if options.null_space_gain > 0.0 {
            if let Some(z) = centering_direction(model, &q) {
                let ns = project_to_null_space(jac, &z);
                ns_step = options.null_space_gain * ns.norm();
                dq.axpy(options.null_space_gain, &ns, 1.0);
            }
        }
        if converged && ns_step < 1e-6 {
            break;
        }
        let step = dq.norm();
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

    const SIX_DOF: &str = include_str!("../../../examples/assets/simple_arm.urdf");

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

    /// Planar 2R whose second joint follows the first at half rate: one
    /// DOF, two moving joints on the chain to the tool.
    const MIMIC_CHAIN: &str = r#"
    <robot name="coupled">
      <link name="base"/><link name="link1"/><link name="link2"/><link name="tool"/>
      <joint name="q1" type="revolute">
        <parent link="base"/><child link="link1"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3" upper="3" effort="1" velocity="1"/>
      </joint>
      <joint name="q2" type="revolute">
        <parent link="link1"/><child link="link2"/>
        <origin xyz="1 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3" upper="3" effort="1" velocity="1"/>
        <mimic joint="q1" multiplier="0.5" offset="0.1"/>
      </joint>
      <joint name="tip" type="fixed">
        <parent link="link2"/><child link="tool"/>
        <origin xyz="1 0 0"/>
      </joint>
    </robot>"#;

    #[test]
    fn jacobian_folds_mimic_joints_into_their_source() {
        let model = RobotModel::from_urdf_str(MIMIC_CHAIN).unwrap();
        assert_eq!(model.dof(), 1);
        let tool = model.link_index("tool").unwrap();
        let q = [0.35];
        let poses = forward_kinematics(&model, &q).unwrap();
        let jac = jacobian(&model, &poses, tool);

        // Finite differences over the single DOF move both joints, so a
        // Jacobian that ignored the mimic term would be off by its share.
        let h = 1e-7;
        let poses_p = forward_kinematics(&model, &[q[0] + h]).unwrap();
        let dp = (poses_p[tool].translation.vector - poses[tool].translation.vector) / h;
        let dr = (poses_p[tool].rotation * poses[tool].rotation.inverse()).scaled_axis() / h;
        for row in 0..3 {
            assert!(
                (jac[(row, 0)] - dp[row]).abs() < 1e-5,
                "linear mismatch at {row}: {} vs {}",
                jac[(row, 0)],
                dp[row]
            );
            assert!(
                (jac[(row + 3, 0)] - dr[row]).abs() < 1e-5,
                "angular mismatch at {row}: {} vs {}",
                jac[(row + 3, 0)],
                dr[row]
            );
        }
        // The coupled joint turns 1.5x as fast as the DOF alone would.
        assert!((jac[(5, 0)] - 1.5).abs() < 1e-9, "{}", jac[(5, 0)]);
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

    /// The Franka FR3 kinematics (joint frames/limits from the catalog
    /// URDF, geometry dropped): joints 4 and 6 have limits excluding zero,
    /// so the neutral configuration clamps onto two limits at once — and
    /// the plain descent pins there instead of converging. This is the
    /// failure observed live against the model catalog.
    const LIMIT_PINNED: &str = r#"
    <robot name="fr3_frames">
      <link name="l0"/><link name="l1"/><link name="l2"/><link name="l3"/>
      <link name="l4"/><link name="l5"/><link name="l6"/><link name="l7"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="l0"/><child link="l1"/>
        <origin xyz="0 0 0.333"/><axis xyz="0 0 1"/>
        <limit lower="-2.9007" upper="2.9007" effort="87" velocity="2.62"/>
      </joint>
      <joint name="j2" type="revolute">
        <parent link="l1"/><child link="l2"/>
        <origin rpy="-1.57079633 0 0"/><axis xyz="0 0 1"/>
        <limit lower="-1.8361" upper="1.8361" effort="87" velocity="2.62"/>
      </joint>
      <joint name="j3" type="revolute">
        <parent link="l2"/><child link="l3"/>
        <origin xyz="0 -0.316 0" rpy="1.57079633 0 0"/><axis xyz="0 0 1"/>
        <limit lower="-2.9007" upper="2.9007" effort="87" velocity="2.62"/>
      </joint>
      <joint name="j4" type="revolute">
        <parent link="l3"/><child link="l4"/>
        <origin xyz="0.0825 0 0" rpy="1.57079633 0 0"/><axis xyz="0 0 1"/>
        <limit lower="-3.077" upper="-0.1169" effort="87" velocity="2.62"/>
      </joint>
      <joint name="j5" type="revolute">
        <parent link="l4"/><child link="l5"/>
        <origin xyz="-0.0825 0.384 0" rpy="-1.57079633 0 0"/><axis xyz="0 0 1"/>
        <limit lower="-2.8763" upper="2.8763" effort="12" velocity="5.26"/>
      </joint>
      <joint name="j6" type="revolute">
        <parent link="l5"/><child link="l6"/>
        <origin rpy="1.57079633 0 0"/><axis xyz="0 0 1"/>
        <limit lower="0.4398" upper="4.6216" effort="12" velocity="4.18"/>
      </joint>
      <joint name="j7" type="revolute">
        <parent link="l6"/><child link="l7"/>
        <origin xyz="0.088 0 0" rpy="1.57079633 0 0"/><axis xyz="0 0 1"/>
        <limit lower="-3.0508" upper="3.0508" effort="12" velocity="5.26"/>
      </joint>
      <joint name="j8" type="fixed">
        <parent link="l7"/><child link="tip"/>
        <origin xyz="0 0 0.107"/>
      </joint>
    </robot>"#;

    #[test]
    fn restarts_rescue_a_limit_pinned_seed() {
        let model = RobotModel::from_urdf_str(LIMIT_PINNED).unwrap();
        let tip = model.link_index("tip").unwrap();
        // The exact configuration the live catalog check used.
        let q_true = [0.3, -0.6, 0.2, -1.8, 0.15, 1.6, 0.5];
        let target = forward_kinematics(&model, &q_true).unwrap()[tip];
        let seed = model.neutral_positions();

        // The plain descent must genuinely fail here, or this test guards
        // nothing. Null-space centering is disabled because it rescues this
        // fixture on its own (covered by its dedicated test below).
        let plain = IkOptions {
            restarts: 0,
            null_space_gain: 0.0,
            ..IkOptions::default()
        };
        let stuck = solve_ik(&model, tip, &target, &seed, &plain).unwrap();
        assert!(!stuck.converged, "fixture no longer pins: {stuck:?}");

        let rescued = solve_ik(
            &model,
            tip,
            &target,
            &seed,
            &IkOptions {
                null_space_gain: 0.0,
                ..IkOptions::default()
            },
        )
        .unwrap();
        assert!(rescued.converged, "{rescued:?}");
        let reached = forward_kinematics(&model, &rescued.q).unwrap()[tip];
        let (e_pos, e_rot) = pose_error(&reached, &target);
        assert!(e_pos.norm() < 1e-4 && e_rot.norm() < 1e-3);
    }

    /// On a redundant arm the centering term must (a) not disturb the
    /// reached pose and (b) leave the joints measurably closer to
    /// mid-range than the uncentered solve from the same seed.
    #[test]
    fn null_space_centers_redundant_joints_without_moving_the_task() {
        let model = RobotModel::from_urdf_str(LIMIT_PINNED).unwrap();
        let tip = model.link_index("tip").unwrap();
        let q_true = [0.3, -0.6, 0.2, -1.8, 0.15, 1.6, 0.5];
        let target = forward_kinematics(&model, &q_true).unwrap()[tip];
        // A seed that already converges without help, so the two runs are
        // comparable descents rather than a rescue story.
        let seed = [0.2, -0.4, 0.1, -1.5, 0.1, 1.2, 0.3];

        let centering = |q: &[f64]| -> f64 {
            q.iter()
                .zip(model.actuated_joint_limits())
                .map(|(qi, l)| {
                    let (lo, hi) = l.unwrap();
                    let mid = 0.5 * (lo + hi);
                    ((qi - mid) / (0.5 * (hi - lo))).powi(2)
                })
                .sum()
        };

        let plain_opts = IkOptions {
            restarts: 0,
            null_space_gain: 0.0,
            ..IkOptions::default()
        };
        let ns_opts = IkOptions {
            restarts: 0,
            ..IkOptions::default()
        };
        let plain = solve_ik(&model, tip, &target, &seed, &plain_opts).unwrap();
        let centered = solve_ik(&model, tip, &target, &seed, &ns_opts).unwrap();
        assert!(plain.converged, "{plain:?}");
        assert!(centered.converged, "{centered:?}");

        // (a) task untouched: both land on the target to tolerance.
        let reached = forward_kinematics(&model, &centered.q).unwrap()[tip];
        let (e_pos, e_rot) = pose_error(&reached, &target);
        assert!(e_pos.norm() < 1e-4 && e_rot.norm() < 1e-3);

        // (b) secondary objective improved.
        assert!(
            centering(&centered.q) < centering(&plain.q),
            "centering did not improve: {} vs {}",
            centering(&centered.q),
            centering(&plain.q)
        );
    }

    #[test]
    fn null_space_solves_are_deterministic() {
        let model = RobotModel::from_urdf_str(LIMIT_PINNED).unwrap();
        let tip = model.link_index("tip").unwrap();
        let target =
            forward_kinematics(&model, &[0.3, -0.6, 0.2, -1.8, 0.15, 1.6, 0.5]).unwrap()[tip];
        let seed = model.neutral_positions();
        let options = IkOptions {
            restarts: 0,
            ..IkOptions::default()
        };
        let a = solve_ik(&model, tip, &target, &seed, &options).unwrap();
        let b = solve_ik(&model, tip, &target, &seed, &options).unwrap();
        assert_eq!(a.q, b.q);
        assert_eq!(a.iters, b.iters);
        assert_eq!(a.converged, b.converged);
    }

    #[test]
    fn restarts_are_deterministic() {
        let model = RobotModel::from_urdf_str(LIMIT_PINNED).unwrap();
        let tip = model.link_index("tip").unwrap();
        let target =
            forward_kinematics(&model, &[0.3, -0.6, 0.2, -1.8, 0.15, 1.6, 0.5]).unwrap()[tip];
        let seed = model.neutral_positions();
        let a = solve_ik(&model, tip, &target, &seed, &IkOptions::default()).unwrap();
        let b = solve_ik(&model, tip, &target, &seed, &IkOptions::default()).unwrap();
        // Bit-identical, not merely close: the restart seeds come from a
        // fixed-seed generator.
        assert_eq!(a.q, b.q);
        assert_eq!(a.iters, b.iters);
        assert_eq!(a.converged, b.converged);
    }

    #[test]
    fn streaming_solves_never_restart() {
        assert_eq!(IkOptions::streaming().restarts, 0);
    }

    #[test]
    fn axis_mode_reaches_position_and_aligns_axis() {
        let model = six_dof();
        let tool = model.link_index("tool0").unwrap();
        let options = IkOptions {
            mode: IkMode::Axis,
            ..IkOptions::default()
        };
        for q_true in [
            [0.4, -0.9, 1.2, 0.3, 0.8, -0.5],
            [-1.2, 0.6, -0.7, 1.0, -1.4, 0.2],
        ] {
            let target = forward_kinematics(&model, &q_true).unwrap()[tool];
            let result =
                solve_ik(&model, tool, &target, &model.neutral_positions(), &options).unwrap();
            assert!(
                result.converged,
                "axis IK failed for {q_true:?}: pos={}, rot={}",
                result.pos_error, result.rot_error
            );
            let reached = forward_kinematics(&model, &result.q).unwrap()[tool];
            assert!((reached.translation.vector - target.translation.vector).norm() < 1e-4);
            let a_reached = reached.rotation * Vector3::z();
            let a_target = target.rotation * Vector3::z();
            let angle = a_reached
                .cross(&a_target)
                .norm()
                .atan2(a_reached.dot(&a_target));
            assert!(angle < 1e-3, "axis misaligned by {angle}");
        }
    }

    #[test]
    fn axis_mode_ignores_target_spin() {
        // Two targets differing only by rotation about their own +Z must
        // produce the same solve — the spin is not part of the task.
        let model = six_dof();
        let tool = model.link_index("tool0").unwrap();
        let base = forward_kinematics(&model, &[0.4, -0.9, 1.2, 0.3, 0.8, -0.5]).unwrap()[tool];
        let spun = base
            * Isometry3::from_parts(
                nalgebra::Translation3::identity(),
                UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 1.234),
            );
        let options = IkOptions {
            mode: IkMode::Axis,
            ..IkOptions::default()
        };
        let seed = model.neutral_positions();
        let a = solve_ik(&model, tool, &base, &seed, &options).unwrap();
        let b = solve_ik(&model, tool, &spun, &seed, &options).unwrap();
        assert!(a.converged && b.converged);
        for (qa, qb) in a.q.iter().zip(&b.q) {
            assert!((qa - qb).abs() < 1e-6, "{:?} vs {:?}", a.q, b.q);
        }
    }

    #[test]
    fn axis_mode_null_space_centers_through_the_spin() {
        // A 6-DOF arm under a 5-row task keeps a one-dimensional null
        // space — the spin — which the centering term must exploit.
        let model = six_dof();
        let tool = model.link_index("tool0").unwrap();
        let q_true = [0.4, -0.9, 1.2, 0.3, 0.8, 1.4];
        let target = forward_kinematics(&model, &q_true).unwrap()[tool];
        let seed = [0.35, -0.85, 1.15, 0.25, 0.75, 1.35];
        let plain = IkOptions {
            mode: IkMode::Axis,
            restarts: 0,
            null_space_gain: 0.0,
            ..IkOptions::default()
        };
        let centered = IkOptions {
            mode: IkMode::Axis,
            restarts: 0,
            ..IkOptions::default()
        };
        let a = solve_ik(&model, tool, &target, &seed, &plain).unwrap();
        let b = solve_ik(&model, tool, &target, &seed, &centered).unwrap();
        assert!(a.converged && b.converged);
        assert!(
            centering_measure(&model, &b.q) < centering_measure(&model, &a.q),
            "spin centering did not improve: {} vs {}",
            centering_measure(&model, &b.q),
            centering_measure(&model, &a.q)
        );
        // The task itself is untouched by the secondary objective.
        let reached = forward_kinematics(&model, &b.q).unwrap()[tool];
        assert!((reached.translation.vector - target.translation.vector).norm() < 1e-4);
    }

    #[test]
    fn axis_alignment_error_handles_the_antiparallel_case() {
        let a = Vector3::z();
        let e = axis_alignment_error(&a, &(-Vector3::z()));
        assert!((e.norm() - std::f64::consts::PI).abs() < 1e-9, "{e:?}");
        assert!(e.dot(&a).abs() < 1e-9, "not perpendicular: {e:?}");
        // And exact alignment is a zero error, not NaN.
        assert_eq!(axis_alignment_error(&a, &Vector3::z()), Vector3::zeros());
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

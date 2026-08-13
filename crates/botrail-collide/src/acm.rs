//! Allowed collision matrix: link pairs excluded from self-collision checks.

use std::collections::HashSet;

use botrail_model::RobotModel;

#[derive(Debug, Clone, Default)]
pub struct Acm {
    allowed: HashSet<(usize, usize)>,
}

fn key(i: usize, j: usize) -> (usize, usize) {
    (i.min(j), i.max(j))
}

impl Acm {
    /// Default ACM: every pair of links directly connected by a joint is
    /// allowed to "collide" (they usually touch by construction).
    pub fn adjacent(model: &RobotModel) -> Self {
        let mut acm = Acm::default();
        for joint in &model.joints {
            acm.allow(joint.parent_link, joint.child_link);
        }
        acm
    }

    pub fn allow(&mut self, i: usize, j: usize) {
        self.allowed.insert(key(i, j));
    }

    pub fn allows(&self, i: usize, j: usize) -> bool {
        self.allowed.contains(&key(i, j))
    }

    pub fn allowed_pairs(&self) -> impl Iterator<Item = (usize, usize)> + '_ {
        self.allowed.iter().copied()
    }
}

/// Allowed collision pairs between links of *different* robots, keyed by
/// `(robot, link)`. Unlike the intra-robot [`Acm`], nothing is generated
/// automatically: inter-robot contact depends on the relative base poses
/// (and both configurations), so the identity-base sampling behind
/// [`detect_always_colliding`] does not apply. Pairs are stored with the
/// lower robot index first.
#[derive(Debug, Clone, Default)]
pub struct InterRobotAcm {
    allowed: HashSet<((usize, usize), (usize, usize))>,
}

fn inter_key(a: (usize, usize), b: (usize, usize)) -> ((usize, usize), (usize, usize)) {
    if a <= b {
        (a, b)
    } else {
        (b, a)
    }
}

impl InterRobotAcm {
    pub fn allow(&mut self, a: (usize, usize), b: (usize, usize)) {
        self.allowed.insert(inter_key(a, b));
    }

    pub fn allows(&self, a: (usize, usize), b: (usize, usize)) -> bool {
        self.allowed.contains(&inter_key(a, b))
    }

    pub fn allowed_pairs(&self) -> impl Iterator<Item = ((usize, usize), (usize, usize))> + '_ {
        self.allowed.iter().copied()
    }
}

/// Samples random configurations (deterministic xorshift) and returns link
/// pairs colliding in at least `threshold` of them — these are almost
/// certainly in contact by design and should be added to the ACM.
/// This is the core of a MoveIt-Setup-Assistant-style ACM generation.
pub fn detect_always_colliding(
    model: &RobotModel,
    robot: &crate::RobotCollider,
    acm: &Acm,
    samples: usize,
    threshold: f64,
) -> Vec<(usize, usize)> {
    let limits = model.actuated_joint_limits();
    let mut rng: u64 = 0x51_7C_C1_B7_27_22_0A_95;
    let mut unit = move || {
        rng ^= rng >> 12;
        rng ^= rng << 25;
        rng ^= rng >> 27;
        (rng.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
    };

    let mut counts: std::collections::HashMap<(usize, usize), usize> =
        std::collections::HashMap::new();
    for _ in 0..samples {
        let q: Vec<f64> = limits
            .iter()
            .map(|l| match l {
                Some((lo, hi)) => lo + unit() * (hi - lo),
                None => (unit() - 0.5) * 2.0 * std::f64::consts::PI,
            })
            .collect();
        let poses = match botrail_kin::forward_kinematics(model, &q) {
            Ok(p) => p,
            Err(_) => continue,
        };
        let query = crate::RobotQuery {
            collider: robot,
            link_poses: &poses,
            acm,
        };
        for pair in crate::check_scene(
            &[query],
            &InterRobotAcm::default(),
            &[],
            &[],
            &crate::ContactAllowance::default(),
        ) {
            if let (
                crate::ColliderId::Link { link: i, .. },
                crate::ColliderId::Link { link: j, .. },
            ) = (pair.a, pair.b)
            {
                *counts.entry(key(i, j)).or_default() += 1;
            }
        }
    }
    let needed = (samples as f64 * threshold).ceil() as usize;
    let mut result: Vec<(usize, usize)> = counts
        .into_iter()
        .filter(|(_, c)| *c >= needed)
        .map(|(k, _)| k)
        .collect();
    result.sort_unstable();
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::RobotCollider;

    #[test]
    fn always_colliding_pair_is_detected() {
        // Two boxes welded together overlapping, plus a third link far away
        // on a revolute joint that never reaches them.
        let urdf = r#"
        <robot name="welded">
          <link name="a"><visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual></link>
          <link name="hop"/>
          <link name="b"><visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual></link>
          <link name="c"><visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link>
          <joint name="a_hop" type="fixed">
            <parent link="a"/><child link="hop"/>
          </joint>
          <joint name="hop_b" type="fixed">
            <parent link="hop"/><child link="b"/>
            <origin xyz="0.05 0 0"/>
          </joint>
          <joint name="a_c" type="revolute">
            <parent link="a"/><child link="c"/>
            <origin xyz="2 0 0"/>
            <axis xyz="0 0 1"/>
            <limit lower="-3" upper="3" effort="1" velocity="1"/>
          </joint>
        </robot>"#;
        let model = botrail_model::RobotModel::from_urdf_str(urdf).unwrap();
        let (collider, _) = RobotCollider::from_model(&model);
        let acm = Acm::adjacent(&model);

        let a = model.link_index("a").unwrap();
        let b = model.link_index("b").unwrap();
        // a and b are NOT adjacent (connected through `hop`), always overlap.
        let always = detect_always_colliding(&model, &collider, &acm, 64, 0.95);
        assert_eq!(always, vec![key(a, b)]);
    }
}

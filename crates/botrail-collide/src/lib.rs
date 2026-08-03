//! Collision checking for botrail on top of parry3d.
//!
//! Shape policy (from `design/bench-parry3d.md`): only *solid* parry shapes
//! are used — primitives map directly, meshes map to VHACD convex compounds
//! (see [`mesh`], with a content-addressed disk cache). Raw `TriMesh` is
//! never used as collision geometry because parry's surface semantics
//! silently miss full containment and report bogus distances for contained
//! shapes.
//!
//! Math boundary: the public API speaks nalgebra (`Isometry3<f64>`, matching
//! the rest of botrail); parry >= 0.23 is glam-based, so poses are converted
//! at the query boundary (a quat + vec copy).

mod acm;
mod convert;
pub mod mesh;

use botrail_model::RobotModel;
use nalgebra::Isometry3;
use parry3d_f64::math::Pose;
use parry3d_f64::query;
use parry3d_f64::shape::SharedShape;
use thiserror::Error;

pub use acm::{detect_always_colliding, Acm, InterRobotAcm};
pub use convert::to_parry_pose;

#[derive(Debug, Error)]
pub enum CollideError {
    #[error("unsupported collision geometry: {0}")]
    UnsupportedGeometry(String),
    #[error("mesh collision shape failed: {0}")]
    MeshLoad(String),
}

/// A shape set in a local frame: `(local_pose, shape)` pairs.
type Parts = Vec<(Pose, SharedShape)>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ColliderId {
    /// A robot link: `robot` indexes the `robots` slice passed to
    /// [`check_scene`], `link` the model's links.
    Link {
        robot: usize,
        link: usize,
    },
    Obstacle(usize),
    /// Index into the `attached` slice passed to [`check_scene`].
    Attached(usize),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CollisionPair {
    pub a: ColliderId,
    pub b: ColliderId,
}

/// One robot's collision inputs for [`check_scene`] /
/// [`min_robot_obstacle_distance`]: its colliders, its world link poses
/// (aligned with the model's links), and its intra-robot ACM.
pub struct RobotQuery<'a> {
    pub collider: &'a RobotCollider,
    pub link_poses: &'a [Isometry3<f64>],
    pub acm: &'a Acm,
}

/// An obstacle rigidly attached to a robot link (a grasped object): its
/// world pose is `robots[robot].link_poses[link] * offset` at query time, so
/// it moves with that robot. `skip_links` are links *of the carrying robot*
/// allowed to touch it — the carrying link and e.g. gripper fingers; links
/// of other robots are always checked.
pub struct AttachedCollider<'a> {
    pub robot: usize,
    pub link: usize,
    pub offset: Isometry3<f64>,
    pub collider: &'a ObstacleCollider,
    pub skip_links: &'a [usize],
}

/// Builds obstacle colliders for a batch of geometries — in parallel under
/// the `parallel` feature, where mesh VHACD decomposition dominates.
pub fn build_obstacle_colliders(
    geometries: &[botrail_model::Geometry],
) -> Vec<Result<ObstacleCollider, CollideError>> {
    #[cfg(feature = "parallel")]
    {
        use rayon::prelude::*;
        geometries
            .par_iter()
            .map(ObstacleCollider::from_geometry)
            .collect()
    }
    #[cfg(not(feature = "parallel"))]
    {
        geometries
            .iter()
            .map(ObstacleCollider::from_geometry)
            .collect()
    }
}

/// Per-link collision shapes derived from a robot model.
#[derive(Clone)]
pub struct RobotCollider {
    links: Vec<Parts>,
}

impl RobotCollider {
    /// Builds colliders from each link's `<collision>` geometry, falling
    /// back to `<visual>` when a link declares none (common in simple
    /// URDFs). Shapes that fail to convert (e.g. unreadable mesh files) are
    /// skipped with a warning rather than failing the whole robot.
    pub fn from_model(model: &RobotModel) -> (Self, Vec<String>) {
        let build = |link: &botrail_model::Link| -> (Parts, Vec<String>) {
            let source = if link.collisions.is_empty() {
                &link.visuals
            } else {
                &link.collisions
            };
            let mut warnings = Vec::new();
            let parts = source
                .iter()
                .filter_map(|shape| match convert::shape_to_parry(shape) {
                    Ok(part) => Some(part),
                    Err(e) => {
                        warnings.push(format!(
                            "link `{}`: {e}; shape ignored for collision",
                            link.name
                        ));
                        None
                    }
                })
                .collect();
            (parts, warnings)
        };
        // Mesh shapes VHACD-decompose on first load (~1s each), so links
        // build in parallel when the feature is on.
        #[cfg(feature = "parallel")]
        let results: Vec<(Parts, Vec<String>)> = {
            use rayon::prelude::*;
            model.links.par_iter().map(build).collect()
        };
        #[cfg(not(feature = "parallel"))]
        let results: Vec<(Parts, Vec<String>)> = model.links.iter().map(build).collect();

        let mut links = Vec::with_capacity(results.len());
        let mut warnings = Vec::new();
        for (parts, mut link_warnings) in results {
            links.push(parts);
            warnings.append(&mut link_warnings);
        }
        (RobotCollider { links }, warnings)
    }

    pub fn link_has_geometry(&self, link: usize) -> bool {
        !self.links[link].is_empty()
    }
}

/// Collision shapes for one obstacle (world pose supplied per query).
#[derive(Clone)]
pub struct ObstacleCollider {
    parts: Parts,
}

impl ObstacleCollider {
    pub fn from_geometry(geometry: &botrail_model::Geometry) -> Result<Self, CollideError> {
        let (offset, shape) = convert::geometry_to_parry(geometry)?;
        Ok(ObstacleCollider {
            parts: vec![(offset, shape)],
        })
    }

    /// Wraps an already-built shape (e.g. [`mesh::mesh_to_compound`] on
    /// in-memory mesh data, where no file path exists to load from).
    pub fn from_shape(shape: SharedShape) -> Self {
        ObstacleCollider {
            parts: vec![(Pose::identity(), shape)],
        }
    }

    /// A solid box (pseudo-sensor zones).
    pub fn cuboid(half_extents: nalgebra::Vector3<f64>) -> Self {
        Self::from_shape(SharedShape::cuboid(
            half_extents.x,
            half_extents.y,
            half_extents.z,
        ))
    }

    /// A capsule spanning two local points (pseudo photoelectric beams).
    pub fn capsule(a: nalgebra::Point3<f64>, b: nalgebra::Point3<f64>, radius: f64) -> Self {
        let p = |p: nalgebra::Point3<f64>| parry3d_f64::math::Vector::new(p.x, p.y, p.z);
        Self::from_shape(SharedShape::capsule(p(a), p(b), radius))
    }

    /// Overlap test between two colliders at world poses (boolean contact,
    /// the semantics a presence sensor or light beam wants).
    pub fn intersects(
        &self,
        pose: &Isometry3<f64>,
        other: &ObstacleCollider,
        other_pose: &Isometry3<f64>,
    ) -> bool {
        parts_intersect(
            &to_parry_pose(pose),
            &self.parts,
            &to_parry_pose(other_pose),
            &other.parts,
        )
    }
}

/// True when any robot link overlaps `collider` (light-curtain style robot
/// sensing). `link_poses` must align with the model's links.
pub fn robot_intersects(
    robot: &RobotCollider,
    link_poses: &[Isometry3<f64>],
    collider: &ObstacleCollider,
    pose: &Isometry3<f64>,
) -> bool {
    let p = to_parry_pose(pose);
    robot
        .links
        .iter()
        .zip(link_poses)
        .any(|(parts, link_pose)| {
            !parts.is_empty()
                && parts_intersect(&to_parry_pose(link_pose), parts, &p, &collider.parts)
        })
}

fn parts_intersect(pose_a: &Pose, a: &Parts, pose_b: &Pose, b: &Parts) -> bool {
    for (la, sa) in a {
        let wa = *pose_a * *la;
        for (lb, sb) in b {
            let wb = *pose_b * *lb;
            if query::intersection_test(&wa, sa.as_ref(), &wb, sb.as_ref()).unwrap_or(false) {
                return true;
            }
        }
    }
    false
}

/// Pairs farther apart than this are skipped in distance queries (meters).
const DISTANCE_PREDICTION: f64 = 100.0;

fn parts_distance(pose_a: &Pose, a: &Parts, pose_b: &Pose, b: &Parts) -> Option<f64> {
    let mut min = None;
    for (la, sa) in a {
        let wa = *pose_a * *la;
        for (lb, sb) in b {
            let wb = *pose_b * *lb;
            // `query::contact` instead of `query::distance`: the latter's
            // GJK path overestimates by ~0.35% in the degenerate
            // parallel-face case (parry 0.29), while contact() is exact.
            // Overestimating clearance is the unsafe direction, so pay the
            // small extra cost. Negative dist (penetration) clamps to 0.
            if let Ok(Some(c)) =
                query::contact(&wa, sa.as_ref(), &wb, sb.as_ref(), DISTANCE_PREDICTION)
            {
                let d = c.dist.max(0.0);
                min = Some(min.map_or(d, |m: f64| m.min(d)));
            }
        }
    }
    min
}

/// Checks, for every robot in `robots`: self-collision (non-ACM link
/// pairs), robot-vs-robot collision (all cross-robot link pairs minus
/// `inter_acm`), robot-vs-obstacle collision, and attached-object collision.
/// Each robot's `link_poses` must align with its model's links; obstacles
/// are `(world_pose, collider)` pairs whose index becomes
/// `ColliderId::Obstacle`; attached objects (index → `ColliderId::Attached`)
/// ride their carrying robot's link and are checked against all robots'
/// links (minus the carrier's `skip_links`), obstacles, and each other.
pub fn check_scene(
    robots: &[RobotQuery<'_>],
    inter_acm: &InterRobotAcm,
    obstacles: &[(Isometry3<f64>, &ObstacleCollider)],
    attached: &[AttachedCollider],
) -> Vec<CollisionPair> {
    let world: Vec<Vec<Pose>> = robots
        .iter()
        .map(|r| r.link_poses.iter().map(to_parry_pose).collect())
        .collect();
    let mut pairs = Vec::new();
    for (r, robot) in robots.iter().enumerate() {
        let links = &robot.collider.links;
        for i in 0..links.len() {
            if links[i].is_empty() {
                continue;
            }
            for j in (i + 1)..links.len() {
                if links[j].is_empty() || robot.acm.allows(i, j) {
                    continue;
                }
                if parts_intersect(&world[r][i], &links[i], &world[r][j], &links[j]) {
                    pairs.push(CollisionPair {
                        a: ColliderId::Link { robot: r, link: i },
                        b: ColliderId::Link { robot: r, link: j },
                    });
                }
            }
        }
    }
    for r1 in 0..robots.len() {
        for r2 in (r1 + 1)..robots.len() {
            for (i, parts_i) in robots[r1].collider.links.iter().enumerate() {
                if parts_i.is_empty() {
                    continue;
                }
                for (j, parts_j) in robots[r2].collider.links.iter().enumerate() {
                    if parts_j.is_empty() || inter_acm.allows((r1, i), (r2, j)) {
                        continue;
                    }
                    if parts_intersect(&world[r1][i], parts_i, &world[r2][j], parts_j) {
                        pairs.push(CollisionPair {
                            a: ColliderId::Link { robot: r1, link: i },
                            b: ColliderId::Link { robot: r2, link: j },
                        });
                    }
                }
            }
        }
    }
    for (k, (obs_pose, obs)) in obstacles.iter().enumerate() {
        let op = to_parry_pose(obs_pose);
        for (r, robot) in robots.iter().enumerate() {
            for (i, parts) in robot.collider.links.iter().enumerate() {
                if parts.is_empty() {
                    continue;
                }
                if parts_intersect(&world[r][i], parts, &op, &obs.parts) {
                    pairs.push(CollisionPair {
                        a: ColliderId::Link { robot: r, link: i },
                        b: ColliderId::Obstacle(k),
                    });
                }
            }
        }
    }
    let att_world: Vec<Pose> = attached
        .iter()
        .map(|a| to_parry_pose(&(robots[a.robot].link_poses[a.link] * a.offset)))
        .collect();
    for (k, att) in attached.iter().enumerate() {
        for (r, robot) in robots.iter().enumerate() {
            for (i, parts) in robot.collider.links.iter().enumerate() {
                if parts.is_empty() || (r == att.robot && att.skip_links.contains(&i)) {
                    continue;
                }
                if parts_intersect(&world[r][i], parts, &att_world[k], &att.collider.parts) {
                    pairs.push(CollisionPair {
                        a: ColliderId::Link { robot: r, link: i },
                        b: ColliderId::Attached(k),
                    });
                }
            }
        }
        for (j, (obs_pose, obs)) in obstacles.iter().enumerate() {
            let op = to_parry_pose(obs_pose);
            if parts_intersect(&att_world[k], &att.collider.parts, &op, &obs.parts) {
                pairs.push(CollisionPair {
                    a: ColliderId::Attached(k),
                    b: ColliderId::Obstacle(j),
                });
            }
        }
        for k2 in (k + 1)..attached.len() {
            if parts_intersect(
                &att_world[k],
                &att.collider.parts,
                &att_world[k2],
                &attached[k2].collider.parts,
            ) {
                pairs.push(CollisionPair {
                    a: ColliderId::Attached(k),
                    b: ColliderId::Attached(k2),
                });
            }
        }
    }
    pairs
}

/// Minimum distance between the robot side (every robot's links plus
/// attached objects) and any obstacle (0 when colliding). `None` when there
/// is nothing to measure. Robot-robot clearance is not included.
pub fn min_robot_obstacle_distance(
    robots: &[RobotQuery<'_>],
    obstacles: &[(Isometry3<f64>, &ObstacleCollider)],
    attached: &[AttachedCollider],
) -> Option<f64> {
    let world: Vec<Vec<Pose>> = robots
        .iter()
        .map(|r| r.link_poses.iter().map(to_parry_pose).collect())
        .collect();
    let att_world: Vec<Pose> = attached
        .iter()
        .map(|a| to_parry_pose(&(robots[a.robot].link_poses[a.link] * a.offset)))
        .collect();
    let mut min: Option<f64> = None;
    for (obs_pose, obs) in obstacles {
        let op = to_parry_pose(obs_pose);
        for (r, robot) in robots.iter().enumerate() {
            for (i, parts) in robot.collider.links.iter().enumerate() {
                if parts.is_empty() {
                    continue;
                }
                if let Some(d) = parts_distance(&world[r][i], parts, &op, &obs.parts) {
                    min = Some(min.map_or(d, |m| m.min(d)));
                }
            }
        }
        for (k, att) in attached.iter().enumerate() {
            if let Some(d) = parts_distance(&att_world[k], &att.collider.parts, &op, &obs.parts) {
                min = Some(min.map_or(d, |m| m.min(d)));
            }
        }
    }
    min
}

#[cfg(test)]
mod tests {
    use super::*;
    use botrail_model::Geometry;
    use nalgebra::{Translation3, UnitQuaternion};

    /// Three stacked 0.2m cubes connected by prismatic joints along z; at
    /// q = 0 all three overlap at the origin.
    const STACK: &str = r#"
    <robot name="stack">
      <link name="a">
        <visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual>
      </link>
      <link name="b">
        <visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual>
      </link>
      <link name="c">
        <visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual>
      </link>
      <joint name="ab" type="prismatic">
        <parent link="a"/><child link="b"/>
        <axis xyz="0 0 1"/>
        <limit lower="0" upper="1" effort="1" velocity="1"/>
      </joint>
      <joint name="bc" type="prismatic">
        <parent link="b"/><child link="c"/>
        <axis xyz="0 0 1"/>
        <limit lower="0" upper="1" effort="1" velocity="1"/>
      </joint>
    </robot>"#;

    fn stack() -> (botrail_model::RobotModel, RobotCollider, Acm) {
        let model = botrail_model::RobotModel::from_urdf_str(STACK).unwrap();
        let (collider, warnings) = RobotCollider::from_model(&model);
        assert!(warnings.is_empty(), "{warnings:?}");
        let acm = Acm::adjacent(&model);
        (model, collider, acm)
    }

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity())
    }

    /// A one-robot query slice for the pre-multi-robot call shape.
    fn solo<'a>(
        collider: &'a RobotCollider,
        link_poses: &'a [Isometry3<f64>],
        acm: &'a Acm,
    ) -> [RobotQuery<'a>; 1] {
        [RobotQuery {
            collider,
            link_poses,
            acm,
        }]
    }

    fn link(robot: usize, link: usize) -> ColliderId {
        ColliderId::Link { robot, link }
    }

    #[test]
    fn self_collision_respects_acm() {
        let (model, collider, acm) = stack();
        // q = 0: a, b, c all coincide. a-b and b-c are adjacent (allowed);
        // only the non-adjacent a-c pair must be reported.
        let poses = botrail_kin::forward_kinematics(&model, &[0.0, 0.0]).unwrap();
        let pairs = check_scene(
            &solo(&collider, &poses, &acm),
            &InterRobotAcm::default(),
            &[],
            &[],
        );
        assert_eq!(
            pairs,
            vec![CollisionPair {
                a: link(0, 0),
                b: link(0, 2),
            }]
        );
        // Separate all three: no collisions.
        let poses = botrail_kin::forward_kinematics(&model, &[0.5, 0.5]).unwrap();
        assert!(check_scene(
            &solo(&collider, &poses, &acm),
            &InterRobotAcm::default(),
            &[],
            &[]
        )
        .is_empty());
    }

    #[test]
    fn robots_collide_with_each_other_unless_allowed() {
        let (model, collider, acm) = stack();
        // Two copies of the separated stack: robot 0 at the origin, robot 1
        // shifted so its link a (at z=0) overlaps robot 0's link b (z=0.5).
        let poses_a = botrail_kin::forward_kinematics(&model, &[0.5, 0.5]).unwrap();
        let shift = iso(0.0, 0.0, 0.5);
        let poses_b: Vec<Isometry3<f64>> = poses_a.iter().map(|p| shift * p).collect();
        let robots = [
            RobotQuery {
                collider: &collider,
                link_poses: &poses_a,
                acm: &acm,
            },
            RobotQuery {
                collider: &collider,
                link_poses: &poses_b,
                acm: &acm,
            },
        ];
        // Overlaps: (0,b)-(1,a) coincide at z=0.5, (0,c)-(1,b) at z=1.0.
        let pairs = check_scene(&robots, &InterRobotAcm::default(), &[], &[]);
        assert_eq!(
            pairs,
            vec![
                CollisionPair {
                    a: link(0, 1),
                    b: link(1, 0),
                },
                CollisionPair {
                    a: link(0, 2),
                    b: link(1, 1),
                },
            ]
        );
        // Allowing both cross pairs silences them; intra-robot checks stay.
        let mut inter = InterRobotAcm::default();
        inter.allow((0, 1), (1, 0));
        inter.allow((1, 1), (0, 2)); // reversed order must normalize
        assert!(check_scene(&robots, &inter, &[], &[]).is_empty());

        // Far apart: nothing regardless of the inter ACM.
        let far: Vec<Isometry3<f64>> = poses_a.iter().map(|p| iso(5.0, 0.0, 0.0) * p).collect();
        let robots = [
            RobotQuery {
                collider: &collider,
                link_poses: &poses_a,
                acm: &acm,
            },
            RobotQuery {
                collider: &collider,
                link_poses: &far,
                acm: &acm,
            },
        ];
        assert!(check_scene(&robots, &InterRobotAcm::default(), &[], &[]).is_empty());
    }

    #[test]
    fn obstacle_collision_and_distance() {
        let (model, collider, acm) = stack();
        let poses = botrail_kin::forward_kinematics(&model, &[0.5, 0.5]).unwrap();
        let ball = ObstacleCollider::from_geometry(&Geometry::Sphere { radius: 0.05 }).unwrap();

        // Ball 0.3m to the side of link a (cube half-extent 0.1): gap 0.15.
        let far = [(iso(0.3, 0.0, 0.0), &ball)];
        let robots = solo(&collider, &poses, &acm);
        assert!(check_scene(&robots, &InterRobotAcm::default(), &far, &[]).is_empty());
        let d = min_robot_obstacle_distance(&robots, &far, &[]).unwrap();
        assert!((d - 0.15).abs() < 1e-9, "d = {d}");

        // Ball overlapping link b (which sits at z = 0.5).
        let hit = [(iso(0.0, 0.0, 0.55), &ball)];
        let pairs = check_scene(&robots, &InterRobotAcm::default(), &hit, &[]);
        assert_eq!(
            pairs,
            vec![CollisionPair {
                a: link(0, 1),
                b: ColliderId::Obstacle(0),
            }]
        );
        assert_eq!(
            min_robot_obstacle_distance(&robots, &hit, &[]).unwrap(),
            0.0
        );
    }

    #[test]
    fn attached_object_collides_and_follows_link() {
        let (model, collider, acm) = stack();
        // Separated stack: a at 0, b at 0.5, c at 1.0 — no self collision.
        let poses = botrail_kin::forward_kinematics(&model, &[0.5, 0.5]).unwrap();
        let held = ObstacleCollider::from_geometry(&Geometry::Sphere { radius: 0.05 }).unwrap();

        // Sphere held 0.3m beside link c (link 2): clear of everything.
        let robots = solo(&collider, &poses, &acm);
        let att = AttachedCollider {
            robot: 0,
            link: 2,
            offset: iso(0.3, 0.0, 0.0),
            collider: &held,
            skip_links: &[2],
        };
        assert!(check_scene(&robots, &InterRobotAcm::default(), &[], &[att]).is_empty());

        // An obstacle overlapping the held sphere's *world* position
        // (0.3, 0, 1.0) — proves the offset composes with the link pose.
        let ball = ObstacleCollider::from_geometry(&Geometry::Sphere { radius: 0.05 }).unwrap();
        let env = [(iso(0.3, 0.0, 1.0), &ball)];
        let att = AttachedCollider {
            robot: 0,
            link: 2,
            offset: iso(0.3, 0.0, 0.0),
            collider: &held,
            skip_links: &[2],
        };
        let pairs = check_scene(&robots, &InterRobotAcm::default(), &env, &[att]);
        assert_eq!(
            pairs,
            vec![CollisionPair {
                a: ColliderId::Attached(0),
                b: ColliderId::Obstacle(0),
            }]
        );

        // Distance includes the attached object: env ball moved 0.2m along x
        // from the held sphere (surface gap 0.1), links all >= 0.2 away.
        let env = [(iso(0.5, 0.0, 1.0), &ball)];
        let att = AttachedCollider {
            robot: 0,
            link: 2,
            offset: iso(0.3, 0.0, 0.0),
            collider: &held,
            skip_links: &[2],
        };
        let d = min_robot_obstacle_distance(&robots, &env, &[att]).unwrap();
        assert!((d - 0.1).abs() < 1e-9, "d = {d}");
    }

    #[test]
    fn attached_object_hits_other_robots_despite_skip_links() {
        let (model, collider, acm) = stack();
        let poses_a = botrail_kin::forward_kinematics(&model, &[0.5, 0.5]).unwrap();
        // Robot 1 far to the side, except its link a sits exactly where
        // robot 0's held sphere rides (0.3, 0, 1.0).
        let shift = iso(0.3, 0.0, 1.0);
        let poses_b: Vec<Isometry3<f64>> = poses_a.iter().map(|p| shift * p).collect();
        let robots = [
            RobotQuery {
                collider: &collider,
                link_poses: &poses_a,
                acm: &acm,
            },
            RobotQuery {
                collider: &collider,
                link_poses: &poses_b,
                acm: &acm,
            },
        ];
        let held = ObstacleCollider::from_geometry(&Geometry::Sphere { radius: 0.05 }).unwrap();
        // skip_links names link 0 — but that only suppresses the *carrier's*
        // (robot 0's) link 0; robot 1's link 0 must still be reported.
        let att = AttachedCollider {
            robot: 0,
            link: 2,
            offset: iso(0.3, 0.0, 0.0),
            collider: &held,
            skip_links: &[0, 2],
        };
        let pairs: Vec<CollisionPair> =
            check_scene(&robots, &InterRobotAcm::default(), &[], &[att])
                .into_iter()
                .filter(|p| {
                    matches!(p.a, ColliderId::Attached(_)) || matches!(p.b, ColliderId::Attached(_))
                })
                .collect();
        assert_eq!(
            pairs,
            vec![CollisionPair {
                a: link(1, 0),
                b: ColliderId::Attached(0),
            }]
        );
    }

    #[test]
    fn attached_skip_links_suppress_carrier_contact() {
        let (model, collider, acm) = stack();
        let poses = botrail_kin::forward_kinematics(&model, &[0.5, 0.5]).unwrap();
        let held = ObstacleCollider::from_geometry(&Geometry::Sphere { radius: 0.05 }).unwrap();

        // Sphere at the center of its carrying link c: touching the carrier.
        let robots = solo(&collider, &poses, &acm);
        let overlapping = AttachedCollider {
            robot: 0,
            link: 2,
            offset: Isometry3::identity(),
            collider: &held,
            skip_links: &[2],
        };
        assert!(check_scene(&robots, &InterRobotAcm::default(), &[], &[overlapping]).is_empty());

        // Same but skip_links empty: the carrier contact must be reported.
        let reported = AttachedCollider {
            robot: 0,
            link: 2,
            offset: Isometry3::identity(),
            collider: &held,
            skip_links: &[],
        };
        let pairs = check_scene(&robots, &InterRobotAcm::default(), &[], &[reported]);
        assert_eq!(
            pairs,
            vec![CollisionPair {
                a: link(0, 2),
                b: ColliderId::Attached(0),
            }]
        );
    }

    #[test]
    fn cylinder_axis_is_urdf_z() {
        // A single cylinder link (r=0.1, l=1.0): URDF cylinders extend along
        // z. A ball at z=0.45 must collide; at x=0.3 it must not (this fails
        // if the parry y-axis convention leaks through unconverted).
        let urdf = r#"
        <robot name="cyl">
          <link name="only">
            <visual><geometry><cylinder radius="0.1" length="1.0"/></geometry></visual>
          </link>
        </robot>"#;
        let model = botrail_model::RobotModel::from_urdf_str(urdf).unwrap();
        let (collider, _) = RobotCollider::from_model(&model);
        let acm = Acm::adjacent(&model);
        let poses = vec![Isometry3::identity()];
        let ball = ObstacleCollider::from_geometry(&Geometry::Sphere { radius: 0.05 }).unwrap();
        let robots = solo(&collider, &poses, &acm);

        let along_axis = [(iso(0.0, 0.0, 0.45), &ball)];
        assert_eq!(
            check_scene(&robots, &InterRobotAcm::default(), &along_axis, &[]).len(),
            1
        );

        let off_side = [(iso(0.3, 0.0, 0.0), &ball)];
        assert!(check_scene(&robots, &InterRobotAcm::default(), &off_side, &[]).is_empty());

        // Beyond the cap (cylinder ends at z=0.5, ball spans 0.65..0.75).
        let beyond_cap = [(iso(0.0, 0.0, 0.7), &ball)];
        assert!(check_scene(&robots, &InterRobotAcm::default(), &beyond_cap, &[]).is_empty());
    }

    #[test]
    fn mesh_geometry_warns_and_is_skipped() {
        let urdf = r#"
        <robot name="m">
          <link name="l">
            <visual><geometry><mesh filename="nope.stl"/></geometry></visual>
          </link>
        </robot>"#;
        let model = botrail_model::RobotModel::from_urdf_str(urdf).unwrap();
        let (collider, warnings) = RobotCollider::from_model(&model);
        assert_eq!(warnings.len(), 1);
        assert!(!collider.link_has_geometry(0));
    }
}

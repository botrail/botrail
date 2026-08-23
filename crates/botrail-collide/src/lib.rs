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
use parry3d_f64::bounding_volume::{Aabb, BoundingVolume};
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
    /// Local AABB per link (`None` for geometry-less links), the broad
    /// phase's raw material.
    link_aabbs: Vec<Option<Aabb>>,
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
        let link_aabbs = links.iter().map(parts_local_aabb).collect();
        (RobotCollider { links, link_aabbs }, warnings)
    }

    pub fn link_has_geometry(&self, link: usize) -> bool {
        !self.links[link].is_empty()
    }
}

/// Collision shapes for one obstacle (world pose supplied per query).
#[derive(Clone)]
pub struct ObstacleCollider {
    parts: Parts,
    /// Local AABB over the parts, cached for the broad phase.
    local_aabb: Option<Aabb>,
}

impl ObstacleCollider {
    pub fn from_geometry(geometry: &botrail_model::Geometry) -> Result<Self, CollideError> {
        let (offset, shape) = convert::geometry_to_parry(geometry)?;
        let parts = vec![(offset, shape)];
        let local_aabb = parts_local_aabb(&parts);
        Ok(ObstacleCollider { parts, local_aabb })
    }

    /// The obstacle's *exact* surface rather than its collision proxy:
    /// primitives as themselves (already exact), meshes as the triangle
    /// mesh in the file instead of its convex decomposition. For ray
    /// probes that need the true hit distance and normal — a curved
    /// panel's — where the VHACD hulls would answer with their own facets.
    /// Not for collision checking: a triangle mesh has no interior.
    pub fn exact_surface(geometry: &botrail_model::Geometry) -> Result<Self, CollideError> {
        match geometry {
            botrail_model::Geometry::Mesh { path, scale } => {
                let data = mesh::load_mesh_data(path, scale)?;
                Ok(Self::from_shape(mesh::mesh_to_trimesh(&data)?))
            }
            other => Self::from_geometry(other),
        }
    }

    /// Wraps an already-built shape (e.g. [`mesh::mesh_to_compound`] on
    /// in-memory mesh data, where no file path exists to load from).
    pub fn from_shape(shape: SharedShape) -> Self {
        let parts = vec![(Pose::identity(), shape)];
        let local_aabb = parts_local_aabb(&parts);
        ObstacleCollider { parts, local_aabb }
    }

    /// Whether the *local-frame* point lies inside any solid part. The
    /// shape policy makes every part solid (primitives and VHACD hulls),
    /// so containment is exact for the collision geometry — what the
    /// stock-carving voxelizer initializes its grid from.
    pub fn contains_local_point(&self, point: &nalgebra::Point3<f64>) -> bool {
        let p = parry3d_f64::math::Vector::new(point.x, point.y, point.z);
        self.parts
            .iter()
            .any(|(offset, shape)| shape.contains_point(offset, p))
    }

    /// Distance from `origin` along `dir` to the first solid part hit,
    /// both in the collider's *local* frame, or `None` past `max_toi`.
    /// `dir` need not be normalized; the result is in units of `dir`.
    ///
    /// What spray coating needs to answer "does this patch see the gun":
    /// a shadow test against the target's own body. Rays start outside
    /// the shape, so the solid-interior convention of the other queries
    /// does not come into play.
    pub fn cast_local_ray(
        &self,
        origin: &nalgebra::Point3<f64>,
        dir: &nalgebra::Vector3<f64>,
        max_toi: f64,
    ) -> Option<f64> {
        self.cast_local_ray_with_normal(origin, dir, max_toi)
            .map(|(toi, _)| toi)
    }

    /// [`Self::cast_local_ray`] plus the outward surface normal at the
    /// hit, in the collider's local frame — what a standoff check needs to
    /// turn a hit into an incidence angle. Convex parts and hulls report a
    /// clean face normal; a mesh's is the triangle's, so a curved surface
    /// reads a little faceted at the patch scale.
    pub fn cast_local_ray_with_normal(
        &self,
        origin: &nalgebra::Point3<f64>,
        dir: &nalgebra::Vector3<f64>,
        max_toi: f64,
    ) -> Option<(f64, nalgebra::Vector3<f64>)> {
        let ray = parry3d_f64::query::Ray::new(
            parry3d_f64::math::Vector::new(origin.x, origin.y, origin.z),
            parry3d_f64::math::Vector::new(dir.x, dir.y, dir.z),
        );
        self.parts
            .iter()
            .filter_map(|(offset, shape)| {
                shape.cast_ray_and_get_normal(offset, &ray, max_toi, true)
            })
            .min_by(|a, b| {
                a.time_of_impact
                    .partial_cmp(&b.time_of_impact)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|hit| {
                (
                    hit.time_of_impact,
                    nalgebra::Vector3::new(hit.normal.x, hit.normal.y, hit.normal.z),
                )
            })
    }

    /// World-frame axis-aligned bounds at `pose`, as `(min, max)`.
    ///
    /// Cheap: the parts are already built, so this reads the shapes rather
    /// than reloading the mesh. `None` for a collider with no parts.
    pub fn aabb(&self, pose: &Isometry3<f64>) -> Option<([f64; 3], [f64; 3])> {
        use parry3d_f64::bounding_volume::BoundingVolume;
        let world = convert::to_parry_pose(pose);
        let mut merged: Option<parry3d_f64::bounding_volume::Aabb> = None;
        for (offset, shape) in &self.parts {
            let box_ = shape.compute_aabb(&(world * offset));
            merged = Some(match merged {
                Some(acc) => acc.merged(&box_),
                None => box_,
            });
        }
        merged.map(|a| {
            (
                [a.mins.x, a.mins.y, a.mins.z],
                [a.maxs.x, a.maxs.y, a.maxs.z],
            )
        })
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

/// Local-frame AABB enclosing every part; `None` when there are none.
/// Cached at build time — the broad phase transforms it per query instead
/// of re-measuring shapes.
fn parts_local_aabb(parts: &Parts) -> Option<Aabb> {
    let mut merged: Option<Aabb> = None;
    for (offset, shape) in parts {
        let aabb = shape.compute_aabb(offset);
        merged = Some(match merged {
            Some(m) => m.merged(&aabb),
            None => aabb,
        });
    }
    merged
}

/// Broad-phase gate: can these two (possibly absent) boxes touch at all?
/// Disjoint AABBs prove disjoint shapes, so a `false` here skips the exact
/// test without changing any result.
fn boxes_hit(a: &Option<Aabb>, b: &Option<Aabb>) -> bool {
    match (a, b) {
        (Some(a), Some(b)) => a.intersects(b),
        _ => false,
    }
}

/// Lower bound on the distance between shapes inside two AABBs.
fn boxes_gap(a: &Aabb, b: &Aabb) -> f64 {
    let mut sq = 0.0;
    for c in 0..3 {
        let gap = (b.mins[c] - a.maxs[c]).max(a.mins[c] - b.maxs[c]).max(0.0);
        sq += gap * gap;
    }
    sq.sqrt()
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
/// The broad-phase working set for one query: world poses and world AABBs
/// for every link, per-robot merged bounds, and the attached objects'
/// poses/bounds. Building it is O(links + attached); every section of the
/// scene check gates on it before running an exact test, which changes no
/// result — disjoint AABBs prove disjoint shapes — only the bill.
struct BroadPhase {
    world: Vec<Vec<Pose>>,
    link_aabbs: Vec<Vec<Option<Aabb>>>,
    robot_bounds: Vec<Option<Aabb>>,
    att_world: Vec<Pose>,
    att_aabbs: Vec<Option<Aabb>>,
}

impl BroadPhase {
    fn new(robots: &[RobotQuery<'_>], attached: &[AttachedCollider]) -> Self {
        let world: Vec<Vec<Pose>> = robots
            .iter()
            .map(|r| r.link_poses.iter().map(to_parry_pose).collect())
            .collect();
        let link_aabbs: Vec<Vec<Option<Aabb>>> = robots
            .iter()
            .enumerate()
            .map(|(r, robot)| {
                robot
                    .collider
                    .link_aabbs
                    .iter()
                    .enumerate()
                    .map(|(i, local)| local.map(|a| a.transform_by(&world[r][i])))
                    .collect()
            })
            .collect();
        let robot_bounds: Vec<Option<Aabb>> = link_aabbs
            .iter()
            .map(|links| {
                links.iter().flatten().fold(None, |acc: Option<Aabb>, a| {
                    Some(match acc {
                        Some(m) => m.merged(a),
                        None => *a,
                    })
                })
            })
            .collect();
        let att_world: Vec<Pose> = attached
            .iter()
            .map(|a| to_parry_pose(&(robots[a.robot].link_poses[a.link] * a.offset)))
            .collect();
        let att_aabbs: Vec<Option<Aabb>> = attached
            .iter()
            .zip(&att_world)
            .map(|(a, pose)| a.collider.local_aabb.map(|b| b.transform_by(pose)))
            .collect();
        BroadPhase {
            world,
            link_aabbs,
            robot_bounds,
            att_world,
            att_aabbs,
        }
    }
}

/// Allowed link-obstacle contact pairs — the third exemption mechanism
/// beside the intra-robot [`Acm`] and the [`InterRobotAcm`], for process
/// contact that is *supposed* to happen (a milling cutter in its stock).
/// Keyed by the obstacle index as passed to the query, so callers that
/// filter their obstacle list map names to filtered indices first.
#[derive(Debug, Default, Clone)]
pub struct ContactAllowance {
    pairs: std::collections::HashSet<(usize, usize, usize)>,
}

impl ContactAllowance {
    pub fn allow(&mut self, robot: usize, link: usize, obstacle: usize) {
        self.pairs.insert((robot, link, obstacle));
    }

    pub fn allows(&self, robot: usize, link: usize, obstacle: usize) -> bool {
        self.pairs.contains(&(robot, link, obstacle))
    }

    pub fn is_empty(&self) -> bool {
        self.pairs.is_empty()
    }
}

pub fn check_scene(
    robots: &[RobotQuery<'_>],
    inter_acm: &InterRobotAcm,
    obstacles: &[(Isometry3<f64>, &ObstacleCollider)],
    attached: &[AttachedCollider],
    allowance: &ContactAllowance,
) -> Vec<CollisionPair> {
    let bp = BroadPhase::new(robots, attached);
    let mut pairs = Vec::new();
    for (r, robot) in robots.iter().enumerate() {
        let links = &robot.collider.links;
        for i in 0..links.len() {
            if links[i].is_empty() {
                continue;
            }
            for j in (i + 1)..links.len() {
                if links[j].is_empty()
                    || robot.acm.allows(i, j)
                    || !boxes_hit(&bp.link_aabbs[r][i], &bp.link_aabbs[r][j])
                {
                    continue;
                }
                if parts_intersect(&bp.world[r][i], &links[i], &bp.world[r][j], &links[j]) {
                    pairs.push(CollisionPair {
                        a: ColliderId::Link { robot: r, link: i },
                        b: ColliderId::Link { robot: r, link: j },
                    });
                }
            }
        }
    }
    cross_robot_pairs(robots, inter_acm, &bp, &mut pairs);
    obstacle_pairs(robots, obstacles, attached, allowance, &bp, &mut pairs);
    attached_robot_and_mutual_pairs(robots, attached, &bp, false, &mut pairs);
    pairs
}

/// Only the robot-vs-obstacle section of [`check_scene`]: every link and
/// every attached object against the given obstacles, with no self or
/// cross-robot pairs. What a robot riding a travelling vehicle is checked
/// with — the aisle check for the part of the machine that is a robot —
/// where pricing its self-collisions every tick would be waste and its
/// fellow robots are the scan tick's own question.
pub fn check_against_obstacles(
    robots: &[RobotQuery<'_>],
    obstacles: &[(Isometry3<f64>, &ObstacleCollider)],
    attached: &[AttachedCollider],
    allowance: &ContactAllowance,
) -> Vec<CollisionPair> {
    let bp = BroadPhase::new(robots, attached);
    let mut pairs = Vec::new();
    obstacle_pairs(robots, obstacles, attached, allowance, &bp, &mut pairs);
    pairs
}

/// The robot-vs-obstacle section, shared by [`check_scene`] and
/// [`check_against_obstacles`].
fn obstacle_pairs(
    robots: &[RobotQuery<'_>],
    obstacles: &[(Isometry3<f64>, &ObstacleCollider)],
    attached: &[AttachedCollider],
    allowance: &ContactAllowance,
    bp: &BroadPhase,
    pairs: &mut Vec<CollisionPair>,
) {
    for (k, (obs_pose, obs)) in obstacles.iter().enumerate() {
        let op = to_parry_pose(obs_pose);
        let obs_aabb = obs.local_aabb.map(|a| a.transform_by(&op));
        for (r, robot) in robots.iter().enumerate() {
            if !boxes_hit(&bp.robot_bounds[r], &obs_aabb) {
                continue;
            }
            for (i, parts) in robot.collider.links.iter().enumerate() {
                if parts.is_empty()
                    || allowance.allows(r, i, k)
                    || !boxes_hit(&bp.link_aabbs[r][i], &obs_aabb)
                {
                    continue;
                }
                if parts_intersect(&bp.world[r][i], parts, &op, &obs.parts) {
                    pairs.push(CollisionPair {
                        a: ColliderId::Link { robot: r, link: i },
                        b: ColliderId::Obstacle(k),
                    });
                }
            }
        }
        for (k2, att) in attached.iter().enumerate() {
            if !boxes_hit(&bp.att_aabbs[k2], &obs_aabb) {
                continue;
            }
            if parts_intersect(&bp.att_world[k2], &att.collider.parts, &op, &obs.parts) {
                pairs.push(CollisionPair {
                    a: ColliderId::Attached(k2),
                    b: ColliderId::Obstacle(k),
                });
            }
        }
    }
}

/// The robot-vs-robot section, shared by [`check_scene`] and
/// [`check_cross_robot`] so the two can never disagree on it.
fn cross_robot_pairs(
    robots: &[RobotQuery<'_>],
    inter_acm: &InterRobotAcm,
    bp: &BroadPhase,
    pairs: &mut Vec<CollisionPair>,
) {
    for r1 in 0..robots.len() {
        for r2 in (r1 + 1)..robots.len() {
            if !boxes_hit(&bp.robot_bounds[r1], &bp.robot_bounds[r2]) {
                continue;
            }
            for (i, parts_i) in robots[r1].collider.links.iter().enumerate() {
                if parts_i.is_empty() {
                    continue;
                }
                for (j, parts_j) in robots[r2].collider.links.iter().enumerate() {
                    if parts_j.is_empty()
                        || inter_acm.allows((r1, i), (r2, j))
                        || !boxes_hit(&bp.link_aabbs[r1][i], &bp.link_aabbs[r2][j])
                    {
                        continue;
                    }
                    if parts_intersect(&bp.world[r1][i], parts_i, &bp.world[r2][j], parts_j) {
                        pairs.push(CollisionPair {
                            a: ColliderId::Link { robot: r1, link: i },
                            b: ColliderId::Link { robot: r2, link: j },
                        });
                    }
                }
            }
        }
    }
}

/// Attached-object pairs: vs robot links (all robots, or — `cross_only` —
/// just the ones that do not carry the object) and vs other attached
/// objects (all pairs, or just cross-carrier pairs).
fn attached_robot_and_mutual_pairs(
    robots: &[RobotQuery<'_>],
    attached: &[AttachedCollider],
    bp: &BroadPhase,
    cross_only: bool,
    pairs: &mut Vec<CollisionPair>,
) {
    for (k, att) in attached.iter().enumerate() {
        for (r, robot) in robots.iter().enumerate() {
            if cross_only && r == att.robot {
                continue;
            }
            if !boxes_hit(&bp.robot_bounds[r], &bp.att_aabbs[k]) {
                continue;
            }
            for (i, parts) in robot.collider.links.iter().enumerate() {
                if parts.is_empty()
                    || (r == att.robot && att.skip_links.contains(&i))
                    || !boxes_hit(&bp.link_aabbs[r][i], &bp.att_aabbs[k])
                {
                    continue;
                }
                if parts_intersect(
                    &bp.world[r][i],
                    parts,
                    &bp.att_world[k],
                    &att.collider.parts,
                ) {
                    pairs.push(CollisionPair {
                        a: ColliderId::Link { robot: r, link: i },
                        b: ColliderId::Attached(k),
                    });
                }
            }
        }
        for (k2, other) in attached.iter().enumerate().skip(k + 1) {
            if cross_only && other.robot == att.robot {
                continue;
            }
            if !boxes_hit(&bp.att_aabbs[k], &bp.att_aabbs[k2]) {
                continue;
            }
            if parts_intersect(
                &bp.att_world[k],
                &att.collider.parts,
                &bp.att_world[k2],
                &other.collider.parts,
            ) {
                pairs.push(CollisionPair {
                    a: ColliderId::Attached(k),
                    b: ColliderId::Attached(k2),
                });
            }
        }
    }
}

/// Only what spans two robots: cross-robot link pairs, attached objects
/// vs *other* robots' links, and attached objects on different carriers.
///
/// This is the scan-tick verification's dedicated path. The full
/// [`check_scene`] computes self-collisions and every obstacle contact too
/// — which a per-tick check that only reports cross-robot contact used to
/// compute and throw away, and which dominates the bill the moment the
/// scene holds a line's worth of bodywork.
pub fn check_cross_robot(
    robots: &[RobotQuery<'_>],
    inter_acm: &InterRobotAcm,
    attached: &[AttachedCollider],
) -> Vec<CollisionPair> {
    let bp = BroadPhase::new(robots, attached);
    let mut pairs = Vec::new();
    cross_robot_pairs(robots, inter_acm, &bp, &mut pairs);
    attached_robot_and_mutual_pairs(robots, attached, &bp, true, &mut pairs);
    pairs
}

/// Minimum distance between the robot side (every robot's links plus
/// attached objects) and any obstacle (0 when colliding). `None` when there
/// is nothing to measure. Robot-robot clearance is not included.
pub fn min_robot_obstacle_distance(
    robots: &[RobotQuery<'_>],
    obstacles: &[(Isometry3<f64>, &ObstacleCollider)],
    attached: &[AttachedCollider],
    allowance: &ContactAllowance,
) -> Option<f64> {
    let bp = BroadPhase::new(robots, attached);
    // Broad phase for a *distance* query: every candidate pair gets an
    // AABB gap — a sound lower bound on its exact distance — and pairs run
    // nearest-bound first. As soon as the next bound is at or beyond the
    // best exact distance so far, everything after it is too (the list is
    // sorted), and the loop stops. The minimum cannot change: a skipped
    // pair's distance is >= its bound >= the answer already in hand.
    enum Side {
        Link(usize, usize),
        Attached(usize),
    }
    let obs_world: Vec<(Pose, Option<Aabb>)> = obstacles
        .iter()
        .map(|(pose, obs)| {
            let op = to_parry_pose(pose);
            let aabb = obs.local_aabb.map(|a| a.transform_by(&op));
            (op, aabb)
        })
        .collect();
    let mut candidates: Vec<(f64, Side, usize)> = Vec::new();
    for (k, (_, obs_aabb)) in obs_world.iter().enumerate() {
        let Some(obs_aabb) = obs_aabb else { continue };
        for (r, robot) in robots.iter().enumerate() {
            for (i, parts) in robot.collider.links.iter().enumerate() {
                if parts.is_empty() || allowance.allows(r, i, k) {
                    continue;
                }
                let Some(link_aabb) = &bp.link_aabbs[r][i] else {
                    continue;
                };
                let bound = boxes_gap(link_aabb, obs_aabb);
                if bound <= DISTANCE_PREDICTION {
                    candidates.push((bound, Side::Link(r, i), k));
                }
            }
        }
        for (a, att_aabb) in bp.att_aabbs.iter().enumerate() {
            let Some(att_aabb) = att_aabb else { continue };
            let bound = boxes_gap(att_aabb, obs_aabb);
            if bound <= DISTANCE_PREDICTION {
                candidates.push((bound, Side::Attached(a), k));
            }
        }
    }
    candidates.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut min: Option<f64> = None;
    for (bound, side, k) in candidates {
        if let Some(best) = min {
            if bound >= best {
                break;
            }
        }
        let (op, _) = &obs_world[k];
        let obs = obstacles[k].1;
        let d = match side {
            Side::Link(r, i) => parts_distance(
                &bp.world[r][i],
                &robots[r].collider.links[i],
                op,
                &obs.parts,
            ),
            Side::Attached(a) => parts_distance(
                &bp.att_world[a],
                &attached[a].collider.parts,
                op,
                &obs.parts,
            ),
        };
        if let Some(d) = d {
            min = Some(min.map_or(d, |m| m.min(d)));
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
            &ContactAllowance::default(),
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
            &[],
            &ContactAllowance::default()
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
        let pairs = check_scene(
            &robots,
            &InterRobotAcm::default(),
            &[],
            &[],
            &ContactAllowance::default(),
        );
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
        assert!(check_scene(&robots, &inter, &[], &[], &ContactAllowance::default()).is_empty());

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
        assert!(check_scene(
            &robots,
            &InterRobotAcm::default(),
            &[],
            &[],
            &ContactAllowance::default()
        )
        .is_empty());
    }

    #[test]
    fn obstacle_collision_and_distance() {
        let (model, collider, acm) = stack();
        let poses = botrail_kin::forward_kinematics(&model, &[0.5, 0.5]).unwrap();
        let ball = ObstacleCollider::from_geometry(&Geometry::Sphere { radius: 0.05 }).unwrap();

        // Ball 0.3m to the side of link a (cube half-extent 0.1): gap 0.15.
        let far = [(iso(0.3, 0.0, 0.0), &ball)];
        let robots = solo(&collider, &poses, &acm);
        assert!(check_scene(
            &robots,
            &InterRobotAcm::default(),
            &far,
            &[],
            &ContactAllowance::default()
        )
        .is_empty());
        let d =
            min_robot_obstacle_distance(&robots, &far, &[], &ContactAllowance::default()).unwrap();
        assert!((d - 0.15).abs() < 1e-9, "d = {d}");

        // Ball overlapping link b (which sits at z = 0.5).
        let hit = [(iso(0.0, 0.0, 0.55), &ball)];
        let pairs = check_scene(
            &robots,
            &InterRobotAcm::default(),
            &hit,
            &[],
            &ContactAllowance::default(),
        );
        assert_eq!(
            pairs,
            vec![CollisionPair {
                a: link(0, 1),
                b: ColliderId::Obstacle(0),
            }]
        );
        assert_eq!(
            min_robot_obstacle_distance(&robots, &hit, &[], &ContactAllowance::default()).unwrap(),
            0.0
        );
    }

    #[test]
    fn contact_allowance_excuses_exactly_its_pair() {
        let (model, collider, acm) = stack();
        let poses = botrail_kin::forward_kinematics(&model, &[0.5, 0.5]).unwrap();
        let ball = ObstacleCollider::from_geometry(&Geometry::Sphere { radius: 0.05 }).unwrap();
        // Two obstacles overlapping link b (index 1): one is process
        // contact (allowed), the other is a genuine collision.
        let obs = [(iso(0.0, 0.0, 0.55), &ball), (iso(0.0, 0.05, 0.5), &ball)];
        let robots = solo(&collider, &poses, &acm);
        let mut allowance = ContactAllowance::default();
        allowance.allow(0, 1, 0);
        let pairs = check_scene(&robots, &InterRobotAcm::default(), &obs, &[], &allowance);
        assert_eq!(
            pairs,
            vec![CollisionPair {
                a: link(0, 1),
                b: ColliderId::Obstacle(1),
            }]
        );
        // The distance query skips the allowed pair the same way: with the
        // second ball moved clear, the touching-but-allowed pair no longer
        // zeroes the metric.
        let clear = [(iso(0.0, 0.0, 0.55), &ball), (iso(0.4, 0.0, 0.5), &ball)];
        let d = min_robot_obstacle_distance(&robots, &clear, &[], &allowance).unwrap();
        assert!(d > 0.1, "allowed contact still zeroed the distance: {d}");
        // Without the allowance both report.
        let strict = check_scene(
            &robots,
            &InterRobotAcm::default(),
            &obs,
            &[],
            &ContactAllowance::default(),
        );
        assert_eq!(strict.len(), 2);
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
        assert!(check_scene(
            &robots,
            &InterRobotAcm::default(),
            &[],
            &[att],
            &ContactAllowance::default()
        )
        .is_empty());

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
        let pairs = check_scene(
            &robots,
            &InterRobotAcm::default(),
            &env,
            &[att],
            &ContactAllowance::default(),
        );
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
        let d = min_robot_obstacle_distance(&robots, &env, &[att], &ContactAllowance::default())
            .unwrap();
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
        let pairs: Vec<CollisionPair> = check_scene(
            &robots,
            &InterRobotAcm::default(),
            &[],
            &[att],
            &ContactAllowance::default(),
        )
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
        assert!(check_scene(
            &robots,
            &InterRobotAcm::default(),
            &[],
            &[overlapping],
            &ContactAllowance::default()
        )
        .is_empty());

        // Same but skip_links empty: the carrier contact must be reported.
        let reported = AttachedCollider {
            robot: 0,
            link: 2,
            offset: Isometry3::identity(),
            collider: &held,
            skip_links: &[],
        };
        let pairs = check_scene(
            &robots,
            &InterRobotAcm::default(),
            &[],
            &[reported],
            &ContactAllowance::default(),
        );
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
            check_scene(
                &robots,
                &InterRobotAcm::default(),
                &along_axis,
                &[],
                &ContactAllowance::default()
            )
            .len(),
            1
        );

        let off_side = [(iso(0.3, 0.0, 0.0), &ball)];
        assert!(check_scene(
            &robots,
            &InterRobotAcm::default(),
            &off_side,
            &[],
            &ContactAllowance::default()
        )
        .is_empty());

        // Beyond the cap (cylinder ends at z=0.5, ball spans 0.65..0.75).
        let beyond_cap = [(iso(0.0, 0.0, 0.7), &ball)];
        assert!(check_scene(
            &robots,
            &InterRobotAcm::default(),
            &beyond_cap,
            &[],
            &ContactAllowance::default()
        )
        .is_empty());
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

#[cfg(test)]
mod broadphase_tests {
    use super::*;
    use botrail_model::RobotModel;
    use nalgebra::{Translation3, UnitQuaternion, Vector3};

    /// 2-link arm: revolute Z at z = 0.3, 0.2 m cubes on both links.
    fn arm() -> RobotModel {
        RobotModel::from_urdf_str(
            r#"
        <robot name="r">
          <link name="a">
            <visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual>
          </link>
          <link name="b">
            <visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual>
          </link>
          <joint name="j" type="revolute">
            <parent link="a"/><child link="b"/>
            <origin xyz="0.3 0 0"/>
            <axis xyz="0 0 1"/>
            <limit lower="-3" upper="3" effort="1" velocity="1"/>
          </joint>
        </robot>"#,
        )
        .unwrap()
    }

    fn at(x: f64, y: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, 0.0), UnitQuaternion::identity())
    }

    /// The cross-robot fast path returns exactly the cross-robot subset of
    /// the full scene check — same pairs, same order — on a scene that has
    /// every contact kind at once: self-adjacent links, two robots
    /// touching, an obstacle inside a robot, and carried objects touching
    /// the other robot and each other.
    #[test]
    fn cross_path_equals_filtered_full_check() {
        let model = arm();
        let (collider, _) = RobotCollider::from_model(&model);
        let acm = Acm::default();
        let fk =
            |base: &Isometry3<f64>| -> Vec<Isometry3<f64>> { vec![*base, *base * at(0.3, 0.0)] };
        // Robot 1 close enough that its base link touches robot 0's tip.
        let base0 = at(0.0, 0.0);
        let base1 = at(0.45, 0.0);
        let poses0 = fk(&base0);
        let poses1 = fk(&base1);
        let robots = vec![
            RobotQuery {
                collider: &collider,
                link_poses: &poses0,
                acm: &acm,
            },
            RobotQuery {
                collider: &collider,
                link_poses: &poses1,
                acm: &acm,
            },
        ];
        let inter = InterRobotAcm::default();
        // An obstacle embedded in robot 0's base and one far away.
        let near = ObstacleCollider::from_geometry(&botrail_model::Geometry::Box {
            size: Vector3::new(0.1, 0.1, 0.1),
        })
        .unwrap();
        let far = ObstacleCollider::from_geometry(&botrail_model::Geometry::Box {
            size: Vector3::new(0.1, 0.1, 0.1),
        })
        .unwrap();
        let obstacles = vec![(at(0.0, 0.05), &near), (at(50.0, 50.0), &far)];
        // Robot 0 carries a box that overlaps robot 1's base link; robot 1
        // carries one that overlaps it back.
        let carried0 = ObstacleCollider::from_geometry(&botrail_model::Geometry::Box {
            size: Vector3::new(0.2, 0.2, 0.2),
        })
        .unwrap();
        let carried1 = ObstacleCollider::from_geometry(&botrail_model::Geometry::Box {
            size: Vector3::new(0.2, 0.2, 0.2),
        })
        .unwrap();
        let offset = at(0.15, 0.0);
        let attached = vec![
            AttachedCollider {
                robot: 0,
                link: 1,
                offset,
                collider: &carried0,
                skip_links: &[1],
            },
            AttachedCollider {
                robot: 1,
                link: 0,
                offset: at(-0.1, 0.0),
                collider: &carried1,
                skip_links: &[0],
            },
        ];

        let full = check_scene(
            &robots,
            &inter,
            &obstacles,
            &attached,
            &ContactAllowance::default(),
        );
        let cross = check_cross_robot(&robots, &inter, &attached);

        let robot_of = |id: &ColliderId| -> Option<usize> {
            match id {
                ColliderId::Link { robot, .. } => Some(*robot),
                ColliderId::Attached(k) => Some(attached[*k].robot),
                ColliderId::Obstacle(_) => None,
            }
        };
        let expected: Vec<CollisionPair> = full
            .iter()
            .filter(|p| match (robot_of(&p.a), robot_of(&p.b)) {
                (Some(a), Some(b)) => a != b,
                _ => false,
            })
            .copied()
            .collect();
        assert!(
            !expected.is_empty(),
            "the fixture is supposed to produce cross-robot contact"
        );
        assert!(
            full.iter().any(|p| matches!(
                (p.a, p.b),
                (ColliderId::Link { .. }, ColliderId::Obstacle(_))
            )),
            "the fixture is supposed to produce robot-obstacle contact too"
        );
        assert_eq!(cross, expected);
    }

    /// The far obstacle proves the broad phase prunes without changing the
    /// distance: the minimum is set by the near box, exactly.
    #[test]
    fn distance_survives_the_broad_phase() {
        let model = arm();
        let (collider, _) = RobotCollider::from_model(&model);
        let acm = Acm::default();
        let poses = vec![at(0.0, 0.0), at(0.3, 0.0)];
        let robots = vec![RobotQuery {
            collider: &collider,
            link_poses: &poses,
            acm: &acm,
        }];
        let near = ObstacleCollider::from_geometry(&botrail_model::Geometry::Box {
            size: Vector3::new(0.1, 0.1, 0.1),
        })
        .unwrap();
        let far = near.clone();
        let obstacles = vec![(at(0.0, 0.75), &near), (at(30.0, 0.0), &far)];
        let d = min_robot_obstacle_distance(&robots, &obstacles, &[], &ContactAllowance::default())
            .unwrap();
        // Gap between the 0.2 cube face (y = 0.1) and the 0.1 cube face
        // (y = 0.7).
        assert!((d - 0.6).abs() < 1e-9, "{d}");
    }
}

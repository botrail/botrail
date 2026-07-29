//! USD articulation import: UsdPhysics joints + rigid bodies -> RobotModel.
//!
//! # Frame conversion
//!
//! A USD physics joint gives the joint frame relative to *both* bodies
//! (`localPose0` in body0, `localPose1` in body1); botrail follows URDF,
//! where the child link frame *is* the joint frame. Each body's model frame
//! is therefore the joint frame of its inbound joint, and:
//!
//! - model joint origin  = K_parent⁻¹ ∘ localPose0   (K = that body's own
//!   inbound `localPose1`, identity at the root)
//! - geometry under a body is re-expressed by K⁻¹
//! - the motion axis stays the USD joint-frame axis
//!
//! so `parent * origin * R(axis, q)` reproduces the physics chain
//! `parent * localPose0 * R * localPose1⁻¹` exactly.
//!
//! # Units and axes
//!
//! Revolute limits arrive in degrees (USD spec) and are converted to
//! radians; prismatic limits and all translations scale by
//! `metersPerUnit`. On Y-up stages every transform is conjugated by the
//! up-axis fix (and axes/vertices rotated), yielding a Z-up-modeled robot.
//! PhysX's `physxJoint:maxJointVelocity` (deg/s or units/s) feeds the
//! velocity limit when authored.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use botrail_mesh::MeshData;
use botrail_model::{
    Geometry, Joint, JointLimits, JointType, Link, RobotModel, RobotSource, Shape,
};
use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};
use openusd::schemas::geom::{self, Imageable, PointBased, Purpose, Visibility, Xformable};
use openusd::schemas::physics::{self, JointBase};
use openusd::usd::{Prim, SchemaBase, Stage, TimeCode};
use openusd::{gf, sdf};

use crate::{
    decompose_matrix, default_mesh_cache_dir, int_vec, triangulate, write_stl_cached, y_up_to_z_up,
    AnyPrim, SearchPathResolver, UsdImportError,
};

/// Untyped joint view granting the shared [`JointBase`] accessors
/// (bodies, localPose0/1) for any concrete joint type.
struct AnyJoint(Prim);

impl SchemaBase for AnyJoint {
    const KIND: openusd::usd::SchemaKind = openusd::usd::SchemaKind::ConcreteTyped;
    fn prim(&self) -> &Prim {
        &self.0
    }
}
impl JointBase for AnyJoint {}

#[derive(Debug, Clone, Default)]
pub struct RobotImportOptions {
    /// See [`crate::ImportOptions::search_paths`].
    pub search_paths: Vec<PathBuf>,
    pub mesh_cache_dir: Option<PathBuf>,
    /// Prim path of the articulation to import; defaults to the first prim
    /// carrying `PhysicsArticulationRootAPI`.
    pub articulation_root: Option<String>,
}

pub struct ImportedRobot {
    pub model: RobotModel,
    /// Stage-world pose of the robot's root body (normalized to meters /
    /// Z-up, frame semantics) — a hint for initial base placement.
    pub root_pose: Isometry3<f64>,
    pub warnings: Vec<String>,
}

/// One prim recorded during the collection walk.
struct PrimInfo {
    prim: Prim,
    path: String,
    type_name: String,
    world: gf::Matrix4d,
    is_body: bool,
    is_articulation_root: bool,
    visible: bool,
    default_purpose: bool,
    has_collision_api: bool,
}

struct JointRecord {
    name: String,
    joint_type: JointType,
    body0: Option<String>,
    body1: String,
    local_pose0: Isometry3<f64>,
    local_pose1: Isometry3<f64>,
    axis: Vector3<f64>,
    /// Raw (USD units/degrees) limits.
    lower: Option<f64>,
    upper: Option<f64>,
    max_velocity: Option<f64>,
    effort: f64,
}

pub fn import_robot(
    path: &Path,
    options: &RobotImportOptions,
) -> Result<ImportedRobot, UsdImportError> {
    let mut search_paths = options.search_paths.clone();
    if let Some(dir) = path.parent() {
        search_paths.push(dir.to_path_buf());
    }
    let stage = Stage::builder()
        .resolver(SearchPathResolver::new(search_paths))
        .open(&path.display().to_string())
        .map_err(|e| UsdImportError::Open {
            path: path.display().to_string(),
            message: e.to_string(),
        })?;

    // Stage normalization (USD defaults: centimeters, Y-up).
    let mut mpu = 0.01;
    let mut up_fix = y_up_to_z_up();
    {
        let layer = stage.root_layer();
        if let Some(root) = layer.pseudo_root() {
            match root.field("metersPerUnit") {
                Ok(Some(sdf::Value::Double(d))) => mpu = d,
                Ok(Some(sdf::Value::Float(f))) => mpu = f as f64,
                _ => {}
            }
            if let Ok(Some(sdf::Value::Token(t))) = root.field("upAxis") {
                if t.as_str() == "Z" {
                    up_fix = UnitQuaternion::identity();
                }
            }
        }
    }

    let mut prims = Vec::new();
    collect(
        stage.prim(sdf::Path::abs_root()),
        gf::Matrix4d::default(),
        &mut prims,
    )
    .map_err(|e| UsdImportError::Traverse(e.to_string()))?;

    let builder = RobotBuilder {
        stage: &stage,
        source_path: path.to_path_buf(),
        mpu,
        up_fix,
        mesh_cache_dir: options.mesh_cache_dir.clone(),
        warnings: Vec::new(),
    };
    builder.build(prims, options.articulation_root.as_deref())
}

fn collect(prim: Prim, parent_world: gf::Matrix4d, out: &mut Vec<PrimInfo>) -> anyhow::Result<()> {
    let view = AnyPrim(prim);
    let world = view
        .local_to_parent_transform(TimeCode::EARLIEST)
        .map(|local| local * parent_world)
        .unwrap_or(parent_world);
    let path = view.prim().path().to_string();
    let type_name = view
        .prim()
        .type_name()?
        .map(|t| t.to_string())
        .unwrap_or_default();
    let is_body = view.prim().has_api_schema("PhysicsRigidBodyAPI").unwrap_or(false);
    let is_articulation_root = view
        .prim()
        .has_api_schema("PhysicsArticulationRootAPI")
        .unwrap_or(false);
    let has_collision_api = view.prim().has_api_schema("PhysicsCollisionAPI").unwrap_or(false);
    let visible =
        view.compute_visibility().unwrap_or(Visibility::Inherited) != Visibility::Invisible;
    let default_purpose = view.compute_purpose().unwrap_or_default() == Purpose::Default;
    let children = view.prim().children()?;
    out.push(PrimInfo {
        prim: view.0,
        path,
        type_name,
        world,
        is_body,
        is_articulation_root,
        visible,
        default_purpose,
        has_collision_api,
    });
    for child in children {
        collect(child, world, out)?;
    }
    Ok(())
}

struct RobotBuilder<'a> {
    stage: &'a Stage,
    source_path: PathBuf,
    mpu: f64,
    up_fix: UnitQuaternion<f64>,
    mesh_cache_dir: Option<PathBuf>,
    warnings: Vec<String>,
}

impl RobotBuilder<'_> {
    fn build(
        mut self,
        prims: Vec<PrimInfo>,
        articulation_root: Option<&str>,
    ) -> Result<ImportedRobot, UsdImportError> {
        let art = |msg: String| UsdImportError::Articulation(msg);

        let root_path = match articulation_root {
            Some(path) => prims
                .iter()
                .find(|p| p.path == path)
                .ok_or_else(|| art(format!("articulation root `{path}` not found")))?
                .path
                .clone(),
            None => prims
                .iter()
                .find(|p| p.is_articulation_root)
                .ok_or_else(|| {
                    art("no prim with PhysicsArticulationRootAPI found (pass articulation_root)"
                        .to_string())
                })?
                .path
                .clone(),
        };
        let in_subtree =
            |p: &str| p == root_path || p.starts_with(&format!("{root_path}/")) || root_path == "/";

        // ---- joints --------------------------------------------------
        let mut joints = Vec::new();
        for info in prims.iter().filter(|p| in_subtree(&p.path)) {
            let joint_type = match info.type_name.as_str() {
                "PhysicsRevoluteJoint" => JointType::Revolute,
                "PhysicsPrismaticJoint" => JointType::Prismatic,
                "PhysicsFixedJoint" => JointType::Fixed,
                "PhysicsJoint" | "PhysicsSphericalJoint" | "PhysicsDistanceJoint" => {
                    self.warnings.push(format!(
                        "{}: unsupported joint type `{}`; skipped",
                        info.path, info.type_name
                    ));
                    continue;
                }
                _ => continue,
            };
            match self.read_joint(info, joint_type) {
                Ok(Some(joint)) => joints.push(joint),
                Ok(None) => {}
                Err(e) => self.warnings.push(format!("{}: {e}", info.path)),
            }
        }
        if joints.is_empty() {
            return Err(art(format!("no physics joints under `{root_path}`")));
        }

        // ---- bodies --------------------------------------------------
        // Discovery order (stable prim order), bodies referenced by joints
        // plus any rigid body in the subtree.
        let mut body_paths: Vec<String> = Vec::new();
        let mut seen = HashMap::new();
        let mut add_body = |path: &str, body_paths: &mut Vec<String>| {
            if !seen.contains_key(path) {
                seen.insert(path.to_string(), body_paths.len());
                body_paths.push(path.to_string());
            }
        };
        for info in prims.iter().filter(|p| p.is_body && in_subtree(&p.path)) {
            add_body(&info.path, &mut body_paths);
        }
        for joint in &joints {
            if let Some(b0) = &joint.body0 {
                add_body(b0, &mut body_paths);
            }
            add_body(&joint.body1, &mut body_paths);
        }
        let body_index: HashMap<String, usize> = seen;

        // Frame correction per body: its inbound joint's localPose1
        // (identity at the root). World-anchored joints (no body0) only fix
        // the base; they produce no model joint.
        let mut corrections = vec![Isometry3::identity(); body_paths.len()];
        for joint in &joints {
            corrections[body_index[&joint.body1]] = joint.local_pose1;
        }

        // ---- links ---------------------------------------------------
        let prim_by_path: HashMap<&str, &PrimInfo> =
            prims.iter().map(|p| (p.path.as_str(), p)).collect();
        let mut links = Vec::with_capacity(body_paths.len());
        for (bi, body_path) in body_paths.iter().enumerate() {
            let Some(body) = prim_by_path.get(body_path.as_str()) else {
                return Err(art(format!(
                    "joint references body `{body_path}` which is not in the stage"
                )));
            };
            links.push(self.build_link(body, &prims, &body_paths, &corrections[bi]));
        }

        // ---- model joints -------------------------------------------
        let mut model_joints = Vec::new();
        for joint in &joints {
            let Some(body0) = &joint.body0 else {
                // World anchor: fixes the base in the stage; the model root
                // simply is that body.
                continue;
            };
            let parent = body_index[body0.as_str()];
            let child = body_index[joint.body1.as_str()];
            let correction = corrections[parent];
            let origin = self.conjugate(&(correction.inverse() * joint.local_pose0));
            let axis = self.up_fix * joint.axis;
            let limits = self.joint_limits(joint);
            model_joints.push(Joint {
                name: joint.name.clone(),
                joint_type: joint.joint_type,
                origin,
                axis: Unit::try_new(axis, 1e-9).unwrap_or_else(|| Unit::new_unchecked(Vector3::z())),
                limits,
                parent_link: parent,
                child_link: child,
                q_index: None,
            });
        }

        let root_prim = prim_by_path[root_path.as_str()];
        let name = root_prim
            .path
            .rsplit('/')
            .next()
            .unwrap_or("robot")
            .to_string();
        let model = RobotModel::from_parts(
            name,
            links,
            model_joints,
            RobotSource::Usd {
                path: self.source_path.clone(),
                articulation_root: root_path.clone(),
            },
        )
        .map_err(|e| art(e.to_string()))?;

        // Base placement hint: the root body's stage-world pose with frame
        // semantics (conjugated up-axis fix).
        let root_body_path = &body_paths[model.root_link];
        let (raw, _) = decompose_matrix(&prim_by_path[root_body_path.as_str()].world);
        let root_pose = Isometry3::from_parts(
            Translation3::from(self.up_fix * (raw.translation.vector * self.mpu)),
            self.up_fix * raw.rotation * self.up_fix.inverse(),
        );

        Ok(ImportedRobot {
            model,
            root_pose,
            warnings: self.warnings,
        })
    }

    /// Reads one joint prim; `None` for joints missing body1.
    fn read_joint(
        &mut self,
        info: &PrimInfo,
        joint_type: JointType,
    ) -> anyhow::Result<Option<JointRecord>> {
        // All joint types share the JointBase attribute interface; view the
        // prim through a local wrapper so bodies/localPose read uniformly.
        let joint = AnyJoint(info.prim.clone());
        let first_target = |rel: openusd::usd::Relationship| -> anyhow::Result<Option<String>> {
            Ok(rel.targets()?.first().map(|p| p.to_string()))
        };
        let body0 = first_target(joint.body0_rel())?;
        let Some(body1) = first_target(joint.body1_rel())? else {
            self.warnings
                .push(format!("{}: joint has no body1; skipped", info.path));
            return Ok(None);
        };

        let pos = |attr: openusd::usd::Attribute| -> anyhow::Result<Vector3<f64>> {
            Ok(attr
                .get::<[f32; 3]>()?
                .map(|p| Vector3::new(p[0] as f64, p[1] as f64, p[2] as f64))
                .unwrap_or_else(Vector3::zeros))
        };
        let rot = |attr: openusd::usd::Attribute| -> anyhow::Result<UnitQuaternion<f64>> {
            Ok(attr
                .get::<gf::Quatf>()?
                .map(|q| {
                    UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        q.w as f64, q.x as f64, q.y as f64, q.z as f64,
                    ))
                })
                .unwrap_or_else(UnitQuaternion::identity))
        };
        let local_pose0 = Isometry3::from_parts(
            Translation3::from(pos(joint.local_pos0_attr())?),
            rot(joint.local_rot0_attr())?,
        );
        let local_pose1 = Isometry3::from_parts(
            Translation3::from(pos(joint.local_pos1_attr())?),
            rot(joint.local_rot1_attr())?,
        );

        // Axis + limits live on the concrete views.
        let (axis_token, lower, upper) = match joint_type {
            JointType::Revolute => {
                let v = physics::RevoluteJoint::get(self.stage, info.prim.path().clone())?
                    .expect("typed by type_name");
                (
                    v.axis_attr().get::<openusd::tf::Token>()?,
                    v.lower_limit_attr().get::<f32>()?.map(f64::from),
                    v.upper_limit_attr().get::<f32>()?.map(f64::from),
                )
            }
            JointType::Prismatic => {
                let v = physics::PrismaticJoint::get(self.stage, info.prim.path().clone())?
                    .expect("typed by type_name");
                (
                    v.axis_attr().get::<openusd::tf::Token>()?,
                    v.lower_limit_attr().get::<f32>()?.map(f64::from),
                    v.upper_limit_attr().get::<f32>()?.map(f64::from),
                )
            }
            _ => (None, None, None),
        };
        let axis = match axis_token.as_ref().map(|t| t.as_str()) {
            Some("X") => Vector3::x(),
            Some("Y") => Vector3::y(),
            _ => Vector3::z(),
        };

        // PhysX velocity limit (deg/s for revolute, units/s for prismatic).
        let max_velocity = info
            .prim
            .attribute("physxJoint:maxJointVelocity")
            .get::<f32>()
            .ok()
            .flatten()
            .map(f64::from);
        // Drive effort (angular or linear namespace).
        let effort = ["drive:angular:physics:maxForce", "drive:linear:physics:maxForce"]
            .iter()
            .find_map(|name| info.prim.attribute(*name).get::<f32>().ok().flatten())
            .map(f64::from)
            .unwrap_or(0.0);

        Ok(Some(JointRecord {
            name: info.path.clone(),
            joint_type,
            body0,
            body1,
            local_pose0,
            local_pose1,
            axis,
            lower,
            upper,
            max_velocity,
            effort,
        }))
    }

    fn joint_limits(&self, joint: &JointRecord) -> Option<JointLimits> {
        match joint.joint_type {
            JointType::Revolute => {
                let (lower, upper) = (joint.lower?, joint.upper?);
                Some(JointLimits {
                    lower: lower.to_radians(),
                    upper: upper.to_radians(),
                    velocity: joint.max_velocity.map(f64::to_radians).unwrap_or(0.0),
                    effort: joint.effort,
                })
            }
            JointType::Prismatic => {
                let (lower, upper) = (joint.lower?, joint.upper?);
                Some(JointLimits {
                    lower: lower * self.mpu,
                    upper: upper * self.mpu,
                    velocity: joint.max_velocity.map(|v| v * self.mpu).unwrap_or(0.0),
                    effort: joint.effort,
                })
            }
            _ => None,
        }
    }

    /// Gathers geometry under `body` (stopping at nested bodies), expressed
    /// in the body's corrected model frame.
    fn build_link(
        &mut self,
        body: &PrimInfo,
        prims: &[PrimInfo],
        body_paths: &[String],
        correction: &Isometry3<f64>,
    ) -> Link {
        let body_prefix = format!("{}/", body.path);
        let (body_raw, _) = decompose_matrix(&body.world);
        let mut visuals = Vec::new();
        let mut collisions = Vec::new();

        for info in prims {
            let is_self = info.path == body.path;
            if !is_self && !info.path.starts_with(&body_prefix) {
                continue;
            }
            // Stop at nested rigid bodies (their geometry belongs to them).
            if !is_self
                && body_paths
                    .iter()
                    .any(|b| b != &body.path && (info.path == *b || info.path.starts_with(&format!("{b}/"))))
            {
                continue;
            }
            let geometry = match self.read_geometry(info) {
                Ok(Some(g)) => g,
                Ok(None) => continue,
                Err(e) => {
                    self.warnings.push(format!("{}: {e}", info.path));
                    continue;
                }
            };
            // X_body_geom in raw stage units, then corrected + normalized.
            let (geom_raw, _) = decompose_matrix(&info.world);
            let relative = body_raw.inverse() * geom_raw;
            let origin = self.conjugate(&(correction.inverse() * relative));
            let shape = Shape { origin, geometry };
            if info.visible && info.default_purpose {
                visuals.push(shape.clone());
            }
            if info.has_collision_api {
                collisions.push(shape);
            }
        }

        Link {
            name: body.path.clone(),
            visuals,
            collisions,
            parent_joint: None,
        }
    }

    /// Reads a gprim as botrail geometry (meshes materialized as STL, with
    /// residual scale/shear and unit conversion baked into the data).
    fn read_geometry(&mut self, info: &PrimInfo) -> anyhow::Result<Option<Geometry>> {
        let (_, residual) = decompose_matrix(&info.world);
        let scale = [
            residual.column(0).norm(),
            residual.column(1).norm(),
            residual.column(2).norm(),
        ];
        let geometry = match info.type_name.as_str() {
            "Mesh" => {
                let Some(mesh) = geom::Mesh::get(self.stage, info.prim.path().clone())? else {
                    return Ok(None);
                };
                let points: Vec<[f32; 3]> = mesh.points_attr().get()?.unwrap_or_default();
                let counts = int_vec(mesh.face_vertex_counts_attr().get::<sdf::Value>()?);
                let face_indices = int_vec(mesh.face_vertex_indices_attr().get::<sdf::Value>()?);
                let data = triangulate(&points, &counts, &face_indices)
                    .map_err(|e| anyhow::anyhow!("mesh: {e}"))?;
                let baked = MeshData {
                    vertices: data
                        .vertices
                        .iter()
                        .map(|v| {
                            let p =
                                self.up_fix * (residual * Vector3::new(v[0], v[1], v[2]) * self.mpu);
                            [p.x, p.y, p.z]
                        })
                        .collect(),
                    indices: data.indices,
                };
                Geometry::Mesh {
                    path: {
                        let dir = self
                            .mesh_cache_dir
                            .clone()
                            .unwrap_or_else(default_mesh_cache_dir);
                        write_stl_cached(&dir, &baked)?
                    },
                    scale: Vector3::new(1.0, 1.0, 1.0),
                }
            }
            "Cube" => {
                let size: f64 = geom::Cube::get(self.stage, info.prim.path().clone())?
                    .and_then(|c| c.size_attr().get().transpose())
                    .transpose()?
                    .unwrap_or(2.0);
                Geometry::Box {
                    size: Vector3::new(
                        size * scale[0] * self.mpu,
                        size * scale[1] * self.mpu,
                        size * scale[2] * self.mpu,
                    ),
                }
            }
            "Sphere" => {
                let radius: f64 = geom::Sphere::get(self.stage, info.prim.path().clone())?
                    .and_then(|s| s.radius_attr().get().transpose())
                    .transpose()?
                    .unwrap_or(1.0);
                let s = scale[0].max(scale[1]).max(scale[2]);
                Geometry::Sphere {
                    radius: radius * s * self.mpu,
                }
            }
            "Cylinder" => {
                let Some(cyl) = geom::Cylinder::get(self.stage, info.prim.path().clone())? else {
                    return Ok(None);
                };
                let radius: f64 = cyl.radius_attr().get()?.unwrap_or(1.0);
                let height: f64 = cyl.height_attr().get()?.unwrap_or(2.0);
                let s = scale[0].max(scale[1]).max(scale[2]);
                // Note: axis re-orientation is handled by the shape origin in
                // scene import; link shapes keep it simple (Z only, warn).
                if let Ok(Some(axis)) = cyl.axis_attr().get::<openusd::tf::Token>() {
                    if axis.as_str() != "Z" {
                        self.warnings.push(format!(
                            "{}: cylinder axis {} approximated as Z",
                            info.path,
                            axis.as_str()
                        ));
                    }
                }
                Geometry::Cylinder {
                    radius: radius * s * self.mpu,
                    length: height * s * self.mpu,
                }
            }
            _ => return Ok(None),
        };
        Ok(Some(geometry))
    }

    /// Mesh vertices are already rotated into Z-up (`up_fix * v`), so link
    /// transforms conjugate: X' = F X F⁻¹ (translations scaled to meters).
    fn conjugate(&self, x: &Isometry3<f64>) -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::from(self.up_fix * (x.translation.vector * self.mpu)),
            self.up_fix * x.rotation * self.up_fix.inverse(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 2-DOF arm authored as a USD articulation (meters, Z-up): fixed
    /// world anchor, revolute Z at z=0.5 with localPose1 offset, revolute Y
    /// at z=0.4. j1's joint frame sits 0.2 *above* link1's body frame
    /// (localPos1 = (0,0,-0.2)), so link1's geometry (at its body origin)
    /// must land 0.2 below the model link frame — exercised against a URDF
    /// twin authored directly in joint frames.
    const ARM: &str = r#"#usda 1.0
(
    defaultPrim = "Robot"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "Robot" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
{
    def Xform "base" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "geom"
        {
            double size = 0.1
        }
    }

    def Xform "link1" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "geom"
        {
            double size = 0.1
        }
    }

    def Xform "link2" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "geom"
        {
            double size = 0.1
        }
    }

    def Scope "joints"
    {
        def PhysicsFixedJoint "anchor"
        {
            rel physics:body1 = </Robot/base>
        }

        def PhysicsRevoluteJoint "j1"
        {
            rel physics:body0 = </Robot/base>
            rel physics:body1 = </Robot/link1>
            uniform token physics:axis = "Z"
            point3f physics:localPos0 = (0, 0, 0.5)
            point3f physics:localPos1 = (0, 0, -0.2)
            float physics:lowerLimit = -90
            float physics:upperLimit = 90
            custom float physxJoint:maxJointVelocity = 120
        }

        def PhysicsRevoluteJoint "j2"
        {
            rel physics:body0 = </Robot/link1>
            rel physics:body1 = </Robot/link2>
            uniform token physics:axis = "Y"
            point3f physics:localPos0 = (0, 0, 0.2)
            float physics:lowerLimit = -120
            float physics:upperLimit = 120
        }
    }
}
"#;

    /// The same robot in URDF joint-frame terms: j1 at z=0.5 (link1 frame =
    /// joint frame, geometry 0.2 below); j2 at 0.2 above j1's frame... but
    /// note j2's localPos0 is authored in link1's BODY frame, which sits
    /// 0.2 below the model frame: origin = K1^-1 * localPose0 = 0.2 + 0.2.
    const TWIN: &str = r#"
    <robot name="twin">
      <link name="base">
        <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
      </link>
      <link name="link1">
        <visual><origin xyz="0 0 0.2"/><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
      </link>
      <link name="link2">
        <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
      </link>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="link1"/>
        <origin xyz="0 0 0.5"/><axis xyz="0 0 1"/>
        <limit lower="-1.5707963" upper="1.5707963" effort="0" velocity="2.0943951"/>
      </joint>
      <joint name="j2" type="revolute">
        <parent link="link1"/><child link="link2"/>
        <origin xyz="0 0 0.4"/><axis xyz="0 1 0"/>
        <limit lower="-2.0943951" upper="2.0943951" effort="0" velocity="1"/>
      </joint>
    </robot>"#;

    fn import_arm(usda: &str) -> ImportedRobot {
        // Unique per call: parallel tests must not share (and then delete)
        // each other's scratch dirs.
        static COUNTER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let n = COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!(
            "botrail-usd-art-{}-{n}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("robot.usda");
        std::fs::write(&path, usda).unwrap();
        let imported = import_robot(
            &path,
            &RobotImportOptions {
                mesh_cache_dir: Some(dir.join("meshes")),
                ..Default::default()
            },
        )
        .unwrap();
        let _ = std::fs::remove_dir_all(&dir);
        imported
    }

    /// World pose of every link's first visual shape — the frame-invariant
    /// quantity to compare across the two encodings.
    fn visual_world_poses(model: &RobotModel, q: &[f64]) -> Vec<Isometry3<f64>> {
        let link_poses = botrail_kin::forward_kinematics(model, q).unwrap();
        model
            .links
            .iter()
            .enumerate()
            .filter(|(_, l)| !l.visuals.is_empty())
            .map(|(i, l)| link_poses[i] * l.visuals[0].origin)
            .collect()
    }

    #[test]
    fn matches_urdf_twin_across_configurations() {
        let imported = import_arm(ARM);
        assert!(imported.warnings.is_empty(), "{:?}", imported.warnings);
        let model = &imported.model;
        let twin = RobotModel::from_urdf_str(TWIN).unwrap();

        assert_eq!(model.dof(), 2);
        // Naming contract: prim paths.
        assert_eq!(
            model.actuated_joint_names(),
            vec!["/Robot/joints/j1", "/Robot/joints/j2"]
        );
        assert_eq!(model.links[model.root_link].name, "/Robot/base");

        // Degrees -> radians, physx velocity deg/s -> rad/s.
        let j1 = &model.joints[model.joint_index("/Robot/joints/j1").unwrap()];
        let limits = j1.limits.unwrap();
        assert!((limits.lower + std::f64::consts::FRAC_PI_2).abs() < 1e-6);
        assert!((limits.velocity - 120f64.to_radians()).abs() < 1e-6);

        // Geometry world placement matches the URDF twin at several
        // configurations (the localPose1 offset must cancel exactly).
        for q in [
            vec![0.0, 0.0],
            vec![0.7, 0.0],
            vec![0.0, -1.1],
            vec![0.9, 0.6],
            vec![-1.2, 1.8],
        ] {
            let ours = visual_world_poses(model, &q);
            let theirs = visual_world_poses(&twin, &q);
            assert_eq!(ours.len(), theirs.len());
            for (a, b) in ours.iter().zip(&theirs) {
                let dp = (a.translation.vector - b.translation.vector).norm();
                let dr = a.rotation.angle_to(&b.rotation);
                assert!(dp < 1e-6 && dr < 1e-6, "q={q:?}: dp={dp} dr={dr}");
            }
        }
    }

    #[test]
    fn source_records_stage_reference() {
        let imported = import_arm(ARM);
        match &imported.model.source {
            RobotSource::Usd {
                articulation_root, ..
            } => assert_eq!(articulation_root, "/Robot"),
            other => panic!("expected Usd source, got {other:?}"),
        }
    }
}

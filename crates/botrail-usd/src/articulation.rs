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
//!
//! # Joint couplings (mimic)
//!
//! Two encodings collapse a driven joint into its source's DOF:
//! `PhysxMimicJointAPI` (USD units, PhysX sign convention) and the
//! `botrail:mimic` customData dictionary URDF converters author (URDF
//! semantics: `q = multiplier·q_source + offset`, SI units regardless of
//! stage units). When both are present the applied schema wins.

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
    decompose_matrix, default_mesh_cache_dir, display_color, int_vec, triangulate,
    write_stl_cached, y_up_to_z_up, AnyPrim, SearchPathResolver, UsdImportError,
};

/// Every `PhysxMimicJointAPI` instance name (one per dof of a joint).
const MIMIC_INSTANCES: [&str; 6] = ["rotX", "rotY", "rotZ", "transX", "transY", "transZ"];

/// The API instance a revolute/prismatic joint's single dof answers to:
/// its `physics:axis` letter under the prefix its type implies.
fn axis_instance(joint_type: JointType, axis_token: Option<&str>) -> Option<&'static str> {
    instance_for(
        joint_type,
        match axis_token {
            Some("X") => 'X',
            Some("Y") => 'Y',
            _ => 'Z',
        },
    )
}

/// [`axis_instance`] from a raw (pre up-axis-fix) joint axis, which the
/// importer always builds as an exact basis vector.
fn raw_axis_instance(joint_type: JointType, axis: &Vector3<f64>) -> Option<&'static str> {
    instance_for(
        joint_type,
        if axis.x != 0.0 {
            'X'
        } else if axis.y != 0.0 {
            'Y'
        } else {
            'Z'
        },
    )
}

fn instance_for(joint_type: JointType, letter: char) -> Option<&'static str> {
    match (joint_type, letter) {
        (JointType::Revolute | JointType::Continuous, 'X') => Some("rotX"),
        (JointType::Revolute | JointType::Continuous, 'Y') => Some("rotY"),
        (JointType::Revolute | JointType::Continuous, _) => Some("rotZ"),
        (JointType::Prismatic, 'X') => Some("transX"),
        (JointType::Prismatic, 'Y') => Some("transY"),
        (JointType::Prismatic, _) => Some("transZ"),
        (JointType::Fixed, _) => None,
    }
}

/// Untyped joint view granting the shared [`JointBase`] accessors
/// (bodies, localPose0/1) for any concrete joint type.
pub(crate) struct AnyJoint(pub(crate) Prim);

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
    /// See [`crate::ImportOptions::meshes_in_memory`]. Link geometry then
    /// names its triangles `usd:/<prim>` and ships them in
    /// [`ImportedRobot::meshes`] instead of writing STL files.
    pub meshes_in_memory: bool,
}

pub struct ImportedRobot {
    pub model: RobotModel,
    /// Stage-world pose of the robot's root body (normalized to meters /
    /// Z-up, frame semantics) — a hint for initial base placement.
    pub root_pose: Isometry3<f64>,
    pub warnings: Vec<String>,
    /// Populated when [`RobotImportOptions::meshes_in_memory`] is set: the
    /// baked triangles behind each `usd:/<prim>` geometry path. Register
    /// them with `botrail_collide::mesh::register_memory_mesh` before
    /// building colliders.
    pub meshes: Vec<(PathBuf, MeshData)>,
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
    /// Joint coupling, as authored (see [`MimicRecord`] for units).
    mimic: Option<MimicRecord>,
}

/// A joint coupling read off a joint prim, in one of the two encodings
/// botrail understands.
enum MimicRecord {
    /// One `PhysxMimicJointAPI` instance, straight off the prim. PhysX
    /// couples the pair as `qA + gearing·qB + offset = 0`, in USD joint
    /// units (degrees for angular dofs, stage units for linear ones).
    Physx {
        /// Prim path of the reference joint (`...:referenceJoint`).
        reference: String,
        /// The reference joint's coupled axis (`...:referenceJointAxis`),
        /// e.g. `rotZ`; `None` when unauthored.
        reference_axis: Option<String>,
        gearing: f64,
        offset: f64,
    },
    /// The `botrail:mimic` customData dictionary, authored by URDF-to-USD
    /// converters (UsdPhysics has no native mimic schema). It carries the
    /// URDF `<mimic>` relation verbatim: `q = multiplier·q_source + offset`
    /// in SI units (radians/meters), independent of stage units.
    CustomData {
        /// Name of the source joint — its prim name (last path segment),
        /// or a full prim path.
        joint: String,
        multiplier: f64,
        offset: f64,
    },
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
    import_robot_stage(&stage, path, options)
}

/// Imports a robot from a set of in-memory layers — the browser path, where
/// there is no filesystem to resolve references against. `root` names the
/// entry layer within `layers` (e.g. `franka.usd`); every layer it
/// references must be present under the path the reference spells.
///
/// Forces `meshes_in_memory`: writing STL files is exactly what this variant
/// exists to avoid.
pub fn import_robot_bundle(
    layers: Vec<(String, Vec<u8>)>,
    root: &str,
    options: &RobotImportOptions,
) -> Result<ImportedRobot, UsdImportError> {
    let stage = Stage::builder()
        .resolver(crate::BundleResolver::new(layers))
        .open(root)
        .map_err(|e| UsdImportError::Open {
            path: root.to_string(),
            message: e.to_string(),
        })?;
    let options = RobotImportOptions {
        meshes_in_memory: true,
        ..options.clone()
    };
    import_robot_stage(&stage, Path::new(root), &options)
}

fn import_robot_stage(
    stage: &Stage,
    path: &Path,
    options: &RobotImportOptions,
) -> Result<ImportedRobot, UsdImportError> {
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
        stage,
        source_path: path.to_path_buf(),
        mpu,
        up_fix,
        mesh_cache_dir: options.mesh_cache_dir.clone(),
        meshes_in_memory: options.meshes_in_memory,
        meshes: Vec::new(),
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
    let is_body = view
        .prim()
        .has_api_schema("PhysicsRigidBodyAPI")
        .unwrap_or(false);
    let is_articulation_root = view
        .prim()
        .has_api_schema("PhysicsArticulationRootAPI")
        .unwrap_or(false);
    let has_collision_api = view
        .prim()
        .has_api_schema("PhysicsCollisionAPI")
        .unwrap_or(false);
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
    meshes_in_memory: bool,
    meshes: Vec<(PathBuf, MeshData)>,
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
            None => {
                let mut roots = prims
                    .iter()
                    .filter(|p| p.is_articulation_root)
                    .map(|p| p.path.clone());
                let first =
                    roots.next().ok_or_else(|| {
                        art("no prim with PhysicsArticulationRootAPI found (pass articulation_root)"
                        .to_string())
                    })?;
                let rest: Vec<String> = roots.collect();
                if !rest.is_empty() {
                    self.warnings.push(format!(
                        "stage has multiple articulation roots — importing `{first}` \
                             (also found: {}); pass articulation_root to choose",
                        rest.join(", ")
                    ));
                }
                first
            }
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
        // Joints may reference bodies outside the subtree (a shared mount
        // in a multi-robot stage). Those bodies must not be dragged into
        // this model: an outside parent acts exactly like a world anchor
        // (it fixes the base), and an outside child cannot be modeled.
        for joint in &mut joints {
            if joint.body0.as_ref().is_some_and(|b0| !in_subtree(b0)) {
                let b0 = joint.body0.take().expect("checked above");
                self.warnings.push(format!(
                    "{}: parent body `{b0}` is outside `{root_path}`; treated as a world anchor",
                    joint.name
                ));
            }
        }
        let mut kept = Vec::with_capacity(joints.len());
        for joint in joints {
            if in_subtree(&joint.body1) {
                kept.push(joint);
            } else {
                self.warnings.push(format!(
                    "{}: child body `{}` is outside `{root_path}`; joint skipped",
                    joint.name, joint.body1
                ));
            }
        }
        let mut joints = kept;
        // A stage may articulate a static part — rigid bodies wired by no
        // joint at all (couplings, fingertips), at most world-anchored. A
        // rigid assembly *is* a weld: anchor every body to one root at its
        // relative stage pose, and the model comes out with zero DOF.
        if !joints.iter().any(|j| j.body0.is_some()) {
            let bodies: Vec<&PrimInfo> = prims
                .iter()
                .filter(|p| p.is_body && in_subtree(&p.path))
                .collect();
            if bodies.is_empty() && joints.is_empty() {
                return Err(art(format!(
                    "no physics joints or rigid bodies under `{root_path}`"
                )));
            }
            if let Some(&first_body) = bodies.first() {
                // Weld root: the world-anchored body when one is named,
                // else the first body in prim order.
                let first = joints
                    .iter()
                    .find_map(|anchor| bodies.iter().copied().find(|b| b.path == anchor.body1))
                    .unwrap_or(first_body);
                let (first_iso, _) = decompose_matrix(&first.world);
                for body in bodies.iter().filter(|b| b.path != first.path) {
                    let (iso, _) = decompose_matrix(&body.world);
                    joints.push(JointRecord {
                        // `:` cannot appear in a prim name, so the synthetic
                        // name can never collide with a real joint's path.
                        name: format!("{}:weld", body.path),
                        joint_type: JointType::Fixed,
                        body0: Some(first.path.clone()),
                        body1: body.path.clone(),
                        local_pose0: first_iso.inverse() * iso,
                        local_pose1: Isometry3::identity(),
                        axis: Vector3::z(),
                        lower: None,
                        upper: None,
                        max_velocity: None,
                        effort: 0.0,
                        mimic: None,
                    });
                }
            }
        }
        let joints = joints;

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
                axis: Unit::try_new(axis, 1e-9)
                    .unwrap_or_else(|| Unit::new_unchecked(Vector3::z())),
                limits,
                parent_link: parent,
                child_link: child,
                q_index: None,
                mimic: None,
            });
        }
        self.resolve_mimics(&joints, &mut model_joints);

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
            meshes: self.meshes,
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
        let effort = [
            "drive:angular:physics:maxForce",
            "drive:linear:physics:maxForce",
        ]
        .iter()
        .find_map(|name| info.prim.attribute(*name).get::<f32>().ok().flatten())
        .map(f64::from)
        .unwrap_or(0.0);

        let physx = self.read_mimic(info, joint_type, axis_token.as_ref().map(|t| t.as_str()));
        let custom = self.read_custom_mimic(info, joint_type);
        let mimic = match (physx, custom) {
            (Some(physx), Some(_)) => {
                self.warnings.push(format!(
                    "{}: both PhysxMimicJointAPI and botrail:mimic customData authored; \
                     using PhysxMimicJointAPI",
                    info.path
                ));
                Some(physx)
            }
            (physx, custom) => physx.or(custom),
        };

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
            mimic,
        }))
    }

    /// Reads `PhysxMimicJointAPI` off a joint prim. The API is
    /// multiple-apply, keyed by the *mimic* joint's own axis
    /// (`rotX`..`transZ`); a revolute/prismatic joint has exactly one axis,
    /// so only that instance can be represented.
    fn read_mimic(
        &mut self,
        info: &PrimInfo,
        joint_type: JointType,
        axis_token: Option<&str>,
    ) -> Option<MimicRecord> {
        let own_instance = axis_instance(joint_type, axis_token)?;
        let rel = |instance: &str| {
            info.prim
                .relationship(format!("physxMimicJoint:{instance}:referenceJoint").as_str())
                .targets()
                .ok()
                .and_then(|t| t.first().map(|p| p.to_string()))
        };
        let Some(reference) = rel(own_instance) else {
            // Report a coupling authored on an axis this joint does not
            // move rather than dropping it silently.
            if let Some(other) = MIMIC_INSTANCES
                .iter()
                .find(|instance| **instance != own_instance && rel(instance).is_some())
            {
                self.warnings.push(format!(
                    "{}: mimic joint authored on `{other}` but the joint moves about `{own_instance}`; ignored",
                    info.path
                ));
            }
            return None;
        };
        let attr = |suffix: &str| {
            info.prim
                .attribute(format!("physxMimicJoint:{own_instance}:{suffix}").as_str())
                .get::<f32>()
                .ok()
                .flatten()
                .map(f64::from)
        };
        Some(MimicRecord::Physx {
            reference,
            reference_axis: info
                .prim
                .attribute(format!("physxMimicJoint:{own_instance}:referenceJointAxis").as_str())
                .get::<openusd::tf::Token>()
                .ok()
                .flatten()
                .map(|t| t.to_string()),
            gearing: attr("gearing").unwrap_or(1.0),
            offset: attr("offset").unwrap_or(0.0),
        })
    }

    /// Reads the `botrail:mimic` customData dictionary off a joint prim.
    /// pxr treats the `:` in `SetCustomDataByKey("botrail:mimic", ...)` as a
    /// namespace delimiter and nests the entry
    /// (`customData = { dictionary botrail = { dictionary mimic = {...} } }`),
    /// so look there first and fall back to a flat `botrail:mimic` key.
    fn read_custom_mimic(&mut self, info: &PrimInfo, joint_type: JointType) -> Option<MimicRecord> {
        let sdf::Value::Dictionary(top) = info.prim.custom_data().ok().flatten()? else {
            return None;
        };
        let entry = match top.get("botrail") {
            Some(sdf::Value::Dictionary(ns)) => ns.get("mimic"),
            _ => top.get("botrail:mimic"),
        }?;
        let sdf::Value::Dictionary(entry) = entry else {
            self.warnings.push(format!(
                "{}: botrail:mimic customData is not a dictionary; ignored",
                info.path
            ));
            return None;
        };
        if joint_type == JointType::Fixed {
            self.warnings.push(format!(
                "{}: botrail:mimic customData on a fixed joint; ignored",
                info.path
            ));
            return None;
        }
        let joint = match entry.get("joint") {
            Some(sdf::Value::String(s)) => s.clone(),
            Some(sdf::Value::Token(t)) => t.to_string(),
            _ => {
                self.warnings.push(format!(
                    "{}: botrail:mimic customData names no source `joint`; ignored",
                    info.path
                ));
                return None;
            }
        };
        let number = |key: &str, default: f64| match entry.get(key) {
            Some(sdf::Value::Double(d)) => *d,
            Some(sdf::Value::Float(f)) => *f as f64,
            Some(sdf::Value::Int(i)) => *i as f64,
            _ => default,
        };
        Some(MimicRecord::CustomData {
            joint,
            multiplier: number("multiplier", 1.0),
            offset: number("offset", 0.0),
        })
    }

    /// Turns each joint's coupling record into a model mimic relation.
    ///
    /// PhysX constrains the pair as `qA + G·qB + γ = 0`, i.e.
    /// `qA = -G·qB - γ`, with USD's joint units (degrees for angular dofs,
    /// stage units for linear ones). Converting both sides into botrail's
    /// radians/meters scales the gearing by the two joints' unit factors and
    /// the offset by the mimic joint's own. `botrail:mimic` customData
    /// already speaks URDF (SI, `q = m·q_src + o`) and passes through
    /// unchanged.
    fn resolve_mimics(&mut self, records: &[JointRecord], model_joints: &mut [Joint]) {
        let index: HashMap<String, usize> = model_joints
            .iter()
            .enumerate()
            .map(|(i, j)| (j.name.clone(), i))
            .collect();
        let record_by_name: HashMap<&str, &JointRecord> =
            records.iter().map(|r| (r.name.as_str(), r)).collect();

        for record in records {
            let Some(mimic) = &record.mimic else { continue };
            let Some(&ji) = index.get(record.name.as_str()) else {
                continue; // world anchor: no model joint to carry the relation
            };
            let resolved = match mimic {
                MimicRecord::Physx {
                    reference,
                    reference_axis,
                    gearing,
                    offset,
                } => {
                    let Some(&source_joint) = index.get(reference.as_str()) else {
                        self.warnings.push(format!(
                            "{}: mimic reference joint `{reference}` is not part of this articulation; ignored",
                            record.name
                        ));
                        continue;
                    };
                    let Some(source) = record_by_name.get(reference.as_str()) else {
                        continue;
                    };
                    // The model gives every joint one axis, so a coupling to
                    // some other axis of the reference joint cannot be
                    // represented.
                    let source_instance = raw_axis_instance(source.joint_type, &source.axis);
                    if let (Some(authored), Some(actual)) = (reference_axis, source_instance) {
                        if authored != actual {
                            self.warnings.push(format!(
                                "{}: mimic references `{reference}` on axis `{authored}`, which is not the axis it moves about (`{actual}`); ignored",
                                record.name
                            ));
                            continue;
                        }
                    }
                    let (Some(k_a), Some(k_b)) = (
                        self.joint_unit(record.joint_type),
                        self.joint_unit(source.joint_type),
                    ) else {
                        continue;
                    };
                    Some(botrail_model::JointMimic {
                        source_joint,
                        multiplier: -gearing * k_a / k_b,
                        offset: -offset * k_a,
                    })
                }
                MimicRecord::CustomData {
                    joint,
                    multiplier,
                    offset,
                } => self
                    .resolve_custom_source(&record.name, joint, ji, model_joints, &index)
                    .map(|source_joint| botrail_model::JointMimic {
                        source_joint,
                        multiplier: *multiplier,
                        offset: *offset,
                    }),
            };
            if let Some(resolved) = resolved {
                model_joints[ji].mimic = Some(resolved);
            }
        }
    }

    /// Finds the model joint a `botrail:mimic` entry names: an exact prim
    /// path if given one, otherwise the unique joint whose prim name (last
    /// path segment) matches. Rejections warn rather than fail the import —
    /// the joint then simply keeps its own DOF.
    fn resolve_custom_source(
        &mut self,
        mimic_name: &str,
        source: &str,
        ji: usize,
        model_joints: &[Joint],
        index: &HashMap<String, usize>,
    ) -> Option<usize> {
        let found = match index.get(source) {
            Some(&si) => Some(si),
            None => {
                let matches: Vec<usize> = model_joints
                    .iter()
                    .enumerate()
                    .filter(|(_, j)| j.name.rsplit('/').next() == Some(source))
                    .map(|(i, _)| i)
                    .collect();
                match matches.as_slice() {
                    [one] => Some(*one),
                    [] => {
                        self.warnings.push(format!(
                            "{mimic_name}: botrail:mimic source `{source}` is not part of this articulation; ignored"
                        ));
                        None
                    }
                    _ => {
                        self.warnings.push(format!(
                            "{mimic_name}: botrail:mimic source `{source}` matches several joints; ignored"
                        ));
                        None
                    }
                }
            }
        }?;
        // Cheap local checks for relations the model would reject with a
        // hard error (the importer's contract is warn-and-continue).
        if found == ji {
            self.warnings.push(format!(
                "{mimic_name}: botrail:mimic source is the joint itself; ignored"
            ));
            return None;
        }
        if model_joints[found].joint_type == JointType::Fixed {
            self.warnings.push(format!(
                "{mimic_name}: botrail:mimic source `{source}` is a fixed joint; ignored"
            ));
            return None;
        }
        Some(found)
    }

    /// botrail units per USD unit for a joint's position: degrees to
    /// radians for angular dofs, stage units to meters for linear ones.
    fn joint_unit(&self, joint_type: JointType) -> Option<f64> {
        match joint_type {
            JointType::Revolute | JointType::Continuous => Some(std::f64::consts::PI / 180.0),
            JointType::Prismatic => Some(self.mpu),
            JointType::Fixed => None,
        }
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
            // Geometry belongs to its *nearest* ancestor body: stop at
            // nested rigid bodies, but keep prims whose closest body is
            // this one (a body nested under another body still owns its
            // own subtree).
            if !is_self {
                let nearest = body_paths
                    .iter()
                    .filter(|b| info.path == **b || info.path.starts_with(&format!("{b}/")))
                    .max_by_key(|b| b.len());
                if nearest.map(|b| b != &body.path).unwrap_or(false) {
                    continue;
                }
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
            let mut shape = Shape {
                origin,
                geometry,
                color: None,
            };
            if info.visible && info.default_purpose {
                // The gprim's own shade, so a link keeps the colour its
                // stage gave it once something other than the stage draws
                // it (the wire, a re-export).
                shape.color = display_color(&AnyPrim(info.prim.clone()));
                visuals.push(shape.clone());
            }
            if info.has_collision_api {
                shape.color = None;
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
                            let p = self.up_fix
                                * (residual * Vector3::new(v[0], v[1], v[2]) * self.mpu);
                            [p.x, p.y, p.z]
                        })
                        .collect(),
                    indices: data.indices,
                    face_colors: Vec::new(),
                };
                Geometry::Mesh {
                    path: if self.meshes_in_memory {
                        let virtual_path = PathBuf::from(format!("usd:/{}", info.path));
                        self.meshes.push((virtual_path.clone(), baked));
                        virtual_path
                    } else {
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
        let dir = std::env::temp_dir().join(format!("botrail-usd-art-{}-{n}", std::process::id()));
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

    /// Coupled joints on a centimetre stage: `j2` follows `j1` (angular,
    /// degrees) and `j4` follows `j3` (linear, stage units), so both unit
    /// conversions of `PhysxMimicJointAPI` are exercised.
    const MIMIC: &str = r#"#usda 1.0
(
    defaultPrim = "Robot"
    metersPerUnit = 0.01
    upAxis = "Z"
)

def Xform "Robot" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
{
    def Xform "base" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) { def Cube "geom" { double size = 1 } }
    def Xform "link1" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) { def Cube "geom" { double size = 1 } }
    def Xform "link2" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) { def Cube "geom" { double size = 1 } }
    def Xform "left" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) { def Cube "geom" { double size = 1 } }
    def Xform "right" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) { def Cube "geom" { double size = 1 } }

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
            float physics:lowerLimit = -90
            float physics:upperLimit = 90
        }

        def PhysicsRevoluteJoint "j2" (prepend apiSchemas = ["PhysxMimicJointAPI:rotZ"])
        {
            rel physics:body0 = </Robot/link1>
            rel physics:body1 = </Robot/link2>
            uniform token physics:axis = "Z"
            float physics:lowerLimit = -90
            float physics:upperLimit = 90
            float physxMimicJoint:rotZ:gearing = -0.5
            float physxMimicJoint:rotZ:offset = 30
            rel physxMimicJoint:rotZ:referenceJoint = </Robot/joints/j1>
            uniform token physxMimicJoint:rotZ:referenceJointAxis = "rotZ"
        }

        def PhysicsPrismaticJoint "j3"
        {
            rel physics:body0 = </Robot/link2>
            rel physics:body1 = </Robot/left>
            uniform token physics:axis = "X"
            float physics:lowerLimit = 0
            float physics:upperLimit = 4
        }

        def PhysicsPrismaticJoint "j4" (prepend apiSchemas = ["PhysxMimicJointAPI:transX"])
        {
            rel physics:body0 = </Robot/left>
            rel physics:body1 = </Robot/right>
            uniform token physics:axis = "X"
            float physics:lowerLimit = -4
            float physics:upperLimit = 0
            float physxMimicJoint:transX:gearing = 1
            float physxMimicJoint:transX:offset = 2
            rel physxMimicJoint:transX:referenceJoint = </Robot/joints/j3>
        }
    }
}
"#;

    /// A stage that shades its links: the constant `displayColor` on a
    /// gprim is the robot's own appearance and rides into the model, so
    /// anything that draws from the model — the studio wire, a re-export —
    /// shows the arm the way its stage does. Collision proxies stay
    /// uncoloured: they are not part of the picture.
    #[test]
    fn link_display_colour_rides_into_the_model() {
        let usda = ARM
            .replacen(
                "        def Cube \"geom\"\n        {\n            double size = 0.1\n        }",
                "        def Cube \"geom\"\n        {\n            double size = 0.1\n                             color3f[] primvars:displayColor = [(0.9, 0.1, 0.05)]\n        }",
                1,
            );
        let imported = import_arm(&usda);
        let model = &imported.model;
        let base = &model.links[model.link_index("/Robot/base").unwrap()];
        assert_eq!(base.visuals[0].color, Some([0.9, 0.1, 0.05]));
        // The untouched links said nothing, and stay free to be shaded.
        let link1 = &model.links[model.link_index("/Robot/link1").unwrap()];
        assert_eq!(link1.visuals[0].color, None);
    }

    #[test]
    fn mimic_joints_import_as_coupled_dofs() {
        let imported = import_arm(MIMIC);
        let model = &imported.model;
        // Four moving joints, two of them driven: two DOFs.
        assert_eq!(model.dof(), 2, "{:?}", imported.warnings);
        assert_eq!(
            model.actuated_joint_names(),
            vec!["/Robot/joints/j1", "/Robot/joints/j3"]
        );

        // PhysX couples the pair as qA + G*qB + gamma = 0, so a gearing of
        // -0.5 means the joint turns at +0.5x, and the 30 deg offset lands
        // as -30 deg in radians.
        let angular = model.joints[model.joint_index("/Robot/joints/j2").unwrap()]
            .mimic
            .unwrap();
        assert_eq!(
            angular.source_joint,
            model.joint_index("/Robot/joints/j1").unwrap()
        );
        assert!((angular.multiplier - 0.5).abs() < 1e-12);
        assert!((angular.offset + 30f64.to_radians()).abs() < 1e-12);

        // Linear: gearing is unitless between two prismatic joints, the
        // offset converts out of centimetres.
        let linear = model.joints[model.joint_index("/Robot/joints/j4").unwrap()]
            .mimic
            .unwrap();
        assert!((linear.multiplier + 1.0).abs() < 1e-12);
        assert!((linear.offset + 0.02).abs() < 1e-12, "{}", linear.offset);
    }

    #[test]
    fn mimic_on_an_axis_the_joint_does_not_move_is_reported() {
        let usda = MIMIC
            .replace("PhysxMimicJointAPI:transX", "PhysxMimicJointAPI:rotX")
            .replace("physxMimicJoint:transX", "physxMimicJoint:rotX");
        let imported = import_arm(&usda);
        // j4 keeps its own DOF, and the dropped coupling is reported.
        assert_eq!(imported.model.dof(), 3);
        assert!(
            imported
                .warnings
                .iter()
                .any(|w| w.contains("/Robot/joints/j4") && w.contains("rotX")),
            "{:?}",
            imported.warnings
        );
    }

    /// huggingface_hub-style cache: the stage file is a symlink onto an
    /// extensionless content-addressed blob. The resolver canonicalizes,
    /// so without care the link's extension — and the format dispatch —
    /// would be lost.
    #[cfg(unix)]
    #[test]
    fn symlinked_stage_keeps_its_file_format() {
        static COUNTER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let n = COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("botrail-usd-ln-{}-{n}", std::process::id()));
        std::fs::create_dir_all(dir.join("blobs")).unwrap();
        std::fs::create_dir_all(dir.join("snap")).unwrap();
        std::fs::write(dir.join("blobs/0123abcd"), ARM).unwrap();
        std::os::unix::fs::symlink("../blobs/0123abcd", dir.join("snap/robot.usda")).unwrap();

        let imported = import_robot(
            &dir.join("snap/robot.usda"),
            &RobotImportOptions {
                mesh_cache_dir: Some(dir.join("meshes")),
                ..Default::default()
            },
        )
        .unwrap();
        assert_eq!(imported.model.dof(), 2, "{:?}", imported.warnings);
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The `botrail:mimic` customData encoding, exactly as pxr serializes
    /// `SetCustomDataByKey("botrail:mimic", {...})` — the `:` becomes a
    /// nested namespace dictionary. Authored on a centimetre stage with a
    /// radian offset: customData carries URDF SI values, so nothing may be
    /// rescaled on import.
    const CUSTOM_MIMIC: &str = r#"#usda 1.0
(
    defaultPrim = "Robot"
    metersPerUnit = 0.01
    upAxis = "Z"
)

def Xform "Robot" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
{
    def Xform "base" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) { def Cube "geom" { double size = 1 } }
    def Xform "link1" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) { def Cube "geom" { double size = 1 } }
    def Xform "link2" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) { def Cube "geom" { double size = 1 } }

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
            float physics:lowerLimit = -90
            float physics:upperLimit = 90
        }

        def PhysicsRevoluteJoint "j2" (
            customData = {
                dictionary botrail = {
                    dictionary mimic = {
                        string joint = "j1"
                        double multiplier = -0.5
                        double offset = 0.25
                    }
                }
            }
        )
        {
            rel physics:body0 = </Robot/link1>
            rel physics:body1 = </Robot/link2>
            uniform token physics:axis = "Z"
            float physics:lowerLimit = -90
            float physics:upperLimit = 90
        }
    }
}
"#;

    #[test]
    fn custom_data_mimic_imports_with_urdf_units() {
        let imported = import_arm(CUSTOM_MIMIC);
        let model = &imported.model;
        assert_eq!(model.dof(), 1, "{:?}", imported.warnings);
        assert_eq!(model.actuated_joint_names(), vec!["/Robot/joints/j1"]);

        // URDF semantics pass through untouched: no degree conversion, no
        // stage-unit scaling (the stage is centimetres, the offset radians).
        let mimic = model.joints[model.joint_index("/Robot/joints/j2").unwrap()]
            .mimic
            .unwrap();
        assert_eq!(
            mimic.source_joint,
            model.joint_index("/Robot/joints/j1").unwrap()
        );
        assert_eq!(mimic.multiplier, -0.5);
        assert_eq!(mimic.offset, 0.25);
    }

    #[test]
    fn custom_data_mimic_with_unknown_source_keeps_the_dof() {
        let usda = CUSTOM_MIMIC.replace("string joint = \"j1\"", "string joint = \"nope\"");
        let imported = import_arm(&usda);
        assert_eq!(imported.model.dof(), 2);
        assert!(
            imported
                .warnings
                .iter()
                .any(|w| w.contains("/Robot/joints/j2") && w.contains("`nope`")),
            "{:?}",
            imported.warnings
        );
    }

    #[test]
    fn physx_mimic_wins_over_custom_data() {
        // j2 carries both encodings with contradicting couplings; the
        // applied schema is authoritative and the conflict is reported.
        let usda = CUSTOM_MIMIC.replace(
            "        def PhysicsRevoluteJoint \"j2\" (",
            "        def PhysicsRevoluteJoint \"j2\" (\n            \
             prepend apiSchemas = [\"PhysxMimicJointAPI:rotZ\"]",
        );
        let usda = usda.replace(
            "            uniform token physics:axis = \"Z\"\n            float physics:lowerLimit = -90\n            float physics:upperLimit = 90\n        }\n    }\n}",
            "            uniform token physics:axis = \"Z\"\n            float physics:lowerLimit = -90\n            float physics:upperLimit = 90\n            float physxMimicJoint:rotZ:gearing = -0.5\n            float physxMimicJoint:rotZ:offset = 30\n            rel physxMimicJoint:rotZ:referenceJoint = </Robot/joints/j1>\n        }\n    }\n}",
        );
        let imported = import_arm(&usda);
        let model = &imported.model;
        assert_eq!(model.dof(), 1, "{:?}", imported.warnings);
        let mimic = model.joints[model.joint_index("/Robot/joints/j2").unwrap()]
            .mimic
            .unwrap();
        // PhysX semantics (gearing -0.5 -> +0.5x, 30 deg -> -30 deg), not
        // the customData numbers.
        assert!((mimic.multiplier - 0.5).abs() < 1e-12);
        assert!((mimic.offset + 30f64.to_radians()).abs() < 1e-12);
        assert!(
            imported
                .warnings
                .iter()
                .any(|w| w.contains("both PhysxMimicJointAPI and botrail:mimic")),
            "{:?}",
            imported.warnings
        );
    }

    /// A static part (coupling, fingertip): rigid bodies under an
    /// articulation root, no joints anywhere.
    const JOINT_LESS: &str = r#"#usda 1.0
(
    defaultPrim = "Part"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "Part" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
{
    def Xform "body" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "geom" { double size = 0.05 }
    }

    def Xform "cap" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        double3 xformOp:translate = (0.1, 0, 0.3)
        uniform token[] xformOpOrder = ["xformOp:translate"]

        def Cube "geom" { double size = 0.02 }
    }
}
"#;

    #[test]
    fn joint_less_bodies_weld_at_their_stage_poses() {
        let imported = import_arm(JOINT_LESS);
        let model = &imported.model;
        assert_eq!(model.dof(), 0, "{:?}", imported.warnings);
        assert_eq!(model.links.len(), 2);
        assert_eq!(model.links[model.root_link].name, "/Part/body");

        // The synthetic weld reproduces the relative stage pose.
        let poses = botrail_kin::forward_kinematics(model, &[]).unwrap();
        let cap = poses[model.link_index("/Part/cap").unwrap()].translation;
        assert!((cap.x - 0.1).abs() < 1e-9 && cap.z - 0.3 < 1e-9, "{cap:?}");
    }

    #[test]
    fn joint_less_single_body_imports_as_a_static_part() {
        // Strip the second body: a lone rigid body is the smallest dof=0 part.
        let start = JOINT_LESS.find("    def Xform \"cap\"").unwrap();
        let usda = format!("{}}}\n", &JOINT_LESS[..start]);
        let imported = import_arm(&usda);
        assert_eq!(imported.model.dof(), 0, "{:?}", imported.warnings);
        assert_eq!(imported.model.links.len(), 1);
    }

    #[test]
    fn world_anchored_static_part_roots_at_the_anchored_body() {
        // The pre-#4 workaround: a body1-only fixed joint anchoring the
        // part. It must keep loading, and the anchor picks the weld root —
        // here the *second* body, which prim order alone would not choose.
        let usda = JOINT_LESS.replace(
            "    def Xform \"cap\"",
            "    def PhysicsFixedJoint \"anchor\"\n    {\n        \
             rel physics:body1 = </Part/cap>\n    }\n\n    def Xform \"cap\"",
        );
        let imported = import_arm(&usda);
        let model = &imported.model;
        assert_eq!(model.dof(), 0, "{:?}", imported.warnings);
        assert_eq!(model.links.len(), 2);
        assert_eq!(model.links[model.root_link].name, "/Part/cap");
    }
    const DUAL: &str = r#"#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "World"
{
    def Xform "Base" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "geom"
        {
            double size = 0.2
        }
    }

    def Xform "ArmA" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
    {
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
            def PhysicsRevoluteJoint "mount"
            {
                rel physics:body0 = </World/Base>
                rel physics:body1 = </World/ArmA/link1>
                uniform token physics:axis = "Z"
                float physics:lowerLimit = -90
                float physics:upperLimit = 90
            }

            def PhysicsRevoluteJoint "elbow"
            {
                rel physics:body0 = </World/ArmA/link1>
                rel physics:body1 = </World/ArmA/link2>
                uniform token physics:axis = "Y"
                float physics:lowerLimit = -90
                float physics:upperLimit = 90
            }
        }
    }

    def Xform "ArmB" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
    {
        def Xform "link1" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
        {
            def Cube "geom"
            {
                double size = 0.1
            }
        }

        def Scope "joints"
        {
            def PhysicsRevoluteJoint "mount"
            {
                rel physics:body0 = </World/Base>
                rel physics:body1 = </World/ArmB/link1>
                uniform token physics:axis = "Z"
                float physics:lowerLimit = -90
                float physics:upperLimit = 90
            }
        }
    }
}
"#;

    fn import_dual(articulation_root: Option<&str>) -> ImportedRobot {
        static COUNTER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let n = COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!("botrail-usd-dual-{}-{n}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("dual.usda");
        std::fs::write(&path, DUAL).unwrap();
        let imported = import_robot(
            &path,
            &RobotImportOptions {
                mesh_cache_dir: Some(dir.join("meshes")),
                articulation_root: articulation_root.map(str::to_string),
                ..Default::default()
            },
        )
        .unwrap();
        let _ = std::fs::remove_dir_all(&dir);
        imported
    }

    #[test]
    fn shared_mount_body_stays_out_of_the_subtree_model() {
        let imported = import_dual(Some("/World/ArmA"));
        let names: Vec<&str> = imported
            .model
            .links
            .iter()
            .map(|l| l.name.as_str())
            .collect();
        // The outside mount body is NOT dragged in; the anchor joint acts
        // like a world anchor, so link1 is the base and elbow still moves.
        assert_eq!(names, vec!["/World/ArmA/link1", "/World/ArmA/link2"]);
        assert_eq!(imported.model.dof(), 1);
        assert!(
            imported
                .warnings
                .iter()
                .any(|w| w.contains("outside") && w.contains("/World/Base")),
            "{:?}",
            imported.warnings
        );
    }

    #[test]
    fn multiple_articulation_roots_warn_when_unspecified() {
        let imported = import_dual(None);
        // DFS-first root wins, loudly.
        assert!(imported
            .model
            .links
            .iter()
            .all(|l| l.name.starts_with("/World/ArmA")));
        assert!(
            imported
                .warnings
                .iter()
                .any(|w| w.contains("multiple articulation roots") && w.contains("/World/ArmB")),
            "{:?}",
            imported.warnings
        );
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

    /// The browser path: layers handed over as bytes, references between
    /// them resolved without touching a filesystem, and mesh geometry
    /// returned in memory rather than written to the cache.
    #[test]
    fn bundled_layers_compose_without_a_filesystem() {
        // A root layer whose link geometry lives in a referenced sublayer,
        // spelled the way a real asset does it (`./Props/...`).
        const ROOT: &str = r#"#usda 1.0
(
    defaultPrim = "Robot"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "Robot" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
{
    def Xform "base" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def "geom" (prepend references = @./Props/base.usda@</Body>) {}
    }

    def Xform "link1" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        double3 xformOp:translate = (0, 0, 0.5)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }

    def PhysicsRevoluteJoint "j1"
    {
        rel physics:body0 = </Robot/base>
        rel physics:body1 = </Robot/link1>
        uniform token physics:axis = "Z"
        point3f physics:localPos0 = (0, 0, 0.5)
        point3f physics:localPos1 = (0, 0, 0)
    }
}
"#;
        // A single tetrahedron: enough to prove the triangles came through.
        const PROP: &str = r#"#usda 1.0
(
    defaultPrim = "Body"
)
def Xform "Body"
{
    def Mesh "Collision" (prepend apiSchemas = ["PhysicsCollisionAPI"])
    {
        point3f[] points = [(0, 0, 0), (0.1, 0, 0), (0, 0.1, 0), (0, 0, 0.1)]
        int[] faceVertexCounts = [3, 3, 3, 3]
        int[] faceVertexIndices = [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3]
    }
}
"#;
        let layers = vec![
            ("robot.usda".to_string(), ROOT.as_bytes().to_vec()),
            ("Props/base.usda".to_string(), PROP.as_bytes().to_vec()),
        ];
        let imported =
            import_robot_bundle(layers, "robot.usda", &RobotImportOptions::default()).unwrap();

        assert_eq!(imported.model.dof(), 1, "{:?}", imported.warnings);
        // The referenced layer's mesh arrived, and as memory rather than a
        // path on disk.
        assert_eq!(imported.meshes.len(), 1, "{:?}", imported.warnings);
        let (path, mesh) = &imported.meshes[0];
        assert!(
            path.to_string_lossy().starts_with("usd:/"),
            "{}",
            path.display()
        );
        assert_eq!(mesh.indices.len(), 4);
        assert!(!path.exists(), "in-memory import must not write files");
    }

    /// The bundle path must agree with the filesystem path on a real
    /// multi-file asset. Runs only with `BOTRAIL_ISAAC_DIR` pointing at a
    /// downloaded Franka (same convention as the golden tests).
    #[test]
    fn bundled_franka_matches_the_filesystem_import() {
        let Some(dir) = std::env::var_os("BOTRAIL_ISAAC_DIR").map(PathBuf::from) else {
            return;
        };
        let root = dir.join("franka.usd");
        if !root.exists() {
            return;
        }
        let options = RobotImportOptions {
            articulation_root: Some("/panda".to_string()),
            ..Default::default()
        };
        let from_disk = import_robot(&root, &options).unwrap();

        // Everything under the asset directory, keyed the way the
        // references spell it (relative to franka.usd).
        let mut layers = Vec::new();
        let mut stack = vec![dir.clone()];
        while let Some(current) = stack.pop() {
            for entry in std::fs::read_dir(&current).unwrap().flatten() {
                let path = entry.path();
                if path.is_dir() {
                    stack.push(path);
                } else if path.extension().is_some_and(|e| e == "usd" || e == "usda") {
                    let rel = path
                        .strip_prefix(&dir)
                        .unwrap()
                        .to_string_lossy()
                        .to_string();
                    layers.push((rel, std::fs::read(&path).unwrap()));
                }
            }
        }
        let from_bundle = import_robot_bundle(layers, "franka.usd", &options).unwrap();

        assert_eq!(from_bundle.model.dof(), from_disk.model.dof());
        let names = |m: &RobotModel| m.links.iter().map(|l| l.name.clone()).collect::<Vec<_>>();
        assert_eq!(names(&from_bundle.model), names(&from_disk.model));
        assert_eq!(
            from_bundle.model.actuated_joint_limits().len(),
            from_disk.model.actuated_joint_limits().len()
        );
        // Geometry came through as memory, not as cache files.
        assert!(
            !from_bundle.meshes.is_empty(),
            "no mesh reached memory: {:?}",
            from_bundle.warnings
        );
    }

    /// Reference spellings that name the same layer resolve to one entry.
    #[test]
    fn bundle_paths_normalize() {
        assert_eq!(crate::normalize_bundle_path("./Props/a.usd"), "Props/a.usd");
        assert_eq!(
            crate::normalize_bundle_path("Props/../Props/a.usd"),
            "Props/a.usd"
        );
        assert_eq!(
            crate::normalize_bundle_path("omniverse://host/Props/a.usd"),
            "Props/a.usd"
        );
        assert_eq!(crate::normalize_bundle_path("/a.usd"), "a.usd");
    }
}

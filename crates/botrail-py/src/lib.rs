//! Python bindings for botrail (`botrail._core`).

mod catalog;
mod hub;
mod server;

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;

use botrail_model::RobotModel;
use pyo3::exceptions::{PyIOError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyDict;

use hub::SceneHub;

fn model_err(e: botrail_model::ModelError) -> PyErr {
    PyValueError::new_err(e.to_string())
}

/// A parsed robot model (kinematic tree + geometry). Immutable.
#[pyclass(frozen, module = "botrail._core")]
#[derive(Clone)]
struct Robot {
    inner: Arc<RobotModel>,
}

#[pymethods]
impl Robot {
    /// Loads a robot from a URDF file. Mesh paths are resolved relative to
    /// the file; `package://` URIs are resolved heuristically.
    #[staticmethod]
    fn from_urdf(path: PathBuf) -> PyResult<Self> {
        Ok(Robot {
            inner: Arc::new(RobotModel::from_urdf_file(&path).map_err(model_err)?),
        })
    }

    #[staticmethod]
    fn from_urdf_string(xml: &str) -> PyResult<Self> {
        Ok(Robot {
            inner: Arc::new(RobotModel::from_urdf_str(xml).map_err(model_err)?),
        })
    }

    /// Expands a Xacro file (no ROS required) and loads the resulting URDF.
    #[staticmethod]
    fn from_xacro(path: PathBuf) -> PyResult<Self> {
        Ok(Robot {
            inner: Arc::new(RobotModel::from_xacro_file(&path).map_err(model_err)?),
        })
    }

    /// Imports a robot from a USD articulation (UsdPhysics joints and rigid
    /// bodies, e.g. Isaac Sim assets). Link/joint names are the prim paths;
    /// revolute limits are converted from degrees, distances from the
    /// stage's `metersPerUnit`, and Y-up stages are re-modeled as Z-up.
    /// `articulation_root` defaults to the first prim carrying
    /// `PhysicsArticulationRootAPI`; `search_paths` resolve external
    /// (`omniverse://`) references against local directories.
    #[staticmethod]
    #[pyo3(signature = (path, articulation_root = None, search_paths = None))]
    fn from_usd(
        path: PathBuf,
        articulation_root: Option<String>,
        search_paths: Option<Vec<PathBuf>>,
    ) -> PyResult<Self> {
        let imported = botrail_usd::import_robot(
            &path,
            &botrail_usd::RobotImportOptions {
                search_paths: search_paths.unwrap_or_default(),
                articulation_root,
                ..Default::default()
            },
        )
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
        for warning in &imported.warnings {
            eprintln!("botrail: usd robot import: {warning}");
        }
        Ok(Robot {
            inner: Arc::new(imported.model),
        })
    }

    /// Loads a robot or tool from the botrail model catalog (the Hugging
    /// Face dataset `botrail/botrail-catalog`) by id — exact
    /// (`robotiq/2f/2f-85/r1`) or any unambiguous shorthand (`2f-85`,
    /// `robotiq/2f-85`). Needs the optional dependency `huggingface_hub`
    /// (`pip install botrail[catalog]`).
    ///
    /// `revision` pins a dataset commit SHA; without it the newest catalog
    /// is fetched and the resolved SHA is recorded in the robot's source,
    /// so saved projects and generated scripts replay bit-identically.
    /// Downloads land in the standard Hugging Face cache. `format` forces
    /// `"urdf"` or `"usd"`; by default the URDF is preferred. A TCP the
    /// package manifest declares (`frames.tcp_default`) becomes `tcp_link`.
    /// Packages distributed as `recipe_only`/`metadata_only` raise with a
    /// pointer to building them locally.
    #[staticmethod]
    #[pyo3(signature = (id, revision = None, format = None))]
    fn from_catalog(
        py: Python<'_>,
        id: &str,
        revision: Option<&str>,
        format: Option<&str>,
    ) -> PyResult<Self> {
        Ok(Robot {
            inner: catalog::robot_from_catalog(py, id, revision, format)?,
        })
    }

    #[getter]
    fn name(&self) -> String {
        self.inner.name.clone()
    }

    #[getter]
    fn dof(&self) -> usize {
        self.inner.dof()
    }

    /// Actuated joint names in `q`-vector order.
    #[getter]
    fn joint_names(&self) -> Vec<String> {
        self.inner
            .actuated_joint_names()
            .into_iter()
            .map(str::to_string)
            .collect()
    }

    /// Per actuated joint: `(lower, upper)` or `None` for continuous joints.
    #[getter]
    fn joint_limits(&self) -> Vec<Option<(f64, f64)>> {
        self.inner.actuated_joint_limits()
    }

    /// Joints that follow another joint (URDF `<mimic>`, USD
    /// `PhysxMimicJointAPI`) instead of carrying a DOF of their own:
    /// `{joint: (source joint, multiplier, offset)}`. Their value is
    /// `multiplier * <source position> + offset`, and they never appear in
    /// `joint_names` or in a joint position vector.
    #[getter]
    fn mimic_joints(&self) -> BTreeMap<String, (String, f64, f64)> {
        self.inner
            .mimic_joints()
            .into_iter()
            .map(|ji| {
                let joint = &self.inner.joints[ji];
                let mimic = joint.mimic.expect("listed as a mimic joint");
                (
                    joint.name.clone(),
                    (
                        self.inner.joints[mimic.source_joint].name.clone(),
                        mimic.multiplier,
                        mimic.offset,
                    ),
                )
            })
            .collect()
    }

    /// Value of every joint at `positions`, keyed by joint name: the DOF
    /// value for actuated joints, the mimic relation for driven ones, and
    /// 0 for fixed joints.
    fn joint_values(&self, positions: Vec<f64>) -> PyResult<BTreeMap<String, f64>> {
        if positions.len() != self.inner.dof() {
            return Err(PyValueError::new_err(format!(
                "expected {} joint positions, got {}",
                self.inner.dof(),
                positions.len()
            )));
        }
        Ok(self
            .inner
            .joints
            .iter()
            .zip(self.inner.joint_values(&positions))
            .map(|(joint, value)| (joint.name.clone(), value))
            .collect())
    }

    #[getter]
    fn link_names(&self) -> Vec<String> {
        self.inner.links.iter().map(|l| l.name.clone()).collect()
    }

    /// Default end-effector link name: the TCP declared by a tool
    /// attachment or catalog manifest when present, otherwise the deepest
    /// leaf in the kinematic tree.
    #[getter]
    fn tcp_link(&self) -> String {
        self.inner.links[self.inner.default_tcp_link()].name.clone()
    }

    /// Declared tool-mounting face (catalog `frames.flange_frame`; after
    /// `attach_tool`, the mounted tool's onward flange if it has one).
    /// `attach_tool` uses it when `flange` is omitted.
    #[getter]
    fn flange_link(&self) -> Option<String> {
        self.inner
            .flange_link
            .map(|i| self.inner.links[i].name.clone())
    }

    /// Declared mounting face when this model is a tool (catalog
    /// `frames.mount_frame`). `attach_tool` uses it when `mount` is
    /// omitted, falling back to the tool's root link.
    #[getter]
    fn mount_link(&self) -> Option<String> {
        self.inner
            .mount_link
            .map(|i| self.inner.links[i].name.clone())
    }

    /// Solves inverse kinematics. With `quaternion=None` only the position
    /// is matched. `link` defaults to the TCP link, `seed` to the neutral
    /// configuration. When the seeded solve does not converge, up to
    /// `restarts` further attempts run from deterministically generated
    /// seeds (limits midpoint first, then fixed-seed uniform samples within
    /// the limits) — the same call always returns the same answer. Pass
    /// `restarts=0` to solve strictly from the given seed. Always returns
    /// the best configuration found; check `result.converged`.
    #[pyo3(signature = (position, quaternion = None, link = None, seed = None, max_iters = 100, restarts = None))]
    fn ik(
        &self,
        position: [f64; 3],
        quaternion: Option<[f64; 4]>,
        link: Option<&str>,
        seed: Option<Vec<f64>>,
        max_iters: usize,
        restarts: Option<usize>,
    ) -> PyResult<IkResult> {
        let (target, mode) = ik_target(position, quaternion);
        let link_index = resolve_link(&self.inner, link)?;
        let seed = seed.unwrap_or_else(|| self.inner.neutral_positions());
        let defaults = botrail_kin::IkOptions::default();
        let options = botrail_kin::IkOptions {
            mode,
            max_iters,
            restarts: restarts.unwrap_or(defaults.restarts),
            ..defaults
        };
        let result = botrail_kin::solve_ik(&self.inner, link_index, &target, &seed, &options)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(IkResult { inner: result })
    }

    /// Welds a tool (end-effector) onto this robot's flange and returns the
    /// composite robot; neither input is modified. The DOF vector becomes
    /// this robot's joints followed by the tool's, mimic joints included.
    ///
    /// `flange` defaults to the robot's declared `flange_link` and `mount`
    /// to the tool's declared `mount_link` (catalog manifests declare
    /// both), falling back to the tool's root — so catalog parts attach
    /// with no arguments, and a coupling's outward face becomes the
    /// composite's flange for the next `attach_tool` in the stack.
    /// `offset` places the mount relative to the flange (e.g. a coupling's
    /// thickness); the mount must resolve to the tool's root link. `tcp`
    /// names a tool link to become the composite's `tcp_link` — otherwise a
    /// TCP declared on the tool carries over, falling back to the
    /// deepest-leaf heuristic. When both models share a link/joint name,
    /// pass `prefix` to namespace the tool's names.
    ///
    /// ```python
    /// robot = ur5e.attach_tool(coupling).attach_tool(gripper)  # catalog parts
    /// robot = ur5e.attach_tool(
    ///     gripper, flange="flange", mount="robotiq_arg2f_base_link",
    ///     offset_position=(0, 0, 0.0139), tcp="tcp",
    /// )
    /// ```
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (tool, flange = None, mount = None, offset_position = None, offset_quaternion = None, tcp = None, prefix = None))]
    fn attach_tool(
        &self,
        tool: &Robot,
        flange: Option<&str>,
        mount: Option<&str>,
        offset_position: Option<[f64; 3]>,
        offset_quaternion: Option<[f64; 4]>,
        tcp: Option<&str>,
        prefix: Option<&str>,
    ) -> PyResult<Robot> {
        let offset = pose_from(offset_position.unwrap_or([0.0; 3]), offset_quaternion);
        Ok(Robot {
            inner: Arc::new(
                self.inner
                    .attach_tool(&tool.inner, flange, mount, offset, tcp, prefix)
                    .map_err(model_err)?,
            ),
        })
    }

    fn __repr__(&self) -> String {
        format!(
            "Robot(name='{}', dof={}, links={})",
            self.inner.name,
            self.inner.dof(),
            self.inner.links.len()
        )
    }
}

fn resolve_link(model: &RobotModel, link: Option<&str>) -> PyResult<usize> {
    match link {
        Some(name) => model
            .link_index(name)
            .ok_or_else(|| PyValueError::new_err(format!("unknown link `{name}`"))),
        None => Ok(model.default_tcp_link()),
    }
}

fn pose_from(position: [f64; 3], quaternion: Option<[f64; 4]>) -> nalgebra::Isometry3<f64> {
    (&botrail_scene::wire::PoseMsg {
        position,
        quaternion: quaternion.unwrap_or([0.0, 0.0, 0.0, 1.0]),
    })
        .into()
}

/// An obstacle pose in a scenario dict: a bare position (upright) or a
/// `(position, quaternion)` pair.
#[derive(FromPyObject)]
enum ScenarioPose {
    Pair(([f64; 3], [f64; 4])),
    Position([f64; 3]),
}

impl ScenarioPose {
    fn into_iso(self) -> nalgebra::Isometry3<f64> {
        match self {
            ScenarioPose::Pair((position, quaternion)) => pose_from(position, Some(quaternion)),
            ScenarioPose::Position(position) => pose_from(position, None),
        }
    }
}

fn sensor_watch(
    watch: Option<Vec<String>>,
    watch_robot: bool,
    watch_robots: Option<Vec<String>>,
) -> botrail_scene::seq::SensorWatch {
    use botrail_scene::seq::SensorWatch;
    if let Some(robots) = watch_robots {
        // Named robots combined with objects or "any robot" has no
        // dedicated variant; watch everything (the superset) rather than
        // silently dropping one.
        return match (watch.as_deref(), watch_robot) {
            (None | Some([]), false) => SensorWatch::Robots(robots),
            _ => SensorWatch::All,
        };
    }
    match (watch, watch_robot) {
        (None, false) => SensorWatch::AllObjects,
        (None, true) => SensorWatch::All,
        (Some(names), false) => SensorWatch::Objects(names),
        (Some(names), true) if names.is_empty() => SensorWatch::Robot,
        // Named objects plus the robot has no dedicated variant; watch
        // everything (the superset) rather than silently dropping one.
        (Some(_), true) => SensorWatch::All,
    }
}

/// What the studio uses for an obstacle with no authored material. Filling
/// one knob leaves the other here rather than at zero, so naming a
/// metalness does not silently turn a surface into a mirror.
const DEFAULT_METALNESS: f32 = 0.05;
const DEFAULT_ROUGHNESS: f32 = 0.80;

fn scene_err(e: botrail_scene::SceneError) -> PyErr {
    PyValueError::new_err(e.to_string())
}

fn ik_target(
    position: [f64; 3],
    quaternion: Option<[f64; 4]>,
) -> (nalgebra::Isometry3<f64>, botrail_kin::IkMode) {
    let pose = botrail_scene::wire::PoseMsg {
        position,
        quaternion: quaternion.unwrap_or([0.0, 0.0, 0.0, 1.0]),
    };
    let mode = match quaternion {
        Some(_) => botrail_kin::IkMode::Pose,
        None => botrail_kin::IkMode::Position,
    };
    ((&pose).into(), mode)
}

/// Result of an IK solve.
#[pyclass(frozen, module = "botrail._core")]
struct IkResult {
    inner: botrail_kin::IkResult,
}

#[pymethods]
impl IkResult {
    /// Best joint configuration found (always within limits).
    #[getter]
    fn q(&self) -> Vec<f64> {
        self.inner.q.clone()
    }

    #[getter]
    fn converged(&self) -> bool {
        self.inner.converged
    }

    /// Remaining position error (m).
    #[getter]
    fn pos_error(&self) -> f64 {
        self.inner.pos_error
    }

    /// Remaining orientation error (rad).
    #[getter]
    fn rot_error(&self) -> f64 {
        self.inner.rot_error
    }

    #[getter]
    fn iters(&self) -> usize {
        self.inner.iters
    }

    fn __repr__(&self) -> String {
        format!(
            "IkResult(converged={}, pos_error={:.2e}, rot_error={:.2e}, iters={})",
            self.inner.converged, self.inner.pos_error, self.inner.rot_error, self.inner.iters
        )
    }
}

/// The cell: one or more robots in a workspace, with the obstacles,
/// frames, sensors, devices, motions, and sequences around them. Shared
/// with the studio server: state changes made here are pushed to
/// connected browsers immediately.
#[pyclass(frozen, module = "botrail._core")]
struct Scene {
    hub: Arc<SceneHub>,
    robot: Robot,
}

impl Scene {
    /// Resolves an optional `robot=` argument to a robot index. `None`
    /// means the sole robot and is an error when the scene has several.
    fn resolve_robot(&self, robot: Option<&str>) -> PyResult<usize> {
        self.hub.robot_index(robot).map_err(PyValueError::new_err)
    }

    /// Applies an `add_*` call's optional `color=`, passing the obstacle's
    /// final name through. A `None` colour is left alone so the obstacle
    /// keeps the viewer's neutral shading.
    fn paint(&self, name: String, color: Option<[f32; 3]>) -> PyResult<String> {
        if color.is_some() {
            self.hub
                .set_obstacle_color(&name, color)
                .map_err(scene_err)?;
        }
        Ok(name)
    }
}

#[pymethods]
impl Scene {
    /// A scene with the robot root placed at the world-frame base pose
    /// (identity when omitted). `name` sets the robot's scene-unique
    /// instance name (default: the model name).
    #[new]
    #[pyo3(signature = (robot, base_position = None, base_quaternion = None, name = None))]
    fn new(
        robot: &Robot,
        base_position: Option<[f64; 3]>,
        base_quaternion: Option<[f64; 4]>,
        name: Option<&str>,
    ) -> Self {
        let base = pose_from(base_position.unwrap_or([0.0; 3]), base_quaternion);
        let mut scene = botrail_scene::Scene::with_base(robot.inner.clone(), base);
        if let Some(name) = name {
            scene.rename_robot(0, name);
        }
        Scene {
            hub: Arc::new(SceneHub::new(scene)),
            robot: robot.clone(),
        }
    }

    /// Adds another robot instance and returns its (possibly uniquified)
    /// scene-unique instance name. `name` defaults to the model name.
    /// Connected studios pick the new robot up immediately (the handshake
    /// is re-broadcast).
    #[pyo3(signature = (robot, name = None, base_position = None, base_quaternion = None))]
    fn add_robot(
        &self,
        robot: &Robot,
        name: Option<&str>,
        base_position: Option<[f64; 3]>,
        base_quaternion: Option<[f64; 4]>,
    ) -> String {
        let base = pose_from(base_position.unwrap_or([0.0; 3]), base_quaternion);
        self.hub.add_robot(robot.inner.clone(), name, base)
    }

    /// Puts a robot on a vehicle: from here its base is derived from that
    /// vehicle's frame, `offset` away, and re-derived every scan tick — an
    /// arm and a chassis become an AMR. Planned motions cannot start while
    /// the vehicle is driving (a plan is baked in world coordinates); ramps
    /// can, which is how an arm stows itself on the move.
    #[pyo3(signature = (device, offset_position = None, offset_quaternion = None, robot = None))]
    fn mount_robot(
        &self,
        device: &str,
        offset_position: Option<[f64; 3]>,
        offset_quaternion: Option<[f64; 4]>,
        robot: Option<&str>,
    ) -> PyResult<()> {
        let index = self.resolve_robot(robot)?;
        let offset = pose_from(offset_position.unwrap_or([0.0; 3]), offset_quaternion);
        self.hub
            .mount_robot(index, device, offset)
            .map_err(scene_err)
    }

    #[getter]
    fn robot(&self) -> Robot {
        self.robot.clone()
    }

    /// Instance names of every robot in the scene, in insertion order.
    #[getter]
    fn robots(&self) -> Vec<String> {
        self.hub.robot_names()
    }

    /// The model of the robot instance named `name`.
    fn robot_of(&self, name: &str) -> PyResult<Robot> {
        let index = self.resolve_robot(Some(name))?;
        Ok(Robot {
            inner: self.hub.robot_model(index),
        })
    }

    /// World pose of the robot root as `(position, quaternion_xyzw)`.
    #[getter]
    fn robot_base_pose(&self) -> ([f64; 3], [f64; 4]) {
        self.hub.robot_base_pose()
    }

    /// World base pose of the robot instance named `name`.
    fn robot_base_pose_of(&self, name: &str) -> PyResult<([f64; 3], [f64; 4])> {
        let index = self.resolve_robot(Some(name))?;
        let pose = self.hub.robot_base_isometry_for(index);
        let t = pose.translation;
        let q = pose.rotation.coords;
        Ok(([t.x, t.y, t.z], [q.x, q.y, q.z, q.w]))
    }

    /// Places the robot root at the world-frame pose and pushes the new
    /// state to connected studio clients.
    #[pyo3(signature = (position, quaternion = None, robot = None))]
    fn set_robot_base_pose(
        &self,
        position: [f64; 3],
        quaternion: Option<[f64; 4]>,
        robot: Option<&str>,
    ) -> PyResult<()> {
        let index = self.resolve_robot(robot)?;
        self.hub
            .set_robot_base_pose_for(index, pose_from(position, quaternion));
        Ok(())
    }

    #[getter]
    fn joint_positions(&self) -> Vec<f64> {
        self.hub.joint_positions()
    }

    /// Joint configuration of the robot instance named `name`.
    fn joint_positions_of(&self, name: &str) -> PyResult<Vec<f64>> {
        let index = self.resolve_robot(Some(name))?;
        Ok(self.hub.joint_positions_for(index))
    }

    #[pyo3(signature = (positions, robot = None))]
    fn set_joint_positions(&self, positions: Vec<f64>, robot: Option<&str>) -> PyResult<()> {
        let index = self.resolve_robot(robot)?;
        self.hub
            .set_joint_positions_for(index, positions)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    /// World pose of a link as `(position, quaternion_xyzw)`.
    #[pyo3(signature = (link_name, robot = None))]
    fn link_pose(&self, link_name: &str, robot: Option<&str>) -> PyResult<([f64; 3], [f64; 4])> {
        let index = self.resolve_robot(robot)?;
        self.hub
            .link_pose_for(index, link_name)
            .ok_or_else(|| PyValueError::new_err(format!("unknown link `{link_name}`")))
    }

    /// Adds a box obstacle (full extents, meters). Returns the final name,
    /// which may be uniquified. Changes are pushed to connected studios.
    #[pyo3(signature = (name, size, position, quaternion = None, color = None))]
    fn add_box(
        &self,
        name: &str,
        size: [f64; 3],
        position: [f64; 3],
        quaternion: Option<[f64; 4]>,
        color: Option<[f32; 3]>,
    ) -> PyResult<String> {
        let name = self
            .hub
            .add_obstacle(
                name,
                botrail_model::Geometry::Box {
                    size: nalgebra::Vector3::new(size[0], size[1], size[2]),
                },
                pose_from(position, quaternion),
            )
            .map_err(scene_err)?;
        self.paint(name, color)
    }

    #[pyo3(signature = (name, radius, position, quaternion = None, color = None))]
    fn add_sphere(
        &self,
        name: &str,
        radius: f64,
        position: [f64; 3],
        quaternion: Option<[f64; 4]>,
        color: Option<[f32; 3]>,
    ) -> PyResult<String> {
        let name = self
            .hub
            .add_obstacle(
                name,
                botrail_model::Geometry::Sphere { radius },
                pose_from(position, quaternion),
            )
            .map_err(scene_err)?;
        self.paint(name, color)
    }

    /// Adds a cylinder obstacle (URDF convention: axis along local +z).
    #[pyo3(signature = (name, radius, length, position, quaternion = None, color = None))]
    fn add_cylinder(
        &self,
        name: &str,
        radius: f64,
        length: f64,
        position: [f64; 3],
        quaternion: Option<[f64; 4]>,
        color: Option<[f32; 3]>,
    ) -> PyResult<String> {
        let name = self
            .hub
            .add_obstacle(
                name,
                botrail_model::Geometry::Cylinder { radius, length },
                pose_from(position, quaternion),
            )
            .map_err(scene_err)?;
        self.paint(name, color)
    }

    /// Adds a mesh obstacle from an STL/OBJ file. The collision shape is a
    /// VHACD convex decomposition (computed on first load, then cached on
    /// disk); the studio renders the original mesh.
    #[pyo3(signature = (name, path, position, scale = None, quaternion = None, color = None))]
    fn add_mesh(
        &self,
        name: &str,
        path: PathBuf,
        position: [f64; 3],
        scale: Option<[f64; 3]>,
        quaternion: Option<[f64; 4]>,
        color: Option<[f32; 3]>,
    ) -> PyResult<String> {
        let s = scale.unwrap_or([1.0; 3]);
        let name = self
            .hub
            .add_obstacle(
                name,
                botrail_model::Geometry::Mesh {
                    path,
                    scale: nalgebra::Vector3::new(s[0], s[1], s[2]),
                },
                pose_from(position, quaternion),
            )
            .map_err(scene_err)?;
        self.paint(name, color)
    }

    /// Imports the static geometry of a USD stage (usda/usdc/usdz —
    /// references, variants, and instancing are composed) as obstacles,
    /// normalized to meters / Z-up. Leaf Xform/Scope prims become named
    /// frames (see `frame()`), usable as robot mount points. Obstacle and
    /// frame names are the prim paths, optionally prefixed. Returns the
    /// added obstacle names.
    #[pyo3(signature = (path, prefix = None, search_paths = None))]
    fn load_usd(
        &self,
        path: PathBuf,
        prefix: Option<String>,
        search_paths: Option<Vec<PathBuf>>,
    ) -> PyResult<Vec<String>> {
        let options = botrail_usd::ImportOptions {
            search_paths: search_paths.unwrap_or_default(),
            ..Default::default()
        };
        let imported = botrail_usd::import_usd(&path, &options)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        for warning in &imported.warnings {
            eprintln!("botrail: usd import: {warning}");
        }
        for root in &imported.robot_roots {
            eprintln!(
                "botrail: usd import: skipped robot articulation at `{root}` — import it with \
                 bt.Robot.from_usd(path, articulation_root={root:?})"
            );
        }
        let prefix = prefix.unwrap_or_default();
        let batch = imported
            .nodes
            .into_iter()
            .map(|n| botrail_scene::ObstacleSpec {
                name: format!("{prefix}{}", n.name),
                geometry: n.geometry,
                pose: n.pose,
                color: n.color,
                // USD import reads displayColor but not shading: a stage
                // that binds real materials is rendered from the stage
                // itself, not from these proxies.
                material: None,
            })
            .collect();
        let names = self.hub.add_obstacles(batch).map_err(scene_err)?;
        self.hub.add_frames(
            imported
                .frames
                .into_iter()
                .map(|f| (format!("{prefix}{}", f.name), f.pose))
                .collect(),
        );
        Ok(names)
    }

    /// Registers (or updates) a named world frame.
    #[pyo3(signature = (name, position, quaternion = None))]
    fn add_frame(&self, name: &str, position: [f64; 3], quaternion: Option<[f64; 4]>) {
        self.hub.add_frame(name, pose_from(position, quaternion));
    }

    /// All named frames as `{name: (position, quaternion_xyzw)}`.
    #[getter]
    fn frames(&self) -> std::collections::HashMap<String, ([f64; 3], [f64; 4])> {
        self.hub.frames().into_iter().collect()
    }

    /// Pose of a named frame as `(position, quaternion_xyzw)` — e.g.
    /// `scene.set_robot_base_pose(*scene.frame("/World/mount"))`.
    fn frame(&self, name: &str) -> PyResult<([f64; 3], [f64; 4])> {
        self.hub
            .frame_pose(name)
            .ok_or_else(|| PyValueError::new_err(format!("unknown frame `{name}`")))
    }

    fn remove_obstacle(&self, name: &str) -> PyResult<()> {
        self.hub.remove_obstacle(name).map_err(scene_err)
    }

    /// Includes/excludes an obstacle from collision checking (it keeps
    /// rendering in the studio either way).
    fn set_obstacle_enabled(&self, name: &str, enabled: bool) -> PyResult<()> {
        self.hub
            .set_obstacle_enabled(name, enabled)
            .map_err(scene_err)
    }

    /// Sets an obstacle's display colour, linear RGB in 0..1. `None` hands
    /// the shading back to the viewer. Display only — collision and planning
    /// see the same geometry either way.
    #[pyo3(signature = (name, color))]
    fn set_obstacle_color(&self, name: &str, color: Option<[f32; 3]>) -> PyResult<()> {
        self.hub.set_obstacle_color(name, color).map_err(scene_err)
    }

    /// Hides or shows an obstacle without touching whether it collides.
    /// A hidden obstacle is still a real obstacle: this is how a workpiece
    /// carries a display mesh and its convex collision pieces at once.
    fn set_obstacle_visible(&self, name: &str, visible: bool) -> PyResult<()> {
        self.hub
            .set_obstacle_visible(name, visible)
            .map_err(scene_err)
    }

    /// Sets how an obstacle's surface takes light. Passing neither knob
    /// clears the material, handing the choice back to the viewer.
    #[pyo3(signature = (name, metalness = None, roughness = None))]
    fn set_obstacle_material(
        &self,
        name: &str,
        metalness: Option<f32>,
        roughness: Option<f32>,
    ) -> PyResult<()> {
        let material = match (metalness, roughness) {
            (None, None) => None,
            // One knob given is still an authored material; the other takes
            // the studio's own default rather than silently going to zero.
            (m, r) => Some(botrail_scene::Material::new(
                m.unwrap_or(DEFAULT_METALNESS),
                r.unwrap_or(DEFAULT_ROUGHNESS),
            )),
        };
        self.hub
            .set_obstacle_material(name, material)
            .map_err(scene_err)
    }

    /// `(metalness, roughness)`, or `None` when the obstacle has no
    /// authored material.
    fn obstacle_material(&self, name: &str) -> PyResult<Option<(f32, f32)>> {
        self.hub.obstacle_material(name).map_err(scene_err)
    }

    #[pyo3(signature = (name, position, quaternion = None))]
    fn set_obstacle_pose(
        &self,
        name: &str,
        position: [f64; 3],
        quaternion: Option<[f64; 4]>,
    ) -> PyResult<()> {
        self.hub
            .set_obstacle_pose(name, pose_from(position, quaternion))
            .map_err(scene_err)
    }

    #[getter]
    fn obstacle_names(&self) -> Vec<String> {
        self.hub.obstacle_names()
    }

    /// World pose of an obstacle as `(position, quaternion_xyzw)`.
    fn obstacle_pose(&self, name: &str) -> PyResult<([f64; 3], [f64; 4])> {
        self.hub
            .obstacle_pose(name)
            .ok_or_else(|| PyValueError::new_err(format!("unknown obstacle `{name}`")))
    }

    /// World-frame axis-aligned bounds of an obstacle, as `(min, max)`.
    /// A cell that has to sit a workpiece on a pallet asks the geometry
    /// where its underside is instead of hard-coding a measured number
    /// that quietly stops matching when the mesh is rebuilt.
    fn obstacle_bounds(&self, name: &str) -> PyResult<([f64; 3], [f64; 3])> {
        self.hub
            .obstacle_bounds(name)
            .ok_or_else(|| PyValueError::new_err(format!("unknown obstacle `{name}`")))
    }

    /// Renames a robot instance, returning the name it actually got (a
    /// name already taken is uniquified). Sequence actions, `robot_done`
    /// conditions and a zone sensor's watch list follow the robot, so a
    /// cell can be renamed after it has been authored.
    #[pyo3(signature = (robot, name))]
    fn rename_robot(&self, robot: &str, name: &str) -> PyResult<String> {
        let index = self.resolve_robot(Some(robot))?;
        Ok(self.hub.rename_robot(index, name))
    }

    /// Excuses one link pair of two *different* robots from collision
    /// checking — the escape hatch for arms that share a mount plate or are
    /// meant to touch. Unlike a robot's own self-collision matrix, which is
    /// generated by sampling, inter-robot pairs are never inferred: whether
    /// two arms may touch depends on where their bases stand, so it is the
    /// author's call.
    #[pyo3(signature = (robot_a, link_a, robot_b, link_b))]
    fn allow_inter_robot_collision(
        &self,
        robot_a: &str,
        link_a: &str,
        robot_b: &str,
        link_b: &str,
    ) -> PyResult<()> {
        self.hub
            .allow_inter_robot_collision(robot_a, link_a, robot_b, link_b)
            .map_err(PyValueError::new_err)
    }

    /// An obstacle's display colour as linear RGB, or `None` when it has
    /// none and the viewer picks the shading.
    fn obstacle_color(&self, name: &str) -> PyResult<Option<[f32; 3]>> {
        self.hub
            .obstacle_color(name)
            .ok_or_else(|| PyValueError::new_err(format!("unknown obstacle `{name}`")))
    }

    /// Attaches an obstacle to a robot link at its current relative pose —
    /// a grasp. While attached the object follows the link (live, in
    /// planning, and in playback) and collides as part of the robot.
    /// `link=None` uses the TCP link; `touch_links=None` allows contact
    /// with the link's subtree (the gripper).
    #[pyo3(signature = (name, link = None, touch_links = None, robot = None))]
    fn attach(
        &self,
        name: &str,
        link: Option<&str>,
        touch_links: Option<Vec<String>>,
        robot: Option<&str>,
    ) -> PyResult<()> {
        let index = self.resolve_robot(robot)?;
        self.hub
            .attach_obstacle_to(index, name, link, touch_links.as_deref())
            .map_err(scene_err)
    }

    /// Detaches an obstacle; its pose freezes where the robot holds it.
    fn detach(&self, name: &str) -> PyResult<()> {
        self.hub.detach_obstacle(name).map_err(scene_err)
    }

    /// Attached obstacles as `(object, link)` name pairs.
    #[getter]
    fn attachments(&self) -> Vec<(String, String)> {
        self.hub.attachments()
    }

    /// Bakes a trajectory to a USD animation layer that plays in usdview /
    /// Omniverse / Blender: robot link motion as timeSamples, obstacles as
    /// prims, grasped objects riding along. The extension picks the
    /// serialization — `.usda` text, `.usdc`/`.usd` binary crate (about half
    /// the size). USD-sourced robots reference their original stage (assets
    /// copied to a sibling `<stem>_assets/` directory); URDF robots are
    /// authored from the model's visuals. `robot` names the instance the
    /// trajectory belongs to (required when the scene has several). Returns
    /// exporter warnings.
    #[pyo3(signature = (trajectory, path, fps = 60.0, robot = None))]
    fn export_usd(
        &self,
        trajectory: &Trajectory,
        path: PathBuf,
        fps: f64,
        robot: Option<&str>,
    ) -> PyResult<Vec<String>> {
        let index = self.resolve_robot(robot)?;
        self.hub
            .export_trajectory_usd(index, &trajectory.inner, &path, fps)
            .map_err(PyValueError::new_err)
    }

    /// Plays a baked USD recording (an Isaac Sim capture or a botrail
    /// export) on the scene's robots and broadcasts it to the studio.
    /// Joint playback is used when the layer carries `JointStateAPI`
    /// samples for every actuated joint; otherwise the recorded body
    /// transforms are replayed directly (`force_transforms` forces the
    /// latter). With several robots each is located at
    /// `/World/<sanitized instance name>` (the export convention);
    /// `robot_roots` maps instance names to prim paths when the recording
    /// placed them elsewhere. Returns `{"mode", "duration", "warnings"}`.
    #[pyo3(signature = (path, force_transforms = false, robot_roots = None))]
    fn play_usd_animation<'py>(
        &self,
        py: Python<'py>,
        path: PathBuf,
        force_transforms: bool,
        robot_roots: Option<std::collections::HashMap<String, String>>,
    ) -> PyResult<Bound<'py, PyDict>> {
        let (mode, duration, warnings, object_tracks) = self
            .hub
            .play_usd_animation(
                &path,
                force_transforms,
                robot_roots
                    .map(|m| m.into_iter().collect())
                    .unwrap_or_default(),
            )
            .map_err(PyValueError::new_err)?;
        let out = PyDict::new(py);
        out.set_item("mode", mode)?;
        out.set_item("duration", duration)?;
        out.set_item("warnings", warnings)?;
        out.set_item("object_tracks", object_tracks)?;
        Ok(out)
    }

    // ------------------------------------------------------------ sequences

    /// Declares (or re-initializes) an internal signal — a PLC internal
    /// relay written by `bt.seq.set_signal` actions and read by
    /// `bt.seq.signal` transitions.
    #[pyo3(signature = (name, initial = false))]
    fn define_signal(&self, name: &str, initial: bool) {
        self.hub.define_signal(name, initial);
    }

    fn remove_signal(&self, name: &str) -> PyResult<()> {
        self.hub.remove_signal(name).map_err(scene_err)
    }

    /// Binds a weld flash to a signal at a robot's TCP: while the signal
    /// is true during playback, the studio draws an arc flash there and
    /// the USD export blinks an emissive prim. Pure presentation, driven
    /// by the same baked signal a weld controller's "current on" output
    /// would be — declare the signal first (`define_signal`), author it
    /// from the sequence that owns the weld.
    #[pyo3(signature = (name, signal, robot))]
    fn add_weld_flash(&self, name: &str, signal: &str, robot: &str) -> PyResult<()> {
        self.hub
            .add_weld_flash(name, signal, robot)
            .map_err(scene_err)
    }

    /// Binds an accumulating cut trace to a signal at a robot's TCP:
    /// while the signal is true during playback, the studio draws the
    /// TCP's trail (the cut so far) and spins `spin_link` if given. Pure
    /// presentation, like `add_weld_flash`; in USD the toolpath curves
    /// already carry the picture.
    #[pyo3(signature = (name, signal, robot, spin_link = None))]
    fn add_cut_trace(
        &self,
        name: &str,
        signal: &str,
        robot: &str,
        spin_link: Option<&str>,
    ) -> PyResult<()> {
        self.hub
            .add_cut_trace(name, signal, robot, spin_link)
            .map_err(scene_err)
    }

    /// Declared internal signals as `(name, initial)` pairs.
    #[getter]
    fn signals(&self) -> Vec<(String, bool)> {
        self.hub.signals()
    }

    /// Adds a box-shaped presence sensor: its name becomes a read-only
    /// input signal, ON while a watched body overlaps the zone. `watch` is
    /// a list of obstacle names (default: every obstacle); pass
    /// `watch_robot=True` to sense robot links too (with `watch=[]` for a
    /// robot-only light curtain).
    #[pyo3(signature = (name, position, size, quaternion = None, watch = None, watch_robot = false, watch_robots = None, mount = None))]
    #[allow(clippy::too_many_arguments)]
    fn add_zone_sensor(
        &self,
        name: &str,
        position: [f64; 3],
        size: [f64; 3],
        quaternion: Option<[f64; 4]>,
        watch: Option<Vec<String>>,
        watch_robot: bool,
        watch_robots: Option<Vec<String>>,
        mount: Option<String>,
    ) -> PyResult<()> {
        self.hub.upsert_sensor(botrail_scene::seq::Sensor {
            name: name.to_string(),
            kind: botrail_scene::seq::SensorKind::Zone {
                pose: pose_from(position, quaternion),
                size: nalgebra::Vector3::new(size[0], size[1], size[2]),
            },
            watch: sensor_watch(watch, watch_robot, watch_robots),
            mount,
        });
        Ok(())
    }

    /// Adds a photoelectric beam sensor between two world points, ON while
    /// the beam is interrupted. Watch semantics as in `add_zone_sensor`.
    #[pyo3(signature = (name, frm, to, radius = 0.005, watch = None, watch_robot = false, watch_robots = None, mount = None))]
    #[allow(clippy::too_many_arguments)]
    fn add_beam_sensor(
        &self,
        name: &str,
        frm: [f64; 3],
        to: [f64; 3],
        radius: f64,
        watch: Option<Vec<String>>,
        watch_robot: bool,
        watch_robots: Option<Vec<String>>,
        mount: Option<String>,
    ) -> PyResult<()> {
        self.hub.upsert_sensor(botrail_scene::seq::Sensor {
            name: name.to_string(),
            kind: botrail_scene::seq::SensorKind::Beam {
                from: nalgebra::Point3::new(frm[0], frm[1], frm[2]),
                to: nalgebra::Point3::new(to[0], to[1], to[2]),
                radius,
            },
            watch: sensor_watch(watch, watch_robot, watch_robots),
            mount,
        });
        Ok(())
    }

    fn remove_sensor(&self, name: &str) -> PyResult<()> {
        self.hub.remove_sensor(name).map_err(scene_err)
    }

    #[getter]
    fn sensor_names(&self) -> Vec<String> {
        self.hub.sensor_names()
    }

    /// Adds a conveyor: while running, any unattached obstacle whose origin
    /// lies inside the zone box is carried at `velocity` (m/s). Start/stop
    /// it from sequences with `bt.seq.start`/`bt.seq.stop`.
    #[pyo3(signature = (name, zone_position, zone_size, velocity, zone_quaternion = None, running = true))]
    fn add_conveyor(
        &self,
        name: &str,
        zone_position: [f64; 3],
        zone_size: [f64; 3],
        velocity: [f64; 3],
        zone_quaternion: Option<[f64; 4]>,
        running: bool,
    ) -> PyResult<()> {
        self.hub.upsert_device(botrail_scene::seq::Device {
            name: name.to_string(),
            kind: botrail_scene::seq::DeviceKind::Conveyor {
                zone_pose: pose_from(zone_position, zone_quaternion),
                zone_size: nalgebra::Vector3::new(zone_size[0], zone_size[1], zone_size[2]),
                velocity: nalgebra::Vector3::new(velocity[0], velocity[1], velocity[2]),
                running,
            },
        });
        Ok(())
    }

    /// Adds a feeder: every `interval` seconds while running it puts the
    /// next waiting member of `pool` at `position`.
    ///
    /// The pool is finite because a baked timeline holds a fixed set of
    /// named object tracks — an endless line is this plus an `add_sink`
    /// that returns carriers to the magazine. Member `i` waits at
    /// `park + pitch * i`, and a member that does not start on its slot
    /// starts out on the line (an already-loaded belt).
    #[pyo3(signature = (name, pool, park, position, pitch = None, interval = 0.0, running = false))]
    #[allow(clippy::too_many_arguments)]
    fn add_source(
        &self,
        name: &str,
        pool: Vec<String>,
        park: [f64; 3],
        position: [f64; 3],
        pitch: Option<[f64; 3]>,
        interval: f64,
        running: bool,
    ) -> PyResult<()> {
        for object in &pool {
            if self.hub.obstacle_pose(object).is_none() {
                return Err(PyValueError::new_err(format!(
                    "unknown obstacle `{object}`"
                )));
            }
        }
        let pitch = pitch.unwrap_or([0.0; 3]);
        self.hub.upsert_device(botrail_scene::seq::Device {
            name: name.to_string(),
            kind: botrail_scene::seq::DeviceKind::Source {
                pool,
                park: pose_from(park, None),
                pitch: nalgebra::Vector3::new(pitch[0], pitch[1], pitch[2]),
                pose: pose_from(position, None),
                interval,
                running,
            },
        });
        Ok(())
    }

    /// Adds the far end of a line: any unattached carrier reaching the zone
    /// goes back to `source`'s magazine, free to be fed again.
    #[pyo3(signature = (name, zone_position, zone_size, source, zone_quaternion = None))]
    fn add_sink(
        &self,
        name: &str,
        zone_position: [f64; 3],
        zone_size: [f64; 3],
        source: &str,
        zone_quaternion: Option<[f64; 4]>,
    ) -> PyResult<()> {
        if !self.hub.device_names().iter().any(|d| d == source) {
            return Err(PyValueError::new_err(format!(
                "unknown source device `{source}`"
            )));
        }
        self.hub.upsert_device(botrail_scene::seq::Device {
            name: name.to_string(),
            kind: botrail_scene::seq::DeviceKind::Sink {
                zone_pose: pose_from(zone_position, zone_quaternion),
                zone_size: nalgebra::Vector3::new(zone_size[0], zone_size[1], zone_size[2]),
                source: source.to_string(),
            },
        });
        Ok(())
    }

    /// Adds a linear axis (door / lifter / indexer) moving the listed
    /// obstacles along `axis` at `speed`, positioned within `range` by
    /// `bt.seq.move_to`; await it with `bt.seq.device_done`.
    #[pyo3(signature = (name, objects, axis, speed, range, position = 0.0))]
    fn add_linear_axis(
        &self,
        name: &str,
        objects: Vec<String>,
        axis: [f64; 3],
        speed: f64,
        range: [f64; 2],
        position: f64,
    ) -> PyResult<()> {
        let axis = nalgebra::Unit::try_new(nalgebra::Vector3::new(axis[0], axis[1], axis[2]), 1e-9)
            .ok_or_else(|| PyValueError::new_err("axis must be a nonzero vector"))?;
        if !(speed.is_finite() && speed > 0.0) {
            return Err(PyValueError::new_err(format!(
                "speed must be positive, got {speed}"
            )));
        }
        self.hub.upsert_device(botrail_scene::seq::Device {
            name: name.to_string(),
            kind: botrail_scene::seq::DeviceKind::LinearAxis {
                objects,
                axis,
                speed,
                position,
                range: (range[0], range[1]),
            },
        });
        Ok(())
    }

    /// Adds a guided transport vehicle (an AGV / AMR as the cell sees it):
    /// it drives station to station along `path` — straight legs at
    /// `speed`, in-place pivot turns at `turn_speed` — carrying the `body`
    /// obstacles rigidly. Dispatch it with `bt.seq.goto(name, station)` and
    /// await arrival with `bt.seq.device_done(name)`. The arrival heading
    /// is the last leg's direction, so the waypoint before a station sets
    /// how the vehicle docks. Body entries name obstacles exactly, or as
    /// subtree prefixes (`"/World/AGV"` takes every obstacle under it).
    #[pyo3(signature = (name, body, path, stations, speed = 0.5,
                        turn_speed = std::f64::consts::FRAC_PI_2,
                        start = None, ring = false, allow_reverse = false,
                        tray_position = None, tray_size = None,
                        tray_quaternion = None))]
    #[allow(clippy::too_many_arguments)]
    fn add_vehicle(
        &self,
        name: &str,
        body: Vec<String>,
        path: Vec<[f64; 2]>,
        stations: std::collections::BTreeMap<String, usize>,
        speed: f64,
        turn_speed: f64,
        start: Option<String>,
        ring: bool,
        allow_reverse: bool,
        tray_position: Option<[f64; 3]>,
        tray_size: Option<[f64; 3]>,
        tray_quaternion: Option<[f64; 4]>,
    ) -> PyResult<()> {
        if path.len() < 2 {
            return Err(PyValueError::new_err(format!(
                "path needs at least 2 waypoints, got {}",
                path.len()
            )));
        }
        if stations.is_empty() {
            return Err(PyValueError::new_err(
                "stations is empty; name at least the stop the vehicle starts at",
            ));
        }
        for (station, index) in &stations {
            if *index >= path.len() {
                return Err(PyValueError::new_err(format!(
                    "station `{station}` points at waypoint {index}, \
                     but the path has {}",
                    path.len()
                )));
            }
        }
        if !(speed.is_finite() && speed > 0.0) {
            return Err(PyValueError::new_err(format!(
                "speed must be positive, got {speed}"
            )));
        }
        if !(turn_speed.is_finite() && turn_speed > 0.0) {
            return Err(PyValueError::new_err(format!(
                "turn_speed must be positive, got {turn_speed}"
            )));
        }
        // The default start is the lowest-index station (deterministic).
        let start = match start {
            Some(s) => {
                if !stations.contains_key(&s) {
                    return Err(PyValueError::new_err(format!(
                        "start `{s}` is not a station (stations: {})",
                        stations
                            .keys()
                            .map(|k| format!("`{k}`"))
                            .collect::<Vec<_>>()
                            .join(", ")
                    )));
                }
                s
            }
            None => stations
                .iter()
                .min_by_key(|(name, index)| (**index, (*name).clone()))
                .map(|(name, _)| name.clone())
                .expect("stations is non-empty"),
        };
        // Body entries: exact obstacle names, or subtree prefixes.
        let known = self.hub.obstacle_names();
        let mut members: Vec<String> = Vec::new();
        for entry in &body {
            if known.iter().any(|n| n == entry) {
                members.push(entry.clone());
                continue;
            }
            let prefix = format!("{}/", entry.trim_end_matches('/'));
            let hits: Vec<String> = known
                .iter()
                .filter(|n| n.starts_with(&prefix))
                .cloned()
                .collect();
            if hits.is_empty() {
                return Err(PyValueError::new_err(format!(
                    "body entry `{entry}` matches no obstacle (exactly or as a prefix)"
                )));
            }
            members.extend(hits);
        }
        let mut seen = std::collections::HashSet::new();
        members.retain(|m| seen.insert(m.clone()));
        // The load deck, given in the vehicle frame: anything resting in it
        // rides along, so a part the arm sets down simply joins the load.
        let tray = match (tray_position, tray_size) {
            (Some(position), Some(size)) => {
                if size.iter().any(|v| !(v.is_finite() && *v > 0.0)) {
                    return Err(PyValueError::new_err(format!(
                        "tray_size must be positive, got {size:?}"
                    )));
                }
                Some((
                    pose_from(position, tray_quaternion),
                    nalgebra::Vector3::new(size[0], size[1], size[2]),
                ))
            }
            (None, None) => None,
            _ => {
                return Err(PyValueError::new_err(
                    "a tray needs both tray_position and tray_size",
                ))
            }
        };
        self.hub.upsert_device(botrail_scene::seq::Device {
            name: name.to_string(),
            kind: botrail_scene::seq::DeviceKind::Vehicle {
                path: botrail_scene::seq::VehiclePath {
                    waypoints: path
                        .iter()
                        .map(|p| nalgebra::Point2::new(p[0], p[1]))
                        .collect(),
                    stations: stations.into_iter().collect(),
                    ring,
                },
                body: members,
                speed,
                turn_speed,
                start,
                allow_reverse,
                tray,
            },
        });
        Ok(())
    }

    fn remove_device(&self, name: &str) -> PyResult<()> {
        self.hub.remove_device(name).map_err(scene_err)
    }

    #[getter]
    fn device_names(&self) -> Vec<String> {
        self.hub.device_names()
    }

    /// Starts (or replaces) a PLC-style sequence and returns a builder:
    /// `scene.sequence("pick").step("run", actions=[bt.seq.motion("go")])`.
    fn sequence(slf: Py<Self>, py: Python<'_>, name: &str) -> PyResult<Py<PyAny>> {
        let module = py.import("botrail.seq")?;
        Ok(module
            .getattr("SequenceBuilder")?
            .call1((slf, name))?
            .unbind())
    }

    /// Adds or replaces a sequence from wire-format JSON (the
    /// `SequenceBuilder` sugar calls this).
    fn _upsert_sequence_json(&self, json: &str) -> PyResult<()> {
        self.hub
            .upsert_sequence_json(json)
            .map_err(PyValueError::new_err)
    }

    fn remove_sequence(&self, name: &str) -> PyResult<()> {
        self.hub.remove_sequence(name).map_err(scene_err)
    }

    #[getter]
    fn sequence_names(&self) -> Vec<String> {
        self.hub.sequence_names()
    }

    /// Marks contact between `link` (of `robot`) and `obstacle` as
    /// process-intended — a milling cutter in its stock. The pair stops
    /// counting as a collision (checking, planning, `min_clearance`);
    /// toolpath *rapids* deliberately ignore the exemption — while not
    /// cutting, any contact is a crash.
    #[pyo3(signature = (link, obstacle, robot = None))]
    fn allow_link_obstacle_contact(
        &self,
        link: &str,
        obstacle: &str,
        robot: Option<&str>,
    ) -> PyResult<()> {
        let index = self.resolve_robot(robot)?;
        let model = self.hub.robot_model(index);
        let link_index = resolve_link(&model, Some(link))?;
        self.hub
            .allow_link_obstacle_contact(index, link_index, obstacle)
            .map_err(scene_err)
    }

    /// Removes an allowed-contact entry added by
    /// `allow_link_obstacle_contact`.
    #[pyo3(signature = (link, obstacle, robot = None))]
    fn disallow_link_obstacle_contact(
        &self,
        link: &str,
        obstacle: &str,
        robot: Option<&str>,
    ) -> PyResult<()> {
        let index = self.resolve_robot(robot)?;
        let model = self.hub.robot_model(index);
        let link_index = resolve_link(&model, Some(link))?;
        if !self
            .hub
            .disallow_link_obstacle_contact(index, link_index, obstacle)
        {
            return Err(PyValueError::new_err(format!(
                "no allowed contact for link `{link}` and obstacle `{obstacle}`"
            )));
        }
        Ok(())
    }

    /// Adds or replaces a toolpath (a continuous Cartesian process path,
    /// see `bt.toolpath`). `toolpath` is the dict built by
    /// `bt.toolpath.builder()` / `bt.toolpath.from_gcode()` — or its JSON
    /// string. Targets live in the part frame named by its `frame` key
    /// (resolved at bake time, so moving the frame re-solves the path).
    fn add_toolpath(&self, py: Python<'_>, name: &str, toolpath: Bound<'_, PyAny>) -> PyResult<()> {
        let json: String = if let Ok(s) = toolpath.extract::<String>() {
            s
        } else {
            py.import("json")?
                .call_method1("dumps", (&toolpath,))?
                .extract()?
        };
        let mut msg: botrail_scene::toolpath::ToolpathMsg = serde_json::from_str(&json)
            .map_err(|e| PyValueError::new_err(format!("invalid toolpath JSON: {e}")))?;
        msg.name = name.to_string();
        let tp = botrail_scene::toolpath::toolpath_from_msg(&msg)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        self.hub.add_toolpath(tp);
        Ok(())
    }

    fn remove_toolpath(&self, name: &str) -> PyResult<()> {
        if !self.hub.remove_toolpath(name) {
            return Err(PyValueError::new_err(format!("unknown toolpath `{name}`")));
        }
        Ok(())
    }

    #[getter]
    fn toolpath_names(&self) -> Vec<String> {
        self.hub.toolpath_names()
    }

    /// Bakes a toolpath into one continuous trajectory: seed-continuous IK
    /// along the resampled path (5-DOF axis-aligned where the spin is
    /// free), collision-checked per sample, then time-parameterized in one
    /// piece with the commanded feed as a floor — the TCP holds the feed
    /// and slows only where joint limits force it. `segment_ends` on the
    /// result marks each move's completion time. The trajectory starts at
    /// the path's first target; author the approach separately.
    ///
    /// `spin` picks how the free rotation about the tool axis is chosen:
    /// `"greedy"` (seed-continuous, milliseconds) or `"optimize"`
    /// (Descartes-style global pass over a spin grid — spends spin early
    /// to stay solvable late; seconds). `axis_tolerance` (rad) permits
    /// lead/tilt deviation from the authored axis on spin-free samples.
    #[pyo3(signature = (name, robot = None, tcp_link = None, step_pos = 0.005, step_rot = 0.05, jump_threshold = 0.5, rapid_speed = None, axis_tolerance = 0.0, spin = "greedy"))]
    #[allow(clippy::too_many_arguments)]
    fn plan_toolpath(
        &self,
        name: &str,
        robot: Option<&str>,
        tcp_link: Option<&str>,
        step_pos: f64,
        step_rot: f64,
        jump_threshold: f64,
        rapid_speed: Option<f64>,
        axis_tolerance: f64,
        spin: &str,
    ) -> PyResult<Trajectory> {
        let index = self.resolve_robot(robot)?;
        let model = self.hub.robot_model(index);
        let tcp = tcp_link
            .map(|l| resolve_link(&model, Some(l)))
            .transpose()?;
        let options = botrail_scene::toolpath::ToolpathOptions {
            step_pos,
            step_rot,
            jump_threshold,
            rapid_speed,
            axis_tolerance,
            spin: spin_mode(spin)?,
        };
        let planned = self
            .hub
            .plan_toolpath(name, index, tcp, &options)
            .map_err(PyValueError::new_err)?;
        // Per-sample linear segments carrying their interval's commanded
        // speed: script export renders the polyline as a blended
        // movep/movel chain at the feed (`to_script(blend_radius=...)`).
        let segments = planned
            .path
            .windows(2)
            .zip(planned.samples.iter().skip(1))
            .map(|(w, sample)| botrail_scene::motion::PlannedSegment {
                kind: botrail_scene::motion::SegmentKind::CartesianLine,
                waypoints: w.to_vec(),
                tcp_speed: sample.feed.or(rapid_speed),
            })
            .collect();
        Ok(Trajectory {
            inner: planned.trajectory,
            joint_names: model
                .actuated_joint_names()
                .into_iter()
                .map(str::to_string)
                .collect(),
            segment_ends: planned.move_ends,
            segments,
            limits: hub::traj_limits(&model),
            feed_report: Some(planned.feed_report),
        })
    }

    /// Attempts every sample of a toolpath and reports all failures
    /// (unreachable / IK-branch jump / collision) without aborting — the
    /// pre-teach "which points can I not reach" face diagnosis.
    #[pyo3(signature = (name, robot = None, tcp_link = None, step_pos = 0.005, step_rot = 0.05, jump_threshold = 0.5, axis_tolerance = 0.0, spin = "greedy"))]
    #[allow(clippy::too_many_arguments)]
    fn check_toolpath(
        &self,
        name: &str,
        robot: Option<&str>,
        tcp_link: Option<&str>,
        step_pos: f64,
        step_rot: f64,
        jump_threshold: f64,
        axis_tolerance: f64,
        spin: &str,
    ) -> PyResult<ToolpathReport> {
        let index = self.resolve_robot(robot)?;
        let model = self.hub.robot_model(index);
        let tcp = tcp_link
            .map(|l| resolve_link(&model, Some(l)))
            .transpose()?;
        let options = botrail_scene::toolpath::ToolpathOptions {
            step_pos,
            step_rot,
            jump_threshold,
            rapid_speed: None,
            axis_tolerance,
            spin: spin_mode(spin)?,
        };
        let report = self
            .hub
            .check_toolpath(name, index, tcp, &options)
            .map_err(PyValueError::new_err)?;
        Ok(ToolpathReport { inner: report })
    }

    /// Progressive material removal for a baked cycle: carves `stock` in
    /// `stages` equal time slices (default: one slice per second of
    /// cycle, capped at 240 — the display lags the tool by at most one
    /// slice, so this keeps the lag around a second), registers one
    /// display-only obstacle per changed slice (grouped under
    /// `{stock}_cut/…` in the scene tree, cheap AABB colliders — they
    /// never collide), and returns the timeline with the visibility
    /// windows injected: during playback — studio, USD export, and a
    /// replayed recording alike — the stock disappears as it is cut
    /// instead of starting pre-cut. The stock keeps colliding unchanged;
    /// everything here is presentation.
    #[pyo3(signature = (timeline, stock, stages = None, voxel_size = 0.001, cutter_radius = 0.004, cutter_length = 0.03, dt = 0.01, robot = None, tcp_link = None))]
    #[allow(clippy::too_many_arguments)]
    fn animate_carve(
        &self,
        timeline: PyRef<'_, SequenceTimeline>,
        stock: &str,
        stages: Option<usize>,
        voxel_size: f64,
        cutter_radius: f64,
        cutter_length: f64,
        dt: f64,
        robot: Option<&str>,
        tcp_link: Option<&str>,
    ) -> PyResult<SequenceTimeline> {
        let stages =
            stages.unwrap_or_else(|| (timeline.inner.duration.ceil() as usize).clamp(1, 240));
        let (index, _) = timeline.track_for(robot)?;
        let model = &timeline.scene.robots()[index].model;
        let tcp = match tcp_link {
            Some(l) => resolve_link(&std::sync::Arc::clone(model), Some(l))?,
            None => model.default_tcp_link(),
        };
        let options = botrail_scene::carve::CarveOptions {
            voxel_size,
            cutter_radius,
            cutter_length,
            dt,
            ..botrail_scene::carve::CarveOptions::default()
        };
        let (carve, stage_list) = botrail_scene::carve::carve_stock_staged(
            &timeline.scene,
            &timeline.inner,
            stock,
            index,
            tcp,
            &options,
            stages,
        )
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
        if stage_list.is_empty() {
            return Err(PyValueError::new_err(format!(
                "the cycle never cuts `{stock}` — nothing to animate"
            )));
        }

        // Stage meshes go to the cache as OBJ + MTL (the format the
        // studio and the USD export read face colors back from),
        // content-addressed so re-runs reuse files.
        let dir = cache_base().join("carve");
        std::fs::create_dir_all(&dir).map_err(|e| PyIOError::new_err(e.to_string()))?;
        let material = botrail_scene::Material::new(0.75, 0.35);
        let mut entries: Vec<(
            String,
            botrail_model::Geometry,
            botrail_scene::ObstacleCollider,
        )> = Vec::with_capacity(stage_list.len());
        let mut times = Vec::with_capacity(stage_list.len());
        for (i, stage) in stage_list.iter().enumerate() {
            let hash = {
                // FNV-1a over the vertex/index bytes: stable across runs
                // and Rust versions, which is all a cache name needs.
                let mut h: u64 = 0xcbf2_9ce4_8422_2325;
                let mut eat = |bytes: &[u8]| {
                    for b in bytes {
                        h ^= *b as u64;
                        h = h.wrapping_mul(0x1000_0000_01b3);
                    }
                };
                for v in &stage.mesh.vertices {
                    for c in v {
                        eat(&c.to_le_bytes());
                    }
                }
                for t in &stage.mesh.indices {
                    for c in t {
                        eat(&c.to_le_bytes());
                    }
                }
                h
            };
            let obj_path = dir.join(format!("{hash:016x}.obj"));
            let mtl_name = format!("{hash:016x}.mtl");
            if !obj_path.exists() {
                let (obj, mtl) = botrail_mesh::to_obj_with_mtl(&stage.mesh, &mtl_name);
                std::fs::write(&obj_path, obj).map_err(|e| PyIOError::new_err(e.to_string()))?;
                std::fs::write(dir.join(&mtl_name), mtl)
                    .map_err(|e| PyIOError::new_err(e.to_string()))?;
            }
            // A cheap stand-in collider: the stage registers disabled, so
            // VHACD on it would be pure cost.
            let half = {
                let mut lo = [f64::INFINITY; 3];
                let mut hi = [f64::NEG_INFINITY; 3];
                for v in &stage.mesh.vertices {
                    for a in 0..3 {
                        lo[a] = lo[a].min(v[a]);
                        hi[a] = hi[a].max(v[a]);
                    }
                }
                nalgebra::Vector3::new(
                    ((hi[0] - lo[0]) / 2.0).max(1e-6),
                    ((hi[1] - lo[1]) / 2.0).max(1e-6),
                    ((hi[2] - lo[2]) / 2.0).max(1e-6),
                )
            };
            entries.push((
                format!("{stock}_cut/{i:03}"),
                botrail_model::Geometry::Mesh {
                    path: obj_path.clone(),
                    scale: nalgebra::Vector3::new(1.0, 1.0, 1.0),
                },
                botrail_scene::ObstacleCollider::cuboid(half),
            ));
            times.push(stage.time);
        }

        // Register on the live scene (one broadcast) and on the
        // timeline's snapshot — the augmented timeline's own scene must
        // hold the stages for its USD export to include them.
        let mut snapshot = timeline.scene.clone();
        for (name, geometry, collider) in &entries {
            let _ = snapshot.remove_obstacle(name);
            let final_name = snapshot.add_obstacle_with_collider(
                name,
                geometry.clone(),
                carve.pose,
                collider.clone(),
            );
            let _ = snapshot.set_obstacle_enabled(&final_name, false);
            let _ = snapshot.set_obstacle_material(&final_name, Some(material));
        }
        let names = self.hub.add_carve_stages(entries, carve.pose, material);

        let augmented = botrail_scene::carve::staged_timeline(
            &timeline.inner,
            stock,
            carve.pose,
            &names,
            &times,
        );
        self.hub.emit_timeline(&snapshot, &augmented);
        Ok(SequenceTimeline {
            inner: augmented,
            scene: snapshot,
        })
    }

    /// Wraps a planned trajectory as a single-robot `SequenceTimeline` so
    /// the timeline consumers — studio playback, `export_usd`,
    /// `min_clearance` — accept it without authoring a sequence. Other
    /// robots hold their current pose; objects stay static. Script export
    /// is not supported on the result.
    #[pyo3(signature = (trajectory, robot = None, label = "trajectory"))]
    fn timeline_from_trajectory(
        &self,
        trajectory: PyRef<'_, Trajectory>,
        robot: Option<&str>,
        label: &str,
    ) -> PyResult<SequenceTimeline> {
        let index = self.resolve_robot(robot)?;
        let (timeline, scene) = self
            .hub
            .timeline_from_trajectory(index, &trajectory.inner, label);
        Ok(SequenceTimeline {
            inner: timeline,
            scene,
        })
    }

    /// Rolls out a sequence with the PLC scan loop against a snapshot of
    /// this scene (motions plan at their step, grasped objects ride along)
    /// and returns the baked timeline. Also broadcasts the result to
    /// connected studio clients for playback.
    ///
    /// `scenario` applies a named initial-state delta (`add_scenario`) to
    /// the snapshot first — the live scene is never touched. `None` and
    /// `"baseline"` both mean the scene as it stands.
    #[pyo3(signature = (name, dt = 0.01, max_duration = 120.0, plan_resolution = None, scenario = None, toolpath_spin = None))]
    #[allow(clippy::too_many_arguments)]
    fn simulate_sequence(
        &self,
        name: &str,
        dt: f64,
        max_duration: f64,
        plan_resolution: Option<f64>,
        scenario: Option<&str>,
        toolpath_spin: Option<&str>,
    ) -> PyResult<SequenceTimeline> {
        if !(dt.is_finite() && dt > 0.0) {
            return Err(PyValueError::new_err(format!(
                "dt must be positive, got {dt}"
            )));
        }
        let mut options = botrail_scene::rollout::RolloutOptions {
            dt,
            max_duration,
            ..Default::default()
        };
        if let Some(resolution) = plan_resolution {
            if !(resolution.is_finite() && resolution > 0.0) {
                return Err(PyValueError::new_err(format!(
                    "plan_resolution must be positive, got {resolution}"
                )));
            }
            options.plan.resolution = resolution;
        }
        if let Some(mode) = toolpath_spin {
            options.toolpath.spin = spin_mode(mode)?;
        }
        let (timeline, scene) = self
            .hub
            .simulate_sequence(name, scenario, &options)
            .map_err(PyValueError::new_err)?;
        Ok(SequenceTimeline {
            inner: timeline,
            scene,
        })
    }

    /// Rolls out several sequences **concurrently** — the PLC picture of a
    /// line: one program per station plus a transfer program, each a plain
    /// serial SFC, synchronized only through signals and sensors. One scan
    /// tick advances every program in list order, so the bake stays
    /// bit-identical run to run; the result is a single timeline whose
    /// step spans carry `program/step` names.
    ///
    /// Every robot, device, and written signal must be commanded by at
    /// most one of the programs — two programs driving one resource is
    /// rejected up front, like two PLC programs writing one coil.
    /// `plan_resolution` tightens the planner's edge-validity stride (rad,
    /// joint-space L2). The default 0.05 samples a big arm's sweep every
    /// ~10 cm of TCP travel — coarse enough to step across sheet metal, so
    /// cells full of 12 mm flanges pass 0.005.
    #[pyo3(signature = (names, dt = 0.01, max_duration = 120.0, plan_resolution = None, scenario = None, toolpath_spin = None))]
    #[allow(clippy::too_many_arguments)]
    fn simulate_sequences(
        &self,
        names: Vec<String>,
        dt: f64,
        max_duration: f64,
        plan_resolution: Option<f64>,
        scenario: Option<&str>,
        toolpath_spin: Option<&str>,
    ) -> PyResult<SequenceTimeline> {
        if !(dt.is_finite() && dt > 0.0) {
            return Err(PyValueError::new_err(format!(
                "dt must be positive, got {dt}"
            )));
        }
        let mut options = botrail_scene::rollout::RolloutOptions {
            dt,
            max_duration,
            ..Default::default()
        };
        if let Some(resolution) = plan_resolution {
            if !(resolution.is_finite() && resolution > 0.0) {
                return Err(PyValueError::new_err(format!(
                    "plan_resolution must be positive, got {resolution}"
                )));
            }
            options.plan.resolution = resolution;
        }
        if let Some(mode) = toolpath_spin {
            options.toolpath.spin = spin_mode(mode)?;
        }
        let refs: Vec<&str> = names.iter().map(String::as_str).collect();
        let (timeline, scene) = self
            .hub
            .simulate_sequences(&refs, scenario, &options)
            .map_err(PyValueError::new_err)?;
        Ok(SequenceTimeline {
            inner: timeline,
            scene,
        })
    }

    /// Defines (or replaces) a scenario — a named initial-state delta the
    /// `simulate_*` calls can run under. Deltas only: `signals` overrides
    /// declared internal-signal initial values, `obstacles` maps names to
    /// a position or a `(position, quaternion)` pair, `joints` maps robot
    /// instances to start configurations. `"baseline"` is the reserved
    /// name of the unmodified scene. Everything is validated when the
    /// scenario is *applied* (at simulate), so deltas may name things
    /// authored later.
    #[pyo3(signature = (name, signals = None, obstacles = None, joints = None))]
    fn add_scenario(
        &self,
        name: &str,
        signals: Option<&Bound<'_, PyDict>>,
        obstacles: Option<&Bound<'_, PyDict>>,
        joints: Option<&Bound<'_, PyDict>>,
    ) -> PyResult<()> {
        let mut scenario = botrail_scene::seq::Scenario {
            name: name.to_string(),
            signals: Vec::new(),
            obstacles: Vec::new(),
            joints: Vec::new(),
        };
        if let Some(signals) = signals {
            for (key, value) in signals.iter() {
                scenario
                    .signals
                    .push((key.extract::<String>()?, value.extract::<bool>()?));
            }
        }
        if let Some(obstacles) = obstacles {
            for (key, value) in obstacles.iter() {
                let pose: ScenarioPose = value.extract()?;
                scenario
                    .obstacles
                    .push((key.extract::<String>()?, pose.into_iso()));
            }
        }
        if let Some(joints) = joints {
            for (key, value) in joints.iter() {
                scenario
                    .joints
                    .push((key.extract::<String>()?, value.extract::<Vec<f64>>()?));
            }
        }
        self.hub.add_scenario(scenario).map_err(scene_err)
    }

    fn remove_scenario(&self, name: &str) -> PyResult<()> {
        self.hub.remove_scenario(name).map_err(scene_err)
    }

    /// Defined scenario names, in authoring order (`baseline` — the
    /// unmodified scene — is implicit and never listed).
    #[getter]
    fn scenario_names(&self) -> Vec<String> {
        self.hub.scenario_names()
    }

    /// Rolls the same sequences under a set of scenarios — the cell's
    /// test-case matrix in one call. `scenarios=None` runs `baseline`
    /// plus every defined scenario. A scenario that fails (bad delta,
    /// plan failure, timeout) is *collected* into the result's `errors`
    /// rather than aborting the sweep — finding the failing scenario is
    /// the point. Each run is deterministic, so coverage and cycle times
    /// off the result are CI-assertable numbers.
    #[pyo3(signature = (names, scenarios = None, dt = 0.01, max_duration = 120.0,
        plan_resolution = None))]
    fn simulate_scenarios(
        &self,
        py: Python<'_>,
        names: Vec<String>,
        scenarios: Option<Vec<String>>,
        dt: f64,
        max_duration: f64,
        plan_resolution: Option<f64>,
    ) -> PyResult<ScenarioRuns> {
        if !(dt.is_finite() && dt > 0.0) {
            return Err(PyValueError::new_err(format!(
                "dt must be positive, got {dt}"
            )));
        }
        let mut options = botrail_scene::rollout::RolloutOptions {
            dt,
            max_duration,
            ..Default::default()
        };
        if let Some(resolution) = plan_resolution {
            if !(resolution.is_finite() && resolution > 0.0) {
                return Err(PyValueError::new_err(format!(
                    "plan_resolution must be positive, got {resolution}"
                )));
            }
            options.plan.resolution = resolution;
        }
        let set: Vec<String> = match scenarios {
            Some(list) => {
                for (i, name) in list.iter().enumerate() {
                    if list[..i].contains(name) {
                        return Err(PyValueError::new_err(format!(
                            "scenario `{name}` is listed twice"
                        )));
                    }
                }
                list
            }
            None => std::iter::once(botrail_scene::seq::BASELINE_SCENARIO.to_string())
                .chain(self.hub.scenario_names())
                .collect(),
        };
        let refs: Vec<&str> = names.iter().map(String::as_str).collect();
        let mut runs = Vec::new();
        let mut errors = Vec::new();
        for scenario in &set {
            match self.hub.simulate_sequences(&refs, Some(scenario), &options) {
                Ok((timeline, scene)) => runs.push((
                    scenario.clone(),
                    Py::new(
                        py,
                        SequenceTimeline {
                            inner: timeline,
                            scene,
                        },
                    )?,
                )),
                Err(e) => errors.push((scenario.clone(), e)),
            }
        }
        Ok(ScenarioRuns {
            runs,
            failures: errors,
            scene: self.hub.authored_snapshot(),
        })
    }

    /// Colliding pairs at the current configuration, as
    /// `((kind, name), (kind, name))` tuples with kind `"link"`/`"obstacle"`.
    fn check_collisions(&self) -> Vec<((String, String), (String, String))> {
        self.hub.collision_pairs()
    }

    fn in_collision(&self) -> bool {
        !self.hub.collision_pairs().is_empty()
    }

    /// Minimum robot-obstacle distance (0 when colliding); `None` without
    /// obstacles.
    fn min_obstacle_distance(&self) -> Option<f64> {
        self.hub.min_obstacle_distance()
    }

    /// Link shapes skipped for collision checking (e.g. meshes, until the
    /// mesh I/O crate lands).
    #[getter]
    fn collision_warnings(&self) -> Vec<String> {
        self.hub.collision_warnings()
    }

    /// Plans a collision-free, time-parameterized trajectory from the
    /// current configuration to `goal` (joint positions in DOF order).
    /// With `broadcast=True` (default) the result is also pushed to
    /// connected studio clients for preview playback.
    #[pyo3(signature = (goal, max_iters = 10_000, seed = None, broadcast = true, robot = None))]
    fn plan(
        &self,
        goal: Vec<f64>,
        max_iters: usize,
        seed: Option<u64>,
        broadcast: bool,
        robot: Option<&str>,
    ) -> PyResult<Trajectory> {
        let index = self.resolve_robot(robot)?;
        let mut options = botrail_plan::PlanOptions {
            max_iters,
            ..botrail_plan::PlanOptions::default()
        };
        if let Some(seed) = seed {
            options.seed = seed;
        }
        let result = if broadcast {
            self.hub.plan_and_broadcast_for(index, &goal, &options)
        } else {
            self.hub.plan_to_for(index, &goal, &options)
        };
        let (traj, path, _) = result.map_err(PyValueError::new_err)?;
        let model = self.hub.robot_model(index);
        Ok(Trajectory {
            inner: traj,
            segment_ends: Vec::new(),
            segments: vec![botrail_scene::motion::PlannedSegment {
                kind: botrail_scene::motion::SegmentKind::Joint,
                waypoints: path,
                tcp_speed: None,
            }],
            joint_names: model
                .actuated_joint_names()
                .into_iter()
                .map(str::to_string)
                .collect(),
            limits: hub::traj_limits(&model),
            feed_report: None,
        })
    }

    /// Appends a waypoint segment to `motion` (created when missing).
    /// `goal=None` captures the current configuration. Constraints:
    /// `orientation_cone=(axis_local, axis_world, angle_rad)` keeps the tool
    /// axis inside a cone; `position_box=(min, max)` keeps the TCP inside a
    /// world-aligned box. Both apply along the whole segment.
    #[pyo3(signature = (motion, goal = None, kind = "joint", orientation_cone = None, position_box = None, robot = None))]
    fn add_segment(
        &self,
        motion: &str,
        goal: Option<Vec<f64>>,
        kind: &str,
        orientation_cone: Option<([f64; 3], [f64; 3], f64)>,
        position_box: Option<([f64; 3], [f64; 3])>,
        robot: Option<&str>,
    ) -> PyResult<()> {
        let index = self.resolve_robot(robot)?;
        let kind = match kind {
            "joint" => botrail_scene::motion::SegmentKind::Joint,
            "cartesian_line" => botrail_scene::motion::SegmentKind::CartesianLine,
            other => {
                return Err(PyValueError::new_err(format!(
                    "kind must be \"joint\" or \"cartesian_line\", got {other:?}"
                )))
            }
        };
        let mut constraints = Vec::new();
        if let Some((axis_local, axis_world, angle)) = orientation_cone {
            constraints.push(botrail_scene::motion::Constraint::OrientationCone {
                axis_local: nalgebra::Vector3::new(axis_local[0], axis_local[1], axis_local[2]),
                axis_world: nalgebra::Vector3::new(axis_world[0], axis_world[1], axis_world[2]),
                angle,
            });
        }
        if let Some((min, max)) = position_box {
            constraints.push(botrail_scene::motion::Constraint::PositionBox {
                min: nalgebra::Vector3::new(min[0], min[1], min[2]),
                max: nalgebra::Vector3::new(max[0], max[1], max[2]),
            });
        }
        let segment = botrail_scene::motion::Segment {
            kind,
            goal_positions: goal.unwrap_or_else(|| self.hub.joint_positions_for(index)),
            constraints,
        };
        self.hub
            .add_segment_for(index, motion, segment)
            .map_err(PyValueError::new_err)
    }

    fn remove_segment(&self, motion: &str, index: usize) -> PyResult<()> {
        self.hub
            .remove_segment(motion, index)
            .map_err(PyValueError::new_err)
    }

    fn clear_motion(&self, motion: &str) -> PyResult<()> {
        self.hub.clear_motion(motion).map_err(PyValueError::new_err)
    }

    #[getter]
    fn motion_names(&self) -> Vec<String> {
        self.hub.motion_names()
    }

    /// Segments of a motion as `(kind, goal_positions)` tuples.
    fn motion_segments(&self, name: &str) -> Vec<(String, Vec<f64>)> {
        self.hub.motion_segments(name)
    }

    /// Plans every segment of `motion` from the current configuration into
    /// one trajectory (rest-to-rest at segment boundaries). With
    /// `broadcast=True` the result is pushed to connected studios.
    #[pyo3(signature = (motion, seed = None, broadcast = true))]
    fn plan_motion(
        &self,
        motion: &str,
        seed: Option<u64>,
        broadcast: bool,
    ) -> PyResult<Trajectory> {
        let mut options = botrail_plan::PlanOptions::default();
        if let Some(seed) = seed {
            options.seed = seed;
        }
        let planned = if broadcast {
            self.hub
                .plan_motion_and_broadcast(motion, &options)
                .map(|(planned, _)| planned)
        } else {
            self.hub.plan_motion_snapshot(motion, &options)
        }
        .map_err(PyValueError::new_err)?;
        let owner = self.hub.motion_owner(motion).unwrap_or(0);
        let model = self.hub.robot_model(owner);
        Ok(Trajectory {
            inner: planned.trajectory,
            segment_ends: planned.segment_ends,
            segments: planned.segments,
            joint_names: model
                .actuated_joint_names()
                .into_iter()
                .map(str::to_string)
                .collect(),
            limits: hub::traj_limits(&model),
            feed_report: None,
        })
    }

    /// Saves the whole cell — robots (URDF embedded, USD by reference),
    /// joint state, obstacles, frames, motions, sequences, signals,
    /// sensors, and devices — as a `.botrail` project file. Plain JSON
    /// when everything is self-contained; a zip archive (`project.json` +
    /// `assets/`) when mesh files are referenced, so the file stays
    /// portable across machines.
    fn save_project(&self, path: PathBuf) -> PyResult<()> {
        let io_err = |e: std::io::Error| PyIOError::new_err(format!("{}: {e}", path.display()));
        let mut project = self.hub.project();

        // Collect referenced mesh files and rewrite their urls to bundled
        // asset names.
        let mut assets: Vec<(String, PathBuf)> = Vec::new();
        for o in &mut project.obstacles {
            if let botrail_scene::wire::GeometryMsg::Mesh { url, .. } = &mut o.geometry {
                let source = PathBuf::from(&*url);
                let file_name = source
                    .file_name()
                    .map(|f| f.to_string_lossy().into_owned())
                    .unwrap_or_else(|| "mesh".to_string());
                let asset = format!("assets/{}_{}", assets.len(), file_name);
                *url = asset.clone();
                assets.push((asset, source));
            }
        }

        // A USD-sourced robot bundles its stage layers (root + sublayers +
        // reference targets under the stage directory) as
        // `robot/<relpath>` (`robot_<n>/<relpath>` for later robots, so two
        // stages with same-named files cannot collide).
        for (i, robot) in project.robots.iter_mut().enumerate() {
            let bundle_dir = if i == 0 {
                "robot".to_string()
            } else {
                format!("robot_{}", i + 1)
            };
            let botrail_scene::project::RobotSourceMsg::Usd {
                path: stage_path, ..
            } = &mut robot.source
            else {
                continue;
            };
            let root = PathBuf::from(&*stage_path);
            let Some(root_dir) = root.parent().map(|d| d.to_path_buf()) else {
                continue;
            };
            let deps = botrail_usd::stage_dependencies(&root, &[])
                .map_err(|e| PyValueError::new_err(e.to_string()))?;
            for dep in deps {
                match dep.strip_prefix(&root_dir) {
                    Ok(rel) => assets.push((
                        format!("{bundle_dir}/{}", rel.to_string_lossy().replace('\\', "/")),
                        dep.clone(),
                    )),
                    Err(_) => eprintln!(
                        "botrail: project: stage layer {} is outside the robot directory; \
                         referenced by absolute path (not bundled)",
                        dep.display()
                    ),
                }
            }
            let rel_root = root
                .strip_prefix(&root_dir)
                .expect("root is inside its parent")
                .to_string_lossy()
                .replace('\\', "/");
            *stage_path = format!("{bundle_dir}/{rel_root}");
        }

        if assets.is_empty() {
            return std::fs::write(&path, project.to_json()).map_err(io_err);
        }

        use std::io::Write as _;
        let file = std::fs::File::create(&path).map_err(io_err)?;
        let mut archive = zip::ZipWriter::new(file);
        let options = zip::write::SimpleFileOptions::default();
        let zip_err =
            |e: zip::result::ZipError| PyIOError::new_err(format!("{}: {e}", path.display()));
        archive
            .start_file("project.json", options)
            .map_err(zip_err)?;
        archive
            .write_all(project.to_json().as_bytes())
            .map_err(io_err)?;
        for (name, source) in assets {
            let bytes = std::fs::read(&source)
                .map_err(|e| PyIOError::new_err(format!("{}: {e}", source.display())))?;
            archive.start_file(name, options).map_err(zip_err)?;
            archive.write_all(&bytes).map_err(io_err)?;
        }
        archive.finish().map_err(zip_err)?;
        Ok(())
    }

    /// Loads a `.botrail` project file into a fresh scene (robots
    /// included). URDF robots rebuild from the embedded XML; USD robots
    /// re-import from the referenced stage path.
    #[staticmethod]
    fn load_project(path: PathBuf) -> PyResult<Self> {
        let bytes = std::fs::read(&path)
            .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))?;
        let project = read_project(&bytes)
            .map_err(|e| PyValueError::new_err(format!("{}: {e}", path.display())))?;
        let import_usd = |path: &str, articulation_root: &str| {
            botrail_usd::import_robot(
                std::path::Path::new(path),
                &botrail_usd::RobotImportOptions {
                    articulation_root: Some(articulation_root.to_string()),
                    ..Default::default()
                },
            )
            .map(|imported| imported.model)
            .map_err(|e| e.to_string())
        };
        let mut models = Vec::with_capacity(project.robots.len());
        for robot_msg in &project.robots {
            let model = botrail_scene::project::model_from_source(&robot_msg.source, &import_usd)
                .map_err(|e| PyValueError::new_err(e.to_string()))?;
            models.push(Arc::new(model));
        }
        let mut models = models.into_iter();
        let first = models
            .next()
            .ok_or_else(|| PyValueError::new_err("project has no robots"))?;
        let mut scene = botrail_scene::Scene::new(first);
        for model in models {
            scene.add_robot(model, None, nalgebra::Isometry3::identity());
        }
        // apply_project restores instance names, bases, and joints.
        scene
            .apply_project(&project)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let robot = Robot {
            inner: scene.robot().clone(),
        };
        Ok(Scene {
            hub: Arc::new(SceneHub::new(scene)),
            robot,
        })
    }

    /// Generates a Python script that rebuilds this scene with the botrail
    /// API (same content as the studio's "Export Python").
    fn generate_python(&self) -> String {
        self.hub.python_code()
    }

    /// IK to the given pose, then plan to the found configuration.
    #[pyo3(signature = (position, quaternion = None, link = None, max_iters = 10_000, seed = None, broadcast = true, robot = None))]
    #[allow(clippy::too_many_arguments)]
    fn plan_to_pose(
        &self,
        position: [f64; 3],
        quaternion: Option<[f64; 4]>,
        link: Option<&str>,
        max_iters: usize,
        seed: Option<u64>,
        broadcast: bool,
        robot: Option<&str>,
    ) -> PyResult<Trajectory> {
        let index = self.resolve_robot(robot)?;
        let model = self.hub.robot_model(index);
        let link_index = resolve_link(&model, link)?;
        let (target, mode) = ik_target(position, quaternion);
        // The target is world-frame; re-express it in the robot base frame
        // for the base-frame solver.
        let target = self.hub.robot_base_isometry_for(index).inverse() * target;
        let seed_q = self.hub.joint_positions_for(index);
        let options = botrail_kin::IkOptions {
            mode,
            ..botrail_kin::IkOptions::default()
        };
        let ik = botrail_kin::solve_ik(&model, link_index, &target, &seed_q, &options)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        if !ik.converged {
            return Err(PyValueError::new_err(format!(
                "IK did not converge (pos_error={:.4}, rot_error={:.4})",
                ik.pos_error, ik.rot_error
            )));
        }
        self.plan(ik.q, max_iters, seed, broadcast, robot)
    }

    /// Solves IK toward the given pose (seeded from the current
    /// configuration), applies the best-effort result to the scene, and
    /// pushes it to connected studio clients. `quaternion=None` matches
    /// position only; `link` defaults to the TCP link.
    #[pyo3(signature = (position, quaternion = None, link = None, max_iters = 100, robot = None))]
    fn set_tcp_target(
        &self,
        position: [f64; 3],
        quaternion: Option<[f64; 4]>,
        link: Option<&str>,
        max_iters: usize,
        robot: Option<&str>,
    ) -> PyResult<IkResult> {
        let index = self.resolve_robot(robot)?;
        let model = self.hub.robot_model(index);
        let link_index = resolve_link(&model, link)?;
        let link_name = model.links[link_index].name.clone();
        let pose = botrail_scene::wire::PoseMsg {
            position,
            quaternion: quaternion.unwrap_or([0.0, 0.0, 0.0, 1.0]),
        };
        let mode = match quaternion {
            Some(_) => botrail_kin::IkMode::Pose,
            None => botrail_kin::IkMode::Position,
        };
        let options = botrail_kin::IkOptions {
            mode,
            max_iters,
            ..botrail_kin::IkOptions::default()
        };
        let result = self
            .hub
            .set_tcp_target_for(index, &link_name, &pose, &options)
            .map_err(PyValueError::new_err)?;
        Ok(IkResult { inner: result })
    }

    fn __repr__(&self) -> String {
        let names = self.hub.robot_names();
        match names.as_slice() {
            [single] => format!("Scene(robot='{single}')"),
            names => format!("Scene(robots={names:?})"),
        }
    }
}

/// Parses project bytes: a zip archive (`project.json` + `assets/`, with
/// assets extracted to the cache and urls rewritten) or plain JSON.
fn read_project(bytes: &[u8]) -> Result<botrail_scene::project::ProjectFile, String> {
    use std::io::Read as _;
    if !bytes.starts_with(b"PK\x03\x04") {
        let json = std::str::from_utf8(bytes).map_err(|e| e.to_string())?;
        return botrail_scene::project::ProjectFile::from_json(json).map_err(|e| e.to_string());
    }

    let mut archive =
        zip::ZipArchive::new(std::io::Cursor::new(bytes)).map_err(|e| e.to_string())?;
    let mut json = String::new();
    archive
        .by_name("project.json")
        .map_err(|e| format!("project.json: {e}"))?
        .read_to_string(&mut json)
        .map_err(|e| e.to_string())?;
    let mut project =
        botrail_scene::project::ProjectFile::from_json(&json).map_err(|e| e.to_string())?;

    // Extract bundled assets to a content-addressed cache directory.
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    std::hash::Hash::hash(bytes, &mut hasher);
    let dir = cache_base()
        .join("projects")
        .join(format!("{:016x}", std::hash::Hasher::finish(&hasher)));
    let names: Vec<String> = archive
        .file_names()
        .filter(|n| n.starts_with("assets/") || n.starts_with("robot/") || n.starts_with("robot_"))
        .map(str::to_string)
        .collect();
    for name in &names {
        let target = dir.join(name);
        if target.exists() {
            continue;
        }
        if let Some(parent) = target.parent() {
            std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
        }
        let mut data = Vec::new();
        archive
            .by_name(name)
            .map_err(|e| e.to_string())?
            .read_to_end(&mut data)
            .map_err(|e| e.to_string())?;
        std::fs::write(&target, data).map_err(|e| e.to_string())?;
    }

    // Point mesh urls and bundled robot stages at the extracted copies.
    for o in &mut project.obstacles {
        if let botrail_scene::wire::GeometryMsg::Mesh { url, .. } = &mut o.geometry {
            if url.starts_with("assets/") {
                *url = dir.join(&*url).display().to_string();
            }
        }
    }
    for robot in &mut project.robots {
        if let botrail_scene::project::RobotSourceMsg::Usd { path, .. } = &mut robot.source {
            if path.starts_with("robot/") || path.starts_with("robot_") {
                *path = dir.join(&*path).display().to_string();
            }
        }
    }
    Ok(project)
}

/// `$BOTRAIL_CACHE_DIR`, else `~/.cache/botrail`, else the system temp dir.
fn cache_base() -> PathBuf {
    std::env::var_os("BOTRAIL_CACHE_DIR")
        .map(PathBuf::from)
        .or_else(|| {
            std::env::var_os("HOME").map(|home| PathBuf::from(home).join(".cache").join("botrail"))
        })
        .unwrap_or_else(|| std::env::temp_dir().join("botrail-cache"))
}

/// A planned, time-parameterized joint trajectory.
#[pyclass(frozen, module = "botrail._core")]
struct Trajectory {
    inner: botrail_traj::JointTrajectory,
    joint_names: Vec<String>,
    /// Time at which each motion segment ends (empty for single plans).
    segment_ends: Vec<f64>,
    /// Per-segment sparse planned paths (single plans hold one segment).
    segments: Vec<botrail_scene::motion::PlannedSegment>,
    /// Joint limits the trajectory was timed with (drive script speeds).
    limits: botrail_traj::Limits,
    /// Feed adherence of a toolpath bake (`None` for ordinary plans).
    feed_report: Option<botrail_scene::toolpath::FeedReport>,
}

impl Trajectory {
    /// Renders the sparse planned path as a vendor robot script.
    #[allow(clippy::too_many_arguments)]
    fn render_script(
        &self,
        dialect: &str,
        name: &str,
        speed_scale: f64,
        blend_radius: f64,
        tcp_speed: f64,
        tcp_accel: f64,
        move_to_start: bool,
    ) -> PyResult<String> {
        let backend = botrail_export::backend(dialect).ok_or_else(|| {
            PyValueError::new_err(format!(
                "unknown dialect {dialect:?} (available: {})",
                botrail_export::DIALECTS.join(", ")
            ))
        })?;
        let segments: Vec<botrail_export::PathSegment> = self
            .segments
            .iter()
            .map(|s| botrail_export::PathSegment {
                kind: match s.kind {
                    botrail_scene::motion::SegmentKind::Joint => botrail_export::PathKind::Joint,
                    botrail_scene::motion::SegmentKind::CartesianLine => {
                        botrail_export::PathKind::Linear
                    }
                },
                tcp_speed: s.tcp_speed,
                waypoints: s.waypoints.clone(),
            })
            .collect();
        let options = botrail_export::ProgramOptions {
            speed_scale,
            blend_radius,
            tcp_speed,
            tcp_accel,
            move_to_start,
        };
        let program = botrail_export::build_program(
            name,
            &self.joint_names,
            &segments,
            &self.limits.velocity,
            &self.limits.acceleration,
            &options,
        )
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
        backend
            .emit(&program)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }
}

#[pymethods]
impl Trajectory {
    #[getter]
    fn joint_names(&self) -> Vec<String> {
        self.joint_names.clone()
    }

    /// Time at which each motion segment ends (empty for single plans).
    #[getter]
    fn segment_ends(&self) -> Vec<f64> {
        self.segment_ends.clone()
    }

    /// Feed adherence of a toolpath bake (`None` for ordinary plans).
    #[getter]
    fn feed_report(&self) -> Option<FeedReport> {
        self.feed_report.as_ref().map(|r| FeedReport {
            inner: r.clone(),
            joint_names: self.joint_names.clone(),
        })
    }

    /// Sparse planned path per segment, as `(kind, waypoints)` tuples with
    /// `kind` in `{"joint", "cartesian_line"}`. For joint segments these
    /// are the shortcut waypoints, for cartesian segments the IK follow
    /// points; both endpoints are included. This is the input for script
    /// exporters (one move command per waypoint), unlike `positions`,
    /// which is densified for time parameterization.
    #[getter]
    fn segments(&self) -> Vec<(String, Vec<Vec<f64>>)> {
        self.segments
            .iter()
            .map(|s| {
                let kind = match s.kind {
                    botrail_scene::motion::SegmentKind::Joint => "joint",
                    botrail_scene::motion::SegmentKind::CartesianLine => "cartesian_line",
                };
                (kind.to_string(), s.waypoints.clone())
            })
            .collect()
    }

    #[getter]
    fn times(&self) -> Vec<f64> {
        self.inner.times.clone()
    }

    #[getter]
    fn positions(&self) -> Vec<Vec<f64>> {
        self.inner.positions.clone()
    }

    #[getter]
    fn velocities(&self) -> Vec<Vec<f64>> {
        self.inner.velocities.clone()
    }

    #[getter]
    fn duration(&self) -> f64 {
        self.inner.duration()
    }

    /// Joint positions at time `t` (cubic Hermite, clamped to the span).
    fn sample(&self, t: f64) -> Vec<f64> {
        self.inner.sample(t)
    }

    /// Writes `{joint_names, times, positions, velocities}` as JSON.
    fn export_json(&self, path: PathBuf) -> PyResult<()> {
        let payload = serde_json::json!({
            "joint_names": self.joint_names,
            "times": self.inner.times,
            "positions": self.inner.positions,
            "velocities": self.inner.velocities,
        });
        std::fs::write(&path, serde_json::to_string_pretty(&payload).expect("json"))
            .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))
    }

    /// Writes a CSV with header `t,<joint...>`. With `dt` the trajectory is
    /// resampled uniformly; otherwise the internal waypoints are written.
    #[pyo3(signature = (path, dt = None))]
    fn export_csv(&self, path: PathBuf, dt: Option<f64>) -> PyResult<()> {
        let (times, positions) = match dt {
            Some(dt) if dt > 0.0 => self.inner.resample(dt),
            Some(_) => return Err(PyValueError::new_err("dt must be positive")),
            None => (self.inner.times.clone(), self.inner.positions.clone()),
        };
        let mut out = String::new();
        out.push('t');
        for name in &self.joint_names {
            out.push(',');
            out.push_str(name);
        }
        out.push('\n');
        for (t, q) in times.iter().zip(&positions) {
            out.push_str(&format!("{t:.6}"));
            for v in q {
                out.push_str(&format!(",{v:.6}"));
            }
            out.push('\n');
        }
        std::fs::write(&path, out)
            .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))
    }

    /// Renders the planned motion as a vendor robot script and returns it
    /// as a string. The script replays the sparse planned waypoints as
    /// vendor move commands (`segments`); time parameterization is left to
    /// the robot controller. Speeds derive from the joint limits scaled by
    /// `speed_scale`; `blend_radius` (m) rounds intermediate waypoints
    /// (keep 0 unless verified on the controller — overlapping blends
    /// abort some controllers); linear-move speed is `tcp_speed` (m/s).
    /// With `move_to_start` the program begins with a joint move to the
    /// first waypoint. Currently supported dialects: "urscript".
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (dialect = "urscript", name = "botrail_program", speed_scale = 1.0,
                        blend_radius = 0.0, tcp_speed = 0.25, tcp_accel = 1.2,
                        move_to_start = true))]
    fn to_script(
        &self,
        dialect: &str,
        name: &str,
        speed_scale: f64,
        blend_radius: f64,
        tcp_speed: f64,
        tcp_accel: f64,
        move_to_start: bool,
    ) -> PyResult<String> {
        self.render_script(
            dialect,
            name,
            speed_scale,
            blend_radius,
            tcp_speed,
            tcp_accel,
            move_to_start,
        )
    }

    /// Writes `to_script` output to `path`. The program name defaults to
    /// the file stem (e.g. `pick.script` → `def pick():`).
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (path, dialect = "urscript", name = None, speed_scale = 1.0,
                        blend_radius = 0.0, tcp_speed = 0.25, tcp_accel = 1.2,
                        move_to_start = true))]
    fn export_script(
        &self,
        path: PathBuf,
        dialect: &str,
        name: Option<&str>,
        speed_scale: f64,
        blend_radius: f64,
        tcp_speed: f64,
        tcp_accel: f64,
        move_to_start: bool,
    ) -> PyResult<()> {
        let stem = path
            .file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default();
        let script = self.render_script(
            dialect,
            name.unwrap_or(&stem),
            speed_scale,
            blend_radius,
            tcp_speed,
            tcp_accel,
            move_to_start,
        )?;
        std::fs::write(&path, script)
            .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))
    }

    fn __repr__(&self) -> String {
        format!(
            "Trajectory(duration={:.3}s, waypoints={}, dof={})",
            self.inner.duration(),
            self.inner.times.len(),
            self.inner.dof()
        )
    }
}

/// Handle to a running studio server. Dropping it (or calling `stop`)
/// shuts the server down.
#[pyclass(module = "botrail._core")]
struct StudioServer {
    url: String,
    shutdown: Option<tokio::sync::oneshot::Sender<()>>,
    thread: Option<std::thread::JoinHandle<()>>,
}

#[pymethods]
impl StudioServer {
    #[getter]
    fn url(&self) -> String {
        self.url.clone()
    }

    fn stop(&mut self) {
        if let Some(tx) = self.shutdown.take() {
            let _ = tx.send(());
        }
        if let Some(handle) = self.thread.take() {
            let _ = handle.join();
        }
    }

    fn __repr__(&self) -> String {
        format!("StudioServer(url='{}')", self.url)
    }
}

impl Drop for StudioServer {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Starts the studio server on a background thread and returns immediately.
/// `port = 0` picks a free port.
#[pyfunction]
#[pyo3(signature = (scene, studio_dir, host = "127.0.0.1", port = 0))]
fn serve_studio(
    scene: &Scene,
    studio_dir: PathBuf,
    host: &str,
    port: u16,
) -> PyResult<StudioServer> {
    let listener = std::net::TcpListener::bind((host, port))
        .map_err(|e| PyIOError::new_err(format!("failed to bind {host}:{port}: {e}")))?;
    listener
        .set_nonblocking(true)
        .map_err(|e| PyIOError::new_err(e.to_string()))?;
    let addr = listener
        .local_addr()
        .map_err(|e| PyIOError::new_err(e.to_string()))?;

    let hub = scene.hub.clone();
    let (shutdown_tx, shutdown_rx) = tokio::sync::oneshot::channel::<()>();
    let thread = std::thread::Builder::new()
        .name("botrail-studio".to_string())
        .spawn(move || {
            let runtime = tokio::runtime::Builder::new_multi_thread()
                .worker_threads(2)
                .enable_all()
                .build()
                .expect("failed to build tokio runtime");
            runtime.block_on(async move {
                let listener = tokio::net::TcpListener::from_std(listener)
                    .expect("listener is already bound and non-blocking");
                let app = server::router(hub, studio_dir);
                let serve = axum::serve(listener, app).with_graceful_shutdown(async {
                    let _ = shutdown_rx.await;
                });
                if let Err(e) = serve.await {
                    eprintln!("botrail: studio server error: {e}");
                }
            });
        })
        .map_err(|e| PyIOError::new_err(e.to_string()))?;

    let display_host = if host == "0.0.0.0" { "127.0.0.1" } else { host };
    Ok(StudioServer {
        url: format!("http://{display_host}:{}", addr.port()),
        shutdown: Some(shutdown_tx),
        thread: Some(thread),
    })
}

/// A baked sequence rollout: per-robot joint tracks, grasped-object
/// motion, signal waveforms, and step spans (the timing chart).
#[pyclass(frozen, module = "botrail._core")]
struct SequenceTimeline {
    inner: botrail_scene::rollout::SequenceTimeline,
    /// The pre-rollout snapshot the timeline was baked against (FK model,
    /// base pose, obstacle geometry for USD export).
    scene: botrail_scene::Scene,
}

impl SequenceTimeline {
    /// Resolves an optional robot name to `(scene index, track)`. `None`
    /// means the sole robot and is ambiguous when several exist.
    fn track_for(
        &self,
        robot: Option<&str>,
    ) -> PyResult<(usize, &botrail_scene::rollout::RobotTrack)> {
        let names = || {
            self.inner
                .robots
                .iter()
                .map(|r| format!("`{}`", r.name))
                .collect::<Vec<_>>()
                .join(", ")
        };
        match robot {
            Some(name) => self
                .inner
                .robots
                .iter()
                .position(|r| r.name == name)
                .map(|i| (i, &self.inner.robots[i]))
                .ok_or_else(|| {
                    PyValueError::new_err(format!("unknown robot `{name}` (robots: {})", names()))
                }),
            None if self.inner.robots.len() == 1 => Ok((0, &self.inner.robots[0])),
            None => Err(PyValueError::new_err(format!(
                "the timeline has {} robots; pass robot=<name> (one of: {})",
                self.inner.robots.len(),
                names()
            ))),
        }
    }

    /// Every robot's FK world poses at time `t`.
    fn all_poses_at(&self, t: f64) -> PyResult<Vec<Vec<nalgebra::Isometry3<f64>>>> {
        self.inner
            .robots
            .iter()
            .enumerate()
            .map(|(r, track)| {
                let q = track.trajectory.sample(t);
                // A robot riding a vehicle carries its base in the track.
                match botrail_scene::rollout::SequenceTimeline::base_pose(track, t) {
                    Some(base) => botrail_kin::forward_kinematics_with_base(
                        &self.scene.robots()[r].model,
                        &q,
                        &base,
                    )
                    .map_err(|e| PyValueError::new_err(e.to_string())),
                    None => self
                        .scene
                        .fk_for(r, &q)
                        .map_err(|e| PyValueError::new_err(e.to_string())),
                }
            })
            .collect()
    }
}

#[pymethods]
impl SequenceTimeline {
    /// Cycle time in seconds.
    #[getter]
    fn duration(&self) -> f64 {
        self.inner.duration
    }

    /// Instance names of the robots on this timeline, in scene order.
    #[getter]
    fn robots(&self) -> Vec<String> {
        self.inner.robots.iter().map(|r| r.name.clone()).collect()
    }

    /// `(step name, start, end)` per step, in execution order.
    #[getter]
    fn step_spans(&self) -> Vec<(String, f64, f64)> {
        self.inner
            .step_spans
            .iter()
            .map(|s| (s.name.clone(), s.start, s.end))
            .collect()
    }

    /// Signal waveforms as `(name, [(time, value), ...])` edge lists.
    #[getter]
    fn signals(&self) -> Vec<(String, Vec<(f64, bool)>)> {
        self.inner
            .signals
            .iter()
            .map(|s| (s.name.clone(), s.edges.clone()))
            .collect()
    }

    /// A robot's joint positions at time `t` (clamped to the cycle).
    #[pyo3(signature = (t, robot = None))]
    fn sample(&self, t: f64, robot: Option<&str>) -> PyResult<Vec<f64>> {
        Ok(self.track_for(robot)?.1.trajectory.sample(t))
    }

    /// A robot's move intervals as `(label, start, end)` — the intervals a
    /// motion (by name) or ramp drove it.
    #[pyo3(signature = (robot = None))]
    fn moves(&self, robot: Option<&str>) -> PyResult<Vec<(String, f64, f64)>> {
        Ok(self
            .track_for(robot)?
            .1
            .moves
            .iter()
            .map(|s| (s.name.clone(), s.start, s.end))
            .collect())
    }

    /// Seconds a robot spent in motion (overlapping move intervals merged).
    #[pyo3(signature = (robot = None))]
    fn busy_seconds(&self, robot: Option<&str>) -> PyResult<f64> {
        let (_, track) = self.track_for(robot)?;
        Ok(self.inner.busy_seconds(&track.name).unwrap_or(0.0))
    }

    /// Fraction of the cycle a robot spent moving, 0..1 — the
    /// line-balancing number. The bottleneck is whoever sits near 1, and
    /// this is what predicts where moving a spot lands the takt.
    #[pyo3(signature = (robot = None))]
    fn utilization(&self, robot: Option<&str>) -> PyResult<f64> {
        let (_, track) = self.track_for(robot)?;
        Ok(self.inner.utilization(&track.name).unwrap_or(0.0))
    }

    /// `{robot: utilization}` for every robot on the timeline.
    fn utilizations(&self) -> std::collections::HashMap<String, f64> {
        self.inner
            .robots
            .iter()
            .map(|r| {
                (
                    r.name.clone(),
                    self.inner.utilization(&r.name).unwrap_or(0.0),
                )
            })
            .collect()
    }

    /// Where a mounted robot's base was at time `t` — `None` for a robot
    /// bolted to the floor, whose base is a scene constant.
    #[pyo3(signature = (t, robot = None))]
    fn base_pose(&self, t: f64, robot: Option<&str>) -> PyResult<Option<([f64; 3], [f64; 4])>> {
        let (_, track) = self.track_for(robot)?;
        let Some(pose) = botrail_scene::rollout::SequenceTimeline::base_pose(track, t) else {
            return Ok(None);
        };
        let q = pose.rotation.coords;
        Ok(Some((
            [pose.translation.x, pose.translation.y, pose.translation.z],
            [q.x, q.y, q.z, q.w],
        )))
    }

    /// World pose of a grasped/tracked object at time `t`.
    fn object_pose(&self, name: &str, t: f64) -> PyResult<([f64; 3], [f64; 4])> {
        let track = self
            .inner
            .objects
            .iter()
            .find(|o| o.name == name)
            .ok_or_else(|| {
                PyValueError::new_err(format!("`{name}` is not tracked by this timeline"))
            })?;
        let poses = self.all_poses_at(t)?;
        let pose = botrail_scene::rollout::SequenceTimeline::object_pose(track, &poses, t)
            .ok_or_else(|| PyValueError::new_err("empty object track"))?;
        let q = pose.rotation.coords;
        Ok((
            [pose.translation.x, pose.translation.y, pose.translation.z],
            [q.x, q.y, q.z, q.w],
        ))
    }

    /// Whether a tracked object should be drawn at `t`. False only while
    /// it is stowed — waiting in a magazine, or taken off the line.
    fn object_visible(&self, name: &str, t: f64) -> PyResult<bool> {
        let track = self
            .inner
            .objects
            .iter()
            .find(|o| o.name == name)
            .ok_or_else(|| {
                PyValueError::new_err(format!("`{name}` is not tracked by this timeline"))
            })?;
        Ok(botrail_scene::rollout::SequenceTimeline::object_visible(
            track, t,
        ))
    }

    /// Carves `stock` with the cutter swept along this cycle: a voxel
    /// subtraction in the stock's frame, returning the machined part as
    /// a mesh plus removed/remaining volume. Presentation and numbers —
    /// the cut can never contradict the plan in a kinematic world.
    #[pyo3(signature = (stock, robot = None, tcp_link = None, voxel_size = 0.001, cutter_radius = 0.004, cutter_length = 0.03, dt = 0.01))]
    #[allow(clippy::too_many_arguments)]
    fn carve_stock(
        &self,
        stock: &str,
        robot: Option<&str>,
        tcp_link: Option<&str>,
        voxel_size: f64,
        cutter_radius: f64,
        cutter_length: f64,
        dt: f64,
    ) -> PyResult<StockCarve> {
        let (index, _) = self.track_for(robot)?;
        let model = &self.scene.robots()[index].model;
        let tcp = match tcp_link {
            Some(l) => resolve_link(&std::sync::Arc::clone(model), Some(l))?,
            None => model.default_tcp_link(),
        };
        let options = botrail_scene::carve::CarveOptions {
            voxel_size,
            cutter_radius,
            cutter_length,
            dt,
            ..botrail_scene::carve::CarveOptions::default()
        };
        let inner = botrail_scene::carve::carve_stock(
            &self.scene,
            &self.inner,
            stock,
            index,
            tcp,
            &options,
        )
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(StockCarve { inner })
    }

    /// Feed adherence of a toolpath the cycle ran (`StartToolpath`).
    /// `toolpath=None` means the only one; name it when the cycle cuts
    /// several.
    #[pyo3(signature = (toolpath = None))]
    fn feed_report(&self, toolpath: Option<&str>) -> PyResult<FeedReport> {
        let mut found: Vec<(&str, usize, &botrail_scene::toolpath::FeedReport)> = Vec::new();
        for (r, track) in self.inner.robots.iter().enumerate() {
            for planned in &track.planned {
                if let (Some(name), Some(report)) = (&planned.motion, &planned.feed_report) {
                    if toolpath.is_none_or(|t| t == name) {
                        found.push((name, r, report));
                    }
                }
            }
        }
        match (found.len(), toolpath) {
            (0, Some(name)) => Err(PyValueError::new_err(format!(
                "the cycle ran no toolpath named `{name}`"
            ))),
            (0, None) => Err(PyValueError::new_err(
                "the cycle ran no toolpath (start one with bt.seq.toolpath)",
            )),
            (1, _) => {
                let (_, r, report) = found[0];
                Ok(FeedReport {
                    inner: report.clone(),
                    joint_names: self.scene.robots()[r]
                        .model
                        .actuated_joint_names()
                        .iter()
                        .map(|n| n.to_string())
                        .collect(),
                })
            }
            (_, None) => Err(PyValueError::new_err(format!(
                "the cycle ran {} toolpath moves; name one (candidates: {})",
                found.len(),
                found
                    .iter()
                    .map(|(n, _, _)| format!("`{n}`"))
                    .collect::<Vec<_>>()
                    .join(", ")
            ))),
            (_, Some(_)) => {
                // The same toolpath started twice: report the first run.
                let (_, r, report) = found[0];
                Ok(FeedReport {
                    inner: report.clone(),
                    joint_names: self.scene.robots()[r]
                        .model
                        .actuated_joint_names()
                        .iter()
                        .map(|n| n.to_string())
                        .collect(),
                })
            }
        }
    }

    /// A robot's cycle joint track as a [`Trajectory`] (CSV/JSON export,
    /// joint access). Step boundaries land in `segment_ends`.
    #[pyo3(signature = (robot = None))]
    fn robot_trajectory(&self, robot: Option<&str>) -> PyResult<Trajectory> {
        let (index, track) = self.track_for(robot)?;
        let model = &self.scene.robots()[index].model;
        Ok(Trajectory {
            inner: track.trajectory.clone(),
            joint_names: model
                .actuated_joint_names()
                .iter()
                .map(|n| n.to_string())
                .collect(),
            segment_ends: self.inner.step_spans.iter().map(|s| s.end).collect(),
            segments: Vec::new(),
            feed_report: None,
            limits: crate::hub::traj_limits(model),
        })
    }

    /// The sole robot's cycle track (see `robot_trajectory`; with several
    /// robots this is ambiguous — name one).
    #[getter]
    fn trajectory(&self) -> PyResult<Trajectory> {
        self.robot_trajectory(None)
    }

    /// Bakes the whole cycle to a USD animation layer (see
    /// `Scene.export_usd`): every robot + every obstacle, with grasped
    /// objects riding, releasing, resting — and handed over — exactly as
    /// simulated. A sole robot exports under the historical `Robot` prim;
    /// with several, each lands at `/World/<sanitized instance name>`.
    /// The extension picks the serialization: `.usda` text, `.usdc`/`.usd`
    /// binary crate at roughly half the size.
    ///
    /// `start`/`end` clip the export to a window of the cycle. A line's
    /// full run is mostly repetition — one steady-state takt carries the
    /// whole story at a fraction of the bytes, which is what makes a
    /// line recording shippable at all.
    #[pyo3(signature = (path, fps = 60.0, start = None, end = None))]
    fn export_usd(
        &self,
        path: PathBuf,
        fps: f64,
        start: Option<f64>,
        end: Option<f64>,
    ) -> PyResult<Vec<String>> {
        let stem = path
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| "animation".to_string());
        let exported =
            botrail_session::usd::bake_timeline(&self.scene, &self.inner, fps, start, end, &stem)
                .map_err(PyValueError::new_err)?;
        botrail_usd::export::write_exported(&path, exported)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    /// The baked interval of the named step (assertion-friendly view of
    /// one `step_spans` row).
    fn step_span(&self, name: &str) -> PyResult<Span> {
        self.inner
            .step_spans
            .iter()
            .find(|s| s.name == name)
            .map(|s| Span {
                name: s.name.clone(),
                start: s.start,
                end: s.end,
            })
            .ok_or_else(|| {
                PyValueError::new_err(format!(
                    "unknown step `{name}` (steps: {})",
                    self.inner
                        .step_spans
                        .iter()
                        .map(|s| format!("`{}`", s.name))
                        .collect::<Vec<_>>()
                        .join(", ")
                ))
            })
    }

    /// The named waveform lane — an internal signal, a sensor, or a
    /// device's running state.
    fn signal(&self, name: &str) -> PyResult<SignalTrack> {
        self.inner
            .signals
            .iter()
            .find(|s| s.name == name)
            .map(|s| SignalTrack {
                inner: s.clone(),
                duration: self.inner.duration,
            })
            .ok_or_else(|| {
                PyValueError::new_err(format!(
                    "unknown signal `{name}` (lanes: {})",
                    self.inner
                        .signals
                        .iter()
                        .map(|s| format!("`{}`", s.name))
                        .collect::<Vec<_>>()
                        .join(", ")
                ))
            })
    }

    /// The tightest robot-to-environment approach over the cycle, sampled
    /// every `dt` seconds against the scene the timeline was baked from
    /// (carried and conveyed objects replay their baked motion; robot-robot
    /// contact is already a hard rollout error). Raises when the cell has
    /// nothing to measure.
    #[pyo3(signature = (dt = 0.01))]
    fn min_clearance(&self, dt: f64) -> PyResult<Clearance> {
        if !(dt.is_finite() && dt > 0.0) {
            return Err(PyValueError::new_err(format!(
                "dt must be positive, got {dt}"
            )));
        }
        self.scene
            .timeline_min_clearance(&self.inner, dt)
            .map_err(|e| PyValueError::new_err(e.to_string()))?
            .map(|inner| Clearance { inner })
            .ok_or_else(|| {
                PyValueError::new_err(
                    "nothing to measure: the cell has no enabled environment \
                     obstacle with collision geometry",
                )
            })
    }

    /// Names of the sequences this timeline was rolled from, in scan
    /// order — the programs `to_script` can export.
    #[getter]
    fn sequences(&self) -> Vec<String> {
        self.inner.sequences.clone()
    }

    /// The scenario this bake ran under; `None` is the unmodified scene
    /// (`baseline`).
    #[getter]
    fn scenario(&self) -> Option<String> {
        self.inner.scenario.clone()
    }

    /// The path the bake took through branching steps, in resolution
    /// order: `(sequence, step name, arm index)`. Untaken arms have no
    /// spans — this is how a timeline says which way it went.
    #[getter]
    fn branches(&self) -> Vec<(String, String, usize)> {
        self.inner
            .branches
            .iter()
            .map(|b| (b.sequence.clone(), b.step.clone(), b.arm))
            .collect()
    }

    /// Renders one program of this timeline as a vendor robot script —
    /// the same steps that drove the simulation, with real I/O: `inputs`
    /// maps signal/device/robot names to digital input ports (level
    /// waits), `outputs` maps signal/device names to digital output ports
    /// (coil writes). Timers become sleeps; moves are the rollout's own
    /// planned sparse paths.
    ///
    /// The program is named after the sequence (`name` overrides). The
    /// sequence must drive exactly one robot — a multi-robot cell exports
    /// one script per program. Approximations (unmapped device commands,
    /// waits that ran beside a move in simulation) are raised as Python
    /// warnings; what cannot be expressed at all (`any_of` waits,
    /// conveyor tracking) raises `ValueError`.
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (sequence = None, dialect = "urscript", name = None,
        inputs = None, outputs = None, speed_scale = 1.0, blend_radius = 0.0,
        tcp_speed = 0.25, tcp_accel = 1.2, move_to_start = true))]
    fn to_script(
        &self,
        py: Python<'_>,
        sequence: Option<&str>,
        dialect: &str,
        name: Option<&str>,
        inputs: Option<std::collections::HashMap<String, u32>>,
        outputs: Option<std::collections::HashMap<String, u32>>,
        speed_scale: f64,
        blend_radius: f64,
        tcp_speed: f64,
        tcp_accel: f64,
        move_to_start: bool,
    ) -> PyResult<String> {
        let backend = botrail_export::backend(dialect).ok_or_else(|| {
            PyValueError::new_err(format!(
                "unknown dialect {dialect:?} (available: {})",
                botrail_export::DIALECTS.join(", ")
            ))
        })?;
        let io = botrail_scene::script::SequenceIo {
            inputs: inputs.unwrap_or_default(),
            outputs: outputs.unwrap_or_default(),
        };
        let options = botrail_export::ProgramOptions {
            speed_scale,
            blend_radius,
            tcp_speed,
            tcp_accel,
            move_to_start,
        };
        let out = botrail_scene::script::sequence_program(
            &self.scene,
            &self.inner,
            sequence,
            &io,
            &options,
        )
        .map_err(PyValueError::new_err)?;
        for warning in &out.warnings {
            let message = std::ffi::CString::new(warning.as_str())
                .unwrap_or_else(|_| std::ffi::CString::new("sequence export warning").unwrap());
            PyErr::warn(
                py,
                &py.get_type::<pyo3::exceptions::PyUserWarning>(),
                &message,
                2,
            )?;
        }
        let mut program = out.program;
        if let Some(name) = name {
            program.name = name.to_string();
        }
        backend
            .emit(&program)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    /// Writes `to_script` output to `path` (see there for the semantics).
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (path, sequence = None, dialect = "urscript", name = None,
        inputs = None, outputs = None, speed_scale = 1.0, blend_radius = 0.0,
        tcp_speed = 0.25, tcp_accel = 1.2, move_to_start = true))]
    fn export_script(
        &self,
        py: Python<'_>,
        path: PathBuf,
        sequence: Option<&str>,
        dialect: &str,
        name: Option<&str>,
        inputs: Option<std::collections::HashMap<String, u32>>,
        outputs: Option<std::collections::HashMap<String, u32>>,
        speed_scale: f64,
        blend_radius: f64,
        tcp_speed: f64,
        tcp_accel: f64,
        move_to_start: bool,
    ) -> PyResult<()> {
        let script = self.to_script(
            py,
            sequence,
            dialect,
            name,
            inputs,
            outputs,
            speed_scale,
            blend_radius,
            tcp_speed,
            tcp_accel,
            move_to_start,
        )?;
        std::fs::write(&path, script)
            .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))
    }
}

/// A scenario sweep's results: one deterministic timeline per scenario
/// that completed, per-scenario failures, and branch coverage over the
/// whole set — the cell's test-case matrix as one object.
#[pyclass(frozen, module = "botrail._core")]
struct ScenarioRuns {
    /// `(scenario, timeline)` in run order (`baseline` first by default).
    runs: Vec<(String, Py<SequenceTimeline>)>,
    failures: Vec<(String, String)>,
    /// Authored-content snapshot (sequences for coverage).
    scene: botrail_scene::Scene,
}

#[pymethods]
impl ScenarioRuns {
    /// Scenarios that completed, in run order.
    #[getter]
    fn names(&self) -> Vec<String> {
        self.runs.iter().map(|(name, _)| name.clone()).collect()
    }

    /// `{scenario: error}` for the runs that failed — a bad delta, a plan
    /// failure, or a timeout, with the rollout's own diagnosis.
    #[getter]
    fn errors<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let out = PyDict::new(py);
        for (name, error) in &self.failures {
            out.set_item(name, error)?;
        }
        Ok(out)
    }

    /// `{scenario: cycle time}` for the completed runs.
    #[getter]
    fn durations<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let out = PyDict::new(py);
        for (name, timeline) in &self.runs {
            out.set_item(name, timeline.borrow(py).inner.duration)?;
        }
        Ok(out)
    }

    fn __len__(&self) -> usize {
        self.runs.len()
    }

    fn __contains__(&self, name: &str) -> bool {
        self.runs.iter().any(|(n, _)| n == name)
    }

    fn __getitem__(&self, py: Python<'_>, name: &str) -> PyResult<Py<SequenceTimeline>> {
        if let Some((_, timeline)) = self.runs.iter().find(|(n, _)| n == name) {
            return Ok(timeline.clone_ref(py));
        }
        if let Some((_, error)) = self.failures.iter().find(|(n, _)| n == name) {
            return Err(pyo3::exceptions::PyKeyError::new_err(format!(
                "scenario `{name}` failed: {error}"
            )));
        }
        Err(pyo3::exceptions::PyKeyError::new_err(format!(
            "unknown scenario `{name}` (ran: {})",
            self.names().join(", ")
        )))
    }

    /// `(scenario, timeline)` pairs in run order.
    fn items(&self, py: Python<'_>) -> Vec<(String, Py<SequenceTimeline>)> {
        self.runs
            .iter()
            .map(|(name, timeline)| (name.clone(), timeline.clone_ref(py)))
            .collect()
    }

    /// Branch arms *no* run in this set took, as `(sequence, step, arm,
    /// condition)` rows in authoring order — empty means every authored
    /// path was exercised. The condition is the arm's guard in the
    /// authoring vocabulary, so each row says which scenario is missing.
    fn uncovered_arms(&self, py: Python<'_>) -> PyResult<Vec<(String, String, usize, String)>> {
        let borrowed: Vec<PyRef<'_, SequenceTimeline>> =
            self.runs.iter().map(|(_, t)| t.borrow(py)).collect();
        let timelines: Vec<&botrail_scene::rollout::SequenceTimeline> =
            borrowed.iter().map(|t| &t.inner).collect();
        let uncovered = botrail_scene::rollout::arm_coverage(&self.scene, &timelines)
            .map_err(PyValueError::new_err)?;
        Ok(uncovered
            .into_iter()
            .map(|u| (u.sequence, u.step, u.arm, u.condition))
            .collect())
    }

    /// `{scenario: Clearance}` — the tightest robot-to-environment
    /// approach of every completed run, so the whole matrix (skipped-arm
    /// paths included, via the scenarios that take them) gets margin
    /// checked, not just the happy path.
    #[pyo3(signature = (dt = 0.01))]
    fn min_clearances<'py>(&self, py: Python<'py>, dt: f64) -> PyResult<Bound<'py, PyDict>> {
        if !(dt.is_finite() && dt > 0.0) {
            return Err(PyValueError::new_err(format!(
                "dt must be positive, got {dt}"
            )));
        }
        let out = PyDict::new(py);
        for (name, timeline) in &self.runs {
            let timeline = timeline.borrow(py);
            let clearance = timeline
                .scene
                .timeline_min_clearance(&timeline.inner, dt)
                .map_err(|e| PyValueError::new_err(e.to_string()))?
                .map(|inner| Clearance { inner })
                .ok_or_else(|| {
                    PyValueError::new_err(
                        "nothing to measure: the cell has no enabled environment \
                         obstacle with collision geometry",
                    )
                })?;
            out.set_item(name, clearance.into_pyobject(py)?)?;
        }
        Ok(out)
    }

    /// Renders one program from the whole sweep as a vendor robot script
    /// — every branch arm included: the primary run (default: the first,
    /// i.e. `baseline`) supplies the shared path, and an arm it skipped
    /// is spliced in from the run that took it. That splice is refused
    /// when the donor reached the branch at a different configuration
    /// (the scenario changed the path *before* the branch), and a
    /// straight-line move right after arms that rejoin apart raises a
    /// warning — see `SequenceTimeline.to_script` for the I/O wiring and
    /// the rest of the semantics.
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (sequence = None, dialect = "urscript", name = None, primary = None,
        inputs = None, outputs = None, speed_scale = 1.0, blend_radius = 0.0,
        tcp_speed = 0.25, tcp_accel = 1.2, move_to_start = true))]
    fn to_script(
        &self,
        py: Python<'_>,
        sequence: Option<&str>,
        dialect: &str,
        name: Option<&str>,
        primary: Option<&str>,
        inputs: Option<std::collections::HashMap<String, u32>>,
        outputs: Option<std::collections::HashMap<String, u32>>,
        speed_scale: f64,
        blend_radius: f64,
        tcp_speed: f64,
        tcp_accel: f64,
        move_to_start: bool,
    ) -> PyResult<String> {
        if self.runs.is_empty() {
            return Err(PyValueError::new_err(
                "no completed runs to export (every scenario failed — see .errors)",
            ));
        }
        let backend = botrail_export::backend(dialect).ok_or_else(|| {
            PyValueError::new_err(format!(
                "unknown dialect {dialect:?} (available: {})",
                botrail_export::DIALECTS.join(", ")
            ))
        })?;
        let lead = match primary {
            Some(primary) => self
                .runs
                .iter()
                .position(|(n, _)| n == primary)
                .ok_or_else(|| {
                    PyValueError::new_err(format!(
                        "unknown primary scenario `{primary}` (ran: {})",
                        self.names().join(", ")
                    ))
                })?,
            None => 0,
        };
        let io = botrail_scene::script::SequenceIo {
            inputs: inputs.unwrap_or_default(),
            outputs: outputs.unwrap_or_default(),
        };
        let options = botrail_export::ProgramOptions {
            speed_scale,
            blend_radius,
            tcp_speed,
            tcp_accel,
            move_to_start,
        };
        let borrowed: Vec<PyRef<'_, SequenceTimeline>> =
            self.runs.iter().map(|(_, t)| t.borrow(py)).collect();
        let mut ordered: Vec<(&str, &botrail_scene::rollout::SequenceTimeline)> =
            vec![(self.runs[lead].0.as_str(), &borrowed[lead].inner)];
        for (i, (scenario, _)) in self.runs.iter().enumerate() {
            if i != lead {
                ordered.push((scenario.as_str(), &borrowed[i].inner));
            }
        }
        let out = botrail_scene::script::merged_sequence_program(
            &self.scene,
            &ordered,
            sequence,
            &io,
            &options,
        )
        .map_err(PyValueError::new_err)?;
        for warning in &out.warnings {
            let message = std::ffi::CString::new(warning.as_str())
                .unwrap_or_else(|_| std::ffi::CString::new("sequence export warning").unwrap());
            PyErr::warn(
                py,
                &py.get_type::<pyo3::exceptions::PyUserWarning>(),
                &message,
                2,
            )?;
        }
        let mut program = out.program;
        if let Some(name) = name {
            program.name = name.to_string();
        }
        backend
            .emit(&program)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    /// Writes `to_script` output to `path` (see there for the semantics).
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (path, sequence = None, dialect = "urscript", name = None,
        primary = None, inputs = None, outputs = None, speed_scale = 1.0,
        blend_radius = 0.0, tcp_speed = 0.25, tcp_accel = 1.2, move_to_start = true))]
    fn export_script(
        &self,
        py: Python<'_>,
        path: PathBuf,
        sequence: Option<&str>,
        dialect: &str,
        name: Option<&str>,
        primary: Option<&str>,
        inputs: Option<std::collections::HashMap<String, u32>>,
        outputs: Option<std::collections::HashMap<String, u32>>,
        speed_scale: f64,
        blend_radius: f64,
        tcp_speed: f64,
        tcp_accel: f64,
        move_to_start: bool,
    ) -> PyResult<()> {
        let script = self.to_script(
            py,
            sequence,
            dialect,
            name,
            primary,
            inputs,
            outputs,
            speed_scale,
            blend_radius,
            tcp_speed,
            tcp_accel,
            move_to_start,
        )?;
        std::fs::write(&path, script)
            .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))
    }

    fn __repr__(&self) -> String {
        format!(
            "ScenarioRuns({} run{}, {} failure{})",
            self.runs.len(),
            if self.runs.len() == 1 { "" } else { "s" },
            self.failures.len(),
            if self.failures.len() == 1 { "" } else { "s" },
        )
    }
}

/// One step's baked interval on a timeline.
#[pyclass(frozen, module = "botrail._core")]
#[derive(Clone)]
struct Span {
    name: String,
    start: f64,
    end: f64,
}

#[pymethods]
impl Span {
    /// Step name.
    #[getter]
    fn name(&self) -> String {
        self.name.clone()
    }

    /// Start time in seconds.
    #[getter]
    fn start(&self) -> f64 {
        self.start
    }

    /// End time in seconds.
    #[getter]
    fn end(&self) -> f64 {
        self.end
    }

    /// `end - start`.
    #[getter]
    fn duration(&self) -> f64 {
        self.end - self.start
    }

    fn __repr__(&self) -> String {
        format!(
            "Span('{}', {:.3}s..{:.3}s)",
            self.name, self.start, self.end
        )
    }
}

/// A signal/sensor/device waveform lane on a baked timeline.
#[pyclass(frozen, module = "botrail._core")]
#[derive(Clone)]
struct SignalTrack {
    inner: botrail_scene::rollout::BoolTrack,
    duration: f64,
}

#[pymethods]
impl SignalTrack {
    /// Lane name.
    #[getter]
    fn name(&self) -> String {
        self.inner.name.clone()
    }

    /// `(time, new value)` edges, starting with `(0, initial)`.
    #[getter]
    fn edges(&self) -> Vec<(f64, bool)> {
        self.inner.edges.clone()
    }

    /// The level at time `t`.
    fn value_at(&self, t: f64) -> bool {
        self.inner.value_at(t)
    }

    /// Times the lane turns ON (the initial level at 0 is not an edge).
    fn rising_edges(&self) -> Vec<f64> {
        self.inner
            .edges
            .windows(2)
            .filter(|w| !w[0].1 && w[1].1)
            .map(|w| w[1].0)
            .collect()
    }

    /// Times the lane turns OFF.
    fn falling_edges(&self) -> Vec<f64> {
        self.inner
            .edges
            .windows(2)
            .filter(|w| w[0].1 && !w[1].1)
            .map(|w| w[1].0)
            .collect()
    }

    /// `(start, end)` intervals the lane is ON; an interval still open at
    /// the cycle end closes at `duration`.
    fn high_spans(&self) -> Vec<(f64, f64)> {
        let mut spans = Vec::new();
        let mut on_since: Option<f64> = None;
        for &(t, v) in &self.inner.edges {
            match (on_since, v) {
                (None, true) => on_since = Some(t),
                (Some(t0), false) => {
                    spans.push((t0, t));
                    on_since = None;
                }
                _ => {}
            }
        }
        if let Some(t0) = on_since {
            spans.push((t0, self.duration));
        }
        spans
    }

    /// Total ON time over the cycle.
    fn high_total(&self) -> f64 {
        self.high_spans().iter().map(|(a, b)| b - a).sum()
    }

    fn __repr__(&self) -> String {
        format!(
            "SignalTrack('{}', {} rising, on {:.3}s of {:.3}s)",
            self.inner.name,
            self.rising_edges().len(),
            self.high_total(),
            self.duration
        )
    }
}

/// The tightest robot-to-environment approach on a timeline. Compares and
/// converts like its `distance`, so `assert tl.min_clearance() >= 0.005`
/// reads directly — and its repr names the time (and touching pair) when
/// the assertion fires.
#[pyclass(frozen, module = "botrail._core")]
#[derive(Clone)]
struct Clearance {
    inner: botrail_scene::verify::Clearance,
}

#[pymethods]
impl Clearance {
    /// Distance in meters (0 while touching).
    #[getter]
    fn distance(&self) -> f64 {
        self.inner.distance
    }

    /// When it first happens (seconds on the timeline).
    #[getter]
    fn t(&self) -> f64 {
        self.inner.t
    }

    /// The touching `(robot side, obstacle)` names while in contact.
    #[getter]
    fn pair(&self) -> Option<(String, String)> {
        self.inner.pair.clone()
    }

    fn __float__(&self) -> f64 {
        self.inner.distance
    }

    fn __richcmp__(&self, other: &Bound<'_, PyAny>, op: pyo3::basic::CompareOp) -> PyResult<bool> {
        let value = if let Ok(c) = other.downcast::<Clearance>() {
            c.get().inner.distance
        } else {
            other.extract::<f64>()?
        };
        let ordering = self
            .inner
            .distance
            .partial_cmp(&value)
            .ok_or_else(|| PyValueError::new_err("cannot compare a NaN clearance"))?;
        Ok(op.matches(ordering))
    }

    fn __repr__(&self) -> String {
        match &self.inner.pair {
            Some((a, b)) => format!("Clearance(contact at t={:.3}s: {a} <-> {b})", self.inner.t),
            None if self.inner.distance <= 0.0 => {
                format!("Clearance(contact at t={:.3}s)", self.inner.t)
            }
            None => format!(
                "Clearance({:.4} m at t={:.3}s)",
                self.inner.distance, self.inner.t
            ),
        }
    }
}

fn spin_mode(name: &str) -> PyResult<botrail_scene::toolpath::SpinMode> {
    match name {
        "greedy" => Ok(botrail_scene::toolpath::SpinMode::Greedy),
        "optimize" => Ok(botrail_scene::toolpath::SpinMode::optimize()),
        other => Err(PyValueError::new_err(format!(
            "spin must be \"greedy\" or \"optimize\", got {other:?}"
        ))),
    }
}

/// How a bake held the commanded feed: the floors make `length / feed` a
/// hard lower bound, so joints can only slow a cut — this says where they
/// did, and which axis owned it.
#[pyclass(frozen, module = "botrail._core")]
struct FeedReport {
    inner: botrail_scene::toolpath::FeedReport,
    joint_names: Vec<String>,
}

#[pymethods]
impl FeedReport {
    /// Commanded cutting time / achieved; 1.0 = the feed was held
    /// everywhere.
    #[getter]
    fn hold_ratio(&self) -> f64 {
        self.inner.hold_ratio
    }

    #[getter]
    fn commanded_cut_seconds(&self) -> f64 {
        self.inner.commanded_cut_seconds
    }

    #[getter]
    fn achieved_cut_seconds(&self) -> f64 {
        self.inner.achieved_cut_seconds
    }

    /// One dict per slow stretch: `{start, end, move, commanded_feed,
    /// achieved_feed, limiting_joint}` — the joint name is the axis that
    /// ran closest to its velocity limit there.
    #[getter]
    fn slow_spans<'py>(&self, py: Python<'py>) -> PyResult<Vec<Bound<'py, pyo3::types::PyDict>>> {
        self.inner
            .slow_spans
            .iter()
            .map(|s| {
                let d = pyo3::types::PyDict::new(py);
                d.set_item("start", s.start)?;
                d.set_item("end", s.end)?;
                d.set_item("move", s.move_index)?;
                d.set_item("commanded_feed", s.commanded_feed)?;
                d.set_item("achieved_feed", s.achieved_feed)?;
                d.set_item(
                    "limiting_joint",
                    self.joint_names
                        .get(s.limiting_joint)
                        .cloned()
                        .unwrap_or_else(|| s.limiting_joint.to_string()),
                )?;
                Ok(d)
            })
            .collect()
    }

    fn __repr__(&self) -> String {
        format!(
            "FeedReport(hold {:.1}%, commanded {:.2}s / achieved {:.2}s, {} slow span{})",
            self.inner.hold_ratio * 100.0,
            self.inner.commanded_cut_seconds,
            self.inner.achieved_cut_seconds,
            self.inner.slow_spans.len(),
            if self.inner.slow_spans.len() == 1 {
                ""
            } else {
                "s"
            },
        )
    }
}

/// A carved stock: the machined part as a mesh plus the volume
/// bookkeeping. Presentation and numbers, not verification — in a
/// kinematic world the TCP follows the toolpath exactly.
#[pyclass(frozen, module = "botrail._core")]
struct StockCarve {
    inner: botrail_scene::carve::StockCarve,
}

#[pymethods]
impl StockCarve {
    #[getter]
    fn removed_volume(&self) -> f64 {
        self.inner.removed_volume
    }

    #[getter]
    fn remaining_volume(&self) -> f64 {
        self.inner.remaining_volume
    }

    #[getter]
    fn initial_volume(&self) -> f64 {
        self.inner.initial_volume
    }

    #[getter]
    fn voxel_size(&self) -> f64 {
        self.inner.voxel_size
    }

    #[getter]
    fn triangle_count(&self) -> usize {
        self.inner.mesh.indices.len()
    }

    /// World pose to place the mesh at (the stock's pose at carve time).
    #[getter]
    fn pose(&self) -> hub::PoseArrays {
        let t = self.inner.pose.translation;
        let q = self.inner.pose.rotation.coords;
        ([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
    }

    /// Writes the machined-part mesh as binary STL (stock-local
    /// coordinates; add it to the scene at `pose`). STL carries no
    /// colors — prefer `save_obj` for the cut-surface finish.
    fn save_stl(&self, path: std::path::PathBuf) -> PyResult<()> {
        std::fs::write(&path, botrail_mesh::to_stl_binary(&self.inner.mesh))
            .map_err(|e| PyIOError::new_err(e.to_string()))
    }

    /// Writes the machined part as OBJ plus a sibling `.mtl`, carrying
    /// the surface classing as face colors: the surviving skin in the
    /// stock color, cutter-made surfaces in the bright machined finish.
    /// The studio and the USD export both read face colors back from
    /// this format (add the obstacle *without* a `color=` override).
    fn save_obj(&self, path: std::path::PathBuf) -> PyResult<()> {
        let mtl_path = path.with_extension("mtl");
        let mtl_name = mtl_path
            .file_name()
            .map(|n| n.to_string_lossy().to_string())
            .ok_or_else(|| PyValueError::new_err("path has no file name"))?;
        let (obj, mtl) = botrail_mesh::to_obj_with_mtl(&self.inner.mesh, &mtl_name);
        std::fs::write(&path, obj).map_err(|e| PyIOError::new_err(e.to_string()))?;
        std::fs::write(&mtl_path, mtl).map_err(|e| PyIOError::new_err(e.to_string()))
    }

    fn __repr__(&self) -> String {
        format!(
            "StockCarve(removed {:.2} cm3 of {:.2} cm3, {} tris at {:.1} mm voxels)",
            self.inner.removed_volume * 1e6,
            self.inner.initial_volume * 1e6,
            self.inner.mesh.indices.len(),
            self.inner.voxel_size * 1e3,
        )
    }
}

/// Face diagnosis of a toolpath: every sample attempted, all failures
/// collected. Truthy iff clean (`ok`).
#[pyclass(frozen, module = "botrail._core")]
struct ToolpathReport {
    inner: botrail_scene::toolpath::ToolpathReport,
}

#[pymethods]
impl ToolpathReport {
    /// Number of resampled path points attempted.
    #[getter]
    fn total_samples(&self) -> usize {
        self.inner.total_samples
    }

    /// True when every sample solved, stayed on its IK branch, and was
    /// collision-free.
    #[getter]
    fn ok(&self) -> bool {
        self.inner.ok()
    }

    /// One dict per failing sample: `{sample, move, kind, position,
    /// detail}` with `kind` in `"unreachable" | "config_jump" |
    /// "collision"` and `position` the world target the sample aimed at.
    #[getter]
    fn issues<'py>(&self, py: Python<'py>) -> PyResult<Vec<Bound<'py, pyo3::types::PyDict>>> {
        use botrail_scene::toolpath::IssueKind;
        self.inner
            .issues
            .iter()
            .map(|i| {
                let d = pyo3::types::PyDict::new(py);
                d.set_item("sample", i.sample)?;
                d.set_item("move", i.move_index)?;
                d.set_item(
                    "kind",
                    match i.kind {
                        IssueKind::Unreachable => "unreachable",
                        IssueKind::ConfigJump => "config_jump",
                        IssueKind::Collision => "collision",
                    },
                )?;
                d.set_item("position", (i.position.x, i.position.y, i.position.z))?;
                d.set_item("detail", i.detail.clone())?;
                Ok(d)
            })
            .collect()
    }

    fn __bool__(&self) -> bool {
        self.inner.ok()
    }

    fn __repr__(&self) -> String {
        if self.inner.ok() {
            format!("ToolpathReport(ok, {} samples)", self.inner.total_samples)
        } else {
            let first = &self.inner.issues[0];
            format!(
                "ToolpathReport({} issues / {} samples, first at sample {}: {})",
                self.inner.issues.len(),
                self.inner.total_samples,
                first.sample,
                first.detail
            )
        }
    }
}

/// Parses a G-code subset into toolpath-move JSON:
/// `{"moves": [...], "warnings": [...]}`. `bt.toolpath.from_gcode` wraps
/// this — call that instead.
#[pyfunction]
#[pyo3(signature = (text, chord_tol = 1e-4))]
fn _parse_gcode_json(text: &str, chord_tol: f64) -> PyResult<String> {
    let parsed =
        botrail_scene::gcode::parse_gcode(text, &botrail_scene::gcode::GcodeOptions { chord_tol })
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
    let msg = botrail_scene::toolpath::toolpath_msg(&botrail_scene::toolpath::Toolpath {
        name: String::new(),
        frame: None,
        moves: parsed.moves,
    });
    Ok(serde_json::json!({
        "moves": msg.moves,
        "warnings": parsed.warnings,
    })
    .to_string())
}

/// Parses an APT/CL subset into toolpath-move JSON:
/// `{"moves": [...], "warnings": [...]}`. `bt.toolpath.from_apt` wraps
/// this — call that instead.
#[pyfunction]
fn _parse_apt_json(text: &str) -> PyResult<String> {
    let parsed =
        botrail_scene::apt::parse_apt(text).map_err(|e| PyValueError::new_err(e.to_string()))?;
    let msg = botrail_scene::toolpath::toolpath_msg(&botrail_scene::toolpath::Toolpath {
        name: String::new(),
        frame: None,
        moves: parsed.moves,
    });
    Ok(serde_json::json!({
        "moves": msg.moves,
        "warnings": parsed.warnings,
    })
    .to_string())
}

#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Robot>()?;
    m.add_class::<Scene>()?;
    m.add_class::<IkResult>()?;
    m.add_class::<Trajectory>()?;
    m.add_class::<SequenceTimeline>()?;
    m.add_class::<ScenarioRuns>()?;
    m.add_class::<Span>()?;
    m.add_class::<SignalTrack>()?;
    m.add_class::<Clearance>()?;
    m.add_class::<ToolpathReport>()?;
    m.add_class::<FeedReport>()?;
    m.add_class::<StockCarve>()?;
    m.add_class::<StudioServer>()?;
    m.add_function(wrap_pyfunction!(serve_studio, m)?)?;
    m.add_function(wrap_pyfunction!(catalog::catalog_package, m)?)?;
    m.add_function(wrap_pyfunction!(_parse_gcode_json, m)?)?;
    m.add_function(wrap_pyfunction!(_parse_apt_json, m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}

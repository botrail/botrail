//! Python bindings for botrail (`botrail._core`).

mod hub;
mod server;

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

    #[getter]
    fn link_names(&self) -> Vec<String> {
        self.inner.links.iter().map(|l| l.name.clone()).collect()
    }

    /// Default end-effector link name (deepest leaf in the kinematic tree).
    #[getter]
    fn tcp_link(&self) -> String {
        self.inner.links[self.inner.default_tcp_link()].name.clone()
    }

    /// Solves inverse kinematics. With `quaternion=None` only the position
    /// is matched. `link` defaults to the TCP link, `seed` to the neutral
    /// configuration. Always returns the best configuration found; check
    /// `result.converged`.
    #[pyo3(signature = (position, quaternion = None, link = None, seed = None, max_iters = 100))]
    fn ik(
        &self,
        position: [f64; 3],
        quaternion: Option<[f64; 4]>,
        link: Option<&str>,
        seed: Option<Vec<f64>>,
        max_iters: usize,
    ) -> PyResult<IkResult> {
        let (target, mode) = ik_target(position, quaternion);
        let link_index = resolve_link(&self.inner, link)?;
        let seed = seed.unwrap_or_else(|| self.inner.neutral_positions());
        let options = botrail_kin::IkOptions {
            mode,
            max_iters,
            ..botrail_kin::IkOptions::default()
        };
        let result = botrail_kin::solve_ik(&self.inner, link_index, &target, &seed, &options)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(IkResult { inner: result })
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

/// A robot in a workspace. Shared with the studio server: joint state
/// changes made here are pushed to connected browsers immediately.
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

    /// Bakes a trajectory to a USD animation layer (`.usda`) that plays in
    /// usdview / Omniverse / Blender: robot link motion as timeSamples,
    /// obstacles as prims, grasped objects riding along. USD-sourced robots
    /// reference their original stage (assets copied to a sibling
    /// `<stem>_assets/` directory); URDF robots are authored from the
    /// model's visuals. `robot` names the instance the trajectory belongs
    /// to (required when the scene has several). Returns exporter warnings.
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
    #[pyo3(signature = (name, position, size, quaternion = None, watch = None, watch_robot = false, watch_robots = None))]
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
    ) -> PyResult<()> {
        self.hub.upsert_sensor(botrail_scene::seq::Sensor {
            name: name.to_string(),
            kind: botrail_scene::seq::SensorKind::Zone {
                pose: pose_from(position, quaternion),
                size: nalgebra::Vector3::new(size[0], size[1], size[2]),
            },
            watch: sensor_watch(watch, watch_robot, watch_robots),
        });
        Ok(())
    }

    /// Adds a photoelectric beam sensor between two world points, ON while
    /// the beam is interrupted. Watch semantics as in `add_zone_sensor`.
    #[pyo3(signature = (name, frm, to, radius = 0.005, watch = None, watch_robot = false, watch_robots = None))]
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
    ) -> PyResult<()> {
        self.hub.upsert_sensor(botrail_scene::seq::Sensor {
            name: name.to_string(),
            kind: botrail_scene::seq::SensorKind::Beam {
                from: nalgebra::Point3::new(frm[0], frm[1], frm[2]),
                to: nalgebra::Point3::new(to[0], to[1], to[2]),
                radius,
            },
            watch: sensor_watch(watch, watch_robot, watch_robots),
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

    /// Rolls out a sequence with the PLC scan loop against a snapshot of
    /// this scene (motions plan at their step, grasped objects ride along)
    /// and returns the baked timeline. Also broadcasts the result to
    /// connected studio clients for playback.
    #[pyo3(signature = (name, dt = 0.01, max_duration = 120.0))]
    fn simulate_sequence(
        &self,
        name: &str,
        dt: f64,
        max_duration: f64,
    ) -> PyResult<SequenceTimeline> {
        if !(dt.is_finite() && dt > 0.0) {
            return Err(PyValueError::new_err(format!(
                "dt must be positive, got {dt}"
            )));
        }
        let options = botrail_scene::rollout::RolloutOptions {
            dt,
            max_duration,
            ..Default::default()
        };
        let (timeline, scene) = self
            .hub
            .simulate_sequence(name, &options)
            .map_err(PyValueError::new_err)?;
        Ok(SequenceTimeline {
            inner: timeline,
            scene,
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
            }],
            joint_names: model
                .actuated_joint_names()
                .into_iter()
                .map(str::to_string)
                .collect(),
            limits: hub::traj_limits(&model),
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
        })
    }

    /// Saves the scene (robot URDF, joint state, obstacles, motions) as a
    /// self-contained `.botrail` project file.
    /// Saves the project. Plain JSON when everything is self-contained; a
    /// zip archive (`project.json` + `assets/`) when mesh files are
    /// referenced, so the file stays portable across machines.
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
        use botrail_scene::project::RobotSourceMsg;
        let bytes = std::fs::read(&path)
            .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))?;
        let project = read_project(&bytes)
            .map_err(|e| PyValueError::new_err(format!("{}: {e}", path.display())))?;
        let mut models = Vec::with_capacity(project.robots.len());
        for robot_msg in &project.robots {
            let model = match &robot_msg.source {
                RobotSourceMsg::Urdf { xml } => Arc::new(
                    RobotModel::from_urdf_str(xml)
                        .map_err(|e| PyValueError::new_err(e.to_string()))?,
                ),
                RobotSourceMsg::Usd {
                    path,
                    articulation_root,
                } => {
                    let imported = botrail_usd::import_robot(
                        std::path::Path::new(path),
                        &botrail_usd::RobotImportOptions {
                            articulation_root: Some(articulation_root.clone()),
                            ..Default::default()
                        },
                    )
                    .map_err(|e| PyValueError::new_err(e.to_string()))?;
                    Arc::new(imported.model)
                }
            };
            models.push(model);
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
                self.scene
                    .fk_for(r, &track.trajectory.sample(t))
                    .map_err(|e| PyValueError::new_err(e.to_string()))
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
    #[pyo3(signature = (path, fps = 60.0))]
    fn export_usd(&self, path: PathBuf, fps: f64) -> PyResult<Vec<String>> {
        if !(fps.is_finite() && fps > 0.0) {
            return Err(PyValueError::new_err(format!(
                "fps must be positive, got {fps}"
            )));
        }
        let duration = self.inner.duration;
        let mut times = Vec::new();
        let mut k = 0u64;
        loop {
            let t = k as f64 / fps;
            if t >= duration - 1e-9 {
                break;
            }
            times.push(t);
            k += 1;
        }
        times.push(duration);

        // Per-robot FK per frame (robot-major for the exporter)...
        let mut robot_frames: Vec<Vec<Vec<nalgebra::Isometry3<f64>>>> =
            Vec::with_capacity(self.inner.robots.len());
        let mut joint_samples: Vec<Vec<Vec<f64>>> = Vec::with_capacity(self.inner.robots.len());
        for (r, track) in self.inner.robots.iter().enumerate() {
            let mut frames = Vec::with_capacity(times.len());
            let mut samples = Vec::with_capacity(times.len());
            for &t in &times {
                let q = track.trajectory.sample(t);
                frames.push(
                    self.scene
                        .fk_for(r, &q)
                        .map_err(|e| PyValueError::new_err(e.to_string()))?,
                );
                samples.push(q);
            }
            robot_frames.push(frames);
            joint_samples.push(samples);
        }
        // ...and frame-major for the object tracks (handover-aware: each
        // span names its carrying robot).
        let all_frames: Vec<Vec<Vec<nalgebra::Isometry3<f64>>>> = (0..times.len())
            .map(|k| robot_frames.iter().map(|rf| rf[k].clone()).collect())
            .collect();

        let objects: Vec<botrail_usd::export::ObjectSpec> = self
            .scene
            .obstacles()
            .iter()
            .map(|o| {
                let track = match self.inner.objects.iter().find(|t| t.name == o.name) {
                    Some(track) => botrail_usd::export::PoseTrack::Sampled(
                        times
                            .iter()
                            .enumerate()
                            .map(|(k, &t)| {
                                botrail_scene::rollout::SequenceTimeline::object_pose(
                                    track,
                                    &all_frames[k],
                                    t,
                                )
                                .unwrap_or(o.pose)
                            })
                            .collect(),
                    ),
                    None => botrail_usd::export::PoseTrack::Static(o.pose),
                };
                botrail_usd::export::ObjectSpec {
                    name: o.name.clone(),
                    geometry: o.geometry.clone(),
                    track,
                    color: o.color,
                }
            })
            .collect();

        // A sole robot keeps the historical `Robot` prim (byte compat).
        let single = self.inner.robots.len() == 1;
        let names: Vec<String> = self
            .inner
            .robots
            .iter()
            .map(|r| {
                if single {
                    "Robot".to_string()
                } else {
                    r.name.clone()
                }
            })
            .collect();
        let robots: Vec<botrail_usd::export::RobotAnimation> = self
            .inner
            .robots
            .iter()
            .enumerate()
            .map(|(r, _)| botrail_usd::export::RobotAnimation {
                name: &names[r],
                model: &self.scene.robots()[r].model,
                link_poses: &robot_frames[r],
                joint_samples: Some(&joint_samples[r]),
            })
            .collect();
        let input = botrail_usd::export::AnimationInput {
            robots: &robots,
            times: &times,
            objects: &objects,
        };
        let options = botrail_usd::export::ExportOptions { fps };
        botrail_usd::export::write_animation(&path, &input, &options)
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

#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Robot>()?;
    m.add_class::<Scene>()?;
    m.add_class::<IkResult>()?;
    m.add_class::<Trajectory>()?;
    m.add_class::<SequenceTimeline>()?;
    m.add_class::<Span>()?;
    m.add_class::<SignalTrack>()?;
    m.add_class::<Clearance>()?;
    m.add_class::<StudioServer>()?;
    m.add_function(wrap_pyfunction!(serve_studio, m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}

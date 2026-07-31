//! Python bindings for botrail (`botrail._core`).

mod hub;
mod server;

use std::path::PathBuf;
use std::sync::Arc;

use botrail_model::RobotModel;
use pyo3::exceptions::{PyIOError, PyValueError};
use pyo3::prelude::*;

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
                mesh_cache_dir: None,
                articulation_root,
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

#[pymethods]
impl Scene {
    /// A scene with the robot root placed at the world-frame base pose
    /// (identity when omitted).
    #[new]
    #[pyo3(signature = (robot, base_position = None, base_quaternion = None))]
    fn new(
        robot: &Robot,
        base_position: Option<[f64; 3]>,
        base_quaternion: Option<[f64; 4]>,
    ) -> Self {
        let base = pose_from(base_position.unwrap_or([0.0; 3]), base_quaternion);
        let scene = botrail_scene::Scene::with_base(robot.inner.clone(), base);
        Scene {
            hub: Arc::new(SceneHub::new(scene)),
            robot: robot.clone(),
        }
    }

    #[getter]
    fn robot(&self) -> Robot {
        self.robot.clone()
    }

    /// World pose of the robot root as `(position, quaternion_xyzw)`.
    #[getter]
    fn robot_base_pose(&self) -> ([f64; 3], [f64; 4]) {
        self.hub.robot_base_pose()
    }

    /// Places the robot root at the world-frame pose and pushes the new
    /// state to connected studio clients.
    #[pyo3(signature = (position, quaternion = None))]
    fn set_robot_base_pose(&self, position: [f64; 3], quaternion: Option<[f64; 4]>) {
        self.hub
            .set_robot_base_pose(pose_from(position, quaternion));
    }

    #[getter]
    fn joint_positions(&self) -> Vec<f64> {
        self.hub.joint_positions()
    }

    fn set_joint_positions(&self, positions: Vec<f64>) -> PyResult<()> {
        self.hub
            .set_joint_positions(positions)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    /// World pose of a link as `(position, quaternion_xyzw)`.
    fn link_pose(&self, link_name: &str) -> PyResult<([f64; 3], [f64; 4])> {
        self.hub
            .link_pose(link_name)
            .ok_or_else(|| PyValueError::new_err(format!("unknown link `{link_name}`")))
    }

    /// Adds a box obstacle (full extents, meters). Returns the final name,
    /// which may be uniquified. Changes are pushed to connected studios.
    #[pyo3(signature = (name, size, position, quaternion = None))]
    fn add_box(
        &self,
        name: &str,
        size: [f64; 3],
        position: [f64; 3],
        quaternion: Option<[f64; 4]>,
    ) -> PyResult<String> {
        self.hub
            .add_obstacle(
                name,
                botrail_model::Geometry::Box {
                    size: nalgebra::Vector3::new(size[0], size[1], size[2]),
                },
                pose_from(position, quaternion),
            )
            .map_err(scene_err)
    }

    #[pyo3(signature = (name, radius, position, quaternion = None))]
    fn add_sphere(
        &self,
        name: &str,
        radius: f64,
        position: [f64; 3],
        quaternion: Option<[f64; 4]>,
    ) -> PyResult<String> {
        self.hub
            .add_obstacle(
                name,
                botrail_model::Geometry::Sphere { radius },
                pose_from(position, quaternion),
            )
            .map_err(scene_err)
    }

    /// Adds a cylinder obstacle (URDF convention: axis along local +z).
    #[pyo3(signature = (name, radius, length, position, quaternion = None))]
    fn add_cylinder(
        &self,
        name: &str,
        radius: f64,
        length: f64,
        position: [f64; 3],
        quaternion: Option<[f64; 4]>,
    ) -> PyResult<String> {
        self.hub
            .add_obstacle(
                name,
                botrail_model::Geometry::Cylinder { radius, length },
                pose_from(position, quaternion),
            )
            .map_err(scene_err)
    }

    /// Adds a mesh obstacle from an STL/OBJ file. The collision shape is a
    /// VHACD convex decomposition (computed on first load, then cached on
    /// disk); the studio renders the original mesh.
    #[pyo3(signature = (name, path, position, scale = None, quaternion = None))]
    fn add_mesh(
        &self,
        name: &str,
        path: PathBuf,
        position: [f64; 3],
        scale: Option<[f64; 3]>,
        quaternion: Option<[f64; 4]>,
    ) -> PyResult<String> {
        let s = scale.unwrap_or([1.0; 3]);
        self.hub
            .add_obstacle(
                name,
                botrail_model::Geometry::Mesh {
                    path,
                    scale: nalgebra::Vector3::new(s[0], s[1], s[2]),
                },
                pose_from(position, quaternion),
            )
            .map_err(scene_err)
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
        let prefix = prefix.unwrap_or_default();
        let batch = imported
            .nodes
            .into_iter()
            .map(|n| (format!("{prefix}{}", n.name), n.geometry, n.pose))
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

    /// Attaches an obstacle to a robot link at its current relative pose —
    /// a grasp. While attached the object follows the link (live, in
    /// planning, and in playback) and collides as part of the robot.
    /// `link=None` uses the TCP link; `touch_links=None` allows contact
    /// with the link's subtree (the gripper).
    #[pyo3(signature = (name, link = None, touch_links = None))]
    fn attach(
        &self,
        name: &str,
        link: Option<&str>,
        touch_links: Option<Vec<String>>,
    ) -> PyResult<()> {
        self.hub
            .attach_obstacle(name, link, touch_links.as_deref())
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
    /// model's visuals. Returns exporter warnings.
    #[pyo3(signature = (trajectory, path, fps = 60.0))]
    fn export_usd(
        &self,
        trajectory: &Trajectory,
        path: PathBuf,
        fps: f64,
    ) -> PyResult<Vec<String>> {
        self.hub
            .export_trajectory_usd(&trajectory.inner, &path, fps)
            .map_err(PyValueError::new_err)
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
    #[pyo3(signature = (goal, max_iters = 10_000, seed = None, broadcast = true))]
    fn plan(
        &self,
        goal: Vec<f64>,
        max_iters: usize,
        seed: Option<u64>,
        broadcast: bool,
    ) -> PyResult<Trajectory> {
        let mut options = botrail_plan::PlanOptions {
            max_iters,
            ..botrail_plan::PlanOptions::default()
        };
        if let Some(seed) = seed {
            options.seed = seed;
        }
        let result = if broadcast {
            self.hub.plan_and_broadcast(&goal, &options)
        } else {
            self.hub.plan_to(&goal, &options)
        };
        let (traj, path, _) = result.map_err(PyValueError::new_err)?;
        Ok(Trajectory {
            inner: traj,
            segment_ends: Vec::new(),
            segments: vec![botrail_scene::motion::PlannedSegment {
                kind: botrail_scene::motion::SegmentKind::Joint,
                waypoints: path,
            }],
            joint_names: self
                .robot
                .inner
                .actuated_joint_names()
                .into_iter()
                .map(str::to_string)
                .collect(),
            limits: hub::traj_limits(&self.robot.inner),
        })
    }

    /// Appends a waypoint segment to `motion` (created when missing).
    /// `goal=None` captures the current configuration. Constraints:
    /// `orientation_cone=(axis_local, axis_world, angle_rad)` keeps the tool
    /// axis inside a cone; `position_box=(min, max)` keeps the TCP inside a
    /// world-aligned box. Both apply along the whole segment.
    #[pyo3(signature = (motion, goal = None, kind = "joint", orientation_cone = None, position_box = None))]
    fn add_segment(
        &self,
        motion: &str,
        goal: Option<Vec<f64>>,
        kind: &str,
        orientation_cone: Option<([f64; 3], [f64; 3], f64)>,
        position_box: Option<([f64; 3], [f64; 3])>,
    ) -> PyResult<()> {
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
            goal_positions: goal.unwrap_or_else(|| self.hub.joint_positions()),
            constraints,
        };
        self.hub
            .add_segment(motion, segment)
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
        Ok(Trajectory {
            inner: planned.trajectory,
            segment_ends: planned.segment_ends,
            segments: planned.segments,
            joint_names: self
                .robot
                .inner
                .actuated_joint_names()
                .into_iter()
                .map(str::to_string)
                .collect(),
            limits: hub::traj_limits(&self.robot.inner),
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
        // reference targets under the stage directory) as `robot/<relpath>`.
        for robot in &mut project.robots {
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
                        format!("robot/{}", rel.to_string_lossy().replace('\\', "/")),
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
            *stage_path = format!("robot/{rel_root}");
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

    /// Loads a `.botrail` project file into a fresh scene (robot included).
    /// URDF robots rebuild from the embedded XML; USD robots re-import from
    /// the referenced stage path.
    #[staticmethod]
    fn load_project(path: PathBuf) -> PyResult<Self> {
        use botrail_scene::project::RobotSourceMsg;
        let bytes = std::fs::read(&path)
            .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))?;
        let project = read_project(&bytes)
            .map_err(|e| PyValueError::new_err(format!("{}: {e}", path.display())))?;
        let robot_msg = project
            .single_robot()
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let scene = match &robot_msg.source {
            RobotSourceMsg::Urdf { .. } => botrail_scene::Scene::from_project(&project)
                .map_err(|e| PyValueError::new_err(e.to_string()))?,
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
                let mut scene = botrail_scene::Scene::new(Arc::new(imported.model));
                scene
                    .apply_project(&project)
                    .map_err(|e| PyValueError::new_err(e.to_string()))?;
                scene
            }
        };
        let robot = Robot {
            inner: scene.robot.clone(),
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
    #[pyo3(signature = (position, quaternion = None, link = None, max_iters = 10_000, seed = None, broadcast = true))]
    #[allow(clippy::too_many_arguments)]
    fn plan_to_pose(
        &self,
        position: [f64; 3],
        quaternion: Option<[f64; 4]>,
        link: Option<&str>,
        max_iters: usize,
        seed: Option<u64>,
        broadcast: bool,
    ) -> PyResult<Trajectory> {
        let link_index = resolve_link(&self.robot.inner, link)?;
        let (target, mode) = ik_target(position, quaternion);
        // The target is world-frame; re-express it in the robot base frame
        // for the base-frame solver.
        let target = self.hub.robot_base_isometry().inverse() * target;
        let seed_q = self.hub.joint_positions();
        let options = botrail_kin::IkOptions {
            mode,
            ..botrail_kin::IkOptions::default()
        };
        let ik = botrail_kin::solve_ik(&self.robot.inner, link_index, &target, &seed_q, &options)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        if !ik.converged {
            return Err(PyValueError::new_err(format!(
                "IK did not converge (pos_error={:.4}, rot_error={:.4})",
                ik.pos_error, ik.rot_error
            )));
        }
        self.plan(ik.q, max_iters, seed, broadcast)
    }

    /// Solves IK toward the given pose (seeded from the current
    /// configuration), applies the best-effort result to the scene, and
    /// pushes it to connected studio clients. `quaternion=None` matches
    /// position only; `link` defaults to the TCP link.
    #[pyo3(signature = (position, quaternion = None, link = None, max_iters = 100))]
    fn set_tcp_target(
        &self,
        position: [f64; 3],
        quaternion: Option<[f64; 4]>,
        link: Option<&str>,
        max_iters: usize,
    ) -> PyResult<IkResult> {
        let link_index = resolve_link(&self.robot.inner, link)?;
        let link_name = self.robot.inner.links[link_index].name.clone();
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
            .set_tcp_target(&link_name, &pose, &options)
            .map_err(PyValueError::new_err)?;
        Ok(IkResult { inner: result })
    }

    fn __repr__(&self) -> String {
        format!("Scene(robot='{}')", self.robot.inner.name)
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
        .filter(|n| n.starts_with("assets/") || n.starts_with("robot/"))
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
            if path.starts_with("robot/") {
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

/// A baked sequence rollout: the cycle's joint track, grasped-object
/// motion, signal waveforms, and step spans (the timing chart).
#[pyclass(frozen, module = "botrail._core")]
struct SequenceTimeline {
    inner: botrail_scene::rollout::SequenceTimeline,
    /// The pre-rollout snapshot the timeline was baked against (FK model,
    /// base pose, obstacle geometry for USD export).
    scene: botrail_scene::Scene,
}

#[pymethods]
impl SequenceTimeline {
    /// Cycle time in seconds.
    #[getter]
    fn duration(&self) -> f64 {
        self.inner.duration
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

    /// Joint positions at time `t` (clamped to the cycle).
    fn sample(&self, t: f64) -> Vec<f64> {
        self.inner.robot.sample(t)
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
        let poses = self
            .scene
            .fk(&self.inner.robot.sample(t))
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let pose = botrail_scene::rollout::SequenceTimeline::object_pose(track, &poses, t)
            .ok_or_else(|| PyValueError::new_err("empty object track"))?;
        let q = pose.rotation.coords;
        Ok((
            [pose.translation.x, pose.translation.y, pose.translation.z],
            [q.x, q.y, q.z, q.w],
        ))
    }

    /// The cycle's joint track as a [`Trajectory`] (CSV/JSON export, joint
    /// access). Step boundaries land in `segment_ends`.
    #[getter]
    fn trajectory(&self) -> Trajectory {
        Trajectory {
            inner: self.inner.robot.clone(),
            joint_names: self
                .scene
                .robot
                .actuated_joint_names()
                .iter()
                .map(|n| n.to_string())
                .collect(),
            segment_ends: self.inner.step_spans.iter().map(|s| s.end).collect(),
            segments: Vec::new(),
            limits: crate::hub::traj_limits(&self.scene.robot),
        }
    }

    /// Bakes the whole cycle to a USD animation layer (see
    /// `Scene.export_usd`): robot + every obstacle, with grasped objects
    /// riding, releasing, and resting exactly as simulated.
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

        let mut link_poses = Vec::with_capacity(times.len());
        for &t in &times {
            let poses = self
                .scene
                .fk(&self.inner.robot.sample(t))
                .map_err(|e| PyValueError::new_err(e.to_string()))?;
            link_poses.push(poses);
        }
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
                                    &link_poses[k],
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
                }
            })
            .collect();
        let input = botrail_usd::export::AnimationInput {
            model: &self.scene.robot,
            times: &times,
            link_poses: &link_poses,
            objects: &objects,
        };
        let options = botrail_usd::export::ExportOptions { fps };
        botrail_usd::export::write_animation(&path, &input, &options)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }
}

#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Robot>()?;
    m.add_class::<Scene>()?;
    m.add_class::<IkResult>()?;
    m.add_class::<Trajectory>()?;
    m.add_class::<SequenceTimeline>()?;
    m.add_class::<StudioServer>()?;
    m.add_function(wrap_pyfunction!(serve_studio, m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}

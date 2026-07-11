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
    #[new]
    fn new(robot: &Robot) -> Self {
        let scene = botrail_scene::Scene::new(robot.inner.clone());
        Scene {
            hub: Arc::new(SceneHub::new(scene)),
            robot: robot.clone(),
        }
    }

    #[getter]
    fn robot(&self) -> Robot {
        self.robot.clone()
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

    fn remove_obstacle(&self, name: &str) -> PyResult<()> {
        self.hub.remove_obstacle(name).map_err(scene_err)
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
        let (traj, _, _) = result.map_err(PyValueError::new_err)?;
        Ok(Trajectory {
            inner: traj,
            segment_ends: Vec::new(),
            joint_names: self
                .robot
                .inner
                .actuated_joint_names()
                .into_iter()
                .map(str::to_string)
                .collect(),
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
            joint_names: self
                .robot
                .inner
                .actuated_joint_names()
                .into_iter()
                .map(str::to_string)
                .collect(),
        })
    }

    /// Saves the scene (robot URDF, joint state, obstacles, motions) as a
    /// self-contained `.botrail` project file.
    fn save_project(&self, path: PathBuf) -> PyResult<()> {
        std::fs::write(&path, self.hub.project_json())
            .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))
    }

    /// Loads a `.botrail` project file into a fresh scene (robot included).
    #[staticmethod]
    fn load_project(path: PathBuf) -> PyResult<Self> {
        let json = std::fs::read_to_string(&path)
            .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))?;
        let project = botrail_scene::project::ProjectFile::from_json(&json)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let scene = botrail_scene::Scene::from_project(&project)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
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

/// A planned, time-parameterized joint trajectory.
#[pyclass(frozen, module = "botrail._core")]
struct Trajectory {
    inner: botrail_traj::JointTrajectory,
    joint_names: Vec<String>,
    /// Time at which each motion segment ends (empty for single plans).
    segment_ends: Vec<f64>,
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

#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Robot>()?;
    m.add_class::<Scene>()?;
    m.add_class::<IkResult>()?;
    m.add_class::<Trajectory>()?;
    m.add_class::<StudioServer>()?;
    m.add_function(wrap_pyfunction!(serve_studio, m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}

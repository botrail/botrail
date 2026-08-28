//! Python bindings for botrail (`botrail._core`).

mod catalog;
mod hub;
mod server;

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
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

/// A camera pose at `position` aimed at `target`: -Z along the view ray,
/// +Y up-ish (world +Z as the up hint; world +Y for a straight-down or
/// straight-up view, where the hint degenerates).
fn look_at_pose(position: [f64; 3], target: [f64; 3]) -> PyResult<nalgebra::Isometry3<f64>> {
    use nalgebra::{Isometry3, Matrix3, Translation3, UnitQuaternion, Vector3};
    let eye = Vector3::from(position);
    let fwd = Vector3::from(target) - eye;
    if fwd.norm() < 1e-9 {
        return Err(PyValueError::new_err(
            "look_at target coincides with the camera position",
        ));
    }
    let f = fwd.normalize();
    let hint = if f.z.abs() > 0.999 {
        Vector3::y()
    } else {
        Vector3::z()
    };
    let z = -f;
    let x = hint.cross(&z).normalize();
    let y = z.cross(&x);
    let rot = Matrix3::from_columns(&[x, y, z]);
    // The closed-form conversion: `from_matrix`'s iterative refinement
    // stalls at exactly π (e.g. any camera looking along -Y from +Y) and
    // silently returns identity.
    let q = UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(rot));
    Ok(Isometry3::from_parts(Translation3::from(eye), q))
}

fn pose_from(position: [f64; 3], quaternion: Option<[f64; 4]>) -> nalgebra::Isometry3<f64> {
    (&botrail_scene::wire::PoseMsg {
        position,
        quaternion: quaternion.unwrap_or([0.0, 0.0, 0.0, 1.0]),
    })
        .into()
}

/// One footfall as Python sees it: `(leg, lift, land, (x, y, z))`.
type FootfallRow = (String, f64, f64, (f64, f64, f64));

/// A gait from a `bt.Gait` (anything with a `_spec()` returning the plain
/// dict `bt.gait.Gait._spec` builds) or from such a dict directly.
fn gait_from_py(obj: &Bound<'_, PyAny>) -> PyResult<botrail_scene::seq::GaitSpec> {
    use botrail_scene::seq::{FootContact, GaitPattern, GaitSpec, LegSpec};
    use pyo3::types::PyDict;
    let spec: Bound<'_, PyDict> = if obj.is_instance_of::<PyDict>() {
        obj.downcast::<PyDict>()?.clone()
    } else {
        obj.call_method0("_spec")
            .map_err(|_| {
                PyValueError::new_err("gait must be a bt.Gait (or the dict its _spec() builds)")
            })?
            .downcast_into::<PyDict>()?
    };
    let field = |key: &str| -> PyResult<Bound<'_, PyAny>> {
        spec.get_item(key)?
            .ok_or_else(|| PyValueError::new_err(format!("gait spec lacks `{key}`")))
    };
    let optional = |key: &str| -> PyResult<Option<Bound<'_, PyAny>>> {
        Ok(spec.get_item(key)?.filter(|v| !v.is_none()))
    };
    let legs: Vec<(String, String, String)> = field("legs")?.extract()?;
    let legs = legs
        .into_iter()
        .map(|(name, foot, contact)| {
            let contact = match contact.as_str() {
                "point" => FootContact::Point,
                "sole" => FootContact::Sole { yaw_free: false },
                "sole_yaw_free" => FootContact::Sole { yaw_free: true },
                other => {
                    return Err(PyValueError::new_err(format!(
                        "leg `{name}`: contact must be point, sole or sole_yaw_free, got `{other}`"
                    )))
                }
            };
            Ok(LegSpec {
                name,
                foot,
                contact,
            })
        })
        .collect::<PyResult<Vec<_>>>()?;
    let pattern: String = match optional("pattern")? {
        Some(v) => v.extract()?,
        None => "trot".to_string(),
    };
    let pattern = match pattern.as_str() {
        "walk" => GaitPattern::Walk,
        "trot" => GaitPattern::Trot,
        "biped" => GaitPattern::Biped,
        "custom" => GaitPattern::Custom {
            duty: field("duty")?.extract()?,
            phases: field("phases")?.extract()?,
        },
        other => {
            return Err(PyValueError::new_err(format!(
                "pattern must be walk, trot, biped or custom, got `{other}`"
            )))
        }
    };
    let number = |key: &str, default: f64| -> PyResult<f64> {
        match optional(key)? {
            Some(v) => v.extract(),
            None => Ok(default),
        }
    };
    let pairs = |key: &str| -> PyResult<Vec<(String, f64)>> {
        match optional(key)? {
            Some(v) => v.extract(),
            None => Ok(Vec::new()),
        }
    };
    Ok(GaitSpec {
        body_link: optional("body_link")?.map(|v| v.extract()).transpose()?,
        legs,
        pattern,
        period: number("period", 0.5)?,
        lift: number("lift", 0.06)?,
        stance: pairs("stance")?,
        max_stride: number("max_stride", 0.4)?,
        foot_radius: number("foot_radius", 0.0)?,
        arm_swing: pairs("arm_swing")?,
        bob: number("bob", 0.0)?,
        lateral: number("lateral", 0.0)?,
        max_step: optional("max_step")?.map(|v| v.extract()).transpose()?,
    })
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

/// A part attribute from a Python value: int/float → number (bools are
/// refused — they are ints in Python and would sum), str → text.
fn part_attr(key: &str, value: &Bound<'_, PyAny>) -> PyResult<botrail_scene::part::PartAttr> {
    use botrail_scene::part::PartAttr;
    if value.is_instance_of::<pyo3::types::PyBool>() {
        return Err(PyValueError::new_err(format!(
            "set_part: attribute {key:?} is a bool — attributes are numbers or text"
        )));
    }
    if let Ok(number) = value.extract::<f64>() {
        return Ok(PartAttr::Number(number));
    }
    if let Ok(text) = value.extract::<String>() {
        return Ok(PartAttr::Text(text));
    }
    Err(PyValueError::new_err(format!(
        "set_part: attribute {key:?} must be a number or a string"
    )))
}

fn part_attr_object(py: Python<'_>, value: &botrail_scene::part::PartAttr) -> PyObject {
    use botrail_scene::part::PartAttr;
    match value {
        PartAttr::Number(n) => n.into_pyobject(py).expect("float").into_any().unbind(),
        PartAttr::Text(t) => t.into_pyobject(py).expect("str").into_any().unbind(),
    }
}

fn part_dict<'py>(
    py: Python<'py>,
    part: &botrail_scene::part::Part,
) -> PyResult<Bound<'py, PyDict>> {
    let d = PyDict::new(py);
    d.set_item("catalog", part.catalog.as_ref().map(|c| c.display()))?;
    d.set_item("manufacturer", part.manufacturer.clone())?;
    d.set_item("model", part.model.clone())?;
    d.set_item("category", part.category.clone())?;
    d.set_item("description", part.description.clone())?;
    d.set_item("qty", part.qty)?;
    let attributes = PyDict::new(py);
    for (key, value) in &part.attributes {
        attributes.set_item(key, part_attr_object(py, value))?;
    }
    d.set_item("attributes", attributes)?;
    Ok(d)
}

fn part_entry_dict(py: Python<'_>, entry: &botrail_scene::part::PartEntry) -> PyResult<PyObject> {
    let d = part_dict(py, &entry.part)?;
    d.set_item("target", entry.target.clone())?;
    d.set_item("kind", entry.kind.as_str())?;
    Ok(d.into_any().unbind())
}

/// A serde_json value as the matching Python object (dicts, lists,
/// numbers, strings, bools, None) — how report sections come back.
fn json_to_py(py: Python<'_>, value: &serde_json::Value) -> PyResult<PyObject> {
    use pyo3::types::{PyList, PyString};
    Ok(match value {
        serde_json::Value::Null => py.None(),
        serde_json::Value::Bool(b) => b.into_pyobject(py)?.to_owned().into_any().unbind(),
        serde_json::Value::Number(n) => {
            if let Some(i) = n.as_i64() {
                i.into_pyobject(py)?.into_any().unbind()
            } else if let Some(u) = n.as_u64() {
                u.into_pyobject(py)?.into_any().unbind()
            } else {
                n.as_f64()
                    .unwrap_or(f64::NAN)
                    .into_pyobject(py)?
                    .into_any()
                    .unbind()
            }
        }
        serde_json::Value::String(s) => PyString::new(py, s).into_any().unbind(),
        serde_json::Value::Array(items) => {
            let list = PyList::empty(py);
            for item in items {
                list.append(json_to_py(py, item)?)?;
            }
            list.into_any().unbind()
        }
        serde_json::Value::Object(map) => {
            let dict = PyDict::new(py);
            for (k, v) in map {
                dict.set_item(k, json_to_py(py, v)?)?;
            }
            dict.into_any().unbind()
        }
    })
}

fn bom_row_dict(py: Python<'_>, row: &botrail_scene::part::BomRow) -> PyResult<PyObject> {
    let d = PyDict::new(py);
    d.set_item("category", row.category.clone())?;
    d.set_item("names", row.names.clone())?;
    d.set_item("manufacturer", row.manufacturer.clone())?;
    d.set_item("model", row.model.clone())?;
    d.set_item("catalog", row.catalog.as_ref().map(|c| c.display()))?;
    d.set_item("qty", row.qty)?;
    d.set_item("description", row.description.clone())?;
    let attributes = PyDict::new(py);
    for (key, value) in &row.attributes {
        attributes.set_item(key, part_attr_object(py, value))?;
    }
    d.set_item("attributes", attributes)?;
    Ok(d.into_any().unbind())
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
    robot: Option<Robot>,
}

impl Scene {
    /// Resolves an optional `robot=` argument to a robot index. `None`
    /// means the sole robot and is an error when the scene has several.
    fn resolve_robot(&self, robot: Option<&str>) -> PyResult<usize> {
        self.hub.robot_index(robot).map_err(PyValueError::new_err)
    }

    #[allow(clippy::too_many_arguments)]
    fn bind_point(
        &self,
        direction: botrail_scene::iomap::IoDirection,
        name: &str,
        node: &str,
        channel: &str,
        tag: Option<String>,
        field: Option<String>,
        invert: bool,
        contact: Option<&str>,
        safety: bool,
        voltage: Option<f64>,
        logic: Option<&str>,
        note: Option<String>,
    ) -> PyResult<()> {
        use botrail_scene::iomap::{Contact, Electrical, IoBinding, IoPointId, Logic};
        let contact =
            match contact {
                None => None,
                Some(c) => Some(Contact::parse(c).ok_or_else(|| {
                    PyValueError::new_err(format!("unknown contact {c:?} (no, nc)"))
                })?),
            };
        let logic =
            match logic {
                None => None,
                Some(l) => Some(Logic::parse(l).ok_or_else(|| {
                    PyValueError::new_err(format!("unknown logic {l:?} (pnp, npn)"))
                })?),
            };
        let device = if voltage.is_some() || logic.is_some() {
            Some(Electrical { voltage, logic })
        } else {
            None
        };
        self.hub
            .bind_io(IoBinding {
                point: IoPointId::parse(name, direction),
                node: node.to_string(),
                channel: channel.to_string(),
                tag,
                field,
                invert,
                contact,
                safety,
                device,
                note,
                auto: false,
            })
            .map_err(scene_err)
    }

    /// Derives the I/O map over the authored snapshot (state-free: nothing
    /// is stored on the scene).
    fn derive_io(
        &self,
        sequences: Option<Vec<String>>,
    ) -> PyResult<botrail_scene::iomap::IoDerivation> {
        let scene = self.hub.authored_snapshot();
        let names: Option<Vec<&str>> = sequences
            .as_ref()
            .map(|v| v.iter().map(String::as_str).collect());
        botrail_scene::iomap::derive(&scene, names.as_deref())
            .map_err(|e| PyValueError::new_err(e.to_string()))
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
    #[pyo3(signature = (robot = None, base_position = None, base_quaternion = None, name = None))]
    fn new(
        robot: Option<&Robot>,
        base_position: Option<[f64; 3]>,
        base_quaternion: Option<[f64; 4]>,
        name: Option<&str>,
    ) -> PyResult<Self> {
        let Some(robot) = robot else {
            // A cell with no robot in it — devices, vehicles and obstacles
            // only. The base/name kwargs describe the robot, so passing
            // them without one is a confusion worth naming.
            if base_position.is_some() || base_quaternion.is_some() || name.is_some() {
                return Err(PyValueError::new_err(
                    "base_position/base_quaternion/name describe the robot; \
                     pass them with one, or place robots via add_robot",
                ));
            }
            return Ok(Scene {
                hub: Arc::new(SceneHub::new(botrail_scene::Scene::empty())),
                robot: None,
            });
        };
        let base = pose_from(base_position.unwrap_or([0.0; 3]), base_quaternion);
        let mut scene = botrail_scene::Scene::with_base(robot.inner.clone(), base);
        if let Some(name) = name {
            scene.rename_robot(0, name);
        }
        Ok(Scene {
            hub: Arc::new(SceneHub::new(scene)),
            robot: Some(robot.clone()),
        })
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
    ///
    /// With a `gait` (a `bt.Gait`) the robot *is* the vehicle's legs: it
    /// walks whenever the vehicle drives, and stands in the gait's stance
    /// when it does not. The offset then defaults to the one that puts the
    /// stance feet on the vehicle plane, and the robot is set to its stance.
    /// `spin` is presentation: `{joint: rad/s}` turned while the vehicle
    /// is off its starting ground or moving — a multirotor's propellers,
    /// signed so counter-rotating pairs read right. Continuous joints
    /// only; no check reads the phase (the collision stays the swept
    /// solid the catalog authors).
    #[pyo3(signature = (device, offset_position = None, offset_quaternion = None, robot = None, gait = None, spin = None))]
    fn mount_robot(
        &self,
        device: &str,
        offset_position: Option<[f64; 3]>,
        offset_quaternion: Option<[f64; 4]>,
        robot: Option<&str>,
        gait: Option<&Bound<'_, PyAny>>,
        spin: Option<std::collections::BTreeMap<String, f64>>,
    ) -> PyResult<()> {
        let index = self.resolve_robot(robot)?;
        let gait = gait.map(gait_from_py).transpose()?;
        let offset = match (offset_position, offset_quaternion, &gait) {
            // Derived from the stance: the feet on the floor.
            (None, None, Some(_)) => None,
            (position, quaternion, _) => Some(pose_from(position.unwrap_or([0.0; 3]), quaternion)),
        };
        let spin = spin.map(|m| m.into_iter().collect()).unwrap_or_default();
        self.hub
            .mount_robot_with(index, device, offset, gait, spin)
            .map_err(scene_err)
    }

    #[getter]
    fn robot(&self) -> PyResult<Robot> {
        self.robot
            .clone()
            .ok_or_else(|| PyValueError::new_err("scene has no robot; add one with add_robot"))
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

    /// Imports a URDF or xacro as **scenery**: every visual becomes an
    /// obstacle named `<prefix>/<link>`, posed at the model's zero
    /// configuration and placed at `position` / `quaternion`. Links that
    /// carry no geometry become named frames (see `frame()`), so a file can
    /// name where the next thing mounts. `args` fills the file's
    /// `$(arg …)` substitutions, which is what lets one parametric file
    /// draw every size a product is sold in. `geometry="collision"` reads
    /// the collision shapes instead of the visuals. Returns the obstacle
    /// names it added.
    ///
    /// This is furniture, not a machine: joints are taken at zero and
    /// nothing here articulates. A robot is `Robot.from_urdf` /
    /// `Robot.from_xacro` and `add_robot`.
    // The argument list is the Python signature; grouping it would change
    // the call, not tidy it.
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (
        path, prefix = None, position = None, quaternion = None, args = None,
        geometry = "visual", frames = true, package_paths = None
    ))]
    fn load_urdf(
        &self,
        path: PathBuf,
        prefix: Option<String>,
        position: Option<[f64; 3]>,
        quaternion: Option<[f64; 4]>,
        args: Option<std::collections::HashMap<String, String>>,
        geometry: &str,
        frames: bool,
        package_paths: Option<std::collections::HashMap<String, PathBuf>>,
    ) -> PyResult<Vec<String>> {
        let collision = match geometry {
            "visual" => false,
            "collision" => true,
            other => {
                return Err(PyValueError::new_err(format!(
                    "geometry must be \"visual\" or \"collision\", not `{other}`"
                )))
            }
        };
        let options = botrail_model::ModelOptions {
            package_paths: package_paths.unwrap_or_default(),
            xacro_args: args.unwrap_or_default(),
        };
        // Plain URDF passes through the xacro reader unchanged, so one path
        // serves both.
        let model = RobotModel::from_xacro_file_with(&path, &options).map_err(model_err)?;
        let poses = botrail_kin::forward_kinematics(&model, &vec![0.0; model.dof()])
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let base = pose_from(position.unwrap_or([0.0; 3]), quaternion);
        let prefix = match prefix {
            Some(p) if !p.is_empty() => format!("{}/", p.trim_end_matches('/')),
            _ => String::new(),
        };
        let mut batch = Vec::new();
        let mut named = Vec::new();
        for (index, link) in model.links.iter().enumerate() {
            let shapes = if collision {
                &link.collisions
            } else {
                &link.visuals
            };
            let world = base * poses[index];
            if shapes.is_empty() {
                if frames {
                    named.push((format!("{prefix}{}", link.name), world));
                }
                continue;
            }
            for (i, shape) in shapes.iter().enumerate() {
                let name = if shapes.len() == 1 {
                    format!("{prefix}{}", link.name)
                } else {
                    format!("{prefix}{}/{i}", link.name)
                };
                batch.push(botrail_scene::ObstacleSpec {
                    name,
                    geometry: shape.geometry.clone(),
                    pose: world * shape.origin,
                    color: shape.color,
                    material: None,
                });
            }
        }
        let names = self.hub.add_obstacles(batch).map_err(scene_err)?;
        self.hub.add_frames(named);
        Ok(names)
    }

    /// Registers (or updates) a named world frame.
    #[pyo3(signature = (name, position, quaternion = None))]
    fn add_frame(&self, name: &str, position: [f64; 3], quaternion: Option<[f64; 4]>) {
        self.hub.add_frame(name, pose_from(position, quaternion));
    }

    /// Removes a named frame.
    fn remove_frame(&self, name: &str) -> PyResult<()> {
        self.hub.remove_frame(name).map_err(scene_err)
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

    /// Marks an obstacle's top face as a place a walking machine's feet
    /// may stand — a stair tread, a mezzanine slab. Footfalls snap onto
    /// it and the walker may touch it (nobody collision-checks a floor
    /// against the machine standing on it); everything else still
    /// collides with it normally. Only an upright box (yaw rotation is
    /// fine) can be walkable.
    #[pyo3(signature = (name, walkable = true))]
    fn set_obstacle_walkable(&self, name: &str, walkable: bool) -> PyResult<()> {
        self.hub
            .set_obstacle_walkable(name, walkable)
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

    /// Attaches a colour key to an obstacle whose colours mean something
    /// — `stops` is a list of `((r, g, b), label)` swatches top to bottom,
    /// linear RGB, empty labels allowed — or clears it with `stops=None`.
    /// The studio draws it beside the viewport while the obstacle is in
    /// the scene. Presentation only.
    #[pyo3(signature = (name, title = "", stops = None))]
    fn set_obstacle_legend(
        &self,
        name: &str,
        title: &str,
        stops: Option<Vec<LegendStopArg>>,
    ) -> PyResult<()> {
        let legend = stops.map(|stops| botrail_scene::Legend {
            title: title.to_string(),
            stops: stops
                .into_iter()
                .map(|((r, g, b), label)| botrail_scene::LegendStop {
                    color: [r, g, b],
                    label,
                })
                .collect(),
        });
        self.hub
            .set_obstacle_legend(name, legend)
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

    /// Binds a spray-cone effect to a signal at a robot's TCP: while the
    /// signal is true during playback, the studio draws a translucent
    /// cone `length` long and `radius` wide at its base along the TCP's
    /// spray direction (its -Z), and USD export carries a beam of the
    /// same size with animated visibility. Bind it to the effective
    /// trigger a timeline writes with `with_trigger_signal` so it
    /// follows what actually sprayed rather than the enable alone. Pure
    /// presentation, like `add_weld_flash`.
    #[pyo3(signature = (name, signal, robot, length = 0.25, radius = 0.08))]
    fn add_spray_cone(
        &self,
        name: &str,
        signal: &str,
        robot: &str,
        length: f64,
        radius: f64,
    ) -> PyResult<()> {
        self.hub
            .add_spray_cone(name, signal, robot, length, radius)
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
        self.hub
            .upsert_sensor(botrail_scene::seq::Sensor {
                name: name.to_string(),
                kind: botrail_scene::seq::SensorKind::Zone {
                    pose: pose_from(position, quaternion),
                    size: nalgebra::Vector3::new(size[0], size[1], size[2]),
                },
                watch: sensor_watch(watch, watch_robot, watch_robots),
                mount,
            })
            .map_err(scene_err)
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
        self.hub
            .upsert_sensor(botrail_scene::seq::Sensor {
                name: name.to_string(),
                kind: botrail_scene::seq::SensorKind::Beam {
                    from: nalgebra::Point3::new(frm[0], frm[1], frm[2]),
                    to: nalgebra::Point3::new(to[0], to[1], to[2]),
                    radius,
                },
                watch: sensor_watch(watch, watch_robot, watch_robots),
                mount,
            })
            .map_err(scene_err)
    }

    /// Adds a vision presence sensor looking through `camera`: its name
    /// becomes a read-only input signal, ON while a watched body overlaps
    /// the camera's view frustum. `detect_range` narrows the detection
    /// band along the view axis (default: the camera's near/far clip);
    /// `occlusion` (default on) ray-tests each candidate's origin against
    /// the other obstacles, so a body hidden behind another does not trip
    /// it. Geometry only — no pixels are rendered or interpreted, and
    /// robot links (when watched) detect by overlap alone.
    #[pyo3(signature = (name, camera, watch = None, watch_robot = false, watch_robots = None, detect_range = None, occlusion = true))]
    fn add_vision_sensor(
        &self,
        name: &str,
        camera: &str,
        watch: Option<Vec<String>>,
        watch_robot: bool,
        watch_robots: Option<Vec<String>>,
        detect_range: Option<[f64; 2]>,
        occlusion: bool,
    ) -> PyResult<()> {
        self.hub
            .upsert_sensor(botrail_scene::seq::Sensor {
                name: name.to_string(),
                kind: botrail_scene::seq::SensorKind::Vision {
                    camera: camera.to_string(),
                    detect_range,
                    occlusion,
                },
                watch: sensor_watch(watch, watch_robot, watch_robots),
                mount: None,
            })
            .map_err(scene_err)
    }

    fn remove_sensor(&self, name: &str) -> PyResult<()> {
        self.hub.remove_sensor(name).map_err(scene_err)
    }

    #[getter]
    fn sensor_names(&self) -> Vec<String> {
        self.hub.sensor_names()
    }

    /// Adds a camera: a named viewpoint with pinhole optics, drawn as a
    /// frustum in the studio. Presentation only — it publishes no signal
    /// and never affects planning or the cycle. `position`/`quaternion`
    /// are in the mount frame (-Z looks, +Y is image-up); `look_at` aims
    /// the camera at a world point instead of giving a quaternion. Mount
    /// it with `mount=` (a vehicle device) or `robot=`/`link=` (a wrist
    /// camera); default is a world fixture. `fov` is the horizontal field
    /// of view in degrees (default 60); `resolution` sets the frustum
    /// aspect and the pixel size of exports (default 1280x720).
    ///
    /// `from_catalog=` names a `sensor.camera` package: its flat specs
    /// become the optics defaults (fov/resolution and, from the range
    /// specs, near/far), explicit arguments still win, and the package's
    /// identity lands on the BOM (`set_part(kind="camera")`). With an
    /// explicit pose, `position`/`quaternion` place the package's *mount
    /// face* and the optical axis follows the package's own calibration
    /// (`frames.camera_frames`); `look_at` aims the optical axis itself.
    #[pyo3(signature = (name, position = [0.0, 0.0, 0.0], quaternion = None, look_at = None, fov = None, resolution = None, near = None, far = None, mount = None, robot = None, link = None, from_catalog = None, revision = None))]
    #[allow(clippy::too_many_arguments)]
    fn add_camera(
        &self,
        py: Python<'_>,
        name: &str,
        position: [f64; 3],
        quaternion: Option<[f64; 4]>,
        look_at: Option<[f64; 3]>,
        fov: Option<f64>,
        resolution: Option<[u32; 2]>,
        near: Option<f64>,
        far: Option<f64>,
        mount: Option<String>,
        robot: Option<String>,
        link: Option<String>,
        from_catalog: Option<String>,
        revision: Option<String>,
    ) -> PyResult<()> {
        let package = from_catalog
            .as_deref()
            .map(|query| catalog::camera_from_catalog(py, query, revision.as_deref()))
            .transpose()?;
        let fov = fov
            .or(package.as_ref().and_then(|p| p.fov_h_deg))
            .unwrap_or(60.0);
        let resolution = resolution
            .or(package.as_ref().and_then(|p| p.resolution))
            .unwrap_or([1280, 720]);
        let near = near
            .or(package.as_ref().and_then(|p| p.near))
            .unwrap_or(0.05);
        let far = far.or(package.as_ref().and_then(|p| p.far)).unwrap_or(30.0);
        let camera_mount = match (mount, robot, link) {
            (None, None, None) => botrail_scene::seq::CameraMount::World,
            (Some(device), None, None) => botrail_scene::seq::CameraMount::Vehicle { device },
            (None, Some(robot), Some(link)) => {
                botrail_scene::seq::CameraMount::Link { robot, link }
            }
            (None, Some(_), None) => {
                return Err(PyValueError::new_err(
                    "a robot-mounted camera needs link= (the link it is bolted to)",
                ))
            }
            _ => {
                return Err(PyValueError::new_err(
                    "pass either mount= (a vehicle) or robot=/link=, not both",
                ))
            }
        };
        let pose = match look_at {
            Some(target) => {
                if quaternion.is_some() {
                    return Err(PyValueError::new_err(
                        "pass either quaternion= or look_at=, not both",
                    ));
                }
                if !matches!(camera_mount, botrail_scene::seq::CameraMount::World) {
                    return Err(PyValueError::new_err(
                        "look_at= aims a world fixture; a mounted camera moves, so give \
                         its quaternion in the mount frame instead",
                    ));
                }
                look_at_pose(position, target)?
            }
            // The given pose places the package's mount face; the optical
            // axis follows its calibration. (`look_at` above aims the
            // optical axis itself, so the offset does not apply there.)
            None => match package.as_ref().and_then(|p| p.optical_offset) {
                Some(offset) => pose_from(position, quaternion) * offset,
                None => pose_from(position, quaternion),
            },
        };
        self.hub
            .upsert_camera(botrail_scene::seq::Camera {
                name: name.to_string(),
                mount: camera_mount,
                pose,
                fov_deg: fov,
                resolution,
                near,
                far,
            })
            .map_err(scene_err)?;
        if let Some(pkg) = package {
            // The identity a BOM line names it by — the same shape
            // `bt.catalog.Product.identify` writes.
            let mut attributes = std::collections::BTreeMap::new();
            for (key, value) in &pkg.meta.specs {
                attributes.insert(key.clone(), botrail_scene::part::PartAttr::Number(*value));
            }
            let part = botrail_scene::part::Part {
                catalog: Some(botrail_scene::part::CatalogRef {
                    id: pkg.id,
                    revision: Some(pkg.revision),
                }),
                manufacturer: pkg.meta.manufacturer,
                model: pkg.meta.product,
                category: pkg.meta.category,
                description: None,
                qty: 1,
                attributes,
            };
            self.hub
                .set_part(
                    name,
                    Some(botrail_scene::part::PartTargetKind::Camera),
                    part,
                )
                .map_err(scene_err)?;
        }
        Ok(())
    }

    fn remove_camera(&self, name: &str) -> PyResult<()> {
        self.hub.remove_camera(name).map_err(scene_err)
    }

    #[getter]
    fn camera_names(&self) -> Vec<String> {
        self.hub.camera_names()
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

    /// Adds a lift (elevator): the `car` obstacles ride along `axis`
    /// between named `stops`, and whatever the capture zone holds when
    /// the ride is commanded rides too — loose parts by origin, and
    /// vehicles whole (body, deck load, mounted robot). Command it with
    /// `bt.seq.move_to(name, "2F")` and await `bt.seq.device_done(name)`.
    /// The zone (like the car) is authored where the car stands at
    /// `start`; a vehicle half out of it refuses to board by name.
    /// Doors are ordinary authoring — an `add_linear_axis` panel and a
    /// signal — not part of the device. Car entries name obstacles
    /// exactly, or as subtree prefixes.
    #[pyo3(signature = (name, car, zone_position, zone_size, stops, speed = 0.5,
                        axis = [0.0, 0.0, 1.0], zone_quaternion = None, start = None))]
    #[allow(clippy::too_many_arguments)]
    fn add_lift(
        &self,
        name: &str,
        car: Vec<String>,
        zone_position: [f64; 3],
        zone_size: [f64; 3],
        stops: std::collections::BTreeMap<String, f64>,
        speed: f64,
        axis: [f64; 3],
        zone_quaternion: Option<[f64; 4]>,
        start: Option<String>,
    ) -> PyResult<()> {
        let axis = nalgebra::Unit::try_new(nalgebra::Vector3::new(axis[0], axis[1], axis[2]), 1e-9)
            .ok_or_else(|| PyValueError::new_err("axis must be a nonzero vector"))?;
        if stops.is_empty() {
            return Err(PyValueError::new_err(
                "stops is empty; name at least the stop the car starts at",
            ));
        }
        if !(speed.is_finite() && speed > 0.0) {
            return Err(PyValueError::new_err(format!(
                "speed must be positive, got {speed}"
            )));
        }
        // The default start is the lowest stop (deterministic).
        let start = match start {
            Some(s) => {
                if !stops.contains_key(&s) {
                    return Err(PyValueError::new_err(format!(
                        "start `{s}` is not a stop (stops: {})",
                        stops
                            .keys()
                            .map(|k| format!("`{k}`"))
                            .collect::<Vec<_>>()
                            .join(", ")
                    )));
                }
                s
            }
            None => stops
                .iter()
                .min_by(|a, b| {
                    a.1.partial_cmp(b.1)
                        .unwrap_or(std::cmp::Ordering::Equal)
                        .then(a.0.cmp(b.0))
                })
                .map(|(n, _)| n.clone())
                .expect("stops is non-empty"),
        };
        // Car entries: exact obstacle names, or subtree prefixes.
        let known = self.hub.obstacle_names();
        let mut members: Vec<String> = Vec::new();
        for entry in &car {
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
                    "car entry `{entry}` matches no obstacle (exactly or as a prefix)"
                )));
            }
            members.extend(hits);
        }
        let mut seen = std::collections::HashSet::new();
        members.retain(|m| seen.insert(m.clone()));
        // The zone is authored where the car stands at `start`; store it
        // at the reference position so it rides `axis · position`.
        let start_value = stops[&start];
        let mut zone_pose = pose_from(zone_position, zone_quaternion);
        zone_pose.translation.vector -= axis.into_inner() * start_value;
        self.hub.upsert_device(botrail_scene::seq::Device {
            name: name.to_string(),
            kind: botrail_scene::seq::DeviceKind::Lift {
                car: members,
                zone_pose,
                zone_size: nalgebra::Vector3::new(zone_size[0], zone_size[1], zone_size[2]),
                axis,
                speed,
                stops: stops.into_iter().collect(),
                start,
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
    ///
    /// Waypoints are `(x, y)` or `(x, y, z)`: z is the floor height on the
    /// guidance surface, so a ramp climbs with its waypoints (the body
    /// stays level, and speed is spent along the 3D path). A path that
    /// climbs needs `max_grade` — the steepest rise over horizontal run
    /// the machine may take (0.10 = 10 %); without it only level paths
    /// pass validation.
    ///
    /// `drive="aerial"` makes the machine a multirotor: z is its own axis
    /// (any climb, no grade rule, vertical legs fly — a ground station
    /// under an overhead waypoint *is* the takeoff), `speed` is the
    /// horizontal cruise and each leg's clock is the slower axis,
    /// `max(run/speed, rise/climb_speed (or descent_speed))`. The nose
    /// faces each leg's course, or holds `fixed_yaw` the whole flight.
    #[pyo3(signature = (name, body, path, stations, speed = 0.5,
                        turn_speed = std::f64::consts::FRAC_PI_2,
                        start = None, ring = false, allow_reverse = false,
                        max_grade = None, drive = "differential",
                        climb_speed = None, descent_speed = None,
                        fixed_yaw = None,
                        tray_position = None, tray_size = None,
                        tray_quaternion = None))]
    #[allow(clippy::too_many_arguments)]
    fn add_vehicle(
        &self,
        name: &str,
        body: Vec<String>,
        path: Vec<Vec<f64>>,
        stations: std::collections::BTreeMap<String, usize>,
        speed: f64,
        turn_speed: f64,
        start: Option<String>,
        ring: bool,
        allow_reverse: bool,
        max_grade: Option<f64>,
        drive: &str,
        climb_speed: Option<f64>,
        descent_speed: Option<f64>,
        fixed_yaw: Option<f64>,
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
        if let Some(g) = max_grade {
            if !(g.is_finite() && g > 0.0) {
                return Err(PyValueError::new_err(format!(
                    "max_grade must be positive (rise over run), got {g}"
                )));
            }
        }
        let mut waypoints: Vec<nalgebra::Point3<f64>> = Vec::with_capacity(path.len());
        for (i, p) in path.iter().enumerate() {
            match p.as_slice() {
                [x, y] => waypoints.push(nalgebra::Point3::new(*x, *y, 0.0)),
                [x, y, z] => waypoints.push(nalgebra::Point3::new(*x, *y, *z)),
                other => {
                    return Err(PyValueError::new_err(format!(
                        "path waypoint {i} needs 2 or 3 coordinates (x, y[, z]), got {}",
                        other.len()
                    )))
                }
            }
        }
        let drive = match drive {
            "differential" => {
                if climb_speed.is_some() || descent_speed.is_some() || fixed_yaw.is_some() {
                    return Err(PyValueError::new_err(
                        "climb_speed / descent_speed / fixed_yaw belong to drive=\"aerial\"",
                    ));
                }
                botrail_scene::seq::Drive::Differential {
                    allow_reverse,
                    max_grade,
                }
            }
            "holonomic" => {
                if climb_speed.is_some() || descent_speed.is_some() || fixed_yaw.is_some() {
                    return Err(PyValueError::new_err(
                        "climb_speed / descent_speed / fixed_yaw belong to drive=\"aerial\"",
                    ));
                }
                if allow_reverse {
                    return Err(PyValueError::new_err(
                        "allow_reverse is a differential-drive idea; a holonomic \
                         machine never turns in the first place",
                    ));
                }
                botrail_scene::seq::Drive::Holonomic { max_grade }
            }
            "aerial" => {
                if allow_reverse || max_grade.is_some() {
                    return Err(PyValueError::new_err(
                        "allow_reverse / max_grade belong to a ground drive; an aerial \
                         machine flies its legs",
                    ));
                }
                let (Some(climb), Some(descent)) = (climb_speed, descent_speed) else {
                    return Err(PyValueError::new_err(
                        "drive=\"aerial\" needs climb_speed and descent_speed (m/s)",
                    ));
                };
                if !(climb.is_finite() && climb > 0.0 && descent.is_finite() && descent > 0.0) {
                    return Err(PyValueError::new_err(format!(
                        "climb_speed / descent_speed must be positive, got {climb} / {descent}"
                    )));
                }
                botrail_scene::seq::Drive::Aerial {
                    climb_speed: climb,
                    descent_speed: descent,
                    yaw: fixed_yaw
                        .map(botrail_scene::seq::AerialYaw::Fixed)
                        .unwrap_or(botrail_scene::seq::AerialYaw::Course),
                }
            }
            other => {
                return Err(PyValueError::new_err(format!(
                    "drive must be \"differential\", \"holonomic\" or \"aerial\", \
                     got {other:?}"
                )))
            }
        };
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
                    waypoints,
                    stations: stations.into_iter().collect(),
                    ring,
                },
                body: members,
                speed,
                turn_speed,
                start,
                drive,
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

    /// The cell's I/O points, *derived* from how the sequences use the
    /// scene's names (nothing to author): sensors are inputs, coils and
    /// device commands are outputs, signals read or written across
    /// controllers are handshake wires, robots driven from another host
    /// get start/done points. `sequences` picks the program set (default:
    /// every sequence — pass what you would pass to `simulate_sequences`
    /// when alternative programs coexist). See docs/guides/io-map.md.
    #[pyo3(signature = (sequences = None))]
    fn io_points(&self, sequences: Option<Vec<String>>) -> PyResult<Vec<IoPoint>> {
        let d = self.derive_io(sequences)?;
        Ok(d.points.into_iter().map(|p| IoPoint { inner: p }).collect())
    }

    /// Lint findings over the derived I/O map: name clashes, unreferenced
    /// definitions, numeric (word/analog) points, programs on the
    /// implicit cell host. `assert scene.io_report().errors() == []` is
    /// the CI form.
    #[pyo3(signature = (sequences = None))]
    fn io_report(&self, sequences: Option<Vec<String>>) -> PyResult<IoReport> {
        let d = self.derive_io(sequences)?;
        Ok(IoReport { inner: d.report })
    }

    /// The I/O list as text: `format` is `"csv"`, `"md"` (Markdown table)
    /// or `"json"` (raw fields, step indices included).
    #[pyo3(signature = (format = "csv", sequences = None))]
    fn io_list(&self, format: &str, sequences: Option<Vec<String>>) -> PyResult<String> {
        let d = self.derive_io(sequences)?;
        render_io(&d, format)
    }

    /// Writes the I/O list to `path`; the format follows the extension
    /// (`.csv`, `.md`, `.json`).
    #[pyo3(signature = (path, sequences = None))]
    fn export_io_list(&self, path: PathBuf, sequences: Option<Vec<String>>) -> PyResult<()> {
        let format = match path.extension().and_then(|e| e.to_str()) {
            Some("csv") => "csv",
            Some("md") | Some("markdown") => "md",
            Some("json") => "json",
            other => {
                return Err(PyValueError::new_err(format!(
                    "export_io_list: unknown extension {:?} — use .csv, .md or .json",
                    other.unwrap_or("")
                )))
            }
        };
        let d = self.derive_io(sequences)?;
        let text = render_io(&d, format)?;
        std::fs::write(&path, text)
            .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))
    }

    /// Declares a controller / I/O node of the cell's assignment layer:
    /// `kind` is `"plc"`, `"safety_plc"`, `"remote_io"`,
    /// `"robot_controller"` (with `robots=[...]`) or `"other"`.
    /// `programs` lists the sequences this node runs (unlisted programs
    /// are placed implicitly — see the I/O map guide); `uplink` is the
    /// parent node (`"PLC1"` or `("PLC1", "PROFINET")`) whose I/O a remote
    /// station or safety module belongs to; `channels` are the dicts the
    /// `bt.io` templates build (`bt.io.di8(base="%IX0.0") + bt.io.do8(...)`
    /// or `bt.io.ur_standard()`).
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (name, kind = "plc", robots = None, programs = None, uplink = None,
        channels = None, place = None, model = None, label = None))]
    fn add_io_node(
        &self,
        name: &str,
        kind: &str,
        robots: Option<Vec<String>>,
        programs: Option<Vec<String>>,
        uplink: Option<Bound<'_, PyAny>>,
        channels: Option<Vec<Bound<'_, PyAny>>>,
        place: Option<String>,
        model: Option<String>,
        label: Option<String>,
    ) -> PyResult<()> {
        use botrail_scene::iomap::{IoNode, IoNodeKind, Uplink};
        let kind = match kind {
            "plc" => IoNodeKind::Plc,
            "safety_plc" => IoNodeKind::SafetyPlc,
            "remote_io" => IoNodeKind::RemoteIo,
            "robot_controller" => IoNodeKind::RobotController {
                robots: robots.clone().unwrap_or_default(),
            },
            "other" => IoNodeKind::Other {
                label: label.clone().unwrap_or_else(|| "other".to_string()),
            },
            other => {
                return Err(PyValueError::new_err(format!(
                    "unknown I/O node kind {other:?} (plc, safety_plc, remote_io, robot_controller, other)"
                )))
            }
        };
        if kind.as_str() != "robot_controller" && robots.as_ref().is_some_and(|r| !r.is_empty()) {
            return Err(PyValueError::new_err(
                "robots= only applies to kind=\"robot_controller\"",
            ));
        }
        if let IoNodeKind::RobotController { robots } = &kind {
            if robots.is_empty() {
                return Err(PyValueError::new_err(
                    "a robot_controller node needs robots=[...] — the arms it drives",
                ));
            }
        }
        let uplink = match uplink {
            None => None,
            Some(obj) => Some(if let Ok(parent) = obj.extract::<String>() {
                Uplink { parent, bus: None }
            } else if let Ok((parent, bus)) = obj.extract::<(String, String)>() {
                Uplink {
                    parent,
                    bus: Some(bus),
                }
            } else {
                return Err(PyValueError::new_err(
                    "uplink= is a node name or a (node, bus) pair",
                ));
            }),
        };
        let channels = channels
            .unwrap_or_default()
            .iter()
            .map(channel_from_py)
            .collect::<PyResult<Vec<_>>>()?;
        self.hub
            .upsert_io_node(IoNode {
                name: name.to_string(),
                kind,
                programs: programs.unwrap_or_default(),
                uplink,
                channels,
                place,
                model,
            })
            .map_err(scene_err)
    }

    /// Removes a node and every binding on it.
    fn remove_io_node(&self, name: &str) -> PyResult<()> {
        self.hub.remove_io_node(name).map_err(scene_err)
    }

    // ------------------------------------------------------------- parts

    /// Pins a part — what the thing *is* commercially — to a resident or
    /// group by name: a robot, a device, a sensor, an I/O node, an
    /// obstacle, or an obstacle group (everything under `name/` — an
    /// imported subtree, a generated fence). Identity is optional and
    /// free-form: `catalog` (`"id"` or `"id@revision"` or `(id,
    /// revision)`), `manufacturer`, `model`, `category` (`"conveyor"`,
    /// `"structure.fence"`, ...), `description`, `qty` (how many the
    /// target stands for), and any further keywords or `attributes={...}`
    /// as free attributes (numbers are summed by `bom().total(key)`,
    /// text is carried). Pass `kind=` (`"robot"`, `"device"`, `"sensor"`,
    /// `"io_node"`, `"obstacle"`, `"group"`) when a name lives in several
    /// name spaces. Re-pinning replaces. Returns the kind resolved. The
    /// BOM (`bom()`) is derived from these plus the catalog identity of
    /// robots and tools.
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (name, *, kind = None, catalog = None, manufacturer = None, model = None,
        category = None, description = None, qty = 1, attributes = None, **extra))]
    fn set_part(
        &self,
        name: &str,
        kind: Option<&str>,
        catalog: Option<Bound<'_, PyAny>>,
        manufacturer: Option<String>,
        model: Option<String>,
        category: Option<String>,
        description: Option<String>,
        qty: u32,
        attributes: Option<Bound<'_, PyDict>>,
        extra: Option<Bound<'_, PyDict>>,
    ) -> PyResult<String> {
        use botrail_scene::part::{CatalogRef, Part, PartTargetKind};
        let kind = match kind {
            None => None,
            Some(text) => Some(PartTargetKind::parse(text).ok_or_else(|| {
                PyValueError::new_err(format!(
                    "set_part: unknown kind {text:?} — use \"robot\", \"device\", \"sensor\", \
                     \"io_node\", \"obstacle\" or \"group\""
                ))
            })?),
        };
        let catalog = match catalog {
            None => None,
            Some(value) => Some(if let Ok(text) = value.extract::<String>() {
                CatalogRef::parse(&text)
            } else if let Ok((id, revision)) = value.extract::<(String, Option<String>)>() {
                CatalogRef { id, revision }
            } else {
                return Err(PyValueError::new_err(
                    "set_part: catalog must be \"id\", \"id@revision\" or (id, revision)",
                ));
            }),
        };
        let mut part = Part {
            catalog,
            manufacturer,
            model,
            category,
            description,
            qty,
            attributes: BTreeMap::new(),
        };
        for dict in [attributes, extra].into_iter().flatten() {
            for (key, value) in dict.iter() {
                let key: String = key.extract().map_err(|_| {
                    PyValueError::new_err("set_part: attribute names must be strings")
                })?;
                part.attributes
                    .insert(key.clone(), part_attr(&key, &value)?);
            }
        }
        self.hub
            .set_part(name, kind, part)
            .map(|k| k.as_str().to_string())
            .map_err(scene_err)
    }

    /// Unpins the part on `name`.
    fn remove_part(&self, name: &str) -> PyResult<()> {
        self.hub.remove_part(name).map_err(scene_err)
    }

    /// The part pinned to `name` as a dict (`target`, `kind`, `catalog`,
    /// `manufacturer`, `model`, `category`, `description`, `qty`,
    /// `attributes`), or `None`.
    fn part(&self, py: Python<'_>, name: &str) -> PyResult<Option<PyObject>> {
        self.hub
            .part(name)
            .map(|entry| part_entry_dict(py, &entry))
            .transpose()
    }

    /// Every pinned part, in authoring order (see `part()` for the shape).
    fn parts(&self, py: Python<'_>) -> PyResult<Vec<PyObject>> {
        self.hub
            .parts()
            .iter()
            .map(|entry| part_entry_dict(py, entry))
            .collect()
    }

    /// The bill of materials derived from the scene: robots and their
    /// tools (catalog identity when loaded from the catalog), conveyors /
    /// axes / vehicles, sensors and I/O nodes — each listed whether or
    /// not it has been identified — plus every obstacle or group a part
    /// was pinned to. Identical products merge into one row with the
    /// quantity summed.
    fn bom(&self) -> Bom {
        Bom {
            inner: self.hub.bom(),
        }
    }

    /// Writes the BOM to `path`; the format follows the extension
    /// (`.csv`, `.md`, `.json`) unless `format` says otherwise.
    #[pyo3(signature = (path, format = None))]
    fn export_bom(&self, path: PathBuf, format: Option<&str>) -> PyResult<()> {
        self.bom().save(path, format)
    }

    // ------------------------------------------------------- PLCopen XML

    /// The sequences as PLCopen XML (IEC 61131-10, TC6 v2.01): one SFC
    /// program per sequence (`sequences=` a subset; default all), steps
    /// with their entry actions and transitions, `select` as a selection
    /// divergence, and the cycle jump at the end (`cycle=False` parks the
    /// program in a final step). Conditions are ST expressions; device
    /// coils and commands write the I/O map's variables (declared once as
    /// resource globals, with `AT` addresses from PLC-side bindings);
    /// robot commands call stub function blocks the control engineer
    /// replaces — or the start / done handshake where the map says the
    /// robot is driven from another host. Opens in Beremiz / OpenPLC
    /// Editor. Deterministic (fixed timestamps).
    #[pyo3(signature = (sequences = None, *, name = "cell", cycle = true, task_interval_ms = 10))]
    fn plcopen(
        &self,
        sequences: Option<Vec<String>>,
        name: &str,
        cycle: bool,
        task_interval_ms: u32,
    ) -> PyResult<String> {
        let options = botrail_scene::plcopen::PlcopenOptions {
            sequences,
            name: name.to_string(),
            task_interval_ms,
            cycle,
        };
        botrail_scene::plcopen::render_plcopen(&self.hub.authored_snapshot(), &options)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    /// Writes `plcopen()` to `path` (`.xml`).
    #[pyo3(signature = (path, sequences = None, *, name = "cell", cycle = true, task_interval_ms = 10))]
    fn export_plcopen(
        &self,
        path: PathBuf,
        sequences: Option<Vec<String>>,
        name: &str,
        cycle: bool,
        task_interval_ms: u32,
    ) -> PyResult<()> {
        let text = self.plcopen(sequences, name, cycle, task_interval_ms)?;
        std::fs::write(&path, text)
            .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))
    }

    // ------------------------------------------------------ layout sheet

    /// The plan-view layout sheet as text: `format` is `"svg"` (a
    /// self-contained drawing, `scale` pixels per metre), `"dxf"` (a
    /// minimal R12 file for 2D CAD, in `units` — `"mm"` or `"m"`) or
    /// `"json"` (the drawn items in world metres). The sheet is derived
    /// from the scene: every visible obstacle as its footprint (convex
    /// hulls of primitives, bounding boxes of meshes), robots as base marks
    /// with the catalog reach as a dashed circle, conveyor / sink zones,
    /// axis travel and vehicle routes, sensor zones and beams, named
    /// frames, labels (pinned parts first, then named groups), a metre
    /// grid and the overall dimensions. Anything whose top sits at or
    /// below `ground_z` is floor: drawn faint, left out of the extents.
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (format = "svg", *, scale = 100.0, units = "mm", ground_z = 0.02,
        frames = true, labels = true, reach = true, grid = Some(1.0), title = None))]
    fn layout(
        &self,
        format: &str,
        scale: f64,
        units: &str,
        ground_z: f64,
        frames: bool,
        labels: bool,
        reach: bool,
        grid: Option<f64>,
        title: Option<String>,
    ) -> PyResult<String> {
        if !matches!(units, "mm" | "m") {
            return Err(PyValueError::new_err(format!(
                "layout: units must be \"mm\" or \"m\", got {units:?}"
            )));
        }
        let options = botrail_scene::layout::LayoutOptions {
            ground_z,
            frames,
            labels,
            reach,
            grid,
            title: title.unwrap_or_default(),
        };
        let sheet = self.hub.authored_snapshot().layout(&options);
        match format {
            "svg" => Ok(sheet.to_svg(scale)),
            "dxf" => Ok(sheet.to_dxf(units)),
            "json" => Ok(sheet.to_json()),
            other => Err(PyValueError::new_err(format!(
                "layout: unknown format {other:?} — use \"svg\", \"dxf\" or \"json\""
            ))),
        }
    }

    /// Writes the layout sheet to `path`; the format follows the extension
    /// (`.svg`, `.dxf`, `.json`) unless `format` says otherwise. The other
    /// keywords are `layout()`'s.
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (path, format = None, *, scale = 100.0, units = "mm", ground_z = 0.02,
        frames = true, labels = true, reach = true, grid = Some(1.0), title = None))]
    fn export_layout(
        &self,
        path: PathBuf,
        format: Option<&str>,
        scale: f64,
        units: &str,
        ground_z: f64,
        frames: bool,
        labels: bool,
        reach: bool,
        grid: Option<f64>,
        title: Option<String>,
    ) -> PyResult<()> {
        let format = match format {
            Some(f) => f.to_string(),
            None => match path.extension().and_then(|e| e.to_str()) {
                Some("svg") => "svg".to_string(),
                Some("dxf") => "dxf".to_string(),
                Some("json") => "json".to_string(),
                other => {
                    return Err(PyValueError::new_err(format!(
                        "export_layout: unknown extension {:?} — use .svg, .dxf or .json (or pass format=)",
                        other.unwrap_or("")
                    )))
                }
            },
        };
        let text = self.layout(
            &format, scale, units, ground_z, frames, labels, reach, grid, title,
        )?;
        std::fs::write(&path, text)
            .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))
    }

    /// The plan-view extent of the equipment as a dict — `min`, `max`
    /// (x, y in metres), `width`, `depth`, `area` (m²), `height` (tallest
    /// non-ground item). Ground is anything whose top is at or below
    /// `ground_z`.
    #[pyo3(signature = (ground_z = 0.02))]
    fn footprint(&self, py: Python<'_>, ground_z: f64) -> PyResult<PyObject> {
        let fp = self.hub.authored_snapshot().footprint(ground_z);
        let d = PyDict::new(py);
        d.set_item("min", fp.min)?;
        d.set_item("max", fp.max)?;
        d.set_item("width", fp.width())?;
        d.set_item("depth", fp.depth())?;
        d.set_item("area", fp.area())?;
        d.set_item("height", fp.height)?;
        Ok(d.into_any().unbind())
    }

    // ------------------------------------------------------- cell report

    /// Gathers the cell report: robots, the cycles you pass (`timelines`
    /// — a `SequenceTimeline`, a list, or a `{name: timeline}` dict; each
    /// with its step spans, robot utilization and, unless
    /// `clearance_dt=None`, the tightest clearance re-scanned against the
    /// scene it was baked from), the I/O map's counts and findings, the
    /// scenario matrix (`scenarios=` a `ScenarioRuns` — its runs also
    /// stand in for `timelines` when none are given), the BOM's totals,
    /// the plan-view footprint, and the SHA-256 of every file in
    /// `deliverables` (paths of things written from this scene). A
    /// reading surface — pytest keeps the `assert`s.
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (timelines = None, *, scenarios = None, deliverables = None,
        clearance_dt = Some(0.01), title = None, ground_z = 0.02))]
    fn cell_report(
        &self,
        py: Python<'_>,
        timelines: Option<Bound<'_, PyAny>>,
        scenarios: Option<PyRef<'_, ScenarioRuns>>,
        deliverables: Option<Vec<PathBuf>>,
        clearance_dt: Option<f64>,
        title: Option<String>,
        ground_z: f64,
    ) -> PyResult<CellReport> {
        use botrail_scene::report::{CellReportInput, CycleInput, Deliverable, ScenarioRow};
        if let Some(dt) = clearance_dt {
            if !(dt.is_finite() && dt > 0.0) {
                return Err(PyValueError::new_err(format!(
                    "clearance_dt must be positive, got {dt}"
                )));
            }
        }
        // Collect (name, timeline pyref) pairs.
        let mut named: Vec<(String, PyRef<'_, SequenceTimeline>)> = Vec::new();
        let explicit = timelines.is_some();
        match timelines {
            None => {}
            Some(value) => {
                if let Ok(tl) = value.extract::<PyRef<'_, SequenceTimeline>>() {
                    let name = tl.inner.sequences.join("+");
                    named.push((name, tl));
                } else if let Ok(dict) = value.downcast::<PyDict>() {
                    for (k, v) in dict.iter() {
                        let name: String = k.extract().map_err(|_| {
                            PyValueError::new_err("cell_report: timeline names must be strings")
                        })?;
                        let tl: PyRef<'_, SequenceTimeline> = v.extract().map_err(|_| {
                            PyValueError::new_err(
                                "cell_report: timelines values must be SequenceTimeline",
                            )
                        })?;
                        named.push((name, tl));
                    }
                } else if let Ok(list) = value.extract::<Vec<PyRef<'_, SequenceTimeline>>>() {
                    for tl in list {
                        let mut name = tl.inner.sequences.join("+");
                        if let Some(sc) = &tl.inner.scenario {
                            name = format!("{name} ({sc})");
                        }
                        named.push((name, tl));
                    }
                } else {
                    return Err(PyValueError::new_err(
                        "cell_report: timelines must be a SequenceTimeline, a list of them, or a {name: timeline} dict",
                    ));
                }
            }
        }
        let mut scenario_rows: Vec<ScenarioRow> = Vec::new();
        if let Some(runs) = &scenarios {
            for (name, tl) in &runs.runs {
                let tl = tl.borrow(py);
                scenario_rows.push(ScenarioRow {
                    name: name.clone(),
                    ok: true,
                    duration: Some(tl.inner.duration),
                    error: None,
                });
                if !explicit {
                    named.push((name.clone(), tl));
                }
            }
            for (name, error) in &runs.failures {
                scenario_rows.push(ScenarioRow {
                    name: name.clone(),
                    ok: false,
                    duration: None,
                    error: Some(error.clone()),
                });
            }
        }
        // Clearances against each timeline's own snapshot.
        let mut clearances: Vec<Option<botrail_scene::verify::Clearance>> = Vec::new();
        for (_, tl) in &named {
            let clearance = match clearance_dt {
                Some(dt) => tl
                    .scene
                    .timeline_min_clearance(&tl.inner, dt)
                    .map_err(|e| PyValueError::new_err(e.to_string()))?,
                None => None,
            };
            clearances.push(clearance);
        }
        let cycles: Vec<CycleInput<'_>> = named
            .iter()
            .zip(clearances)
            .map(|((name, tl), clearance)| CycleInput {
                name: name.clone(),
                timeline: &tl.inner,
                clearance,
            })
            .collect();
        // Deliverable digests: hashlib on the Python side keeps the core
        // dependency-free.
        let mut files = Vec::new();
        for path in deliverables.unwrap_or_default() {
            let bytes = std::fs::read(&path)
                .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))?;
            let digest: String = py
                .import("hashlib")?
                .call_method1("sha256", (pyo3::types::PyBytes::new(py, &bytes),))?
                .call_method0("hexdigest")?
                .extract()?;
            files.push(Deliverable {
                path: path.display().to_string(),
                sha256: Some(digest),
                bytes: Some(bytes.len() as u64),
            });
        }
        let report = self.hub.authored_snapshot().cell_report(CellReportInput {
            title,
            cycles,
            scenarios: scenario_rows,
            deliverables: files,
            ground_z,
        });
        Ok(CellReport { inner: report })
    }

    /// Wires an input point (`"beam_pick"`, `"line"` for a device's
    /// in-position input, `"far.done"` for a robot's done contact) to a
    /// channel of `node`. `invert=True` flips the wire level (NC wiring);
    /// `contact` (`"no"` / `"nc"`), `field` (the device on the far end),
    /// `voltage` / `logic` (`"pnp"` / `"npn"`) and `note` document it.
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (name, node, channel, tag = None, field = None, invert = false, contact = None,
        safety = false, voltage = None, logic = None, note = None))]
    fn bind_input(
        &self,
        name: &str,
        node: &str,
        channel: &str,
        tag: Option<String>,
        field: Option<String>,
        invert: bool,
        contact: Option<&str>,
        safety: bool,
        voltage: Option<f64>,
        logic: Option<&str>,
        note: Option<String>,
    ) -> PyResult<()> {
        self.bind_point(
            botrail_scene::iomap::IoDirection::Input,
            name,
            node,
            channel,
            tag,
            field,
            invert,
            contact,
            safety,
            voltage,
            logic,
            note,
        )
    }

    /// Wires an output point (`"conv"` for a run coil, `"vacuum"` for a
    /// coil, `"line.index"` for an indexed-transfer start, `"far.start"`
    /// for a robot start) to a channel of `node`. Same keywords as
    /// `bind_input`.
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (name, node, channel, tag = None, field = None, invert = false, contact = None,
        safety = false, voltage = None, logic = None, note = None))]
    fn bind_output(
        &self,
        name: &str,
        node: &str,
        channel: &str,
        tag: Option<String>,
        field: Option<String>,
        invert: bool,
        contact: Option<&str>,
        safety: bool,
        voltage: Option<f64>,
        logic: Option<&str>,
        note: Option<String>,
    ) -> PyResult<()> {
        self.bind_point(
            botrail_scene::iomap::IoDirection::Output,
            name,
            node,
            channel,
            tag,
            field,
            invert,
            contact,
            safety,
            voltage,
            logic,
            note,
        )
    }

    /// Drops the binding of an input point — on `node`, or everywhere.
    #[pyo3(signature = (name, node = None))]
    fn unbind_input(&self, name: &str, node: Option<&str>) -> PyResult<usize> {
        let point =
            botrail_scene::iomap::IoPointId::parse(name, botrail_scene::iomap::IoDirection::Input);
        self.hub.unbind_io(&point, node).map_err(scene_err)
    }

    /// Drops the binding of an output point — on `node`, or everywhere.
    #[pyo3(signature = (name, node = None))]
    fn unbind_output(&self, name: &str, node: Option<&str>) -> PyResult<usize> {
        let point =
            botrail_scene::iomap::IoPointId::parse(name, botrail_scene::iomap::IoDirection::Output);
        self.hub.unbind_io(&point, node).map_err(scene_err)
    }

    /// An exception to the derivation, or an unmodelled point. `role` is
    /// `"input"` (an external contact whatever the sequences do),
    /// `"output"` (a coil — also promotes a magazine to a real feeder),
    /// `"internal"` (a relay, no I/O) or `"exclude"` (off the table). A
    /// name the scene does not have becomes a new declared point when
    /// `role` is input or output. `kind` overrides the channel type
    /// (`"safe_di"`, ...), `safety` marks the safety class, `pair` names
    /// the other channel of a two-channel safety input.
    #[pyo3(signature = (name, role = None, kind = None, safety = false, pair = None, note = None))]
    fn declare_io(
        &self,
        name: &str,
        role: Option<&str>,
        kind: Option<&str>,
        safety: bool,
        pair: Option<String>,
        note: Option<String>,
    ) -> PyResult<()> {
        use botrail_scene::iomap::{ChannelKind, DeclRole, IoDecl};
        let role = match role {
            None => None,
            Some(r) => Some(DeclRole::parse(r).ok_or_else(|| {
                PyValueError::new_err(format!(
                    "unknown role {r:?} (input, output, internal, exclude)"
                ))
            })?),
        };
        let kind = match kind {
            None => None,
            Some(k) => Some(ChannelKind::parse(k).ok_or_else(|| {
                PyValueError::new_err(format!(
                    "unknown channel kind {k:?} (di, do, ai, ao, safe_di, safe_do, word)"
                ))
            })?),
        };
        self.hub.declare_io(IoDecl {
            name: name.to_string(),
            role,
            kind,
            safety,
            pair,
            note,
        });
        Ok(())
    }

    fn undeclare_io(&self, name: &str) -> PyResult<()> {
        self.hub.undeclare_io(name).map_err(scene_err)
    }

    /// The assignment layer as authored — nodes, bindings, declarations.
    /// Pass it to `to_script(io=...)` to project a newer assignment onto
    /// a timeline baked earlier.
    fn io_map(&self) -> IoMap {
        IoMap {
            inner: self.hub.io_map(),
        }
    }

    /// Gives every unbound point a channel, deterministically: points in
    /// table order, channels in declaration order, on the point's host
    /// and the stations uplinked to it, first free channel of a compatible
    /// family (safety points prefer safety channels). Existing bindings
    /// are kept; `reassign=True` first drops the bindings an earlier run
    /// placed (hand bindings keep their channels). Points on an
    /// implicit host (`<cell>`, `<robot>`) are not placed — declare the
    /// node that runs their program. Returns the report afterwards.
    #[pyo3(signature = (sequences = None, reassign = false))]
    fn auto_assign_io(&self, sequences: Option<Vec<String>>, reassign: bool) -> PyResult<IoReport> {
        let names: Option<Vec<&str>> = sequences
            .as_ref()
            .map(|v| v.iter().map(String::as_str).collect());
        let report = self
            .hub
            .auto_assign_io(names.as_deref(), reassign)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(IoReport { inner: report })
    }

    /// The electrical topology as text: `format` is `"mermaid"` (a
    /// `flowchart LR` for Markdown), `"dot"` (Graphviz) or `"json"`.
    /// `layers` filters the edges — any of `"functional"`, `"io"`,
    /// `"network"`, `"wiring"`, `"safety"` (default: everything).
    /// Magazines stay out unless `include_cosmetic=True`.
    #[pyo3(signature = (format = "mermaid", sequences = None, layers = None, include_cosmetic = false))]
    fn io_topology(
        &self,
        format: &str,
        sequences: Option<Vec<String>>,
        layers: Option<Vec<String>>,
        include_cosmetic: bool,
    ) -> PyResult<String> {
        let d = self.derive_io(sequences)?;
        let scene = self.hub.authored_snapshot();
        let layers = parse_layers(layers)?;
        let t = botrail_scene::iomap::topology(&scene, &d, include_cosmetic);
        render_topology(&t, &layers, format)
    }

    /// Writes `io_topology` to `path`; the format follows the extension
    /// (`.mmd` / `.md` Mermaid, `.dot` / `.gv` Graphviz, `.json`).
    #[pyo3(signature = (path, sequences = None, layers = None, include_cosmetic = false))]
    fn export_topology(
        &self,
        path: PathBuf,
        sequences: Option<Vec<String>>,
        layers: Option<Vec<String>>,
        include_cosmetic: bool,
    ) -> PyResult<()> {
        let format = match path.extension().and_then(|e| e.to_str()) {
            Some("mmd") | Some("md") | Some("mermaid") => "mermaid",
            Some("dot") | Some("gv") => "dot",
            Some("json") => "json",
            other => {
                return Err(PyValueError::new_err(format!(
                    "export_topology: unknown extension {:?} — use .mmd, .dot or .json",
                    other.unwrap_or("")
                )))
            }
        };
        let text = self.io_topology(format, sequences, layers, include_cosmetic)?;
        std::fs::write(&path, text)
            .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))
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

    /// World pose of `link_name` at joint configuration `joints` — forward
    /// kinematics without moving the robot (its current joints and any
    /// connected studio are untouched). `bt.select.requirements` measures
    /// taught targets through this.
    #[pyo3(signature = (link_name, joints, robot = None))]
    fn link_pose_at(
        &self,
        link_name: &str,
        joints: Vec<f64>,
        robot: Option<&str>,
    ) -> PyResult<([f64; 3], [f64; 4])> {
        let index = self.resolve_robot(robot)?;
        self.hub
            .link_pose_at(index, link_name, &joints)
            .map_err(PyValueError::new_err)
    }

    /// The project (`.botrail` contents) as JSON without writing a file —
    /// the one read-out `bt.select` derives requirements through.
    fn _project_json(&self) -> String {
        self.hub.project_json()
    }

    /// What every bill-of-materials line must be able to do, derived from
    /// the cell (payload from the tool and the grasped parts, reach from
    /// the taught targets, a beam's span, a conveyor's size and load, ...)
    /// and compared with what the chosen part says. Returns a
    /// `bt.select.Requirements` (rows, `findings()`, `to_markdown()`,
    /// `to_json()`); botrail derives and compares — it does not choose.
    #[pyo3(signature = (*, sequences = None, margin = 0.1, timeline = None))]
    fn requirements(
        slf: Py<Self>,
        py: Python<'_>,
        sequences: Option<Vec<String>>,
        margin: f64,
        timeline: Option<Py<PyAny>>,
    ) -> PyResult<Py<PyAny>> {
        let module = py.import("botrail.select")?;
        let kwargs = PyDict::new(py);
        kwargs.set_item("sequences", sequences)?;
        kwargs.set_item("margin", margin)?;
        kwargs.set_item("timeline", timeline)?;
        Ok(module
            .getattr("requirements")?
            .call((slf,), Some(&kwargs))?
            .unbind())
    }

    /// Every static check in one report — the I/O lint, each sequence
    /// walked for dangling references, unidentified equipment lines and
    /// the requirement comparison (`spec_short` / `spec_unknown`). Returns
    /// a `bt.select.CheckReport` (`ok`, `findings`, `to_json()`,
    /// `to_markdown()`); `botrail check` prints the same thing.
    #[pyo3(signature = (*, sequences = None, timeline = None))]
    fn check(
        slf: Py<Self>,
        py: Python<'_>,
        sequences: Option<Vec<String>>,
        timeline: Option<Py<PyAny>>,
    ) -> PyResult<Py<PyAny>> {
        let module = py.import("botrail.select")?;
        let kwargs = PyDict::new(py);
        kwargs.set_item("sequences", sequences)?;
        kwargs.set_item("timeline", timeline)?;
        Ok(module
            .getattr("check")?
            .call((slf,), Some(&kwargs))?
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
    /// Declares a spray applicator under `name` — the dict
    /// `bt.paint.applicator(...)` builds — so brushes can refer to it.
    /// Validated now, not at bake time.
    fn define_applicator(
        &self,
        py: Python<'_>,
        name: &str,
        applicator: Bound<'_, PyAny>,
    ) -> PyResult<()> {
        let gun = applicator_from_py(py, applicator)?;
        self.hub
            .define_applicator(name, gun)
            .map_err(PyValueError::new_err)
    }

    /// Declares a brush: a named process setting a toolpath's strokes run
    /// with — `applicator` (a `define_applicator` name) at `flow` times
    /// its calibrated flow, opened `lead` seconds before each stroke with
    /// this brush begins and closed `lag` seconds after it ends. The
    /// program's own trigger, per stroke: pass the brush on
    /// `ToolpathBuilder.feed(...)` (or `bt.paint.strokes(brush=...)`), and
    /// the film integrator sprays each stroke with it; feed moves that
    /// name no brush in such a path run with the gun off.
    #[pyo3(signature = (name, applicator, flow = 1.0, lead = 0.0, lag = 0.0))]
    fn define_brush(
        &self,
        name: &str,
        applicator: &str,
        flow: f64,
        lead: f64,
        lag: f64,
    ) -> PyResult<()> {
        self.hub
            .define_brush(botrail_scene::coat::Brush {
                name: name.to_string(),
                applicator: applicator.to_string(),
                flow,
                lead,
                lag,
            })
            .map_err(PyValueError::new_err)
    }

    fn remove_applicator(&self, name: &str) -> PyResult<()> {
        if !self.hub.remove_applicator(name) {
            return Err(PyValueError::new_err(format!(
                "unknown applicator `{name}`"
            )));
        }
        Ok(())
    }

    fn remove_brush(&self, name: &str) -> PyResult<()> {
        if !self.hub.remove_brush(name) {
            return Err(PyValueError::new_err(format!("unknown brush `{name}`")));
        }
        Ok(())
    }

    #[getter]
    fn applicator_names(&self) -> Vec<String> {
        self.hub.applicator_names()
    }

    #[getter]
    fn brush_names(&self) -> Vec<String> {
        self.hub.brush_names()
    }

    /// `{"applicator", "flow", "lead", "lag"}` of a declared brush.
    fn brush<'py>(&self, py: Python<'py>, name: &str) -> PyResult<Bound<'py, pyo3::types::PyDict>> {
        let b = self
            .hub
            .brush(name)
            .ok_or_else(|| PyValueError::new_err(format!("unknown brush `{name}`")))?;
        let d = pyo3::types::PyDict::new(py);
        d.set_item("applicator", b.applicator)?;
        d.set_item("flow", b.flow)?;
        d.set_item("lead", b.lead)?;
        d.set_item("lag", b.lag)?;
        Ok(d)
    }

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
        // The check leaves its marks on the path: the studio draws them
        // over the strokes, and a clean re-check wipes them.
        self.hub
            .set_toolpath_marks(name, reach_marks(&report))
            .map_err(PyValueError::new_err)?;
        Ok(ToolpathReport { inner: report })
    }

    /// Checks a toolpath as a spray program against obstacle `target`
    /// before anything is baked: every feed sample (rapids are not
    /// spraying) looks along its spray axis — the TCP's `-Z`, against
    /// the tool axis — and reports standoff and incidence, judged against
    /// `standoff` (acceptable band, meters) and `max_incidence` (steepest
    /// acceptable angle, radians). Pure geometry: no robot is involved,
    /// so this runs before one is chosen. `at` on the issues is meters
    /// along the path.
    ///
    /// Like `check_toolpath`, the findings are drawn on the path in the
    /// studio until the next check or edit of that path.
    #[pyo3(signature = (name, target, standoff = None, max_incidence = std::f64::consts::FRAC_PI_4, max_range = None, step_pos = 0.005, step_rot = 0.05))]
    #[allow(clippy::too_many_arguments)]
    fn check_paint(
        &self,
        name: &str,
        target: &str,
        standoff: Option<(f64, f64)>,
        max_incidence: f64,
        max_range: Option<f64>,
        step_pos: f64,
        step_rot: f64,
    ) -> PyResult<PaintReport> {
        let limits = paint_limits(standoff, max_incidence, max_range)?;
        let options = botrail_scene::toolpath::ToolpathOptions {
            step_pos,
            step_rot,
            ..botrail_scene::toolpath::ToolpathOptions::default()
        };
        let report = self
            .hub
            .check_paint(name, target, &limits, &options)
            .map_err(PyValueError::new_err)?;
        self.hub
            .set_toolpath_marks(name, paint_marks(&report))
            .map_err(PyValueError::new_err)?;
        Ok(PaintReport { inner: report })
    }

    /// Clears the marks a check left on toolpath `name`.
    fn clear_toolpath_marks(&self, name: &str) -> PyResult<()> {
        self.hub
            .set_toolpath_marks(name, Vec::new())
            .map_err(PyValueError::new_err)
    }

    /// Puts a film map in the picture: the coated target's own colour
    /// gives way to `film`'s heatmap mesh, registered as a display-only
    /// obstacle named `{target}_film` (disabled for collision, cheap
    /// collider) with its micron colour key attached, so the studio draws
    /// the legend beside the viewport. Collision and planning still see
    /// the original target; everything here is presentation. Returns the
    /// obstacle name; remove it and re-show the target to undo.
    #[pyo3(signature = (film, name = None))]
    fn show_film(&self, film: PyRef<'_, FilmCoat>, name: Option<&str>) -> PyResult<String> {
        let inner = &film.inner;
        let owned = format!("{}_film", inner.target);
        let name = name.unwrap_or(&owned);
        let dir = cache_base().join("film");
        let obj_path = write_cached_obj(&dir, &inner.mesh)?;
        // Panels take light like paint does.
        let material = botrail_scene::Material::new(0.1, 0.45);
        let legend = botrail_scene::Legend {
            title: film_legend_title(inner),
            stops: film_legend_stops(inner),
        };
        let name = self
            .hub
            .show_display_mesh(
                name,
                botrail_model::Geometry::Mesh {
                    path: obj_path,
                    scale: nalgebra::Vector3::new(1.0, 1.0, 1.0),
                },
                inner.pose,
                botrail_scene::ObstacleCollider::cuboid(mesh_half_extents(&inner.mesh)),
                material,
                Some(legend),
                Some(inner.target.as_str()),
            )
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(name)
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
        let material = botrail_scene::Material::new(0.75, 0.35);
        let mut entries: Vec<(
            String,
            botrail_model::Geometry,
            botrail_scene::ObstacleCollider,
        )> = Vec::with_capacity(stage_list.len());
        let mut times = Vec::with_capacity(stage_list.len());
        for (i, stage) in stage_list.iter().enumerate() {
            let obj_path = write_cached_obj(&dir, &stage.mesh)?;
            // A cheap stand-in collider: the stage registers disabled, so
            // VHACD on it would be pure cost.
            entries.push((
                format!("{stock}_cut/{i:03}"),
                botrail_model::Geometry::Mesh {
                    path: obj_path,
                    scale: nalgebra::Vector3::new(1.0, 1.0, 1.0),
                },
                botrail_scene::ObstacleCollider::cuboid(mesh_half_extents(&stage.mesh)),
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

    /// Progressive film build-up for a baked cycle: re-walks the coat in
    /// `stages` equal time slices (default: one slice per second of
    /// cycle, capped at 60 — the display lags the gun by at most one
    /// slice), registers one display-only obstacle per changed slice
    /// (grouped under `{target}_film/…`, cheap AABB colliders — they never
    /// collide) each carrying the film's colour key, and returns the
    /// timeline with the visibility windows injected: during playback —
    /// studio, USD export, and a replayed recording alike — the target's
    /// own colour gives way to the film building up on it. Optionally
    /// writes the effective spray trigger as signal `trigger_signal`
    /// (declare it first) for a timing lane and a spray-cone effect. The
    /// target keeps colliding unchanged; everything here is presentation.
    ///
    /// Stages walk at `patch_size` — coarser than a `spray_coat` for the
    /// numbers, since a mesh per stage is what a viewer has to carry —
    /// with the same trigger rules (`applicator`, `gate`, brushes) as
    /// `spray_coat`. Coloured by *amount* by default (a build-up is about
    /// how much paint is there: the ramp — in `paint_color`, if given —
    /// runs from a light wash to the full colour at the spec's high edge,
    /// or the final maximum without one), on the target's own colour;
    /// `style="spec"` colours every stage against the band instead.
    #[pyo3(signature = (timeline, target, applicator = None, stages = None, patch_size = 0.01, dt = 0.01, gate = None, spec = None, facing = None, facing_tolerance = std::f64::consts::FRAC_PI_3, occlusion = true, robot = None, tcp_link = None, trigger_signal = None, style = "amount", paint_color = None, substrate = None))]
    #[allow(clippy::too_many_arguments)]
    fn animate_paint(
        &self,
        py: Python<'_>,
        timeline: PyRef<'_, SequenceTimeline>,
        target: &str,
        applicator: Option<Bound<'_, PyAny>>,
        stages: Option<usize>,
        patch_size: f64,
        dt: f64,
        gate: Option<String>,
        spec: Option<(f64, f64)>,
        facing: Option<[f64; 3]>,
        facing_tolerance: f64,
        occlusion: bool,
        robot: Option<&str>,
        tcp_link: Option<&str>,
        trigger_signal: Option<&str>,
        style: &str,
        paint_color: Option<[f32; 3]>,
        substrate: Option<[f32; 3]>,
    ) -> PyResult<SequenceTimeline> {
        let stages =
            stages.unwrap_or_else(|| (timeline.inner.duration.ceil() as usize).clamp(1, 60));
        let (index, _) = timeline.track_for(robot)?;
        let model = &timeline.scene.robots()[index].model;
        let tcp = match tcp_link {
            Some(l) => resolve_link(&std::sync::Arc::clone(model), Some(l))?,
            None => model.default_tcp_link(),
        };
        let gun = applicator.map(|a| applicator_from_py(py, a)).transpose()?;
        let options = botrail_scene::coat::CoatOptions {
            patch_size,
            dt,
            gate: gate.clone(),
            spec,
            max_incidence: std::f64::consts::FRAC_PI_3,
            facing: facing.map(nalgebra::Vector3::from),
            facing_tolerance,
            occlusion,
            style: film_style(style)?,
            paint_color,
            substrate,
        };
        let (film, stage_list) = botrail_scene::coat::spray_coat_staged(
            &timeline.scene,
            &timeline.inner,
            target,
            index,
            tcp,
            gun.as_ref(),
            &options,
            stages,
        )
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
        if stage_list.is_empty() {
            return Err(PyValueError::new_err(format!(
                "the cycle never sprays `{target}` — nothing to animate"
            )));
        }

        // Stage meshes go to the cache as OBJ + MTL, content-addressed.
        // Every stage carries the same colour key; the studio collapses
        // identical keys into one card.
        let dir = cache_base().join("film");
        let material = botrail_scene::Material::new(0.1, 0.45);
        let legend = botrail_scene::Legend {
            title: film_legend_title(&film),
            stops: film_legend_stops(&film),
        };
        let mut entries: Vec<(
            String,
            botrail_model::Geometry,
            botrail_scene::ObstacleCollider,
        )> = Vec::with_capacity(stage_list.len());
        let mut times = Vec::with_capacity(stage_list.len());
        for (i, stage) in stage_list.iter().enumerate() {
            let obj_path = write_cached_obj(&dir, &stage.mesh)?;
            entries.push((
                format!("{target}_film/{i:03}"),
                botrail_model::Geometry::Mesh {
                    path: obj_path,
                    scale: nalgebra::Vector3::new(1.0, 1.0, 1.0),
                },
                botrail_scene::ObstacleCollider::cuboid(mesh_half_extents(&stage.mesh)),
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
                film.pose,
                collider.clone(),
            );
            let _ = snapshot.set_obstacle_enabled(&final_name, false);
            let _ = snapshot.set_obstacle_material(&final_name, Some(material));
            let _ = snapshot.set_obstacle_legend(&final_name, Some(legend.clone()));
        }
        let names = self
            .hub
            .add_display_stages(entries, film.pose, material, Some(legend));

        let mut augmented = botrail_scene::carve::staged_timeline(
            &timeline.inner,
            target,
            film.pose,
            &names,
            &times,
        );
        if let Some(name) = trigger_signal {
            let track = botrail_scene::coat::trigger_track(
                &timeline.scene,
                &timeline.inner,
                index,
                gate.as_deref(),
                name,
                dt,
            )
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
            augmented.signals.retain(|s| s.name != name);
            augmented.signals.push(track);
        }
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
    /// instances to start configurations, and `faults` pins inputs for
    /// the whole run — `bt.io.stuck("part_at_pick", False)` ignores the
    /// sensor's geometry (or a program's `set` on an internal signal),
    /// `bt.io.open("part_at_pick")` is a broken wire (input level low, so
    /// the value follows the binding's `invert`). `"baseline"` is the
    /// reserved name of the unmodified scene. Everything is validated when
    /// the scenario is *applied* (at simulate), so deltas may name things
    /// authored later.
    #[pyo3(signature = (name, signals = None, obstacles = None, joints = None, faults = None))]
    fn add_scenario(
        &self,
        name: &str,
        signals: Option<&Bound<'_, PyDict>>,
        obstacles: Option<&Bound<'_, PyDict>>,
        joints: Option<&Bound<'_, PyDict>>,
        faults: Option<Vec<Bound<'_, PyAny>>>,
    ) -> PyResult<()> {
        let mut scenario = botrail_scene::seq::Scenario {
            name: name.to_string(),
            signals: Vec::new(),
            obstacles: Vec::new(),
            joints: Vec::new(),
            faults: Vec::new(),
        };
        for fault in faults.unwrap_or_default() {
            let dict = fault.downcast::<PyDict>().map_err(|_| {
                PyValueError::new_err(
                    "faults are bt.io.stuck(name, value) / bt.io.open(name) entries",
                )
            })?;
            let target: String = dict
                .get_item("target")?
                .ok_or_else(|| PyValueError::new_err("a fault needs a `target`"))?
                .extract()?;
            let kind: String = dict
                .get_item("kind")?
                .ok_or_else(|| PyValueError::new_err("a fault needs a `kind`"))?
                .extract()?;
            let kind = match kind.as_str() {
                "stuck" => botrail_scene::seq::FaultKind::StuckAt(
                    dict.get_item("value")?
                        .ok_or_else(|| PyValueError::new_err("a stuck fault needs a `value`"))?
                        .extract::<bool>()?,
                ),
                "open" => botrail_scene::seq::FaultKind::Open,
                "node_down" => botrail_scene::seq::FaultKind::NodeDown,
                other => {
                    return Err(PyValueError::new_err(format!(
                        "unknown fault kind {other:?} (stuck, open, node_down)"
                    )))
                }
            };
            scenario
                .faults
                .push(botrail_scene::seq::Fault { target, kind });
        }
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
        let mut scene = botrail_scene::Scene::empty();
        for model in models {
            scene.add_robot(model, None, nalgebra::Isometry3::identity());
        }
        // apply_project restores instance names, bases, and joints.
        scene
            .apply_project(&project)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let robot = scene.robots().first().map(|sr| Robot {
            inner: sr.model.clone(),
        });
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

/// Writes `mesh` as OBJ + MTL into `dir` under a content-addressed name
/// (FNV-1a over the vertex/index bytes: stable across runs and Rust
/// versions, which is all a cache name needs), reusing an existing file.
/// OBJ + MTL because that is the one format the studio and the USD
/// export both read face colors back from. Returns the OBJ path.
fn write_cached_obj(dir: &Path, mesh: &botrail_mesh::MeshData) -> PyResult<PathBuf> {
    std::fs::create_dir_all(dir).map_err(|e| PyIOError::new_err(e.to_string()))?;
    let hash = {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        let mut eat = |bytes: &[u8]| {
            for b in bytes {
                h ^= *b as u64;
                h = h.wrapping_mul(0x1000_0000_01b3);
            }
        };
        for v in &mesh.vertices {
            for c in v {
                eat(&c.to_le_bytes());
            }
        }
        for t in &mesh.indices {
            for c in t {
                eat(&c.to_le_bytes());
            }
        }
        for c in &mesh.face_colors {
            for ch in c {
                eat(&ch.to_le_bytes());
            }
        }
        h
    };
    let obj_path = dir.join(format!("{hash:016x}.obj"));
    let mtl_name = format!("{hash:016x}.mtl");
    if !obj_path.exists() {
        let (obj, mtl) = botrail_mesh::to_obj_with_mtl(mesh, &mtl_name);
        std::fs::write(&obj_path, obj).map_err(|e| PyIOError::new_err(e.to_string()))?;
        std::fs::write(dir.join(&mtl_name), mtl).map_err(|e| PyIOError::new_err(e.to_string()))?;
    }
    Ok(obj_path)
}

/// Half extents of a mesh's local AABB — a stand-in collider for
/// display-only obstacles that never collide.
fn mesh_half_extents(mesh: &botrail_mesh::MeshData) -> nalgebra::Vector3<f64> {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for v in &mesh.vertices {
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
}

/// The legend's title: what the map is coloured by.
fn film_legend_title(film: &botrail_scene::coat::FilmCoat) -> String {
    match film.palette.style {
        botrail_scene::coat::FilmStyle::Spec => "film vs spec [um]".to_string(),
        _ => "film [um]".to_string(),
    }
}

/// The colour key of a film map, as `Legend` stops.
fn film_legend_stops(film: &botrail_scene::coat::FilmCoat) -> Vec<botrail_scene::LegendStop> {
    botrail_scene::coat::film_legend(film)
        .into_iter()
        .map(|(color, label)| botrail_scene::LegendStop { color, label })
        .collect()
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

    /// The steps a walking robot's legs took, as `(leg, lift, land,
    /// (x, y, z))` in landing order: the foot left its previous anchor at
    /// `lift` and has stood at the position since `land`. Empty unless the
    /// robot walks its vehicle (a `bt.Gait` on its mount).
    #[pyo3(signature = (robot = None))]
    fn footfalls(&self, robot: Option<&str>) -> PyResult<Vec<FootfallRow>> {
        Ok(self
            .track_for(robot)?
            .1
            .footfalls
            .iter()
            .map(|f| {
                (
                    f.leg.clone(),
                    f.lift,
                    f.land,
                    (f.position.x, f.position.y, f.position.z),
                )
            })
            .collect())
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

    /// Seconds the vehicle spent off its starting ground: every span that
    /// moves it, plus every hold above the altitude it started at — exact,
    /// the spans are closed form. For an aerial machine this is the
    /// motor-on time a declared `flight_time_min` must cover (hover at a
    /// station counts, waiting on the pad does not); for a ground machine
    /// it is simply its driving time. A vehicle that never drove flew 0 s.
    fn vehicle_airborne(&self, name: &str) -> PyResult<f64> {
        if let Some(seconds) = self.inner.vehicle_airborne(name) {
            return Ok(seconds);
        }
        let exists = self.scene.devices().iter().any(|d| {
            d.name == name && matches!(d.kind, botrail_scene::seq::DeviceKind::Vehicle { .. })
        });
        if exists {
            Ok(0.0)
        } else {
            Err(PyValueError::new_err(format!(
                "`{name}` is not a vehicle of this timeline's scene"
            )))
        }
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

    /// Sprays `applicator` along this cycle and reports the film left on
    /// `target`: a thickness map as a colored mesh plus the numbers a
    /// paint engineer reads — in-spec area, holidays, paint used.
    ///
    /// What sprays comes from the program: a toolpath whose strokes name
    /// brushes (`scene.define_brush`) sprays each with that brush's
    /// applicator, flow and trigger timing, and `applicator` may be left
    /// out; one that names none sprays every feed move with `applicator`
    /// (the dict `bt.paint.applicator(...)` builds), which is then
    /// required. The applicator's footprint is *calibrated geometry, not
    /// fluid dynamics*: no air
    /// flow, no electrostatics, so the electrostatic wrap around edges is
    /// not modeled and the absolute micrometers are only as good as the
    /// pattern fed in. Relative structure — lap streaks, thick corners,
    /// the film left by a stroke that lost speed — is the robust part,
    /// because the walk runs on the *baked* trajectory.
    ///
    /// Two triggers decide when paint flows, and both must agree: `gate`
    /// names the PLC's enable signal (without one it is taken as always
    /// on), and the program's own trigger is the *feed* strokes of the
    /// toolpath the robot was running — rapids and the approach planned
    /// in from wherever the robot stood never spray, however the enable
    /// was authored. A timeline that ran no toolpath has no program to
    /// say when the process was on, so there the enable alone decides.
    /// `spec` is the acceptable film band in meters. `style` picks how the
    /// film map is coloured: `"amount"` (a sequential ramp, light to dark
    /// — how much paint; in `paint_color` if given, so it looks like the
    /// coat going on) or `"spec"` (diverging over the band: neutral on
    /// target, blue thin, red thick — the verdict); `"auto"` is `"spec"`
    /// when a spec was given. Bare patches wear `substrate`, or the
    /// target's own colour.
    ///
    /// Statistics run over the surface the gun *addressed* — in range and
    /// within `max_incidence` of square on. A part's back face is not a
    /// holiday, and neither is the rim of a panel sprayed from above,
    /// which would otherwise swamp the film map with one grazing band.
    /// Deposition ignores the limit, so paint stays conserved.
    ///
    /// `facing` names the job by the way it faces — a world direction,
    /// with only patches whose normal lies within `facing_tolerance` of
    /// it counted (`(0, 0, 1)` for "the top"). Without it the addressed
    /// set depends on the path: a rim swings into the mask as the gun
    /// turns around past the edge, so lengthening the overtravel quietly
    /// changes every denominator. Name the face for numbers that compare
    /// across programs.
    #[pyo3(signature = (target, applicator = None, robot = None, tcp_link = None, patch_size = 0.005, dt = 0.01, gate = None, spec = None, max_incidence = std::f64::consts::FRAC_PI_3, facing = None, facing_tolerance = std::f64::consts::FRAC_PI_3, occlusion = true, style = "auto", paint_color = None, substrate = None))]
    #[allow(clippy::too_many_arguments)]
    fn spray_coat(
        &self,
        py: Python<'_>,
        target: &str,
        applicator: Option<Bound<'_, PyAny>>,
        robot: Option<&str>,
        tcp_link: Option<&str>,
        patch_size: f64,
        dt: f64,
        gate: Option<String>,
        spec: Option<(f64, f64)>,
        max_incidence: f64,
        facing: Option<[f64; 3]>,
        facing_tolerance: f64,
        occlusion: bool,
        style: &str,
        paint_color: Option<[f32; 3]>,
        substrate: Option<[f32; 3]>,
    ) -> PyResult<FilmCoat> {
        let gun = applicator.map(|a| applicator_from_py(py, a)).transpose()?;
        if let Some((lo, hi)) = spec {
            if !(lo.is_finite() && hi.is_finite() && lo < hi) {
                return Err(PyValueError::new_err(format!(
                    "spec must be (low, high) with low < high, got ({lo}, {hi})"
                )));
            }
        }
        let (index, _) = self.track_for(robot)?;
        let model = &self.scene.robots()[index].model;
        let tcp = match tcp_link {
            Some(l) => resolve_link(&std::sync::Arc::clone(model), Some(l))?,
            None => model.default_tcp_link(),
        };
        let options = botrail_scene::coat::CoatOptions {
            patch_size,
            dt,
            gate,
            spec,
            max_incidence,
            facing: facing.map(nalgebra::Vector3::from),
            facing_tolerance,
            occlusion,
            style: film_style(style)?,
            paint_color,
            substrate,
        };
        let inner = botrail_scene::coat::spray_coat(
            &self.scene,
            &self.inner,
            target,
            index,
            tcp,
            gun.as_ref(),
            &options,
        )
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(FilmCoat { inner })
    }

    /// Checks what the robot actually did against the teaching rules of a
    /// spray program: every `dt` while the gun was spraying — `gate` high
    /// (or no gate) *and* inside a feed stroke, the same two triggers
    /// `spray_coat` uses — the TCP's spray axis is cast at `target` and
    /// the standoff and incidence read off the hit. `standoff` is the
    /// acceptable band in meters, `max_incidence` the steepest acceptable
    /// angle in radians. `at` on the issues is timeline seconds.
    ///
    /// The baked twin of `Scene.check_paint`: that one checks the
    /// authored path before any robot is involved; this one includes
    /// whatever the solver did with the free spin and the tolerance it
    /// was given.
    #[pyo3(signature = (target, robot = None, tcp_link = None, gate = None, standoff = None, max_incidence = std::f64::consts::FRAC_PI_4, max_range = None, dt = 0.01))]
    #[allow(clippy::too_many_arguments)]
    fn paint_report(
        &self,
        target: &str,
        robot: Option<&str>,
        tcp_link: Option<&str>,
        gate: Option<&str>,
        standoff: Option<(f64, f64)>,
        max_incidence: f64,
        max_range: Option<f64>,
        dt: f64,
    ) -> PyResult<PaintReport> {
        let (index, _) = self.track_for(robot)?;
        let model = &self.scene.robots()[index].model;
        let tcp = match tcp_link {
            Some(l) => resolve_link(&std::sync::Arc::clone(model), Some(l))?,
            None => model.default_tcp_link(),
        };
        let limits = paint_limits(standoff, max_incidence, max_range)?;
        let inner = botrail_scene::coat::timeline_paint_report(
            &self.scene,
            &self.inner,
            target,
            index,
            tcp,
            gate,
            dt,
            &limits,
        )
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(PaintReport { inner })
    }

    /// The timeline with the effective spray trigger — the enable signal
    /// AND the program's own (feed strokes, brush lead/lag included) —
    /// written as signal `name`, replacing any lane of that name. What a
    /// timing chart shows as "spraying", and what a spray-cone effect
    /// (`scene.add_spray_cone`) should bind to; declare `name` with
    /// `scene.define_signal` first so the effect can be bound. Nothing
    /// else about the timeline changes.
    #[pyo3(signature = (name = "spraying", gate = None, robot = None, dt = 0.01))]
    fn with_trigger_signal(
        &self,
        name: &str,
        gate: Option<&str>,
        robot: Option<&str>,
        dt: f64,
    ) -> PyResult<SequenceTimeline> {
        let (index, _) = self.track_for(robot)?;
        let track =
            botrail_scene::coat::trigger_track(&self.scene, &self.inner, index, gate, name, dt)
                .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let mut inner = self.inner.clone();
        inner.signals.retain(|s| s.name != name);
        inner.signals.push(track);
        Ok(SequenceTimeline {
            inner,
            scene: self.scene.clone(),
        })
    }

    /// The `(start, end, brush)` intervals a robot's toolpath moves spent
    /// spraying — the program's own process trigger, as opposed to
    /// rapids, gun-off moves, and the approach planned in from wherever
    /// the robot stood; `brush` is `None` in a program that names none.
    /// Merged, in time order. Empty when the robot ran no toolpath: then
    /// there is no program to say when the process was on, and
    /// `spray_coat` / `paint_report` take the whole timeline as process
    /// time.
    #[pyo3(signature = (robot = None))]
    fn process_spans(&self, robot: Option<&str>) -> PyResult<Vec<(f64, f64, Option<String>)>> {
        let (index, _) = self.track_for(robot)?;
        Ok(self
            .inner
            .process_spans(index)
            .unwrap_or_default()
            .into_iter()
            .map(|s| (s.start, s.end, s.brush))
            .collect())
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

    /// Compares this bake with a controller trace (`bt.trace.load` — a
    /// CSV path / text, a `{name: [(t, value), ...]}` dict, or a `Trace`)
    /// edge by edge, by name: matched edges within `tolerance` seconds,
    /// missing ones (baked, never seen), extra ones (seen, never baked).
    /// `align_on=` names a signal whose first rising edge sets the trace's
    /// clock against the bake's; `signals=` picks the names to judge;
    /// `io=` renames binding tags to point names. Returns a
    /// `bt.trace.TraceDiff` (`ok`, `signals`, `findings()`, `to_markdown()`,
    /// `to_json()`) — the offline commissioning check.
    #[pyo3(signature = (trace, *, tolerance = 0.05, signals = None, align_on = None, io = None))]
    fn diff(
        slf: Py<Self>,
        py: Python<'_>,
        trace: Py<PyAny>,
        tolerance: f64,
        signals: Option<Vec<String>>,
        align_on: Option<String>,
        io: Option<Py<PyAny>>,
    ) -> PyResult<Py<PyAny>> {
        let module = py.import("botrail.trace")?;
        let kwargs = PyDict::new(py);
        kwargs.set_item("tolerance", tolerance)?;
        kwargs.set_item("signals", signals)?;
        kwargs.set_item("align_on", align_on)?;
        kwargs.set_item("io", io)?;
        Ok(module
            .getattr("diff")?
            .call((slf, trace), Some(&kwargs))?
            .unbind())
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
        tcp_speed = 0.25, tcp_accel = 1.2, move_to_start = true, node = None, io = None))]
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
        node: Option<&str>,
        io: Option<PyRef<'_, IoMap>>,
    ) -> PyResult<String> {
        let backend = botrail_export::backend(dialect).ok_or_else(|| {
            PyValueError::new_err(format!(
                "unknown dialect {dialect:?} (available: {})",
                botrail_export::DIALECTS.join(", ")
            ))
        })?;
        let io = project_io(
            &self.scene,
            &self.inner,
            sequence,
            node,
            io.as_deref(),
            inputs,
            outputs,
        )?;
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

    /// Intervals a robot was driven by a motion, ramp or toolpath,
    /// merged where they touch — the "busy" contact a robot controller
    /// would show a PLC, synthesized from the bake (a robot has no signal
    /// lane). Robot defaults to the scene's first.
    #[pyo3(signature = (robot = None))]
    fn robot_busy(&self, robot: Option<&str>) -> PyResult<Vec<(f64, f64)>> {
        let (_, track) = self.track_for(robot)?;
        Ok(botrail_scene::handshake::robot_busy(&self.inner, &track.name).unwrap_or_default())
    }

    /// The handshake specification of this bake as Markdown: every line
    /// between controllers — handshake signals, robot start / done /
    /// program handshakes, device command and in-position lines — with
    /// direction, both ends (node and channel when bound), the steps that
    /// write and wait on it, and its waveform (high spans, or the robot's
    /// start pulses and busy spans). The draft of the robot ⇔ PLC
    /// interface sheet, per scenario. `io=` projects a newer assignment
    /// onto a bake made before the wiring.
    #[pyo3(signature = (io = None))]
    fn handshake_spec(&self, io: Option<PyRef<'_, IoMap>>) -> PyResult<String> {
        let owned;
        let scene = match io {
            Some(io) => {
                let mut s = self.scene.clone();
                s.set_io_map(io.inner.clone()).map_err(scene_err)?;
                owned = s;
                &owned
            }
            None => &self.scene,
        };
        botrail_scene::handshake::render_handshake_spec(scene, &self.inner)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    /// Writes `handshake_spec()` to `path` (Markdown).
    #[pyo3(signature = (path, io = None))]
    fn export_handshake_spec(&self, path: PathBuf, io: Option<PyRef<'_, IoMap>>) -> PyResult<()> {
        let text = self.handshake_spec(io)?;
        std::fs::write(&path, text)
            .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))
    }

    /// Writes `to_script` output to `path` (see there for the semantics).
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (path, sequence = None, dialect = "urscript", name = None,
        inputs = None, outputs = None, speed_scale = 1.0, blend_radius = 0.0,
        tcp_speed = 0.25, tcp_accel = 1.2, move_to_start = true, node = None, io = None))]
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
        node: Option<&str>,
        io: Option<PyRef<'_, IoMap>>,
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
            node,
            io,
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
        tcp_speed = 0.25, tcp_accel = 1.2, move_to_start = true, node = None, io = None))]
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
        node: Option<&str>,
        io: Option<PyRef<'_, IoMap>>,
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
        let borrowed: Vec<PyRef<'_, SequenceTimeline>> =
            self.runs.iter().map(|(_, t)| t.borrow(py)).collect();
        let io = project_io(
            &self.scene,
            &borrowed[lead].inner,
            sequence,
            node,
            io.as_deref(),
            inputs,
            outputs,
        )?;
        let options = botrail_export::ProgramOptions {
            speed_scale,
            blend_radius,
            tcp_speed,
            tcp_accel,
            move_to_start,
        };
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
        blend_radius = 0.0, tcp_speed = 0.25, tcp_accel = 1.2, move_to_start = true,
        node = None, io = None))]
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
        node: Option<&str>,
        io: Option<PyRef<'_, IoMap>>,
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
            node,
            io,
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

    /// Where the lane comes from: `"signal"` (internal relay, or a lane
    /// synthesized under a signal's name), `"sensor"` (input) or
    /// `"device"` (running / moving output).
    #[getter]
    fn kind(&self) -> String {
        self.inner.kind.as_str().to_string()
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
/// A channel dict — flat (`{"id", "kind", "port", "address", "voltage",
/// "logic"}`, what `bt.io` templates build) or the serialized form with a
/// nested `electrical`.
fn channel_from_py(obj: &Bound<'_, PyAny>) -> PyResult<botrail_scene::iomap::IoChannel> {
    use botrail_scene::iomap::{ChannelKind, Electrical, IoChannel, Logic};
    let dict = obj.downcast::<PyDict>().map_err(|_| {
        PyValueError::new_err("channels= is a list of dicts (see bt.io.di8 / bt.io.ur_standard)")
    })?;
    let get = |key: &str| -> PyResult<Option<Bound<'_, PyAny>>> { dict.get_item(key) };
    let id: String = get("id")?
        .ok_or_else(|| PyValueError::new_err("channel dict needs an \"id\""))?
        .extract()?;
    let kind_s: String = get("kind")?
        .ok_or_else(|| PyValueError::new_err(format!("channel {id:?} needs a \"kind\"")))?
        .extract()?;
    let kind = ChannelKind::parse(&kind_s).ok_or_else(|| {
        PyValueError::new_err(format!(
            "channel {id:?}: unknown kind {kind_s:?} (di, do, ai, ao, safe_di, safe_do, word)"
        ))
    })?;
    let port: Option<u32> = match get("port")? {
        Some(v) if !v.is_none() => Some(v.extract()?),
        _ => None,
    };
    let address: Option<String> = match get("address")? {
        Some(v) if !v.is_none() => Some(v.extract()?),
        _ => None,
    };
    let (mut voltage, mut logic): (Option<f64>, Option<Logic>) = (None, None);
    if let Some(e) = get("electrical")? {
        if let Ok(e) = e.downcast::<PyDict>() {
            if let Some(v) = e.get_item("voltage")? {
                if !v.is_none() {
                    voltage = Some(v.extract()?);
                }
            }
            if let Some(l) = e.get_item("logic")? {
                if !l.is_none() {
                    let l: String = l.extract()?;
                    logic = Logic::parse(&l);
                }
            }
        }
    }
    if let Some(v) = get("voltage")? {
        if !v.is_none() {
            voltage = Some(v.extract()?);
        }
    }
    if let Some(l) = get("logic")? {
        if !l.is_none() {
            let l: String = l.extract()?;
            logic = Some(Logic::parse(&l).ok_or_else(|| {
                PyValueError::new_err(format!("channel {id:?}: unknown logic {l:?} (pnp, npn)"))
            })?);
        }
    }
    let electrical = if voltage.is_some() || logic.is_some() {
        Some(Electrical { voltage, logic })
    } else {
        None
    };
    Ok(IoChannel {
        id,
        kind,
        port,
        address,
        electrical,
    })
}

/// The assignment layer of a scene's I/O map (see `Scene.io_map`).
#[pyclass(frozen, module = "botrail._core")]
#[derive(Clone)]
struct IoMap {
    inner: botrail_scene::iomap::IoMap,
}

#[pymethods]
impl IoMap {
    /// Node names, in declaration order.
    #[getter]
    fn nodes(&self) -> Vec<String> {
        self.inner.nodes.iter().map(|n| n.name.clone()).collect()
    }

    /// `(label, direction, node, channel)` per binding.
    #[getter]
    fn bindings(&self) -> Vec<(String, String, String, String)> {
        self.inner
            .bindings
            .iter()
            .map(|b| {
                (
                    b.point.label(),
                    b.point.direction.as_str().to_string(),
                    b.node.clone(),
                    b.channel.clone(),
                )
            })
            .collect()
    }

    /// Declared names.
    #[getter]
    fn decls(&self) -> Vec<String> {
        self.inner.decls.iter().map(|d| d.name.clone()).collect()
    }

    /// The layer as JSON — the same form the `.botrail` project stores.
    fn to_json(&self) -> String {
        serde_json::to_string_pretty(&self.inner).unwrap_or_default()
    }

    fn __repr__(&self) -> String {
        format!(
            "IoMap({} nodes, {} bindings, {} decls)",
            self.inner.nodes.len(),
            self.inner.bindings.len(),
            self.inner.decls.len()
        )
    }
}

/// The wiring a script export uses: explicit `inputs=` / `outputs=` dicts
/// win, the rest is projected from the bindings on the robot-controller
/// node (`node`, or the declared node driving the sequence's robot) —
/// against `io` when a newer assignment layer is handed over, else the
/// timeline's snapshot.
#[allow(clippy::too_many_arguments)]
fn project_io(
    scene: &botrail_scene::Scene,
    timeline: &botrail_scene::rollout::SequenceTimeline,
    sequence: Option<&str>,
    node: Option<&str>,
    io: Option<&IoMap>,
    inputs: Option<std::collections::HashMap<String, u32>>,
    outputs: Option<std::collections::HashMap<String, u32>>,
) -> PyResult<botrail_scene::script::SequenceIo> {
    let mut wired = botrail_scene::script::SequenceIo::from_ports(
        inputs.unwrap_or_default(),
        outputs.unwrap_or_default(),
    );
    let owned;
    let scene = match io {
        Some(io) => {
            let mut s = scene.clone();
            s.set_io_map(io.inner.clone()).map_err(scene_err)?;
            owned = s;
            &owned
        }
        None => scene,
    };
    let node = match node {
        Some(n) => {
            if scene.io_map().node(n).is_none() {
                return Err(PyValueError::new_err(format!("unknown I/O node `{n}`")));
            }
            Some(n.to_string())
        }
        None => {
            let robot = botrail_scene::script::driven_robot_name(scene, timeline, sequence)
                .map_err(PyValueError::new_err)?;
            scene
                .io_map()
                .robot_controller(&robot)
                .map(|n| n.name.clone())
        }
    };
    if let Some(node) = node {
        let names: Vec<&str> = timeline.sequences.iter().map(String::as_str).collect();
        let d = botrail_scene::iomap::derive(scene, Some(&names))
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        let (inputs, outputs) = botrail_scene::iomap::sequence_io(&d, scene.io_map(), &node)
            .map_err(PyValueError::new_err)?;
        for (k, v) in inputs {
            wired.inputs.entry(k).or_insert(v);
        }
        for (k, v) in outputs {
            wired.outputs.entry(k).or_insert(v);
        }
    }
    Ok(wired)
}

fn parse_layers(layers: Option<Vec<String>>) -> PyResult<Vec<botrail_scene::iomap::TopoLayer>> {
    layers
        .unwrap_or_default()
        .iter()
        .map(|l| {
            botrail_scene::iomap::TopoLayer::parse(l).ok_or_else(|| {
                PyValueError::new_err(format!(
                    "unknown topology layer {l:?} (functional, io, network, wiring, safety)"
                ))
            })
        })
        .collect()
}

fn render_topology(
    t: &botrail_scene::iomap::Topology,
    layers: &[botrail_scene::iomap::TopoLayer],
    format: &str,
) -> PyResult<String> {
    use botrail_scene::iomap::{render_dot, render_mermaid, render_topology_json};
    match format {
        "mermaid" | "mmd" => Ok(render_mermaid(t, layers)),
        "dot" | "graphviz" => Ok(render_dot(t, layers)),
        "json" => Ok(render_topology_json(t, layers)),
        other => Err(PyValueError::new_err(format!(
            "unknown topology format {other:?} (mermaid, dot, json)"
        ))),
    }
}

fn render_io(d: &botrail_scene::iomap::IoDerivation, format: &str) -> PyResult<String> {
    use botrail_scene::iomap::{render_csv, render_json, render_markdown};
    match format {
        "csv" => Ok(render_csv(d)),
        "md" | "markdown" => Ok(render_markdown(d)),
        "json" => Ok(render_json(d)),
        other => Err(PyValueError::new_err(format!(
            "unknown I/O list format {other:?} (csv, md, json)"
        ))),
    }
}

fn step_ref_tuple(s: &botrail_scene::iomap::StepRef) -> (String, usize, String) {
    (s.sequence.clone(), s.index, s.name.clone())
}

/// One derived I/O point of the cell (see `Scene.io_points`).
#[pyclass(frozen, module = "botrail._core")]
#[derive(Clone)]
struct IoPoint {
    inner: botrail_scene::iomap::IoPoint,
}

#[pymethods]
impl IoPoint {
    /// The scene name the point belongs to (signal, sensor, device, robot).
    #[getter]
    fn name(&self) -> String {
        self.inner.id.name.clone()
    }

    /// The facet for device commands and robot handshakes (`"index"`,
    /// `"dispatch"`, `"station"`, `"position"`, `"speed"`, `"start"`,
    /// `"done"`, `"program"`), or None.
    #[getter]
    fn aspect(&self) -> Option<String> {
        self.inner.id.aspect.map(|a| a.as_str().to_string())
    }

    /// `name` or `name.aspect` — the label the tables use.
    #[getter]
    fn label(&self) -> String {
        self.inner.label()
    }

    /// `"input"` or `"output"`, from the host's side.
    #[getter]
    fn direction(&self) -> String {
        self.inner.id.direction.as_str().to_string()
    }

    /// Channel type: `"DI"`, `"DO"`, `"Word"`, `"AO"`, ...
    #[getter]
    fn kind(&self) -> String {
        self.inner.kind.as_str().to_string()
    }

    /// The derivation rule that produced it: `"sensor"`,
    /// `"signal:handshake"`, `"signal:internal"`, `"signal:write-only"`,
    /// `"signal:read-only"`, `"device:run"`, `"device:done"`,
    /// `"device:command"`, `"device:cosmetic"`, `"robot:start"`,
    /// `"robot:done"`, `"robot:program"`.
    #[getter]
    fn source(&self) -> String {
        self.inner.source.as_str().to_string()
    }

    /// The controller that owns the point: `"<cell>"`, `"<robot name>"`
    /// (implicit placement) or a declared node; None when nothing pins it.
    #[getter]
    fn host(&self) -> Option<String> {
        self.inner.host.clone()
    }

    #[getter]
    fn safety(&self) -> bool {
        self.inner.safety
    }

    /// `(sequence, flat step index, step name)` of the steps that write
    /// the point (coil writes, device commands, robot starts).
    #[getter]
    fn writers(&self) -> Vec<(String, usize, String)> {
        self.inner.writers.iter().map(step_ref_tuple).collect()
    }

    /// `(sequence, flat step index, step name)` of the steps that read it.
    #[getter]
    fn readers(&self) -> Vec<(String, usize, String)> {
        self.inner.readers.iter().map(step_ref_tuple).collect()
    }

    /// `"unbound"`, `"internal"` (a relay, no I/O), `"cosmetic"`
    /// (magazine), `"constant"` (a coil that is on from t = 0 and never
    /// commanded).
    #[getter]
    fn status(&self) -> String {
        self.inner.status.as_str().to_string()
    }

    fn __repr__(&self) -> String {
        format!(
            "IoPoint({} {} {} {} host={} {})",
            self.inner.label(),
            self.inner.id.direction.as_str(),
            self.inner.kind.as_str(),
            self.inner.source.as_str(),
            self.inner.host.as_deref().unwrap_or("-"),
            self.inner.status.as_str(),
        )
    }
}

/// One lint finding of the I/O map.
/// The cell report `Scene.cell_report()` gathers: robots, cycles, I/O,
/// scenarios, BOM totals, footprint, deliverable digests. Every section
/// is a plain dict / list (JSON-shaped); `to_markdown()` renders the same
/// data for people.
#[pyclass(frozen, module = "botrail._core")]
#[derive(Clone)]
struct CellReport {
    inner: botrail_scene::report::CellReport,
}

impl CellReport {
    fn section(&self, py: Python<'_>, key: &str) -> PyResult<PyObject> {
        let value =
            serde_json::to_value(&self.inner).map_err(|e| PyValueError::new_err(e.to_string()))?;
        json_to_py(py, value.get(key).unwrap_or(&serde_json::Value::Null))
    }
}

#[pymethods]
impl CellReport {
    #[getter]
    fn title(&self) -> String {
        self.inner.title.clone()
    }

    /// The robots: `name`, `dof`, `base`, and `catalog` / `manufacturer` /
    /// `model` / `reach` when the catalog knows them.
    #[getter]
    fn robots(&self, py: Python<'_>) -> PyResult<PyObject> {
        self.section(py, "robots")
    }

    /// The cycles passed in: `name`, `sequences`, `scenario`, `duration`,
    /// `steps` (`name`, `sequence`, `start`, `end`), `robots` (`robot`,
    /// `busy`, `utilization`), `clearance` (`distance`, `t`, `pair`) and
    /// `branches`.
    #[getter]
    fn cycles(&self, py: Python<'_>) -> PyResult<PyObject> {
        self.section(py, "cycles")
    }

    /// The I/O summary — `points`, `by_kind`, `bound`, `unbound`,
    /// `internal`, `safety`, `nodes`, `findings` — or `None` when the map
    /// could not be derived (see `io_error`).
    #[getter]
    fn io(&self, py: Python<'_>) -> PyResult<PyObject> {
        self.section(py, "io")
    }

    #[getter]
    fn io_error(&self) -> Option<String> {
        self.inner.io_error.clone()
    }

    /// The scenario matrix: `name`, `ok`, `duration`, `error`.
    #[getter]
    fn scenarios(&self, py: Python<'_>) -> PyResult<PyObject> {
        self.section(py, "scenarios")
    }

    /// BOM totals: `rows`, `unidentified`, `by_category`, `totals`.
    #[getter]
    fn bom(&self, py: Python<'_>) -> PyResult<PyObject> {
        self.section(py, "bom")
    }

    /// The plan-view footprint: `min`, `max`, `width`, `depth`, `area`,
    /// `height`.
    #[getter]
    fn footprint(&self, py: Python<'_>) -> PyResult<PyObject> {
        self.section(py, "footprint")
    }

    /// The hashed deliverables: `path`, `sha256`, `bytes`.
    #[getter]
    fn deliverables(&self, py: Python<'_>) -> PyResult<PyObject> {
        self.section(py, "deliverables")
    }

    /// The cycle time of `name` (or of the first cycle), or `None`.
    #[pyo3(signature = (name = None))]
    fn cycle_time(&self, name: Option<&str>) -> Option<f64> {
        self.inner.cycle_time(name)
    }

    /// The tightest clearance over every cycle that measured one, or
    /// `None`.
    fn min_clearance(&self) -> Option<f64> {
        self.inner.min_clearance()
    }

    fn to_markdown(&self) -> String {
        self.inner.to_markdown()
    }

    fn to_json(&self) -> String {
        self.inner.to_json()
    }

    /// Writes the report to `path`; the format follows the extension
    /// (`.md`, `.json`) unless `format` says otherwise.
    #[pyo3(signature = (path, format = None))]
    fn save(&self, path: PathBuf, format: Option<&str>) -> PyResult<()> {
        let format = match format {
            Some(f) => f.to_string(),
            None => match path.extension().and_then(|e| e.to_str()) {
                Some("md") | Some("markdown") => "md".to_string(),
                Some("json") => "json".to_string(),
                other => {
                    return Err(PyValueError::new_err(format!(
                        "CellReport: unknown extension {:?} — use .md or .json (or pass format=)",
                        other.unwrap_or("")
                    )))
                }
            },
        };
        let text = match format.as_str() {
            "md" | "markdown" => self.inner.to_markdown(),
            "json" => self.inner.to_json(),
            other => {
                return Err(PyValueError::new_err(format!(
                    "CellReport: unknown format {other:?} — use \"md\" or \"json\""
                )))
            }
        };
        std::fs::write(&path, text)
            .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))
    }

    fn __repr__(&self) -> String {
        format!(
            "CellReport({:?}, {} cycle(s), footprint {:.2}×{:.2} m, BOM {} lines)",
            self.inner.title,
            self.inner.cycles.len(),
            self.inner.footprint.width,
            self.inner.footprint.depth,
            self.inner.bom.rows
        )
    }
}

/// The bill of materials `Scene.bom()` derives — one row per distinct
/// product, in scene order.
#[pyclass(frozen, module = "botrail._core")]
#[derive(Clone)]
struct Bom {
    inner: botrail_scene::part::Bom,
}

#[pymethods]
impl Bom {
    /// The rows as dicts: `category`, `names` (the residents the row
    /// stands for), `manufacturer`, `model`, `catalog` (`id@revision`),
    /// `qty`, `description`, `attributes`.
    #[getter]
    fn rows(&self, py: Python<'_>) -> PyResult<Vec<PyObject>> {
        self.inner
            .rows
            .iter()
            .map(|row| bom_row_dict(py, row))
            .collect()
    }

    /// Rows nothing identifies yet (no catalog reference, maker or
    /// model) — the purchasing to-do list.
    fn unidentified(&self, py: Python<'_>) -> PyResult<Vec<PyObject>> {
        self.inner
            .unidentified()
            .into_iter()
            .map(|row| bom_row_dict(py, row))
            .collect()
    }

    /// Σ qty × `key` over the rows carrying it as a number, or `None`
    /// when no row does (a missing figure must not read as zero).
    fn total(&self, key: &str) -> Option<f64> {
        self.inner.total(key)
    }

    /// Every attribute column any row carries, sorted.
    fn attribute_keys(&self) -> Vec<String> {
        self.inner.attribute_keys()
    }

    fn to_csv(&self) -> String {
        self.inner.to_csv()
    }

    fn to_markdown(&self) -> String {
        self.inner.to_markdown()
    }

    /// `{"rows": [...], "totals": {...}}`.
    fn to_json(&self) -> String {
        self.inner.to_json()
    }

    /// Writes the table to `path`; the format follows the extension
    /// (`.csv`, `.md`, `.json`) unless `format` says otherwise.
    #[pyo3(signature = (path, format = None))]
    fn save(&self, path: PathBuf, format: Option<&str>) -> PyResult<()> {
        let format = match format {
            Some(f) => f.to_string(),
            None => match path.extension().and_then(|e| e.to_str()) {
                Some("csv") => "csv".to_string(),
                Some("md") | Some("markdown") => "md".to_string(),
                Some("json") => "json".to_string(),
                other => {
                    return Err(PyValueError::new_err(format!(
                        "BOM: unknown extension {:?} — use .csv, .md or .json (or pass format=)",
                        other.unwrap_or("")
                    )))
                }
            },
        };
        let text = match format.as_str() {
            "csv" => self.inner.to_csv(),
            "md" | "markdown" => self.inner.to_markdown(),
            "json" => self.inner.to_json(),
            other => {
                return Err(PyValueError::new_err(format!(
                    "BOM: unknown format {other:?} — use \"csv\", \"md\" or \"json\""
                )))
            }
        };
        std::fs::write(&path, text)
            .map_err(|e| PyIOError::new_err(format!("{}: {e}", path.display())))
    }

    fn __len__(&self) -> usize {
        self.inner.rows.len()
    }

    fn __repr__(&self) -> String {
        let unidentified = self.inner.unidentified().len();
        if unidentified == 0 {
            format!("Bom({} rows)", self.inner.rows.len())
        } else {
            format!(
                "Bom({} rows, {unidentified} unidentified)",
                self.inner.rows.len()
            )
        }
    }
}

#[pyclass(frozen, module = "botrail._core")]
#[derive(Clone)]
struct IoFinding {
    inner: botrail_scene::iomap::IoFinding,
}

#[pymethods]
impl IoFinding {
    /// `"error"`, `"warning"` or `"info"`.
    #[getter]
    fn severity(&self) -> String {
        self.inner.severity.as_str().to_string()
    }

    /// The finding code (`"name_clash"`, `"unreferenced"`,
    /// `"word_unexpressible"`, `"implicit_host"`, ...).
    #[getter]
    fn code(&self) -> String {
        self.inner.code.as_str().to_string()
    }

    #[getter]
    fn message(&self) -> String {
        self.inner.message.clone()
    }

    /// The steps the finding is attributed to, as `(sequence, flat step
    /// index, step name)`.
    #[getter]
    fn at(&self) -> Vec<(String, usize, String)> {
        self.inner.at.iter().map(step_ref_tuple).collect()
    }

    fn __str__(&self) -> String {
        self.inner.to_string()
    }

    fn __repr__(&self) -> String {
        format!("IoFinding({})", self.inner)
    }
}

/// The findings of `Scene.io_report()`.
#[pyclass(frozen, module = "botrail._core")]
#[derive(Clone)]
struct IoReport {
    inner: botrail_scene::iomap::IoReport,
}

#[pymethods]
impl IoReport {
    /// Every finding, most severe first.
    #[getter]
    fn findings(&self) -> Vec<IoFinding> {
        let mut all: Vec<IoFinding> = self
            .inner
            .findings
            .iter()
            .map(|f| IoFinding { inner: f.clone() })
            .collect();
        all.sort_by_key(|f| f.inner.severity);
        all
    }

    fn errors(&self) -> Vec<IoFinding> {
        self.inner
            .errors()
            .into_iter()
            .map(|f| IoFinding { inner: f.clone() })
            .collect()
    }

    fn warnings(&self) -> Vec<IoFinding> {
        self.inner
            .warnings()
            .into_iter()
            .map(|f| IoFinding { inner: f.clone() })
            .collect()
    }

    /// The findings as JSON: `{"ok": bool, "findings": [{"severity", "code",
    /// "message", "at": [[sequence, step index, step name], ...]}]}`.
    fn to_json(&self) -> String {
        let findings: Vec<serde_json::Value> = self
            .inner
            .findings
            .iter()
            .map(|f| {
                serde_json::json!({
                    "severity": f.severity.as_str(),
                    "code": f.code.as_str(),
                    "message": f.message,
                    "at": f.at.iter().map(step_ref_tuple).collect::<Vec<_>>(),
                })
            })
            .collect();
        serde_json::to_string_pretty(&serde_json::json!({
            "ok": self.inner.errors().is_empty(),
            "findings": findings,
        }))
        .expect("report serializes")
    }

    fn infos(&self) -> Vec<IoFinding> {
        self.inner
            .infos()
            .into_iter()
            .map(|f| IoFinding { inner: f.clone() })
            .collect()
    }

    /// True when there are no errors.
    #[getter]
    fn ok(&self) -> bool {
        self.inner.errors().is_empty()
    }

    fn __len__(&self) -> usize {
        self.inner.findings.len()
    }

    fn __str__(&self) -> String {
        self.inner.to_string()
    }

    fn __repr__(&self) -> String {
        format!(
            "IoReport({} errors, {} warnings, {} infos)",
            self.inner.errors().len(),
            self.inner.warnings().len(),
            self.inner.infos().len()
        )
    }
}

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

/// A film-map colouring style by name.
fn film_style(name: &str) -> PyResult<botrail_scene::coat::FilmStyle> {
    use botrail_scene::coat::FilmStyle;
    match name {
        "auto" => Ok(FilmStyle::Auto),
        "amount" => Ok(FilmStyle::Amount),
        "spec" => Ok(FilmStyle::Spec),
        other => Err(PyValueError::new_err(format!(
            "style must be \"auto\", \"amount\" or \"spec\", got {other:?}"
        ))),
    }
}

/// An applicator as Python passes it: the dict `bt.paint.applicator`
/// builds, or its JSON.
fn applicator_from_py(
    py: Python<'_>,
    applicator: Bound<'_, PyAny>,
) -> PyResult<botrail_scene::coat::Applicator> {
    let json: String = if let Ok(s) = applicator.extract::<String>() {
        s
    } else {
        py.import("json")?
            .call_method1("dumps", (&applicator,))?
            .extract()?
    };
    serde_json::from_str(&json)
        .map_err(|e| PyValueError::new_err(format!("invalid applicator: {e}")))
}

/// The teaching rules of a spray program, validated. `max_range` defaults
/// to twice the far end of the standoff band (or a meter without one):
/// looking further than that finds scenery, not the part.
fn paint_limits(
    standoff: Option<(f64, f64)>,
    max_incidence: f64,
    max_range: Option<f64>,
) -> PyResult<botrail_scene::coat::PaintLimits> {
    if let Some((lo, hi)) = standoff {
        if !(lo.is_finite() && hi.is_finite() && 0.0 <= lo && lo < hi) {
            return Err(PyValueError::new_err(format!(
                "standoff must be (low, high) with 0 <= low < high, got ({lo}, {hi})"
            )));
        }
    }
    if !(max_incidence.is_finite() && max_incidence > 0.0) {
        return Err(PyValueError::new_err(format!(
            "max_incidence must be a positive angle in radians, got {max_incidence}"
        )));
    }
    let max_range = max_range.unwrap_or_else(|| standoff.map(|(_, hi)| hi * 2.0).unwrap_or(1.0));
    if !(max_range.is_finite() && max_range > 0.0) {
        return Err(PyValueError::new_err(format!(
            "max_range must be positive, got {max_range}"
        )));
    }
    Ok(botrail_scene::coat::PaintLimits {
        standoff,
        max_incidence,
        max_range,
    })
}

/// One legend swatch as Python passes it: `((r, g, b), label)`.
type LegendStopArg = ((f32, f32, f32), String);

/// Marks a face check leaves on its toolpath for the studio: one point per
/// flagged sample, tagged by kind.
fn reach_marks(report: &botrail_scene::toolpath::ToolpathReport) -> Vec<botrail_scene::PathMark> {
    use botrail_scene::toolpath::IssueKind;
    report
        .issues
        .iter()
        .map(|i| botrail_scene::PathMark {
            position: i.position,
            kind: match i.kind {
                IssueKind::Unreachable => "unreachable",
                IssueKind::ConfigJump => "config_jump",
                IssueKind::Collision => "collision",
            }
            .to_string(),
        })
        .collect()
}

fn paint_marks(report: &botrail_scene::coat::PaintReport) -> Vec<botrail_scene::PathMark> {
    report
        .issues
        .iter()
        .map(|i| botrail_scene::PathMark {
            position: i.position,
            kind: i.kind.as_str().to_string(),
        })
        .collect()
}

/// Face diagnosis of a spray program against a target: standoff and
/// incidence at every spraying sample, judged against the teaching
/// rules. Truthy iff clean (`ok`). `at` is meters along the path for
/// `Scene.check_paint`, timeline seconds for `SequenceTimeline.paint_report`.
#[pyclass(frozen, module = "botrail._core")]
struct PaintReport {
    inner: botrail_scene::coat::PaintReport,
}

#[pymethods]
impl PaintReport {
    /// True when the program met the target somewhere and, everywhere it
    /// did, kept the standoff band and the incidence limit. Off-target
    /// stretches (`no_target`) do not fail it — a raster's overtravel is
    /// supposed to run past the part; they are reported for the marks,
    /// `spans("no_target")`, and `on_target_ratio`.
    #[getter]
    fn ok(&self) -> bool {
        self.inner.ok()
    }

    /// Probes taken (spraying samples only).
    #[getter]
    fn total_samples(&self) -> usize {
        self.inner.probes.len()
    }

    /// Probes whose spray axis met the target.
    #[getter]
    fn hits(&self) -> usize {
        self.inner.hits
    }

    /// Of the probes that met the target, the fraction inside every rule
    /// — adherence to the teaching rules where they apply.
    #[getter]
    fn in_band_ratio(&self) -> f64 {
        self.inner.in_band_ratio
    }

    /// Fraction of all probes that met the target at all — how much of
    /// the spraying was pointed at the part. The rest is overspray, and
    /// where per-stroke triggering would earn its keep.
    #[getter]
    fn on_target_ratio(&self) -> f64 {
        self.inner.on_target_ratio
    }

    /// Standoff [m] over the hits (zero without any).
    #[getter]
    fn standoff_min(&self) -> f64 {
        self.inner.standoff_min
    }

    #[getter]
    fn standoff_max(&self) -> f64 {
        self.inner.standoff_max
    }

    #[getter]
    fn standoff_mean(&self) -> f64 {
        self.inner.standoff_mean
    }

    /// Steepest incidence [rad] seen over the hits.
    #[getter]
    fn incidence_max(&self) -> f64 {
        self.inner.incidence_max
    }

    /// One dict per flagged sample: `{sample, at, move, kind, position,
    /// value}` with `kind` in `"no_target" | "too_far" | "too_close" |
    /// "oblique"`, `position` the world gun-tip position, and `value` the
    /// offending standoff [m] or incidence [rad].
    #[getter]
    fn issues<'py>(&self, py: Python<'py>) -> PyResult<Vec<Bound<'py, pyo3::types::PyDict>>> {
        self.inner
            .issues
            .iter()
            .map(|i| {
                let d = pyo3::types::PyDict::new(py);
                d.set_item("sample", i.sample)?;
                d.set_item("at", i.at)?;
                d.set_item("move", i.move_index)?;
                d.set_item("kind", i.kind.as_str())?;
                d.set_item("position", (i.position.x, i.position.y, i.position.z))?;
                d.set_item("value", i.value)?;
                Ok(d)
            })
            .collect()
    }

    /// One dict per probe: `{at, move, position, standoff, incidence}`
    /// (`standoff`/`incidence` `None` when the target was missed) — the
    /// raw material for colouring a path by distance.
    #[getter]
    fn probes<'py>(&self, py: Python<'py>) -> PyResult<Vec<Bound<'py, pyo3::types::PyDict>>> {
        self.inner
            .probes
            .iter()
            .map(|p| {
                let d = pyo3::types::PyDict::new(py);
                d.set_item("at", p.at)?;
                d.set_item("move", p.move_index)?;
                d.set_item("position", (p.position.x, p.position.y, p.position.z))?;
                d.set_item("standoff", p.standoff)?;
                d.set_item("incidence", p.incidence)?;
                Ok(d)
            })
            .collect()
    }

    /// Runs of consecutive flagged samples of one kind, as `(at, at)`
    /// ranges: the stretches of the program to look at.
    fn spans(&self, kind: &str) -> PyResult<Vec<(f64, f64)>> {
        use botrail_scene::coat::PaintIssueKind;
        let kind = match kind {
            "no_target" => PaintIssueKind::NoTarget,
            "too_far" => PaintIssueKind::TooFar,
            "too_close" => PaintIssueKind::TooClose,
            "oblique" => PaintIssueKind::Oblique,
            other => {
                return Err(PyValueError::new_err(format!(
                    "kind must be one of no_target/too_far/too_close/oblique, got {other:?}"
                )))
            }
        };
        Ok(self.inner.spans(kind))
    }

    fn __bool__(&self) -> bool {
        self.inner.ok()
    }

    fn __repr__(&self) -> String {
        let n = self.inner.probes.len();
        use botrail_scene::coat::PaintIssueKind::*;
        let count = |k| self.inner.issues.iter().filter(|i| i.kind == k).count();
        let off = count(NoTarget);
        let on = if off > 0 {
            format!(", {} of {n} off target", off)
        } else {
            format!(", {n} samples")
        };
        if self.inner.ok() {
            format!(
                "PaintReport(ok{on}: standoff {:.0}-{:.0} mm, incidence <= {:.0} deg)",
                self.inner.standoff_min * 1e3,
                self.inner.standoff_max * 1e3,
                self.inner.incidence_max.to_degrees(),
            )
        } else if self.inner.hits == 0 {
            format!("PaintReport(never met the target in {n} samples)")
        } else {
            // Distinct samples: one can be both too far and oblique.
            let flagged = {
                let mut seen: Vec<usize> = self
                    .inner
                    .issues
                    .iter()
                    .filter(|i| i.kind != NoTarget)
                    .map(|i| i.sample)
                    .collect();
                seen.dedup();
                seen.len()
            };
            format!(
                "PaintReport({} of {} on-target samples flagged: {} too far, {} too close, {} oblique{on}; standoff {:.0}-{:.0} mm)",
                flagged,
                self.inner.hits,
                count(TooFar),
                count(TooClose),
                count(Oblique),
                self.inner.standoff_min * 1e3,
                self.inner.standoff_max * 1e3,
            )
        }
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

/// The film a cycle sprayed onto one target: a thickness map plus the
/// numbers. Thicknesses are meters (divide by 1e-6 for microns), volumes
/// cubic meters (1e6 for cc), areas square meters.
#[pyclass(frozen, module = "botrail._core")]
struct FilmCoat {
    inner: botrail_scene::coat::FilmCoat,
}

#[pymethods]
impl FilmCoat {
    /// Area-weighted mean film [m].
    #[getter]
    fn mean(&self) -> f64 {
        self.inner.mean
    }

    /// Thinnest patch [m] — zero whenever anything went uncoated.
    #[getter]
    fn min(&self) -> f64 {
        self.inner.min
    }

    /// Thickest patch [m]. Where runs and sags would start on a real part.
    #[getter]
    fn max(&self) -> f64 {
        self.inner.max
    }

    /// Area-weighted standard deviation of the film [m] — the number that
    /// moves when lap overlap changes.
    #[getter]
    fn sigma(&self) -> f64 {
        self.inner.sigma
    }

    /// Fraction of the target area inside the spec band, or `None` when
    /// `spray_coat` was called without one. The headline quality number.
    #[getter]
    fn in_spec_ratio(&self) -> Option<f64> {
        self.inner.in_spec_ratio
    }

    /// Area that never took any paint [m^2] — holidays. Resolution-bound:
    /// nothing smaller than a patch can be seen.
    #[getter]
    fn uncoated_area(&self) -> f64 {
        self.inner.uncoated_area
    }

    /// Area below / above the spec band [m^2]; zero without a spec.
    #[getter]
    fn thin_area(&self) -> f64 {
        self.inner.thin_area
    }

    #[getter]
    fn thick_area(&self) -> f64 {
        self.inner.thick_area
    }

    /// Area the gun worked over [m^2] — in range and facing it at some
    /// point. Every statistic above is over this, not over the target's
    /// whole skin: a part's back face is not a holiday.
    #[getter]
    fn total_area(&self) -> f64 {
        self.inner.total_area
    }

    /// Whole tessellated area of the target [m^2], worked or not.
    #[getter]
    fn surface_area(&self) -> f64 {
        self.inner.surface_area
    }

    /// Paint delivered while the gun was on [m^3].
    #[getter]
    fn sprayed_volume(&self) -> f64 {
        self.inner.sprayed_volume
    }

    /// Paint that landed on this target [m^3] — anywhere on it, including
    /// the grazing faces the incidence mask keeps out of the statistics.
    /// So this is a shade more than `mean * total_area`.
    #[getter]
    fn deposited_volume(&self) -> f64 {
        self.inner.deposited_volume
    }

    /// Deposited over sprayed. Below the applicator's nominal transfer
    /// efficiency by whatever overshot the part or landed elsewhere.
    #[getter]
    fn effective_transfer_efficiency(&self) -> f64 {
        self.inner.effective_transfer_efficiency()
    }

    #[getter]
    fn gun_on_time(&self) -> f64 {
        self.inner.gun_on_time
    }

    /// Seconds the gun spent closer to the surface than the pattern
    /// measurement can speak for. Nonzero means the film is under-reported
    /// there and the standoff wants looking at — not a rounding detail.
    #[getter]
    fn too_close_time(&self) -> f64 {
        self.inner.too_close_time
    }

    /// Paint that landed on *other* obstacles [m^3], as `{name: volume}`
    /// — the overspray, and where a masking leak shows: a fixture that
    /// took paint is a fixture that was not masked. Enabled obstacles
    /// only. From a ray quadrature of the footprint, so approximate at
    /// the percent level.
    fn overspray<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, pyo3::types::PyDict>> {
        let d = pyo3::types::PyDict::new(py);
        for (name, v) in &self.inner.overspray {
            d.set_item(name, *v)?;
        }
        Ok(d)
    }

    /// Paint that landed nowhere in the scene [m^3]: past every obstacle,
    /// plus the atomization loss (`1 - transfer_efficiency`) that never
    /// reaches any surface. `sprayed - deposited - sum(overspray)`.
    #[getter]
    fn lost_volume(&self) -> f64 {
        self.inner.lost_volume
    }

    /// Paint sprayed per brush [m^3], `{brush: volume}`; a program
    /// without brushes reports one entry named `""`.
    fn sprayed_by_brush<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, pyo3::types::PyDict>> {
        let d = pyo3::types::PyDict::new(py);
        for (name, v) in &self.inner.sprayed_by_brush {
            d.set_item(name, *v)?;
        }
        Ok(d)
    }

    /// Paint that landed on the target per brush [m^3], `{brush: volume}`.
    fn deposited_by_brush<'py>(
        &self,
        py: Python<'py>,
    ) -> PyResult<Bound<'py, pyo3::types::PyDict>> {
        let d = pyo3::types::PyDict::new(py);
        for (name, v) in &self.inner.deposited_by_brush {
            d.set_item(name, *v)?;
        }
        Ok(d)
    }

    #[getter]
    fn patch_count(&self) -> usize {
        self.inner.patch_count
    }

    #[getter]
    fn patch_size(&self) -> f64 {
        self.inner.patch_size
    }

    /// Per-patch film [m], aligned with the mesh triangles.
    #[getter]
    fn thickness(&self) -> Vec<f64> {
        self.inner.thickness.clone()
    }

    /// World pose to place the film map at (the target's pose at coat
    /// time).
    #[getter]
    fn pose(&self) -> hub::PoseArrays {
        let t = self.inner.pose.translation;
        let q = self.inner.pose.rotation.coords;
        ([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
    }

    /// Writes the film map as OBJ plus a sibling `.mtl`: the thickness
    /// banded onto a sequential ramp as face colors, bare substrate in a
    /// dark neutral. The studio and the USD export both read face colors
    /// back from this format (add the obstacle *without* a `color=`
    /// override).
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
        let spec = match self.inner.in_spec_ratio {
            Some(r) => format!(", {:.1}% in spec", r * 100.0),
            None => String::new(),
        };
        format!(
            "FilmCoat({:.1} um mean, {:.1}-{:.1} um{}, {:.1} cc on target, {} patches)",
            self.inner.mean * 1e6,
            self.inner.min * 1e6,
            self.inner.max * 1e6,
            spec,
            self.inner.deposited_volume * 1e6,
            self.inner.patch_count,
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
/// The JSON Schema (draft 2020-12) of the `.botrail` project file, as a
/// string — generated from the Rust types the loader reads, so it is
/// always the contract `Scene.load_project` enforces. Doc comments are
/// the descriptions. Write it out for an editor or hand it to an agent
/// that authors projects directly.
#[pyfunction]
fn project_schema() -> String {
    botrail_scene::project::ProjectFile::json_schema()
}

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
    m.add_class::<IoPoint>()?;
    m.add_class::<IoFinding>()?;
    m.add_class::<IoReport>()?;
    m.add_class::<IoMap>()?;
    m.add_class::<Bom>()?;
    m.add_class::<CellReport>()?;
    m.add_class::<ToolpathReport>()?;
    m.add_class::<FeedReport>()?;
    m.add_class::<StockCarve>()?;
    m.add_class::<FilmCoat>()?;
    m.add_class::<PaintReport>()?;
    m.add_class::<StudioServer>()?;
    m.add_function(wrap_pyfunction!(serve_studio, m)?)?;
    m.add_function(wrap_pyfunction!(catalog::catalog_package, m)?)?;
    m.add_function(wrap_pyfunction!(project_schema, m)?)?;
    m.add_function(wrap_pyfunction!(_parse_gcode_json, m)?)?;
    m.add_function(wrap_pyfunction!(_parse_apt_json, m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}

//! Vectorised live rollouts for `botrail.rl` (design-rl.md R2): N worlds
//! of one cell stepped together with the GIL released.

use std::sync::Arc;

use numpy::{IntoPyArray, PyArray1, PyArray2, PyArrayMethods, PyReadonlyArray1};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use botrail_session::SessionHost;

use botrail_scene::rl::{Control, ObsSpec, PolicyDriver, PolicyInput, StepResult, VecOptions};
use pyo3::types::PyDict;

use super::{backend_named, hub::SceneHub, physics_engine, spin_mode, LiveRollout, Scene};

/// One step's report per world: `(t, finished, error, collisions,
/// contacts, ik_failed)`.
type StepTuple = (
    f64,
    bool,
    Option<String>,
    Vec<(String, String)>,
    Vec<(String, String, f64)>,
    bool,
);

fn step_tuple(r: StepResult) -> StepTuple {
    (
        r.t,
        r.finished,
        r.error,
        r.collisions,
        r.contacts
            .into_iter()
            .map(|c| (c.a, c.b, c.force))
            .collect(),
        r.ik_failed,
    )
}

/// N live rollouts of one cell, stepped together — the engine under
/// `botrail.rl.VecEnv`. Every world is one scene's rollout opened the
/// way `Scene.open_rollout` opens it (physics, programs, sensors) with
/// the task robot driven; `step_all` commands all of them, advances each
/// `k` scan ticks on its own thread, and packs the observations the spec
/// names into one flat row per world.
#[pyclass(module = "botrail._core")]
pub struct VecRollout {
    inner: botrail_scene::rl::VecRollout,
    opts: VecOptions,
    engine: Option<String>,
    /// Per world: the snapshot its rollout runs against and the hub that
    /// owns the live scene (for a finished world's studio playback).
    snapshots: Vec<botrail_scene::Scene>,
    hubs: Vec<Arc<SceneHub>>,
    robot_name: String,
}

#[pymethods]
impl VecRollout {
    /// Opens one world per scene. `spec` is the JSON channel list
    /// (`[{"kind": "joints", "robot": ..., "velocities": true}, ...]`),
    /// resolved against the first scene; `robot` (default: the sole
    /// robot) and `group` name what is driven, `max_velocity` caps the
    /// drive. `physics` is the bake's `physics=` argument.
    #[new]
    #[pyo3(signature = (scenes, sequences, spec, robot = None, group = None, max_velocity = None, dt = 0.01, max_duration = 120.0, plan_resolution = None, toolpath_spin = None, physics = None, control = None, seed = 0))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        py: Python<'_>,
        scenes: Vec<PyRef<'_, Scene>>,
        sequences: Vec<String>,
        spec: &str,
        robot: Option<&str>,
        group: Option<&str>,
        max_velocity: Option<f64>,
        dt: f64,
        max_duration: f64,
        plan_resolution: Option<f64>,
        toolpath_spin: Option<&str>,
        physics: Option<&Bound<'_, PyAny>>,
        control: Option<&str>,
        seed: u64,
    ) -> PyResult<Self> {
        if scenes.is_empty() {
            return Err(PyValueError::new_err(
                "a VecRollout needs at least one scene",
            ));
        }
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
        let engine = physics_engine(physics)?;
        let hubs: Vec<Arc<SceneHub>> = scenes.iter().map(|s| s.hub.clone()).collect();
        let snapshots: Vec<botrail_scene::Scene> = hubs.iter().map(|h| h.snapshot()).collect();
        let first = &snapshots[0];
        let robot_index = match robot {
            Some(name) => first.robot_index(name).ok_or_else(|| {
                PyValueError::new_err(format!(
                    "unknown robot `{name}` (have: {})",
                    first
                        .robots()
                        .iter()
                        .map(|r| format!("{:?}", r.name))
                        .collect::<Vec<_>>()
                        .join(", ")
                ))
            })?,
            None => match first.robots().len() {
                1 => 0,
                0 => return Err(PyValueError::new_err("the scene has no robot")),
                _ => {
                    return Err(PyValueError::new_err(
                        "the scene has several robots; pass robot=",
                    ))
                }
            },
        };
        let robot_name = first.robots()[robot_index].name.clone();
        let group = match group {
            Some(name) => Some(
                first.robots()[robot_index]
                    .model
                    .group_index(name)
                    .ok_or_else(|| PyValueError::new_err(format!("unknown group `{name}`")))?,
            ),
            None => None,
        };
        let spec = ObsSpec::from_json(spec, first).map_err(PyValueError::new_err)?;
        let control = match control {
            Some(json) => Some(
                Control::from_json(json, first, robot_index, group)
                    .map_err(PyValueError::new_err)?,
            ),
            None => None,
        };
        let opts = VecOptions {
            names: sequences,
            options,
            robot: robot_index,
            group,
            max_velocity,
            seed,
        };
        let inner = py
            .allow_threads(|| {
                botrail_scene::rl::VecRollout::open(
                    &snapshots,
                    &opts,
                    &|| backend_named(&engine),
                    spec,
                    control,
                )
            })
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(VecRollout {
            inner,
            opts,
            engine,
            snapshots,
            hubs,
            robot_name,
        })
    }

    fn __len__(&self) -> usize {
        self.inner.len()
    }

    /// Width of one observation row.
    #[getter]
    fn obs_dim(&self) -> usize {
        self.inner.obs_dim()
    }

    /// Widths per channel, in spec order.
    #[getter]
    fn dims(&self) -> Vec<usize> {
        self.inner.spec().dims()
    }

    /// Joints of the driven robot (one command row's width).
    #[getter]
    fn dof(&self) -> usize {
        self.inner.dof()
    }

    /// The driven robot's name.
    #[getter]
    fn robot(&self) -> String {
        self.robot_name.clone()
    }

    /// Width of one action (`0` without a control).
    #[getter]
    fn action_dim(&self) -> usize {
        self.inner.action_dim()
    }

    /// Like `step_all`, taking one action row per world (`N × action_dim`)
    /// instead of joint commands: each world's control (bound at
    /// construction) maps its action to the command — a `TcpDelta`'s
    /// setpoint integration and IK run here, per world, on its thread.
    /// The last element of each report says whether that IK failed.
    #[pyo3(signature = (k, actions, skip = None))]
    fn step_actions<'py>(
        &mut self,
        py: Python<'py>,
        k: u32,
        actions: PyReadonlyArray1<'py, f64>,
        skip: Option<Vec<bool>>,
    ) -> PyResult<(Bound<'py, PyArray2<f64>>, Vec<StepTuple>)> {
        let n = self.inner.len();
        let adim = self.inner.action_dim();
        let actions = actions.as_slice()?;
        if actions.len() != n * adim {
            return Err(PyValueError::new_err(format!(
                "actions: expected {n} × {adim} values, got {}",
                actions.len()
            )));
        }
        let skip = skip.unwrap_or_default();
        if !skip.is_empty() && skip.len() != n {
            return Err(PyValueError::new_err(format!(
                "skip: expected {n} flags, got {}",
                skip.len()
            )));
        }
        let dim = self.inner.obs_dim();
        let mut obs = vec![0.0; n * dim];
        let inner = &mut self.inner;
        let results = py
            .allow_threads(|| inner.step_actions(k, actions, &mut obs, &skip))
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok((
            rows(py, obs, n, dim)?,
            results.into_iter().map(step_tuple).collect(),
        ))
    }

    /// Commands every world (`commands` is a flat `N × dof` list),
    /// advances each `k` scan ticks and returns the flat `N × dim`
    /// observations plus one `(t, finished, error, collisions, contacts)`
    /// per world. A world flagged in `skip` is only observed (its first
    /// observation after a reset). The GIL is released while the worlds
    /// step.
    #[pyo3(signature = (k, commands, skip = None))]
    fn step_all<'py>(
        &mut self,
        py: Python<'py>,
        k: u32,
        commands: PyReadonlyArray1<'py, f64>,
        skip: Option<Vec<bool>>,
    ) -> PyResult<(Bound<'py, PyArray2<f64>>, Vec<StepTuple>)> {
        let n = self.inner.len();
        let commands = commands.as_slice()?;
        if commands.len() != n * self.inner.dof() {
            return Err(PyValueError::new_err(format!(
                "commands: expected {} × {} values, got {}",
                n,
                self.inner.dof(),
                commands.len()
            )));
        }
        let skip = skip.unwrap_or_default();
        if !skip.is_empty() && skip.len() != n {
            return Err(PyValueError::new_err(format!(
                "skip: expected {n} flags, got {}",
                skip.len()
            )));
        }
        let dim = self.inner.obs_dim();
        let mut obs = vec![0.0; n * dim];
        let inner = &mut self.inner;
        let results = py.allow_threads(|| inner.step_all(k, commands, &mut obs, &skip));
        Ok((
            rows(py, obs, n, dim)?,
            results.into_iter().map(step_tuple).collect(),
        ))
    }

    /// The `(N, dim)` observations as the worlds stand.
    fn observe_all<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let (n, dim) = (self.inner.len(), self.inner.obs_dim());
        let mut obs = vec![0.0; n * dim];
        self.inner.observe_all(&mut obs);
        rows(py, obs, n, dim)
    }

    /// The driven robot's joints, `(N, dof)`.
    fn joints_all<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let (n, dof) = (self.inner.len(), self.inner.dof());
        let mut out = vec![0.0; n * dof];
        self.inner.joints_all(&mut out);
        rows(py, out, n, dof)
    }

    /// The driven robot's TCP poses, `(N, 7)` (position + xyzw).
    fn tcp_all<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let n = self.inner.len();
        let mut out = vec![0.0; n * 7];
        self.inner.tcp_all(&mut out);
        rows(py, out, n, 7)
    }

    /// Replaces the worlds at `indices` with fresh rollouts of `scenes`
    /// (one per index, in order) — the reset of finished episodes. The
    /// scenes are snapshotted now, so randomise them first.
    fn reopen(
        &mut self,
        py: Python<'_>,
        indices: Vec<usize>,
        scenes: Vec<PyRef<'_, Scene>>,
    ) -> PyResult<()> {
        if indices.len() != scenes.len() {
            return Err(PyValueError::new_err(format!(
                "{} indices for {} scenes",
                indices.len(),
                scenes.len()
            )));
        }
        let hubs: Vec<Arc<SceneHub>> = scenes.iter().map(|s| s.hub.clone()).collect();
        let snapshots: Vec<botrail_scene::Scene> = hubs.iter().map(|h| h.snapshot()).collect();
        let (inner, opts, engine) = (&mut self.inner, &self.opts, &self.engine);
        py.allow_threads(|| inner.reopen(&indices, &snapshots, opts, &|| backend_named(engine)))
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        for ((&i, snapshot), hub) in indices.iter().zip(snapshots).zip(hubs) {
            self.snapshots[i] = snapshot;
            self.hubs[i] = hub;
        }
        Ok(())
    }

    /// Sets the light world `index`'s colour pictures are lit by (see
    /// `LiveRollout.set_lighting`).
    #[pyo3(signature = (index, light, ambient = 0.35, shadows = false))]
    fn set_lighting(
        &mut self,
        index: usize,
        light: [f64; 3],
        ambient: f64,
        shadows: bool,
    ) -> PyResult<()> {
        let lighting = botrail_scene::raster::Lighting::new(
            nalgebra::Vector3::new(light[0], light[1], light[2]),
            ambient,
        )
        .with_shadows(shadows);
        if self.inner.set_lighting(index, lighting) {
            Ok(())
        } else {
            Err(PyValueError::new_err(format!(
                "no live world at index {index}"
            )))
        }
    }

    /// Sets the decimation cell (m) of world `index`'s pictures (see
    /// `LiveRollout.set_render_decimate`).
    #[pyo3(signature = (index, cell))]
    fn set_render_decimate(&mut self, index: usize, cell: Option<f64>) -> PyResult<()> {
        if let Some(c) = cell {
            if !(c.is_finite() && c > 0.0) {
                return Err(PyValueError::new_err(
                    "decimate cell must be positive (or None)",
                ));
            }
        }
        if self.inner.set_render_decimate(index, cell) {
            Ok(())
        } else {
            Err(PyValueError::new_err(format!(
                "no live world at index {index}"
            )))
        }
    }

    /// Takes world `index` out as a `LiveRollout` (to read it, or to
    /// `finish()` it into a timeline); the slot is dead until `reopen`ed.
    fn take(&mut self, index: usize) -> PyResult<LiveRollout> {
        let live = self
            .inner
            .take(index)
            .ok_or_else(|| PyValueError::new_err(format!("no live world at index {index}")))?;
        Ok(LiveRollout {
            inner: Some(live),
            scene: self.snapshots[index].clone(),
            hub: self.hubs[index].clone(),
            label: self.opts.names.join(" + "),
            scenario: None,
            spec: None,
        })
    }
}

/// A flat row-major buffer as an `(n, width)` array, moved, not copied.
fn rows(
    py: Python<'_>,
    data: Vec<f64>,
    n: usize,
    width: usize,
) -> PyResult<Bound<'_, PyArray2<f64>>> {
    data.into_pyarray(py)
        .reshape([n, width])
        .map_err(|e| PyValueError::new_err(e.to_string()))
}

/// A flat buffer as a 1-D array, moved, not copied.
pub fn row(py: Python<'_>, data: Vec<f64>) -> Bound<'_, PyArray1<f64>> {
    data.into_pyarray(py)
}

/// A Python policy as the rollout's driver (design-rl.md R3): an object
/// with a `channels` attribute (the JSON channel list `botrail.rl`
/// lowers a task's observations to) that is called with one dict per
/// decision — `t`, `step`, `obs`, `q`, `tcp`, `collisions`, `contacts` —
/// and answers the next joint target (a sequence of floats) or `None`
/// when its step is done. `botrail.rl.Policy` is that object.
pub struct PyPolicyDriver {
    name: String,
    callable: Py<PyAny>,
    channels: String,
    /// The control spec (`control` attribute) the returned actions go
    /// through; without one the callable returns joint targets.
    control: Option<String>,
}

impl PolicyDriver for PyPolicyDriver {
    fn channels(&self) -> &str {
        &self.channels
    }

    fn control(&self) -> Option<&str> {
        self.control.as_deref()
    }

    fn act(&mut self, input: &PolicyInput<'_>) -> Result<Option<Vec<f64>>, String> {
        Python::with_gil(|py| -> PyResult<Option<Vec<f64>>> {
            let call = PyDict::new(py);
            call.set_item("t", input.t)?;
            call.set_item("step", input.step)?;
            call.set_item("obs", input.obs.to_vec())?;
            call.set_item("q", input.q.to_vec())?;
            let t = input.tcp.translation;
            let r = input.tcp.rotation.coords;
            call.set_item("tcp", ([t.x, t.y, t.z], [r.x, r.y, r.z, r.w]))?;
            call.set_item("collisions", input.collisions.to_vec())?;
            call.set_item(
                "contacts",
                input
                    .contacts
                    .iter()
                    .map(|c| (c.a.clone(), c.b.clone(), c.force))
                    .collect::<Vec<_>>(),
            )?;
            let out = self.callable.bind(py).call1((call,))?;
            if out.is_none() {
                return Ok(None);
            }
            let target: Vec<f64> = out.extract().map_err(|_| {
                PyValueError::new_err(
                    "a policy must return its action (a sequence of floats) or None",
                )
            })?;
            Ok(Some(target))
        })
        .map_err(|e| format!("{} ({e})", self.name))
    }
}

/// Lowers a `policies={name: policy}` argument to the rollout's drivers.
pub fn policy_drivers(
    policies: Option<&Bound<'_, PyDict>>,
) -> PyResult<Vec<(String, Box<dyn PolicyDriver>)>> {
    let mut out: Vec<(String, Box<dyn PolicyDriver>)> = Vec::new();
    let Some(policies) = policies else {
        return Ok(out);
    };
    for (key, value) in policies.iter() {
        let name: String = key
            .extract()
            .map_err(|_| PyValueError::new_err("policies: keys are the policy names (str)"))?;
        if !value.is_callable() {
            return Err(PyValueError::new_err(format!(
                "policies[{name:?}] is not callable: pass a botrail.rl.Policy (or any callable \
                 with a `channels` attribute)"
            )));
        }
        let channels: String = value
            .getattr("channels")
            .and_then(|c| c.extract())
            .map_err(|_| {
                PyValueError::new_err(format!(
                    "policies[{name:?}] has no `channels` attribute (the JSON channel list a \
                     botrail.rl.Policy carries)"
                ))
            })?;
        let control: Option<String> = match value.getattr("control") {
            Ok(c) if !c.is_none() => Some(c.extract().map_err(|_| {
                PyValueError::new_err(format!(
                    "policies[{name:?}].control must be the JSON control spec (str)"
                ))
            })?),
            _ => None,
        };
        out.push((
            name.clone(),
            Box::new(PyPolicyDriver {
                name,
                callable: value.clone().unbind(),
                channels,
                control,
            }),
        ));
    }
    Ok(out)
}

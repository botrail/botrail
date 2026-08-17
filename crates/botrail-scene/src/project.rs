//! The `.botrail` project format: a self-contained JSON snapshot of the
//! robots (embedded source + world base pose + joint state), obstacles, and
//! motions — plus a Python code generator that reproduces the project
//! programmatically.
//!
//! Version 2 stores robots as a list; each entry carries its instance
//! name, source, base pose, and joint state. Version 1 files (single
//! `robot_urdf`, base implicitly at the world origin) are still read.
//!
//! Known limitation: mesh assets are referenced by filesystem path, not
//! embedded (asset bundling arrives with the zip-based v3 format). In a
//! project file a mesh obstacle's `url` field holds that local path.
//! Primitive-only robots and scenes are fully self-contained.

use std::sync::Arc;

use nalgebra::Isometry3;
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::wire::{
    device_from_msg, device_msg, geometry_from_msg, geometry_msg, motion_from_msg, motion_msg,
    sensor_from_msg, sensor_msg, sequence_from_msg, sequence_msg, signal_def_msg, ActionMsg,
    ConditionMsg, ConstraintMsg, DeviceMsg, FrameMsg, GeometryMsg, MotionMsg, ObstacleMsg, PoseMsg,
    SegmentKindMsg, SensorMsg, SequenceMsg, SignalDefMsg,
};
use crate::{ObstacleSpec, Scene};

pub const PROJECT_VERSION: u32 = 2;

#[derive(Debug, Error)]
pub enum ProjectError {
    #[error("unsupported project version {0} (expected <= {PROJECT_VERSION})")]
    Version(u32),
    #[error("invalid project JSON: {0}")]
    Json(String),
    #[error("embedded robot failed to parse: {0}")]
    Robot(String),
    #[error("project does not fit this scene: {0}")]
    Incompatible(String),
    #[error("{0}")]
    Scene(String),
}

/// Where a project robot comes from.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum RobotSourceMsg {
    /// URDF XML (xacro already expanded), embedded verbatim.
    Urdf { xml: String },
    /// USD stage reference (local path until asset bundling lands). The
    /// application layer re-imports it via botrail-usd on load.
    Usd {
        path: String,
        articulation_root: String,
    },
    /// A catalog package (`Robot.from_catalog`): id + pinned revision for
    /// provenance and script export, plus the fetched inner source so
    /// loading needs no network.
    Catalog {
        id: String,
        revision: String,
        #[serde(default)]
        tcp: Option<String>,
        #[serde(default)]
        flange: Option<String>,
        #[serde(default)]
        mount: Option<String>,
        inner: Box<RobotSourceMsg>,
    },
    /// A tool welded onto a base robot (`Robot.attach_tool`): both part
    /// sources plus the weld parameters, so loading re-runs the attach.
    Composite {
        base: Box<RobotSourceMsg>,
        tool: Box<RobotSourceMsg>,
        flange: String,
        mount: String,
        offset: PoseMsg,
        #[serde(default)]
        tcp: Option<String>,
        #[serde(default)]
        prefix: Option<String>,
    },
}

/// [`RobotSourceMsg`] from a model's provenance record.
fn robot_source_msg(source: &botrail_model::RobotSource) -> RobotSourceMsg {
    match source {
        botrail_model::RobotSource::UrdfXml(xml) => RobotSourceMsg::Urdf { xml: xml.clone() },
        botrail_model::RobotSource::Usd {
            path,
            articulation_root,
        } => RobotSourceMsg::Usd {
            path: path.display().to_string(),
            articulation_root: articulation_root.clone(),
        },
        botrail_model::RobotSource::Catalog {
            id,
            revision,
            tcp,
            flange,
            mount,
            inner,
        } => RobotSourceMsg::Catalog {
            id: id.clone(),
            revision: revision.clone(),
            tcp: tcp.clone(),
            flange: flange.clone(),
            mount: mount.clone(),
            inner: Box::new(robot_source_msg(inner)),
        },
        botrail_model::RobotSource::Composite {
            base,
            tool,
            flange,
            mount,
            offset,
            tcp,
            prefix,
        } => RobotSourceMsg::Composite {
            base: Box::new(robot_source_msg(base)),
            tool: Box::new(robot_source_msg(tool)),
            flange: flange.clone(),
            mount: mount.clone(),
            offset: PoseMsg::from(offset),
            tcp: tcp.clone(),
            prefix: prefix.clone(),
        },
    }
}

/// Rebuilds a robot model from its persisted source. `import_usd` supplies
/// the USD importer, which lives outside this crate — pass a closure that
/// errors when importing is unavailable in the caller's context.
pub fn model_from_source(
    msg: &RobotSourceMsg,
    import_usd: &dyn Fn(&str, &str) -> Result<botrail_model::RobotModel, String>,
) -> Result<botrail_model::RobotModel, ProjectError> {
    match msg {
        RobotSourceMsg::Urdf { xml } => botrail_model::RobotModel::from_urdf_str(xml)
            .map_err(|e| ProjectError::Robot(e.to_string())),
        RobotSourceMsg::Usd {
            path,
            articulation_root,
        } => import_usd(path, articulation_root).map_err(ProjectError::Robot),
        RobotSourceMsg::Catalog {
            id,
            revision,
            tcp,
            flange,
            mount,
            inner,
        } => {
            // Rebuild from the embedded inner source (no network), then
            // restore the catalog provenance and manifest frames on the
            // model.
            let mut model = model_from_source(inner, import_usd)?;
            if let Some(tcp) = tcp {
                model.tcp_link = model.link_index(tcp);
            }
            if let Some(flange) = flange {
                model.flange_link = model.link_index(flange);
            }
            if let Some(mount) = mount {
                model.mount_link = model.link_index(mount);
            }
            let inner_source = std::mem::replace(
                &mut model.source,
                botrail_model::RobotSource::UrdfXml(String::new()),
            );
            model.source = botrail_model::RobotSource::Catalog {
                id: id.clone(),
                revision: revision.clone(),
                tcp: tcp.clone(),
                flange: flange.clone(),
                mount: mount.clone(),
                inner: Box::new(inner_source),
            };
            Ok(model)
        }
        RobotSourceMsg::Composite {
            base,
            tool,
            flange,
            mount,
            offset,
            tcp,
            prefix,
        } => {
            let base = model_from_source(base, import_usd)?;
            let tool = model_from_source(tool, import_usd)?;
            base.attach_tool(
                &tool,
                Some(flange),
                Some(mount),
                offset.into(),
                tcp.as_deref(),
                prefix.as_deref(),
            )
            .map_err(|e| ProjectError::Robot(e.to_string()))
        }
    }
}

/// One robot in a project.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProjectRobotMsg {
    /// Scene-unique instance name; `None` (older files) falls back to the
    /// model name.
    #[serde(default)]
    pub name: Option<String>,
    pub source: RobotSourceMsg,
    /// World pose of the robot's root link.
    pub base_pose: PoseMsg,
    pub joint_positions: Vec<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProjectFile {
    pub version: u32,
    pub robots: Vec<ProjectRobotMsg>,
    pub obstacles: Vec<ObstacleMsg>,
    pub motions: Vec<MotionMsg>,
    /// Named world frames (absent in older v2 files).
    #[serde(default)]
    pub frames: Vec<FrameMsg>,
    /// PLC-style sequences (absent in older v2 files).
    #[serde(default)]
    pub sequences: Vec<SequenceMsg>,
    /// Internal signal declarations (absent in older v2 files).
    #[serde(default)]
    pub signals: Vec<SignalDefMsg>,
    /// Pseudo-sensors (absent in older v2 files).
    #[serde(default)]
    pub sensors: Vec<SensorMsg>,
    /// Auxiliary devices (absent in older v2 files).
    #[serde(default)]
    pub devices: Vec<DeviceMsg>,
    /// Weld-flash bindings (absent in older files).
    #[serde(default)]
    pub flashes: Vec<crate::wire::FlashMsg>,
    /// Scenarios — named initial-state deltas (absent in older files).
    #[serde(default)]
    pub scenarios: Vec<crate::wire::ScenarioMsg>,
    /// Toolpaths — Cartesian process paths (absent in older files).
    #[serde(default)]
    pub toolpaths: Vec<crate::toolpath::ToolpathMsg>,
    /// Process-contact exemptions, by names (absent in older files).
    #[serde(default)]
    pub allowed_contacts: Vec<AllowedContactMsg>,
    /// Spray applicators by name (absent in older files).
    #[serde(default)]
    pub applicators: Vec<ApplicatorMsg>,
    /// Brushes — named process settings toolpath strokes run with (absent
    /// in older files).
    #[serde(default)]
    pub brushes: Vec<crate::coat::Brush>,
    /// The I/O map's assignment layer — controller nodes, point → channel
    /// bindings, declarations (absent in older files). The points
    /// themselves are derived, never stored.
    #[serde(default)]
    pub io: crate::iomap::IoMap,
}

/// One named [`crate::coat::Applicator`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApplicatorMsg {
    pub name: String,
    #[serde(flatten)]
    pub applicator: crate::coat::Applicator,
}

/// One [`crate::AllowedContact`] by names: robot instance, link, obstacle.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AllowedContactMsg {
    pub robot: String,
    pub link: String,
    pub obstacle: String,
}

fn identity_pose() -> PoseMsg {
    PoseMsg {
        position: [0.0; 3],
        quaternion: [0.0, 0.0, 0.0, 1.0],
    }
}

impl ProjectFile {
    pub fn from_json(json: &str) -> Result<Self, ProjectError> {
        #[derive(Deserialize)]
        struct VersionProbe {
            version: u32,
        }
        let probe: VersionProbe =
            serde_json::from_str(json).map_err(|e| ProjectError::Json(e.to_string()))?;
        match probe.version {
            1 => {
                /// The v1 layout: one implicit robot at the world origin.
                #[derive(Deserialize)]
                struct ProjectV1 {
                    robot_urdf: String,
                    joint_positions: Vec<f64>,
                    obstacles: Vec<ObstacleMsg>,
                    motions: Vec<MotionMsg>,
                }
                let v1: ProjectV1 =
                    serde_json::from_str(json).map_err(|e| ProjectError::Json(e.to_string()))?;
                Ok(ProjectFile {
                    version: PROJECT_VERSION,
                    robots: vec![ProjectRobotMsg {
                        name: None,
                        source: RobotSourceMsg::Urdf { xml: v1.robot_urdf },
                        base_pose: identity_pose(),
                        joint_positions: v1.joint_positions,
                    }],
                    obstacles: v1.obstacles,
                    motions: v1.motions,
                    frames: Vec::new(),
                    sequences: Vec::new(),
                    signals: Vec::new(),
                    sensors: Vec::new(),
                    devices: Vec::new(),
                    flashes: Vec::new(),
                    scenarios: Vec::new(),
                    toolpaths: Vec::new(),
                    allowed_contacts: Vec::new(),
                    applicators: Vec::new(),
                    brushes: Vec::new(),
                    io: crate::iomap::IoMap::default(),
                })
            }
            2 => {
                let project: ProjectFile =
                    serde_json::from_str(json).map_err(|e| ProjectError::Json(e.to_string()))?;
                if project.robots.is_empty() {
                    return Err(ProjectError::Incompatible(
                        "project has no robots".to_string(),
                    ));
                }
                Ok(project)
            }
            v => Err(ProjectError::Version(v)),
        }
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).expect("project serializes infallibly")
    }
}

/// Project files reference mesh files by local path, carried in the wire
/// geometry's `url` field.
fn mesh_path_url(path: &std::path::Path) -> (String, String) {
    let ext = path
        .extension()
        .map(|e| e.to_string_lossy().to_lowercase())
        .unwrap_or_default();
    (path.display().to_string(), ext)
}

/// The inverse of [`mesh_path_url`]: rebuilds model geometry, mapping a
/// mesh's `url` back to a filesystem path (unlike the wire-facing
/// `geometry_from_msg`, which rejects meshes from clients).
fn geometry_from_project(msg: &GeometryMsg) -> Result<botrail_model::Geometry, String> {
    match msg {
        GeometryMsg::Mesh { url, scale, .. } => Ok(botrail_model::Geometry::Mesh {
            path: url.into(),
            scale: nalgebra::Vector3::new(scale[0], scale[1], scale[2]),
        }),
        other => geometry_from_msg(other),
    }
}

impl Scene {
    pub fn to_project(&self) -> ProjectFile {
        let mut mesh_url = mesh_path_url;
        ProjectFile {
            version: PROJECT_VERSION,
            robots: self
                .robots()
                .iter()
                .map(|r| ProjectRobotMsg {
                    name: Some(r.name.clone()),
                    source: robot_source_msg(&r.model.source),
                    base_pose: PoseMsg::from(r.base_pose()),
                    joint_positions: r.joint_positions().to_vec(),
                })
                .collect(),
            obstacles: self
                .obstacles()
                .iter()
                .map(|o| ObstacleMsg {
                    name: o.name.clone(),
                    geometry: geometry_msg(&o.geometry, &mut mesh_url),
                    pose: PoseMsg::from(&o.pose),
                    enabled: o.enabled,
                    visible: o.visible,
                    color: o.color,
                    material: o.material.map(Into::into),
                    legend: o.legend.as_ref().map(Into::into),
                    attached_to: self
                        .attachment(&o.name)
                        .map(|a| crate::wire::attachment_msg(self, a)),
                })
                .collect(),
            motions: self.motions().iter().map(|m| motion_msg(self, m)).collect(),
            sequences: self.sequences().iter().map(sequence_msg).collect(),
            signals: self.signals().iter().map(signal_def_msg).collect(),
            sensors: self.sensors().iter().map(sensor_msg).collect(),
            devices: self.devices().iter().map(device_msg).collect(),
            flashes: self
                .weld_flashes()
                .iter()
                .map(|f| crate::wire::FlashMsg {
                    name: f.name.clone(),
                    signal: f.signal.clone(),
                    robot: f.robot.clone(),
                    kind: crate::wire::flash_kind_msg(f.kind),
                    spin_link: f.spin_link.clone(),
                    cone: f.cone.map(|c| [c.length, c.radius]),
                })
                .collect(),
            scenarios: self
                .scenarios()
                .iter()
                .map(crate::wire::scenario_msg)
                .collect(),
            toolpaths: self
                .toolpaths()
                .iter()
                .map(crate::toolpath::toolpath_msg)
                .collect(),
            allowed_contacts: self
                .allowed_contacts()
                .iter()
                .map(|c| AllowedContactMsg {
                    robot: self.robots()[c.robot].name.clone(),
                    link: self.robots()[c.robot].model.links[c.link].name.clone(),
                    obstacle: c.obstacle.clone(),
                })
                .collect(),
            applicators: self
                .applicators()
                .iter()
                .map(|(name, a)| ApplicatorMsg {
                    name: name.clone(),
                    applicator: a.clone(),
                })
                .collect(),
            brushes: self.brushes().to_vec(),
            io: self.io_map().clone(),
            frames: self
                .frames()
                .iter()
                .map(|f| FrameMsg {
                    name: f.name.clone(),
                    pose: PoseMsg::from(&f.pose),
                })
                .collect(),
        }
    }

    /// Builds a fresh scene (robots included) from a project. USD-sourced
    /// robots need the importer and are handled one layer up (botrail-py's
    /// `load_project`), which re-imports each and then calls
    /// `apply_project`.
    pub fn from_project(project: &ProjectFile) -> Result<Scene, ProjectError> {
        let mut models = Vec::with_capacity(project.robots.len());
        for robot_msg in &project.robots {
            let robot = model_from_source(&robot_msg.source, &|_, _| {
                Err(
                    "USD-sourced robot: re-import it via the USD importer, then apply_project"
                        .to_string(),
                )
            })?;
            models.push(Arc::new(robot));
        }
        let mut iter = models.into_iter();
        let mut scene = Scene::new(iter.next().expect("from_json rejects empty robot lists"));
        for model in iter {
            scene.add_robot(model, None, Isometry3::identity());
        }
        scene.apply_project(project)?;
        Ok(scene)
    }

    /// Applies a project's state (names, base poses, joints, obstacles,
    /// motions) onto this scene. The robot models themselves are kept; the
    /// project must have the same robot count and per-robot DOF.
    pub fn apply_project(&mut self, project: &ProjectFile) -> Result<(), ProjectError> {
        if project.robots.len() != self.robots().len() {
            return Err(ProjectError::Incompatible(format!(
                "project has {} robots, scene has {}",
                project.robots.len(),
                self.robots().len()
            )));
        }
        for (i, robot_msg) in project.robots.iter().enumerate() {
            if robot_msg.joint_positions.len() != self.robots()[i].model.dof() {
                return Err(ProjectError::Incompatible(format!(
                    "project robot {i} has {} DOF, scene robot has {}",
                    robot_msg.joint_positions.len(),
                    self.robots()[i].model.dof()
                )));
            }
        }
        // Restore instance names (older files carry none and keep the
        // model-derived defaults); reject duplicates before mutating.
        {
            let mut names: Vec<String> = self.robots().iter().map(|r| r.name.clone()).collect();
            for (i, robot_msg) in project.robots.iter().enumerate() {
                if let Some(name) = &robot_msg.name {
                    names[i] = name.clone();
                }
            }
            let mut seen = std::collections::HashSet::new();
            for name in &names {
                if !seen.insert(name) {
                    return Err(ProjectError::Incompatible(format!(
                        "duplicate robot instance name `{name}`"
                    )));
                }
            }
            for (i, name) in names.into_iter().enumerate() {
                self.robots[i].name = name;
            }
        }
        // Build the new obstacle set before mutating anything.
        let mut obstacles = Vec::with_capacity(project.obstacles.len());
        for o in &project.obstacles {
            let geometry = geometry_from_project(&o.geometry).map_err(ProjectError::Scene)?;
            obstacles.push((
                ObstacleSpec {
                    name: o.name.clone(),
                    geometry,
                    pose: (&o.pose).into(),
                    color: o.color,
                    material: o.material.map(Into::into),
                },
                o.enabled,
                o.visible,
                o.legend.as_ref().map(Into::into),
            ));
        }

        while let Some(existing) = self.obstacles().first().map(|o| o.name.clone()) {
            self.remove_obstacle(&existing)
                .expect("existing obstacle is removable");
        }
        for (spec, enabled, visible, legend) in obstacles {
            let final_name = self
                .add_obstacle(&spec.name, spec.geometry, spec.pose)
                .map_err(|e| ProjectError::Scene(e.to_string()))?;
            self.set_obstacle_color(&final_name, spec.color)
                .expect("obstacle was just added");
            self.set_obstacle_material(&final_name, spec.material)
                .expect("obstacle was just added");
            self.set_obstacle_legend(&final_name, legend)
                .expect("obstacle was just added");
            if !enabled {
                self.set_obstacle_enabled(&final_name, false)
                    .expect("obstacle was just added");
            }
            if !visible {
                self.set_obstacle_visible(&final_name, false)
                    .expect("obstacle was just added");
            }
        }
        for (i, robot_msg) in project.robots.iter().enumerate() {
            self.set_robot_base_pose_for(i, (&robot_msg.base_pose).into());
            self.set_joint_positions_for(i, robot_msg.joint_positions.clone())
                .map_err(|e| ProjectError::Scene(e.to_string()))?;
        }
        // Restore attachments verbatim (stored grasp transforms, not
        // re-captured) once obstacles and joints are in place.
        let mut attachments = Vec::new();
        for o in &project.obstacles {
            if let Some(msg) = &o.attached_to {
                let attachment =
                    crate::wire::attachment_from_msg(self, &o.name, msg).map_err(|name| {
                        ProjectError::Incompatible(format!(
                            "attachment of `{}` references unknown robot or link `{name}`",
                            o.name
                        ))
                    })?;
                attachments.push(attachment);
            }
        }
        self.set_attachments(attachments);
        let mut motions = Vec::with_capacity(project.motions.len());
        for msg in &project.motions {
            motions.push(motion_from_msg(self, msg).map_err(|name| {
                ProjectError::Incompatible(format!(
                    "motion `{}` references unknown robot `{name}`",
                    msg.name
                ))
            })?);
        }
        self.set_motions(motions);
        let mut toolpaths = Vec::with_capacity(project.toolpaths.len());
        for msg in &project.toolpaths {
            toolpaths.push(crate::toolpath::toolpath_from_msg(msg).map_err(|e| {
                ProjectError::Incompatible(format!("toolpath `{}`: {e}", msg.name))
            })?);
        }
        self.set_toolpaths(toolpaths);
        for a in &project.applicators {
            a.applicator
                .validate()
                .map_err(|e| ProjectError::Incompatible(format!("applicator `{}`: {e}", a.name)))?;
        }
        for b in &project.brushes {
            b.validate()
                .map_err(|e| ProjectError::Incompatible(e.to_string()))?;
            if !project.applicators.iter().any(|a| a.name == b.applicator) {
                return Err(ProjectError::Incompatible(format!(
                    "brush `{}` references unknown applicator `{}`",
                    b.name, b.applicator
                )));
            }
        }
        self.set_process_settings(
            project
                .applicators
                .iter()
                .map(|a| (a.name.clone(), a.applicator.clone()))
                .collect(),
            project.brushes.clone(),
        );
        for msg in &project.allowed_contacts {
            let robot = self.robot_index(&msg.robot).ok_or_else(|| {
                ProjectError::Incompatible(format!(
                    "allowed contact references unknown robot `{}`",
                    msg.robot
                ))
            })?;
            let link = self.robots()[robot]
                .model
                .link_index(&msg.link)
                .ok_or_else(|| {
                    ProjectError::Incompatible(format!(
                        "allowed contact references unknown link `{}`",
                        msg.link
                    ))
                })?;
            self.allow_link_obstacle_contact(robot, link, &msg.obstacle)
                .map_err(|e| ProjectError::Incompatible(e.to_string()))?;
        }
        self.set_sequences(project.sequences.iter().map(sequence_from_msg).collect());
        self.set_sensors(project.sensors.iter().map(sensor_from_msg).collect());
        self.set_devices(project.devices.iter().map(device_from_msg).collect());
        self.set_scenarios(
            project
                .scenarios
                .iter()
                .map(crate::wire::scenario_from_msg)
                .collect(),
        );
        self.set_signals(
            project
                .signals
                .iter()
                .map(|s| crate::seq::SignalDef {
                    name: s.name.clone(),
                    initial: s.initial,
                })
                .collect(),
        );
        self.set_frames(
            project
                .frames
                .iter()
                .map(|f| crate::Frame {
                    name: f.name.clone(),
                    pose: (&f.pose).into(),
                })
                .collect(),
        );
        self.set_weld_flashes(
            project
                .flashes
                .iter()
                .map(|f| crate::seq::WeldFlash {
                    name: f.name.clone(),
                    signal: f.signal.clone(),
                    robot: f.robot.clone(),
                    kind: crate::wire::flash_kind_from_msg(f.kind),
                    spin_link: f.spin_link.clone(),
                    cone: f
                        .cone
                        .map(|[length, radius]| crate::seq::SprayCone { length, radius }),
                })
                .collect(),
        );
        self.set_io_map(project.io.clone())
            .map_err(|e| ProjectError::Incompatible(format!("I/O map: {e}")))?;
        Ok(())
    }
}

// ------------------------------------------------------------- codegen

fn py_list(values: &[f64]) -> String {
    let items: Vec<String> = values.iter().map(|v| format!("{v:.6}")).collect();
    format!("[{}]", items.join(", "))
}

fn py_tuple(values: &[f64]) -> String {
    let items: Vec<String> = values.iter().map(|v| format!("{v:.6}")).collect();
    format!("({})", items.join(", "))
}

fn is_identity_pose(pose: &PoseMsg) -> bool {
    pose.position.iter().all(|v| v.abs() < 1e-12)
        && (pose.quaternion[3] - 1.0).abs() < 1e-12
        && pose.quaternion[..3].iter().all(|v| v.abs() < 1e-12)
}

/// The `, robot="…"` kwarg selecting robot `i` — empty for single-robot
/// projects, where the implicit default keeps the script identical to the
/// pre-multi-robot output.
fn robot_kwarg(project: &ProjectFile, i: usize) -> String {
    if project.robots.len() == 1 {
        return String::new();
    }
    match &project.robots[i].name {
        Some(name) => format!(", robot={name:?}"),
        // Multi-robot projects written by botrail always carry names; a
        // hand-edited file without them cannot be addressed reliably.
        None => String::new(),
    }
}

/// `robot_kwarg` looked up by stored instance name (attachment/motion
/// references), falling back to the first robot.
fn robot_kwarg_for_name(project: &ProjectFile, name: &Option<String>) -> String {
    let index = name
        .as_ref()
        .and_then(|n| {
            project
                .robots
                .iter()
                .position(|r| r.name.as_ref() == Some(n))
        })
        .unwrap_or(0);
    robot_kwarg(project, index)
}

/// Emits Python that builds `source` into the variable `var`; embedded URDF
/// text goes into an `r'''...'''` constant named `konst`. Composites emit
/// their parts first (`{var}_tool`, `{konst}_TOOL`), then the attach call.
fn emit_robot_build(out: &mut String, source: &RobotSourceMsg, var: &str, konst: &str) {
    match source {
        RobotSourceMsg::Urdf { xml } => {
            // Triple-quote guard: a URDF containing ''' would break the literal.
            let urdf = xml.replace("'''", "'\\''\\''\\'");
            out.push_str(&format!("{konst} = r'''{urdf}'''\n\n"));
            out.push_str(&format!("{var} = bt.Robot.from_urdf_string({konst})\n"));
        }
        RobotSourceMsg::Usd {
            path,
            articulation_root,
        } => {
            out.push_str(&format!(
                "{var} = bt.Robot.from_usd({path:?}, articulation_root={articulation_root:?})\n"
            ));
        }
        RobotSourceMsg::Catalog { id, revision, .. } => {
            // Deterministic re-fetch: the pinned revision makes this the
            // same bytes the project was authored from.
            out.push_str(&format!(
                "{var} = bt.Robot.from_catalog({id:?}, revision={revision:?})\n"
            ));
        }
        RobotSourceMsg::Composite {
            base,
            tool,
            flange,
            mount,
            offset,
            tcp,
            prefix,
        } => {
            emit_robot_build(out, base, var, konst);
            let tool_var = format!("{var}_tool");
            emit_robot_build(out, tool, &tool_var, &format!("{konst}_TOOL"));
            let mut kwargs = String::new();
            if !is_identity_pose(offset) {
                kwargs.push_str(&format!(
                    ", offset_position={}, offset_quaternion={}",
                    py_tuple(&offset.position),
                    py_tuple(&offset.quaternion)
                ));
            }
            if let Some(tcp) = tcp {
                kwargs.push_str(&format!(", tcp={tcp:?}"));
            }
            if let Some(prefix) = prefix {
                kwargs.push_str(&format!(", prefix={prefix:?}"));
            }
            out.push_str(&format!(
                "{var} = {var}.attach_tool({tool_var}, flange={flange:?}, mount={mount:?}{kwargs})\n"
            ));
        }
    }
}

/// Generates a standalone Python script that rebuilds the project with the
/// botrail API. The robot sources are embedded so the script is
/// self-contained.
pub fn generate_python(project: &ProjectFile) -> String {
    if project.robots.is_empty() {
        return "# cannot generate script: project has no robots\n".to_string();
    }
    let multi = project.robots.len() > 1;
    let mut out = String::new();
    out.push_str("\"\"\"Generated by botrail studio — rebuilds the saved project.\"\"\"\n\n");
    if project.applicators.is_empty() && project.io.nodes.is_empty() {
        out.push_str("import botrail as bt\n\n");
    } else {
        // Applicators and I/O node channel lists are emitted as JSON
        // literals (see below).
        out.push_str("import json\n\nimport botrail as bt\n\n");
    }
    for (i, robot_msg) in project.robots.iter().enumerate() {
        let var = if i == 0 {
            "robot".to_string()
        } else {
            format!("robot_{}", i + 1)
        };
        let konst = if i == 0 {
            "URDF".to_string()
        } else {
            format!("URDF_{}", i + 1)
        };
        emit_robot_build(&mut out, &robot_msg.source, &var, &konst);
        let mut kwargs = String::new();
        if multi {
            if let Some(name) = &robot_msg.name {
                kwargs.push_str(&format!(", name={name:?}"));
            }
        }
        if !is_identity_pose(&robot_msg.base_pose) {
            kwargs.push_str(&format!(
                ", base_position={}, base_quaternion={}",
                py_tuple(&robot_msg.base_pose.position),
                py_tuple(&robot_msg.base_pose.quaternion)
            ));
        }
        if i == 0 {
            out.push_str(&format!("scene = bt.Scene({var}{kwargs})\n"));
        } else {
            out.push_str(&format!("scene.add_robot({var}{kwargs})\n"));
        }
    }

    for o in &project.obstacles {
        let pos = py_tuple(&o.pose.position);
        let quat = py_tuple(&o.pose.quaternion);
        match &o.geometry {
            GeometryMsg::Box { size } => out.push_str(&format!(
                "scene.add_box({:?}, size={}, position={}, quaternion={})\n",
                o.name,
                py_tuple(size),
                pos,
                quat
            )),
            GeometryMsg::Sphere { radius } => out.push_str(&format!(
                "scene.add_sphere({:?}, radius={radius}, position={pos}, quaternion={quat})\n",
                o.name
            )),
            GeometryMsg::Cylinder { radius, length } => out.push_str(&format!(
                "scene.add_cylinder({:?}, radius={radius}, length={length}, position={pos}, quaternion={quat})\n",
                o.name
            )),
            GeometryMsg::Mesh { url, scale, .. } => out.push_str(&format!(
                "scene.add_mesh({:?}, path={url:?}, position={pos}, scale={}, quaternion={quat})\n",
                o.name,
                py_tuple(scale)
            )),
        }
        // Switches and appearance are part of the scene, so a rebuild has
        // to carry them: a script that comes back in bare grey, with every
        // collision proxy on show, has not rebuilt the cell.
        if !o.enabled {
            out.push_str(&format!(
                "scene.set_obstacle_enabled({:?}, False)\n",
                o.name
            ));
        }
        if !o.visible {
            out.push_str(&format!(
                "scene.set_obstacle_visible({:?}, False)\n",
                o.name
            ));
        }
        if let Some([r, g, b]) = o.color {
            out.push_str(&format!(
                "scene.set_obstacle_color({:?}, ({r}, {g}, {b}))\n",
                o.name
            ));
        }
        if let Some(m) = o.material {
            out.push_str(&format!(
                "scene.set_obstacle_material({:?}, metalness={}, roughness={})\n",
                o.name, m.metalness, m.roughness
            ));
        }
    }
    for frame in &project.frames {
        out.push_str(&format!(
            "scene.add_frame({:?}, position={}, quaternion={})\n",
            frame.name,
            py_tuple(&frame.pose.position),
            py_tuple(&frame.pose.quaternion)
        ));
    }
    for (i, robot_msg) in project.robots.iter().enumerate() {
        out.push_str(&format!(
            "scene.set_joint_positions({}{})\n",
            py_list(&robot_msg.joint_positions),
            robot_kwarg(project, i)
        ));
    }
    // Attach after obstacles and joints so the captured grasp matches the
    // saved relative pose.
    for o in &project.obstacles {
        if let Some(att) = &o.attached_to {
            let touch: Vec<String> = att.touch_links.iter().map(|l| format!("{l:?}")).collect();
            out.push_str(&format!(
                "scene.attach({:?}, link={:?}, touch_links=[{}]{})\n",
                o.name,
                att.link,
                touch.join(", "),
                robot_kwarg_for_name(project, &att.robot)
            ));
        }
    }

    for motion in &project.motions {
        out.push('\n');
        for segment in &motion.segments {
            let kind = match segment.kind {
                SegmentKindMsg::Joint => "joint",
                SegmentKindMsg::CartesianLine => "cartesian_line",
            };
            let mut extras = String::new();
            for constraint in &segment.constraints {
                match constraint {
                    ConstraintMsg::OrientationCone {
                        axis_local,
                        axis_world,
                        angle,
                    } => extras.push_str(&format!(
                        ", orientation_cone=({}, {}, {angle})",
                        py_tuple(axis_local),
                        py_tuple(axis_world)
                    )),
                    ConstraintMsg::PositionBox { min, max } => extras.push_str(&format!(
                        ", position_box=({}, {})",
                        py_tuple(min),
                        py_tuple(max)
                    )),
                }
            }
            out.push_str(&format!(
                "scene.add_segment({:?}, goal={}, kind={:?}{}{})\n",
                motion.name,
                py_list(&segment.goal_positions),
                kind,
                extras,
                robot_kwarg_for_name(project, &motion.robot)
            ));
        }
        out.push_str(&format!(
            "trajectory = scene.plan_motion({:?})\n",
            motion.name
        ));
    }

    for sensor in &project.sensors {
        let watch = match &sensor.watch {
            crate::wire::SensorWatchMsg::AllObjects => String::new(),
            crate::wire::SensorWatchMsg::Objects { names } => {
                let items: Vec<String> = names.iter().map(|n| format!("{n:?}")).collect();
                format!(", watch=[{}]", items.join(", "))
            }
            crate::wire::SensorWatchMsg::Robot => ", watch=[], watch_robot=True".to_string(),
            crate::wire::SensorWatchMsg::Robots { names } => {
                let items: Vec<String> = names.iter().map(|n| format!("{n:?}")).collect();
                format!(", watch=[], watch_robots=[{}]", items.join(", "))
            }
            crate::wire::SensorWatchMsg::All => ", watch_robot=True".to_string(),
        };
        match &sensor.kind {
            crate::wire::SensorKindMsg::Zone { pose, size } => out.push_str(&format!(
                "scene.add_zone_sensor({:?}, position={}, size={}, quaternion={}{watch})\n",
                sensor.name,
                py_tuple(&pose.position),
                py_tuple(size),
                py_tuple(&pose.quaternion),
            )),
            crate::wire::SensorKindMsg::Beam { from, to, radius } => out.push_str(&format!(
                "scene.add_beam_sensor({:?}, frm={}, to={}, radius={radius}{watch})\n",
                sensor.name,
                py_tuple(from),
                py_tuple(to),
            )),
        }
    }
    for device in &project.devices {
        match &device.kind {
            crate::wire::DeviceKindMsg::Conveyor {
                zone_pose,
                zone_size,
                velocity,
                running,
            } => out.push_str(&format!(
                "scene.add_conveyor({:?}, zone_position={}, zone_size={}, velocity={}, zone_quaternion={}, running={})\n",
                device.name,
                py_tuple(&zone_pose.position),
                py_tuple(zone_size),
                py_tuple(velocity),
                py_tuple(&zone_pose.quaternion),
                if *running { "True" } else { "False" },
            )),
            crate::wire::DeviceKindMsg::LinearAxis {
                objects,
                axis,
                speed,
                position,
                range,
            } => {
                let items: Vec<String> = objects.iter().map(|n| format!("{n:?}")).collect();
                out.push_str(&format!(
                    "scene.add_linear_axis({:?}, objects=[{}], axis={}, speed={speed}, range={}, position={position})\n",
                    device.name,
                    items.join(", "),
                    py_tuple(axis),
                    py_tuple(range),
                ));
            }
            crate::wire::DeviceKindMsg::Source {
                pool,
                park,
                pitch,
                pose,
                interval,
                running,
            } => {
                let items: Vec<String> = pool.iter().map(|n| format!("{n:?}")).collect();
                out.push_str(&format!(
                    "scene.add_source({:?}, pool=[{}], park={}, pitch={}, position={}, interval={interval}, running={})\n",
                    device.name,
                    items.join(", "),
                    py_tuple(&park.position),
                    py_tuple(pitch),
                    py_tuple(&pose.position),
                    if *running { "True" } else { "False" },
                ));
            }
            crate::wire::DeviceKindMsg::Sink {
                zone_pose,
                zone_size,
                source,
            } => out.push_str(&format!(
                "scene.add_sink({:?}, zone_position={}, zone_size={}, source={source:?})\n",
                device.name,
                py_tuple(&zone_pose.position),
                py_tuple(zone_size),
            )),
            crate::wire::DeviceKindMsg::Vehicle {
                path,
                body,
                speed,
                turn_speed,
                start,
                allow_reverse,
                tray,
            } => {
                let waypoints: Vec<String> = path
                    .waypoints
                    .iter()
                    .map(|p| format!("({}, {})", p[0], p[1]))
                    .collect();
                let stations: Vec<String> = path
                    .stations
                    .iter()
                    .map(|s| format!("{:?}: {}", s.name, s.index))
                    .collect();
                let members: Vec<String> = body.iter().map(|n| format!("{n:?}")).collect();
                let deck = match tray {
                    Some(t) => format!(
                        ", tray_position={}, tray_size={}",
                        py_tuple(&t.pose.position),
                        py_tuple(&t.size)
                    ),
                    None => String::new(),
                };
                out.push_str(&format!(
                    "scene.add_vehicle({:?}, body=[{}], path=[{}], stations={{{}}}, \
                     speed={speed}, turn_speed={turn_speed}, start={start:?}{}{deck})\n",
                    device.name,
                    members.join(", "),
                    waypoints.join(", "),
                    stations.join(", "),
                    if path.ring { ", ring=True" } else { "" },
                ));
                if *allow_reverse {
                    out.truncate(out.len() - 1);
                    out.push_str(", allow_reverse=True)\n");
                }
            }
        }
    }
    for signal in &project.signals {
        out.push_str(&format!(
            "scene.define_signal({:?}, initial={})\n",
            signal.name,
            if signal.initial { "True" } else { "False" }
        ));
    }
    for flash in &project.flashes {
        match flash.kind {
            crate::wire::FlashKindMsg::Flash => out.push_str(&format!(
                "scene.add_weld_flash({:?}, signal={:?}, robot={:?})\n",
                flash.name, flash.signal, flash.robot
            )),
            crate::wire::FlashKindMsg::Trace => {
                let spin = match &flash.spin_link {
                    Some(link) => format!(", spin_link={link:?}"),
                    None => String::new(),
                };
                out.push_str(&format!(
                    "scene.add_cut_trace({:?}, signal={:?}, robot={:?}{spin})\n",
                    flash.name, flash.signal, flash.robot
                ));
            }
            crate::wire::FlashKindMsg::Spray => {
                let [length, radius] = flash.cone.unwrap_or([0.25, 0.08]);
                out.push_str(&format!(
                    "scene.add_spray_cone({:?}, signal={:?}, robot={:?}, length={length}, radius={radius})\n",
                    flash.name, flash.signal, flash.robot
                ));
            }
        }
    }
    for a in &project.applicators {
        // The applicator as the dict `bt.paint.applicator` builds — the
        // JSON is that dict, so it round-trips as a literal.
        let json = serde_json::to_string(&a.applicator).unwrap_or_default();
        out.push_str(&format!(
            "scene.define_applicator({:?}, json.loads({json:?}))\n",
            a.name
        ));
    }
    for b in &project.brushes {
        let mut kwargs = format!("flow={}", b.flow);
        if b.lead != 0.0 {
            kwargs.push_str(&format!(", lead={}", b.lead));
        }
        if b.lag != 0.0 {
            kwargs.push_str(&format!(", lag={}", b.lag));
        }
        out.push_str(&format!(
            "scene.define_brush({:?}, applicator={:?}, {kwargs})\n",
            b.name, b.applicator
        ));
    }
    for tp in &project.toolpaths {
        out.push('\n');
        let frame_kwarg = match &tp.frame {
            Some(f) => format!("frame={f:?}"),
            None => String::new(),
        };
        out.push_str(&format!("_tp = bt.toolpath.builder({frame_kwarg})\n"));
        for m in &tp.moves {
            let (targets, feed, brush) = match m {
                crate::toolpath::ToolMoveMsg::Rapid { targets } => (targets, None, None),
                crate::toolpath::ToolMoveMsg::Feed {
                    feed,
                    targets,
                    brush,
                } => (targets, Some(*feed), brush.as_deref()),
            };
            if let Some(f) = feed {
                match brush {
                    Some(b) => out.push_str(&format!("_tp.feed({f}, brush={b:?})\n")),
                    None => out.push_str(&format!("_tp.feed({f})\n")),
                }
            }
            let call = if feed.is_some() {
                "line_to"
            } else {
                "rapid_to"
            };
            for t in targets {
                let mut extras = String::new();
                if t.tool_axis != [0.0, 0.0, 1.0] {
                    extras.push_str(&format!(", axis={}", py_tuple(&t.tool_axis)));
                }
                if let Some(s) = t.spin {
                    extras.push_str(&format!(", spin={s}"));
                }
                out.push_str(&format!("_tp.{call}({}{extras})\n", py_tuple(&t.position)));
            }
        }
        out.push_str(&format!("scene.add_toolpath({:?}, _tp.build())\n", tp.name));
    }
    for contact in &project.allowed_contacts {
        out.push_str(&format!(
            "scene.allow_link_obstacle_contact({:?}, {:?}{})\n",
            contact.link,
            contact.obstacle,
            robot_kwarg_for_name(project, &Some(contact.robot.clone()))
        ));
    }
    for scenario in &project.scenarios {
        let mut kwargs = String::new();
        if !scenario.signals.is_empty() {
            let entries: Vec<String> = scenario
                .signals
                .iter()
                .map(|s| format!("{:?}: {}", s.name, if s.value { "True" } else { "False" }))
                .collect();
            kwargs.push_str(&format!(", signals={{{}}}", entries.join(", ")));
        }
        if !scenario.obstacles.is_empty() {
            let entries: Vec<String> = scenario
                .obstacles
                .iter()
                .map(|o| {
                    format!(
                        "{:?}: ({}, {})",
                        o.name,
                        py_tuple(&o.pose.position),
                        py_tuple(&o.pose.quaternion)
                    )
                })
                .collect();
            kwargs.push_str(&format!(", obstacles={{{}}}", entries.join(", ")));
        }
        if !scenario.joints.is_empty() {
            let entries: Vec<String> = scenario
                .joints
                .iter()
                .map(|j| format!("{:?}: {}", j.robot, py_list(&j.positions)))
                .collect();
            kwargs.push_str(&format!(", joints={{{}}}", entries.join(", ")));
        }
        if !scenario.faults.is_empty() {
            let entries: Vec<String> = scenario
                .faults
                .iter()
                .map(|f| match f {
                    crate::wire::FaultMsg::Stuck { target, value } => {
                        format!(
                            "bt.io.stuck({:?}, {})",
                            target,
                            if *value { "True" } else { "False" }
                        )
                    }
                    crate::wire::FaultMsg::Open { target } => format!("bt.io.open({:?})", target),
                    crate::wire::FaultMsg::NodeDown { target } => {
                        format!("bt.io.node_down({:?})", target)
                    }
                })
                .collect();
            kwargs.push_str(&format!(", faults=[{}]", entries.join(", ")));
        }
        out.push_str(&format!(
            "scene.add_scenario({:?}{kwargs})\n",
            scenario.name
        ));
    }
    for sequence in &project.sequences {
        out.push('\n');
        out.push_str(&format!("sequence = scene.sequence({:?})\n", sequence.name));
        py_steps(&mut out, "sequence", &sequence.steps, 0);
    }
    py_io_map(&mut out, &project.io);

    out.push_str("\nbt.studio(scene)\n");
    out
}

/// Emits the I/O map's assignment layer: nodes (channels as a JSON
/// literal — the dicts `bt.io` templates build), then bindings, then
/// declarations. Nodes first because a binding names its node.
fn py_io_map(out: &mut String, io: &crate::iomap::IoMap) {
    use crate::iomap::{IoDirection, IoNodeKind};
    if io.is_empty() {
        return;
    }
    out.push('\n');
    for node in &io.nodes {
        let mut kwargs = format!("kind={:?}", node.kind.as_str());
        match &node.kind {
            IoNodeKind::RobotController { robots } => {
                kwargs.push_str(&format!(", robots={}", py_str_list(robots)));
            }
            IoNodeKind::Other { label } => kwargs.push_str(&format!(", label={label:?}")),
            _ => {}
        }
        if !node.programs.is_empty() {
            kwargs.push_str(&format!(", programs={}", py_str_list(&node.programs)));
        }
        if let Some(uplink) = &node.uplink {
            match &uplink.bus {
                Some(bus) => kwargs.push_str(&format!(", uplink=({:?}, {:?})", uplink.parent, bus)),
                None => kwargs.push_str(&format!(", uplink={:?}", uplink.parent)),
            }
        }
        if !node.channels.is_empty() {
            let json = serde_json::to_string(&node.channels).unwrap_or_default();
            kwargs.push_str(&format!(", channels=json.loads({json:?})"));
        }
        if let Some(place) = &node.place {
            kwargs.push_str(&format!(", place={place:?}"));
        }
        if let Some(model) = &node.model {
            kwargs.push_str(&format!(", model={model:?}"));
        }
        out.push_str(&format!("scene.add_io_node({:?}, {kwargs})\n", node.name));
    }
    for b in &io.bindings {
        let method = match b.point.direction {
            IoDirection::Input => "bind_input",
            IoDirection::Output => "bind_output",
        };
        let mut kwargs = String::new();
        if let Some(tag) = &b.tag {
            kwargs.push_str(&format!(", tag={tag:?}"));
        }
        if let Some(field) = &b.field {
            kwargs.push_str(&format!(", field={field:?}"));
        }
        if b.invert {
            kwargs.push_str(", invert=True");
        }
        if let Some(contact) = b.contact {
            kwargs.push_str(&format!(
                ", contact={:?}",
                contact.as_str().to_ascii_lowercase()
            ));
        }
        if b.safety {
            kwargs.push_str(", safety=True");
        }
        if let Some(e) = &b.device {
            if let Some(v) = e.voltage {
                kwargs.push_str(&format!(", voltage={v}"));
            }
            if let Some(l) = e.logic {
                kwargs.push_str(&format!(", logic={:?}", l.as_str()));
            }
        }
        if let Some(note) = &b.note {
            kwargs.push_str(&format!(", note={note:?}"));
        }
        out.push_str(&format!(
            "scene.{method}({:?}, {:?}, {:?}{kwargs})\n",
            b.point.label(),
            b.node,
            b.channel
        ));
    }
    for d in &io.decls {
        let mut kwargs = String::new();
        if let Some(role) = d.role {
            kwargs.push_str(&format!(", role={:?}", role.as_str()));
        }
        if let Some(kind) = d.kind {
            kwargs.push_str(&format!(", kind={:?}", kind.as_str().to_ascii_lowercase()));
        }
        if d.safety {
            kwargs.push_str(", safety=True");
        }
        if let Some(pair) = &d.pair {
            kwargs.push_str(&format!(", pair={pair:?}"));
        }
        if let Some(note) = &d.note {
            kwargs.push_str(&format!(", note={note:?}"));
        }
        out.push_str(&format!("scene.declare_io({:?}{kwargs})\n", d.name));
    }
}

fn py_str_list(items: &[String]) -> String {
    let parts: Vec<String> = items.iter().map(|s| format!("{s:?}")).collect();
    format!("[{}]", parts.join(", "))
}

/// Emits `owner.step(...)` lines, recursing into branch arms with
/// depth-suffixed `sel`/`arm` variables (rebinding across sequential
/// branches at one depth is fine — each is finished before the next).
fn py_steps(out: &mut String, owner: &str, steps: &[crate::wire::StepMsg], depth: usize) {
    let suffix = |base: &str| {
        if depth == 0 {
            base.to_string()
        } else {
            format!("{base}{}", depth + 1)
        }
    };
    for step in steps {
        if step.select.is_empty() {
            let actions: Vec<String> = step.actions.iter().map(py_action).collect();
            out.push_str(&format!(
                "{owner}.step({:?}, actions=[{}], transition={})\n",
                step.name,
                actions.join(", "),
                py_condition(&step.transition)
            ));
        } else {
            let sel = suffix("sel");
            let arm = suffix("arm");
            out.push_str(&format!("{sel} = {owner}.select({:?})\n", step.name));
            for select_arm in &step.select {
                out.push_str(&format!(
                    "{arm} = {sel}.when({})\n",
                    py_condition(&select_arm.condition)
                ));
                py_steps(out, &arm, &select_arm.steps, depth + 1);
            }
        }
    }
}

fn py_action(action: &ActionMsg) -> String {
    // A `robot=` kwarg appears only when the action names one, so
    // single-robot scripts keep their pre-multi-robot output byte for byte.
    let robot_kwarg = |robot: &Option<String>| match robot {
        Some(name) => format!(", robot={name:?}"),
        None => String::new(),
    };
    match action {
        ActionMsg::StartMotion { motion } => format!("bt.seq.motion({motion:?})"),
        ActionMsg::StartToolpath { robot, toolpath } => {
            format!("bt.seq.toolpath({toolpath:?}{})", robot_kwarg(robot))
        }
        ActionMsg::StartRamp {
            robot,
            targets,
            duration,
        } => {
            let entries: Vec<String> = targets
                .iter()
                .map(|t| format!("{:?}: {:.6}", t.joint, t.value))
                .collect();
            format!(
                "bt.seq.ramp({{{}}}, duration={duration}{})",
                entries.join(", "),
                robot_kwarg(robot)
            )
        }
        ActionMsg::Attach {
            robot,
            object,
            link,
            touch_links,
        } => {
            let mut extras = String::new();
            if let Some(link) = link {
                extras.push_str(&format!(", link={link:?}"));
            }
            if let Some(touch) = touch_links {
                let names: Vec<String> = touch.iter().map(|t| format!("{t:?}")).collect();
                extras.push_str(&format!(", touch_links=[{}]", names.join(", ")));
            }
            extras.push_str(&robot_kwarg(robot));
            format!("bt.seq.attach({object:?}{extras})")
        }
        ActionMsg::Detach { object } => format!("bt.seq.detach({object:?})"),
        ActionMsg::Track {
            robot,
            object,
            link,
        } => match link {
            Some(link) => format!(
                "bt.seq.track({object:?}, link={link:?}{})",
                robot_kwarg(robot)
            ),
            None => format!("bt.seq.track({object:?}{})", robot_kwarg(robot)),
        },
        ActionMsg::Untrack { robot } => match robot {
            Some(name) => format!("bt.seq.untrack(robot={name:?})"),
            None => "bt.seq.untrack()".to_string(),
        },
        ActionMsg::Set { signal, value } => format!(
            "bt.seq.set_signal({signal:?}, {})",
            if *value { "True" } else { "False" }
        ),
        ActionMsg::Device { device, command } => match command {
            crate::wire::DeviceCommandMsg::Start => format!("bt.seq.start({device:?})"),
            crate::wire::DeviceCommandMsg::Stop => format!("bt.seq.stop({device:?})"),
            crate::wire::DeviceCommandMsg::SetSpeed { speed } => {
                format!("bt.seq.set_speed({device:?}, {speed})")
            }
            crate::wire::DeviceCommandMsg::MoveTo { position } => {
                format!("bt.seq.move_to({device:?}, {position})")
            }
            crate::wire::DeviceCommandMsg::Goto { station } => {
                format!("bt.seq.goto({device:?}, {station:?})")
            }
            crate::wire::DeviceCommandMsg::Advance { distance } => {
                format!("bt.seq.advance({device:?}, {distance})")
            }
        },
    }
}

/// A condition in the authoring vocabulary (`bt.seq.signal("x", True)`)
/// — the script generator's rendering, reused by the coverage report so
/// an uncovered arm is named the way it was written.
pub(crate) fn py_condition(condition: &ConditionMsg) -> String {
    match condition {
        ConditionMsg::Immediately => "bt.seq.immediately()".to_string(),
        ConditionMsg::Done => "bt.seq.done()".to_string(),
        ConditionMsg::RobotDone { robot } => format!("bt.seq.robot_done({robot:?})"),
        ConditionMsg::Elapsed { seconds } => format!("bt.seq.elapsed({seconds})"),
        ConditionMsg::Signal { name, value } => format!(
            "bt.seq.signal({name:?}, {})",
            if *value { "True" } else { "False" }
        ),
        ConditionMsg::Rising { name } => format!("bt.seq.rising({name:?})"),
        ConditionMsg::Falling { name } => format!("bt.seq.falling({name:?})"),
        ConditionMsg::DeviceDone { device } => format!("bt.seq.device_done({device:?})"),
        ConditionMsg::All { conditions } => {
            let inner: Vec<String> = conditions.iter().map(py_condition).collect();
            format!("bt.seq.all_of({})", inner.join(", "))
        }
        ConditionMsg::Any { conditions } => {
            let inner: Vec<String> = conditions.iter().map(py_condition).collect();
            format!("bt.seq.any_of({})", inner.join(", "))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::{Constraint, Segment, SegmentKind};
    use botrail_model::{Geometry, RobotModel};
    use nalgebra::{Isometry3, Vector3};

    const ARM: &str = include_str!("../../../examples/simple_arm.urdf");

    fn sample_scene() -> Scene {
        let mut scene = Scene::new(Arc::new(RobotModel::from_urdf_str(ARM).unwrap()));
        scene
            .add_obstacle(
                "wall",
                Geometry::Box {
                    size: Vector3::new(0.05, 0.8, 0.5),
                },
                Isometry3::translation(0.28, 0.0, 0.45),
            )
            .unwrap();
        scene
            .set_obstacle_color("wall", Some([0.2, 0.4, 0.6]))
            .unwrap();
        scene
            .set_obstacle_material("wall", Some(crate::Material::new(0.8, 0.3)))
            .unwrap();
        scene
            .set_joint_positions(vec![0.1, 0.2, -0.3, 0.0, 0.4, 0.0])
            .unwrap();
        scene
            .add_segment(
                "main",
                Segment {
                    kind: SegmentKind::Joint,
                    goal_positions: vec![0.5, 0.6, -0.7, 0.1, 0.0, 0.0],
                    constraints: vec![Constraint::OrientationCone {
                        axis_local: Vector3::z(),
                        axis_world: Vector3::x(),
                        angle: 0.8,
                    }],
                },
            )
            .unwrap();
        scene
            .add_segment(
                "main",
                Segment {
                    kind: SegmentKind::CartesianLine,
                    goal_positions: vec![0.5, 0.7, -0.9, 0.1, 0.0, 0.0],
                    constraints: vec![],
                },
            )
            .unwrap();
        scene
    }

    #[test]
    fn toolpaths_round_trip_through_project_and_python() {
        use crate::toolpath::{PathTarget, ToolMove, ToolMoveKind, Toolpath};
        use nalgebra::{Point3, Unit};
        let mut scene = sample_scene();
        scene.add_frame("part", Isometry3::translation(0.3, 0.0, 0.2));
        scene.add_toolpath(Toolpath {
            name: "trim".into(),
            frame: Some("part".into()),
            moves: vec![
                ToolMove {
                    kind: ToolMoveKind::Rapid,
                    targets: vec![PathTarget {
                        position: Point3::new(0.0, 0.0, 0.02),
                        tool_axis: Unit::new_normalize(Vector3::z()),
                        spin: None,
                    }],
                    brush: None,
                },
                ToolMove {
                    kind: ToolMoveKind::Feed(0.015),
                    targets: vec![PathTarget {
                        position: Point3::new(0.1, 0.0, 0.0),
                        tool_axis: Unit::new_normalize(Vector3::new(0.1, 0.0, 1.0)),
                        spin: Some(0.4),
                    }],
                    brush: None,
                },
            ],
        });

        let json = scene.to_project().to_json();
        let reloaded = Scene::from_project(&ProjectFile::from_json(&json).unwrap()).unwrap();
        assert_eq!(reloaded.toolpaths().len(), 1);
        let tp = reloaded.toolpath("trim").unwrap();
        assert_eq!(tp.frame.as_deref(), Some("part"));
        assert_eq!(tp.moves.len(), 2);
        assert!(matches!(tp.moves[1].kind, ToolMoveKind::Feed(f) if (f - 0.015).abs() < 1e-12));
        assert_eq!(tp.moves[1].targets[0].spin, Some(0.4));
        // The generated Python re-authors the toolpath through the builder.
        let py = generate_python(&reloaded.to_project());
        assert!(py.contains("bt.toolpath.builder(frame=\"part\")"), "{py}");
        assert!(py.contains("_tp.feed(0.015)"), "{py}");
        assert!(
            py.contains("scene.add_toolpath(\"trim\", _tp.build())"),
            "{py}"
        );
        // Older files without the field still load.
        let mut without: serde_json::Value = serde_json::from_str(&json).unwrap();
        without.as_object_mut().unwrap().remove("toolpaths");
        let legacy = Scene::from_project(
            &ProjectFile::from_json(&serde_json::to_string(&without).unwrap()).unwrap(),
        )
        .unwrap();
        assert!(legacy.toolpaths().is_empty());
    }

    #[test]
    fn io_map_round_trips_through_project_and_python() {
        use crate::iomap::{
            ChannelKind, DeclRole, IoBinding, IoChannel, IoDecl, IoDirection, IoNode, IoNodeKind,
            IoPointId, Uplink,
        };
        let mut scene = sample_scene();
        scene.define_signal("vacuum", false);
        scene
            .upsert_io_node(IoNode {
                name: "PLC1".into(),
                kind: IoNodeKind::Plc,
                programs: vec!["pick".into()],
                uplink: None,
                channels: vec![IoChannel {
                    id: "DO0".into(),
                    kind: ChannelKind::Do,
                    port: None,
                    address: Some("%QX0.0".into()),
                    electrical: None,
                }],
                place: Some("panel".into()),
                model: Some("S7-1200".into()),
            })
            .unwrap();
        scene
            .upsert_io_node(IoNode {
                name: "RIO1".into(),
                kind: IoNodeKind::RemoteIo,
                programs: Vec::new(),
                uplink: Some(Uplink {
                    parent: "PLC1".into(),
                    bus: Some("PROFINET".into()),
                }),
                channels: vec![IoChannel {
                    id: "DI0".into(),
                    kind: ChannelKind::Di,
                    port: None,
                    address: Some("%IX1.0".into()),
                    electrical: Some(crate::iomap::Electrical {
                        voltage: Some(24.0),
                        logic: Some(crate::iomap::Logic::Pnp),
                    }),
                }],
                place: None,
                model: None,
            })
            .unwrap();
        scene
            .bind_io(IoBinding {
                point: IoPointId::parse("vacuum", IoDirection::Output),
                node: "PLC1".into(),
                channel: "DO0".into(),
                tag: Some("Vacuum".into()),
                field: Some("YV1".into()),
                invert: true,
                contact: Some(crate::iomap::Contact::Nc),
                safety: false,
                device: None,
                note: Some("valve".into()),
                auto: false,
            })
            .unwrap();
        scene.declare_io(IoDecl {
            name: "estop_ok".into(),
            role: Some(DeclRole::Input),
            kind: Some(ChannelKind::SafeDi),
            safety: true,
            pair: None,
            note: None,
        });

        let json = scene.to_project().to_json();
        let reloaded = Scene::from_project(&ProjectFile::from_json(&json).unwrap()).unwrap();
        assert_eq!(reloaded.io_map(), scene.io_map());
        // The generated Python re-authors the layer through the API.
        let py = generate_python(&reloaded.to_project());
        assert!(py.contains("import json"), "{py}");
        assert!(
            py.contains("scene.add_io_node(\"PLC1\", kind=\"plc\", programs=[\"pick\"], channels=json.loads("),
            "{py}"
        );
        assert!(py.contains("uplink=(\"PLC1\", \"PROFINET\")"), "{py}");
        assert!(
            py.contains("scene.bind_output(\"vacuum\", \"PLC1\", \"DO0\", tag=\"Vacuum\", field=\"YV1\", invert=True, contact=\"nc\", note=\"valve\")"),
            "{py}"
        );
        assert!(
            py.contains(
                "scene.declare_io(\"estop_ok\", role=\"input\", kind=\"safedi\", safety=True)"
            ),
            "{py}"
        );
        // Older files without the field still load, empty.
        let mut without: serde_json::Value = serde_json::from_str(&json).unwrap();
        without.as_object_mut().unwrap().remove("io");
        let legacy = Scene::from_project(
            &ProjectFile::from_json(&serde_json::to_string(&without).unwrap()).unwrap(),
        )
        .unwrap();
        assert!(legacy.io_map().is_empty());
        // A binding onto a node the file does not have is refused on load.
        let mut broken: serde_json::Value = serde_json::from_str(&json).unwrap();
        broken["io"]["bindings"][0]["node"] = serde_json::Value::String("nope".into());
        let err = Scene::from_project(
            &ProjectFile::from_json(&serde_json::to_string(&broken).unwrap()).unwrap(),
        )
        .err()
        .expect("must fail");
        assert!(err.to_string().contains("I/O map"), "{err}");
    }

    #[test]
    fn project_roundtrip_preserves_everything() {
        let mut scene = sample_scene();
        scene.set_robot_base_pose(Isometry3::translation(0.5, -0.2, 0.8));
        let json = scene.to_project().to_json();
        let reloaded = Scene::from_project(&ProjectFile::from_json(&json).unwrap()).unwrap();

        assert_eq!(reloaded.robot().name, scene.robot().name);
        assert_eq!(reloaded.joint_positions(), scene.joint_positions());
        let base = reloaded.robot_base_pose();
        assert!((base.translation.vector - Vector3::new(0.5, -0.2, 0.8)).norm() < 1e-12);
        assert_eq!(reloaded.obstacles().len(), 1);
        assert_eq!(reloaded.obstacles()[0].name, "wall");
        assert_eq!(reloaded.obstacles()[0].color, Some([0.2, 0.4, 0.6]));
        // Appearance survives the round trip in both channels: a project
        // that comes back grey and matte has not been reloaded.
        assert_eq!(
            reloaded.obstacles()[0].material,
            Some(crate::Material::new(0.8, 0.3))
        );
        assert_eq!(reloaded.motions().len(), 1);
        let motion = &reloaded.motions()[0];
        assert_eq!(motion.name, "main");
        assert_eq!(motion.segments.len(), 2);
        assert_eq!(motion.segments[1].kind, SegmentKind::CartesianLine);
        assert!(matches!(
            motion.segments[0].constraints[0],
            Constraint::OrientationCone { angle, .. } if (angle - 0.8).abs() < 1e-12
        ));
        // The reloaded scene still collision-checks: the collider was rebuilt
        // from the project, so it agrees with the original both on the
        // colliding pairs and on the clearance — and a `None` clearance would
        // mean it came back with no obstacle geometry at all.
        assert_eq!(reloaded.check_collisions(), scene.check_collisions());
        let clearance = reloaded
            .min_obstacle_distance()
            .expect("the rebuilt collider lost its obstacles");
        assert!((clearance - scene.min_obstacle_distance().unwrap()).abs() < 1e-12);
    }

    #[test]
    fn attachments_roundtrip_and_generate_python() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "held",
                Geometry::Sphere { radius: 0.02 },
                Isometry3::translation(0.2, 0.0, 0.4),
            )
            .unwrap();
        let tcp = scene.robot().links[scene.robot().default_tcp_link()]
            .name
            .clone();
        scene.attach_obstacle("held", None, None).unwrap();
        let saved_grasp = scene.attachment("held").unwrap().grasp;

        let json = scene.to_project().to_json();
        let reloaded = Scene::from_project(&ProjectFile::from_json(&json).unwrap()).unwrap();
        let att = reloaded.attachment("held").expect("attachment survives");
        assert_eq!(reloaded.robot().links[att.link].name, tcp);
        assert!(
            (att.grasp.translation.vector - saved_grasp.translation.vector).norm() < 1e-12
                && att.grasp.rotation.angle_to(&saved_grasp.rotation) < 1e-12
        );
        // The reloaded held obstacle still rides the arm.
        assert_eq!(
            reloaded.check_collisions().len(),
            scene.check_collisions().len()
        );

        let code = generate_python(&scene.to_project());
        assert!(
            code.contains(&format!("scene.attach(\"held\", link=\"{tcp}\"")),
            "missing attach line:\n{code}"
        );

        // A project referencing a link the robot doesn't have is rejected.
        let mut bad = scene.to_project();
        bad.obstacles[1].attached_to.as_mut().unwrap().link = "phantom".into();
        let mut other = Scene::new(scene.robot().clone());
        assert!(matches!(
            other.apply_project(&bad),
            Err(ProjectError::Incompatible(_))
        ));
    }

    #[test]
    fn sequences_and_signals_roundtrip_and_generate_python() {
        let mut scene = sample_scene();
        scene.define_signal("armed", true);
        scene.upsert_sensor(crate::seq::Sensor {
            name: "eye".into(),
            kind: crate::seq::SensorKind::Zone {
                pose: Isometry3::translation(0.3, 0.0, 0.5),
                size: Vector3::new(0.1, 0.1, 0.1),
            },
            watch: crate::seq::SensorWatch::AllObjects,
            mount: None,
        });
        scene.upsert_sequence(crate::seq::Sequence {
            name: "cycle".into(),
            steps: vec![
                crate::seq::Step {
                    name: "run".into(),
                    actions: vec![crate::seq::Action::StartMotion {
                        motion: "main".into(),
                    }],
                    transition: crate::seq::Condition::All(vec![
                        crate::seq::Condition::Done,
                        crate::seq::Condition::Signal {
                            name: "armed".into(),
                            value: true,
                        },
                    ]),
                    select: Vec::new(),
                },
                crate::seq::Step {
                    name: "wait".into(),
                    actions: vec![crate::seq::Action::Set {
                        signal: "armed".into(),
                        value: false,
                    }],
                    transition: crate::seq::Condition::Elapsed { seconds: 0.5 },
                    select: Vec::new(),
                },
                crate::seq::Step {
                    name: "next part".into(),
                    actions: vec![],
                    transition: crate::seq::Condition::Rising {
                        name: "armed".into(),
                    },
                    select: Vec::new(),
                },
                crate::seq::Step {
                    name: "judge".into(),
                    actions: vec![],
                    transition: crate::seq::Condition::Immediately,
                    select: vec![
                        crate::seq::SelectArm {
                            condition: crate::seq::Condition::Signal {
                                name: "armed".into(),
                                value: true,
                            },
                            steps: vec![crate::seq::Step {
                                name: "disarm".into(),
                                actions: vec![crate::seq::Action::Set {
                                    signal: "armed".into(),
                                    value: false,
                                }],
                                transition: crate::seq::Condition::Immediately,
                                select: Vec::new(),
                            }],
                        },
                        crate::seq::SelectArm {
                            condition: crate::seq::Condition::Falling {
                                name: "armed".into(),
                            },
                            steps: vec![],
                        },
                    ],
                },
            ],
        });

        let robot = scene.robots()[0].name.clone();
        scene
            .upsert_scenario(crate::seq::Scenario {
                name: "disarmed".into(),
                signals: vec![("armed".into(), false)],
                obstacles: vec![("wall".into(), Isometry3::translation(0.5, 0.0, 0.45))],
                joints: vec![(robot.clone(), vec![0.1, 0.0, 0.0, 0.0, 0.0, 0.0])],
                faults: vec![
                    crate::seq::Fault {
                        target: "armed".into(),
                        kind: crate::seq::FaultKind::StuckAt(true),
                    },
                    crate::seq::Fault {
                        target: "eye".into(),
                        kind: crate::seq::FaultKind::Open,
                    },
                ],
            })
            .unwrap();

        let json = scene.to_project().to_json();
        let doc: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(
            doc["scenarios"][0]["faults"],
            serde_json::json!([
                {"kind": "stuck", "target": "armed", "value": true},
                {"kind": "open", "target": "eye"}
            ])
        );
        let reloaded = Scene::from_project(&ProjectFile::from_json(&json).unwrap()).unwrap();
        assert_eq!(reloaded.signals().len(), 1);
        assert!(reloaded.signals()[0].initial);
        // Scenarios survive the round trip and still apply.
        assert_eq!(reloaded.scenarios().len(), 1);
        let mut applied = reloaded.clone();
        applied.apply_scenario("disarmed").unwrap();
        assert!(!applied.signals()[0].initial);
        // The faults came back and resolved: the open eye reads low (nothing bound).
        assert_eq!(
            applied.forced_inputs(),
            &[("armed".to_string(), true), ("eye".to_string(), false)]
        );
        assert!(
            reloaded.forced_inputs().is_empty(),
            "the live scene never carries forces"
        );
        assert!((applied.obstacles()[0].pose.translation.x - 0.5).abs() < 1e-9);
        assert!((applied.joint_positions()[0] - 0.1).abs() < 1e-9);
        let seq = reloaded.sequence("cycle").expect("sequence survives");
        assert_eq!(seq.steps.len(), 4);
        assert!(matches!(
            &seq.steps[0].transition,
            crate::seq::Condition::All(cs) if cs.len() == 2
        ));
        // Edges and branch arms make the round trip intact.
        assert!(matches!(
            &seq.steps[2].transition,
            crate::seq::Condition::Rising { name } if name == "armed"
        ));
        assert_eq!(seq.steps[3].select.len(), 2);
        assert_eq!(seq.steps[3].select[0].steps.len(), 1);
        assert!(seq.steps[3].select[1].steps.is_empty());

        let code = generate_python(&scene.to_project());
        for needle in [
            "scene.define_signal(\"armed\", initial=True)",
            "sequence = scene.sequence(\"cycle\")",
            "bt.seq.motion(\"main\")",
            "bt.seq.all_of(bt.seq.done(), bt.seq.signal(\"armed\", True))",
            "bt.seq.set_signal(\"armed\", False)",
            "bt.seq.elapsed(0.5)",
            "bt.seq.rising(\"armed\")",
            "sel = sequence.select(\"judge\")",
            "arm = sel.when(bt.seq.signal(\"armed\", True))",
            "arm.step(\"disarm\", actions=[bt.seq.set_signal(\"armed\", False)], \
             transition=bt.seq.immediately())",
            "arm = sel.when(bt.seq.falling(\"armed\"))",
            "scene.add_scenario(\"disarmed\", signals={\"armed\": False}, \
             obstacles={\"wall\": ((0.500000, 0.000000, 0.450000), \
             (0.000000, 0.000000, 0.000000, 1.000000))}, joints={",
            "]}, faults=[bt.io.stuck(\"armed\", True), bt.io.open(\"eye\")])",
        ] {
            assert!(code.contains(needle), "missing `{needle}`:\n{code}");
        }

        // Older projects without the fields still read.
        let mut bare: serde_json::Value = serde_json::from_str(&json).unwrap();
        bare.as_object_mut().unwrap().remove("sequences");
        bare.as_object_mut().unwrap().remove("signals");
        let project = ProjectFile::from_json(&bare.to_string()).unwrap();
        assert!(project.sequences.is_empty() && project.signals.is_empty());
    }

    #[test]
    fn v1_project_reads_with_identity_base() {
        // A minimal v1 file (flat robot_urdf, no robots list).
        let v1 = serde_json::json!({
            "version": 1,
            "robot_urdf": ARM,
            "joint_positions": [0.1, 0.2, -0.3, 0.0, 0.4, 0.0],
            "obstacles": [],
            "motions": [],
        })
        .to_string();
        let project = ProjectFile::from_json(&v1).unwrap();
        assert_eq!(project.version, PROJECT_VERSION);
        let scene = Scene::from_project(&project).unwrap();
        assert_eq!(scene.robot_base_pose(), &Isometry3::identity());
        assert_eq!(scene.joint_positions()[1], 0.2);
    }

    #[test]
    fn version_mismatch_is_rejected() {
        let mut project = sample_scene().to_project();
        project.version = 99;
        let json = project.to_json();
        assert!(matches!(
            ProjectFile::from_json(&json),
            Err(ProjectError::Version(99))
        ));
    }

    #[test]
    fn empty_robot_lists_are_rejected() {
        let mut project = sample_scene().to_project();
        project.robots.clear();
        assert!(matches!(
            ProjectFile::from_json(&project.to_json()),
            Err(ProjectError::Incompatible(_))
        ));
    }

    #[test]
    fn robot_count_and_duplicate_names_are_checked() {
        let mut project = sample_scene().to_project();
        project.robots.push(project.robots[0].clone());
        // Two robots into a one-robot scene: count mismatch.
        let mut scene = sample_scene();
        assert!(matches!(
            scene.apply_project(&project),
            Err(ProjectError::Incompatible(_))
        ));
        // Duplicate instance names are rejected.
        let mut scene = sample_scene();
        scene.add_robot(
            scene.robot().clone(),
            None,
            Isometry3::translation(2.0, 0.0, 0.0),
        );
        assert!(matches!(
            scene.apply_project(&project),
            Err(ProjectError::Incompatible(_))
        ));
    }

    #[test]
    fn multi_robot_projects_round_trip() {
        let mut scene = sample_scene();
        scene.add_robot(
            scene.robot().clone(),
            Some("second"),
            Isometry3::translation(2.0, 0.0, 0.0),
        );
        scene
            .set_joint_positions_for(1, vec![0.3, -0.2, 0.1, 0.0, 0.0, 0.6])
            .unwrap();
        // A motion owned by the second robot, and a grasp by it.
        scene
            .add_segment_for(
                1,
                "second_move",
                Segment {
                    kind: SegmentKind::Joint,
                    goal_positions: vec![0.0; 6],
                    constraints: vec![],
                },
            )
            .unwrap();
        scene
            .add_obstacle(
                "held",
                Geometry::Sphere { radius: 0.02 },
                Isometry3::translation(2.0, 0.0, 0.8),
            )
            .unwrap();
        scene.attach_obstacle_to(1, "held", None, None).unwrap();

        let project = scene.to_project();
        assert_eq!(project.robots.len(), 2);
        assert_eq!(project.robots[1].name.as_deref(), Some("second"));
        assert_eq!(project.robots[1].base_pose.position, [2.0, 0.0, 0.0]);
        assert_eq!(project.robots[1].joint_positions[5], 0.6);
        let motion = project
            .motions
            .iter()
            .find(|m| m.name == "second_move")
            .unwrap();
        assert_eq!(motion.robot.as_deref(), Some("second"));

        let reloaded =
            Scene::from_project(&ProjectFile::from_json(&project.to_json()).unwrap()).unwrap();
        assert_eq!(reloaded.robots().len(), 2);
        assert_eq!(reloaded.robots()[1].name, "second");
        assert_eq!(reloaded.robots()[1].base_pose().translation.vector.x, 2.0);
        assert_eq!(reloaded.robots()[1].joint_positions()[5], 0.6);
        let motion = reloaded
            .motions()
            .iter()
            .find(|m| m.name == "second_move")
            .unwrap();
        assert_eq!(motion.robot, 1);
        let att = reloaded.attachment("held").unwrap();
        assert_eq!(att.robot, 1);
        // The generated script re-addresses each robot by name.
        let script = generate_python(&project);
        assert!(script.contains("scene.add_robot(robot_2, name=\"second\""));
        assert!(script.contains("robot=\"second\""));
    }

    #[test]
    fn apply_project_replaces_state_and_checks_dof() {
        let scene = sample_scene();
        let project = scene.to_project();

        let mut other = Scene::new(scene.robot().clone());
        other
            .add_obstacle(
                "old",
                Geometry::Sphere { radius: 0.05 },
                Isometry3::translation(1.0, 0.0, 0.0),
            )
            .unwrap();
        other.apply_project(&project).unwrap();
        assert_eq!(other.obstacles().len(), 1);
        assert_eq!(other.obstacles()[0].name, "wall");
        assert_eq!(other.motions().len(), 1);

        let mut bad = project.clone();
        bad.robots[0].joint_positions = vec![0.0; 3];
        assert!(matches!(
            other.apply_project(&bad),
            Err(ProjectError::Incompatible(_))
        ));
    }

    #[test]
    fn generated_python_contains_the_full_recipe() {
        let mut scene = sample_scene();
        scene.set_robot_base_pose(Isometry3::translation(1.0, 0.0, 0.0));
        let code = generate_python(&scene.to_project());
        for needle in [
            "import botrail as bt",
            "bt.Robot.from_urdf_string(URDF)",
            "base_position=(1.000000, 0.000000, 0.000000)",
            "scene.add_box(\"wall\"",
            // Appearance is part of the recipe: a rebuild that loses it
            // is not a rebuild.
            "scene.set_obstacle_color(\"wall\", (0.2, 0.4, 0.6))",
            "scene.set_obstacle_material(\"wall\", metalness=0.8, roughness=0.3)",
            "scene.set_joint_positions(",
            "scene.add_segment(\"main\"",
            "kind=\"cartesian_line\"",
            "orientation_cone=(",
            "scene.plan_motion(\"main\")",
            "bt.studio(scene)",
        ] {
            assert!(code.contains(needle), "missing `{needle}`:\n{code}");
        }
        // Identity base stays out of the generated constructor.
        let mut plain = sample_scene();
        plain.set_robot_base_pose(Isometry3::identity());
        assert!(generate_python(&plain.to_project()).contains("bt.Scene(robot)\n"));
    }
}

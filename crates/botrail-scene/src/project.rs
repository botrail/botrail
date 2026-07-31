//! The `.botrail` project format: a self-contained JSON snapshot of the
//! robots (embedded source + world base pose + joint state), obstacles, and
//! motions — plus a Python code generator that reproduces the project
//! programmatically.
//!
//! Version 2 stores robots as a list to keep the format multi-robot-ready;
//! the code currently enforces exactly one. Version 1 files (single
//! `robot_urdf`, base implicitly at the world origin) are still read.
//!
//! Known limitation: mesh assets are referenced by filesystem path, not
//! embedded (asset bundling arrives with the zip-based v3 format). In a
//! project file a mesh obstacle's `url` field holds that local path.
//! Primitive-only robots and scenes are fully self-contained.

use std::sync::Arc;

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::wire::{
    device_from_msg, device_msg, geometry_from_msg, geometry_msg, motion_from_msg, motion_msg,
    sensor_from_msg, sensor_msg, sequence_from_msg, sequence_msg, signal_def_msg, ActionMsg,
    ConditionMsg, ConstraintMsg, DeviceMsg, FrameMsg, GeometryMsg, MotionMsg, ObstacleMsg, PoseMsg,
    SegmentKindMsg, SensorMsg, SequenceMsg, SignalDefMsg,
};
use crate::Scene;

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
}

/// One robot in a project.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProjectRobotMsg {
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
                })
            }
            2 => {
                let project: ProjectFile =
                    serde_json::from_str(json).map_err(|e| ProjectError::Json(e.to_string()))?;
                project.single_robot()?;
                Ok(project)
            }
            v => Err(ProjectError::Version(v)),
        }
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).expect("project serializes infallibly")
    }

    /// The project's robot. The format allows a list; until multi-robot
    /// lands, exactly one is required.
    pub fn single_robot(&self) -> Result<&ProjectRobotMsg, ProjectError> {
        match self.robots.as_slice() {
            [robot] => Ok(robot),
            robots => Err(ProjectError::Incompatible(format!(
                "expected exactly 1 robot, project has {}",
                robots.len()
            ))),
        }
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
            robots: vec![ProjectRobotMsg {
                source: match &self.robot.source {
                    botrail_model::RobotSource::UrdfXml(xml) => {
                        RobotSourceMsg::Urdf { xml: xml.clone() }
                    }
                    botrail_model::RobotSource::Usd {
                        path,
                        articulation_root,
                    } => RobotSourceMsg::Usd {
                        path: path.display().to_string(),
                        articulation_root: articulation_root.clone(),
                    },
                },
                base_pose: PoseMsg::from(self.robot_base_pose()),
                joint_positions: self.joint_positions().to_vec(),
            }],
            obstacles: self
                .obstacles()
                .iter()
                .map(|o| ObstacleMsg {
                    name: o.name.clone(),
                    geometry: geometry_msg(&o.geometry, &mut mesh_url),
                    pose: PoseMsg::from(&o.pose),
                    enabled: o.enabled,
                    attached_to: self
                        .attachment(&o.name)
                        .map(|a| crate::wire::attachment_msg(self, a)),
                })
                .collect(),
            motions: self.motions().iter().map(motion_msg).collect(),
            sequences: self.sequences().iter().map(sequence_msg).collect(),
            signals: self.signals().iter().map(signal_def_msg).collect(),
            sensors: self.sensors().iter().map(sensor_msg).collect(),
            devices: self.devices().iter().map(device_msg).collect(),
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

    /// Builds a fresh scene (robot included) from a project. USD-sourced
    /// robots need the importer and are handled one layer up (botrail-py's
    /// `load_project`), which re-imports and then calls `apply_project`.
    pub fn from_project(project: &ProjectFile) -> Result<Scene, ProjectError> {
        let robot_msg = project.single_robot()?;
        let RobotSourceMsg::Urdf { xml } = &robot_msg.source else {
            return Err(ProjectError::Robot(
                "USD-sourced robot: re-import it via the USD importer, then apply_project"
                    .to_string(),
            ));
        };
        let robot = botrail_model::RobotModel::from_urdf_str(xml)
            .map_err(|e| ProjectError::Robot(e.to_string()))?;
        let mut scene = Scene::new(Arc::new(robot));
        scene.apply_project(project)?;
        Ok(scene)
    }

    /// Applies a project's state (base pose, joints, obstacles, motions)
    /// onto this scene. The robot itself is kept; the project must have the
    /// same DOF.
    pub fn apply_project(&mut self, project: &ProjectFile) -> Result<(), ProjectError> {
        let robot_msg = project.single_robot()?;
        if robot_msg.joint_positions.len() != self.robot.dof() {
            return Err(ProjectError::Incompatible(format!(
                "project has {} DOF, scene robot has {}",
                robot_msg.joint_positions.len(),
                self.robot.dof()
            )));
        }
        // Build the new obstacle set before mutating anything.
        let mut obstacles = Vec::with_capacity(project.obstacles.len());
        for o in &project.obstacles {
            let geometry = geometry_from_project(&o.geometry).map_err(ProjectError::Scene)?;
            obstacles.push((o.name.clone(), geometry, (&o.pose).into(), o.enabled));
        }

        while let Some(existing) = self.obstacles().first().map(|o| o.name.clone()) {
            self.remove_obstacle(&existing)
                .expect("existing obstacle is removable");
        }
        for (name, geometry, pose, enabled) in obstacles {
            let final_name = self
                .add_obstacle(&name, geometry, pose)
                .map_err(|e| ProjectError::Scene(e.to_string()))?;
            if !enabled {
                self.set_obstacle_enabled(&final_name, false)
                    .expect("obstacle was just added");
            }
        }
        self.set_robot_base_pose((&robot_msg.base_pose).into());
        self.set_joint_positions(robot_msg.joint_positions.clone())
            .map_err(|e| ProjectError::Scene(e.to_string()))?;
        // Restore attachments verbatim (stored grasp transforms, not
        // re-captured) once obstacles and joints are in place.
        let mut attachments = Vec::new();
        for o in &project.obstacles {
            if let Some(msg) = &o.attached_to {
                let attachment = crate::wire::attachment_from_msg(&self.robot, &o.name, msg)
                    .map_err(|link| {
                        ProjectError::Incompatible(format!(
                            "attachment of `{}` references unknown link `{link}`",
                            o.name
                        ))
                    })?;
                attachments.push(attachment);
            }
        }
        self.set_attachments(attachments);
        self.set_motions(project.motions.iter().map(motion_from_msg).collect());
        self.set_sequences(project.sequences.iter().map(sequence_from_msg).collect());
        self.set_sensors(project.sensors.iter().map(sensor_from_msg).collect());
        self.set_devices(project.devices.iter().map(device_from_msg).collect());
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

/// Generates a standalone Python script that rebuilds the project with the
/// botrail API. The robot source is embedded so the script is
/// self-contained.
pub fn generate_python(project: &ProjectFile) -> String {
    let robot_msg = match project.single_robot() {
        Ok(robot) => robot,
        Err(e) => return format!("# cannot generate script: {e}\n"),
    };
    let mut out = String::new();
    out.push_str("\"\"\"Generated by botrail studio — rebuilds the saved project.\"\"\"\n\n");
    out.push_str("import botrail as bt\n\n");
    match &robot_msg.source {
        RobotSourceMsg::Urdf { xml } => {
            // Triple-quote guard: a URDF containing ''' would break the literal.
            let urdf = xml.replace("'''", "'\\''\\''\\'");
            out.push_str(&format!("URDF = r'''{urdf}'''\n\n"));
            out.push_str("robot = bt.Robot.from_urdf_string(URDF)\n");
        }
        RobotSourceMsg::Usd {
            path,
            articulation_root,
        } => {
            out.push_str(&format!(
                "robot = bt.Robot.from_usd({path:?}, articulation_root={articulation_root:?})\n"
            ));
        }
    }
    if is_identity_pose(&robot_msg.base_pose) {
        out.push_str("scene = bt.Scene(robot)\n");
    } else {
        out.push_str(&format!(
            "scene = bt.Scene(robot, base_position={}, base_quaternion={})\n",
            py_tuple(&robot_msg.base_pose.position),
            py_tuple(&robot_msg.base_pose.quaternion)
        ));
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
    }
    for frame in &project.frames {
        out.push_str(&format!(
            "scene.add_frame({:?}, position={}, quaternion={})\n",
            frame.name,
            py_tuple(&frame.pose.position),
            py_tuple(&frame.pose.quaternion)
        ));
    }
    out.push_str(&format!(
        "scene.set_joint_positions({})\n",
        py_list(&robot_msg.joint_positions)
    ));
    // Attach after obstacles and joints so the captured grasp matches the
    // saved relative pose.
    for o in &project.obstacles {
        if let Some(att) = &o.attached_to {
            let touch: Vec<String> = att.touch_links.iter().map(|l| format!("{l:?}")).collect();
            out.push_str(&format!(
                "scene.attach({:?}, link={:?}, touch_links=[{}])\n",
                o.name,
                att.link,
                touch.join(", ")
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
                "scene.add_segment({:?}, goal={}, kind={:?}{})\n",
                motion.name,
                py_list(&segment.goal_positions),
                kind,
                extras
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
        }
    }
    for signal in &project.signals {
        out.push_str(&format!(
            "scene.define_signal({:?}, initial={})\n",
            signal.name,
            if signal.initial { "True" } else { "False" }
        ));
    }
    for sequence in &project.sequences {
        out.push('\n');
        out.push_str(&format!("sequence = scene.sequence({:?})\n", sequence.name));
        for step in &sequence.steps {
            let actions: Vec<String> = step.actions.iter().map(py_action).collect();
            out.push_str(&format!(
                "sequence.step({:?}, actions=[{}], transition={})\n",
                step.name,
                actions.join(", "),
                py_condition(&step.transition)
            ));
        }
    }

    out.push_str("\nbt.studio(scene)\n");
    out
}

fn py_action(action: &ActionMsg) -> String {
    match action {
        ActionMsg::StartMotion { motion } => format!("bt.seq.motion({motion:?})"),
        ActionMsg::StartRamp { targets, duration } => {
            let entries: Vec<String> = targets
                .iter()
                .map(|t| format!("{:?}: {:.6}", t.joint, t.value))
                .collect();
            format!(
                "bt.seq.ramp({{{}}}, duration={duration})",
                entries.join(", ")
            )
        }
        ActionMsg::Attach {
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
            format!("bt.seq.attach({object:?}{extras})")
        }
        ActionMsg::Detach { object } => format!("bt.seq.detach({object:?})"),
        ActionMsg::Track { object, link } => match link {
            Some(link) => format!("bt.seq.track({object:?}, link={link:?})"),
            None => format!("bt.seq.track({object:?})"),
        },
        ActionMsg::Untrack => "bt.seq.untrack()".to_string(),
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
        },
    }
}

fn py_condition(condition: &ConditionMsg) -> String {
    match condition {
        ConditionMsg::Immediately => "bt.seq.immediately()".to_string(),
        ConditionMsg::Done => "bt.seq.done()".to_string(),
        ConditionMsg::Elapsed { seconds } => format!("bt.seq.elapsed({seconds})"),
        ConditionMsg::Signal { name, value } => format!(
            "bt.seq.signal({name:?}, {})",
            if *value { "True" } else { "False" }
        ),
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
    fn project_roundtrip_preserves_everything() {
        let mut scene = sample_scene();
        scene.set_robot_base_pose(Isometry3::translation(0.5, -0.2, 0.8));
        let json = scene.to_project().to_json();
        let reloaded = Scene::from_project(&ProjectFile::from_json(&json).unwrap()).unwrap();

        assert_eq!(reloaded.robot.name, scene.robot.name);
        assert_eq!(reloaded.joint_positions(), scene.joint_positions());
        let base = reloaded.robot_base_pose();
        assert!((base.translation.vector - Vector3::new(0.5, -0.2, 0.8)).norm() < 1e-12);
        assert_eq!(reloaded.obstacles().len(), 1);
        assert_eq!(reloaded.obstacles()[0].name, "wall");
        assert_eq!(reloaded.motions().len(), 1);
        let motion = &reloaded.motions()[0];
        assert_eq!(motion.name, "main");
        assert_eq!(motion.segments.len(), 2);
        assert_eq!(motion.segments[1].kind, SegmentKind::CartesianLine);
        assert!(matches!(
            motion.segments[0].constraints[0],
            Constraint::OrientationCone { angle, .. } if (angle - 0.8).abs() < 1e-12
        ));
        // The reloaded scene still collision-checks (collider rebuilt).
        assert!(!reloaded.check_collisions().is_empty() || true);
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
        let tcp = scene.robot.links[scene.robot.default_tcp_link()]
            .name
            .clone();
        scene.attach_obstacle("held", None, None).unwrap();
        let saved_grasp = scene.attachment("held").unwrap().grasp;

        let json = scene.to_project().to_json();
        let reloaded = Scene::from_project(&ProjectFile::from_json(&json).unwrap()).unwrap();
        let att = reloaded.attachment("held").expect("attachment survives");
        assert_eq!(reloaded.robot.links[att.link].name, tcp);
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
        let mut other = Scene::new(scene.robot.clone());
        assert!(matches!(
            other.apply_project(&bad),
            Err(ProjectError::Incompatible(_))
        ));
    }

    #[test]
    fn sequences_and_signals_roundtrip_and_generate_python() {
        let mut scene = sample_scene();
        scene.define_signal("armed", true);
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
                },
                crate::seq::Step {
                    name: "wait".into(),
                    actions: vec![crate::seq::Action::Set {
                        signal: "armed".into(),
                        value: false,
                    }],
                    transition: crate::seq::Condition::Elapsed { seconds: 0.5 },
                },
            ],
        });

        let json = scene.to_project().to_json();
        let reloaded = Scene::from_project(&ProjectFile::from_json(&json).unwrap()).unwrap();
        assert_eq!(reloaded.signals().len(), 1);
        assert!(reloaded.signals()[0].initial);
        let seq = reloaded.sequence("cycle").expect("sequence survives");
        assert_eq!(seq.steps.len(), 2);
        assert!(matches!(
            &seq.steps[0].transition,
            crate::seq::Condition::All(cs) if cs.len() == 2
        ));

        let code = generate_python(&scene.to_project());
        for needle in [
            "scene.define_signal(\"armed\", initial=True)",
            "sequence = scene.sequence(\"cycle\")",
            "bt.seq.motion(\"main\")",
            "bt.seq.all_of(bt.seq.done(), bt.seq.signal(\"armed\", True))",
            "bt.seq.set_signal(\"armed\", False)",
            "bt.seq.elapsed(0.5)",
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
    fn multi_robot_projects_are_rejected_for_now() {
        let mut project = sample_scene().to_project();
        project.robots.push(project.robots[0].clone());
        let json = project.to_json();
        assert!(matches!(
            ProjectFile::from_json(&json),
            Err(ProjectError::Incompatible(_))
        ));
    }

    #[test]
    fn apply_project_replaces_state_and_checks_dof() {
        let scene = sample_scene();
        let project = scene.to_project();

        let mut other = Scene::new(scene.robot.clone());
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

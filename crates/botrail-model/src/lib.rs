//! Robot model layer: wraps xurdf's URDF/Xacro parsing into an indexed
//! kinematic tree suitable for FK and scene serialization.

mod mesh_path;

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};
use thiserror::Error;

pub use mesh_path::ModelOptions;

#[derive(Debug, Error)]
pub enum ModelError {
    #[error("failed to parse robot description: {0}")]
    Parse(String),
    #[error("link `{0}` referenced by a joint does not exist")]
    UnknownLink(String),
    #[error("robot has no root link (empty model or kinematic loop)")]
    NoRoot,
    #[error("robot has multiple root links: {0:?}")]
    MultipleRoots(Vec<String>),
    #[error("link `{0}` has more than one parent joint")]
    MultipleParents(String),
    #[error("kinematic loops are not supported (joint `{0}` is unreachable from the root)")]
    Loop(String),
    #[error("unsupported joint type `{joint_type}` on joint `{name}`")]
    UnsupportedJointType { name: String, joint_type: String },
    #[error("joint `{0}` has a zero-length axis")]
    ZeroAxis(String),
    #[error("joint `{joint}` mimics `{driver}`, which does not exist")]
    UnknownMimicSource { joint: String, driver: String },
    #[error("joint `{joint}` mimics `{driver}`, which has no degree of freedom")]
    MimicSourceNotActuated { joint: String, driver: String },
    #[error("fixed joint `{0}` cannot mimic another joint")]
    MimicOnFixedJoint(String),
    #[error("joint `{0}` is part of a mimic cycle")]
    MimicCycle(String),
    #[error("flange link `{0}` does not exist on the robot")]
    UnknownFlange(String),
    #[error("mount link `{0}` does not exist on the tool")]
    UnknownMount(String),
    #[error("TCP link `{0}` does not exist on the tool")]
    UnknownTcp(String),
    #[error(
        "the robot declares no flange frame to mount a tool on (catalog packages do); pass \
         `flange=` explicitly"
    )]
    NoFlangeDeclared,
    #[error(
        "mount link `{link}` is not the tool's root link (`{root}`); botrail welds the tool by \
         its root — re-root the tool model or mount it by `{root}`"
    )]
    MountNotRoot { link: String, root: String },
    #[error("`{0}` names both a robot and a tool {1}; pass a prefix to keep them apart")]
    NameCollision(String, &'static str),
}

/// Joint types supported by botrail. URDF `floating`/`planar` are rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointType {
    Revolute,
    Continuous,
    Prismatic,
    Fixed,
}

impl JointType {
    pub fn dof(self) -> usize {
        match self {
            JointType::Fixed => 0,
            _ => 1,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            JointType::Revolute => "revolute",
            JointType::Continuous => "continuous",
            JointType::Prismatic => "prismatic",
            JointType::Fixed => "fixed",
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct JointLimits {
    pub lower: f64,
    pub upper: f64,
    pub velocity: f64,
    pub effort: f64,
}

/// A joint driven by another joint through a fixed affine relation —
/// URDF `<mimic>`, USD `PhysxMimicJointAPI`. The classic case is a
/// two-finger gripper whose second finger mirrors the first.
///
/// A mimic joint has no degree of freedom of its own: its value is
/// `multiplier * q[source] + offset`, so it never appears in `q`.
#[derive(Debug, Clone, Copy)]
pub struct JointMimic {
    /// Index of the driving joint. [`RobotModel::from_parts`] flattens
    /// mimic chains, so this always names an actuated joint.
    pub source_joint: usize,
    pub multiplier: f64,
    pub offset: f64,
}

#[derive(Debug, Clone)]
pub struct Joint {
    pub name: String,
    pub joint_type: JointType,
    /// Transform from the parent link frame to the child link frame at q = 0.
    pub origin: Isometry3<f64>,
    pub axis: Unit<Vector3<f64>>,
    /// Position limits; `None` for fixed and continuous joints, and for
    /// revolute/prismatic joints whose URDF omitted `<limit>` (spec
    /// violation, but they exist — xurdf 0.6 surfaces the absence and
    /// botrail treats them as continuous-like rather than frozen at an
    /// all-zero limit). On a mimic joint these are informational: botrail
    /// computes the joint's value from its source and does not clamp it
    /// (a URDF that declares mimic limits without applying the multiplier
    /// is common enough that enforcing them would freeze real grippers).
    pub limits: Option<JointLimits>,
    pub parent_link: usize,
    pub child_link: usize,
    /// Index into the joint position vector `q`; `None` for fixed and
    /// mimic joints.
    pub q_index: Option<usize>,
    /// Set when this joint follows another one; see [`JointMimic`].
    pub mimic: Option<JointMimic>,
}

#[derive(Debug, Clone)]
pub enum Geometry {
    Box { size: Vector3<f64> },
    Cylinder { radius: f64, length: f64 },
    Sphere { radius: f64 },
    Mesh { path: PathBuf, scale: Vector3<f64> },
}

#[derive(Debug, Clone)]
pub struct Shape {
    /// Transform from the link frame to the shape frame.
    pub origin: Isometry3<f64>,
    pub geometry: Geometry,
    /// The colour the file gave this visual — URDF `<material><color
    /// rgba>` or USD `primvars:displayColor` — and `None` when it named
    /// none, which is what makes a viewer free to shade the link its own
    /// way. RGB only: alpha is dropped (botrail has no transparency), and
    /// the numbers pass through unconverted, the same as an obstacle's
    /// colour. Collision shapes never carry one; they are not drawn.
    pub color: Option<[f32; 3]>,
}

#[derive(Debug, Clone)]
pub struct Link {
    pub name: String,
    pub visuals: Vec<Shape>,
    pub collisions: Vec<Shape>,
    /// Joint connecting this link to its parent; `None` for the root link.
    pub parent_joint: Option<usize>,
}

/// Where a robot model came from — kept so projects can persist and
/// re-create the robot.
#[derive(Debug, Clone)]
pub enum RobotSource {
    /// URDF XML (xacro already expanded); embedded verbatim in projects.
    UrdfXml(String),
    /// A USD stage: file path plus the articulation root prim path.
    /// Referenced (not embedded) until asset bundling lands.
    Usd {
        path: PathBuf,
        articulation_root: String,
    },
    /// A model catalog package (`Robot.from_catalog`): the catalog id, the
    /// resolved dataset revision, and the fetched package's own source.
    Catalog {
        /// Full catalog id (e.g. `robotiq/2f/2f-85/r1`), as resolved.
        id: String,
        /// The dataset commit SHA the package was fetched at — a floating
        /// "newest" pins to a concrete revision here.
        revision: String,
        /// TCP link declared by the package manifest (`frames.tcp_default`),
        /// reapplied when the model is rebuilt from `inner`.
        tcp: Option<String>,
        /// Tool-mounting face declared by the manifest
        /// (`frames.flange_frame`), reapplied on rebuild.
        flange: Option<String>,
        /// Mounting face declared by the manifest (`frames.mount_frame`),
        /// reapplied on rebuild.
        mount: Option<String>,
        /// The fetched file's own source (URDF XML / USD reference), so
        /// projects rebuild without touching the network.
        inner: Box<RobotSource>,
    },
    /// A robot composed by [`RobotModel::attach_tool`]: both part sources
    /// plus the weld parameters, so the composite can be rebuilt.
    Composite {
        base: Box<RobotSource>,
        tool: Box<RobotSource>,
        /// Base-side link the tool is welded to.
        flange: String,
        /// Tool-side link (its root) welded to the flange, pre-prefix.
        mount: String,
        /// Flange-to-mount transform (e.g. a coupling's thickness).
        offset: Isometry3<f64>,
        /// Tool-side TCP link name as passed by the caller, pre-prefix.
        tcp: Option<String>,
        /// Prefix applied to every tool link/joint name in the composite.
        prefix: Option<String>,
    },
}

impl RobotSource {
    /// The USD stage this robot renders and replays from, when its geometry
    /// maps one-to-one onto a referenced stage: a [`RobotSource::Usd`]
    /// import, possibly wrapped by a catalog record. Composites and
    /// URDF-sourced robots return `None` — their animation bakes into
    /// per-link transforms instead.
    pub fn usd_stage(&self) -> Option<(&Path, &str)> {
        match self {
            RobotSource::Usd {
                path,
                articulation_root,
            } => Some((path, articulation_root)),
            RobotSource::Catalog { inner, .. } => inner.usd_stage(),
            RobotSource::UrdfXml(_) | RobotSource::Composite { .. } => None,
        }
    }
}

#[derive(Debug)]
pub struct RobotModel {
    pub name: String,
    pub links: Vec<Link>,
    pub joints: Vec<Joint>,
    pub root_link: usize,
    /// Joint indices ordered parent-before-child (tree traversal order).
    pub joint_order: Vec<usize>,
    /// Indices of the joints carrying a DOF, in `q`-vector order (base to
    /// tip). Fixed and mimic joints are excluded.
    pub actuated_joints: Vec<usize>,
    pub source: RobotSource,
    /// Explicitly declared TCP link ([`RobotModel::attach_tool`], catalog
    /// metadata). When set it overrides the deepest-leaf heuristic in
    /// [`RobotModel::default_tcp_link`].
    pub tcp_link: Option<usize>,
    /// Declared tool-mounting face on this robot (catalog
    /// `frames.flange_frame`, ISO 9409-1 on arms; a coupling's outward
    /// face). [`RobotModel::attach_tool`] uses it when `flange` is omitted.
    pub flange_link: Option<usize>,
    /// Declared mounting face when this model *is* a tool (catalog
    /// `frames.mount_frame`). [`RobotModel::attach_tool`] uses it when
    /// `mount` is omitted, falling back to the tool's root link.
    pub mount_link: Option<usize>,
}

impl RobotModel {
    pub fn from_urdf_file(path: impl AsRef<Path>) -> Result<Self, ModelError> {
        Self::from_urdf_file_with(path, &ModelOptions::default())
    }

    pub fn from_urdf_file_with(
        path: impl AsRef<Path>,
        options: &ModelOptions,
    ) -> Result<Self, ModelError> {
        let path = path.as_ref();
        let xml = std::fs::read_to_string(path).map_err(|e| ModelError::Parse(e.to_string()))?;
        Self::from_urdf_str_with(&xml, path.parent(), options)
    }

    /// Parses a URDF from a string. Relative mesh paths cannot be resolved in
    /// this mode; pass `base_dir` via [`RobotModel::from_urdf_str_with`] if needed.
    pub fn from_urdf_str(xml: &str) -> Result<Self, ModelError> {
        Self::from_urdf_str_with(xml, None, &ModelOptions::default())
    }

    pub fn from_urdf_str_with(
        xml: &str,
        base_dir: Option<&Path>,
        options: &ModelOptions,
    ) -> Result<Self, ModelError> {
        let robot =
            xurdf::parse_urdf_from_string(xml).map_err(|e| ModelError::Parse(e.to_string()))?;
        Self::build(robot, base_dir, options, xml.to_string())
    }

    /// Expands a Xacro file and parses the resulting URDF.
    pub fn from_xacro_file(path: impl AsRef<Path>) -> Result<Self, ModelError> {
        Self::from_xacro_file_with(path, &ModelOptions::default())
    }

    pub fn from_xacro_file_with(
        path: impl AsRef<Path>,
        options: &ModelOptions,
    ) -> Result<Self, ModelError> {
        let path = path.as_ref();
        let xml =
            xurdf::parse_xacro_from_file(path).map_err(|e| ModelError::Parse(e.to_string()))?;
        Self::from_urdf_str_with(&xml, path.parent(), options)
    }

    pub fn dof(&self) -> usize {
        self.actuated_joints.len()
    }

    pub fn link_index(&self, name: &str) -> Option<usize> {
        self.links.iter().position(|l| l.name == name)
    }

    pub fn joint_index(&self, name: &str) -> Option<usize> {
        self.joints.iter().position(|j| j.name == name)
    }

    pub fn actuated_joint_names(&self) -> Vec<&str> {
        self.actuated_joints
            .iter()
            .map(|&i| self.joints[i].name.as_str())
            .collect()
    }

    /// Position limits per actuated joint (`None` for continuous joints).
    pub fn actuated_joint_limits(&self) -> Vec<Option<(f64, f64)>> {
        self.actuated_joints
            .iter()
            .map(|&i| self.joints[i].limits.map(|l| (l.lower, l.upper)))
            .collect()
    }

    /// Default end-effector link: the explicitly declared TCP when one is
    /// set (tool attachment, catalog metadata), otherwise the leaf reached
    /// through the longest joint chain from the root (ties broken by
    /// traversal order) — a heuristic for tools/TCP frames, which URDF does
    /// not mark explicitly.
    pub fn default_tcp_link(&self) -> usize {
        if let Some(tcp) = self.tcp_link {
            return tcp;
        }
        let mut depth = vec![0usize; self.links.len()];
        let mut best = self.root_link;
        for &ji in &self.joint_order {
            let joint = &self.joints[ji];
            depth[joint.child_link] = depth[joint.parent_link] + 1;
            if depth[joint.child_link] > depth[best] {
                best = joint.child_link;
            }
        }
        best
    }

    /// The deepest link every tool leaf hangs off: the wrist a gripper is
    /// bolted to, or simply the last link of a single chain. It is the
    /// deepest frame a *pose* fully describes — joints below it (a gripper's)
    /// move parts of the tool relative to each other, so servoing a link
    /// below the mount lets a solver spend the grip as if it were a DOF.
    pub fn tool_mount_link(&self) -> usize {
        let leaves: Vec<usize> = (0..self.links.len())
            .filter(|link| !self.joints.iter().any(|j| j.parent_link == *link))
            .collect();
        let Some((first, rest)) = leaves.split_first() else {
            return self.root_link;
        };
        // Deepest link on one leaf's chain that every other leaf hangs off.
        let mut link = Some(*first);
        while let Some(current) = link {
            if rest
                .iter()
                .all(|&leaf| self.is_ancestor_or_self(current, leaf))
            {
                return current;
            }
            link = self.links[current]
                .parent_joint
                .map(|ji| self.joints[ji].parent_link);
        }
        self.root_link
    }

    /// Actuated joints that move `link` — its ancestors in the chain, i.e.
    /// exactly the DOFs a solver can use to place it. A mimic ancestor
    /// contributes the joint that drives *it*, since that is the DOF the
    /// solver would actually have to spend.
    pub fn driving_joints(&self, link: usize) -> Vec<usize> {
        let mut out = Vec::new();
        let mut current = link;
        while let Some(ji) = self.links[current].parent_joint {
            let driver = match self.joints[ji].mimic {
                Some(m) => Some(m.source_joint),
                None if self.joints[ji].q_index.is_some() => Some(ji),
                None => None,
            };
            if let Some(driver) = driver {
                if !out.contains(&driver) {
                    out.push(driver);
                }
            }
            current = self.joints[ji].parent_link;
        }
        out
    }

    /// Value of joint `joint` in configuration `q`: its own DOF value, the
    /// mimic relation for a mimic joint, and 0 for a fixed one.
    pub fn joint_value(&self, joint: usize, q: &[f64]) -> f64 {
        let joint = &self.joints[joint];
        if let Some(m) = joint.mimic {
            let source = self.joints[m.source_joint]
                .q_index
                .expect("mimic sources are actuated after from_parts");
            return m.multiplier * q[source] + m.offset;
        }
        match joint.q_index {
            Some(qi) => q[qi],
            None => 0.0,
        }
    }

    /// [`RobotModel::joint_value`] for every joint, indexed like `joints`.
    pub fn joint_values(&self, q: &[f64]) -> Vec<f64> {
        (0..self.joints.len())
            .map(|ji| self.joint_value(ji, q))
            .collect()
    }

    /// Joints that follow another joint, in model order.
    pub fn mimic_joints(&self) -> Vec<usize> {
        (0..self.joints.len())
            .filter(|&ji| self.joints[ji].mimic.is_some())
            .collect()
    }

    fn is_ancestor_or_self(&self, ancestor: usize, link: usize) -> bool {
        let mut current = Some(link);
        while let Some(index) = current {
            if index == ancestor {
                return true;
            }
            current = self.links[index]
                .parent_joint
                .map(|ji| self.joints[ji].parent_link);
        }
        false
    }

    /// Per-DOF sampling bounds for planning: the position limits, with
    /// continuous joints bounded to one full turn.
    pub fn sampling_bounds(&self) -> (Vec<f64>, Vec<f64>) {
        let mut lower = Vec::with_capacity(self.dof());
        let mut upper = Vec::with_capacity(self.dof());
        for limits in self.actuated_joint_limits() {
            match limits {
                Some((lo, hi)) => {
                    lower.push(lo);
                    upper.push(hi);
                }
                None => {
                    lower.push(-std::f64::consts::PI);
                    upper.push(std::f64::consts::PI);
                }
            }
        }
        (lower, upper)
    }

    /// Neutral configuration: zero, clamped into each joint's position limits.
    pub fn neutral_positions(&self) -> Vec<f64> {
        self.actuated_joints
            .iter()
            .map(|&i| match self.joints[i].limits {
                Some(l) => 0.0f64.clamp(l.lower, l.upper),
                None => 0.0,
            })
            .collect()
    }

    fn build(
        robot: xurdf::Robot,
        base_dir: Option<&Path>,
        options: &ModelOptions,
        urdf_source: String,
    ) -> Result<Self, ModelError> {
        let link_index: HashMap<&str, usize> = robot
            .links
            .iter()
            .enumerate()
            .map(|(i, l)| (l.name.as_str(), i))
            .collect();

        let links: Vec<Link> = robot
            .links
            .iter()
            .map(|l| Link {
                name: l.name.clone(),
                visuals: l
                    .visuals
                    .iter()
                    .map(|v| {
                        let mut shape = convert_shape(&v.origin, &v.geometry, base_dir, options);
                        shape.color = v.material.as_ref().and_then(material_color);
                        shape
                    })
                    .collect(),
                collisions: l
                    .collisions
                    .iter()
                    .map(|c| convert_shape(&c.origin, &c.geometry, base_dir, options))
                    .collect(),
                parent_joint: None,
            })
            .collect();

        let joint_index: HashMap<&str, usize> = robot
            .joints
            .iter()
            .enumerate()
            .map(|(i, j)| (j.name.as_str(), i))
            .collect();

        let mut joints = Vec::with_capacity(robot.joints.len());
        for (ji, j) in robot.joints.iter().enumerate() {
            let joint_type = match j.joint_type.as_str() {
                "revolute" => JointType::Revolute,
                "continuous" => JointType::Continuous,
                "prismatic" => JointType::Prismatic,
                "fixed" => JointType::Fixed,
                other => {
                    return Err(ModelError::UnsupportedJointType {
                        name: j.name.clone(),
                        joint_type: other.to_string(),
                    })
                }
            };
            let parent_link = *link_index
                .get(j.parent.as_str())
                .ok_or_else(|| ModelError::UnknownLink(j.parent.clone()))?;
            let child_link = *link_index
                .get(j.child.as_str())
                .ok_or_else(|| ModelError::UnknownLink(j.child.clone()))?;
            let axis = if joint_type == JointType::Fixed {
                Unit::new_unchecked(Vector3::z())
            } else {
                Unit::try_new(j.axis, 1e-9).ok_or_else(|| ModelError::ZeroAxis(j.name.clone()))?
            };
            // An absent `<limit>` (xurdf 0.6: `limit: Option`) maps to
            // `None`, i.e. continuous-like — before 0.6 it arrived as an
            // all-zero limit and silently froze the joint at 0, presenting
            // as "IK mysteriously never converges".
            let limits = match joint_type {
                JointType::Revolute | JointType::Prismatic => {
                    j.limit.as_ref().map(|l| JointLimits {
                        lower: l.lower,
                        upper: l.upper,
                        velocity: l.velocity,
                        effort: l.effort,
                    })
                }
                _ => None,
            };
            let mimic = j
                .mimic
                .as_ref()
                .map(|m| {
                    joint_index
                        .get(m.joint.as_str())
                        .map(|&source_joint| JointMimic {
                            source_joint,
                            multiplier: m.multiplier,
                            offset: m.offset,
                        })
                        .ok_or_else(|| ModelError::UnknownMimicSource {
                            joint: j.name.clone(),
                            driver: m.joint.clone(),
                        })
                })
                .transpose()?;
            let _ = ji;
            joints.push(Joint {
                name: j.name.clone(),
                joint_type,
                origin: pose_to_isometry(&j.origin),
                axis,
                limits,
                parent_link,
                child_link,
                q_index: None,
                mimic,
            });
        }

        Self::from_parts(robot.name, links, joints, RobotSource::UrdfXml(urdf_source))
    }

    /// Builds a model from converted parts, computing the tree invariants:
    /// per-link parent joints, root detection, breadth-first joint order,
    /// q-index assignment, mimic resolution, and loop rejection. Callers may
    /// leave `Joint::q_index` and `Link::parent_joint` unset — both are
    /// assigned here. This is the entry point for non-URDF importers.
    pub fn from_parts(
        name: String,
        mut links: Vec<Link>,
        mut joints: Vec<Joint>,
        source: RobotSource,
    ) -> Result<Self, ModelError> {
        for link in &mut links {
            link.parent_joint = None;
        }
        for joint in &mut joints {
            joint.q_index = None;
        }
        flatten_mimics(&mut joints)?;
        for (ji, j) in joints.iter().enumerate() {
            if links[j.child_link].parent_joint.is_some() {
                return Err(ModelError::MultipleParents(
                    links[j.child_link].name.clone(),
                ));
            }
            links[j.child_link].parent_joint = Some(ji);
        }

        let roots: Vec<usize> = links
            .iter()
            .enumerate()
            .filter(|(_, l)| l.parent_joint.is_none())
            .map(|(i, _)| i)
            .collect();
        let root_link = match roots.as_slice() {
            [] => return Err(ModelError::NoRoot),
            [root] => *root,
            many => {
                return Err(ModelError::MultipleRoots(
                    many.iter().map(|&i| links[i].name.clone()).collect(),
                ))
            }
        };

        // Breadth-first traversal from the root; assigns q indices base-to-tip.
        let mut children: Vec<Vec<usize>> = vec![Vec::new(); links.len()];
        for (ji, j) in joints.iter().enumerate() {
            children[j.parent_link].push(ji);
        }
        let mut joint_order = Vec::with_capacity(joints.len());
        let mut actuated_joints = Vec::new();
        let mut queue = std::collections::VecDeque::from([root_link]);
        while let Some(li) = queue.pop_front() {
            for &ji in &children[li] {
                joint_order.push(ji);
                if joints[ji].joint_type.dof() > 0 && joints[ji].mimic.is_none() {
                    joints[ji].q_index = Some(actuated_joints.len());
                    actuated_joints.push(ji);
                }
                queue.push_back(joints[ji].child_link);
            }
        }
        if joint_order.len() != joints.len() {
            let unreachable = (0..joints.len())
                .find(|ji| !joint_order.contains(ji))
                .expect("some joint must be unreachable");
            return Err(ModelError::Loop(joints[unreachable].name.clone()));
        }

        Ok(RobotModel {
            name,
            links,
            joints,
            root_link,
            joint_order,
            actuated_joints,
            source,
            tcp_link: None,
            flange_link: None,
            mount_link: None,
        })
    }

    /// Welds a tool (end-effector) onto this robot's `flange` link and
    /// returns the composite: one kinematic tree whose DOF vector is this
    /// robot's joints followed by the tool's (mimic joints keep following
    /// their source). `offset` is the flange-to-mount transform — a
    /// coupling's thickness, for instance.
    ///
    /// `flange` defaults to the robot's declared [`flange_link`]
    /// (catalog manifests declare one) and `mount` to the tool's declared
    /// [`mount_link`], falling back to its root — so two catalog parts
    /// attach with no arguments at all. `mount` must resolve to the tool's
    /// root link (a URDF tree cannot hang off a mid-chain link without
    /// re-rooting). `tcp` optionally names a tool link to become the
    /// composite's TCP; when omitted, a TCP declared on the tool model
    /// carries over. `prefix` is prepended to every tool link/joint name —
    /// required when the two models share a name.
    ///
    /// On the composite, a flange the *tool* declares carries over
    /// (remapped): mounting a coupling leaves its outward face as the new
    /// flange, so a whole coupling-then-gripper stack chains without
    /// naming a single frame.
    ///
    /// [`flange_link`]: RobotModel::flange_link
    /// [`mount_link`]: RobotModel::mount_link
    pub fn attach_tool(
        &self,
        tool: &RobotModel,
        flange: Option<&str>,
        mount: Option<&str>,
        offset: Isometry3<f64>,
        tcp: Option<&str>,
        prefix: Option<&str>,
    ) -> Result<RobotModel, ModelError> {
        let flange = match flange {
            Some(name) => name.to_string(),
            None => match self.flange_link {
                Some(i) => self.links[i].name.clone(),
                None => return Err(ModelError::NoFlangeDeclared),
            },
        };
        let mount = match mount {
            Some(name) => name.to_string(),
            None => {
                let i = tool.mount_link.unwrap_or(tool.root_link);
                tool.links[i].name.clone()
            }
        };
        let (flange, mount) = (flange.as_str(), mount.as_str());
        let flange_index = self
            .link_index(flange)
            .ok_or_else(|| ModelError::UnknownFlange(flange.to_string()))?;
        let mount_index = tool
            .link_index(mount)
            .ok_or_else(|| ModelError::UnknownMount(mount.to_string()))?;
        if mount_index != tool.root_link {
            return Err(ModelError::MountNotRoot {
                link: mount.to_string(),
                root: tool.links[tool.root_link].name.clone(),
            });
        }
        let tool_tcp = match tcp {
            Some(name) => Some(
                tool.link_index(name)
                    .ok_or_else(|| ModelError::UnknownTcp(name.to_string()))?,
            ),
            // A TCP the tool declares (catalog metadata) survives mounting.
            None => tool.tcp_link,
        };
        let rename = |name: &str| match prefix {
            Some(p) => format!("{p}{name}"),
            None => name.to_string(),
        };

        let link_offset = self.links.len();
        let mut links = self.links.clone();
        for link in &tool.links {
            let name = rename(&link.name);
            if self.link_index(&name).is_some() {
                return Err(ModelError::NameCollision(name, "link"));
            }
            links.push(Link {
                name,
                ..link.clone()
            });
        }

        let joint_offset = self.joints.len();
        let mut joints = self.joints.clone();
        for joint in &tool.joints {
            let name = rename(&joint.name);
            if self.joint_index(&name).is_some() {
                return Err(ModelError::NameCollision(name, "joint"));
            }
            joints.push(Joint {
                name,
                parent_link: joint.parent_link + link_offset,
                child_link: joint.child_link + link_offset,
                // Tool-internal index; `q_index` is reassigned by from_parts.
                mimic: joint.mimic.map(|m| JointMimic {
                    source_joint: m.source_joint + joint_offset,
                    ..m
                }),
                ..joint.clone()
            });
        }
        joints.push(Joint {
            name: format!("{flange}_to_{}", rename(mount)),
            joint_type: JointType::Fixed,
            origin: offset,
            axis: Unit::new_unchecked(Vector3::z()),
            limits: None,
            parent_link: flange_index,
            child_link: link_offset + tool.root_link,
            q_index: None,
            mimic: None,
        });

        let source = RobotSource::Composite {
            base: Box::new(self.source.clone()),
            tool: Box::new(tool.source.clone()),
            flange: flange.to_string(),
            mount: mount.to_string(),
            offset,
            tcp: tcp.map(str::to_string),
            prefix: prefix.map(str::to_string),
        };
        let mut model = Self::from_parts(self.name.clone(), links, joints, source)?;
        // The base's own TCP (if any) sits behind the tool now and does not
        // carry over; without a tool TCP the deepest-leaf heuristic applies,
        // which lands inside the tool.
        model.tcp_link = tool_tcp.map(|tcp| tcp + link_offset);
        // A flange the tool declares (a coupling's outward face) becomes the
        // composite's flange; the base's is occupied. The base's mount face
        // stays what it was, so pre-assembled tool stacks remain mountable.
        model.flange_link = tool.flange_link.map(|i| i + link_offset);
        model.mount_link = self.mount_link;
        Ok(model)
    }
}

/// Rewrites every mimic relation to point straight at a joint that carries
/// a DOF, so a mimic joint's value is one multiply-add away from `q`. A
/// joint mimicking a mimic joint composes: following `a = m1*b + o1` with
/// `b = m2*c + o2` gives `a = m1*m2*c + m1*o2 + o1`. URDF does not forbid
/// such chains, and nothing downstream should have to walk them.
fn flatten_mimics(joints: &mut [Joint]) -> Result<(), ModelError> {
    let count = joints.len();
    for ji in 0..count {
        let Some(mut mimic) = joints[ji].mimic else {
            continue;
        };
        if joints[ji].joint_type.dof() == 0 {
            return Err(ModelError::MimicOnFixedJoint(joints[ji].name.clone()));
        }
        // Bounded by the joint count: a longer walk can only be a cycle.
        let mut resolved = false;
        for _ in 0..count {
            if mimic.source_joint >= count {
                return Err(ModelError::UnknownMimicSource {
                    joint: joints[ji].name.clone(),
                    driver: format!("joint index {}", mimic.source_joint),
                });
            }
            if mimic.source_joint == ji {
                return Err(ModelError::MimicCycle(joints[ji].name.clone()));
            }
            let source = &joints[mimic.source_joint];
            if source.joint_type.dof() == 0 {
                return Err(ModelError::MimicSourceNotActuated {
                    joint: joints[ji].name.clone(),
                    driver: source.name.clone(),
                });
            }
            let Some(next) = source.mimic else {
                resolved = true;
                break;
            };
            mimic = JointMimic {
                source_joint: next.source_joint,
                multiplier: mimic.multiplier * next.multiplier,
                offset: mimic.multiplier * next.offset + mimic.offset,
            };
        }
        if !resolved {
            return Err(ModelError::MimicCycle(joints[ji].name.clone()));
        }
        joints[ji].mimic = Some(mimic);
    }
    Ok(())
}

/// Converts a URDF pose (xyz + fixed-axis rpy) to an isometry.
/// nalgebra's `from_euler_angles(r, p, y)` builds Rz(y)·Ry(p)·Rx(r), which
/// matches the URDF convention.
pub fn pose_to_isometry(pose: &xurdf::Pose) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::from(pose.xyz),
        UnitQuaternion::from_euler_angles(pose.rpy.x, pose.rpy.y, pose.rpy.z),
    )
}

fn convert_shape(
    origin: &xurdf::Pose,
    geometry: &xurdf::Geometry,
    base_dir: Option<&Path>,
    options: &ModelOptions,
) -> Shape {
    let geometry = match geometry {
        xurdf::Geometry::Box { size } => Geometry::Box { size: *size },
        xurdf::Geometry::Cylinder { radius, length } => Geometry::Cylinder {
            radius: *radius,
            length: *length,
        },
        xurdf::Geometry::Sphere { radius } => Geometry::Sphere { radius: *radius },
        xurdf::Geometry::Mesh { filename, scale } => Geometry::Mesh {
            path: mesh_path::resolve(filename, base_dir, options),
            scale: scale.unwrap_or_else(|| Vector3::new(1.0, 1.0, 1.0)),
        },
    };
    Shape {
        origin: pose_to_isometry(origin),
        geometry,
        color: None,
    }
}

/// A URDF material's RGB, dropping alpha. A material that only names a
/// texture (no `<color>`) has nothing to give, and the visual stays
/// uncoloured rather than turning black.
fn material_color(material: &xurdf::Material) -> Option<[f32; 3]> {
    material
        .color
        .map(|c| [c[0] as f32, c[1] as f32, c[2] as f32])
}

#[cfg(test)]
mod tests {
    use super::*;

    const TWO_LINK: &str = r#"
    <robot name="two_link">
      <link name="base_link"/>
      <link name="link1">
        <visual>
          <origin xyz="0.5 0 0" rpy="0 1.5707963267948966 0"/>
          <geometry><cylinder radius="0.05" length="1.0"/></geometry>
        </visual>
      </link>
      <link name="tool"/>
      <joint name="shoulder" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
      </joint>
      <joint name="tool_joint" type="fixed">
        <parent link="link1"/>
        <child link="tool"/>
        <origin xyz="1 0 0"/>
      </joint>
    </robot>
    "#;

    /// Materials by reference and inline, plus a visual with none and a
    /// texture-only material — the four shapes a URDF states colour in.
    const PAINTED: &str = r#"
    <robot name="painted">
      <material name="red"><color rgba="0.9 0.1 0.05 1.0"/></material>
      <material name="skin"><texture filename="skin.png"/></material>
      <link name="base">
        <visual>
          <geometry><box size="0.2 0.2 0.2"/></geometry>
          <material name="red"/>
        </visual>
        <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
      </link>
      <link name="arm">
        <visual>
          <geometry><sphere radius="0.1"/></geometry>
          <material name="inline"><color rgba="0.0 0.25 1.0 0.5"/></material>
        </visual>
        <visual>
          <geometry><sphere radius="0.05"/></geometry>
        </visual>
        <visual>
          <geometry><sphere radius="0.02"/></geometry>
          <material name="skin"/>
        </visual>
      </link>
      <joint name="j" type="revolute">
        <parent link="base"/><child link="arm"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
    </robot>
    "#;

    #[test]
    fn visual_materials_carry_their_colour() {
        let model = RobotModel::from_urdf_str(PAINTED).unwrap();
        let base = &model.links[model.link_index("base").unwrap()];
        assert_eq!(base.visuals[0].color, Some([0.9, 0.1, 0.05]));
        // Collision shapes are never drawn, so they never carry one.
        assert_eq!(base.collisions[0].color, None);
        let arm = &model.links[model.link_index("arm").unwrap()];
        // Inline material, alpha dropped: botrail has no transparency.
        assert_eq!(arm.visuals[0].color, Some([0.0, 0.25, 1.0]));
        // No material at all, and a texture-only one: both leave the
        // viewer free to shade the link, rather than turning it black.
        assert_eq!(arm.visuals[1].color, None);
        assert_eq!(arm.visuals[2].color, None);
    }

    /// A wrist carrying a two-finger gripper: the tool mount is the wrist
    /// the fingers hang off, not the deepest leaf.
    const GRIPPER: &str = r#"
    <robot name="arm">
      <link name="base"/>
      <link name="wrist"/>
      <link name="left"/>
      <link name="right"/>
      <joint name="elbow" type="revolute">
        <parent link="base"/><child link="wrist"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
      <joint name="finger_left" type="prismatic">
        <parent link="wrist"/><child link="left"/>
        <axis xyz="0 1 0"/>
        <limit lower="0" upper="0.04" effort="1" velocity="1"/>
      </joint>
      <joint name="finger_right" type="prismatic">
        <parent link="wrist"/><child link="right"/>
        <axis xyz="0 -1 0"/>
        <limit lower="0" upper="0.04" effort="1" velocity="1"/>
      </joint>
    </robot>
    "#;

    /// The same gripper wired as a real one: the right finger mirrors the
    /// left through `<mimic>`, so the pair costs a single DOF.
    const MIMIC_GRIPPER: &str = r#"
    <robot name="arm">
      <link name="base"/>
      <link name="wrist"/>
      <link name="left"/>
      <link name="right"/>
      <joint name="elbow" type="revolute">
        <parent link="base"/><child link="wrist"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
      <joint name="finger_left" type="prismatic">
        <parent link="wrist"/><child link="left"/>
        <axis xyz="0 1 0"/>
        <limit lower="0" upper="0.04" effort="1" velocity="1"/>
      </joint>
      <joint name="finger_right" type="prismatic">
        <parent link="wrist"/><child link="right"/>
        <axis xyz="0 1 0"/>
        <limit lower="-0.04" upper="0" effort="1" velocity="1"/>
        <mimic joint="finger_left" multiplier="-1" offset="0.005"/>
      </joint>
    </robot>
    "#;

    /// A revolute joint without `<limit>` violates the URDF spec but exists
    /// in the wild. Through xurdf ≤0.5 it arrived as an all-zero limit and
    /// the joint froze at 0 without a word; with 0.6's `Option` it must map
    /// to `limits: None` — continuous-like, movable, ±π sampling bounds.
    #[test]
    fn revolute_without_limit_is_unlimited_not_frozen() {
        let urdf = r#"
        <robot name="r">
          <link name="base"/><link name="l1"/><link name="l2"/>
          <joint name="no_limit" type="revolute">
            <parent link="base"/><child link="l1"/>
            <axis xyz="0 0 1"/>
          </joint>
          <joint name="sparse_limit" type="revolute">
            <parent link="l1"/><child link="l2"/>
            <origin xyz="1 0 0"/><axis xyz="0 0 1"/>
            <limit lower="-1" upper="1"/>
          </joint>
        </robot>"#;
        let model = RobotModel::from_urdf_str(urdf).unwrap();
        assert_eq!(model.dof(), 2);

        let bare = model.joint_index("no_limit").unwrap();
        assert!(model.joints[bare].limits.is_none());
        let (lower, upper) = model.sampling_bounds();
        assert_eq!(lower[0], -std::f64::consts::PI);
        assert_eq!(upper[0], std::f64::consts::PI);

        // `<limit>` without effort/velocity now parses (xurdf 0.6 defaults
        // them to 0); the position bounds survive and the zero velocity is
        // the downstream fallback's job (`traj_limits`).
        let sparse = model.joint_index("sparse_limit").unwrap();
        let l = model.joints[sparse].limits.unwrap();
        assert_eq!((l.lower, l.upper), (-1.0, 1.0));
        assert_eq!((l.velocity, l.effort), (0.0, 0.0));
    }

    #[test]
    fn mimic_joints_cost_no_dof_and_follow_their_source() {
        let model = RobotModel::from_urdf_str(MIMIC_GRIPPER).unwrap();
        assert_eq!(model.dof(), 2);
        assert_eq!(model.actuated_joint_names(), vec!["elbow", "finger_left"]);

        let right = model.joint_index("finger_right").unwrap();
        assert_eq!(model.joints[right].q_index, None);
        let mimic = model.joints[right].mimic.unwrap();
        assert_eq!(
            mimic.source_joint,
            model.joint_index("finger_left").unwrap()
        );
        assert_eq!((mimic.multiplier, mimic.offset), (-1.0, 0.005));

        // q = [elbow, finger_left]; the right finger follows the formula.
        let q = [0.2, 0.03];
        assert_eq!(model.joint_value(right, &q), -0.03 + 0.005);
        assert_eq!(
            model.joint_value(model.joint_index("elbow").unwrap(), &q),
            0.2
        );
        assert_eq!(model.mimic_joints(), vec![right]);
        // Fixed joints report 0 rather than reading `q`.
        let chain = RobotModel::from_urdf_str(TWO_LINK).unwrap();
        let tool = chain.joint_index("tool_joint").unwrap();
        assert_eq!(chain.joint_value(tool, &[0.5]), 0.0);
    }

    #[test]
    fn mimic_chains_are_flattened_to_a_dof() {
        // c mimics b mimics a: c = 3*(2*qa + 0.5) + 0.25.
        let urdf = r#"
        <robot name="chain">
          <link name="l0"/><link name="l1"/><link name="l2"/><link name="l3"/>
          <joint name="a" type="revolute">
            <parent link="l0"/><child link="l1"/>
            <axis xyz="0 0 1"/>
            <limit lower="-3" upper="3" effort="1" velocity="1"/>
          </joint>
          <joint name="b" type="revolute">
            <parent link="l1"/><child link="l2"/>
            <axis xyz="0 0 1"/>
            <limit lower="-3" upper="3" effort="1" velocity="1"/>
            <mimic joint="a" multiplier="2" offset="0.5"/>
          </joint>
          <joint name="c" type="revolute">
            <parent link="l2"/><child link="l3"/>
            <axis xyz="0 0 1"/>
            <limit lower="-3" upper="3" effort="1" velocity="1"/>
            <mimic joint="b" multiplier="3" offset="0.25"/>
          </joint>
        </robot>"#;
        let model = RobotModel::from_urdf_str(urdf).unwrap();
        assert_eq!(model.dof(), 1);
        let a = model.joint_index("a").unwrap();
        let c = model.joints[model.joint_index("c").unwrap()].mimic.unwrap();
        assert_eq!(c.source_joint, a);
        assert_eq!((c.multiplier, c.offset), (6.0, 1.75));
        assert_eq!(model.joint_values(&[1.0]), vec![1.0, 2.5, 7.75]);
    }

    #[test]
    fn rejects_broken_mimic_declarations() {
        let joint = |extra: &str| {
            format!(
                r#"
                <robot name="r">
                  <link name="a"/><link name="b"/><link name="c"/>
                  <joint name="j1" type="fixed">
                    <parent link="a"/><child link="b"/>
                  </joint>
                  <joint name="j2" type="revolute">
                    <parent link="b"/><child link="c"/>
                    <axis xyz="0 0 1"/>
                    <limit lower="-1" upper="1" effort="1" velocity="1"/>
                    {extra}
                  </joint>
                </robot>"#
            )
        };
        let err = RobotModel::from_urdf_str(&joint(r#"<mimic joint="nope"/>"#)).unwrap_err();
        assert!(
            matches!(err, ModelError::UnknownMimicSource { .. }),
            "{err}"
        );
        // A fixed joint has no value to follow.
        let err = RobotModel::from_urdf_str(&joint(r#"<mimic joint="j1"/>"#)).unwrap_err();
        assert!(
            matches!(err, ModelError::MimicSourceNotActuated { .. }),
            "{err}"
        );
        let err = RobotModel::from_urdf_str(&joint(r#"<mimic joint="j2"/>"#)).unwrap_err();
        assert!(matches!(err, ModelError::MimicCycle(_)), "{err}");
    }

    #[test]
    fn mimic_ancestors_report_the_joint_that_drives_them() {
        let model = RobotModel::from_urdf_str(MIMIC_GRIPPER).unwrap();
        let names = |link: &str| {
            let mut out: Vec<String> = model
                .driving_joints(model.link_index(link).unwrap())
                .into_iter()
                .map(|ji| model.joints[ji].name.clone())
                .collect();
            out.sort();
            out
        };
        // Moving the right fingertip means moving `finger_left`.
        assert_eq!(names("right"), vec!["elbow", "finger_left"]);
        assert_eq!(names("left"), vec!["elbow", "finger_left"]);
    }

    #[test]
    fn tool_mount_is_the_link_the_tool_hangs_off() {
        let model = RobotModel::from_urdf_str(GRIPPER).unwrap();
        let mount = model.tool_mount_link();
        assert_eq!(model.links[mount].name, "wrist");
        // The deepest-leaf heuristic picks a fingertip instead, which is
        // exactly the difference that matters for pose servoing.
        let tcp = &model.links[model.default_tcp_link()].name;
        assert!(tcp == "left" || tcp == "right", "{tcp}");
        // A single chain has no branch: mount == deepest link.
        let chain = RobotModel::from_urdf_str(TWO_LINK).unwrap();
        assert_eq!(chain.tool_mount_link(), chain.default_tcp_link());
    }

    #[test]
    fn driving_joints_are_the_ancestors() {
        let model = RobotModel::from_urdf_str(GRIPPER).unwrap();
        let names = |link: &str| {
            let mut out: Vec<String> = model
                .driving_joints(model.link_index(link).unwrap())
                .into_iter()
                .map(|ji| model.joints[ji].name.clone())
                .collect();
            out.sort();
            out
        };
        assert_eq!(names("wrist"), vec!["elbow"]);
        assert_eq!(names("left"), vec!["elbow", "finger_left"]);
        assert!(names("base").is_empty());
    }

    /// A one-DOF mimic gripper whose root is its mounting plate, with an
    /// explicit TCP frame between the fingers.
    const TOOL: &str = r#"
    <robot name="gripper">
      <link name="mount_plate"/>
      <link name="finger_l"/>
      <link name="finger_r"/>
      <link name="grasp_center"/>
      <joint name="drive" type="prismatic">
        <parent link="mount_plate"/><child link="finger_l"/>
        <axis xyz="0 1 0"/>
        <limit lower="0" upper="0.04" effort="1" velocity="1"/>
      </joint>
      <joint name="follow" type="prismatic">
        <parent link="mount_plate"/><child link="finger_r"/>
        <axis xyz="0 1 0"/>
        <limit lower="-0.04" upper="0" effort="1" velocity="1"/>
        <mimic joint="drive" multiplier="-1"/>
      </joint>
      <joint name="tcp_joint" type="fixed">
        <parent link="mount_plate"/><child link="grasp_center"/>
        <origin xyz="0 0 0.12"/>
      </joint>
    </robot>
    "#;

    #[test]
    fn attach_tool_composes_the_kinematics() {
        let arm = RobotModel::from_urdf_str(TWO_LINK).unwrap();
        let tool = RobotModel::from_urdf_str(TOOL).unwrap();
        let offset = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 0.0139),
            UnitQuaternion::identity(),
        );
        let combined = arm
            .attach_tool(
                &tool,
                Some("tool"),
                Some("mount_plate"),
                offset,
                Some("grasp_center"),
                None,
            )
            .unwrap();

        // Arm DOF + the gripper's single actuated joint.
        assert_eq!(combined.dof(), 2);
        assert_eq!(combined.actuated_joint_names(), vec!["shoulder", "drive"]);
        // The mimic relation survives with its source remapped.
        let follow = combined.joint_index("follow").unwrap();
        let mimic = combined.joints[follow].mimic.unwrap();
        assert_eq!(mimic.source_joint, combined.joint_index("drive").unwrap());
        // The declared TCP wins over the deepest-leaf heuristic.
        assert_eq!(
            combined.links[combined.default_tcp_link()].name,
            "grasp_center"
        );
        // The weld carries the offset.
        let weld = combined.joint_index("tool_to_mount_plate").unwrap();
        assert_eq!(combined.joints[weld].joint_type, JointType::Fixed);
        assert!((combined.joints[weld].origin.translation.z - 0.0139).abs() < 1e-12);
        // The originals are untouched.
        assert_eq!(arm.dof(), 1);
        assert_eq!(tool.dof(), 1);
        // The composite records how it was built.
        assert!(matches!(combined.source, RobotSource::Composite { .. }));
    }

    #[test]
    fn attach_tool_without_tcp_inherits_the_tool_declaration() {
        let arm = RobotModel::from_urdf_str(TWO_LINK).unwrap();
        let mut tool = RobotModel::from_urdf_str(TOOL).unwrap();
        tool.tcp_link = Some(tool.link_index("grasp_center").unwrap());
        let combined = arm
            .attach_tool(
                &tool,
                Some("tool"),
                Some("mount_plate"),
                Isometry3::identity(),
                None,
                None,
            )
            .unwrap();
        assert_eq!(
            combined.links[combined.default_tcp_link()].name,
            "grasp_center"
        );
    }

    #[test]
    fn attach_tool_prefix_disambiguates_collisions() {
        let arm = RobotModel::from_urdf_str(TWO_LINK).unwrap();
        // A "tool" that reuses the arm's link names wholesale.
        let clone = RobotModel::from_urdf_str(TWO_LINK).unwrap();
        let err = arm
            .attach_tool(
                &clone,
                Some("tool"),
                Some("base_link"),
                Isometry3::identity(),
                None,
                None,
            )
            .unwrap_err();
        assert!(matches!(err, ModelError::NameCollision(_, "link")), "{err}");

        let combined = arm
            .attach_tool(
                &clone,
                Some("tool"),
                Some("base_link"),
                Isometry3::identity(),
                None,
                Some("t2_"),
            )
            .unwrap();
        assert_eq!(combined.dof(), 2);
        assert!(combined.link_index("t2_base_link").is_some());
        assert!(combined.joint_index("t2_shoulder").is_some());
    }

    #[test]
    fn attach_tool_defaults_chain_through_declared_frames() {
        let mut arm = RobotModel::from_urdf_str(TWO_LINK).unwrap();
        arm.flange_link = arm.link_index("tool");
        // A coupling declares both faces: its robot-side mount and an
        // onward flange for whatever screws on next.
        let coupling_urdf = r#"
        <robot name="coupling">
          <link name="c_mount"/><link name="c_flange"/>
          <joint name="c_body" type="fixed">
            <parent link="c_mount"/><child link="c_flange"/>
            <origin xyz="0 0 0.02"/>
          </joint>
        </robot>"#;
        let mut coupling = RobotModel::from_urdf_str(coupling_urdf).unwrap();
        coupling.mount_link = coupling.link_index("c_mount");
        coupling.flange_link = coupling.link_index("c_flange");
        let mut tool = RobotModel::from_urdf_str(TOOL).unwrap();
        tool.mount_link = tool.link_index("mount_plate");
        tool.tcp_link = tool.link_index("grasp_center");

        // Not a single frame named: the whole stack chains off declarations.
        let id = Isometry3::identity();
        let stack = arm
            .attach_tool(&coupling, None, None, id, None, None)
            .unwrap()
            .attach_tool(&tool, None, None, id, None, None)
            .unwrap();
        assert_eq!(stack.dof(), 2);
        assert_eq!(stack.links[stack.default_tcp_link()].name, "grasp_center");
        // The weld joints prove which faces were picked at each step.
        assert!(stack.joint_index("tool_to_c_mount").is_some());
        assert!(stack.joint_index("c_flange_to_mount_plate").is_some());
        // The gripper declares no onward flange, so the stack ends here.
        assert_eq!(stack.flange_link, None);

        // A tool with no declared mount falls back to its root link.
        let plain = RobotModel::from_urdf_str(TOOL).unwrap();
        let combined = arm.attach_tool(&plain, None, None, id, None, None).unwrap();
        assert!(combined.joint_index("tool_to_mount_plate").is_some());
    }

    #[test]
    fn attach_tool_rejects_bad_frames() {
        let arm = RobotModel::from_urdf_str(TWO_LINK).unwrap();
        let tool = RobotModel::from_urdf_str(TOOL).unwrap();
        let id = Isometry3::identity();
        assert!(matches!(
            arm.attach_tool(&tool, Some("nope"), Some("mount_plate"), id, None, None),
            Err(ModelError::UnknownFlange(_))
        ));
        assert!(matches!(
            arm.attach_tool(&tool, Some("tool"), Some("nope"), id, None, None),
            Err(ModelError::UnknownMount(_))
        ));
        assert!(matches!(
            arm.attach_tool(&tool, Some("tool"), Some("finger_l"), id, None, None),
            Err(ModelError::MountNotRoot { .. })
        ));
        assert!(matches!(
            arm.attach_tool(
                &tool,
                Some("tool"),
                Some("mount_plate"),
                id,
                Some("nope"),
                None
            ),
            Err(ModelError::UnknownTcp(_))
        ));
        // No declared flange and no explicit one: refuse rather than guess.
        assert!(matches!(
            arm.attach_tool(&tool, None, None, id, None, None),
            Err(ModelError::NoFlangeDeclared)
        ));
    }

    #[test]
    fn builds_tree_and_q_mapping() {
        let model = RobotModel::from_urdf_str(TWO_LINK).unwrap();
        assert_eq!(model.name, "two_link");
        assert_eq!(model.links.len(), 3);
        assert_eq!(model.joints.len(), 2);
        assert_eq!(model.dof(), 1);
        assert_eq!(model.root_link, model.link_index("base_link").unwrap());
        assert_eq!(model.actuated_joint_names(), vec!["shoulder"]);

        let shoulder = &model.joints[model.joint_index("shoulder").unwrap()];
        assert_eq!(shoulder.joint_type, JointType::Revolute);
        assert_eq!(shoulder.q_index, Some(0));
        let limits = shoulder.limits.unwrap();
        assert_eq!((limits.lower, limits.upper), (-1.57, 1.57));

        let tool_joint = &model.joints[model.joint_index("tool_joint").unwrap()];
        assert_eq!(tool_joint.joint_type, JointType::Fixed);
        assert_eq!(tool_joint.q_index, None);
        assert!((tool_joint.origin.translation.x - 1.0).abs() < 1e-12);
    }

    #[test]
    fn neutral_positions_respect_limits() {
        let urdf = r#"
        <robot name="r">
          <link name="a"/><link name="b"/>
          <joint name="j" type="revolute">
            <parent link="a"/><child link="b"/>
            <axis xyz="0 0 1"/>
            <limit lower="0.5" upper="1.0" effort="1" velocity="1"/>
          </joint>
        </robot>"#;
        let model = RobotModel::from_urdf_str(urdf).unwrap();
        assert_eq!(model.neutral_positions(), vec![0.5]);
    }

    #[test]
    fn rejects_unsupported_joint_type() {
        let urdf = r#"
        <robot name="r">
          <link name="a"/><link name="b"/>
          <joint name="j" type="floating">
            <parent link="a"/><child link="b"/>
          </joint>
        </robot>"#;
        let err = RobotModel::from_urdf_str(urdf).unwrap_err();
        assert!(matches!(err, ModelError::UnsupportedJointType { .. }));
    }

    #[test]
    fn rejects_multiple_roots() {
        let urdf = r#"
        <robot name="r">
          <link name="a"/><link name="b"/>
        </robot>"#;
        let err = RobotModel::from_urdf_str(urdf).unwrap_err();
        assert!(matches!(err, ModelError::MultipleRoots(_)));
    }

    #[test]
    fn urdf_rpy_convention_matches_fixed_axis_rotations() {
        // rpy="0 0 pi/2" must rotate x into y.
        let pose = xurdf::Pose {
            xyz: nalgebra::zero(),
            rpy: Vector3::new(0.0, 0.0, std::f64::consts::FRAC_PI_2),
        };
        let iso = pose_to_isometry(&pose);
        let v = iso * nalgebra::Point3::new(1.0, 0.0, 0.0);
        assert!((v.y - 1.0).abs() < 1e-12 && v.x.abs() < 1e-12);
    }
}

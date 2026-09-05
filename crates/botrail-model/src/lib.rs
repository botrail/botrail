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
    #[error("unknown group `{0}`")]
    UnknownGroup(String),
    #[error("group `{group}`: link `{link}` does not exist on the robot")]
    UnknownGroupLink { group: String, link: String },
    #[error("group `{group}`: joint `{joint}` does not exist on the robot")]
    UnknownGroupJoint { group: String, joint: String },
    #[error(
        "group `{group}`: joint `{joint}` carries no degree of freedom (fixed, or a mimic \
         follower — name the joint that drives it)"
    )]
    GroupJointNotActuated { group: String, joint: String },
    #[error("the robot has several groups ({0:?}); pass `group=` to say which arm")]
    AmbiguousGroup(Vec<String>),
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
/// URDF `<mimic>`, USD `NewtonMimicAPI` / `PhysxMimicJointAPI`. The classic case is a
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
        /// Grasp-surface frames declared by the manifest
        /// (`frames.grasp_frames`), reapplied on rebuild.
        grasp: Vec<String>,
        /// The arms a dual-arm package declares (`frames.arms[]`), each
        /// applied as a planning group on load and again on rebuild.
        arms: Vec<CatalogArm>,
        /// What the package *is* commercially (maker, product name,
        /// category, headline specs) — the manifest's identity fields, kept
        /// so a bill of materials can name the machine without re-reading
        /// the catalog. Empty when the manifest carried none.
        meta: CatalogMeta,
        /// The fetched file's own source (URDF XML / USD reference), so
        /// projects rebuild without touching the network.
        inner: Box<RobotSource>,
    },
    /// A robot composed by [`RobotModel::attach_tool`] or
    /// [`RobotModel::mount`]: both part sources plus the weld parameters,
    /// so the composite can be rebuilt.
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
        /// What the welded part is to the composite: a tool on an arm, or
        /// an arm on a body.
        role: MountRole,
        /// The group addressed (a tool's arm) or created (a mounted arm),
        /// as the caller named it.
        group: Option<String>,
    },
}

/// What a part welded by [`RobotModel::mount`] is to the composite.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MountRole {
    /// An end-effector: the arm it hangs off gains its TCP
    /// ([`RobotModel::attach_tool`]).
    #[default]
    Tool,
    /// A manipulator of its own — an arm bolted to a body: its joints
    /// become a [`Group`] of the composite, with its own TCP and flange.
    Arm,
}

/// A planning group: the joints one planned motion drives and the link it
/// drives to — an arm of a dual-arm robot, or the whole of a single-arm
/// one. See [`RobotModel::groups`].
#[derive(Debug, Clone, PartialEq)]
pub struct Group {
    pub name: String,
    /// q indices, ascending. The tree is traversed breadth-first, so a
    /// dual-arm's two arms interleave: a group is a set, not a range.
    pub joints: Vec<usize>,
    /// The link a motion of this group places (its TCP).
    pub tip: usize,
    /// This arm's own tool-mounting face, when declared.
    pub flange: Option<usize>,
    /// The link the group's chain hangs off: a dual-arm's shoulder mount,
    /// the root for a single arm.
    pub base: usize,
    /// Read off the tree ([`RobotModel::derive_groups`]) rather than
    /// declared.
    pub derived: bool,
}

/// A declared group, by names: what a catalog manifest or
/// [`RobotModel::define_group`] states. Names survive the rebuilds
/// (a tool attached, a project reloaded) that renumber q indices.
#[derive(Debug, Clone, PartialEq)]
pub struct GroupSpec {
    pub name: String,
    pub tip: String,
    pub joints: Vec<String>,
    pub flange: Option<String>,
}

/// A chain of at least this many actuated joints above a fork is an arm
/// carrying a hand, and the fork's branches are its fingers: they stay in
/// the arm's group. A shorter stem (a waist, a fixed body) is a torso
/// carrying arms, and the branches become groups of their own.
const ARM_STEM_MIN: usize = 4;

/// A branch under a fork counts as a limb (an arm, a leg) — and so makes
/// the fork a body rather than a hand — when it carries at least this many
/// joints of its own. One-joint branches are fingers.
const LIMB_MIN_JOINTS: usize = 2;

/// The identity a catalog manifest declares for a package — who makes
/// it, what it is called, which category it files under, and the numeric
/// headline specs (`mass_kg`, `reach_mm`, `payload_kg`, ...). Carried on
/// [`RobotSource::Catalog`] purely so downstream consumers (a bill of
/// materials) can describe the machine; nothing kinematic reads it.
/// One arm of a dual-arm catalog package (`frames.arms[]`): what
/// [`RobotModel::define_group`] is called with on load.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CatalogArm {
    pub name: String,
    /// The arm's TCP link (`tcp_default`).
    pub tip: String,
    /// The arm's actuated joints, as the manifest lists them; empty lets
    /// the tip's chain decide.
    pub joints: Vec<String>,
    /// The arm's tool-mounting face (`flange_frame`).
    pub flange: Option<String>,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct CatalogMeta {
    /// `manufacturer.name` in the manifest.
    pub manufacturer: Option<String>,
    /// The manifest's `name` — the product as its maker calls it.
    pub product: Option<String>,
    /// The manifest's `category` (`manipulator`, `gripper.parallel`, ...).
    pub category: Option<String>,
    /// Numeric `specs.*` entries, in manifest order (non-numeric specs
    /// such as `controller` lists are dropped).
    pub specs: Vec<(String, f64)>,
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

#[derive(Debug, Clone)]
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
    /// Declared grasp-surface frames (catalog `frames.grasp_frames`):
    /// fingertips of a hand, the pads of a gripper — where the product
    /// says it holds things. Advisory metadata for authoring; empty when
    /// the source declares none.
    pub grasp_links: Vec<usize>,
    /// Declared planning groups (catalog `frames.arms`,
    /// [`RobotModel::define_group`], a mounted arm). Empty means the groups
    /// are derived from the tree — see [`RobotModel::groups`].
    pub declared_groups: Vec<GroupSpec>,
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
        // XacroOptions keeps a private resolver, so it is built and then set.
        let mut xacro = xurdf::XacroOptions::default();
        xacro.args = options.xacro_args.clone();
        xacro.package_paths = options.package_paths.clone();
        let xml = xurdf::parse_xacro_from_file_with_options(path, xacro)
            .map_err(|e| ModelError::Parse(e.to_string()))?;
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
    ///
    /// A leaf with no moving joint above it is base furniture, not a tool
    /// leaf — ROS-convention stub frames (`base`, `world`) hang off the
    /// root by fixed joints, and counting them would drag the "common
    /// ancestor of all leaves" down to the base link. They are ignored
    /// (unless the whole model is fixed, where they are all there is).
    pub fn tool_mount_link(&self) -> usize {
        self.subtree_tool_mount(self.root_link)
    }

    /// [`RobotModel::tool_mount_link`] for the subtree under `root`: the
    /// deepest link every moving leaf below `root` hangs off, with
    /// "moving" judged by the joints between `root` and the leaf. `root`
    /// itself when the subtree has no leaf, or no common link below it.
    pub fn subtree_tool_mount(&self, root: usize) -> usize {
        let moving = |mut link: usize| loop {
            if link == root {
                return false;
            }
            match self.links[link].parent_joint {
                Some(ji) => {
                    let j = &self.joints[ji];
                    if j.q_index.is_some() || j.mimic.is_some() {
                        return true;
                    }
                    link = j.parent_link;
                }
                None => return false,
            }
        };
        let is_leaf = |link: usize| !self.joints.iter().any(|j| j.parent_link == link);
        let in_subtree = |link: usize| self.is_ancestor_or_self(root, link);
        let mut leaves: Vec<usize> = (0..self.links.len())
            .filter(|&link| is_leaf(link) && in_subtree(link) && moving(link))
            .collect();
        if leaves.is_empty() {
            leaves = (0..self.links.len())
                .filter(|&link| is_leaf(link) && in_subtree(link))
                .collect();
        }
        let Some((first, rest)) = leaves.split_first() else {
            return root;
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
            if current == root {
                break;
            }
            link = self.links[current]
                .parent_joint
                .map(|ji| self.joints[ji].parent_link);
        }
        root
    }

    // ------------------------------------------------------------- groups

    /// The robot's planning groups — the unit a planned motion drives and
    /// a TCP belongs to. The declared ones when any are (a catalog
    /// manifest's arms, [`RobotModel::define_group`], an arm bolted on by
    /// [`RobotModel::mount`]); otherwise read off the tree by
    /// [`RobotModel::derive_groups`]. Never empty.
    pub fn groups(&self) -> Vec<Group> {
        if self.declared_groups.is_empty() {
            return self.derive_groups();
        }
        self.declared_groups
            .iter()
            .map(|spec| {
                self.resolve_group_spec(spec)
                    .expect("declared groups are validated when they are declared")
            })
            .collect()
    }

    /// Index into [`RobotModel::groups`] of the group called `name`.
    pub fn group_index(&self, name: &str) -> Option<usize> {
        self.groups().iter().position(|g| g.name == name)
    }

    /// Whether the groups come from the tree rather than a declaration.
    pub fn groups_are_derived(&self) -> bool {
        self.declared_groups.is_empty()
    }

    /// The groups the tree implies, without any declaration. A robot with
    /// no fork is one group of every actuated joint, tipped at the default
    /// TCP — exactly the single-arm behaviour. Where a short stem (a fixed
    /// body, a waist of fewer than `ARM_STEM_MIN` joints) forks into
    /// several limbs (branches of `LIMB_MIN_JOINTS` joints or more — a
    /// fork into one-joint fingers is a hand), each branch is a group of
    /// its own (an arm of a dual-arm body, a leg of a quadruped), tipped at the branch's
    /// own tool mount rather than at a fingertip, and the stem is a group
    /// by itself. A long stem is an arm and its branches are fingers,
    /// which stay in the arm's group. Names come from the joints' common
    /// name prefix (`openarm_left`, `FL`), a lone tip link's name failing
    /// that, uniquified.
    pub fn derive_groups(&self) -> Vec<Group> {
        let mut out = Vec::new();
        self.derive_into(self.root_link, true, &mut out);
        if out.is_empty() {
            // A rigid body, or nothing but mimic followers: one empty
            // group, so "the group" always has an answer.
            out.push(Group {
                name: "arm".to_string(),
                joints: Vec::new(),
                tip: self.default_tcp_link(),
                flange: self.flange_link,
                base: self.root_link,
                derived: true,
            });
        }
        out
    }

    fn derive_into(&self, root: usize, top: bool, out: &mut Vec<Group>) {
        let joints = self.subtree_actuated(root);
        if joints.is_empty() {
            return;
        }
        let mount = self.subtree_tool_mount(root);
        // The stem: actuated joints on the path from the subtree root down
        // to the mount, base to tip.
        let mut stem = Vec::new();
        let mut link = mount;
        while link != root {
            let ji = self.links[link]
                .parent_joint
                .expect("the mount lies below the subtree root");
            if let Some(qi) = self.joints[ji].q_index {
                stem.push(qi);
            }
            link = self.joints[ji].parent_link;
        }
        stem.reverse();
        let branches: Vec<usize> = self
            .joint_order
            .iter()
            .map(|&ji| &self.joints[ji])
            .filter(|j| j.parent_link == mount)
            .map(|j| j.child_link)
            .filter(|&child| !self.subtree_actuated(child).is_empty())
            .collect();
        // A branch is a limb — an arm, a leg — when it carries at least
        // two joints of its own; one-joint branches are fingers, and a
        // fork into fingers is a hand, whatever sits above it.
        let limbs = branches
            .iter()
            .filter(|&&child| self.subtree_actuated(child).len() >= LIMB_MIN_JOINTS)
            .count();
        if limbs >= 2 && stem.len() < ARM_STEM_MIN {
            if !stem.is_empty() {
                let name = self.derived_name(&stem, mount, out);
                out.push(Group {
                    name,
                    joints: stem,
                    tip: mount,
                    flange: None,
                    base: root,
                    derived: true,
                });
            }
            for child in branches {
                self.derive_into(child, false, out);
            }
            return;
        }
        let (name, tip, flange) = if top {
            ("arm".to_string(), self.default_tcp_link(), self.flange_link)
        } else {
            (self.derived_name(&joints, mount, out), mount, None)
        };
        out.push(Group {
            name,
            joints,
            tip,
            flange,
            base: root,
            derived: true,
        });
    }

    /// Actuated joints (q indices, ascending) whose child link lies under
    /// `root`.
    fn subtree_actuated(&self, root: usize) -> Vec<usize> {
        self.actuated_joints
            .iter()
            .enumerate()
            .filter(|(_, &ji)| self.is_ancestor_or_self(root, self.joints[ji].child_link))
            .map(|(qi, _)| qi)
            .collect()
    }

    /// A name for a derived group: the joints' common `_`-token prefix,
    /// the tip link's name when they share none, kept unique among
    /// `taken` (first by one more token of the first joint, then by a
    /// counter).
    fn derived_name(&self, joints: &[usize], tip: usize, taken: &[Group]) -> String {
        let names: Vec<Vec<&str>> = joints
            .iter()
            .map(|&qi| {
                self.joints[self.actuated_joints[qi]]
                    .name
                    .split('_')
                    .filter(|t| !t.is_empty())
                    .collect()
            })
            .collect();
        let mut common = 0;
        if let Some(first) = names.first() {
            while common < first.len()
                && names.iter().all(|n| n.get(common) == Some(&first[common]))
            {
                common += 1;
            }
        }
        let base = match names.first() {
            Some(first) if common > 0 => first[..common].join("_"),
            _ => self.links[tip].name.clone(),
        };
        let used = |name: &str| taken.iter().any(|g| g.name == name);
        if !used(&base) {
            return base;
        }
        if let Some(first) = names.first() {
            if let Some(next) = first.get(common) {
                let longer = format!("{base}_{next}");
                if !used(&longer) {
                    return longer;
                }
            }
        }
        (2..)
            .map(|n| format!("{base}_{n}"))
            .find(|candidate| !used(candidate))
            .expect("some counter is free")
    }

    /// The most specific group whose chain `link` hangs off: among the
    /// groups whose base is an ancestor of `link` (or the link itself),
    /// the one based deepest. `None` when no group contains it or two are
    /// based equally deep (overlapping declarations).
    pub fn group_for_link(&self, link: usize) -> Option<usize> {
        let depth = |mut l: usize| {
            let mut d = 0;
            while let Some(ji) = self.links[l].parent_joint {
                d += 1;
                l = self.joints[ji].parent_link;
            }
            d
        };
        let groups = self.groups();
        let mut best: Option<(usize, usize)> = None;
        let mut tied = false;
        for (gi, g) in groups.iter().enumerate() {
            if !self.is_ancestor_or_self(g.base, link) {
                continue;
            }
            let d = depth(g.base);
            match best {
                Some((_, bd)) if d < bd => {}
                Some((_, bd)) if d == bd => tied = true,
                _ => {
                    best = Some((gi, d));
                    tied = false;
                }
            }
        }
        if tied {
            return None;
        }
        best.map(|(gi, _)| gi)
    }

    /// [`RobotModel::tool_mount_link`] of one group: the deepest link every
    /// moving leaf under the group's base hangs off — the wrist a hand is
    /// bolted to, per arm.
    pub fn group_tool_mount(&self, group: &Group) -> usize {
        self.subtree_tool_mount(group.base)
    }

    /// Declares (or redeclares, by name) a planning group and returns the
    /// model carrying it; the input is not modified. `tip` is the link a
    /// motion of the group places. `joints` names the actuated joints it
    /// drives; omitted, they are the joints on `tip`'s chain minus those of
    /// any existing group tipped above it (a waist declared as a group of
    /// its own stays out of the arm). `flange` names this arm's
    /// tool-mounting face, which [`RobotModel::attach_tool`] with
    /// `group=` uses.
    ///
    /// The first declaration replaces the derived groups outright — from
    /// then on the declaration is the truth, and arms not declared are not
    /// groups.
    pub fn define_group(
        &self,
        name: &str,
        tip: &str,
        joints: Option<&[&str]>,
        flange: Option<&str>,
    ) -> Result<RobotModel, ModelError> {
        let tip_index = self
            .link_index(tip)
            .ok_or_else(|| ModelError::UnknownGroupLink {
                group: name.to_string(),
                link: tip.to_string(),
            })?;
        let joints: Vec<String> = match joints {
            Some(list) => list.iter().map(|s| s.to_string()).collect(),
            None => {
                let above: Vec<usize> = self
                    .groups()
                    .iter()
                    .filter(|g| g.name != name)
                    .filter(|g| g.tip != tip_index && self.is_ancestor_or_self(g.tip, tip_index))
                    .flat_map(|g| g.joints.clone())
                    .collect();
                let mut chain = self.driving_joints(tip_index);
                chain.reverse();
                chain
                    .into_iter()
                    .filter(|&ji| {
                        self.joints[ji]
                            .q_index
                            .is_some_and(|qi| !above.contains(&qi))
                    })
                    .map(|ji| self.joints[ji].name.clone())
                    .collect()
            }
        };
        let spec = GroupSpec {
            name: name.to_string(),
            tip: tip.to_string(),
            joints,
            flange: flange.map(str::to_string),
        };
        let mut model = self.clone();
        let mut specs = self.declared_groups.clone();
        match specs.iter().position(|s| s.name == name) {
            Some(i) => specs[i] = spec,
            None => specs.push(spec),
        }
        model.declared_groups = specs;
        model.validate_groups()?;
        Ok(model)
    }

    /// Checks every declared group resolves on this tree.
    pub fn validate_groups(&self) -> Result<(), ModelError> {
        for spec in &self.declared_groups {
            self.resolve_group_spec(spec)?;
        }
        Ok(())
    }

    fn resolve_group_spec(&self, spec: &GroupSpec) -> Result<Group, ModelError> {
        let tip = self
            .link_index(&spec.tip)
            .ok_or_else(|| ModelError::UnknownGroupLink {
                group: spec.name.clone(),
                link: spec.tip.clone(),
            })?;
        let flange = spec
            .flange
            .as_deref()
            .map(|f| {
                self.link_index(f)
                    .ok_or_else(|| ModelError::UnknownGroupLink {
                        group: spec.name.clone(),
                        link: f.to_string(),
                    })
            })
            .transpose()?;
        let mut joints = Vec::with_capacity(spec.joints.len());
        for joint in &spec.joints {
            let ji = self
                .joint_index(joint)
                .ok_or_else(|| ModelError::UnknownGroupJoint {
                    group: spec.name.clone(),
                    joint: joint.clone(),
                })?;
            let qi = self.joints[ji]
                .q_index
                .ok_or_else(|| ModelError::GroupJointNotActuated {
                    group: spec.name.clone(),
                    joint: joint.clone(),
                })?;
            if !joints.contains(&qi) {
                joints.push(qi);
            }
        }
        joints.sort_unstable();
        let base = match joints.first() {
            Some(&qi) => self.joints[self.actuated_joints[qi]].parent_link,
            None => self.root_link,
        };
        Ok(Group {
            name: spec.name.clone(),
            joints,
            tip,
            flange,
            base,
            derived: false,
        })
    }

    /// The groups as declarations: the declared ones, or the derived ones
    /// written down by name (empty groups dropped — a bare body has no arm
    /// to keep). What a group-addressing composition starts from.
    fn group_specs(&self) -> Vec<GroupSpec> {
        if !self.declared_groups.is_empty() {
            return self.declared_groups.clone();
        }
        self.derive_groups()
            .into_iter()
            .filter(|g| !g.joints.is_empty())
            .map(|g| GroupSpec {
                name: g.name,
                tip: self.links[g.tip].name.clone(),
                joints: g
                    .joints
                    .iter()
                    .map(|&qi| self.joints[self.actuated_joints[qi]].name.clone())
                    .collect(),
                flange: g.flange.map(|i| self.links[i].name.clone()),
            })
            .collect()
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
            grasp_links: Vec::new(),
            declared_groups: Vec::new(),
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
    /// `group` names the arm the tool goes on: its declared flange is the
    /// default `flange`, and it is that arm whose TCP becomes the tool's.
    /// A single-arm robot needs none; a robot with several arms needs
    /// either `group` or a `flange` that lies on one of them.
    ///
    /// [`flange_link`]: RobotModel::flange_link
    /// [`mount_link`]: RobotModel::mount_link
    #[allow(clippy::too_many_arguments)]
    pub fn attach_tool(
        &self,
        tool: &RobotModel,
        flange: Option<&str>,
        mount: Option<&str>,
        offset: Isometry3<f64>,
        tcp: Option<&str>,
        prefix: Option<&str>,
        group: Option<&str>,
    ) -> Result<RobotModel, ModelError> {
        let groups = self.groups();
        // The arm addressed: named, the sole declared one, or the one the
        // named flange lies on. A sole *derived* group is the whole robot
        // and keeps the model-level TCP/flange bookkeeping.
        let target: Option<usize> = match group {
            Some(name) => Some(
                self.group_index(name)
                    .ok_or_else(|| ModelError::UnknownGroup(name.to_string()))?,
            ),
            None if groups.len() == 1 => (!groups[0].derived).then_some(0),
            None => match flange.and_then(|f| self.link_index(f)) {
                Some(link) => Some(self.group_for_link(link).ok_or_else(|| {
                    ModelError::AmbiguousGroup(groups.iter().map(|g| g.name.clone()).collect())
                })?),
                None => {
                    return Err(ModelError::AmbiguousGroup(
                        groups.iter().map(|g| g.name.clone()).collect(),
                    ))
                }
            },
        };
        let flange = match flange {
            Some(name) => name.to_string(),
            None => match target.and_then(|g| groups[g].flange).or(self.flange_link) {
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
        let source = RobotSource::Composite {
            base: Box::new(self.source.clone()),
            tool: Box::new(tool.source.clone()),
            flange: flange.to_string(),
            mount: mount.to_string(),
            offset,
            tcp: tcp.map(str::to_string),
            prefix: prefix.map(str::to_string),
            role: MountRole::Tool,
            group: group.map(str::to_string),
        };
        let (mut model, link_offset) =
            self.weld(tool, flange_index, mount, offset, prefix, source)?;
        // The base's own TCP (if any) sits behind the tool now and does not
        // carry over; without a tool TCP the deepest-leaf heuristic applies,
        // which lands inside the tool.
        model.tcp_link = tool_tcp.map(|tcp| tcp + link_offset);
        // A flange the tool declares (a coupling's outward face) becomes the
        // composite's flange; the base's is occupied. The base's mount face
        // stays what it was, so pre-assembled tool stacks remain mountable.
        model.flange_link = tool.flange_link.map(|i| i + link_offset);
        model.mount_link = self.mount_link;
        // Grasp surfaces accumulate: the base keeps whatever it declared
        // (usually nothing on an arm) and the tool's ride along remapped —
        // a hand bolted on still knows its fingertips.
        model.grasp_links = self
            .grasp_links
            .iter()
            .copied()
            .chain(tool.grasp_links.iter().map(|i| i + link_offset))
            .collect();
        // The addressed arm's declaration follows the tool: its TCP is the
        // tool's now, its flange the tool's onward face. Other arms are
        // untouched. With no arm addressed (a single-arm robot) the groups
        // stay derived and re-derive on the composite.
        if let Some(g) = target {
            let rename = |name: &str| match prefix {
                Some(p) => format!("{p}{name}"),
                None => name.to_string(),
            };
            let mut specs = self.group_specs();
            let spec = specs
                .iter_mut()
                .find(|s| s.name == groups[g].name)
                .expect("the addressed group is among the specs");
            spec.tip =
                rename(&tool.links[tool_tcp.unwrap_or_else(|| tool.default_tcp_link())].name);
            spec.flange = tool.flange_link.map(|i| rename(&tool.links[i].name));
            model.declared_groups = specs;
            model.validate_groups()?;
        }
        Ok(model)
    }

    /// Welds `part` onto this robot's link `at` as a manipulator of its
    /// own (`role` [`MountRole::Arm`]) — an arm bolted to a dual-arm body —
    /// or as a tool (`role` [`MountRole::Tool`], the same as
    /// [`RobotModel::attach_tool`] by `at`). The part's root goes on `at`
    /// at `offset`; `prefix` namespaces its link and joint names.
    ///
    /// A mounted arm becomes a group of the composite: `group` names it
    /// (the prefix without its trailing `_`, else the part's name, when
    /// omitted), its TCP and flange are the part's, and the part's own
    /// groups — when it has several — become `<group>_<name>`. The body's
    /// TCP and flange bookkeeping is left alone: with several arms the
    /// groups are what a TCP belongs to.
    #[allow(clippy::too_many_arguments)]
    pub fn mount(
        &self,
        part: &RobotModel,
        at: &str,
        offset: Isometry3<f64>,
        prefix: Option<&str>,
        role: MountRole,
        group: Option<&str>,
    ) -> Result<RobotModel, ModelError> {
        if role == MountRole::Tool {
            return self.attach_tool(part, Some(at), None, offset, None, prefix, group);
        }
        let at_index = self
            .link_index(at)
            .ok_or_else(|| ModelError::UnknownFlange(at.to_string()))?;
        let mount = part.links[part.root_link].name.clone();
        let group_name = match group {
            Some(name) => name.to_string(),
            None => prefix
                .map(|p| p.trim_end_matches(['_', '-', '/']).to_string())
                .filter(|p| !p.is_empty())
                .unwrap_or_else(|| part.name.clone()),
        };
        let source = RobotSource::Composite {
            base: Box::new(self.source.clone()),
            tool: Box::new(part.source.clone()),
            flange: at.to_string(),
            mount: mount.clone(),
            offset,
            tcp: None,
            prefix: prefix.map(str::to_string),
            role: MountRole::Arm,
            group: Some(group_name.clone()),
        };
        let (mut model, link_offset) = self.weld(part, at_index, &mount, offset, prefix, source)?;
        model.tcp_link = self.tcp_link;
        model.flange_link = self.flange_link;
        model.mount_link = self.mount_link;
        model.grasp_links = self
            .grasp_links
            .iter()
            .copied()
            .chain(part.grasp_links.iter().map(|i| i + link_offset))
            .collect();
        let rename = |name: &str| match prefix {
            Some(p) => format!("{p}{name}"),
            None => name.to_string(),
        };
        let mut specs = self.group_specs();
        let part_specs = part.group_specs();
        let several = part_specs.len() > 1;
        for spec in part_specs {
            let name = if several {
                format!("{group_name}_{}", spec.name)
            } else {
                group_name.clone()
            };
            if specs.iter().any(|s| s.name == name) {
                return Err(ModelError::NameCollision(name, "group"));
            }
            specs.push(GroupSpec {
                name,
                tip: rename(&spec.tip),
                joints: spec.joints.iter().map(|j| rename(j)).collect(),
                flange: spec.flange.as_deref().map(rename),
            });
        }
        model.declared_groups = specs;
        model.validate_groups()?;
        Ok(model)
    }

    /// Two arms on one body, as one robot with the groups `left` and
    /// `right` (link and joint names prefixed `left_` / `right_`). `body`
    /// is the torso the arms bolt to — omitted, a bare frame named after
    /// the left arm — and `left_mount` / `right_mount` name the body links
    /// the arm roots go on (the body's root by default) at `left_at` /
    /// `right_at`. Tools then attach per arm:
    /// `attach_tool(gripper, …, group = "left")`.
    pub fn dual_arm(
        body: Option<&RobotModel>,
        left: &RobotModel,
        right: &RobotModel,
        left_mount: Option<&str>,
        left_at: Isometry3<f64>,
        right_mount: Option<&str>,
        right_at: Isometry3<f64>,
    ) -> Result<RobotModel, ModelError> {
        let body = match body {
            Some(body) => body.clone(),
            None => RobotModel::from_urdf_str(&format!(
                "<robot name=\"{}_dual\"><link name=\"body\"/></robot>",
                left.name
            ))?,
        };
        let root = body.links[body.root_link].name.clone();
        body.mount(
            left,
            left_mount.unwrap_or(&root),
            left_at,
            Some("left_"),
            MountRole::Arm,
            Some("left"),
        )?
        .mount(
            right,
            right_mount.unwrap_or(&root),
            right_at,
            Some("right_"),
            MountRole::Arm,
            Some("right"),
        )
    }

    /// The weld itself: this robot's links and joints, then `part`'s
    /// (renamed by `prefix`, indices offset), joined by a fixed joint from
    /// `flange_index` to the part's root at `offset`. Returns the rebuilt
    /// composite (q indices reassigned by [`RobotModel::from_parts`]) and
    /// the part's link index offset.
    fn weld(
        &self,
        part: &RobotModel,
        flange_index: usize,
        mount: &str,
        offset: Isometry3<f64>,
        prefix: Option<&str>,
        source: RobotSource,
    ) -> Result<(RobotModel, usize), ModelError> {
        let rename = |name: &str| match prefix {
            Some(p) => format!("{p}{name}"),
            None => name.to_string(),
        };
        let flange = &self.links[flange_index].name;

        let link_offset = self.links.len();
        let mut links = self.links.clone();
        for link in &part.links {
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
        for joint in &part.joints {
            let name = rename(&joint.name);
            if self.joint_index(&name).is_some() {
                return Err(ModelError::NameCollision(name, "joint"));
            }
            joints.push(Joint {
                name,
                parent_link: joint.parent_link + link_offset,
                child_link: joint.child_link + link_offset,
                // Part-internal index; `q_index` is reassigned by from_parts.
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
            child_link: link_offset + part.root_link,
            q_index: None,
            mimic: None,
        });

        let model = Self::from_parts(self.name.clone(), links, joints, source)?;
        Ok((model, link_offset))
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
    pub(crate) const GRIPPER: &str = r#"
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

    /// Grasp-surface frames declared on a tool (catalog `grasp_frames`)
    /// ride along the weld remapped — the composite still knows its
    /// fingertips.
    #[test]
    fn attach_tool_carries_grasp_links_along() {
        let arm = RobotModel::from_urdf_str(TWO_LINK).unwrap();
        let mut tool = RobotModel::from_urdf_str(TOOL).unwrap();
        tool.grasp_links = vec![
            tool.link_index("finger_l").unwrap(),
            tool.link_index("finger_r").unwrap(),
        ];
        let combined = arm
            .attach_tool(
                &tool,
                Some("tool"),
                Some("mount_plate"),
                Isometry3::identity(),
                None,
                None,
                None,
            )
            .unwrap();
        let names: Vec<&str> = combined
            .grasp_links
            .iter()
            .map(|&l| combined.links[l].name.as_str())
            .collect();
        assert_eq!(names, vec!["finger_l", "finger_r"]);
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
                None,
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
            .attach_tool(&coupling, None, None, id, None, None, None)
            .unwrap()
            .attach_tool(&tool, None, None, id, None, None, None)
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
        let combined = arm
            .attach_tool(&plain, None, None, id, None, None, None)
            .unwrap();
        assert!(combined.joint_index("tool_to_mount_plate").is_some());
    }

    #[test]
    fn attach_tool_rejects_bad_frames() {
        let arm = RobotModel::from_urdf_str(TWO_LINK).unwrap();
        let tool = RobotModel::from_urdf_str(TOOL).unwrap();
        let id = Isometry3::identity();
        assert!(matches!(
            arm.attach_tool(
                &tool,
                Some("nope"),
                Some("mount_plate"),
                id,
                None,
                None,
                None
            ),
            Err(ModelError::UnknownFlange(_))
        ));
        assert!(matches!(
            arm.attach_tool(&tool, Some("tool"), Some("nope"), id, None, None, None),
            Err(ModelError::UnknownMount(_))
        ));
        assert!(matches!(
            arm.attach_tool(&tool, Some("tool"), Some("finger_l"), id, None, None, None),
            Err(ModelError::MountNotRoot { .. })
        ));
        assert!(matches!(
            arm.attach_tool(
                &tool,
                Some("tool"),
                Some("mount_plate"),
                id,
                Some("nope"),
                None,
                None
            ),
            Err(ModelError::UnknownTcp(_))
        ));
        // No declared flange and no explicit one: refuse rather than guess.
        assert!(matches!(
            arm.attach_tool(&tool, None, None, id, None, None, None),
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

#[cfg(test)]
mod group_tests {
    use super::tests::GRIPPER;
    use super::*;

    const DUAL: &str = include_str!("../../../examples/assets/dual_arm_test.urdf");
    const ARM: &str = include_str!("../../../examples/assets/simple_arm.urdf");

    fn dual() -> RobotModel {
        RobotModel::from_urdf_str(DUAL).unwrap()
    }

    fn names(model: &RobotModel, g: &Group) -> Vec<String> {
        g.joints
            .iter()
            .map(|&qi| model.joints[model.actuated_joints[qi]].name.clone())
            .collect()
    }

    /// A single chain is one group of every joint, tipped at the default
    /// TCP — the single-arm behaviour, untouched.
    #[test]
    fn a_single_chain_is_one_whole_group() {
        let arm = RobotModel::from_urdf_str(ARM).unwrap();
        let groups = arm.groups();
        assert_eq!(groups.len(), 1);
        let g = &groups[0];
        assert_eq!(g.name, "arm");
        assert_eq!(g.joints, (0..arm.dof()).collect::<Vec<_>>());
        assert_eq!(g.tip, arm.default_tcp_link());
        assert_eq!(g.base, arm.root_link);
        assert!(g.derived);
        assert!(arm.groups_are_derived());
    }

    /// A body forking into two arms derives one group per arm, tipped at
    /// each arm's own tool mount (the hand), not at a fingertip, and
    /// named by the joints' common prefix.
    #[test]
    fn a_dual_arm_body_derives_one_group_per_arm() {
        let model = dual();
        assert_eq!(model.dof(), 8);
        let groups = model.groups();
        let by_name: Vec<&str> = groups.iter().map(|g| g.name.as_str()).collect();
        assert_eq!(by_name, vec!["left", "right"]);
        let left = &groups[0];
        assert_eq!(
            names(&model, left),
            vec!["left_shoulder", "left_elbow", "left_wrist", "left_finger"]
        );
        assert_eq!(model.links[left.tip].name, "left_hand");
        assert_eq!(model.links[left.base].name, "left_base");
        // The deepest-leaf heuristic lands on a fingertip; the group does not.
        let deepest = &model.links[model.default_tcp_link()].name;
        assert!(deepest.contains("finger"), "{deepest}");
        // The right arm's joints interleave with the left's in q order.
        let right = &groups[1];
        assert!(right
            .joints
            .iter()
            .any(|&qi| qi < *left.joints.last().unwrap()));
    }

    #[test]
    fn group_for_link_picks_the_most_specific_arm() {
        let model = dual();
        let hand = model.link_index("right_finger_a").unwrap();
        assert_eq!(model.group_for_link(hand), Some(1));
        let body = model.root_link;
        assert_eq!(model.group_for_link(body), None);
    }

    /// Declaring replaces the derived groups; the default joints are the
    /// tip's chain, minus any group tipped above it.
    #[test]
    fn define_group_declares_by_name() {
        let model = dual()
            .define_group("l", "left_hand", None, Some("left_hand"))
            .unwrap();
        assert!(!model.groups_are_derived());
        let groups = model.groups();
        assert_eq!(
            groups.len(),
            1,
            "the first declaration replaces the derived set"
        );
        let g = &groups[0];
        assert_eq!(
            names(&model, g),
            vec!["left_shoulder", "left_elbow", "left_wrist"]
        );
        assert_eq!(model.links[g.flange.unwrap()].name, "left_hand");
        assert!(!g.derived);
        // Redeclaring by name replaces; a second name adds.
        let model = model
            .define_group("l", "left_hand", Some(&["left_shoulder"]), None)
            .unwrap()
            .define_group("r", "right_hand", None, None)
            .unwrap();
        let groups = model.groups();
        assert_eq!(groups.len(), 2);
        assert_eq!(names(&model, &groups[0]), vec!["left_shoulder"]);
        assert_eq!(model.group_index("r"), Some(1));
        // A mimic follower is not a joint one drives.
        let err = dual()
            .define_group("f", "left_finger_b_link", Some(&["left_finger_b"]), None)
            .unwrap_err();
        assert!(
            matches!(err, ModelError::GroupJointNotActuated { .. }),
            "{err}"
        );
    }

    /// Two arms on a body through `dual_arm`: one robot, two declared
    /// groups with the arms' own tips, joints prefixed apart.
    #[test]
    fn dual_arm_composes_two_arms_into_groups() {
        let arm = RobotModel::from_urdf_str(ARM).unwrap();
        let left_at = Isometry3::translation(0.0, 0.3, 0.8);
        let right_at = Isometry3::translation(0.0, -0.3, 0.8);
        let pair = RobotModel::dual_arm(None, &arm, &arm, None, left_at, None, right_at).unwrap();
        assert_eq!(pair.dof(), 12);
        assert_eq!(pair.name, "simple_arm_dual");
        let groups = pair.groups();
        assert_eq!(
            groups.iter().map(|g| g.name.as_str()).collect::<Vec<_>>(),
            vec!["left", "right"]
        );
        assert_eq!(pair.links[groups[0].tip].name, "left_tool0");
        assert_eq!(pair.links[groups[1].tip].name, "right_tool0");
        assert_eq!(groups[0].joints.len(), 6);
        assert!(groups[0]
            .joints
            .iter()
            .all(|qi| !groups[1].joints.contains(qi)));
        assert!(matches!(
            pair.source,
            RobotSource::Composite {
                role: MountRole::Arm,
                ..
            }
        ));
        // A tool then goes on one arm, and only that arm's tip moves to it.
        let tool = RobotModel::from_urdf_str(GRIPPER).unwrap();
        let with_tool = pair
            .attach_tool(
                &tool,
                Some("left_tool0"),
                None,
                Isometry3::identity(),
                Some("left"),
                Some("g_"),
                None,
            )
            .unwrap();
        let groups = with_tool.groups();
        assert_eq!(with_tool.links[groups[0].tip].name, "g_left");
        assert_eq!(with_tool.links[groups[1].tip].name, "right_tool0");
        // Naming no arm and no flange is ambiguous on a two-arm robot.
        let err = pair
            .attach_tool(
                &tool,
                None,
                None,
                Isometry3::identity(),
                None,
                Some("g_"),
                None,
            )
            .unwrap_err();
        assert!(matches!(err, ModelError::AmbiguousGroup(_)), "{err}");
        // Naming the arm picks its flange... which a bare arm does not
        // declare — so the error is the flange one, not the ambiguity.
        let err = pair
            .attach_tool(
                &tool,
                None,
                None,
                Isometry3::identity(),
                None,
                Some("g_"),
                Some("right"),
            )
            .unwrap_err();
        assert!(matches!(err, ModelError::NoFlangeDeclared), "{err}");
    }

    /// Attaching a tool to a single-arm robot leaves its groups derived —
    /// the composite re-derives one whole group tipped at the tool.
    #[test]
    fn attach_tool_on_one_arm_keeps_the_groups_derived() {
        let arm = RobotModel::from_urdf_str(ARM).unwrap();
        let tool = RobotModel::from_urdf_str(GRIPPER).unwrap();
        let combined = arm
            .attach_tool(
                &tool,
                Some("tool0"),
                None,
                Isometry3::identity(),
                Some("left"),
                Some("g_"),
                None,
            )
            .unwrap();
        assert!(combined.groups_are_derived());
        let groups = combined.groups();
        assert_eq!(groups.len(), 1);
        assert_eq!(groups[0].tip, combined.default_tcp_link());
        assert_eq!(groups[0].joints.len(), combined.dof());
    }
}

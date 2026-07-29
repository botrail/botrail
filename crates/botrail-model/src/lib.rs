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

#[derive(Debug, Clone)]
pub struct Joint {
    pub name: String,
    pub joint_type: JointType,
    /// Transform from the parent link frame to the child link frame at q = 0.
    pub origin: Isometry3<f64>,
    pub axis: Unit<Vector3<f64>>,
    /// Position limits; `None` for fixed and continuous joints.
    pub limits: Option<JointLimits>,
    pub parent_link: usize,
    pub child_link: usize,
    /// Index into the joint position vector `q`; `None` for fixed joints.
    pub q_index: Option<usize>,
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
}

#[derive(Debug)]
pub struct RobotModel {
    pub name: String,
    pub links: Vec<Link>,
    pub joints: Vec<Joint>,
    pub root_link: usize,
    /// Joint indices ordered parent-before-child (tree traversal order).
    pub joint_order: Vec<usize>,
    /// Indices of non-fixed joints, in `q`-vector order (base to tip).
    pub actuated_joints: Vec<usize>,
    pub source: RobotSource,
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

    /// Default end-effector link: the leaf reached through the longest joint
    /// chain from the root (ties broken by traversal order). This is a
    /// heuristic for tools/TCP frames, which URDF does not mark explicitly.
    pub fn default_tcp_link(&self) -> usize {
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
                    .map(|v| convert_shape(&v.origin, &v.geometry, base_dir, options))
                    .collect(),
                collisions: l
                    .collisions
                    .iter()
                    .map(|c| convert_shape(&c.origin, &c.geometry, base_dir, options))
                    .collect(),
                parent_joint: None,
            })
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
            let limits = match joint_type {
                JointType::Revolute | JointType::Prismatic => Some(JointLimits {
                    lower: j.limit.lower,
                    upper: j.limit.upper,
                    velocity: j.limit.velocity,
                    effort: j.limit.effort,
                }),
                _ => None,
            };
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
            });
        }

        Self::from_parts(robot.name, links, joints, RobotSource::UrdfXml(urdf_source))
    }

    /// Builds a model from converted parts, computing the tree invariants:
    /// per-link parent joints, root detection, breadth-first joint order,
    /// q-index assignment, and loop rejection. Callers may leave
    /// `Joint::q_index` and `Link::parent_joint` unset — both are assigned
    /// here. This is the entry point for non-URDF importers.
    pub fn from_parts(
        name: String,
        mut links: Vec<Link>,
        mut joints: Vec<Joint>,
        source: RobotSource,
    ) -> Result<Self, ModelError> {
        for link in &mut links {
            link.parent_joint = None;
        }
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
                if joints[ji].joint_type.dof() > 0 {
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
        })
    }
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
    }
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

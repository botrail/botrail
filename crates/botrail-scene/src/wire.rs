//! JSON wire protocol between the botrail server and the studio UI.
//!
//! Messages (tagged with `"type"`):
//! - server -> client: `scene_init` (full scene description), `state`
//!   (joint positions + world pose of every link, in `links` order)
//! - client -> server: `set_joint_positions`

use std::path::Path;

use nalgebra::Isometry3;
use serde::{Deserialize, Serialize};

use crate::Scene;
use botrail_model::{Geometry, JointType};

/// Position + quaternion (x, y, z, w), in meters / world frame unless noted.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct PoseMsg {
    pub position: [f64; 3],
    pub quaternion: [f64; 4],
}

impl From<&Isometry3<f64>> for PoseMsg {
    fn from(iso: &Isometry3<f64>) -> Self {
        let t = iso.translation;
        let q = iso.rotation.coords;
        PoseMsg {
            position: [t.x, t.y, t.z],
            quaternion: [q.x, q.y, q.z, q.w],
        }
    }
}

impl From<&PoseMsg> for Isometry3<f64> {
    fn from(msg: &PoseMsg) -> Self {
        let [x, y, z, w] = msg.quaternion;
        let rotation = nalgebra::Unit::try_new(nalgebra::Quaternion::new(w, x, y, z), 1e-9)
            .unwrap_or_else(nalgebra::UnitQuaternion::identity);
        Isometry3::from_parts(
            nalgebra::Translation3::new(msg.position[0], msg.position[1], msg.position[2]),
            rotation,
        )
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum GeometryMsg {
    Box {
        size: [f64; 3],
    },
    Cylinder {
        radius: f64,
        length: f64,
    },
    Sphere {
        radius: f64,
    },
    Mesh {
        /// URL the studio can fetch the mesh from (e.g. `/meshes/0`).
        url: String,
        /// Lower-case file extension so the client can pick a loader.
        ext: String,
        scale: [f64; 3],
    },
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct VisualMsg {
    /// Link-local transform of this shape.
    pub origin: PoseMsg,
    pub geometry: GeometryMsg,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct LinkMsg {
    pub name: String,
    pub visuals: Vec<VisualMsg>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum JointTypeMsg {
    Revolute,
    Continuous,
    Prismatic,
    Fixed,
}

impl From<JointType> for JointTypeMsg {
    fn from(jt: JointType) -> Self {
        match jt {
            JointType::Revolute => JointTypeMsg::Revolute,
            JointType::Continuous => JointTypeMsg::Continuous,
            JointType::Prismatic => JointTypeMsg::Prismatic,
            JointType::Fixed => JointTypeMsg::Fixed,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct JointMsg {
    pub name: String,
    pub joint_type: JointTypeMsg,
    /// Index into the joint position vector; `None` for fixed joints.
    pub q_index: Option<usize>,
    /// `[lower, upper]` position limits; `None` for fixed/continuous joints.
    pub limits: Option<[f64; 2]>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct SceneDescriptionMsg {
    pub robot_name: String,
    pub links: Vec<LinkMsg>,
    pub joints: Vec<JointMsg>,
    /// Suggested end-effector link for the TCP gizmo (deepest leaf link).
    pub tcp_link: Option<String>,
}

/// Outcome of the most recent IK solve, echoed with the resulting state.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct IkStatusMsg {
    pub converged: bool,
    /// Remaining position error (m).
    pub pos_error: f64,
    /// Remaining orientation error (rad).
    pub rot_error: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum ServerMessage {
    SceneInit {
        scene: SceneDescriptionMsg,
    },
    State {
        joint_positions: Vec<f64>,
        /// World pose per link, aligned with `SceneDescriptionMsg::links`.
        link_poses: Vec<PoseMsg>,
        /// Present when this state is the result of an IK solve.
        ik_status: Option<IkStatusMsg>,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum ClientMessage {
    SetJointPositions {
        positions: Vec<f64>,
    },
    /// Ask the server to solve IK for `link` toward `pose` (world frame),
    /// seeded from the current configuration, and apply the result.
    SetTcpTarget {
        link: String,
        pose: PoseMsg,
    },
}

impl SceneDescriptionMsg {
    /// Builds the description, mapping each mesh path to a URL + extension
    /// via `mesh_url` (URL assignment is a server concern).
    pub fn from_scene(scene: &Scene, mut mesh_url: impl FnMut(&Path) -> (String, String)) -> Self {
        let robot = &scene.robot;
        let links = robot
            .links
            .iter()
            .map(|link| LinkMsg {
                name: link.name.clone(),
                visuals: link
                    .visuals
                    .iter()
                    .map(|shape| VisualMsg {
                        origin: PoseMsg::from(&shape.origin),
                        geometry: match &shape.geometry {
                            Geometry::Box { size } => GeometryMsg::Box {
                                size: [size.x, size.y, size.z],
                            },
                            Geometry::Cylinder { radius, length } => GeometryMsg::Cylinder {
                                radius: *radius,
                                length: *length,
                            },
                            Geometry::Sphere { radius } => GeometryMsg::Sphere { radius: *radius },
                            Geometry::Mesh { path, scale } => {
                                let (url, ext) = mesh_url(path);
                                GeometryMsg::Mesh {
                                    url,
                                    ext,
                                    scale: [scale.x, scale.y, scale.z],
                                }
                            }
                        },
                    })
                    .collect(),
            })
            .collect();
        let joints = robot
            .joints
            .iter()
            .map(|joint| JointMsg {
                name: joint.name.clone(),
                joint_type: joint.joint_type.into(),
                q_index: joint.q_index,
                limits: match joint.joint_type {
                    JointType::Revolute | JointType::Prismatic => {
                        joint.limits.map(|l| [l.lower, l.upper])
                    }
                    _ => None,
                },
            })
            .collect();
        SceneDescriptionMsg {
            robot_name: robot.name.clone(),
            links,
            joints,
            tcp_link: Some(robot.links[robot.default_tcp_link()].name.clone()),
        }
    }
}

/// Current scene state as a `state` message.
pub fn state_message(scene: &Scene) -> ServerMessage {
    state_message_with_ik(scene, None)
}

/// Current scene state as a `state` message, tagged with an IK outcome.
pub fn state_message_with_ik(scene: &Scene, ik_status: Option<IkStatusMsg>) -> ServerMessage {
    ServerMessage::State {
        joint_positions: scene.joint_positions().to_vec(),
        link_poses: scene.link_poses().iter().map(PoseMsg::from).collect(),
        ik_status,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use botrail_model::RobotModel;
    use std::sync::Arc;

    fn sample_scene() -> Scene {
        let urdf = r#"
        <robot name="wire_bot">
          <link name="base">
            <visual>
              <geometry><box size="0.1 0.2 0.3"/></geometry>
            </visual>
            <visual>
              <geometry><mesh filename="meshes/arm.stl" scale="2 2 2"/></geometry>
            </visual>
          </link>
          <link name="tip"/>
          <joint name="j" type="revolute">
            <parent link="base"/><child link="tip"/>
            <origin xyz="0 0 1"/>
            <axis xyz="0 0 1"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/>
          </joint>
        </robot>"#;
        Scene::new(Arc::new(RobotModel::from_urdf_str(urdf).unwrap()))
    }

    #[test]
    fn scene_description_json_shape() {
        let scene = sample_scene();
        let desc = SceneDescriptionMsg::from_scene(&scene, |path| {
            assert!(path.ends_with("meshes/arm.stl"));
            ("/meshes/0".to_string(), "stl".to_string())
        });
        let msg = ServerMessage::SceneInit { scene: desc };
        let json: serde_json::Value =
            serde_json::from_str(&serde_json::to_string(&msg).unwrap()).unwrap();
        assert_eq!(json["type"], "scene_init");
        assert_eq!(json["scene"]["robot_name"], "wire_bot");
        assert_eq!(
            json["scene"]["links"][0]["visuals"][0]["geometry"]["kind"],
            "box"
        );
        assert_eq!(
            json["scene"]["links"][0]["visuals"][1]["geometry"]["url"],
            "/meshes/0"
        );
        assert_eq!(json["scene"]["joints"][0]["joint_type"], "revolute");
        assert_eq!(json["scene"]["joints"][0]["q_index"], 0);
        assert_eq!(json["scene"]["tcp_link"], "tip");
    }

    #[test]
    fn state_message_json_shape() {
        let scene = sample_scene();
        let json: serde_json::Value =
            serde_json::from_str(&serde_json::to_string(&state_message(&scene)).unwrap()).unwrap();
        assert_eq!(json["type"], "state");
        assert_eq!(json["joint_positions"].as_array().unwrap().len(), 1);
        assert_eq!(json["link_poses"].as_array().unwrap().len(), 2);
        // tip link sits 1m above base at q = 0
        assert_eq!(json["link_poses"][1]["position"][2], 1.0);
        assert_eq!(json["ik_status"], serde_json::Value::Null);
    }

    #[test]
    fn client_message_roundtrip() {
        let msg: ClientMessage =
            serde_json::from_str(r#"{"type":"set_joint_positions","positions":[0.25]}"#).unwrap();
        assert_eq!(
            msg,
            ClientMessage::SetJointPositions {
                positions: vec![0.25]
            }
        );

        let msg: ClientMessage = serde_json::from_str(
            r#"{"type":"set_tcp_target","link":"tip","pose":{"position":[0.1,0.2,0.3],"quaternion":[0,0,0,1]}}"#,
        )
        .unwrap();
        let ClientMessage::SetTcpTarget { link, pose } = msg else {
            panic!("wrong variant");
        };
        assert_eq!(link, "tip");
        let iso: nalgebra::Isometry3<f64> = (&pose).into();
        assert!((iso.translation.z - 0.3).abs() < 1e-12);
    }

    #[test]
    fn denormalized_quaternion_is_normalized() {
        let pose = PoseMsg {
            position: [0.0; 3],
            quaternion: [0.0, 0.0, 2.0, 0.0], // 2x unit z-quat (180 deg about z)
        };
        let iso: nalgebra::Isometry3<f64> = (&pose).into();
        assert!((iso.rotation.norm() - 1.0).abs() < 1e-12);
        let zero = PoseMsg {
            position: [0.0; 3],
            quaternion: [0.0; 4],
        };
        let iso: nalgebra::Isometry3<f64> = (&zero).into();
        assert_eq!(iso.rotation, nalgebra::UnitQuaternion::identity());
    }
}

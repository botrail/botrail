//! JSON wire protocol between the botrail server and the studio UI.
//!
//! Messages (tagged with `"type"`):
//! - server -> client: `scene_init` (per-robot descriptions), `obstacles`
//!   (full obstacle list, resent on every change), `state` (per-robot joint
//!   positions and link poses, collision pairs, min obstacle distance)
//! - client -> server: `set_joint_positions`, `set_tcp_target`,
//!   `add_obstacle`, `update_obstacle_pose`, `update_obstacle_geometry`,
//!   `remove_obstacle`
//!
//! Robot addressing: server -> client messages carry the robot **instance
//! name** (`RobotDescMsg::name`) as the scope key; client -> server messages
//! take `robot: Option<String>` where `None` means the first robot (kept for
//! pre-multi-robot clients).

use std::path::Path;

use nalgebra::{Isometry3, Vector3};
use serde::{Deserialize, Serialize};

use crate::Scene;
use botrail_collide::ColliderId;
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

/// Reference to the robot's source USD stage, for client-side rendering
/// (three-usd-robot). Present only for USD-sourced robots on hosts that
/// serve assets.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct UsdAssetMsg {
    /// URL of the stage; relative references resolve against it.
    pub url: String,
    /// Articulation root prim path (joint/link names are prim paths).
    pub articulation_root: String,
}

/// One robot instance. The instance `name` is the scope key every
/// robot-addressed message uses from here on.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct RobotDescMsg {
    /// Scene-unique instance name.
    pub name: String,
    /// When present, the client renders the robot from this USD asset
    /// (client-side FK); the `links` visuals then serve only as fallback.
    pub usd_asset: Option<UsdAssetMsg>,
    /// World pose of the robot's root link.
    pub base_pose: PoseMsg,
    pub links: Vec<LinkMsg>,
    /// `q_index` is the robot-local DOF index.
    pub joints: Vec<JointMsg>,
    /// Suggested end-effector link for the TCP gizmo (deepest leaf link).
    pub tcp_link: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct SceneDescriptionMsg {
    pub robots: Vec<RobotDescMsg>,
}

/// A named world-frame pose (mount point / teach reference).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct FrameMsg {
    pub name: String,
    pub pose: PoseMsg,
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

/// Per-robot slice of a `state` message.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct RobotStateMsg {
    /// Instance name (matches `RobotDescMsg::name`).
    pub name: String,
    pub joint_positions: Vec<f64>,
    /// World pose of the robot's root link.
    pub base_pose: PoseMsg,
    /// World pose per link, aligned with `RobotDescMsg::links`.
    pub link_poses: Vec<PoseMsg>,
    /// Present on the robot an IK solve just ran for.
    pub ik_status: Option<IkStatusMsg>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct ObstacleMsg {
    pub name: String,
    pub geometry: GeometryMsg,
    /// World pose.
    pub pose: PoseMsg,
    /// Disabled obstacles render but are excluded from collision checking.
    #[serde(default = "default_true")]
    pub enabled: bool,
    /// Display colour, linear RGB, from the scene file's
    /// `primvars:displayColor`. Absent means "no authored appearance": the
    /// studio then draws the obstacle as a neutral collision proxy.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub color: Option<[f32; 3]>,
    /// Present while the obstacle is attached to (grasped by) a robot link.
    #[serde(default)]
    pub attached_to: Option<AttachmentMsg>,
}

fn default_true() -> bool {
    true
}

/// Attachment state of a grasped obstacle (see `ObstacleMsg::attached_to`).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct AttachmentMsg {
    /// Carrying robot instance name; `None` means the first robot.
    #[serde(default)]
    pub robot: Option<String>,
    /// Carrying link name.
    pub link: String,
    /// Fixed relative pose: `link ← object`.
    pub grasp: PoseMsg,
    /// Links allowed to touch the object (carrying link, gripper fingers).
    pub touch_links: Vec<String>,
}

/// One side of a collision pair.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum ColliderRefMsg {
    Link {
        /// Owning robot instance name.
        robot: String,
        name: String,
    },
    Obstacle {
        name: String,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct CollisionPairMsg {
    pub a: ColliderRefMsg,
    pub b: ColliderRefMsg,
}

/// A time-parameterized trajectory, uniformly sampled for playback.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct TrajectoryMsg {
    pub duration: f64,
    /// Sample timestamps (uniform except the exact final point).
    pub times: Vec<f64>,
    /// Joint positions per sample.
    pub joint_positions: Vec<Vec<f64>>,
    /// World pose of every link per sample (FK precomputed server-side).
    /// `None` for USD-rendered robots — the client applies
    /// `joint_positions` itself.
    pub link_poses: Option<Vec<Vec<PoseMsg>>>,
    /// World-pose track per attached (grasped) object, aligned with
    /// `times`. `None` when nothing is attached.
    #[serde(default)]
    pub object_tracks: Option<Vec<ObjectTrackMsg>>,
}

/// Per-sample world poses for one scene object riding the robot during
/// playback (a grasped obstacle).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct ObjectTrackMsg {
    /// Obstacle name.
    pub name: String,
    /// One world pose per trajectory sample.
    pub poses: Vec<PoseMsg>,
}

/// A TCP path constraint (see `motion::Constraint`).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum ConstraintMsg {
    OrientationCone {
        axis_local: [f64; 3],
        axis_world: [f64; 3],
        /// Half-angle of the cone (rad).
        angle: f64,
    },
    PositionBox {
        min: [f64; 3],
        max: [f64; 3],
    },
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum SegmentKindMsg {
    Joint,
    CartesianLine,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct SegmentMsg {
    pub kind: SegmentKindMsg,
    /// Goal configuration in DOF order.
    pub goal_positions: Vec<f64>,
    pub constraints: Vec<ConstraintMsg>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct MotionMsg {
    pub name: String,
    /// Owning robot instance name; `None` means the first robot.
    #[serde(default)]
    pub robot: Option<String>,
    pub segments: Vec<SegmentMsg>,
}

// --------------------------------------------------- sequences (PLC-style)

/// A user-defined internal signal (PLC internal relay).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct SignalDefMsg {
    pub name: String,
    pub initial: bool,
}

/// A pseudo-sensor: geometric test published as a read-only input signal.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct SensorMsg {
    pub name: String,
    pub kind: SensorKindMsg,
    pub watch: SensorWatchMsg,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum SensorKindMsg {
    /// Presence/area sensor: ON while a watched body overlaps the box.
    Zone { pose: PoseMsg, size: [f64; 3] },
    /// Photoelectric beam: ON while the segment is interrupted.
    Beam {
        from: [f64; 3],
        to: [f64; 3],
        radius: f64,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum SensorWatchMsg {
    Objects {
        names: Vec<String>,
    },
    AllObjects,
    /// Any robot's links trip it.
    Robot,
    /// Only the named robot instances' links trip it.
    Robots {
        names: Vec<String>,
    },
    All,
}

/// A scripted auxiliary device commanded from sequences.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct DeviceMsg {
    pub name: String,
    pub kind: DeviceKindMsg,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum DeviceKindMsg {
    /// Advects unattached obstacles whose origin lies inside the zone.
    Conveyor {
        zone_pose: PoseMsg,
        zone_size: [f64; 3],
        velocity: [f64; 3],
        running: bool,
    },
    /// Moves the listed obstacles along `axis` within `range`.
    LinearAxis {
        objects: Vec<String>,
        axis: [f64; 3],
        speed: f64,
        position: f64,
        range: [f64; 2],
    },
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum DeviceCommandMsg {
    Start,
    Stop,
    SetSpeed { speed: f64 },
    MoveTo { position: f64 },
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct SequenceMsg {
    pub name: String,
    pub steps: Vec<StepMsg>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct StepMsg {
    pub name: String,
    /// Entry actions, fired when the step becomes active.
    pub actions: Vec<ActionMsg>,
    /// The step completes when this condition holds.
    pub transition: ConditionMsg,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct RampTargetMsg {
    pub joint: String,
    pub value: f64,
}

/// Robot-addressed actions carry `robot` — the instance name, or `None`
/// for the scene's sole robot (ambiguous, and rejected, with several).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum ActionMsg {
    /// Start a named motion (its owning robot drives); await it with the
    /// `done` condition.
    StartMotion { motion: String },
    /// Linearly ramp joints (gripper open/close); await with `done`.
    StartRamp {
        #[serde(default)]
        robot: Option<String>,
        targets: Vec<RampTargetMsg>,
        duration: f64,
    },
    /// Grasp an obstacle at its current relative pose (instantaneous).
    Attach {
        #[serde(default)]
        robot: Option<String>,
        object: String,
        #[serde(default)]
        link: Option<String>,
        #[serde(default)]
        touch_links: Option<Vec<String>>,
    },
    /// Release an obstacle where it is (instantaneous; the carrier is
    /// looked up from the attachment).
    Detach { object: String },
    /// Latch onto a moving part: taught poses ride its motion until the
    /// track is released (conveyor tracking).
    Track {
        #[serde(default)]
        robot: Option<String>,
        object: String,
        #[serde(default)]
        link: Option<String>,
    },
    /// Stop following the tracked part (instantaneous).
    Untrack {
        #[serde(default)]
        robot: Option<String>,
    },
    /// Write an internal signal.
    Set { signal: String, value: bool },
    /// Command an auxiliary device (output coil).
    Device {
        device: String,
        command: DeviceCommandMsg,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum ConditionMsg {
    /// Always true — fire the actions and move on.
    Immediately,
    /// Every motion/ramp started by this step has finished.
    Done,
    /// The named robot has no motion/ramp in flight (interlock building
    /// block).
    RobotDone { robot: String },
    /// On-delay timer from step entry (TON).
    Elapsed { seconds: f64 },
    /// Level test of a signal.
    Signal { name: String, value: bool },
    /// A linear axis reached its commanded position.
    DeviceDone { device: String },
    /// Series contacts (AND).
    All { conditions: Vec<ConditionMsg> },
    /// Parallel contacts (OR).
    Any { conditions: Vec<ConditionMsg> },
}

/// One step's interval on a baked timeline.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct StepSpanMsg {
    pub name: String,
    pub start: f64,
    pub end: f64,
}

/// A boolean signal as a step function (timing-chart lane).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct SignalTrackMsg {
    pub name: String,
    /// Edge times; `values[i]` holds from `times[i]` on. `times[0] = 0`.
    pub times: Vec<f64>,
    pub values: Vec<bool>,
}

/// One robot's playable track on a baked timeline.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct RobotTimelineMsg {
    /// Instance name (matches `RobotDescMsg::name`).
    pub name: String,
    pub trajectory: TrajectoryMsg,
    /// Intervals a motion/ramp drove this robot, labelled with the motion
    /// name — the timeline's robot lanes.
    #[serde(default)]
    pub moves: Vec<StepSpanMsg>,
}

/// A baked sequence rollout: each robot rides its own embedded trajectory
/// on a single clock (the studio plays them with the existing machinery),
/// grasped/tracked objects follow world-pose tracks, plus the step bands
/// and signal lanes for the timeline display.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct TimelineMsg {
    /// Cycle time in seconds.
    pub duration: f64,
    /// Per-robot tracks; every trajectory shares the same sample grid.
    pub robots: Vec<RobotTimelineMsg>,
    /// World-pose track per moving scene object, aligned with the (shared)
    /// trajectory sample grid. Empty when nothing rides along.
    pub objects: Vec<ObjectTrackMsg>,
    pub step_spans: Vec<StepSpanMsg>,
    pub signals: Vec<SignalTrackMsg>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct PlanStatsMsg {
    /// Wall-clock planning + timing time.
    pub planning_time_ms: f64,
    /// Waypoints in the (shortcut) path before time sampling.
    pub waypoints: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum ServerMessage {
    SceneInit {
        scene: SceneDescriptionMsg,
    },
    /// The full obstacle list; resent whenever it changes.
    Obstacles {
        obstacles: Vec<ObstacleMsg>,
    },
    /// The full named-frame list; resent whenever it changes.
    Frames {
        frames: Vec<FrameMsg>,
    },
    State {
        /// One entry per robot, in `SceneDescriptionMsg::robots` order.
        robots: Vec<RobotStateMsg>,
        /// Colliding pairs at this configuration (empty when collision-free).
        collisions: Vec<CollisionPairMsg>,
        /// Minimum robot-obstacle distance; `null` without obstacles.
        min_distance: Option<f64>,
    },
    /// Response to a `plan_request` (broadcast to every client).
    PlanResult {
        /// Robot instance the plan is for (plays back on that robot).
        robot: String,
        ok: bool,
        error: Option<String>,
        trajectory: Option<TrajectoryMsg>,
        stats: Option<PlanStatsMsg>,
    },
    /// The full motion list; resent whenever it changes.
    Motions {
        motions: Vec<MotionMsg>,
    },
    /// The full sequence + internal-signal lists; resent on every change.
    Sequences {
        sequences: Vec<SequenceMsg>,
        signals: Vec<SignalDefMsg>,
    },
    /// The full pseudo-sensor list; resent on every change.
    Sensors {
        sensors: Vec<SensorMsg>,
    },
    /// The full device list; resent on every change.
    Devices {
        devices: Vec<DeviceMsg>,
    },
    /// Response to a `simulate_sequence` request (broadcast to every client).
    SequenceResult {
        ok: bool,
        sequence: String,
        error: Option<String>,
        timeline: Option<TimelineMsg>,
        planning_time_ms: Option<f64>,
    },
    /// Response to a USD-recording playback request (broadcast to every
    /// client): a baked animation (an Isaac Sim capture or a botrail
    /// export) lifted onto the scene's robot.
    RecordingResult {
        ok: bool,
        /// Source layer path (display form).
        source: String,
        error: Option<String>,
        /// `"joint_state"` (q(t) recovered, client plays joints) or
        /// `"transforms"` (link-pose playback) when ok.
        mode: Option<String>,
        warnings: Vec<String>,
        /// Playable timeline (no step/signal lanes) when ok.
        timeline: Option<TimelineMsg>,
    },
    /// Response to a `plan_motion` request (broadcast to every client).
    MotionResult {
        /// Owning robot instance (plays back on that robot).
        robot: String,
        ok: bool,
        motion: String,
        error: Option<String>,
        trajectory: Option<TrajectoryMsg>,
        /// Time at which each segment ends (playback markers).
        segment_ends: Vec<f64>,
        planning_time_ms: Option<f64>,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum ClientMessage {
    SetJointPositions {
        /// Target robot instance name; `None` means the first robot.
        #[serde(default)]
        robot: Option<String>,
        positions: Vec<f64>,
    },
    /// Ask the server to solve IK for `link` toward `pose` (world frame),
    /// seeded from the current configuration, and apply the result.
    SetTcpTarget {
        /// Target robot instance name; `None` means the first robot.
        #[serde(default)]
        robot: Option<String>,
        link: String,
        pose: PoseMsg,
    },
    /// Places the robot's root link at `pose` (world frame).
    SetRobotBasePose {
        /// Target robot instance name; `None` means the first robot.
        #[serde(default)]
        robot: Option<String>,
        pose: PoseMsg,
    },
    /// The server may uniquify the requested name; the authoritative list
    /// comes back in the next `obstacles` broadcast.
    AddObstacle {
        obstacle: ObstacleMsg,
    },
    UpdateObstaclePose {
        name: String,
        pose: PoseMsg,
    },
    /// Batched pose write: the studio's group gizmo drags a whole imported
    /// subtree (a pedestal, a conveyor) rather than one prim. One message
    /// rather than one per member, because each pose write rebroadcasts the
    /// entire obstacle list — a 20-prim group would otherwise cost 20 full
    /// broadcasts per drag frame.
    ///
    /// Frames ride along: a teach frame left behind by the machine it was
    /// taught on is a silently wrong cell, not a cosmetic issue.
    UpdatePoses {
        #[serde(default)]
        obstacles: Vec<(String, PoseMsg)>,
        #[serde(default)]
        frames: Vec<(String, PoseMsg)>,
    },
    UpdateObstacleGeometry {
        name: String,
        geometry: GeometryMsg,
    },
    RemoveObstacle {
        name: String,
    },
    /// Include/exclude an obstacle from collision checking.
    SetObstacleEnabled {
        name: String,
        enabled: bool,
    },
    /// Attach an obstacle to a robot link at its current relative pose.
    /// `link = None` uses the default TCP link; `touch_links = None`
    /// defaults to the link's subtree (the gripper).
    AttachObstacle {
        name: String,
        /// Carrying robot instance name; `None` means the first robot.
        #[serde(default)]
        robot: Option<String>,
        #[serde(default)]
        link: Option<String>,
        #[serde(default)]
        touch_links: Option<Vec<String>>,
    },
    /// Detach an obstacle; its pose freezes where the robot holds it.
    DetachObstacle {
        name: String,
    },
    /// Plan from the current configuration to `goal_positions` (the target
    /// robot's DOF order); other robots are frozen collision bodies.
    PlanRequest {
        /// Target robot instance name; `None` means the first robot.
        #[serde(default)]
        robot: Option<String>,
        goal_positions: Vec<f64>,
    },
    /// Append a segment (creates the motion when missing).
    AddSegment {
        motion: String,
        /// Owner when the motion is created (an existing motion keeps its
        /// owner); `None` means the first robot.
        #[serde(default)]
        robot: Option<String>,
        segment: SegmentMsg,
    },
    RemoveSegment {
        motion: String,
        index: usize,
    },
    ClearMotion {
        motion: String,
    },
    /// Plan every segment of the motion from the current configuration.
    PlanMotion {
        motion: String,
    },
    /// Add or replace a sequence wholesale (steps are small).
    UpsertSequence {
        sequence: SequenceMsg,
    },
    RemoveSequence {
        name: String,
    },
    /// Declare (or re-initialize) an internal signal.
    DefineSignal {
        name: String,
        initial: bool,
    },
    RemoveSignal {
        name: String,
    },
    /// Roll out the sequence against a scene snapshot; the result arrives
    /// as a `sequence_result`.
    SimulateSequence {
        name: String,
    },
    /// Add or replace a pseudo-sensor.
    UpsertSensor {
        sensor: SensorMsg,
    },
    RemoveSensor {
        name: String,
    },
    /// Add or replace an auxiliary device.
    UpsertDevice {
        device: DeviceMsg,
    },
    RemoveDevice {
        name: String,
    },
}

/// Converts a model geometry to its wire form; `mesh_url` maps a mesh path
/// to a URL + extension (URL assignment is a server concern).
pub fn geometry_msg(
    geometry: &Geometry,
    mesh_url: &mut impl FnMut(&Path) -> (String, String),
) -> GeometryMsg {
    match geometry {
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
    }
}

/// Converts a wire geometry back into a model geometry. Mesh geometries are
/// rejected: clients cannot upload meshes over the socket (yet).
pub fn geometry_from_msg(msg: &GeometryMsg) -> Result<Geometry, String> {
    match msg {
        GeometryMsg::Box { size } => Ok(Geometry::Box {
            size: Vector3::new(size[0], size[1], size[2]),
        }),
        GeometryMsg::Cylinder { radius, length } => Ok(Geometry::Cylinder {
            radius: *radius,
            length: *length,
        }),
        GeometryMsg::Sphere { radius } => Ok(Geometry::Sphere { radius: *radius }),
        GeometryMsg::Mesh { .. } => Err("mesh obstacles are not supported yet".to_string()),
    }
}

impl SceneDescriptionMsg {
    /// Builds the description, mapping each mesh path to a URL + extension
    /// via `mesh_url` (URL assignment is a server concern). `usd_asset`
    /// yields the host-mapped stage reference for the robot at the given
    /// index, for client-side robot rendering.
    pub fn from_scene(
        scene: &Scene,
        mut mesh_url: impl FnMut(&Path) -> (String, String),
        mut usd_asset: impl FnMut(usize) -> Option<UsdAssetMsg>,
    ) -> Self {
        let robots = scene
            .robots()
            .iter()
            .enumerate()
            .map(|(index, sr)| {
                let model = &sr.model;
                let links = model
                    .links
                    .iter()
                    .map(|link| LinkMsg {
                        name: link.name.clone(),
                        visuals: link
                            .visuals
                            .iter()
                            .map(|shape| VisualMsg {
                                origin: PoseMsg::from(&shape.origin),
                                geometry: geometry_msg(&shape.geometry, &mut mesh_url),
                            })
                            .collect(),
                    })
                    .collect();
                let joints = model
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
                RobotDescMsg {
                    name: sr.name.clone(),
                    usd_asset: usd_asset(index),
                    base_pose: PoseMsg::from(sr.base_pose()),
                    links,
                    joints,
                    tcp_link: Some(model.links[model.default_tcp_link()].name.clone()),
                }
            })
            .collect();
        SceneDescriptionMsg { robots }
    }
}

// ------------------------------------------------------ motion conversions

use crate::motion::{Constraint, Motion, Segment, SegmentKind};

fn vec3(a: [f64; 3]) -> Vector3<f64> {
    Vector3::new(a[0], a[1], a[2])
}

fn arr3(v: &Vector3<f64>) -> [f64; 3] {
    [v.x, v.y, v.z]
}

pub fn constraint_msg(c: &Constraint) -> ConstraintMsg {
    match c {
        Constraint::OrientationCone {
            axis_local,
            axis_world,
            angle,
        } => ConstraintMsg::OrientationCone {
            axis_local: arr3(axis_local),
            axis_world: arr3(axis_world),
            angle: *angle,
        },
        Constraint::PositionBox { min, max } => ConstraintMsg::PositionBox {
            min: arr3(min),
            max: arr3(max),
        },
    }
}

pub fn constraint_from_msg(msg: &ConstraintMsg) -> Constraint {
    match msg {
        ConstraintMsg::OrientationCone {
            axis_local,
            axis_world,
            angle,
        } => Constraint::OrientationCone {
            axis_local: vec3(*axis_local),
            axis_world: vec3(*axis_world),
            angle: *angle,
        },
        ConstraintMsg::PositionBox { min, max } => Constraint::PositionBox {
            min: vec3(*min),
            max: vec3(*max),
        },
    }
}

pub fn segment_msg(segment: &Segment) -> SegmentMsg {
    SegmentMsg {
        kind: match segment.kind {
            SegmentKind::Joint => SegmentKindMsg::Joint,
            SegmentKind::CartesianLine => SegmentKindMsg::CartesianLine,
        },
        goal_positions: segment.goal_positions.clone(),
        constraints: segment.constraints.iter().map(constraint_msg).collect(),
    }
}

pub fn segment_from_msg(msg: &SegmentMsg) -> Segment {
    Segment {
        kind: match msg.kind {
            SegmentKindMsg::Joint => SegmentKind::Joint,
            SegmentKindMsg::CartesianLine => SegmentKind::CartesianLine,
        },
        goal_positions: msg.goal_positions.clone(),
        constraints: msg.constraints.iter().map(constraint_from_msg).collect(),
    }
}

pub fn motion_msg(scene: &Scene, motion: &Motion) -> MotionMsg {
    MotionMsg {
        name: motion.name.clone(),
        // `None` for the first robot keeps single-robot wire/projects
        // byte-identical to the pre-multi-robot format.
        robot: (motion.robot != 0).then(|| scene.robots()[motion.robot].name.clone()),
        segments: motion.segments.iter().map(segment_msg).collect(),
    }
}

/// Converts a wire motion back into a scene motion. An unknown owning robot
/// name is rejected (`Err` carries the name).
pub fn motion_from_msg(scene: &Scene, msg: &MotionMsg) -> Result<Motion, String> {
    let robot = match &msg.robot {
        Some(name) => scene.robot_index(name).ok_or_else(|| name.clone())?,
        None => 0,
    };
    Ok(Motion {
        name: msg.name.clone(),
        robot,
        segments: msg.segments.iter().map(segment_from_msg).collect(),
    })
}

/// The full motion list as a `motions` message.
pub fn motions_message(scene: &Scene) -> ServerMessage {
    ServerMessage::Motions {
        motions: scene
            .motions()
            .iter()
            .map(|m| motion_msg(scene, m))
            .collect(),
    }
}

// ---------------------------------------------------- sequence conversions

use crate::seq::{
    Action, Condition, Device, DeviceCommand, DeviceKind, Sensor, SensorKind, SensorWatch,
    Sequence, SignalDef, Step,
};

pub fn action_msg(action: &Action) -> ActionMsg {
    match action {
        Action::StartMotion { motion } => ActionMsg::StartMotion {
            motion: motion.clone(),
        },
        Action::StartRamp {
            robot,
            targets,
            duration,
        } => ActionMsg::StartRamp {
            robot: robot.clone(),
            targets: targets
                .iter()
                .map(|(joint, value)| RampTargetMsg {
                    joint: joint.clone(),
                    value: *value,
                })
                .collect(),
            duration: *duration,
        },
        Action::Attach {
            robot,
            object,
            link,
            touch_links,
        } => ActionMsg::Attach {
            robot: robot.clone(),
            object: object.clone(),
            link: link.clone(),
            touch_links: touch_links.clone(),
        },
        Action::Detach { object } => ActionMsg::Detach {
            object: object.clone(),
        },
        Action::Track {
            robot,
            object,
            link,
        } => ActionMsg::Track {
            robot: robot.clone(),
            object: object.clone(),
            link: link.clone(),
        },
        Action::Untrack { robot } => ActionMsg::Untrack {
            robot: robot.clone(),
        },
        Action::Set { signal, value } => ActionMsg::Set {
            signal: signal.clone(),
            value: *value,
        },
        Action::Device { device, command } => ActionMsg::Device {
            device: device.clone(),
            command: match command {
                DeviceCommand::Start => DeviceCommandMsg::Start,
                DeviceCommand::Stop => DeviceCommandMsg::Stop,
                DeviceCommand::SetSpeed(speed) => DeviceCommandMsg::SetSpeed { speed: *speed },
                DeviceCommand::MoveTo(position) => DeviceCommandMsg::MoveTo {
                    position: *position,
                },
            },
        },
    }
}

pub fn action_from_msg(msg: &ActionMsg) -> Action {
    match msg {
        ActionMsg::StartMotion { motion } => Action::StartMotion {
            motion: motion.clone(),
        },
        ActionMsg::StartRamp {
            robot,
            targets,
            duration,
        } => Action::StartRamp {
            robot: robot.clone(),
            targets: targets.iter().map(|t| (t.joint.clone(), t.value)).collect(),
            duration: *duration,
        },
        ActionMsg::Attach {
            robot,
            object,
            link,
            touch_links,
        } => Action::Attach {
            robot: robot.clone(),
            object: object.clone(),
            link: link.clone(),
            touch_links: touch_links.clone(),
        },
        ActionMsg::Detach { object } => Action::Detach {
            object: object.clone(),
        },
        ActionMsg::Track {
            robot,
            object,
            link,
        } => Action::Track {
            robot: robot.clone(),
            object: object.clone(),
            link: link.clone(),
        },
        ActionMsg::Untrack { robot } => Action::Untrack {
            robot: robot.clone(),
        },
        ActionMsg::Set { signal, value } => Action::Set {
            signal: signal.clone(),
            value: *value,
        },
        ActionMsg::Device { device, command } => Action::Device {
            device: device.clone(),
            command: match command {
                DeviceCommandMsg::Start => DeviceCommand::Start,
                DeviceCommandMsg::Stop => DeviceCommand::Stop,
                DeviceCommandMsg::SetSpeed { speed } => DeviceCommand::SetSpeed(*speed),
                DeviceCommandMsg::MoveTo { position } => DeviceCommand::MoveTo(*position),
            },
        },
    }
}

pub fn seq_condition_msg(condition: &Condition) -> ConditionMsg {
    match condition {
        Condition::Immediately => ConditionMsg::Immediately,
        Condition::Done => ConditionMsg::Done,
        Condition::RobotDone { robot } => ConditionMsg::RobotDone {
            robot: robot.clone(),
        },
        Condition::Elapsed { seconds } => ConditionMsg::Elapsed { seconds: *seconds },
        Condition::Signal { name, value } => ConditionMsg::Signal {
            name: name.clone(),
            value: *value,
        },
        Condition::DeviceDone { device } => ConditionMsg::DeviceDone {
            device: device.clone(),
        },
        Condition::All(cs) => ConditionMsg::All {
            conditions: cs.iter().map(seq_condition_msg).collect(),
        },
        Condition::Any(cs) => ConditionMsg::Any {
            conditions: cs.iter().map(seq_condition_msg).collect(),
        },
    }
}

pub fn seq_condition_from_msg(msg: &ConditionMsg) -> Condition {
    match msg {
        ConditionMsg::Immediately => Condition::Immediately,
        ConditionMsg::Done => Condition::Done,
        ConditionMsg::RobotDone { robot } => Condition::RobotDone {
            robot: robot.clone(),
        },
        ConditionMsg::Elapsed { seconds } => Condition::Elapsed { seconds: *seconds },
        ConditionMsg::Signal { name, value } => Condition::Signal {
            name: name.clone(),
            value: *value,
        },
        ConditionMsg::DeviceDone { device } => Condition::DeviceDone {
            device: device.clone(),
        },
        ConditionMsg::All { conditions } => {
            Condition::All(conditions.iter().map(seq_condition_from_msg).collect())
        }
        ConditionMsg::Any { conditions } => {
            Condition::Any(conditions.iter().map(seq_condition_from_msg).collect())
        }
    }
}

pub fn sequence_msg(sequence: &Sequence) -> SequenceMsg {
    SequenceMsg {
        name: sequence.name.clone(),
        steps: sequence
            .steps
            .iter()
            .map(|s| StepMsg {
                name: s.name.clone(),
                actions: s.actions.iter().map(action_msg).collect(),
                transition: seq_condition_msg(&s.transition),
            })
            .collect(),
    }
}

pub fn sequence_from_msg(msg: &SequenceMsg) -> Sequence {
    Sequence {
        name: msg.name.clone(),
        steps: msg
            .steps
            .iter()
            .map(|s| Step {
                name: s.name.clone(),
                actions: s.actions.iter().map(action_from_msg).collect(),
                transition: seq_condition_from_msg(&s.transition),
            })
            .collect(),
    }
}

pub fn signal_def_msg(signal: &SignalDef) -> SignalDefMsg {
    SignalDefMsg {
        name: signal.name.clone(),
        initial: signal.initial,
    }
}

/// The full sequence + signal lists as a `sequences` message.
pub fn sequences_message(scene: &Scene) -> ServerMessage {
    ServerMessage::Sequences {
        sequences: scene.sequences().iter().map(sequence_msg).collect(),
        signals: scene.signals().iter().map(signal_def_msg).collect(),
    }
}

pub fn sensor_msg(sensor: &Sensor) -> SensorMsg {
    SensorMsg {
        name: sensor.name.clone(),
        kind: match &sensor.kind {
            SensorKind::Zone { pose, size } => SensorKindMsg::Zone {
                pose: PoseMsg::from(pose),
                size: [size.x, size.y, size.z],
            },
            SensorKind::Beam { from, to, radius } => SensorKindMsg::Beam {
                from: [from.x, from.y, from.z],
                to: [to.x, to.y, to.z],
                radius: *radius,
            },
        },
        watch: match &sensor.watch {
            SensorWatch::Objects(names) => SensorWatchMsg::Objects {
                names: names.clone(),
            },
            SensorWatch::AllObjects => SensorWatchMsg::AllObjects,
            SensorWatch::Robot => SensorWatchMsg::Robot,
            SensorWatch::Robots(names) => SensorWatchMsg::Robots {
                names: names.clone(),
            },
            SensorWatch::All => SensorWatchMsg::All,
        },
    }
}

pub fn sensor_from_msg(msg: &SensorMsg) -> Sensor {
    Sensor {
        name: msg.name.clone(),
        kind: match &msg.kind {
            SensorKindMsg::Zone { pose, size } => SensorKind::Zone {
                pose: pose.into(),
                size: Vector3::new(size[0], size[1], size[2]),
            },
            SensorKindMsg::Beam { from, to, radius } => SensorKind::Beam {
                from: nalgebra::Point3::new(from[0], from[1], from[2]),
                to: nalgebra::Point3::new(to[0], to[1], to[2]),
                radius: *radius,
            },
        },
        watch: match &msg.watch {
            SensorWatchMsg::Objects { names } => SensorWatch::Objects(names.clone()),
            SensorWatchMsg::AllObjects => SensorWatch::AllObjects,
            SensorWatchMsg::Robots { names } => SensorWatch::Robots(names.clone()),
            SensorWatchMsg::Robot => SensorWatch::Robot,
            SensorWatchMsg::All => SensorWatch::All,
        },
    }
}

pub fn device_msg(device: &Device) -> DeviceMsg {
    DeviceMsg {
        name: device.name.clone(),
        kind: match &device.kind {
            DeviceKind::Conveyor {
                zone_pose,
                zone_size,
                velocity,
                running,
            } => DeviceKindMsg::Conveyor {
                zone_pose: PoseMsg::from(zone_pose),
                zone_size: [zone_size.x, zone_size.y, zone_size.z],
                velocity: [velocity.x, velocity.y, velocity.z],
                running: *running,
            },
            DeviceKind::LinearAxis {
                objects,
                axis,
                speed,
                position,
                range,
            } => DeviceKindMsg::LinearAxis {
                objects: objects.clone(),
                axis: [axis.x, axis.y, axis.z],
                speed: *speed,
                position: *position,
                range: [range.0, range.1],
            },
        },
    }
}

pub fn device_from_msg(msg: &DeviceMsg) -> Device {
    Device {
        name: msg.name.clone(),
        kind: match &msg.kind {
            DeviceKindMsg::Conveyor {
                zone_pose,
                zone_size,
                velocity,
                running,
            } => DeviceKind::Conveyor {
                zone_pose: zone_pose.into(),
                zone_size: Vector3::new(zone_size[0], zone_size[1], zone_size[2]),
                velocity: Vector3::new(velocity[0], velocity[1], velocity[2]),
                running: *running,
            },
            DeviceKindMsg::LinearAxis {
                objects,
                axis,
                speed,
                position,
                range,
            } => DeviceKind::LinearAxis {
                objects: objects.clone(),
                axis: nalgebra::Unit::try_new(Vector3::new(axis[0], axis[1], axis[2]), 1e-9)
                    .unwrap_or_else(|| nalgebra::Unit::new_unchecked(Vector3::x())),
                speed: *speed,
                position: *position,
                range: (range[0], range[1]),
            },
        },
    }
}

/// The full pseudo-sensor list as a `sensors` message.
pub fn sensors_message(scene: &Scene) -> ServerMessage {
    ServerMessage::Sensors {
        sensors: scene.sensors().iter().map(sensor_msg).collect(),
    }
}

/// The full device list as a `devices` message.
pub fn devices_message(scene: &Scene) -> ServerMessage {
    ServerMessage::Devices {
        devices: scene.devices().iter().map(device_msg).collect(),
    }
}

/// The full frame list as a `frames` message.
pub fn frames_message(scene: &Scene) -> ServerMessage {
    ServerMessage::Frames {
        frames: scene
            .frames()
            .iter()
            .map(|f| FrameMsg {
                name: f.name.clone(),
                pose: PoseMsg::from(&f.pose),
            })
            .collect(),
    }
}

/// A scene attachment in wire form (link indices become link names).
pub fn attachment_msg(scene: &Scene, attachment: &crate::Attachment) -> AttachmentMsg {
    let model = &scene.robots()[attachment.robot].model;
    AttachmentMsg {
        robot: (attachment.robot != 0).then(|| scene.robots()[attachment.robot].name.clone()),
        link: model.links[attachment.link].name.clone(),
        grasp: PoseMsg::from(&attachment.grasp),
        touch_links: attachment
            .touch_links
            .iter()
            .map(|&l| model.links[l].name.clone())
            .collect(),
    }
}

/// Converts a wire attachment back into a scene attachment. Unknown robot
/// or link names are rejected (`Err` carries the offending name).
pub fn attachment_from_msg(
    scene: &Scene,
    object: &str,
    msg: &AttachmentMsg,
) -> Result<crate::Attachment, String> {
    let robot = match &msg.robot {
        Some(name) => scene.robot_index(name).ok_or_else(|| name.clone())?,
        None => 0,
    };
    let model = &scene.robots()[robot].model;
    let link_index =
        |name: &str| -> Result<usize, String> { model.link_index(name).ok_or(name.to_string()) };
    Ok(crate::Attachment {
        object: object.to_string(),
        robot,
        link: link_index(&msg.link)?,
        grasp: (&msg.grasp).into(),
        touch_links: msg
            .touch_links
            .iter()
            .map(|l| link_index(l))
            .collect::<Result<_, _>>()?,
    })
}

/// The full obstacle list as an `obstacles` message; `mesh_url` maps a mesh
/// obstacle's file path to the URL + extension clients fetch it from.
pub fn obstacles_message(
    scene: &Scene,
    mut mesh_url: impl FnMut(&Path) -> (String, String),
) -> ServerMessage {
    ServerMessage::Obstacles {
        obstacles: scene
            .obstacles()
            .iter()
            .map(|o| ObstacleMsg {
                name: o.name.clone(),
                geometry: geometry_msg(&o.geometry, &mut mesh_url),
                pose: PoseMsg::from(&o.pose),
                enabled: o.enabled,
                color: o.color,
                attached_to: scene.attachment(&o.name).map(|a| attachment_msg(scene, a)),
            })
            .collect(),
    }
}

fn collider_ref(scene: &Scene, id: ColliderId) -> ColliderRefMsg {
    match id {
        ColliderId::Link { robot, link } => ColliderRefMsg::Link {
            robot: scene.robots()[robot].name.clone(),
            name: scene.robots()[robot].model.links[link].name.clone(),
        },
        ColliderId::Obstacle(k) => ColliderRefMsg::Obstacle {
            name: scene.obstacles()[k].name.clone(),
        },
        // Scene::remap_obstacle_ids rewrites attached ids to obstacle ids
        // before pairs leave the scene layer.
        ColliderId::Attached(_) => unreachable!("attached ids are remapped by Scene"),
    }
}

/// Current scene state as a `state` message.
pub fn state_message(scene: &Scene) -> ServerMessage {
    state_message_with_ik(scene, None)
}

/// Current scene state as a `state` message; `ik_status` tags the robot at
/// the given index with an IK outcome.
pub fn state_message_with_ik(
    scene: &Scene,
    ik_status: Option<(usize, IkStatusMsg)>,
) -> ServerMessage {
    let collisions = scene
        .check_collisions()
        .into_iter()
        .map(|pair| CollisionPairMsg {
            a: collider_ref(scene, pair.a),
            b: collider_ref(scene, pair.b),
        })
        .collect();
    let robots = scene
        .robots()
        .iter()
        .enumerate()
        .map(|(index, sr)| RobotStateMsg {
            name: sr.name.clone(),
            joint_positions: sr.joint_positions().to_vec(),
            base_pose: PoseMsg::from(sr.base_pose()),
            link_poses: scene
                .link_poses_for(index)
                .iter()
                .map(PoseMsg::from)
                .collect(),
            ik_status: ik_status
                .as_ref()
                .filter(|(robot, _)| *robot == index)
                .map(|(_, status)| status.clone()),
        })
        .collect();
    ServerMessage::State {
        robots,
        collisions,
        min_distance: scene.min_obstacle_distance(),
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
        let desc = SceneDescriptionMsg::from_scene(
            &scene,
            |path| {
                assert!(path.ends_with("meshes/arm.stl"));
                ("/meshes/0".to_string(), "stl".to_string())
            },
            |_| None,
        );
        let msg = ServerMessage::SceneInit { scene: desc };
        let json: serde_json::Value =
            serde_json::from_str(&serde_json::to_string(&msg).unwrap()).unwrap();
        assert_eq!(json["type"], "scene_init");
        let robot = &json["scene"]["robots"][0];
        assert_eq!(robot["name"], "wire_bot");
        assert_eq!(robot["links"][0]["visuals"][0]["geometry"]["kind"], "box");
        assert_eq!(
            robot["links"][0]["visuals"][1]["geometry"]["url"],
            "/meshes/0"
        );
        assert_eq!(robot["joints"][0]["joint_type"], "revolute");
        assert_eq!(robot["joints"][0]["q_index"], 0);
        assert_eq!(robot["tcp_link"], "tip");
    }

    #[test]
    fn state_message_json_shape() {
        let scene = sample_scene();
        let json: serde_json::Value =
            serde_json::from_str(&serde_json::to_string(&state_message(&scene)).unwrap()).unwrap();
        assert_eq!(json["type"], "state");
        let robot = &json["robots"][0];
        assert_eq!(robot["name"], "wire_bot");
        assert_eq!(robot["joint_positions"].as_array().unwrap().len(), 1);
        assert_eq!(robot["link_poses"].as_array().unwrap().len(), 2);
        // tip link sits 1m above base at q = 0
        assert_eq!(robot["link_poses"][1]["position"][2], 1.0);
        assert_eq!(robot["ik_status"], serde_json::Value::Null);
        assert_eq!(json["collisions"].as_array().unwrap().len(), 0);
        assert_eq!(json["min_distance"], serde_json::Value::Null);
    }

    #[test]
    fn multi_robot_scene_and_state_shapes() {
        let mut scene = sample_scene();
        let model = scene.robots()[0].model.clone();
        let second = scene.add_robot(model, None, nalgebra::Isometry3::translation(1.0, 0.0, 0.0));
        assert_eq!(second, "wire_bot_2");

        let desc =
            SceneDescriptionMsg::from_scene(&scene, |_| (String::new(), String::new()), |_| None);
        assert_eq!(desc.robots.len(), 2);
        assert_eq!(desc.robots[1].name, "wire_bot_2");
        assert_eq!(desc.robots[1].base_pose.position[0], 1.0);
        // q_index stays robot-local for the twin.
        assert_eq!(desc.robots[1].joints[0].q_index, Some(0));

        // Per-robot state, with the IK tag landing on the addressed robot.
        let status = IkStatusMsg {
            converged: true,
            pos_error: 0.0,
            rot_error: 0.0,
        };
        let msg = state_message_with_ik(&scene, Some((1, status)));
        let ServerMessage::State { robots, .. } = &msg else {
            panic!("expected state");
        };
        assert_eq!(robots.len(), 2);
        assert_eq!(robots[1].name, "wire_bot_2");
        assert!((robots[1].base_pose.position[0] - 1.0).abs() < 1e-12);
        assert!(robots[0].ik_status.is_none());
        assert!(robots[1].ik_status.is_some());

        // Overlapping twins produce link collisions scoped by robot name.
        scene.set_robot_base_pose_for(1, nalgebra::Isometry3::identity());
        let json: serde_json::Value =
            serde_json::from_str(&serde_json::to_string(&state_message(&scene)).unwrap()).unwrap();
        let pair = &json["collisions"][0];
        assert_eq!(pair["a"]["kind"], "link");
        assert_eq!(pair["a"]["robot"], "wire_bot");
        assert_eq!(pair["b"]["robot"], "wire_bot_2");
    }

    #[test]
    fn robot_addressed_client_messages_parse() {
        // Old JSON without `robot` still parses (first-robot default)...
        let msg: ClientMessage =
            serde_json::from_str(r#"{"type":"plan_request","goal_positions":[0.5]}"#).unwrap();
        assert_eq!(
            msg,
            ClientMessage::PlanRequest {
                robot: None,
                goal_positions: vec![0.5]
            }
        );
        // ...and the addressed form carries the instance name.
        let msg: ClientMessage = serde_json::from_str(
            r#"{"type":"set_robot_base_pose","robot":"arm_b",
                "pose":{"position":[0,0,0],"quaternion":[0,0,0,1]}}"#,
        )
        .unwrap();
        let ClientMessage::SetRobotBasePose { robot, .. } = msg else {
            panic!("wrong variant");
        };
        assert_eq!(robot.as_deref(), Some("arm_b"));
    }

    #[test]
    fn obstacles_and_collisions_in_messages() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "ball",
                Geometry::Sphere { radius: 0.2 },
                nalgebra::Isometry3::translation(0.0, 0.0, 1.0),
            )
            .unwrap();

        let no_mesh = |_: &Path| (String::new(), String::new());
        let json: serde_json::Value = serde_json::from_str(
            &serde_json::to_string(&obstacles_message(&scene, no_mesh)).unwrap(),
        )
        .unwrap();
        assert_eq!(json["type"], "obstacles");
        assert_eq!(json["obstacles"][0]["name"], "ball");
        assert_eq!(json["obstacles"][0]["geometry"]["kind"], "sphere");
        assert_eq!(json["obstacles"][0]["pose"]["position"][2], 1.0);

        // The ball (r=0.2 at z=1.0) engulfs the tip link's location; the tip
        // has no geometry, but the base's visual box does not reach it, so
        // check a colliding configuration via a bigger obstacle instead.
        scene
            .set_obstacle_pose("ball", nalgebra::Isometry3::translation(0.0, 0.0, 0.0))
            .unwrap();
        let json: serde_json::Value =
            serde_json::from_str(&serde_json::to_string(&state_message(&scene)).unwrap()).unwrap();
        assert_eq!(json["collisions"][0]["a"]["kind"], "link");
        assert_eq!(json["collisions"][0]["a"]["robot"], "wire_bot");
        assert_eq!(json["collisions"][0]["b"]["kind"], "obstacle");
        assert_eq!(json["collisions"][0]["b"]["name"], "ball");
        assert_eq!(json["min_distance"], 0.0);
    }

    #[test]
    fn attached_obstacles_carry_attachment_state() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "box",
                Geometry::Sphere { radius: 0.02 },
                nalgebra::Isometry3::translation(0.1, 0.0, 1.0),
            )
            .unwrap();
        scene.attach_obstacle("box", Some("tip"), None).unwrap();

        let no_mesh = |_: &Path| (String::new(), String::new());
        let json: serde_json::Value = serde_json::from_str(
            &serde_json::to_string(&obstacles_message(&scene, no_mesh)).unwrap(),
        )
        .unwrap();
        assert_eq!(json["obstacles"][0]["attached_to"]["link"], "tip");
        assert_eq!(
            json["obstacles"][0]["attached_to"]["grasp"]["position"][0],
            0.1
        );
        assert_eq!(json["obstacles"][0]["attached_to"]["touch_links"][0], "tip");

        // Un-attached obstacle messages (e.g. from older clients) read back
        // with `attached_to = None`.
        let msg: ObstacleMsg = serde_json::from_str(
            r#"{"name":"o","geometry":{"kind":"sphere","radius":0.1},
                "pose":{"position":[0,0,0],"quaternion":[0,0,0,1]}}"#,
        )
        .unwrap();
        assert_eq!(msg.attached_to, None);
    }

    #[test]
    fn attach_client_messages_parse_with_defaults() {
        let msg: ClientMessage =
            serde_json::from_str(r#"{"type":"attach_obstacle","name":"box"}"#).unwrap();
        assert_eq!(
            msg,
            ClientMessage::AttachObstacle {
                name: "box".into(),
                robot: None,
                link: None,
                touch_links: None,
            }
        );
        let msg: ClientMessage = serde_json::from_str(
            r#"{"type":"attach_obstacle","name":"box","link":"tip","touch_links":["tip"]}"#,
        )
        .unwrap();
        assert_eq!(
            msg,
            ClientMessage::AttachObstacle {
                name: "box".into(),
                robot: None,
                link: Some("tip".into()),
                touch_links: Some(vec!["tip".into()]),
            }
        );
        let msg: ClientMessage =
            serde_json::from_str(r#"{"type":"detach_obstacle","name":"box"}"#).unwrap();
        assert_eq!(msg, ClientMessage::DetachObstacle { name: "box".into() });
    }

    #[test]
    fn trajectory_msg_object_tracks_default_none() {
        // Trajectories serialized before object_tracks existed still read.
        let msg: TrajectoryMsg = serde_json::from_str(
            r#"{"duration":1.0,"times":[0.0,1.0],
                "joint_positions":[[0.0],[1.0]],"link_poses":null}"#,
        )
        .unwrap();
        assert_eq!(msg.object_tracks, None);
    }

    #[test]
    fn geometry_msg_roundtrip_and_mesh_rejection() {
        let geom = geometry_from_msg(&GeometryMsg::Box {
            size: [0.1, 0.2, 0.3],
        })
        .unwrap();
        assert!(matches!(geom, Geometry::Box { size } if (size.y - 0.2).abs() < 1e-12));
        assert!(geometry_from_msg(&GeometryMsg::Mesh {
            url: "/meshes/0".into(),
            ext: "stl".into(),
            scale: [1.0, 1.0, 1.0],
        })
        .is_err());
    }

    #[test]
    fn client_message_roundtrip() {
        let msg: ClientMessage =
            serde_json::from_str(r#"{"type":"set_joint_positions","positions":[0.25]}"#).unwrap();
        assert_eq!(
            msg,
            ClientMessage::SetJointPositions {
                robot: None,
                positions: vec![0.25]
            }
        );

        let msg: ClientMessage = serde_json::from_str(
            r#"{"type":"set_tcp_target","link":"tip","pose":{"position":[0.1,0.2,0.3],"quaternion":[0,0,0,1]}}"#,
        )
        .unwrap();
        let ClientMessage::SetTcpTarget { link, pose, .. } = msg else {
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

use botrail_model::{Geometry, RobotModel};
use botrail_rope::{pass::*, rapier_rope_types::*};
use botrail_scene::{rollout::*, Scene};
use botrail_traj::JointTrajectory;
use nalgebra::{Isometry3, Vector3};
use std::sync::Arc;

/// A baked two-axis tool carriage: settle, acquire, raise/carry, release.
pub fn scene_and_pass() -> (Scene, SequenceTimeline, RopePass) {
    let model=RobotModel::from_urdf_str(r#"<robot name="carrier">
      <link name="base"><visual><geometry><box size="0.15 0.15 0.1"/></geometry></visual></link>
      <link name="slide"><visual><geometry><box size="0.06 0.06 0.1"/></geometry></visual></link>
      <joint name="x" type="prismatic"><parent link="base"/><child link="slide"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="10" velocity="1"/></joint>
      <link name="tool"><visual><geometry><box size="0.04 0.04 0.03"/></geometry></visual></link>
      <joint name="z" type="prismatic"><parent link="slide"/><child link="tool"/><axis xyz="0 0 1"/><limit lower="0" upper="1" effort="10" velocity="1"/></joint>
    </robot>"#).unwrap();
    let mut scene = Scene::new(Arc::new(model));
    scene
        .add_obstacle(
            "table",
            Geometry::Box {
                size: Vector3::new(1.5, 1.0, 0.05),
            },
            Isometry3::translation(0.25, 0.0, -0.025),
        )
        .unwrap();
    let trajectory = JointTrajectory {
        times: vec![0.0, 0.2, 0.4, 1.0, 1.4, 2.0],
        positions: vec![
            vec![0.0, 0.02],
            vec![0.0, 0.02],
            vec![0.0, 0.02],
            vec![0.15, 0.2],
            vec![0.15, 0.2],
            vec![0.15, 0.2],
        ],
        velocities: vec![vec![0.0; 2]; 6],
    };
    let robot = RobotTrack {
        name: scene.robots()[0].name.clone(),
        trajectory,
        moves: vec![],
        planned: vec![],
        base: None,
        footfalls: vec![],
        sway: vec![],
        pitch: vec![],
        rise: vec![],
        locomotion: vec![],
    };
    let timeline = SequenceTimeline {
        duration: 2.0,
        sequences: vec!["carry".into()],
        scenario: None,
        physics: None,
        physics_scope: None,
        robots: vec![robot],
        objects: vec![],
        vehicles: vec![],
        signals: vec![BoolTrack {
            name: "grip".into(),
            edges: vec![(0.0, false), (0.2, true), (1.403, false)],
            kind: LaneKind::Signal,
        }],
        step_spans: vec![],
        branches: vec![],
        grasps: vec![],
        contacts: vec![],
        cloths: vec![],
        ropes: vec![],
    };
    let pass = RopePass {
        spec: RopeSpec::new(
            "cable",
            vec![[0.0, 0.0, 0.006], [0.5, 0.0, 0.006]],
            NativeRopeMaterial::new(
                0.1,
                SpringSettings::new(500.0, 1.0),
                SpringSettings::new(20.0, 0.8),
            ),
            SamplingSettings::new(0.5 / 32.0),
            CollisionSettings::new(0.005),
        ),
        grippers: vec![GripperBinding {
            link: LinkBinding {
                robot: scene.robots()[0].name.clone(),
                link: "tool".into(),
            },
            signal: "grip".into(),
            location: botrail_rope::RopeLocation::Start,
        }],
        collision_links: vec![],
        obstacles: vec!["table".into()],
        connectors: vec![Connector {
            name: "plug".into(),
            location: botrail_rope::RopeLocation::End,
            shape: botrail_rope::shape_bridge::ShapeData::Ball(0.015),
            pose: botrail_rope::PoseData {
                position: [0.52, 0.0, 0.02],
                ..Default::default()
            },
            mass_kg: 0.01,
        }],
        step_s: 1.0 / 240.0,
        pins: vec![],
        anchors: vec![],
        color: None,
    };
    (scene, timeline, pass)
}

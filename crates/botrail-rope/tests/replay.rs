use botrail_rope::{
    pass::*,
    shape_bridge::{self, ShapeData},
    PoseData, RopeLocation,
};
use rapier_rope::{
    CollisionSettings, NativeRopeMaterial, RopeSpec, SamplingSettings, SpringSettings,
};

fn pass() -> RopePass {
    RopePass {
        spec: RopeSpec::new(
            "cable",
            vec![[0.0, 0.0, 0.4], [0.5, 0.0, 0.4]],
            NativeRopeMaterial::new(
                0.1,
                SpringSettings::new(500.0, 1.0),
                SpringSettings::new(20.0, 0.8),
            ),
            SamplingSettings::new(0.5 / 32.0),
            CollisionSettings::new(0.005),
        ),
        grippers: vec![],
        collision_links: vec![],
        obstacles: vec![],
        connectors: vec![],
        step_s: 1.0 / 240.0,
        pins: vec![],
    }
}
fn proxy() -> Proxy {
    Proxy {
        name: "robot/tool".into(),
        times: vec![0.0, 0.2, 0.6, 1.0],
        poses: vec![
            PoseData {
                position: [0.0, 0.0, 0.4],
                ..Default::default()
            },
            PoseData {
                position: [0.0, 0.0, 0.4],
                ..Default::default()
            },
            PoseData {
                position: [0.1, 0.0, 0.5],
                ..Default::default()
            },
            PoseData {
                position: [0.1, 0.0, 0.5],
                ..Default::default()
            },
        ],
        shapes: vec![],
    }
}
fn binding() -> GripperBinding {
    GripperBinding {
        link: LinkBinding {
            robot: "robot".into(),
            link: "tool".into(),
        },
        signal: "grip".into(),
        location: RopeLocation::Start,
    }
}
#[test]
fn gravity_is_z_down_and_positions_are_si_without_cloth_axis_swap() {
    let replay = drive(
        &Bake {
            duration_s: 0.1,
            proxies: vec![],
            signals: vec![],
        },
        &pass(),
    )
    .unwrap();
    let start = &replay.track.points[0][0];
    let end = &replay.track.points.last().unwrap()[0];
    assert!((end[0] - start[0]).abs() < 1e-9 && (end[1] - start[1]).abs() < 1e-9);
    assert!(end[2] < start[2] - 0.04);
    assert_eq!(replay.track.radius_m, 0.005);
    assert_eq!(replay.track.points[0].len(), 33);
    assert_eq!(replay.track.coupling, "baked_one_way_no_source_reaction");
}
#[test]
fn settle_grasp_carry_release_and_exact_final_frame_share_the_clock() {
    let mut pass = pass();
    pass.grippers.push(binding());
    // Rest on the exact old-shape bridge's floor, then carry and release.
    pass.spec
        .reference_centerline_m
        .iter_mut()
        .for_each(|p| p[2] = 0.006);
    let mut tool = proxy();
    tool.poses.iter_mut().for_each(|p| p.position[2] -= 0.394);
    let floor = Proxy {
        name: "floor".into(),
        times: vec![0.0, 1.0],
        poses: vec![PoseData::default(); 2],
        shapes: vec![(PoseData::default(), ShapeData::HalfSpace([0.0, 0.0, 1.0]))],
    };
    let bake = Bake {
        duration_s: 1.0,
        proxies: vec![tool, floor],
        signals: vec![Signal {
            name: "grip".into(),
            edges: vec![(0.0, false), (0.2, true), (0.703, false)],
        }],
    };
    let replay = drive(&bake, &pass).unwrap();
    let t = &replay.track;
    assert_eq!(t.times.first(), Some(&0.0));
    assert_eq!(t.times.last(), Some(&1.0));
    assert!(t.times.windows(2).all(|w| w[0] < w[1]));
    assert!(t.held_at(0.199).is_empty());
    assert_eq!(t.held_at(0.2), [0]);
    assert!(t.held_at(0.703).is_empty());
    let a = t.positions_at(0.2)[0];
    let b = t.positions_at(0.6)[0];
    assert!(
        (b[0] - a[0] - 0.1).abs() < 0.003 && (b[2] - a[2] - 0.1).abs() < 0.003,
        "{a:?} -> {b:?}"
    );
    assert!(t.positions_at(1.0)[0][2] < b[2] - 0.02);
    assert!(t.events.iter().all(|e| e.source_time_s == e.applied_time_s));
    assert_eq!(t.events[1].body, "robot/tool");
    let mut bytes = vec![];
    replay.native.write_json(&mut bytes).unwrap();
    let decoded = rapier_rope::RopeTrack::read_json(&bytes[..]).unwrap();
    assert_eq!(decoded.precision(), "f64");
    assert_eq!(decoded.frames().last().unwrap().capture.time_s.value(), 1.0);
    // Every recorded kinematic pose equals the same interpolated baked pose.
    for frame in decoded.frames() {
        let time = frame.capture.time_s.value();
        let p = bake.proxies[0].pose_at(time);
        let b = &frame.bodies[1];
        for i in 0..3 {
            assert!((b.translation_m[i].value() - p.position[i]).abs() < 1e-10);
        }
    }
}
#[test]
fn substep_pulse_and_final_time_event_are_not_lost() {
    let mut p = pass();
    p.step_s = 0.01;
    p.grippers.push(binding());
    let mut proxy = proxy();
    proxy.times = vec![0.0, 0.023];
    proxy.poses = vec![proxy.poses[0]; 2];
    let b = Bake {
        duration_s: 0.023,
        proxies: vec![proxy],
        signals: vec![Signal {
            name: "grip".into(),
            edges: vec![(0.0, false), (0.0031, true), (0.0032, false), (0.023, true)],
        }],
    };
    let t = drive(&b, &p).unwrap().track;
    assert!(t.times.contains(&0.0031) && t.times.contains(&0.0032));
    assert_eq!(t.held_at(0.00315), [0]);
    assert!(t.held_at(0.0032).is_empty());
    assert_eq!(t.held_at(0.023), [0]);
    assert_eq!(t.times.last(), Some(&0.023));
    assert_eq!(t.events.last().unwrap().source_time_s, 0.023);
}
#[test]
fn rotated_offset_local_anchor_keeps_its_link_frame() {
    let mut p = pass();
    p.grippers.push(binding());
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let pose = PoseData {
        position: [-0.1, -0.2, 0.4],
        quaternion: [0.0, 0.0, h, h],
    };
    let proxy = Proxy {
        name: "robot/tool".into(),
        times: vec![0.0, 0.1],
        poses: vec![pose; 2],
        shapes: vec![],
    };
    let b = Bake {
        duration_s: 0.1,
        proxies: vec![proxy],
        signals: vec![Signal {
            name: "grip".into(),
            edges: vec![(0.0, true)],
        }],
    };
    let t = drive(&b, &p).unwrap().track;
    let anchor = t.events[0].local_anchor_m.unwrap();
    assert!(
        (anchor[0] - 0.2).abs() < 1e-12
            && (anchor[1] + 0.1).abs() < 1e-12
            && anchor[2].abs() < 1e-12
    );
    let mut end = pose;
    end.position = [0.2, -0.2, 0.8];
    end.quaternion = [-0.0, -0.0, -h, -h];
    let mid = pose.interpolate(end, 0.5);
    for (a, b) in mid.position.into_iter().zip([0.05, -0.2, 0.6]) {
        assert!((a - b).abs() < 1e-14);
    }
    assert!(mid.quaternion[2] > 0.0);
    mid.native().unwrap();
}
#[test]
fn connector_is_a_native_dynamic_mass_and_not_a_driven_proxy() {
    let mut p = pass();
    p.pins.push((RopeLocation::Start, [0.0, 0.0, 0.4]));
    p.spec.reference_centerline_m[1] = [0.0, 0.0, -0.1];
    p.connectors.push(Connector {
        name: "plug".into(),
        location: RopeLocation::End,
        shape: ShapeData::Ball(0.01),
        pose: PoseData {
            position: [0.0, 0.0, -0.112],
            ..Default::default()
        },
        mass_kg: 0.01,
    });
    let t = drive(
        &Bake {
            duration_s: 0.5,
            proxies: vec![],
            signals: vec![],
        },
        &p,
    )
    .unwrap()
    .track;
    let c = &t.connectors[0];
    assert_eq!(c.poses.len(), t.times.len());
    assert_eq!(c.mass_kg, 0.01);
    let z = c.poses.last().unwrap().position[2];
    assert!(z > -0.2, "native rope must support the connector, z={z}");
    assert!(z < c.poses[0].position[2], "connector remains dynamic");
    assert!(t
        .points
        .iter()
        .all(|ps| ps.iter().flatten().all(|v| v.is_finite())));
}
#[test]
fn shape_bridge_keeps_offsets_and_mesh_topology_and_rejects_unsupported() {
    use parry3d_f64::{math::*, shape::SharedShape};
    let old = SharedShape::compound(vec![(
        Pose::from_translation(Vector::new(0.1, 0.2, 0.3)),
        SharedShape::cuboid(0.2, 0.3, 0.4),
    )]);
    let data = shape_bridge::extract(&old).unwrap();
    let new = shape_bridge::reconstruct(&data).unwrap();
    let a = old.compute_local_aabb();
    let b = new.compute_local_aabb();
    assert_eq!(
        [a.mins.x, a.mins.y, a.mins.z],
        [b.mins.x, b.mins.y, b.mins.z]
    );
    let mesh =
        SharedShape::trimesh(vec![Vector::ZERO, Vector::X, Vector::Y], vec![[0, 1, 2]]).unwrap();
    let ShapeData::TriMesh { vertices, indices } = shape_bridge::extract(&mesh).unwrap() else {
        panic!("mesh expected")
    };
    assert_eq!(vertices, [[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]);
    assert_eq!(indices, [[0, 1, 2]]);
    let error = shape_bridge::extract(&SharedShape::round_cuboid(1.0, 2.0, 3.0, 0.1))
        .unwrap_err()
        .to_string();
    assert!(error.contains("RoundCuboid") && error.contains("no approximation"));
    assert!(shape_bridge::reconstruct(&ShapeData::TriMesh {
        vertices,
        indices: vec![[0, 1, 9]]
    })
    .is_err());
}
#[test]
fn malformed_input_fails_closed() {
    let mut b = Bake {
        duration_s: 0.1,
        proxies: vec![proxy()],
        signals: vec![],
    };
    assert!(drive(&b, &pass()).is_err()); // pose clock ends after cycle
    b.duration_s = 1.0;
    b.proxies[0].poses[0].position[1] = f64::NAN;
    assert!(drive(&b, &pass()).is_err());
    let mut p = pass();
    p.grippers.push(binding());
    assert!(drive(
        &Bake {
            duration_s: 0.0,
            proxies: vec![],
            signals: vec![]
        },
        &p
    )
    .unwrap_err_string()
    .contains("missing proxy"));
}

#[test]
fn zero_duration_records_one_initial_frame_without_a_step() {
    let result = drive(
        &Bake {
            duration_s: 0.0,
            proxies: vec![],
            signals: vec![],
        },
        &pass(),
    )
    .unwrap();
    assert_eq!(result.track.times, [0.0]);
    assert_eq!(result.native.frames().len(), 1);
    assert_eq!(
        result.native.frames()[0].capture.phase,
        rapier_rope::CapturePhase::BeforeStep
    );
    assert_eq!(result.track.points[0][0], [0.0, 0.0, 0.4]);
}

#[test]
fn simultaneous_release_and_acquisition_handoff_preserves_one_attachment() {
    let mut p = pass();
    let mut a = binding();
    a.link.link = "a".into();
    a.signal = "a".into();
    let mut b = binding();
    b.link.link = "b".into();
    b.signal = "b".into();
    p.grippers = vec![a, b];
    let mut pa = proxy();
    pa.name = "robot/a".into();
    pa.times = vec![0.0, 0.06];
    pa.poses = vec![pa.poses[0]; 2];
    let mut pb = pa.clone();
    pb.name = "robot/b".into();
    pb.poses.iter_mut().for_each(|p| p.position[0] += 0.1);
    let result = drive(
        &Bake {
            duration_s: 0.06,
            proxies: vec![pa, pb],
            signals: vec![
                Signal {
                    name: "a".into(),
                    edges: vec![(0.0, false), (0.03, true), (0.04, false)],
                },
                Signal {
                    name: "b".into(),
                    edges: vec![(0.0, false), (0.04, true), (0.06, false)],
                },
            ],
        },
        &p,
    )
    .unwrap();
    assert_eq!(result.track.held_at(0.039), [0]);
    assert_eq!(result.track.held_at(0.04), [0]);
    assert!(result.track.held_at(0.06).is_empty());
    let handoff: Vec<_> = result
        .track
        .events
        .iter()
        .filter(|e| e.source_time_s == 0.04)
        .collect();
    assert_eq!(handoff.len(), 2);
    assert!(!handoff[0].closed && handoff[1].closed);
    assert_eq!(handoff[1].body, "robot/b");
}
trait ErrorString {
    fn unwrap_err_string(self) -> String;
}
impl ErrorString for Result<botrail_rope::track::Replay, botrail_rope::Error> {
    fn unwrap_err_string(self) -> String {
        match self {
            Err(e) => e.to_string(),
            Ok(_) => panic!("error expected"),
        }
    }
}

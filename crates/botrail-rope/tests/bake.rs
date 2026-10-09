#[path = "../examples/common/mod.rs"]
mod common;
use botrail_rope::pass::*;

#[test]
fn scene_bake_links_and_wire_replay_use_the_same_poses_and_leave_the_source_unchanged() {
    let (scene, mut timeline, pass) = common::scene_and_pass();
    let before = serde_json::to_vec(&botrail_session::timeline_msg(&scene, &timeline)).unwrap();
    let poses = bake(&scene, &timeline, &pass).unwrap();
    let wire = botrail_session::timeline_msg(&scene, &timeline);
    let li = scene.robots()[0]
        .model
        .links
        .iter()
        .position(|l| l.name == "tool")
        .unwrap();
    let proxy = poses
        .proxies
        .iter()
        .find(|p| p.name.ends_with("/tool"))
        .unwrap();
    for (i, p) in proxy.poses.iter().enumerate() {
        assert_eq!(
            p.position,
            wire.robots[0].trajectory.link_poses.as_ref().unwrap()[i][li].position
        );
    }
    let replay = animate(&scene, &timeline, &pass).unwrap();
    assert_eq!(
        before,
        serde_json::to_vec(&botrail_session::timeline_msg(&scene, &timeline)).unwrap()
    );
    assert!(timeline.ropes.is_empty());
    botrail_rope::track::install(&mut timeline, &replay);
    let full = botrail_session::timeline_msg(&scene, &timeline);
    assert_eq!(full.ropes[0], replay.track);
    assert!(replay
        .native
        .frames()
        .iter()
        .all(|f| f.capture.outer_dt_s.value() > 1e-10));
    // The export carries the cable as a tube mesh with time-sampled points.
    let text = botrail_session::usd::bake_timeline(
        &scene,
        &timeline,
        &Default::default(),
        None,
        None,
        "rope",
    )
    .unwrap()
    .to_usda()
    .unwrap();
    assert!(
        text.contains("def Xform \"Ropes\"") && text.contains("def Mesh \"cable\""),
        "no rope tube in the export"
    );
    assert!(text.contains("point3f[] points.timeSamples"));
    let window = botrail_session::timeline_window_msg(&scene, &timeline, 1.4);
    assert!(window.ropes[0].times[0] < 1.4 && window.ropes[0].times.last() == Some(&2.0));
    assert_eq!(window.ropes[0].times.len(), window.ropes[0].held.len());
    assert!(replay.track.positions_at(1.0)[0][2] > 0.15);
}
#[test]
fn names_and_unknown_or_omitted_geometry_fail_with_a_reason() {
    let (scene, mut timeline, mut pass) = common::scene_and_pass();
    timeline.robots[0].name = "wrong".into();
    assert!(bake(&scene, &timeline, &pass)
        .unwrap_err()
        .to_string()
        .contains("names/order"));
    timeline.robots[0].name = scene.robots()[0].name.clone();
    pass.grippers[0].link.link = "missing".into();
    assert!(bake(&scene, &timeline, &pass)
        .unwrap_err()
        .to_string()
        .contains("unknown link"));
}
#[test]
fn an_obstacle_anchor_is_a_frame_and_may_be_one_the_cell_does_not_check() {
    use botrail_model::Geometry;
    use nalgebra::{Isometry3, Vector3};
    let (mut scene, timeline, mut pass) = common::scene_and_pass();
    scene
        .add_obstacle(
            "clamp",
            Geometry::Box {
                size: Vector3::new(0.03, 0.02, 0.02),
            },
            Isometry3::translation(0.49, 0.0, 0.006),
        )
        .unwrap();
    scene.set_obstacle_enabled("clamp", false).unwrap();
    pass.connectors.clear();
    pass.anchors.push(Anchor {
        body: AnchorBody::Obstacle("clamp".into()),
        location: botrail_rope::RopeLocation::End,
        length_m: 0.03,
    });
    let baked = bake(&scene, &timeline, &pass).unwrap();
    let clamp = baked
        .proxies
        .iter()
        .find(|p| p.name == "obstacle/clamp")
        .unwrap();
    assert!(clamp.shapes.is_empty());
    let track = drive(&baked, &pass).unwrap().track;
    let (first, last) = (&track.points[0], track.points.last().unwrap());
    let end = first.len() - 1;
    for i in [end, end - 1] {
        let moved = (0..3)
            .map(|k| (last[i][k] - first[i][k]).powi(2))
            .sum::<f64>()
            .sqrt();
        assert!(moved < 1e-3, "held sample {i} moved {moved}");
    }
    pass.obstacles.push("clamp".into());
    assert!(bake(&scene, &timeline, &pass)
        .unwrap_err()
        .to_string()
        .contains("disabled obstacle clamp"));
}
#[test]
fn a_duration_a_bit_off_the_last_knot_is_the_same_clock() {
    // The rollout sums the duration and the knot times separately; they can
    // differ in the last bit and the replay must not refuse the bake.
    let (scene, mut timeline, pass) = common::scene_and_pass();
    timeline.duration = f64::from_bits(timeline.duration.to_bits() + 1);
    let baked = bake(&scene, &timeline, &pass).unwrap();
    assert!(baked
        .proxies
        .iter()
        .all(|p| *p.times.last().unwrap() == timeline.duration));
    let track = drive(&baked, &pass).unwrap().track;
    assert_eq!(track.times.last(), Some(&timeline.duration));
    timeline.duration += 1e-6;
    assert!(bake(&scene, &timeline, &pass)
        .unwrap_err()
        .to_string()
        .contains("invalid trajectory"));
}

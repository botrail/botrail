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
    assert!(botrail_session::usd::bake_timeline(
        &scene,
        &timeline,
        &Default::default(),
        None,
        None,
        "rope"
    )
    .err()
    .expect("rope export must fail")
    .contains("rope USD export is not implemented"));
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

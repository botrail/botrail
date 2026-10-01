mod common;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "target/rope-replay.json".into());
    let (scene, mut timeline, pass) = common::scene_and_pass();
    let replay = botrail_rope::pass::animate(&scene, &timeline, &pass)?;
    botrail_rope::track::install(&mut timeline, &replay);
    let wire = botrail_session::timeline_msg(&scene, &timeline);
    let messages = vec![
        botrail_scene::wire::ServerMessage::SceneInit {
            scene: botrail_scene::wire::SceneDescriptionMsg::from_scene(
                &scene,
                |_| unreachable!("primitive-only demo"),
                |_| None,
            ),
        },
        botrail_scene::wire::obstacles_message(&scene, |_| unreachable!("primitive-only demo")),
        botrail_scene::wire::ServerMessage::SequenceResult {
            sequence: "carry".into(),
            ok: true,
            error: None,
            timeline: Some(wire),
            scenario: None,
            planning_time_ms: None,
            stream: None,
        },
    ];
    std::fs::write(&path, serde_json::to_vec(&messages)?)?;
    replay
        .native
        .write_json(std::fs::File::create(format!("{path}.native.json"))?)?;
    println!(
        "{} rope samples, {} events, exact final {}s -> {path}",
        replay.track.times.len(),
        replay.track.events.len(),
        replay.track.times.last().unwrap()
    );
    Ok(())
}

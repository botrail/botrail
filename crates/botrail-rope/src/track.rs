//! The simulation result carries both a timeline track and native observations.
pub struct Replay {
    pub track: crate::RopeTrack,
    /// Checked f64 snapshots, events and body poses; playback-only, not a checkpoint.
    pub native: rapier_rope::RopeTrack,
}

/// Replaces a same-name track only after the complete pass succeeds.
pub fn install(timeline: &mut botrail_scene::rollout::SequenceTimeline, replay: &Replay) {
    timeline.ropes.retain(|t| t.name != replay.track.name);
    timeline.ropes.push(replay.track.clone());
}

/// Installs a track computed without a native replay (the static shape),
/// replacing a same-name track.
pub fn install_track(
    timeline: &mut botrail_scene::rollout::SequenceTimeline,
    track: crate::RopeTrack,
) {
    timeline.ropes.retain(|t| t.name != track.name);
    timeline.ropes.push(track);
}

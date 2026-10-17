//! Visibility tracks for progressive-removal and film build-up display:
//! stage obstacles (static meshes registered by the caller) shown for
//! their windows while the original object is stowed. Presentation only.

use crate::rollout::{ObjectTrack, SequenceTimeline, TrackSpan};
use nalgebra::Isometry3;

/// One visible window of a stage obstacle: `name` shows from `from` until `to`
/// (seconds on the timeline) and is stowed outside it.
#[derive(Debug, Clone, PartialEq)]
pub struct StageWindow {
    pub name: String,
    pub from: f64,
    pub to: f64,
}

/// The timeline with progressive display injected: `target` shows until
/// `handover`, then each window's obstacle shows for its span (stowed
/// outside it). Windows may overlap (one chain per tile of a carved stock)
/// or follow one another (a single chain of film stages). `windows[i].name`
/// must be an obstacle placed at `pose`; the caller registers those.
/// Nothing else about the timeline changes.
pub fn staged_timeline_windows(
    timeline: &SequenceTimeline,
    target: &str,
    pose: Isometry3<f64>,
    handover: f64,
    windows: &[StageWindow],
) -> SequenceTimeline {
    let mut out = timeline.clone();
    let end = out.duration;
    let handover = handover.clamp(0.0, end);
    out.objects.retain(|track| track.name != target);
    let mut spans = vec![TrackSpan::Hold {
        t0: 0.0,
        t1: handover,
        pose,
    }];
    if handover < end {
        spans.push(TrackSpan::Stowed {
            t0: handover,
            t1: end,
            pose,
        });
    }
    out.objects.push(ObjectTrack {
        name: target.to_string(),
        spans,
    });
    for window in windows {
        let from = window.from.clamp(0.0, end);
        let to = window.to.clamp(from, end);
        let mut spans = Vec::with_capacity(3);
        if from > 0.0 {
            spans.push(TrackSpan::Stowed {
                t0: 0.0,
                t1: from,
                pose,
            });
        }
        spans.push(TrackSpan::Hold {
            t0: from,
            t1: to,
            pose,
        });
        if to < end {
            spans.push(TrackSpan::Stowed {
                t0: to,
                t1: end,
                pose,
            });
        }
        out.objects.push(ObjectTrack {
            name: window.name.clone(),
            spans,
        });
    }
    out
}

/// A single chain of stages: `stage_names[i]` shows from `stage_times[i]`
/// until the next stage (the last one through the end of the cycle), and
/// `target` shows until the first stage. See [`staged_timeline_windows`].
pub fn staged_timeline(
    timeline: &SequenceTimeline,
    target: &str,
    pose: Isometry3<f64>,
    stage_names: &[String],
    stage_times: &[f64],
) -> SequenceTimeline {
    let end = timeline.duration;
    let first = stage_times.first().copied().unwrap_or(end);
    let windows: Vec<StageWindow> = stage_names
        .iter()
        .zip(stage_times)
        .enumerate()
        .map(|(i, (name, &from))| StageWindow {
            name: name.clone(),
            from,
            to: stage_times.get(i + 1).copied().unwrap_or(end),
        })
        .collect();
    staged_timeline_windows(timeline, target, pose, first, &windows)
}

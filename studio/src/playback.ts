// Pure helpers for trajectory playback interpolation.
//
// Playback is normalized into `PlaybackTracks` — per-robot trajectories on
// one clock plus world-pose object tracks — whatever produced it (a plan
// preview, a motion, a sequence timeline, a USD recording). Sampling yields
// the per-robot display overrides the viewport applies.

import type {
  ObjectTrackMsg,
  PoseMsg,
  TimelineMsg,
  TrajectoryMsg,
} from "./protocol";

/** Per-robot trajectories on a single clock. */
export interface PlaybackTracks {
  duration: number;
  /** One track per moving robot; robots not listed stay at their live state. */
  robots: { name: string; trajectory: TrajectoryMsg }[];
  /** World-pose tracks of moving scene objects, sampled on `times`. */
  objects: { times: number[]; tracks: ObjectTrackMsg[] } | null;
}

/** Display overrides at one playback instant. */
export interface PlaybackSample {
  /** Robot name -> world pose per link (legacy link-visual robots). */
  poses: Record<string, PoseMsg[]> | null;
  /** Robot name -> joint values (USD robots, client-side FK). */
  joints: Record<string, number[]> | null;
  /** Obstacle name -> world pose (attached objects riding along). */
  objects: Record<string, PoseMsg> | null;
}

/** Tracks for a single-robot result trajectory (plan / motion preview). */
export function tracksFromTrajectory(
  robot: string,
  traj: TrajectoryMsg,
): PlaybackTracks {
  return {
    duration: traj.duration,
    robots: [{ name: robot, trajectory: traj }],
    objects:
      traj.object_tracks && traj.object_tracks.length > 0
        ? { times: traj.times, tracks: traj.object_tracks }
        : null,
  };
}

/** Tracks for a baked timeline (sequence rollout / USD recording). */
export function tracksFromTimeline(timeline: TimelineMsg): PlaybackTracks {
  return {
    duration: timeline.duration,
    robots: timeline.robots.map((r) => ({
      name: r.name,
      trajectory: r.trajectory,
    })),
    objects:
      timeline.objects.length > 0
        ? {
            // All robot tracks of a timeline share one sample grid.
            times: timeline.robots[0]?.trajectory.times ?? [],
            tracks: timeline.objects,
          }
        : null,
  };
}

/** Every robot's override + object poses at time `t` (clamped). */
export function samplePlayback(
  tracks: PlaybackTracks,
  t: number,
): PlaybackSample {
  let poses: Record<string, PoseMsg[]> | null = null;
  let joints: Record<string, number[]> | null = null;
  for (const { name, trajectory } of tracks.robots) {
    if (trajectory.link_poses) {
      (poses ??= {})[name] = samplePoses(
        { ...trajectory, link_poses: trajectory.link_poses },
        t,
      );
    } else {
      (joints ??= {})[name] = sampleJoints(trajectory, t);
    }
  }
  return { poses, joints, objects: sampleObjectPoses(tracks.objects, t) };
}

function lerp(a: number, b: number, u: number): number {
  return a + (b - a) * u;
}

/** Normalized linear quaternion interpolation (shortest arc). */
function nlerpQuat(
  a: [number, number, number, number],
  b: [number, number, number, number],
  u: number,
): [number, number, number, number] {
  const dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  const sign = dot < 0 ? -1 : 1;
  const q: [number, number, number, number] = [
    lerp(a[0], sign * b[0], u),
    lerp(a[1], sign * b[1], u),
    lerp(a[2], sign * b[2], u),
    lerp(a[3], sign * b[3], u),
  ];
  const norm = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm];
}

/** Index pair + blend factor around time `t` (clamped). */
function bracket(times: number[], t: number): [number, number, number] {
  const last = times.length - 1;
  if (t <= times[0]) return [0, 0, 0];
  if (t >= times[last]) return [last, last, 0];
  let lo = 0;
  let hi = last;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (times[mid] <= t) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return [lo, hi, (t - times[lo]) / (times[hi] - times[lo])];
}

function lerpPose(a: PoseMsg, b: PoseMsg, u: number): PoseMsg {
  return {
    position: [
      lerp(a.position[0], b.position[0], u),
      lerp(a.position[1], b.position[1], u),
      lerp(a.position[2], b.position[2], u),
    ],
    quaternion: nlerpQuat(a.quaternion, b.quaternion, u),
  };
}

/**
 * World poses of moving objects at time `t`, keyed by obstacle name.
 * `null` when there are no object tracks.
 */
export function sampleObjectPoses(
  objects: PlaybackTracks["objects"],
  t: number,
): Record<string, PoseMsg> | null {
  if (!objects || objects.tracks.length === 0 || objects.times.length === 0) {
    return null;
  }
  const [lo, hi, u] = bracket(objects.times, t);
  const out: Record<string, PoseMsg> = {};
  for (const track of objects.tracks) {
    out[track.name] = lerpPose(track.poses[lo], track.poses[hi], u);
  }
  return out;
}

/** Joint positions at time `t`, linearly interpolated. */
export function sampleJoints(traj: TrajectoryMsg, t: number): number[] {
  const [lo, hi, u] = bracket(traj.times, t);
  return traj.joint_positions[lo].map((a, i) =>
    lerp(a, traj.joint_positions[hi][i], u),
  );
}

/**
 * Link poses at time `t`, interpolated between the trajectory's ~30Hz
 * samples (linear positions, nlerp orientations). `t` is clamped. Only
 * valid for trajectories that carry precomputed poses (legacy robots).
 */
export function samplePoses(
  traj: TrajectoryMsg & { link_poses: PoseMsg[][] },
  t: number,
): PoseMsg[] {
  const [lo, hi, u] = bracket(traj.times, t);
  return traj.link_poses[lo].map((pa, link) =>
    lerpPose(pa, traj.link_poses[hi][link], u),
  );
}

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
  VehicleTrackMsg,
} from "./protocol";

/** Per-robot trajectories on a single clock. */
export interface PlaybackTracks {
  duration: number;
  /** One track per moving robot; robots not listed stay at their live state.
   *  `base` is present only for a robot riding a vehicle, sampled on the
   *  trajectory's own times. */
  robots: { name: string; trajectory: TrajectoryMsg; base?: PoseMsg[] }[];
  /** World-pose tracks of moving scene objects, sampled on `times`. */
  objects: { times: number[]; tracks: ObjectTrackMsg[] } | null;
  /** Vehicle reference-frame tracks — what places mounted sensors. */
  vehicles: { times: number[]; tracks: VehicleTrackMsg[] } | null;
}

/** Display overrides at one playback instant. */
export interface PlaybackSample {
  /** Robot name -> world pose per link (legacy link-visual robots). */
  poses: Record<string, PoseMsg[]> | null;
  /** Robot name -> joint values (USD robots, client-side FK). */
  joints: Record<string, number[]> | null;
  /** Obstacle name -> world pose (attached objects riding along). */
  objects: Record<string, PoseMsg> | null;
  /** Objects stowed at this instant; not drawn. */
  stowed: Set<string>;
  /** Robot name -> base pose, for robots riding a vehicle. */
  bases: Record<string, PoseMsg> | null;
  /** Vehicle name -> reference-frame pose (places mounted sensors). */
  vehicles: Record<string, PoseMsg> | null;
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
    vehicles: null,
  };
}

/** The shared sample grid of a timeline's tracks. Robot trajectories carry
 * it explicitly; a robot-less cell (an AGV loop, a conveyor line) ships
 * only per-sample poses, so the clock is rebuilt from the duration and the
 * densest track — the same uniform grid the server sampled on. */
function timelineTimes(timeline: TimelineMsg): number[] {
  const fromRobot = timeline.robots[0]?.trajectory.times;
  if (fromRobot && fromRobot.length > 0) return fromRobot;
  const n = Math.max(
    0,
    ...timeline.objects.map((o) => o.poses.length),
    ...(timeline.vehicles ?? []).map((v) => v.poses.length),
  );
  if (n <= 1) return [0];
  return Array.from({ length: n }, (_, k) => (k / (n - 1)) * timeline.duration);
}

/** Tracks for a baked timeline (sequence rollout / USD recording). */
export function tracksFromTimeline(timeline: TimelineMsg): PlaybackTracks {
  const times = timelineTimes(timeline);
  return {
    duration: timeline.duration,
    robots: timeline.robots.map((r) => ({
      name: r.name,
      trajectory: r.trajectory,
      base: r.base && r.base.length > 0 ? r.base : undefined,
    })),
    objects:
      timeline.objects.length > 0
        ? {
            times,
            tracks: timeline.objects,
          }
        : null,
    vehicles:
      timeline.vehicles && timeline.vehicles.length > 0
        ? {
            times,
            tracks: timeline.vehicles,
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
  let bases: Record<string, PoseMsg> | null = null;
  for (const { name, trajectory, base } of tracks.robots) {
    if (base && base.length > 0) {
      (bases ??= {})[name] = samplePose(trajectory.times, base, t);
    }
    if (trajectory.link_poses) {
      (poses ??= {})[name] = samplePoses(
        { ...trajectory, link_poses: trajectory.link_poses },
        t,
      );
    } else {
      (joints ??= {})[name] = sampleJoints(trajectory, t);
    }
  }
  let vehicles: Record<string, PoseMsg> | null = null;
  if (tracks.vehicles) {
    for (const { name, poses: vposes } of tracks.vehicles.tracks) {
      // A one-pose track is constant (the collapsed form off the wire).
      (vehicles ??= {})[name] =
        vposes.length === 1
          ? vposes[0]
          : samplePose(tracks.vehicles.times, vposes, t);
    }
  }
  return {
    poses,
    joints,
    bases,
    vehicles,
    objects: sampleObjectPoses(tracks.objects, t),
    stowed: sampleStowedObjects(tracks.objects, t),
  };
}

/** One interpolated pose from a pose track sampled on `times`. */
function samplePose(times: number[], poses: PoseMsg[], t: number): PoseMsg {
  const [i, j, u] = bracket(times, t);
  const a = poses[i];
  const b = poses[j] ?? a;
  return {
    position: [
      lerp(a.position[0], b.position[0], u),
      lerp(a.position[1], b.position[1], u),
      lerp(a.position[2], b.position[2], u),
    ],
    quaternion: nlerpQuat(a.quaternion, b.quaternion, u),
  };
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
    // A single-pose track is a constant: the object never moves, its
    // whole animation is the visibility flags (a carve stage).
    out[track.name] =
      track.poses.length === 1
        ? track.poses[0]
        : lerpPose(track.poses[lo], track.poses[hi], u);
  }
  return out;
}

/**
 * Objects that are stowed at time `t` — waiting in a magazine or taken off
 * the line — and so should not be drawn. A track with no `visible` flags is
 * on the line throughout, which is every track a cell without magazines
 * produces.
 */
export function sampleStowedObjects(
  objects: PlaybackTracks["objects"],
  t: number,
): Set<string> {
  const stowed = new Set<string>();
  if (!objects || objects.times.length === 0) return stowed;
  const [lo, , ] = bracket(objects.times, t);
  for (const track of objects.tracks) {
    const flags = track.visible ?? [];
    if (flags.length > 0 && !flags[lo]) stowed.add(track.name);
  }
  return stowed;
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

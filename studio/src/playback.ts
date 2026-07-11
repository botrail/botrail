// Pure helpers for trajectory playback interpolation.

import type { PoseMsg, TrajectoryMsg } from "./protocol";

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

/**
 * Link poses at time `t`, interpolated between the trajectory's ~30Hz
 * samples (linear positions, nlerp orientations). `t` is clamped.
 */
export function samplePoses(traj: TrajectoryMsg, t: number): PoseMsg[] {
  const times = traj.times;
  const last = times.length - 1;
  if (t <= times[0]) return traj.link_poses[0];
  if (t >= times[last]) return traj.link_poses[last];

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
  const u = (t - times[lo]) / (times[hi] - times[lo]);
  return traj.link_poses[lo].map((pa, link) => {
    const pb = traj.link_poses[hi][link];
    return {
      position: [
        lerp(pa.position[0], pb.position[0], u),
        lerp(pa.position[1], pb.position[1], u),
        lerp(pa.position[2], pb.position[2], u),
      ],
      quaternion: nlerpQuat(pa.quaternion, pb.quaternion, u),
    };
  });
}

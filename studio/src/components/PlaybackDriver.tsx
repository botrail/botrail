import { useFrame } from "@react-three/fiber";

import { sampleJoints, samplePoses } from "../playback";
import { useStudioStore } from "../store";

/** Advances trajectory playback each rendered frame while playing. */
export function PlaybackDriver() {
  useFrame((_, delta) => {
    const s = useStudioStore.getState();
    if (!s.playing || !s.trajectory) return;
    const traj = s.trajectory;
    const t = Math.min(s.playbackTime + delta, traj.duration);
    if (traj.link_poses) {
      // Legacy robots: precomputed link poses drive the display.
      s.setPlayback(t, samplePoses({ ...traj, link_poses: traj.link_poses }, t), null);
    } else {
      // USD-rendered robots: joint values, FK happens client-side.
      s.setPlayback(t, null, sampleJoints(traj, t));
    }
    if (t >= traj.duration) {
      s.setPlaying(false);
    }
  });
  return null;
}

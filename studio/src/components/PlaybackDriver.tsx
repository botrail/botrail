import { useFrame } from "@react-three/fiber";

import { sampleOverride } from "../playback";
import { useStudioStore } from "../store";

/** Advances trajectory playback each rendered frame while playing. */
export function PlaybackDriver() {
  useFrame((_, delta) => {
    const s = useStudioStore.getState();
    if (!s.playing || !s.trajectory) return;
    const traj = s.trajectory;
    const t = Math.min(s.playbackTime + delta, traj.duration);
    // Poses for legacy robots, joints for USD robots (client-side FK), and
    // attached-object poses for either.
    s.setPlayback(t, ...sampleOverride(traj, t));
    if (t >= traj.duration) {
      s.setPlaying(false);
    }
  });
  return null;
}

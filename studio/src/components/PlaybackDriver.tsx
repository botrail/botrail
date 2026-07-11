import { useFrame } from "@react-three/fiber";

import { samplePoses } from "../playback";
import { useStudioStore } from "../store";

/** Advances trajectory playback each rendered frame while playing. */
export function PlaybackDriver() {
  useFrame((_, delta) => {
    const s = useStudioStore.getState();
    if (!s.playing || !s.trajectory) return;
    const t = Math.min(s.playbackTime + delta, s.trajectory.duration);
    s.setPlayback(t, samplePoses(s.trajectory, t));
    if (t >= s.trajectory.duration) {
      s.setPlaying(false);
    }
  });
  return null;
}

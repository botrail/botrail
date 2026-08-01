import { useFrame } from "@react-three/fiber";

import { samplePlayback } from "../playback";
import { useStudioStore } from "../store";

/** Advances trajectory playback each rendered frame while playing. */
export function PlaybackDriver() {
  useFrame((_, delta) => {
    const s = useStudioStore.getState();
    if (!s.playing || !s.playback) return;
    const tracks = s.playback;
    const t = Math.min(s.playbackTime + delta, tracks.duration);
    // Poses for legacy robots, joints for USD robots (client-side FK), and
    // attached-object poses — each robot keyed by its instance name.
    s.setPlayback(t, samplePlayback(tracks, t));
    if (t >= tracks.duration) {
      s.setPlaying(false);
    }
  });
  return null;
}

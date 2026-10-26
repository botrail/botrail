import { useEffect, useMemo } from "react";

import { updateLamps } from "../playbackRig";
import { useStudioStore } from "../store";

/**
 * Indicator lamps (`scene.add_lamp`): an obstacle lit in its colour while
 * the bound signal is on. The playback driver lights them every frame
 * while playing; this keeps them right while *paused* — a seek onto the
 * fault, the end of a run — and darkens them when no timeline is loaded
 * (the signal only exists on a bake). Nothing is mounted: the lamp is the
 * obstacle's own mesh.
 */
export function LampView() {
  const all = useStudioStore((s) => s.flashes);
  const lamps = useMemo(() => all.filter((f) => f.kind === "lamp"), [all]);
  const playing = useStudioStore((s) => s.playing);
  const playbackTime = useStudioStore((s) => s.playbackTime);
  const timeline = useStudioStore((s) => s.timeline);
  const obstacles = useStudioStore((s) => s.obstacles);
  useEffect(() => {
    if (playing || lamps.length === 0) return;
    updateLamps(lamps, timeline?.signals ?? [], playbackTime);
  }, [playing, playbackTime, timeline, lamps, obstacles]);
  return null;
}

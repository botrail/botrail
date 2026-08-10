import { useRef } from "react";
import { useFrame } from "@react-three/fiber";

import { samplePlayback, type PlaybackSample } from "../playback";
import { applySample, playbackRig, signalAt, styleFlash } from "../playbackRig";
import { useStudioStore, robotByName, type StudioState } from "../store";

/** How often the React-side playhead (timeline cursor, time label) is
 * refreshed while playing. Poses go straight onto the Object3Ds every
 * rendered frame; this is only the UI's notion of "now". */
const UI_PERIOD = 0.12;

/**
 * Advances trajectory playback each rendered frame while playing.
 *
 * The hot path is imperative: every frame samples the tracks and writes
 * transforms onto the registered objects (see `playbackRig`), so playing a
 * 16-robot line costs the sampling, not a React reconciliation of the
 * whole scene. State is synchronized at the edges — a full sample lands in
 * the store when playback starts, pauses, seeks, or ends, so everything
 * renders correctly from state the moment the driver stops driving.
 */
export function PlaybackDriver() {
  // The driver's own clock. Store `playbackTime` is only updated at
  // UI_PERIOD while playing, so the authoritative time lives here and
  // re-adopts the store value whenever playback (re)starts.
  const clock = useRef(0);
  const wasPlaying = useRef(false);
  const lastPushed = useRef(-1);

  useFrame((_, delta) => {
    const s = useStudioStore.getState();
    if (!s.playing || !s.playback) {
      wasPlaying.current = false;
      return;
    }
    const tracks = s.playback;
    if (!wasPlaying.current) {
      // (Re)starting: resume from wherever the UI left the playhead.
      clock.current = s.playbackTime >= tracks.duration ? 0 : s.playbackTime;
      wasPlaying.current = true;
      lastPushed.current = -1;
    }
    // Clamp the frame delta: a backgrounded or software-rendered tab can
    // sit without rAF for seconds (or minutes), and an unclamped delta
    // would leap the playhead to the end the moment it wakes.
    let t = clock.current + Math.min(delta, 0.25) * s.playbackSpeed;
    if (t >= tracks.duration) {
      if (s.playbackLoop) {
        t = t % tracks.duration;
      } else {
        // Land exactly on the end and hand the display back to React.
        s.setPlayback(tracks.duration, samplePlayback(tracks, tracks.duration));
        s.setPlaying(false);
        wasPlaying.current = false;
        return;
      }
    }
    clock.current = t;
    const sample = samplePlayback(tracks, t);
    applySample(sample);
    updateFlashes(s, sample, t);
    if (
      lastPushed.current < 0 ||
      Math.abs(t - lastPushed.current) >= UI_PERIOD
    ) {
      s.setPlaybackTime(t);
      lastPushed.current = t;
    }
  });
  return null;
}

/** Positions and blinks every declared weld flash at time `t`: on while
 * its weld-current signal is on, standing at the bound robot's TCP. */
function updateFlashes(s: StudioState, sample: PlaybackSample, t: number) {
  if (playbackRig.flashes.size === 0) return;
  const signals = s.timeline?.signals ?? [];
  for (const flash of s.flashes) {
    const node = playbackRig.flashes.get(flash.name);
    if (!node) continue;
    const lane = signals.find((sig: { name: string }) => sig.name === flash.signal);
    const poses = sample.poses?.[flash.robot];
    const robot = robotByName(s.robots, flash.robot);
    const tcpName = robot?.desc.tcp_link ?? null;
    const tcpIndex =
      tcpName && robot
        ? robot.desc.links.findIndex((l) => l.name === tcpName)
        : -1;
    const on =
      !!lane && !!poses && tcpIndex >= 0 && signalAt(lane.times, lane.values, t);
    if (!on || !poses) {
      node.visible = false;
      continue;
    }
    styleFlash(node, poses[tcpIndex], t);
  }
}


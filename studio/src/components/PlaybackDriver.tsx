import { useRef } from "react";
import { useFrame } from "@react-three/fiber";
import * as THREE from "three";

import { samplePlayback, type PlaybackSample } from "../playback";
import {
  applySample,
  linkKey,
  playbackRig,
  signalAt,
  styleFlash,
  styleSpray,
} from "../playbackRig";
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
    updateSprays(s, sample, t);
    updateTraces(s, sample, t, Math.min(delta, 0.25));
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

/** Poses every declared spray cone at time `t`: at the bound robot's TCP,
 * along its -Z, while the effect's signal is on. */
function updateSprays(s: StudioState, sample: PlaybackSample, t: number) {
  if (playbackRig.sprays.size === 0) return;
  const signals = s.timeline?.signals ?? [];
  for (const spray of s.flashes) {
    if (spray.kind !== "spray") continue;
    const node = playbackRig.sprays.get(spray.name);
    if (!node) continue;
    const lane = signals.find((sig: { name: string }) => sig.name === spray.signal);
    const poses = sample.poses?.[spray.robot];
    const robot = robotByName(s.robots, spray.robot);
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
    styleSpray(node, poses[tcpIndex], t);
  }
}

const Z_AXIS = new THREE.Vector3(0, 0, 1);
const SPIN_QUAT = new THREE.Quaternion();

/** Appends the TCP position to every cut trace whose signal is on at `t`
 * (the trail restarts when the playhead jumps backward) and spins the
 * bound cutter link — a visible strobe, not a model of 18k rpm. */
function updateTraces(
  s: StudioState,
  sample: PlaybackSample,
  t: number,
  delta: number,
) {
  if (playbackRig.traces.size === 0) return;
  const signals = s.timeline?.signals ?? [];
  for (const trace of s.flashes) {
    if (trace.kind !== "trace") continue;
    const handle = playbackRig.traces.get(trace.name);
    if (!handle) continue;
    if (t < handle.lastT - 0.05) {
      handle.positions.length = 0;
      handle.line.visible = false;
    }
    handle.lastT = t;
    const lane = signals.find((sig: { name: string }) => sig.name === trace.signal);
    const poses = sample.poses?.[trace.robot];
    const robot = robotByName(s.robots, trace.robot);
    const tcpName = robot?.desc.tcp_link ?? null;
    const tcpIndex =
      tcpName && robot
        ? robot.desc.links.findIndex((l) => l.name === tcpName)
        : -1;
    const on =
      !!lane && !!poses && tcpIndex >= 0 && signalAt(lane.times, lane.values, t);
    if (!on || !poses) continue;
    const p = poses[tcpIndex].position;
    const n = handle.positions.length;
    const moved =
      n < 3 ||
      (p[0] - handle.positions[n - 3]) ** 2 +
        (p[1] - handle.positions[n - 2]) ** 2 +
        (p[2] - handle.positions[n - 1]) ** 2 >
        1e-7;
    if (moved) {
      handle.positions.push(p[0], p[1], p[2]);
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute(
        "position",
        new THREE.Float32BufferAttribute(handle.positions.slice(), 3),
      );
      handle.line.geometry.dispose();
      handle.line.geometry = geometry;
      handle.line.visible = handle.positions.length >= 6;
    }
    if (trace.spin_link && robot) {
      const linkIndex = robot.desc.links.findIndex(
        (l) => l.name === trace.spin_link,
      );
      const link =
        linkIndex >= 0
          ? playbackRig.links.get(linkKey(trace.robot, linkIndex))
          : null;
      if (link) {
        // applySample rewrote the link pose this frame; append the
        // accumulated spin about the link's own Z (the tool axis).
        handle.spinAngle = (handle.spinAngle + delta * 45) % (Math.PI * 2);
        link.quaternion.multiply(
          SPIN_QUAT.setFromAxisAngle(Z_AXIS, handle.spinAngle),
        );
      }
    }
  }
}


// The studio's bake requests, and the physics toggle over them.
//
// A bake is remembered as the request that made it, so the toggle can
// send the very same request again with the physics bit flipped and the
// result restart playback from the top (design-world-physics.md §3.6).
// What "physics on" means — the whole cell, its ground, whether the
// robots' motors are on — is the host's business (`bt.studio(scene,
// physics=)`); the studio only sends a bit.

import { isWasmMode } from "./backend";
import { useStudioStore, type BakeRequest } from "./store";
import {
  sendSimulatePhysics,
  sendSimulateSequence,
  sendSimulateSequences,
  sendStartBake,
  sendStopBake,
} from "./ws";

/** Seconds a program-less physics bake runs where it cannot stream (the
 * browser-only session has no thread to stream from): enough for anything
 * unsupported to land, and for a machine with no power to fold and rest. */
export const PHYSICS_SETTLE_SECONDS = 10;

/** Whether a request bakes under the host's physics. */
export function underPhysics(req: BakeRequest): boolean {
  return req.kind === "physics" || req.physics;
}

/** Records the request and sends it. A kinematic bake is a batch: its
 * `sequence_result` restarts playback (or reports why it failed). A
 * physics bake *streams* from a host with a thread — the tracks land as
 * they grow and playback follows, the programs' steps banding the dock as
 * they happen — so a slow or long bake is watched, not waited for; the
 * browser-only session has no thread and bakes it in one go. */
export function startBake(req: BakeRequest): void {
  const s = useStudioStore.getState();
  if (underPhysics(req) && !isWasmMode()) {
    s.beginBakeStream(req);
    if (req.kind === "physics") {
      sendStartBake([], req.scenario, undefined, true);
    } else {
      sendStartBake(req.names, req.scenario, req.cap, true);
    }
    return;
  }
  s.beginBake(req);
  if (req.kind === "physics") {
    sendSimulatePhysics(req.duration, req.scenario);
  } else if (req.names.length === 1) {
    sendSimulateSequence(req.names[0], req.scenario, req.cap, req.physics);
  } else {
    sendSimulateSequences(req.names, req.scenario, req.cap, req.physics);
  }
}

/** The physics toggle. With a program baked: the same bake again under
 * physics (on, streamed) or kinematically (off, from the top) — off while
 * the physics stream runs stops it where it stands first. With no
 * program: on streams the world under gravity until off stops it where it
 * stands (the clip stays on the dock) — the cap is the programs' and does
 * not end it; off with the stream already over puts the cell back as
 * authored. */
export function setPhysics(on: boolean): void {
  const s = useStudioStore.getState();
  const last = s.bakeStream?.request ?? s.lastBake;
  if (!on && s.bakeStream) {
    s.setPhysicsOn(false);
    sendStopBake();
    if (last && last.kind === "sequences") {
      startBake({ ...last, physics: false });
    }
    return;
  }
  if (last && last.kind === "sequences") {
    s.setPhysicsOn(on);
    startBake({ ...last, physics: on });
    return;
  }
  if (on) {
    s.setPhysicsOn(true);
    startBake({
      kind: "physics",
      duration: PHYSICS_SETTLE_SECONDS,
      scenario: last?.scenario,
    });
  } else {
    s.setPhysicsOn(false);
    s.clearBake();
  }
}

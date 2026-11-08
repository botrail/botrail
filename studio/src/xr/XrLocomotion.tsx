import { useEffect, useMemo, useRef } from "react";
import { useFrame } from "@react-three/fiber";
import { useXRInputSourceState } from "@react-three/xr";
import * as THREE from "three";

import { xrActions } from "./actions";
import { AxisLatch, ButtonEdges, THUMBSTICK_AXES, XR_STANDARD, readPad } from "./gamepad";
import {
  ARC,
  type Arc,
  SNAP_TURN,
  applyPlacement,
  snapTurnPlacement,
  teleportArc,
  teleportPlacement,
  yawOf,
} from "./locomotion";
import { type Hand, headPose, headWorld, xrRig } from "./rig";

const HANDS: readonly Hand[] = ["left", "right"];
const VALID = new THREE.Color("#5fd08a");
const INVALID = new THREE.Color("#e0574f");

/**
 * The controllers' sticks and buttons, polled once per XR frame
 * (`.internal/design-vr.md` §3.4, §3.6):
 *
 * - either thumbstick pushed forward throws a teleport arc from that hand;
 *   letting go lands there (green ring) or does nothing (red);
 * - a thumbstick flicked sideways snap-turns 45° about the head;
 * - right A plays / pauses the bake, right B cycles the speed, left X
 *   recenters on the launch stance, left Y teaches the pose as a waypoint.
 *
 * The arc and its ring are world-space objects (a sibling of the origin),
 * invisible to the ray pointer. Nothing here runs on the desktop: the
 * frame callback leaves at once when no session presents.
 */
export function XrLocomotion() {
  const left = useXRInputSourceState("controller", "left");
  const right = useXRInputSourceState("controller", "right");
  const detectors = useRef({ left: makeDetectors(), right: makeDetectors() });
  const preview = useRef<{ hand: Hand; arc: Arc } | null>(null);
  const arc = useMemo(() => makeArcObjects(), []);
  useEffect(() => () => arc.dispose(), [arc]);

  useFrame(({ gl, camera }) => {
    const origin = xrRig.origin;
    const head = headPose(camera);
    if (!gl.xr.isPresenting || !origin || !head) {
      if (arc.group.visible) hide();
      return;
    }
    let aiming: Hand | null = null;
    for (const hand of HANDS) {
      const state = hand === "left" ? left : right;
      const pad = readPad(state?.inputSource.gamepad);
      const d = detectors.current[hand];
      for (const i of d.buttons.update(pad.pressed)) {
        if (i === XR_STANDARD.primary) {
          if (hand === "right") xrActions.togglePlay();
          else xrActions.recenter();
        } else if (i === XR_STANDARD.secondary) {
          if (hand === "right") xrActions.cycleSpeed();
          else xrActions.addWaypoint();
        }
      }
      const x = pad.axes[THUMBSTICK_AXES.x] ?? 0;
      const y = pad.axes[THUMBSTICK_AXES.y] ?? 0;
      const side = d.side.feed(x);
      if (side.engaged) {
        // Stick to the right turns right: clockwise seen from above is a
        // negative heading.
        const pivot = headWorld();
        if (pivot) {
          applyPlacement(
            origin,
            snapTurnPlacement(pivot, yawOf(origin.quaternion), -side.engaged * SNAP_TURN, head),
          );
        }
      }
      const forward = d.forward.feed(-y);
      if (forward.held > 0) aiming = hand;
      if (forward.released > 0) {
        const p = preview.current;
        if (p && p.hand === hand) {
          if (p.arc.hit && p.arc.valid) {
            applyPlacement(origin, teleportPlacement(p.arc.hit, yawOf(origin.quaternion), head));
          }
          preview.current = null;
        }
      }
    }
    if (aiming) {
      const ray = xrRig.rays[aiming];
      if (ray) {
        ray.updateWorldMatrix(true, false);
        const from = new THREE.Vector3().setFromMatrixPosition(ray.matrixWorld);
        const direction = new THREE.Vector3(0, 0, -1).transformDirection(ray.matrixWorld);
        const thrown = teleportArc(from, direction);
        preview.current = { hand: aiming, arc: thrown };
        show(thrown);
        xrRig.teleport = { hand: aiming, hit: thrown.hit?.toArray() ?? null, valid: thrown.valid };
      }
    } else if (arc.group.visible) {
      hide();
    }
  });

  function show(thrown: Arc) {
    const n = thrown.points.length;
    const positions = arc.line.geometry.getAttribute("position") as THREE.BufferAttribute;
    for (let i = 0; i < positions.count; i++) {
      const p = thrown.points[Math.min(i, n - 1)] ?? new THREE.Vector3();
      positions.setXYZ(i, p.x, p.y, p.z);
    }
    positions.needsUpdate = true;
    arc.line.geometry.setDrawRange(0, n);
    const color = thrown.valid ? VALID : INVALID;
    arc.lineMaterial.color.copy(color);
    arc.ringMaterial.color.copy(color);
    arc.ring.visible = thrown.hit !== null;
    if (thrown.hit) arc.ring.position.copy(thrown.hit).setZ(0.005);
    arc.group.visible = n > 0;
  }

  function hide() {
    arc.group.visible = false;
    xrRig.teleport = { hand: null, hit: null, valid: false };
  }

  return <primitive object={arc.group} />;
}

function makeDetectors() {
  return { buttons: new ButtonEdges(), forward: new AxisLatch(), side: new AxisLatch() };
}

function makeArcObjects() {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    "position",
    new THREE.BufferAttribute(new Float32Array((ARC.steps + 1) * 3), 3),
  );
  const lineMaterial = new THREE.LineBasicMaterial({
    color: VALID,
    depthTest: false,
    transparent: true,
    opacity: 0.9,
    toneMapped: false,
  });
  const line = new THREE.Line(geometry, lineMaterial);
  line.frustumCulled = false;
  const ringMaterial = new THREE.MeshBasicMaterial({
    color: VALID,
    depthTest: false,
    transparent: true,
    opacity: 0.8,
    side: THREE.DoubleSide,
    toneMapped: false,
  });
  // The ring lies in XY already: the floor plane of a Z-up world.
  const ring = new THREE.Mesh(new THREE.RingGeometry(0.17, 0.24, 40), ringMaterial);
  const group = new THREE.Group();
  group.add(line, ring);
  group.visible = false;
  group.renderOrder = 90;
  for (const o of [line, ring]) {
    o.renderOrder = 90;
    o.raycast = () => {};
  }
  return {
    group,
    line,
    lineMaterial,
    ring,
    ringMaterial,
    dispose() {
      geometry.dispose();
      lineMaterial.dispose();
      ring.geometry.dispose();
      ringMaterial.dispose();
    },
  };
}

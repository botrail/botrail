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
  floorHeading,
  glideStep,
  snapTurnPlacement,
  stickDemand,
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
 * - the left thumbstick glides: it walks the player across the floor the
 *   way the head faces — forward, back and sideways, as fast as the stick
 *   is pushed, eased in and out;
 * - the right thumbstick pushed forward throws a teleport arc from that
 *   hand; letting go lands there (green ring) or does nothing (red);
 * - the right thumbstick flicked sideways snap-turns 45° about the head;
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
  const input = useRef({
    buttons: { left: new ButtonEdges(), right: new ButtonEdges() },
    aim: new AxisLatch(),
    turn: new AxisLatch(),
    /** The glide's velocity, world XY, m/s. */
    velocity: { x: 0, y: 0 },
    /** When the previous frame was shown (ms), where the frame says. */
    shown: null as number | null,
  });
  const preview = useRef<Arc | null>(null);
  const arc = useMemo(() => makeArcObjects(), []);
  useEffect(() => () => arc.dispose(), [arc]);

  useFrame(({ gl, camera }, delta, frame) => {
    const io = input.current;
    const origin = xrRig.origin;
    const head = headPose(camera);
    if (!gl.xr.isPresenting || !origin || !head) {
      if (arc.group.visible) hide();
      io.velocity = { x: 0, y: 0 };
      io.shown = null;
      return;
    }
    // The frame's length on the display's own clock where the frame
    // carries it: those intervals are the refresh period, while the render
    // loop's delta has the scheduler's jitter in it — and a glide shows
    // every millisecond of that as judder.
    let dt = delta;
    const shown = (frame as { predictedDisplayTime?: number } | undefined)?.predictedDisplayTime;
    if (typeof shown === "number") {
      if (io.shown !== null) dt = (shown - io.shown) / 1000;
      io.shown = shown;
    }

    const pads = {
      left: readPad(left?.inputSource.gamepad),
      right: readPad(right?.inputSource.gamepad),
    };
    for (const hand of HANDS) {
      for (const i of io.buttons[hand].update(pads[hand].pressed)) {
        if (i === XR_STANDARD.primary) {
          if (hand === "right") xrActions.togglePlay();
          else xrActions.recenter();
        } else if (i === XR_STANDARD.secondary) {
          if (hand === "right") xrActions.cycleSpeed();
          else xrActions.addWaypoint();
        }
      }
    }

    // Right stick: turn and teleport.
    const turn = io.turn.feed(pads.right.axes[THUMBSTICK_AXES.x] ?? 0);
    if (turn.engaged) {
      // Stick to the right turns right: clockwise seen from above is a
      // negative heading.
      const pivot = headWorld();
      if (pivot) {
        applyPlacement(
          origin,
          snapTurnPlacement(pivot, yawOf(origin.quaternion), -turn.engaged * SNAP_TURN, head),
        );
      }
    }
    const aim = io.aim.feed(-(pads.right.axes[THUMBSTICK_AXES.y] ?? 0));
    if (aim.released > 0) {
      const thrown = preview.current;
      if (thrown?.hit && thrown.valid) {
        applyPlacement(origin, teleportPlacement(thrown.hit, yawOf(origin.quaternion), head));
      }
      preview.current = null;
    }

    // Left stick: glide. The origin slides; the heading is read off the
    // head every frame, so looking somewhere else steers.
    const demand = stickDemand(
      pads.left.axes[THUMBSTICK_AXES.x] ?? 0,
      -(pads.left.axes[THUMBSTICK_AXES.y] ?? 0),
    );
    origin.updateWorldMatrix(true, false);
    const heading = floorHeading(
      origin.getWorldQuaternion(new THREE.Quaternion()).multiply(head.quaternion),
    );
    const moving = io.velocity.x !== 0 || io.velocity.y !== 0;
    if (moving || demand.x !== 0 || demand.y !== 0) {
      const at = origin.localToWorld(head.position.clone());
      const { velocity, step } = glideStep(io.velocity, demand, heading, dt, at);
      io.velocity = velocity;
      origin.position.x += step.x;
      origin.position.y += step.y;
      origin.updateMatrixWorld(true);
    }
    xrRig.glide = {
      velocity: [io.velocity.x, io.velocity.y],
      heading: heading ? [heading.x, heading.y] : null,
    };

    if (aim.held > 0) {
      const ray = xrRig.rays.right;
      if (ray) {
        ray.updateWorldMatrix(true, false);
        const from = new THREE.Vector3().setFromMatrixPosition(ray.matrixWorld);
        const direction = new THREE.Vector3(0, 0, -1).transformDirection(ray.matrixWorld);
        const thrown = teleportArc(from, direction);
        preview.current = thrown;
        show(thrown);
        xrRig.teleport = { hand: "right", hit: thrown.hit?.toArray() ?? null, valid: thrown.valid };
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

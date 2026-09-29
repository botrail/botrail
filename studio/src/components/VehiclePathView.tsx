import { useMemo } from "react";
import { Html, Line } from "@react-three/drei";
import * as THREE from "three";

import type { DeviceMsg } from "../protocol";
import { useStudioStore } from "../store";

const PATH_COLOR = "#4a7ab8";
const MOVING_COLOR = "#e0a03c";
const FLOOR_LIFT = 0.012;

/** Is `name`'s lane ON at `t` in the loaded timeline? */
function laneActiveAt(
  timeline: { signals: { name: string; times: number[]; values: boolean[] }[] } | null,
  name: string,
  t: number,
): boolean {
  const lane = timeline?.signals.find((s) => s.name === name);
  if (!lane) return false;
  let value = false;
  for (let i = 0; i < lane.times.length && lane.times[i] <= t + 1e-9; i++) {
    value = lane.values[i];
  }
  return value;
}

/**
 * Renders vehicle guide paths: the tape on the floor as a polyline, with a
 * disc and a name chip at each station. While the timeline plays a
 * travelling vehicle's path glows amber (its moving lane at the playhead).
 */
export function VehiclePathView() {
  const devices = useStudioStore((s) => s.devices);
  const timeline = useStudioStore((s) => s.timeline);
  const playbackTime = useStudioStore((s) => s.playbackTime);
  const vehicles = devices.filter((d) => d.kind.kind === "vehicle");

  if (vehicles.length === 0) return null;
  return (
    <>
      {vehicles.map((device) => (
        <GuidePath
          key={device.name}
          device={device}
          moving={timeline !== null && laneActiveAt(timeline, device.name, playbackTime)}
        />
      ))}
    </>
  );
}

/**
 * A steered drive's tape rounds its corners: the fillet arc of `radius`
 * tangent to both legs at every waypoint that is no station, when the legs
 * beside it have room (the TS mirror of `seq::filleted_path`).
 */
function filleted(
  waypoints: [number, number, number][],
  stations: { index: number }[],
  ring: boolean,
  radius: number,
): [number, number, number][] {
  const n = waypoints.length;
  const out: [number, number, number][] = [];
  for (let i = 0; i < n; i++) {
    const [px, py, pz] = waypoints[i];
    const station = stations.some((s) => s.index === i);
    const prev = ring && n > 2 ? (i + n - 1) % n : i > 0 ? i - 1 : -1;
    const next = ring && n > 2 ? (i + 1) % n : i + 1 < n ? i + 1 : -1;
    if (prev < 0 || next < 0 || station) {
      out.push([px, py, pz]);
      continue;
    }
    const [ax, ay] = waypoints[prev];
    const [bx, by] = waypoints[next];
    const la = Math.hypot(px - ax, py - ay);
    const lb = Math.hypot(bx - px, by - py);
    if (la < 1e-9 || lb < 1e-9) {
      out.push([px, py, pz]);
      continue;
    }
    const ux = (px - ax) / la, uy = (py - ay) / la;
    const vx = (bx - px) / lb, vy = (by - py) / lb;
    const delta = Math.atan2(ux * vy - uy * vx, ux * vx + uy * vy);
    const tangent = radius * Math.tan(Math.abs(delta) / 2);
    if (Math.abs(delta) < 1e-6 || tangent > la / 2 || tangent > lb / 2) {
      out.push([px, py, pz]);
      continue;
    }
    const sx = px - ux * tangent, sy = py - uy * tangent;
    const sign = Math.sign(delta);
    const cx = sx - uy * radius * sign, cy = sy + ux * radius * sign;
    const a0 = Math.atan2(sy - cy, sx - cx);
    for (let j = 0; j <= 8; j++) {
      const ang = a0 + (delta * j) / 8;
      out.push([cx + radius * Math.cos(ang), cy + radius * Math.sin(ang), pz]);
    }
  }
  return out;
}

function GuidePath({ device, moving }: { device: DeviceMsg; moving: boolean }) {
  const kind = device.kind;
  const points = useMemo(() => {
    if (kind.kind !== "vehicle") return [];
    const radius = kind.turn_radius ?? null;
    const tape = radius
      ? filleted(kind.path.waypoints, kind.path.stations, kind.path.ring, radius)
      : kind.path.waypoints;
    const pts = tape.map(([x, y, z]) => new THREE.Vector3(x, y, (z ?? 0) + FLOOR_LIFT));
    if (kind.path.ring && pts.length > 1) pts.push(pts[0].clone());
    return pts;
  }, [kind]);
  if (kind.kind !== "vehicle" || points.length < 2) return null;
  const color = moving ? MOVING_COLOR : PATH_COLOR;
  return (
    <group>
      <Line points={points} color={color} lineWidth={2} dashed dashSize={0.08} gapSize={0.05} />
      {kind.path.stations.map((station) => {
        const wp = kind.path.waypoints[station.index];
        if (!wp) return null;
        return (
          <group
            key={station.name}
            position={[wp[0], wp[1], (wp[2] ?? 0) + FLOOR_LIFT]}
          >
            <mesh>
              <circleGeometry args={[0.09, 24]} />
              <meshBasicMaterial color={color} transparent opacity={0.5} />
            </mesh>
            <mesh>
              <ringGeometry args={[0.09, 0.11, 24]} />
              <meshBasicMaterial color={color} />
            </mesh>
            <Html center distanceFactor={6} style={{ pointerEvents: "none" }}>
              <div
                style={{
                  padding: "1px 6px",
                  borderRadius: 4,
                  background: "rgba(20, 24, 32, 0.75)",
                  color: "#cdd6e4",
                  fontSize: 11,
                  whiteSpace: "nowrap",
                  transform: "translateY(-16px)",
                }}
              >
                {station.name}
              </div>
            </Html>
          </group>
        );
      })}
    </group>
  );
}

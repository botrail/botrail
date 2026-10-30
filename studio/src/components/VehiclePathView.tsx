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

type XY = [number, number];

/**
 * The headings a vehicle may stand at on waypoint `at`: along the leg
 * leaving it and along the leg arriving at it (the TS mirror of
 * `VehiclePath::heading_at`, both ways) — a station passed through faces
 * the one way when it arrives and the other when it leaves.
 */
function stationHeadings(
  waypoints: [number, number, number][],
  ring: boolean,
  at: number,
): { leaving: number | null; arriving: number | null } {
  const n = waypoints.length;
  const dir = (i: number, j: number): number | null => {
    const dx = waypoints[j][0] - waypoints[i][0];
    const dy = waypoints[j][1] - waypoints[i][1];
    return Math.hypot(dx, dy) > 1e-9 ? Math.atan2(dy, dx) : null;
  };
  let leaving: number | null = null;
  let arriving: number | null = null;
  for (let step = 1; step < n && leaving === null; step++) {
    const j = ring ? (at + step) % n : at + step;
    if (j >= n) break;
    leaving = dir(at, j);
  }
  for (let step = 1; step < n && arriving === null; step++) {
    const j = ring ? (at + n - step) % n : at - step;
    if (j < 0) break;
    arriving = dir(j, at);
  }
  return { leaving, arriving };
}

/**
 * Where each formation member stands at each of its formation's stations —
 * the station's waypoint, its slot turned by the heading there. A member's
 * own path holds these points where the formation sets it down
 * (`Scene.formation_slot`); the leg between two of them is ridden, not
 * driven, so it is no tape on the floor.
 */
function riddenPoints(vehicles: DeviceMsg[]): Map<string, XY[]> {
  const points = new Map<string, XY[]>();
  for (const device of vehicles) {
    const kind = device.kind;
    if (kind.kind !== "vehicle" || !kind.members?.length) continue;
    const { waypoints, ring } = kind.path;
    for (const station of kind.path.stations) {
      const wp = waypoints[station.index];
      if (!wp) continue;
      const { leaving, arriving } = stationHeadings(waypoints, ring, station.index);
      for (const heading of [leaving, arriving]) {
        if (heading === null) continue;
        const c = Math.cos(heading), s = Math.sin(heading);
        for (const member of kind.members) {
          const [x, y] = member.slot.position;
          const list = points.get(member.vehicle) ?? [];
          list.push([wp[0] + c * x - s * y, wp[1] + s * x + c * y]);
          points.set(member.vehicle, list);
        }
      }
    }
  }
  return points;
}

/**
 * Renders vehicle guide paths: the tape on the floor as a polyline, with a
 * disc and a name chip at each station. While the timeline plays a
 * travelling vehicle's path glows amber (its moving lane at the playhead).
 * A formation also shows where its members stand at each station.
 */
export function VehiclePathView() {
  const devices = useStudioStore((s) => s.devices);
  const timeline = useStudioStore((s) => s.timeline);
  const playbackTime = useStudioStore((s) => s.playbackTime);
  const vehicles = useMemo(() => devices.filter((d) => d.kind.kind === "vehicle"), [devices]);
  const ridden = useMemo(() => riddenPoints(vehicles), [vehicles]);

  if (vehicles.length === 0) return null;
  return (
    <>
      {vehicles.map((device) => (
        <GuidePath
          key={device.name}
          device={device}
          ridden={ridden.get(device.name) ?? []}
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

function GuidePath({
  device,
  ridden,
  moving,
}: {
  device: DeviceMsg;
  ridden: XY[];
  moving: boolean;
}) {
  const kind = device.kind;
  // The tape in runs: a leg the formation carries the vehicle along breaks it.
  const runs = useMemo(() => {
    if (kind.kind !== "vehicle") return [];
    const radius = kind.turn_radius ?? null;
    const tape = radius
      ? filleted(kind.path.waypoints, kind.path.stations, kind.path.ring, radius)
      : kind.path.waypoints;
    const pts = tape.map(([x, y, z]) => new THREE.Vector3(x, y, (z ?? 0) + FLOOR_LIFT));
    if (kind.path.ring && pts.length > 1) pts.push(pts[0].clone());
    const onSlot = (p: THREE.Vector3) => ridden.some(([x, y]) => Math.hypot(p.x - x, p.y - y) < 0.01);
    const out: THREE.Vector3[][] = [];
    let run: THREE.Vector3[] = [];
    for (let i = 0; i < pts.length; i++) {
      if (i > 0 && onSlot(pts[i - 1]) && onSlot(pts[i])) {
        if (run.length > 1) out.push(run);
        run = [];
      }
      run.push(pts[i]);
    }
    if (run.length > 1) out.push(run);
    return out;
  }, [kind, ridden]);
  // A formation's members at each station: a spoke from the station to each slot.
  const spokes = useMemo(() => {
    if (kind.kind !== "vehicle" || !kind.members?.length) return [];
    const { waypoints, ring } = kind.path;
    const out: { key: string; from: THREE.Vector3; to: THREE.Vector3 }[] = [];
    for (const station of kind.path.stations) {
      const wp = waypoints[station.index];
      if (!wp) continue;
      // The heading it stands at there: the start faces the leg leaving
      // it (where a bake parks it), any other station the leg arriving.
      const { leaving, arriving } = stationHeadings(waypoints, ring, station.index);
      const heading = station.name === kind.start ? (leaving ?? arriving) : (arriving ?? leaving);
      if (heading === null) continue;
      const c = Math.cos(heading), s = Math.sin(heading);
      const z = (wp[2] ?? 0) + FLOOR_LIFT;
      for (const member of kind.members) {
        const [x, y] = member.slot.position;
        out.push({
          key: `${station.name}/${member.vehicle}`,
          from: new THREE.Vector3(wp[0], wp[1], z),
          to: new THREE.Vector3(wp[0] + c * x - s * y, wp[1] + s * x + c * y, z),
        });
      }
    }
    return out;
  }, [kind]);
  if (kind.kind !== "vehicle" || runs.length === 0) return null;
  const color = moving ? MOVING_COLOR : PATH_COLOR;
  return (
    <group>
      {runs.map((points, i) => (
        <Line
          key={i}
          points={points}
          color={color}
          lineWidth={2}
          dashed
          dashSize={0.08}
          gapSize={0.05}
        />
      ))}
      {spokes.map(({ key, from, to }) => (
        <group key={key}>
          <Line points={[from, to]} color={color} lineWidth={1} transparent opacity={0.55} />
          <mesh position={to}>
            <ringGeometry args={[0.07, 0.09, 20]} />
            <meshBasicMaterial color={color} transparent opacity={0.8} />
          </mesh>
        </group>
      ))}
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

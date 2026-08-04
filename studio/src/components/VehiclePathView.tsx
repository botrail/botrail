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

function GuidePath({ device, moving }: { device: DeviceMsg; moving: boolean }) {
  const kind = device.kind;
  const points = useMemo(() => {
    if (kind.kind !== "vehicle") return [];
    const pts = kind.path.waypoints.map(
      ([x, y]) => new THREE.Vector3(x, y, FLOOR_LIFT),
    );
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
          <group key={station.name} position={[wp[0], wp[1], FLOOR_LIFT]}>
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

import { useMemo } from "react";
import * as THREE from "three";

import type { SensorMsg } from "../protocol";
import { useStudioStore } from "../store";

const IDLE_COLOR = "#b8a24a";
const ACTIVE_COLOR = "#e0503c";

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
 * Renders pseudo-sensors: zones as translucent boxes, beams as thin rods.
 * During timeline playback a tripped sensor glows red (its input lane
 * sampled at the playhead).
 */
export function SensorView() {
  const sensors = useStudioStore((s) => s.sensors);
  const timeline = useStudioStore((s) => s.timeline);
  const playbackTime = useStudioStore((s) => s.playbackTime);
  const playingTimeline = timeline !== null;

  if (sensors.length === 0) return null;
  return (
    <>
      {sensors.map((sensor) => (
        <SensorShape
          key={sensor.name}
          sensor={sensor}
          active={playingTimeline && laneActiveAt(timeline, sensor.name, playbackTime)}
        />
      ))}
    </>
  );
}

function SensorShape({ sensor, active }: { sensor: SensorMsg; active: boolean }) {
  const color = active ? ACTIVE_COLOR : IDLE_COLOR;
  if (sensor.kind.kind === "zone") {
    const { pose, size } = sensor.kind;
    return (
      <group
        position={pose.position}
        quaternion={new THREE.Quaternion(...pose.quaternion)}
      >
        <mesh>
          <boxGeometry args={size} />
          <meshBasicMaterial color={color} transparent opacity={active ? 0.3 : 0.12} />
        </mesh>
      </group>
    );
  }
  return <BeamRod sensor={sensor} color={color} active={active} />;
}

function BeamRod({
  sensor,
  color,
  active,
}: {
  sensor: SensorMsg;
  color: string;
  active: boolean;
}) {
  const kind = sensor.kind;
  const placement = useMemo(() => {
    if (kind.kind !== "beam") return null;
    const a = new THREE.Vector3(...kind.from);
    const b = new THREE.Vector3(...kind.to);
    const mid = a.clone().add(b).multiplyScalar(0.5);
    const dir = b.clone().sub(a);
    const length = dir.length();
    // CylinderGeometry extends along +Y; rotate it onto the beam axis.
    const quat = new THREE.Quaternion().setFromUnitVectors(
      new THREE.Vector3(0, 1, 0),
      dir.normalize(),
    );
    return { mid, quat, length };
  }, [kind]);
  if (!placement || kind.kind !== "beam") return null;
  return (
    <mesh position={placement.mid} quaternion={placement.quat}>
      <cylinderGeometry
        args={[kind.radius, kind.radius, placement.length, 8]}
      />
      <meshBasicMaterial color={color} transparent opacity={active ? 0.9 : 0.45} />
    </mesh>
  );
}

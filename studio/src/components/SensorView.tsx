import { useMemo } from "react";
import * as THREE from "three";

import type { DeviceMsg, PoseMsg, SensorMsg } from "../protocol";
import { playbackRig } from "../playbackRig";
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

interface Frame {
  position: THREE.Vector3;
  quaternion: THREE.Quaternion;
}

/**
 * The parked reference frame of a vehicle device: its start station's
 * waypoint plus the heading there — along the leg leaving it, or the leg
 * arriving when nothing leaves (the TS mirror of `VehiclePath::frame_at`).
 */
function parkedFrame(device: DeviceMsg): Frame | null {
  const kind = device.kind;
  if (kind.kind !== "vehicle") return null;
  const { waypoints, stations, ring } = kind.path;
  const n = waypoints.length;
  const at = stations.find((s) => s.name === kind.start)?.index ?? 0;
  const wp = waypoints[at];
  if (!wp) return null;
  const dir = (i: number, j: number): number | null => {
    const a = waypoints[i];
    const b = waypoints[j];
    if (!a || !b) return null;
    const dx = b[0] - a[0];
    const dy = b[1] - a[1];
    return Math.hypot(dx, dy) > 1e-9 ? Math.atan2(dy, dx) : null;
  };
  let heading = 0;
  let found = false;
  for (let step = 1; step < n && !found; step++) {
    const j = ring ? (at + step) % n : at + step;
    if (!ring && j >= n) break;
    const h = dir(at, j);
    if (h !== null) {
      heading = h;
      found = true;
    }
  }
  for (let step = 1; step < n && !found; step++) {
    const j = ring ? (at + n - (step % n)) % n : at - step;
    if (!ring && j < 0) break;
    const h = dir(j, at);
    if (h !== null) {
      heading = h;
      found = true;
    }
  }
  return {
    position: new THREE.Vector3(wp[0], wp[1], wp[2] ?? 0),
    quaternion: new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(0, 0, 1),
      heading,
    ),
  };
}

/** Where a mounted sensor's vehicle frame is right now: the playback
 * override while a timeline plays, the parked frame otherwise. `null`
 * for a floor fixture. */
function mountFrame(
  sensor: SensorMsg,
  devices: DeviceMsg[],
  overrides: Record<string, PoseMsg> | null,
): Frame | null {
  if (!sensor.mount) return null;
  const pose = overrides?.[sensor.mount];
  if (pose) {
    return {
      position: new THREE.Vector3(...pose.position),
      quaternion: new THREE.Quaternion(...pose.quaternion),
    };
  }
  const device = devices.find((d) => d.name === sensor.mount);
  return device ? parkedFrame(device) : null;
}

/**
 * Renders pseudo-sensors: zones as translucent boxes, beams as thin rods.
 * A sensor mounted on a vehicle is authored in the vehicle's frame, so it
 * is drawn under that frame — riding the playback track while a timeline
 * plays, parked at the start station otherwise. During timeline playback
 * a tripped sensor glows red (its input lane sampled at the playhead).
 */
export function SensorView() {
  const sensors = useStudioStore((s) => s.sensors);
  const devices = useStudioStore((s) => s.devices);
  const overrideVehiclePoses = useStudioStore((s) => s.overrideVehiclePoses);
  const timeline = useStudioStore((s) => s.timeline);
  const playbackTime = useStudioStore((s) => s.playbackTime);
  const playingTimeline = timeline !== null;

  if (sensors.length === 0) return null;
  return (
    <>
      {sensors.map((sensor) => {
        const mount = mountFrame(sensor, devices, overrideVehiclePoses);
        const shape = (
          <SensorShape
            sensor={sensor}
            active={playingTimeline && laneActiveAt(timeline, sensor.name, playbackTime)}
          />
        );
        return mount ? (
          <group
            key={sensor.name}
            position={mount.position}
            quaternion={mount.quaternion}
            // While a timeline plays, poses bypass the store: the driver
            // writes the sampled vehicle frame straight onto this group
            // (see `playbackRig.mounts`), like every other moving thing.
            ref={(node) => {
              if (node) {
                playbackRig.mounts.set(sensor.name, {
                  vehicle: sensor.mount as string,
                  node,
                });
              } else {
                playbackRig.mounts.delete(sensor.name);
              }
            }}
          >
            {shape}
          </group>
        ) : (
          <group key={sensor.name}>{shape}</group>
        );
      })}
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

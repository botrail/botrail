import { useEffect, useMemo, useRef, useState } from "react";
import { TransformControls } from "@react-three/drei";
import type { ThreeEvent } from "@react-three/fiber";
import * as THREE from "three";

import type { LidarMsg, PoseMsg } from "../protocol";
import { playbackRig } from "../playbackRig";
import { useStudioStore } from "../store";
import { sendUpsertLidarThrottled } from "../ws";
import { laneActiveAt, parkedFrame, type Frame } from "./SensorView";

const IDLE_COLOR = "#5fa8dc";
const SELECTED_COLOR = "#f0b429";
const ACTIVE_COLOR = "#e0503c";

/** Is any field sensor sweeping through `lidar` tripped at the playhead?
 * Turns the scanner sector red — the field's geometry is the scanner's. */
export function fieldActiveAt(
  s: {
    sensors: { name: string; kind: { kind: string; lidar?: string } }[];
    timeline: { signals: { name: string; times: number[]; values: boolean[] }[] } | null;
    playbackTime: number;
  },
  lidar: string,
): boolean {
  if (!s.timeline) return false;
  return s.sensors.some(
    (sensor) =>
      sensor.kind.kind === "field" &&
      sensor.kind.lidar === lidar &&
      laneActiveAt(s.timeline, sensor.name, s.playbackTime),
  );
}

/** An unselected scanner's sector is a compact aim gizmo; selecting it
 * draws the true sweep out to max range for coverage checks. */
const IDLE_REACH = 0.8;

/**
 * Renders LiDAR scanners: a small puck plus a wireframe scan sector in
 * the local XY plane (angle 0 along +X, CCW — the ROS laser frame),
 * whose span follows `fov_deg` and whose reach follows `range`. A world
 * scanner is placed directly and, while selected, gets a move/rotate
 * gizmo; a vehicle scanner rides its machine's frame (parked or played
 * back, like mounted sensors); a link scanner rides the sampled link
 * pose — live state while posing, `playbackRig.linkMounts` while a
 * timeline plays (the camera pattern, verbatim).
 */
export function LidarView() {
  const lidars = useStudioStore((s) => s.lidars);
  if (lidars.length === 0) return null;
  return (
    <>
      {lidars.map((lidar) => (
        <LidarNode key={lidar.name} lidar={lidar} />
      ))}
      {lidars.map((lidar) => (
        <ScanCloud key={`cloud:${lidar.name}`} name={lidar.name} />
      ))}
    </>
  );
}

const SCAN_CLOUD_COLOR = "#f0a94e";

/** One scanner's simulated-sweep overlay: the last `scan_lidar` reply's
 * returns as world-frame points. Rendered at the scene root — the
 * points already live in world coordinates, so they must not ride the
 * scanner's mount group. */
function ScanCloud({ name }: { name: string }) {
  const cloud = useStudioStore((s) => s.scanClouds[name]);
  const geometry = useMemo(() => {
    if (!cloud || cloud.length === 0) return null;
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(cloud, 3));
    return g;
  }, [cloud]);
  useEffect(() => () => geometry?.dispose(), [geometry]);
  if (!geometry) return null;
  return (
    <points geometry={geometry} frustumCulled={false}>
      <pointsMaterial
        color={SCAN_CLOUD_COLOR}
        size={0.03}
        sizeAttenuation
        depthWrite={false}
      />
    </points>
  );
}

function LidarNode({ lidar }: { lidar: LidarMsg }) {
  const devices = useStudioStore((s) => s.devices);
  const robots = useStudioStore((s) => s.robots);
  const overridePoses = useStudioStore((s) => s.overridePoses);
  const overrideVehiclePoses = useStudioStore((s) => s.overrideVehiclePoses);
  const selection = useStudioStore((s) => s.selection);
  const gizmoMode = useStudioStore((s) => s.gizmoMode);
  const selected = selection.type === "lidar" && selection.name === lidar.name;
  // A field tripping through this scanner turns the sector red at the
  // playhead — the field's geometry is the scanner's.
  const fieldActive = useStudioStore((s) => fieldActiveAt(s, lidar.name));

  const [group, setGroup] = useState<THREE.Group | null>(null);
  const draggingRef = useRef(false);
  const mount = lidar.mount;

  // Where the mount frame is right now (world mounts have none).
  let frame: Frame | null = null;
  let linkIndex = -1;
  if (mount.kind === "vehicle") {
    const pose = overrideVehiclePoses?.[mount.device];
    if (pose) {
      frame = {
        position: new THREE.Vector3(...pose.position),
        quaternion: new THREE.Quaternion(...pose.quaternion),
      };
    } else {
      const device = devices.find((d) => d.name === mount.device);
      frame = device ? parkedFrame(device) : null;
    }
  } else if (mount.kind === "link") {
    const robot = robots.find((r) => r.desc.name === mount.robot);
    linkIndex =
      robot?.desc.links.findIndex((l) => l.name === mount.link) ?? -1;
    const pose: PoseMsg | undefined =
      overridePoses?.[mount.robot]?.[linkIndex] ??
      robot?.linkPoses[linkIndex];
    if (pose) {
      frame = {
        position: new THREE.Vector3(...pose.position),
        quaternion: new THREE.Quaternion(...pose.quaternion),
      };
    }
  }

  // World scanners follow the store pose except while being dragged, so
  // the server's echo doesn't fight the gizmo (the obstacle pattern).
  useEffect(() => {
    if (!group || mount.kind !== "world" || draggingRef.current) return;
    group.position.set(...lidar.pose.position);
    group.quaternion.set(...lidar.pose.quaternion);
  }, [group, mount.kind, lidar.pose]);

  const onDrag = () => {
    if (!group || !draggingRef.current) return;
    sendUpsertLidarThrottled(lidar.name, {
      ...lidar,
      pose: {
        position: [group.position.x, group.position.y, group.position.z],
        quaternion: [
          group.quaternion.x,
          group.quaternion.y,
          group.quaternion.z,
          group.quaternion.w,
        ],
      },
    });
  };

  const onSelect = (e: ThreeEvent<MouseEvent>) => {
    e.stopPropagation();
    const s = useStudioStore.getState();
    s.selectLidar(lidar.name);
    s.focusTab("obstacle");
  };

  const shape = (
    <group
      position={lidar.pose.position}
      quaternion={new THREE.Quaternion(...lidar.pose.quaternion)}
    >
      <LidarShape
        lidar={lidar}
        selected={selected}
        active={fieldActive}
        onClick={onSelect}
      />
    </group>
  );

  if (mount.kind === "world") {
    return (
      <>
        <group ref={setGroup}>
          <LidarShape
            lidar={lidar}
            selected={selected}
            active={fieldActive}
            onClick={onSelect}
          />
        </group>
        {selected && group && (
          <TransformControls
            object={group}
            mode={gizmoMode}
            size={0.6}
            onMouseDown={() => {
              draggingRef.current = true;
            }}
            onMouseUp={() => {
              draggingRef.current = false;
            }}
            onObjectChange={onDrag}
          />
        )}
      </>
    );
  }

  if (!frame) return null;
  return (
    <group
      position={frame.position}
      quaternion={frame.quaternion}
      // While a timeline plays, the driver writes the mount frame straight
      // onto this group — the vehicle pose or the sampled link pose —
      // bypassing React (see `playbackRig`). Keys are prefixed so a lidar
      // and a sensor sharing a name cannot collide in the registries.
      ref={(node) => {
        const key = `lidar:${lidar.name}`;
        if (!node) {
          playbackRig.mounts.delete(key);
          playbackRig.linkMounts.delete(key);
        } else if (mount.kind === "vehicle") {
          playbackRig.mounts.set(key, { vehicle: mount.device, node });
        } else if (mount.kind === "link" && linkIndex >= 0) {
          playbackRig.linkMounts.set(key, {
            robot: mount.robot,
            link: linkIndex,
            node,
          });
        }
      }}
    >
      {shape}
    </group>
  );
}

/** Puck + sector lines, in the scanner's own frame (scan plane = XY). */
function LidarShape({
  lidar,
  selected,
  active,
  onClick,
}: {
  lidar: LidarMsg;
  selected: boolean;
  /** A field sweeping through this scanner is tripped. */
  active: boolean;
  onClick: (e: ThreeEvent<MouseEvent>) => void;
}) {
  const color = active ? ACTIVE_COLOR : selected ? SELECTED_COLOR : IDLE_COLOR;
  const sector = useMemo(
    () =>
      sectorGeometry(
        lidar,
        selected ? lidar.range[1] : Math.min(IDLE_REACH, lidar.range[1]),
        selected,
      ),
    [lidar, selected],
  );
  useEffect(() => () => sector.dispose(), [sector]);
  return (
    <group>
      {/* Body: a scanner puck whose dark band at z=0 is the exit window
          of the scan plane. */}
      <mesh position={[0, 0, -0.04]} rotation={[Math.PI / 2, 0, 0]} onClick={onClick}>
        <cylinderGeometry args={[0.05, 0.055, 0.06, 20]} />
        <meshStandardMaterial color={color} roughness={0.6} metalness={0.2} />
      </mesh>
      <mesh rotation={[Math.PI / 2, 0, 0]} onClick={onClick}>
        <cylinderGeometry args={[0.045, 0.045, 0.02, 20]} />
        <meshStandardMaterial color="#2a2e35" roughness={0.4} metalness={0.4} />
      </mesh>
      <mesh position={[0, 0, 0.017]} rotation={[Math.PI / 2, 0, 0]} onClick={onClick}>
        <cylinderGeometry args={[0.048, 0.045, 0.014, 20]} />
        <meshStandardMaterial color={color} roughness={0.6} metalness={0.2} />
      </mesh>
      <lineSegments geometry={sector}>
        <lineBasicMaterial
          color={color}
          transparent
          opacity={selected ? 0.9 : 0.55}
        />
      </lineSegments>
      <FieldSectors lidar={lidar} />
    </group>
  );
}

/** The fields sweeping through one scanner, as translucent fans in the
 * scan plane — the warning/protective rings of a real field set. Each
 * tints by its own lane at the playhead and clicks through to its
 * sensor form. */
function FieldSectors({ lidar }: { lidar: LidarMsg }) {
  const sensors = useStudioStore((s) => s.sensors);
  const fields = sensors.filter(
    (f) => f.kind.kind === "field" && f.kind.lidar === lidar.name,
  );
  return (
    <>
      {fields.map((field) => (
        <FieldFan key={field.name} lidar={lidar} field={field} />
      ))}
    </>
  );
}

function FieldFan({
  lidar,
  field,
}: {
  lidar: LidarMsg;
  field: { name: string; kind: { kind: string; lidar?: string; range?: number | null; sector?: [number, number] | null } };
}) {
  const selected = useStudioStore(
    (s) => s.selection.type === "sensor" && s.selection.name === field.name,
  );
  const active = useStudioStore((s) =>
    s.timeline ? laneActiveAt(s.timeline, field.name, s.playbackTime) : false,
  );
  const radius = field.kind.range ?? lidar.range[1];
  const half = (lidar.fov_deg * Math.PI) / 360;
  const [start, end] = field.kind.sector
    ? [
        (field.kind.sector[0] * Math.PI) / 180,
        (field.kind.sector[1] * Math.PI) / 180,
      ]
    : [-half, half];
  const segments = Math.max(8, Math.ceil(((end - start) * 180) / Math.PI / 7.5));
  const color = active ? ACTIVE_COLOR : selected ? SELECTED_COLOR : IDLE_COLOR;
  const onClick = (e: ThreeEvent<MouseEvent>) => {
    e.stopPropagation();
    const s = useStudioStore.getState();
    s.selectSensor(field.name);
    s.focusTab("obstacle");
  };
  return (
    <mesh onClick={onClick}>
      <circleGeometry args={[radius, segments, start, end - start]} />
      <meshBasicMaterial
        color={color}
        transparent
        opacity={active ? 0.28 : selected ? 0.22 : 0.1}
        side={THREE.DoubleSide}
        depthWrite={false}
      />
    </mesh>
  );
}

/**
 * Wireframe sector out to `reach` in the XY plane: the arc, edge rays
 * from the origin when the sweep is not a full circle, a heading tick at
 * angle 0 (+X), and — while selected — the min-range arc, so the blind
 * ring near the housing reads too.
 */
function sectorGeometry(
  lidar: LidarMsg,
  reach: number,
  selected: boolean,
): THREE.BufferGeometry {
  const half = (lidar.fov_deg * Math.PI) / 360;
  const full = lidar.fov_deg >= 360 - 1e-9;
  const segments: [number, number, number][] = [];
  const edge = (a: [number, number, number], b: [number, number, number]) => {
    segments.push(a, b);
  };
  const at = (ang: number, r: number): [number, number, number] => [
    r * Math.cos(ang),
    r * Math.sin(ang),
    0,
  ];
  const arc = (r: number) => {
    const steps = Math.max(2, Math.ceil(lidar.fov_deg / 7.5));
    for (let i = 0; i < steps; i++) {
      const a0 = -half + (2 * half * i) / steps;
      const a1 = -half + (2 * half * (i + 1)) / steps;
      edge(at(a0, r), at(a1, r));
    }
  };
  arc(reach);
  if (!full) {
    edge([0, 0, 0], at(-half, reach));
    edge([0, 0, 0], at(half, reach));
  }
  if (selected && lidar.range[0] < reach) {
    arc(lidar.range[0]);
  }
  // Heading tick: a chevron on the arc at angle 0 — which way the scan
  // frame's +X points.
  edge(at(0, reach), at(0.06 / Math.max(reach, 0.1), reach * 1.06));
  edge(at(0, reach), at(-0.06 / Math.max(reach, 0.1), reach * 1.06));
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    "position",
    new THREE.Float32BufferAttribute(segments.flat(), 3),
  );
  return geometry;
}

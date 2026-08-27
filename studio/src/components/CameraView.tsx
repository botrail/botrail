import { useEffect, useMemo, useRef, useState } from "react";
import { TransformControls } from "@react-three/drei";
import type { ThreeEvent } from "@react-three/fiber";
import * as THREE from "three";

import type { CameraMsg, PoseMsg } from "../protocol";
import { playbackRig } from "../playbackRig";
import { useStudioStore } from "../store";
import { cameraRig } from "../three/cameraRig";
import { sendUpsertCameraThrottled } from "../ws";
import { laneActiveAt, parkedFrame, type Frame } from "./SensorView";

const IDLE_COLOR = "#5fa8dc";
const SELECTED_COLOR = "#f0b429";
const ACTIVE_COLOR = "#e0503c";

/** Is any vision sensor looking through `camera` tripped at the playhead?
 * Shared by the frustum tint and the PiP's live border. */
export function visionActiveAt(
  s: {
    sensors: { name: string; kind: { kind: string; camera?: string } }[];
    timeline: { signals: { name: string; times: number[]; values: boolean[] }[] } | null;
    playbackTime: number;
  },
  camera: string,
): boolean {
  if (!s.timeline) return false;
  return s.sensors.some(
    (sensor) =>
      sensor.kind.kind === "vision" &&
      sensor.kind.camera === camera &&
      laneActiveAt(s.timeline, sensor.name, s.playbackTime),
  );
}
/** An unselected camera's frustum is a compact aim gizmo; selecting it
 * draws the true frustum out to the far clip for coverage checks. */
const IDLE_DEPTH = 1.2;

/**
 * Renders cameras: a small body plus a wireframe view frustum whose
 * aspect follows `resolution` and whose angle follows `fov_deg` (-Z
 * looks, +Y is image-up). A world camera is placed directly and, while
 * selected, gets a move/rotate gizmo; a vehicle camera rides its
 * machine's frame (parked or played back, like mounted sensors); a
 * link camera rides the sampled link pose — live state while posing,
 * `playbackRig.linkMounts` while a timeline plays.
 */
export function CameraView() {
  const cameras = useStudioStore((s) => s.cameras);
  if (cameras.length === 0) return null;
  return (
    <>
      {cameras.map((camera) => (
        <CameraNode key={camera.name} camera={camera} />
      ))}
    </>
  );
}

function CameraNode({ camera }: { camera: CameraMsg }) {
  const devices = useStudioStore((s) => s.devices);
  const robots = useStudioStore((s) => s.robots);
  const overridePoses = useStudioStore((s) => s.overridePoses);
  const overrideVehiclePoses = useStudioStore((s) => s.overrideVehiclePoses);
  const selection = useStudioStore((s) => s.selection);
  const gizmoMode = useStudioStore((s) => s.gizmoMode);
  const selected =
    selection.type === "camera" && selection.name === camera.name;
  // A vision sensor tripping through this camera turns the frustum red at
  // the playhead — the sensor's geometry is the camera's.
  const visionActive = useStudioStore((s) => visionActiveAt(s, camera.name));

  const [group, setGroup] = useState<THREE.Group | null>(null);
  const draggingRef = useRef(false);
  const mount = camera.mount;

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

  // World cameras follow the store pose except while being dragged, so
  // the server's echo doesn't fight the gizmo (the obstacle pattern).
  useEffect(() => {
    if (!group || mount.kind !== "world" || draggingRef.current) return;
    group.position.set(...camera.pose.position);
    group.quaternion.set(...camera.pose.quaternion);
  }, [group, mount.kind, camera.pose]);

  const onDrag = () => {
    if (!group || !draggingRef.current) return;
    sendUpsertCameraThrottled(camera.name, {
      ...camera,
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
    s.selectCamera(camera.name);
    s.focusTab("obstacle");
  };

  const shape = (
    <group
      position={camera.pose.position}
      quaternion={new THREE.Quaternion(...camera.pose.quaternion)}
    >
      <CameraShape camera={camera} selected={selected} active={visionActive} onClick={onSelect} />
    </group>
  );

  if (mount.kind === "world") {
    return (
      <>
        <group ref={setGroup}>
          <CameraShape camera={camera} selected={selected} active={visionActive} onClick={onSelect} />
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
      // bypassing React (see `playbackRig`). Keys are prefixed so a camera
      // and a sensor sharing a name cannot collide in the registries.
      ref={(node) => {
        const key = `camera:${camera.name}`;
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

/** Body + frustum lines, in the camera's own frame (-Z looks). */
function CameraShape({
  camera,
  selected,
  active,
  onClick,
}: {
  camera: CameraMsg;
  selected: boolean;
  /** A vision sensor looking through this camera is tripped. */
  active: boolean;
  onClick: (e: ThreeEvent<MouseEvent>) => void;
}) {
  const color = active ? ACTIVE_COLOR : selected ? SELECTED_COLOR : IDLE_COLOR;
  const frustum = useMemo(
    () => frustumGeometry(camera, selected ? camera.far : IDLE_DEPTH),
    [camera, selected],
  );
  useEffect(() => () => frustum.dispose(), [frustum]);
  return (
    <group>
      {/* The optical frame: the PiP pass reads this node's world
          transform, so every mount and the playback driver's writes
          reach the picture for free. */}
      <group
        ref={(node) => {
          if (node) cameraRig.nodes.set(camera.name, node);
          else cameraRig.nodes.delete(camera.name);
        }}
      />
      {/* Body: a small box with a lens stub toward -Z. */}
      <mesh position={[0, 0, 0.045]} onClick={onClick}>
        <boxGeometry args={[0.07, 0.05, 0.09]} />
        <meshStandardMaterial color={color} roughness={0.6} metalness={0.2} />
      </mesh>
      <mesh
        position={[0, 0, -0.012]}
        rotation={[Math.PI / 2, 0, 0]}
        onClick={onClick}
      >
        <cylinderGeometry args={[0.016, 0.016, 0.025, 16]} />
        <meshStandardMaterial color="#2a2e35" roughness={0.4} metalness={0.4} />
      </mesh>
      <lineSegments geometry={frustum}>
        <lineBasicMaterial
          color={color}
          transparent
          opacity={selected ? 0.9 : 0.55}
        />
      </lineSegments>
    </group>
  );
}

/**
 * Wireframe frustum out to `depth`: four rays from the origin, the near
 * and `depth` rectangles, and an image-up tick above the near plane.
 * Aspect comes from `resolution`, the angle from the horizontal
 * `fov_deg`.
 */
function frustumGeometry(camera: CameraMsg, depth: number): THREE.BufferGeometry {
  const [rw, rh] = camera.resolution;
  const aspect = rw > 0 && rh > 0 ? rw / rh : 16 / 9;
  const tan = Math.tan((camera.fov_deg * Math.PI) / 360);
  const rect = (d: number): [number, number, number][] => {
    const w = d * tan;
    const h = w / aspect;
    return [
      [-w, -h, -d],
      [w, -h, -d],
      [w, h, -d],
      [-w, h, -d],
    ];
  };
  const near = rect(Math.min(camera.near, depth));
  const far = rect(depth);
  const segments: [number, number, number][] = [];
  const edge = (a: [number, number, number], b: [number, number, number]) => {
    segments.push(a, b);
  };
  for (let i = 0; i < 4; i++) {
    edge([0, 0, 0], far[i]);
    edge(near[i], near[(i + 1) % 4]);
    edge(far[i], far[(i + 1) % 4]);
  }
  // Image-up tick: a little triangle over the near plane's top edge.
  const hn = near[2][1];
  const zn = near[2][2];
  const wn = near[1][0];
  edge([-0.5 * wn, hn, zn], [0, hn * 2.2, zn]);
  edge([0, hn * 2.2, zn], [0.5 * wn, hn, zn]);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    "position",
    new THREE.Float32BufferAttribute(segments.flat(), 3),
  );
  return geometry;
}

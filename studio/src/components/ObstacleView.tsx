import { useEffect, useMemo, useRef, useState } from "react";
import { Edges, TransformControls } from "@react-three/drei";
import type { ThreeEvent } from "@react-three/fiber";
import * as THREE from "three";

import { playbackRig } from "../playbackRig";
import type { GeometryMsg, MaterialMsg, ObstacleMsg, PoseMsg } from "../protocol";
import { collidingObstacleNames, useStudioStore } from "../store";
import { Aid } from "../three/cameraRig";
import { cursorEnter, cursorLeave } from "../three/cursor";
import { authoredColor, COLLISION_COLOR } from "../three/palette";
import { sendUpdatePoses, sendUpdateObstaclePose } from "../ws";
import { MeshVisual } from "./MeshVisual";
import { UsdVisual } from "./UsdVisual";
import { UNIT_BOX, UNIT_CYLINDER, UNIT_SPHERE } from "../three/primitiveGeometry";

const NEUTRAL_COLOR = "#9aa3b2";
const SELECT_EDGE_COLOR = "#cdd4df";

/** The obstacle's own colour (`primvars:displayColor`), or null when the
 * scene file authored none. */
function obstacleColor(obstacle: ObstacleMsg): THREE.Color | null {
  return obstacle.color ? authoredColor(obstacle.color) : null;
}

/** Names under an imported subtree — `/World/Pedestal` owns everything
 * below it, but not a sibling that merely shares the prefix
 * (`/World/PedestalFar`). */
export function inGroup(name: string, path: string): boolean {
  return name.startsWith(`${path}/`);
}

/** Draws every obstacle, handles click-to-select, and drives the move gizmo. */
export function ObstacleView() {
  const obstacles = useStudioStore((s) => s.obstacles);
  const collisions = useStudioStore((s) => s.collisions);
  const selection = useStudioStore((s) => s.selection);
  const hiddenObstacles = useStudioStore((s) => s.hiddenObstacles);
  // Stowed during playback: waiting in a magazine, or taken off the line.
  const stowed = useStudioStore((s) => s.stowedObstacles);
  const collidingObstacles = useMemo(
    () => collidingObstacleNames(collisions),
    [collisions],
  );
  const group = selection.type === "group" ? selection.path : null;

  return (
    <>
      {obstacles
        // `o.visible` is the scene's own answer (a collision proxy the
        // author never meant to draw); hiding is this viewer's. Stowed
        // obstacles stay MOUNTED and merely turn invisible: unmounting
        // would deregister them from the playback rig, and the driver
        // could never show them again mid-play (a carve stage whose
        // window arrives, a part leaving its magazine).
        .filter((o) => o.visible && !hiddenObstacles.has(o.name))
        .map((o) => (
        <ObstacleNode
          key={o.name}
          obstacle={o}
          stowed={stowed.has(o.name)}
          colliding={collidingObstacles.has(o.name)}
          selected={
            (selection.type === "obstacle" && selection.name === o.name) ||
            (group !== null && inGroup(o.name, group))
          }
          // A member of a selected group is outlined but carries no gizmo
          // of its own; the group's gizmo moves them together.
          gizmo={selection.type === "obstacle" && selection.name === o.name}
        />
      ))}
      {group !== null && <GroupGizmo path={group} />}
    </>
  );
}

/**
 * One gizmo for a whole imported subtree. The members keep their world
 * poses in the store — this only computes the rigid delta the drag applied
 * and hands every member its new pose in one message.
 *
 * The anchor starts at the members' centroid and is re-seeded whenever the
 * selection changes, so each drag measures from where the group now is.
 */
function GroupGizmo({ path }: { path: string }) {
  const obstacles = useStudioStore((s) => s.obstacles);
  const frames = useStudioStore((s) => s.frames);
  const gizmoMode = useStudioStore((s) => s.gizmoMode);
  const [anchor, setAnchor] = useState<THREE.Group | null>(null);
  const draggingRef = useRef(false);
  // The poses the drag started from, captured on mouse-down: deltas have to
  // be measured against a fixed origin, not against the previous frame.
  const startRef = useRef<{
    anchor: THREE.Matrix4;
    obstacles: [string, THREE.Matrix4][];
    frames: [string, THREE.Matrix4][];
  } | null>(null);

  const members = useMemo(
    () => obstacles.filter((o) => inGroup(o.name, path)),
    [obstacles, path],
  );
  const memberFrames = useMemo(
    () => frames.filter((f) => inGroup(f.name, path)),
    [frames, path],
  );

  const centroid = useMemo(() => {
    const c = new THREE.Vector3();
    const all = [
      ...members.map((m) => m.pose.position),
      ...memberFrames.map((f) => f.pose.position),
    ];
    all.forEach((p) => c.add(new THREE.Vector3(...p)));
    return all.length ? c.divideScalar(all.length) : c;
  }, [members, memberFrames]);

  // Re-seed on (re)selection and after each drag, so the gizmo sits on the
  // group rather than drifting away from it.
  useEffect(() => {
    if (!anchor || draggingRef.current) return;
    anchor.position.copy(centroid);
    anchor.quaternion.identity();
    anchor.updateMatrixWorld();
  }, [anchor, centroid]);

  const matrixOf = (pose: { position: number[]; quaternion: number[] }) =>
    new THREE.Matrix4().compose(
      new THREE.Vector3(...(pose.position as [number, number, number])),
      new THREE.Quaternion(...(pose.quaternion as [number, number, number, number])),
      new THREE.Vector3(1, 1, 1),
    );

  const onDown = () => {
    if (!anchor) return;
    draggingRef.current = true;
    anchor.updateMatrixWorld();
    startRef.current = {
      anchor: anchor.matrixWorld.clone(),
      obstacles: members.map((m) => [m.name, matrixOf(m.pose)]),
      frames: memberFrames.map((f) => [f.name, matrixOf(f.pose)]),
    };
  };

  const onDrag = () => {
    const start = startRef.current;
    if (!anchor || !draggingRef.current || !start) return;
    anchor.updateMatrixWorld();
    // delta = now ∘ start⁻¹, applied on the left so members keep their
    // relative placement.
    const delta = anchor.matrixWorld
      .clone()
      .multiply(start.anchor.clone().invert());
    const moved = (entries: [string, THREE.Matrix4][]) =>
      entries.map(([name, m]) => {
        const out = delta.clone().multiply(m);
        const position = new THREE.Vector3();
        const quaternion = new THREE.Quaternion();
        out.decompose(position, quaternion, new THREE.Vector3());
        return [
          name,
          {
            position: [position.x, position.y, position.z],
            quaternion: [quaternion.x, quaternion.y, quaternion.z, quaternion.w],
          },
        ] as [string, PoseMsg];
      });
    sendUpdatePoses(path, {
      obstacles: moved(start.obstacles),
      frames: moved(start.frames),
    });
  };

  if (members.length === 0 && memberFrames.length === 0) return null;
  return (
    <>
      <group ref={setAnchor} />
      {anchor && (
        <Aid>
          <TransformControls
            object={anchor}
            mode={gizmoMode}
            size={0.9}
            onMouseDown={onDown}
            onMouseUp={() => {
              draggingRef.current = false;
              startRef.current = null;
            }}
            onObjectChange={onDrag}
          />
        </Aid>
      )}
    </>
  );
}

function ObstacleNode({
  obstacle,
  stowed,
  colliding,
  selected,
  gizmo,
}: {
  obstacle: ObstacleMsg;
  /** Hidden by the playback tracks at the current playhead. The node stays
   * mounted (and rig-registered) so the driver can reveal it mid-play. */
  stowed: boolean;
  colliding: boolean;
  selected: boolean;
  /** Whether this obstacle carries its own move gizmo (a group member does
   * not: the group's gizmo moves it). */
  gizmo: boolean;
}) {
  const gizmoMode = useStudioStore((s) => s.gizmoMode);
  const selectFromViewport = useStudioStore((s) => s.selectFromViewport);

  const [group, setGroup] = useState<THREE.Group | null>(null);
  const draggingRef = useRef(false);
  const { pose, name } = obstacle;
  // Registered so the playback driver can advect this obstacle (a body
  // riding the line) without a React pass per frame.
  useEffect(() => {
    if (!group) return;
    playbackRig.obstacles.set(name, group);
    return () => {
      playbackRig.obstacles.delete(name);
    };
  }, [group, name]);
  // During trajectory playback, attached objects follow their baked track
  // instead of the live scene pose.
  const overridePose = useStudioStore(
    (s) => s.overrideObstaclePoses?.[name] ?? null,
  );

  // Follow the store pose except while the user is dragging this obstacle, so
  // the server's echo (and a rejected move) don't fight the gizmo.
  useEffect(() => {
    if (!group || draggingRef.current) return;
    const shown = overridePose ?? pose;
    group.position.set(...shown.position);
    group.quaternion.set(...shown.quaternion);
  }, [group, pose, overridePose]);

  const onDrag = () => {
    // objectChange also fires on attach/programmatic updates; only real drags
    // should move the obstacle.
    if (!group || !draggingRef.current) return;
    sendUpdateObstaclePose(name, {
      position: [group.position.x, group.position.y, group.position.z],
      quaternion: [
        group.quaternion.x,
        group.quaternion.y,
        group.quaternion.z,
        group.quaternion.w,
      ],
    });
  };

  // A drag ends in a DOM click too, and dragging the group gizmo passes
  // over its own members — without this, letting go would drill into
  // whichever part sat under the cursor.
  const downAt = useRef<[number, number] | null>(null);
  const DRAG_SLOP_PX = 4;

  const onSelect = (e: ThreeEvent<MouseEvent>) => {
    // The raycaster tests invisible nodes too; a currently-stowed obstacle
    // must neither select nor swallow the click meant for whatever is
    // drawn in its place. Live visibility (the driver writes it during
    // playback), not the React prop, is the truth here.
    if (group && !group.visible) return;
    e.stopPropagation();
    const from = downAt.current;
    downAt.current = null;
    if (
      from &&
      Math.hypot(e.clientX - from[0], e.clientY - from[1]) > DRAG_SLOP_PX
    ) {
      return;
    }
    selectFromViewport(name);
  };

  // Scenery the author styled is solid and casts. Unstyled primitives are
  // translucent collision proxies; MeshVisual also recognises a file's own
  // materials as authored scenery.
  const tint = useMemo(() => obstacleColor(obstacle), [obstacle]);
  // "Styled at all" is what separates scenery from a bare collision proxy —
  // a material counts as much as a colour. Without this, authoring only a
  // material leaves the object see-through and casting no shadow, which is
  // the opposite of what saying "this is brushed steel" meant.
  const styled = tint !== null || obstacle.material != null;
  const color = colliding ? new THREE.Color(COLLISION_COLOR) : (tint ?? new THREE.Color(NEUTRAL_COLOR));

  return (
    <>
      <group ref={setGroup} visible={!stowed}>
        {obstacle.visual_asset ? <group onClick={onSelect}
          onPointerDown={(e) => { downAt.current = [e.clientX, e.clientY]; }}
          onPointerOver={(e) => { e.stopPropagation(); cursorEnter(); }}
          onPointerOut={cursorLeave}>
          <UsdVisual source={obstacle.visual_asset} color={color}
            forceColor={colliding || (!!obstacle.visual_asset.color_override && tint !== null)}
            material={obstacle.material} />
        </group> : <ObstacleGeometry
          geometry={obstacle.geometry}
          color={color}
          // An authored color and a collision highlight both mean
          // something the mesh's own materials cannot say, so they paint
          // over it; the bare neutral does not.
          forceColor={colliding || tint !== null}
          material={obstacle.material}
          solid={styled}
          selected={selected}
          onSelect={onSelect}
          onDown={(e) => {
            downAt.current = [e.clientX, e.clientY];
          }}
        />}
      </group>
      {gizmo && !stowed && group && (
        <Aid>
          <TransformControls
            object={group}
            mode={gizmoMode}
            size={0.65}
            onMouseDown={() => {
              draggingRef.current = true;
            }}
            onMouseUp={() => {
              draggingRef.current = false;
            }}
            onObjectChange={onDrag}
          />
        </Aid>
      )}
    </>
  );
}

function ObstacleGeometry({
  geometry,
  color,
  forceColor,
  material,
  solid,
  selected,
  onSelect,
  onDown,
}: {
  geometry: GeometryMsg;
  color: THREE.Color;
  forceColor: boolean;
  material?: MaterialMsg | null;
  solid: boolean;
  selected: boolean;
  onSelect: (e: ThreeEvent<MouseEvent>) => void;
  onDown: (e: ThreeEvent<PointerEvent>) => void;
}) {
  const highlight = selected && <Edges color={SELECT_EDGE_COLOR} lineWidth={2} />;
  const pick = {
    onPointerDown: onDown,
    onClick: onSelect,
    onPointerOver: (e: ThreeEvent<PointerEvent>) => {
      e.stopPropagation();
      cursorEnter();
    },
    onPointerOut: () => cursorLeave(),
    // A see-through proxy casting a hard shadow reads as a glitch, so only
    // solid scenery casts. Everything receives.
    castShadow: solid && (material?.opacity ?? 1) >= 1,
    receiveShadow: true,
  };

  switch (geometry.kind) {
    case "box":
      return (
        <mesh {...pick} scale={geometry.size}>
          <primitive object={UNIT_BOX} attach="geometry" />
          <ObstacleMaterial color={color} solid={solid} material={material} />
          {highlight}
        </mesh>
      );
    case "cylinder":
      // URDF cylinders point along +Z; three's CylinderGeometry points along
      // +Y, so rotate +90deg about X to align them (same as SceneView).
      return (
        <mesh rotation={[Math.PI / 2, 0, 0]} {...pick}
          scale={[geometry.radius, geometry.length, geometry.radius]}>
          <primitive object={UNIT_CYLINDER} attach="geometry" />
          <ObstacleMaterial color={color} solid={solid} material={material} />
          {highlight}
        </mesh>
      );
    case "sphere":
      return (
        <mesh {...pick} scale={geometry.radius}>
          <primitive object={UNIT_SPHERE} attach="geometry" />
          <ObstacleMaterial color={color} solid={solid} material={material} />
          {highlight}
        </mesh>
      );
    case "mesh":
      // URL is empty in wasm mode (no mesh serving there yet).
      if (!geometry.url) return null;
      return (
        <group {...pick}>
          <MeshVisual
            geometry={geometry}
            color={color}
            forceColor={forceColor}
            material={material}
            roughness={solid ? 0.8 : 0.7}
            opacity={solid ? 1 : 0.85}
            castShadow={solid ? true : undefined}
            receiveShadow
          />
        </group>
      );
    default:
      return null;
  }
}

function ObstacleMaterial({
  color,
  solid,
  material,
}: {
  color: THREE.Color;
  solid: boolean;
  material?: MaterialMsg | null;
}) {
  // An authored material says how the surface takes light; without one the
  // studio picks, and a bare collision proxy stays see-through.
  return (
    <meshStandardMaterial
      color={color}
      roughness={material ? material.roughness : solid ? 0.8 : 0.7}
      metalness={material ? material.metalness : 0.05}
      transparent={!solid || (material?.opacity ?? 1) < 1}
      opacity={material?.opacity ?? (solid ? 1 : 0.85)}
      depthWrite={(material?.opacity ?? 1) >= 1}
    />
  );
}

import { useEffect, useRef, useState } from "react";
import { TransformControls } from "@react-three/drei";
import * as THREE from "three";

import type { Via } from "../protocol";
import { useStudioStore } from "../store";
import { sendUpdateRouteVia } from "../ws";

/**
 * Draggable way point of a route: the point selected in ROUTING › edit
 * gets a marker the move gizmo takes; every change goes to the host as
 * the route's new way, and the route redraws from what comes back. While
 * not dragging, the marker tracks the authoritative point.
 */
export function RouteViaGizmo() {
  const selection = useStudioStore((s) => s.selection);
  const routes = useStudioStore((s) => s.routes);
  const route = selection.type === "route_via" ? (routes.find((r) => r.name === selection.route) ?? null) : null;
  const index = selection.type === "route_via" ? selection.index : -1;
  const via: Via | null = route ? (route.via[index] ?? null) : null;
  const point = via && via.kind === "point" ? via.position : null;

  const [target, setTarget] = useState<THREE.Group | null>(null);
  const draggingRef = useRef(false);

  useEffect(() => {
    if (!point || !target || draggingRef.current) return;
    target.position.set(point[0], point[1], point[2]);
  }, [point, target]);

  if (!route || !point) return null;
  const routeName = route.name;
  const way = route.via;

  const onDrag = () => {
    if (!target || !draggingRef.current) return;
    const next: Via[] = way.map((v, i) =>
      i === index ? { kind: "point", position: [target.position.x, target.position.y, target.position.z] } : v,
    );
    sendUpdateRouteVia(routeName, next);
  };

  return (
    <>
      <group ref={setTarget}>
        <mesh renderOrder={10}>
          <sphereGeometry args={[0.014, 16, 12]} />
          <meshBasicMaterial color="#ffb84d" depthTest={false} transparent opacity={0.9} />
        </mesh>
      </group>
      {target && (
        <TransformControls
          object={target}
          mode="translate"
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

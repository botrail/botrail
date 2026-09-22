import { useEffect, useMemo } from "react";
import { useFrame, useThree } from "@react-three/fiber";
import * as THREE from "three";

import { currentPick, endPick, pickRig } from "../physicsPick";
import { useStudioStore } from "../store";

const HELD_COLOR = "#ffd166";
const REFUSED_COLOR = "#ff6b6b";

/** Registers the camera controls with the pick, so a drag can hold the
 * camera still. Mounted once inside the Canvas. */
export function PickBridge() {
  const controls = useThree((s) => s.controls) as { enabled: boolean } | null;
  useEffect(() => {
    pickRig.controls = controls;
    return () => {
      if (pickRig.controls === controls) pickRig.controls = null;
    };
  }, [controls]);
  return null;
}

/** The hand on a body: a line from the grabbed point (riding the body as
 * the stream moves it) to the mouse target, and a dot at the target.
 * Amber while the host holds the body, red when it refused (a bolted
 * mirror, a part a program holds). An authoring aid — wrapped in `Aid`
 * by the viewport so the camera pass never sees it. */
export function DragLineView() {
  const drag = useStudioStore((s) => s.drag);
  const grab = useStudioStore((s) => s.grab);
  const streaming = useStudioStore((s) => s.bakeStream?.live === true);
  const line = useMemo(() => {
    const geometry = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(),
      new THREE.Vector3(),
    ]);
    const material = new THREE.LineBasicMaterial({ color: HELD_COLOR });
    const object = new THREE.Line(geometry, material);
    object.frustumCulled = false;
    return object;
  }, []);
  const dot = useMemo(() => {
    const object = new THREE.Mesh(
      new THREE.SphereGeometry(0.012, 12, 8),
      new THREE.MeshBasicMaterial({ color: HELD_COLOR }),
    );
    object.frustumCulled = false;
    return object;
  }, []);
  useEffect(
    () => () => {
      line.geometry.dispose();
      (line.material as THREE.Material).dispose();
      dot.geometry.dispose();
      (dot.material as THREE.Material).dispose();
    },
    [line, dot],
  );
  // The stream ending under a hand lets go.
  useEffect(() => {
    if (!streaming) endPick();
  }, [streaming]);
  useFrame(() => {
    const pick = currentPick();
    if (!drag || !pick) return;
    const anchor = pick.node.localToWorld(pick.local.clone());
    const positions = line.geometry.getAttribute("position") as THREE.BufferAttribute;
    positions.setXYZ(0, anchor.x, anchor.y, anchor.z);
    positions.setXYZ(1, drag.target[0], drag.target[1], drag.target[2]);
    positions.needsUpdate = true;
    dot.position.set(drag.target[0], drag.target[1], drag.target[2]);
  });
  if (!drag) return null;
  const refused = grab?.name === drag.name && !grab.held;
  const color = refused ? REFUSED_COLOR : HELD_COLOR;
  (line.material as THREE.LineBasicMaterial).color.set(color);
  (dot.material as THREE.MeshBasicMaterial).color.set(color);
  return (
    <>
      <primitive object={line} />
      <primitive object={dot} />
    </>
  );
}

import { useEffect, useState } from "react";
import * as THREE from "three";
import { ThreeUsdRobotLoader } from "three-usd-robot";

import { useStudioStore } from "../store";

/**
 * Client-side rendering of a USD stage dropped in wasm mode. The wasm
 * session owns the collision side (obstacles carry no mesh URLs there);
 * three-usd-robot draws the original stage geometry in place.
 */
export function WasmStageView() {
  const droppedStage = useStudioStore((s) => s.droppedStage);
  const [stage, setStage] = useState<THREE.Object3D | null>(null);

  useEffect(() => {
    if (!droppedStage) {
      setStage(null);
      return;
    }
    let cancelled = false;
    new ThreeUsdRobotLoader({ loadSceneGeometry: true, worldUp: "Z" })
      .parse(droppedStage.data)
      .then((loaded) => {
        if (!cancelled) setStage(loaded);
      })
      .catch((e) =>
        console.error("botrail studio: failed to render dropped stage", e),
      );
    return () => {
      cancelled = true;
    };
  }, [droppedStage]);

  if (!stage) return null;
  // `worldUp: "Z"` aligns the stage with botrail's world (and the
  // collision proxies) regardless of the authored up axis.
  return <primitive object={stage} />;
}

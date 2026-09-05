import { useEffect, useMemo, useState } from "react";
import * as THREE from "three";

import type { MaterialMsg, VisualAssetMsg } from "../protocol";
import { meshInstance } from "../three/meshAppearance";
import { loadUsdVisual } from "../three/usdVisuals";

export function UsdVisual({source, color, forceColor, material}: {
  source: VisualAssetMsg;
  color: string | THREE.Color;
  forceColor: boolean;
  material?: MaterialMsg | null;
}) {
  const [loaded, setLoaded] = useState<THREE.Object3D | null>(null);
  useEffect(() => {
    let active = true;
    setLoaded(null);
    loadUsdVisual(source.url, source.prim_path).then(o => {
      if (active) setLoaded(o);
    }).catch(e => console.error("botrail studio: failed to load USD appearance", e));
    return () => { active = false; };
  }, [source.url, source.prim_path]);
  const {r, g, b} = new THREE.Color(color);
  const metalness = material?.metalness, roughness = material?.roughness, opacity = material?.opacity;
  const instance = useMemo(() => loaded ? meshInstance(loaded, true, {
    color: new THREE.Color(r, g, b), forceColor,
    material: metalness !== undefined && roughness !== undefined ? {metalness, roughness, opacity} : null,
  }, true, true) : null, [loaded, r, g, b, forceColor, metalness, roughness, opacity]);
  useEffect(() => () => instance?.dispose(), [instance]);
  const matrix = useMemo(() => new THREE.Matrix4().fromArray(source.transform), [source.transform]);
  return instance ? <group matrix={matrix} matrixAutoUpdate={false}>
    <primitive object={instance.object} />
  </group> : null;
}

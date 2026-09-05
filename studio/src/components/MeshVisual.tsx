import { useEffect, useMemo, useState } from "react";
import * as THREE from "three";

import type { GeometryMsg, MaterialMsg } from "../protocol";
import { loadMesh, type LoadedMesh } from "../three/loaders";
import { meshInstance } from "../three/meshAppearance";

type MeshGeometry = Extract<GeometryMsg, { kind: "mesh" }>;

export function MeshVisual({
  geometry,
  color,
  forceColor = false,
  material,
  roughness = 0.85,
  opacity = 1,
  castShadow,
  receiveShadow = true,
}: {
  geometry: MeshGeometry;
  color: string | THREE.Color;
  /** Paint `color` over the mesh's own materials. Set when the color
   * carries meaning the mesh cannot — a collision highlight, or a color
   * the scene author chose. */
  forceColor?: boolean;
  material?: MaterialMsg | null;
  roughness?: number;
  opacity?: number;
  /** Undefined casts only when the file supplied its own materials. */
  castShadow?: boolean;
  receiveShadow?: boolean;
}) {
  const [loaded, setLoaded] = useState<LoadedMesh | null>(null);

  useEffect(() => {
    let alive = true;
    loadMesh(geometry.url, geometry.ext).then((m) => {
      if (alive) setLoaded(m);
    });
    return () => {
      alive = false;
    };
  }, [geometry.url, geometry.ext]);

  // Depend on colour components, not a fresh Color object's identity: live
  // poses and playback must not clone the whole mesh on every React render.
  const { r, g, b } = new THREE.Color(color);
  const metalness = material?.metalness;
  const authoredRoughness = material?.roughness;
  const authoredOpacity = material?.opacity;
  const casts = (castShadow ?? (loaded?.kind === "object" && loaded.shaded)) && (authoredOpacity ?? 1) >= 1;
  const objectClone = useMemo(() => {
    if (!loaded || loaded.kind !== "object") return null;
    return meshInstance(loaded.object, loaded.shaded, {
      color: new THREE.Color(r, g, b), forceColor, roughness, opacity,
      material: metalness !== undefined && authoredRoughness !== undefined
      ? { metalness, roughness: authoredRoughness, opacity: authoredOpacity } : null,
    }, casts, receiveShadow);
  }, [loaded, r, g, b, forceColor, metalness, authoredRoughness, authoredOpacity, roughness, opacity, casts, receiveShadow]);

  useEffect(() => () => objectClone?.dispose(), [objectClone]);

  if (!loaded) return null;

  if (loaded.kind === "geometry") {
    return (
      <mesh geometry={loaded.geometry} scale={geometry.scale}
        castShadow={casts} receiveShadow={receiveShadow}>
        <meshStandardMaterial
          color={color}
          roughness={material?.roughness ?? roughness}
          metalness={material?.metalness ?? 0.05}
          opacity={authoredOpacity ?? opacity}
          transparent={(authoredOpacity ?? opacity) < 1}
          depthWrite={(authoredOpacity ?? opacity) >= 1}
        />
      </mesh>
    );
  }

  return objectClone ? (
    <primitive object={objectClone.object} scale={geometry.scale} />
  ) : null;
}

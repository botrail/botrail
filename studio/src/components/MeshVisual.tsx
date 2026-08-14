import { useEffect, useMemo, useState } from "react";
import * as THREE from "three";

import type { GeometryMsg } from "../protocol";
import { loadMesh, type LoadedMesh } from "../three/loaders";

type MeshGeometry = Extract<GeometryMsg, { kind: "mesh" }>;

export function MeshVisual({
  geometry,
  color,
  forceColor = false,
}: {
  geometry: MeshGeometry;
  color: string | THREE.Color;
  /** Paint `color` over the mesh's own materials. Set when the color
   * carries meaning the mesh cannot — a collision highlight, or a color
   * the scene author chose. */
  forceColor?: boolean;
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

  // OBJ files parse to an Object3D that can only live at one place in the
  // graph, so clone per instance. A mesh that brought its own materials
  // keeps them; otherwise stamp the link color onto it.
  const objectClone = useMemo(() => {
    if (!loaded || loaded.kind !== "object") return null;
    const clone = loaded.object.clone(true);
    if (loaded.shaded && !forceColor) return clone;
    const material = new THREE.MeshStandardMaterial({
      color: new THREE.Color(color),
      roughness: 0.85,
      metalness: 0.05,
    });
    clone.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (mesh.isMesh) mesh.material = material;
    });
    return clone;
  }, [loaded, color, forceColor]);

  if (!loaded) return null;

  if (loaded.kind === "geometry") {
    return (
      <mesh geometry={loaded.geometry} scale={geometry.scale}>
        <meshStandardMaterial
          color={color}
          roughness={0.85}
          metalness={0.05}
        />
      </mesh>
    );
  }

  return objectClone ? (
    <primitive object={objectClone} scale={geometry.scale} />
  ) : null;
}

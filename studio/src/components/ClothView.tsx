import { useEffect, useMemo } from "react";
import * as THREE from "three";

import type { ClothTrackMsg } from "../protocol";
import { useStudioStore } from "../store";

const CLOTH_COLOR = "#c8b39a";

/** One simulated cloth: a triangle mesh whose vertices follow the
 * playback sample (cloth tracks carry positions per sample, not poses). */
function ClothMesh({
  track,
  sample,
}: {
  track: ClothTrackMsg;
  sample: Float32Array | undefined;
}) {
  const geometry = useMemo(() => {
    const g = new THREE.BufferGeometry();
    const first = track.points[0] ?? [];
    const positions = new Float32Array(first.length * 3);
    first.forEach((p, i) => {
      positions[3 * i] = p[0];
      positions[3 * i + 1] = p[1];
      positions[3 * i + 2] = p[2];
    });
    g.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    const index = new Uint32Array(track.triangles.length * 3);
    track.triangles.forEach((t, i) => {
      index[3 * i] = t[0];
      index[3 * i + 1] = t[1];
      index[3 * i + 2] = t[2];
    });
    g.setIndex(new THREE.BufferAttribute(index, 1));
    g.computeVertexNormals();
    return g;
  }, [track]);
  useEffect(() => () => geometry.dispose(), [geometry]);
  useEffect(() => {
    if (!sample) return;
    const attr = geometry.getAttribute("position") as THREE.BufferAttribute;
    if (attr.array.length !== sample.length) return;
    (attr.array as Float32Array).set(sample);
    attr.needsUpdate = true;
    geometry.computeVertexNormals();
    geometry.computeBoundingSphere();
  }, [geometry, sample]);
  return (
    <mesh geometry={geometry} castShadow receiveShadow>
      <meshStandardMaterial
        color={CLOTH_COLOR}
        side={THREE.DoubleSide}
        roughness={0.9}
        metalness={0}
      />
    </mesh>
  );
}

/** Draws every cloth track of the current playback. */
export function ClothView() {
  const cloths = useStudioStore((s) => s.playback?.cloths ?? null);
  const samples = useStudioStore((s) => s.clothSamples);
  if (!cloths) return null;
  return (
    <>
      {cloths.tracks.map((track) => (
        <ClothMesh key={track.name} track={track} sample={samples?.[track.name]} />
      ))}
    </>
  );
}

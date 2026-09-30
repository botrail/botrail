import { useCallback, useEffect, useMemo, useRef } from "react";
import * as THREE from "three";

import { clothHeldAt } from "../playback";
import { playbackRig } from "../playbackRig";
import type { ClothTrackMsg } from "../protocol";
import { useStudioStore } from "../store";

const CLOTH_COLOR = "#c8b39a";
const HELD_COLOR = "#ffb020";

/** One simulated cloth: a triangle mesh whose vertices follow playback,
 * with the vertices a gripper holds drawn as dots over it. Cloth tracks
 * carry positions per sample, not poses, so the view owns the buffers and
 * hands the playback driver an applier (`playbackRig.cloths`); the store's
 * sample — set when playback starts, pauses or seeks — goes through the
 * same applier. */
function ClothMesh({ track }: { track: ClothTrackMsg }) {
  const sample = useStudioStore((s) => s.clothSamples?.[track.name]);
  const dots = useRef<THREE.Points>(null);
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
    // A growing live track keeps its mesh: the triangles are what it is.
  }, [track.name, track.triangles]);
  const heldGeometry = useMemo(() => new THREE.BufferGeometry(), []);
  useEffect(() => () => geometry.dispose(), [geometry]);
  useEffect(() => () => heldGeometry.dispose(), [heldGeometry]);

  const apply = useCallback(
    (positions: Float32Array, held: number[]) => {
      const attr = geometry.getAttribute("position") as THREE.BufferAttribute;
      if (attr.array.length !== positions.length) return;
      (attr.array as Float32Array).set(positions);
      attr.needsUpdate = true;
      geometry.computeVertexNormals();
      geometry.computeBoundingSphere();
      const points = new Float32Array(held.length * 3);
      held.forEach((vertex, k) => {
        points[3 * k] = positions[3 * vertex];
        points[3 * k + 1] = positions[3 * vertex + 1];
        points[3 * k + 2] = positions[3 * vertex + 2];
      });
      heldGeometry.setAttribute("position", new THREE.BufferAttribute(points, 3));
      heldGeometry.computeBoundingSphere();
      if (dots.current) dots.current.visible = held.length > 0;
    },
    [geometry, heldGeometry],
  );
  useEffect(() => {
    playbackRig.cloths.set(track.name, apply);
    return () => {
      if (playbackRig.cloths.get(track.name) === apply) playbackRig.cloths.delete(track.name);
    };
  }, [track.name, apply]);
  // The store's sample is the state while the driver is not running. The
  // time is read, not subscribed to: the cursor's throttled clock must not
  // put a stale sample back over what the driver wrote.
  useEffect(() => {
    if (sample) apply(sample, clothHeldAt(track, useStudioStore.getState().playbackTime));
  }, [apply, sample, track]);

  return (
    <>
      <mesh geometry={geometry} castShadow receiveShadow>
        <meshStandardMaterial
          color={CLOTH_COLOR}
          side={THREE.DoubleSide}
          roughness={0.9}
          metalness={0}
        />
      </mesh>
      <points ref={dots} geometry={heldGeometry} renderOrder={2} visible={false}>
        <pointsMaterial color={HELD_COLOR} size={7} sizeAttenuation={false} depthTest={false} />
      </points>
    </>
  );
}

/** Draws every cloth track of the current playback. */
export function ClothView() {
  const cloths = useStudioStore((s) => s.playback?.cloths ?? null);
  if (!cloths) return null;
  return (
    <>
      {cloths.map((track) => (
        <ClothMesh key={track.name} track={track} />
      ))}
    </>
  );
}

import { useCallback, useEffect, useMemo, useRef } from "react";
import * as THREE from "three";
import { clothHeldAt, samplePlayback } from "../playback";
import { playbackRig } from "../playbackRig";
import type { RopeTrack, PoseMsg } from "../protocol";
import { useStudioStore } from "../store";
import { authoredColor } from "../three/palette";

const ROPE_COLOR = "#368aaf";

/** Actual-radius straight tubes over the simulated wire segments. Rendering
 * does not smooth away contact geometry or claim torsion/cross-section state. */
function RopeMesh({ track }: { track: RopeTrack }) {
  const sample = useStudioStore((s) => s.ropeSamples?.[track.name]);
  const tubes = useRef<THREE.InstancedMesh>(null);
  const ends = useRef<THREE.InstancedMesh>(null);
  const dots = useRef<THREE.Points>(null);
  const connectors = useRef<(THREE.Group | null)[]>([]);
  const heldGeometry = useMemo(() => new THREE.BufferGeometry(), []);
  // An authored colour is linear RGB, the way obstacles carry theirs.
  const color = useMemo(() => track.color ? authoredColor(track.color) : new THREE.Color(ROPE_COLOR), [track.color]);
  useEffect(() => () => heldGeometry.dispose(), [heldGeometry]);
  const apply = useCallback((positions: Float32Array, held: number[], bodies: PoseMsg[]) => {
    const matrix = new THREE.Matrix4();
    const q = new THREE.Quaternion();
    const a = new THREE.Vector3();
    const b = new THREE.Vector3();
    const delta = new THREE.Vector3();
    const scale = new THREE.Vector3();
    const up = new THREE.Vector3(0,1,0);
    track.segments.forEach(([i,j],k) => {
      a.fromArray(positions,3*i); b.fromArray(positions,3*j);
      delta.subVectors(b,a); const length=delta.length();
      if (length > 0) q.setFromUnitVectors(up,delta.divideScalar(length));
      else q.identity();
      a.add(b).multiplyScalar(0.5);
      scale.set(track.radius_m,length,track.radius_m);
      tubes.current?.setMatrixAt(k,matrix.compose(a,q,scale));
    });
    for (let i=0;i<positions.length/3;i++) {
      a.fromArray(positions,3*i); q.identity(); scale.setScalar(track.radius_m);
      ends.current?.setMatrixAt(i,matrix.compose(a,q,scale));
    }
    if (tubes.current) { tubes.current.instanceMatrix.needsUpdate=true; tubes.current.computeBoundingSphere(); }
    if (ends.current) { ends.current.instanceMatrix.needsUpdate=true; ends.current.computeBoundingSphere(); }
    const points = new Float32Array(held.length*3);
    held.forEach((v,i) => points.set(positions.subarray(3*v,3*v+3),3*i));
    heldGeometry.setAttribute("position", new THREE.BufferAttribute(points,3));
    heldGeometry.computeBoundingSphere();
    if (dots.current) dots.current.visible=held.length>0;
    bodies.forEach((p,i) => { const node=connectors.current[i]; if (node) { node.position.set(...p.position); node.quaternion.set(...p.quaternion); } });
  }, [track,heldGeometry]);
  useEffect(() => {
    playbackRig.ropes.set(track.name,apply);
    return () => { if (playbackRig.ropes.get(track.name)===apply) playbackRig.ropes.delete(track.name); };
  }, [track.name,apply]);
  useEffect(() => {
    const state=useStudioStore.getState();
    if (sample && state.playback) apply(sample,clothHeldAt(track,state.playbackTime),samplePlayback(state.playback,state.playbackTime).ropeBodies?.[track.name] ?? []);
  }, [sample,track,apply]);
  return <group name={`rope/${track.name}`}>
    <instancedMesh ref={tubes} args={[undefined,undefined,track.segments.length]} castShadow>
      <cylinderGeometry args={[1,1,1,10]} /><meshStandardMaterial color={color} roughness={0.65} />
    </instancedMesh>
    <instancedMesh ref={ends} args={[undefined,undefined,track.points[0]?.length ?? 0]} castShadow>
      <sphereGeometry args={[1,10,6]} /><meshStandardMaterial color={color} roughness={0.65} />
    </instancedMesh>
    <points ref={dots} geometry={heldGeometry} renderOrder={2} visible={false}>
      <pointsMaterial color="#ffb020" size={8} sizeAttenuation={false} depthTest={false} />
    </points>
    {track.connectors.map((c,i) => <group key={c.name} name={`rope-connector/${c.name}`} ref={(node) => { connectors.current[i]=node; }}>
      <mesh castShadow receiveShadow>
        {c.shape==="sphere" ? <sphereGeometry args={[c.dimensions_m[0],16,10]} /> : <boxGeometry args={[2*c.dimensions_m[0],2*c.dimensions_m[1],2*c.dimensions_m[2]]} />}
        <meshStandardMaterial color="#e39632" metalness={0.45} roughness={0.4} />
      </mesh>
    </group>)}
  </group>;
}
export function RopeView() {
  const ropes=useStudioStore((s) => s.playback?.ropes ?? null);
  return <>{ropes?.map((r) => <RopeMesh key={r.name} track={r} />)}</>;
}

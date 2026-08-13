import { useEffect, useMemo } from "react";
import * as THREE from "three";

import { playbackRig } from "../playbackRig";
import { useStudioStore } from "../store";

/**
 * Cut traces: one growing polyline per declared trace, the TCP's trail
 * while the bound signal is on — "what has been cut so far". The driver
 * owns the per-frame state (appending points, spinning the bound link);
 * this component only mounts the lines and registers their handles. The
 * trail restarts when the playhead jumps backward.
 */
export function CutTraceView() {
  const flashes = useStudioStore((s) => s.flashes);
  const traces = useMemo(
    () => flashes.filter((f) => f.kind === "trace"),
    [flashes],
  );
  if (traces.length === 0) return null;
  return (
    <>
      {traces.map((trace) => (
        <Trace key={trace.name} name={trace.name} />
      ))}
    </>
  );
}

function Trace({ name }: { name: string }) {
  const line = useMemo(() => {
    const material = new THREE.LineBasicMaterial({
      color: "#ffc46b",
      transparent: true,
      opacity: 0.95,
      depthTest: false,
    });
    const object = new THREE.Line(new THREE.BufferGeometry(), material);
    // Drawn over the stock and the toolpath overlay, like the gizmo.
    object.renderOrder = 11;
    object.frustumCulled = false;
    object.visible = false;
    return object;
  }, []);
  useEffect(() => {
    playbackRig.traces.set(name, {
      line,
      positions: [],
      lastT: 0,
      spinAngle: 0,
    });
    return () => {
      playbackRig.traces.delete(name);
    };
  }, [name, line]);
  return <primitive object={line} />;
}

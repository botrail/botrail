import { useEffect, useMemo } from "react";
import * as THREE from "three";

import type { FlashMsg } from "../protocol";
import { playbackRig, signalAt, styleSpray } from "../playbackRig";
import { robotByName, useStudioStore } from "../store";

/**
 * Spray cones: one translucent cone per declared spray effect, parked
 * invisible until the playback driver poses it at the bound robot's TCP
 * while the effect's signal is on. The cone's apex sits at the tip and
 * its axis runs along the TCP's -Z — the spray direction, in the same
 * convention the toolpath solver and the film integrator use. The driver
 * owns the playing case; this component mounts the objects and covers the
 * paused case (a seek onto a stroke shows the jet standing).
 */
export function SprayView() {
  const all = useStudioStore((s) => s.flashes);
  const sprays = useMemo(() => all.filter((f) => f.kind === "spray"), [all]);

  const playing = useStudioStore((s) => s.playing);
  const playbackTime = useStudioStore((s) => s.playbackTime);
  const overridePoses = useStudioStore((s) => s.overridePoses);
  const timeline = useStudioStore((s) => s.timeline);
  const robots = useStudioStore((s) => s.robots);
  useEffect(() => {
    if (playing) return;
    for (const spray of sprays) {
      const node = playbackRig.sprays.get(spray.name);
      if (!node) continue;
      const lane = timeline?.signals.find((sig) => sig.name === spray.signal);
      const poses = overridePoses?.[spray.robot];
      const robot = robotByName(robots, spray.robot);
      const tcpName = robot?.desc.tcp_link ?? null;
      const tcpIndex =
        tcpName && robot
          ? robot.desc.links.findIndex((l) => l.name === tcpName)
          : -1;
      const on =
        !!lane &&
        !!poses &&
        tcpIndex >= 0 &&
        signalAt(lane.times, lane.values, playbackTime);
      if (!on || !poses) {
        node.visible = false;
        continue;
      }
      styleSpray(node, poses[tcpIndex], playbackTime);
    }
  }, [playing, playbackTime, overridePoses, timeline, robots, sprays]);

  return (
    <>
      {sprays.map((spray) => (
        <SprayNode key={spray.name} spray={spray} />
      ))}
    </>
  );
}

function SprayNode({ spray }: { spray: FlashMsg }) {
  const [length, radius] = spray.cone ?? [0.25, 0.08];
  useEffect(() => {
    return () => {
      playbackRig.sprays.delete(spray.name);
    };
  }, [spray.name]);
  // three's ConeGeometry stands along +Y with its apex at +height/2, so
  // translating by -height/2 puts the apex at the origin and the base at
  // -Y. The group carries the TCP pose and paint travels along its -Z, so
  // the base has to land there: rotating about X by +90 deg maps
  // (x, y, z) -> (x, -z, y), taking -Y to -Z. (Negating that angle points
  // the cone back up into the gun body, with only the footprint ring —
  // authored at -Z directly — left in the right place.)
  const geometry = useMemo(() => {
    const g = new THREE.ConeGeometry(radius, length, 24, 1, true);
    g.translate(0, -length / 2, 0);
    g.rotateX(Math.PI / 2);
    return g;
  }, [length, radius]);
  // The footprint: a ring at the cone's base — the pattern's diameter at
  // the standoff it was calibrated at. At nominal standoff the base sits
  // *on* the part, so the ring is drawn gizmo-style (no depth test) to
  // stay readable there; when the gun is closer or further it still marks
  // the calibrated spot, which is what the film model projects.
  const ring = useMemo(() => {
    const segments = 48;
    const pts: number[] = [];
    for (let i = 0; i <= segments; i++) {
      const a = (i / segments) * Math.PI * 2;
      pts.push(radius * Math.cos(a), radius * Math.sin(a), -length);
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
    // Built imperatively: JSX `<line>` collides with SVG's element type.
    const object = new THREE.Line(
      g,
      new THREE.LineBasicMaterial({
        color: "#e8f4ff",
        transparent: true,
        opacity: 0.9,
        depthTest: false,
      }),
    );
    object.renderOrder = 12;
    return object;
  }, [length, radius]);
  return (
    <group
      ref={(group: THREE.Group | null) => {
        if (group) {
          group.visible = false;
          playbackRig.sprays.set(spray.name, group);
        } else {
          playbackRig.sprays.delete(spray.name);
        }
      }}
    >
      <mesh geometry={geometry} renderOrder={9}>
        <meshBasicMaterial
          color="#bfe0ff"
          transparent
          opacity={0.3}
          depthWrite={false}
          side={THREE.DoubleSide}
        />
      </mesh>
      <primitive object={ring} />
    </group>
  );
}

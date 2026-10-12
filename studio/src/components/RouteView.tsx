import { useMemo } from "react";
import { Line } from "@react-three/drei";
import * as THREE from "three";

import { sampleRopePoint, type PlaybackTracks } from "../playback";
import type { RouteFindingMsg, RouteMsg, RouteSpanMsg } from "../protocol";
import { useStudioStore } from "../store";
import { authoredColor } from "../three/palette";

const ERROR_COLOR = "#d1495b";
const WARNING_COLOR = "#e0a03c";
const CLAMP_COLOR = "#8a8f98";

/** Arc length at every centerline point — the server's `s`, recomputed
 * from the same chords. */
function arcLengths(points: [number, number, number][]): number[] {
  const s = [0];
  for (let i = 1; i < points.length; i++) {
    const [ax, ay, az] = points[i - 1];
    const [bx, by, bz] = points[i];
    s.push(s[i - 1] + Math.hypot(bx - ax, by - ay, bz - az));
  }
  return s;
}

/** The centerline points of one span, both ends inclusive. */
function spanPoints(route: RouteMsg, span: RouteSpanMsg, s: number[]): [number, number, number][] {
  return route.centerline.filter((_, i) => s[i] >= span.from_s - 1e-9 && s[i] <= span.to_s + 1e-9);
}

/** What a playback does to the routes: the rope tracks it replays, by
 * name, and whether it moves anything a route hangs on (a robot, a carried
 * object). Derived from the playback, which only changes on a new bake. */
function usePlaybackRoutes(): { playing: boolean; moving: boolean; ropes: Set<string> } {
  const playback: PlaybackTracks | null = useStudioStore((s) => s.playback);
  return useMemo(
    () => ({
      playing: playback !== null,
      moving: playback !== null && (playback.robots.length > 0 || (playback.objects?.tracks.length ?? 0) > 0),
      ropes: new Set((playback?.ropes ?? []).map((r) => r.name)),
    }),
    [playback],
  );
}

/**
 * Whether a span's design tube steps aside during a playback: a span the
 * cycle replays is drawn by its rope, and a span that rides a robot is
 * known only at the scene's pose — the playback has moved the arm on, so
 * a tube there would hang where the arm no longer is. Outside a playback
 * every span is drawn.
 */
export function spanStepsAside(span: RouteSpanMsg, moving: boolean, ropes: Set<string>): boolean {
  return span.ropes.some((n) => ropes.has(n)) || (moving && span.kind !== "fixed");
}

/** The centerline of one span (both ends inclusive) as a tube geometry:
 * the polyline as a path of line curves, so the fillets the server
 * tessellated are drawn as they are and nothing is smoothed away. */
function spanTube(points: [number, number, number][], radius: number): THREE.TubeGeometry | null {
  const vs = points.map((p) => new THREE.Vector3(...p));
  const path = new THREE.CurvePath<THREE.Vector3>();
  for (let i = 1; i < vs.length; i++) {
    if (vs[i].distanceTo(vs[i - 1]) > 1e-9) path.add(new THREE.LineCurve3(vs[i - 1], vs[i]));
  }
  if (path.curves.length === 0) return null;
  return new THREE.TubeGeometry(path, Math.max(8, path.curves.length * 2), radius, 10, false);
}

function RouteTubes({
  route,
  focused,
  moving,
  ropes,
}: {
  route: RouteMsg;
  focused: boolean;
  moving: boolean;
  ropes: Set<string>;
}) {
  const color = useMemo(() => {
    const c = authoredColor(route.color);
    return focused ? c.clone().lerp(new THREE.Color("#ffffff"), 0.35) : c;
  }, [route.color, focused]);
  const spans = useMemo(() => {
    const s = arcLengths(route.centerline);
    return route.spans
      .map((span, k) => ({
        k,
        kind: span.kind,
        geometry: spanStepsAside(span, moving, ropes) ? null : spanTube(spanPoints(route, span, s), route.radius_m),
      }))
      .filter((x) => x.geometry !== null);
  }, [route, moving, ropes]);
  return (
    <group name={`route/${route.name}`}>
      {spans.map((span) => (
        <mesh key={span.k} name={`route/${route.name}#${span.k}`} geometry={span.geometry!} castShadow receiveShadow>
          <meshStandardMaterial
            color={color}
            roughness={0.7}
            metalness={0.05}
            transparent={span.kind !== "fixed"}
            opacity={span.kind !== "fixed" ? 0.55 : 1}
          />
        </mesh>
      ))}
    </group>
  );
}

/** The cables and tubes as they lie: a tube per span at the line's
 * radius, a span that rides a robot translucent (its shape at the scene's
 * pose). During a playback a span the cycle replays is left to its rope,
 * and a span that rides a robot is left out — ROUTING › design line draws
 * both as they were designed. */
export function RouteView() {
  const routes = useStudioStore((s) => s.routes);
  const focus = useStudioStore((s) => s.routeFocus);
  const { moving, ropes } = usePlaybackRoutes();
  return (
    <>
      {routes
        .filter((r) => r.centerline.length >= 2)
        .map((r) => (
          <RouteTubes key={r.name} route={r} focused={focus === r.name} moving={moving} ropes={ropes} />
        ))}
    </>
  );
}

/** The design line of the spans a playback hides: thin and dashed, at the
 * scene's pose — what the route was laid as, beside what the replay did. */
function DesignLines({ route, moving, ropes }: { route: RouteMsg; moving: boolean; ropes: Set<string> }) {
  const lines = useMemo(() => {
    const s = arcLengths(route.centerline);
    const color = `#${authoredColor(route.color).getHexString()}`;
    return route.spans
      .map((span, k) => ({ k, color, points: spanStepsAside(span, moving, ropes) ? spanPoints(route, span, s) : [] }))
      .filter((x) => x.points.length >= 2);
  }, [route, moving, ropes]);
  return (
    <group name={`route-design/${route.name}`}>
      {lines.map((l) => (
        <Line
          key={l.k}
          name={`route-design/${route.name}#${l.k}`}
          points={l.points}
          color={l.color}
          lineWidth={1.5}
          dashed
          dashSize={0.03}
          gapSize={0.02}
        />
      ))}
    </group>
  );
}

function Ring({ at, radius, color }: { at: [number, number, number]; radius: number; color: string }) {
  return (
    <mesh position={at} rotation={[Math.PI / 2, 0, 0]}>
      <torusGeometry args={[radius, Math.max(0.0015, radius * 0.12), 8, 32]} />
      <meshBasicMaterial color={color} depthTest={false} transparent opacity={0.9} />
    </mesh>
  );
}

function Exit({ at, dir, color }: { at: [number, number, number]; dir: [number, number, number]; color: string }) {
  const quaternion = useMemo(() => {
    const d = new THREE.Vector3(...dir).normalize();
    return new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), d);
  }, [dir]);
  const length = 0.05;
  const tip = useMemo(() => {
    const d = new THREE.Vector3(...dir).normalize().multiplyScalar(length / 2);
    return new THREE.Vector3(...at).add(d);
  }, [at, dir]);
  return (
    <mesh position={tip} quaternion={quaternion}>
      <coneGeometry args={[0.008, length, 10]} />
      <meshBasicMaterial color={color} depthTest={false} transparent opacity={0.85} />
    </mesh>
  );
}

/** The aids over the routes: a cone at each port along its exit, a ring
 * at each clamp, and a ring where a check found something — red for an
 * error, amber for a warning. Flat, no glow (the overlay idiom); the
 * camera pass hides them. */
export function RouteAidView() {
  const routes = useStudioStore((s) => s.routes);
  const focus = useStudioStore((s) => s.routeFocus);
  const designLine = useStudioStore((s) => s.routeDesignLine);
  const { playing, moving, ropes } = usePlaybackRoutes();
  // The marks of what rides a robot are known only at the scene's pose:
  // during a playback they go with the design line, not on their own.
  const atScenePose = !moving || designLine;
  const cycleFindings = useStudioStore((s) => s.timeline?.routeFindings ?? []);
  const playback = useStudioStore((s) => s.playback);
  const t = useStudioStore((s) => s.playbackTime);
  // A cycle finding shows at its moment: a ring that appears while the
  // playback is within a third of a second of it. One read off a replayed
  // rope rides the line — its ring is at that sample where the playback
  // has it now; one from the kinematics (a twist, a tension) has no place
  // on the line and stays where its moment put it.
  const now = cycleFindings.filter(
    (f) => f.position && f.t_s !== null && f.t_s !== undefined && Math.abs(f.t_s - t) < 0.34 && f.severity !== "info",
  );
  const onTheLine = (f: RouteFindingMsg): [number, number, number] | null => {
    if (!f.rope || f.sample === null || f.sample === undefined) return null;
    const track = playback?.ropes?.find((r) => r.name === f.rope);
    return track ? sampleRopePoint(track, t, f.sample) : null;
  };
  // A static finding on a span that rides the arm is known only at the
  // scene's pose, like the clamps there: during a playback it goes with
  // the design line, not on its own.
  const ridesArm = (r: RouteMsg, f: RouteFindingMsg): boolean =>
    f.s_m !== null &&
    f.s_m !== undefined &&
    r.spans.some((sp) => sp.kind !== "fixed" && f.s_m! >= sp.from_s - 1e-6 && f.s_m! <= sp.to_s + 1e-6);
  return (
    <>
      {now.map((f, i) => (
        <Ring
          key={`cycle/${f.code}/${i}`}
          at={onTheLine(f) ?? f.position!}
          radius={0.04}
          color={f.severity === "error" ? ERROR_COLOR : WARNING_COLOR}
        />
      ))}
      {routes.map((r) => {
        const dim = focus !== null && focus !== r.name;
        return (
          <group key={r.name} name={`route-aid/${r.name}`} visible={!dim}>
            {playing && designLine && <DesignLines route={r} moving={moving} ropes={ropes} />}
            {r.anchors.filter((a) => atScenePose || !a.moving).map((a) =>
              a.kind === "port" && a.exit ? (
                <Exit key={a.name} at={a.position} dir={a.exit} color={a.moving ? WARNING_COLOR : "#4a7ab8"} />
              ) : (
                <Ring key={a.name} at={a.position} radius={r.radius_m * 2.2 + 0.004} color={CLAMP_COLOR} />
              ),
            )}
            {r.findings
              .filter((f) => f.position !== null && f.severity !== "info" && (atScenePose || !ridesArm(r, f)))
              .map((f, i) => (
                <Ring
                  key={`${f.code}/${i}`}
                  at={f.position!}
                  radius={r.radius_m * 3 + 0.02}
                  color={f.severity === "error" ? ERROR_COLOR : WARNING_COLOR}
                />
              ))}
          </group>
        );
      })}
    </>
  );
}

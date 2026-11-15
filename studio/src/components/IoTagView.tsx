import { useEffect, useMemo, useRef } from "react";
import { useFrame } from "@react-three/fiber";
import * as THREE from "three";

import type { DeviceMsg, SensorMsg } from "../protocol";
import { readingsOf } from "../readings";
import { anchorKey, tagGroups, tagLine, type TagAnchor, type TagLine } from "../ioTags";
import { linkKey, playbackRig } from "../playbackRig";
import { ioTagsShown, useStudioStore } from "../store";
import { cameraRig } from "../three/cameraRig";
import { roundRect } from "../xr/TextLabel";
import { parkedFrame } from "./SensorView";

/**
 * The 3D I/O tags: a board over each resident of the I/O map — a sensor,
 * a device, a lamp, a robot — listing its points the way the wiring
 * names them (`X20 beam`), lit green while the point is on, a word with
 * its value. The same address the ladder, the I/O table and the timing
 * chart show, on the thing itself.
 *
 * An annotation (flat colours, no animation, drawn over the scene): it
 * is hidden from every camera pass like the authoring aids, except a
 * video export that asks for it (`cameraRig.labels`). A board faces
 * whichever camera renders it — the viewport, each eye of a headset, an
 * export camera — at a constant share of that camera's view height, so it
 * reads the same near and far.
 */

/** A line's height as a share of the view height. */
const LINE_FRAC = 0.021;
/** How far a board floats over its anchor, in lines. */
const LIFT = 0.4;
const FONT = 30;
const LINE_H = 44;
const PAD = 10;
const GAP = 10;

const BG = "rgba(20, 22, 28, 0.86)";
const TEXT = "#d6dae2";
const MUTED = "#858d9b";
const HOT = "#7fb4ff";
const CHIP: Record<TagLine["state"], [string, string]> = {
  on: ["#3fbf6f", "#0f1115"],
  off: ["#3a414f", TEXT],
  value: ["#2b4a66", TEXT],
  none: ["#2a2f3a", MUTED],
};
const FORCED = "#b8a24a";
/** A reading that parts from the model: the dock's deviation red. */
const DIFFERS = "#ff7a6b";

const camPos = new THREE.Vector3();
const camQuat = new THREE.Quaternion();
const camUp = new THREE.Vector3();
const scale = new THREE.Vector3();
const lifted = new THREE.Vector3();

class TagBoard extends THREE.Mesh<THREE.PlaneGeometry, THREE.MeshBasicMaterial> {
  /** World point the board stands on (its bottom centre). */
  readonly anchor = new THREE.Vector3();
  private readonly canvas: HTMLCanvasElement;
  private readonly ctx: CanvasRenderingContext2D;
  private texture: THREE.CanvasTexture;
  private signature = "";
  private aspect = 1;
  /** The drawn content's width over its height (the canvas is padded). */
  contentAspect = 1;
  rows = 1;
  /** How far the declutter pass lifts the board, as a share of the view
   * height — boards over neighbouring residents stack instead of
   * overprinting each other. */
  shift = 0;

  constructor() {
    const canvas = document.createElement("canvas");
    canvas.width = 64;
    canvas.height = 64;
    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    const geometry = new THREE.PlaneGeometry(1, 1);
    geometry.translate(0, 0.5, 0);
    super(
      geometry,
      new THREE.MeshBasicMaterial({
        map: texture,
        transparent: true,
        depthTest: false,
        depthWrite: false,
        toneMapped: false,
      }),
    );
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d")!;
    this.texture = texture;
    this.renderOrder = 10;
    this.frustumCulled = false;
    this.matrixAutoUpdate = false;
    this.raycast = () => {};
    this.onBeforeRender = (_renderer, _scene, camera) => this.face(camera);
  }

  /** Faces `camera` from the anchor, sized to a share of its view. */
  private face(camera: THREE.Camera): void {
    const e = camera.projectionMatrix.elements;
    camPos.setFromMatrixPosition(camera.matrixWorld);
    const ortho = (camera as THREE.OrthographicCamera).isOrthographicCamera === true;
    const viewH = ortho ? 2 / e[5] : (2 * camPos.distanceTo(this.anchor)) / e[5];
    const h = viewH * LINE_FRAC * this.rows;
    camQuat.setFromRotationMatrix(camera.matrixWorld);
    camUp.set(0, 1, 0).applyQuaternion(camQuat);
    lifted.copy(this.anchor).addScaledVector(camUp, viewH * (LINE_FRAC * LIFT + this.shift));
    this.matrixWorld.compose(lifted, camQuat, scale.set(h * this.aspect, h, 1));
  }

  /** Redraws when what the board says changed. */
  draw(lines: TagLine[], hot: string | null): void {
    const signature = JSON.stringify([lines, hot]);
    if (signature === this.signature) return;
    this.signature = signature;
    const { ctx } = this;
    ctx.font = `500 ${FONT}px system-ui, sans-serif`;
    const widths = lines.map((l) => {
      const chip = l.address ? ctx.measureText(l.address).width + 2 * GAP : 0;
      const value = l.value ? ctx.measureText(l.value).width + GAP : 0;
      const forced = l.forced ? ctx.measureText("F").width + 2 * GAP : 0;
      const differs = l.deviates ? ctx.measureText("≠ model").width + 2 * GAP : 0;
      return chip + (chip ? GAP : 0) + ctx.measureText(l.name).width + value + forced + differs;
    });
    // Widths in 64 px steps, so a changing value rarely resizes the board.
    const w = Math.min(2048, Math.ceil((Math.max(...widths, 1) + 2 * PAD) / 64) * 64);
    const h = PAD * 2 + lines.length * LINE_H;
    if (w !== this.canvas.width || h !== this.canvas.height) {
      this.canvas.width = w;
      this.canvas.height = h;
      // A canvas that changed size needs fresh texture storage.
      this.texture.dispose();
      this.texture = new THREE.CanvasTexture(this.canvas);
      this.texture.colorSpace = THREE.SRGBColorSpace;
      this.material.map = this.texture;
      this.material.needsUpdate = true;
    }
    this.aspect = w / h;
    this.rows = lines.length + (2 * PAD) / LINE_H;
    ctx.clearRect(0, 0, w, h);
    const content = Math.max(...widths, 1) + 2 * PAD;
    this.contentAspect = content / h;
    // The content centred on the anchor; the padding is transparent.
    const ox = Math.floor((w - content) / 2);
    ctx.save();
    ctx.translate(ox, 0);
    ctx.fillStyle = BG;
    roundRect(ctx, 2, 2, content - 4, h - 4, 12);
    ctx.fill();
    if (hot !== null && lines.some((l) => l.key === hot)) {
      ctx.strokeStyle = HOT;
      ctx.lineWidth = 4;
      roundRect(ctx, 2, 2, content - 4, h - 4, 12);
      ctx.stroke();
    }
    ctx.font = `500 ${FONT}px system-ui, sans-serif`;
    ctx.textBaseline = "middle";
    lines.forEach((l, i) => {
      const y = PAD + i * LINE_H + LINE_H / 2;
      let x = PAD;
      if (l.address) {
        const [fill, ink] = CHIP[l.state];
        const cw = ctx.measureText(l.address).width + 2 * GAP;
        ctx.fillStyle = fill;
        roundRect(ctx, x, y - LINE_H / 2 + 5, cw, LINE_H - 10, 8);
        ctx.fill();
        ctx.fillStyle = ink;
        ctx.fillText(l.address, x + GAP, y + 1);
        x += cw + GAP;
      }
      ctx.fillStyle = l.key === hot ? HOT : l.address ? TEXT : l.state === "on" ? CHIP.on[0] : MUTED;
      ctx.fillText(l.name, x, y + 1);
      x += ctx.measureText(l.name).width;
      if (l.value) {
        ctx.fillStyle = "#9fd0ff";
        ctx.fillText(l.value, x + GAP, y + 1);
        x += GAP + ctx.measureText(l.value).width;
      }
      if (l.forced) {
        ctx.fillStyle = FORCED;
        ctx.fillText("F", x + 2 * GAP, y + 1);
        x += 2 * GAP + ctx.measureText("F").width;
      }
      if (l.deviates) {
        ctx.fillStyle = DIFFERS;
        ctx.fillText("≠ model", x + 2 * GAP, y + 1);
      }
    });
    ctx.restore();
    this.texture.needsUpdate = true;
  }

  dispose(): void {
    this.geometry.dispose();
    this.material.dispose();
    this.texture.dispose();
  }
}

/** The thin line from a lifted board down to its resident. */
class Leader extends THREE.Line<THREE.BufferGeometry, THREE.LineBasicMaterial> {
  constructor(readonly board: TagBoard) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(6), 3));
    super(
      geometry,
      new THREE.LineBasicMaterial({ color: MUTED, depthTest: false, depthWrite: false, transparent: true, opacity: 0.85 }),
    );
    this.renderOrder = 9;
    this.frustumCulled = false;
    this.matrixAutoUpdate = false;
    this.raycast = () => {};
    this.onBeforeRender = (_renderer, _scene, camera) => {
      const e = camera.projectionMatrix.elements;
      camPos.setFromMatrixPosition(camera.matrixWorld);
      const ortho = (camera as THREE.OrthographicCamera).isOrthographicCamera === true;
      const viewH = ortho ? 2 / e[5] : (2 * camPos.distanceTo(this.board.anchor)) / e[5];
      camQuat.setFromRotationMatrix(camera.matrixWorld);
      camUp.set(0, 1, 0).applyQuaternion(camQuat);
      const a = this.board.anchor;
      // Unlifted, the board stands on its anchor: no line.
      const up = this.board.shift > 1e-3 ? viewH * (LINE_FRAC * LIFT + this.board.shift) : 0;
      lifted.copy(a).addScaledVector(camUp, up);
      const pos = this.geometry.getAttribute("position") as THREE.BufferAttribute;
      pos.setXYZ(0, a.x, a.y, a.z);
      pos.setXYZ(1, lifted.x, lifted.y, lifted.z);
      pos.needsUpdate = true;
    };
  }

  dispose(): void {
    this.geometry.dispose();
    this.material.dispose();
  }
}

// ------------------------------------------------------------ declutter

const ndc = new THREE.Vector3();

/** Lifts boards that would overprint each other on the screen of
 * `camera` (the viewport's): lowest anchors stay, the ones above stack
 * over them. Shifts are shares of the view height, so another camera
 * (an export, a headset) gets the same arrangement, near enough. */
function declutter(boards: TagBoard[], camera: THREE.Camera): void {
  const aspect = (camera as THREE.PerspectiveCamera).aspect || 1;
  const placed: { x0: number; x1: number; y0: number; y1: number }[] = [];
  const items = boards
    .filter((b) => b.visible)
    .map((b) => {
      ndc.copy(b.anchor).project(camera);
      return { b, x: ndc.x, y: ndc.y, front: ndc.z <= 1 };
    })
    .filter((it) => it.front)
    .sort((a, b) => a.y - b.y);
  for (const { b, x, y } of items) {
    const h = 2 * LINE_FRAC * b.rows;
    const w = (h * b.contentAspect) / aspect;
    const base = y + 2 * LINE_FRAC * LIFT;
    let y0 = base;
    for (let guard = 0; guard < 64; guard++) {
      const hit = placed.find(
        (r) => x - w / 2 < r.x1 && x + w / 2 > r.x0 && y0 < r.y1 && y0 + h > r.y0,
      );
      if (!hit) break;
      y0 = hit.y1 + 2 * LINE_FRAC * 0.15;
    }
    placed.push({ x0: x - w / 2, x1: x + w / 2, y0, y1: y0 + h });
    b.shift = (y0 - base) / 2;
  }
}

// ------------------------------------------------------------- anchors

const local = new THREE.Vector3();
const box = new THREE.Box3();
const frame = new THREE.Matrix4();

/** A sensor's tag point in its own (world or vehicle) frame: on top of a
 * zone, at the middle of a beam, at a vision sensor's camera. */
function sensorPoint(s: SensorMsg, out: THREE.Vector3): boolean {
  const k = s.kind;
  if (k.kind === "zone") {
    const q = new THREE.Quaternion(...k.pose.quaternion);
    out.set(0, 0, k.size[2] / 2).applyQuaternion(q).add(local.set(...k.pose.position));
    return true;
  }
  if (k.kind === "beam") {
    out.set((k.from[0] + k.to[0]) / 2, (k.from[1] + k.to[1]) / 2, (k.from[2] + k.to[2]) / 2);
    return true;
  }
  if (k.kind === "vision") {
    const node = cameraRig.nodes.get(k.camera);
    if (!node) return false;
    node.getWorldPosition(out);
    return true;
  }
  return false;
}

/** The top centre of an obstacle as drawn now (it may ride a device). */
function obstacleTop(name: string, out: THREE.Vector3): boolean {
  const node = playbackRig.obstacles.get(name);
  if (!node) return false;
  box.setFromObject(node);
  if (box.isEmpty()) return false;
  box.getCenter(out);
  out.z = box.max.z;
  return true;
}

function resolveAnchor(
  anchor: TagAnchor,
  sensors: SensorMsg[],
  devices: DeviceMsg[],
  robotBases: Map<string, [number, number, number]>,
  out: THREE.Vector3,
): boolean {
  switch (anchor.kind) {
    case "sensor": {
      const s = sensors.find((x) => x.name === anchor.name);
      if (!s) return false;
      if (s.kind.kind === "vision") return sensorPoint(s, out);
      if (!sensorPoint(s, out)) return false;
      if (s.mount) {
        const mounted = playbackRig.mounts.get(s.name)?.node;
        if (mounted) {
          mounted.updateWorldMatrix(true, false);
          out.applyMatrix4(mounted.matrixWorld);
        } else {
          const d = devices.find((x) => x.name === s.mount);
          const parked = d ? parkedFrame(d) : null;
          if (parked) {
            out.applyMatrix4(frame.compose(parked.position, parked.quaternion, scale.set(1, 1, 1)));
          }
        }
      }
      return true;
    }
    case "device": {
      const d = devices.find((x) => x.name === anchor.name);
      if (!d) return false;
      const k = d.kind;
      if (k.kind === "conveyor") {
        const q = new THREE.Quaternion(...k.zone_pose.quaternion);
        out.set(0, 0, k.zone_size[2] / 2).applyQuaternion(q).add(local.set(...k.zone_pose.position));
        return true;
      }
      const body =
        k.kind === "linear_axis" ? k.objects : k.kind === "vehicle" ? k.body : k.kind === "lift" ? k.car : [];
      return body.length > 0 && obstacleTop(body[0], out);
    }
    case "obstacle":
      return obstacleTop(anchor.name, out);
    case "robot": {
      // A URDF robot's base link as drawn (it may ride a vehicle); a USD
      // robot poses itself through its own library, so its authored base.
      const base = playbackRig.links.get(linkKey(anchor.name, 0));
      if (base) {
        base.getWorldPosition(out);
        return true;
      }
      const p = robotBases.get(anchor.name);
      if (!p) return false;
      out.set(...p);
      return true;
    }
  }
}

// ---------------------------------------------------------------- view

export function IoTagView() {
  const mode = useStudioStore((s) => s.ioTags);
  const points = useStudioStore((s) => s.io.points);
  const nodes = useStudioStore((s) => s.io.io.nodes);
  const sensors = useStudioStore((s) => s.sensors);
  const devices = useStudioStore((s) => s.devices);
  const flashes = useStudioStore((s) => s.flashes);
  const robots = useStudioStore((s) => s.robots);
  const timeline = useStudioStore((s) => s.timeline);
  const playbackTime = useStudioStore((s) => s.playbackTime);
  const plant = useStudioStore((s) => s.plant);
  const analogs = useStudioStore((s) => s.analogs);
  const readings = useStudioStore((s) => readingsOf(s.plant, s.timeline));
  const hot = useStudioStore((s) => s.highlightLane);
  const shown = ioTagsShown(mode, points);

  const groups = useMemo(
    () =>
      tagGroups(
        points,
        {
          sensors: sensors.map((s) => s.name),
          devices,
          lamps: flashes.filter((f) => f.kind === "lamp"),
          robots: robots.map((r) => r.desc.name),
          analogs,
        },
        nodes,
      ),
    [points, sensors, devices, flashes, robots, nodes, analogs],
  );

  const robotBases = useMemo(
    () => new Map(robots.map((r) => [r.desc.name, r.basePose.position] as const)),
    [robots],
  );
  const root = useRef<THREE.Group | null>(null);
  const boards = useRef(new Map<string, TagBoard>());
  const leaders = useRef(new Map<string, Leader>());

  // One board per resident: made and disposed as the residents change.
  useEffect(() => {
    const g = root.current;
    if (!g) return;
    const want = new Set(groups.map((x) => anchorKey(x.anchor)));
    for (const [k, b] of boards.current) {
      if (!want.has(k)) {
        const l = leaders.current.get(k);
        g.remove(b);
        b.dispose();
        boards.current.delete(k);
        if (l) {
          g.remove(l);
          l.dispose();
          leaders.current.delete(k);
        }
      }
    }
    for (const x of groups) {
      const k = anchorKey(x.anchor);
      if (!boards.current.has(k)) {
        const b = new TagBoard();
        const l = new Leader(b);
        boards.current.set(k, b);
        leaders.current.set(k, l);
        g.add(l);
        g.add(b);
      }
    }
  }, [groups]);
  useEffect(
    () => () => {
      for (const b of boards.current.values()) b.dispose();
      for (const l of leaders.current.values()) l.dispose();
      boards.current.clear();
      leaders.current.clear();
    },
    [],
  );

  // What each board says at the playhead.
  useEffect(() => {
    if (!shown) return;
    for (const x of groups) {
      const b = boards.current.get(anchorKey(x.anchor));
      if (!b) continue;
      b.draw(
        x.points.map((p) => tagLine(p, timeline, playbackTime, plant, analogs, readings)),
        hot,
      );
    }
  }, [shown, groups, timeline, playbackTime, plant, hot, analogs, readings]);

  // Where each board stands, every frame (devices carry their residents),
  // then the stacking that keeps neighbours from overprinting.
  useFrame(({ camera }) => {
    if (!shown) return;
    for (const x of groups) {
      const k = anchorKey(x.anchor);
      const b = boards.current.get(k);
      if (!b) continue;
      b.visible = resolveAnchor(x.anchor, sensors, devices, robotBases, b.anchor);
      const l = leaders.current.get(k);
      if (l) l.visible = b.visible;
    }
    declutter([...boards.current.values()], camera);
  });

  return (
    <group
      visible={shown}
      ref={(node) => {
        if (root.current) cameraRig.labels.delete(root.current);
        if (node) cameraRig.labels.add(node);
        root.current = node;
      }}
    />
  );
}

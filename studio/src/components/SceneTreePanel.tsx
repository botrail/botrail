import { useMemo, useState } from "react";

import type { FrameMsg, ObstacleMsg, PartEntry } from "../protocol";
import { collidingObstacleNames, useStudioStore } from "../store";
import {
  sendRemoveCamera,
  sendRemoveLidar,
  sendRemoveDevice,
  sendRemoveIoNode,
  sendRemoveSensor,
  sendRobotBasePose,
  sendSetObstacleEnabled,
} from "../ws";
import { Section } from "./Section";

/**
 * Robots plus a hierarchy over obstacle/frame names (prim paths from USD
 * imports group naturally; flat names sit at the root). Per robot: select
 * (focus its TCP) and place its base. Per obstacle: show/hide (display
 * only, client-side) and a collision toggle (server-side `enabled`).
 */
export function SceneTreePanel() {
  const robots = useStudioStore((s) => s.robots);
  const selectedRobot = useStudioStore((s) => s.selectedRobot);
  const selectTcp = useStudioStore((s) => s.selectTcp);
  const selectRobot = useStudioStore((s) => s.selectRobot);
  const obstacles = useStudioStore((s) => s.obstacles);
  const frames = useStudioStore((s) => s.frames);
  const sensors = useStudioStore((s) => s.sensors);
  const devices = useStudioStore((s) => s.devices);
  const selection = useStudioStore((s) => s.selection);
  const selectSensor = useStudioStore((s) => s.selectSensor);
  const selectDevice = useStudioStore((s) => s.selectDevice);
  const cameras = useStudioStore((s) => s.cameras);
  const selectCamera = useStudioStore((s) => s.selectCamera);
  const lidars = useStudioStore((s) => s.lidars);
  const selectLidar = useStudioStore((s) => s.selectLidar);
  const ioNodes = useStudioStore((s) => s.io.io.nodes);
  const ioPoints = useStudioStore((s) => s.io.points);
  const selectIoNode = useStudioStore((s) => s.selectIoNode);
  const parts = useStudioStore((s) => s.parts);
  const partIndex = useMemo(() => indexParts(parts), [parts]);
  if (
    robots.length === 0 &&
    obstacles.length === 0 &&
    frames.length === 0 &&
    sensors.length === 0 &&
    devices.length === 0 &&
    ioNodes.length === 0
  ) {
    return null;
  }
  return (
    <Section
      id="scene"
      title="Scene"
      badge={
        <span className="badge muted">
          {robots.length > 1 ? `${robots.length} robots · ` : ""}
          {obstacles.length} obj · {frames.length} frames
        </span>
      }
    >
      <div className="scene-tree">
        {robots.map((r) => {
          const name = r.desc.name;
          return (
            <div
              key={name}
              className={`tree-row${selectedRobot === name ? " selected" : ""}`}
            >
              <span className="tree-twist" />
              <span
                className="tree-label"
                title="select this robot"
                onClick={() => selectTcp(name)}
              >
                {"\u{1F916} "}
                {name}
              </span>
              <PartBadge entry={partIndex.get(`robot:${name}`)} />
              <button
                className="tree-toggle"
                title="place robot base"
                onClick={() => selectRobot(name)}
              >
                ⌖
              </button>
            </div>
          );
        })}
      </div>
      <Tree obstacles={obstacles} frames={frames} partIndex={partIndex} />
      {(sensors.length > 0 ||
        devices.length > 0 ||
        cameras.length > 0 ||
        lidars.length > 0) && (
        <div className="scene-tree">
          {sensors.map((s) => (
            <div
              key={s.name}
              className={`tree-row${
                selection.type === "sensor" && selection.name === s.name
                  ? " selected"
                  : ""
              }`}
            >
              <span className="tree-twist" />
              <span
                className="tree-label"
                title={`${s.kind.kind} sensor — click to edit`}
                onClick={() => selectSensor(s.name)}
              >
                {"\u{1F4E1} "}
                {s.name}
              </span>
              <PartBadge entry={partIndex.get(`sensor:${s.name}`)} />
              <button
                className="tree-toggle"
                title="remove sensor"
                onClick={() => sendRemoveSensor(s.name)}
              >
                ×
              </button>
            </div>
          ))}
          {devices.map((d) => (
            <div
              key={d.name}
              className={`tree-row${
                selection.type === "device" && selection.name === d.name
                  ? " selected"
                  : ""
              }`}
            >
              <span className="tree-twist" />
              <span
                className="tree-label"
                title={`${d.kind.kind} — click to edit`}
                onClick={() => selectDevice(d.name)}
              >
                {"\u{2699} "}
                {d.name}
              </span>
              <PartBadge entry={partIndex.get(`device:${d.name}`)} />
              <button
                className="tree-toggle"
                title="remove device"
                onClick={() => sendRemoveDevice(d.name)}
              >
                ×
              </button>
            </div>
          ))}
          {cameras.map((c) => (
            <div
              key={c.name}
              className={`tree-row${
                selection.type === "camera" && selection.name === c.name
                  ? " selected"
                  : ""
              }`}
            >
              <span className="tree-twist" />
              <span
                className="tree-label"
                title={`${c.mount.kind} camera — click to edit`}
                onClick={() => selectCamera(c.name)}
              >
                {"\u{1F3A5} "}
                {c.name}
              </span>
              <PartBadge entry={partIndex.get(`camera:${c.name}`)} />
              <button
                className="tree-toggle"
                title="remove camera"
                onClick={() => sendRemoveCamera(c.name)}
              >
                ×
              </button>
            </div>
          ))}
          {lidars.map((l) => (
            <div
              key={l.name}
              className={`tree-row${
                selection.type === "lidar" && selection.name === l.name
                  ? " selected"
                  : ""
              }`}
            >
              <span className="tree-twist" />
              <span
                className="tree-label"
                title={`${l.mount.kind} lidar — click to edit`}
                onClick={() => selectLidar(l.name)}
              >
                {"\u{1F300} "}
                {l.name}
              </span>
              <PartBadge entry={partIndex.get(`lidar:${l.name}`)} />
              <button
                className="tree-toggle"
                title="remove lidar"
                onClick={() => sendRemoveLidar(l.name)}
              >
                ×
              </button>
            </div>
          ))}
        </div>
      )}
      {/* I/O nodes — controllers and stations of the assignment layer,
          authored from Python. Read-only here: selecting one shows its
          channels and what is bound to them in the Layout inspector. */}
      {ioNodes.length > 0 && (
        <div className="scene-tree">
          {ioNodes.map((n) => {
            const bound = ioPoints.filter((p) => p.node === n.name).length;
            const kind = ioNodeKindLabel(n.kind.kind);
            return (
              <div
                key={n.name}
                className={`tree-row${
                  selection.type === "io_node" && selection.name === n.name
                    ? " selected"
                    : ""
                }`}
              >
                <span className="tree-twist" />
                <span
                  className="tree-label"
                  title={`${kind}${n.uplink ? ` — uplink ${n.uplink.parent}` : ""} — click for details`}
                  onClick={() => selectIoNode(n.name)}
                >
                  {"\u{1F50C} "}
                  {n.name}
                  <span className="seq-cond"> · {kind}</span>
                </span>
                <PartBadge entry={partIndex.get(`io_node:${n.name}`)} />
                <span className="seq-cond" title="bound points / channels">
                  {bound}/{(n.channels ?? []).length}
                </span>
                <button
                  className="tree-toggle"
                  title="remove I/O node"
                  onClick={() => sendRemoveIoNode(n.name)}
                >
                  ×
                </button>
              </div>
            );
          })}
        </div>
      )}
    </Section>
  );
}

/** `plc` → "PLC", `remote_io` → "remote I/O", ... */
export function ioNodeKindLabel(kind: string): string {
  switch (kind) {
    case "plc":
      return "PLC";
    case "safety_plc":
      return "safety PLC";
    case "remote_io":
      return "remote I/O";
    case "robot_controller":
      return "robot controller";
    default:
      return kind;
  }
}

/** `kind:target` → the pinned part, for O(1) lookups while rendering. */
function indexParts(parts: PartEntry[]): Map<string, PartEntry> {
  const index = new Map<string, PartEntry>();
  for (const p of parts) index.set(`${p.kind}:${p.target}`, p);
  return index;
}

/** The short label a part reads as on the tree: model, else catalog id,
 * else maker, else category. */
export function partLabel(entry: PartEntry): string {
  const p = entry.part;
  return p.model ?? p.catalog?.id ?? p.manufacturer ?? p.category ?? "part";
}

/** The full identity for the tooltip. */
export function partTitle(entry: PartEntry): string {
  const p = entry.part;
  const bits = [p.manufacturer, p.model].filter(Boolean).join(" ");
  const cat = p.catalog ? `${p.catalog.id}${p.catalog.revision ? `@${p.catalog.revision}` : ""}` : "";
  const qty = p.qty !== 1 ? ` ×${p.qty}` : "";
  const head = [bits, cat && `(${cat})`].filter(Boolean).join(" ") || "part";
  return `${head}${qty}${p.description ? ` — ${p.description}` : ""}`;
}

/** The model badge a pinned resident wears on the scene tree. Display
 * only — parts are authored from Python (`scene.set_part`). */
function PartBadge({ entry }: { entry: PartEntry | undefined }) {
  if (!entry) return null;
  return (
    <span className="badge muted part-badge" title={partTitle(entry)}>
      {partLabel(entry)}
    </span>
  );
}

interface TreeNode {
  label: string;
  path: string;
  children: Map<string, TreeNode>;
  obstacle?: ObstacleMsg;
  frame?: FrameMsg;
}

function buildTree(obstacles: ObstacleMsg[], frames: FrameMsg[]): TreeNode {
  const root: TreeNode = { label: "", path: "", children: new Map() };
  const insert = (name: string) => {
    let node = root;
    for (const part of name.split("/").filter(Boolean)) {
      let child = node.children.get(part);
      if (!child) {
        child = {
          label: part,
          path: `${node.path}/${part}`,
          children: new Map(),
        };
        node.children.set(part, child);
      }
      node = child;
    }
    return node;
  };
  for (const o of obstacles) insert(o.name).obstacle = o;
  for (const f of frames) insert(f.name).frame = f;
  return root;
}

function Tree({
  obstacles,
  frames,
  partIndex,
}: {
  obstacles: ObstacleMsg[];
  frames: FrameMsg[];
  partIndex: Map<string, PartEntry>;
}) {
  const root = useMemo(() => buildTree(obstacles, frames), [obstacles, frames]);
  // Colliding obstacles read red straight in the tree — the tree is the
  // one obstacle list, so this is where the eye goes.
  const collisions = useStudioStore((s) => s.collisions);
  const colliding = useMemo(() => collidingObstacleNames(collisions), [collisions]);
  return (
    <div className="scene-tree">
      {[...root.children.values()].map((n) => (
        <TreeRow key={n.path} node={n} depth={0} colliding={colliding} partIndex={partIndex} />
      ))}
    </div>
  );
}

function TreeRow({
  node,
  depth,
  colliding,
  partIndex,
}: {
  node: TreeNode;
  depth: number;
  colliding: Set<string>;
  partIndex: Map<string, PartEntry>;
}) {
  const [open, setOpen] = useState(depth < 2);
  const selection = useStudioStore((s) => s.selection);
  const selectedRobot = useStudioStore((s) => s.selectedRobot);
  const selectObstacle = useStudioStore((s) => s.selectObstacle);
  const selectGroup = useStudioStore((s) => s.selectGroup);
  const hiddenObstacles = useStudioStore((s) => s.hiddenObstacles);
  const toggleObstacleHidden = useStudioStore((s) => s.toggleObstacleHidden);

  const kids = [...node.children.values()];
  const o = node.obstacle;
  // A node with children is a subtree from the imported stage — a machine,
  // not a part of one. Selecting it moves everything under it as one body,
  // which is what "the pedestal" means when the pedestal is three plates.
  const isGroup = kids.length > 0;
  const selected = isGroup
    ? selection.type === "group" && selection.path === node.path
    : o && selection.type === "obstacle" && selection.name === o.name;
  const hidden = o ? hiddenObstacles.has(o.name) : false;
  // A part pinned to this prim, or to the group it heads (`<path>/…`).
  // Group targets are name prefixes: USD prim paths keep the tree's
  // leading slash, `add_box("fence/p0")` names do not — try both.
  const bare = node.path.replace(/^\//, "");
  const part =
    (isGroup
      ? (partIndex.get(`group:${node.path}`) ?? partIndex.get(`group:${bare}`))
      : undefined) ?? (o ? partIndex.get(`obstacle:${o.name}`) : undefined);

  return (
    <div>
      <div
        className={`tree-row${selected ? " selected" : ""}`}
        style={{ paddingLeft: `${depth * 12}px` }}
      >
        {kids.length > 0 ? (
          <button className="tree-twist" onClick={() => setOpen(!open)}>
            {open ? "▾" : "▸"}
          </button>
        ) : (
          <span className="tree-twist" />
        )}
        <span
          className={`tree-label${o && colliding.has(o.name) ? " bad" : ""}`}
          onClick={() =>
            isGroup ? selectGroup(node.path) : o && selectObstacle(o.name)
          }
          title={isGroup ? `${node.path} — move as one` : node.path}
        >
          {node.label}
        </span>
        <PartBadge entry={part} />
        {o && (
          <>
            {o.attached_to && (
              <span
                className="tree-toggle"
                title={`attached to ${o.attached_to.link}`}
              >
                🧲
              </span>
            )}
            <button
              className="tree-toggle"
              title={hidden ? "show" : "hide (display only)"}
              onClick={() => toggleObstacleHidden(o.name)}
            >
              {hidden ? "🙈" : "👁"}
            </button>
            <input
              type="checkbox"
              title="collision checking"
              checked={o.enabled}
              onChange={(e) => sendSetObstacleEnabled(o.name, e.target.checked)}
            />
          </>
        )}
        {node.frame && selectedRobot !== null && (
          <button
            className="tree-toggle"
            title={`place ${selectedRobot} base here`}
            onClick={() =>
              node.frame && sendRobotBasePose(selectedRobot, node.frame.pose)
            }
          >
            ⌖
          </button>
        )}
      </div>
      {open &&
        kids.map((n) => (
          <TreeRow
            key={n.path}
            node={n}
            depth={depth + 1}
            colliding={colliding}
            partIndex={partIndex}
          />
        ))}
    </div>
  );
}

import { useMemo, useState } from "react";

import type { FrameMsg, ObstacleMsg } from "../protocol";
import { collidingObstacleNames, useStudioStore } from "../store";
import { sendRemoveDevice, sendRemoveSensor, sendRobotBasePose, sendSetObstacleEnabled } from "../ws";
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
  if (
    robots.length === 0 &&
    obstacles.length === 0 &&
    frames.length === 0 &&
    sensors.length === 0 &&
    devices.length === 0
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
      <Tree obstacles={obstacles} frames={frames} />
      {(sensors.length > 0 || devices.length > 0) && (
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
              <button
                className="tree-toggle"
                title="remove device"
                onClick={() => sendRemoveDevice(d.name)}
              >
                ×
              </button>
            </div>
          ))}
        </div>
      )}
    </Section>
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
}: {
  obstacles: ObstacleMsg[];
  frames: FrameMsg[];
}) {
  const root = useMemo(() => buildTree(obstacles, frames), [obstacles, frames]);
  // Colliding obstacles read red straight in the tree — the tree is the
  // one obstacle list, so this is where the eye goes.
  const collisions = useStudioStore((s) => s.collisions);
  const colliding = useMemo(() => collidingObstacleNames(collisions), [collisions]);
  return (
    <div className="scene-tree">
      {[...root.children.values()].map((n) => (
        <TreeRow key={n.path} node={n} depth={0} colliding={colliding} />
      ))}
    </div>
  );
}

function TreeRow({
  node,
  depth,
  colliding,
}: {
  node: TreeNode;
  depth: number;
  colliding: Set<string>;
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
          <TreeRow key={n.path} node={n} depth={depth + 1} colliding={colliding} />
        ))}
    </div>
  );
}

import { useMemo, useState } from "react";

import type { FrameMsg, ObstacleMsg } from "../protocol";
import { useStudioStore } from "../store";
import { sendRemoveDevice, sendRemoveSensor, sendRobotBasePose, sendSetObstacleEnabled } from "../ws";

/**
 * Hierarchy over obstacle/frame names (prim paths from USD imports group
 * naturally; flat names sit at the root). Per obstacle: show/hide (display
 * only, client-side) and a collision toggle (server-side `enabled`).
 */
export function SceneTreePanel() {
  const obstacles = useStudioStore((s) => s.obstacles);
  const frames = useStudioStore((s) => s.frames);
  const sensors = useStudioStore((s) => s.sensors);
  const devices = useStudioStore((s) => s.devices);
  if (
    obstacles.length === 0 &&
    frames.length === 0 &&
    sensors.length === 0 &&
    devices.length === 0
  ) {
    return null;
  }
  return (
    <section className="panel-section">
      <div className="panel-head">
        <h2>Scene</h2>
        <span className="badge muted">
          {obstacles.length} obj · {frames.length} frames
        </span>
      </div>
      <Tree obstacles={obstacles} frames={frames} />
      {(sensors.length > 0 || devices.length > 0) && (
        <div className="scene-tree">
          {sensors.map((s) => (
            <div key={s.name} className="tree-row">
              <span className="tree-twist" />
              <span className="tree-label" title={`${s.kind.kind} sensor`}>
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
            <div key={d.name} className="tree-row">
              <span className="tree-twist" />
              <span className="tree-label" title={d.kind.kind}>
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
    </section>
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
  return (
    <div className="scene-tree">
      {[...root.children.values()].map((n) => (
        <TreeRow key={n.path} node={n} depth={0} />
      ))}
    </div>
  );
}

function TreeRow({ node, depth }: { node: TreeNode; depth: number }) {
  const [open, setOpen] = useState(depth < 2);
  const selection = useStudioStore((s) => s.selection);
  const selectObstacle = useStudioStore((s) => s.selectObstacle);
  const hiddenObstacles = useStudioStore((s) => s.hiddenObstacles);
  const toggleObstacleHidden = useStudioStore((s) => s.toggleObstacleHidden);

  const kids = [...node.children.values()];
  const o = node.obstacle;
  const selected = o && selection.type === "obstacle" && selection.name === o.name;
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
          className="tree-label"
          onClick={() => o && selectObstacle(o.name)}
          title={node.path}
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
        {node.frame && (
          <button
            className="tree-toggle"
            title="place robot base here"
            onClick={() => node.frame && sendRobotBasePose(node.frame.pose)}
          >
            ⌖
          </button>
        )}
      </div>
      {open &&
        kids.map((n) => <TreeRow key={n.path} node={n} depth={depth + 1} />)}
    </div>
  );
}

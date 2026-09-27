import { Fragment, useMemo, useState } from "react";

import {
  lookup,
  partLabel,
  partTitle,
  robotStack,
  type BomIndex,
} from "../partIdentity";
import type { BomLineMsg, BomTarget, FrameMsg, ObstacleMsg } from "../protocol";
import { collidingObstacleNames, robotArms, useStudioStore } from "../store";
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
 *
 * Every row that is a line of the bill of materials wears what it is —
 * the badge is read off the `bom` message the host derives, never
 * guessed here — and a robot lists the lines the bill hangs off it (its
 * tools, the controller it needs) as rows of their own.
 */
export function SceneTreePanel() {
  const robots = useStudioStore((s) => s.robots);
  const selectedRobot = useStudioStore((s) => s.selectedRobot);
  const selectTcp = useStudioStore((s) => s.selectTcp);
  const selectRobot = useStudioStore((s) => s.selectRobot);
  const setSelectedGroup = useStudioStore((s) => s.setSelectedGroup);
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
  const partIndex = useStudioStore((s) => s.bomIndex);
  const setBomOpen = useStudioStore((s) => s.setBomOpen);
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
        <>
          <span className="badge muted">
            {robots.length > 1 ? `${robots.length} robots · ` : ""}
            {obstacles.length} obj · {frames.length} frames
          </span>
          <button
            className="timeline-button"
            title="the bill of materials over the viewport (▤ BOM)"
            onClick={() => setBomOpen(true)}
          >
            ▤
          </button>
        </>
      }
    >
      <div className="scene-tree">
        {robots.map((r) => {
          const name = r.desc.name;
          return (
            <Fragment key={name}>
              <div
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
                <PartBadge hit={lookup(partIndex, "robot", name)} />
                <button
                  className="tree-toggle"
                  title="place robot base"
                  onClick={() => selectRobot(name)}
                >
                  ⌖
                </button>
              </div>
              {/* The arms of a dual-arm robot: pick one to drive it. */}
              {robotArms(r.desc).map((g) => (
                <div
                  key={g.name}
                  className={`tree-row${
                    selectedRobot === name && r.selectedGroup === g.name
                      ? " selected"
                      : ""
                  }`}
                  style={{ paddingLeft: "12px" }}
                >
                  <span className="tree-twist" />
                  <span
                    className="tree-label"
                    title={`select this arm (TCP ${g.tip_link})`}
                    onClick={() => {
                      setSelectedGroup(name, g.name);
                      selectTcp(name);
                    }}
                  >
                    {"\u{1F9BE} "}
                    {g.name}
                  </span>
                  {/* An arm mounted from the catalog is a line of its own
                      on the bill, `<robot>/<arm>`. */}
                  <PartBadge
                    hit={
                      lookup(partIndex, "tool", `${name}/${g.name}`) ??
                      lookup(partIndex, "robot", `${name}/${g.name}`)
                    }
                  />
                </div>
              ))}
              <StackRows robot={name} index={partIndex} />
            </Fragment>
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
              <PartBadge hit={lookup(partIndex, "sensor", s.name)} />
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
              <PartBadge hit={lookup(partIndex, "device", d.name)} />
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
              <PartBadge hit={lookup(partIndex, "camera", c.name)} />
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
              <PartBadge hit={lookup(partIndex, "lidar", l.name)} />
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
                <PartBadge hit={lookup(partIndex, "io_node", n.name)} />
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

/** The badge a line of the bill wears on the scene tree: its model (or
 * what else identifies it), amber `?` while nobody has identified it.
 * Display only — identity is authored from Python (`scene.set_part`,
 * `Robot.from_catalog`, the generators). */
function PartBadge({
  hit,
}: {
  hit: { line: BomLineMsg; target: BomTarget } | undefined;
}) {
  if (!hit) return null;
  const { line, target } = hit;
  return (
    <span
      className={`badge ${line.identified ? "muted" : "warn"} part-badge`}
      title={partTitle(line)}
    >
      {partLabel(line, target)}
    </span>
  );
}

/** The lines the bill hangs off a robot with no scene object of their
 * own — its tools (`arm/tool`) and the controller it needs
 * (`arm/controller`, until a cabinet is declared). Selecting one opens
 * its identity in Layout; there is nothing to move. */
function StackRows({ robot, index }: { robot: string; index: BomIndex }) {
  const selection = useStudioStore((s) => s.selection);
  const selectLine = useStudioStore((s) => s.selectLine);
  const { tools, controller } = useMemo(() => robotStack(index, robot), [index, robot]);
  const rows = [
    ...tools.map((t) => ({ ...t, icon: "\u{1F527}", label: t.name.slice(robot.length + 1) })),
    ...(controller ? [{ ...controller, icon: "\u{1F50C}", label: "controller" }] : []),
  ];
  return (
    <>
      {rows.map((r) => (
        <div
          key={r.name}
          className={`tree-row${
            selection.type === "line" && selection.name === r.name ? " selected" : ""
          }`}
          style={{ paddingLeft: "12px" }}
        >
          <span className="tree-twist" />
          <span
            className="tree-label"
            title={`${r.name} — a line of the bill, nothing to move`}
            onClick={() => selectLine(r.target.kind, r.name)}
          >
            {r.icon} {r.label}
          </span>
          <PartBadge hit={r} />
        </div>
      ))}
    </>
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
  partIndex: BomIndex;
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
  partIndex: BomIndex;
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
  // The line pinned to this prim, or to the group it heads (`<path>/…`);
  // a prim inside a pinned group wears nothing — the group row does.
  const part =
    (isGroup ? lookup(partIndex, "group", node.path) : undefined) ??
    (o ? lookup(partIndex, "obstacle", o.name) : undefined);

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
        <PartBadge hit={part} />
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

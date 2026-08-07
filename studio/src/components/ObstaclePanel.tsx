import { useEffect, useMemo, useRef, useState } from "react";

import type { GeometryMsg, ObstacleMsg, PoseMsg } from "../protocol";
import { collidingObstacleNames, robotByName, useStudioStore } from "../store";
import {
  sendAddObstacle,
  sendAttachObstacle,
  sendDetachObstacle,
  sendRemoveObstacle,
  sendUpdateObstacleGeometry,
} from "../ws";

// New obstacles spawn just in front of the robot base, upright.
const SPAWN_POSITION: [number, number, number] = [0.3, 0, 0.1];
const IDENTITY_QUAT: [number, number, number, number] = [0, 0, 0, 1];

function spawnPose(): PoseMsg {
  return { position: [...SPAWN_POSITION], quaternion: [...IDENTITY_QUAT] };
}

const DEFAULTS: Record<"box" | "sphere" | "cylinder", () => ObstacleMsg> = {
  box: () => ({
    name: "box",
    geometry: { kind: "box", size: [0.1, 0.1, 0.1] },
    pose: spawnPose(),
    enabled: true,
    visible: true,
    attached_to: null,
  }),
  sphere: () => ({
    name: "sphere",
    geometry: { kind: "sphere", radius: 0.05 },
    pose: spawnPose(),
    enabled: true,
    visible: true,
    attached_to: null,
  }),
  cylinder: () => ({
    name: "cylinder",
    geometry: { kind: "cylinder", radius: 0.05, length: 0.1 },
    pose: spawnPose(),
    enabled: true,
    visible: true,
    attached_to: null,
  }),
};

export function ObstaclePanel() {
  const obstacles = useStudioStore((s) => s.obstacles);
  const collisions = useStudioStore((s) => s.collisions);
  const minDistance = useStudioStore((s) => s.minDistance);
  const selection = useStudioStore((s) => s.selection);
  const selectObstacle = useStudioStore((s) => s.selectObstacle);

  const collidingObstacles = useMemo(
    () => collidingObstacleNames(collisions),
    [collisions],
  );

  const selectedName = selection.type === "obstacle" ? selection.name : null;
  const selected = obstacles.find((o) => o.name === selectedName) ?? null;

  // The server assigns the final (possibly de-duplicated) name, so we can't
  // predict it. After an add, select whichever name shows up new in the next
  // obstacles broadcast.
  const knownNamesRef = useRef<Set<string>>(new Set());
  const pendingAddRef = useRef(false);

  useEffect(() => {
    if (pendingAddRef.current) {
      const added = obstacles.find((o) => !knownNamesRef.current.has(o.name));
      if (added) {
        selectObstacle(added.name);
        pendingAddRef.current = false;
      }
    }
    knownNamesRef.current = new Set(obstacles.map((o) => o.name));
  }, [obstacles, selectObstacle]);

  const add = (kind: "box" | "sphere" | "cylinder") => {
    pendingAddRef.current = true;
    sendAddObstacle(DEFAULTS[kind]());
  };

  return (
    <section className="panel-section">
      <div className="panel-head">
        <h2>Obstacles</h2>
        <ClearanceBadge
          minDistance={minDistance}
          colliding={collisions.length > 0}
        />
      </div>
      <div className="obstacle-controls">
        <div className="seg">
          <button onClick={() => add("box")}>+ Box</button>
          <button onClick={() => add("sphere")}>+ Sphere</button>
          <button onClick={() => add("cylinder")}>+ Cylinder</button>
        </div>

        <div className="obstacle-list">
          {obstacles.map((o) => (
            <div
              key={o.name}
              className={`obstacle-row${o.name === selectedName ? " selected" : ""}`}
              onClick={() => selectObstacle(o.name)}
            >
              <span
                className={`obstacle-name${collidingObstacles.has(o.name) ? " bad" : ""}`}
              >
                {o.attached_to && (
                  <span title={`attached to ${o.attached_to.link}`}>{"\u{1F9F2} "}</span>
                )}
                {o.name}
              </span>
              <button
                className="obstacle-remove"
                title="Remove"
                onClick={(e) => {
                  e.stopPropagation();
                  sendRemoveObstacle(o.name);
                }}
              >
                ×
              </button>
            </div>
          ))}
          {obstacles.length === 0 && (
            <div className="empty">No obstacles</div>
          )}
        </div>

        {selected && <ObstacleForm obstacle={selected} />}
      </div>
    </section>
  );
}

function ClearanceBadge({
  minDistance,
  colliding,
}: {
  minDistance: number | null;
  colliding: boolean;
}) {
  if (minDistance === null) return null;
  if (colliding) return <span className="badge bad">collision</span>;
  return (
    <span className="badge muted">
      clearance {(minDistance * 1000).toFixed(0)}mm
    </span>
  );
}

/** Display label for a link: prim paths shorten to their last segment. */
function linkLabel(link: string): string {
  return link.split("/").filter(Boolean).pop() ?? link;
}

function ObstacleForm({ obstacle }: { obstacle: ObstacleMsg }) {
  const { geometry, pose, name } = obstacle;
  // Attach grasps with the panel-selected robot at its TCP link.
  const selectedRobot = useStudioStore((s) => s.selectedRobot);
  const tcpLink = useStudioStore(
    (s) => robotByName(s.robots, s.selectedRobot)?.tcpLink ?? null,
  );
  const commit = (g: GeometryMsg) => sendUpdateObstacleGeometry(name, g);

  return (
    <div className="obstacle-form">
      {geometry.kind === "box" && (
        <>
          {(["x", "y", "z"] as const).map((axis, i) => (
            <NumberField
              key={axis}
              label={`s${axis}`}
              value={geometry.size[i]}
              onCommit={(v) => {
                const size = [...geometry.size] as [number, number, number];
                size[i] = v;
                commit({ kind: "box", size });
              }}
            />
          ))}
        </>
      )}
      {geometry.kind === "sphere" && (
        <NumberField
          label="r"
          value={geometry.radius}
          onCommit={(v) => commit({ kind: "sphere", radius: v })}
        />
      )}
      {geometry.kind === "cylinder" && (
        <>
          <NumberField
            label="r"
            value={geometry.radius}
            onCommit={(v) =>
              commit({ kind: "cylinder", radius: v, length: geometry.length })
            }
          />
          <NumberField
            label="l"
            value={geometry.length}
            onCommit={(v) =>
              commit({ kind: "cylinder", radius: geometry.radius, length: v })
            }
          />
        </>
      )}
      <div className="field num-field">
        <span className="field-label">pos</span>
        <span className="obstacle-pos">
          {pose.position.map((v) => v.toFixed(3)).join("  ")}
        </span>
      </div>
      <div className="seg">
        {obstacle.attached_to ? (
          <button
            title={`attached to ${obstacle.attached_to.link}`}
            onClick={() => sendDetachObstacle(name)}
          >
            Detach from {linkLabel(obstacle.attached_to.link)}
          </button>
        ) : (
          <button
            title="grasp: follow the TCP link and collide as part of the robot"
            disabled={selectedRobot === null}
            onClick={() =>
              selectedRobot !== null &&
              sendAttachObstacle(name, selectedRobot, tcpLink)
            }
          >
            Attach to {tcpLink ? linkLabel(tcpLink) : "TCP"}
          </button>
        )}
      </div>
    </div>
  );
}

const MIN_DIM = 0.001;

/**
 * Number input that keeps its own text while typing and adopts external
 * changes (e.g. the server echo) only when they differ from the current value.
 */
function NumberField({
  label,
  value,
  onCommit,
}: {
  label: string;
  value: number;
  onCommit: (value: number) => void;
}) {
  const [text, setText] = useState(String(value));

  useEffect(() => {
    setText((t) => (parseFloat(t) === value ? t : String(value)));
  }, [value]);

  return (
    <label className="field num-field">
      <span className="field-label">{label}</span>
      <input
        type="number"
        min={MIN_DIM}
        step={0.01}
        value={text}
        onChange={(e) => {
          setText(e.target.value);
          const v = parseFloat(e.target.value);
          if (Number.isFinite(v) && v >= MIN_DIM) onCommit(v);
        }}
      />
    </label>
  );
}

import { useEffect, useRef, useState } from "react";

import type {
  DeviceMsg,
  PoseMsg,
  SensorMsg,
  SensorWatchMsg,
} from "../protocol";
import { useStudioStore } from "../store";
import {
  sendRemoveDevice,
  sendRemoveSensor,
  sendUpsertDevice,
  sendUpsertSensor,
} from "../ws";

/**
 * Create/edit forms for the world's I/O fixtures: zone/beam sensors and
 * the two workhorse devices (conveyor, linear axis). The deeper device
 * kinds (source, sink, vehicle) carry pools, paths, and stations — those
 * stay Python-authored and appear here read-only.
 *
 * Upserts replace wholesale (the full-list re-send culture), so every
 * field commit sends the whole record; creation picks a fresh name so an
 * upsert never silently overwrites an existing fixture.
 */

const IDENTITY_QUAT: [number, number, number, number] = [0, 0, 0, 1];

function at(position: [number, number, number]): PoseMsg {
  return { position, quaternion: [...IDENTITY_QUAT] };
}

/** `zone`, `zone_2`, `zone_3`, … — whatever is free. */
function freshName(base: string, taken: { name: string }[]): string {
  const names = new Set(taken.map((t) => t.name));
  if (!names.has(base)) return base;
  for (let i = 2; ; i++) {
    const candidate = `${base}_${i}`;
    if (!names.has(candidate)) return candidate;
  }
}

function watchLabel(watch: SensorWatchMsg): string {
  switch (watch.kind) {
    case "objects":
      return watch.names.map((n) => n.split("/").pop() ?? n).join(", ");
    case "all_objects":
      return "all objects";
    case "robot":
    case "robots":
      return "robots";
    case "all":
      return "everything";
  }
}

export function SensorDevicePanel() {
  const sensors = useStudioStore((s) => s.sensors);
  const devices = useStudioStore((s) => s.devices);
  const selection = useStudioStore((s) => s.selection);
  const selectedObstacle = selection.type === "obstacle" ? selection.name : null;

  const [editing, setEditing] = useState<
    { kind: "sensor" | "device"; name: string } | null
  >(null);
  // A freshly created fixture is editable before the server echo lands;
  // `pending` keeps the close-on-deleted effect below from mistaking that
  // in-flight moment for a deletion.
  const pendingRef = useRef<string | null>(null);
  useEffect(() => {
    if (!editing) return;
    const pool: { name: string }[] =
      editing.kind === "sensor" ? sensors : devices;
    if (pool.some((f) => f.name === editing.name)) {
      pendingRef.current = null;
    } else if (pendingRef.current !== editing.name) {
      setEditing(null);
    }
  }, [editing, sensors, devices]);

  // New sensors watch the selected obstacle when there is one — placing a
  // beam for a specific part is the common gesture — else all objects.
  const defaultWatch = (): SensorWatchMsg =>
    selectedObstacle
      ? { kind: "objects", names: [selectedObstacle] }
      : { kind: "all_objects" };

  const addZone = () => {
    const name = freshName("zone", sensors);
    sendUpsertSensor({
      name,
      kind: {
        kind: "zone",
        pose: at([0.4, 0, 0.2]),
        size: [0.3, 0.3, 0.3],
      },
      watch: defaultWatch(),
      mount: null,
    });
    pendingRef.current = name;
    setEditing({ kind: "sensor", name });
  };
  const addBeam = () => {
    const name = freshName("beam", sensors);
    sendUpsertSensor({
      name,
      kind: {
        kind: "beam",
        from: [0.4, -0.2, 0.2],
        to: [0.4, 0.2, 0.2],
        radius: 0.005,
      },
      watch: defaultWatch(),
      mount: null,
    });
    pendingRef.current = name;
    setEditing({ kind: "sensor", name });
  };
  const addConveyor = () => {
    const name = freshName("conveyor", devices);
    sendUpsertDevice({
      name,
      kind: {
        kind: "conveyor",
        zone_pose: at([0.5, 0, 0.1]),
        zone_size: [1.0, 0.4, 0.15],
        velocity: [0.15, 0, 0],
        running: true,
      },
    });
    pendingRef.current = name;
    setEditing({ kind: "device", name });
  };
  const addAxis = () => {
    if (!selectedObstacle) return;
    const name = freshName("axis", devices);
    sendUpsertDevice({
      name,
      kind: {
        kind: "linear_axis",
        objects: [selectedObstacle],
        axis: [0, 0, 1],
        speed: 0.2,
        position: 0,
        range: [0, 0.5],
      },
    });
    pendingRef.current = name;
    setEditing({ kind: "device", name });
  };

  const editedSensor =
    editing?.kind === "sensor"
      ? (sensors.find((s) => s.name === editing.name) ?? null)
      : null;
  const editedDevice =
    editing?.kind === "device"
      ? (devices.find((d) => d.name === editing.name) ?? null)
      : null;

  return (
    <section className="panel-section">
      <div className="panel-head">
        <h2>Sensors & devices</h2>
      </div>
      <div className="obstacle-controls">
        <div className="seg">
          <button onClick={addZone} title="area/presence sensor (box test)">
            + Zone
          </button>
          <button onClick={addBeam} title="photoelectric beam (capsule test)">
            + Beam
          </button>
          <button onClick={addConveyor} title="zone advection belt">
            + Conveyor
          </button>
          <button
            onClick={addAxis}
            disabled={!selectedObstacle}
            title="single axis moving the selected obstacle (door, lifter)"
          >
            + Axis
          </button>
        </div>

        <div className="obstacle-list">
          {sensors.map((s) => (
            <div
              key={s.name}
              className={`obstacle-row${
                editedSensor?.name === s.name ? " selected" : ""
              }`}
              onClick={() => setEditing({ kind: "sensor", name: s.name })}
            >
              <span className="obstacle-name">
                📡 {s.name}
                <span className="seq-cond">
                  {" "}
                  · {s.kind.kind} → {watchLabel(s.watch)}
                </span>
              </span>
              <button
                className="obstacle-remove"
                title="Remove sensor"
                onClick={(e) => {
                  e.stopPropagation();
                  sendRemoveSensor(s.name);
                }}
              >
                ×
              </button>
            </div>
          ))}
          {devices.map((d) => (
            <div
              key={d.name}
              className={`obstacle-row${
                editedDevice?.name === d.name ? " selected" : ""
              }`}
              onClick={() => setEditing({ kind: "device", name: d.name })}
            >
              <span className="obstacle-name">
                ⚙ {d.name}
                <span className="seq-cond"> · {d.kind.kind}</span>
              </span>
              <button
                className="obstacle-remove"
                title="Remove device"
                onClick={(e) => {
                  e.stopPropagation();
                  sendRemoveDevice(d.name);
                }}
              >
                ×
              </button>
            </div>
          ))}
          {sensors.length === 0 && devices.length === 0 && (
            <div className="empty">no sensors or devices</div>
          )}
        </div>

        {editedSensor && (
          <SensorForm sensor={editedSensor} selectedObstacle={selectedObstacle} />
        )}
        {editedDevice && <DeviceForm device={editedDevice} />}
      </div>
    </section>
  );
}

function SensorForm({
  sensor,
  selectedObstacle,
}: {
  sensor: SensorMsg;
  selectedObstacle: string | null;
}) {
  const commit = (patch: Partial<SensorMsg>) =>
    sendUpsertSensor({ ...sensor, ...patch });
  const { kind } = sensor;

  return (
    <div className="obstacle-form">
      {kind.kind === "zone" && (
        <>
          <VecFields
            label="pos"
            value={kind.pose.position}
            onCommit={(position) =>
              commit({ kind: { ...kind, pose: { ...kind.pose, position } } })
            }
          />
          <VecFields
            label="size"
            value={kind.size}
            min={0.001}
            onCommit={(size) => commit({ kind: { ...kind, size } })}
          />
        </>
      )}
      {kind.kind === "beam" && (
        <>
          <VecFields
            label="from"
            value={kind.from}
            onCommit={(from) => commit({ kind: { ...kind, from } })}
          />
          <VecFields
            label="to"
            value={kind.to}
            onCommit={(to) => commit({ kind: { ...kind, to } })}
          />
          <NumField
            label="r"
            value={kind.radius}
            min={0.001}
            onCommit={(radius) => commit({ kind: { ...kind, radius } })}
          />
        </>
      )}
      <div className="seg">
        <span className="seq-cond">watch:</span>
        <button
          disabled={sensor.watch.kind === "all_objects"}
          onClick={() => commit({ watch: { kind: "all_objects" } })}
        >
          all objects
        </button>
        <button
          disabled={!selectedObstacle}
          title="watch only the obstacle selected in the scene"
          onClick={() =>
            selectedObstacle &&
            commit({ watch: { kind: "objects", names: [selectedObstacle] } })
          }
        >
          selection
        </button>
        <button
          disabled={sensor.watch.kind === "all"}
          title="objects and robots (light curtain)"
          onClick={() => commit({ watch: { kind: "all" } })}
        >
          everything
        </button>
      </div>
    </div>
  );
}

function DeviceForm({ device }: { device: DeviceMsg }) {
  const commit = (kind: DeviceMsg["kind"]) =>
    sendUpsertDevice({ ...device, kind });
  const { kind } = device;

  if (kind.kind === "conveyor") {
    return (
      <div className="obstacle-form">
        <VecFields
          label="pos"
          value={kind.zone_pose.position}
          onCommit={(position) =>
            commit({ ...kind, zone_pose: { ...kind.zone_pose, position } })
          }
        />
        <VecFields
          label="size"
          value={kind.zone_size}
          min={0.001}
          onCommit={(zone_size) => commit({ ...kind, zone_size })}
        />
        <VecFields
          label="vel"
          value={kind.velocity}
          onCommit={(velocity) => commit({ ...kind, velocity })}
        />
        <label className="seq-program">
          <input
            type="checkbox"
            checked={kind.running}
            onChange={(e) => commit({ ...kind, running: e.target.checked })}
          />
          running at start
        </label>
      </div>
    );
  }
  if (kind.kind === "linear_axis") {
    return (
      <div className="obstacle-form">
        <div className="field num-field">
          <span className="field-label">moves</span>
          <span className="obstacle-pos">
            {kind.objects.map((o) => o.split("/").pop() ?? o).join(", ")}
          </span>
        </div>
        <VecFields
          label="axis"
          value={kind.axis}
          onCommit={(axis) => commit({ ...kind, axis })}
        />
        <NumField
          label="v"
          value={kind.speed}
          min={0.001}
          onCommit={(speed) => commit({ ...kind, speed })}
        />
        <NumField
          label="lo"
          value={kind.range[0]}
          onCommit={(lo) => commit({ ...kind, range: [lo, kind.range[1]] })}
        />
        <NumField
          label="hi"
          value={kind.range[1]}
          onCommit={(hi) => commit({ ...kind, range: [kind.range[0], hi] })}
        />
      </div>
    );
  }
  // Source / sink / vehicle: authored in Python, shown read-only here.
  return (
    <div className="obstacle-form">
      <span className="seq-cond">
        {kind.kind} devices are authored from Python (pools, paths, and
        stations do not fit a form)
      </span>
    </div>
  );
}

function VecFields({
  label,
  value,
  min,
  onCommit,
}: {
  label: string;
  value: [number, number, number];
  min?: number;
  onCommit: (value: [number, number, number]) => void;
}) {
  return (
    <>
      {(["x", "y", "z"] as const).map((axis, i) => (
        <NumField
          key={axis}
          label={`${label}.${axis}`}
          value={value[i]}
          min={min}
          onCommit={(v) => {
            const next = [...value] as [number, number, number];
            next[i] = v;
            onCommit(next);
          }}
        />
      ))}
    </>
  );
}

/** Number input that keeps its own text while typing and adopts external
 * changes (the server echo) only when they differ. Unlike the obstacle
 * panel's field, positions and velocities may be negative. */
function NumField({
  label,
  value,
  min,
  onCommit,
}: {
  label: string;
  value: number;
  min?: number;
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
        step={0.01}
        min={min}
        value={text}
        onChange={(e) => {
          setText(e.target.value);
          const v = parseFloat(e.target.value);
          if (Number.isFinite(v) && (min === undefined || v >= min)) {
            onCommit(v);
          }
        }}
      />
    </label>
  );
}

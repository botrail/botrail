import { useEffect, useState } from "react";

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
import { Section } from "./Section";

/**
 * Create/edit forms for the world's I/O fixtures: zone/beam sensors and
 * the two workhorse devices (conveyor, linear axis). The deeper device
 * kinds (source, sink, vehicle) carry pools, paths, and stations — those
 * stay Python-authored and appear here read-only. The scene tree is the
 * list; this panel adds and edits the selected fixture.
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
  const selectSensor = useStudioStore((s) => s.selectSensor);
  const selectDevice = useStudioStore((s) => s.selectDevice);
  const selectedObstacle = selection.type === "obstacle" ? selection.name : null;

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
    selectSensor(name);
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
    selectSensor(name);
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
    selectDevice(name);
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
    selectDevice(name);
  };

  const editedSensor =
    selection.type === "sensor"
      ? (sensors.find((s) => s.name === selection.name) ?? null)
      : null;
  const editedDevice =
    selection.type === "device"
      ? (devices.find((d) => d.name === selection.name) ?? null)
      : null;

  return (
    <Section id="sensors" title="Sensors & devices">
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

        {editedSensor ? (
          <SensorForm sensor={editedSensor} selectedObstacle={selectedObstacle} />
        ) : editedDevice ? (
          <DeviceForm device={editedDevice} />
        ) : (
          <div className="hint">
            {sensors.length === 0 && devices.length === 0
              ? "no sensors or devices"
              : "select a sensor or device in the scene tree to edit it"}
          </div>
        )}
      </div>
    </Section>
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
      <div className="inspector-title" title={sensor.name}>
        📡 {sensor.name}
        <span className="seq-cond">
          {" "}
          · {kind.kind} → {watchLabel(sensor.watch)}
        </span>
      </div>
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
      <div className="seg">
        <button
          className="danger"
          title="remove this sensor"
          onClick={() => sendRemoveSensor(sensor.name)}
        >
          Remove
        </button>
      </div>
    </div>
  );
}

function DeviceForm({ device }: { device: DeviceMsg }) {
  const commit = (kind: DeviceMsg["kind"]) =>
    sendUpsertDevice({ ...device, kind });
  const { kind } = device;

  const title = (
    <div className="inspector-title" title={device.name}>
      ⚙ {device.name}
      <span className="seq-cond"> · {kind.kind}</span>
    </div>
  );
  const remove = (
    <div className="seg">
      <button
        className="danger"
        title="remove this device"
        onClick={() => sendRemoveDevice(device.name)}
      >
        Remove
      </button>
    </div>
  );

  if (kind.kind === "conveyor") {
    return (
      <div className="obstacle-form">
        {title}
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
        {remove}
      </div>
    );
  }
  if (kind.kind === "linear_axis") {
    return (
      <div className="obstacle-form">
        {title}
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
        {remove}
      </div>
    );
  }
  // Source / sink / vehicle: authored in Python, shown read-only here.
  return (
    <div className="obstacle-form">
      {title}
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

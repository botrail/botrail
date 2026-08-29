import { useEffect, useState } from "react";

import type {
  CameraMsg,
  DeviceMsg,
  LidarMsg,
  PoseMsg,
  SensorMsg,
  SensorWatchMsg,
} from "../protocol";
import { useStudioStore } from "../store";
import {
  sendRemoveCamera,
  sendRemoveDevice,
  sendRemoveLidar,
  sendScanLidar,
  sendRemoveSensor,
  sendUpsertCamera,
  sendUpsertDevice,
  sendUpsertLidar,
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
  const cameras = useStudioStore((s) => s.cameras);
  const lidars = useStudioStore((s) => s.lidars);
  const selection = useStudioStore((s) => s.selection);
  const selectSensor = useStudioStore((s) => s.selectSensor);
  const selectDevice = useStudioStore((s) => s.selectDevice);
  const selectCamera = useStudioStore((s) => s.selectCamera);
  const selectLidar = useStudioStore((s) => s.selectLidar);
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

  const addVision = () => {
    // The optics come from a camera; default to the one being watched.
    const camera =
      useStudioStore.getState().pipCamera ?? cameras[0]?.name ?? null;
    if (!camera) return;
    const name = freshName("vision", sensors);
    sendUpsertSensor({
      name,
      kind: { kind: "vision", camera, detect_range: null, occlusion: true },
      watch: defaultWatch(),
      mount: null,
    });
    selectSensor(name);
  };

  const addCamera = () => {
    const name = freshName("camera", cameras);
    // Identity quaternion looks straight down (-Z view in a Z-up world):
    // a bird's-eye camera out of the box; aim it with the rotate gizmo.
    sendUpsertCamera({
      name,
      mount: { kind: "world" },
      pose: at([0.8, 0, 1.6]),
      fov_deg: 60,
      resolution: [1280, 720],
      near: 0.05,
      far: 30,
    });
    selectCamera(name);
  };

  const addLidar = () => {
    const name = freshName("lidar", lidars);
    // Identity pose scans the world XY plane, angle 0 along +X; aim it
    // with the rotate gizmo. LMS-class defaults: 270°, 0.05–20 m.
    sendUpsertLidar({
      name,
      mount: { kind: "world" },
      pose: at([0.8, 0, 0.2]),
      fov_deg: 270,
      range: [0.05, 20],
      resolution_deg: 0.5,
      channels: 1,
      vfov_deg: 0,
    });
    selectLidar(name);
  };

  const addField = () => {
    // The sweep comes from a lidar; default to the one being edited.
    const lidar =
      (selection.type === "lidar" ? selection.name : null) ??
      lidars[0]?.name ??
      null;
    if (!lidar) return;
    const name = freshName("field", sensors);
    sendUpsertSensor({
      name,
      kind: { kind: "field", lidar, range: null, sector: null, shadowing: true },
      watch: defaultWatch(),
      mount: null,
    });
    selectSensor(name);
  };

  const editedSensor =
    selection.type === "sensor"
      ? (sensors.find((s) => s.name === selection.name) ?? null)
      : null;
  const editedDevice =
    selection.type === "device"
      ? (devices.find((d) => d.name === selection.name) ?? null)
      : null;
  const editedCamera =
    selection.type === "camera"
      ? (cameras.find((c) => c.name === selection.name) ?? null)
      : null;
  const editedLidar =
    selection.type === "lidar"
      ? (lidars.find((l) => l.name === selection.name) ?? null)
      : null;

  return (
    <Section id="sensors" title="Sensors & devices">
      <div className="obstacle-controls">
        <div className="seg wrap">
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
          <button
            onClick={addCamera}
            title="camera viewpoint (frustum gizmo; picture is presentation only)"
          >
            + Camera
          </button>
          <button
            onClick={addVision}
            disabled={cameras.length === 0}
            title="vision presence sensor looking through a camera (frustum test)"
          >
            + Vision
          </button>
          <button
            onClick={addLidar}
            title="LiDAR scanner (scan sector gizmo; fields are the planned signal path)"
          >
            + Lidar
          </button>
          <button
            onClick={addField}
            disabled={lidars.length === 0}
            title="laser-scanner field sweeping through a lidar (sector test)"
          >
            + Field
          </button>
        </div>

        {editedSensor ? (
          <SensorForm sensor={editedSensor} selectedObstacle={selectedObstacle} />
        ) : editedDevice ? (
          <DeviceForm device={editedDevice} />
        ) : editedCamera ? (
          <CameraForm camera={editedCamera} />
        ) : editedLidar ? (
          <LidarForm lidar={editedLidar} />
        ) : (
          <div className="hint">
            {sensors.length === 0 &&
            devices.length === 0 &&
            cameras.length === 0 &&
            lidars.length === 0
              ? "no sensors, devices or cameras"
              : "select a sensor, device or camera in the scene tree to edit it"}
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
  const cameras = useStudioStore((s) => s.cameras);
  const lidars = useStudioStore((s) => s.lidars);
  const commit = (patch: Partial<SensorMsg>) =>
    sendUpsertSensor({ ...sensor, ...patch });
  const { kind } = sensor;
  const visionCamera =
    kind.kind === "vision"
      ? (cameras.find((c) => c.name === kind.camera) ?? null)
      : null;
  const fieldLidar =
    kind.kind === "field"
      ? (lidars.find((l) => l.name === kind.lidar) ?? null)
      : null;

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
      {kind.kind === "vision" && (
        <>
          <label className="field num-field">
            <span className="field-label">camera</span>
            <select
              value={kind.camera}
              onChange={(e) =>
                commit({ kind: { ...kind, camera: e.target.value } })
              }
            >
              {cameras.map((c) => (
                <option key={c.name} value={c.name}>
                  {c.name}
                </option>
              ))}
            </select>
          </label>
          <NumField
            label="near"
            value={kind.detect_range?.[0] ?? visionCamera?.near ?? 0.05}
            min={0.001}
            onCommit={(a) =>
              commit({
                kind: {
                  ...kind,
                  detect_range: [
                    a,
                    kind.detect_range?.[1] ?? visionCamera?.far ?? 30,
                  ],
                },
              })
            }
          />
          <NumField
            label="far"
            value={kind.detect_range?.[1] ?? visionCamera?.far ?? 30}
            min={0.01}
            onCommit={(b) =>
              commit({
                kind: {
                  ...kind,
                  detect_range: [
                    kind.detect_range?.[0] ?? visionCamera?.near ?? 0.05,
                    b,
                  ],
                },
              })
            }
          />
          {kind.detect_range && (
            <button
              title="follow the camera's near/far clip again"
              onClick={() => commit({ kind: { ...kind, detect_range: null } })}
            >
              camera range
            </button>
          )}
          <label className="seq-program">
            <input
              type="checkbox"
              checked={kind.occlusion}
              onChange={(e) =>
                commit({ kind: { ...kind, occlusion: e.target.checked } })
              }
            />
            occlusion (a body hidden behind another does not trip)
          </label>
        </>
      )}
      {kind.kind === "field" && (
        <>
          <label className="field num-field">
            <span className="field-label">lidar</span>
            <select
              value={kind.lidar}
              onChange={(e) =>
                commit({ kind: { ...kind, lidar: e.target.value } })
              }
            >
              {lidars.map((l) => (
                <option key={l.name} value={l.name}>
                  {l.name}
                </option>
              ))}
            </select>
          </label>
          <NumField
            label="range"
            value={kind.range ?? fieldLidar?.range[1] ?? 20}
            min={0.01}
            onCommit={(range) => commit({ kind: { ...kind, range } })}
          />
          <NumField
            label="start°"
            value={kind.sector?.[0] ?? -(fieldLidar?.fov_deg ?? 270) / 2}
            onCommit={(a) =>
              commit({
                kind: {
                  ...kind,
                  sector: [a, kind.sector?.[1] ?? (fieldLidar?.fov_deg ?? 270) / 2],
                },
              })
            }
          />
          <NumField
            label="end°"
            value={kind.sector?.[1] ?? (fieldLidar?.fov_deg ?? 270) / 2}
            onCommit={(b) =>
              commit({
                kind: {
                  ...kind,
                  sector: [kind.sector?.[0] ?? -(fieldLidar?.fov_deg ?? 270) / 2, b],
                },
              })
            }
          />
          {(kind.range != null || kind.sector) && (
            <button
              title="follow the lidar's full sweep and max range again"
              onClick={() =>
                commit({ kind: { ...kind, range: null, sector: null } })
              }
            >
              full sweep
            </button>
          )}
          <label className="seq-program">
            <input
              type="checkbox"
              checked={kind.shadowing}
              onChange={(e) =>
                commit({ kind: { ...kind, shadowing: e.target.checked } })
              }
            />
            shadowing (a body hidden behind another does not trip)
          </label>
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

function CameraForm({ camera }: { camera: CameraMsg }) {
  const commit = (patch: Partial<CameraMsg>) =>
    sendUpsertCamera({ ...camera, ...patch });
  const mountLabel =
    camera.mount.kind === "world"
      ? "world fixture"
      : camera.mount.kind === "vehicle"
        ? `on ${camera.mount.device}`
        : `on ${camera.mount.robot}/${camera.mount.link}`;

  return (
    <div className="obstacle-form">
      <div className="inspector-title" title={camera.name}>
        🎥 {camera.name}
        <span className="seq-cond"> · {mountLabel}</span>
      </div>
      <VecFields
        label="pos"
        value={camera.pose.position}
        onCommit={(position) =>
          commit({ pose: { ...camera.pose, position } })
        }
      />
      <NumField
        label="fov°"
        value={camera.fov_deg}
        min={1}
        onCommit={(fov_deg) => commit({ fov_deg })}
      />
      <NumField
        label="res.w"
        value={camera.resolution[0]}
        min={16}
        onCommit={(w) =>
          commit({ resolution: [Math.round(w), camera.resolution[1]] })
        }
      />
      <NumField
        label="res.h"
        value={camera.resolution[1]}
        min={16}
        onCommit={(h) =>
          commit({ resolution: [camera.resolution[0], Math.round(h)] })
        }
      />
      <NumField
        label="near"
        value={camera.near}
        min={0.001}
        onCommit={(near) => commit({ near })}
      />
      <NumField
        label="far"
        value={camera.far}
        min={0.01}
        onCommit={(far) => commit({ far })}
      />
      {camera.mount.kind === "world" && (
        <span className="seq-cond">
          aim it with the viewport gizmo (rotate mode)
        </span>
      )}
      <div className="seg">
        <button
          className="danger"
          title="remove this camera"
          onClick={() => sendRemoveCamera(camera.name)}
        >
          Remove
        </button>
      </div>
    </div>
  );
}

function LidarForm({ lidar }: { lidar: LidarMsg }) {
  const commit = (patch: Partial<LidarMsg>) =>
    sendUpsertLidar({ ...lidar, ...patch });
  const hasCloud = useStudioStore((s) => lidar.name in s.scanClouds);
  const clearScanCloud = useStudioStore((s) => s.clearScanCloud);
  const mountLabel =
    lidar.mount.kind === "world"
      ? "world fixture"
      : lidar.mount.kind === "vehicle"
        ? `on ${lidar.mount.device}`
        : `on ${lidar.mount.robot}/${lidar.mount.link}`;

  return (
    <div className="obstacle-form">
      <div className="inspector-title" title={lidar.name}>
        {"\u{1F300}"} {lidar.name}
        <span className="seq-cond"> · {mountLabel}</span>
      </div>
      <VecFields
        label="pos"
        value={lidar.pose.position}
        onCommit={(position) => commit({ pose: { ...lidar.pose, position } })}
      />
      <NumField
        label="fov°"
        value={lidar.fov_deg}
        min={1}
        onCommit={(fov_deg) => commit({ fov_deg: Math.min(fov_deg, 360) })}
      />
      <NumField
        label="min"
        value={lidar.range[0]}
        min={0.001}
        onCommit={(min) => commit({ range: [min, lidar.range[1]] })}
      />
      <NumField
        label="max"
        value={lidar.range[1]}
        min={0.01}
        onCommit={(max) => commit({ range: [lidar.range[0], max] })}
      />
      <NumField
        label="res°"
        value={lidar.resolution_deg}
        min={0.01}
        onCommit={(resolution_deg) => commit({ resolution_deg })}
      />
      <NumField
        label="rings"
        value={lidar.channels}
        min={1}
        onCommit={(n) => {
          // Rings and vertical field travel together: the server
          // rejects a multi-ring scanner without a vertical field (and
          // the reverse), so the pair changes as one.
          const channels = Math.max(1, Math.round(n));
          commit({
            channels,
            vfov_deg:
              channels > 1 ? (lidar.vfov_deg > 0 ? lidar.vfov_deg : 30) : 0,
          });
        }}
      />
      {lidar.channels > 1 && (
        <NumField
          label="vfov°"
          value={lidar.vfov_deg}
          min={0.1}
          onCommit={(vfov_deg) => commit({ vfov_deg: Math.min(vfov_deg, 179) })}
        />
      )}
      {lidar.mount.kind === "world" && (
        <span className="seq-cond">
          aim the scan heading (+X) with the viewport gizmo (rotate mode)
        </span>
      )}
      <div className="seg">
        <button
          title="simulate one sweep and overlay its returns in the viewport — at the playhead when a timeline is loaded, else the scene as authored; a snapshot (press again after edits or seeks to refresh)"
          onClick={() => {
            // The playhead whenever a timeline is loaded: the viewport
            // shows the baked sample there, so the sweep must too.
            const s = useStudioStore.getState();
            sendScanLidar(lidar.name, s.timeline ? s.playbackTime : null);
          }}
        >
          Scan
        </button>
        {hasCloud && (
          <button
            title="hide the sweep overlay"
            onClick={() => clearScanCloud(lidar.name)}
          >
            Hide scan
          </button>
        )}
      </div>
      <div className="seg">
        <button
          className="danger"
          title="remove this lidar"
          onClick={() => sendRemoveLidar(lidar.name)}
        >
          Remove
        </button>
      </div>
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

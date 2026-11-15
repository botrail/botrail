import type { AnalogMsg, DeviceMsg, IoNode, IoPointMsg, PlantStatusMsg, ReadingsMsg } from "./protocol";
import { deviationAt, readLevel } from "./readings.ts";
import { addressOf, pointKey } from "./ioLinks.ts";
import { analogReading, forcedWord } from "./words.ts";

/**
 * The 3D tags of the I/O points: where each point lives in the cell and
 * what its tag reads — the address on the wire, the point's name, its
 * level or value. Pure: the view (components/IoTagView.tsx) resolves the
 * anchors to world positions and draws the boards.
 */

/** The cell resident a point belongs to: a sensor, a device (a limit
 * switch or a coil names its axis), a lamp's obstacle, a robot. */
export type TagAnchor =
  | { kind: "sensor"; name: string }
  | { kind: "device"; name: string }
  | { kind: "obstacle"; name: string }
  | { kind: "robot"; name: string };

export function anchorKey(a: TagAnchor): string {
  return `${a.kind}:${a.name}`;
}

export type TagScene = {
  sensors: string[];
  devices: DeviceMsg[];
  /** Indicator lamps: the obstacle lit by each signal. */
  lamps: { signal: string; obstacle?: string | null }[];
  robots: string[];
  /** Analog inputs: the obstacle that is each transmitter's body. */
  analogs?: { name: string; place?: string | null }[];
};

/** Where `p` lives, or `null` for a point with no body (an internal
 * relay, a step word, a magazine). */
export function anchorOf(p: IoPointMsg, scene: TagScene): TagAnchor | null {
  if (p.status === "cosmetic" || p.source === "monitor") return null;
  const device = (name: string) => scene.devices.find((d) => d.name === name) ?? null;
  // `feed/extended` (a limit switch), `press/extend` (a coil): the axis.
  const base = p.name.includes("/") ? p.name.slice(0, p.name.indexOf("/")) : p.name;
  if (scene.sensors.includes(p.name)) return { kind: "sensor", name: p.name };
  if (p.source.startsWith("robot:") && scene.robots.includes(p.name)) {
    return { kind: "robot", name: p.name };
  }
  const d = device(p.name) ?? device(base);
  if (d && d.kind.kind !== "source" && d.kind.kind !== "sink") {
    return { kind: "device", name: d.name };
  }
  const lamp = scene.lamps.find((l) => l.signal === p.name && l.obstacle);
  if (lamp?.obstacle) return { kind: "obstacle", name: lamp.obstacle };
  // An analog input hangs on its transmitter.
  const analog = p.aspect ? undefined : scene.analogs?.find((a) => a.name === p.name && a.place);
  if (analog?.place) return { kind: "obstacle", name: analog.place };
  return null;
}

/** One line of a tag. */
export type TagLine = {
  key: string;
  address: string | null;
  name: string;
  /** `on` / `off` for a bool with a lane, `value` for a word, `none` when
   * there is nothing recorded to show. */
  state: "on" | "off" | "value" | "none";
  value?: string;
  forced?: boolean;
  /** The controller's reading and the model part here (a replayed log,
   * a monitor): the line shows the reading and wears the mark. */
  deviates?: boolean;
  direction: "input" | "output";
};

type Lanes = {
  signals: { name: string; times: number[]; values: boolean[] }[];
  words?: { name: string; times: number[]; values: number[]; integer: boolean }[];
};

function boolAt(times: number[], values: boolean[], t: number): boolean {
  let v = false;
  for (let i = 0; i < times.length && times[i] <= t + 1e-9; i++) v = values[i];
  return v;
}

function wordAt(times: number[], values: number[], t: number): number | null {
  let v: number | null = null;
  for (let i = 0; i < times.length && times[i] <= t + 1e-9; i++) v = values[i];
  return v;
}

/** The line `p` shows at `t` on the loaded timeline (`null`: authored
 * only, no levels). */
export function tagLine(
  p: IoPointMsg,
  lanes: Lanes | null,
  t: number,
  plant: PlantStatusMsg | null,
  analogs?: AnalogMsg[],
  readings?: ReadingsMsg | null,
): TagLine {
  const base = {
    key: pointKey(p),
    address: addressOf(p),
    name: p.label,
    direction: (p.direction === "output" ? "output" : "input") as "input" | "output",
  };
  // An analog input reads its scaled value in its unit, then the loop
  // current: `%IW100 dp 54.5 kPa 12.72mA`.
  const analog = p.aspect ? undefined : analogs?.find((a) => a.name === p.name);
  if (analog) {
    const r = analogReading(analog, lanes?.words, t);
    if (!r) return { ...base, state: "none" };
    const unit = analog.unit ? ` ${analog.unit}` : "";
    return {
      ...base,
      deviates: deviationAt(readings ?? null, `${p.name}.raw`, t) !== null || undefined,
      state: "value",
      value: r.open
        ? `${r.scaled.toFixed(1)}${unit} open`
        : `${r.scaled.toFixed(1)}${unit} ${r.current.toFixed(2)}mA`,
      forced: forcedWord(plant, p.name) !== null,
    };
  }
  const word = lanes?.words?.find((w) => w.name === p.label);
  if (word) {
    const v = wordAt(word.times, word.values, t);
    return {
      ...base,
      state: v === null ? "none" : "value",
      value: v === null ? undefined : word.integer ? String(Math.round(v)) : v.toFixed(3),
      deviates: deviationAt(readings ?? null, p.label, t) !== null || undefined,
    };
  }
  const lane = p.aspect ? undefined : lanes?.signals.find((s) => s.name === p.name);
  const forced = p.aspect ? undefined : plant?.forced.find(([name]) => name === p.name);
  if (!lane) return { ...base, state: "none", forced: forced !== undefined };
  // A point the controller read shows the reading — what the controller
  // sees — and is marked where the model's lane says otherwise.
  const read = p.aspect ? null : readLevel(readings ?? null, p.name, t);
  const level = read ?? boolAt(lane.times, lane.values, t);
  return {
    ...base,
    state: level ? "on" : "off",
    forced: forced !== undefined,
    deviates:
      (read !== null && deviationAt(readings ?? null, p.name, t) !== null) || undefined,
  };
}

/** Which points get a tag: the bound ones once anything is bound (the
 * cell has a wiring plan — show the wires), every point with a body
 * before that (names only). */
export function taggedPoints(points: IoPointMsg[]): IoPointMsg[] {
  const live = points.filter((p) => p.status !== "cosmetic" && p.status !== "internal");
  const anyBound = live.some((p) => p.status === "bound");
  return anyBound ? live.filter((p) => p.status === "bound") : live;
}

/** The point a label shows when several carry it (a handshake's two
 * ends, a fan-out): one on a PLC-family node first, then any bound one,
 * then the first. */
function preferred(ps: IoPointMsg[], nodes: IoNode[]): IoPointMsg {
  const plc = (p: IoPointMsg) => {
    const n = nodes.find((x) => x.name === p.node);
    return !!n && (n.kind.kind === "plc" || n.kind.kind === "safety_plc" || n.kind.kind === "remote_io");
  };
  // Between two PLC-family ends (a desk and a line PLC) the wire is shown
  // from where it is driven: the output.
  const plcs = ps.filter(plc);
  return (
    plcs.find((p) => p.direction === "output") ??
    plcs[0] ??
    ps.find((p) => p.status === "bound") ??
    ps[0]
  );
}

/** Tags grouped by resident: one board per anchor, one line per label —
 * a handshake's two ends (the PLC's output and the robot controller's
 * input of one `start`) are one point to look at, shown as the PLC
 * addresses it — inputs first, then outputs, each by name. */
export function tagGroups(
  points: IoPointMsg[],
  scene: TagScene,
  nodes: IoNode[],
): { anchor: TagAnchor; points: IoPointMsg[] }[] {
  const byAnchor = new Map<string, { anchor: TagAnchor; byLabel: Map<string, IoPointMsg[]> }>();
  for (const p of taggedPoints(points)) {
    const anchor = anchorOf(p, scene);
    if (!anchor) continue;
    const k = anchorKey(anchor);
    let g = byAnchor.get(k);
    if (!g) {
      g = { anchor, byLabel: new Map() };
      byAnchor.set(k, g);
    }
    g.byLabel.set(p.label, [...(g.byLabel.get(p.label) ?? []), p]);
  }
  return [...byAnchor.values()].map(({ anchor, byLabel }) => ({
    anchor,
    points: [...byLabel.values()]
      .map((ps) => preferred(ps, nodes))
      .sort(
        (a, b) =>
          (a.direction === b.direction ? 0 : a.direction === "input" ? -1 : 1) ||
          a.label.localeCompare(b.label),
      ),
  }));
}

/** The text of a line: `X20 beam`, `D0 feed.encoder 0.120`. */
export function lineText(line: TagLine): string {
  const parts = [line.address ?? "", line.name, line.value ?? "", line.deviates ? "≠" : ""].filter(
    (s) => s.length > 0,
  );
  return parts.join(" ");
}

import type { BomLineMsg, PartAttr } from "./protocol";

/**
 * How a line's attributes read on a card: which few to put first, and
 * what a value looks like. The headline keys are the ones the cell asks
 * of that kind of line (`scene.requirements()` — payload and reach of an
 * arm, stroke of a gripper, length / width / speed / load of a conveyor),
 * then the catalog viewer's card keys, then whatever numbers the line
 * states. Units come from the key suffix, the catalog's one convention
 * (`payload_kg`, `reach_mm`, `max_speed_mps`), so a value arrives in the
 * same words whichever way it was authored.
 */

/** Requirement keys and the attribute keys that answer them, in order
 * (the subset of `bt.select.ALIASES` a card asks). */
const ALIASES: Record<string, string[]> = {
  stroke_mm: ["stroke_mm", "opening_mm"],
  grip_force_max_n: ["grip_force_max_n", "grip_force_n", "gripping_force_n", "grip_force_min_n"],
  sensing_range_mm: ["sensing_range_mm", "range_mm", "max_range_mm"],
  range_mm: ["range_mm", "max_range_mm", "sensing_range_mm"],
  protective_height_mm: ["protective_height_mm", "height_mm"],
  width_mm: ["width_mm", "belt_width_mm"],
  speed_mps: ["speed_mps", "max_speed_mps", "speed_max_mps"],
  max_speed_mps: ["max_speed_mps", "speed_max_mps", "speed_mps"],
  load_kg: ["load_kg", "capacity_kg", "payload_kg", "payload_kg_per_m", "capacity_kg_per_level"],
  fov_h_deg: ["fov_h_deg", "hfov_deg", "fov_deg"],
  cable_m: ["cable_m", "cable_standard_m"],
};

/** Headline keys per category (longest matching prefix wins). A `a×b`
 * entry is one item made of two keys (`1280×720 px`). */
export const HEADLINE: [string, string[]][] = [
  ["manipulator.dual_arm", ["payload_kg", "arm_count", "reach_mm", "mass_kg"]],
  ["manipulator", ["payload_kg", "reach_mm", "repeatability_mm", "mass_kg"]],
  ["robot", ["payload_kg", "reach_mm", "repeatability_mm", "mass_kg"]],
  ["vehicle.mobile_manipulator", ["vertical_reach_min_mm", "vertical_reach_max_mm", "arm_count", "payload_kg"]],
  ["vehicle.uav", ["flight_time_min", "max_climb_mps", "max_descent_mps", "payload_kg"]],
  ["vehicle.legged", ["max_speed_mps", "max_step_height_mm", "payload_kg"]],
  ["vehicle", ["max_speed_mps", "payload_kg", "mass_kg"]],
  ["gripper.parallel", ["stroke_mm", "payload_kg", "grip_force_max_n", "mass_kg"]],
  ["gripper.multifinger", ["aperture_mm", "finger_count", "grip_force_max_n"]],
  ["gripper.vacuum", ["cup_count", "cup_diameter_mm", "max_vacuum_pct"]],
  ["gripper", ["stroke_mm", "payload_kg", "grip_force_max_n", "mass_kg"]],
  ["tool.screwdriver", ["torque_min_nm", "torque_max_nm", "thread_min_mm", "thread_max_mm"]],
  ["tool", ["stroke_mm", "payload_kg", "grip_force_max_n", "mass_kg"]],
  ["sensor.photoelectric", ["sensing_range_mm", "response_ms"]],
  ["sensor.light_curtain", ["protective_height_mm", "range_mm", "resolution_mm"]],
  ["sensor.area", ["range_mm"]],
  ["sensor.camera", ["fov_h_deg", "max_range_mm", "min_range_mm", "resolution_h_px×resolution_v_px"]],
  ["sensor.lidar", ["scan_fov_deg", "max_range_mm", "angular_resolution_deg", "scan_rate_hz"]],
  ["conveyor", ["length_mm", "width_mm", "speed_mps", "load_kg"]],
  ["axis.linear", ["stroke_mm", "speed_mps", "load_kg"]],
  ["external_axis", ["stroke_mm", "speed_mps", "load_kg"]],
  ["robot_controller", ["cable_m", "ip_rating", "mass_kg", "service_clearance_mm"]],
  ["structure.pedestal", ["load_kg", "height_mm", "mass_kg"]],
  ["structure.table", ["load_kg", "height_mm", "mass_kg"]],
  ["power_supply", ["output_v", "output_a"]],
];

/** What any other line leads with, when it states them. */
const FALLBACK = ["footprint_x_mm×footprint_y_mm", "height_mm", "capacity_kg", "mass_kg"];

/** Never auto-filled into a headline: a count, not a capability. */
const NOT_HEADLINE = new Set(["dof"]);

/** Unit by key suffix. Order matters where one suffix ends another. */
const UNITS: [RegExp, string][] = [
  [/_kgm2$/, "kg·m²"],
  [/_kg$/, "kg"],
  [/_mm$/, "mm"],
  [/_mps$/, "m/s"],
  [/_ms$/, "ms"],
  [/_m$/, "m"],
  [/_deg$/, "°"],
  [/_hz$/, "Hz"],
  [/_px$/, "px"],
  [/_min$/, "min"],
  [/_h$/, "h"],
  [/_s$/, "s"],
  [/_nm$/, "N·m"],
  [/_n$/, "N"],
  [/_v$/, "V"],
  [/_a$/, "A"],
  [/_w$/, "W"],
  [/_pct$/, "%"],
  [/_c$/, "°C"],
  [/_lpm$/, "L/min"],
  [/_rpm$/, "rpm"],
];

/** Units that attach to the number without a space. */
const TIGHT = new Set(["°", "%"]);

/** The bare unit a `_per_<x>` denominator names. */
const PER: Record<string, string> = { m: "m", mm: "mm", s: "s", min: "min", h: "h", kg: "kg" };

/** The unit a key's suffix encodes (`payload_kg` → `kg`, `payload_kg_per_m`
 * → `kg/m`), `""` when it names none. */
export function unitOf(key: string): string {
  const per = key.indexOf("_per_");
  if (per >= 0) {
    const denominator = key.slice(per + 5);
    const numerator = unitOf(key.slice(0, per));
    return numerator ? `${numerator}/${PER[denominator] ?? denominator}` : "";
  }
  for (const [suffix, unit] of UNITS) if (suffix.test(key)) return unit;
  return "";
}

/** The key without its unit suffix, words spaced (`fov_h_deg` → `fov h`,
 * `max_tcp_speed_mps` → `max TCP speed`). */
export function labelOf(key: string): string {
  let stem = key;
  const per = stem.indexOf("_per_");
  if (per >= 0) stem = stem.slice(0, per);
  for (const [suffix] of UNITS) {
    if (suffix.test(stem)) {
      stem = stem.replace(suffix, "");
      break;
    }
  }
  return stem
    .split("_")
    .filter(Boolean)
    .map((w) => (w === "ip" || w === "tcp" ? w.toUpperCase() : w))
    .join(" ");
}

/** A number as the tables print it: integers plain, else at most three
 * decimals with the trailing zeros gone. */
export function formatNumber(value: number): string {
  if (!Number.isFinite(value)) return String(value);
  if (Number.isInteger(value)) return String(value);
  return String(parseFloat(value.toFixed(3)));
}

/** A value as the card prints it: a number (or a spec pack's numeric
 * string) with its unit, any other text as written. */
export function formatValue(key: string, value: PartAttr): string {
  const n = typeof value === "number" ? value : /^\s*-?\d+(\.\d+)?\s*$/.test(value) ? parseFloat(value) : NaN;
  if (Number.isNaN(n)) return String(value);
  const unit = unitOf(key);
  if (!unit) return formatNumber(n);
  return TIGHT.has(unit) ? `${formatNumber(n)}${unit}` : `${formatNumber(n)} ${unit}`;
}

/** One entry of a card's spec list. `key` is the attribute the value came
 * from (the tooltip — agents and tests speak keys), `label` its words. */
export interface SpecItem {
  key: string;
  label: string;
  value: string;
}

function has(line: BomLineMsg, key: string): boolean {
  return Object.prototype.hasOwnProperty.call(line.attributes, key);
}

/** The attribute answering `key` on the line: itself, else an alias. */
function answer(line: BomLineMsg, key: string): string | null {
  for (const k of ALIASES[key] ?? [key]) if (has(line, k)) return k;
  return null;
}

function pairItem(line: BomLineMsg, left: string, right: string): SpecItem | null {
  if (!has(line, left) || !has(line, right)) return null;
  const a = formatValue(left, line.attributes[left]);
  const b = formatValue(right, line.attributes[right]);
  const unit = unitOf(left);
  const strip = (s: string) => (unit && s.endsWith(unit) ? s.slice(0, -unit.length).trim() : s);
  const words = labelOf(left).split(" ");
  if (words.length > 1 && words[words.length - 1].length === 1) words.pop();
  return { key: `${left}×${right}`, label: words.join(" "), value: unit ? `${strip(a)}×${b}` : `${a}×${b}` };
}

/** The headline keys for a category: the longest matching table prefix. */
export function headlineKeys(category: string): string[] {
  let best: [string, string[]] | null = null;
  for (const entry of HEADLINE) {
    const [prefix] = entry;
    if (category === prefix || category.startsWith(`${prefix}.`)) {
      if (!best || prefix.length > best[0].length) best = entry;
    }
  }
  return best ? best[1] : [];
}

/** The first `max` specs a line leads with (§5.3): the keys the cell asks
 * of its kind, then the fallback keys, then the rest of its numbers. */
export function headlineSpecs(line: BomLineMsg, max = 4): SpecItem[] {
  const out: SpecItem[] = [];
  const used = new Set<string>();
  const push = (item: SpecItem, keys: string[]) => {
    if (out.length >= max) return;
    out.push(item);
    for (const k of keys) used.add(k);
  };
  const consider = (spec: string) => {
    if (out.length >= max) return;
    const pair = spec.split("×");
    if (pair.length === 2) {
      const item = pairItem(line, pair[0], pair[1]);
      if (item) push(item, pair);
      return;
    }
    const key = answer(line, spec);
    if (key && !used.has(key)) {
      push({ key, label: labelOf(key), value: formatValue(key, line.attributes[key]) }, [key]);
    }
  };
  for (const spec of headlineKeys(line.category)) consider(spec);
  for (const spec of FALLBACK) consider(spec);
  for (const [key, value] of Object.entries(line.attributes)) {
    if (out.length >= max) break;
    if (used.has(key) || NOT_HEADLINE.has(key)) continue;
    if (typeof value === "number" || /^\s*-?\d+(\.\d+)?\s*$/.test(value)) {
      push({ key, label: labelOf(key), value: formatValue(key, value) }, [key]);
    }
  }
  return out;
}

/** Every attribute of a line: the headline block first, the rest in key
 * order. */
export function allSpecs(line: BomLineMsg, max = 4): SpecItem[] {
  const head = headlineSpecs(line, max);
  const used = new Set(head.flatMap((h) => h.key.split("×")));
  const rest = Object.entries(line.attributes)
    .filter(([key]) => !used.has(key))
    .map(([key, value]) => ({ key, label: labelOf(key), value: formatValue(key, value) }));
  return [...head, ...rest];
}

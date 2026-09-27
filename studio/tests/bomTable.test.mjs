import assert from "node:assert/strict";
import { test } from "node:test";
import {
  NO_FILTER,
  attributeCell,
  bomCaption,
  bomRows,
  catalogCell,
  namesCell,
  rowSelected,
  selectionForTarget,
  totalColumns,
} from "../src/bomTable.ts";

const SHA = "1d6bfc13fc805f2290db0da96109ba20919cce65";
const target = (kind, name, derived = false) => ({ kind, name, derived });
const line = (over) => ({
  category: "part", manufacturer: null, model: null, catalog: null, qty: 1, description: null,
  attributes: {}, order: null, targets: [], identified: false, ...over,
});
const LINES = [
  line({ category: "manipulator", manufacturer: "Universal Robots", model: "UR5e", catalog: { id: "universal_robots/ur/ur5e/r2", revision: SHA }, attributes: { mass_kg: 20.7, payload_kg: 5 }, identified: true, targets: [target("robot", "arm")] }),
  line({ category: "tool", manufacturer: "Robotiq", model: "2F-85", identified: true, targets: [target("tool", "arm/tool", true)] }),
  line({ category: "robot_controller", targets: [target("io_node", "arm/controller", true)] }),
  line({ category: "structure.fence", model: "FP-1000", qty: 12, attributes: { mass_kg: 8 }, identified: true, targets: [target("group", "fence/east")] }),
  line({ category: "part", model: "T1", qty: 2, attributes: { mass_kg: 30 }, identified: true, targets: [target("obstacle", "table_a"), target("obstacle", "table_b")] }),
];
const TOTALS = { mass_kg: 20.7 + 12 * 8 + 2 * 30 };

test("rows filter to the to-do list or to the derived lines; the caption counts", () => {
  assert.equal(bomRows(LINES, NO_FILTER).length, 5);
  assert.deepEqual(bomRows(LINES, { unidentified: true, derived: false }).map((l) => l.category), ["robot_controller"]);
  assert.deepEqual(bomRows(LINES, { unidentified: false, derived: true }).map((l) => l.category), ["tool", "robot_controller"]);
  assert.equal(bomCaption(LINES, TOTALS), "5 lines · 1 unidentified · mass 176.7 kg");
  assert.equal(bomCaption([], {}), "no lines — nothing on the bill yet");
  // Only what adds up: a reach or a payload summed over the bill means nothing.
  assert.deepEqual(totalColumns({ power_w: 1, mass_kg: 2, reach_mm: 850, dof: 6, price_jpy: 3 }), ["mass_kg", "power_w", "price_jpy"]);
});

test("cells read as the CSV writes them", () => {
  assert.equal(catalogCell(LINES[0]), "universal_robots/ur/ur5e/r2 @1d6bfc1");
  assert.equal(catalogCell(LINES[1]), "");
  assert.equal(namesCell(LINES[4]), "table_a; table_b");
  assert.equal(attributeCell(LINES[0], "mass_kg"), "20.7 kg");
  assert.equal(attributeCell(LINES[1], "mass_kg"), "—");
});

test("a row selects what it stands for, the way the tree would", () => {
  const declared = new Set(["UR"]);
  assert.deepEqual(selectionForTarget(target("robot", "arm"), declared), { type: "tcp", robot: "arm" });
  assert.deepEqual(selectionForTarget(target("obstacle", "/World/Crate"), declared), { type: "obstacle", name: "/World/Crate" });
  assert.deepEqual(selectionForTarget(target("group", "fence/east"), declared), { type: "group", path: "/fence/east" });
  assert.deepEqual(selectionForTarget(target("group", "/World/Pedestal"), declared), { type: "group", path: "/World/Pedestal" });
  assert.deepEqual(selectionForTarget(target("sensor", "eye"), declared), { type: "sensor", name: "eye" });
  assert.deepEqual(selectionForTarget(target("io_node", "UR"), declared), { type: "io_node", name: "UR" });
  assert.deepEqual(selectionForTarget(target("io_node", "arm/controller", true), declared), { type: "line", kind: "io_node", name: "arm/controller" });
  assert.deepEqual(selectionForTarget(target("tool", "arm/tool", true), declared), { type: "line", kind: "tool", name: "arm/tool" });
});

test("the selected row is the one whose target is selected", () => {
  assert.equal(rowSelected(LINES[0], { type: "tcp", robot: "arm" }), true);
  assert.equal(rowSelected(LINES[3], { type: "group", path: "/fence/east" }), true);
  assert.equal(rowSelected(LINES[4], { type: "obstacle", name: "table_b" }), true);
  assert.equal(rowSelected(LINES[1], { type: "line", kind: "tool", name: "arm/tool" }), true);
  assert.equal(rowSelected(LINES[1], { type: "obstacle", name: "arm/tool" }), false);
});

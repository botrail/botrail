import assert from "node:assert/strict";
import { test } from "node:test";
import {
  allSpecs,
  formatValue,
  headlineKeys,
  headlineSpecs,
  labelOf,
  unitOf,
} from "../src/specFormat.ts";

function line(category, attributes) {
  return {
    category, manufacturer: null, model: null, catalog: null, qty: 1, description: null,
    attributes, order: null, targets: [], identified: true,
  };
}

test("units come from the key suffix, per-units from `_per_`", () => {
  assert.equal(unitOf("payload_kg"), "kg");
  assert.equal(unitOf("reach_mm"), "mm");
  assert.equal(unitOf("cable_m"), "m");
  assert.equal(unitOf("max_speed_mps"), "m/s");
  assert.equal(unitOf("response_ms"), "ms");
  assert.equal(unitOf("fov_h_deg"), "°");
  assert.equal(unitOf("torque_max_nm"), "N·m");
  assert.equal(unitOf("grip_force_max_n"), "N");
  assert.equal(unitOf("flight_time_min"), "min");
  assert.equal(unitOf("max_vacuum_pct"), "%");
  assert.equal(unitOf("payload_kg_per_m"), "kg/m");
  assert.equal(unitOf("capacity_kg_per_level"), "kg/level");
  assert.equal(unitOf("dof"), "");
  assert.equal(unitOf("ip_rating"), "");
});

test("labels drop the unit and space the words", () => {
  assert.equal(labelOf("payload_kg"), "payload");
  assert.equal(labelOf("fov_h_deg"), "fov h");
  assert.equal(labelOf("max_tcp_speed_mps"), "max TCP speed");
  assert.equal(labelOf("ip_rating"), "IP rating");
  assert.equal(labelOf("payload_kg_per_m"), "payload");
  assert.equal(labelOf("repeatability_mm"), "repeatability");
});

test("values: numbers and numeric strings get their unit, text stays as written", () => {
  assert.equal(formatValue("payload_kg", 5), "5 kg");
  assert.equal(formatValue("mass_kg", 20.7), "20.7 kg");
  assert.equal(formatValue("repeatability_mm", 0.03), "0.03 mm");
  assert.equal(formatValue("speed_mps", "0.15"), "0.15 m/s");
  assert.equal(formatValue("height_mm", "550"), "550 mm");
  assert.equal(formatValue("fov_h_deg", 87), "87°");
  assert.equal(formatValue("max_vacuum_pct", 80), "80%");
  assert.equal(formatValue("dof", 6), "6");
  assert.equal(formatValue("ip_rating", "IP54"), "IP54");
  assert.equal(formatValue("color", "RAL 7035"), "RAL 7035");
  assert.equal(formatValue("reach_mm", 1249.0004), "1249 mm");
});

test("headline keys follow the category, longest prefix first", () => {
  assert.deepEqual(headlineKeys("manipulator"), ["payload_kg", "reach_mm", "repeatability_mm", "mass_kg"]);
  assert.deepEqual(headlineKeys("manipulator.dual_arm")[1], "arm_count");
  assert.deepEqual(headlineKeys("conveyor.belt"), ["length_mm", "width_mm", "speed_mps", "load_kg"]);
  assert.deepEqual(headlineKeys("gripper.parallel")[0], "stroke_mm");
  assert.deepEqual(headlineKeys("structure.cabinet"), []);
  assert.deepEqual(headlineKeys("plc"), []);
});

test("an arm leads with what the cell asks of it, at most four", () => {
  const arm = line("manipulator", {
    dof: 6, mass_kg: 20.7, max_tcp_speed_mps: 1, payload_kg: 5, reach_mm: 850, repeatability_mm: 0.03,
  });
  assert.deepEqual(headlineSpecs(arm).map((s) => `${s.label} ${s.value}`),
    ["payload 5 kg", "reach 850 mm", "repeatability 0.03 mm", "mass 20.7 kg"]);
  // Everything, headline first, dof and the rest after.
  assert.deepEqual(allSpecs(arm).map((s) => s.key),
    ["payload_kg", "reach_mm", "repeatability_mm", "mass_kg", "dof", "max_tcp_speed_mps"]);
});

test("a conveyor's load reads through the alias the spec pack wrote, with its own label", () => {
  const conv = line("conveyor.belt", {
    height_mm: "550", length_mm: "3800", mass_kg: 44.5, payload_kg_per_m: "25", speed_mps: "0.15", width_mm: "400",
  });
  assert.deepEqual(headlineSpecs(conv).map((s) => `${s.label} ${s.value}`),
    ["length 3800 mm", "width 400 mm", "speed 0.15 m/s", "payload 25 kg/m"]);
  assert.deepEqual(headlineSpecs(conv).map((s) => s.key)[3], "payload_kg_per_m");
});

test("a camera's resolution is one item of two keys; a rack falls back to footprint and mass", () => {
  const cam = line("sensor.camera", {
    fov_h_deg: 87, fov_v_deg: 58, resolution_h_px: 1280, resolution_v_px: 720, min_range_mm: 70, max_range_mm: 500,
  });
  assert.deepEqual(headlineSpecs(cam).map((s) => `${s.label} ${s.value}`),
    ["fov h 87°", "max range 500 mm", "min range 70 mm", "resolution 1280×720 px"]);
  const rack = line("structure.rack", { depth_mm: "450", height_mm: "1800", levels: "4", mass_kg: 18.4, width_mm: "900" });
  // No table for the category: the fallback keys it states, then its numbers in key order.
  assert.deepEqual(headlineSpecs(rack).map((s) => s.key), ["height_mm", "mass_kg", "depth_mm", "levels"]);
  const bin = line("bin", { footprint_x_mm: 400, footprint_y_mm: 300, height_mm: 147, mass_kg: 1.29 });
  assert.deepEqual(headlineSpecs(bin).map((s) => `${s.label} ${s.value}`),
    ["footprint 400×300 mm", "height 147 mm", "mass 1.29 kg"]);
  // A line with a table but none of its keys still leads with what it has — never dof.
  const bare = line("manipulator", { dof: 6, mass_kg: 40 });
  assert.deepEqual(headlineSpecs(bare).map((s) => s.key), ["mass_kg"]);
  assert.deepEqual(headlineSpecs(line("plc", {})), []);
});

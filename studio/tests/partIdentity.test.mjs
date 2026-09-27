import assert from "node:assert/strict";
import { test } from "node:test";
import {
  ancestors,
  hasTarget,
  identityFor,
  identityOf,
  identityText,
  indexBom,
  partLabel,
  partTitle,
  robotStack,
  shortRevision,
} from "../src/partIdentity.ts";

const SHA = "1d6bfc13fc805f2290db0da96109ba20919cce65";

function line(over) {
  return {
    category: "part",
    manufacturer: null,
    model: null,
    catalog: null,
    qty: 1,
    description: null,
    attributes: {},
    order: null,
    targets: [],
    identified: false,
    ...over,
  };
}
const target = (kind, name, derived = false) => ({ kind, name, derived });

// The docs demo's shape: a catalog arm with a tool and a set controller,
// a bare URDF robot, a generated conveyor, a fence pinned as a group with
// a USD-style leading slash, a pinned prim, a declared PLC.
const LINES = [
  line({
    category: "manipulator", manufacturer: "Universal Robots", model: "UR5e",
    catalog: { id: "universal_robots/ur/ur5e/r2", revision: SHA },
    attributes: { payload_kg: 5, reach_mm: 850 }, identified: true,
    targets: [target("robot", "arm")],
  }),
  line({
    category: "tool", manufacturer: "Robotiq", model: "2F-85",
    catalog: { id: "robotiq/2f/2f-85/r3", revision: SHA }, identified: true,
    targets: [target("tool", "arm/tool", true)],
  }),
  line({
    category: "robot_controller", manufacturer: "Universal Robots", model: null,
    catalog: { id: "universal_robots/ur/ur5e/r2", revision: SHA }, identified: true,
    description: "in the UR5e set: Control Box",
    targets: [target("io_node", "arm/controller", true)],
  }),
  line({ category: "robot", targets: [target("robot", "panda")] }),
  line({ category: "robot_controller", targets: [target("io_node", "panda/controller", true)] }),
  line({
    category: "conveyor.belt", manufacturer: "botrail", model: "BCU-400-3800",
    catalog: { id: "botrail/conveyor/belt-unit/r1", revision: SHA }, identified: true,
    targets: [target("device", "conv")],
  }),
  line({
    category: "structure.fence", model: "FP-1000", qty: 12, identified: true,
    targets: [target("group", "/World/fence/east")],
  }),
  line({ category: "structure.pedestal", targets: [target("group", "/World/Pedestal")] }),
  line({ category: "plc", model: "R04CPU", identified: true, targets: [target("io_node", "PLC1")] }),
  line({
    category: "sensor.light_curtain", manufacturer: "キーエンス", model: "GL-R22L", identified: true,
    targets: [target("sensor", "gate_curtain")],
  }),
];
const index = indexBom(LINES);

test("a selection resolves to its own line, by kind and name", () => {
  assert.equal(identityFor({ type: "tcp", robot: "arm" }, index)?.line.model, "UR5e");
  assert.equal(identityFor({ type: "robot", robot: "panda" }, index)?.line.category, "robot");
  assert.equal(identityFor({ type: "device", name: "conv" }, index)?.line.model, "BCU-400-3800");
  assert.equal(identityFor({ type: "io_node", name: "PLC1" }, index)?.line.model, "R04CPU");
  assert.equal(identityFor({ type: "line", kind: "tool", name: "arm/tool" }, index)?.line.model, "2F-85");
  assert.equal(identityFor({ type: "sensor", name: "nothing" }, index), null);
});

test("a prim inherits the group pin above it, whichever way the slash was written", () => {
  const leaf = identityFor({ type: "obstacle", name: "/World/fence/east/panels/w1000/3" }, index);
  assert.equal(leaf?.line.model, "FP-1000");
  assert.equal(leaf?.inherited, true);
  assert.equal(leaf?.ancestor, "World/fence/east");
  // The same prim named without the leading slash.
  assert.equal(identityFor({ type: "obstacle", name: "World/fence/east/panels/w1000/3" }, index)?.ancestor, "World/fence/east");
  // A sub-group inside the pinned group inherits too; the pinned group is its own.
  assert.equal(identityFor({ type: "group", path: "/World/fence/east/panels" }, index)?.inherited, true);
  assert.equal(identityFor({ type: "group", path: "/World/fence/east" }, index)?.inherited, false);
  // Nothing above an unpinned prim: plain geometry.
  assert.equal(identityFor({ type: "obstacle", name: "/World/Floor" }, index), null);
  // Only prims and groups inherit — a device never reads a group's pin.
  assert.equal(identityOf(index, "device", "World/fence/east/x"), null);
  // A generator's body sits under the resident's own name: the conveyor's
  // slab is part of the device, the light curtain's post — and the group
  // row the posts make — part of the sensor.
  assert.equal(identityFor({ type: "obstacle", name: "conv/belt" }, index)?.line.model, "BCU-400-3800");
  assert.equal(identityFor({ type: "obstacle", name: "conv/belt" }, index)?.ancestor, "conv");
  const post = identityFor({ type: "obstacle", name: "gate_curtain/column_a" }, index);
  assert.equal(post?.line.model, "GL-R22L");
  assert.equal(post?.inherited, true);
  const posts = identityFor({ type: "group", path: "/gate_curtain" }, index);
  assert.equal(posts?.line.model, "GL-R22L");
  assert.equal(posts?.ancestor, "gate_curtain");
  assert.deepEqual(ancestors("/a/b/c"), ["a/b", "a"]);
  assert.deepEqual(ancestors("a"), []);
});

test("labels: model first, `?` for unidentified equipment, the category for a pinned prim", () => {
  const [arm, tool, ctrl, panda, pandaCtrl, conv, fence, pedestal, plc] = LINES;
  assert.equal(partLabel(arm, arm.targets[0]), "UR5e");
  assert.equal(partLabel(tool, tool.targets[0]), "2F-85");
  assert.equal(partLabel(ctrl, ctrl.targets[0]), "in the set");
  assert.equal(partLabel(panda, panda.targets[0]), "?");
  assert.equal(partLabel(pandaCtrl, pandaCtrl.targets[0]), "?");
  assert.equal(partLabel(conv, conv.targets[0]), "BCU-400-3800");
  assert.equal(partLabel(fence, fence.targets[0]), "FP-1000");
  assert.equal(partLabel(pedestal, pedestal.targets[0]), "structure.pedestal");
  assert.equal(partLabel(plc, plc.targets[0]), "R04CPU");
  // A catalog reference with no model reads as the product part of the id.
  const bare = line({ category: "gripper", catalog: { id: "robotiq/2f/2f-85/r3", revision: null }, identified: true, targets: [target("tool", "arm/tool", true)] });
  assert.equal(partLabel(bare, bare.targets[0]), "2f-85/r3");
});

test("the focus chip text and the tooltip", () => {
  const [arm, , ctrl, panda, , , fence] = LINES;
  assert.equal(identityText(arm), "Universal Robots UR5e");
  assert.equal(identityText(ctrl), "controller in the set");
  assert.equal(identityText(panda), "unidentified (robot)");
  assert.equal(partTitle(arm), `Universal Robots UR5e (universal_robots/ur/ur5e/r2@1d6bfc1)`);
  assert.equal(partTitle(fence), "FP-1000 ×12");
  assert.equal(partTitle(panda), "unidentified — no maker, model or catalog reference (robot)");
  assert.equal(shortRevision(SHA), "1d6bfc1");
  assert.equal(shortRevision("local-sha256:abcd"), "local");
  assert.equal(shortRevision(null), null);
});

test("a robot's stack: its tools and the controller it needs, nothing of another robot", () => {
  const arm = robotStack(index, "arm");
  assert.deepEqual(arm.tools.map((t) => t.name), ["arm/tool"]);
  assert.equal(arm.controller?.line.description, "in the UR5e set: Control Box");
  const panda = robotStack(index, "panda");
  assert.deepEqual(panda.tools, []);
  assert.equal(panda.controller?.line.identified, false);
  // A declared cabinet leaves no derived controller line.
  assert.equal(robotStack(index, "nobody").controller, null);
  assert.equal(hasTarget(LINES, "tool", "arm/tool"), true);
  assert.equal(hasTarget(LINES, "tool", "arm/tool2"), false);
});

import assert from "node:assert/strict";
import { test } from "node:test";
import * as THREE from "three";

import { indexBom, inGroup } from "../src/partIdentity.ts";
import { HAND_REACH, handUnit, heldPose, holdOffset, localBounds, underHand } from "../src/xr/grab.ts";

const close = (a, b, eps = 1e-9, what = "") =>
  assert.ok(Math.abs(a - b) <= eps, `${what} ${a} vs ${b}`);
const closeAll = (a, b, eps = 1e-9, what = "") => a.forEach((v, i) => close(v, b[i], eps, `${what}[${i}]`));

/** An obstacle as the studio draws it: a posed group, the size on a mesh below it. */
function obstacle(name, size, position, yaw = 0) {
  const group = new THREE.Group();
  group.position.set(...position);
  group.quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), yaw);
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshBasicMaterial());
  mesh.scale.set(...size);
  group.add(mesh);
  const box = localBounds(group);
  return { name, box, matrixWorld: group.matrixWorld, group };
}
const at = (x, y, z) => new THREE.Vector3(x, y, z);

test("local bounds are the drawn meshes in the obstacle's own frame", () => {
  const { box, group } = obstacle("conveyor", [3, 0.4, 0.1], [5, 2, 0.5], Math.PI / 4);
  closeAll(box.min.toArray(), [-1.5, -0.2, -0.05], 1e-9, "min");
  closeAll(box.max.toArray(), [1.5, 0.2, 0.05], 1e-9, "max");
  // A second piece off to the side widens it; a hidden one does not.
  const lamp = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.1, 0.1), new THREE.MeshBasicMaterial());
  lamp.position.set(0, 0, 0.4);
  const hidden = new THREE.Mesh(new THREE.BoxGeometry(9, 9, 9), new THREE.MeshBasicMaterial());
  hidden.visible = false;
  group.add(lamp, hidden);
  const wider = localBounds(group);
  closeAll(wider.max.toArray(), [1.5, 0.2, 0.45], 1e-9, "with the lamp");
  closeAll(wider.min.toArray(), [-1.5, -0.2, -0.05], 1e-9, "hidden ignored");
  assert.equal(localBounds(new THREE.Group()), null, "nothing drawn, nothing to take");
  group.visible = false;
  assert.equal(localBounds(group), null, "a stowed part is not there");
});

test("the hand takes what is within reach of it, nearest first", () => {
  const crate = obstacle("crate", [0.3, 0.3, 0.3], [0, 0, 0.15]);
  const table = obstacle("table", [1.2, 0.8, 0.05], [2, 0, 0.7]);
  const things = [crate, table];
  assert.equal(underHand(at(0, 0, 0.15), null, things), "crate", "inside it");
  assert.equal(underHand(at(0.15 + 0.05, 0, 0.15), null, things), "crate", "5 cm off its side");
  assert.equal(underHand(at(0.15 + HAND_REACH + 0.01, 0, 0.15), null, things), null, "just out of reach");
  assert.equal(underHand(at(1, 0, 1.5), null, things), null, "mid-air");
  // On the table top, with a part 4 cm further: the table.
  const part = obstacle("part", [0.05, 0.05, 0.05], [2, 0, 0.79]);
  assert.equal(underHand(at(2, 0, 0.73), null, [table, part]), "table");
  assert.equal(underHand(at(2, 0, 0.77), null, [table, part]), "part", "touching the part");
  assert.equal(underHand(at(NaN, 0, 0), null, things), null, "an untracked hand takes nothing");
});

test("a turned obstacle is met by its own box, not the world's", () => {
  // 3 m long, turned 45°: its world box covers the corner (1, -1), the conveyor does not.
  const conveyor = obstacle("conveyor", [3, 0.4, 0.1], [0, 0, 0.5], Math.PI / 4);
  assert.equal(underHand(at(0.9, -0.9, 0.5), null, [conveyor]), null);
  assert.equal(underHand(at(0.9, 0.9, 0.5), null, [conveyor]), "conveyor", "along its length");
});

test("of things as near as each other the smaller is in the hand", () => {
  const tote = obstacle("tote", [0.6, 0.4, 0.3], [0, 0, 0.15]);
  const part = obstacle("part", [0.06, 0.06, 0.06], [0.1, 0, 0.05]);
  assert.equal(underHand(at(0.1, 0, 0.05), null, [tote, part]), "part", "inside both");
  assert.equal(underHand(at(0.1, 0, 0.05), null, [part, tote]), "part", "whatever the order");
  assert.equal(underHand(at(-0.2, 0, 0.2), null, [tote, part]), "tote", "elsewhere in the tote");
});

test("what surrounds the head is where one stands, not something in the hand", () => {
  const room = obstacle("room", [8, 6, 3], [0, 0, 1.5]);
  const mug = obstacle("mug", [0.08, 0.08, 0.1], [1, 1, 0.8]);
  const head = at(0, 0, 1.6);
  assert.equal(underHand(at(0.3, 0.2, 1.2), head, [room, mug]), null, "a squeeze in mid-air");
  assert.equal(underHand(at(1, 1, 0.8), head, [room, mug]), "mug");
  assert.equal(underHand(at(1, 1.07, 0.8), head, [room, mug]), "mug", "beside the mug, inside the room");
  // From outside, the room is a thing like any other.
  assert.equal(underHand(at(4.05, 0, 1.2), at(5, 0, 1.6), [room, mug]), "room");
});

test("a group is its names under the tree path, with or without the leading slash", () => {
  assert.ok(inGroup("/World/Pedestal/Column", "/World/Pedestal"));
  assert.ok(!inGroup("/World/PedestalFar/Column", "/World/Pedestal"), "a sibling sharing the prefix");
  assert.ok(inGroup("fence/east/posts/c0", "/fence/east"), "a generated unit's names are bare");
  assert.ok(inGroup("stack_light/mast", "/stack_light"));
  assert.ok(!inGroup("stack_light", "/stack_light"), "not itself");
});

const line = (targets) => ({
  category: "part", manufacturer: null, model: null, catalog: null, qty: 1, description: null,
  attributes: {}, order: null, identified: false, targets,
});
const BOM = indexBom([
  line([{ kind: "group", name: "/World/Pallet", derived: false }]),
  line([{ kind: "obstacle", name: "/World/Pallet/Crate_1", derived: false }]),
  line([{ kind: "group", name: "stack_light", derived: false }]),
  line([{ kind: "device", name: "conv", derived: false }]),
]);
const TCP = { type: "tcp", robot: "panda" };

test("what comes with the obstacle under the hand: the selection, else its part, else itself", () => {
  // A workpiece with a line of its own comes alone, though it sits in the pallet's group.
  assert.equal(handUnit("/World/Pallet/Crate_1", TCP, BOM), null);
  // A board of the pallet brings the pallet; a lamp the stack light; a rail the conveyor.
  assert.equal(handUnit("/World/Pallet/DeckBoard_3", TCP, BOM), "/World/Pallet");
  assert.equal(handUnit("stack_light/green", TCP, BOM), "/stack_light");
  assert.equal(handUnit("conv/rail_l", TCP, BOM), "/conv");
  // The bill knows nothing of it: itself.
  assert.equal(handUnit("/World/Floor", TCP, BOM), null);
  assert.equal(handUnit("/World/Pallet/DeckBoard_3", TCP, indexBom([])), null);
  // The selection overrides, when the hand is on it.
  assert.equal(
    handUnit("/World/Pallet/Crate_1", { type: "group", path: "/World/Pallet" }, BOM),
    "/World/Pallet",
    "the selected group whole",
  );
  assert.equal(
    handUnit("/World/Pallet/DeckBoard_3", { type: "obstacle", name: "/World/Pallet/DeckBoard_3" }, BOM),
    null,
    "the selected board alone",
  );
  assert.equal(
    handUnit("stack_light/green", { type: "group", path: "/World/Pallet" }, BOM),
    "/stack_light",
    "a selection elsewhere says nothing",
  );
});

test("a held thing keeps its place in the hand as the hand moves and turns", () => {
  const pose = { position: [1, 2, 0.5], quaternion: [0, 0, Math.sin(0.2), Math.cos(0.2)] };
  const grip = new THREE.Matrix4().compose(
    at(1.1, 2, 0.6),
    new THREE.Quaternion().setFromEuler(new THREE.Euler(0.3, -0.2, 0.9)),
    at(1, 1, 1),
  );
  const offset = holdOffset(grip, pose);
  const still = heldPose(grip, offset);
  closeAll(still.position, pose.position, 1e-12, "taken where it is");
  closeAll(still.quaternion, pose.quaternion, 1e-12, "as it is turned");
  // The hand moves 30 cm up and turns a quarter about the vertical through itself.
  const quarter = new THREE.Matrix4().makeRotationZ(Math.PI / 2);
  const hand = at(1.1, 2, 0.6);
  const moved = new THREE.Matrix4()
    .makeTranslation(hand.x, hand.y, hand.z + 0.3)
    .multiply(quarter)
    .multiply(new THREE.Matrix4().makeTranslation(-hand.x, -hand.y, -hand.z))
    .multiply(grip);
  const now = heldPose(moved, offset);
  // The thing was 10 cm to the hand's -x and 10 cm below: now 10 cm to its -y.
  closeAll(now.position, [1.1, 1.9, 0.8], 1e-9, "carried round the hand");
  const expected = new THREE.Quaternion(0, 0, Math.sin(Math.PI / 4), Math.cos(Math.PI / 4)).multiply(
    new THREE.Quaternion(...pose.quaternion),
  );
  close(Math.abs(new THREE.Quaternion(...now.quaternion).dot(expected)), 1, 1e-12, "turned with it");
});

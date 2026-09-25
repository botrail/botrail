import assert from "node:assert/strict";
import { test } from "node:test";
import * as THREE from "three";
import { finishMaps, isFinish, metricGeometry } from "../src/three/finishes.ts";

test("a finish is three seamless tiles shared by every material that draws it", () => {
  for (const name of ["wood", "checker_plate", "plastic"]) {
    assert.ok(isFinish(name));
    const maps = finishMaps(name);
    assert.equal(finishMaps(name), maps, "built once, shared");
    for (const key of ["map", "normalMap", "roughnessMap"]) {
      const t = maps[key];
      assert.equal(t.wrapS, THREE.RepeatWrapping);
      assert.equal(t.repeat.x, 1 / maps.pitch, `${name} ${key} repeats at its pitch`);
      assert.equal(t.image.width, 128);
    }
    // The tint multiplies the authored colour: it is never dark enough to
    // change what colour a board or a plate is.
    const tint = maps.map.image.data;
    let lo = 255;
    for (let i = 0; i < tint.length; i += 4) lo = Math.min(lo, tint[i]);
    assert.ok(lo >= 0.7 * 255, `${name} tint floor ${lo}`);
    assert.equal(maps.map.colorSpace, THREE.SRGBColorSpace);
    assert.equal(maps.normalMap.colorSpace, THREE.NoColorSpace);
  }
  assert.equal(isFinish("marble"), false);
  assert.equal(isFinish(null), false);
});

test("tread plate stands proud and timber is mostly tint", () => {
  const flatness = (name) => {
    const n = finishMaps(name).normalMap.image.data;
    let sum = 0;
    for (let i = 0; i < n.length; i += 4) sum += n[i + 2];
    return sum / (n.length / 4) / 255;
  };
  assert.ok(flatness("checker_plate") < flatness("wood"), "the plate's normals lean more than the board's");
  assert.ok(flatness("plastic") > 0.9, "a pebbled skin is nearly flat");
});

test("a finished primitive carries UVs in metres, so a tile keeps its pitch on any box", () => {
  const box = metricGeometry({ kind: "box", size: [1.5, 0.7, 0.03] });
  const uv = box.getAttribute("uv");
  const span = (face) => {
    let umax = 0, vmax = 0;
    for (let i = face * 4; i < face * 4 + 4; i++) {
      umax = Math.max(umax, uv.getX(i));
      vmax = Math.max(vmax, uv.getY(i));
    }
    return [umax, vmax];
  };
  // +x face: u along the depth (z), v along the height (y); +z face: u
  // along the length (x) — a board's grain runs along its length.
  assert.deepEqual(span(0).map((v) => +v.toFixed(6)), [0.03, 0.7]);
  assert.deepEqual(span(4).map((v) => +v.toFixed(6)), [1.5, 0.7]);
  box.computeBoundingBox();
  assert.ok(Math.abs(box.boundingBox.max.x - 0.75) < 1e-9, "the geometry is at the box's own size");
  const cyl = metricGeometry({ kind: "cylinder", radius: 0.1, length: 2 });
  cyl.computeBoundingBox();
  assert.ok(Math.abs(cyl.boundingBox.max.z - 1) < 1e-9, "a cylinder stands along +z like URDF's");
  assert.equal(metricGeometry({ kind: "mesh", url: "", ext: "obj", scale: [1, 1, 1] }), null);
});

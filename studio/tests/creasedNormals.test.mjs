import assert from "node:assert/strict";
import test from "node:test";

import { creasedNormals } from "../src/three/creasedNormals.ts";

const CREASE = Math.PI / 4;
const soup = (...triangles) => new Float32Array(triangles.flat(2));
const normal = (normals, corner) => Array.from(normals.slice(3 * corner, 3 * corner + 3));
const close = (a, b) => a.every((v, i) => Math.abs(v - b[i]) < 1e-6);

test("a flat face has its normal at every corner, however it is cut into triangles", () => {
  // A unit square in the XY plane, cut along one diagonal: counter-clockwise from +Z.
  const normals = creasedNormals(soup([[0, 0, 0], [1, 0, 0], [1, 1, 0]], [[0, 0, 0], [1, 1, 0], [0, 1, 0]]), CREASE);
  for (let corner = 0; corner < 6; corner++) assert.ok(close(normal(normals, corner), [0, 0, 1]));
});

test("faces that meet shallower than the crease share a normal along their edge; steeper ones do not", () => {
  // Two squares hinged on the Y axis: one flat, one tilted up by `fold`.
  const hinged = (fold) => {
    const [c, s] = [Math.cos(fold), Math.sin(fold)];
    return soup(
      [[0, 0, 0], [0, 1, 0], [-1, 1, 0]], [[0, 0, 0], [-1, 1, 0], [-1, 0, 0]],
      [[0, 0, 0], [c, 0, s], [c, 1, s]], [[0, 0, 0], [c, 1, s], [0, 1, 0]],
    );
  };
  // 30 degrees: the corners on the hinge take the bisector, the outer ones their own face's.
  const fold = Math.PI / 6;
  const smooth = creasedNormals(hinged(fold), CREASE);
  const bisector = [-Math.sin(fold / 2), 0, Math.cos(fold / 2)];
  for (const corner of [0, 1, 3, 6, 9, 11]) assert.ok(close(normal(smooth, corner), bisector), `corner ${corner}`);
  assert.ok(close(normal(smooth, 2), [0, 0, 1]));
  assert.ok(close(normal(smooth, 7), [-Math.sin(fold), 0, Math.cos(fold)]));
  // 90 degrees: an edge. Every corner keeps its own face's normal.
  const sharp = creasedNormals(hinged(Math.PI / 2), CREASE);
  for (const corner of [0, 1, 3]) assert.ok(close(normal(sharp, corner), [0, 0, 1]));
  for (const corner of [6, 9, 11]) assert.ok(close(normal(sharp, corner), [-1, 0, 0]));
});

test("a turned surface shades round whichever way its facets are cut", () => {
  // A 12-sided tube around Z, each side a quad cut along one diagonal —
  // alternately the one and the other. At every corner the normal is the
  // radius through it: the two sides it joins weigh the same, although
  // one of them meets it with two triangles and the other with one.
  const sides = 12;
  const at = (k, z) => [Math.cos((2 * Math.PI * (k % sides)) / sides), Math.sin((2 * Math.PI * (k % sides)) / sides), z];
  const triangles = [];
  for (let k = 0; k < sides; k++) {
    const [a, b, c, d] = [at(k, 0), at(k + 1, 0), at(k + 1, 1), at(k, 1)];
    triangles.push(...(k % 2 === 0 ? [[a, b, c], [a, c, d]] : [[a, b, d], [b, c, d]]));
  }
  const positions = soup(...triangles);
  const normals = creasedNormals(positions, CREASE);
  for (let corner = 0; corner < positions.length / 3; corner++) {
    const [x, y] = positions.slice(3 * corner, 3 * corner + 2);
    const radius = Math.hypot(x, y);
    assert.ok(close(normal(normals, corner), [x / radius, y / radius, 0]), `corner ${corner}`);
  }
});

test("corners are the same point only when they are, and a face with no area harms nothing", () => {
  // Two parallel squares a hair apart, facing the same way: not one surface.
  const lifted = creasedNormals(
    soup([[0, 0, 0], [1, 0, 0], [0, 1, 0]], [[0, 0, 1e-4], [0, 1, 1e-4], [1, 0, 1e-4]]), CREASE);
  assert.ok(close(normal(lifted, 0), [0, 0, 1]) && close(normal(lifted, 3), [0, 0, -1]));
  // A sliver with no area beside a real face; minus zero is zero.
  const normals = creasedNormals(
    soup([[0, 0, 0], [1, 0, 0], [2, 0, 0]], [[-0, 0, 0], [1, 0, 0], [0, 1, 0]]), CREASE);
  assert.ok(Array.from(normals).every(Number.isFinite));
  assert.ok(close(normal(normals, 3), [0, 0, 1]));
});

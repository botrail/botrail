import assert from "node:assert/strict";
import { test } from "node:test";
import * as THREE from "three";
import { UNIT_BOX, UNIT_CYLINDER, UNIT_SPHERE } from "../src/three/primitiveGeometry.ts";

test("shared primitives keep authored extents and picking, including resized neighbours", () => {
  for (const [unit, sized, scale, rotate] of [
    [UNIT_BOX, new THREE.BoxGeometry(.04, 2.3, .7), [.04, 2.3, .7], false],
    [UNIT_CYLINDER, new THREE.CylinderGeometry(.12, .12, .7, 32), [.12, .7, .12], true],
    [UNIT_SPHERE, new THREE.SphereGeometry(.32, 32, 24), [.32, .32, .32], false],
  ]) {
    const a = new THREE.Mesh(unit), b = new THREE.Mesh(sized);
    a.scale.set(...scale);
    for (const mesh of [a, b]) {
      if (rotate) mesh.rotation.x = Math.PI / 2;
      mesh.position.set(.2, .3, .4);
      mesh.updateMatrixWorld();
    }
    const boxA = new THREE.Box3().setFromObject(a), boxB = new THREE.Box3().setFromObject(b);
    assert.ok(boxA.min.distanceTo(boxB.min) < 1e-7 && boxA.max.distanceTo(boxB.max) < 1e-7);
    // Avoid the sphere's polar singularity and triangle-edge tie cases.
    const ray = new THREE.Raycaster(new THREE.Vector3(.207, -3, .413), new THREE.Vector3(0, 1, 0));
    const hit = ray.intersectObject(a)[0].distance;
    assert.ok(Math.abs(hit - ray.intersectObject(b)[0].distance) < 1e-7);
    const neighbour = new THREE.Mesh(unit);
    neighbour.scale.set(10, 20, 30);
    neighbour.updateMatrixWorld();
    assert.equal(ray.intersectObject(a)[0].distance, hit);
    sized.dispose();
  }
});

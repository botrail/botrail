import assert from "node:assert/strict";
import { test } from "node:test";
import * as THREE from "three";
import { meshInstance, meshMaterial } from "../src/three/meshAppearance.ts";

const color = new THREE.Color(0.21, 0.43, 0.67);

test("authored glass reveals interiors without changing the shared source or casting an opaque shadow", () => {
  const source = new THREE.Mesh(new THREE.BoxGeometry(), new THREE.MeshStandardMaterial());
  const glass = meshInstance(source, true, {color, forceColor: false,
    material: {metalness: 0, roughness: 0.16, opacity: 0.24}}, true, true);
  assert.equal(glass.object.material.opacity, 0.24);
  assert.equal(glass.object.material.transparent, true);
  assert.equal(glass.object.material.depthWrite, false);
  assert.equal(glass.object.castShadow, false);
  assert.equal(source.material.opacity, 1);
  assert.equal(source.material.depthWrite, true);
  glass.dispose();
});

test("colour/highlight changes preserve PBR channels and are isolated between instances", () => {
  const map = new THREE.Texture();
  const normalMap = new THREE.Texture();
  const roughnessMap = new THREE.Texture();
  const original = new THREE.MeshPhysicalMaterial({
    color, map, normalMap, roughnessMap, roughness: 0.32, metalness: 0.85,
    clearcoat: 0.6, opacity: 0.7, transparent: true, side: THREE.DoubleSide,
  });
  original.normalScale.set(0.3, 0.4);
  const root = new THREE.Group();
  const geometry = new THREE.BoxGeometry();
  const child = new THREE.Group();
  child.add(new THREE.Mesh(geometry, [original, original]));
  root.add(child);
  const red = new THREE.Color("red");
  const highlighted = meshInstance(root, true, {color: red, forceColor: true}, true, true);
  const other = meshInstance(root, true, {color: red, forceColor: false}, true, true);
  const restored = meshInstance(root, true, {color: red, forceColor: false}, true, true);
  const h = highlighted.object.children[0].children[0];
  const m = h.material[0];
  assert.ok(h.castShadow && h.receiveShadow);
  assert.equal(h.geometry, geometry);
  assert.equal(h.material[1], m);
  assert.notEqual(m, original);
  assert.ok(m.color.equals(red));
  assert.ok(original.color.equals(color));
  for (const instance of [other, restored]) {
    const kept = instance.object.children[0].children[0].material[0];
    assert.ok(kept.color.equals(color));
    assert.equal(kept.map, map);
    assert.equal(kept.roughness, 0.32);
  }
  assert.ok(m.isMeshPhysicalMaterial);
  assert.equal(m.map, map);
  assert.equal(m.normalMap, normalMap);
  assert.equal(m.roughnessMap, roughnessMap);
  assert.ok(m.normalScale.equals(original.normalScale));
  assert.equal(m.clearcoat, 0.6);
  assert.equal(m.roughness, 0.32);
  assert.equal(m.metalness, 0.85);
  assert.equal(m.opacity, 0.7);
  assert.equal(m.side, THREE.DoubleSide);
  let disposed = 0;
  m.addEventListener("dispose", () => disposed++);
  for (const resource of [original, geometry, map, normalMap, roughnessMap]) {
    resource.addEventListener("dispose", () => assert.fail("disposed a cached resource"));
  }
  highlighted.dispose();
  assert.equal(disposed, 1);
  other.dispose();
  restored.dispose();
});

test("authored metallic/roughness converts MTL surfaces without dropping shared channels", () => {
  const map = new THREE.Texture();
  const normalMap = new THREE.Texture();
  const source = new THREE.MeshPhongMaterial({
    color, map, normalMap, alphaMap: map, opacity: 0.8, transparent: true,
    side: THREE.DoubleSide, shininess: 80, emissive: 0x123456, emissiveMap: map,
    bumpMap: map, bumpScale: 0.15, lightMap: map, lightMapIntensity: 0.7,
    aoMap: map, aoMapIntensity: 0.4, polygonOffset: true, polygonOffsetFactor: 2,
  });
  source.normalScale.set(0.4, 0.6);
  const result = meshMaterial(source, {color, forceColor: false,
    material: {metalness: 1, roughness: 0.25}});
  assert.ok(result.isMeshStandardMaterial);
  for (const key of ["map", "normalMap", "alphaMap", "emissiveMap", "bumpMap", "bumpScale",
    "lightMap", "lightMapIntensity", "aoMap", "aoMapIntensity", "opacity", "transparent",
    "side", "polygonOffset", "polygonOffsetFactor"]) assert.equal(result[key], source[key], key);
  assert.ok(result.normalScale.equals(source.normalScale));
  assert.ok(result.emissive.equals(source.emissive));
  assert.equal(result.metalness, 1);
  assert.equal(result.roughness, 0.25);
  assert.ok(source.isMeshPhongMaterial);
  assert.equal(source.shininess, 80);
  result.dispose();
});

test("a colour override alone preserves legacy specular maps and shininess", () => {
  const source = new THREE.MeshPhongMaterial({color, shininess: 60,
    specularMap: new THREE.Texture(), map: new THREE.Texture()});
  const result = meshMaterial(source, {color: new THREE.Color("red"), forceColor: true});
  assert.ok(result.isMeshPhongMaterial);
  assert.equal(result.shininess, 60);
  assert.equal(result.specularMap, source.specularMap);
  assert.equal(result.map, source.map);
  assert.ok(source.color.equals(color));
  result.dispose();
});

test("unshaded OBJ leaves use authored PBR values and the proxy's shadow policy", () => {
  const root = new THREE.Group();
  root.add(new THREE.Mesh(new THREE.BoxGeometry(), new THREE.MeshPhongMaterial()));
  root.add(new THREE.Mesh(new THREE.BoxGeometry(), new THREE.MeshPhongMaterial()));
  const instance = meshInstance(root, false, {color, forceColor: false,
    opacity: 0.85, material: {metalness: 0.7, roughness: 0.2}}, false, true);
  const [a, b] = instance.object.children;
  assert.equal(a.material, b.material);
  assert.ok(a.material.color.equals(color));
  assert.equal(a.material.roughness, 0.2);
  assert.equal(a.material.metalness, 0.7);
  assert.equal(a.material.opacity, 0.85);
  assert.ok(a.material.transparent);
  assert.equal(a.castShadow, false);
  assert.equal(a.receiveShadow, true);
  assert.equal(root.children[0].receiveShadow, false);
  instance.dispose();
});

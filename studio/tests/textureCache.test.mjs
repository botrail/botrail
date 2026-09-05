import assert from "node:assert/strict";
import { test } from "node:test";
import * as THREE from "three";
import { sharedTextureProvider } from "../src/three/textureCache.ts";

test("shared image uses retain data interpretation and sampler independence", () => {
  let uploads = 0;
  const provide = sharedTextureProvider((_path, opts) => {
    uploads++;
    const texture = new THREE.Texture();
    texture.colorSpace = opts.colorSpace === "linear" ? THREE.NoColorSpace : THREE.SRGBColorSpace;
    return texture;
  });
  const data = provide("finish.png", {colorSpace: "linear"});
  assert.equal(provide("finish.png", {colorSpace: "linear", channel: 0}), data);
  assert.equal(data.colorSpace, THREE.NoColorSpace);
  assert.notEqual(provide("finish.png", {colorSpace: "srgb"}), data);
  assert.notEqual(provide("finish.png", {colorSpace: "linear", channel: 1}), data);
  const rotated = {colorSpace: "linear", transform: {rotation: 90}};
  assert.notEqual(provide("finish.png", rotated), data);
  assert.equal(provide("finish.png", rotated), provide("finish.png", {transform: {rotation: 90}, colorSpace: "linear"}));
  assert.notEqual(provide("finish.png", {colorSpace: "linear", wrapS: "clamp"}), data);
  assert.equal(uploads, 5);
});

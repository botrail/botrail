import assert from "node:assert/strict";
import {test} from "node:test";
import {AssetPath, DefaultAssetResolver, composeFile, Stage} from "three-usd-robot";
import {anchoredLayer} from "../src/three/usdVisuals.ts";

test("asset paths retain their authoring layer through composition", async () => {
  const resolver = new DefaultAssetResolver();
  const url = "https://example.test/assets/7/layers/looks.usda";
  const bytes = new TextEncoder().encode(`#usda 1.0
def Material "Paint" {
  asset image = @../textures/paint.png@
  asset[] images = [@../textures/normal.png@, @@]
  float roughness = 0.42
}`);
  const file = anchoredLayer(bytes, url, resolver);
  const stage = Stage.OpenFromFile(await composeFile(file, url, resolver));
  const prim = stage.GetPrimAtPath("/Paint");
  assert.equal(prim.GetAttribute("image").Get().path, "https://example.test/assets/7/textures/paint.png");
  assert.deepEqual(prim.GetAttribute("images").Get(), [new AssetPath("https://example.test/assets/7/textures/normal.png"), new AssetPath("")]);
  assert.equal(prim.GetAttribute("roughness").Get(), 0.42);
});

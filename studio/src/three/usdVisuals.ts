import * as THREE from "three";
import { sharedTextureProvider } from "./textureCache.ts";
import {
  Stage, CrateReader, crateToUsdaFile, composeFile, parseUsda, serializeUsda, AssetPath,
  DefaultAssetResolver, openUsdz, isZip, createTextureProvider,
  buildGprimObject, loadMdlModules, type AssetResolver,
} from "three-usd-robot";

const warn = (message: string) => console.warn(`botrail studio: USD appearance: ${message}`);

/** Composition in three-usd-robot 0.13 keeps asset attributes relative to the
 * authoring layer. Anchor them before composition loses that provenance. */
export function anchoredLayer(bytes: Uint8Array, url: string, resolver: AssetResolver) {
  const file = CrateReader.isCrate(bytes)
    ? crateToUsdaFile(new CrateReader(bytes))
    : parseUsda(new TextDecoder().decode(bytes));
  function anchor(value: unknown): unknown {
    if (value instanceof AssetPath) {
      return value.path ? new AssetPath(resolver.resolve(value.path, url)) : value;
    }
    if (Array.isArray(value)) return value.map(anchor);
    if (value && typeof value === "object" && Object.getPrototypeOf(value) === Object.prototype) {
      return Object.fromEntries(Object.entries(value).map(([k, v]) => [k, anchor(v)]));
    }
    return value;
  }
  return anchor(file) as typeof file;
}

/** Use the existing USD composition/material/geometry implementation, without
 * creating a second articulation or rendering unrelated stage geometry. */
async function openVisualStage(url: string) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`${url}: HTTP ${response.status}`);
  let bytes: Uint8Array = new Uint8Array(await response.arrayBuffer());
  let resolver: AssetResolver = new DefaultAssetResolver();
  let baseUrl = url;
  if (isZip(bytes)) {
    const pkg = openUsdz(bytes);
    resolver = pkg.resolver;
    baseUrl = `/${pkg.rootEntry}`;
    bytes = await pkg.resolver.fetchBytes(baseUrl);
  }
  const assets = resolver;
  const fetchLayer = async (path: string) => {
    const raw = assets.fetchBytes ? await assets.fetchBytes(path)
      : new TextEncoder().encode(await assets.fetchText(path));
    return serializeUsda(anchoredLayer(raw, path, assets));
  };
  const layers: AssetResolver = {
    resolve: (path, base) => assets.resolve(path, base),
    fetchText: fetchLayer,
    fetchBytes: async path => new TextEncoder().encode(await fetchLayer(path)),
  };
  const layer = await composeFile(anchoredLayer(bytes, baseUrl, assets), baseUrl, layers, {onWarn: warn});
  const stage = Stage.OpenFromFile(layer);
  return {stage, textureProvider: sharedTextureProvider(createTextureProvider(resolver, baseUrl)),
    mdl: await loadMdlModules(stage, resolver, baseUrl)};
}

const stages = new Map<string, ReturnType<typeof openVisualStage>>();
const visuals = new Map<string, Promise<THREE.Object3D>>();

/** Cache immutable source geometry/materials; placement and overrides belong
 * to the caller's instance. USD prim paths remain distinct from scene names. */
export function loadUsdVisual(url: string, primPath: string): Promise<THREE.Object3D> {
  const key = JSON.stringify([url, primPath]);
  let visual = visuals.get(key);
  if (!visual) {
    let opened = stages.get(url);
    if (!opened) {
      opened = openVisualStage(url);
      stages.set(url, opened);
    }
    visual = opened.then(({stage, textureProvider, mdl}) => {
      const prim = stage.GetPrimAtPath(primPath);
      if (!prim) throw new Error(`${url}: missing prim ${primPath}`);
      const object = buildGprimObject(prim, stage, {textureProvider, mdl, onWarn: warn});
      if (!object) throw new Error(`${url}: unsupported visual ${primPath}`);
      object.userData.primPath = primPath;
      return object;
    });
    visuals.set(key, visual);
  }
  return visual;
}

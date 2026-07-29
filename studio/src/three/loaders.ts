// Mesh loading with a per-URL cache. STL is parsed to a BufferGeometry (we
// apply our own material); OBJ is parsed to an Object3D we clone per instance.
// Unsupported extensions are warned about and skipped.

import * as THREE from "three";
import { OBJLoader, STLLoader } from "three-stdlib";

export type LoadedMesh =
  | { kind: "geometry"; geometry: THREE.BufferGeometry }
  | { kind: "object"; object: THREE.Object3D };

const cache = new Map<string, Promise<LoadedMesh | null>>();

export function loadMesh(
  url: string,
  ext: string
): Promise<LoadedMesh | null> {
  const key = `${ext.toLowerCase()}:${url}`;
  let entry = cache.get(key);
  if (!entry) {
    entry = doLoad(url, ext).catch((err) => {
      console.warn(`botrail studio: failed to load mesh ${url}`, err);
      return null;
    });
    cache.set(key, entry);
  }
  return entry;
}

async function doLoad(url: string, ext: string): Promise<LoadedMesh | null> {
  const e = ext.toLowerCase();
  if (e === "stl") {
    const buffer = await fetchArrayBuffer(url);
    const geometry = new STLLoader().parse(buffer);
    // Some exporters write zero normals, which shade flat black.
    geometry.computeVertexNormals();
    return { kind: "geometry", geometry };
  }
  if (e === "obj") {
    const text = await fetchText(url);
    const object = new OBJLoader().parse(text);
    return { kind: "object", object };
  }
  console.warn(
    `botrail studio: unsupported mesh extension "${ext}" (${url}), skipping`
  );
  return null;
}

async function fetchArrayBuffer(url: string): Promise<ArrayBuffer> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`HTTP ${res.status}`);
  return res.arrayBuffer();
}

async function fetchText(url: string): Promise<string> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`HTTP ${res.status}`);
  return res.text();
}

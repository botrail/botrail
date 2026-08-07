// Mesh loading with a per-URL cache. STL is parsed to a BufferGeometry (we
// apply our own material); OBJ is parsed to an Object3D we clone per instance,
// with its `mtllib` materials when it names one — a catalog arm ships the
// manufacturer's own colors that way, and inventing a palette for it would
// throw away the better answer.
// Unsupported extensions are warned about and skipped.

import * as THREE from "three";
import { MaterialCreator, MTLLoader, OBJLoader, STLLoader } from "three-stdlib";

export type LoadedMesh =
  | { kind: "geometry"; geometry: THREE.BufferGeometry }
  // `shaded` marks an object that brought its own materials: the viewer
  // keeps them instead of stamping a link color over the top.
  | { kind: "object"; object: THREE.Object3D; shaded: boolean };

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
    const loader = new OBJLoader();
    const materials = await loadMaterials(url, text);
    if (materials) loader.setMaterials(materials);
    return { kind: "object", object: loader.parse(text), shaded: !!materials };
  }
  console.warn(
    `botrail studio: unsupported mesh extension "${ext}" (${url}), skipping`
  );
  return null;
}

/** The OBJ's `mtllib`, fetched from beside it and parsed — `null` when the
 * file names none, or when it cannot be read (a mesh whose material file
 * did not travel with it still draws, in the link color). */
async function loadMaterials(
  objUrl: string,
  text: string
): Promise<MaterialCreator | null> {
  const match = /^\s*mtllib\s+(.+?)\s*$/m.exec(text);
  if (!match) return null;
  const name = match[1];
  // `/meshes/{id}` serves the OBJ; `/meshes/{id}/{name}` serves what sits
  // beside it, so a bare file name is all that ever needs resolving.
  const base = `${objUrl.replace(/\/$/, "")}/`;
  try {
    const mtl = await fetchText(base + encodeURIComponent(name));
    const loader = new MTLLoader();
    loader.setResourcePath(base);
    const creator = loader.parse(mtl, base);
    creator.preload();
    return creator;
  } catch (err) {
    console.warn(
      `botrail studio: ${objUrl} names mtllib "${name}" but it could not be loaded`,
      err
    );
    return null;
  }
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

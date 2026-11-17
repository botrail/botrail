// Mesh loading with a per-URL cache. STL is parsed to a BufferGeometry (we
// apply our own material); OBJ is parsed to an Object3D we clone per instance,
// with its `mtllib` materials when it names one — a catalog arm ships the
// manufacturer's own colors that way, and inventing a palette for it would
// throw away the better answer. A material the MTL gives `Pm`/`Pr` (its PBR
// extension) is drawn physically based; the rest stay Phong.
// Unsupported extensions are warned about and skipped.

import * as THREE from "three";
import { MaterialCreator, MTLLoader, OBJLoader, STLLoader } from "three-stdlib";

import { creasedNormals } from "./creasedNormals.ts";
import { adoptMtlPbr } from "./meshAppearance.ts";

export type LoadedMesh =
  | { kind: "geometry"; geometry: THREE.BufferGeometry }
  // `shaded` marks an object that brought its own materials: the viewer
  // keeps them instead of stamping a link color over the top.
  | { kind: "object"; object: THREE.Object3D; shaded: boolean };

const cache = new Map<string, Promise<LoadedMesh | null>>();

/** Faces of an STL that meet at less than this share a normal across the
 * edge between them (rad). */
export const STL_CREASE = Math.PI / 4;
/** An STL denser than this is shaded a normal a face: its facets are too
 * small to see, and it is not worth matching a million corners up. */
const STL_DENSE = 300_000;

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
    // An STL is loose triangles, a normal apiece: shaded as it comes, every
    // facet shows, and a maker's simplified model — a few hundred triangles
    // a link — reads as cut glass. The normals are recomputed from the
    // faces (some exporters write zeros, which shade black) and shared
    // across every edge shallower than the crease angle, so a turned
    // surface shades round and a corner stays a corner.
    const geometry = new STLLoader().parse(buffer);
    const position = geometry.getAttribute("position");
    if (position.count / 3 > STL_DENSE) {
      geometry.computeVertexNormals();
    } else {
      const normals = creasedNormals(position.array as Float32Array, STL_CREASE);
      geometry.setAttribute("normal", new THREE.BufferAttribute(normals, 3));
    }
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
    adoptMtlPbr(creator);
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

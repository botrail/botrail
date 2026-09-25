import * as THREE from "three";

import type { GeometryMsg } from "../protocol";

/** The surface patterns an author can ask for by name
 * (`scene.set_obstacle_material(name, finish=...)`): timber grain, the
 * raised bars of tread plate, the pebble of moulded plastic. Each is a
 * small seamless tile of non-colour data (a normal map, a roughness map)
 * plus a faint tint map that multiplies the authored colour, so the
 * colour stays the author's and the pattern is the studio's. UVs are in
 * metres (see `metricGeometry`), so a tile draws at its real pitch on
 * a 40 mm leg and a 1.5 m top alike. Presentation only: a rollout's
 * pictures and a USD export never see it. */
export type FinishName = "wood" | "checker_plate" | "plastic";

export type FinishMaps = {
  map: THREE.DataTexture;
  normalMap: THREE.DataTexture;
  roughnessMap: THREE.DataTexture;
  /** Tile pitch in metres. */
  pitch: number;
};

const SIZE = 128;

/** Deterministic LCG in [0, 1): the same tile on every machine. */
function noise(seed: number, count: number): Float32Array {
  const out = new Float32Array(count);
  let s = seed >>> 0;
  for (let i = 0; i < count; i++) {
    s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
    out[i] = s / 0x100000000;
  }
  return out;
}

/** Seamless box blur of a periodic field. */
function blur(field: Float32Array, size: number, rx: number, ry: number): Float32Array {
  const out = new Float32Array(size * size);
  const at = (x: number, y: number) => field[((y + size) % size) * size + ((x + size) % size)];
  const n = (2 * rx + 1) * (2 * ry + 1);
  for (let y = 0; y < size; y++) for (let x = 0; x < size; x++) {
    let sum = 0;
    for (let j = -ry; j <= ry; j++) for (let i = -rx; i <= rx; i++) sum += at(x + i, y + j);
    out[y * size + x] = sum / n;
  }
  return out;
}

function normalise(field: Float32Array): Float32Array {
  let lo = Infinity, hi = -Infinity;
  for (const v of field) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
  const span = hi - lo || 1;
  return field.map((v) => (v - lo) / span);
}

/** `height` (0..1, periodic) as a tangent-space normal map of `strength`. */
type Tile = Uint8Array<ArrayBuffer>;

function normalMap(height: Float32Array, size: number, strength: number): Tile {
  const data = new Uint8Array(size * size * 4);
  const at = (x: number, y: number) => height[((y + size) % size) * size + ((x + size) % size)];
  for (let y = 0; y < size; y++) for (let x = 0; x < size; x++) {
    const n = new THREE.Vector3(
      -(at(x + 1, y) - at(x - 1, y)) * strength,
      -(at(x, y + 1) - at(x, y - 1)) * strength,
      1,
    ).normalize();
    data.set([Math.round((n.x + 1) * 127.5), Math.round((n.y + 1) * 127.5), Math.round((n.z + 1) * 127.5), 255], (y * size + x) * 4);
  }
  return data;
}

function grey(field: Float32Array, size: number, lo: number, hi: number): Tile {
  const data = new Uint8Array(size * size * 4);
  for (let i = 0; i < size * size; i++) {
    const v = Math.round(255 * (lo + field[i] * (hi - lo)));
    data.set([v, v, v, 255], i * 4);
  }
  return data;
}

/** Timber: streaks along u — the length of the board — from a field
 * blurred hard along u and lightly across, with the growth rings' banding
 * across v on top. Grain is mostly a tint (a board is not bumpy), with a
 * little relief and a roughness that follows the dark bands. */
function wood(): Recipe {
  const size = SIZE;
  const streak = normalise(blur(noise(4217, size * size), size, 14, 1));
  const rings = new Float32Array(size * size);
  const wobble = normalise(blur(noise(9151, size * size), size, 6, 6));
  for (let y = 0; y < size; y++) for (let x = 0; x < size; x++) {
    const phase = (y / size) * 7 * Math.PI * 2 + wobble[y * size + x] * 2.2;
    rings[y * size + x] = 0.5 + 0.5 * Math.sin(phase);
  }
  const tone = new Float32Array(size * size);
  for (let i = 0; i < tone.length; i++) tone[i] = 0.55 * streak[i] + 0.45 * (1 - rings[i] * rings[i]);
  const map = grey(tone, size, 0.72, 1.0);
  return {
    map,
    normal: normalMap(tone, size, 0.6),
    roughness: grey(tone, size, 0.86, 1.0),
    pitch: 0.4,
  };
}

/** Tread plate: raised bars in a staggered pattern, each cell's bars
 * turned a quarter from its neighbours', bright on the bar and duller in
 * the plate between. Relief is the point here. */
function checkerPlate(): Recipe {
  const size = SIZE;
  const cells = 2;
  const cell = size / cells;
  const height = new Float32Array(size * size);
  // A bar is a rounded slot: length 0.62 and width 0.16 of a cell, and a
  // cell holds two bars side by side, offset so the rows interleave.
  for (let cy = 0; cy < cells; cy++) for (let cx = 0; cx < cells; cx++) {
    const turned = (cx + cy) % 2 === 1;
    for (let k = 0; k < 2; k++) {
      const along = 0.5, across = 0.28 + k * 0.44;
      const [bx, by] = turned ? [across, along] : [along, across];
      for (let y = 0; y < cell; y++) for (let x = 0; x < cell; x++) {
        const u = (x + 0.5) / cell - bx, v = (y + 0.5) / cell - by;
        const [a, b] = turned ? [v, u] : [u, v];
        // Distance to a capsule of half-length 0.23 and radius 0.08.
        const d = Math.hypot(Math.max(Math.abs(a) - 0.23, 0), b) - 0.08;
        const h = THREE.MathUtils.clamp(1 - d / 0.05, 0, 1);
        const i = (cy * cell + y) * size + cx * cell + x;
        height[i] = Math.max(height[i], h);
      }
    }
  }
  const grain = noise(6673, size * size);
  const tone = new Float32Array(size * size);
  for (let i = 0; i < tone.length; i++) tone[i] = 0.75 + 0.25 * height[i] - 0.06 * grain[i];
  return {
    map: grey(normalise(tone), size, 0.84, 1.0),
    normal: normalMap(height, size, 3.0),
    roughness: grey(height, size, 1.0, 0.72),
    pitch: 0.12,
  };
}

/** Moulded plastic: a fine pebbled skin, its roughness following the
 * bumps, and next to no tint. */
function plastic(): Recipe {
  const size = SIZE;
  const pebble = normalise(blur(noise(2731, size * size), size, 1, 1));
  return {
    map: grey(pebble, size, 0.95, 1.0),
    normal: normalMap(pebble, size, 0.9),
    roughness: grey(pebble, size, 0.9, 1.0),
    pitch: 0.05,
  };
}

type Recipe = { map: Tile; normal: Tile; roughness: Tile; pitch: number };

const RECIPES: Record<FinishName, () => Recipe> = {
  wood,
  checker_plate: checkerPlate,
  plastic,
};

export function isFinish(name: string | null | undefined): name is FinishName {
  return name === "wood" || name === "checker_plate" || name === "plastic";
}

const cache = new Map<FinishName, FinishMaps>();

/** The tile for a finish, built once per page and shared by every
 * material that draws it (textures are never disposed: three obstacles
 * or three hundred, it is the same three tiles). */
export function finishMaps(name: FinishName, anisotropy = 4): FinishMaps {
  const cached = cache.get(name);
  if (cached) return cached;
  const recipe = RECIPES[name]();
  const texture = (data: Tile, label: string, colour: boolean) => {
    const t = new THREE.DataTexture(data, SIZE, SIZE, THREE.RGBAFormat);
    t.name = `${name}-${label}`;
    t.colorSpace = colour ? THREE.SRGBColorSpace : THREE.NoColorSpace;
    t.wrapS = t.wrapT = THREE.RepeatWrapping;
    t.repeat.set(1 / recipe.pitch, 1 / recipe.pitch);
    t.magFilter = THREE.LinearFilter;
    t.minFilter = THREE.LinearMipmapLinearFilter;
    t.generateMipmaps = true;
    t.anisotropy = Math.min(4, anisotropy);
    t.needsUpdate = true;
    return t;
  };
  const maps = {
    map: texture(recipe.map, "tint", true),
    normalMap: texture(recipe.normal, "normal", false),
    roughnessMap: texture(recipe.roughness, "roughness", false),
    pitch: recipe.pitch,
  };
  cache.set(name, maps);
  return maps;
}

/** A primitive's geometry at its real size with UVs in metres, so a
 * finish tile repeats at its pitch whatever the box: a box's faces are
 * unwrapped edge for edge (u along the face's first side, so a board's
 * grain runs along its length), a cylinder round its circumference and
 * along its length, a sphere over its meridians. */
export function metricGeometry(geometry: GeometryMsg): THREE.BufferGeometry | null {
  switch (geometry.kind) {
    case "box": {
      const [sx, sy, sz] = geometry.size;
      const box = new THREE.BoxGeometry(sx, sy, sz);
      // BoxGeometry's faces, in order: +x, -x (u along z, v along y);
      // +y, -y (u along x, v along z); +z, -z (u along x, v along y).
      const spans: [number, number][] = [[sz, sy], [sz, sy], [sx, sz], [sx, sz], [sx, sy], [sx, sy]];
      const uv = box.getAttribute("uv");
      for (let i = 0; i < uv.count; i++) {
        const [u, v] = spans[Math.floor(i / 4)];
        uv.setXY(i, uv.getX(i) * u, uv.getY(i) * v);
      }
      return box;
    }
    case "cylinder": {
      // Around +Z like URDF: three's cylinder is along +Y, so turn it.
      const cyl = new THREE.CylinderGeometry(geometry.radius, geometry.radius, geometry.length, 32);
      const uv = cyl.getAttribute("uv");
      const round = 2 * Math.PI * geometry.radius;
      for (let i = 0; i < uv.count; i++) uv.setXY(i, uv.getX(i) * round, uv.getY(i) * geometry.length);
      cyl.rotateX(Math.PI / 2);
      return cyl;
    }
    case "sphere": {
      const sphere = new THREE.SphereGeometry(geometry.radius, 32, 24);
      const uv = sphere.getAttribute("uv");
      const round = 2 * Math.PI * geometry.radius;
      for (let i = 0; i < uv.count; i++) uv.setXY(i, uv.getX(i) * round, uv.getY(i) * round / 2);
      return sphere;
    }
    default:
      return null;
  }
}

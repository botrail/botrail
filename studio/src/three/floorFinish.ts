import * as THREE from "three";

/** A seamless 0.25 m patch of fine resin-floor grain, not damage or a scan.
 * Non-colour data only: leave the authored floor colour alone. The same
 * deterministic field supplies subtle height slopes and roughness; mipmaps
 * remove subpixel grain at a distance. UVs on the floor are in metres. */
export function floorFinish(anisotropy: number) {
  const size = 128;
  const pitch = 0.25;
  const height = new Float32Array(size * size);
  let seed = 739391;
  for (let i = 0; i < height.length; i++) {
    seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
    height[i] = seed / 0x100000000;
  }
  const normal = new Uint8Array(size * size * 4);
  const roughness = new Uint8Array(size * size * 4);
  const at = (x: number, y: number) => height[((y + size) % size) * size + ((x + size) % size)];
  for (let y = 0; y < size; y++) for (let x = 0; x < size; x++) {
    const i = (y * size + x) * 4;
    const n = new THREE.Vector3(
      -(at(x + 1, y) - at(x - 1, y)) * 0.035,
      -(at(x, y + 1) - at(x, y - 1)) * 0.035, 1,
    ).normalize();
    normal.set([Math.round((n.x + 1) * 127.5), Math.round((n.y + 1) * 127.5), Math.round((n.z + 1) * 127.5), 255], i);
    const r = Math.round(255 * (0.91 + at(x, y) * 0.09));
    roughness.set([r, r, r, 255], i);
  }
  const texture = (data: Uint8Array<ArrayBuffer>, name: string) => {
    const t = new THREE.DataTexture(data, size, size, THREE.RGBAFormat);
    t.name = name;
    t.colorSpace = THREE.NoColorSpace;
    t.wrapS = t.wrapT = THREE.RepeatWrapping;
    t.repeat.set(1 / pitch, 1 / pitch);
    t.magFilter = THREE.LinearFilter;
    t.minFilter = THREE.LinearMipmapLinearFilter;
    t.generateMipmaps = true;
    t.anisotropy = Math.min(4, anisotropy);
    t.needsUpdate = true;
    return t;
  };
  return { normalMap: texture(normal, "floor-grain-normal"), roughnessMap: texture(roughness, "floor-grain-roughness") };
}

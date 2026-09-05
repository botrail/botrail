import * as THREE from "three";

import type { MaterialMsg } from "../protocol";

export type MeshAppearance = {
  color: THREE.Color;
  forceColor: boolean;
  material?: MaterialMsg | null;
  /** Used only when the file brought no material. */
  roughness?: number;
  opacity?: number;
};

/** Per-instance material: cached mesh materials and textures stay untouched.
 * Colour overrides keep every other channel, including texture maps. Only an
 * explicit metallic/roughness pair converts a legacy material to PBR. */
export function meshMaterial(
  source: THREE.Material | null,
  { color, forceColor, material, roughness = 0.85, opacity = 1 }: MeshAppearance,
): THREE.Material {
  let result: THREE.Material;
  if (!source) {
    result = new THREE.MeshStandardMaterial({
      color, roughness, metalness: 0.05, opacity, transparent: opacity < 1,
    });
  } else if (material && !(source as THREE.MeshStandardMaterial).isMeshStandardMaterial) {
    // OBJ/MTL uses Phong. Preserve shared surface channels, without deriving
    // a metalness or a measured roughness from its diffuse colour/shininess.
    const pbr = new THREE.MeshStandardMaterial();
    THREE.Material.prototype.copy.call(pbr, source);
    const legacy = source as THREE.MeshPhongMaterial;
    if (legacy.color) pbr.color.copy(legacy.color);
    if (legacy.emissive) pbr.emissive.copy(legacy.emissive);
    if (legacy.normalScale) pbr.normalScale.copy(legacy.normalScale);
    if (legacy.envMapRotation) pbr.envMapRotation.copy(legacy.envMapRotation);
    for (const key of [
      "map", "lightMap", "lightMapIntensity", "aoMap", "aoMapIntensity",
      "emissiveMap", "emissiveIntensity", "bumpMap", "bumpScale", "normalMap",
      "normalMapType", "displacementMap", "displacementScale", "displacementBias",
      "alphaMap", "envMap", "flatShading", "wireframe",
    ] as const) {
      if (key in legacy) Object.assign(pbr, { [key]: legacy[key] });
    }
    result = pbr;
  } else {
    result = source.clone();
  }
  const shaded = result as THREE.MeshStandardMaterial;
  if (forceColor && shaded.color) shaded.color.copy(color);
  if (material) {
    shaded.metalness = material.metalness;
    shaded.roughness = material.roughness;
    if (material.opacity != null) {
      shaded.opacity = material.opacity;
      shaded.transparent = material.opacity < 1 || shaded.transparent;
      if (material.opacity < 1) shaded.depthWrite = false;
    }
  }
  return result;
}

/** Clone the graph and its materials, sharing only the immutable geometry and
 * textures. Shadow flags belong on Mesh leaves, not their parent Group. */
export function meshInstance(
  source: THREE.Object3D,
  shaded: boolean,
  appearance: MeshAppearance,
  castShadow: boolean,
  receiveShadow: boolean,
) {
  const object = source.clone(true);
  const materials = new Map<THREE.Material | null, THREE.Material>();
  const instanceMaterial = (original: THREE.Material) => {
    const key = shaded ? original : null;
    let material = materials.get(key);
    if (!material) {
      material = meshMaterial(key, appearance);
      materials.set(key, material);
    }
    return material;
  };
  object.traverse((child) => {
    const mesh = child as THREE.Mesh;
    if (!mesh.isMesh) return;
    mesh.castShadow = castShadow && (appearance.material?.opacity ?? 1) >= 1;
    mesh.receiveShadow = receiveShadow;
    mesh.material = Array.isArray(mesh.material)
      ? mesh.material.map(instanceMaterial)
      : instanceMaterial(mesh.material);
  });
  return {
    object,
    // Textures and geometry belong to the loader cache, not this instance.
    dispose: () => materials.forEach((material) => material.dispose()),
  };
}

import * as THREE from "three";

// Immutable unit geometry, shared across repeated equipment panels, screws
// and legs. Per-mesh scale keeps every authored dimension and pick target.
// R3F <primitive> attachments do not dispose these module-owned resources.
export const UNIT_BOX = new THREE.BoxGeometry(1, 1, 1);
export const UNIT_CYLINDER = new THREE.CylinderGeometry(1, 1, 1, 32);
export const UNIT_SPHERE = new THREE.SphereGeometry(1, 32, 24);

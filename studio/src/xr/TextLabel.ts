import * as THREE from "three";

/**
 * A canvas-text quad for in-headset labels (after three-usd-robot's
 * `examples/vite-xr-viewer/src/label.ts`). A plain mesh, not a Sprite:
 * three sizes sprites from their world scale but positions them through
 * the view matrix, so under a scaled origin they come out `playerScale`
 * times too large; a quad goes through the same matrices as the scene.
 * It never answers a ray — a label floating over the hand must not
 * shadow the thing the ray points at.
 */
export class TextLabel extends THREE.Mesh<THREE.PlaneGeometry, THREE.MeshBasicMaterial> {
  private readonly canvas: HTMLCanvasElement;
  private readonly ctx: CanvasRenderingContext2D;
  private readonly texture: THREE.CanvasTexture;
  private lastText = "";
  private readonly lineHeight: number;
  private readonly font: number;

  constructor(widthMeters = 0.2, lines = 1, fontPx = 36) {
    const canvas = document.createElement("canvas");
    const lineHeight = Math.round(fontPx * 1.4);
    canvas.width = 512;
    canvas.height = 24 + lineHeight * lines;
    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    texture.anisotropy = 4;
    super(
      new THREE.PlaneGeometry(widthMeters, (widthMeters * canvas.height) / canvas.width),
      new THREE.MeshBasicMaterial({ map: texture, depthTest: false, transparent: true, toneMapped: false }),
    );
    this.canvas = canvas;
    this.texture = texture;
    this.ctx = canvas.getContext("2d")!;
    this.lineHeight = lineHeight;
    this.font = fontPx;
    this.renderOrder = 100;
    this.visible = false;
    this.raycast = () => {};
  }

  /** Turn to face the camera (call once per frame; cheap). */
  billboard(camera: THREE.Camera): void {
    if (!this.visible) return;
    const q = camera.getWorldQuaternion(new THREE.Quaternion());
    const parentQ = this.parent?.getWorldQuaternion(new THREE.Quaternion());
    if (parentQ) q.premultiply(parentQ.invert());
    this.quaternion.copy(q);
  }

  /** Lines separated by `\n`; an empty string hides the label. */
  setText(text: string): void {
    if (text === this.lastText) return;
    this.lastText = text;
    this.visible = text.length > 0;
    if (!this.visible) return;
    const { ctx, canvas } = this;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "rgba(20, 22, 28, 0.86)";
    roundRect(ctx, 4, 4, canvas.width - 8, canvas.height - 8, 16);
    ctx.fill();
    ctx.fillStyle = "#e8eef8";
    ctx.font = `500 ${this.font}px system-ui, sans-serif`;
    ctx.textBaseline = "top";
    text.split("\n").forEach((line, i) => {
      ctx.fillText(line, 20, 14 + i * this.lineHeight, canvas.width - 40);
    });
    this.texture.needsUpdate = true;
  }

  get text(): string {
    return this.lastText;
  }

  dispose(): void {
    this.geometry.dispose();
    this.material.dispose();
    this.texture.dispose();
  }
}

export function roundRect(
  ctx: CanvasRenderingContext2D,
  x: number,
  y: number,
  w: number,
  h: number,
  r: number,
): void {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

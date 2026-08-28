// In-browser wasm backend: the whole botrail core (kinematics, collision,
// planning) runs client-side; no server involved. The wasm package is
// loaded at runtime from `<base>/wasm/` so regular (non-demo) builds don't
// depend on it existing.

import type { BackendHandlers, SessionBackend } from "./backend";
import { useStudioStore } from "./store";

interface WasmSessionLike {
  initial_messages(): string[];
  handle(text: string): string[];
  load_usd_scene(
    bytes: Uint8Array,
    file_name: string,
    prefix?: string | null,
  ): string[];
  load_prepared_scene(json: string): string[];
  addRobotInstance(
    source: string | null,
    name: string | null,
    basePosition: number[],
    baseQuaternion?: number[] | null,
  ): string[];
}

/**
 * The demo cell: NVIDIA's official Franka on the factory floor from
 * `examples/assets/factory.usda`.
 *
 * The robot is fetched straight from NVIDIA's CDN rather than rehosted with
 * the demo — the bucket sends `Access-Control-Allow-Origin: *`, so the
 * browser can read it, and 10 MB stays out of the deploy artifact. wasm gets
 * the layer bytes for kinematics and collision; the studio's USD loader
 * fetches the same stage again for rendering.
 */
const FRANKA_BASE =
  "https://omniverse-content-production.s3-us-west-2.amazonaws.com" +
  "/Assets/Isaac/4.2/Isaac/Robots/Franka";

const FRANKA_LAYERS = [
  "franka.usd",
  "Materials/Materials.usd",
  ...[
    "hand",
    "leftfinger",
    "rightfinger",
    "link0",
    "link1",
    "link2",
    "link3",
    "link4",
    "link5",
    "link6",
    "link7",
  ].map((part) => `Props/panda_${part}.usd`),
];

/** Ready pose, matching `examples/basics/demo.py`. */
const FRANKA_READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.035, 0.035];

const CELL_STAGE = "cell/factory.usda";
// The cell has two pedestals facing each other across the belt; the demo
// stands an arm on each. The second is a copy of the first's model, so the
// 10 MB of asset is fetched and parsed once.
const CELL_MOUNTS: [string, string][] = [
  ["near", "/World/MountFrame"],
  ["far", "/World/MountFrameFar"],
];

async function fetchBytes(url: string): Promise<Uint8Array> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`${res.status} ${res.statusText}: ${url}`);
  return new Uint8Array(await res.arrayBuffer());
}

export class WasmBackend implements SessionBackend {
  private session: WasmSessionLike | null = null;
  private handlers: BackendHandlers | null = null;
  private moduleUrl: string | null = null;

  /**
   * Imports a dropped USD file into the in-browser session. The expensive
   * part (composition + VHACD) runs in a Web Worker; applying the prepared
   * scene on this thread is cheap. Falls back to a synchronous in-place
   * import when the worker path fails.
   */
  async loadUsdScene(bytes: Uint8Array, fileName: string): Promise<boolean> {
    if (!this.session || !this.handlers) return false;
    try {
      const json = await this.decomposeInWorker(bytes, fileName);
      for (const text of this.session.load_prepared_scene(json)) {
        this.handlers.onMessage(text);
      }
      return true;
    } catch (err) {
      console.warn(
        "botrail studio: worker import failed; falling back to main thread",
        err,
      );
    }
    try {
      for (const text of this.session.load_usd_scene(bytes, fileName, null)) {
        this.handlers.onMessage(text);
      }
      return true;
    } catch (err) {
      console.error("botrail studio: USD import failed", err);
      return false;
    }
  }

  private decomposeInWorker(
    bytes: Uint8Array,
    name: string,
  ): Promise<string> {
    return new Promise((resolve, reject) => {
      if (!this.moduleUrl) {
        reject(new Error("wasm module not loaded"));
        return;
      }
      const worker = new Worker(new URL("./usd-worker.ts", import.meta.url), {
        type: "module",
      });
      worker.onmessage = (e) => {
        worker.terminate();
        if (e.data.ok) {
          resolve(e.data.json);
        } else {
          reject(new Error(e.data.error));
        }
      };
      worker.onerror = (e) => {
        worker.terminate();
        reject(new Error(e.message || "worker error"));
      };
      const buffer = bytes.buffer.slice(
        bytes.byteOffset,
        bytes.byteOffset + bytes.byteLength,
      );
      worker.postMessage(
        { moduleUrl: this.moduleUrl, bytes: buffer, name },
        [buffer],
      );
    });
  }

  start(handlers: BackendHandlers): void {
    if (this.handlers) return;
    this.handlers = handlers;
    handlers.onStatus("connecting");
    void this.init(handlers);
  }

  private async init(handlers: BackendHandlers): Promise<void> {
    try {
      // Resolve against the document (not this module's /assets/ URL) so
      // GitHub-Pages-style subpath deployments work.
      const moduleUrl = new URL("wasm/botrail_wasm.js", document.baseURI).href;
      this.moduleUrl = moduleUrl;
      const mod = await import(/* @vite-ignore */ moduleUrl);
      await mod.default(); // fetches botrail_wasm_bg.wasm next to the module
      this.session = await this.startCell(mod);
      handlers.onStatus("connected");
      for (const text of this.session.initial_messages()) {
        handlers.onMessage(text);
      }
      await this.loadCellStage(handlers);
    } catch (err) {
      console.error("botrail studio: failed to start wasm session", err);
      handlers.onStatus("disconnected");
    }
  }

  /**
   * Builds the session on the Franka, falling back to the embedded sample
   * arm if the asset cannot be reached — an offline visitor still gets a
   * working studio rather than a dead page.
   */
  private async startCell(mod: {
    WasmSession: {
      demo(): WasmSessionLike;
      fromUsdRobot(
        names: string[],
        blobs: Uint8Array[],
        root: string,
        articulationRoot?: string | null,
        assetBase?: string | null,
        instanceName?: string | null,
      ): WasmSessionLike;
    };
  }): Promise<WasmSessionLike> {
    try {
      const blobs = await Promise.all(
        FRANKA_LAYERS.map((name) => fetchBytes(`${FRANKA_BASE}/${name}`)),
      );
      return mod.WasmSession.fromUsdRobot(
        FRANKA_LAYERS,
        blobs,
        "franka.usd",
        "/panda",
        FRANKA_BASE,
        CELL_MOUNTS[0][0],
      );
    } catch (err) {
      console.warn(
        "botrail studio: could not load the Franka asset; falling back to the sample arm",
        err,
      );
      return mod.WasmSession.demo();
    }
  }

  /** Imports the factory cell and stands the robot on its mount frame. */
  private async loadCellStage(handlers: BackendHandlers): Promise<void> {
    const url = new URL(CELL_STAGE, document.baseURI).href;
    let bytes: Uint8Array;
    try {
      bytes = await fetchBytes(url);
    } catch (err) {
      console.warn("botrail studio: no demo cell to load", err);
      return;
    }
    if (!(await this.loadUsdScene(bytes, "factory.usda"))) return;

    // The mount frames only exist once the stage is in, so the placement
    // has to follow the import rather than travel with the session.
    const frames = useStudioStore.getState().frames;
    const mounts = CELL_MOUNTS.map(
      ([name, frame]) => [name, frames.find((f) => f.name === frame)] as const,
    ).filter(([, mount]) => mount !== undefined);
    if (mounts.length < 2) {
      // A cell without the far pedestal: leave the single arm where it is.
      this.placeArm(null, mounts[0]?.[1]?.pose);
      return;
    }

    // Stand the far arm up before placing either, so both exist when the
    // poses go out. Facing back across the belt.
    const [[, nearMount], [farName, farMount]] = mounts;
    try {
      for (const text of this.session!.addRobotInstance(
        null,
        farName,
        [...farMount!.pose.position],
        [0, 0, 1, 0],
      )) {
        this.handlers!.onMessage(text);
      }
    } catch (err) {
      console.warn("botrail studio: could not add the second arm", err);
    }
    this.placeArm(null, nearMount!.pose);
    this.placeArm(farName, farMount!.pose);
    handlers.onStatus("connected");
  }

  /** Puts one arm on its pedestal in the ready pose. */
  private placeArm(robot: string | null, pose?: { position: number[] }): void {
    if (!pose) return;
    const scoped = robot === null ? {} : { robot };
    this.send(
      JSON.stringify({ type: "set_robot_base_pose", ...scoped, pose }),
    );
    // `desc.joints` counts fixed joints too; the store already sizes
    // `jointPositions` to the actuated DOF, which is what a pose must match.
    const arm = useStudioStore
      .getState()
      .robots.find((r) => robot === null || r.desc.name === robot);
    if (arm?.jointPositions.length === FRANKA_READY.length) {
      this.send(
        JSON.stringify({
          type: "set_joint_positions",
          ...scoped,
          positions: FRANKA_READY,
        }),
      );
    }
  }

  send(text: string): void {
    const session = this.session;
    const handlers = this.handlers;
    if (!session || !handlers) return;
    const replies = session.handle(text);
    // Deliver asynchronously so the flow matches a real socket.
    queueMicrotask(() => {
      for (const reply of replies) handlers.onMessage(reply);
    });
  }
}

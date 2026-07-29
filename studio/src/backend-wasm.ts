// In-browser wasm backend: the whole botrail core (kinematics, collision,
// planning) runs client-side; no server involved. The wasm package is
// loaded at runtime from `<base>/wasm/` so regular (non-demo) builds don't
// depend on it existing.

import type { BackendHandlers, SessionBackend } from "./backend";

interface WasmSessionLike {
  initial_messages(): string[];
  handle(text: string): string[];
  load_usd_scene(
    bytes: Uint8Array,
    file_name: string,
    prefix?: string | null,
  ): string[];
  load_prepared_scene(json: string): string[];
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
  async loadUsdScene(
    bytes: Uint8Array,
    fileName: string,
  ): Promise<{ ok: boolean; upAxis: "Y" | "Z" }> {
    if (!this.session || !this.handlers) return { ok: false, upAxis: "Y" };
    try {
      const json = await this.decomposeInWorker(bytes, fileName);
      const upAxis = JSON.parse(json).up_axis === "Z" ? "Z" : "Y";
      for (const text of this.session.load_prepared_scene(json)) {
        this.handlers.onMessage(text);
      }
      return { ok: true, upAxis };
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
      return { ok: true, upAxis: "Y" };
    } catch (err) {
      console.error("botrail studio: USD import failed", err);
      return { ok: false, upAxis: "Y" };
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
      this.session = mod.WasmSession.demo() as WasmSessionLike;
      handlers.onStatus("connected");
      for (const text of this.session.initial_messages()) {
        handlers.onMessage(text);
      }
    } catch (err) {
      console.error("botrail studio: failed to start wasm session", err);
      handlers.onStatus("disconnected");
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

// In-browser wasm backend: the whole botrail core (kinematics, collision,
// planning) runs client-side; no server involved. The wasm package is
// loaded at runtime from `<base>/wasm/` so regular (non-demo) builds don't
// depend on it existing.

import type { BackendHandlers, SessionBackend } from "./backend";

interface WasmSessionLike {
  initial_messages(): string[];
  handle(text: string): string[];
}

export class WasmBackend implements SessionBackend {
  private session: WasmSessionLike | null = null;
  private handlers: BackendHandlers | null = null;

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

// Web Worker for USD imports in wasm mode: composition + VHACD convex
// decomposition run in a separate wasm instance here, keeping the UI thread
// responsive. The prepared-scene JSON goes back to the main thread, where
// applying it (convex hulls -> compound) is cheap.

interface DecomposeRequest {
  moduleUrl: string;
  bytes: ArrayBuffer;
  name: string;
}

self.onmessage = async (e: MessageEvent<DecomposeRequest>) => {
  const { moduleUrl, bytes, name } = e.data;
  try {
    const mod = await import(/* @vite-ignore */ moduleUrl);
    await mod.default();
    const json: string = mod.decompose_usd_scene(new Uint8Array(bytes), name);
    self.postMessage({ ok: true as const, json });
  } catch (err) {
    self.postMessage({ ok: false as const, error: String(err) });
  }
};

import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// The Python server (botrail-py) binds 127.0.0.1:8765 by default and serves
// the websocket feed and mesh files. During `vite dev` we proxy those paths to
// it so the studio behaves the same as when statically served by that server.
const BACKEND = "http://127.0.0.1:8765";

export default defineConfig({
  // Relative base so the built assets work when served from any sub-path
  // (the Python package bundles dist/ and serves it via a fallback ServeDir).
  base: "./",
  plugins: [react()],
  server: {
    proxy: {
      "/ws": { target: BACKEND, ws: true },
      "/meshes": { target: BACKEND },
      "/api": { target: BACKEND },
    },
  },
});

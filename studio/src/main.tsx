import { StrictMode } from "react";
import { createRoot } from "react-dom/client";

import { App } from "./App";
import { useStudioStore } from "./store";
import { startWs } from "./ws";
import { probeXr } from "./xr/probe";
import "./xr/automation";
import "./styles.css";

startWs();

// Automation handle: headless drivers (botrail.capture, doc screenshots)
// reach the store through this — e.g. `__STUDIO__.getState().beginCamExport`
// to run the camera video exporter. Read-mostly, harmless to ship.
(window as unknown as { __STUDIO__?: typeof useStudioStore }).__STUDIO__ =
  useStudioStore;

// WebXR is asked about before the Canvas exists: the XR layer's MSAA follows
// the canvas's antialias attribute, which only the headset path turns on
// (`.internal/design-vr.md` §3.2). WebXR needs a secure context (https or
// localhost); elsewhere there is nothing to ask. A missing or slow runtime
// answers "no" within 300 ms, so the desktop studio opens as before.
probeXr(window.isSecureContext ? navigator.xr : undefined).then((supported) => {
  useStudioStore.getState().setXrSupported(supported);
  createRoot(document.getElementById("root")!).render(
    <StrictMode>
      <App />
    </StrictMode>
  );
});

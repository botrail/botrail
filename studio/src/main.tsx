import { StrictMode } from "react";
import { createRoot } from "react-dom/client";

import { App } from "./App";
import { useStudioStore } from "./store";
import { startWs } from "./ws";
import "./styles.css";

startWs();

// Automation handle: headless drivers (botrail.capture, doc screenshots)
// reach the store through this — e.g. `__STUDIO__.getState().beginCamExport`
// to run the camera video exporter. Read-mostly, harmless to ship.
(window as unknown as { __STUDIO__?: typeof useStudioStore }).__STUDIO__ =
  useStudioStore;

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <App />
  </StrictMode>
);

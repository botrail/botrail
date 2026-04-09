import { StrictMode } from "react";
import { createRoot } from "react-dom/client";

import { App } from "./App";
import { startWs } from "./ws";
import "./styles.css";

startWs();

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <App />
  </StrictMode>
);

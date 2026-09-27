import { useEffect, useMemo, useState } from "react";

import { useDockReserve } from "../dockReserve";
import { identityFor, identityText } from "../partIdentity";
import { headlineSpecs } from "../specFormat";
import { robotByName, useStudioStore, type RobotUiState, type Selection } from "../store";
import { IdLine, StateChips, Thumb, relationTitle, subtitleOf, titleOf, useCatalog } from "./PartPanel";

/** `obstacle · conv/belt`, `TCP · tool0` — what the gizmo (or the Layout
 * inspector) is editing. The robot name only disambiguates when several
 * robots share the scene. */
export function selectionLabel(selection: Selection, robots: RobotUiState[]): string {
  const scope = (robot: string) => (robots.length > 1 ? `${robot} · ` : "");
  switch (selection.type) {
    case "obstacle":
      return `obstacle · ${selection.name}`;
    case "group":
      return `group · ${selection.path}`;
    case "sensor":
    case "device":
    case "camera":
    case "lidar":
      return `${selection.type} · ${selection.name}`;
    case "io_node":
      return `I/O node · ${selection.name}`;
    case "line":
      return `${selection.kind === "tool" ? "tool" : "controller"} · ${selection.name}`;
    case "robot":
      return `${scope(selection.robot)}robot base`;
    case "tcp": {
      const robot = robotByName(robots, selection.robot);
      const arm = robot?.selectedGroup;
      return `${scope(selection.robot)}${arm ? `${arm} · ` : ""}TCP · ${robot?.tcpLink ?? "—"}`;
    }
  }
}

/**
 * The card at the viewport's bottom-left that answers "what did I click"
 * without a trip to the sidebar: the selection's line on the bill —
 * name, maker, catalog reference, its headline specs — and `details ▸`
 * to the PART section. Folded, it is the one-line focus chip the studio
 * always had; the shape is remembered. It sits above the timeline dock
 * the same way the overlays do (`--sfc-reserve`), and Esc folds it when
 * no overlay is up to take the key.
 */
export function FocusCard() {
  const connected = useStudioStore((s) => s.connection === "connected");
  const selection = useStudioStore((s) => s.selection);
  const index = useStudioStore((s) => s.bomIndex);
  const mode = useStudioStore((s) => s.focusCard);
  const setFocusCard = useStudioStore((s) => s.setFocusCard);
  const revealPart = useStudioStore((s) => s.revealPart);
  const overlayOpen = useStudioStore((s) => s.sfcOpen || s.ldOpen || s.ioOpen || s.topoOpen);
  const docked = useStudioStore((s) => s.playback !== null);
  const base = useStudioStore((s) => selectionLabel(s.selection, s.robots));
  const [el, setEl] = useState<HTMLDivElement | null>(null);
  useDockReserve(el, docked);
  const identity = useMemo(() => identityFor(selection, index), [selection, index]);
  const hub = useCatalog(identity?.line.catalog ?? null);
  const info = hub?.status === "ok" ? hub.info : null;

  useEffect(() => {
    if (mode !== "card" || !identity) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape" && !overlayOpen) setFocusCard("chip");
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [mode, identity, overlayOpen, setFocusCard]);

  if (!connected) return null;

  if (!identity || mode === "chip") {
    const text = identity ? `${base} · ${identityText(identity.line)}` : base;
    return (
      <div
        ref={setEl}
        className={`focus-chip${identity ? " expandable" : ""}`}
        title={identity ? "open the card" : undefined}
        onClick={identity ? () => setFocusCard("card") : undefined}
      >
        {text}
        {identity && <span className="fc-toggle">▴</span>}
      </div>
    );
  }

  const { line, target, inherited } = identity;
  const specs = headlineSpecs(line, 4);
  const relation = relationTitle(identity);
  const subtitle = inherited
    ? [titleOf(line), subtitleOf(line)].filter(Boolean).join(" · ")
    : subtitleOf(line);
  return (
    <div ref={setEl} className="focus-card">
      <div className="fc-head">
        <Thumb info={info} category={line.category} />
        <div className="part-head-text">
          <div className="fc-title">
            {relation ?? titleOf(line)}
            {subtitle && <span className="muted"> · {subtitle}</span>}
          </div>
          <IdLine line={line} />
        </div>
        <button className="fc-fold" title="fold to the chip (Esc)" onClick={() => setFocusCard("chip")}>
          ▾
        </button>
      </div>
      {specs.length > 0 && (
        <div className="fc-specs">{specs.map((s) => `${s.label} ${s.value}`).join(" · ")}</div>
      )}
      <div className="fc-foot">
        <span className="fc-chips">
          <StateChips line={line} target={target} info={info} />
          <span className="muted">{base}</span>
        </span>
        <a
          href="#"
          onClick={(e) => {
            e.preventDefault();
            revealPart();
          }}
        >
          details ▸
        </a>
      </div>
    </div>
  );
}

import type { OverlayKind } from "../overlays";
import { useStudioStore } from "../store";

export type { OverlayKind } from "../overlays";

const TABS: { kind: OverlayKind; label: string; title: string }[] = [
  { kind: "sfc", label: "SFC", title: "the programs as an SFC chart" },
  { kind: "ld", label: "LD", title: "the programs as a SET/RST step ladder" },
  { kind: "io", label: "I/O", title: "the I/O table — points, channels, findings, live levels" },
  { kind: "topo", label: "TOPOLOGY", title: "the electrical topology — controllers, channels, wires, handshakes" },
  { kind: "bom", label: "BOM", title: "the bill of materials — every line, identified or not, with what it stands for" },
  { kind: "routing", label: "ROUTING", title: "the cable list — every route with its line, ends and lengths, and the route checks" },
  { kind: "nc", label: "NC", title: "a machine tool's control — the part program at the line it runs, the offsets, the tools" },
];

/**
 * The tab strip of the analysis panel over the viewport. The SFC chart,
 * the I/O table and the topology are one panel with three views — they
 * are all wide, and stacked they hid each other — so switching is a tab
 * click here, and the ◫ / ⚡ / ⌗ buttons elsewhere open their view.
 */
export function OverlayTabs({ active }: { active: OverlayKind }) {
  const setSfcOpen = useStudioStore((s) => s.setSfcOpen);
  const setLdOpen = useStudioStore((s) => s.setLdOpen);
  const setIoOpen = useStudioStore((s) => s.setIoOpen);
  const setTopoOpen = useStudioStore((s) => s.setTopoOpen);
  const setBomOpen = useStudioStore((s) => s.setBomOpen);
  const setRoutingOpen = useStudioStore((s) => s.setRoutingOpen);
  const setNcOpen = useStudioStore((s) => s.setNcOpen);
  // A cell with no machine tool has no control to show.
  const machines = useStudioStore((s) => s.ncMachines.length);
  const open = (kind: OverlayKind) => {
    if (kind === "sfc") setSfcOpen(true);
    else if (kind === "ld") setLdOpen(true);
    else if (kind === "io") setIoOpen(true);
    else if (kind === "topo") setTopoOpen(true);
    else if (kind === "routing") setRoutingOpen(true);
    else if (kind === "nc") setNcOpen(true);
    else setBomOpen(true);
  };
  return (
    <span className="overlay-tabs">
      {TABS.filter((t) => t.kind !== "nc" || machines > 0 || active === "nc").map((t) => (
        <button
          key={t.kind}
          className={`overlay-tab${t.kind === active ? " active" : ""}`}
          onClick={() => {
            if (t.kind !== active) open(t.kind);
          }}
          title={t.title}
        >
          {t.label}
        </button>
      ))}
    </span>
  );
}

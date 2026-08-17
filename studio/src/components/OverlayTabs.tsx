import { useStudioStore } from "../store";

export type OverlayKind = "sfc" | "io" | "topo";

const TABS: { kind: OverlayKind; label: string; title: string }[] = [
  { kind: "sfc", label: "SFC", title: "the programs as an SFC chart" },
  { kind: "io", label: "I/O", title: "the I/O table — points, channels, findings, live levels" },
  { kind: "topo", label: "TOPOLOGY", title: "the electrical topology — controllers, channels, wires, handshakes" },
];

/**
 * The tab strip of the analysis panel over the viewport. The SFC chart,
 * the I/O table and the topology are one panel with three views — they
 * are all wide, and stacked they hid each other — so switching is a tab
 * click here, and the ◫ / ⚡ / ⌗ buttons elsewhere open their view.
 */
export function OverlayTabs({ active }: { active: OverlayKind }) {
  const setSfcOpen = useStudioStore((s) => s.setSfcOpen);
  const setIoOpen = useStudioStore((s) => s.setIoOpen);
  const setTopoOpen = useStudioStore((s) => s.setTopoOpen);
  const open = (kind: OverlayKind) => {
    if (kind === "sfc") setSfcOpen(true);
    else if (kind === "io") setIoOpen(true);
    else setTopoOpen(true);
  };
  return (
    <span className="overlay-tabs">
      {TABS.map((t) => (
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

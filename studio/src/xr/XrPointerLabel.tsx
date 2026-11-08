import { useEffect, useMemo } from "react";
import { useFrame } from "@react-three/fiber";

import { selectionLabel } from "../components/FocusCard";
import { hoverLabel } from "../hoverChip";
import { identityOf, identityText } from "../partIdentity";
import type { PartTargetKind } from "../protocol";
import { type Selection, type StudioState, robotByName, useStudioStore } from "../store";
import { robotRig } from "../three/robotRig";
import { TextLabel } from "./TextLabel";

/**
 * The label over the right controller (`.internal/design-vr.md` §3.7):
 * what the ray is on (the hover chip's words — kind, name, product) and
 * what is selected (the focus card's line, with the product). The same
 * store fields the desktop chips read, so pointing and selecting in the
 * headset reads the same as on the desktop. Faces the head.
 */
export function XrPointerLabel() {
  const label = useMemo(() => new TextLabel(0.24, 3, 34), []);
  useEffect(() => () => label.dispose(), [label]);
  useFrame(({ camera }) => {
    label.setText(pointerLabelText(useStudioStore.getState()));
    label.billboard(camera);
  });
  return <primitive object={label} position={[0, 0.06, 0.02]} />;
}

type LabelState = Pick<StudioState, "hover" | "selection" | "robots" | "bomIndex">;

/** The label's lines: the hover (when there is one), the selection, and
 * for a focused TCP the solver's verdict on its last target. */
export function pointerLabelText(s: LabelState): string {
  const lines: string[] = [];
  if (s.hover) {
    const id = identityOf(s.bomIndex, s.hover.kind, s.hover.name);
    lines.push(hoverLabel(s.hover, id ? identityText(id.line) : null));
  }
  const target = partTargetOf(s.selection);
  const id = target ? identityOf(s.bomIndex, target.kind, target.name) : null;
  lines.push(`▸ ${selectionLabel(s.selection, s.robots)}${id ? ` · ${identityText(id.line)}` : ""}`);
  const ik = ikLine(s);
  if (ik) lines.push(ik);
  return lines.join("\n");
}

/** `tcp in hand · reachable`, `tcp · unreachable · 12 mm`, or nothing
 * before the first solve. */
export function ikLine(s: Pick<StudioState, "selection" | "robots">): string | null {
  if (s.selection.type !== "tcp") return null;
  const status = robotByName(s.robots, s.selection.robot)?.ikStatus ?? null;
  if (!status) return null;
  const held = robotRig.tcpHandle?.grabbed ? "tcp in hand" : "tcp";
  if (status.converged) return `${held} · reachable`;
  return `${held} · unreachable · ${Math.round(status.pos_error * 1000)} mm`;
}

/** The bill-of-materials target a selection stands for, if any. */
function partTargetOf(selection: Selection): { kind: PartTargetKind; name: string } | null {
  switch (selection.type) {
    case "tcp":
    case "robot":
      return selection.robot ? { kind: "robot", name: selection.robot } : null;
    case "obstacle":
      return { kind: "obstacle", name: selection.name };
    case "group":
      return { kind: "group", name: selection.path };
    case "sensor":
    case "device":
    case "camera":
    case "lidar":
    case "io_node":
      return { kind: selection.type, name: selection.name };
    case "line":
      return { kind: selection.kind, name: selection.name };
    case "route_via":
      return { kind: "route", name: selection.route };
  }
}

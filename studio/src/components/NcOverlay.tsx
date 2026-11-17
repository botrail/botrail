import { useEffect, useMemo, useRef, useState } from "react";

import { useDockReserve } from "../dockReserve";
import { machineFunctions, mm, ncStatusAt, toolLabel, type NcStatus } from "../ncPanel";
import { sampleJoints } from "../playback";
import type { NcMachineMsg, NcProgramMsg } from "../protocol";
import { useStudioStore } from "../store";
import { OverlayTabs } from "./OverlayTabs";

/**
 * A machine tool's control over the viewport: the part program with the
 * line it is running — or the machine function it stands at, waiting for
 * the cell — the position page (machine coordinates, the tool in the
 * spindle, speed, coolant), and the two tables an operator fills in at
 * setup with the machine's M-code table beside them.
 *
 * Everything is the host's: the tables and the text arrive as the `nc`
 * message, what the control did over a cycle as the lanes the bake wrote
 * (`ncPanel.ts`). Without a baked cycle the panel is the setup as it
 * stands — the listing, the tables, the axes where they are.
 */
export function NcOverlay() {
  const open = useStudioStore((s) => s.ncOpen);
  const setOpen = useStudioStore((s) => s.setNcOpen);
  const machines = useStudioStore((s) => s.ncMachines);
  const programs = useStudioStore((s) => s.ncPrograms);
  const docked = useStudioStore((s) => s.playback !== null);
  const [panel, setPanel] = useState<HTMLDivElement | null>(null);
  useDockReserve(panel, docked);
  const [chosen, setChosen] = useState<string | null>(null);
  if (!open) return null;
  const machine = machines.find((m) => m.robot === chosen) ?? machines[0] ?? null;
  return (
    <div className="sfc-overlay io-overlay nc-overlay" ref={setPanel}>
      <div className="sfc-head">
        <OverlayTabs active="nc" />
        {machines.length > 1 ? (
          <select
            className="nc-machine"
            value={machine?.robot ?? ""}
            onChange={(e) => setChosen(e.target.value)}
            title="which machine's control"
          >
            {machines.map((m) => (
              <option key={m.robot} value={m.robot}>
                {m.robot}
              </option>
            ))}
          </select>
        ) : (
          <span className="sfc-caption">{machine?.robot ?? ""}</span>
        )}
        {machines.length > 1 && <span className="sfc-caption" />}
        <button
          className="timeline-button"
          onClick={() => setOpen(false)}
          title="close (the nc button on the timeline reopens it)"
        >
          ×
        </button>
      </div>
      {machine === null ? (
        <div className="sfc-empty hint">
          no machine tool in this cell — bt.parts.machine_tool(axes=…) declares one
        </div>
      ) : (
        <Control machine={machine} programs={programs} />
      )}
    </div>
  );
}

function Control({ machine, programs }: { machine: NcMachineMsg; programs: NcProgramMsg[] }) {
  const timeline = useStudioStore((s) => s.timeline);
  const playback = useStudioStore((s) => s.playback);
  const t = useStudioStore((s) => s.playbackTime);
  const live = useStudioStore(
    (s) => s.robots.find((r) => r.desc.name === machine.robot)?.jointPositions ?? null,
  );
  const status: NcStatus | null = useMemo(
    () => (timeline ? ncStatusAt(timeline, machine, programs, t) : null),
    [timeline, machine, programs, t],
  );
  // The axes: off the baked track while a cycle is loaded, else where the
  // scene has them.
  const joints = useMemo(() => {
    const track = playback?.robots.find((r) => r.name === machine.robot)?.trajectory;
    if (track && track.joint_positions.length > 0) return sampleJoints(track, t);
    return live;
  }, [playback, machine.robot, t, live]);
  // The listing follows the control; a program picked by hand stays until
  // the control moves on to another.
  const [picked, setPicked] = useState<string | null>(null);
  const running = status?.program ?? null;
  useEffect(() => setPicked(null), [running]);
  const shown =
    programs.find((p) => p.name === (picked ?? running)) ?? programs[0] ?? null;
  const at = shown && shown.name === running ? status : null;
  return (
    <div className="sfc-scroll nc-body">
      <div className="nc-left">
        <Position machine={machine} joints={joints} status={status} />
        <div className="nc-programs">
          {programs.map((p) => (
            <button
              key={p.name}
              className={`overlay-tab${p.name === shown?.name ? " active" : ""}`}
              onClick={() => setPicked(p.name)}
              title={p.name === running ? "the program the control is in" : undefined}
            >
              {p.name}
              {p.name === running ? " ▸" : ""}
            </button>
          ))}
          {programs.length === 0 && <span className="hint">no part program — bt.nc.program(…)</span>}
        </div>
        {shown && <Listing program={shown} machine={machine} status={at} />}
      </div>
      <Tables machine={machine} status={status} />
    </div>
  );
}

/** The position page: where the axes are and what the spindle is doing. */
function Position({
  machine,
  joints,
  status,
}: {
  machine: NcMachineMsg;
  joints: number[] | null;
  status: NcStatus | null;
}) {
  const standing = status?.standing ?? null;
  return (
    <div className="nc-position">
      <div className="nc-axes" title="machine coordinates, mm">
        {["X", "Y", "Z"].map((axis, i) => (
          <span key={axis} className="nc-axis">
            <b>{axis}</b>
            {joints && joints[machine.axes[i]] !== undefined ? mm(joints[machine.axes[i]]) : "—"}
          </span>
        ))}
      </div>
      {status && (
        <div className="nc-state">
          <span title="the tool in the spindle">T{status.tool || "—"}</span>
          <span className={status.spindle ? "nc-on" : undefined} title="spindle speed, rev/min">
            S{status.rpm}
          </span>
          <span className={status.coolant ? "nc-on" : undefined} title="coolant">
            {status.coolant ? "coolant on" : "coolant off"}
          </span>
          <span
            className={standing?.alarm ? "nc-alarm" : status.running ? "nc-on" : undefined}
            title={standing ? "the step of the machine's program that answers this line" : undefined}
          >
            {standing
              ? standing.alarm
                ? standing.step
                : `waits — ${standing.step}`
              : status.running
                ? `${status.program}${status.block != null ? ` · block ${status.block}` : ""}`
                : status.program
                  ? "ended"
                  : "not started"}
          </span>
        </div>
      )}
    </div>
  );
}

/** The program's text, the line the control is at kept in view. */
function Listing({
  program,
  machine,
  status,
}: {
  program: NcProgramMsg;
  machine: NcMachineMsg;
  status: NcStatus | null;
}) {
  const lines = useMemo(() => program.text.split("\n"), [program.text]);
  const calls = useMemo(
    () => lines.map((line) => machineFunctions(line, machine.m_codes)),
    [lines, machine.m_codes],
  );
  const run = status?.line ?? null;
  const stand = status?.standing ?? null;
  const current = run ?? stand?.line ?? null;
  const row = useRef<HTMLDivElement | null>(null);
  useEffect(() => {
    row.current?.scrollIntoView({ block: "nearest" });
  }, [current, program.name]);
  return (
    <div className="nc-listing">
      {lines.map((text, i) => {
        const n = i + 1;
        const mark =
          n === run ? " nc-run" : n === stand?.line ? (stand.alarm ? " nc-stop" : " nc-wait") : "";
        return (
          <div key={n} className={`nc-line${mark}`} ref={n === current ? row : undefined}>
            <span className="nc-no">{n}</span>
            <span className="nc-text">{text || " "}</span>
            {calls[i].length > 0 && (
              <span className="nc-call" title="a machine function: the control waits here for the machine">
                {calls[i].map(([, what]) => what).join(", ")}
              </span>
            )}
          </div>
        );
      })}
    </div>
  );
}

/** The work offsets, the tool table and the machine's M codes. */
function Tables({ machine, status }: { machine: NcMachineMsg; status: NcStatus | null }) {
  return (
    <div className="nc-tables">
      <table className="io-table">
        <thead>
          <tr>
            <th>offset</th>
            <th>X</th>
            <th>Y</th>
            <th>Z [mm]</th>
          </tr>
        </thead>
        <tbody>
          {machine.work_offsets.map((o) => (
            <tr key={o.name}>
              <td>{o.name}</td>
              {o.xyz.map((v, i) => (
                <td key={i} className="nc-num">
                  {mm(v)}
                </td>
              ))}
            </tr>
          ))}
          {machine.work_offsets.length === 0 && (
            <tr>
              <td colSpan={4} className="hint">
                none entered
              </td>
            </tr>
          )}
        </tbody>
      </table>
      <table className="io-table">
        <thead>
          <tr>
            <th>T</th>
            <th>tool</th>
            <th>H [mm]</th>
            <th />
          </tr>
        </thead>
        <tbody>
          {machine.tools.map((tool) => {
            // The entry is what the axes are positioned by; the body is
            // where the tip is. A difference is a setup that was measured
            // wrong.
            const off = tool.body_length != null ? tool.length - tool.body_length : 0;
            return (
              <tr key={tool.number} className={status?.tool === tool.number ? "nc-in-spindle" : undefined}>
                <td>T{tool.number}</td>
                <td>{toolLabel(tool)}</td>
                <td className="nc-num">{mm(tool.length)}</td>
                <td className={Math.abs(off) > 1e-9 ? "nc-alarm" : "hint"}>
                  {tool.body_length == null
                    ? "no body"
                    : Math.abs(off) > 1e-9
                      ? `${off > 0 ? "+" : ""}${mm(off)} from the tool`
                      : ""}
                </td>
              </tr>
            );
          })}
          {machine.tools.length === 0 && (
            <tr>
              <td colSpan={4} className="hint">
                none entered
              </td>
            </tr>
          )}
        </tbody>
      </table>
      {machine.m_codes.length > 0 && (
        <table className="io-table">
          <thead>
            <tr>
              <th>M</th>
              <th>the machine's</th>
            </tr>
          </thead>
          <tbody>
            {machine.m_codes.map((m) => (
              <tr key={m.code}>
                <td>M{m.code}</td>
                <td>{m.what}</td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
    </div>
  );
}

"""NC part programs on a machine tool's axes.

A machining centre built with its axes (`bt.parts.machine_tool(axes=)`)
is a mechanism whose joints are the machine coordinates, declared as an
**NC machine**: a control with travels and rates, and the two tables an
operator fills in at setup — work offsets and tools. A part program — the
text a CAM post writes — is read against those the way the control would
run it, and what the axes do and how long it takes come out of the
program, not out of a typed-in cycle time:

    vf2 = bt.parts.machine_tool(scene, "vf2", catalog="haas/vf/vf-2", axes=True)
    bt.nc.set_work_offset(scene, vf2, "G54", at=part_top)   # derived from the cell
    bt.nc.set_tool(scene, vf2, 1, length=0.110, diameter=0.010)
    bt.nc.program(scene, "op10", Path("op10.nc"))
    print(bt.nc.check(scene, vf2, "op10").summary())        # a dry run
    sq.step("machining", actions=[bt.seq.run_program(vf2.axes, "op10")])

A tool entered as a **body** (`end_mill`, `drill` … in a `Holder`) cuts:
the dry run, and a baked cycle, carve the stock that rides the table and
say what went wrong on which line — a cutter in the material at rapid, a
holder in the part, a tool in the vise — and, against the design model,
what is left over and what is missing:

    bt.nc.set_tool(scene, vf2, 1, bt.nc.end_mill(0.010, flutes=0.022, stickout=0.035,
                                                 holder=bt.nc.Holder(0.040, 0.075)))
    run = bt.nc.check(scene, vf2, "op10", stock="part", fixtures=vise.obstacles,
                      design="part.stl")
    print(run.machining.summary())
    tl, cut = bt.nc.animate(scene, tl, vf2, "part", design="part.stl")   # … and the picture
    Path("setup.md").write_text(bt.nc.setup_sheet(scene, vf2, "op10", stock="part"))

The table's length is what the axes are positioned by; the body's is
where the tip is. They agree — the entry is taken from the body — unless
you enter another, which is how a setup that was measured wrong is tried.

A program that **runs the cell** — opens the window, clamps the vise,
calls the robot through a relay and waits for its M-FIN, and starts over
on `M99` — does it with M codes that are the machine's to carry out, not
the control's. They are the machine's **M-code table** (`MCode`), and
`sequence` writes the machine's own program from the part program: its
blocks in order, and at every such code the step that answers it.

    hs = bt.tending.haas_mfin(scene, vf2, "o1000", parts=["part", "blank"])   # the Haas table

What the control cannot read is refused with the line number rather than
skipped — a word it does not know, a work offset or a tool nobody
entered, a point past an axis's travel, an M code that is in nobody's
table — at `check`, and again when the step fires in a bake.

What is read (the Haas dialect a CAM post writes for 3-axis milling):
G0/G1, G2/G3 (IJK or R, helical), G4, G17–G19, G20/G21, G28, G40,
G43/G49, G53, G54–G59 and G154 P, G73/G81/G82/G83 with G80 and G98/G99,
G90/G91, G94; M1, M2/M30, M3/M4/M5, M6, M8/M9, M99 (the program starts
over), and the M codes of the machine's table. Not read: cutter radius
compensation (post the path compensated), macros, subprograms, probing.

Timing is a control's look-ahead in outline: a rapid runs straight from
rest to rest at the axes' rapid rates; a run of cutting moves is one
continuous path at the programmed feed, slowed for corners and bounded
by the axes' accelerations. It is an estimate of the cycle, not a servo
model.

The cut is mesh booleans (cutrail): each tool's sweep subtracted from the
stock in the stock's own frame, exact to the chord tolerance. A drill is
cut as a flat end of its diameter — wall and depth, not the cone.
"""

from __future__ import annotations

import json
import re
from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional, Union

from . import seq
from .parts import MachineTool, _rotate

Point3 = tuple[float, float, float]


# ----------------------------------------------------------------- tools
@dataclass(frozen=True)
class Holder:
    """A tool holder, from the gauge line down: its `diameter` and how
    far it reaches (`length`, m) — to the nose the cutter sticks out of."""

    diameter: float
    length: float


@dataclass(frozen=True)
class Tool:
    """A tool as it stands in the spindle (m). `length` is gauge line to
    tip — the figure a presetter reads, and what the tool table's entry
    is taken from. Build one with `end_mill`, `ball_mill`, `bull_nose`
    or `drill`."""

    cutter: str
    diameter: float
    flute_length: float
    length: float
    corner_radius: Optional[float] = None
    shank_diameter: Optional[float] = None
    holder: Optional[tuple[float, float]] = None

    def describe(self) -> str:
        """`10 mm flat end mill`, the way a setup sheet names it."""
        kind = {"flat": "flat end mill", "ball": "ball end mill",
                "bull_nose": "bull-nose end mill", "drill": "drill"}[self.cutter]
        corner = f" R{self.corner_radius * 1e3:g}" if self.corner_radius else ""
        return f"{self.diameter * 1e3:g} mm {kind}{corner}"


def _tool(cutter: str, diameter: float, flutes: float, stickout: Optional[float],
          holder: Optional[Holder], length: Optional[float], shank: Optional[float],
          corner_radius: Optional[float] = None) -> Tool:
    if holder is not None:
        if stickout is None:
            raise ValueError("bt.nc: a tool in a holder needs its `stickout` — how far it stands out of the holder's nose")
        if length is not None:
            raise ValueError("bt.nc: give a holder and a stickout, or the tool's `length` — not both")
        length = holder.length + stickout
    elif length is None:
        raise ValueError("bt.nc: give the tool's `length` (gauge line to tip), or its `holder` and `stickout`")
    reach = length - (holder.length if holder else 0.0)
    if flutes > reach + 1e-12:
        raise ValueError(
            f"bt.nc: {flutes * 1e3:g} mm of flutes do not fit the {reach * 1e3:g} mm the tool stands out"
        )
    return Tool(cutter=cutter, diameter=float(diameter), flute_length=float(flutes), length=float(length),
                corner_radius=corner_radius, shank_diameter=None if shank is None else float(shank),
                holder=None if holder is None else (float(holder.diameter), float(holder.length)))


def end_mill(diameter: float, *, flutes: float, stickout: Optional[float] = None,
             holder: Optional[Holder] = None, length: Optional[float] = None,
             shank: Optional[float] = None) -> Tool:
    """A flat end mill `diameter` across, cutting `flutes` up from its
    tip. Its length is derived from how it is held — `holder` and the
    `stickout` out of the holder's nose — or given as `length` (gauge line
    to tip) for a tool with no holder to collide. `shank` is the shank's
    diameter where it differs from the cutter's. Metres."""
    return _tool("flat", diameter, flutes, stickout, holder, length, shank)


def ball_mill(diameter: float, *, flutes: float, stickout: Optional[float] = None,
              holder: Optional[Holder] = None, length: Optional[float] = None,
              shank: Optional[float] = None) -> Tool:
    """A ball end mill: `end_mill` with a hemispherical end."""
    return _tool("ball", diameter, flutes, stickout, holder, length, shank)


def bull_nose(diameter: float, corner_radius: float, *, flutes: float, stickout: Optional[float] = None,
              holder: Optional[Holder] = None, length: Optional[float] = None,
              shank: Optional[float] = None) -> Tool:
    """A bull-nose end mill: `end_mill` with `corner_radius` on its end."""
    return _tool("bull_nose", diameter, flutes, stickout, holder, length, shank, float(corner_radius))


def drill(diameter: float, *, flutes: float, stickout: Optional[float] = None,
          holder: Optional[Holder] = None, length: Optional[float] = None) -> Tool:
    """A twist drill. It is cut as a flat end of its diameter: the hole's
    wall and its programmed depth are right, the cone at the bottom is not
    drawn — measure a drilled depth to the drill's corner, not its point."""
    return _tool("drill", diameter, flutes, stickout, holder, length, None)


# -------------------------------------------------------------- what was cut
#: What a tool met that it should not have, the way a report says it.
_ISSUES = {
    "rapid_into_stock": "the cutter meets {what} at rapid",
    "spindle_stopped": "the cutter meets {what} with the spindle stopped",
    "holder_into_stock": "the shank or holder meets {what}",
    "into_fixture": "the tool meets {what}",
}


@dataclass(frozen=True)
class Issue:
    """One thing a tool met that it should not have, on the source line
    it happened on. `kind` is `"rapid_into_stock"`, `"spindle_stopped"`,
    `"holder_into_stock"` or `"into_fixture"`; `what` the stock or the
    fixture's obstacle; `volume` the material in the way (m³); `time` the
    first contact on the cycle's clock (s)."""

    kind: str
    program: str
    line: int
    tool: int
    time: float
    volume: float
    what: str

    def __str__(self) -> str:
        said = _ISSUES[self.kind].format(what=f"`{self.what}`")
        return (f"{self.program} line {self.line}: {said} "
                f"(T{self.tool}, {self.volume * 1e9:.1f} mm³, at {self.time:.2f} s)")


@dataclass(frozen=True)
class Machining:
    """What a machine's programs cut out of one stock — `check(stock=)`,
    `cut` and `animate` return it. Volumes are m³."""

    machine: str
    stock: str
    #: The machined part: `part.save_obj(path)` / `save_stl(path)` write
    #: its mesh, in the stock's own frame.
    part: object
    initial_volume: float
    removed_volume: float
    remaining_volume: float
    issues: tuple[Issue, ...]
    #: `(program, line, tool, volume)`: what each source line removed, in
    #: the order the lines first cut.
    lines: tuple[tuple[str, int, int, float], ...]
    #: The first and the last cut on the cycle's clock (s), `None` when
    #: nothing cut.
    cut_span: Optional[tuple[float, float]]
    #: `(tool, from, to)`: what stood in the spindle when.
    tools: tuple[tuple[int, float, float], ...]
    #: `(tool, from, to)`: every tool change the axes stood for, and the
    #: tool it ended with — the stretch a machine's tool changer works in.
    changes: tuple[tuple[int, float, float], ...]
    #: Tools the programs called that have no body in the table — their
    #: moves cut nothing here (`set_tool(scene, machine, n, tool)`).
    unbodied: tuple[int, ...]
    #: Against the design model, where one was given: the tolerance (m),
    #: the material left outside the design and the material missing
    #: inside it, and the box of each in the stock's frame.
    tolerance: Optional[float] = None
    excess_volume: Optional[float] = None
    gouge_volume: Optional[float] = None
    excess_bounds: Optional[tuple[Point3, Point3]] = None
    gouge_bounds: Optional[tuple[Point3, Point3]] = None

    @property
    def ok(self) -> bool:
        """Nothing met anything it should not have, every tool that was
        called is one that cuts, and — where a design was given — the
        part is the design within the tolerance."""
        return (not self.issues and not self.unbodied
                and not self.excess_volume and not self.gouge_volume)

    def by_tool(self) -> dict[int, float]:
        """Removed volume by tool number."""
        out: dict[int, float] = {}
        for _, _, tool, volume in self.lines:
            out[tool] = out.get(tool, 0.0) + volume
        return out

    def summary(self) -> str:
        """A few lines: what was removed, what went wrong, how the part
        compares."""
        cm3 = 1e6
        share = ", ".join(f"T{tool} {volume * cm3:.2f}" for tool, volume in sorted(self.by_tool().items()))
        out = [f"{self.stock} on {self.machine}: {self.removed_volume * cm3:.2f} cm³ removed of "
               f"{self.initial_volume * cm3:.2f}" + (f" ({share})" if share else "")]
        if self.unbodied:
            out.append("no body in the tool table for " + ", ".join(f"T{n}" for n in self.unbodied)
                       + " — those moves cut nothing")
        out.extend(str(issue) for issue in self.issues)
        if self.tolerance is not None:
            if self.excess_volume or self.gouge_volume:
                out.append(
                    f"against the design (±{self.tolerance * 1e3:g} mm): {self.excess_volume * 1e9:.1f} mm³ "
                    f"left over, {self.gouge_volume * 1e9:.1f} mm³ missing"
                )
            else:
                out.append(f"the part is the design within ±{self.tolerance * 1e3:g} mm")
        return "\n".join(out)


def _machining(machine: str, stock: str, data: dict) -> Machining:
    design = data["design"] or {}

    def box(b):
        return None if b is None else (tuple(b[0]), tuple(b[1]))

    return Machining(
        machine=machine, stock=stock, part=data["carve"],
        initial_volume=data["initial_volume"], removed_volume=data["removed_volume"],
        remaining_volume=data["remaining_volume"],
        issues=tuple(Issue(*issue) for issue in data["issues"]),
        lines=tuple(tuple(line) for line in data["lines"]),
        cut_span=None if data["cut_span"] is None else tuple(data["cut_span"]),
        tools=tuple(tuple(span) for span in data["tools"]),
        changes=tuple(tuple(change) for change in data["changes"]),
        unbodied=tuple(data["unbodied"]),
        tolerance=design.get("tolerance"),
        excess_volume=design.get("excess_volume"), gouge_volume=design.get("gouge_volume"),
        excess_bounds=box(design.get("excess_bounds")), gouge_bounds=box(design.get("gouge_bounds")),
    )


@dataclass(frozen=True)
class Run:
    """A part program read and timed for one machine — what `check`
    returns. Times are seconds, lengths metres, positions machine
    coordinates (X, Y, Z)."""

    machine: str
    program: str
    duration: float
    #: The duration's three parts: cutting, at rapid, and standing
    #: (dwells and tool changes).
    cutting_time: float
    rapid_time: float
    pause_time: float
    cutting_distance: float
    rapid_distance: float
    tool_changes: int
    #: The box the axes sweep, and where they stand at the end.
    lower: Point3
    upper: Point3
    end: Point3
    #: Notes that change nothing the axes do (a feed clamped to the
    #: machine's maximum).
    warnings: tuple[str, ...]
    #: `(start, end, line)` per source line, in order.
    spans: tuple[tuple[float, float, int], ...]
    #: `(t, line, what, value)`: `"spindle"` (rev/min — negative
    #: counter-clockwise, 0 stopped), `"coolant"` (1 / 0), `"tool"` (its
    #: number), `"function"` (the M code of a machine function the
    #: program calls), `"end"`.
    events: tuple[tuple[float, int, str, float], ...]
    #: The motion as waypoints.
    times: tuple[float, ...]
    positions: tuple[Point3, ...]
    #: What the run cut, where `check` was given the stock.
    machining: Optional[Machining] = None

    def line_at(self, t: float) -> Optional[int]:
        """The source line running at `t`."""
        for start, end, line in self.spans:
            if start - 1e-12 <= t <= end + 1e-12:
                return line
        return None

    def tools(self) -> list[int]:
        """The tools the program puts in the spindle, in order."""
        return [int(value) for _, _, what, value in self.events if what == "tool"]

    def functions(self) -> list[tuple[int, int]]:
        """`(line, M code)` of every machine function the program calls,
        in order — the codes of the machine's M-code table. Here nothing
        answers them, so they take no time: what the cell takes to answer
        is in the bake."""
        return [(line, int(value)) for _, line, what, value in self.events if what == "function"]

    def summary(self) -> str:
        """One paragraph: the cycle and what it is made of."""
        changes = (f", {self.tool_changes} tool change{'s' if self.tool_changes != 1 else ''}"
                   if self.tool_changes else "")
        box = ", ".join(
            f"{axis} {lo * 1e3:.1f} to {hi * 1e3:.1f}"
            for axis, lo, hi in zip("XYZ", self.lower, self.upper)
        )
        calls = self.functions()
        waits = (f"; {len(calls)} machine function{'s' if len(calls) != 1 else ''} "
                 f"({', '.join(f'M{code}' for _, code in calls)}) not timed" if calls else "")
        return (
            f"{self.program} on {self.machine}: {self.duration:.1f} s — cutting {self.cutting_time:.1f} s "
            f"over {self.cutting_distance * 1e3:.0f} mm, rapids {self.rapid_time:.1f} s over "
            f"{self.rapid_distance * 1e3:.0f} mm, standing {self.pause_time:.1f} s{changes}{waits}; "
            f"axes within {box} mm (machine coordinates)"
        )


def _mechanism(machine: Union[MachineTool, str]) -> tuple[str, str, str]:
    """The NC machine's robot, and its table and head links."""
    if isinstance(machine, str):
        return machine, "table", "head"
    if machine.axes is None:
        raise ValueError(
            f"bt.nc: {machine.name!r} was built without its axes — "
            "bt.parts.machine_tool(axes=True | MachineAxes(...))"
        )
    return machine.axes, machine.table_link, machine.head_link


def _solve3(columns: Sequence[Sequence[float]], rhs: Sequence[float]) -> Point3:
    """`x` with `sum(x[i] * columns[i]) == rhs`, by Cramer's rule."""
    def det(a, b, c):
        return (a[0] * (b[1] * c[2] - b[2] * c[1])
                - b[0] * (a[1] * c[2] - a[2] * c[1])
                + c[0] * (a[1] * b[2] - a[2] * b[1]))

    a, b, c = columns
    d = det(a, b, c)
    if abs(d) < 1e-12:
        raise ValueError("bt.nc: the machine's three axes do not span space")
    return (det(rhs, b, c) / d, det(a, rhs, c) / d, det(a, b, rhs) / d)


def machine_coordinates(scene, machine: Union[MachineTool, str], point: Sequence[float]) -> Point3:
    """The machine coordinates at which the spindle nose stands on `point`
    — a world point on something that rides the table (a corner of the
    part, a frame on the fixture), where the table stands now.

    This is what an operator finds with an edge finder and a gauge block:
    jog until the nose is on the point, read the position page. Here it
    is worked out from the mechanism, so a fixture moved in the cell
    moves its work offset with it."""
    mechanism, table, head = _mechanism(machine)
    names = scene.robot_of(mechanism).joint_names
    q = list(scene.joint_positions_of(mechanism))
    axes = scene.nc_machine(mechanism)["axes"]

    def gap(joints: list) -> Point3:
        """The nose from the point, the point carried by the table."""
        (tx, ty, tz), _ = scene.link_pose_at(table, joints, robot=mechanism)
        (nx, ny, nz), _ = scene.link_pose_at(head, joints, robot=mechanism)
        return (nx - tx, ny - ty, nz - tz)

    (tx, ty, tz), _ = scene.link_pose_at(table, q, robot=mechanism)
    # The axes are prismatic, so the table keeps its orientation: the
    # point stays where it is from the table's frame, and the gap between
    # nose and point changes by a fixed vector per unit of each axis.
    off = (float(point[0]) - tx, float(point[1]) - ty, float(point[2]) - tz)
    now = gap(q)
    columns = []
    for axis in axes:
        stepped = list(q)
        stepped[names.index(axis)] += 1.0
        there = gap(stepped)
        columns.append(tuple(b - a for a, b in zip(now, there)))
    # Close the gap: nose - table == off.
    delta = _solve3(columns, tuple(o - g for o, g in zip(off, now)))
    return tuple(q[names.index(axis)] + d for axis, d in zip(axes, delta))  # type: ignore[return-value]


def set_work_offset(
    scene,
    machine: Union[MachineTool, str],
    name: str = "G54",
    *,
    at: Optional[Sequence[float]] = None,
    xyz: Optional[Sequence[float]] = None,
) -> Point3:
    """Enters a work offset in the machine's table and returns it.

    `at` is the work zero as a world point riding the table — the offset
    is derived from where the axes stand (`machine_coordinates`), for a
    spindle nose with no tool in it: tool lengths go in the tool table
    (`set_tool`) and are added by `G43`. Or give the machine coordinates
    themselves as `xyz`, the figures off a real machine's offset page.
    `name` is `"G54"`…`"G59"` or `"G154 P<n>"`."""
    if (at is None) == (xyz is None):
        raise ValueError("bt.nc.set_work_offset: give the work zero as `at` (a world point) or `xyz` (machine coordinates)")
    mechanism, _, _ = _mechanism(machine)
    offset = machine_coordinates(scene, machine, at) if at is not None else tuple(float(v) for v in xyz)  # type: ignore[union-attr]
    scene.set_work_offset(mechanism, name, offset)
    return offset  # type: ignore[return-value]


def set_tool(
    scene,
    machine: Union[MachineTool, str],
    number: int,
    tool: Optional[Tool] = None,
    *,
    length: Optional[float] = None,
    diameter: Optional[float] = None,
) -> None:
    """Enters tool `number` in the machine's tool table.

    With a `tool` (`end_mill`, `drill` …) the pocket holds that body — it
    is what cuts the stock and what must not touch it — and the table's
    entry is taken from it: the length a presetter would read, the
    cutter's diameter. Giving `length` as well enters *that* in the table
    while the tool stays as long as it is: an operator's mistake, to see
    what the program then does.

    Without a `tool`, `length` is the entry — gauge line to tip (m), what
    `G43 H<number>` adds to Z — and the moves are timed but cut nothing."""
    if tool is None and length is None:
        raise ValueError("bt.nc.set_tool: give the tool, or the length its table entry holds")
    entered = float(tool.length if length is None else length)
    if entered <= 0:
        raise ValueError("bt.nc.set_tool: length must be positive")
    if diameter is None and tool is not None:
        diameter = tool.diameter
    body = None if tool is None else json.dumps({k: v for k, v in asdict(tool).items() if v is not None})
    scene.set_nc_tool(_mechanism(machine)[0], int(number), length=entered,
                      diameter=None if diameter is None else float(diameter), body=body)


def program(scene, name: str, source: Union[str, Path]) -> str:
    """Adds (or replaces) part program `name` and returns the name.
    `source` is the program's text, or a `Path` to read it from."""
    text = source.read_text() if isinstance(source, Path) else str(source)
    scene.add_nc_program(name, text)
    return name


# ------------------------------------------------- the machine's M-code table
@dataclass(frozen=True)
class MCode:
    """One row of a machine's M-code table: an M code that is the
    machine's to carry out, and how it does it.

    The control stops at such a code and waits. `actions` fire as it gets
    there (`bt.seq` actions: a door sent to its end, a relay set) and the
    program goes on once `until` holds (a `bt.seq` condition: the door's
    end lane, the M-FIN, a time; at once without one). `then` fire at that
    moment, and `settle` is what the control still waits for after them —
    a relay dropped once its M-FIN came, and the M-FIN gone again before
    the next call can mistake it for its own.

    `needs` is what must hold **as the control gets to the code**: not a
    wait but an alarm. Where it does not hold the program stops there for
    good, with `alarm` in the step's name — the way a control refuses a
    door command in a cell that is not safe, instead of opening the door
    whenever the cell gets round to being safe.

    `at` is where the axes must stand when the code is called (machine
    coordinates, m) — checked as the machine's program is written: a
    relay that calls an arm calls it to where the arm was taught."""

    what: str
    actions: Sequence[dict] = ()
    until: Optional[dict] = None
    then: Sequence[dict] = ()
    settle: Optional[dict] = None
    needs: Optional[dict] = None
    alarm: str = ""
    at: Optional[Sequence[float]] = None


def _tag(lines: Sequence[str], line: int) -> str:
    """How a step names a source line: its `N` word, or `L<line>`."""
    text = lines[line - 1] if 0 < line <= len(lines) else ""
    match = re.match(r"\s*/?\s*N\s*(\d+)", text, re.IGNORECASE)
    return f"N{match.group(1)}" if match else f"L{line}"


def sequence(
    scene,
    machine: Union[MachineTool, str],
    program: str,
    m_codes: Mapping[int, MCode],
    *,
    name: Optional[str] = None,
    running: Optional[str] = None,
    alarm: Optional[str] = None,
    spindle_needs: Optional[dict] = None,
    spindle_alarm: str = "the spindle may not start",
) -> list[dict]:
    """Writes the machine's own program from a part program that runs the
    cell: the sequence `name` (the program's, by default), a chain of

    * the part program's **blocks** — what the control runs on its own
      between two machine functions (`bt.seq.run_program(block=)`), each
      step named after its lines (`N25-N135`), and
    * at every machine function, the step that answers it out of
      `m_codes` (`N150 M180 window open`), with its alarm where the row
      has a `needs`.

    On `M99` the chain starts over (`gates(cycle=True)`) — the program
    that keeps running, beside which a cell's own programs are baked; on
    `M30` it ends. The rows are entered in the machine's M-code table
    (`scene.set_nc_m_code`), which is what lets the control read them.

    `running` and `alarm` are signals to keep: `running` on while the
    program runs, `alarm` on — and `running` off — where it stopped on an
    alarm. `spindle_needs` is what must hold for a block that turns the
    spindle to start (the doors' closed switches, no E-stop), an alarm
    (`spindle_alarm`) otherwise.

    The program is read here once, whole: what the control cannot read —
    an M code that is in neither table included — is a `ValueError` with
    the line, and so is a function called with the axes away from the
    row's `at`. Returns the blocks (`scene.nc_blocks`)."""
    mechanism, _, _ = _mechanism(machine)
    for code, row in m_codes.items():
        scene.set_nc_m_code(mechanism, int(code), row.what)
    blocks = scene.nc_blocks(mechanism, program)
    lines = scene.nc_program(program).splitlines()

    def said(p: Sequence[float]) -> str:
        return " ".join(f"{axis}{v * 1e3:.1f}" for axis, v in zip("XYZ", p))

    for block in blocks:
        if block["end"] != "function":
            continue
        code, line = block["code"], block["line"]
        if code not in m_codes:
            raise ValueError(
                f"bt.nc.sequence: line {line}: M{code} is in the machine's table but not in the rows "
                f"given here ({', '.join(f'M{c}' for c in sorted(m_codes))})"
            )
        row = m_codes[code]
        if row.at is not None and any(abs(a - b) > 1e-4 for a, b in zip(block["at"], row.at)):
            raise ValueError(
                f"bt.nc.sequence: line {line}: M{code} ({row.what}) is called with the axes at "
                f"{said(block['at'])} (machine coordinates, mm); it is answered with them at "
                f"{said(row.at)} — move there first (G53)"
            )

    stopped = [seq.set_signal(alarm)] if alarm else []
    if running:
        stopped.append(seq.set_signal(running, False))

    def guarded(steps, tag: str, needs: Optional[dict], why: str, body) -> None:
        """`body` behind `needs`: taken where it holds as the chain gets
        here, an alarm that never clears where it does not."""
        if needs is None:
            body(steps)
            return
        choice = steps.select(f"{tag}?")
        body(choice.when(needs))
        choice.when(seq.not_(needs)).step(
            f"{tag} ALARM" + (f" — {why}" if why else ""), actions=stopped,
            transition=seq.not_(seq.immediately()))

    sq = scene.sequence(name or program)
    if running:
        sq.step("cycle start", actions=[seq.set_signal(running)])
    for k, block in enumerate(blocks):
        if block["lines"] is not None:
            first, last = (_tag(lines, n) for n in block["lines"])
            label = first if first == last else f"{first}-{last}"

            def run(steps, label=label, k=k) -> None:
                steps.step(label, actions=[seq.run_program(mechanism, program, block=k)])

            guarded(sq, label, spindle_needs if block["spindle"] else None, spindle_alarm, run)
        if block["end"] != "function":
            continue
        code, row = block["code"], m_codes[block["code"]]
        label = f"{_tag(lines, block['line'])} M{code} {row.what}"

        def answer(steps, label=label, row=row) -> None:
            steps.step(label, actions=list(row.actions), transition=row.until or seq.immediately())
            if row.then or row.settle is not None:
                steps.step(f"{label} — done", actions=list(row.then),
                           transition=row.settle or seq.immediately())

        guarded(sq, label, row.needs, row.alarm, answer)
    sq.gates(cycle=blocks[-1]["end"] == "repeat")
    return blocks


def check(
    scene,
    machine: Union[MachineTool, str],
    program: str,
    *,
    start: Optional[Sequence[float]] = None,
    stock: Optional[str] = None,
    fixtures: Sequence[str] = (),
    design: Optional[Union[str, Path]] = None,
    tolerance: float = 5e-5,
    **options,
) -> Run:
    """A dry run: reads `program` the way the machine's control would —
    from where its axes stand, or from the machine coordinates `start` —
    and times it. Nothing in the scene moves. A `ValueError` names the
    line of whatever the control cannot read.

    With `stock` — the obstacle on the table that is to be machined — the
    run also **cuts**: the stock and the `fixtures` (the vise and its
    jaws, by obstacle name) ride the table from where they stand, the
    tools with bodies are swept through it, and `Run.machining` says what
    came off on which line, what a tool met that it should not have, and
    — with `design`, the finished part as a closed mesh file in the
    stock's own frame — what is left over or missing beyond `tolerance`
    (m). `options` go to the cut (`chord_tolerance`, `max_deviation`,
    `dt`, `tile_size`)."""
    mechanism, table, head = _mechanism(machine)
    joints = None
    if start is not None:
        names = scene.robot_of(mechanism).joint_names
        joints = list(scene.joint_positions_of(mechanism))
        for axis, value in zip(scene.nc_machine(mechanism)["axes"], start):
            joints[names.index(axis)] = float(value)
    data = scene.plan_nc_program(mechanism, program, joints)
    machining = None
    if stock is not None:
        dry = scene.nc_dry_run(mechanism, program, table, [stock, *fixtures], joints)
        machining = _machining(mechanism, stock, dry.machine_stock(
            stock, mechanism, head, list(fixtures), None if design is None else str(design),
            tolerance, **options))
    elif fixtures or design is not None:
        raise ValueError("bt.nc.check: fixtures and a design are checked against a `stock`")
    return Run(
        machine=mechanism,
        program=program,
        duration=data["duration"],
        cutting_time=data["cutting_time"],
        rapid_time=data["rapid_time"],
        pause_time=data["pause_time"],
        cutting_distance=data["cutting_distance"],
        rapid_distance=data["rapid_distance"],
        tool_changes=data["tool_changes"],
        lower=tuple(data["lower"]),
        upper=tuple(data["upper"]),
        end=tuple(data["end"]),
        warnings=tuple(data["warnings"]),
        spans=tuple(tuple(span) for span in data["spans"]),
        events=tuple(tuple(event) for event in data["events"]),
        times=tuple(data["times"]),
        positions=tuple(tuple(p) for p in data["positions"]),
        machining=machining,
    )


def cut(
    timeline,
    machine: Union[MachineTool, str],
    stock: str,
    *,
    fixtures: Sequence[str] = (),
    design: Optional[Union[str, Path]] = None,
    tolerance: float = 5e-5,
    **options,
) -> Machining:
    """What the machine's programs cut out of `stock` over a baked cycle.

    Where `check` assumes the stock sits where it sits, this reads the
    bake: the stock is wherever its own track has it — where the arm
    really put it, clamped to a table that really moved — so the work
    offset, the tool table, the program and the cell's handling all meet
    in the one result. `fixtures`, `design`, `tolerance` and `options` as
    in `check`."""
    mechanism, _, head = _mechanism(machine)
    return _machining(mechanism, stock, timeline.machine_stock(
        stock, mechanism, head, list(fixtures), None if design is None else str(design),
        tolerance, **options))


def animate(
    scene,
    timeline,
    machine: Union[MachineTool, str],
    stock: str,
    *,
    fixtures: Sequence[str] = (),
    design: Optional[Union[str, Path]] = None,
    tolerance: float = 5e-5,
    tools: bool = True,
    **options,
):
    """`cut` with the picture: returns `(timeline, Machining)`, the
    timeline showing the stock as it is cut — on the table that carries
    it, and out of the machine in the gripper as the part — and, with
    `tools`, each tool in the spindle while it is the one in it; where the
    machine is drawn with a tool changer (`machine_tool(shape=)`), a tool
    that is not in the spindle rides its pocket on the carousel.

    Everything added is display (`<stock>_cut/…`, `<machine>/tool/T<n>/…`),
    out of collision: the stock itself goes on being what is gripped and
    checked, unseen. Play the returned timeline (`scene.show_timeline`),
    or export it. `options` also take `stage_period` (s — the display
    lags the cutter by at most this) and `max_stages`."""
    mechanism, _, head = _mechanism(machine)
    shown, data = scene.animate_machining(
        timeline, stock, mechanism, head, list(fixtures), None if design is None else str(design),
        tolerance, show_tools=tools, **options)
    return shown, _machining(mechanism, stock, data)


# --------------------------------------------------------------- setup sheet
def _on_table(scene, mechanism: str, table: str, name: str) -> tuple[Point3, Point3]:
    """An obstacle's centre in the table's frame, and its size along that
    frame's axes (the box of its bounds — right for what stands square on
    a table)."""
    (tx, ty, tz), tq = scene.link_pose(table, robot=mechanism)
    (ox, oy, oz), _ = scene.obstacle_pose(name)
    back = (-tq[0], -tq[1], -tq[2], tq[3])
    centre = _rotate(back, (ox - tx, oy - ty, oz - tz))
    lo, hi = scene.obstacle_bounds(name)
    span = _rotate(back, tuple(h - low for low, h in zip(lo, hi)))
    return centre, tuple(abs(v) for v in span)  # type: ignore[return-value]


def _mm(values: Sequence[float]) -> str:
    return " ".join(f"{axis}{v * 1e3:+.3f}" for axis, v in zip("XYZ", values))


def setup_sheet(
    scene,
    machine: Union[MachineTool, str],
    program: str,
    *,
    stock: Optional[str] = None,
    fixtures: Sequence[str] = (),
    design: Optional[Union[str, Path]] = None,
    tolerance: float = 5e-5,
    title: Optional[str] = None,
    **options,
) -> str:
    """The setup sheet of `program` on `machine`, as Markdown: what an
    operator needs to stand the same setup again, and what a dry run of
    it says — derived from the cell, so it cannot disagree with it.

    The stock and where it stands on the table; the fixtures; the work
    offsets, as machine coordinates and as where the work zero is on the
    stock; the tools — what is in each pocket and what its table entry
    says, flagged where they differ; the speeds the program runs each at;
    the cycle; and the dry run: the travel, what each tool removes, what
    a tool met that it should not have, and the part against its design.
    `stock`, `fixtures`, `design`, `tolerance` and `options` as in
    `check`, which this runs."""
    mechanism, table, head = _mechanism(machine)
    run = check(scene, machine, program, stock=stock, fixtures=fixtures, design=design,
                tolerance=tolerance, **options)
    control = scene.nc_machine(mechanism)
    # The machine is the group its mechanism belongs to (`vf2/axes` of `vf2`).
    name = machine.name if isinstance(machine, MachineTool) else mechanism.rsplit("/", 1)[0]
    part = scene.part(name) or {}
    maker = " ".join(str(v) for v in (part.get("manufacturer"), part.get("model")) if v)
    out = [f"# {title or f'Setup sheet — {program}'}", ""]
    cycle = (f"- Cycle: {run.duration:.1f} s — cutting {run.cutting_time:.1f} s, rapids {run.rapid_time:.1f} s, "
             f"standing {run.pause_time:.1f} s, {run.tool_changes} tool change"
             f"{'' if run.tool_changes == 1 else 's'}")
    out += [f"- Machine: {maker or name} (`{mechanism}`)",
            f"- Program: `{program}`, {len(scene.nc_program(program).splitlines())} lines",
            cycle]
    if calls := run.functions():
        # What the program asks of the machine, in the order it asks: the
        # cycle above is the control's own share — the cell's is in the bake.
        said = control["m_codes"]
        out.append("- Machine functions, in order (the cell's time, not in the cycle above): "
                   + ", ".join(f"M{code} {said.get(code, '')}".strip() + f" (line {line})"
                               for line, code in calls))
    out.append("")

    if stock is not None:
        centre, size = _on_table(scene, mechanism, table, stock)
        described = scene.part(stock) or {}
        model = f" ({described['model']})" if described.get("model") else ""
        stands = (f"- Centre from the middle of the table's top: {_mm(centre)} mm "
                  f"(top face {(centre[2] + size[2] / 2) * 1e3:.1f} mm above the table)")
        out += ["## Stock", "",
                f"- `{stock}`{model}: {' x '.join(f'{v * 1e3:.1f}' for v in size)} mm",
                stands, ""]
    if fixtures:
        out += ["## Fixtures", "", "| obstacle | article | size [mm] | centre on the table [mm] |", "|---|---|---|---|"]
        for fixture in fixtures:
            centre, size = _on_table(scene, mechanism, table, fixture)
            owner = scene.part(fixture) or scene.part(fixture.split("/")[0]) or {}
            article = " ".join(str(v) for v in (owner.get("manufacturer"), owner.get("model")) if v) or "—"
            out.append(f"| `{fixture}` | {article} | {' x '.join(f'{v * 1e3:.1f}' for v in size)} | {_mm(centre)} |")
        out.append("")

    out += ["## Work offsets", ""]
    if control["work_offsets"]:
        header = "| offset | machine coordinates [mm] |" + (" work zero from the stock's centre [mm] |" if stock else "")
        out += [header, "|---|---|" + ("---|" if stock else "")]
        names = scene.robot_of(mechanism).joint_names
        for offset, xyz in control["work_offsets"].items():
            row = f"| {offset} | {_mm(xyz)} |"
            if stock is not None:
                # Where the nose stands on the table with the axes at the
                # offset, seen from the stock: the stock rides the table,
                # so this does not depend on where the table is now.
                joints = list(scene.joint_positions_of(mechanism))
                for axis, value in zip(control["axes"], xyz):
                    joints[names.index(axis)] = value
                (nx, ny, nz), _ = scene.link_pose_at(head, joints, robot=mechanism)
                (tx, ty, tz), tq = scene.link_pose_at(table, joints, robot=mechanism)
                back = (-tq[0], -tq[1], -tq[2], tq[3])
                on_table = _rotate(back, (nx - tx, ny - ty, nz - tz))
                centre, _ = _on_table(scene, mechanism, table, stock)
                row += f" {_mm(tuple(a - b for a, b in zip(on_table, centre)))} |"
            out.append(row)
    else:
        out.append("None entered.")
    out.append("")

    out += ["## Tools", "", "| T | tool | flutes [mm] | holder ⌀ x length [mm] | length [mm] | table entry H [mm] | |",
            "|---|---|---|---|---|---|---|"]
    speeds: dict[int, list[str]] = {}
    current = None
    for _, _, what, value in run.events:
        if what == "tool":
            current = int(value)
        elif what == "spindle" and current is not None and value:
            said = f"S{abs(value):.0f}" + (" CCW" if value < 0 else "")
            if said not in speeds.setdefault(current, []):
                speeds[current].append(said)
    for number, entry in sorted(control["tools"].items()):
        body = entry["body"]
        if body is None:
            out.append(f"| T{number} | — (no body: cuts nothing here) | | | | {entry['length'] * 1e3:.3f} | |")
            continue
        tool = Tool(**{**body, "holder": tuple(body["holder"]) if body.get("holder") else None})
        holder = "—" if tool.holder is None else f"{tool.holder[0] * 1e3:g} x {tool.holder[1] * 1e3:g}"
        off = entry["length"] - tool.length
        flag = "" if abs(off) < 1e-9 else f"**entry {off * 1e3:+.3f} mm from the tool**"
        out.append(f"| T{number} | {tool.describe()} | {tool.flute_length * 1e3:g} | {holder} | "
                   f"{tool.length * 1e3:.3f} | {entry['length'] * 1e3:.3f} | {flag} |")
    out.append("")
    if speeds:
        out += ["Spindle, as programmed: " + "; ".join(
            f"T{number} {', '.join(said)}" for number, said in sorted(speeds.items())) + ".", ""]

    travel = ", ".join(f"{axis} {lo * 1e3:.0f} to {hi * 1e3:.0f}"
                       for axis, (lo, hi) in zip("XYZ", control["travel"]))
    out += ["## Dry run", "",
            (f"- Axes within {_mm(run.lower)} to {_mm(run.upper)} mm (machine coordinates); "
             f"travel {travel} mm")]
    out += [f"- Note: {warning}" for warning in run.warnings]
    cutting = run.machining
    if cutting is None:
        out.append("- Not cut: no stock was named.")
    else:
        cm3 = 1e6
        out.append(f"- Removed: {cutting.removed_volume * cm3:.2f} cm³ of {cutting.initial_volume * cm3:.2f} — "
                   + (", ".join(f"T{tool} {volume * cm3:.2f} cm³" for tool, volume in sorted(cutting.by_tool().items()))
                      or "nothing"))
        for number in cutting.unbodied:
            out.append(f"- **T{number} has no body in the tool table: its moves cut nothing here.**")
        out += [f"- **{issue}**" for issue in cutting.issues] or ["- Nothing met anything it should not have."]
        if cutting.tolerance is not None:
            if cutting.excess_volume or cutting.gouge_volume:
                out.append(f"- **First part against the design (±{cutting.tolerance * 1e3:g} mm): "
                           f"{cutting.excess_volume * 1e9:.1f} mm³ left over, "
                           f"{cutting.gouge_volume * 1e9:.1f} mm³ missing.**")
            else:
                out.append(f"- First part against the design: within ±{cutting.tolerance * 1e3:g} mm.")
    out.append("")
    return "\n".join(out)

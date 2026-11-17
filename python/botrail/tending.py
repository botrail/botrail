"""Machine tending: the machine's side of the handshake, authored as a
program of its own (design/design-machine-tending.md §4).

A machine tool is not a device the robot commands — it runs a part
program and talks to the cell in a fixed vocabulary of signals. FANUC's
Robot Interface 2 says it in M-codes and I/O: `M62` announces the end of
the cycle a few seconds early, `M60` unclamps and opens the side door,
SERVICE REQUEST holds while the door is at its open end, the robot asks
for the work clamp and reports the exchange done, and the door closes
before the next cycle starts. Written out, that is one more PLC program
scanned beside the robot's — which is what these templates author:

    vmc = bt.parts.machine_tool(scene, "vmc", door="servo")
    hs = bt.tending.fanuc_ri2(scene, vmc, cycle_s=42.0)
    tend = scene.sequence("tend")           # the robot's program, by hand
    tend.step("wait", transition=bt.seq.signal(hs.signal("notice")))
    ...
    tl = scene.simulate_sequences(["tend", hs.program])

The templates put nothing in the scene but signals, a sequence and (by
default) the CNC as an I/O node hosting it — so the derived I/O list
shows the handshake as wires between two controllers, PLCopen export
carries the machine's POU on a resource of its own, and the interlock
table (`scene.interlocks()`) reads the guards below as rows. The door
enforces itself: a leaf the machine closes on an arm still inside stops
the bake at that tick, by name.

Three guards every template writes into the machine's program, the way
ISO 16090-1 has a machining centre written:

* the side door opens only with the **front door closed** (the two are
  never open together — while the side door is open, the machine body is
  the fence);
* a cycle starts only with the **side door at its closed end** — the
  confirmation signal, not the command;
* nothing starts while the **E-stop** is pressed.

Four vocabularies: `fanuc_ri2` (FANUC Robot Interface 2), `haas_autodoor`
(Haas M80/M81 auto door with a cell-safe input), `generic` (a
vendor-neutral request/acknowledge, its signal names yours to set) and
`manual` (no interface — the robot works the door and the panel).

In those four the machine's program is written here and machining is one
step of it. `haas_mfin` is the other way round, the way a Haas is
actually integrated: **the part program runs the cell**. One NC program
that keeps running (`M99`) opens the window (`M180`), unclamps and clamps
the vise (`M71`/`M70`), and calls the robot through a user relay (`M21`)
that waits for its M-FIN — and the machine's program is derived from that
text (`bt.nc.sequence`), so the interlock table, the handshake spec and
the bake all read the program the operator loads.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Optional, Union

from . import nc, seq
from .parts import MachineTool

FANUC_RI2 = "fanuc_ri2"
HAAS_AUTODOOR = "haas_autodoor"
HAAS_MFIN = "haas_mfin"
GENERIC = "generic"
MANUAL = "manual"


@dataclass
class Handshake:
    """What a template authored: the machine program's name, the door axis
    it drives (or `None`), the I/O node it lives on, and the signals by
    *role* — `signal("service_req")` is the name the robot's program waits
    on, whatever the machine is called."""

    machine: str
    template: str
    program: str
    door: Optional[str] = None
    node: Optional[str] = None
    signals: dict[str, str] = field(default_factory=dict)

    def signal(self, role: str) -> str:
        if role not in self.signals:
            raise KeyError(f"{self.template}: no signal for the role {role!r} — one of {sorted(self.signals)}")
        return self.signals[role]

    def mtconnect_items(self) -> dict:
        """The machine's lanes as MTConnect data items, for
        `bt.trace.read_mtconnect` / `to_mtconnect`: `Execution` is
        `running`, `DoorState` the side door's two lanes, `EmergencyStop`
        the E-stop, `ChuckState` the work clamp (the nearest standard type
        a vise clamp reports as). Only the roles this machine has."""
        items: dict = {"Execution": self.signals["running"]}
        if "door_closed" in self.signals and "door_open" in self.signals:
            items["DoorState"] = (self.signals["door_closed"], self.signals["door_open"])
        if "estop" in self.signals:
            items["EmergencyStop"] = self.signals["estop"]
        if "clamp" in self.signals:
            items["ChuckState"] = self.signals["clamp"]
        return items


def _door_lane(machine: MachineTool, which: int) -> Optional[str]:
    return machine.door_lanes[which] if machine.door_lanes else None


def _guarded(condition, *lanes: Optional[str], low: Sequence[Optional[str]] = ()):
    """`condition` ANDed with every lane in `lanes` high and every lane in
    `low` low — the guards a machine program carries, skipped where the
    machine has no such lane."""
    terms = [condition]
    terms += [seq.signal(lane) for lane in lanes if lane is not None]
    terms += [seq.signal(lane, False) for lane in low if lane is not None]
    return terms[0] if len(terms) == 1 else seq.all_of(*terms)


def _front_closed(machine: MachineTool) -> tuple:
    """Every lane that is on while the front door is shut: its closed
    switch, or — on a driven door of two leaves — each leaf's closed end.
    Empty without a front door."""
    lanes = tuple(getattr(machine, "front_closed", ()) or ())
    return lanes or ((machine.front_door_lane,) if machine.front_door_lane is not None else ())


def _guards(machine: MachineTool, roles: dict) -> None:
    """Records the guard lanes on the handshake's roles, where the machine
    has them."""
    if machine.front_door_lane is not None:
        roles["front_door_closed"] = machine.front_door_lane
    if machine.estop is not None:
        roles["estop"] = machine.estop


def _stated(machine: MachineTool, template: str, function: str, also: Sequence[str] = ()) -> None:
    stated = (machine.interface or {}).get("template")
    if stated not in (None, template, *also):
        raise ValueError(
            f"{function}: the catalog says {machine.name!r} speaks `{stated}` — use bt.tending.{stated}, "
            "bt.tending.generic with its signal names, or bt.tending.manual to work it through its panel"
        )


def _machining(sq_scene, sq, roles: dict, machine: MachineTool, cycle_s: float,
               part_program: Optional[str], function: str, clamped: Sequence = ()) -> None:
    """The machining step: `cycle_s` on a timer, or — with `part_program`
    — the NC program run on the machine's axes, as long as it takes.
    `clamped` are the actions that put the part in the vise on the table
    before anything moves."""
    if part_program is None:
        sq.step("machining", actions=[*clamped, seq.set_signal(roles["running"])],
                transition=seq.elapsed(cycle_s))
        return
    axes = getattr(machine, "axes", None)
    if axes is None:
        raise ValueError(
            f"{function}: {machine.name!r} was built without its axes — a part program runs on "
            "bt.parts.machine_tool(axes=True | MachineAxes(...))"
        )
    # Read now, as the control will read it: what it cannot read is said
    # here, by line — and so is a program that does not bring the table
    # back to where the cell exchanges parts, which no arm taught there
    # would survive.
    end = sq_scene.plan_nc_program(axes, part_program)["end"]
    park = getattr(machine, "park", None)
    if park is not None and any(abs(a - b) > 1e-4 for a, b in zip(end, park)):
        def at(p):
            return " ".join(f"{axis}{v * 1e3:.1f}" for axis, v in zip("XYZ", p))

        raise ValueError(
            f"{function}: part program {part_program!r} leaves the axes at {at(end)} (machine "
            f"coordinates, mm); the exchange position {machine.name!r} was built at is {at(park)} — "
            "end the program there (G53)"
        )
    sq.step("machining", actions=[*clamped, seq.set_signal(roles["running"]),
                                  seq.run_program(axes, part_program)],
            transition=seq.done())


def _clamped(scene, machine: MachineTool, parts: Optional[Sequence[str]], function: str):
    """What the vise does to the parts of a machine whose table moves:
    `parts = (unloaded, loaded)` — the part clamped as the cycle runs and
    the one the cell loads in its place — ride the table while clamped.
    Returns three lists of actions, all the machine's own: take the first
    onto the table (as the cycle begins), free it (at the unclamp), and
    take the second (as the next cycle starts). All empty for an envelope
    whose table stands still."""
    axes = getattr(machine, "axes", None)
    if parts is None or axes is None:
        if parts is not None:
            raise ValueError(
                f"{function}: {machine.name!r} was built without its axes — its table stands "
                "still, so a clamped part needs nothing to ride on"
            )
        return [], [], []
    if len(parts) != 2:
        raise ValueError(f"{function}: parts = (the part unloaded, the part loaded)")
    unloaded, loaded = parts

    def take(part: str):
        return seq.attach(part, link=machine.table_link, robot=axes)

    return [take(unloaded)], [seq.detach(unloaded)], [take(loaded)]


def _node(scene, machine: str, program: str, node: bool) -> Optional[str]:
    if not node:
        return None
    name = f"{machine}/cnc"
    scene.add_io_node(name, kind="plc", programs=[program], label=f"{machine} CNC (PMC)")
    return name


def fanuc_ri2(
    scene,
    machine: MachineTool,
    *,
    cycle_s: float = 42.0,
    clamp_s: float = 0.8,
    notice_s: float = 5.0,
    part_program: Optional[str] = None,
    program: Optional[str] = None,
    node: bool = True,
) -> Handshake:
    """The FANUC Robot Interface 2 cycle, as the machine runs it.

    Signals (all under `<machine>/`): `running` (a cycle is in progress),
    `notice` (`M62` — complete notice, `notice_s` before the end),
    `service_req` (SERVICE REQUEST: the door is at its open end and the
    machine waits for the robot), `clamp` (the work clamp, `SO25_2`),
    and two the **robot's** program writes — `clamp_req` (asks for the
    clamp once the blank is in the jaws) and `service_ok` (the exchange
    is done and the arm is out).

    The program `<machine>` (or `program=`): machining for
    `cycle_s - notice_s`, the notice, then unclamp (`clamp_s`), the side
    door to its open end, SERVICE REQUEST until the robot asks for the
    clamp, the clamp (`clamp_s`), then wait for `service_ok`, close the
    door and start the next cycle. A machine without an automatic door
    (`door="manual"` or none) skips the door steps — the interlock is
    then the robot's, or nobody's.

    The one thing to get right on the robot's side: `service_ok` goes up
    **after** the arm has left the enclosure. Set it earlier and the door
    closes on the arm — which the bake refuses at that instant, naming
    the door, the leaf, the robot and the link (`DeviceCollision`)."""
    name = machine.name
    program = program or name
    if cycle_s <= 0 or clamp_s < 0 or notice_s < 0:
        raise ValueError("fanuc_ri2: cycle_s must be positive, clamp_s and notice_s non-negative")
    if part_program is not None:
        raise ValueError(
            "fanuc_ri2: a part program is not supported yet — the complete notice goes up "
            "`notice_s` before the end, which a program only states by having run"
        )
    _stated(machine, FANUC_RI2, "fanuc_ri2")
    roles = {
        role: f"{name}/{role}"
        for role in ("running", "notice", "service_req", "clamp", "clamp_req", "service_ok")
    }
    for signal in roles.values():
        scene.define_signal(signal)
    if machine.door_lanes:
        roles["door_closed"], roles["door_open"] = machine.door_lanes
    _guards(machine, roles)
    door = machine.door
    front, estop = _front_closed(machine), machine.estop

    sq = scene.sequence(program)
    sq.step("machining", actions=[seq.set_signal(roles["running"])],
            transition=seq.elapsed(max(cycle_s - notice_s, 0.0)))
    sq.step("notice", actions=[seq.set_signal(roles["notice"])],
            transition=seq.elapsed(notice_s))
    # The side door opens only with the front door shut: the two are
    # never open together.
    sq.step("finish", actions=[seq.set_signal(roles["running"], False),
                               seq.set_signal(roles["notice"], False),
                               seq.set_signal(roles["clamp"], False)],
            transition=_guarded(seq.elapsed(clamp_s), *front))
    if door is not None:
        sq.step("open_door", actions=[seq.move_to(door, "open")],
                transition=seq.device_done(door))
    # Request/acknowledge on levels, the PLC way: the robot raises
    # `clamp_req`, the machine answers with `clamp`, the robot drops the
    # request once it sees the answer — no edge can fall between scans.
    sq.step("service", actions=[seq.set_signal(roles["service_req"])],
            transition=seq.signal(roles["clamp_req"]))
    sq.step("clamp", actions=[seq.set_signal(roles["clamp"])],
            transition=seq.elapsed(clamp_s))
    sq.step("wait_robot", transition=seq.signal(roles["service_ok"]))
    # The next cycle starts on the door's closed *confirmation*, not on the
    # close command having been given — and never with the E-stop in.
    if door is not None:
        sq.step("close_door", actions=[seq.set_signal(roles["service_req"], False),
                                       seq.move_to(door, "closed")],
                transition=_guarded(seq.device_done(door), roles.get("door_closed"), low=[estop]))
    else:
        sq.step("resume", actions=[seq.set_signal(roles["service_req"], False)],
                transition=_guarded(seq.immediately(), low=[estop]))
    sq.step("cycle_start", actions=[seq.set_signal(roles["running"])],
            transition=seq.elapsed(1.0))
    return Handshake(name, FANUC_RI2, program, door=door,
                     node=_node(scene, name, program, node), signals=roles)


def haas_autodoor(
    scene,
    machine: MachineTool,
    *,
    cycle_s: float = 42.0,
    clamp_s: float = 0.8,
    part_program: Optional[str] = None,
    parts: Optional[Sequence[str]] = None,
    program: Optional[str] = None,
    node: bool = True,
) -> Handshake:
    """The Haas auto-door cycle: `M80` opens the door, `M81` closes it, and
    the door works only while the **cell-safe** input is on (a light
    curtain clear, a cell fence shut); cycle start closes the door.

    Signals (under `<machine>/`): `running`, `part_done` (the program has
    ended and the part is unclamped — the M-code relay a robot cell wires
    to), `clamp` (a user M-code relay, `M21`-style, with its M-Fin), and
    three the **cell** writes — `cell_safe` (it is safe to move the
    door), `clamp_req` (clamp the blank) and `start_req` (remote cycle
    start, once the arm is out). A driven side door is required
    (`door="air"` / `"servo"`): `M80`/`M81` are auto-door codes.

    The program: machining for `cycle_s`, unclamp (`clamp_s`) and raise
    `part_done`, wait for `cell_safe` **and the front door closed**, open
    the door, clamp on request, wait for `start_req` **with no E-stop**,
    close the door, and start once it reads closed.

    This is the machine's program written for it, started from the cell.
    A Haas is more often integrated the other way round — one part
    program that keeps running and works the door, the vise and a relay
    with M-FIN itself, with no start from outside: that is `haas_mfin`,
    which reads the machine's side off the program.

    With `part_program` — an NC program added with `bt.nc.program`, on a
    machine built with its axes — machining is that program run on the
    axes instead of the `cycle_s` timer: the cycle is as long as the
    program takes. It is read here once, as the control will read it, so
    what it cannot read is refused now and by line — and so is a program
    that does not end at the exchange position the machine was built at
    (`machine.park`), where the arm is taught. Enter the work offsets and
    the tools before calling this.

    On such a machine the table moves, and what the vise holds moves with
    it: `parts = (unloaded, loaded)` names the part clamped as the cycle
    runs and the one the cell loads in its place. The first is taken onto
    the table as the cycle begins and freed at the unclamp; the second is
    taken as the door closes for the next cycle — once the arm has let go
    of it."""
    name = machine.name
    program = program or name
    if cycle_s <= 0 or clamp_s < 0:
        raise ValueError("haas_autodoor: cycle_s must be positive, clamp_s non-negative")
    if machine.door is None:
        raise ValueError(
            f"haas_autodoor: {name!r} has no driven side door — M80/M81 need one "
            "(machine_tool(door='air' | 'servo'))"
        )
    _stated(machine, HAAS_AUTODOOR, "haas_autodoor")
    roles = {
        role: f"{name}/{role}"
        for role in ("running", "part_done", "cell_safe", "clamp", "clamp_req", "start_req")
    }
    for signal in roles.values():
        scene.define_signal(signal)
    roles["door_closed"], roles["door_open"] = machine.door_lanes  # type: ignore[misc]
    _guards(machine, roles)
    door = machine.door
    front, estop = _front_closed(machine), machine.estop

    first, free, hold = _clamped(scene, machine, parts, "haas_autodoor")
    sq = scene.sequence(program)
    _machining(scene, sq, roles, machine, cycle_s, part_program, "haas_autodoor", first)
    sq.step("finish", actions=[seq.set_signal(roles["running"], False),
                               seq.set_signal(roles["clamp"], False), *free],
            transition=seq.elapsed(clamp_s))
    # M80 waits for the cell to be safe — and the front door shut.
    sq.step("part_done", actions=[seq.set_signal(roles["part_done"])],
            transition=_guarded(seq.signal(roles["cell_safe"]), *front))
    sq.step("open_door", actions=[seq.move_to(door, "open")],
            transition=seq.device_done(door))
    sq.step("wait_clamp", transition=seq.signal(roles["clamp_req"]))
    sq.step("clamp", actions=[seq.set_signal(roles["clamp"])],
            transition=seq.elapsed(clamp_s))
    # Cycle start from the cell closes the door (M81) — never on an E-stop.
    sq.step("wait_start", transition=_guarded(seq.signal(roles["start_req"]), low=[estop]))
    sq.step("close_door", actions=[seq.set_signal(roles["part_done"], False), *hold,
                                   seq.move_to(door, "closed")],
            transition=_guarded(seq.device_done(door), roles["door_closed"]))
    sq.step("cycle_start", actions=[seq.set_signal(roles["running"])],
            transition=seq.elapsed(1.0))
    return Handshake(name, HAAS_AUTODOOR, program, door=door,
                     node=_node(scene, name, program, node), signals=roles)


def _m_code(machine: MachineTool, meaning: str) -> Optional[int]:
    """The M code the machine's pack gives for `meaning`, if it gives one."""
    for row in (machine.interface or {}).get("signals") or ():
        if row.get("role") == "mcode" and row.get("meaning") == meaning:
            digits = str(row.get("id", "")).lstrip("Mm")
            if digits.isdigit():
                return int(digits)
    return None


def haas_mfin(
    scene,
    machine: MachineTool,
    part_program: str,
    *,
    parts: Sequence[str],
    clamp_s: float = 0.8,
    relays: Union[Sequence[int], Mapping[int, str]] = (21,),
    door_codes: Optional[tuple[int, int]] = None,
    front_codes: Optional[tuple[int, int]] = None,
    reach: Optional[float] = None,
    program: Optional[str] = None,
    node: bool = True,
) -> Handshake:
    """A Haas run by its part program: the program — added with
    `bt.nc.program`, on a machine built with its axes — opens and closes
    the door, works the vise and calls the cell, and this writes the
    machine's side of it from the text (`bt.nc.sequence`). End the program
    on `M99` and it keeps running, cycle after cycle.

    The M codes it answers (the machine's M-code table):

    * `M80` / `M81` — the auto door, or the codes the machine's pack names
      for its side door (`M180` / `M181`, a VF's auto window; `door_codes`
      overrides). The door goes to its end and the program waits for that
      end's switch. **Both need the cell safe** — and opening needs the
      front door shut: without, the control alarms and the program stops
      there (a Haas reports alarm 548), it does not wait.
    * On a machine built with an automatic **front door**
      (`machine_tool(front_drive=)`), `M80` / `M81` are that door — both
      leaves, at once (`front_codes` overrides) — and the side door's
      codes are the pack's, or `M180` / `M181`. The two doors are never
      open together: either's opening needs the other shut.
    * `M70` / `M71` — the vise clamps / unclamps, `clamp_s` each.
    * `M21`…`M28` (`relays`) — a user relay with M-FIN: the relay comes
      on, the program waits for `m_fin`, drops the relay and waits for
      `m_fin` to go again. Each is called where the arm was taught: with
      the axes at the exchange position the machine was built at
      (`machine.park`), which is checked here. A mapping says what each
      asks of the cell (`{21: "arm: part out, blank in"}`) — the words
      its step, the interlock table and the setup sheet then carry.

    A block that turns the spindle needs the doors' closed switches and
    no E-stop: an alarm otherwise, not a wait.

    Signals (under `<machine>/`): `running`, `alarm`, `clamp`, one per
    relay (`m21`), and two the **cell** writes — `cell_safe` and `m_fin`.
    The control's own lanes are there without being asked for:
    `<axes>.spindle`, `.coolant`, and the words `.rpm`, `.tool` and `.line`
    (the source line running, 0 while the control waits).

    **The vise holds what stands in it.** `clamp` is a coil the table's
    grasp follows (`scene.bind_grasp`): on, the table takes whichever of
    `parts` — every part this vise will hold, the one in it now first —
    stands within `reach` of it, and off lets it go. So nothing is
    attached by name, cycle after cycle; a blank the arm still holds as
    the vise closes becomes the vise's the moment the arm lets go; and a
    program that cuts without `M70` leaves its part behind as the table
    moves. The vise is clamped as the cell starts. `reach` defaults to
    how high the first part stands over the table, and a little.

    On the cell's side: hold `cell_safe` on from before the door is
    commanded; answer a relay by doing the work and raising `m_fin`,
    drop it once the relay has dropped."""
    name = machine.name
    program = program or name
    axes = getattr(machine, "axes", None)
    if axes is None:
        raise ValueError(
            f"haas_mfin: {name!r} was built without its axes — a part program runs on "
            "bt.parts.machine_tool(axes=True | MachineAxes(...))"
        )
    if clamp_s < 0:
        raise ValueError("haas_mfin: clamp_s must be non-negative")
    if not parts:
        raise ValueError("haas_mfin: name the parts the vise holds (parts=[...]), the one in it now first")
    asks = dict(relays) if isinstance(relays, Mapping) else {int(code): "call the cell" for code in relays}
    if bad := [code for code in asks if not 21 <= int(code) <= 28]:
        raise ValueError(f"haas_mfin: {bad} — the user relays that wait for M-FIN are M21 to M28")
    _stated(machine, HAAS_MFIN, "haas_mfin", also=(HAAS_AUTODOOR,))
    roles = {role: f"{name}/{role}" for role in ("running", "alarm", "cell_safe", "clamp", "m_fin")}
    roles.update({f"m{int(code)}": f"{name}/m{int(code)}" for code in asks})
    for role, signal in roles.items():
        # Clamped as the cell starts: somebody stood the first part.
        scene.define_signal(signal, initial=role == "clamp")
    if machine.door_lanes:
        roles["door_closed"], roles["door_open"] = machine.door_lanes
    _guards(machine, roles)
    roles["spindle"], roles["coolant"] = f"{axes}.spindle", f"{axes}.coolant"
    door = machine.door
    front, estop = _front_closed(machine), machine.estop

    # The vise: the table takes what stands in it while the clamp is on.
    if reach is None:
        (_, _, top), _ = scene.frame(f"{name}/table")
        reach = max(scene.obstacle_bounds(parts[0])[0][2] - top, 0.0) + 0.02
    scene.bind_grasp(roles["clamp"], robot=axes, parts=list(parts), reach_m=reach, link=machine.table_link)

    table: dict[int, nc.MCode] = {}
    safe = seq.signal(roles["cell_safe"])
    leaves = tuple(getattr(machine, "front_doors", ()) or ())
    not_safe = "the cell is not safe"
    if door is not None:
        # The side door: the pack's codes — or, where the front door is
        # the auto door, the auto window's.
        spare = (180, 181) if leaves else (80, 81)
        opens, closes = door_codes or (_m_code(machine, "side_door_open") or spare[0],
                                       _m_code(machine, "side_door_close") or spare[1])
        said = "side door" if leaves else "door"
        table[opens] = nc.MCode(
            f"{said} open", actions=[seq.move_to(door, "open")],
            until=_guarded(seq.device_done(door), roles.get("door_open")),
            needs=_guarded(safe, *front),
            alarm=f"{not_safe} or the front door is open" if front else not_safe)
        table[closes] = nc.MCode(
            f"{said} closed", actions=[seq.move_to(door, "closed")],
            until=_guarded(seq.device_done(door), roles.get("door_closed")),
            needs=safe, alarm=not_safe)
    if leaves:
        # The automatic front door: its two leaves are one door to the
        # program — commanded together, and there when both are.
        roles["front_door_open"] = machine.front_open[0]
        opens, closes = front_codes or (_m_code(machine, "front_door_open") or 80,
                                        _m_code(machine, "front_door_close") or 81)
        if opens in table or closes in table:
            raise ValueError(
                f"haas_mfin: M{opens}/M{closes} are the side door's codes here — give the front "
                "door its own (front_codes=) or the side door's (door_codes=)"
            )
        side = roles.get("door_closed")
        table[opens] = nc.MCode(
            "front door open", actions=[seq.move_to(leaf, "open") for leaf in leaves],
            until=seq.all_of(*(seq.device_done(leaf) for leaf in leaves),
                             *(seq.signal(lane) for lane in machine.front_open)),
            needs=_guarded(safe, side),
            alarm=f"{not_safe} or the side door is open" if side else not_safe)
        table[closes] = nc.MCode(
            "front door closed", actions=[seq.move_to(leaf, "closed") for leaf in leaves],
            until=seq.all_of(*(seq.device_done(leaf) for leaf in leaves),
                             *(seq.signal(lane) for lane in front)),
            needs=safe, alarm=not_safe)
    table[70] = nc.MCode("vise clamp", actions=[seq.set_signal(roles["clamp"])], until=seq.elapsed(clamp_s))
    table[71] = nc.MCode("vise unclamp", actions=[seq.set_signal(roles["clamp"], False)],
                         until=seq.elapsed(clamp_s))
    for code, ask in asks.items():
        relay = roles[f"m{int(code)}"]
        table[int(code)] = nc.MCode(
            ask, actions=[seq.set_signal(relay)], until=seq.signal(roles["m_fin"]),
            then=[seq.set_signal(relay, False)], settle=seq.signal(roles["m_fin"], False), at=machine.park)

    # What the spindle starts on: both doors shut, the E-stop out.
    spins = [seq.signal(lane) for lane in (roles.get("door_closed"), *front) if lane is not None]
    if estop is not None:
        spins.append(seq.signal(estop, False))
    nc.sequence(
        scene, machine, part_program, table, name=program, running=roles["running"], alarm=roles["alarm"],
        spindle_needs=None if not spins else spins[0] if len(spins) == 1 else seq.all_of(*spins),
        spindle_alarm="a door is open or the E-stop is in")
    return Handshake(name, HAAS_MFIN, program, door=door,
                     node=_node(scene, name, program, node), signals=roles)


def generic(
    scene,
    machine: MachineTool,
    *,
    cycle_s: float = 42.0,
    clamp_s: float = 0.8,
    signals: Optional[dict[str, str]] = None,
    part_program: Optional[str] = None,
    parts: Optional[Sequence[str]] = None,
    program: Optional[str] = None,
    node: bool = True,
) -> Handshake:
    """A vendor-neutral request/acknowledge handshake — the shape most
    integrators wire when the machine's interface is a handful of relays:

    * the machine says `running` while a cycle runs and `ready` once the
      part is unclamped and the door (if driven) stands open;
    * the cell asks `clamp_req` with the blank in the jaws and gets
      `clamp` back, then reports `exchange_done` once the arm is out;
    * the machine closes the door and starts the next cycle.

    `signals` renames any role to the maker's tag (`{"ready": "vmc/M_FIN",
    ...}`) — a catalog pack's `interface.signals` slots in here — so the
    I/O list and the handshake spec carry the names the electrician reads.
    A machine without a driven door skips the door steps. Guards: the
    door opens only with the front door closed; the next cycle starts only
    with the side door confirmed closed and no E-stop. `part_program`
    runs an NC program on the machine's axes in place of the `cycle_s`
    timer, and `parts` names what the vise holds on a table that moves
    (see `haas_autodoor`)."""
    name = machine.name
    program = program or name
    if cycle_s <= 0 or clamp_s < 0:
        raise ValueError("generic: cycle_s must be positive, clamp_s non-negative")
    roles = {
        role: f"{name}/{role}"
        for role in ("running", "ready", "clamp", "clamp_req", "exchange_done")
    }
    for role, signal in (signals or {}).items():
        if role not in roles:
            raise ValueError(f"generic: no role {role!r} to rename — one of {sorted(roles)}")
        roles[role] = signal
    for signal in roles.values():
        scene.define_signal(signal)
    if machine.door_lanes:
        roles["door_closed"], roles["door_open"] = machine.door_lanes
    _guards(machine, roles)
    door = machine.door
    front, estop = _front_closed(machine), machine.estop

    first, free, hold = _clamped(scene, machine, parts, "generic")
    sq = scene.sequence(program)
    _machining(scene, sq, roles, machine, cycle_s, part_program, "generic", first)
    sq.step("finish", actions=[seq.set_signal(roles["running"], False),
                               seq.set_signal(roles["clamp"], False), *free],
            transition=_guarded(seq.elapsed(clamp_s), *front))
    if door is not None:
        sq.step("open_door", actions=[seq.move_to(door, "open")],
                transition=seq.device_done(door))
    sq.step("ready", actions=[seq.set_signal(roles["ready"])],
            transition=seq.signal(roles["clamp_req"]))
    sq.step("clamp", actions=[seq.set_signal(roles["clamp"])],
            transition=seq.elapsed(clamp_s))
    sq.step("wait_exchange", transition=seq.signal(roles["exchange_done"]))
    if door is not None:
        sq.step("close_door", actions=[seq.set_signal(roles["ready"], False), *hold,
                                       seq.move_to(door, "closed")],
                transition=_guarded(seq.device_done(door), roles.get("door_closed"), low=[estop]))
    else:
        sq.step("resume", actions=[seq.set_signal(roles["ready"], False), *hold],
                transition=_guarded(seq.immediately(), low=[estop]))
    sq.step("cycle_start", actions=[seq.set_signal(roles["running"])],
            transition=seq.elapsed(1.0))
    return Handshake(name, GENERIC, program, door=door,
                     node=_node(scene, name, program, node), signals=roles)


def manual(
    scene,
    machine: MachineTool,
    *,
    cycle_s: float = 42.0,
    clamp_s: float = 0.8,
    buttons: Sequence[str] = ("unclamp", "clamp", "cycle_start"),
    part_program: Optional[str] = None,
    parts: Optional[Sequence[str]] = None,
    program: Optional[str] = None,
    node: bool = True,
) -> Handshake:
    """A machine with no robot interface, worked through its panel — the
    retrofit case: the robot pulls the door and presses the buttons an
    operator would.

    `buttons` names the panel buttons for unclamp, clamp and cycle start,
    in that order (they must exist on the machine's panel). Signals:
    `running` and `clamp`, plus the buttons' own zone lanes. The program
    `<machine>`: machining for `cycle_s`, then wait for UNCLAMP, release
    the clamp (`clamp_s`), wait for CLAMP, clamp (`clamp_s`), then wait
    for CYCLE START **with the side door at its closed end** — a start
    pressed with the door open is ignored, the way a guard interlock
    ignores it (ISO 16090-1), and the bake reports the deadlock naming
    this step. `part_program` runs an NC program on the machine's axes in
    place of the `cycle_s` timer, and `parts` names what the vise holds
    on a table that moves (see `haas_autodoor`)."""
    name = machine.name
    program = program or name
    if cycle_s <= 0 or clamp_s < 0:
        raise ValueError("manual: cycle_s must be positive, clamp_s non-negative")
    if len(buttons) != 3:
        raise ValueError("manual: buttons = (unclamp, clamp, cycle_start)")
    lanes = []
    for button in buttons:
        lane = f"{name}/panel/{button}"
        if lane not in machine.buttons:
            raise ValueError(
                f"manual: the panel of {name!r} has no button {button!r} — it has "
                f"{[b.rsplit('/', 1)[-1] for b in machine.buttons]}"
            )
        lanes.append(lane)
    roles = {"running": f"{name}/running", "clamp": f"{name}/clamp",
             "unclamp_button": lanes[0], "clamp_button": lanes[1], "start_button": lanes[2]}
    scene.define_signal(roles["running"])
    scene.define_signal(roles["clamp"], initial=True)
    closed = _door_lane(machine, 0)
    if closed is not None:
        roles["door_closed"], roles["door_open"] = machine.door_lanes  # type: ignore[misc]
    _guards(machine, roles)

    first, free, hold = _clamped(scene, machine, parts, "manual")
    sq = scene.sequence(program)
    _machining(scene, sq, roles, machine, cycle_s, part_program, "manual", first)
    sq.step("done", actions=[seq.set_signal(roles["running"], False)],
            transition=seq.rising(lanes[0]))
    sq.step("unclamp", actions=[seq.set_signal(roles["clamp"], False), *free],
            transition=seq.elapsed(clamp_s))
    sq.step("wait_clamp", transition=seq.rising(lanes[1]))
    sq.step("clamp", actions=[seq.set_signal(roles["clamp"])],
            transition=seq.elapsed(clamp_s))
    # The start takes with both doors confirmed shut and no E-stop — a
    # press with the door open is ignored, the way a guard interlock
    # ignores it.
    start = _guarded(seq.rising(lanes[2]), closed, *_front_closed(machine), low=[machine.estop])
    sq.step("wait_start", transition=start)
    sq.step("cycle_start", actions=[seq.set_signal(roles["running"]), *hold],
            transition=seq.elapsed(1.0))
    return Handshake(name, MANUAL, program, door=None,
                     node=_node(scene, name, program, node), signals=roles)

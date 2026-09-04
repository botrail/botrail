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
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Optional

from . import seq
from .parts import MachineTool

FANUC_RI2 = "fanuc_ri2"
HAAS_AUTODOOR = "haas_autodoor"
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


def _guards(machine: MachineTool, roles: dict) -> None:
    """Records the guard lanes on the handshake's roles, where the machine
    has them."""
    if machine.front_door_lane is not None:
        roles["front_door_closed"] = machine.front_door_lane
    if machine.estop is not None:
        roles["estop"] = machine.estop


def _stated(machine: MachineTool, template: str, function: str) -> None:
    stated = (machine.interface or {}).get("template")
    if stated not in (None, template):
        raise ValueError(
            f"{function}: the catalog says {machine.name!r} speaks `{stated}` — use bt.tending.{stated}, "
            "bt.tending.generic with its signal names, or bt.tending.manual to work it through its panel"
        )


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
    front, estop = machine.front_door_lane, machine.estop

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
            transition=_guarded(seq.elapsed(clamp_s), front))
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
    close the door, and start once it reads closed."""
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
    front, estop = machine.front_door_lane, machine.estop

    sq = scene.sequence(program)
    sq.step("machining", actions=[seq.set_signal(roles["running"])],
            transition=seq.elapsed(cycle_s))
    sq.step("finish", actions=[seq.set_signal(roles["running"], False),
                               seq.set_signal(roles["clamp"], False)],
            transition=seq.elapsed(clamp_s))
    # M80 waits for the cell to be safe — and the front door shut.
    sq.step("part_done", actions=[seq.set_signal(roles["part_done"])],
            transition=_guarded(seq.signal(roles["cell_safe"]), front))
    sq.step("open_door", actions=[seq.move_to(door, "open")],
            transition=seq.device_done(door))
    sq.step("wait_clamp", transition=seq.signal(roles["clamp_req"]))
    sq.step("clamp", actions=[seq.set_signal(roles["clamp"])],
            transition=seq.elapsed(clamp_s))
    # Cycle start from the cell closes the door (M81) — never on an E-stop.
    sq.step("wait_start", transition=_guarded(seq.signal(roles["start_req"]), low=[estop]))
    sq.step("close_door", actions=[seq.set_signal(roles["part_done"], False),
                                   seq.move_to(door, "closed")],
            transition=_guarded(seq.device_done(door), roles["door_closed"]))
    sq.step("cycle_start", actions=[seq.set_signal(roles["running"])],
            transition=seq.elapsed(1.0))
    return Handshake(name, HAAS_AUTODOOR, program, door=door,
                     node=_node(scene, name, program, node), signals=roles)


def generic(
    scene,
    machine: MachineTool,
    *,
    cycle_s: float = 42.0,
    clamp_s: float = 0.8,
    signals: Optional[dict[str, str]] = None,
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
    with the side door confirmed closed and no E-stop."""
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
    front, estop = machine.front_door_lane, machine.estop

    sq = scene.sequence(program)
    sq.step("machining", actions=[seq.set_signal(roles["running"])],
            transition=seq.elapsed(cycle_s))
    sq.step("finish", actions=[seq.set_signal(roles["running"], False),
                               seq.set_signal(roles["clamp"], False)],
            transition=_guarded(seq.elapsed(clamp_s), front))
    if door is not None:
        sq.step("open_door", actions=[seq.move_to(door, "open")],
                transition=seq.device_done(door))
    sq.step("ready", actions=[seq.set_signal(roles["ready"])],
            transition=seq.signal(roles["clamp_req"]))
    sq.step("clamp", actions=[seq.set_signal(roles["clamp"])],
            transition=seq.elapsed(clamp_s))
    sq.step("wait_exchange", transition=seq.signal(roles["exchange_done"]))
    if door is not None:
        sq.step("close_door", actions=[seq.set_signal(roles["ready"], False),
                                       seq.move_to(door, "closed")],
                transition=_guarded(seq.device_done(door), roles.get("door_closed"), low=[estop]))
    else:
        sq.step("resume", actions=[seq.set_signal(roles["ready"], False)],
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
    this step."""
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

    sq = scene.sequence(program)
    sq.step("machining", actions=[seq.set_signal(roles["running"])],
            transition=seq.elapsed(cycle_s))
    sq.step("done", actions=[seq.set_signal(roles["running"], False)],
            transition=seq.rising(lanes[0]))
    sq.step("unclamp", actions=[seq.set_signal(roles["clamp"], False)],
            transition=seq.elapsed(clamp_s))
    sq.step("wait_clamp", transition=seq.rising(lanes[1]))
    sq.step("clamp", actions=[seq.set_signal(roles["clamp"])],
            transition=seq.elapsed(clamp_s))
    # The start takes with both doors confirmed shut and no E-stop — a
    # press with the door open is ignored, the way a guard interlock
    # ignores it.
    start = _guarded(seq.rising(lanes[2]), closed, machine.front_door_lane, low=[machine.estop])
    sq.step("wait_start", transition=start)
    sq.step("cycle_start", actions=[seq.set_signal(roles["running"])],
            transition=seq.elapsed(1.0))
    return Handshake(name, MANUAL, program, door=None,
                     node=_node(scene, name, program, node), signals=roles)

"""Robot controller interfaces as data (design-virtual-commissioning.md
VC5, §4.6 / §4.8 fidelity level 1).

A PLC drives a robot through a handful of wires the vendor names: FANUC
calls them the UOP (`PROD_START`, `PNS1-8` + `PNSTROBE`, `PROGRUN`), KUKA
Automatic External (`$EXT_START`, `PGNO` + `PGNO_VALID`, `APPL_RUN`), ABB
System I/O (`Start at Main`, `Cycle On`), Universal Robots the remote
control inputs of its I/O setup. botrail's generic handshake has the same
three parts — a `start` pulse, a `program` word, a `done` contact (the I/O
map's rule ⑥) — so a vendor interface is a **mapping**: which channel of
the controller node each part lands on, with the vendor's name, polarity,
edge and pulse width. That mapping is what these templates hold, as data,
and `apply` writes it into the I/O map as bindings (`tag`, `invert`,
`note`), from where the I/O list, the exchange table and the handshake
specification carry it::

    scene.add_io_node("RC1", kind="robot_controller", robots=["arm"], channels=bt.io.fanuc_uop())
    uop = bt.vc.interfaces.fanuc_uop_pns(programs={"pick": 1})
    uop.apply(scene, "arm", "RC1")              # arm.start → UI18 PROD_START, arm.program → PNS, arm.done → UO3 PROGRUN
    print(uop.to_markdown())                    # the interface sheet: every signal, its polarity, edge, timing
    print(tl.handshake_spec())                  # the robot lines now say `PROD_START — falling edge, pulse ≥ 100 ms`

Nothing here changes how the plant or the exporters behave: the engine
keeps its generic handshake, the sheet tells the integrator what the
controller expects. The numbers are the manuals' standard assignments and
the usual PLC timings — check them against the controller's own I/O
configuration before wiring.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Optional

__all__ = [
    "Line",
    "RobotInterface",
    "fanuc_uop_pns",
    "kuka_automatic_external",
    "abb_system_io",
    "ur_remote",
    "robot_interface",
]

#: The three parts of botrail's generic robot handshake, the aspects the
#: I/O map derives for a robot driven from another host.
HANDSHAKE_ROLES = ("start", "program", "done")


@dataclass(frozen=True)
class Line:
    """One signal of a vendor interface: its `role` (botrail's word for
    it — `start`, `program`, `done`, and the ones the generic handshake
    does not drive: `enable`, `ready`, `fault`, `reset`, `at_home`, …),
    the controller channel it lands on, the vendor's name (`tag`), the
    direction seen from the PLC (`in` = PLC → robot, `out` = robot → PLC),
    the wire polarity (`invert` for an active-low / NC signal), the edge
    the controller acts on and the pulse width the PLC must hold."""

    role: str
    channel: str
    tag: str
    direction: str
    meaning: str
    invert: bool = False
    edge: Optional[str] = None
    pulse_ms: Optional[int] = None
    note: str = ""

    @property
    def timing(self) -> str:
        """`falling edge, pulse ≥ 100 ms` — the binding note."""
        parts = []
        if self.edge:
            parts.append(f"{self.edge} edge")
        if self.pulse_ms:
            parts.append(f"pulse ≥ {self.pulse_ms} ms")
        if self.invert:
            parts.append("active low")
        if self.note:
            parts.append(self.note)
        return ", ".join(parts)


@dataclass(frozen=True)
class RobotInterface:
    """A vendor interface template: its lines, the program numbers the
    cell's motions map to (`programs`), and where the data comes from."""

    vendor: str
    name: str
    reference: str
    lines: tuple[Line, ...]
    programs: Mapping[str, int] = field(default_factory=dict)

    def line(self, role: str) -> Line:
        for line in self.lines:
            if line.role == role:
                return line
        raise KeyError(f"{self.name}: no line for the role {role!r} — one of {[l.role for l in self.lines]}")

    def has(self, role: str) -> bool:
        return any(line.role == role for line in self.lines)

    def apply(self, scene, robot: str, node: str) -> dict[str, str]:
        """Binds the robot's handshake points on its controller node to the
        template's channels: `<robot>.start` on the start line (an input
        of the controller), `<robot>.program` on the program-select word
        when the cell selects among programs, `<robot>.done` on the busy /
        running line (inverted: done is the program *not* running). The
        node must carry the template's channels (`bt.io.fanuc_uop()` and
        friends). Returns `{role: "<robot>.<aspect>"}` for what was bound."""
        bound: dict[str, str] = {}
        points = {(p.name, p.aspect, p.direction) for p in scene.io_points()}
        for role, aspect in (("start", "start"), ("program", "program")):
            if not self.has(role) or (robot, aspect, "input") not in points:
                continue
            line = self.line(role)
            label = f"{robot}.{aspect}"
            scene.bind_input(label, node, line.channel, tag=line.tag, invert=line.invert,
                             note=line.timing or None)
            bound[role] = label
        if self.has("done") and (robot, "done", "output") in points:
            line = self.line("done")
            label = f"{robot}.done"
            scene.bind_output(label, node, line.channel, tag=line.tag, invert=line.invert,
                              note=line.timing or None)
            bound["done"] = label
        if not bound:
            raise ValueError(
                f"{self.name}: robot `{robot}` is driven by no other host — the handshake points "
                f"(`{robot}.start`, `{robot}.done`) exist only for a robot a program on another controller drives"
            )
        return bound

    def to_markdown(self) -> str:
        """The interface sheet: every line with its direction, polarity,
        edge and timing, and the program numbers."""
        out = [f"# {self.vendor} — {self.name}", "", self.reference, ""]
        out += ["| role | signal | channel | direction | polarity | timing | meaning |",
                "|---|---|---|---|---|---|---|"]
        for line in self.lines:
            direction = "PLC → robot" if line.direction == "in" else "robot → PLC"
            polarity = "active low (NC)" if line.invert else "active high"
            timing = ", ".join(
                p for p in ((f"{line.edge} edge" if line.edge else ""),
                            (f"pulse ≥ {line.pulse_ms} ms" if line.pulse_ms else ""),
                            line.note) if p
            ) or "level"
            out.append(f"| {line.role} | `{line.tag}` | {line.channel} | {direction} | {polarity} | {timing} | {line.meaning} |")
        if self.programs:
            out += ["", "| program | number |", "|---|---|"]
            out += [f"| {name} | {number} |" for name, number in self.programs.items()]
        return "\n".join(out) + "\n"

    def to_dict(self) -> dict:
        return {
            "vendor": self.vendor, "name": self.name, "reference": self.reference,
            "programs": dict(self.programs),
            "lines": [line.__dict__ for line in self.lines],
        }


def _programs(programs: Optional[Mapping[str, int]]) -> dict[str, int]:
    out = {str(k): int(v) for k, v in (programs or {}).items()}
    if len(set(out.values())) != len(out):
        raise ValueError("program numbers must be distinct")
    return out


def fanuc_uop_pns(programs: Optional[Mapping[str, int]] = None, *, strobe_ms: int = 100,
                  start_ms: int = 100) -> RobotInterface:
    """FANUC's **UOP with PNS program selection** on the standard UOP map
    (`bt.io.fanuc_uop()`): the PLC puts the program number on PNS1–8
    (UI[9]–UI[16]), latches it with PNSTROBE (UI[17], a pulse), reads it
    back on SNO1–8 (UO[11]–UO[18]) acknowledged by SNACK (UO[19]), then
    pulses PROD_START (UI[18]) — the program starts on its **falling
    edge**. PROGRUN (UO[3]) is high while it runs. The NC signals *IMSTP,
    *HOLD and *SFSPD (UI[1]–UI[3]) must be held high, ENBL (UI[8]) too,
    and CMDENBL (UO[1]) says the controller takes remote commands.
    `programs` maps the cell's motions to PNS numbers (PNS0001 = 1)."""
    return RobotInterface(
        vendor="FANUC", name="UOP / PNS",
        reference="R-30iB controller operator's manual, UOP signals and PNS program selection (standard assignment).",
        programs=_programs(programs),
        lines=(
            Line("estop_ok", "UI1", "*IMSTP", "in", "immediate stop (must be high to run)", invert=True),
            Line("hold_ok", "UI2", "*HOLD", "in", "hold (must be high to run)", invert=True),
            Line("safe_speed", "UI3", "*SFSPD", "in", "safe speed (must be high to run)", invert=True),
            Line("cycle_stop", "UI4", "CSTOPI", "in", "cycle stop: end the program at its end"),
            Line("reset", "UI5", "FAULT_RESET", "in", "clear the fault", edge="falling", pulse_ms=start_ms),
            Line("resume", "UI6", "START", "in", "resume the paused program", edge="falling", pulse_ms=start_ms),
            Line("home", "UI7", "HOME", "in", "run the home-return program"),
            Line("enable", "UI8", "ENBL", "in", "enable motion (must be high)"),
            Line("program", "PNS", "PNS1-8", "in", "program number, binary on UI[9]–UI[16]", pulse_ms=strobe_ms,
                 note=f"latched by PNSTROBE (UI17) pulse ≥ {strobe_ms} ms, echoed on SNO1-8 (UO11-18) with SNACK (UO19)"),
            Line("strobe", "UI17", "PNSTROBE", "in", "latch the PNS number", edge="rising", pulse_ms=strobe_ms),
            Line("start", "UI18", "PROD_START", "in", "start the selected program from its first line",
                 edge="falling", pulse_ms=start_ms),
            Line("ready", "UO1", "CMDENBL", "out", "remote commands accepted (AUTO, no fault, ENBL)"),
            Line("system_ready", "UO2", "SYSRDY", "out", "servo power on"),
            Line("done", "UO3", "PROGRUN", "out", "a program is running (done = not running)", invert=True),
            Line("paused", "UO4", "PAUSED", "out", "program paused"),
            Line("held", "UO5", "HELD", "out", "hold active"),
            Line("fault", "UO6", "FAULT", "out", "a fault is posted"),
            Line("at_home", "UO7", "ATPERCH", "out", "the arm is at the perch (home) position"),
            Line("tp_enabled", "UO8", "TPENBL", "out", "teach pendant enabled (not remote)"),
            Line("battery_alarm", "UO9", "BATALM", "out", "backup battery alarm"),
            Line("busy", "UO10", "BUSY", "out", "the controller is busy"),
            Line("program_ack", "SNO", "SNO1-8", "out", "the PNS number read back", note="valid with SNACK (UO19)"),
            Line("ack", "UO19", "SNACK", "out", "PNS number acknowledged", edge="rising", pulse_ms=strobe_ms),
        ),
    )


def kuka_automatic_external(programs: Optional[Mapping[str, int]] = None, *, pulse_ms: int = 100) -> RobotInterface:
    """KUKA's **Automatic External** interface (KRC4 / KRC5,
    `bt.io.kuka_automatic_external()`): the controller asks for a program
    number with `PGNO_REQ`, the PLC puts it on the `PGNO` bits and strobes
    `PGNO_VALID`, then pulses `$EXT_START` (rising edge) — `APPL_RUN` /
    `$PRO_ACT` are high while the program runs. `$MOVE_ENABLE` and
    `$DRIVES_OFF` (active low) must be held high, `$DRIVES_ON` pulsed to
    switch the drives on, `$CONF_MESS` acknowledges messages. `$RC_RDY1`
    says the controller is ready for a start."""
    return RobotInterface(
        vendor="KUKA", name="Automatic External",
        reference="KSS 8.x system software manual, Automatic External (CELL.SRC); addresses are the integrator's $IN/$OUT mapping.",
        programs=_programs(programs),
        lines=(
            Line("program", "PGNO", "PGNO", "in", "program number (PGNO_LENGTH bits from PGNO_FBIT)",
                 pulse_ms=pulse_ms, note=f"on PGNO_REQ; made valid by PGNO_VALID pulse ≥ {pulse_ms} ms"),
            Line("strobe", "PGNO_VALID", "PGNO_VALID", "in", "the program number is valid", edge="rising", pulse_ms=pulse_ms),
            Line("start", "EXT_START", "$EXT_START", "in", "start / continue the program", edge="rising", pulse_ms=pulse_ms),
            Line("enable", "MOVE_ENABLE", "$MOVE_ENABLE", "in", "motion enable (must be high)"),
            Line("ack_messages", "CONF_MESS", "$CONF_MESS", "in", "acknowledge messages", edge="rising", pulse_ms=pulse_ms),
            Line("drives_on", "DRIVES_ON", "$DRIVES_ON", "in", "switch the drives on", edge="rising", pulse_ms=pulse_ms),
            Line("drives_ok", "DRIVES_OFF", "$DRIVES_OFF", "in", "drives off (active low: hold high)", invert=True),
            Line("interface_active", "I_O_ACT", "$I_O_ACT", "in", "the Automatic External interface is active"),
            Line("ready", "RC_RDY1", "$RC_RDY1", "out", "ready for a program start"),
            Line("estop_ok", "ALARM_STOP", "$ALARM_STOP", "out", "no emergency stop (active low)", invert=True),
            Line("safety_ok", "USER_SAF", "$USER_SAF", "out", "operator safety closed"),
            Line("drives_ready", "PERI_RDY", "$PERI_RDY", "out", "drives on and ready"),
            Line("calibrated", "ROB_CAL", "$ROB_CAL", "out", "the robot is mastered"),
            Line("interface_confirm", "I_O_ACTCONF", "$I_O_ACTCONF", "out", "interface active, confirmed"),
            Line("fault", "STOPMESS", "$STOPMESS", "out", "a stop message is pending"),
            Line("program_request", "PGNO_REQ", "PGNO_REQ", "out", "the controller asks for a program number"),
            Line("done", "APPL_RUN", "APPL_RUN", "out", "the application program runs (done = not running)", invert=True),
            Line("program_active", "PRO_ACT", "$PRO_ACT", "out", "a program is active"),
            Line("at_home", "IN_HOME", "$IN_HOME", "out", "the arm is at the home position"),
            Line("on_path", "ON_PATH", "$ON_PATH", "out", "the arm is on its programmed path"),
            Line("ext_mode", "EXT", "$EXT", "out", "Automatic External mode selected"),
        ),
    )


def abb_system_io(programs: Optional[Mapping[str, int]] = None, *, pulse_ms: int = 100) -> RobotInterface:
    """ABB's **System I/O** (IRC5 / OmniCore, `bt.io.abb_system_io()`):
    `MotorsOn` then `StartAtMain` (both edge-triggered system inputs)
    start the RAPID main; `CycleOn` is high while it runs, `AutoOn` and
    `MotorsOnState` say the controller is in automatic with motors on.
    Program selection is not a system I/O: a group input (`ProgNo`) the
    RAPID main reads, here on the `program` role."""
    return RobotInterface(
        vendor="ABB", name="System I/O",
        reference="RobotWare technical reference manual — system parameters, topic I/O System: System Input / System Output actions.",
        programs=_programs(programs),
        lines=(
            Line("motors_on", "MotorsOn", "Motors On", "in", "switch the motors on", edge="rising", pulse_ms=pulse_ms),
            Line("motors_off", "MotorsOff", "Motors Off", "in", "switch the motors off", edge="rising", pulse_ms=pulse_ms),
            Line("start", "StartAtMain", "Start at Main", "in", "start the RAPID main from its first line", edge="rising", pulse_ms=pulse_ms),
            Line("resume", "Start", "Start", "in", "continue from the program pointer", edge="rising", pulse_ms=pulse_ms),
            Line("stop", "Stop", "Stop", "in", "stop program execution", edge="rising", pulse_ms=pulse_ms),
            Line("quick_stop", "QuickStop", "Quick Stop", "in", "stop at once", edge="rising", pulse_ms=pulse_ms),
            Line("cycle_stop", "StopAtEndOfCycle", "Stop at End of Cycle", "in", "stop when the cycle ends", edge="rising", pulse_ms=pulse_ms),
            Line("reset_estop", "ResetEmergencyStop", "Reset Emergency Stop", "in", "reset the E-stop state", edge="rising", pulse_ms=pulse_ms),
            Line("reset", "ResetExecutionError", "Reset Execution Error", "in", "reset an execution error", edge="rising", pulse_ms=pulse_ms),
            Line("pp_to_main", "PPToMain", "PP to Main", "in", "program pointer to main", edge="rising", pulse_ms=pulse_ms),
            Line("program", "ProgNo", "ProgNo", "in", "program number (a group input the RAPID main reads)", note="read on Start at Main"),
            Line("auto", "AutoOn", "Auto On", "out", "automatic mode"),
            Line("done", "CycleOn", "Cycle On", "out", "the program runs (done = not running)", invert=True),
            Line("estop", "EmergencyStop", "Emergency Stop", "out", "emergency stop state"),
            Line("fault", "ExecutionError", "Execution Error", "out", "an execution error is pending"),
            Line("ready", "MotorsOnState", "Motors On State", "out", "motors on"),
            Line("motors_off_state", "MotorsOffState", "Motors Off State", "out", "motors off"),
            Line("run_chain_ok", "RunChainOk", "Run Chain OK", "out", "the run chain is closed"),
            Line("task_executing", "TaskExecuting", "TaskExecuting", "out", "a task executes"),
            Line("supervision", "MotionSupervisionTriggered", "Motion Supervision Triggered", "out", "a collision was detected"),
        ),
    )


def ur_remote(*, start: str = "DI0", stop: str = "DI1", running: str = "DO0") -> RobotInterface:
    """Universal Robots' **remote control** on the standard digital I/O
    (`bt.io.ur_standard()`): an input assigned *Start-Prog* in the
    installation's I/O setup starts the loaded program on its rising
    edge, one assigned *Stop-Prog* stops it, an output set to *high when
    running* reports it. UR has no program-number word on the I/O: the
    loaded program itself branches (or the dashboard loads another)."""
    return RobotInterface(
        vendor="Universal Robots", name="remote control (I/O setup)",
        reference="PolyScope manual, Installation → I/O Setup: input actions Start-Prog / Stop-Prog / Pause-Prog, output actions high-when-running; Remote Control enabled.",
        lines=(
            Line("start", start, "Start-Prog", "in", "start the loaded program", edge="rising", pulse_ms=100),
            Line("stop", stop, "Stop-Prog", "in", "stop the program", edge="rising", pulse_ms=100),
            Line("done", running, "Prog-Running", "out", "the program runs (done = not running)", invert=True),
        ),
    )


def robot_interface(scene, robot: str, node: str, template: RobotInterface) -> dict[str, str]:
    """`template.apply(scene, robot, node)` — binds a robot's handshake
    points on its controller node the way the vendor names them."""
    return template.apply(scene, robot, node)

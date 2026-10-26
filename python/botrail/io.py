"""I/O map helpers: channel templates for `scene.add_io_node(channels=...)`
and the fault entries `scene.add_scenario(faults=[...])` takes.

A template returns a list of channel dicts — `{"id", "kind", "port",
"address", "voltage", "logic"}` — that `add_io_node` accepts, so a node's
channel table is one expression:

    scene.add_io_node("PLC1", kind="plc", programs=["transfer"],
                      channels=bt.io.di16(base="%IX0.0") + bt.io.do16(base="%QX0.0"))
    scene.add_io_node("UR", kind="robot_controller", robots=["arm"],
                      channels=bt.io.ur_standard())

Address dialects live here, in Python, not in the core: `base` is a
string the template counts up from — IEC `%IX0.0` / Siemens `I0.0`
(byte.bit, eight bits to a byte), MELSEC `X10` (hex, or octal on the FX
series), Logix `Local:1:I.Data.0` (a flat bit index) — and `radix` /
`word_bits` say how. `port` is the vendor's standard-I/O number — what a
URScript export writes — set by the robot-controller templates.
"""

import csv
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Optional, Union

Channel = dict


def _split_number(text: str, radix: int) -> tuple:
    """`("X", 16, 2)` for `"X10"` in hex: letters, value, digit count."""
    digits = "0123456789abcdefABCDEF"[: radix if radix <= 10 else 10 + 2 * (radix - 10)]
    i = len(text)
    while i > 0 and text[i - 1] in digits:
        i -= 1
    letters, num = text[:i], text[i:]
    return letters, int(num, radix) if num else 0, len(num)


def address(base: str, offset: int, radix: int = 10, word_bits: Optional[int] = 8) -> str:
    """`base` counted up by `offset` bits.

    * `"%IX0.0"` / `"I0.0"` → `%IX0.7`, `%IX1.0`, … — the part after the
      last `.` is the bit, the number before it the byte, `word_bits` bits
      to a byte (8 for IEC / Siemens; `None` for a flat index such as
      Logix `Local:1:I.Data.0` … `.31`).
    * `"X10"` with `radix=16` → `X10 … X1F, X20` (MELSEC Q / iQ-R);
      `radix=8` → `X10 … X17, X20` (MELSEC FX). Digit count is kept
      (`X000` → `X00F`).
    * anything else → decimal suffix counting up.
    """
    if "." in base:
        head, bit = base.rsplit(".", 1)
        bit_no = int(bit) + offset
        if word_bits is None:
            return f"{head}.{bit_no}"
        letters, byte, width = _split_number(head, 10)
        return f"{letters}{byte + bit_no // word_bits:0{width}d}.{bit_no % word_bits}"
    letters, value, width = _split_number(base, radix)
    value += offset
    if radix == 16:
        num = f"{value:0{width}X}"
    elif radix == 8:
        num = f"{value:0{width}o}"
    else:
        num = f"{value:0{width}d}"
    return f"{letters}{num}"


def channels(
    kind: str,
    count: int,
    prefix: Optional[str] = None,
    base: Optional[str] = None,
    port_from: Optional[int] = None,
    voltage: Optional[float] = None,
    logic: Optional[str] = None,
    radix: int = 10,
    word_bits: Optional[int] = 8,
    unit: Optional[str] = None,
) -> list:
    """`count` channels of `kind` (`"di"`, `"do"`, `"ai"`, `"ao"`,
    `"safe_di"`, `"safe_do"`, `"word"`), ids `{prefix}{n}` (prefix defaults
    to the kind in upper case), addresses counted up from `base` (see
    `address` for the dialects `radix` / `word_bits` select), ports from
    `port_from`.

    `unit` (word channels) is the engineering unit the register carries a
    measurement in — `"mm"`, `"0.01mm"`, `"mm/s"`: a REAL point bound to
    the channel (an axis position, a speed, an encoder) is exchanged as an
    INT scaled to it, which is what a 16-bit register holds and how a
    drive takes a position anyway. Without a unit the word carries the
    point's own type."""
    prefix = prefix if prefix is not None else kind.upper().replace("SAFE_", "S")
    out = []
    for n in range(count):
        ch: Channel = {"id": f"{prefix}{n}", "kind": kind}
        if base is not None:
            ch["address"] = address(base, n, radix, word_bits)
        if port_from is not None:
            ch["port"] = port_from + n
        if voltage is not None:
            ch["voltage"] = voltage
        if logic is not None:
            ch["logic"] = logic
        if unit is not None:
            ch["unit"] = unit
        out.append(ch)
    return out


def di8(base: Optional[str] = None, **kw) -> list:
    return channels("di", 8, "DI", base, **kw)


def do8(base: Optional[str] = None, **kw) -> list:
    return channels("do", 8, "DO", base, **kw)


def di16(base: Optional[str] = None, **kw) -> list:
    return channels("di", 16, "DI", base, **kw)


def do16(base: Optional[str] = None, **kw) -> list:
    return channels("do", 16, "DO", base, **kw)


def safe_di8(base: Optional[str] = None, **kw) -> list:
    return channels("safe_di", 8, "SDI", base, **kw)


def word(count: int = 4, base: Optional[str] = None, unit: Optional[str] = None, **kw) -> list:
    """`count` word (register) channels `W0…`; `unit="mm"` makes them
    carry measurements as scaled INTs (see `channels`)."""
    return channels("word", count, "W", base, unit=unit, **kw)


def ao(count: int = 2, base: Optional[str] = None, **kw) -> list:
    return channels("ao", count, "AO", base, **kw)


# ---- vendor spellings: the same channels, addressed the way the PLC
# programmer will type them.


def melsec(kind: str, count: int = 16, base: Optional[str] = None, octal: bool = False, **kw) -> list:
    """MELSEC X (inputs) / Y (outputs): `melsec("di", 16, "X10")` → X10 …
    X1F on the Q / iQ-R series (hex), `octal=True` for the FX series (X10
    … X17, X20 …). `base` defaults to `X0` / `Y0`."""
    if base is None:
        base = "X0" if kind in ("di", "safe_di") else "Y0" if kind in ("do", "safe_do") else "D0"
    return channels(kind, count, base=base, radix=8 if octal else 16, **kw)


def siemens(kind: str, count: int = 16, base: Optional[str] = None, **kw) -> list:
    """S7 `I0.0` / `Q0.0` byte.bit addressing (eight bits to a byte)."""
    if base is None:
        base = "I0.0" if kind in ("di", "safe_di") else "Q0.0" if kind in ("do", "safe_do") else "MW0"
    return channels(kind, count, base=base, word_bits=8, **kw)


def logix(kind: str, count: int = 16, base: Optional[str] = None, **kw) -> list:
    """Logix module tags: `Local:1:I.Data.0` … `.15` — a flat bit index,
    no byte rollover."""
    if base is None:
        base = "Local:1:I.Data.0" if kind in ("di", "safe_di") else "Local:2:O.Data.0"
    return channels(kind, count, base=base, word_bits=None, **kw)


def ur_standard() -> list:
    """A UR controller's standard digital I/O: DI0-7 and DO0-7, ports 0-7
    — the numbers `set_standard_digital_out` / `get_standard_digital_in`
    take."""
    return channels("di", 8, "DI", port_from=0) + channels("do", 8, "DO", port_from=0)


def fanuc_uop() -> list:
    """A FANUC controller's **UOP** (user operator panel) interface: the
    standard assignment UI[1]–UI[18] / UO[1]–UO[20] as channels `UI1`…`UO20`,
    plus `PNS` — the program-number byte PNS1–8 (UI[9]–UI[16]) as one word
    channel — and `SNO` (UO[11]–UO[18]) echoing it. Names, polarity and
    edges are the template's (`bt.vc.interfaces.fanuc_uop_pns`); the
    numbers are the controller's standard UOP map — check them against the
    controller's I/O configuration screen."""
    ins = channels("di", 18, "UI", port_from=1)
    outs = channels("do", 20, "UO", port_from=1)
    # `channels` numbers ids from 0; the UOP numbering starts at 1.
    for ch in ins + outs:
        ch["id"] = ch["id"][:2] + str(ch["port"])
    return ins + [{"id": "PNS", "kind": "word"}] + outs + [{"id": "SNO", "kind": "word"}]


def kuka_automatic_external() -> list:
    """A KUKA controller's **Automatic External** interface (KRC4 / KRC5)
    as channels named after its signals: inputs `PGNO_VALID`, `EXT_START`,
    `MOVE_ENABLE`, `CONF_MESS`, `DRIVES_ON`, `DRIVES_OFF`, `I_O_ACT`, the
    program number `PGNO` as a word, and outputs `RC_RDY1`, `ALARM_STOP`,
    `USER_SAF`, `PERI_RDY`, `ROB_CAL`, `I_O_ACTCONF`, `STOPMESS`, `PGNO_REQ`,
    `APPL_RUN`, `PRO_ACT`, `IN_HOME`, `ON_PATH`, `EXT`. The addresses are
    the integrator's (`$IN[n]` / `$OUT[n]` in the WorkVisual mapping)."""
    ins = ["PGNO_VALID", "EXT_START", "MOVE_ENABLE", "CONF_MESS", "DRIVES_ON", "DRIVES_OFF", "I_O_ACT"]
    outs = ["RC_RDY1", "ALARM_STOP", "USER_SAF", "PERI_RDY", "ROB_CAL", "I_O_ACTCONF", "STOPMESS", "PGNO_REQ",
            "APPL_RUN", "PRO_ACT", "IN_HOME", "ON_PATH", "EXT"]
    return ([{"id": n, "kind": "di"} for n in ins] + [{"id": "PGNO", "kind": "word"}]
            + [{"id": n, "kind": "do"} for n in outs])


def abb_system_io() -> list:
    """An ABB controller's **System I/O** (IRC5 / OmniCore) as channels
    named after the system actions and statuses: inputs `MotorsOn`,
    `MotorsOff`, `Start`, `StartAtMain`, `Stop`, `QuickStop`,
    `StopAtEndOfCycle`, `ResetEmergencyStop`, `ResetExecutionError`,
    `PPToMain`, the program number `ProgNo` as a word (a group input the
    RAPID main reads — not a system I/O), and outputs `AutoOn`, `CycleOn`,
    `EmergencyStop`, `ExecutionError`, `MotorsOnState`, `MotorsOffState`,
    `RunChainOk`, `TaskExecuting`, `MotionSupervisionTriggered`. Each is a
    signal the integrator ties to a system action in the controller's I/O
    configuration."""
    ins = ["MotorsOn", "MotorsOff", "Start", "StartAtMain", "Stop", "QuickStop", "StopAtEndOfCycle",
           "ResetEmergencyStop", "ResetExecutionError", "PPToMain"]
    outs = ["AutoOn", "CycleOn", "EmergencyStop", "ExecutionError", "MotorsOnState", "MotorsOffState",
            "RunChainOk", "TaskExecuting", "MotionSupervisionTriggered"]
    return ([{"id": n, "kind": "di"} for n in ins] + [{"id": "ProgNo", "kind": "word"}]
            + [{"id": n, "kind": "do"} for n in outs])


def ur_registers(first: int = 64, count: int = 32) -> list:
    """A UR controller's general-purpose **bit registers** as channels —
    the ones an external RTDE client reads and writes (`first` from 64:
    registers 0-63 belong to the fieldbus interfaces). Input registers
    `IR64…` are what the robot program reads (`read_input_boolean_register`,
    the plant's sensors and handshakes), output registers `OR64…` what it
    writes (`write_output_boolean_register`, its coils). Bind the cell's
    points on them and export the program with `rtde=True`; a
    `bt.vc.ursim` link then carries them over RTDE."""
    if first < 0 or first + count > 128:
        raise ValueError("ur_registers: bit registers run 0-127 (64-127 for RTDE clients)")
    out = channels("di", count, "IR", port_from=first) + channels("do", count, "OR", port_from=first)
    for ch in out:  # ids carry the register number, not a count from zero
        ch["id"] = ch["id"][:2] + str(ch["port"])
    return out


def merge(*groups: Iterable[Channel]) -> list:
    """Concatenates channel lists (the same as `+`)."""
    out: list = []
    for g in groups:
        out.extend(g)
    return out


# ---------------------------------------------------------------- catalog

_CHANNEL_KEYS = ("id", "kind", "port", "address", "voltage", "logic")


def from_catalog(ref, revision: Optional[str] = None) -> list:
    """The channels a catalog product declares — a controller's, a remote
    I/O station's, a robot controller's — read from its package manifest
    (`electrical.io`, the catalog's electrical layer). `ref` is a catalog id
    (downloaded like `Robot.from_catalog`), an `(id, revision)` pair, or a
    path to a package directory. A manifest that names a known template
    instead of listing channels (`standard: ur`) gets that template, so
    `bt.io.ur_standard()` and the UR5e package agree.

        scene.add_io_node("UR", kind="robot_controller", robots=["arm"],
                          channels=bt.io.from_catalog("universal_robots/ur/ur5e/r2"))
    """
    manifest = _catalog_manifest(ref, revision)
    io = (manifest.get("electrical") or {}).get("io") or {}
    rows = io.get("channels") or []
    if rows:
        return [{k: row[k] for k in _CHANNEL_KEYS if row.get(k) is not None} for row in rows]
    standard = io.get("standard")
    if standard == "ur":
        return ur_standard()
    raise ValueError(
        f"{manifest.get('id', ref)}: no electrical.io.channels in the manifest"
        + (f" and no template for standard {standard!r}" if standard else "")
    )


def electrical(ref, revision: Optional[str] = None) -> dict:
    """The whole `electrical:` section of a catalog product's manifest —
    supply, output, io, bus, connector, power — as plain dicts."""
    return dict(_catalog_manifest(ref, revision).get("electrical") or {})


def _catalog_manifest(ref, revision: Optional[str]) -> dict:
    id_or_path, rev = ref if isinstance(ref, tuple) else (ref, revision)
    path = Path(str(id_or_path))
    if path.is_dir():
        directory = path
    elif path.is_file():
        directory = path.parent
    else:
        from . import catalog_package

        directory = Path(catalog_package(str(id_or_path), revision=rev))
    import yaml

    data = yaml.safe_load((directory / "manifest.yaml").read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{directory}: manifest.yaml is not a mapping")
    return data


# ------------------------------------------------------------------ faults


def stuck(name: str, value: bool = False) -> dict:
    """A contact stuck at `value` for the whole run: the sensor's geometry
    is ignored, a program's `set` on the internal signal is dropped.
    Pass in `scene.add_scenario(name, faults=[...])`."""
    return {"target": name, "kind": "stuck", "value": bool(value)}


def node_down(name: str) -> dict:
    """An I/O node — a controller or a station (`add_io_node`) — dropped
    off the bus: every sensor or signal input wired on it (and on the
    stations uplinked to it) reads as an open wire, each with its own
    binding's polarity. Resolved through the bindings when the scenario is
    applied, so wire first."""
    return {"target": name, "kind": "node_down"}


def open(name: str) -> dict:  # noqa: A001 — the electrical term
    """An open wire on an input: the level is low, so the functional
    value follows the point's binding — `False` on a normally-open wiring,
    `True` on an inverted (NC) one, `False` when unbound. Whether the
    cell fails safe under the break is what the run then shows."""
    return {"target": name, "kind": "open"}


# ------------------------------------------------ external sheets and diffs

#: The columns that make a point's identity in a sheet.
KEY_COLUMNS = ("name", "aspect", "direction")
#: The wiring columns a diff compares when both sides carry them.
WIRING_COLUMNS = ("kind", "host", "node", "channel", "address", "tag", "field", "contact", "invert", "safety")


def read_io_list(path: Union[str, Path]) -> list:
    """Reads an I/O sheet: the `.csv` / `.json` botrail writes with
    `export_io_list`, or any CSV with a `name` column (the electrical
    designer's own sheet — extra columns are kept, missing ones are simply
    not compared). Rows come back as dicts of strings; `#` comment lines
    (the count footer) are skipped."""
    path = Path(path)
    if path.suffix.lower() == ".json":
        doc = json.loads(path.read_text(encoding="utf-8"))
        rows = doc["points"] if isinstance(doc, dict) else doc
        return [{k: _cell(v) for k, v in r.items()} for r in rows]
    with path.open(encoding="utf-8", newline="") as f:
        lines = [l for l in f if not l.startswith("#")]
    reader = csv.DictReader(lines)
    if reader.fieldnames is None or "name" not in reader.fieldnames:
        raise ValueError(f"{path}: an I/O sheet needs a `name` column")
    return [{k: (v or "") for k, v in r.items() if k is not None} for r in reader]


def _cell(v) -> str:
    if v is None:
        return ""
    if isinstance(v, bool):
        return "yes" if v else ""
    if isinstance(v, (list, dict)):
        return json.dumps(v)
    return str(v)


def _key(row: dict) -> tuple:
    return tuple((row.get(k) or "") for k in KEY_COLUMNS)


@dataclass
class IoDiff:
    """The difference between the cell's derived I/O list and a sheet.

    * `added` — points the cell needs that the sheet does not list;
    * `removed` — sheet rows the cell no longer derives;
    * `changed` — `(key, {column: (cell, sheet)})` where a wiring column
      differs (only columns both sides carry are compared).
    """

    added: list = field(default_factory=list)
    removed: list = field(default_factory=list)
    changed: list = field(default_factory=list)
    columns: tuple = ()

    @property
    def ok(self) -> bool:
        return not (self.added or self.removed or self.changed)

    def __bool__(self) -> bool:  # `if diff:` reads as "there are differences"
        return not self.ok

    def __str__(self) -> str:
        if self.ok:
            return "I/O sheet matches the cell"
        lines = []
        for r in self.added:
            lines.append(f"+ {'.'.join(x for x in _key(r) if x)}: in the cell, not on the sheet")
        for r in self.removed:
            lines.append(f"- {'.'.join(x for x in _key(r) if x)}: on the sheet, not in the cell")
        for key, cols in self.changed:
            what = ", ".join(f"{c}: {ours!r} → {theirs!r}" for c, (ours, theirs) in cols.items())
            lines.append(f"~ {'.'.join(x for x in key if x)}: {what}")
        return "\n".join(lines)


def diff(scene, sheet: Union[str, Path, list], sequences: Optional[list] = None,
         columns: Optional[Iterable[str]] = None, include_cosmetic: bool = False) -> IoDiff:
    """Compares the cell's derived I/O list with a sheet (a path, or rows
    from `read_io_list`): what the sheet lacks, what it still lists, and
    where a wiring column disagrees. Keyed by `(name, aspect, direction)`
    — plus `host` when the sheet carries it, since a handshake signal is
    one row per controller.

        d = bt.io.diff(scene, "electrical/pick_cell_io.csv")
        assert d.ok, d
    """
    ours = json.loads(scene.io_list("json", sequences=sequences))["points"]
    ours = [{k: _cell(v) for k, v in r.items()} for r in ours]
    if not include_cosmetic:
        ours = [r for r in ours if r.get("status") != "cosmetic"]
    theirs = read_io_list(sheet) if isinstance(sheet, (str, Path)) else list(sheet)
    if not theirs:
        their_cols: set = set()
    else:
        their_cols = set().union(*(r.keys() for r in theirs))
    keyed = "host" in their_cols

    def key(r: dict) -> tuple:
        k = _key(r)
        return k + ((r.get("host") or ""),) if keyed else k

    compare = tuple(columns) if columns is not None else tuple(
        c for c in WIRING_COLUMNS if c in their_cols and c != "host"
    )
    ours_by = {key(r): r for r in ours}
    theirs_by = {key(r): r for r in theirs}
    out = IoDiff(columns=compare)
    for k, r in ours_by.items():
        if k not in theirs_by:
            out.added.append(r)
    for k, r in theirs_by.items():
        if k not in ours_by:
            out.removed.append(r)
    for k, r in ours_by.items():
        t = theirs_by.get(k)
        if t is None:
            continue
        cols = {}
        for c in compare:
            a, b = (r.get(c) or ""), (t.get(c) or "")
            if a != b:
                cols[c] = (a, b)
        if cols:
            out.changed.append((k, cols))
    return out


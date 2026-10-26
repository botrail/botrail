"""Network links: the plant as a field device on the wire (VC2).

A link carries one host's exchange over a protocol while the session runs
the plant in wall-clock time. The plant is the **slave / server** where the
real cell's I/O would be — a PLC polls it as it would a remote I/O station
— and the **client** where the controller offers a server (OPC UA):

* [`ModbusServerLink`] — botrail is a Modbus TCP server. The controller
  reads the plant's outputs as discrete inputs (`BOOL`) and input registers
  (`INT`, `REAL` as two registers) and writes the plant's inputs as coils
  and holding registers; the numbering follows the exchange's IEC
  addresses (`%IX100.3` → discrete input 3 with `iec_base=100`), unbound
  points are appended.
* [`OpcUaClientLink`] — botrail is an OPC UA client of the controller's
  server (PLCSIM Advanced, TwinCAT, CODESYS, an OpenPLC plugin): it writes
  the plant's outputs to the nodes the controller reads, and subscribes to
  the nodes the controller writes.
* [`UrRtdeLink`] — botrail is the RTDE client of a Universal Robots
  controller (URSim, or one on the bench) whose program runs *on the
  controller*: the program's I/O crosses the bit registers, and its joints
  stream into the plant, which mirrors the arm (VC5).

Both run their I/O on a background thread and hand the session the latest
image on each scan — a controller's write lands at the next plant tick, as
it does on a real bus. The session records every write, so the run replays
exactly without the link (`bt.vc.replay`).

The Modbus server is botrail's own (no library); the OPC UA client needs
`asyncua` — `pip install "botrail[vc]"`.
"""

from __future__ import annotations

import asyncio
import math
import re
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Union

Value = Union[bool, int, float]

__all__ = ["ModbusServerLink", "OpcUaClientLink", "RegisterRow", "UrDashboard", "UrRtdeLink", "modbus_server",
           "opcua_client", "ursim"]


# ------------------------------------------------------------ register map


_IEC = re.compile(r"^%([IQM])([XWDL]?)(\d+)(?:\.(\d+))?$")


def _iec(address: Optional[str]):
    """`(memory, size, byte_or_word, bit)` of an IEC location, or None."""
    if not address:
        return None
    m = _IEC.match(address.strip())
    if not m:
        return None
    memory, size, index, bit = m.groups()
    return memory, (size or "X"), int(index), (int(bit) if bit is not None else None)


@dataclass
class RegisterRow:
    """One row of a Modbus register map."""

    label: str
    var: str
    table: str  # "discrete_input" | "coil" | "input_register" | "holding_register"
    address: int
    count: int
    type: str  # BOOL / INT / REAL
    iec: Optional[str]

    def __iter__(self):
        yield from (self.label, self.var, self.table, self.address, self.count, self.type, self.iec or "")


def _to_regs(value: Value, ty: str, word_order: str) -> list[int]:
    if ty == "REAL":
        raw = struct.unpack(">HH", struct.pack(">f", float(value)))
        return list(raw) if word_order == "big" else list(reversed(raw))
    v = int(round(float(value))) if not isinstance(value, bool) else int(value)
    return [v & 0xFFFF]


def _from_regs(regs: list[int], ty: str, word_order: str) -> Value:
    if ty == "REAL":
        hi, lo = (regs[0], regs[1]) if word_order == "big" else (regs[1], regs[0])
        return struct.unpack(">f", struct.pack(">HH", hi, lo))[0]
    v = regs[0] & 0xFFFF
    return v - 0x10000 if v >= 0x8000 else v


class ModbusServerLink:
    """botrail as a Modbus TCP server — the virtual remote I/O station the
    controller polls. `exchange` is the host's (`bt.vc.exchange`), `bind`
    the address to listen on (`"127.0.0.1:5020"`; a non-loopback address
    needs `allow_remote=True` — the plant is for a controller under test,
    not a plant floor). `iec_base` is the IEC byte / word the controller's
    image starts at (OpenPLC v3 maps a slave device from `%IX100.0`,
    `%QX100.0`, `%IW100`, `%QW100`); points bound outside that image, or
    unbound, are appended in exchange order.

    The server is botrail's own (function codes 1–6, 15, 16 over the MBAP
    framing): a field device's register image, no library behind it."""

    _TABLES = ("discrete_input", "coil", "input_register", "holding_register")

    def __init__(
        self,
        exchange,
        bind: str = "127.0.0.1:5020",
        *,
        iec_base: int = 100,
        word_order: str = "big",
        unit: int = 1,
        allow_remote: bool = False,
    ):
        host, _, port = bind.rpartition(":")
        host = host or "127.0.0.1"
        if host not in ("127.0.0.1", "localhost", "::1") and not allow_remote:
            raise ValueError(
                f"modbus_server: binding {bind!r} reaches beyond this machine — pass allow_remote=True "
                "for a controller on the bench (never a plant in production)"
            )
        self.xc = exchange
        self.bind = (host, int(port or 502))
        self.unit = unit
        self.word_order = word_order
        self.rows: list[RegisterRow] = []
        self._by_label: dict[str, RegisterRow] = {}
        self._build_map(iec_base)
        self._lock = threading.Lock()
        self._image: dict[str, list[int]] = {t: [0] * self._size(t) for t in self._TABLES}
        self._thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._server: Optional[asyncio.AbstractServer] = None
        self._ready = threading.Event()
        self._error: Optional[BaseException] = None
        self._last_inputs: dict[str, Value] = {}
        #: Requests answered so far (reads and writes).
        self.polls = 0
        for p in self.xc.points:
            row = self._by_label[p.label]
            if row.table in ("coil", "holding_register"):
                self._write(row, p.initial)
                self._last_inputs[p.label] = p.initial

    # -- the map --------------------------------------------------------------

    def _build_map(self, base: int) -> None:
        points = list(self.xc.points)
        tables = {t: {} for t in self._TABLES}
        pending = []
        for p in points:
            ty = p.value_type
            if p.side == "output":  # the controller reads
                table = "discrete_input" if ty == "BOOL" else "input_register"
            else:
                table = "coil" if ty == "BOOL" else "holding_register"
            count = 2 if ty == "REAL" else 1
            loc = _iec(p.address)
            addr = None
            if loc is not None:
                memory, size, index, bit = loc
                if ty == "BOOL" and size == "X" and bit is not None and index >= base:
                    addr = (index - base) * 8 + bit
                elif ty != "BOOL" and size in ("W", "D") and bit is None and index >= base:
                    addr = index - base
            row = RegisterRow(p.label, p.var, table, -1, count, ty, p.address)
            if addr is not None and all(
                not (a <= addr < a + r.count or addr <= a < addr + count) for a, r in tables[table].items()
            ):
                row.address = addr
                tables[table][addr] = row
            else:
                pending.append(row)
            self.rows.append(row)
        for row in pending:
            used = tables[row.table]
            addr = 0
            while any(a <= addr < a + r.count or addr <= a < addr + row.count for a, r in used.items()):
                addr += 1
            row.address = addr
            used[addr] = row
        self.rows.sort(key=lambda r: (r.table, r.address))
        self._by_label = {r.label: r for r in self.rows}

    def register_map(self) -> list[RegisterRow]:
        """The register map, by table and address."""
        return list(self.rows)

    def register_map_markdown(self) -> str:
        lines = [
            "| point | variable | table | address | count | type | IEC |",
            "|---|---|---|---|---|---|---|",
        ]
        for r in self.rows:
            lines.append(f"| `{r.label}` | `{r.var}` | {r.table} | {r.address} | {r.count} | {r.type} | {r.iec or ''} |")
        return "\n".join(lines) + "\n"

    def _size(self, table: str) -> int:
        rows = [r for r in self.rows if r.table == table]
        return max((r.address + r.count for r in rows), default=0) + 8

    # -- the image ------------------------------------------------------------

    def _write(self, row: RegisterRow, value: Value) -> None:
        with self._lock:
            table = self._image[row.table]
            if row.type == "BOOL":
                table[row.address] = 1 if value else 0
            else:
                for i, reg in enumerate(_to_regs(value, row.type, self.word_order)):
                    table[row.address + i] = reg

    def _read(self, row: RegisterRow) -> Value:
        with self._lock:
            regs = self._image[row.table][row.address:row.address + row.count]
        if row.type == "BOOL":
            return bool(regs[0])
        return _from_regs(list(regs), row.type, self.word_order)

    # -- the server -----------------------------------------------------------

    def start(self) -> "ModbusServerLink":
        if self._thread is not None:
            return self

        def run() -> None:
            loop = asyncio.new_event_loop()
            self._loop = loop
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(self._serve())
            except BaseException as e:  # noqa: BLE001 - reported to the session
                self._error = e
                self._ready.set()
            finally:
                loop.close()

        self._thread = threading.Thread(target=run, name="botrail-modbus", daemon=True)
        self._thread.start()
        if not self._ready.wait(5.0):
            raise RuntimeError("modbus_server: the server did not come up within 5 s")
        if self._error is not None:
            raise RuntimeError(f"modbus_server: {self._error}") from self._error
        return self

    async def _serve(self) -> None:
        self._server = await asyncio.start_server(self._client, self.bind[0], self.bind[1])
        self._stopping = asyncio.Event()
        self._ready.set()
        await self._stopping.wait()
        self._server.close()
        await self._server.wait_closed()

    def stop(self) -> None:
        if self._loop is not None and self._server is not None:
            try:
                self._loop.call_soon_threadsafe(self._stopping.set)
            except RuntimeError:
                pass
        if self._thread is not None:
            self._thread.join(5.0)
        self._thread = None
        self._server = None

    @property
    def address(self) -> tuple[str, int]:
        return self.bind

    def describe(self) -> str:
        """`modbus 127.0.0.1:5020` — the link as the studio's dock names it."""
        return f"modbus {self.bind[0]}:{self.bind[1]}"

    async def _client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        try:
            while True:
                head = await reader.readexactly(7)
                tid, pid, length, unit = struct.unpack(">HHHB", head)
                pdu = await reader.readexactly(max(length - 1, 0))
                if not pdu:
                    continue
                reply = self._handle(pdu)
                writer.write(struct.pack(">HHHB", tid, pid, len(reply) + 1, unit) + reply)
                await writer.drain()
                self.polls += 1
        except (asyncio.IncompleteReadError, ConnectionError):
            pass
        finally:
            writer.close()

    def _handle(self, pdu: bytes) -> bytes:
        fc = pdu[0]

        def exception(code: int) -> bytes:
            return bytes([fc | 0x80, code])

        try:
            if fc in (1, 2):
                addr, qty = struct.unpack(">HH", pdu[1:5])
                table = "coil" if fc == 1 else "discrete_input"
                if not 1 <= qty <= 2000:
                    return exception(3)
                with self._lock:
                    image = self._image[table]
                    if addr + qty > len(image):
                        return exception(2)
                    bits = image[addr:addr + qty]
                out = bytearray((qty + 7) // 8)
                for i, b in enumerate(bits):
                    if b:
                        out[i // 8] |= 1 << (i % 8)
                return bytes([fc, len(out)]) + bytes(out)
            if fc in (3, 4):
                addr, qty = struct.unpack(">HH", pdu[1:5])
                table = "holding_register" if fc == 3 else "input_register"
                if not 1 <= qty <= 125:
                    return exception(3)
                with self._lock:
                    image = self._image[table]
                    if addr + qty > len(image):
                        return exception(2)
                    regs = image[addr:addr + qty]
                return bytes([fc, 2 * qty]) + b"".join(struct.pack(">H", r & 0xFFFF) for r in regs)
            if fc == 5:
                addr, value = struct.unpack(">HH", pdu[1:5])
                if value not in (0x0000, 0xFF00):
                    return exception(3)
                with self._lock:
                    image = self._image["coil"]
                    if addr >= len(image):
                        return exception(2)
                    image[addr] = 1 if value else 0
                return pdu[:5]
            if fc == 6:
                addr, value = struct.unpack(">HH", pdu[1:5])
                with self._lock:
                    image = self._image["holding_register"]
                    if addr >= len(image):
                        return exception(2)
                    image[addr] = value
                return pdu[:5]
            if fc == 15:
                addr, qty, nbytes = struct.unpack(">HHB", pdu[1:6])
                data = pdu[6:6 + nbytes]
                if not 1 <= qty <= 1968 or len(data) < (qty + 7) // 8:
                    return exception(3)
                with self._lock:
                    image = self._image["coil"]
                    if addr + qty > len(image):
                        return exception(2)
                    for i in range(qty):
                        image[addr + i] = (data[i // 8] >> (i % 8)) & 1
                return pdu[:5]
            if fc == 16:
                addr, qty, nbytes = struct.unpack(">HHB", pdu[1:6])
                data = pdu[6:6 + nbytes]
                if not 1 <= qty <= 123 or len(data) < 2 * qty:
                    return exception(3)
                with self._lock:
                    image = self._image["holding_register"]
                    if addr + qty > len(image):
                        return exception(2)
                    for i in range(qty):
                        image[addr + i] = struct.unpack(">H", data[2 * i:2 * i + 2])[0]
                return pdu[:5]
            return exception(1)
        except struct.error:
            return exception(3)

    def exchange(self, host: str, t: float, outputs: dict[str, Value]) -> dict[str, Value]:  # type: ignore[override]
        """Publishes the plant's outputs to the controller's image and
        returns the inputs the controller has written since the last scan."""
        if self._thread is None:
            self.start()
        for label, value in outputs.items():
            row = self._by_label.get(label)
            if row is not None and row.table in ("discrete_input", "input_register"):
                self._write(row, value)
        writes: dict[str, Value] = {}
        for row in self.rows:
            if row.table not in ("coil", "holding_register"):
                continue
            value = self._read(row)
            if self._last_inputs.get(row.label) != value:
                self._last_inputs[row.label] = value
                writes[row.label] = value
        return writes

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()


def modbus_server(exchange, bind: str = "127.0.0.1:5020", **kw) -> ModbusServerLink:
    """botrail as the Modbus TCP server a controller polls (see
    [`ModbusServerLink`])."""
    return ModbusServerLink(exchange, bind, **kw)


# --------------------------------------------------------------- OPC UA


class OpcUaClientLink:
    """botrail as an OPC UA client of the controller's server. `nodes` maps
    a point's label to its NodeId string (`"ns=4;s=GVL.beam"`), or is a
    callable `label -> NodeId`; by default every point is looked up as
    `ns=<namespace>;s=<tag or variable>`. The plant's outputs are written
    every `period_ms`; the controller's writes are taken from a data-change
    subscription."""

    def __init__(
        self,
        exchange,
        url: str,
        *,
        nodes: Union[dict[str, str], Callable[[str], str], None] = None,
        namespace: int = 2,
        period_ms: int = 50,
        allow_remote: bool = False,
    ):
        host = re.sub(r"^opc\.tcp://", "", url).split("/")[0].rsplit(":", 1)[0]
        if host not in ("127.0.0.1", "localhost", "::1", "[::1]") and not allow_remote:
            raise ValueError(
                f"opcua_client: {url!r} reaches beyond this machine — pass allow_remote=True for a "
                "controller on the bench (never a plant in production)"
            )
        self.xc = exchange
        self.url = url
        self.period_ms = period_ms
        self.points = {p.label: p for p in exchange.points}

        def default(label: str) -> str:
            p = self.points[label]
            return f"ns={namespace};s={p.tag or p.var}"

        if nodes is None:
            self._node = default
        elif callable(nodes):
            self._node = nodes
        else:
            self._node = lambda label: nodes.get(label) or default(label)
        self._thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._error: Optional[BaseException] = None
        self._lock = threading.Lock()
        self._inbox: dict[str, Value] = {}
        self._outbox: dict[str, Value] = {}
        self._last: dict[str, Value] = {}
        #: Scans exchanged so far.
        self.exchanges = 0

    def describe(self) -> str:
        """`opcua opc.tcp://…` — the link as the studio's dock names it."""
        return f"opcua {self.url}"

    def start(self) -> "OpcUaClientLink":
        if self._thread is not None:
            return self

        def run() -> None:
            loop = asyncio.new_event_loop()
            self._loop = loop
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(self._main())
            except BaseException as e:  # noqa: BLE001
                self._error = e
                self._ready.set()
            finally:
                loop.close()

        self._thread = threading.Thread(target=run, name="botrail-opcua", daemon=True)
        self._thread.start()
        if not self._ready.wait(10.0):
            raise RuntimeError(f"opcua_client: no connection to {self.url} within 10 s")
        if self._error is not None:
            raise RuntimeError(f"opcua_client: {self._error}") from self._error
        return self

    async def _main(self) -> None:
        from asyncua import Client, ua

        link = self

        class Handler:
            def datachange_notification(self, node, val, data):
                label = labels.get(node.nodeid.to_string())
                if label is not None:
                    with link._lock:
                        link._inbox[label] = _coerce(val, link.points[label].value_type)

        inputs = [p for p in self.xc.points if p.side == "input"]
        outputs = [p for p in self.xc.points if p.side == "output"]
        async with Client(url=self.url) as client:
            in_nodes = {p.label: client.get_node(self._node(p.label)) for p in inputs}
            out_nodes = {p.label: client.get_node(self._node(p.label)) for p in outputs}
            labels = {n.nodeid.to_string(): label for label, n in in_nodes.items()}
            # The controller's current image, then its changes.
            for label, node in in_nodes.items():
                val = await node.read_value()
                with self._lock:
                    self._inbox[label] = _coerce(val, self.points[label].value_type)
            sub = await client.create_subscription(self.period_ms, Handler())
            if in_nodes:
                await sub.subscribe_data_change(list(in_nodes.values()))
            self._ready.set()
            types = {}
            for label, node in out_nodes.items():
                try:
                    types[label] = await node.read_data_type_as_variant_type()
                except Exception:  # noqa: BLE001 - fall back on the point's type
                    types[label] = None
            while not self._stop.is_set():
                with self._lock:
                    pending = dict(self._outbox)
                    self._outbox.clear()
                for label, value in pending.items():
                    node = out_nodes.get(label)
                    if node is None:
                        continue
                    ty = types.get(label)
                    if ty is None:
                        ty = {"BOOL": ua.VariantType.Boolean, "INT": ua.VariantType.Int16,
                              "REAL": ua.VariantType.Float}[self.points[label].value_type]
                    await node.write_value(ua.DataValue(ua.Variant(_cast(value, ty), ty)))
                await asyncio.sleep(self.period_ms / 1000.0)
            await sub.delete()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(10.0)
        self._thread = None

    def exchange(self, host: str, t: float, outputs: dict[str, Value]) -> dict[str, Value]:  # type: ignore[override]
        if self._thread is None:
            self.start()
        if self._error is not None:
            raise RuntimeError(f"opcua_client: {self._error}") from self._error
        with self._lock:
            for label, value in outputs.items():
                if self._last.get(label) != value:
                    self._last[label] = value
                    self._outbox[label] = value
            writes = dict(self._inbox)
            self._inbox.clear()
        self.exchanges += 1
        return writes

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()


def _coerce(value, ty: str) -> Value:
    if ty == "BOOL":
        return bool(value)
    if ty == "INT":
        return int(value)
    return float(value) if value is not None and not (isinstance(value, float) and math.isnan(value)) else 0.0


def _cast(value: Value, variant_type):
    from asyncua import ua

    if variant_type == ua.VariantType.Boolean:
        return bool(value)
    if variant_type in (ua.VariantType.Int16, ua.VariantType.Int32, ua.VariantType.Int64,
                        ua.VariantType.UInt16, ua.VariantType.UInt32, ua.VariantType.UInt64,
                        ua.VariantType.Byte, ua.VariantType.SByte):
        return int(round(float(value)))
    return float(value)


def opcua_client(exchange, url: str, **kw) -> OpcUaClientLink:
    """botrail as an OPC UA client of the controller's server (see
    [`OpcUaClientLink`])."""
    return OpcUaClientLink(exchange, url, **kw)


# ------------------------------------------------------------ UR / RTDE


# RTDE package types (Universal Robots, Real-Time Data Exchange, protocol v2).
_RTDE_REQUEST_PROTOCOL_VERSION = 86  # 'V'
_RTDE_TEXT_MESSAGE = 77  # 'M'
_RTDE_DATA_PACKAGE = 85  # 'U'
_RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79  # 'O'
_RTDE_CONTROL_PACKAGE_SETUP_INPUTS = 73  # 'I'
_RTDE_CONTROL_PACKAGE_START = 83  # 'S'
_RTDE_CONTROL_PACKAGE_PAUSE = 80  # 'P'

_RTDE_FORMATS = {
    "BOOL": "?", "UINT8": "B", "UINT32": "I", "UINT64": "Q", "INT32": "i", "DOUBLE": "d",
    "VECTOR3D": "3d", "VECTOR6D": "6d", "VECTOR6INT32": "6i", "VECTOR6UINT32": "6I",
}

#: `runtime_state` values of the RTDE output: the controller's program state.
RUNTIME_STATE = {0: "stopping", 1: "stopped", 2: "playing", 3: "pausing", 4: "paused", 5: "resuming", 6: "retracting"}


def _rtde_struct(types: list[str]) -> struct.Struct:
    return struct.Struct(">" + "".join(_RTDE_FORMATS[t] for t in types))


def _rtde_unpack(st: struct.Struct, types: list[str], payload: bytes) -> list:
    raw = list(st.unpack(payload))
    out = []
    for t in types:
        n = 6 if t.startswith("VECTOR6") else (3 if t == "VECTOR3D" else 1)
        out.append(list(raw[:n]) if n > 1 else raw[0])
        raw = raw[n:]
    return out


class UrRtdeLink:
    """botrail as the RTDE client of a Universal Robots controller — URSim
    or a real one on the bench — with the robot's program running **on the
    controller** and the plant mirroring it (design-virtual-commissioning.md
    VC5, fidelity level 2).

    The exchange is the robot controller host's (`bt.vc.exchange(scene,
    "UR")`): what the robot program *writes* — its coils, the handshakes it
    raises — the plant reads from the controller's **output bit registers**
    (`write_output_boolean_register` in the exported script, `rtde=True`);
    what the program *reads* — the sensors, the handshakes from other hosts
    — the plant writes to the **input bit registers**
    (`read_input_boolean_register`). Register numbers are the exchange
    points' ports (`bt.io.ur_registers()`, 64 up: 0–63 belong to the
    fieldbus interfaces). Alongside, every output package carries
    `actual_q`, which the session feeds to the plant as the robot's joints
    (`open_plant(joints=[robot])` — the arm moves as the controller moves
    it, botrail watches collisions, clearance, sensors and handshakes), and
    `runtime_state`, the controller's word on whether a program runs
    (`idle` — the robot's done contact on the plant side).

    The client is botrail's own (protocol v2, one output recipe at
    `frequency` Hz, one input recipe), no library behind it. `robot` names
    the scene robot the controller moves (the host's robot)."""

    def __init__(
        self,
        exchange,
        host: str = "127.0.0.1",
        *,
        port: int = 30004,
        robot: Optional[str] = None,
        frequency: float = 125.0,
        allow_remote: bool = False,
        timeout: float = 5.0,
    ):
        if host not in ("127.0.0.1", "localhost", "::1") and not allow_remote:
            raise ValueError(
                f"ursim: {host!r} reaches beyond this machine — pass allow_remote=True for a controller on the "
                "bench (never a robot in production: the link moves nothing, but the program it watches does)"
            )
        if not (0.0 < frequency <= 500.0):
            raise ValueError(f"ursim: frequency must be within (0, 500] Hz, got {frequency}")
        self.xc = exchange
        self.host = host
        self.port = port
        self.frequency = float(frequency)
        self.timeout = timeout
        self.robots: list[str] = [robot] if robot else []
        # Robot program → plant: the exchange's inputs on output registers.
        self._reads: list[tuple[str, int]] = []
        # Plant → robot program: the exchange's outputs on input registers.
        self._writes: list[tuple[str, int]] = []
        for p in exchange.points:
            if p.port is None:
                raise ValueError(
                    f"ursim: `{p.label}` has no register — bind it on the controller node to a bit register "
                    "(bt.io.ur_registers())"
                )
            if p.value_type != "BOOL":
                raise ValueError(f"ursim: `{p.label}` is a {p.value_type} — RTDE bit registers carry BOOL only")
            if p.port < 64 or p.port > 127:
                raise ValueError(f"ursim: `{p.label}` is on register {p.port} — RTDE clients get 64-127")
            (self._reads if p.side == "input" else self._writes).append((p.label, p.port))
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._error: Optional[BaseException] = None
        self._out_types: list[str] = []
        self._out_struct: Optional[struct.Struct] = None
        self._out_names: list[str] = []
        self._out_recipe = 0
        self._in_struct: Optional[struct.Struct] = None
        self._in_recipe = 0
        self._latest: dict = {}
        self._sent: dict[str, bool] = {}
        self._seen: dict[str, bool] = {}
        self._initial = {p.label: bool(p.initial) for p in exchange.points}
        #: Output packages received so far.
        self.packages = 0
        #: Scans exchanged so far.
        self.exchanges = 0

    def describe(self) -> str:
        """`rtde 127.0.0.1:30004` — the link as the studio's dock names it."""
        return f"rtde {self.host}:{self.port}"

    # -- the wire ---------------------------------------------------------

    def _send(self, kind: int, payload: bytes = b"") -> None:
        assert self._sock is not None
        self._sock.sendall(struct.pack(">HB", 3 + len(payload), kind) + payload)

    def _recv_exact(self, n: int) -> bytes:
        assert self._sock is not None
        buf = b""
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("RTDE connection closed")
            buf += chunk
        return buf

    def _recv(self) -> tuple[int, bytes]:
        size, kind = struct.unpack(">HB", self._recv_exact(3))
        return kind, self._recv_exact(size - 3)

    def _request(self, kind: int, payload: bytes = b"") -> bytes:
        """Sends a control package and returns the controller's reply of
        the same kind (data packages and text in between are consumed)."""
        self._send(kind, payload)
        while True:
            got, body = self._recv()
            if got == kind:
                return body
            if got == _RTDE_DATA_PACKAGE:
                self._take(body)

    def start(self) -> "UrRtdeLink":
        if self._sock is not None:
            return self
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        sock.settimeout(self.timeout)
        self._sock = sock
        try:
            ok = self._request(_RTDE_REQUEST_PROTOCOL_VERSION, struct.pack(">H", 2))
            if not ok or not ok[0]:
                raise RuntimeError("the controller refused RTDE protocol version 2")
            names = ["actual_q", "runtime_state", "robot_mode"] + [f"output_bit_register_{n}" for _, n in self._reads]
            reply = self._request(_RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS,
                                  struct.pack(">d", self.frequency) + ",".join(names).encode("ascii"))
            recipe, types = reply[0], reply[1:].decode("ascii").split(",")
            if len(types) != len(names) or any(t not in _RTDE_FORMATS for t in types):
                bad = [n for n, t in zip(names, types) if t not in _RTDE_FORMATS]
                raise RuntimeError(f"the controller does not know the RTDE outputs {bad} (reply {types})")
            self._out_recipe, self._out_names, self._out_types = recipe, names, types
            self._out_struct = _rtde_struct(types)
            if self._writes:
                names_in = [f"input_bit_register_{n}" for _, n in self._writes]
                reply = self._request(_RTDE_CONTROL_PACKAGE_SETUP_INPUTS, ",".join(names_in).encode("ascii"))
                recipe, types = reply[0], reply[1:].decode("ascii").split(",")
                if len(types) != len(names_in) or any(t not in _RTDE_FORMATS for t in types):
                    bad = [n for n, t in zip(names_in, types) if t not in _RTDE_FORMATS]
                    raise RuntimeError(f"the controller refused the RTDE inputs {bad} (reply {types})")
                self._in_recipe = recipe
                self._in_struct = _rtde_struct(types)
            ok = self._request(_RTDE_CONTROL_PACKAGE_START)
            if not ok or not ok[0]:
                raise RuntimeError("the controller refused to start the RTDE stream")
        except Exception:
            sock.close()
            self._sock = None
            raise
        # The plant's outputs as they stand go out at once, so the program
        # never reads a register nobody wrote.
        self._push({label: self._initial[label] for label, _ in self._writes})
        self._thread = threading.Thread(target=self._reader, name="botrail-rtde", daemon=True)
        self._thread.start()
        return self

    def _reader(self) -> None:
        try:
            while not self._stop.is_set():
                try:
                    kind, body = self._recv()
                except socket.timeout:
                    continue
                if kind == _RTDE_DATA_PACKAGE:
                    self._take(body)
        except Exception as e:  # noqa: BLE001 - reported to the session
            if not self._stop.is_set():
                self._error = e

    def _take(self, body: bytes) -> None:
        if self._out_struct is None or not body or body[0] != self._out_recipe:
            return
        values = _rtde_unpack(self._out_struct, self._out_types, body[1:])
        with self._lock:
            self._latest = dict(zip(self._out_names, values))
            self.packages += 1

    def _push(self, values: dict[str, bool]) -> None:
        """Writes the plant's outputs that changed into the input registers
        (a data package carries the whole recipe)."""
        if self._in_struct is None:
            return
        self._sent.update(values)
        payload = bytes([self._in_recipe]) + self._in_struct.pack(
            *(bool(self._sent.get(label, self._initial[label])) for label, _ in self._writes)
        )
        self._send(_RTDE_DATA_PACKAGE, payload)

    # -- the session's view -------------------------------------------------

    def exchange(self, host: str, t: float, outputs: dict[str, Value]) -> dict[str, Value]:
        """The plant's outputs go to the program's input registers (changes
        only); the program's output registers come back as the plant's
        inputs (changes only, every register on the first scan)."""
        if self._error is not None:
            raise RuntimeError(f"ursim: {self._error}") from self._error
        self.exchanges += 1
        changed = {label: bool(v) for label, v in outputs.items()
                   if label in dict(self._writes) and self._sent.get(label) != bool(v)}
        if changed:
            self._push(changed)
        with self._lock:
            latest = dict(self._latest)
        writes: dict[str, Value] = {}
        for label, n in self._reads:
            value = latest.get(f"output_bit_register_{n}")
            if value is None:
                continue
            value = bool(value)
            if self._seen.get(label) != value:
                self._seen[label] = value
                writes[label] = value
        return writes

    def joints(self) -> Optional[list[float]]:
        """The controller's `actual_q` of the last package, or `None` before
        the first one."""
        with self._lock:
            q = self._latest.get("actual_q")
        return list(q) if q is not None else None

    @property
    def runtime_state(self) -> Optional[str]:
        """The controller's program state (`stopped`, `playing`, `paused` …)."""
        with self._lock:
            v = self._latest.get("runtime_state")
        return None if v is None else RUNTIME_STATE.get(int(v), str(v))

    @property
    def idle(self) -> Optional[bool]:
        """Whether no program runs on the controller — the robot's done
        contact on the plant side (`None` before the first package)."""
        state = self.runtime_state
        return None if state is None else state not in ("playing", "resuming", "retracting")

    def stop(self) -> None:
        self._stop.set()
        if self._sock is not None:
            try:
                self._send(_RTDE_CONTROL_PACKAGE_PAUSE)
            except OSError:
                pass
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
        if self._thread is not None:
            self._thread.join(self.timeout)
        self._sock = None
        self._thread = None


class UrDashboard:
    """The controller's dashboard (port 29999) and primary interface (port
    30001) as the demo needs them: bring the arm to RUNNING, send a script,
    read the mode."""

    def __init__(self, host: str = "127.0.0.1", *, dashboard_port: int = 29999, primary_port: int = 30001,
                 timeout: float = 5.0):
        self.host = host
        self.dashboard_port = dashboard_port
        self.primary_port = primary_port
        self.timeout = timeout

    def command(self, *commands: str, attempts: int = 5) -> list[str]:
        """Dashboard commands, one reply line each. A connection the
        controller accepts and drops (PolyScope still starting behind a
        port mapping) is retried."""
        last: Optional[BaseException] = None
        for _ in range(attempts):
            try:
                with socket.create_connection((self.host, self.dashboard_port), timeout=self.timeout) as s:
                    f = s.makefile("rw", newline="\n")
                    banner = f.readline()
                    if not banner:
                        raise ConnectionError("the dashboard closed the connection before its banner")
                    replies = []
                    for command in commands:
                        f.write(command + "\n")
                        f.flush()
                        reply = f.readline()
                        if not reply:
                            raise ConnectionError(f"the dashboard closed the connection on {command!r}")
                        replies.append(reply.strip())
                    return replies
            except OSError as e:
                last = e
                time.sleep(1.0)
        raise RuntimeError(f"the UR dashboard at {self.host}:{self.dashboard_port} did not answer: {last}")

    def robotmode(self) -> str:
        return self.command("robotmode")[0]

    def wait_ready(self, seconds: float = 180.0) -> None:
        """Waits for the dashboard to answer a `robotmode` (a container
        coming up: the port answers before PolyScope does)."""
        deadline = time.time() + seconds
        while time.time() < deadline:
            try:
                if self.command("robotmode", attempts=1)[0].startswith("Robotmode"):
                    return
            except (OSError, RuntimeError):
                pass
            time.sleep(2.0)
        raise RuntimeError(f"the UR dashboard at {self.host}:{self.dashboard_port} did not answer within {seconds} s")

    def prepare(self, seconds: float = 120.0) -> None:
        """Closes popups, unlocks a protective stop a previous program left
        behind (the controller refuses to start the next one while it
        stands), powers on and releases the brakes until the arm reports
        RUNNING."""
        self.command("close safety popup", "close popup", "stop")
        deadline = time.time() + seconds
        while time.time() < deadline:
            mode = self.robotmode()
            if mode.endswith("RUNNING"):
                if "PROTECTIVE_STOP" in self.command("safetystatus")[0]:
                    # Refused within 5 s of the stop: the loop asks again.
                    self.command("unlock protective stop")
                    time.sleep(1.0)
                    continue
                return
            if mode.endswith("CONFIRM_SAFETY"):
                self.command("close safety popup")
            elif mode.endswith("POWER_OFF"):
                self.command("power on")
            elif mode.endswith("IDLE"):
                self.command("brake release")
            time.sleep(1.0)
        raise RuntimeError(f"the arm did not reach RUNNING within {seconds} s (last: {self.robotmode()!r})")

    def run_script(self, script: str) -> None:
        """Sends a URScript program over the primary interface; the
        controller runs it at once."""
        with socket.create_connection((self.host, self.primary_port), timeout=self.timeout) as s:
            s.sendall(script.encode("utf-8"))

    def stop(self) -> None:
        self.command("stop")


def ursim(exchange, host: str = "127.0.0.1", **kw) -> UrRtdeLink:
    """botrail as the RTDE client of a UR controller (URSim in Docker, or
    one on the bench with `allow_remote=True`): the robot program runs on
    the controller, its I/O crosses the bit registers, and its joints
    stream into the plant. See `UrRtdeLink`."""
    return UrRtdeLink(exchange, host, **kw)

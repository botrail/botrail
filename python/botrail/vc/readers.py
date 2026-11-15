"""Readers: a running controller's I/O image, read and never written.

The links in `links.py` put botrail where the cell's I/O would be: they
write the controller's inputs, which is what a controller under test wants
and what running equipment must never get. A reader is a different type
for a different job — watching a controller that runs a real machine — and
it is read-only by construction, not by a flag:

* [`ModbusReader`] — a Modbus TCP client that sends *read* requests only:
  function codes 1 (coils), 2 (discrete inputs), 3 (holding registers) and
  4 (input registers). The module has one request builder and it refuses
  any other code; there is no code here that assembles a write (5, 6, 15,
  16), a diagnostic or a file transfer.
* [`OpcUaReader`] — an OPC UA client that reads each node once and then
  subscribes to its data changes. It never calls Write or a method.

Neither carries a way to start or stop the controller, load a program or
set its clock. A reader hands `bt.vc.monitor` the latest image it read
(`sample`); the monitor moves the model with what the controller wrote and
compares what the controller read with what the model computes.

Reading is not free for the controller: every request is served by its
communication task. The poll period is explicit and unhurried by default
(`period=0.2` s; a trend, not a scope) — slow it down on a busy CPU, and
prefer the controller's OPC UA server, a gateway or its data logger to a
direct connection to the CPU. Connecting a tool to a control network is
the plant's call (network segmentation, permits), not this module's.

This module imports nothing from `links.py`.
"""

from __future__ import annotations

import asyncio
import math
import re
import socket
import struct
import threading
import time
from typing import Callable, Optional, Union

Value = Union[bool, int, float]

__all__ = ["ModbusReader", "OpcUaReader", "READ_FUNCTION_CODES", "modbus_reader", "opcua_reader"]

#: The only Modbus function codes a reader sends.
READ_FUNCTION_CODES = (1, 2, 3, 4)

_TABLE_CODE = {"coil": 1, "discrete_input": 2, "holding_register": 3, "input_register": 4}
_MAX_QTY = {1: 2000, 2: 2000, 3: 125, 4: 125}
_IEC = re.compile(r"^%([IQM])([XWDL]?)(\d+)(?:\.(\d+))?$")


def _read_request(tid: int, unit: int, code: int, address: int, quantity: int) -> bytes:
    """The one frame a reader builds: MBAP header + a read request."""
    if code not in READ_FUNCTION_CODES:
        raise ValueError(f"a reader sends read requests only (function codes 1-4), not {code}")
    if not 1 <= quantity <= _MAX_QTY[code]:
        raise ValueError(f"a read of {quantity} items does not fit one request")
    return struct.pack(">HHHBBHH", tid & 0xFFFF, 0, 6, unit, code, address, quantity)


def _number(regs: list[int], ty: str, word_order: str) -> Value:
    if ty == "REAL":
        hi, lo = (regs[0], regs[1]) if word_order == "big" else (regs[1], regs[0])
        return struct.unpack(">f", struct.pack(">HH", hi, lo))[0]
    v = regs[0] & 0xFFFF
    return v - 0x10000 if v >= 0x8000 else v


class ModbusReader:
    """A read-only Modbus TCP client of a running controller's server.

    `exchange` is the controller's (`bt.vc.exchange(scene, "PLC1")`): the
    points it writes are read from its coils and holding registers, the
    points it reads from its discrete inputs and input registers — the
    tables a PLC's Modbus server publishes its output and input images in.
    The numbering follows the IEC addresses (`%IX100.5` → discrete input
    `(100 - iec_base) * 8 + 5`, `%QW100` → holding register `100 -
    iec_base`); `iec_base=0` reads the image where OpenPLC serves it.
    `addresses={point: number}` places a point whose address is in
    another notation (`X0`), `tables={point: "coil"}` moves one to
    another table. Every point must have a place: a reader does not guess
    where a controller keeps a bit.

    `period` is the poll period in seconds (0.2 by default). The image is
    read in as few requests as the map allows (contiguous blocks, small
    gaps bridged)."""

    def __init__(
        self,
        exchange,
        address: str,
        *,
        iec_base: int = 0,
        unit: int = 1,
        period: float = 0.2,
        word_order: str = "big",
        timeout: float = 2.0,
        addresses: Optional[dict[str, int]] = None,
        tables: Optional[dict[str, str]] = None,
    ):
        if not (period > 0.0):
            raise ValueError(f"modbus_reader: period must be positive, got {period}")
        host, _, port = address.rpartition(":")
        self.address = (host or address, int(port) if host else 502)
        self.xc = exchange
        self.host: str = exchange.host
        self.unit = unit
        self.period = float(period)
        self.word_order = word_order
        self.timeout = timeout
        #: `(label, table, address, count, type)` of every point read.
        self.rows: list[tuple[str, str, int, int, str]] = []
        unplaced = []
        for p in exchange.points:
            ty = p.value_type
            if p.side == "input":  # the controller writes it: its output image
                table = "coil" if ty == "BOOL" else "holding_register"
            else:
                table = "discrete_input" if ty == "BOOL" else "input_register"
            table = (tables or {}).get(p.label, table)
            if table not in _TABLE_CODE:
                raise ValueError(f"modbus_reader: `{p.label}`: unknown table {table!r}")
            at = (addresses or {}).get(p.label)
            if at is None:
                m = _IEC.match((p.address or "").strip())
                if m:
                    _, size, index, bit = m.groups()
                    index = int(index)
                    if ty == "BOOL" and (size or "X") == "X" and bit is not None and index >= iec_base:
                        at = (index - iec_base) * 8 + int(bit)
                    elif ty != "BOOL" and size in ("W", "D") and bit is None and index >= iec_base:
                        at = index - iec_base
            if at is None:
                unplaced.append(f"{p.label} ({p.address or 'unbound'})")
                continue
            self.rows.append((p.label, table, int(at), 2 if ty == "REAL" else 1, ty))
        if unplaced:
            raise ValueError(
                "modbus_reader: no register for " + ", ".join(unplaced) + " — bind the points to IEC addresses, "
                "or pass addresses={point: number}"
            )
        self._blocks = self._plan()
        self._lock = threading.Lock()
        self._image: Optional[dict[str, Value]] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._error: Optional[BaseException] = None
        self._tid = 0
        #: Whole images read so far.
        self.polls = 0
        #: Every function code sent so far — reads, by construction.
        self.function_codes: set[int] = set()

    def _plan(self) -> list[tuple[int, int, int]]:
        """`(function code, address, quantity)` blocks covering the map."""
        blocks: list[tuple[int, int, int]] = []
        for table, code in _TABLE_CODE.items():
            spans = sorted((at, at + count) for _, t, at, count, _ in self.rows if t == table)
            gap = 16 if code in (1, 2) else 4
            start = end = None
            for a, b in spans:
                if start is not None and a - end <= gap and b - start <= _MAX_QTY[code]:
                    end = max(end, b)
                    continue
                if start is not None:
                    blocks.append((code, start, end - start))
                start, end = a, b
            if start is not None:
                blocks.append((code, start, end - start))
        return blocks

    def describe(self) -> str:
        """`modbus read 10.0.0.5:502 · 200 ms` — the reader as the studio names it."""
        return f"modbus read {self.address[0]}:{self.address[1]} · {self.period * 1000:.0f} ms"

    # -- the wire -----------------------------------------------------------

    def _ask(self, sock: socket.socket, code: int, address: int, quantity: int) -> list[int]:
        self._tid = (self._tid + 1) & 0xFFFF
        sock.sendall(_read_request(self._tid, self.unit, code, address, quantity))
        self.function_codes.add(code)
        head = _recv(sock, 7)
        _, _, length, _ = struct.unpack(">HHHB", head)
        pdu = _recv(sock, length - 1)
        if pdu[0] != code:
            reason = pdu[1] if len(pdu) > 1 else 0
            raise RuntimeError(
                f"the controller refused the read of {quantity} at {address} (function {code}, exception {reason})")
        data = pdu[2:2 + pdu[1]]
        if code in (1, 2):
            return [(data[i // 8] >> (i % 8)) & 1 for i in range(quantity)]
        return [struct.unpack(">H", data[2 * i:2 * i + 2])[0] for i in range(quantity)]

    def _poll(self, sock: socket.socket) -> dict[str, Value]:
        tables: dict[tuple[str, int], int] = {}
        codes = {code: table for table, code in _TABLE_CODE.items()}
        for code, address, quantity in self._blocks:
            for i, v in enumerate(self._ask(sock, code, address, quantity)):
                tables[(codes[code], address + i)] = v
        image: dict[str, Value] = {}
        for label, table, at, count, ty in self.rows:
            if ty == "BOOL":
                image[label] = bool(tables[(table, at)])
            else:
                image[label] = _number([tables[(table, at + i)] for i in range(count)], ty, self.word_order)
        return image

    def _run(self) -> None:
        sock: Optional[socket.socket] = None
        while not self._stop.is_set():
            due = time.monotonic() + self.period
            try:
                if sock is None:
                    sock = socket.create_connection(self.address, timeout=self.timeout)
                    sock.settimeout(self.timeout)
                image = self._poll(sock)
                with self._lock:
                    self._image = image
                    self.polls += 1
                self._error = None
            except Exception as e:  # noqa: BLE001 - shown by the monitor; the next poll dials again
                self._error = e
                if sock is not None:
                    sock.close()
                    sock = None
            self._stop.wait(max(0.0, due - time.monotonic()))
        if sock is not None:
            sock.close()

    def start(self) -> "ModbusReader":
        if self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="botrail-modbus-reader", daemon=True)
            self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(self.timeout + 1.0)
        self._thread = None

    def sample(self, t: float = 0.0) -> Optional[dict[str, Value]]:
        """The last image read — `None` before the first poll answers."""
        with self._lock:
            return None if self._image is None else dict(self._image)

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()


def _recv(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("the controller closed the connection")
        buf += chunk
    return buf


def modbus_reader(exchange, address: str, **kw) -> ModbusReader:
    """A read-only Modbus TCP client of a running controller (see
    [`ModbusReader`])."""
    return ModbusReader(exchange, address, **kw)


class OpcUaReader:
    """A read-only OPC UA client of a running controller's server: every
    point's node is read once, then followed through a data-change
    subscription. `nodes` maps a point to its NodeId string
    (`"ns=4;s=GVL.beam"`) or is a callable `label -> NodeId`; by default a
    point is looked up as `ns=<namespace>;s=<tag or variable>`. `period`
    is the subscription's publishing interval in seconds (0.5 by default).
    Needs `asyncua` — `pip install "botrail[vc]"`."""

    def __init__(
        self,
        exchange,
        url: str,
        *,
        nodes: Union[dict[str, str], Callable[[str], str], None] = None,
        namespace: int = 2,
        period: float = 0.5,
    ):
        if not (period > 0.0):
            raise ValueError(f"opcua_reader: period must be positive, got {period}")
        self.xc = exchange
        self.host: str = exchange.host
        self.url = url
        self.period = float(period)
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
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._error: Optional[BaseException] = None
        self._lock = threading.Lock()
        self._image: dict[str, Value] = {}
        #: Data changes received so far.
        self.polls = 0

    def describe(self) -> str:
        """`opcua read opc.tcp://… · 500 ms` — the reader as the studio names it."""
        return f"opcua read {self.url} · {self.period * 1000:.0f} ms"

    def start(self) -> "OpcUaReader":
        if self._thread is not None:
            return self

        def run() -> None:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(self._main())
            except BaseException as e:  # noqa: BLE001 - reported to the monitor
                self._error = e
                self._ready.set()
            finally:
                loop.close()

        self._thread = threading.Thread(target=run, name="botrail-opcua-reader", daemon=True)
        self._thread.start()
        if not self._ready.wait(10.0):
            raise RuntimeError(f"opcua_reader: no connection to {self.url} within 10 s")
        if self._error is not None:
            raise RuntimeError(f"opcua_reader: {self._error}") from self._error
        return self

    async def _main(self) -> None:
        from asyncua import Client

        reader = self

        class Handler:
            def datachange_notification(self, node, val, data):
                label = labels.get(node.nodeid.to_string())
                if label is not None:
                    with reader._lock:
                        reader._image[label] = _coerce(val, reader.points[label].value_type)
                        reader.polls += 1

        async with Client(url=self.url) as client:
            nodes = {label: client.get_node(self._node(label)) for label in self.points}
            labels = {n.nodeid.to_string(): label for label, n in nodes.items()}
            for label, node in nodes.items():
                val = await node.read_value()
                with self._lock:
                    self._image[label] = _coerce(val, self.points[label].value_type)
            sub = await client.create_subscription(int(self.period * 1000), Handler())
            if nodes:
                await sub.subscribe_data_change(list(nodes.values()))
            self._ready.set()
            while not self._stop.is_set():
                await asyncio.sleep(0.05)
            await sub.delete()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(10.0)
        self._thread = None

    def sample(self, t: float = 0.0) -> Optional[dict[str, Value]]:
        """The image as last read or notified — `None` before the first read."""
        with self._lock:
            return dict(self._image) if self._image else None

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


def opcua_reader(exchange, url: str, **kw) -> OpcUaReader:
    """A read-only OPC UA client of a running controller (see
    [`OpcUaReader`])."""
    return OpcUaReader(exchange, url, **kw)

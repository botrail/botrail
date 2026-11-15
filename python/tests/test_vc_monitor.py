"""A machine's I/O log replayed, and a running controller watched read-only
(design-plc-resolution.md §11, PM0 / PM1): `bt.vc.IoLog`, `bt.vc.tap`,
`bt.vc.monitor`, and the readers that only read.

The controller here is a stand-in: a few lines of Modbus server that keeps
an image, plays a log into it, and writes down every function code it is
asked — which is how the read-only promise is checked on the wire.
"""

import inspect
import json
import socket
import struct
import threading
import time

import pytest

import botrail as bt
from botrail.vc import readers

PROGRAMS = ["line", "operator"]
PLC_PROGRAMS = ["line"]


def cell() -> bt.Scene:
    """A belt that carries one work to a beam: the PLC's rungs hold `run`
    from the start button until a moment after the beam, drive the belt
    from it and light a lamp at the beam. The operator is a script on a
    desk."""
    sig, both, either, neg = bt.seq.signal, bt.seq.all_of, bt.seq.any_of, bt.seq.not_
    scene = bt.Scene()
    scene.add_box("work", size=(0.08, 0.08, 0.06), position=(-0.4, 0.0, 0.83))
    scene.add_conveyor("conv", zone_position=(0.0, 0.0, 0.85), zone_size=(1.2, 0.24, 0.14),
                       velocity=(0.25, 0.0, 0.0), running=False)
    scene.add_beam_sensor("beam", (0.2, -0.14, 0.83), (0.2, 0.14, 0.83), watch=["work"])
    for name in ("pb_start", "run", "lamp"):
        scene.define_signal(name)
    lad = scene.sequence("line")
    # The belt stops a moment after the beam, as a scanned controller's does:
    # the work stands well inside the beam, not on its edge.
    lad.rung(sig("beam")).ton("t_beam", 0.1)
    lad.rung(both(either(sig("pb_start"), sig("run")), neg(sig("t_beam")))).coil("run")
    lad.rung(sig("run")).coil("conv")
    lad.rung(sig("beam")).coil("lamp")
    op = scene.sequence("operator")
    op.step("idle", transition=bt.seq.elapsed(0.3))
    op.step("press", actions=[bt.seq.set_signal("pb_start", True)], transition=bt.seq.elapsed(0.2))
    op.step("release", actions=[bt.seq.set_signal("pb_start", False)], transition=sig("beam"))
    op.step("done", transition=bt.seq.elapsed(0.5))
    scene.add_io_node("PLC1", kind="plc", programs=PLC_PROGRAMS,
                      channels=bt.io.di8(base="%IX100.0") + bt.io.do8(base="%QX100.0"))
    scene.add_io_node("DESK", kind="plc", programs=["operator"],
                      channels=bt.io.di8(base="%IX0.0") + bt.io.do8(base="%QX0.0"))
    scene.auto_assign_io()
    return scene


def logged(scene: bt.Scene):
    """The run with a logger on PLC1's wires: `(log, timeline)`."""
    xc = bt.vc.exchange(scene, "PLC1", PROGRAMS)
    with bt.vc.loopback(scene, PROGRAMS, host="PLC1", studio=False) as s:
        s.run()
        recording = s.recording()
    tapped = bt.vc.tap(bt.vc.replay(recording), xc)
    with bt.vc.Session(scene, PROGRAMS, links={"PLC1": tapped}, studio=False) as s:
        s.run()
        tl = s.timeline()
    return tapped.log, tl


def poses(tl, until: float) -> list:
    return [tl.object_pose("work", until * k / 30) for k in range(31)]


# ------------------------------------------------------------------ the log


def test_a_log_is_read_by_address_tag_variable_or_name() -> None:
    scene = cell()
    xc = bt.vc.exchange(scene, "PLC1", PLC_PROGRAMS)
    by = {p.label: p for p in xc.points}
    beam, conv = by["beam"].address, by["conv"].address
    assert beam.startswith("%IX100.") and conv.startswith("%QX100.")
    text = f"t,{beam},{conv},{by['lamp'].var},pb_start,TEMP\n0,0,0,0,0,21.5\n0.5,0,1,0,1,21.5\n2.0,1,0,1,0,21.6\n"
    log = bt.vc.IoLog.from_csv(text, xc)
    assert sorted(log.labels) == ["beam", "conv", "lamp", "pb_start"]
    assert log.unmatched == ["TEMP"]
    assert sorted(log.written) == ["conv", "lamp"] and sorted(log.read) == ["beam", "pb_start"]
    assert log.series["beam"] == [(0.0, False), (2.0, True)]
    assert log.value("conv", 1.0) is True and log.value("conv", 2.5) is False
    assert log.duration == 2.0
    # Without the percent sign, in another case, and by a name of the plant's own.
    again = bt.vc.IoLog.from_csv(f"Time,{beam[1:].lower()},Y0\n10.0,0,0\n10.5,1,1\n", xc, names={"Y0": "conv"})
    assert again.series["beam"] == [(0.0, False), (0.5, True)]          # the first row is zero
    assert again.series["conv"] == [(0.0, False), (0.5, True)]
    with pytest.raises(ValueError, match="no point"):
        bt.vc.IoLog.from_csv("t,Y0\n0,1\n", xc, names={"Y0": "nothing"})
    with pytest.raises(ValueError, match="no column is a point"):
        bt.vc.IoLog.from_csv("t,A,B\n0,1,1\n", xc)
    with pytest.raises(ValueError, match="as a bit"):
        bt.vc.IoLog.from_csv(f"t,{beam}\n0,maybe\n", xc)


def test_a_long_table_and_clock_times_are_read_too() -> None:
    xc = bt.vc.exchange(cell(), "PLC1", PLC_PROGRAMS)
    text = ("time,tag,value\n2026-11-10 08:15:02.000,beam,off\n2026-11-10 08:15:02.000,conv,on\n"
            "2026-11-10 08:15:04.250,beam,on\n2026-11-10 08:15:04.250,OTHER,1\n")
    log = bt.vc.IoLog.from_csv(text, xc)
    assert log.series["beam"] == [(0.0, False), (2.25, True)]
    assert log.series["conv"] == [(0.0, True)]
    assert log.unmatched == ["OTHER"] and log.period == 0.0
    # A table sampled on a period knows its period.
    wide = "t,beam\n" + "".join(f"{k * 0.1:.1f},{int(k > 5)}\n" for k in range(12))
    assert bt.vc.IoLog.from_csv(wide, xc).period == pytest.approx(0.1)


def test_a_tap_writes_the_table_a_logger_would_and_it_reads_back(tmp_path) -> None:
    scene = cell()
    log, _ = logged(scene)
    # The PLC's own handshake to the desk is not in the monitored exchange.
    path = tmp_path / "line_io.csv"
    log.save(path)
    header = path.read_text().splitlines()[0].split(",")
    assert header[0] == "t" and all(h.startswith("%") for h in header[1:])
    xc = bt.vc.exchange(scene, "PLC1", PLC_PROGRAMS)
    back = bt.vc.IoLog.load(path, xc)
    assert back.source == "log line_io.csv"
    for label in back.labels:
        want = [(round(t, 3), v) for t, v in log.series[label]]
        assert [(round(t, 3), v) for t, v in back.series[label]] == want
    rec = back.recording()
    assert {label for _, _, label, _ in rec.records} == set(back.written)
    late = back.delayed("beam", 0.5)
    assert late.series["beam"][0] == back.series["beam"][0]
    assert late.series["beam"][1][0] == pytest.approx(back.series["beam"][1][0] + 0.5)
    with pytest.raises(ValueError, match="not in the log"):
        back.delayed("nothing", 1.0)


# ---------------------------------------------------------------- the replay


def test_a_log_replays_as_the_machine_ran_and_agrees_with_the_model() -> None:
    scene = cell()
    log, machine = logged(scene)
    xc = bt.vc.exchange(scene, "PLC1", PLC_PROGRAMS)
    read = bt.vc.IoLog.from_csv(log.to_csv(), xc)
    with bt.vc.monitor(scene, PLC_PROGRAMS, read, studio=False) as m:
        assert m.pace == "sync" and m.tolerance == pytest.approx(0.1)
        m.run()
        # What the PLC wrote drives the model, the sensor is compared, the
        # button — which nothing in the model presses — is followed.
        assert sorted(m.driving) == ["conv", "lamp"]
        assert m.compared == ["beam"] and m.followed == ["pb_start"]
        assert m.deviations() == []
        tl = m.timeline()
        report = m.report()
    assert "agrees with the model" in report
    end = min(machine.duration, tl.duration)
    assert poses(tl, end) == poses(machine, end), "the model did not move as the machine did"
    assert dict(tl.signals)["pb_start"] == dict(machine.signals)["pb_start"]
    readings = json.loads(tl.readings_json())
    assert readings["host"] == "PLC1" and readings["deviations"] == []
    lane = {s["name"]: s for s in readings["signals"]}["beam"]
    # The reading lies on the model's lane, edge for edge.
    assert list(zip(lane["times"], lane["values"])) == [(pytest.approx(t), v) for t, v in dict(tl.signals)["beam"]]
    assert tl.with_readings(None).readings_json() is None


def test_a_sensor_that_answers_late_is_named() -> None:
    scene = cell()
    log, _ = logged(scene)
    xc = bt.vc.exchange(scene, "PLC1", PLC_PROGRAMS)
    read = bt.vc.IoLog.from_csv(log.to_csv(), xc)
    rose = read.series["beam"][1][0]
    with bt.vc.monitor(scene, PLC_PROGRAMS, read.delayed("beam", 0.4), studio=False) as m:
        m.run()
        found = m.deviations()
        tl = m.timeline()
    assert [(d.lane, d.kind, d.open) for d in found] == [("beam", "late", False)]
    assert found[0].start == pytest.approx(rose) and found[0].duration == pytest.approx(0.4)
    assert "0.40 s later on the machine" in found[0].note
    carried = json.loads(tl.readings_json())["deviations"]
    assert carried[0]["lane"] == "beam" and carried[0]["kind"] == "late"
    # Early by as much, and inside the tolerance: nothing.
    with bt.vc.monitor(scene, PLC_PROGRAMS, read.delayed("beam", -0.4), studio=False) as m:
        m.run()
        assert [(d.lane, d.kind) for d in m.deviations()] == [("beam", "early")]
    with bt.vc.monitor(scene, PLC_PROGRAMS, read.delayed("beam", 0.4), tolerance=0.5, studio=False) as m:
        m.run()
        assert m.deviations() == []


def test_a_sensor_that_never_answers_stays_open() -> None:
    scene = cell()
    log, _ = logged(scene)
    xc = bt.vc.exchange(scene, "PLC1", PLC_PROGRAMS)
    read = bt.vc.IoLog.from_csv(log.to_csv(), xc)
    dead = bt.vc.IoLog(xc, {**read.series, "beam": read.series["beam"][:1]})
    with bt.vc.monitor(scene, PLC_PROGRAMS, dead, studio=False) as m:
        m.run()
        assert m.deviations() == []           # the log ends inside the tolerance …
        m.run(for_=0.5)                       # … and past it the silence counts
        found = m.deviations()
    assert [(d.lane, d.kind, d.open) for d in found] == [("beam", "late", True)]
    assert "the machine has not followed" in found[0].note


def test_a_point_can_be_followed_instead_of_compared() -> None:
    scene = cell()
    log, _ = logged(scene)
    xc = bt.vc.exchange(scene, "PLC1", PLC_PROGRAMS)
    read = bt.vc.IoLog.from_csv(log.to_csv(), xc)
    with bt.vc.monitor(scene, PLC_PROGRAMS, read.delayed("beam", 0.4), follow=["beam", "pb_start"],
                       studio=False) as m:
        m.run()
        assert m.compared == [] and m.deviations() == []
        # The model's lane is the machine's reading.
        tl = m.timeline()
    assert dict(tl.signals)["beam"][1][0] == pytest.approx(read.series["beam"][1][0] + 0.4, abs=0.011)
    with pytest.raises(ValueError, match="not points PLC1 reads"):
        bt.vc.monitor(scene, PLC_PROGRAMS, read, follow=["conv"], studio=False)


# ------------------------------------------------------------- read-only


WRITING = ("force", "unforce", "release", "press", "write", "apply", "inject", "pause", "resume", "step",
           "break_on", "speed")


def test_the_monitor_and_the_readers_have_no_way_to_write() -> None:
    # The types: no hand on the plant, no write on the wire.
    for name in WRITING:
        assert not hasattr(bt.vc.Monitor, name), f"Monitor.{name}"
    for cls in (bt.vc.ModbusReader, bt.vc.OpcUaReader):
        public = [n for n in dir(cls) if not n.startswith("_")]
        assert not [n for n in public if any(w in n.lower() for w in ("write", "force", "set", "call", "send"))], public
    # The module: one request builder, reads only; nothing borrowed from the links that write.
    for code in (5, 6, 15, 16, 22, 23, 8, 43, 0):
        with pytest.raises(ValueError, match="read requests only"):
            readers._read_request(1, 1, code, 0, 1)
    for code in readers.READ_FUNCTION_CODES:
        frame = readers._read_request(7, 1, code, 3, 2)
        assert frame == struct.pack(">HHHBBHH", 7, 0, 6, 1, code, 3, 2)
    source = inspect.getsource(readers)
    code_only = "\n".join(line.split("#")[0] for line in source.split('"""')[2::2] for line in line.splitlines())
    for banned in ("write_value", "write_attribute", "call_method", "write_coil", "write_register", "links"):
        assert banned not in code_only, banned
    assert "from .links" not in source and "import links" not in source


def test_a_monitor_refuses_a_link_that_writes() -> None:
    scene = cell()
    xc = bt.vc.exchange(scene, "PLC1", PLC_PROGRAMS)
    link = bt.vc.modbus_server(xc, "127.0.0.1:5999", iec_base=100)
    with pytest.raises(TypeError, match="controller under test"):
        bt.vc.monitor(scene, PLC_PROGRAMS, link, host="PLC1", studio=False)


def test_a_monitor_refuses_the_studios_hands() -> None:
    scene = cell()
    log, _ = logged(scene)
    xc = bt.vc.exchange(scene, "PLC1", PLC_PROGRAMS)
    read = bt.vc.IoLog.from_csv(log.to_csv(), xc)
    with bt.vc.monitor(scene, PLC_PROGRAMS, read, studio=False) as m:
        m.tick(5)
        core = m._core
        for cmd in ({"command": "force", "lane": "beam", "value": True},
                    {"command": "force_word", "lane": "x", "value": 1.0},
                    {"command": "write", "word": "x", "value": 1.0},
                    {"command": "press", "lane": "pb_start"},
                    {"command": "pause"}, {"command": "step", "n": 1}, {"command": "speed", "speed": 2.0},
                    {"command": "break", "lane": "beam"}):
            core._apply_command(cmd)
            status = m.status()
            assert status["refused"].startswith(cmd["command"]) and "only reads" in status["refused"]
            assert status["read_only"] is True and status["forced"] == [] and status["buttons"] == []
            assert not core.paused and core.forced == {} and core.breaks == []
        m.tick(5)
        assert m.signal("beam") is False             # the force never reached the model
        # The stream's own controls are not hands on anything.
        core._apply_command({"command": "record", "on": True})
        assert core.recording_from is not None
        assert status["label"].endswith("(monitor of PLC1)")
        assert status["readings"]["host"] == "PLC1"


class FakePlc:
    """A Modbus TCP server with an image and a notebook: it answers reads
    from the image, refuses everything else, and writes down every
    function code it is asked."""

    def __init__(self):
        self.tables = {1: [0] * 64, 2: [0] * 64, 3: [0] * 16, 4: [0] * 16}
        self.codes: list[int] = []
        self._server = socket.socket()
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(("127.0.0.1", 0))
        self._server.listen(2)
        self.port = self._server.getsockname()[1]
        self._stop = False
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self) -> None:
        while not self._stop:
            try:
                conn, _ = self._server.accept()
            except OSError:
                return
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def _serve(self, conn: socket.socket) -> None:
        try:
            while True:
                head = conn.recv(7, socket.MSG_WAITALL)
                if len(head) < 7:
                    return
                tid, pid, length, unit = struct.unpack(">HHHB", head)
                pdu = conn.recv(length - 1, socket.MSG_WAITALL)
                code = pdu[0]
                self.codes.append(code)
                if code in (1, 2, 3, 4):
                    address, quantity = struct.unpack(">HH", pdu[1:5])
                    values = self.tables[code][address:address + quantity]
                    if code in (1, 2):
                        data = bytearray((quantity + 7) // 8)
                        for i, v in enumerate(values):
                            if v:
                                data[i // 8] |= 1 << (i % 8)
                    else:
                        data = b"".join(struct.pack(">H", v & 0xFFFF) for v in values)
                    reply = bytes([code, len(data)]) + bytes(data)
                else:
                    reply = bytes([code | 0x80, 1])
                conn.sendall(struct.pack(">HHHB", tid, pid, len(reply) + 1, unit) + reply)
        except OSError:
            pass
        finally:
            conn.close()

    def close(self) -> None:
        self._stop = True
        self._server.close()


def test_a_running_controller_is_followed_with_read_requests_only(tmp_path) -> None:
    """The fake PLC plays the machine's log into its image in real time; a
    monitor follows it through a reader. Every frame the PLC received was
    a read, the model made the machine's moves, and what was read replays."""
    scene = cell()
    log, machine = logged(scene)
    xc = bt.vc.exchange(scene, "PLC1", PLC_PROGRAMS)
    read = bt.vc.IoLog.from_csv(log.to_csv(), xc)
    plc = FakePlc()
    reader = bt.vc.modbus_reader(xc, f"127.0.0.1:{plc.port}", iec_base=100, period=0.02)
    place = {label: (table, at) for label, table, at, _, _ in reader.rows}
    code = {"coil": 1, "discrete_input": 2, "holding_register": 3, "input_register": 4}
    assert place["beam"][0] == "discrete_input" and place["conv"][0] == "coil"
    assert reader.describe() == f"modbus read 127.0.0.1:{plc.port} · 20 ms"

    def play() -> None:
        events = sorted((t, label, v) for label, s in read.series.items() for t, v in s)
        t0 = time.monotonic() + 0.3
        for t, label, v in events:
            wait = t0 + t - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            table, at = place[label]
            plc.tables[code[table]][at] = int(v)

    threading.Thread(target=play, daemon=True).start()
    try:
        with bt.vc.monitor(scene, PLC_PROGRAMS, reader, studio=False) as m:
            assert m.pace == "realtime" and m.tolerance == pytest.approx(0.1)
            with pytest.raises(ValueError, match="no end"):
                m.run()
            m.run(for_=read.duration + 1.0)
            tl = m.timeline()
            found = m.deviations()
            seen = m.log()
            stats = m.stats()
    finally:
        plc.close()
    assert plc.codes and set(plc.codes) <= {1, 2, 3, 4}, sorted(set(plc.codes))
    assert reader.function_codes <= {1, 2, 3, 4} and stats["reads"] > 20 and stats["error"] is None
    # The model did what the machine did: the belt ran once, the work reached the beam.
    conv = dict(tl.signals)["conv"]
    assert [v for _, v in conv] == [False, True, False]
    assert len(tl.signal("beam").rising_edges()) == 1 and len(tl.signal("lamp").rising_edges()) == 1
    travelled = tl.object_pose("work", tl.duration)[0][0] - tl.object_pose("work", 0.0)[0][0]
    want = machine.object_pose("work", machine.duration)[0][0] - machine.object_pose("work", 0.0)[0][0]
    assert travelled == pytest.approx(want, abs=0.02)
    assert found == [], [d.note for d in found]
    # What was read is a log: it replays without the wire, the same every time.
    path = tmp_path / "seen.csv"
    seen.save(path)
    again = bt.vc.IoLog.load(path, xc)
    runs = []
    for _ in range(2):
        with bt.vc.monitor(scene, PLC_PROGRAMS, again, studio=False) as r:
            r.run()
            runs.append(r.timeline())
    assert runs[0].signals == runs[1].signals
    assert [v for _, v in dict(runs[0].signals)["conv"]] == [False, True, False]


def test_a_reader_wants_a_place_for_every_point() -> None:
    scene = cell()
    xc = bt.vc.exchange(scene, "PLC1", PLC_PROGRAMS)
    with pytest.raises(ValueError, match="no register for"):
        bt.vc.modbus_reader(xc, "127.0.0.1:502", iec_base=200)
    reader = bt.vc.modbus_reader(xc, "10.0.0.5", iec_base=200, addresses={p.label: k for k, p in enumerate(xc.points)})
    assert reader.address == ("10.0.0.5", 502) and reader.period == pytest.approx(0.2)
    with pytest.raises(ValueError, match="period"):
        bt.vc.modbus_reader(xc, "127.0.0.1:502", period=0.0)


def test_an_opcua_server_is_read_and_followed() -> None:
    pytest.importorskip("asyncua", reason="botrail[vc] extra")
    import asyncio

    from asyncua import Server, ua

    scene = cell()
    xc = bt.vc.exchange(scene, "PLC1", PLC_PROGRAMS)
    ready, stop = threading.Event(), threading.Event()
    box: dict = {}

    def serve() -> None:
        async def main() -> None:
            server = Server()
            await server.init()
            sock = socket.socket()
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
            sock.close()
            server.set_endpoint(f"opc.tcp://127.0.0.1:{port}/plc")
            ns = await server.register_namespace("plc")
            folder = await server.nodes.objects.add_folder(ns, "GVL")
            nodes = {}
            for p in xc.points:
                nodes[p.label] = await folder.add_variable(ua.NodeId(p.var, ns), p.var, False)
            box.update(port=port, ns=ns, nodes=nodes, loop=asyncio.get_running_loop())
            async with server:
                ready.set()
                while not stop.is_set():
                    await asyncio.sleep(0.02)

        asyncio.run(main())

    threading.Thread(target=serve, daemon=True).start()
    assert ready.wait(20.0)

    def put(label: str, value: bool) -> None:
        asyncio.run_coroutine_threadsafe(box["nodes"][label].write_value(value), box["loop"]).result(5.0)

    reader = bt.vc.opcua_reader(xc, f"opc.tcp://127.0.0.1:{box['port']}/plc", namespace=box["ns"], period=0.05)
    try:
        with bt.vc.monitor(scene, PLC_PROGRAMS, reader, studio=False) as m:
            m.run(for_=0.3)
            put("conv", True)                       # the PLC starts the belt …
            m.run(until=lambda mon: mon.signal("beam"), timeout=10.0)
            put("beam", True)                       # … and reads its beam when the work is there
            put("conv", False)
            m.run(for_=0.5)
            tl = m.timeline()
            found = m.deviations()
    finally:
        stop.set()
    assert [v for _, v in dict(tl.signals)["conv"]] == [False, True, False]
    assert found == [], [d.note for d in found]
    assert reader.describe().startswith("opcua read opc.tcp://127.0.0.1:")


# ------------------------------------------------- what the log needed fixed


def test_a_declared_feeder_coil_crosses_the_exchange() -> None:
    """A magazine's run coil declared an output is a point like any other:
    the reference run writes it through the exchange, so the recording —
    and a log — carry it, and the replay feeds the work."""
    scene = bt.Scene()
    scene.add_box("work", size=(0.08, 0.08, 0.06), position=(-0.9, 0.0, 0.5))
    scene.add_conveyor("conv", zone_position=(0.0, 0.0, 0.85), zone_size=(1.2, 0.24, 0.14),
                       velocity=(0.25, 0.0, 0.0), running=True)
    scene.add_source("feeder", ["work"], park=(-0.9, 0.0, 0.5), position=(-0.4, 0.0, 0.83), interval=0.0)
    scene.add_beam_sensor("beam", (0.0, -0.14, 0.83), (0.0, 0.14, 0.83), watch=["work"])
    scene.define_signal("go")
    lad = scene.sequence("line")
    lad.rung(bt.seq.rising("go")).coil("feeder")
    op = scene.sequence("operator")
    op.step("idle", transition=bt.seq.elapsed(0.2))
    op.step("go", actions=[bt.seq.set_signal("go", True)], transition=bt.seq.signal("beam"))
    scene.add_io_node("PLC1", kind="plc", programs=["line"],
                      channels=bt.io.di8(base="%IX100.0") + bt.io.do8(base="%QX100.0"))
    scene.add_io_node("DESK", kind="plc", programs=["operator"],
                      channels=bt.io.di8(base="%IX0.0") + bt.io.do8(base="%QX0.0"))
    scene.declare_io("feeder", role="output")
    scene.auto_assign_io()
    bake = scene.simulate_sequences(["line", "operator"])
    with bt.vc.loopback(scene, ["line", "operator"], host="PLC1", studio=False) as s:
        s.run()
        looped, rec = s.timeline(), s.recording()
    assert looped.signals == bake.signals
    assert [v for _, _, label, v in rec.records if label == "feeder"] == [False, True, False]
    with bt.vc.Session(scene, ["line", "operator"], links={"PLC1": bt.vc.replay(rec)}, studio=False) as s:
        s.run()
        replayed = s.timeline()
    assert len(replayed.signal("beam").rising_edges()) == 1

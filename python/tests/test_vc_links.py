"""Virtual commissioning, VC2: the plant on the wire — a Modbus TCP server
the controller polls, an OPC UA client of the controller's server, the
Structured Text export a runtime compiles, and a session paced to the
wall clock. The controllers here are Python stand-ins (a pymodbus client,
an asyncua server) doing what the PLC program would.
"""

import asyncio
import sys
import threading
import time
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "export"))

pymodbus = pytest.importorskip("pymodbus", reason="botrail[vc] extra")


def plc_pick_cell() -> bt.Scene:
    """The pick cell with a PLC driving the arm from outside its controller;
    the PLC's image starts at %IX100.0 / %QX100.0 / %QW100."""
    import export_urscript as pick

    scene = pick.build_cell()
    pick.author_sequence(scene, coil_grasp=True)  # the grasp is the coil's: no attach command on the wire
    scene.add_io_node("UR", kind="robot_controller", robots=["simple_arm"], channels=bt.io.ur_standard())
    scene.add_io_node(
        "PLC1", kind="plc", programs=["pick"],
        channels=bt.io.di8(base="%IX100.0") + bt.io.do8(base="%QX100.0") + bt.io.word(count=2, base="%QW100"),
    )
    scene.auto_assign_io()
    return scene


def free_port() -> int:
    import socket

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# ------------------------------------------------------------ register map


def test_modbus_map_follows_the_iec_addresses_and_appends_the_rest() -> None:
    scene = plc_pick_cell()
    xc = bt.vc.exchange(scene, "PLC1")
    link = bt.vc.modbus_server(xc, f"127.0.0.1:{free_port()}", iec_base=100)
    rows = {r.label: r for r in link.register_map()}
    # Sensors the PLC reads → discrete inputs at their %IX100.n bit.
    beam = rows["part_at_pick"]
    assert beam.table == "discrete_input" and beam.iec.startswith("%IX100.")
    assert beam.address == int(beam.iec.split(".")[1])
    # Coils the PLC writes → coils at their %QX100.n bit; the program word →
    # holding register 0 (%QW100).
    assert rows["conv"].table == "coil"
    assert rows["simple_arm.program"].table == "holding_register" and rows["simple_arm.program"].address == 0
    assert rows["simple_arm.program"].type == "INT"
    # Done contacts come back as discrete inputs too.
    assert rows["simple_arm.done"].table == "discrete_input"
    md = link.register_map_markdown()
    assert "| `conv` |" in md
    with pytest.raises(ValueError, match="allow_remote"):
        bt.vc.modbus_server(xc, "0.0.0.0:5020")


# ---------------------------------------------------------------- Modbus


def test_a_modbus_client_drives_the_plant_as_the_plc_would() -> None:
    """A pymodbus client plays the PLC: it polls the discrete inputs, runs the
    pick program's first steps by hand (belt on until the beam, then the
    arm's `to_pick` motion by start pulse and program word) and the plant
    follows — in wall-clock time, sped up."""
    from pymodbus.client import ModbusTcpClient

    scene = plc_pick_cell()
    xc = bt.vc.exchange(scene, "PLC1")
    port = free_port()
    link = bt.vc.modbus_server(xc, f"127.0.0.1:{port}", iec_base=100)
    rows = {r.label: r for r in link.register_map()}
    entries = {label: number for _, number, _key, label in xc.entries}

    observed = {}
    stop = threading.Event()

    def plc() -> None:
        client = ModbusTcpClient("127.0.0.1", port=port, timeout=2.0)
        assert client.connect()
        try:
            # feed: belt on, wait for the beam.
            client.write_coil(rows["conv"].address, True)
            deadline = time.time() + 20.0
            while time.time() < deadline and not stop.is_set():
                di = client.read_discrete_inputs(rows["part_at_pick"].address, count=1)
                if not di.isError() and di.bits[0]:
                    observed["beam"] = time.time()
                    break
                time.sleep(0.01)
            client.write_coil(rows["conv"].address, False)
            # to_pick: program word, then start; hold start until busy (done low).
            client.write_register(rows["simple_arm.program"].address, entries["to_pick"])
            client.write_coil(rows["simple_arm.start"].address, True)
            deadline = time.time() + 20.0
            while time.time() < deadline and not stop.is_set():
                di = client.read_discrete_inputs(rows["simple_arm.done"].address, count=1)
                if not di.isError() and not di.bits[0]:
                    observed["busy"] = time.time()
                    break
                time.sleep(0.01)
            client.write_coil(rows["simple_arm.start"].address, False)
            deadline = time.time() + 20.0
            while time.time() < deadline and not stop.is_set():
                di = client.read_discrete_inputs(rows["simple_arm.done"].address, count=1)
                if not di.isError() and di.bits[0]:
                    observed["done"] = time.time()
                    break
                time.sleep(0.01)
        finally:
            client.close()

    thread = threading.Thread(target=plc, daemon=True)
    with bt.vc.Session(scene, ["pick"], links={"PLC1": link}, pace="realtime", speed=5.0) as s:
        thread.start()
        s.run(until=lambda sess: "done" in observed, timeout=120.0)
        stop.set()
        tl = s.timeline()
        rec = s.recording()
        stats = s.stats()
    thread.join(5.0)
    assert {"beam", "busy", "done"} <= set(observed), observed
    # The plant did what the PLC said: the belt ran until the beam, the arm
    # moved once, and the controller's writes are on record.
    conv = [e for e in tl.signals if e[0] == "conv"][0][1]
    assert conv[1][1] is True and conv[-1][1] is False
    assert tl.robot_busy("simple_arm"), "the arm never moved"
    labels = {label for _, _, label, _ in rec.records}
    assert {"conv", "simple_arm.start", "simple_arm.program"} <= labels
    assert stats["scans"] > 10 and stats["pace"] == "realtime"
    # And the recording replays without the wire, deterministically.
    with bt.vc.Session(scene, ["pick"], links={"PLC1": bt.vc.replay(rec)}) as r:
        r.run(for_=tl.duration)
        again = r.timeline()
    assert again.signals == tl.signals


# ---------------------------------------------------------------- OPC UA


def test_an_opcua_server_as_the_controller() -> None:
    asyncua = pytest.importorskip("asyncua", reason="botrail[vc] extra")
    from asyncua import Server, ua

    scene = plc_pick_cell()
    xc = bt.vc.exchange(scene, "PLC1")
    port = free_port()
    url = f"opc.tcp://127.0.0.1:{port}/botrail/"
    ready = threading.Event()
    stop = threading.Event()
    nodes: dict = {}
    seen: dict = {}

    async def serve() -> None:
        server = Server()
        await server.init()
        server.set_endpoint(url)
        server.set_security_policy([ua.SecurityPolicyType.NoSecurity])
        ns = await server.register_namespace("botrail")
        folder = await server.nodes.objects.add_folder(ns, "PLC1")
        for p in xc.points:
            # Typed like a PLC's tags: BOOL, INT (16 bit), REAL (32 bit).
            if p.value_type == "BOOL":
                variant = ua.Variant(bool(p.initial), ua.VariantType.Boolean)
            elif p.value_type == "INT":
                variant = ua.Variant(int(p.initial), ua.VariantType.Int16)
            else:
                variant = ua.Variant(float(p.initial), ua.VariantType.Float)
            node = await folder.add_variable(ua.NodeId(p.var, ns), p.var, variant)
            await node.set_writable()
            nodes[p.label] = node
        async with server:
            ready.set()
            # The "PLC program": belt on; when the beam trips, belt off and
            # the arm's `to_pick` by program word + start, start dropped on
            # busy, done awaited.
            entries = {label: number for _, number, _key, label in xc.entries}
            await nodes["conv"].write_value(True)
            phase = "feed"
            while not stop.is_set():
                beam = await nodes["part_at_pick"].read_value()
                done = await nodes["simple_arm.done"].read_value()
                if phase == "feed" and beam:
                    await nodes["conv"].write_value(False)
                    await nodes["simple_arm.program"].write_value(
                        ua.DataValue(ua.Variant(entries["to_pick"], ua.VariantType.Int16)))
                    await nodes["simple_arm.start"].write_value(True)
                    seen["beam"] = True
                    phase = "start"
                elif phase == "start" and not done:
                    await nodes["simple_arm.start"].write_value(False)
                    seen["busy"] = True
                    phase = "wait"
                elif phase == "wait" and done:
                    seen["done"] = True
                    phase = "end"
                await asyncio.sleep(0.02)

    def run_server() -> None:
        asyncio.run(serve())

    thread = threading.Thread(target=run_server, daemon=True)
    thread.start()
    assert ready.wait(20.0), "the OPC UA server did not come up"
    ns_index = 2
    link = bt.vc.opcua_client(xc, url, namespace=ns_index, period_ms=20)
    try:
        with bt.vc.Session(scene, ["pick"], links={"PLC1": link}, pace="realtime", speed=5.0) as s:
            s.run(until=lambda sess: "done" in seen, timeout=120.0)
            tl = s.timeline()
    finally:
        stop.set()
        thread.join(10.0)
    assert {"beam", "busy", "done"} <= set(seen), seen
    assert tl.robot_busy("simple_arm")


# -------------------------------------------------------------------- ST


def test_structured_text_carries_the_programs_and_the_located_globals() -> None:
    scene = plc_pick_cell()
    st = scene.st("PLC1")
    assert "PROGRAM pick" in st and "END_PROGRAM" in st
    assert "CONFIGURATION Config0" in st and "RESOURCE Res0 ON PLC" in st
    # The exchange's points as located globals, with the PLC's addresses.
    assert "part_at_pick AT %IX100." in st
    assert "conv_run AT %QX100." in st
    assert "simple_arm_program AT %QW100 : INT" in st
    # The robot handshake — posted by the step, issued on ready, dropped on
    # busy — and the numbered program.
    assert "simple_arm_cmd := TRUE;" in st
    assert "IF simple_arm_cmd AND NOT simple_arm_start AND simple_arm_done THEN simple_arm_start := TRUE; simple_arm_cmd := FALSE; END_IF;" in st
    assert "IF simple_arm_start AND NOT simple_arm_done THEN simple_arm_start := FALSE; simple_arm_busy := TRUE; END_IF;" in st
    assert "simple_arm_program := 1; (* to_pick *)" in st
    assert "(simple_arm_busy AND simple_arm_done)" in st
    # The grasp is the coil's: no attach / detach command crosses the wire,
    # and the coil's global says what the plant does with it.
    assert "attach:" not in st and "detach:" not in st
    assert "grasp of `simple_arm`" in st
    assert all("attach" not in label and "detach" not in label for _, _, _key, label in bt.vc.exchange(scene, "PLC1").entries)
    # Steps are flags; edges are sampled every scan; the program cycles.
    assert "s_" in st and "R_TRIG" in st or "TON" in st
    assert "(* cycle *)" in st
    parked = scene.st("PLC1", cycle=False)
    assert "(* end of cycle *)" in parked and "(* cycle *)" not in parked
    with pytest.raises(ValueError, match="runs no program"):
        scene.st("UR")
    # A host of the cell's own robot writes its motions as comments, never
    # as wires.
    plain = bt.vc.exchange(scene, "PLC1")
    assert all(p.side in ("input", "output") for p in plain.points)

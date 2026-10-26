"""Virtual commissioning, VC5 (design-virtual-commissioning.md): the robot
controller in the loop — the program's I/O on the controller's bit
registers (`rtde=True`), the RTDE link streaming the controller's joints
into a plant that mirrors them, the controller's word on being busy, and
the vendor interface templates as data in the I/O map and the handshake
specification. The controller here is a stand-in on the RTDE port; the
real URSim run is `examples/commissioning/ursim_pick_cell/run_demo.py`.
"""

import importlib.util
import socket
import struct
import sys
import threading
import time
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "export"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

vc = bt.vc


def ursim_demo():
    path = EXAMPLES / "commissioning" / "ursim_pick_cell" / "run_demo.py"
    spec = importlib.util.spec_from_file_location("ursim_pick_cell_demo", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def wait_for(predicate, seconds: float = 3.0, what: str = "") -> None:
    deadline = time.time() + seconds
    while time.time() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    pytest.fail(f"timed out waiting for {what or predicate}")


# ------------------------------------------------------------ a fake UR


_TYPES = {"actual_q": ("VECTOR6D", "6d"), "runtime_state": ("UINT32", "I"), "robot_mode": ("INT32", "i")}


class FakeUr(threading.Thread):
    """A controller stand-in on the RTDE port: the protocol handshake, one
    output recipe streamed at `hz`, one input recipe taken in. The test
    plays the robot program by hand — moves `q`, sets `state`, flips
    output registers — and reads what the plant wrote into the input
    registers."""

    def __init__(self, hz: float = 200.0):
        super().__init__(name="fake-ur", daemon=True)
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self.hz = hz
        self.q = [0.0] * 6
        self.state = 1  # stopped
        self.mode = 7  # running
        self.out_regs: dict[int, bool] = {}
        self.in_regs: dict[int, bool] = {}
        self.out_names: list[str] = []
        self.in_names: list[str] = []
        self.started = threading.Event()
        self.stop_evt = threading.Event()
        self.lock = threading.Lock()
        self.conn = None
        self.requests: list[int] = []

    def set_out(self, n: int, value: bool) -> None:
        with self.lock:
            self.out_regs[n] = bool(value)

    def _reply(self, kind: int, body: bytes) -> None:
        with self.lock:
            try:
                self.conn.sendall(struct.pack(">HB", 3 + len(body), kind) + body)
            except OSError:  # the client hung up (its stop sends a pause we cannot answer)
                pass

    def run(self) -> None:
        self.srv.settimeout(10.0)
        try:
            self.conn, _ = self.srv.accept()
        except socket.timeout:
            return
        self.conn.settimeout(0.1)
        threading.Thread(target=self._push, name="fake-ur-push", daemon=True).start()
        buf = b""
        while not self.stop_evt.is_set():
            try:
                chunk = self.conn.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while len(buf) >= 3:
                size, kind = struct.unpack(">HB", buf[:3])
                if len(buf) < size:
                    break
                body, buf = buf[3:size], buf[size:]
                self._on_package(kind, body)

    def _on_package(self, kind: int, body: bytes) -> None:
        self.requests.append(kind)
        if kind == 86:  # protocol version
            self._reply(kind, b"\x01" if struct.unpack(">H", body)[0] == 2 else b"\x00")
        elif kind == 79:  # setup outputs
            names = body[8:].decode().split(",")
            types = [_TYPES[n][0] if n in _TYPES else ("BOOL" if n.startswith("output_bit_register_") else "NOT_FOUND")
                     for n in names]
            self.out_names = names
            self._reply(kind, bytes([1]) + ",".join(types).encode())
        elif kind == 73:  # setup inputs
            names = body.decode().split(",")
            types = ["BOOL" if n.startswith("input_bit_register_") else "NOT_FOUND" for n in names]
            self.in_names = names
            self._reply(kind, bytes([2]) + ",".join(types).encode())
        elif kind == 83:  # start
            self.started.set()
            self._reply(kind, b"\x01")
        elif kind == 80:  # pause
            self._reply(kind, b"\x01")
        elif kind == 85 and body and body[0] == 2:  # input data package
            values = struct.unpack(">" + "?" * len(self.in_names), body[1:])
            with self.lock:
                for name, value in zip(self.in_names, values):
                    self.in_regs[int(name.rsplit("_", 1)[1])] = bool(value)

    def _push(self) -> None:
        self.started.wait(10.0)
        while not self.stop_evt.is_set():
            with self.lock:
                parts = [bytes([1])]
                for name in self.out_names:
                    if name == "actual_q":
                        parts.append(struct.pack(">6d", *self.q))
                    elif name == "runtime_state":
                        parts.append(struct.pack(">I", self.state))
                    elif name == "robot_mode":
                        parts.append(struct.pack(">i", self.mode))
                    else:
                        n = int(name.rsplit("_", 1)[1])
                        parts.append(struct.pack(">?", self.out_regs.get(n, False)))
                payload = b"".join(parts)
                try:
                    self.conn.sendall(struct.pack(">HB", 3 + len(payload), 85) + payload)
                except OSError:
                    return
            time.sleep(1.0 / self.hz)

    def close(self) -> None:
        self.stop_evt.set()
        try:
            if self.conn is not None:
                self.conn.close()
        finally:
            self.srv.close()


# ----------------------------------------------------------------- tests


def test_the_exported_program_lives_in_the_controllers_bit_registers() -> None:
    demo = ursim_demo()
    scene = demo.build()
    runs = scene.simulate_scenarios(["pick"])  # the sweep carries the reject arm too
    script = runs.to_script(sequence="pick", rtde=True, name="botrail_pick")
    assert "while (not read_input_boolean_register(64)):" in script  # the beam, rising edge: low then high
    assert "write_output_boolean_register(64, True)" in script  # the belt on
    assert "write_output_boolean_register(65, True)" in script and "write_output_boolean_register(65, False)" in script
    assert "read_input_boolean_register(65)" in script  # the gauge verdict
    assert "standard_digital" not in script
    # The same program on the standard digital I/O still says it the old way.
    plain = runs.to_script(sequence="pick", name="botrail_pick")
    assert "write_output_boolean_register" not in plain and "set_standard_digital_out(64" in plain
    # The channel set the registers come from.
    regs = bt.io.ur_registers()
    assert [c["id"] for c in regs[:2]] == ["IR64", "IR65"] and regs[0]["port"] == 64 and regs[32]["id"] == "OR64"
    with pytest.raises(ValueError, match="0-127"):
        bt.io.ur_registers(first=100, count=40)


def test_the_plant_mirrors_the_controller_and_trades_its_registers() -> None:
    demo = ursim_demo()
    scene = demo.build()
    xc = vc.exchange(scene, "UR")
    assert sorted(p.label for p in xc.inputs) == ["conv", "vacuum"]  # what the robot program writes
    assert sorted(p.label for p in xc.outputs) == ["part_at_pick", "spec_ok"]  # what it reads
    fake = FakeUr(hz=200.0)
    fake.start()
    link = vc.ursim(xc, "127.0.0.1", port=fake.port, robot="simple_arm", frequency=200.0)
    assert link.describe() == f"rtde 127.0.0.1:{fake.port}" and link.robots == ["simple_arm"]
    try:
        with vc.Session(scene, ["pick"], links={"UR": link}, max_duration=120.0, studio=False) as s:
            assert s._live.streamed == ["simple_arm"]
            # The handshake went through and the plant's outputs as they
            # stand reached the input registers before the first scan.
            assert fake.requests[:4] == [86, 79, 73, 83]
            wait_for(lambda: fake.in_regs == {64: False, 65: True}, what="the initial registers")
            s.tick()
            # The controller plays and moves the arm: the plant mirrors it
            # (the stream is wall-clock; the plant takes it on its next scan).
            target = [0.3, -1.0, 1.2, -1.5, -1.2, 0.2]
            fake.state = 2
            fake.q = list(target)
            wait_for(lambda: link.joints() == target and link.idle is False, what="the controller's joints")
            s.tick()
            mirrored = s._live.joint_positions("simple_arm")
            assert max(abs(a - b) for a, b in zip(mirrored, target)) < 1e-9
            assert link.runtime_state == "playing"
            # The program switches the belt on: the plant's coil follows.
            fake.set_out(64, True)
            wait_for(lambda: link._latest.get("output_bit_register_64") is True, what="the belt register")
            s.tick()
            assert s.signal("conv") is True
            t_conv = s.t
            # The part rides to the beam: the plant's sensor lands in the
            # program's input register.
            s.run(until=lambda sess: sess.signal("part_at_pick"), timeout=30.0)
            s.tick()  # the scan after the edge is the one that writes the register
            wait_for(lambda: fake.in_regs.get(64) is True, what="the beam register")
            # The program grips: the coil-bound grasp takes the part.
            fake.set_out(64, False)
            fake.set_out(65, True)
            wait_for(lambda: link._latest.get("output_bit_register_65") is True
                     and link._latest.get("output_bit_register_64") is False, what="the vacuum register")
            s.tick()
            assert s.signal("vacuum") is True and s.signal("conv") is False
            # The program ends: the controller is idle again.
            fake.set_out(65, False)
            fake.state = 1
            wait_for(lambda: link.idle is True and link._latest.get("output_bit_register_65") is False,
                     what="the controller idle")
            s.tick()
            assert s.signal("vacuum") is False
            rec = s.recording()
            t_end = s.t
            tl = s.finish()
        # The run is the ordinary timeline: the mirrored joints are its track.
        q = tl.sample(t_end, "simple_arm")
        assert max(abs(a - b) for a, b in zip(q, target)) < 1e-9
        writes = {(r[1], r[2]) for r in rec.records if r[0] > 0.0}
        assert {("UR", "conv"), ("UR", "vacuum")} <= writes
        assert any(r[2] == "conv" and r[3] is True and abs(r[0] - t_conv) <= 0.011 for r in rec.records)
        assert link.packages > 10 and link.exchanges > 10
    finally:
        fake.close()


def test_a_streamed_robots_idle_is_its_controllers_word() -> None:
    from test_vc import two_controller_cell

    scene, names = two_controller_cell()
    # `near`'s own controller NEAR runs `watch`; the PLC's `line` waits on
    # `robot_done("near")` at its last step.
    live = scene.open_plant(names, ["NEAR"], joints=["near"])
    assert live.streamed == ["near"]
    live.report_idle("near", False)
    for _ in range(600):
        live.tick()
        if dict(live.steps())["line"] == "finish":
            break
    assert dict(live.steps())["line"] == "finish"
    for _ in range(200):
        live.tick()
    assert dict(live.steps())["line"] == "finish"  # the controller says busy: no finish
    live.report_idle("near", True)
    for _ in range(5):
        live.tick()
    assert dict(live.steps())["line"] is None  # done
    live.command([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], "near")
    live.tick()
    assert live.joint_positions("near") == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    with pytest.raises(ValueError, match="not streamed"):
        live.report_idle("far", True)
    # Refusals: a robot's own controller external with no stream, an
    # unknown robot, a robot whose controller is inside.
    with pytest.raises(ValueError, match="stream its joints"):
        scene.open_plant(names, ["NEAR"])
    with pytest.raises(ValueError, match="no robot"):
        scene.open_plant(names, ["NEAR"], joints=["nope"])
    with pytest.raises(ValueError, match="not an external host"):
        scene.open_plant(names, ["PLC1"], joints=["near"])


def test_the_rtde_link_refuses_what_the_registers_cannot_carry() -> None:
    demo = ursim_demo()
    scene = demo.build()
    xc = vc.exchange(scene, "UR")
    with pytest.raises(ValueError, match="allow_remote"):
        vc.ursim(xc, "192.168.0.10")
    with pytest.raises(ValueError, match="frequency"):
        vc.ursim(xc, frequency=0.0)
    # Registers below 64 belong to the fieldbus interfaces.
    low = demo.build.__module__  # the demo module is loaded; rebuild its cell with low registers
    import export_urscript as pick

    scene2 = pick.build_cell()
    pick.author_sequence(scene2, coil_grasp=True)
    scene2.add_io_node("UR", kind="robot_controller", robots=["simple_arm"], channels=bt.io.ur_registers(first=0))
    scene2.bind_input("part_at_pick", "UR", "IR0")
    scene2.bind_input("spec_ok", "UR", "IR1")
    scene2.bind_output("conv", "UR", "OR0")
    scene2.bind_output("vacuum", "UR", "OR1")
    with pytest.raises(ValueError, match="64-127"):
        vc.ursim(vc.exchange(scene2, "UR"))
    # A point with no register at all.
    scene3 = pick.build_cell()
    pick.author_sequence(scene3, coil_grasp=True)
    scene3.add_io_node("UR", kind="robot_controller", robots=["simple_arm"], channels=bt.io.ur_registers())
    with pytest.raises(ValueError, match="no register"):
        vc.ursim(vc.exchange(scene3, "UR"))
    assert low


def test_vendor_templates_bind_the_handshake_with_their_names_and_timing() -> None:
    from test_vc import two_controller_cell

    scene, names = two_controller_cell()
    # The PLC drives `far` with two motions (a program word); give `far` a
    # FANUC controller on the UOP.
    scene.add_io_node("FAR", kind="robot_controller", robots=["far"], channels=bt.io.fanuc_uop())
    uop = vc.interfaces.fanuc_uop_pns(programs={"far_go": 1, "far_home": 2})
    bound = uop.apply(scene, "far", "FAR")
    assert bound == {"start": "far.start", "program": "far.program", "done": "far.done"}
    points = {(p.label, p.host): p for p in scene.io_points(names)}
    assert points[("far.start", "FAR")].status.startswith("bound")
    sheet = scene.io_list("csv", names)
    assert "PROD_START" in sheet and "PROGRUN" in sheet and "PNS1-8" in sheet
    tl = scene.simulate_sequences(names, max_duration=60.0)
    spec = tl.handshake_spec()
    assert "`PROD_START` — falling edge, pulse ≥ 100 ms" in spec
    assert "PNSTROBE (UI17) pulse ≥ 100 ms" in spec and "SNACK (UO19)" in spec
    assert "`PROGRUN` — active low" in spec
    # The sheet of the interface itself.
    md = uop.to_markdown()
    assert "| start | `PROD_START` | UI18 | PLC → robot | active high | falling edge, pulse ≥ 100 ms |" in md
    assert "| far_go | 1 |" in md
    assert uop.line("estop_ok").invert and uop.line("done").invert
    with pytest.raises(KeyError, match="no line"):
        uop.line("warp")
    # `near` runs its own program, but the PLC waits on its done: that one
    # line is all there is to bind on NEAR.
    assert vc.interfaces.ur_remote().apply(scene, "near", "NEAR") == {"done": "near.done"}
    # A robot nobody else drives or waits on has no handshake to bind.
    alone = ursim_demo().build()
    with pytest.raises(ValueError, match="driven by no other host"):
        vc.interfaces.ur_remote(start="IR64", stop="IR65", running="OR64").apply(alone, "simple_arm", "UR")
    # The other vendors, each on its own channel set.
    for template, channels in (
        (vc.interfaces.kuka_automatic_external(programs={"far_go": 1}), bt.io.kuka_automatic_external()),
        (vc.interfaces.abb_system_io(), bt.io.abb_system_io()),
        (vc.interfaces.ur_remote(), bt.io.ur_standard()),
    ):
        scene, names = two_controller_cell()
        scene.add_io_node("FAR", kind="robot_controller", robots=["far"], channels=channels)
        bound = template.apply(scene, "far", "FAR")
        assert {"start", "done"} <= set(bound)
        assert ("program" in bound) == template.has("program")
        assert template.to_markdown().startswith(f"# {template.vendor}")
        spec = scene.simulate_sequences(names, max_duration=60.0).handshake_spec()
        assert f"`{template.line('start').tag}`" in spec
    with pytest.raises(ValueError, match="distinct"):
        vc.interfaces.fanuc_uop_pns(programs={"a": 1, "b": 1})

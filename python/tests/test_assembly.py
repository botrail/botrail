"""`bt.assembly` — bolted joints, the screwdriver's program, the fastening
reports — and the parts that serve it (`bt.parts.compound`, `bolt`,
`screw_feeder`, `bt.tools.screwdriver`), asserted without the catalog
(design/design-assembly.md A0).

What these pin: a screw's figures come from the standard's table and its
reference torque is a ceiling; the star order is opposite pairs from the
middle out; a joint the screw cannot make is refused with the numbers;
the driver's rundown is the thread's feed, its program answers OK unless
a scenario faults it and ends with the cell; a magazine of one screw is
one line on the bill; the presenter hands out one screw per request and
its presence lane follows the screw."""

from pathlib import Path

import botrail as bt
import pytest

A = bt.assembly
EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
HOLES = ((-60, -45), (60, -45), (60, 45), (-60, 45), (0, 45), (0, -45))
SEAT = ((1.0, 0.0, 0.06), (0.0, 0.0, 0.0, 1.0))


def scene_() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))


def patterns(depth: float = 13.0, grip: float = 10.0):
    threads = A.BoltPattern([A.Hole(f"h{i}", xy, "threaded", depth_mm=depth) for i, xy in enumerate(HOLES)],
                            [A.Locator("p0", (-60, 0), "pin", 6.0, 8.0)])
    clearances = A.BoltPattern([A.Hole(f"h{i}", xy, "clearance", 5.5, grip_mm=grip) for i, xy in enumerate(HOLES)],
                               [A.Locator("p0", (-60, 0), "hole", 6.1, 10.0)])
    return threads, clearances


def joint_(scene, fastener=None, **kw):
    scene.add_box("housing", size=(0.16, 0.12, 0.06), position=(1.0, 0.0, 0.03))
    scene.add_box("cover", size=(0.16, 0.12, 0.01), position=(1.0, 0.5, 0.005))
    threads, clearances = patterns()
    args = {"a": "housing", "b": "cover", "pattern_a": threads, "pattern_b": clearances,
            "fastener": fastener or A.iso4762(5, 20), "seat": SEAT, "thickness": 0.010,
            "torque_nm": (4.0, 5.0), "min_engagement_mm": 10.0}
    args.update(kw)
    return A.joint(scene, "cover_joint", **args)


def test_iso4762_tables_the_screw_and_its_ceiling() -> None:
    m5 = A.iso4762(5, 20)
    assert (m5.pitch_mm, m5.head_dk_mm, m5.head_k_mm, m5.drive_s_mm) == (0.8, 8.5, 5.0, 4.0)
    assert m5.model_name == "ISO 4762 M5x20-8.8" and m5.torque_ref_nm == 6.15
    assert 0.004 < m5.mass_kg < 0.006
    assert A.iso4762(6, 20, "10.9").torque_ref_nm == 15.0
    assert A.iso4762(8, 30, "A2-70").torque_ref_nm is None
    with pytest.raises(ValueError, match="not tabled"):
        A.iso4762(7, 20)
    with pytest.raises(ValueError, match="positive"):
        A.Fastener(5, 0.8, -1, 8.5, 5, 4)


def test_star_order_is_opposite_pairs_from_the_middle_out() -> None:
    holes = [A.Hole(f"h{i}", xy) for i, xy in enumerate(HOLES)]
    order = A.star_order(holes)
    # The middle pair first, then the two diagonals — each pair opposite
    # through the centroid.
    assert order[:2] in (["h4", "h5"], ["h5", "h4"])
    xy = {h.id: h.xy_mm for h in holes}
    for a, b in zip(order[::2], order[1::2]):
        assert xy[a][0] == -xy[b][0] and xy[a][1] == -xy[b][1]
    assert sorted(order) == sorted(xy)
    # A ring of eight: 1-5-3-7-2-6-4-8 up to where it starts.
    import math

    ring = {str(i): (math.cos(i * math.pi / 4), math.sin(i * math.pi / 4)) for i in range(8)}
    star = A.star_order(ring)
    assert len(star) == 8 and all(abs(int(a) - int(b)) == 4 for a, b in zip(star[::2], star[1::2]))
    assert A.star_order({}) == []


def test_joint_puts_a_frame_on_every_hole_and_derives_the_engagement() -> None:
    scene = scene_()
    joint = joint_(scene)
    assert joint.order[:2] in (["h4", "h5"], ["h5", "h4"]) and len(joint.frames) == 6
    (x, y, z), q = scene.frame(joint.hole_frame("h0"))
    # On the cover's top face over the hole: the seat plus the thickness.
    assert (x, y, z) == pytest.approx((1.0 - 0.060, -0.045, 0.06 + 0.010))
    assert q == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert "cover_joint/seat" in scene.frames and "cover_joint/locator/p0" in scene.frames
    # M5×20 through 10 mm: 10 mm engaged, the tip 10 mm into a 13 mm hole.
    assert joint.engagement_mm("h0") == 10.0 and joint.tip_depth_mm("h0") == 10.0
    assert A.check(joint) == []
    # The joint's own torque above the class's reference is a warning, not
    # a refusal: the table is a ceiling for steel, the joint knows better.
    hot = joint_(scene_(), torque_nm=(6.0, 7.0))
    assert [f.code for f in A.check(hot)] == ["torque_over_table"]


def test_a_screw_the_joint_cannot_use_is_refused_with_the_numbers() -> None:
    with pytest.raises(ValueError, match=r"engages 6.0 mm of thread, under the 10.0 mm"):
        joint_(scene_(), fastener=A.iso4762(5, 16))
    with pytest.raises(ValueError, match="hole_bottom|takes 13.0 mm of tip"):
        joint_(scene_(), fastener=A.iso4762(5, 25))
    # Not strict: the findings come back instead.
    loose = joint_(scene_(), fastener=A.iso4762(5, 16), strict=False)
    assert {f.code for f in A.check(loose)} == {"engagement_short"}
    scene = scene_()
    scene.add_box("housing", size=(0.16, 0.12, 0.06), position=(1.0, 0.0, 0.03))
    scene.add_box("cover", size=(0.16, 0.12, 0.01), position=(1.0, 0.5, 0.005))
    threads, clearances = patterns()
    with pytest.raises(ValueError, match="patterns disagree"):
        A.joint(scene, "j", a="housing", b="cover", pattern_a=threads,
                pattern_b=A.BoltPattern(clearances.holes[:5]), fastener=A.iso4762(5, 20), seat=SEAT,
                thickness=0.01, torque_nm=4.5)
    with pytest.raises(ValueError, match="every hole once"):
        A.joint(scene, "j", a="housing", b="cover", pattern_a=threads, pattern_b=clearances,
                fastener=A.iso4762(5, 20), seat=SEAT, thickness=0.01, torque_nm=4.5, order=["h0"])


def test_the_driver_is_checked_against_the_joint_and_its_datasheet() -> None:
    scene = scene_()
    joint = joint_(scene)
    tool = {"torque_nm": (0.15, 5.0), "screw_length_mm": 50, "bit_mm": 4}
    ok = A.driver(scene, "driver", robot=scene.robots[0], bit="tool0", fastener=joint.fastener,
                  torque_nm=4.5, cycles=7, tool=tool)
    assert A.check(joint, ok) == []
    # M6 class 8.8 at its table torque is more than a 5 N·m driver gives,
    # and a driver set to 6 N·m is outside a 4–5 N·m joint.
    m6 = joint_(scene_(), fastener=A.iso4762(6, 20), torque_nm=(10.0, 10.5), strict=False)
    hot = A.Driver(name="d", program="d", robot="r", bit="b", fastener=m6.fastener, torque_nm=6.0,
                   tool={"torque_nm": (0.15, 5.0)})
    assert {f.code for f in A.check(m6, hot)} == {"torque_off_joint", "torque_over_tool"}
    wrong_bit = A.Driver(name="d", program="d", robot="r", bit="b", fastener=joint.fastener, torque_nm=4.5,
                         tool={"bit_mm": 5, "screw_length_mm": 15}, shank="s", stroke_m=0.015)
    assert {f.code for f in A.check(joint, wrong_bit)} == {"bit_mismatch", "screw_too_long", "stroke_short"}
    with pytest.raises(ValueError, match="shank and stroke together"):
        A.driver(scene, "d2", robot=scene.robots[0], bit="tool0", fastener=joint.fastener, torque_nm=4.5, cycles=1,
                 shank="s")


def test_the_drivers_program_times_the_rundown_from_the_thread_and_answers() -> None:
    scene = scene_()
    m5 = A.iso4762(5, 20)
    drv = A.driver(scene, "driver", robot=scene.robots[0], bit="tool0", fastener=m5, torque_nm=4.5, cycles=3,
                   rpm_run=600.0, rpm_final=40.0, engage=0.0)
    # 20 mm of M5×0.8 at 600 rpm: 2.5 s. A third of a turn at 40 rpm: 0.5 s.
    assert drv.t_run_s == pytest.approx(2.5) and drv.t_final_s == pytest.approx(0.5)
    assert drv.node == "driver/controller" and drv.program == "driver" and drv.signal("ok") == "driver/ok"
    assert A.rundown_s(0.020, 0.8, 600.0) == pytest.approx(2.5)
    # A robot program that starts the driver twice and raises `finish`.
    S = bt.seq
    sq = scene.sequence("cell")
    for k in range(2):
        sq.step(f"start{k}", actions=[S.set_signal(drv.signal("start"))],
                transition=S.any_of(S.signal(drv.signal("ok")), S.signal(drv.signal("nok"))))
        sq.step(f"drop{k}", actions=[S.set_signal(drv.signal("start"), False)],
                transition=S.signal(drv.signal("busy"), False))
    sq.step("end", actions=[S.set_signal(drv.signal("finish"))])
    tl = scene.simulate_sequences(["cell", "driver"], max_duration=30.0)
    oks = tl.signal(drv.signal("ok")).rising_edges()
    assert len(oks) == 2 and tl.signal(drv.signal("nok")).rising_edges() == []
    # Each answer comes the driver's cycle after its start; the bake ends
    # with the robot's program, the third cycle never entered.
    assert oks[0] == pytest.approx(drv.t_cycle_s, abs=0.03)
    assert tl.duration < 2 * drv.t_cycle_s + 1.0
    runs = tl.signal(drv.signal("run")).high_spans()
    assert len(runs) == 2 and runs[0][1] - runs[0][0] == pytest.approx(drv.t_cycle_s, abs=0.03)
    # The faults: the first run NOK, then OK; both NOK.
    scene.add_scenario("nok_once", signals={drv.signal("fault_first"): True})
    scene.add_scenario("nok_twice", signals={drv.signal("fault_first"): True, drv.signal("fault_retry"): True})
    runs = scene.simulate_scenarios(["cell", "driver"], max_duration=30.0)
    once, twice = runs["nok_once"], runs["nok_twice"]
    assert len(once.signal(drv.signal("nok")).rising_edges()) == 1 and len(once.signal(drv.signal("ok")).rising_edges()) == 1
    assert len(twice.signal(drv.signal("nok")).rising_edges()) == 2
    # The handshake is wires between two controllers on the I/O list.
    hosts = {(p.name, p.direction, p.host) for p in scene.io_points()}
    assert (drv.signal("start"), "input", drv.node) in hosts and (drv.signal("ok"), "output", drv.node) in hosts


def test_the_driver_refuses_a_start_with_the_estop_in() -> None:
    scene = scene_()
    scene.add_zone_sensor("estop", position=(3.0, 0.0, 0.5), size=(0.05, 0.05, 0.05), watch_robot=True)
    drv = A.driver(scene, "driver", robot=scene.robots[0], bit="tool0", fastener=A.iso4762(4, 12), torque_nm=2.0,
                   cycles=1, estop="estop")
    S = bt.seq
    sq = scene.sequence("cell")
    sq.step("start", actions=[S.set_signal(drv.signal("start"))], transition=S.signal(drv.signal("ok")))
    sq.step("drop", actions=[S.set_signal(drv.signal("start"), False)], transition=S.signal(drv.signal("busy"), False))
    sq.step("end", actions=[S.set_signal(drv.signal("finish"))])
    assert scene.simulate_sequences(["cell", "driver"], max_duration=10.0).duration < 10.0
    scene.add_scenario("estop_pressed", faults=[bt.io.stuck("estop", True)])
    runs = scene.simulate_scenarios(["cell", "driver"], max_duration=5.0)
    assert "estop_pressed" in runs.errors and "cell/start" in runs.errors["estop_pressed"]
    table = scene.interlocks(["driver"])
    assert any("estop" in str(row.get("inputs", "")) for row in table.rows)


def test_a_magazine_of_screws_is_one_line_and_the_feeder_hands_them_out() -> None:
    scene = scene_()
    m5 = A.iso4762(5, 20)
    screws = [m5.place(scene, f"screw{i}", (2.0, 0.0, 0.0), manufacturer="botrail") for i in range(4)]
    by = {row["names"][0]: row for row in scene.bom().rows}
    assert by["screw0"]["names"] == screws and by["screw0"]["qty"] == 4
    assert by["screw0"]["category"] == "fastener" and by["screw0"]["model"] == "ISO 4762 M5x20-8.8"
    assert by["screw0"]["attributes"]["thread_mm"] == 5.0 and scene.bom().total("mass_kg") == pytest.approx(4 * m5.mass_kg, abs=1e-4)
    # One obstacle each: a shank with its tip at the origin and a head on top.
    lo, hi = scene.obstacle_bounds("screw0")
    assert lo[2] == pytest.approx(0.0, abs=1e-6) and hi[2] == pytest.approx(0.025, abs=1e-6)
    assert hi[0] - lo[0] == pytest.approx(0.0085, abs=2e-4)
    feeder = bt.parts.screw_feeder(scene, "feeder", (2.0, 0.5), screws=screws, model="SP-1")
    assert (feeder.device, feeder.present, feeder.pick) == ("feeder", "feeder/present", "feeder/pick")
    assert feeder.screws == screws and "feeder" in scene.device_names and "feeder/present" in scene.sensor_names
    by = {row["names"][0]: row for row in scene.bom().rows}
    assert by["feeder"]["category"] == "feeder.screw" and by["feeder"]["model"] == "SP-1"
    # The screws went into the magazine: none at the pick point until asked.
    (px, py, pz), _ = scene.frame(feeder.pick)
    assert pz == pytest.approx(0.15 + 0.001)
    for screw in screws:
        assert scene.obstacle_pose(screw)[0][2] == pytest.approx(0.075)
    S = bt.seq
    sq = scene.sequence("cell")
    sq.step("ask", actions=[S.start("feeder")], transition=S.signal("feeder/present"))
    sq.step("take", actions=[S.attach("screw0", link="tool0")])
    sq.step("away", actions=[S.detach("screw0")], transition=S.elapsed(0.5))
    sq.step("ask2", actions=[S.start("feeder")], transition=S.signal("feeder/present"))
    sq.step("wait", transition=S.elapsed(0.5))
    tl = scene.simulate_sequence("cell", max_duration=10.0)
    # One screw per request, each at the pick point; the lane follows them.
    # (A pose is read a tick after the release: the track's jump is at
    # the step boundary.)
    p0, _ = tl.object_pose("screw0", tl.step_span("take").start + 0.005)
    assert (p0[0], p0[1], p0[2]) == pytest.approx((px, py, pz), abs=1e-6)
    p1, _ = tl.object_pose("screw1", tl.duration)
    assert (p1[0], p1[1]) == pytest.approx((px, py), abs=1e-6)
    p2, _ = tl.object_pose("screw2", tl.duration)
    assert p2[2] == pytest.approx(0.075)
    # The lane rose with the first screw and is on with the second standing
    # there (this arm never lifts one away, so it never drops between).
    present = tl.signal("feeder/present")
    assert present.rising_edges() == pytest.approx([0.01]) and present.value_at(tl.duration)
    with pytest.raises(ValueError, match="make it with bt.parts.bolt"):
        bt.parts.screw_feeder(scene, "f2", (3.0, 0.5), screws=["nothing"])
    with pytest.raises(ValueError, match="magazine"):
        bt.parts.screw_feeder(scene, "f3", (3.0, 0.5), screws=screws, size=(0.08, 0.16, 0.15))


def test_compound_is_one_obstacle_from_several_solids() -> None:
    scene = scene_()
    name = bt.parts.compound(scene, "cover", [bt.parts.Box((0.16, 0.12, 0.01), at=(0, 0, 0.005)),
                                              bt.parts.Cylinder(0.0225, 0.03, at=(0, 0, 0.01))], (1.0, 0.0, 0.0))
    assert name == "cover" and scene.obstacle_names == ["cover"]
    lo, hi = scene.obstacle_bounds("cover")
    assert hi[2] == pytest.approx(0.04, abs=1e-6) and hi[0] - lo[0] == pytest.approx(0.16, abs=1e-6)
    # Solid, as a plate with a boss is: a probe inside the boss collides.
    scene.add_box("probe", size=(0.01, 0.01, 0.01), position=(1.0, 0.0, 0.025))
    scene.set_joint_positions(scene.joint_positions)
    with pytest.raises(ValueError, match="at least one solid"):
        bt.parts.compound(scene, "empty", [], (0, 0, 0))
    with pytest.raises(ValueError, match="positive"):
        bt.parts.compound(scene, "flat", [bt.parts.Box((0.1, 0.0, 0.1))], (0, 0, 0))


def test_compound_preserves_hard_edges_when_viewers_compute_normals() -> None:
    """Box faces and cylinder caps must not share smoothed side vertices."""
    import math

    for solid in (bt.parts.Box((0.16, 0.12, 0.01)), bt.parts.Cylinder(0.02, 0.03)):
        obj = bt.parts._obj_text([solid], segments=32)
        vertices, faces = [], []
        for line in obj.splitlines():
            fields = line.split()
            if fields[0] == "v":
                vertices.append(tuple(float(v) for v in fields[1:]))
            elif fields[0] == "f":
                faces.append(tuple(int(v) - 1 for v in fields[1:]))
        adjacent = {}
        for face in faces:
            a, b, c = (vertices[i] for i in face)
            u, v = [b[i] - a[i] for i in range(3)], [c[i] - a[i] for i in range(3)]
            cross = (u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0])
            length = math.sqrt(sum(n*n for n in cross))
            normal = tuple(n / length for n in cross)
            for i in face:
                adjacent.setdefault(i, []).append(normal)
        for normals in adjacent.values():
            if isinstance(solid, bt.parts.Box):
                assert all(n == pytest.approx(normals[0]) for n in normals)
            else:
                assert all(abs(n[2]) < 1e-8 for n in normals) or all(
                    n == pytest.approx(normals[0]) and abs(n[2]) == pytest.approx(1) for n in normals)


def test_screwdriver_is_an_envelope_with_its_own_stroke() -> None:
    drv = bt.tools.screwdriver()
    assert drv.joint_names == ["shank"] and drv.joint_limits == [(0.0, 0.055)]
    assert drv.link_names == ["mount", "body", "nose", "bit", "tip"]
    scene = bt.Scene(drv)
    (x, y, z), q = scene.link_pose("tip")
    # The tip at the bit's end, turned half a turn about X: +Z back along the tool.
    assert (x, y) == pytest.approx((0.0, 0.0)) and z == pytest.approx(0.311, abs=1e-6)
    assert abs(q[0]) == pytest.approx(1.0, abs=1e-9)
    scene.set_joint_positions([0.055])
    assert scene.link_pose("tip")[0][2] == pytest.approx(0.366, abs=1e-6)
    assert scene.check_collisions() == []
    fixed = bt.tools.screwdriver(stroke=None)
    assert fixed.dof == 0
    with pytest.raises(ValueError, match="stroke must be positive"):
        bt.tools.screwdriver(stroke=-1.0)

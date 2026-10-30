"""Formations: several vehicles carrying one load as one machine
(design-cooperative-transport.md K0). These tests need no catalog."""
import math

import botrail as bt
import pytest

# Two platforms under a 2.4 m beam, 0.8 m either side of its centre.
SLOTS = {"m1": (0.8, 0.0), "m2": (-0.8, 0.0)}
WHEELS = {"fl": (0.2, 0.15, -1), "fr": (0.2, -0.15, 1), "rl": (-0.2, 0.15, 1), "rr": (-0.2, -0.15, -1)}
# The formation: 4 m along +x, a quarter turn about the beam's centre,
# 4 m along +y — at 0.4 m/s and 0.4 rad/s, 10 + (π/2)/0.4 + 10 s.
CARRY = 20.0 + (math.pi / 2) / 0.4


def cell(*, member_drive="holonomic", slots=SLOTS, speed=0.4, turn_speed=0.4, wheels=False):
    scene = bt.Scene()
    for name, (x, y) in slots.items():
        scene.add_box(f"{name}/body", (0.5, 0.4, 0.2), (x, y, 0.1))
        if wheels:
            for w, (wx, wy, _) in WHEELS.items():
                scene.add_box(f"{name}/{w}", (0.12, 0.05, 0.12), (x + wx, y + wy, 0.06))
                scene.set_obstacle_enabled(f"{name}/{w}", False)
    # The beam rides the formation's own deck, 5 mm over the platforms'
    # bodies — clear of them as they drive out from under it at the end.
    scene.add_box("beam", (2.4, 0.3, 0.1), (0.0, 0.0, 0.255))
    scene.add_formation("f", members={m: (x, y) for m, (x, y) in slots.items()},
                        path=[(0, 0), (4, 0), (4, 4)], stations={"a": 0, "c": 2},
                        speed=speed, turn_speed=turn_speed,
                        tray_position=(0, 0, 0.25), tray_size=(0.4, 0.4, 0.2))
    # Each member's own path runs from its slot (facing the formation's
    # way: the first leg points +x) on to where the formation sets it down.
    for name, (x, y) in slots.items():
        there, _ = scene.formation_slot("f", name, "c")
        scene.add_vehicle(name, body=[name], path=[(x, y), (x + 1.0, y), there[:2]],
                          stations={"home": 0, "there": 2}, speed=0.5, turn_speed=1.0,
                          drive=member_drive)
        if wheels:
            for w, (_, _, lateral) in WHEELS.items():
                scene.set_vehicle_wheel(name, f"{name}/{w}", radius=0.06, lateral_ratio=lateral)
    sq = scene.sequence("carry")
    sq.step("carry", actions=[bt.seq.goto("f", "c")], transition=bt.seq.device_done("f"))
    sq.step("off", actions=[bt.seq.goto(m, "home") for m in slots],
            transition=bt.seq.all_of(*[bt.seq.device_done(m) for m in slots]))
    return scene


def planar(pose):
    (x, y, _), (qx, qy, qz, qw) = pose
    return x, y, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def relative(a, b):
    """Pose b in pose a's frame, planar: (x, y, yaw)."""
    ax, ay, ath = a
    bx, by, bth = b
    dx, dy = bx - ax, by - ay
    c, s = math.cos(ath), math.sin(ath)
    return c * dx + s * dy, -s * dx + c * dy, math.remainder(bth - ath, math.tau)


def test_members_ride_the_formation_and_drive_home():
    scene = cell()
    tl = scene.simulate_sequence("carry")
    carry = tl.step_span("carry")
    assert carry.end == pytest.approx(CARRY, abs=0.011)
    # Beam and platforms move as one: each body holds its place under the beam.
    for k in range(41):
        t = carry.end * k / 40
        beam = planar(tl.object_pose("beam", t))
        for name, (x, y) in SLOTS.items():
            rx, ry, rth = relative(beam, planar(tl.object_pose(f"{name}/body", t)))
            assert (rx, ry, rth) == pytest.approx((x, y, 0.0), abs=1e-9), (name, t)
    beam = planar(tl.object_pose("beam", carry.end))
    assert beam == pytest.approx((4.0, 4.0, math.pi / 2), abs=1e-9)
    # Set down on their own paths, they drive home on their own.
    assert tl.duration > carry.end + 1.0
    for name, (x, y) in SLOTS.items():
        assert planar(tl.object_pose(f"{name}/body", tl.duration))[:2] == pytest.approx((x, y), abs=1e-9)


def test_formation_slot_is_where_the_goto_parks():
    scene = cell()
    (x, y, z), yaw = scene.formation_slot("f", "m1", "c")
    assert (x, y, z, yaw) == pytest.approx((4.0, 4.8, 0.0, math.pi / 2), abs=1e-12)
    (x, y, _), yaw = scene.formation_slot("f", "m2", "a")
    assert (x, y, yaw) == pytest.approx((-0.8, 0.0, 0.0), abs=1e-12)
    with pytest.raises(ValueError, match="no member `m3`"):
        scene.formation_slot("f", "m3", "c")
    with pytest.raises(ValueError, match="no station `z`"):
        scene.formation_slot("f", "m1", "z")


def test_members_wheels_turn_while_they_ride():
    tl = cell(wheels=True).simulate_sequence("carry")
    end = tl.step_span("carry").end
    # Parked, the wheel stands as placed; carried, it has rolled.
    _, before = tl.object_pose("m1/fl", 0.0)
    _, during = tl.object_pose("m1/fl", end / 2)
    body_yaw = planar(tl.object_pose("m1/body", end / 2))[2]
    spin = abs(sum(a * b for a, b in zip(during, (0.0, 0.0, math.sin(body_yaw / 2), math.cos(body_yaw / 2)))))
    assert before == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-12)
    assert spin < 0.999, "the wheel never rolled while its vehicle rode the formation"


def test_differential_members_are_never_dragged_sideways():
    with pytest.raises(ValueError, match=r"member `m1` \(differential\) would slide sideways"):
        cell(member_drive="differential").simulate_sequence("carry")
    # Side by side they turn like the wheels of one large differential drive.
    side = {"m1": (0.0, 0.8), "m2": (0.0, -0.8)}
    tl = cell(member_drive="differential", slots=side).simulate_sequence("carry")
    assert tl.step_span("carry").end == pytest.approx(CARRY, abs=0.011)


def test_rates_are_held_to_the_members():
    with pytest.raises(ValueError, match=r"formation `f` drives at 0\.600 m/s, over member `m1`'s 0\.500 m/s"):
        cell(speed=0.6).simulate_sequence("carry")
    with pytest.raises(ValueError, match=r"turn_speed ≤ 0\.625 rad/s"):
        cell(turn_speed=0.8).simulate_sequence("carry")


def test_bad_members_are_refused():
    scene = bt.Scene()
    scene.add_box("cart", (0.5, 0.4, 0.2), (0, 0, 0.1))
    with pytest.raises(ValueError, match="members is empty"):
        scene.add_formation("f", members={}, path=[(0, 0), (1, 0)], stations={"a": 0, "b": 1})
    with pytest.raises(ValueError, match=r"a slot is \(x, y\) or \(x, y, yaw\)"):
        scene.add_formation("f", members={"cart": (0,)}, path=[(0, 0), (1, 0)], stations={"a": 0, "b": 1})
    # Members are checked when the cell is simulated: they may be added later.
    scene.add_formation("f", members={"ghost": (0, 0)}, path=[(0, 0), (1, 0)], stations={"a": 0, "b": 1})
    scene.sequence("go").step("go", actions=[bt.seq.goto("f", "b")], transition=bt.seq.device_done("f"))
    with pytest.raises(ValueError, match="formation `f` names unknown member `ghost`"):
        scene.simulate_sequence("go")


def test_a_formation_is_no_line_of_the_bom():
    names = {name for row in cell().bom().rows for name in row["names"]}
    assert "f" not in names and {"m1", "m2"} <= names


def test_a_formation_round_trips_through_a_project(tmp_path):
    scene = cell()
    baseline = scene.simulate_sequence("carry").duration
    source = scene.generate_python()
    assert 'scene.add_formation("f", members={"m1": (0.8, 0.0, 0.0), "m2": (-0.8, 0.0, 0.0)}' in source
    scene.save_project(tmp_path / "cell.botrail")
    loaded = bt.Scene.load_project(tmp_path / "cell.botrail")
    assert loaded.simulate_sequence("carry").duration == baseline


# ------------------------------------------------------------ load sharing
PLATFORM = """<?xml version="1.0"?>
<robot name="platform">
  <link name="base_footprint"/>
  <link name="chassis"><collision><origin xyz="0 0 0.1"/><geometry><box size="0.5 0.4 0.2"/></geometry></collision></link>
  <link name="plate"><collision><origin xyz="0 0 0.01"/><geometry><box size="0.4 0.3 0.02"/></geometry></collision></link>
  <link name="deck"/>
  <joint name="base" type="fixed"><parent link="base_footprint"/><child link="chassis"/></joint>
  <joint name="lift" type="prismatic"><parent link="chassis"/><child link="plate"/><origin xyz="0 0 0.2"/>
    <axis xyz="0 0 1"/><limit lower="0" upper="0.05" effort="1" velocity="0.05"/></joint>
  <joint name="deck_joint" type="fixed"><parent link="plate"/><child link="deck"/><origin xyz="0 0 0.02"/></joint>
</robot>
"""


def crew(slots, *, offset=0.0, mass=900.0):
    """Platforms (one robot each) under a beam held by the first: `offset`
    moves the beam's centre of mass along +x."""
    scene = bt.Scene()
    model = bt.Robot.from_urdf_string(PLATFORM)
    scene.add_formation("f", members=slots, path=[(0, 0), (4, 0)], stations={"a": 0, "b": 1})
    for name in slots:
        (x, y, _), _ = scene.formation_slot("f", name, "a")
        scene.add_vehicle(name, body=[], path=[(x - 1.0, y), (x, y)], stations={"home": 0, "under": 1},
                          drive="holonomic", start="under")
        scene.add_robot(model, name=f"{name}_unit")
        scene.mount_robot(name, robot=f"{name}_unit")
    scene.add_box("beam", (3.0, 0.3, 0.1), (offset, 0.0, 0.27))
    scene.set_part("beam", mass_kg=mass)
    first = f"{next(iter(slots))}_unit"
    scene.sequence("carry").step("hold", actions=[bt.seq.attach("beam", link="deck", robot=first)],
                                 transition=bt.seq.immediately())
    return scene


def asked(req, robot):
    return {r.key: r for r in req[robot].requirements}


def test_each_platform_asks_its_share_not_the_whole_load():
    req = bt.select.requirements(crew({"m1": (0.8, 0.0), "m2": (-0.8, 0.0)}, offset=0.2))
    # The lever: 900 kg at +0.2 between supports at ±0.8 — 562.5 and 337.5.
    one, two = asked(req, "m1_unit"), asked(req, "m2_unit")
    assert one["payload_kg"].value == pytest.approx(562.5) and two["payload_kg"].value == pytest.approx(337.5)
    assert "of beam 900 kg on 2 platforms of formation `f`" in one["payload_kg"].basis
    assert one["cooperative_units"].value == 2 and one["cooperative_payload_kg"].value == 900
    # The holder asks no more than its crewmate would for the same place.
    assert not req.problems


def test_a_load_off_its_supports_is_an_error():
    req = bt.select.requirements(crew({"m1": (0.8, 0.0), "m2": (-0.8, 0.0)}, offset=1.2))
    assert not req.ok
    (problem,) = req.problems
    assert problem.code == "load_off_support" and "`m2` (-225 kg) would lift off" in problem.message
    assert problem in req.findings()


def test_four_platforms_share_by_a_plane():
    slots = {"m1": (0.8, 0.5), "m2": (0.8, -0.5), "m3": (-0.8, 0.5), "m4": (-0.8, -0.5)}
    req = bt.select.requirements(crew(slots, offset=0.4, mass=1000.0))
    shares = {m: asked(req, f"{m}_unit")["payload_kg"].value for m in slots}
    # 250 each, plus 1000 · 0.4 · x / Σx² (= 4 · 0.8²) along the length: ±125.
    assert shares == pytest.approx({"m1": 375.0, "m2": 375.0, "m3": 125.0, "m4": 125.0})
    assert "equally stiff supports" in asked(req, "m1_unit")["payload_kg"].basis


def test_load_shares_lever_statics_and_plane():
    from botrail.select import _load_shares
    assert _load_shares([(1.0, 0.0), (-1.0, 0.0)], 100.0, (0.5, 0.0)) == pytest.approx([75.0, 25.0])
    # Collinear supports are a lever whatever their count; three off a line are exact.
    assert sum(_load_shares([(0, 0), (1, 0), (2, 0)], 90.0, (1.5, 0.0))) == pytest.approx(90.0)
    three = _load_shares([(0, 0), (2, 0), (0, 2)], 300.0, (0.5, 0.5))
    assert three == pytest.approx([150.0, 75.0, 75.0])

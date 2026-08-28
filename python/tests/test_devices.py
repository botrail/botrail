import math
from pathlib import Path

import pytest

import botrail as bt

SQ2 = math.sqrt(0.5)

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def scene() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))


def test_conveyor_feed_sensor_stop_cycle(scene: bt.Scene) -> None:
    # A box rides a conveyor along +x (well away from the arm) until it
    # trips a beam; the sequence then stops the belt.
    scene.add_box("crate", (0.04, 0.04, 0.04), (-0.5, 0.6, 0.3))
    scene.add_conveyor(
        "belt",
        zone_position=(-0.2, 0.6, 0.3),
        zone_size=(1.2, 0.3, 0.3),
        velocity=(0.25, 0.0, 0.0),
        running=False,
    )
    scene.add_beam_sensor("eye", frm=(0.0, 0.4, 0.3), to=(0.0, 0.8, 0.3))
    assert scene.sensor_names == ["eye"]
    assert scene.device_names == ["belt"]

    sq = scene.sequence("feed")
    sq.step("feed", actions=[bt.seq.start("belt")], transition=bt.seq.signal("eye"))
    sq.step("stop", actions=[bt.seq.stop("belt")], transition=bt.seq.elapsed(0.1))
    tl = sq.simulate()

    # Analytic trip time: 0.475 m at 0.25 m/s.
    feed_end = tl.step_spans[0][2]
    assert abs(feed_end - 1.9) <= 0.011
    lanes = dict(tl.signals)
    assert [v for _, v in lanes["eye"]] == [False, True]
    assert [v for _, v in lanes["belt"]] == [False, True, False]
    # The crate travelled with the belt and settles once stopped.
    p_end, _ = tl.object_pose("crate", tl.duration)
    assert abs((p_end[0] - (-0.5)) - 0.25 * feed_end) < 1e-9
    # The live scene is untouched.
    pos, _ = scene.obstacle_pose("crate")
    assert pos[0] == -0.5


def test_linear_axis_and_project_roundtrip(scene: bt.Scene, tmp_path: Path) -> None:
    scene.add_box("door", (0.1, 0.1, 0.1), (0.6, 0.0, 0.2))
    scene.add_linear_axis(
        "lift", objects=["door"], axis=(0, 0, 1), speed=0.5, range=(0.0, 0.4)
    )
    sq = scene.sequence("open")
    sq.step(
        "raise",
        actions=[bt.seq.move_to("lift", 0.3)],
        transition=bt.seq.device_done("lift"),
    )
    tl = sq.simulate()
    assert abs(tl.step_spans[0][2] - 0.6) <= 0.011
    p_end, _ = tl.object_pose("door", tl.duration)
    assert abs(p_end[2] - 0.5) < 1e-9

    # Sensors/devices round-trip through the project file and codegen.
    scene.add_zone_sensor("mat", position=(0.5, 0.0, 0.1), size=(0.4, 0.4, 0.2))
    path = tmp_path / "cell.botrail"
    scene.save_project(path)
    reloaded = bt.Scene.load_project(path)
    assert reloaded.sensor_names == ["mat"]
    assert reloaded.device_names == ["lift"]
    code = scene.generate_python()
    for needle in (
        'scene.add_zone_sensor("mat"',
        'scene.add_linear_axis("lift"',
        'bt.seq.move_to("lift", 0.3)',
        'bt.seq.device_done("lift")',
    ):
        assert needle in code, f"missing {needle}:\n{code}"


def test_tracking_pick_follows_a_moving_part(scene: bt.Scene) -> None:
    # Conveyor tracking: the taught grasp is met while the belt keeps
    # running, and grasping the part freezes the offset so the lift after it
    # does not drag the part back to where it was taught.
    scene.set_joint_positions([0.0, -0.4, 0.9, 0.0, 0.9, 0.0])
    tcp = scene.robot.tcp_link
    start, _ = scene.link_pose(tcp)
    scene.add_box("crate", (0.04, 0.04, 0.04), (start[0], start[1], start[2] - 0.1))
    scene.add_conveyor(
        "belt",
        zone_position=(start[0] + 0.5, start[1], start[2] - 0.1),
        zone_size=(1.4, 0.3, 0.3),
        velocity=(0.1, 0.0, 0.0),
        running=True,
    )

    sq = scene.sequence("track")
    sq.step("latch", actions=[bt.seq.track("crate")])
    sq.step("follow", transition=bt.seq.elapsed(0.5))
    sq.step("grasp", actions=[bt.seq.attach("crate", link=tcp)])
    sq.step("hold", transition=bt.seq.elapsed(0.4))
    sq.step("release", actions=[bt.seq.untrack()])
    tl = sq.simulate()

    # The tool held station over the part for the whole tracked stretch.
    for t in (0.0, 0.25, 0.5):
        scene.set_joint_positions(tl.sample(t))
        tool, _ = scene.link_pose(tcp)
        part, _ = tl.object_pose("crate", t)
        assert abs((tool[0] - part[0])) < 1e-4, t
        assert abs(part[0] - (start[0] + 0.1 * t)) < 1e-9, t

    # Grasping the tracked part freezes the sync: the belt is still running,
    # but the part is the robot's now and the arm stops chasing it.
    caught, _ = tl.object_pose("crate", 0.5)
    end, _ = tl.object_pose("crate", tl.duration)
    assert tl.duration >= 0.9
    assert max(abs(a - b) for a, b in zip(end, caught)) < 1e-9
    scene.set_joint_positions(tl.sample(tl.duration))
    tool, _ = scene.link_pose(tcp)
    assert abs(tool[0] - end[0]) < 1e-4  # still in the gripper


def test_tracking_rules_are_reported(scene: bt.Scene) -> None:
    scene.add_box("crate", (0.04, 0.04, 0.04), (0.4, 0.0, 0.2))
    scene.add_segment("go", goal=[0.2, -0.4, 0.9, 0.0, 0.9, 0.0])
    sq = scene.sequence("bad")
    sq.step("latch", actions=[bt.seq.track("crate")])
    sq.step("move", actions=[bt.seq.motion("go")])
    with pytest.raises(ValueError, match="release the track first"):
        sq.simulate()


def test_vehicle_goto_carries_the_body(scene: bt.Scene) -> None:
    # An L-shaped guide path well away from the arm: 2 m along +x, a 90°
    # pivot, then 1 m along +y — 4 + 1 + 2 s at 0.5 m/s and 90°/s. The
    # body is named by prefix and rides the frame rigidly, rotation included.
    scene.add_box("agv/chassis", (0.2, 0.2, 0.2), (0.3, 2.2, 0.1))
    scene.add_box("agv/mast", (0.05, 0.05, 0.3), (0.3, 2.2, 0.35))
    scene.add_vehicle(
        "agv",
        body=["agv"],
        path=[(0.0, 2.0), (2.0, 2.0), (2.0, 3.0)],
        stations={"dock": 0, "warehouse": 2},
        speed=0.5,
        turn_speed=math.pi / 2,
        start="dock",
    )
    assert "agv" in scene.device_names

    sq = scene.sequence("haul")
    sq.step(
        "out",
        actions=[bt.seq.goto("agv", "warehouse")],
        transition=bt.seq.device_done("agv"),
    )
    tl = sq.simulate()

    assert abs(tl.duration - 7.0) <= 0.011
    lanes = dict(tl.signals)
    assert [v for _, v in lanes["agv"]] == [False, True, False]
    # Net rigid motion: +2 x, pivot +90° about (2, 2), +1 y.
    p, q = tl.object_pose("agv/chassis", tl.duration)
    assert max(abs(a - b) for a, b in zip(p, (1.8, 3.3, 0.1))) < 1e-9
    assert abs(q[2] - SQ2) < 1e-9 and abs(q[3] - SQ2) < 1e-9
    # Mid-turn sample is the closed form, not a resample grid.
    p, q = tl.object_pose("agv/chassis", 4.5)
    phi = math.pi / 4
    expected = (
        2.0 + 0.3 * math.cos(phi) - 0.2 * math.sin(phi),
        2.0 + 0.3 * math.sin(phi) + 0.2 * math.cos(phi),
        0.1,
    )
    assert max(abs(a - b) for a, b in zip(p, expected)) < 1e-9
    # The live scene is untouched.
    pos, _ = scene.obstacle_pose("agv/chassis")
    assert pos[0] == 0.3

    # The generated script re-authors the vehicle and the dispatch.
    code = scene.generate_python()
    assert 'scene.add_vehicle("agv"' in code
    assert 'bt.seq.goto("agv", "warehouse")' in code


def test_vehicle_authoring_errors(scene: bt.Scene) -> None:
    scene.add_box("cart", (0.1, 0.1, 0.1), (0.0, 2.0, 0.05))
    with pytest.raises(ValueError, match="matches no obstacle"):
        scene.add_vehicle(
            "agv", body=["ghost"], path=[(0.0, 2.0), (1.0, 2.0)], stations={"a": 0}
        )
    with pytest.raises(ValueError, match="points at waypoint"):
        scene.add_vehicle(
            "agv", body=["cart"], path=[(0.0, 2.0), (1.0, 2.0)], stations={"a": 9}
        )
    with pytest.raises(ValueError, match="not a station"):
        scene.add_vehicle(
            "agv",
            body=["cart"],
            path=[(0.0, 2.0), (1.0, 2.0)],
            stations={"a": 0},
            start="b",
        )
    # A second goto while travelling is a sequencing error.
    scene.add_vehicle(
        "agv", body=["cart"], path=[(0.0, 2.0), (1.0, 2.0)], stations={"a": 0, "b": 1}
    )
    sq = scene.sequence("amend")
    sq.step("go", actions=[bt.seq.goto("agv", "b")], transition=bt.seq.elapsed(0.5))
    sq.step("again", actions=[bt.seq.goto("agv", "a")], transition=bt.seq.device_done("agv"))
    with pytest.raises(ValueError, match="still travelling"):
        sq.simulate()


def test_vehicle_tray_carries_what_is_set_on_it(scene: bt.Scene) -> None:
    # A deck 0.3 m up, and a carton resting on it. Nothing declares the
    # carton as cargo — being in the zone is what makes it cargo.
    scene.add_box("agv/base", (0.6, 0.4, 0.3), (0.0, 2.0, 0.15))
    scene.add_box("crate", (0.1, 0.1, 0.1), (0.0, 2.0, 0.35))
    scene.add_box("bystander", (0.1, 0.1, 0.1), (0.0, 2.8, 0.05))
    scene.add_vehicle(
        "agv",
        body=["agv"],
        path=[(0.0, 2.0), (2.0, 2.0)],
        stations={"a": 0, "b": 1},
        speed=0.5,
        start="a",
        tray_position=(0.0, 0.0, 0.35),
        tray_size=(0.6, 0.4, 0.2),
    )
    # Load-present, riding along; and the same zone bolted to the floor.
    scene.add_zone_sensor("loaded", position=(0.0, 0.0, 0.35), size=(0.6, 0.4, 0.2),
                          watch=["crate"], mount="agv")
    scene.add_zone_sensor("at_a", position=(0.0, 2.0, 0.35), size=(0.6, 0.4, 0.2),
                          watch=["crate"])

    sq = scene.sequence("haul")
    sq.step("go", actions=[bt.seq.goto("agv", "b")], transition=bt.seq.device_done("agv"))
    tl = sq.simulate()

    assert abs(tl.duration - 4.0) <= 0.011  # 2 m at 0.5 m/s
    p_end, _ = tl.object_pose("crate", tl.duration)
    assert max(abs(a - b) for a, b in zip(p_end, (2.0, 2.0, 0.35))) < 1e-9
    # The bystander was never cargo.
    with pytest.raises(ValueError, match="crate|bystander|unknown"):
        tl.object_pose("bystander", tl.duration)
    # The mounted eye keeps its load; the floor fixture loses it.
    lanes = dict(tl.signals)
    assert [v for _, v in lanes["loaded"]] == [False, True]
    assert [v for _, v in lanes["at_a"]] == [False, True, False]
    assert tl.signal("loaded").value_at(tl.duration)
    assert not tl.signal("at_a").value_at(tl.duration)


def test_vehicle_climbs_a_ramp_with_a_declared_grade(scene: bt.Scene) -> None:
    # A 4 m run rising 0.4 m — a 10 % ramp. Without a declared ability the
    # slope is refused by name; with one, the whole machine rides up it and
    # cruise speed is spent along the 3D path.
    scene.add_box("agv/chassis", (0.2, 0.2, 0.2), (0.3, 2.0, 0.1))
    ramp = [(0.0, 2.0), (4.0, 2.0, 0.4)]
    scene.add_vehicle(
        "agv", body=["agv"], path=ramp, stations={"a": 0, "b": 1}, speed=0.5
    )
    sq = scene.sequence("haul")
    sq.step("go", actions=[bt.seq.goto("agv", "b")], transition=bt.seq.device_done("agv"))
    with pytest.raises(ValueError, match="declares no max_grade"):
        sq.simulate()

    # Too weak a drive is named with both numbers.
    scene.add_vehicle(
        "agv", body=["agv"], path=ramp, stations={"a": 0, "b": 1}, speed=0.5,
        max_grade=0.05,
    )
    with pytest.raises(ValueError, match="over the drive's max_grade"):
        sq.simulate()

    scene.add_vehicle(
        "agv", body=["agv"], path=ramp, stations={"a": 0, "b": 1}, speed=0.5,
        max_grade=0.2,
    )
    tl = sq.simulate()
    assert abs(tl.duration - math.hypot(4.0, 0.4) / 0.5) <= 0.011
    p, _ = tl.object_pose("agv/chassis", tl.duration)
    assert max(abs(a - b) for a, b in zip(p, (4.3, 2.0, 0.5))) < 1e-9
    # Halfway through the drive the chassis is halfway up the ramp.
    p, _ = tl.object_pose("agv/chassis", tl.duration / 2)
    assert abs(p[2] - 0.3) < 1e-3
    # The generated script round-trips the 3D waypoint and the grade.
    code = scene.generate_python()
    assert "(4, 2, 0.4)" in code and "max_grade=0.2" in code

    # Straight up is never a drive's job (that is a lift's).
    scene.add_vehicle(
        "agv", body=["agv"], path=[(0.0, 2.0), (0.0, 2.0, 1.0)],
        stations={"a": 0, "b": 1}, max_grade=5.0,
    )
    with pytest.raises(ValueError, match="vertical"):
        sq.simulate()


def test_a_holonomic_vehicle_holds_its_heading(scene: bt.Scene) -> None:
    # Mecanum wheels: the L-path costs only its length, and the body
    # arrives unrotated — it docks facing what it faced when parked.
    scene.add_box("agv/chassis", (0.2, 0.2, 0.2), (0.3, 2.2, 0.1))
    scene.add_vehicle(
        "agv", body=["agv"], path=[(0.0, 2.0), (2.0, 2.0), (2.0, 3.0)],
        stations={"dock": 0, "warehouse": 2}, speed=0.5, drive="holonomic",
    )
    sq = scene.sequence("haul")
    sq.step("out", actions=[bt.seq.goto("agv", "warehouse")],
            transition=bt.seq.device_done("agv"))
    tl = sq.simulate()
    assert abs(tl.duration - 6.0) <= 0.011
    _p, q = tl.object_pose("agv/chassis", tl.duration)
    assert q == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-9)
    assert 'drive="holonomic"' in scene.generate_python()
    # The kwargs guard each other: reversing is a differential-drive idea.
    with pytest.raises(ValueError, match="never turns"):
        scene.add_vehicle(
            "agv2", body=["agv"], path=[(0.0, 2.0), (1.0, 2.0)],
            stations={"a": 0}, drive="holonomic", allow_reverse=True,
        )


def test_vehicle_reverses_out_instead_of_turning(scene: bt.Scene) -> None:
    # Out and back on a straight run. Turning around costs two 180 deg
    # pivots; reversing costs none, and the machine ends facing the way it
    # started — which is what a dead-end dock forces.
    scene.add_box("agv/base", (0.4, 0.3, 0.2), (0.0, 2.0, 0.1))
    common = dict(body=["agv"], path=[(0.0, 2.0), (1.0, 2.0)],
                  stations={"a": 0, "b": 1}, speed=0.5,
                  turn_speed=math.pi / 2, start="a")

    def cycle(**extra) -> float:
        scene.add_vehicle("agv", **common, **extra)
        sq = scene.sequence("cycle")
        sq.step("out", actions=[bt.seq.goto("agv", "b")],
                transition=bt.seq.device_done("agv"))
        sq.step("back", actions=[bt.seq.goto("agv", "a")],
                transition=bt.seq.device_done("agv"))
        tl = sq.simulate()
        _, q = tl.object_pose("agv/base", tl.duration)
        return tl.duration, q

    turning, q_turn = cycle()
    reversing, q_back = cycle(allow_reverse=True)
    # 2 x 2 s of driving, plus a 180 deg about-face (2 s) only when turning.
    assert abs(turning - 6.0) <= 0.011, turning
    assert abs(reversing - 4.0) <= 0.011, reversing
    # Turning around leaves it facing back the way it came; reversing does
    # not turn it at all.
    assert abs(abs(q_turn[2]) - 1.0) < 1e-6, q_turn   # yaw = pi
    assert abs(q_back[2]) < 1e-9, q_back              # yaw = 0


def test_mounted_robot_base_rides_the_vehicle(scene: bt.Scene) -> None:
    # An arm on a chassis: its base stops being a scene constant.
    scene.add_box("agv/base", (0.6, 0.4, 0.3), (0.0, 2.0, 0.15))
    scene.add_vehicle(
        "amr",
        body=["agv"],
        path=[(0.0, 2.0), (2.0, 2.0)],
        stations={"a": 0, "b": 1},
        speed=0.5,
        start="a",
    )
    scene.mount_robot("amr", offset_position=(0.0, 0.0, 0.3))
    # Mounting places it on the parked vehicle at once.
    pos, _ = scene.robot_base_pose
    assert max(abs(a - b) for a, b in zip(pos, (0.0, 2.0, 0.3))) < 1e-12

    sq = scene.sequence("go")
    sq.step("drive", actions=[bt.seq.goto("amr", "b")], transition=bt.seq.device_done("amr"))
    sq.step("dwell", transition=bt.seq.elapsed(1.0))
    tl = sq.simulate()

    # The base track exists and keeps a constant offset to the body it is
    # bolted to (the chassis box is drawn about its own centre, 0.15 up).
    for t in (0.0, 2.0, 4.0, tl.duration):
        base, _ = tl.base_pose(t)
        body, _ = tl.object_pose("agv/base", t)
        offset = tuple(a - b for a, b in zip(base, body))
        assert max(abs(a - b) for a, b in zip(offset, (0.0, 0.0, 0.15))) < 1e-9, t
    end, _ = tl.base_pose(tl.duration)
    assert max(abs(a - b) for a, b in zip(end, (2.0, 2.0, 0.3))) < 1e-9


def test_plan_while_driving_is_rejected_but_a_ramp_is_not(scene: bt.Scene) -> None:
    scene.add_box("agv/base", (0.6, 0.4, 0.3), (0.0, 2.0, 0.15))
    scene.add_vehicle("amr", body=["agv"], path=[(0.0, 2.0), (2.0, 2.0)],
                      stations={"a": 0, "b": 1}, speed=0.5, start="a")
    scene.mount_robot("amr", offset_position=(0.0, 0.0, 0.3))
    scene.add_segment("reach", goal=[0.2, -0.4, 0.9, 0.0, 0.9, 0.0])

    # A ramp alongside the drive: fine, and it actually moves the arm.
    joint = scene.robot.joint_names[1]
    sq = scene.sequence("stow")
    sq.step(
        "drive",
        actions=[bt.seq.goto("amr", "b"), bt.seq.ramp({joint: -0.5}, 1.0)],
        transition=bt.seq.device_done("amr"),
    )
    tl = sq.simulate()
    assert abs(tl.sample(tl.duration)[1] - (-0.5)) < 1e-9

    # A planned motion in the same step: rejected, with the reason.
    sq = scene.sequence("bad")
    sq.step(
        "drive",
        actions=[bt.seq.goto("amr", "b"), bt.seq.motion("reach")],
        transition=bt.seq.device_done("amr"),
    )
    with pytest.raises(ValueError, match="cannot start while `amr` is driving"):
        sq.simulate()

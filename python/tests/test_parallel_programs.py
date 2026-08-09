"""Parallel programs (`simulate_sequences`) and the indexed transfer
(`bt.seq.advance`) — the W1 engine work, asserted at the Python surface.

One scan tick advances every program in list order over one shared world;
programs coordinate through signals and sensors, never by sharing outputs
(single-owner rule); and a conveyor pitch is a *distance* that lands exact
no matter how the scan period divides it.
"""

import json

import pytest

import botrail as bt

URDF = """
<robot name="r"><link name="a"/><link name="b"/>
<joint name="j" type="revolute"><parent link="a"/><child link="b"/>
<origin xyz="0 0 0.5"/><axis xyz="0 0 1"/>
<limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>
"""


@pytest.fixture()
def scene() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf_string(URDF))
    scene.add_box("/box", (0.1, 0.1, 0.1), (0.0, 2.0, 0.5))
    scene.add_conveyor("belt", zone_position=(5.0, 2.0, 0.5),
                       zone_size=(20.0, 1.0, 1.0),
                       velocity=(0.4, 0.0, 0.0), running=False)
    return scene


def test_programs_overlap_and_the_total_is_the_max(scene: bt.Scene) -> None:
    work = scene.sequence("work")
    work.step("ramp", actions=[bt.seq.ramp({"j": 0.8}, 1.0)])
    index = scene.sequence("index")
    index.step("advance", actions=[bt.seq.advance("belt", 0.2)],
               transition=bt.seq.device_done("belt"))
    tl = scene.simulate_sequences(["work", "index"])
    # 1.0 s of ramp beside 0.5 s of indexing: parallel is the max, and the
    # step spans carry their program.
    assert tl.duration == pytest.approx(1.0, abs=0.02)
    names = {s for s, _, _ in tl.step_spans}
    assert names == {"work/ramp", "index/advance"}


def test_a_signal_is_the_barrier_between_programs(scene: bt.Scene) -> None:
    scene.define_signal("welded", False)
    work = scene.sequence("work")
    work.step("ramp", actions=[bt.seq.ramp({"j": 0.8}, 1.0)])
    work.step("flag", actions=[bt.seq.set_signal("welded", True)])
    index = scene.sequence("index")
    index.step("gate", transition=bt.seq.signal("welded", True))
    index.step("advance", actions=[bt.seq.advance("belt", 0.2)],
               transition=bt.seq.device_done("belt"))
    tl = scene.simulate_sequences(["work", "index"])
    at = {s: (a, b) for s, a, b in tl.step_spans}
    assert at["index/advance"][0] == pytest.approx(at["work/ramp"][1], abs=0.02)
    assert tl.duration == pytest.approx(1.5, abs=0.03)


def test_advance_lands_exactly_even_on_an_odd_pitch(scene: bt.Scene) -> None:
    # 0.123 m is not a multiple of v*dt = 4 mm: the final scan moves the
    # 3 mm remainder, which the old start/elapsed/stop pattern always lost.
    sq = scene.sequence("index")
    sq.step("advance", actions=[bt.seq.advance("belt", 0.123)],
            transition=bt.seq.device_done("belt"))
    tl = scene.simulate_sequence("index")
    x = tl.object_pose("/box", tl.duration)[0][0]
    assert x == pytest.approx(0.123, abs=1e-12)


def test_two_programs_may_not_drive_one_resource(scene: bt.Scene) -> None:
    a = scene.sequence("a")
    a.step("x", actions=[bt.seq.ramp({"j": 0.5}, 0.2)])
    b = scene.sequence("b")
    b.step("y", actions=[bt.seq.ramp({"j": -0.5}, 0.2)])
    with pytest.raises(ValueError, match="commanded by both `a` and `b`"):
        scene.simulate_sequences(["a", "b"])

    scene.define_signal("flag", False)
    for name in ("c", "d"):
        sq = scene.sequence(name)
        sq.step("w", actions=[bt.seq.set_signal("flag", True)])
    with pytest.raises(ValueError, match="signal `flag` is commanded by both"):
        scene.simulate_sequences(["c", "d"])


def test_the_timeout_names_every_stuck_program(scene: bt.Scene) -> None:
    scene.define_signal("never", False)
    for name in ("st1", "st2"):
        sq = scene.sequence(name)
        sq.step("gate", transition=bt.seq.signal("never", True))
    with pytest.raises(ValueError, match=r"st1/gate.*st2/gate"):
        scene.simulate_sequences(["st1", "st2"], max_duration=0.5)


def test_advance_survives_the_project_round_trip(scene: bt.Scene,
                                                 tmp_path) -> None:
    sq = scene.sequence("index")
    sq.step("advance", actions=[bt.seq.advance("belt", 5.2)],
            transition=bt.seq.device_done("belt"))
    path = tmp_path / "line.btproj"
    scene.save_project(path)
    code = bt.Scene.load_project(path).generate_python()
    assert 'bt.seq.advance("belt", 5.2)' in code
    assert 'bt.seq.device_done("belt")' in code

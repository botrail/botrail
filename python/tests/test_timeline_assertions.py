"""Assertion-layer queries on a baked timeline.

`step_span` / `signal` / `min_clearance` are the thin skin that turns
deterministic baking into pytest-able cell checks: cycle-time budgets, step
deadlines, signal handshakes, and the clearance the rollout itself never
measures (tracking ticks and conveyed parts are not collision-checked).
"""

from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def scene() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    tcp, _ = scene.link_pose(scene.robot.tcp_link)
    # 20 mm above the tool box (tool0 spans +0.04; the box's lower face sits
    # at +0.06): grasped without touching, so post-drop clearance is 0.02.
    scene.add_box("held", (0.04, 0.04, 0.04), (tcp[0], tcp[1], tcp[2] + 0.08))
    return scene


def _cycle(scene: bt.Scene):
    start = list(scene.joint_positions)
    scene.add_segment("swing", goal=[1.2, -0.5, 0.6, 0.0, 0.3, 0.0])
    scene.add_segment("home", goal=start)
    scene.define_signal("carrying")
    sq = scene.sequence("cycle")
    sq.step("grasp", actions=[bt.seq.attach("held"), bt.seq.set_signal("carrying")])
    sq.step("swing", actions=[bt.seq.motion("swing")])
    sq.step(
        "drop",
        actions=[bt.seq.detach("held"), bt.seq.set_signal("carrying", False)],
    )
    sq.step("settle", transition=bt.seq.elapsed(0.3))
    sq.step("home", actions=[bt.seq.motion("home")])
    return sq.simulate()


def test_step_span_lookup(scene: bt.Scene) -> None:
    tl = _cycle(scene)
    span = tl.step_span("settle")
    assert span.name == "settle"
    assert span.start == tl.step_spans[3][1]
    assert span.end == tl.step_spans[3][2]
    # The 0.3 s timer quantizes up to one scan tick.
    assert 0.3 - 1e-9 <= span.duration <= 0.31 + 1e-9
    assert "settle" in repr(span)
    with pytest.raises(ValueError, match="unknown step `nope`.*`grasp`"):
        tl.step_span("nope")


def test_signal_track_queries(scene: bt.Scene) -> None:
    tl = _cycle(scene)
    lane = tl.signal("carrying")
    assert lane.name == "carrying"

    rising = lane.rising_edges()
    falling = lane.falling_edges()
    assert len(rising) == 1 and len(falling) == 1
    assert rising[0] == tl.step_span("grasp").start
    assert falling[0] == tl.step_span("drop").start

    swing = tl.step_span("swing")
    assert lane.value_at((swing.start + swing.end) / 2) is True
    assert lane.value_at(tl.duration) is False

    assert lane.high_spans() == [(rising[0], falling[0])]
    assert lane.high_total() == pytest.approx(falling[0] - rising[0])
    assert "carrying" in repr(lane)

    with pytest.raises(ValueError, match="unknown signal `ghost`.*`carrying`"):
        tl.signal("ghost")


def test_min_clearance_measures_the_cycle(scene: bt.Scene) -> None:
    tl = _cycle(scene)
    c = tl.min_clearance()
    # While carried, `held` is on the robot side (nothing to measure); the
    # tightest approach comes after the drop — the 20 mm grasp offset, or
    # whatever margin the home plan keeps past the frozen box.
    assert 0.0 < float(c) <= 0.02 + 1e-9
    assert tl.step_span("drop").start <= c.t <= tl.duration
    assert c.pair is None
    # Compares like its distance, in both spellings.
    assert c >= c.distance and c <= c.distance
    assert c > 0.0 and not (c < 0.0)
    assert "m at t=" in repr(c)


def test_min_clearance_contact_names_the_pair() -> None:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    # A conveyed crate drives straight through the robot base: no motion is
    # planned, so nothing in the rollout objects — only the clearance scan
    # sees it. Contact starts at x = -(0.075 + 0.02) → t = 0.405/0.25.
    scene.add_box("crate", (0.04, 0.04, 0.04), (-0.5, 0.0, 0.06))
    scene.add_conveyor(
        "belt",
        zone_position=(0.0, 0.0, 0.06),
        zone_size=(1.4, 0.3, 0.2),
        velocity=(0.25, 0.0, 0.0),
        running=False,
    )
    sq = scene.sequence("feed")
    sq.step("run", actions=[bt.seq.start("belt")], transition=bt.seq.elapsed(3.0))
    tl = sq.simulate()

    c = tl.min_clearance()
    assert c == 0.0 and float(c) == 0.0
    assert c.pair == ("base_link", "crate")
    assert c.t == pytest.approx(1.62, abs=0.05)
    assert "contact" in repr(c)


def test_min_clearance_argument_errors(scene: bt.Scene) -> None:
    tl = _cycle(scene)
    with pytest.raises(ValueError, match="dt must be positive"):
        tl.min_clearance(dt=0.0)

    empty = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    sq = empty.sequence("idle")
    sq.step("wait", transition=bt.seq.elapsed(0.2))
    with pytest.raises(ValueError, match="nothing to measure"):
        sq.simulate().min_clearance()

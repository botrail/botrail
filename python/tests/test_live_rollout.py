"""Live rollouts (design-rl.md R0): the batch bake opened tick by tick from
Python, an external joint drive with a rate limit, and the between-tick
readers a controller or a policy closes its loop on."""

from pathlib import Path

import botrail as bt
import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
TABLE_TOP = 0.72
PART_HALF = 0.015


def settle_scene() -> bt.Scene:
    scene = bt.Scene()
    scene.add_box("table", size=(1.2, 0.8, TABLE_TOP), position=(0, 0, TABLE_TOP / 2))
    scene.add_box("part", size=(0.1, 0.05, 2 * PART_HALF), position=(0.1, 0.05, 1.2))
    scene.set_physics("part", dynamic=True, mass=0.2)
    sq = scene.sequence("settle")
    sq.step("wait", transition=bt.seq.elapsed(1.5))
    return scene


def arm_scene() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.add_box("floor", size=(3.0, 3.0, 0.1), position=(0, 0, -0.06))
    sq = scene.sequence("hold")
    sq.step("wait", transition=bt.seq.elapsed(5.0))
    return scene


def test_ticking_reproduces_the_batch_bake() -> None:
    scene = settle_scene()
    batch = scene.simulate_sequence("settle", physics=True)
    live = scene.open_rollout("settle", physics=True)
    assert live.t == 0.0 and live.dt == 0.01 and not live.finished
    ticks = 0
    while not live.finished:
        live.tick()
        ticks += 1
    assert ticks == 150
    tl = live.finish(publish=False)
    assert tl.duration == batch.duration
    assert tl.physics == "rapier"
    for k in range(16):
        t = batch.duration * k / 15
        assert tl.object_pose("part", t) == batch.object_pose("part", t)
    with pytest.raises(ValueError, match="finished"):
        live.tick()


def test_readers_between_ticks() -> None:
    live = settle_scene().open_rollout(["settle"], physics=True)
    assert live.steps() == [("settle", "wait")]
    z0 = live.object_pose("part")[0][2]
    assert live.tick(20) == pytest.approx(0.2)
    assert live.object_pose("part")[0][2] < z0 - 0.1
    (vx, vy, vz), _ = live.object_velocity("part")
    assert vz < -1.0
    with pytest.raises(ValueError, match="not a physics-dynamic"):
        live.object_velocity("table")
    assert live.contacts() == []
    live.tick(130)
    assert live.finished
    assert live.steps() == [("settle", None)]
    ((a, b, force),) = live.contacts()
    assert {a, b} == {"table", "part"}
    assert force >= 0.0
    assert live.object_pose("part")[0][2] == pytest.approx(TABLE_TOP + PART_HALF, abs=3e-3)
    with pytest.raises(ValueError, match="unknown obstacle"):
        live.object_pose("nothing")


def test_signals_read_as_levels() -> None:
    scene = settle_scene()
    scene.define_signal("go", False)
    sq = scene.sequence("flag")
    sq.step("raise", actions=[bt.seq.set_signal("go")], transition=bt.seq.elapsed(0.1))
    sq.step("lower", actions=[bt.seq.set_signal("go", False)], transition=bt.seq.elapsed(0.1))
    live = scene.open_rollout("flag")
    assert live.signal("go") is True
    live.tick(15)
    assert live.signal("go") is False
    with pytest.raises(ValueError, match="unknown signal"):
        live.signal("nothing")


def test_external_drive_is_rate_limited_and_bakes_per_tick() -> None:
    live = arm_scene().open_rollout("hold")
    live.drive()
    live.command([1.0, 0.0, -1.0, 0.0, 0.0, 0.0])
    prev = live.joint_positions()
    for _ in range(100):
        live.tick()
        q = live.joint_positions()
        # simple_arm limits: shoulder_pan 2.0 rad/s, elbow 2.5 rad/s.
        assert abs(q[0] - prev[0]) <= 2.0 * live.dt + 1e-12
        assert abs(q[2] - prev[2]) <= 2.5 * live.dt + 1e-12
        v = live.joint_velocities()
        assert v[0] == pytest.approx((q[0] - prev[0]) / live.dt)
        prev = q
    assert q[0] == pytest.approx(1.0, abs=1e-9)
    assert q[2] == pytest.approx(-1.0, abs=1e-9)
    tl = live.finish(publish=False)
    # Baked tick by tick: half-way at 0.25 s, at the target from 0.5 s on.
    assert tl.sample(0.25)[0] == pytest.approx(0.5, abs=1e-6)
    assert tl.sample(tl.duration)[0] == pytest.approx(1.0, abs=1e-6)


def test_max_velocity_caps_the_drive_and_limits_clamp_the_target() -> None:
    live = arm_scene().open_rollout("hold")
    live.drive(max_velocity=0.5)
    live.command([1.0, 10.0, 0.0, 0.0, 0.0, 0.0])
    live.tick(10)
    assert live.joint_positions()[0] == pytest.approx(0.05, abs=1e-9)
    live.tick(1000)
    assert live.joint_positions()[1] == pytest.approx(2.2, abs=1e-9)  # shoulder_lift upper
    with pytest.raises(ValueError, match="joints"):
        live.command([0.0, 0.0])


def test_drive_conflicts_and_undrive() -> None:
    scene = arm_scene()
    names = scene.robot.joint_names
    sq = scene.sequence("ramp")
    sq.step("wait", transition=bt.seq.elapsed(0.1))
    sq.step("up", actions=[bt.seq.ramp({names[1]: 0.6}, 1.0)], transition=bt.seq.done())
    live = scene.open_rollout("ramp")
    live.drive()
    with pytest.raises(ValueError, match="driven externally"):
        live.tick(20)
    live = scene.open_rollout("ramp")
    live.tick(20)
    with pytest.raises(ValueError, match="driven by `ramp`"):
        live.drive()
    live.tick(200)
    assert live.finished
    live.drive()
    live.undrive()
    with pytest.raises(ValueError, match="not driven"):
        live.command([0.0] * 6)
    with pytest.raises(ValueError, match="unknown group"):
        live.drive(group="left")


def test_driven_arm_reports_collisions_without_failing() -> None:
    live = arm_scene().open_rollout("hold")
    live.drive()
    live.tick()
    assert live.collisions() == []
    live.command([0.0, 1.5, 1.5, 0.0, 0.0, 0.0])
    hit = False
    for _ in range(200):
        live.tick()
        if any("floor" in pair for pair in live.collisions()):
            hit = True
            break
    assert hit
    live.undrive()
    assert live.collisions() == []


def test_tcp_and_link_poses_follow_the_drive(tmp_path) -> None:
    scene = arm_scene()
    live = scene.open_rollout("hold")
    before = live.tcp_pose()[0]
    live.drive()
    # Tilt the shoulder: a pan alone spins the upright arm about its own
    # axis and leaves the TCP where it is.
    live.command([0.0, 0.8, 0.0, 0.0, 0.0, 0.0])
    live.tick(60)
    after = live.tcp_pose()[0]
    assert after != before
    assert live.link_pose(scene.robot.tcp_link)[0] == after
    with pytest.raises(ValueError, match="unknown link"):
        live.link_pose("nothing")
    tl = live.finish(publish=False)
    out = tmp_path / "live.usda"
    tl.export_usd(out, fps=30.0)
    assert out.exists()

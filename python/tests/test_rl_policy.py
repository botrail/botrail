"""Policy steps (design-rl.md R3): a learned or scripted controller runs
as one step of a cell's sequence — `bt.seq.policy` + `rl.Policy` — and the
bake around it (conveyor, sensor, planned motions, physics) is the batch
bake's: deterministic, a timeline, a project that refuses to run without
its controller."""

from pathlib import Path

import botrail as bt
import numpy as np
import pytest
from botrail import rl

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
READY = [0.0, 0.6, 0.8, 0.0, 0.5, 0.0]


def build() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.set_joint_positions(READY)
    scene.add_box("floor", size=(3.0, 3.0, 0.1), position=(0, 0, -0.06))
    scene.add_box("table", size=(0.6, 0.6, 0.1), position=(0.55, 0.0, 0.05))
    scene.add_box("part", size=(0.06, 0.06, 0.06), position=(0.5, 0.1, 0.135))
    scene.set_physics("part", dynamic=True, mass=0.2)
    scene.add_box("goal", size=(0.02, 0.02, 0.02), position=(0.45, 0.1, 0.45))
    scene.set_obstacle_enabled("goal", False)
    names = scene.robot.joint_names
    sq = scene.sequence("cycle")
    sq.step("settle", transition=bt.seq.elapsed(0.2))
    sq.step("reach", actions=[bt.seq.policy("reach", hz=20, max_duration=4.0)], transition=bt.seq.done())
    sq.step("home", actions=[bt.seq.ramp(dict(zip(names, READY)), 0.5)], transition=bt.seq.done())
    return scene


def reach_task() -> rl.Task:
    return rl.Task(
        control=rl.TcpDelta(max_step_m=0.02, hz=20, frame="tcp"),
        observe=[rl.Joints(), rl.TcpPose(), rl.Relative("tcp", "goal"), rl.ObjectPose("part")],
        done=rl.rewards.within("tcp→goal", 0.02),
    )


def scripted(obs: np.ndarray) -> np.ndarray:
    # Channel layout: joints 12, tcp 7, tcp→goal 7 (translation at 19..22).
    rel = obs[19:22]
    return np.clip(rel / 0.02, -1.0, 1.0)


def test_a_policy_step_runs_inside_the_cycle(tmp_path) -> None:
    scene = build()
    policy = rl.Policy(scene, reach_task(), scripted)
    assert '"kind": "relative"' in policy.channels
    tl = scene.simulate_sequence("cycle", physics=True, policies={"reach": policy})
    reach = tl.step_span("reach")
    assert 0.1 < reach.duration < 2.0, reach
    home = tl.step_span("home")
    assert home.start == pytest.approx(reach.end)
    assert home.duration == pytest.approx(0.5, abs=0.011)
    # The arm came within 2 cm of the goal when the policy said done.
    (x, y, z), _ = scene.link_pose_at(scene.robot.tcp_link, tl.sample(reach.end))
    assert ((x - 0.45) ** 2 + (y - 0.1) ** 2 + (z - 0.45) ** 2) ** 0.5 < 0.03
    # The physics ran throughout: the part is tracked and rests on the table.
    (px, py, pz), _ = tl.object_pose("part", tl.duration)
    assert pz == pytest.approx(0.13, abs=5e-3)
    # The robot lane names the policy stretch.
    assert any(name == "policy reach" for name, *_ in tl.moves())
    assert tl.export_usd(tmp_path / "policy.usda", fps=30.0) is not None


def test_a_policy_bake_is_deterministic() -> None:
    scene = build()
    a = scene.simulate_sequence("cycle", physics=True, policies={"reach": rl.Policy(scene, reach_task(), scripted)})
    b = scene.simulate_sequence("cycle", physics=True, policies={"reach": rl.Policy(scene, reach_task(), scripted)})
    assert a.duration == b.duration
    for k in range(11):
        t = a.duration * k / 10
        assert a.sample(t) == b.sample(t)
        assert a.object_pose("part", t) == b.object_pose("part", t)


def test_a_missing_or_broken_policy_fails_the_bake() -> None:
    scene = build()
    with pytest.raises(ValueError, match="policy `reach` is not registered"):
        scene.simulate_sequence("cycle", physics=True)
    with pytest.raises(ValueError, match="not callable"):
        scene.simulate_sequence("cycle", policies={"reach": 42})
    with pytest.raises(ValueError, match="channels"):
        scene.simulate_sequence("cycle", policies={"reach": lambda inp: None})

    def broken(inp):
        raise RuntimeError("no model loaded")

    broken.channels = "[]"
    with pytest.raises(ValueError, match="no model loaded"):
        scene.simulate_sequence("cycle", policies={"reach": broken})


def test_a_collision_under_a_policy_fails_the_bake() -> None:
    scene = build()
    task = rl.Task(control=rl.JointTarget(hz=20), observe=[rl.Joints()])
    # Folds the arm into the floor.
    fold = rl.Policy(scene, task, lambda obs: np.array([0.0, 0.5, 0.5, 0.0, 0.0, 0.0]))
    with pytest.raises(ValueError, match="policy `reach` drove .* into"):
        scene.simulate_sequence("cycle", policies={"reach": fold})


def test_the_deadline_ends_a_policy_and_the_lane_records_it() -> None:
    scene = build()
    names = scene.robot.joint_names
    sq = scene.sequence("cycle")
    sq.step("reach", actions=[bt.seq.policy("reach", hz=10, max_duration=0.3)], transition=bt.seq.done())
    sq.step("home", actions=[bt.seq.ramp(dict(zip(names, READY)), 0.5)], transition=bt.seq.done())
    task = rl.Task(control=rl.JointDelta(max_step=0.05, hz=10), observe=[rl.Joints()])
    never = rl.Policy(scene, task, lambda obs: np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    tl = scene.simulate_sequence("cycle", policies={"reach": never})
    assert tl.step_span("reach").duration == pytest.approx(0.3, abs=0.011)
    assert tl.sample(tl.duration) == pytest.approx(READY, abs=1e-6)


def test_the_policy_step_survives_a_project_round_trip(tmp_path) -> None:
    scene = build()
    path = tmp_path / "cell.json"
    scene.save_project(path)
    again = bt.Scene.load_project(path)
    with pytest.raises(ValueError, match="not registered"):
        again.simulate_sequence("cycle", physics=True)
    tl = again.simulate_sequence("cycle", physics=True, policies={"reach": rl.Policy(again, reach_task(), scripted)})
    assert tl.step_span("reach").duration > 0.1
    code = again.generate_python()
    assert 'bt.seq.policy("reach"' in code, code


def test_rl_load_wraps_a_callable_and_rejects_unknown_files(tmp_path) -> None:
    scene = build()
    policy = rl.load(scripted, scene, reach_task())
    assert isinstance(policy, rl.Policy)
    with pytest.raises(ValueError, match="unknown policy file"):
        rl.load(tmp_path / "policy.pkl", scene, reach_task())

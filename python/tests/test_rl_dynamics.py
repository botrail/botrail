"""Dynamic robots and torque control (design-rl-dynamics.md RD1/RD2)."""

from pathlib import Path

import botrail as bt
import numpy as np
import pytest
from botrail import rl

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
READY = [0.0, 0.6, 0.8, 0.0, 0.5, 0.0]


def build(dynamic: bool = True) -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.set_joint_positions(READY)
    scene.add_box("floor", size=(3.0, 3.0, 0.1), position=(0, 0, -0.06))
    scene.add_box("goal", size=(0.02, 0.02, 0.02), position=(0.45, 0.1, 0.5))
    scene.set_obstacle_enabled("goal", False)
    if dynamic:
        scene.set_robot_physics()
    sq = scene.sequence("run")
    sq.step("wait", transition=bt.seq.elapsed(30.0))
    hold = scene.sequence("hold")
    hold.step("wait", transition=bt.seq.elapsed(1.0))
    return scene


def randomize(scene: bt.Scene, rng: np.random.Generator) -> None:
    scene.set_obstacle_pose("goal", (0.40 + rng.uniform(-0.05, 0.05), rng.uniform(-0.15, 0.15), 0.5))


def torque_task(**overrides) -> rl.Task:
    fields = dict(
        control=rl.Torque(hz=20),
        observe=[rl.Joints(), rl.Relative("tcp", "goal")],
        reset=randomize,
        reward=rl.rewards.reach("tcp→goal"),
        horizon_s=1.0,
    )
    fields.update(overrides)
    return rl.Task(**fields)


def test_a_dynamic_robot_bakes_the_physical_joints() -> None:
    scene = build()
    assert scene.robot_physics()
    tl = scene.simulate_sequence("hold", physics=True)
    # The robot lane is the engine's read-back, scan by scan: every
    # sample of a one-second hold sits within a few milliradians of READY
    # (the servo droops under gravity a little).
    samples = np.array([tl.sample(t) for t in np.arange(0.0, 1.0, 0.01)])
    worst = np.abs(samples - READY).max()
    assert worst < 0.01, worst
    # Not exactly READY: the physics, not the plan, wrote the lane.
    assert np.abs(samples[-1] - READY).max() > 1e-6
    # Undeclared, the same bake is the kinematic one.
    scene.set_robot_physics(dynamic=False)
    assert not scene.robot_physics()
    plain = scene.simulate_sequence("hold", physics=True)
    assert all(plain.sample(t) == READY for t in (0.0, 0.5, 1.0))


def test_torque_env_runs_and_matches_the_vector_env() -> None:
    env = rl.make(build, torque_task(), seed=0, flatten=False)
    assert env.action_space.shape == (6,)
    obs, info = env.reset(seed=3)
    assert env.live.is_dynamic()
    joints = obs["simple_arm/joints"]
    assert np.allclose(joints[:6], READY, atol=1e-6) and np.allclose(joints[6:], 0.0)
    rng = np.random.default_rng(0)
    actions = rng.uniform(-1.0, 1.0, size=(5, 6))
    singles = []
    for a in actions:
        obs, reward, terminated, truncated, info = env.step(a)
        assert info["ik_failed"] is False
        assert "error" not in info
        singles.append(obs["simple_arm/joints"].copy())
    # Torques moved the joints and gave them velocity.
    assert not np.allclose(singles[-1][:6], READY, atol=1e-3)
    assert np.abs(singles[-1][6:]).max() > 0.05
    # The vector environment steps the same physics, bit for bit.
    venv = rl.make(build, torque_task(), seed=0, num_envs=2, flatten=False)
    vobs, _ = venv.reset(seed=3)
    assert np.array_equal(vobs["simple_arm/joints"][0], np.asarray(joints_of(env, seed=3), dtype=np.float32))
    for k, a in enumerate(actions):
        vobs, *_ = venv.step(np.stack([a, -a]))
        assert np.array_equal(vobs["simple_arm/joints"][0], singles[k]), f"step {k}"


def joints_of(env, seed: int):
    obs, _ = env.reset(seed=seed)
    return obs["simple_arm/joints"]


def test_torque_control_validates() -> None:
    with pytest.raises(ValueError, match="declared dynamic"):
        rl.make(lambda: build(dynamic=False), torque_task(), seed=0)
    with pytest.raises(ValueError, match="values for 6 joints"):
        rl.make(build, torque_task(control=rl.Torque(max_torque=[1.0, 2.0])), seed=0)
    env = rl.make(build, torque_task(control=rl.Torque(max_torque=5.0)), seed=0)
    env.reset()
    with pytest.raises(ValueError, match="out of range"):
        env.live.command_torque([(7, 1.0)])
    # A kinematic rollout refuses torques outright.
    plain = build(dynamic=False)
    live = plain.open_rollout(["run"], physics=True)
    live.drive()
    assert not live.is_dynamic()
    with pytest.raises(ValueError, match="not dynamic"):
        live.command_torque([(0, 1.0)])
    with pytest.raises(ValueError, match="no effort limit|positive"):
        plain.set_robot_physics(max_force=-1.0)


def test_position_controls_drive_a_dynamic_robot() -> None:
    task = torque_task(control=rl.JointDelta(max_step=0.05, hz=20))
    env = rl.make(build, task, seed=0, flatten=False)
    obs, _ = env.reset(seed=1)
    for _ in range(10):
        obs, reward, terminated, truncated, info = env.step(np.ones(6))
        assert "error" not in info
    q = obs["simple_arm/joints"][:6]
    # Every joint followed its steps as a physical servo: each action
    # asks for 0.05 rad from where the joint *is*, and the servo covers
    # most of it before the next — the joints moved a good part of the
    # 0.5 rad asked, none of them exactly.
    moved = q - np.array(READY)
    assert np.all(moved > 0.2) and np.all(moved < 0.6), moved
    tl = env.timeline(publish=False)
    assert tl.duration >= 0.49
    assert np.abs(np.array(tl.sample(tl.duration)) - q).max() < 1e-6


def test_the_declaration_round_trips_through_a_project(tmp_path) -> None:
    scene = build()
    scene.set_robot_physics(max_force=25.0, armature=0.05)
    path = tmp_path / "cell.botrail"
    scene.save_project(path)
    back = bt.Scene.load_project(path)
    assert back.robot_physics()
    text = path.read_text()
    assert '"max_force": 25.0' in text and '"armature": 0.05' in text


def test_gravity_torques_read_out_and_the_compensation_opt_in() -> None:
    task = torque_task(observe=[rl.Joints(), rl.GravityTorque()], reward=None)
    env = rl.make(build, task, seed=0, flatten=False)
    obs, _ = env.reset(seed=1)
    g = obs["simple_arm/gravity"]
    assert g.shape == (6,)
    assert abs(float(g[0])) < 1e-6 and abs(float(g[1])) > 1.0
    assert np.allclose(g, env.live.gravity_torques(), atol=1e-5)
    # Raw torques: a zero action lets the arm fall.
    for _ in range(20):
        obs, *_ = env.step(np.zeros(6))
    fell = np.abs(obs["simple_arm/joints"][:6] - READY).max()
    assert fell > 0.1, fell
    # Compensated: the same zero action holds it.
    comp = rl.make(build, torque_task(control=rl.Torque(hz=20, gravity_compensation=True), reward=None), seed=0, flatten=False)
    obs, _ = comp.reset(seed=1)
    for _ in range(20):
        obs, *_ = comp.step(np.zeros(6))
    held = np.abs(obs["simple_arm/joints"][:6] - READY).max()
    assert held < 0.02, held
    # The read-out and the flag need a dynamic robot.
    with pytest.raises(ValueError, match="declared dynamic"):
        rl.make(lambda: build(dynamic=False), torque_task(control=rl.JointDelta(), observe=[rl.GravityTorque()]), seed=0)
    live = build(dynamic=False).open_rollout(["run"], physics=True)
    with pytest.raises(ValueError, match="dynamic robot"):
        live.gravity_torques()


def test_a_dynamic_walker_keeps_its_legs_kinematic() -> None:
    import sys

    sys.path.insert(0, str(EXAMPLES / "legged"))
    import legged_patrol_demo as demo

    scene = demo.build_scene("quad")
    names = demo.build_cycle(scene)
    kinematic = scene.simulate_sequences(names, max_duration=90.0, physics=True)
    scene.set_robot_physics("dog")
    dynamic = scene.simulate_sequences(names, max_duration=90.0, physics=True)
    assert abs(dynamic.duration - kinematic.duration) < 1e-9
    assert dynamic.footfalls("dog") == kinematic.footfalls("dog")
    # The legs are the gait's in both bakes; the neck (the cycle ramps it
    # to look at the dock) is a servoed body that follows its command.
    neck = scene.robot_of("dog").joint_names.index("neck")
    for t in np.linspace(0.0, dynamic.duration, 60):
        a = np.array(kinematic.sample(t, robot="dog"))
        b = np.array(dynamic.sample(t, robot="dog"))
        legs = np.array([i != neck for i in range(len(a))])
        assert np.abs(a[legs] - b[legs]).max() < 1e-9, t
        assert abs(a[neck] - b[neck]) < 0.05, (t, a[neck], b[neck])

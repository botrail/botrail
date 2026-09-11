"""botrail.rl (design-rl.md R1): a Task over a cell becomes a gymnasium-style
environment on the live rollout — spaces from the spec, seeded episodes
that reproduce bit for bit, controls that respect the drive's limits, and
an episode that closes into an ordinary timeline."""

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
    scene.add_box("part", size=(0.06, 0.06, 0.06), position=(0.55, 0.0, 0.135))
    scene.set_physics("part", dynamic=True, mass=0.2)
    # The goal marker is a picture, not a thing: nothing collides with it.
    scene.add_box("goal", size=(0.02, 0.02, 0.02), position=(0.45, 0.1, 0.5))
    scene.set_obstacle_enabled("goal", False)
    sq = scene.sequence("run")
    sq.step("wait", transition=bt.seq.elapsed(30.0))
    return scene


def randomize(scene: bt.Scene, rng: np.random.Generator) -> None:
    scene.set_obstacle_pose("goal", (0.40 + rng.uniform(-0.05, 0.05), rng.uniform(-0.15, 0.15), 0.45 + rng.uniform(0.0, 0.1)))
    scene.set_obstacle_pose("part", (0.55 + rng.uniform(-0.03, 0.03), rng.uniform(-0.05, 0.05), 0.135))


def reach_task(**overrides) -> rl.Task:
    fields = dict(
        control=rl.TcpDelta(max_step_m=0.02, hz=20, frame="tcp"),
        observe=[
            rl.Joints(),
            rl.TcpPose(),
            rl.Relative("tcp", "goal"),
            rl.ObjectPose("part"),
            rl.Contacts("simple_arm/tool0", "part"),
            rl.Clearance(),
            rl.Collision(),
        ],
        reset=randomize,
        reward=rl.rewards.combine(rl.rewards.reach("tcp→goal"), rl.rewards.collision_penalty(1.0)),
        done=rl.rewards.within("tcp→goal", 0.02),
        horizon_s=3.0,
    )
    fields.update(overrides)
    return rl.Task(**fields)


def scripted(obs_dict, max_step: float) -> np.ndarray:
    """A P-controller in action space: step toward the goal. The goal is
    observed in the TCP frame and the control acts in it (`frame="tcp"`),
    so the step is the relative vector itself."""
    rel = obs_dict["tcp→goal"][:3]
    return np.clip(rel / max_step, -1.0, 1.0)


def run_episode(env: rl.Env, seed: int, steps: int = 40):
    obs, info = env.reset(seed=seed)
    trace = [obs]
    rewards = []
    for _ in range(steps):
        action = scripted(info["channels"], env.task.control.max_step_m)
        obs, reward, terminated, truncated, info = env.step(action)
        trace.append(obs)
        rewards.append(reward)
        if terminated or truncated:
            break
    return np.stack(trace), np.array(rewards), terminated, truncated, info


def test_spaces_follow_the_spec() -> None:
    env = rl.make(build, reach_task(), seed=0)
    assert env.obs_dim == 12 + 7 + 7 + 7 + 2 + 1 + 1
    assert env.observation_space.shape == (env.obs_dim,)
    assert env.action_space.shape == (3,)
    assert env.scans_per_step == 5
    assert [c.key for c in env.channels] == [
        "simple_arm/joints", "simple_arm/tcp", "tcp→goal", "part/pose",
        "contact:simple_arm/tool0×part", "clearance", "collision",
    ]
    obs, info = env.reset(seed=1)
    assert obs.dtype == np.float32 and obs.shape == (env.obs_dim,)
    assert env.observation_space.contains(obs)
    assert info["t"] == 0.0 and info["steps"] == 0 and not info["collision"]
    assert 0.0 < info["channels"]["clearance"][0] <= 1.0


def test_python_readers_agree_with_the_rust_packer() -> None:
    env = rl.make(build, reach_task(), seed=0)
    obs, info = env.reset(seed=2)
    for _ in range(3):
        obs, _, _, _, info = env.step(scripted(info["channels"], 0.02))
    for channel in env.channels:
        packed = info["channels"][channel.key]
        read = channel.read(env.live, env.task.robot)
        assert read.shape == packed.shape, channel
        assert np.allclose(read, packed, atol=1e-12), channel


def test_same_seed_reproduces_the_episode_bit_for_bit() -> None:
    env = rl.make(build, reach_task(), seed=0)
    a = run_episode(env, seed=3)
    b = run_episode(env, seed=3)
    assert np.array_equal(a[0], b[0])
    assert np.array_equal(a[1], b[1])
    assert a[2:4] == b[2:4]
    c = run_episode(env, seed=4)
    assert not np.array_equal(a[0][0], c[0][0])  # a different goal


def test_a_scripted_policy_reaches_the_goal_and_the_episode_is_a_timeline(tmp_path) -> None:
    env = rl.make(build, reach_task(), seed=0)
    trace, rewards, terminated, truncated, info = run_episode(env, seed=7, steps=60)
    assert terminated and not truncated, info
    assert rewards[-1] > rewards[0]  # closer at the end than at the start
    assert not info["ik_failed"]
    tl = env.timeline()
    assert tl.duration == pytest.approx(info["t"])
    assert tl.duration == pytest.approx(info["steps"] * env.scans_per_step * 0.01)
    assert float(tl.min_clearance()) > 0.0
    # The dynamic part rode the physics the whole time: it is tracked.
    (x, y, z), _ = tl.object_pose("part", tl.duration)
    assert z == pytest.approx(0.13, abs=5e-3)
    out = tmp_path / "episode.usda"
    tl.export_usd(out, fps=30.0)
    assert out.exists()
    with pytest.raises(RuntimeError, match="reset"):
        env.step(np.zeros(3))
    assert env.timeline() is tl


def test_the_horizon_truncates() -> None:
    env = rl.make(build, reach_task(horizon_s=1.0), seed=0)
    env.reset()
    for k in range(1, 21):
        obs, reward, terminated, truncated, info = env.step(np.zeros(3))
        if k < 20:
            assert not (terminated or truncated)
    assert truncated and not terminated
    assert info["t"] == pytest.approx(1.0)
    with pytest.raises(RuntimeError):
        env.step(np.zeros(3))


def test_joint_delta_and_joint_target_controls() -> None:
    env = rl.make(build, reach_task(control=rl.JointDelta(max_step=0.05, hz=20), observe=[rl.Joints()], reward=None, done=None), seed=0)
    assert env.action_space.shape == (6,)
    env.reset()
    q0 = env.live.joint_positions()
    env.step([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    q1 = env.live.joint_positions()
    assert q1[0] == pytest.approx(q0[0] + 0.05)
    assert q1[1:] == pytest.approx(q0[1:])

    env = rl.make(build, reach_task(control=rl.JointTarget(hz=20), observe=[rl.Joints()], horizon_s=5.0, reward=None, done=None), seed=0)
    env.reset()
    for _ in range(40):
        env.step(-np.ones(6))
    q = env.live.joint_positions()
    limits = env.scene.robot.joint_limits
    assert q[1] == pytest.approx(limits[1][0], abs=1e-9)  # shoulder_lift at its lower limit
    assert env.action_space.shape == (6,)


def test_tcp_delta_holds_the_orientation_and_reports_ik_failure() -> None:
    # World-frame steps: +x is the world's x whatever the tool points at.
    env = rl.make(build, reach_task(control=rl.TcpDelta(max_step_m=0.02, hz=20), observe=[rl.TcpPose()], reward=None, done=None), seed=0)
    obs, info = env.reset()
    (x0, _, _), q0 = env.live.tcp_pose()
    # Two steps stay inside the reach the held orientation allows (the
    # third runs into the workspace boundary and would flag ik_failed).
    for _ in range(2):
        obs, reward, terminated, truncated, info = env.step([1.0, 0.0, 0.0])
        assert not info["ik_failed"]
    # Let a rate-limited drive settle on the setpoint, then the
    # orientation is the one the episode started with.
    for _ in range(5):
        env.step([0.0, 0.0, 0.0])
    (x1, _, _), q1 = env.live.tcp_pose()
    assert x1 > x0 + 0.03
    assert abs(np.dot(q0, q1)) > 1.0 - 1e-6
    # A step no configuration can reach holds the target and says so.
    env = rl.make(build, reach_task(control=rl.TcpDelta(max_step_m=3.0, hz=20), observe=[rl.TcpPose()], reward=None, done=None), seed=0)
    env.reset()
    _, _, _, _, info = env.step([0.0, 0.0, 1.0])
    assert info["ik_failed"]


def test_tcp_delta_gripper_dimension_drives_a_joint() -> None:
    task = reach_task(control=rl.TcpDelta(max_step_m=0.0, gripper=["wrist_3"], hz=20), observe=[rl.Joints()], reward=None, done=None)
    env = rl.make(build, task, seed=0)
    assert env.action_space.shape == (4,)
    env.reset()
    for _ in range(30):
        env.step([0.0, 0.0, 0.0, 1.0])
    assert env.live.joint_positions()[5] == pytest.approx(3.1416, abs=1e-6)


def test_dict_observations_and_signal_channel() -> None:
    def build_with_signal():
        scene = build()
        scene.define_signal("go", True)
        return scene

    task = reach_task(observe=[rl.Signal("go"), rl.Joints(velocities=False)], reward=None, done=None)
    env = rl.make(build_with_signal, task, seed=0, flatten=False)
    obs, info = env.reset()
    assert set(obs) == {"signal:go", "simple_arm/joints"}
    assert obs["signal:go"].tolist() == [1.0]
    assert obs["simple_arm/joints"].shape == (6,)


def test_a_bake_error_terminates_the_episode() -> None:
    # The rollout's own timeout is the fault here: max_duration below the
    # horizon runs the bake off its clock.
    env = rl.make(build, reach_task(horizon_s=2.0), seed=0, max_duration=0.5)
    env.reset()
    outcome = None
    for _ in range(60):
        _, _, terminated, truncated, info = env.step(np.zeros(3))
        if terminated or truncated:
            outcome = (terminated, truncated, info)
            break
    assert outcome is not None
    terminated, truncated, info = outcome
    assert terminated and "timed out" in info["error"]


def test_task_validation() -> None:
    with pytest.raises(ValueError, match="unknown robot"):
        rl.make(build, reach_task(robot="nope"))
    with pytest.raises(ValueError, match="whole number"):
        rl.make(build, reach_task(control=rl.TcpDelta(hz=30)))
    with pytest.raises(ValueError, match="num_envs"):
        rl.make(build, reach_task(), num_envs=0)

    def no_program():
        scene = build()
        scene.remove_sequence("run")
        return scene

    env = rl.make(no_program, reach_task())
    with pytest.raises(ValueError, match="no sequence"):
        env.reset()


def test_gymnasium_check_env_passes() -> None:
    gym = pytest.importorskip("gymnasium")
    from gymnasium.utils.env_checker import check_env

    env = rl.make(build, reach_task(), seed=0)
    assert isinstance(env, gym.Env)
    check_env(env, skip_render_check=True)

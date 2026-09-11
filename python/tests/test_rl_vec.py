"""botrail.rl.VecEnv (design-rl.md R2): N worlds of one cell stepped on
Rust threads, observations packed in Rust — every world bit-identical to
the single environment under the same seed, with gymnasium's NEXT_STEP
autoreset."""

import numpy as np
import pytest
from botrail import rl
from test_rl import build, reach_task, scripted


def test_vector_spaces_and_first_observation() -> None:
    venv = rl.make(build, reach_task(), seed=0, num_envs=3)
    assert isinstance(venv, rl.VecEnv)
    assert venv.num_envs == 3
    assert venv.single_observation_space.shape == (37,)
    assert venv.single_action_space.shape == (3,)
    assert venv.observation_space.shape == (3, 37)
    assert venv.action_space.shape == (3, 3)
    obs, info = venv.reset(seed=5)
    assert obs.shape == (3, 37) and obs.dtype == np.float32
    assert info["reset"].tolist() == [True, True, True]
    assert len(info["envs"]) == 3
    assert info["envs"][0]["channels"]["tcp→goal"].shape == (7,)
    # Three different goals: the seeds differ per world.
    goals = [tuple(e["channels"]["tcp→goal"][:3]) for e in info["envs"]]
    assert len(set(goals)) == 3


def test_each_world_matches_the_single_env_bit_for_bit() -> None:
    venv = rl.make(build, reach_task(), seed=0, num_envs=3)
    vobs, vinfo = venv.reset(seed=10)
    singles = [rl.make(build, reach_task(), seed=0) for _ in range(3)]
    sobs = [env.reset(seed=10 + i) for i, env in enumerate(singles)]
    for i in range(3):
        assert np.array_equal(vobs[i], sobs[i][0])
    infos = [s[1] for s in sobs]
    for step in range(12):
        actions = np.stack([scripted(vinfo["envs"][i]["channels"], 0.02) for i in range(3)])
        vobs, vrew, vterm, vtrunc, vinfo = venv.step(actions)
        for i, env in enumerate(singles):
            action = scripted(infos[i]["channels"], 0.02)
            obs, rew, term, trunc, infos[i] = env.step(action)
            assert np.array_equal(vobs[i], obs), f"world {i} step {step}"
            assert vrew[i] == rew and vterm[i] == term and vtrunc[i] == trunc
            assert vinfo["envs"][i]["t"] == infos[i]["t"]
            if term or trunc:
                # Single envs stay done; the vector world resets next step.
                infos[i] = env.reset(seed=None)[1]
        if vterm.any():
            break
    assert vterm.any() or vtrunc.any()


def test_next_step_autoreset_and_timelines(tmp_path) -> None:
    venv = rl.make(build, reach_task(horizon_s=0.5), seed=1, num_envs=2)
    obs, info = venv.reset()
    zeros = np.zeros((2, 3))
    for _ in range(9):
        obs, rewards, terminated, truncated, info = venv.step(zeros)
        assert not truncated.any()
    obs, rewards, terminated, truncated, info = venv.step(zeros)
    assert truncated.tolist() == [True, True]
    assert info["t"].tolist() == [0.5, 0.5]
    # The next step resets both: first observations, no reward, no flags.
    obs, rewards, terminated, truncated, info = venv.step(np.ones((2, 3)))
    assert info["reset"].tolist() == [True, True]
    assert rewards.tolist() == [0.0, 0.0]
    assert not terminated.any() and not truncated.any()
    assert info["t"].tolist() == [0.0, 0.0]
    assert info["steps"].tolist() == [0, 0]
    obs, rewards, terminated, truncated, info = venv.step(zeros)
    assert info["t"].tolist() == pytest.approx([0.05, 0.05])
    # A world's episode closes into a timeline; it resets at the next step.
    tl = venv.timeline(0)
    assert tl.duration == pytest.approx(0.05)
    assert float(tl.min_clearance()) > 0.0
    tl.export_usd(tmp_path / "world0.usda", fps=30.0)
    obs, rewards, terminated, truncated, info = venv.step(zeros)
    assert info["reset"].tolist() == [True, False]
    assert info["t"].tolist() == pytest.approx([0.0, 0.10])


def test_batched_joint_controls_and_ik_failures() -> None:
    task = reach_task(control=rl.JointDelta(max_step=0.05, hz=20), observe=[rl.Joints()], reward=None, done=None)
    venv = rl.make(build, task, seed=0, num_envs=2)
    venv.reset()
    q0 = venv.joints()
    actions = np.array([[1.0, 0, 0, 0, 0, 0], [-1.0, 0, 0, 0, 0, 0]])
    venv.step(actions)
    q1 = venv.joints()
    assert q1[0, 0] == pytest.approx(q0[0, 0] + 0.05)
    assert q1[1, 0] == pytest.approx(q0[1, 0] - 0.05)

    task = reach_task(control=rl.JointTarget(hz=20), observe=[rl.Joints()], reward=None, done=None, horizon_s=5.0)
    venv = rl.make(build, task, seed=0, num_envs=2)
    venv.reset()
    for _ in range(40):
        venv.step(-np.ones((2, 6)))
    lower = venv.scenes[0].robot.joint_limits[1][0]
    assert venv.joints()[:, 1] == pytest.approx([lower, lower], abs=1e-9)

    task = reach_task(control=rl.TcpDelta(max_step_m=3.0, hz=20), observe=[rl.TcpPose()], reward=None, done=None)
    venv = rl.make(build, task, seed=0, num_envs=2)
    venv.reset()
    _, _, _, _, info = venv.step(np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 0.0]]))
    assert info["ik_failed"].tolist() == [True, False]
    assert info["envs"][0]["ik_failed"] and not info["envs"][1]["ik_failed"]


def test_a_dead_world_terminates_and_comes_back() -> None:
    venv = rl.make(build, reach_task(horizon_s=2.0), seed=0, num_envs=2, max_duration=0.5)
    venv.reset()
    zeros = np.zeros((2, 3))
    for _ in range(12):
        obs, rewards, terminated, truncated, info = venv.step(zeros)
        if terminated.any():
            break
    assert terminated.tolist() == [True, True]
    assert "timed out" in info["envs"][0]["error"]
    obs, rewards, terminated, truncated, info = venv.step(zeros)
    assert info["reset"].tolist() == [True, True]
    assert not terminated.any()


def test_vector_env_checker() -> None:
    gym = pytest.importorskip("gymnasium")
    venv = rl.make(build, reach_task(), seed=0, num_envs=2)
    assert isinstance(venv, gym.vector.VectorEnv)
    assert venv.metadata["autoreset_mode"] == gym.vector.AutoresetMode.NEXT_STEP
    obs, info = venv.reset(seed=0)
    assert venv.observation_space.contains(obs)
    obs, rewards, terminated, truncated, info = venv.step(venv.action_space.sample())
    assert venv.observation_space.contains(obs)
    assert rewards.shape == (2,) and terminated.shape == (2,) and truncated.shape == (2,)

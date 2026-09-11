"""Sensor observations for RL (design-rl-sensors.md RS0): a LiDAR channel
swept on the live world in Rust, thinned and seeded, observations coming
back as ndarrays, dict observation spaces, and the single and vector
environments agreeing bit for bit."""

import math

import botrail as bt
import numpy as np
import pytest
from botrail import rl
from test_rl import build as build_reach
from test_rl import randomize, reach_task


def build() -> bt.Scene:
    scene = build_reach()
    scene.add_lidar("front", position=(0.0, -0.6, 0.3), fov=270.0, range=(0.05, 8.0), resolution=0.5)
    scene.add_lidar("rings", position=(0.0, -0.6, 0.3), fov=360.0, range=(0.05, 8.0), resolution=2.0, channels=4, vfov=10.0)
    return scene


def lidar_task(**overrides) -> rl.Task:
    fields = dict(
        control=rl.TcpDelta(max_step_m=0.02, hz=20, frame="tcp"),
        observe=[rl.Joints(), rl.Relative("tcp", "goal"), rl.Lidar("front"), rl.Lidar("rings", stride=3, rings=[0, 3], noise=0.02)],
        reset=randomize,
        reward=rl.rewards.reach("tcp→goal"),
        done=rl.rewards.within("tcp→goal", 0.02),
        horizon_s=2.0,
    )
    fields.update(overrides)
    return rl.Task(**fields)


def test_lidar_channels_size_from_the_scene_and_read_as_ndarrays() -> None:
    env = rl.make(build, lidar_task(), seed=0)
    dims = {c.key: c.dim for c in env.channels}
    assert dims["lidar:front"] == 541
    assert dims["lidar:rings"] == 60 * 2  # 180 azimuths / 3, rings 0 and 3
    assert env.obs_dim == 12 + 7 + 541 + 120
    obs, info = env.reset(seed=1)
    assert isinstance(obs, np.ndarray) and obs.dtype == np.float32
    front = info["channels"]["lidar:front"]
    assert front.shape == (541,)
    assert front.max() == pytest.approx(8.0)  # misses read the max range
    assert front.min() < 2.0  # the table and the arm are in view
    # Python's reader agrees with the packer, bit for bit (no noise).
    assert np.array_equal(env.live.lidar("front"), front)
    # A noisy channel: the seed pins the draw, another seed differs.
    a = env.live.lidar("rings", stride=3, rings=[0, 3], noise=0.02)
    b = env.live.lidar("rings", stride=3, rings=[0, 3], noise=0.02)
    assert np.array_equal(a, b)
    env.live.set_noise_seed(99)
    c = env.live.lidar("rings", stride=3, rings=[0, 3], noise=0.02)
    assert not np.array_equal(a, c)
    with pytest.raises(ValueError, match="unknown lidar"):
        env.live.lidar("nothing")


def test_lidar_channel_validation() -> None:
    with pytest.raises(ValueError, match="unknown lidar"):
        rl.make(build, lidar_task(observe=[rl.Lidar("nothing")], reward=None, done=None))
    with pytest.raises(ValueError, match="ring"):
        rl.make(build, lidar_task(observe=[rl.Lidar("rings", rings=[4])], reward=None, done=None))


def test_vector_and_single_envs_agree_with_lidar_channels() -> None:
    venv = rl.make(build, lidar_task(), seed=0, num_envs=3)
    vobs, vinfo = venv.reset(seed=10)
    assert isinstance(vobs, np.ndarray) and vobs.shape == (3, venv.obs_dim)
    singles = [rl.make(build, lidar_task(), seed=0) for _ in range(3)]
    sobs = [env.reset(seed=10 + i) for i, env in enumerate(singles)]
    for i in range(3):
        assert np.array_equal(vobs[i], sobs[i][0]), f"world {i} at reset"
    infos = [s[1] for s in sobs]
    for step in range(6):
        actions = np.stack([np.clip(vinfo["envs"][i]["channels"]["tcp→goal"][:3] / 0.02, -1, 1) for i in range(3)])
        vobs, vrew, vterm, vtrunc, vinfo = venv.step(actions)
        for i, env in enumerate(singles):
            action = np.clip(infos[i]["channels"]["tcp→goal"][:3] / 0.02, -1, 1)
            obs, rew, term, trunc, infos[i] = env.step(action)
            assert np.array_equal(vobs[i], obs), f"world {i} step {step}"
            assert vrew[i] == rew
        if vterm.any() or vtrunc.any():
            break


def test_dict_observations_in_both_environments() -> None:
    env = rl.make(build, lidar_task(), seed=0, flatten=False)
    obs, info = env.reset()
    assert set(obs) == {"simple_arm/joints", "tcp→goal", "lidar:front", "lidar:rings"}
    assert obs["lidar:front"].shape == (541,) and obs["lidar:front"].dtype == np.float32
    venv = rl.make(build, lidar_task(), seed=0, num_envs=2, flatten=False)
    vobs, vinfo = venv.reset()
    assert set(vobs) == set(obs)
    assert vobs["lidar:rings"].shape == (2, 120)
    vobs, rewards, terminated, truncated, vinfo = venv.step(np.zeros((2, 3)))
    assert vobs["simple_arm/joints"].shape == (2, 12)
    gym = pytest.importorskip("gymnasium")
    assert isinstance(env.observation_space, gym.spaces.Dict)
    assert env.observation_space.contains(obs)
    assert isinstance(venv.single_observation_space, gym.spaces.Dict)
    assert venv.observation_space.contains(vobs)


def test_ndarray_transfers_keep_the_state_only_path_identical() -> None:
    # The R2 tests already pin single == vector; here the returned types.
    venv = rl.make(build_reach, reach_task(), seed=0, num_envs=2)
    obs, info = venv.reset(seed=3)
    assert isinstance(obs, np.ndarray) and obs.shape == (2, 37)
    assert isinstance(venv.joints(), np.ndarray) and venv.joints().shape == (2, 6)
    assert isinstance(venv.tcps(), np.ndarray) and venv.tcps().shape == (2, 7)
    obs, rewards, terminated, truncated, info = venv.step(np.zeros((2, 3)))
    assert isinstance(obs, np.ndarray)
    assert isinstance(info["envs"][0]["channels"]["simple_arm/tcp"], np.ndarray)


# ---------------------------------------------------------------- RS1: pictures


def _quat_from_basis(x, y, z) -> tuple[float, float, float, float]:
    """The xyzw quaternion of the rotation whose columns are the camera
    axes `x`, `y`, `z` expressed in the world."""
    m = np.array([x, y, z], dtype=float).T
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    q[3] = (m[k, j] - m[j, k]) / s
    return tuple(q)


def box_cell() -> bt.Scene:
    """A cube 2 m in front of a world camera looking down +x (-Z view,
    +Y up: the -Z axis rotated onto +x), plus the reach arm off to the
    side with a wrist camera."""
    scene = build_reach()
    scene.add_box("cube", size=(1.0, 1.0, 1.0), position=(4.5, 3.0, 0.5))
    scene.add_box("wall", size=(0.2, 6.0, 6.0), position=(30.0, 3.0, 0.0))
    # An upright camera looking down +x: camera -Z → world +x, camera +Y
    # (image up) → world +z, so camera +X → world -y. As a quaternion
    # (xyzw) that basis is (-0.5, 0.5, 0.5, 0.5)... derived below from
    # the rotation matrix so the test states the frame, not a number.
    scene.add_camera("fixed", position=(2.0, 3.0, 0.5), quaternion=_quat_from_basis(x=(0, -1, 0), y=(0, 0, 1), z=(-1, 0, 0)), fov=60.0, resolution=(64, 48), near=0.1, far=10.0)
    scene.add_camera("wrist", position=(0.0, 0.0, 0.05), quaternion=(1.0, 0.0, 0.0, 0.0), fov=70.0, resolution=(48, 48), near=0.05, far=3.0, robot="simple_arm", link="tool0")
    return scene


def picture_task(**overrides) -> rl.Task:
    fields = dict(
        control=rl.TcpDelta(max_step_m=0.02, hz=20, frame="tcp"),
        observe=[
            rl.Joints(),
            rl.Relative("tcp", "goal"),
            rl.Depth("fixed", size=(64, 48)),
            rl.Segmentation("fixed", size=(64, 48)),
            rl.PointCloud("fixed", size=(16, 12)),
            rl.Depth("wrist", size=(24, 24), noise=0.002, name="wrist/noisy"),
        ],
        reset=randomize,
        reward=rl.rewards.reach("tcp→goal"),
        done=rl.rewards.within("tcp→goal", 0.02),
        horizon_s=1.0,
    )
    fields.update(overrides)
    return rl.Task(**fields)


def test_picture_channels_have_shapes_and_read_the_cube() -> None:
    env = rl.make(box_cell, picture_task(), seed=0, flatten=False)
    dims = {c.key: (c.dim, c.shape) for c in env.channels}
    assert dims["fixed/depth"] == (64 * 48, (48, 64))
    assert dims["fixed/segmentation"] == (64 * 48, (48, 64))
    assert dims["fixed/point_cloud"] == (3 * 16 * 12, (16 * 12, 3))
    assert dims["wrist/noisy"] == (24 * 24, (24, 24))
    obs, info = env.reset(seed=1)
    depth = obs["fixed/depth"]
    assert depth.shape == (48, 64) and depth.dtype == np.float32
    # The cube's near face is 2 m ahead of the camera; the wall lies
    # beyond `far`, so the edges of the picture read 0.
    assert depth[24, 32] == pytest.approx(2.0, abs=1e-3)
    assert depth[24, 0] == 0.0 and depth[0, 32] == 0.0
    ids = obs["fixed/segmentation"]
    names = rl.segmentation_ids(env.scene)
    assert int(ids[24, 32]) == names["cube"]
    assert int(ids[24, 0]) == 0
    pts = obs["fixed/point_cloud"]
    assert pts.shape == (16 * 12, 3)
    centre = pts[6 * 16 + 8]
    assert centre[2] == pytest.approx(-2.0, abs=1e-3)
    # Python's readers agree with the packer (noiseless channels).
    for channel in env.channels:
        if channel.key == "wrist/noisy":
            continue
        packed = info["channels"][channel.key]
        read = channel.read(env.live, env.task.robot).reshape(packed.shape)
        assert np.allclose(read, packed, atol=1e-6), channel.key
    # The wrist picture sees the table below the tool, with noise on
    # valid pixels only; the seed pins the draw.
    noisy = obs["wrist/noisy"]
    assert (noisy > 0).any()
    clean, _ = env.live.render("wrist", 24, 24)
    valid = clean > 0
    assert np.array_equal(noisy == 0, ~valid)
    assert not np.allclose(noisy[valid], clean[valid])
    assert np.abs(noisy[valid] - clean[valid]).max() < 0.02
    obs2, _ = env.reset(seed=1)
    assert np.array_equal(obs2["wrist/noisy"], noisy)


def test_picture_channels_validate() -> None:
    with pytest.raises(ValueError, match="unknown camera"):
        rl.make(box_cell, picture_task(observe=[rl.Depth("nothing")], reward=None, done=None))
    with pytest.raises(ValueError, match="geometry"):
        rl.Depth("fixed", geometry="wire")
    env = rl.make(box_cell, picture_task(observe=[rl.Depth("fixed", size=(8, 8), geometry="collision")], reward=None, done=None))
    obs, info = env.reset()
    assert obs.shape == (64,)
    with pytest.raises(ValueError, match="unknown camera"):
        env.live.render("nothing", 8, 8)
    with pytest.raises(ValueError, match="geometry"):
        env.live.render("fixed", 8, 8, geometry="wire")


def test_vector_and_single_envs_agree_with_pictures() -> None:
    task = picture_task(observe=[rl.Joints(), rl.Relative("tcp", "goal"), rl.Depth("wrist", size=(16, 16)), rl.Segmentation("wrist", size=(16, 16))])
    venv = rl.make(box_cell, task, seed=0, num_envs=2, flatten=False)
    vobs, vinfo = venv.reset(seed=4)
    assert vobs["wrist/depth"].shape == (2, 16, 16)
    singles = [rl.make(box_cell, picture_task(observe=list(task.observe)), seed=0, flatten=False) for _ in range(2)]
    sobs = [env.reset(seed=4 + i) for i, env in enumerate(singles)]
    for i in range(2):
        for key in vobs:
            assert np.array_equal(vobs[key][i], sobs[i][0][key]), f"{key} world {i}"
    actions = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    vobs, *_ = venv.step(actions)
    for i, env in enumerate(singles):
        obs, *_ = env.step(actions[i])
        for key in vobs:
            assert np.array_equal(vobs[key][i], obs[key]), f"{key} world {i} after a step"
    gym = pytest.importorskip("gymnasium")
    assert isinstance(venv.single_observation_space, gym.spaces.Dict)
    assert venv.single_observation_space["wrist/depth"].shape == (16, 16)
    assert venv.observation_space.contains(vobs)


def test_wrist_camera_rides_the_link() -> None:
    env = rl.make(box_cell, picture_task(observe=[rl.Joints(), rl.Depth("wrist", size=(8, 8))], reward=None, done=None), seed=0, flatten=False)
    env.reset()
    before = env.live.camera_pose("wrist")
    env.live.command([1.0, 0.6, 0.8, 0.0, 0.5, 0.0])
    env.live.tick(60)
    after = env.live.camera_pose("wrist")
    assert after[0] != before[0]
    with pytest.raises(ValueError, match="unknown camera"):
        env.live.camera_pose("nothing")


def test_the_rasteriser_agrees_with_the_studio_depth_capture(tmp_path) -> None:
    pytest.importorskip("playwright.sync_api")
    from playwright.sync_api import sync_playwright

    try:
        with sync_playwright() as p:
            p.chromium.launch().close()
    except Exception as e:
        pytest.skip(f"chromium unavailable: {e}")
    from botrail import capture

    scene = box_cell()
    frame = capture.capture_depth(scene, "fixed")
    studio = np.asarray(frame.depth, dtype=np.float32)
    env = rl.make(box_cell, picture_task(observe=[rl.Depth("fixed", size=(studio.shape[1], studio.shape[0]))], reward=None, done=None), seed=0, flatten=False)
    obs, _ = env.reset()
    ours = obs["fixed/depth"]
    assert ours.shape == studio.shape
    both = (ours > 0) & (studio > 0)
    assert both.sum() > 0.2 * both.size
    assert np.abs(ours[both] - studio[both]).max() < 1e-3
    # Above the horizon the pictures agree on what is background too, bar
    # edge pixels; below it the studio draws its own floor plane, which
    # is a picture, not scene geometry (studio-floor-hides-below-zero).
    upper = slice(0, studio.shape[0] // 2 - 2)
    assert ((ours[upper] > 0) != (studio[upper] > 0)).mean() < 0.02


# ---------------------------------------------------------------- RS2: colour


def colour_task(**overrides) -> rl.Task:
    fields = dict(
        control=rl.TcpDelta(max_step_m=0.02, hz=20, frame="tcp"),
        observe=[rl.Joints(), rl.Relative("tcp", "goal"), rl.Depth("fixed", size=(32, 24)), rl.Rgb("fixed", size=(32, 24))],
        reset=randomize,
        reward=rl.rewards.reach("tcp→goal"),
        done=rl.rewards.within("tcp→goal", 0.02),
        horizon_s=1.0,
    )
    fields.update(overrides)
    return rl.Task(**fields)


def test_rgb_channel_paints_colour_under_the_light() -> None:
    def build():
        scene = box_cell()
        scene.set_obstacle_color("cube", (1.0, 0.2, 0.2))
        return scene

    task = colour_task(render={"light": (-1.0, 0.0, 0.0), "ambient": 0.2})
    env = rl.make(build, task, seed=0, flatten=False)
    obs, info = env.reset(seed=1)
    rgb = obs["fixed/rgb"]
    assert rgb.shape == (24, 32, 3) and rgb.dtype == np.float32
    # Lit head-on, the red face reads its albedo; the background is black.
    assert rgb[12, 16].tolist() == [255.0, 51.0, 51.0]
    assert rgb[0, 0].tolist() == [0.0, 0.0, 0.0]
    # Python's reader agrees with the packer.
    read = env.channels[3].read(env.live, env.task.robot).reshape(24, 32, 3)
    assert np.array_equal(read, rgb)
    # Lit from the side: only the ambient share remains.
    env.live.set_lighting((0.0, 1.0, 0.0), 0.2)
    assert env.live.render_rgb("fixed", 32, 24)[12, 16].tolist() == [51, 10, 10]
    with pytest.raises(ValueError, match="unknown camera"):
        env.live.render_rgb("nothing", 8, 8)


def test_colour_and_light_randomisation_change_rgb_but_not_depth_or_state() -> None:
    def randomize_colour(scene, rng):
        randomize(scene, rng)
        scene.set_obstacle_color("cube", tuple(rng.uniform(0.2, 1.0, size=3)))

    def lighting(rng):
        d = rng.normal(size=3)
        return {"light": tuple(d), "ambient": float(rng.uniform(0.2, 0.5))}

    task = colour_task(reset=randomize_colour, render=lighting)
    env = rl.make(box_cell, task, seed=0, flatten=False)
    a, _ = env.reset(seed=3)
    b, _ = env.reset(seed=4)
    c, _ = env.reset(seed=3)
    # Same seed, same picture — colours, light and all; another seed
    # repaints the cube and relights it, while depth and joints stay.
    for key in a:
        assert np.array_equal(a[key], c[key]), key
    assert not np.array_equal(a["fixed/rgb"], b["fixed/rgb"])
    assert np.array_equal(a["fixed/depth"], b["fixed/depth"])
    assert np.array_equal(a["simple_arm/joints"], b["simple_arm/joints"])
    # The cube is coloured, the background black, in both.
    for obs in (a, b):
        assert obs["fixed/rgb"][12, 16].max() > 40
        assert obs["fixed/rgb"][0, 0].max() == 0


def test_camera_pose_randomisation_moves_the_picture() -> None:
    def randomize_pose(scene, rng):
        randomize(scene, rng)
        scene.set_camera_pose("fixed", (2.0 + rng.uniform(-0.3, 0.3), 3.0, 0.5))

    env = rl.make(box_cell, colour_task(reset=randomize_pose), seed=0, flatten=False)
    a, _ = env.reset(seed=1)
    b, _ = env.reset(seed=2)
    assert a["fixed/depth"][12, 16] != b["fixed/depth"][12, 16]
    assert abs(float(a["fixed/depth"][12, 16]) - 2.0) < 0.31
    with pytest.raises(ValueError, match="unknown camera"):
        env.scene.set_camera_pose("nothing", (0.0, 0.0, 0.0))


def test_vector_env_lights_each_world_from_its_own_seed() -> None:
    def lighting(rng):
        return {"light": tuple(rng.normal(size=3)), "ambient": 0.3}

    task = colour_task(render=lighting)
    venv = rl.make(box_cell, task, seed=0, num_envs=2, flatten=False)
    vobs, _ = venv.reset(seed=8)
    singles = [rl.make(box_cell, colour_task(render=lighting), seed=0, flatten=False) for _ in range(2)]
    for i, env in enumerate(singles):
        obs, _ = env.reset(seed=8 + i)
        assert np.array_equal(vobs["fixed/rgb"][i], obs["fixed/rgb"]), f"world {i}"
    assert not np.array_equal(vobs["fixed/rgb"][0], vobs["fixed/rgb"][1])


# ---------------------------------------------------------------- RS3: sensor models, shadows, decimation


def test_lidar_points_channel_is_the_sweep_in_the_scanner_frame() -> None:
    task = lidar_task(observe=[rl.Joints(), rl.Relative("tcp", "goal"), rl.Lidar("front"), rl.Lidar("front", points=True, name="front/points")])
    env = rl.make(build, task, seed=0, flatten=False)
    dims = {c.key: (c.dim, c.shape) for c in env.channels}
    assert dims["lidar:front"] == (541, None)
    assert dims["front/points"] == (3 * 541, (541, 3))
    obs, info = env.reset(seed=1)
    ranges = obs["lidar:front"]
    pts = obs["front/points"]
    assert pts.shape == (541, 3)
    hit = ranges < 8.0
    assert hit.any() and (~hit).any()
    # A hit beam's point lies at its range; a miss reads zeros where the
    # range channel reads the scanner's maximum.
    assert np.allclose(np.linalg.norm(pts[hit], axis=1), ranges[hit], atol=1e-4)
    assert not pts[~hit].any()
    # A planar scanner: every point in its own z = 0 plane.
    assert np.abs(pts[:, 2]).max() < 1e-6
    direct = env.live.lidar("front", points=True)
    assert direct.shape == (541, 3) and np.allclose(direct, pts, atol=1e-5)
    read = env.channels[3].read(env.live, env.task.robot).reshape(541, 3)
    assert np.allclose(read, pts, atol=1e-5)
    venv = rl.make(build, task, seed=0, num_envs=2, flatten=False)
    vobs, _ = venv.reset(seed=1)
    assert np.array_equal(vobs["front/points"][0], pts)


def test_depth_sensor_model_quantises_drops_and_widens_with_range() -> None:
    def depth(**kw):
        task = picture_task(observe=[rl.Joints(), rl.Depth("fixed", size=(64, 48), **kw)], reward=None, done=None)
        env = rl.make(box_cell, task, seed=0, flatten=False)
        return env, env.reset(seed=1)[0]["fixed/depth"]

    _, clean = depth()
    assert clean[24, 32] == pytest.approx(2.0, abs=1e-3)
    valid = clean > 0
    # Disparity quantisation: a 0.1 m baseline at fx ≈ 55.4 px reads the
    # 2 m face at fx·B / round(fx·B / 2) — the third disparity step.
    fx = 32.0 / math.tan(math.radians(30.0))
    _, stereo = depth(baseline=0.1)
    assert stereo[24, 32] == pytest.approx(fx * 0.1 / round(fx * 0.1 / 2.0), abs=1e-4)
    assert np.array_equal(stereo == 0, ~valid)
    # Range-dependent noise, σ = noise_z2·z², on the valid pixels only,
    # pinned by the seed.
    env_n, noisy = depth(noise_z2=0.002)
    assert np.array_equal(noisy == 0, ~valid)
    assert not np.allclose(noisy[valid], clean[valid])
    assert np.abs(noisy[valid] - clean[valid]).max() < 6 * 0.002 * 4.0
    assert np.array_equal(env_n.reset(seed=1)[0]["fixed/depth"], noisy)
    assert not np.array_equal(env_n.reset(seed=2)[0]["fixed/depth"], noisy)
    # Dropout: about that share of the valid pixels reads no return and
    # the rest are untouched; 1.0 blanks the picture.
    _, holed = depth(dropout=0.3)
    lost = valid & (holed == 0)
    assert 0.15 < lost.sum() / valid.sum() < 0.45
    kept = valid & (holed > 0)
    assert np.array_equal(holed[kept], clean[kept])
    _, blank = depth(dropout=1.0)
    assert not blank.any()
    for bad in (dict(noise_z2=-1.0), dict(dropout=1.5), dict(baseline=0.0)):
        with pytest.raises(ValueError):
            depth(**bad)


def _plane_obj(path, n: int = 20) -> None:
    """A unit plane meshed n × n (2·n² triangles) as an OBJ file."""
    lines = [f"v {i / n} {j / n} 0" for j in range(n + 1) for i in range(n + 1)]
    for j in range(n):
        for i in range(n):
            a = j * (n + 1) + i + 1
            b, c, d = a + 1, a + n + 1, a + n + 2
            lines += [f"f {a} {b} {d}", f"f {a} {d} {c}"]
    path.write_text("\n".join(lines) + "\n")


def test_shadows_and_decimation_come_from_task_render(tmp_path) -> None:
    obj = tmp_path / "plane.obj"
    _plane_obj(obj)

    def build():
        scene = box_cell()
        scene.set_obstacle_color("cube", (1.0, 0.2, 0.2))
        # An awning above and in front of the cube, out of the camera's
        # 23° vertical half-field: lit from up-front it shades the top
        # strip z ∈ [0.3, 0.5] of the face the camera sees.
        scene.add_box("awning", size=(0.2, 2.0, 0.05), position=(3.5, 3.0, 1.4))
        scene.add_mesh("plane", obj, position=(4.0, 1.5, 0.0))
        return scene

    def env_with(render, **kw):
        return rl.make(build, colour_task(render=render), seed=0, flatten=False, **kw)

    light = {"light": (-1.0, 0.0, 1.0), "ambient": 0.2}
    plain = env_with(light)
    a, _ = plain.reset(seed=1)
    shaded = env_with({**light, "shadows": True})
    b, _ = shaded.reset(seed=1)
    # Lambert cos 45° on the face: 0.2 + 0.8·0.707 of the albedo. Row 6
    # of the 32 × 24 picture looks at z ≈ 0.4 on the face: under the
    # awning, which itself is never in the picture.
    assert a["fixed/rgb"][12, 16].tolist() == [195.0, 39.0, 39.0]
    assert a["fixed/rgb"][6, 16].tolist() == [195.0, 39.0, 39.0]
    assert b["fixed/rgb"][12, 16].tolist() == [195.0, 39.0, 39.0]
    assert b["fixed/rgb"][6, 16].tolist() == [51.0, 10.0, 10.0]
    assert np.array_equal(a["fixed/depth"], b["fixed/depth"])
    venv = env_with({**light, "shadows": True}, num_envs=2)
    vobs, _ = venv.reset(seed=1)
    assert np.array_equal(vobs["fixed/rgb"][0], b["fixed/rgb"])
    # Decimation cuts the plane's 800 triangles (the boxes are a dozen
    # each and stay); the pictures' triangles are rebuilt per episode.
    full = plain.live.render_triangles()
    coarse_env = env_with({"decimate": 0.5})
    coarse_env.reset(seed=1)
    coarse = coarse_env.live.render_triangles()
    assert 760 <= full - coarse < 800, (full, coarse)
    assert coarse_env.live.render_triangles("collision") is None
    for bad in ({"foo": 1}, {"decimate": -1.0}):
        with pytest.raises(ValueError):
            env_with(bad).reset()

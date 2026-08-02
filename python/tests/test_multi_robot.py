"""Multi-robot scenes (R1): add_robot, robot= addressing, project round-trip."""

import math
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def robot() -> bt.Robot:
    return bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf")


@pytest.fixture()
def duo(robot: bt.Robot) -> bt.Scene:
    scene = bt.Scene(robot)
    scene.add_robot(robot, name="arm_b", base_position=(1.5, 0.0, 0.0))
    return scene


def test_add_robot_names_and_listing(robot: bt.Robot) -> None:
    scene = bt.Scene(robot)
    assert scene.robots == ["simple_arm"]
    # Same model again: the default name is uniquified.
    assert scene.add_robot(robot) == "simple_arm_2"
    assert scene.add_robot(robot, name="arm_c") == "arm_c"
    assert scene.robots == ["simple_arm", "simple_arm_2", "arm_c"]
    assert scene.robot_of("arm_c").dof == 6
    with pytest.raises(ValueError, match="unknown robot"):
        scene.robot_of("nope")


def test_scene_constructor_names_the_first_robot(robot: bt.Robot) -> None:
    scene = bt.Scene(robot, name="left")
    assert scene.robots == ["left"]


def test_robot_kwarg_addresses_instances(duo: bt.Scene) -> None:
    q = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]
    duo.set_joint_positions(q, robot="arm_b")
    assert duo.joint_positions_of("arm_b") == pytest.approx(q)
    # The first robot is untouched.
    assert duo.joint_positions_of("simple_arm") == pytest.approx([0.0] * 6)

    duo.set_robot_base_pose((2.0, 1.0, 0.0), robot="arm_b")
    position, _ = duo.robot_base_pose_of("arm_b")
    assert position == pytest.approx((2.0, 1.0, 0.0))
    # link_pose resolves within the addressed robot despite identical names.
    (x, y, _z), _ = duo.link_pose("base_link", robot="arm_b")
    assert (x, y) == pytest.approx((2.0, 1.0))


def test_omitted_robot_is_ambiguous_with_two_robots(duo: bt.Scene) -> None:
    with pytest.raises(ValueError, match="pass robot="):
        duo.set_joint_positions([0.0] * 6)
    with pytest.raises(ValueError, match="pass robot="):
        duo.plan([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_omitted_robot_still_works_with_one_robot(robot: bt.Robot) -> None:
    scene = bt.Scene(robot)
    scene.set_joint_positions([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert scene.joint_positions == pytest.approx([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_plan_avoids_the_other_robot(robot: bt.Robot) -> None:
    scene = bt.Scene(robot)
    # Lean toward +x; the goal is the same lean panned to -x. The naive
    # sweep passes +y, where a second upright arm stands.
    lean = [0.0, 1.2, 0.0, 0.0, 0.0, 0.0]
    scene.set_joint_positions(lean)
    scene.add_robot(robot, name="blocker", base_position=(0.0, 0.55, 0.0))
    goal = [math.pi, 1.2, 0.0, 0.0, 0.0, 0.0]
    traj = scene.plan(goal, robot="simple_arm", broadcast=False)
    assert traj.sample(traj.duration) == pytest.approx(goal, abs=1e-6)
    # The planner had to detour: joint names stay the planning robot's.
    assert traj.joint_names[0] == "shoulder_pan"


def test_attach_to_second_robot(duo: bt.Scene) -> None:
    duo.add_box("box", size=(0.04, 0.04, 0.04), position=(1.5, 0.1, 0.9))
    duo.attach("box", link="tool0", robot="arm_b")
    assert ("box", "tool0") in duo.attachments
    # Rotating arm_b's base joint carries the box; the first arm does not.
    duo.set_joint_positions([math.pi / 2, 0.0, 0.0, 0.0, 0.0, 0.0], robot="arm_b")
    (x, y, _z), _ = duo.obstacle_pose("box")
    assert (x, y) == pytest.approx((1.4, 0.0), abs=1e-6)


def test_watch_robots_sensor(duo: bt.Scene) -> None:
    duo.add_zone_sensor(
        "zone_a",
        position=(0.0, 0.0, 0.5),
        size=(0.4, 0.4, 1.0),
        watch_robots=["simple_arm"],
    )
    assert "zone_a" in duo.sensor_names
    # Round-trips through the project/codegen path below.


def test_project_round_trip(tmp_path: Path, duo: bt.Scene) -> None:
    duo.set_joint_positions([0.2, 0.4, -0.3, 0.0, 0.1, 0.0], robot="arm_b")
    duo.add_box("crate", size=(0.1, 0.1, 0.1), position=(1.5, 0.2, 0.9))
    duo.attach("crate", link="tool0", robot="arm_b")
    duo.add_segment("b_home", goal=[0.0] * 6, robot="arm_b")
    duo.add_zone_sensor(
        "zone_a",
        position=(0.0, 0.0, 0.5),
        size=(0.4, 0.4, 1.0),
        watch_robots=["arm_b"],
    )

    path = tmp_path / "cell.botrail"
    duo.save_project(path)
    loaded = bt.Scene.load_project(path)
    assert loaded.robots == ["simple_arm", "arm_b"]
    position, _ = loaded.robot_base_pose_of("arm_b")
    assert position == pytest.approx((1.5, 0.0, 0.0))
    assert loaded.joint_positions_of("arm_b") == pytest.approx(
        [0.2, 0.4, -0.3, 0.0, 0.1, 0.0]
    )
    assert ("crate", "tool0") in loaded.attachments
    assert "b_home" in loaded.motion_names
    # The reloaded motion still plans on arm_b (its goal DOF checks against
    # the owner), and the grasped crate still rides arm_b's joints.
    loaded.set_joint_positions([math.pi / 2, 0.4, -0.3, 0.0, 0.1, 0.0], robot="arm_b")
    (bx, _by, _bz), _ = loaded.obstacle_pose("crate")
    assert bx < 1.5


def test_generated_python_rebuilds_two_robots(tmp_path: Path, duo: bt.Scene) -> None:
    duo.set_joint_positions([0.2, 0.0, 0.0, 0.0, 0.0, 0.0], robot="arm_b")
    code = duo.generate_python()
    assert 'scene.add_robot(robot_2, name="arm_b"' in code
    assert 'robot="arm_b"' in code
    # The script must actually run and rebuild the same cell.
    namespace: dict = {}
    exec(compile(code.replace("bt.studio(scene)", ""), "<generated>", "exec"), namespace)
    rebuilt = namespace["scene"]
    assert rebuilt.robots == duo.robots
    assert rebuilt.joint_positions_of("arm_b") == pytest.approx(
        [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    position, _ = rebuilt.robot_base_pose_of("arm_b")
    assert position == pytest.approx((1.5, 0.0, 0.0))


def test_allow_inter_robot_collision(robot: bt.Robot) -> None:
    """Two arms sharing a spot collide until the pair is excused."""
    scene = bt.Scene(robot)
    # Second base close enough that the two base links overlap.
    scene.add_robot(robot, name="arm_b", base_position=(0.05, 0.0, 0.0))

    def pairs() -> list:
        return [
            (a[1], b[1])
            for a, b in scene.check_collisions()
            if a[0] == "link" and b[0] == "link"
        ]

    assert ("base_link", "base_link") in pairs()
    scene.allow_inter_robot_collision("simple_arm", "base_link", "arm_b", "base_link")
    assert ("base_link", "base_link") not in pairs()
    # Only that pair is excused; the rest of the overlap still reports.
    assert pairs()

    # Order is immaterial: the pair is stored canonically.
    scene2 = bt.Scene(robot)
    scene2.add_robot(robot, name="arm_b", base_position=(0.05, 0.0, 0.0))
    scene2.allow_inter_robot_collision("arm_b", "base_link", "simple_arm", "base_link")
    assert ("base_link", "base_link") not in [
        (a[1], b[1])
        for a, b in scene2.check_collisions()
        if a[0] == "link" and b[0] == "link"
    ]


def test_allow_inter_robot_collision_rejects_bad_pairs(duo: bt.Scene) -> None:
    with pytest.raises(ValueError, match="unknown robot"):
        duo.allow_inter_robot_collision("nope", "base_link", "arm_b", "base_link")
    with pytest.raises(ValueError, match="no link"):
        duo.allow_inter_robot_collision("simple_arm", "nope", "arm_b", "base_link")
    # A robot's own links are the self-collision matrix's business.
    with pytest.raises(ValueError, match="both sides"):
        duo.allow_inter_robot_collision("arm_b", "base_link", "arm_b", "rod")


def test_rename_robot(duo: bt.Scene) -> None:
    assert duo.robots == ["simple_arm", "arm_b"]
    assert duo.rename_robot("arm_b", "far") == "far"
    assert duo.robots == ["simple_arm", "far"]
    # Addressing follows the new name, and the old one is gone.
    duo.set_joint_positions([0.3, 0.0, 0.0, 0.0, 0.0, 0.0], robot="far")
    assert duo.joint_positions_of("far")[0] == pytest.approx(0.3)
    with pytest.raises(ValueError, match="unknown robot"):
        duo.joint_positions_of("arm_b")
    with pytest.raises(ValueError, match="unknown robot"):
        duo.rename_robot("arm_b", "x")
    # A name already taken is uniquified rather than silently merged.
    assert duo.rename_robot("far", "simple_arm") == "simple_arm_2"


def test_rename_robot_carries_the_authored_cell(duo: bt.Scene) -> None:
    """Sequences and zone sensors address robots by name, so a rename after
    authoring has to move them too — otherwise the cell only breaks when it
    is simulated."""
    duo.add_zone_sensor(
        "zone", position=(1.5, 0.0, 0.3), size=(0.6, 0.6, 0.6), watch_robots=["arm_b"]
    )
    duo.add_segment("b_out", goal=[0.4, 0.0, 0.0, 0.0, 0.0, 0.0], robot="arm_b")
    sq = duo.sequence("cell")
    sq.step("go", actions=[bt.seq.motion("b_out")])
    sq.step("wait", transition=bt.seq.all_of(bt.seq.robot_done("arm_b")))
    sq.step("open", actions=[bt.seq.ramp({"shoulder_pan": 0.1}, 0.2, robot="arm_b")])

    duo.rename_robot("arm_b", "far")
    # Simulating is the real test: an orphaned reference fails here.
    timeline = duo.simulate_sequence("cell")
    assert timeline.robots == ["simple_arm", "far"]
    assert timeline.duration > 0.0


def test_recordings_name_their_robot_instances(duo: bt.Scene, tmp_path: Path) -> None:
    """`examples/play_record.py` picks which cell to rebuild by reading the
    instance names off a recording, which only works because the exporter
    puts each robot at `/World/<instance name>`. Pin that contract here: if
    the export layout changes, the replay script would quietly load the
    wrong cell instead of failing."""
    import sys

    sys.path.insert(0, str(EXAMPLES))
    import play_record

    duo.add_segment("go", goal=[0.2, 0.0, 0.0, 0.0, 0.0, 0.0], robot="arm_b")
    sq = duo.sequence("cell")
    sq.step("move", actions=[bt.seq.motion("go")])
    out = tmp_path / "duo.usda"
    duo.simulate_sequence("cell").export_usd(out, fps=30.0)

    assert play_record.robot_instances(out) == {"simple_arm", "arm_b"}
    # The static scenery prim is not a robot.
    assert "Env" not in play_record.robot_instances(out)

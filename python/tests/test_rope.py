import json
import math
from pathlib import Path
import pytest
import botrail as bt

ASSETS = Path(__file__).resolve().parents[2] / "examples/assets"

def scene_and_bake():
    scene = bt.Scene(bt.Robot.from_urdf(ASSETS / "simple_arm.urdf"))
    scene.define_signal("grip")
    sequence = scene.sequence("rope")
    sequence.step("settle", transition=bt.seq.elapsed(0.02))
    sequence.step("take", actions=[bt.seq.set_signal("grip")], transition=bt.seq.elapsed(0.02))
    sequence.step("release", actions=[bt.seq.set_signal("grip", False)], transition=bt.seq.elapsed(0.02))
    timeline = scene.simulate_sequence("rope")
    return scene, timeline

def test_explicit_python_pass_adds_playback_without_mutating_the_bake():
    scene, source = scene_and_bake()
    target = bt.rope.animate(scene, source, name="cable", points=[(0,0,0.3),(0.3,0,0.3)],
        grippers=[bt.rope.gripper("grip",robot=scene.robot.name,link=scene.robot.tcp_link)], spacing_m=0.01)
    assert source.ropes == [] and target.ropes == ["cable"]
    track = bt.rope.track(target,"cable")
    assert track["times"][0] == 0 and track["times"][-1] == source.duration
    assert len(track["points"][0]) == 31
    assert all(math.isfinite(v) for p in track["points"] for xyz in p for v in xyz)
    assert track["held"][-1] == []
    assert len([e for e in track["events"] if e["closed"]]) == 1
    assert all(e["source_time_s"] == e["applied_time_s"] for e in track["events"])
    assert track["coupling"] == "baked_one_way_no_source_reaction"
    with pytest.raises(ValueError, match="no rope track"):
        bt.rope.track(target,"missing")

def test_unknown_link_and_extra_declaration_fields_fail_closed():
    scene, source = scene_and_bake()
    with pytest.raises(ValueError, match="unknown link"):
        bt.rope.animate(scene, source, name="cable",points=[(0,0,0),(1,0,0)],
            grippers=[bt.rope.gripper("grip",robot=scene.robot.name,link="missing")])
    with pytest.raises(ValueError, match="unknown field"):
        scene._animate_rope_json(source,json.dumps(dict(name="c",points=[[0,0,0],[1,0,0]],invented=True)))

def test_an_anchored_end_rides_the_housing_a_robot_carries_and_exports_as_a_tube(tmp_path):
    scene = bt.Scene(bt.Robot.from_urdf(ASSETS / "simple_arm.urdf"))
    tcp, _ = scene.link_pose(scene.robot.tcp_link)
    # A connector housing at the tool, a cable crimped in it running off along +x.
    scene.add_box("plug", size=(0.05, 0.02, 0.02), position=tcp)
    scene.set_obstacle_enabled("plug", False)
    names = scene.robot.joint_names
    home = list(scene.joint_positions)
    lifted = list(home)
    lifted[1] -= 0.3
    sequence = scene.sequence("carry")
    sequence.step("take", actions=[bt.seq.attach("plug")], transition=bt.seq.elapsed(0.1))
    sequence.step("swing", actions=[bt.seq.ramp(dict(zip(names, lifted)), 1.0)])
    sequence.step("set down", actions=[bt.seq.detach("plug")], transition=bt.seq.elapsed(0.5))
    timeline = scene.simulate_sequence("carry")
    cable = [(tcp[0] + 0.02 * i, tcp[1], tcp[2]) for i in range(16)]
    target = bt.rope.animate(scene, timeline, name="hv", points=cable,
        anchors=[bt.rope.anchor("plug", location="Start", length_m=0.03)],
        spacing_m=0.01, radius_m=0.006, density_kg_m=0.4, bending_hz=200.0, friction=0.3,
        color=(0.9, 0.16, 0.015))
    track = bt.rope.track(target, "hv")
    assert track["color"] == pytest.approx([0.9, 0.16, 0.015])
    assert track["held"][-1] == [] and track["events"] == []
    # The crimped end went with the housing and stays where it was set down.
    first, last = track["points"][0][0], track["points"][-1][0]
    plug = target.object_pose("plug", timeline.duration)[0]
    assert math.dist(first, last) > 0.05
    assert math.dist(last, plug) < 0.005
    out = tmp_path / "carry.usda"
    target.export_usd(str(out))
    text = out.read_text()
    assert 'def Xform "Ropes"' in text and 'def Mesh "hv"' in text


def test_an_anchor_names_one_body():
    with pytest.raises(ValueError, match="an anchor names"):
        bt.rope.anchor()
    with pytest.raises(ValueError, match="an anchor names"):
        bt.rope.anchor("plug", robot="r", link="tool")
    with pytest.raises(ValueError, match="an anchor names"):
        bt.rope.anchor(robot="r")
    assert bt.rope.anchor(robot="r", link="tool", location="End", length_m=0.02) == dict(
        robot="r", link="tool", location="End", length_m=0.02, roll=False)
    scene, source = scene_and_bake()
    with pytest.raises(ValueError, match="color"):
        bt.rope.animate(scene, source, name="c", points=[(0, 0, 0), (1, 0, 0)], color=(1, 0))


def _sharpest_turn_deg(points):
    """The largest turn over two particles each side, in degrees."""
    worst = 0.0
    for j in range(2, len(points) - 2):
        a = [points[j][k] - points[j - 2][k] for k in range(3)]
        b = [points[j + 2][k] - points[j][k] for k in range(3)]
        la, lb = math.hypot(*a), math.hypot(*b)
        if la > 1e-12 and lb > 1e-12:
            c = sum(x * y for x, y in zip(a, b)) / (la * lb)
            worst = max(worst, math.degrees(math.acos(max(-1.0, min(1.0, c)))))
    return worst


def test_a_straight_rest_lets_a_laid_corner_round_out():
    scene, source = scene_and_bake()
    # An L laid in the air, both ends held, under a stiff bending spring.
    corner = [(0.0, 0.5, 0.6), (0.2, 0.5, 0.6), (0.2, 0.7, 0.6)]

    def sharpest(rest):
        out = bt.rope.animate(scene, source, name=f"l_{rest}", points=corner, spacing_m=0.01,
                              pins=[("Start", corner[0]), ("End", corner[-1])], bending_hz=200.0,
                              preroll_s=1.0, rest=rest)
        return _sharpest_turn_deg(bt.rope.track(out, f"l_{rest}")["points"][-1])

    # Laid with a corner, the default rest keeps it; a straight rest (a tube
    # off the reel) springs it round.
    assert sharpest("straight") < 0.5 * sharpest("start")
    with pytest.raises(ValueError, match="rest"):
        bt.rope.animate(scene, source, name="x", points=corner, rest="curved")


def test_a_rod_bows_where_a_rope_hangs():
    scene, source = scene_and_bake()
    # 0.3 m of tube laid as a V between two points 0.2 m apart, straight at
    # rest: a rope hangs in it, a rod bows less.
    v = [(0.0, 0.5, 0.6), (0.1, 0.5, 0.6 - 0.112), (0.2, 0.5, 0.6)]

    def settled(rod_hz):
        out = bt.rope.animate(scene, source, name=f"v_{rod_hz}", points=v, spacing_m=0.006, radius_m=0.003,
                              density_kg_m=0.02, step_s=1 / 480, pins=[("Start", v[0]), ("End", v[-1])],
                              bending_hz=120.0, preroll_s=1.0, rest="straight", rod_hz=rod_hz)
        return min(p[2] for p in bt.rope.track(out, f"v_{rod_hz}")["points"][-1])

    low_rope = settled(0.0)
    low_rod = settled(120.0)
    assert low_rope < 0.6 - 0.07, low_rope
    assert low_rod > low_rope + 0.02, (low_rope, low_rod)
    with pytest.raises(ValueError, match="rod_hz"):
        bt.rope.animate(scene, source, name="x", points=v, rod_hz=-1.0)

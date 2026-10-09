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
        robot="r", link="tool", location="End", length_m=0.02)
    scene, source = scene_and_bake()
    with pytest.raises(ValueError, match="color"):
        bt.rope.animate(scene, source, name="c", points=[(0, 0, 0), (1, 0, 0)], color=(1, 0))

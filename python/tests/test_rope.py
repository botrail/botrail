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

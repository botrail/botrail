import math
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"

# The arm reaching down to the cloth, and lifted: the tool point rises 6 cm
# and comes 7 cm back.
LOW = [0.0, 0.9, 0.6, 0.0, 0.0, 0.0]
HIGH = [0.0, 0.6, 0.9, 0.0, 0.0, 0.0]


@pytest.fixture()
def scene() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.set_joint_positions(LOW)
    scene.add_segment("lift", goal=HIGH)
    scene.define_signal("grip")
    sq = scene.sequence("fold")
    sq.step("settle", transition=bt.seq.elapsed(0.2))
    sq.step("close", actions=[bt.seq.set_signal("grip")], transition=bt.seq.elapsed(0.2))
    sq.step("lift", actions=[bt.seq.motion("lift")])
    sq.step("hold", transition=bt.seq.elapsed(0.3))
    sq.step("open", actions=[bt.seq.set_signal("grip", False)], transition=bt.seq.elapsed(0.8))
    return scene


def _tool(scene: bt.Scene, joints) -> tuple:
    before = list(scene.joint_positions)
    scene.set_joint_positions(joints)
    position, _ = scene.link_pose(scene.robot.tcp_link)
    scene.set_joint_positions(before)
    return tuple(position)


def _sheet(scene: bt.Scene):
    """A 30 x 20 cm sheet whose east edge lies under the tool point."""
    tool = _tool(scene, LOW)
    return bt.cloth.sheet(
        "sheet", (0.3, 0.2), position=(tool[0] - 0.15, tool[1], tool[2]), spacing=0.05, workers=1
    )


def test_a_cloth_in_the_cell_rides_every_bake(scene: bt.Scene, tmp_path: Path) -> None:
    low, high = _tool(scene, LOW), _tool(scene, HIGH)
    sheet = _sheet(scene)
    # The landmarks are where the cloth will lie — and frames to teach against.
    marks = bt.cloth.landmarks(scene, sheet)
    assert len(marks) == 9
    assert math.dist(marks["edge_e"][:2], low[:2]) < 1e-9
    assert 0.0 < marks["edge_e"][2] - low[2] < 0.002
    assert math.dist(scene.frames["sheet/edge_e"][0], marks["edge_e"]) < 1e-12

    bt.cloth.add(scene, sheet, grippers=[bt.cloth.gripper("grip", radius=0.06)])
    tl = scene.simulate_sequence("fold")
    assert tl.cloths == ["sheet"]
    track = tl.cloth("sheet")
    assert track.name == "sheet"
    assert track.failure is None and track.warnings == []
    assert track.times[0] == 0.0 and track.times[-1] == tl.duration
    assert len(track.positions(0.0)) == 35 and len(track.triangles) == 48
    edge = track.landmarks["edge_e"]

    # The gripper holds while the signal is on.
    steps = {name: (start, end) for name, start, end in tl.step_spans}
    lift_start, lift_end = steps["lift"]
    assert track.held(0.1) == []
    assert edge in track.held(lift_start)
    assert edge in track.held(lift_end)
    assert track.held(tl.duration) == []
    # The held edge rides the tool point up; released, it falls back onto
    # the support while the far edge never left it.
    lifted = track.position("edge_e", lift_end + 0.2)
    assert math.dist(lifted, high) < 2e-3, (lifted, high)
    assert lifted[2] - track.position("edge_e", 0.1)[2] > 0.05
    assert track.position("edge_e", tl.duration)[2] < low[2] + 0.01
    assert all(track.position("edge_w", t)[2] < low[2] + 0.005 for t in track.times)
    assert track.positions(lift_end)[edge] == track.position("edge_e", lift_end)
    with pytest.raises(ValueError, match="no landmark"):
        track.position("pocket", 0.0)

    # The recording carries the cloth: a mesh whose points are time-sampled
    # on the cloth's own clock.
    out = tmp_path / "fold.usda"
    assert tl.export_usd(out, fps=30.0) == []
    text = out.read_text()
    assert 'def Mesh "sheet"' in text and "point3f[] points.timeSamples" in text
    mesh = text[text.index('def Mesh "sheet"'):]
    samples = mesh[mesh.index("points.timeSamples"):mesh.index("float3[] extent")]
    assert samples.count(": [(") == len(track.times)
    with pytest.raises(ValueError, match="no cloth"):
        tl.cloth("towel")

    # Out of the cell, the next bake is the robot's alone; the same pass on
    # that cycle gives the same cloth.
    assert bt.cloth.remove(scene, "sheet")
    assert not bt.cloth.remove(scene, "sheet")
    bare = scene.simulate_sequence("fold")
    assert bare.cloths == []
    again = bt.cloth.animate(scene, bare, sheet, grippers=[bt.cloth.gripper("grip", radius=0.06)])
    assert again.cloths == ["sheet"] and bare.cloths == []
    assert again.cloth("sheet").positions(lift_end) == track.positions(lift_end)


def test_a_tshirt_is_lifted_by_its_cuff(scene: bt.Scene) -> None:
    low = _tool(scene, LOW)
    # Where the east cuff lies relative to the shirt's centre decides where
    # the shirt goes for that cuff to be under the tool point.
    flat = bt.cloth.tshirt("shirt", position=(0.0, 0.0, low[2]), spacing=0.05, workers=1)
    marks = bt.cloth.landmarks(scene, flat, frames=False)
    assert len(marks) == 12 and "shirt/cuff_left" not in scene.frames
    cuff = max(("cuff_left", "cuff_right"), key=lambda name: marks[name][0])
    shirt = bt.cloth.tshirt(
        "shirt",
        position=(low[0] - marks[cuff][0], low[1] - marks[cuff][1], low[2]),
        spacing=0.05,
        workers=1,
    )
    tl = bt.cloth.animate(
        scene,
        scene.simulate_sequence("fold"),
        shirt,
        grippers=[bt.cloth.gripper("grip", radius=0.06, compliance=0.02, soften=2)],
    )
    track = tl.cloth("shirt")
    assert track.failure is None and track.warnings == []
    steps = {name: (start, end) for name, start, end in tl.step_spans}
    # The pinch goes through both layers, and the cuff comes up with it.
    assert len(track.held(steps["lift"][0])) >= 4
    risen = track.position(cuff, steps["lift"][1])[2] - track.position(cuff, 0.0)[2]
    assert risen > 0.04, risen
    assert "botrail" in repr(type(track)) and "shirt" in repr(track)


def test_cloth_declarations_are_checked(scene: bt.Scene) -> None:
    sheet = _sheet(scene)
    grip = bt.cloth.gripper("grip")
    with pytest.raises(ValueError, match="unknown frame"):
        bt.cloth.add(scene, bt.cloth.sheet("s", (0.3, 0.2), on="table/top"), [grip])
    with pytest.raises(ValueError, match="needs `on=<frame>`"):
        bt.cloth.landmarks(scene, bt.cloth.sheet("s", (0.3, 0.2)))
    with pytest.raises(ValueError, match="unknown robot"):
        bt.cloth.add(scene, sheet, [bt.cloth.gripper("grip", robot="left")])
    with pytest.raises(ValueError, match="no link"):
        bt.cloth.add(scene, sheet, [bt.cloth.gripper("grip", link="finger")])
    with pytest.raises(ValueError, match="unknown layers"):
        bt.cloth.landmarks(scene, bt.cloth.tshirt("t", position=(0, 0, 0), layers="triple"))
    with pytest.raises(ValueError, match="unknown cloth kind"):
        bt.cloth.landmarks(scene, {**sheet, "kind": "sock"})
    # A signal is checked against the cycle: a gripper on one the cell
    # never declared fails the bake it was asked to ride.
    bt.cloth.add(scene, sheet, [bt.cloth.gripper("pinch")])
    with pytest.raises(ValueError, match="no signal `pinch`"):
        scene.simulate_sequence("fold")
    assert bt.cloth.remove(scene, "sheet")
    # A frame places the cloth: its origin, its height and its heading.
    scene.add_frame("bench/top", (0.5, 0.1, 0.3), (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)))
    on_bench = bt.cloth.sheet("s", (0.3, 0.2), on="bench/top", spacing=0.05)
    marks = bt.cloth.landmarks(scene, on_bench, frames=False)
    assert math.dist(marks["center"][:2], (0.5, 0.1)) < 1e-9 and 0.3 < marks["center"][2] < 0.302
    # A quarter turn: the east edge now points along +y.
    assert math.dist(marks["edge_e"][:2], (0.5, 0.25)) < 1e-9
    # A gripper that never closes on cloth says so.
    far = bt.cloth.sheet("far", (0.2, 0.2), position=(2.0, 2.0, 0.0), spacing=0.05, workers=1)
    tl = bt.cloth.animate(scene, scene.simulate_sequence("fold"), far, [grip])
    assert "closed on nothing" in tl.cloth("far").warnings[0]
    # A pad is a box the gripper pushes cloth with; its size has to be one.
    padded = bt.cloth.gripper("grip", pad=(0.04, 0.04, 0.04), pad_offset=(0.0, 0.0, 0.02))
    assert bt.cloth.animate(scene, tl, far, [padded]).cloth("far").failure is None
    with pytest.raises(ValueError, match="pad"):
        bt.cloth.animate(scene, tl, far, [bt.cloth.gripper("grip", pad=(0.04, 0.0, 0.04))])

"""The flagship pick cycle under world physics (design-world-physics.md W3):
the same programs on the physical cell, the same steps, the box on the
pallet — the acceptance the design set."""

import sys
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES))

from basics import demo, sequence_demo  # noqa: E402


@pytest.fixture(scope="module")
def cell():
    pytest.importorskip("huggingface_hub", reason="the demo's equipment needs the optional catalog extra")
    scene = demo.build_scene()
    name = sequence_demo.build_cycle(scene)
    return scene, name


def test_the_pick_cycle_runs_the_same_steps_under_world_physics(cell):
    scene, name = cell
    kinematic = scene.simulate_sequence(name)
    physical = scene.simulate_sequence(name, physics=bt.Physics(world=True))
    assert physical.physics == "rapier" and physical.physics_scope == "world"
    assert [s for s, _, _ in physical.step_spans] == [s for s, _, _ in kinematic.step_spans]
    # The physical cycle keeps its takt: the belt carries the box a shade
    # slower by friction than by advection, nothing else drifts.
    assert abs(physical.duration - kinematic.duration) < 0.5
    # The box ends on the pallet either way — within a couple of
    # centimetres of the taught place, resting.
    box = sequence_demo.BOX
    (kx, ky, kz), _ = kinematic.object_pose(box, kinematic.duration)
    (px, py, pz), _ = physical.object_pose(box, physical.duration)
    assert abs(px - kx) < 0.03 and abs(py - ky) < 0.03 and abs(pz - kz) < 0.02, ((px, py, pz), (kx, ky, kz))
    assert physical.settled_at(box) is not None
    # It was carried, not dropped: through the carry it stays with the hand.
    spans = {s: (a, b) for s, a, b in physical.step_spans}
    carry_start, carry_end = spans["carry"]
    for t in (carry_start + 0.5, (carry_start + carry_end) / 2, carry_end - 0.1):
        (bx, by, bz), _ = physical.object_pose(box, t)
        assert bz > 0.55, (t, bz)
    # The arm was powered by the program: it holds the hover before the
    # descend to within a centimetre of the taught pose.
    assert abs(spans["descend"][0] - spans["latch"][1]) < 1e-6

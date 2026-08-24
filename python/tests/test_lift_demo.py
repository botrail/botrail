"""The elevator cell (`examples/lift_demo.py`): the AMR — chassis, tote on
the deck, mounted arm — rides the lift whole; the door enforces itself as
an obstacle; the lift edge in the path is never driven.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES))

import lift_demo as demo  # noqa: E402


def test_the_amr_rides_the_lift_whole() -> None:
    scene, tl = demo.bake()
    # The arm's base rode from the lobby deck to the mezzanine.
    z0 = tl.base_pose(0.0)[0][2]
    z1 = tl.base_pose(tl.duration)[0][2]
    assert z0 == pytest.approx(0.32, abs=1e-6)
    assert z1 == pytest.approx(demo.TOP + 0.32, abs=1e-6)
    # The tote rode the deck all the way to the dock.
    p, _ = tl.object_pose("tote", tl.duration)
    assert p[0] == pytest.approx(demo.DOCK[0] - 0.1, abs=1e-6)
    assert p[2] == pytest.approx(demo.TOP + 0.37, abs=1e-6)
    # The interlock chain is on the chart: call, door, ride.
    lanes = dict(tl.signals)
    assert [v for _, v in lanes["call"]][:2] == [False, True]
    assert len(lanes["door"]) == 5  # off, open, off, close, off
    assert [v for _, v in lanes["lift"]] == [False, True, False]
    assert True in [v for _, v in lanes["car_occupied"]]
    assert tl.duration < 40.0
    # The whole cell regenerates, lift and lift call included.
    code = scene.generate_python()
    assert 'scene.add_lift("lift"' in code
    assert 'bt.seq.move_to("lift", "2F")' in code


def test_a_closed_door_blocks_boarding_by_name() -> None:
    with pytest.raises(ValueError, match="door/panel"):
        demo.bake(skip_door=True)


def test_a_goto_across_the_lift_edge_is_refused() -> None:
    scene = demo.build()
    seq = scene.sequence("jump")
    seq.step("go", actions=[bt.seq.goto("amr", "dock")],
             transition=bt.seq.device_done("amr"))
    with pytest.raises(ValueError, match="across the lift edge"):
        seq.simulate()

"""The EV module-connection cell, asserted the way its owner would.

`examples/assembly/ev_battery_harness_demo.py` is the shipped demonstration
of ropes in a cell: two arms take flexible HV module connectors from their
blister by the two housings and plug both ends at once; each cable is a
`bt.rope` whose ends are anchored in its housings. This pins what makes the
cell *right*: the cables ride their housings from the blister to the
headers and stay there, no cable is pulled taut or droops onto what lies in
the channel, the cables never cross, and the recording carries them.

The arms come from the catalog, so the cell is baked only where the catalog
is cached. It is the demo's own bake: about a minute, once for the module.
"""

import math
import os
import sys
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "assembly"))

import _ev_battery_pack as P  # noqa: E402
import ev_battery_harness_demo as demo  # noqa: E402

HF_CACHE = Path(os.environ.get("HF_HOME") or Path.home() / ".cache" / "huggingface") / "hub"
CATALOG_CACHED = any(HF_CACHE.glob("datasets--botrail--botrail-catalog*"))
needs_catalog = pytest.mark.skipif(not CATALOG_CACHED, reason="the arms come from the catalog")


@pytest.fixture(scope="module")
def cell():
    pytest.importorskip("huggingface_hub", reason="the arms come from the catalog")
    scene, timeline, info = demo.bake()
    return scene, timeline, info["cell"]


@needs_catalog
def test_each_cable_rides_its_housings_from_the_blister_to_the_headers(cell) -> None:
    scene, timeline, c = cell
    assert sorted(timeline.ropes) == [k.name for k in P.CABLES]
    for cable in c.cables:
        track = bt.rope.track(timeline, cable.name)
        assert track["color"] == pytest.approx(list(P.HV_ORANGE))
        first, last = track["points"][0], track["points"][-1]
        # It starts straight in its groove, crimp to crimp.
        assert math.dist(first[0], (cable.kit_x, cable.length / 2, P.KIT_PLUG_Z)) < 1e-6
        assert math.dist(first[-1], (cable.kit_x, -cable.length / 2, P.KIT_PLUG_Z)) < 1e-6
        # Each housing ends on its header, and the crimp inside it with it.
        for side, end, sign in (("l", last[0], 1.0), ("r", last[-1], -1.0)):
            x, y = P.terminal_xy(cable.left if side == "l" else cable.right)
            at, _ = timeline.object_pose(c.housings[cable.name][side], timeline.duration)
            assert math.dist(at, (x, y, P.PLUG_Z)) < 1e-3
            crimp = (x, y + sign * (P.RECEPT[1] / 2 - P.CRIMP_IN), P.PLUG_Z)
            assert math.dist(end, crimp) < 3e-3, (cable.name, side, end, crimp)
        # The housings hold the cable, not the grippers.
        assert all(held == [] for held in track["held"])


@needs_catalog
def test_the_arms_plug_in_step_and_every_cable_checks_out(cell) -> None:
    scene, timeline, c = cell
    assert 55.0 < timeline.duration < 80.0
    signals = dict(timeline.signals)
    # Both grippers close and open together, once a connector.
    assert signals["L/grip"] == signals["R/grip"]
    assert sum(1 for _, on in signals["L/grip"] if on) == len(c.cables)
    rows = demo.cable_report(timeline, c)
    assert demo.verdict(rows) == []
    for row in rows:
        assert 0.0 <= row["stretch"] < demo.STRETCH_MAX
        assert row["flying_module_gap"] > 0.05
        assert 0.04 < row["droop"] < 0.10
        assert row["lv_gap"] >= demo.LV_CLEAR_MIN and row["pipe_gap"] >= demo.LV_CLEAR_MIN


@needs_catalog
def test_the_cables_cross_the_channel_in_order(cell) -> None:
    """In plan the five cross the channel's middle in the order they are
    fitted along it: the zig-zag never crosses itself."""
    scene, timeline, c = cell
    crossings = []
    for cable in c.cables:
        points = bt.rope.track(timeline, cable.name)["points"][-1]
        middle = min(points, key=lambda p: abs(p[1]))
        crossings.append(middle[0])
    assert crossings == sorted(crossings)
    assert min(b - a for a, b in zip(crossings, crossings[1:])) > 0.08


@needs_catalog
def test_the_recording_carries_the_cables_as_tubes(cell, tmp_path) -> None:
    scene, timeline, c = cell
    out = tmp_path / "ev.usda"
    timeline.export_usd(str(out), fps=10.0)
    text = out.read_text()
    assert 'def Xform "Ropes"' in text
    for cable in c.cables:
        assert f'def Mesh "{cable.name}"' in text


def test_the_verdict_refuses_by_name() -> None:
    short = dict(cable="c1", part="HV-MC-35-250", length=0.25, stretch=0.059, flying_module_gap=0.13,
                 droop=0.001, lv_gap=0.035, pipe_gap=0.15, bend=3.8)
    long = dict(cable="c2", part="HV-MC-35-400", length=0.40, stretch=0.004, flying_module_gap=0.11,
                droop=0.11, lv_gap=-0.004, pipe_gap=0.047, bend=0.05)
    near = dict(long, cable="c3", lv_gap=0.006)
    refused = demo.verdict([short, long, near])
    assert refused == [
        "c1 is pulled 5.9 % longer between the hands (over 1 %): 250 mm is too short",
        "c2 lies on the LV harness: 400 mm droops too far",
        "c3 comes 6 mm from the LV harness (at least 10 mm): 400 mm droops too far",
    ]

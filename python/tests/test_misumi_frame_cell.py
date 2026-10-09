"""`examples/assembly/misumi_frame_cell_demo.py` — a screwdriving cell in a
guard of MISUMI aluminium frame, asserted the way the equipment's designer
would. The frames, the arm and the cell's products come from the catalog,
so these tests skip where it is unreachable. What they pin:

* the frames are the order: the base and the guard cut from the HFS6
  pack on its 0.5 mm grid and weighed by its kg/m, one line per article
  across both frames, the sheets by size after them;
* the guard keeps its 50 mm from everything the arm does, face by face,
  and no more than it needs — `fit` finds the same guard again from a
  roomy one;
* the shutter: the arm moves only while its closed switch is made, and
  the FAT rows refuse the cycle where they must;
* the equipment stands where it is drawn: the controllers inside the
  base, the guard on the plate.
"""

import sys
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "assembly"))

import misumi_frame_cell_demo as demo


def _bake_or_skip(**kwargs):
    try:
        return demo.bake(**kwargs)
    except Exception as err:
        if "catalog" in str(err).lower() or "fetch" in str(err).lower() or "resolve" in str(err).lower():
            pytest.skip(f"catalog unavailable: {err}")
        raise


@pytest.fixture(scope="module")
def cell():
    return _bake_or_skip()


def test_the_frames_are_the_cut_list(cell) -> None:
    scene, *_ = cell
    rows = scene.bom().rows
    by_model = {row["model"]: row for row in rows}
    # The guard, W 965 x D 1000 x H 760 in 30 x 30: the window's mid-rail is
    # a third member as long as the lower width rails.
    guard = {m: r["qty"] for m, r in by_model.items() if m.startswith("HFSB6-3030-")}
    assert guard == {"HFSB6-3030-965": 2, "HFSB6-3030-940": 4, "HFSB6-3030-905": 3, "HFSB6-3030-730": 4}
    # The base under it: 6060 legs a 3060 rail shorter than its 740 mm.
    base = {m: r["qty"] for m, r in by_model.items() if m.startswith(("HFSB6-6060-", "HFSB6-3060-"))}
    assert base == {"HFSB6-3060-965": 2, "HFSB6-3060-940": 2, "HFSB6-3060-880": 2, "HFSB6-3060-845": 2,
                    "HFSB6-6060-680": 4}
    # One line per article across both frames: 24 + 26 brackets, two bolts
    # and two nuts to each.
    brackets = by_model["HBLFS6"]
    assert brackets["qty"] == 50
    assert sorted(brackets["names"]) == ["base/hardware/brackets", "guard/hardware/brackets"]
    assert by_model["CBM6-12"]["qty"] == 100 and by_model["HNTT6-6"]["qty"] == 100
    # Every profile on the pack's grid, weighed by its kg/m (to the gram).
    per_m = {"3030": 0.9, "3060": 1.6, "6060": 2.63}
    for row in rows:
        if row["category"] == "structure.frame.profile":
            length = float(row["model"].rsplit("-", 1)[1])
            assert length * 2 == int(length * 2)
            kg = per_m[row["model"].split("-")[1]] * length / 1000
            assert row["attributes"]["mass_kg"] == pytest.approx(kg, abs=0.001)
    # The sheets after the frames, by size, with no part number to give.
    sheets = {m: r["qty"] for m, r in by_model.items() if m.startswith("PC panel")}
    assert sheets == {"PC panel 917x362 t5": 1, "PC panel 917x712 t5": 1, "PC panel 952x712 t5": 2,
                      "PC panel 917x952 t5": 1}
    assert by_model["PC door 935x350 t5"]["category"] == "structure.door"
    assert scene.bom().unidentified() == []


def test_the_guard_keeps_its_clearance_on_every_face(cell) -> None:
    scene, tl, *_ = cell
    measured = demo.clearances(scene, tl)
    assert set(measured) == set(demo.FACES)
    for face, clearance in measured.items():
        # Kept, never touched — and no more than the 5 mm grid and an
        # oblique approach give away: the guard is fitted.
        assert demo.CLEARANCE - 1e-9 <= float(clearance) < demo.CLEARANCE + 0.010, (face, clearance)
        assert clearance.pair is None
    # What the arm is meant to come close to — the screw in its hole — is
    # not the guard's business.
    assert float(tl.min_clearance(0.02)) < 0.005


def test_fit_finds_the_guard_again_from_a_roomy_one(cell) -> None:
    roomy = {face: value + 0.15 for face, value in demo.GUARD.items()}
    scene, tl, *_ = _bake_or_skip(guard=roomy)
    assert demo.fit(roomy, demo.clearances(scene, tl)) == pytest.approx(demo.GUARD)


def test_the_arm_moves_only_behind_the_closed_shutter(cell) -> None:
    scene, tl, joint, driver, *_ = cell
    closed = tl.signal("shutter/closed").high_spans()
    assert len(closed) == 1
    t_closed, t_open = closed[0]
    assert t_closed == pytest.approx(demo.WINDOW / demo.SHUTTER_SPEED, abs=0.05)
    moves = tl.moves(demo.ROBOT)
    assert moves and all(t_closed <= start and end <= t_open for _label, start, end in moves)
    assert tl.signal("shutter/open").value_at(0.0) and tl.signal("shutter/open").value_at(tl.duration)
    # The drive and its two switches are on the bill — the switches one
    # line, two of a kind.
    rows = {name: row for row in scene.bom().rows for name in row["names"]}
    assert rows["shutter"]["category"] == "axis.linear"
    assert rows["shutter/closed"] is rows["shutter/open"] and rows["shutter/open"]["qty"] == 2


def test_the_fat_rows_refuse_what_they_must(cell) -> None:
    scene, tl, joint, driver, *_ = cell
    runs = scene.simulate_scenarios(["assemble", driver.program], max_duration=tl.duration + 30.0)
    refused = {name for name in scene.scenario_names if runs.errors.get(name)}
    assert refused == {"nok_twice", "estop_pressed", "feeder_empty", "start_wire_open", "shutter_jammed"}
    # A jammed shutter and a pressed E-stop stop the arm before it moves.
    for name in ("shutter_jammed", "estop_pressed"):
        assert "assemble/guarded" in runs.errors[name]


def test_the_equipment_stands_where_it_is_drawn(cell) -> None:
    scene, *_ = cell
    w, d, cx, cy = demo.outline(demo.GUARD)
    legs = 0.06
    inside = [n for n in scene.obstacle_names if n.startswith(("controller/", "driver/controller/"))]
    assert inside
    for name in inside:
        lo, hi = scene.obstacle_bounds(name)
        assert cx - w / 2 + legs <= lo[0] and hi[0] <= cx + w / 2 - legs, name
        assert cy - d / 2 + legs <= lo[1] and hi[1] <= cy + d / 2 - legs, name
        assert hi[2] <= demo.BASE_H, name
    (_x, _y, top), _ = scene.frame("base/top")
    bottoms = [scene.obstacle_bounds(n)[0][2] for n in scene.obstacle_names if n.startswith("guard/profiles/")]
    assert min(bottoms) == pytest.approx(top)

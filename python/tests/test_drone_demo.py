"""The warehouse cycle-count cell (`examples/drone_survey_demo.py`),
asserted the way its owner would assert it.

A UR12e with a vacuum gripper palletizes cases at the mouth of an aisle
while a PX4 X500 flies an inventory count down it. Both machines, the
racks and the case infeed are catalog products, so what these tests pin is
the part that is easy to break silently:

* the machines really are the ordered ones — three lines for the arm, its
  adapter plate and its cup, one for the airframe, each with its package,
* the aerial drive's clock is closed form and the propellers turn only
  while it flies,
* the count reads one location per stop and finds the empty one,
* the zone handshake is what makes the cell legal: the same paths and the
  same volumes without it are refused, naming a link of each machine and
  the instant,
* and a crossing flown under the staging stack is refused outright.

The cell is built from catalog products, so these tests need the catalog
packages — cached locally or fetched once. Where the catalog is
unreachable they skip rather than fail; the engine's own coverage lives in
the Rust suites.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import botrail as bt
import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES))

HF_CACHE = Path(os.environ.get("HF_HOME") or Path.home() / ".cache" / "huggingface") / "hub"
HAS_CATALOG = any(HF_CACHE.glob("datasets--botrail--botrail-catalog*"))
needs_catalog = pytest.mark.skipif(not HAS_CATALOG, reason="botrail catalog not in the HF cache")

if HAS_CATALOG:
    import drone_survey_demo as demo


@pytest.fixture(scope="module")
def shift():
    return demo.bake()


@needs_catalog
def test_the_cell_is_ordered_not_drawn(shift) -> None:
    """Every machine in the cell names the package it came from. The arm is
    three products — arm, adapter plate, cup — because the gripper's
    manifest says it needs the plate, and the airframe is one, because a
    rigid mount on a bodiless vehicle *is* the machine."""
    scene, _ = shift
    rows = {r["names"][0]: r for r in scene.bom().rows}
    arm, plate, cup = rows["palletizer"], rows["palletizer/tool"], rows["palletizer/tool2"]
    assert (arm["model"], arm["category"]) == ("UR12e", "manipulator")
    assert arm["attributes"]["payload_kg"] == 12.5 and arm["attributes"]["reach_mm"] == 1300
    assert plate["category"] == "adapter" and cup["category"] == "gripper.vacuum"
    # The cup lifts a case with room to spare — which is the point of
    # deriving the requirement rather than trusting the drawing.
    assert cup["attributes"]["payload_kg"] >= demo.CASE_KG
    drone = rows["drone"]
    assert (drone["model"], drone["category"]) == ("X500", "vehicle.uav")
    assert all(r.get("catalog") for r in (arm, plate, cup, drone))
    # The scenery is ordered too: the bays and the belt are catalog sizes.
    assert any(r["category"] == "structure.rack" for r in scene.bom().rows)
    assert any(r["category"] == "conveyor.belt" for r in scene.bom().rows)


@needs_catalog
def test_the_count_reads_one_location_per_stop(shift) -> None:
    """Nine locations, eight answers. The read window is no taller than a
    label on purpose: one as deep as the shelf pitch would answer two
    locations at once and count neither — and then the empty shelf, which
    is the entire reason to fly a count, would never show up."""
    scene, tl = shift
    blips = [t for t, v in dict(tl.signals)["scan"] if v]
    assert len(blips) == len(demo.BAYS) * demo.LEVELS - 1 == 8
    bay, level = demo.EMPTY
    assert f"bay{bay + 1}/tote{level}" not in scene.obstacle_names
    # Every other location is really there to be found.
    assert sum(1 for n in scene.obstacle_names if "/tote" in n) == 8


@needs_catalog
def test_the_airframe_flies_its_own_rates_and_turns_its_props(shift) -> None:
    """The cell states indoor rates; the catalog caps them. Each leg's
    clock is the slower axis, closed form — the serpentine's climbs are
    climb-limited and its runs cruise-limited — and the propellers turn at
    their authored rate exactly while the machine is off the pad."""
    scene, tl = shift
    row = scene.requirements(timeline=tl)["drone"]
    asked = {r.key: r for r in row.requirements}
    assert asked["max_speed_mps"].value == pytest.approx(demo.SPEED)
    assert asked["max_climb_mps"].value == pytest.approx(demo.CLIMB)
    assert asked["max_descent_mps"].value == pytest.approx(demo.DESCENT)
    assert all(r.status == "ok" for r in row.requirements), row.requirements
    # One shelf pitch is a climb-limited leg, one bay pitch a cruise one.
    pitch = demo.SCAN_Z[1] - demo.SCAN_Z[0]
    spans = {n: (a, b) for n, a, b in tl.step_spans}
    a0, a1 = spans["count/fly_b1l1"]
    assert a1 - a0 == pytest.approx(pitch / demo.CLIMB, abs=0.05)
    b0, b1 = spans["count/fly_b2l2"]
    assert b1 - b0 == pytest.approx((demo.BAYS[1] - demo.BAYS[0]) / demo.SPEED, abs=0.05)
    # Props: turned by the airborne seconds, still on the pad.
    airborne = tl.vehicle_airborne("drone")
    turn = tl.sample(tl.duration, robot="drone")[0] - tl.sample(0.0, robot="drone")[0]
    assert turn == pytest.approx(demo.SPIN * airborne, abs=1.0)
    assert tl.sample(2.0, robot="drone")[0] == pytest.approx(0.0, abs=1e-9)


@needs_catalog
def test_the_handshake_is_what_makes_the_cell_legal(shift) -> None:
    """The whole conversation, in order and on the chart: the drone asks
    from the pad, the palletizer finishes its case and parks, the aisle is
    granted, the count flies, landing hands it back. The cell is only legal
    because of the order — the paths never change."""
    _scene, tl = shift
    lanes = dict(tl.signals)
    asked = next(t for t, v in lanes["count_request"] if v)
    granted = next(t for t, v in lanes["aisle_clear"] if v)
    done = next(t for t, v in lanes["count_done"] if v)
    handed_back = next(t for t, v in lanes["aisle_clear"] if not v and t > granted)
    assert asked < granted < done <= handed_back
    spans = {n: (a, b) for n, a, b in tl.step_spans}
    # The case in the cup when the drone asked is finished, not dropped.
    assert spans["palletize/release1"][1] < granted
    # The drone leaves the pad only once the aisle is its own.
    assert spans["count/launch"][0] >= granted - 1e-6
    # ... and every scan happens inside the granted window.
    for name, (start, end) in spans.items():
        if name.startswith("count/read_"):
            assert granted <= start and end <= handed_back
    # What it cost: a real block, and the number a planner asks for.
    stood_by = sum(b - a for n, (a, b) in spans.items()
                   if n in ("palletize/park", "palletize/stand_by"))
    assert 20.0 < stood_by < tl.duration


@needs_catalog
def test_the_same_cell_without_the_handshake_is_refused() -> None:
    """Same machines, same paths, same volumes — only the clock differs,
    and the bake refuses at the instant the two meet, naming a link of
    each. That is what the cross-robot check is for."""
    with pytest.raises(ValueError) as refusal:
        demo.bake(interlock=False)
    said = " ".join(str(refusal.value).split())
    assert "palletizer" in said and "drone" in said and "collide" in said, said
    assert "×" in said, said            # a link of each machine, named
    assert "interlock" in said, said    # and what to do about it


@needs_catalog
def test_a_crossing_under_the_staging_stack_is_refused() -> None:
    """No timing fixes a lane flown through the pallet the arm is
    building: the refusal names the case it would hit, and the drone's own
    airframe link that hits it."""
    with pytest.raises(ValueError) as refusal:
        demo.bake(lane=0.5)
    said = " ".join(str(refusal.value).split())
    assert "case" in said and "drone" in said, said


@needs_catalog
def test_the_arm_is_taught_against_its_own_reach() -> None:
    """Nothing about the palletizer's poses is typed in: move the machine
    out of its own range and the cell says so before anything is baked,
    naming the point it cannot reach and by how much."""
    far = demo.PALLET_XY[1] + 2.5
    with pytest.raises(RuntimeError, match="cannot reach"):
        demo.build(pallet_y=far)
    # …and a wound wrist never survives teaching: every taught pose is the
    # short way round from the one the arm arrives in.
    scene = demo.build()
    named = [n for n in scene.motion_names if n.startswith(("pick", "set"))]
    for name in named:
        for joint in [q for _, q in scene.motion_segments(name)][-1]:
            assert abs(joint) <= 2 * math.pi


@needs_catalog
def test_the_cell_regenerates_and_the_shift_is_deterministic(shift, tmp_path) -> None:
    """The saved project rebuilds the same cell — the aerial drive, the
    fixed yaw the scanner needs, the propeller spin — and baking twice
    gives the same shift, because that is what makes any of these numbers
    worth quoting."""
    scene, tl = shift
    code = scene.generate_python()
    assert 'drive="aerial"' in code and "fixed_yaw=" in code and "spin={" in code
    scene.save_project(tmp_path / "cell.botrail")
    again = bt.Scene.load_project(tmp_path / "cell.botrail")
    assert 'drive="aerial"' in again.generate_python()
    assert again.simulate_sequences(["palletize", "count"],
                                    max_duration=300.0).duration == pytest.approx(tl.duration)

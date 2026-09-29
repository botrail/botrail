"""The automated forklift cell (`examples/vehicles/forklift_demo.py`), asserted
the way its owner would: the truck really is the ordered one, the pallets
end where the shift put them, the aisle the sheet quotes is what the sweep
finds, the mast that cannot reach a level is refused by the sheet's own
rule, and the truck travels the aisle drive-unit first.

The cell is built from catalog products (the SAE160 packages, TRUSCO
racking), so these tests need them — from a catalog builder's `build/`
directory when `BOTRAIL_CATALOG_BUILD` names one (the sibling checkout is
tried by default), else from the published dataset, cached locally or
fetched once. Where neither can be reached they skip rather than fail.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "vehicles"))

BUILD = Path(os.environ.get("BOTRAIL_CATALOG_BUILD")
             or EXAMPLES.parents[0].parent / "botrail-catalog-builder" / "build")
HAS_BUILD = (BUILD / "toyota_material_handling/autopilot/sae160/r1/manifest.yaml").is_file() \
    and (BUILD / "trusco/pallet-rack/1d-2500x1100/r1/manifest.yaml").is_file()
HF_CACHE = Path(os.environ.get("HF_HOME") or Path.home() / ".cache" / "huggingface") / "hub"
HAS_CATALOG = any(HF_CACHE.glob("datasets--botrail--botrail-catalog*"))
needs_packages = pytest.mark.skipif(not (HAS_BUILD or HAS_CATALOG),
                                    reason="forklift packages neither built locally nor cached")

import forklift_demo as demo

if HAS_BUILD:
    demo.CATALOG_ROOT = BUILD.resolve()


@pytest.fixture(scope="module")
def shift():
    return demo.bake()


@needs_packages
def test_the_cell_is_ordered_not_drawn(shift) -> None:
    scene, truck, _info, _tl = shift
    rows = {r["names"][0]: r for r in scene.bom().rows}
    assert rows["agf"]["category"] == "vehicle.forklift"
    assert rows["agf"]["catalog"].startswith("toyota_material_handling/autopilot/sae160/")
    assert rows["rack/A/unit"]["model"] == "1D-35B25-11-2" and rows["rack/A/unit"]["qty"] == 2
    assert rows["pallet/A"]["category"] == "pallet" and rows["pallet/A"]["qty"] == 2
    assert rows["charger"]["category"] == "vehicle.charger"
    # The truck's numbers came out of its package, and it drives as a tricycle.
    assert truck.h23 == pytest.approx(4.7) and truck.x == pytest.approx(0.64)
    assert truck.wheels.drive == "tricycle" and truck.wheels.steer == {"drive_wheel": "steer"}
    assert truck.mast == ["free_lift", "mast_lift"]
    assert truck.swing == pytest.approx(truck.wa, rel=0.01)


@needs_packages
def test_the_pallets_end_where_the_shift_put_them(shift) -> None:
    _scene, _truck, info, tl = shift
    end = tl.duration
    seat = info["seats"][("A", 1, 2)]
    (ax, ay, az), _ = tl.object_pose("pallet/A/deck2", end)
    assert (ax, ay) == pytest.approx((seat[0], seat[1]), abs=0.15)
    assert az - demo.PALLET[2] + 0.011 == pytest.approx(seat[2], abs=0.002)   # deck board centre over the seat
    (bx, by, bz), _ = tl.object_pose("pallet/B/deck2", end)
    assert (bx, by) == pytest.approx((info["ship"][0], info["ship"][1] + 0.02), abs=0.15)
    assert bz - demo.PALLET[2] + 0.011 == pytest.approx(0.0, abs=0.002)
    # The cases went with their pallets.
    (lx, ly, lz), _ = tl.object_pose("pallet/A/load", end)
    assert lz == pytest.approx(seat[2] + demo.PALLET[2] + demo.LOAD[2] / 2, abs=0.002)
    lanes = dict(tl.signals)
    assert any(v for _, v in lanes["bay1_full"]) and any(v for _, v in lanes["ship_full"])
    assert lanes["at_charger"][-1][1] is True


@needs_packages
def test_the_truck_travels_the_aisle_drive_unit_first(shift) -> None:
    _scene, _truck, _info, tl = shift
    # Leaving the received pallet's spot for the rack: the truck backs east
    # down the lane, forks trailing, at the sheet's drive-unit-first speed.
    span = tl.step_span("to_bay1_pre")
    t0, t1 = span.start + 0.3 * span.duration, span.start + 0.7 * span.duration
    (x0, _, _), q0 = tl.base_pose(t0, robot="agf")
    (x1, _, _), q1 = tl.base_pose(t1, robot="agf")
    assert x1 > x0 + 1.0
    for q in (q0, q1):
        yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]))
        assert abs(abs(yaw) - math.pi) < 1e-6          # facing west while moving east
    assert (x1 - x0) / (t1 - t0) == pytest.approx(2.22, abs=0.05)
    # Into the bay it goes forks first, at the forks-first speed.
    span = tl.step_span("to_bay1")
    (_, y0, _), q = tl.base_pose(span.start + 0.6 * span.duration, robot="agf")
    (_, y1, _), _ = tl.base_pose(span.start + 0.9 * span.duration, robot="agf")
    yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]))
    assert abs(yaw - math.pi / 2) < 1e-6 and y1 > y0
    assert (y1 - y0) / (0.3 * span.duration) == pytest.approx(0.31, abs=0.05)


@needs_packages
def test_a_narrow_aisle_is_refused_by_name_and_the_sheets_ast_passes() -> None:
    with pytest.raises(ValueError, match=r"collides with `rack/B/") as refused:
        demo.bake(aisle=2.5)
    assert "riding `agf_base`" in str(refused.value)
    _scene, truck, _info, tl = demo.bake(aisle=2.8)   # the sheet quotes 2.696 m
    assert truck.ast == pytest.approx(2.696) and tl.duration > 0


@needs_packages
def test_the_duplex_mast_is_refused_for_level_2_by_the_sheets_rule() -> None:
    with pytest.raises(ValueError, match="h23 − 200 mm = 2.15 m") as refused:
        demo.bake(mast="dx", level=2)
    assert "use --level 1" in str(refused.value)
    _scene, truck, info, tl = demo.bake(mast="dx", level=1)
    assert truck.mast == ["mast_lift"] and truck.h23 == pytest.approx(2.35)
    seat = info["seats"][("A", 1, 1)]
    (_, _, az), _ = tl.object_pose("pallet/A/deck2", tl.duration)
    assert az - demo.PALLET[2] + 0.011 == pytest.approx(seat[2], abs=0.002)


@needs_packages
def test_a_truck_that_cannot_pivot_rounds_the_corners_instead() -> None:
    """`--turn-radius`: the same route driven as a counterbalance truck —
    every corner an arc, the bay entered by an overshoot and a swing back —
    with the lane and the stubs sized for the forks to clear a pallet
    before the arc begins. The heading is never a pivot's jump: half way
    into the bay's corner it points between the lane and the bay."""
    _scene, _truck, info, tl = demo.bake(turn_radius=0.6, aisle=3.5)
    assert info["turn_radius"] == 0.6 and info["lane"] < info["face"] - 1.3
    span = tl.step_span("to_bay1")
    yaws = []
    for k in range(1, 40):
        _, q = tl.base_pose(span.start + span.duration * k / 40, robot="agf")
        yaws.append(math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])))
    off_axis = [y for y in yaws if abs(math.remainder(y, math.pi / 2)) > 0.05]
    assert off_axis, "a steered drive rounds the corner rather than pivoting"
    seat = info["seats"][("A", 1, 2)]
    (_, _, az), _ = tl.object_pose("pallet/A/deck2", tl.duration)
    assert az - demo.PALLET[2] + 0.011 == pytest.approx(seat[2], abs=0.002)


@needs_packages
def test_the_cell_asks_the_truck_what_its_sheet_answers(shift) -> None:
    """`scene.requirements()` on the truck's line reads it the way its
    type sheet is written: the lift height the put-away needs (the fork
    seat at the level's beams plus the entry clearance), the lowered fork
    height the floor pallet's pockets allow, the pallet and its cases
    lifted as one, and both gears' speeds — each answered by the sheet."""
    scene, truck, info, _tl = shift
    row = scene.requirements()["agf"]
    assert row.category == "vehicle.forklift" and row.status == "ok" and row.notes == []
    asked = {r.key: r for r in row.requirements}
    need = (info["seat_put"] + demo.STRINGER + demo.ENTRY_CLEAR) * 1000
    assert asked["lift_height_mm"].value == pytest.approx(need, abs=0.1)
    assert "`raise_a`" in asked["lift_height_mm"].basis and asked["lift_height_mm"].provided == truck.h23 * 1000
    assert asked["fork_height_lowered_mm"].op == "<=" and asked["fork_height_lowered_mm"].value <= demo.STRINGER * 1000
    assert asked["payload_kg"].value == pytest.approx(demo.LOAD_KG + demo.PALLET_KG)
    assert asked["payload_kg"].basis == "grasps pallet/A + pallet/A/load 625 kg"
    assert asked["max_speed_mps"].value == pytest.approx(truck.speed_drive) and "reverse" in asked["max_speed_mps"].basis
    assert asked["max_speed_fork_first_mps"].value == pytest.approx(truck.speed_forks)

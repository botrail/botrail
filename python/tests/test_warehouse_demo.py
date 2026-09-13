"""The warehouse from the consultation sketch (`examples/vehicles/warehouse_demo.py`),
asserted the way its owner would assert it.

Two pallet AMRs, a case picker and the traffic control between them, all
ordered from the catalog. What these tests pin is what is easy to break
silently:

* the machines really are the ordered ones — carrier, lift, stands,
  racking, arm, its tool and its box each on the bill with a package,
* a pallet rides the lift: taken off a stand, set down on another, cases
  and all, at the stand's own seat height,
* the picking station's call is answered: the empty pallet leaves, a full
  one arrives, and the picker resumes on it,
* the traffic control is what makes two machines on one aisle legal — the
  same paths without it are refused at a named time, naming the machines,
* and a narrower aisle is refused by name too.

The cell is built from catalog products, so these tests need the packages
— from a catalog builder's `build/` directory when `BOTRAIL_CATALOG_BUILD`
names one (the sibling checkout is tried by default), else from the
published dataset, cached locally or fetched once. Where neither can be
reached they skip rather than fail.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "vehicles"))

BUILD = Path(os.environ.get("BOTRAIL_CATALOG_BUILD")
             or EXAMPLES.parents[0].parent / "botrail-catalog-builder" / "build")
HAS_BUILD = (BUILD / "mobile_industrial_robots/mir/mir1350/r1/manifest.yaml").is_file() \
    and (BUILD / "universal_robots/ur/ur20/r2/manifest.yaml").is_file()
HF_CACHE = Path(os.environ.get("HF_HOME") or Path.home() / ".cache" / "huggingface") / "hub"
HAS_CATALOG = any(HF_CACHE.glob("datasets--botrail--botrail-catalog*"))
HAS_PACKAGES = HAS_BUILD or HAS_CATALOG
needs_packages = pytest.mark.skipif(not HAS_PACKAGES, reason="warehouse packages neither built locally nor cached")

if HAS_PACKAGES:
    import warehouse_demo as demo

    if HAS_BUILD:
        demo.CATALOG_ROOT = BUILD.resolve()


@pytest.fixture(scope="module")
def shift():
    return demo.bake()


@needs_packages
def test_the_shed_is_ordered_not_drawn(shift) -> None:
    scene, machine, _ = shift
    rows = {r["names"][0]: r for r in scene.bom().rows}
    assert rows["amr1"]["model"] == "MiR1350" and rows["amr1"]["qty"] == 2
    assert rows["amr1"]["catalog"].startswith("mobile_industrial_robots/mir/mir1350/")
    assert rows["amr1_lift"]["category"] == "vehicle.top_module" and rows["amr1_lift"]["qty"] == 2
    assert rows["stand/recv"]["category"] == "structure.pallet_stand" and rows["stand/recv"]["qty"] == 9
    assert rows["rack/A/unit"]["model"] == "1D-40B25-11-3" and rows["rack/A/unit"]["qty"] == 4
    assert rows["rack/A/ext"]["model"] == "1D-40B25-11-3B" and rows["rack/A/ext"]["qty"] == 12
    assert rows["charger/1"]["category"] == "vehicle.charger" and rows["charger/1"]["qty"] == 2
    assert rows["picker"]["model"] == "UR20" and rows["picker/tool2"]["category"] == "gripper.vacuum"
    assert rows["pack_line"]["category"] == "conveyor.belt"
    # The machine's numbers came out of its packages.
    assert machine.deck == pytest.approx(machine.specs["deck_height_mm"] / 1e3)
    assert machine.stroke == pytest.approx(0.06)
    assert machine.lowered < demo.STAND_SUPPORT   # it fits under a loaded stand


@needs_packages
def test_a_pallet_rides_the_lift_from_stand_to_stand(shift) -> None:
    scene, machine, tl = shift
    end = tl.duration
    # ①: the received pallet and its cases end on rack A's stand, seated at
    # the stand's own support height, cases still on it.
    seat, _ = scene.frame("stand/pdA/pallet")
    deck, _ = tl.object_pose("p1/deck2", end)
    assert deck[0] == pytest.approx(seat[0], abs=0.02) and deck[1] == pytest.approx(seat[1], abs=0.02)
    assert deck[2] == pytest.approx(seat[2] + 0.144 - 0.011, abs=0.003)
    case, _ = tl.object_pose("p1/case00", end)
    assert abs(case[0] - seat[0]) < 0.6 and case[2] > seat[2] + 0.144
    # ③: the finished pallet is on the shipping stand.
    ship, _ = scene.frame("stand/ship/pallet")
    deck4, _ = tl.object_pose("p4/deck2", end)
    assert (deck4[0], deck4[1]) == pytest.approx((ship[0], ship[1]), abs=0.02)
    # Mid-carry the pallet is 60 mm up and moving with the machine.
    lift = tl.step_span("receiving/lift_p1")
    drive = tl.step_span("receiving/to_pdA")
    z_on_stand = tl.object_pose("p1/deck2", lift.start)[0][2]
    z_carried = tl.object_pose("p1/deck2", drive.start + 1.0)[0][2]
    assert z_carried - z_on_stand == pytest.approx(0.06, abs=0.003)
    p_a = tl.object_pose("p1/deck2", drive.start + 1.0)[0]
    p_b = tl.object_pose("p1/deck2", drive.start + 3.0)[0]
    assert (p_a[0], p_a[1]) != pytest.approx((p_b[0], p_b[1]), abs=0.05)


@needs_packages
def test_the_call_is_answered(shift) -> None:
    scene, _, tl = shift
    lanes = dict(tl.signals)
    call = next(t for t, v in lanes["call"] if v)
    done = next(t for t, v in lanes["supply_done"] if v)
    assert 0 < call < done < tl.duration
    # The empty pallet went to the store and the full one took its place.
    store, _ = scene.frame("stand/empty/pallet")
    pick, _ = scene.frame("stand/pick/pallet")
    p2, _ = tl.object_pose("p2/deck2", tl.duration)
    p3, _ = tl.object_pose("p3/deck2", tl.duration)
    assert (p2[0], p2[1]) == pytest.approx((store[0], store[1]), abs=0.02)
    assert (p3[0], p3[1]) == pytest.approx((pick[0], pick[1]), abs=0.02)
    # The picker took every case off both pallets onto the belt.
    infeed, _ = scene.frame("pack_line/infeed")
    for name in [f"p2/case0{k}" for k in range(4)] + [f"p3/case0{k}" for k in range(4)]:
        p, _ = tl.object_pose(name, tl.duration)
        assert p[0] > infeed[0] and abs(p[1] - infeed[1]) < 0.05 and p[2] > demo.BELT_TOP
    # Traffic: every aisle leg drove on a grant, one machine at a time.
    for name in ("amr1", "amr2"):
        for t0, t1 in demo._high_spans(lanes[f"aisle_{name}"], tl.duration):
            other = "amr2" if name == "amr1" else "amr1"
            assert not any(a < t1 and b > t0 for a, b in demo._high_spans(lanes[f"aisle_{other}"], tl.duration))


@needs_packages
def test_without_traffic_control_the_machines_meet() -> None:
    with pytest.raises(ValueError, match="amr1.*amr2|amr2.*amr1"):
        demo.bake(interlock=False)


@needs_packages
def test_a_narrow_aisle_is_refused_by_name() -> None:
    with pytest.raises(ValueError, match="collides with"):
        demo.bake(aisle=1.5)

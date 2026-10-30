"""The cooperative transport cell (`examples/vehicles/cooperative_transport_demo.py`),
asserted the way its owner would: the two platforms are the ordered KMP 3000P,
they carry the bed rigidly from the assembly bay to the inspection bay and turn
it about its centre, the parts AMR waits for the aisle, a narrower crossing is
refused by name at the turn, and dropping the traffic control is refused with a
time and a place (design-cooperative-transport.md K3).

The cell is built from catalog products (KMP 3000P, MiR250), so these tests
need them — from a catalog builder's `build/` directory when
`BOTRAIL_CATALOG_BUILD` names one (the sibling checkout is tried by default),
else from the published dataset, cached locally. Where neither can be reached
they skip rather than fail.
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
HAS_BUILD = (BUILD / "kuka/kmp/kmp-3000p/r1/manifest.yaml").is_file() \
    and (BUILD / "mobile_industrial_robots/mir/mir250/r2/manifest.yaml").is_file()
HF_CACHE = Path(os.environ.get("HF_HOME") or Path.home() / ".cache" / "huggingface") / "hub"
HAS_CATALOG = any(HF_CACHE.glob("datasets--botrail--botrail-catalog*"))
needs_packages = pytest.mark.skipif(not (HAS_BUILD or HAS_CATALOG),
                                    reason="KMP 3000P / MiR250 packages neither built locally nor cached")

import cooperative_transport_demo as demo

if HAS_BUILD:
    demo.CATALOG_ROOT = BUILD.resolve()


@pytest.fixture(scope="module")
def cycle():
    return demo.bake()


def _planar(pose):
    (x, y, _), (qx, qy, qz, qw) = pose
    return x, y, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


@needs_packages
def test_the_platforms_are_the_ordered_ones(cycle) -> None:
    scene, _tl = cycle
    rows = {r["names"][0]: r for r in scene.bom().rows}
    kmp = next(r for name, r in rows.items() if name.startswith("kmp"))
    assert kmp["catalog"].startswith("kuka/kmp/kmp-3000p/") and kmp["qty"] == 2
    assert rows["bed"]["category"] == "workpiece"


@needs_packages
def test_the_bed_rides_the_formation_and_is_set_down_in_the_inspection_bay(cycle) -> None:
    _scene, tl = cycle
    carry0 = tl.step_span("carry/to_corner").start
    carry1 = tl.step_span("carry/to_inspection").end
    # The platforms hold their slots under the bed through the straights and the turn.
    for k in range(21):
        t = carry0 + (carry1 - carry0) * k / 20
        bx, by, bth = _planar(tl.object_pose("bed/base", t))
        for unit, ahead in (("kmp1_unit", demo.SLOT), ("kmp2_unit", -demo.SLOT)):
            (ux, uy, _), _ = tl.base_pose(t, robot=unit)
            assert math.hypot(ux - (bx + ahead * math.cos(bth)), uy - (by + ahead * math.sin(bth))) < 1e-6, (unit, t)
    x, y, yaw = _planar(tl.object_pose("bed/base", tl.duration))
    assert (x, y) == pytest.approx(demo.INSPECTION, abs=1e-6)
    assert yaw == pytest.approx(math.pi / 2, abs=1e-6)
    # Set down: the stand's legs back on the floor, as high as they started.
    (_, _, z0), _ = tl.object_pose("stand/leg_fl", 0.0)
    (_, _, z1), _ = tl.object_pose("stand/leg_fl", tl.duration)
    assert z1 == pytest.approx(z0, abs=1e-6)
    # The carry costs its straights and the quarter turn at the coupled rates.
    legs = (demo.CORNER[0] - demo.ASSEMBLY[0]) + (demo.INSPECTION[1] - demo.CORNER[1])
    assert carry1 - carry0 == pytest.approx(legs / demo.COUPLED_SPEED + (math.pi / 2) / demo.TURN, abs=0.15)


@needs_packages
def test_the_parts_amr_waits_for_the_formation_to_clear_the_crossing(cycle) -> None:
    _scene, tl = cycle
    corner = tl.step_span("carry/to_corner").end
    cross = tl.step_span("parts/cross")
    assert tl.step_span("parts/request").start == pytest.approx(demo.CALL_AT, abs=0.02)
    assert cross.start >= corner - 0.02


@needs_packages
def test_a_narrow_crossing_is_refused_at_the_turn() -> None:
    assert demo.sweep_radius() * math.sqrt(2) == pytest.approx(4.16, abs=0.01)
    with pytest.raises(ValueError, match=r"collides with `column/\d+` at t = 4[5-9]\.\d+s"):
        demo.bake(aisle=4.0)


@needs_packages
def test_without_traffic_control_the_parts_amr_meets_the_formation() -> None:
    with pytest.raises(ValueError, match=r"`mir_unit`.* at t = \d+\.\d+s|collide at t = .*mir"):
        demo.bake(interlock=False)


@needs_packages
def test_each_platform_bears_half_and_the_pair_is_within_kukas_load_sharing(cycle) -> None:
    scene, tl = cycle
    req, row = demo.load_share(scene, tl)
    asked = {r.key: r for r in row.requirements}
    assert asked["payload_kg"].value == pytest.approx((demo.BED_MASS + demo.STAND_MASS) / 2, abs=0.5)
    assert asked["payload_kg"].provided == 3000 and asked["payload_kg"].status == "ok"
    assert asked["cooperative_units"].provided == 2 and asked["cooperative_units"].status == "ok"
    assert asked["cooperative_payload_kg"].provided == 6000 and asked["cooperative_payload_kg"].status == "ok"
    assert req.ok
    # The parts AMR is a vehicle, not an arm: its wheel groups ask for no arms.
    assert "arm_count" not in {r.key for r in req["mir_unit"].requirements}


@needs_packages
def test_a_casting_set_off_centre_overloads_the_leading_platform() -> None:
    scene, _ = demo.build(cog=0.6)
    req, row = demo.load_share(scene)
    payload = next(r for r in row.requirements if r.key == "payload_kg")
    assert payload.status == "short" and payload.value == pytest.approx(3170, abs=1)
    assert any(f.code == "spec_short" and "kmp1_unit: payload_kg 3000 < required 3170" in f.message
               for f in req.findings())
    scene, _ = demo.build(cog=2.0)
    req, _ = demo.load_share(scene)
    assert any(f.code == "load_off_support" and "`kmp2`" in f.message for f in req.findings())

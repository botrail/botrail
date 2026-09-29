"""The pallet's timber follows the EPAL 1 layout (design-forklift.md §3.4):
what a fork, a pallet truck's arms or a stacker's support arms enter."""

from __future__ import annotations

import pytest

import botrail as bt


def extent(scene: bt.Scene, name: str) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    return scene.obstacle_bounds(name)


def test_pallet_timber_is_laid_out_like_an_epal_1() -> None:
    scene = bt.Scene()
    built = bt.parts.pallet(scene, "p", (0.0, 0.0), size=(1.2, 0.8, 0.144))
    tags = [n.rsplit("/", 1)[1] for n in built.obstacles]
    assert sum(t.startswith("bottom") for t in tags) == 3
    assert sum(t.startswith("block") for t in tags) == 9
    assert sum(t.startswith("stringer") for t in tags) == 3
    assert sum(t.startswith("deck") for t in tags) == 5
    # Bottom boards run along the length: 100 wide at the edges, 145 in the middle.
    lo, hi = extent(scene, "p/bottom0")
    assert (hi[0] - lo[0], hi[1] - lo[1]) == pytest.approx((1.2, 0.100))
    assert lo[1] == pytest.approx(-0.4)
    lo1, hi1 = extent(scene, "p/bottom1")
    assert (hi1[0] - lo1[0], hi1[1] - lo1[1]) == pytest.approx((1.2, 0.145))
    # The short-side pocket between them is an EUR pallet's 227.5 mm.
    assert lo1[1] - hi[1] == pytest.approx(0.2275)
    # Stringers run across the width over the block columns, under the deck.
    slo, shi = extent(scene, "p/stringer0")
    assert (shi[0] - slo[0], shi[1] - slo[1]) == pytest.approx((0.145, 0.8))
    assert slo[0] == pytest.approx(-0.6)
    dlo, dhi = extent(scene, "p/deck0")
    assert dhi[2] == pytest.approx(0.144)
    assert dlo[2] - shi[2] == pytest.approx(0.001)          # a millimetre clear, layer by layer
    # The pocket: bottom board top to stringer underside, 77 mm with the
    # clearances — a stacker's fork tops out at 87.5 mm and passes.
    pocket = slo[2] - hi[2]
    assert pocket == pytest.approx(0.077)
    assert hi[2] + 0.0875 < slo[2] + hi[2]
    # Blocks: 145 along the length, 100 across on the outer rows, 145 in the middle.
    blo, bhi = extent(scene, "p/block00")
    assert (bhi[0] - blo[0], bhi[1] - blo[1]) == pytest.approx((0.145, 0.100))
    blo, bhi = extent(scene, "p/block11")
    assert (bhi[0] - blo[0], bhi[1] - blo[1]) == pytest.approx((0.145, 0.145))
    assert blo[2] - hi[2] == pytest.approx(0.001)


def test_a_turned_pallet_keeps_the_pockets_on_its_short_side() -> None:
    scene = bt.Scene()
    bt.parts.pallet(scene, "p", (1.0, 2.0), size=(1.2, 0.8, 0.144), yaw=3.141592653589793 / 2)
    lo, hi = extent(scene, "p/bottom0")
    # Turned a quarter, the bottom boards run along y and the pocket opens along x.
    assert hi[1] - lo[1] == pytest.approx(1.2) and hi[0] - lo[0] == pytest.approx(0.100)

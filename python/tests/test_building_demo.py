"""The building cell (`examples/legged/building_delivery_demo.py`): six
storeys, one stair, one lift, and a machine that has to use the stair.

The walking tests need the Go2 (the catalog package, or Unitree's URDF in
the botrail cache); they skip where it cannot be had. What always runs is
the building itself — the storey the stair sets, the corridor the drawing
promises, the floors being walkable-but-not-obstacles, and the lift being
present and never commanded.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "legged"))

import building_delivery_demo as demo  # noqa: E402

FOOT_R = 0.022  # the Go2's ball feet


def _has_dog() -> bool:
    try:
        demo.patrol.dog_of("go2")
    except Exception:  # noqa: BLE001 - no catalog, no cache, no network
        return False
    return True


needs_dog = pytest.mark.skipif(not _has_dog(), reason="the Go2 is not reachable")


def _quad_building(**kwargs) -> bt.Scene:
    """The building on the primitive quadruped: no download, and every
    question below is about the building rather than the machine."""
    return demo.build(robot="quad", floors=1, racks=None, **kwargs)


# ------------------------------------------------------- the building


def test_the_stair_sets_the_storey() -> None:
    """A switchback of two flights is what a storey *is* here — order a
    different riser and the building gets taller, rather than the stair
    being asked for a rise nobody sells."""
    assert demo.storey_of(demo.RISE) == pytest.approx(3.0)
    assert demo.storey_of(demo.CODE_RISE) == pytest.approx(3.5)
    scene = _quad_building(rise=demo.CODE_RISE, tread=demo.CODE_TREAD)
    # The 1F slab follows the flight, not a constant.
    _lo, hi = scene.obstacle_bounds("1F/slab/corridor")
    assert hi[2] == pytest.approx(demo.storey_of(demo.CODE_RISE))


def test_every_floor_is_walkable_and_none_of_them_is_an_obstacle() -> None:
    """A machine standing on a floor is not a collision. Floors and landings
    are walkable so footfalls snap to them, and disabled so the aisle check
    never reports the storey the dog is standing on."""
    scene = _quad_building()
    code = scene.generate_python()
    floors = [n for n in scene.obstacle_names
              if "/slab/" in n or n.endswith("/landing") or n == "roof"]
    assert len(floors) == 2 * len(demo.LEVELS) + 1 + 1  # slabs, roof, one landing
    for name in floors:
        assert f'set_obstacle_walkable("{name}", True)' in code, name
        assert f'set_obstacle_enabled("{name}", False)' in code, name


def test_the_corridor_is_wider_than_the_drawing_asks_for() -> None:
    """通路幅 約1.5 m 以上 — measured between the two things that actually
    bound it: the spandrel on the slab edge and the office screen. Both are
    real obstacles, so this is the width the aisle check enforces."""
    scene = _quad_building()
    _lo, glass = scene.obstacle_bounds("2F/facade/e0_0")
    screen, _hi = scene.obstacle_bounds("2F/screen/e0_0")
    assert screen[1] - glass[1] == pytest.approx(demo.COR_Y1 - demo.COR_Y0)
    assert screen[1] - glass[1] >= 1.5
    # The dog fits it with room to pass a person, which is the point.
    _m, _g, footprint, _s, _t = demo.patrol.dog_of("quad")
    assert footprint[1] < 0.5 * (screen[1] - glass[1])


def test_the_glass_collides_and_is_not_drawn() -> None:
    """botrail has no transparency. The pane is what a machine meets — an
    obstacle that is never rendered — and the mullions are what a camera
    sees, drawn and never collided."""
    scene = _quad_building()
    code = scene.generate_python()
    assert 'set_obstacle_visible("2F/facade/glass", False)' in code
    assert 'set_obstacle_enabled("2F/facade/glass"' not in code
    assert 'set_obstacle_enabled("2F/facade/trim/mullion00", False)' in code


def test_the_lift_is_in_the_cell_and_never_commanded() -> None:
    """EV(使用せず): the lift is a device with a stop at every level and a
    line on the BOM. Nothing in the sequence moves it — that is the premise
    being costed, and it should be visible in the cell rather than absent
    from it."""
    scene = _quad_building()
    assert "lift" in scene.device_names
    row = next(r for r in scene.bom().rows if r["names"][0] == "lift")
    assert (row["category"], row["model"]) == ("vehicle.lift", "P-6-CO-600")
    # Parked at the lobby, which is where a lift nobody calls stays.
    _lo, hi = scene.obstacle_bounds("lift/car/floor")
    assert hi[2] == pytest.approx(demo.storey_of(demo.RISE))


def test_the_route_is_one_polyline_that_visits_every_handover() -> None:
    path, stations = demo._route(5, demo.RISE, demo.TREAD, demo.storey_of(demo.RISE))
    for i in range(1, 6):
        assert f"ho_{demo.LEVELS[i]}" in stations
        x, y, z = path[stations[f"ho_{demo.LEVELS[i]}"]]
        assert (x, y) == pytest.approx((demo.HO_X, demo.COR_MID))
        assert z == pytest.approx(i * demo.storey_of(demo.RISE))
    # Each flight is entered on a ramp, not a step: the last stretch before
    # the first tread climbs at about half the flight's grade.
    foot = path[stations["stair_B1F"] + 2]
    lead = path[stations["stair_B1F"] + 1]
    grade = (foot[2] - lead[2]) / abs(foot[0] - lead[0])
    assert grade == pytest.approx(0.5 * demo.RISE / demo.TREAD, rel=0.15)


# ------------------------------------------------------------ the walk


@needs_dog
def test_the_dog_climbs_a_storey_on_the_treads() -> None:
    scene, tl = demo.bake(floors=1)
    steps = tl.footfalls("dog")
    assert steps, "the dog never walked"
    h = demo.storey_of(demo.RISE)
    assert max(z for *_f, (_x, _y, z) in steps) == pytest.approx(h + FOOT_R, abs=1e-6)
    # It arrives at the handover point, and the zone is what says so.
    p, _q = tl.base_pose(tl.duration, "dog")
    assert (p[0], p[1]) == pytest.approx((demo.HO_X, demo.COR_MID), abs=0.05)
    at = tl.signal("at_1F")
    assert at.value_at(tl.duration) is True and at.value_at(0.0) is False
    # EV(使用せず), checked rather than asserted: a timeline tracks what
    # moved, and the car is not in it.
    with pytest.raises(ValueError, match="not tracked"):
        tl.object_pose("lift/car/floor", tl.duration)


@needs_dog
def test_the_stair_is_what_caps_the_pace() -> None:
    """One vehicle, one speed, and the flight is what sets it — not the
    corridor. The demo's own figure walks; a little faster and the leading
    leg runs out of fold on a tread, by name."""
    demo.bake(floors=1, walk=demo.WALK_SPEED)
    with pytest.raises(ValueError, match="cannot reach its footfall"):
        demo.bake(floors=5, walk=0.45)


@needs_dog
def test_a_code_stair_is_refused_for_the_dog_s_rating() -> None:
    """175 mm risers under a machine rated for 160: refused for the
    *rating*, before it walks — not by a leg running out of range half way
    up. 300 keeps 2R + T inside what the flight is sold in, so the order
    reaches the machine at all."""
    with pytest.raises(ValueError) as refusal:
        demo.bake(floors=1, rise=demo.CODE_RISE, tread=demo.CODE_TREAD)
    said = str(refusal.value)
    assert "max_step" in said and "0.175" in said and "0.160" in said, said


@needs_dog
def test_a_cart_left_in_the_corridor_is_named() -> None:
    """The thing a building actually does to a robot's route. The bake says
    which piece, and when."""
    with pytest.raises(ValueError) as refusal:
        demo.bake(floors=2, cart=True)
    assert "2F/cart/body" in str(refusal.value)

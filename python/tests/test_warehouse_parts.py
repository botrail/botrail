"""The warehouse generators: a pallet stand, a charging station, pallet racking.

Three more standard structures nobody wants to model, generated from the
numbers a data sheet gives — and, with a spec pack, from the numbers the
maker sells. What these tests pin is the contract the warehouse demo
builds on: where the pallet seat and the docking frames land, that the
collision boxes leave the vehicle's path open, and that a catalog pack
turns a run of racking into the 単体 / 連結 articles it is bought as.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

import botrail as bt

STAND_MANIFEST = """
schema_version: '0.1'
id: acme/pallet-rack/eu/r1
kind: spec
category: structure.pallet_stand
name: Pallet Stand EU
manufacturer:
  name: ACME Modules
distribution: public
mechanical:
  footprint_mm: [1300, 1178]
  height_mm: 389
  mount: floor
configuration:
  generator: pallet_stand
  params:
    pallet:
      values: [eu]
      default: eu
  components:
    - role: stand
      category: structure.pallet_stand
      part_number: PS-{pallet_code}
      codes:
        pallet: {eu: EU}
      dimensions_mm: {length: 1300, width: 1178, height: 389, support: 348, inner: 1000, leg: 60}
  rules:
    pallet_mm: [1200, 800]
"""

CHARGER_MANIFEST = """
schema_version: '0.1'
id: acme/charger/dock/r1
kind: spec
category: vehicle.charger
name: Dock Charger
manufacturer:
  name: ACME Power
distribution: public
mechanical:
  footprint_mm: [487, 622]
  height_mm: 287
  mass_kg: 20
  mount: floor
configuration:
  generator: charging_station
  params:
    supply_vac:
      values: [120, 240]
      default: 240
  components:
    - role: station
      category: vehicle.charger
      part_number: DC-48
      dimensions_mm: {length: 237, width: 622, height: 287, plate: 250}
      mass:
        base_kg: 20
  rules:
    charging_current_a: {120: 20, 240: 40}
"""

RACK_MANIFEST = """
schema_version: '0.1'
id: acme/pallet-rack/heavy/r1
kind: spec
category: structure.rack
name: Heavy Pallet Rack
manufacturer:
  name: ACME Racking
distribution: public
mechanical:
  footprint_mm: [2680, 1100]
  height_mm: 3000
  mass_kg: 137.66
  mount: floor
configuration:
  generator: pallet_rack
  params:
    width_mm:
      values: [2500]
      default: 2500
    depth_mm:
      values: [1100]
      default: 1100
    height_mm:
      values: [2500, 3000, 4000]
      default: 3000
    levels:
      values: [2, 3]
      default: 2
  components:
    - role: unit
      category: structure.rack
      dimensions_mm: {upright: 90, upright_depth: 70, beam: 100}
      variants:
        - {height_mm: 2500, levels: 2, part_number: PR-25-2, kg: 129.2}
        - {height_mm: 3000, levels: 2, part_number: PR-30-2, kg: 137.66}
        - {height_mm: 4000, levels: 3, part_number: PR-40-3, kg: 196}
    - role: extension
      category: structure.rack
      variants:
        - {height_mm: 2500, levels: 2, part_number: PR-25-2B, kg: 99.1}
        - {height_mm: 3000, levels: 2, part_number: PR-30-2B, kg: 103.51}
        - {height_mm: 4000, levels: 3, part_number: PR-40-3B, kg: 150}
  rules:
    beam_pitch_mm: 50
    pallets_per_bay: 2
"""


def _pack(tmp_path: Path, text: str, slug: str) -> Path:
    directory = tmp_path / slug
    directory.mkdir()
    (directory / "manifest.yaml").write_text(text, encoding="utf-8")
    return directory


def _rows(scene: bt.Scene) -> dict[str, dict]:
    """BOM rows keyed by the resident they cover, attributes flattened in."""
    return {row["names"][0]: {**row, **(row.get("attributes") or {})} for row in scene.bom().rows}


# ------------------------------------------------------------ pallet stand


def test_pallet_stand_seats_the_pallet_and_leaves_the_path_open() -> None:
    scene = bt.Scene()
    built = bt.parts.pallet_stand(scene, "stand", (2.0, 1.0), yaw=math.pi / 2)
    assert set(built.frames) == {"stand/pallet", "stand/entry"}
    seat, _ = scene.frame("stand/pallet")
    assert seat == pytest.approx((2.0, 1.0, 0.348))
    # The open end faces local +X, rotated to +Y here; the entry frame is on
    # the floor there, facing back into the stand.
    entry, q = scene.frame("stand/entry")
    assert entry == pytest.approx((2.0, 1.65, 0.0), abs=1e-9)
    assert math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)) == pytest.approx(-math.pi / 2)
    # Nothing in the inner gap: a body 0.91 wide passes between the rails.
    gap = 1.0
    for name in built.obstacles:
        lo, hi = scene.obstacle_bounds(name)
        assert hi[0] <= 2.0 - gap / 2 + 1e-9 or lo[0] >= 2.0 + gap / 2 - 1e-9, name
    # Rails carry the pallet at the support height, stops rise above it.
    _, top = scene.obstacle_bounds("stand/rail_l")
    assert top[2] == pytest.approx(0.348)
    _, stop = scene.obstacle_bounds("stand/stop_l")
    assert stop[2] == pytest.approx(0.389)
    row = _rows(scene)["stand"]
    assert row["category"] == "structure.pallet_stand" and row["qty"] == 1


def test_pallet_stand_from_a_pack_records_the_article(tmp_path: Path) -> None:
    scene = bt.Scene()
    built = bt.parts.pallet_stand(scene, "stand", (0.0, 0.0), catalog=_pack(tmp_path, STAND_MANIFEST, "stand"))
    assert len(built.obstacles) == 8   # no trim in the pack: the boxes are the picture
    row = _rows(scene)["stand"]
    assert row["model"] == "PS-EU" and row["catalog"] == "acme/pallet-rack/eu/r1"
    assert row["pallet_mm"] == "1200x800" and row["support_mm"] == "348"
    with pytest.raises(ValueError, match="pallet_stand"):
        bt.parts.rack(scene, "wrong", catalog=_pack(tmp_path, STAND_MANIFEST, "wrong"))
    with pytest.raises(ValueError, match="not available"):
        bt.parts.pallet_stand(scene, "us", (3.0, 0.0), catalog=tmp_path / "stand", pallet="us")


# ------------------------------------------------------- charging station


def test_charging_station_docks_at_the_plate_edge(tmp_path: Path) -> None:
    scene = bt.Scene()
    built = bt.parts.charging_station(scene, "charger", (10.0, 2.4), yaw=math.pi / 2,
                                      catalog=_pack(tmp_path, CHARGER_MANIFEST, "charger"))
    dock, _ = scene.frame("charger/dock")
    assert dock == pytest.approx((10.0, 2.4 + 0.237 / 2 + 0.25, 0.0), abs=1e-9)
    assert set(built.obstacles) == {"charger/housing", "charger/plate"}
    _, top = scene.obstacle_bounds("charger/plate")
    assert top[2] < 0.025   # under a 25 mm ground clearance
    row = _rows(scene)["charger"]
    assert row["model"] == "DC-48" and row["mass_kg"] == 20
    assert row["supply_vac"] == "240" and row["charging_current_a"] == "40"
    plain = bt.Scene()
    bt.parts.charging_station(plain, "c", (0.0, 0.0), model="Charge 48V", manufacturer="ACME")
    assert _rows(plain)["c"]["category"] == "vehicle.charger"


# ------------------------------------------------------------- pallet rack


def test_pallet_rack_frames_every_seat_and_spaces_the_bays() -> None:
    scene = bt.Scene()
    built = bt.parts.pallet_rack(scene, "row", (0.0, 0.0), bays=4, width=2.5, depth=1.1,
                                 height=4.0, levels=3, yaw=math.pi / 2)
    assert len(built.frames) == 4 * 4
    # Bays pitch by the clear width plus one upright, along +Y here.
    y0 = scene.frame("row/bay0/level0")[0][1]
    y1 = scene.frame("row/bay1/level0")[0][1]
    assert y1 - y0 == pytest.approx(2.5 + 0.09)
    assert scene.frame("row/bay0/level0")[0][2] == pytest.approx(0.0)
    assert scene.frame("row/bay3/level3")[0][2] == pytest.approx(3.9)
    # Posts: five frames of two, all 4 m; beams: two per level per bay.
    posts = [o for o in built.obstacles if "/post_" in o]
    beams = [o for o in built.obstacles if "/beam" in o]
    assert len(posts) == 10 and len(beams) == 4 * 3 * 2
    lo, hi = scene.obstacle_bounds(posts[0])
    assert hi[2] - lo[2] == pytest.approx(4.0)
    lo, hi = scene.obstacle_bounds("row/unit/beam1f")
    assert hi[2] == pytest.approx(1.3) and hi[2] - lo[2] == pytest.approx(0.10)
    row = _rows(scene)["row"]
    assert row["category"] == "structure.rack" and row["bays"] == "4"


def test_pallet_rack_from_a_pack_is_one_unit_plus_extensions(tmp_path: Path) -> None:
    pack = _pack(tmp_path, RACK_MANIFEST, "rack")
    scene = bt.Scene()
    bt.parts.pallet_rack(scene, "row", (0.0, 0.0), bays=4, height=4.0, levels=3, catalog=pack)
    rows = _rows(scene)
    unit, ext = rows["row/unit"], rows["row/ext"]
    assert unit["model"] == "PR-40-3" and unit["qty"] == 1 and unit["mass_kg"] == 196
    assert ext["model"] == "PR-40-3B" and ext["qty"] == 3 and ext["mass_kg"] == 150
    assert unit["beam_tops_mm"] == "1300/2600/3900"
    # Beam tops snap to the maker's pitch, and a level count the maker does
    # not sell at that height is refused with the table.
    scene2 = bt.Scene()
    bt.parts.pallet_rack(scene2, "row", (0.0, 0.0), catalog=pack, beam_heights=[1.33, 2.51])
    assert _rows(scene2)["row/unit"]["beam_tops_mm"] == "1350/2500"
    with pytest.raises(ValueError, match="not available"):
        bt.parts.pallet_rack(scene2, "tall", (5.0, 0.0), catalog=pack, height=3.5)
    with pytest.raises(ValueError, match="sold"):
        bt.parts.pallet_rack(scene2, "three", (5.0, 0.0), catalog=pack, height=3.0, levels=3)
    single = bt.Scene()
    bt.parts.pallet_rack(single, "one", (0.0, 0.0), catalog=pack)
    assert "one/ext" not in _rows(single)


def test_pallet_rack_refuses_beams_that_do_not_fit() -> None:
    scene = bt.Scene()
    with pytest.raises(ValueError, match="does not fit"):
        bt.parts.pallet_rack(scene, "row", (0.0, 0.0), width=2.5, depth=1.1, height=3.0,
                             levels=2, beam_heights=[1.5, 3.2])
    with pytest.raises(ValueError, match="beam heights given"):
        bt.parts.pallet_rack(scene, "row", (0.0, 0.0), width=2.5, depth=1.1, height=3.0,
                             levels=2, beam_heights=[1.5])

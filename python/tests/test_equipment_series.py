"""Dimension/decoration contract for series-specific equipment packs, offline."""

import json

import botrail as bt
import pytest


def scene():
    return bt.Scene(bt.Robot.from_urdf_string('<robot name="anchor"><link name="root"/></robot>'))


def pack(tmp_path, generator, dims=None, rules=None):
    roles = {"fence": ["panel", "post"], "conveyor": ["unit", "stand"], "cabinet": ["body"]}[
        generator
    ]
    components = [
        {"role": role, "part_number": role, "dimensions_mm": (dims or {}).get(role, {})}
        for role in roles
    ]
    if generator == "fence":
        components[0]["widths_mm"] = [1000]
    params = (
        {"height_mm": {"values": [2200], "default": 2200}}
        if generator == "fence"
        else {
            "width_mm": {
                "values": [300 if generator == "conveyor" else 800],
                "default": 300 if generator == "conveyor" else 800,
            },
            "height_mm": {
                "values": [750 if generator == "conveyor" else 2100],
                "default": 750 if generator == "conveyor" else 2100,
            },
            ("length_mm" if generator == "conveyor" else "depth_mm"): {
                "values": [2000, 8000] if generator == "conveyor" else [600],
                "default": 2000 if generator == "conveyor" else 600,
            },
        }
    )
    root = tmp_path / generator
    root.mkdir(exist_ok=True)
    (root / "manifest.yaml").write_text(
        json.dumps(
            {
                "id": f"test/{generator}/series/r2",
                "kind": "spec",
                "configuration": {
                    "generator": generator,
                    "params": params,
                    "components": components,
                    "rules": rules or {},
                },
            }
        )
    )
    return root


@pytest.mark.parametrize("gap", [0, 20, 100])
@pytest.mark.parametrize("detail", ["plain", "full"])
def test_fence_panel_gap_and_all_posts_share_the_correct_top(tmp_path, gap, detail):
    p = pack(tmp_path, "fence", {"post": {"section_w": 50, "section_d": 50}}, {"floor_gap_mm": gap})
    s = scene()
    bt.parts.fence(s, "f", [(0, 0), (2.1, 0)], closed=False, catalog=p, detail=detail)
    for name in s.obstacle_names:
        lo, hi = s.obstacle_bounds(name)
        if "/posts/" in name:
            assert lo[2] == pytest.approx(0)
            assert hi[2] == pytest.approx(2.2 + gap / 1000)
        elif "/panels/" in name:
            assert lo[2] == pytest.approx(gap / 1000)
            assert hi[2] == pytest.approx(2.2 + gap / 1000)
        elif name.endswith("/frame_b"):
            assert lo[2] == pytest.approx(gap / 1000)


@pytest.mark.parametrize("gap", [-1, float("nan"), float("inf")])
def test_fence_rejects_invalid_floor_gap(tmp_path, gap):
    p = pack(tmp_path, "fence", rules={"floor_gap_mm": gap})
    with pytest.raises(ValueError, match="floor_gap_mm"):
        bt.parts.fence(scene(), "f", [(0, 0), (2, 0)], catalog=p)


def test_trim_parameters_preserve_strings_and_explicit_metre_arguments():
    class FakeSpec:
        def trim(self, role):
            return "part.xacro"

    class FakeScene:
        frames = ()

        def load_urdf(self, path, **kwargs):
            self.args = kwargs["args"]
            return []

    s = FakeScene()
    bt.parts._load_trim(
        s,
        bt.parts.Built("f"),
        FakeSpec(),
        "post",
        "trim",
        (0, 0, 0),
        parameters={"post_finish": "zinc-yellow", "height": 2200},
        height=2.3,
    )
    assert s.args == {"post_finish": "zinc-yellow", "height": "2.3"}


DRIVE = {
    "unit": {
        "belt_thickness": 34,
        "rail": 20,
        "rail_rise": 0,
        "drive_length": 300,
        "drive_drop": 135,
        "drive_overhang": 120,
        "mid_tension_after_length": 6000,
        "mid_tension_length": 250,
        "mid_tension_drop": 100,
    },
    "stand": {"leg": 40, "inset": 150},
}


@pytest.mark.parametrize("detail", ["plain", "full"])
def test_drive_envelope_flat_rail_and_inset_are_independent_of_detail(tmp_path, detail):
    p = pack(tmp_path, "conveyor", DRIVE)
    s = scene()
    bt.parts.conveyor(s, "c", position=(0, 0), catalog=p, detail=detail)
    lo, hi = s.obstacle_bounds("c/drive")
    assert lo == pytest.approx((-0.15, -0.29, 0.581))
    assert hi == pytest.approx((0.15, 0.17, 0.716))
    assert s.obstacle_bounds("c/rail_l")[1][2] == pytest.approx(0.75)
    assert s.obstacle_bounds("c/stands/s0_l")[0][0] == pytest.approx(-0.87)
    assert "c/mid_tension" not in s.obstacle_names


def test_long_conveyor_has_tension_envelope_in_both_detail_modes(tmp_path):
    p = pack(tmp_path, "conveyor", DRIVE)
    bounds = []
    for mode in ("plain", "full"):
        s = scene()
        bt.parts.conveyor(s, "c", position=(0, 0), catalog=p, length_mm=8000, detail=mode)
        bounds.append(s.obstacle_bounds("c/mid_tension"))
    assert bounds[0] == bounds[1]
    assert bounds[0][0] == pytest.approx((1.875, -0.17, 0.616))


def test_old_conveyor_pack_does_not_gain_a_drive_envelope(tmp_path):
    p = pack(tmp_path, "conveyor")
    s = scene()
    bt.parts.conveyor(s, "c", position=(0, 0), catalog=p, detail="plain")
    assert "c/drive" not in s.obstacle_names
    assert s.obstacle_bounds("c/rail_l")[1][2] == pytest.approx(0.79)


@pytest.mark.parametrize("detail", ["plain", "full"])
def test_cabinet_top_clearance_excludes_body_dimension_and_bom(tmp_path, detail):
    p = pack(tmp_path, "cabinet", {"body": {"lifting_eye_height": 52}})
    s = scene()
    bt.parts.cabinet(s, "cab", position=(1, 2, 0.3), catalog=p, detail=detail)
    assert s.obstacle_bounds("cab/body")[1][2] == pytest.approx(2.4)
    assert s.obstacle_bounds("cab/lifting_clearance")[1][2] == pytest.approx(2.452)
    assert len([r for r in s.bom().rows if r["category"] != "robot"]) == 1


def test_partial_drive_spec_is_rejected(tmp_path):
    p = pack(tmp_path, "conveyor", {"unit": {"drive_length": 300}})
    with pytest.raises(ValueError, match="drive_length, drive_drop"):
        bt.parts.conveyor(scene(), "c", position=(0, 0), catalog=p)

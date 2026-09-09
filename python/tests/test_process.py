"""Declared process data, physical load math, and missing-input regressions."""

import json
import math
from pathlib import Path

import botrail as bt
import pytest
from botrail import _cli

ROBOT = """<robot name="load_test">
<link name="base"/><link name="flange"/><link name="moving"/><link name="tcp"/>
<joint name="wrist" type="revolute"><parent link="base"/><child link="flange"/>
<axis xyz="0 1 0"/><limit lower="-3.14" upper="3.14" effort="100" velocity="1"/></joint>
<joint name="jaw" type="revolute"><parent link="flange"/><child link="moving"/>
<origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="0" upper="3.14" effort="100" velocity="1"/></joint>
<joint name="tip" type="fixed"><parent link="flange"/><child link="tcp"/><origin xyz="0 0 .3"/></joint>
</robot>"""


def cell():
    s = bt.Scene(bt.Robot.from_urdf_string(ROBOT))
    s.set_joint_positions([0, math.pi / 2])
    s.add_box("stock", (0.1, 0.1, 0.01), (3, 0, 0))
    s.set_part(
        "stock",
        model="Coupon",
        category="workpiece",
        attributes={"material": "steel", "lot": "42"},
    )
    p = json.loads(
        (Path(__file__).parents[2] / "examples/process/ati_rcv250.json").read_text()
    )
    p.update(
        robot="load_test",
        flange="flange",
        tool_frame="flange",
        tcp="tcp",
        tcp_offset_m=[0, 0, 0.3],
        tcp_axis=[0, 0, 1],
        load_components_complete=True,
        load_components=[
            {
                "name": "fixed",
                "frame": "flange",
                "mass_kg": 2,
                "com_m": [0.1, 0, 0],
                "inertia_kg_m2": [0.02, 0.03, 0.04, 0, 0, 0],
            },
            {
                "name": "moving",
                "frame": "moving",
                "mass_kg": 3,
                "com_m": [0.2, 0, 0],
                "inertia_kg_m2": [0.2, 0.3, 0.4, 0, 0, 0],
            },
        ],
    )
    bt.process.configure(s, "stock", p)
    return s, p


def status(s, key):
    return next(
        c["status"] for c in bt.process.report(s, "stock").checks if c["id"] == key
    )


def test_rotated_tensor_parallel_axis_and_gravity_follow_live_pose():
    s, _p = cell()
    a = bt.process.report(s, "stock").load
    assert a["mass_kg"] == 5
    assert a["com_m"] == pytest.approx([0.64, 0.12, 0])
    for row, expected in zip(
        a["inertia_at_flange_kg_m2"], [[0.44, -0.6, 0], [-0.6, 3.25, 0], [0, 0, 3.58]]
    ):
        assert row == pytest.approx(expected)
    assert a["gravity_moment_nm"] == pytest.approx([-5.88399, 31.38128, 0])
    s.set_joint_positions([math.pi / 2, math.pi / 2])
    b = bt.process.report(s, "stock").load
    assert b["com_m"] == pytest.approx(a["com_m"])
    assert b["gravity_moment_nm"] == pytest.approx([0, 0, -5.88399], abs=1e-8)
    s.set_joint_positions([0, 0])
    assert bt.process.report(s, "stock").load["com_m"] == pytest.approx([0.76, 0, 0])


def test_missing_parts_never_become_zero_and_known_overload_still_fails():
    s, p = cell()
    p["load_components"][1]["mass_kg"] = None
    bt.process.configure(s, "stock", p)
    r = bt.process.report(s, "stock")
    assert r.load["mass_kg"] is None and r.load["known_subtotal_kg"] == 2
    assert r.load["com_m"] is None and r.load["inertia_at_flange_kg_m2"] is None
    assert status(s, "payload_kg") == "unknown"
    p["facts"]["robot_payload_kg"]["value"] = 1
    bt.process.configure(s, "stock", p)
    assert status(s, "payload_kg") == "fail"
    p["facts"]["robot_payload_kg"]["value"] = None
    bt.process.configure(s, "stock", p)
    assert status(s, "payload_kg") == "unknown"


@pytest.mark.parametrize("field", ["com_m", "inertia_kg_m2"])
def test_missing_component_measurements_invalidate_aggregate(field):
    s, p = cell()
    p["load_components"][1][field] = None
    bt.process.configure(s, "stock", p)
    r = bt.process.report(s, "stock")
    assert r.load["mass_kg"] == 5
    assert r.load["inertia_at_flange_kg_m2"] is None
    if field == "com_m":
        assert r.load["com_m"] is None


@pytest.mark.parametrize(
    "tensor", [[1, 1, 3, 0, 0, 0], [1, 1, 1, 2, 0, 0], [-1, 1, 1, 0, 0, 0]]
)
def test_impossible_inertia_is_rejected(tensor):
    s, p = cell()
    p["load_components"][0]["inertia_kg_m2"] = tensor
    with pytest.raises(ValueError, match="physical tensor"):
        bt.process.configure(s, "stock", p)


def test_bad_tcp_and_pressure_feed_into_scene_check():
    s, p = cell()
    p["tcp_offset_m"][2] += 0.01
    p["tcp_axis"] = [1, 0, 0]
    p["settings"]["motor_pressure_bar"] = 4
    p["settings"]["collet_mm"] = 6  # 6 mm is not the 1/4-inch bit's shank.
    bt.process.configure(s, "stock", p)
    assert all(
        status(s, k) == "fail"
        for k in ("tcp_geometry", "tcp_axis", "motor_pressure_bar", "collet_shank_mm")
    )
    result = s.check()
    assert not result.ok
    assert {f.code for f in result.findings if f.severity == "error"} >= {
        "process_tcp_geometry",
        "process_tcp_axis",
        "process_motor_pressure_bar",
        "process_collet_shank_mm",
    }


def test_missing_limit_and_dangling_unknown_mass_frame_are_not_passes():
    s, p = cell()
    for key in ("bur_diameter_mm", "robot_payload_kg", "compliance_pressure_max_bar"):
        p["facts"][key]["value"] = None
    bt.process.configure(s, "stock", p)
    assert all(
        status(s, k) == "unknown"
        for k in ("payload_kg", "radial_engagement_mm", "compliance_pressure_bar")
    )
    p["load_components"][1].update(mass_kg=None, frame="removed_jaw")
    bt.process.configure(s, "stock", p)
    assert status(s, "load_frames") == "fail"


def test_air_flow_basis_rating_and_qualification_remain_explicit():
    s, p = cell()
    assert status(s, "motor_air_flow") == status(s, "bur_speed_rating_rpm") == "unknown"
    p["facts"]["motor_flow_reference"]["value"] = "test reference"
    p["settings"].update(
        motor_flow_reference="test reference", motor_capacity_l_min=800
    )
    p["facts"]["bur_rated_rpm"]["value"] = 30000
    bt.process.configure(s, "stock", p)
    assert status(s, "motor_air_flow") == status(s, "bur_speed_rating_rpm") == "fail"
    assert not bt.process.report(s, "stock").ready


def test_zero_feed_is_not_a_positive_value_within_tolerance():
    s, p = cell()
    p["settings"]["feed_mps"] = 0
    bt.process.configure(s, "stock", p)
    assert status(s, "feed_mps") == "fail"


def test_setup_survives_project_and_python_preserving_part_identity(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(bt, "studio", lambda *args, **kwargs: None)
    s, p = cell()
    before = bt.process.report(s, "stock").to_dict()
    assert s.part("stock")["attributes"]["lot"] == "42"
    assert s.part("stock")["model"] == "Coupon"
    saved = bt.process.setup(s, "stock")
    saved["settings"]["motor_pressure_bar"] = 99
    assert bt.process.setup(s, "stock") == p
    s.save_project(tmp_path / "cell.botrail")
    (tmp_path / "cell.py").write_text(s.generate_python())
    for other in (
        bt.Scene.load_project(tmp_path / "cell.botrail"),
        _cli.load_cell(str(tmp_path / "cell.py")),
    ):
        assert bt.process.targets(other) == ["stock"]
        assert bt.process.report(other, "stock").to_dict() == before


def test_invalid_saved_setup_is_a_check_error():
    s, _ = cell()
    s.set_part(
        "stock", model="Coupon", attributes={"botrail_process_setup_v1": "not JSON"}
    )
    assert any(
        f.code == "process_setup" and f.severity == "error" for f in s.check().findings
    )


def test_spot_weld_process_checks_do_not_infer_a_wps():
    s, p = cell()
    p.update(operation="spot_weld")
    p["facts"].update(
        electrode_force_max_n={"value": 5500, "source": "test", "basis": "fixture"},
        transformer_reference_coolant_l_min={
            "value": 6,
            "source": "test",
            "basis": "series_reference",
        },
    )
    p["settings"] = {
        "electrode_force_n": 6000,
        "weld_current_ka": None,
        "weld_time_s": 0.2,
        "hold_time_s": 0.2,
        "coolant_l_min": 3,
    }
    bt.process.configure(s, "stock", p)
    assert status(s, "electrode_force_n") == status(s, "coolant_l_min") == "fail"
    assert (
        status(s, "weld_current_ka")
        == status(s, "whole_gun_cooling")
        == status(s, "process_qualification")
        == "unknown"
    )


def test_bur_tip_axis_and_separate_contact_envelope():
    r = bt.tools.rotary_bur(
        diameter=0.009525,
        cutting_length=0.015875,
        shank_diameter=0.00635,
        exposed_shank=0.010,
    )
    s = bt.Scene(r)
    p, q = s.link_pose("tcp")
    assert p == pytest.approx([0, 0, 0.025875])
    assert bt.parts._rotate(q, (0, 0, 1)) == pytest.approx([0, 0, -1], abs=1e-9)
    s.add_box("stock", (0.005, 0.005, 0.005), (0, 0, 0.020))
    assert s.check_collisions()
    s.allow_link_obstacle_contact("cutter", "stock")
    assert not s.check_collisions()
    s.add_box("clamp", (0.005, 0.005, 0.005), (0, 0, 0.005))
    assert any(("link", "shank") in pair for pair in s.check_collisions())
    with pytest.raises(ValueError):
        bt.tools.rotary_bur(
            diameter=float("nan"),
            cutting_length=0.01,
            shank_diameter=0.006,
            exposed_shank=0.01,
        )

"""Drawing-level fit: synthetic tolerances, never hardware qualification."""

# Generated Python replay is part of the persistence contract.
# ruff: noqa: S102

import copy
import json
import math

import botrail as bt
import pytest
from test_mounting import ADAPTER, ARM, EVIDENCE, IDENTITY, SOURCE, TOOL, face, load
from test_mounting import catalog as catalog_fixture
from test_vehicle_mounting import CARRIER, cell

catalog = catalog_fixture


def bounds(low, high=None):
    return {"min": low, "max": low if high is None else high}


def drawing(frame, role):
    mount = role == "mount"
    holes = []
    for i, xy in enumerate([(-20, -20), (20, -20), (20, 20), (-20, 20)]):
        h = {"id": ("c" if mount else "t") + str(i), "position_mm": xy,
             "kind": "clearance" if mount else "threaded", "position_tolerance_mm": 0.05}
        if mount:
            h.update(diameter_mm=bounds(6.6, 6.8), grip_mm=bounds(5))
        else:
            h.update(thread={"diameter_mm": 6, "pitch_mm": 1}, depth_mm=bounds(12),
                     thread_start_mm=bounds(1), fastener_rules={
                         "head_standard": "ISO 4762", "property_class": "8.8",
                         "min_engagement_mm": 6, "torque_nm": bounds(7, 9),
                     })
        holes.append(h)
    out = face(frame, role, "fixture-interface", requirements_complete=True,
               geometry={"normal_z": -1 if mount else 1, "frame_verified": True,
                         "complete": True, "holes": holes, "evidence": EVIDENCE,
                         "locators": [{"id": "socket" if mount else "pin",
                                       "kind": "hole" if mount else "pin", "position_mm": [0, 0],
                                       "position_tolerance_mm": 0.05,
                                       "diameter_mm": bounds(6.3, 6.4) if mount else bounds(5.9, 6),
                                       "depth_mm": bounds(4 if mount else 3)}]},
               clearance={"frame_verified": True, "complete": True, "evidence": EVIDENCE,
                          "solids": [{"id": "body", "center_mm": [0, 0, 10 if mount else -10], "size_mm": [60, 60, 20]}],
                          "access": [{"id": "driver", "center_mm": [80, 0, 20], "size_mm": [5, 5, 10]}]})
    if mount:
        out["allowed_poses"] = [IDENTITY]
        out["fasteners"] = [{"holes": [h["id"] for h in holes], "thread": {"diameter_mm": 6, "pitch_mm": 1},
                             "head_standard": "ISO 4762", "property_class": "8.8", "length_mm": bounds(14),
                             "washer_mm": bounds(1), "torque_nm": bounds(8), "evidence": EVIDENCE}]
    return copy.deepcopy(out)


@pytest.fixture
def faces():
    return drawing("left", "flange"), drawing("tool_root", "mount")


def build(catalog, faces, **offset):
    a, b = faces
    catalog(ARM, category="manipulator", mount="base", flange="left", mounting={"interfaces": [a]})
    catalog(TOOL, mount="tool_root", mounting={"interfaces": [b]})
    return bt.Scene(load(ARM).attach_tool(load(TOOL), prefix="tool_", **offset))


def checks(scene):
    return {i.key: i for i in bt.mounting.report(scene).items}


def test_fully_evidenced_planar_joint_passes_with_unordered_different_hole_names(catalog, faces):
    faces[1]["geometry"]["holes"].reverse()
    scene = build(catalog, faces)
    report = bt.mounting.report(scene)
    assert report.ready
    assert {i.key for i in report.items} == {"requirements_complete", "interface", "declared_pose", "geometry", "fasteners", "clearance"}
    assert all(i.status == "pass" for i in report.items)
    assert checks(scene)["fasteners"].evidence["checks"][0]["values"]["checks"][-1]["values"]["penetration_mm"] == bounds(8)


@pytest.mark.parametrize("field,value,status", [("interface_id", "different-interface", "fail"), ("interface_id", None, "unknown"), ("evidence", [], "unknown")])
def test_bare_interface_declaration_is_separate_from_dimensional_fit(catalog, faces, field, value, status):
    faces[1][field] = value
    result = checks(build(catalog, faces))
    assert result["interface"].status == status
    assert result["geometry"].status == "pass"


@pytest.mark.parametrize("field,value,status", [
    ("length_mm", bounds(20), "fail"),  # bottoming
    ("length_mm", bounds(10), "fail"),  # insufficient engagement
    ("length_mm", bounds(10, 14), "unknown"),
    ("length_mm", bounds(14, 19), "unknown"),
    ("washer_mm", bounds(5), "fail"),
    ("torque_nm", bounds(10), "fail"),
    ("torque_nm", bounds(8, 10), "unknown"),
    ("torque_nm", bounds(7, 9), "pass"),
    ("head_standard", "different-head", "fail"),
    ("property_class", "10.9", "fail"),
    ("thread", {"diameter_mm": 6, "pitch_mm": 0.75}, "fail"),
    ("thread", {"diameter_mm": 5, "pitch_mm": 1}, "fail"),
    ("thread", {"diameter_mm": 6, "pitch_mm": 1, "left_hand": True}, "fail"),
    ("thread", {"diameter_mm": 6}, "unknown"),
    ("evidence", [], "unknown"),
])
def test_selected_fastener_is_compared_as_bounds(catalog, faces, field, value, status):
    faces[1]["fasteners"][0][field] = value
    result = checks(build(catalog, faces))
    assert result["fasteners"].status == status
    assert result["geometry"].status == "pass"


@pytest.mark.parametrize("side,field,value,key,status", [
    (0, "position_mm", [-17, -20], "geometry", "fail"),
    (1, "diameter_mm", bounds(5), "geometry", "fail"),
    (1, "diameter_mm", bounds(6.1, 6.8), "geometry", "unknown"),
    (1, "position_tolerance_mm", None, "geometry", "unknown"),
    (0, "depth_mm", None, "fasteners", "unknown"),
    (0, "depth_mm", bounds(7, 9), "fasteners", "unknown"),
    (1, "grip_mm", None, "fasteners", "unknown"),
    (0, "thread_start_mm", bounds(3), "fasteners", "fail"),
    (0, "thread_start_mm", bounds(1, 3), "fasteners", "unknown"),
    (0, "thread_start_mm", None, "fasteners", "pass"),
    (0, "fastener_rules", None, "fasteners", "unknown"),
])
def test_hole_geometry_and_thread_depth_preserve_unknowns(catalog, faces, side, field, value, key, status):
    faces[side]["geometry"]["holes"][0][field] = value
    assert checks(build(catalog, faces))[key].status == status


@pytest.mark.parametrize("field,value,status", [
    ("diameter_mm", bounds(5), "fail"),
    ("diameter_mm", bounds(6.1, 6.4), "unknown"),
    ("depth_mm", bounds(2), "fail"),
    ("depth_mm", bounds(2, 4), "unknown"),
    ("position_tolerance_mm", None, "unknown"),
])
def test_locator_fit_includes_depth_and_positional_tolerances(catalog, faces, field, value, status):
    faces[1]["geometry"]["locators"][0][field] = value
    assert checks(build(catalog, faces))["geometry"].status == status


@pytest.mark.parametrize("field,value", [("frame_verified", False), ("complete", False), ("evidence", [])])
def test_partial_drawing_cannot_pass(catalog, faces, field, value):
    faces[0]["geometry"][field] = value
    result = checks(build(catalog, faces))
    assert result["geometry"].status == "unknown"
    assert result["fasteners"].status == "unknown"


def test_missing_and_duplicate_candidates_do_not_get_matched_twice(catalog, faces):
    faces[1]["geometry"]["holes"][0]["position_mm"] = [20, -20]
    assert checks(build(catalog, faces))["geometry"].status == "fail"
    faces[0]["geometry"]["holes"][0]["position_mm"] = [20, -20]
    assert checks(build(catalog, faces))["geometry"].status == "unknown"


def test_rotation_uses_declared_pose_and_transformed_feature_locations(catalog, faces):
    q = [0, 0, math.sqrt(0.5), math.sqrt(0.5)]
    faces[1]["allowed_poses"].append({**IDENTITY, "quaternion": q})
    assert bt.mounting.report(build(catalog, faces, offset_quaternion=q)).ready
    moved = checks(build(catalog, faces, offset_position=(0, 0, 0.001)))
    assert moved["geometry"].status == "fail"
    assert moved["declared_pose"].status == "fail"
    assert moved["fasteners"].status == "unknown"


def test_reversed_plane_normal_does_not_pass(catalog, faces):
    faces[1]["geometry"]["normal_z"] = 1
    assert checks(build(catalog, faces))["geometry"].status == "fail"


def test_robot_mount_arm_edges_receive_the_same_review(catalog, faces):
    build(catalog, faces)
    model = load(ARM).mount(load(TOOL), "left", prefix="mounted_", group="mounted")
    report = bt.mounting.report(model)
    assert report.ready
    assert all(i.target == "robot/mounted" for i in report.items)
    assert len(report.assemblies) == 1


@pytest.mark.parametrize("allowed,status", [(bounds(13, 15), "pass"), (bounds(15, 18), "fail"), (bounds(13, 14), "unknown")])
def test_documented_screw_length_rule_is_applied(catalog, faces, allowed, status):
    faces[1]["fasteners"][0]["length_mm"] = bounds(13.5, 14.5)
    faces[0]["geometry"]["holes"][0]["fastener_rules"]["length_mm"] = allowed
    assert checks(build(catalog, faces))["fasteners"].status == status


def test_omitting_a_selected_screw_is_unknown(catalog, faces):
    faces[1]["fasteners"][0]["holes"].pop()
    assert checks(build(catalog, faces))["fasteners"].status == "unknown"


def test_clearance_uses_rotated_boxes_not_axis_aligned_bounds(catalog, faces):
    faces[0]["clearance"]["access"][0].update(center_mm=[0, 80, 10], size_mm=[5, 5, 10])
    faces[1]["clearance"]["solids"][0].update(center_mm=[80, 0, 10], size_mm=[5, 5, 10])
    assert checks(build(catalog, faces))["clearance"].status == "pass"
    q = [0, 0, math.sqrt(0.5), math.sqrt(0.5)]
    assert checks(build(catalog, faces, offset_quaternion=q))["clearance"].status == "fail"


@pytest.mark.parametrize("kind,side,center,status", [
    ("solids", 1, [0, 0, 9], "fail"),
    ("access", 0, [0, 0, 10], "fail"),
    ("access", 1, [0, 0, -10], "fail"),
    ("access", 0, [80, 0, 20], "pass"),
])
def test_body_and_bidirectional_tool_access_envelopes(catalog, faces, kind, side, center, status):
    faces[side]["clearance"][kind][0]["center_mm"] = center
    assert checks(build(catalog, faces))["clearance"].status == status


@pytest.mark.parametrize("field,value", [("complete", False), ("frame_verified", False), ("evidence", [])])
def test_clearance_without_coverage_and_evidence_is_unknown(catalog, faces, field, value):
    faces[0]["clearance"][field] = value
    assert checks(build(catalog, faces))["clearance"].status == "unknown"


@pytest.mark.parametrize("mutate", [
    lambda f: f["geometry"].update(normal_z=0),
    lambda f: f["geometry"]["holes"][0].update(position_mm=[float("nan"), 0]),
    lambda f: f["geometry"]["holes"][0].update(diameter_mm=bounds(8, 7)),
    lambda f: f["geometry"]["holes"][0].update(position_tolerance_mm=-1),
    lambda f: f["geometry"]["holes"][0].update(thread={"diameter_mm": 6}),
    lambda f: f["geometry"]["holes"][0].update(thread_start_mm=bounds(1)),
    lambda f: f["geometry"]["locators"][0].update(id="c0"),
    lambda f: f["fasteners"][0].update(holes=["c0", "c0"]),
    lambda f: f["fasteners"][0].update(holes=["missing"]),
    lambda f: f["fasteners"][0].update(evidence=[{"source": 7, "section": "drawing"}]),
    lambda f: f["fasteners"][0].update(length_mm=bounds(0)),
    lambda f: f["clearance"].update(solids=[]),
    lambda f: f["clearance"]["solids"][0].update(size_mm=[1, 0, 1]),
    lambda f: f["geometry"].update(unrecognized_tolerance=0.5),
])
def test_malformed_nested_declarations_are_rejected(catalog, faces, mutate):
    mutate(faces[1])
    with pytest.raises(ValueError):
        build(catalog, faces)


def vehicle(catalog, faces, requirement=None):
    a, b = copy.deepcopy(faces)
    a["frame"], b["frame"] = "deck", "base"
    catalog(CARRIER, category="vehicle.amr", mount="ground", flange="deck", mounting={"interfaces": [a]})
    spec = {"interfaces": [b]}
    order = None
    if requirement:
        spec["requirements"] = [{"id": "support", "frame": "base", "order_requires": 0, "note": "Required support", "evidence": EVIDENCE}]
        order = {"requires": [{"catalog": requirement, "qty": 1}]}
    catalog(ARM, category="manipulator", mount="base", mounting=spec, order=order)
    scene = cell()
    scene.mount_robot("cart", carrier=load(CARRIER))
    return scene


def test_vehicle_arm_uses_same_fit_and_actual_required_part_path(catalog, faces):
    scene = vehicle(catalog, faces, CARRIER)
    report = bt.mounting.report(scene)
    assert report.ready
    assert checks(scene)["required:support"].status == "pass"
    assert report.assemblies[-1]["upstream_parts"] == ["cart"]
    scene = vehicle(catalog, faces, "acme/missing/support/r1")
    scene.set_part("cart", kind="device", catalog="acme/missing/support/r1")
    assert checks(scene)["required:support"].status == "fail"


def test_uncaptured_carrier_assembly_does_not_prove_a_required_part_absent(catalog, faces):
    scene = vehicle(catalog, faces, ADAPTER)
    assembled_carrier = load(CARRIER).attach_tool(load(ADAPTER), prefix="adapter_")
    scene.mount_robot("cart", carrier=assembled_carrier)
    requirement = checks(scene)["required:support"]
    assert requirement.status == "unknown"
    assert not requirement.evidence["path_complete"]


@pytest.mark.parametrize("as_vehicle", [False, True])
def test_detail_and_results_survive_project_snapshot_and_offline_python(catalog, faces, tmp_path, monkeypatch, as_vehicle):
    scene = vehicle(catalog, faces) if as_vehicle else build(catalog, faces)
    before = bt.mounting.report(scene).to_dict()
    path = tmp_path / "fit.botrail"
    scene.save_project(path)
    restored = bt.Scene.load_project(path)
    assert bt.mounting.report(restored).to_dict() == before
    assert bt.mounting.report(scene._snapshot()).to_dict() == before
    # Corrupt later catalog metadata. Replays must retain captured declarations.
    catalog(TOOL, mount="tool_root", sources=[])
    catalog(CARRIER, category="vehicle.amr", mount="ground", flange="deck", sources=[])
    monkeypatch.setattr(bt, "studio", lambda *a, **k: None)
    scope = {}
    exec(restored.generate_python(), scope)
    assert bt.mounting.report(scope["scene"]).to_dict() == before
    import huggingface_hub as hub

    def reject(*args, **kwargs):
        pytest.fail("offline replay must not fetch a catalog")

    monkeypatch.setattr(hub, "dataset_info", reject)
    scope = {}
    exec(restored.generate_python(embed_catalog=True), scope)
    assert bt.mounting.report(scope["scene"]).to_dict() == before


def test_saved_vehicle_details_are_validated(catalog, faces, tmp_path):
    scene = vehicle(catalog, faces)
    path = tmp_path / "bad.botrail"
    scene.save_project(path)
    data = json.loads(path.read_text())
    g = data["robots"][0]["mount"]["reference"]["mounting"]["interfaces"][0]["geometry"]
    g["holes"][0]["thread"]["pitch_mm"] = -1
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="mount reference"):
        bt.Scene.load_project(path)


def test_community_dimensions_cannot_prove_fit(catalog, faces):
    scene = build(catalog, faces)
    assert bt.mounting.report(scene).ready
    catalog(TOOL, mount="tool_root", mounting={"interfaces": [faces[1]]}, sources=[{**SOURCE, "kind": "community"}])
    result = checks(bt.Scene(load(ARM).attach_tool(load(TOOL), prefix="tool_")))
    assert all(i.status == "unknown" for i in result.values())

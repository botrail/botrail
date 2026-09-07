"""Synthetic drawings exercise the bounded mechanical contract, not real hardware."""

# ruff: noqa: S102

import copy
import json

import botrail as bt
import pytest
import yaml
from test_mounting import ADAPTER, ARM, EVIDENCE, IDENTITY, TOOL, face, item, load
from test_mounting import catalog as catalog  # noqa: PLC0414 - pytest fixture re-export


def bounds(lo, hi=None):
    return {"min": lo, "max": lo if hi is None else hi}


def drawing(frame, role):
    receiver = role == "flange"
    holes = []
    for i, (x, y) in enumerate([(-20, -20), (20, -20), (20, 20), (-20, 20)]):
        hole = {"id": f"h{i}", "position_mm": [x, y], "position_tolerance_mm": 0.01,
                "kind": "threaded" if receiver else "clearance"}
        if receiver:
            hole.update(thread={"diameter_mm": 6, "pitch_mm": 1}, depth_mm=bounds(8),
                        fastener_rules={"head_standard": "ISO 4762", "property_class": "8.8",
                                        "torque_nm": bounds(8), "min_engagement_mm": 5})
        else:
            hole.update(diameter_mm=bounds(6.4, 6.5), grip_mm=bounds(5))
        holes.append(hole)
    locator = {"id": "pilot", "kind": "boss" if receiver else "recess",
               "position_mm": [0, 0], "position_tolerance_mm": 0.01,
               "diameter_mm": bounds(9.9, 10) if receiver else bounds(10.2, 10.3),
               "depth_mm": bounds(2) if receiver else bounds(3)}
    return face(frame, role, "fixture-mating-face", requirements_complete=True,
                **({"allowed_poses": [IDENTITY]} if not receiver else {}),
                geometry={"normal_z": 1 if receiver else -1, "frame_verified": True,
                          "complete": True, "holes": holes, "locators": [locator], "evidence": EVIDENCE},
                # Conservative main-body boxes exclude the explicitly matched pilot.
                clearance={"frame_verified": True, "complete": True, "evidence": EVIDENCE,
                           "solids": [{"id": "body", "center_mm": [0, 0, -5 if receiver else 5],
                                       "size_mm": [60, 60, 10]}], "access": []},
                fasteners=[] if receiver else [{"holes": [h["id"] for h in holes],
                    "thread": {"diameter_mm": 6, "pitch_mm": 1}, "head_standard": "ISO 4762",
                    "property_class": "8.8", "length_mm": bounds(12), "washer_mm": bounds(1),
                    "torque_nm": bounds(8), "evidence": EVIDENCE}])


def document(*faces, revision="drawing-A"):
    return {"schema_version": "1", "revision": revision,
            "sources": [{"kind": "user_drawing", "url": "drawing:synthetic-A", "ref": revision}],
            "mounting": {"interfaces": list(faces)}}


@pytest.fixture
def fitted(catalog):
    def build(a=None, b=None, **kwargs):
        catalog(ARM, category="manipulator", mount="base", flange="left",
                mounting={"interfaces": [a or drawing("left", "flange")]})
        catalog(TOOL, mount="tool_root", mounting={"interfaces": [b or drawing("tool_root", "mount")]})
        return load(ARM).attach_tool(load(TOOL), **kwargs)
    return build


def test_complete_declared_joint_and_shuffled_holes_pass(fitted):
    robot = fitted()
    report = bt.mounting.report(robot)
    assert report.ready, report.to_dict()
    assert bt.review(bt.Scene(robot), required=["mounting"]).ready
    b = drawing("tool_root", "mount")
    b["geometry"]["holes"].reverse()
    b["geometry"]["holes"][0]["id"] = "renamed"
    b["fasteners"][0]["holes"][-1] = "renamed"
    assert bt.mounting.report(fitted(b=b)).ready


@pytest.mark.parametrize("change", ["position", "kind", "normal", "pilot_diameter", "pilot_depth", "count"])
def test_geometric_counterexamples_fail(fitted, change):
    a = drawing("left", "flange")
    g = a["geometry"]
    if change == "position":
        g["holes"][0]["position_mm"][0] += 2
    elif change == "kind":
        g["holes"][0].update(kind="clearance", thread=None)
    elif change == "normal":
        g["normal_z"] = -1
    elif change == "pilot_diameter":
        g["locators"][0]["diameter_mm"] = bounds(11)
    elif change == "pilot_depth":
        g["locators"][0]["depth_mm"] = bounds(4)
    else:
        g["holes"].pop()
    report = bt.mounting.report(fitted(a=a))
    assert item(report, "dimensions").status == "fail"
    assert not report.ready
    assert not bt.Scene(fitted(a=a)).check().ok


@pytest.mark.parametrize("key,value,status", [
    ("thread", {"diameter_mm": 6, "pitch_mm": 0.75}, "fail"),
    ("thread", {"diameter_mm": 6, "pitch_mm": 1, "left_hand": True}, "fail"),
    ("head_standard", "ISO 10642", "fail"),
    ("property_class", "4.8", "fail"),
    ("length_mm", bounds(9), "fail"),
    ("length_mm", bounds(16), "fail"),
    ("length_mm", bounds(10, 12), "unknown"),
    ("torque_nm", bounds(9), "fail"),
    ("torque_nm", bounds(7, 9), "unknown"),
    ("holes", ["h0", "h1", "h2"], "unknown"),
    ("evidence", [], "unknown"),
])
def test_selected_screw_conditions(fitted, key, value, status):
    b = drawing("tool_root", "mount")
    b["fasteners"][0][key] = value
    report = bt.mounting.report(fitted(b=b))
    assert item(report, "fasteners").status == status
    assert not report.ready


@pytest.mark.parametrize("start,length,status", [
    (bounds(0), 12, "pass"),
    (bounds(2), 12, "fail"),  # 6 mm penetration, only 4 mm engaged; needs 5
    (bounds(2), 13, "pass"),
    (bounds(2), 15, "fail"),  # sufficient engagement but tip bottoms beyond 8
    (bounds(0, 2), 12, "unknown"),
    (bounds(6), 12, "fail"),  # tip does not reach a usable thread
])
def test_unthreaded_lead_is_excluded_without_moving_the_depth_limit(fitted, start, length, status):
    a, b = drawing("left", "flange"), drawing("tool_root", "mount")
    a["geometry"]["holes"][0]["thread_start_mm"] = start
    b["fasteners"][0]["length_mm"] = bounds(length)
    report = bt.mounting.report(fitted(a, b))
    fasteners = item(report, "fasteners")
    assert fasteners.status == status
    check = next(c for c in fasteners.evidence["checks"] if c["check"] == "h0:engagement")
    assert check["inputs"]["tip_penetration_mm"] == bounds(length - 6)
    assert check["inputs"]["engagement_mm"] == bounds(length - 6 - start["max"], length - 6 - start["min"])


def test_clearance_cannot_declare_thread_lead(fitted):
    b = drawing("tool_root", "mount")
    b["geometry"]["holes"][0]["thread_start_mm"] = bounds(2)
    with pytest.raises(ValueError, match="clearance"):
        fitted(b=b)


ALT = "acme/adapter/alternative/r1"


def declared_alternative(catalog, *, evidence=None, qty=1, primary=ADAPTER):
    catalog(ALT, flange="out", mounting={"interfaces": [
        face("mount", "mount", "robot-face"), face("out", "flange", "tool-face")]})
    return catalog(TOOL, mount="tool_root", mounting={
        "interfaces": [face("tool_root", "mount", "tool-face")],
        "requirements": [{"id": "adapter", "frame": "tool_root", "order_requires": 0,
                          "catalog_alternatives": [{"catalog": ALT, "evidence": EVIDENCE if evidence is None else evidence}],
                          "evidence": EVIDENCE, "note": "Explicit fixture variant"}]},
        order={"requires": [{"catalog": primary, "qty": qty}]})


def test_documented_catalog_alternative_and_primary_both_count(catalog):
    declared_alternative(catalog)
    for pid in (ADAPTER, ALT):
        robot = load(ARM).attach_tool(load(pid), prefix="a_").attach_tool(load(TOOL))
        report = bt.mounting.report(robot)
        assert item(report, "required:adapter").status == "pass"
        assert not report.ready  # this exception does not supply dimension/fastener proofs


def test_catalog_alternative_uses_actual_path_and_counts_each_instance_once(catalog):
    declared_alternative(catalog)
    arm = load(ARM).attach_tool(load(ALT), flange="right", prefix="spare_")
    scene = bt.Scene(arm.attach_tool(load(TOOL), flange="left"))
    scene.add_robot(load(ALT), name="spare")
    assert item(bt.mounting.report(scene), "required:adapter").status == "fail"
    declared_alternative(catalog, qty=2, primary=ALT)
    robot = load(ARM).attach_tool(load(ALT), prefix="a_").attach_tool(load(TOOL))
    assert item(bt.mounting.report(robot), "required:adapter").status == "fail"


@pytest.mark.parametrize("community", [False, True])
def test_present_alternative_with_missing_evidence_is_unknown(catalog, community):
    path = declared_alternative(catalog, evidence=[{"source": 1, "section": "unverified"}] if community else [])
    if community:
        manifest = yaml.safe_load((path / "manifest.yaml").read_text())
        manifest["sources"].append({"kind": "community", "url": "https://example.com/unverified"})
        (path / "manifest.yaml").write_text(yaml.safe_dump(manifest))
    robot = load(ARM).attach_tool(load(ALT), prefix="a_").attach_tool(load(TOOL))
    assert item(bt.mounting.report(robot), "required:adapter").status == "unknown"


def test_alternative_source_indices_rebase_and_replay(catalog, tmp_path):
    declared_alternative(catalog)
    catalog(TOOL, mount="tool_root", mounting={"interfaces": [face("tool_root", "mount", "tool-face")]})
    doc = document(face("tool_root", "mount", "tool-face"))
    doc["mounting"]["requirements"] = [{"id": "variant", "frame": "tool_root",
        "catalog_alternatives": [{"catalog": ALT, "evidence": EVIDENCE}],
        "evidence": EVIDENCE, "note": "Fixture approved alternative"}]
    tool = load(TOOL)._with_mounting_json(json.dumps(doc))
    scene = bt.Scene(load(ARM).attach_tool(load(ALT), prefix="a_").attach_tool(tool))
    before = bt.mounting.report(scene).to_dict()
    requirement = item(bt.mounting.report(scene), "required:document:variant")
    assert requirement.status == "pass"
    assert requirement.evidence["requirement"]["catalog_alternatives"][0]["evidence"][0]["source"] == 1
    project = tmp_path / "alternatives.botrail"
    scene.save_project(project)
    restored = bt.Scene.load_project(project)
    ns = {}
    exec(restored.generate_python().replace("bt.studio(scene)", ""), ns)
    assert bt.mounting.report(restored).to_dict() == before
    assert bt.mounting.report(ns["scene"]).to_dict() == before


@pytest.mark.parametrize("field", ["frame_verified", "complete", "evidence"])
def test_unverified_or_incomplete_geometry_never_passes(fitted, field):
    b = drawing("tool_root", "mount")
    b["geometry"][field] = [] if field == "evidence" else False
    assert item(bt.mounting.report(fitted(b=b)), "dimensions").status == "unknown"


def test_missing_position_tolerance_is_unknown(fitted):
    b = drawing("tool_root", "mount")
    del b["geometry"]["holes"][0]["position_tolerance_mm"]
    assert item(bt.mounting.report(fitted(b=b)), "dimensions").status == "unknown"


def test_prescribed_length_and_missing_pitch(fitted):
    a = drawing("left", "flange")
    a["geometry"]["holes"][0]["fastener_rules"]["length_mm"] = bounds(13)
    report = bt.mounting.report(fitted(a=a))
    assert item(report, "fasteners").status == "fail"
    assert "h0:length" in report.to_markdown()
    a = drawing("left", "flange")
    del a["geometry"]["holes"][0]["thread"]["pitch_mm"]
    assert item(bt.mounting.report(fitted(a=a)), "fasteners").status == "unknown"


def test_fastener_constraints_from_both_drawings_are_combined(fitted):
    b = drawing("tool_root", "mount")
    for hole in b["geometry"]["holes"]:
        hole["fastener_rules"] = {"length_mm": bounds(12)}
    assert bt.mounting.report(fitted(b=b)).ready
    b["geometry"]["holes"][0]["fastener_rules"]["thread"] = {"diameter_mm": 4}
    assert item(bt.mounting.report(fitted(b=b)), "fasteners").status == "fail"


def test_permitted_rotation_is_applied_to_features_and_envelopes(fitted):
    import math

    b = drawing("tool_root", "mount")
    q = [0, 0, math.sqrt(0.5), math.sqrt(0.5)]
    b["allowed_poses"] = [{**IDENTITY, "quaternion": q}]
    assert bt.mounting.report(fitted(b=b, offset_quaternion=q)).ready


@pytest.mark.parametrize("blocked", ["body", "tool_access", "flange_access"])
def test_assembly_envelope_and_access_constraints(fitted, blocked):
    a, b = drawing("left", "flange"), drawing("tool_root", "mount")
    if blocked == "body":
        b["clearance"]["solids"][0]["center_mm"][2] = 4
    elif blocked == "tool_access":
        b["clearance"]["access"] = [{"id": "wrench", "center_mm": [20, 20, -5], "size_mm": [5, 5, 5]}]
    else:
        a["clearance"]["access"] = [{"id": "wrench", "center_mm": [20, 20, 5], "size_mm": [5, 5, 5]}]
    assert item(bt.mounting.report(fitted(a, b)), "assembly_clearance").status == "fail"
    b["clearance"]["frame_verified"] = False
    assert item(bt.mounting.report(fitted(a, b)), "assembly_clearance").status == "unknown"


def test_custom_document_is_immutable_and_survives_project_and_python(tmp_path):
    arm_xml = '<robot name="arm"><link name="base"/><link name="out"/><joint name="j" type="fixed"><parent link="base"/><child link="out"/></joint></robot>'
    tool_xml = '<robot name="bracket"><link name="mount"/></robot>'
    # Both sources are in-memory URDF; no product identity is invented.
    path = tmp_path / "bracket.yaml"
    path.write_text(yaml.safe_dump(document(drawing("mount", "mount"))))
    base = bt.Robot.from_urdf_string(arm_xml)._with_mounting_json(json.dumps(document(drawing("out", "flange"))))
    original = bt.Robot.from_urdf_string(tool_xml)
    tool = original.with_mounting(path)
    robot = base.attach_tool(tool, flange="out", mount="mount")
    scene = bt.Scene(robot)
    before = bt.mounting.report(scene).to_dict()
    assert before["ready"]
    path.write_text(yaml.safe_dump(document(drawing("mount", "mount"), revision="drawing-B")))
    updated = tool.with_mounting(path)
    assert bt.mounting.report(scene).to_dict() == before
    assert bt.mounting.report(base.attach_tool(updated, flange="out", mount="mount")).input_hash != before["input_hash"]
    path.unlink()
    project = tmp_path / "assembly.botrail"
    scene.save_project(project)
    restored = bt.Scene.load_project(project)
    assert bt.mounting.report(restored).to_dict() == before
    assert scene.bom().rows == restored.bom().rows
    ns = {}
    exec("\n".join(line for line in restored.generate_python().splitlines() if line != "bt.studio(scene)"), ns)
    assert bt.mounting.report(ns["scene"]).to_dict() == before
    assert bt.mounting.report(scene._snapshot()).to_dict() == before
    with pytest.raises(ValueError, match="individual part"):
        robot._with_mounting_json(json.dumps(document()))
    assert item(bt.mounting.report(tool), "unmounted").status == "unknown"


def test_document_cannot_erase_catalog_mandatory_coupling(catalog, tmp_path):
    path = tmp_path / "tool.yaml"
    path.write_text(yaml.safe_dump(document(drawing("tool_root", "mount"))))
    tool = load(TOOL).with_mounting(path)
    robot = load(ARM).attach_tool(tool)
    assert item(bt.mounting.report(robot), "required:adapter").status == "fail"
    assert not bt.mounting.report(robot).ready


def test_robot_base_mount_is_not_a_standalone_end_effector(catalog):
    catalog(ARM, category="manipulator", mount="base", flange="left", mounting={
        "interfaces": [face("base", "mount", "floor"), face("left", "flange", "robot-face")]})
    report = bt.mounting.report(load(ARM))
    assert item(report, "none").status == "not_applicable"
    assert not any(i.id.endswith(":unmounted") for i in report.items)


def test_usd_frames_and_visual_sources_survive_portable_project(tmp_path):
    from test_visual_assets import write_visual_assets

    _, _, hand = write_visual_assets(tmp_path / "source")
    usd = bt.Robot.from_usd(hand)
    payload = json.dumps(document(drawing("base", "mount")))
    mounted = usd._with_mounting_json(payload)
    # The same drawing works before and after a display-only replacement.
    urdf = bt.Robot.from_urdf_string('<robot name="r"><link name="base"/><link name="tip"/><joint name="j" type="fixed"><parent link="base"/><child link="tip"/><origin xyz="0 0 .1"/></joint></robot>')
    for candidate in (mounted, urdf._with_mounting_json(payload).with_visuals(usd),
                      urdf.with_visuals(usd)._with_mounting_json(payload)):
        scene = bt.Scene(candidate)
        before = bt.mounting.report(scene).to_dict()
        project = tmp_path / "portable.botrail"
        scene.save_project(project)
        restored = bt.Scene.load_project(project)
        assert bt.mounting.report(restored).to_dict() == before
        assert restored.robot.link_names == candidate.link_names
        ns = {}
        exec(restored.generate_python().replace("bt.studio(scene)", ""), ns)
        assert bt.mounting.report(ns["scene"]).to_dict() == before


def test_explicit_alternative_adapter_must_be_in_the_actual_path(catalog):
    spec = {"interfaces": [face("tool_root", "mount", "tool-face")], "requirements": [{
        "id": "adapter", "frame": "tool_root", "interface_pair": {"mount": "robot-face", "flange": "tool-face"},
        "evidence": EVIDENCE, "note": "This fixture drawing explicitly permits equivalent adapters"}]}
    catalog(TOOL, mount="tool_root", mounting=spec)
    custom_xml = '<robot name="custom"><link name="mount"/><link name="out"/><joint name="j" type="fixed"><parent link="mount"/><child link="out"/></joint></robot>'
    custom = bt.Robot.from_urdf_string(custom_xml)._with_mounting_json(json.dumps(document(
        face("mount", "mount", "robot-face"), face("out", "flange", "tool-face"))))
    robot = load(ARM).attach_tool(custom, mount="mount", prefix="custom_").attach_tool(load(TOOL), flange="custom_out")
    assert item(bt.mounting.report(robot), "required:adapter").status == "pass"
    robot = load(ARM).attach_tool(custom, flange="right", mount="mount", prefix="other_")
    robot = robot.attach_tool(load(TOOL), flange="left")
    assert item(bt.mounting.report(robot), "required:adapter").status == "fail"
    # An unused matching output face cannot authorize an incompatible outlet.
    wrong = custom._with_mounting_json(json.dumps(document(
        face("mount", "mount", "robot-face"), face("out", "flange", "wrong-face"),
        face("mount", "flange", "tool-face"))))
    robot = load(ARM).attach_tool(wrong, mount="mount", prefix="custom_").attach_tool(load(TOOL), flange="custom_out")
    assert item(bt.mounting.report(robot), "required:adapter").status == "fail"


@pytest.mark.parametrize("change, message", [
    (lambda d: d.update(schema_version="2"), "schema_version"),
    (lambda d: d.update(revision=""), "blank"),
    (lambda d: d["mounting"]["interfaces"][0].update(frame="missing"), "does not exist"),
    (lambda d: d["mounting"]["interfaces"][0]["geometry"].update(normal_z=0), "normal_z"),
    (lambda d: d["mounting"]["interfaces"][0]["geometry"]["holes"][0].update(position_tolerance_mm=-1), "tolerance"),
    (lambda d: d["mounting"]["interfaces"][0]["fasteners"][0].update(holes=["h0", "h0"]), "distinct"),
    (lambda d: d["mounting"]["interfaces"][0]["clearance"].update(solids=[]), "solid"),
    (lambda d: d["mounting"]["interfaces"][0]["geometry"]["evidence"][0].update(source=9), "sources index"),
])
def test_invalid_custom_drawings_are_rejected(change, message):
    d = copy.deepcopy(document(drawing("mount", "mount")))
    change(d)
    with pytest.raises(ValueError, match=message):
        bt.Robot.from_urdf_string('<robot name="r"><link name="mount"/></robot>')._with_mounting_json(json.dumps(d))

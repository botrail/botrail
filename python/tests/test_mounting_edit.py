"""Assembly changes preserve source provenance and cannot reuse old approvals."""
import json

import botrail as bt
import pytest
from test_mounting_fit import bounds, document, drawing

ARM = '''<robot name="arm"><link name="root"/><link name="face"/>
<joint name="axis" type="revolute"><parent link="root"/><child link="face"/>
<axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="1" effort="1"/></joint></robot>'''
TOOL = '''<robot name="tool"><link name="mount"/><link name="tip"/>
<joint name="end" type="fixed"><parent link="mount"/><child link="tip"/>
<origin xyz="0 0 0.1"/></joint></robot>'''
ADAPTER = '''<robot name="adapter"><link name="mount"/><link name="out"/>
<joint name="thickness" type="fixed"><parent link="mount"/><child link="out"/>
<origin xyz="0 0 0.03"/></joint></robot>'''


def parts(*, axis="0 0 1", length=12):
    arm = bt.Robot.from_urdf_string(ARM.replace("0 0 1", axis))._with_mounting_json(json.dumps(document(drawing("face", "flange"))))
    mount = drawing("mount", "mount")
    mount["fasteners"][0]["length_mm"] = bounds(length)
    tool = bt.Robot.from_urdf_string(TOOL)._with_mounting_json(json.dumps(document(mount)))
    adapter = bt.Robot.from_urdf_string(ADAPTER)._with_mounting_json(json.dumps(document(drawing("mount", "mount"), drawing("out", "flange"))))
    return arm, tool, adapter


def direct(**kwargs):
    a, t, _ = parts(**kwargs)
    return a.attach_tool(t, flange="face", mount="mount", prefix="tool_")


def adapted(**kwargs):
    a, t, adapter = parts(**kwargs)
    return a.attach_tool(adapter, flange="face", mount="mount", prefix="adapter_").attach_tool(t, flange="adapter_out", mount="mount", prefix="tool_")


def test_adapter_preview_apply_geometry_tcp_bom_and_joint_state(tmp_path):
    scene = bt.Scene(direct())
    scene.set_joint_positions([0.4])
    before = scene._project_json()
    proposal = bt.mounting.preview(scene, adapted())
    assert scene._project_json() == before
    assert proposal.can_apply, proposal.data
    assert proposal.route == "adapter_evidence"
    assert proposal.preserved_joints == ["axis"]
    assert proposal.reset_joints == []
    assert proposal.after["tcp"]["pose"]["position"][2] - proposal.before["tcp"]["pose"]["position"][2] == pytest.approx(.03)
    assert proposal.after["links"] == proposal.before["links"] + 2
    assert len(proposal.after["bom"]) == len(proposal.before["bom"]) + 1
    assert proposal.after["mass"]["known_kg"] is None  # no invented zero
    proposal.save(tmp_path / "assembly.botrail")
    reloaded = bt.Scene.load_project(tmp_path / "assembly.botrail")
    assert bt.mounting.report(reloaded).to_dict() == proposal.report.to_dict()
    assert reloaded.joint_positions == pytest.approx([.4])
    proposal.apply()
    assert "adapter_out" in scene.robot.link_names
    assert scene.joint_positions == pytest.approx([.4])
    assert bt.mounting.report(scene).ready
    with pytest.raises(ValueError, match="Scene changed"):
        proposal.apply()


@pytest.mark.parametrize("change", ["pose", "base", "obstacle", "part"])
def test_any_scene_edit_rejects_old_preview(change):
    scene = bt.Scene(direct())
    proposal = bt.mounting.preview(scene, adapted())
    if change == "pose":
        scene.set_joint_positions([.2])
    elif change == "base":
        scene.set_robot_base_pose(position=(.1, 0, 0))
    elif change == "obstacle":
        scene.add_box("workpiece", (.1, .1, .1), (1, 0, 0))
    else:
        scene.set_part("arm", model="Different arm")
    with pytest.raises(ValueError, match="Scene changed"):
        proposal.apply()
    assert "adapter_out" not in scene.robot.link_names


def test_changed_joint_definition_resets_pose_instead_of_copying_by_name():
    scene = bt.Scene(direct())
    scene.set_joint_positions([.8])
    proposal = bt.mounting.preview(scene, adapted(axis="0 1 0"))
    assert proposal.preserved_joints == []
    assert proposal.reset_joints == ["axis"]
    assert proposal.scene.joint_positions == pytest.approx([0])
    proposal.apply()
    assert scene.joint_positions == pytest.approx([0])


@pytest.mark.parametrize("candidate", [lambda: direct(length=9), lambda: bt.Robot.from_urdf_string(ARM).attach_tool(bt.Robot.from_urdf_string(TOOL), flange="face", mount="mount"), lambda: parts()[0]])
def test_unknown_failed_and_empty_connections_remain_saveable_drafts(tmp_path, candidate):
    scene = bt.Scene(direct())
    proposal = bt.mounting.preview(scene, candidate())
    assert not proposal.can_apply
    before = scene._project_json()
    proposal.save(tmp_path / "draft.botrail")
    saved = bt.Scene.load_project(tmp_path / "draft.botrail")
    assert bt.mounting.report(saved).to_dict() == proposal.report.to_dict()
    with pytest.raises(ValueError, match="unresolved"):
        proposal.apply()
    assert scene._project_json() == before


def test_changed_tool_joint_reference_blocks_apply_and_preserves_authored_motion(tmp_path):
    scene = bt.Scene(direct())
    project = json.loads(scene._project_json())
    project["motions"] = [{"name": "move", "robot": "arm", "segments": [{"kind": "joint", "goal_positions": [.4], "constraints": []}]}]
    (tmp_path / "authored.botrail").write_text(json.dumps(project))
    scene = bt.Scene.load_project(tmp_path / "authored.botrail")
    before = scene._project_json()
    proposal = bt.mounting.preview(scene, adapted(axis="0 1 0"))
    assert not proposal.can_apply
    assert "identity changed" in " ".join(proposal.blockers)
    assert scene._project_json() == before


@pytest.mark.parametrize("broadcast", [True, False])
def test_motion_revalidation_survives_save_and_clears_only_after_fresh_plan(tmp_path, broadcast):
    scene = bt.Scene(direct())
    project = json.loads(scene._project_json())
    project["motions"] = [{"name": "move", "robot": "arm", "segments": [{"kind": "joint", "goal_positions": [.4], "constraints": []}]}]
    (tmp_path / "authored.botrail").write_text(json.dumps(project))
    scene = bt.Scene.load_project(tmp_path / "authored.botrail")
    proposal = bt.mounting.preview(scene, adapted())
    assert proposal.revalidation == ["motion:move"]
    proposal.apply()
    scene.save_project(tmp_path / "cell.botrail")
    loaded = bt.Scene.load_project(tmp_path / "cell.botrail")
    assert json.loads(loaded._project_json())["mounting_revalidation"] == ["motion:move"]
    loaded.plan_motion("move", broadcast=broadcast)
    assert not json.loads(loaded._project_json()).get("mounting_revalidation")


def test_candidates_keep_failures_and_unknowns_visible():
    scene = bt.Scene(direct())
    proposals = bt.mounting.candidates(scene, [parts()[0], adapted(), direct(length=9), direct()])
    assert [p.route for p in proposals] == ["direct_evidence", "direct_evidence", "adapter_evidence", "needs_information"]
    assert not proposals[0].can_apply
    assert proposals[1].can_apply


def test_removed_attachment_frame_is_an_apply_blocker():
    scene = bt.Scene(direct())
    scene.add_box("work", (.01, .01, .01), (0, 0, .1))
    scene.attach("work", link="tool_tip")
    arm, tool, _ = parts()
    candidate = arm.attach_tool(tool, flange="face", mount="mount", prefix="changed_")
    before = scene._project_json()
    proposal = bt.mounting.preview(scene, candidate)
    assert not proposal.can_apply
    assert "tool_tip" in " ".join(proposal.blockers)
    with pytest.raises(ValueError):
        proposal.apply()
    assert scene._project_json() == before


def test_nonfirst_robot_replacement_preserves_other_robot_pose():
    scene = bt.Scene(direct())
    scene.add_robot(direct(), name="second", base_position=(1, 0, 0))
    scene.set_joint_positions([.1], robot="arm")
    scene.set_joint_positions([.7], robot="second")
    proposal = bt.mounting.preview(scene, adapted(), robot="second")
    proposal.apply()
    assert "adapter_out" not in scene.robot.link_names
    assert "adapter_out" in scene.robot_of("second").link_names
    assert scene.joint_positions_of("arm") == [.1]
    assert scene.joint_positions_of("second") == [.7]


def test_server_rechecks_candidate_instead_of_trusting_preview_approval():
    scene = bt.Scene(direct())
    proposal = bt.mounting.preview(scene, adapted())
    with pytest.raises(ValueError, match="Candidate inputs changed"):
        scene._mounting_apply(direct(length=9), proposal.data["base_revision"], proposal.data["candidate_revision"])
    assert bt.mounting.report(scene).ready


def test_kit_candidate_keeps_purchase_unit_and_evidence_when_saved(tmp_path):
    import test_catalog_kits as kits
    import yaml

    root, _ = kits.packages.__wrapped__(tmp_path)
    for pid, mass in [(kits.ARM, 10), (kits.KIT, 2.5)]:
        path = root / pid / "manifest.yaml"
        manifest = yaml.safe_load(path.read_text())
        manifest["specs"] = {"mass_kg": mass}
        path.write_text(yaml.safe_dump(manifest))
    arm = kits.load(root, kits.ARM)
    scene = bt.Scene(arm)
    proposal = bt.mounting.preview(scene, arm.attach_tool(kits.load(root), prefix="kit_"))
    assert proposal.route == "adapter_evidence"
    assert proposal.can_apply
    assert not proposal.report.ready
    assert proposal.mounting_blockers == []
    assert len(proposal.after["bom"]) == 2
    assert proposal.after["mass"] == {"known_kg": 12.5, "missing": []}
    assert proposal.report.kits[0]["manufacturer_support"]["status"] == "pass"
    assert {c["basis"] for c in proposal.report.simulation["connections"]} == {"manufacturer_kit"}
    assert "catalog_adapter" in {c["method"] for c in proposal.report.simulation["connections"]}
    proposal.save(tmp_path / "kit-proposal.botrail")
    restored = bt.Scene.load_project(tmp_path / "kit-proposal.botrail")
    assert restored.bom().rows == proposal.scene.bom().rows
    assert bt.mounting.report(restored).to_dict() == proposal.report.to_dict()
    proposal.apply()
    assert bt.mounting.report(scene).to_dict() == proposal.report.to_dict()
    assert scene.bom().rows == restored.bom().rows


@pytest.mark.parametrize("change", ["host", "host_offset", "host_rotation", "host_frame", "component", "internal_offset", "identity", "interface", "extra_tool", "standalone"])
def test_standard_kit_application_is_scoped_to_the_actual_assembly(tmp_path, change):
    import test_catalog_kits as kits
    import yaml

    root, _ = kits.packages.__wrapped__(tmp_path)
    if change == "interface":
        path = root / kits.BASE / "manifest.yaml"
        data = yaml.safe_load(path.read_text())
        data["mounting"]["interfaces"][0]["interface_id"] = "wrong-face"
        path.write_text(yaml.safe_dump(data))
    if change == "host_frame":
        path = root / kits.ARM / "model.urdf"
        path.write_text(path.read_text().replace("</robot>", '<link name="other"/><joint name="second_face" type="fixed"><parent link="body"/><child link="other"/></joint></robot>'))
    arm = kits.load(root, kits.ARM)
    scene = bt.Scene(arm)
    kit = kits.load(root)
    host = kits.load(root, kits.BASE) if change == "host" else arm
    options = {"prefix": "kit_"}
    if change == "host_offset":
        options["offset_position"] = (0, 0, .02)
    if change == "host_rotation":
        options["offset_quaternion"] = (0, 0, .7071067811865476, .7071067811865476)
    if change == "host_frame":
        options["flange"] = "other"
    candidate = host.attach_tool(kit, **options)
    if change in ("component", "internal_offset"):
        source = json.loads(bt.Scene(candidate)._project_json())["robots"][0]["source"]
        inside = source["tool"]["inner"]
        if change == "component":
            inside["tool"]["id"] = kits.BASE
        else:
            inside["offset"]["position"][2] = .02
        candidate = bt.Robot._from_source_json(json.dumps(source))
    if change == "identity":
        scene.set_part("robot", catalog=kits.BASE)
    if change == "extra_tool":
        candidate = candidate.attach_tool(bt.Robot.from_urdf_string(TOOL), flange="kit_gripper/body", mount="mount", prefix="extra_")
    if change == "standalone":
        candidate = kit
    proposal = bt.mounting.preview(scene, candidate)
    assert not proposal.can_apply, proposal.data
    assert proposal.mounting_blockers
    if change in ("interface", "identity", "extra_tool"):
        # A valid claim for one connection never suppresses mismatches or
        # missing information outside that connection.
        assert proposal.report.kits[0]["manufacturer_support"]["status"] == "pass"
    before = scene._project_json()
    with pytest.raises(ValueError, match="unresolved"):
        proposal.apply()
    assert scene._project_json() == before


def test_kit_with_explicit_permitted_host_pose_requires_that_pose(tmp_path):
    import test_catalog_kits as kits
    import yaml

    root, _ = kits.packages.__wrapped__(tmp_path)
    path = root / kits.BASE / "manifest.yaml"
    data = yaml.safe_load(path.read_text())
    data["mounting"]["interfaces"][0]["allowed_poses"] = [
        {"position": [0, 0, .01], "quaternion": [0, 0, 0, 1]}
    ]
    path.write_text(yaml.safe_dump(data))
    arm, kit = kits.load(root, kits.ARM), kits.load(root)
    scene = bt.Scene(arm)
    assert not bt.mounting.preview(scene, arm.attach_tool(kit, prefix="kit_")).can_apply
    proposal = bt.mounting.preview(scene, arm.attach_tool(kit, prefix="kit_", offset_position=(0, 0, .01)))
    assert proposal.can_apply and not proposal.report.ready


def test_text_mass_does_not_make_a_partial_total_look_complete():
    scene = bt.Scene(direct())
    scene.set_part("arm", attributes={"mass_kg": 10})
    scene.set_part("arm/tool", attributes={"mass_kg": "unknown"})
    proposal = bt.mounting.preview(scene, direct())
    assert proposal.after["mass"] == {"known_kg": 10, "missing": ["arm/tool"]}


def test_orphan_annotation_blocks_apply_but_draft_remains_loadable(tmp_path):
    scene = bt.Scene(adapted())
    scene.set_part("arm/tool2", model="Hand label")
    proposal = bt.mounting.preview(scene, direct())
    assert not proposal.can_apply
    assert "omitted from the standalone draft" in " ".join(proposal.blockers)
    proposal.save(tmp_path / "draft.botrail")
    restored = bt.Scene.load_project(tmp_path / "draft.botrail")
    assert bt.mounting.report(restored).ready
    assert json.loads(scene._project_json())["parts"][0]["target"] == "arm/tool2"


def declared_parts(*, detail=False, change=None):
    """Synthetic drawing-backed route, deliberately not a manufacturer kit."""
    a, b, c, d = (drawing("face", "flange"), drawing("mount", "mount"),
                  drawing("mount", "mount"), drawing("out", "flange"))
    for face in (a, b, c, d):
        face.pop("clearance")
        if not detail:
            face.pop("geometry")
            face.pop("fasteners")
    if change == "interface":
        b["interface_id"] = "different-face"
    elif change == "pose":
        b["allowed_poses"] = [{"position": [0, 0, .01], "quaternion": [0, 0, 0, 1]}]
    elif change == "parts":
        b["requirements_complete"] = False
    elif change == "evidence":
        b["evidence"] = []
    elif change == "names_only":
        b.pop("allowed_poses")
    elif change == "screw":
        b["fasteners"][0]["length_mm"] = bounds(9)
    elif change == "holes":
        b["geometry"]["holes"][0]["position_mm"][0] += 10
    elif change == "no_identifiers":
        for face in (a, b, c, d):
            face.pop("interface_id")
    return tuple(bt.Robot.from_urdf_string(xml)._with_mounting_json(json.dumps(document(*faces)))
                 for xml, faces in ((ARM, (a,)), (TOOL, (b,)), (ADAPTER, (c, d))))


@pytest.mark.parametrize("adapter", [False, True])
@pytest.mark.parametrize("detail", [False, True])
def test_documented_nonkit_route_applies_with_separate_detail_status(tmp_path, adapter, detail):
    a, t, plate = declared_parts(detail=detail)
    candidate = a
    if adapter:
        candidate = candidate.attach_tool(plate, flange="face", mount="mount", prefix="plate_")
    candidate = candidate.attach_tool(t, flange="plate_out" if adapter else "face", mount="mount", prefix="tool_")
    scene = bt.Scene(a)
    proposal = bt.mounting.preview(scene, candidate)
    assert proposal.can_apply and not proposal.report.ready
    assert not proposal.report.kits
    sim = proposal.report.simulation
    assert sim["ready"] and not sim["blockers"]
    connection = sim["connections"][-1]
    assert connection["method"] == ("custom_adapter" if adapter else "direct")
    assert connection["basis"] == ("dimensional_checks" if detail else "interface_declarations")
    assert connection["evidence_items"]
    assert any(i.id.endswith(":assembly_clearance") and i.status == "not_run" for i in proposal.report.items)
    assert connection["basis"] in proposal.report.to_markdown()
    # Immutable drawing sources, topology and results survive both handoffs.
    proposal.save(tmp_path / "declared-route.botrail")
    restored = bt.Scene.load_project(tmp_path / "declared-route.botrail")
    assert bt.mounting.report(restored).to_dict() == proposal.report.to_dict()
    proposal.apply()
    assert bt.mounting.report(scene).to_dict() == proposal.report.to_dict()
    assert not bt.review(scene, required=["mounting"]).ready


@pytest.mark.parametrize("change", ["interface", "pose", "parts", "evidence", "names_only", "screw", "holes"])
def test_nonkit_support_never_hides_mismatches_or_missing_installation_evidence(change):
    a, t, _ = declared_parts(detail=change in ("screw", "holes"), change=change)
    scene = bt.Scene(a)
    proposal = bt.mounting.preview(scene, a.attach_tool(t, flange="face", mount="mount"))
    assert not proposal.can_apply
    assert proposal.mounting_blockers == proposal.report.simulation["blockers"]
    before = scene._project_json()
    with pytest.raises(ValueError, match="unresolved"):
        proposal.apply()
    assert scene._project_json() == before


def test_dimension_checks_do_not_require_inventing_a_shared_interface_name():
    a, t, _ = declared_parts(detail=True, change="no_identifiers")
    proposal = bt.mounting.preview(bt.Scene(a), a.attach_tool(t, flange="face", mount="mount"))
    assert proposal.can_apply and not proposal.report.ready
    assert proposal.report.simulation["connections"][0]["basis"] == "dimensional_checks"
    assert proposal.route == "direct_evidence"


def test_evidence_on_one_connection_cannot_cover_an_extra_undeclared_tool():
    a, t, _ = declared_parts()
    supported = a.attach_tool(t, flange="face", mount="mount", prefix="tool_")
    candidate = supported.attach_tool(bt.Robot.from_urdf_string(TOOL), flange="tool_tip", mount="mount", prefix="extra_")
    proposal = bt.mounting.preview(bt.Scene(a), candidate)
    assert not proposal.can_apply
    assert proposal.report.simulation["connections"][0]["basis"] == "interface_declarations"
    assert proposal.report.simulation["connections"][1]["basis"] == "unknown"


def test_arm_pedestal_is_not_an_end_effector_adapter():
    body = bt.Robot.from_urdf_string('<robot name="cell"><link name="pedestal"/></robot>')
    a, t, _ = declared_parts()
    arm = a.attach_tool(t, flange="face", mount="mount", prefix="tool_")
    mounted = body.mount(arm, at="pedestal", prefix="arm_", group="arm")
    proposal = bt.mounting.preview(bt.Scene(body), mounted)
    assert proposal.can_apply
    assert proposal.report.simulation["connections"][0]["method"] == "direct"
    assert proposal.route == "direct_evidence"

"""Vehicle frame alignment; fixtures are synthetic, not hardware qualification."""

# Generated-script replay is the contract under test; all sources are local fixtures.
# ruff: noqa: S102

import json
import math

import botrail as bt
import pytest
from test_mounting import ARM, IDENTITY, SHA, SOURCE, complete, face, load
from test_mounting import catalog as catalog_fixture

catalog = catalog_fixture  # Reuse the local catalog transport fixture.

CARRIER = "acme/carrier/mobile/r1"


def cell(robot=None):
    scene = bt.Scene(robot or load(ARM), name="arm")
    scene.add_box("cart/body", (0.4, 0.3, 0.02), (3, 4, 0.01))
    scene.set_obstacle_enabled("cart/body", False)
    scene.add_vehicle("cart", body=["cart"], path=[(3, 4), (4, 4)],
                      stations={"a": 0, "b": 1}, speed=0.5)
    return scene


def carrier(catalog, **kwargs):
    catalog(CARRIER, category="vehicle.amr", mount="ground", flange="deck",
            extra_frames=["other"], **kwargs)
    return load(CARRIER)


def verdict(scene):
    return next(i for i in bt.mounting.report(scene).items if i.key == "mount_pose")


def test_declared_frames_supply_default_pose_without_changing_tool_checks(catalog):
    scene = cell(complete())
    scene.mount_robot("cart", carrier=carrier(catalog))
    report = bt.mounting.report(scene)
    assert [(i.key, i.status) for i in report.items if i.key in {"required:adapter", "mount_pose"}] == [("required:adapter", "pass"), ("mount_pose", "pass")]
    assert not report.ready  # Frame alignment alone does not verify detailed mechanical fit.
    assert scene.robot_base_pose[0] == pytest.approx((3, 4, 0.05))
    assert verdict(scene).evidence["reference"]["carrier"] == {"id": CARRIER, "revision": SHA}
    assert verdict(scene).evidence["scope"] == "catalog_frame_alignment"
    assert report.assemblies[-1]["upstream_parts"] == ["cart"]


@pytest.mark.parametrize("offset,rotation", [
    ((-0.2423, -0.1765, 0.0655), (0, 0, 0, 1)),
    ((0, 0, 0.065), (0, 0, 0, 1)),
    ((0, 0, 0.05), (0, 0, math.sin(0.2), math.cos(0.2))),
])
def test_explicit_translation_or_rotation_is_preserved_and_fails(catalog, offset, rotation):
    scene = cell()
    scene.mount_robot("cart", carrier=carrier(catalog), offset_position=offset,
                      offset_quaternion=rotation)
    item = verdict(scene)
    assert item.status == "fail"
    assert not bt.mounting.report(scene).ready
    assert scene.robot_base_pose[0] == pytest.approx((3 + offset[0], 4 + offset[1], offset[2]))
    comparison = item.evidence["comparison"]
    assert comparison["actual_offset"]["position"] == pytest.approx(offset)
    assert comparison["translation_error_m"] == pytest.approx(math.dist(offset, (0, 0, 0.05)))
    assert comparison["rotation_error_rad"] == pytest.approx(2 * math.acos(rotation[3]))


def test_opposite_quaternion_sign_is_the_same_orientation(catalog):
    scene = cell()
    scene.mount_robot("cart", carrier=carrier(catalog), offset_position=(0, 0, 0.05),
                      offset_quaternion=(0, 0, 0, -1))
    assert verdict(scene).status == "pass"


def test_mount_side_allowed_poses_follow_the_tool_mounting_convention(catalog):
    shifted = {**IDENTITY, "position": [0, 0, 0.015]}
    declaration = face("base", "mount", "carrier-face", allowed_poses=[shifted, IDENTITY])
    catalog(ARM, category="manipulator", mount="base", mounting={"interfaces": [declaration]})
    scene = cell()
    base = carrier(catalog)
    scene.mount_robot("cart", carrier=base)
    assert scene.robot_base_pose[0] == pytest.approx((3, 4, 0.065))
    assert verdict(scene).status == "pass"
    scene.mount_robot("cart", carrier=base, offset_position=(0, 0, 0.05))
    assert verdict(scene).status == "pass"
    assert verdict(scene).evidence["comparison"]["translation_error_m"] == pytest.approx(0)
    declaration["evidence"] = []
    catalog(ARM, category="manipulator", mount="base", mounting={"interfaces": [declaration]})
    unknown = cell()
    unknown.mount_robot("cart", carrier=base)
    assert verdict(unknown).status == "unknown"


def test_composite_carrier_does_not_misattribute_its_new_flange(catalog):
    from test_mounting import ADAPTER

    composite = carrier(catalog).attach_tool(load(ADAPTER), prefix="adapter")
    scene = cell()
    scene.mount_robot("cart", carrier=composite)
    assert verdict(scene).status == "unknown"


def test_unknown_inputs_never_pass_or_get_fixed_by_bom_labels(catalog):
    scene = cell()
    scene.mount_robot("cart", offset_position=(0, 0, 0.05))
    scene.set_part("cart", kind="device", catalog=CARRIER)
    assert verdict(scene).status == "unknown"
    assert not bt.mounting.report(scene).ready
    scene.mount_robot("cart", carrier=carrier(catalog, sources=[]))
    assert verdict(scene).status == "unknown"
    scene.mount_robot("cart", carrier=carrier(catalog, sources=[{**SOURCE, "kind": "community"}]))
    assert verdict(scene).status == "unknown"
    scene.mount_robot("cart", carrier=carrier(catalog, sources=[{**SOURCE, "url": ""}]))
    assert verdict(scene).status == "unknown"
    scene.mount_robot("cart", carrier=carrier(catalog), flange="other")
    assert verdict(scene).status == "unknown"
    scene.mount_robot("cart", carrier=load(CARRIER), mount="left")
    assert verdict(scene).status == "unknown"
    raw = bt.Robot.from_urdf_string('<robot name="raw"><link name="deck"/></robot>')
    scene.mount_robot("cart", carrier=raw, flange="deck")
    assert verdict(scene).status == "unknown"
    raw_arm = cell(raw)
    raw_arm.mount_robot("cart", carrier=load(CARRIER))
    assert verdict(raw_arm).status == "unknown"


def test_rotated_carrier_and_nonroot_arm_mount_use_full_transforms(catalog):
    arm = catalog(ARM, category="manipulator", mount="foot")
    (arm / "robot.urdf").write_text('''<robot name="arm">
      <link name="root"/><link name="foot"/>
      <joint name="foot_joint" type="fixed"><parent link="root"/><child link="foot"/>
        <origin xyz="0.05 0 0.02" rpy="0 0 -1.5707963267948966"/>
      </joint></robot>''')
    base = catalog(CARRIER, category="vehicle.amr", mount="ground", flange="deck")
    (base / "robot.urdf").write_text('''<robot name="carrier">
      <link name="ground"/><link name="deck"/>
      <joint name="deck_joint" type="fixed"><parent link="ground"/><child link="deck"/>
        <origin xyz="0.2 -0.1 0.7" rpy="0 0 1.5707963267948966"/>
      </joint></robot>''')
    scene = cell()
    scene.mount_robot("cart", carrier=load(CARRIER))
    assert verdict(scene).status == "pass"
    position, quaternion = scene.robot_base_pose
    assert position == pytest.approx((3.25, 3.9, 0.68))
    assert abs(quaternion[2]) == pytest.approx(1)
    assert scene.link_pose("foot")[0] == pytest.approx((3.2, 3.9, 0.7))


@pytest.mark.parametrize("sources", [[], [{**SOURCE, "kind": "community"}], [{**SOURCE, "url": ""}]])
def test_arm_frame_without_supporting_sources_is_unknown(catalog, sources):
    package = catalog(ARM, category="manipulator", mount="foot", sources=sources)
    (package / "robot.urdf").write_text('''<robot name="arm">
      <link name="root"/><link name="foot"/>
      <joint name="foot_joint" type="fixed"><parent link="root"/><child link="foot"/>
        <origin xyz="0 0 0.02"/>
      </joint></robot>''')
    scene = cell()
    scene.mount_robot("cart", carrier=carrier(catalog))
    assert verdict(scene).status == "unknown"


def test_model_root_comparison_does_not_claim_a_physical_arm_mount_face(catalog):
    catalog(ARM, raw={"id": ARM, "category": "manipulator", "name": "arm",
                     "manufacturer": {"name": "ACME"}, "frames": {},
                     "sources": [{**SOURCE, "kind": "community"}]})
    scene = cell()
    scene.mount_robot("cart", carrier=carrier(catalog))
    item = verdict(scene)
    assert item.status == "pass"
    assert item.evidence["arm_mount_basis"] == "model_root"
    assert item.evidence["arm_sources"][0]["kind"] == "community"


def test_bad_frames_and_nonfinite_offsets_fail_before_mutating_mount(catalog):
    scene = cell()
    base = carrier(catalog)
    scene.mount_robot("cart", carrier=base)
    before = bt.mounting.report(scene).to_dict()
    for kwargs, message in [
        ({"flange": "missing"}, "unknown link"),
        ({"mount": "missing"}, "unknown link"),
        ({"offset_position": (float("nan"), 0, 0)}, "finite"),
    ]:
        with pytest.raises(ValueError, match=message):
            scene.mount_robot("cart", carrier=base, **kwargs)
        assert bt.mounting.report(scene).to_dict() == before
    with pytest.raises(ValueError, match="require carrier"):
        scene.mount_robot("cart", flange="deck")


def test_articulated_mount_frame_is_rejected(catalog):
    package = catalog(CARRIER, category="vehicle.amr", mount="ground", flange="deck")
    xml = (package / "robot.urdf").read_text().replace('type="fixed"', 'type="continuous"')
    (package / "robot.urdf").write_text(xml)
    with pytest.raises(ValueError, match="must be fixed"):
        cell().mount_robot("cart", carrier=load(CARRIER))


@pytest.mark.parametrize("aligned", [False, True])
def test_snapshot_project_and_python_preserve_result_without_carrier_fetch(catalog, tmp_path, monkeypatch, aligned):
    scene = cell()
    kwargs = {} if aligned else {"offset_position": (-0.24, -0.17, 0.065)}
    scene.mount_robot("cart", carrier=carrier(catalog), **kwargs)
    before = bt.mounting.report(scene).to_dict()
    project = tmp_path / "mount.botrail"
    scene.save_project(project)
    restored = bt.Scene.load_project(project)
    assert bt.mounting.report(restored).to_dict() == before
    assert bt.mounting.report(scene._snapshot()).to_dict() == before
    # Frozen frame evidence must survive later changes to the catalog.
    catalog(CARRIER, category="vehicle.amr", mount="ground", flange="changed", sources=[])
    monkeypatch.setattr(bt, "studio", lambda *a, **k: None)
    namespace = {}
    exec(restored.generate_python(), namespace)
    assert bt.mounting.report(namespace["scene"]).to_dict() == before
    import huggingface_hub as hub

    def unexpected_fetch(*args, **kwargs):
        pytest.fail("embedded replay must not fetch any catalog")

    monkeypatch.setattr(hub, "dataset_info", unexpected_fetch)
    namespace = {}
    exec(restored.generate_python(embed_catalog=True), namespace)
    assert bt.mounting.report(namespace["scene"]).to_dict() == before
    # Old saved mounts are readable and explicitly unverified.
    data = json.loads(project.read_text())
    del data["robots"][0]["mount"]["reference"]
    project.write_text(json.dumps(data))
    assert verdict(bt.Scene.load_project(project)).status == "unknown"


@pytest.mark.parametrize("field,value", [("mount", "missing"), ("flange_pose", {"position": [0, 0, 0], "quaternion": [0, 0, 0, 2]})])
def test_corrupt_saved_reference_is_rejected(catalog, tmp_path, field, value):
    scene = cell()
    scene.mount_robot("cart", carrier=carrier(catalog))
    path = tmp_path / "mount.botrail"
    scene.save_project(path)
    data = json.loads(path.read_text())
    data["robots"][0]["mount"]["reference"][field] = value
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="mount reference"):
        bt.Scene.load_project(path)

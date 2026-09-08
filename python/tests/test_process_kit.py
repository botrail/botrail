"""Real ATI kit declarations on synthetic geometry; no hardware-fit claim."""
import json
from pathlib import Path

import botrail as bt
import pytest
from test_catalog_kits import write_manifest


@pytest.mark.parametrize("change", [None, "missing_bracket", "rotated_spindle"])
def test_documented_side_mount_spindle_kit(tmp_path, change):
    data = json.loads((Path(__file__).parent / "data/mounting/ati_rcv250_crx10_kit.json").read_text())
    for component in [{"id": data["host"]}, *data["components"]]:
        pid = component["id"]
        host = pid == data["host"]
        root = data["host_flange"] if host else "mount"
        frames = {"mount_frame": root, "flange_frame": root, "tcp_default": root}
        links = f'<link name="{root}"><visual><geometry><box size=".01 .01 .01"/></geometry></visual></link>'
        if "flange_z_m" in component:
            frames["flange_frame"] = "flange"
            links += f'<link name="flange"/><joint name="out" type="fixed"><parent link="{root}"/><child link="flange"/><origin xyz="0 0 {component["flange_z_m"]}"/></joint>'
        if not host:
            frames["tcp_default"] = "tcp"
            links += f'<link name="tcp"/><joint name="reference_tcp" type="fixed"><parent link="{root}"/><child link="tcp"/></joint>'
        path = write_manifest(tmp_path, pid, {
            "id": pid, "kind": "model", "category": "manipulator" if host else "adapter",
            "name": pid.split("/")[-2], "manufacturer": {"name": "FANUC" if host else "ATI"},
            "frames": frames, "assets": {"urdf": "model.urdf"},
        })
        (path / "model.urdf").write_text(f'<robot name="fixture">{links}</robot>')
    manifest = data["manifest"]
    write_manifest(tmp_path, manifest["id"], manifest)
    arm = bt.Robot.from_package(tmp_path / data["host"])
    kit = bt.Robot.from_package(tmp_path / manifest["id"], catalog_root=tmp_path)
    assembled = arm.attach_tool(kit, prefix="kit_")
    if change:
        source = json.loads(bt.Scene(assembled)._project_json())["robots"][0]["source"]
        inner = source["tool"]["inner"]
        if change == "missing_bracket":
            inner["flange"] = inner["base"]["flange"]
            inner["base"] = inner["base"]["base"]
        else:
            inner["offset"]["quaternion"] = [0, 0, 0, 1]
        assembled = bt.Robot._from_source_json(json.dumps(source))
    scene = bt.Scene(assembled)
    report = bt.mounting.report(scene)
    result, = report.kits
    assert result["composition"]["status"] == ("fail" if change else "pass")
    assert result["manufacturer_support"]["status"] == ("not_applicable" if change else "pass")
    assert bt.mounting.preview(bt.Scene(arm), assembled).can_apply is (change is None)
    assert not report.ready
    if change is None:
        assert len(report.assemblies) == 3
        assert result["detailed_fit"]["status"] == "unknown"
        assert len(scene.bom().rows) == 2
        purchase = scene.bom().rows[-1]
        assert purchase["order"]["part_number"] == "9150-COB-CRX10-RCV250-01"
        assert purchase["qty"] == 1
        included = purchase["order"]["includes"]
        assert [i["catalog"] for i in included if i.get("catalog")] == [p["id"] for p in data["components"]]

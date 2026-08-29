"""Field sensors: a lidar's scan-plane sector as a presence input.

The rollout regressions the design asks for — in the field / beyond its
radius / hidden behind another body — plus the sector window, the
vehicle anchor with its own-body exclusion, and the things that must
come for free (I/O derivation) or must not happen (a BOM line).
"""

from pathlib import Path

import botrail as bt
import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def scene() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))


def bake_hold(scene: bt.Scene, name: str = "sweep"):
    sq = scene.sequence(name)
    sq.step("hold", transition=bt.seq.elapsed(0.1))
    return sq.simulate()


def test_field_in_range_and_shadowing(scene) -> None:
    # A part 1 m in front of a fixture scanner looking back down -X
    # (yaw=180 puts the scan heading on -X; the part sits mid-sweep).
    scene.add_box("part", (0.1, 0.1, 0.1), (0.5, 0.0, 0.2))
    scene.add_lidar("gate", position=(1.5, 0.0, 0.2), yaw=180.0, fov=270.0, range=(0.05, 5.0))
    scene.add_field_sensor("near", lidar="gate", watch=["part"])
    scene.add_field_sensor("close_only", lidar="gate", range=0.4, watch=["part"])
    tl = bake_hold(scene)
    assert tl.signal("near").value_at(0.05) is True
    # The part sits 1 m out — beyond a 0.4 m field.
    assert tl.signal("close_only").value_at(0.05) is False

    # A wall between scanner and part: shadowing blocks the default
    # field; one with shadowing off still trips on the sector overlap.
    scene.add_box("wall", (0.05, 0.6, 0.6), (1.0, 0.0, 0.3))
    scene.add_field_sensor("xray", lidar="gate", watch=["part"], shadowing=False)
    tl = bake_hold(scene, "sweep2")
    assert tl.signal("near").value_at(0.05) is False
    assert tl.signal("xray").value_at(0.05) is True


def test_field_sector_window(scene) -> None:
    # Two parts at 1 m: one on the scan heading, one at 90° to it. A
    # ±30° field sees only the first; the full sweep sees both.
    scene.add_box("ahead", (0.1, 0.1, 0.1), (1.0, 0.0, 0.2))
    scene.add_box("beside", (0.1, 0.1, 0.1), (0.0, 1.0, 0.2))
    scene.add_lidar("scan", position=(0.0, 0.0, 0.2), fov=270.0, range=(0.05, 5.0))
    scene.add_field_sensor("narrow", lidar="scan", sector=(-30.0, 30.0))
    scene.add_field_sensor("wide", lidar="scan")
    tl = bake_hold(scene)
    assert tl.signal("narrow").value_at(0.05) is True
    assert tl.signal("wide").value_at(0.05) is True
    # Narrow the watch to the side part: the window excludes it.
    scene.add_field_sensor("narrow_side", lidar="scan", sector=(-30.0, 30.0), watch=["beside"])
    tl = bake_hold(scene, "sweep2")
    assert tl.signal("narrow_side").value_at(0.05) is False


def test_vehicle_field_rides_and_ignores_own_body(scene) -> None:
    # An AGV with a nav scanner drives toward a wall. Its field watches
    # everything — which includes its own chassis crossing the scan
    # plane, so without the own-body exclusion the lane would sit high
    # from t=0.
    scene.add_box("chassis", (0.5, 0.35, 0.25), (0.0, 2.0, 0.125))
    scene.add_vehicle(
        "agv", body=["chassis"], path=[(0.0, 2.0), (2.0, 2.0)], stations={"A": 0, "B": 1}
    )
    scene.add_lidar("nav", mount="agv", position=(0.3, 0.0, 0.2), fov=360.0, range=(0.05, 1.2))
    scene.add_box("wall", (0.1, 0.8, 0.6), (2.8, 2.0, 0.3))
    scene.add_field_sensor("near_wall", lidar="nav")
    sq = scene.sequence("drive")
    sq.step("go", actions=[bt.seq.goto("agv", "B")], transition=bt.seq.device_done("agv"))
    tl = sq.simulate()
    # Parked at A: the wall is 2.5 m out, only the chassis is inside the
    # sweep — excluded, so the lane is low.
    assert tl.signal("near_wall").value_at(0.05) is False
    # Docked at B: the scanner sits 0.5 m from the wall face.
    assert tl.signal("near_wall").value_at(tl.duration - 0.05) is True
    # The lane rose somewhere mid-drive, not at t=0.
    rising = tl.signal("near_wall").rising_edges()
    assert rising and 0.2 < rising[0] < tl.duration, rising


def test_field_derives_io_but_no_bom_line(scene) -> None:
    scene.add_box("part", (0.1, 0.1, 0.1), (0.5, 0.0, 0.2))
    scene.add_lidar("gate", position=(1.5, 0.0, 0.2), yaw=180.0)
    scene.add_field_sensor("near", lidar="gate", watch=["part"])
    # Rule ① is kind-agnostic: the lane derives a DI input contact.
    points = {p.name: p for p in scene.io_points()}
    assert "near" in points
    assert points["near"].direction == "input"
    # The purchasable article is the lidar, not the judgement: exactly
    # one row, the scanner's.
    rows = {tuple(row["names"]): row for row in scene.bom().rows}
    assert rows[("gate",)]["category"] == "sensor.lidar"
    assert ("near",) not in rows
    # Selection walks the scanner's row without tripping (the proper
    # requirement derivation is L4's work).
    bt.select.requirements(scene)


def test_field_fault_pins_the_lane(scene) -> None:
    # "The field is blocked" as a scenario fault: the lane is forced high
    # with nothing in the sector — the forced-lane machinery is
    # kind-agnostic, so this comes for free.
    scene.add_lidar("gate", position=(1.5, 0.0, 0.2), yaw=180.0)
    scene.add_field_sensor("near", lidar="gate")
    scene.add_scenario("blocked", faults=[bt.io.stuck("near", True)])
    sq = scene.sequence("sweep")
    sq.step("hold", transition=bt.seq.elapsed(0.1))
    assert sq.simulate().signal("near").value_at(0.05) is False
    assert sq.simulate(scenario="blocked").signal("near").value_at(0.05) is True


def test_field_round_trip_and_codegen_executes(scene, tmp_path) -> None:
    scene.add_lidar("gate", position=(1.5, 0.0, 0.2), yaw=180.0, fov=270.0)
    scene.add_field_sensor(
        "warn", lidar="gate", range=2.5, sector=(-60.0, 60.0), shadowing=False, watch=["ghost"]
    )
    scene.add_camera("cam", position=(1.5, 0, 0.3), look_at=(0, 0, 0.3))
    scene.add_vision_sensor("seen", camera="cam", watch=["ghost"])
    path = tmp_path / "cell.botrail"
    scene.save_project(path)
    reloaded = bt.Scene.load_project(path)
    assert "warn" in reloaded.sensor_names
    code = scene.generate_python()
    assert 'scene.add_field_sensor("warn", lidar="gate"' in code
    assert "range=2.5" in code and "sector=(-60, 60)" in code and "shadowing=False" in code, code
    # The generated script runs top to bottom: the optics (camera, lidar)
    # are emitted before the sensors judging through them.
    namespace: dict = {}
    exec(code.replace("bt.studio(scene)", ""), namespace)  # noqa: S102 — our own generated code
    rebuilt = namespace["scene"]
    assert "warn" in rebuilt.sensor_names and "seen" in rebuilt.sensor_names
    assert rebuilt.lidar_names == ["gate"]


def test_field_validation(scene) -> None:
    with pytest.raises(ValueError):
        scene.add_field_sensor("bad", lidar="nope")
    scene.add_lidar("gate", position=(1, 0, 0.2), fov=270.0, range=(0.05, 5.0))
    with pytest.raises(ValueError):
        scene.add_field_sensor("bad", lidar="gate", range=0.0)
    with pytest.raises(ValueError):
        scene.add_field_sensor("bad", lidar="gate", range=6.0)
    with pytest.raises(ValueError):
        scene.add_field_sensor("bad", lidar="gate", sector=(30.0, 30.0))
    with pytest.raises(ValueError):
        scene.add_field_sensor("bad", lidar="gate", sector=(-30.0, 150.0))
    scene.add_field_sensor("warn", lidar="gate")
    # The lidar a field sweeps through cannot be removed out from under it.
    with pytest.raises(ValueError, match="swept by field sensor"):
        scene.remove_lidar("gate")
    scene.remove_sensor("warn")
    scene.remove_lidar("gate")

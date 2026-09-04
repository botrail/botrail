"""`examples/machining/two_machine_cell_demo.py` — one arm, two machining
centres (T4 of design/design-machine-tending.md). The bake takes a couple
of minutes (two full swaps taught and rolled), so it runs once for the
module; the catalog products skip where the catalog is unreachable."""

import sys
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "machining"))

import two_machine_cell_demo as two


@pytest.fixture(scope="module")
def cell():
    try:
        return two.bake()
    except Exception as err:
        if any(word in str(err).lower() for word in ("catalog", "fetch", "resolve")):
            pytest.skip(f"catalog unavailable: {err}")
        raise


def test_one_arm_serves_both_machines_in_turn(cell) -> None:
    scene, hs, tl = cell
    assert tl.sequences == ["tend", "vmc_a", "vmc_b"]
    # Machine A is served first, then B: every press on A before the first
    # on B, and each machine's start pressed with its own door confirmed
    # shut. Both are cutting again when the arm parks.
    a_start = tl.signal("vmc_a/panel/cycle_start").high_spans()
    b_unclamp = tl.signal("vmc_b/panel/unclamp").high_spans()
    assert len(a_start) == 1 and len(b_unclamp) == 1 and a_start[0][1] <= b_unclamp[0][0]
    for tag in ("a", "b"):
        (t0, _), = tl.signal(f"vmc_{tag}/panel/cycle_start").high_spans()
        assert tl.signal(f"vmc_{tag}/side_door/closed").value_at(t0)
        assert tl.signal(hs[tag].signal("running")).value_at(tl.duration)
        assert tl.signal(f"vmc_{tag}/panel/estop").high_spans() == []
    # The parts changed places at both machines.
    for tag in ("a", "b"):
        (ox, oy, _), _ = scene.frame(f"stocker_{tag}/out")
        p, _ = tl.object_pose(f"finished_{tag}", tl.duration)
        assert (p[0], p[1]) == pytest.approx((ox, oy), abs=1e-3)
        (jx, jy, _), _ = scene.frame(f"vise_{tag}/jaw")
        p, _ = tl.object_pose(f"blank_{tag}", tl.duration)
        assert (p[0], p[1]) == pytest.approx((jx, jy), abs=1e-3)
    # With a part program longer than the swap, the arm is the constraint:
    # busy most of the cycle, and the first machine cutting most of it.
    assert tl.utilization(two.ROBOT) > 0.5
    # Nothing closer than the jaws around a blank — the far machine's
    # door leaf included, which the tucked gripper now clears.
    assert float(tl.min_clearance()) > 0.0015
    assert two.running_fraction(tl, hs["a"].signal("running")) > 0.5
    assert two.running_fraction(tl, hs["b"].signal("running")) > 0.25
    assert tl.duration < 400.0


def test_the_documents_carry_three_programs_and_two_controllers(cell, tmp_path: Path) -> None:
    scene, hs, tl = cell
    # Three programs on the interlock table, each machine's start guarded
    # by its own switches; two CNCs and the arm's controller in the PLCopen
    # configuration; both handshakes on the spec.
    rows = {(r["program"], r["step"]): r for r in scene.interlocks().rows}
    for tag in ("a", "b"):
        start = rows[(f"vmc_{tag}", "cycle_start")]
        assert start["host"] == f"vmc_{tag}/cnc"
        assert start["condition"] == (
            f"(RISING(vmc_{tag}/panel/cycle_start) AND vmc_{tag}/side_door/closed AND "
            f"vmc_{tag}/front_door/closed AND NOT vmc_{tag}/panel/estop)")
    assert rows[("tend", "a_to_unclamp")]["condition"] == "NOT vmc_a/running"
    assert rows[("tend", "b_to_unclamp")]["inputs"][0]["written_by"] == ["vmc_b/machining", "vmc_b/done", "vmc_b/cycle_start"]
    xml = scene.plcopen()
    for resource in ("arm", "vmc_a_cnc", "vmc_b_cnc"):
        assert f'<resource name="{resource}">' in xml
    spec = tl.handshake_spec()
    assert "vmc_a/running" in spec and "vmc_b/running" in spec
    report = scene.cell_report({"cycle": tl}, title="two machine cell")
    assert [m["name"] for m in report.machines] == ["vmc_a", "vmc_b"]
    assert all(m["controller"] == f"{m['name']}/cnc" for m in report.machines)

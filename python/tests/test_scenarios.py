"""Scenario management (S11): named initial-state deltas, the scenario
sweep, and branch coverage over the set."""

from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def cell() -> bt.Scene:
    """A judged cell: one branch reads an internal signal, one reads a
    zone sensor — so scenarios can steer branches through both a signal
    delta and an obstacle-pose delta."""
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.add_box("part", size=(0.06, 0.06, 0.06), position=(0.6, 0.0, 0.03))
    scene.add_zone_sensor(
        "at_gate",
        position=(0.3, 0.0, 0.05),
        size=(0.2, 0.2, 0.2),
        watch=["part"],
    )
    scene.define_signal("ok", True)

    sq = scene.sequence("s")
    judge = sq.select("judge")
    judge.when(bt.seq.signal("ok")).step("pass")
    judge.when(bt.seq.otherwise()).step("reject")
    gate = sq.select("gate")
    gate.when(bt.seq.signal("at_gate")).step("present")
    gate.when(bt.seq.otherwise()).step("absent")
    return scene


def test_scenarios_steer_branches(cell: bt.Scene) -> None:
    cell.add_scenario("ng", signals={"ok": False})
    # Both pose forms: a bare position and a (position, quaternion) pair.
    cell.add_scenario("arrived", obstacles={"part": (0.3, 0.0, 0.05)})
    cell.add_scenario(
        "arrived_pair",
        obstacles={"part": ((0.3, 0.0, 0.05), (0.0, 0.0, 0.0, 1.0))},
    )
    assert cell.scenario_names == ["ng", "arrived", "arrived_pair"]
    with pytest.raises(ValueError, match="reserved"):
        cell.add_scenario("baseline")

    tl = cell.simulate_sequence("s", scenario="ng")
    assert tl.scenario == "ng"
    assert tl.branches == [("s", "judge", 1), ("s", "gate", 1)]

    # The live scene is untouched: without a scenario, `ok` still holds.
    tl = cell.simulate_sequence("s")
    assert tl.scenario is None
    assert tl.branches[0] == ("s", "judge", 0)

    for scenario in ("arrived", "arrived_pair"):
        tl = cell.simulate_sequence("s", scenario=scenario)
        assert tl.branches[1] == ("s", "gate", 0), scenario


def test_scenario_sweep_and_coverage(cell: bt.Scene) -> None:
    runs = cell.simulate_scenarios(["s"])
    # Baseline alone: two arms never run, named with their guards.
    assert runs.names == ["baseline"]
    assert runs["baseline"].scenario is None
    uncovered = runs.uncovered_arms()
    assert [(step, arm) for _, step, arm, _ in uncovered] == [("judge", 1), ("gate", 0)]
    assert 'bt.seq.signal("at_gate", True)' in uncovered[1][3]

    cell.add_scenario("ng", signals={"ok": False})
    cell.add_scenario("arrived", obstacles={"part": (0.3, 0.0, 0.05)})
    runs = cell.simulate_scenarios(["s"])
    assert runs.names == ["baseline", "ng", "arrived"]
    assert len(runs) == 3 and "ng" in runs
    assert runs.errors == {}
    # The set drains the coverage report — the CI assertion.
    assert runs.uncovered_arms() == []
    assert set(runs.durations) == {"baseline", "ng", "arrived"}
    clearances = runs.min_clearances()
    assert set(clearances) == {"baseline", "ng", "arrived"}
    assert all(c.distance >= 0.0 for c in clearances.values())
    assert [name for name, _ in runs.items()] == runs.names

    # An explicit subset runs exactly that, in order.
    runs = cell.simulate_scenarios(["s"], scenarios=["arrived", "baseline"])
    assert runs.names == ["arrived", "baseline"]
    with pytest.raises(ValueError, match="listed twice"):
        cell.simulate_scenarios(["s"], scenarios=["ng", "ng"])


def test_scenario_failures_are_collected(cell: bt.Scene) -> None:
    cell.add_scenario("broken", signals={"ghost": True})
    runs = cell.simulate_scenarios(["s"])
    assert runs.names == ["baseline"]
    assert "ghost" in runs.errors["broken"]
    with pytest.raises(KeyError, match="failed"):
        runs["broken"]
    with pytest.raises(KeyError, match="unknown scenario"):
        runs["nope"]

    cell.remove_scenario("broken")
    assert cell.scenario_names == []
    with pytest.raises(ValueError, match="unknown scenario"):
        cell.remove_scenario("broken")


def test_merged_script_carries_both_arms(tmp_path: Path) -> None:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.define_signal("ok", True)
    scene.add_segment("approach", goal=[0.6, 0.5, -0.8, 0.2, 0.0, 0.0])
    scene.add_segment("place", goal=[0.9, 0.85, -1.1, 0.25, 0.0, 0.0])
    scene.add_segment("rework", goal=[-0.9, 0.85, -1.1, 0.25, 0.0, 0.0])
    scene.add_segment("home", goal=[0.0] * 6)

    sq = scene.sequence("qc")
    sq.step("approach", actions=[bt.seq.motion("approach")])
    judge = sq.select("judge")
    judge.when(bt.seq.signal("ok")).step("place", actions=[bt.seq.motion("place")])
    judge.when(bt.seq.otherwise()).step("rework", actions=[bt.seq.motion("rework")])
    sq.step("home", actions=[bt.seq.motion("home")])

    scene.add_scenario("ng", signals={"ok": False})
    runs = scene.simulate_scenarios(["qc"])
    assert runs.uncovered_arms() == []

    # One bake alone cannot carry the arm it skipped.
    with pytest.raises(ValueError, match="never planned"):
        runs["baseline"].to_script(inputs={"ok": 1})

    # The sweep merges into one program with both arms' real moves.
    code = runs.to_script(inputs={"ok": 1})
    body = code[code.index("if (get_standard_digital_in(1)):") :]
    if_body = body[: body.index("elif")]
    elif_body = body[body.index("elif") :]
    assert "movej([0.9," in if_body  # the place goal, from the baseline
    assert "movej([-0.9," in elif_body  # the rework goal, from `ng`
    # The primary can be swapped; the arms stay each bake's own.
    swapped = runs.to_script(inputs={"ok": 1}, primary="ng")
    assert "movej([-0.9," in swapped[swapped.index("elif") :]
    with pytest.raises(ValueError, match="unknown primary"):
        runs.to_script(inputs={"ok": 1}, primary="ghost")

    path = tmp_path / "qc.script"
    runs.export_script(path, inputs={"ok": 1})
    assert path.read_text().startswith("def qc():")


def test_builder_simulate_takes_a_scenario(cell: bt.Scene) -> None:
    cell.add_scenario("ng", signals={"ok": False})
    # `scene.sequence` restarts a sequence; author a fresh one and run it
    # under the scenario through the builder sugar.
    sq = cell.sequence("t")
    sel = sq.select("judge")
    sel.when(bt.seq.signal("ok")).step("pass")
    sel.when(bt.seq.otherwise()).step("reject")
    tl = sq.simulate(scenario="ng")
    assert tl.scenario == "ng"
    assert tl.branches == [("t", "judge", 1)]

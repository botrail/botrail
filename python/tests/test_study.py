"""`bt.sweep` / `bt.optimize` — parameter studies as one call (D6 of
design/design-cell-engineering.md).

A cell authored as a function of its parameters is baked at every point of
a grid (or searched over one), deterministically, and read as a table.
These tests pin the table's shape and order, the failed-row behaviour, the
best / Pareto / pivot readers, the two optimisers agreeing, and that a
parallel sweep gives byte-identical rows.
"""

import json
import sys
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "basics"))

import sweep_demo as sd  # noqa: E402


def metrics(tl):
    return {"cycle": tl.duration, "feed": tl.step_span("feed").duration, "clearance": float(tl.min_clearance())}


def test_sweep_tables_the_grid_in_order_and_keeps_failed_rows() -> None:
    result = bt.sweep(
        sd.build_cell,
        grid={"velocity": [0.1, 0.3], "lane_y": [0.6, 0.4, 0.05]},
        metrics=metrics,
        sequence="cycle",
    )
    assert repr(result).startswith("Sweep(6 rows, 2 failed")
    assert result.params == ["velocity", "lane_y"] and result.metrics == ["cycle", "feed", "clearance"]
    # Grid order: the last parameter varies fastest.
    assert [(r["velocity"], r["lane_y"]) for r in result] == [
        (0.1, 0.6), (0.1, 0.4), (0.1, 0.05), (0.3, 0.6), (0.3, 0.4), (0.3, 0.05),
    ]
    # The rows that baked carry the metrics; the lane through the robot
    # is a row with the planner's reason, not an exception.
    ok = result.ok
    assert len(ok) == 4 and all(r["ok"] and r["error"] is None for r in ok)
    assert ok[0]["cycle"] == pytest.approx(10.30, abs=0.01) and ok[0]["clearance"] == pytest.approx(0.53, abs=0.01)
    failed = result.failed
    assert [r["lane_y"] for r in failed] == [0.05, 0.05]
    assert "planning failed" in failed[0]["error"] and "cycle" not in failed[0]
    # Readers.
    best = result.best("cycle")
    assert (best["velocity"], best["lane_y"]) == (0.3, 0.6)
    assert result.best("clearance", minimize=False)["clearance"] == pytest.approx(0.53, abs=0.01)
    assert result.best("cycle", where=lambda r: r["velocity"] < 0.2)["velocity"] == 0.1
    assert result.best("cycle", where=lambda r: r["velocity"] > 1) is None
    narrow = result.where(lambda r: r["lane_y"] >= 0.5)
    assert len(narrow) == 2 and all(r["lane_y"] == 0.6 for r in narrow)
    front = result.pareto(minimize=["cycle"], maximize=["clearance"])
    assert (front[-1]["velocity"], front[-1]["lane_y"]) == (0.3, 0.6)
    assert all(not (o["cycle"] < r["cycle"] and o["clearance"] > r["clearance"]) for r in front for o in result.ok)
    with pytest.raises(ValueError, match="at least one metric"):
        result.pareto()
    # The pivot: parameters as written, metrics at table precision, failed
    # cells marked.
    pivot = result.pivot("lane_y", "velocity", "cycle")
    assert pivot.splitlines()[0] == "| lane_y \\ velocity | 0.1 | 0.3 |"
    assert "| 0.6 | 10.30 | 7.13 |" in pivot and "| 0.05 | — | — |" in pivot
    # Renderings.
    md = result.to_markdown()
    assert md.splitlines()[0] == "| velocity | lane_y | cycle | feed | clearance | ok | error |"
    assert "| 0.1 | 0.6 | 10.30 | 4.76 | 0.530 | True |  |" in md
    csv = result.to_csv()
    assert csv.splitlines()[0] == "velocity,lane_y,cycle,feed,clearance,ok,error"
    assert csv.splitlines()[3].startswith("0.1,0.05,,,,false,ValueError")
    doc = json.loads(result.to_json())
    assert doc["params"] == ["velocity", "lane_y"] and len(doc["rows"]) == 6
    # An all-good sweep hides the ok / error columns in Markdown only.
    good = bt.sweep(sd.build_cell, grid={"velocity": [0.2], "lane_y": [0.6]}, metrics=metrics, sequence="cycle")
    assert good.to_markdown().splitlines()[0] == "| velocity | lane_y | cycle | feed | clearance |"
    assert good.to_csv().splitlines()[0].endswith(",ok,error")


def test_sweep_points_default_metrics_and_errors(tmp_path: Path) -> None:
    # Explicit points, the default metric, a scene with one sequence
    # needs no `sequence=`.
    result = bt.sweep(sd.build_cell, points=[{"velocity": 0.2, "lane_y": 0.6}, {"velocity": 0.3, "lane_y": 0.6}])
    assert result.metrics == ["cycle"] and [round(r["cycle"], 2) for r in result] == [7.92, 7.13]
    # A build that hands back a timeline is used as is.
    result2 = bt.sweep(lambda v: sd.build_cell(v, 0.6).simulate_sequence("cycle"), grid={"v": [0.2]})
    assert round(result2.rows[0]["cycle"], 2) == 7.92
    # A metrics function that misbehaves is a failed row, not a crash.
    bad = bt.sweep(sd.build_cell, grid={"velocity": [0.2], "lane_y": [0.6]}, metrics=lambda tl: 3, sequence="cycle")
    assert not bad.rows[0]["ok"] and "TypeError" in bad.rows[0]["error"]
    result.save(tmp_path / "s.csv")
    result.save(tmp_path / "s.md")
    result.save(tmp_path / "s.json")
    assert (tmp_path / "s.csv").read_text() == result.to_csv()
    assert json.loads((tmp_path / "s.json").read_text())["rows"] == result.rows
    with pytest.raises(ValueError, match="unknown format"):
        result.save(tmp_path / "s.xlsx")
    with pytest.raises(ValueError, match="exactly one of"):
        bt.sweep(sd.build_cell)
    with pytest.raises(ValueError, match="grid is empty"):
        bt.sweep(sd.build_cell, grid={"velocity": []})
    with pytest.raises(ValueError, match="same parameters"):
        bt.sweep(sd.build_cell, points=[{"velocity": 0.2, "lane_y": 0.6}, {"lane_y": 0.6}])


def test_optimize_grid_and_descent_agree_and_report_their_search() -> None:
    space = {"velocity": (0.1, 0.4, 0.05), "lane_y": (0.3, 0.7, 0.05)}
    grid = bt.optimize(sd.build_cell, space=space, objective="cycle", constraints={"clearance": (">=", 0.4)},
                       metrics=metrics, sequence="cycle", method="grid")
    assert grid.ok and grid.method == "grid" and len(grid.evaluated) == 7 * 9
    assert grid.params == {"velocity": 0.4, "lane_y": 0.5}
    assert grid.row["cycle"] == pytest.approx(6.73, abs=0.01) and grid.row["clearance"] >= 0.4
    descent = bt.optimize(sd.build_cell, space=space, objective="cycle", constraints={"clearance": (">=", 0.4)},
                          metrics=metrics, sequence="cycle", method="descent")
    assert descent.ok and descent.params == grid.params
    assert len(descent.evaluated) < len(grid.evaluated) and descent.iterations >= 1
    # The search is a table too, in evaluation order, starting from the
    # middle of the space.
    first = descent.evaluated.rows[0]
    assert (first["velocity"], first["lane_y"]) == (0.25, 0.5)
    doc = json.loads(descent.to_json())
    assert doc["ok"] and doc["params"] == descent.params and doc["evaluated"] == len(descent.evaluated)
    assert repr(descent).startswith("Optimum({'velocity': 0.4, 'lane_y': 0.5} → cycle=6.73")
    # A callable objective and constraint, maximisation, a start point.
    widest = bt.optimize(sd.build_cell, space=space, objective=lambda r: r["clearance"], minimize=False,
                         constraints=lambda r: r["cycle"] <= 7.5, metrics=metrics, sequence="cycle",
                         method="descent", start={"velocity": 0.3, "lane_y": 0.4})
    assert widest.ok and widest.params["lane_y"] == 0.7 and widest.row["cycle"] <= 7.5
    # Nothing feasible: an honest none, with the rows it tried.
    none = bt.optimize(sd.build_cell, space={"velocity": [0.2], "lane_y": [0.6]}, objective="cycle",
                       constraints={"clearance": (">=", 5.0)}, metrics=metrics, sequence="cycle")
    assert not none.ok and none.params is None and len(none.evaluated) == 1
    assert repr(none).startswith("Optimum(none feasible")
    # An infeasible start walks the grid until a feasible point, then descends.
    walk = bt.optimize(sd.build_cell, space={"velocity": [0.2, 0.3], "lane_y": [0.05, 0.6]}, objective="cycle",
                       metrics=metrics, sequence="cycle", method="descent", start={"velocity": 0.2, "lane_y": 0.05})
    assert walk.ok and walk.params == {"velocity": 0.3, "lane_y": 0.6}
    with pytest.raises(ValueError, match="unknown method"):
        bt.optimize(sd.build_cell, space=space, method="anneal")
    with pytest.raises(ValueError, match="step > 0"):
        bt.optimize(sd.build_cell, space={"velocity": (0.1, 0.4, 0.0)})
    with pytest.raises(ValueError, match="unknown constraint operator"):
        bt.optimize(sd.build_cell, space={"velocity": [0.2], "lane_y": [0.6]}, constraints={"cycle": ("~", 1)})


def test_parallel_sweep_gives_the_same_rows() -> None:
    grid = {"velocity": [0.1, 0.2], "lane_y": [0.6, 0.4]}
    serial = bt.sweep(sd.build_cell, grid=grid, metrics=metrics, sequence="cycle")
    parallel = bt.sweep(sd.build_cell, grid=grid, metrics=metrics, sequence="cycle", workers=2)
    assert parallel.rows == serial.rows

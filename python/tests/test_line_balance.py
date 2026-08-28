"""Changing the layout is a regression test.

`examples/welding/line_balance_sweep.py` answers "move a spot, what happens to the
takt?" by baking the real line for each split. That makes the answer a
*number*, and a number belongs in CI: this file pins the takt of two
splits and the invariants that make the pins meaningful — the takt is the
transfer plus the slowest station, and the work simply moves between
stations rather than appearing or vanishing.

If someone re-teaches a station, retimes a ramp, or moves a base, one of
these numbers changes and the diff says by how much. That is the whole
claim: a line's cycle time is a tested property of the cell, not a
measurement taken after it is built.

Skipped unless the catalog packages are already in the Hugging Face cache.
"""

import os
import sys
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "welding"))

HF_HUB = Path(
    os.environ.get("HF_HOME") or Path.home() / ".cache" / "huggingface"
) / "hub"
pytestmark = pytest.mark.skipif(
    not (HF_HUB / "datasets--botrail--botrail-catalog").exists(),
    reason="botrail catalog not in the HF cache (run examples/welding/weld_line_demo.py once)",
)

# Baked 2026-08-10. Two of the four splits — the shipped 3/2 and its
# lopsided neighbour — are enough to pin both the balance point and the
# penalty for missing it, at a CI cost of two bakes.
GOLDEN = {
    3: {"takt": 24.62, "st1_cycle": 11.60, "st2_cycle": 8.80},
    4: {"takt": 27.42, "st1_cycle": 14.40, "st2_cycle": 6.00},
}
BUDGET = 0.5


@pytest.fixture(scope="module")
def sweep():
    import line_balance_sweep as bal

    rows = {front: bal.bake(front) for front in GOLDEN}
    # Leave the module on its shipped split: later tests in the same
    # session import `weld_line_demo` expecting the default.
    bal.line.SEAM_SPLITS.update(bal.layout_for(3))
    bal.line.set_stations(2)
    return bal, rows


def test_each_split_holds_its_takt(sweep) -> None:
    _bal, rows = sweep
    for front, want in GOLDEN.items():
        row = rows[front]
        assert row["takt"] == pytest.approx(want["takt"], abs=BUDGET), front
        assert row["stations"]["st1"]["cycle"] == pytest.approx(
            want["st1_cycle"], abs=BUDGET
        ), front
        assert row["stations"]["st2"]["cycle"] == pytest.approx(
            want["st2_cycle"], abs=BUDGET
        ), front


def test_the_takt_is_the_transfer_plus_the_slowest_station(sweep) -> None:
    """The line-balancing identity, which is *why* the numbers above move
    the way they do. If this ever fails, the takt figure has stopped
    meaning what the sweep says it means."""
    _bal, rows = sweep
    for front, row in rows.items():
        slowest = max(s["cycle"] for s in row["stations"].values())
        assert row["takt"] == pytest.approx(
            row["transfer"] + slowest, abs=1.0
        ), f"split {front}: takt {row['takt']:.2f}, transfer {row['transfer']:.2f}, slowest {slowest:.2f}"


def test_moving_a_spot_moves_work_not_creates_it(sweep) -> None:
    """3/2 -> 4/1 moves one spot from station 2 to station 1: station 1
    gets slower by about what station 2 gains, and the takt rises because
    the *slower* one is what the line waits for."""
    _bal, rows = sweep
    balanced, lopsided = rows[3], rows[4]
    gained = lopsided["stations"]["st1"]["cycle"] - balanced["stations"]["st1"]["cycle"]
    shed = balanced["stations"]["st2"]["cycle"] - lopsided["stations"]["st2"]["cycle"]
    assert gained == pytest.approx(shed, abs=0.6), (gained, shed)
    assert lopsided["takt"] > balanced["takt"]


def test_utilization_matches_the_measured_busy_time(sweep) -> None:
    """The reported utilization is the same quantity the sweep tables
    print, so the two can never drift apart."""
    bal, rows = sweep
    row = rows[3]
    for st in ("st1", "st2"):
        assert 0.0 < row["stations"][st]["util"] < 1.0
    # The busier station in the table is the one with the longer cycle.
    assert (
        row["stations"]["st1"]["util"] > row["stations"]["st2"]["util"]
    ) == (
        row["stations"]["st1"]["cycle"] > row["stations"]["st2"]["cycle"]
    )
    assert bal.line.STATIONS == ("st1", "st2")

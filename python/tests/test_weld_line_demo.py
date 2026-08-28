"""The two-station line, asserted the way a line owner would assert it.

`examples/welding/weld_line_demo.py` is the W1 build of design/design-weld-line.md:
three programs (two stations and a transfer) advancing over one world via
`simulate_sequences`, an indexed transfer moved by `advance` (a distance,
not a timer), and bodies pipelining through both stations. This pins the
properties that make it a line: stations weld *different bodies at the same
time*, every body lands on every station to numerical precision, the takt
is the slowest station plus the transfer — and removing the transfer gate
drags a body out of closed weld guns, which only the frame sweep catches.

Skipped unless the catalog packages are already in the Hugging Face cache
(running the example once fetches them).
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

# First pinned 2026-08-09, on the first clean W1 bake (park/slide
# corridors, all-ramp cycle, part-present gated transfer).
GOLDEN_TOTAL = 108.70
TOTAL_BUDGET = 1.0


@pytest.fixture(scope="module")
def baked():
    import weld_line_demo as line

    scene, ln, riders = line.build_line()
    poses = line.teach(scene, ln, riders)
    programs = [line.build_station_program(scene, st, poses)
                for st in line.STATIONS]
    programs.append(line.build_transfer_program(scene, riders))
    timeline = scene.simulate_sequences(programs, max_duration=400.0)
    return line, scene, ln, riders, timeline


def spans(timeline) -> dict:
    return {step: (start, end) for step, start, end in timeline.step_spans}


def witness(riders: dict, body: int) -> str:
    """One mesh rider of body `body` (they all share the body origin)."""
    return next(p for p, o in riders[body] if o[0] == 0.0 and o[1] == 0.0
                and not p.endswith(("/shell", "/skid")))


def test_the_line_bakes_one_deterministic_takt(baked) -> None:
    line, scene, _ln, _riders, timeline = baked
    assert sorted(timeline.robots) == sorted(line.ARMS)
    assert timeline.duration == pytest.approx(GOLDEN_TOTAL, abs=TOTAL_BUDGET)

    again = scene.simulate_sequences(
        [*line.STATIONS, "transfer"], max_duration=400.0)
    assert again.duration == timeline.duration
    for robot in timeline.robots:
        assert (
            again.robot_trajectory(robot).positions
            == timeline.robot_trajectory(robot).positions
        )


def test_step_spans_carry_their_program(baked) -> None:
    """Three programs share one timeline; the qualified names are how a
    step stays attributable ('feed' alone would name nobody)."""
    line, _scene, _ln, _riders, timeline = baked
    prefixes = {s.split("/", 1)[0] for s, _, _ in timeline.step_spans}
    assert prefixes == {*line.STATIONS, "transfer"}


def test_every_body_lands_on_its_station_exactly(baked) -> None:
    """`advance` is the point: the pitch is a distance, so a body lands on
    a station datum to numerical precision — not one scan short of it.
    (The old timer pattern needed `elapsed(pitch/v + one tick)`, and the
    tolerance here was 1e-6 instead of 1e-9.)"""
    line, _scene, _ln, riders, timeline = baked
    at = spans(timeline)
    for k, st in enumerate(line.STATIONS, start=1):
        for body in range(1, line.BODIES + 1):
            t_enter = at[f"{st}/b{body}_slide_in"][0]
            got = timeline.object_pose(witness(riders, body), t_enter)[0]
            assert got[0] == pytest.approx(line.ST_X[st], abs=1e-9), (
                f"body {body} at {st}: x = {got[0]!r}"
            )
            assert timeline.object_visible(witness(riders, body), t_enter)


def test_stations_weld_different_bodies_at_the_same_time(baked) -> None:
    """The E1 property. While station 1 welds body b, station 2 welds
    body b-1 — concurrency a single serial sequence cannot express. The
    squeeze windows of st1/b2 and st2/b1 must literally overlap."""
    line, _scene, _ln, _riders, timeline = baked
    at = spans(timeline)
    overlap_total = line.overlap(
        line.busy_windows(timeline, [f"st1_{s}" for s in line.SIDES]),
        line.busy_windows(timeline, [f"st2_{s}" for s in line.SIDES]),
    )
    assert overlap_total > 5.0, f"stations overlapped {overlap_total:.2f}s"

    for body in range(2, line.BODIES + 1):
        a0, a1 = at[f"st1/b{body}_s1_travel"][0], at[f"st1/b{body}_traverse"][1]
        b0, b1 = at[f"st2/b{body - 1}_s1_travel"][0], at[f"st2/b{body - 1}_traverse"][1]
        assert min(a1, b1) - max(a0, b0) > 1.0, (
            f"st1 on body {body} ({a0:.1f}-{a1:.1f}s) never overlapped "
            f"st2 on body {body - 1} ({b0:.1f}-{b1:.1f}s)"
        )


def test_the_takt_is_the_slowest_station_plus_the_transfer(baked) -> None:
    """Line balancing in one assertion: at steady state, one pitch-to-pitch
    period equals the transfer plus the slowest station's cycle (the other
    station hides inside it)."""
    line, _scene, _ln, _riders, timeline = baked
    at = spans(timeline)
    takt = at["transfer/p4_landed"][1] - at["transfer/p3_landed"][1]
    advance_len = at["transfer/p4_index"][1] - at["transfer/p4_index"][0]
    station = {
        st: at[f"{st}/b2_report"][0] - at[f"{st}/b2_slide_in"][0]
        for st in line.STATIONS
    }
    slowest = max(station.values())
    assert takt == pytest.approx(advance_len + slowest, abs=1.0), (
        f"takt {takt:.2f}s, transfer {advance_len:.2f}s, stations {station}"
    )


def test_no_gun_ever_touches_a_body(baked) -> None:
    """Same property as the station demo, now with bodies in motion between
    two working stations. The rollout only reports robot-robot pairs, so
    the frame sweep is the check that counts."""
    line, scene, _ln, riders, timeline = baked
    offences = line.sweep_for_contact(scene, riders, timeline)
    assert not offences, "gun through a body: " + "; ".join(
        f"{a} x {b} at {t[0]}s ({len(t)} frames)"
        for (a, b), t in offences.items()
    )


def test_the_transfer_gate_is_load_bearing(baked) -> None:
    """Drop the gate and the belt indexes under the welding: the body
    slides out from between the electrodes (untouched — the squeeze is
    taught 5 mm off the sheet) and the spots land on air, metres past the
    datum. The rollout reports nothing, every pose is collision-free, and
    the line is simply not welding the car — which is why the datum
    assertion owns this failure mode, and what the gate is for."""
    line, *_ = baked

    scene, ln, riders = line.build_line()
    poses = line.teach(scene, ln, riders)
    programs = [line.build_station_program(scene, st, poses, bodies=1)
                for st in line.STATIONS]
    programs.append(line.build_transfer_program(scene, riders, gated=False))
    timeline = scene.simulate_sequences(programs, max_duration=400.0)
    at = spans(timeline)
    start, end = at["st1/b1_s1_weld"]
    got = timeline.object_pose(witness(riders, 1), (start + end) / 2)[0]
    off_datum = abs(got[0] - line.ST_X["st1"])
    assert off_datum > 0.5, (
        f"expected the ungated weld to fire metres off datum, got {off_datum:.3f} m"
    )


def test_bodies_pipeline_head_to_sink(baked) -> None:
    """Body b enters on load b, is gone by the end, and no body is ever
    drawn inside the line before its load."""
    line, _scene, _ln, riders, timeline = baked
    at = spans(timeline)
    for body in range(1, line.BODIES + 1):
        name = witness(riders, body)
        load = at[f"transfer/p{body}_load"][0]
        if load > 0.0:
            assert not timeline.object_visible(name, max(0.0, load - 0.05))
        assert timeline.object_visible(name, load + 0.5)
        assert not timeline.object_visible(name, timeline.duration), (
            f"body {body} still on the line at the end"
        )


def test_exports_usd(baked, tmp_path: Path) -> None:
    line, _scene, _ln, _riders, timeline = baked
    out = tmp_path / "line.usda"
    assert timeline.export_usd(out, fps=30.0) == []
    text = out.read_text()
    for name in line.ARMS:
        assert f'def Xform "{name}"' in text, name

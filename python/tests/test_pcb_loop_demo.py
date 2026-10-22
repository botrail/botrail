"""Material conservation, process order, fixture interlock and runtime sorting.

Build the optional product packages with build_pcb_loop_catalog.py first.
"""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "examples/assembly"))
import pcb_loop_demo as demo


@pytest.fixture(scope="module")
def batch():
    root = (
        demo.default_catalog_root()
        or demo.HERE.parents[2] / "botrail-catalog-builder/build"
    )
    required = (
        demo.KUKA,
        demo.eq.CURVE,
        demo.eq.STRAIGHT,
        demo.eq.SHUTTLE,
        demo.UR5,
        demo.UR3,
        demo.tooling.PCB_CUP,
    )
    missing = [pid for pid in required if not (root / pid / "manifest.yaml").is_file()]
    if missing:
        pytest.skip(
            "PCB cell catalog packages not built locally: " + ", ".join(missing)
        )
    return demo.bake(root)


def test_path_has_no_jump_in_either_direction():
    for sections in (1, 3):
        loop = demo.eq.Loop(sections=sections)
        for d in loop.cuts(-2 * loop.lap, 2 * loop.lap):
            assert math.dist(loop.pose(d - 1e-7)[0], loop.pose(d + 1e-7)[0]) < 3e-7
        assert loop.pose(0)[0] == pytest.approx(loop.pose(-loop.lap)[0], abs=1e-9)
        assert loop.lap == pytest.approx(2 * sections + 2 * math.pi * 0.650)


def test_every_board_flows_from_magazine_through_both_tests_to_output(batch):
    scene, tl, info = batch
    assert not scene.check_collisions()
    assert tl.signal("batch_done").value_at(tl.duration)
    assert not any(
        tl.signal(f"carrier{i:02d}_occupied").value_at(tl.duration) for i in range(8)
    )
    for i, members in enumerate(info["boards"]):
        board = members[0]
        assert tl.object_pose(board, 0)[0] == pytest.approx(
            info["equipment"]["magazine"][i][0]
        )
        assert not tl.signal(f"pcb{i:02d}_loaded").value_at(0)
        feed = tl.step_span(f"deposit{i}")
        aoi = tl.step_span(f"inspect{i + 3}")
        electrical = tl.step_span(f"inspect{i + 5}")
        result = info["results"][i]["destination"]
        output = tl.step_span(f"sort_{result}_release{i + 7}")
        assert feed.end < aoi.start < electrical.start < output.start
        assert tl.object_pose(board, feed.end)[0] == pytest.approx(
            info["loop"].pose(3 * info["loop"].pitch)[0], abs=2e-5
        )
        assert tl.object_pose(board, (aoi.start + aoi.end) / 2)[0] == pytest.approx(
            (-0.68, -0.19, 1.118), abs=2e-5
        )
        assert tl.object_pose(board, (electrical.start + electrical.end) / 2)[
            0
        ] == pytest.approx(info["equipment"]["fixture"][0], abs=2e-5)
        assert tl.object_pose(board, tl.duration)[0] == pytest.approx(
            info["equipment"]["outputs"][result][i][0], abs=2e-5
        )
        assert tl.object_pose(board, output.end)[0] == pytest.approx(
            tl.object_pose(board, tl.duration)[0]
        )
        # IC0 travels with the same board through both tests and the output rack.
        for span in (aoi, electrical, output):
            t = (span.start + span.end) / 2
            chip = tl.object_pose(f"pcb{i:02d}/ic0", t)[0]
            centre = tl.object_pose(board, t)[0]
            assert math.dist(chip, centre) == pytest.approx(
                math.sqrt(0.040**2 + 0.018**2 + 0.0025**2), abs=1e-5
            )


def test_clamp_tests_stationary_released_board_and_opens_before_retrieval(batch):
    _, tl, info = batch
    for i in range(info["board_count"]):
        cycle = i + 5
        span = tl.step_span(f"inspect{cycle}")
        start, end = span.start + 0.05, span.end - 0.05
        assert tl.signal("fixture_clamped").value_at(start)
        assert tl.signal("electrical_busy").value_at(start)
        assert tl.sample(start, robot="ict_clamp") == pytest.approx([0.029, 0.029])
        board = info["boards"][i][0]
        assert tl.object_pose(board, start) == tl.object_pose(board, end)
        assert (
            tl.step_span(f"clamp_open{cycle}").end
            <= tl.step_span(f"fixture_grasp{cycle}").start
        )
        # Robot has backed away; it is not holding the PCB during the test.
        assert tl.sample(start, robot=demo.ROLES["electrical"]) == pytest.approx(
            info["poses"][demo.ROLES["electrical"]]["hover"]
        )


def test_both_fault_modes_route_to_reject_and_all_other_boards_to_good(batch):
    scene, tl, info = batch
    report = demo.report(scene, tl, info)
    assert (report["good"], report["reject"]) == (6, 2)
    assert report["max_output_position_error_m"] < 2e-5
    assert not tl.signal("pcb02_aoi_ok").value_at(tl.duration)
    assert tl.signal("pcb02_electrical_ok").value_at(tl.duration)
    assert tl.signal("pcb05_aoi_ok").value_at(tl.duration)
    assert not tl.signal("pcb05_electrical_ok").value_at(tl.duration)
    for i in range(8):
        assert tl.signal(f"pcb{i:02d}_complete").value_at(tl.duration)
        assert tl.signal(f"pcb{i:02d}_reject").value_at(tl.duration) == (i in (2, 5))
        assert tl.signal(f"pcb{i:02d}_good").value_at(tl.duration) == (i not in (2, 5))


def test_transport_waits_for_tools_and_preserves_material_count(batch):
    _, tl, info = batch
    loop = info["loop"]
    intervals = tl.signal("indexing").high_spans()
    assert len(intervals) == 14
    for start, end in intervals:
        assert not tl.signal("fixture_clamped").value_at(start)
        assert not tl.signal("aoi_busy").value_at(start)
        assert not tl.signal("electrical_busy").value_at(start)
        for name in demo.STATIONS:
            assert tl.signal(f"{name}_clear").value_at(start)
            assert tl.sample(start, robot=name) == pytest.approx(
                tl.sample(end, robot=name)
            )
        loaded = sum(tl.signal(f"pcb{i:02d}_loaded").value_at(start) for i in range(8))
        completed = sum(
            tl.signal(f"pcb{i:02d}_complete").value_at(start) for i in range(8)
        )
        occupied = sum(
            tl.signal(f"carrier{i:02d}_occupied").value_at(start) for i in range(8)
        )
        assert loaded - completed == occupied
        for t in (start + (end - start) * f / 10 for f in range(11)):
            for mover in info["movers"]:
                slide, angle = tl.sample(t, robot=mover)
                x, y = (
                    -loop.radius * math.cos(angle),
                    slide - loop.radius * math.sin(angle),
                )
                centre_y = max(-loop.length / 2, min(loop.length / 2, y))
                assert math.hypot(x, y - centre_y) == pytest.approx(
                    loop.radius, abs=1e-5
                )


def test_full_batch_retains_positive_clearance(batch):
    _, tl, _ = batch
    clearance = tl.min_clearance(dt=0.025)
    assert clearance > 0.0005, repr(clearance)


def test_one_board_batch_drains_and_sorting_reads_results(batch, monkeypatch):
    # Exercise the same board/carrier with the opposite quality result. Routing
    # must not be a fixed animation selected by the board or cycle index.
    monkeypatch.setattr(demo, "MISSING_ICS", {0: 1})
    scene, tl, info = demo.bake(batch[2]["root"], board_count=1)
    result = demo.report(scene, tl, info)
    assert (
        result["boards"],
        result["good"],
        result["reject"],
        result["index_steps"],
    ) == (1, 0, 1, 7)
    assert result["max_output_position_error_m"] < 2e-5
    assert tl.signal("batch_done").value_at(tl.duration)
    assert not any(
        tl.signal(f"carrier{i:02d}_occupied").value_at(tl.duration) for i in range(8)
    )

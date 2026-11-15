"""What the programs wait on (`tl.waiting(t)`, `tl.waits()`,
`Session.waiting()`), the device-comment table, and the FAT clips — the
data behind the studio's waiting line, its 3D tags and its focus."""

import sys
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "export"))
sys.path.insert(0, str(Path(__file__).resolve().parent))


def pick_cell() -> bt.Scene:
    import export_urscript as pick

    scene = pick.build_cell()
    pick.author_sequence(scene)
    pick.wire_cell(scene)
    return scene


def test_a_program_waits_on_the_beam_by_its_address() -> None:
    scene = pick_cell()
    tl = scene.simulate_sequence("pick")
    await_part = tl.step_span("await part")
    w = tl.waiting((await_part.start + await_part.end) / 2)
    assert [(x["program"], x["step"]) for x in w] == [("pick", "await part")]
    assert w[0]["since"] == pytest.approx(await_part.start)
    (exit,) = w[0]["exits"]
    assert exit["condition"] == "↑part_at_pick"
    assert exit["holds"] is False
    (leaf,) = exit["unmet"]
    assert leaf["kind"] == "rising"
    assert leaf["name"] == "part_at_pick"
    assert leaf["level"] is False
    assert leaf["point"] == "part_at_pick"
    assert "DI2" in leaf["address"]  # bound on the controller's standard DI2


def test_a_timer_says_what_it_has_left_and_a_move_names_itself() -> None:
    scene = pick_cell()
    tl = scene.simulate_sequence("pick")
    grip = tl.step_span("grip")
    (w,) = tl.waiting(grip.start + 0.1)
    (leaf,) = w["exits"][0]["unmet"]
    assert (leaf["kind"], leaf["condition"]) == ("elapsed", "0.30s")
    assert leaf["remaining"] == pytest.approx(0.2)
    descend = tl.step_span("descend")
    (w,) = tl.waiting((descend.start + descend.end) / 2)
    (leaf,) = w["exits"][0]["unmet"]
    assert leaf["condition"] == "done ▶pick_down"
    # Past the cycle nothing waits.
    assert tl.waiting(tl.duration + 1.0) == []


def test_the_waits_tile_the_cycle_with_their_releasing_exits() -> None:
    scene = pick_cell()
    tl = scene.simulate_sequence("pick")
    rows = tl.waits()
    assert [r["step"] for r in rows] == [name for name, _, _ in tl.step_spans]
    assert sum(r["waited"] for r in rows) == pytest.approx(tl.duration, abs=1e-6)
    by_step = {r["step"]: r["released_by"] for r in rows}
    assert by_step["await part"] == "↑part_at_pick"
    assert by_step["grip"] == "0.30s"
    assert by_step["descend"] == "done ▶pick_down"
    # The judge took one arm; its guard is what let the program on.
    assert by_step["judge"] in ("spec_ok=1", "→")


def test_a_pressed_button_released_the_step_that_waited_on_it() -> None:
    """A press lands a scan before the program reads it: the waits still
    credit the button's edge with the release (the studio's afterglow)."""
    from test_vc import panel_cell

    scene = panel_cell()
    with bt.vc.loopback(scene, ["pick"], host="PLC1", studio=False) as s:
        s.run(for_=0.5)
        s.press("panel/cycle_start")
        s.run()
        tl = s.timeline()
    rows = {r["step"]: r for r in tl.waits()}
    assert rows["ready"]["released_by"] == "↑panel/cycle_start"
    assert rows["arrival"]["released_by"] == "↑part_at_pick"
    (w,) = tl.waiting(0.3)
    (leaf,) = w["exits"][0]["unmet"]
    assert (leaf["condition"], leaf["address"]) == ("↑panel/cycle_start", "%IX100.0")


def test_a_cylinder_is_waited_on_through_the_switch_it_was_sent_to() -> None:
    """A coil-driven axis is in position at the limit switch of the stop it
    was last sent to: the press going down is waited on at `press/extended`."""
    import importlib.util

    path = EXAMPLES / "commissioning" / "openplc_feed_line" / "run_demo.py"
    spec = importlib.util.spec_from_file_location("openplc_feed_line_demo", path)
    demo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(demo)
    scene = demo.build()
    tl, _ = demo.reference(scene)
    down = tl.step_span("down2")
    (w,) = tl.waiting((down.start + down.end) / 2)
    press, jam = w["exits"]
    (leaf,) = press["unmet"]
    assert (leaf["condition"], leaf["point"]) == ("press done", "press/extended")
    assert leaf["address"] is not None and leaf["address"].startswith("%IX100.")
    assert jam["unmet"][0]["kind"] == "elapsed"
    # The lift waits on the other switch.
    lift = tl.step_span("lift2")
    (w,) = tl.waiting((lift.start + lift.end) / 2)
    assert w["exits"][0]["unmet"][0]["point"] == "press/retracted"


def test_a_live_session_waits_where_it_stands() -> None:
    scene = pick_cell()
    with bt.vc.loopback(scene, ["pick"], host="UR") as s:
        s.run(until=bt.vc.step("pick", "await part"))
        s.tick(5)
        (w,) = s.waiting()
        assert (w["program"], w["step"]) == ("pick", "await part")
        assert w["exits"][0]["unmet"][0]["condition"] == "↑part_at_pick"
        s.run()
        tl = s.finish()
    # After the run the waits are the timeline's own.
    assert s.waiting() == tl.waiting(tl.duration)


def test_the_device_comments_list_the_bound_points_by_address() -> None:
    scene = pick_cell()
    text = scene.io_list("comments")
    rows = [line.split(",") for line in text.strip().splitlines()]
    assert rows[0] == ["device", "comment", "point", "node", "channel", "field"]
    by_point = {r[2]: r for r in rows[1:]}
    assert set(by_point) == {"part_at_pick", "spec_ok", "conv", "vacuum"}
    beam = by_point["part_at_pick"]
    assert beam[3:] == ["UR", "DI2", "BEAM1"]
    assert beam[1] == "part_at_pick"  # no tag, no note: the point's name
    with pytest.raises(ValueError, match="comments"):
        scene.io_list("nonsense")


def test_the_fat_sheet_films_each_row(monkeypatch, tmp_path) -> None:
    import botrail.capture as capture

    scene = pick_cell()
    sheet = bt.vc.run_fat(
        scene,
        [bt.vc.TestCase("one cycle", expect=[bt.vc.step_reached("pick", "return")])],
        ["pick"],
    )
    assert sheet.ok
    filmed = []

    def fake(scene_, camera, out, *, fps=30, labels=False, **kw):
        filmed.append((camera, Path(out).name, fps, labels))
        return Path(out)

    monkeypatch.setattr(capture, "record_camera", fake)
    clips = sheet.record(scene, "overview", tmp_path)
    assert clips == {"one cycle": tmp_path / "one_cycle.mp4"}
    assert filmed == [("overview", "one_cycle.mp4", 30, True)]

from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def scene() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))


def test_urscript_from_motion(scene: bt.Scene) -> None:
    g1 = [0.6, 0.4, -0.5, 0.2, 0.0, 0.0]
    g2 = [-0.4, 0.8, -1.0, 0.0, 0.3, 0.0]
    scene.add_segment("main", goal=g1)
    scene.add_segment("main", goal=g2)
    traj = scene.plan_motion("main", broadcast=False)

    code = traj.to_script(dialect="urscript", name="pick")
    lines = code.splitlines()
    assert lines[0] == "def pick():"
    assert lines[-1] == "end"

    # One movej per waypoint, shared segment boundaries deduplicated,
    # plus the initial move-to-start.
    total = sum(len(wps) for _, wps in traj.segments)
    assert code.count("movej(") == total - 1
    # Speeds from the URDF limits: min velocity 2.0, acceleration 4.0.
    assert "a=4, v=2" in code
    # Segment goals stop exactly.
    assert code.rstrip().splitlines()[-2].endswith("r=0)")


def test_urscript_speed_scale_and_start(scene: bt.Scene) -> None:
    scene.add_segment("main", goal=[0.5, 0.4, -0.5, 0.2, 0.0, 0.0])
    traj = scene.plan_motion("main", broadcast=False)

    code = traj.to_script(speed_scale=0.5)
    assert "a=2, v=1" in code

    with_start = traj.to_script()
    without_start = traj.to_script(move_to_start=False)
    assert with_start.count("movej(") == without_start.count("movej(") + 1


def test_urscript_cartesian_becomes_movel(scene: bt.Scene) -> None:
    start = [0.0, 1.1, -0.6, -0.5, 0.0, 0.0]
    scene.set_joint_positions(start)
    (x, y, z), quat = scene.link_pose("tool0")
    goal_ik = scene.robot.ik((x, y, z - 0.08), quat, seed=start)
    assert goal_ik.converged

    scene.add_segment("descend", goal=goal_ik.q, kind="cartesian_line")
    traj = scene.plan_motion("descend", broadcast=False)

    code = traj.to_script(name="descend", tcp_speed=0.1)
    # The whole IK follow path collapses to one linear move; the only
    # movej is the initial move-to-start.
    assert code.count("movel(") == 1
    assert code.count("movej(") == 1
    assert "v=0.1" in code


def test_export_script_names_program_after_file(scene: bt.Scene, tmp_path: Path) -> None:
    scene.add_segment("main", goal=[0.5, 0.4, -0.5, 0.2, 0.0, 0.0])
    traj = scene.plan_motion("main", broadcast=False)

    path = tmp_path / "pick_and_place.script"
    traj.export_script(path)
    code = path.read_text()
    assert code.startswith("def pick_and_place():")


def test_single_plan_exports_too(scene: bt.Scene) -> None:
    traj = scene.plan([1.2, 0.9, -1.5, 0.5, 0.0, 0.0], broadcast=False)
    code = traj.to_script()
    assert code.count("movej(") == len(traj.segments[0][1])


def test_unknown_dialect_is_rejected(scene: bt.Scene) -> None:
    traj = scene.plan([0.5, 0.0, 0.0, 0.0, 0.0, 0.0], broadcast=False)
    with pytest.raises(ValueError, match="urscript"):
        traj.to_script(dialect="klingon")


# ---- sequence → URScript (moves + real I/O from one source) -------------


@pytest.fixture()
def cell_timeline() -> "bt.SequenceTimeline":
    """A pick cell on the six-axis arm, rolled out: a conveyor feeds a box
    over a beam, the robot picks with a vacuum coil — every lowerable
    element in one sequence."""
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))
    scene.add_box("part", size=(0.06, 0.06, 0.06), position=(-0.45, 0.35, 0.03))
    scene.add_conveyor(
        "conv",
        zone_position=(-0.1, 0.35, 0.05),
        zone_size=(0.9, 0.2, 0.14),
        velocity=(0.2, 0.0, 0.0),
        running=False,
    )
    scene.add_beam_sensor(
        "part_at_pick",
        frm=(0.25, 0.25, 0.03),
        to=(0.25, 0.45, 0.03),
        watch=["part"],
    )
    scene.define_signal("vacuum")

    over_pick = [0.95, 0.85, -1.1, 0.25, 0.0, 0.0]
    scene.add_segment("to_pick", goal=over_pick)
    scene.add_segment("home", goal=[0.0] * 6)

    sq = scene.sequence("pick")
    sq.step(
        "feed",
        actions=[bt.seq.start("conv"), bt.seq.motion("to_pick")],
        transition=bt.seq.all_of(bt.seq.signal("part_at_pick"), bt.seq.done()),
    )
    sq.step("halt", actions=[bt.seq.stop("conv")])
    sq.step("grip", actions=[bt.seq.set_signal("vacuum")], transition=bt.seq.elapsed(0.3))
    sq.step("hold", actions=[bt.seq.attach("part")])
    sq.step("return", actions=[bt.seq.motion("home")])
    return sq.simulate()


IO = dict(
    inputs={"part_at_pick": 2},
    outputs={"conv": 0, "vacuum": 1},
)


def test_sequence_to_urscript(cell_timeline) -> None:
    tl = cell_timeline
    assert tl.sequences == ["pick"]

    with pytest.warns(UserWarning, match="beside the move"):
        code = tl.to_script(**IO)
    lines = code.splitlines()
    assert lines[0] == "def pick():"

    # The program follows the authored step order: conveyor coil on, the
    # approach moves, the beam wait, coil off, vacuum on, the dwell, the
    # attach comment, the return moves.
    on = code.index("set_standard_digital_out(0, True)")
    wait = code.index("not get_standard_digital_in(2)")
    off = code.index("set_standard_digital_out(0, False)")
    vac = code.index("set_standard_digital_out(1, True)")
    dwell = code.index("sleep(0.3)")
    grasp = code.index("# attach part")
    assert on < wait < off < vac < dwell < grasp
    assert code.count("movej(") >= 3  # move-to-start + both motions
    # Step names annotate the script.
    for step in ("feed", "halt", "grip", "hold", "return"):
        assert f": {step}" in code


def test_sequence_script_files_and_warnings(cell_timeline, tmp_path: Path) -> None:
    tl = cell_timeline
    path = tmp_path / "pick.script"
    with pytest.warns(UserWarning, match="beside the move"):
        # The feed step's beam wait runs beside the approach move in
        # simulation but after it in the script — flagged, not silent.
        tl.export_script(path, **IO)
    assert path.read_text().startswith("def pick():")

    with pytest.warns(UserWarning, match="beside the move"):
        named = tl.to_script(name="station_1", **IO)
    assert named.startswith("def station_1():")


def test_sequence_script_unmapped_names_raise(cell_timeline) -> None:
    tl = cell_timeline
    with pytest.raises(ValueError, match="part_at_pick"):
        tl.to_script(outputs=IO["outputs"])
    with pytest.raises(ValueError, match="vacuum"):
        tl.to_script(inputs=IO["inputs"], outputs={"conv": 0})
    with pytest.raises(ValueError, match="not part of this rollout"):
        tl.to_script(sequence="other", **IO)


# ---- SFC branches + edges: authored once, simulated and exported --------


def test_branches_and_edges_simulate_and_export() -> None:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))
    scene.define_signal("part_ok", True)  # this scenario: the part passes
    scene.define_signal("part_ng")
    scene.define_signal("pulse")  # part-arrival edge, raised by the feeder

    scene.add_segment("inspect", goal=[0.6, 0.5, -0.8, 0.2, 0.0, 0.0])
    scene.add_segment("place", goal=[0.0] * 6)

    qc = scene.sequence("qc")
    # Edge, not level: a part already sitting there is not an arrival.
    qc.step("await part", transition=bt.seq.rising("pulse"))
    qc.step("inspect", actions=[bt.seq.motion("inspect")])
    sel = qc.select("judge")
    sel.when(bt.seq.signal("part_ok")).step(
        "place", actions=[bt.seq.motion("place")]
    )
    ng = sel.when(bt.seq.signal("part_ng"))
    ng.step(
        "purge",
        actions=[bt.seq.set_signal("part_ng", False)],
        transition=bt.seq.elapsed(0.2),
    )

    feeder = scene.sequence("feeder")
    feeder.step("hold", transition=bt.seq.elapsed(0.1))
    feeder.step("fire", actions=[bt.seq.set_signal("pulse")])

    tl = scene.simulate_sequences(["qc", "feeder"])
    # The bake records the path: arm 0 (part_ok) — and only its spans exist.
    assert tl.branches == [("qc", "judge", 0)]
    names = [name for name, _, _ in tl.step_spans]
    assert "qc/place" in names and "qc/purge" not in names
    # The edge held the program past the startup level.
    await_span = tl.step_span("qc/await part")
    assert 0.1 <= await_span.end <= 0.13

    code = tl.to_script(
        sequence="qc",
        inputs={"pulse": 0, "part_ok": 1, "part_ng": 2},
        outputs={"part_ng": 3},
    )
    # The rising edge is the two-stage interlock: wait low, then high.
    assert code.index("while (get_standard_digital_in(0)):") < code.index(
        "while (not get_standard_digital_in(0)):"
    )
    # Both arms are in the controller program — the skipped one included.
    assert "if (get_standard_digital_in(1)):" in code
    assert "elif (get_standard_digital_in(2)):" in code
    assert "set_standard_digital_out(3, False)" in code  # skipped arm's coil
    assert "sleep(0.2)" in code  # and its dwell

    # A skipped arm with a planned motion cannot be exported honestly.
    sel2 = qc.select("rework gate")
    sel2.when(bt.seq.signal("part_ok")).step("noop")
    sel2.when(bt.seq.signal("part_ng")).step(
        "rework", actions=[bt.seq.motion("inspect")]
    )
    tl = scene.simulate_sequences(["qc", "feeder"])
    with pytest.raises(ValueError, match="never took"):
        tl.to_script(
            sequence="qc",
            inputs={"pulse": 0, "part_ok": 1, "part_ng": 2},
            outputs={"part_ng": 3},
        )

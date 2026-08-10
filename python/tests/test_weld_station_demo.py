"""The weld-station example, asserted the way a line owner would assert it.

`examples/weld_station_demo.py` is the W0 cell of the body-in-white plan
(design/design-weld-line.md): four catalog R-2000iC arms with catalog servo
guns, the catalog body-in-white indexed through the station on a skid,
twenty-four spots per pair of bodies, and a zone interlock over the stretch
of seam the two arms on a side both reach. This pins the properties that
make it a *line*: the body lands on the same millimetre every cycle, every
spot is squeezed on the sheet, the contested stretch is exclusive, and
simultaneity without the interlock is a caught collision, not a near-miss.

Skipped unless the catalog packages are already in the Hugging Face cache
(running the example once fetches them).
"""

import math
import os
import sys
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES))

HF_HUB = Path(
    os.environ.get("HF_HOME") or Path.home() / ".cache" / "huggingface"
) / "hub"
pytestmark = pytest.mark.skipif(
    not (HF_HUB / "datasets--botrail--botrail-catalog").exists(),
    reason="botrail catalog not in the HF cache (run examples/weld_station_demo.py once)",
)

# Baked on the pinned dependency set and the pinned catalog revision. The
# tolerance absorbs libm-level drift between machines, not behaviour
# changes — a replanned transfer shifts the cycle by far more than this.
# 2026-08-08: 110.12 → 110.04 when the arm moved to FANUC's own description
# package (`r2`). Same kinematics, so the taught configurations are
# unchanged to a tenth of a degree; the transfer plans differ only because
# the collision meshes are now the real castings rather than one
# collision-quality mesh doing both jobs.
# 2026-08-08 (later): 110.04 → 104.50 when the catalog's weld gun was
# rebuilt with a 400 mm throat (was 315 mm). A deeper gun holds the wrist
# further from the spot, so the bases moved out to 2.15 m and every taught
# configuration was re-solved.
# 2026-08-08 (later still): 104.50 → 116.34 buying the wrist out of its
# full turns (see `test_the_wrist_never_takes_a_full_turn`), then → 103.48
# once the cell was taught against the body instead of the empty station.
# → 102.22 when the body became a car and the cell grew its guarding, and
# → 113.14 with per-obstacle materials and a wider conveyor zone.
# 2026-08-08 (last): the primitive body was replaced by the catalog's
# body-in-white, and the reach survey in design-weld-line.md §4.2 moved the
# spots to the one seam both arms can actually work — the flank, not the
# roof ditch. That is a different cell: four arms, six spots a side, and a
# takt of its own.
# 2026-08-09: 131.36 → 182.48 opening the layout out. The two arms on a
# side moved from 0.70 m apart to 1.45 m (which is what buys every spot
# two or three clear approaches instead of one) and the spots spread to
# ±1.5 m, so crossing to the contested spot is now a long swing past where
# the other arm works — and the waiting pair has to retreat home first.
# Those four retreats per body are the whole difference.
# 2026-08-09 (later): 182.48 → 182.50 moving the transfer onto W1's
# `advance(distance)` plus a part-present beam at the line head. The beam
# is not decoration: a source emits its body *after* the belt has advected
# that scan, so a load and an advance issued together cost the body its
# first 4 mm and it lands short of datum forever (caught by this file's
# datum and sweep tests — the dn-attitude tabs drifted into the closing
# electrodes). Gated on the beam, the body boards before the pitch is
# commanded and lands to 1e-9 — not one scan short, not one past.
GOLDEN_CYCLE = 182.50
CYCLE_BUDGET = 1.0


@pytest.fixture(scope="module")
def baked():
    import weld_station_demo as demo

    scene, station, riders = demo.build_cell()
    poses = demo.teach(scene, station, riders)
    name = demo.build_sequence(scene, poses, riders)
    return demo, scene, station, riders, scene.simulate_sequence(name, max_duration=400.0)


def spans(timeline) -> dict:
    return {step: (start, end) for step, start, end in timeline.step_spans}


def test_the_station_bakes_one_deterministic_takt(baked) -> None:
    demo, scene, _station, _riders, timeline = baked
    assert sorted(timeline.robots) == sorted(demo.ARMS)
    assert timeline.duration == pytest.approx(GOLDEN_CYCLE, abs=CYCLE_BUDGET)

    again = scene.simulate_sequence("weld_station", max_duration=400.0)
    assert again.duration == timeline.duration
    for robot in timeline.robots:
        assert (
            again.robot_trajectory(robot).positions
            == timeline.robot_trajectory(robot).positions
        )


def test_the_body_indexes_onto_the_same_millimetre(baked) -> None:
    """Taught poses only mean anything because the transfer is metric: the
    conveyor advances v*dt per tick, so after the timed feed every piece is
    at the station datum — the same body, cycle after cycle, courtesy of
    the sink-to-source return loop."""
    demo, _scene, _station, riders, timeline = baked
    at = spans(timeline)
    for body in range(demo.BODIES):
        t_spot = at[f"b{body + 1}_spot"][1]
        for name, want in riders:
            got = timeline.object_pose(name, t_spot)[0]
            assert timeline.object_visible(name, t_spot), name
            assert got == pytest.approx(want, abs=1e-9), (
                f"cycle {body + 1}: {name} landed at {got}, authored at {want}"
            )


def weld_steps(demo) -> list:
    """Every weld step, as `(step name, the arms welding in it)`."""
    out = []
    private = len(demo.SPOT_X["up"]) - 1
    for body in range(demo.BODIES):
        tag = f"b{body + 1}"
        for i in range(private):
            out.append((f"{tag}_s{i + 1}_weld", list(demo.ARMS)))
        for role in demo.ROLES:
            out.append((
                f"{tag}_{role}_s{private + 1}_weld",
                [a for a in demo.ARMS if demo.role_of(a) == role],
            ))
    return out


def test_every_spot_is_squeezed_on_the_sheet(baked) -> None:
    """Twenty-four spots, and each one is a real squeeze: during every weld
    step the gun joint of the welding arm is at the squeeze stroke — the
    electrodes are on the tab, not hovering somewhere."""
    demo, _scene, _station, _riders, timeline = baked
    at = spans(timeline)
    trajectories = {r: timeline.robot_trajectory(r) for r in timeline.robots}
    gun = {
        r: trajectories[r].joint_names.index(f"{r}/{demo.GUN}")
        if f"{r}/{demo.GUN}" in trajectories[r].joint_names
        else trajectories[r].joint_names.index(demo.GUN)
        for r in timeline.robots
    }
    welds = 0
    for step, arms in weld_steps(demo):
        start, end = at[step]
        assert end - start == pytest.approx(demo.WELD_T, abs=0.02), step
        for arm in arms:
            q = trajectories[arm].sample((start + end) / 2)
            assert q[gun[arm]] == pytest.approx(demo.GUN_SQUEEZE, abs=1e-3), (step, arm)
            welds += 1
    assert welds == demo.BODIES * len(demo.ARMS) * len(demo.SPOT_X["up"])


def test_no_gun_ever_touches_the_body(baked) -> None:
    """The guns must not pass through the body they are welding.

    The rollout will not tell you: it checks every tick, but reports only
    *robot-against-robot* pairs, so a gun sweeping through a flank is
    silent. And the taught poses alone do not settle it either — the moves
    between them are joint-space ramps, which nothing plans around. So
    replay the baked cycle and check the whole scene, frame by frame."""
    demo, scene, _station, riders, timeline = baked
    # The display shell rides the line too, but it is not what collides —
    # its convex decomposition fills the apertures the guns work through,
    # which is the whole reason the package ships convex pieces as well.
    # Switching it on here would report every weld as a crash.
    pieces = [name for name, _ in riders if name != demo.SHELL]
    trajectories = {r: timeline.robot_trajectory(r) for r in timeline.robots}

    # Pushing every piece's pose each frame dominates the run, and the body
    # is only in motion while the belt runs — so track one piece and move
    # the set when it has actually moved.
    witness = pieces[0]
    placed = None
    offences = {}
    step, t = 0.05, 0.0
    while t <= timeline.duration:
        for robot, trajectory in trajectories.items():
            scene.set_joint_positions(list(trajectory.sample(t)), robot=robot)
        state = (timeline.object_pose(witness, t), timeline.object_visible(witness, t))
        if state != placed:
            for name in pieces:
                position, quaternion = timeline.object_pose(name, t)
                scene.set_obstacle_pose(name, position, quaternion)
                scene.set_obstacle_enabled(name, timeline.object_visible(name, t))
            placed = state
        for a, b in scene.check_collisions():
            if {a[0], b[0]} == {"link", "obstacle"}:
                offences.setdefault((a[1], b[1]), []).append(round(t, 2))
        t += step
    assert not offences, "gun through the body: " + "; ".join(
        f"{a} x {b} at {times[0]}s ({len(times)} frames)"
        for (a, b), times in offences.items()
    )


def test_the_wrist_never_takes_a_full_turn(baked) -> None:
    """A joint with more than a turn of travel reaches the same pose many
    ways, and the solver picks one per spot with no memory of the last —
    so a cell can come out with the wrist unwinding a whole lap between
    two spots a hand's width apart. Nothing fails when it does: the poses
    are right and the collisions are clean, it just looks wrong and wastes
    the cycle. Assert the property instead, on the baked motion."""
    _demo, _scene, _station, _riders, timeline = baked
    for robot in timeline.robots:
        trajectory = timeline.robot_trajectory(robot)
        positions = trajectory.positions
        for j, name in enumerate(trajectory.joint_names):
            column = [q[j] for q in positions]
            span = math.degrees(max(column) - min(column))
            assert span < 300.0, f"{robot} {name} sweeps {span:.0f} deg"


def test_taught_poses_stay_off_the_joint_stops(baked) -> None:
    """Unwinding chooses between whole turns, and the nearest one can sit
    exactly on a limit. A taught pose there has nowhere to go, and a
    planner asked for it rejects the goal outright."""
    _demo, scene, _station, _riders, timeline = baked
    limits = scene.robot.joint_limits
    for robot in timeline.robots:
        positions = timeline.robot_trajectory(robot).positions
        for j, limit in enumerate(limits):
            if limit is None:
                continue
            column = [q[j] for q in positions]
            margin = min(min(column) - limit[0], limit[1] - max(column))
            assert margin > math.radians(0.5), (
                f"{robot} joint {j} comes within {math.degrees(margin):.2f} deg "
                "of a stop"
            )


def test_the_private_spots_are_welded_in_lockstep(baked) -> None:
    """Half the takt argument is that the private work is simultaneous: all
    four arms are in motion at once for a substantial part of the cycle."""
    _demo, _scene, _station, _riders, timeline = baked
    moves = {r: timeline.moves(r) for r in timeline.robots}
    overlap, t, step = 0.0, 0.0, 0.02
    while t < timeline.duration:
        if all(any(a <= t <= b for _, a, b in moves[r]) for r in timeline.robots):
            overlap += step
        t += step
    assert overlap > 10.0, f"only {overlap:.2f}s of concurrency"


def test_the_contested_stretch_is_exclusive_and_cleared_first(baked) -> None:
    """The contested volume is never shared by the two arms on a side, and
    every cycle the second enters only after the first has left it. (In one
    PLC sequence the steps already serialise them at nominal timing — the
    gate is the guard against drift, and `--clash` below is the proof it is
    needed. Gate-*released* motion is what W1's parallel sequences buy.)"""
    demo, _scene, _station, _riders, timeline = baked
    assert demo.zone_overlap(timeline) == 0.0
    at = spans(timeline)
    edges = dict(timeline.signals)

    def occupied(arm: str, t: float) -> bool:
        state = False
        for edge_t, on in edges[f"zone_{arm}"]:
            if edge_t <= t:
                state = on
        return state

    for body in range(demo.BODIES):
        for role in demo.ROLES:
            start = at[f"b{body + 1}_{role}_across"][0]
            for arm in demo.ARMS:
                if demo.role_of(arm) == role:
                    continue
                assert not occupied(arm, start), (
                    f"cycle {body + 1}: {role} started crossing at {start:.2f}s "
                    f"while {arm} was still in the contested stretch"
                )


def test_simultaneous_entry_is_a_caught_collision(baked) -> None:
    """What the gates are for. Each arm's contested pose is collision-free
    on its own; going together, the two guns on a side meet in the stretch
    they share and the per-tick cross check names the moment."""
    demo, _scene, _station, _riders, _timeline = baked

    scene, station, riders = demo.build_cell()
    poses = demo.teach(scene, station, riders)
    demo.build_sequence(scene, poses, riders)
    with pytest.raises(ValueError, match=r"collide at t = "):
        scene.simulate_sequence(demo.build_clash(scene, poses), max_duration=60.0)


def test_the_body_travels_in_and_recirculates(baked) -> None:
    """No teleporting stock: the body is never drawn inside the station
    before its feed, it enters travelling every cycle (the return loop
    re-feeds it at the line head), and it has left through the sink by the
    end of the run."""
    demo, _scene, _station, riders, timeline = baked
    witness = riders[0][0]
    assert not timeline.object_visible(witness, 0.0)
    arrivals, was_visible = 0, False
    for i in range(int(timeline.duration / 0.05) + 1):
        t = i * 0.05
        visible = timeline.object_visible(witness, t)
        if visible and not was_visible:
            arrivals += 1
            x = timeline.object_pose(witness, t)[0][0]
            assert x < -2.0, f"arrival {arrivals} appeared at x={x:.2f}"
        was_visible = visible
    assert arrivals == demo.BODIES, f"{arrivals} arrivals for {demo.BODIES} cycles"
    for name, _ in riders:
        assert not timeline.object_visible(name, timeline.duration), (
            f"{name} still on the line at the end"
        )


def test_the_recording_replays_into_a_rebuilt_cell(baked, tmp_path: Path) -> None:
    """The baked cycle comes back: a fresh build of the same cell accepts
    the exported recording as per-link transform tracks. The composite
    arms (catalog arm + catalog gun) have no joint tracks to offer — the
    baked export carries link poses, and playback follows them."""
    demo, _scene, _station, riders, timeline = baked
    out = tmp_path / "weld_replay.usda"
    assert timeline.export_usd(out, fps=30.0) == []
    fresh, _station, _riders = demo.build_cell()
    res = fresh.play_usd_animation(out)
    assert res["mode"] == "transforms"
    assert res["warnings"] == []
    assert res["duration"] == pytest.approx(timeline.duration, abs=1 / 30 + 1e-6)
    # Exactly the freight moves: every rider the cell *draws* plus the
    # spot marks it leaves behind, and never the scenery. The 73 collision
    # pieces are hidden — a recording is the animation of the visible
    # world, so they are neither exported nor expected back.
    drawn = {
        name for name, _ in riders
        if name == demo.SHELL or not name.startswith("/World/Line/body/")
    } | set(demo.MARKS)
    moving = set(res["object_tracks"])
    assert moving == drawn
    assert "/World/Cell/Bed" not in moving


def test_exports_usd_in_both_formats(baked, tmp_path: Path) -> None:
    demo, _scene, _station, _riders, timeline = baked
    out = tmp_path / "weld.usda"
    assert timeline.export_usd(out, fps=30.0) == []
    text = out.read_text()
    assert 'upAxis = "Z"' in text
    for name in demo.ARMS:
        assert f'def Xform "{name}"' in text, name
    assert "timeSamples" in text

    crate = tmp_path / "weld.usdc"
    assert timeline.export_usd(crate, fps=30.0) == []
    header = crate.read_bytes()[:8]
    assert header.startswith(b"PXR-USDC")
    assert crate.stat().st_size < out.stat().st_size

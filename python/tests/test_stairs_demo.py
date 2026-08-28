"""The stairs cell (`examples/legged/stairs_delivery_demo.py`): a steel stair flight
ordered from the catalog, and a machine that has to be able to climb what
was ordered.

The walking tests need the Go2 (the catalog package, or Unitree's URDF in
the botrail cache); they skip where it cannot be had. What always runs is
the part itself — the geometry, the walkable treads, the two BOM lines and
the sizes the catalog refuses.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "legged"))

import stairs_delivery_demo as demo  # noqa: E402

FOOT_R = 0.022  # the Go2's ball feet


def _has_dog() -> bool:
    try:
        demo.patrol.dog_of("go2")
    except Exception:  # noqa: BLE001 - no catalog, no cache, no network
        return False
    return True


needs_dog = pytest.mark.skipif(not _has_dog(), reason="the Go2 is not reachable")


# ----------------------------------------------------------- the flight


def test_the_flight_is_a_walkable_stair_with_two_bom_lines() -> None:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    built = bt.parts.stairs(scene, "s", steps=5, rise=0.12, tread=0.35, width=0.8,
                            position=(2.0, 1.0), yaw=0.4, detail="full")
    # Five treads, each walkable and overhanging the one below.
    treads = [n for n in built.obstacles if "/tread" in n]
    assert len(treads) == 5
    code = scene.generate_python()
    assert code.count("set_obstacle_walkable") == 5
    # Stringers, legs and a handrail per side — the flight is more than
    # its treads, and all of it collides.
    assert sum("stringer" in n for n in built.obstacles) == 2
    assert sum("/handrails/" in n and "trim" not in n for n in built.obstacles) == 10
    # The frames the guide path is authored between.
    foot_p, _ = scene.frame("s/foot")
    top_p, _ = scene.frame("s/top")
    assert foot_p[2] == pytest.approx(0.0) and top_p[2] == pytest.approx(0.6)
    rows = {r["names"][0]: r for r in scene.bom().rows}
    assert rows["s"]["category"] == "structure.stairs" and rows["s"]["qty"] == 1
    # A tilted box cannot be walkable — the refusal names the problem.
    scene.add_box("ramp", size=(0.4, 0.2, 0.05), position=(0.0, 3.0, 0.2),
                  quaternion=(0.0, 0.19867, 0.0, 0.98007))
    with pytest.raises(ValueError, match="tilted"):
        scene.set_obstacle_walkable("ramp", True)


def test_the_catalog_sizes_the_flight_and_refuses_what_it_does_not_sell() -> None:
    from botrail._spec import Spec

    try:
        Spec.load(demo.FLIGHT)
    except Exception:  # noqa: BLE001 - the pack is not published yet
        pytest.skip(f"{demo.FLIGHT} is not in the catalog")
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    bt.parts.stairs(scene, "s", catalog=demo.FLIGHT, steps=6, rise=0.15,
                    tread=0.40, width=0.9, position=(2.0, 0.0))
    rows = {r["names"][0]: r for r in scene.bom().rows}
    assert rows["s"]["model"] == "SF-150x400x900-6"
    assert rows["s"]["attributes"]["mass_kg"] == pytest.approx(65.4)
    # The handrail is bought by the side.
    rails = rows["s/handrails"]
    assert rails["qty"] == 2 and rails["category"] == "structure.stairs.rail"
    # A rise nobody sells, and a pairing the walking rule rejects.
    with pytest.raises(ValueError, match="not available"):
        bt.parts.stairs(scene, "bad", catalog=demo.FLIGHT, steps=4, rise=0.09)
    # And the machine's own posture is the cell's to state: the catalog
    # carries the standing stance, a flight is walked lower than that.
    assert demo.STAIR_DEPTH < 0.311
    with pytest.raises(ValueError, match="2R \\+ T"):
        bt.parts.stairs(scene, "bad", catalog=demo.FLIGHT, steps=4, rise=0.08,
                        tread=0.24)


# ------------------------------------------------------------- the climb


@needs_dog
def test_the_dog_climbs_onto_the_treads() -> None:
    scene, tl = demo.bake()
    steps = tl.footfalls("dog")
    assert steps, "the dog never walked"
    tops = {round(i * demo.RISE + FOOT_R, 6) for i in range(1, demo.STEPS + 1)}
    foot_x = scene.frame("flight/foot")[0][0]
    top_x = scene.frame("flight/top")[0][0]
    on_treads = 0
    for _leg, _lift, _land, (x, _y, z) in steps:
        if foot_x + 0.05 < x < top_x - 0.05:
            assert round(z, 6) in tops, f"foothold at x={x:.3f} floats at z={z:.4f}"
            on_treads += 1
    assert on_treads >= 8, f"only {on_treads} footholds on the flight"
    # It ends on the mezzanine, with the tote still on its back.
    assert max(z for *_f, (_x, _y, z) in steps) == pytest.approx(
        demo.STEPS * demo.RISE + FOOT_R, abs=1e-6
    )
    # …and the tote is where the dog's back is, not where its route is:
    # parked on the mezzanine, the body stands its stair depth over the
    # deck and the tote rests on top of that.
    p, _ = tl.object_pose("tote", tl.duration)
    base, _q = tl.base_pose(tl.duration, "dog")
    assert base[2] == pytest.approx(demo.STEPS * demo.RISE + demo.STAIR_DEPTH + FOOT_R, abs=0.01)
    start, _q0 = tl.object_pose("tote", 0.0)
    assert p[2] - start[2] == pytest.approx(demo.STEPS * demo.RISE, abs=0.01)


@needs_dog
def test_the_load_rides_the_body_not_the_route() -> None:
    """The deck of a walking machine is its body. On the flight the body
    rides up the steps and tilts onto the pitch; a load pinned to the route
    would float off the back on the way up and reach inside it coming
    down."""
    import math

    _scene, tl = demo.bake()

    def pitch(q) -> float:
        x, y, z, w = q
        return math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x)))))

    # Rigid with the body means the distance from the base never changes and
    # the load turns with it — not that its *height* over the base is fixed,
    # which a tilt moves (the offset vector rotates with the machine).
    def apart(a, b) -> float:
        return math.dist(a, b)

    start, _q0 = tl.object_pose("tote", 0.0)
    base0, _b0 = tl.base_pose(0.0, "dog")
    on_back = apart(start, base0)

    height, tilted = [], []
    for i in range(41):
        t = tl.duration * i / 40.0
        pose, tq = tl.object_pose("tote", t)
        base, bq = tl.base_pose(t, "dog")
        assert apart(pose, base) == pytest.approx(on_back, abs=1e-6), f"at t = {t:.2f}"
        assert pitch(tq) == pytest.approx(pitch(bq), abs=1e-6), f"at t = {t:.2f}"
        height.append(pose[2])
        tilted.append(pitch(tq))
    assert min(tilted) < -15.0, f"the tote never tilted onto the flight ({min(tilted):.1f} deg)"
    assert max(height) - min(height) > demo.STEPS * demo.RISE * 0.8, "the tote never climbed"


@needs_dog
def test_the_body_tilts_onto_the_flight() -> None:
    """A level body would ask the downhill legs for more reach than a real
    one has; the body pitches onto the grade instead."""
    import math

    _scene, tl = demo.bake()
    pitch_of = lambda q: math.degrees(  # noqa: E731
        math.asin(max(-1.0, min(1.0, 2.0 * (q[3] * q[1] - q[2] * q[0]))))
    )
    # Parked at either end it stands level; mid-flight it is on the pitch.
    assert abs(pitch_of(tl.base_pose(0.0, robot="dog")[1])) < 0.5
    assert abs(pitch_of(tl.base_pose(tl.duration, robot="dog")[1])) < 0.5
    grade = math.degrees(math.atan2(demo.RISE, demo.TREAD))
    mid = max(
        abs(pitch_of(tl.base_pose(t / 20.0 * tl.duration, robot="dog")[1]))
        for t in range(1, 20)
    )
    assert mid == pytest.approx(grade, abs=1.5), f"{mid:.1f} deg vs a {grade:.1f} deg flight"


@needs_dog
def test_the_standard_flight_is_refused() -> None:
    """175 mm risers under a dog rated for 160 mm: refused before it walks,
    and refused for the *rating* — not by a leg running out of range half
    way up. 300 keeps 2R + T inside what the flight is sold in, so the
    order reaches the machine at all."""
    with pytest.raises(ValueError) as refusal:
        demo.bake(rise=0.175, tread=0.30)
    said = str(refusal.value)
    assert "max_step" in said, said
    assert "0.175" in said and "0.160" in said, said


def test_the_stair_posture_is_measured_not_assumed() -> None:
    """The posture is `depth` off the treads, and which fold that is depends
    on the legs. The primitive quad's are shorter than the Go2's, so it gets
    a different fold — and cannot take the flight the Go2 takes."""
    model, gait, *_rest = demo.patrol.dog_of("quad")
    fold = demo.stair_fold(model, gait)
    assert fold is not None
    probe = bt.Scene(model, name="probe")
    pose = dict(gait.stance)
    pose.update({j: fold for j in pose if "thigh" in j})
    pose.update({j: -2.0 * fold for j in pose if "calf" in j})
    (_x, _y, z), _q = probe.link_pose_at(
        next(iter(gait.legs.values())), [pose.get(n, 0.0) for n in model.joint_names]
    )
    assert -z == pytest.approx(demo.STAIR_DEPTH, abs=1e-3)

    with pytest.raises(ValueError) as refusal:
        demo.bake(robot="quad", rise=0.15, pack=None)
    assert "cannot reach" in str(refusal.value)
    _scene, tl = demo.bake(robot="quad", rise=0.12, pack=None)
    assert tl.footfalls("dog"), "the quad never walked"


@needs_dog
def test_the_rating_is_armed_even_when_the_package_is_silent() -> None:
    """The published Go2 predates `max_step_height_mm`, so the gait arrives
    without a rating and the step check would never run. The cell states the
    datasheet figure for *that* machine; a package carrying its own keeps
    it, and one that says nothing is left to the IK."""
    _model, gait, *_rest = demo.patrol.dog_of("go2")
    assert demo.rate_step("go2", gait).max_step == pytest.approx(demo.MAX_STEP)

    # not for a machine it is not the datasheet of, and never over a rating
    # the package states itself
    other = bt.Gait(legs=gait.legs, stance=gait.stance)
    assert demo.rate_step("/some/other/package", other).max_step is None
    stated = bt.Gait(legs=gait.legs, stance=gait.stance, max_step=0.05)
    assert demo.rate_step("go2", stated).max_step == pytest.approx(0.05)

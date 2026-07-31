"""PLC-style sequence demo: a tracking pick on the factory cell.

The conveyor feeds Box_A down the belt until it interrupts a photoelectric
beam at the pick point — and then keeps running. The sequence latches onto
the box (`bt.seq.track`), so every pose taught at the station rides along
with the part: the robot dives onto the moving box, closes on it in motion,
and only lets go of the belt sync once the box is its own. The rest is
structured like a real cell — *planned* transfer moves between stations,
*guarded* ramp moves (no collision check) for the approach/retreat through
contact, an internal `carrying` signal, and timer steps. Everything bakes
into one deterministic timeline (cycle time printed), then exports to USD.

Run with:  python examples/sequence_demo.py [out.usda]
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import botrail as bt  # noqa: E402
from demo import build_scene, teach_grasp  # noqa: E402

BOX = "/World/Conveyor/Box_A"
BOX_SIZE = 0.06  # the carton in factory.usda, sized for the Franka hand
BEAM_RADIUS = 0.005
# Finger stroke: open wide enough to drop over the box, closed a millimetre
# a side into it. That squeeze is the cycle's only by-design contact, so the
# pads are the only links allowed to touch the carried box. The open width
# also has to swallow the few millimetres a joint-space ramp bows sideways
# on its way down (0.04 is the joint limit, which the planner excludes).
OPEN, CLOSED = 0.039, 0.029
TOUCH = ["/panda/panda_leftfinger", "/panda/panda_rightfinger"]
# Vertical standoff for the hover poses either side of a grasp.
HOVER = 0.15


def build_cycle(scene: bt.Scene) -> str:
    """Teaches the motions and the pick_place sequence; returns its name."""
    names = scene.robot.joint_names
    fingers = [n for n in names if "panda_finger_joint" in n]
    home_q = list(scene.joint_positions)

    # ---- the two taught stations, straight out of the USD cell ----------
    pick = scene.frame("/World/Conveyor/PickFrame")
    place = scene.frame("/World/Pallet/PlaceFrame")

    # ---- conveyor feed: Box_A starts upstream, a beam guards the pick ---
    # The taught grasp sits at the box's centre, so the pick frame's height
    # is also where the box rides down the belt.
    scene.set_obstacle_pose(BOX, (-0.9, pick[0][1], pick[0][2]))
    # The transport zone floor sits above the belt slab (top 0.55) so the
    # advection carries the goods, not the conveyor's own structure.
    scene.add_conveyor(
        "conv",
        zone_position=(-0.45, 0.62, 0.60),
        zone_size=(1.3, 0.4, 0.14),
        velocity=(0.15, 0.0, 0.0),
        running=False,
    )
    # The beam trips once the box's leading face comes within the beam
    # radius, so parking it half a box downstream of the pick frame fires
    # the latch just as the box reaches the taught grasp — which is what
    # makes tracking pick the box up rather than a fixed point in space.
    trip_x = pick[0][0] + BOX_SIZE / 2 + BEAM_RADIUS
    scene.add_beam_sensor(
        "beam_pick",
        frm=(trip_x, 0.42, pick[0][2]),
        to=(trip_x, 0.82, pick[0][2]),
        radius=BEAM_RADIUS,
        watch=[BOX],
    )

    # ---- teach the poses by IK posing (studio-equivalent workflow) ------
    # Each station is solved hover-first, so the grasp warm-starts from the
    # pose right above it and stays in the same posture family. Between the
    # stations the robot goes back to the ready pose first: the pallet is a
    # 150 deg base swing from the conveyor, and warm-starting across that
    # walks the solver into a local minimum.
    hover_q = teach_grasp(scene, pick, standoff=HOVER)  # above the belt
    grasp_q = teach_grasp(scene, pick)  # pads around the box, still open
    scene.set_joint_positions(home_q)
    drop_q = teach_grasp(scene, place, standoff=HOVER)  # above the pallet
    place_q = teach_grasp(scene, place)  # box resting on the crate
    scene.set_joint_positions(home_q)

    def with_fingers(q: list, width: float) -> list:
        """The configuration with both finger joints set to `width`
        (joint_names is in q-vector order)."""
        q = list(q)
        for f in fingers:
            q[names.index(f)] = width
        return q

    # ---- planned transfer motions (fingers stay closed while carrying) --
    scene.add_segment("to_hover", goal=with_fingers(hover_q, OPEN))
    scene.add_segment("to_pallet", goal=with_fingers(drop_q, CLOSED))
    scene.add_segment("home", goal=home_q)

    # ---- the sequence ---------------------------------------------------
    scene.define_signal("carrying")
    ramp_to = lambda q: dict(zip(names, q))  # noqa: E731

    sq = scene.sequence("pick_place")
    # The belt starts and the robot pre-positions over the pick point at the
    # same time; the step ends when the part has arrived *and* the arm is
    # there to meet it (series contacts).
    sq.step(
        "feed",
        actions=[bt.seq.start("conv"), bt.seq.motion("to_hover")],
        transition=bt.seq.all_of(bt.seq.signal("beam_pick"), bt.seq.done()),
    )
    # No halt: from here the taught poses ride the box down the belt.
    sq.step("latch", actions=[bt.seq.track(BOX)])
    sq.step("descend", actions=[bt.seq.ramp(ramp_to(with_fingers(grasp_q, OPEN)), 0.6)])
    sq.step("close", actions=[bt.seq.ramp({f: CLOSED for f in fingers}, 0.4)])
    sq.step(
        "grasp",
        actions=[
            # Grasping the tracked part freezes the sync offset, so the lift
            # goes straight up from wherever the box was caught.
            bt.seq.attach(BOX, link="/panda/panda_hand", touch_links=TOUCH),
            bt.seq.set_signal("carrying"),
        ],
    )
    sq.step("lift", actions=[bt.seq.ramp(ramp_to(with_fingers(hover_q, CLOSED)), 0.6)])
    sq.step("carry", actions=[bt.seq.untrack(), bt.seq.motion("to_pallet")])
    sq.step("lower", actions=[bt.seq.ramp(ramp_to(with_fingers(place_q, CLOSED)), 0.8)])
    sq.step(
        "release",
        actions=[bt.seq.detach(BOX), bt.seq.set_signal("carrying", False)],
    )
    sq.step("open", actions=[bt.seq.ramp({f: OPEN for f in fingers}, 0.4)])
    sq.step("retreat", actions=[bt.seq.ramp(ramp_to(with_fingers(drop_q, OPEN)), 0.8)])
    sq.step("settle", transition=bt.seq.elapsed(0.5))
    sq.step("home", actions=[bt.seq.motion("home")])
    return sq.name


def main() -> None:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("cell_seq.usda")
    scene = build_scene()
    name = build_cycle(scene)

    timeline = scene.simulate_sequence(name)
    print(f"cycle time: {timeline.duration:.2f}s")
    spans = {step: (start, end) for step, start, end in timeline.step_spans}
    for step, start, end in timeline.step_spans:
        print(f"  {start:6.2f} – {end:6.2f}  {step}")

    # How far the belt carried the box between the latch and the grasp: the
    # distance the pick would have missed by without tracking.
    latch, grasp = spans["latch"][0], spans["grasp"][0]
    travel = timeline.object_pose(BOX, grasp)[0][0] - timeline.object_pose(BOX, latch)[0][0]
    print(f"tracked pick: caught the box {travel * 1e3:.0f} mm downstream, belt still running")

    warnings = timeline.export_usd(out, fps=60.0)
    for w in warnings:
        print(f"warning: {w}")
    print(f"exported to {out} — view with: usdview {out}")


if __name__ == "__main__":
    main()

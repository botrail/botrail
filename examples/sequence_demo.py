"""PLC-style sequence demo: the full §1 scenario on the factory cell.

The conveyor feeds Box_A down the belt until it interrupts a photoelectric
beam at the pick point; the sequence stops the belt and runs a pick cycle
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
from demo import build_scene  # noqa: E402

BOX = "/World/Conveyor/Box_A"
# The 12 cm box is bulky for the Franka hand: allow by-design contact with
# the whole wrist assembly, not just the gripper subtree.
TOUCH = [
    "/panda/panda_hand",
    "/panda/panda_leftfinger",
    "/panda/panda_rightfinger",
    "/panda/panda_link7",
    "/panda/panda_link6",
    "/panda/panda_link5",
]


def build_cycle(scene: bt.Scene) -> str:
    """Teaches the motions and the pick_place sequence; returns its name."""
    names = scene.robot.joint_names
    fingers = [n for n in names if "panda_finger_joint" in n]
    home_q = list(scene.joint_positions)

    # ---- conveyor feed: Box_A starts upstream, a beam guards the pick ---
    scene.set_obstacle_pose(BOX, (-0.9, 0.62, 0.61))
    # The transport zone floor sits above the belt slab (top 0.55) so the
    # advection carries the goods, not the conveyor's own structure.
    scene.add_conveyor(
        "conv",
        zone_position=(-0.45, 0.62, 0.66),
        zone_size=(1.3, 0.4, 0.14),
        velocity=(0.15, 0.0, 0.0),
        running=False,
    )
    scene.add_beam_sensor(
        "beam_pick",
        frm=(0.06, 0.42, 0.61),
        to=(0.06, 0.82, 0.61),
        watch=[BOX],
    )

    # ---- teach the poses by IK posing (studio-equivalent workflow) ------
    pick_pos, pick_quat = scene.frame("/World/Conveyor/PickFrame")
    place_pos, _ = scene.frame("/World/Pallet/PlaceFrame")

    scene.set_tcp_target(pick_pos, pick_quat)
    grasp_q = list(scene.joint_positions)  # touching the box (open)
    scene.set_tcp_target((pick_pos[0], pick_pos[1], pick_pos[2] + 0.15), pick_quat)
    hover_q = list(scene.joint_positions)  # above the conveyor
    scene.set_tcp_target((place_pos[0], place_pos[1], place_pos[2] + 0.20), pick_quat)
    drop_q = list(scene.joint_positions)  # above the pallet
    scene.set_tcp_target((place_pos[0], place_pos[1], place_pos[2] + 0.38), pick_quat)
    retreat_q = list(scene.joint_positions)  # clear of the released box
    scene.set_joint_positions(home_q)

    closed = 0.025  # slight squeeze on the 12 cm box

    def with_fingers(q: list, width: float) -> list:
        """The configuration with both finger joints set to `width`
        (joint_names is in q-vector order)."""
        q = list(q)
        for f in fingers:
            q[names.index(f)] = width
        return q

    # ---- planned transfer motions (fingers stay closed while carrying) --
    scene.add_segment("to_hover", goal=hover_q)
    scene.add_segment("to_pallet", goal=with_fingers(drop_q, closed))
    scene.add_segment("home", goal=home_q)

    # ---- the sequence ---------------------------------------------------
    scene.define_signal("carrying")
    ramp_to = lambda q: dict(zip(names, q))  # noqa: E731

    sq = scene.sequence("pick_place")
    sq.step("feed", actions=[bt.seq.start("conv")], transition=bt.seq.signal("beam_pick"))
    sq.step("halt", actions=[bt.seq.stop("conv")])
    sq.step("approach", actions=[bt.seq.motion("to_hover")])
    sq.step("descend", actions=[bt.seq.ramp(ramp_to(grasp_q), 0.8)])
    sq.step("close", actions=[bt.seq.ramp({f: closed for f in fingers}, 0.4)])
    sq.step(
        "grasp",
        actions=[
            bt.seq.attach(BOX, link="/panda/panda_hand", touch_links=TOUCH),
            bt.seq.set_signal("carrying"),
        ],
    )
    sq.step("lift", actions=[bt.seq.ramp(ramp_to(with_fingers(hover_q, closed)), 0.8)])
    sq.step("carry", actions=[bt.seq.motion("to_pallet")])
    sq.step(
        "release",
        actions=[bt.seq.detach(BOX), bt.seq.set_signal("carrying", False)],
    )
    sq.step("open", actions=[bt.seq.ramp({f: 0.035 for f in fingers}, 0.4)])
    sq.step("retreat", actions=[bt.seq.ramp(ramp_to(with_fingers(retreat_q, 0.035)), 0.8)])
    sq.step("settle", transition=bt.seq.elapsed(0.5))
    sq.step("home", actions=[bt.seq.motion("home")])
    return sq.name


def main() -> None:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("cell_seq.usda")
    scene = build_scene()
    name = build_cycle(scene)

    timeline = scene.simulate_sequence(name)
    print(f"cycle time: {timeline.duration:.2f}s")
    for name, start, end in timeline.step_spans:
        print(f"  {start:6.2f} – {end:6.2f}  {name}")

    warnings = timeline.export_usd(out, fps=60.0)
    for w in warnings:
        print(f"warning: {w}")
    print(f"exported to {out} — view with: usdview {out}")


if __name__ == "__main__":
    main()

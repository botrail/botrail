"""An AGV serving the factory cell: call → gate → dock → release.

The cell was already drawn for this. Its floor carries a painted walkway
south of the cell boundary, and its safety fence has a 0.93 m gate between
two posts at x = ±0.5 — so the route is not invented, it is the one the
layout asks for: run the walkway, turn at the gate, nose in to the dock.

The vehicle is a device in the PLC sense — the same standing as a conveyor
or a lifter. `goto` is the dispatch order, `device_done` is position
reached, and a dock zone reports presence. What makes it a *cell* rather
than two machines sharing a floor is the interlock: the gate zone watches
the arm, and the AGV is not called in until the arm has retreated and that
zone has gone off. That is written the way a PLC writes it — a zone, a
level test, and a step that waits — and it is the same vocabulary the
two-arm cell uses for its shared airspace.

Two authoring notes that a vehicle forces and a conveyor never does:

  * **Ground clearance is real.** The aisle check tests the body against
    everything it does not carry, and this cell's floor decals are 3 mm
    obstacles. The chassis rides 90 mm up (as a real AGV does) and the
    wheels — which do touch the floor — carry collision off, the same way
    the two-arm cell's belt cleats do.
  * **A turn sweeps wider than the body.** The pivot at the gate swings a
    0.30 m radius, so the clearance that matters is around the turn, not
    along the straight.

Run with `--clash` to push the dock 0.6 m deeper into the cell: the AGV
then noses into the pallet and the cycle fails as a hard `VehicleCollision`
with the time, the body part and the pallet board named.

Run with:  python examples/agv_cell_demo.py [out.usda] [--clash]
"""

import math
import os
import sys
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import botrail as bt  # noqa: E402
from demo import HAND, build_scene as build_factory_cell, teach_grasp  # noqa: E402

# ------------------------------------------------------ the vehicle: MiR250
#
# The AGV is a MiR250 — a real AMR, modelled from published numbers rather
# than invented ones, because what this cell check answers ("does it fit
# through, how long does the tact take") is decided by the footprint and
# the speeds, not by the styling.
#
#   * Parameters come from the MiR250 product specification: 800 x 580 x
#     300 mm, 83 kg, 250 kg payload, 2.0 m/s, 1.0 m/s^2.
#   * Shape comes from the community ROS description (DFKI-NI/mir_robot,
#     BSD-3-Clause — a community model, not affiliated with MiR), fetched
#     at first run like the Franka is. Its mesh measures 800.3 x 580.3 mm
#     and stands 34.3 mm off the floor, so the spec and the mesh agree.
#
# The mount offsets below are that description's, so the wheels and the
# safety scanners sit where they do on the real machine.
MIR_REPO = "https://raw.githubusercontent.com/DFKI-NI/mir_robot/noetic"
MIR_FILES = [
    "LICENSE",
    "mir_description/meshes/visual/mir_250_base.stl",
    "mir_description/meshes/visual/sick_lms-100.stl",
]

MIR250_SIZE = (0.800, 0.580, 0.300)  # L x W x H, spec
MIR250_MAX_SPEED = 2.0  # m/s, spec (full payload, flat)
MIR250_ACCEL = 1.0  # m/s^2, spec

# Body mesh offset, drive wheels, and the two diagonal safety scanners —
# all from mir_250 in the ROS description.
BODY_DX = -0.004485
WHEEL_R, WHEEL_W, WHEEL_DY = 0.100, 0.038, 0.2015
LASER_DX, LASER_DY, LASER_DZ = 0.315, 0.205, 0.1914

# Linear-RGB colors (USD convention — never raw sRGB bytes).
MIR_WHITE = (0.807, 0.807, 0.813)
BLACK = (0.010, 0.010, 0.011)

# The painted walkway runs between the y = -2.45 and y = -3.35 lines; the
# fence gate is the gap between the posts at x = ±0.5, y = -2.0, which
# leaves a 0.93 m opening for a 0.58 m wide robot.
LANE_Y = -2.90
WAREHOUSE = (-2.60, LANE_Y)
GATE = (0.0, LANE_Y)
# As far in as the cell allows: 30 mm further and the body clips the
# pallet's deck boards, which is also why the handover ends up at the very
# limit of the arm's reach (see `build_cycle`).
DOCK = (0.0, -1.15)
CLASH_DOCK = (0.0, -0.75)

# Not the spec 2.0 m/s, and the reason is worth stating: V0 vehicles move at
# constant speed (no acceleration model), and at 1.0 m/s^2 the ramp to
# 2.0 m/s alone is v²/2a = 2.0 m — 77 % of this 2.6 m walkway run, so the
# constant-speed bake would be badly wrong. At 0.8 m/s the ramp is 0.32 m
# (12 %), which the model can absorb. It is also what an AMR actually does
# next to a manned cell: full speed is for open aisles.
SPEED = 0.8
# The spec sheet lists turning diameter as "to be determined", so unlike
# every other number here this one is *assumed*, not published — a sedate
# in-cell pivot. It is the obvious thing to sweep.
TURN = math.radians(45.0)

READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.035, 0.035]

# The carton the cell hands over, and the gripper stroke around it — the
# same numbers the single-arm cell uses.
CARTON = "/World/Conveyor/Box_A"
BOX_SIZE = 0.06
OPEN, CLOSED = 0.039, 0.029
PADS = ["/panda/panda_leftfinger", "/panda/panda_rightfinger"]
HOVER = 0.15
# The deck the load actually rests on. The drawn shell tops out at 303 mm,
# but collision runs on its convex decomposition, and the convex hull of a
# dished top fills the dish — so the deck the checker sees is a few
# millimetres higher than the one you can see. Setting the carton on the
# drawn surface is rejected as a collision; 310 mm is the first height that
# clears, and the 7 mm it floats by is invisible.
DECK_TOP = 0.310


def fetch_mir250() -> Path:
    """Download the MiR250 meshes into the botrail cache (once)."""
    cache = Path(os.environ.get("BOTRAIL_CACHE_DIR") or Path.home() / ".cache" / "botrail")
    dest = cache / "assets" / "mir250"
    for rel in MIR_FILES:
        target = dest / rel
        if target.exists():
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        print(f"downloading {rel} ...")
        part = target.with_suffix(target.suffix + ".part")
        urllib.request.urlretrieve(f"{MIR_REPO}/{rel}", part)
        part.rename(target)
    return dest / "mir_description" / "meshes" / "visual"


def yaw_flip(yaw: float) -> tuple:
    """Quaternion for Rz(yaw)·Rx(π) — the scanner mount, whose mesh is
    modelled upside down (as the ROS description mounts it)."""
    return (math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0)


def add_agv(scene: bt.Scene) -> None:
    """Parks a MiR250 at the warehouse end of the walkway, facing the gate.

    The vehicle frame is the robot's own: floor level, midway between the
    drive wheels — so every offset below is the one in its description."""
    meshes = fetch_mir250()
    x, y = WAREHOUSE

    # The shell is the body proper: it is what has to clear the aisle. Its
    # own 34 mm of ground clearance is modelled in the mesh, which is what
    # keeps it off the cell's 3 mm floor decals.
    scene.add_mesh("agv/base", meshes / "mir_250_base.stl",
                   (x + BODY_DX, y, 0.0), color=MIR_WHITE)

    # Ground-contact parts ride with collision off — scenery, not body,
    # exactly like the belt cleats in the two-arm cell. A wheel that keeps
    # its collision on is touching the floor slab from the first tick.
    lie = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))
    for side, name in ((1, "l"), (-1, "r")):
        wheel = f"agv/wheel_{name}"
        scene.add_cylinder(wheel, WHEEL_R, WHEEL_W,
                           (x, y + side * WHEEL_DY, WHEEL_R),
                           quaternion=lie, color=BLACK)
        scene.set_obstacle_enabled(wheel, False)

    # The two safety laser scanners sit at diagonally opposite corners —
    # the visual signature of this machine, and the reason it can see all
    # four sides with two sensors.
    for dx, dy, yaw, name in ((LASER_DX, LASER_DY, 0.25 * math.pi, "front"),
                              (-LASER_DX, -LASER_DY, -0.75 * math.pi, "back")):
        laser = f"agv/scanner_{name}"
        scene.add_mesh(laser, meshes / "sick_lms-100.stl",
                       (x + dx + BODY_DX, y + dy, LASER_DZ),
                       quaternion=yaw_flip(yaw), color=BLACK)
        scene.set_obstacle_enabled(laser, False)


def build_scene(clash: bool = False) -> bt.Scene:
    """The factory cell with an AGV on the walkway. Shared with
    `play_record.py`, which rebuilds the cell a recording was baked from."""
    scene = build_factory_cell()
    add_agv(scene)
    scene.add_vehicle(
        "agv",
        body=["agv"],
        path=[WAREHOUSE, GATE, CLASH_DOCK if clash else DOCK],
        # The gate is a station in its own right, not just a corner: a
        # vehicle waiting for entry permission has to have somewhere to
        # wait that is *outside* the cell.
        stations={"warehouse": 0, "gate": 1, "dock": 2},
        speed=SPEED,
        turn_speed=TURN,
        start="warehouse",
        # It backs out of the dock rather than turning around in it. That is
        # what the machine does, and here it is also what fits: a pivot
        # sweeps the body's half-diagonal (0.49 m), which from this dock
        # reaches straight into the pallet — the cycle fails with a
        # `VehicleCollision` on the way out if you turn this off.
        allow_reverse=True,
        # The load deck, in the vehicle's own frame. Anything resting in it
        # rides along — there is no load or unload action to author, which
        # is the same bargain the conveyor makes with its zone. It is drawn
        # a little larger than the 800 x 580 deck on purpose: the zone
        # answers "is this aboard", and a carton set down on the very edge
        # is aboard. Its floor is the deck top, so nothing on the ground
        # can be mistaken for cargo.
        tray_position=(BODY_DX, 0.0, DECK_TOP + 0.06),
        tray_size=(0.84, 0.62, 0.12),
    )
    # Presence at the dock, and the interlock zone the arm shares with the
    # gate approach. One watches the vehicle, the other watches the arm —
    # a zone that saw both would latch on itself.
    scene.add_zone_sensor(
        "dock_occupied", position=(DOCK[0], DOCK[1], 0.3), size=(0.9, 0.9, 0.6),
        watch=["agv/base"],
    )
    scene.add_zone_sensor(
        "gate_zone", position=(0.0, -1.15, 0.6), size=(1.1, 1.5, 1.2),
        watch_robots=["panda"],
    )
    # Load-present, and the one sensor here that has to travel: bolted to
    # the floor it would report the carton for the moment it is set down and
    # then lose it. Mounted, it is authored in the vehicle's frame and
    # re-resolved every scan, so it still reads "loaded" out on the aisle —
    # which is what a departure permit has to be able to ask.
    scene.add_zone_sensor(
        "tray_loaded", position=(BODY_DX, 0.0, DECK_TOP + 0.06), size=(0.84, 0.62, 0.12),
        watch=[CARTON], mount="agv",
    )
    return scene


def build_cycle(scene: bt.Scene) -> str:
    """Teaches the poses and writes the handover cycle; returns its name.

    The shape of it is the one a cell PLC would write: the vehicle is
    called while the arm is still picking, waits outside the gate, and only
    comes in once the arm's zone is clear; the arm in turn waits on
    `dock_occupied` before reaching over the deck, and the departure permit
    asks the vehicle's own load sensor whether it actually has the carton.
    """
    names = scene.robot.joint_names
    fingers = [n for n in names if "panda_finger_joint" in n]
    home = list(scene.joint_positions)
    pick = scene.frame("/World/Conveyor/PickFrame")

    # The handover point: the deck's leading edge. It is not a free choice
    # — the cell's pallet keeps the vehicle at y = -1.15, which puts the
    # deck edge at exactly 0.75 m from the robot's base, and that is this
    # arm's limit. 50 mm further onto the deck is already out of reach.
    # A layout that wants margin has to move the pallet or the pedestal;
    # this is the kind of thing the check exists to make visible.
    deck = ((DOCK[0], DOCK[1] + 0.40, DECK_TOP + BOX_SIZE / 2 + 0.002), pick[1])

    # Teach hover-first at each station so the grasp warm-starts from the
    # pose above it, and go home between stations — the deck is a 180 deg
    # swing from the belt (the house rule from the single-arm cell).
    hover_q = teach_grasp(scene, pick, standoff=HOVER)
    grasp_q = teach_grasp(scene, pick)
    scene.set_joint_positions(home)
    over_deck_q = teach_grasp(scene, deck, standoff=HOVER)
    on_deck_q = teach_grasp(scene, deck)
    scene.set_joint_positions(home)

    def with_fingers(q: list, width: float) -> list:
        q = list(q)
        for f in fingers:
            q[names.index(f)] = width
        return q

    scene.add_segment("to_pick", goal=with_fingers(hover_q, OPEN))
    scene.add_segment("to_pick", goal=with_fingers(grasp_q, OPEN))
    scene.add_segment("lift", goal=with_fingers(hover_q, CLOSED))
    scene.add_segment("to_deck", goal=with_fingers(over_deck_q, CLOSED))
    scene.add_segment("to_deck", goal=with_fingers(on_deck_q, CLOSED))
    scene.add_segment("clear", goal=with_fingers(over_deck_q, OPEN))
    scene.add_segment("home", goal=with_fingers(home, OPEN))

    sq = scene.sequence("agv_service")
    # The call goes out while the arm is still picking: the vehicle drives
    # up to the gate and waits there. `goto` and `start_motion` are both
    # fire-and-await, one per actor, so this step runs them in parallel.
    sq.step(
        "呼出",
        actions=[bt.seq.goto("agv", "gate"), bt.seq.motion("to_pick")],
        transition=bt.seq.all_of(bt.seq.device_done("agv"), bt.seq.done()),
    )
    sq.step("グリッパ閉", actions=[bt.seq.ramp({f: CLOSED for f in fingers}, 0.4)])
    sq.step("把持", actions=[bt.seq.attach(CARTON, link=HAND, touch_links=PADS)])
    sq.step("持上げ", actions=[bt.seq.motion("lift")])
    # The interlock: the loaded arm is out of the shared airspace by now,
    # and that — not a timer — is what lets the vehicle come in.
    sq.step("入構許可", transition=bt.seq.signal("gate_zone", False))
    sq.step(
        "入構",
        actions=[bt.seq.goto("agv", "dock")],
        transition=bt.seq.all_of(
            bt.seq.device_done("agv"), bt.seq.signal("dock_occupied")
        ),
    )
    # The handshake in the other direction: the arm does not reach over the
    # deck until the vehicle reports itself parked there.
    sq.step("移載", actions=[bt.seq.motion("to_deck")])
    sq.step("開放", actions=[bt.seq.ramp({f: OPEN for f in fingers}, 0.4)])
    sq.step("離脱", actions=[bt.seq.detach(CARTON)])
    sq.step("退避", actions=[bt.seq.motion("clear")])
    sq.step("復帰", actions=[bt.seq.motion("home")])
    # Departure permit, asked of the machine itself: the deck's own sensor
    # says it has the carton, and the arm is clear of the gate.
    sq.step(
        "発進許可",
        transition=bt.seq.all_of(
            bt.seq.signal("tray_loaded"), bt.seq.signal("gate_zone", False)
        ),
    )
    sq.step(
        "搬出",
        actions=[bt.seq.goto("agv", "warehouse")],
        transition=bt.seq.device_done("agv"),
    )
    return sq.name


def main() -> None:
    clash = "--clash" in sys.argv[1:]
    out = next((a for a in sys.argv[1:] if not a.startswith("--")), "cell_agv.usda")

    scene = build_scene(clash)
    name = build_cycle(scene)
    try:
        tl = scene.simulate_sequence(name, max_duration=90.0)
    except ValueError as err:
        print(f"cycle failed: {err}")
        sys.exit(1)

    print(f"cycle time: {tl.duration:.2f}s")
    for step, start, end in tl.step_spans:
        print(f"  {step:<8} {start:6.2f} – {end:6.2f}s")
    lanes = dict(tl.signals)
    edges = lambda lane: ", ".join(f"{t:.2f}→{'on' if v else 'off'}" for t, v in lanes[lane])
    for lane in ("agv", "dock_occupied", "gate_zone", "tray_loaded"):
        print(f"  {lane:<14} {edges(lane)}")
    # The point of the mounted sensor: it still reads "loaded" once the
    # vehicle has left, which is what makes it a departure permit rather
    # than a snapshot.
    loaded_at = next(t for t, v in lanes["tray_loaded"] if v)
    left_at = next(t for t, v in reversed(lanes["agv"]) if v)
    print(f"loaded at {loaded_at:.2f}s, still loaded when it pulls out at {left_at:.2f}s: "
          f"{tl.signal('tray_loaded').value_at(tl.duration)}")
    # Where the carton ended up — it rode the deck out of the cell.
    carried = tl.object_pose(CARTON, tl.duration)[0]
    print(f"carton ends at {tuple(round(v, 3) for v in carried)} "
          f"(placed at y = {DOCK[1] + 0.40:.2f})")

    tl.export_usd(out, fps=60)
    print(f"wrote {out}")
    print(f"  replay it with:  python examples/play_record.py {out}")


if __name__ == "__main__":
    main()

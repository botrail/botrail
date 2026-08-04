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
from demo import HAND, build_scene as build_factory_cell  # noqa: E402

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
DOCK = (0.0, -1.35)
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
    return scene


def teach(scene: bt.Scene, position) -> list:
    """The joint vector that puts the hand at `position` — the scripted
    form of dragging the studio's TCP gizmo."""
    ik = scene.set_tcp_target(position, link=HAND)
    if not ik.converged:
        raise RuntimeError(f"IK missed {position}: {ik.pos_error * 1e3:.1f} mm short")
    return list(scene.joint_positions)


def build_cycle(scene: bt.Scene) -> str:
    """Teaches the two arm poses and writes the interlocked cycle."""
    over_gate = teach(scene, (0.0, -0.62, 0.62))
    scene.set_joint_positions(READY)

    scene.add_segment("reach_gate", goal=over_gate)
    scene.add_segment("retreat", goal=READY)

    sq = scene.sequence("agv_service")
    # The order goes out while the arm is still working: the vehicle drives
    # up to the gate and the arm keeps hold of the cell. Both start here —
    # `goto` and `start_motion` are both fire-and-await, one per actor.
    sq.step(
        "呼出",
        actions=[bt.seq.goto("agv", "gate"), bt.seq.motion("reach_gate")],
        transition=bt.seq.device_done("agv"),
    )
    # The interlock, and the one wait in this cycle that is load-bearing:
    # the vehicle is parked at the gate and may not come in until the arm
    # is out of the shared airspace. The retreat starts here, and what ends
    # the step is the *zone going off* — the arm's position, not a timer
    # and not the motion's own completion (it clears the zone well before
    # it finishes parking).
    sq.step(
        "入構待ち",
        actions=[bt.seq.motion("retreat")],
        transition=bt.seq.signal("gate_zone", False),
    )
    sq.step(
        "入構",
        actions=[bt.seq.goto("agv", "dock")],
        transition=bt.seq.all_of(
            bt.seq.device_done("agv"), bt.seq.signal("dock_occupied")
        ),
    )
    # V1 turns this dwell into a real transfer onto the vehicle's tray.
    sq.step("移載", transition=bt.seq.elapsed(1.0))
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
    # What the interlock actually cost: the vehicle sat at the gate for
    # this long, held by the arm's position.
    wait = tl.step_span("入構待ち")
    print(f"held at the gate for {wait.end - wait.start:.2f}s "
          f"(arm left the zone at {wait.end:.2f}s)")
    print("agv moving: " + ", ".join(f"{t:.2f}→{'on' if v else 'off'}"
                                     for t, v in lanes["agv"]))

    tl.export_usd(out, fps=60)
    print(f"wrote {out}")
    print(f"  replay it with:  python examples/play_record.py {out}")


if __name__ == "__main__":
    main()

"""Two arms fold a T-shirt.

A child's T-shirt lies on a table between two UR5e arms, each with a
Robotiq 2F-85. The arms turn the sleeves in over the body, then take the
hem by its corners and turn the lower half over onto the shoulders — the
fold a person makes on a table, authored as ordinary robot programs.

The cloth is `bt.cloth`: a shell simulated *after* the bake, against the
finished cycle. Each gripper's tool point follows its arm and takes the
cloth within a few centimetres of it while its signal is on — through both
layers of the shirt, so a pinch at a cuff lifts the whole sleeve. The
robots never feel the cloth and plan as if it were not there; what the
demo teaches is therefore where the hands go and how fast:

* a fold is a *turn*: the hand carries its patch on an arc about the fold
  line, so the flap stays as long as it is — a straight carry would
  stretch it or drag the shirt along;
* the turn stops a few centimetres above the cloth it lands on and lets
  go there: the flap drops the rest of the way instead of being pressed
  into the layer under it;
* the hem is held on springs (`compliance`) and let go softly (`soften`):
  two hands holding one flap rigidly fight each other through the cloth.

The shirt is a front and a back sewn together, on 2 cm cells: about a
thousand vertices and a minute of cloth simulation for the 38 s cycle. A
coarser mesh (`--spacing`) is not a quicker look at the same fold: cells
wider than the cloth's natural crease cannot fold at a seam, and the sleeve
drags the body's side over with it.

Run with:  python examples/cloth/tshirt_fold_demo.py [out.usda] [--studio]
               [--spacing 0.02]
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import botrail as bt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import _robotiq as rq

# --- cell dimensions (metres; z = 0 is the shop floor) ------------------
TABLE = (0.80, 0.84, 0.72)  # length (x), width (y), height
BASE_Z = 0.80  # the arms' mounting plane, on their pedestals
SPAN = 0.62  # the arms stand at y = ±SPAN, facing each other across the table
BODY = (0.36, 0.50)  # the shirt's body: width, length
SLEEVE = (0.14, 0.16)  # sleeve length, width
HOVER = 0.10  # approach height above a grasp
PINCH = 0.015  # the tool point's height above the table when it closes
RELEASE = 0.04  # how far above the cloth under it a turned flap is let go
RADIUS = 0.05  # a closing gripper takes the cloth this close to its tool point
# Nothing holds the shirt down, so a crease travels while its flap is turned
# and the flap lands twice that far beyond where a fixed hinge would put it.
# How far is read off the first bake, the way a fold is taught on a real
# cell.
SLEEVE_CREEP = 0.035
HEM_CREEP = 0.055
DOWN = (1.0, 0.0, 0.0, 0.0)  # tool +Z at the floor
OPEN, SHUT = 0.0, 0.7  # finger joint: open, closed on cloth
READY = [0.0, -1.9, 1.8, -1.5, -1.57, 0.0]
ARMS = ("north", "south")  # at y = +SPAN and y = -SPAN


def build_cell() -> bt.Scene:
    arm = rq.attach(rq.load_arm())
    quarter = math.sin(math.pi / 4)
    # Each arm faces the table: the north one looks along -y, the south +y.
    scene = bt.Scene(arm, base_position=(0.0, SPAN, BASE_Z),
                     base_quaternion=(0.0, 0.0, -quarter, quarter), name="north")
    scene.add_robot(arm, name="south", base_position=(0.0, -SPAN, BASE_Z),
                    base_quaternion=(0.0, 0.0, quarter, quarter))
    for name in ARMS:
        scene.set_joint_positions([*READY, OPEN], robot=name)
    scene.add_box("floor", size=(2.4, 2.4, 0.05), position=(0.0, 0.0, -0.025),
                  color=(0.35, 0.37, 0.40))
    for name, y in zip(ARMS, (SPAN, -SPAN)):
        scene.add_cylinder(f"pedestal/{name}", 0.09, BASE_Z, (0.0, y, BASE_Z / 2),
                           color=(0.34, 0.36, 0.40))
        scene.set_obstacle_enabled(f"pedestal/{name}", False)
    bt.parts.table(scene, "table", size=TABLE, position=(0.0, 0.0))
    for name in ARMS:
        scene.define_signal(f"grip_{name}")
    return scene


def shirt(spacing: float):
    # Turned a quarter: the hem at -x, the shoulders at +x, a cuff towards
    # each arm.
    return bt.cloth.tshirt("shirt", on="table/top", yaw=math.pi / 2, spacing=spacing,
                           body=BODY, sleeve=SLEEVE)


def turn(start, hinge, axis, angle, steps):
    """Points on the arc `start` makes turning by `angle` about the line
    through `hinge` along the unit vector `axis` (horizontal)."""
    sx, sy, sz = (start[i] - hinge[i] for i in range(3))
    ax, ay, _ = axis
    out = []
    for k in range(1, steps + 1):
        a = angle * k / steps
        c, s = math.cos(a), math.sin(a)
        # Rodrigues, for a horizontal axis.
        dot = sx * ax + sy * ay
        cx, cy, cz = ay * sz, -ax * sz, ax * sy - ay * sx
        out.append((
            hinge[0] + sx * c + cx * s + ax * dot * (1 - c),
            hinge[1] + sy * c + cy * s + ay * dot * (1 - c),
            hinge[2] + sz * c + cz * s,
        ))
    return out


def teach(scene: bt.Scene, marks: dict) -> dict:
    """The joint-space waypoints of both arms, solved by IK with the tool
    pointing down, each warm-started from the one before."""
    top = scene.frame("table/top")[0][2]

    def at(arm: str, position, seed):
        scene.set_joint_positions(seed, robot=arm)
        result = scene.set_tcp_target(position, DOWN, robot=arm)
        if not result.converged:
            raise SystemExit(f"the {arm} arm cannot reach {tuple(round(v, 3) for v in position)} "
                             f"({result.pos_error * 1000:.1f} mm short)")
        return list(scene.joint_positions_of(arm))[:6]

    taught = {}
    for arm, side in zip(ARMS, (1.0, -1.0)):
        cuff = next(marks[name] for name in ("cuff_left", "cuff_right")
                    if marks[name][1] * side > 0)
        hem = next(marks[name] for name in ("hem_left", "hem_right")
                   if marks[name][1] * side > 0)
        shoulder = next(marks[name] for name in ("shoulder_left", "shoulder_right")
                        if marks[name][1] * side > 0)
        # The sleeve turns about the body's side and the lower half about
        # the line half-way up the body — each hinge set back by the creep
        # of its crease.
        side_y = side * (BODY[0] / 2 + SLEEVE_CREEP)
        waist_x = (hem[0] + shoulder[0]) / 2 - HEM_CREEP
        grip_z = top + PINCH
        cuff_grip = (cuff[0], cuff[1], grip_z)
        hem_grip = (hem[0], hem[1], grip_z)
        reach = abs(cuff[1]) - abs(side_y)
        sleeve_turn = math.pi - math.asin(RELEASE / reach)
        rise = abs(waist_x - hem[0])
        hem_turn = math.pi - math.asin(RELEASE / rise)
        sleeve_arc = turn(cuff_grip, (cuff[0], side_y, grip_z), (side, 0.0, 0.0), sleeve_turn, 6)
        hem_arc = turn(hem_grip, (waist_x, hem[1], grip_z), (0.0, 1.0, 0.0), hem_turn, 10)

        seed = [*READY, OPEN]
        points = {}
        points["over_cuff"] = at(arm, (cuff_grip[0], cuff_grip[1], grip_z + HOVER), seed)
        points["cuff"] = at(arm, cuff_grip, [*points["over_cuff"], OPEN])
        q = points["cuff"]
        points["sleeve"] = []
        for p in sleeve_arc:
            q = at(arm, p, [*q, SHUT])
            points["sleeve"].append(q)
        points["clear_sleeve"] = at(arm, (sleeve_arc[-1][0], sleeve_arc[-1][1], grip_z + HOVER),
                                    [*q, OPEN])
        points["over_hem"] = at(arm, (hem_grip[0], hem_grip[1], grip_z + HOVER),
                                [*points["clear_sleeve"], OPEN])
        points["hem"] = at(arm, hem_grip, [*points["over_hem"], OPEN])
        q = points["hem"]
        points["fold"] = []
        for p in hem_arc:
            q = at(arm, p, [*q, SHUT])
            points["fold"].append(q)
        points["clear_fold"] = at(arm, (hem_arc[-1][0], hem_arc[-1][1], grip_z + HOVER), [*q, OPEN])
        taught[arm] = points
        scene.set_joint_positions([*READY, OPEN], robot=arm)
    return taught


def program(scene: bt.Scene, taught: dict) -> str:
    """One program drives both arms: every move is a joint ramp between
    taught points, timed so the cloth can follow. The sleeves go one after
    the other — the two hands would meet over the chest — and the hem is
    turned by both at once."""
    names = scene.robot.joint_names
    finger = names[-1]
    sq = scene.sequence("fold")

    def move(arms, name, key, seconds, grip, index=None):
        ramps = []
        for arm in arms:
            q = taught[arm][key] if index is None else taught[arm][key][index]
            ramps.append(bt.seq.ramp(dict(zip(names, [*q, grip])), seconds, robot=arm))
        sq.step(name, actions=ramps)

    def grip(arms, name, value, on, settle):
        sq.step(name,
                actions=[bt.seq.ramp({finger: value}, 0.4, robot=arm) for arm in arms]
                + [bt.seq.set_signal(f"grip_{arm}", on) for arm in arms],
                transition=bt.seq.all_of(bt.seq.done(), bt.seq.elapsed(settle)))

    for arm in ARMS:
        one = (arm,)
        move(one, f"{arm}: over the cuff", "over_cuff", 2.0, OPEN)
        move(one, f"{arm}: down to the cuff", "cuff", 1.0, OPEN)
        grip(one, f"{arm}: pinch the cuff", SHUT, True, 0.6)
        for k in range(len(taught[arm]["sleeve"])):
            move(one, f"{arm}: turn the sleeve {k + 1}", "sleeve", 0.7, SHUT, index=k)
        grip(one, f"{arm}: let the sleeve go", OPEN, False, 1.0)
        move(one, f"{arm}: clear the sleeve", "clear_sleeve", 1.0, OPEN)
        move(one, f"{arm}: over the hem", "over_hem", 2.0, OPEN)
    move(ARMS, "down to the hem", "hem", 1.0, OPEN)
    grip(ARMS, "pinch the hem", SHUT, True, 0.6)
    for k in range(len(taught[ARMS[0]]["fold"])):
        move(ARMS, f"turn the hem {k + 1}", "fold", 0.8, SHUT, index=k)
    grip(ARMS, "let the hem go", OPEN, False, 1.5)
    move(ARMS, "clear the fold", "clear_fold", 1.0, OPEN)
    sq.step("home",
            actions=[bt.seq.ramp(dict(zip(names, [*READY, OPEN])), 2.0, robot=arm) for arm in ARMS],
            transition=bt.seq.all_of(bt.seq.done(), bt.seq.elapsed(1.0)))
    return "fold"


def build(spacing: float = 0.02):
    """The cell with the shirt in it and the fold taught: `(scene, cloth,
    program name)`."""
    scene = build_cell()
    cloth = shirt(spacing)
    marks = bt.cloth.landmarks(scene, cloth)
    taught = teach(scene, marks)
    name = program(scene, taught)
    # Held on springs and let go softly: what the hem turn needs, and no
    # harm to a sleeve.
    grippers = [bt.cloth.gripper(f"grip_{arm}", robot=arm, radius=RADIUS,
                                 compliance=0.02, soften=3) for arm in ARMS]
    bt.cloth.add(scene, cloth, grippers)
    return scene, cloth, name


def measure(scene: bt.Scene, timeline) -> dict:
    """The fold in numbers: the shirt's footprint before and after, how
    high the pile stands, how far the hem is from the shoulders (positive:
    short of them) and the cuffs inside the body's edges."""
    track = timeline.cloth("shirt")
    top = scene.frame("table/top")[0][2]
    end = timeline.duration

    def footprint(points):
        return tuple(max(p[i] for p in points) - min(p[i] for p in points) for i in (0, 1))

    folded = track.positions(end)
    sides = ("left", "right")
    hem_short = sum(track.position(f"shoulder_{s}", end)[0] - track.position(f"hem_{s}", end)[0]
                    for s in sides) / 2
    cuffs_in = sum(BODY[0] / 2 - abs(track.position(f"cuff_{s}", end)[1]) for s in sides) / 2
    return {
        "flat": footprint(track.positions(0.0)),
        "folded": footprint(folded),
        "height": max(p[2] for p in folded) - top,
        "hem_short": hem_short,
        "cuffs_in": cuffs_in,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("out", nargs="?", default="tshirt_fold.usda")
    parser.add_argument("--studio", action="store_true")
    parser.add_argument("--spacing", type=float, default=0.02)
    args = parser.parse_args()

    scene, cloth, name = build(args.spacing)
    started = time.perf_counter()
    timeline = scene.simulate_sequence(name, max_duration=120.0)
    track = timeline.cloth("shirt")
    print(f"baked {timeline.duration:.1f} s of cycle with the shirt in "
          f"{time.perf_counter() - started:.1f} s ({len(track.positions(0.0))} vertices, "
          f"{len(track.times)} cloth samples)")
    for warning in track.warnings:
        print(f"  note: {warning}")
    if track.failure is not None:
        print(f"  the cloth stopped at {track.failure[0]:.2f} s: {track.failure[1]}")
    fold = measure(scene, timeline)
    print(f"  folded from {fold['flat'][0]:.2f} x {fold['flat'][1]:.2f} m to "
          f"{fold['folded'][0]:.2f} x {fold['folded'][1]:.2f} m, {1000 * fold['height']:.0f} mm high")
    print(f"  the hem lies {1000 * fold['hem_short']:+.0f} mm short of the shoulders, "
          f"the cuffs {1000 * fold['cuffs_in']:.0f} mm inside the body's edges")

    warnings = timeline.export_usd(args.out, fps=30.0)
    print(f"wrote {args.out}" + (f" ({len(warnings)} warnings)" if warnings else ""))

    if args.studio:
        bt.studio(scene, view=((1.5, -1.3, 1.75), (0.0, 0.0, 0.78)))


if __name__ == "__main__":
    main()

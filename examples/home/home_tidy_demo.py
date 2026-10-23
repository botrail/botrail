"""A home robot clears up after tea, in a flat furnished from SimReady Central.

The flat (`_house.py`) is a 7.2 x 4.8 m living-dining-kitchen whose
furniture and appliances are NVIDIA SimReady Central packages from four
publishers — NVIDIA's sofa and kettle, LG's refrigerator, dishwasher,
microwave, OLED TV and air purifier, ForgeCAD's kitchen cabinets, dining
chairs, glasses and bin, Palatial's range hood, faucet, coffee maker, lamp,
cabinet, plant and mugs — each fetched at the commit the library's index
registered, drawn as it ships (textures and MDL looks) and colliding as its
own geometry. The tables, the sink and the walls are built from parameters,
because those are the dimensions the tidy-up depends on.

The robot is a catalog Unitree G1-D: a wheeled base, a two-stage column, a
waist that bows, two 7-axis arms with three-finger hands. Two trips, both
hands at once:

  * **ローテーブル** — it drives from its dock, the head camera finds the two
    mugs, the torso bows over the 0.40 m table (teaching tries the gentlest
    bow first), each hand comes down onto its mug from above and lifts it,
    and torso and arms straighten together, the mugs held upright.
  * **シンク** — the column lifts the mugs clear of the worktop, the hands
    reach out over the basin, the column sinks and they go down onto its
    floor.
  * **ダイニング** — the same with the two glasses at the kitchen end of the
    table.
  * **シンク** — the glasses go down either side of the mugs; then back to
    the dock, which it reverses into.

Every arm motion is planned against the flat — the sofa, the chairs, the
basin's walls, the other arm — every ramp that moves the arms with the torso
is checked for what it touches and how far it tips the dishes, and the baked
run is replayed against the flat every 50 ms. What is printed is the cycle:
how long, what took the time, how far the dishes tipped, where each ended
up. The bill of materials is the flat's inventory: every SimReady package
with the URL it came from at its commit and its license (CC BY 4.0
throughout — the attribution travels with the cell).

Run with:  python examples/home/home_tidy_demo.py [out.usdc] [--plan plan.svg] [--studio]

`--plan FILE` writes the floor plan (the layout sheet: `.svg` or `.dxf`).
Needs `pip install botrail[catalog]`; the first run downloads the robot from
the botrail catalog and about 0.5 GB of SimReady packages into the Hugging
Face cache. Teaching and the bake take about half a minute.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import botrail as bt

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / "vehicles")]
import _house  # noqa: E402
from semi_humanoid_demo import G1D, quat_mul, rotate, yaw_quat  # noqa: E402

S = bt.seq
ROBOT, BASE = "robot", "base"
LINE = "cartesian_line"
SIDES = ("right", "left")
RESOLUTION = (1280, 720)
PLURAL = {"mug": "mugs", "glass": "glasses"}

# ------------------------------------------------------------- the route
# The robot stands this far from the edge of what it works at: its base is
# 0.57 m long, so a little over its half length.
STANDOFF = 0.40
DINING_STANDOFF = 0.34
SINK_STANDOFF = 0.36
CORRIDOR_Y = 1.0                  # the way along the south of the room
AISLE_X = 5.85                    # the kitchen aisle, between table and counter


def stations() -> dict:
    """Station -> ((x, y), heading): where the robot stands to work."""
    (ccx, ccy), (_, cly, _) = _house.COFFEE_TABLE
    (dcx, dcy), (dlx, _, _) = _house.DINING_TABLE
    sink_y, _ = _house.SINK
    counter = _house.W - _house.COUNTER_DEPTH
    return {
        "dock": ((_house.DOCK_X, 0.75), math.pi / 2),
        "coffee": ((ccx, ccy - cly / 2 - STANDOFF), math.pi / 2),
        "sink": ((counter - SINK_STANDOFF, sink_y), 0.0),
        "dining": ((dcx + dlx / 2 + DINING_STANDOFF, dcy), math.pi),
    }


def route() -> tuple[list, dict]:
    """One path, walked once: dock → coffee table → sink → dining table →
    sink → dock. A parked vehicle faces the leg it arrived by, so each
    station is the end of a leg pointing at what it works on; the points
    the path comes back through let it back out and pivot clear of the
    furniture, and the dock is reversed into, as it was left."""
    st = stations()
    (dock, _), (coffee, _), (sink, _), (dining, _) = st["dock"], st["coffee"], st["sink"], st["dining"]
    turn = (AISLE_X, sink[1])
    path = [dock, (dock[0], CORRIDOR_Y), (coffee[0], CORRIDOR_Y), coffee, (coffee[0], CORRIDOR_Y),
            (AISLE_X, CORRIDOR_Y), turn, sink, turn, dining, turn, sink, turn,
            (AISLE_X, CORRIDOR_Y), (dock[0], CORRIDOR_Y), dock]
    names = {"dock": 0, "coffee": 3, "sink": 7, "dining": 9, "sink2": 11, "home": 15}
    return path, names


# -------------------------------------------------------- the robot, built
def build() -> tuple[bt.Scene, _house.House]:
    robot = bt.Robot.from_catalog(G1D.package)
    wheels = bt.Wheels.from_catalog(G1D.package)
    scene = bt.Scene(robot, name=ROBOT)
    house = _house.build(scene)
    path, names = route()
    forward = {name: "forward" for name in names if name not in ("dock", "home")}
    scene.add_vehicle(BASE, body=[], path=path, stations=names, start="dock", speed=0.6, turn_speed=1.0,
                      drive=wheels.vehicle_drive, allow_reverse=True, arrive={**forward, "home": "reverse"})
    scene.mount_robot(BASE, robot=ROBOT, wheels=wheels)
    # The head camera finds the dishes before the hands go for them.
    link, offset, orientation, fov = G1D.camera
    scene.add_camera("head_cam", robot=ROBOT, link=link, position=offset, quaternion=orientation, fov=fov,
                     resolution=RESOLUTION, near=0.1, far=4.0, body_visible=False)
    for kind in ("mug", "glass"):
        watched = [obstacle for side in SIDES for obstacle in house.things[f"{kind}_{side}"]]
        scene.add_vision_sensor(f"sees_{PLURAL[kind]}", camera="head_cam", watch=watched)
    return scene, house


def joints(scene: bt.Scene, base: list, *targets: dict) -> list:
    names = scene.robot_of(ROBOT).joint_names
    q = list(base)
    for target in targets:
        for joint, value in target.items():
            q[names.index(joint)] = value
    return q


class Stand:
    """Stands the robot at a station for teaching — the base where the
    vehicle will carry it, the mount's offset and all — and puts it back."""

    def __init__(self, scene: bt.Scene):
        self.scene = scene
        self.base = scene.robot_base_pose_of(ROBOT)
        self.q = list(scene.joint_positions_of(ROBOT))
        (dock, heading) = stations()["dock"]
        world = (self.base[0][0] - dock[0], self.base[0][1] - dock[1], self.base[0][2])
        self.offset = rotate(yaw_quat(-heading), world)

    def at(self, station: str) -> float:
        (x, y), heading = stations()[station]
        dx, dy, dz = rotate(yaw_quat(heading), self.offset)
        self.scene.set_robot_base_pose((x + dx, y + dy, dz), yaw_quat(heading), robot=ROBOT)
        return heading

    def __enter__(self):
        return self

    def __exit__(self, *exc) -> None:
        self.scene.set_robot_base_pose(*self.base, robot=ROBOT)
        self.scene.set_joint_positions(self.q, robot=ROBOT)


# ------------------------------------------------------------- teaching
# The torso postures teaching may choose from, gentlest first: the G1-D
# makes height by its column and reaches down by bowing at the waist.
BOWS = [{"Yaw_Joint": bow} for bow in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0)]
# A hand from above holds a dish just under its rim — its palm is over the
# rim. Heights over the dish's underside: the mug is 80 mm tall, the glass 150.
MUG_GRIP = 0.058
GLASS_GRIP = 0.125
MUG_RADIUS = 0.034
# The G1-D's hand cannot open its fingers past straight: they lie 39 mm to
# the outside of its grasp centre, and a mug is 68 mm across. The hand holds
# a dish this far towards the inside, so the open fingers come down beside
# it, clear of its rim.
INSIDE = {"right": 0.022, "left": -0.022}


UPRIGHT = {"Yaw_Joint": 0.0}
# The hand takes a dish from above, pointing straight down, from the moment
# it takes it to the moment it lets go — and carries it that way, so the
# dish stays upright. A level hand would have its palm 12 cm and its wrist
# 20 cm behind what it holds, at the same height: in a sink that is the
# front wall, and over a low table one finger is under the other.
PITCH = math.pi / 2
APPROACH = (0.0, math.sin(PITCH / 2), 0.0, math.cos(PITCH / 2))
TURNS = (0.0, 0.3, -0.3, 0.6, -0.6, 0.9, -0.9)
# Over the sink the fingers must stay off the near side: the bow carries a
# hand forward over the basin's front wall, and fingers turned towards the
# robot would catch it. The fingers lie outboard of the dish; turning the
# right hand anticlockwise, the left clockwise, takes them away from the robot.
AWAY = {"right": (0.0, 0.3, 0.6, 0.9), "left": (0.0, -0.3, -0.6, -0.9)}
# The dishes go into the sink over the worktop: the column lifts them this
# far (two stages, half each) so that their undersides clear it, the hands
# reach out over the basin, and the column takes them straight down into it.
LIFT = 0.16
RAISED = {"LZ_mt_Joint": LIFT / 2, "LZ_it_Joint": LIFT / 2}
# How far a ramp that moves the arms with the torso may tip a held dish.
MAX_TILT = 15.0
# Where each hand carries a dish, in the robot's own frame (x ahead, y left,
# z over the floor): just ahead of the base, in front of the chest — high
# enough that the torso can bow over the dining table with a dish in each
# hand (at 0.80 m the hands go into its top).
CARRY = {"left": (0.26, 0.22, 0.90), "right": (0.26, -0.22, 0.90)}


def conj(q: tuple) -> tuple:
    return (-q[0], -q[1], -q[2], q[3])


def both(scene: bt.Scene, right: list, left: list) -> list:
    """One configuration: `right`, with the left arm and hand as in `left`."""
    names = scene.robot_of(ROBOT).joint_names
    q = list(right)
    for i, name in enumerate(names):
        if name.startswith("left_"):
            q[i] = left[i]
    return q


def hand_at(dish: tuple, heading: float, side: str) -> tuple:
    """Where the grasp centre goes for the hand to hold a dish centred at `dish`."""
    dx, dy, _ = rotate(yaw_quat(heading), (0.0, INSIDE[side], 0.0))
    return (dish[0] - dx, dish[1] - dy, dish[2])


def grips(scene: bt.Scene, house: _house.House) -> dict:
    """Where each hand takes each dish: the middle of its body, read off
    the placed package (a mug's handle is turned away from the robot)."""
    out = {}
    for side in SIDES:
        (lo, hi) = scene.obstacle_bounds(house.things[f"mug_{side}"][0])
        # The handle points north, away from the robot: the body is the south part of the box.
        out[f"mug_{side}"] = ((lo[0] + hi[0]) / 2, lo[1] + MUG_RADIUS, lo[2] + MUG_GRIP)
        boxes = [scene.obstacle_bounds(o) for o in house.things[f"glass_{side}"]]
        lo = [min(b[0][k] for b in boxes) for k in range(3)]
        hi = [max(b[1][k] for b in boxes) for k in range(3)]
        out[f"glass_{side}"] = ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2] + GLASS_GRIP)
    return out


def teach(scene: bt.Scene, house: _house.House) -> dict:
    """Every taught configuration by name, and the torso chosen per station.
    Each is solved with the robot standing where it will work and its torso
    where the ramp will have put it: an arm motion holds the torso still,
    so the two must agree. Every hand points straight down, so a dish taken
    from above is held upright in every taught pose."""
    poses: dict = {}

    def solve(arm: str, torso: dict, points: list, heading: float, seed: list, alone: bool = False,
              turns: tuple = TURNS) -> list:
        """The arm through `points` — where the dish's centre goes, one after
        the other (over it, then at it) — the hand pointing straight down
        and turned the same way for all of them, free of collision. One
        configuration per point. `alone`: the other arm is left out of the
        check — it is not where this solve leaves it, it works alongside."""
        other = "left_" if arm == "right" else "right_"
        short, touching, missed, unstraight = None, set(), points[0], False
        # Seeds: the last pose and the arm at rest, the package's arm
        # postures from each, and every pose this arm was already taught — a
        # hand pointing down is far from the postures a level hand starts from.
        taught = [q for name, q in poses.items() if f"_{arm}_" in name and isinstance(q, list)]
        starts = [seed, rest] + [joints(scene, q, s) for q in (seed, rest) for s in G1D.seeds[arm]] + taught
        for turn in turns:
            # The hand may turn about the vertical: a dish held from above
            # only turns about its own axis with it, and stays upright.
            yaw = heading + turn
            orientation = quat_mul(yaw_quat(yaw), APPROACH)
            for start in starts:
                q, out = joints(scene, start, torso, G1D.hands_open[arm]), []
                for point in points:
                    scene.set_joint_positions(q, robot=ROBOT)
                    result = scene.set_tcp_target(hand_at(point, yaw, arm), orientation, link=G1D.grasp[arm],
                                                  robot=ROBOT, group=arm, max_iters=300)
                    if not result.converged:
                        short, missed = result.pos_error if short is None else min(short, result.pos_error), point
                        break
                    hits = [(a, b) for a, b in scene.check_collisions()
                            if not (alone and (a[1].startswith(other) or b[1].startswith(other)))]
                    if hits:
                        touching.update(f"{a[1]} × {b[1]}" for a, b in hits)
                        missed = point
                        break
                    q = list(scene.joint_positions_of(ROBOT))
                    out.append(q)
                if len(out) == len(points):
                    if all(straight(arm, a, b) for a, b in zip(out, out[1:])):
                        return out
                    unstraight = True
        where = ", ".join(f"{v:.2f}" for v in missed)
        raise LookupError(f"the {arm} hand cannot get to ({where})"
                          + (f", {short * 1e3:.0f} mm short" if short is not None and not touching else "")
                          + (f"; it touches {', '.join(sorted(touching)[:3])}" if touching else "")
                          + ("; or not down a straight line" if unstraight else ""))

    def straight(arm: str, a: list, b: list) -> bool:
        """Does a straight line take the hand from `a` to `b` — as the
        cycle's `cartesian_line` segments will?"""
        scene.set_joint_positions(a, robot=ROBOT)
        scene.add_segment("_line", goal=b, kind=LINE, group=arm, robot=ROBOT)
        try:
            scene.plan_motion("_line", broadcast=False)
            return True
        except (RuntimeError, ValueError):
            return False
        finally:
            scene.remove_motion("_line")

    def task(station: str, heading: float, strokes: dict, together: bool = False, turns=None) -> dict:
        """One torso posture for every stroke (`(names) -> (arm, points)`):
        the gentlest bow that reaches them all. `together`: the two hands
        work at the same time — each is solved on its own, then the pairs of
        poses named alike (`..._right_over` with `..._left_over`) are checked
        side by side. `turns`: per arm, the hand turns allowed here."""
        refused = []
        for torso in BOWS:
            solved, seeds = {}, {}
            try:
                for names, (arm, points) in strokes.items():
                    qs = solve(arm, torso, points, heading, seeds.get(arm, poses["travel"]), alone=together,
                               turns=turns[arm] if turns else TURNS)
                    solved.update(zip(names, qs))
                    seeds[arm] = qs[-1]
                if together:
                    for name in [n for n in solved if "_right_" in n]:
                        scene.set_joint_positions(both(scene, solved[name], solved[name.replace("_right_", "_left_")]),
                                                  robot=ROBOT)
                        hits = scene.check_collisions()
                        if hits:
                            raise LookupError(f"side by side at `{name.replace('_right', '')}` the hands touch "
                                              + ", ".join(sorted(f"{a[1]} × {b[1]}" for a, b in hits)[:3]))
            except LookupError as err:
                refused.append(f"{torso}: {err}")
                continue
            poses.update(solved)
            poses[f"torso_{station}"] = torso
            return torso
        raise RuntimeError(f"no torso posture works at the {station}:\n  " + "\n  ".join(refused))

    at = grips(scene, house)
    rest = joints(scene, list(scene.joint_positions_of(ROBOT)), G1D.travel)
    with Stand(scene) as stand:
        # The carry: both hands in front of the chest, as the robot travels.
        heading = stand.at("dock")
        poses["travel"] = rest
        for arm in SIDES:
            point = tuple(p + d for p, d in zip(stations()["dock"][0] + (0.0,), rotate(yaw_quat(heading), CARRY[arm])))
            poses["travel"] = solve(arm, G1D.travel, [point], heading, poses["travel"])[0]

        # The coffee table, then the dining table: each hand comes down onto
        # its dish from above.
        for station, kind in (("coffee", "mug"), ("dining", "glass")):
            heading = stand.at(station)
            strokes = {}
            for side in SIDES:
                x, y, z = at[f"{kind}_{side}"]
                strokes[(f"{kind}_{side}_over", f"{kind}_{side}_grip")] = (side, [(x, y, z + 0.10), (x, y, z)])
            task(station, heading, strokes)

        # The sink, twice, both hands at once: each hand over its spot
        # inside the basin with the torso bowed (`_into`) — the basin's
        # floor is out of reach standing up — and on the floor (`_seat`).
        # How the dishes get there is the cycle's: over the worktop on the
        # raised column, then down into the basin as the column sinks.
        heading = stand.at("sink")
        for kind, grip_at in (("mug", MUG_GRIP), ("glass", GLASS_GRIP)):
            strokes = {}
            for side in SIDES:
                x, y, floor = house.sink_spots[f"{kind}_{side}"]
                seat = (x, y, floor + grip_at + 0.003)
                strokes[(f"{kind}_{side}_into", f"{kind}_{side}_seat")] = (side, [(x, y, seat[2] + 0.08), seat])
            task(f"sink_{kind}", heading, strokes, together=True, turns=AWAY)
    scene.set_joint_positions(poses["travel"], robot=ROBOT)
    return poses


def sweep(scene: bt.Scene, house: _house.House, poses: dict, station: str, a: list, b: list,
          holding: tuple = (), samples: int = 24) -> tuple[list, float]:
    """What a ramp from `a` to `b` at `station` would touch, sampled along
    its ease, and how far (degrees) the hands holding the dishes in
    `holding` (`(thing, side, where it was taken, the grip pose)`) turn
    from straight down on the way — the dishes held the way they were
    taken. A ramp is not planned, so its sweep is checked here, before the
    bake."""
    hits, tilt = set(), 0.0
    held = {o: scene.obstacle_pose(o) for thing, *_ in holding for o in house.things[thing]}
    with Stand(scene) as stand:
        try:
            for thing, side, taken_at, grip in holding:
                stand.at(taken_at)
                scene.set_joint_positions(poses[grip], robot=ROBOT)
                for o in house.things[thing]:
                    scene.attach(o, robot=ROBOT, group=side, touch_links=G1D.touch[side])
            stand.at(station)
            for k in range(samples + 1):
                u = k / samples
                ease = u * u * (3 - 2 * u)
                q = [x + (y - x) * ease for x, y in zip(a, b)]
                scene.set_joint_positions(q, robot=ROBOT)
                hits.update((p[1], o[1]) for p, o in scene.check_collisions())
                for _, side, *_ in holding:
                    _, rot = scene.link_pose_at(G1D.grasp[side], q, robot=ROBOT)
                    down = -rotate(rot, (1.0, 0.0, 0.0))[2]
                    tilt = max(tilt, math.degrees(math.acos(max(-1.0, min(1.0, down)))))
        finally:
            for o, pose in held.items():
                scene.detach(o)
                scene.set_obstacle_pose(o, *pose)
    return sorted(hits), tilt


# ----------------------------------------------------------------- the cycle
def build_cycle(scene: bt.Scene, house: _house.House, poses: dict) -> str:
    travel = poses["travel"]
    torso_s, hand_s = 2.0, 0.4
    model = scene.robot_of(ROBOT)
    body = [j for g in ("torso", "right", "left") for j in model.group(g).joints]

    def under(station: str) -> list:
        """The carry, with the torso where it is at `station`."""
        return joints(scene, travel, poses[f"torso_{station}"])

    def pair(tag: str) -> list:
        """Both hands at their `<kind>_<side>_<tag>` poses."""
        kind, step = tag.split("_")
        return both(scene, poses[f"{kind}_right_{step}"], poses[f"{kind}_left_{step}"])

    def raised(q: list) -> list:
        return joints(scene, q, RAISED)

    # A torso that bows or straightens with dishes in the hands moves the
    # arms with it, in one ramp — torso, column and both arms on one joint
    # line between two poses that hold the hands straight down. Every such
    # ramp is checked here: what it touches, and how far it tips the dishes.
    mugs = tuple((f"mug_{side}", side, "coffee", f"mug_{side}_grip") for side in SIDES)
    glasses = tuple((f"glass_{side}", side, "dining", f"glass_{side}_grip") for side in SIDES)
    tilts = {}
    for label, station, a, b, holding in (
        ("bowing over the coffee table", "coffee", travel, under("coffee"), ()),
        ("straightening with the mugs", "coffee", pair("mug_over"), travel, mugs),
        ("raising the mugs", "sink", travel, raised(travel), mugs),
        ("reaching over the sink with the mugs", "sink", raised(travel), raised(pair("mug_into")), mugs),
        ("lowering the mugs into the sink", "sink", raised(pair("mug_into")), pair("mug_into"), mugs),
        ("straightening at the sink", "sink", pair("mug_into"), joints(scene, pair("mug_into"), UPRIGHT), ()),
        ("leaning over the dining table", "dining", travel, under("dining"), ()),
        ("straightening with the glasses", "dining", pair("glass_over"), travel, glasses),
        ("raising the glasses", "sink", travel, raised(travel), glasses),
        ("reaching over the sink with the glasses", "sink", raised(travel), raised(pair("glass_into")), glasses),
        ("lowering the glasses into the sink", "sink", raised(pair("glass_into")), pair("glass_into"), glasses),
        ("straightening at the sink again", "sink", pair("glass_into"), joints(scene, pair("glass_into"), UPRIGHT), ()),
    ):
        hits, tilt = sweep(scene, house, poses, station, a, b, holding)
        if hits:
            raise RuntimeError(f"{label} sweeps through the flat: {hits[:3]}")
        if tilt > MAX_TILT:
            raise RuntimeError(f"{label} tips the dishes {tilt:.0f}° (at most {MAX_TILT:.0f}°)")
        tilts[label] = tilt

    def seg(motion: str, side: str, pose, kind: str = "joint") -> None:
        goal = poses[pose] if isinstance(pose, str) else pose
        scene.add_segment(motion, goal=goal, kind=kind, group=side, robot=ROBOT)

    for side in SIDES:
        for kind in ("mug", "glass"):
            seg(f"pick_{kind}_{side}", side, f"{kind}_{side}_over")
            seg(f"pick_{kind}_{side}", side, f"{kind}_{side}_grip", LINE)
            seg(f"lift_{kind}_{side}", side, f"{kind}_{side}_over", LINE)   # and wait there, the dish held up
            seg(f"place_{kind}_{side}", side, f"{kind}_{side}_seat", LINE)  # down onto the basin's floor
            seg(f"rise_{kind}_{side}", side, f"{kind}_{side}_into", LINE)
            seg(f"clear_{kind}_{side}", side, travel)                       # standing: back to the carry

    sq = scene.sequence("tidy")

    def ramp(*targets: dict) -> list:
        merged = {joint: value for target in targets for joint, value in target.items()}
        return [S.ramp(merged, torso_s, robot=ROBOT)] if merged else []

    def whole(q: list) -> dict:
        """Torso, column and both arms to `q`, the hands as they are."""
        names = model.joint_names
        return {joint: q[names.index(joint)] for joint in body}

    def hand(side: str, shut: bool) -> list:
        closed = G1D.hands_shut[side]
        opened = {**{joint: 0.0 for joint in closed}, **G1D.hands_open[side]}
        return [S.ramp(closed if shut else opened, hand_s, robot=ROBOT)]

    def step(name: str, actions: list, *also) -> None:
        moving = [S.done()] if any(a["type"] in ("start_ramp", "start_motion") for a in actions) else []
        waits = [*moving, *also] or [S.immediately()]
        sq.step(name, actions=actions, transition=waits[0] if len(waits) == 1 else S.all_of(*waits))

    def grasp(thing: str, side: str) -> list:
        return [*(S.attach(o, robot=ROBOT, group=side, touch_links=G1D.touch[side]) for o in house.things[thing]),
                *hand(side, True)]

    def release(thing: str, side: str) -> list:
        return [*(S.detach(o) for o in house.things[thing]), *hand(side, False)]

    def pick_up(kind: str, station: str, bow: str) -> None:
        plural = PLURAL[kind]
        step(f"find the {plural}", [], S.signal(f"sees_{plural}"))
        step(bow, ramp(poses[f"torso_{station}"]))
        for side in SIDES:
            step(f"pick {side} {kind}", [S.motion(f"pick_{kind}_{side}")])
            step(f"grasp {side} {kind}", grasp(f"{kind}_{side}", side))
            step(f"lift {side} {kind}", [S.motion(f"lift_{kind}_{side}")])
        step(f"straighten with the {plural}", ramp(whole(travel)))

    def at_the_sink(kind: str) -> None:
        plural = PLURAL[kind]
        step(f"raise the {plural}", ramp(RAISED))
        step(f"reach over the sink ({plural})", ramp(whole(raised(pair(f"{kind}_into")))))
        step(f"lower the {plural} into the sink", ramp({joint: 0.0 for joint in RAISED}))
        step(f"put the {plural} down", [S.motion(f"place_{kind}_right"), S.motion(f"place_{kind}_left")])
        step(f"let go of the {plural}", [*release(f"{kind}_right", "right"), *release(f"{kind}_left", "left")])
        step(f"lift the hands ({plural})", [S.motion(f"rise_{kind}_right"), S.motion(f"rise_{kind}_left")])
        step(f"straighten ({plural})", ramp(UPRIGHT))
        step(f"hands back ({plural})", [S.motion(f"clear_{kind}_right"), S.motion(f"clear_{kind}_left")])

    sq.step("to the coffee table", actions=[S.goto(BASE, "coffee")], transition=S.device_done(BASE))
    pick_up("mug", "coffee", "bow")
    step("to the sink", [S.goto(BASE, "sink")], S.device_done(BASE))
    at_the_sink("mug")
    step("to the dining table", [S.goto(BASE, "dining")], S.device_done(BASE))
    pick_up("glass", "dining", "lean")
    step("back to the sink", [S.goto(BASE, "sink2")], S.device_done(BASE))
    at_the_sink("glass")
    step("to the dock", [S.goto(BASE, "home")], S.device_done(BASE))
    poses["tilts"] = tilts
    return "tidy"


def audit(scene: bt.Scene, house: _house.House, tl, dt: float = 0.05) -> list:
    """The baked run replayed against the flat every `dt`: the robot where
    the timeline has it, the dishes where they were, and every contact the
    robot makes that is not a hand on a dish — the bake checks the planned
    moves, this checks everything in between (drives, ramps, hand-overs)."""
    dishes = [o for obstacles in house.things.values() for o in obstacles]
    was = [(o, scene.obstacle_pose(o)) for o in dishes]
    base, q = scene.robot_base_pose_of(ROBOT), list(scene.joint_positions_of(ROBOT))
    hand = tuple(f"{side}_hand_" for side in SIDES)
    met: dict = {}
    try:
        for i in range(int(tl.duration / dt) + 1):
            t = i * dt
            pose = tl.base_pose(t, ROBOT)
            if pose:
                scene.set_robot_base_pose(*pose, robot=ROBOT)
            scene.set_joint_positions(tl.sample(t, ROBOT), robot=ROBOT)
            for o in dishes:
                at = tl.object_pose(o, t)
                if at:
                    scene.set_obstacle_pose(o, *at)
            for a, b in scene.check_collisions():
                if a[1].startswith(hand) and b[1] in dishes:
                    continue
                pair = (a[1], b[1])
                if pair not in met:
                    met[pair] = (round(t, 2), next((n for n, x, y in tl.step_spans if x <= t < y), "?"), pair)
    finally:
        for o, pose in was:
            scene.set_obstacle_pose(o, *pose)
        scene.set_robot_base_pose(*base, robot=ROBOT)
        scene.set_joint_positions(q, robot=ROBOT)
    return sorted(met.values())


def bake():
    """The flat, taught and baked: `(scene, house, poses, timeline)`."""
    scene, house = build()
    poses = teach(scene, house)
    name = build_cycle(scene, house, poses)
    return scene, house, poses, scene.simulate_sequence(name, max_duration=300.0)


def main() -> None:
    args = sys.argv[1:]
    plan = args[args.index("--plan") + 1] if "--plan" in args else None
    out = next((a for a in args if not a.startswith("--") and a != plan), "home_tidy.usdc")
    try:
        scene, house, poses, tl = bake()
    except (RuntimeError, ValueError) as err:
        sys.exit(f"the tidy-up does not work: {err}")

    print(f"cycle: {tl.duration:.1f} s")
    for step, start, end in tl.step_spans:
        print(f"  {step:<32} {start:6.1f} – {end:6.1f} s")
    print("torso bow: " + ", ".join(f"{s} {poses[f'torso_{s}']['Yaw_Joint']:.1f} rad"
                                    for s in ("coffee", "sink_mug", "dining", "sink_glass")))
    label, worst = max(((k, v) for k, v in poses["tilts"].items() if v > 0.0), key=lambda kv: kv[1])
    print(f"the dishes stay within {worst:.0f}° of upright while held (most: {label})")
    middles = grips(scene, house)
    for thing, obstacles in house.things.items():
        # The dish's middle, carried along from where it stood to where it ends.
        p0, q0 = scene.obstacle_pose(obstacles[0])
        local = rotate(conj(q0), tuple(a - b for a, b in zip(middles[thing], p0)))
        p1, q1 = tl.object_pose(obstacles[0], tl.duration)
        x, y, _ = (a + b for a, b in zip(p1, rotate(q1, local)))
        spot = house.sink_spots[thing]
        print(f"  {thing:<12} ends {math.dist((x, y), spot[:2]) * 1e3:.0f} mm from its spot in the sink, "
              f"on the floor of the basin at {p1[2]:.2f} m")
    met = audit(scene, house, tl)
    print("replayed every 50 ms: " + ("no contact" if not met else f"{len(met)} contacts"))
    for t, step, (a, b) in met[:8]:
        print(f"  {t:6.2f} s  {step:<32} {a} x {b}")
    rows = [r for r in scene.bom().rows if r["attributes"].get("source")]
    print(f"\n{len(rows)} SimReady packages furnish the flat:")
    for r in rows:
        a = r["attributes"]
        print(f"  {r['qty']} x {r['model']:<26} {r['manufacturer']:<15} {a.get('license', ''):<10} {r['category']}")

    tl.export_usd(out, fps=30)
    print(f"wrote {out}")
    if plan:
        scene.export_layout(plan, scale=120, title="LDK 7.2 x 4.8 m")
        print(f"wrote {plan}")
    if "--studio" in args:
        bt.studio(scene, view=((3.6, -1.6, 6.4), (3.6, 2.5, 0.3)))


if __name__ == "__main__":
    main()

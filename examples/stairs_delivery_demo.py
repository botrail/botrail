"""A catalog quadruped carries a tote up a catalog stair flight.

Both ends of this cell are ordered, and the point of the demo is that they
have to agree with each other:

* the dog is `unitree/go2/go2` — the catalog states the step it can take
  (`max_step_height_mm`, 160 mm), and `bt.Gait.from_catalog` turns that into
  the gait's `max_step`;
* the flight is `botrail/stairs/steel-flight` — a steel stair unit with
  checker-plate treads, plate stringers, levelling feet and a tubular
  handrail in safety orange, sold in rises from 80 to 200 mm.

Order the standard 175 rise and the bake refuses it by name, before the dog
takes a step: *this* machine cannot climb *that* stair (`--tall`). Order the
150 it is rated for and it walks up with the tote on its back. Nothing about
the walk is authored — the treads are walkable, so the footfalls land on them
instead of on the ramp the guide path interpolates; the body tilts onto the
pitch and rides up with the steps (held level, or held at a fixed height, the
downhill legs would want more reach than a real one has); and the legs do the
rest. What the cell does state is the stair *posture* — lower, with a shorter
swing — because the catalog carries only how a machine stands on the floor.

    python examples/stairs_delivery_demo.py            # bake + USD
    python examples/stairs_delivery_demo.py --tall     # the refusal, named
    python examples/stairs_delivery_demo.py --studio   # watch it climb

`--robot quad` runs the primitive quadruped (no download) — a shorter-legged
machine than the Go2, which the bake will tell you: it refuses this flight
and takes `--rise 120`. `--robot <dir>` runs a package the catalog builder
wrote, and `--flight <dir>` the same for the stairs.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import botrail as bt

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import legged_patrol_demo as patrol  # noqa: E402  (the Go2, and its fallbacks)

FLIGHT = "botrail/stairs/steel-flight"  # the catalog's steel stair unit
STEPS, TREAD, WIDTH = 6, 0.40, 0.90
RISE = 0.15  # a real riser, and inside the dog's 160 mm rating. The deep
# tread is what this machine needs to take it: its feet are 0.39 m apart
# fore and aft, so on a shallower tread the front and rear pairs land more
# than a riser apart and the legs run out of range (the bake says so).
# On a flight a machine does not stand as it does on the floor: it lowers
# its body (so the legs keep range at *both* ends — reaching down to a low
# tread, folding up under a high one) and picks its feet up less. These are
# the two numbers that decide whether a real riser can be climbed at all;
# the catalog carries only the standing stance, so the cell states them.
STAIR_DEPTH = 0.25   # foot below the hip, against 0.311 standing
STAIR_LIFT = 0.015   # swing apex over the nosing
FOOT_X = 1.5  # where the flight stands
MAX_GRADE = 0.7  # the drive may take the pitch; the *step* check answers
# A flight is taken slower than a floor: at the pitch, a step is the
# horizontal stride *and* the riser, and the legs have to reach both.
CLIMB_SPEED = 0.30
MAX_STEP = 0.160  # Go2's rated step, for packs that do not state their own
_TOLD: set = set()


def order_flight(scene, rise: float, tread: float = TREAD,
                 pack: str | Path | None = FLIGHT):
    """The stair unit. From the catalog where it can be reached — the pack
    then decides the sections, the part numbers and the two BOM lines (a
    flight, and a handrail per side) — else the same flight drawn from its
    dimensions. `pack=None` skips the catalog entirely.

    The catalog is consulted first so the two failures stay apart: a pack
    that cannot be *reached* falls back quietly, while a size the pack does
    not *sell* is an order the caller has to hear about.
    """
    spec = None
    if pack is not None:
        from botrail._spec import Spec

        try:
            spec = Spec.load(pack)
        except Exception as err:  # noqa: BLE001 - unreachable catalog, not a bad order
            if "flight" not in _TOLD:
                _TOLD.add("flight")
                print(f"catalog {pack} unavailable ({patrol.first_line(err)}); "
                      "drawing the flight from its dimensions")
    if spec is not None:
        return bt.parts.stairs(
            scene, "flight", catalog=pack, steps=STEPS, rise=rise,
            tread=tread, width=WIDTH, position=(FOOT_X, 0.0),
        )
    return bt.parts.stairs(
        scene, "flight", steps=STEPS, rise=rise, tread=tread, width=WIDTH,
        position=(FOOT_X, 0.0), detail="full", manufacturer="botrail",
        model=f"SF-{rise * 1000:.0f}x{tread * 1000:.0f}x{WIDTH * 1000:.0f}-{STEPS}",
    )


def rate_step(robot: str, gait):
    """The step this machine is rated for, when its package does not say.

    The rating belongs in the pack (`specs.max_step_height_mm`) and the Go2
    recipe states it — but the *published* Go2 predates that field, so a
    downloaded one arrives without it and the step check would never be
    armed. State the datasheet figure rather than let the demo quietly lose
    one of its two checks; only for the machine it is the datasheet *of*,
    though. Another package that says nothing is answered by the IK, which
    is the backstop for exactly this.
    """
    if robot == "go2" and gait.max_step is None:
        gait.max_step = MAX_STEP
    return gait


def stance_depth(model, gait) -> float:
    """How far the gait's stance hangs a foot below the body."""
    leg = next(iter(gait.legs.values()), None)
    if isinstance(leg, (tuple, list)):
        leg = leg[0]
    probe = bt.Scene(model, name="probe")
    stand = [gait.stance.get(n, 0.0) for n in model.joint_names]
    (_x, _y, z), _q = probe.link_pose_at(leg, stand)
    return -z


def stair_fold(model, gait, depth: float = STAIR_DEPTH) -> float | None:
    """The thigh angle that stands this machine `depth` off its treads.

    A leg folded thigh θ / calf −2θ hangs straight down, so one number is
    the whole posture — but how deep it hangs is the machine's business,
    and a standing stance is not a clean fold (the Go2 stands at 0.8 / −1.5,
    its foot 0.178 m ahead of the hip). So measure rather than assume:
    stand a scratch copy at a fold, read a foot, and bisect on the fold
    until the foot is where the stair posture wants it. `None` when the
    machine cannot reach that depth at all, or names its joints its own
    way — the caller then leaves the standing stance alone and the checks
    say what it cannot climb.
    """
    leg = next(iter(gait.legs.values()), None)
    if isinstance(leg, (tuple, list)):
        leg = leg[0]
    thighs = [j for j in gait.stance if "thigh" in j]
    calves = [j for j in gait.stance if "calf" in j or "knee" in j]
    if leg is None or not thighs or not calves:
        return None

    probe = bt.Scene(model, name="probe")
    names = model.joint_names

    def depth_at(fold: float) -> float:
        pose = dict(gait.stance)
        pose.update({j: fold for j in thighs})
        pose.update({j: -2.0 * fold for j in calves})
        (_x, _y, z), _q = probe.link_pose_at(leg, [pose.get(n, 0.0) for n in names])
        return -z

    lo, hi = 0.0, 1.4  # straight leg (deepest) .. tucked right up
    if not depth_at(lo) >= depth >= depth_at(hi):
        return None  # out of the fold's range: keep the stance it came with
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if depth_at(mid) > depth:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def stair_gait(gait, model=None):
    """The gait, in its stair posture: the same legs, lower and with a
    shorter swing.

    A package that carries `locomotion.stairs` arrives in that posture
    already — the machine's own measured one — and this leaves it alone.
    Anything else is solved here from the depth, and a machine whose stance
    is not named the usual way keeps the one it came with (the checks then
    say what it cannot climb).
    """
    if model is not None and stance_depth(model, gait) <= STAIR_DEPTH + 1e-3:
        return gait  # already on a flight's posture (the package stated one)
    thigh = stair_fold(model, gait) if model is not None else None
    if thigh is None:
        return gait
    posture = dict(gait.stance)
    for joint in posture:
        if "thigh" in joint:
            posture[joint] = thigh
        elif "calf" in joint or "knee" in joint:
            posture[joint] = -2.0 * thigh
    gait.stance, gait.lift = posture, STAIR_LIFT
    return gait


def build(*, robot: str = "go2", rise: float = RISE, tread: float = TREAD,
          pack: str | Path | None = FLIGHT) -> bt.Scene:
    # Standing first, because that is the pose `specs.height_mm` is measured
    # in: what the deck needs is the machine's shape *above its root*, and
    # that part does not fold with the legs. Take the difference now, before
    # the posture changes how high the root rides.
    model, standing, footprint, speed, _turn = patrol.dog_of(robot)
    over_root = footprint[2] - (stance_depth(model, standing) + standing.foot_radius)

    # Then the package's own stair posture where it has one; else solved
    # from the depth, which is the number that transfers between machines.
    _m, gait, *_rest = patrol.dog_of(robot, posture="stairs")
    gait = rate_step(robot, stair_gait(gait, model))
    # Where the machine's back actually is, in the posture it will walk in.
    # Read off the standing height instead and the tote rides 60 mm of air.
    back = stance_depth(model, gait) + gait.foot_radius + over_root
    scene = bt.Scene(model, name="dog")

    # The yard floor is walkable too: a foot outside the flight stands on
    # the ground, not on the ramp the guide path interpolates.
    floor = scene.add_box("yard", size=(9.0, 4.0, 0.05), position=(3.0, 0.0, -0.025),
                          color=(0.30, 0.31, 0.32))
    scene.set_obstacle_walkable(floor, True)
    # Ground, not an obstacle: a machine standing on the floor is not a
    # collision, and the studio draws its own floor under this one.
    scene.set_obstacle_enabled(floor, False)

    order_flight(scene, rise, tread, pack)
    foot_p, _ = scene.frame("flight/foot")
    top_p, _ = scene.frame("flight/top")
    top_z = top_p[2]

    # The mezzanine the flight lands on, tucked under the top tread's
    # nosing so the seam has no gap a foot could straddle.
    deck = scene.add_box("mezz/deck", size=(1.8, 2.2, 0.08),
                         position=(top_p[0] + 0.84, 0.0, top_z - 0.04),
                         color=bt.parts.CHECKER_PLATE)
    scene.set_obstacle_walkable(deck, True)
    # A column at each corner. They stand clear of everything that moves —
    # the dog climbs along y = 0, the handrails end at y = +-0.5 — so all
    # four are honest structure, not scenery trimmed to the camera.
    for i, (sx, sy) in enumerate(
        ((-0.86, -0.95), (0.86, -0.95), (-0.86, 0.95), (0.86, 0.95))
    ):
        scene.add_box(f"mezz/leg{i}", size=(0.09, 0.09, top_z - 0.08),
                      position=(top_p[0] + 0.84 + sx, sy, (top_z - 0.08) / 2),
                      color=bt.parts.STEEL)

    # The delivery, riding the dog's back: being in the tray zone is what
    # makes it cargo — there is no load action to author.
    scene.add_box("tote", size=(0.26, 0.22, 0.14), position=(0.0, 0.0, back + 0.07),
                  color=bt.parts.WOOD)

    # The guide path: the floor, a short ramp onto the flight, the pitch of
    # the flight itself, then the landing. The climbing leg runs level with
    # the *middle* of the steps (half a rise above the nosing line's foot),
    # so no leg has to reach more than half a riser off the body plane.
    scene.add_vehicle(
        "dog", body=[],
        path=[
            (0.0, 0.0, 0.0),
            (foot_p[0] - 0.45, 0.0, 0.0),
            (foot_p[0], 0.0, rise / 2),
            (top_p[0], 0.0, top_z + rise / 2),
            (top_p[0] + 0.95, 0.0, top_z),
        ],
        stations={"yard": 0, "mezz": 4},
        speed=min(speed, CLIMB_SPEED), start="yard", max_grade=MAX_GRADE,
        tray_position=(0.0, 0.0, back + 0.06), tray_size=(0.5, 0.4, 0.26),
    )
    scene.mount_robot("dog", gait=gait)

    seq = scene.sequence("deliver")
    seq.step("climb", actions=[bt.seq.goto("dog", "mezz")],
             transition=bt.seq.device_done("dog"))
    seq.step("handover", transition=bt.seq.elapsed(2.0))
    return scene


def bake(*, robot: str = "go2", rise: float = RISE, tread: float = TREAD,
         pack: str | Path | None = FLIGHT):
    scene = build(robot=robot, rise=rise, tread=tread, pack=pack)
    return scene, scene.simulate_sequence("deliver", max_duration=150.0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", nargs="?", default=str(HERE / "stairs_cell.usdc"))
    parser.add_argument("--robot", default="go2",
                        help="go2 (the catalog dog), quad, or a package directory")
    parser.add_argument("--flight", default=FLIGHT,
                        help="the stair pack: a catalog id or a package directory")
    parser.add_argument("--rise", type=float, default=RISE * 1000.0, metavar="MM",
                        help="the riser to order, in mm (the pack sells 80..200)")
    parser.add_argument("--tall", action="store_true",
                        help="order the 175 mm rise: over the dog's rating, refused by name")
    parser.add_argument("--studio", action="store_true")
    args = parser.parse_args()

    if args.tall:
        # 175 x 300 is a standard flight the pack sells (2R + T = 650 mm, a
        # comfortable stair for a person) — it is the *dog* that cannot take
        # it. Ordering it at the demo's own 400 tread would be refused one
        # step earlier, by the flight's walk rule, and the machine's rating
        # would never be consulted.
        try:
            bake(robot=args.robot, rise=0.175, tread=0.30, pack=args.flight)
        except ValueError as err:
            print("refused, as it should be:")
            print(f"  {err}")
            return
        raise SystemExit("a 175 mm riser against a 160 mm step should have been refused")

    scene, tl = bake(robot=args.robot, rise=args.rise / 1000.0, pack=args.flight)
    for row in scene.bom().rows:
        mass = row["attributes"].get("mass_kg")
        print(f"  {row['names'][0]:<18} {row['category']:<22} x{row['qty']:<3} "
              f"{row.get('model') or '':<20} {f'{mass} kg' if mass else ''}")
    steps = tl.footfalls("dog")
    print(f"cycle {tl.duration:.2f}s, {len(steps)} footfalls, "
          f"highest foothold z = {max(f[3][2] for f in steps):.3f}")
    p, _ = tl.object_pose("tote", tl.duration)
    print(f"tote delivered at ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})")
    tl.export_usd(args.out, fps=60)
    print(f"wrote {args.out}")
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()

"""An inventory drone shares a cell with a working arm.

The machine is `px4/x500/x500` from the catalog — the PX4 reference 500-class
airframe, carbon frame, four motors, four props and the landing gear it stands
on, with the rates it is capable of on its manifest. It is a *robot*, mounted
rigidly on the aerial vehicle — the exact symmetry of a quadruped with a gait
on a differential one — so its interference is computed at link level: rotors
against the shelving while it flies, links against the parked arm's links
every tick, and the live distance readout while you author. The `--low`
refusal names both links.

The drone is the same `Vehicle` every AGV is — a body, an authored path,
stations, `goto` / `device_done` — with an aerial drive: z is its own axis,
so the path climbs freely (no `max_grade`, no lift), and each leg's clock
is the slower axis, `max(run / speed, rise / climb (or descent))`, closed
form. There is no takeoff command: the pad station under an overhead
waypoint *is* the takeoff, as a vertical leg.

The point of the cell is the corridor. The outbound leg crosses the arm's
bench at working height — inside the arm's reach *by design*, with the
arm's tool a quarter metre above it when raised. Geometry alone would
forbid this cell; what permits it is time. The arm tends its bench, stows,
and raises `arm_clear`; the drone holds at the gate until it does. Run
`--no-interlock` and nothing about the paths or the volumes changes — only
the clock — and the bake refuses at the instant the two machines meet,
naming a link of each. That is what the cross-robot check is *for*: not
proving a flight clears parked scenery (any obstacle does that), but
pricing two machines' motions against each other in time.

The onboard scanner is an ordinary mounted zone sensor — it rides the
airframe (Z0's vehicle track) and blips once over every rack, which is the
whole survey as timing-chart lanes. And the airspace is checked like any
aisle: fly the route low (`--low`) and the drone crosses the parked arm —
`VehicleRobotCollision` names the link, before anything is flown for real.

    python examples/drone_survey_demo.py                 # bake + USD
    python examples/drone_survey_demo.py --no-interlock  # right paths, wrong clock
    python examples/drone_survey_demo.py --low           # wrong corridor outright
    python examples/drone_survey_demo.py --studio        # watch the survey

`--airframe <dir>` flies a package the catalog builder wrote instead; with no
catalog at all the cell draws a box the size of the same airframe, so the
timing and the checks still run.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import botrail as bt

HERE = Path(__file__).resolve().parent

AIRFRAME = "px4/x500/x500"        # the catalog package, tried first
PAD = (0.5, 0.0)
RACKS = [2.0, 3.2, 4.4]          # rack centres along x
CRUISE = 2.4                     # survey altitude, over the racks
# The outbound corridor crosses the arm's bench at working height — inside
# the arm's reach *by design*. Sharing that space is a matter of when, not
# where: the arm tends its bench with the tool a quarter metre above the
# corridor, stows, and only then may the drone come through. The climb
# point is past the bench, where the racks begin.
CORRIDOR = 1.02                  # low corridor altitude, over the stowed arm
CLIMB_AT = (1.7, 0.0)            # end of the corridor, where it climbs to cruise
ARM_STOW = {"shoulder_lift": -1.9, "elbow": 2.4}   # folded under the corridor
ARM_TEND, ARM_FOLD = 8.0, 1.5    # bench work, then the stow ramp
# Indoor rates. The airframe is rated far faster (PX4's multicopter limits,
# `specs.max_*`); a rack aisle is not the place to use them, and the cell is
# what says so — the catalog caps, the cell chooses.
SPEED, CLIMB, DESCENT = 0.8, 0.6, 0.9
_TOLD: set = set()


def airframe_of(pack=AIRFRAME):
    """The machine as a `Robot`, with its manifest — or `(None, {})`.

    A UAV is a robot mounted on an aerial vehicle, the exact symmetry of a
    quadruped on a differential one (`vehicle.legged` = Robot + Gait +
    Vehicle; `vehicle.uav` = Robot + rigid mount + Vehicle). Being a robot
    is what buys it interference computation: its links against the racks
    while it flies (`RiderCollision`), its links against other robots'
    links every tick (`RobotCollision`), and the live distance readout
    while you author. `pack=None` skips the catalog entirely (what the
    offline tests use) and the cell falls back to a box airframe riding
    the vehicle as plain body geometry — same flight, coarser collisions.
    """
    if pack is None:
        return None, {}
    try:
        directory = Path(pack)
        if directory.is_dir():
            # A package the catalog builder wrote: the URDF carries the
            # meshes, the manifest the identity and the rates.
            return bt.Robot.from_urdf(directory / "urdf" / "model.urdf"), _manifest(directory)
        model = bt.Robot.from_catalog(str(pack))  # identity rides the model
        return model, _manifest(Path(bt.catalog_package(str(pack))))
    except Exception as err:  # noqa: BLE001 - unreachable catalog, not a bad order
        if "drone" not in _TOLD:
            _TOLD.add("drone")
            text = " ".join(str(err).split())
            if len(text) > 110:
                text = text[:107] + "..."
            print(f"catalog {pack} unavailable ({text}); drawing a box airframe")
        return None, {}


def _manifest(directory: Path) -> dict:
    import yaml

    return yaml.safe_load((directory / "manifest.yaml").read_text(encoding="utf-8")) or {}


def build(*, altitude: float = CRUISE, corridor: float = CORRIDOR,
          interlock: bool = True, pack=AIRFRAME) -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(HERE / "simple_arm.urdf"), name="survey")

    # The working arm under the corridor — the airspace the obstacle
    # checks cannot see, and the cross-robot check can. It starts with its
    # tool up at 1.26 m, *through* the 1.02 m corridor: geometry alone
    # would forbid this cell. Time is what permits it.
    ped = bt.parts.pedestal(scene, "stand", height=0.4, position=(1.3, 0.0))
    mount_p, mount_q = scene.frame(ped.frames[0])
    # 5 mm above the plate: standing exactly on it reads as a live
    # collision in the studio.
    scene.set_robot_base_pose((mount_p[0], mount_p[1], mount_p[2] + 0.005), mount_q)

    # Three rack bays with a carton on the top shelf — what the scanner
    # looks for, placed on the shelf frame the part declares.
    for i, x in enumerate(RACKS, start=1):
        bay = bt.parts.rack(scene, f"rack{i}", size=(0.9, 0.45, 1.5),
                            position=(x, 1.1), levels=3)
        top, _ = scene.frame(bay.frames[-1])
        scene.add_box(f"carton{i}", size=(0.3, 0.3, 0.25),
                      position=(x, 1.1, top[2] + 0.125))

    # The airframe, parked on its pad: the robot when the catalog answers,
    # a box of the same footprint when it does not.
    model, spec = airframe_of(pack)
    if model is not None:
        scene.add_robot(model, name="drone")
    else:
        scene.add_box("drone/body", size=(0.36, 0.36, 0.12), position=(*PAD, 0.06))
    scene.add_vehicle(
        "drone", body=[] if model is not None else ["drone"],
        path=[(*PAD, 0.0), (*PAD, corridor), (*CLIMB_AT, corridor),
              (*CLIMB_AT, altitude)]
        + [(x, 1.1, altitude) for x in RACKS],
        stations={"pad": 0, "gate": 1, "r1": 4, "r2": 5, "r3": 6},
        speed=SPEED, start="pad",
        drive="aerial", climb_speed=CLIMB, descent_speed=DESCENT,
    )
    if model is not None:
        # The machine rides its vehicle rigidly — the AMR-arm mount, with
        # the whole robot as the machine. Its base is the manifest's
        # base_footprint, so the landing gear stands on the pad.
        scene.mount_robot("drone", robot="drone")
        # One machine, one BOM line, on the robot. `from_catalog` carries
        # the identity itself; a locally built package is pinned from its
        # manifest (`spec["id"]`, not the directory it happens to sit in).
        if spec and Path(pack).is_dir():
            scene.set_part("drone", kind="robot", catalog=spec.get("id"),
                           manufacturer=(spec.get("manufacturer") or {}).get("name"),
                           model=spec.get("name"), category=spec.get("category"),
                           mass_kg=(spec.get("specs") or {}).get("mass_kg"))

    # The scanner: a zone under the belly, riding the airframe.
    scene.add_zone_sensor("scan", position=(0.0, 0.0, -0.65), size=(0.5, 0.5, 1.1),
                          watch=[f"carton{i}" for i in range(1, 4)], mount="drone")

    # The arm's own program, running beside the survey: tend the bench,
    # stow, say so. The signal is the interlock's fabric — a PLC bit.
    scene.define_signal("arm_clear", initial=False)
    tend = scene.sequence("tend")
    tend.step("work", transition=bt.seq.elapsed(ARM_TEND))
    tend.step("stow", actions=[bt.seq.ramp(ARM_STOW, ARM_FOLD, robot="survey")],
              transition=bt.seq.done())
    tend.step("clear", actions=[bt.seq.set_signal("arm_clear", True)],
              transition=bt.seq.elapsed(0.0))

    seq = scene.sequence("survey")
    seq.step("lift", actions=[bt.seq.goto("drone", "gate")],
             transition=bt.seq.device_done("drone"))
    if interlock:
        # The gate: hover at the pad until the arm says the corridor is
        # free. Drop this step (--no-interlock) and nothing about the
        # geometry changes — only the clock — and the bake refuses with
        # both links named at the instant they meet.
        seq.step("clearance", transition=bt.seq.signal("arm_clear"))
    for st in ("r1", "r2", "r3"):
        seq.step(f"to_{st}", actions=[bt.seq.goto("drone", st)],
                 transition=bt.seq.device_done("drone"))
        seq.step(f"scan_{st}", transition=bt.seq.elapsed(0.8))
    seq.step("home", actions=[bt.seq.goto("drone", "pad")],
             transition=bt.seq.device_done("drone"))
    return scene


def bake(*, altitude: float = CRUISE, corridor: float = CORRIDOR,
         interlock: bool = True, pack=AIRFRAME):
    scene = build(altitude=altitude, corridor=corridor, interlock=interlock, pack=pack)
    return scene, scene.simulate_sequences(["survey", "tend"], max_duration=120.0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", nargs="?", default=str(HERE / "drone_cell.usdc"))
    parser.add_argument("--low", action="store_true",
                        help="a 0.45 m corridor: too low even for the stowed arm, refused by name")
    parser.add_argument("--no-interlock", dest="no_interlock", action="store_true",
                        help="drop the arm_clear gate: same geometry, wrong clock, refused")
    parser.add_argument("--airframe", default=AIRFRAME,
                        help="the UAV pack: a catalog id or a package directory")
    parser.add_argument("--studio", action="store_true")
    args = parser.parse_args()

    if args.low:
        # 0.45 m: above the pedestal, but through even the *stowed* arm —
        # no timing fixes a corridor this low, and the check says which
        # link. (0.65 actually flies: the stowed wrist threads the gap
        # between the landing gear and the belly plate — the collision
        # model is the machine's own, not a box around it.)
        try:
            bake(corridor=0.45, pack=args.airframe)
        except ValueError as err:
            print("refused, as it should be:")
            print(f"  {err}")
            return
        raise SystemExit("a 0.45 m corridor through the stowed arm should have been refused")

    if args.no_interlock:
        # Same paths, same volumes, same machines — only the gate removed.
        # The drone enters the corridor while the arm is still up, and the
        # refusal names the instant and both links. This is the difference
        # between checking against scenery and checking against a machine
        # that moves: collision is a property of the *pair of clocks*.
        try:
            bake(interlock=False, pack=args.airframe)
        except ValueError as err:
            print("refused, as it should be:")
            print(f"  {err}")
            return
        raise SystemExit("flying the corridor before arm_clear should have been refused")

    scene, tl = bake(pack=args.airframe)
    # The machine's own base reports the landing; the box fallback has no
    # robot, so its body obstacle stands in.
    if "drone" in scene.robots:
        parked, _ = tl.base_pose(0.0, "drone")
        p, _ = tl.base_pose(tl.duration, "drone")
    else:
        frame = next(o for o in scene.obstacle_names if o.startswith("drone/"))
        parked, _ = tl.object_pose(frame, 0.0)
        p, _ = tl.object_pose(frame, tl.duration)
    lanes = dict(tl.signals)
    blips = sum(1 for _, v in lanes["scan"] if v)
    off = max(abs(p[i] - parked[i]) for i in range(3))
    print(f"cycle {tl.duration:.2f}s, {blips} scan passes (3 out, 2 on the "
          f"retrace), back on the pad at ({p[0]:.2f}, {p[1]:.2f}) "
          f"within {off * 1e3:.1f} mm of where it took off")
    tl.export_usd(args.out, fps=60)
    print(f"wrote {args.out}")
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()

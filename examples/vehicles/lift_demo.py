"""An AMR rides an elevator to the mezzanine.

The lift is a device (`scene.add_lift`): a car of ordinary obstacles moved
between named stops, carrying **whatever its capture zone holds when the
ride is commanded** — here a whole vehicle, chassis, tote on the deck and
the mounted arm riding one rigid motion. The doors are not part of the
device: the panel is a plain `add_linear_axis`, and it enforces itself —
closed, it physically blocks the path, and boarding fails the aisle check
by name (try `bake(skip_door=True)`).

The vehicle's path carries the vertical hop as a *lift edge* (two waypoints
stacked at the car): validation only accepts it because both ends sit in
the lift's zone at its stops, and `goto` refuses to walk across it — you
drive to the near side, ride, and continue. The interlock chain is plain
sequence authoring: call → door open → board → door close → ride → alight,
every step a lane on the timing chart.

    python examples/vehicles/lift_demo.py            # bake + USD
    python examples/vehicles/lift_demo.py --studio   # watch the ride
"""

from __future__ import annotations

import argparse
from pathlib import Path

import botrail as bt

HERE = Path(__file__).resolve().parent

TOP = 2.2                      # mezzanine height
CAR_X = 3.25                   # car centre
DOCK = (4.45, 0.0, TOP)        # 2F station


def build(*, skip_door: bool = False) -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(HERE.parent / "assets" / "simple_arm.urdf"), name="amr_lift")

    # ---- the AMR: chassis, deck, tote, and the arm riding it ----------
    scene.add_box("amr/chassis", size=(0.6, 0.5, 0.25), position=(1.0, 0.0, 0.185))
    scene.add_box("tote", size=(0.24, 0.2, 0.12), position=(0.9, 0.0, 0.37))
    scene.add_vehicle(
        "amr", body=["amr"],
        path=[(1.0, 0.0, 0.0), (2.4, 0.0, 0.0), (CAR_X, 0.0, 0.0),
              (CAR_X, 0.0, TOP), (DOCK[0], 0.0, TOP)],
        stations={"lobby": 0, "car": 2, "dock": 4},
        speed=0.5, start="lobby",
        tray_position=(0.0, 0.0, 0.36), tray_size=(0.6, 0.5, 0.2),
    )
    # 5 mm above the chassis top: standing exactly on it reads as a live
    # collision in the studio (mount contact is not auto-allowed for
    # wheeled mounts, unlike a gait's footprint).
    scene.mount_robot("amr", offset_position=(0.15, 0.0, 0.32))

    # ---- the shaft: car floor + side walls, zone, stops ---------------
    scene.add_box("lift/floor", size=(1.4, 1.4, 0.04), position=(CAR_X, 0.0, -0.02))
    for side, y in (("l", 0.72), ("r", -0.72)):
        scene.add_box(f"lift/wall_{side}", size=(1.4, 0.05, 1.8),
                      position=(CAR_X, y, 0.9))
    scene.add_lift(
        "lift", car=["lift"],
        zone_position=(CAR_X, 0.0, 1.0), zone_size=(1.3, 1.3, 2.0),
        stops={"1F": 0.0, "2F": TOP}, speed=0.6,
    )

    # ---- the door: a plain axis panel that blocks the path ------------
    scene.add_box("door/panel", size=(0.06, 1.3, 1.6), position=(2.4, 0.0, 0.8))
    scene.add_linear_axis("door", objects=["door/panel"], axis=(0, 0, 1),
                          speed=0.8, range=(0.0, 1.7))

    # ---- the mezzanine the car lands on -------------------------------
    scene.add_box("mezz/deck", size=(2.0, 2.5, 0.06),
                  position=(4.7, 0.0, TOP - 0.03))

    # ---- interlock chain: every step a lane on the chart --------------
    scene.define_signal("call")
    scene.add_zone_sensor("car_occupied", position=(CAR_X, 0.0, 0.9),
                          size=(1.3, 1.3, 1.7), watch=["amr/chassis"])

    seq = scene.sequence("deliver")
    seq.step("approach", actions=[bt.seq.goto("amr", "lobby")],
             transition=bt.seq.device_done("amr"))
    seq.step("call", actions=[bt.seq.set_signal("call")],
             transition=bt.seq.device_done("lift"))
    if not skip_door:
        seq.step("door_open", actions=[bt.seq.move_to("door", 1.65)],
                 transition=bt.seq.device_done("door"))
    seq.step("board", actions=[bt.seq.goto("amr", "car")],
             transition=bt.seq.device_done("amr"))
    if not skip_door:
        seq.step("door_close",
                 actions=[bt.seq.move_to("door", 0.0),
                          bt.seq.set_signal("call", False)],
                 transition=bt.seq.device_done("door"))
    seq.step("ride", actions=[bt.seq.move_to("lift", "2F")],
             transition=bt.seq.device_done("lift"))
    seq.step("alight", actions=[bt.seq.goto("amr", "dock")],
             transition=bt.seq.device_done("amr"))
    seq.step("handover", transition=bt.seq.elapsed(1.5))
    return scene


def bake(*, skip_door: bool = False):
    scene = build(skip_door=skip_door)
    return scene, scene.simulate_sequence("deliver", max_duration=120.0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", nargs="?", default=str(HERE / "lift_cell.usdc"))
    parser.add_argument("--studio", action="store_true")
    args = parser.parse_args()

    scene, tl = bake()
    z0 = tl.base_pose(0.0)[0][2]
    z1 = tl.base_pose(tl.duration)[0][2]
    print(f"cycle {tl.duration:.2f}s — the arm's base rides {z0:.2f} m -> {z1:.2f} m")
    p, _ = tl.object_pose("tote", tl.duration)
    print(f"tote delivered at ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})")
    tl.export_usd(args.out, fps=60)
    print(f"wrote {args.out}")
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()

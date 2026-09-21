"""洗浄・検査・出荷 — a humanoid runs the finishing line by hand.

The three jobs a machine shop cannot hire for — wash the machined part,
check it, pack it — done by a Unitree G1 walking a loop between the
stations a person works at today: the parts tray beside the machining
centre, an ultrasonic bath with a START button, an image-inspection
station under a 3D camera, and the shipping tray in front of the shelf.
Nothing is rebuilt for the robot: the benches are the benches, the button
is a button, the inspection is a camera's verdict.

The robot walks as a vehicle with legs (`bt.Gait`, the way
`humanoid_carry_demo.py` walks) and works with its right arm as a planning
group (the catalog package declares its arms): one set of joint-space reaches, taught once and
planned at every station, picks off the tray, sets down in the bath,
presents to the camera and packs the shipping tray — every station puts
its work at the same offset from where the robot parks, which is the
mobile-manipulator inversion (teach the pose, put the equipment where it
reaches). The button is pressed with the fingertips, whose reach past the
hand's frame is *measured* off the model at build rather than assumed.
The bath and the inspector run their own programs on their own
controllers, and the verdict branches the cycle: OK walks on to shipping,
NG goes into the reject tray.

Products from the catalog: the G1, with a D435i and MID-360 seated on its
camera/radar fixed plates; TRUSCO AE-1500 benches; an Erecta wire shelf; a KEYENCE GL-R light
curtain across the gate; an OMRON E3Z eye on the tray; a Mech-Mind
Mech-Eye PRO M over the inspection stage; a 22 mm button box on the bath;
an Axelent X-Guard fence. From Yagishita Giken's own equipment list: the
Okuma MB-56VA the tray stands beside (public figures — 2,470 × 3,000 mm
floor, 2,750 mm high, table top at 800 mm). The machined workpiece is an
illustrative 60 x 60 x 40 mm part, using the 0.39 kg solid aluminium blank
as a conservative handling load; its holes are not a supplier drawing.
The ultrasonic bath is the one thing
no public source names, so it is left *unidentified* on purpose: the
report's open question is the one to ask.

What the bake is for: the cycle time with the walking in it; the button
actually pressed and the verdict actually read (both are sensed, not
written); the interlock table — no walk while the gate is broken, no wash
without the basket loaded, no shipping without an OK; the FAT rows — an
NG part (takes the other branch), a visitor in the gate (nothing walks),
an open wire on START (nothing washes), an empty tray (nothing starts);
and the requirement the part puts on the robot (its mass against the arm's
payload).

Run with:  python examples/legged/wash_inspect_ship_demo.py [out.usdc] [--studio]
                 [--wash-s SECONDS]

Needs the catalog (`pip install botrail[catalog]`; the packages are
fetched from the Hugging Face dataset botrail/botrail-catalog and cached).
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import botrail as bt

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import _workshop_visuals as visuals

# ------------------------------------------------------------- products
G1 = "unitree/g1/g1"
BENCH = "trusco/ae/ae-1500"
SHELF = "erecta/basic/wire-shelf"
FENCE = "axelent/fence/x-guard-classic"
CURTAIN = "keyence/gl-r/series"
EYE = "omron/e3z/standard"
CAMERA = "mech_mind/mech-eye/pro-m"
PANEL = "botrail/hmi/button-box-22"
HEAD_CAMERA = "realsense/d400/d435i/r1"
HEAD_LIDAR = "livox/mid/mid-360/r2"
CAMERA_PLATE = "unitree/g1/camera-fixed-plate/r1"
RADAR_PLATE = "unitree/g1/radar-fixed-plate/r1"
# Seat the camera toward the plate's open fork. This reference placement
# leaves the rear web clear of the LiDAR; the real hole alignment is unknown.
CAMERA_PLATE_SHIFT = 0.016
# Reference-geometry seating correction along the inverted LiDAR's +Z.
# At the nominal G1 datum the reconstructed flat MID-360 base protrudes
# through the rounded head STL by ~6.5 mm. This 8 mm adjustment is a demo
# fit estimate, not a measured extrinsic calibration of a real G1.
LIDAR_SEAT = 0.008
# Okuma MB-56VA, from the public listing: floor 2,470 x 3,000 mm, height
# 2,750 mm, table 1,300 x 560 mm with its top 800 mm off the floor.
MC = {"manufacturer": "オークマ", "model": "MB-56VA", "size": (2.47, 3.0, 2.75), "table": (1.3, 0.56, 0.80),
      "chamber": 1.9, "front_door": (1.4, 1.6, 0.30), "door": None, "panel": None, "detail": "full"}

ROBOT, LEGS, ARM, HAND = "walker", "legs", "right", "right_rubber_hand"
G1_PAYLOAD_KG = 3.0            # Unitree's published arm payload (G1 EDU); not on the catalog manifest
FOOTPRINT = (0.40, 0.30, 1.25)  # the body the aisle check drives: shoulders across, at the home heading (-y)
SPEED, TURN = 0.5, 0.8          # 0.5 m/s keeps the cycle under the studio's default 120 s simulate cap

# ---------------------------------------------------------------- the loop
# A rectangle walked clockwise, the stations on its corners and the home
# mat on the side between the last and the first. A parked vehicle faces
# the leg it arrived by, so each station's work stands *beyond* its corner
# in line with the incoming leg, and the next leg turns away from it. It
# turns *right* at every corner: the working arm and whatever it holds
# swing away from the bench, the idle arm's sweep stays under the bench top.
LX, LY = 4.8, 2.4
STATIONS = {"h": (0.0, 1.2), "a": (0.0, 0.0), "w": (-LX, 0.0), "i": (-LX, LY), "s": (0.0, LY)}
ORDER = ["h", "a", "w", "i", "s"]
HEADING = {"h": -math.pi / 2, "a": -math.pi / 2, "w": math.pi, "i": math.pi / 2, "s": 0.0}
LABEL = {"a": "tray", "w": "bath", "i": "inspection", "s": "shipping"}

# ----------------------------------------------- the work, in the robot's frame
# Everything a station presents is placed in the frame of the robot parked
# at it: (forward, right, up). The same numbers at every station is what
# lets one taught reach serve all of them.
BENCH_FWD = 0.65                # an AE-1500 is 750 deep: its front edge 0.275 m ahead of the base, clear of the pivot (0.25 m corner radius)
WORK = (0.40, 0.18)             # a part's centre when it is taken or set down
RISER = (0.26, 0.26, 0.12)      # the tray / stage under it: its front face 0.27 m out, past the pivot's 0.25 m sweep
RISER_TOP = 0.86
PART = (0.06, 0.06, 0.04)       # workpiece envelope; the bores are illustrative
PART_MASS = 0.39                # conservative load: solid aluminium blank, 60 x 60 x 40
SEAT = 0.018                    # a part is set down this far above the floor the planner checks — its own
                                # conservative check refuses a set-down closer than ~15 mm — on an insert it does not see
PART_TOP = RISER_TOP + SEAT + PART[2]
HAND_BACK = 0.075               # G1 rubber hand: the work centre lies 75 mm along its local +X
PALM_DEPTH = 0.018              # local +Y palm/finger surface, measured against the shipped visual
APPROACH = 0.08                 # 80 mm lift clears the tray with the palm-down grip
CARRY = (0.20, 0.22, 1.02)      # the hand while walking with a part: above every tray it passes
NG_WORK = (0.34, 0.42)          # the reject tray, to the right of the stage: past the body's half-width, since
                                # the next leg walks right through where it stands (the arm is 35 mm short at right 0.50)
NG_APPROACH = 0.07              # a shorter descent onto the reject tray: the high approach is what runs out of reach
PANEL_AT = (0.30, 0.46, 0.98)   # the bath's START box, to the right of the basket
PRESS_BACK = 0.04               # the fingertips' stand-off before a press
WASH_S, INSPECT_S = 18.0, 1.5


def local(st: str, fwd: float, right: float, up: float = 0.0) -> tuple[float, float, float]:
    """A point given in the frame of the robot parked at `st`, in the world."""
    h = HEADING[st]
    x0, y0 = STATIONS[st]
    return (x0 + fwd * math.cos(h) + right * math.sin(h), y0 + fwd * math.sin(h) - right * math.cos(h), up)


def dims(st: str, fwd_len: float, right_len: float, height: float) -> tuple[float, float, float]:
    """A box's world size from its extents along and across the parked heading."""
    across = abs(math.sin(HEADING[st])) > 0.5     # heading along y: forward is the world y
    return (right_len, fwd_len, height) if across else (fwd_len, right_len, height)


def facing(st: str) -> float:
    """The yaw that turns a `-Y`-faced thing (a panel, a machine's front) toward the parked robot."""
    return HEADING[st] - math.pi / 2


# ----------------------------------------------------------------- the cell
def _inverse_pose(p, q):
    """Invert an assembly datum so it coincides with the G1 sensor frame."""
    x, y, z, w = -q[0], -q[1], -q[2], q[3]
    a, b, c = (-v for v in p)
    tx, ty, tz = 2 * (y*c - z*b), 2 * (z*a - x*c), 2 * (x*b - y*a)
    return ((a + w*tx + y*tz - z*ty, b + w*ty + z*tx - x*tz, c + w*tz + x*ty - y*tx),
            (x, y, z, w))


def build_robot() -> tuple[bt.Robot, bt.Gait]:
    """The G1 with its head sensor assemblies and a working right arm.

    The plates are purchasable G1 spares with photo-reference geometry;
    their dimensions and fasteners are unverified. Preserve the G1 URDF's
    camera datum and the inverted LiDAR orientation. The reference LiDAR
    housing uses the explicitly estimated LIDAR_SEAT correction. Neither
    sensor applies its mount-to-sensing-frame offset twice.
    """
    robot = bt.Robot.from_catalog(G1)
    for name, plate_id, sensor_id, datum, host in (
        ("camera", CAMERA_PLATE, HEAD_CAMERA, "camera_depth_frame", "d435_link"),
        ("radar", RADAR_PLATE, HEAD_LIDAR, "livox_frame", "mid360_link"),
    ):
        plate = bt.Robot.from_catalog(plate_id)
        sensor = bt.Robot.from_catalog(sensor_id)
        assembly = plate.attach_tool(sensor, prefix="device_",
                                     offset_position=(-CAMERA_PLATE_SHIFT, 0, 0) if name == "camera" else None)
        position, quaternion = _inverse_pose(*bt.Scene(assembly).link_pose(f"device_{datum}"))
        if name == "radar":
            position = (position[0], position[1], position[2] + LIDAR_SEAT)
        robot = robot.mount(assembly, at=host, offset_position=position, offset_quaternion=quaternion,
                            prefix=f"head_{name}_", group=f"head_{name}")
    # The arms are the package's own planning groups (`frames.arms[]`: `left`
    # and `right`, seven joints each, tipped at the rubber hands) — nothing
    # to declare here.
    assert robot.group(ARM).tip == HAND
    return robot, bt.Gait.from_catalog(G1)


def riser(scene: bt.Scene, st: str, name: str, at=WORK, size=RISER, **kwargs) -> bt.parts.Built:
    """A tray at a station, its top at `RISER_TOP`: the box the planner
    checks, with the foam insert the part visibly rests on (`SEAT` thick,
    a picture — the planner refuses a set-down that close to a thing it
    sees) and the frame `<name>/seat` where the part sets down."""
    return bt.parts.tray(scene, name, size, local(st, at[0], at[1], RISER_TOP - size[2]), yaw=HEADING[st],
                         seat=SEAT, **kwargs)


def _finish_visuals(scene: bt.Scene) -> None:
    """The unidentified bath is shown with its basket raised for loading.

    The process model times washing; it does not simulate immersion or
    liquid. Curved/perforated visuals preserve the taught loading height.
    """
    bt.parts.appearance(scene, "part", "workpiece", PART)

    # Open tubular stand under a pressed stainless tank. A cantilevered
    # cradle supports the tank's existing forward overhang.
    scene.set_obstacle_visible("washer/stand", False)
    for i, (f, r) in enumerate(((0.44, -0.25), (0.44, 0.25), (0.86, -0.25), (0.86, 0.25))):
        visuals.box(scene, f"washer/detail/leg{i}", dims("w", 0.035, 0.035, 0.70),
                    local("w", f, r, 0.39), metalness=0.8)
        bt.parts.shaped_box(scene, f"washer/detail/foot{i}", "adjuster", (0.055, 0.055, 0.04), local("w", f, r, 0.02))
    for side in (-1, 1):
        visuals.box(scene, f"washer/detail/cradle{side}", dims("w", 0.62, 0.025, 0.035),
                    local("w", 0.59, WORK[1] + side * 0.19, 0.65), metalness=0.8)
    for end in (0.44, 0.86):
        visuals.box(scene, f"washer/detail/crossmember{end}", dims("w", 0.035, 0.65, 0.035),
                    local("w", end, 0.065, 0.615), metalness=0.8)
    bt.parts.shaped_box(scene, "washer/detail/lower_shelf", "panel", dims("w", 0.47, 0.55, 0.018),
                        local("w", BENCH_FWD, 0.0, 0.18), color=visuals.STEEL)
    for tag in ("near", "far", "left", "right"):
        scene.set_obstacle_visible(f"washer/tank/{tag}", False)
    scene.set_obstacle_visible("washer/basket", False)
    bt.parts.shaped_box(scene, "washer/detail/tank", "tray", dims("w", 0.27, 0.44, 0.20),
                        local("w", 0.415, WORK[1], 0.77))
    bt.parts.shaped_box(scene, "washer/detail/rim", "rim", dims("w", 0.285, 0.455, 0.012),
                        local("w", 0.415, WORK[1], 0.864))
    # The perforated basket sheet finishes exactly at the original insert
    # top: the visual workpiece rests on it, both before and after washing.
    bt.parts.appearance(scene, "washer/basket/mesh", "basket", dims("w", 0.21, 0.38, 0.003),
                        offset=(0.0, 0.0, SEAT / 2 - 0.0015))
    for side in (-1, 1):
        bt.parts.shaped_box(scene, f"washer/detail/basket_grip{side}", "handle", dims("w", 0.09, 0.012, 0.035),
                            local("w", 0.415, WORK[1] + side * 0.195, 0.89))
    bt.parts.shaped_box(scene, "washer/detail/open_lid", "panel", dims("w", 0.012, 0.43, 0.22),
                        local("w", 0.565, WORK[1], 0.985), color=visuals.STEEL)
    for name in ("washer/detail/lower_shelf", "washer/detail/open_lid"):
        scene.set_obstacle_material(name, metalness=0.8, roughness=0.32)
    bt.parts.shaped_box(scene, "washer/detail/lid_handle", "handle", (0.12, 0.012, 0.035),
                        local("w", 0.543, WORK[1], 1.03), quaternion=(0.5, 0.5, 0.5, 0.5))
    bt.parts.shaped_box(scene, "washer/detail/drain", "hose", dims("w", 0.18, 0.025, 0.32),
                        local("w", 0.72, 0.27, 0.47))
    visuals.cylinder(scene, "washer/detail/valve", 0.022, 0.025,
                     local("w", 0.78, 0.27, 0.64), color=(0.045, 0.16, 0.26))


def build_scene() -> bt.Scene:
    robot, gait = build_robot()
    scene = bt.Scene(robot, name=ROBOT)
    h = STATIONS["h"]
    scene.add_box(f"{ROBOT}/footprint", FOOTPRINT, (h[0], h[1], FOOTPRINT[2] / 2))
    scene.set_obstacle_visible(f"{ROBOT}/footprint", False)
    scene.add_vehicle(LEGS, body=[f"{ROBOT}/footprint"], path=[STATIONS[s] for s in ORDER],
                      stations={s: i for i, s in enumerate(ORDER)}, speed=SPEED, turn_speed=TURN,
                      start="h", ring=True)
    scene.mount_robot(LEGS, robot=ROBOT, gait=gait)
    # Sensor optics start at the physical device's mount face. The catalog
    # supplies calibration and specs; the robot assembly supplies its body.
    scene.add_camera("g1_eyes", robot=ROBOT, link="head_camera_device_mount",
                      from_catalog=HEAD_CAMERA, body_visible=False)
    scene.add_lidar("g1_lidar", robot=ROBOT, link="head_radar_device_mount",
                     from_catalog=HEAD_LIDAR, body_visible=False)
    # Each physical sensor already has a BOM line inside its plate assembly.
    scene.remove_part("g1_eyes")
    scene.remove_part("g1_lidar")
    scene.add_box("home/mat", (0.8, 0.6, 0.01), (h[0], h[1], 0.005), color=(0.25, 0.28, 0.32))
    scene.set_obstacle_enabled("home/mat", False)      # a marking on the floor, not a thing

    # The benches: one per working station, the 1,500 side across the robot.
    for st in ("a", "i", "s"):
        c = local(st, BENCH_FWD, 0.0)
        bt.parts.table(scene, f"bench_{st}", position=(c[0], c[1]), yaw=HEADING[st] + math.pi / 2, catalog=BENCH)

    # A: the parts tray at the machining centre's outfeed, an eye across it.
    riser(scene, "a", "tray", model="部品トレイ", description="the machined parts, as they leave the MC")
    p = local("a", WORK[0], WORK[1], RISER_TOP + SEAT + PART[2] / 2)
    scene.add_box("part", PART, p, color=(0.78, 0.8, 0.83))
    scene.set_part("part", category="workpiece", model="加工ワーク 60x60x40 (形状例)", mass_kg=PART_MASS,
                   description="穴・座ぐり付きの表示用形状。0.39 kg は加工前の中実ブロックによる保守的な搬送負荷")
    bt.parts.photoelectric(scene, "part_present", local("a", WORK[0], WORK[1] - 0.20, RISER_TOP + 0.02),
                           local("a", WORK[0], WORK[1] + 0.20, RISER_TOP + 0.02), catalog=EYE, watch=["part"])
    mc = local("a", BENCH_FWD + 0.375 + 0.15 + MC["size"][1] / 2, 0.0)
    bt.parts.machine_tool(scene, "mc", position=(mc[0], mc[1]), yaw=facing("a"), **MC)

    # W: the ultrasonic bath on its stand — a shallow tank whose floor is the
    # basket, and the START box to the right. The bath is deliberately not
    # identified: the report asks.
    stand = local("w", BENCH_FWD, 0.0, 0.40)
    scene.add_box("washer/stand", dims("w", 0.50, 0.60, 0.80), stand, color=(0.42, 0.44, 0.47))
    # The tank: its opening runs from 0.30 m out (past the pivot's sweep and
    # under the forearm) to 0.53 m (past the fingertips), a shallow bath
    # whose rim the forearm passes 5 cm over.
    rim, wall, open_fwd, open_right = 0.87, 0.02, 0.23, 0.40
    tank_fwd = 0.30 + open_fwd / 2
    floor = local("w", tank_fwd, WORK[1], (0.80 + RISER_TOP) / 2)
    scene.add_box("washer/basket", dims("w", open_fwd, open_right, RISER_TOP - 0.80), floor, color=(0.62, 0.64, 0.66))
    # The basket's mesh floor is a picture `SEAT` thick over the box the planner sees.
    mesh = scene.add_box("washer/basket/mesh", dims("w", open_fwd - 0.02, open_right - 0.02, SEAT),
                         local("w", tank_fwd, WORK[1], RISER_TOP + SEAT / 2), color=bt.parts.FOAM)
    scene.set_obstacle_enabled(mesh, False)
    for tag, (df, dr, lf, lr) in {"near": (-open_fwd / 2 - wall / 2, 0, wall, open_right + 2 * wall),
                                  "far": (open_fwd / 2 + wall / 2, 0, wall, open_right + 2 * wall),
                                  "left": (0, -open_right / 2 - wall / 2, open_fwd, wall),
                                  "right": (0, open_right / 2 + wall / 2, open_fwd, wall)}.items():
        at = local("w", tank_fwd + df, WORK[1] + dr, (0.80 + rim) / 2)
        scene.add_box(f"washer/tank/{tag}", dims("w", lf, lr, rim - 0.80), at, color=(0.72, 0.74, 0.76))
    scene.set_part("washer", kind="group", category="washer", description="卓上超音波洗浄槽 — 機種は未確認 (要ヒアリング)")
    scene.add_zone_sensor("basket_loaded", local("w", WORK[0], WORK[1], RISER_TOP + 0.04), dims("w", 0.30, 0.30, 0.08),
                          watch=["part"])
    bt.parts.operator_panel(scene, "washer_panel", local("w", *PANEL_AT), yaw=facing("w"), buttons=("start", "estop"),
                            watch_robots=[ROBOT], catalog=PANEL)

    # I: the inspection stage under a 3D camera on a gantry, the reject tray
    # to the right. The inspector is the camera's verdict, read as an input.
    bt.parts.stage(scene, "stage", RISER, local("i", WORK[0], WORK[1], RISER_TOP - RISER[2]), yaw=HEADING["i"],
                   seat=SEAT, model="検査ステージ", description="the part is presented here")
    riser(scene, "i", "ng_tray", at=NG_WORK, size=(0.18, 0.18, 0.12), model="NG トレイ")
    # The camera's portal: its beam across the parked heading, over the stage.
    bt.parts.gantry(scene, "inspector", span=0.84, height=2.05, position=local("i", WORK[0], WORK[1]), yaw=facing("i"),
                    category="machine.inspection", manufacturer="柳下技研", model="画像検査ステーション (構想)",
                    description="gantry, stage and the camera's verdict as an input")
    scene.add_camera("insp_cam", position=local("i", WORK[0], WORK[1], 1.95),
                     look_at=local("i", WORK[0], WORK[1], RISER_TOP), from_catalog=CAMERA)
    scene.add_vision_sensor("part_seen", camera="insp_cam", watch=["part"])
    # The trigger: the part *on the stage* with nobody's hand over it. The
    # camera sees a part carried past it too, and a hand over the part is a
    # hand in the picture — neither is an inspection.
    scene.add_zone_sensor("on_stage", local("i", WORK[0], WORK[1], RISER_TOP + 0.05), dims("i", RISER[0], RISER[1], 0.10),
                          watch=["part"])
    scene.add_zone_sensor("stage_busy", local("i", WORK[0], WORK[1], RISER_TOP + 0.45), dims("i", RISER[0] + 0.10, RISER[1] + 0.10, 0.70),
                          watch=[], watch_robots=[ROBOT])

    # S: the shipping tray on the bench, the shelf of cartons behind it.
    riser(scene, "s", "ship_tray", model="出荷トレイ")
    sh = local("s", BENCH_FWD + 0.375 + 0.35, 0.0)
    bt.parts.rack(scene, "shelf", position=(sh[0], sh[1]), yaw=HEADING["s"] + math.pi / 2, catalog=SHELF,
                  size=(0.9, 0.45, 1.595), levels=4)
    for k, (dr, level) in enumerate(((-0.25, 0), (0.25, 0), (0.0, 1), (-0.25, 2))):
        seat, _ = scene.frame(f"shelf/level{level}")
        # Stock on the shelf, not a purchase: the part a carton pins comes off.
        scene.remove_part(bt.parts.carton(scene, f"cartons/c{k}", (0.35, 0.30, 0.22), (seat[0], seat[1] - dr, seat[2]),
                                          yaw=-math.pi / 2))

    # The fence around the loop, open on the east side where the light
    # curtain guards the gate; a visitor waits outside it.
    x0, x1, y0, y1 = -6.9, 1.9, -4.6, 4.0
    bt.parts.fence(scene, "fence", path=[(x0, 1.6), (x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, 2.6)],
                   closed=False, catalog=FENCE)
    bt.parts.light_curtain(scene, "gate", (x0 + 0.1, 1.62), (x0 + 0.1, 2.58), catalog=CURTAIN)
    bt.parts.person(scene, "visitor", (x0 - 0.8, 2.1))
    scene.allow_link_obstacle_contact(HAND, "part", robot=ROBOT)
    # Pressing a 22 mm button flush puts the fingertips on the plate around
    # the cap: that touch is the press, not a collision.
    scene.allow_link_obstacle_contact(HAND, "washer_panel/plate", robot=ROBOT)
    # The number the catalog manifest does not carry yet: the published arm payload.
    scene.set_part(ROBOT, kind="robot", payload_kg=G1_PAYLOAD_KG)
    _finish_visuals(scene)
    return scene


# ------------------------------------------------------------- teaching
def finger_reach(scene: bt.Scene, q: list) -> float:
    """How far the hand's collision body reaches past its frame along its
    fingers, probed with a thin wall: the fingertip is what presses a button."""
    was = list(scene.joint_positions_of(ROBOT))
    scene.set_joint_positions(q, robot=ROBOT)
    p, _ = scene.link_pose(HAND, robot=ROBOT)
    h = HEADING["h"]
    ahead = (math.cos(h), math.sin(h), 0.0)
    size = dims("h", 0.002, 0.30, 0.30)
    reach = None
    for mm in range(200, 0, -1):
        d = mm / 1000
        at = tuple(pi + a * d for pi, a in zip(p, ahead))
        scene.add_box("probe", size, at)
        hit = any("probe" in (a[1], b[1]) for a, b in scene.check_collisions())
        scene.remove_obstacle("probe")
        if hit:
            reach = d
            break
    scene.set_joint_positions(was, robot=ROBOT)
    if reach is None:
        raise RuntimeError("the hand reaches nothing within 0.2 m of its frame")
    return reach


def teach(scene: bt.Scene) -> dict[str, list]:
    """The arm's reaches, as joint vectors, taught at the home mat against
    a stand-in of the tray: the geometry every station repeats. The hand
    faces down over the part: G1's finger row spans local Z and its palm
    faces local +Y. Roll -90 degrees to put that face over the workpiece,
    with the link frame `HAND_BACK` behind its centre and `PALM_DEPTH`
    above its top. Button presses retain the forward-facing hand pose."""
    q0 = list(scene.joint_positions_of(ROBOT))
    names = scene.robot_of(ROBOT).joint_names
    right = scene.robot_of(ROBOT).group(ARM).joints
    h = HEADING["h"]
    forward = (0.0, 0.0, math.sin(h / 2), math.cos(h / 2))    # fingers along the parked heading
    def palm_down(yaw):
        half = math.sqrt(0.5)
        return (-half * math.cos(yaw / 2), -half * math.sin(yaw / 2),
                 half * math.sin(yaw / 2), half * math.cos(yaw / 2))
    grip_orientation = palm_down(h)

    def with_arm(vals):
        q = list(q0)
        for j, v in zip(right, vals):
            q[names.index(j)] = v
        return q

    seeds = [q0] + [with_arm(s) for s in ((-0.6, -0.3, 0.0, 1.0, 0.0, 0.0, 0.0), (-1.0, -0.5, 0.3, 1.2, 0.0, 0.0, 0.0),
                                           (-0.3, -0.8, 0.0, 0.6, 0.0, 0.0, 0.0), (-1.2, -0.2, -0.3, 0.9, 0.5, 0.0, 0.0),
                                           (-0.8, -0.6, 0.6, 1.4, 0.0, 0.0, 0.0))]

    def at(label: str, fwd: float, right_: float, up: float, seed=None, orientation=grip_orientation) -> list:
        target = local("h", fwd, right_, up)
        short, touching = None, set()
        for s in ([seed] if seed is not None else []) + seeds:
            scene.set_joint_positions(s, robot=ROBOT)
            r = scene.set_tcp_target(target, orientation, group=ARM, robot=ROBOT)
            if not r.converged:
                short = r.pos_error
                continue
            pairs = scene.check_collisions()
            if not pairs:
                return list(scene.joint_positions_of(ROBOT))
            touching.update(f"{a[1]} x {b[1]}" for a, b in pairs)
        raise RuntimeError(f"`{label}` at (fwd {fwd:.2f}, right {right_:.2f}, up {up:.2f}) is out of the arm's reach"
                           + (f" ({short * 1000:.0f} mm short)" if short is not None else "")
                           + (f"; it touches {', '.join(sorted(touching))}" if touching else ""))

    # The stand-in tray and part at the home mat, so the taught poses are
    # checked against the geometry they will meet at every station.
    stand_ins = [riser(scene, "h", "teach/riser", detail="plain"),
                 riser(scene, "h", "teach/ng", at=NG_WORK, size=(0.18, 0.18, 0.12), detail="plain")]
    scene.add_box("teach/part", PART, local("h", WORK[0], WORK[1], RISER_TOP + SEAT + PART[2] / 2))
    scene.allow_link_obstacle_contact(HAND, "teach/part", robot=ROBOT)
    try:
        grip_z = PART_TOP + PALM_DEPTH
        poses = {"home": q0}
        poses["above"] = at("above", WORK[0] - HAND_BACK, WORK[1], grip_z + APPROACH)
        poses["grip"] = at("grip", WORK[0] - HAND_BACK, WORK[1], grip_z, seed=poses["above"])
        # The carry is taught with the part in hand: what must clear the
        # trays on the way out is the part, not the empty hand.
        scene.set_joint_positions(poses["grip"], robot=ROBOT)
        scene.attach("teach/part", group=ARM, robot=ROBOT)
        try:
            poses["carry"] = at("carry", *CARRY, seed=poses["above"])
        finally:
            scene.detach("teach/part")
        # Turn the wrist towards the side tray. Keeping it straight ahead
        # puts the palm-down wrist outside the arm's reach; the diagonal
        # grip keeps the same work centre and a horizontal seating face.
        ng_back = HAND_BACK / math.sqrt(2)
        ng_fwd, ng_right = NG_WORK[0] - ng_back, NG_WORK[1] - ng_back
        ng_orientation = palm_down(h - math.pi / 4)
        poses["ng_above"] = at("ng_above", ng_fwd, ng_right, grip_z + NG_APPROACH,
                                seed=poses["carry"], orientation=ng_orientation)
        poses["ng_place"] = at("ng_place", ng_fwd, ng_right, grip_z,
                                seed=poses["ng_above"], orientation=ng_orientation)
        # The START button: pressed with the fingertips, against a replica
        # of the bath's box at the mat. The reach past the hand's frame is
        # the estimate; the depth is *calibrated* by pressing — walked in
        # a millimetre at a time until the button's zone reads the hand,
        # clear of the plate — because a 2.6 mm stroke is not a number to
        # assume off a mesh.
        reach = finger_reach(scene, poses["grip"])
        replica = bt.parts.operator_panel(scene, "teach/panel", local("h", *PANEL_AT), yaw=facing("h"),
                                          buttons=("start", "estop"), watch_robots=[ROBOT], catalog=PANEL)
        scene.allow_link_obstacle_contact(HAND, "teach/panel/plate", robot=ROBOT)
        (bx, by, bz), _ = scene.frame("teach/panel/start/press")
        dx, dy = bx - STATIONS["h"][0], by - STATIONS["h"][1]
        fwd = dx * math.cos(h) + dy * math.sin(h)
        right_ = dx * math.sin(h) - dy * math.cos(h)
        nominal = fwd - reach                      # the fingertips at the press frame, if the reach were exact
        seed, found = None, None
        for mm in range(-6, 13):
            try:
                q = at("press", nominal + mm / 1000, right_, bz, seed=seed, orientation=forward)
            except RuntimeError:
                break                              # into the plate: no deeper
            seed = q
            if reads(scene, "teach/panel/start", q):
                found = (nominal + mm / 1000, q)
                break
        scene.disallow_link_obstacle_contact(HAND, "teach/panel/plate", robot=ROBOT)
        replica.remove(scene)
        if found is None:
            raise RuntimeError("the fingertips never press START: the box stands out of the hand's reach or stroke")
        depth, q = found
        poses["press"] = at("press", depth + 0.001, right_, bz, seed=q, orientation=forward)    # a millimetre past the first read
        poses["pre_press"] = at("pre_press", depth - PRESS_BACK, right_, bz, seed=poses["press"], orientation=forward)
        poses["press_depth_mm"] = (depth + 0.001 - nominal) * 1000
    finally:
        scene.disallow_link_obstacle_contact(HAND, "teach/part", robot=ROBOT)
        scene.remove_obstacle("teach/part")
        for stand_in in stand_ins:
            stand_in.remove(scene)
        scene.set_joint_positions(q0, robot=ROBOT)
    poses["finger_reach"] = reach
    return poses


def reads(scene: bt.Scene, signal: str, q: list) -> bool:
    """Whether the sensor `signal` reads the robot standing at `q`: one scan
    of a throwaway program that waits on it."""
    scene.set_joint_positions(q, robot=ROBOT)
    probe = "teach/probe"
    scene.sequence(probe).step("read", transition=bt.seq.signal(signal))
    try:
        scene.simulate_sequence(probe, max_duration=0.05)
        return True
    except Exception:  # noqa: BLE001 — a timeout is the answer "no"
        return False
    finally:
        scene.remove_sequence(probe)


def build_motions(scene: bt.Scene, poses: dict) -> None:
    line = "cartesian_line"

    def seg(motion, key, kind="joint"):
        scene.add_segment(motion, goal=poses[key], kind=kind, group=ARM, robot=ROBOT)

    seg("pick", "above"); seg("pick", "grip", line)           # over the part, straight down onto it
    seg("lift", "above", line)                                # straight up with it
    seg("carry", "carry")                                     # the hand ahead of the body for the walk
    seg("home", "home")                                       # the arm at the side
    seg("press", "pre_press"); seg("press", "press", line)    # to the button, then the fingertips in
    # Back from the button to the carry pose, ahead of the body — not to
    # the side: a hand swung down past the box on its way home pressed
    # START and the E-stop again (the lanes showed it), and the next reach
    # into the basket starts from ahead anyway.
    seg("unpress", "pre_press", line); seg("unpress", "carry")
    seg("reject", "ng_above"); seg("reject", "ng_place", line)
    seg("reject_clear", "ng_above", line); seg("reject_clear", "home")


# ------------------------------------------------------------- programs
def build_programs(scene: bt.Scene, wash_s: float = WASH_S) -> None:
    for sig in ("wash_running", "wash_done", "insp_busy", "insp_ok", "insp_ng", "part_defect", "ok_latched", "ng_latched"):
        scene.define_signal(sig, False)
    S = bt.seq

    # The bath: its own controller waits for START with the basket loaded,
    # runs, and reports done until the basket is emptied.
    w = scene.sequence("washer")
    w.step("ready", transition=S.all_of(S.signal("washer_panel/start"), S.signal("basket_loaded"),
                                        S.signal("washer_panel/estop", False)))
    w.step("wash", actions=[S.set_signal("wash_running", True)], transition=S.elapsed(wash_s))
    w.step("done", actions=[S.set_signal("wash_running", False), S.set_signal("wash_done", True)],
           transition=S.signal("basket_loaded", False))
    w.step("reset", actions=[S.set_signal("wash_done", False)])

    # The inspector: a part in the camera's view is measured, and the
    # verdict holds until the part leaves the view. `part_defect` stands in
    # for the image processing: an input the scenario flips.
    i = scene.sequence("inspector")
    i.step("idle", transition=S.all_of(S.signal("part_seen"), S.signal("on_stage"), S.signal("stage_busy", False)))
    i.step("measure", actions=[S.set_signal("insp_busy", True)], transition=S.elapsed(INSPECT_S))
    verdict = i.select("verdict")
    verdict.when(S.signal("part_defect")).step("ng", actions=[S.set_signal("insp_ng", True)])
    verdict.when(S.otherwise()).step("ok", actions=[S.set_signal("insp_ok", True)])
    i.step("hold", actions=[S.set_signal("insp_busy", False)], transition=S.signal("part_seen", False))
    i.step("clear", actions=[S.set_signal("insp_ok", False), S.set_signal("insp_ng", False)])

    # The humanoid: fetch, wash, present, and pack or reject.
    M = lambda n: S.motion(n)
    go = lambda st: S.goto(LEGS, st)
    arrived = S.device_done(LEGS)
    gate_clear = S.signal("gate", False)
    sq = scene.sequence("finish")
    sq.step("wait part", transition=S.signal("part_present"))
    sq.step("gate clear 1", transition=gate_clear)
    sq.step("to tray", actions=[go("a")], transition=arrived)
    sq.step("pick", actions=[M("pick")])
    sq.step("grasp", actions=[S.attach("part", group=ARM, robot=ROBOT)], transition=S.immediately())
    sq.step("lift", actions=[M("lift")])
    sq.step("carry 1", actions=[M("carry")])
    sq.step("gate clear 2", transition=gate_clear)
    sq.step("to bath", actions=[go("w")], transition=arrived)
    sq.step("into basket", actions=[M("pick")])
    sq.step("release 1", actions=[S.detach("part")], transition=S.immediately())
    sq.step("clear basket", actions=[M("lift")])
    sq.step("press start", actions=[M("press")], transition=S.all_of(S.done(), S.signal("washer_panel/start")))
    sq.step("let go", actions=[M("unpress")])
    sq.step("washing", transition=S.signal("wash_done"))
    sq.step("out of basket", actions=[M("pick")])
    sq.step("grasp 2", actions=[S.attach("part", group=ARM, robot=ROBOT)], transition=S.immediately())
    sq.step("lift 2", actions=[M("lift")])
    sq.step("carry 2", actions=[M("carry")])
    sq.step("gate clear 3", transition=gate_clear)
    sq.step("to inspection", actions=[go("i")], transition=arrived)
    sq.step("present", actions=[M("pick")])
    sq.step("release 2", actions=[S.detach("part")], transition=S.immediately())
    sq.step("hands off", actions=[M("lift")])
    sq.step("arm clear", actions=[M("home")])          # out of the camera's view: a hand over the part occludes it
    sq.step("inspecting", transition=S.any_of(S.signal("insp_ok"), S.signal("insp_ng")))
    # The verdict is latched before the hand goes back in: a hand over the
    # part takes it out of the camera's view, and the inspector's outputs
    # follow the view. The PLC idiom — read, latch, then act.
    read = sq.select("read verdict")
    read.when(S.signal("insp_ok")).step("latch ok", actions=[S.set_signal("ok_latched", True)])
    read.when(S.signal("insp_ng")).step("latch ng", actions=[S.set_signal("ng_latched", True)])
    sq.step("take back", actions=[M("pick")])
    sq.step("grasp 3", actions=[S.attach("part", group=ARM, robot=ROBOT)], transition=S.immediately())
    sq.step("lift 3", actions=[M("lift")])
    judge = sq.select("judge")
    ok = judge.when(S.signal("ok_latched"))
    ok.step("carry 3", actions=[M("carry")])
    ok.step("gate clear 4", transition=gate_clear)
    ok.step("to shipping", actions=[go("s")], transition=arrived)
    ok.step("pack", actions=[M("pick")])
    ok.step("release 3", actions=[S.detach("part")], transition=S.immediately())
    ok.step("clear tray", actions=[M("lift")])
    ok.step("arm home 1", actions=[M("home")])
    ng = judge.when(S.signal("ng_latched"))
    ng.step("to reject tray", actions=[M("reject")])
    ng.step("release ng", actions=[S.detach("part")], transition=S.immediately())
    ng.step("arm home 2", actions=[M("reject_clear")])
    sq.step("gate clear 5", transition=gate_clear)
    sq.step("to home", actions=[go("h")], transition=arrived)
    sq.step("stand", actions=[S.set_signal("ok_latched", False), S.set_signal("ng_latched", False)],
            transition=S.elapsed(1.0))

    # Who runs what: the G1's onboard controller, the bath's and the
    # inspector's own boxes. Points are assigned by the I/O map.
    scene.add_io_node("g1", kind="robot_controller", robots=[ROBOT], programs=["finish"], label="G1 onboard controller")
    scene.add_io_node("washer/plc", kind="plc", programs=["washer"], label="ultrasonic bath controller")
    scene.add_io_node("inspector/plc", kind="plc", programs=["inspector"], label="inspection station controller")

    # The FAT rows. The NG part completes by the other branch; the rest
    # must refuse to go on, and the run shows where they stop.
    scene.add_scenario("ng_part", signals={"part_defect": True})
    scene.add_scenario("visitor_in_gate", obstacles={"visitor": (-6.8, 2.1, 0.85)})
    scene.add_scenario("start_wire_open", faults=[bt.io.open("washer_panel/start")])
    scene.add_scenario("tray_empty", obstacles={"part": (-8.2, -3.8, PART[2] / 2)})


PROGRAMS = ["finish", "washer", "inspector"]


def build(wash_s: float = WASH_S) -> tuple[bt.Scene, dict]:
    scene = build_scene()
    poses = teach(scene)
    build_motions(scene, poses)
    build_programs(scene, wash_s)
    return scene, poses


def bake(wash_s: float = WASH_S, max_duration: float = 400.0):
    scene, poses = build(wash_s)
    return scene, poses, scene.simulate_sequences(PROGRAMS, max_duration=max_duration)


# ------------------------------------------------------------- hand-over
def deliver(scene: bt.Scene, tl, out: Path):
    """The document set, written into `out` from the one source: the
    drawing, the bill, the I/O list and the handshakes between the three
    controllers, the three programs as PLCopen, the interlock table, and
    the report with the FAT scenarios' verdicts and every file's digest."""
    out.mkdir(parents=True, exist_ok=True)
    files: list[Path] = []

    def write(name: str, fn) -> Path:
        path = out / name
        fn(path)
        files.append(path)
        return path

    write("wash_inspect_ship.botrail", scene.save_project)
    write("wash_inspect_ship.py", lambda p: p.write_text(scene.generate_python()))
    write("wash_inspect_ship_bom.csv", scene.export_bom)
    write("wash_inspect_ship_bom.md", scene.export_bom)
    write("wash_inspect_ship_io.csv", scene.export_io_list)
    write("wash_inspect_ship_topology.mmd", scene.export_topology)
    write("wash_inspect_ship_handshake.md", tl.export_handshake_spec)
    write("wash_inspect_ship.plcopen.xml", lambda p: scene.export_plcopen(p, name="wash inspect ship"))
    write("wash_inspect_ship_interlocks.md", scene.export_interlocks)
    write("wash_inspect_ship_interlocks.csv", scene.export_interlocks)
    write("wash_inspect_ship_layout.svg", lambda p: scene.export_layout(p, scale=60, title="wash / inspect / ship"))
    write("wash_inspect_ship_layout.dxf", lambda p: scene.export_layout(p, title="wash / inspect / ship"))
    runs = scene.simulate_scenarios(PROGRAMS, max_duration=tl.duration + 30.0)
    report = scene.cell_report({"baseline": tl}, scenarios=runs, deliverables=files, title="wash / inspect / ship")
    report.save(out / "wash_inspect_ship_report.md")
    report.save(out / "wash_inspect_ship_report.json")
    return report, runs


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("out", nargs="?", default=str(HERE / "wash_inspect_ship.usdc"))
    parser.add_argument("--studio", action="store_true")
    parser.add_argument("--wash-s", type=float, default=WASH_S, help="the bath's programme, seconds")
    parser.add_argument("--no-deliver", action="store_true", help="skip the document set and the scenario matrix")
    args = parser.parse_args()

    scene, poses = build(args.wash_s)
    print(f"fingertips reach {poses['finger_reach'] * 1000:.0f} mm past the hand's frame (measured); "
          f"START reads {poses['press_depth_mm']:+.0f} mm from the nominal press (calibrated)")
    if args.studio:
        bt.studio(scene)
        return
    try:
        tl = scene.simulate_sequences(PROGRAMS, max_duration=400.0)
    except ValueError as err:
        print(f"cycle failed: {err}")
        sys.exit(1)

    print(f"cycle {tl.duration:.2f}s — walking {tl.signal(LEGS).high_total():.2f}s over {len(tl.footfalls(ROBOT))} steps")
    for name, t0, t1 in tl.step_spans:
        if name.startswith("finish/") and not name.startswith("finish/gate clear"):
            print(f"  {name[7:]:<16} {t0:7.2f} - {t1:7.2f}s")
    for lane in ("washer_panel/start", "washer_panel/estop", "wash_running", "wash_done", "part_seen", "on_stage",
                 "stage_busy", "insp_ok", "insp_ng", "gate"):
        spans = ", ".join(f"{a:.2f}-{b:.2f}" for a, b in tl.signal(lane).high_spans())
        print(f"  {lane:<20} on: {spans or '-'}")
    clearance = tl.min_clearance()
    pair = f" ({clearance.pair[0]} x {clearance.pair[1]})" if clearance.pair else ""
    print(f"min clearance over the cycle: {float(clearance) * 1e3:.1f} mm at {clearance.t:.2f}s{pair}")
    end = tl.object_pose("part", tl.duration)[0]
    print(f"the part ends at ({end[0]:.2f}, {end[1]:.2f}, {end[2]:.2f}) — the shipping tray is at "
          f"{tuple(round(v, 2) for v in local('s', WORK[0], WORK[1], PART_TOP - PART[2] / 2))}")
    print(scene.requirements().to_markdown())

    warnings = tl.export_usd(args.out, fps=60)
    print(f"wrote {args.out}" + (f" ({warnings})" if warnings else ""))
    if args.no_deliver:
        return
    out_dir = Path(args.out).with_name("wash_inspect_ship_deliverables")
    _report, runs = deliver(scene, tl, out_dir)
    print(f"wrote the document set to {out_dir}/")
    for name in list(runs.names) + [n for n in runs.errors if n not in runs.names]:
        err = runs.errors.get(name)
        status = f"refused — {err}" if err else f"completed in {runs.durations[name]:.2f}s"
        print(f"  scenario {name:<18} {status}")


if __name__ == "__main__":
    main()

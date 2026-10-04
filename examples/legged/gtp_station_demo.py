"""棚搬送 AGV の出荷ステーションに人型を入れる — a Unitree H2 takes the
worker's place at the centre window of a goods-to-person (shelf-to-person
AGV) shipping station rebuilt from one frame of a video: it reads each
carton's QR code with its head camera, takes the carton out of the window in
both hands and loads it into the roll cage beside the window, crouching for
the low layers (`bt.seq.crouch`).

The station is `_gtp_station.py` (as the frame shows it: the fence and its
three windows, the pod field, the people at the other windows). What the
proposal changes, and nothing else:

  * a Unitree H2 (catalog `unitree/h2/h2` r2, 1.82 m, two 7-axis arms, a
    3-axis waist and a 2-axis neck) works the centre window where the worker
    stood; its head camera reads the carton labels (a vision sensor on the
    camera, range and occlusion checked);
  * the AGVs present the racks at the window as today; the robot's racks
    carry one column of medium cartons (100 size, 4.5 kg), three high on a
    platform (0.65 to 1.4 m), the hands going either side of them; the AGVs
    are 日立 Racrew (catalog) under 1.2 m racks;
  * the roll cage stands where the table stood, left of the window, an open
    side to the robot's path (a catalog MRC-S5, as at the other windows); a
    resin platform raises its floor to 0.41 m; the table and its papers go
    (the QR read is the record).

The cycle, per carton: look at the label → QR read → head up → both palms
onto its end faces (crouching for the lower ones, from a step back) → lift →
stand up → back away along the window line (that draws the carton out of the
rack) → the carton in to the chest → on to the cage → turn the torso to the
cage on the left (the legs keep facing the window: crouched facing the cage
its knees would meet it; for the upper layers the carton lifted before the
chest first) → crouch → set the carton into its slot from above → hands out
→ stand up → back to the window. The hands keep the grip they took (no
re-grip), which is what sets the heights of the rack and the cage. Two racks
fill the cage's column five high (a sixth is beyond the H2 from beside it);
the rack swap between them is the AGVs' own.

How it moves: every pose is taught by IK
against the cell (with the reach into it and the stand-up out of it clear);
every move between two poses is a straight joint line checked against the
cell as it stands at that point of the cycle, by way of via poses where it is
not, run as a ramp timed to the joints' pace; the walks are checked with the
gait's sway; crouches are `bt.seq.crouch` (feet planted, legs solved by the
walk's IK, the robot and what it holds checked every tick); the baked run is
replayed against the cell every 50 ms (`audit`).

The result (5 cartons from two racks in about 143 s, no contact in the
50 ms replay) is printed with where the time goes and every crouch. The
first run fetches the H2, the drive unit (日立 Racrew), the fence's, the roll
cage's and the containers' packs from the catalog and teaches every pose (five
processes, a few minutes; cached under ~/.cache/botrail/demo after).

Run with:  python examples/legged/gtp_station_demo.py [out.usdc] [--studio] [--no-ceiling]
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import botrail as bt

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import _gtp_station as S

H2 = "unitree/h2/h2"
ROBOT, LEGS = "h2", "legs"
WAIST = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
HEAD = ["head_yaw_joint", "head_pitch_joint"]
ARM = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
HANDS = {"left": "left_hand_link", "right": "right_hand_link"}
SPEED, TURN, REVERSE = 0.4, 0.8, 0.2   # m/s, rad/s, m/s backing — a careful walk beside the fence

# The upper body's pace (rad/s; accelerations twice that): the URDF gives the
# motors' limits (10-38 rad/s), which time a reach as a whip. Written into the
# model the robot is built from; every move is timed to it (`ramp_time`).
PACE = {"shoulder": 1.2, "elbow": 1.4, "wrist": 2.0, "waist": 1.0, "head": 1.5}

# ---------------------------------------------------------------- the window
WIN = S.WINDOWS["centre"]
WIN_Y = (WIN[0] + WIN[1]) / 2        # 0.54
RACK_X = S.RACK_FRONT + S.RACK[0] / 2  # the rack's centre
RACK_GAP = 0.10                      # between the rack's two columns: the hand is 8.6 cm thick
# the robot's racks present their cartons between 0.65 and 1.4 m: a platform
# on the deck of a 1.6 m rack (the deck 0.52 m up, carried). From a plain
# rack's deck the hands must grip a carton steeply from above, and a carton
# gripped so (the hands do not re-grip) cannot go onto the cage's top layers
CARTON_FLOOR = 0.645                 # the lowest carton's underside
RACK_RISER, RACK_TALL = round(CARTON_FLOOR - S.RACK_SEAT, 3), 1.60
CARTON_KG = 4.5
AGV_SPEED, AGV_TURN = 0.8, 1.2       # m/s, rad/s
AGV_TURN_X = 2.30                    # where an emptied rack turns away south (north, the left window's next pod waits)
AGV_WAIT_X = 3.85                    # where the next rack waits, in line behind the window (clear of the 1.2 m
                                     # rack spinning round at AGV_TURN_X: 0.85 m to its corners)

# ---------------------------------------------------------------- the cage
# where the table stood, left of the window: turned a quarter so one open side
# faces the robot's path (-y); its end frames on its ±x faces. One column of
# cartons down its middle (the hands go either side), five high
CAGE_FRONT = 0.89                    # the open side's plane (y); the cage beyond it
CAGE_X = -1.15                       # the cartons' column (and the stand beside it)
CAGE_SHIFT = 0.035                   # the cage this far toward the window off the column: MRC-S5's end frames stand
                                     # 0.52 m off its middle, and the top carton swung in wants 0.555 on the window
                                     # side. Its frame there is then 0.565 m off the fence, 0.215 m behind the lower
                                     # window stand (nearer, the left elbow meets it as the arms reach into the rack)
RISER = (0.98, 0.70, 0.166)          # a resin platform raising the cage's floor to 0.41 m: a carton taken from the
                                     # top of the rack (gripped level) reaches no lower, and five layers still fit
CAGE_DECK = S.CAGE_DECK_Z + RISER[2]
CAGE_GAP = 0.10
# the upper three layers 0.10 m deeper: at chest height the stack is beside the
# left arm (the cage's open side is 0.27 m off the robot's middle), and the
# arms need that room to swing a carton up past it (still 0.05 m over the
# layer below at the back: it stands)
UPPER_FROM, UPPER_SETBACK = 3, 0.03
CAGE_LAYERS = 5                      # five high (1.66 m): over a fifth layer at chest height the left elbow,
                                     # swinging a carton round, meets it, and higher is out of the H2's reach from
                                     # beside the cage; the second rack's last carton waits for the next cage

# ---------------------------------------------------------------- the stands
# (x, y, heading): the pelvis over the floor, the heading it works in.
# The legs always face the window: the robot steps back from the rack and
# loads the cage beside it with its torso turned a quarter (the waist turns
# 100 degrees). Crouched facing the cage its knees would meet the cage and
# the cartons in it; turned, they point along the cage's front.
LINE_Y = 0.55                        # the line it walks: on the cartons' middle, 0.34 m off the cage's open side
                                     # (walking past the cage's end panel, the shoulders reach 0.27 m off the middle,
                                     # the arms holding a carton out further)
STATIONS = {
    "win": (-0.15, LINE_Y, 0.0),            # at the window, its toes at the fence line
    "win_lo": (-0.35, LINE_Y, 0.0),         # 0.2 m back for the low cartons (crouched, the knees clear the rack)
    "side": (CAGE_X, LINE_Y, 0.0),          # beside the cage, which is on its left
    "clear": (-0.50, LINE_Y, 0.0),          # backed out of the window: the carton out of the rack, short of the cage
}
HOLD_YAW = {"win": 0.0, "win_lo": 0.0, "side": math.pi / 2, "clear": 0.0}   # the way the hands reach at each stand


START = "win"                        # where the robot stands when the cycle begins


def local(st: str, fwd: float, right: float, up: float) -> tuple:
    """A point in the frame of the robot standing at `st`: forward, right, up (world z)."""
    x0, y0, h = STATIONS[st]
    return (x0 + fwd * math.cos(h) + right * math.sin(h), y0 + fwd * math.sin(h) - right * math.cos(h), up)


def yaw_q(a: float) -> tuple:
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


# ================================================================ the robot
def paced_urdf() -> Path:
    """The catalog package's H2 URDF (r2: the silver/black finish) with the
    upper body paced (PACE) and the mesh paths made absolute, written beside
    the botrail cache (a new name per pace: models load once per path)."""
    pkg = Path(bt._core.catalog_package(H2))
    xml = (pkg / "urdf" / "model.urdf").read_text()
    xml = xml.replace('filename="../', f'filename="{pkg}/')

    def pace(m: re.Match) -> str:
        name, body = m.group(1), m.group(2)
        for key, v in PACE.items():
            if key in name and "hip" not in name and "ankle" not in name and "knee" not in name:
                body = re.sub(r'velocity="[0-9.]+"', f'velocity="{v}"', body)
                break
        return f'<joint name="{name}" type="revolute">{body}</joint>'

    xml = re.sub(r'<joint name="([^"]+)" type="revolute">(.*?)</joint>', pace, xml, flags=re.DOTALL)
    tag = "-".join(f"{k}{v:g}" for k, v in sorted(PACE.items())) + "-" + pkg.name
    out = Path.home() / ".cache" / "botrail" / "demo" / f"h2_gtp_{tag}.urdf"
    out.parent.mkdir(parents=True, exist_ok=True)
    if not out.exists() or out.read_text() != xml:
        out.write_text(xml)
    return out


def build_robot():
    """The H2 (paced) with each arm as a planning group, each arm with the
    waist in front of it, and the neck. The gait is the catalog package's."""
    robot = bt.Robot.from_urdf(paced_urdf())
    for side in ("left", "right"):
        joints = [f"{side}_{j}_joint" for j in ARM]
        robot = robot.define_group(side, tip=HANDS[side], joints=joints)
        robot = robot.define_group(f"{side}_w", tip=HANDS[side], joints=WAIST + joints)
    robot = robot.define_group("head", tip="head_pitch_link", joints=HEAD)
    return robot, bt.Gait.from_catalog(H2)


def joint_pace() -> dict:
    """Each joint's velocity bound in the paced model (rad/s)."""
    root = ET.parse(paced_urdf()).getroot()
    return {j.get("name"): float(j.find("limit").get("velocity")) for j in root.iter("joint") if j.find("limit") is not None}


# ================================================================ the cell
@dataclass
class Cell:
    """The names and places the programs address."""

    station: S.Station
    cartons: list = field(default_factory=list)      # on the rack, in the order the robot takes them
    seats: dict = field(default_factory=dict)        # carton -> its seat on the rack (underside centre)
    slots: dict = field(default_factory=dict)        # carton -> its slot in the cage (underside centre)
    rack: list = field(default_factory=list)         # the rack's obstacles (they ride the AGV)
    next_rack: list = field(default_factory=list)    # the next rack, waiting behind on the second AGV
    rack_of: dict = field(default_factory=dict)      # carton -> the AGV that brings it
    cage: list = field(default_factory=list)
    qr: dict = field(default_factory=dict)           # carton -> the vision sensor that reads its label


def cage_slot(layer: int) -> tuple:
    """A carton's underside centre in the cage: its length across the cage,
    its labelled face 2 cm inside the open side (the upper half 0.10 m
    deeper), `layer` from the deck."""
    back = UPPER_SETBACK if layer >= UPPER_FROM else 0.0
    return (CAGE_X, CAGE_FRONT + 0.02 + back + S.CARTON[1] / 2, CAGE_DECK + 0.002 + S.CARTON[2] * layer)


def cage_pose() -> tuple:
    """The cage's centre and yaw: left of the window, an open side to the robot."""
    return (CAGE_X + CAGE_SHIFT, CAGE_FRONT + S.CAGE[1] / 2), 0.0


def head_camera(scene) -> str:
    """The H2's head camera: the model's `camera` link on the head (the neck
    pitches, then yaws: the head is `head_yaw_link`), looking along the
    head's +x (a camera looks along its -Z with +Y up)."""
    import numpy as np
    m = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])   # columns: the camera's X, Y, Z in the head frame
    w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    q = ((m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w), w)
    scene.add_camera("h2/head_camera", robot=ROBOT, link="camera", position=(0.0, 0.0, 0.0), quaternion=q, fov=70.0,
                     resolution=(1280, 800), near=0.1, far=3.0, body_visible=False)
    return "h2/head_camera"


def build_scene(*, ceiling: bool = True):
    robot, gait = build_robot()
    scene = bt.Scene(robot, name=ROBOT)
    # one straight line: it backs from the window to the cage's side and
    # walks forward again, never turning
    order = ("side", "clear", "win_lo", "win")
    path = [STATIONS[k][:2] for k in order]
    scene.add_vehicle(LEGS, body=[], path=path, stations={k: i for i, k in enumerate(order)}, speed=SPEED, turn_speed=TURN,
                      start=START, allow_reverse=True, reverse_speed=REVERSE,
                      arrive={"win": "forward", "clear": "reverse", "side": "reverse"})
    scene.mount_robot(LEGS, robot=ROBOT, gait=gait)
    scene.set_part(ROBOT, kind="robot", catalog=H2, manufacturer="Unitree Robotics", model="H2", payload_kg=7.0,
                   description="humanoid, 1.82 m; arms paced to 1.2-2.0 rad/s in this cell")
    st = S.build(scene, to_be=True, ceiling=ceiling)
    cell = Cell(st)
    # the cage where the table stood, an open side to the robot, its floor raised
    cell.cage = S.roll_cage(scene, "centre/cage", *cage_pose())
    (cx, cy), _ = cage_pose()
    riser = S.solid(scene, "centre/cage/riser", RISER, (cx, cy, S.CAGE_DECK_Z + RISER[2] / 2), S.srgb(38, 62, 110),
                    roughness=0.55, finish="plastic")
    scene.set_part(riser, category="structure.pallet", model="樹脂製底上げ台 {}x{}x{}(カゴ台車)".format(*(round(v * 1000) for v in RISER)),
                   description="added for the robot: the lowest layer within its crouched reach")
    cell.cage.append(riser)
    # the rack the AGV presents (one column of medium cartons, three high) and
    # the next one waiting in line behind it on the second AGV
    cell.rack, seats1 = S.carton_rack(scene, "rack1", WIN_Y, load="medium", gap=RACK_GAP, riser=RACK_RISER, height=RACK_TALL)
    cell.next_rack, seats2 = S.carton_rack(scene, "rack2", WIN_Y, load="medium", gap=RACK_GAP, front=AGV_WAIT_X - S.RACK[0] / 2,
                                           riser=RACK_RISER, height=RACK_TALL)
    top_down = lambda seats: sorted(seats, key=lambda c: -seats[c][2])
    shift = S.RACK_FRONT - (AGV_WAIT_X - S.RACK[0] / 2)        # rack 2 is picked where rack 1 stood
    cell.cartons = (top_down(seats1) + top_down(seats2))[:CAGE_LAYERS]
    # what the cage cannot take stays on its rack (and rides it)
    cell.next_rack += [c for c in seats2 if c not in cell.cartons]
    cell.seats = {**{c: seats1[c] for c in seats1}, **{c: (seats2[c][0] + shift, seats2[c][1], seats2[c][2]) for c in seats2}}
    cell.rack_of = {**{c: "agv1" for c in seats1}, **{c: "agv2" for c in seats2}}
    # set into the cage bottom up, one column: rack 1 the lower three, rack 2 the upper
    cell.slots = {c: cage_slot(layer) for layer, c in enumerate(cell.cartons)}
    for c in cell.cartons:
        scene.set_part(c, category="workpiece", model="段ボールケース 400x300x250(中、行き先別)", mass_kg=CARTON_KG)
    # the AGVs under the racks: the first presents its rack at the window and,
    # emptied, backs it out and takes it away south; the second then brings
    # its rack in from the line behind
    tray = ((0.0, 0.0, S.RACK_SEAT + RACK_RISER + 0.42), (S.RACK[0] - 0.04, S.RACK[1] - 0.04, 0.84))
    scene.add_vehicle("agv1", body=list(cell.rack), path=[(RACK_X, WIN_Y), (AGV_TURN_X, WIN_Y), (AGV_TURN_X, WIN_Y - 2.4)],
                      stations={"window": 0, "lane": 1, "away": 2}, speed=AGV_SPEED, turn_speed=AGV_TURN, start="window",
                      allow_reverse=True, tray_position=tray[0], tray_size=tray[1])
    scene.add_vehicle("agv2", body=list(cell.next_rack), path=[(AGV_WAIT_X, WIN_Y), (RACK_X, WIN_Y)],
                      stations={"wait": 0, "window": 1}, speed=AGV_SPEED, turn_speed=AGV_TURN, start="wait",
                      tray_position=tray[0], tray_size=tray[1])
    for agv in ("agv1", "agv2"):
        _root, m = S.agv_package()
        scene.set_part(agv, kind="device", category=m["category"], catalog=S.AGV_PACK, manufacturer=m["manufacturer"]["name"],
                       model=m["name"], description="existing")
    cam = head_camera(scene)
    for k, c in enumerate(cell.cartons):
        name = f"h2/qr{k}"
        scene.add_vision_sensor(name, camera=cam, watch=[c], detect_range=(0.25, 1.4))
        cell.qr[c] = name
    return scene, cell


# ================================================================ teaching
def rot(q, v):
    return S.rotate(q, v)


def qmul(a, b) -> tuple:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def qy(a: float) -> tuple:
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


def compose(a, b) -> tuple:
    """Pose a then b (each (position, quaternion))."""
    (pa, qa), (pb, qb) = a, b
    return (tuple(x + y for x, y in zip(pa, rot(qa, pb))), qmul(qa, qb))


def invert(a) -> tuple:
    p, q = a
    qi = (-q[0], -q[1], -q[2], q[3])
    return (tuple(-v for v in rot(qi, p)), qi)


def frame_for(q, point, offset) -> tuple:
    """The hand frame's position that puts the hand's local `offset` at `point`."""
    o = rot(q, offset)
    return tuple(p - c for p, c in zip(point, o))


# The hand, measured off the package's mesh (hand frame): fingers along +x
# (tips at 0.152), the palm toward -y on the left hand and +y on the right,
# the fingers curled 4 cm past the palm (the hand is 8.6 cm thick: y from
# -0.060 to 0.026 on the left), its width along z (-0.039 .. 0.075). A fixed
# hand: a carton is held by pressing the palm sides of both hands on its long
# faces and attaching it.
HAND_PALM_SIDE = {"left": -0.060, "right": 0.061}
HAND_GRIP = 0.075                    # along the fingers: where the hand meets the carton
HAND_MID_Z = 0.018
GRIP_DEPTH = 0.15                    # the palms this far behind the carton's near (labelled) face: mid-depth
APPROACH = 0.04                      # the hands open this far outward before they close on a carton
# The box as the hands see it: (depth along the reach, width between the palms,
# height). The H2 holds a carton by its two END faces, its length across the
# body: the hands then pass outside the knees in a deep crouch (on the long
# faces, 0.30 m apart, the wrists hit its own hips and knees below 0.5 m)
HELD = (S.CARTON[1], S.CARTON[0], S.CARTON[2])
READY_BOX = (0.24, 0.20, 0.20)        # the box the ready hands are round, for via poses
# the ready pose's left arm (mirrored for the right): forearms raised before
# the chest, the elbows in (0.21 m off the middle — out wider they meet the
# cage beside the window), the hands at 1.18 m, over a carton in the window
HUB = {"shoulder_pitch": 0.7, "shoulder_roll": 0.1, "shoulder_yaw": -0.1, "elbow": -0.3,
       "wrist_roll": 0.8, "wrist_pitch": -0.5, "wrist_yaw": -0.6}


def hold_targets(centre, yaw: float, *, tilt: float = 0.0, out: float = 0.0, grip: float | None = None, box=None) -> dict:
    """Both hands' frames (position, quaternion) holding a carton whose
    centre is `centre` and whose length points along `yaw` (away from the
    robot): the palms on its long faces `grip` behind its near end at
    mid-height, the fingers along the carton tilted down by `tilt`, each
    hand `out` off its face."""
    grip = GRIP_DEPTH if grip is None else grip
    q = qmul(yaw_q(yaw), qy(tilt))
    qc = yaw_q(yaw)
    l, w, _h = box or HELD
    out_ = {}
    for side, sgn in (("left", 1.0), ("right", -1.0)):
        local_pt = (-l / 2 + grip, sgn * (w / 2 + out), 0.0)
        p = tuple(c + d for c, d in zip(centre, rot(qc, local_pt)))
        out_[side] = (frame_for(q, p, (HAND_GRIP, HAND_PALM_SIDE[side], HAND_MID_Z)), q)
    return out_


class Teacher:
    """Stands the robot at a stand (crouched as asked) and solves both hands
    there against the real cell (collisions checked; the contacts a grip
    makes allowed). Remembers every pose by name, with its crouch."""

    def __init__(self, scene):
        self.scene = scene
        self.names = scene.robot_of(ROBOT).joint_names
        self.q_home = list(scene.joint_positions_of(ROBOT))
        self.base = scene.robot_base_pose_of(ROBOT)
        x, y, h = STATIONS[START]                # where the robot was built standing
        p = self.base[0]
        self.offset = rot(yaw_q(-h), (p[0] - x, p[1] - y, p[2]))
        self.poses: dict = {}
        self.crouches: dict = {}
        self.tilts: dict = {}
        self.station = None
        self.crouch = (0.0, 0.0)

    def stand(self, st: str, crouch=(0.0, 0.0), q=None):
        """The robot standing at `st`, crouched `(depth, lean)`, the upper
        body as `q` has it (the home pose by default)."""
        x, y, h = STATIONS[st]
        dx, dy, dz = rot(yaw_q(h), self.offset)
        q = list(self.q_home if q is None else q)
        legs = [i for i, n in enumerate(self.names) if any(k in n for k in ("hip", "knee", "ankle"))]
        for i in legs:
            q[i] = self.q_home[i]
        self.scene.set_robot_base_pose((x + dx, y + dy, dz), yaw_q(h), robot=ROBOT)
        self.scene.set_joint_positions(q, robot=ROBOT)
        if crouch[0] > 1e-6 or abs(crouch[1]) > 1e-6:
            try:
                base, qc = self.scene.crouch_pose(crouch[0], lean=crouch[1], robot=ROBOT)
            except ValueError as err:          # the legs do not go that low
                raise RuntimeError(f"no crouch {crouch}: {err}") from None
            for i in legs:
                q[i] = qc[i]
            self.scene.set_robot_base_pose(*base, robot=ROBOT)
            self.scene.set_joint_positions(q, robot=ROBOT)
        self.station, self.crouch = st, tuple(crouch)
        return q

    def restore(self):
        self.scene.set_robot_base_pose(*self.base, robot=ROBOT)
        self.scene.set_joint_positions(self.q_home, robot=ROBOT)

    def with_joints(self, base, values: dict) -> list:
        q = list(base)
        for j, v in values.items():
            q[self.names.index(j)] = v
        return q

    def touching(self, allow=()) -> list:
        """The robot's contacts, a hand on an `allow`ed obstacle excepted
        (a knee on the carton it is about to take is still a contact)."""
        allowed, hands = set(allow), set(HANDS.values())
        out = []
        for a, b in self.scene.check_collisions():
            pair = {a[1], b[1]}
            if pair & allowed and pair & hands:
                continue
            out.append((a[1], b[1]))
        return out

    def arm_seeds(self, side: str):
        """Arm postures to start the IK from. On the H2 the elbow at 0 is a
        right angle (the forearm level, ahead) and positive straightens it
        downward."""
        sg = 1.0 if side == "left" else -1.0
        for sp, sr, sy, el in ((0.0, 0.15, 0.0, 0.0), (-0.4, 0.1, 0.0, 0.3), (0.2, 0.2, 0.0, -0.3), (-0.8, 0.1, 0.0, 0.8),
                               (0.3, 0.25, 0.2, 0.6), (-0.3, 0.3, -0.3, -0.5), (0.4, 0.15, 0.0, 1.2)):
            yield {f"{side}_shoulder_pitch_joint": sp, f"{side}_shoulder_roll_joint": sr * sg, f"{side}_shoulder_yaw_joint": sy * sg,
                   f"{side}_elbow_joint": el}

    def solve_hands(self, targets: dict, q0: list, allow=()) -> tuple:
        """Both arms (each on its own group, the waist as `q0` has it) onto
        `targets` {side: (position, quaternion)}; (q, None) or (None, why)."""
        q = list(q0)
        for side in ("left", "right"):
            pos, quat = targets[side]
            best, done = None, False
            for seed in [None, *self.arm_seeds(side)]:
                s = q if seed is None else self.with_joints(q, seed)
                self.scene.set_joint_positions(s, robot=ROBOT)
                r = self.scene.set_tcp_target(pos, quat, group=side, robot=ROBOT)
                if r.converged:
                    q = list(self.scene.joint_positions_of(ROBOT))
                    done = True
                    break
                best = r.pos_error if best is None else min(best, r.pos_error)
            if not done:
                return None, f"{side} hand {best * 1000:.0f} mm short"
        self.scene.set_joint_positions(q, robot=ROBOT)
        pairs = self.touching(allow)
        if pairs:
            return None, "touches " + ", ".join(f"{a} x {b}" for a, b in pairs[:3])
        return q, None

    def hold(self, name: str, st: str, centre, yaw: float, *, crouches=((0.0, 0.0),), waists=None,
             tilts=(0.0, 0.3, 0.6, 0.9, 1.2), out: float = 0.0, allow=(), seed=None) -> list:
        """The first (crouch, waist, tilt) that puts both hands on the carton
        at `centre` (length along `yaw`); remembered as `name`."""
        if waists is None:
            # the torso turned toward the carton first, then bent
            x0, y0, h = STATIONS[st]
            toward = math.atan2(centre[1] - y0, centre[0] - x0) - h
            toward = (toward + math.pi) % (2 * math.pi) - math.pi
            # (leaning back a little is how a short arm holds a box close to the chest)
            yaws = [toward] if abs(toward) < 0.3 else [toward, toward - 0.17 * math.copysign(1, toward),
                                                        max(-1.74, min(1.74, toward + 0.17 * math.copysign(1, toward)))]
            yaws += [0.6 * toward, 0.0]
            waists = [(wy, wp) for wp in (0.0, -0.15, 0.2, -0.3, 0.4, 0.52) for wy in yaws]
        why = []
        for crouch in crouches:
            try:
                base_q = self.stand(st, crouch, seed)
            except RuntimeError as err:
                why.append(str(err)[:80])
                continue
            for wy, wp in waists:
                q0 = self.with_joints(base_q, {"waist_yaw_joint": wy, "waist_pitch_joint": wp, "waist_roll_joint": 0.0})
                for tilt in tilts:
                    q, w = self.solve_hands(hold_targets(centre, yaw, tilt=tilt, out=out), q0, allow)
                    if q is not None:
                        self.poses[name], self.crouches[name], self.tilts[name] = q, tuple(crouch), tilt
                        return q
                    why.append(f"crouch {crouch} waist {wp:.2f} tilt {tilt:.1f}: {w}")
        raise RuntimeError(f"cannot teach `{name}` at {st}: " + " | ".join(why[-4:]))


# ================================================================ what is taught
CARRY = (0.33, 1.15)                 # the carried carton's centre: this far ahead of the pelvis, this high
CARRY_RIGHT = (0.0, 0.06, 0.12)       # ... and right of the middle, nearest first (clear of the loaded cage)
SWAY = 0.03                          # the gait sways the body 2.5 cm either side of the line it walks
                                      # (the H2 holds a box by its sides at chest height, not lower)
LIFT = 0.025                         # off its seat before it moves
PULL = (0.08, 0.14)                  # drawn back this far before standing up, where it must be
SLOT_OUT = 0.18                      # held this far out of its cage slot before it goes in
SLOT_UP = 0.025                      # and this high over its seat
WITHDRAW = (0.05, 0.22)              # the open hands drawn back and up out of the cage (back toward the body,
                                      # crouched, the arms do not fold)
WITHDRAW_HIGH = (0.20, 0.03)          # ... and from the upper layers, back toward the body (up is out of reach)
# crouches tried, shallowest first: (depth, lean)
CROUCHES = [(0.0, 0.0)] + [(d, l) for d in (0.10, 0.20, 0.30, 0.38, 0.45, 0.52, 0.58) for l in (0.0, 0.2, 0.4)]


def centre_of(seat) -> tuple:
    return (seat[0], seat[1], seat[2] + S.CARTON[2] / 2)


def shifted(p, dx=0.0, dy=0.0, dz=0.0) -> tuple:
    return (p[0] + dx, p[1] + dy, p[2] + dz)


class Arranger:
    """Puts the cell in the state a pose meets it in — the racks where the
    AGVs have them, the cartons already loaded in the cage, the held carton
    in the hands — and puts everything back."""

    def __init__(self, scene, c: Cell):
        self.scene, self.c = scene, c
        movers = list(c.rack) + list(c.next_rack) + list(c.cartons)
        self.home = {o: scene.obstacle_pose(o) for o in movers}

    def restore(self):
        for o, pose in self.home.items():
            self.scene.set_obstacle_pose(o, *pose)

    def move(self, o: str, d):
        (x, y, z), q = self.home[o]
        self.scene.set_obstacle_pose(o, (x + d[0], y + d[1], z + d[2]), q)

    def arrange(self, k: int):
        """As the k-th carton finds the cell: the cartons before it in the
        cage; from the fourth on, the first rack gone and the second at the
        window."""
        self.restore()
        c = self.c
        if k >= 3:
            for o in c.rack + [x for x in c.cartons if c.rack_of[x] == "agv1"]:
                self.move(o, (0.0, 30.0, 0.0))
            shift = S.RACK_FRONT - (AGV_WAIT_X - S.RACK[0] / 2)
            for o in c.next_rack + [x for x in c.cartons if c.rack_of[x] == "agv2"]:
                self.move(o, (shift, 0.0, 0.0))
        for j in range(k):
            self.put(c.cartons[j], centre_of(c.slots[c.cartons[j]]), HOLD_YAW["side"])

    def put(self, carton: str, centre, yaw: float):
        """A carton with its centre at `centre`, held by a robot facing `yaw`:
        its length across the robot and its labelled face toward it."""
        self.scene.set_obstacle_pose(carton, centre, yaw_q(yaw + math.pi / 2))


def held_contacts(T: Teacher, carton: str) -> list:
    """What the carton, held in the left hand where it stands, touches —
    the hands excepted (attached for the check, then let go)."""
    sc = T.scene
    sc.attach(carton, link=HANDS["left"], touch_links=list(HANDS.values()), robot=ROBOT)
    try:
        return [(a[1], b[1]) for a, b in sc.check_collisions() if carton in (a[1], b[1])]
    finally:
        sc.detach(carton)


LEG_WORDS, ARM_WORDS = ("hip", "knee", "ankle"), ("shoulder_yaw", "elbow", "wrist", "hand")


def body_line(T: Teacher, a: list, b: list, step: float = 0.02) -> list:
    """The contacts of the body (the arms aside: a via pose steers them)
    along the straight joint line from `a` to `b`, the legs as they stand."""
    sc = T.scene
    now = sc.joint_positions_of(ROBOT)
    legs = [i for i, n in enumerate(T.names) if any(w in n for w in LEG_WORDS)]
    n = max(2, math.ceil(math.dist(a, b) / step))
    try:
        for i in range(n + 1):
            q = [x + (y - x) * i / n for x, y in zip(a, b)]
            for j in legs:
                q[j] = now[j]
            sc.set_joint_positions(q, robot=ROBOT)
            bad = [(p[1], o[1]) for p, o in sc.check_collisions() if not any(w in p[1] + o[1] for w in ARM_WORDS)]
            if bad:
                return bad
        return []
    finally:
        sc.set_joint_positions(now, robot=ROBOT)


def stand_up_contacts(T: Teacher, arr: Arranger, st: str, crouch, q: list, held, centre, yaw: float) -> list:
    """What the robot (and the carton it holds, carried with the hands)
    meets standing up from `crouch` at `st` with the upper body as `q` has
    it — the hands on that carton excepted."""
    sc = T.scene
    rel = None
    if held is not None:
        arr.put(held, centre, yaw)
        T.stand(st, crouch, q)
        rel = compose(invert(sc.link_pose(HANDS["left"], robot=ROBOT)), sc.obstacle_pose(held))
    try:
        for f in (0.75, 0.5, 0.25, 0.0):
            T.stand(st, (crouch[0] * f, crouch[1] * f), q)
            if held is not None:
                sc.set_obstacle_pose(held, *compose(sc.link_pose(HANDS["left"], robot=ROBOT), rel))
            bad = T.touching(allow=(held,) if held else ()) + (held_contacts(T, held) if held else [])
            if bad:
                return bad
        return []
    finally:
        if held is not None:
            arr.put(held, centre, yaw)


def teach_group(T: Teacher, P: dict, st: str, carton: str, poses: list, *, arr: Arranger, crouches=CROUCHES,
                yaw: float | None = None, enter: str | None = None, tilt: float | None = None, seed0=None,
                rise: bool = False) -> tuple:
    """Poses at one stand that share one crouch (the body does not move
    between them): `poses` = [(name, carton centre, out, held)], `held` = the
    carton is in the hands there (it is put there and must touch nothing but
    the hands). The first crouch, shallowest first, where every pose solves
    against the cell. Returns the crouch. `yaw` = the way the hands reach
    (the stand's own by default). `enter` = a pose the first is reached from
    in a straight line, the body clear on the way (leaning in under the
    rack's top bar, the head would have to pass through it). `tilt` = the
    hands' tilt wherever they are on the carton or hold it (once gripped
    the carton is fixed in the hands: a hold at another tilt would pitch it).
    `seed0` = the arms to start the first pose's IK from (the pose before it
    in the cycle: the arms keep their posture, and the straight joint line
    between the two stays near the straight line in space). `rise` = it
    stands up from the last pose as it is: that must be clear the whole way."""
    yaw = HOLD_YAW.get(st, STATIONS[st][2]) if yaw is None else yaw
    why = []
    # skip the crouches that cannot be enough: standing reaches a carton's
    # middle down to about 0.95 m, and each metre lower asks for about as much
    # crouch — start a little shallower than that, shallowest first, and try
    # the rest after
    low = min(centre[2] for _n, centre, _o, _h in poses)
    guess = max(0.0, min(0.55, 0.95 - low)) - 0.12
    crouches = sorted(crouches, key=lambda c: (c[0] < guess, c[0], c[1]))
    for crouch in crouches:
        got, seed = {}, seed0
        for name, centre, out, held in poses:
            if carton is not None:
                arr.put(carton, centre, yaw)
            try:
                q = T.hold(name, st, centre, yaw, crouches=(crouch,), out=out, seed=seed,
                           allow=(carton,) if carton and (held or out < 0.02) else (),
                           **({"tilts": (tilt,)} if tilt is not None and (held or out < 1e-6) else {}))
            except RuntimeError as err:
                why.append(f"{crouch}: {str(err)[-150:]}")
                break
            if held:
                bad = held_contacts(T, carton)
                if bad:
                    why.append(f"{crouch}: {name} holds the carton against {bad[:2]}")
                    break
            got[name], seed = q, q
        else:
            if enter is not None:
                name, centre = poses[0][0], poses[0][1]
                if carton is not None:
                    arr.put(carton, centre, yaw)
                T.stand(st, crouch, got[name])
                bad = body_line(T, P[enter], got[name])
                if bad:
                    why.append(f"{crouch}: from {enter} the body meets {bad[:2]}")
                    continue
            if rise and crouch != (0.0, 0.0):
                # and it stands up from the last pose as it is, the whole way,
                # the carton (held) rising with the hands — no via steers a
                # crouch (the elbows of a steep grip rise into the rack's top bar)
                last = poses[-1]
                if not last[3] and carton is not None:
                    # let go: the carton is where the last pose on it left it
                    rest = next((cen for _n, cen, o, h in reversed(poses) if h or o < 1e-6), None)
                    if rest is not None:
                        arr.put(carton, rest, yaw)
                bad = stand_up_contacts(T, arr, st, crouch, got[last[0]], carton if last[3] else None, last[1], yaw)
                if bad:
                    why.append(f"{crouch}: standing up it meets {bad[:2]}")
                    continue
            P.update(got)
            meta = P.setdefault("meta", {})
            x0, y0, h = STATIONS[st]
            for name, centre, out, held in poses:
                T.crouches[name] = crouch
                dx, dy = centre[0] - x0, centre[1] - y0
                # in the stand's frame (forward, left, up) and the reach's yaw from the heading,
                # so a pose taught at one stand reads right at another
                rel = (dx * math.cos(h) + dy * math.sin(h), -dx * math.sin(h) + dy * math.cos(h), centre[2])
                meta[name] = {"rel": rel, "yaw": yaw - h, "out": out, "held": held, "box": tuple(HELD)}
            return crouch
    raise RuntimeError(f"cannot teach {poses[0][0]}..: " + " | ".join(why[-3:]))


def look_at(T: Teacher, q: list, target) -> list:
    """`q` with the neck turned so the head camera looks at `target` (the
    camera link looks along its +x; the neck pitches, then yaws): a coarse
    grid, then a finer one."""
    sc, names = T.scene, T.names
    iy, ip = names.index("head_yaw_joint"), names.index("head_pitch_joint")

    def miss(yaw, pitch):
        qq = list(q)
        qq[iy], qq[ip] = yaw, pitch
        cam, o = sc.link_pose_at("camera", qq, robot=ROBOT)
        ax = rot(o, (1.0, 0.0, 0.0))
        d = tuple(t - c for t, c in zip(target, cam))
        n = math.sqrt(sum(v * v for v in d))
        return -sum(a * b for a, b in zip(ax, d)) / n

    best = min((miss(y, p), y, p) for y in [i * 0.1 for i in range(-12, 13)] for p in [i * 0.06 for i in range(-8, 14)])
    _, y0, p0 = best
    best = min((miss(y, p), y, p) for y in [y0 + i * 0.02 for i in range(-5, 6)] for p in [p0 + i * 0.015 for i in range(-5, 6)])
    qq = list(q)
    qq[iy] = max(-1.7, min(1.7, best[1]))
    qq[ip] = max(-0.5, min(0.83, best[2]))
    return qq


def teach_shared(T: Teacher, P: dict, c: Cell, arr: Arranger) -> None:
    """The hands ready before the chest, narrower (they fit between the
    window's posts): the same arms everywhere. (The carry poses are taught
    with the cartons, one per grip tilt.)"""
    # ready: both forearms raised before the chest, elbows in (HUB): clear of
    # a carton in the window in front and of the cage beside it
    vals = {}
    for side, sg in (("left", 1.0), ("right", -1.0)):
        vals.update({f"{side}_{j}_joint": v * (sg if j in ("shoulder_roll", "shoulder_yaw", "wrist_roll", "wrist_yaw") else 1.0)
                     for j, v in HUB.items()})
    P["ready"] = T.with_joints(T.q_home, vals)
    for st in ("win", "win_lo", "side"):
        arr.arrange(0)
        T.stand(st, (0.0, 0.0), P["ready"])
        if T.touching():
            raise RuntimeError(f"the ready pose touches {T.touching()[:3]} at {st}")
    T.crouches["ready"] = (0.0, 0.0)
    P.setdefault("meta", {})["ready"] = {"rel": (0.15, 0.0, 1.18), "yaw": 0.0, "out": 0.0, "held": False, "box": READY_BOX}
    P["home"] = list(T.q_home)


# the hands' tilt on a carton (fingers down), level first. A carton keeps its
# grip from the rack to the cage, and the two ends want different hands: a
# high place takes only a level grip, a low one only a steep one, and a carton
# taken high gives no steep grip. Measured (tilt in rad, + fingers down): the
# cage's sixth layer (1.64 m) took -0.3..0, its floor (0.39 m, the floor 10 cm
# up) 0.75..1.2; the rack's carton at 1.27 m gave 0..0.75, one at 0.52 m
# 0.9..1.2. Hence the rack's cartons at 0.65..1.4 m and the cage's floor raised
# to 0.41 m, five layers
GRIP_TILTS = (0.0, 0.3, 0.6, 0.7, 0.75, 0.8, 0.9, 1.2)


def teach_carry(T: Teacher, P: dict, c: Cell, arr: Arranger, k: int, t: float) -> None:
    """Carton k carried before the chest at tilt `t`, taught beside the cage
    as the cage stands when it gets there: once the cartons loaded before it
    reach chest height they are at the left elbow (the cage's open side is
    0.27 m off the robot's middle), so the carry moves right until it clears."""
    why = []
    for right in CARRY_RIGHT:
        arr.arrange(k)
        centre = local("side", CARRY[0], right, CARRY[1])
        try:
            teach_group(T, P, "side", c.cartons[k], [(f"carry{k}", centre, 0.0, True)],
                        arr=arr, crouches=[(0.0, 0.0)], yaw=0.0, tilt=t)
        except RuntimeError as err:
            why.append(f"{right:+.2f}: {str(err)[-160:]}")
            continue
        # and carried past the cage's end panel, the body swayed toward it
        bad = past_the_panel(T, arr, c.cartons[k], P[f"carry{k}"], centre)
        if not bad:
            return
        why.append(f"{right:+.2f}: walking past the cage meets {bad[:2]}")
        del P[f"carry{k}"]
    raise RuntimeError(f"cannot carry carton {k}: " + " | ".join(why))


def past_the_panel(T: Teacher, arr: Arranger, carton: str, q: list, centre) -> list:
    """What the robot, holding `carton` (at `centre` in the pose `q` at the
    side stand), meets walking past the cage's end panel with the gait
    swaying it toward the cage."""
    sc = T.scene
    arr.put(carton, centre, 0.0)
    T.stand("side", (0.0, 0.0), q)
    sc.attach(carton, link=HANDS["left"], touch_links=list(HANDS.values()), robot=ROBOT)
    panel = cage_pose()[0][0] + S.CAGE[0] / 2
    try:
        for dx in (-0.15, -0.10, -0.05, 0.0, 0.05, 0.10, 0.15):
            STATIONS["_walk"] = (panel + dx, LINE_Y + SWAY, 0.0)
            T.stand("_walk", (0.0, 0.0), q)
            hits = [(a[1], b[1]) for a, b in sc.check_collisions()]
            if hits:
                return hits
        return []
    finally:
        sc.detach(carton)
        STATIONS.pop("_walk", None)
        arr.put(carton, centre, 0.0)


def teach_carton(T: Teacher, P: dict, c: Cell, arr: Arranger, k: int) -> None:
    """Carton k's poses with the hands level on it where they can be, then
    tilted further (the first tilt that the pick, the carry and the place
    all take: every hold keeps the grip's)."""
    why = []
    for t in GRIP_TILTS:
        for key in [x for x in P if x.startswith((f"pick{k}:", f"place{k}:", f"carry{k}", f"look{k}"))]:
            del P[key]
        try:
            teach_carton_at(T, P, c, arr, k, t)
        except RuntimeError as err:
            why.append(f"tilt {t}: {str(err)[-240:]}")
            continue
        P.setdefault("tilt", {})[k] = t
        return
    raise RuntimeError(f"cannot teach carton {k}: " + " || ".join(why))


def teach_carton_at(T: Teacher, P: dict, c: Cell, arr: Arranger, k: int, t: float) -> None:
    """Carton k's poses at hand tilt `t`: taken from the window (or a step
    back from it, where the knees would meet the rack), its label read,
    carried, and set into the cage."""
    carton = c.cartons[k]
    seat, slot = centre_of(c.seats[carton]), centre_of(c.slots[carton])
    why = []
    # a carton below the top layer is taken crouched, from the step back
    P.pop(f"pick{k}:out", None)
    for st, pull in [(st, pull) for st in (("win", "win_lo") if seat[2] > 0.9 else ("win_lo", "win")) for pull in (None, *PULL)]:
        arr.arrange(k)
        # lifted off its seat; backing away draws it out of the rack (pulled
        # all the way toward the body, crouched, it meets the hips) — drawn
        # back a little first where standing up with it in the rack meets the
        # rack (the elbows of a steep grip under its top bar)
        out = [(f"pick{k}:out", shifted(seat, dx=-pull, dz=LIFT), 0.0, True)] if pull else []
        try:
            crouch = teach_group(T, P, st, carton, [
                (f"pick{k}:pre", seat, APPROACH, False), (f"pick{k}:grip", seat, 0.0, False),
                (f"pick{k}:lift", shifted(seat, dz=LIFT), 0.0, True), *out], arr=arr, enter="ready", tilt=t, rise=True)
        except RuntimeError as err:
            why.append(f"{st}{f' pull {pull}' if pull else ''}: {str(err)[-200:]}")
            continue
        if not pull:
            P.pop(f"pick{k}:out", None)
        P.setdefault("pick_stand", {})[k] = st
        break
    else:
        raise RuntimeError(f"cannot pick carton {k}: " + " | ".join(why))
    try:
        teach_carry(T, P, c, arr, k, t)
    except RuntimeError:
        # no chest carry at this grip (steeply tilted hands raise the elbows
        # into the loaded cage): the carton goes to the cage from the lift's arms
        pass
    # the label, on the carton's face toward the robot, read from the pick's crouch
    T.stand(st, crouch, P[f"pick{k}:pre"])
    label = (seat[0] - HELD[0] / 2, seat[1], seat[2])
    P[f"look{k}"] = look_at(T, P["ready"], label)
    # beside the cage: the carry pose, crouched, the torso turned to the cage
    # and the arms setting it into its slot from above
    arr.arrange(k)
    wd = WITHDRAW_HIGH if slot[2] > 1.0 else WITHDRAW
    cp = teach_group(T, P, "side", carton, [
        (f"place{k}:above", shifted(slot, dz=SLOT_UP), 0.0, True), (f"place{k}:set", slot, 0.0, True),
        (f"place{k}:open", slot, APPROACH, False),
        (f"place{k}:back", shifted(slot, dy=-wd[0], dz=wd[1]), APPROACH, False)],
        arr=arr, tilt=t, rise=True)
    # for the upper layers, the carton lifted in front to its slot's height
    # first (carried at chest height it would swing through the stack below
    # its slot); the router takes the line from there or finds a via
    high = slot[2] + SLOT_UP
    if cp == (0.0, 0.0) and high > CARRY[1]:
        teach_rise(T, P, c, arr, k, t, high)


RISE_COLUMNS = [(fwd, right) for fwd in (0.33, 0.40, 0.46) for right in (0.0, 0.08, 0.16)]
RISE_STEPS = 4                       # waypoints from the carry's height up to the slot's


def rise_keys(P: dict, k: int) -> list:
    return [n for n in (*(f"place{k}:rise{i}" for i in range(1, RISE_STEPS)), f"place{k}:high") if n in P]


def teach_rise(T: Teacher, P: dict, c: Cell, arr: Arranger, k: int, t: float, top: float) -> None:
    """For an upper layer, the carton lifted before the chest from the carry
    up to over its slot's height (`place{k}:rise1..`, `place{k}:high`): a
    column of waypoints, each solved from the one before so the arms keep
    their posture. The first column (ahead, right of the middle) where every
    waypoint clears — lifted straight from the chest the left elbow flares
    into the cartons loaded beside it. None found: no lift (the router finds
    its own way)."""
    carton = c.cartons[k]
    for fwd, right in RISE_COLUMNS:
        seed, done = P.get(f"carry{k}") or P.get(f"pick{k}:lift"), []
        try:
            for i in range(1, RISE_STEPS + 1):
                name = f"place{k}:rise{i}" if i < RISE_STEPS else f"place{k}:high"
                z = CARRY[1] + (top - CARRY[1]) * i / RISE_STEPS
                arr.arrange(k)
                teach_group(T, P, "side", carton, [(name, local("side", fwd, right, z), 0.0, True)], arr=arr,
                            crouches=[(0.0, 0.0)], yaw=0.0, tilt=t, seed0=seed)
                done.append(name)
                seed = P[name]
            return
        except RuntimeError:
            for name in done:
                del P[name]


def teach_cell(scene, c: Cell, *, only=None) -> dict:
    """Every pose of the cycle (`P[name]`), the crouch each is taken in
    (`P["crouch"][name]`), the stand each carton is taken from
    (`P["pick_stand"][k]`) and the neck that reads each label (`P[f"look{k}"]`).
    `only` = the cartons to teach (the shared poses always)."""
    T = Teacher(scene)
    arr = Arranger(scene, c)
    P: dict = {}
    try:
        teach_shared(T, P, c, arr)
        for k in range(len(c.cartons)) if only is None else only:
            teach_carton(T, P, c, arr, k)
        P["crouch"] = dict(T.crouches)
        return P
    finally:
        arr.restore()
        T.restore()


def teach_parallel(c: Cell) -> dict:
    """`teach_cell` one carton a process (they do not depend on each other),
    merged: the shared poses from the first."""
    import pickle
    import subprocess
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        procs = []
        for k in range(len(c.cartons)):
            out = Path(tmp) / f"k{k}.pkl"
            procs.append((out, subprocess.Popen([sys.executable, str(Path(__file__).resolve()), "--teach", str(k), "--to", str(out)],
                                                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)))
        P: dict = {"crouch": {}, "pick_stand": {}}
        for out, proc in procs:
            _, err = proc.communicate()
            if proc.returncode != 0:
                raise RuntimeError(f"teaching {out.stem} failed: {err.strip().splitlines()[-1] if err.strip() else proc.returncode}")
            part = pickle.loads(out.read_bytes())
            for key, v in part.items():
                if key in ("crouch", "pick_stand", "meta", "tilt"):
                    P.setdefault(key, {}).update(v)
                else:
                    P.setdefault(key, v)
        return P


# ================================================================ routes
def arm_targets(names: list, q: list, joints=None) -> dict:
    """The joints a two-hand move drives (both arms and the waist), as `q` has them."""
    joints = joints or [f"{side}_{j}_joint" for side in ("left", "right") for j in ARM] + WAIST
    return {n: q[names.index(n)] for n in joints}


def arm_targets_into(names: list, q: list, src: list, joints: list) -> list:
    """`q` with `joints` as `src` has them."""
    out = list(q)
    for n in joints:
        out[names.index(n)] = src[names.index(n)]
    return out


def ramp_time(names: list, pace: dict, a: list, b: list, joints=None) -> float:
    """How long a straight joint move from `a` to `b` takes as a ramp (a
    cubic, rest to rest): its peak speed 1.5 d/T and its peak acceleration
    6 d/T^2 within each joint's pace and twice that."""
    use = set(joints) if joints else None
    t = 0.3
    for n, x, y in zip(names, a, b):
        if use is not None and n not in use:
            continue
        if any(k in n for k in ("hip", "knee", "ankle")):
            continue
        d = abs(y - x)
        if d > 1e-9:
            v = pace.get(n, 1.0)
            t = max(t, 1.5 * d / v, math.sqrt(3.0 * d / v))
    return math.ceil(t * 100.0) / 100.0


def crouch_time(a, b) -> float:
    """A crouch from (depth, lean) `a` to `b` at a careful pace: 0.45 m/s of
    depth and 0.8 rad/s of lean at the mean (the smoothstep peaks at 1.5x)."""
    return round(max(0.6, 1.5 * abs(b[0] - a[0]) / 0.45, 1.5 * abs(b[1] - a[1]) / 0.8), 2)


def motion_specs(c: Cell, P: dict) -> list:
    """Every arm move of the cycle as the poses it passes, with the cell as
    it stands then: the stand, the crouch, the carton k, whether the carton
    is held, `direct` = a contact move (onto the carton, off it) taken as it is."""
    out = []
    cr = P["crouch"]
    for k in range(len(c.cartons)):
        ck, cp = cr[f"pick{k}:pre"], cr[f"place{k}:above"]
        ws = P["pick_stand"][k]
        carry = f"carry{k}" if f"carry{k}" in P else None
        taken = f"pick{k}:out" if f"pick{k}:out" in P else f"pick{k}:lift"
        out += [
            {"name": f"k{k}:reach", "st": ws, "crouch": ck, "k": k, "held": False, "keys": ["ready", f"pick{k}:pre"]},
            {"name": f"k{k}:grip", "st": ws, "crouch": ck, "k": k, "held": False, "keys": [f"pick{k}:pre", f"pick{k}:grip"], "direct": True},
            {"name": f"k{k}:lift", "st": ws, "crouch": ck, "k": k, "held": True, "keys": [f"pick{k}:grip", f"pick{k}:lift"]},
            *([{"name": f"k{k}:pull", "st": ws, "crouch": ck, "k": k, "held": True, "keys": [f"pick{k}:lift", f"pick{k}:out"]}]
              if f"pick{k}:out" in P else []),
            *([{"name": f"k{k}:carry", "st": "clear", "crouch": (0.0, 0.0), "k": k, "held": True, "keys": [taken, carry]}]
              if carry else []),
            # standing: the torso turns to the cage with the arms as they set the
            # carton in, so it is over its slot — then the crouch lowers it in
            # (crouched with the carton before the chest, the thighs come up into it)
            {"name": f"k{k}:in", "st": "side", "crouch": (0.0, 0.0), "k": k, "held": True,
                 "keys": [carry or taken, *rise_keys(P, k), f"place{k}:above"]},
            # the last 2.5 cm onto its seat: a contact move, like the hands closing on it
            {"name": f"k{k}:set", "st": "side", "crouch": cp, "k": k, "held": True, "keys": [f"place{k}:above", f"place{k}:set"], "direct": True},
            {"name": f"k{k}:open", "st": "side", "crouch": cp, "k": k + 1, "held": False, "keys": [f"place{k}:set", f"place{k}:open"], "direct": True},
            {"name": f"k{k}:out", "st": "side", "crouch": cp, "k": k + 1, "held": False, "keys": [f"place{k}:open", f"place{k}:back"]},
            {"name": f"k{k}:ready", "st": "side", "crouch": (0.0, 0.0), "k": k + 1, "held": False, "keys": [f"place{k}:back", "ready"]},
        ]
    return out


class Router:
    """Each move a straight line in joint space where that line is clear of
    the cell as it stands at that point of the cycle (the body crouched as
    it is then, the carton in the hands or in its place), by way of a via
    pose where it is not (the midpoint, solved again by IK with the carton
    where the two ends have it, then the quarter points)."""

    def __init__(self, scene, T: Teacher, c: Cell, P: dict, arr: Arranger):
        self.scene, self.T, self.c, self.P, self.arr = scene, T, c, P, arr
        self.notes: list = []

    def grip_of(self, k: int):
        """Carton k in the left hand's frame, as the grip takes it."""
        if not hasattr(self, "_grips"):
            self._grips = {}
        if k not in self._grips:
            c, P = self.c, self.P
            self.T.stand(P["pick_stand"][k], P["crouch"][f"pick{k}:grip"], P[f"pick{k}:grip"])
            hand = self.scene.link_pose(HANDS["left"], robot=ROBOT)
            carton = (centre_of(c.seats[c.cartons[k]]), yaw_q(math.pi / 2))
            self._grips[k] = compose(invert(hand), carton)
        return self._grips[k]

    def setup(self, spec: dict):
        T, c, k = self.T, self.c, spec["k"]
        self.held = c.cartons[k] if spec["held"] else None
        rel = self.grip_of(k) if self.held else None
        self.arr.arrange(min(k, len(c.cartons)))
        T.stand(spec["st"], spec["crouch"], self.P[spec["keys"][0]])
        # what the move leaves alone: the legs as the crouch has them, the
        # head where the program turned it
        now, names = self.scene.joint_positions_of(ROBOT), T.names
        self.fixed = {i: now[i] for i, n in enumerate(names) if any(w in n for w in LEG_WORDS)}
        if spec.get("head"):
            self.fixed.update({names.index(n): self.P[spec["head"]][names.index(n)] for n in HEAD})
        if self.held:
            # the carton where the first pose holds it, in the hand as the grip took it
            hand = self.scene.link_pose(HANDS["left"], robot=ROBOT)
            self.scene.set_obstacle_pose(self.held, *compose(hand, rel))
            self.scene.attach(self.held, link=HANDS["left"], touch_links=list(HANDS.values()), robot=ROBOT)

    def teardown(self):
        if self.held:
            self.scene.detach(self.held)
        self.held = None

    def line(self, a: list, b: list, step: float = 0.02) -> list:
        n = max(2, math.ceil(math.dist(a, b) / step))
        for i in range(n + 1):
            q = [x + (y - x) * i / n for x, y in zip(a, b)]
            for j, v in self.fixed.items():
                q[j] = v
            self.scene.set_joint_positions(q, robot=ROBOT)
            pairs = [(p[1], o[1]) for p, o in self.scene.check_collisions()]
            if pairs:
                return pairs
        return []

    def route(self, spec: dict) -> list:
        keys = spec["keys"]
        if spec.get("direct"):
            return keys
        P = self.P
        out = [keys[0]]
        self.setup(spec)
        try:
            for a, b in zip(keys, keys[1:]):
                if not self.line(P[a], P[b]):
                    out.append(b)
                    continue
                vias = self.vias(spec, a, b)
                if vias is None:
                    raise RuntimeError(f"no clear way for `{spec['name']}` from {a} to {b}: {self.line(P[a], P[b])[:3]}")
                out += vias + [b]
                self.notes.append(f"{spec['name']}: {a} -> {' -> '.join(vias)} -> {b}")
        finally:
            self.teardown()
        return out

    def solve_via(self, name: str, spec: dict, centre, yaw: float, out: float, box, seed=None) -> str | None:
        """A via pose holding (or open round) a box at `centre`, at the
        motion's stand and crouch; kept in the poses as `name` (None where it
        does not solve or the held carton would touch something)."""
        P = self.P
        if name in P:
            return name if P[name] is not None else None
        global HELD
        keep, HELD = HELD, tuple(box)
        # the hands at the carton's tilt: held, only that; open round it, that first
        t = P["tilt"].get(spec["k"], 0.0)
        tilts = (t,) if self.held else (t, *[x for x in (0.0, 0.3, 0.6, 0.9, 1.2) if abs(x - t) > 1e-6])
        try:
            q = self.T.hold(name, spec["st"], centre, yaw, crouches=(spec["crouch"],), out=out,
                            seed=P[spec["keys"][0]] if seed is None else seed,
                            allow=(self.held,) if self.held else (), tilts=tilts)
        except RuntimeError:
            q = None
        finally:
            HELD = keep
            # the robot back where the motion's first pose has it (hold() moved it)
            self.T.stand(spec["st"], spec["crouch"], P[spec["keys"][0]])
        P[name] = q
        if q is not None:
            P["crouch"][name] = spec["crouch"]
        return name if q is not None else None

    def vias(self, spec: dict, a: str, b: str):
        """The fewest via poses that make every line clear: the box held (or
        the hands open) half way between the two ends, lifted, drawn back
        toward the body, or up (down) first and across after; or, without IK,
        the torso turned first or last. Singly, then in pairs. None where
        nothing clears."""
        P, meta = self.P, self.P.get("meta", {})
        ma, mb = meta.get(a), meta.get(b)
        if ma is None or mb is None:
            return None
        x0, y0, h = STATIONS[spec["st"]]
        sink = spec["crouch"][0]

        def world(name, m):
            # the box where the pose holds it with the body at this motion's
            # crouch (taught crouched otherwise, it is that much higher or lower)
            f, l, z = m["rel"]
            z += P["crouch"].get(name, (0.0, 0.0))[0] - sink
            return (x0 + f * math.cos(h) - l * math.sin(h), y0 + f * math.sin(h) + l * math.cos(h), z)

        ca, cb = world(a, ma), world(b, mb)
        ya, yb = ma["yaw"] + h, mb["yaw"] + h
        out = max(ma["out"], mb["out"])
        box = mb["box"] if mb["out"] >= ma["out"] else ma["box"]

        def at(c, yaw, dz=0.0, dback=0.0):
            # `dback` toward the body: against the way the hands reach
            return (c[0] - math.cos(yaw) * dback, c[1] - math.sin(yaw) * dback, c[2] + dz)

        mid = tuple((x + y) / 2 for x, y in zip(ca, cb))
        ym = (ya + yb) / 2
        tag = spec["name"]
        cands = []
        for label, c, yaw in (("mid", mid, ym), ("mid_up10", at(mid, ym, 0.10), ym), ("mid_up20", at(mid, ym, 0.20), ym),
                              ("a_up12", at(ca, ya, 0.12), ya), ("b_up12", at(cb, yb, 0.12), yb),
                              ("a_back12", at(ca, ya, 0.0, 0.12), ya), ("b_back12", at(cb, yb, 0.0, 0.12), yb),
                              ("mid_back12", at(mid, ym, 0.0, 0.12), ym), ("b_up25", at(cb, yb, 0.25), yb),
                              ("a_up25", at(ca, ya, 0.25), ya), ("a_at_bz", (ca[0], ca[1], cb[2]), ya),
                              ("b_at_az", (cb[0], cb[1], ca[2]), yb), ("a_back25", at(ca, ya, 0.0, 0.25), ya),
                              ("b_back25", at(cb, yb, 0.0, 0.25), yb)):
            cands.append((f"{tag}~{label}", c, yaw))
        solved = []
        # first, no IK: one end's arms with the other's waist (or only its yaw)
        # — the torso turns with the arms as they are, then the arms move; or
        # the arms first and the torso after
        names = self.T.names
        for label, arms, waist, joints in (("turn_first", a, b, WAIST), ("arms_first", b, a, WAIST),
                                           ("yaw_first", a, b, ["waist_yaw_joint"]), ("yaw_last", b, a, ["waist_yaw_joint"])):
            name = f"{tag}~{label}"
            if name not in P:
                P[name] = arm_targets_into(names, P[arms], P[waist], joints)
                P["crouch"][name] = spec["crouch"]
            solved.append(name)
        chain_ok = lambda chain: all(not self.line(P[x], P[y]) for x, y in zip(chain, chain[1:]))
        # the box along the straight line between the ends: two points, each
        # solved from the one before (the arms keep their posture)
        lin, prev = [], a
        for i, f in enumerate((1 / 3, 2 / 3)):
            c = tuple(x + (y - x) * f for x, y in zip(ca, cb))
            v = self.solve_via(f"{tag}~lin{i}", spec, c, ya + (yb - ya) * f, out, box, seed=P[prev])
            if v is None:
                break
            lin.append(v)
            prev = v
        if len(lin) == 2 and chain_ok([a, *lin, b]):
            return lin
        for name, c, yaw in cands:
            v = self.solve_via(name, spec, c, yaw, out, box)
            if v is not None:
                solved.append(v)
        for v in solved:
            if chain_ok([a, v, b]):
                return [v]
        for v in solved:
            for w in solved:
                if v != w and chain_ok([a, v, w, b]):
                    return [v, w]
        return None


def walk_contacts(scene, T: Teacher, c: Cell, P: dict, arr: Arranger, router: Router) -> list:
    """The walks, stepped every 5 cm along the line: each carton carried
    from its stand at the window to the cage's side with the arms as they
    took it, and the ready arms back to the next stand. What anything meets."""
    out = []
    side, clear = STATIONS["side"][0], STATIONS["clear"][0]
    for k in range(len(c.cartons)):
        taken = f"pick{k}:out" if f"pick{k}:out" in P else f"pick{k}:lift"
        carried = f"carry{k}" if f"carry{k}" in P else taken
        nxt = STATIONS[P["pick_stand"][k + 1]][0] if k + 1 < len(c.cartons) else side
        for x0, x1, held, q, kk in ((STATIONS[P["pick_stand"][k]][0], clear, True, P[taken], k),
                                    (clear, side, True, P[carried], k), (side, nxt, False, P["ready"], k + 1)):
            n = max(1, round(abs(x1 - x0) / 0.05))
            arr.arrange(min(kk, len(c.cartons)))
            carton, rel = (c.cartons[k], router.grip_of(k)) if held else (None, None)
            for i in range(n + 1):
                STATIONS["_walk"] = (x0 + (x1 - x0) * i / n, LINE_Y + SWAY, 0.0)   # swayed toward the cage
                T.stand("_walk", (0.0, 0.0), q)
                if carton:
                    scene.set_obstacle_pose(carton, *compose(scene.link_pose(HANDS["left"], robot=ROBOT), rel))
                    scene.attach(carton, link=HANDS["left"], touch_links=list(HANDS.values()), robot=ROBOT)
                try:
                    hits = [(a[1], b[1]) for a, b in scene.check_collisions()]
                finally:
                    if carton:
                        scene.detach(carton)
                if hits:
                    out.append((f"#{k + 1} {'carried' if held else 'back'} at x {STATIONS['_walk'][0]:+.2f} ({'ready' if not held else 'as taken' if q is P[taken] else 'at the chest'})", hits[:2]))
                    break
    STATIONS.pop("_walk", None)
    return out


def build_routes(scene, c: Cell, P: dict) -> tuple:
    T = Teacher(scene)
    arr = Arranger(scene, c)
    router = Router(scene, T, c, P, arr)
    try:
        walks = walk_contacts(scene, T, c, P, arr, router)
        if walks:
            raise RuntimeError(f"the walks meet the cell: {walks}")
        routes = {spec["name"]: [*router.route(spec)] for spec in motion_specs(c, P)}
    finally:
        arr.restore()
        T.restore()
    return routes, router.notes


# ================================================================ the programs
def allow_touches(scene, c: Cell) -> None:
    """The contacts the work makes, declared once: each carton sits on what
    it is set on — on the rack and in the cage — only while seated (over its
    seat within 5 mm and level), so one swung onto its support still counts."""
    def support(carton, seat, below_name):
        scene.allow_object_obstacle_contact(carton, below_name, window=(seat, (0.0, 0.0, 1.0), 0.005, 0.02))

    for i, carton in enumerate(c.cartons):
        rack = "rack1" if c.rack_of[carton] == "agv1" else "rack2"
        seat = c.seats[carton]
        base = S.RACK_SEAT + RACK_RISER
        layer = round((seat[2] - base) / S.CARTON[2])
        same = [x for x in c.seats if c.rack_of[x] == c.rack_of[carton]]
        below = f"{rack}/riser" if layer == 0 else next(x for x in same if round((c.seats[x][2] - base) / S.CARTON[2]) == layer - 1)
        if c.rack_of[carton] == "agv1":
            support(carton, seat, below)
        else:
            # rack 2's cartons ride their rack in from the line; seated where they stand now
            (x, y, z), _q = scene.obstacle_pose(carton)
            support(carton, (x, y, z - S.CARTON[2] / 2), below)
            support(carton, seat, below)
        slot = c.slots[carton]
        clayer = round((slot[2] - CAGE_DECK) / S.CARTON[2])
        under = "centre/cage/riser" if clayer == 0 else next(
            x for x in c.cartons if abs(c.slots[x][0] - slot[0]) < 1e-6 and round((c.slots[x][2] - CAGE_DECK) / S.CARTON[2]) == clayer - 1)
        support(carton, slot, under)


PROGRAMS = ["h2", "agv1", "agv2"]


def build_programs(scene, c: Cell, P: dict, routes: dict) -> None:
    Sq = bt.seq
    names, pace = scene.robot_of(ROBOT).joint_names, joint_pace()
    for sig, v in (("rack1/present", True), ("rack1/empty", False), ("rack1/gone", False), ("rack2/present", False)):
        scene.define_signal(sig, v)
    sq = scene.sequence("h2")
    done = Sq.done()
    cr = P["crouch"]

    def ramp(a, b, *, check=True, joints=None, s=0.0):
        js = joints or [f"{side}_{j}_joint" for side in ("left", "right") for j in ARM] + WAIST
        return Sq.ramp(arm_targets(names, P[b], js), max(s, ramp_time(names, pace, P[a], P[b], js)), robot=ROBOT, check=check)

    def head(a, b):
        return Sq.ramp(arm_targets(names, P[b], HEAD), max(0.4, ramp_time(names, pace, P[a], P[b], HEAD)), robot=ROBOT)

    def move(label, name, *, check=True, last=None, extra=()):
        keys = routes[name]
        legs = list(zip(keys, keys[1:]))
        for i, (a, b) in enumerate(legs):
            acts = [ramp(a, b, check=check)] + (list(extra) if i == 0 else [])
            sq.step(label if len(legs) == 1 else f"{label} {i + 1}/{len(legs)}", actions=acts,
                    transition=last if (last is not None and i == len(legs) - 1) else done)

    def crouch(label, frm, to, extra=()):
        if frm == to:
            return
        sq.step(label, actions=[Sq.crouch(ROBOT, to[0], lean=to[1], duration=crouch_time(frm, to)), *extra], transition=done)

    P["neutral"] = list(P["ready"])
    level = (0.0, 0.0)
    sq.step("arms ready", actions=[ramp("home", "ready")], transition=done)
    if P["pick_stand"][0] != START:
        sq.step("to the window", actions=[Sq.goto(LEGS, P["pick_stand"][0])], transition=Sq.device_done(LEGS))
    for k, carton in enumerate(c.cartons):
        if k == 3:
            sq.step("wait rack 2", transition=Sq.signal("rack2/present"))
        ck, cp = cr[f"pick{k}:pre"], cr[f"place{k}:above"]
        tag = f"#{k + 1}"
        # at the window: crouch to the carton while the head turns to its label
        if ck != level:
            crouch(f"{tag} crouch", level, ck, extra=[head("neutral", f"look{k}")])
        else:
            sq.step(f"{tag} look", actions=[head("neutral", f"look{k}")], transition=done)
        sq.step(f"{tag} QR read", transition=Sq.all_of(Sq.signal(c.qr[carton]), Sq.elapsed(0.4)))
        # read: the head up before the hands go in (bent to the label, it is
        # where the carton comes up and back)
        sq.step(f"{tag} head up", actions=[head(f"look{k}", "neutral")], transition=done)
        move(f"{tag} reach", f"k{k}:reach")
        move(f"{tag} hands on", f"k{k}:grip", check=False)
        sq.step(f"{tag} take", actions=[Sq.attach(carton, group="left_w", robot=ROBOT, touch_links=list(HANDS.values()))],
                transition=Sq.immediately())
        move(f"{tag} lift", f"k{k}:lift")
        if f"k{k}:pull" in routes:
            move(f"{tag} draw back", f"k{k}:pull")
        crouch(f"{tag} stand up", ck, (0.0, 0.0))
        # backing away draws the carton the rest of the way out of the rack;
        # clear of it, the carton comes in to the chest (held out as it was
        # taken, the elbows would meet the cage's end panel walking past)
        sq.step(f"{tag} back out", actions=[Sq.goto(LEGS, "clear")], transition=Sq.device_done(LEGS))
        if k == 2:
            # the first rack is empty and the robot clear of the window: the AGVs
            # swap racks while it loads this one
            sq.step("rack 1 empty", actions=[Sq.set_signal("rack1/empty", True)], transition=Sq.immediately())
        if f"k{k}:carry" in routes:
            move(f"{tag} carry", f"k{k}:carry")
        sq.step(f"{tag} to the cage", actions=[Sq.goto(LEGS, "side")], transition=Sq.device_done(LEGS))
        move(f"{tag} into the cage" if cp == level else f"{tag} turn to the cage", f"k{k}:in")
        crouch(f"{tag} crouch ", (0.0, 0.0), cp)
        move(f"{tag} set down", f"k{k}:set", check=False)
        sq.step(f"{tag} let go", actions=[Sq.detach(carton)], transition=Sq.immediately())
        move(f"{tag} hands off", f"k{k}:open", check=False)
        move(f"{tag} hands out", f"k{k}:out")
        crouch(f"{tag} stand up ", cp, (0.0, 0.0))
        move(f"{tag} ready", f"k{k}:ready")
        if k < len(c.cartons) - 1:
            sq.step(f"{tag} to the window", actions=[Sq.goto(LEGS, P["pick_stand"][k + 1])], transition=Sq.device_done(LEGS))
        level = (0.0, 0.0)
    sq.step("done", transition=Sq.elapsed(1.0))

    # the AGVs: the first presents its rack and, emptied, takes it away; the
    # second brings the next rack in once the first has cleared the lane
    a1 = scene.sequence("agv1")
    a1.step("presented", transition=Sq.signal("rack1/empty"))
    a1.step("back out", actions=[Sq.goto("agv1", "away")], transition=Sq.device_done("agv1"))
    a1.step("gone", actions=[Sq.set_signal("rack1/present", False), Sq.set_signal("rack1/gone", True)])
    a2 = scene.sequence("agv2")
    a2.step("waiting", transition=Sq.signal("rack1/gone"))
    a2.step("to the window", actions=[Sq.goto("agv2", "window")], transition=Sq.device_done("agv2"))
    a2.step("presented", actions=[Sq.set_signal("rack2/present", True)])
    scene.add_io_node("h2", kind="robot_controller", robots=[ROBOT], programs=["h2"], label="H2 onboard controller")
    scene.add_io_node("gtp", kind="plc", programs=["agv1", "agv2"], label="GTP system (rack dispatch)")


TEACH_VERSION = 27                   # bump when what teaching records changes


def taught(scene, cell) -> dict:
    """`teach_cell`, kept on disk keyed by everything the poses depend on
    (teaching takes minutes; a bake should not wait for it twice)."""
    import hashlib
    import pickle
    key = repr((TEACH_VERSION, STATIONS, HOLD_YAW, CARRY, CARRY_RIGHT, SWAY, GRIP_TILTS, LIFT, PULL, SLOT_UP, WITHDRAW, WITHDRAW_HIGH, CROUCHES, GRIP_DEPTH, HELD, READY_BOX, APPROACH,
                sorted(cell.seats.items()), sorted(cell.slots.items()), cage_pose(), S.CAGE, RISER, sorted(scene.obstacle_names),
                str(paced_urdf())))
    path = Path.home() / ".cache" / "botrail" / "demo" / f"gtp_poses_{hashlib.sha1(key.encode()).hexdigest()[:12]}.pkl"
    if path.exists():
        return pickle.loads(path.read_bytes())
    poses = teach_parallel(cell)
    path.write_bytes(pickle.dumps(poses))
    return poses


def build(*, ceiling: bool = True):
    scene, cell = build_scene(ceiling=ceiling)
    allow_touches(scene, cell)
    poses = taught(scene, cell)
    routes, notes = build_routes(scene, cell, poses)
    build_programs(scene, cell, poses, routes)
    return scene, {"cell": cell, "poses": poses, "routes": routes, "notes": notes}


def bake(*, ceiling: bool = True, max_duration: float = 400.0):
    scene, info = build(ceiling=ceiling)
    tl = scene.simulate_sequences(PROGRAMS, max_duration=max_duration)
    info["contacts"] = audit(scene, tl, info["cell"])
    return scene, tl, info


def audit(scene, tl, c: Cell, dt: float = 0.05) -> list:
    """The baked run replayed against the cell every `dt`: the robot where
    the timeline has it, the cartons and the racks where they were, and every
    contact the robot makes that is not a hand on a carton."""
    movers = list(c.cartons) + list(c.rack) + list(c.next_rack)
    was = [(o, scene.obstacle_pose(o)) for o in movers]
    base, q = scene.robot_base_pose_of(ROBOT), list(scene.joint_positions_of(ROBOT))
    spans = [(n, a, b) for n, a, b in tl.step_spans if n.startswith(f"{ROBOT}/")]
    end = max(b for _, _, b in spans)
    hands = set(HANDS.values())
    met: dict = {}
    try:
        for i in range(int(end / dt) + 1):
            t = i * dt
            pose = tl.base_pose(t, ROBOT)
            if pose:
                scene.set_robot_base_pose(*pose, robot=ROBOT)
            scene.set_joint_positions(tl.sample(t, ROBOT), robot=ROBOT)
            for o in movers:
                op = tl.object_pose(o, t)
                if op:
                    scene.set_obstacle_pose(o, *op)
            for a, b in scene.check_collisions():
                pair = (a[1], b[1])
                if b[1] in c.cartons and a[1] in hands or a[1] in c.cartons and b[1] in hands:
                    continue
                if pair not in met:
                    met[pair] = (round(t, 2), next((n for n, x, y in spans if x <= t < y), "?"), pair)
    finally:
        for o, pose in was:
            scene.set_obstacle_pose(o, *pose)
        scene.set_robot_base_pose(*base, robot=ROBOT)
        scene.set_joint_positions(q, robot=ROBOT)
    return sorted(met.values())


# ================================================================ the report
def report(scene, tl, info) -> dict:
    """What the run shows, printed; the numbers returned."""
    c, P = info["cell"], info["poses"]
    spans = [(n.split("/", 1)[1], a, b) for n, a, b in tl.step_spans if n.startswith(f"{ROBOT}/")]
    first = lambda pre: min(a for n, a, b in spans if n.startswith(pre))
    lambda pre: max(b for n, a, b in spans if n.startswith(pre))
    total = lambda pre: sum(b - a for n, a, b in spans if n.startswith(pre))
    end = max(b for _, _, b in spans)
    n = len(c.cartons)
    print(f"GTP station, centre window: one H2 reads, takes and loads {n} cartons from two racks into the roll cage "
          f"in {end:.1f} s")
    per = []
    for k in range(n):
        t0 = first(f"#{k + 1} ")
        t1 = first(f"#{k + 2} ") if k + 1 < n else end
        per.append(t1 - t0)
    swap = total("wait rack 2")
    steady = sorted(per)[len(per) // 2]
    print(f"  per carton: {', '.join(f'{v:.1f}' for v in per)} s (median {steady:.1f} s -> {3600 / steady:.0f} cartons an hour "
          f"while racks keep coming); the rack swap held the robot {swap:.1f} s")
    walk = sum(b - a for n_, a, b in spans if "to the" in n_ or "back out" in n_)
    crouching = sum(b - a for n_, a, b in spans if "crouch" in n_ or "stand up" in n_)
    qr = sum(b - a for n_, a, b in spans if "QR read" in n_ or "look" in n_)
    arms = end - walk - crouching - qr - swap
    print(f"  where the time goes: walking {walk:.0f} s, crouching and standing {crouching:.0f} s, "
          f"reading labels {qr:.0f} s, the arms {arms:.0f} s, waiting for the rack {swap:.0f} s")
    deep = [(k, P["crouch"][f"pick{k}:pre"], P["crouch"][f"place{k}:above"]) for k in range(n)]
    print("  crouches (depth m / lean rad): " + "; ".join(
        f"#{k + 1} pick {a[0]:.2f}/{a[1]:.1f} set {b[0]:.2f}/{b[1]:.1f}" for k, a, b in deep))
    contacts = info.get("contacts", [])
    print("  the baked run replayed every 50 ms against the cell: "
          + ("no contact" if not contacts else f"{len(contacts)} contacts — {contacts[:3]}"))
    legs = tl.locomotion(ROBOT)
    print(f"  walked {sum(d for *_, d in legs):.1f} m; the heaviest thing carried: a carton, {CARTON_KG:.1f} kg (payload 7 kg)")
    vias = sum(1 for keys in info["routes"].values() for k in keys if "~" in k)
    print(f"  {len(info['routes'])} arm moves as {sum(len(keys) - 1 for keys in info['routes'].values())} straight joint moves "
          f"({vias} via poses)")
    return {"cycle_s": end, "per_carton_s": per, "contacts": len(contacts)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("out", nargs="?", default=None, help="the baked run as USD (default: gtp_station.usdc here)")
    parser.add_argument("--studio", action="store_true", help="open the studio on the run after the bake")
    parser.add_argument("--no-ceiling", action="store_true", help="leave out the roof (views from above)")
    parser.add_argument("--teach", type=int, default=None, help=argparse.SUPPRESS)   # one carton, for teach_parallel
    parser.add_argument("--to", default=None, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.teach is not None:
        import pickle
        scene, cell = build_scene(ceiling=not args.no_ceiling)
        allow_touches(scene, cell)
        Path(args.to).write_bytes(pickle.dumps(teach_cell(scene, cell, only=[args.teach])))
        return
    t0 = time.time()
    scene, tl, info = bake(ceiling=not args.no_ceiling)
    print(f"(taught, routed and baked in {time.time() - t0:.0f} s)")
    report(scene, tl, info)
    out = args.out or str(HERE / "gtp_station.usdc")
    tl.export_usd(out, fps=15)
    print(f"wrote {out}")
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()

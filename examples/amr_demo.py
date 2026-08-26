"""An AMR assembled from the catalog: a carrier, an arm, a gripper.

The machine in this cell is not modelled — it is *specified*. Three
catalog packages stack into one mobile manipulator, and everything the
cell needs to put them together is read out of those packages rather
than typed in: where the arm bolts on (`frames.flange_frame`), how big
the body that has to fit the aisle is (its own collision geometry), how
fast it may run and what it may carry (`specs`). Nothing below knows it
is a Robotnik.

That is the point of the exercise. `--carrier` swaps the base: the arm
re-mounts itself at the new deck height, the body driving the aisle
becomes that machine's body, the machine stops where *its* arm is level
with the work, and the cycle re-bakes. `--compare` does it across the
mobile bases the catalog ships and prints the table a buyer wants —
cycle time, and the reason the ones that do not fit do not fit.

The cycle is what an AMR is bought for: fetch a part from a bench in the
aisle, carry it on the machine's own deck to the machining bay, hand it
to the outfeed conveyor.

  * **把持 / 載置** — pick the part off the bench and set it on the deck.
    From the moment it is released inside the tray zone it is cargo:
    there is no load action, because loading is the conveyor's zone rule
    on a frame that moves.
  * **走行** — the departure permit, asked of the machine itself: its own
    load sensor says the part is aboard and its own envelope says nothing
    is hanging over the side. Then dispatch, and fold the arm away
    *while* travelling.
  * **払出** — nose into the bay as deep as this body allows, pick the
    part back off the deck with the *same taught pose*, and place it on
    the belt.

Three rules of a moving base fall out of it:

  * **A planned motion cannot start while the machine is driving.** Plans
    are baked in world coordinates when they start, so a base that moves
    underneath one invalidates every waypoint. The rollout rejects it by
    name (`--drive-and-plan`).
  * **A ramp can**, and that is what the stow is: the bake shows the ramp
    running inside the drive it shares a step with, which is what "the
    fold costs no cycle time" means. Nothing checks a ramp's path,
    though — that is the other half of the same property — so the cycle
    checks the fold itself before writing it.
  * **A pose is taught in the machine's frame, not the world's.** The
    deck is in the same place at every station, so one taught pose seats
    the part and picks it up again; the bench and the belt are taught
    from where the machine will be standing when it serves them.

Run with:  python examples/amr_demo.py [out.usda] [--carrier NAME]
                                       [--compare] [--drive-and-plan]

(Name the output `cell_amr.usdc` for a binary stage: the same recording
at a quarter of the size, since a catalog arm brings a lot of mesh with
it. `play_record.py` reads either.)
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import yaml  # noqa: E402  (catalog manifests; `from_catalog` needs it too)

import botrail as bt  # noqa: E402

# ------------------------------------------------------------- the machine
# Three catalog ids. `from_catalog` takes a short name as long as it is
# unambiguous, and what a project records is the id and dataset revision
# it resolved to — so naming them this way costs nothing in replayability.
CARRIER = "rb-kairos"  # Robotnik RB-KAIROS: a base sold to carry exactly this
ARM = "ur16e"  # 900 mm of reach: a low deck has to cross the aisle to work
COUPLING = "gripper-coupling"
GRIPPER = "2f-85"

# What `--compare` re-bakes the cell on. Any `vehicle.amr` package with a
# mount frame is a candidate; the cell asks nothing else of a carrier.
CANDIDATES = ["rb-kairos", "rb-theron", "mpo-700", "ridgeback", "rb-summit",
              "rb-robout"]

# ---------------------------------------------------------------- the cell
# An aisle with racking down one side and the served side down the other,
# and a bay in the racking where a machine tool stands. Ordinary warehouse
# numbers; what they decide is which carriers fit.
LANE_Y = 0.0
RACK_FACE = 0.75  # the racking's near face, north of the lane
RACK_DEPTH, RACK_H = 0.50, 2.00
RACK_RUN = (-3.10, 2.60)
BAY = (0.15, 1.55)  # the gap in the racking, in x
SERVED_FACE = -0.65  # bench, machine tool and pallets all line up here,
# which with the racking opposite makes a 1.40 m aisle: wide enough to
# drive any of these machines down, and not wide enough to turn all of them

CORNER_X = 0.70  # where the machine turns to face the bay
CNC_FACE = 1.60  # the machine tool at the back of the bay
NOSE = 0.16  # how close the machine's front stops to it
BENCH_TOP = 0.75
BENCH = (1.30, 0.55)
PART_X = -2.45  # where the part waits on the bench
BELT_X = 1.34  # the outfeed belt, up the bay's east side
BELT_RUN, BELT_W, BELT_TOP = (0.35, 1.45), 0.30, 0.62

PART = 0.06  # a 60 mm part, and the 2F-85's stroke around it
PART_MASS = 2.4
OPEN, SHUT = 0.0, 0.33  # finger joint: 92 mm and 60 mm across the pads
GRIP = 0.05  # how far above its seat the tool grips the part
HOVER = 0.14
SEAT_AHEAD = 0.40  # where on the deck the part rides, ahead of the arm
SEAT_GAP = 0.006  # a *carried* part is set down proud of the surface: it
# is checked as part of the robot, so resting on something reads as touching it
TOTE = "part"

# The arm at rest, and the arm folded over its own deck — where it rides
# between stations. The fold is relative to the deck it is bolted to, so
# one set of angles serves every carrier; whether it *clears* that deck,
# and whether it can get there without sweeping through it, is the
# carrier's business, and `build_cycle` checks both. The last wrist is
# filled in from the deck pose rather than fixed: parking is no reason to
# unwind a wrist, and unwinding one costs seconds.
READY = [0.0, -1.75, 1.75, -1.55, -1.57, 0.0, OPEN]
STOWED = [0.20, -0.88, 1.96, -1.18, -1.57, None, OPEN]

# The links a grip legitimately rests on.
PADS = [f"{side}_inner_{part}"
        for side in ("left", "right")
        for part in ("finger", "finger_pad", "knuckle")]

# Assumed, not published: no carrier's data sheet gives an acceleration,
# and botrail's vehicles run at constant speed. The bake stays honest
# while the ramp-up distance is a small share of the leg, so the cruise
# speed is derated to that share instead of taken from the spec sheet.
ACCEL, RAMP_SHARE = 1.0, 0.15
TURN = math.radians(45.0)  # in-cell pivot rate — assumed, as the AGV cell's

# Linear-RGB colours (the USD convention — never raw sRGB bytes).
SHELL = (0.76, 0.77, 0.79)
BLACK = (0.010, 0.010, 0.011)

UR_BASE_R = 0.075  # UR base flange radius: how far in from an edge it bolts
PLATE = 0.015  # the adapter plate between deck and arm
ARM_CLEAR = 0.22  # the arm's own room on the deck, ahead of which the tray starts
DECK_EDGE = 0.04  # how far in from the deck's edges cargo is allowed


def manifest(package: str) -> dict:
    """The catalog manifest of `package`, whole."""
    return yaml.safe_load((Path(bt.catalog_package(package)) / "manifest.yaml").read_text())


class Carrier:
    """A mobile base, measured from its own catalog package.

    A cell that means to put an arm on a machine it did not design has
    four questions for it — where does the arm bolt on, how much deck is
    left for the load, how much room does the body need to turn, how fast
    may it go — and a `vehicle.amr` package answers all four. Asking once
    and passing the answers around is what keeps `--carrier` an argument
    rather than an edit.
    """

    def __init__(self, package: str):
        spec = manifest(package)
        self.package, self.id = package, spec["id"]
        self.product, self.maker = spec["name"], spec["manufacturer"]["name"]
        self.specs = spec["specs"]
        self.model = bt.Robot.from_catalog(package, format="usd")
        if self.model.flange_link is None:
            raise ValueError(f"{self.id} declares no mount frame to bolt an arm to")
        self.pieces = sorted((Path(bt.catalog_package(package)) / "collision").glob("*.stl"))

        # FK in a scene of its own: the mount frame, and every collision
        # piece in the machine's frame. Those pieces are the body — the
        # thing that has to clear the aisle *is* the thing you see.
        probe = bt.Scene(self.model)
        self.deck = probe.link_pose(self.link(self.model.flange_link))[0][2]
        self.poses, self.bounds = {}, {}
        for stl in self.pieces:
            pose = probe.link_pose(self.link(stl.stem))
            name = probe.add_mesh(stl.stem, stl, pose[0], quaternion=pose[1])
            self.poses[stl.stem], self.bounds[stl.stem] = pose, probe.obstacle_bounds(name)
        lo, hi = zip(*(self.bounds[stl.stem] for stl in self.pieces))
        self.lo = tuple(min(v[i] for v in lo) for i in range(3))
        self.hi = tuple(max(v[i] for v in hi) for i in range(3))

        # The surface things stand on is the top of the chassis — the
        # biggest piece — which sits a few millimetres above the declared
        # mount frame on most machines. Seat a part on the *drawn* deck
        # instead and it reads as a collision, because collision runs on
        # the convex decomposition and the hull of a dished top fills it.
        chassis = max(self.pieces, key=lambda stl: self.volume(self.bounds[stl.stem]))
        self.surface = max(self.deck, self.bounds[chassis.stem][1][2])
        self.proud = self.surface - self.deck
        # A deck with its own structure standing on it is not a deck this
        # cell can use: an arm bolted to the frame would sit *inside* the
        # chassis around it. That wants a riser and a bracket drawing, and
        # a bracket drawing is a different exercise from this one.
        if self.proud > PLATE:
            raise ValueError(
                f"{self.product}: its chassis stands {self.proud * 1e3:.0f} mm above the "
                f"mount frame, so a {PLATE * 1e3:.0f} mm plate leaves the arm inside it "
                f"— this one needs a riser"
            )

        # Where the arm bolts on: one base radius in from the deck's
        # rear corner on the served side, which leaves the whole front of
        # the deck as tray and puts both work sides within a side reach.
        inset = UR_BASE_R + 0.06
        self.mount = (self.lo[0] + inset, self.lo[1] + inset, self.surface + PLATE)
        front, back = self.hi[0] - DECK_EDGE, self.mount[0] + ARM_CLEAR
        self.tray = ((back + front) / 2, (self.lo[1] + self.hi[1]) / 2)
        self.tray_size = (max(front - back, 0.0), self.width - 2 * DECK_EDGE)
        # Where on the tray the part is set down: an arm's length in front
        # of the arm, not the middle of the deck. On a 1.8 m machine the
        # middle of the deck is nowhere near the arm that has to reach it.
        self.seat = (min(self.mount[0] + SEAT_AHEAD, self.tray[0] + self.tray_size[0] / 2 - PART),
                     self.tray[1])

    def link(self, name: str) -> str:
        """USD link names are prim paths; a manifest names segments."""
        if name in self.model.link_names:
            return name
        return next(n for n in self.model.link_names if n.rsplit("/", 1)[-1] == name)

    @staticmethod
    def volume(bounds) -> float:
        lo, hi = bounds
        return (hi[0] - lo[0]) * (hi[1] - lo[1]) * (hi[2] - lo[2])

    @property
    def length(self) -> float:
        return self.hi[0] - self.lo[0]

    @property
    def width(self) -> float:
        return self.hi[1] - self.lo[1]

    @property
    def swing(self) -> float:
        """What a pivot sweeps: the body's half-diagonal. It is this, not
        the width, that decides whether a corner works."""
        return math.hypot(self.length, self.width) / 2

    def cruise(self, leg: float) -> float:
        """The fastest constant speed whose acceleration ramp stays a
        small share of `leg` — capped, of course, by the data sheet."""
        return min(float(self.specs.get("max_speed_mps", 1.0)),
                   math.sqrt(2 * ACCEL * RAMP_SHARE * leg))

    # -- where this particular machine has to stand -----------------------
    @property
    def infeed(self) -> tuple:
        """Not the bench's x but the bench's x *offset by the mount*: the
        machine stops where its own arm is level with the part."""
        return (PART_X - self.mount[0], LANE_Y)

    @property
    def outfeed(self) -> tuple:
        """As deep into the bay as this body allows. A longer machine
        simply stops further out — and its arm reaches further in."""
        return (CORNER_X, CNC_FACE - NOSE - self.hi[0])

    def add_body(self, scene: bt.Scene, prefix: str) -> None:
        """Draws the body at the infeed station, heading +x, as the
        obstacles the vehicle carries."""
        x, y = self.infeed
        for stl in self.pieces:
            (px, py, pz), q = self.poses[stl.stem]
            sensor = any(word in stl.stem for word in ("laser", "lidar"))
            name = scene.add_mesh(f"{prefix}/{stl.stem}", stl, (x + px, y + py, pz),
                                  quaternion=q, color=BLACK if sensor else SHELL)
            scene.set_obstacle_material(name, metalness=0.2 if sensor else 0.35,
                                        roughness=0.6 if sensor else 0.45)


# ------------------------------------------------------------------ the cell
def build_scene(carrier: str = CARRIER, *, holonomic: bool = False) -> bt.Scene:
    """The aisle, the bay, and the machine standing at the bench.

    Shared with `play_record.py`, which rebuilds the cell a recording was
    baked from."""
    machine = Carrier(carrier)
    arm = (bt.Robot.from_catalog(ARM)
           .attach_tool(bt.Robot.from_catalog(COUPLING), prefix="cpl_")
           .attach_tool(bt.Robot.from_catalog(GRIPPER)))
    scene = bt.Scene(arm, name="ur")
    scene.set_joint_positions(READY)

    # -- the aisle: racking one side, the served side the other ----------
    # Racking is a table with shelves in it as far as this cell is
    # concerned: what it contributes is a face at a distance, and the
    # face is what the machine has to clear.
    rack_y = RACK_FACE + RACK_DEPTH / 2
    for i, (x0, x1) in enumerate(((RACK_RUN[0], BAY[0]), (BAY[1], RACK_RUN[1]))):
        mid = (x0 + x1) / 2
        bt.parts.table(scene, f"rack{i}", size=(x1 - x0, RACK_DEPTH, RACK_H),
                       position=(mid, rack_y), top_thickness=0.05, leg=0.07,
                       model="SR-2000", manufacturer="Generic", color=(0.32, 0.35, 0.40))
        for level in (0.55, 1.10, 1.55):
            scene.add_box(f"rack{i}/shelf{level:.2f}", (x1 - x0, RACK_DEPTH, 0.05),
                          (mid, rack_y, level), color=(0.32, 0.35, 0.40))
        # Stock on the shelves, so the aisle looks like the aisle it is.
        for j, (dx, level, size) in enumerate(((-0.9, 0.60, 0.34), (0.5, 1.15, 0.30),
                                               (1.1, 0.60, 0.26), (-0.3, 1.60, 0.30))):
            x = mid + dx
            if not x0 + 0.3 < x < x1 - 0.3:
                continue
            scene.add_box(f"rack{i}/crate{j}", (size, size, size),
                          (x, rack_y, level + size / 2), color=(0.45, 0.33, 0.20))
    bt.parts.table(scene, "bench", size=(*BENCH, BENCH_TOP),
                   position=(PART_X, SERVED_FACE - BENCH[1] / 2),
                   model="HFS8-1300", manufacturer="Generic", mass_kg=34)
    bt.parts.table(scene, "lathe", size=(1.10, 0.70, 1.35),
                   position=(-0.60, SERVED_FACE - 0.35), top_thickness=0.10, leg=0.10,
                   model="TL-25", manufacturer="Generic", color=(0.22, 0.26, 0.32))
    # The staging pallets opposite the bay. They are what the pivot has to
    # miss: a turn there sweeps the body's half-diagonal, not its width.
    for i, x in enumerate((0.35, 1.35)):
        bt.parts.pallet(scene, f"pallet{i}", position=(x, SERVED_FACE - 0.40))

    # -- the bay: the machine tool, and the belt that takes parts away ---
    bt.parts.table(scene, "cnc", size=(1.20, 0.90, 1.70),
                   position=((BAY[0] + BAY[1]) / 2, CNC_FACE + 0.45),
                   top_thickness=0.12, leg=0.12, model="VMC-500",
                   manufacturer="Generic", color=(0.20, 0.24, 0.30))
    bt.parts.conveyor(scene, "outfeed", length=BELT_RUN[1] - BELT_RUN[0], width=BELT_W,
                      position=(BELT_X, sum(BELT_RUN) / 2, BELT_TOP),
                      direction=(0.0, -1.0), speed=0.30,
                      model="GVL-1100", manufacturer="Generic")

    # -- the machine: body, adapter plate, arm ---------------------------
    machine.add_body(scene, "amr")
    x, y = machine.infeed
    scene.add_box("amr/plate", (0.26, 0.26, PLATE),
                  (x + machine.mount[0], y + machine.mount[1], machine.mount[2] - PLATE / 2),
                  color=(0.20, 0.22, 0.26))
    # The plate is what the arm stands on, so it is scenery to the arm —
    # the same call a pedestal gets in a fixed cell. It still rides:
    # riding and colliding are different questions.
    scene.set_obstacle_enabled("amr/plate", False)

    legs = (CORNER_X - machine.infeed[0], machine.outfeed[1] - LANE_Y)
    # A mecanum variant translates the same path without ever pivoting —
    # it docks facing what it faced when parked, and the corner costs
    # nothing but its length.
    drive = dict(drive="holonomic") if holonomic else dict(allow_reverse=True)
    scene.add_vehicle(
        "amr",
        body=["amr"],
        path=[machine.infeed, (CORNER_X, LANE_Y), machine.outfeed],
        stations={"infeed": 0, "outfeed": 2},
        speed=machine.cruise(max(legs)),
        turn_speed=TURN,
        start="infeed",
        tray_position=(*machine.tray, machine.surface + 0.06),
        tray_size=(*machine.tray_size, 0.12),
        **drive,
    )
    # From here the arm's base is not a scene constant: it is the deck.
    scene.mount_robot("amr", offset_position=machine.mount)
    # The vehicle is a product, so it goes on the bill of materials as one.
    # The arm and the gripper carry their own identity already — they were
    # loaded from the catalog — and this is the same fact for the machine
    # they ride: one source, and every document downstream agrees.
    scene.set_part("amr", kind="device", catalog=machine.id,
                   manufacturer=machine.maker, model=machine.product,
                   category="vehicle.amr",
                   payload_kg=machine.specs["payload_kg"],
                   max_speed_mps=machine.specs["max_speed_mps"])

    # -- what the cell handles, and what watches it ----------------------
    # Standing a few millimetres proud of the bench, like everything
    # this cell sets down: the moment the arm grasps it the part is
    # checked as part of the robot, and a part resting on a surface reads
    # as a robot touching it.
    scene.add_box(TOTE, (PART,) * 3,
                  (PART_X, SERVED_FACE - 0.09, BENCH_TOP + SEAT_GAP + PART / 2),
                  color=(0.62, 0.36, 0.14))
    scene.set_part(TOTE, category="workpiece", model="TOTE-60", mass_kg=PART_MASS)

    # The load sensor rides the machine, so it still reads "loaded" out on
    # the aisle — which is what a departure permit has to be able to ask.
    scene.add_zone_sensor("tray_loaded", position=(*machine.tray, machine.surface + 0.06),
                          size=(*machine.tray_size, 0.12), watch=[TOTE], mount="amr")
    # And a safety envelope that rides with it: the strip of aisle just
    # off the served side. The arm is in it whenever it works over the
    # bench and out of it once folded — "nothing overhangs while
    # travelling", written the way a cell writes an interlock. It watches
    # the arm *by name*: a zone told only `watch_robot=True` watches
    # everything, and this one is pointed straight at a bench.
    scene.add_zone_sensor("overhang",
                          position=(0.5 * (machine.lo[0] + machine.hi[0]),
                                    machine.lo[1] - 0.30, 1.00),
                          size=(machine.length, 0.50, 2.00),
                          watch_robots=["ur"], mount="amr")
    scene.set_obstacle_material("bench/top", metalness=0.4, roughness=0.5)
    return scene


# ------------------------------------------------------------------ teaching
def yaw_of(quaternion) -> float:
    x, y, z, w = quaternion
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def down(yaw: float) -> tuple:
    """Tool +Z at the floor, jaws square to `yaw`.

    Not decoration: with the tool pointing straight down the wrist is
    free to take any angle that produces the *world* orientation asked
    for, and asking for the same one from every direction hands back
    solutions with a turn of wrist wound into them. Aiming the jaws
    along the reach keeps the last joint near zero — and a wound wrist
    costs seconds later, unwinding.
    """
    return (math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0)


def in_machine(point, station, yaw: float) -> tuple:
    """A world point read in the machine's own frame when it stands at
    `station` heading `yaw`. The arm's program is written in this frame:
    a mobile manipulator does not know where it is, only what is in front
    of it."""
    dx, dy = point[0] - station[0], point[1] - station[1]
    c, s = math.cos(-yaw), math.sin(-yaw)
    return (c * dx - s * dy, s * dx + c * dy, point[2])


def build_cycle(scene: bt.Scene, carrier: str = CARRIER,
                drive_and_plan: bool = False) -> str:
    """Teaches the poses in the machine's frame and writes the cycle."""
    machine = Carrier(carrier)

    def teach(point, standoff: float, finger: float, seed=None) -> list:
        """IK on a point given in the machine's frame.

        The scene's base is wherever the vehicle is parked — the infeed
        station — so the target is read back out into the world *there*.
        What comes back is a joint vector, and a joint vector is relative
        to the base by construction: it lands on the same spot in the
        machine's frame whichever station the machine is standing at.

        `seed` is the pose to solve from, and it matters: an arm working
        over its own deck has a fold for every target, and hovering with
        one and descending with another is a lurch, not a motion.
        """
        scene.set_joint_positions(seed or READY)
        target = (machine.infeed[0] + point[0], machine.infeed[1] + point[1],
                  point[2] + standoff)
        reach = math.atan2(point[1] - machine.mount[1], point[0] - machine.mount[0])
        ik = scene.set_tcp_target(target, down(reach))
        if not ik.converged:
            raise RuntimeError(
                f"{machine.product}: the arm cannot reach "
                f"({point[0]:+.2f}, {point[1]:+.2f}, {point[2]:.2f}) in the "
                f"machine's own frame, {ik.pos_error * 1e3:.0f} mm short — a "
                f"{machine.deck * 1e3:.0f} mm deck puts that "
                f"{point[2] - machine.mount[2]:+.2f} m from the arm's base"
            )
        return [*scene.joint_positions[:6], finger]

    # The three places the arm works, all in the machine's frame: the
    # bench beside it at the infeed station, its own deck (which is in
    # the same place everywhere), and the belt beside it in the bay.
    bench = in_machine((PART_X, SERVED_FACE - 0.09, BENCH_TOP + SEAT_GAP + GRIP),
                       machine.infeed, 0.0)
    deck = (*machine.seat, machine.surface + SEAT_GAP + GRIP)
    belt = in_machine((BELT_X, machine.outfeed[1] + machine.mount[0],
                       BELT_TOP + SEAT_GAP + GRIP), machine.outfeed, math.pi / 2)

    # A hover and a seat at each, taught in that order and seeded from
    # each other — then each in both gripper states, because every taught
    # pose carries the fingers with it. A pose taught open would re-open
    # the gripper mid-carry; the deck is approached both ways, holding a
    # part on the way out and empty on the way back.
    for name, point in (("bench", bench), ("deck", deck), ("belt", belt)):
        over = teach(point, HOVER, OPEN)
        seat = teach(point, 0.0, OPEN, over)
        for state, finger in (("open", OPEN), ("shut", SHUT)):
            scene.add_segment(f"over_{name}_{state}", goal=[*over[:6], finger])
            scene.add_segment(f"at_{name}_{state}", goal=[*seat[:6], finger])
    # The travelling pose: the fold, with the wrist left where the deck
    # work put it.
    parked = [q for _, q in scene.motion_segments("over_deck_open")][-1]
    stow = [*STOWED[:5], parked[5], OPEN]
    scene.add_segment("home", goal=stow)

    # A ramp is not planned and not collision-checked — it is a commanded
    # interpolation, which is exactly what makes it legal while driving.
    # So the fold is checked here instead, the whole way in, against the
    # machine it folds onto: whoever writes a ramp owns its path.
    for i in range(21):
        blend = i / 20
        scene.set_joint_positions([a + (b - a) * blend for a, b in zip(parked, stow)])
        fouled = scene.check_collisions()
        if fouled:
            (_, a), (_, b) = fouled[0]
            raise RuntimeError(f"{machine.product}: folding into the stow puts {a} "
                               f"through {b} ({blend:.0%} of the way in)")
    scene.set_joint_positions(READY)

    sq = scene.sequence("amr_transfer")
    sq.step("接近", actions=[bt.seq.motion("over_bench_open")])
    sq.step("下降", actions=[bt.seq.motion("at_bench_open")])
    sq.step("把持", actions=[bt.seq.ramp({"finger_joint": SHUT}, 0.5)])
    sq.step("保持", actions=[bt.seq.attach(TOTE, touch_links=PADS)])
    sq.step("持上", actions=[bt.seq.motion("over_bench_shut")])
    sq.step("移載", actions=[bt.seq.motion("over_deck_shut")])
    sq.step("載置", actions=[bt.seq.motion("at_deck_shut")])
    sq.step("解放", actions=[bt.seq.ramp({"finger_joint": OPEN}, 0.4),
                             bt.seq.detach(TOTE)])
    # The departure permit a real machine waits on, asked of the machine
    # itself: its own load sensor says the part is aboard — it rides with
    # the load, so it still answers out on the aisle — and its own
    # envelope says nothing is hanging over the side.
    sq.step("退避", actions=[bt.seq.motion("over_deck_open")],
            transition=bt.seq.all_of(bt.seq.done(), bt.seq.signal("tray_loaded"),
                                     bt.seq.signal("overhang", False)))

    if drive_and_plan:
        # The error this demo exists to show: a plan cannot be baked
        # against a base that is about to move out from under it.
        sq.step("走行(誤)", actions=[bt.seq.goto("amr", "outfeed"),
                                     bt.seq.motion("over_belt_shut")])
        return sq.name

    # The right way round: dispatch, and *ramp* the arm into its stow
    # while it travels. The fold costs no cycle time at all.
    sq.step("走行", actions=[bt.seq.goto("amr", "outfeed"),
                             bt.seq.ramp(dict(zip(scene.robot.joint_names, stow)), 1.4)],
            transition=bt.seq.device_done("amr"))
    # Same deck, same taught pose. The machine has moved; the deck has not.
    sq.step("取出", actions=[bt.seq.motion("over_deck_open")])
    sq.step("下降", actions=[bt.seq.motion("at_deck_open")])
    sq.step("把持", actions=[bt.seq.ramp({"finger_joint": SHUT}, 0.5)])
    sq.step("保持", actions=[bt.seq.attach(TOTE, touch_links=PADS)])
    sq.step("持上", actions=[bt.seq.motion("over_deck_shut")])
    sq.step("払出", actions=[bt.seq.motion("over_belt_shut")])
    sq.step("投入", actions=[bt.seq.motion("at_belt_shut")])
    sq.step("離脱", actions=[bt.seq.ramp({"finger_joint": OPEN}, 0.4),
                             bt.seq.detach(TOTE), bt.seq.start("outfeed")])
    sq.step("復帰", actions=[bt.seq.motion("over_belt_open")])
    sq.step("格納", actions=[bt.seq.motion("home")])
    return sq.name


# ------------------------------------------------------------------- reports
def load_chain(machine: Carrier) -> list:
    """What each product in the stack is rated for against what it
    actually carries, tightest margin first.

    A mobile manipulator is three data sheets in series: the gripper
    holds the part, the arm holds the gripper and the part, the carrier
    holds all of it. Exactly one of them is the binding one, and it is
    worth knowing which before the cell is built rather than after.
    """
    arm, gripper = manifest(ARM)["specs"], manifest(GRIPPER)["specs"]
    return sorted([
        ("gripper", gripper["payload_kg"], PART_MASS),
        ("arm", arm["payload_kg"], PART_MASS + gripper["mass_kg"]),
        ("carrier", machine.specs["payload_kg"],
         PART_MASS + gripper["mass_kg"] + arm["mass_kg"]),
    ], key=lambda row: row[1] - row[2])


def pivot_at(tl, dt: float = 0.02) -> float:
    """When the machine stopped running and started turning — read off
    the baked base track rather than assumed from the layout."""
    heading = yaw_of(tl.base_pose(0.0)[1])
    t = 0.0
    while t <= tl.duration:
        if abs(yaw_of(tl.base_pose(t)[1]) - heading) > 1e-3:
            return t
        t += dt
    return tl.duration


def bake(carrier: str, drive_and_plan: bool = False, holonomic: bool = False):
    """One carrier, all the way through: scene, cycle, timeline."""
    scene = build_scene(carrier, holonomic=holonomic)
    name = build_cycle(scene, carrier, drive_and_plan)
    return scene, scene.simulate_sequence(name, max_duration=150.0)


def compare() -> None:
    """The same authored cell, baked on every carrier in the catalog.

    Three kinds of answer come back, and they come from three different
    places: the *package* can rule a machine out before anything is
    built (a deck that is not a deck), the *teaching* can (a reach the
    arm has not got), and the *bake* can (a body that will not go round
    the corner). All three are deterministic, and none of them is an
    opinion about the machine — they are this cell's requirements met or
    not met.
    """
    print(f"{'carrier':<11} {'deck':>5} {'body':>12} {'swing':>6} {'v':>5} "
          f"{'cycle':>7}   verdict")
    for name in CANDIDATES:
        try:
            machine = Carrier(name)
        except ValueError as err:
            print(f"{manifest(name)['name']:<11} {'—':>5} {'—':>12} {'—':>6} "
                  f"{'—':>5} {'—':>7}   {first_line(err)}")
            continue
        row = (f"{machine.product:<11} {machine.deck * 1e3:4.0f}  "
               f"{machine.length:5.2f}x{machine.width:<5.2f} {machine.swing:6.2f} "
               f"{machine.cruise(CORNER_X - machine.infeed[0]):5.2f}")
        try:
            _, tl = bake(name)
        except (ValueError, RuntimeError) as err:
            print(f"{row} {'—':>7}   {first_line(err)}")
            continue
        print(f"{row} {tl.duration:6.2f}s   ok")
    print("\nOne cell, one cycle, one set of taught poses. What changed is the")
    print("carrier — and with it the height the arm works from, where the")
    print("machine has to stand to reach the bench, how deep into the bay it")
    print("fits, the body that has to clear the corner, and the leg speeds.")


def first_line(err: Exception) -> str:
    """The one line of a failure worth putting in a table."""
    text = " ".join(str(err).split())
    return text if len(text) < 110 else text[:107] + "..."


def main() -> None:
    args = sys.argv[1:]
    carrier = args[args.index("--carrier") + 1] if "--carrier" in args else CARRIER
    if "--compare" in args:
        compare()
        return
    out = next((a for a in args if not a.startswith("--") and a != carrier),
               "cell_amr.usda")

    try:
        machine = Carrier(carrier)
    except ValueError as err:
        print(f"{carrier} cannot carry this cell: {err}")
        sys.exit(1)
    print(f"{machine.product} ({machine.maker}) + {manifest(ARM)['name']} + "
          f"{manifest(GRIPPER)['name']}")
    print(f"  deck      {machine.deck * 1e3:4.0f} mm, chassis {machine.proud * 1e3:.0f} mm "
          f"proud of the mount frame")
    print(f"  arm at    ({machine.mount[0]:+.3f}, {machine.mount[1]:+.3f}, "
          f"{machine.mount[2]:.3f}) in the machine's frame; "
          f"tray {machine.tray_size[0]:.2f} x {machine.tray_size[1]:.2f} m ahead of it")
    print(f"  body      {machine.length:.2f} x {machine.width:.2f} m, "
          f"pivot sweeps {machine.swing:.2f} m")
    print(f"  cruise    {machine.cruise(CORNER_X - machine.infeed[0]):.2f} m/s "
          f"(data sheet {machine.specs['max_speed_mps']:.2f})")
    for who, rated, carried in load_chain(machine):
        print(f"  {who:<9} carries {carried:5.1f} kg of {rated:6.1f} kg rated")

    try:
        _, tl = bake(carrier, "--drive-and-plan" in args,
                     holonomic="--holonomic" in args)
    except (ValueError, RuntimeError) as err:
        print(f"\ncycle failed: {err}")
        sys.exit(1)

    print(f"\ncycle time: {tl.duration:.2f}s")
    for step, start, end in tl.step_spans:
        print(f"  {step:<9} {start:6.2f} – {end:6.2f}s")

    lanes = dict(tl.signals)
    for lane in ("amr", "tray_loaded", "overhang", "outfeed"):
        edges = ", ".join(f"{t:.2f}→{'on' if v else 'off'}" for t, v in lanes[lane])
        print(f"  {lane:<12} {edges}")

    # The base is a track now, not a constant — and the pivot in the
    # middle of it is read off that track, not assumed from the layout.
    drive = tl.step_span("走行")
    for label, t in (("infeed", 0.0), ("outfeed", tl.duration)):
        p, q = tl.base_pose(t)
        print(f"  arm base at {label:<8}{tuple(round(v, 3) for v in p)}, "
              f"heading {math.degrees(yaw_of(q)):+.0f}°")
    print(f"  corner at {pivot_at(tl):.2f}s, {machine.swing:.2f} m of swing in a "
          f"{-SERVED_FACE:.2f} m half-aisle")

    # The stow is free, and this is what free looks like: the ramp runs
    # inside the drive it shares a step with, so no cycle time is spent
    # folding. A *planned* motion there is the error `--drive-and-plan`
    # shows; a ramp is re-evaluated every tick and simply travels along.
    fold = next((m for m in tl.moves()
                 if m[0] == "ramp" and drive.start <= m[1] < drive.end), None)
    if fold:
        print(f"  stow ramp {fold[1]:.2f} – {fold[2]:.2f}s, inside the "
              f"{drive.start:.2f} – {drive.end:.2f}s drive: the fold costs "
              f"{max(0.0, fold[2] - drive.end):.2f}s of cycle time")

    landed = tl.object_pose(TOTE, tl.duration)[0]
    print(f"  part ends at {tuple(round(v, 3) for v in landed)}, "
          f"picked at ({PART_X}, {SERVED_FACE - 0.09:.2f})")

    tl.export_usd(out, fps=60)
    print(f"wrote {out}")
    print(f"  replay it with:  python examples/play_record.py {out}")


if __name__ == "__main__":
    main()

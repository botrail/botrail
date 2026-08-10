"""One spot-welding station: four R-2000iC arms weld a real body-in-white.

This is the W0 cell of design/design-weld-line.md — the smallest slice of a
body-in-white line that still has the line's three defining ingredients:

* **Indexed transfer.** A 4.1 m body rides a skid down a conveyor that runs in
  fixed-length steps: feed in, stand still while the guns work, feed out.
  The advance is `start -> elapsed(pitch/v) -> stop`, and because the
  conveyor advances a deterministic `v*dt` per tick, the body lands on the
  same millimetre every cycle — which is what lets poses be taught against
  the *stopped* body at all.

* **Spot welds, not grasps.** Each robot carries a servo spot gun from the
  catalog (`botrail/weld/weld-gun-x1`). A weld is a little program of its
  own — slide onto the tab, squeeze the electrodes, hold the current,
  release, withdraw — and `weld_pass` below writes those five steps per spot
  so the cell reads as "weld this list", not as forty steps of ramp soup.

* **A contested stretch.** Two arms work each side of the body, one upstream
  and one downstream, and most of their spots are private. The two nearest
  the middle are not: each arm takes the spot just *past* the centre of the
  seam, so the guns swap ends and stand in the same half-metre. A zone
  interlock over that stretch serialises them. It is load-bearing: run with
  `--clash` (which swaps the gates for "go now") and the rollout reports the
  guns colliding, with a timestamp.

The body is the catalog's `botrail/body/biw-sedan`, loaded the way a
workpiece is meant to be: the shell mesh for looks with collision off, and
73 authored convex pieces for collision with rendering off — a convex
decomposition of the shell would fill the apertures a gun works through.

Where the spots go was not a styling choice. `design/design-weld-line.md`
records the reach survey that found the two flange families the asset ships
(roof ditch, rocker) have no process window for this arm and gun, while the
seam along the body's flank gives every arm a real one. It also set the
layout: pulled in to 0.70 m apart the two arms on a side box each other in
and the outermost spot drops to a single approach; at 1.45 m every spot has
two or three. The welded tabs are authored here rather than in the asset for
the same reason a real plant owns its weld schedule: the body is geometry,
the spots are process.

Run with:  python examples/weld_station_demo.py [out.usda] [--clash]
"""

import math
import sys
from pathlib import Path

import botrail as bt

S2 = math.sqrt(0.5)
SIDES = ("lh", "rh")
ROLES = ("up", "dn")                # upstream / downstream along the line
ARMS = [f"{side}_{role}" for side in SIDES for role in ROLES]
SIDE = {"lh": 1.0, "rh": -1.0}


def side_of(arm: str) -> float:
    return SIDE[arm.split("_")[0]]


def role_of(arm: str) -> str:
    return arm.split("_")[1]


def other_role(role: str) -> str:
    return "dn" if role == "up" else "up"


# Catalog parts. The arm is pinned to a revision on purpose: `r2` is
# FANUC's own description package (Apache-2.0), whose visual meshes are the
# real castings — the community `r1` ships one collision-quality mesh for
# both roles, which reads as a faceted blob at demo scale. Both revisions
# are the same machine and the same kinematics; only J3's declared range
# differs (`r2` is the narrower, official 259°), and this cell stays well
# inside it.
ARM = "fanuc/r2000ic/r2000ic-165f/r2"
GUN_ID = "botrail/weld/weld-gun-x1/r2"
BODY_ID = "botrail/body/biw-sedan/r7"
SHELL = "/World/Line/body/shell"

# --- line geometry -------------------------------------------------------
# Everything hangs off the skid plane: the top of the skid is where the body
# rests, and the body is seated on it by asking the mesh where its underside
# is (`obstacle_bounds`) rather than by writing down a number measured off
# one revision of the asset.
SKID_TOP = 0.90
SHEET = 0.012                      # flange thickness everywhere
# Robot bases. Two per side, straddling the station along the line: the
# stand-off is set by the gun (the wrist sits a throat-depth back from the
# spot, so a deeper gun pushes it towards the base), and the split along x
# is what gives each arm its own half of the seam and the pair a stretch
# they share.
BASE_Y = 2.40
# How far apart the two arms on a side stand. Measured, not styled: pulled
# in to 0.70 they work the same 2.4 m of seam and each one's approach is
# boxed in by the other's half — the process window at the outermost spot
# drops to a single orientation. At 1.45 every spot has two or three clear
# approaches and each arm owns its own end of the body.
BASE_X = 1.45
BELT_V = 0.40                      # m/s
# Indexed transfer is a *distance*, commanded as one: `advance(pitch)` runs
# the belt for exactly this many metres and stops on the millimetre. The
# previous authoring was `start -> elapsed(pitch/v) -> stop`, which loses
# one scan of travel to the stop landing on the step after the timer — the
# 8.01-not-8.00 arithmetic this cell used to carry (design-weld-line.md E2,
# retired by W1's `bt.seq.advance`).
FEED_IN = 3.2                      # metres: line head -> station datum
FEED_OUT = 6.0                     # metres: the body clears into the sink
LINE_IN = -3.2
SINK_X = 4.10                      # inside the guarding, past the station

# The seam. `PROUD` is how far outboard of the body's own skin the weld
# plane sits, `TAB_OUT` how far the welded tab stands out: the electrode
# has to reach the tab's face without the gun body touching the flank, and
# the tab has to be deeper than the weld plane so the gun straddles metal
# rather than air.
PROUD = 0.09
TAB_OUT = 0.12
TAB_H = 0.10
# Where the tab sits along the electrode axis, measured from the taught
# point. The TCP is the fixed electrode's tip, so a tab centred on it is a
# tab the gun is already standing in: the sheet has to be offset by its own
# half-thickness plus a standoff, and the moving electrode closes onto it
# from the far side. Contact *is* collision to a rollout, so the cycle
# squeezes to just shy of the sheet — the same 5 mm the rest of the cell is
# taught with.
TAB_STANDOFF = 0.005 + SHEET
ROLE_SIGN = {"up": 1.0, "dn": -1.0}
SEAM_Z_BODY = 0.70                 # height up the body, from its underside
# Which arm takes which spot. The two private spots are on the arm's own
# half of the seam; the third is past the middle, in the other arm's half —
# which is exactly what the interlock is for. Ordered as the cycle runs
# them: far, near, then across.
SPOT_X = {"up": (-1.50, -0.75, 0.15), "dn": (1.50, 0.75, -0.15)}
# The gun leaves a spot by dropping, not by backing off: the throat opens
# upward, so lowering the gun is what lets the tab out of it. Backing off
# along the seam instead folds the arm towards its own base and puts J3
# into the gun body — measured, not assumed.
CLEAR = 0.10

# Gun stroke: open enough to travel with the tab in the throat, squeezed to
# electrode contact (the taught plane leaves ~5 mm of gap a side).
GUN = "electrode_joint"
GUN_OPEN = 0.25
# Measured against the tab, not chosen: the moving electrode reaches the
# sheet at 0.055, so the cycle stops 5 mm short of it. A rollout counts
# electrode-on-sheet as a collision, which is the one part of a real weld
# this cell cannot act out.
GUN_SQUEEZE = 0.060
SQUEEZE_T, WELD_T, LIFT_T, TRAVEL_T = 0.25, 0.4, 0.5, 0.9

# Gun attitude at the seam, as (x, y, z, w). The throat points straight up
# and the electrode axis lies along the line, so the gun straddles a tab
# whose faces are normal to x — pinching across the joint, not along it.
# Upstream and downstream arms take opposite ends of that axis, which is
# the same pose seen from the two ends of the station. Both sides of the
# line use the same pair: two arms facing each other across a line are
# related by a half turn about z, not by a mirror, and a half turn about z
# leaves "throat up" alone while swapping +x for -x.
Q_ROLE = {"up": (S2, 0.0, S2, 0.0), "dn": (0.0, -S2, 0.0, S2)}

# Drawn back and up: at ready the guns are high on their own side of the
# line, clear of each other and of the body. J4 is parked half a turn round
# because that is the band the cycle actually works in; with the wrist at
# zero instead, every approach has to cross ~250 deg of J4 to reach the
# seam. A full turn of J4 leaves the arm in exactly this pose, so the
# choice costs nothing and is only about which way round the cycle travels.
# Every arm parks J4 at the *same* half turn, not mirrored: +pi and -pi are
# one pose but different numbers, and the unwinding below chases whichever
# is written here.
READY = [0.0, -0.9, -0.3, -math.pi, -0.9, 0.0, GUN_OPEN]

BODIES = 2                         # cycles through the station
# Bare steel with a zinc cast, which is what a body looks like before paint
# — a body-in-white is only "white" in the trade sense.
STEEL = (0.42, 0.45, 0.48)
SKID_COLOR = (0.09, 0.10, 0.12)

# How each surface takes light, as `(metalness, roughness)`. Colour alone
# cannot tell bare steel from a painted cabinet from a concrete-grey floor:
# they can share a grey and still look nothing alike. Metal reflects its
# surroundings and has no diffuse colour of its own, which is exactly what
# makes an unpainted body read as *metal* rather than as grey plastic.
MAT_STEEL = (0.85, 0.42)           # bare, lightly oiled panel steel
MAT_SKID = (0.70, 0.68)            # painted and scuffed by years of use
MAT_MACHINE = (0.80, 0.35)         # machined conveyor bed and rails
MAT_PAINT = (0.15, 0.55)           # painted sheet: cabinets, guarding
MAT_MATTE = (0.05, 0.92)           # non-metals — markings, trunking

PARK = (LINE_IN - 1.6, 0.0, -1.6)

# Spot-mark obstacle names, filled by `build_cell` — the nuggets the cycle
# leaves behind. The replay regression asserts exactly what moves, and the
# marks ride the line too.
MARKS: list = []

# Static line furniture: what the cell looks like around the robots. Each
# entry is `(name, size, position, color, collides)` — the guarding and the
# cabinets are real obstacles the planner has to respect, while markings
# and overhead trays are scenery that would only get in the way of a plan
# that was never going to touch them.
FENCE = (0.72, 0.62, 0.16)
GUARD_H = 2.00
CABINET = (0.24, 0.26, 0.30)
FLOOR_MARK = (0.68, 0.58, 0.12)
GUARD_X = 4.40
# The guarding stands off the robots, not off the origin: move the bases
# out for a wider body and a fixed fence ends up inside the J1 casting.
GUARD_Y = BASE_Y + 0.95
OPENING = 1.55                     # half-width of the gap the line runs through
# The transfer bed is drawn as a deep beam rather than a thin plate so that
# its *origin* sits well below the skid's. Conveyor advection tests each
# obstacle's origin against the zone box, so the bed and the thing it
# carries have to be separable by that one point — a slab whose centre is
# 55 mm under the skid's centre cannot be (caught once by playing a
# recording back, with the bed sliding 3.2 m downstream).
BED_TOP = SKID_TOP - 0.06
BED_H = 0.24
SCENERY = [
    ("Bed", (9.60, 0.86, BED_H), (0.0, 0.0, BED_TOP - BED_H / 2), (0.16, 0.17, 0.19), True),
    ("BedLeg_1", (0.16, 0.70, 0.72), (-3.40, 0.0, 0.36), (0.20, 0.21, 0.23), True),
    ("BedLeg_2", (0.16, 0.70, 0.72), (0.0, 0.0, 0.36), (0.20, 0.21, 0.23), True),
    ("BedLeg_3", (0.16, 0.70, 0.72), (3.40, 0.0, 0.36), (0.20, 0.21, 0.23), True),
    # Mounting plates, under the base casting. The top face is held 20 mm
    # below the mounting plane, not flush: the base's convex hull bulges a
    # little under it, so a plate drawn flush reads as a permanent
    # collision.
    ("Plate_lh_up", (1.10, 1.10, 0.06), (-BASE_X, BASE_Y, -0.05), (0.22, 0.23, 0.25), True),
    ("Plate_lh_dn", (1.10, 1.10, 0.06), (BASE_X, BASE_Y, -0.05), (0.22, 0.23, 0.25), True),
    ("Plate_rh_up", (1.10, 1.10, 0.06), (-BASE_X, -BASE_Y, -0.05), (0.22, 0.23, 0.25), True),
    ("Plate_rh_dn", (1.10, 1.10, 0.06), (BASE_X, -BASE_Y, -0.05), (0.22, 0.23, 0.25), True),
    # Weld controllers and transformers, one pair per side.
    ("WeldCtrl_lh", (0.70, 0.60, 1.60), (-2.60, BASE_Y + 0.55, 0.80), CABINET, True),
    ("WeldCtrl_rh", (0.70, 0.60, 1.60), (-2.60, -BASE_Y - 0.55, 0.80), CABINET, True),
    ("Transformer_lh", (0.55, 0.50, 0.90), (2.55, BASE_Y + 0.60, 0.45), (0.30, 0.31, 0.33), True),
    ("Transformer_rh", (0.55, 0.50, 0.90), (2.55, -BASE_Y - 0.60, 0.45), (0.30, 0.31, 0.33), True),
    # Decoration: nothing a plan could reasonably want to pass through.
    ("Andon", (0.14, 0.14, 0.60), (-GUARD_X + 0.3, GUARD_Y - 0.3, 2.30), (0.85, 0.55, 0.10), False),
    ("Tray_lh", (8.60, 0.30, 0.10), (0.0, BASE_Y + 1.05, 2.85), (0.28, 0.29, 0.32), False),
    ("Tray_rh", (8.60, 0.30, 0.10), (0.0, -BASE_Y - 1.05, 2.85), (0.28, 0.29, 0.32), False),
    ("Mark_lh", (8.60, 0.10, 0.004), (0.0, 1.45, 0.002), FLOOR_MARK, False),
    ("Mark_rh", (8.60, 0.10, 0.004), (0.0, -1.45, 0.002), FLOOR_MARK, False),
]


def guarding() -> list:
    """Cell guarding as posts and rails rather than panels.

    A solid panel walls the cell off from every camera; a translucent one
    fogs everything behind it, and two of them fog it twice. Real guarding
    is mostly holes — posts, a top rail, a knee rail, and mesh you can see
    straight through — so drawing the frame and leaving the mesh out is
    both the honest shape and the one you can watch a cycle through.
    """
    out = []
    post = (0.08, 0.08, GUARD_H)
    span = 2 * GUARD_X - 0.16
    for side, y in (("lh", GUARD_Y), ("rh", -GUARD_Y)):
        for i, x in enumerate((-4.4, -2.2, 0.0, 2.2, 4.4)):
            out.append((f"Post_{side}_{i}", post, (x, y, GUARD_H / 2), FENCE, True))
        for tag, z in (("top", GUARD_H - 0.06), ("knee", 0.55)):
            out.append((f"Rail_{side}_{tag}", (span, 0.05, 0.08), (0.0, y, z), FENCE, True))
    # The ends are open where the line runs through; guarding stops either
    # side of the opening. The opening is set by what has to pass through
    # it — a 1.78 m body with tabs standing proud of the flank — and by
    # what must *not* end up inside the transfer zone: a post whose origin
    # lands in the zone rides the belt along with the body.
    span = GUARD_Y - OPENING
    for end, x in (("in", -GUARD_X), ("out", GUARD_X)):
        for side, sign in (("lh", 1.0), ("rh", -1.0)):
            out.append((f"Post_{end}_{side}", post, (x, sign * OPENING, GUARD_H / 2),
                        FENCE, True))
            for tag, z in (("top", GUARD_H - 0.06), ("knee", 0.55)):
                out.append((
                    f"Rail_{end}_{side}_{tag}", (0.05, span, 0.08),
                    (x, sign * (OPENING + span / 2), z), FENCE, True,
                ))
    return out


SCENERY += guarding()


def material_for(name: str) -> tuple:
    """Which of the five surfaces a piece of line furniture is made of."""
    if name.startswith(("Bed", "Plate")):
        return MAT_MACHINE
    if name.startswith(("Post_", "Rail_", "WeldCtrl", "Transformer", "Andon")):
        return MAT_PAINT
    return MAT_MATTE


class Station:
    """Where the body actually is, once the asset has been measured.

    Seating the body and placing the tabs are the same arithmetic — both
    start from the mesh's own bounding box — and every later stage (teach,
    zones, conveyor) needs the answers. Computing them once and passing
    them around beats three copies of `SKID_TOP - min_z` that can drift
    apart when the asset is rebuilt.
    """

    def __init__(self, lo, hi):
        self.lift = SKID_TOP - lo[2]         # seats the body on the skid
        self.flank = hi[1]                   # the body's own half-width
        self.seam_y = hi[1] + PROUD          # where an electrode meets a tab
        self.seam_z = self.lift + SEAM_Z_BODY
        self.length = hi[0] - lo[0]
        self.height = hi[2] - lo[2]

    def spot(self, arm: str, index: int) -> tuple:
        return (SPOT_X[role_of(arm)][index], side_of(arm) * self.seam_y, self.seam_z)

    def withdrawn(self, arm: str, index: int) -> tuple:
        x, y, z = self.spot(arm, index)
        return (x, y, z - CLEAR)


def body_meshes() -> tuple:
    """The catalog body, as `(display shell, [(collision name, mesh path)])`.

    A `workpiece` package ships both, and the cell needs both: convex
    decomposition of a body shell fills the door and window apertures, and
    a welding gun works *through* those, so the pieces that do the
    colliding are authored convex parts with the openings left between
    them — and they are not what the body looks like. Load the shell for
    looks with collision off, the pieces for collision with rendering off.
    """
    package = Path(bt.catalog_package(BODY_ID))
    pieces = sorted((package / "collision").glob("*.stl"))
    if not pieces:
        raise SystemExit(f"catalog package {BODY_ID} ships no collision meshes")
    shell = package / "sources" / "_derived" / "biw__biw_shell.stl"
    return (shell if shell.exists() else None,
            [(f"/World/Line/body/{p.stem.split('__')[-1]}", p) for p in pieces])


def build_cell() -> tuple:
    arm = bt.Robot.from_catalog(ARM)
    gun = bt.Robot.from_catalog(GUN_ID)
    robot = arm.attach_tool(gun)

    scene = bt.Scene(robot)
    scene.rename_robot(scene.robots[0], ARMS[0])
    for name in ARMS:
        x = -BASE_X if role_of(name) == "up" else BASE_X
        y = side_of(name) * BASE_Y
        facing = (0.0, 0.0, -S2, S2) if side_of(name) > 0 else (0.0, 0.0, S2, S2)
        if name == ARMS[0]:
            scene.set_robot_base_pose((x, y, 0.0), facing, robot=name)
        else:
            scene.add_robot(robot, name=name, base_position=(x, y, 0.0),
                            base_quaternion=facing)
        scene.set_joint_positions(READY, robot=name)

    # The cell around the robots: transfer bed, pedestals, weld controllers
    # and the guarding. Collision is on for everything an arm could
    # plausibly reach, so a plan has to respect the cell it stands in.
    for name, size, position, color, collides in SCENERY:
        prim = f"/World/Cell/{name}"
        scene.add_box(prim, size, position, color=color)
        scene.set_obstacle_material(prim, *material_for(name))
        if not collides:
            scene.set_obstacle_enabled(prim, False)

    # The body goes in at its own origin first, purely to be measured: the
    # asset decides where its underside and its flank are, and everything
    # downstream is derived from that.
    shell, meshes = body_meshes()
    for name, path in meshes:
        scene.add_mesh(name, str(path), (0.0, 0.0, 0.0), color=STEEL)
        scene.set_obstacle_material(name, *MAT_STEEL)
        scene.set_obstacle_visible(name, False)      # collides, never drawn
    lo, hi = [1e9] * 3, [-1e9] * 3
    for name, _ in meshes:
        a, b = scene.obstacle_bounds(name)
        lo = [min(v, w) for v, w in zip(lo, a)]
        hi = [max(v, w) for v, w in zip(hi, b)]
    station = Station(lo, hi)
    if shell is not None:
        scene.add_mesh(SHELL, str(shell), (0.0, 0.0, 0.0), color=STEEL)
        scene.set_obstacle_material(SHELL, *MAT_STEEL)
        scene.set_obstacle_enabled(SHELL, False)     # drawn, never collides
        meshes = meshes + [(SHELL, shell)]

    # Everything that rides the line, as `(name, working pose)`. The 73
    # body pieces share one origin — the mesh vertices carry the shape —
    # so the whole shell advects as one rigid thing without a single joint.
    riders = [(name, (0.0, 0.0, station.lift)) for name, _ in meshes]
    for name, _ in meshes:
        scene.set_obstacle_pose(name, PARK)     # measured; now wait to be fed
    riders.append(("/World/Line/skid", (0.0, 0.0, SKID_TOP - 0.03)))
    scene.add_box("/World/Line/skid", (4.30, 0.70, 0.06), PARK, color=SKID_COLOR)
    scene.set_obstacle_material("/World/Line/skid", *MAT_SKID)

    # The welded features. Each is a plate normal to the line, standing
    # proud of the flank, so the gun straddles it across the joint.
    for role in ROLES:
        for index, x in enumerate(SPOT_X[role]):
            for side in SIDES:
                name = f"/World/Line/tab_{side}_{role}{index + 1}"
                pose = (x + ROLE_SIGN[role] * TAB_STANDOFF,
                        SIDE[side] * (station.flank + TAB_OUT / 2), station.seam_z)
                scene.add_box(name, (2 * SHEET, TAB_OUT, TAB_H), PARK, color=STEEL)
                scene.set_obstacle_material(name, *MAT_STEEL)
                riders.append((name, pose))

    # The skid pool: one source and one sink per piece, parked below the
    # floor until called. Sources are indexing feeders (one emission per
    # start) and the sinks hand pieces straight back, so the same body and
    # the same skid serve every cycle.
    for name, pose in riders:
        tag = name.rsplit("/", 1)[-1]
        scene.add_source(
            f"src_{tag}",
            pool=[name],
            park=PARK,
            pitch=(0.0, 0.0, 0.0),
            position=(LINE_IN + pose[0], pose[1], pose[2]),
            interval=0.0,
            running=False,
        )
        scene.add_sink(
            f"snk_{tag}",
            zone_position=(SINK_X, 0.0, station.seam_z - 0.3),
            zone_size=(0.6, 2.4, 1.4),
            source=f"src_{tag}",
        )
    # Advection tests each obstacle's *origin* against this box, so the
    # zone has to contain every rider's origin over its whole run and
    # nothing else. It is derived from the riders rather than written down:
    # the one static thing sharing their y is the transfer bed, and the bed
    # is kept out by its own (deliberately low) origin.
    ride_lo = [min(p[i] for _, p in riders) for i in range(3)]
    ride_hi = [max(p[i] for _, p in riders) for i in range(3)]
    x_from = LINE_IN + ride_lo[0] - 0.3
    x_to = SINK_X + ride_hi[0] + 0.3
    y_half = ride_hi[1] + 0.05
    z_from = (BED_TOP - BED_H / 2 + ride_lo[2]) / 2     # between bed and skid
    z_to = ride_hi[2] + 0.10
    # A conveyor carries whatever has its *origin* in the zone, so anything
    # standing in the cell whose origin happens to land there rides the belt
    # too — silently, 3.2 m at a time, and you find out when a fence post is
    # parked in front of a robot. The zone has to be drawn to fit the
    # freight; say so here rather than discover it in a replay.
    riding = {name for name, _ in riders}
    inside = [
        name for name in scene.obstacle_names
        if name not in riding
        and all(lo <= v <= hi
                for v, lo, hi in zip(scene.obstacle_pose(name)[0],
                                     (x_from, -y_half, z_from),
                                     (x_to, y_half, z_to)))
    ]
    if inside:
        raise SystemExit(
            "these would ride the transfer zone with the body: " + ", ".join(inside)
        )
    scene.add_conveyor(
        "line",
        zone_position=((x_from + x_to) / 2, 0.0, (z_from + z_to) / 2),
        zone_size=(x_to - x_from, 2 * y_half, z_to - z_from),
        velocity=(BELT_V, 0.0, 0.0),
        running=False,
    )

    # Part-present at the line head — the photo-eye every real transfer
    # gates on. It also closes a scan-order race: a source emits its body
    # *after* the belt has advected this tick, so a load and an advance
    # issued in the same scan cost the body its first 4 mm of travel and
    # it lands one scan short of datum, forever. Gating the feed on the
    # beam means the body is aboard before the pitch is commanded.
    scene.add_beam_sensor(
        "body_at_head",
        frm=(LINE_IN, -1.2, SKID_TOP + 0.35),
        to=(LINE_IN, 1.2, SKID_TOP + 0.35),
        radius=0.03,
    )

    # The contested stretch. Each arm's third spot is past the middle of
    # the seam, in the other's half, so the two guns on a side stand in the
    # same half-metre of it. One zone per arm over that volume: a zone says
    # "somebody is inside", so each gate has to watch the other arm through
    # its own sensor.
    for name in ARMS:
        scene.add_zone_sensor(
            f"zone_{name}",
            position=(0.0, side_of(name) * (station.seam_y + 0.1), station.seam_z),
            size=(0.70, 0.90, 1.30),
            watch_robots=[name],
        )

    # Process presentation, driven the PLC way. The weld-current signals
    # (one per role — that is who welds together) are set by the weld
    # steps like a weld controller's "current on" output; each arm's
    # flash is bound to its role's signal and blinks at its TCP in the
    # studio and in the USD export. The spot marks are the nuggets: one
    # small dark pad per (arm, spot), fed onto the tab the moment its
    # weld releases — the existing source machinery *is* the visibility
    # mechanism, so the marks appear in usdview exactly as in the studio,
    # and the tail sink recirculates them with everything else.
    MARKS.clear()
    for role in ROLES:
        scene.define_signal(f"arc_{role}", False)
    for name in ARMS:
        scene.add_weld_flash(f"flash_{name}", signal=f"arc_{role_of(name)}",
                             robot=name)
    for arm in ARMS:
        role, s = role_of(arm), side_of(arm)
        for index, x in enumerate(SPOT_X[role]):
            pose = (x + ROLE_SIGN[role] * TAB_STANDOFF,
                    s * (station.flank + TAB_OUT / 2), station.seam_z)
            # One mark per spot is the whole magazine: it rides out with
            # its body and the tail sink hands it back for the next one
            # (the same recirculation as every other rider).
            prim = f"/World/Line/mark_{arm}_s{index + 1}"
            scene.add_box(prim, (2 * SHEET + 0.004, 0.055, 0.055), PARK,
                          color=(0.09, 0.07, 0.06))
            scene.set_obstacle_material(prim, 0.55, 0.65)
            scene.set_obstacle_enabled(prim, False)
            pool = [prim]
            MARKS.append(prim)
            scene.add_source(
                f"src_mark_{arm}_s{index + 1}",
                pool=pool,
                park=PARK,
                pitch=(0.0, 0.0, 0.0),
                position=pose,
                interval=0.0,
                running=False,
            )
            scene.add_sink(
                f"snk_mark_{arm}_s{index + 1}",
                zone_position=(SINK_X, 0.0, station.seam_z - 0.3),
                zone_size=(0.6, 2.4, 1.4),
                source=f"src_mark_{arm}_s{index + 1}",
            )
    return scene, station, riders


TURN = 2.0 * math.pi
LIMIT_MARGIN = math.radians(2.0)


def unwind(q: list, reference: list, limits: list) -> list:
    """The same configuration, with every full-turn joint taken the short
    way round from `reference`.

    A wrist joint with more than a turn of travel reaches the same pose at
    q, q±2π, …, and the solver has no reason to prefer one — so two spots
    a few centimetres apart can come back a whole lap apart, and the ramp
    between them spins the wrist. Rotating by whole turns does not move a
    single link, so this is free: same poses, same collisions, shorter
    path."""
    out = list(q)
    for j, limit in enumerate(limits):
        if limit is None:
            continue
        # Stay off the stops: a goal *exactly* at a limit is rejected as out
        # of range (the planner treats the bound as exclusive), and a taught
        # pose sitting on one has nowhere to go if anything shifts.
        low, high = limit[0] + LIMIT_MARGIN, limit[1] - LIMIT_MARGIN
        for turn in (TURN, -TURN):
            candidate = out[j] + turn
            while low <= candidate <= high:
                if abs(candidate - reference[j]) < abs(out[j] - reference[j]):
                    out[j] = candidate
                candidate += turn
    return out


def unwind_chain(spots: list, limits: list) -> None:
    """Unwinds one arm's taught poses in the order the cycle runs them, so
    each is short-way-round from the one actually preceding it."""
    reference = READY
    for withdrawn, at in spots:
        for q in (withdrawn, at):
            q[:] = unwind(q, reference, limits)
            reference = q


def solve(scene: bt.Scene, robot: str, pos, quat, label: str) -> list:
    result = scene.set_tcp_target(pos, quat, robot=robot, max_iters=200)
    if not result.converged:
        raise SystemExit(f"teaching failed at {robot} {label}: {result.pos_error:.3e}")
    collisions = scene.check_collisions()
    if collisions:
        raise SystemExit(f"taught pose collides at {robot} {label}: {collisions}")
    return list(scene.joint_positions_of(robot))


def teach(scene: bt.Scene, station: Station, riders: list) -> dict:
    """Every taught configuration, taught with the body standing at the
    station — which is where it is whenever a gun is anywhere near it.

    Teaching against the empty cell instead makes every pose *look* clear
    (the body is parked under the floor) and the cell only finds out by
    driving a gun through a flank."""
    for name, pose in riders:
        scene.set_obstacle_pose(name, pose)
    try:
        return teach_against_the_body(scene, station)
    finally:
        for name, _ in riders:
            scene.set_obstacle_pose(name, PARK)


def teach_against_the_body(scene: bt.Scene, station: Station) -> dict:
    poses = {}
    for arm in ARMS:
        quat = Q_ROLE[role_of(arm)]
        # Park every arm before each solve. Leaving the others wherever
        # their last solve put them — often inside the body — poisons the
        # collision check for the one being taught, and does it
        # asymmetrically, which reads convincingly as a reach problem.
        spots = []
        for index in range(len(SPOT_X[role_of(arm)])):
            for name in ARMS:
                scene.set_joint_positions(READY, robot=name)
            if spots:
                # Warm-start from where the cycle actually leaves this arm,
                # so consecutive spots stay in one posture family instead of
                # coming back a wrist-lap apart.
                scene.set_joint_positions(spots[-1][1], robot=arm)
            out = solve(scene, arm, station.withdrawn(arm, index), quat,
                        f"withdrawn from spot {index + 1}")
            scene.set_joint_positions(out, robot=arm)
            at = solve(scene, arm, station.spot(arm, index), quat,
                       f"spot {index + 1}")
            spots.append((out, at))
            scene.set_joint_positions(READY, robot=arm)
        # Solved pose by pose, the wrist can come back a lap from where the
        # cycle left it. Retake every full turn the short way, in run order.
        unwind_chain(spots, scene.robot.joint_limits)
        poses[arm] = spots
    for name in ARMS:
        scene.set_joint_positions(READY, robot=name)
    return poses


def build_clash(scene: bt.Scene, poses: dict) -> str:
    """What the gates are for: both arms on a side go for the contested
    stretch in the same steps. Their taught poses are individually
    collision-free — it is only *simultaneity* that is fatal, which is
    exactly what a per-tick cross-robot check catches and a plan-one-arm-
    alone check cannot."""
    names = scene.robot.joint_names
    sq = scene.sequence("clash")
    sq.step("both_across",
            actions=[bt.seq.motion(f"{arm}_across") for arm in ARMS],
            transition=bt.seq.all_of(*[bt.seq.robot_done(arm) for arm in ARMS]))
    sq.step("both_engage", actions=[
        bt.seq.ramp(dict(zip(names, poses[arm][-1][1])), 1.0, robot=arm)
        for arm in ARMS
    ])
    return sq.name


def build_sequence(scene: bt.Scene, poses: dict, riders: list) -> str:
    names = scene.robot.joint_names

    def arm_to(q: list) -> dict:
        return dict(zip(names, q))

    for arm in ARMS:
        scene.add_segment(f"{arm}_enter", goal=poses[arm][0][0], robot=arm)
        scene.add_segment(f"{arm}_across", goal=poses[arm][-1][0], robot=arm)
        scene.add_segment(f"{arm}_home", goal=READY, robot=arm)

    sq = scene.sequence("weld_station")

    def weld_pass(tag: str, index: int, arms: list) -> None:
        """The E5 sugar: five steps per spot, every listed arm in lockstep.
        The weld step raises the participating roles' weld-current signals
        (what the flashes render); the release drops them and feeds the
        spot marks onto the tabs."""
        spot = f"{tag}_s{index + 1}"
        roles = sorted({role_of(a) for a in arms})
        for phase, which, duration in (("travel", 0, TRAVEL_T),
                                       ("engage", 1, LIFT_T)):
            sq.step(f"{spot}_{phase}", actions=[
                bt.seq.ramp(arm_to(poses[arm][index][which]), duration, robot=arm)
                for arm in arms
            ])
        sq.step(f"{spot}_squeeze", actions=[
            bt.seq.ramp({GUN: GUN_SQUEEZE}, SQUEEZE_T, robot=arm) for arm in arms
        ])
        sq.step(f"{spot}_weld",
                actions=[bt.seq.set_signal(f"arc_{role}", True)
                         for role in roles],
                transition=bt.seq.elapsed(WELD_T))
        sq.step(f"{spot}_release", actions=[
            bt.seq.ramp({GUN: GUN_OPEN}, SQUEEZE_T, robot=arm) for arm in arms
        ] + [bt.seq.set_signal(f"arc_{role}", False) for role in roles]
          + [bt.seq.start(f"src_mark_{arm}_s{index + 1}") for arm in arms])
        sq.step(f"{spot}_withdraw", actions=[
            bt.seq.ramp(arm_to(poses[arm][index][0]), LIFT_T, robot=arm)
            for arm in arms
        ])

    for body in range(BODIES):
        tag = f"b{body + 1}"
        # Call the next body onto the line and index it into the station.
        sq.step(f"{tag}_load",
                actions=[bt.seq.start(f"src_{name.rsplit('/', 1)[-1]}")
                         for name, _ in riders],
                transition=bt.seq.signal("body_at_head", True))
        sq.step(f"{tag}_feed", actions=[bt.seq.advance("line", FEED_IN)],
                transition=bt.seq.device_done("line"))
        sq.step(f"{tag}_spot")
        # All four arms come onto their own half of the seam together.
        sq.step(f"{tag}_enter",
                actions=[bt.seq.motion(f"{arm}_enter") for arm in ARMS],
                transition=bt.seq.all_of(*[bt.seq.robot_done(a) for a in ARMS]))
        last = len(SPOT_X["up"]) - 1
        for index in range(last):
            weld_pass(tag, index, ARMS)
        # The contested stretch is taken by role, not by side: the two
        # upstream arms cross over while both downstream arms are clear,
        # then the other way round. The sides never contend with each
        # other, so they still run together.
        #
        # The waiting pair stands off first. An arm crossing to the far
        # half of the seam swings 1.6 m past where the other one works,
        # and "out of the contested volume" is not the same as "out of
        # the way" — the zone is the shared *weld* volume, not the
        # corridor the elbow travels through.
        #
        # Everything below moves one role at a time — one arm per side.
        # That matters because a planned motion is planned against the
        # *cell*, not against the other robots: send all four home at once
        # and two same-side arms sweep through each other on the way.
        # Distance is not the safeguard either. Retreating to the far end
        # of its own half instead of home is a shorter trip, and the
        # planner routes it up over the roof and down — where it meets the
        # arm on the *other* side of the line, 4.8 m from its base. Home
        # is high, outboard, and reached the same way every cycle, which
        # is why the retreats go there and not somewhere cleverer.
        for role in ROLES:
            movers = [a for a in ARMS if role_of(a) == role]
            waiting = [a for a in ARMS if role_of(a) == other_role(role)]
            sq.step(f"{tag}_{role}_standoff",
                    actions=[bt.seq.motion(f"{arm}_home") for arm in waiting],
                    transition=bt.seq.all_of(*[bt.seq.robot_done(a) for a in waiting]))
            sq.step(f"{tag}_{role}_gate",
                    transition=bt.seq.all_of(*[bt.seq.signal(f"zone_{a}", False)
                                               for a in waiting]))
            sq.step(f"{tag}_{role}_across",
                    actions=[bt.seq.motion(f"{arm}_across") for arm in movers],
                    transition=bt.seq.all_of(*[bt.seq.robot_done(a) for a in movers]))
            weld_pass(f"{tag}_{role}", last, movers)
            # Leave the stretch before the other role is allowed in: the
            # gate watches the zone, so clearing it is what releases them.
            sq.step(f"{tag}_{role}_clear",
                    actions=[bt.seq.motion(f"{arm}_home") for arm in movers],
                    transition=bt.seq.all_of(*[bt.seq.robot_done(a) for a in movers]))
        sq.step(f"{tag}_home",
                actions=[bt.seq.motion(f"{arm}_home") for arm in ARMS],
                transition=bt.seq.all_of(*[bt.seq.robot_done(a) for a in ARMS]))
        # Index out: the body slides into the sink and hands its pieces
        # back to the pool.
        sq.step(f"{tag}_out", actions=[bt.seq.advance("line", FEED_OUT)],
                transition=bt.seq.device_done("line"))
        sq.step(f"{tag}_done")
    return sq.name


def zone_overlap(timeline) -> float:
    """Seconds two arms on the same side spend in the contested stretch at
    once."""
    spans = {}
    edges = dict(timeline.signals)
    for arm in ARMS:
        acc, start = [], None
        for t, on in edges[f"zone_{arm}"]:
            if on and start is None:
                start = t
            elif not on and start is not None:
                acc.append((start, t))
                start = None
        if start is not None:
            acc.append((start, timeline.duration))
        spans[arm] = acc
    return sum(
        max(0.0, min(a1, b1) - max(a0, b0))
        for side in SIDES
        for a0, a1 in spans[f"{side}_up"]
        for b0, b1 in spans[f"{side}_dn"]
    )


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    clash = "--clash" in sys.argv
    out = Path(args[0]) if args else Path("cell_weld.usda")

    scene, station, riders = build_cell()
    poses = teach(scene, station, riders)
    name = build_sequence(scene, poses, riders)

    if clash:
        # Both arms on each side go for the contested stretch together, and
        # the guns meet where the survey said they would. The rollout
        # reports it as a hard failure with a timestamp.
        try:
            scene.simulate_sequence(build_clash(scene, poses), max_duration=60.0)
            raise SystemExit("expected a robot-robot collision")
        except ValueError as e:
            print(f"unarbitrated seam, as caught by the rollout:\n   {e}")
        return

    timeline = scene.simulate_sequence(name, max_duration=400.0)
    welds = BODIES * len(ARMS) * len(SPOT_X["up"])
    print(f"body: {station.length:.2f} m long, {station.height:.2f} m tall, "
          f"seam at y=±{station.seam_y:.3f}, z={station.seam_z:.3f}")
    print(f"line takt: {timeline.duration / BODIES:.2f}s per body "
          f"({timeline.duration:.2f}s for {BODIES} cycles, {welds} spots, "
          f"one body and one skid recirculated)")
    for arm in timeline.robots:
        busy = sum(end - start for _, start, end in timeline.moves(arm))
        print(f"  {arm} moving {busy:5.2f}s of {timeline.duration:.2f}s")
    print(f"contested stretch co-occupancy: {zone_overlap(timeline):.2f}s")

    warnings = timeline.export_usd(out, fps=60.0)
    for w in warnings:
        print(f"warning: {w}")
    print(f"exported to {out} — view with: usdview {out}")


if __name__ == "__main__":
    main()

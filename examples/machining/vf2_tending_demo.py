"""A cobot tends a Haas VF-2 through its side window.

The cell Haas sells as a robot package, built from public figures: a
40-taper vertical machining centre with an arm at its right-hand window,
swapping the finished part in the vise for a blank between cycles. The
machine is the catalog pack `haas/vf/vf-2` — the layout drawing's travels,
table and openings, and the heights the drawing leaves out measured from
the maker's 3D model — stood by `bt.parts.machine_tool` as the *envelopes*
a tending cell verifies against, not as its shape:

* the enclosure stops at 1818 mm and is open above; the column and the
  spindle head rise to 2724 mm;
* the side window **lifts**: its leaf rests on the 819 mm sill and rises
  595 mm, less than the opening is tall, so what the arm passes under is
  the raised leaf's lower edge at 1414 mm — a 676 x 595 mm way in;
* the table, the saddle and the spindle head are a **mechanism** whose
  three joints are the machine coordinates — X 762, Y 406 and Z 508 mm
  to the negative side of home — and the vise rides the table.

The arm is a FANUC CR-7iA/L, the collaborative sibling of the LR Mate
200iD/7L in Haas's own package (7 kg, 911 mm), carrying a Robotiq Hand-E.
It stands on a low pedestal beside the window so that its elbow goes in
under the raised leaf.

**One part program runs the cell** (`PROGRAM`, the shape a CAM post
writes for a Haas, and the way a Haas is integrated): two tools, a facing
pass, a circular pocket and four pecked holes — then the table to the
window, `M71` to open the vise, `M180` to open the window, `M21` to call
the arm and wait for its M-FIN, `M70` to clamp the blank the arm holds in
the vise, `M22` to have it let go and leave, `M181` to close the window,
and `M99` to start over. Nothing starts the machine from outside: the
program keeps running, and the arm answers its relays.

The machine's side of that is not written here — `bt.tending.haas_mfin`
derives it from the program's text: the blocks the control runs on its
own, and at every one of those M codes the step that answers it. So the
interlock table and the handshake are the program's; the window opens
only while the cell says it is safe (an alarm otherwise, not a wait), a
block that turns the spindle needs the doors shut and the E-stop out, and
the bake checks the window against the arm every tick.

The control reads the program against its work-offset and tool tables
and runs it on the axes — so the cycle time comes out of the program
(`bt.nc.check` is the dry run that prints it) instead of being typed in.
The work offset `G54` is not typed in either: it is derived from where
the part stands in the vise. The relays are called with the table parked
at the far end of X, toward the window and in line with it — where the
arm was taught; a program that called the arm with the table elsewhere
is refused as the cell is built, and so is whatever the control cannot
read (an unknown word, a tool nobody entered, a point past a travel, an
M code in nobody's table), with the line.

**The part is cut.** The tools are bodies — an end mill and a drill in
their holders, whose lengths the table's entries are taken from — and the
block in the vise is carved by them on the table that carries it under
the spindle: faced, pocketed, drilled, and carried out through the window
as the part (`bt.nc.animate`). What comes off is measured against the
**design model** (`design_model`, the part as drawn — written here, not
taken from the program), and what a tool meets that it should not is
reported by source line. So the setup is something the cell checks: with
`--setup g54_high` the work zero is entered 2 mm too high and the first
part comes out with material left on it; with `--setup t2_short` the
drill's table entry is 20 mm short of the drill, and the dry run names
the line where it goes into the part at rapid; with `--setup vise_out`
the vise is bolted 30 mm nearer the window and the program is refused at
the line that would take X past its home. `bt.nc.setup_sheet` writes the
setup — stock, vise, offsets, tools — next to the USD.

**The vise holds what stands in it.** Its clamp is a coil the table's
grasp follows: `M70` takes whichever part is in the jaws, `M71` lets it
go — the block that was machined, then the blank the arm brought. The
arm holds the blank until the vise has closed on it.

**And what if the cell is not safe?** The scenario `no_cell_safe` breaks
that one wire: the program alarms at its `M180` and stops there, the arm
waits for a call that never comes, and the FAT sheet says at which line.
The document set (`deliver`) is written beside the USD: the bill, the
I/O list, the handshake, both programs as PLCopen, the interlock table —
its rows read off the part program's M codes — and the FAT.

**The picture is a model of the machine; the envelopes are what is
checked.** `shape=` draws a model package over them — the table and the
head as the castings that run the program, the window as the leaf that
lifts, the glass seen through, the umbrella tool changer bringing each
tool to the spindle. The one here is `haas/vf/vf-2-reference`:
drawn from the numbers of Haas's published layout drawing and of the
pack, with no geometry of Haas's own 3D model in it, so it is in the
catalog like any other pack. (Haas's model itself can be the picture too, where you have built
it — `recipes/haas/vf-2-cad-r2.yaml` in the catalog builder, from the ZIP
you download yourself; it is Haas's to distribute, not ours.) Where the
catalog cannot be reached the same cell is drawn as its envelopes, and
comes out the same to the tick.

What is not modelled: the Hand-E's coupling for this arm's 31.5 mm flange
(GRP-CPL-063 is not in the catalog — the hand is mounted on the flange
directly), and the pedestal is a plain one typed in here, not a product.

Run with:  python examples/machining/vf2_tending_demo.py [out.usdc]
                 [--catalog-root DIR] [--setup g54_high|t2_short|vise_out] [--studio]

Needs the catalog (`pip install botrail[catalog]`). `--catalog-root DIR`
(or `BOTRAIL_CATALOG_BUILD`) reads the machine's pack from a local catalog
build instead of the published one.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import tempfile
from pathlib import Path

import botrail as bt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import machine_tending_demo as one

HERE = Path(__file__).resolve().parent

# --------------------------------------------------------------- products
MACHINE = "haas/vf/vf-2/r2"
#: The machine's picture: a reference model drawn from the published
#: layout drawing's numbers (botrail-assets, CC0) — the enclosure with its
#: doors and windows, the table, the head and the umbrella tool changer as
#: links of their own. The cell is the same cell without it.
MACHINE_SHAPE = "haas/vf/vf-2-reference/r1"
VISE = one.VISE_CATALOG
ARM = "fanuc/cr7ia/cr7ia-l/r1"
HAND = "robotiq/hand-e/hand-e/r1"
ROBOT = "arm"
# ------------------------------------------------------------------ cell
# The machine stands at the origin, its window on the right (+X). Where
# the part program parks the table for the exchange, as the offset of the
# table's centre from the chamber's: 381 mm toward the window is the end
# of the X travel, 50 mm rearward puts it in line with the window.
EXCHANGE = (0.381, 0.050)
# The vise from the table's centre, toward the window. Not as far as it
# would go: the spindle only reaches 381 mm from the table's centre (X
# home), and a part closer to the table's end than its own half width
# could not be faced.
VISE_OUT = 0.33
# The pedestal: low enough for the elbow to pass under the raised leaf,
# close enough for 911 mm of reach to cover the vise. The leaf's lower
# edge is what decides it: at 350 mm the elbow clears it by 63 mm; at
# 450 mm the forearm meets it and the run through the window is refused.
STAND_H = 0.35
STAND_OUT = 0.12                 # from the `entry` frame (150 mm off the leaf)
WAIT_OUT = 0.12                  # the hand's wait, off the leaf's outer face
STOCKER = (0.60, 0.45, 0.80)     # a bench with the blank and the finished part
STOCKER_AT = (0.08, -0.62)       # its centre from the pedestal's, on the operator's side
SLOT = 0.15                      # the two slots, either side of the bench centre

PART, PART_MASS = one.PART, one.PART_MASS
SEAT, HOVER, GRIP = one.SEAT, one.HOVER, one.GRIP
OPEN, SHUT = one.OPEN, one.SHUT
PAD_DEPTH = one.tooling.PAD_DEPTH

CLAMP_S = 0.8

# The tools, as they stand in the spindle: a 40-taper collet chuck 75 mm
# from gauge line to nose, the cutter so far out of it. The table's
# entries (110 and 135 mm) are taken from these.
HOLDER = bt.nc.Holder(diameter=0.040, length=0.075)
TOOLS = {1: bt.nc.end_mill(0.010, flutes=0.022, stickout=0.035, holder=HOLDER),
         2: bt.nc.drill(0.005, flutes=0.045, stickout=0.060, holder=HOLDER)}
# The part as designed, from the block's top face down (m): faced by
# 0.5 mm, a 20 mm pocket 3 mm deep in the middle, four 5 mm holes on a
# 28 mm square, 7.5 mm deep from the faced top. The program below is one
# way to make it; `design_model` is what it is measured against.
FACE = 0.0005
POCKET = (0.020, 0.003)          # diameter, depth below the faced top
HOLES = (0.005, 0.0075, 0.014)   # diameter, depth below the faced top, offset of each from the centre
TOLERANCE = 5e-5
# Setups that were stood wrong, to see what the cell says: the work zero
# entered 2 mm too high, the drill's table entry 20 mm short of it, and
# the vise bolted 30 mm nearer the window than the spindle can reach.
SETUPS = ("g54_high", "t2_short", "vise_out")
# The relays the program calls the arm with: to swap the parts, and —
# once the vise has closed on the blank — to let go and leave.
CALL, LEAVE = 21, 22
# The part program: the part, then the cell. Work zero (G54) is the centre
# of the part's top face; N220 parks the table for the exchange, in
# machine coordinates filled in from where the cell was built; and M99
# starts it over on the blank the arm has just loaded.
PROGRAM = """%
O00010 (VF-2 CELL - FACE, POCKET, DRILL - THEN THE NEXT PART)
(T1 D=10. - FLAT END MILL)
(T2 D=5. - DRILL)
N10 G90 G94 G17
N15 G21
N20 G53 G0 Z0.

(FACE)
N25 T1 M6
N30 S6000 M3
N35 G54
N40 G0 X-27. Y-16.
N45 G43 Z15. H1
N50 M8
N55 G0 Z2.
N60 G1 Z-0.5 F300.
N65 X27. F900.
N66 Y-8.
N67 X-27.
N70 Y0.
N75 X27.
N80 Y8.
N82 X-27.
N84 Y16.
N85 X27.
N90 G0 Z5.

(POCKET)
N95 G0 X0. Y0.
N100 G1 Z-3.5 F200.
N105 X5. F600.
N110 G3 X5. Y0. I-5. J0.
N115 G1 X0.
N120 G0 Z15.
N125 M5
N130 M9
N135 G53 G0 Z0.

(DRILL)
N140 M1
N145 T2 M6
N150 S4000 M3
N155 G54
N160 G0 X-14. Y-14.
N165 G43 Z15. H2
N170 M8
N175 G98 G83 X-14. Y-14. Z-8. R2. Q3. F150.
N180 X14.
N185 Y14.
N190 X-14.
N195 G80
N200 G0 Z15.
N205 M5
N210 M9
N215 G53 G0 Z0.
N220 G53 G0 X{x:.1f} Y{y:.1f}

(THE NEXT PART)
N300 M71 (VISE OPEN)
N305 M180 (WINDOW OPEN)
N310 M21 (ROBOT - PART OUT, BLANK IN AND HELD)
N315 M70 (VISE CLOSED)
N320 M22 (ROBOT - LET GO AND LEAVE)
N325 M181 (WINDOW CLOSED)
N330 M99
%
"""
# The step the machine's program stops in for good when its M180 finds the
# cell not safe — named, as every step of it is, after the program's line.
NOT_SAFE = "N305 M180 door open ALARM — the cell is not safe or the front door is open"

# Where the arm parks between cycles: turned to the bench, folded over
# its own pedestal — a jog, typed in joints and checked at build.
READY = [math.pi / 2, -0.35, -0.45, 0.0, -1.25, 0.0, OPEN]
# The branches tried for every taught pose: shoulder forward or back,
# elbow more or less bent, wrist either way.
BRANCHES = [[0.0, j2, j3, 0.0, j5, 0.0, OPEN]
            for j2 in (0.4, 0.0, 0.8, -0.4, -1.0) for j3 in (0.0, 0.5, -0.5, 1.0, -1.0)
            for j5 in (-1.2, -0.6, -1.8, 0.6, 1.2)]


# ----------------------------------------------------------------- design
def design_model(path: Path | None = None) -> Path:
    """The finished part as designed, written as a closed mesh (OBJ) in
    the block's own frame — origin at its centre, top face on +Z — and
    returned as the file. It is the drawing: built from `FACE`, `POCKET`
    and `HOLES`, not from the program, so the two can disagree.

    The part is prismatic, so the mesh is too: the top is a 3 x 3 grid of
    cells with a round opening in the middle one and in each corner one,
    and every wall and floor hangs off the same points."""
    half_x, half_y, half_z = (v / 2 for v in PART)
    top = half_z - FACE
    pocket_r, pocket_z = POCKET[0] / 2, top - POCKET[1]
    hole_r, hole_z, hole_at = HOLES[0] / 2, top - HOLES[1], HOLES[2]
    # The grid line between the pocket and a hole: nearer the pocket, so
    # that every corner of a hole's cell sees the hole's rim from outside.
    cut = pocket_r + 0.2 * (hole_at - hole_r - pocket_r)
    vertices: dict[tuple[float, float, float], int] = {}
    faces: list[tuple[int, int, int]] = []

    def face(a, b, c, normal) -> None:
        """One triangle, wound so that it looks along `normal`."""
        u = tuple(q - p for p, q in zip(a, b))
        v = tuple(q - p for p, q in zip(a, c))
        cross = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        if sum(n * x for n, x in zip(normal, cross)) < 0:
            b, c = c, b
        faces.append(tuple(vertices.setdefault(tuple(round(x, 9) for x in p), len(vertices) + 1)
                           for p in (a, b, c)))

    def ring(cx: float, cy: float, r: float, n: int) -> list[tuple[float, float]]:
        return [(cx + r * math.cos(math.tau * k / n), cy + r * math.sin(math.tau * k / n)) for k in range(n)]

    def plate(x0, x1, y0, y1, z, normal, opening=None) -> None:
        """A rectangle at height `z`, less a round `opening` inside it:
        every edge of the opening fans to the corner it faces."""
        corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        if opening is None:
            a, b, c, d = ((x, y, z) for x, y in corners)
            face(a, b, c, normal)
            face(a, c, d, normal)
            return
        (cx, cy), points = opening
        # The corner an edge of the rim faces: the one in the direction of
        # its middle, seen from the opening's centre — the corner that
        # sees the edge from outside.
        bearings = [math.atan2(y - cy, x - cx) for x, y in corners]

        def turn(a: float, b: float) -> float:
            return abs((a - b + math.pi) % math.tau - math.pi)

        count = len(points)
        owner = []
        for k, (px, py) in enumerate(points):
            qx, qy = points[(k + 1) % count]
            middle = math.atan2((py + qy) / 2 - cy, (px + qx) / 2 - cx)
            owner.append(min(range(4), key=lambda i: turn(bearings[i], middle)))
        for k, (px, py) in enumerate(points):
            qx, qy = points[(k + 1) % count]
            face((*corners[owner[k]], z), (px, py, z), (qx, qy, z), normal)
            # Where the next edge faces the next corner, the two corners
            # and the rim point between the edges close the gap.
            following = owner[(k + 1) % count]
            if following != owner[k]:
                face((*corners[owner[k]], z), (*corners[following], z), (qx, qy, z), normal)

    def bore(points, floor: float) -> None:
        """The wall of an opening down to `floor`, and the floor."""
        cx = sum(p[0] for p in points) / len(points)
        cy = sum(p[1] for p in points) / len(points)
        for k, (px, py) in enumerate(points):
            qx, qy = points[(k + 1) % len(points)]
            inward = (cx - (px + qx) / 2, cy - (py + qy) / 2, 0.0)
            face((px, py, top), (qx, qy, top), (qx, qy, floor), inward)
            face((px, py, top), (qx, qy, floor), (px, py, floor), inward)
            face((cx, cy, floor), (px, py, floor), (qx, qy, floor), (0.0, 0.0, 1.0))

    xs = (-half_x, -cut, cut, half_x)
    ys = (-half_y, -cut, cut, half_y)
    openings = {(1, 1): ((0.0, 0.0), ring(0.0, 0.0, pocket_r, 96))}
    for i, sx in ((0, -1.0), (2, 1.0)):
        for j, sy in ((0, -1.0), (2, 1.0)):
            centre = (sx * hole_at, sy * hole_at)
            openings[(i, j)] = (centre, ring(*centre, hole_r, 48))
    for i in range(3):
        for j in range(3):
            plate(xs[i], xs[i + 1], ys[j], ys[j + 1], top, (0.0, 0.0, 1.0), openings.get((i, j)))
            plate(xs[i], xs[i + 1], ys[j], ys[j + 1], -half_z, (0.0, 0.0, -1.0))
    for (i, j), (_, points) in openings.items():
        bore(points, pocket_z if (i, j) == (1, 1) else hole_z)
    for k in range(3):
        for y, out in ((-half_y, -1.0), (half_y, 1.0)):
            a, b = (xs[k], y, -half_z), (xs[k + 1], y, -half_z)
            c, d = (xs[k + 1], y, top), (xs[k], y, top)
            face(a, b, c, (0.0, out, 0.0))
            face(a, c, d, (0.0, out, 0.0))
        for x, out in ((-half_x, -1.0), (half_x, 1.0)):
            a, b = (x, ys[k], -half_z), (x, ys[k + 1], -half_z)
            c, d = (x, ys[k + 1], top), (x, ys[k], top)
            face(a, b, c, (out, 0.0, 0.0))
            face(a, c, d, (out, 0.0, 0.0))

    path = path or Path(tempfile.gettempdir()) / "botrail-vf2-demo" / "wp40_design.obj"
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f"v {x:.9f} {y:.9f} {z:.9f}" for x, y, z in vertices]
    lines += [f"f {a} {b} {c}" for a, b, c in faces]
    path.write_text("\n".join(lines) + "\n")
    return path


# ------------------------------------------------------------------ build
def local_pack(pack: str, catalog_root: str | None = None) -> Path | None:
    """`pack` out of a local catalog build, where one is named
    (`catalog_root`, or `BOTRAIL_CATALOG_BUILD`) and holds it."""
    configured = catalog_root or os.environ.get("BOTRAIL_CATALOG_BUILD")
    if configured:
        local = Path(configured).expanduser() / pack
        if (local / "manifest.yaml").is_file():
            return local
    return None


def machine_pack(catalog_root: str | None = None):
    """The VF-2's pack: out of a local catalog build where there is one,
    the published catalog otherwise."""
    return local_pack(MACHINE, catalog_root) or MACHINE


def machine_shape(catalog_root: str | None = None) -> Path | str | None:
    """The VF-2's picture: out of a local catalog build where there is one
    (its directory), the published catalog otherwise (its id) — and `None`
    where neither has it (no network, or a catalog from before the model
    was published): the cell is then drawn as the envelopes it is verified
    against."""
    local = local_pack(MACHINE_SHAPE, catalog_root)
    if local is not None:
        return local
    try:
        bt.catalog_package(MACHINE_SHAPE)
    except (ValueError, OSError, ImportError):
        return None
    return MACHINE_SHAPE


def tool() -> bt.Robot:
    """CR-7iA/L with the Hand-E on its flange."""
    return bt.Robot.from_catalog(ARM).attach_tool(bt.Robot.from_catalog(HAND), prefix="hand_")


def build(*, catalog_root: str | None = None, setup: str | None = None) -> tuple[bt.Scene, bt.tending.Handshake]:
    """The cell, taught and programmed. `setup` stands it wrong in one of
    the `SETUPS` ways; `None` is the setup as it should be."""
    if setup is not None and setup not in SETUPS:
        raise ValueError(f"setup is one of {SETUPS}, not {setup!r}")
    scene = bt.Scene(tool(), name=ROBOT)

    # -- the machine, the vise, the parts ----------------------------------
    # The pack carries the envelope — the lifting window, where it stands
    # along the wall, the open top — and the axes: travels, home, rates.
    # What is written here is where the program parks the table.
    # The maker's geometry is the picture over that where it has been built,
    # and changes nothing that is checked.
    vf2 = bt.parts.machine_tool(scene, "vf2", catalog=machine_pack(catalog_root), exchange=EXCHANGE,
                                axes=True, shape=machine_shape(catalog_root))
    (tx, ty, tz), _ = scene.frame("vf2/table")
    # (Or 30 mm nearer the window, where the arm would reach it better —
    # and the spindle would not reach its far side at all.)
    out = VISE_OUT + (0.030 if setup == "vise_out" else 0.0)
    vise = bt.parts.vise(scene, "vise", (tx + out, ty, tz), opening=PART[1] + 0.004,
                         catalog=VISE, jaw_width=0.125)
    # Bolted to the table: the vise goes where the table goes.
    for piece in vise.obstacles:
        scene.attach(piece, link=vf2.table_link, touch_links=[vf2.table_link], robot=vf2.axes)
    (jx, jy, jz), _ = scene.frame(vise.frames[0])
    # The block this cycle machines: raw in the vise, the part when the
    # arm takes it out.
    scene.add_box("part", size=PART, position=(jx, jy, jz + SEAT + PART[2] / 2), color=one.BLANK)
    scene.set_part("part", kind="obstacle", category="workpiece", model="WP-40", mass_kg=PART_MASS)

    # -- the setup: where the work zero is, the tools, the program -----------
    # G54 is the centre of the block's top face as it stands in the vise —
    # derived from the mechanism, not typed in. (Or typed in wrong: an
    # edge finder read 2 mm above the face.)
    zero = bt.nc.set_work_offset(scene, vf2, "G54", at=(jx, jy, jz + SEAT + PART[2]))
    if setup == "g54_high":
        bt.nc.set_work_offset(scene, vf2, "G54", xyz=(zero[0], zero[1], zero[2] + 0.002))
    # The tools go in as bodies, and the table's entries are taken from
    # them. (Or not: the drill entered 20 mm shorter than it is.)
    for number, body in TOOLS.items():
        entered = body.length - 0.020 if (setup, number) == ("t2_short", 2) else None
        bt.nc.set_tool(scene, vf2, number, body, length=entered)
    bt.nc.program(scene, "op10", PROGRAM.format(x=vf2.park[0] * 1e3, y=vf2.park[1] * 1e3))

    # -- the pedestal and the stocker, at the window -------------------------
    (ex, ey, _), _ = scene.frame("vf2/entry")
    stand = bt.parts.pedestal(scene, "stand", height=STAND_H, position=(ex + STAND_OUT, ey), yaw=math.pi,
                              model="RS-450", manufacturer="ACME", mass_kg=38)
    (mx, my, mz), mq = scene.frame(stand.frames[0])
    scene.set_robot_base_pose((mx, my, mz + 0.005), mq, robot=ROBOT)
    # Bolted down: the base and the plate it stands on are one contact
    # the clearance measure must not report.
    scene.allow_link_obstacle_contact(scene.robot_of(ROBOT).link_names[0], "stand/top", robot=ROBOT)
    bench = bt.parts.table(scene, "stocker", size=STOCKER, position=(mx + STOCKER_AT[0], my + STOCKER_AT[1]),
                           model="WB-600", manufacturer="ACME", mass_kg=32)
    (bx, by, bz), _ = scene.frame(bench.frames[0])
    scene.add_box("blank", size=PART, position=(bx - SLOT, by, bz + SEAT + PART[2] / 2), color=one.BLANK)
    scene.set_part("blank", kind="obstacle", category="workpiece", model="WP-40-raw", mass_kg=PART_MASS)
    scene.add_frame("stocker/out", position=(bx + SLOT, by, bz))
    scene.add_frame("stocker/blank", position=(bx - SLOT, by, bz))
    # The pads touch the part they close on — declared, not found.
    for part in ("part", "blank"):
        for link in one.tooling.pads(scene.robot_of(ROBOT)):
            scene.allow_link_obstacle_contact(link, part, robot=ROBOT)

    # -- the machine's program: the part program's own, read off its text ----
    # The blocks the control runs, and its M codes answered: the window on
    # the cell-safe input, the vise, the two relays that wait for M-FIN.
    # The vise takes what stands in it — the part now, the blank next.
    hs = bt.tending.haas_mfin(scene, vf2, "op10", parts=["part", "blank"], clamp_s=CLAMP_S,
                              relays={CALL: "arm: part out, blank in", LEAVE: "arm: let go and leave"})

    teach(scene, vf2)
    program(scene, hs)
    # The fault the FAT is about: the cell-safe wire broken. The program
    # alarms at its M180 and the arm waits for a call that never comes.
    scene.add_scenario("no_cell_safe", faults=[bt.io.stuck(hs.signal("cell_safe"), False)])
    return scene, hs


def fat_cases(hs: bt.tending.Handshake) -> list:
    """The FAT row of `no_cell_safe` as a test case: with the cell-safe
    input low the machine's program must stop on the alarm at its `M180`
    — the window never commanded — rather than open it."""
    return [bt.vc.TestCase("cell not safe", faults=[bt.io.stuck(hs.signal("cell_safe"), False)],
                           expect=[bt.vc.stalls(hs.program, at=NOT_SAFE)],
                           notes="the cell-safe input low: M180 alarms, the window stays shut")]


# ------------------------------------------------------------------ teach
def lift_window(scene: bt.Scene, vf2: bt.parts.MachineTool, home: dict, fraction: float) -> None:
    """Puts the leaf (and what rides on it) at `fraction` of its stroke —
    raised for teaching the poses inside, down again before the bake."""
    ax, ay, az = vf2.door_axis
    d = fraction * vf2.door_travel
    for name in vf2.door_objects:
        (x, y, z), q = scene.obstacle_pose(name)
        x0, y0, z0 = home.setdefault(name, (x, y, z))
        scene.set_obstacle_pose(name, (x0 + ax * d, y0 + ay * d, z0 + az * d), q)


def teach(scene: bt.Scene, vf2: bt.parts.MachineTool, *, vise: str = "vise", stocker: str = "stocker") -> None:
    """Every pose the arm works from, solved against its own kinematics
    from the cell's frames — nothing but the park pose typed in joints.
    The poses inside are taught with the window raised: an arm through a
    closed leaf is a collision, not a pose."""
    limits = scene.robot_of(ROBOT).joint_limits
    name = vf2.name

    def unwind(q: list, seed: list) -> list:
        """Each joint the short way round from `seed` — an IK hands back
        an angle, not a winding, and this arm's last axis offers a full
        turn to lose between two taught points."""
        out = []
        for value, want, limit in zip(q, seed, limits):
            lo, hi = limit or (-math.inf, math.inf)
            best = value
            for turn in (-1, 1):
                other = value + turn * 2 * math.pi
                if lo - 1e-9 <= other <= hi + 1e-9 and abs(other - want) < abs(best - want):
                    best = other
            out.append(best)
        return out

    def solve(target, quat, *seeds: list, bearing: float = 0.0) -> list:
        """The tool at `target`, from the seeds given and then every
        branch turned to `bearing` — refused only when all of them foul
        or fall short."""
        short, fouled = None, []
        for seed in (*seeds, *([bearing, *b[1:]] for b in BRANCHES)):
            seed = list(seed[:6]) + [OPEN]
            scene.set_joint_positions(seed, robot=ROBOT)
            ik = scene.set_tcp_target(target, quat, robot=ROBOT)
            if not ik.converged:
                short = ik.pos_error
                continue
            q = unwind(list(scene.joint_positions_of(ROBOT)), seed)
            q[-1] = OPEN
            scene.set_joint_positions(q, robot=ROBOT)
            hits = [f"{a[1]} x {b[1]}" for a, b in scene.check_collisions()]
            if not hits:
                return q
            fouled = hits
        where = tuple(round(v, 3) for v in target)
        if short is not None and not fouled:
            raise RuntimeError(f"{ARM} cannot reach {where}: {short * 1e3:.0f} mm short")
        raise RuntimeError(f"every branch at {where} fouls: {', '.join(fouled)}")

    def joint(motion: str, q: list) -> list:
        scene.add_segment(motion, goal=q, robot=ROBOT)
        return q

    def line(motion: str, q: list) -> list:
        scene.add_segment(motion, goal=q, kind="cartesian_line", robot=ROBOT)
        return q

    # Parked over the pedestal, and waiting at the window — both with the
    # leaf down, which is how the arm meets them.
    q_part = one.down(0.0)
    ready = list(READY)
    scene.set_joint_positions(ready, robot=ROBOT)
    if hits := [f"{a[1]} x {b[1]}" for a, b in scene.check_collisions()]:
        raise RuntimeError(f"the park pose fouls: {', '.join(hits)}")
    scene.add_segment("home", goal=ready, robot=ROBOT)
    (jx, jy, jz), _ = scene.frame(f"{vise}/jaw")
    grasp_z = jz + SEAT + PART[2] - GRIP - PAD_DEPTH
    (_, ey, _), _ = scene.frame(f"{name}/entry")
    leaf_out = max(scene.obstacle_bounds(f"{name}/side_door/leaf")[1][0],
                   scene.obstacle_bounds(f"{name}/side_door/leaf")[0][0])
    wait = solve((leaf_out + WAIT_OUT, ey, grasp_z + HOVER), q_part)
    joint("approach", wait)
    # Between the window and the bench the arm turns folded as it waits:
    # swung at full stretch, the hand would pass the wall's front pier.
    turned = [math.pi / 2, *wait[1:]]
    scene.set_joint_positions(turned, robot=ROBOT)
    if hits := [f"{a[1]} x {b[1]}" for a, b in scene.check_collisions()]:
        raise RuntimeError(f"the folded turn fouls: {', '.join(hits)}")
    joint("turn", turned)
    # The straight run through the window, leaf raised: over the finished
    # part, pads across its free faces (the jaws hold the other two), then
    # down onto it.
    home: dict = {}
    lift_window(scene, vf2, home, 1.0)
    over = solve((jx, jy, grasp_z + HOVER), q_part, wait)
    grasp = solve((jx, jy, grasp_z), q_part, over)
    line("enter", over)
    line("down", grasp)
    line("up", over)
    line("exit", wait)
    lift_window(scene, vf2, home, 0.0)

    # The stocker: the finished part's slot and the blank's, each a frame
    # on the bench top, the arm turned a quarter to face them.
    for tag in ("out", "blank"):
        (sx, sy, sz), _ = scene.frame(f"{stocker}/{tag}")
        set_z = sz + SEAT + PART[2] - GRIP - PAD_DEPTH
        hi = solve((sx, sy, set_z + HOVER), q_part, ready, bearing=math.pi / 2)
        lo = solve((sx, sy, set_z), q_part, hi, bearing=math.pi / 2)
        joint(f"to_{tag}", hi)
        line(f"set_{tag}" if tag == "out" else "down_blank", lo)
        line(f"clear_{tag}" if tag == "out" else "up_blank", hi)

    scene.set_joint_positions(ready, robot=ROBOT)


# ---------------------------------------------------------------- program
def program(scene: bt.Scene, hs: bt.tending.Handshake, *, parts: tuple[str, str] = ("part", "blank")) -> None:
    """The robot's program: say the cell is safe, wait at the window for
    the machine's call (`M21`), swap the parts through the raised window
    and answer with the blank still in hand; let go once the machine
    calls again (`M22` — the vise has closed), and answer that one only
    with the arm out: the machine's next line closes the window.

    A relay is answered on levels, both ways: M-FIN goes up when the work
    is done and down once the relay has dropped, so the next call cannot
    take the last answer for its own."""
    S = bt.seq
    finger = scene.robot_of(ROBOT).joint_names[-1]
    sq = scene.sequence("tend")
    finished, blank = parts
    call, leave, fin = hs.signal(f"m{CALL}"), hs.signal(f"m{LEAVE}"), hs.signal("m_fin")

    def grip(value: float):
        return S.ramp({finger: value}, 0.4, robot=ROBOT)

    def run(step: str, motion: str | None = None) -> None:
        sq.step(step, actions=[S.motion(motion or step)])

    # Safe for as long as this program runs — and on before the machine
    # gets to its M180, which alarms without it.
    sq.step("approach", actions=[S.set_signal(hs.signal("cell_safe")), S.motion("approach")])
    sq.step("wait_call", transition=S.signal(call))
    # Finished part out.
    run("enter")
    run("down")
    sq.step("grip", actions=[grip(SHUT)])
    sq.step("hold", actions=[S.attach(finished, touch_links="tool", robot=ROBOT)])
    run("up")
    run("exit")
    run("turn")
    run("to_out")
    run("set_out")
    sq.step("release_out", actions=[S.detach(finished), grip(OPEN)])
    run("clear_out")
    # Blank in.
    run("to_blank")
    run("down_blank")
    sq.step("grip_blank", actions=[grip(SHUT)])
    sq.step("hold_blank", actions=[S.attach(blank, touch_links="tool", robot=ROBOT)])
    run("up_blank")
    run("turn_2", "turn")
    run("approach_2", "approach")
    run("enter_2", "enter")
    run("load", "down")
    # The blank is in the jaws, in hand: M-FIN. The machine closes the
    # vise on it (M70) and calls again once the jaws have stroked.
    sq.step("loaded", actions=[S.set_signal(fin)], transition=S.signal(call, False))
    sq.step("wait_clamp", actions=[S.set_signal(fin, False)], transition=S.signal(leave))
    sq.step("release", actions=[S.detach(blank), grip(OPEN)])
    run("up_2", "up")
    run("exit_2", "exit")
    # Out of the enclosure: this M-FIN is what lets the window close.
    sq.step("out", actions=[S.set_signal(fin)], transition=S.signal(leave, False))
    sq.step("clear", actions=[S.set_signal(fin, False)], transition=S.signal(hs.signal("door_closed")))
    run("home")


# ------------------------------------------------------------------- bake
def bake(*, catalog_root: str | None = None, setup: str | None = None):
    scene, hs = build(catalog_root=catalog_root, setup=setup)
    tl = scene.simulate_sequences(["tend", hs.program], max_duration=240.0)
    return scene, hs, tl


def fixtures(scene: bt.Scene) -> list[str]:
    """What a tool must not touch: the vise, every piece of it."""
    return [name for name in scene.obstacle_names if name.startswith("vise/")]


def dry_run(scene: bt.Scene) -> bt.nc.Run:
    """The part program on the machine alone, before anything else in the
    cell moves: timed, cut, and measured against the design."""
    return bt.nc.check(scene, "vf2/axes", "op10", stock="part", fixtures=fixtures(scene),
                       design=design_model(), tolerance=TOLERANCE)


def cut(scene: bt.Scene, tl):
    """The baked cycle with the part cut in it: the timeline to play, and
    what was cut — the block where the bake had it, on the table that
    moved, out through the window in the gripper."""
    return bt.nc.animate(scene, tl, "vf2/axes", "part", fixtures=fixtures(scene),
                         design=design_model(), tolerance=TOLERANCE)


# ------------------------------------------------------------- hand-over
def deliver(scene: bt.Scene, hs: bt.tending.Handshake, tl, out: Path, sheet: Path):
    """The document set, written into `out` from the one source, and the
    report that hashes it: the project, the part program, the bill, the
    I/O and the handshake, both programs as PLCopen, the interlock table,
    the drawing, the scenario matrix and the FAT sheet — with the setup
    sheet `sheet`, already written. Returns `(report, runs)`.

    The machine's rows in all of them are read off the part program: its
    blocks and its M codes are the steps."""
    out.mkdir(parents=True, exist_ok=True)
    files: list[Path] = [sheet]

    def write(name: str, fn) -> Path:
        path = out / name
        fn(path)
        files.append(path)
        return path

    write("vf2_tending.botrail", scene.save_project)
    write("vf2_tending_op10.nc", lambda p: p.write_text(scene.nc_program("op10")))
    write("vf2_tending_bom.csv", scene.export_bom)
    write("vf2_tending_bom.md", scene.export_bom)
    write("vf2_tending_io.csv", scene.export_io_list)
    write("vf2_tending_handshake.md", tl.export_handshake_spec)
    write("vf2_tending.plcopen.xml", lambda p: scene.export_plcopen(p, name="VF-2 tending"))
    write("vf2_tending_interlocks.md", scene.export_interlocks)
    write("vf2_tending_interlocks.csv", scene.export_interlocks)
    write("vf2_tending_layout.svg", lambda p: scene.export_layout(p, scale=120, title="VF-2 tending"))
    # The scenario matrix: the baseline and the broken wire. An alarm
    # never clears, so that run goes to the cap and says where it stands.
    runs = scene.simulate_scenarios(["tend", hs.program], max_duration=tl.duration + 20.0)
    # The FAT sheet: the arm's cycle (the machine's never ends), the broken
    # wire as a test case, and a row per row of the interlock table.
    cases = [bt.vc.TestCase("nominal cycle", expect=[bt.vc.cycle_within(tl.duration + 1.0, program="tend")]),
             *fat_cases(hs), *bt.vc.interlock_tests(scene, hs.program)]
    fat = bt.vc.run_fat(scene, cases, ["tend", hs.program], title="VF-2 tending FAT")
    write("vf2_tending_fat.md", fat.save)
    write("vf2_tending_fat.csv", fat.save)
    report = scene.cell_report({"baseline": tl}, scenarios=runs, deliverables=files,
                               title="VF-2 tending", sections=[fat.section()])
    report.save(out / "vf2_tending_report.md")
    report.save(out / "vf2_tending_report.json")
    return report, runs


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("out", nargs="?", default=str(HERE / "vf2_tending.usdc"))
    parser.add_argument("--catalog-root", default=None,
                        help=f"a local catalog build to read the machine's pack ({MACHINE}) and "
                             f"its picture ({MACHINE_SHAPE}) from, instead of the published catalog")
    parser.add_argument("--setup", choices=SETUPS, default=None,
                        help="stand the setup wrong: the work zero 2 mm high, the drill's entry 20 mm short, "
                             "or the vise 30 mm too near the window")
    parser.add_argument("--studio", action="store_true")
    args = parser.parse_args()

    try:
        scene, hs = build(catalog_root=args.catalog_root, setup=args.setup)
    except ValueError as refusal:
        if args.setup is None:
            raise
        # A setup the control cannot run is said as the cell is built.
        print(f"refused: {refusal}")
        return
    # The dry run comes first, as it does at the machine: the program
    # alone, timed, cut and measured — and the setup sheet that goes with
    # it. What it says of a setup stood wrong is the point of running it.
    run = dry_run(scene)
    print(f"  {run.summary()}")
    print("  " + run.machining.summary().replace("\n", "\n  "))
    sheet = Path(args.out).with_name(Path(args.out).stem + "_setup.md")
    sheet.write_text(bt.nc.setup_sheet(
        scene, "vf2/axes", "op10", stock="part", fixtures=fixtures(scene), design=design_model(),
        tolerance=TOLERANCE, title="VF-2 — WP-40, OP10"))
    print(f"wrote {sheet}")

    tl = scene.simulate_sequences(["tend", hs.program], max_duration=240.0)
    print(f"{hs.template}: cycle {tl.duration:.2f}s")
    offsets = scene.nc_machine("vf2/axes")["work_offsets"]
    print("  G54 = " + ", ".join(f"{axis}{v * 1e3:.3f}" for axis, v in zip("XYZ", offsets["G54"])))
    for name, t0, t1 in tl.step_spans:
        print(f"  {name:<28} {t0:7.2f} - {t1:7.2f}s")
    for role in ("door_closed", "door_open", "clamp", "cell_safe", f"m{CALL}", f"m{LEAVE}", "m_fin",
                 "spindle", "coolant"):
        spans = ", ".join(f"{a:.2f}-{b:.2f}" for a, b in tl.signal(hs.signal(role)).high_spans())
        print(f"  {hs.signal(role):<28} on: {spans or '-'}")
    clearance = tl.min_clearance()
    pair = f" ({clearance.pair[0]} x {clearance.pair[1]})" if clearance.pair else ""
    print(f"min clearance over the cycle: {float(clearance) * 1e3:.1f} mm at {clearance.t:.2f}s{pair}")

    # The documents are the cell's, before anything is added for show.
    print(scene.bom().to_markdown())
    print(scene.interlocks([hs.program]).to_markdown())
    out_dir = Path(args.out).with_name("vf2_tending_deliverables")
    report, runs = deliver(scene, hs, tl, out_dir, sheet)
    for name in ("baseline", *scene.scenario_names):
        verdict = runs.errors.get(name)
        print(f"  scenario {name:<20} {'ok' if verdict is None else 'refused — ' + verdict}")
    fat = next(s for s in report.sections if s["title"] == "VF-2 tending FAT")["json"]
    for row in fat["cases"]:
        print(f"  FAT {row['name']:<40} {row['verdict']:<5} "
              f"{row['detail'] or '; '.join(c['detail'] for c in row['checks'])}")
    print(f"wrote the document set to {out_dir}/ ({len(report.deliverables)} files hashed in the report)")
    # The same cycle with the part cut in it — where the bake had the
    # block, which is where the arm will have put the next one.
    shown, machined = cut(scene, tl)
    print("  " + machined.summary().replace("\n", "\n  "))
    warnings = shown.export_usd(args.out, fps=60)
    print(f"wrote {args.out}" + (f" ({warnings})" if warnings else ""))
    if args.studio:
        scene.show_timeline(shown)
        bt.studio(scene)


if __name__ == "__main__":
    main()

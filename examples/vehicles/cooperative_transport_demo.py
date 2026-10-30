"""Cooperative transport: two KUKA KMP 3000P carry one machine bed as a formation.

A 5.0 m machine-tool bed (4.3 t) stands on its transport stand in the assembly
bay at the west end of the main aisle. Two KMP 3000P — the omnidirectional
3-ton platforms KUKA pairs for loads up to 6 t, one leading — leave their
parking slots south of the aisle, crab in under the stand, close the 20 mm to
its beams, take the load and lift it 80 mm clear of the floor. As one machine
(`scene.add_formation`) they carry it east down the aisle, turn it a quarter
about its own centre at the crossing with the north aisle, carry it into the
inspection bay, set it down and drive out to their stand-by slots. A parts AMR
(MiR250) crosses the main aisle on the line's call while all this goes on.

Every number comes from the catalog or is derived: the platforms' size, deck
height, lift stroke and speed are the KMP 3000P package's; where each one
stands under the bed is its slot in the formation, and where it is set down
is `formation_slot` — so its own path picks up where the formation left it.
The coupled speed is not published: 0.3 m/s is an assumption, stated.

What the bake answers:

  * **Does the bed turn the corner?** Turning on the spot, the stand's legs
    sweep a 2.95 m circle about the bed's centre; the four columns of the
    crossing stand `aisle / √2` from it. The aisle check runs every platform,
    the stand and the bed against the columns on every tick the formation
    moves: `--aisle 4.0` is refused, by name — a column, at a time.
  * **What does the choreography cost?** Drive-in, touch, lift, carry, turn,
    lower and drive-out, each a step on the timeline — and the formation can
    go no faster than its slowest member, its turn no faster than its
    outermost member allows (checked when the cell is simulated).
  * **Do two platforms carry it?** Each platform's share of the bed and its
    stand, from where its slot holds them and where their centre of mass is
    (`bt.select.requirements`: the lever between the two), against the KMP
    3000P's 3 t — and the pair against the 2 units / 6 t KUKA states for
    load sharing. `--cog 0.6` sets the casting 0.6 m off the stand's centre:
    the leading platform's share passes 3 t, refused by name.
  * **Can the parts AMR share the aisle?** Each machine requests the main
    aisle and drives on a grant; a small arbiter hands it to one at a time,
    the formation keeping it until it reaches the corner. `--no-interlock`
    drops the arbiter: the MiR250 meets the bed at a named time and place.

Run with:  python examples/vehicles/cooperative_transport_demo.py [recording.usdc] [--studio]
                        [--out DIR] [--aisle M] [--cog M] [--no-interlock] [--catalog-root DIR]

`--out DIR` writes the document set (layout SVG/DXF, BOM, I/O list, interlock
table, PLCopen, report, USD recording). `--catalog-root` points at a catalog
builder's `build/` directory instead of the published dataset.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import botrail as bt

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from _workshop_visuals import box as deco

KMP = "kuka/kmp/kmp-3000p"
MIR = "mobile_industrial_robots/mir/mir250"
CATALOG_ROOT: Path | None = None   # set by --catalog-root: a builder's build/ directory

# ---- the hall ---------------------------------------------------------------
AISLE = 5.0                       # clear width between the column faces, both aisles
COLUMN = 0.5
HALL = (-4.0, 23.0, -8.0, 16.5)   # x0, x1, y0, y1
ASSEMBLY = (3.5, 0.0)             # the bed's centre on its stand
CORNER = (14.0, 0.0)              # the crossing with the north aisle
INSPECTION = (14.0, 10.0)
CONCRETE = (0.32, 0.33, 0.32)
LINE_YELLOW = (0.85, 0.62, 0.05)
LINE_GREEN = (0.10, 0.45, 0.20)

# ---- the load ---------------------------------------------------------------
BED = (5.0, 1.6)                  # the casting's footprint
BED_MASS, STAND_MASS = 4300.0, 260.0
LEG, BEAM = 0.12, (0.16, 0.12)    # stand leg square, beam width x height
LEG_X, LEG_Y = 2.74, 0.84         # leg centres
BEAM_Y = 0.45                     # the two beams the platforms lift by
GAP = 0.020                       # beams over the lowered decks
SLOT = 1.45                       # each platform's centre ahead of / behind the bed's

# ---- the motion -------------------------------------------------------------
COUPLED_SPEED = 0.30              # m/s — the pair's coupled speed is not published (assumed)
TURN = 0.15                       # rad/s about the bed's centre
CALL_AT = 20.0                    # the line calls for parts this long into the cycle


def ref(pid: str):
    """A package reference: the id, or — with --catalog-root — the built
    package directory, the newest revision built there."""
    if not CATALOG_ROOT:
        return pid
    path = CATALOG_ROOT / pid
    revisions = sorted((p for p in path.glob("r*") if p.name[1:].isdigit()), key=lambda p: int(p.name[1:]))
    return revisions[-1] if revisions and not (path / "manifest.yaml").is_file() else path


def load(pid: str) -> bt.Robot:
    if CATALOG_ROOT:
        return bt.Robot.from_package(ref(pid), catalog_root=CATALOG_ROOT)
    return bt.Robot.from_catalog(pid)


def wheels(pid: str) -> bt.Wheels:
    return bt.Wheels.from_catalog(ref(pid))


def offset(aisle: float) -> float:
    """Column centre off an aisle's centre line."""
    return aisle / 2 + COLUMN / 2


# ================================================================== the hall
def hall(scene: bt.Scene, aisle: float) -> list[tuple[float, float]]:
    """Slab, column grid and floor paint. Returns the column centres."""
    x0, x1, y0, y1 = HALL
    slab = scene.add_box("floor/slab", (x1 - x0, y1 - y0, 0.05), ((x0 + x1) / 2, (y0 + y1) / 2, -0.022),
                         color=CONCRETE)
    scene.set_obstacle_enabled(slab, False)
    scene.set_obstacle_material(slab, metalness=0.0, roughness=0.9)
    c = offset(aisle)
    cx, _ = CORNER
    columns = [(0.0, -c), (0.0, c), (7.0, -c), (7.0, c),
               (cx - c, -c), (cx + c, -c), (cx - c, c), (cx + c, c),
               (cx - c, 7.0), (cx + c, 7.0), (cx - c, 14.0), (cx + c, 14.0)]
    for i, (x, y) in enumerate(columns):
        scene.add_box(f"column/{i}", (COLUMN, COLUMN, 6.0), (x, y, 3.0), color=(0.62, 0.62, 0.60))
    scene.set_part("column", kind="group", category="structure.column", qty=len(columns),
                   model="RC column 500x500", manufacturer="Building")
    # Aisle edges and the two bays.
    half = aisle / 2
    for name, line in (("main_s", ((x0 + 0.5, -half), (cx - half, -half))),
                       ("main_n", ((x0 + 0.5, half), (cx - half, half))),
                       ("corner_s", ((cx - half, -half), (cx + half, -half))),
                       ("north_w", ((cx - half, half), (cx - half, y1 - 0.5))),
                       ("north_e", ((cx + half, -half), (cx + half, y1 - 0.5)))):
        bt.parts.marking(scene, f"marking/{name}", line=line, width=0.10, color=LINE_YELLOW, floor=0.003)
    ax, ay = ASSEMBLY
    bt.parts.marking(scene, "marking/assembly", rect=(ax - 3.1, ay - 1.15, ax + 3.1, ay + 1.15), color=LINE_GREEN,
                     floor=0.003)
    ix, iy = INSPECTION
    bt.parts.marking(scene, "marking/inspection", rect=(ix - 1.15, iy - 3.1, ix + 1.15, iy + 3.1), color=LINE_GREEN,
                     floor=0.003)
    return columns


# ================================================================== the load
def bed_on_stand(scene: bt.Scene, deck: float, cog: float = 0.0) -> list[str]:
    """The transport stand and the machine bed on it, at the assembly bay:
    two beams GAP over the platforms' lowered decks, on four legs, the casting
    on the beams — `cog` metres along them off the stand's centre. Returns
    every piece (what the formation carries)."""
    ax, ay = ASSEMBLY
    bw, bh = BEAM
    under = deck + GAP                      # the beams' underside
    top = under + bh                        # the stand's top: the casting sits here
    pieces = []
    steel = (0.20, 0.22, 0.24)
    for sy in (-1, 1):
        pieces.append(scene.add_box(f"stand/beam_{'l' if sy > 0 else 'r'}", (2 * LEG_X + LEG, bw, bh),
                                    (ax, ay + sy * BEAM_Y, under + bh / 2), color=steel))
    for sx in (-1, 1):
        pieces.append(scene.add_box(f"stand/cross_{'f' if sx > 0 else 'r'}", (LEG, 2 * LEG_Y + LEG, bh),
                                    (ax + sx * LEG_X, ay, under + bh / 2), color=steel))
        for sy in (-1, 1):
            tag = f"{'f' if sx > 0 else 'r'}{'l' if sy > 0 else 'r'}"
            pieces.append(scene.add_box(f"stand/leg_{tag}", (LEG, LEG, under), (ax + sx * LEG_X, ay + sy * LEG_Y,
                                                                               under / 2), color=steel))
    scene.set_part("stand", kind="group", category="fixture", model="搬送架台 5600x1800 (KMP 2 台用)",
                   manufacturer="Generic", mass_kg=STAND_MASS)
    # The casting: a ribbed base, the column bed over it, two guideways on top.
    paint = (0.33, 0.40, 0.38)
    length, width = BED
    bx = ax + cog
    pieces.append(scene.add_box("bed/base", (length, width, 0.14), (bx, ay, top + 0.07), color=paint))
    pieces.append(scene.add_box("bed/body", (length - 0.3, width - 0.35, 0.42), (bx, ay, top + 0.14 + 0.21),
                                color=paint))
    for sy in (-1, 1):
        rail = scene.add_box(f"bed/guideway_{'l' if sy > 0 else 'r'}", (length - 0.5, 0.09, 0.05),
                             (bx, ay + sy * 0.42, top + 0.56 + 0.025), color=(0.72, 0.74, 0.75))
        scene.set_obstacle_material(rail, metalness=0.8, roughness=0.3)
        pieces.append(rail)
    scene.set_part("bed", kind="group", category="workpiece", model="工作機械ベッド 5000x1600 (鋳物)",
                   manufacturer="Generic", mass_kg=BED_MASS)
    return pieces


# ================================================================== the machines
def platforms(scene: bt.Scene) -> dict[str, str]:
    """The formation and its two KMP 3000P. Each platform is a vehicle of
    its own with the catalog machine mounted as its running gear; the
    formation's path is the bed's. Returns vehicle -> robot."""
    kmp, gear = load(KMP), wheels(KMP)
    slots = {"kmp1": (SLOT, 0.0), "kmp2": (-SLOT, 0.0)}
    scene.add_formation("carrier", members=slots, path=[ASSEMBLY, CORNER, INSPECTION],
                        stations={"assembly": 0, "corner": 1, "inspection": 2},
                        speed=COUPLED_SPEED, turn_speed=TURN, start="assembly")
    units = {}
    for name in slots:
        (ux, uy, _), _ = scene.formation_slot("carrier", name, "assembly")
        (tx, ty, _), _ = scene.formation_slot("carrier", name, "inspection")
        # Parked south of the aisle; the first hop points +x, so the machine
        # (holonomic: it never turns) faces the bed's long axis, then crabs
        # north in under the stand. Set down in the inspection bay, it crabs
        # east out between the stand's legs to its stand-by slot.
        path = [(ux - 0.8, -5.2), (ux, -5.2), (ux, uy), (tx, ty), (tx + 3.2, ty), (tx + 5.4, ty)]
        scene.add_vehicle(name, body=[], path=path,
                          stations={"park": 0, "under": 2, "there": 3, "standby": 5},
                          speed=1.0, turn_speed=0.5, drive="holonomic", start="park")
        unit = f"{name}_unit"
        scene.add_robot(kmp, name=unit)
        scene.mount_robot(name, robot=unit, wheels=gear)
        units[name] = unit
    return units


def parts_amr(scene: bt.Scene) -> None:
    """The MiR250 that brings a tote from the store north of the aisle to the
    line south of it — straight across the main aisle."""
    x = 9.0
    scene.add_robot(load(MIR), name="mir_unit")
    deck = scene.link_pose("deck", robot="mir_unit")[0][2]
    scene.add_box("mir/tote", (0.6, 0.4, 0.28), (x, 6.0, deck + 0.142), color=(0.15, 0.32, 0.62))
    scene.set_part("mir/tote", category="container", model="通い箱 600x400", manufacturer="Generic")
    # The tote rides the deck (the tray zone), so the machine is the vehicle: one BOM line.
    scene.add_vehicle("mir", body=[], path=[(x, 6.0), (x, -6.0)], stations={"store": 0, "line": 1},
                      speed=1.0, turn_speed=1.0, start="store",
                      tray_position=(0.0, 0.0, deck + 0.142), tray_size=(0.7, 0.5, 0.3))
    scene.mount_robot("mir", robot="mir_unit")


def dressing(scene: bt.Scene) -> None:
    """Context outside the aisles: the people at the bays, out of collision."""
    ax, ay = ASSEMBLY
    ix, iy = INSPECTION
    for name, at, yaw in (("assembler", (ax - 1.0, ay + 3.4), -math.pi / 2),
                          ("inspector", (ix - 3.4, iy + 1.2), 0.0)):
        bt.parts.person(scene, f"people/{name}", at, yaw=yaw, detail="full")
    # The stand-by area east of the inspection bay, and the parking slots south of the aisle.
    bt.parts.marking(scene, "marking/standby", rect=(ix + 4.1, iy - 3.1, ix + 6.7, iy + 3.1), color=LINE_GREEN,
                     floor=0.003)
    bt.parts.marking(scene, "marking/park", rect=(ax - 3.6, -6.0, ax + 3.6, -4.4), color=LINE_GREEN, floor=0.003)
    # The parts store the MiR250 loads at: a steel shelving unit with totes on it.
    sx, sy = 9.0, 7.7
    post, shelf = (0.20, 0.24, 0.27), (0.36, 0.40, 0.44)
    for i, (px, py) in enumerate(((-0.97, -0.27), (0.97, -0.27), (-0.97, 0.27), (0.97, 0.27))):
        deco(scene, f"store/post{i}", (0.04, 0.04, 1.8), (sx + px, sy + py, 0.9), color=post, metalness=0.5)
    for k, z in enumerate((0.15, 0.75, 1.35)):
        deco(scene, f"store/shelf{k}", (2.0, 0.6, 0.03), (sx, sy, z), color=shelf, metalness=0.4)
        for j in range(3):
            deco(scene, f"store/tote{k}_{j}", (0.6, 0.4, 0.28), (sx - 0.65 + 0.65 * j, sy, z + 0.155),
                 color=(0.15, 0.32, 0.62))
    scene.set_part("store", kind="group", category="structure.rack", model="中量棚 1800x600x1800",
                   manufacturer="Generic")
    bt.parts.table(scene, "line/bench", (2.0, 0.8, 0.85), (9.0, -7.4), detail="full")


# ================================================================== the programs
def programs(scene: bt.Scene, units: dict[str, str], load_pieces: list[str], interlock: bool) -> list[str]:
    for sig in ("req_carrier", "grant_carrier", "req_mir", "grant_mir"):
        scene.define_signal(sig, initial=(not interlock) and sig.startswith("grant"))
    every = list(units.values())
    lift_speed = 0.03                                   # the package's lift: 30 mm/s

    def lift(sq, name: str, to: float, frm: float) -> None:
        sq.step(name, actions=[bt.seq.ramp({"lift": to}, abs(to - frm) / lift_speed, robot=u) for u in every])

    def all_done(names):
        return bt.seq.all_of(*[bt.seq.device_done(n) for n in names])

    sq = scene.sequence("carry")
    sq.step("drive_in", actions=[bt.seq.goto(v, "under") for v in units], transition=all_done(units))
    lift(sq, "touch", GAP, 0.0)
    # The leading platform holds the load (its deck); the other bears it.
    sq.step("take", actions=[bt.seq.attach(p, link="deck", robot=units["kmp1"]) for p in load_pieces],
            transition=bt.seq.immediately())
    lift(sq, "lift", 0.100, GAP)
    sq.step("request", actions=[bt.seq.set_signal("req_carrier")], transition=bt.seq.signal("grant_carrier"))
    sq.step("to_corner", actions=[bt.seq.goto("carrier", "corner")], transition=bt.seq.device_done("carrier"))
    # Past the crossing: the aisle goes back to the traffic control.
    sq.step("clear", actions=[bt.seq.set_signal("req_carrier", False)], transition=bt.seq.elapsed(0.05))
    sq.step("to_inspection", actions=[bt.seq.goto("carrier", "inspection")],
            transition=bt.seq.device_done("carrier"))
    # Down until the legs stand on the floor, let go, then the decks clear the beams.
    lift(sq, "lower", GAP, 0.100)
    sq.step("release", actions=[bt.seq.detach(p) for p in load_pieces], transition=bt.seq.immediately())
    lift(sq, "clear_beams", 0.0, GAP)
    sq.step("drive_out", actions=[bt.seq.goto(v, "standby") for v in units], transition=all_done(units))

    pt = scene.sequence("parts")
    pt.step("wait_call", transition=bt.seq.elapsed(CALL_AT))
    pt.step("request", actions=[bt.seq.set_signal("req_mir")], transition=bt.seq.signal("grant_mir"))
    pt.step("cross", actions=[bt.seq.goto("mir", "line")], transition=bt.seq.device_done("mir"))
    pt.step("clear", actions=[bt.seq.set_signal("req_mir", False)], transition=bt.seq.elapsed(0.05))

    names = ["carry", "parts"]
    if interlock:
        # One machine in the main aisle at a time: each round grants the
        # first request (the formation when both ask in one scan) and holds
        # until it drops. One round per aisle use authored above.
        tc = scene.sequence("traffic")
        for k in range(2):
            sel = tc.select(f"round{k}")
            for name in ("carrier", "mir"):
                arm = sel.when(bt.seq.signal(f"req_{name}"))
                arm.step(f"grant{k}_{name}", actions=[bt.seq.set_signal(f"grant_{name}")],
                         transition=bt.seq.signal(f"req_{name}", False))
                arm.step(f"release{k}_{name}", actions=[bt.seq.set_signal(f"grant_{name}", False)],
                         transition=bt.seq.immediately())
        names.append("traffic")
    return names


# ================================================================== the cell
def build(*, aisle: float = AISLE, interlock: bool = True, cog: float = 0.0) -> tuple[bt.Scene, list[str]]:
    scene = bt.Scene()
    hall(scene, aisle)
    units = platforms(scene)
    deck = scene.link_pose("deck", robot=units["kmp1"])[0][2]
    pieces = bed_on_stand(scene, deck, cog)
    parts_amr(scene)
    dressing(scene)
    names = programs(scene, units, pieces, interlock)
    return scene, names


def bake(*, aisle: float = AISLE, interlock: bool = True):
    scene, names = build(aisle=aisle, interlock=interlock)
    return scene, scene.simulate_sequences(names, max_duration=600.0)


# ================================================================== reports
def sweep_radius() -> float:
    """How far the formation's farthest point — a stand leg's outer corner —
    stands from the bed's centre: the circle the quarter turn sweeps."""
    return math.hypot(LEG_X + LEG / 2, LEG_Y + LEG / 2)


def load_share(scene: bt.Scene, timeline=None) -> tuple[bt.select.Requirements, bt.select.Row]:
    """What the cell asks of the platforms' line — each one's share of the
    load, and the pair's — compared with the KMP 3000P's catalog specs."""
    req = bt.select.requirements(scene, timeline=timeline)
    return req, req["kmp1_unit"]


def deliver(scene: bt.Scene, tl, out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    files = []

    def write(name, fn):
        path = out / name
        fn(path)
        files.append(path)

    title = "大物の協調搬送 (KMP 3000P × 2)"
    write("cooperative_layout.svg", lambda p: scene.export_layout(p, scale=40, title=title))
    write("cooperative_layout.dxf", lambda p: scene.export_layout(p, title="cooperative transport"))
    write("cooperative_bom.csv", scene.export_bom)
    write("cooperative_bom.md", scene.export_bom)
    write("cooperative_io.csv", scene.export_io_list)
    write("cooperative_interlocks.md", scene.export_interlocks)
    write("cooperative.plcopen.xml", lambda p: scene.export_plcopen(p, name="cooperative"))
    write("cooperative_cell.usdc", lambda p: tl.export_usd(p, fps=15.0))
    report = scene.cell_report(tl, deliverables=files, title=title)
    report.save(out / "cooperative_report.md")
    report.save(out / "cooperative_report.json")
    print(f"wrote the document set to {out}/ ({len(files) + 2} files)")


def main() -> None:
    global CATALOG_ROOT
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recording", nargs="?", default="cooperative_cell.usdc", help="write the baked cycle as USD here")
    parser.add_argument("--aisle", type=float, default=AISLE, help="clear aisle width between column faces, metres")
    parser.add_argument("--cog", type=float, default=0.0,
                        help="set the casting this far (m) off the stand's centre: the platforms' load shares")
    parser.add_argument("--no-interlock", dest="no_interlock", action="store_true",
                        help="drop the traffic control: the parts AMR meets the bed")
    parser.add_argument("--out", type=Path, default=None, help="write the document set here")
    parser.add_argument("--catalog-root", type=Path, default=None, help="a catalog builder's build/ directory")
    parser.add_argument("--studio", action="store_true")
    args = parser.parse_args()
    if args.catalog_root:
        CATALOG_ROOT = args.catalog_root.resolve()

    radius = sweep_radius()
    if args.cog:
        # A selection question, not a bake: what each platform bears.
        scene, _ = build(cog=args.cog)
        req, row = load_share(scene)
        payload = next(r for r in row.requirements if r.key == "payload_kg")
        print(f"the casting {args.cog:+.2f} m off the stand's centre: {payload.basis}")
        if req.ok:
            print(f"the KMP 3000P's {payload.provided:.0f} kg carries it")
            return
        print("refused, as it should be:")
        for finding in req.findings():
            if finding.severity == "error":
                print(f"  {finding.message}")
        return
    if args.no_interlock or args.aisle != AISLE:
        print(f"the quarter turn sweeps {radius:.2f} m about the bed's centre; the crossing's columns stand "
              f"{args.aisle / math.sqrt(2):.2f} m from it")
        try:
            bake(aisle=args.aisle, interlock=not args.no_interlock)
        except ValueError as err:
            print("refused, as it should be:")
            print(f"  {err}")
            return
        if args.no_interlock:
            raise SystemExit("the parts AMR crossing a carried bed without traffic control should have been refused")
        print(f"a {args.aisle:.2f} m aisle passes")
        return

    scene, tl = bake()
    print(f"KMP 3000P x 2 under a {BED[0]:.1f} x {BED[1]:.1f} m bed ({BED_MASS + STAND_MASS:.0f} kg with its stand), "
          f"slots ±{SLOT:.2f} m; coupled {COUPLED_SPEED:.2f} m/s (assumed), turn {TURN:.2f} rad/s")
    print(f"the quarter turn sweeps {radius:.2f} m; the crossing's columns stand {AISLE / math.sqrt(2):.2f} m "
          f"from the bed's centre — the corner needs an aisle of {radius * math.sqrt(2):.2f} m")
    _, row = load_share(scene, tl)
    for r in row.requirements:
        if r.key.startswith(("payload", "cooperative")):
            print(f"  {r} ({r.basis}) — the KMP 3000P states {r.provided:g}: {r.status}")
    print(f"cycle {tl.duration:.1f} s")
    for name in ("drive_in", "touch", "lift", "request", "to_corner", "to_inspection", "lower", "drive_out"):
        span = tl.step_span(f"carry/{name}")
        print(f"  {name:14s} {span.start:6.1f} – {span.end:6.1f} s  ({span.duration:.1f} s)")
    wait = tl.step_span("parts/request")
    cross = tl.step_span("parts/cross")
    print(f"  parts AMR: called {CALL_AT:.0f} s, waited {wait.duration:.1f} s for the aisle, "
          f"crossed {cross.start:.1f} – {cross.end:.1f} s")
    tl.export_usd(args.recording, fps=15)
    print(f"wrote {args.recording}")
    if args.out:
        deliver(scene, tl, args.out)
    if args.studio:
        bt.studio(scene, view=((2.0, -12.0, 11.0), (10.0, 3.0, 0.0)))


if __name__ == "__main__":
    main()
